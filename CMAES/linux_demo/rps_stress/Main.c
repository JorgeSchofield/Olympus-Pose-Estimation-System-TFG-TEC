// OE3 indicator test: rock-paper-scissors at 50 Hz, run until a target number
// of messages has been processed (default 45 000), with every round going
// through AMS suspend and resume of both players.
//
// Pass criteria (exit code 0):
//   - every message arrives once, in order, with intact content (per-player
//     sequence numbers and a checksum in each payload: PE-RF-007 multi-buffer
//     integrity);
//   - no send failure, no AMS refusal, no player that fails to suspend;
//   - process RSS does not grow after warm-up (memory leak check).
//
// Usage: cmaes_rps_stress_demo [messages=45000] [period_ms=20] [cpu=-1]
// Run as root so SCHED_FIFO applies, e.g.:  cmaes_rps_stress_demo 45000 20 3

#include <CMAES.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define N_BUF 3                     // rotation required by a depth-1 mailbox
#define WARMUP_MSGS 1000            // RSS baseline taken after this many messages
#define REPORT_EVERY 5000
#define SEND_TIMEOUT_MS 100
#define RECV_TIMEOUT_MS 1000
#define SUSPEND_TIMEOUT_MS 500
#define CHECK_MAGIC 0xC3A5F00Du

typedef struct {
	uint32_t seq;
	uint32_t choice;                // 0 rock, 1 paper, 2 scissors
	uint32_t check;
} throw_msg_t;

typedef struct {
	throw_msg_t buf[N_BUF];
	unsigned next;
	uint32_t seq;
	uint32_t rng;
	MAESTickType_t last_wake;
	uint32_t send_failures;
	uint32_t overruns;
} player_state_t;

typedef struct {
	uint32_t received;
	uint32_t rounds;
	uint32_t last_seq[2];
	uint32_t gaps, duplicates, bad_checksums;
	uint32_t ams_confirms, ams_errors, unexpected;
	uint32_t suspend_timeouts, receive_timeouts;
	uint32_t wins[3];               // draw, A, B
	long rss_baseline_kb, rss_max_kb;
	MAESTickType_t t_start;
} referee_state_t;

MAESAgent AgentA, AgentB, Referee;
Agent_Platform Platform;
sysVars env;
CyclicBehaviour playBehaviourA, playBehaviourB, refereeBehaviour;
Agent_Msg msg_playA, msg_playB, msg_referee;

static uint32_t target_msgs = 45000;
static uint32_t period_ms = 20;
static int pin_cpu = -1;
static player_state_t stateA, stateB;
static referee_state_t ref;

static uint32_t checksum(const throw_msg_t* m) {
	return m->seq ^ (m->choice * 0x9E3779B1u) ^ CHECK_MAGIC;
}

static long rss_kb(void) {
	long pages_total = 0, pages_resident = 0;
	FILE* f = fopen("/proc/self/statm", "r");
	if (f == NULL) {
		return -1;
	}
	if (fscanf(f, "%ld %ld", &pages_total, &pages_resident) != 2) {
		pages_resident = -1;
	}
	fclose(f);
	return pages_resident < 0 ? -1 : pages_resident * (sysconf(_SC_PAGESIZE) / 1024);
}

// ---------------------------------------------------------------- players

static player_state_t* player_of(CyclicBehaviour* Behaviour) {
	return Behaviour == &playBehaviourA ? &stateA : &stateB;
}

void playSetup(CyclicBehaviour* Behaviour, void* pvParameters) {
	(void)pvParameters;
	player_state_t* st = player_of(Behaviour);
	Behaviour->msg->Agent_Msg(Behaviour->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), pin_cpu);
	st->rng = (st == &stateA) ? 12345u : 67890u;
	st->last_wake = MAES_GetTickCount();
}

void playAction(CyclicBehaviour* Behaviour, void* pvParameters) {
	(void)pvParameters;
	player_state_t* st = player_of(Behaviour);

	if (!MAES_DelayUntil(&st->last_wake, period_ms)) {
		st->overruns++;               // late (also right after each resume)
	}

	st->rng = st->rng * 1103515245u + 12345u;
	throw_msg_t* m = &st->buf[st->next];
	m->seq = st->seq + 1;
	m->choice = (st->rng >> 16) % 3;
	m->check = checksum(m);

	Behaviour->msg->set_msg_type(Behaviour->msg, INFORM);
	Behaviour->msg->set_msg_content(Behaviour->msg, (char*)m);
	if (Behaviour->msg->send(Behaviour->msg, Referee.AID(&Referee), SEND_TIMEOUT_MS) == NO_ERRORS) {
		st->seq = m->seq;
		st->next = (st->next + 1) % N_BUF;   // advance only after a successful send
	}
	else {
		st->send_failures++;
	}
}

void playA(void* pvParameters) {
	playBehaviourA.msg = &msg_playA;
	playBehaviourA.setup = &playSetup;
	playBehaviourA.action = &playAction;
	playBehaviourA.execute(&playBehaviourA, pvParameters);
}

void playB(void* pvParameters) {
	playBehaviourB.msg = &msg_playB;
	playBehaviourB.setup = &playSetup;
	playBehaviourB.action = &playAction;
	playBehaviourB.execute(&playBehaviourB, pvParameters);
}

// ---------------------------------------------------------------- referee

static void finish(void) {
	MAESTickType_t elapsed = MAES_GetTickCount() - ref.t_start;
	long rss_end = rss_kb();
	long growth = (ref.rss_baseline_kb >= 0 && rss_end >= 0) ? rss_end - ref.rss_baseline_kb : 0;
	uint32_t failures = ref.gaps + ref.duplicates + ref.bad_checksums + ref.ams_errors +
		ref.unexpected + ref.suspend_timeouts + ref.receive_timeouts +
		stateA.send_failures + stateB.send_failures + (growth > 0 ? 1u : 0u);

	printf("\n===== CMAES rps_stress summary =====\n");
	printf("messages processed : %" PRIu32 " (target %" PRIu32 ")\n", ref.received, target_msgs);
	printf("rounds             : %" PRIu32 " (draw %" PRIu32 ", A %" PRIu32 ", B %" PRIu32 ")\n",
		ref.rounds, ref.wins[0], ref.wins[1], ref.wins[2]);
	printf("suspend/resume     : %" PRIu32 " each, AMS confirms %" PRIu32 "\n",
		2 * ref.rounds, ref.ams_confirms);
	printf("elapsed            : %.1f s, %.1f msg/s\n", elapsed / 1000.0,
		elapsed ? ref.received * 1000.0 / elapsed : 0.0);
	printf("sequence gaps      : %" PRIu32 "\n", ref.gaps);
	printf("duplicates         : %" PRIu32 "\n", ref.duplicates);
	printf("bad checksums      : %" PRIu32 "\n", ref.bad_checksums);
	printf("send failures      : A %" PRIu32 ", B %" PRIu32 "\n", stateA.send_failures, stateB.send_failures);
	printf("AMS errors         : %" PRIu32 ", unexpected msgs %" PRIu32 "\n", ref.ams_errors, ref.unexpected);
	printf("suspend timeouts   : %" PRIu32 ", receive timeouts %" PRIu32 "\n", ref.suspend_timeouts, ref.receive_timeouts);
	printf("period overruns    : A %" PRIu32 ", B %" PRIu32 " (expected ~1 per round: after resume)\n",
		stateA.overruns, stateB.overruns);
	printf("RSS                : baseline %ld kB, max %ld kB, end %ld kB, growth %ld kB\n",
		ref.rss_baseline_kb, ref.rss_max_kb, rss_end, growth);
	printf("RESULT             : %s\n", failures == 0 ? "PASS" : "FAIL");
	fflush(stdout);
	exit(failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE);
}

static void account_throw(int player, const throw_msg_t* m) {
	if (checksum(m) != m->check) {
		ref.bad_checksums++;
	}
	if (m->seq == ref.last_seq[player] + 1) {
		// in order
	}
	else if (m->seq <= ref.last_seq[player]) {
		ref.duplicates++;             // a reused buffer would show up here
	}
	else {
		ref.gaps += m->seq - ref.last_seq[player] - 1;
	}
	ref.last_seq[player] = m->seq;
	ref.received++;

	if (ref.received == WARMUP_MSGS) {
		ref.rss_baseline_kb = rss_kb();
	}
	if (ref.received % REPORT_EVERY == 0) {
		long rss = rss_kb();
		if (rss > ref.rss_max_kb) {
			ref.rss_max_kb = rss;
		}
		printf("%7" PRIu32 " msgs, %6" PRIu32 " rounds, RSS %ld kB, failures so far %" PRIu32 "\n",
			ref.received, ref.rounds, rss,
			ref.gaps + ref.duplicates + ref.bad_checksums + ref.suspend_timeouts);
	}
}

void refereeSetup(CyclicBehaviour* Behaviour, void* pvParameters) {
	(void)pvParameters;
	Behaviour->msg->Agent_Msg(Behaviour->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), pin_cpu);
	ref.rss_baseline_kb = -1;
	ref.t_start = MAES_GetTickCount();
}

// One action = one round: collect a throw from each player (suspending each
// one through the AMS as its throw arrives), decide, then resume both.
void refereeAction(CyclicBehaviour* Behaviour, void* pvParameters) {
	(void)pvParameters;
	Agent_Msg* msg = Behaviour->msg;
	Agent_AID aidA = AgentA.AID(&AgentA);
	Agent_AID aidB = AgentB.AID(&AgentB);
	bool have[2] = { false, false };
	uint32_t choice[2] = { 0, 0 };

	while (!(have[0] && have[1])) {
		MSG_TYPE type = msg->receive(msg, RECV_TIMEOUT_MS);
		if (type == NO_RESPONSE) {
			ref.receive_timeouts++;
			if (ref.receive_timeouts > 5) {
				finish();
			}
			continue;
		}
		if (type == CONFIRM) {
			ref.ams_confirms++;       // reply to an earlier suspend/resume
			continue;
		}
		if (type == REFUSE || type == NOT_UNDERSTOOD) {
			ref.ams_errors++;
			continue;
		}
		if (type != INFORM) {
			ref.unexpected++;
			continue;
		}

		Agent_AID sender = msg->get_sender(msg);
		int player = pthread_equal(sender, aidA) ? 0 : (pthread_equal(sender, aidB) ? 1 : -1);
		if (player < 0) {
			ref.unexpected++;
			continue;
		}
		throw_msg_t copy = *(const throw_msg_t*)msg->get_msg_content(msg);
		account_throw(player, &copy);
		if (!have[player]) {
			have[player] = true;
			choice[player] = copy.choice % 3;
			msg->suspend(msg, sender);
		}
		// A second throw from the same player in one round is a normal race:
		// the suspension only takes effect at the player's next checkpoint.
		// It is still counted and checked, just not played.

		if (ref.received >= target_msgs) {
			finish();
		}
	}

	MAESTickType_t t0 = MAES_GetTickCount();
	while (Platform.get_state(&Platform, aidA) != SUSPENDED ||
		Platform.get_state(&Platform, aidB) != SUSPENDED) {
		if (MAES_GetTickCount() - t0 > SUSPEND_TIMEOUT_MS) {
			ref.suspend_timeouts++;
			break;
		}
		Platform.agent_wait(&Platform, 1);
	}

	static const int winner[3][3] = { {0, 2, 1}, {1, 0, 2}, {2, 1, 0} };
	ref.wins[winner[choice[0]][choice[1]]]++;
	ref.rounds++;

	msg->resume(msg, aidA);
	msg->resume(msg, aidB);
}

void referee(void* pvParameters) {
	refereeBehaviour.msg = &msg_referee;
	refereeBehaviour.setup = &refereeSetup;
	refereeBehaviour.action = &refereeAction;
	refereeBehaviour.execute(&refereeBehaviour, pvParameters);
}

// ---------------------------------------------------------------- main

int main(int argc, char** argv) {
	setvbuf(stdout, NULL, _IOLBF, 0);
	if (argc > 1) {
		target_msgs = (uint32_t)strtoul(argv[1], NULL, 10);
	}
	if (argc > 2) {
		period_ms = (uint32_t)strtoul(argv[2], NULL, 10);
	}
	if (argc > 3) {
		pin_cpu = atoi(argv[3]);
	}
	if (target_msgs == 0 || period_ms == 0) {
		fprintf(stderr, "usage: %s [messages=45000] [period_ms=20] [cpu=-1]\n", argv[0]);
		return EXIT_FAILURE;
	}
	printf("------ CMAES rps_stress: %" PRIu32 " messages, %" PRIu32 " ms period per player, cpu %d ------\n",
		target_msgs, period_ms, pin_cpu);

	ConstructorAgente(&AgentA);
	ConstructorAgente(&AgentB);
	ConstructorAgente(&Referee);
	ConstructorSysVars(&env);
	ConstructorAgent_Platform(&Platform, &env);
	ConstructorAgent_Msg(&msg_playA, &env);
	ConstructorAgent_Msg(&msg_playB, &env);
	ConstructorAgent_Msg(&msg_referee, &env);
	ConstructorCyclicBehaviour(&playBehaviourA);
	ConstructorCyclicBehaviour(&playBehaviourB);
	ConstructorCyclicBehaviour(&refereeBehaviour);

	AgentA.Iniciador(&AgentA, "Player A", 1, 0);
	AgentB.Iniciador(&AgentB, "Player B", 1, 0);
	Referee.Iniciador(&Referee, "Referee", 2, 0);
	Platform.Agent_Platform(&Platform, "RPS_stress_platform");

	Platform.agent_init(&Platform, &AgentA, &playA);
	Platform.agent_init(&Platform, &AgentB, &playB);
	Platform.agent_init(&Platform, &Referee, &referee);
	Platform.boot(&Platform);

	// The referee ends the process with exit() once the target is reached.
	pthread_join(Platform.description.AMS_AID, NULL);
	return EXIT_FAILURE;
}
