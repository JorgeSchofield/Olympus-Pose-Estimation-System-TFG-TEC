// Port of libCMAES_FreeRTOS/src/rock_paper_scissors.c onto the pthreads
// backend. Exercises more of the API than the sender/receiver smoke test:
// OneShotBehaviour, multi-receiver broadcast (send0), suspend/resume routed
// through the AMS, and get_state - all useful groundwork for the Pose
// Estimation agents, which will suspend/resume subsystem agents the same
// way. See docs/README.md for the clear_all_receiver bug this game
// exposed and fixed in the library itself.

#include <CMAES.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

MAESAgent AgentA, AgentB, Referee;
Agent_Platform Platform;
sysVars env;
CyclicBehaviour playBehaviourA, playBehaviourB;
OneShotBehaviour watchoverBehaviour;
Agent_Msg msg_playA, msg_playB, msg_watchover;

//Random Number Generator Function: This function generates a random number between 0 and 2.
//Inputs: None.
//Outputs: Random number between 0 and 2.
int getRandom(void) {
	return rand() % 3;
};

//Msg to int Function: This function translates a specific msg to a certain numeric value.
//Inputs: A message that represents the player's choice.
//Outputs: A number that represents the player's choice.
int choices(char* msg) {
	if (strcmp(msg, "ROCK") == 0)
	{
		return 0;
	}
	else if (strcmp(msg, "PAPER") == 0)
	{
		return 1;
	}
	else
	{
		return 2; //scissors
	}
};

//Functions related to the play Behaviour

void playSetup(CyclicBehaviour* Behaviour, void* pvParameters) {
	Behaviour->msg->Agent_Msg(Behaviour->msg);
	Behaviour->msg->add_receiver(Behaviour->msg, Referee.AID(&Referee));
};

void playAction(CyclicBehaviour* Behaviour, void* pvParameters) {
	Agent_info informacion = Platform.get_Agent_description(Platform.get_running_agent(&Platform));
	printf("%s: Rock, Paper, Scissors... \n", informacion.agent_name);
	Platform.agent_wait(&Platform, 10);
	int num = getRandom();
	char* bet = "";
	switch (num) {
	case 0:
		bet = "ROCK";
		break;

	case 1:
		bet = "PAPER";
		break;

	case 2:
		bet = "SCISSORS";
		break;

	default:
		break;
	}
	Behaviour->msg->set_msg_content(Behaviour->msg, bet);
	Behaviour->msg->set_msg_type(Behaviour->msg, INFORM);
	Behaviour->msg->send0(Behaviour->msg);
};

//Wrappers: Since two different agents are going to use the play Behaviour, a wrapper function must be created for each one of the agents. Doing this, two different instances of the play Behaviour will be created and
//          there will not be an error regarding overwriting values from the other agent.

//PlayA Wrapper:
void playA(void* pvParameters) {
	playBehaviourA.msg = &msg_playA;
	playBehaviourA.setup = &playSetup;
	playBehaviourA.action = &playAction;
	for (;;) {
		playBehaviourA.execute(&playBehaviourA, &pvParameters);
	}
};

//PlayB Wrapper:
void playB(void* pvParameters) {
	playBehaviourB.msg = &msg_playB;
	playBehaviourB.setup = &playSetup;
	playBehaviourB.action = &playAction;
	for (;;) {
		playBehaviourB.execute(&playBehaviourB, &pvParameters);
	}
};

//Functions related to the Watchover Behaviour

void watchoverSetup(OneShotBehaviour* Behaviour, void* pvParameters) {
	Behaviour->msg->Agent_Msg(Behaviour->msg);
	Behaviour->msg->add_receiver(Behaviour->msg, AgentA.AID(&AgentA));
	Behaviour->msg->add_receiver(Behaviour->msg, AgentB.AID(&AgentB));
};

void watchoverAction(OneShotBehaviour* Behaviour, void* pvParameters) {
	char* msgA = "";
	char* msgB = "";
	int choiceA = 0;
	int choiceB = 0;
	int winner[3][3] = {
		{0, 2, 1},
		{1, 0, 2},
		{2, 1, 0}
	};
	while (true)
	{
		Behaviour->msg->receive(Behaviour->msg, MAES_MAX_DELAY);
		if (Behaviour->msg->get_msg_type(Behaviour->msg) == INFORM)
		{
			Agent_info PlayerInfo = Platform.get_Agent_description(Behaviour->msg->get_sender(Behaviour->msg));
			printf("%s: %s\n", PlayerInfo.agent_name, Behaviour->msg->get_msg_content(Behaviour->msg));
			if (pthread_equal(Behaviour->msg->get_sender(Behaviour->msg), AgentA.AID(&AgentA)))
			{
				msgA = Behaviour->msg->get_msg_content(Behaviour->msg);
				choiceA = choices(msgA);
			}
			else if (pthread_equal(Behaviour->msg->get_sender(Behaviour->msg), AgentB.AID(&AgentB)))
			{
				msgB = Behaviour->msg->get_msg_content(Behaviour->msg);
				choiceB = choices(msgB);
			}
			Behaviour->msg->suspend(Behaviour->msg, Behaviour->msg->get_sender(Behaviour->msg));
		}
		if (Platform.get_state(&Platform, AgentA.AID(&AgentA)) == SUSPENDED && Platform.get_state(&Platform, AgentB.AID(&AgentB)) == SUSPENDED) {
			break;
		}
	}
	Agent_info RefereeInfo = Platform.get_Agent_description(Platform.get_running_agent(&Platform));
	switch (winner[choiceA][choiceB])
	{
	case 0:
		printf("\n%s: DRAW!\n", RefereeInfo.agent_name);
		break;

	case 1:
		printf("\n%s: PLAYER A WINS!\n", RefereeInfo.agent_name);
		break;

	case 2:
		printf("\n%s: PLAYER B WINS!\n", RefereeInfo.agent_name);
		break;

	default:
		break;
	}
	Platform.agent_wait(&Platform, 2000);
	Behaviour->msg->resume(Behaviour->msg, AgentA.AID(&AgentA));
	Behaviour->msg->resume(Behaviour->msg, AgentB.AID(&AgentB));
	printf("\n-------------PLAYING AGAIN---------------\n");
};

//Watchover Wrapper:
void watchover(void* pvParameters) {
	watchoverBehaviour.msg = &msg_watchover;
	watchoverBehaviour.setup = &watchoverSetup;
	watchoverBehaviour.action = &watchoverAction;
	for (;;) {
		watchoverBehaviour.execute(&watchoverBehaviour, &pvParameters);
	}
};

//Main
int main(void) {
	setvbuf(stdout, NULL, _IOLBF, 0);
	printf("------Rock Paper Scissors APP (pthreads backend)------ \n");

	//Constructors for each initialized class
	ConstructorAgente(&AgentA);
	ConstructorAgente(&AgentB);
	ConstructorAgente(&Referee);
	ConstructorSysVars(&env);
	ConstructorAgent_Platform(&Platform, &env);
	ConstructorAgent_Msg(&msg_playA, &env);
	ConstructorAgent_Msg(&msg_playB, &env);
	ConstructorAgent_Msg(&msg_watchover, &env);
	ConstructorCyclicBehaviour(&playBehaviourA);
	ConstructorCyclicBehaviour(&playBehaviourB);
	ConstructorOneShotBehaviour(&watchoverBehaviour);

	//Initializing the Agents and the Platform.
	AgentA.Iniciador(&AgentA, "Player A", 1, 20);
	AgentB.Iniciador(&AgentB, "Player B", 1, 20);
	Referee.Iniciador(&Referee, "Referee", 2, 20);
	Platform.Agent_Platform(&Platform, "RPS_platform");

	//Registering the Agents and their respective behaviour into the Platform
	Platform.agent_init(&Platform, &AgentA, &playA);
	Platform.agent_init(&Platform, &AgentB, &playB);
	Platform.agent_init(&Platform, &Referee, &watchover);
	Platform.boot(&Platform);
	printf("CMAES booted successfully \n");
	printf("Initiating APP\n\n");

	// See linux_demo/sender_receiver/Main.c: block the main thread on the
	// AMS instead of vTaskStartScheduler()/for(;;), since agent threads are
	// already running once boot() returns.
	pthread_join(Platform.description.AMS_AID, NULL);

	return 0;
};
