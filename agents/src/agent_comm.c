/*
 * agent_comm.c - Communication agent (DESIGN.md §7.4, §11).
 *
 *   - publishes every pose_decimation-th estimate as a UDP datagram (PE-RF-012);
 *   - logs every estimate and every GPS fix with HLC timestamps (PE-RF-013);
 *   - serves the local control port (MARK, RESET_POSE, SUSPEND, RESUME, STATS);
 *   - once per second writes the validation statistics: per-link delivery and
 *     worst gap (PE-RNF-003), latency p50/p95/max (PE-RNF-002), pose interval
 *     (PE-RNF-001), LLC clock scale (§10.4), RSS (PE-RNF-005).
 *
 * The log files are written by the asynchronous writer (port_log.c), so a slow
 * SD card never blocks this agent.
 */
#include "app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pose_dgram.h"
#include "port.h"
#include "stats.h"

static CyclicBehaviour beh;
static Agent_Msg       msg;
static pe_udp_dest_t   dest1, dest2;
static int             have_dest2;
static int             ctrl_fd = -1;
static uint32_t        dgram_seq, decim, n_pose, n_gps, n_gps_valid, last_pose_seq, last_gps_seq;
static pe_link_stats_t link_est, link_gps;
static pe_hist_t       lat_hist, int_hist;
static pe_clockfit_t   clockfit;
static uint64_t        last_pub_ns, last_stats_ns, t0_ns;
static pe_pose_msg_t   last_pose;
static pe_ctrl_msg_t   ctrl_pool[PE_POOL_DEPTH];
static unsigned        ctrl_next;
static uint32_t        ctrl_seq;
static double          last_llc_reset_t_cum = -1.0;

static void handle_pose(const pe_pose_msg_t* p)
{
	uint64_t now = pe_now_ns();
	double lat_ms = (double)(now - p->hdr.t_rx_ns) * 1e-6;
	uint8_t d[PE_POSE_DGRAM_LEN];

	pe_link_account(&link_est, &last_pose_seq, p->hdr.seq);
	link_est.producer_drops = p->producer_drops;
	n_pose++;
	pe_hist_add(&lat_hist, lat_ms);
	if (last_pub_ns != 0) {
		pe_hist_add(&int_hist, (double)(now - last_pub_ns) * 1e-6);
	}
	last_pub_ns = now;
	if ((p->status & PE_ST_LLC_RESET) && p->t_cum_ms != last_llc_reset_t_cum) {
		pe_clockfit_reset(&clockfit);
		last_llc_reset_t_cum = p->t_cum_ms;
	}
	if (!(p->status & PE_ST_STALE)) {
		pe_clockfit_add(&clockfit, p->t_llc_ms, p->hdr.t_rx_ns);
	}

	if (++decim >= (uint32_t)g_params.pose_decimation) {
		decim = 0;
		pe_pose_dgram_encode(p, ++dgram_seq, now, d);
		pe_udp_send(&dest1, d, sizeof(d));
		if (have_dest2) {
			pe_udp_send(&dest2, d, sizeof(d));
		}
	}

	pe_log_printf(g_log_pose,
		"%llu,%llu,%llu,%u,%.3f,%u,%u,"
		"%.6f,%.6f,%.6f,%.6f,%.7f,"
		"%.4e,%.4e,%.4e,%.4e,%.4e,%.4e,"
		"%.6f,%.6f,%.3f,%.3f,%.3f,"
		"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
		(unsigned long long)now, (unsigned long long)p->hdr.t_rx_ns, (unsigned long long)p->t_est_ns,
		p->t_llc_ms, p->t_cum_ms, p->mode, p->status,
		p->x[0], p->x[1], p->x[2], p->x[3], p->x[4],
		p->P[0], p->P[6], p->P[1], p->P[12], p->P[18], p->P[24],
		p->s_m, p->diag[0], p->diag[1], p->diag[2], lat_ms,
		p->link_acq.delivered, p->link_acq.lost, p->link_acq.max_consec_lost, p->link_acq.producer_drops,
		p->link_fus.delivered, p->link_fus.lost, p->link_fus.max_consec_lost, p->link_fus.producer_drops,
		p->frames_ok, p->frames_bad, p->rejects, p->n_substeps, p->n_gaps);
	last_pose = *p;
}

static void handle_gps(const pe_gps_msg_t* g)
{
	pe_link_account(&link_gps, &last_gps_seq, g->hdr.seq);
	n_gps++;
	if (g->valid) {
		n_gps_valid++;
	}
	pe_log_printf(g_log_gps, "%llu,%u,%.8f,%.8f,%.2f,%u,%u,%.2f,%.3f,%.2f,%u\n",
		(unsigned long long)g->hdr.t_rx_ns, g->utc_ms_of_day, g->lat_deg, g->lon_deg, g->alt_m,
		g->fix_quality, g->n_sats, g->hdop, g->speed_mps, g->course_deg, g->valid);
}

static int stats_line(char* buf, size_t n)
{
	uint64_t now = pe_now_ns();
	return snprintf(buf, n,
		"%llu,%.1f,%ld,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.5f,"
		"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
		"%d,%d,%d,%d,%d\n",
		(unsigned long long)now, (double)(now - t0_ns) * 1e-9, pe_rss_kb(), pe_cpu_seconds(), n_pose,
		pe_hist_percentile(&lat_hist, 50), pe_hist_percentile(&lat_hist, 95), lat_hist.max_ms,
		pe_hist_percentile(&int_hist, 95), int_hist.max_ms, pe_clockfit_scale(&clockfit),
		last_pose.link_acq.delivered, last_pose.link_acq.lost, last_pose.link_acq.max_consec_lost,
		last_pose.link_acq.producer_drops,
		last_pose.link_fus.delivered, last_pose.link_fus.lost, last_pose.link_fus.max_consec_lost,
		last_pose.link_fus.producer_drops,
		link_est.delivered, link_est.lost, link_est.max_consec_lost, link_est.producer_drops,
		n_gps, n_gps_valid, pe_log_drops(),
		(int)Platform.get_state(&Platform, ag_acq.AID(&ag_acq)),
		(int)Platform.get_state(&Platform, ag_fus.AID(&ag_fus)),
		(int)Platform.get_state(&Platform, ag_est.AID(&ag_est)),
		(int)Platform.get_state(&Platform, ag_com.AID(&ag_com)),
		(int)Platform.get_state(&Platform, ag_gps.AID(&ag_gps)));
}

static MAESAgent* agent_by_name(const char* name)
{
	if (strcmp(name, "acquisition") == 0) return &ag_acq;
	if (strcmp(name, "fusion") == 0) return &ag_fus;
	if (strcmp(name, "estimation") == 0) return &ag_est;
	if (strcmp(name, "gps") == 0) return &ag_gps;
	return NULL;   /* communication cannot suspend itself */
}

static void handle_control(char* cmd, const pe_udp_dest_t* from)
{
	char reply[1100];
	char word[32] = "", arg[64] = "";
	size_t n = strlen(cmd);
	while (n > 0 && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r' || cmd[n - 1] == ' ')) {
		cmd[--n] = '\0';
	}
	sscanf(cmd, "%31s %63s", word, arg);

	if (strcmp(word, "MARK") == 0) {
		pe_event("operator", "MARK %s", cmd + 4 + (cmd[4] == ' '));
		snprintf(reply, sizeof(reply), "OK MARK\n");
	}
	else if (strcmp(word, "RESET_POSE") == 0) {
		pe_ctrl_msg_t* c = &ctrl_pool[ctrl_next];
		double v[3] = { 0, 0, 0 };
		sscanf(cmd + 10, "%lf %lf %lf", &v[0], &v[1], &v[2]);
		memset(c, 0, sizeof(*c));
		c->hdr.kind = PE_MSG_CTRL;
		c->hdr.version = 1;
		c->hdr.seq = ++ctrl_seq;
		c->cmd = PE_CTRL_RESET_POSE;
		memcpy(c->arg, v, sizeof(v));
		if (pe_send(&msg, ag_est.AID(&ag_est), c, REQUEST, 20)) {
			ctrl_next = (ctrl_next + 1u) % PE_POOL_DEPTH;
			snprintf(reply, sizeof(reply), "OK RESET_POSE\n");
		}
		else {
			snprintf(reply, sizeof(reply), "ERR estimation busy, retry\n");
		}
	}
	else if (strcmp(word, "SUSPEND") == 0 || strcmp(word, "RESUME") == 0) {
		MAESAgent* a = agent_by_name(arg);
		if (a == NULL) {
			snprintf(reply, sizeof(reply), "ERR unknown agent '%s'\n", arg);
		}
		else {
			/* Through the AMS, as the thesis describes. The AMS reply (CONFIRM
			 * or REFUSE) arrives in this agent's mailbox and is logged. */
			ERROR_CODE e = word[0] == 'S' ? msg.suspend(&msg, a->AID(a)) : msg.resume(&msg, a->AID(a));
			pe_event("operator", "%s %s -> request %s", word, arg, e == NO_ERRORS ? "sent" : "refused");
			snprintf(reply, sizeof(reply), "%s %s %s\n", e == NO_ERRORS ? "OK" : "ERR", word, arg);
		}
	}
	else if (strcmp(word, "STATS") == 0) {
		stats_line(reply, sizeof(reply));
	}
	else {
		snprintf(reply, sizeof(reply), "ERR commands: MARK <text> | RESET_POSE [x y th] | "
			"SUSPEND|RESUME <acquisition|fusion|estimation|gps> | STATS\n");
	}
	pe_udp_reply(ctrl_fd, from, reply, strlen(reply));
}

static void com_setup(CyclicBehaviour* b, void* arg)
{
	(void)arg;
	b->msg->Agent_Msg(b->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), g_params.cpu_pipeline);
	t0_ns = pe_now_ns();
	last_stats_ns = t0_ns;
	pe_hist_reset(&lat_hist);
	pe_hist_reset(&int_hist);
	pe_clockfit_reset(&clockfit);
	if (pe_udp_sender(&dest1, g_params.udp_pose_host, g_params.udp_pose_port) != 0) {
		pe_event("communication", "cannot open UDP output %s:%d", g_params.udp_pose_host, g_params.udp_pose_port);
	}
	if (g_params.udp_pose_host2[0] != '\0' && g_params.udp_pose_port2 > 0) {
		have_dest2 = pe_udp_sender(&dest2, g_params.udp_pose_host2, g_params.udp_pose_port2) == 0;
	}
	if (g_params.udp_ctrl_port > 0) {
		ctrl_fd = pe_udp_listener(g_params.udp_ctrl_port);
		if (ctrl_fd < 0) {
			pe_event("communication", "cannot open control port %d", g_params.udp_ctrl_port);
		}
	}
}

static void com_action(CyclicBehaviour* b, void* arg)
{
	MSG_TYPE type;
	uint64_t now;
	(void)arg;

	type = b->msg->receive(b->msg, 100);
	if (type == INFORM) {
		const void* c = b->msg->get_msg_content(b->msg);
		uint16_t kind = ((const pe_hdr_t*)c)->kind;
		if (kind == PE_MSG_POSE) {
			pe_pose_msg_t p;
			memcpy(&p, c, sizeof(p));
			handle_pose(&p);
		}
		else if (kind == PE_MSG_GPS) {
			pe_gps_msg_t g;
			memcpy(&g, c, sizeof(g));
			handle_gps(&g);
		}
	}
	else if (type == CONFIRM || type == REFUSE || type == NOT_UNDERSTOOD) {
		pe_event("communication", "AMS reply %s", type == CONFIRM ? "CONFIRM" : "REFUSE");
	}

	if (ctrl_fd >= 0) {
		char buf[256];
		pe_udp_dest_t from;
		int k;
		for (k = 0; k < 8; k++) {
			long n = pe_udp_recv(ctrl_fd, buf, sizeof(buf) - 1, &from);
			if (n <= 0) {
				break;
			}
			buf[n] = '\0';
			handle_control(buf, &from);
		}
	}

	now = pe_now_ns();
	if (now - last_stats_ns >= 1000000000ull) {
		char line[1100];
		int n;
		last_stats_ns = now;
		n = stats_line(line, sizeof(line));
		if (n > 0) {
			pe_log_write(g_log_stats, line, (size_t)n);
		}
	}
}

void communication_main(void* arg)
{
	ConstructorCyclicBehaviour(&beh);
	ConstructorAgent_Msg(&msg, &env);
	beh.msg = &msg;
	beh.setup = &com_setup;
	beh.action = &com_action;
	beh.execute(&beh, arg);
}
