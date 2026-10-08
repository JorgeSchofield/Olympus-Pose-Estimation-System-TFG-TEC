/*
 * agent_estimation.c - Estimation agent (DESIGN.md §7.3).
 *
 * Wraps the generated sep_estimation_step: differences of the cumulative
 * values against what THIS agent last processed, sub-steps for long gaps, and
 * the unchanged 5-state EKF (sep_ekf_step). Keeps x and P between cycles
 * (PE-RF-010) and flags slip (PE-RF-011).
 *
 * Mode: WAIT_DATA -> INITIALIZING (first message) -> RUNNING once the rover
 * has been still for est.t_init_s (the gyro bias is only observable at rest,
 * thesis §3.2.8). The pose is published in every mode; VALID is set only in
 * RUNNING.
 */
#include "app.h"

#include <string.h>

#include "port.h"
#include "sep_estimation_init.h"
#include "sep_estimation_step.h"

static CyclicBehaviour        beh;
static Agent_Msg              msg;
static pe_pose_msg_t          pool[PE_POOL_DEPTH];
static unsigned               pool_next;
static uint32_t               seq, last_in_seq, drops;
static pe_link_stats_t        link_fus;
static sep_estimation_state_t est;
static pe_mode_t              mode = PE_MODE_WAIT_DATA;
static int                    stale;
static pe_odom_msg_t          last_in;     /* for STALE publications */
static double                 last_diag[3];

static void publish(const pe_odom_msg_t* in, const double diag[3], uint16_t extra_status)
{
	pe_pose_msg_t* m = &pool[pool_next];
	uint16_t st = extra_status;

	memset(m, 0, sizeof(*m));
	m->hdr.kind = PE_MSG_POSE;
	m->hdr.version = 1;
	m->hdr.seq = seq + 1u;
	m->hdr.t_rx_ns = in->hdr.t_rx_ns;
	m->t_est_ns = pe_now_ns();
	memcpy(m->x, est.x, sizeof(m->x));
	memcpy(m->P, est.P, sizeof(m->P));
	m->s_m = in->odom.S;
	memcpy(m->diag, diag, sizeof(m->diag));
	m->t_cum_ms = in->odom.t_cum;
	m->t_llc_ms = (uint32_t)in->odom.tick;
	m->mode = (uint16_t)mode;

	if (mode == PE_MODE_RUNNING) st |= PE_ST_VALID;
	if (in->odom.still) st |= PE_ST_STILL;
	if (diag[0] > g_params.prm.slip_thresh) st |= PE_ST_SLIP;
	if (est.gap) st |= PE_ST_GAP;
	if (est.imu_invalid) st |= PE_ST_IMU_INVALID;
	if (in->odom.rollover) st |= PE_ST_ROLLOVER;
	if (in->odom.llc_reset) st |= PE_ST_LLC_RESET;
	if (in->odom.reref) st |= PE_ST_REREF;
	m->status = st;

	m->link_acq = in->link_acq;
	m->link_fus = link_fus;
	m->frames_ok = in->frames_ok;
	m->frames_bad = in->frames_bad;
	m->rejects = in->rejects;
	m->n_substeps = (uint32_t)est.n_substeps;
	m->n_gaps = (uint32_t)est.n_gaps;
	m->producer_drops = drops;

	if (pe_send(&msg, ag_com.AID(&ag_com), m, INFORM, pe_send_timeout())) {
		seq++;
		pool_next = (pool_next + 1u) % PE_POOL_DEPTH;
	}
	else {
		drops++;
	}
}

static void est_setup(CyclicBehaviour* b, void* arg)
{
	(void)arg;
	b->msg->Agent_Msg(b->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), g_params.cpu_pipeline);
	sep_estimation_init(&g_params.prm, &est);
}

static void handle_ctrl(const pe_ctrl_msg_t* c)
{
	if (c->cmd == PE_CTRL_RESET_POSE) {
		est.x[0] = c->arg[0];
		est.x[1] = c->arg[1];
		est.x[2] = c->arg[2];
		pe_event("estimation", "pose reset to x=%.3f y=%.3f theta=%.4f", c->arg[0], c->arg[1], c->arg[2]);
	}
}

static void est_action(CyclicBehaviour* b, void* arg)
{
	MSG_TYPE type;
	double xe[5], diag[3];
	boolean_T did = 0;
	const void* content;
	(void)arg;

	type = b->msg->receive(b->msg, (MAESTickType_t)g_params.stale_ms);
	if (type == NO_RESPONSE) {
		if (mode != PE_MODE_WAIT_DATA && !stale && !g_params.replay) {
			stale = 1;
			pe_event("estimation", "no input for %.0f ms: STALE", g_params.stale_ms);
			publish(&last_in, last_diag, PE_ST_STALE);
		}
		return;
	}
	if (type != INFORM && type != REQUEST) {   /* data are INFORM, operator commands REQUEST */
		return;
	}
	content = b->msg->get_msg_content(b->msg);

	if (((const pe_hdr_t*)content)->kind == PE_MSG_CTRL) {
		pe_ctrl_msg_t c;
		memcpy(&c, content, sizeof(c));
		handle_ctrl(&c);
		return;
	}
	if (((const pe_hdr_t*)content)->kind != PE_MSG_ODOM) {
		return;
	}
	memcpy(&last_in, content, sizeof(last_in));   /* copy out at once */
	pe_link_account(&link_fus, &last_in_seq, last_in.hdr.seq);
	link_fus.producer_drops = last_in.producer_drops;
	if (stale) {
		stale = 0;
		pe_event("estimation", "input back");
	}

	sep_estimation_step(&est, &last_in.odom, &g_params.prm, &g_params.ag, xe, diag, &did);

	if (mode == PE_MODE_WAIT_DATA) {
		mode = PE_MODE_INITIALIZING;
		pe_event("estimation", "first data: INITIALIZING (waiting for %.1f s at rest)", g_params.t_init_s);
	}
	if (mode == PE_MODE_INITIALIZING && last_in.odom.still &&
		last_in.odom.still_frames * g_params.ag.loop_ms >= g_params.t_init_s * 1000.0) {
		mode = PE_MODE_RUNNING;
		pe_event("estimation", "RUNNING (bias %.5f rad/s)", est.x[4]);
	}
	if (!did) {
		return;   /* reference only (first message or re-reference) */
	}
	memcpy(last_diag, diag, sizeof(last_diag));
	publish(&last_in, diag, 0);
}

void estimation_main(void* arg)
{
	ConstructorCyclicBehaviour(&beh);
	ConstructorAgent_Msg(&msg, &env);
	beh.msg = &msg;
	beh.setup = &est_setup;
	beh.action = &est_action;
	beh.execute(&beh, arg);
}
