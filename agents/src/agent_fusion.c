/*
 * agent_fusion.c - Data fusion agent (DESIGN.md §7.2, decision D2).
 *
 * Wraps the generated sep_fusion_step: plausibility against the last ACCEPTED
 * frame, LLC-reset handling, counts -> metres with sep_odometry, and CUMULATIVE
 * S, TH, OM sent to the estimation agent. Because the values are cumulative,
 * a message dropped at the next mailbox costs time resolution, not distance.
 */
#include "app.h"

#include <string.h>

#include "sep_fusion_init.h"
#include "sep_fusion_step.h"

static CyclicBehaviour    beh;
static Agent_Msg          msg;
static pe_odom_msg_t      pool[PE_POOL_DEPTH];
static unsigned           pool_next;
static uint32_t           seq, last_in_seq, drops;
static pe_link_stats_t    link_acq;
static sep_fusion_state_t fst;

static void fus_setup(CyclicBehaviour* b, void* arg)
{
	(void)arg;
	b->msg->Agent_Msg(b->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), g_params.cpu_pipeline);
	sep_fusion_init(&fst);
}

static void fus_action(CyclicBehaviour* b, void* arg)
{
	pe_raw_msg_t in;
	pe_odom_msg_t* m;
	double acc[3], llc_resets_before, rerefs_before;
	boolean_T emit = 0;
	sep_odom_t odom;
	(void)arg;

	if (b->msg->receive(b->msg, MAES_MAX_DELAY) != INFORM) {
		return;
	}
	memcpy(&in, b->msg->get_msg_content(b->msg), sizeof(in));   /* copy out at once */
	if (in.hdr.kind != PE_MSG_RAW) {
		return;
	}
	pe_link_account(&link_acq, &last_in_seq, in.hdr.seq);
	link_acq.producer_drops = in.producer_drops;

	acc[0] = in.acc[0];
	acc[1] = in.acc[1];
	acc[2] = in.acc[2];
	llc_resets_before = fst.n_llc_reset;
	rerefs_before = fst.n_reref;
	sep_fusion_step(&fst, (double)in.t_llc_ms, (double)in.enc_l, (double)in.enc_r,
		(double)in.gyr[g_params.gyro_yaw_axis], acc,
		(boolean_T)((in.flags & PE_RAW_IMU_VALID) != 0),
		&g_params.geo, &g_params.ag, &odom, &emit);

	if (fst.n_llc_reset != llc_resets_before) {
		pe_event("fusion", "LLC reset detected (tick %u)", in.t_llc_ms);
	}
	if (fst.n_reref != rerefs_before) {
		pe_event("fusion", "re-referenced after %d rejected frames", (int)g_params.ag.k_reref);
	}
	if (!emit) {
		return;   /* frame rejected: the next good frame carries the totals */
	}

	m = &pool[pool_next];
	memset(m, 0, sizeof(*m));
	m->hdr.kind = PE_MSG_ODOM;
	m->hdr.version = 1;
	m->hdr.seq = seq + 1u;
	m->hdr.t_rx_ns = in.hdr.t_rx_ns;
	m->odom = odom;
	m->link_acq = link_acq;
	m->frames_ok = in.frames_ok;
	m->frames_bad = in.frames_bad;
	m->rejects = (uint32_t)fst.n_reject;
	m->rerefs = (uint32_t)fst.n_reref;
	m->llc_resets = (uint32_t)fst.n_llc_reset;
	m->producer_drops = drops;

	if (pe_send(b->msg, ag_est.AID(&ag_est), m, INFORM, pe_send_timeout())) {
		seq++;
		pool_next = (pool_next + 1u) % PE_POOL_DEPTH;
	}
	else {
		drops++;
	}
}

void fusion_main(void* arg)
{
	ConstructorCyclicBehaviour(&beh);
	ConstructorAgent_Msg(&msg, &env);
	beh.msg = &msg;
	beh.setup = &fus_setup;
	beh.action = &fus_action;
	beh.execute(&beh, arg);
}
