/*
 * agent_acquisition.c - Acquisition agent (DESIGN.md §7.1).
 *
 * Reads the byte stream from llcmux (pty B), a serial port or a replay file,
 * assembles lines, accepts only RAW: frames, and hands the fields to the
 * fusion agent UNCONVERTED (PE-RF-008). Syntax checks only; plausibility is
 * the fusion agent's job.
 */
#include "app.h"

#include <string.h>

#include "line_buf.h"
#include "port.h"
#include "raw_parser.h"

static CyclicBehaviour beh;
static Agent_Msg       msg;
static pe_raw_msg_t    pool[PE_POOL_DEPTH];
static unsigned        pool_next;
static uint32_t        seq;
static pe_line_buf_t   lb;
static int             fd = -1;
static uint32_t        frames_ok, frames_bad, tlm_lines, drops;
static uint64_t        rx_now_ns;
static int             eof_seen;

static void on_line(void* ctx, const char* line, size_t len)
{
	pe_raw_frame_t f;
	pe_raw_msg_t* m;
	(void)ctx;

	switch (pe_raw_parse(line, len, &f)) {
	case PE_LINE_RAW:
		break;
	case PE_LINE_TLM:
		tlm_lines++;
		return;
	case PE_LINE_BAD:
		frames_bad++;
		return;
	default:
		return;
	}
	frames_ok++;

	m = &pool[pool_next];
	memset(m, 0, sizeof(*m));
	m->hdr.kind = PE_MSG_RAW;
	m->hdr.version = 1;
	m->hdr.seq = seq + 1u;
	m->hdr.t_rx_ns = f.has_ts ? f.t_rx_ns : rx_now_ns;
	m->t_llc_ms = f.tick;
	m->enc_l = f.enc_l;
	m->enc_r = f.enc_r;
	memcpy(m->acc, f.acc, sizeof(m->acc));
	memcpy(m->gyr, f.gyr, sizeof(m->gyr));
	/* firmware v2.20 only sends RAW: when the IMU read succeeded */
	m->flags = (uint8_t)(PE_RAW_IMU_VALID | (f.has_ts ? 0u : PE_RAW_RX_TS_LOCAL));
	m->line_len = (uint16_t)len;
	m->frames_ok = frames_ok;
	m->frames_bad = frames_bad;
	m->tlm_lines = tlm_lines;
	m->producer_drops = drops;

	if (pe_send(&msg, ag_fus.AID(&ag_fus), m, INFORM, pe_send_timeout())) {
		seq++;
		pool_next = (pool_next + 1u) % PE_POOL_DEPTH;   /* advance only after success */
	}
	else {
		drops++;                                        /* keep the same buffer */
	}
}

static void acq_setup(CyclicBehaviour* b, void* arg)
{
	(void)arg;
	b->msg->Agent_Msg(b->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), g_params.cpu_pipeline);
	pe_line_buf_reset(&lb);
}

static void acq_action(CyclicBehaviour* b, void* arg)
{
	char buf[512];
	long n;
	int w;
	(void)b;
	(void)arg;

	if (eof_seen) {
		Platform.agent_wait(&Platform, 1000);
		return;
	}
	if (fd < 0) {
		fd = pe_stream_open(g_params.raw_input, g_params.raw_baud);
		if (fd < 0) {
			Platform.agent_wait(&Platform, 500);
			return;
		}
		pe_line_buf_reset(&lb);
		pe_event("acquisition", "opened %s", g_params.raw_input);
	}

	w = pe_stream_wait(fd, 100);
	if (w == 0) {
		return;
	}
	if (w < 0) {
		pe_event("acquisition", "input hang-up, reopening");
		pe_stream_close(fd);
		fd = -1;
		return;
	}
	n = pe_stream_read(fd, buf, sizeof(buf));
	if (n == -2) {
		return;
	}
	if (n <= 0) {
		pe_stream_close(fd);
		fd = -1;
		if (g_params.replay && n == 0) {
			eof_seen = 1;
			pe_event("acquisition", "end of replay file: %u frames, %u bad", frames_ok, frames_bad);
			Platform.agent_wait(&Platform, 1000);   /* let the chain drain */
			pe_request_stop();
		}
		else {
			pe_event("acquisition", "input closed, reopening");
		}
		return;
	}
	rx_now_ns = pe_now_ns();
	pe_line_buf_feed(&lb, buf, (size_t)n, on_line, NULL);
}

void acquisition_main(void* arg)
{
	ConstructorCyclicBehaviour(&beh);
	ConstructorAgent_Msg(&msg, &env);
	beh.msg = &msg;
	beh.setup = &acq_setup;
	beh.action = &acq_action;
	beh.execute(&beh, arg);
}
