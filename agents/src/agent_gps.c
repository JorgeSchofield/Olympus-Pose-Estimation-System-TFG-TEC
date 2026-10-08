/*
 * agent_gps.c - GPS agent (DESIGN.md §7.5).
 *
 * Reads NMEA from the GPIO UART (/dev/ttyAMA0 by default), joins the RMC and
 * GGA of the same epoch into one fix, stamps it with the HLC monotonic clock
 * and forwards it to the communication agent for logging (PE-RF-013, OE2).
 * The GPS NEVER enters the filter (thesis §1.6). Not on the latency path: if
 * the communication mailbox is full the fix is kept and retried.
 */
#include "app.h"

#include <string.h>

#include "line_buf.h"
#include "nmea_parser.h"
#include "port.h"

static CyclicBehaviour beh;
static Agent_Msg       msg;
static pe_gps_msg_t    pool[PE_POOL_DEPTH];
static unsigned        pool_next;
static uint32_t        seq, drops, bad_sentences;
static pe_line_buf_t   lb;
static int             fd = -1;
static uint64_t        next_open_ns;
static pe_nmea_t       epoch;
static int             have_rmc, have_gga, pending;
static uint64_t        rx_now_ns;

static void try_send_pending(void)
{
	if (!pending) {
		return;
	}
	if (pe_send(&msg, ag_com.AID(&ag_com), &pool[pool_next], INFORM, 0)) {
		seq++;
		pool_next = (pool_next + 1u) % PE_POOL_DEPTH;
		pending = 0;
	}
	else {
		drops++;   /* retried on the next wake-up */
	}
}

static void emit_epoch(void)
{
	pe_gps_msg_t* g;
	if (pending) {
		try_send_pending();
		if (pending) {
			return;   /* communication still busy: this epoch is lost */
		}
	}
	g = &pool[pool_next];
	memset(g, 0, sizeof(*g));
	g->hdr.kind = PE_MSG_GPS;
	g->hdr.version = 1;
	g->hdr.seq = seq + 1u;
	g->hdr.t_rx_ns = rx_now_ns;
	g->lat_deg = epoch.lat_deg;
	g->lon_deg = epoch.lon_deg;
	g->alt_m = epoch.alt_m;
	g->hdop = epoch.hdop;
	g->speed_mps = epoch.speed_mps;
	g->course_deg = epoch.course_deg;
	g->utc_ms_of_day = epoch.utc_ms_of_day;
	g->fix_quality = (uint8_t)epoch.fix_quality;
	g->n_sats = (uint8_t)epoch.n_sats;
	g->valid = (uint8_t)(epoch.rmc_valid && epoch.fix_quality > 0);
	g->producer_drops = drops;
	pending = 1;
	try_send_pending();
}

static void on_line(void* ctx, const char* line, size_t len)
{
	pe_nmea_t s;
	pe_nmea_kind_t k;
	(void)ctx;
	memset(&s, 0, sizeof(s));
	k = pe_nmea_parse(line, len, &s);
	if (k == PE_NMEA_BAD) {
		bad_sentences++;
		return;
	}
	if (k != PE_NMEA_RMC && k != PE_NMEA_GGA) {
		return;
	}
	if ((have_rmc || have_gga) && s.utc_ms_of_day != epoch.utc_ms_of_day) {
		have_rmc = have_gga = 0;   /* new epoch: drop the incomplete one */
	}
	if (!have_rmc && !have_gga) {
		memset(&epoch, 0, sizeof(epoch));
		epoch.utc_ms_of_day = s.utc_ms_of_day;
	}
	if (k == PE_NMEA_RMC) {
		epoch.rmc_valid = s.rmc_valid;
		epoch.speed_mps = s.speed_mps;
		epoch.course_deg = s.course_deg;
		if (s.rmc_valid) {
			epoch.lat_deg = s.lat_deg;
			epoch.lon_deg = s.lon_deg;
		}
		have_rmc = 1;
	}
	else {
		epoch.fix_quality = s.fix_quality;
		epoch.n_sats = s.n_sats;
		epoch.hdop = s.hdop;
		epoch.alt_m = s.alt_m;
		if (s.fix_quality > 0) {
			epoch.lat_deg = s.lat_deg;
			epoch.lon_deg = s.lon_deg;
		}
		have_gga = 1;
	}
	if (have_rmc && have_gga) {
		emit_epoch();
		have_rmc = have_gga = 0;
	}
}

static void gps_setup(CyclicBehaviour* b, void* arg)
{
	(void)arg;
	b->msg->Agent_Msg(b->msg);
	MAES_SetAffinity(MAES_GetCurrentTaskHandle(), g_params.cpu_gps);
	pe_line_buf_reset(&lb);
}

static void gps_action(CyclicBehaviour* b, void* arg)
{
	char buf[256];
	long n;
	int w;
	(void)b;
	(void)arg;

	if (g_params.gps_port[0] == '\0' || g_params.replay) {
		Platform.agent_wait(&Platform, 1000);   /* GPS disabled: stay idle */
		return;
	}
	try_send_pending();
	if (fd < 0) {
		uint64_t now = pe_now_ns();
		if (now < next_open_ns) {
			Platform.agent_wait(&Platform, 200);
			return;
		}
		fd = pe_stream_open(g_params.gps_port, g_params.gps_baud);
		if (fd < 0) {
			next_open_ns = now + 5000000000ull;   /* retry every 5 s */
			return;
		}
		pe_line_buf_reset(&lb);
		pe_event("gps", "opened %s at %d baud", g_params.gps_port, g_params.gps_baud);
	}
	w = pe_stream_wait(fd, 50);
	if (w == 0) {
		return;
	}
	n = w > 0 ? pe_stream_read(fd, buf, sizeof(buf)) : -1;
	if (n == -2) {
		return;
	}
	if (n <= 0) {
		pe_event("gps", "port closed, reopening");
		pe_stream_close(fd);
		fd = -1;
		return;
	}
	rx_now_ns = pe_now_ns();
	pe_line_buf_feed(&lb, buf, (size_t)n, on_line, NULL);
}

void gps_main(void* arg)
{
	ConstructorCyclicBehaviour(&beh);
	ConstructorAgent_Msg(&msg, &env);
	beh.msg = &msg;
	beh.setup = &gps_setup;
	beh.action = &gps_action;
	beh.execute(&beh, arg);
}
