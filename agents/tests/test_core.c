/*
 * test_core.c - Unit tests of the portable core (DESIGN.md §16, V-U-*).
 *
 * Runs on any host (Windows, Linux, the RPi): no OS calls, no CMAES.
 *   - RAW parser: v2.20 and v3.0 frames, every malformed class, 2x10^6 random
 *     mutations, real firmware v3.0 frames (tests/data/fw_v3_capture.txt);
 *   - NMEA parser; pose.conf parser; line assembly; datagram CRC; statistics;
 *   - V-U-3 / V-U-5: the GENERATED fusion and estimation C reproduce the MATLAB
 *     functions on tests/data/core_vectors.csv (export_core_vectors.m).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "line_buf.h"
#include "nmea_parser.h"
#include "pe_msgs.h"
#include "pe_params.h"
#include "pipeline.h"
#include "pose_dgram.h"
#include "raw_parser.h"
#include "sep_estimation_init.h"
#include "sep_estimation_step.h"
#include "sep_fusion_init.h"
#include "sep_fusion_init_initialize.h"
#include "sep_fusion_step.h"
#include "stats.h"

static int n_pass, n_fail;

#define CHECK(cond, ...) do { \
	if (cond) { n_pass++; } \
	else { n_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static pe_line_kind_t parse_str(const char* s, pe_raw_frame_t* f)
{
	return pe_raw_parse(s, strlen(s), f);
}

/* ------------------------------------------------------------ RAW parser */
static void test_raw(void)
{
	pe_raw_frame_t f;
	static const char* bad[] = {
		"RAW:", "RAW:1:2:3", "RAW:1:2:3:4:5:6:7:8", "RAW:1:2:3:4:5:6:7:8:9:10",
		"RAW:1:2:3:4:5:6:7:8:x", "RAW:1:2:3:4:5:6:7:8:", "RAW::2:3:4:5:6:7:8:9",
		"RAW:-1:2:3:4:5:6:7:8:9",                 /* tick is unsigned */
		"RAW:4294967296:2:3:4:5:6:7:8:9",         /* tick overflow */
		"RAW:1:32768:3:4:5:6:7:8:9",              /* int16 overflow */
		"RAW:1:2:3:4:5:6:7:2147483648:9",         /* int32 overflow */
		"RAW:1:2:3:4:5:6:7:8:9 ", "RAW:1:+2:3:4:5:6:7:8:9", "RAW:1: 2:3:4:5:6:7:8:9",
		"RAW:1:2:3:4:5:6:7:8:-", "@12x RAW:1:2:3:4:5:6:7:8:9", "@123", "@ RAW:1:2:3:4:5:6:7:8:9",
		/* v3.0: 11 fields, temp int16, imu_ok 0 or 1 */
		"RAW:1:2:3:4:5:6:7:8:9:10:2", "RAW:1:2:3:4:5:6:7:8:9:10:-1", "RAW:1:2:3:4:5:6:7:8:9:32768:1",
		"RAW:1:2:3:4:5:6:7:8:9:10:1:0", "RAW:1:2:3:4:5:6:7:8:9:10:", "RAW:1:2:3:4:5:6:7:8:9::1",
		NULL };
	int i;

	CHECK(parse_str("RAW:4294967295:-32768:16384:-100:100:-100:32767:-2147483648:2147483647", &f) == PE_LINE_RAW
		&& f.tick == 4294967295u && f.acc[0] == -32768 && f.acc[1] == 16384 && f.gyr[2] == 32767
		&& f.enc_l == (int32_t)(-2147483647 - 1) && f.enc_r == 2147483647 && !f.has_ts,
		"extreme values roundtrip");
	CHECK(parse_str("@123456789012 RAW:20:1:2:3:4:5:6:7:8\r", &f) == PE_LINE_RAW && f.has_ts
		&& f.t_rx_ns == 123456789012ull && f.tick == 20 && f.enc_r == 8,
		"llcmux prefix and CR");
	CHECK(f.version == 2 && f.imu_ok == 1 && f.temp_raw == 0, "v2.20 frame: IMU assumed valid");
	CHECK(parse_str("RAW:40:-8548:12960:2580:-293:208:-122:-2:-2:3797:1", &f) == PE_LINE_RAW
		&& f.version == 3 && f.tick == 40 && f.acc[0] == -8548 && f.gyr[2] == -122 && f.enc_l == -2
		&& f.temp_raw == 3797 && f.imu_ok == 1,
		"v3.0 frame (firmware capture 2026-10-06)");
	CHECK(parse_str("@99 RAW:60:0:0:0:0:0:0:15:16:0:0\r", &f) == PE_LINE_RAW && f.has_ts
		&& f.version == 3 && f.imu_ok == 0 && f.enc_r == 16,
		"v3.0 frame without IMU, prefixed");
	CHECK(parse_str("RAW:1:2:3:4:5:6:7:8:9:-32768:0", &f) == PE_LINE_RAW && f.temp_raw == -32768,
		"v3.0 temp lower limit");
	CHECK(parse_str("TLM:NORMAL:0", &f) == PE_LINE_TLM, "TLM line");
	CHECK(parse_str("@5 TLM:NORMAL:0", &f) == PE_LINE_TLM, "prefixed TLM line");
	CHECK(parse_str("ACK:STB", &f) == PE_LINE_OTHER, "other line");
	for (i = 0; bad[i] != NULL; i++) {
		CHECK(parse_str(bad[i], &f) == PE_LINE_BAD || parse_str(bad[i], &f) == PE_LINE_OTHER,
			"malformed line accepted: \"%s\"", bad[i]);
		CHECK(parse_str(bad[i], &f) != PE_LINE_RAW, "malformed line parsed as RAW: \"%s\"", bad[i]);
	}

	/* fuzz: random mutations of valid v2.20 and v3.0 lines never crash, and
	 * anything accepted is strictly "RAW:" + 9 or 11 fields of digits with an
	 * optional leading '-' (no spaces, '+', or other leniency), with the values
	 * it reports. */
	{
		static const char* bases[2] = {
			"RAW:123456:-120:44:16390:12:-7:88:120034:-119875",
			"RAW:123456:-120:44:16390:12:-7:88:120034:-119875:3797:1" };
		char line[96];
		int bi;
		for (bi = 0; bi < 2; bi++) {
			const char* base = bases[bi];
			long k, accepted = 0, bad_roundtrip = 0;
			srand(12345 + bi);
			for (k = 0; k < 1000000; k++) {
				size_t len = strlen(base);
				int m, nmut = 1 + rand() % 3;
				memcpy(line, base, len + 1);
				for (m = 0; m < nmut; m++) {
					int op = rand() % 3;
					size_t pos = (size_t)rand() % len;
					char c = (char)(" :-0123456789ARWx\r\n+"[rand() % 21]);
					if (op == 0) {
						line[pos] = c;
					}
					else if (op == 1 && len < sizeof(line) - 2) {
						memmove(line + pos + 1, line + pos, len - pos + 1);
						line[pos] = c;
						len++;
					}
					else if (len > 1) {
						memmove(line + pos, line + pos + 1, len - pos);
						len--;
					}
				}
				if (pe_raw_parse(line, strlen(line), &f) == PE_LINE_RAW) {
					size_t L = strlen(line), q;
					int colons = 0, ok = memcmp(line, "RAW:", 4) == 0;
					long long vals[11];
					int nv = 0, nf;
					char* e;
					if (L > 0 && line[L - 1] == '\r') {
						L--;
					}
					accepted++;
					for (q = 4; ok && q < L; q++) {
						char c = line[q];
						if (c == ':') colons++;
						else if (c == '-') ok = (line[q - 1] == ':') && q + 1 < L && line[q + 1] != ':';
						else ok = (c >= '0' && c <= '9');
					}
					ok = ok && (colons == 8 || colons == 10);
					nf = colons + 1;
					/* the reported values are the decimal values of the fields */
					for (q = 3; ok && nv < nf; nv++) {
						vals[nv] = strtoll(line + q + 1, &e, 10);
						q = (size_t)(e - line);
					}
					ok = ok && vals[0] == (long long)f.tick && vals[1] == f.acc[0]
						&& vals[3] == f.acc[2] && vals[6] == f.gyr[2] && vals[7] == f.enc_l && vals[8] == f.enc_r
						&& f.version == (nf == 11 ? 3 : 2)
						&& (nf == 9 ? f.imu_ok == 1 : (vals[9] == f.temp_raw && vals[10] == f.imu_ok));
					if (!ok) {
						bad_roundtrip++;
					}
				}
			}
			CHECK(bad_roundtrip == 0, "fuzz v%d: %ld accepted lines do not re-serialise", bi + 2, bad_roundtrip);
			printf("  RAW fuzz (%s): 1000000 mutations, %ld accepted, all exact\n", bi ? "v3.0" : "v2.20", accepted);
		}
	}
}

/* Real firmware v3.0 frames: the first 300 lines of raw_20261006_183344.csv
 * (capture_rover_data.py, converted by tools/capture_to_replay.py). All must
 * parse as v3 with the IMU valid, and the LLC clock scale measured from them is
 * the ~1.87 that motivated est.dt_source (DESIGN.md §10.4). */
static void test_fw_capture(void)
{
	char path[512], line[256];
	FILE* fp;
	pe_raw_frame_t f;
	pe_clockfit_t cf;
	long n = 0, n_ok = 0;
	uint32_t prev_tick = 0;
	int ticks_ok = 1;
	double k;

	snprintf(path, sizeof(path), "%s/fw_v3_capture.txt", PE_TEST_DATA);
	fp = fopen(path, "r");
	CHECK(fp != NULL, "cannot open %s", path);
	if (fp == NULL) {
		return;
	}
	pe_clockfit_reset(&cf);
	while (fgets(line, sizeof(line), fp) != NULL) {
		size_t L = strcspn(line, "\r\n");
		n++;
		if (pe_raw_parse(line, L, &f) == PE_LINE_RAW && f.version == 3 && f.imu_ok == 1 && f.has_ts) {
			n_ok++;
			if (n > 1 && f.tick - prev_tick != 20u) {
				ticks_ok = 0;
			}
			prev_tick = f.tick;
			pe_clockfit_add(&cf, f.tick, f.t_rx_ns);
		}
	}
	fclose(fp);
	k = pe_clockfit_scale(&cf);
	CHECK(n == 300 && n_ok == n, "capture: %ld of %ld lines parsed as v3.0", n_ok, n);
	CHECK(ticks_ok, "capture: tick does not advance 20 ms per frame");
	CHECK(k > 1.7 && k < 2.1, "capture: LLC clock scale %.3f", k);
	printf("  firmware v3.0 capture: %ld frames, LLC clock scale %.3f\n", n_ok, k);
}

/* ----------------------------------------------------------------- NMEA */
static void with_checksum(const char* body, char* out, size_t n)
{
	unsigned s = 0;
	const char* p;
	for (p = body; *p; p++) {
		s ^= (unsigned char)*p;
	}
	snprintf(out, n, "$%s*%02X", body, s & 0xFFu);
}

static void test_nmea(void)
{
	char line[128];
	pe_nmea_t g;

	with_checksum("GPRMC,123519.00,A,0956.1234,N,08405.4321,W,0.5,90.0,071026,,", line, sizeof(line));
	memset(&g, 0, sizeof(g));
	CHECK(pe_nmea_parse(line, strlen(line), &g) == PE_NMEA_RMC && g.rmc_valid
		&& fabs(g.lat_deg - (9 + 56.1234 / 60)) < 1e-9 && fabs(g.lon_deg + (84 + 5.4321 / 60)) < 1e-9
		&& g.utc_ms_of_day == 45319000u && fabs(g.speed_mps - 0.257222) < 1e-6, "RMC");
	with_checksum("GNGGA,123519.00,0956.1234,N,08405.4321,W,1,08,0.9,1150.3,M,,M,,", line, sizeof(line));
	memset(&g, 0, sizeof(g));
	CHECK(pe_nmea_parse(line, strlen(line), &g) == PE_NMEA_GGA && g.fix_quality == 1 && g.n_sats == 8
		&& fabs(g.hdop - 0.9) < 1e-12 && fabs(g.alt_m - 1150.3) < 1e-9, "GGA");
	with_checksum("GPRMC,123520.00,V,,,,,,,071026,,", line, sizeof(line));
	memset(&g, 0, sizeof(g));
	CHECK(pe_nmea_parse(line, strlen(line), &g) == PE_NMEA_RMC && !g.rmc_valid, "RMC without fix");
	line[strlen(line) - 1] = (line[strlen(line) - 1] == '0') ? '1' : '0';
	CHECK(pe_nmea_parse(line, strlen(line), &g) == PE_NMEA_BAD, "bad checksum refused");
	with_checksum("GPGSV,3,1,11,03,03,111,00", line, sizeof(line));
	CHECK(pe_nmea_parse(line, strlen(line), &g) == PE_NMEA_OTHER, "other sentence");
	CHECK(pe_nmea_parse("garbage", 7, &g) == PE_NMEA_BAD, "garbage refused");
}

/* --------------------------------------------------------------- params */
static void test_params(void)
{
	pe_params_t p;
	char err[256];
	static char dump[8192];

	pe_params_defaults(&p);
	CHECK(pe_params_check(&p, err, sizeof(err)) != 0, "defaults alone must miss the model parameters");
	CHECK(pe_params_load(&p, PE_TEST_CONF, err, sizeof(err)) == 0, "load %s: %s", PE_TEST_CONF, err);
	CHECK(pe_params_check(&p, err, sizeof(err)) == 0, "shipped config incomplete: %s", err);
	CHECK(p.prm.P0[3] == 0.01 && p.geo.B_eff == 0.933 && p.ag.loop_ms == 20, "values");
	CHECK(p.udp_pose_host2[0] == '\0' && p.cpu_pipeline == 3, "empty string and int");
	CHECK(pe_params_parse_line(&p, "prm.k_rho = 2e-4", err, sizeof(err)) == 0 && p.prm.k_rho == 2e-4, "update");
	CHECK(p.dt_source == 0, "shipped time base is the LLC tick");
	CHECK(pe_params_parse_line(&p, "est.dt_source = 2", err, sizeof(err)) == 0
		&& pe_params_check(&p, err, sizeof(err)) != 0, "est.dt_source out of range refused");
	CHECK(pe_params_parse_line(&p, "est.dt_source = 1", err, sizeof(err)) == 0
		&& pe_params_check(&p, err, sizeof(err)) == 0 && p.dt_source == 1, "est.dt_source = 1");
	CHECK(pe_params_parse_line(&p, "prm.kro = 1", err, sizeof(err)) != 0, "unknown key refused");
	CHECK(pe_params_parse_line(&p, "prm.P0 = 1 2 3", err, sizeof(err)) != 0, "short vector refused");
	CHECK(pe_params_parse_line(&p, "rt.cpu_gps = two", err, sizeof(err)) != 0, "bad integer refused");
	CHECK(pe_params_parse_line(&p, "  # comment only", err, sizeof(err)) == 0, "comment");
	CHECK(pe_params_dump(&p, dump, sizeof(dump)) != 0 && strstr(dump, "prm.P0 =") != NULL, "dump");
}

/* ------------------------------------------------------ line buf, CRC, stats */
static int n_lines;
static char last_line[PE_LINE_MAX];
static void count_line(void* ctx, const char* l, size_t n)
{
	(void)ctx;
	n_lines++;
	memcpy(last_line, l, n + 1);
}

static void test_misc(void)
{
	pe_line_buf_t lb;
	static pe_hist_t h;
	static pe_clockfit_t cf;
	char big[400];
	int i;

	pe_line_buf_reset(&lb);
	pe_line_buf_feed(&lb, "RAW:1:2", 7, count_line, NULL);
	pe_line_buf_feed(&lb, ":3\nTLM:x\n", 9, count_line, NULL);
	CHECK(n_lines == 2 && strcmp(last_line, "TLM:x") == 0, "split lines");
	memset(big, 'A', sizeof(big));
	big[sizeof(big) - 1] = '\n';
	pe_line_buf_feed(&lb, big, sizeof(big), count_line, NULL);
	pe_line_buf_feed(&lb, "ok\n", 3, count_line, NULL);
	CHECK(n_lines == 3 && lb.n_overflow == 1 && strcmp(last_line, "ok") == 0, "overflow discarded");

	CHECK(pe_crc32((const uint8_t*)"123456789", 9) == 0xCBF43926u, "CRC-32 check value");

	/* GPS datagram (pose_dgram.h): layout and CRC as tools/olympus-pose_gui.py decodes it */
	{
		pe_gps_msg_t g;
		uint8_t d[PE_GPS_DGRAM_LEN];
		double lat;
		uint32_t crc;
		memset(&g, 0, sizeof(g));
		g.hdr.t_rx_ns = 0x0102030405060708ull;
		g.lat_deg = 9.85612345;
		g.lon_deg = -83.91234567;
		g.utc_ms_of_day = 45296789u;
		g.fix_quality = 1;
		g.n_sats = 9;
		g.valid = 1;
		pe_gps_dgram_encode(&g, 7u, 42u, d);
		memcpy(&lat, d + 16, sizeof(lat));   /* little-endian hosts only, as the targets */
		crc = (uint32_t)d[76] | (uint32_t)d[77] << 8 | (uint32_t)d[78] << 16 | (uint32_t)d[79] << 24;
		CHECK(memcmp(d, "OGP1", 4) == 0 && d[4] == 7 && d[8] == 0x08 && d[15] == 0x01
			&& lat == g.lat_deg && d[64] == (uint8_t)(45296789u & 0xFF)
			&& d[68] == 1 && d[69] == 9 && d[70] == 1 && d[72] == 42 && crc == pe_crc32(d, 76u),
			"GPS datagram layout");
	}

	pe_hist_reset(&h);
	for (i = 1; i <= 100; i++) {
		pe_hist_add(&h, (double)i);
	}
	CHECK(fabs(pe_hist_percentile(&h, 95) - 95.1) < 1e-9 && h.max_ms == 100.0, "percentile");

	pe_clockfit_reset(&cf);
	for (i = 0; i < 500; i++) {   /* LLC says 20 ms, real cycle 30.6 ms -> k = 1.53 */
		pe_clockfit_add(&cf, (uint32_t)(i * 20), (uint64_t)(i * 30.6e6));
	}
	CHECK(fabs(pe_clockfit_scale(&cf) - 1.53) < 1e-6, "clock scale %.6f", pe_clockfit_scale(&cf));
}

/* --------------------------------------- V-U-3 / V-U-5: generated C vs MATLAB */
static double rel(double a, double b)
{
	double d = fabs(a - b), m = fabs(b) > 1.0 ? fabs(b) : 1.0;
	return d / m;
}

static void test_equivalence(void)
{
	char path[512], line[2048];
	FILE* f;
	pe_params_t p;
	char err[256];
	sep_fusion_state_t fs;
	sep_estimation_state_t es;
	long rows = 0, est_steps = 0;
	double worst_fus = 0, worst_est = 0;

	pe_params_defaults(&p);
	if (pe_params_load(&p, PE_TEST_CONF, err, sizeof(err)) != 0) {
		CHECK(0, "config: %s", err);
		return;
	}
	snprintf(path, sizeof(path), "%s/core_vectors.csv", PE_TEST_DATA);
	f = fopen(path, "r");
	CHECK(f != NULL, "cannot open %s", path);
	if (f == NULL) {
		return;
	}
	sep_fusion_init_initialize();
	sep_fusion_init(&fs);
	sep_estimation_init(&p.prm, &es);

	while (fgets(line, sizeof(line), f) != NULL) {
		double v[26];
		int k = 0;
		char* s = line;
		sep_odom_t od;
		boolean_T emit = 0, did = 0;
		double acc[3], xe[5], dg[3];
		if (line[0] == '#') {
			continue;
		}
		while (k < 26) {
			char* end;
			v[k++] = strtod(s, &end);
			if (*end != ',') break;
			s = end + 1;
		}
		if (k != 26) {
			CHECK(0, "row %ld has %d columns", rows, k);
			break;
		}
		rows++;
		acc[0] = v[4]; acc[1] = v[5]; acc[2] = v[6];
		sep_fusion_step(&fs, v[0], v[1], v[2], v[3], acc, 1, &p.geo, &p.ag, &od, &emit);
		if ((double)emit != v[8]) {
			CHECK(0, "row %ld: emit %d vs %g", rows, (int)emit, v[8]);
			continue;
		}
		{
			double e = rel(od.S, v[9]);
			if (rel(od.TH, v[10]) > e) e = rel(od.TH, v[10]);
			if (rel(od.OM, v[11]) > e) e = rel(od.OM, v[11]);
			if (od.n_g != v[12] || od.n_s != v[13] || od.t_cum != v[14] || od.still_frames != v[15]
				|| (double)od.still != v[16]) e = 1.0;
			if (e > worst_fus) worst_fus = e;
		}
		if (emit && v[7] == 0.0) {
			int i;
			sep_estimation_step(&es, &od, &p.prm, &p.ag, xe, dg, &did);
			if ((double)did != v[17]) {
				CHECK(0, "row %ld: did %d vs %g", rows, (int)did, v[17]);
			}
			if (did) {
				est_steps++;
				for (i = 0; i < 5; i++) {
					double e = rel(xe[i], v[18 + i]);
					if (e > worst_est) worst_est = e;
				}
				for (i = 0; i < 3; i++) {
					double e = rel(dg[i], v[23 + i]);
					if (e > worst_est) worst_est = e;
				}
			}
		}
	}
	fclose(f);
	CHECK(rows > 2000, "only %ld rows", rows);
	CHECK(worst_fus < 1e-12, "fusion differs from MATLAB: %.3g", worst_fus);
	CHECK(worst_est < 1e-9, "estimation differs from MATLAB: %.3g", worst_est);
	printf("  equivalence: %ld frames, %ld EKF steps, worst rel. diff fusion %.2e, estimation %.2e\n",
		rows, est_steps, worst_fus, worst_est);
}

/* ------------------------ V-U-10: model and app run the same estimation chain
 * tests/data/replay_input.txt goes through what the agents do in replay mode
 * (pe_raw_parse -> pipeline.c -> generated fusion -> generated estimation, no
 * message dropped, estimation once per fusion message) and must publish the
 * same poses, at the same input lines, as the model's hlc_replay.m
 * (replay_vectors_dt<k>.csv, from tests/matlab/export_replay_vectors.m). */
static void replay_parity(int dt_source)
{
	char path[512], line[512], row[1024];
	FILE *fin, *fv;
	pe_params_t p;
	char err[256];
	sep_fusion_state_t fs;
	sep_estimation_state_t es;
	long lineno = 0, poses = 0, bad_rows = 0;
	double worst = 0;

	pe_params_defaults(&p);
	if (pe_params_load(&p, PE_TEST_CONF, err, sizeof(err)) != 0) {
		CHECK(0, "config: %s", err);
		return;
	}
	p.dt_source = dt_source;
	snprintf(path, sizeof(path), "%s/replay_input.txt", PE_TEST_DATA);
	fin = fopen(path, "r");
	snprintf(path, sizeof(path), "%s/replay_vectors_dt%d.csv", PE_TEST_DATA, dt_source);
	fv = fopen(path, "r");
	CHECK(fin != NULL && fv != NULL, "cannot open the replay files (%s)", path);
	if (fin == NULL || fv == NULL) {
		if (fin) fclose(fin);
		if (fv) fclose(fv);
		return;
	}
	if (fgets(row, sizeof(row), fv) == NULL) {   /* header */
		row[0] = '\0';
	}
	sep_fusion_init_initialize();
	sep_fusion_init(&fs);
	sep_estimation_init(&p.prm, &es);

	while (fgets(line, sizeof(line), fin) != NULL) {
		pe_raw_frame_t f;
		sep_odom_t od;
		boolean_T emit = 0, did = 0;
		double acc[3], gz, xe[5], dg[3], v[15];
		uint32_t t_in;
		size_t len = strcspn(line, "\r\n");
		int k = 0, i;
		char* s = row;

		lineno++;
		if (pe_raw_parse(line, len, &f) != PE_LINE_RAW) {
			continue;
		}
		t_in = pe_time_base_ms(f.tick, f.t_rx_ns, p.dt_source);
		pe_fusion_axes(f.acc, f.gyr, p.gyro_yaw_axis, &gz, acc);
		sep_fusion_step(&fs, (double)t_in, (double)f.enc_l, (double)f.enc_r, gz, acc,
			(boolean_T)f.imu_ok, &p.geo, &p.ag, &od, &emit);
		if (!emit) {
			continue;
		}
		sep_estimation_step(&es, &od, &p.prm, &p.ag, xe, dg, &did);
		if (!did) {
			continue;
		}
		poses++;
		if (fgets(row, sizeof(row), fv) == NULL) {
			bad_rows++;
			continue;
		}
		while (k < 15) {
			char* end;
			v[k++] = strtod(s, &end);
			if (*end != ',') break;
			s = end + 1;
		}
		/* same input line, same time value, then S, x[5], diag[3], still, imu_ok */
		if (k != 15 || v[0] != (double)lineno || v[3] != (double)t_in
			|| v[13] != (double)od.still || v[14] != (double)f.imu_ok) {
			bad_rows++;
			continue;
		}
		{
			double e = rel(od.S, v[4]);
			for (i = 0; i < 5; i++) if (rel(xe[i], v[5 + i]) > e) e = rel(xe[i], v[5 + i]);
			for (i = 0; i < 3; i++) if (rel(dg[i], v[10 + i]) > e) e = rel(dg[i], v[10 + i]);
			if (e > worst) worst = e;
		}
	}
	if (fgets(row, sizeof(row), fv) != NULL) {
		bad_rows++;                              /* the model published more poses */
	}
	fclose(fin);
	fclose(fv);
	CHECK(poses > 2000, "replay dt_source=%d: only %ld poses", dt_source, poses);
	CHECK(bad_rows == 0, "replay dt_source=%d: %ld poses at a different line/time/flag than the model",
		dt_source, bad_rows);
	CHECK(worst < 1e-9, "replay dt_source=%d: pose differs from the model: %.3g", dt_source, worst);
	printf("  replay parity (dt_source=%d): %ld lines, %ld poses, worst rel. diff vs model %.2e\n",
		dt_source, lineno, poses, worst);
}

/* V-U-9, parser parity: pe_raw_parse and the model's raw_frame_from_bytes give the
 * same verdict and values on 5000 mutated lines (parser_vectors.csv from
 * tests/matlab/export_parser_vectors.m). */
static void parser_parity(void)
{
	char path[512], line[256], row[256];
	FILE *fin, *fv;
	long n = 0, accepted = 0, differ = 0;

	snprintf(path, sizeof(path), "%s/parser_cases.txt", PE_TEST_DATA);
	fin = fopen(path, "r");
	snprintf(path, sizeof(path), "%s/parser_vectors.csv", PE_TEST_DATA);
	fv = fopen(path, "r");
	CHECK(fin != NULL && fv != NULL, "cannot open the parser parity files");
	if (fin == NULL || fv == NULL) {
		if (fin) fclose(fin);
		if (fv) fclose(fv);
		return;
	}
	while (fgets(line, sizeof(line), fin) != NULL) {
		pe_raw_frame_t f;
		long long m[9];
		int ok = pe_raw_parse(line, strcspn(line, "\r\n"), &f) == PE_LINE_RAW;
		n++;
		if (fgets(row, sizeof(row), fv) == NULL
			|| sscanf(row, "%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld",
				&m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]) != 9) {
			differ++;
			continue;
		}
		accepted += ok;
		if (ok != (int)m[0] || (ok && (m[1] != f.version || m[2] != (long long)f.tick
			|| m[3] != f.acc[0] || m[4] != f.gyr[2] || m[5] != f.enc_l || m[6] != f.enc_r
			|| m[7] != f.temp_raw || m[8] != f.imu_ok))) {
			differ++;
		}
	}
	fclose(fin);
	fclose(fv);
	CHECK(n == 5000 && differ == 0, "parser parity: %ld of %ld lines differ from the model", differ, n);
	printf("  parser parity: %ld lines, %ld accepted, %ld differ from the model\n", n, accepted, differ);
}

/* Published-pose frame (io.pose_frame): frame 1 = frame 0 rotated +90 deg. */
static void test_pose_frame(void)
{
	double x0[5] = { 1.188, 0.136, 0.2461, 0.01, 0.0045 };
	double P0[25], x1[5], P1[25], back[3], in[3];
	int i, j;
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			P0[i + 5 * j] = (i == j) ? 0.1 * (i + 1) : 0.01 * (i + j + 1);   /* symmetric */
		}
	}
	pe_pose_to_frame(0, x0, P0, x1, P1);
	CHECK(memcmp(x0, x1, sizeof(x0)) == 0 && memcmp(P0, P1, sizeof(P0)) == 0, "frame 0 is the identity");
	pe_pose_to_frame(1, x0, P0, x1, P1);
	/* forward 1.188 m and 0.136 m to the left -> x = -0.136 (left is -x), y = 1.188 */
	CHECK(x1[0] == -0.136 && x1[1] == 1.188 && x1[2] == x0[2] && x1[3] == x0[3] && x1[4] == x0[4],
		"frame 1 position");
	CHECK(P1[0] == P0[6] && P1[6] == P0[0] && P1[1] == -P0[1] && P1[12] == P0[12]
		&& P1[0 + 5 * 2] == -P0[1 + 5 * 2] && P1[1 + 5 * 2] == P0[0 + 5 * 2],
		"frame 1 covariance rotates with the position");
	in[0] = x1[0]; in[1] = x1[1]; in[2] = x1[2];
	pe_pose_from_frame(1, in, back);
	CHECK(back[0] == x0[0] && back[1] == x0[1] && back[2] == x0[2], "RESET_POSE inverse of frame 1");
}

static void test_replay_parity(void)
{
	parser_parity();
	replay_parity(0);
	replay_parity(1);
}

int main(void)
{
	test_raw();
	test_nmea();
	test_params();
	test_fw_capture();
	test_misc();
	test_equivalence();
	test_replay_parity();
	test_pose_frame();
	printf("%s: %d checks passed, %d failed\n", n_fail ? "FAIL" : "OK", n_pass, n_fail);
	return n_fail ? 1 : 0;
}
