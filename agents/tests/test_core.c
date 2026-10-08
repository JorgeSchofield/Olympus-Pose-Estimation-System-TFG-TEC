/*
 * test_core.c - Unit tests of the portable core (DESIGN.md §16, V-U-*).
 *
 * Runs on any host (Windows, Linux, the RPi): no OS calls, no CMAES.
 *   - RAW parser: valid frames, every malformed class, 10^6 random mutations;
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
		NULL };
	int i;

	CHECK(parse_str("RAW:4294967295:-32768:16384:-100:100:-100:32767:-2147483648:2147483647", &f) == PE_LINE_RAW
		&& f.tick == 4294967295u && f.acc[0] == -32768 && f.acc[1] == 16384 && f.gyr[2] == 32767
		&& f.enc_l == (int32_t)(-2147483647 - 1) && f.enc_r == 2147483647 && !f.has_ts,
		"extreme values roundtrip");
	CHECK(parse_str("@123456789012 RAW:20:1:2:3:4:5:6:7:8\r", &f) == PE_LINE_RAW && f.has_ts
		&& f.t_rx_ns == 123456789012ull && f.tick == 20 && f.enc_r == 8,
		"llcmux prefix and CR");
	CHECK(parse_str("TLM:NORMAL:0", &f) == PE_LINE_TLM, "TLM line");
	CHECK(parse_str("@5 TLM:NORMAL:0", &f) == PE_LINE_TLM, "prefixed TLM line");
	CHECK(parse_str("ACK:STB", &f) == PE_LINE_OTHER, "other line");
	for (i = 0; bad[i] != NULL; i++) {
		CHECK(parse_str(bad[i], &f) == PE_LINE_BAD || parse_str(bad[i], &f) == PE_LINE_OTHER,
			"malformed line accepted: \"%s\"", bad[i]);
		CHECK(parse_str(bad[i], &f) != PE_LINE_RAW, "malformed line parsed as RAW: \"%s\"", bad[i]);
	}

	/* fuzz: random mutations of a valid line never crash, and anything accepted
	 * is strictly "RAW:" + 9 fields of digits with an optional leading '-'
	 * (no spaces, '+', or other leniency), with the values it reports. */
	{
		const char* base = "RAW:123456:-120:44:16390:12:-7:88:120034:-119875";
		char line[96];
		long k, accepted = 0, bad_roundtrip = 0;
		srand(12345);
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
				long long vals[9];
				int nv = 0;
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
				ok = ok && colons == 8;
				/* the reported values are the decimal values of the fields */
				for (q = 3; ok && nv < 9; nv++) {
					vals[nv] = strtoll(line + q + 1, &e, 10);
					q = (size_t)(e - line);
				}
				ok = ok && vals[0] == (long long)f.tick && vals[1] == f.acc[0]
					&& vals[3] == f.acc[2] && vals[6] == f.gyr[2] && vals[7] == f.enc_l && vals[8] == f.enc_r;
				if (!ok) {
					bad_roundtrip++;
				}
			}
		}
		CHECK(bad_roundtrip == 0, "fuzz: %ld accepted lines do not re-serialise", bad_roundtrip);
		printf("  RAW fuzz: 1000000 mutations, %ld accepted, all exact\n", accepted);
	}
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
	CHECK(p.prm.P0[3] == 0.01 && p.geo.B_eff == 0.622 && p.ag.loop_ms == 20, "values");
	CHECK(p.udp_pose_host2[0] == '\0' && p.cpu_pipeline == 3, "empty string and int");
	CHECK(pe_params_parse_line(&p, "prm.k_rho = 2e-4", err, sizeof(err)) == 0 && p.prm.k_rho == 2e-4, "update");
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

int main(void)
{
	test_raw();
	test_nmea();
	test_params();
	test_misc();
	test_equivalence();
	printf("%s: %d checks passed, %d failed\n", n_fail ? "FAIL" : "OK", n_pass, n_fail);
	return n_fail ? 1 : 0;
}
