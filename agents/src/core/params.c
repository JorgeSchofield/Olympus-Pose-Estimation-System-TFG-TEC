/*
 * params.c - pose.conf parser (DESIGN.md §12). Portable C99, no OS calls
 * beyond stdio.
 */
#include "pe_params.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { T_DBL, T_INT, T_STR } ptype_t;

typedef struct {
	const char* key;
	ptype_t     type;
	size_t      offset;
	int         count;      /* doubles: number of values; strings: buffer size */
	int         required;   /* model parameters: must appear in the file */
} pdesc_t;

#define D(k, f)        { k, T_DBL, offsetof(pe_params_t, f), 1, 1 }
#define DV(k, f, n)    { k, T_DBL, offsetof(pe_params_t, f), n, 1 }
#define DO(k, f)       { k, T_DBL, offsetof(pe_params_t, f), 1, 0 }
#define I(k, f)        { k, T_INT, offsetof(pe_params_t, f), 1, 0 }
#define S(k, f)        { k, T_STR, offsetof(pe_params_t, f), (int)sizeof(((pe_params_t*)0)->f), 0 }

static const pdesc_t table[] = {
	/* model: sep_geo_t */
	D("geo.m_per_tick_R", geo.m_per_tick_R),
	D("geo.m_per_tick_L", geo.m_per_tick_L),
	D("geo.B_eff", geo.B_eff),
	D("geo.stop_ticks", geo.stop_ticks),
	/* model: sep_agent_params_t */
	D("ag.gyro_scale", ag.gyro_scale),
	D("ag.dt_reject", ag.dt_reject),
	D("ag.dcount_reject", ag.dcount_reject),
	D("ag.loop_ms", ag.loop_ms),
	D("ag.reset_window", ag.reset_window),
	D("ag.k_reref", ag.k_reref),
	D("ag.n_still", ag.n_still),
	D("ag.tilt_rollover", ag.tilt_rollover),
	D("ag.t_gap_max", ag.t_gap_max),
	/* model: sep_ekf_params_t */
	D("prm.k_rho", prm.k_rho),
	D("prm.s2_ds_floor", prm.s2_ds_floor),
	D("prm.s2_theta", prm.s2_theta),
	D("prm.s2_w_enc", prm.s2_w_enc),
	D("prm.s2_bg", prm.s2_bg),
	D("prm.r_gyro", prm.r_gyro),
	D("prm.r_zaru", prm.r_zaru),
	D("prm.slip_thresh", prm.slip_thresh),
	D("prm.slip_gain", prm.slip_gain),
	D("prm.slip_cap", prm.slip_cap),
	D("prm.dt_min", prm.dt_min),
	D("prm.dt_max", prm.dt_max),
	DV("prm.P0", prm.P0, 5),
	/* application */
	S("io.raw_input", raw_input),
	I("io.raw_baud", raw_baud),
	I("io.replay", replay),
	S("io.gps_port", gps_port),
	I("io.gps_baud", gps_baud),
	S("io.udp_pose_host", udp_pose_host),
	I("io.udp_pose_port", udp_pose_port),
	S("io.udp_pose_host2", udp_pose_host2),
	I("io.udp_pose_port2", udp_pose_port2),
	I("io.udp_ctrl_port", udp_ctrl_port),
	I("io.pose_decimation", pose_decimation),
	S("io.log_dir", log_dir),
	DO("est.t_init_s", t_init_s),
	DO("est.stale_ms", stale_ms),
	I("est.gyro_yaw_axis", gyro_yaw_axis),
	I("rt.prio_ams", prio_ams),
	I("rt.prio_acq", prio_acq),
	I("rt.prio_fus", prio_fus),
	I("rt.prio_est", prio_est),
	I("rt.prio_com", prio_com),
	I("rt.prio_gps", prio_gps),
	I("rt.cpu_pipeline", cpu_pipeline),
	I("rt.cpu_gps", cpu_gps),
};
#define N_TABLE (sizeof(table) / sizeof(table[0]))

static void set_err(char* err, size_t errlen, const char* fmt, const char* a, const char* b)
{
	if (err != NULL && errlen > 0) {
		snprintf(err, errlen, fmt, a, b);
	}
}

void pe_params_defaults(pe_params_t* p)
{
	size_t i;
	memset(p, 0, sizeof(*p));
	for (i = 0; i < N_TABLE; i++) {
		if (table[i].required) {
			double* d = (double*)((char*)p + table[i].offset);
			int k;
			for (k = 0; k < table[i].count; k++) {
				d[k] = NAN;   /* "not set": pe_params_check refuses to start */
			}
		}
	}
	strcpy(p->raw_input, "/run/olympus/llc_raw");
	p->raw_baud = 115200;
	p->replay = 0;
	strcpy(p->gps_port, "/dev/ttyAMA0");
	p->gps_baud = 9600;
	strcpy(p->udp_pose_host, "127.0.0.1");
	p->udp_pose_port = 47001;
	p->udp_pose_host2[0] = '\0';
	p->udp_pose_port2 = 0;
	p->udp_ctrl_port = 47002;
	p->pose_decimation = 1;
	strcpy(p->log_dir, "/var/log/olympus-pose");
	p->t_init_s = 3.0;
	p->stale_ms = 100.0;
	p->gyro_yaw_axis = 2;
	p->prio_ams = 46;
	p->prio_acq = 44;
	p->prio_fus = 43;
	p->prio_est = 42;
	p->prio_com = 41;
	p->prio_gps = 30;
	p->cpu_pipeline = 3;
	p->cpu_gps = 2;
}

static char* trim(char* s)
{
	char* e;
	while (isspace((unsigned char)*s)) {
		s++;
	}
	e = s + strlen(s);
	while (e > s && isspace((unsigned char)e[-1])) {
		*--e = '\0';
	}
	return s;
}

int pe_params_parse_line(pe_params_t* p, const char* line, char* err, size_t errlen)
{
	char buf[512];
	char *key, *val, *eq, *hash;
	size_t i;

	if (strlen(line) >= sizeof(buf)) {
		set_err(err, errlen, "line too long: %.40s%s", line, "...");
		return -1;
	}
	strcpy(buf, line);
	hash = strchr(buf, '#');
	if (hash != NULL) {
		*hash = '\0';
	}
	key = trim(buf);
	if (*key == '\0') {
		return 0;
	}
	eq = strchr(key, '=');
	if (eq == NULL) {
		set_err(err, errlen, "missing '=' in \"%s\"%s", key, "");
		return -1;
	}
	*eq = '\0';
	val = trim(eq + 1);
	key = trim(key);

	for (i = 0; i < N_TABLE; i++) {
		if (strcmp(table[i].key, key) == 0) {
			break;
		}
	}
	if (i == N_TABLE) {
		set_err(err, errlen, "unknown key \"%s\"%s", key, "");
		return -1;
	}

	{
		const pdesc_t* d = &table[i];
		void* dst = (char*)p + d->offset;
		if (d->type == T_STR) {
			if (strlen(val) >= (size_t)d->count) {
				set_err(err, errlen, "value of %s too long%s", key, "");
				return -1;
			}
			strcpy((char*)dst, val);
		}
		else if (d->type == T_INT) {
			char* end;
			long v;
			errno = 0;
			v = strtol(val, &end, 10);
			if (errno != 0 || end == val || *trim(end) != '\0' || v < -2147483647L || v > 2147483647L) {
				set_err(err, errlen, "%s: \"%s\" is not an integer", key, val);
				return -1;
			}
			*(int*)dst = (int)v;
		}
		else {
			double* out = (double*)dst;
			const char* s = val;
			int k;
			for (k = 0; k < d->count; k++) {
				char* end;
				errno = 0;
				out[k] = strtod(s, &end);
				if (errno != 0 || end == s || !isfinite(out[k])) {
					set_err(err, errlen, "%s: bad number in \"%s\"", key, val);
					return -1;
				}
				s = end;
			}
			if (*trim((char*)s) != '\0') {
				set_err(err, errlen, "%s: too many values in \"%s\"", key, val);
				return -1;
			}
		}
	}
	return 0;
}

int pe_params_load(pe_params_t* p, const char* path, char* err, size_t errlen)
{
	FILE* f = fopen(path, "r");
	char line[512];
	int n = 0;
	if (f == NULL) {
		set_err(err, errlen, "cannot open %s%s", path, "");
		return -1;
	}
	while (fgets(line, sizeof(line), f) != NULL) {
		char sub[400];
		n++;
		if (pe_params_parse_line(p, line, sub, sizeof(sub)) != 0) {
			char where[64];
			snprintf(where, sizeof(where), "%s:%d", path, n);
			set_err(err, errlen, "%s: %s", where, sub);
			fclose(f);
			return -1;
		}
	}
	fclose(f);
	return 0;
}

int pe_params_check(const pe_params_t* p, char* err, size_t errlen)
{
	size_t i;
	for (i = 0; i < N_TABLE; i++) {
		if (table[i].required) {
			const double* d = (const double*)((const char*)p + table[i].offset);
			int k;
			for (k = 0; k < table[i].count; k++) {
				if (isnan(d[k])) {
					set_err(err, errlen, "missing model parameter %s%s", table[i].key,
						" (export it with export_pose_conf.m)");
					return -1;
				}
			}
		}
	}
	if (p->pose_decimation < 1 || p->gyro_yaw_axis < 0 || p->gyro_yaw_axis > 2) {
		set_err(err, errlen, "io.pose_decimation must be >= 1 and est.gyro_yaw_axis 0..2%s%s", "", "");
		return -1;
	}
	return 0;
}

unsigned long pe_params_dump(const pe_params_t* p, char* buf, size_t buflen)
{
	size_t i, used = 0;
	unsigned long h = 2166136261UL;
	if (buflen == 0) {
		return 0;
	}
	buf[0] = '\0';
	for (i = 0; i < N_TABLE; i++) {
		const pdesc_t* d = &table[i];
		const void* src = (const char*)p + d->offset;
		char line[400];
		int n;
		if (d->type == T_STR) {
			n = snprintf(line, sizeof(line), "%s = %s\n", d->key, (const char*)src);
		}
		else if (d->type == T_INT) {
			n = snprintf(line, sizeof(line), "%s = %d\n", d->key, *(const int*)src);
		}
		else {
			const double* v = (const double*)src;
			int k, m = snprintf(line, sizeof(line), "%s =", d->key);
			for (k = 0; k < d->count && m > 0 && (size_t)m < sizeof(line); k++) {
				m += snprintf(line + m, sizeof(line) - (size_t)m, " %.17g", v[k]);
			}
			if (m > 0 && (size_t)m < sizeof(line) - 1) {
				line[m++] = '\n';
				line[m] = '\0';
			}
			n = m;
		}
		if (n > 0 && used + (size_t)n < buflen) {
			memcpy(buf + used, line, (size_t)n + 1);
			used += (size_t)n;
		}
	}
	for (i = 0; i < used; i++) {
		h ^= (unsigned char)buf[i];
		h *= 16777619UL;
		h &= 0xFFFFFFFFUL;
	}
	return h;
}
