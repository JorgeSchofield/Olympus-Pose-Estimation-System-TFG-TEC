/*
 * raw_parser.c - Strict parser of the LLC RAW: line. Portable C99.
 *
 * Written by hand (no sscanf): sscanf accepts leading '+', whitespace and
 * overflows silently, and the frame has no CRC, so the parser is the only
 * place where a malformed line can be refused (DESIGN.md §13).
 */
#include "raw_parser.h"

#include <string.h>

/* Parses a signed base-10 integer in [lo, hi] from s[0..n). The whole range
 * must be digits (one optional leading '-'), at most 11 characters. */
static int parse_int(const char* s, size_t n, long long lo, long long hi, long long* out)
{
	size_t i = 0;
	int neg = 0;
	long long v = 0;
	if (n == 0 || n > 11) {
		return -1;
	}
	if (s[0] == '-') {
		neg = 1;
		i = 1;
		if (n == 1) {
			return -1;
		}
	}
	for (; i < n; i++) {
		if (s[i] < '0' || s[i] > '9') {
			return -1;
		}
		v = v * 10 + (s[i] - '0');
	}
	if (neg) {
		v = -v;
	}
	if (v < lo || v > hi) {
		return -1;
	}
	*out = v;
	return 0;
}

static int parse_u64(const char* s, size_t n, uint64_t* out)
{
	size_t i;
	uint64_t v = 0;
	if (n == 0 || n > 19) {
		return -1;
	}
	for (i = 0; i < n; i++) {
		if (s[i] < '0' || s[i] > '9') {
			return -1;
		}
		v = v * 10u + (uint64_t)(s[i] - '0');
	}
	*out = v;
	return 0;
}

pe_line_kind_t pe_raw_parse(const char* line, size_t len, pe_raw_frame_t* out)
{
	/* tick, ax, ay, az, gx, gy, gz, encL, encR, [temp, imu_ok] */
	static const long long lo[11] = { 0, -32768, -32768, -32768, -32768, -32768, -32768,
		-2147483648LL, -2147483648LL, -32768, 0 };
	static const long long hi[11] = { 4294967295LL, 32767, 32767, 32767, 32767, 32767, 32767,
		2147483647LL, 2147483647LL, 32767, 1 };
	long long v[11];
	size_t pos, start;
	int field, nfields;

	memset(out, 0, sizeof(*out));
	if (len > 0 && line[len - 1] == '\r') {
		len--;
	}

	/* optional "@<t_rx_ns> " prefix */
	pos = 0;
	if (len > 0 && line[0] == '@') {
		size_t sp = 1;
		while (sp < len && line[sp] != ' ') {
			sp++;
		}
		if (sp >= len || parse_u64(line + 1, sp - 1, &out->t_rx_ns) != 0) {
			return PE_LINE_BAD;
		}
		out->has_ts = 1;
		pos = sp + 1;
	}

	if (len - pos >= 4 && memcmp(line + pos, "TLM:", 4) == 0) {
		return PE_LINE_TLM;
	}
	if (len - pos < 4 || memcmp(line + pos, "RAW:", 4) != 0) {
		return PE_LINE_OTHER;
	}
	pos += 4;

	/* the field count decides the version: 9 (v2.20) or 11 (v3.0) */
	nfields = 1;
	for (start = pos; start < len; start++) {
		if (line[start] == ':') {
			nfields++;
		}
	}
	if (nfields != 9 && nfields != 11) {
		return PE_LINE_BAD;
	}

	for (field = 0; field < nfields; field++) {
		start = pos;
		while (pos < len && line[pos] != ':') {
			pos++;
		}
		if (parse_int(line + start, pos - start, lo[field], hi[field], &v[field]) != 0) {
			return PE_LINE_BAD;
		}
		if (field < nfields - 1) {
			pos++;                           /* skip ':' (count already checked) */
		}
	}
	if (pos != len) {
		return PE_LINE_BAD;
	}

	out->version = nfields == 11 ? 3 : 2;
	out->tick = (uint32_t)v[0];
	out->acc[0] = (int16_t)v[1];
	out->acc[1] = (int16_t)v[2];
	out->acc[2] = (int16_t)v[3];
	out->gyr[0] = (int16_t)v[4];
	out->gyr[1] = (int16_t)v[5];
	out->gyr[2] = (int16_t)v[6];
	out->enc_l = (int32_t)v[7];
	out->enc_r = (int32_t)v[8];
	if (nfields == 11) {
		out->temp_raw = (int16_t)v[9];
		out->imu_ok = (int)v[10];
	}
	else {
		out->temp_raw = 0;
		out->imu_ok = 1;     /* v2.20 only sends RAW: when the IMU read succeeded */
	}
	return PE_LINE_RAW;
}
