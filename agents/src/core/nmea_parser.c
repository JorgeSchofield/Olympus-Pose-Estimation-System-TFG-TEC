/*
 * nmea_parser.c - Minimal, strict NMEA 0183 parser (RMC, GGA). Portable C99.
 */
#include "nmea_parser.h"

#include <stdlib.h>
#include <string.h>

#define MAX_FIELDS 20

static int hexval(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}

/* Copies field i of the comma-separated body into buf (empty if absent). */
static void field(const char* const* f, const size_t* fl, int nf, int i, char* buf, size_t bl)
{
	size_t n = 0;
	if (i < nf) {
		n = fl[i] < bl - 1 ? fl[i] : bl - 1;
		memcpy(buf, f[i], n);
	}
	buf[n] = '\0';
}

/* "hhmmss.sss" -> ms of day; -1 if malformed. */
static long parse_time(const char* s)
{
	double t;
	char* end;
	long hh, mm;
	double ss;
	if (strlen(s) < 6) {
		return -1;
	}
	t = strtod(s, &end);
	if (*end != '\0' || t < 0) {
		return -1;
	}
	hh = (long)(t / 10000.0);
	mm = (long)((t - hh * 10000.0) / 100.0);
	ss = t - hh * 10000.0 - mm * 100.0;
	if (hh > 23 || mm > 59 || ss >= 61.0) {
		return -1;
	}
	return (long)(hh * 3600000L + mm * 60000L + (long)(ss * 1000.0 + 0.5));
}

/* "ddmm.mmmm" / "dddmm.mmmm" + hemisphere -> signed degrees. */
static int parse_latlon(const char* v, const char* hemi, double* out)
{
	char* end;
	double raw, deg, min;
	if (*v == '\0' || *hemi == '\0') {
		return -1;
	}
	raw = strtod(v, &end);
	if (*end != '\0' || raw < 0) {
		return -1;
	}
	deg = (double)(long)(raw / 100.0);
	min = raw - deg * 100.0;
	if (min >= 60.0) {
		return -1;
	}
	*out = deg + min / 60.0;
	if (*hemi == 'S' || *hemi == 'W') {
		*out = -*out;
	}
	else if (*hemi != 'N' && *hemi != 'E') {
		return -1;
	}
	return 0;
}

static double num_or(const char* s, double dflt)
{
	char* end;
	double v;
	if (*s == '\0') {
		return dflt;
	}
	v = strtod(s, &end);
	return *end == '\0' ? v : dflt;
}

pe_nmea_kind_t pe_nmea_parse(const char* line, size_t len, pe_nmea_t* out)
{
	const char* f[MAX_FIELDS];
	size_t fl[MAX_FIELDS];
	int nf = 0;
	size_t star, i;
	unsigned sum = 0;
	int h1, h2;
	char a[32], b[32];

	while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
		len--;
	}
	if (len < 9 || line[0] != '$') {
		return PE_NMEA_BAD;
	}
	star = len;
	for (i = 1; i < len; i++) {
		if (line[i] == '*') {
			star = i;
			break;
		}
		sum ^= (unsigned char)line[i];
	}
	if (star + 3 != len) {
		return PE_NMEA_BAD;
	}
	h1 = hexval(line[star + 1]);
	h2 = hexval(line[star + 2]);
	if (h1 < 0 || h2 < 0 || (unsigned)(h1 * 16 + h2) != (sum & 0xFFu)) {
		return PE_NMEA_BAD;
	}

	/* split "$TTSSS,f1,f2,...,fn" (fields keep their position, may be empty) */
	{
		size_t s = 1;
		for (i = 1; i <= star; i++) {
			if (i == star || line[i] == ',') {
				if (nf >= MAX_FIELDS) {
					return PE_NMEA_BAD;
				}
				f[nf] = line + s;
				fl[nf] = i - s;
				nf++;
				s = i + 1;
			}
		}
	}
	if (fl[0] != 5) {
		return PE_NMEA_OTHER;
	}

	if (memcmp(f[0] + 2, "RMC", 3) == 0) {
		long t;
		/* 1 time, 2 status, 3 lat, 4 N/S, 5 lon, 6 E/W, 7 speed kn, 8 course */
		field(f, fl, nf, 1, a, sizeof(a));
		t = parse_time(a);
		if (t < 0) {
			return PE_NMEA_BAD;
		}
		out->utc_ms_of_day = (uint32_t)t;
		field(f, fl, nf, 2, a, sizeof(a));
		out->rmc_valid = (a[0] == 'A');
		if (out->rmc_valid) {
			field(f, fl, nf, 3, a, sizeof(a));
			field(f, fl, nf, 4, b, sizeof(b));
			if (parse_latlon(a, b, &out->lat_deg) != 0) {
				return PE_NMEA_BAD;
			}
			field(f, fl, nf, 5, a, sizeof(a));
			field(f, fl, nf, 6, b, sizeof(b));
			if (parse_latlon(a, b, &out->lon_deg) != 0) {
				return PE_NMEA_BAD;
			}
		}
		field(f, fl, nf, 7, a, sizeof(a));
		out->speed_mps = num_or(a, 0.0) * 0.514444;
		field(f, fl, nf, 8, a, sizeof(a));
		out->course_deg = num_or(a, 0.0);
		return PE_NMEA_RMC;
	}
	if (memcmp(f[0] + 2, "GGA", 3) == 0) {
		long t;
		/* 1 time, 2 lat, 3 N/S, 4 lon, 5 E/W, 6 quality, 7 sats, 8 hdop, 9 alt */
		field(f, fl, nf, 1, a, sizeof(a));
		t = parse_time(a);
		if (t < 0) {
			return PE_NMEA_BAD;
		}
		out->utc_ms_of_day = (uint32_t)t;
		field(f, fl, nf, 6, a, sizeof(a));
		out->fix_quality = (int)num_or(a, 0.0);
		field(f, fl, nf, 7, a, sizeof(a));
		out->n_sats = (int)num_or(a, 0.0);
		field(f, fl, nf, 8, a, sizeof(a));
		out->hdop = num_or(a, 99.9);
		field(f, fl, nf, 9, a, sizeof(a));
		out->alt_m = num_or(a, 0.0);
		if (out->fix_quality > 0) {
			field(f, fl, nf, 2, a, sizeof(a));
			field(f, fl, nf, 3, b, sizeof(b));
			if (parse_latlon(a, b, &out->lat_deg) != 0) {
				return PE_NMEA_BAD;
			}
			field(f, fl, nf, 4, a, sizeof(a));
			field(f, fl, nf, 5, b, sizeof(b));
			if (parse_latlon(a, b, &out->lon_deg) != 0) {
				return PE_NMEA_BAD;
			}
		}
		return PE_NMEA_GGA;
	}
	return PE_NMEA_OTHER;
}
