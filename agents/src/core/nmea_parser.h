/*
 * nmea_parser.h - NMEA 0183 parser for the GPS agent (DESIGN.md §7.5).
 *
 * Accepts $xxRMC and $xxGGA (any talker: GP, GN, GL, ...) with a valid
 * checksum. An RMC and a GGA with the same UTC time form one fix epoch.
 */
#ifndef PE_NMEA_PARSER_H
#define PE_NMEA_PARSER_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	PE_NMEA_RMC = 0,
	PE_NMEA_GGA,
	PE_NMEA_OTHER,      /* valid checksum, sentence not used */
	PE_NMEA_BAD         /* no '$', bad checksum or malformed field */
} pe_nmea_kind_t;

typedef struct {
	uint32_t utc_ms_of_day;
	int      rmc_valid;          /* RMC status 'A' */
	int      fix_quality;        /* GGA: 0 none, 1 GPS, 2 DGPS, ... */
	int      n_sats;
	double   lat_deg, lon_deg;   /* signed: north and east positive */
	double   alt_m;
	double   hdop;
	double   speed_mps;
	double   course_deg;
} pe_nmea_t;

/* line without "\r\n". Fills only the fields the sentence carries. */
pe_nmea_kind_t pe_nmea_parse(const char* line, size_t len, pe_nmea_t* out);

#endif
