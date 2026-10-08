/*
 * raw_parser.h - Parser of the LLC sensor line (DESIGN.md §3.1, §7.1).
 *
 *   [@<t_rx_ns> ]RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>
 *
 * The optional "@<t_rx_ns> " prefix is added by llcmux (arrival time on the
 * HLC monotonic clock) and by recorded replay files. Syntax only: exactly 9
 * base-10 integers in range, nothing else. Physical plausibility (time and
 * count jumps) is the fusion agent's job.
 */
#ifndef PE_RAW_PARSER_H
#define PE_RAW_PARSER_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	PE_LINE_RAW = 0,     /* a valid RAW frame */
	PE_LINE_TLM,         /* a TLM: line (counted, not used) */
	PE_LINE_OTHER,       /* any other line (ACK, INFO, ...) */
	PE_LINE_BAD          /* starts like RAW: but is malformed */
} pe_line_kind_t;

typedef struct {
	int      has_ts;      /* the line carried an @t_rx_ns prefix */
	uint64_t t_rx_ns;
	uint32_t tick;
	int16_t  acc[3], gyr[3];
	int32_t  enc_l, enc_r;
} pe_raw_frame_t;

/* line: without the trailing '\n' (a trailing '\r' is tolerated). */
pe_line_kind_t pe_raw_parse(const char* line, size_t len, pe_raw_frame_t* out);

#endif
