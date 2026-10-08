/*
 * raw_parser.h - Parser of the LLC sensor line (DESIGN.md §3.1, §7.1).
 *
 * Two frame versions are accepted:
 *
 *   v2.20: [@<t_rx_ns> ]RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>
 *   v3.0:  [@<t_rx_ns> ]RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>:<temp>:<imu_ok>
 *
 * v3.0 (MPU-9250 firmware, 2026-10) appends the raw die temperature and an
 * IMU-valid flag at the END, so the first nine columns keep their position.
 * In v3.0 the frame is ALWAYS sent: if the IMU did not answer, the inertial
 * fields are 0 and imu_ok = 0, so the HLC can tell "IMU down" from "frame
 * lost". v2.20 frames only exist when the IMU answered (imu_ok is implied 1).
 *
 * The optional "@<t_rx_ns> " prefix is added by llcmux (arrival time on the
 * HLC monotonic clock) and by recorded replay files. Syntax only: exact field
 * count, base-10 integers in range, imu_ok in {0, 1}. Physical plausibility
 * (time and count jumps) is the fusion agent's job.
 */
#ifndef PE_RAW_PARSER_H
#define PE_RAW_PARSER_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	PE_LINE_RAW = 0,     /* a valid RAW frame (v2.20 or v3.0) */
	PE_LINE_TLM,         /* a TLM: line (counted, not used) */
	PE_LINE_OTHER,       /* any other line (ACK, INFO, ...) */
	PE_LINE_BAD          /* starts like RAW: but is malformed */
} pe_line_kind_t;

typedef struct {
	int      has_ts;      /* the line carried an @t_rx_ns prefix */
	uint64_t t_rx_ns;
	int      version;     /* 2 (9 fields, v2.20) or 3 (11 fields, v3.0) */
	uint32_t tick;
	int16_t  acc[3], gyr[3];
	int32_t  enc_l, enc_r;
	int16_t  temp_raw;    /* v3.0 only; 0 in v2.20. deg C = temp_raw/333.87 + 21 */
	int      imu_ok;      /* v3.0 field; always 1 for v2.20 */
} pe_raw_frame_t;

/* line: without the trailing '\n' (a trailing '\r' is tolerated). */
pe_line_kind_t pe_raw_parse(const char* line, size_t len, pe_raw_frame_t* out);

#endif
