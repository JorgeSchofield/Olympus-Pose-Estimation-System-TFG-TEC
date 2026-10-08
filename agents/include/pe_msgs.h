/*
 * pe_msgs.h - Messages exchanged by the pose-estimation agents (DESIGN.md §8).
 *
 * Every message is a fixed-size struct that lives in its producer's static
 * buffer pool; the CMAES mailbox only carries a pointer to it (MsgObj.content).
 * Rules (DESIGN.md §8.1, decision D4):
 *   - data is sent with timeout 0 (a full mailbox drops the NEW message);
 *   - each producer rotates PE_POOL_DEPTH buffers and advances only after a
 *     successful send, so a buffer is never rewritten while the consumer may
 *     still be reading it;
 *   - hdr.seq grows by one per successful send; consumers count the gaps.
 *
 * Everything that crosses the fusion -> estimation mailbox is CUMULATIVE
 * (decision D2): a dropped message costs time resolution, never distance.
 */
#ifndef PE_MSGS_H
#define PE_MSGS_H

#include <stdint.h>
#include "sep_fusion_init_types.h"   /* generated: sep_odom_t */

#define PE_POOL_DEPTH 4   /* >= 3 for a depth-1 mailbox, plus one of margin */

typedef enum {
	PE_MSG_RAW = 1,       /* acquisition -> fusion */
	PE_MSG_ODOM,          /* fusion -> estimation */
	PE_MSG_POSE,          /* estimation -> communication */
	PE_MSG_GPS,           /* GPS -> communication */
	PE_MSG_CTRL           /* communication -> estimation */
} pe_msg_kind_t;

typedef struct {
	uint16_t kind;        /* pe_msg_kind_t */
	uint16_t version;
	uint32_t seq;         /* per-link sequence, +1 per successful send */
	uint64_t t_rx_ns;     /* HLC CLOCK_MONOTONIC arrival of the newest LLC frame */
} pe_hdr_t;

/* Cumulative link statistics, measured by the consumer. */
typedef struct {
	uint32_t delivered;        /* messages received on this link */
	uint32_t lost;             /* sum of sequence gaps */
	uint32_t max_consec_lost;  /* largest single gap */
	uint32_t producer_drops;   /* sends that failed, reported by the producer */
} pe_link_stats_t;

/* ------------------------------------------------------------------ RAW */
/* Acquisition -> fusion. Accumulators UNCONVERTED (PE-RF-008). */
enum { PE_RAW_IMU_VALID = 1u << 0, PE_RAW_RX_TS_LOCAL = 1u << 1 };

typedef struct {
	pe_hdr_t hdr;
	uint32_t t_llc_ms;           /* LLC tick as sent */
	int32_t  enc_l, enc_r;       /* side accumulators, raw */
	int16_t  acc[3], gyr[3];     /* raw IMU LSB */
	uint8_t  flags;              /* PE_RAW_* */
	uint8_t  reserved;
	uint16_t line_len;
	uint32_t frames_ok, frames_bad, tlm_lines;  /* acquisition counters */
	uint32_t producer_drops;
} pe_raw_msg_t;

/* ----------------------------------------------------------------- ODOM */
/* Fusion -> estimation. odom is the generated message (all cumulative). */
typedef struct {
	pe_hdr_t        hdr;
	sep_odom_t      odom;
	pe_link_stats_t link_acq;    /* acquisition -> fusion, measured by fusion */
	uint32_t        frames_ok, frames_bad;
	uint32_t        rejects, rerefs, llc_resets;
	uint32_t        producer_drops;
} pe_odom_msg_t;

/* ----------------------------------------------------------------- POSE */
typedef enum { PE_MODE_WAIT_DATA = 0, PE_MODE_INITIALIZING, PE_MODE_RUNNING } pe_mode_t;

enum {
	PE_ST_VALID       = 1u << 0,
	PE_ST_STILL       = 1u << 1,
	PE_ST_SLIP        = 1u << 2,
	PE_ST_STALE       = 1u << 3,
	PE_ST_GAP         = 1u << 4,
	PE_ST_IMU_INVALID = 1u << 5,
	PE_ST_ROLLOVER    = 1u << 6,
	PE_ST_LLC_RESET   = 1u << 7,
	PE_ST_RESUMED     = 1u << 8,
	PE_ST_REREF       = 1u << 9
};

typedef struct {
	pe_hdr_t        hdr;          /* t_rx_ns of the newest input frame */
	uint64_t        t_est_ns;     /* end of the EKF step */
	double          x[5];         /* px, py, theta, omega, b_omega */
	double          P[25];        /* full covariance, column-major (MATLAB), symmetric */
	double          s_m;          /* distance travelled (cumulative S) */
	double          diag[3];      /* slip metric delta, Q gain gamma, NIS */
	double          t_cum_ms;     /* continuous LLC time */
	uint32_t        t_llc_ms;
	uint16_t        mode;         /* pe_mode_t */
	uint16_t        status;       /* PE_ST_* */
	pe_link_stats_t link_acq, link_fus;
	uint32_t        frames_ok, frames_bad;
	uint32_t        rejects, n_substeps, n_gaps;
	uint32_t        producer_drops;
} pe_pose_msg_t;

/* ------------------------------------------------------------------ GPS */
typedef struct {
	pe_hdr_t hdr;                 /* t_rx_ns = end of the last sentence of the epoch */
	double   lat_deg, lon_deg, alt_m;
	double   hdop, speed_mps, course_deg;
	uint32_t utc_ms_of_day;
	uint8_t  fix_quality, n_sats, valid, reserved;
	uint32_t producer_drops;
} pe_gps_msg_t;

/* ----------------------------------------------------------------- CTRL */
typedef enum { PE_CTRL_RESET_POSE = 1 } pe_ctrl_cmd_t;

typedef struct {
	pe_hdr_t hdr;
	uint16_t cmd;
	uint16_t reserved;
	double   arg[3];              /* RESET_POSE: x, y, theta */
} pe_ctrl_msg_t;

/* Consumer-side link accounting. */
static inline void pe_link_account(pe_link_stats_t* s, uint32_t* last_seq, uint32_t seq)
{
	if (s->delivered > 0 || *last_seq != 0) {
		uint32_t gap = seq - *last_seq - 1u;      /* wrap-safe */
		if (gap > 0 && gap < 0x80000000u) {
			s->lost += gap;
			if (gap > s->max_consec_lost) {
				s->max_consec_lost = gap;
			}
		}
	}
	*last_seq = seq;
	s->delivered++;
}

#endif /* PE_MSGS_H */
