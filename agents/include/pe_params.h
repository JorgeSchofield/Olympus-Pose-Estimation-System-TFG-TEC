/*
 * pe_params.h - Run-time parameters of the pose-estimation application
 * (DESIGN.md §12).
 *
 * Two groups:
 *   - model parameters (geo.*, ag.*, prm.*): the arguments of the generated
 *     core, exported from the MATLAB model by export_pose_conf.m. All are
 *     REQUIRED: the application refuses to start if one is missing, so the C
 *     code can never run with values the model did not produce.
 *   - application parameters (io.*, rt.*, est.*): ports, logging, scheduling.
 *     All have defaults.
 *
 * File format: one "key = value [value ...]" per line, '#' starts a comment.
 * Unknown keys are an error (a typo must not silently keep a default).
 */
#ifndef PE_PARAMS_H
#define PE_PARAMS_H

#include <stddef.h>
#include "sep_fusion_init_types.h"   /* generated: sep_geo_t, sep_agent_params_t, sep_ekf_params_t */

#define PE_PATH_MAX 256

typedef struct {
	/* model (required) */
	sep_geo_t          geo;
	sep_agent_params_t ag;
	sep_ekf_params_t   prm;

	/* inputs and outputs */
	char   raw_input[PE_PATH_MAX];   /* pty from llcmux, a serial port, or a replay file */
	int    raw_baud;                 /* used only if raw_input is a real serial port */
	int    replay;                   /* 1: raw_input is a recorded file, no drops, exit at EOF */
	char   gps_port[PE_PATH_MAX];    /* "" disables the GPS agent */
	int    gps_baud;
	char   udp_pose_host[64];
	int    udp_pose_port;
	char   udp_pose_host2[64];       /* optional second destination, "" = none */
	int    udp_pose_port2;
	int    udp_ctrl_port;            /* 0 disables the control port */
	int    pose_decimation;          /* send every Nth estimate (1 = all) */
	char   log_dir[PE_PATH_MAX];     /* a run sub-folder is created inside */

	/* estimation agent (application side) */
	double t_init_s;                 /* rest time before the pose is declared valid */
	double stale_ms;                 /* no input for this long -> STALE */
	int    gyro_yaw_axis;            /* 0 = x, 1 = y, 2 = z */

	/* real-time scheduling (SCHED_FIFO priorities, CPUs; -1 = do not pin) */
	int    prio_ams, prio_acq, prio_fus, prio_est, prio_com, prio_gps;
	int    cpu_pipeline, cpu_gps;
} pe_params_t;

/* Fills the application defaults and marks every model parameter as unset. */
void pe_params_defaults(pe_params_t* p);

/* Parses a file on top of the current values. Returns 0 on success; on
 * failure returns -1 and writes a one-line reason to err. */
int pe_params_load(pe_params_t* p, const char* path, char* err, size_t errlen);

/* Parses one "key = value" line (exposed for the unit tests). Returns 0, or -1
 * with a reason in err. Blank and comment lines return 0. */
int pe_params_parse_line(pe_params_t* p, const char* line, char* err, size_t errlen);

/* Returns 0 if every required model parameter was set, else -1 and the name
 * of the first missing one in err. */
int pe_params_check(const pe_params_t* p, char* err, size_t errlen);

/* Writes every parameter as "key = value" (run header, DESIGN.md §11.2).
 * Returns the FNV-1a hash of the text, to identify the configuration. */
unsigned long pe_params_dump(const pe_params_t* p, char* buf, size_t buflen);

#endif /* PE_PARAMS_H */
