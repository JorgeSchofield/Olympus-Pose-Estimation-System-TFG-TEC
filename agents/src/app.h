/*
 * app.h - Shared declarations of the olympus-pose agents (DESIGN.md §4, §7).
 *
 *   acquisition -> fusion -> estimation -> communication <- GPS
 *
 * Each agent is a CMAES CyclicBehaviour whose action() handles ONE input and
 * returns, so suspension is checked every iteration. Data messages are sent
 * with timeout 0 (in replay mode with MAES_MAX_DELAY, so a recorded run is
 * processed without drops and is reproducible).
 */
#ifndef PE_APP_H
#define PE_APP_H

#include <CMAES.h>

#include "pe_msgs.h"
#include "pe_params.h"

#define PE_APP_VERSION "0.1.0"

/* Read-only after start-up. */
extern pe_params_t g_params;

/* CMAES objects (defined in main.c). */
extern sysVars        env;
extern Agent_Platform Platform;
extern MAESAgent      ag_acq, ag_fus, ag_est, ag_com, ag_gps;

/* Log streams (pe_log_open ids, -1 if unavailable). */
extern int g_log_pose, g_log_gps, g_log_stats, g_log_events;

/* Agent entry points (behaviour wrappers). */
void acquisition_main(void* arg);
void fusion_main(void* arg);
void estimation_main(void* arg);
void communication_main(void* arg);
void gps_main(void* arg);

/* Timeout for data sends: 0 live, forever in replay mode. */
static inline MAESTickType_t pe_send_timeout(void)
{
	return g_params.replay ? MAES_MAX_DELAY : 0;
}

/* Sends one message; returns 1 on success. */
static inline int pe_send(Agent_Msg* m, Agent_AID to, void* content, MSG_TYPE type,
                          MAESTickType_t timeout)
{
	m->set_msg_type(m, type);
	m->set_msg_content(m, (char*)content);
	return m->send(m, to, timeout) == NO_ERRORS;
}

/* Logs an event line: "<t_ns>,<agent>,<text>". */
void pe_event(const char* agent, const char* fmt, ...);

/* Asks the supervisor to shut down cleanly (used at the end of a replay). */
void pe_request_stop(void);

#endif
