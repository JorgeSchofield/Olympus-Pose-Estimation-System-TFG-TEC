/*
 * main.c - olympus-pose supervisor (DESIGN.md §7.6).
 *
 *   olympus-pose [-c pose.conf] [-r replay_file] [--check]
 *
 * Loads and validates the configuration (every model parameter must come from
 * export_pose_conf.m), starts the asynchronous logs, builds the CMAES platform
 * with the five agents and then only waits for SIGINT/SIGTERM to shut down
 * cleanly. The main thread is not an agent: it never sends CMAES messages.
 */
#include "app.h"

#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "port.h"
#include "sep_fusion_init_initialize.h"

pe_params_t    g_params;
sysVars        env;
Agent_Platform Platform;
MAESAgent      ag_acq, ag_fus, ag_est, ag_com, ag_gps;
int            g_log_pose = -1, g_log_gps = -1, g_log_stats = -1, g_log_events = -1;

static pthread_t main_thread;

void pe_event(const char* agent, const char* fmt, ...)
{
	char text[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(text, sizeof(text), fmt, ap);
	va_end(ap);
	pe_log_printf(g_log_events, "%llu,%s,%s\n", (unsigned long long)pe_now_ns(), agent, text);
	if (g_params.replay == 0 || strcmp(agent, "acquisition") == 0) {
		fprintf(stderr, "[%s] %s\n", agent, text);
	}
}

void pe_request_stop(void)
{
	pthread_kill(main_thread, SIGTERM);
}

static void usage(const char* argv0)
{
	fprintf(stderr,
		"usage: %s [-c pose.conf] [-r replay_file] [--check]\n"
		"  -c FILE   configuration (default /etc/olympus-pose/pose.conf)\n"
		"  -r FILE   process a recorded stream (llcmux --record) without drops, then exit\n"
		"  --check   validate the configuration and exit\n", argv0);
}

/* SCHED_FIFO priority -> CMAES ordinal (CMAES maps it to min + ordinal). */
static MAESUBaseType_t ordinal(int fifo_prio)
{
	return fifo_prio > 1 ? (MAESUBaseType_t)(fifo_prio - 1) : 0u;
}

static void write_header(const char* conf_path, const char* run_dir)
{
	static char dump[8192];
	unsigned long hash = pe_params_dump(&g_params, dump, sizeof(dump));
	int h = pe_log_open("header.txt");
	pe_log_printf(h, "olympus-pose %s\nconfig file: %s\nrun dir: %s\nconfig hash: %08lx\nmode: %s\n\n",
		PE_APP_VERSION, conf_path, run_dir, hash, g_params.replay ? "replay" : "live");
	pe_log_write(h, dump, strlen(dump));
	fprintf(stderr, "olympus-pose %s: logging to %s (config hash %08lx)\n", PE_APP_VERSION, run_dir, hash);
}

int main(int argc, char** argv)
{
	const char* conf = "/etc/olympus-pose/pose.conf";
	const char* replay = NULL;
	int check_only = 0, i;
	char err[512], run_dir[512];
	sigset_t set;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
			conf = argv[++i];
		}
		else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
			replay = argv[++i];
		}
		else if (strcmp(argv[i], "--check") == 0) {
			check_only = 1;
		}
		else {
			usage(argv[0]);
			return 2;
		}
	}

	pe_params_defaults(&g_params);
	if (pe_params_load(&g_params, conf, err, sizeof(err)) != 0 ||
		pe_params_check(&g_params, err, sizeof(err)) != 0) {
		fprintf(stderr, "olympus-pose: configuration error: %s\n", err);
		return 1;
	}
	if (replay != NULL) {
		snprintf(g_params.raw_input, sizeof(g_params.raw_input), "%s", replay);
		g_params.replay = 1;
	}
	if (check_only) {
		printf("configuration OK\n");
		return 0;
	}

	if (pe_log_start(g_params.log_dir, run_dir, sizeof(run_dir)) != 0) {
		fprintf(stderr, "olympus-pose: cannot create a log folder in %s\n", g_params.log_dir);
		return 1;
	}
	pe_log_set_blocking(g_params.replay);
	write_header(conf, run_dir);
	g_log_events = pe_log_open("events.csv");
	g_log_pose = pe_log_open("pose.csv");
	g_log_gps = pe_log_open("gps.csv");
	g_log_stats = pe_log_open("stats.csv");
	pe_log_printf(g_log_events, "t_ns,agent,event\n");
	pe_log_printf(g_log_pose,
		"t_pub_ns,t_rx_ns,t_est_ns,t_llc_ms,t_cum_ms,mode,status,"
		"x_m,y_m,theta_rad,omega_rad_s,bias_rad_s,"
		"P_xx,P_yy,P_xy,P_thth,P_ww,P_bb,"
		"s_m,delta,gamma,nis,latency_ms,"
		"acq_delivered,acq_lost,acq_max_gap,acq_drops,fus_delivered,fus_lost,fus_max_gap,fus_drops,"
		"frames_ok,frames_bad,rejects,substeps,gaps\n");
	pe_log_printf(g_log_gps, "t_rx_ns,utc_ms,lat_deg,lon_deg,alt_m,quality,sats,hdop,speed_mps,course_deg,valid\n");
	pe_log_printf(g_log_stats,
		"t_ns,t_run_s,rss_kb,cpu_s,poses,lat_p50_ms,lat_p95_ms,lat_max_ms,interval_p95_ms,interval_max_ms,"
		"llc_clock_scale,acq_delivered,acq_lost,acq_max_gap,acq_drops,fus_delivered,fus_lost,fus_max_gap,"
		"fus_drops,est_delivered,est_lost,est_max_gap,est_drops,gps_fixes,gps_valid,log_drops,"
		"st_acq,st_fus,st_est,st_com,st_gps\n");

	/* Generated core: one-time initialisation (non-finite constants). */
	sep_fusion_init_initialize();

	if (pe_lock_memory() != 0) {
		pe_event("supervisor", "mlockall failed (not root?): page faults may add latency");
	}

	/* Agents inherit this mask, so only the main thread receives the signals. */
	main_thread = pthread_self();
	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGTERM);
	pthread_sigmask(SIG_BLOCK, &set, NULL);
	signal(SIGPIPE, SIG_IGN);

	MAES_SetAMSPriority(g_params.prio_ams);
	ConstructorSysVars(&env);
	ConstructorAgent_Platform(&Platform, &env);
	ConstructorAgente(&ag_acq);
	ConstructorAgente(&ag_fus);
	ConstructorAgente(&ag_est);
	ConstructorAgente(&ag_com);
	ConstructorAgente(&ag_gps);
	ag_acq.Iniciador(&ag_acq, "acquisition", ordinal(g_params.prio_acq), 0);
	ag_fus.Iniciador(&ag_fus, "fusion", ordinal(g_params.prio_fus), 0);
	ag_est.Iniciador(&ag_est, "estimation", ordinal(g_params.prio_est), 0);
	ag_com.Iniciador(&ag_com, "communication", ordinal(g_params.prio_com), 0);
	ag_gps.Iniciador(&ag_gps, "gps", ordinal(g_params.prio_gps), 0);
	Platform.Agent_Platform(&Platform, "olympus_pose");
	Platform.agent_init(&Platform, &ag_acq, &acquisition_main);
	Platform.agent_init(&Platform, &ag_fus, &fusion_main);
	Platform.agent_init(&Platform, &ag_est, &estimation_main);
	Platform.agent_init(&Platform, &ag_com, &communication_main);
	Platform.agent_init(&Platform, &ag_gps, &gps_main);
	/* boot() is declared bool but returns an ERROR_CODE (NO_ERRORS == 0) in
	 * every backend, so its value is not a reliable success flag: ignore it. */
	(void)Platform.boot(&Platform);
	pe_event("supervisor", "5 agents registered and running (input %s)", g_params.raw_input);

	for (;;) {
		int sig = sigwaitinfo(&set, NULL);
		if (sig == SIGINT || sig == SIGTERM) {
			pe_event("supervisor", "%s received: shutting down", sig == SIGINT ? "SIGINT" : "SIGTERM");
			break;
		}
	}
	pe_log_stop();
	return 0;
}
