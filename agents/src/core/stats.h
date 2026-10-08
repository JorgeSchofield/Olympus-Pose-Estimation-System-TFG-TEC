/*
 * stats.h - Measurement helpers for the validation indicators (DESIGN.md §10.3,
 * §10.4, §16). Portable C99, static storage only.
 */
#ifndef PE_STATS_H
#define PE_STATS_H

#include <stdint.h>

/* Latency / interval histogram: 0.1 ms bins up to 1 s, overflow bin above. */
#define PE_HIST_BINS 10000
typedef struct {
	uint32_t bin[PE_HIST_BINS + 1];
	uint32_t count;
	double   max_ms;
	double   sum_ms;
} pe_hist_t;

void   pe_hist_reset(pe_hist_t* h);
void   pe_hist_add(pe_hist_t* h, double ms);
double pe_hist_percentile(const pe_hist_t* h, double pct);   /* pct in [0, 100] */

/* LLC clock scale k = d(t_rx)/d(t_llc), least squares over the last
 * PE_CLOCKFIT_N frames (~60 s at 50 Hz). k ~ 1 means the LLC tick counts real
 * time; the model predicts k ~ 1.53 for firmware v2.20 (DESIGN.md §10.4). */
#define PE_CLOCKFIT_N 3000
typedef struct {
	double   llc_s[PE_CLOCKFIT_N];
	double   rx_s[PE_CLOCKFIT_N];
	uint32_t head, n;
	double   llc_unwrapped_ms;
	uint32_t last_tick;
	int      have_last;
	uint64_t rx0_ns;
} pe_clockfit_t;

void   pe_clockfit_reset(pe_clockfit_t* c);
/* Adds one frame. Call pe_clockfit_reset on an LLC reset. */
void   pe_clockfit_add(pe_clockfit_t* c, uint32_t tick_ms, uint64_t t_rx_ns);
/* Returns the slope, or 0 if fewer than 50 frames. */
double pe_clockfit_scale(const pe_clockfit_t* c);

#endif
