/*
 * stats.c - Latency histogram and LLC clock-scale fit. Portable C99.
 */
#include "stats.h"

#include <string.h>

void pe_hist_reset(pe_hist_t* h)
{
	memset(h, 0, sizeof(*h));
}

void pe_hist_add(pe_hist_t* h, double ms)
{
	long b;
	if (ms < 0) {
		ms = 0;
	}
	b = (long)(ms * 10.0);
	if (b > PE_HIST_BINS) {
		b = PE_HIST_BINS;
	}
	h->bin[b]++;
	h->count++;
	h->sum_ms += ms;
	if (ms > h->max_ms) {
		h->max_ms = ms;
	}
}

double pe_hist_percentile(const pe_hist_t* h, double pct)
{
	uint32_t target, acc = 0;
	long b;
	if (h->count == 0) {
		return 0.0;
	}
	target = (uint32_t)((pct / 100.0) * (double)h->count + 0.5);
	if (target < 1) {
		target = 1;
	}
	for (b = 0; b <= PE_HIST_BINS; b++) {
		acc += h->bin[b];
		if (acc >= target) {
			/* upper edge of the bin: never under-reports a latency */
			return b == PE_HIST_BINS ? h->max_ms : (double)(b + 1) / 10.0;
		}
	}
	return h->max_ms;
}

void pe_clockfit_reset(pe_clockfit_t* c)
{
	memset(c, 0, sizeof(*c));
}

void pe_clockfit_add(pe_clockfit_t* c, uint32_t tick_ms, uint64_t t_rx_ns)
{
	if (!c->have_last) {
		c->have_last = 1;
		c->rx0_ns = t_rx_ns;
		c->llc_unwrapped_ms = 0.0;
	}
	else {
		c->llc_unwrapped_ms += (double)(uint32_t)(tick_ms - c->last_tick);   /* u32 wrap-safe */
	}
	c->last_tick = tick_ms;

	c->llc_s[c->head] = c->llc_unwrapped_ms * 1e-3;
	c->rx_s[c->head] = (double)(t_rx_ns - c->rx0_ns) * 1e-9;
	c->head = (c->head + 1u) % PE_CLOCKFIT_N;
	if (c->n < PE_CLOCKFIT_N) {
		c->n++;
	}
}

double pe_clockfit_scale(const pe_clockfit_t* c)
{
	double mx = 0, my = 0, sxx = 0, sxy = 0;
	uint32_t i;
	if (c->n < 50) {
		return 0.0;
	}
	for (i = 0; i < c->n; i++) {
		mx += c->llc_s[i];
		my += c->rx_s[i];
	}
	mx /= c->n;
	my /= c->n;
	for (i = 0; i < c->n; i++) {
		double dx = c->llc_s[i] - mx;
		sxx += dx * dx;
		sxy += dx * (c->rx_s[i] - my);
	}
	return sxx > 0 ? sxy / sxx : 0.0;
}
