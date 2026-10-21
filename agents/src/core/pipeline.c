/*
 * pipeline.c - Inputs of the fusion step (see pipeline.h). Mirror of the
 * model's hlc_fusion_inputs.m.
 */
#include "pipeline.h"

uint32_t pe_time_base_ms(uint32_t llc_tick, uint64_t t_rx_ns, int dt_source)
{
	/* dt_source = 1 for firmware whose tick does not count real time
	 * (v3.0: +20 per ~37 ms loop). Wraps like the tick, every 49.7 days. */
	return dt_source == 1 ? (uint32_t)(t_rx_ns / 1000000ull) : llc_tick;
}

void pe_fusion_axes(const int16_t acc_in[3], const int16_t gyr_in[3], int gyro_yaw_axis,
                    double* gz_lsb, double acc_out[3])
{
	int a = gyro_yaw_axis;
	*gz_lsb = (double)gyr_in[a];
	acc_out[0] = (double)acc_in[(a + 1) % 3];
	acc_out[1] = (double)acc_in[(a + 2) % 3];
	acc_out[2] = (double)acc_in[a];
}
