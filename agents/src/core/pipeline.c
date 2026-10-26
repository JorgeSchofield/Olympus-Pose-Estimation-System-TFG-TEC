/*
 * pipeline.c - Inputs of the fusion step and axes of the published pose (see
 * pipeline.h). The fusion inputs mirror
 * the model's hlc_fusion_inputs.m.
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

void pe_pose_to_frame(int pose_frame, const double x_in[5], const double P_in[25],
                      double x_out[5], double P_out[25])
{
	/* T = blockdiag([0 -1; 1 0], I3): new state = T * state */
	double T[25] = { 0 };
	double TP[25];
	int i, j, k;

	if (pose_frame != 1) {
		for (i = 0; i < 5; i++) x_out[i] = x_in[i];
		for (i = 0; i < 25; i++) P_out[i] = P_in[i];
		return;
	}
	T[0 + 5 * 1] = -1.0;   /* row 0, column 1: x1 = -y0 */
	T[1 + 5 * 0] = 1.0;    /* row 1, column 0: y1 =  x0 */
	T[2 + 5 * 2] = T[3 + 5 * 3] = T[4 + 5 * 4] = 1.0;
	for (i = 0; i < 5; i++) {
		x_out[i] = 0.0;
		for (k = 0; k < 5; k++) x_out[i] += T[i + 5 * k] * x_in[k];
	}
	for (i = 0; i < 5; i++) {              /* TP = T * P */
		for (j = 0; j < 5; j++) {
			TP[i + 5 * j] = 0.0;
			for (k = 0; k < 5; k++) TP[i + 5 * j] += T[i + 5 * k] * P_in[k + 5 * j];
		}
	}
	for (i = 0; i < 5; i++) {              /* P_out = TP * T' */
		for (j = 0; j < 5; j++) {
			P_out[i + 5 * j] = 0.0;
			for (k = 0; k < 5; k++) P_out[i + 5 * j] += TP[i + 5 * k] * T[j + 5 * k];
		}
	}
}

void pe_pose_from_frame(int pose_frame, const double in[3], double out[3])
{
	if (pose_frame == 1) {
		out[0] = in[1];     /* x0 =  y1 */
		out[1] = -in[0];    /* y0 = -x1 */
	}
	else {
		out[0] = in[0];
		out[1] = in[1];
	}
	out[2] = in[2];
}
