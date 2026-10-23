/*
 * pipeline.h - What the fusion step receives from an accepted RAW frame
 * (DESIGN.md §7.1-7.2, §9.6).
 *
 * The two choices the agents make between pe_raw_parse and the generated
 * sep_fusion_step live here, so the acquisition and fusion agents and the
 * replay-parity test (tests/test_core.c) share one implementation. The MATLAB
 * model has the mirror function hlc_fusion_inputs.m; change both together.
 *
 *   - time base (est.dt_source): 0 = LLC tick; 1 = HLC arrival time in ms;
 *   - IMU axes (est.gyro_yaw_axis): the axis that points up gives the yaw
 *     rate, and the same accelerometer axis goes last (vertical, used only
 *     for the rollover tilt).
 */
#ifndef PE_PIPELINE_H
#define PE_PIPELINE_H

#include <stdint.h>

/* Time value handed to sep_fusion_step, in ms. */
uint32_t pe_time_base_ms(uint32_t llc_tick, uint64_t t_rx_ns, int dt_source);

/* Yaw-rate sample [LSB] and accelerometer [LSB] with the vertical last. */
void pe_fusion_axes(const int16_t acc_in[3], const int16_t gyr_in[3], int gyro_yaw_axis,
                    double* gz_lsb, double acc_out[3]);

#endif /* PE_PIPELINE_H */
