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

/* Axes of the PUBLISHED pose (io.pose_frame, DESIGN.md §11.0). The estimator
 * always works in frame 0: x forward at start, y left, theta CCW. Frame 1 is
 * the same frame rotated by +90 degrees: x right, y forward at start, so
 *     x1 = -y0,   y1 = x0,   theta1 = theta0   (heading still relative to the
 * start direction, CCW positive; omega and bias unchanged).
 * x/P are the estimator's state [px py theta omega b] and 5x5 covariance
 * (column-major); the covariance rotates with the position: P1 = T P0 T'. */
void pe_pose_to_frame(int pose_frame, const double x_in[5], const double P_in[25],
                      double x_out[5], double P_out[25]);

/* Inverse for RESET_POSE: x, y, theta given in the published frame -> frame 0. */
void pe_pose_from_frame(int pose_frame, const double in[3], double out[3]);

#endif /* PE_PIPELINE_H */
