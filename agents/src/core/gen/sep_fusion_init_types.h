/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_fusion_init_types.h
 *
 * Code generation for function 'sep_fusion_init'
 *
 */

#ifndef SEP_FUSION_INIT_TYPES_H
#define SEP_FUSION_INIT_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_sep_fusion_state_t
#define typedef_sep_fusion_state_t
typedef struct {
  boolean_T have_ref;
  double tick_prev;
  double encL_prev;
  double encR_prev;
  double S;
  double TH;
  double OM;
  double n_g;
  double n_s;
  double t_cum;
  double still_frames;
  double n_rej_consec;
  double n_reject;
  double n_reref;
  double n_llc_reset;
} sep_fusion_state_t;
#endif /* typedef_sep_fusion_state_t */

#ifndef typedef_sep_geo_t
#define typedef_sep_geo_t
typedef struct {
  double m_per_tick_R;
  double m_per_tick_L;
  double B_eff;
  double stop_ticks;
} sep_geo_t;
#endif /* typedef_sep_geo_t */

#ifndef typedef_sep_agent_params_t
#define typedef_sep_agent_params_t
typedef struct {
  double gyro_scale;
  double dt_reject;
  double dcount_reject;
  double loop_ms;
  double reset_window;
  double k_reref;
  double n_still;
  double tilt_rollover;
  double t_gap_max;
} sep_agent_params_t;
#endif /* typedef_sep_agent_params_t */

#ifndef typedef_sep_ekf_params_t
#define typedef_sep_ekf_params_t
typedef struct {
  double k_rho;
  double s2_ds_floor;
  double s2_theta;
  double s2_w_enc;
  double s2_bg;
  double r_gyro;
  double r_zaru;
  double slip_thresh;
  double slip_gain;
  double slip_cap;
  double dt_min;
  double dt_max;
  double P0[5];
} sep_ekf_params_t;
#endif /* typedef_sep_ekf_params_t */

#ifndef typedef_sep_odom_t
#define typedef_sep_odom_t
typedef struct {
  double n_s;
  double t_cum;
  double tick;
  double S;
  double TH;
  double OM;
  double n_g;
  double still_frames;
  boolean_T still;
  boolean_T rollover;
  boolean_T llc_reset;
  boolean_T reref;
  boolean_T imu_ok;
  double w_gyro;
  double tilt;
} sep_odom_t;
#endif /* typedef_sep_odom_t */

#ifndef typedef_sep_estimation_state_t
#define typedef_sep_estimation_state_t
typedef struct {
  double x[5];
  double P[25];
  boolean_T have_ref;
  double t_prev;
  double S_prev;
  double TH_prev;
  double OM_prev;
  double ng_prev;
  double ns_prev;
  double n_steps;
  double n_substeps;
  double n_gaps;
  double n_rerefs;
  boolean_T gap;
  boolean_T imu_invalid;
} sep_estimation_state_t;
#endif /* typedef_sep_estimation_state_t */

#endif
/* End of code generation (sep_fusion_init_types.h) */
