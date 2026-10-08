/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_estimation_step.h
 *
 * Code generation for function 'sep_estimation_step'
 *
 */

#ifndef SEP_ESTIMATION_STEP_H
#define SEP_ESTIMATION_STEP_H

/* Include files */
#include "rtwtypes.h"
#include "sep_fusion_init_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void sep_estimation_step(sep_estimation_state_t *est,
                                const sep_odom_t *odom,
                                const sep_ekf_params_t *prm,
                                const sep_agent_params_t *ag, double x_est[5],
                                double diag_out[3], boolean_T *did);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (sep_estimation_step.h) */
