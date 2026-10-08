/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_fusion_step.h
 *
 * Code generation for function 'sep_fusion_step'
 *
 */

#ifndef SEP_FUSION_STEP_H
#define SEP_FUSION_STEP_H

/* Include files */
#include "rtwtypes.h"
#include "sep_fusion_init_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void sep_fusion_step(sep_fusion_state_t *fst, double tick, double encL,
                            double encR, double gz_lsb, const double acc[3],
                            boolean_T imu_ok, const sep_geo_t *geo,
                            const sep_agent_params_t *ag, sep_odom_t *odom,
                            boolean_T *emit);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (sep_fusion_step.h) */
