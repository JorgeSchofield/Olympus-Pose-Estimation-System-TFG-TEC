/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_fusion_init_initialize.c
 *
 * Code generation for function 'sep_fusion_init_initialize'
 *
 */

/* Include files */
#include "sep_fusion_init_initialize.h"
#include "rt_nonfinite.h"
#include "sep_fusion_init_data.h"

/* Function Definitions */
void sep_fusion_init_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_sep_fusion_init = true;
}

/* End of code generation (sep_fusion_init_initialize.c) */
