/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diag.c
 *
 * Code generation for function 'diag'
 *
 */

/* Include files */
#include "diag.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void diag(const double v[5], double d[25])
{
  int j;
  memset(&d[0], 0, 25U * sizeof(double));
  for (j = 0; j < 5; j++) {
    d[j + 5 * j] = v[j];
  }
}

/* End of code generation (diag.c) */
