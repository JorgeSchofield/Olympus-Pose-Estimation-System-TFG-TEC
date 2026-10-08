/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_estimation_init.c
 *
 * Code generation for function 'sep_estimation_init'
 *
 */

/* Include files */
#include "sep_estimation_init.h"
#include "diag.h"
#include "rt_nonfinite.h"
#include "sep_fusion_init_data.h"
#include "sep_fusion_init_initialize.h"
#include "sep_fusion_init_types.h"

/* Function Definitions */
void sep_estimation_init(const sep_ekf_params_t *prm,
                         sep_estimation_state_t *est)
{
  int i;
  if (!isInitialized_sep_fusion_init) {
    sep_fusion_init_initialize();
  }
  /* SEP_ESTIMATION_INIT  Estado inicial del agente de estimacion. */
  /*  */
  /*    est = SEP_ESTIMATION_INIT(prm) */
  /*  */
  /*    prm : struct de sep_ekf_params() (usa P0). */
  /*  */
  /*    Guarda el estado del EKF y los ultimos acumulados que el agente proceso:
   */
  /*    las diferencias se calculan contra ESTOS valores, no contra el mensaje
   */
  /*    anterior del buzon (que pudo perderse). Ver sep_estimation_step. */
  /*  */
  /*    Codegen-safe: todos los campos se crean aqui. */
  for (i = 0; i < 5; i++) {
    est->x[i] = 0.0;
  }
  diag(prm->P0, est->P);
  est->have_ref = false;
  est->t_prev = 0.0;
  /*  [ms] t_cum del ultimo mensaje procesado */
  est->S_prev = 0.0;
  est->TH_prev = 0.0;
  est->OM_prev = 0.0;
  est->ng_prev = 0.0;
  est->ns_prev = 0.0;
  est->n_steps = 0.0;
  /*  pasos del agente que actualizaron el filtro */
  est->n_substeps = 0.0;
  /*  llamadas a sep_ekf_step (>= n_steps) */
  est->n_gaps = 0.0;
  /*  pasos divididos por intervalo largo */
  est->n_rerefs = 0.0;
  /*  re-referencias (dt <= 0 o mayor que t_gap_max) */
  est->gap = false;
  /*  el ultimo paso se dividio en subpasos */
  est->imu_invalid = false;
  /*  el ultimo paso no tuvo muestras de giroscopio */
}

/* End of code generation (sep_estimation_init.c) */
