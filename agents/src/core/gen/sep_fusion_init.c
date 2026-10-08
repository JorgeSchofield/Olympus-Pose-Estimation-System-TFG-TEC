/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_fusion_init.c
 *
 * Code generation for function 'sep_fusion_init'
 *
 */

/* Include files */
#include "sep_fusion_init.h"
#include "rt_nonfinite.h"
#include "sep_fusion_init_data.h"
#include "sep_fusion_init_initialize.h"
#include "sep_fusion_init_types.h"

/* Function Definitions */
void sep_fusion_init(sep_fusion_state_t *fst)
{
  if (!isInitialized_sep_fusion_init) {
    sep_fusion_init_initialize();
  }
  /* SEP_FUSION_INIT  Estado inicial del agente de fusion de datos. */
  /*  */
  /*    fst = SEP_FUSION_INIT() */
  /*  */
  /*    El agente de fusion recibe los acumuladores CRUDOS del agente de */
  /*    adquisicion y entrega magnitudes fisicas ACUMULADAS (S, TH, OM) al
   * agente */
  /*    de estimacion. Ver sep_fusion_step. */
  /*  */
  /*    Codegen-safe: todos los campos se crean aqui y nunca se anaden despues.
   */
  fst->have_ref = false;
  fst->tick_prev = 0.0;
  /*  [ms] tick del LLC de la ultima trama ACEPTADA */
  fst->encL_prev = 0.0;
  /*  acumuladores crudos de la ultima trama aceptada */
  fst->encR_prev = 0.0;
  fst->S = 0.0;
  /*  [m]     distancia acumulada */
  fst->TH = 0.0;
  /*  [rad]   giro acumulado de la odometria (sin envolver) */
  fst->OM = 0.0;
  /*  [rad/s] suma de las tasas del giroscopio */
  fst->n_g = 0.0;
  /*  muestras sumadas en OM */
  fst->n_s = 0.0;
  /*  tramas aceptadas */
  fst->t_cum = 0.0;
  /*  [ms]    tiempo continuo del LLC */
  fst->still_frames = 0.0;
  /*  tramas consecutivas en reposo */
  fst->n_rej_consec = 0.0;
  /*  rechazos consecutivos */
  fst->n_reject = 0.0;
  /*  tramas rechazadas por plausibilidad */
  fst->n_reref = 0.0;
  /*  re-referencias tras rechazos consecutivos */
  fst->n_llc_reset = 0.0;
  /*  reinicios del LLC detectados */
}

/* End of code generation (sep_fusion_init.c) */
