/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_fusion_step.c
 *
 * Code generation for function 'sep_fusion_step'
 *
 */

/* Include files */
#include "sep_fusion_step.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "sep_fusion_init_data.h"
#include "sep_fusion_init_initialize.h"
#include "sep_fusion_init_types.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
void sep_fusion_step(sep_fusion_state_t *fst, double tick, double encL,
                     double encR, double gz_lsb, const double acc[3],
                     boolean_T imu_ok, const sep_geo_t *geo,
                     const sep_agent_params_t *ag, sep_odom_t *odom,
                     boolean_T *emit)
{
  double an;
  double d;
  double d1;
  double dCL;
  double dCR;
  double dtick;
  double tilt;
  double w_m;
  boolean_T guard1;
  if (!isInitialized_sep_fusion_init) {
    sep_fusion_init_initialize();
  }
  /* SEP_FUSION_STEP  Un ciclo del agente de fusion de datos. */
  /*  */
  /*    [fst, odom, emit] = SEP_FUSION_STEP(fst, tick, encL, encR, gz_lsb, acc,
   */
  /*                                        imu_ok, geo, ag) */
  /*  */
  /*    Entradas (una trama RAW ya decodificada, SIN convertir): */
  /*      fst     : estado, de sep_fusion_init() */
  /*      tick    : [ms] contador del LLC (u32) */
  /*      encL/R  : acumuladores por lado (i32, envuelven) */
  /*      gz_lsb  : giroscopio de yaw crudo [LSB] */
  /*      acc     : acelerometro crudo [LSB], 1x3, +1 g en z con el rover
   * nivelado */
  /*      imu_ok  : la lectura de la IMU de esta trama es valida */
  /*      geo     : struct de sep_geo_params() */
  /*      ag      : parametros de los agentes (sim_params().agents). Lee: */
  /*                gyro_scale, dt_reject, dcount_reject, loop_ms, */
  /*                reset_window, k_reref, n_still, tilt_rollover */
  /*  */
  /*    Salidas: */
  /*      fst  : estado actualizado */
  /*      odom : mensaje para el agente de estimacion (sep_odom_empty) */
  /*      emit : true si hay mensaje que enviar (false si la trama se rechazo)
   */
  /*  */
  /*    POR QUE EL MENSAJE LLEVA MAGNITUDES ACUMULADAS */
  /*    ---------------------------------------------- */
  /*    Es la regla de los acumuladores aplicada un eslabon mas adelante: entre
   */
  /*    fusion y estimacion tambien hay un buzon de profundidad 1 que puede */
  /*    descartar mensajes. Si el mensaje llevara incrementos (ds, dth), uno */
  /*    descartado seria distancia perdida para siempre. Con S y TH acumulados,
   */
  /*    el agente de estimacion diferencia contra lo ultimo que proceso y el */
  /*    mensaje siguiente trae el avance completo. La conversion es lineal, asi
   */
  /*    que S = sum(ds) es exactamente (lambda_R*C_R + lambda_L*C_L)/2 sobre los
   */
  /*    acumuladores desenvueltos (DESIGN.md, decision D2). */
  /*  */
  /*    PLAUSIBILIDAD */
  /*    ------------- */
  /*    La trama ASCII no lleva CRC: un byte alterado da un numero creible. Se
   */
  /*    rechazan saltos de tiempo o de cuentas imposibles, y la referencia sigue
   */
  /*    siendo la ULTIMA TRAMA ACEPTADA. Asi el avance de la trama rechazada */
  /*    llega integro con la siguiente buena. (hlc_step re-referenciaba con la
   */
  /*    trama rechazada y perdia ~2 intervalos de distancia por rechazo.) */
  /*    Tras ag.k_reref rechazos seguidos se asume una discontinuidad real y se
   */
  /*    re-referencia sin integrar el salto. */
  odom->n_s = 0.0;
  odom->t_cum = 0.0;
  odom->tick = 0.0;
  odom->S = 0.0;
  odom->TH = 0.0;
  odom->OM = 0.0;
  odom->n_g = 0.0;
  odom->still_frames = 0.0;
  odom->still = false;
  odom->rollover = false;
  odom->llc_reset = false;
  odom->reref = false;
  odom->imu_ok = true;
  odom->w_gyro = 0.0;
  odom->tilt = 0.0;
  *emit = false;
  w_m = gz_lsb * ag->gyro_scale;
  /*  ===================================================================== */
  /* LOCAL_TILT  Angulo entre el eje z de la IMU y la vertical. Solo detecta */
  /*    vuelco: el acelerometro NO entra al filtro (ver sep_ekf_step). */
  an = sqrt((acc[0] * acc[0] + acc[1] * acc[1]) + acc[2] * acc[2]);
  if (an > 0.0) {
    an = acc[2] / an;
    if ((an <= -1.0) || rtIsNaN(an)) {
      an = -1.0;
    }
    if (an >= 1.0) {
      an = 1.0;
    }
    tilt = acos(an);
  } else {
    tilt = 0.0;
  }
  /*  --- primera trama: solo referencia --------------------------------- */
  if (!fst->have_ref) {
    fst->tick_prev = tick;
    fst->encL_prev = encL;
    fst->encR_prev = encR;
    /*  ===================================================================== */
    fst->have_ref = true;
    fst->still_frames = 0.0;
    /*  ===================================================================== */
    odom->n_s = fst->n_s;
    odom->t_cum = fst->t_cum;
    odom->tick = tick;
    odom->S = fst->S;
    odom->TH = fst->TH;
    odom->OM = fst->OM;
    odom->n_g = fst->n_g;
    odom->still_frames = 0.0;
    odom->still = (ag->n_still <= 0.0);
    odom->rollover = (tilt > ag->tilt_rollover);
    odom->imu_ok = imu_ok;
    odom->w_gyro = w_m;
    odom->tilt = tilt;
    *emit = true;

    /*  --- reinicio del LLC: el tick retrocede y los acumuladores vuelven a ~0
     */
    /*      No se integra el salto. S, TH y OM siguen continuos; el tiempo */
    /*      avanza un periodo nominal porque el real no se conoce. */
  } else if ((tick < fst->tick_prev) && (fabs(encL) < ag->reset_window) &&
             (fabs(encR) < ag->reset_window)) {
    fst->tick_prev = tick;
    fst->encL_prev = encL;
    fst->encR_prev = encR;
    /*  ===================================================================== */
    fst->t_cum += ag->loop_ms;
    fst->n_llc_reset++;
    fst->still_frames = 0.0;
    fst->n_rej_consec = 0.0;
    /*  ===================================================================== */
    odom->n_s = fst->n_s;
    odom->t_cum = fst->t_cum;
    odom->tick = tick;
    odom->S = fst->S;
    odom->TH = fst->TH;
    odom->OM = fst->OM;
    odom->n_g = fst->n_g;
    odom->still_frames = 0.0;
    odom->still = (ag->n_still <= 0.0);
    odom->rollover = (tilt > ag->tilt_rollover);
    odom->imu_ok = imu_ok;
    odom->w_gyro = w_m;
    odom->tilt = tilt;
    odom->llc_reset = true;
    *emit = true;
  } else {
    /*  --- diferencias contra la ultima trama aceptada --------------------- */
    dtick = b_mod(tick - fst->tick_prev);
    /*  [ms], envolvente u32 */
    /*  ===================================================================== */
    /* LOCAL_WRAP_I32  Resta envolvente sobre i32. */
    dCL = b_mod((encL - fst->encL_prev) + 2.147483648E+9) - 2.147483648E+9;
    /*  ===================================================================== */
    /* LOCAL_WRAP_I32  Resta envolvente sobre i32. */
    dCR = b_mod((encR - fst->encR_prev) + 2.147483648E+9) - 2.147483648E+9;
    /*  El umbral de cuentas es por periodo nominal: un intervalo largo (tramas
     */
    /*  perdidas en el enlace) admite proporcionalmente mas cuentas. */
    an = dtick / ag->loop_ms;
    guard1 = false;
    if ((dtick > 0.0) && (dtick <= ag->dt_reject * 1000.0)) {
      if ((an <= 1.0) || rtIsNaN(an)) {
        an = 1.0;
      }
      an *= ag->dcount_reject;
      d = fabs(dCL);
      if (d < an) {
        d1 = fabs(dCR);
        if (d1 < an) {
          fst->n_rej_consec = 0.0;
          /*  --- conversion fisica (sep_odometry, sin cambios) y acumulacion
           * ----- */
          /* SEP_ODOMETRY  Odometria skid-steer por lado -> diferencial
           * equivalente. */
          /*  */
          /*    [ds, dth, moving] = SEP_ODOMETRY(dC_R, dC_L, geo) */
          /*  */
          /*    v0.3 - Entradas POR LADO. El LLC suma las cuentas de sus tres
           * ruedas */
          /*    antes de transmitir, de modo que el HLC recibe dos acumuladores
           * y no */
          /*    seis. Ver la derivacion de la constante por lado en
           * sep_geo_params. */
          /*  */
          /*    Entradas: */
          /*      dC_R : incremento del acumulador del lado DERECHO  [cuentas]
           */
          /*      dC_L : incremento del acumulador del lado IZQUIERDO [cuentas]
           */
          /*  */
          /*             CONVENCION DE SIGNO. Los motores del lado derecho estan
           */
          /*             montados en espejo: al avanzar, el lado izquierdo
           * cuenta */
          /*             POSITIVO y el derecho NEGATIVO. Esta funcion no
           * rectifica el */
          /*             signo aqui, porque geo.m_per_tick_R ya lo lleva
           * incorporado. */
          /*             Pasar cuentas ya rectificadas produce un doble cambio
           * de signo */
          /*             y un avance recto se leeria como giro puro. */
          /*  */
          /*      geo  : struct de sep_geo_params(). */
          /*  */
          /*    Salidas: */
          /*      ds     : desplazamiento del centro del cuerpo [m] */
          /*      dth    : giro de rumbo por odometria          [rad] */
          /*      moving : 1 si algun lado conto en este paso */
          /*  */
          /*    NOTA DE ARQUITECTURA */
          /*    -------------------- */
          /*    Esta funcion concentra TODA la dependencia geometrica. El nucleo
           * del */
          /*    EKF (sep_ekf_step) no conoce radios, ticks ni ancho de via:
           * recibe */
          /*    (ds, dth) ya en unidades fisicas. Esa frontera refleja el
           * reparto entre */
          /*    el agente de fusion de datos y el agente de estimacion de la */
          /*    arquitectura multiagente (DRT-SEP-001, PE-RF-006 y PE-RF-007).
           */
          /*  */
          /*    ACUMULADORES, NO INCREMENTOS */
          /*    ---------------------------- */
          /*    Las cuentas llegan ACUMULADAS y la diferencia se calcula en el
           * HLC con */
          /*    resta envolvente sobre i32. Importa por dos razones: */
          /*      1) Cuantizar cada incremento por separado introduce un sesgo
           */
          /*         determinista de redondeo; diferenciar acumulados conserva
           * la */
          /*         informacion sub-tick y acota el error de cuantizacion. */
          /*      2) Una trama perdida solo alarga un dt; el desplazamiento se
           * recupera */
          /*         integro en la siguiente. Con incrementos seria error
           * PERMANENTE de */
          /*         posicion, inaceptable con hasta 1 % de perdida admitida. */
          /*  */
          /*    QUE SE PERDIO AL SUMAR POR LADO */
          /*    ------------------------------- */
          /*    La version anterior devolvia ademas la dispersion intra-lado,
           * que */
          /*    detectaba una rueda individual patinando o bloqueada sin
           * sensores */
          /*    adicionales. Con acumuladores por lado esa senal no existe: tres
           * ruedas */
          /*    sumadas son un solo numero. Queda UNA sola senal de
           * deslizamiento, la */
          /*    discrepancia giroscopio-encoders, con su punto ciego declarado:
           * si las */
          /*    seis ruedas patinan por igual, ambos sensores concuerdan y el
           * filtro */
          /*    integra distancia que no ocurrio. */
          /*    La dispersion se recupera sin tocar el contrato si alguna vez el
           * LLC */
          /*    vuelve a transmitir las seis cuentas; el campo enc[] de la trama
           * tiene */
          /*    los seis huecos reservados. */
          /*  Distancia recorrida por cada lado. La constante lleva el signo del
           */
          /*  montaje en espejo (ver sep_geo_params). */
          dCR *= geo->m_per_tick_R;
          an = dCL * geo->m_per_tick_L;
          /*  Reposo: habilita ZARU, que es lo unico que hace observable el
           * sesgo */
          /*  del giroscopio. Se mide sobre las cuentas crudas, no sobre metros,
           */
          /*  para que el umbral se exprese en la unidad que produce el sensor.
           */
          fst->S += 0.5 * (dCR + an);
          fst->TH += (dCR - an) / geo->B_eff;
          fst->t_cum += dtick;
          fst->n_s++;
          if (imu_ok) {
            fst->OM += w_m;
            fst->n_g++;
          }
          if ((d1 + d > geo->stop_ticks) > 0.5) {
            fst->still_frames = 0.0;
          } else {
            fst->still_frames++;
          }
          fst->tick_prev = tick;
          fst->encL_prev = encL;
          fst->encR_prev = encR;
          /*  =====================================================================
           */
          odom->n_s = fst->n_s;
          odom->t_cum = fst->t_cum;
          odom->tick = tick;
          odom->S = fst->S;
          odom->TH = fst->TH;
          odom->OM = fst->OM;
          odom->n_g = fst->n_g;
          odom->still_frames = fst->still_frames;
          odom->still = (fst->still_frames >= ag->n_still);
          odom->rollover = (tilt > ag->tilt_rollover);
          odom->imu_ok = imu_ok;
          odom->w_gyro = w_m;
          odom->tilt = tilt;
          *emit = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      fst->n_reject++;
      fst->n_rej_consec++;
      if (fst->n_rej_consec >= ag->k_reref) {
        fst->tick_prev = tick;
        fst->encL_prev = encL;
        fst->encR_prev = encR;
        /*  =====================================================================
         */
        fst->t_cum += ag->loop_ms;
        fst->n_reref++;
        fst->n_rej_consec = 0.0;
        fst->still_frames = 0.0;
        /*  =====================================================================
         */
        odom->n_s = fst->n_s;
        odom->t_cum = fst->t_cum;
        odom->tick = tick;
        odom->S = fst->S;
        odom->TH = fst->TH;
        odom->OM = fst->OM;
        odom->n_g = fst->n_g;
        odom->still_frames = 0.0;
        odom->still = (ag->n_still <= 0.0);
        odom->rollover = (tilt > ag->tilt_rollover);
        odom->imu_ok = imu_ok;
        odom->w_gyro = w_m;
        odom->tilt = tilt;
        odom->reref = true;
        *emit = true;
      }
    }
  }
}

/* End of code generation (sep_fusion_step.c) */
