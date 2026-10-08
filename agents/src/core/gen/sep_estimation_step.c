/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sep_estimation_step.c
 *
 * Code generation for function 'sep_estimation_step'
 *
 */

/* Include files */
#include "sep_estimation_step.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "sep_fusion_init_data.h"
#include "sep_fusion_init_initialize.h"
#include "sep_fusion_init_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void sep_estimation_step(sep_estimation_state_t *est, const sep_odom_t *odom,
                         const sep_ekf_params_t *prm,
                         const sep_agent_params_t *ag, double x_est[5],
                         double diag_out[3], boolean_T *did)
{
  static const signed char b_I[25] = {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
                                      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1};
  static const signed char b_H[5] = {0, 0, 0, 1, 0};
  static const signed char iv[5] = {0, 0, 0, 1, 0};
  double F[25];
  double Q[25];
  double b_F[25];
  double PHt[5];
  double S;
  double b_ds;
  double b_dt;
  double cm;
  double d;
  double dng;
  double ds;
  double dt;
  double dth;
  double m;
  double p_use_dt_max;
  double p_use_dt_min;
  double p_use_r_gyro;
  double p_use_r_zaru;
  double p_use_slip_cap;
  double p_use_slip_thresh;
  double q_gain;
  double slip;
  double sm;
  double w_enc;
  double yk;
  double z;
  int b_i;
  int c_i;
  int i;
  int i1;
  int i2;
  int moving;
  signed char H[5];
  if (!isInitialized_sep_fusion_init) {
    sep_fusion_init_initialize();
  }
  /* SEP_ESTIMATION_STEP  Un ciclo del agente de estimacion. */
  /*  */
  /*    [est, x_est, diag_out, did] = SEP_ESTIMATION_STEP(est, odom, prm, ag) */
  /*  */
  /*    Entradas: */
  /*      est  : estado, de sep_estimation_init() */
  /*      odom : mensaje del agente de fusion (sep_odom_empty), ACUMULADO */
  /*      prm  : struct de sep_ekf_params() */
  /*      ag   : parametros de los agentes. Lee: t_gap_max */
  /*  */
  /*    Salidas: */
  /*      est      : estado actualizado */
  /*      x_est    : estado del EKF tras el paso */
  /*      diag_out : [slip_metric; q_gain; nis_gyro] del ultimo subpaso */
  /*      did      : true si el filtro avanzo (false en la primera referencia o
   */
  /*                 en una re-referencia) */
  /*  */
  /*    QUE HACE */
  /*    -------- */
  /*    Diferencia los acumulados contra los ultimos que ESTE agente proceso */
  /*    (no contra el mensaje anterior, que pudo perderse en el buzon): */
  /*  */
  /*        dt  = t_cum - t_prev          ds  = S - S_prev */
  /*        dth = TH - TH_prev            z   = (OM - OM_prev) / (n_g - ng_prev)
   */
  /*  */
  /*    y llama a sep_ekf_step SIN CAMBIOS. Sin perdidas, n_g - ng_prev = 1 y la
   */
  /*    llamada es identica a la que hacia hlc_step con incrementos. */
  /*  */
  /*    Con perdidas, el mensaje siguiente trae el avance completo: ds y dth */
  /*    exactos sobre el intervalo, y z es la tasa MEDIA del giroscopio sobre */
  /*    ese mismo intervalo, coherente con w_enc = dth/dt. El ruido de esa media
   */
  /*    es r_gyro/(n_g - ng_prev); se deja r_gyro, que es conservador en los */
  /*    huecos (raros) y no obliga a tocar sep_ekf_step. */
  /*  */
  /*    INTERVALOS LARGOS */
  /*    ----------------- */
  /*    sep_ekf_step limita dt a [dt_min, dt_max]. Con acumulados, un intervalo
   */
  /*    mayor que dt_max lleva un ds EXACTO; recortarlo falsearia w_enc. Por eso
   */
  /*    se divide en m = ceil(dt/dt_max) subpasos iguales (giro constante en el
   */
  /*    intervalo), y el recorte de sep_ekf_step queda solo como guarda. Mas
   * alla */
  /*    de ag.t_gap_max se re-referencia sin integrar. */
  /*  */
  /*    REPOSO */
  /*    ------ */
  /*    La ZARU solo se aplica si TODAS las tramas del paso estuvieron quietas:
   */
  /*    la racha de reposo (still_frames) debe cubrir las tramas del paso. */
  for (i = 0; i < 5; i++) {
    x_est[i] = est->x[i];
  }
  diag_out[0] = 0.0;
  diag_out[1] = 0.0;
  diag_out[2] = 0.0;
  *did = false;
  if (!est->have_ref) {
    /*  ===================================================================== */
    est->t_prev = odom->t_cum;
    est->S_prev = odom->S;
    est->TH_prev = odom->TH;
    est->OM_prev = odom->OM;
    est->ng_prev = odom->n_g;
    est->ns_prev = odom->n_s;
    est->have_ref = true;
  } else {
    dt = (odom->t_cum - est->t_prev) * 0.001;
    /*  [s] */
    ds = odom->S - est->S_prev;
    dth = odom->TH - est->TH_prev;
    dng = odom->n_g - est->ng_prev;
    if ((dt <= 0.0) || (dt > ag->t_gap_max)) {
      est->n_rerefs++;
      est->gap = (dt > 0.0);
      /*  =====================================================================
       */
      est->t_prev = odom->t_cum;
      est->S_prev = odom->S;
      est->TH_prev = odom->TH;
      est->OM_prev = odom->OM;
      est->ng_prev = odom->n_g;
      est->ns_prev = odom->n_s;
    } else {
      /*  Giroscopio: tasa media sobre el paso. Sin muestras validas se anula la
       */
      /*  correccion (r_gyro enorme) y el filtro sigue solo con odometria. */
      p_use_r_gyro = prm->r_gyro;
      p_use_r_zaru = prm->r_zaru;
      p_use_slip_thresh = prm->slip_thresh;
      p_use_slip_cap = prm->slip_cap;
      p_use_dt_min = prm->dt_min;
      p_use_dt_max = prm->dt_max;
      if (dng > 0.0) {
        z = (odom->OM - est->OM_prev) / dng;
        est->imu_invalid = false;
      } else {
        z = est->x[3] + est->x[4];
        p_use_r_gyro = 1.0E+12;
        est->imu_invalid = true;
      }
      if (odom->still && (odom->still_frames >= odom->n_s - est->ns_prev)) {
        moving = 0;
      } else {
        moving = 1;
      }
      dng = ceil(dt / prm->dt_max - 1.0E-9);
      if ((dng <= 1.0) || rtIsNaN(dng)) {
        m = 1.0;
      } else {
        m = dng;
      }
      b_i = (int)m;
      for (i = 0; i < b_i; i++) {
        b_ds = ds / m;
        b_dt = dt / m;
        /* SEP_EKF_STEP  Un ciclo del EKF de estimacion de pose del rover
         * Olympus. */
        /*  */
        /*    Prediccion con odometria skid-steer por lado, correccion con el */
        /*    giroscopio del MPU-9250. El GPS NO interviene. */
        /*  */
        /*    Vector de estado (5x1): */
        /*        x = [ px; py; theta; w; bw ] */
        /*          px,py : posicion en el plano del mundo             [m] */
        /*          theta : rumbo, envuelto a [-pi,pi)                 [rad] */
        /*          w     : velocidad angular de rumbo, fusionada      [rad/s]
         */
        /*          bw    : sesgo del giroscopio de yaw                [rad/s]
         */
        /*  */
        /*    Entradas: */
        /*        ds      : desplazamiento por odometria en este paso   [m] */
        /*        dth_enc : giro por odometria en este paso             [rad] */
        /*        w_gyro  : tasa de yaw medida por el giroscopio        [rad/s]
         */
        /*        dt      : intervalo REPORTADO por el LLC              [s] */
        /*        moving  : 1 si los encoders detectan movimiento */
        /*        prm     : struct de sep_ekf_params() */
        /*  */
        /*    Salidas: */
        /*        x, P     : estado y covarianza actualizados */
        /*        x_est    : copia de x para registro */
        /*        diag_out : [slip_metric; q_gain; nis_gyro] */
        /*  */
        /*    POR QUE 5 ESTADOS */
        /*    ----------------- */
        /*    La velocidad la dan los encoders, asi que vx,vy salen del estado y
         * con */
        /*    ellos la restriccion no-holonomica: al modelar el cuerpo como
         * uniciclo, */
        /*    el "no hay velocidad lateral" queda impuesto por la ESTRUCTURA del
         */
        /*    modelo en lugar de anadirse como pseudo-medida. */
        /*  */
        /*    Los sesgos del acelerometro tambien salen, y no por debilidad de
         */
        /*    observabilidad sino por una razon cuantitativa: a las velocidades
         * de */
        /*    este rover la firma completa de aceleracion propia es ~0.15 m/s^2,
         */
        /*    mientras que UN GRADO de cabeceo proyecta 0.17 m/s^2 de gravedad
         * sobre */
        /*    el eje longitudinal. La senal esta enterrada bajo la inclinacion,
         * no */
        /*    bajo el ruido. El acelerometro queda fuera del filtro y se usa
         * solo */
        /*    como detector de vuelco. */
        /*  */
        /*    OBSERVABILIDAD DEL SESGO DEL GIROSCOPIO */
        /*    --------------------------------------- */
        /*    Durante un giro sostenido, w y bw NO son separables: cualquier */
        /*    discrepancia entre encoders y giroscopio puede explicarse moviendo
         */
        /*    cualquiera de los dos. Si se deja el sesgo libre, absorbe el error
         * de */
        /*    calibracion del ancho de via y el filtro converge al valor ERRONEO
         * de */
        /*    los encoders, anulando el aporte del giroscopio. Por eso bw solo
         * se */
        /*    actualiza en reposo, donde la velocidad angular verdadera es cero
         * y la */
        /*    lectura del giroscopio ES el sesgo. */
        /*  */
        /*    Consecuencia de metodo: los intervalos de reposo son parte del */
        /*    procedimiento de medicion, no una cortesia. Un recorrido sin
         * paradas */
        /*    deja el sesgo sin anclar. */
        /*  */
        /*    POR QUE SE ADAPTA Q Y NO R */
        /*    -------------------------- */
        /*    Con la odometria en la PREDICCION, la incertidumbre de los
         * encoders */
        /*    vive en el ruido de proceso. Inflar el ruido de MEDIDA durante un
         */
        /*    deslizamiento degradaria al giroscopio, que es precisamente el
         * sensor */
        /*    del que se depende en ese momento. */
        /*  */
        /*    SOBRE dt */
        /*    -------- */
        /*    dt es el intervalo que REPORTA el LLC, no el real. En el firmware
         */
        /*    actual el reloj es un contador de software que declara LOOP_MS
         * aunque */
        /*    el ciclo haya durado mas, de modo que dt viene sistematicamente */
        /*    subestimado. Eso sesga w_enc = dth_enc/dt hacia arriba y contamina
         * la */
        /*    innovacion del giroscopio de forma proporcional a la tasa de giro.
         */
        /*    El filtro no puede corregirlo sin una referencia externa; el
         * modelo lo */
        /*    reproduce para poder medir cuanto cuesta. */
        /*  ---------------------------------------------------------------------
         */
        /*  0) Guarda de dt y deteccion de deslizamiento */
        /*     Ambas entradas estan disponibles ANTES de predecir, asi que el */
        /*     inflado de Q se aplica en el mismo paso que lo origina. */
        /*  ---------------------------------------------------------------------
         */
        if (b_dt < p_use_dt_min) {
          b_dt = p_use_dt_min;
        } else if (b_dt > p_use_dt_max) {
          b_dt = p_use_dt_max;
        }
        w_enc = dth / m / b_dt;
        d = est->x[4];
        slip = fabs((z - d) - w_enc);
        if (slip > p_use_slip_thresh) {
          q_gain = prm->slip_gain * (slip / p_use_slip_thresh - 1.0) + 1.0;
          if (q_gain > p_use_slip_cap) {
            q_gain = p_use_slip_cap;
          }
        } else {
          q_gain = 1.0;
        }
        /*  ---------------------------------------------------------------------
         */
        /*  1) Prediccion con odometria */
        /*  ---------------------------------------------------------------------
         */
        yk = est->x[2];
        S = est->x[3];
        dng = yk + 0.5 * S * b_dt;
        /*  rumbo en el punto medio del paso */
        cm = cos(dng);
        sm = sin(dng);
        /*  Jacobiano de transicion. La fila de w es NULA a proposito: la */
        /*  velocidad angular se reemplaza cada paso por la que dictan los */
        /*  encoders, de modo que no arrastra covarianza previa. */
        for (i1 = 0; i1 < 25; i1++) {
          F[i1] = b_I[i1];
        }
        dng = -b_ds * sm;
        F[10] = dng;
        F[15] = dng * b_dt * 0.5;
        dng = b_ds * cm;
        F[11] = dng;
        F[16] = dng * b_dt * 0.5;
        F[17] = b_dt;
        F[18] = 0.0;
        /*  =====================================================================
         */
        /*  Envuelve un angulo al intervalo [-pi, pi). */
        x_est[0] = est->x[0] + dng;
        x_est[1] = est->x[1] + b_ds * sm;
        x_est[2] =
            c_mod((yk + S * b_dt) + 3.1415926535897931) - 3.1415926535897931;
        x_est[3] = w_enc;
        x_est[4] = d;
        /*  Ruido de proceso. La incertidumbre de ds se proyecta sobre el rumbo
         */
        /*  actual: el error de odometria es longitudinal, no isotropo. */
        dng = prm->k_rho * fabs(b_ds) + prm->s2_ds_floor;
        memset(&Q[0], 0, 25U * sizeof(double));
        Q[0] = cm * cm * dng;
        Q[5] = cm * sm * dng;
        Q[1] = Q[5];
        Q[6] = sm * sm * dng;
        Q[12] = prm->s2_theta;
        Q[18] = prm->s2_w_enc * q_gain;
        /*  <-- knob adaptativo */
        Q[24] = prm->s2_bg * b_dt;
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            yk = 0.0;
            for (c_i = 0; c_i < 5; c_i++) {
              yk += F[i1 + 5 * c_i] * est->P[c_i + 5 * i2];
            }
            b_F[i1 + 5 * i2] = yk;
          }
        }
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            yk = 0.0;
            for (c_i = 0; c_i < 5; c_i++) {
              yk += b_F[i1 + 5 * c_i] * F[i2 + 5 * c_i];
            }
            c_i = i1 + 5 * i2;
            est->P[c_i] = yk + Q[c_i];
          }
        }
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            c_i = i2 + 5 * i1;
            Q[c_i] = 0.5 * (est->P[c_i] + est->P[i1 + 5 * i2]);
          }
        }
        memcpy(&est->P[0], &Q[0], 25U * sizeof(double));
        /*  ---------------------------------------------------------------------
         */
        /*  2) Correccion con el giroscopio */
        /*     Medida escalar: z = w + bw. No hay inversion de matrices en todo
         */
        /*     el filtro, lo que simplifica el port a C embebido. */
        /*  ---------------------------------------------------------------------
         */
        for (i1 = 0; i1 < 5; i1++) {
          H[i1] = 0;
        }
        H[3] = 1;
        if (moving < 0.5) {
          H[4] = 1;
          /*  en reposo el sesgo es observable */
        } else {
          H[4] = 0;
          /*  en movimiento se mantiene, no se actualiza */
        }
        yk = z - (w_enc + d);
        /*  =====================================================================
         */
        /*  Actualizacion de Kalman para una medida escalar, forma de Joseph. */
        /*  Joseph y no (I-KH)P: preserva simetria y semidefinicion positiva
         * bajo */
        /*  errores de redondeo, que importa en aritmetica embebida. */
        /*  5x1 */
        dng = 0.0;
        for (i1 = 0; i1 < 5; i1++) {
          d = 0.0;
          for (i2 = 0; i2 < 5; i2++) {
            d += est->P[i1 + 5 * i2] * (double)H[i2];
          }
          PHt[i1] = d;
          dng += (double)H[i1] * d;
        }
        S = dng + p_use_r_gyro;
        /*  escalar */
        /*  5x1 */
        for (i1 = 0; i1 < 5; i1++) {
          d = PHt[i1] / S;
          PHt[i1] = d;
          x_est[i1] += d * yk;
        }
        /*  =====================================================================
         */
        /*  Envuelve un angulo al intervalo [-pi, pi). */
        x_est[2] = c_mod(x_est[2] + 3.1415926535897931) - 3.1415926535897931;
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            c_i = i2 + 5 * i1;
            F[c_i] = (double)b_I[c_i] - PHt[i2] * (double)H[i1];
          }
        }
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            d = 0.0;
            for (c_i = 0; c_i < 5; c_i++) {
              d += F[i1 + 5 * c_i] * est->P[c_i + 5 * i2];
            }
            b_F[i1 + 5 * i2] = d;
            Q[i2 + 5 * i1] = PHt[i2] * PHt[i1];
          }
        }
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            d = 0.0;
            for (c_i = 0; c_i < 5; c_i++) {
              d += b_F[i1 + 5 * c_i] * F[i2 + 5 * c_i];
            }
            c_i = i1 + 5 * i2;
            est->P[c_i] = d + Q[c_i] * p_use_r_gyro;
          }
        }
        for (i1 = 0; i1 < 5; i1++) {
          for (i2 = 0; i2 < 5; i2++) {
            c_i = i2 + 5 * i1;
            Q[c_i] = 0.5 * (est->P[c_i] + est->P[i1 + 5 * i2]);
          }
        }
        memcpy(&est->P[0], &Q[0], 25U * sizeof(double));
        /*  ---------------------------------------------------------------------
         */
        /*  3) ZARU - pseudo-medida de velocidad angular nula en reposo */
        /*     Es gratis (no requiere sensor) y es lo que ancla el sesgo. */
        /*  ---------------------------------------------------------------------
         */
        if (moving < 0.5) {
          /*  =====================================================================
           */
          /*  Actualizacion de Kalman para una medida escalar, forma de Joseph.
           */
          /*  Joseph y no (I-KH)P: preserva simetria y semidefinicion positiva
           * bajo */
          /*  errores de redondeo, que importa en aritmetica embebida. */
          /*  5x1 */
          dng = 0.0;
          for (i1 = 0; i1 < 5; i1++) {
            d = 0.0;
            for (i2 = 0; i2 < 5; i2++) {
              d += est->P[i1 + 5 * i2] * (double)iv[i2];
            }
            PHt[i1] = d;
            dng += (double)b_H[i1] * d;
          }
          dng += p_use_r_zaru;
          /*  escalar */
          /*  5x1 */
          b_ds = x_est[3];
          for (i1 = 0; i1 < 5; i1++) {
            d = PHt[i1] / dng;
            PHt[i1] = d;
            x_est[i1] += d * -b_ds;
          }
          /*  =====================================================================
           */
          /*  Envuelve un angulo al intervalo [-pi, pi). */
          x_est[2] = c_mod(x_est[2] + 3.1415926535897931) - 3.1415926535897931;
          for (i1 = 0; i1 < 5; i1++) {
            for (i2 = 0; i2 < 5; i2++) {
              c_i = i2 + 5 * i1;
              F[c_i] = (double)b_I[c_i] - PHt[i2] * (double)b_H[i1];
            }
          }
          for (i1 = 0; i1 < 5; i1++) {
            for (i2 = 0; i2 < 5; i2++) {
              d = 0.0;
              for (c_i = 0; c_i < 5; c_i++) {
                d += F[i1 + 5 * c_i] * est->P[c_i + 5 * i2];
              }
              b_F[i1 + 5 * i2] = d;
              Q[i2 + 5 * i1] = PHt[i2] * PHt[i1];
            }
          }
          for (i1 = 0; i1 < 5; i1++) {
            for (i2 = 0; i2 < 5; i2++) {
              d = 0.0;
              for (c_i = 0; c_i < 5; c_i++) {
                d += b_F[i1 + 5 * c_i] * F[i2 + 5 * c_i];
              }
              c_i = i1 + 5 * i2;
              est->P[c_i] = d + Q[c_i] * p_use_r_zaru;
            }
          }
          for (i1 = 0; i1 < 5; i1++) {
            for (i2 = 0; i2 < 5; i2++) {
              c_i = i2 + 5 * i1;
              Q[c_i] = 0.5 * (est->P[c_i] + est->P[i1 + 5 * i2]);
            }
          }
          memcpy(&est->P[0], &Q[0], 25U * sizeof(double));
        }
        diag_out[0] = slip;
        diag_out[1] = q_gain;
        diag_out[2] = yk * yk / S;
        for (c_i = 0; c_i < 5; c_i++) {
          est->x[c_i] = x_est[c_i];
        }
      }
      est->gap = (m > 1.0);
      est->n_gaps += (double)(m > 1.0);
      est->n_steps++;
      est->n_substeps += m;
      /*  =====================================================================
       */
      est->t_prev = odom->t_cum;
      est->S_prev = odom->S;
      est->TH_prev = odom->TH;
      est->OM_prev = odom->OM;
      est->ng_prev = odom->n_g;
      est->ns_prev = odom->n_s;
      *did = true;
    }
  }
}

/* End of code generation (sep_estimation_step.c) */
