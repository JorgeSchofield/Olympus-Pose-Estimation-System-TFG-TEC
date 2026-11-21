// Version: v1.4
//! # Configuración del firmware — parámetros ajustables en tiempo de compilación
//!
//! Centraliza todas las constantes que afectan el comportamiento del rover.
//! Para cambiar un parámetro basta modificar este archivo y recompilar;
//! no se requieren cambios en `main.rs`.
//!
//! ## Variantes de hardware
//! Los umbrales de sobrecorriente se seleccionan en tiempo de compilación
//! mediante features de Cargo (ver `Cargo.toml`):
//!
//! | Feature         | Motores          | Umbral OC      |
//! |-----------------|------------------|----------------|
//! | (default)       | L298N × 6        | OC_*_L298N × 6 |
//! | `mixed-drivers` | FR/FL=L298N, resto=BTS7960 | mixto |
//! | `all-bts7960`   | BTS7960 × 6      | OC_*_BTS × 6  |

// ─── Loop principal ───────────────────────────────────────────────────────────

/// Duración de cada ciclo del loop principal en ms.
pub const LOOP_MS: u32 = 20;

/// Ciclos entre emisiones de telemetría periódica (~1 s a 20 ms/ciclo).
pub const TLM_PERIOD: u8 = 50;

/// Tamaño del buffer de respuesta ASCII en bytes.
/// TLM extendido: `TLM:NORMAL:000000:99999ms:36000mV:±32768mA:±30000×6:100C:100×6C:8189mm:±2147483647:±2147483647\n` ≈ 185 bytes.
pub const RESP_BUF: usize = 200;

// ─── Sensores de proximidad ──────────────────────────────────────────────────
//
// Análisis de distancia de frenado para HC_EMERGENCY_MM = 200 mm:
//
// 1. Latencia de detección (peor caso):
//      t_scan     = HC_READ_PERIOD × LOOP_MS = 5 × 20 ms = 100 ms
//      t_proceso  = 1 ciclo = 20 ms  (procesamiento tras detección)
//      t_total    = 120 ms
//    Un obstáculo que entra en rango justo después de una lectura no se detecta
//    hasta el siguiente ciclo de lectura, 100 ms más tarde.
//
// 2. Tipo de parada: hard_stop (ver ramp.rs) — aplica duty = 0 en el mismo ciclo.
//    No hay rampa de desaceleración; la parada es efectiva en ≤ 20 ms.
//
// 3. Velocidad máxima segura:
//      v_max = HC_EMERGENCY_MM / t_total = 200 mm / 120 ms ≈ 1.67 m/s
//    El umbral de 200 mm garantiza parada segura para cualquier velocidad
//    del rover por debajo de 1.67 m/s, lo cual cubre ampliamente la velocidad
//    estimada de operación (ver punto 4).
//
// 4. Velocidad de exploración estimada (EXP_SPEED = 40 % — HLC config):
//    Los motores DC con L298N a 12 V y la reductora NFP-5840-31ZY-EN tienen
//    velocidad de salida TBD (calibrar con encoders). Estimación conservadora:
//      v_100pct ≈ 0.5–1.0 m/s  →  v_explore ≈ 0.2–0.4 m/s
//    Distancia recorrida durante t_total = 120 ms:
//      d = 0.4 m/s × 0.12 s = 48 mm   (peor caso dentro del estimado)
//    Margen de seguridad: 200 mm − 48 mm = 152 mm  ✓
//    ACTUALIZAR este cálculo con la velocidad real medida en campo.
//
// 5. Zona de detección (HC_ECHO_TIMEOUT limita a ~300 mm):
//    Objeto a 250 mm avanzando hacia el rover a 0.4 m/s:
//      t_hasta_emergencia = 50 mm / 0.4 m/s = 125 ms > t_total (120 ms)
//    → el rover puede detenerse antes de que el obstáculo llegue a 200 mm ✓
//
// 6. Ángulo de apertura HC-SR04: ±15° efectivos (Cytron Technologies, 2013,
//    HC-SR04 User Manual, §3). Objetos angulados o estrechos pueden no ser
//    detectados. El VL53L0X (25° FOV, precisión ±3 %) actúa como respaldo.
//
// Ref.: Borenstein, J., Everett, B. & Feng, L. (1996). Navigating Mobile
//   Robots: Sensors and Techniques. A K Peters. §4.2 — detección de obstáculos
//   con sonar, cálculo de margen de parada.
// Ref.: Cytron Technologies. (2013). HC-SR04 Ultrasonic Ranging Module —
//   User Guide. §2: range 20–4000 mm, accuracy ±3 mm, beam angle 15°.

/// Cada cuántos ciclos leer el HC-SR04 (5 × 20 ms = 100 ms).
/// El driver es bloqueante; ver `docs/consideration_implementation.md §5`.
pub const HC_READ_PERIOD: u8 = 5;

/// Distancia de emergencia HC-SR04 en mm. Por debajo → hard_stop + FAULT inmediato.
///
/// 200 mm garantiza parada segura para velocidades ≤ 1.67 m/s (ver análisis
/// del bloque anterior). Cubre el estimado de operación (≤ 0.4 m/s) con
/// 152 mm de margen. ACTUALIZAR si la velocidad real medida supera 1.67 m/s.
pub const HC_EMERGENCY_MM: u16 = 200;

/// Timeout de eco HC-SR04 en µs.
///
/// Limita el bloqueo del loop principal. Sin límite, un obstáculo a ~4 m provoca
/// ~23 ms de busy-wait, excediendo el ciclo de 20 ms. Con este valor solo se
/// espera el eco de objetos hasta ~300 mm (1.5× `HC_EMERGENCY_MM`), reduciendo
/// el bloqueo máximo de ~30 ms a ~1.75 ms.
///
/// Fórmula: `distance_mm × 10_000 / 1_715`. 300 mm → 1 749 µs.
/// La zona de detección resultante (200–300 mm) da al menos 125 ms de aviso
/// antes de que el rover (a 0.4 m/s) alcance el umbral de emergencia.
pub const HC_ECHO_TIMEOUT_US: u32 = 1_750;

/// Distancia de emergencia VL53L0X en mm. Por debajo → hard_stop + FAULT inmediato.
///
/// Umbral más ajustado (150 mm vs 200 mm del HC-SR04) por dos razones:
///   1. El VL53L0X mide con precisión ±3 % (vs ±3 mm del HC-SR04 en condiciones
///      ideales; en superficies no perpendiculares el HC-SR04 puede errar ±15 mm).
///   2. El VL53L0X tiene FOV de 25°, menos susceptible a falsas lecturas
///      por reflexión especular que el HC-SR04 (15° apertura).
/// A 150 mm de emergencia: v_max = 150 mm / 120 ms = 1.25 m/s — suficiente
/// para el rango de operación estimado (≤ 0.4 m/s).
pub const TOF_EMERGENCY_MM: u16 = 150;

// ─── Modo escalada (CLB) ──────────────────────────────────────────────────────
//
// Durante la escalada el terreno inclinado queda a ~80–120 mm del sensor frontal,
// por lo que los umbrales normales (HC=200 mm, TOF=150 mm) dispararían FAULT
// ante el suelo, no ante un obstáculo real. Los umbrales CLB son lo suficientemente
// bajos para ignorar el terreno pero detener ante una pared frontal.
//
// El umbral de stall se extiende a 3× porque los motores contra una inclinación
// toleran pausas más largas antes de confirmar un bloqueo real.

/// Distancia HC-SR04 en mm para FAULT en modo CLB.
pub const CLB_HC_EMERGENCY_MM: u16 = 60;

/// Distancia VL53L0X en mm para FAULT en modo CLB.
pub const CLB_TOF_EMERGENCY_MM: u16 = 50;

/// Ciclos sin pulso de encoder para declarar stall en modo CLB (~3 s).
pub const CLB_STALL_THRESHOLD: u16 = 150;

// ─── Detección de stall ──────────────────────────────────────────────────────

/// Ciclos sin movimiento de encoder para declarar stall (~1 s a 20 ms/ciclo).
pub const STALL_THRESHOLD: u16 = 50;

/// Velocidad mínima absoluta (%) para activar la detección de stall.
/// Por debajo se asume que el motor está intencionalmente parado.
/// Con feature `no-stall` se pone en 100 (imposible de alcanzar) para deshabilitar.
#[cfg(not(feature = "no-stall"))]
pub const STALL_SPEED_MIN: i16 = 20;
#[cfg(feature = "no-stall")]
pub const STALL_SPEED_MIN: i16 = 100;

// ─── Muestreo ADC ────────────────────────────────────────────────────────────

/// Cada cuántos ciclos leer sensores analógicos completos (~500 ms).
/// Lectura precisa para niveles Warn/Limit y para el frame TLM.
pub const SEN_READ_PERIOD: u8 = 25;

/// Cada cuántos ciclos ejecutar el chequeo rápido de sobrecorriente (~60 ms).
/// Solo detecta nivel Fault; usa menos muestras para reducir latencia.
pub const SEN_FAST_PERIOD: u8 = 3;

/// Muestras ADC por canal en la lectura lenta (8 × ~104 µs ≈ 832 µs/canal).
pub const SEN_SAMPLES: u8 = 8;

/// Muestras ADC por canal en el chequeo rápido (2 × ~104 µs ≈ 208 µs/canal).
pub const SEN_FAST_SAMPLES: u8 = 2;

// ─── Hardware de monitorización ──────────────────────────────────────────────

/// Resistencia de shunt del INA226 en mΩ.
/// Ajustar según el componente físico instalado (10 mΩ = 0.01 Ω = R010).
/// Calibrar antes de confiar en las lecturas de corriente.
pub const INA226_SHUNT_MOHM: u16 = 10;

// ─── Protección de corriente ─────────────────────────────────────────────────

/// Velocidad mínima efectiva del motor (%). Comandos con |speed| < MIN_MOTOR_SPEED
/// se descartan (→ 0) para no energizar el motor sin que gire (dead-band de fricción).
/// CALIBRAR: medir duty mínimo para vencer fricción estática con el rover cargado.
pub const MIN_MOTOR_SPEED: i16 = 15;

/// Velocidad máxima (%) aplicada a todos los motores cuando `safety == Limit`.
pub const LIMIT_SPEED_CAP: i16 = 60;

/// Incremento máximo de velocidad (%) por ciclo (20 ms) en modo soft-stop/start.
///
/// Con este valor, la desaceleración desde 100% hasta 0% tarda 10 ticks × 20 ms
/// = 200 ms, dentro del margen de RF-005 (Fault Stop ≤ 500 ms).
///
/// Reducir este valor → rampa más suave, mayor protección contra back-EMF,
/// pero mayor latencia de parada. No bajar de 5 (400 ms máx.) para RF-005.
pub const RAMP_STEP_SOFT: i16 = 10;

/// Umbrales de sobrecorriente para driver L298N.
///
/// El L298N tolera 2 A continuos y 3 A de pico por canal (ST Microelectronics,
/// 2000, *L298 Dual Full-Bridge Driver*, Table 3 — DC output current per channel).
///
/// **Historial de revisiones:**
/// - v1 (default histórico): FAULT = 2 000 mA (100 % del spec, sin margen).
/// - v2 (2026-05-27 mañana): FAULT = 1 500 mA (75 % del spec, derated).
///   Disparaba con falsos positivos en la combinación `all-bts7960 + rl-l298n`
///   porque la ACS712 RL recoge ~1 500 mA de crosstalk PWM de los otros 5
///   motores conmutando a 490 Hz (verificado en `test_only_rl_l298n`: motor
///   real consume ~300 mA, lectura inducida por bus ~1 500 mA).
/// - **v3 (2026-05-27 tarde): FAULT = 2 500 mA.** Se acepta que el L298N
///   opere brevemente sobre el spec continuo (2 000 mA) pero bajo el pico
///   absoluto (3 000 mA). Esto NO da licencia para operación sostenida en
///   2.5 A — el chip se calienta y eventualmente falla. La justificación es:
///   * crosstalk PWM ~1.5 A inducido en bus con 5+ motores activos
///   * corriente real del motor en marcha libre ≤ 500 mA
///   * stall NFP-5840 = 6 500 mA → OC sigue protegiendo contra stall real
///   * spike transitorio < 100 ms es tolerable por el L298N
///
/// **Mitigaciones complementarias recomendadas:**
/// 1. Reducir crosstalk con cableado físico (twisted pair ACS712 RL, separar
///    de cables PWM, blindaje).
/// 2. Promediar más muestras antes del OC check (latencia vs SNR).
/// 3. Cuando se haga (1) o (2), bajar OC_FAULT_L298N de vuelta a 1 800 mA.
pub const OC_WARN_L298N:  i32 = 1_500; // 60 % de 2 500 mA
pub const OC_LIMIT_L298N: i32 = 2_000; // 80 % de 2 500 mA (spec continuo del chip)
pub const OC_FAULT_L298N: i32 = 2_500; // sobre spec continuo pero bajo pico 3A — solo transitorios cortos

/// Umbrales de sobrecorriente para driver BTS7960.
///
/// # Fuente: datasheet NFP-5840-31ZY-EN (microdcmotors.com, 2024)
///
/// Motor NFP-5840-31ZY-EN — especificaciones eléctricas (todos los ratios de reducción):
///
///   | Tensión | I_no_load | I_rated | I_stall |
///   |---------|-----------|---------|---------|
///   | 12 V    |  300 mA   | 1600 mA | 6500 mA |
///   | 24 V    |  200 mA   | 1200 mA | 4400 mA |
///
/// La corriente de stall es idéntica para todos los ratios (17:1 … 1340:1):
/// depende del bobinado del motor, no de la reducción mecánica.
///
/// Umbral FAULT = 100 % de I_stall (el motor está bloqueado → stop inmediato).
/// LIMIT = 80 %, WARN = 60 % — misma escala que los umbrales L298N.
///
/// El BTS7960 soporta 43 A de pico (Infineon, BTS7960B §6.1), por lo que el
/// límite de protección lo impone el motor, no el driver.
///
/// NOTA: el límite efectivo es el convertidor DC-DC de alimentación (5 A máx.),
/// no el stall del motor (6.5 A a 12 V). OC_FAULT_BTS = 4800 mA protege el
/// convertidor con 200 mA de margen antes de su límite de 5 A, y también
/// detecta stall del motor (I_stall = 6.5 A > 4.8 A → FAULT siempre dispara).
///
/// Ref.: microdcmotors.com — NFP-5840-31ZY-EN product page, spec table (2024).
///   I_stall = 6500 mA @ 12V, I_rated = 1600 mA @ 12V (1340:1, 6 RPM).
/// Ref.: NEMA MG 1-2021, §12.53 — protección de sobrecorriente ≥ 115 % I_rated.
#[cfg(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a"))]
pub const OC_WARN_BTS:  i32 = 2_880; // 60 % de 4800 mA — carga alta
#[cfg(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a"))]
pub const OC_LIMIT_BTS: i32 = 3_840; // 80 % de 4800 mA — reducir velocidad
#[cfg(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a"))]
pub const OC_FAULT_BTS: i32 = 4_800; // 96 % del límite del convertidor (5 A) — stop

/// Umbrales de sobrecorriente por motor `[FR, FL, CR, CL, RR, RL]`.
/// Seleccionados en tiempo de compilación según el feature activo.
#[cfg(all(feature = "mixed-drivers", not(feature = "no-oc")))]
pub const OC_WARN:  [i32; 6] = [OC_WARN_L298N,  OC_WARN_L298N,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS];
#[cfg(all(feature = "mixed-drivers", not(feature = "no-oc")))]
pub const OC_LIMIT: [i32; 6] = [OC_LIMIT_L298N, OC_LIMIT_L298N, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS];
#[cfg(all(feature = "mixed-drivers", not(feature = "no-oc")))]
pub const OC_FAULT: [i32; 6] = [OC_FAULT_L298N, OC_FAULT_L298N, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS];

#[cfg(all(feature = "all-bts7960", not(feature = "rl-l298n"), not(feature = "no-oc")))]
pub const OC_WARN:  [i32; 6] = [OC_WARN_BTS;  6];
#[cfg(all(feature = "all-bts7960", not(feature = "rl-l298n"), not(feature = "no-oc")))]
pub const OC_LIMIT: [i32; 6] = [OC_LIMIT_BTS; 6];
#[cfg(all(feature = "all-bts7960", not(feature = "rl-l298n"), not(feature = "no-oc")))]
pub const OC_FAULT: [i32; 6] = [OC_FAULT_BTS; 6];

// `all-bts7960 + rl-l298n`: 5× BTS7960 + RL como L298N (sustituto del BTS7960
// RL quemado en 2026-05-27). El L298N tiene OC_FAULT derated a 1 500 mA.
#[cfg(all(feature = "all-bts7960", feature = "rl-l298n", not(feature = "no-oc")))]
pub const OC_WARN:  [i32; 6] = [OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_L298N];
#[cfg(all(feature = "all-bts7960", feature = "rl-l298n", not(feature = "no-oc")))]
pub const OC_LIMIT: [i32; 6] = [OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_L298N];
#[cfg(all(feature = "all-bts7960", feature = "rl-l298n", not(feature = "no-oc")))]
pub const OC_FAULT: [i32; 6] = [OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_L298N];

// 6× ACS712-20A — layout: FR/FL → L298N, CR/CL/RR/RL → BTS7960.
// Los umbrales son idénticos a mixed-drivers: el sensor cambia, no los límites
// del motor. La resolución de 49 mA/count es suficiente para ambos:
//   L298N fault 2000 mA → 41 counts de margen
//   BTS7960 fault 15000 mA → 306 counts de margen (< 20 A, no satura)
#[cfg(all(feature = "all-20a", not(feature = "no-oc")))]
pub const OC_WARN:  [i32; 6] = [OC_WARN_L298N,  OC_WARN_L298N,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS,  OC_WARN_BTS];
#[cfg(all(feature = "all-20a", not(feature = "no-oc")))]
pub const OC_LIMIT: [i32; 6] = [OC_LIMIT_L298N, OC_LIMIT_L298N, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS, OC_LIMIT_BTS];
#[cfg(all(feature = "all-20a", not(feature = "no-oc")))]
pub const OC_FAULT: [i32; 6] = [OC_FAULT_L298N, OC_FAULT_L298N, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS, OC_FAULT_BTS];

#[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a", feature = "no-oc")))]
pub const OC_WARN:  [i32; 6] = [OC_WARN_L298N;  6];
#[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a", feature = "no-oc")))]
pub const OC_LIMIT: [i32; 6] = [OC_LIMIT_L298N; 6];
#[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a", feature = "no-oc")))]
pub const OC_FAULT: [i32; 6] = [OC_FAULT_L298N; 6];

// Protección OC desactivada — solo para pruebas de motores sin ACS712 conectados.
// NUNCA usar en operación normal; los motores pueden quemarse sin esta protección.
#[cfg(feature = "no-oc")]
pub const OC_WARN:  [i32; 6] = [i32::MAX; 6];
#[cfg(feature = "no-oc")]
pub const OC_LIMIT: [i32; 6] = [i32::MAX; 6];
#[cfg(feature = "no-oc")]
pub const OC_FAULT: [i32; 6] = [i32::MAX; 6];

// ─── Protección térmica de batería ────────────────────────────────────────────
//
// Sensor: NTC thermistor (módulo AD36958) en superficie de cada celda 18650 NMC.
// Estos umbrales aplican a temperatura de celda, NO a temperatura ambiente.
// La temperatura de celda bajo carga excede la temperatura ambiente 10-15 °C.
//
// Fundamento para celdas Li-ion NMC 18650 (ej. Samsung INR18650-30Q,
// Panasonic NCR18650B):
//   - Rango operativo de descarga recomendado: −20 °C … 60 °C (Samsung SDI,
//     2015, *INR18650-30Q Specification Sheet*, §Discharge Characteristics).
//   - Inicio de descomposición del electrolito (SEI): ~70–80 °C.
//   - Onset de thermal runaway (exotérmico autosostenible): ~80–130 °C según
//     nivel de SoC y variante química (NMC vs NCA).
//     Ref.: Feng, X. et al. (2018). "Thermal runaway mechanism of lithium ion
//     battery for electric vehicles: A review." *Energy Storage Materials*,
//     10, 246-267. — Fig. 3: ARC onset para celda NMC a 100 % SoC ≈ 80 °C.
//   - Requisito estándar: IEC 62133-2:2017 §4.3.8 — máx. 60 °C de operación
//     para celdas secundarias de litio en equipos portátiles.
//
// Escala de umbrales:
//   WARN  = 45 °C → tope del rango recomendado; alertar al operador.
//   LIMIT = 55 °C → reducir velocidad (< corriente de carga → < calor Joule).
//   FAULT = 65 °C → detención inmediata; margen de 15 °C antes del onset de
//           descomposición del SEI (~80 °C) y 25-65 °C antes del thermal runaway.
//
// Nota: estos umbrales disparan ANTES que el TEMP_CRIT_C del HLC (60 °C de
// temperatura AMBIENTE). En operación normal, la temperatura de celda supera
// la ambiente al menos 10 °C bajo carga, por lo que BATT_FAULT_C = 65 °C
// se alcanza cuando el ambiente está aún en ~50–55 °C (< TEMP_CRIT_C).
// La protección térmica del HLC es una red de seguridad secundaria para el
// caso de fallo del sensor NTC o de la telemetría de temperatura de celda.

/// Temperatura de celda 18650 NMC en °C para cada nivel de protección.
/// Sensor: NTC en superficie de celda — ver comentario del bloque anterior.
pub const BATT_WARN_C:  i32 = 45; // tope del rango operativo recomendado
pub const BATT_LIMIT_C: i32 = 55; // IEC 62133-2:2017 §4.3.8 máx. continuo
pub const BATT_FAULT_C: i32 = 65; // 15 °C antes del onset de descomposición SEI

// ─── Plausibilidad de sensores de temperatura ─────────────────────────────────
//
// Si una lectura cae fuera de estos rangos, el sensor probablemente está
// desconectado o falla (pin ADC flotante). Ignorar la lectura y emitir
// "WARN:LM335_OOR" / "WARN:NTC_OOR" en lugar de usar el valor basura.
//
// LM335 desconectado (ADC=0): read_celsius(0) = -273 °C → silencioso, jamás alarma.
// NTC desconectado  (ADC flotante alto ~1023): read_celsius(1023) = -20 °C → ídem.

/// Rango plausible de temperatura ambiente para LM335.
pub const AMBIENT_TEMP_MIN_C: i32 = -40;
pub const AMBIENT_TEMP_MAX_C: i32 =  80;

/// Umbral de temperatura ambiente para Safe Mode directo en el LLC (EPS-REQ-003).
///
/// Si la temperatura ambiente supera este valor, el LLC entra en Safe Mode
/// sin esperar al HLC. Esto cubre el escenario de pérdida de enlace TLM
/// con el rover en un entorno de alta temperatura (e.g. exposición solar directa).
///
/// Valor: 60 °C — por debajo del OTP del ATmega2560 (85 °C) y del umbral de
/// inicio de degradación de la electrónica de consumo (70 °C). El HLC también
/// monitorea la temperatura vía TLM con umbral más conservador (≥ 55 °C → WARN).
pub const AMBIENT_SAFE_C: i32 = 60;

/// Rango plausible de temperatura para sensores NTC de baterías.
pub const BATT_TEMP_MIN_C: i32 = -20;
pub const BATT_TEMP_MAX_C: i32 = 100;

// ─── Odometría (calibrar con hardware real) ───────────────────────────────────
//
// Estos valores son TBD — se calibrarán en campo con el rover real.
// Para calibrar TICKS_PER_REV: elevar el rover, girar una rueda una vuelta
// completa y contar pulsos con `test_encoders`. Repetir 5 veces y promediar.
// Para WHEEL_RADIUS_MM: medir el diámetro de la rueda con calibrador y dividir.
// Para WHEEL_BASE_MM: medir la separación entre centros de ruedas izq/der.

/// Pulsos de encoder por vuelta completa del eje de salida (Phase A solamente).
/// CALIBRAR en campo — NFP-5840-31ZY-EN, ratio de reductora desconocido.
pub const TICKS_PER_REV: u32 = 20; // TBD

/// Radio de la rueda en mm.
/// CALIBRAR en campo — medir con calibrador sobre rueda PLA MAX.
pub const WHEEL_RADIUS_MM: u32 = 50; // TBD

/// Distancia entre centros de contacto de ruedas izquierda y derecha (track width) en mm.
/// CALIBRAR en campo — medir separación física entre ruedas.
pub const WHEEL_BASE_MM: u32 = 280; // TBD
