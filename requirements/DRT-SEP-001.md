# DRT-SEP-001 — Documento de Requerimientos Técnicos
## Subsistema de Estimación de Pose (SEP) del róver Olympus

| Campo | Valor |
|---|---|
| Identificador | DRT-SEP-001 |
| Versión | v0.2 (borrador) |
| Entregable | Semana 3 — «Documento de requerimientos técnicos» (Cuadro 1.2, anteproyecto) |
| Proyecto | TFG — Ingeniería Electrónica, ITCR / SETEC Lab |
| Marco | Proyecto ELANaV — plataforma róver Olympus |
| Documentos padre | SRS Olympus v0.1; ICD-LLC-001 v1.3; Anteproyecto TFG |
| Norma de referencia | IEEE 29148 (estructura); ECSS-E-ST-10-02C (métodos V&V: T/A/D/I) |

---

## 1. Propósito y alcance

Este documento especifica los requerimientos técnicos verificables del **Subsistema de
Estimación de Pose (SEP)**: el conjunto de firmware, agentes de software y sensores que
producen una estimación continua de la pose planar $\mathbf{x} = [x,\ y,\ \theta]^{\top}$ del
róver Olympus mediante fusión de odometría de seis ruedas e IMU bajo el paradigma de
sistemas multiagente.

**Está dentro del alcance**: la adquisición de encoders e IMU en el LLC, su transporte al
HLC, la aplicación multiagente sobre CMAES adaptada a Linux, el algoritmo de fusión (EKF),
la integración del receptor GPS como referencia de validación, y la evaluación cuantitativa
de precisión.

**Está fuera del alcance**: la planificación de trayectoria, el control de lazo cerrado de
velocidad por rueda, la navegación visual, y cualquier modificación funcional de la pila de
software que el róver ya ejecuta en producción.

### 1.1 Relación con el SRS de Olympus

El SEP **no redefine** requisitos del sistema Olympus: los deriva. Concretamente, cierra dos
requisitos que la campaña TRL-4 dejó abiertos:

| Requisito Olympus | Estado TRL-4 | Cómo lo cierra el SEP |
|---|---|---|
| `RF-004-R1` — compensación de deslizamiento por fusión odometría–IMU (EKF) | PENDIENTE | `PE-RF-007`, `PE-RF-008` |
| `RNF-003` — precisión de navegación | PENDIENTE | `PE-RNF-004` |

**Nota de consistencia.** `RNF-003` fija el umbral en «error < 5 % de la distancia total»,
mientras que el indicador de la meta del anteproyecto exige **≤ 3 %**. El SEP adopta el
criterio más estricto (3 %); el cumplimiento de `PE-RNF-004` implica por construcción el de
`RNF-003`. Esta decisión debe reflejarse en la próxima revisión del SRS de Olympus para
evitar dos umbrales en circulación.

---

## 2. Contexto del sistema e interfaces

### 2.1 Nodos

| Nodo | Plataforma | Rol en el SEP |
|---|---|---|
| LLC | ATmega2560, Rust `no_std`, lazo determinista de 20 ms | Adquiere 6 encoders en cuadratura e IMU; emite trama de sensores |
| HLC | Raspberry Pi 5 (8 GB), Linux Yocto | Ejecuta la aplicación multiagente CMAES; integra el GPS; estima la pose |

### 2.2 Interfaces externas

| ID | Interfaz | Descripción | Dirección |
|---|---|---|---|
| `IF-01` | Canal 1 — ICD-LLC-001 v1.3 | Telemetría ASCII de 26 campos a ~1 Hz y comandos con semántica ACK/ERR. **Preexistente; el SEP no lo modifica.** | LLC ↔ HLC |
| `IF-02` | Canal 2 — ICD-PE-002 (documento independiente) | Trama binaria de sensores a 50 Hz sobre el USART libre del ATmega2560, con adaptador USB-TTL a un puerto USB dedicado de la RPi5. | LLC → HLC |
| `IF-03` | GPS GY-GPSV3-NEO | NMEA sobre UART/USB al HLC. **Exclusivamente verdad de terreno.** | GPS → HLC |
| `IF-04` | MPU-9250 | I²C por software (D42/PL7 = SDA, D43/PL6 = SCL), dirección 0x68, ~100 kHz. | IMU → LLC |
| `IF-05` | Bus de mensajería CMAES | Buzones de agentes con paso de punteros. | interno HLC |
| `IF-06` | Publicación de pose | Salida del agente de comunicación hacia el sistema de navegación existente y hacia el registro de validación. | HLC → externo |

### 2.3 Justificación del Canal 2

El mapa de USART del ATmega2560 no admite un tercer periférico serie nuevo:

| USART | Asignación | Disponibilidad |
|---|---|---|
| USART0 | Enlace RPi5 por USB (configuración actual) | Ocupado |
| USART1 | D18/D19 coinciden con INT2/INT3 (encoders CR/CL) | Bloqueado de forma permanente |
| USART2 | TF02 LiDAR (D17/RX2) | Ocupado |
| USART3 | D14/D15 — reservado en el código para la RPi5, sin usar hoy | **Libre** |

Exactamente uno de USART0/USART3 queda libre en cualquiera de las dos configuraciones
posibles del enlace heredado. El SEP toma el que quede libre. Se descartaron dos
alternativas:

- **Elevar la cadencia de la trama ASCII de `IF-01` a 50 Hz** (`TLM_PERIOD: 50 → 1`): rompe a
  todo consumidor que asume 1 Hz y el formateo de 26 campos de texto 50 veces por segundo en
  un AVR a 16 MHz es un costo injustificable dentro de un ciclo de 20 ms.
- **Multiplexar un prefijo nuevo sobre el mismo puerto**: obliga a que `olympus_hlc`
  demultiplexe y reenvíe, lo que exige modificar software en producción.

Ambas violan `PE-RNF-006` (no interferencia). El canal físico separado cuesta un adaptador
USB-TTL (≈ US$ 4, dentro del margen de imprevistos del presupuesto) y deja el sistema
heredado intacto por construcción.

---

## 3. Requerimientos funcionales

Métodos de verificación: **T** = prueba, **A** = análisis, **D** = demostración, **I** = inspección.

| ID | Requerimiento | Criterio de aceptación | V&V | Obj. |
|---|---|---|---|---|
| `PE-RF-001` | El LLC adquiere las cuentas de los seis encoders en cuadratura y las lecturas de acelerómetro y giroscopio del MPU-9250 dentro de cada ciclo de control de 20 ms. | Las seis cuentas y los seis ejes inerciales aparecen en la trama de `IF-02` con marca de tiempo del LLC; ninguna muestra se repite ni se omite en 1000 ciclos consecutivos. | T | 1 |
| `PE-RF-002` | El LLC transmite la trama de sensores por `IF-02` con una tasa de muestreo ≥ 50 Hz. | Medición en el HLC sobre 10 min: intervalo entre `seq` consecutivos ≤ 20 ms en el p99. | T | 1 |
| `PE-RF-003` | El HLC integra el receptor GPS y obtiene coordenadas válidas con refresco ≥ 1 Hz en pruebas exteriores. | Registro de ≥ 300 s con fix válido y cadencia ≥ 1 Hz. | T | 1 |
| `PE-RF-004` | La aplicación del HLC se compone de al menos cuatro agentes CMAES funcionales: adquisición, fusión de datos, estimación y comunicación. | Los cuatro agentes se registran en la plataforma y aparecen en la traza de ejecución intercambiando mensajes. | D | 2 |
| `PE-RF-005` | Los agentes intercambian mensajes mediante el bus de CMAES usando estructuras binarias de tamaño fijo con esquema de múltiples buffers. | Prueba unitaria de mensajería: ningún mensaje entregado presenta contenido sobrescrito por el productor. | T | 2 |
| `PE-RF-006` | El agente de fusión convierte cuentas de encoder en velocidad lineal y angular del cuerpo mediante el modelo cinemático skid-steer de seis ruedas reducido a diferencial equivalente. | Comparación contra el modelo Simulink de referencia sobre entradas sintéticas; discrepancia ≤ 1 % en $v$ y $\omega$. | A/T | 2 |
| `PE-RF-007` | El agente de estimación mantiene un EKF con predicción por odometría y corrección por giroscopio, conservando $\mathbf{x}$ y $P$ como estado persistente entre ciclos. | Convergencia del filtro sobre trayectoria sintética; $P$ acotada y definida positiva durante toda la ejecución. | A/T | 2, 3 |
| `PE-RF-008` | El SEP detecta deslizamiento por discrepancia entre la velocidad angular derivada de encoders y la medida por el giroscopio, y adapta la matriz $R$ en consecuencia. | Ensayo con una rueda elevada o sobre superficie de bajo agarre: el indicador de deslizamiento se activa y la traza de $R$ aumenta de forma correlacionada. | T | 2 |
| `PE-RF-009` | El agente de comunicación publica la pose estimada hacia el sistema de navegación existente. | La pose es legible por un consumidor externo en el formato acordado, sin interferir con `IF-01`. | D | 3 |
| `PE-RF-010` | El SEP registra pose estimada y referencia GPS con marcas de tiempo comunes para el análisis offline de la Fase 5. | Archivo de registro con ambas series alineadas temporalmente y desfase de reloj acotado y documentado. | T | 4 |

---

## 4. Requerimientos no funcionales

| ID | Requerimiento | Criterio de aceptación | V&V | Prioridad |
|---|---|---|---|---|
| `PE-RNF-001` | El SEP entrega estimaciones de pose con frecuencia de actualización ≥ 10 Hz. | Intervalo entre publicaciones ≤ 100 ms en el p95 durante ≥ 15 min. | T | Crítica |
| `PE-RNF-002` | La latencia extremo a extremo, desde la lectura del sensor en el LLC hasta la publicación de la pose en el HLC, es < 100 ms. | Medición por marca de tiempo propagada (§ 6.1); p95 < 100 ms sostenido ≥ 15 min. | T | Crítica |
| `PE-RNF-003` | La tasa de entrega de mensajes entre agentes es ≥ 99 %. | Conteo por número de secuencia sobre ≥ 10⁵ mensajes: pérdidas ≤ 1 %. | T | Crítica |
| `PE-RNF-004` | El error de estimación de posición es ≤ 3 % de la distancia recorrida frente a la verdad de terreno, y ≤ 3 % frente al modelo de simulación. | Protocolo UMBmark adaptado (Anexo B) en ≥ 3 escenarios: recta, giro en el lugar y trayectoria compuesta. | T/A | Crítica |
| `PE-RNF-005` | El SEP opera de forma continua ≥ 15 min sin fallo, degradación de cadencia ni fuga de memoria. | Ejecución supervisada de 15 min: sin reinicios de agente, RSS estable. | T | Alta |
| `PE-RNF-006` | El SEP no altera el comportamiento funcional de la pila de software en producción del róver. | `IF-01` conserva su formato de 26 campos y su cadencia; `olympus_hlc` no requiere modificación de código; el lazo del LLC conserva su periodo de 20 ms. | I/T | **Crítica** |
| `PE-RNF-007` | El código del HLC es portable a C embebido: C99, sin asignación dinámica después de la inicialización, sin dependencias específicas de Linux fuera de la capa de portabilidad. | Inspección de código; compilación de los módulos de fusión y estimación con `-std=c99 -Wall -Wextra` sin advertencias. | I/A | Alta |
| `PE-RNF-008` | El tiempo de cómputo añadido al ciclo del LLC por la lectura del IMU y la emisión de la trama de `IF-02` no supera el 25 % del periodo de 20 ms. | Medición con pin de traza o contador de ciclos: ≤ 5 ms por ciclo. | T | Alta |
| `PE-RNF-009` | El SEP degrada de forma segura ante pérdida de una fuente: si el IMU deja de responder, el filtro continúa en modo solo-odometría con incertidumbre creciente y lo señaliza. | Desconexión física del IMU en caliente: el sistema no se detiene y marca el estado degradado. | T | Media |

---

## 5. Restricciones de diseño

| ID | Restricción | Origen |
|---|---|---|
| `PE-CON-001` | El GPS **nunca** entra como medida de corrección del filtro; su uso es exclusivamente verdad de terreno para validación offline. | Decisión de diseño del TFG |
| `PE-CON-002` | La decodificación de cuadratura es **x2** (ambos flancos de la fase A, fase B leída por GPIO). La resolución efectiva es la mitad de una decodificación x4. | `main.rs`, ISR INT0–INT5 |
| `PE-CON-003` | El MPU-9250 comparte el bus I²C por software D42/D43 a ~100 kHz. No hay TWI por hardware disponible: D20/D21 están ocupados por INT0/INT1 (encoders FR/FL). | `soft_i2c.rs`; tabla de pines |
| `PE-CON-004` | La configuración de compilación del LLC queda congelada para toda la campaña de pruebas. El mapeo de la fase B de FR/FL **depende del feature activo** (D44/D45 en `default`/`mixed-drivers`; A13/A14 con `all-bts7960`); compilar con el feature incorrecto invierte el signo de dos ruedas sin generar error. | `main.rs`, comentarios de mapeo |
| `PE-CON-005` | El andamiaje de EKF existente en el LLC permanece deshabilitado. Al habilitar el IMU debe separarse el feature `no-mpu` en dos: lectura del IMU y ejecución del filtro local, con este último desactivado por omisión. | `main.rs`, `Cargo.toml` |
| `PE-CON-006` | La validación de precisión en interiores usa marcadores y cinta métrica; el GPS solo es utilizable en exteriores y en trayectorias suficientemente largas frente a su incertidumbre. | § 6.2, Anexo B |
| `PE-CON-007` | El proyecto dispone de 16 semanas y de un único desarrollador; toda solución que exija rediseño del cableado de encoders o del reparto de pines del ATmega2560 queda descartada. | Anteproyecto, § 2.3 |

---

## 6. Presupuestos de diseño

### 6.1 Presupuesto de latencia (`PE-RNF-002`)

La latencia se mide como el intervalo entre el instante de muestreo en el LLC (`t_llc_ms`,
propagado en la trama) y el instante de publicación de la pose en el HLC (reloj monotónico).
El desfase entre ambos relojes se estima por regresión sobre una ventana larga y se resta.

| Etapa | Presupuesto | Fundamento |
|---|---|---|
| Muestreo en el LLC | ≤ 20 ms | Periodo del lazo determinista (`LOOP_MS = 20`) |
| Lectura del IMU y armado de trama | ≤ 5 ms | `PE-RNF-008` |
| Transmisión por `IF-02` | ≈ 4,4 ms | 50 B × 10 bits ÷ 115 200 baud |
| Recepción y validación en el agente de adquisición | ≤ 10 ms | Incluye espera de planificación |
| Agente de fusión | ≤ 10 ms | |
| Agente de estimación (EKF) | ≤ 15 ms | Estado de 3 variables; costo dominado por planificación, no por aritmética |
| Agente de comunicación | ≤ 10 ms | |
| **Subtotal** | **≈ 74 ms** | |
| Margen | ≈ 26 ms | 26 % del presupuesto |

El presupuesto cierra con holgura. El riesgo no está en la aritmética del filtro sino en la
**varianza de planificación** de los cuatro hilos POSIX bajo Linux no-tiempo-real: la cola es
lo que puede violar el p95, no la media. Mitigación prevista: política `SCHED_FIFO` con
prioridades escalonadas y fijación de afinidad de CPU en la capa de portabilidad.

### 6.2 Presupuesto de error (`PE-RNF-004`)

El error se descompone según Borenstein y Feng en dos familias:

- **Sistemático** — diámetros de rueda desiguales ($E_d$) y ancho de vía efectivo incorrecto
  ($E_b$). Es el término dominante en superficie plana y **se corrige por calibración**
  (Anexo B). Nota de riesgo: las ruedas de Olympus son impresas en 3D sin control de
  tolerancias, por lo que $E_d$ puede ser sustancialmente mayor que en las plataformas
  comerciales de la literatura.
- **No sistemático** — deslizamiento, irregularidades del terreno, cuantización del encoder.
  Es lo que atacan la corrección por giroscopio y la $R$ adaptativa (`PE-RF-007`,
  `PE-RF-008`).

Asignación provisional del presupuesto de 3 %: ≤ 1,5 % sistemático residual tras
calibración, ≤ 1,5 % no sistemático. **Esta partición es provisional y debe recalcularse
cuando se cierre la caracterización del Anexo B.**

---

## 7. Matriz de trazabilidad

| Objetivo específico | Indicador del anteproyecto | Requisitos que lo cubren | Entregable |
|---|---|---|---|
| 1 — Integrar IMU y GPS | Encoders + IMU por UART ≥ 50 Hz; GPS ≥ 1 Hz | `PE-RF-001`, `PE-RF-002`, `PE-RF-003`, `PE-RNF-008` | Firmware LLC (sem. 7); módulo GPS (sem. 7) |
| 2 — Aplicación multiagente | ≥ 4 agentes; entrega de mensajes ≥ 99 % | `PE-RF-004`–`PE-RF-008`, `PE-RNF-003`, `PE-RNF-007` | Doc. de arquitectura (sem. 5); aplicación (sem. 10) |
| 3 — Integración HW/SW | Latencia < 100 ms sostenida ≥ 15 min | `PE-RF-009`, `PE-RNF-001`, `PE-RNF-002`, `PE-RNF-005`, `PE-RNF-006` | Sistema integrado (sem. 12) |
| 4 — Evaluación de precisión | Error ≤ 3 % vs. simulación y verdad de terreno | `PE-RF-010`, `PE-RNF-004` | Modelo Simulink (sem. 6); informe (sem. 15) |

---

## Anexo A — Interfaz del Canal 2

La especificación completa del Canal 2 se publica como documento independiente:

> **ICD-PE-002 — Canal de sensores del subsistema de estimación de pose**

Se separó de este documento porque el contrato de interfaz tiene dos implementaciones
—el emisor en el firmware del LLC y el receptor en el agente de adquisición— que deben
citar una misma fuente, y porque su ciclo de versiones es independiente del de esta
especificación de requerimientos. Es el mismo tratamiento que recibe el ICD-LLC-001 en la
documentación de la plataforma.

Resumen de las características que este documento presupone:

| Aspecto | Valor |
|---|---|
| Capa física | UART 115 200 baud, 8N1, unidireccional LLC → HLC sobre el USART libre |
| Trama | 50 B: sincronismo (2 B) + carga útil (46 B) + CRC-16/CCITT (2 B) |
| Contenido | Cuentas acumuladas de los seis encoders, acelerómetro, giroscopio, secuencia, marca de tiempo e indicadores |
| Cadencia | 50 Hz, una trama por ciclo de control del LLC |
| Utilización del enlace | 21,7 % a 115 200 baud |

Los requisitos `PE-RF-001`, `PE-RF-002`, `PE-RNF-006` y `PE-RNF-008` son trazables hacia
ese documento, y los asuntos abiertos que allí se registran —en particular la asignación
definitiva del USART— condicionan la implementación del firmware.

---

## Anexo B — Protocolo de caracterización y calibración

Este anexo es **bloqueante**: los parámetros que produce alimentan el modelo de Simulink
(actividad 4, ruta crítica), el EKF (actividad 7, ruta crítica) y la evaluación (actividad 10,
ruta crítica). Hoy `config.rs` declara `TICKS_PER_REV = 20`, `WHEEL_RADIUS_MM = 50` y
`WHEEL_BASE_MM = 280`, los tres marcados `TBD`.

### B.1 Anomalía a resolver antes de calibrar

Los datos disponibles de la campaña TRL-4 son mutuamente inconsistentes:

| Dato | Valor reportado |
|---|---|
| Velocidad de saturación del encoder | 11 257 ticks/s |
| Velocidad máxima del róver en suelo al 100 % de PWM | 0,029 m/s |
| `TICKS_PER_REV` provisional | 20 |
| `WHEEL_RADIUS_MM` provisional | 50 |

Con esos valores, 11 257 ticks/s ÷ 20 = 563 rev/s ≈ 33 800 rpm, lo que implicaría 177 m/s.
En sentido inverso, 0,029 m/s con radio de 50 mm son 0,092 rev/s, lo que exigiría ≈ 122 000
ticks por vuelta. Bajo un modelo plausible del NFP-5840-31ZY-EN (≈ 11 pulsos por vuelta de
motor, reducción ≈ 31:1, decodificación x2), lo esperable a 0,029 m/s sería del orden de
decenas de ticks por segundo, no once mil.

Hipótesis, en orden de probabilidad:

1. **Conteo espurio** por rebote o interferencia electromagnética en las líneas de encoder.
   Es coherente con ISR por cualquier flanco sin filtro temporal, con seis puentes H
   conmutando cerca y con `RNF-005` (EMC/blindaje) explícitamente descopado del TRL-4.
2. **Error en la medición de velocidad en suelo** (unidades, o medición con ruedas patinando).
3. Combinación de ambas.

Nota adicional de riesgo: si la tasa de 11 257 ticks/s por rueda fuese real, las seis ISR
agregarían ~67 kHz de interrupciones, es decir una ISR cada ~237 ciclos a 16 MHz, lo que
compromete el presupuesto de `PE-RNF-008` y compite con las ISR de USART. La caracterización
es, por tanto, compuerta tanto de la precisión como del presupuesto de cómputo.

### B.2 Secuencia de caracterización

| Paso | Procedimiento | Salida |
|---|---|---|
| B.2.1 | Róver elevado, **motores sin energía**. Girar cada rueda 10 vueltas completas a mano, marcadas con referencia visible. Registrar cuentas. | Ticks por vuelta reales, libres de interferencia |
| B.2.2 | Róver elevado, motores al 25 % de PWM. Contar ticks durante 60 s registrando en vídeo el giro real de la rueda. | Ticks por vuelta bajo interferencia |
| B.2.3 | Comparar B.2.1 con B.2.2. Si difieren de forma significativa, se confirma conteo espurio: implementar rechazo de flancos separados por menos de un umbral temporal derivado de la velocidad máxima física, y repetir. | Diagnóstico y, si procede, corrección de firmware |
| B.2.4 | Medir el diámetro de cada una de las seis ruedas con calibrador, en tres posiciones angulares. | Diámetro medio y dispersión → cota de $E_d$ |
| B.2.5 | Medir la separación entre centros de ruedas izquierda y derecha. | `WHEEL_BASE_MM` nominal |
| B.2.6 | Recorrido recto de longitud conocida sobre superficie de ensayo, ida y vuelta. | Ancho de vía efectivo $b_{\text{eff}}$ del modelo skid-steer, que difiere del nominal |
| B.2.7 | Repetir la caracterización de la curva PWM–velocidad con los ticks ya validados. | Zona muerta, tramo lineal y saturación; entrada del modelo Simulink |

Valores de partida conocidos de la campaña TRL-4, a reconfirmar: sin movimiento por debajo
del 15 % de PWM, respuesta lineal entre 25 % y 75 %, saturación por encima del 75 %.

### B.3 Protocolo de evaluación de precisión (UMBmark adaptado)

La velocidad de desplazamiento condiciona el diseño del ensayo. Si se confirma el orden de
0,029 m/s, un cuadrado UMBmark clásico de 4 × 4 m son 32 m por vuelta ≈ 18 min, y el
protocolo completo de cinco vueltas en cada sentido superaría las tres horas de marcha,
excediendo la autonomía de `RNF-007` (2 h). En ese caso:

- Cuadrado reducido de **2 × 2 m** (8 m por vuelta, ≈ 4,6 min por vuelta, ≈ 46 min de campaña).
- Tolerancia correspondiente al 3 %: **24 cm** de error de posición final, medible con cinta
  métrica y plomada.
- **El GPS queda descartado como verdad de terreno en este ensayo**: su incertidumbre es del
  orden de la propia magnitud a medir. Su uso se restringe a trayectorias rectas largas en
  exteriores (`PE-RF-003`, `PE-RF-010`).

Si la caracterización del Anexo B.2 revela que la velocidad real es un orden de magnitud
mayor, se recupera el cuadrado de 4 × 4 m y el protocolo UMBmark estándar.

Escenarios exigidos por el indicador de la meta (≥ 3): trayectoria recta, giro en el lugar y
trayectoria compuesta.

---

## Anexo C — Riesgos y asuntos abiertos

| ID | Asunto | Impacto | Acción y plazo |
|---|---|---|---|
| `R-01` | Parámetros cinemáticos sin caracterizar e inconsistentes entre sí (Anexo B.1). | Bloquea el modelo Simulink, el EKF y toda la evaluación. Afecta cuatro actividades de la ruta crítica. | Ejecutar B.2.1–B.2.3 en semanas 3–4, antes de lo previsto en el cronograma. |
| `R-02` | Espacio de pruebas no asignado. | Sin superficie plana de ≥ 3 × 3 m con anclaje para marcadores no hay evaluación de precisión. | Gestionar asignación en semana 4. |
| `R-03` | `docs/encoder.md` del repositorio del LLC describe un diseño obsoleto de fase única sin dirección por hardware, contradiciendo la implementación real de `main.rs`. | Induce a error de diseño a cualquier desarrollador futuro. | Corregir el documento como aporte del TFG. |
| `R-04` | Defecto latente: en el andamiaje de EKF del LLC, la conversión a la trama castea a entero antes de escalar (`ekf.x as i32 * 1000`), truncando a cero toda posición inferior a 1 m. | Nulo mientras el filtro esté deshabilitado; crítico si se habilita. | Registrar como hallazgo; corregir o eliminar junto con `PE-CON-005`. |
| `R-05` | Riesgo de doble filtro: al habilitar el IMU se reactiva el EKF local del LLC si no se separa el feature de compilación. | Dos estimadores simultáneos rompen la trazabilidad del indicador de precisión. | Separar `no-mpu` en dos features antes del trabajo de firmware de la semana 7. |
| `R-06` | Varianza de planificación de hilos POSIX bajo Linux no-tiempo-real. | Amenaza el p95 de `PE-RNF-002`, no la media. | Definir política de planificación en el documento de arquitectura de la semana 5. |
| `R-07` | Ruedas impresas en 3D sin control de tolerancias. | Eleva el error sistemático $E_d$ y puede consumir todo el presupuesto de 3 %. | Cuantificar en B.2.4; si $E_d$ resulta dominante, evaluar corrección por calibración individual por rueda. |

---

## Anexo D — Historial de revisiones

| Versión | Fecha | Cambios |
|---|---|---|
| v0.1 | Semana 3 | Emisión inicial. Requisitos derivados del anteproyecto y del SRS de Olympus v0.1. Parámetros cinemáticos pendientes de caracterización (Anexo B). |
| v0.2 | Semana 4 | El Anexo A se extrae como documento independiente ICD-PE-002 v1.0 y se sustituye por una referencia cruzada. Sin cambios en los requisitos. |
