# DRT-SEP-001: Documento de Requerimientos Técnicos
## Subsistema de Estimación de Pose (SEP) del róver Olympus

| Campo | Valor |
|---|---|
| Identificador | DRT-SEP-001 |
| Versión | v1.0 (cierre del TFG) |
| Entregable | Semana 3: «Documento de requerimientos técnicos» (cuadro de entregables del informe) |
| Proyecto | TFG, Ingeniería Electrónica, ITCR / SETEC Lab |
| Marco | Proyecto ELANaV, plataforma róver Olympus |
| Documentos padre | SRS Olympus v0.1; ICD-LLC-001 v1.3; anteproyecto del TFG y observaciones de los profesores asesores |
| Documentos hermanos | ICD-PE-002 v2.0 (canal de sensores); modelo de simulación de referencia v2.0 |
| Norma de referencia | IEEE 29148 (estructura); ECSS-E-ST-10-02C (métodos de verificación T/A/D/I) |

> **Fuente de verdad.** Cuando este documento y el modelo de simulación v2.0 no coincidan,
> prevalece el modelo. Los valores numéricos de parámetros viven en `sep_geo_params.m`,
> `sep_ekf_params.m`, `llc_params.m` y `sim_params.m`, con su etiqueta de procedencia
> (`MED`, `DER`, `PROV`, `TBD`). Este documento los cita, no los duplica como autoridad.

---

## 1. Propósito y alcance

Este documento especifica los requerimientos técnicos verificables del **Subsistema de
Estimación de Pose (SEP)**: el conjunto de firmware, agentes de software y sensores que
producen una estimación continua de la pose planar del róver Olympus mediante la fusión de
la odometría de seis ruedas con el giroscopio de una IMU, bajo el paradigma de sistemas
multiagente.

**Está dentro del alcance:** la interfaz eléctrica de la IMU con el LLC; la adquisición de
encoders e IMU en el LLC y su transporte al HLC; la adaptación de CMAES a Linux; la
aplicación multiagente; el estimador (EKF de cinco estados); la integración del receptor
GPS como referencia complementaria de validación; y la evaluación cuantitativa de
precisión.

**Está fuera del alcance:** la planificación de trayectoria, el control de velocidad por
rueda, la navegación visual, la estimación en seis grados de libertad, la compensación de
inclinación del giroscopio y cualquier modificación funcional de la pila de software que el
róver ya ejecuta en producción.

### 1.1 Relación con el SRS de Olympus

El SEP **no redefine** requisitos del sistema Olympus: los deriva. Cierra dos requisitos que
la campaña TRL-4 dejó abiertos:

| Requisito Olympus | Estado TRL-4 | Cómo lo cierra el SEP |
|---|---|---|
| `RF-004-R1`: compensación de deslizamiento por fusión odometría-IMU (EKF) | Implementado; la detección de deslizamiento no está calibrada ni validada (§ 8) | `PE-RF-010`, `PE-RF-011` |
| `RNF-003`: precisión de navegación (error < 5 % de la distancia) | Cumplido en las corridas realizadas: 1,4 a 2,1 % (§ 8) | `PE-RNF-004` |

**Nota de consistencia.** El SEP adopta un criterio más estricto que `RNF-003`: error de
posición final ≤ 3 % de la distancia recorrida. Cumplir `PE-RNF-004` en un escenario implica
cumplir `RNF-003` en ese escenario, con 2 puntos porcentuales de margen.

### 1.2 Cambios respecto a v0.2

| Tema | v0.2 | v0.3 |
|---|---|---|
| Vector de estado | $[x,\ y,\ \theta]^\top$ | $[p_x,\ p_y,\ \theta,\ \omega,\ b_\omega]^\top$ |
| Canal de sensores | Canal 2 binario de 50 B sobre un USART libre | Trama RAW ASCII del firmware v2.20 sobre USART0 → USB (vigente) y trama binaria SENSOR de 55 B con CRC (objetivo). Ver ICD-PE-002 v2.0 |
| Encoders | Seis acumuladores | Dos acumuladores, la suma de las tres ruedas de cada lado |
| Adaptación ante deslizamiento | Sobre $R$ (ruido de medición) | Sobre $Q$ (ruido de proceso de la odometría) |
| Sesgo del giroscopio | No considerado | Estado del filtro, observable solo en reposo (ZARU) |
| Buzones CMAES | Paso de punteros | Profundidad 1, envío con tiempo límite cero en la ruta de sensores y rotación de al menos tres búferes |
| Entrega de mensajes | ≥ 99 % sobre 10⁵ mensajes | ≥ 99 % y ≤ 4 pérdidas seguidas en una prueba de 15 min a 50 Hz |
| Verdad de terreno | GPS en exteriores, cinta en interiores | Marcadores medidos con cinta (≤ 1 cm) como referencia principal; GPS complementario |
| Objetivos específicos | Cuatro | Seis (se agregan la interfaz eléctrica y la adaptación de CMAES) |
| Reloj del LLC | Lazo determinista de 20 ms | El modelo predice que el lazo real dura más de 20 ms; **hipótesis pendiente de verificar** (Anexo C, `R-02`) |

### 1.3 Cambios respecto a v0.3

| Tema | v0.3 | v1.0 |
|---|---|---|
| Reloj del LLC | Hipótesis sin verificar | Verificado en el róver (escala 1,465 con el lazo original) y corregido con el firmware v3.0 por temporizador (escala 1,0024, 49,88 Hz). `R-02` cerrado |
| Interfaz de la IMU | Traductor de niveles previsto | Conexión directa con las resistencias de pull-up de 3,3 V del módulo, alimentada desde el pin de 3,3 V del Arduino Mega; desviación aceptada de `PE-RF-001` (§ 8) |
| Parámetros | $\chi$, sesgo y deriva del giroscopio pendientes | Medidos (Anexo B.1); siguen pendientes $\delta_{th}$ y el error de escala del giroscopio |
| Cuentas por vuelta | Anomalía abierta | La calibración y la hoja de datos del motor apuntan a una medición de banco errónea (Anexo B.2) |
| Estado de verificación | — | Nueva § 8 con el estado de cada requerimiento |

---

## 2. Contexto del sistema e interfaces

### 2.1 Nodos

| Nodo | Plataforma | Rol en el SEP |
|---|---|---|
| LLC | ATmega2560, Rust `no_std`, firmware v2.20, lazo nominal de 20 ms | Adquiere los seis encoders (decodificación ×2) y la IMU; suma las cuentas por lado; emite la trama de sensores |
| HLC | Raspberry Pi 5 (8 GB), Linux Yocto | Ejecuta la aplicación multiagente CMAES; integra el GPS; estima y publica la pose |

### 2.2 Interfaces externas

| ID | Interfaz | Descripción | Dirección |
|---|---|---|---|
| `IF-01` | Telemetría heredada (ICD-LLC-001 v1.3) | TLM ASCII extendida de 26 campos, una vez cada 50 ciclos del LLC, y comandos con semántica ACK/ERR. **Preexistente; el SEP no la modifica.** | LLC ↔ HLC |
| `IF-02` | Canal de sensores (ICD-PE-002 v2.0) | Trama de sensores una vez por ciclo del LLC sobre el mismo enlace USART0 → USB a 115 200 baud. Formato vigente: RAW ASCII. Formato objetivo: SENSOR binaria con CRC. | LLC → HLC |
| `IF-03` | GPS GY-GPSV3-NEO | NMEA sobre UART o USB al HLC. **Nunca entra al filtro.** | GPS → HLC |
| `IF-04` | MPU-9250 | I²C por software (D42 = SDA, D43 = SCL), dirección 0x68, ≈ 59 kHz medidos (modo estándar), conexión directa con las resistencias de pull-up de 10 kΩ a 3,3 V del módulo y alimentación desde el pin de 3,3 V del Arduino Mega. Giroscopio ±250 °/s (131 LSB/(°/s)); acelerómetro ±2 g (16 384 LSB/g). | IMU → LLC |
| `IF-05` | Bus de mensajería CMAES | Buzones de profundidad 1 con paso de punteros. | Interno HLC |
| `IF-06` | Publicación de pose | Salida del agente de comunicación hacia el sistema de navegación existente y hacia el registro de validación. | HLC → externo |

### 2.3 Por qué la trama de sensores comparte el enlace USART0

El firmware v2.20 ya emite la trama `RAW:` en cada ciclo por el mismo enlace que la
telemetría heredada, con un prefijo que la distingue de la TLM. Usarla no exige cambiar el
software en producción, que solo consume las líneas de TLM. La versión v0.2 proponía un
canal físico separado sobre USART3 con un adaptador USB-serial; esa opción queda como
alternativa si la carga del enlace USART0 resulta insuficiente (ICD-PE-002 v2.0, §5).

---

## 3. Requerimientos funcionales

Métodos de verificación: **T** = prueba, **A** = análisis, **D** = demostración, **I** = inspección.

| ID | Requerimiento | Criterio de aceptación | V&V | OE |
|---|---|---|---|---|
| `PE-RF-001` | La IMU se conecta al LLC mediante una interfaz eléctrica que respeta los umbrales de entrada de ambos extremos. | Niveles alto y bajo del bus I²C dentro de los márgenes de ruido de la especificación I²C (0,2 V_DD alto, 0,1 V_DD bajo); tiempo de subida < 1000 ns; 45 000 lecturas seguidas a 50 Hz sin errores. | T | 1 |
| `PE-RF-002` | El LLC adquiere las cuentas de los seis encoders y la ráfaga de 14 B del MPU-9250 en cada ciclo, y transmite por `IF-02` la suma de cuentas de cada lado, el giroscopio, el acelerómetro y la marca de tiempo. | Todos los campos aparecen en cada trama; ninguna muestra se repite ni se omite en 1000 ciclos consecutivos. | T | 2 |
| `PE-RF-003` | La trama de sensores llega al HLC con una cadencia ≥ 50 Hz. | Medición **con el reloj del HLC** sobre 10 min: intervalo entre tramas consecutivas ≤ 20 ms en el p99. La marca de tiempo del LLC no se usa para esta verificación (ver `R-02`). | T | 2 |
| `PE-RF-004` | El HLC integra el receptor GPS y obtiene coordenadas válidas con refresco ≥ 1 Hz en pruebas exteriores. | Registro de ≥ 300 s con fix válido y cadencia ≥ 1 Hz. | T | 2 |
| `PE-RF-005` | La biblioteca CMAES funciona sobre el Linux del HLC con la misma interfaz pública que la versión para FreeRTOS. | Compilación en la RPi5 con `-std=c99 -Wall -Wextra` sin advertencias nuevas respecto a la versión original; una versión del ejemplo piedra-papel-tijera a 50 Hz procesa ≥ 45 000 mensajes con registro, suspensión y reanudación, sin fallos ni crecimiento de memoria. | T | 3 |
| `PE-RF-006` | La aplicación del HLC se compone de al menos cuatro agentes CMAES: adquisición, fusión de datos, estimación y comunicación. | Los cuatro agentes se registran en la plataforma y aparecen en la traza intercambiando mensajes. | D | 4 |
| `PE-RF-007` | Los agentes intercambian estructuras binarias de tamaño fijo. Cada productor usa una rotación de al menos tres búferes, y en la ruta de sensores envía con tiempo límite cero. | Prueba unitaria: ningún mensaje entregado presenta contenido sobrescrito por el productor; ningún productor de la ruta de sensores se bloquea. | T | 4 |
| `PE-RF-008` | El agente de adquisición deposita en el buzón los **acumuladores sin convertir**; las diferencias se calculan en el agente que las consume, contra el último valor que él mismo procesó. | Con pérdida de tramas inyectada, la distancia integrada no presenta error permanente atribuible a mensajes perdidos (prueba del modelo, caso 10). | T/A | 4 |
| `PE-RF-009` | El agente de fusión convierte los acumuladores por lado en $\Delta s$, $\Delta\theta_{enc}$ y $\omega_{enc}$ con el modelo diferencial equivalente ($b_{nom}$ = promedio de los tres ejes; $b_{eff} = \chi\, b_{nom}$), y detecta el reposo cuando ningún lado registra cuentas. | Comparación contra `sep_odometry` sobre entradas sintéticas; discrepancia ≤ 1 % en $\Delta s$ y $\Delta\theta$. | A/T | 4 |
| `PE-RF-010` | El agente de estimación mantiene un EKF de cinco estados $[p_x,\ p_y,\ \theta,\ \omega,\ b_\omega]$, con predicción por odometría, corrección por giroscopio en forma de Joseph y ZARU en reposo; el sesgo solo se actualiza en reposo. | Equivalencia con `sep_ekf_step` sobre trayectorias sintéticas; $P$ simétrica y definida positiva durante toda la ejecución; NEES del bloque de posición consistente. | A/T | 4 |
| `PE-RF-011` | El SEP detecta deslizamiento por la discrepancia $\delta = \lvert\omega_m - \hat b_\omega - \omega_{enc}\rvert$ y, cuando supera $\delta_{th}$, **aumenta el ruido de proceso** de la velocidad angular de la odometría ($Q_{44}$) con el factor $\gamma$. | Ensayo con deslizamiento de un lado: $\gamma > 1$ mientras dura el deslizamiento; $\gamma = 1$ sin deslizamiento. | T | 4 |
| `PE-RF-012` | El agente de comunicación publica la pose hacia el sistema de navegación existente. | La pose es legible por un consumidor externo en el formato acordado, sin interferir con `IF-01`. | D | 5 |
| `PE-RF-013` | El SEP registra la pose estimada y la referencia GPS con marcas de tiempo del HLC para el análisis posterior. | Archivo de registro con ambas series alineadas temporalmente. | T | 6 |

---

## 4. Requerimientos no funcionales

| ID | Requerimiento | Criterio de aceptación | V&V | Prioridad |
|---|---|---|---|---|
| `PE-RNF-001` | El SEP entrega estimaciones de pose con frecuencia ≥ 10 Hz. | Intervalo entre publicaciones ≤ 100 ms en el p95 durante ≥ 15 min. | T | Crítica |
| `PE-RNF-002` | La latencia de extremo a extremo, desde la lectura en el LLC hasta la publicación en el HLC, es < 100 ms. | Medición con marca de tiempo propagada (§ 6.1); p95 < 100 ms sostenido ≥ 15 min. | T | Crítica |
| `PE-RNF-003` | La entrega de mensajes entre agentes es ≥ 99 %, sin más de cuatro pérdidas seguidas. | Conteo por número de secuencia en una prueba de 15 min a 50 Hz (45 000 mensajes). Cuatro pérdidas seguidas a 50 Hz equivalen a 100 ms sin actualización. | T | Crítica |
| `PE-RNF-004` | El error de posición final es ≤ 3 % de la distancia recorrida frente a la verdad de terreno y frente al modelo de simulación. En el giro en el lugar, el error de rumbo final es ≤ 3 % del giro total. | Anexo B.3: recta de 5 m, giro de 360° y UMBmark de 2 m por lado, medidos contra marcadores con incertidumbre ≤ 1 cm. | T/A | Crítica |
| `PE-RNF-005` | El SEP opera ≥ 15 min sin fallo, degradación de cadencia ni fuga de memoria. | Ejecución supervisada: sin reinicios de agente, memoria residente estable. | T | Alta |
| `PE-RNF-006` | El SEP no altera el comportamiento funcional de la pila de software en producción. | `IF-01` conserva formato y cadencia; `olympus_hlc` no requiere cambios; los cambios al firmware del LLC se limitan a lo que el SEP necesite y no cambian lo que la TLM entrega. | I/T | Crítica |
| `PE-RNF-007` | El código del HLC es portable a C embebido: C99, sin memoria dinámica después de la inicialización, sin dependencias de Linux fuera de la capa de portabilidad de CMAES. | Inspección; compilación de fusión y estimación con `-std=c99 -Wall -Wextra` sin advertencias; los cuatro archivos del núcleo del modelo son aptos para generación de código. | I/A | Alta |
| `PE-RNF-008` | El tiempo añadido al ciclo del LLC por la lectura de la IMU y la emisión de la trama no supera el 25 % del periodo nominal de 20 ms. | Medición con pin de traza u osciloscopio: ≤ 5 ms por ciclo. | T | Alta |
| `PE-RNF-009` | El SEP degrada de forma segura ante la pérdida de la IMU: el filtro continúa solo con odometría, con incertidumbre creciente, y lo señaliza. | Desconexión de la IMU en caliente: el sistema no se detiene y marca el estado degradado. | T | Media |

---

## 5. Restricciones de diseño

| ID | Restricción | Origen |
|---|---|---|
| `PE-CON-001` | El GPS **nunca** entra como medición del filtro. | Decisión de diseño del TFG |
| `PE-CON-002` | La decodificación de cuadratura es **×2**: ambos flancos de la fase A; la fase B solo da el sentido. | `main.rs`, ISR INT0 a INT5 |
| `PE-CON-003` | El MPU-9250 usa el bus I²C por software D42/D43 en modo estándar (≤ 100 kHz; ≈ 59 kHz medidos). No hay TWI por hardware disponible: D20/D21 están ocupados por INT0/INT1. | `soft_i2c.rs`; tabla de pines |
| `PE-CON-004` | La configuración de compilación del LLC queda congelada durante la campaña de pruebas. El mapeo de la fase B de FR/FL depende del feature activo; compilar con el feature incorrecto invierte el signo de dos ruedas sin error. | `main.rs` |
| `PE-CON-005` | El andamiaje de EKF del LLC permanece deshabilitado. Al habilitar la IMU, el feature `no-mpu` debe separarse en lectura de IMU y filtro local, con el filtro local desactivado por omisión. | `main.rs`, `Cargo.toml` |
| `PE-CON-006` | La evaluación de precisión usa marcadores fijos medidos con cinta métrica como referencia principal. El GPS (2,5 m CEP) no puede certificar un 3 % en las trayectorias previstas: necesitaría ≥ 520 m con criterio 3:1 sobre el radio de 95 %, unas 5 h a 2,9 cm/s. | § 6.2; Anexo B.3 |
| `PE-CON-007` | El proyecto dispone de 16 semanas y un único desarrollador; se descarta toda solución que exija rediseñar el cableado de encoders o el reparto de pines del ATmega2560. | Anteproyecto |
| `PE-CON-008` | La trama RAW vigente no tiene CRC. Mientras sea el formato en uso, el agente de estimación descarta muestras con $\Delta t \notin (0,\ 0{,}5\ \mathrm{s}]$ o con saltos de cuenta ≥ 20 000 por lado, y el filtro limita $\Delta t$ a $[1, 500]$ ms. | `raw_frame_from_bytes.m`, `hlc_step.m`, `sep_ekf_params.m` |

---

## 6. Presupuestos de diseño

### 6.1 Presupuesto de latencia (`PE-RNF-002`)

La latencia se mide entre el instante de muestreo en el LLC y la publicación de la pose en
el HLC. El desfase y la escala entre ambos relojes se estiman por regresión sobre una
ventana larga, usando las marcas de tiempo del HLC; con el firmware v3.0 la escala es 1,0024.
En el TFG la latencia se midió dentro del HLC y se acotó para el LLC (`PE-RNF-002`, § 8).

| Etapa | Presupuesto | Fundamento |
|---|---|---|
| Muestreo en el LLC | ≤ 20 ms | Periodo nominal del lazo |
| Lectura de la IMU y armado de la trama | ≤ 5 ms | `PE-RNF-008`; la ráfaga I²C toma ≈ 2,6 ms (14 B a ≈ 59 kHz) |
| Transmisión por `IF-02` | ≤ 7,8 ms | Trama RAW v3.0 de hasta 90 B a 115 200 baud (4,8 ms con la trama binaria de 55 B) |
| Recepción y validación (agente de adquisición) | ≤ 10 ms | Incluye la espera de planificación |
| Edad adicional por buzón lleno | ≤ 20 ms | Con envío de tiempo límite cero, una trama descartada se reemplaza por la siguiente, un periodo después |
| Agente de fusión | ≤ 5 ms | Conversión de acumuladores |
| Agente de estimación (EKF de 5 estados) | ≤ 10 ms | Costo modelado ≈ 3 ms; dominado por la planificación |
| Agente de comunicación | ≤ 5 ms | Costo modelado ≈ 2 ms |
| **Total** | **≤ 83 ms** | Margen ≥ 17 ms |

El riesgo no está en la aritmética del filtro sino en la variación de planificación de los
hilos POSIX bajo Linux sin tiempo real (`R-07`). Mitigación: `SCHED_FIFO` con prioridades
escalonadas y afinidad de CPU; requiere que el proceso tenga `CAP_SYS_NICE`.

### 6.2 Presupuesto de error (`PE-RNF-004`)

El error se descompone en dos familias (Borenstein y Feng):

- **Sistemático:** diferencia de escala entre lados ($E_d$) y ancho de vía efectivo
  incorrecto ($E_b$). Se corrige por calibración (Anexo B). El barrido de sensibilidad del
  modelo muestra que el **error de escala común domina**: 4 % de error en la escala produce
  6 a 7 cm de error máximo en un UMBmark de 1 m, frente a < 3 cm por un 20 % de error en
  $\chi$, porque el giroscopio corrige el rumbo pero no la distancia.
- **No sistemático:** deslizamiento, irregularidades del terreno y cuantización. Lo atacan
  la corrección por giroscopio y la adaptación de $Q$ (`PE-RF-010`, `PE-RF-011`).

Asignación provisional del 3 %: ≤ 1,5 % sistemático residual tras la calibración y ≤ 1,5 % no
sistemático. Se recalcula al cerrar el Anexo B.

---

## 7. Matriz de trazabilidad

| Objetivo específico | Indicador | Requisitos | Entregable (semana) |
|---|---|---|---|
| OE1: interfaz eléctrica de la IMU | Márgenes de ruido I²C, $t_r$ < 1000 ns, 45 000 lecturas sin errores | `PE-RF-001` | Esquemático y conexionado (5); caracterización medida (6) |
| OE2: adquisición y telemetría | Encoders e IMU ≥ 50 Hz medidos con el reloj del HLC; GPS ≥ 1 Hz | `PE-RF-002`, `PE-RF-003`, `PE-RF-004`, `PE-RNF-008` | Firmware del LLC y módulo GPS (8) |
| OE3: CMAES en Linux | Sin advertencias nuevas; 45 000 mensajes a 50 Hz | `PE-RF-005` | Biblioteca e informe de pruebas (8) |
| OE4: aplicación multiagente | Cuatro agentes; entrega ≥ 99 % y ≤ 4 pérdidas seguidas | `PE-RF-006` a `PE-RF-011`, `PE-RNF-003`, `PE-RNF-007` | Arquitectura (5); aplicación (10) |
| OE5: integración | Latencia < 100 ms sostenida ≥ 15 min | `PE-RF-012`, `PE-RNF-001`, `PE-RNF-002`, `PE-RNF-005`, `PE-RNF-006` | Sistema integrado (12) |
| OE6: evaluación de precisión | Error ≤ 3 % frente a simulación y verdad de terreno | `PE-RF-013`, `PE-RNF-004` | Modelo de referencia (6); informe de evaluación (14) |


## 8. Estado de verificación (v1.0)

Estado de cada requerimiento al cierre del TFG, con el método usado y la evidencia del
informe final. **Cumple**: el criterio de aceptación se verificó como está escrito.
**Parcial**: se verificó una parte. **No cumple**: se midió y no alcanza el criterio.
**No verificado**: no hay evidencia de la prueba que pide el criterio. Las pruebas de precisión
se evalúan con una corrida por prueba y sentido.

| ID | Método usado | Evidencia | Estado |
|---|---|---|---|
| `PE-RF-001` | Osciloscopio (sondas 1x, corrección de dos sondas); bandera `imu_ok` | Nivel bajo, voltaje máximo, subida (0,47 µs) y margen alto de la IMU cumplen; 96 590 lecturas seguidas sin error. El ATmega2560 reconoce el nivel alto con 0,22 V (SCL) y 0,26 V (SDA) sobre su umbral de 3,0 V, menos que 0,2 V_DD | **Parcial**: desviación aceptada (conexión directa, sin traductor); la IMU funciona sin errores en el róver |
| `PE-RF-002` | Contadores del agente de adquisición | 248 150 tramas en las tres corridas del OE6, sin huecos de secuencia; todos los campos en cada trama | Cumple |
| `PE-RF-003` | Reloj del HLC, tres corridas y una de 45 min | Media 49,88 Hz (20,048 ms); p99 del intervalo 22,0 ms | **No cumple**: el p99 supera 20 ms porque en ≈ 1 % de los ciclos la trama se arma unos ms tarde y la siguiente temprano (el par suma 40 ms; lo muestra el contador del LLC); la media queda 0,24 % bajo 50 Hz |
| `PE-RF-004` | Prueba exterior en reposo | 1808 posiciones válidas en 1807 s, sin huecos | Cumple |
| `PE-RF-005` | Compilación en la RPi 5; prueba de esfuerzo | C99 estricto sin advertencias; 45 000 mensajes, 0 errores, memoria constante | Cumple |
| `PE-RF-006` | Demostración | Cinco agentes registrados e intercambiando mensajes | Cumple |
| `PE-RF-007` | Pruebas de la aplicación y del modelo | Ningún contenido sobrescrito; envío con tiempo límite cero en la ruta de sensores | Cumple (en desarrollo) |
| `PE-RF-008` | Prueba del modelo, caso 10 | Sin error permanente con pérdida de tramas inyectada | Cumple |
| `PE-RF-009` | Comparación con `sep_odometry` | Diferencia cero en 2591 tramas | Cumple |
| `PE-RF-010` | Comparación con `sep_ekf_step`; grabación del róver | Diferencia ≤ 4,3·10⁻¹⁹ y 6,9·10⁻⁷ m sobre datos reales. La NEES no se evaluó (no hay referencia continua); el NIS en movimiento da media 0,04 a 0,11 (≥ 99,97 % bajo 3,84), un filtro conservador | **Parcial**: NEES no evaluada |
| `PE-RF-011` | — | No se hizo el ensayo con deslizamiento de un lado. $\delta_{th}$ = 0,10 rad/s sin calibrar; en el róver $\gamma > 1$ solo en reposos con el chasis en movimiento | **No verificado** |
| `PE-RF-012` | Demostración | Datagrama UDP leído por la interfaz web; `IF-01` sin cambios | Cumple |
| `PE-RF-013` | Inspección de registros | `pose.csv` y `gps.csv` con marcas del HLC | Cumple |
| `PE-RNF-001` | Reloj del HLC | 49,88 Hz; p95 del intervalo 21,0 ms | Cumple |
| `PE-RNF-002` | Latencia en el HLC más una cota del LLC | p99 en el HLC 0,096 ms; cota de extremo a extremo ≈ 30 ms | **Verificado por análisis**: no se midió con marca de tiempo propagada |
| `PE-RNF-003` | Números de secuencia | 100 % de los mensajes enviados; contando los descartes por buzón ocupado, > 99,9 %, nunca dos seguidos | Cumple |
| `PE-RNF-004` | Cinta y línea en el piso; modelo | Recta 2,1 %; giros 2,7 % y 1,7 %; UMBmark 1,5 % y 1,4 % | Cumple en las corridas realizadas |
| `PE-RNF-005` | Corrida supervisada de 45 min | Memoria constante, sin reinicios de agentes | Cumple |
| `PE-RNF-006` | Inspección y operación | `olympus_hlc` sin cambios y con telemetría intacta a través de `llcmux` | Cumple por inspección; las pruebas formales de no interferencia no se registraron |
| `PE-RNF-007` | Inspección y compilación | C99, sin memoria dinámica tras el arranque; núcleo generado con MATLAB Coder | Cumple |
| `PE-RNF-008` | Estimación (no se midió con pin) | Lectura de la IMU ≈ 2,6 ms más la transmisión bloqueante de la trama, hasta 7,8 ms | **No verificado**; por estimación no cumple si se cuenta la transmisión, aunque el ciclo cabe en 20 ms |
| `PE-RNF-009` | Grabación de prueba con caída de la IMU | El filtro sigue con odometría y marca el estado | **No verificado en hardware** |

---

## Anexo A: Canal de sensores

La especificación completa está en **ICD-PE-002 v2.0: Canal de sensores del subsistema de
estimación de pose**. Resumen:

| Aspecto | Formato vigente (RAW) | Formato objetivo (SENSOR) |
|---|---|---|
| Capa física | USART0 → USB, 115 200 baud, 8N1 | Igual |
| Trama | Texto `RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>\n`, 40 a 81 B | Binaria de 55 B, CRC-16/CCITT-FALSE (ICD-LLC-002 v1.1) |
| Integridad | Ninguna; solo rechazo por plausibilidad | CRC |
| Encoders | Dos acumuladores por lado | Seis acumuladores o modo suma por lado (bit `ENC_SIDE_SUM`) |
| Marca de tiempo | `tick` en ms, del contador del LLC | `t_llc_us` en µs |
| Cadencia | Una trama por ciclo del LLC | Igual |

---

## Anexo B: Caracterización, calibración y evaluación

### B.1 Estado de la caracterización

La campaña del 14/09/2026 resolvió la inconsistencia de la v0.2 entre la tasa de cuentas del
encoder y la velocidad en suelo: con los valores medidos, la relación es ×2,80, dentro de la
banda que acepta la verificación de consistencia del modelo.

| Parámetro | Valor | Estado |
|---|---|---|
| Diámetro efectivo de las ruedas | 10,6 a 10,7 cm | `MED` |
| Cuentas por vuelta (FR, FL, CR, CL, RR, RL) | 43 698, 54 159, 42 660, 42 882, 50 355, 44 876 | `MED` (anomalía B.2) |
| Ancho de los ejes (F, C, R) | 58,1; 57,1; 71,4 cm | `MED` |
| Ancho de vía nominal $b_{nom}$ | 0,622 m (promedio de los tres ejes) | `DER` |
| Diferencia entre las constantes por lado | 3,7 % | `DER` |
| Escala de la odometría | 3,9456·10⁻⁶ m por cuenta y lado (± 3 %; recta de 5 m: −0,3 %) | `MED` |
| Factor de ancho de vía efectivo $\chi$ | ≈ 1,54 (un giro en el sitio); el filtro usa 1,50 | `MED` |
| Velocidad máxima en suelo (100 % PWM) | 0,029 m/s; las pruebas del OE6 se hicieron a 1,65 cm/s y 1,42 °/s | `PROV` |
| Sesgo del giroscopio | Cambia por encendido: +0,0090 rad/s en la grabación cruda, −0,0023 a −0,0006 rad/s en las corridas del OE6; deriva < 0,0002 rad/s por prueba | `MED` |
| Ruido del giroscopio | 0,033 °/s en reposo con el filtro de 10 Hz (0,01 °/s/√Hz, como la hoja de datos) | `MED` |
| Base de tiempo del LLC (temporizador) | +2400 ppm (escala 1,0024) | `MED` |
| Inclinación de la IMU | ≈ 18° respecto a la vertical (acelerómetro en reposo); el chasis está inclinado ≈ 10° | `MED` |
| Error de escala del giroscopio | Los dos giros de 360° quedan cortos (−2,2 %); causa no establecida | `TBD` |
| Umbral de deslizamiento $\delta_{th}$ | 0,10 rad/s provisional, sin calibrar; a 1,65 cm/s un deslizamiento de 30 % en un lado da $\delta$ ≈ 0,005 rad/s | `TBD` |

### B.2 Anomalías abiertas

**Cuentas por vuelta (actualizado en v1.0).** El promedio de banco de ≈ 46 400 cuentas por
vuelta implicaría una reducción de ≈ 2111:1 con un sensor de 11 pulsos y decodificación ×2.
El motor es el NFP-5840-31ZY a 12 V, cuya hoja de datos ofrece reducciones de 17:1 a 670:1;
la velocidad máxima del róver (≈ 5,2 rpm en la rueda a 2,9 cm/s con 10,65 cm de diámetro) es
coherente con la de 670:1 bajo carga (9 rpm nominales y 12 rpm en vacío). La escala calibrada
en el róver (3,9456·10⁻⁶ m por cuenta y lado) implica ≈ 28 300 cuentas por vuelta de rueda y,
con 670:1, ≈ 42 cuentas por vuelta del motor: con decodificación ×2, unos 21 a 22 pulsos por
vuelta en la fase A, el doble de los 11 supuestos. Las ≈ 46 400 cuentas del banco son 1,6 veces
lo que implica la calibración, así que el error está en la medición de banco, no en el motor.
La calibración también descartó que la rueda delantera derecha cuente 2,47 veces lo que la
izquierda (los dos lados cuentan lo mismo por metro, cocientes 1,003 y 1,010). El banco se
midió a 20, 50 y 80 % de PWM y el promedio de las seis ruedas varió menos de 0,5 %
(46 313, 46 480 y 46 522 cuentas), así que la escala no depende de la velocidad. Pendiente:
confirmar la reducción instalada y los pulsos del sensor Hall.

**Diferencia entre lados (cerrado en v1.0).** La calibración mostró que los dos lados cuentan
lo mismo por metro; la deriva a la izquierda del róver (≈ 3,4 °/m) se explica como
deslizamiento de las ruedas izquierdas (4,3 % en el modelo ajustado). Texto original: las
pruebas recientes del róver muestran que, para una misma
distancia recorrida, los acumuladores de los dos lados difieren más de lo que explica la
asimetría de 3,7 % de las constantes por lado. La causa sigue sin explicarse. Pruebas
propuestas: (1) girar las ruedas con el róver elevado y los motores sin energía, para
separar el conteo del contacto con el suelo; (2) medir el diámetro de cada rueda en tres
posiciones (las ruedas son impresas en 3D); (3) repetir la recta en ambos sentidos.

### B.3 Protocolo de evaluación de precisión

| Escenario | Distancia o giro | Cota | Referencia |
|---|---|---|---|
| Recta | 5 m | ≤ 15 cm (3 %) | Marcadores y cinta, ≤ 1 cm |
| Giro en el lugar | 360° | ≤ 10,8° de rumbo (3 %) | Línea marcada en el piso, ≈ 1° |
| UMBmark | Cuadrado de 2 m por lado (8 m) | ≤ 24 cm (3 %) | Marcadores y cinta, ≤ 1 cm |

Las pausas de 3 s en las esquinas son parte del método: son el único momento en que el sesgo
del giroscopio es observable (ZARU). El GPS se registra en las pruebas exteriores como
referencia complementaria; no certifica el indicador (`PE-CON-006`).

---

## Anexo C: Riesgos y asuntos abiertos

| ID | Asunto | Impacto | Acción |
|---|---|---|---|
| `R-01` | Parámetros pendientes. **v1.0:** $\chi$, escala, sesgo, ruido y base de tiempo medidos; siguen pendientes $\delta_{th}$ y el error de escala del giroscopio. | La detección de deslizamiento no está validada; la causa del −2,2 % de los giros no está establecida. | Ensayo con rueda elevada para $\delta_{th}$; giros repetidos con la IMU nivelada. |
| `R-02` | **Cerrado en v1.0:** verificado en el róver (escala 1,465) y corregido con el firmware por temporizador (1,0024). Texto original: **Reloj del LLC (hipótesis sin verificar).** El modelo predice que el lazo suma 20 ms a su contador sin medir el trabajo del ciclo, de modo que el ciclo real dura ≈ 30,6 ms (32,7 Hz) y el $\Delta t$ reportado es menor que el real. En simulación, esto lleva el error a ≈ 43 % en un UMBmark. **No se ha verificado en el róver**: el cambio que activa la IMU todavía no se ha probado. | Si se confirma, `PE-RF-003` no se cumple con el firmware actual y la odometría sobrestima los giros. | Después de probar la IMU, comparar el contador del LLC con las marcas de tiempo del HLC. Solo si se confirma, cambiar el firmware a un temporizador y transmisión por interrupción. |
| `R-03` | La trama RAW no tiene CRC. | Un byte alterado produce un valor creíble. | Rechazo por plausibilidad (`PE-CON-008`); migrar a la trama SENSOR cuando el firmware se modifique. |
| `R-04` | `docs/encoder.md` del LLC describe un diseño obsoleto. | Induce a error de diseño. | Corregir como aporte del TFG. |
| `R-05` | Defecto latente en el andamiaje de EKF del LLC (`ekf.x as i32 * 1000`). | Nulo mientras el filtro local esté deshabilitado. | Registrar; corregir o eliminar junto con `PE-CON-005`. |
| `R-06` | Doble filtro si se reactiva el EKF del LLC al habilitar la IMU. | Rompe la trazabilidad de la precisión. | Separar `no-mpu` antes del trabajo de firmware. |
| `R-07` | Variación de planificación de hilos POSIX bajo Linux sin tiempo real. | Amenaza el p95 de `PE-RNF-002`. | `SCHED_FIFO`, prioridades escalonadas, afinidad de CPU y `CAP_SYS_NICE`. |
| `R-08` | Ruedas impresas en 3D sin control de tolerancias. | Eleva $E_d$. | Cuantificar en B.2; calibrar por lado. |
| `R-09` | El buzón de CMAES no puede sobrescribir y pasa punteros. | Bloquear al emisor pierde tramas en el puerto; sobrescribir sin rotación de búferes produce uso después de liberar. | Tiempo límite cero en la ruta de sensores (mínimo) y `MAES_QueueOverwrite` con rotación de búferes (mejora). |
| `R-10` | **Reposo falso.** El reposo se declara con una trama sin cuentas (`n_still` = 1). En las pruebas hubo pausas con el chasis asentándose o manipulado: los encoders no cuentan, pero el giroscopio sí ve giro. | El ZARU desvía el sesgo hasta 0,11 rad/s por algunos segundos y el error pasa al rumbo en el movimiento siguiente. | Declarar reposo solo con varias tramas sin cuentas y una magnitud del giroscopio menor que un umbral. |
| `R-11` | `llcmux` es un punto único de falla en la ruta de mando del róver. | Si falla, `olympus_hlc` pierde el LLC y los motores se detienen en 2 s. | Análisis de modos de falla y prueba de terminación con reinicio automático. |

---

## Anexo D: Historial de revisiones

| Versión | Fecha | Cambios |
|---|---|---|
| v0.1 | Semana 3 | Emisión inicial a partir del anteproyecto y del SRS de Olympus v0.1. |
| v0.2 | Semana 4 | El Anexo A se extrae como ICD-PE-002 v1.0. |
| v0.3 | Semana 9 | Alineación con el modelo de simulación v2.0, la campaña del 14/09/2026 y las observaciones de los profesores asesores. Ver § 1.2. |
| v1.0 | Cierre del TFG | Estado de verificación (§ 8), parámetros medidos (B.1), cuentas por vuelta y hoja de datos del motor (B.2), riesgos `R-02` cerrado y `R-10`, `R-11` nuevos. Ver § 1.3. |
