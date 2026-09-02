# ICD-PE-002 — Canal de sensores del subsistema de estimación de pose

| Campo | Valor |
|---|---|
| Identificador | ICD-PE-002 |
| Versión | v1.0 |
| Estado | Borrador para revisión |
| Título | Interfaz de sensores LLC → HLC para estimación de pose |
| Emisor | LLC — ATmega2560, firmware Rust `no_std` |
| Receptor | HLC — Raspberry Pi 5, agente de adquisición CMAES |
| Documento padre | DRT-SEP-001 (requisitos `PE-RF-001`, `PE-RF-002`, `PE-RNF-006`, `PE-RNF-008`) |
| Documento relacionado | ICD-LLC-001 v1.3 — **este documento no lo modifica** |

---

## 1. Propósito y alcance

Este documento especifica el contrato de interfaz del **Canal 2**, el enlace dedicado por
el que el controlador de bajo nivel entrega al controlador de alto nivel las cuentas de los
seis encoders y las mediciones inerciales necesarias para la estimación de pose.

El Canal 2 es un enlace **nuevo y físicamente independiente** del canal de telemetría y
comandos existente. La razón es una restricción de diseño: el subsistema de estimación de
pose debe integrarse sin alterar el protocolo ni el software actualmente desplegados en el
róver (`PE-RNF-006`). Al añadir un puerto en lugar de compartirlo, la no interferencia
queda garantizada por construcción y no por convención.

Este documento es la **fuente única de verdad** del formato. Existen dos implementaciones
del mismo contrato —el emisor en Rust y el receptor en C— y ambas deben derivarse de esta
especificación.

---

## 2. Capa física

| Parámetro | Valor |
|---|---|
| Tipo | UART asíncrono |
| Velocidad | 115 200 baud |
| Formato | 8N1, sin control de flujo |
| Puerto en el LLC | USART libre del ATmega2560 (ver nota 2.1) |
| Puerto en el HLC | Puerto USB dedicado, mediante conversor USB-TTL |
| Dirección | **Unidireccional**, LLC → HLC |
| Niveles lógicos | 5 V TTL en el lado del ATmega2560 |

**2.1 Selección del USART.** El mapa de periféricos serie del ATmega2560 no admite un
tercer enlace nuevo: USART1 está bloqueado de forma permanente porque sus pines coinciden
con las interrupciones de dos encoders, y USART2 está ocupado por el sensor LiDAR. Queda
libre exactamente uno de USART0 y USART3, según cuál emplee el enlace heredado. El Canal 2
toma el que quede libre.

> **Asunto abierto.** La asignación definitiva depende de si el enlace heredado se traslada
> a USART3, como indica la configuración de producción del firmware. Debe cerrarse antes de
> la implementación del firmware.

**2.2 Adaptación de niveles.** Si el conversor USB-TTL opera a 3,3 V, la línea de
transmisión del ATmega2560 requiere un divisor resistivo. Al ser el canal unidireccional,
no se requiere adaptación en sentido contrario.

**2.3 Justificación de la unidireccionalidad.** El canal no transporta comandos. Esta
decisión elimina por diseño toda posibilidad de que el subsistema de estimación interfiera
con el control del róver, incluso ante un fallo de software en el HLC.

---

## 3. Estructura de trama

Trama de longitud fija de **50 bytes**:

```
┌─────────┬─────────────────────┬──────────────────┐
│  0xA5   │                     │                  │
│  0x5A   │   carga útil 46 B   │   CRC-16  2 B    │
│  (2 B)  │                     │   (lo, hi)       │
└─────────┴─────────────────────┴──────────────────┘
```

| Campo | Tamaño | Descripción |
|---|---|---|
| Sincronismo | 2 B | Patrón fijo `0xA5 0x5A` |
| Carga útil | 46 B | Ver sección 4 |
| CRC-16 | 2 B | CRC-16/CCITT sobre los 46 bytes de carga útil, orden little-endian |

**Orden de bytes.** Little-endian en todos los campos multibyte, coincidiendo con la
representación nativa del AVR y de AArch64. No se requiere conversión en ninguno de los dos
extremos.

---

## 4. Carga útil

Los campos se declaran ordenados de mayor a menor requisito de alineación. Esto no es una
convención de estilo: garantiza que una declaración `#[repr(C)]` en Rust y una `struct` en
C produzcan idénticos desplazamientos de campo tanto en el AVR de 8 bits como en AArch64,
sin necesidad de empaquetado ni de relleno explícito.

```c
typedef struct {                /* 46 bytes, sin padding en AVR ni en AArch64 */
    int32_t  enc[6];            /* 24 B  cuentas acumuladas, orden FR FL CR CL RR RL */
    uint32_t seq;               /*  4 B  número de secuencia                          */
    uint32_t t_llc_ms;          /*  4 B  reloj del LLC en el instante de muestreo     */
    int16_t  acc[3];            /*  6 B  acelerómetro crudo, ejes X Y Z               */
    int16_t  gyr[3];            /*  6 B  giroscopio crudo, ejes X Y Z                 */
    uint8_t  stall_mask;        /*  1 B  ver sección 4.4                              */
    uint8_t  flags;             /*  1 B  ver sección 4.5                              */
} pe_frame_t;

_Static_assert(sizeof(pe_frame_t) == 46, "ICD-PE-002: tamano de trama inconsistente");
```

La declaración equivalente en Rust vive en el firmware del LLC con el mismo orden de campos
y una aserción de tamaño análoga. **Ambos extremos deben verificar el tamaño en tiempo de
compilación**; una discrepancia silenciosa produce un desalineamiento que el CRC no
detecta, porque la trama sigue siendo consistente consigo misma.

### 4.1 Cuentas de encoder — `enc[6]`

Cuentas **acumuladas y con signo**, no incrementos. Orden: FR, FL, CR, CL, RR, RL.
Decodificación en cuadratura ×2 —ambos flancos de la fase A, fase B leída por GPIO para el
signo—. Envuelven en $2^{32}$; la diferencia entre lecturas consecutivas es correcta a
través del envolvimiento con aritmética de enteros con signo.

**Justificación.** Transmitir cuentas acumuladas en lugar de incrementos tiene dos efectos.
Primero, el residuo sub-cuenta se arrastra entre intervalos, de modo que el error de
cuantización permanece acotado por una cuenta con independencia del número de intervalos;
transmitir incrementos ya cuantizados introduciría un sesgo determinista de redondeo.
Segundo, una trama perdida no produce pérdida de desplazamiento, sino únicamente un paso de
integración más largo.

### 4.2 Número de secuencia — `seq`

Incrementa en uno por trama emitida y envuelve en $2^{32}$. Permite al receptor distinguir
pérdida de trama de retraso de trama, y constituye la base de medición del requisito de
tasa de entrega de mensajes.

### 4.3 Marca de tiempo — `t_llc_ms`

Reloj en milisegundos del LLC **en el instante de muestreo**, no en el de transmisión. Es la
referencia que permite estimar la latencia extremo a extremo y el desfase entre los relojes
de ambos controladores.

### 4.4 Máscara de bloqueo — `stall_mask`

Un bit por rueda, activo en alto. Bit 5 = FR, bit 4 = FL, bit 3 = CR, bit 2 = CL,
bit 1 = RR, bit 0 = RL. Bits 6 y 7 reservados, transmitidos en cero.

### 4.5 Indicadores — `flags`

| Bit | Nombre | Descripción |
|---|---|---|
| 0 | `imu_ok` | La lectura del IMU de este ciclo fue válida |
| 1 | `enc_ok` | Los seis encoders respondieron en este ciclo |
| 2–3 | `safety` | Nivel de seguridad del LLC |
| 4–7 | — | **Reservados**, transmitidos en cero |

> Los bits 4 a 7 están disponibles para extensiones. Un uso previsto es señalizar la
> presencia de un campo de temperatura del IMU, si se decide transportarlo para compensar
> la deriva térmica del sesgo del giroscopio.

---

## 5. Temporización y presupuesto de enlace

| Parámetro | Valor |
|---|---|
| Cadencia de emisión | 50 Hz, una trama por ciclo de control del LLC |
| Periodo | 20 ms, coincidente con `LOOP_MS` del firmware |
| Carga por trama | 50 B × 10 bits = 500 bits |
| Tasa de datos | 25 000 bit/s |
| Utilización del enlace | **21,7 %** a 115 200 baud |

La emisión está sincronizada con el ciclo de control, de modo que el intervalo entre
muestras no presenta jitter atribuible al canal.

El margen del 78 % permite elevar la cadencia o ampliar la trama sin cambiar la velocidad
de línea. El tiempo de cómputo añadido al ciclo del LLC por la lectura del IMU y el armado
de la trama no debe superar los 5 ms, conforme al requisito correspondiente del documento
padre.

---

## 6. Comportamiento del receptor

1. Buscar el patrón de sincronismo `0xA5 0x5A` en el flujo de entrada.
2. Leer los 48 bytes siguientes.
3. Calcular el CRC-16/CCITT sobre los primeros 46 y compararlo con los 2 últimos.
4. Si el CRC no coincide, **descartar la trama completa** y volver al paso 1.
5. Si coincide, comparar `seq` con la trama anterior. Una discrepancia mayor que uno indica
   trama perdida y debe contabilizarse.
6. Calcular los incrementos de cuenta por diferencia con la trama anterior válida.

**El receptor no reconstruye tramas parciales ni interpola valores ausentes.** Ante pérdida
de trama, el paso de integración siguiente abarca un intervalo mayor, situación que el
estimador maneja con el argumento de tiempo transcurrido.

---

## 7. Verificación

| Requisito | Método |
|---|---|
| Cadencia ≥ 50 Hz | Medición del intervalo entre `seq` consecutivos durante 10 min; percentil 99 ≤ 20 ms |
| Integridad de trama | Inyección de errores de bit; verificar que toda trama corrupta se descarta |
| Tasa de entrega | Conteo por `seq` sobre ≥ 10⁵ tramas; pérdidas ≤ 1 % |
| No interferencia | Inspección: el protocolo heredado conserva su formato y su cadencia; el software desplegado no requiere modificación |
| Consistencia de formato | Aserción estática de tamaño en ambos extremos, verificada en compilación |

---

## 8. Asuntos abiertos

| ID | Asunto | Efecto si no se cierra |
|---|---|---|
| A-01 | Asignación definitiva del USART (sección 2.1) | Bloquea la implementación del firmware |
| A-02 | Transporte de la temperatura del IMU en los bits reservados de `flags` | Sin ella no es posible compensar la deriva térmica del sesgo |
| A-03 | Confirmación del conversor USB-TTL y de la necesidad de divisor resistivo | Riesgo de daño al puerto del conversor |

---

## 9. Historial de revisiones

| Versión | Fecha | Cambios |
|---|---|---|
| v1.0 | Semana 4 | Emisión inicial como documento independiente. El contenido procede del Anexo A de DRT-SEP-001 v0.1, ampliado con el comportamiento del receptor, la matriz de verificación y los asuntos abiertos. |
