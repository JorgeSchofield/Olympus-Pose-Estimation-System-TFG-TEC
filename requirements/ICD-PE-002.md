# ICD-PE-002: Canal de sensores del subsistema de estimación de pose

| Campo | Valor |
|---|---|
| Identificador | ICD-PE-002 |
| Versión | v2.0 |
| Estado | Borrador para revisión |
| Título | Interfaz de sensores LLC → HLC para estimación de pose |
| Emisor | LLC: ATmega2560, firmware Rust `no_std` v2.20 |
| Receptor | HLC: Raspberry Pi 5, agente de adquisición CMAES |
| Documento padre | DRT-SEP-001 v0.3 (`PE-RF-002`, `PE-RF-003`, `PE-RF-008`, `PE-RNF-006`, `PE-RNF-008`, `PE-CON-008`) |
| Documentos relacionados | ICD-LLC-001 v1.3 (telemetría heredada; **este documento no la modifica**); ICD-LLC-002 v1.1 (contrato de datos de la trama binaria SENSOR) |
| Implementación de referencia | Modelo de simulación v2.0: `raw_frame_bytes.m`, `raw_frame_from_bytes.m`, `sep_frame_pack.m`, `sep_frame_parse.m`, `sep_crc16.m`, `test_sep_frame.m` |

---

## 1. Propósito y alcance

Este documento especifica cómo el controlador de bajo nivel entrega al controlador de alto
nivel las cuentas de los encoders y las mediciones de la IMU que necesita el estimador de
pose. Define dos formatos:

- **Formato vigente (RAW).** La trama de texto que el firmware v2.20 ya emite en cada ciclo.
  Es el formato que usa el modelo de simulación v2.0 en su camino de datos y el que usará el
  subsistema mientras el firmware no cambie.
- **Formato objetivo (SENSOR).** La trama binaria de 55 bytes con CRC del contrato
  ICD-LLC-002 v1.1. Está implementada y verificada en el modelo, pero no está en el camino
  de datos. Se adopta si el firmware del LLC se modifica por otra razón (ver § 7).

**Cambio respecto a v1.0.** La v1.0 definía un canal físico separado sobre el USART libre,
con una trama binaria de 50 bytes. El modelo v2.0 mostró que el firmware ya entrega los
datos necesarios por el enlace existente, sin afectar la telemetría heredada; el canal
separado queda como alternativa (§ 5.2).

---

## 2. Capa física

| Parámetro | Valor |
|---|---|
| Tipo | UART asíncrono |
| Puerto en el LLC | USART0 |
| Puerto en el HLC | USB (conversor integrado del Arduino Mega) |
| Velocidad | 115 200 baud (no 500 000) |
| Formato | 8N1, sin control de flujo; 10 bits por byte |
| Compartición | El mismo enlace transporta la TLM heredada de ICD-LLC-001 una vez cada 50 ciclos y los comandos con semántica ACK/ERR. Los prefijos distinguen cada tipo de línea. |

---

## 3. Formato vigente: trama RAW

### 3.1 Estructura

Una línea de texto ASCII por ciclo del LLC:

```
RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>\n
```

| Campo | Tipo | Unidad | Descripción |
|---|---|---|---|
| `RAW:` | literal | | Prefijo que distingue la trama de sensores de la TLM |
| `tick` | u32 decimal | ms | Contador de tiempo del LLC (ver § 3.3) |
| `ax`, `ay`, `az` | i16 decimal | LSB | Acelerómetro crudo; ±2 g, 16 384 LSB/g |
| `gx`, `gy`, `gz` | i16 decimal | LSB | Giroscopio crudo; ±250 °/s, 131 LSB/(°/s) |
| `encL`, `encR` | i32 decimal | cuentas | Acumuladores por lado: suma de las tres ruedas de cada lado, decodificación ×2 |
| `\n` | literal | | Fin de trama |

Los enteros se escriben con la conversión del firmware (`write_u32` y `write_i32` de
`main.rs`): sin ceros a la izquierda, sin signo de más, y los negativos con un guion seguido
de la magnitud.

### 3.2 Longitud

La longitud es **variable**, entre ≈ 40 y 81 bytes según los valores. El peor caso es
4 + 10 + 6 × 7 + 2 × 12 + 1 = 81 bytes. El firmware reserva un búfer de 100 bytes
(`raw_buf`); el receptor reserva el mismo tamaño. Por eso el tiempo de transmisión también
varía: hasta 7,0 ms por trama a 115 200 baud.

### 3.3 Marca de tiempo

`tick` es el contador de tiempo del LLC en milisegundos. El estimador calcula $\Delta t$
como la diferencia entre los `tick` de dos tramas consecutivas.

> **Advertencia: hipótesis sin verificar.** El modelo v2.0 predice que el firmware actual
> suma 20 ms al contador en cada ciclo sin medir la duración real del trabajo del ciclo, de
> modo que `tick` avanzaría más lento que el tiempo real (≈ 65 % según el modelo). Esto **no
> se ha verificado en el róver**, porque el cambio de firmware que activa la IMU todavía no
> se ha probado. Hasta verificarlo, toda medición de cadencia y de latencia se hace con el
> reloj del HLC (DRT-SEP-001, `PE-RF-003` y `R-02`). La verificación propuesta es comparar,
> sobre una ventana larga, el avance de `tick` con las marcas de tiempo que el HLC asigna a
> cada trama.

### 3.4 Integridad

La trama RAW **no tiene CRC**. Un byte alterado dentro de un número produce un valor
creíble que el analizador de texto no detecta. La única defensa disponible en este formato
es el rechazo por plausibilidad del § 6.

---

## 4. Formato objetivo: trama SENSOR (ICD-LLC-002 v1.1)

### 4.1 Estructura

Trama binaria de longitud fija de **55 bytes**:

| Offset | Tamaño | Campo | Valor o descripción |
|---|---|---|---|
| 0 | 1 | SOF0 | 0xAA (bit 7 en 1: imposible en ASCII de 7 bits) |
| 1 | 1 | SOF1 | 0x55 |
| 2 | 1 | VER | 0x01 |
| 3 | 1 | TYPE | 0x01 (SENSOR) |
| 4 | 1 | LEN | 48 |
| 5 | 48 | Carga útil | § 4.2 |
| 53 | 2 | CRC | CRC-16/CCITT-FALSE sobre los bytes 2 a 52, little-endian |

### 4.2 Carga útil (48 bytes, little-endian)

| Offset | Tipo | Campo | Descripción |
|---|---|---|---|
| 0 | u32 | `t_llc_us` | Instante de muestreo en microsegundos |
| 4 | i32[6] | `enc` | Acumuladores en orden FR, FL, CR, CL, RR, RL |
| 28 | u16 | `seq` | Número de secuencia |
| 30 | u16 | `tx_drop` | Tramas que el LLC no pudo transmitir |
| 32 | i16[3] | `acc` | Acelerómetro crudo (registro 0x3B) |
| 38 | i16 | `imu_temp` | Temperatura cruda de la IMU (registro 0x41) |
| 40 | i16[3] | `gyr` | Giroscopio crudo (registro 0x43) |
| 46 | u8 | `flags` | Indicadores; bit 6 = `ENC_SIDE_SUM` |
| 47 | u8 | `stall_mask` | Motores bloqueados |

Los 14 bytes de los offsets 32 a 45 son la ráfaga I²C 0x3B a 0x48 del MPU-9250 en su orden
nativo, convertida a little-endian.

**Modo suma por lado.** Si el LLC suma las cuentas antes de transmitir, escribe la suma del
lado derecho en `enc[0]`, la del izquierdo en `enc[1]`, deja `enc[2..5]` en cero y activa
`ENC_SIDE_SUM`. Es un cambio aditivo: un receptor que ignore el bit no interpreta la trama
como seis ruedas casi detenidas, porque debe verificar el bit antes de usar `enc`.

### 4.3 Ventajas frente a RAW

- Integridad por CRC.
- Número de secuencia para contar pérdidas en el propio enlace.
- Marca de tiempo en microsegundos.
- Temperatura de la IMU, necesaria para compensar la deriva térmica del sesgo.
- Longitud fija: tiempo de transmisión constante de 4,8 ms.

### 4.4 Verificación del formato

`test_sep_frame.m` reproduce el vector de prueba del contrato de datos, que también
verifican las implementaciones en C y en Python. Ambos extremos deben comprobar el tamaño de
la estructura en tiempo de compilación.

---

## 5. Temporización y carga del enlace

### 5.1 Carga

| Formato | Bytes por trama | Bytes por segundo a 50 Hz | Uso de los 11 520 B/s del enlace |
|---|---|---|---|
| RAW (peor caso) | 81 | 4050 | 35,2 % |
| SENSOR | 55 | 2750 | 23,9 % |
| TLM heredada | 185, cada 50 ciclos | 185 | 1,6 % |

### 5.2 Alternativa de canal separado

Si la carga de USART0 o la transmisión bloqueante del firmware resultaran un problema, la
alternativa es un canal físico separado sobre USART3 con un conversor USB-serial dedicado
(diseño de la v1.0). USART1 está bloqueado por las interrupciones de dos encoders y USART2
por el LiDAR.

---

## 6. Comportamiento del receptor

El agente de adquisición:

1. Separa las líneas por `\n` y procesa solo las que empiezan con `RAW:`. Las líneas de TLM y
   las respuestas ACK/ERR pertenecen al software heredado.
2. Decodifica campo por campo. **Nunca** convierte el búfer de recepción directamente a una
   estructura.
3. Rechaza la trama si falta el prefijo, si el número de campos no es diez o si algún campo
   no es un entero decimal.
4. Deposita en el buzón los acumuladores `encL` y `encR` **sin convertir**, junto con `tick`
   y el giroscopio. Las diferencias se calculan en el agente que las consume, contra el
   último valor que él mismo procesó (DRT-SEP-001, `PE-RF-008`).
5. Envía al buzón con tiempo límite cero: si está lleno, la trama nueva se descarta y la
   distancia no se pierde, porque la siguiente trae el acumulado completo.

El agente de estimación rechaza por plausibilidad (`PE-CON-008`):

- $\Delta t \le 0$ o $\Delta t > 0{,}5$ s;
- un salto de cuenta de 20 000 o más en cualquiera de los lados.

**El receptor no reconstruye tramas parciales ni interpola valores ausentes.** Ante una trama
perdida o rechazada, el paso siguiente del filtro abarca un intervalo mayor.

---

## 7. Condiciones para migrar al formato SENSOR

La migración no es un requisito del SEP. Se recomienda si el firmware del LLC se modifica por
otra razón, en particular si la verificación del reloj (§ 3.3) confirma que hay que cambiar
el lazo a un temporizador. En ese caso, el mismo cambio puede adoptar la trama SENSOR en modo
suma por lado y conservar la compatibilidad con la TLM heredada.

---

## 8. Verificación

| Requisito | Método |
|---|---|
| Cadencia ≥ 50 Hz | Intervalo entre tramas medido **con el reloj del HLC** durante 10 min; p99 ≤ 20 ms |
| Escala del reloj del LLC | Regresión del avance de `tick` contra las marcas de tiempo del HLC sobre una ventana larga |
| Formato RAW | Pruebas del analizador con tramas válidas, truncadas, con campos de más o de menos y con caracteres no numéricos |
| Plausibilidad | Inyección de corrupción de un byte; verificar el rechazo por $\Delta t$ o por salto de cuenta |
| Formato SENSOR | Vector de prueba de `test_sep_frame`; inyección de errores de bit con descarte por CRC |
| No interferencia | Inspección: la TLM heredada conserva formato y cadencia; el software en producción no cambia |

---

## 9. Asuntos abiertos

| ID | Asunto | Efecto si no se cierra |
|---|---|---|
| A-01 | Verificar la escala del contador `tick` después de probar el firmware con la IMU activa (§ 3.3). | Sin ella no se sabe si el firmware debe cambiar ni si el $\Delta t$ es confiable. |
| A-02 | Decidir si se migra a la trama SENSOR. | Sin CRC, la integridad depende solo de la plausibilidad. |
| A-03 | Transporte de la temperatura de la IMU en el formato RAW. | Sin ella no se puede compensar la deriva térmica del sesgo. |

---

## 10. Historial de revisiones

| Versión | Fecha | Cambios |
|---|---|---|
| v1.0 | Semana 4 | Emisión inicial: canal separado con trama binaria de 50 B. |
| v2.0 | Semana 9 | Alineación con el modelo de simulación v2.0: formato vigente RAW sobre USART0; formato objetivo SENSOR de ICD-LLC-002 v1.1; advertencia sobre el reloj del LLC; comportamiento del receptor con acumuladores sin convertir y envío con tiempo límite cero. El canal separado pasa a alternativa. |
