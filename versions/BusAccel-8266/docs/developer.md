# Developer — BusAccel-8266 (documentación técnica del código)

> **Documento correspondiente al firmware `BusAccel-8266.ino` — CICLO 16**
> («SALIDA SERIAL CSV PARA GRAFICADO», ASCII puro, 505 líneas).
>
> Este documento describe **exclusivamente el código actual**: el sketch y la
> **aplicación de escritorio `app/`**. El sketch **no** contiene WiFi, servidor
> web, HTML, MQTT, EEPROM ni `secrets.h`. Cualquier referencia a conectividad en
> este documento es **histórica** (Fase 1, ya eliminada) o se refiere a la
> **fase futura planificada** (§10), que **no está implementada**.

---

## 1. Arquitectura general

### 1.1 Resumen en una frase

Un único sketch, sin tareas ni interrupciones, con un bucle `loop()` no
bloqueante basado en `millis()` que (a) muestrea el ADXL345 a **100 Hz** por I²C,
(b) calcula la desviación de cada eje respecto a un *setpoint* adaptativo,
(c) decide **un único carácter ASCII** por evento mediante una máquina de estados
de dos estados, y (d) imprime **una línea CSV de 8 campos cada 50 ms** (20 Hz)
por la consola serie a 115200 baud. En el PC, `app/` lee esas líneas, las
parsea y las dibuja en un gráfico de canvas en tiempo real (§11).

### 1.2 Diagrama de módulos

```
┌───────────────────────────────────────────────────────────────────────┐
│                          BusAccel-8266.ino                            │
│                                                                       │
│  ┌────────────────────────────┐    ┌───────────────────────────────┐  │
│  │  CAPA I2C (Wire)           │    │  CAPA DE CONVERSIÓN           │  │
│  │  readReg(reg)      -> u8   │    │  readRawAxes()  -> rawX/Y/Z   │  │
│  │  writeReg(reg,val) -> bool │    │       |                       │  │
│  │  readRawAxes()    -> bool  │    │       v                       │  │
│  └────────────────────────────┘    │  readAccel() -> gx,gy,gz      │  │
│              |                     │     (raw/256.0f, rango ±2g)    │  │
│              v                     └───────────────┬───────────────┘  │
│  ┌────────────────────────────┐                    │                  │
│  │  DIAGNÓSTICO (1 sola vez)  │                    v                  │
│  │  bwRateStr(code)           │    ┌───────────────────────────────┐  │
│  │  runChipDiagnostic()       │    │  NÚCLEO DE DETECCIÓN          │  │
│  │    (WHO_AM_I, POWER_CTL,   │    │  processSample()              │  │
│  │     DATA_FORMAT, BW_RATE,  │    │    - setpoint inicial (20)    │  │
│  │     OFS, force measure,    │    │    - desviación devX/Y/Z      │  │
│  │     1ª muestra cruda)      │    │    - eje ganador + severidad  │  │
│  └────────────────────────────┘    │    - máquina IDLE/LOCKED      │  │
│                                    │    - recalculo setpoint ×20    │  │
│                                    └───────────────┬───────────────┘  │
│                                                    │ eventChar +       │
│                                                    │ symbolPending    │
│                                                    v                  │
│                                    ┌───────────────────────────────┐  │
│                                    │  SALIDA CSV (20 Hz)           │  │
│                                    │  printCharTick()              │  │
│                                    │   t,X,Y,Z,setX,setY,setZ,sym  │  │
│                                    │   (1 línea completa por tick) │  │
│                                    └───────────────┬───────────────┘  │
└────────────────────────────────────────────────────┼──────────────────┘
                                                      v
                                             Serial @ 115200 8N1
                                        (sin WiFi · sin web · sin MQTT)
                                                      │
              ┌───────────────────────────────────────┘
              v
┌───────────────────────────────────────────────────────────────────────┐
│                     app/  (PC — Python 3 + pywebview)                  │
│                                                                       │
│  serial_reader.py                         app.py                      │
│  ┌──────────────────────────────┐    ┌─────────────────────────────┐  │
│  │ SerialReader (hilo daemon)   │    │ ThreadingHTTPServer 127.0.0.1│ │
│  │  _serial_loop()  pyserial    │    │   GET /api/state?since=N     │ │
│  │  _simulator_loop() 20 Hz     │───>│   GET /api/ports|connect|…   │ │
│  │  parse_line()  8 campos      │    │   GET /, /app.js, /style.css │ │
│  │  deque(maxlen=6000) puntos   │    │   webview.create_window()    │ │
│  │  conteo por símbolo          │    │   fallback: webbrowser.open()│ │
│  └──────────────────────────────┘    └──────────────┬──────────────┘  │
│                                                      v HTTP  │
│                                          ┌───────────────────────────┐  │
│                                          │ app.js  (canvas 2D)       │  │
│                                          │  poll() 40 ms (~25 fps)   │  │
│                                          │  ventana deslizante 10-120s│ │
│                                          │  zonas leve/severo        │  │
│                                          │  línea punteada setpoint  │  │
│                                          │  marcadores con glow      │  │
│                                          └───────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────┘
```

### 1.3 Principios de diseño

| Principio | Cómo se materializa en el código |
|---|---|
| **Sin dependencias de red** | Ni `ESP8266WiFi.h`, ni `ESP8266WebServer.h`, ni `PubSubClient.h`, ni `EEPROM.h`. Compilar y ejecutar no depende de credenciales ni de router. |
| **Muestreo e impresión desacoplados** | `processSample()` corre a 100 Hz; `printCharTick()` a 20 Hz. La cadencia de salida es independiente de la de adquisición. |
| **Un símbolo = un evento** | La máquina de estados `ST_IDLE`/`ST_LOCKED` garantiza que un mismo bache no produzca `+` y luego `-` por el rebote inercial del ADXL345. |
| **Acceso directo a registros** | La ruta crítica de lectura es `readRawAxes()` (burst de 6 bytes) sobre `Wire`, sin capa de objetos: mínima latencia y traza física del dispositivo. |
| **Sin `delay()` en `loop()`** | Toda la temporización usa `millis()`. El único `delay(50)` está en `runChipDiagnostic()`, ejecutado una sola vez en `setup()`. |
| **ASCII puro** | Sólo dígitos, coma, punto, signo y los cinco símbolos. Ningún byte UTF-8 ni secuencia ANSI: cualquier terminal los muestra bien. |
| **Visualización fuera del microcontrolador** | Desde el ciclo 16 el sketch **no** dibuja nada: publica datos. Toda la presentación (gráfico, colores, estadísticas) vive en `app/`, en el PC, sin coste de RAM ni de CPU para el ESP8266. |
| **Contrato de datos explícito** | La línea CSV es la **API** entre firmware y app: 8 campos separados por coma, `\n` final, el símbolo como último campo. El parser de la app valida longitud y tipos, de modo que texto del arranque se ignora sin coordinación adicional. |
| **Reporte desacoplado del conteo** | La salida es un flujo de muestras, no un estado agregado. La fase futura podrá sustituir `printCharTick()` por un publicador MQTT sin tocar la detección. |


### 1.4 Include y macros del sketch

| Línea | Elemento | Rol |
|---|---|---|
| 57 | `#define MODE_DIAGNOSTIC_SERIAL 1` | Bandera documental: `1` = sólo consola serie. No se usa en ninguna condición del código. |
| 59 | `#include <Wire.h>` | Bus I²C del ESP8266. |
| 60 | `#include <math.h>` | `fabsf()` para comparar magnitudes de desviación. |
| 61 | `#include <Adafruit_Sensor.h>` | Dependencia de la librería Unified. |
| 62 | `#include <Adafruit_ADXL345_U.h>` | Driver del acelerómetro (usado **sólo** para `begin()`, `setRange()`, `setDataRate()`; la lectura de datos es directa por I²C). |

---

## 2. Constantes de configuración (líneas 64–136)

Todas las constantes están en el **preámbulo** y son el único punto que hay que
tocar para reparametrizar el instrumento. No hay `EEPROM` ni menú de
configuración: **recompilar es el ajuste**.

### 2.1 Adquisición e impresión

| Constante | Valor | Línea | Significado / criterio |
|---|---|---|---|
| `BAUD_RATE` | `115200` | 67 | Velocidad de `Serial`. 8N1, sin control de flujo. |
| `SENSOR_RANGE` | `ADXL345_RANGE_2_G` | 68 | Rango del ADXL345. **Debe coincidir con `DATA_FORMAT & 0x03 == 0x00`**, porque la conversión usa `raw/256.0f` (256 LSB/g). |
| `SENSOR_DATARATE` | `ADXL345_DATARATE_100_HZ` | 69 | ODR del sensor = 100 Hz = Nyquist de la salida a 20 Hz. |
| `SAMPLE_INTERVAL_MS` | `10` | 70 | Muestreo interno → 100 Hz. |
| `PRINT_INTERVAL_MS` | `50` | 71 | Impresión → 20 Hz, **1 línea CSV por tick** (ciclo 16). |
| `SENSOR_RETRY_MS` | `1000` | 72 | Reintento en background si el sensor no responde. Evita spam: `notDetectedReported` asegura un único mensaje. |

### 2.2 Setpoint y umbrales de detección

| Constante | Valor | Línea | Significado / criterio |
|---|---|---|---|
| `SETPOINT_N` | `20` | 81 | Muestras por eje para el promedio del setpoint. Ciclo 14: bajó de 50 a 20 por petición del usuario («50 lecturas es mucho»). **Se usa tanto en la adquisición inicial como en el recálculo periódico**, de modo que ambos son coherentes. |
| `ARROW_THRESHOLD_G` | `0.20f` | 82 | Desviación mínima (g) para que exista evento (`+`/`*`/`-`/`!`). Ciclo 13: bajó de 0.30 g a 0.20 g → más sensible. **Ajustable en campo.** |
| `SEVERE_FACTOR` | `1.2f` | 94 | Multiplicador que define la categoría severa. |
| `SEVERE_THRESHOLD_G` | `(ARROW_THRESHOLD_G * SEVERE_FACTOR)` → **0.24 g** | 95 | Umbral severo. **Derivado**: cambiar `ARROW_THRESHOLD_G` reescala automáticamente el severo. Historial: 0.45 g → 0.30 g (ciclo 13) → 0.24 g (ciclo 15, a petición del usuario: «bájalo más aun para `*` y `!`»). |
| `RETURN_G` | `0.15f` | 96 | Histeresis de retorno: todos los ejes deben estar dentro de ±0.15 g para considerar reposo. **Debe ser < `ARROW_THRESHOLD_G`** (0.15 < 0.20) ⇒ volver a reposo es más exigente que disparar, sin incompatibilidad. |
| `LINE_CHARS` | `50` | 104 | ⚠️ **Ciclo 16: definido pero SIN USO.** Era el contador de caracteres por línea del ciclo 15; ya no existe el salto de línea periódico porque cada tick emite su propia línea CSV. Se conserva por trazabilidad; no affecta al comportamiento. |

### 2.3 Supresión del rebote inercial

| Constante | Valor | Línea | Significado / criterio |
|---|---|---|---|
| `EVENT_LOCKOUT_MS` | `250` | 113 | Bloqueo mínimo tras capturar un evento. El rebote inercial del ADXL345 llega ~100–200 ms después y en sentido contrario; 250 ms es el **mínimo viable** que lo anula (por debajo, reaparece el segundo símbolo). Permite estimulación repetida a ~4 eventos/s. |
| `REST_MS` | `50` | 119 | Reposo sostenido exigido para desbloquear = 5 muestras a 100 Hz. Evita que un instante de reposo entre dos rebotes libere el estado. |

> **Relación entre constantes:** `RETURN_G (0.15) < ARROW_THRESHOLD_G (0.20) ≤ SEVERE_THRESHOLD_G (0.24) < EVENT_LOCKOUT_MS (250) / 2`. Y `SAMPLE_INTERVAL_MS (10) < REST_MS (50) = 5 muestras`; `PRINT_INTERVAL_MS (50) = 1 tick` ⇒ el mínimo entre dos símbolos visibles es `EVENT_LOCKOUT_MS + REST_MS + PRINT_INTERVAL_MS` ≈ 350 ms.

### 2.4 Pines y registros

| Constante | Valor | Línea | Significado |
|---|---|---|---|
| `PIN_SDA` | `4` (D2) | 122 | SDA I²C |
| `PIN_SCL` | `5` (D1) | 123 | SCL I²C |
| `ADXL345_I2C_ADDR` | `0x53` | 126 | Dirección I²C del ADXL345 |
| `REG_DEVID` | `0x00` | 127 | WHO_AM_I (debe leer `0xE5`) |
| `REG_OFSX/Y/Z` | `0x1E`/`0x1F`/`0x20` | 128–130 | Offsets de calibración de fabricante |
| `REG_BW_RATE` | `0x2C` | 131 | Output data rate |
| `REG_POWER_CTL` | `0x2D` | 132 | Bit 3 = measurement mode |
| `REG_DATA_FORMAT` | `0x31` | 133 | Bit 0 = full/byte; bits 0-1 = rango |
| `REG_DATAX0` | `0x32` | 134 | Inicio del burst de datos (`Y0 = 0x34`, `Z0 = 0x36`) |

---

## 3. Tipos y estado global (líneas 138–175)

### 3.1 Objeto y sensor

| Símbolo | Tipo | Línea | Descripción |
|---|---|---|---|
| `accel` | `Adafruit_ADXL345_Unified` | 141 | Driver Adafruit. Se usa **sólo** para `begin()`/`setRange()`/`setDataRate()`. El constructor recibe un `12345` arbitrario (identificador de bus interno de la librería Unified; irrelevante para el bus real). |
| `sensorOnline` | `bool` | 143 | `true` si la última lectura I²C de 6 bytes fue correcta. Lo mantiene `readAccel()`. |
| `notDetectedReported` | `bool` | 144 | Evita repetir el mensaje `NO DETECTADO` cada segundo. |
| `gx, gy, gz` | `float` | 145 | Última muestra en **g** (`raw / 256.0f`). |
| `rawX, rawY, rawZ` | `int16_t` | 146 | Últimos valores crudos leídos del burst. |

### 3.2 Setpoint y desviación

| Símbolo | Tipo | Línea | Descripción |
|---|---|---|---|
| `setpointReady` | `bool` | 149 | `false` ⇒ aún adquiriendo el setpoint inicial; en ese estado `loop()` **no imprime** el flujo de 20 Hz (sólo los puntos de adquisición). |
| `spX, spY, spZ` | `float` | 150 | Setpoint por eje (g): media de `SETPOINT_N` muestras en reposo. |
| `devX, devY, devZ` | `float` | 151 | Desviaciones instantáneas: `gx - spX`, `gy - spY`, `gz - spZ`. |
| `accSumX/Y/Z` | `float` | 152 | Acumulador del promedio del setpoint. |
| `accCount` | `uint16_t` | 153 | Contador de muestras acumuladas; al alcanzar `SETPOINT_N` se calcula la media y se pone a cero. |

### 3.3 Máquina de estados de detección

```cpp
enum DetectState : uint8_t { ST_IDLE = 0, ST_LOCKED = 1 };
```

| Símbolo | Tipo | Línea | Descripción |
|---|---|---|---|
| `detectState` | `DetectState` | 162 | Estado actual de la máquina. |
| `eventChar` | `char` | 163 | Símbolo capturado en el primer cruce (`'+'`, `'*'`, `'-'`, `'!'`). |
| `symbolPending` | `bool` | 164 | `true` mientras el símbolo único aún no se ha impreso. Desacopla la captura (100 Hz) de la emisión (20 Hz). |
| `lockUntilMs` | `unsigned long` | 167 | Instante `millis()` mínimo en el que se permite desbloquear. |
| `restSinceMs` | `unsigned long` | 168 | Instante en que arrancó el reposo sostenido (`0` = sin reposo). |
| `printCount` | `uint16_t` | 171 | Caracteres impresos en la línea actual. |
| `lastSampleMs` / `lastPrintMs` / `lastRetryMs` | `unsigned long` | 173–175 | Marcas de la temporización no bloqueante. |

---

## 4. API de funciones

Todas las funciones son **`void` o `bool`/`char`/`uint8_t`, sin parámetros de
clase**, declaradas antes de su uso. No hay callbacks, ni `attachInterrupt()`, ni
tareas FreeRTOS (el core ESP8266 Arduino las oculta tras el scheduler).

### 4.1 Acceso directo a registros (líneas 177–208)

```cpp
uint8_t readReg(uint8_t reg);
```
Lee **un** registro del ADXL345 por I²C.
Secuencia: `beginTransmission(0x53)` → `write(reg)` → `endTransmission(false)`
(sin stop) → `requestFrom(0x53, 1)` → `read()`.
**Devuelve `0xFF` ante cualquier fallo** de ACK o de longitud, de modo que un
cable roto se manifiesta como un valor improbable en lugar de colgar el bus.

```cpp
bool writeReg(uint8_t reg, uint8_t val);
```
Escribe un registro. Devuelve `true` si `endTransmission(true) == 0` (ACK del
slave). Se usa para forzar `POWER_CTL = 0x08`.

```cpp
bool readRawAxes();
```
**Función crítica de la ruta de datos.** Lee los 6 bytes `DATAX0..DATAZ1` en una
sola transacción *burst* y los ensambla en tres `int16_t` little-endian:

```
[X0 X1 Y0 Y1 Z0 Z1]  ->  rawX = x1<<8 | x0   (y análogamente Y, Z)
```

Devuelve `false` si falla la dirección o si `requestFrom` no devuelve
exactamente 6 bytes. **No hay bytes de relleno**: el ADXL345 entrega
justo los 6 solicitados. La ventaja del burst frente a
tres lecturas separadas es **coherencia temporal**: los tres ejes corresponden a
la misma muestra.

### 4.2 Diagnóstico del chip (líneas 210–279)

```cpp
static const char* bwRateStr(uint8_t code);
```
Traduce los 4 bits bajos de `BW_RATE` a texto (`"3200 Hz"` … `"0.10 Hz"`) mediante
un `switch` sobre `code & 0x0F`. Es una función de presentación pura: no toca I²C
ni estado.

```cpp
void runChipDiagnostic();
```
Se invoca **una única vez** desde `setup()` y **siempre**, incluso si
`accel.begin()` falló (lee por I²C directo). Secuencia:

1. Cabecera `=== ADXL345 DIAGNOSTICO PROFUNDO ===`.
2. Lectura y valoración de `DEVID` (`0xE5` = OK; otro valor = clon/defecto o mala
   lectura I²C).
3. Lectura de `POWER_CTL` con interpretación del **bit 3** (measurement mode vs
   STANDBY).
4. Lectura de `DATA_FORMAT` con los **bits 0-1** → rango `±2g / ±4g / ±8g / ±16g`.
5. Lectura de `BW_RATE` (valor + código + texto) y de `OFSX/OFSY/OFSZ` (offsets de
   fabricante, en `int8_t`).
6. **Forzado de measurement mode**: escribe `0x08` en `POWER_CTL`, `delay(50)`, y
   vuelve a leer para **verificar que persiste**. Si no persiste ⇒ diagnóstico
   explícito de «problema de escritura I²C (revisar cableado/soldadura)». Este es
   el fallo clásico de los clones baratos del ADXL345.
7. **Primera muestra cruda** (`PRIMERA MUESTRA rawX=… rawY=… rawZ=…`) o mensaje
   de error de I²C.
8. `=== FIN DIAGNOSTICO ===`.

### 4.3 Lectura y conversión (líneas 281–295)

```cpp
bool readAccel();
```
Llama a `readRawAxes()` y, si tiene éxito, actualiza `sensorOnline = true` y
convierte:

```
gx = rawX / 256.0f;   gy = rawY / 256.0f;   gz = rawZ / 256.0f;
```

El factor **256** corresponde a **256 LSB/g**, es decir al rango **±2 g**. Si se
cambia `SENSOR_RANGE` a ±4g (±8g) hay que cambiar el divisor a **128** (**64**)
o las lecturas se saturarán numéricamente. Es la única fuente de datos tanto
para el setpoint como para los símbolos.

### 4.4 Procesado de muestra (líneas 297–409)

```cpp
void processSample();
```
Se llama **sólo** si `readAccel()` devolvió `true`, a ~100 Hz. Dos fases:

**Fase A — adquisición inicial (`!setpointReady`)**
Acumula `gx/gy/gz` en `accSumX/Y/Z`, incrementa `accCount`, imprime un `'.'` por
lectura, y cuando `accCount >= SETPOINT_N` calcula `spX/spY/spZ`, pone
`setpointReady = true`, **reinicia los acumuladores** e imprime
`setpoint establecido: X=%.2f Y=%.2f Z=%.2f`.
(Ciclo 15 additionally reseteaba `printCount`; en el ciclo 16 ese contador ya no
existe y los 20 puntos quedan en su propia línea porque se emite un `\n` tras
ellos.)

**Fase B — detección en tiempo real (`setpointReady`)**
1. Calcula `devX/devY/devZ`.
2. Elige el **eje ganador**: el de mayor `|desviación|`, Recordando su signo en
   `maxDevPositive`.
3. Calcula dos banderas:
   ```cpp
   eventActive = fabsf(maxDev) > ARROW_THRESHOLD_G;
   allResting  = (|devX| <= RETURN_G && |devY| <= RETURN_G && |devZ| <= RETURN_G);
   ```
4. Ejecuta el `switch (detectState)`.

### 4.5 Máquina de estados de detección (el núcleo)

```
                    eventActive (|dev| > 0.20 g)
      ┌──────────┐  ─────────────────────────────────►  ┌──────────┐
      │  ST_IDLE │                                      │ ST_LOCKED │
      │          │  ◄─────────────────────────────────  │          │
      └──────────┘  allResting sostenido REST_MS=50 ms  └──────────┘
            ▲           Y  millis() >= lockUntilMs
            │           (lockout EVENT_LOCKOUT_MS=250 ms ya cumplido)
            │
            │  (mientras LOCKED) la desviación puede cruzar el umbral
            │  en sentido contrario: NO se emite ningún símbolo, sólo '.'
            ▼
      imprime UN símbolo (symbolPending=true) →
      printCharTick() lo emite 1 sola vez y lo consume

      En ST_IDLE y sin evento: acumula muestras; cada SETPOINT_N (20)
      recalcula el setpoint EN SILENCIO (sin imprimir nada)
```

**Transición `ST_IDLE → ST_LOCKED` (líneas 344–368)**

Se captura **un único símbolo** decidiendo por signo **y** magnitud:

| Signo de `maxDev` | Condición | Símbolo |
|---|---|---|
| `maxDevPositive` (≥ 0) | `maxDev >= SEVERE_THRESHOLD_G` (0.24 g) | `'*'` (elevación severa) |
| `maxDevPositive` (≥ 0) | en otro caso | `'+'` (elevación leve) |
| negativo (< 0) | `maxDev <= -SEVERE_THRESHOLD_G` | `'!'` (caída severa) |
| negativo (< 0) | en otro caso | `'-'` (caída leve) |

Después: `symbolPending = true`, `detectState = ST_LOCKED`,
`lockUntilMs = millis() + EVENT_LOCKOUT_MS`, `restSinceMs = 0`.

**Transición `ST_LOCKED → ST_IDLE` (líneas 393–407)**

Se exige **la conjunción de dos condiciones**:

* **(a) lockout temporal cumplido:** `millis() >= lockUntilMs` (han pasado los
  250 ms que anulan el rebote inercial).
* **(b) reposo sostenido:** `allResting` mantenido durante `REST_MS = 50 ms`. El
  cronómetro `restSinceMs` se arranca en la primera muestra en reposo y **se
  reinicia a 0 en cuanto cualquier eje vuelve a superar `RETURN_G`**. Si
  `allResting` es falso, `restSinceMs = 0` y hay que empezar de cero.

> ⚠️ **Detalle sutil:** `restSinceMs == 0` se usa como centinela de «sin reposo».
> Es seguro porque `millis()` sólo es 0 durante los primeros milisegundos tras el
> arranque, y la primera captura de evento no puede ocurrir antes de completar la
> adquisición del setpoint (200 ms a 100 Hz). Si en el futuro se introdujera un
> disparador antes de los 200 ms, habría que sustituir este centinela por un
> `bool restTimerRunning`.

### 4.6 Salida CSV por tick (líneas 423–442)

```cpp
void printCharTick();
```

Se invoca a **20 Hz** desde `loop()` (sólo si `setpointReady`). Desde el ciclo 16
imprime **una línea CSV completa**:

```cpp
void printCharTick() {
  char c = '.';
  if (symbolPending) { c = eventChar; symbolPending = false; }  // se consume 1 vez
  Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%c\n",
                millis(), gx, gy, gz, spX, spY, spZ, c);
}
```

| Campo | Origen | Nota |
|---|---|---|
| `t` | `millis()` | Se imprime el instante **de la impresión**, no el de la muestra (diferencia ≤ 10 ms, irrelevante) |
| `X`,`Y`,`Z` | `gx`,`gy`,`gz` | Ya en g (`raw/256.0f`); `%.2f` |
| `setX`,`setY`,`setZ` | `spX`,`spY`,`spZ` | El setpoint vigente en ese tick; al recalcularse en silencio, **cambia dentro del log** |
| `sym` | `c` | `.` salvo que haya símbolo pendiente |

**Diferencias con el ciclo 15 (importantes para quien compare versiones):**

| Aspecto | Ciclo 15 | Ciclo 16 |
|---|---|---|
| Unidad de salida | 1 carácter por tick | 1 línea CSV por tick |
| Contador de línea | `printCount` + `LINE_CHARS` | **eliminado** (`LINE_CHARS` queda sin uso) |
| Separación temporal | implícita (posición en la línea) | explícita (`t` en ms) |
| Magnitud | no disponible | implícita: `eje − setEje` |
| Referencia | fija (no se emitía) | se emite en cada línea (`setX/setY/setZ`) |
| Parseo | contar caracteres | `split(",")` y validar 8 campos |

El resto de ticks imprimen `sym='.'`, tanto en reposo real como mientras el
estado está `ST_LOCKED` (eso es intencionado: el flujo se mantiene legible y
continuo para poder medir la cadencia real de eventos, y la curva de la app no
tiene huecos).

> ⚠️ **Contrato con la app.** El `printf` **no** debe ganar campos, cambiar el
> separador ni el orden sin actualizar a la vez `parse_line()` en
> `app/serial_reader.py` (§11.2). Es la única interfaz entre firmware y PC.
> Añadir un campo **al final** es seguro; añadirlo antes de `sym` rompe el
> parser actual.


### 4.7 `setup()` (líneas 434–459)

```
Serial.begin(115200) ; delay(300)
print banner "=== BusAccel-8266 DIAG CSV (ciclo 16, t,X,Y,Z,setX,setY,setZ,sym) ==="
Wire.begin(PIN_SDA=4, PIN_SCL=5)                 // SDA, SCL  (¡en ese orden!)
accel.begin() ?
    sí  -> setRange(±2g) ; setDataRate(100 Hz) ; sensorOnline = true
           print "[ADXL345] OK (begin() correcto)"
    no  -> sensorOnline = false ; notDetectedReported = true
           print "[ADXL345] NO DETECTADO (revisar cableado VCC/GND/SDA/SCL)"
runChipDiagnostic()                             // SIEMPRE
print "Adquisiendo setpoint (20 lecturas en reposo)..."
```

`delay(300)` tras `Serial.begin()` da tiempo al USB-serial del ESP8266 a
enlazarse; sin él, las primeras líneas del banner se pierden en muchos sistemas.

### 4.8 `loop()` (líneas 461–495)

```
now = millis()

1) if (now - lastSampleMs >= 10)            -> lastSampleMs = now
       if (readAccel()) processSample()                       // 100 Hz

2) if (!sensorOnline && (now - lastRetryMs >= 1000))           // 1 Hz
       lastRetryMs = now
       accel.begin() ? (setRange, setDataRate, sensorOnline=true,
                        notDetectedReported=false, print "[ADXL345] OK")
                     : (!notDetectedReported -> print "NO DETECTADO" una sola vez)

3) if (setpointReady && (now - lastPrintMs >= 50))              // 20 Hz
       lastPrintMs = now ; printCharTick()
```

Los tres bloques son **independientes y no excluyentes**: en un mismo `loop()`
pueden dispararse los tres. Esto es intencionado: desacopla adquisición,
recuperación del sensor y salida. El único coste es que la alineación entre el
muestreo y la impresión deriva ≤ 10 ms, irrelevante frente a los 50 ms del tick.

> La rama de reintento (2) es un **watchdog de hardware**: si el GY-291 se
> desconecta en marcha, el firmware no se cuelga ni silencia, sino que lo
> reconfigura y vuelve a detectarlo solo.

---

## 5. Diagramas de flujo

### 5.1 `setup()`

```
        ┌─────────┐
        │  start  │
        └────┬────┘
             v
   Serial.begin(115200); delay(300)
             │
   ┌─────────┴──────────┐
   │ print banner ciclo 16│
   └─────────┬──────────┘
             v
      Wire.begin(4, 5)
             │
        accel.begin()?
        ┌────┴────┐
       SÍ        NO
        │         │
 setRange ±2g     sensorOnline=false
 setDataRate100Hz notDetectedReported=true
        │         │
 print "[ADXL345] OK"  print "[ADXL345] NO DETECTADO"
        └────┬────┘
             v
    runChipDiagnostic()      <-- SIEMPRE (lee registros por Wire)
      · DEVID / POWER_CTL / DATA_FORMAT / BW_RATE / OFS
      · fuerza POWER_CTL=0x08 y verifica
      · primera muestra cruda
             v
 print "Adquisiendo setpoint (20 lecturas en reposo)..."
             │
             v
        [ -> loop() ]
```

### 5.2 `loop()` y `processSample()`

```
        ┌────────────┐
        │  millis()  │◄─────────────────────────────┐
        └─────┬──────┘                              │
              v                                     │
   ┌──────────────────────────┐   no                │
   │ now-lastSampleMs >= 10 ? │────┐                 │
   └────────┬─────────────────┘    │                 │
            SÍ                     │      ┌──────────┴───────────┐
             │                     │      │ !sensorOnline &&     │
    readAccel()                    │      │  now-lastRetry>=1000?│
      ┌──┴──┐                      │      └────┬──────────┬──────┘
     true  false                    │         SÍ         NO
      │      │                      │          │          │
 processSample()  (nada)            │   accel.begin()     │
      │                             │   OK -> imprime OK  │
      v                             │   KO -> imprime      │
 ┌──────────────────────┐           │   "NO DETECTADO"    │
 │ setpointReady?       │           │   (solo 1 vez)      │
 └───┬──────────────┬───┘           │                     │
    NO             SÍ                │                     │
     │               │               │                     │
 acumular        devX/Y/Z           │                     │
 +'.'            eje ganador        │                     │
 accCount        eventActive        │                     │
     │           allResting         │                     │
  >=20?           switch(detectState)│                     │
     │            ┌──┴─────────┐     │                     │
    SÍ           IDLE        LOCKED  │                     │
     │            │             │     │                     │
 setpoint=      evento?      allRest? │                     │
 media          símbolo      │  SÍ   │                     │
 setpointReady  LOCKED     restSince │                     │
 =true          lockUntil  =now     │                     │
 printCount=0   restSince=0   │     │                     │
 'setpoint      │             │  NO  │                     │
 establecido'   recalcular    restSince│                    │
                setpoint ×20  =0      │                    │
                (silencioso)          │                     │
                     │          │     │                     │
                     │     lockout+rest │                     │
                     │     -> IDLE      │                     │
                     └────────┴────┘    │                     │
                                        │                     │
   setpointReady && now-lastPrint >= 50 ?                      │
              ┌─────────┴─────────┐                            │
             SÍ                  NO                            │
              │                  └────────────────────────────┘
        printCharTick()  (1 carácter; salto de línea cada 50)
              │
              └────────────────── (vuelve a millis())
```

---

## 6. Línea temporal de un evento (ejemplo real)

`t = 0 ms` — el bus pasa por un lomo de toro a gran velocidad.

| t (ms) | Muestreo (100 Hz) | Estado | Efecto en la salida (20 Hz) |
|---|---|---|---|
| 0 | 1ª muestra con `devZ = +0.31 g` | `ST_IDLE` | `maxDev = +0.31`, positivo y ≥ 0.24 ⇒ `eventChar = '*'`, `symbolPending = true`, → `ST_LOCKED`, `lockUntil = 250` |
| 10 | rebote: `devZ = −0.26 g` | `ST_LOCKED` | **ignorado** (no se emite `!`) |
| 50 | — | `ST_LOCKED` | tick de impresión: emite `'*'` (una sola vez) y consume `symbolPending` |
| 100 | `devZ = +0.05 g` | `ST_LOCKED` | `.` |
| 150 | reposo | `ST_LOCKED`, `restSince = t≈130` | `.` |
| 180 | `devZ = −0.18 g` (rebote tardío) | `ST_LOCKED` | `allResting = false` ⇒ **`restSince = 0`**; `lockUntil` aún no cumplido ⇒ sigue bloqueado |
| 200 | reposo | `ST_LOCKED`, `restSince = 185` | `.` |
| 235 | reposo continuo desde 185 | `ST_LOCKED` | se cumplen (a) `t ≥ 250`… aún no |
| 250 | reposo continuo desde 185 | — | `t ≥ 250` ✔ y `250−185 = 65 ≥ 50` ✔ ⇒ **vuelve a `ST_IDLE`** |
| 300 | — | `ST_IDLE` | tick: `.` |

Resultado en la consola: `....*......` — **un solo símbolo** por evento, sin
par de `*`/`!` espurios. Este es exactamente el fallo que los ciclos 11–15
corrigieron.

---

## 7. Guía de modificación y extensión

### 7.1 Ajustar umbrales (lo más frecuente)

Todo se hace en el bloque de constantes (líneas 67–119). Ejemplos:

```cpp
#define ARROW_THRESHOLD_G    0.30f   // menos sensible: ignora baches pequeños
#define SEVERE_FACTOR        1.5f    // sólo marca como severo lo ≥ 0.45 g
#define RETURN_G             0.10f   //histéresis más floja (debe seguir < umbral)
#define EVENT_LOCKOUT_MS     400     // eventos más espaciados (rebote más largo)
#define REST_MS              100     // exige más reposo antes de rearmar
#define SETPOINT_N           40      // setpoint más estable, arranque más lento
#define PRINT_INTERVAL_MS    100     // salida a 10 Hz, 1 carácter cada 100 ms
```

> Si cambias `PRINT_INTERVAL_MS`, `LINE_CHARS` deja de durar 2,5 s por línea: a
> 10 Hz serían 5 s. Recalcula la conversión temporal de los ejemplos del README.

### 7.2 Añadir un nuevo símbolo (p. ej. curvatura lateral)

1. Añade el umbral: `#define TURN_THRESHOLD_G 0.15f`.
2. En `processSample()`, Fase B, calcula `maxLat = fabsf(devX) - fabsf(devZ)` (o
   usa la componente lateral si cambias el montaje).
3. Antes del `switch (detectState)`, si se cumple el criterio lateral, fija
   `eventChar = '/'`, `symbolPending = true` y **reutiliza la misma transición**
   `detectState = ST_LOCKED; lockUntilMs = millis() + EVENT_LOCKOUT_MS; restSinceMs = 0;`
   (extráela a un `void latchEvent(char c)` para no duplicar código).
4. Actualiza la tabla de símbolos del README y de `docs/datasheet.md`.

> **El carácter debe ser ASCII imprimible (0x20–0x7E).** Nada de UTF-8.

### 7.3 Cambiar el rango del sensor

Si pasas a ±4 g o ±8 g **debes** cambiar el divisor de `readAccel()`:

```cpp
gx = rawX / 128.0f;   // ±4 g  (DATA_FORMAT bits 0-1 = 0x01)
gx = rawX /  64.0f;   // ±8 g  (DATA_FORMAT bits 0-1 = 0x02)
```

Y reajustar `ARROW_THRESHOLD_G` / `SEVERE_FACTOR` a la nueva escala.

### 7.4 Añadir un segundo sensor (p. ej. giróscopo MPU-6050)

1. Añade la dirección y el `Wire.beginTransmission` en una función propia
   (`readMpu6050()`), sin tocar `readRawAxes()`.
2. Añade un acumulador de setpoint paralelo (`mpuX0/Y0/Z0` + contadores) o
   reutiliza `accSumX/Y/Z` si el promedio se hace conjunto.
3. En `processSample()` añade el bloque de desviación de su eje; decide el «eje
   ganador» entre los 6 valores en un bucle o con comparaciones encadenadas.
4. Si cabe en la memoria, reutiliza `eventChar`/`symbolPending` (no hace falta
   una segunda máquina de estados salvo que quieras eventos simultáneos).

### 7.5 Añadir una salida física nueva (LED, display, etc.)

El LED del NodeMCU es `D4`/GPIO2 (activo a bajo):

```cpp
#define PIN_LED 2   // D4
pinMode(PIN_LED, OUTPUT);
digitalWrite(PIN_LED, HIGH);   // apagado (el NodeMCU es active-LOW)
```

Un patrón útil: parpadeo rápido durante `ST_LOCKED` y estado fijo en `ST_IDLE`.
No hay conflicto con I²C (GPIO2 es libre).

### 7.6 Evolucionar el formato CSV (o volver atrás)

**Ya está hecho en el ciclo 16:** la salida numérica que este apartado
proponía es la que existe hoy, con una diferencia — la línea lleva
`millis()` + los tres ejes + el setpoint + el símbolo, en lugar de sólo las
desviaciones. Si añades un campo nuevo (p. ej. `devZ`), hazlo **al final, antes
del `\n`**, o preferiblemente **después de `sym`**, y actualiza `parse_line()`
y `Point.to_dict()` en `app/serial_reader.py` (§11.2).

Consideraciones de ritmo:

| Cambio | Coste |
|---|---|
| 20 Hz × 8 campos (actual) | ~700 B/s — 6 % del ancho de banda de 115200 baud |
| Subir a 50 Hz (`PRINT_INTERVAL_MS = 20`) | ~1,8 kB/s — sostenible; la app tolera más puntos por segundo (el buffer y el `poll()` no cambian) |
| Bajar a 10 Hz | ~350 B/s — se pierden eventos cortos entre muestras |
| Imprimir la salida cruda a 100 Hz | ~3,5 kB/s — viable, pero el `printf` de 8 floats a 100 Hz añade latencia a `loop()`; mejor acumular en un buffer y volcar por bloques |

Recomendación: si necesitas más resolución temporal **para análisis**, no
subas la impresión: usa el muestreo interno a 100 Hz, guarda en un buffer
circular y vuelca por bloques, dejando la salida a 20 Hz para la visualización.

### 7.7 Buenas prácticas del proyecto


* **No introduzcas `delay()` en `loop()`**: rompe la temporización de las tres
  tareas y deforma la cadencia de 20 Hz.
* **No introduzcas `Serial.println()` dentro de `processSample()`** salvo en la
  adquisición inicial: a 100 Hz, un mensaje por evento destrozaría el flujo y
  añadiría latencia.
* Mantén `RETURN_G < ARROW_THRESHOLD_G`: si no, la máquina puede quedar
  bloqueada en `ST_IDLE` sin poder disparar (o con oscilación en el borde).
* Mantén `SEVERE_THRESHOLD_G > ARROW_THRESHOLD_G` (es decir `SEVERE_FACTOR > 1`);
  si no, la categoría leve desaparecería.
* Comprueba la condición de longitud en toda lectura I²C
  (`requestFrom(...) == n`), como hace `readRawAxes()`.
* Usa `unsigned long` para `millis()`; nunca compares con `!=` cuando
  provenga de una resta (`if (now - lastX >= T)`).

---

## 8. Memoria y compilación

* El sketch usa **un solo objeto** de librería (`accel`) y **ningún buffer de
  datos**: el flujo es completamente *streaming*. El pico de RAM en el bucle está
  en las variables locales de `runChipDiagnostic()` (≈ 20 bytes). El `printf` de
  8 campos no usa buffer estático propio del sketch.
* Dependencias externas: **`Adafruit_ADXL345_U`** y **`Adafruit_Sensor`**
  (más `Wire`, incluida en el core).
* Compilación: `arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 .`
  (ver README §9 para el flujo completo).
* `secrets.h` **no** debe incluirse: si aparece un error
  `#include "secrets.h"`, estás compilando un sketch de otra fase.
* El **coste de RAM/flash de la visualización es cero** en el ESP8266 desde el
  ciclo 16: todo el coste de la gráfica (buffer, canvas, UI) está en el PC
  (§11). Los ~700 B/s de CSV ocupan el 6 % del ancho de banda del puerto.


---

## 9. Fases futuras ya presupuestadas (NO implementadas)

Estas tres adiciones están **planificadas** y son la razón de la separación
detección/reporte del código actual. **Nada de esto está en el sketch.**

### 9.1 Conteo de incidencias

Añadir contadores `uint32_t` incrementados en la transición
`ST_IDLE → ST_LOCKED` (no en `printCharTick()`, para que el conteo no dependa de
la cadencia de impresión):

```cpp
// Puntos de extensión: justo después de decidir eventChar (línea ~361)
switch (eventChar) {
  case '*': countSevereUp++;   break;
  case '!': countSevereDown++; break;
  case '+': countMildUp++;     break;
  case '-': countMildDown++;   break;
}
```

### 9.2 Ciclo de publicación MQTT de 1 minuto

1. Añadir `#include <ESP8266WiFi.h>` y `#include <PubSubClient.h>`.
2. **Credenciales en `secrets.h`** (no versionado; existe `secrets.example.h`
   como plantilla histórica) con `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_SERVER`,
   `MQTT_PORT`, `MQTT_USER`, `MQTT_PASS`.
3. Cliente MQTT con cliente ID único (p. ej. `macAddress`), LWT y QoS 1.
4. Temporizador de 60 s con `millis()` (nunca `delay`) que publique los
   contadores como JSON, reutilizando el mismo estilo de `Serial.printf` que ya
   usa el sketch.
5. **Reintento en background** con el mismo patrón que ya existe para el sensor
   (`SENSOR_RETRY_MS`): si el broker no responde, se reintenta sin bloquear la
   detección.
6. **Recomendación fuerte:** ejecutar la publicación MQTT en un *task* de
   FreeRTOS (`xTaskCreate`) o, al menos, agrupar la transmisión, para no alterar
   la latencia de `readRawAxes()`. El diagnóstico serial se hizo precisamente
   para aislar la medición del stack de red.

### 9.3 Persistencia en EEPROM

1. `#include <EEPROM.h>` (EEPROM emulado en el flash del ESP8266).
2. Estructura con `magic` + versión + los contadores; `EEPROM.put()` +
   `EEPROM.commit()` con **cadencia baja** (el flash del ESP8266 tiene ~10 000
   ciclos de escritura por sector).
3. Restaurar en `setup()` antes de nada; **poner a cero los contadores sólo tras
   confirmación de entrega** (ACK/PUBACK), para no perder datos si hay un corte.

---

## 10. Referencia rápida

| Quiero… | Dónde |
|---|---|
| Cambiar sensibilidad | `ARROW_THRESHOLD_G` (línea 93) |
| Cambiar cuándo es «severo» | `SEVERE_FACTOR` (línea 105) |
| Cambiar histéresis de retorno | `RETURN_G` (línea 107) |
| Cambiar cadencia de salida | `PRINT_INTERVAL_MS` (línea 82) |
| Cambiar la supresión de rebote | `EVENT_LOCKOUT_MS` (125) / `REST_MS` (131) |
| Cambiar número de muestras del setpoint | `SETPOINT_N` (línea 92) |
| Cambiar pines I²C | `PIN_SDA` / `PIN_SCL` (líneas 134–135) |
| Ver el estado interno | variables de §3 |
| Entender la decisión del símbolo | §4.5 (tabla signo × magnitud) |
| Cambiar el formato de la línea CSV | `printCharTick()` §4.6 + `parse_line()` §11.2 |
| Entender la arquitectura de la app | §11 |
| Ajustar las zonas de color del gráfico | `LEVE_THRESHOLD_G` / `SEVERO_THRESHOLD_G` en `app/serial_reader.py` |
| Añadir un símbolo nuevo | §7.2 (y `VALID_SYMBOLS` en la app, §11.2) |
| Añadir un sensor | §7.4 |
| Añadir salida física | §7.5 |
| Evolver el formato de salida | §7.6 |
| Planificar la conectividad | §9 |

---

## 11. Arquitectura de la aplicación de escritorio (`app/`)

> **NUEVO en el ciclo 16.** Documentación de usuario en
> [`app/README.md`](../app/README.md); aquí se explica **cómo está construida**
> y dónde tocar para extenderla.
>
> **Principio rector:** la app **no toca el firmware**. Consume el CSV como un
> contrato de datos. Cualquier mejora de la interfaz se hace en el PC, sin
> recompilar ni recargar la placa.

### 11.1 Vista de conjunto

```
   /dev/ttyUSB0 @ 115200
          │  bytes
          v
┌──────────────────────┐   lee líneas,   ┌──────────────────────────┐
│  serial_reader.py    │   parsea CSV    │  app.py                  │
│  ────────────────────│───────────────▶│  ────────────────────────│
│  SerialReader        │  buffer + estado│  ThreadingHTTPServer    │
│   · _serial_loop()   │                 │   127.0.0.1:<puerto>     │
│   · _simulator_loop()│                 │   + /api/state, /api/…   │
│   · parse_line()     │                 │  webview (Qt) o navegador│
│   · deque(maxlen=6000)                └────────────┬─────────────┘
└──────────────────────┘                              │ HTTP/JSON (poll)
                                                      v
                                          ┌──────────────────────┐
                                          │  app.js + index.html  │
                                          │  canvas 2D, ~25 fps   │
                                          │  + style.css (neón)   │
                                          └──────────────────────┘
```

Ficheros y su responsabilidad:

| Fichero | Responsabilidad | Tamaño aprox. |
|---|---|---|
| `app/app.py` | Servidor HTTP local + API JSON + ventana (`pywebview`) o navegador | 202 líneas |
| `app/serial_reader.py` | Hilo de lectura del puerto, parser CSV, buffer circular, simulador | 375 líneas |
| `app/index.html` | Estructura de la interfaz (cabecera, tarjeta de gráfico, barra lateral, pie) | 171 líneas |
| `app/style.css` | Tema oscuro con neón/glow; variables `--cyan`, `--amber`, `--red`, `--green`, `--blue` | 14 KB |
| `app/app.js` | Parseo del estado, buffer del cliente, dibujo en canvas, eventos de UI | 591 líneas |
| `app/requirements.txt` | `pyserial`, `pywebview`, `qtpy`, `PyQt5`, `PyQtWebEngine` | 5 líneas |

### 11.2 `serial_reader.py` — adquisición y parseo

| Elemento | Detalle |
|---|---|
| `parse_line(line) -> Point \| None` | **El parser.** Divide por `,`; exige **8 campos**; `t` entero, seis floats, `sym` ∈ `VALID_SYMBOLS` (`.` `+` `-` `*` `!`). Cualquier otra cosa (banner, diagnóstico, puntos `.`) devuelve `None` y se descarta |
| `Point` | `__slots__` con `t,x,y,z,set_x,set_y,set_z,sym`; `to_dict()` serializa a claves cortas (`t,x,y,z,sx,sy,sz,c`) para reducir el JSON |
| `SerialReader` | Estado de conexión (`idle`/`connecting`/`connected`/`error`/`simulator`), buffer y contadores |
| `SerialReader._serial_loop()` | Hilo daemon: `serial.Serial(port, baudrate, timeout=0.2)`, `readline()` en bucle, decodifica UTF-8 con `errors="replace"`, `reset_input_buffer()` al abrir. Marca `error` con el texto de la excepción si el puerto no existe / está ocupado / hay permisos |
| `SerialReader._simulator_loop()` | Genera líneas CSV sintéticas a 20 Hz con eventos aleatorios cada 3–9 s; setpoint fijo `1.00` en los tres ejes. Permite probar la UI sin placa |
| `SerialReader._handle_line()` | Guarda `last_raw` (para el pie de la UI), parsea, y **corrige el wrap de `millis()`** (~49,7 días) acumulando `_wrap_offset += 1<<32` cuando el tiempo salta hacia atrás |
| `SerialReader.get_points_since(i)` | Devuelve los puntos nuevos desde el índice `i` y el nuevo índice absoluto → polling incremental, sin reenviar todo el buffer |
| `SerialReader.get_state()` | Un snapshot: `status`, `port`, `baud`, `error`, `sim`, `raw`, `last`, `counts` (por símbolo), `total`, `t0`, `wrap`, `last_t`, `bytes`, `uptime`, `ports`, `thresholds` |
| `SerialReader.clear()` | Vacía buffer, contadores y referencias temporales |
| Constantes | `MAX_POINTS = 6000` (~5 min a 20 Hz) — `deque(maxlen=…)` descarta lo más viejo; `LEVE_THRESHOLD_G = 0.20`, `SEVERO_THRESHOLD_G = 0.24` — **umbrales visuales**, deben mantenerse sincronizados con el firmware |

**Extender el parser** (p. ej. si el firmware añade un 9.º campo): edita
`parse_line()` para aceptar 9 campos, añade el atributo a `Point.__slots__` y a
`to_dict()`, y en `app.js` añádelo al `mergePoints()`.

### 11.3 `app.py` — servidor y ventana

| Elemento | Detalle |
|---|---|
| `ApiHandler` | `BaseHTTPRequestHandler` que sirve el frontend y la API. `reader` se inyecta como atributo de clase desde `start_server()` |
| Rutas `GET` | `/` y `/index.html`, `/style.css`, `/app.js`; `/api/state?since=N`; `/api/ports`; `/api/connect?port=&baud=`; `/api/disconnect`; `/api/clear` |
| `do_POST` | Delega en `do_GET()` (las acciones son idempotentes y sin cuerpo) |
| Cabeceras | `Cache-Control: no-store` en recursos y API: evita que el navegador sirva un `app.js` viejo tras editarlo |
| `log_message()` | Sobrescrito y vacío: no se escribe un log de acceso por cada `poll` de 40 ms |
| `_free_port(preferred)` | Si 8765 está ocupado, pide uno efímero (`bind(("127.0.0.1", 0))`); si falla, puerto aleatorio |
| `start_server()` | `ThreadingHTTPServer` en hilo daemon; como es *threading*, un `poll` lento no bloquea al resto |
| `launch_gui(url)` | `webview.create_window(...)` (1320×860, min 1024×660, fondo `#0b0e14`) + `webview.start()`. Devuelve `False` si `import webview` falla o si la ventana revienta |
| `main()` | Parsea CLI → crea `SerialReader` → arranca servidor → conecta (`SIM` si `--sim` o `--port SIM`) → intenta pywebview → **si falla, `webbrowser.open(url)`** y espera en bucle con `threading.Event().wait(3600)` |

**Opciones de línea de comandos:** `--port` (default `/dev/ttyUSB0`),
`--baud` (115200), `--sim`, `--browser`, `--http-port` (8765), `--debug`.

### 11.4 `app.js` — renderizado en canvas

| Elemento | Detalle |
|---|---|
| `api(path)` | `fetch` con actualización de `state.since`; encapsula errores |
| `poll()` + `setInterval(poll, 40)` | **≈ 25 Hz** de sondeo de la API; cada respuesta trae sólo los puntos nuevos. A 20 Hz de datos, se hace una consulta de más por muestra |
| `mergePoints(pts, t0, wrap)` | Añade al buffer del cliente aplicando el offset de wrap para tener `t` absoluto; `state.t0` fija el origen temporal |
| `render()` + `requestAnimationFrame(render)` | Dibujo **independiente del polling**: la animación (pulsos de los marcadores) continúa a la frecuencia de refresco de pantalla, aun sin datos nuevos |
| `resizeCanvas()` | Ajusta el buffer del canvas al `devicePixelRatio` para que no se vea borroso |
| Escalas `X(sec)` / `Y(g)` | Cierre de la ventana temporal (10–120 s, por defecto 60) y del rango vertical alrededor de `setZ` |
| Zonas | Relleno `zoneFill()` entre `setZ ± leve` y `setZ ± severo`, con bordes punteados: ámbar arriba (leve), rojo arriba (severo), ámbar abajo, rojo abajo. Los colores vienen de `COLORS` |
| Línea de setpoint | `setLineDash([7,5])` en `COLORS.setline` + etiqueta `SETPOINT 1.00 g`; el valor se toma de `sz` de los puntos, **no de una constante** |
| Curvas | `drawCurve(pts, accessor, color, width, glow, X, Y)`: Z en cian con `shadowBlur`, X en ámbar y Y en violeta opcionales |
| Marcadores | Por cada evento, un halo radial + punto con glow + el símbolo dibujado encima; el alfa decae en 4 s (`fade = 1 - age/4`) |
| `pushEventItems(pts, t0)` | Alimenta la lista «Eventos recientes» y los contadores con los `sym` distintos de `.` |
| `SYM_META` | Etiquetas y subtítulos del badge (`ELEVACIÓN SEVERA`, etc.) |
| Pausa / limpiar | `chkPause` fija `freezeAnchor`; `clearBtn` llama a `/api/clear` y reinicia el buffer del cliente |

### 11.5 Modificar o extender la app

| Quiero… | Dónde |
|---|---|
| Cambiar el puerto por defecto | `--port` en `app.py` (o el desplegable de la cabecera) |
| Añadir un campo al gráfico | `drawCurve(...)` en `render()` + toggle en `index.html` |
| Cambiar colores del tema | variables `--*` de `style.css` y `COLORS` de `app.js` (deben coincidir) |
| Cambiar los umbrales de las zonas | `LEVE_THRESHOLD_G` / `SEVERO_THRESHOLD_G` en `serial_reader.py` (y sincronizar con el firmware) |
| Ampliar el histórico en memoria | `MAX_POINTS` en `serial_reader.py` |
| Cambiar la frecuencia de sondeo | `setInterval(poll, 40)` en `app.js` |
| Añadir un eje o una serie | `Point`, `parse_line`, `mergePoints`, `render` |
| Simular otra señal | `_simulator_loop()` en `serial_reader.py` |
| Exponer el estado a otro programa | las rutas `/api/*` ya son JSON; se pueden consultar con `curl` |

> 🔗 **Regla de oro:** el sketch no sabe que la app existe. Si cambias el
> `printf` de `printCharTick()`, actualiza `parse_line()` en el mismo commit, o
> la gráfica se quedará plana sin ningún error visible.

---

*Documento alineado con `BusAccel-8266.ino` — **ciclo 16** (salida CSV) — y con
`app/` (Python + pywebview + canvas). Si el sketch cambia (umbrales, símbolos,
cadencias) o la app cambia su contrato de datos, actualiza en el mismo commit
las tablas de este documento, del `README.md`, de `docs/datasheet.md` y de
`app/README.md`.*
