# BusAccel-8266-v3 — Pico de aceleración por evento (eje Z) sobre MQTT

> **Firmware v3 — PUBLICACIÓN MQTT DEL PICO DEL EVENTO (eje Z).**
>
> Acelerómetro GY-291 (ADXL345) en un bus urbano. Cuando la aceleración
> vertical (**eje Z**) se sale del rango normal —en cualquiera de los dos
> sentidos: subida `+`/`*`, caída `-`/`!`— el equipo mide el **PICO** de la
> ventana (máxima `|devZ|` de 200 muestras a 100 Hz: 1 s antes + 1 s después)
> y publica **un único mensaje compacto** por evento. Sin evento no hay
> tráfico de serie ni de red.
>
> **Cambio clave v3:** ya NO se transmite la ventana cruda. Desaparecen los
> 5 mensajes de chunk (`accel/event/chunk`) y las 200 líneas `EV,` por serie.
> Por cada evento se publica/imprime **SOLO el PICO**. La ventana se sigue
> capturando y procesando internamente, pero su contenido crudo no sale de la
> placa. Objetivo del cliente: saber **cuánto** golpeó el bus, no la forma de
> onda completa.

[![Versión](https://img.shields.io/badge/firmware-v3-ff69b4)](#)
[![Protocolo](https://img.shields.io/badge/transporte-MQTT-00e5ff)](#7-contrato-mqtt)
[![Sampling](https://img.shields.io/badge/muestreo-100%20Hz-4c9aff)](#)
[![Sensor](https://img.shields.io/badge/sensor-ADXL345%20(GY--291)-4c9aff)](https://learn.adafruit.com/adafruit-adxl345-3-axis-accelerometer)
[![Placa](https://img.shields.io/badge/placa-NodeMCU%20ESP8266%20ESP--12E-007acc)](https://docs.espressif.com/projects/esp8266/en/latest/hw/nodemcu/nodemcu.html)
[![Serial](https://img.shields.io/badge/consola-115200%20baud-3fb950)](https://www.arduino.cc)

---

## 0. Qué cambió respecto de v2 (migración rápida)

| Aspecto | v2 | **v3** |
|---|---|---|
| Qué se transmite por evento | 1 resumen + **5 chunks** (200 muestras) | **1 mensaje: el PICO** |
| Topic de evento | `accel/event/summary` + `accel/event/chunk` | **`accel/event/peak`** (único) |
| Líneas serie por evento | 200 líneas `EV,…` | **1 línea `[PICO] …`** |
| `eventBuf[200]` (1600 B) | sí | **eliminado** (pico en línea) |
| Buffer MQTT | 1024 B | **384 B** (payload pico ~150 B) |
| RAM del sketch | ~34.3 KB (42 %) | **~30–32 KB (37–40 %)** |
| Contador RTC sin validar (bug v2) | podía arrancar con basura | **validado: magic + rango + ≠0** |
| `"z":[]` vacío en chunks (bug v2) | presente | **imposible: no hay chunks** |
| Detección, símbolos, umbrales, máquina de estados | — | **idénticos a v2** |
| `MQTT_DRY_RUN` por defecto | 1 | **1 (ensayo en seco)** |

### Por qué el cambio

La ventana cruda saturaba el puerto serie (~0.9 s por evento a 115200 baud)
y el broker con 5 mensajes por evento. Para el objetivo del cliente —saber
**cuánto** golpeó el bus— el pico es dato suficiente. La forma de onda completa
sigue disponible compilando con `MODE_SERIAL_CSV 1` (CSV 20 Hz, formato v1).

---

## 1. ¿Qué es este proyecto?

**BusAccel-8266** es un registrador de aceleraciones para un **bus urbano**
equipado con un acelerómetro ADXL345 (módulo GY-291) conectado por I²C a un
NodeMCU ESP8266. El objetivo es **diagnosticar los malos hábitos de
conducción**: baches, bordillos, hoyos, cambios de carril bruscos y, sobre
todo, **saltos peligrosos por encima de un lomo de toro a gran velocidad**.

### Estado actual: pico por evento sobre el eje Z (v3)

| Característica | Estado |
|---|---|
| **Un mensaje MQTT con el PICO por evento** (`accel/event/peak`) | 🟡 **En ensayo en seco** (`MQTT_DRY_RUN 1`: se imprime por serie, no sale de la placa) |
| **Ventana interna de 200 muestras (1 s + 1 s @ 100 Hz) para medir el pico** | ✅ **Activo** (no se almacena completa) |
| **Detección en los dos sentidos: subida `+`/`*` y caída `-`/`!`** | ✅ **Activo** |
| Contador de evento persistente (`id`) en RTC user memory **validado** | ✅ **Activo** |
| Comando `RECAL` por MQTT para recalibrar el setpoint en marcha | ✅ **Activo** |
| Watchdog de hardware 8 s + watchdog de "sin publicar" 15 min | ✅ **Activo** |
| Sonda ICMP a 8.8.8.8 cada 30 s (5 intentos) | 🟡 **Compilado, inactivo en ensayo seco** |
| Reinicio con MAC aleatoria si el broker no responde | 🟡 **Compilado, inactivo en ensayo seco** |
| Setpoint automático (20 lecturas), recalculable por MQTT | ✅ **Activo** |
| Diagnóstico profundo de registros ADXL345 al arranque | ✅ **Activo** |
| Modo serie puro (`MODE_SERIAL_CSV 1`) para la app de escritorio | ✅ **Activo** |
| Formato de línea CSV `t,X,Y,Z,setX,setY,setZ,sym` | ✅ **Igual que v1/v2** |
| Volcado de la ventana cruda por evento | ❌ **Eliminado en v3** (era la causa de la saturación) |

---

## 2. Hardware y especificaciones

### Placa objetivo

| Parámetro | Valor |
|---|---|
| **Placa** | NodeMCU ESP8266 (módulo ESP-12E) |
| **Microcontrolador** | ESP8266EX (Tensilica L106, 32-bit) @ 80/160 MHz |
| **Flash** | 4 MB |
| **SRAM usable** | 80192 B (~80 KB) |
| **Uso de RAM de este sketch** | **~30–32 KB (37–40 %)** |
| **IRAM** | 65536 B; baseline del SDK ~60931 B (**~92 %**, no es margen útil del proyecto) |
| **Uso de Flash** | **~24–25 %** (~246 KB de 1048576 B) |
| **Alimentación** | 5 V por USB (pines `VIN`/`5V`) o 3.3 V por el pin `3V3` |
| **Interfaz de usuario** | exclusivamente la consola serie a 115200 baud |
| **FQBN (arduino-cli)** | `esp8266:esp8266:nodemcuv2` |
| **Core Arduino** | `esp8266:esp8266@3.1.2` |
| **Toolchain** | `arduino-cli 1.5.2-rc.1` |

### Sensor

| Parámetro | Valor |
|---|---|
| **Módulo** | GY-291 (breakout del ADXL345) |
| **Chip** | Analog Devices ADXL345, acelerómetro MEMS 3 ejes |
| **Interfaz** | I²C, dirección **0x53** (selector SDO a GND) |
| **Rango configurado** | **±2 g** (`ADXL345_RANGE_2_G`) |
| **Escala de conversión** | 256 LSB/g → `g = raw / 256.0f` |
| **Frecuencia de datos** | **100 Hz** (`ADXL345_DATARATE_100_HZ`), code `0x0A` |
| **Resolución real** | 10 bits en ±2 g → ~3.9 mg/LSB |
| **Lectura** | *burst read* directo de 6 bytes (`DATAX0..DATAZ1`), little-endian int16 |
| **Identificador** | `WHO_AM_I (0x00) = 0xE5` |

> **Nota sobre la librería Adafruit:** el sketch instancia
> `Adafruit_ADXL345_Unified(12345)` y llama a `begin()`, `setRange()` y
> `setDataRate()` para verificación de presencia y configuración del chip. Las
> lecturas que alimentan la detección **no** pasan por `accel.getEvent()`, sino
> por acceso directo a registros con `Wire` (`readRawAxes()`). El `12345` es
> el *sensor ID* interno de la librería, **no** una dirección I²C.

---

## 3. Principio de funcionamiento

### 3.1 El setpoint: la línea base de la marcha

El bus no está perfecto: tiene pendiente, la suspensión está en equilibrio,
el vehículo se inclina en curvas. El firmware no mide aceleración absoluta,
sino la **desviación respecto a una referencia**: el **setpoint**, obtenido
promediando **20 lecturas** (`SETPOINT_N`) con el bus detenido y en reposo:

```
spX = (gx0 + … + gx19) / 20
spY = (gy0 + … + gy19) / 20
spZ = (gz0 + … + gz19) / 20
```

Con el módulo plano lo normal es `Z ≈ +1.00` (gravedad), `X ≈ 0`, `Y ≈ 0`.
Esos valores **no son un error**: son la gravedad + inclinación de referencia,
y quedan restados automáticamente. El setpoint se **recalcula en silencio**
cada 20 muestras de reposo (solo en `ST_IDLE`), para seguir lentamente la
postura real sin que un evento la contamine.

### 3.2 La desviación: lo que realmente importa

En cada muestra a 100 Hz: `devZ = gz − spZ`. La gravedad desaparece del
cálculo porque está contenida en el setpoint. Solo queda la **aceleración
dinámica**, que es lo que delata una conducción brusca.

> **Solo el eje Z decide.** `devX` y `devY` se siguen calculando (y viajan en
> el CSV), pero **no disparan eventos**: son vibración del chasis, del motor y
> del suelo, y en v1 producían falsos positivos.

### 3.3 El símbolo: sentido y severidad sobre el eje Z

| `devZ` | Símbolo | Lectura |
|---|---|---|
| `≥ +0.24` | `*` | **Elevación severa** — lomo de toro a alta velocidad, salto peligroso |
| `> +0.20` | `+` | Elevación leve — bordillo, catenaria |
| `≤ −0.24` | `!` | **Caída severa** — hoyo profundo / bache de magnitud relevante |
| `< −0.20` | `-` | Caída leve — transición de rasante |
| `\|devZ\| ≤ 0.20` | `.` | Reposo |

Umbrales: `ARROW_THRESHOLD_G = 0.20`, `SEVERE_FACTOR = 1.2` →
`SEVERE_THRESHOLD_G = 0.24`, histéresis de retorno `RETURN_G = 0.15`.

### 3.4 Supresión del rebote inercial (máquina de estados)

Un bache produce una **segunda aceleración en sentido contrario** 100–200 ms
después (rebote de suspensión). Un detector ingenuo generaría dos eventos. La
máquina de estados de dos estados lo evita:

```
        ┌──────────────────────────────────────────────┐
        │                                              │
        ▼                                              │
    ┌────────┐   primer cruce (|devZ| > 0.20 g)        │  (a) millis() >= lockUntilMs
    │ST_IDLE │──────────────────────────────────────────┼─── (b) reposo sostenido
    └────────┘   abre la ventana y bloquea ─────────────┘    >= REST_MS (50 ms)
                 ┌──────────┐                                con |devZ| <= 0.15 g
                 ▼          │                             (c) LA VENTANA HA TERMINADO
             ┌──────────┐  │                                (!collecting)
             │ST_LOCKED │  │
             └──────────┘  │
```

* **`ST_IDLE`**: si no hay cruce, se acumula la muestra para el recálculo
  silencioso del setpoint. En el **primer** cruce se elige el símbolo, se pasa
  a `ST_LOCKED`, se arma el bloqueo de `EVENT_LOCKOUT_MS = 250 ms` y se arranca
  la ventana (`beginCapture()`).
* **`ST_LOCKED`**: se ignoran todos los cruces posteriores. El desbloqueo
  exige **las tres condiciones a la vez**: (a) lockout cumplido, (b) reposo
  sostenido `REST_MS`, (c) **la ventana del evento anterior ha terminado**.

> ⚠️ **La condición (c) es innegociable.** La ventana dura 2000 ms, mucho más
> que lockout + reposo (300 ms). Sin ella, un segundo evento a los 300 ms pisaría
> la ventana mientras la anterior se sigue llenando.

La histéresis (`RETURN_G 0.15 < ARROW_THRESHOLD_G 0.20`) hace el regreso a
reposo más exigente que el disparo: elimina el oscilamiento en el borde.

### 3.5 El pico: qué mide la ventana en v3

La ventana de 200 muestras (100 PRE + 100 POST) **no se almacena completa**.
El pico se calcula **en línea**:

1. **Parte previa (PRE_N = 100):** un anillo `preRing` siempre rodante guarda el
   pasado. Al disparar, `beginCapture()` **escanea el anillo** y se queda con la
   máxima `|devZ|` de lo ocurrido antes del cruce.
2. **Parte posterior (POST_N = 100):** `advanceCapture()` actualiza el pico con
   cada muestra POST (`eventPeakZ = max(eventPeakZ, |devZ|)`) y cuenta.

Resultado: `eventPeakZ` = **máxima `|devZ|` de toda la ventana (pre + post), en
g**. Es el dato que viaja en el campo `peak` del payload.

---

## 4. Salida serie

### 4.1 La línea `[PICO]` (modo normal, `MODE_SERIAL_CSV 0`)

Cada evento cierra su ventana con **UNA sola línea** (en v2 eran 200):

```
[PICO] id=12 sym=* peak=0.31g spz=0.93g dev=0.27g up=63959ms n=200 ms=2000
```

| Campo | Significado |
|---|---|
| `id` | contador de evento (RTC), sin huecos |
| `sym` | símbolo del evento (`. + * - !`) |
| `peak` | **máxima `|devZ|` de la ventana (g)** — el dato principal |
| `spz` | setpoint de Z en el instante del evento (g) |
| `dev` | `devZ` en el instante del cruce (g), con signo |
| `up` | `millis()` del equipo en el disparo (ms desde el arranque) |
| `n` | muestras de la ventana (siempre 200) |
| `ms` | duración nominal de la ventana (siempre 2000) |

> ⚠️ **`MODE_SERIAL_CSV 1` NO emite `[PICO]`.** En modo serie puro la salida es
> exclusivamente el CSV a 20 Hz (formato v1). La línea `[PICO]` está
> condicionada a `MODE_SERIAL_CSV == 0` (corrección I-004).

### 4.2 El ensayo en seco (`MQTT_DRY_RUN 1`, el default)

En este modo la placa **no abre WiFi ni MQTT**. Cada mensaje que se publicaría
se imprime por serie con su topic y su payload:

```
[MQTT-OUT] --> <client>/accel/event/peak  (150 B)
[MQTT-OUT] {"id":12,"sym":"*","spz":0.93,"dev":0.27,"peak":0.31,"n":200,"ms":2000,"up":63959}
[PICO] id=12 sym=* peak=0.31g spz=0.93g dev=0.27g up=63959ms n=200 ms=2000
```

Es el modo para validar formato y detección **sin suscriptor en el broker**.
Para MQTT real: `MQTT_DRY_RUN 0`, recompilar, subir.

### 4.3 La línea CSV continua (solo `MODE_SERIAL_CSV 1`)

```
t,X,Y,Z,setX,setY,setZ,sym
```

| Campo | Tipo | Significado |
|---|---|---|
| `t` | entero | `millis()` del ESP8266 |
| `X`,`Y`,`Z` | float, 2 dec. | aceleración actual en g (`raw/256.0f`) |
| `setX`,`setY`,`setZ` | float, 2 dec. | setpoint de cada eje en g |
| `sym` | 1 carácter | `. + * - !` |

**Línea real en reposo (módulo plano, Z hacia arriba):**

```
248150,0.14,0.17,0.91,0.14,0.16,0.92,.
248200,0.14,0.17,0.91,0.14,0.16,0.92,.
248250,0.15,0.16,0.92,0.14,0.16,0.92,.
```

> `Z − setZ` es exactamente la desviación que la máquina de estados evaluó.
> Formato **idéntico a v1/v2**: la app de escritorio sigue parseándolo.

---

## 5. Cómo operar

### 5.1 Conexión

1. Conecta el **GY-291** al NodeMCU según el diagrama de §7.
2. Alimenta y conecta el **NodeMCU por USB** a tu ordenador.
3. Compila y carga el firmware (§9).

### 5.2 Consola serie

Abre el monitor a **115200 baud, 8N1, sin control de flujo**:

```bash
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

Verás el banner, el diagnóstico y los 20 puntos de adquisición. A partir de
ahí, en modo normal: **silencio en reposo, una línea `[PICO]` por evento**.
También puedes usar **cutecom**, **PuTTY** (`Serial` / `115200` / `8-N-1` /
`Flow control: None`), **Arduino IDE** → *Monitor Serie* o
`screen /dev/ttyUSB0 115200`.

> 💡 **No te quedes en el terminal:** para ver la aceleración dibujada en el
> tiempo, usa la **app de escritorio** de `../BusAccel-8266-v2/app/` con
> `MODE_SERIAL_CSV 1`. El terminal sirve para capturar logs.

> **Sobre el gestor de arranque:** al pulsar RST puede aparecer un mensaje del
> cargador ROM a 74880 baud antes del banner. Es normal.

### 5.3 Secuencia de arranque

1. **Banner** (según modo):
   ```
   === BusAccel-8266-v3 (pico por evento, eje Z, ENSAYO EN SECO) ===
   Sin WiFi y sin MQTT: 1 mensaje de PICO por evento por serie.
   [FW] version 3
   [FW] build 2026-10-08
   ```
2. **Contador RTC** (validado):
   ```
   [RTC] contador de eventos: 0
   ```
3. **Verificación del sensor**:
   ```
   [ADXL345] OK (begin() correcto)
   ```
4. **Diagnóstico profundo de registros** (§5.4).
5. **Adquisición del setpoint** — 20 puntos:
   ```
   Adquiriendo setpoint (20 lecturas en reposo)...
   ....................
   setpoint establecido: X=0.01 Y=-0.16 Z=1.00
   ```
6. **Umbrales y ventana**:
   ```
   Umbral Z: leve 0.20g | severo 0.24g | retorno 0.15g
   Ventana: 100 antes + 100 despues = 200 muestras ~2000ms | 1 mensaje/evento (pico)
   ```

> ⚠️ **Coloca el bus en reposo antes de los 3 primeros segundos.** El setpoint
> se fija sobre las 20 primeras lecturas; si el bus está en movimiento, la
> referencia quedará contaminada.

### 5.4 Diagnóstico de registros (se ejecuta siempre, al arrancar)

| Línea | Qué comprueba | Valor esperado |
|---|---|---|
| `WHO_AM_I = 0xE5` | Identidad del chip | `0xE5` |
| `POWER_CTL = 0x08` | Bit 3 = measurement mode | `bit3=1` |
| `DATA_FORMAT = 0x00` | Bits 0-1 = rango | `±2 g` |
| `BW_RATE = 0x0A` | Code 0x0A | `100 Hz` |
| `OFSX/OFSY/OFSZ` | Offsets de fábrica | `0/0/0` (habitual) |
| `[POWER_CTL] OK: 0x08 persiste` | Escritura I2C efectiva | `OK` |
| `PRIMERA MUESTRA rawX=… rawY=… rawZ=…` | Primera lectura cruda | 3 enteros int16 |

El firmware **fuerza** el modo de medición escribiendo `0x08` en `POWER_CTL` y
verificando que persiste. Es la causa más frecuente de lecturas congeladas en
un clon barato del ADXL345.

### 5.5 Procedimiento recomendado de campo

1. Estaciona el bus en superficie plana, motor en ralentí, sin gente dentro.
2. Pulsa **RST** y espera al mensaje `setpoint establecido:`.
3. **Anota el valor** — es tu referencia de calibración. `Z` debe rondar ±1.00.
4. Inicia el recorrido. En modo normal, cada evento imprime su `[PICO]`.
5. Al volver a parada, comprueba el nuevo setpoint: debe ser casi idéntico.
6. Interpreta cada `sym` con la tabla de §3.3.

---

## 6. Tabla de pines (pinout)

| Pin Arduino | GPIO | Modo | Componente conectado | Notas |
|---|---|---|---|---|
| **D1** | **GPIO5** | I²C `SCL` | GY-291 → pin `SCL` | Reloj I²C. `PIN_SCL = 5`. Pull-up en el módulo |
| **D2** | **GPIO4** | I²C `SDA` | GY-291 → pin `SDA` | Datos I²C. `PIN_SDA = 4`. `Wire.begin(4,5)` |
| **3V3** | — | Salida 3.3 V | GY-291 → pin `VCC` | **NUNCA 5 V** (el ESP8266 es 3.3 V) |
| **GND** | — | Referencia 0 V | GY-291 → pin `GND` | Masa común, **obligatoria** |
| **USB / VIN (5V)** | — | Entrada 5 V | Cable USB o fuente | Alimenta el NodeMCU. `arduino-cli upload` usa este enlace |
| **TX** | GPIO1 | UART TX | Monitor serie / app | 115200 baud. **No usar como GPIO** |
| **RX** | GPIO3 | UART RX | Monitor serie / app | 115200 baud. **No usar como GPIO** |
| `A0` (ADC0) | GPIO17 | INPUT analógico | — | **Libre**. No se usa |
| `D3` | GPIO0 | — | — | Libre. Botón **FLASH** (a GND = bootloader) |
| `D4` | GPIO2 | — | LED integrado | **Libre**: el firmware NO lo controla |
| `D5`–`D8` | GPIO14/12/13/15 | — | — | Libres (`D8` a GND: arranque falla) |
| `EN` / `RST` | — | — | Botón RST | Reinicio manual. Útil para volver a tomar el setpoint |

**Solo 4 cables de señal + alimentación.** El resto de pines queda libre para
futuras extensiones (ver `docs/developer.md`).

> **D1/D2 son los pines I²C por defecto del ESP8266.** Aun así el sketch los
> declara explícitamente en `PIN_SCL`/`PIN_SDA` y los pasa a `Wire.begin(4, 5)`
> para que la intención sea inequívoca.

---

## 7. Diagrama de conexión

```
                       ┌─────────────────────────────────┐
                       │      NodeMCU ESP8266 (ESP-12E)  │
                       │                                 │
    USB ───────────────┤ USB          ●TX (GPIO1)        │
    (5 V + datos)      │                 │               │
                       │                 │ (solo UART,    │
                       │                 │  NO conectar   │
                       │                 │  al GY-291)    │
                       │                 │               │
                       │            ●GND │               │
                       │                 │               │
                       │            ●3V3 │───────────────┼──────┐
                       │                 │               │      │
                       │            ●D2  │  GPIO4        │      │
                       │            (SDA) │───────────────┼───┐  │
                       │                 │               │   │  │
                       │            ●D1  │  GPIO5        │   │  │
                       │            (SCL) │───────────────┼───┼──┼──┐
                       │                 │               │   │  │  │
                       │   ●D4 ●D3 ●D5… │ libres        │   │  │  │
                       └─────────────────┼───────────────┼───┼──┼──┼────┘
                                         │               │   │  │  │
                               ┌─────────▼───────────────▼───▼──▼──▼────┐
                               │        GY-291 (ADXL345)                 │
                               │                                      │
                               │   ┌──────┐    ┌────────┐    ┌──────┐  │
                     SDA ─────┤   │ SDA  ├────┤ +      │    │ VIN  │  │
                     SCL ─────┤   │ SCL  ├────┤    ┌───┼───►│ GND  │  │
                               │   │ SDO  │    │   │   │    │ CS   │  │
                               │   └──────┘    │   │   │    └──────┘  │
                               │               │   │   │             │
                               │         ┌─────┴───┴─────┐       │
                               │         │  ADXL345      │       │
                               │         │  3.3 V logic  │       │
                               │         └───────────────┘       │
                               │                                      │
                               │   VCC ──── 3V3 (pin 3V3 del NodeMCU)  │
                               │   GND ──── GND                      │
                               └──────────────────────────────────────┘
```

### Lista de conexiones punto por punto

| # | Origen (NodeMCU) | Destino (GY-291) | Cable | Nota |
|---|---|---|---|---|
| 1 | `3V3` | `VCC` | 1 | **3.3 V**, no 5 V. Alimentar el ADXL345 a 3.3 V es siempre seguro |
| 2 | `GND` | `GND` | 1 | Masa común, **obligatoria** |
| 3 | `D2` / GPIO4 | `SDA` | 2 | Datos I²C |
| 4 | `D1` / GPIO5 | `SCL` | 2 | Reloj I²C |
| 5 | `GND` | `SDO` | opcional | Define la dirección I²C. **A GND = 0x53 (por defecto)** |
| 6 | `GND` | `CS` | opcional | El breakout no usa SPI; a GND selecciona I²C |

### Notas prácticas de cableado

* **Longitudes:** cables de 10–30 cm funcionan sin problema. Más de ~50 cm con
  mucho ruido eléctrico puede requerir bajar la velocidad I²C
  (`Wire.setClock(100000)`) o añadir pull-ups.
* **Sin resistencias pull-up externas:** el GY-291 ya las incluye (típ.
  4.7 kΩ a 3.3 V). No añadas más salvo mediciones erráticas.
* **Estabilidad mecánica:** el módulo debe ir **rígidamente atornillado** al
  chasis. Un acelerómetro sobre un soporte flexible amplifica el ruido y
  genera falsos `*` e `!`.
* **Aislamiento del motor:** aleja el cable del GY-291 de la cableería del
  motor, del alternador y del bus de 24 V. Si aparecen lecturas erráticas con
  el motor en marcha, añade un condensador de 100 nF entre `VCC` y `GND`
  **junto al módulo**.
* **Alineación:** el eje que quieras que sea "vertical" (baches/lomos) debe
  ser el que marca el datasheet como Z con el módulo plano. Cualquier rotación
  cambia qué eje reporta cada símbolo, pero la detección sigue siendo válida
  porque el setpoint se toma en la misma posición.

---

## 8. Contrato MQTT

> 🚦 **Estado: en ensayo en seco.** El firmware compila con
> `MQTT_DRY_RUN 1`: **no abre WiFi ni MQTT**, imprime por consola cada
> mensaje con su topic y su payload tal cual saldría, y sigue midiendo. Es lo
> que se usa ahora para validar formato y detección sin suscriptor en el
> broker. Cuando haya algo escuchando, poné `MQTT_DRY_RUN 0` y recompilá.

Todo se publica bajo el prefijo del `MQTT_CLIENT` de `secrets.h`.

| Topic | Cuándo | Payload |
|---|---|---|
| `<client>/accel/online` | al conectar | `true` |
| `<client>/accel/status` | al conectar | `OK` o `NO_DETECTADO` |
| `<client>/accel/setpoint` | cada recálculo del setpoint | `{"x":…,"y":…,"z":…}` en g |
| **`<client>/accel/event/peak`** | **1 por evento** | **el pico, ver abajo** |

> **Los topics `accel/event/summary` y `accel/event/chunk` YA NO EXISTEN** en
> v3. Se unifican en el único `accel/event/peak`.

### `accel/event/peak` — 1 mensaje por evento

```json
{"id":12,"sym":"*","spz":0.93,"dev":0.27,"peak":0.31,"n":200,"ms":2000,"up":63959}
```

| Campo | Significado |
|---|---|
| `id` | identificador del evento, persistente en RTC memory (empieza en 1) |
| `sym` | `. + * - !` — el mismo vocabulario de v1/v2 |
| `spz` | setpoint de Z en el instante del evento (g) |
| `dev` | desviación de Z en el instante del cruce (g), con signo |
| `peak` | **máxima `|devZ|` de toda la ventana (pre + post), en g** |
| `n` | muestras en la ventana (siempre 200) |
| `ms` | duración nominal de la ventana (siempre 2000) |
| `up` | `millis()` del equipo en el inicio de la ventana |

> ⚠️ **`up` NO es hora de reloj.** Son milisegundos desde el arranque de la
> placa, en el instante del **disparo**. No hay NTP/RTC de hora. Solo son
> comparables entre eventos de la misma sesión: si la placa se reinicia, `up`
> vuelve a cero. Para dibujar una jornada de 11 h el cliente debe usar `id`
> (monótono y persistente) como eje y la hora de recepción del mensaje como
> ancla. **Nunca uses `up` como si fuera un timestamp Unix.**

> **Compatibilidad con la app de escritorio:** `app/serial_reader.py`
> `_handle_json()` reconoce un resumen por tener `sym` **e** `id` en el payload
> y **no mira el topic**. El mensaje de pico se sigue listando como resumen de
> evento en la tarjeta «Eventos detectados». Lo que **ya no** habrá es forma de
> onda, porque no hay líneas `EV,`; para ver el gráfico se usa
> `MODE_SERIAL_CSV 1`.

### Suscripción y comando `RECAL`

El dispositivo se suscribe a `<client>/#`. La única orden implementada:

| Topic de entrada | Payload | Efecto |
|---|---|---|
| `<client>/…/RECAL` | `RECAL` | recalibra el setpoint sin reiniciar |

### Identidad en el broker: un ClientId por placa

> ⚠️ **El `MQTT_CLIENT` tiene que ser distinto por placa.** El broker usa el
> ClientId como identidad de sesión: si dos placas conectan con el mismo ID,
> el broker **expulsa a una de las dos de forma continua**. Idealmente el MAC
> de la placa.

### Ejemplo de consumidor en Python

```python
import json, paho.mqtt.client as mqtt

CLIENT = "3f7c1e90b2a45d68"

def on_peak(_, topic, payload):
    if not topic.endswith("/accel/event/peak"):
        return
    d = json.loads(payload)
    print(f"evento #{d['id']} {d['sym']} pico={d['peak']:+.2f}g "
          f"(dev={d['dev']:+.2f}g, up={d['up']}ms)")

c = mqtt.Client(CLIENT + "_admin")
c.username_pw_set("josemanuel", "<clave>")
c.on_message = on_peak
c.connect("www.smartage.cl", 1886, 60)
c.subscribe(f"{CLIENT}/accel/#")
c.loop_forever()
```

---

## 9. Ejemplos de uso

### 9.1 Arranque normal en reposo (ensayo en seco)

```
=== BusAccel-8266-v3 (pico por evento, eje Z, ENSAYO EN SECO) ===
Sin WiFi y sin MQTT: 1 mensaje de PICO por evento por serie.
[FW] version 3
[FW] build 2026-10-08
[RTC] contador de eventos: 0
[ADXL345] OK (begin() correcto)

=== ADXL345 DIAGNOSTICO PROFUNDO ===
WHO_AM_I    = 0xE5  (OK: 0xE5 es el ADXL345)
POWER_CTL   = 0x08  (bit3=1 -> measurement mode)
DATA_FORMAT = 0x00  (rango +-2g)
BW_RATE     = 0x0A  (code 0x0A -> 100 Hz)
OFSX/OFSY/OFSZ = 0/0/0
-- Forzando measurement mode: escribiendo 0x08 en POWER_CTL --
[POWER_CTL] OK: 0x08 persiste -> measurement mode ACTIVO
PRIMERA MUESTRA rawX=     2 rawY=   -41 rawZ=   255
=== FIN DIAGNOSTICO ===

Adquiriendo setpoint (20 lecturas en reposo)...
....................
setpoint establecido: X=0.01 Y=-0.16 Z=1.00
Umbral Z: leve 0.20g | severo 0.24g | retorno 0.15g
Ventana: 100 antes + 100 despues = 200 muestras ~2000ms | 1 mensaje/evento (pico)
Golpea/mueve el sensor: cada evento imprime su PICO (topic + payload).
```

* `Z = 1.00` → el módulo está plano y la gravedad se mide correctamente.
* `Y = -0.16` → leve inclinación lateral permanente. **No es un error**: está en
  el setpoint y se cancela sola.

### 9.2 Evento detectado (ensayo en seco)

Al golpear o mover el sensor con fuerza:

```
[MQTT-OUT] --> 3f7c1e90b2a45d68/accel/event/peak  (150 B)
[MQTT-OUT] {"id":1,"sym":"!","spz":1.00,"dev":-0.24,"peak":0.31,"n":200,"ms":2000,"up":37795}
[PICO] id=1 sym=! peak=0.31g spz=1.00g dev=-0.24g up=37795ms n=200 ms=2000
```

Lectura:

| Campo | Valor | Interpretación |
|---|---|---|
| `sym` | `!` | caída severa (≤ −0.24 g) |
| `dev` | −0.24 g | desviación en el instante del cruce |
| `peak` | 0.31 g | **máxima `|devZ|` de toda la ventana** (pre + post) |
| `up` | 37795 ms | uptime en el disparo (no hora de reloj) |

> **Un solo mensaje por evento.** No hay chunks, no hay 200 líneas `EV,`. Si
> la consola está callada, es que el bus va suave — golpea la mesa para verla
> hablar.

### 9.3 Modo CSV puro (`MODE_SERIAL_CSV 1`)

```
=== BusAccel-8266-v3 (modo serie puro, SIN WiFi) ===
Formato CSV: t,X,Y,Z,setX,setY,setZ,sym
...
20150,0.01,-0.16,1.00,0.01,-0.16,1.00,.
20200,0.02,-0.15,0.99,0.01,-0.16,1.00,.
20250,0.01,-0.17,1.01,0.01,-0.16,1.00,.
20300,0.00,-0.16,1.00,0.01,-0.16,1.00,.
20350,0.01,-0.15,0.99,0.01,-0.16,1.00,.
```

> En este modo **NO** se emite `[PICO]` y **NO** hay WiFi. Es el formato que
> parsea la app de escritorio de `../BusAccel-8266-v2/app/`.

### 9.4 Verificación rápida sobre la mesa

| Movimiento | Desviación esperada | Símbolo |
|---|---|---|
| Módulo inmóvil, sin tocar nada | 0 | `.` |
| Golpe brusco hacia abajo | negativa grande | `!` |
| Empujón brusco hacia arriba | positiva grande | `*` |
| Inclinar y volver lentamente | < 0.20 g | `.` |
| Sacudida corta y seca | negativa fuerte | `!` seguido de al menos 5 líneas con `.` |

---

## 10. Compilación y carga

### 10.1 Instalar `arduino-cli`

```bash
# Instalar arduino-cli si no está instalado
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Si no se agrega al PATH automáticamente:
export PATH="$PATH:$HOME/bin"

arduino-cli version
```

### 10.2 Instalar el core de la placa

```bash
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli core list      # verificar que esp8266:esp8266 aparece instalado
```

### 10.3 Instalar las librerías

```bash
arduino-cli lib install "Adafruit ADXL345"
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib list
```

| Librería | Versión | Papel |
|---|---|---|
| `Adafruit ADXL345` | 1.3.x | `Adafruit_ADXL345_Unified`: verificación y configuración del chip |
| `Adafruit Unified Sensor` | 1.1.x | Dependencia de la anterior; tipos compartidos |
| `Wire` | incluida en el core | Transporte I²C real del firmware |
| `PubSubClient` | 2.8.0 | Cliente MQTT (solo modo MQTT) |

### 10.4 Compilar

```bash
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 .
```

Opciones útiles:

```bash
# Salida compacta y show warnings
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --warnings all .

# Compilar y subir en un solo paso
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 .
```

**Métricas esperadas de memoria:**

| Métrica | Valor esperado |
|---|---|
| RAM | **~30–32 KB de 80 KB** (37–40 %) |
| Flash | **~24–25 %** (~246 KB de 1048576 B) |
| IRAM | ~92 % (baseline del SDK, no margen útil del proyecto) |

### 10.5 Transferir a la placa

```bash
arduino-cli board list                 # localizar el puerto
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 .
```

> Ajusta `/dev/ttyUSB0` al puerto real (`/dev/ttyUSB1`, `/dev/ttyACM0`, `COM3`
> en Windows, `/dev/cu.usbserial-*` en macOS). `arduino-cli board list` lo
> indica.

### 10.6 Monitor serie

```bash
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

### 10.7 Si la carga falla

| Síntoma | Causa | Solución |
|---|---|---|
| `Failed to connect to ESP8266: Timed out waiting for packet header` | el gestor de arranque no entra en modo bootloader | Mantén **FLASH** (GPIO0→GND), pulsa **RST**, suelta FLASH a los 2 s |
| `No serial data received` | el driver USB-SD no está instalado | En Linux, usuario en el grupo `dialout`; en macOS, driver CH340/CP2102 |
| `Error: Unknown FQBN` | core no instalado | `arduino-cli core install esp8266:esp8266` |
| `Sketch uses XXX bytes` tras recompilar | normal | Sin ficheros de partición complicados, cabe de sobra en 4 MB |

---

## 11. Calibración

Todas las constantes del algoritmo están **agrupadas en la cabecera del sketch**
y se pueden ajustar sin tocar la lógica.

### 11.1 Constantes ajustables

| Constante | Valor | Significado |
|---|---|---|
| `ARROW_THRESHOLD_G` | 0.20 | desviación Z (g) para `+`/`-` |
| `SEVERE_FACTOR` | 1.2 | multiplicador → `SEVERE_THRESHOLD_G` = 0.24 g |
| `RETURN_G` | 0.15 | histéresis de retorno a reposo |
| `EVENT_LOCKOUT_MS` | 250 | bloqueo temporal anti-rebote |
| `REST_MS` | 50 | reposo sostenido para desbloquear |
| `SETPOINT_N` | 20 | lecturas para el promedio del setpoint |
| `PRE_N` / `POST_N` | 100 / 100 | muestras antes/después del cruce |
| `SAMPLE_INTERVAL_MS` | 10 | muestreo interno ~100 Hz |
| `MQTT_BUF_MAX` | 384 | buffer de publicación (payload pico ~150 B) |

### 11.2 Procedimiento de calibración

1. **Setpoint en reposo:** con el bus detenido y plano, pulsa RST y anota
   `setpoint establecido:`. `Z` debe rondar ±1.00.
2. **Umbrales:** si aparecen `+`/`-` esporádicamente en reposo (microvibración
   del motor, bomba de aire), sube `ARROW_THRESHOLD_G` a 0.25–0.30 g.
3. **Verificación de severidad:** con el sensor sobre la mesa, un golpe seco
   hacia abajo debe dar `!` con `peak` coherente con la magnitud del golpe.
4. **Recalibración en marcha:** en modo MQTT real, publica `RECAL` a
   `<client>/#` para fijar el setpoint en el ángulo real de montaje sin
   reiniciar.

> 💡 Si cambias umbrales, actualiza también `LEVE_THRESHOLD_G` /
> `SEVERO_THRESHOLD_G` en `app/serial_reader.py` para que las zonas de color de
> la app sigan coincidiendo con el firmware.

---

## 12. Advertencias y precauciones

| ⚠️ | Advertencia |
|---|---|
| **Voltaje** | El ESP8266 y el ADXL345 son **3.3 V**. **NUNCA** conectes el GY-291 a 5 V ni uses los pines `5V`/`VIN` para alimentar el sensor (salvo que el módulo tenga regulador). |
| **I²C** | SDA y SCL son **open-drain**: necesitan pull-up. El GY-291 ya los incluye; no añadas pull-ups externos salvo mediciones erráticas. |
| **Masa común** | El GND entre NodeMCU y GY-291 es **obligatorio**. Sin él, las lecturas I²C son basura. |
| **Pines UART** | TX (GPIO1) y RX (GPIO3) son del monitor serie. **No los uses como GPIO**; romperías la consola y la carga de firmware. |
| **GPIO0 (D3)** | Es el botón **FLASH**: a GND durante el arranque entra en bootloader y no ejecuta el sketch. |
| **GPIO15 (D8)** | Con D8 a GND el arranque **falla** (es pull-down obligatorio). |
| **Estabilidad mecánica** | El módulo debe ir rígidamente atornillado al chasis. Un soporte flexible amplifica el ruido y genera falsos `*` e `!`. |
| **Aislamiento eléctrico** | Aleja el cable del sensor de la cableería del motor, alternador y bus de 24 V. |
| **Credenciales** | `secrets.h` **no se versiona**. Usa `secrets.example.h` como plantilla. |
| **ClientId** | `MQTT_CLIENT` debe ser **distinto por placa** o el broker las expulsa entre sí. |
| **Broker** | El broker `www.smartage.cl:1886` responde `CONNACK 5` (no autorizado) a las credenciales actuales. Confirma usuario/contraseña antes de activar `MQTT_DRY_RUN 0`. |
| **Watchdog** | El watchdog de hardware es de **8 s**. Cualquier `delay()` largo en código propio debe intercalarse con `ESP.wdtFeed()`. |
| **RTC** | El contador `id` vive en RTC user memory: sobrevive a reinicios por red pero **se pierde si se corta la luz**. |

---

## 13. Estructura de archivos

```
BusAccel-8266-v3/
├── BusAccel-8266-v3.ino      # sketch principal (setup + loop + detección + pico + red)
├── secrets.h                 # credenciales reales WiFi/MQTT (local, NO versionado)
├── secrets.example.h       # plantilla de secrets.h (sí versionado)
├── .gitignore                # exclusiones de build, credenciales y artefactos
├── PLAN.md                   # plan de implementación
├── README.md                 # este archivo
└── docs/
    ├── datasheet.md          # documentación de usuario
    ├── developer.md          # explicación del código
    └── BITACORA.md           # problemas, decisiones y lecciones
```

---

## 14. Solución de problemas

| Síntoma | Causa probable | Solución |
|---|---|---|
| `[ADXL345] NO DETECTADO` | cableado VCC/GND/SDA/SCL | revisa conexiones; verifica 3.3 V |
| `WHO_AM_I` ≠ 0xE5 | clon defectuoso o mala lectura I²C | revisa cableado/soldadura; prueba otro módulo |
| `POWER_CTL` no mantiene 0x08 | problema de escritura I²C | revisa cableado/soldadura |
| No se detecta ningún evento | setpoint contaminado o umbral alto | pulsa RST con el bus en reposo; baja `ARROW_THRESHOLD_G` |
| Falsos `*`/`!` en reposo | microvibración o soporte flexible | sube `ARROW_THRESHOLD_G` a 0.25–0.30 g; atornilla el módulo |
| `id` se resetea en cada arranque | RTC sin validar (bug v2) | corregido en v3: validación magic + rango + ≠0 |
| `WiFi timeout` | red no visible / contraseña | verifica `secrets.h` |
| `MQTT auth error` | credenciales broker | confirma usuario/contraseña con el admin del broker |
| `PermissionError` en el puerto | usuario sin grupo `dialout` | `sudo usermod -aG dialout $USER` y reinicia sesión |

Documentación detallada en [`docs/datasheet.md`](docs/datasheet.md).
