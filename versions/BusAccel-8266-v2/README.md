# BusAccel-8266-v2 — Diagnóstico de aceleraciones en bus urbano con MQTT

> **Firmware v2 — PUBLICACIÓN MQTT POR EVENTO SOBRE EL EJE Z.**
>
> Cuando la aceleración en **Z** se sale del rango normal, en **cualquiera de los
> dos sentidos** (subida = `+`/`*`, caída = `-`/`!`), el equipo captura una
> **ventana de 200 muestras a 100 Hz (1 s antes + 1 s después del cruce)** y la
> publica por MQTT. Sin evento no hay tráfico de red.
>
> Cada evento genera **6 mensajes**: 1 resumen + 5 trozos de 40 muestras. El
> cliente administrador reconstruye la forma de onda completa del evento.
>
> **v1 no se toca.** Sigue siendo el instrumento de medición puro (solo CSV por
> serie, sin WiFi) para calibrar umbrales en carretera. Este proyecto es una
> carpeta independiente.

[![Versión](https://img.shields.io/badge/firmware-v2-ff69b4)](#)
[![Protocolo](https://img.shields.io/badge/transporte-MQTT-00e5ff)](#57-contrato-mqtt)
[![Sampling](https://img.shields.io/badge/muestreo-100%20Hz-4c9aff)](#)
[![Sensor](https://img.shields.io/badge/sensor-ADXL345%20(GY--291)-4c9aff)](https://learn.adafruit.com/adafruit-adxl345-3-axis-accelerometer)
[![Placa](https://img.shields.io/badge/placa-NodeMCU%20ESP8266%20ESP--12E-007acc)](https://docs.espressif.com/projects/esp8266/en/latest/hw/nodemcu/nodemcu.html)
[![Serial](https://img.shields.io/badge/consola-115200%20baud-3fb950)](https://www.arduino.cc)

---

## 0. Qué cambió respecto de v1 (migrate rápido)

| Aspecto | v1 | v2 |
|---|---|---|
| Eje que decide el evento | mayor desviación entre X, Y **y Z** | **solo Z** |
| Qué se captura del evento | 1 muestra (el símbolo) | **200 muestras** (1 s antes + 1 s después) |
| Transporte | línea CSV por serie a 20 Hz | **MQTT** (resumen + 5 trozos) |
| Estado del MQTT | — | 🟡 **ensayo en seco** (`MQTT_DRY_RUN 1`): se ve por serie, no sale de la placa |
| Envío | continuo, 20 líneas/s | **solo cuando hay evento** |
| Tiempo real entre muestras | asumido 10 ms fijos | **`dt` real grabado por muestra** |
| Identidad del evento | no existía | **`id` persistente en RTC user memory** |
| Estado de red | ninguno | WiFi + MQTT + watchdog 8 s + ICMP + reinicio con MAC aleatoria |
| Setpoint | 20 muestras | igual, **más** comando `RECAL` por MQTT |
| Formato de línea CSV | `t,X,Y,Z,setX,setY,setZ,sym` | **idéntico** (con `MODE_SERIAL_CSV 1`) |
| Símbolos `.` `+` `*` `-` `!` | 5 | **los mismos 5** |

### Por qué no "1 lectura por segundo, 30 lecturas, un envío cada 30 s"

Esa fue la idea original. **No sirve** para el objetivo del cliente
administrador: a 1 Hz el teorema de Nyquist dice que solo se puede reconstruir
el *nivel promedio* del movimiento, no la forma de onda. La vibración de un bus
va de 10 a 1000 Hz; muestreada a 1 Hz el gráfico sale plano y no permite ver el
bache ni el lomo de toro.

La solución implementada es estrictamente mejor: el muestreo interno ya corría a
**100 Hz**, así que cada evento guarda **2 segundos de datos reales** alrededor
del instante interesting. Además el tráfico de red baja de 20 mensajes/s a
~6 mensajes por evento, y los eventos son raros.

### Por qué se reintrodujo el WiFi que v1 había eliminado

v1 quitó el WiFi a propósito: contaminaba las mediciones. En v2 se conserva el
modo puro (`MODE_SERIAL_CSV 1`) para cuando haga falta medir sin red, y cuando
se publica por MQTT hay dos mitigaciones para que la contaminación no mienta:

1. **`dt` real por muestra.** Si `client.publish()` retrasa el muestreo, el
   cliente sabe cuánto se retrasó y el eje de tiempos del gráfico es fiel.
2. **Volcado por serie después de publicar.** Los datos crudos de cada evento
   también salen por consola (después de entregar al broker, nunca antes), así
   que en campo nunca se pierde un evento por culpa de la red.

---

## 1. ¿Qué es este proyecto?

**BusAccel-8266** es un registrador de aceleraciones para un **bus urbano**
equipado con un acelerómetro ADXL345 (módulo GY-291) conectado por I²C a un
NodeMCU ESP8266.

El objetivo de fondo es **diagnosticar los malos hábitos de conducción**
del bus: baches, bordillos, hoyos, cambios de carril bruscos y, sobre todo,
**saltos peligrosos por encima de un lomo de toro a gran velocidad**. Cada uno
de esos eventos produce una aceleración característica que el acelerómetro
detecta con facilidad.

### Estado actual: MQTT por evento sobre el eje Z (v2)

| Característica | Estado |
|---|---|
| **Publicación MQTT solo cuando hay evento en Z** | 🟡 **En ensayo en seco** (`MQTT_DRY_RUN 1`: se imprime por serie, no sale de la placa) |
| **Ventana de 200 muestras por evento (1 s antes + 1 s después @ 100 Hz)** | ✅ **Activo** |
| **Detección en los dos sentidos: subida `+`/`*` y caída `-`/`!`** | ✅ **Activo** |
| `dt` real entre muestras (gráfico fiel pese a la latencia del WiFi) | ✅ **Activo** |
| Contador de evento persistente (`id`) en RTC user memory | ✅ **Activo** |
| Comando `RECAL` por MQTT para recalibrar el setpoint en marcha | ✅ **Activo** |
| Watchdog de hardware 8 s + watchdog de "sin publicar" 15 min | ✅ **Activo** |
| Sonda ICMP a 8.8.8.8 cada 30 s (5 intentos) | 🟡 **Compilado, inactivo en ensayo seco** |
| Reinicio con MAC aleatoria si el broker no responde | 🟡 **Compilado, inactivo en ensayo seco** |
| Setpoint automático (20 lecturas), recalculable por MQTT | ✅ **Activo** |
| Diagnóstico profundo de registros ADXL345 al arranque | ✅ **Activo** |
| Volcado CSV de cada evento por consola (rescate en campo) | ✅ **Activo** |
| Modo serie puro (`MODE_SERIAL_CSV 1`) para la app de escritorio | ✅ **Activo** |
| Formato de línea CSV `t,X,Y,Z,setX,setY,setZ,sym` | ✅ **Igual que v1** |
| App de escritorio con gráfico en tiempo real (`app/`) | ✅ **Activo (modo serie)** |
| LED / indicador luminoso | ❌ No se usa (el LED del NodeMCU no parpadea) |
| EEPROM (persistencia de datos) | ❌ **No implementado, y no hace falta** (ver §1.1) |
| Cola de eventos con el broker caído | ❌ **Descartado por diseño** (ver §1.1) |

> **Nota sobre la EEPROM.** La primera versión de este plan era "1 lectura por
> segundo, 30 lecturas al EEPROM, un envío cada 30 s". Se descartó: el EEPROM
> emulado del ESP8266 es **un único sector de 4096 bytes sin wear leveling**, y
> `commit()` borra y reescribe el sector entero. Con un commit cada 30 s son
> ~105 000 ciclos de borrado al año (el flash del ESP8266 aguanta ~10 000), es
> decir la placa muere en ~3 años. Escribir cada segundo la mata en ~11 horas.
>
> El buffer de la ventana vive en **RAM** (2400 bytes) y no se persiste. Si el
> equipo se reinicia o se corta la luz, se pierde como máximo la ventana del
> evento en curso: en una jornada de 11 h es irrelevante para reconstruir el
> comportamiento de conducción. El único dato persistente es el contador `id`,
> que va en la **RTC user memory** (alimentada, sobrevive al reinicio) y no toca
> la flash.

> **Decisiones de diseño tomadas con el usuario**, para que quede registrado por
> qué el firmware hace lo que hace:
>
> - Umbrales: se **mantienen** los de v1 (leve 0.20 g, severo 0.24 g, retorno
>   0.15 g). Ya estaban calibrados sobre datos reales.
> - Con el broker caído en el instante del evento, el evento **se descarta** y el
>   equipo sigue midiendo. Sin cola, sin reinicio por ese motivo.
> - Con el broker caído de forma sostenida, **sí** se reinicia (con MAC
>   aleatoria), igual que hace la placa de temperatura.


---

## 2. Hardware y especificaciones

### Placa objetivo

| Parámetro | Valor |
|---|---|
| **Placa** | NodeMCU ESP8266 (módulo ESP-12E) |
| **Microcontrolador** | ESP8266EX @ 80 MHz |
| **Flash** | 4 MB |
| **SRAM** | ~80 KB útiles (~40–50 KB libres con este sketch) |
| **Alimentación** | 5 V por USB (pines `VIN`/`5V`) o 3.3 V por el pin `3V3` |
| **Consumo (sin WiFi)** | ~70–90 mA en operación; picos hasta ~120 mA al arrancar |
| **Consumo en reposo (light sleep no usado)** | el firmware permanece en `loop()` activo; el ESP8266 nunca duerme |
| **Interfaz de usuario** | exclusivamente la consola serie a 115200 baud |
| **FQBN (arduino-cli)** | `esp8266:esp8266:nodemcuv2` |
| **Core Arduino** | `esp8266:esp8266` (3.x) |

### Sensor

| Parámetro | Valor |
|---|---|
| **Módulo** | GY-291 (breakout del ADXL345) |
| **Chip** | Analog Devices ADXL345, acelerómetro MEMS 3 ejes |
| **Interfaz** | I²C, dirección **0x53** (fija, selector de solder de la placa a GND) |
| **Rango configurado** | **±2 g** (`ADXL345_RANGE_2_G`) |
| **Escala de conversión** | 256 LSB/g → `g = raw / 256.0f` |
| **Frecuencia de datos** | **100 Hz** (`ADXL345_DATARATE_100_HZ`), code `0x0A` |
| **Rango legible** | ±16 g por registro, recortado a ±2 g por `DATA_FORMAT` |
| **Resolución real** | 10 bits en ±2 g → ~3.9 mg/LSB |
| **Consumo del sensor** | ~140 µA típico |
| **Lectura** | *burst read* directo de 6 bytes (`DATAX0..DATAZ1`), little-endian int16 |
| **Identificador** | `WHO_AM_I (0x00) = 0xE5` |

> **Nota sobre la librería Adafruit:** el sketch instancia
> `Adafruit_ADXL345_Unified(12345)` y llama a `begin()`, `setRange()` y
> `setDataRate()` para la **verificación de presencia** y la **configuración del
> chip**. Las lecturas de datos que alimentan la detección **no** pasan por
> `accel.getEvent()`, sino por acceso directo a registros con `Wire`
> (`readRawAxes()`), lo que evita conversiones a m/s² y da control total del
> timing. El primer argumento `12345` es el *sensor ID* interno de la librería,
> **no** una dirección I²C.

---

## 3. Principio de funcionamiento

### 3.1 El setpoint: la línea base de la marcha

El bus no está perfectamente plano: tiene pendiente, la suspensión está en
equilibrio, el vehículo se inclina en las curvas. Un acelerómetro captura
**todas** esas inclinaciones como valores distintos de cero. Por eso el
firmware no mide la aceleración absoluta, sino la **desviación respecto a una
referencia**: el **setpoint**.

El setpoint se obtiene promediando **20 lecturas** (`SETPOINT_N`) tomadas con el
bus **detenido y en reposo**:

```
spX = (gx0 + gx1 + ... + gx19) / 20
spY = (gy0 + gy1 + ... + gy19) / 20
spZ = (gz0 + gz1 + ... + gz19) / 20
```

Con el bus típicamente horizontal y el módulo plano, lo normal es ver `Z ≈ +1.00`
(la gravedad), `X ≈ 0.00` y `Y ≈ 0.00`. Esos valores **no son un error**: son
la gravity + inclinación de referencia, y quedan restados automáticamente.

El setpoint se **recalcula en silencio** cada 20 muestras de reposo, y
**únicamente** mientras la máquina de estados está en `ST_IDLE`. Esto permite
que la referencia siga lentamente a la postura real del bus (curvas largas,
pendientes) sin que un evento en curso la contamine.

### 3.2 La desviación: lo que realmente importa

En cada muestra a 100 Hz se calcula:

```
devX = gx - spX          (aceleración en el eje longitudinal del bus)
devY = gy - spY          (aceleración lateral — curvas y cambios de carril)
devZ = gz - spZ          (aceleración vertical — baches, lomos, hoyos)
```

La gravedad desaparece del cálculo porque está contenida en el setpoint. Solo
queda la **aceleración dinámica**, que es exactamente lo que delata una
conducción brusca.

> **En v2 solo el eje Z decide.** `devX` y `devY` se siguen calculando y se
> transmiten, pero **no disparan eventos**: son vibración del chasis, del motor y
> del suelo, y en v1 producían falsos positivos que inundaban el registro.

### 3.3 El símbolo: sentido y severidad sobre el eje Z

La decisión se toma sobre `devZ = gz − spZ`, y solo sobre él:

1. **El sentido** (signo): positiva → el bus sube (compresión de suspensión,
   lomo de toro). Negativa → el bus baja (bache, hoyo).
2. **La magnitud**: si supera **1.2× el umbral leve**
   (`SEVERE_FACTOR = 1.2`, o sea 0.24 g), el evento se considera **severo**.

De ahí los cinco símbolos:

| `devZ` | Símbolo |
|---|---|
| `≥ +0.24` | `*` elevación severa |
| `> +0.20` | `+` elevación leve |
| `≤ −0.24` | `!` caída severa |
| `< −0.20` | `-` caída leve |
| `\|devZ\| ≤ 0.20` | `.` reposo |

Un evento produce **un** símbolo, en **una** ventana de 200 muestras. No un
carácter cada 50 ms como en v1.

### 3.4 Supresión del rebote inercial (máquina de estados)

Un problema clásico: cuando el bus pasa por un bache, el sistema de suspensión
rebota y el acelerómetro registra una **segunda aceleración en sentido
contrario** unos 100–200 ms después. Un detector ingenuo generaría **dos** eventos
por un mismo bache.

Para evitarlo, el firmware usa una máquina de estados de dos estados:

```
        ┌──────────────────────────────────────────────┐
        │                                              │
        ▼                                              │
    ┌────────┐   primer cruce del umbral (|devZ| >     │  (a) millis() >= lockUntilMs
    │ST_IDLE │────────────────── 0.20 g)                │  (b) reposo sostenido
    └────────┘   abre la ventana y bloquea ─────────────┼───────► ST_IDLE
                 ┌──────────┐                           │          (>= REST_MS = 50 ms
                 ▼          │                           │           con |devZ| <= 0.15 g)
             ┌──────────┐   │                           │  (c) LA VENTANA HA TERMINADO
             │ST_LOCKED │   │                           │      (!collecting)
             └──────────┘   │                           │
```

* **En `ST_IDLE`**: si `|devZ|` no supera `ARROW_THRESHOLD_G`, se marca reposo y se
  acumula la muestra para el recálculo silencioso del setpoint. En el **primer**
  cruce se elige el símbolo, se pasa a `ST_LOCKED`, se arma el bloqueo temporal de
  `EVENT_LOCKOUT_MS = 250 ms` y se arranca la ventana de evento.
* **En `ST_LOCKED`**: se ignoran todos los cruces posteriores — el rebote no
  produce ningún evento. El desbloqueo exige **las tres condiciones a la vez**:
  * **(a)** que hayan pasado los 250 ms de bloqueo temporal;
  * **(b)** que `|devZ|` haya permanecido dentro de `±RETURN_G = 0.15 g` durante
    `REST_MS = 50 ms` **seguidos**;
  * **(c)** que **la ventana del evento anterior haya terminado**.

> ⚠️ **La condición (c) es la que no se puede omitir en v2.** La ventana dura
> **2000 ms**, mucho más que `lockout + reposo` (300 ms). Sin ella, un segundo
> evento a los 300 ms **pisaría `eventBuf` mientras la ventana anterior se sigue
> llenando**, y la publicación saldría con muestras de dos eventos distintos
> mezcladas.

La histéresis (`RETURN_G = 0.15 g` < `ARROW_THRESHOLD_G = 0.20 g`) hace que el
regreso a reposo sea **más exigente** que el disparo, lo que elimina el
oscilamiento en el borde del umbral.

---

## 4. Salida serie

> En v2 el modo normal **no** emite una línea por tick. La salida por serie es el
> **ensayo en seco** (`MQTT_DRY_RUN 1`) más el volcado de cada evento. La línea CSV
> continua de v1 sigue existiendo, pero solo si compilas con
> `MODE_SERIAL_CSV 1`.

### 4.0 El volcado del evento (modo normal)

Cada evento cierra su ventana con 200 líneas:

```
EV,t0,Xraw,Yraw,Zraw,dt,sym
EV,37795,30,33,235,10,.
```

`t0` se repite en las 200 líneas, así que el tiempo real se reconstruye sumando
los `dt`. Ver §5.7 y `docs/datasheet.md` §2.5.2.

### 4.1 La línea CSV continua (solo `MODE_SERIAL_CSV 1`)

Con `MODE_SERIAL_CSV 1` cada tick de 20 Hz emite **una línea CSV completa**,
ya no un carácter suelto. Es el mismo algoritmo y los mismos símbolos del
ciclo 15: lo único que cambia es que el símbolo viaja **dentro** de la línea,
en el campo `sym`.

### 4.1 Formato de la línea

```
t,X,Y,Z,setX,setY,setZ,sym
```

| Campo | Tipo | Significado |
|---|---|---|
| `t` | entero (`unsigned long`) | `millis()` del ESP8266, en milisegundos desde el arranque |
| `X`,`Y`,`Z` | float, 2 decimales | Aceleración **actual** en g (`raw / 256.0f`, rango ±2 g) |
| `setX`,`setY`,`setZ` | float, 2 decimales | **Setpoint** (línea base en reposo) de cada eje, en g |
| `sym` | 1 carácter ASCII | Símbolo de detección de este tick: `.` `+` `*` `-` `!` |

Cada línea termina en `\n` (CRLF), de modo que un simple *split por líneas*
—en un terminal, en `awk`, en Python o en la app de escritorio— ya devuelve
cada muestra.

**Línea real de la placa en reposo (módulo plano, Z hacia arriba):**

```
248150,0.14,0.17,0.91,0.14,0.16,0.92,.
248200,0.14,0.17,0.91,0.14,0.16,0.92,.
248250,0.15,0.16,0.92,0.14,0.16,0.92,.
248300,0.14,0.18,0.93,0.14,0.16,0.92,.
248350,0.13,0.16,0.94,0.14,0.16,0.92,.
248400,0.15,0.17,1.16,0.14,0.16,0.92,*     ← lomo de toro: +0.24 g → severo
248450,0.14,0.18,1.11,0.14,0.16,0.92,.
248500,0.13,0.17,0.94,0.14,0.16,0.92,.
248550,0.14,0.16,0.91,0.14,0.16,0.92,.
```

> Los tres primeros campos de la desviación son la clave para un análisis
> posterior: `Z - setZ` es exactamente la desviación que la máquina de estados
> evaluó para decidir el símbolo. En el ejemplo: `1.16 - 0.92 = +0.24 g` ⇒ `*`.

### 4.2 Tabla de símbolos (campo `sym`)

| Símbolo | Significado | Condición sobre la desviación del eje ganador | Lectura del conductor |
|:---:|---|---|---|
| `.` | **Reposo** | `\|dev\| ≤ 0.20 g` en los 3 ejes (o estado bloqueado) | Marcha suave, superficie regular |
| `+` | **Elevación leve** | `+0.20 g < dev < +0.24 g` | Subida suave: bordillo, catenaria, Branding de vía |
| `*` | **Elevación SEVERA** | `dev ≥ +0.24 g` (1.2× el umbral leve) | **Lomo de toro a alta velocidad, salto peligroso** |
| `-` | **Caída leve** | `-0.24 g < dev < -0.20 g` | Descenso suave, transición de calzada |
| `!` | **Caída SEVERA** | `dev ≤ -0.24 g` | **Hoyo profundo / bache de magnitud relevante** |

> Todos los caracteres son **ASCII puro** (0x2E, 0x2B, 0x2A, 0x2D, 0x21).
> Sin UTF-8, sin secuencias de escape: cualquier terminal los muestra
> correctamente, y la app puede mapearlos a colores sin ambigüedad.

### 4.3 Lectura rápida: del CSV al "flujo" de símbolos

Para leer el recorrido como en el ciclo 15, basta con quedarse con el campo 8:

```bash
# Extraer SOLO los símbolos, en orden temporal
awk -F, '/^[0-9]+,/ {printf "%s", $8}' recorrido.log; echo
# ..........*......-.....!....*......+..........-....!......*.........
#            ↑     ↑            ↑      ↑       ↑     ↑            ↑     ↑
#         bache lomo         catenaria hoyo  lomo       hoyo    lomo  hoyo
#         leve   de toro      leve    prof.  de toro     prof.   de toro prof.
```

`+`/`-` = 0.20–0.24 g (magnitud) · `*`/`!` = ≥ 0.24 g (severos)

> **Antes** (ciclo 15) este "flujo" era literalmente lo que salía por el
> puerto, y se partía en líneas de 50 caracteres. **Ahora** es una proyección
> del campo `sym`; el salto de línea cada 50 caracteres ya no existe (la
> constante `LINE_CHARS` se conserva definida pero **sin uso**).

### 4.4 Parámetros de la línea de salida

| Parámetro | Valor | Significado |
|---|---|---|
| Frecuencia de muestreo | 100 Hz (`SAMPLE_INTERVAL_MS = 10`) | resolución temporal de la detección |
| Frecuencia de impresión | 20 Hz (`PRINT_INTERVAL_MS = 50`) | **1 línea CSV cada 50 ms** |
| Caracteres por línea | — (eliminado) | cada línea es una muestra completa, de ~35 bytes |
| Ritmo de datos | ~700 B/s | muy por debajo del límite de 115200 baud (~11 kB/s) |
| Duración de un evento bloqueado | ≥ 250 ms | mínimo 5 líneas con `sym='.'` tras cada símbolo |
| Periodo mínimo entre eventos | 300 ms | ~3.3 eventos/s como máximo teórico |
| Setpoint | 20 muestras (200 ms) | se recalcula en silencio cada 20 muestras en reposo |


---

## 5. Cómo operar

### 5.1 Conexión

1. Conecta el **GY-291** al NodeMCU según el diagrama de §7.
2. Alimenta y conecta el **NodeMCU por USB** a tu ordenador.
3. Compila y carga el firmware (§9).

### 5.2 Consola serie

Abre el monitor serie a **115200 baud, 8N1, sin control de flujo**:

```bash
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

Verás el banner, el diagnóstico y los 20 puntos de adquisición, y a partir de
ahí **una línea CSV cada 50 ms** (§4). También puedes usar **cutecom**,
**PuTTY** (`Serial` / `115200` / `8-N-1` / `Flow control: None`),
**Arduino IDE** → *Monitor Serie* (pestaña inferior derecha) o
`screen /dev/ttyUSB0 115200`.

> 💡 **No te quedes en el terminal:** para ver la aceleración dibujada en el
> tiempo, usa la **app de escritorio** (§5.6). El terminal sirve para capturar
> logs, no para interpretarlos.

> **Además, no verás el LED parpadear.** Todo el estado del dispositivo está en
> la consola. Si la placa parece "muerta", abre el monitor: ahí está hablando.

> **Sobre el gestor de arranque:** al pulsar RST también puede aparecer
> un mensaje del cargador ROM del ESP8266 a 74880 baud antes del banner. Es
> normal y desaparece al conmutar a 115200.

### 5.2.1 Qué se ve con el ensayo en seco (`MQTT_DRY_RUN 1`)

En este modo la placa **no abre WiFi ni MQTT**: arranca, calibra el setpoint y se
queda midiendo en silencio. Cuando detecta una desviación de Z fuera de rango
imprime por consola, en este orden:

1. **El resumen y los 5 trozos que se publicarían**, cada uno con su topic:
   ```
   [MQTT-OUT] --> 3f7c1e90b2a45d68/accel/event/summary  (110 B)
   [MQTT-OUT] {"id":1,"sym":"!","spz":0.92,"dev":-0.24,"peak":1.03,"n":200,"ms":2000,"up":37795,"count":1}
   [MQTT-OUT] --> 3f7c1e90b2a45d68/accel/event/chunk  (617 B)
   [MQTT-OUT] {"id":1,"i":0,"n":5,"up0":37795,"z":[235,…],"x":[30,…],"y":[33,…],"dt":[10,…]}
   … (5 trozos)
   ```
2. **El pie, con el volumen real que habría salido**:
   ```
   [MQTT-OUT] === evento 1: 6 mensajes, 2260 B, 223ms (ENSAYO EN SECO: nada enviado al broker) ===
   ```
3. **La línea de estado del evento** y las 200 líneas CSV de la ventana (`EV,…`),
   idénticas a las del modo serie de v1.

> ⚠️ **No hay salida si no hay evento.** La placa está midiendo, pero solo habla
> cuando Z se sale del rango. Es el comportamiento correcto: si la consola está
> callada, es que el bus va suave — golpea la mesa para verla hablar.
>
> 💡 Cada evento son ~2.6 KB a 115200 baud. Con el scroll automático vas a perder
> líneas: para capturar el log completo, redirige la salida a un archivo.

### 5.3 Secuencia de arranque

1. **Banner**:
   ```
   === BusAccel-8266 DIAG CSV (ciclo 16, t,X,Y,Z,setX,setY,setZ,sym) ===
   ```
2. **Verificación del sensor**:
   ```
   [ADXL345] OK (begin() correcto)
   ```
3. **Diagnóstico profundo de registros** (§5.4).
4. **Adquisición del setpoint** — 20 puntos impresos (uno por lectura):
   ```
   Adquiriendo setpoint (20 lecturas en reposo)...
   ....................
   setpoint establecido: X=0.01 Y=-0.15 Z=1.00
   ```
5. **Flujo continuo de líneas CSV** a 20 Hz, una por tick, hasta `reset`.

> ⚠️ **Coloca el bus en reposo antes de los 3 primeros segundos.** El setpoint
> se fija sobre las 20 primeras lecturas; si el bus está en movimiento, la
> referencia quedará contaminada y todo se detectará como desviación.

### 5.4 Diagnóstico de registros (se ejecuta siempre, al arrancar)

| Línea | Qué comprueba | Valor esperado |
|---|---|---|
| `WHO_AM_I = 0xE5` | Identidad del chip | `0xE5` |
| `POWER_CTL = 0x08` | Bit 3 = measurement mode | `bit3=1` |
| `DATA_FORMAT = 0x00` | Bits 0-1 = rango | `±2 g` |
| `BW_RATE = 0x0A` | Code 0x0A | `100 Hz` |
| `OFSX/OFSY/OFSZ` | Offsets de calibración del fabricante | `0/0/0` (habitual) |
| `[POWER_CTL] OK: 0x08 persiste` | Escritura I2C efectiva | `OK` |
| `PRIMERA MUESTRA rawX=… rawY=… rawZ=…` | Primera lectura cruda | 3 enteros int16 |

El firmware **fuerza** el modo de medición escribiendo `0x08` en `POWER_CTL` y
verificando que persiste. Es la causa más frecuente de lecturas congeladas en
un clon barato del ADXL345.

### 5.5 Procedimiento recomendado de campo

1. Estaciona el bus en superficie plana, motor en ralentí, sin gente dentro.
2. Pulsa **RST** y espera al mensaje `setpoint establecido:`.
3. **Anota el valor** — es tu referencia de calibración. `Z` debe rondar ±1.00.
4. Inicia el recorrido. La consola emite líneas CSV; mejor aún, **abre la app**
   (§5.6) y observa la curva en tiempo real.
5. Al volver a parada, comprueba el nuevo setpoint: debe ser casi idéntico.
6. Interpreta cada `sym` con la tabla de §4.2.

---

## 5.6 Aplicación de escritorio (Monitor)

> Nueva en el **ciclo 16**. Es la forma cómoda de usar el instrumento: el
> firmware sigue siendo puro serial, pero el PC dibuja la señal, marca los
> eventos y cuenta incidencias. Documentación completa en
> [`app/README.md`](app/README.md).

### Qué hace

`app/` es una aplicación de escritorio (Python + `pywebview`/Qt + `pyserial`)
que:

- **Lee el puerto serial** (`/dev/ttyUSB0`, 115200 baud) y **parsea las líneas
  CSV** `t,X,Y,Z,setX,setY,setZ,sym`. Ignora automáticamente el banner, el
  diagnóstico y los puntos de adquisición (sólo acepta líneas de **8 campos**
  cuyo primer campo sea un entero).
- **Grafica en tiempo real la aceleración**: eje **X = tiempo** (ventana
  deslizante de **60 s** por defecto, ajustable de **10 a 120 s**), eje **Y =
  aceleración en g**.
- Dibuja la **curva de Z en cian con glow**, la **línea horizontal punteada del
  setpoint de Z** (se lee de los campos `setX/setY/setZ`, no es una constante
  de la app) y **zonas de color**:
  - por **encima** del setpoint: **ámbar** = elevación leve (0.20–0.24 g) ·
    **rojo** = elevación severa (≥ 0.24 g)
  - por **debajo**: **azul** = caída leve · **magenta** = caída severa
- Pone un **marcador con glow** en cada detección (verde `+`, azul `-`,
  rojo `*`, magenta `!`), un **badge** con el último símbolo y su descripción,
  **estadísticas** (Hz, muestras, t, setpoint Z) y **contadores por tipo**,
  más una **lista de eventos recientes**.
- Permite **pausar**, **limpiar el buffer**, y mostrar los ejes X e Y
  superpuestos al de Z.

Los **umbrales mostrados en las zonas** (0.20 / 0.24 g) vienen del backend
y coinciden con `ARROW_THRESHOLD_G` y `SEVERE_THRESHOLD_G` del firmware; si
cambias una constante en el sketch, actualiza `LEVE_THRESHOLD_G` /
`SEVERO_THRESHOLD_G` en `app/serial_reader.py` para que las zonas coincidan.

### Instalación

```bash
cd BusAccel-8266
python3 -m venv venv
source venv/bin/activate
pip install -r app/requirements.txt
```

Dependencias (`app/requirements.txt`): `pyserial`, `pywebview`, `qtpy`,
`PyQt5`, `PyQtWebEngine`. El frontend es **canvas HTML5 nativo, sin CDN**: no
necesita internet.

### Ejecución

```bash
cd /ruta/a/BusAccel-8266-v2
source venv/bin/activate
python3 app/app.py
```

Opciones de línea de comandos:

| Opción | Default | Descripción |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Puerto serie. `SIM` arranca el simulador |
| `--baud` | `115200` | Velocidad del puerto |
| `--sim` | — | Arrancar en **modo simulador** (datos sintéticos a 20 Hz, sin placa) |
| `--browser` | — | Forzar el navegador en lugar de la ventana nativa |
| `--http-port` | `8765` | Puerto del servidor HTTP local (si está ocupado, elige otro) |
| `--debug` | — | Devtools de `pywebview` |

Ejemplos:

```bash
python3 app/app.py                      # placa en /dev/ttyUSB0
python3 app/app.py --port /dev/ttyUSB1   # otra placa
python3 app/app.py --sim                 # probar la UI sin hardware
python3 app/app.py --browser             # sin ventana nativa
```

La app levanta un **servidor HTTP local en `127.0.0.1`** que sirve el frontend y
una pequeña API JSON, y abre la interfaz en una **ventana nativa**. Si
`pywebview` no está disponible o falla, **cae automáticamente al navegador**.

### Uso rápido

1. Con la placa conectada y el firmware cargado, lanza `python3 app/app.py`.
2. Elige el **puerto** en el desplegable y pulsa **Conectar**.
3. Si no hay placa, pulsa **▶ Modo simulador** para ver la interfaz funcionando.
4. Ajusta la **ventana de tiempo** (10–120 s) y usa los toggles **X** / **Y**.
5. Verás la banda gris de reposo, las zonas de severidad y los marcadores con
   glow en cada detección.

> ⚠️ **Permisos de puerto serie:** si aparece un `PermissionError`, añade tu
> usuario al grupo `dialout` (`sudo usermod -aG dialout $USER`) y vuelve a
> iniciar sesión. Sólo un programa puede tener el puerto abierto: **cierra
> `arduino-cli monitor`, cutecom o el Monitor Serie** antes de conectar la app.

---

## 5.7 Contrato MQTT

> 🚦 **Estado: en ensayo en seco.** El firmware está compilado con
> `MQTT_DRY_RUN 1`: **no abre WiFi ni MQTT**, imprime por consola cada mensaje
> con su topic y su payload tal cual saldría, y sigue midiendo. Es lo que se está
> usando ahora mismo para validar formato y detección sin suscriptor en el broker.
> Cuando haya algo escuchando del otro lado, poné `MQTT_DRY_RUN 0` y recompilá.
>
> El broker `www.smartage.cl:1886` responde `CONNACK 5` (no autorizado) a las
> credenciales actuales, incluso a clientes anónimos: el rechazo es del broker,
> no del firmware. Hay que confirmar usuario/contraseña antes de activar.

Todo se publica bajo el prefijo del `MQTT_CLIENT` de `secrets.h`. Los topics
que se emiten son:

| Topic | Cuándo | Payload |
|---|---|---|
| `<client>/accel/online` | al conectar | `true` |
| `<client>/accel/status` | al conectar | `OK` o `NO_DETECTADO` |
| `<client>/accel/setpoint` | cada recálculo del setpoint | `{"x":…,"y":…,"z":…}` en g |
| `<client>/accel/event/summary` | 1 por evento | el resumen, ver abajo |
| `<client>/accel/event/chunk` | 5 por evento | 40 muestras cada uno, ver abajo |

El dispositivo **se suscribe** a `<client>/#`, así que puede recibir órdenes.
La única implementada es:

| Topic de entrada | Payload | Efecto |
|---|---|---|
| `<client>/…/RECAL` | `RECAL` | recalibra el setpoint sin reiniciar |

### `accel/event/summary` — 1 mensaje por evento

```json
{"id":47,"sym":"*","spz":0.98,"dev":0.31,"peak":0.58,"n":200,"ms":2000,"up":418273,"count":47}
```

| Campo | Significado |
|---|---|
| `id` | identificador del evento, persistente en RTC memory (empieza en 1) |
| `sym` | `.` `+` `*` `-` `!` — el mismo vocabulario de v1 |
| `spz` | setpoint de Z en el instante del evento (g) |
| `dev` | desviación de Z en el instante del cruce (g), con signo |
| `peak` | **máximo** `\|desviación\|` dentro de toda la ventana (g) |
| `n` | muestras en la ventana (siempre 200) |
| `ms` | duración de la ventana (siempre 2000) |
| `up` | `millis()` del equipo en el inicio de la ventana |

Con `summary` solamente, el cliente ya puede dibujar un **marcador por evento**
sobre un eje temporal sin esperar los trozos. `count` es el total de eventos
detectados desde el último corte de energía.

> ⚠️ **`up` NO es hora de reloj.** Son milisegundos desde el arranque de la placa,
> así que solo son comparables entre eventos de la misma sesión: si la placa se
> reinicia, `up` vuelve a cero. Para dibujar una jornada de 11 h el cliente tiene
> dos opciones: (a) usar `id`, que es monótono y persistente, como eje y usar la
> hora de recepción del mensaje como ancla; (b) esperar a que se añada la hora de
> reloj (epoch NTP) como campo `ep`, que es lo que se hará al activar el broker.
> **Nunca uses `up` como si fuera un timestamp Unix.**

### `accel/event/chunk` — 5 mensajes por evento

```json
{"id":47,"i":0,"n":5,"up0":418273,"z":[256,258,261,270,310,…],"x":[0,3,-1,4,…],"y":[0,-2,1,0,…],"dt":[10,10,10,10,10,…]}
```

| Campo | Significado |
|---|---|
| `id` | el mismo `id` del resumen → así se reensambla la ventana |
| `i` | índice del trozo, de 0 a `n-1` |
| `n` | cuántos trozos tiene el evento (siempre 5) |
| `up0` | `millis()` del inicio de la ventana, para alinear todos los trozos |
| `z`, `x`, `y` | **cuentas crudas del ADXL345**, no floats |
| `dt` | ms reales entre cada muestra y la anterior |

El **orden de los campos es fijo**: `z`, `x`, `y`, `dt`. Los cuatro vectores
tienen exactamente 40 elementos (`CHUNK_N`).

**Tamaño real:** 577 B en reposo, 617 B con el golpe, y **902 B en el peor caso**
(todos los valores a `-512`, que es el límite de ±2 g). El buffer de publicación
es de 1024 B, así que el peor caso entra con 122 B de margen.

**Los valores van como enteros crudos, no como float.** Es la resolución nativa
del sensor (±2 g → 256 LSB/g), ocupa 3-4 caracteres en vez de 6-7, y no pierde
nada. Para obtener g: `valor / 256.0`. Ejemplo: `z = 256` → 1.00 g.

**`x` e `y` se envían aunque la detección sea solo sobre Z**, para que el
cliente administrador pueda graficar cualquier eje sin volver a flashear.

**`dt` es el tiempo real**, no 10 ms fijos. Es lo que hace que el gráfico sea
fiel: si el stack WiFi retrasó el muestreo, el cliente lo ve en vez de recibir
una línea recta inventada. Para alinear el eje temporal del trozo *i*:

```python
# en Python
i, chunk_n = 0, 5
t = up0 + sum(data["dt"][i*chunk_n : (i+1)*chunk_n])
```

### Reensamblado rápido (ejemplo en Python)

```python
import json, paho.mqtt.client as mqtt

CLIENT = "3f7c1e90b2a45d68"
ventanas = {}

def on_chunk(_, topic, payload):
    if not topic.endswith("/accel/event/chunk"):
        return
    d = json.loads(payload)
    v = ventanas.setdefault(d["id"], {"up0": d["up0"], "dt": [], "z": [], "x": [], "y": []})
    for campo in ("z", "x", "y", "dt"):      # el orden importa
        v[campo] += d[campo]

def on_summary(_, topic, payload):
    s = json.loads(payload)
    print(f"evento #{s['id']} {s['sym']} pico={s['peak']:+.2f}g")

c = mqtt.Client(CLIENT + "_admin")
c.username_pw_set("josemanuel", "<clave>")
c.on_message = on_message
c.connect("www.smartage.cl", 1886, 60)
c.subscribe(f"{CLIENT}/accel/#")
c.loop_forever()
```

### Identidad en el broker: un ClientId por placa

> ⚠️ **El `MQTT_CLIENT` tiene que ser distinto por placa.** El broker usa el
> ClientId como identidad de sesión: si la placa del acelerómetro y la de
> temperatura conectan con el mismo ID, el broker **expulsa a una de las dos de
> forma continua** y ninguna publica de forma estable. En este proyecto:
> temperatura = `6abd16e311a8892fd2927dde`, acelerómetro = `3f7c1e90b2a45d68`.
>
> Si añades una tercera placa, dale otro ID. Idealmente el MAC de la placa.

---

## 6. Tabla de pines (pinout)

| Pin Arduino | GPIO | Modo | Componente conectado | Notas |
|---|---|---|---|---|
| **D1** | **GPIO5** | I²C `SCL` (OPEN_DRAIN) | GY-291 → pin `SCL` | Reloj I²C. Definido en `PIN_SCL = 5` |
| **D2** | **GPIO4** | I²C `SDA` (OPEN_DRAIN) | GY-291 → pin `SDA` | Datos I²C. Definido en `PIN_SDA = 4` |
| **3V3** | — | Salida 3.3 V | GY-291 → pin `VCC` | Alimenta el sensor. **No usar `5V`/`VIN` para el ADXL345 si el módulo no tiene regulador** |
| **GND** | — | Referencia 0 V | GY-291 → pin `GND` | Masa común, obligatoria |
| **USB / VIN (5V)** | — | Entrada 5 V | Cable USB o fuente | Alimenta el NodeMCU. `arduino-cli upload` usa este mismo enlace |
| **TX / RX (D0/D1 UART)** | GPIO1 / GPIO3 | UART0 | Convertidor USB-SD del NodeMCU | **OJO**: `RX` del NodeMCU es el pin `D4`, no un `D1` de numbering Arduino. No se usa como I²C |
| `A0` (ADC0) | GPIO17 | INPUT analógico | — | **Libre**. El firmware no lo usa (no hay lecturas de voltaje de batería) |
| `D3` | GPIO0 | — | — | Libre. Botón **FLASH** (GPIO0 a GND = modo bootloader) |
| `D4` | GPIO2 | — | LED integrado del NodeMCU | **Libre**: el firmware NO lo controla |
| `D5` | GPIO14 | — | — | Libre |
| `D6` | GPIO12 | — | — | Libre |
| `D7` | GPIO13 | — | — | Libre |
| `D8` | GPIO15 | — | — | Libre (con `D8` a GND el arranque falla) |
| `EN` / `RST` | — | — | Botón RST del NodeMCU | Reinicio manual. Útil para volver a tomar el setpoint |

**Solo 4 cables de señal + alimentación.** El resto de pines está libre para
futuras extensiones (ver `docs/developer.md` §7).

> **D1/D2 son los pines I²C por defecto del ESP8266** y coinciden con los del
> ejemplo oficial de la librería Adafruit. Aun así el sketch los declara
> explícitamente en `PIN_SCL`/`PIN_SDA` y los pasa a `Wire.begin(4, 5)` para que
> la intención sea inequívoca.

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
| 1 | `3V3` | `VCC` | 1 | **3.3 V**, no 5 V. Muchos GY-291 llevan regulador LDO onboard y aceptan 5 V, pero alimentarlo a 3.3 V es siempre seguro |
| 2 | `GND` | `GND` | 1 | Masa común, **obligatoria** |
| 3 | `D2` / GPIO4 | `SDA` | 2 | Datos I²C |
| 4 | `D1` / GPIO5 | `SCL` | 2 | Reloj I²C |
| 5 | `GND` | `SDO` | opcional | Define la dirección I²C. **A GND = 0x53 (por defecto).** Si el módulo tiene el jumper de SDO, déjalo como viene de fábrica |
| 6 | `GND` | `CS` | opcional | El breakout GY-291 no usa SPI; a GND selecciona I²C |

### Notas prácticas de cableado

* **Longitudes:** cables de 10–30 cm funcionan sin problema. Más de ~50 cm en un
  bus con mucho ruido eléctrico puede requerir bajar la velocidad I²C
  (`Wire.setClock(100000)`) o añadir pull-ups.
* **Sin resistencias pull-up externas:** el GY-291 ya las incluye (típ.
  4.7 kΩ a 3.3 V). No añadas más salvo que las mediciones sean erráticas.
* **Estabilidad mecánica:** el módulo debe ir **rígidamente atornillado** al
  chasis. Un acelerómetro que vibra sobre un soporte flexible amplifica el
  ruido y genera falsos `*` e `!`.
* **Aislamiento del motor:** aleja el cable del GY-291 de la cableería del
  motor, del alternador y del bus de alimentación de 24 V. Si aparecen
  lecturas erráticas con el motor en marcha, añade un condensador de 100 nF
  entre `VCC` y `GND` **junto al módulo**.
* **Alineación:** el eje que quieras que sea "vertical" (baches/lomos) debe ser
  el que marca el datasheet como Z con el módulo plano. Cualquier rotación
  cambia qué eje reporta cada símbolo, pero la detección sigue siendo válida
  porque el setpoint se toma en la misma posición.

---

## 8. Ejemplos de uso

### 8.1 Arranque normal en reposo

```
=== BusAccel-8266 DIAG CSV (ciclo 16, t,X,Y,Z,setX,setY,setZ,sym) ===
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
20150,0.01,-0.16,1.00,0.01,-0.16,1.00,.
20200,0.02,-0.15,0.99,0.01,-0.16,1.00,.
20250,0.01,-0.17,1.01,0.01,-0.16,1.00,.
20300,0.00,-0.16,1.00,0.01,-0.16,1.00,.
20350,0.01,-0.15,0.99,0.01,-0.16,1.00,.
```

* `Z = 1.00` → el módulo está plano y la gravedad se mide correctamente.
* `Y = -0.16` → leve inclinación lateral permanente (suspensión descentrada).
  **No es un error**: está en el setpoint y se cancela sola.
* Todas las líneas con `sym='.'` → bus parado. Correcto.

> En la app de escritorio (§5.6) este arranque se ve como una curva plana
> de Z en ≈ 1.00 g, atravesada por la línea punteada del setpoint, sin zonas
> coloreadas y sin marcadores.

### 8.2 Recorrido con baches, bordillo y lomo de toro

```
45200,0.01,-0.16,1.00,0.01,-0.16,1.00,.
45250,0.03,-0.15,1.09,0.01,-0.16,1.00,.
45300,0.05,-0.14,1.21,0.01,-0.16,1.00,*      ← lomo de toro: +0.21 ... +0.29 g
45350,0.04,-0.15,1.24,0.01,-0.16,1.00,.
45400,0.02,-0.16,0.98,0.01,-0.16,1.00,.
45450,0.00,-0.17,0.72,0.01,-0.16,1.00,!      ← caída severa: -0.28 g
45500,-0.01,-0.16,0.81,0.01,-0.16,1.00,.
45550,0.00,-0.15,0.95,0.01,-0.16,1.00,.
45600,0.01,-0.16,0.78,0.01,-0.16,1.00,-
45650,0.02,-0.16,1.00,0.01,-0.16,1.00,.
45700,0.01,-0.15,0.99,0.01,-0.16,1.00,.
```

Lectura línea a línea:

| Línea | `sym` | Cálculo | Interpretación |
|---|---|---|---|
| `45250` | `.` | `1.09 − 1.00 = +0.09 g` | Subida aún bajo umbral |
| `45300`–`45350` | `*` | `1.21 − 1.00 = +0.21 g` … `1.24 − 1.00 = +0.24 g` | **Lomo de toro a ~50 km/h**: al cruzar +0.20 g se captura el símbolo y el estado se bloquea; el segundo flanco (`.`) queda **suprimido** |
| `45450` | `!` | `0.72 − 1.00 = −0.28 g` | Entrada a un bache profundo (≤ −0.24 g) |
| `45600` | `-` | `0.78 − 1.00 = −0.22 g` | Transición de rasante, caída leve (entre −0.20 y −0.24 g) |

> **Un símbolo por evento, igual que en el ciclo 15.** El `*` de `45300` es el
> único símbolo de todo el lomo de toro: el flanco de bajada y el rebote
> inercial (~100–200 ms) llegan mientras el estado está `ST_LOCKED` y se
> descartan. El desbloqueo exige 250 ms de lockout **y** 50 ms de reposo
> sostenido dentro de ±0.15 g.

### 8.3 Salida para analizar en un editor

El log es CSV, así que el post-procesamiento es trivial con `awk`, `cut` o
Python — y además **cada línea ya trae los tres ejes y su setpoint**, cosa que
el flujo de caracteres del ciclo 15 no daba:

```bash
# Capturar 5 minutos de recorrido a 20 Hz
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200 | \
  tee recorrido.log > /dev/null

# Contar incidencias por tipo (solo líneas CSV: el 1er campo es un entero)
awk -F, '/^[0-9]+,/ {c[$8]++} END {for (s in c) printf "%s : %d\n", s, c[s]}' recorrido.log
# . : 5942
# + : 37
# * : 11
# - : 29
# ! : 8

# Extraer los eventos con su instante y su desviación real
awk -F, '/^[0-9]+,/ && $8!="." {printf "t=%6d s  sym=%s  dZ=%+.2f g\n",
                               $1/1000, $8, $4-$7}' recorrido.log

# Guardar solo Z y su setpoint para un plot externo
awk -F, '/^[0-9]+,/ {print $1/1000, $4, $7}' recorrido.log > z.dat

# Marcar solo los eventos severos
grep -E ',[0-9.,]*[0-9]+,[*!]$' recorrido.log
```

```python
# conteo_csv.py — analiza el log CSV del ciclo 16
import csv
from collections import Counter

SIMBOLOS = {".", "+", "-", "*", "!"}
contador = Counter()
severos = []

with open("recorrido.log", encoding="utf-8", errors="ignore") as fh:
    for linea in fh:
        partes = linea.strip().split(",")
        if len(partes) != 8:            # banner, diagnóstico, puntos -> fuera
            continue
        try:
            t = int(partes[0]); z = float(partes[3]); sz = float(partes[6])
        except ValueError:
            continue
        sym = partes[7]
        if sym not in SIMBOLOS:         # campo de texto -> fuera
            continue
        contador[sym] += 1
        if sym in "*!":
            severos.append((t / 1000.0, sym, z - sz))

print("reposo      :", contador["."])
print("elev. leve  :", contador["+"])
print("elev. SEVERA:", contador["*"])
print("caída leve  :", contador["-"])
print("caída SEVERA:", contador["!"])

for t, sym, dz in severos[:10]:
    print(f"  t={t:8.2f} s  {sym}  {dz:+.2f} g")

if severos:
    print(f"primer evento severo en t = {severos[0][0]:.2f} s")
```

### 8.4 Ver el recorrido en la app de escritorio

Es la forma recomendada de interpretar un recorrido: gráfico en tiempo real de
Z, zonas de severidad y marcadores con glow por evento (§5.6).

```bash
source venv/bin/activate
python3 app/app.py                 # con la placa
python3 app/app.py --sim           # sin placa, datos sintéticos
```

Alternativamente, con la placa **sobre la mesa** (sin bus), para verificar
que todo funciona antes de la carretera:

| Movimiento | Desviación esperada | Símbolo |
|---|---|---|
| Módulo inmóvil, sin tocar nada | 0 | `.` |
| Golpe brusco hacia abajo | negativa grande | `!` |
| Empujón brusco hacia arriba | positiva grande | `*` |
| Inclinar y volver lentamente | < 0.20 g | `.` |
| Sacudida corta y seca | negativa fuerte | `!` seguido de al menos 5 líneas con `.` |

> **Truco de calibración rápida:** con el bus parado, mira la curva de la app
> (o la columna `sym` del log). Si aparecen `+` o `-` esporádicamente en
> reposo, el bus tiene microvibración (motor, la bomba de aire) y conviene subir
> `ARROW_THRESHOLD_G` a 0.25–0.30 g (ver §10). Recuerda actualizar también
> `LEVE_THRESHOLD_G` en `app/serial_reader.py` para que las zonas de color
> sigan coincidiendo con el firmware.


---

## 9. Compilación y carga

### 9.1 Instalar `arduino-cli`

```bash
# Instalar arduino-cli si no está instalado
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Si no se agrega al PATH automáticamente:
export PATH="$PATH:$HOME/bin"

arduino-cli version
```

### 9.2 Instalar el core de la placa

```bash
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli core list      # verificar que esp8266:esp8266 aparece instalado
```

### 9.3 Instalar las librerías

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

### 9.4 Compilar

```bash
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266.ino
```

O sobre el directorio (el nombre de la carpeta debe coincidir con el del
sketch):

```bash
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 .
```

Opciones útiles:

```bash
# Salida compacta y show warnings
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --warnings all BusAccel-8266.ino

# Compilar y subir en un solo paso
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266.ino
```

### 9.5 Transferir a la placa

```bash
arduino-cli board list                 # localizar el puerto
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266.ino
```

> Ajusta `/dev/ttyUSB0` al puerto real (`/dev/ttyUSB1`, `/dev/ttyACM0`, `COM3` en
> Windows, `/dev/cu.usbserial-*` en macOS). `arduino-cli board list` lo indica.

### 9.6 Monitor serie

```bash
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

> 💡 Para **graficar** en lugar de leer líneas de texto, cierra este monitor
> (el puerto queda ocupado) y lanza la app: `python3 app/app.py` (§5.6).

### 9.7 Si la carga falla

| Síntoma | Causa | Solución |
|---|---|---|
| `Failed to connect to ESP8266: Timed out waiting for packet header` | el gestor de arranque no entra en modo bootloader | Mantén **FLASH** (GPIO0→GND), pulsa **RST**, suelta FLASH a los 2 s |
| `No serial data received` | el driver USB-SD no está instalado | En Linux, usuario en el grupo `dialout`; en macOS, driver CH340/CP2102 |
| `Error: Unknown FQBN` | core no instalado | `arduino-cli core install esp8266:esp8266` |
| `Sketch uses XXX bytes` tras recompilar | normal | Sin ficheros de partición complicados, cabe de sobra en 4 MB |

---

## 10. Calibración

Todas las constantes del algoritmo están **agrupadas en la cabecera del sketch**
(`BusAccel-8266.ino`, sección *Constantes*) y se pueden ajustar sin tocar la
lógica.

### 10.1 Constantes ajustables

| Constante | Valor por defecto | Qué controla | Recomendación de ajuste |
|---|---|---|---|
| `ARROW_THRESHOLD_G` | `0.20f` | Umbral de detección leve (g) | **Si hay falsos positivos en marcha suave con motor en marcha → subir a 0.25–0.30 g.** Si no se detecta nada → bajar a 0.15 g |
| `SEVERE_FACTOR` | `1.2f` | Multiplicador que define la severidad | Bajar a 1.1 → la categoría severa se alcanza antes |
| `SEVERE_THRESHOLD_G` | `ARROW_THRESHOLD_G * 1.2` = `0.24f` | Umbral severo (g) — **derivado, no editar** | Cambiar `SEVERE_FACTOR` o `ARROW_THRESHOLD_G` |
| `RETURN_G` | `0.15f` | Hysteresis de retorno a reposo (g) | Debe quedar **por debajo** de `ARROW_THRESHOLD_G`. Subirlo hace el desbloqueo más rápido pero puede reintroducir rebote |
| `EVENT_LOCKOUT_MS` | `250` | Anulación del rebote inercial (ms) | **No bajar de 250 ms**: el rebote (~200 ms) reaparece como segundo símbolo. Subir a 300–400 si el bus rebota mucho |
| `REST_MS` | `50` | Reposo sostenido exigido (ms) | 50 ms = 5 muestras a 100 Hz. Subir a 80–100 si aún hay rebotes residuales |
| `SETPOINT_N` | `20` | Muestras para el promedio del setpoint | 20 = 200 ms de media. Más → más suave, arranque más lento; menos → más reactivo, más ruido |
| `SAMPLE_INTERVAL_MS` | `10` | Muestreo interno (ms) | Coincide con el data rate del chip (100 Hz). **Bajarlo crea lecturas repetidas**, no más información |
| `PRINT_INTERVAL_MS` | `50` | Impresión (ms) | 50 ms = 20 Hz. Subirlo a 100 alarga el ritmo de lectura; bajarlo aumenta el ruido en la consola |
| `LINE_CHARS` | `50` | ⚠️ **Sin uso en el ciclo 16** | Residuo del ciclo 15 (caracteres por línea). Ya no se consulta: cada tick emite su propia línea CSV completa. No lo edites |
| `LEVE_THRESHOLD_G` / `SEVERO_THRESHOLD_G` (en `app/serial_reader.py`) | `0.20` / `0.24` | Umbrales **visuales** de la app | Sólo afectan a las zonas de color del gráfico. **Mantenerlos sincronizados** con `ARROW_THRESHOLD_G` y `SEVERE_THRESHOLD_G` del firmware |


### 10.2 Procedimiento de calibración de umbrales

1. **Registra la línea base.** Estaciona el bus con el motor en marcha lenta.
   Cuenta cuántos `+`, `-`, `*` o `!` aparecen en 60 s:
   ```bash
   # objetivo: 0 eventos en 60 s
   ```
   Si hay eventos, el umbral está demasiado bajo → sube `ARROW_THRESHOLD_G` en
   pasos de 0.05 g.

2. **Calibra el severo.** Pasa por un lomo de toro a velocidad moderada (~30
   km/h) y a alta (~60 km/h). El primer debe dar `+`; el segundo debe dar `*`.
   Si ambos dan `+`, baja `SEVERE_FACTOR` a 1.1 (o sube `ARROW_THRESHOLD_G`).

3. **Valida los huecos.** Busca un badén profundo conocido. Debe dar `!`.

4. **Verifica la histeresis.** En un tramo recto y suave no debe aparecer ni un
   solo `+` o `-` durante varios segundos seguidos.

5. **Rueda del setpoint.** Al volver a parada tras un recorrido, comprueba que
   `setpoint establecido` es casi idéntico al del arranque. Si ha derivado
   mucho, sube `SETPOINT_N`.

> ⚠️ **El chip lee ±2 g con 10 bits útiles (~3.9 mg/LSB).** Por debajo de ~0.05 g
> de desviación estás dentro del ruido del sensor y del redondeo de `float`.
> No tiene sentido bajar `ARROW_THRESHOLD_G` por debajo de 0.10 g.

---

## 11. Advertencias y precauciones

### 🔴 Electricidad y alimentación

* **Alimentación del GY-291 a 3.3 V**, no a 5 V. Muchos módulos llevan un
  regulador, pero muchos otros (clones) no: alimentarlos a 5 V puede
  **destruir el sensor de forma permanente**.
* **Nunca inviertas la polaridad** de `VCC`/`GND`. Si conectas VCC a GND puedes
  cortocircuitar el regulator del NodeMCU.
* **No confundir `3V3` con `5V`/`VIN`.** `3V3` entrega 3.3 V; `5V`/`VIN` son la
  entrada de alimentación USB de 5 V.
* En un bus real, el sistema de alimentación es de **24 V**: el NodeMCU
  **nunca** se conecta directamente. Se alimenta por USB, por un convertidor
  DC-DC de 24 V a 5 V o por un power bank con aislamiento adecuado.
* El pin `D8` (GPIO15) debe quedar **al aire o a GND**: a HIGH impide el
  arranque del ESP8266.

### 🟡 Instalación en vehículo

* **Fijación rígida.** Un acelerómetro montado sobre material blando o un
  soporte con holgura convierte la vibración del chasis en lecturas falsas
  `*` e `!` constantes. Atorníllalo directamente al chasis.
* **Aislamiento eléctrico.** Separa el cableado del sensor de la línea de
  alimentación de 24 V, del motor y del alternador. Un condensador de 100 nF
  entre `VCC` y `GND`, junto al módulo, elimina gran parte del ruido.
* **El USB no es un puerto de datos y alimentación aislado.** En un vehículo,
  prefiere un power bank **con aislamiento galvojánico** o un convertidor
  automotive. Un cortocircuito en la masa del bus puede dañar el ESP8266.
* **Regulación de tráfico.** El conductor no debe mirar la pantalla ni
  interactuar con el dispositivo mientras conduce. El registro se usa **en
  reposo, al inicio y al final del recorrido**.

### 🟡 Operación

* **No hay guardado de datos en el microcontrolador.** El log solo existe en el
  ordenador que captura la consola serie o en la memoria de la app. Si cierras
  el terminal, se pierde.
* **El flujo de 20 líneas/s llena cualquier terminal.** A ~700 B/s es soportable,
  pero para roadtests largos conviene **guardar a fichero**
  (`| tee recorrido.log`) o usar directamente la **app de escritorio**, que
  mantiene una ventana deslizante en memoria y no inunda la consola.
* **Sólo un programa puede abrir el puerto serie:** no mantengas a la vez
  `arduino-cli monitor` / cutecom y la app. Cierra el monitor antes de abrir
  `python3 app/app.py`.
* **`RST` durante la marcha invalida el setpoint**: tras un reinicio, el
  dispositivo vuelve a pedir 20 lecturas de reposo antes de detectar nada.
  En la app hay que volver a pulsar **Conectar** (o limpiar).

* **Reiniciosperiódicos si el módulo se queda sin alimentación:** al volver, el
  banner y el diagnóstico se repiten desde el principio.

### 🟢 Seguro

* Bajo voltaje (3.3 V / 5 V): riesgo de daño bajo, salvo cortocircuito o
  polaridad invertida.
* La opción de montaje más segura: **placa fuera del bus**, capturing por
  USB, con el GY-291 atornillado al chasis y el cableado del sensor corto.

---

## 12. Estructura del proyecto

```
BusAccel-8266-v2/
├── BusAccel-8266-v2.ino      # El firmware completo — v2 (evento por eje Z + MQTT)
├── README.md                 # Este documento
├── app/                      # Aplicación de escritorio (monitor en tiempo real)
│   ├── app.py                #   Servidor HTTP local + API JSON + ventana pywebview
│   ├── serial_reader.py      #   Hilo de lectura del puerto + parser CSV + simulador
│   ├── index.html            #   Estructura de la interfaz
│   ├── style.css             #   Tema oscuro con neón/glow
│   ├── app.js                #   Gráfico en canvas (sin CDN)
│   ├── requirements.txt      #   pyserial, pywebview, qtpy, PyQt5, PyQtWebEngine
│   └── README.md             #   Documentación específica de la app
├── venv/                     # Entorno virtual de Python de la app (no versionar)
├── docs/
│   ├── datasheet.md          # Guía de usuario: especificaciones, operación, problemas
│   └── developer.md          # Guía de desarrollo: arquitectura, máquina de estados, extensión
├── secrets.h                 # Credenciales reales de WiFi y MQTT — **NO versionar**
├── secrets.example.h         # Plantilla de credenciales (placeholders)
└── .gitignore
```

> ⚠️ **`secrets.h` SÍ se usa en v2**, a diferencia de v1: contiene el SSID del
> WiFi y del broker MQTT. Está en `.gitignore`. Copiá `secrets.example.h` a
> `secrets.h` y rellenalo antes de compilar. Si compila con `MQTT_DRY_RUN 1`
> igual lo necesita, porque el topic que se imprime sale de ahí.

---

## 13. Referencia rápida

| Elemento | Valor |
|---|---|
| **FQBN** | `esp8266:esp8266:nodemcuv2` |
| **Puerto serie** | 115200 baud, 8N1 |
| **Formato de salida** | ensayo seco: `[MQTT-OUT] topic` + payload JSON, y `EV,…` con las 200 muestras. Con `MODE_SERIAL_CSV 1`: `t,X,Y,Z,setX,setY,setZ,sym` |
| **Monitor de escritorio** | `python3 app/app.py` (ver §5.6) |
| **Direcciones I²C** | ADXL345 @ `0x53`, bus I²C estándar |
| **Pines** | `D2`/GPIO4 = SDA, `D1`/GPIO5 = SCL, `3V3`, `GND` |
| **Librerías** | `Adafruit ADXL345` 1.3.x, `Adafruit Unified Sensor` 1.1.x, `Wire` (core) |
| **Muestreo** | 100 Hz (10 ms) |
| **Impresión** | 20 Hz (50 ms) — 1 línea CSV por tick |
| **Umbral leve** | 0.20 g |
| **Umbral severo** | 0.24 g (1.2×) |
| **Histeresis de retorno** | 0.15 g |
| **Setpoint** | 20 muestras (200 ms), recalculado en silencio cada 20 muestras en reposo |
| **Símbolos** | `.` reposo, `+`/`*` elevación leve/severa, `-`/`!` caída leve/severa |
| **Detección** | **solo eje Z**, en los dos sentidos |
| **Ventana por evento** | 200 muestras (100 antes + 100 después) a 100 Hz = 2 s |
| **Mensajes por evento** | 6 → 1 `summary` + 5 `chunk` de 40 muestras |
| **Publicación MQTT real** | 🟡 desactivada: `MQTT_DRY_RUN 1` (ensayo en seco por serie) |
| **Persistencia** | contador `id` en RTC user memory (no toca la flash) |
| **EEPROM** | ❌ no se usa, no hace falta (§1.1) |
| **Cola con broker caído** | ❌ el evento se descarta y se sigue midiendo |
| **Fase actual** | v2: detección por evento en Z, MQTT en ensayo en seco |
| **Pendiente** | Credenciales válidas del broker → `MQTT_DRY_RUN 0`, y hora de reloj (NTP) como campo `ep` |

---

## 14. Licencia y notas finales

Proyecto de uso educativo / de investigación aplicada. El código es
proporcionado "tal cual", sin garantía de Healthcare apta para uso en
vehículos reales. **Los umbrales de detección deben recalibrarse con datos del
recorrido antes de cualquier uso operacional.**

Toda la documentación de este proyecto (este README, `docs/datasheet.md`,
`docs/developer.md` y `app/README.md`) refleja la **v2: detección de eventos
solo sobre el eje Z con ventana de 200 muestras**, el contrato MQTT definido en
§5.7, y la **aplicación de escritorio** que consume el CSV del modo serie puro.

