# BusAccel-8266-v2 — Hoja de datos y guía de usuario

> Acelerómetro ADXL345 sobre NodeMCU ESP8266 instalado en un bus urbano.
> **v2**: detección por eventos sobre el eje Z y publicación MQTT por evento.
> Firmware `FW_VERSION "2"` · build `2026-09-30`.

---

## 1. Descripción general

### 1.1 Qué es

Un módulo que se fija en el interior de un bus y mide la **aceleración vertical**
sobre su eje Z a 100 Hz. Cuando esa aceleración se sale de una banda de reposo
—porque el bus frena, arranca, entra en un bache o da un golpecito— el firmware
**no manda una lectura aislada: captura una ventana completa de 2 segundos a
ambos lados del incidente** y la publica como un evento.

El objetivo es poder **reconstruir después el comportamiento del bus durante una
jornada de 11 horas**, sin llenar la memoria de la placa ni saturar el broker con
un flujo continuo de datos.

### 1.2 Qué NO hace (aclaración importante)

- **No es un detector de eventos de tránsito ni un censor de seguridad.** Los
  umbrales son un criterio de *registro de eventos*, no una norma. Nada de lo que
  mida está homologado para uso normativo.
- **No mide velocidad, aceleración longitudinal ni giros.** X e Y se leen y se
  transmiten, pero **no disparan eventos**: solo Z decide (ver §2.4).
- **No guarda nada de forma permanente.** Si el broker está caído durante un
  evento, **el evento se descarta** (política deliberada, §2.7).
- **No tiene reloj de reloj.** Los campos `up` / `up0` son milisegundos desde el
  arranque de la placa, **no la hora del día**. Ver §6.3.
- **No envía nada mientras el bus va tranquilo.** El silencio es el caso normal.

### 1.3 Diferencias con la versión 1

| | v1 | v2 (este proyecto) |
|---|---|---|
| Criterio de detección | eje de mayor desviación entre X/Y/Z | **solo eje Z** (`devZ = gz − spZ`) |
| Qué se transmite | línea CSV cada 50 ms, siempre | **solo cuando hay evento** |
| Red | ninguna (eliminada a propósito) | MQTT conBroker, apagable |
| Formato de evento | 1 muestra aislada | **200 muestras** (100 antes + 100 después) |
| Persistencia | — | contador de eventos en RTC (16 B) |

---

## 2. Especificaciones técnicas

### 2.1 Placa — NodeMCU ESP8266 (ESP-12E)

| Parámetro | Valor |
|---|---|
| Microcontrolador | ESP8266 @ 80 MHz (Xtensa LX106) |
| Flash | 1 MB |
| RAM | 80 KB usable |
| Alimentación | 5 V por USB o VIN; 3.3 V para la lógica |
| Bus | I²C a 100 kHz |
| Baud del monitor | 115200 |

### 2.2 Sensor — GY-291 / ADXL345

Es un acelerómetro **MEMS** de Analog Devices: un(os) masa(s) interna(s)
suspendida(s) en molas micromecánicas. La aceleración mueve la masa y cambia la
tensión de los puentes de medición; un convertidor convierte esa diferencia en
un número.

| Parámetro | Valor |
|---|---|
| Ejes | 3 (X, Y, Z) |
| Rango configurable | ±2, ±4, ±8, ±16 g — **aquí ±2 g** (`SENSOR_RANGE = ADXL345_RANGE_2_G`) |
| Resolución de salida | 13 bits con signo (10 bits de magnitud) |
| Sensibilidad a ±2 g | **256 LSB/g** |
| Rango de lectura en bruto | −32768 … +32767 |
| Tasa de datos | 10 … 3200 Hz — **aquí 100 Hz** (`SENSOR_DATARATE_100_HZ`) |
| Dirección I²C | **0x53** (`ADXL345_I2C_ADDR`) |
| Identificador | `DEVID` = **0xE5** |
| Alimentación | 2.0 V … 3.6 V (el GY-291 ya trae regulador a 3.3 V) |
| Consumo típico | ~100 µA en medición |

**Conversión a g.** Los bytes que llegan del sensor son un entero de 16 bits con
signo, en complemento a dos, con 256 cuentas por g. Para obtener gramos:

```
g = valor_bruto / 256.0
```

Ejemplo: un reposo en Z de 235 cuentas son `235 / 256 = 0.918 g` ≈ 1 g con el
sensor ligeramente inclinado, que es lo normal en un bus con pendiente leve.

### 2.3 Parámetros del algoritmo de detección

Todos son `#define` en `BusAccel-8266-v2.ino` y se pueden ajustar:

| Constante | Valor | Significado |
|---|---|---|
| `SETPOINT_N` | 20 | lecturas para promediar el setpoint (≈0.2 s a 100 Hz) |
| `ARROW_THRESHOLD_G` | 0.20 | desviación de Z que cuenta como evento |
| `SEVERE_FACTOR` | 1.2 | multiplicador para el umbral severo |
| `SEVERE_THRESHOLD_G` | 0.24 | = 0.20 × 1.2, calculado por el propio firmware |
| `RETURN_G` | 0.15 | histéresis: hay que bajar de aquí para volver a reposo |
| `EVENT_LOCKOUT_MS` | 250 | tiempo mínimo bloqueado tras disparar (anti-rebote) |
| `REST_MS` | 50 | reposo sostenido exigido para desbloquear |
| `PRE_N` | 100 | muestras **antes** del cruce (= 1 s) |
| `POST_N` | 100 | muestras **después** del cruce (= 1 s) |
| `EVENT_N` | 200 | tamaño total de la ventana (= `PRE_N + POST_N`) |
| `EVENT_SPAN_MS` | 2000 | duración de la ventana en ms |
| `CHUNK_N` | 40 | muestras por publicación |
| `CHUNKS` | 5 | = `EVENT_N / CHUNK_N` |
| `SAMPLE_INTERVAL_MS` | 10 | muestreo interno ≈ 100 Hz |

**Por qué la histéresis.** Sin ella, una muestra que cruza el umbral y la
siguiente que vuelve atrás generaría dos eventos por el mismo bache. Con
`RETURN_G = 0.15 < ARROW_THRESHOLD_G = 0.20` hace falta bajar claramente por
debajo del umbral para considerar que el bus volvió a la calma.

### 2.4 Los símbolos

`devZ = gz − spZ` es la desviación del eje Z respecto del setpoint. **Es el único
valor que decide.** El símbolo se elige así:

| Desviación `devZ` | Símbolo | Significado |
|---|---|---|
| `devZ ≥ +0.24` | `*` | **elevación severa** (frenazo/brusco adelante) |
| `devZ > +0.20` | `+` | elevación leve |
| `devZ ≤ −0.24` | `!` | **caída severa** (bache, frenada brusca) |
| `devZ < −0.20` | `-` | caída leve |
| `|devZ| ≤ 0.20` | `.` | reposo |

Los umbrales son simétricos: el mismo magnitud en positivo y negativo se
clasifica igual.

### 2.5 Formato de la salida por serie

Hay **tres** formas de línea, según el modo de compilación.

#### 2.5.1 Modo evento MQTT + ensayo en seco (estado actual)

Con `MQTT_DRY_RUN 1` la placa **no abre WiFi ni MQTT**: imprime por el monitor
serie exactamente lo que enviaría, con el topic y el número de bytes.

```
=== BusAccel-8266-v2 (evento eje Z, ENSAYO EN SECO) ===
Sin WiFi y sin MQTT: cada mensaje se imprime por serie.
[FW] version 2
[FW] build 2026-09-30
[RTC] contador de eventos: 0
[ADXL345] OK (begin() correcto)
... diagnóstico de registros ...
setpoint establecido: X=0.12 Y=0.13 Z=0.92
[MQTT-OUT] --> 3f7c1e90b2a45d68/accel/setpoint (28 B)
{"x":0.12,"y":0.13,"z":0.92}
```

Y al mover o golpear el sensor:

```
[MQTT-OUT] --> 3f7c1e90b2a45d68/accel/event/summary (NN B)
{"id":1,"sym":"!","spz":0.92,"dev":-0.24,"peak":1.03,"n":200,"ms":2000,"up":37795,"count":1}
[MQTT-OUT] --> 3f7c1e90b2a45d68/accel/event/chunk (NN B)
{"id":1,"i":0,"n":5,"up0":37795,"z":[235,234,...],"x":[30,31,...],"y":[33,32,...],"dt":[10,10,...]}
... (5 chunks) ...
[EVENTO] id=1 sym=! spz=0.92g dev=-0.24g pico=1.03g ventana=2000ms (pre=100 post=100)
EV,37795,30,33,235,10,.
EV,37795,30,33,235,10,.
... (200 líneas) ...
[MQTT-OUT] === evento 1: 6 mensajes, 617 B, 2ms (ENSAYO EN SECO: nada enviado al broker) ===
```

#### 2.5.2 El volcado `EV,…` (formato B)

| Campo | Tipo | Significado |
|---|---|---|
| `EV` | — | marca de que la línea pertenece a una ventana de evento |
| `t0` | uint32 | **inicio de la ventana**, en ms de uptime. **Se repite en las 200 líneas** |
| `Xraw` | int16 | eje X en **cuentas crudas** del ADXL345 (÷256 = g) |
| `Yraw` | int16 | eje Y en cuentas crudas |
| `Zraw` | int16 | eje Z en cuentas crudas |
| `dt` | uint16 | milisegundos **reales** entre esta muestra y la anterior |
| `sym` | char | el símbolo va **solo en la muestra que disparó** (índice `PRE_N` = 100) |

Como `t0` se repite, el tiempo real de cada muestra se reconstruye sumando los
`dt` hacia adelante desde `t0`. El campo `dt` existe precisamente porque la
transmisión por WiFi introduce latencia: sin él, el gráfico del cliente no
reflejaría la duración real del evento.

#### 2.5.3 Modo serie puro (`MODE_SERIAL_CSV 1`)

Sin WiFi, una línea CSV cada 50 ms, **formato idéntico a v1**:

```
t,X,Y,Z,setX,setY,setZ,sym
```

`123456,0.14,0.28,0.88,0.14,0.28,0.88,.`

### 2.6 Persistencia

- **No se usa EEPROM.** El EEPROM emulado del ESP8266 es **un solo sector de 4096
  bytes sin wear leveling**: cada `commit()` borra el sector entero y lo
  reescribe. Un commit por segundo destruye la flash en unas 11 horas.
- Solo se usa la **memoria de usuario del RTC**, 16 bytes = 4 palabras, que **no
  toca la flash** y sobrevive a los reinicios por corte de red:
  - palabras 0–1 → marca y flag de reinicio por fallo
  - palabra 2 → contador de eventos
  - palabra 3 → `RTC_EVT_MAGIC` = `0x45565431` (los caracteres `EVT1`)
- Al arrancar, el contador se valida contra la marca; si no cuadra o es absurdo
  (>100000000) se pone a cero y se avisa por serie.

### 2.7 Qué pasa si el broker está caído

Si en el momento de cerrar una ventana no hay conexión MQTT, el evento **se
descarta** y se avisa:

```
[MQTT] sin broker: evento descartado (politica v2)
```

**No hay cola.** Es una decisión deliberada: una cola en RAM obliga a decidir
cuántos eventos aguantar y qué hacer cuando se llena, y en un bus los eventos
con 10 minutos de retraso tienen menos valor que los de ahora. La placa **sigue
midiendo** sin problema y el siguiente evento se publica con normalidad.

---

## 3. Instalación

### 3.1 Requisitos

- NodeMCU ESP8266 (o cualquier placa ESP8266 con 4 MB de flash)
- Módulo GY-291 con ADXL345
- Cable USB de datos
- `arduino-cli` con el core `esp8266:esp8266` instalado
- Un **ClientId MQTT distinto por placa** (ver §3.5)

### 3.2 Conexión del hardware

```
   NodeMCU ESP8266                GY-291 / ADXL345
  ┌────────────────┐            ┌──────────────────┐
  │            3V3 ├────────────┤ VCC              │
  │             GND├────────────┤ GND              │
  │                │            │                  │
  │          D2/GPIO4├────────────┤ SDA  ───────────┼──┐
  │          D1/GPIO5├────────────┤ SCL  ───────────┼──┤
  │                │            │                  │  │ 4k7Ω a 3V3
  └────────────────┘            └──────────────────┘  │ (opcional)
                                                     └──┴
```

**Tabla de pines**

| Señal | NodeMCU | GPIO | Modulo | Nota |
|---|---|---|---|---|
| I²C SDA | **D2** | **4** | SDA | `#define PIN_SDA 4` (línea 159) |
| I²C SCL | **D1** | **5** | SCL | `#define PIN_SCL 5` (línea 160) |
| Alimentación | 3V3 / VIN | — | VCC | 3.3 V; el GY-291 trae regulador |
| Masa | GND | — | GND | común con el GY-291 |
| LED | D4 | 2 | onboard | no lo usa el firmware |
| Buzzer | D8 | 16 | onboard | no lo usa el firmware |

**El LED y el buzzer son opcionales.** El firmware no los controla
(`blinkLedDummy()` está vacío, línea 517) porque en bus los LEDs y el zumbador
integrados distraen al conductor.

**Sobre las resistencias pull-up.** El GY-291 ya las trae, pero si la lectura sale
errática o el bus es largo, añadir 4.7 kΩ de SDA y de SCL a 3.3 V ayuda mucho.
Conectar SDA y SCL a 5 V **no** funciona: el ADXL345 es de 3.3 V.

### 3.3 Compilar y cargar

```bash
# 1) Compilar
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266-v2

# 2) Localizar el puerto (NUNCA hardcodearlo)
arduino-cli board list

# 3) Transferir a la placa
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266-v2

# 4) Monitor serie
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

Cierra cualquier otra aplicación que tenga el puerto abierto (CuTTeM, minicom,
el monitor de la app de escritorio): si no, la carga falla con
`[Errno 16] Device or resource busy`.

### 3.4 Qué mirar en el arranque

Una secuencia correcta se ve así:

```
=== BusAccel-8266-v2 (evento eje Z, ENSAYO EN SECO) ===
[FW] version 2
[RTC] contador de eventos: 0
[ADXL345] OK (begin() correcto)
[ADXL345] chip DEVID = 0xE5 (correcto)
... POWER_CTL = 0x08 (medición activa) ...
setpoint establecido: X=0.12 Y=0.13 Z=0.92
Umbral Z: leve 0.20g | severo 0.24g | retorno 0.15g
Ventana: 100 antes + 100 despues = 200 muestras ~ 2000ms | 6 publishes
Adquiriendo setpoint (20 lecturas en reposo)...
```

Checklist:

| Lo que ves | Significado |
|---|---|
| `DEVID = 0xE5` | el chip es un ADXL345 |
| `POWER_CTL` con bit 3 activo (0x08) | está midiendo, no en reposo profundo |
| `setpoint establecido` | Las 20 muestras de calibración salieron |
| `NO DETECTADO` | revisar VCC/GND/SDA/SCL; la placa reintenta cada segundo solo |

### 3.5 Credenciales (`secrets.h`)

`secrets.h` **está en `.gitignore`** y no debe subirse nunca. Se copia de
`secrets.example.h`:

| Clave | Qué es |
|---|---|
| `WIFI_SSID` / `WIFI_PASSWORD` | red a la que se conecta la placa |
| `AP_SSID` / `AP_PASSWORD` | AP de respaldo (reservado para v3) |
| `MQTT_BROKER` / `MQTT_PORT` | dirección del broker |
| `MQTT_USERNAME` / `MQTT_PASSWORD` | credenciales MQTT |
| `MQTT_CLIENT` | **ClientId: debe ser ÚNICO por placa** |

> ⚠️ **`MQTT_CLIENT` tiene que ser distinto en cada placa que use el mismo
> broker.** El protocolo MQTT identifica a cada cliente por su ClientId, y si dos
> placas se conectan con el mismo, el broker **expulsa a una de las dos** de forma
> continua. Cada placa tiene que tener el suyo propio y reconocible.

### 3.6 Aplicación de escritorio

Está en `app/` y requiere Python 3 con `pyserial` y `pywebview`. Ver §5.

---

## 4. Operación

### 4.1 Procedimiento normal

1. Alimentar la placa por USB y abrir el monitor a **115200 baud**.
2. Esperar a `setpoint establecido` — **el vehicle tiene que estar quieto y plano
   durante esos 20 milisegundos** de calibración, o el setpoint queda mal.
3. Colocar el módulo en su posición definitiva en el bus.
4. Montar y probar con la app de escritorio (§5).
5. Cerrar todo y dejar que corra la jornada.

### 4.2 Interpretar el setpoint

El setpoint es el promedio de 20 lecturas taken en reposo. **No debería ser
exactamente 1.00 g**: si el bus tiene las plazas inclinadas, o el módulo está
montado en ángulo, el eje Z verá una componente constante distinta de 1 g. En un
montaje típico se han visto valores como **Z = 0.92 g**.

Eso **no es un error**: es la posición real de montaje. Lo que importa es la
desviación *respecto de ese valor*, y por eso el firmware siempre compara
`gz − spZ` y no `gz` contra la gravedad.

Para recalibrar sin reiniciar, con la placa conectada al broker:

```
<MQTT_CLIENT>/cmd/RECAL
```

`mqttCallback()` (línea 745) registra el comando y pone `setpointReady = false`,
de modo que las siguientes 20 muestras en reposo recalibran solas.

### 4.3 Interpretar la salida

En la línea `[EVENTO]` del resumen:

| Campo | Qué te dice |
|---|---|
| `id` | número de evento desde el arranque; **monótono**, es el índice natural |
| `sym` | símbolo del cruce |
| `spz` | el setpoint de Z en ese momento |
| `dev` | desviación **en el instante del cruce** (con signo) |
| `pico` | **máxima** desviación absoluta dentro de toda la ventana |
| `pico` vs `dev` | si difieren mucho, el golpe tuvo un pico posterior a la deteccion |

`pico` se mide sobre las 200 muestras completas, no solo sobre el cruce: es el
excursion máximo real.

### 4.4 Ejemplos reales de uso

**Caso 1 — frenazo brusco.** El bus frena; el cuerpo del conductor y el módulo se
inclinan hacia delante. `devZ` se hace negativo con magnitud ≥ 0.24 → símbolo
`!`. En la ventana de 2 s se ve el frente de la aceleración, el máximo y la
vuelta a la normalidad.

**Caso 2 — bache en una calle en mal estado.** Serie de oscilaciones, varias
ventanas consecutivas, alternando `!` y `-` según el primer cruce de cada ventana.

**Caso 3 — arranque suave.** Desviación positiva pequeña, símbolo `+`, por debajo
del severo. Un evento "menor" que queda registrado igual, porque el objetivo es
reconstruir la jornada completa, no solo lo grave.

**Caso 4 — módulo mal orientado.** Si el bus tiene un tramo con pendiente
pronunciada y el setpoint no se recalibró, todas las ventanascile will show the
same sign. La solución es un `RECAL` o dejar la placaquieta 20 ms antes de
arrancar.

### 4.5 Cómo extraer los datos para analizarlos

La forma más cómoda es la **app de escritorio** (§5), que dibuja el golpe y
guarda un log de la sesión. Si prefieres procesar con tus propias herramientas,
coge el volcado `EV,…` de la serie (200 líneas por evento) o, cuando el broker
esté activo, suscríbete a los topics de §6.

---

## 5. La aplicación de escritorio

### 5.1 Qué es

Una app local en Python: lee el puerto serie, dibuja el eje Z en tiempo real con
un canvas, y muestra los eventos detectados. La interfaz se abre en una ventana
nativa (pywebview) o, si eso no está disponible, en el navegador.

### 5.2 Arrancar

```bash
cd BusAccel-8266-v2/app
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt          # pyserial, pywebview

# contra la placa real
python3 app.py --port /dev/ttyUSB0 --baud 115200

# sin placa, con datos sintéticos en formato v2
python3 app.py --sim

# reviviendo una captura guardada, más rápido o más lento que el original
python3 app.py --replay captura.log --speed 4
```

| Argumento | Qué hace |
|---|---|
| `--port` | puerto serie; `SIM` arranca el simulador |
| `--baud` | velocidad del puerto (115200 por defecto) |
| `--sim` | arranca en modo simulador |
| `--replay FICHERO` | reproduce un `.log` en vez de leer el puerto |
| `--speed` | multiplicador de velocidad del replay (0.1 … 50) |
| `--browser` | fuerza el navegador en vez de pywebview |
| `--http-port` | puerto del servidor local (8765 por defecto) |
| `--debug` | abre las devtools de pywebview |

### 5.3 Las tres fuentes de datos

1. **Puerto serie real** — la placa conectada.
2. **Simulador** (`--sim`) — genera ventanas de evento v2 sintéticas, útiles para
   probar la interfaz sin placa.
3. **Captura** (`--replay`) — reproduce un log de CuTTeM. El botón **"Captura…"**
   de la barra superior abre un diálogo para elegir el fichero.

### 5.4 Exportar una captura de CuTTeM

Para guardar una sesión y verla después:

1. En CuTTeM, **Recording → Start** (o el icono de grabar).
2. Reproduce la sesión que te interese.
3. **Recording → Stop** y guarda el `.log`.
4. Ábrelo con el botón **"Captura…"** o con `--replay`.

El replay usa las marcas de tiempo del propio log (`[T+1:02.345]`,
`01:02:03.456`, `[62.345s]`) para reproducir el ritmo real, así que un golpe se ve
con la duración correcta aunque el fichero tenga una línea de ruido entre medias.

### 5.5 Qué se ve

- **Gráfico del eje Z** en g, con el setpoint punteado y las zonas de umbral
  sombreadas (leve en ámbar, severo en rojo, arriba y abajo).
- **Botón de última detección** con el símbolo y su significado.
- **Estadísticas**: Hz, número de muestras, tiempo y setpoint Z.
- **Contadores** de cada símbolo.
- **Últimas detecciones**: los cruces agrupados por renglón.
- **Eventos detectados**: un renglón por evento con su `id`, su `up` y su pico.

Los datos se guardan **por tiempo** (los últimos 300 s, con un suelo de 4000
puntos), no por número de muestras, precisamente porque en v2 no llega nada
durante el reposo y un buffer contado en muestras perdería eventos.

---

## 6. Contrato MQTT

*(Solo activo cuando `MQTT_DRY_RUN = 0` y el broker acepta las credenciales.)*

### 6.1 Topics

Todos bajo el ClientId de la placa:

| Topic | Frecuencia | Contenido |
|---|---|---|
| `<client>/accel/online` | al conectar | mensaje de presencia |
| `<client>/accel/status` | cada 15 min, y al cambiar | estado, setpoint, contadores |
| `<client>/accel/setpoint` | al calibrar | `{"x":…,"y":…,"z":…}` |
| `<client>/accel/event/summary` | 1 por evento | ficha del evento |
| `<client>/accel/event/chunk` | 5 por evento | las 200 muestras en 5 trozos |

Además la placa **se suscribe a `<client>/#`**, así que recibe cualquier topic por
debajo de su ClientId. El único comando implementado es **`RECAL`**: publica
`RECAL` en cualquier topic bajo `<client>/` (por ejemplo `<client>/cmd/RECAL`) y
la placa recalibra sin reiniciar. `mqttCallback()` ignora el topic y solo mira el
payload.

### 6.2 Un evento completo

Un evento son **6 mensajes**: 1 resumen + 5 chunks de 40 muestras. La razón del
troceado es el tamaño máximo de un topic QoS 0: el chunk más largo en el peor
caso medido ocupa **902 bytes**, y partirlo mantiene cada mensaje holgado y evita
que uno solo falle y se pierda la ventana entera.

#### Resumen

```json
{
  "id": 1,
  "sym": "!",
  "spz": 0.92,
  "dev": -0.24,
  "peak": 1.03,
  "n": 200,
  "ms": 2000,
  "up": 37795,
  "count": 1
}
```

| Campo | Tipo | Significado |
|---|---|---|
| `id` | uint32 | identificador del evento, **monótono** |
| `sym` | char | símbolo del cruce (`*` `+` `!` `-`) |
| `spz` | float | setpoint de Z en ese momento, en g |
| `dev` | float | desviación de Z **en el instante del cruce**, en g, con signo |
| `peak` | float | desviación **máxima** de toda la ventana, en g, sin signo |
| `n` | uint16 | muestras en la ventana (siempre 200) |
| `ms` | uint16 | duración de la ventana (siempre 2000) |
| `up` | uint32 | **uptime** del inicio de la ventana, en ms |
| `count` | uint32 | mismo valor que `id`; se conserva por compatibilidad |

#### Chunk

```json
{
  "id": 1,
  "i": 0,
  "n": 5,
  "up0": 37795,
  "z": [235, 234, 236, 501, 184, ...],
  "x": [30, 31, 30, 29, 31, ...],
  "y": [33, 32, 33, 34, 32, ...],
  "dt": [10, 10, 10, 11, 10, ...]
}
```

| Campo | Tipo | Significado |
|---|---|---|
| `id` | uint32 | **el mismo** `id` del resumen: es lo que reensambla la ventana |
| `i` | uint8 | índice de este chunk, de 0 a `n−1` |
| `n` | uint8 | número total de chunks del evento (5) |
| `up0` | uint32 | **uptime** del inicio de la ventana |
| `z` | int16[] | eje Z, **40 cuentas crudas** (÷256 = g) |
| `x` | int16[] | ídem eje X |
| `y` | int16[] | ídem eje Y |
| `dt` | uint16[] | ms reales entre muestras consecutivas |

**Por qué los valores van crudos y no en coma flotante.** `0.92` ocupa 4
caracteres; `235` ocupa 3. Con 40 valores por campo y 4 campos, son cientos de
bytes de diferencia por evento, y **no se pierde nada**: 235 es exactamente la
resolución nativa del sensor. El cliente divide por 256.

**Por qué el orden es `z, x, y, dt`.** El macro `APPEND_FIELD` (línea 558) escribe
la coma inicial y **cierra su propio array**. Así el orden de los campos es
libre y no hace falta llevar la cuenta de cuál va primero. El orden es fijo solo
para que los clientes puedan confiar en él.

### 6.3 ⚠️ `up` y `up0` NO son hora de reloj

Son **milisegundos transcurridos desde que arrancó la placa**. La placa **no
sincroniza ningún reloj**: no hay NTP, ni RTC externo, ni `configTime()`.

Consecuencia práctica: **el `up` de una placa no es comparable con el de otra**, y
tampoco sirve para saber a qué hora del día pasó el evento. Para la jornada de
11 horas hay dos caminos:

1. **Con el broker funcionando**, añadir un campo `ep` con la hora epoch obtenida
   por NTP al conectar. Es un pendiente conocido del firmware.
2. **Mientras tanto**, usar `id` como eje temporal (es monótono y sin huecos) y el
   instante de **recepción** del mensaje como ancla. Todos los chunks de un mismo
   evento llegan con menos de un segundo de diferencia, así que la ventana se
   puede colocar en la línea de tiempo con esa precisión.

### 6.4 Reensamblar un evento

```python
import json, time
import paho.mqtt.client as mqtt

CHUNKS = 5
CHUNK_N = 40

ventanas = {}

def on_connect(c, u, f, rc):
    c.subscribe(f"{CLIENT}/accel/event/#")

def on_message(c, u, msg):
    d = json.loads(msg.payload)
    topic = msg.topic.rsplit("/", 1)[-1]
    if topic == "summary":
        ventanas[d["id"]] = {"meta": d, "t0": None, "muestras": []}
        return
    if topic != "chunk":
        return
    v = ventanas.setdefault(d["id"], {"meta": None, "t0": None, "muestras": []})
    if v["t0"] is None:
        v["t0"] = d["up0"]           # inicio real de la ventana
    # z, x, y y dt vienen alineados muestra a muestra
    for k in range(len(d["z"])):
        v["muestras"].append({
            "t": v["t0"] + sum(d["dt"][:k]),   # tiempo real reconstruido
            "z": d["z"][k] / 256.0,
            "x": d["x"][k] / 256.0,
            "y": d["y"][k] / 256.0,
        })

mqtt.Client(CLIENT).on_connect = on_connect
mqtt.Client(CLIENT).on_message = on_message
# ... connect, loop_forever
```

Cuando un evento tenga sus 5 chunks, quedan 200 puntos listos para dibujar, cada
uno con su tiempo real.

---

## 7. Calibración y mantenimiento

### 7.1 Ajustar los umbrales

Edita `ARROW_THRESHOLD_G` (línea 112) y recompila. El severo sale de ahí
automáticamente (`SEVERE_THRESHOLD_G = ARROW_THRESHOLD_G * SEVERE_FACTOR`), así
que si quieres separarlos más sube también `SEVERE_FACTOR`.

Cuanto más bajo el umbral, más eventos y más tráfico. Con 0.20 g sobre un bus en
marcha se obtienen eventos con mucha frecuencia.

### 7.2 Ajustar la ventana

`PRE_N` y `POST_N` (líneas 125-126) controlan cuánto se guarda a cada lado. Juntos
determinan `EVENT_N`, y `CHUNKS` debe seguir siendo entero:

```
EVENT_N = PRE_N + POST_N
CHUNKS  = EVENT_N / CHUNK_N
```

Duplicar la ventana duplica la RAM del buffer (`sample_t` son 8 bytes, así que
200 muestras son 1600 bytes).

---

## 8. Resolución de problemas

| Síntoma | Causa probable | Solución |
|---|---|---|
| `NO DETECTADO` | cableado | revisar VCC/GND/SDA/SCL; el módulo necesita 3.3 V, no 5 V |
| `DEVID` ≠ `0xE5` | no es un ADXL345, o falla I²C | verificar módulo y pull-ups |
| No aparece ningún evento | el bus va realmente plano | normal; golpea suavemente el módulo para probar |
| Eventos con el mismo símbolo duplicados | el umbral roza el valor en reposo | subir `ARROW_THRESHOLD_G` o bajar `SETPOINT_N` |
| El setpoint es raro | la placa se movió al calibrar | dejar quieta 20 ms antes de que arranque la captura |
| `[Errno 16] Device or resource busy` | otro programa tiene el puerto | cerrar CuTTeM / minicom / la app |
| El broker no acepta | credenciales o ClientId | probar con `mosquitto_sub` desde el PC |
| Lecturas erráticas | cable I²C largo | pull-ups de 4.7 kΩ a 3.3 V, acortar el cable |

---

## 9. Advertencias

> **Este equipo no es un instrumento de medición homologado.** Los umbrales son un
> criterio de registro, no una norma de seguridad. Nada de lo que mida debe usarse
> para evaluar a un conductor, para fiscalizar el servicio, ni como evidencia.

- **No lo uses como elemento de seguridad.** No es un sensor de airbag ni un
  detector homologado.
- **Borrar `secrets.h` antes de publicar el proyecto.** Contiene la contraseña del
  WiFi y del broker. Está en `.gitignore` a propósito.
- **El ClientId debe ser único por placa**, o expulsarás a otra placa del mismo
  broker.
- **No uses el EEPROM.** No está en el firmware, y no debería añadirse: es un solo
  sector de 4096 bytes sin wear leveling.
- **Al instalar en un bus, fija el módulo con seguridad.** Un objeto suelto en el
  habitáculo es un riesgo para todos los ocupantes.
- **Los datos son Confidential Information de tu operación de transporte.** Cuida
  el log antes de compartirlo.
- **Verifica el consumo en el bus.** El GY-291 alimentado por VIN del USB consume
  del USB; si lo alimentas del bus, añade un regulador y un fuse.
