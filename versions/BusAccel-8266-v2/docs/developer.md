# BusAccel-8266-v2 — Documentación del desarrollador

Explicación del código fuente de `BusAccel-8266-v2.ino` (~1040 líneas) y de la
app de escritorio del directorio `app/`.

Las referencias `archivo:línea` son de la versión con `FW_BUILD 2026-09-30`.

---

## 1. Por qué existe la v2

La v1 (`../BusAccel-8266`) emitía **una línea CSV cada 50 ms, para siempre**, por
consola, y su WiFi estaba desactivado a propósito porque la reconexión «contaminaba
las mediciones». Eso servía para ver el bus en el escritorio, pero no para una
jornada de 11 horas: 20 muestras por segundo son 792000 líneas por jornada.

La v2 invierte el planteamiento:

| Decisión | Motivo |
|---|---|
| **Detectar por evento, no transmitir siempre** | durante un bus en marcha normal la aceleración vertical es plana; lo interesante son los instantes en que no lo es |
| **Solo el eje Z decide** | X e Y son ruido de vibración del chasis y del motor; al decidir con ellos la v1 generaba falsos positivos |
| **Ventana de 2 s alrededor del cruce** | un golpe no dura una sola muestra: dura decenas de milisegundos. Mandar solo el cruce perdía la forma |
| **Reintroducir el WiFi** | había que poder entregar los eventos a un servidor, no solo verlos por cable |

### 1.1 Nota sobre el muestreo

El planteamiento original era «1 lectura por segundo, guardar 30 lecturas, mandarlas
en lote cada 30 s». **Eso no funciona**, por el criterio de Nyquist: la vibración
de un bus tiene componentes entre 10 y 1000 Hz, y muestrear a 1 Hz las
destruye por completo. Lo que sí sirve —y es lo que hace v2— es **muestrear a
100 Hz** y publicar solo cuando algo noteworthy ocurre.

---

## 2. Estructura del sketch

```
líneas   1- 99   includes, modo de compilación, Arduino.h
líneas 101-172   constantes (agrupadas por dominio)
líneas 174-252   tipos, estado global, prototipos
líneas 254-285   includes condicionados + prototipos de red/MQTT
líneas 287-438   acceso directo al sensor (Wire) y persistencia en RTC
líneas 440-587   la maquina de la ventana de evento y su publicacion
líneas 590-690   proceso de muestra y salida CSV
líneas 692-916   red, MQTT, sonda ICMP
líneas 918-1039  setup() y loop()
```

### 2.1 Los dos `#define` que gobiernan todo

```cpp
#define MODE_SERIAL_CSV  0    // linea 75
#define MQTT_DRY_RUN  1       // linea 81
```

| `MODE_SERIAL_CSV` | Qué hace |
|---|---|
| `0` | modo evento + MQTT. Es el modo de producción |
| `1` | modo serie puro: sin WiFi, una línea CSV cada 50 ms, **formato idéntico a v1**. Sirve para depurar en el escritorio |

| `MQTT_DRY_RUN` | Qué hace |
|---|---|
| `0` | publica de verdad al broker |
| `1` | **no abre WiFi ni MQTT**; imprime por serie cada mensaje que *enviaría*, con su topic y su tamaño |

El ensayo en seco no es un debug improvisado: es el mecanismo con el que se
validó el payload MQTT sin depender de un broker. `setup()` lo salta con
`#if !MQTT_DRY_RUN` (líneas 976-979) y `sendToBroker()` tiene las dos ramas
(líneas 803-835).

Las dos combinaciones compilan:

| Combinación | RAM | IRAM | Flash |
|---|---|---|---|
| `DRY_RUN 1` + `MODE 0` (estado actual) | 34300 / 80192 (42 %) | 60931 / 65536 (92 %) | 258264 (24 %) |
| `DRY_RUN 0` + `MODE 0` (producción) | 34580 / 80192 (43 %) | 61011 / 65536 (93 %) | 271724 (25 %) |

El IRAM al 92-93 % es **baseline del SDK de Espressif**, no algo introducido por el
proyecto: v1 ya estaba en 92 % sin haber escrito una línea de red. v2 solo aporta
+80 bytes.

---

## 3. Constantes (líneas 101-172)

### 3.1 Sensor y muestreo

| Constante | Línea | Valor | Por qué ese valor |
|---|---|---|---|
| `BAUD_RATE` | 101 | 115200 | 200 líneas de volcado ≈ 2.6 KB ≈ 0.9 s. A 9600 serían 27 s |
| `SENSOR_RANGE` | 104 | `±2 g` | máxima sensibilidad: un golpe de bus no pasa de 2 g |
| `SENSOR_DATARATE` | 105 | 100 Hz | 2× el ancho de banda de la vibración que interesa |
| `SAMPLE_INTERVAL_MS` | 106 | 10 | 100 Hz |
| `PRINT_INTERVAL_MS` | 107 | 50 | la línea CSV de v1, solo en `MODE_SERIAL_CSV 1` |
| `SENSOR_RETRY_MS` | 108 | 1000 | reintento en background si el chip no responde |

### 3.2 Detección

| Constante | Línea | Valor |
|---|---|---|
| `SETPOINT_N` | 111 | 20 |
| `ARROW_THRESHOLD_G` | 112 | 0.20 |
| `SEVERE_FACTOR` | 113 | 1.2 |
| `SEVERE_THRESHOLD_G` | 114 | `0.20 × 1.2` = 0.24, **calculado** |
| `RETURN_G` | 115 | 0.15 |
| `EVENT_LOCKOUT_MS` | 118 | 250 |
| `REST_MS` | 119 | 50 |

`SEVERE_THRESHOLD_G` se define como expresión para que subir `ARROW_THRESHOLD_G`
arrastre al severo sin editar dos sitios.

`RETURN_G < ARROW_THRESHOLD_G` es la **histéresis**: hace falta bajar claramente
por debajo del umbral para volver a reposo. Sin ella, una muestra que cruza y la
siguiente que vuelve generarían dos eventos por el mismo bache.

### 3.3 Ventana y troceado

| Constante | Línea | Valor | RAM |
|---|---|---|---|
| `PRE_N` | 125 | 100 | `preRing` = 800 B |
| `POST_N` | 126 | 100 | — |
| `EVENT_N` | 127 | 200 | `eventBuf` = 1600 B |
| `EVENT_SPAN_MS` | 128 | 2000 | derivado: `(PRE_N + POST_N) × SAMPLE_INTERVAL_MS` |
| `CHUNK_N` | 129 | 40 | |
| `CHUNKS` | 130 | 5 | `EVENT_N / CHUNK_N` |
| `MQTT_BUF_MAX` | 144 | **1024** | > 870 B, el chunk más largo medido |

El reparto de RAM está medido: el delta total de v2 sobre v1 es **+4428 B**
(800 del anillo + 1600 del buffer + 1024 de `mqttBuf` + ~1000 del stack de WiFi).

`mqttBuf` (línea 229) es **global y estático a propósito**: 1 KB en el stack del
ESP8266 es arriesgado, y `mqttBuf` solo se usa dentro de `publishEvent()`, que
no se anida con nada.

### 3.4 Red

| Constante | Línea | Valor |
|---|---|---|
| `MAX_WIFI_RETRIES` | 141 | 120 × 250 ms = 30 s antes de reiniciar |
| `MAX_MQTT_FAILURES` | 142 | 10 publicaciones fallidas seguidas → reinicio |
| `MQTT_CONNECT_TIMEOUT` | 143 | 120 s de reintento de conexión MQTT |
| `WDT_FEED_INTERVAL` | 145 | 4 s, margen 2× sobre `WDTO_8S` |
| `STALE_REPORT_MS` | 146 | 15 min sin publicar nada → reinicio |
| `PING_TARGET_IP` | 149 | `0x08080808` = 8.8.8.8 |
| `PING_MAX_TRIES` | 150 | 5 |
| `PING_INTERVAL` | 154 | 30 rondas |

---

## 4. Acceso al sensor (líneas 287-402)

El firmware **no se apoya en la librería de SparkFun**. Usa `Wire` directamente,
porque así puede leer los registros que necesita diagnosticar el módulo y
recuperarse de cables flojos.

| Función | Línea | Qué hace |
|---|---|---|
| `readReg(reg)` | 291 | lee un registro por I²C |
| `writeReg(reg, val)` | 299 | escribe un registro |
| `readRawAxes()` | 306 | burst de 6 bytes desde `0x32` → `rawX/Y/Z` en `int16_t` |
| `bwRateStr(code)` | 323 | nombre legible del código de `BW_RATE`, para el diagnóstico |
| `runChipDiagnostic()` | 336 | vuelca `DEVID`, offsets, `BW_RATE`, `POWER_CTL`, `DATA_FORMAT` |
| `readAccel()` | 392 | `readRawAxes()` + conversión a g |

### 4.1 La conversión

```cpp
gx = rawX / 256.0f;   // linea 395
```

256 LSB/g es la sensibilidad del ADXL345 en rango ±2 g. Es la razón por la que
los valores viajan como **enteros crudos** por MQTT: son la resolución nativa del
sensor, no una aproximación.

`readRawAxes()` lee 6 bytes de golpe. `Wire.requestFrom()` **no** es un
`requestFrom` bloqueante: si el bus está ocupado devuelve menos bytes de los
pedidos, y por eso la función valida el recuento antes de desensamblar el burst.

---

## 5. La máquina de estados

Solo dos estados (línea 199):

```cpp
enum DetectState : uint8_t { ST_IDLE = 0, ST_LOCKED = 1 };
```

No hay estado «capturando», porque `collecting` (bool) ya cubre eso. Separar
«detectar» de «capturar» permite que la ventana siga llenándose **mientras** el
detector está bloqueado.

### 5.1 `ST_IDLE` (líneas 627-654)

Calcula `devZ = gz - spZ` (línea 621) y:

```cpp
bool eventActive = fabsf(devZ) > ARROW_THRESHOLD_G;   // linea 622
bool allResting  = (fabsf(devZ) <= RETURN_G);         // linea 623
```

Si `eventActive`, elige el símbolo y abre la ventana:

```cpp
eventChar = (devZ >=  SEVERE_THRESHOLD_G) ? '*'
          : (devZ >   ARROW_THRESHOLD_G)  ? '+'
          : (devZ <= -SEVERE_THRESHOLD_G) ? '!'
          :                                '-';       // lineas 634-637
symbolPending = true;
detectState = ST_LOCKED;
lockUntilMs = millis() + EVENT_LOCKOUT_MS;
beginCapture();                                        // linea 642
```

Si no hay evento, **recalcula el setpoint silenciosamente** cada 20 muestras en
reposo (líneas 644-652). Es decir: el setpoint no es solo el del arranque, se va
ajustando al ángulo real de montaje. No avisa por serie porque pasa cada 200 ms.

### 5.2 `ST_LOCKED` — el desbloqueo de tres condiciones

Este es el punto más delicado del sketch. Desbloquear exige **las tres** a la vez
(líneas 664-673):

```cpp
if (!collecting && allResting) {
  if (restSinceMs == 0) restSinceMs = millis();
  if (millis() >= lockUntilMs &&
      (millis() - restSinceMs) >= REST_MS) {
    detectState = ST_IDLE;
    restSinceMs = 0;
  }
} else {
  restSinceMs = 0;
}
```

| Condición | Qué evita |
|---|---|
| `millis() >= lockUntilMs` | el rebote de ~100-200 ms del contacto del sensor |
| `millis() - restSinceMs >= REST_MS` | que un instante aislado reinicie el detector |
| **`!collecting`** | que un segundo evento **pise `eventBuf` mientras la ventana anterior se sigue llenando** |

La tercera es la crítica. La ventana dura **2000 ms**, mucho más que
`EVENT_LOCKOUT_MS + REST_MS` (300 ms). Sin ese `!collecting`, un evento a los
300 ms del anterior sobrescribiría `eventBuf` a mitad de llenado y la publicación
saldría con muestras mezcladas de dos eventos distintos.

---

## 6. La ventana de evento

Tres funciones y dos buffers.

### 6.1 `pushSample(dt)` — el anillo del pasado (línea 440)

```cpp
preRing[ringIdx].x = rawX;  /* … */
ringIdx = (ringIdx + 1) % PRE_N;
```

Se llama en **cada** muestra, haya evento o no (línea 1011). Es lo que permite
enviar el segundo previo al cruce. 100 muestras × 8 bytes = 800 bytes.

`struct sample_t` (línea 181):

```cpp
struct sample_t { int16_t x, y, z; uint16_t dt; };   // 8 bytes
```

El campo `dt` —milisegundos **reales** entre muestras— es lo que permite que el
gráfico del cliente sea fiel: sin él, la latencia del WiFi y del troceado
distorsionarían la duración del evento.

### 6.2 `beginCapture()` (línea 453)

```cpp
memcpy(eventBuf, preRing, sizeof(preRing));   // vuelca 1 s de historial
copyIndex    = PRE_N;
collecting   = true;
eventStartMs = millis();
eventPeakZ   = fabsf(devZ);
eventCount++;
saveEventCounter();
```

**Detalle fino:** la última muestra del anillo es precisamente la que cruzó el
umbral, así que queda incluida por el `memcpy` y no hay que duplicarla. Por eso
`copyIndex` arranca en `PRE_N` (=100) y no en `PRE_N - 1`.

`saveEventCounter()` se llama aquí y no en `finishCapture()`: si la placa se
reinicia a mitad de la ventana, el evento ya consumió un `id` y el cliente no ve
huecos.

### 6.3 `advanceCapture(dt)` (línea 470)

Rellena el resto y, al llenarse, cierra:

```cpp
if (d > eventPeakZ) eventPeakZ = d;    // el pico se mide sobre TODA la ventana
…
copyIndex++;
if (copyIndex < EVENT_N) return;
collecting = false;
finishCapture();
```

`eventPeakZ` se actualiza en **cada** muestra de la ventana, no solo en el cruce.
Por eso `peak` del resumen dice «cuánto fue elinos excursion máximo» y no
«cuánto valía en el instante del cruce».

### 6.4 `finishCapture()` — el orden importa (línea 493)

```cpp
#if MODE_SERIAL_CSV == 0
  publishEvent();          // MQTT PRIMERO
#endif
  Serial.printf("[EVENTO] …");
#if DUMP_EVENT_CSV
  for (uint8_t i = 0; i < EVENT_N; i++) { … }   // volcado serie DESPUES
#endif
```

Publicar primero y volcar después es deliberado: **imprimir 200 líneas por serie a
115200 baud tarda ~0.9 s**, y hacerlo antes retrasaría la entrega al broker medio
segundo por evento.

El símbolo se coloca en `i == PRE_N` (línea 512), que es la muestra que
efectivamente disparó. Con `PRE_N - 1` marcaba la muestra equivocada — un bug
real, corregido.

---

## 7. `publishEvent()` (línea 526)

### 7.1 El descarte

```cpp
#if !MQTT_DRY_RUN
  if (!client.connected()) {
    Serial.println(F("[MQTT] sin broker: evento descartado (politica v2)"));
    return;
  }
#endif
```

**No hay cola.** Es una decisión de diseño, no una limitación: una cola en RAM
obliga a decidir cuántos eventos aguantar y qué hacer al llenarse, y en un bus los
eventos con 10 minutos de retraso valen mucho menos que los de ahora. La placa
sigue midiendo y el siguiente evento se publica con normalidad.

El descarte va bajo `#if !MQTT_DRY_RUN` porque en ensayo en seco **siempre** se
genera el payload: es justo lo que hay que poder ver.

### 7.2 El resumen

```cpp
char sum[200];
snprintf(sum, sizeof(sum),
         "{\"id\":%lu,\"sym\":\"%c\",\"spz\":%.2f,\"dev\":%.2f,\"peak\":%.2f,"
         "\"n\":%u,\"ms\":%u,\"up\":%lu,\"count\":%lu}", …);
```

`sum[200]` es un buffer **local** y no `mqttBuf`: son 200 bytes, no 1 KB, y así
el buffer grande queda libre para los chunks.

`count` duplica `id`. Se conserva deliberadamente por compatibilidad con
consumidores que ya leían ese nombre.

### 7.3 El macro `APPEND_FIELD` (línea 558)

```cpp
#define APPEND_FIELD(NAME, EXPR)                                              \
  do {                                                                        \
    if (n >= MQTT_BUF_MAX - 8) break;                                        \
    n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n, ",\"%s\":[", NAME);          \
    for (uint8_t k = 0; k < CHUNK_N; k++) {                                   \
      n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n, "%s%d",                   \
                    k ? "," : "", (int)(EXPR));                               \
      if (n >= MQTT_BUF_MAX) { n = (uint16_t)(MQTT_BUF_MAX - 1); break; }     \
    }                                                                         \
    n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n, "]");                        \
  } while (0)
```

Se invoca así (líneas 571-574):

```cpp
APPEND_FIELD("z", eventBuf[base + k].z);
APPEND_FIELD("x", eventBuf[base + k].x);
APPEND_FIELD("y", eventBuf[base + k].y);
APPEND_FIELD("dt", eventBuf[base + k].dt);
```

Dos propiedades deliberadas:

**Cada campo lleva su coma delante y cierra su propio `]`.** Así el orden de los
campos es libre y no hay que llevar la cuenta de cuál va primero. La versión
anterior empezaba el macro con `],"NAME":[`, que **cerraba el array anterior antes
de escribir nada** y producía `"z":[]` vacío. Ese fue un bug real que se detectó
golpeando la mesa y mirando la salida por serie.

**El clamp de tamaño.** `MQTT_BUF_MAX - n` con `n` desbordado daría un
underflow de `size_t` y un `snprintf` con un tamaño enorme. El macro comprueba
antes de restar y, si aun así se llega al tope, fija `n = MQTT_BUF_MAX - 1` y
sale del bucle.

### 7.4 Tamaños medidos

Los chunks se validaron replicando la secuencia exacta de `snprintf` en un
programa de host y comprobando el resultado con un parser JSON:

| Escenario | Tamaño |
|---|---|
| Reposo (`z`≈235) | 577 B |
| Con el golpe | 617 B |
| **Peor caso** (todo a −512, `up0`=4294967295) | **902 B** |

902 B contra un buffer de 1024 deja 122 B de margen. Por eso
`MQTT_BUF_MAX` es 1024 y no 512: **512 no habría cabido**.

---

## 8. Persistencia — RTC user memory (líneas 404-434)

El EEPROM emulado del ESP8266 es **un solo sector de 4096 bytes sin wear
leveling**: cada `commit()` llama a `flashEraseSector()` y reescribe los 4096
bytes. Con ~10000 ciclos de erase, un commit por segundo destruye la flash en
unas 11 horas. **v2 no usa EEPROM**, y no debería añadirse.

Lo que sí hay son los **16 bytes (4 palabras) de user memory del RTC**, que están
en RAM alimentada y **no tocan la flash**:

| Palabra | Contenido |
|---|---|
| 0–1 | `rtcData_t { magic 0xDEAD, failureFlag }` |
| 2 | contador de eventos |
| 3 | `RTC_EVT_MAGIC` = `0x45565431` (`'E','V','T','1'`) |

```cpp
#define RTC_EVT_MAGIC 0x45565431UL        // linea 415
#define RTC_EVT_MAX   100000000UL         // linea 416

void loadEventCounter() {
  ESP.rtcUserMemoryRead(2, &v, 1);
  ESP.rtcUserMemoryRead(3, &magic, 1);
  if (magic == RTC_EVT_MAGIC && v <= RTC_EVT_MAX) {
    eventCount = v;
  } else {
    eventCount = 0;
    Serial.println(F("[RTC] contador sin validar: empieza en 0"));
  }
}
```

La validación con magic es indispensable: **la memoria del RTC no se inicializa
al reset**, así que en el primer arranque, o después de un corte de luz, la
palabra 2 contiene basura. Sin comprobarla, el primer evento salía con
`"id":2626976494`. La doble condición (magic **y** rango razonable) evita que una
palabra válida por casualidad con un valor imposible se cuele.

El RTC alimentado sobrevive a los reinicios por red, pero **no** a un corte de
luz. Es aceptable: al cliente le importa que los `id` estén ordenados, no que
sean continuos.

---

## 9. Red y MQTT (líneas 692-916)

### 9.1 `randomizeMac()` (línea 697)

```cpp
WiFi.disconnect();
uint8_t mac[6]; WiFi.macAddress(mac);
uint8_t old = mac[5];
mac[5] = random(0x01, 0xFE);
wifi_set_macaddr(STATION_IF, mac);
```

Solo cambia **el último byte**, dejando el OUI del fabricante. Es una medida
anti-registro: una misma MAC pidiendo las mismas credenciales, una y otra vez, es el
patrón más fácil de detectar y de bloquear.

Requiere el `extern "C"` de `user_interface.h` en el bloque de includes
condicionados al modo MQTT. **Sin ese `extern "C"` la compilación falla** con
`undefined reference to wifi_set_macaddr`.

### 9.2 `rebootWithNewMac()` (línea 706)

```cpp
rtcData_t rtc; rtc.magic = 0xDEAD; rtc.failureFlag = 1;
ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, 2);
delay(100); ESP.restart();
```

Deja la marca en el RTC **antes** de reiniciar. `setup()` (líneas 945-952) la
detecta, randomiza una vez más y limpia el flag. Así el reinicio por fallo de red
**no entra en bucle**: la segunda pasada ya no encuentra el flag.

### 9.3 `startWifi()` (línea 712)

Bucle con `delay(250)` y `ESP.wdtFeed()` en cada vuelta, y un timeout de
`MAX_WIFI_RETRIES` (30 s). Al expirar, distingue los cuatro `wl_status_t`
posibles (`WL_NO_SSID_AVAIL`, `WL_CONNECT_FAILED`, `WL_CONNECTION_LOST`,
`WL_DISCONNECTED`) porque el mensaje de diagnóstico es lo primero que hace falta
cuando la placa "no conecta".

### 9.4 `startMqtt()` (línea 763)

```cpp
client.setServer(MQTT_BROKER, MQTT_PORT);
client.setBufferSize(MQTT_BUF_MAX);   // el chunk completo no entra en 256
```

**El `setBufferSize` es obligatorio.** El buffer por defecto de `PubSubClient` son
256 bytes; con 902 bytes de chunk, `publish()` fallaría siempre. Es el motivo
directo de que `MQTT_BUF_MAX` sea 1024.

Detecta el error de autenticación explícitamente:

```cpp
if (client.state() == 5) {
  Serial.println(F("MQTT auth error. Check credentials/ClientId"));
}
```

Reintenta 10 veces con `delay(1000)` **enzamizados con `wdtFeed()`** — un
`delay(10000)` a pelo lo mataría el watchdog a los 8 s.

### 9.5 `mqttCallback()` (línea 745)

Registra todo lo recibido y atiende un comando:

```cpp
if (strcmp(msg, "RECAL") == 0) {
  setpointReady = false;
  accSumX = accSumY = accSumZ = 0.0f;
  accCount = 0;
}
```

`RECAL` **recalibra en caliente**, sin reiniciar. En campo es lo único práctico
para fijar el setpoint en el ángulo real de montaje con el bus en marcha.

El `memcpy` está acotado a `sizeof(msg) - 1` (línea 747): `length` viene de la
red y no se puede confiar en él.

### 9.6 La sonda ICMP (líneas 838-916)

Usa el ping ICMP real del SDK (`ping.h` + `user_interface.h`, dentro de un
`extern "C"`). **El `#include <ping.h>` es obligatorio**: sin él fallan
`ping_resp`, `ping_option` y `ping_start`.

Sondea **8.8.8.8**, no el broker. La razón es separar las dos causas de fallo: si
8.8.8.8 responde pero el broker no acepta, el problema son las credenciales; si
8.8.8.8 tampoco responde, es cobertura o enlace.

`servicePing()` corre una ronda de `PING_MAX_TRIES` pings cada
`PING_INTERVAL` rondas. El resultado alimenta `STALE_REPORT_MS`: si pasan 15
minutos sin una publicación buena, se reinicia.

### 9.7 `sendToBroker()` (línea 802)

Dos ramas. En ensayo en seco:

```cpp
dryMsgs++;
dryBytes += strlen(message);
Serial.printf("[MQTT-OUT] --> %s/%s  (%u B)\n", MQTT_CLIENT, topic, …);
Serial.print  ("[MQTT-OUT] ");
Serial.println(message);
```

`lastGoodPublish` y `mqttPublishFailCount` se actualizan igual, así que la lógica
de salud de red es idéntica en ambos modos y se puede probar el watchdog sin
broker.

En producción, cuenta fallos y reinicia a los `MAX_MQTT_FAILURES`, tanto si
`publish()` falla como si el cliente está desconectado.

---

## 10. `setup()` y `loop()` (líneas 918-1039)

### 10.1 Orden de `setup()`

1. `ESP.wdtEnable(WDTO_8S)` — **primero de todo**, antes de cualquier bloqueo.
2. Banner según el modo.
3. `memset` de `preRing` y `eventBuf`.
4. `loadEventCounter()`.
5. Detección del reinicio por fallo (magic `0xDEAD`) → `randomizeMac()`.
6. `Wire.begin(PIN_SDA, PIN_SCL)`.
7. `accel.begin()`, `setRange()`, `setDataRate()`.
8. `runChipDiagnostic()`.
9. `startWifi()` + `startMqtt()` (solo si `!MQTT_DRY_RUN`).

### 10.2 `loop()` (línea 993)

```cpp
ESP.wdtFeed();
unsigned long now = millis();

#if MODE_SERIAL_CSV == 0
#if !MQTT_DRY_RUN
  if (WiFi.status() != WL_CONNECTED) { wifiConnectRetries = 0; startWifi(); }
  checkMqtt();
  client.loop();
  servicePing();
#endif
#endif

if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
  uint16_t dt = (uint16_t)(now - lastSampleMs);
  lastSampleMs = now;
  if (readAccel()) {
    pushSample(dt);       // anillo del pasado, siempre
    processSample();      // deteccion -> puede abrir ventana
    advanceCapture(dt);   // si hay ventana, la sigue llenando
  }
}
```

El **orden de las tres llamadas es lo importante**:

1. `pushSample()` **antes** de `processSample()`, para que la muestra que dispara
   el evento esté ya en el anillo cuando `beginCapture()` haga el `memcpy`.
2. `processSample()` antes de `advanceCapture()`, para que `devZ` esté fresco cuando
   se mide el pico.
3. `advanceCapture()` al final, que es la que puede disparar `finishCapture()`.

El muestreo usa `if (now - last >= intervalo)`, no `delay()`. Con
`delay(10)` más el trabajo de red, el intervalo real se iría hacia los 15-20 ms
y la frecuencia caería por debajo de los 100 Hz anunciados.

Al final, el reintento en background del sensor (líneas 1026-1037): si
`sensorOnline` es falso, reintenta `accel.begin()` cada segundo **sin bloquear la
adquisición**, y solo imprime el aviso de «NO DETECTADO» una vez
(`notDetectedReported`) para no inundar el puerto.

---

## 11. Contrato MQTT (resumen para el cliente)

Ver `docs/datasheet.md` §6 para la tabla completa. Puntos que un cliente **debe**
respetar:

| Regla | Motivo |
|---|---|
| Los valores de `z`/`x`/`y` son **enteros crudos** | hay que dividir por 256 |
| `up` y `up0` son **uptime**, no hora de reloj | la placa no sincroniza ningún reloj |
| `dt` son **ms reales**, no nominales | include el jitter real del muestreo y del WiFi |
| Reensamblar por **`id`**, en orden de `i` | es lo único que une los 5 chunks |
| Un evento son **6 mensajes** (1 + 5) | hay que esperar a los 5 chunks |
| `sym` aparece **una sola vez** en el volcado serie, en la muestra `PRE_N` | el resto de la ventana es `.` |

### 11.1 El hueco conocido: la hora de reloj

No hay `configTime()`, ni NTP, ni RTC externo. `up` es un contador que empieza en
cero en cada arranque.

Para la jornada de 11 horas esto importa. Dos salidas, en orden deydata:

1. **Pendiente**: añadir `configTime()` y un campo `ep` con el epoch al conectar.
2. **Mientras tanto**: usar `id` como eje (monótono, sin huecos) y el instante de
   **recepción** como ancla temporal.

---

## 12. La app de escritorio

### 12.1 Arquitectura

```
   puerto serie ──┐
   SIM ───────────┼──> SerialReader (hilo) ──> deque de puntos ──┐
   REPLAY:.log ───┘         + lista de summaries                  │
                                                                    ├──> HTTP local
   index.html + app.js + style.css ────────────────────────────────┘      127.0.0.1
                                                                              │
                                                                          pywebview
                                                                          o navegador
```

| Fichero | Responsabilidad |
|---|---|
| `app/app.py` | servidor HTTP local en `127.0.0.1`, API JSON, CLI, pywebview |
| `app/serial_reader.py` | hilo de lectura, parseo, retención, replay |
| `app/app.js` | frontend: canvas, panel de eventos, barra de replay |
| `app/index.html` | estructura |
| `app/style.css` | estilos |

Sin CDN: todo es local, así que funciona sin internet.

### 12.2 Dos formatos de línea

`parse_line(line, state)` acepta ambos, porque v2 debe seguir leyéndose con el
parser de v1:

**Formato A** — v1 continuo, 8 campos:

```
t,X,Y,Z,setX,setY,setZ,sym
123456,0.14,0.28,0.88,0.14,0.28,0.88,.
```

**Formato B** — v2 ventana de evento, prefijo `EV,`:

```
EV,t0,Xraw,Yraw,Zraw,dt,sym
EV,37795,30,33,235,10,.
```

### 12.3 La reconstrucción del tiempo (el detalle importante)

En el formato B, **`t0` se repite en las 200 líneas** de la ventana. Usar `t0`
como eje temporal aplastaría toda la ventana en un solo punto.

`_parse_event_line()` lleva un acumulador:

```python
if state.get("t0") != t0:      # ventana nueva -> reiniciar
    state["t0"] = t0
    state["acc"] = 0
t = t0 + state.get("acc", 0)
state["acc"] = state.get("acc", 0) + dt
```

Es el mismo `dt` que viaja en el payload MQTT, y por eso el gráfico del
escritorio reproduce la duración real del evento y no la ideal.

### 12.4 Retención por tiempo, no por muestras

```python
MAX_POINTS     = 40000   # techo duro
KEEP_SECONDS   = 300.0
MIN_KEEP_POINTS = 4000
```

Un buffer contado en muestras es un error en v2: **durante el reposo no llega
nada**, así que un evento por minuto llenaría el buffer en minutos y perdería los
anteriores. `_trim_locked()` descarta por antigüedad pero nunca baja de
`MIN_KEEP_POINTS`.

Verificado inyectando 12000 puntos con un evento cada 40 s: retiene los 4000 del
suelo, con un span de 762 s.

### 12.5 El replay de capturas

`_replay_loop()` reproduce un `.log` de CuTTeM. Para mantener el ritmo original
parsea la marca de tiempo del propio log con `_split_replay_stamp()`, que acepta
varios formatos:

| Formato | Ejemplo | Resultado |
|---|---|---|
| `_STAMP_TPLUS` | `[T+004.971]` | 4971 ms |
| `_STAMP_TPLUS` | `[T+1:02.345]` | 62345 ms |
| `_STAMP_HMS` | `01:02:03.456 ` | 3723456 ms |
| `_STAMP_SIMPLE` | `[62.345s]` | 62345 ms |

El delay entre líneas se deriva de esas marcas y se divide por `--speed`. Si una
línea no lleva marca, se devuelve intacta y no se interrumpe la reproducción.

`--speed` va de 0.1 a 50. Con la velocidad máxima un golpe se ve en un par de segundos en
vez de en dos.

### 12.6 El panel de resúmenes

`_handle_json()` parsea las líneas `[MQTT-OUT] {...}` con `json.loads` y las
clasifica:

| Tipo de línea | Cómo se reconoce | Qué hace |
|---|---|---|
| setpoint | tiene `x`,`y`,`z` escalares y `z` no es lista | cachea `spz` |
| summary | tiene `sym` **y** `id` | añade a `_summaries`, incrementa `_summaries_seen`, cachea `spz` |
| chunk | tiene `id`,`i`,`up0` | se reconoce y se ignora (los datos ya vienen por `EV,`) |

Los resúmenes se muestran en la tarjeta «Eventos detectados» con su `id`, su
`up` y su pico.

---

## 13. Compilación y auditoría

```bash
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266-v2
```

Consola de `arduino-cli`, verificado que está en `~/.local/bin/arduino-cli`:

- `arduino-cli` 1.5.2-rc.1
- core `esp8266:esp8266@3.1.2`
- data dir `~/.arduino15`

### 13.1 Consumo de memoria

| Build | RAM | IRAM | Flash |
|---|---|---|---|
| v1 (referencia) | 30068 (37 %) | 60931 (92 %) | 246552 (23 %) |
| v2 `DRY_RUN 1` | 34300 (42 %) | 60931 (92 %) | 258264 (24 %) |
| v2 `DRY_RUN 0` | 34580 (43 %) | 61011 (93 %) | 271724 (25 %) |
| v2 `MODE_SERIAL_CSV 1` | 32216 (40 %) | 60931 (92 %) | — |

Las cuatro combinaciones compilan.

**Sobre el IRAM al 92-93 %:** no es algo introducido por este proyecto. El
SDK de Espressif y el stack de Arduino ya consumen esa cantidad antes de que
escribamos una línea. v1 estaba en 92 % y v2 aporta +80 bytes. **No es un
indicador útil de margen** en este proyecto; el que sí lo es es la RAM.

### 13.2 Cargar en la placa

```bash
arduino-cli board list                                    # SIEMPRE primero
arduino-cli upload -p <PUERTO> --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266-v2
```

**El puerto nunca se hardcodea.** Puede ser `/dev/ttyUSB0` (CH340/CP2102) o
`/dev/ttyACM0` (CDC nativo). Si `board list` no muestra nada, la placa no está
conectada: no hay nada que subir, y no se debe adivinar un puerto.

Si la carga falla con `[Errno 16] Device or resource busy`, hay otro programa con
el puerto abierto (CuTTeM, minicom, la app de escritorio). Cerrarlo; **no matar el
proceso del usuario**.

---

## 14. Notas de mantenimiento

### 14.1 Cambiar los umbrales

`ARROW_THRESHOLD_G` (línea 112) y `SEVERE_FACTOR` (línea 113). El severo se deriva
solo. Con el bus en marcha, 0.20 g da bastante evento; si hay demasiados, subir.

### 14.2 Cambiar la ventana

`PRE_N` y `POST_N` (líneas 125-126). Recordar:

```
EVENT_N = PRE_N + POST_N           debe ser múltiplo de CHUNK_N
CHUNKS  = EVENT_N / CHUNK_N
```

Y que `eventBuf` crece linealmente: 200 muestras = 1600 B.

### 14.3 Reactivar la publicación real

1. Conseguir credenciales que el broker acepte. Con las actuales
   `www.smartage.cl:1886` devuelve **`CONNACK 5` (not authorised)** para todos los
   intentos, **incluso anónimo**; verificado desde el PC con `mosquitto_sub` y
   con un cliente TCP en crudo, no desde el firmware.
2. Poner `MQTT_DRY_RUN 0` (línea 81).
3. Recompilar y subir.
4. **Dar un ClientId distinto por placa.** Con el mismo ClientId, el broker
   expulsa a una de las dos placas continuamente.

### 14.4 Lo que queda pendiente

| Tarea | Notas |
|---|---|
| Hora de reloj | `configTime()` + campo `ep` con el epoch |
| Credenciales válidas | bloquea la publicación real |
| Subir el build actual a la placa | la build en la placa es anterior a los arreglos de JSON |
| Bus de respaldo | `AP_SSID`/`AP_PASSWORD` ya están en `secrets.h`, sin usar |

---

## 15. Historial de arreglos relevantes

Errores encontrados por prueba en campo, todos corregidos. Se documentan porque
vuelven: son la clase de bug que reintroduce quien edite el serializado.

| Síntoma | Causa | Corrección |
|---|---|---|
| `"z":[]` vacío en los chunks | el macro empezaba con `],"z":[`, cerrando el array anterior | cada campo antepone su coma y cierra su propio `]` |
| JSON inválido por coma | faltaba la coma entre `up0` y `z` | la cabecera ya no lleva coma final |
| `"id":2626976494` | `loadEventCounter()` leía la palabra del RTC sin validar | magic `0x45565431` + rango máximo |
| símbolo en la muestra anterior | `(i == PRE_N - 1)` | `(i == PRE_N)` |
| segundo evento pisaba la ventana | `ST_LOCKED` no comprobaba `!collecting` | tercera condición de desbloqueo |
| riesgo de underflow en `snprintf` | `MQTT_BUF_MAX - n` con `n` desbordado | clamp antes de restar |
| `ts`/`t0` confirmados como hora | eran uptime | renombrados a `up`/`up0` y documentados |
| error de compilación con `ping_*` | faltaba `#include <ping.h>` | incluido en el `extern "C"` |
