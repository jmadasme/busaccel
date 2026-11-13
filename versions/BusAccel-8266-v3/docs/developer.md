# BusAccel-8266-v3 — Documentación de desarrollador

> Explicación del código fuente de `BusAccel-8266-v3.ino` (977 líneas).
> Para el uso del firmware ver [`datasheet.md`](datasheet.md).

---

## 1. Arquitectura del sketch

```
┌─────────────────────────────────────────────────────────────────────┐
│                         BusAccel-8266-v3.ino                       │
├─────────────────────────────────────────────────────────────────────┤
│  1.  Cabecera: descripción, mapa de pines, contrato, changelog     │
│  2.  #define MODE_SERIAL_CSV 0  /  #define MQTT_DRY_RUN 1         │
│  3.  Includes: Wire, math, Adafruit_Sensor, Adafruit_ADXL345_U     │
│      (condicionales a MODE 0: ESP8266WiFi, PubSubClient,           │
│       user_interface.h, ping.h, secrets.h)                          │
│  4.  Constantes: sensor/muestreo, detección, ventana, red          │
│  5.  Pines y registros ADXL345                                      │
│  6.  struct sample_t { int16_t x,y,z; uint16_t dt; }               │
│  7.  Estado global: sensor, setpoint, detección, ventana, red      │
│  8.  Prototipos                                                     │
│  9.  Acceso a registros por Wire + diagnóstico                      │
│ 10.  Contador RTC (load/save con validación reforzada)              │
│ 11.  pushSample()  — anillo del pasado                             │
│ 12.  beginCapture() — escaneo del anillo para el pico inicial       │
│ 13.  advanceCapture() — actualiza pico en cada muestra POST         │
│ 14.  finishCapture() — publica UNA vez + imprime UNA línea [PICO]   │
│ 15.  publishPeakEvent() — serializa el único mensaje                │
│ 16.  processSample() — máquina de estados IDLE/LOCKED               │
│ 17.  printCharTick() — CSV 20 Hz (solo MODE 1)                      │
│ 18.  Red: WiFi/MQTT/ping (solo MODE 0)                              │
│ 19.  setup() / loop()                                               │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.1 Estructura de `setup()`

```
ESP.wdtEnable(WDTO_8S)          ← primero, antes de cualquier bloqueo
Serial.begin(115200) + banner v3 según modo
memset(preRing, 0, sizeof(preRing))   ← solo el anillo (ya no eventBuf)
loadEventCounter()              ← ANTES de imprimir (bug 1); valida siempre
detección de reinicio por fallo (magic 0xDEAD en palabras 0-1) → randomizeMac()
Wire.begin(PIN_SDA, PIN_SCL)    ← Wire.begin(4, 5)
accel.begin() + setRange(±2g) + setDataRate(100Hz)
runChipDiagnostic() + forzado de measurement
solo si MODE 0 && !MQTT_DRY_RUN: startWifi() + startMqtt()
imprime umbrales y resumen de la ventana
```

### 1.2 Estructura de `loop()`

```cpp
ESP.wdtFeed();
unsigned long now = millis();

// --- Red (solo MODE 0 && !MQTT_DRY_RUN) ---
if (WiFi.status() != WL_CONNECTED) { wifiConnectRetries = 0; startWifi(); }
checkMqtt();
client.loop();
servicePing();
// stale publish: si millis() - lastGoodPublish > STALE_REPORT_MS (15 min) → reinicio

// --- Muestreo interno a ~100 Hz ---
if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
  uint16_t dt = (uint16_t)(now - lastSampleMs);
  lastSampleMs = now;
  if (readAccel()) {
    pushSample(dt);       // 1) anillo del pasado, SIEMPRE
    processSample();      // 2) detección -> puede abrir ventana (beginCapture)
    advanceCapture(dt);   // 3) si hay ventana, actualiza pico y cuenta POST
  }
}

// --- Impresión CSV (solo MODE 1) ---
#if MODE_SERIAL_CSV == 1
  if (setpointReady && (now - lastPrintMs >= PRINT_INTERVAL_MS)) {
    lastPrintMs = now; printCharTick();
  }
#endif

// --- Reintento del sensor en background ---
if (!sensorOnline && (now - lastRetryMs >= SENSOR_RETRY_MS)) { ... }
```

> **El orden de las tres llamadas es crítico:** `pushSample()` antes de
> `processSample()` para que la muestra que dispara ya esté en el anillo;
> `processSample()` antes de `advanceCapture()` para que `devZ` esté fresco al
> medir el pico.

---

## 2. Máquina de estados de detección

### 2.1 Estados

```cpp
enum DetectState : uint8_t { ST_IDLE = 0, ST_LOCKED = 1 };
```

### 2.2 Diagrama de flujo

```
                    ┌──────────────────────────────────────────────┐
                    │                                              │
                    ▼                                              │
    ┌────────────────────────┐                                     │
    │       ST_IDLE          │                                     │
    │                        │                                     │
    │  |devZ| <= 0.20 ?      │                                     │
    │    sí: acumular para   │                                     │
    │    setpoint, sym='.'   │                                     │
    │    no: elegir símbolo  │                                     │
    │         → ST_LOCKED    │                                     │
    │         → beginCapture │                                     │
    └───────────┬────────────┘                                     │
                │                                                  │
                ▼                                                  │
    ┌────────────────────────┐                                     │
    │      ST_LOCKED         │                                     │
    │                        │                                     │
    │  ignorar cruces        │                                     │
    │  (rebote)              │                                     │
    │                        │                                     │
    │  desbloqueo exige:    │                                     │
    │   (a) lockout cumplido │                                     │
    │   (b) reposo >= 50 ms  │                                     │
    │   (c) !collecting      │                                     │
    └───────────┬────────────┘                                     │
                │                                                  │
                └──────────────────────────────────────────────────┘
```

### 2.3 Lógica en `processSample()`

```cpp
void processSample() {
  if (!setpointReady) {
    // acumular 20 lecturas → fijar spX/spY/spZ
    // publicar accel/setpoint (solo MODE 0)
    return;
  }

  devZ = gz - spZ;
  bool eventActive = fabsf(devZ) > ARROW_THRESHOLD_G;   // 0.20
  bool allResting  = (fabsf(devZ) <= RETURN_G);          // 0.15

  switch (detectState) {
    case ST_IDLE:
      if (eventActive) {
        eventChar = (devZ >=  SEVERE_THRESHOLD_G) ? '*'   // ≥ +0.24
                  : (devZ >   ARROW_THRESHOLD_G)  ? '+'   // >  +0.20
                  : (devZ <= -SEVERE_THRESHOLD_G) ? '!'   // ≤ -0.24
                  :                                '-';   // <  -0.20
        symbolPending = true;
        detectState = ST_LOCKED;
        lockUntilMs = millis() + EVENT_LOCKOUT_MS;        // 250
        restSinceMs = 0;
        beginCapture();                                    // abre la ventana
      } else {
        // acumular para recálculo silencioso del setpoint
      }
      break;

    case ST_LOCKED:
      if (!collecting && allResting) {
        if (restSinceMs == 0) restSinceMs = millis();
        if (millis() >= lockUntilMs &&
            (millis() - restSinceMs) >= REST_MS) {        // 50
          detectState = ST_IDLE;
          restSinceMs = 0;
        }
      } else {
        restSinceMs = 0;
      }
      break;
  }
}
```

---

## 3. Captura y cálculo del pico (v3)

### 3.1 Estructura de la ventana

```cpp
sample_t preRing[PRE_N];       // 100 × 8 B = 800 B: historial rodante
uint8_t  ringIdx    = 0;       // próxima escritura en preRing
uint16_t postCount  = 0;       // muestras POST ya contadas
bool     collecting = false;   // hay una ventana en curso
uint32_t eventStartMs = 0;
float    eventDevZ   = 0.0f;   // devZ en el instante del cruce
float    eventPeakZ  = 0.0f;   // máx |devZ| de la ventana (pre + post)
char     eventSym    = '.';
uint32_t eventCount  = 0;      // eventos detectados desde el arranque
// eventBuf[EVENT_N]  <-- ELIMINADO en v3
// copyIndex          <-- ELIMINADO (reemplazado por postCount)
```

### 3.2 `pushSample(uint16_t dt)` — anillo del pasado

```cpp
void pushSample(uint16_t dt) {
  preRing[ringIdx].x  = rawX;
  preRing[ringIdx].y  = rawY;
  preRing[ringIdx].z  = rawZ;
  preRing[ringIdx].dt = dt;
  ringIdx = (uint8_t)((ringIdx + 1) % PRE_N);
}
```

Se escribe en **cada** muestra, evento o no. Es lo que permite medir el pico
de lo ocurrido **antes** del cruce.

### 3.3 `beginCapture()` — arranque de la ventana

```cpp
void beginCapture() {
  eventDevZ  = devZ;
  eventSym   = eventChar;
  eventPeakZ = fabsf(devZ);
  for (uint8_t i = 0; i < PRE_N; i++) {          // escanea el anillo
    float d = fabsf(preRing[i].z / 256.0f - spZ);
    if (d > eventPeakZ) eventPeakZ = d;
  }
  postCount    = 0;
  collecting   = true;
  eventStartMs = millis();
  eventCount++;
  saveEventCounter();          // consume id al abrir (ids sin huecos)
}
```

**Mide el pico de la parte previa escaneando el anillo**, en vez de copiarlo
(ya no se almacena la ventana completa).

### 3.4 `advanceCapture(uint16_t dt)` — avance de la ventana

```cpp
void advanceCapture(uint16_t dt) {
  (void)dt;
  if (!collecting) return;

  float d = fabsf(devZ);
  if (d > eventPeakZ) eventPeakZ = d;   // actualiza el pico

  postCount++;
  if (postCount >= POST_N) {            // 100
    collecting = false;
    finishCapture();
  }
}
```

**No almacena muestras**: solo actualiza el pico y cuenta POST. Al llegar a
`POST_N` cierra la ventana.

### 3.5 `finishCapture()` — cierre de la ventana

```cpp
void finishCapture() {
#if MODE_SERIAL_CSV == 0
  publishPeakEvent();          // MQTT primero
#endif

#if MODE_SERIAL_CSV == 0
  Serial.printf("[PICO] id=%lu sym=%c peak=%.2fg spz=%.2fg dev=%.2fg up=%lums n=%u ms=%u\n",
                (unsigned long)eventCount, eventSym, eventPeakZ, spZ, eventDevZ,
                (unsigned long)eventStartMs, (unsigned)EVENT_N, (unsigned)EVENT_SPAN_MS);
#endif
}
```

**MQTT primero, UNA línea serie después.** Sin volcado de la ventana (clave de
v3). La línea `[PICO]` está condicionada a `MODE_SERIAL_CSV == 0` (corrección
I-004).

### 3.6 `publishPeakEvent()` — un solo mensaje

```cpp
void publishPeakEvent() {
#if !MQTT_DRY_RUN
  if (!client.connected()) {
    Serial.println(F("[MQTT] sin broker: pico descartado (politica v3)"));
    return;
  }
#endif

  char msg[200];
  snprintf(msg, sizeof(msg),
           "{\"id\":%lu,\"sym\":\"%c\",\"spz\":%.2f,\"dev\":%.2f,\"peak\":%.2f,"
           "\"n\":%u,\"ms\":%u,\"up\":%lu}",
           (unsigned long)eventCount, eventSym, spZ, eventDevZ, eventPeakZ,
           (unsigned)EVENT_N, (unsigned)EVENT_SPAN_MS, (unsigned long)eventStartMs);
  sendToBroker("accel/event/peak", msg);
}
```

**Un único mensaje compacto por evento** con el PICO como dato principal.

---

## 4. Contador RTC con validación

### 4.1 Mapa de memoria RTC

| Palabras | Contenido |
|---|---|
| 0–1 | `rtcData_t` (magic + failureFlag) — detección de reinicio por fallo |
| 2 | contador de eventos (`eventCount`) |
| 3 | magic del contador (`RTC_EVT_MAGIC = 0x45565431` = `'EVT1'`) |

### 4.2 `loadEventCounter()` — validación reforzada (bug 1)

```cpp
#define RTC_EVT_MAGIC 0x45565431UL        // 'E','V','T','1'
#define RTC_EVT_MAX   100000000UL         // por encima, es basura seguro

void loadEventCounter() {
  uint32_t v = 0, magic = 0;
  ESP.rtcUserMemoryRead(2, &v, sizeof(v));       // tamaño en BYTES
  ESP.rtcUserMemoryRead(3, &magic, sizeof(magic));
  // magic correcto Y dentro de rango Y no es la palabra 2 sin inicializar (0)
  if (magic == RTC_EVT_MAGIC && v <= RTC_EVT_MAX && v != 0) {
    eventCount = v;
  } else {
    eventCount = 0;
    saveEventCounter();
    Serial.println(F("[RTC] contador sin validar: empieza en 0"));
  }
}
```

**Validación triple:** magic correcto **Y** valor dentro de rango **Y** valor
distinto de 0 (palabra sin inicializar). Si falla, el contador empieza en 0 y
se reescriben counter + magic.

### 4.3 `saveEventCounter()`

```cpp
void saveEventCounter() {
  ESP.rtcUserMemoryWrite(2, &eventCount, sizeof(eventCount));
  uint32_t magic = RTC_EVT_MAGIC;
  ESP.rtcUserMemoryWrite(3, &magic, sizeof(magic));
}
```

Se llama en `beginCapture()` (al abrir la ventana) para que los `id` queden
sin huecos ante un reinicio a mitad de ventana.

> **Nota:** la API `ESP.rtcUserMemoryRead/Write` espera el **tamaño en bytes**,
> no un número de palabras (corrección I-001). No toca la flash: sobrevive a
> reinicios por red (RTC alimentado) pero se pierde si se corta la luz.

---

## 5. Modos de compilación

### 5.1 `MODE_SERIAL_CSV`

```cpp
#define MODE_SERIAL_CSV  0   // 0 = MQTT por evento | 1 = CSV puro sin WiFi
```

| Valor | Efecto |
|---|---|
| **0** | Incluye `ESP8266WiFi`, `PubSubClient`, `user_interface.h`, `ping.h`, `secrets.h`. Detección por evento + publicación MQTT. Emite `[PICO]` |
| **1** | **Sin WiFi.** CSV puro a 20 Hz (`t,X,Y,Z,setX,setY,setZ,sym`). **No emite `[PICO]`** |

### 5.2 `MQTT_DRY_RUN`

```cpp
#define MQTT_DRY_RUN  1   // 1 = ensayo en seco | 0 = MQTT real
```

| Valor | Efecto |
|---|---|
| **1** (default) | **NO abre WiFi ni MQTT.** `sendToBroker()` imprime por serie: `[MQTT-OUT] --> <client>/<topic> (NN B)` + `[MQTT-OUT] {json}` |
| **0** | MQTT real: conecta WiFi y publica al broker. Requiere credenciales válidas |

### 5.3 `sendToBroker()` — punto de ramificación

```cpp
void sendToBroker(const char* topic, const char* message) {
#if MQTT_DRY_RUN
  // ---- ENSAYO EN SECO: nada sale de la placa, se muestra por serie ----
  dryMsgs++;
  dryBytes += strlen(message);
  Serial.printf("[MQTT-OUT] --> %s/%s  (%u B)\n",
                MQTT_CLIENT, topic, (unsigned)strlen(message));
  Serial.print  ("[MQTT-OUT] ");
  Serial.println(message);
  lastGoodPublish = millis();
  mqttPublishFailCount = 0;
#else
  if (client.connected()) {
    char t[160];
    snprintf(t, sizeof(t), "%s/%s", MQTT_CLIENT, topic);
    if (client.publish(t, message)) {
      mqttPublishFailCount = 0;
      lastGoodPublish = millis();
    } else {
      // fallo → contador → reinicio con MAC aleatoria si >= MAX_MQTT_FAILURES
    }
  } else {
    // sin conexión → contador → reinicio con MAC aleatoria
  }
#endif
}
```

---

## 6. API de funciones

### 6.1 Sensor y registros

| Función | Propósito |
|---|---|
| `uint8_t readReg(uint8_t reg)` | Lee 1 byte de un registro ADXL345 por I²C |
| `bool writeReg(uint8_t reg, uint8_t val)` | Escribe 1 byte en un registro |
| `bool readRawAxes()` | Burst de 6 bytes → `rawX/rawY/rawZ` (int16) |
| `bool readAccel()` | `readRawAxes()` + conversión a g (`raw/256.0f`) → `gx/gy/gz` |
| `void runChipDiagnostic()` | Diagnóstico profundo de registros al arrancar + forzado de measurement |
| `static const char* bwRateStr(uint8_t code)` | Decodifica el code de `BW_RATE` a string |

### 6.2 Captura y pico

| Función | Propósito |
|---|---|
| `void pushSample(uint16_t dt)` | Escribe la muestra en `preRing` y avanza `ringIdx` |
| `void beginCapture()` | Abre la ventana: escanea el anillo para el pico inicial, fija `eventDevZ/eventSym`, incrementa `eventCount`, guarda RTC |
| `void advanceCapture(uint16_t dt)` | Actualiza `eventPeakZ` con `|devZ|`, incrementa `postCount`, cierra la ventana al llegar a `POST_N` |
| `void finishCapture()` | Publica el pico (MQTT) + imprime la línea `[PICO]` |
| `void publishPeakEvent()` | Serializa el único mensaje JSON y lo envía por `sendToBroker()` |

### 6.3 Detección

| Función | Propósito |
|---|---|
| `void processSample()` | Máquina de estados `ST_IDLE`/`ST_LOCKED`: setpoint, `devZ`, símbolos, apertura de ventana |
| `void printCharTick()` | Línea CSV a 20 Hz (solo `MODE_SERIAL_CSV 1`) |

### 6.4 Contador RTC

| Función | Propósito |
|---|---|
| `void loadEventCounter()` | Lee y **valida** el contador de RTC (magic + rango + ≠0) |
| `void saveEventCounter()` | Escribe contador + magic en RTC |

### 6.5 Red (solo `MODE_SERIAL_CSV 0`)

| Función | Propósito |
|---|---|
| `void startWifi()` | Conexión WiFi STA con reintentos y diagnóstico de estado |
| `void startMqtt()` | Conexión MQTT, suscripción a `<client>/#`, publicación de `online`/`status` |
| `void checkMqtt()` | Reconexión MQTT si se perdió |
| `void sendToBroker(const char* topic, const char* message)` | Publica (o simula en dry-run) un mensaje |
| `void mqttCallback(char* topic, byte* payload, unsigned int length)` | Procesa mensajes recibidos; `RECAL` recalibra el setpoint |
| `void randomizeMac()` | Cambia el último byte de la MAC |
| `void rebootWithNewMac()` | Escribe magic `0xDEAD` en RTC y reinicia |
| `void servicePing()` | Gestiona la ronda ICMP a 8.8.8.8 |
| `void startPingRound()` | Inicia una ronda de ping |
| `bool brokerReachable()` | Fallback TCP al broker si ICMP está filtrado |
| `void onPingRecv(void* arg, void* pdata)` | Callback de respuesta ICMP |
| `void onPingSent(void* arg, void* pdata)` | Callback de envío ICMP (vacío) |

---

## 7. Guía de modificación y extensión

### 7.1 Ajustar umbrales

Edita las constantes en la cabecera del sketch:

```cpp
#define ARROW_THRESHOLD_G    0.20f
#define SEVERE_FACTOR        1.2f
#define RETURN_G             0.15f
```

Recuerda actualizar también `LEVE_THRESHOLD_G` / `SEVERE_THRESHOLD_G` en
`app/serial_reader.py` para que las zonas de color de la app coincidan.

### 7.2 Cambiar el tamaño de la ventana

```cpp
#define PRE_N                100           // 1 s a 100 Hz
#define POST_N               100           // 1 s a 100 Hz
```

`EVENT_N` y `EVENT_SPAN_MS` se derivan automáticamente. Ojo: `PRE_N` define el
tamaño de `preRing` (800 B con `PRE_N=100`); si lo subes mucho, vigila la RAM.

### 7.3 Añadir un nuevo sensor/actuador

1. **Pines libres:** `D3` (GPIO0), `D4` (GPIO2), `D5`–`D8` (GPIO14/12/13/15),
   `A0` (ADC0).
2. **I²C adicional:** el bus ya está en `Wire.begin(4,5)`; cualquier otro
   dispositivo I²C puede compartirlo (distinta dirección).
3. **Evita:** GPIO0 (FLASH), GPIO15 (pull-down obligatorio), GPIO1/GPIO3
   (UART).
4. **Alimentación:** 3.3 V máximo.

### 7.4 Añadir un campo al payload del pico

Edita el `snprintf` en `publishPeakEvent()`:

```cpp
snprintf(msg, sizeof(msg),
         "{\"id\":%lu,\"sym\":\"%c\",\"spz\":%.2f,\"dev\":%.2f,\"peak\":%.2f,"
         "\"n\":%u,\"ms\":%u,\"up\":%lu}",
         ...);
```

Si el payload crece más de ~300 B, sube `MQTT_BUF_MAX` a 512.

### 7.5 Añadir un topic de salida

1. Define una constante para el topic.
2. Llama `sendToBroker("topic", payload)` en el punto adecuado.
3. En `MQTT_DRY_RUN 1` se imprimirá automáticamente.

### 7.6 Añadir un comando de entrada

En `mqttCallback()`, añade una comparación:

```cpp
if (strcmp(msg, "MI_COMANDO") == 0) {
  // hacer algo
}
```

---

## 8. Decisiones de diseño

| Decisión | Justificación |
|---|---|
| **Pico en línea, sin almacenar la ventana** | ahorra 1600 B (`eventBuf`) y evita saturar serie/broker |
| **Anillo `preRing` siempre rodante** | permite medir el pico de lo ocurrido antes del cruce sin almacenar la ventana completa |
| **`saveEventCounter()` al abrir la ventana** | ids sin huecos ante reinicio a mitad de ventana |
| **Validación triple del contador RTC** | la RTC no se inicializa al reset; sin validación el contador arranca con basura |
| **`MQTT_DRY_RUN 1` por defecto** | permite validar formato y detección sin suscriptor en el broker |
| **`MODE_SERIAL_CSV 1` sin WiFi** | la app de escritorio sigue parseando el formato v1 |
| **Línea `[PICO]` condicionada a `MODE 0`** | en modo CSV puro la salida es exclusivamente CSV (contrato con la app) |
| **Buffer MQTT a 384 B** | el payload de pico es ~150 B; sobra margen |
| **Sin cola de eventos con el broker caído** | el evento se descarta y el equipo sigue midiendo; sin reinicio por ese motivo |
| **Reinicio con MAC aleatoria** | si el broker no responde de forma sostenida, se reinicia con MAC nueva (evita IP bloqueada) |
