// ============================================================================
// BusAccel-8266-v2 — PUBLICACION MQTT POR EVENTO SOBRE EL EJE Z (version 2)
// Placa: NodeMCU ESP8266 (ESP-12E)  |  FQBN: esp8266:esp8266:nodemcuv2
//
// Sensor: GY-291 (ADXL345) via I2C (rango +-2g, 256 LSB/g = raw/256.0f)
//   SDA -> D2/GPIO4 , SCL -> D1/GPIO5  (Wire.begin(4,5))
//
// QUE CAMBIA RESPECTO DE v1 (proyecto BusAccel-8266):
//   1. La deteccion pasa a SOLO EJE Z. v1 tomaba el eje de mayor desviacion
//      entre X/Y/Z; ahora el evento es que Z salga del rango normal, en los
//      dos sentidos: subida -> '+'/'*', caida -> '-'/'!'.
//   2. Cada evento captura una VENTANA de 200 muestras a 100 Hz (1 s antes +
//      1 s despues del cruce) en RAM, no una sola muestra. El cliente
//      administrador recibe la forma de onda y puede recrear el grafico.
//   3. Se publica por MQTT SOLO cuando hay evento. Sin evento no hay trafico.
//   4. La ventana se parte en 5 mensajes de 40 muestras, para no exceder el
//      tamano de paquete de PubSubClient (setBufferSize(1024)).
//   5. Se graba el dt REAL entre muestras: si el stack WiFi retrasa el
//      muestreo, el grafico del cliente usa el tiempo verdadero y no una
//      linea recta mentirosa.
//   6. Estado de red completo (WiFi + MQTT + watchdog + ICMP + reinicio con
//      MAC aleatoria), con la misma politica que la placa de temperatura.
//   7. Contador de evento persistente en RTC user memory (no toca flash), para
//      que el cliente pueda ordenar una jornada de 11 h aunque la placa se
//      haya reiniciado un par de veces.
//
// SE MANTIENE DE v1 (para calibrar en carretera sin WiFi):
//   - Diagnostico profundo de registros al arranque + forzado de measurement.
//   - Reintento del sensor en background si no aparece en I2C.
//   - Setpoint automatico de 20 lecturas, recalculado cada 20 en reposo.
//   - Maquina de estados ST_IDLE/ST_LOCKED con supresion del rebote inercial
//     (EVENT_LOCKOUT_MS + REST_MS).
//   - Salida CSV a 20 Hz -> app de escritorio, con MODE_SERIAL_CSV 1.
//   - Los 5 simbolos '.', '+', '*', '-', '!' y el formato de la linea CSV NO
//     cambian: la app de escritorio de v1 sigue leyendo identico.
//
// NOTA SOBRE EL ALCANCE: el idea original era "1 lectura por segundo, 30
// lecturas, un publish cada 30 s". A 1 Hz no se puede reconstruir la forma de
// onda de un acelerometro (Nyquist: la vibracion va de 10 a 1000 Hz), asi que
// se sustituyo por eventos con ventana de 1 s a 100 Hz. Es estrictamente
// mejor para el objetivo del cliente administrador y ademas genera trafico
// solo cuando hay algo que reportar.
//
// MODOS (definidos arriba del todo):
//   MODE_SERIAL_CSV 0  -> deteccion por evento, ventana de 200 muestras,
//                          salida por MQTT (o por serie si MQTT_DRY_RUN 1)
//   MODE_SERIAL_CSV 1  -> CSV puro a 20 Hz, sin WiFi (herramienta de v1)
//   MQTT_DRY_RUN   1   -> NO se abre WiFi ni MQTT; cada mensaje que se
//                          publicaria se imprime por serie con su topic y su
//                          payload, byte a byte. Para validar formato y
//                          deteccion sin suscriptor en el broker.
//
// CONTRATO DE DATOS (ver README.md 5.7):
//   <MQTT_CLIENT>/accel/setpoint        {"x":g,"y":g,"z":g}
//   <MQTT_CLIENT>/accel/online          true
//   <MQTT_CLIENT>/accel/status          OK | NO_DETECTADO
//   <MQTT_CLIENT>/accel/event/summary   1 por evento
//   <MQTT_CLIENT>/accel/event/chunk     CHUNKS (5) por evento
//   Los 4 vectores del chunk (z,x,y,dt) tienen CHUNK_N (40) enteros crudos del
//   ADXL345: g = raw/256. El ORDEN importa: z,x,y,dt.
//
// OJO CON EL TIEMPO: los campos "up" (resumen) y "up0" (chunk) son milisegundos
// DESDE EL ARRANQUE de la placa, NO hora de reloj. Para dibujar una jornada de
// 11 h el cliente debe usar el instante de recepcion del mensaje como ancla y
// sumar el "dt" de cada muestra. Cuando se active el broker se anadira la hora
// de reloj (epoch NTP) como campo "ep".
//
// MAPA DE PINES:
//   D1/GPIO5  SCL -> GY-291 SCL
//   D2/GPIO4  SDA -> GY-291 SDA
//   3V3/GND   alimentacion GY-291
// ============================================================================

// 0 = MQTT por evento (modo normal)   1 = solo CSV por serie, SIN WiFi
#define MODE_SERIAL_CSV  0

// Ensayo en seco: 1 = NO se abre WiFi ni MQTT; cada mensaje que se publicaria
// se imprime por serie con su topic y su payload, byte a byte. Sirve para
// validar el formato y la deteccion sin tener un suscriptor en el broker.
// Poner 0 en cuanto haya algo escuchando del otro lado.
#define MQTT_DRY_RUN  1

#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#if MODE_SERIAL_CSV == 0
  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
  extern "C" {
    #include <user_interface.h>
    #include <ping.h>
  }
  #include "secrets.h"
#endif

// ---------------------------------------------------------------------------
// Constantes
// ---------------------------------------------------------------------------
#define BAUD_RATE            115200
#define FW_VERSION          "2"
#define FW_BUILD            "2026-09-30"
#define SENSOR_RANGE         ADXL345_RANGE_2_G
#define SENSOR_DATARATE      ADXL345_DATARATE_100_HZ
#define SAMPLE_INTERVAL_MS   10        // muestreo interno ~100 Hz
#define PRINT_INTERVAL_MS    50        // impresion serial ~20 Hz (modo serie puro)
#define SENSOR_RETRY_MS      1000      // reintento en background cada 1 s

// Setpoint
#define SETPOINT_N           20            // lecturas para el promedio por eje
#define ARROW_THRESHOLD_G    0.20f         // desviacion Z (g) para '+'/'-'
#define SEVERE_FACTOR        1.2f
#define SEVERE_THRESHOLD_G   (ARROW_THRESHOLD_G * SEVERE_FACTOR)  // 0.24g
#define RETURN_G             0.15f         // histeresis de retorno a reposo

// --- Anulacion del rebote (identico a v1) ---
#define EVENT_LOCKOUT_MS     250
#define REST_MS              50

// --- Ventana del evento (nuevo en v2) ---
// PRE_N muestras se mantienen SIEMPRE en un anillo: eso es lo que permite
// enviar tambien lo que ocurrio ANTES del cruce del umbral. Sin este
// historial el cliente recibiria solo la cola del evento.
#define PRE_N                100           // 1 s a 100 Hz
#define POST_N               100           // 1 s a 100 Hz
#define EVENT_N              (PRE_N + POST_N)          // 200
#define EVENT_SPAN_MS        ((PRE_N + POST_N) * SAMPLE_INTERVAL_MS)  // 2000 ms
#define CHUNK_N              40                         // muestras por publish
#define CHUNKS               (EVENT_N / CHUNK_N)        // 5

// Volcado de la ventana por serie en modo MQTT. Son 200 lineas (~0.9 s a 115200)
// pero se imprimen DESPUES de entregar al broker, asi que no retrasan la
// publicacion. Es el camino de rescate en campo: si el broker falla, los datos
// crudos del evento quedan en el portatil que este leyendo la consola.
#define DUMP_EVENT_CSV       1

// ---------------------------------------------------------------------------
// Constantes de red (solo modo MQTT)
// ---------------------------------------------------------------------------
#define MAX_WIFI_RETRIES    120          // 120 x 250 ms = 30 s
#define MAX_MQTT_FAILURES    10
#define MQTT_CONNECT_TIMEOUT 120000UL
#define MQTT_BUF_MAX         1024        // > 870 B: el chunk mas largo
#define WDT_FEED_INTERVAL    4           // s (margen 2x sobre WDTO_8S)
#define STALE_REPORT_MS   900000UL       // 15 min sin publicar nada -> reinicio

// Conectividad ICMP a 8.8.8.8
#define PING_TARGET_IP     0x08080808UL
#define PING_MAX_TRIES         5
#define PING_TIMEOUT_MS   3000UL
#define PING_GAP_MS       2000UL
#define PING_ROUND_MAX_MS 20000UL
#define PING_INTERVAL        30

// ---------------------------------------------------------------------------
// Pines y registros ADXL345
// ---------------------------------------------------------------------------
#define PIN_SDA              4         // D2/GPIO4
#define PIN_SCL              5         // D1/GPIO5

#define ADXL345_I2C_ADDR     0x53
#define REG_DEVID            0x00
#define REG_OFSX             0x1E
#define REG_OFSY             0x1F
#define REG_OFSZ             0x20
#define REG_BW_RATE          0x2C
#define REG_POWER_CTL        0x2D
#define REG_DATA_FORMAT      0x31
#define REG_DATAX0           0x32
#define REG_DATAY0           0x34
#define REG_DATAZ0           0x36

// ---------------------------------------------------------------------------
// Estructura de una muestra de la ventana
//   x, y, z en cuentas crudas del ADXL345 (+-2 g -> 256 LSB/g).
//   dt = ms REALES entre esta muestra y la anterior. El muestreo nominal es de
//   10 ms, pero client.publish() y el stack WiFi lo retrasan; grabar el dt
//   real es lo que permite que el grafico del cliente sea fiel.
// ---------------------------------------------------------------------------
struct sample_t { int16_t x, y, z; uint16_t dt; };   // 8 bytes

// ---------------------------------------------------------------------------
// Estado global
// ---------------------------------------------------------------------------
Adafruit_ADXL345_Unified accel(12345);

bool sensorOnline = false;
bool notDetectedReported = false;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
int16_t rawX = 0, rawY = 0, rawZ = 0;

bool setpointReady = false;
float spX = 0.0f, spY = 0.0f, spZ = 0.0f;
float devZ = 0.0f;                            // v2: solo el eje Z decide
float accSumX = 0.0f, accSumY = 0.0f, accSumZ = 0.0f;
uint16_t accCount = 0;

enum DetectState : uint8_t { ST_IDLE = 0, ST_LOCKED = 1 };
DetectState detectState = ST_IDLE;
char  eventChar = '.';
bool  symbolPending = false;

unsigned long lockUntilMs = 0;
unsigned long restSinceMs = 0;

// --- Ventana del evento ---
sample_t preRing[PRE_N];       // anillo del pasado, siempre rodando
sample_t eventBuf[EVENT_N];    // ventana en curso
uint8_t  ringIdx    = 0;       // proxima escritura en preRing
uint8_t  copyIndex  = 0;       // muestras ya volcadas a eventBuf
bool     collecting = false;   // hay una ventana en curso
uint32_t eventStartMs = 0;
float    eventDevZ   = 0.0f;   // desviacion Z en el instante del cruce
float    eventPeakZ  = 0.0f;   // max |devZ| dentro de la ventana
char     eventSym    = '.';
uint32_t eventCount  = 0;      // eventos detectados desde el arranque

unsigned long lastSampleMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastRetryMs = 0;

// ---------------------------------------------------------------------------
// Estado de red (solo modo MQTT)
// ---------------------------------------------------------------------------
#if MODE_SERIAL_CSV == 0
WiFiClient espClient;
PubSubClient client(espClient);
static char mqttBuf[MQTT_BUF_MAX];      // global: 1 KB en el stack es arriesgado

int mqttPublishFailCount = 0;
int wifiConnectRetries = 0;

typedef struct {
  uint32_t magic;
  uint16_t failureFlag;
} rtcData_t;

volatile bool    pingRoundActive = false;
volatile bool    pingRoundDone = false;
volatile uint32_t pingReplies = 0;
ping_resp        pingLast;
struct ping_option pingOpt;
uint32_t pingNextTryAt = 0;
uint32_t pingRoundStart = 0;
uint32_t lastGoodPublish = 0;

#if MQTT_DRY_RUN
uint8_t  dryMsgs  = 0;      // mensajes "publicados" en el ensayo
uint32_t dryBytes = 0;      // bytes acumulados del ensayo
#endif
#endif

// ---------------------------------------------------------------------------
// Declaraciones
// ---------------------------------------------------------------------------
void blinkLedDummy();
bool readAccel();
void processSample();
void printCharTick();
void runChipDiagnostic();
uint8_t readReg(uint8_t reg);
bool writeReg(uint8_t reg, uint8_t val);
bool readRawAxes();
void pushSample(uint16_t dt);
void beginCapture();
void advanceCapture(uint16_t dt);
void finishCapture();
void publishEvent();
void loadEventCounter();
void saveEventCounter();

#if MODE_SERIAL_CSV == 0
void startWifi();
void startMqtt();
void checkMqtt();
void sendToBroker(const char* topic, const char* message);
void randomizeMac();
void rebootWithNewMac();
void servicePing();
void startPingRound();
bool brokerReachable();
void onPingRecv(void* arg, void* pdata);
void onPingSent(void* arg, void* pdata);
void mqttCallback(char* topic, byte* payload, unsigned int length);
#endif

// ---------------------------------------------------------------------------
// Acceso directo a registros del ADXL345 por I2C (Wire)
// ---------------------------------------------------------------------------
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ADXL345_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((uint8_t)ADXL345_I2C_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL345_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool readRawAxes() {
  Wire.beginTransmission(ADXL345_I2C_ADDR);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADXL345_I2C_ADDR, (uint8_t)6) != 6) return false;
  uint8_t x0 = Wire.read(), x1 = Wire.read();
  uint8_t y0 = Wire.read(), y1 = Wire.read();
  uint8_t z0 = Wire.read(), z1 = Wire.read();
  rawX = (int16_t)((uint16_t)x1 << 8 | x0);
  rawY = (int16_t)((uint16_t)y1 << 8 | y0);
  rawZ = (int16_t)((uint16_t)z1 << 8 | z0);
  return true;
}

// ---------------------------------------------------------------------------
// Diagnostico del chip (identico a v1): se imprime UNA VEZ al arrancar
// ---------------------------------------------------------------------------
static const char* bwRateStr(uint8_t code) {
  switch (code & 0x0F) {
    case 0x0F: return "3200 Hz"; case 0x0E: return "1600 Hz";
    case 0x0D: return "800 Hz";  case 0x0C: return "400 Hz";
    case 0x0B: return "200 Hz";  case 0x0A: return "100 Hz";
    case 0x09: return "50 Hz";   case 0x08: return "25 Hz";
    case 0x07: return "12.5 Hz"; case 0x06: return "6.25 Hz";
    case 0x05: return "3.13 Hz"; case 0x04: return "1.56 Hz";
    case 0x03: return "0.78 Hz"; case 0x02: return "0.39 Hz";
    case 0x01: return "0.20 Hz"; default:   return "0.10 Hz";
  }
}

void runChipDiagnostic() {
  Serial.println();
  Serial.println("=== ADXL345 DIAGNOSTICO PROFUNDO ===");

  uint8_t who  = readReg(REG_DEVID);
  uint8_t pwr  = readReg(REG_POWER_CTL);
  uint8_t dfmt = readReg(REG_DATA_FORMAT);
  uint8_t bwr  = readReg(REG_BW_RATE);
  int8_t ofsx  = (int8_t)readReg(REG_OFSX);
  int8_t ofsy  = (int8_t)readReg(REG_OFSY);
  int8_t ofsz  = (int8_t)readReg(REG_OFSZ);

  Serial.printf("WHO_AM_I    = 0x%02X", who);
  Serial.println(who == 0xE5
    ? "  (OK: 0xE5 es el ADXL345)"
    : "  (ESPERADO 0xE5; otro valor = clon/defecto o mala lectura I2C)");

  Serial.printf("POWER_CTL   = 0x%02X", pwr);
  Serial.println((pwr & 0x08)
    ? "  (bit3=1 -> measurement mode)"
    : "  (bit3=0 -> NO en measurement mode / STANDBY)");

  Serial.printf("DATA_FORMAT = 0x%02X", dfmt);
  switch (dfmt & 0x03) {
    case 0x00: Serial.println("  (rango +-2g)");  break;
    case 0x01: Serial.println("  (rango +-4g)");  break;
    case 0x02: Serial.println("  (rango +-8g)");  break;
    default:   Serial.println("  (rango +-16g)"); break;
  }

  Serial.printf("BW_RATE     = 0x%02X  (code 0x%02X -> %s)\n", bwr, bwr & 0x0F, bwRateStr(bwr));
  Serial.printf("OFSX/OFSY/OFSZ = %d/%d/%d\n", ofsx, ofsy, ofsz);

  Serial.println("-- Forzando measurement mode: escribiendo 0x08 en POWER_CTL --");
  bool wrote = writeReg(REG_POWER_CTL, 0x08);
  delay(50);
  uint8_t pwr2 = readReg(REG_POWER_CTL);
  if (wrote && (pwr2 & 0x08)) {
    Serial.println(F("[POWER_CTL] OK: 0x08 persiste -> measurement mode ACTIVO"));
  } else {
    Serial.printf("[POWER_CTL] NO se mantiene 0x08 (escritura=%d, releido=0x%02X) -> problema de escritura I2C (revisar cableado/soldadura)\n",
                  wrote ? 1 : 0, pwr2);
  }

  if (readRawAxes()) {
    Serial.printf("PRIMERA MUESTRA rawX=%6d rawY=%6d rawZ=%6d\n", rawX, rawY, rawZ);
  } else {
    Serial.println(F("[I2C] No se pudo leer DATAX0..DATAZ1 (revisar SDA/SCL)"));
  }
  Serial.println(F("=== FIN DIAGNOSTICO ==="));
  Serial.println();
}

// ---------------------------------------------------------------------------
// Lectura del acelerometro: burst de 6 bytes -> g = raw / 256.0f (+-2g)
// ---------------------------------------------------------------------------
bool readAccel() {
  if (readRawAxes()) {
    sensorOnline = true;
    gx = rawX / 256.0f;
    gy = rawY / 256.0f;
    gz = rawZ / 256.0f;
    return true;
  }
  sensorOnline = false;
  return false;
}

// ---------------------------------------------------------------------------
// Contador de evento persistente en RTC user memory.
// Las 4 palabras de 32 bits del RTC user memory del ESP8266 se reparten asi:
//   palabras 0-1 -> rtcData_t (8 bytes: magic + failureFlag)
//   palabra  2   -> contador de evento
//   palabra  3   -> magic del contador ("EVT1"), para validar que la palabra 2
//                   no sea memoria sin inicializar
// No toca la flash: sobrevive a reinicios por red (RTC alimentado) pero se
// pierde si se corta la luz. Es aceptable: el cliente solo necesita IDs
// ordenados, no continuidad absoluta.
// ---------------------------------------------------------------------------
#define RTC_EVT_MAGIC 0x45565431UL        // 'E','V','T','1'
#define RTC_EVT_MAX   100000000UL         // por encima, es basura seguro

void loadEventCounter() {
  uint32_t v = 0, magic = 0;
  ESP.rtcUserMemoryRead(2, &v, 1);
  ESP.rtcUserMemoryRead(3, &magic, 1);
  if (magic == RTC_EVT_MAGIC && v <= RTC_EVT_MAX) {
    eventCount = v;
  } else {
    eventCount = 0;
    Serial.println(F("[RTC] contador sin validar: empieza en 0"));
  }
}

void saveEventCounter() {
  ESP.rtcUserMemoryWrite(2, &eventCount, 1);
  uint32_t magic = RTC_EVT_MAGIC;
  ESP.rtcUserMemoryWrite(3, &magic, 1);
}

// ---------------------------------------------------------------------------
// Anillo del pasado. Se escribe en CADA muestra, evento o no: es lo que
// permite enviar las 100 muestras previas al cruce.
// ---------------------------------------------------------------------------
void pushSample(uint16_t dt) {
  preRing[ringIdx].x  = rawX;
  preRing[ringIdx].y  = rawY;
  preRing[ringIdx].z  = rawZ;
  preRing[ringIdx].dt = dt;
  ringIdx = (uint8_t)((ringIdx + 1) % PRE_N);
}

// ---------------------------------------------------------------------------
// Arranque de la ventana: se vuelca el anillo del pasado. La ultima muestra
// del anillo es precisamente la que cruzo el umbral, asi que queda incluida
// y no hay que duplicarla.
// ---------------------------------------------------------------------------
void beginCapture() {
  memcpy(eventBuf, preRing, sizeof(preRing));
  copyIndex    = PRE_N;
  collecting   = true;
  eventStartMs = millis();
  eventDevZ    = devZ;
  eventPeakZ   = fabsf(devZ);
  eventSym     = eventChar;
  eventCount++;
  saveEventCounter();
}

// ---------------------------------------------------------------------------
// Avance de la ventana: se anaden POST_N muestras mas y, al llenarse, se
// publica. El pico se mide sobre TODA la ventana para que el resumen digas
// cuanto fue el excursion maximo, no solo el valor del cruce.
// ---------------------------------------------------------------------------
void advanceCapture(uint16_t dt) {
  if (!collecting) return;

  float d = fabsf(devZ);
  if (d > eventPeakZ) eventPeakZ = d;

  eventBuf[copyIndex].x  = rawX;
  eventBuf[copyIndex].y  = rawY;
  eventBuf[copyIndex].z  = rawZ;
  eventBuf[copyIndex].dt = dt;
  copyIndex++;

  if (copyIndex < EVENT_N) return;

  collecting = false;
  finishCapture();
}

// ---------------------------------------------------------------------------
// Cierre de la ventana: MQTT primero, volcado por serie despues. El orden
// importa: imprimir 200 lineas por serie a 115200 baud tarda ~0.9 s y retrasaria
// la entrega al broker.
// ---------------------------------------------------------------------------
void finishCapture() {
#if MODE_SERIAL_CSV == 0
  publishEvent();
#endif

  Serial.printf("[EVENTO] id=%lu sym=%c spz=%.2fg dev=%.2fg pico=%.2fg "
                "ventana=%ums (pre=%d post=%d)\n",
                (unsigned long)eventCount, eventSym, spZ, eventDevZ, eventPeakZ,
                (unsigned)EVENT_SPAN_MS, PRE_N, POST_N);

#if DUMP_EVENT_CSV
  // El simbolo va en la muestra QUE DISPARO el evento, que es eventBuf[PRE_N]
  // (beginCapture() deja copyIndex=PRE_N y advanceCapture() escribe ahi la
  // muestra actual). Ponerlo en PRE_N-1 marcaba la muestra equivocada.
  for (uint8_t i = 0; i < EVENT_N; i++) {
    Serial.printf("EV,%lu,%d,%d,%d,%u,%c\n",
                  (unsigned long)eventStartMs,
                  eventBuf[i].x, eventBuf[i].y, eventBuf[i].z,
                  eventBuf[i].dt,
                  (i == PRE_N) ? eventSym : '.');
  }
#endif
}

void blinkLedDummy() {}

// ---------------------------------------------------------------------------
// Publicacion MQTT de la ventana: 1 resumen + 5 trozos de 40 muestras.
// Los valores viajan como enteros CRUDOS del ADXL345 (no float): 3-4
// caracteres en vez de 6-7, sin perder nada porque es la resolucion nativa
// del sensor. El cliente divide por 256 para obtener g.
// ---------------------------------------------------------------------------
#if MODE_SERIAL_CSV == 0
void publishEvent() {
#if !MQTT_DRY_RUN
  if (!client.connected()) {
    Serial.println(F("[MQTT] sin broker: evento descartado (politica v2)"));
    return;
  }
#endif

#if MQTT_DRY_RUN
  dryMsgs = 0;
  dryBytes = 0;
  unsigned long t0 = millis();
#endif

  char sum[200];
  snprintf(sum, sizeof(sum),
           "{\"id\":%lu,\"sym\":\"%c\",\"spz\":%.2f,\"dev\":%.2f,\"peak\":%.2f,"
           "\"n\":%u,\"ms\":%u,\"up\":%lu,\"count\":%lu}",
           (unsigned long)eventCount, eventSym, spZ, eventDevZ, eventPeakZ,
           (unsigned)EVENT_N, (unsigned)EVENT_SPAN_MS,
           (unsigned long)eventStartMs, (unsigned long)eventCount);
  sendToBroker("accel/event/summary", sum);

  for (uint8_t c = 0; c < CHUNKS; c++) {
uint8_t base = (uint8_t)(c * CHUNK_N);
    uint16_t n = snprintf(mqttBuf, MQTT_BUF_MAX,
                          "{\"id\":%lu,\"i\":%u,\"n\":%u,\"up0\":%lu",
                          (unsigned long)eventCount, c, CHUNKS,
                          (unsigned long)eventStartMs);

// Cada campo lleva su coma delante y cierra su propio array. Asi el orden de
// los campos es libre y no hay que llevar la cuenta de cual va primero.
#define APPEND_FIELD(NAME, EXPR)                                              \
    do {                                                                      \
      if (n >= MQTT_BUF_MAX - 8) break;                                      \
      n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n,                            \
                    ",\"%s\":[", NAME);                                        \
      for (uint8_t k = 0; k < CHUNK_N; k++) {                                 \
        n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n, "%s%d",                 \
                      k ? "," : "", (int)(EXPR));                             \
        if (n >= MQTT_BUF_MAX) { n = (uint16_t)(MQTT_BUF_MAX - 1); break; }   \
      }                                                                       \
      n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n, "]");                      \
    } while (0)

    APPEND_FIELD("z", eventBuf[base + k].z);
    APPEND_FIELD("x", eventBuf[base + k].x);
    APPEND_FIELD("y", eventBuf[base + k].y);
    APPEND_FIELD("dt", eventBuf[base + k].dt);
#undef APPEND_FIELD

    n += snprintf(mqttBuf + n, MQTT_BUF_MAX - n, "}");
    sendToBroker("accel/event/chunk", mqttBuf);
  }

#if MQTT_DRY_RUN
  Serial.printf("[MQTT-OUT] === evento %lu: %u mensajes, %lu B, %ums "
                "(ENSAYO EN SECO: nada enviado al broker) ===\n",
                (unsigned long)eventCount, dryMsgs,
                (unsigned long)dryBytes, (unsigned long)(millis() - t0));
#endif
}
#endif

// ---------------------------------------------------------------------------
// Procesado de una muestra (~100 Hz)
// ---------------------------------------------------------------------------
void processSample() {
  if (!setpointReady) {
    accSumX += gx; accSumY += gy; accSumZ += gz;
    accCount++;
    Serial.print('.');
    if (accCount >= SETPOINT_N) {
      spX = accSumX / SETPOINT_N;
      spY = accSumY / SETPOINT_N;
      spZ = accSumZ / SETPOINT_N;
      setpointReady = true;
      accSumX = accSumY = accSumZ = 0.0f;
      accCount = 0;
      Serial.println();
      Serial.printf("setpoint establecido: X=%.2f Y=%.2f Z=%.2f\n", spX, spY, spZ);
#if MODE_SERIAL_CSV == 0
      {
        static char spBuf[64];
        snprintf(spBuf, sizeof(spBuf),
                 "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}", spX, spY, spZ);
        sendToBroker("accel/setpoint", spBuf);
      }
#endif
    }
    return;
  }

  // v2: la desviacion que decide es SOLO la de Z (gx, gy y spX/spY se siguen
  // calculando para la linea CSV y para el setpoint, pero no disparan evento).
  devZ        = gz - spZ;
  bool eventActive = fabsf(devZ) > ARROW_THRESHOLD_G;
  bool allResting  = (fabsf(devZ) <= RETURN_G);

  switch (detectState) {

    case ST_IDLE:
      if (eventActive) {
        // Sentido y magnitud de la desviacion de Z respecto del setpoint:
        //   devZ >=  +SEVERE  -> '*'   elevacion severa
        //   devZ >   +ARROW   -> '+'   elevacion leve
        //   devZ <= -SEVERE   -> '!'   caida severa
        //   devZ <   -ARROW   -> '-'   caida leve
        eventChar = (devZ >=  SEVERE_THRESHOLD_G) ? '*'
                  : (devZ >   ARROW_THRESHOLD_G)  ? '+'
                  : (devZ <= -SEVERE_THRESHOLD_G) ? '!'
                  :                                '-';
        symbolPending = true;
        detectState = ST_LOCKED;
        lockUntilMs = millis() + EVENT_LOCKOUT_MS;
        restSinceMs = 0;
        beginCapture();          // arranca la ventana del evento
      } else {
        accSumX += gx; accSumY += gy; accSumZ += gz;
        accCount++;
        if (accCount >= SETPOINT_N) {
          spX = accSumX / SETPOINT_N;
          spY = accSumY / SETPOINT_N;
          spZ = accSumZ / SETPOINT_N;
          accSumX = accSumY = accSumZ = 0.0f;
          accCount = 0;
        }
      }
      break;

    case ST_LOCKED:
      // Sin simbolos adicionales (rebote). El desbloqueo exige LAS TRES
      // condiciones:
      //   (a) lockout temporal ya cumplido  -> anula el rebote ~100-200 ms
      //   (b) reposo sostenido REST_MS      -> no un instante aislado
      //   (c) ventana del evento TERMINADA  -> sin esto, un evento a los 300 ms
      //       podria pisar eventBuf mientras la ventana anterior sigue
      //       llenandose (la ventana dura 2 s, mas que lockout+reposo).
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
      break;
  }
}

// ---------------------------------------------------------------------------
// Linea CSV por tick a 20 Hz (solo modo MODE_SERIAL_CSV == 1)
//   t,X,Y,Z,setX,setY,setZ,sym  -- formato IDENTICO a v1
// ---------------------------------------------------------------------------
void printCharTick() {
  char c = '.';
  if (symbolPending) {
    c = eventChar;
    symbolPending = false;
  }
  Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%c\n",
                millis(), gx, gy, gz, spX, spY, spZ, c);
}

// ═══════════════════════════════════════════════════════════════════════════
//  RED (solo modo MQTT)
// ═══════════════════════════════════════════════════════════════════════════
#if MODE_SERIAL_CSV == 0

void randomizeMac() {
  WiFi.disconnect();
  uint8_t mac[6]; WiFi.macAddress(mac);
  uint8_t old = mac[5];
  mac[5] = random(0x01, 0xFE);
  wifi_set_macaddr(STATION_IF, mac);
  Serial.printf("MAC: %02x -> %02x\n", old, mac[5]);
}

void rebootWithNewMac() {
  rtcData_t rtc; rtc.magic = 0xDEAD; rtc.failureFlag = 1;
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, 2);
  delay(100); ESP.restart();
}

void startWifi() {
  delay(10); Serial.println();
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiConnectRetries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(250); Serial.print('.'); ESP.wdtFeed();
    wifiConnectRetries++;
    if (wifiConnectRetries >= MAX_WIFI_RETRIES) {
      wl_status_t st = WiFi.status();
      Serial.printf("\nWiFi timeout. status=%d", (int)st);
      switch (st) {
        case WL_NO_SSID_AVAIL:   Serial.print(F(" WL_NO_SSID_AVAIL (red no visible)")); break;
        case WL_CONNECT_FAILED:  Serial.print(F(" WL_CONNECT_FAILED (contrasena incorrecta)")); break;
        case WL_CONNECTION_LOST: Serial.print(F(" WL_CONNECTION_LOST (se cae el enlace)")); break;
        case WL_DISCONNECTED:    Serial.print(F(" WL_DISCONNECTED")); break;
        default:                 Serial.print(F(" (sin asociar)"));
      }
      Serial.println(F("\nRebooting..."));
      rebootWithNewMac();
    }
  }
  Serial.println();
  Serial.print("Connected: "); Serial.println(WiFi.SSID());
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d dBm   MAC: ", WiFi.RSSI());
  uint8_t mac[6]; WiFi.macAddress(mac);
  for (int i = 0; i < 6; i++) Serial.printf("%02X%s", mac[i], i < 5 ? ":" : "\n");
  wifiConnectRetries = 0;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[64];
  unsigned int n = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;
  memcpy(msg, payload, n);
  msg[n] = '\0';
  Serial.print(F("[MQTT RX] ")); Serial.print(topic);
  Serial.print(" -> "); Serial.println(msg);

  // Recalibracion en caliente: util con el bus en marcha, para fijar el
  // setpoint en el angulo real de montaje sin reiniciar el equipo.
  if (strcmp(msg, "RECAL") == 0) {
    setpointReady = false;
    accSumX = accSumY = accSumZ = 0.0f;
    accCount = 0;
    Serial.println(F("[MQTT] recalibrando setpoint..."));
  }
}

void startMqtt() {
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setBufferSize(MQTT_BUF_MAX);   // el chunk completo no entra en 256
  client.setCallback(mqttCallback);
  unsigned long start = millis();
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(MQTT_CLIENT, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      mqttPublishFailCount = 0;
      lastGoodPublish = millis();
      char sub[128]; snprintf(sub, sizeof(sub), "%s/#", MQTT_CLIENT);
      if (client.subscribe(sub)) Serial.println(F("Subscribed to <client>/#"));
      else Serial.println(F("!! MQTT subscribe failed"));
      sendToBroker("accel/online", "true");
      sendToBroker("accel/status", sensorOnline ? "OK" : "NO_DETECTADO");
    } else {
      if (client.state() == 5) {
        Serial.println(F("MQTT auth error. Check credentials/ClientId"));
      } else {
        Serial.print("MQTT error: "); Serial.println(client.state());
      }
      if (millis() - start > MQTT_CONNECT_TIMEOUT) {
        Serial.println(F("MQTT timeout. Rebooting..."));
        rebootWithNewMac();
      }
      // delay() enzamizados: el WDT real corta a los 8 s
      for (uint8_t i = 0; i < 10; i++) { delay(1000); ESP.wdtFeed(); }
    }
  }
}

void checkMqtt() {
  if (client.connected()) return;
  Serial.println(F("MQTT disconnected, reconnecting..."));
  lastGoodPublish = millis();
  startMqtt();
}

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
      Serial.printf("!! publish fallo: %s (%u B)\n", topic, strlen(message));
      mqttPublishFailCount++;
      if (mqttPublishFailCount >= MAX_MQTT_FAILURES) {
        Serial.println(F("Too many MQTT failures. Rebooting..."));
        rebootWithNewMac();
      }
    }
  } else {
    mqttPublishFailCount++;
    if (mqttPublishFailCount >= MAX_MQTT_FAILURES) {
      Serial.println(F("MQTT offline. Rebooting..."));
      rebootWithNewMac();
    }
  }
#endif
}

// --- ICMP real a 8.8.8.8 ---
void onPingRecv(void* arg, void* pdata) {
  (void)arg;
  ping_resp* r = (ping_resp*)pdata;
  if (r == NULL) return;
  pingLast = *r;
  if (r->ping_err == 0) pingReplies++;
  if (r->total_count >= PING_MAX_TRIES || r->timeout_count >= PING_MAX_TRIES) {
    pingRoundDone = true;
  }
}

void onPingSent(void* arg, void* pdata) { (void)arg; (void)pdata; }

void startPingRound() {
  pingReplies = 0;
  pingRoundDone = false;
  memset(&pingLast, 0, sizeof(pingLast));

  pingOpt.count       = PING_MAX_TRIES;
  pingOpt.ip          = PING_TARGET_IP;
  pingOpt.coarse_time = PING_GAP_MS;
  pingOpt.recv_function = onPingRecv;
  pingOpt.sent_function = onPingSent;
  pingOpt.reverse     = NULL;

  if (ping_start(&pingOpt)) {
    pingRoundActive = true;
    pingRoundStart  = millis();
  } else {
    Serial.println(F("ping_start fallo"));
  }
  ESP.wdtFeed();
}

void servicePing() {
  uint32_t now = millis();
  uint32_t elapsed = now - pingRoundStart;

  if (pingRoundActive) {
    bool roundOver = pingRoundDone;
    if (!roundOver && elapsed >= PING_ROUND_MAX_MS) roundOver = true;
    if (!roundOver) return;

    pingRoundActive = false;
    bool ok = (pingReplies > 0);
    if (ok) {
      Serial.printf("Ping 8.8.8.8: OK (%lu respuesta(s))\n", (unsigned long)pingReplies);
      pingNextTryAt = now + (PING_INTERVAL * 1000UL);
      return;
    }

    Serial.printf("Ping 8.8.8.8: FALLO (0/%d respondieron)\n", PING_MAX_TRIES);
    if (brokerReachable()) {
      Serial.println(F("Broker MQTT responde: no se reinicia (ICMP filtrado)"));
      pingNextTryAt = now + (PING_INTERVAL * 1000UL);
    } else {
      Serial.println(F("Broker MQTT tampoco responde. Reiniciando..."));
      rebootWithNewMac();
    }
    return;
  }

  if ((int32_t)(now - pingNextTryAt) >= 0) startPingRound();
}

bool brokerReachable() {
  WiFiClient tc;
  tc.setTimeout(2000);
  bool ok = tc.connect(MQTT_BROKER, MQTT_PORT);
  if (ok) tc.stop();
  ESP.wdtFeed();
  return ok;
}

#endif  // MODE_SERIAL_CSV == 0

// ═══════════════════════════════════════════════════════════════════════════
//  setup() / loop()
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  ESP.wdtEnable(WDTO_8S);

  Serial.begin(BAUD_RATE);
  delay(300);
  Serial.println();
#if MODE_SERIAL_CSV == 0
  #if MQTT_DRY_RUN
    Serial.println(F("=== BusAccel-8266-v2 (evento eje Z, ENSAYO EN SECO) ==="));
    Serial.println(F("Sin WiFi y sin MQTT: cada mensaje se imprime por serie."));
  #else
    Serial.println(F("=== BusAccel-8266-v2 (MQTT por evento, eje Z) ==="));
  #endif
#else
  Serial.println(F("=== BusAccel-8266-v2 (modo serie puro, SIN WiFi) ==="));
  Serial.println(F("Formato CSV: t,X,Y,Z,setX,setY,setZ,sym"));
#endif
  Serial.print("[FW] version "); Serial.println(FW_VERSION);
  Serial.print("[FW] build ");   Serial.println(FW_BUILD);

  memset(preRing, 0, sizeof(preRing));
  memset(eventBuf, 0, sizeof(eventBuf));

#if MODE_SERIAL_CSV == 0
  loadEventCounter();
  Serial.printf("[RTC] contador de eventos: %lu\n", (unsigned long)eventCount);

  rtcData_t rtc;
  ESP.rtcUserMemoryRead(0, (uint32_t*)&rtc, 2);
  if (rtc.magic == 0xDEAD && rtc.failureFlag == 1) {
    Serial.println(F("Failure reboot detected. Randomizing MAC..."));
    randomizeMac();
    rtc.failureFlag = 0;
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, 2);
  }
  wifiConnectRetries = 0;
  mqttPublishFailCount = 0;
  pingRoundActive = false;
  pingNextTryAt = 0;
#endif

  Wire.begin(PIN_SDA, PIN_SCL);

  if (accel.begin()) {
    accel.setRange(SENSOR_RANGE);
    accel.setDataRate(SENSOR_DATARATE);
    sensorOnline = true;
    Serial.println(F("[ADXL345] OK (begin() correcto)"));
  } else {
    sensorOnline = false;
    notDetectedReported = true;
    Serial.println(F("[ADXL345] NO DETECTADO (revisar cableado VCC/GND/SDA/SCL)"));
  }

  runChipDiagnostic();
  ESP.wdtFeed();

#if MODE_SERIAL_CSV == 0
#if !MQTT_DRY_RUN
  startWifi();
  startMqtt();
#endif
  Serial.printf("Umbral Z: leve %.2fg | severo %.2fg | retorno %.2fg\n",
                (double)ARROW_THRESHOLD_G, (double)SEVERE_THRESHOLD_G, (double)RETURN_G);
  Serial.printf("Ventana: %d antes + %d despues = %d muestras ~ %ums | %d publishes\n",
                PRE_N, POST_N, EVENT_N, (unsigned)EVENT_SPAN_MS, CHUNKS + 1);
  Serial.print  ("Broker (si se activa MQTT_DRY_RUN=0): ");
  Serial.println(MQTT_BROKER);
  Serial.println("Golpea/mueve el sensor: cada evento imprime topic + payload.");
  Serial.println("Adquiriendo setpoint (20 lecturas en reposo)...");
#else
  Serial.println(F("Adquiriendo setpoint (20 lecturas en reposo)..."));
#endif
}

void loop() {
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

  // Muestreo interno a ~100 Hz
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    uint16_t dt = (uint16_t)(now - lastSampleMs);
    lastSampleMs = now;
    if (readAccel()) {
      pushSample(dt);       // anillo del pasado (siempre, evento o no)
      processSample();      // deteccion -> puede abrir una ventana
      advanceCapture(dt);   // si hay ventana abierta, la sigue llenando
    }
  }

#if MODE_SERIAL_CSV == 1
  // Impresion serial a ~20 Hz: UNA linea CSV por tick
  if (setpointReady && (now - lastPrintMs >= PRINT_INTERVAL_MS)) {
    lastPrintMs = now;
    printCharTick();
  }
#endif

  // Reintento en background si el sensor no responde
  if (!sensorOnline && (now - lastRetryMs >= SENSOR_RETRY_MS)) {
    lastRetryMs = now;
    if (accel.begin()) {
      accel.setRange(SENSOR_RANGE);
      accel.setDataRate(SENSOR_DATARATE);
      sensorOnline = true;
      notDetectedReported = false;
      Serial.println(F("[ADXL345] OK"));
    } else if (!notDetectedReported) {
      notDetectedReported = true;
      Serial.println(F("[ADXL345] NO DETECTADO (revisar cableado VCC/GND/SDA/SCL)"));
    }
  }
}
