// ============================================================================
// BusAccel-8266-v3 — PUBLICACION MQTT DEL PICO DEL EVENTO (eje Z), version 3
// Placa: NodeMCU ESP8266 (ESP-12E)  |  FQBN: esp8266:esp8266:nodemcuv2
//
// Sensor: GY-291 (ADXL345) via I2C (rango +-2g, 256 LSB/g = raw/256.0f)
//   SDA -> D2/GPIO4 , SCL -> D1/GPIO5  (Wire.begin(4,5))
//
// QUE CAMBIA RESPECTO DE v2 (BusAccel-8266-v2):
//   1. YA NO se transmite la ventana completa de 200 muestras. Desaparecen los
//      5 mensajes MQTT de chunk (accel/event/chunk) y desaparecen las 200 lineas
//      "EV," por el monitor serie (DUMP_EVENT_CSV).
//   2. Se publica UN UNICO mensaje compacto por evento con el PICO
//      (max |devZ| de toda la ventana, pre + post) al topic nuevo
//      <MQTT_CLIENT>/accel/event/peak.
//   3. La ventana se sigue capturando y procesando INTERNAMENTE (200 muestras a
//      100 Hz: 1 s antes + 1 s despues) pero NO se almacena completa: el pico de
//      la parte previa se mide escaneando el anillo preRing al disparar, y el de
//      la parte posterior se actualiza muestra a muestra.
//   4. Se ahorra RAM: se elimina eventBuf (1600 B) y el buffer MQTT baja de
//      1024 a 384 B (el payload maximo pasa de ~902 B a ~150 B). ~2240 B menos.
//
// POR QUE: la ventana cruda saturaba el puerto serie (~0.9 s por evento a
// 115200 baud) y el broker con 5 mensajes por evento. Para el objetivo del
// cliente (saber CUANTO golpeo el bus) alcanza con el pico.
//
// CORRECCION DE BUGS DE v2:
//   - bug 1: el contador RTC podia arrancar con un valor basura (>RTC_EVT_MAX)
//     porque loadEventCounter() corria DESPUES de imprimir. En v3 se carga antes
//     y la validacion se refuerza (magic 'EVT1' Y v<=RTC_EVT_MAX Y v!=0).
//   - bug 2: el pico se calcula sobre datos reales del eje Z (no hay arrays
//     serializados ni "z":[] vacios).
//
// SE MANTIENE DE v2 (arquitectura intacta):
//   - Deteccion SOLO sobre eje Z (devZ = gz - spZ), setpoint de 20 lecturas,
//     recalculado cada 20 muestras en reposo.
//   - Simbolos '.', '+', '*', '-', '!' y umbrales 0.20 g / severo 0.24 g /
//     retorno 0.15 g.
//   - Maquina de estados ST_IDLE/ST_LOCKED con EVENT_LOCKOUT_MS + REST_MS y la
//     tercera condicion de desbloqueo !collecting.
//   - Muestreo ~100 Hz sin delay() en el loop; orden pushSample -> processSample
//     -> advanceCapture.
//   - Modos MODE_SERIAL_CSV 0/1 y MQTT_DRY_RUN 0/1.
//   - Red: WiFi STA con reintentos, PubSubClient, ping ICMP a 8.8.8.8 con
//     fallback brokerReachable(), reinicio con MAC aleatoria, WDTO_8S.
//   - Diagnostico profundo de registros + forzado de measurement.
//   - Reintento del sensor en background si no aparece en I2C.
//   - Contador de evento persistente en RTC user memory (palabras 2-3, magic 'EVT1').
//   - Suscripcion a <MQTT_CLIENT>/#; payload RECAL recalibra el setpoint.
//   - Credenciales en secrets.h (no versionado); secrets.example.h de plantilla.
//
// MODOS (definidos arriba del todo):
//   MODE_SERIAL_CSV 0  -> deteccion por evento, 1 mensaje de pico por evento
//                          (por MQTT, o por serie si MQTT_DRY_RUN 1)
//   MODE_SERIAL_CSV 1  -> CSV puro a 20 Hz, sin WiFi (herramienta de v1/v2)
//   MQTT_DRY_RUN   1   -> NO se abre WiFi ni MQTT; cada mensaje que se
//                          publicaria se imprime por serie con su topic y su
//                          payload. Validacion sin suscriptor en el broker.
//
// CONTRATO DE DATOS (v3):
//   <MQTT_CLIENT>/accel/setpoint   {"x":g,"y":g,"z":g}
//   <MQTT_CLIENT>/accel/online     true
//   <MQTT_CLIENT>/accel/status     OK | NO_DETECTADO
//   <MQTT_CLIENT>/accel/event/peak {"id":..,"sym":"*","spz":..,"dev":..,
//                                   "peak":..,"n":200,"ms":2000,"up":..}
//   (los topics accel/event/summary y accel/event/chunk YA NO EXISTEN)
//
// NOTA SOBRE EL TIEMPO: el campo "up" son milisegundos DESDE EL ARRANQUE en el
// instante del DISPARO (no hora de reloj). No hay NTP/RTC de hora.
//
// MAPA DE PINES:
//   D1/GPIO5  SCL -> GY-291 SCL
//   D2/GPIO4  SDA -> GY-291 SDA
//   3V3/GND   alimentacion GY-291 (NUNCA 5 V)
//   TX/GPIO1  UART -> monitor serie / app (115200 baud)
// ============================================================================

// 0 = MQTT por evento (modo normal)   1 = solo CSV por serie, SIN WiFi
#define MODE_SERIAL_CSV  0

// Ensayo en seco: 1 = NO se abre WiFi ni MQTT; cada mensaje que se publicaria
// se imprime por serie con su topic y su payload. Poner 0 para MQTT real.
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
#define FW_VERSION          "3"
#define FW_BUILD            "2026-10-08"
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

// --- Anulacion del rebote (identico a v1/v2) ---
#define EVENT_LOCKOUT_MS     250
#define REST_MS              50

// --- Ventana del evento (se conservan los tamanios de v2) ---
// PRE_N muestras se mantienen SIEMPRE en un anillo: es lo que permite medir el
// pico de lo que ocurrio ANTES del cruce. La ventana no se almacena completa:
// el pico se calcula en linea.
#define PRE_N                100           // 1 s a 100 Hz
#define POST_N               100           // 1 s a 100 Hz
#define EVENT_N              (PRE_N + POST_N)          // 200
#define EVENT_SPAN_MS        ((PRE_N + POST_N) * SAMPLE_INTERVAL_MS)  // 2000 ms

// ---------------------------------------------------------------------------
// Constantes de red (solo modo MQTT)
// ---------------------------------------------------------------------------
#define MAX_WIFI_RETRIES    120          // 120 x 250 ms = 30 s
#define MAX_MQTT_FAILURES    10
#define MQTT_CONNECT_TIMEOUT 120000UL
#define MQTT_BUF_MAX         384         // v3: payload de pico ~150 B (v2: 1024)
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
// Estructura de una muestra del anillo
//   x, y, z en cuentas crudas del ADXL345 (+-2 g -> 256 LSB/g).
//   dt = ms REALES entre esta muestra y la anterior.
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
float devZ = 0.0f;                            // solo el eje Z decide
float accSumX = 0.0f, accSumY = 0.0f, accSumZ = 0.0f;
uint16_t accCount = 0;

enum DetectState : uint8_t { ST_IDLE = 0, ST_LOCKED = 1 };
DetectState detectState = ST_IDLE;
char  eventChar = '.';
bool  symbolPending = false;

unsigned long lockUntilMs = 0;
unsigned long restSinceMs = 0;

// --- Ventana del evento (v3: sin eventBuf, pico en linea) ---
sample_t preRing[PRE_N];       // anillo del pasado, siempre rodando
uint8_t  ringIdx    = 0;       // proxima escritura en preRing
uint16_t postCount  = 0;       // muestras POST ya contadas
bool     collecting = false;   // hay una ventana en curso
uint32_t eventStartMs = 0;
float    eventDevZ   = 0.0f;   // desviacion Z en el instante del cruce
float    eventPeakZ  = 0.0f;   // max |devZ| de la ventana (pre + post)
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
// I-002: se elimino el buffer global mqttBuf[MQTT_BUF_MAX] (384 B); el propio
// PubSubClient reserva su buffer con client.setBufferSize(MQTT_BUF_MAX).

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
#else
unsigned long lastStaleCheckMs = 0;   // I-003: ultima comprobacion de stale publish
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
void publishPeakEvent();
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
// Diagnostico del chip (identico a v1/v2): se imprime UNA VEZ al arrancar
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
//   palabras 0-1 -> rtcData_t (magic + failureFlag)
//   palabra  2   -> contador de evento
//   palabra  3   -> magic del contador ("EVT1")
// No toca la flash: sobrevive a reinicios por red (RTC alimentado) pero se
// pierde si se corta la luz. Validacion reforzada (bug 1 de v2).
// ---------------------------------------------------------------------------
#define RTC_EVT_MAGIC 0x45565431UL        // 'E','V','T','1'
#define RTC_EVT_MAX   100000000UL         // por encima, es basura seguro

void loadEventCounter() {
  uint32_t v = 0, magic = 0;
  // I-001: la API espera el TAMANIO EN BYTES, no un numero de palabras.
  ESP.rtcUserMemoryRead(2, &v, sizeof(v));
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

void saveEventCounter() {
  // I-001: tamano en BYTES (4 para el contador y su magic).
  ESP.rtcUserMemoryWrite(2, &eventCount, sizeof(eventCount));
  uint32_t magic = RTC_EVT_MAGIC;
  ESP.rtcUserMemoryWrite(3, &magic, sizeof(magic));
}

// ---------------------------------------------------------------------------
// Anillo del pasado. Se escribe en CADA muestra, evento o no: es lo que
// permite medir el pico de lo ocurrido antes del cruce.
// ---------------------------------------------------------------------------
void pushSample(uint16_t dt) {
  preRing[ringIdx].x  = rawX;
  preRing[ringIdx].y  = rawY;
  preRing[ringIdx].z  = rawZ;
  preRing[ringIdx].dt = dt;
  ringIdx = (uint8_t)((ringIdx + 1) % PRE_N);
}

// ---------------------------------------------------------------------------
// Arranque de la ventana: se escanea el anillo del pasado para medir el pico
// inicial, en vez de copiarlo (ya no se almacena la ventana completa).
// ---------------------------------------------------------------------------
void beginCapture() {
  eventDevZ  = devZ;
  eventSym   = eventChar;
  eventPeakZ = fabsf(devZ);
  for (uint8_t i = 0; i < PRE_N; i++) {
    float d = fabsf(preRing[i].z / 256.0f - spZ);
    if (d > eventPeakZ) eventPeakZ = d;
  }
  postCount    = 0;
  collecting   = true;
  eventStartMs = millis();
  eventCount++;
  saveEventCounter();          // consume id al abrir (ids sin huecos)
}

// ---------------------------------------------------------------------------
// Avance de la ventana: actualiza el pico con cada muestra POST y cuenta. Al
// llegar a POST_N cierra la ventana. No almacena muestras.
// ---------------------------------------------------------------------------
void advanceCapture(uint16_t dt) {
  (void)dt;
  if (!collecting) return;

  float d = fabsf(devZ);
  if (d > eventPeakZ) eventPeakZ = d;

  postCount++;
  if (postCount >= POST_N) {
    collecting = false;
    finishCapture();
  }
}

// ---------------------------------------------------------------------------
// Cierre de la ventana: MQTT primero, UNA linea serie despues. Sin volcado de
// la ventana (clave de v3).
// ---------------------------------------------------------------------------
void finishCapture() {
#if MODE_SERIAL_CSV == 0
  publishPeakEvent();
#endif
}

void blinkLedDummy() {}

// ---------------------------------------------------------------------------
// Publicacion MQTT del pico: UN solo mensaje por evento (topic nuevo).
// ---------------------------------------------------------------------------
#if MODE_SERIAL_CSV == 0
void publishPeakEvent() {
  char msg[32];
  snprintf(msg, sizeof(msg), "{\"peak\":%.2f}", eventPeakZ);

#if MQTT_DRY_RUN
  // Ensayo en seco: NO se llama a sendToBroker() para el pico (evita el eco
  // [MQTT-OUT] de 2 lineas). Solo se emite la linea serie minima al final.
#else
  if (!client.connected()) {
    Serial.println(F("[MQTT] sin broker: pico descartado (politica v3)"));
    return;
  }
  sendToBroker("accel/event/peak", msg);
#endif

  // Salida serie del evento: UNA sola linea, SOLO el numero de la magnitud
  // maxima (pico), sin prefijo ni unidad. Ej: "0.44"
  Serial.printf("%.2f\n", eventPeakZ);
}
#endif

// ---------------------------------------------------------------------------
// Procesado de una muestra (~100 Hz). Logica identica a v2 (solo Z decide).
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

  // La desviacion que decide es SOLO la de Z (gx, gy y spX/spY se siguen
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
      //       podria pisar la ventana mientras la anterior sigue llenandose.
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
//   t,X,Y,Z,setX,setY,setZ,sym  -- formato IDENTICO a v1/v2
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
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, sizeof(rtc));  // I-001: 8 bytes
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
  client.setBufferSize(MQTT_BUF_MAX);   // v3: 384 B (el pico no necesita mas)
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
      // delay() enrzmizados: el WDT real corta a los 8 s
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
    Serial.println(F("=== BusAccel-8266-v3 (pico por evento, eje Z, ENSAYO EN SECO) ==="));
    Serial.println(F("Sin WiFi y sin MQTT: 1 linea por evento con el pico (solo el numero)."));
  #else
    Serial.println(F("=== BusAccel-8266-v3 (MQTT pico por evento, eje Z) ==="));
  #endif
#else
  Serial.println(F("=== BusAccel-8266-v3 (modo serie puro, SIN WiFi) ==="));
  Serial.println(F("Formato CSV: t,X,Y,Z,setX,setY,setZ,sym"));
#endif
  Serial.print("[FW] version "); Serial.println(FW_VERSION);
  Serial.print("[FW] build ");   Serial.println(FW_BUILD);

  memset(preRing, 0, sizeof(preRing));   // solo el anillo (ya no eventBuf)

#if MODE_SERIAL_CSV == 0
  loadEventCounter();                    // ANTES de imprimir (bug 1)
  Serial.printf("[RTC] contador de eventos: %lu\n", (unsigned long)eventCount);

  rtcData_t rtc;
  ESP.rtcUserMemoryRead(0, (uint32_t*)&rtc, sizeof(rtc));   // I-001: 8 bytes
  if (rtc.magic == 0xDEAD && rtc.failureFlag == 1) {
    Serial.println(F("Failure reboot detected. Randomizing MAC..."));
    randomizeMac();
    rtc.failureFlag = 0;
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, sizeof(rtc));  // I-001: 8 bytes
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
  Serial.printf("Ventana: %d antes + %d despues = %d muestras ~%ums | 1 mensaje/evento (pico)\n",
                PRE_N, POST_N, EVENT_N, (unsigned)EVENT_SPAN_MS);
  Serial.print  ("Broker (si se activa MQTT_DRY_RUN=0): ");
  Serial.println(MQTT_BROKER);
  Serial.println("Golpea/mueve el sensor: cada evento imprime SOLO el pico (el numero).");
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

  // I-003: stale publish. Si no se logra publicar nada durante STALE_REPORT_MS
  // (15 min) y la red/MQTT no se recuperan, se reinicia con MAC aleatoria.
  // Opcion simple elegida: comprobacion periodica cada ~1 s en el propio loop().
  if (now - lastStaleCheckMs >= 1000UL) {
    lastStaleCheckMs = now;
    if ((now - lastGoodPublish) > STALE_REPORT_MS) {
      Serial.println(F("[WDT] stale publish: sin publicar por 15 min. Reiniciando..."));
      rebootWithNewMac();
    }
  }
#endif
#endif

  // Muestreo interno a ~100 Hz
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    uint16_t dt = (uint16_t)(now - lastSampleMs);
    lastSampleMs = now;
    if (readAccel()) {
      pushSample(dt);       // 1) anillo del pasado (siempre, evento o no)
      processSample();      // 2) deteccion -> puede abrir una ventana
      advanceCapture(dt);   // 3) si hay ventana, actualiza el pico y cuenta POST
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
