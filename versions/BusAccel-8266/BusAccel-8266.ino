// ============================================================================
// BusAccel-8266 — SALIDA SERIAL CSV PARA GRAFICADO (ciclo 16) — version 2
// Placa: NodeMCU ESP8266 (ESP-12E)  |  FQBN: esp8266:esp8266:nodemcuv2
//
// Sensor: GY-291 (ADXL345) via I2C (rango +-2g, 256 LSB/g = raw/256.0f)
//   SDA -> D2/GPIO4 , SCL -> D1/GPIO5  (Wire.begin(4,5))
//
// ENFOQUE (ciclo 16): la salida PERIODICA es una linea CSV por tick a 20 Hz,
//   parseable por una app de escritorio/web que grafica en tiempo real:
//     t,X,Y,Z,setX,setY,setZ,sym
//   donde t=millis() (ms), X/Y/Z = aceleracion en g, setX/setY/setZ = setpoint
//   por eje en g, sym = simbolo de deteccion del tick ('.','+','*','-','!').
//   La LOGICA de deteccion es identica al ciclo 15: UN SOLO simbolo por evento
//   (estados ST_IDLE/ST_LOCKED) que aparece como campo `sym` de la linea CSV.
//   1. Cada tick de 20 Hz emite UNA linea CSV (nada mas, para que la app
//      pueda parsear por lineas):  t,X,Y,Z,setX,setY,setZ,sym
//        t    = millis() en ms
//        X,Y,Z      = aceleracion actual en g (2 decimales)
//        setX,setY,setZ = setpoint por eje en g (2 decimales)
//        sym  = simbolo de deteccion del tick:
//          '.' = reposo (ninguna desviacion supera ARROW_THRESHOLD_G)
//          '+' = elevacion LEVE (desviacion positiva entre 0.20g y 0.24g)
//          '*' = elevacion SEVERA (desviacion positiva >= 0.24g = 1.2x umbral
//                leve -> salto peligroso del bus, ej. lomo de toro a gran velocidad)
//          '-' = caida LEVE (desviacion negativa entre -0.20g y -0.24g)
//          '!' = caida SEVERA (desviacion negativa <= -0.24g = 1.2x umbral leve
//                -> bache profundo / magnitud relevante)
//   2. Al detectar el PRIMER cruce del umbral se captura UN SOLO simbolo
//      ('+'/'*' o '-'/'!' segun signo y MAGNITUD) y se entra en estado
//      BLOQUEADO/IGNORAR: los ticks siguientes llevan sym='.' aunque el rebote
//      inercial cruce el umbral en sentido contrario.
//   3. El bloqueo se libera SOLO si se cumplen DOS condiciones (ciclo 11):
//        (a) ha pasado EVENT_LOCKOUT_MS (anulacion del rebote post-evento)
//        (b) TODOS los ejes llevan REST_MS consecutivos dentro de +-RETURN_G
//            del setpoint (reposo sostenido, no un breve instante aislado);
//      entonces se permite la siguiente deteccion.
//   4. El setpoint se recalcula EN SILENCIO cada 20 muestras (solo en estado
//      no bloqueado / IDLE). Ya NO hay salto de linea cada N caracteres: cada
//      tick es una linea CSV completa terminada en \n.
//   5. Al arranque (una sola vez): banner + diagnostico profundo de registros
//      (WHO_AM_I, POWER_CTL, DATA_FORMAT, BW_RATE, OFS) + adquisicion inicial
//      de 20 puntos "." + "setpoint establecido: ..." . Despues SOLO lineas CSV.
//   6. Muestreo interno 100 Hz (SAMPLE_INTERVAL_MS=10), impresion 20 Hz
//      (PRINT_INTERVAL_MS=50), umbral ARROW_THRESHOLD_G=0.20g, histeresis de
//      retorno RETURN_G=0.15g, sin WiFi, sin web, sin secrets.h.
//   7. Sentido en el primer cruce: la desviacion de mayor |valor| entre X/Y/Z
//      decide el simbolo. Caracteres 100% ASCII; sin UTF-8.
//   8. SEVERIDAD (ciclo 12, recalibrada en ciclos 13 y 15): se deduce de la
//      MAGNITUD de la desviacion respecto al setpoint, no solo del sentido. Si la
//      desviacion es >= 1.2x el umbral leve (SEVERE_FACTOR=1.2 -> con umbral leve
//      0.20g, SEVERE_THRESHOLD_G=0.24g) indica un salto peligroso del bus (pasar
//      a gran velocidad por un lomo de toro). En el ciclo 13 el umbral severo
//      bajo a 0.30g (antes 0.45g); en el ciclo 15, a peticion del usuario
//      ("baja mas aun el limite para '*' y '!'"), baja a 0.24g: la categoria
//      severa ('*'/'!') se alcanza con un lomo de toro mas comun.
//
// Se MANTIENE del ciclo 4c/5:
//   - Diagnostico profundo de registros al arranque + forzado de measurement.
//   - Reintento del sensor en background si no aparece en I2C.
//   - Modo diagnostico: SIN WiFi, SIN web, SIN secrets.h.
//
// Mapa de pines:
//   D1/GPIO5  SCL -> GY-291 SCL
//   D2/GPIO4  SDA -> GY-291 SDA
//   3V3/GND   alimentacion GY-291
// ============================================================================

#define MODE_DIAGNOSTIC_SERIAL 1   // 1 = solo consola serial (sin WiFi/web)

#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ---------------------------------------------------------------------------
// Constantes
// ---------------------------------------------------------------------------
#define BAUD_RATE            115200
// Version del firmware impresa por la consola al arrancar.
#define FW_VERSION          "2"
#define FW_BUILD            "2026-09-30"
#define SENSOR_RANGE         ADXL345_RANGE_2_G
#define SENSOR_DATARATE      ADXL345_DATARATE_100_HZ
#define SAMPLE_INTERVAL_MS   10        // muestreo interno ~100 Hz
#define PRINT_INTERVAL_MS    50        // impresion serial ~20 Hz (1 linea CSV/tick)
#define SENSOR_RETRY_MS      1000      // reintento en background cada 1 s

// Setpoint / caracteres (ciclo 6)
// Ciclo 14: SETPOINT_N baja de 50 a 20 (peticion del usuario: "la 50 lecturas
//   para obtener promedio es mucho, reduzcamos a 20"). Al ser la MISMA constante
//   que usa el recalculo periodico en ST_IDLE, la adquisicion inicial Y el
//   recalculo en silencio quedan ambos en 20 muestras: mas rapido y coherente.
//   La tira de puntos "." al arranque pasa de 50 a 20 (usar SETPOINT_N, no un
//   literal "50", en el codigo que imprime los puntos).
#define SETPOINT_N           20            // lecturas para el promedio por eje
#define ARROW_THRESHOLD_G    0.20f         // desviacion (g) vs setpoint para '+'/'-'
                                           //   >> AJUSTABLE por el usuario <<
                                           //   (ciclo 13: 0.30g -> 0.20g, mas
                                           //   sensible: '+'/'-' desde 0.20g)
// SEVERIDAD (ciclo 12, recalibrada en ciclos 13 y 15): la magnitud de la
//   desviacion determina si el evento es leve o severo. SEVERE_FACTOR = 1.2 ->
//   un evento cuya desviacion es >= 1.2x el umbral leve es un salto peligroso
//   del bus (pasar a gran velocidad por un lomo de toro / bache profundo). Con
//   el umbral leve en 0.20g, SEVERE_THRESHOLD_G = 0.20*1.2 = 0.24g (ciclo 15:
//   antes 0.30g). El usuario pidio bajar AUN MAS el limite para '*'/'!':
//   ahora la categoria severa se alcanza desde 0.24g, mas facil con un lomo
//   de toro comun; el leve sigue en 0.20g.
#define SEVERE_FACTOR        1.2f
#define SEVERE_THRESHOLD_G   (ARROW_THRESHOLD_G * SEVERE_FACTOR)  // 0.24g
#define RETURN_G             0.15f         // histeresis: volver a '.' solo cuando
                                           //   TODOS los ejes esten a +-0.15g
                                           //   (ciclo 13: se MANTIENE 0.15g; es
                                           //   < ARROW_THRESHOLD_G=0.20g, por lo
                                           //   que el retorno a reposo es mas
                                           //   exigente que el disparo, sin
                                           //   incompatibilidad. Bajarlo no es
                                           //   necesario ni beneficioso aqui.)
#define LINE_CHARS           50            // (ciclo 16: ya NO se usa; cada tick
                                           //  emite una linea CSV completa)

// --- Anulacion del rebote (ciclo 11) ---
// EVENT_LOCKOUT_MS: tiempo minimo que el estado permanece BLOQUEADO tras
//   capturar un evento. Anula el rebote inercial del ADXL345, que provoca
//   un segundo simbolo en sentido contrario ~100-200 ms despues del evento.
//   Criterio (ciclo 11): 250 ms es el minimo viable para anular el rebote
//   (~200 ms); por debajo de 250 ms el rebote reaparece (segundo simbolo).
//   Permite estimulacion repetida mas rapida (~4 simbolos por segundo).
#define EVENT_LOCKOUT_MS     250
// REST_MS: tiempo de reposo sostenido que debe mantenerse (TODOS los ejes
//   dentro de +-RETURN_G del setpoint) antes de desbloquear. Evita que un
//   breve instante de reposo entre dos rebotes desbloquee el estado.
//   Criterio (ciclo 11): 50 ms = 5 muestras a 100 Hz de reposo sostenido,
//   suficiente para confirmar el fin del rebote sin demorar el desbloqueo.
#define REST_MS              50

// Pines I2C
#define PIN_SDA              4         // D2/GPIO4
#define PIN_SCL              5         // D1/GPIO5

// Registros ADXL345 (lectura directa por I2C con Wire)
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
// Estado global
// ---------------------------------------------------------------------------
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

bool sensorOnline = false;
bool notDetectedReported = false;   // para imprimir NO DETECTADO una sola vez
float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // valores en g (raw / 256.0f)
int16_t rawX = 0, rawY = 0, rawZ = 0;    // crudos int16 leidos por I2C

// Setpoint y deteccion de caracteres
bool setpointReady = false;             // true tras las 20 lecturas iniciales
float spX = 0.0f, spY = 0.0f, spZ = 0.0f;  // setpoint por eje (g)
float devX = 0.0f, devY = 0.0f, devZ = 0.0f;  // desviacion gx-spX, etc.
float accSumX = 0.0f, accSumY = 0.0f, accSumZ = 0.0f;  // acumulador setpoint
uint16_t accCount = 0;                  // lecturas acumuladas para el promedio

// Maquina de estados de deteccion (ciclo 11): suprime el rebote inercial.
//   ST_IDLE   : reposo; se permite detectar el PRIMER cruce del umbral.
//   ST_LOCKED : evento ya capturado; se IGNORA cualquier cruce posterior
//               (rebote). Desbloquea solo con DOS condiciones:
//               (a) lockout temporal EVENT_LOCKOUT_MS ya cumplido y
//               (b) reposo sostenido REST_MS (todos los ejes a +-RETURN_G).
enum DetectState : uint8_t { ST_IDLE = 0, ST_LOCKED = 1 };
DetectState detectState = ST_IDLE;
char  eventChar = '.';        // simbolo unico ('+'/'*' o '-'/'!') capturado al cruzar
bool  symbolPending = false;  // true mientras el simbolo unico no se ha impreso

// Lockout temporal post-evento (ciclo 11)
unsigned long lockUntilMs = 0;  // instante minimo (millis) en que se permite desbloquear
unsigned long restSinceMs = 0;  // instante en que arranco el reposo sostenido (0 = sin reposo)

// Salida CSV por tick (ciclo 16): se imprime una linea completa por tick
// (t,X,Y,Z,setX,setY,setZ,sym). printCount de ciclos anteriores se elimina:
// cada tick es SU PROPIA linea terminada en \n.

unsigned long lastSampleMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastRetryMs = 0;

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

// Burst read de DATAX0..DATAZ1 (6 bytes, little-endian int16)
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
// Diagnostico del chip: se imprime UNA VEZ al arrancar
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

  // --- Forzar measurement mode (bit 3 = 0x08 en POWER_CTL) y verificar ---
  Serial.println("-- Forzando measurement mode: escribiendo 0x08 en POWER_CTL --");
  bool wrote = writeReg(REG_POWER_CTL, 0x08);
  delay(50);
  uint8_t pwr2 = readReg(REG_POWER_CTL);
  if (wrote && (pwr2 & 0x08)) {
    Serial.println("[POWER_CTL] OK: 0x08 persiste -> measurement mode ACTIVO");
  } else {
    Serial.printf("[POWER_CTL] NO se mantiene 0x08 (escritura=%d, releido=0x%02X) -> problema de escritura I2C (revisar cableado/soldadura)\n",
                  wrote ? 1 : 0, pwr2);
  }

  // Primera muestra cruda tras forzar measure
  if (readRawAxes()) {
    Serial.printf("PRIMERA MUESTRA rawX=%6d rawY=%6d rawZ=%6d\n", rawX, rawY, rawZ);
  } else {
    Serial.println("[I2C] No se pudo leer DATAX0..DATAZ1 (revisar SDA/SCL)");
  }
  Serial.println("=== FIN DIAGNOSTICO ===");
  Serial.println();
}

// ---------------------------------------------------------------------------
// Lectura del acelerometro: burst read de 6 bytes -> g = raw / 256.0f (+-2g).
// Mantiene sensorOnline y sirve de unica fuente para setpoint y caracteres.
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
// Procesado de una muestra (se llama a ~100 Hz)
//   - Adquisicion inicial: 20 muestras -> setpoint (imprime '.' por muestra).
//   - Luego: maquina de estados IDLE -> LOCKED (un solo simbolo por evento)
//     y acumula muestras para recalcular el setpoint cada 20 lecturas
//     (SETPOINT_N, ciclo 14) EN SILENCIO (sin imprimir mensaje) y solo en
//     estado IDLE.
// ---------------------------------------------------------------------------
void processSample() {
  if (!setpointReady) {
    // --- Adquisicion inicial del setpoint (en reposo) ---
    accSumX += gx; accSumY += gy; accSumZ += gz;
    accCount++;
    Serial.print('.');                     // senal de adquisicion: solo el punto
    if (accCount >= SETPOINT_N) {
      spX = accSumX / SETPOINT_N;
      spY = accSumY / SETPOINT_N;
      spZ = accSumZ / SETPOINT_N;
      setpointReady = true;
      accSumX = accSumY = accSumZ = 0.0f;
      accCount = 0;
      Serial.println();
      Serial.printf("setpoint establecido: X=%.2f Y=%.2f Z=%.2f\n", spX, spY, spZ);
    }
    return;
  }

  // --- Modo deteccion en tiempo real ---
  devX = gx - spX;
  devY = gy - spY;
  devZ = gz - spZ;

  // Desviacion maxima (la de mayor |desviacion| entre X/Y/Z) y su signo
  float maxDev = devX;
  bool maxDevPositive = (devX >= 0.0f);
  if (fabsf(devY) > fabsf(maxDev)) { maxDev = devY; maxDevPositive = (devY >= 0.0f); }
  if (fabsf(devZ) > fabsf(maxDev)) { maxDev = devZ; maxDevPositive = (devZ >= 0.0f); }

  bool eventActive = fabsf(maxDev) > ARROW_THRESHOLD_G;
  bool allResting  = (fabsf(devX) <= RETURN_G &&
                      fabsf(devY) <= RETURN_G &&
                      fabsf(devZ) <= RETURN_G);

  switch (detectState) {

    case ST_IDLE:
      if (eventActive) {
        // PRIMER cruce del umbral: capturar UN SOLO simbolo y BLOQUEAR.
        // El simbolo se imprime en el siguiente tick de 20 Hz (ver
        // printCharTick); el estado LOCKED impide cualquier simbolo
        // adicional (rebote inercial del ADXL345 en sentido contrario).
        // Ciclo 12 (SEVERIDAD): el simbolo depende del SENTIDO y de la
        //   MAGNITUD de la desviacion maxima respecto al setpoint:
        //     devMax >  +ARROW_THRESHOLD_G (elevacion):
        //        devMax >= +SEVERE_THRESHOLD_G -> '*'
        //        si no                            -> '+'
        //     devMax <  -ARROW_THRESHOLD_G (caida):
        //        devMax <= -SEVERE_THRESHOLD_G -> '!'
        //        si no                           -> '-'
        if (maxDevPositive) {
          eventChar = (maxDev >= SEVERE_THRESHOLD_G) ? '*' : '+';
        } else {
          eventChar = (maxDev <= -SEVERE_THRESHOLD_G) ? '!' : '-';
        }
        symbolPending = true;
        detectState = ST_LOCKED;
        // Lockout temporal post-evento: el estado permanece bloqueado al
        // menos EVENT_LOCKOUT_MS (anula el rebote inercial en sentido
        // contrario que llega ~100-200 ms despues del evento).
        lockUntilMs = millis() + EVENT_LOCKOUT_MS;
        restSinceMs = 0;
      } else {
        // Reposo: acumular para el recalculo del setpoint (EN SILENCIO),
        // solo en estado no bloqueado (requisito ciclo 8, punto 4).
        accSumX += gx; accSumY += gy; accSumZ += gz;
        accCount++;
        if (accCount >= SETPOINT_N) {
          spX = accSumX / SETPOINT_N;
          spY = accSumY / SETPOINT_N;
          spZ = accSumZ / SETPOINT_N;
          accSumX = accSumY = accSumZ = 0.0f;
          accCount = 0;
          // Recalculo en silencio: sin imprimir "setpoint actualizado"
        }
      }
      break;

    case ST_LOCKED:
      // Evento ya capturado: NO se emite ningun '+' ni '-' (se ignora el
      // rebote aunque la desviacion cruce el umbral en sentido contrario).
      // Desbloqueo con DOS condiciones acumuladas (ciclo 11):
      //   (a) millis() >= lockUntilMs  -> ha pasado el lockout temporal
      //       post-evento (EVENT_LOCKOUT_MS), y
      //   (b) reposo SOSTENIDO: TODOS los ejes llevan REST_MS consecutivos
      //       dentro de +-RETURN_G del setpoint (restSinceMs no se reinicia).
      if (allResting) {
        if (restSinceMs == 0) {
          restSinceMs = millis();          // arrancar cronometro de reposo
        }
        if (millis() >= lockUntilMs &&
            (millis() - restSinceMs) >= REST_MS) {
          detectState = ST_IDLE;
          restSinceMs = 0;
        }
      } else {
        // Alguna desviacion volvio a superar +-RETURN_G: reiniciar el
        // cronometro de reposo sostenido (debe partir de cero de nuevo).
        restSinceMs = 0;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Impresion de la linea CSV por tick a ~20 Hz (ciclo 16). Formato EXACTO:
//   t,X,Y,Z,setX,setY,setZ,sym
//     t           = millis() en ms (unsigned long)
//     X,Y,Z       = aceleracion actual en g (float, 2 decimales)
//     setX,setY,setZ = setpoint actual por eje en g (float, 2 decimales)
//     sym         = simbolo de deteccion del tick ('.','+','*','-','!')
// Sin texto extra en la linea: la app de graficado parsea por lineas.
// ---------------------------------------------------------------------------
void printCharTick() {
  // El simbolo de este tick: si hay un evento unico pendiente se consume una
  // sola vez ('+'/'-' leve o '*'/'!' severo); en cualquier otro caso '.'.
  char c = '.';
  if (symbolPending) {
    c = eventChar;
    symbolPending = false;               // el simbolo se emite una sola vez
  }
  Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%c\n",
                millis(), gx, gy, gz, spX, spY, spZ, c);
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  delay(300);
  Serial.println("=== BusAccel-8266 DIAG CSV (ciclo 16, t,X,Y,Z,setX,setY,setZ,sym) ===");
  Serial.print("[FW] BusAccel-8266 version ");
  Serial.println(FW_VERSION);
  Serial.print("[FW] build ");
  Serial.println(FW_BUILD);

  Wire.begin(PIN_SDA, PIN_SCL);

  if (accel.begin()) {
    accel.setRange(SENSOR_RANGE);
    accel.setDataRate(SENSOR_DATARATE);
    sensorOnline = true;
    Serial.println("[ADXL345] OK (begin() correcto)");
  } else {
    sensorOnline = false;
    notDetectedReported = true;
    Serial.println("[ADXL345] NO DETECTADO (revisar cableado VCC/GND/SDA/SCL)");
  }

  // Diagnostico profundo de registros: se ejecuta SIEMPRE (lee por I2C directo)
  runChipDiagnostic();

  Serial.println("Adquiriendo setpoint (20 lecturas en reposo)...");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // Muestreo interno a ~100 Hz (10 ms): lee el sensor y procesa la muestra
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    if (readAccel()) {
      processSample();
    }
  }

  // Reintento en background si el sensor no responde (sin spam)
  if (!sensorOnline && (now - lastRetryMs >= SENSOR_RETRY_MS)) {
    lastRetryMs = now;
    if (accel.begin()) {
      accel.setRange(SENSOR_RANGE);
      accel.setDataRate(SENSOR_DATARATE);
      sensorOnline = true;
      notDetectedReported = false;
      Serial.println("[ADXL345] OK");
    } else if (!notDetectedReported) {
      notDetectedReported = true;
      Serial.println("[ADXL345] NO DETECTADO (revisar cableado VCC/GND/SDA/SCL)");
    }
  }

  // Impresion serial a ~20 Hz (50 ms): UNA linea CSV por tick
  if (setpointReady && (now - lastPrintMs >= PRINT_INTERVAL_MS)) {
    lastPrintMs = now;
    printCharTick();
  }
}