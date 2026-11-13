# BusAccel-8266-v3 — Datasheet de usuario

> Documentación de usuario del firmware **v3** (pico por evento, eje Z).
> Para la explicación del código ver [`developer.md`](developer.md).

---

## 1. Descripción general

**BusAccel-8266-v3** es un registrador de aceleraciones para un **bus urbano**.
Un acelerómetro **GY-291 (ADXL345)** conectado por I²C a un **NodeMCU
ESP8266** mide la aceleración vertical (eje Z) a 100 Hz. Cuando la
desviación de Z respecto al setpoint supera el umbral, el equipo mide el
**PICO** (máxima `|devZ|` de una ventana de 200 muestras: 1 s antes + 1 s
después) y emite **un único mensaje compacto** por evento.

**Diferencia clave respecto de v2:** ya NO se transmite la ventana cruda.
Desaparecen los 5 mensajes de chunk y las 200 líneas `EV,` por serie. Por cada
evento se publica/imprime **SOLO el PICO**.

---

## 2. Especificaciones técnicas

### 2.1 Placa

| Parámetro | Valor |
|---|---|
| Placa | NodeMCU ESP8266 (módulo ESP-12E) |
| MCU | ESP8266EX (Tensilica L106, 32-bit) @ 80/160 MHz |
| Flash | 4 MB |
| SRAM usable | 80192 B (~80 KB) |
| **RAM usada por el sketch** | **~30–32 KB (37–40 %)** |
| **Flash usada** | **~24–25 %** (~246 KB) |
| IRAM | 65536 B; baseline SDK ~60931 B (~92 %, no margen útil) |
| Alimentación | 5 V por USB o 3.3 V por pin `3V3` |
| Interfaz usuario | consola serie 115200 baud |
| FQBN | `esp8266:esp8266:nodemcuv2` |
| Core | `esp8266:esp8266@3.1.2` |

### 2.2 Sensor

| Parámetro | Valor |
|---|---|
| Módulo | GY-291 (breakout ADXL345) |
| Chip | Analog Devices ADXL345, MEMS 3 ejes |
| Interfaz | I²C, dirección **0x53** (SDO a GND) |
| Rango | **±2 g** (`ADXL345_RANGE_2_G`) |
| Conversión | 256 LSB/g → `g = raw / 256.0f` |
| Datarate | **100 Hz** (`ADXL345_DATARATE_100_HZ`) |
| Resolución | 10 bits en ±2 g → ~3.9 mg/LSB |
| Lectura | burst de 6 bytes (`DATAX0..DATAZ1`), little-endian int16 |
| WHO_AM_I | `0xE5` |

### 2.3 Constantes de detección

| Constante | Valor | Significado |
|---|---|---|
| `ARROW_THRESHOLD_G` | 0.20 g | umbral leve (`+`/`-`) |
| `SEVERE_FACTOR` | 1.2 | multiplicador → severo |
| `SEVERE_THRESHOLD_G` | 0.24 g | umbral severo (`*`/`!`) |
| `RETURN_G` | 0.15 g | histéresis de retorno a reposo |
| `EVENT_LOCKOUT_MS` | 250 ms | bloqueo anti-rebote |
| `REST_MS` | 50 ms | reposo sostenido para desbloquear |
| `SETPOINT_N` | 20 | lecturas para el setpoint |
| `PRE_N` / `POST_N` | 100 / 100 | muestras antes/después del cruce |
| `EVENT_N` | 200 | total de la ventana |
| `EVENT_SPAN_MS` | 2000 ms | duración nominal de la ventana |
| `SAMPLE_INTERVAL_MS` | 10 ms | muestreo interno ~100 Hz |
| `PRINT_INTERVAL_MS` | 50 ms | impresión CSV ~20 Hz (solo MODE 1) |

---

## 3. Librerías requeridas e instalación

### 3.1 Librerías

| Librería | Versión | Propósito | Instalación |
|---|---|---|---|
| `Wire` | built-in (core 3.1.2) | I²C con el ADXL345 | incluida en el core |
| `math` | built-in | `fabsf()` | incluida en el core |
| `Adafruit Unified Sensor` | 1.1.x | interfaz base de sensores | Library Manager o `arduino-cli lib install` |
| `Adafruit ADXL345` | 1.3.x | `begin()`, `setRange()`, `setDataRate()` | Library Manager o `arduino-cli lib install` |
| `ESP8266WiFi` | built-in (core 3.1.2) | WiFi STA | incluida en el core (solo MODE 0) |
| `PubSubClient` | 2.8.0 | cliente MQTT | Library Manager o `arduino-cli lib install` (solo MODE 0) |
| `user_interface.h` | built-in | `wifi_set_macaddr` (en `extern "C"`) | incluida en el core (solo MODE 0) |
| `ping.h` | built-in | ICMP a 8.8.8.8 (en `extern "C"`) | incluida en el core (solo MODE 0) |

### 3.2 Instalación con arduino-cli

```bash
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli lib install "Adafruit ADXL345"
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib install "PubSubClient"
arduino-cli lib list
```

### 3.3 Instalación con Arduino IDE

1. **Herramientas → Gestionar librerías** (Ctrl+Shift+I).
2. Busca **"Adafruit ADXL345"** e instala (instala también **Adafruit Unified
   Sensor** como dependencia).
3. Busca **"PubSubClient"** e instala la de Nick O'Leary.
4. Selecciona la placa **NodeMCU 1.0 (ESP-12E Module)** y el puerto correcto.

> Las versiones de las librerías externas deben **confirmarse** con
> `arduino-cli lib list` en el equipo antes de compilar.

---

## 4. Modos de funcionamiento

### 4.1 `MODE_SERIAL_CSV` (compilación)

| Valor | Comportamiento |
|---|---|
| **0** (default) | detección por evento + MQTT (o ensayo en seco). Cada evento imprime **una línea `[PICO]`** |
| **1** | CSV puro a 20 Hz, **sin WiFi**. Formato `t,X,Y,Z,setX,setY,setZ,sym` (idéntico a v1/v2). **No emite `[PICO]`** |

### 4.2 `MQTT_DRY_RUN` (compilación)

| Valor | Comportamiento |
|---|---|
| **1** (default) | **NO abre WiFi ni MQTT**. Cada mensaje que se publicaría se imprime por serie con su topic y su payload (`[MQTT-OUT] --> …` + `[MQTT-OUT] {json}`). Validación sin suscriptor en el broker |
| **0** | MQTT real: conecta WiFi y publica al broker. Requiere credenciales válidas en `secrets.h` |

### 4.3 Combinaciones típicas

| Uso | `MODE_SERIAL_CSV` | `MQTT_DRY_RUN` |
|---|---|---|
| Validación en seco (default) | 0 | 1 |
| MQTT real en campo | 0 | 0 |
| App de escritorio / CSV | 1 | (irrelevante, no hay WiFi) |

---

## 5. Contrato de datos

### 5.1 Topics de salida

| Topic | Cuándo | Payload |
|---|---|---|
| `<client>/accel/online` | al conectar | `true` |
| `<client>/accel/status` | al conectar | `OK` o `NO_DETECTADO` |
| `<client>/accel/setpoint` | cada recálculo del setpoint | `{"x":…,"y":…,"z":…}` en g |
| **`<client>/accel/event/peak`** | **1 por evento** | ver §5.2 |

> **Los topics `accel/event/summary` y `accel/event/chunk` YA NO EXISTEN** en
> v3.

### 5.2 Payload del pico (`accel/event/peak`)

```json
{"id":12,"sym":"*","spz":0.93,"dev":0.27,"peak":0.31,"n":200,"ms":2000,"up":63959}
```

| Campo | Tipo | Significado |
|---|---|---|
| `id` | entero | identificador del evento, persistente en RTC (empieza en 1) |
| `sym` | string | `. + * - !` |
| `spz` | float | setpoint de Z en el instante del evento (g) |
| `dev` | float | desviación de Z en el instante del cruce (g), con signo |
| `peak` | float | **máxima `|devZ|` de toda la ventana (pre + post), en g** |
| `n` | entero | muestras en la ventana (siempre 200) |
| `ms` | entero | duración nominal de la ventana (siempre 2000) |
| `up` | entero | `millis()` del equipo en el inicio de la ventana |

> ⚠️ **`up` NO es hora de reloj.** Son milisegundos desde el arranque en el
> instante del **disparo**. No hay NTP/RTC de hora. Solo son comparables entre
> eventos de la misma sesión.

### 5.3 Salida serie por evento (`MODE_SERIAL_CSV 0`)

```
[PICO] id=12 sym=* peak=0.31g spz=0.93g dev=0.27g up=63959ms n=200 ms=2000
```

En `MQTT_DRY_RUN 1`, además:

```
[MQTT-OUT] --> <client>/accel/event/peak  (150 B)
[MQTT-OUT] {"id":12,"sym":"*","spz":0.93,"dev":0.27,"peak":0.31,"n":200,"ms":2000,"up":63959}
```

### 5.4 Entrada (suscripción)

El dispositivo se suscribe a `<client>/#`. Única orden implementada:

| Topic | Payload | Efecto |
|---|---|---|
| `<client>/…/RECAL` | `RECAL` | recalibra el setpoint sin reiniciar |

---

## 6. Guía de solución de problemas

### 6.1 El sensor no responde

| Síntoma | Causa | Solución |
|---|---|---|
| `[ADXL345] NO DETECTADO` | cableado VCC/GND/SDA/SCL | revisa conexiones; verifica 3.3 V |
| `WHO_AM_I` ≠ 0xE5 | clon defectuoso o mala lectura I²C | revisa cableado/soldadura; prueba otro módulo |
| `POWER_CTL` no mantiene 0x08 | problema de escritura I²C | revisa cableado/soldadura |
| Lecturas congeladas | sensor en STANDBY | el firmware fuerza measurement mode al arrancar |

### 6.2 Detección

| Síntoma | Causa | Solución |
|---|---|---|
| No se detecta ningún evento | setpoint contaminado o umbral alto | pulsa RST con el bus en reposo; baja `ARROW_THRESHOLD_G` |
| Falsos `*`/`!` en reposo | microvibración o soporte flexible | sube `ARROW_THRESHOLD_G` a 0.25–0.30 g; atornilla el módulo |
| Doble evento por bache | rebote inercial | verifica `EVENT_LOCKOUT_MS` (250) y `REST_MS` (50) |
| Eventos muy seguidos | ventana anterior sin terminar | la condición (c) `!collecting` lo evita; no la quites |

### 6.3 Red (solo `MQTT_DRY_RUN 0`)

| Síntoma | Causa | Solución |
|---|---|---|
| `WiFi timeout` | red no visible / contraseña | verifica `secrets.h` |
| `MQTT auth error` | credenciales broker | confirma usuario/contraseña con el admin |
| `MQTT disconnected` | broker caído | el firmware reconecta solo; si persiste, reinicia con MAC aleatoria |
| `stale publish: sin publicar por 15 min` | red/MQTT sin recuperar | el firmware reinicia con MAC aleatoria |

### 6.4 Serie

| Síntoma | Causa | Solución |
|---|---|---|
| `PermissionError` en el puerto | usuario sin grupo `dialout` | `sudo usermod -aG dialout $USER` y reinicia sesión |
| Mensaje ROM a 74880 baud | cargador de arranque | normal; desaparece al conmutar a 115200 |
| No se ve nada | monitor cerrado o baud distinto | abre a 115200 baud |

---

## 7. Tabla de compatibilidad con otras placas Arduino

| Placa | Compatibilidad | Notas |
|---|---|---|
| **NodeMCU ESP8266 (ESP-12E)** | ✅ **Objetivo principal** | FQBN `esp8266:esp8266:nodemcuv2` |
| NodeMCU ESP8266 (ESP-12/F) | ✅ Compatible | mismo core; pines D1/D2 = GPIO5/GPIO4 |
| Wemos D1 mini | ✅ Compatible | mismo core; pines D1/D2 = GPIO5/GPIO4 |
| ESP8266 genérico (ESP-01) | ⚠️ Limitado | solo 2 GPIO; no hay I²C en pines estándar |
| ESP32 | ⚠️ Requiere cambios | otro core (`esp32:esp32`); `ESP.rtcUserMemory*` y `ping.h` no existen; `wifi_set_macaddr` distinto |
| Arduino Uno/Nano/Mega | ❌ No compatible | sin WiFi; `ESP.rtcUserMemory*`, `ESP8266WiFi`, `PubSubClient` con `WiFiClient` |
| Arduino MKR WiFi 1010 | ⚠️ Requiere cambios | otro core; RTC user memory distinta |

> El sketch está acoplado al **ESP8266** por: `ESP.rtcUserMemoryRead/Write`
> (contador persistente), `ESP8266WiFi`, `wifi_set_macaddr` (user_interface.h),
> `ping.h` (ICMP) y `ESP.wdtEnable/wdtFeed`. Portarlo a otra arquitectura
> requiere sustituir estas APIs.

---

## 8. Seguridad

| Aspecto | Estado |
|---|---|
| Credenciales | en `secrets.h` (**no versionado**); `secrets.example.h` de plantilla |
| ClientId | debe ser **distinto por placa** o el broker las expulsa |
| Broker | `www.smartage.cl:1886` responde `CONNACK 5` a las credenciales actuales; confirmar antes de `MQTT_DRY_RUN 0` |
| Ensayo en seco | `MQTT_DRY_RUN 1` (default): **nada sale de la placa** |
| RTC | el contador `id` vive en RTC user memory: sobrevive a reinicios por red, se pierde con corte de luz |
