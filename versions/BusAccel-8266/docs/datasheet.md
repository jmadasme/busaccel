# BusAccel-8266 — Hoja de datos y guía de usuario

> **Firmware documentado: ciclo 16 — MODO DIAGNÓSTICO SERIAL PURO (CSV).**
> Sin WiFi, sin servidor web, sin MQTT, sin EEPROM. Toda la interacción con el
> dispositivo es por **consola serie a 115200 baud** y el periodic output es una
> **línea CSV por tick** (`t,X,Y,Z,setX,setY,setZ,sym`), legible tanto por un
> terminal como por la **aplicación de escritorio** de `app/`.

| | |
|---|---|
| **Proyecto** | BusAccel-8266 |
| **Firmware** | `BusAccel-8266.ino` — ciclo 16 (diagnóstico serial con salida CSV) |
| **Placa objetivo** | NodeMCU ESP8266 (ESP-12E) |
| **FQBN** | `esp8266:esp8266:nodemcuv2` |
| **Sensor** | ADXL345 (módulo GY-291), I²C @ 0x53, ±2 g, 100 Hz |
| **Interfaz de usuario** | Consola serie 115200 baud, 8N1 · **App de escritorio** (`app/`) |
| **Fase actual** | Diagnóstico serial CSV + monitor de escritorio en tiempo real |
| **Fase futura** | Conteo de incidencias + MQTT (1 min) + EEPROM — **NO implementada** |

---

## 1. Descripción general

BusAccel-8266 es un **instrumento de medición** para analizar cómo se mueve un
bus urbano. Mide la aceleración en los tres ejes, la compara con una
referencia tomada en reposo (**setpoint**) y emite **una línea CSV cada
50 ms** (`t,X,Y,Z,setX,setY,setZ,sym`) que lleva tanto los valores crudos en g
como el símbolo de la detección (`.` `+` `*` `-` `!`).

El resultado es un "log" legible de un vistazo que permite identificar:

* **Baches y hoyos** → caídas (`-`, `!`)
* **Catenarias, bordillos y rasantes** → elevaciones leves (`+`)
* **Lomos de toro** → elevaciones (`+` a velocidad suave, `*` a alta velocidad)
* **Curvas y cambios de carril** → desviaciones en el eje lateral
* **Golpes de frenada** → desviaciones negativas

Y, desde el **ciclo 16**, un **gráfico en tiempo real** en el PC: la app
`app/` (§3.6) lee esas mismas líneas CSV y dibuja Z con zonas de severidad.

### 1.1 Qué NO hace (aclaración importante)

Estos puntos son causa frecuente de confusión porque la documentación de fases
anteriores los mencionaba. **El firmware actual NO los implementa:**

| Función | Estado | Nota |
|---|---|---|
| Access Point / modo STA WiFi | ❌ Eliminado | No hay ninguna llamada a `WiFi` |
| Servidor web HTTP (puerto 80) | ❌ Eliminado | No hay `WebServer`, ni HTML, ni canvas en el ESP8266 |
| Gráfico web servido por la placa | ❌ Eliminado | El gráfico se hace **en el PC** con `app/` (§3.6) |
| Cliente MQTT / broker | ❌ No implementado | Fase futura (ciclo posterior) |
| Persistencia en EEPROM | ❌ No implementado | Fase futura; los datos no se guardan en el chip |
| Lectura de voltaje de batería (`A0`) | ❌ No implementado | El pin `A0` está libre |
| Control del LED integrado | ❌ No implementado | `D4`/GPIO2 está libre |
| Lectura de credenciales (`secrets.h`) | ❌ No implementado | `secrets.example.h` es residuo; no hace falta |
| Rodillo de odómetro / GPS | ❌ No implementado | Sin ningún pin de entrada digital conectado |

**Consecuencia práctica:** si enciendes la placa no obtendrás ninguna IP, no
habrá servidor web, y tras un reinicio se pierde todo el registro. El registro
vive únicamente en el ordenador conectado por USB, ya sea como log de texto o
como buffer en memoria de la app.


---

## 2. Especificaciones técnicas

### 2.1 Placa — NodeMCU ESP8266 (ESP-12E)

| Parámetro | Valor |
|---|---|
| Microcontrolador | ESP8266EX |
| Frecuencia | 80 MHz |
| Flash | 4 MB |
| SRAM | ~80 KB de heap para el usuario (~40–50 KB libres con este sketch) |
| WiFi | 802.11 b/g/n presente en el chip — **deshabilitado en el firmware** |
| GPIO usados | 2 (GPIO4 = SDA, GPIO5 = SCL) |
| Convertidor USB-SD | CH340 o CP2102 según fabricante |
| Tensión de trabajo | 3.3 V (regulada en la placa) |
| Corriente | ~70–90 mA en régimen; ~120 mA en picos de arranque |
| Indicadores | LED azul de alimentación + LED rojo de TX/RX (controlados por el gestor de arranque, no por el firmware) |
| Botones | `RST` (reinicio) y `FLASH` (GPIO0 → modo bootloader) |

### 2.2 Sensor — GY-291 / ADXL345

| Parámetro | Valor |
|---|---|
| Dispositivo | ADXL345, acelerómetro MEMS 3 ejes de Analog Devices |
| Interfaz | I²C (SPI disponible en el chip, no se usa aquí) |
| Dirección I²C | `0x53` (fija; el selector de solder SDO del breakout va a GND) |
| Rango configurado | **±2 g** (`DATA_FORMAT` bits 0-1 = `0b00`) |
| Escala | 256 LSB/g → `g = raw / 256.0f` |
| Frecuencia de datos | **100 Hz** (`BW_RATE` = `0x0A`) |
| Ejes | X, Y, Z (acceleration, nogiroscopio) |
| Formato de datos | little-endian int16 de 2 bytes por eje |
| Acceso a datos | *burst read* de 6 bytes `DATAX0..DATAZ1` con `Wire` |
| Identificador | `DEVID (0x00)` = `0xE5` |
| Offsets de fábrica | registros `OFSX/OFSY/OFSZ` (`0x1E..0x20`) |
| Corriente | ~140 µA |
| Rango de temperatura | −40 °C a +85 °C |
| Vibración no sensible | vibration immunity > 10 kHz |

> **Sobre la escala de conversión:** el firmware lee los **registros crudos** del
> ADXL345 y los divide por 256, lo que da **exactamente gramos**, sin pasar por
> la conversión a m/s² de la librería Adafruit. Por eso los umbrales de
> detección están expresados en `g` y son válidos para el rango ±2 g.

### 2.3 Parámetros del algoritmo de detección

| Parámetro | Constante | Valor | Significado |
|---|---|---|---|
| Periodo de muestreo | `SAMPLE_INTERVAL_MS` | 10 ms | 100 Hz, igual que el data rate del chip |
| Periodo de impresión | `PRINT_INTERVAL_MS` | 50 ms | 20 Hz, **una línea CSV por tick** |
| Caracteres por línea | `LINE_CHARS` | 50 | ⚠️ **Definido pero sin uso en el ciclo 16** (residuo del ciclo 15) |
| Muestras de setpoint | `SETPOINT_N` | 20 | 200 ms de media; también el periodo de recálculo en reposo |
| Umbral leve | `ARROW_THRESHOLD_G` | 0.20 g | desviación mínima que dispara |
| Factor de severidad | `SEVERE_FACTOR` | 1.2 | multiplicador del umbral leve |
| Umbral severo | `SEVERE_THRESHOLD_G` | 0.24 g | derivado: `0.20 × 1.2` |
| Histeresis de retorno | `RETURN_G` | 0.15 g | banda de reposo para desbloquear |
| Bloqueo post-evento | `EVENT_LOCKOUT_MS` | 250 ms | anulación del rebote inercial |
| Reposo sostenido | `REST_MS` | 50 ms | 5 muestras en reposo para desbloquear |
| Reintento de sensor | `SENSOR_RETRY_MS` | 1000 ms | reintento en background si falla I²C |

### 2.4 Símbolos de salida (campo `sym`)

| Carácter | Código | ASCII | Nombre | Condición |
|---|---|---|---|---|
| `.` | 0x2E | 46 | Reposo | `\|dev\| ≤ 0.20 g` en los tres ejes |
| `+` | 0x2B | 43 | Elevación leve | `+0.20 g < dev < +0.24 g` |
| `*` | 0x2A | 42 | Elevación severa | `dev ≥ +0.24 g` |
| `-` | 0x2D | 45 | Caída leve | `−0.24 g < dev < −0.20 g` |
| `!` | 0x21 | 33 | Caída severa | `dev ≤ −0.24 g` |

> Los cinco son **ASCII imprimible estándar**. No hay bytes UTF-8, ni secuencias
> ANSI, ni caracteres de control: el log se puede procesar con `awk`, `grep` o
> cualquier script sin decodificación especial.

### 2.5 Formato de la línea de salida (ciclo 16)

Cada tick de 20 Hz emite **una línea CSV completa** terminada en `\n` (CRLF):

```
t,X,Y,Z,setX,setY,setZ,sym
```

| Pos | Campo | Tipo | Rango / valores | Significado |
|---|---|---|---|---|
| 0 | `t` | entero | 0 – 4 294 967 295 | `millis()` del ESP8266 desde el arranque, en ms |
| 1 | `X` | float 2 dec. | ±2.00 g | Aceleración longitudinal actual |
| 2 | `Y` | float 2 dec. | ±2.00 g | Aceleración lateral actual |
| 3 | `Z` | float 2 dec. | ±2.00 g | Aceleración vertical actual |
| 4 | `setX` | float 2 dec. | ±2.00 g | Setpoint longitudinal (línea base en reposo) |
| 5 | `setY` | float 2 dec. | ±2.00 g | Setpoint lateral |
| 6 | `setZ` | float 2 dec. | ±2.00 g | Setpoint vertical (típicamente ≈ 1.00) |
| 7 | `sym` | 1 carácter | `.` `+` `*` `-` `!` | Símbolo de detección de este tick |

**Ejemplo real:**

```
248150,0.14,0.17,0.91,0.14,0.16,0.92,.
248200,0.14,0.17,0.91,0.14,0.16,0.92,.
248400,0.15,0.17,1.16,0.14,0.16,0.92,*
```

| Propiedad | Valor |
|---|---|
| Longitud típica de línea | 30–38 bytes |
| Ritmo de datos | 20 líneas/s ≈ **700 B/s** (límite a 115200 baud ≈ 11 kB/s) |
| Separador de campos | coma `,` (sin comillas ni espacios) |
| Fin de línea | `\n` (CRLF) |
| Prefijo fijo | no lo hay → se parsea por longitud (8 campos) y por tipo del 1.er campo |
| Texto libre previo | banner, diagnóstico y 20 puntos `.` (se ignoran al parsear) |

> **Regla de parseo robusta:** una línea es válida sólo si tiene **exactamente 8
> campos separados por coma**, el primero parsea como entero, los seis
> siguientes como float y el último es uno de los cinco símbolos. Cualquier otra
> cosa es texto del arranque o ruido y se descarta. Es exactamente lo que
> hace `parse_line()` en `app/serial_reader.py`.


---

## 3. Instalación

### 3.1 Requisitos

| Requisito | Detalle |
|---|---|
| NodeMCU ESP8266 (ESP-12E) | con drivers USB-SD instalados en el SO |
| Módulo GY-291 (ADXL345) | listo para usar, sin descuentos |
| Cables Dupont | 4 (3–20 cm recomendados) |
| `arduino-cli` ≥ 0.30 | o el IDE de Arduino 2.x con ESP8266 instalado |
| Core `esp8266:esp8266` | 3.x |
| Librerías | `Adafruit ADXL345` 1.3.x, `Adafruit Unified Sensor` 1.1.x |
| SO | Linux, macOS o Windows 10/11 |

### 3.2 Conexión del hardware

```
   NodeMCU                          GY-291 (ADXL345)
   ───────                          ─────────────────
   3V3  ───────────────────────►    VCC
   GND  ───────────────────────►    GND
   D2 (GPIO4) ────────────────────► SDA
   D1 (GPIO5) ────────────────────► SCL
                    (opcional)
   GND  ───────────────────────►    SDO   → fija la dirección en 0x53
```

Para la lista completa de conexiones, la tabla de pines y el diagrama
detallado, ver **`README.md` §6 y §7**.

### 3.3 Instalación de `arduino-cli` y del core

```bash
# 1) Instalar arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
export PATH="$PATH:$HOME/bin"
arduino-cli version

# 2) Instalar el core de la placa
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli core list

# 3) Instalar las librerías
arduino-cli lib install "Adafruit ADXL345"
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib list
```

### 3.4 Compilar y cargar

```bash
# Compilar
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266.ino

# Localizar el puerto
arduino-cli board list

# Transferir a la placa
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 BusAccel-8266.ino

# Abrir el monitor serie
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

En **Linux**, el usuario debe pertenecer al grupo `dialout` para acceder al
puerto serie sin `sudo`:

```bash
sudo usermod -aG dialout $USER   # cerrar sesión y volver a entrar
```

### 3.5 Configuración de credenciales

**Ninguna.** El firmware actual no usa WiFi ni MQTT, por lo que **no hay que
crear `secrets.h`**. Los ficheros `secrets.example.h` y `secrets.h` que
pueden existir en el directorio son residuo de la fase WiFi anterior; el sketch
del ciclo 16 no los incluye y compilará sin ellos.

> ⚠️ Si compilas y obtienes un error `#include "secrets.h"` has compilado una
> versión antigua del sketch. Verifica que el archivo abierto es
> `BusAccel-8266.ino` del ciclo 16.

### 3.6 Aplicación de escritorio (monitor en tiempo real) — nueva en el ciclo 16

Documentación completa en [`app/README.md`](../app/README.md). Resumen operativo:

| Requisito | Detalle |
|---|---|
| Python | 3.8 o superior |
| Sistema | Linux (probado en Linux/WSL2), macOS o Windows |
| Permisos | usuario en el grupo `dialout` (Linux) para abrir `/dev/ttyUSB*` |
| Puerto libre | 8765 (HTTP local de la app) — configurable con `--http-port` |
| Display | cualquiera; sin pywebview cae al navegador |

**Instalación** (el `venv/` del repositorio ya está creado; si prefieres
recrearlo):

```bash
cd BusAccel-8266
python3 -m venv venv
source venv/bin/activate
pip install -r app/requirements.txt
# pyserial, pywebview, qtpy, PyQt5, PyQtWebEngine
```

**Uso:**

```bash
source venv/bin/activate
python3 app/app.py                 # placa en /dev/ttyUSB0, 115200 baud
python3 app/app.py --port /dev/ttyUSB1
python3 app/app.py --sim           # datos sintéticos a 20 Hz, sin placa
python3 app/app.py --browser       # sin ventana nativa
python3 app/app.py --http-port 9000
```

**Qué muestra:**

| Elemento | Descripción |
|---|---|
| Gráfico | Eje X = tiempo (ventana deslizante 60 s, ajustable 10–120 s), eje Y = aceleración en g |
| Curva Z | Cian con glow; opcionalmente X (ámbar) e Y (violeta) superpuestos |
| Línea de setpoint | Horizontal **punteada**, leída de los campos `setX/setY/setZ` de cada línea |
| Zonas por encima | **Ámbar** = elevación leve (0.20–0.24 g) · **Rojo** = elevación severa (≥ 0.24 g) |
| Zonas por debajo | **Azul** = caída leve (0.20–0.24 g) · **Magenta** = caída severa (≥ 0.24 g) |
| Marcadores | Punto con glow en cada detección, coloreado por símbolo, que decae en ~4 s |
| Badge | Último símbolo con su etiqueta (`REPOSO`, `ELEVACIÓN SEVERA`, …) |
| Estadísticas | Frecuencia medida (Hz), nº de muestras, tiempo transcurrido, setpoint Z |
| Contadores | Nº de `+`, `-`, `*`, `!` acumulados en la sesión |
| Eventos | Lista de las últimas detecciones con instante y valor de Z |
| Controles | Puerto, baud, Conectar, Simulador, ventana 10–120 s, pausa, limpiar, toggles X/Y |

**Notas importantes:**

* La app **parsea sólo líneas CSV de 8 campos**; el banner, el diagnóstico y
  los 20 puntos `.` del arranque se ignoran solos.
* La ventana de tiempo del gráfico no altera el buffer: la app retiene hasta
  **6000 puntos** (~5 minutos a 20 Hz) en un buffer circular.
* Los umbrales de las zonas de color están **duplicados en la app**
  (`LEVE_THRESHOLD_G`, `SEVERO_THRESHOLD_G` en `app/serial_reader.py`). Si
  cambias `ARROW_THRESHOLD_G` o `SEVERE_FACTOR` en el firmware, actualiza
  también esos dos valores para que las zonas coincidan.
* La app **no guarda los datos a disco**: cerrar la ventana pierde la sesión.
  Para archivar, captura el log por consola serie (`tee`).

---

## 4. Operación

### 4.1 Procedimiento normal

1. **Cablea** el GY-291 según §3.2.
2. **Energiza** el NodeMCU por USB.
3. **Abre la consola serie** a 115200 baud, 8N1, sin control de flujo
   — o lanza la app (§3.6), que abre el puerto por ti.
4. **Pulsa `RST`**.
5. **Espera** al mensaje `setpoint establecido:` (unos 3–4 s).
6. **Lee** el flujo de líneas CSV (o observa el gráfico de la app).


### 4.2 Secuencia de arranque detallada

```
=== BusAccel-8266 DIAG CSV (ciclo 16, t,X,Y,Z,setX,setY,setZ,sym) ===
[ADXL345] OK (begin() correcto)

=== ADXL345 DIAGNOSTICO PROFUNDO ===
WHO_AM_I    = 0xE5  (OK: 0xE5 es el ADXL345)
POWER_CTL   = 0x00  (bit3=0 -> NO en measurement mode / STANDBY)
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
```

| Fase | Qué ocurre | Si falla |
|---|---|---|
| Banner | Confirma que corre el ciclo 16 | Nada aparece → problema de puerto o baudios |
| `[ADXL345] OK` | `accel.begin()` responde | `[ADXL345] NO DETECTADO` → ver §6 |
| Diagnóstico | Lectura directa de 7 registros | Ver §6, tabla "No detecte el sensor" |
| Forzado de `POWER_CTL` | Escribe `0x08` y verifica | Ver §6, "POWER_CTL no mantiene 0x08" |
| 20 puntos `.` | Adquisición del setpoint | No avanza → sensor sin datos |
| `setpoint establecido` | Referencia fijada | Valores absurdos → ver §6 |
| Líneas CSV | Detección a 20 Hz | Silencio → ver §6, "No imprime nada" |


> **Nota importante sobre `POWER_CTL`:** la librería Adafruit ya deja el chip en
> measurement mode, así que normalmente imprimirá `0x08` desde la primera
> lectura. Si aparece `0x00`, el firmware lo corrige escribiéndolo y verifica que
> persista. Ese forzado es la reparación automática del problema más frecuente
> en clones baratos del ADXL345.

### 4.3 Interpretación del setpoint

```
setpoint establecido: X=0.01 Y=-0.16 Z=1.00
```

| Eje | Valor esperado | Significado |
|---|---|---|
| **X** (longitudinal) | ≈ 0.00 ± 0.10 | Inclinación permanentemarcha (pendiente del aparcamiento) |
| **Y** (lateral) | ≈ 0.00 ± 0.10 | Inclinación lateral permanente (suspensión descentrada) |
| **Z** (vertical) | **≈ +1.00** | Gravedad. Si el módulo está plano, debe marcar ~1 g |

| Valor observado | Diagnóstico | Acción |
|---|---|---|
| `Z ≈ 1.00` | ✅ Normal | — |
| `Z ≈ 0.00` | ❌ Sensor mal orientado, o en STANDBY sin forzar | Verificar el cableado, mirar el diagnóstico |
| `Z ≈ -1.00` | ⚠️ Módulo boca abajo | Da igual: el setpoint se ajusta, pero conviene fijarlo boca arriba |
| `Z > 2.00` | ❌ Rango mal configurado (±4g/±8g) o lecturas corruptas | Revisar `DATA_FORMAT` en el diagnóstico |
| `X` o `Y` con \|valor\| > 0.5 | ⚠️ Inclinación extrema en reposo | Nivelar el montaje |
| `Z ≈ 0.05` constante | ⚠️ Rango ±16 g (1 LSB ≈ 0.0625 g) | Revisar `DATA_FORMAT` |

### 4.4 Interpretación de la salida

Desde el ciclo 16 la salida es **una línea CSV por muestra de 50 ms**. Para leer
el recorrido "de un vistazo" como en el ciclo 15, extrae el campo 8 (`sym`):

```bash
awk -F, '/^[0-9]+,/ {printf "%s", $8}' recorrido.log; echo
# ..........+.....*......-.....!....*......+..........-....!......*.........
#            ↑     ↑            ↑      ↑       ↑     ↑            ↑     ↑
#        bache  lomo de     catenaria hoyo  lomo    hoyo      lomo   hoyo
#        leve   toro a alta  leve     profundo de toro profundo  de toro profundo
#                                      (2 flancos separados 300 ms)
```

Con el CSV tienes además información que el flujo de caracteres no daba: la
**magnitud exacta** de la desviación (`eje − setEje`), que puedes comprobar en
la octava columna del §2.5.

| Patrón | Interpretación |
|---|---|
| Líneas con `sym='.'` encadenadas | Bus en marcha muy suave o parado |
| `Z` oscilando dentro de ±0.20 g del setpoint | Frenada suave sostenida |
| `!` aislado | Un bache/hoyo de magnitud relevante |
| `*` seguido de ≥ 5 líneas con `sym='.'` y otro `*` | Dos flancos del **mismo** lomo de toro (subida y bajada) |
| `*!` o `!*` con 1–4 líneas con `.` entre medias | Rebote inercial o desnivel real de la calzada |
| `*` repetido cada 3–5 muestras | Pavimento muy deteriorado: el bus está "castañeando" |
| `+` o `-` cada 2–3 muestras | Umbral demasiado bajo para la velocidad de la vía |

> **Con 250 ms de bloqueo tras cada evento**, entre dos símbolos hay siempre al
> menos **5 líneas** (5 × 50 ms). Si ves dos símbolos separados por menos de 4
> líneas, no puede ser el mismo evento: o es un segundo evento real, o la
> constante está mal editada.

> 💡 **En la app de escritorio (§3.6) esto se ve directamente:** cada símbolo
> deja un marcador con glow en la curva, las zonas ámbar/rojo y azul/magenta
> marcan la magnitud, y los contadores de la barra lateral sustituyen a los
> `awk` de conteo.

### 4.5 Registro de un recorrido

```bash
# Capturar 10 minutos de datos con marca de tiempo
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200 | \
  tee recorrido-$(date +%Y%m%d-%H%M).log > /dev/null

# En paralelo, análisis básico
awk -F, '/^[0-9]+,/ {c[$8]++} END {for (s in c) print s, c[s]}' recorrido.log
```

Ver `README.md` §8.3 para el post-procesamiento completo en `awk` y Python.


### 4.6 Operar en condiciones adversas

| Condición | Comportamiento | Recomendación |
|---|---|---|
| Bus en movimiento al encender | El setpoint se fija sobre datos dinámicos → todo se detecta como desviación | Encender solo en reposo |
| Pérdida de alimentación del GY-291 | El firmware **no se reinicia**; el bucle de reintento imprime `[ADXL345] NO DETECTADO` una vez y sigue | Revisar cableado; recuperar y esperar `[ADXL345] OK` |
| Cable SDA flojo | Fallos intermitentes de lectura | Reforzar la conexión; cable < 20 cm |
| Ruido del motor (24 V) | Lecturas erráticas, falsos `*` | Condensador de 100 nF en VCC/GND; apantallado |
| Bus en vibración permanente | `+`/`-` constantes | Subir `ARROW_THRESHOLD_G` a 0.25–0.30 g |

---

## 5. Calibración

### 5.1 Constantes ajustables

Todas están en la sección `Constantes` de `BusAccel-8266.ino`. Modificarlas y
recompilar es el único ajuste necesario; no hay menús ni EEPROM de configuración.

| Constante | Defecto | Rango práctico | Efecto del cambio |
|---|---|---|---|
| `ARROW_THRESHOLD_G` | `0.20f` | 0.10 – 0.50 g | Umbral de detección. **Mayor → menos sensibilidad, menos falsos positivos.** Menor → más sensible, más ruido |
| `SEVERE_FACTOR` | `1.2f` | 1.05 – 2.0 | Multiplicador del umbral para definir "severo". Menor → `*`/`!` antes |
| `SEVERE_THRESHOLD_G` | derivado | — | **No editar directamente.** Se recalcula como `ARROW_THRESHOLD_G * SEVERE_FACTOR` |
| `RETURN_G` | `0.15f` | 0.05 – 0.19 g | Histeresis. **Debe ser < `ARROW_THRESHOLD_G`.** Menor → desbloqueo más exigente |
| `EVENT_LOCKOUT_MS` | `250` | 250 – 400 ms | Anulación del rebote. **Nunca por debajo de 250 ms** |
| `REST_MS` | `50` | 40 – 150 ms | Reposo sostenido. Mayor → menos falsos desbloqueos |
| `SETPOINT_N` | `20` | 10 – 50 | Media del setpoint. Mayor → arranque más lento, referencia más estable |
| `SAMPLE_INTERVAL_MS` | `10` | 10 (fijo) | Debe coincidir con el data rate del chip (100 Hz) |
| `PRINT_INTERVAL_MS` | `50` | 50 – 100 ms | Ritmo de lectura. 50 ms = 20 Hz |
| `LINE_CHARS` | `50` | — | ⚠️ **Sin uso en el ciclo 16** (residuo del ciclo 15). No lo edites |
| `LEVE_THRESHOLD_G` (app) | `0.20` | 0.10 – 0.40 | Sólo visual: umbral de las zonas ámbar/azul del gráfico |
| `SEVERO_THRESHOLD_G` (app) | `0.24` | 0.15 – 0.50 | Sólo visual: umbral de las zonas rojo/magenta del gráfico |

### 5.2 Protocolo de calibración

```
   ① Baseline    ② Lomo suave    ③ Lomo rápido    ④ Hoyo     ⑤ Verificación
   ─────────    ────────────    ────────────    ──────     ──────────────
   parado       30 km/h         60 km/h         badén      parado de nuevo
   60 s         → debe ser +     → debe ser *    → debe     setpoint idéntico
   0 eventos    → sube si hay    → sube si hay   ser !      al arranque
                 falsos          falsos
```

1. **Baseline (umbral).** Bus parado, motor en marcha lenta, 60 s. Debe haber
   **cero** eventos. Si hay, sube `ARROW_THRESHOLD_G` en pasos de 0.05 g.
2. **Severidad.** Un lomo de toro a 30 km/h debería dar `+`; a 60 km/h, `*`.
   Si ambos dan `+`, baja `SEVERE_FACTOR` a 1.1.
3. **Hoyo.** Un badén o bache conocido debe dar `!`. Si da `-`, sube
   `ARROW_THRESHOLD_G` o baja `SEVERE_FACTOR`.
4. **Verificación.** Tras el recorrido, en parada, comprueba que el nuevo
   setpoint coincide con el inicial (±0.05 g por eje).
5. **Documenta los valores** en un comentario junto a la constante.

### 5.3 Límites físicos

| Límite | Valor | Motivo |
|---|---|---|
| Resolución práctica | ~3.9 mg/LSB (10 bits en ±2 g) | `DATA_FORMAT` pone el rango en ±2 g |
| Ruido del MEMS | ±1–2 mg RMS | \_\_ del sensor, del bus y del ADC |
| Umbral mínimo razonable | 0.10 g | Por debajo, el ruido del sensor domina |
| Frecuencia máxima real | 100 Hz | `ADXL345_DATARATE_100_HZ`; por encima se repite la misma muestra |
| Rango de la banda del bus | ~0.05 – 1.5 g | Eventos de conducción urbana; por encima de 2 g saturaría el rango |

> ⚠️ **Nunca pongas `ARROW_THRESHOLD_G` por debajo de 0.10 g**: crearás una
> falsa alarma permanente por ruido cuántico del sensor.

### 5.4 Ajustes de montaje que afectan a la medición

| Ajuste | Efecto |
|---|---|
| Fijación rígida al chasis | Imprescindible. Un soporte con holgura convierte la vibración en falsos `*` |
| Módulo boca arriba, eje Z vertical | Permite que `*`/`!` signifiquen siempre "vertical" |
| Cable de menos de 30 cm | Reduce la entrada de ruido |
| Condensador de 100 nF en VCC/GND | Filtra el ruido de la línea de 24 V |
| Aislar del cableado del motor | Evita lecturas corruptas con el motor en marcha |

---

## 6. Solución de problemas

### 6.1 Problemas de arranque

| Síntoma | Causa probable | Diagnóstico | Solución |
|---|---|---|---|
| **No aparece nada** | Puerto equivocado o driver ausente | `arduino-cli board list` | Cambiar `-p`; instalar driver CH340/CP2102; en Linux, grupo `dialout` |
| **Sale basura ilegible** | Baudios incorrectos | Monitor a 115200 | Configurar 115200, 8N1, sin flow control |
| **Sale el texto del cargador ROM a 74880 baud** | Se ha pulsado `RST` durante el arranque | Cambiar a 74880 temporalmente | Normal; volver a 115200. Verás `ets Jan  8 2013,rst cause:2` |
| **`[ADXL345] NO DETECTADO`** | Cableado o alimentación del sensor | Revisar VCC/GND/SDA/SCL con el diagnóstico | Ver §6.2 |
| **`load addr 0x40100000`** | El sketch no entra en modo bootloader | Mantener `FLASH` + `RST` | Ver §6.5 |

### 6.2 Problemas de detección del sensor

| Síntoma | Causa | Solución |
|---|---|---|
| `WHO_AM_I = 0xFF` | No hay respuesta I²C (dirección 0xFF = error de `readReg`) | Revisar SDA/SCL, masa común, longitud del cable |
| `WHO_AM_I = 0xE5` pero sin datos | El chip está en STANDBY | Ver la línea siguiente; el forzado de `0x08` debería corregirlo |
| `POWER_CTL` no mantiene `0x08` | Clon de baja calidad o mala soldadura | Soldadura del módulo; considerarlo defectuoso |
| `DATA_FORMAT = 0x0B` (±16 g) | Configuración no aplicada | Verificar que `setRange()` se ejecutó; el rango por defecto es ±16 g |
| `I2C` intermitente | Cables largos, pull-ups débiles | Acortar a < 20 cm; añadir pull-ups de 4.7 kΩ a 3V3 |
| `[ADXL345] NO DETECTADO` repetido | Sensor desconectado en marcha | El firmware reintenta cada 1 s; revisar el cable |

### 6.3 Problemas de salida

| Síntoma | Causa | Solución |
|---|---|---|
| **Todo `sym='.'`** | No hay eventos: umbral alto, o el bus está parado | Bajar `ARROW_THRESHOLD_G`; comprobar que el bus se mueve |
| **Todo `*` o `!`** | Ruido, o setpoint tomado en movimiento | Volver a tomar el setpoint con el bus parado; subir umbral; condensador de 100 nF |
| **La app no recibe datos** | El puerto ya está ocupado por `arduino-cli monitor`, cutecom o PuTTY | Cerrar el otro programa; sólo un proceso puede abrir `/dev/ttyUSB*` |
| **La app da `PermissionError`** | El usuario no está en el grupo `dialout` | `sudo usermod -aG dialout $USER` y reiniciar la sesión |
| **La app no abre ventana** | Falta pywebview o el backend Qt | `pip install -r app/requirements.txt`, o usar `--browser` |
| **La gráfica sale plana** | Se ha instalado el firmware del ciclo 15 (caracteres sueltos) o de un ciclo anterior | Recargar el ciclo 16; la app sólo parsea líneas CSV de 8 campos |
| **Líneas CSV truncadas / ilegibles** | Baud incorrecto | Verificar 115200 baud en firmware y en la app |
| **Símbolos duplicados seguidos** | Rebote inercial no suprimido | Verificar que `EVENT_LOCKOUT_MS ≥ 250`; subir a 300 |
| **Símbolo cada 3 muestras** | Umbral demasiado bajo para la velocidad | Subir `ARROW_THRESHOLD_G` |
| **Zonas de color desalineadas del firmware** | `LEVE_THRESHOLD_G` / `SEVERO_THRESHOLD_G` desincronizados en la app | Actualizar esos dos valores en `app/serial_reader.py` |
| **No imprime nada tras el setpoint** | `setpointReady` no llegó a activarse | Revisar que se impresieron 20 puntos al arranque |


### 6.4 Problemas físicos / mecánicos

| Síntoma | Causa | Solución |
|---|---|---|
| Lecturas que «tiemblan» en reposo | Montaje flexible | Atornillar rígidamente al chasis |
| Falsos eventos con el bus parado | Microvibración (motor, bomba) | Subir `ARROW_THRESHOLD_G` |
| Desviación constante en `Z` | Sensor desalineado | Verificar que `Z ≈ 1.00` en el setpoint |
| Eventos que no coinciden con la carretera | Rotor mal apretado, o `setpoint` desactualizado | Recalibrar el setpoint antes del recorrido |

### 6.5 Problemas de programación (carga)

| Síntoma | Solución |
|---|---|
| `Failed to connect to ESP8266: Timed out` | Mantén `FLASH`, pulsa `RST`, suelta `FLASH` a los 2 s, reintenta la carga |
| `Error: Unknown FQBN` | `arduino-cli core install esp8266:esp8266` |
| `error: 'Adafruit_ADXL345_U.h' file not found` | `arduino-cli lib install "Adafruit ADXL345" "Adafruit Unified Sensor"` |
| `Serial has been declared...` | Conectar varios Sketch a la vez; compilar solo uno |
| `Too many dynamic allocations` | No debería ocurrir; el sketch no asigna memoria dinámica en `loop()` |

### 6.6 Procedimiento de diagnóstico rápido

```
 1. ¿Abre la consola a 115200?
       NO → problema de puerto/baudios  (§6.1)
       SÍ ↓
 2. ¿Aparece el banner "=== BusAccel-8266 DIAG"?
       NO → sketch antiguo cargado, o comunicación corrupta
       SÍ ↓
 3. ¿Aparece "WHO_AM_I = 0xE5"?
       NO → revisar cableado del sensor  (§6.2)
       SÍ ↓
 4. ¿Aparece "[POWER_CTL] OK: 0x08 persiste"?
       NO → clon defectuoso o mala soldadura
       SÍ ↓
 5. ¿Imprime 20 puntos y "setpoint establecido"?
       NO → el sensor entrega datos, revisar la consola
       SÍ ↓
 6. ¿Aparecen líneas `t,X,Y,Z,setX,setY,setZ,sym`?
       NO → usar RST; si persiste, recargar el firmware
       SÍ ↓
 7. ¿La salida refleja lo que hace el bus?
       NO → ajustar umbrales  (§5.2) o montaje  (§5.4)
       SÍ ✓ → dispositivo operativo
```

---

## 7. Glosario

| Término | Definición |
|---|---|
| **ADXL345** | Acelerómetro MEMS de 3 ejes de Analog Devices. Rango ±2/4/8/16 g, hasta 3200 Hz |
| **GY-291** | Placa breakout Zeus que integra el ADXL345 con conector de 4 pines |
| **Setpoint** | Vector de referencia (X, Y, Z) en g tomado como promedio de 20 lecturas en reposo. Toda la detección se hace respecto a él |
| **Desviación (`dev`)** | Diferencia entre la lectura actual y el setpoint en un eje: `gx - spX` |
| **Eje ganador** | Eje con mayor valor absoluto de desviación. Decide el carácter impreso |
| **Umbral leve** (`ARROW_THRESHOLD_G`) | Desviación mínima (0.20 g) para considerar que hay un evento |
| **Umbral severo** (`SEVERE_THRESHOLD_G`) | Desviación a partir de la cual el evento se considera grave (0.24 g = 1.2×) |
| **Severidad** | Clasificación leve/severo deducida de la **magnitud** de la desviación, no solo del sentido |
| **Histeresis** (`RETURN_G`) | Banda de reposo (0.15 g) más estrecha que el umbral de disparo. Evita oscilaciones en el borde |
| **Rebote inercial** | Segunda aceleración en sentido contrario tras un evento, 100–200 ms después, causada por la suspensión |
| **Lockout** (`EVENT_LOCKOUT_MS`) | Tiempo mínimo (250 ms) que el detector permanece bloqueado tras un evento, para anular el rebote |
| **Reposo sostenido** (`REST_MS`) | Tiempo (50 ms) que los tres ejes deben permanecer dentro de ±0.15 g para desbloquear |
| **Máquina de estados** | Lógica `ST_IDLE` / `ST_LOCKED` que garantiza un único símbolo por evento |
| **Burst read** | Lectura I²C de varios bytes consecutivos (`DATAX0..DATAZ1`) en una sola transacción |
| **WHO_AM_I / DEVID** | Registro `0x00` del ADXL345. Siempre vale `0xE5`. Otro valor = clon o error de lectura |
| **POWER_CTL** | Registro `0x2D`. El bit 3 (`0x08`) activa el modo de medición |
| **DATA_FORMAT** | Registro `0x31`. Los bits 0–1 seleccionan el rango (±2/4/8/16 g) |
| **BW_RATE** | Registro `0x2C`. Selecciona el ancho de banda y la frecuencia de datos |
| **LSB/g** | Bits menos significativos por gramo. En ±2 g, 1 LSB = 1/256 g = 3.9 mg |
| **LZOFY / LSFIR** | Filtros de paso bajo del ADXL345. No se configuran: el chip usa su filtro por defecto |
| **millis()** | Temporizador de Arduino. Toda la temporización del firmware se basa en él (sin `delay()` en `loop()`) |
| **ASCII puro** | Los cinco símbolos (`+*-!.`) son bytes imprimibles estándar, sin UTF-8 ni secuencias de escape |
| **ESP8266EX** | Microcontrolador principal del NodeMCU (80 MHz, WiFi integrado, flash 4 MB) |
| **GPIO** | Pin general de entrada/salida. En el NodeMCU, `D1` = GPIO5 y `D2` = GPIO4 |
| **I²C** | Bus de comunicación de 2 hilos (SDA + SCL). El ADXL345 está en la dirección `0x53` |
| **Pull-up** | Resistencia a +3.3 V que mantiene las líneas I²C en alto. Ya incluidas en el GY-291 |
| **FQBN** | *Fully Qualified Board Name*. Identificador de placa en arduino-cli: `esp8266:esp8266:nodemcuv2` |
| **EEPROM** | Memoria no volátil. **No se usa en el ciclo 16**; persistencia es fase futura |
| **MQTT** | Protocolo de mensajería ligero. **No se usa en el ciclo 16**; publicación cada 1 min es fase futura |

---

## 8. Compatibilidad con otras placas

### 8.1 Compatibilidad verificada

| Placa | FQBN | Compat. | Cambios necesarios |
|---|---|---|---|
| **NodeMCU ESP8266 (ESP-12E)** | `esp8266:esp8266:nodemcuv2` | ✅ **Objetivo** | Ninguno |
| **NodeMCU 0.9 (ESP-12)** | `esp8266:esp8266:nodemcuv` | ✅ Alta | Ninguno. GPIO4/5 idénticos |
| **Wemos D1 mini** | `esp8266:esp8266:d1_mini` | ✅ Alta | Ninguno. SDA=`D2`/GPIO4, SCL=`D1`/GPIO5 — coinciden |
| **Wemos D1 mini Pro** | `esp8266:esp8266:d1_mini_pro` | ✅ Alta | Ninguno |
| **Generic ESP8266 (ESP-12E/12F)** | `esp8266:esp8266:generic` | ✅ Alta | Mapear SDA/SCL a GPIO4/GPIO5 en tu placa |
| **ESP-01** | `esp8266:esp8266:esp01_1m` | ⚠️ Baja | Sin headers; requiere programador FTDI externo. I²C ok pero sin USB propio |
| **LOLIN D32 (ESP32)** | `esp32:esp32:d32` | ⚠️ Media | `Wire.begin(21, 22)` (SDA=21, SCL=22); el resto de la lógica es portable |
| **ESP32 DevKit v1** | `esp32:esp32:esp32` | ⚠️ Media | Igual que arriba; revisar pines I²C por variante |
| **Arduino Uno R3** | `arduino:avr:uno` | ❌ Baja | Sin I²C nativo (usar `Wire` para ATmega328P); falta `millis()` de 32 bits (ok). **El sketch funciona con adaptaciones menores** |
| **Arduino Nano 33 IoT** | `arduino:mbed_nano:nanonano33iot` | ❌ Baja | Pines I²C por defecto distintos; `Serial` no es USB nativo (usar `SerialUSB`) |
| **Arduino MKR1000** | `arduino:mbed_nano:mkr1000wifi` | ❌ Baja | Sin I²C por defecto; usar `Wire1` y renombrar llamadas |

### 8.2 Portabilidad del firmware

El sketch es **casi completamente portable**. Lo que realmente depende de la
placa es:

| Elemento | Dependencia | Cómo adaptarlo |
|---|---|---|
| Pines I²C | `PIN_SDA = 4`, `PIN_SCL = 5` | Cambiar las dos constantes y `Wire.begin(SDA, SCL)` |
| Address I²C del sensor | `ADXL345_I2C_ADDR = 0x53` | Constante; igual en toda la familia ADXL34x |
| Formato de la serie | `Serial` | En placas con USB nativo, usar `SerialUSB` |
| Tamaño de `millis()` | `unsigned long` | En AVR es `unsigned long` (32 bits); compatible |
| `Serial.printf` | Extensión de ESP8266 | En AVR usar `Serial.print` + `F()` o `snprintf` |
| `Wire` | Incluida en todos los cores | Sin cambios |

La lógica de negocio —setpoint, desviación, eje ganador, severidad, máquina de
estados `ST_IDLE`/`ST_LOCKED`— es **independiente de la plataforma** y se
porta tal cual.

### 8.3 Sensores alternativos

| Sensor | Conexión | Cambios necesarios |
|---|---|---|
| **ADXL345 (GY-291)** | I²C `0x53` | Ninguno — es el actual |
| **ADXL346** | I²C `0x53` | Ninguno; registros compatibles |
| **MMA8451** | I²C `0x1C` | Reescribir `readRawAxes()` (registros y endianness distintos) y el diagnóstico |
| **MPU6050** | I²C `0x68` | Reescribir lectura y diagnóstico; contiene giroscopio (no usado) |
| **BMI270** | I²C `0x68` | Reescribir lectura; mejor resolución, pero otro mapa de registros |

> Todos los sensores de la lista son de 3 ejes y ±2 g (o configurables a ello).
> La lógica de setpoint/desviación/severidad es **independiente del sensor**;
> solo habría que cambiar `readRawAxes()`, la escala de conversión y la sección
> de diagnóstico.

---

## 9. Referencias

| Documento | Contenido |
|---|---|
| **`README.md`** | Descripción, principio de funcionamiento, pinout, diagrama, ejemplos, app de escritorio |
| **`app/README.md`** | Manual de la aplicación de escritorio: instalación, opciones, interfaz, problemas |
| **`docs/developer.md`** | Arquitectura del código, máquina de estados, funciones, arquitectura de la app, extensión |
| **`PLAN.md`** | Plan de diseño original y detalle de la fase futura MQTT/EEPROM |
| **`AUDIT_REPORT.md`** | Informe de auditoría del proyecto |
| Datasheet ADXL345 (Analog Devices) | Registro `0x00..0x3D`, escalas, BW_RATE |
| Guía I²C del ESP8266 | `Wire.begin(SDA, SCL)`, inherent pull-ups |
| `arduino-cli` docs | `compile`, `upload`, `monitor`, `board list` |
| Documentación de pyserial / pywebview | `serial.Serial`, `webview.create_window` |

---

*Documento generado para el firmware del **ciclo 16: modo diagnóstico serial con
salida CSV** (`t,X,Y,Z,setX,setY,setZ,sym`) y para la **aplicación de escritorio**
`app/` que consume ese CSV. Toda referencia a WiFi, servidor web servido por el
ESP8266, MQTT o EEPROM es **histórica** o se refiere a la **fase futura planificada
y no implementada**.*
