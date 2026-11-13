# BusAccel-8266-v2 — Monitor de escritorio (GY-291 / ADXL345)

Aplicación local en Python que **lee, grafica y resume** lo que el firmware
`BusAccel-8266-v2` produce. Funciona contra la placa real, contra un simulador, o
contra un `.log` de captura guardado.

Todo es local: **no hay CDN ni internet**. El backend levanta un servidor HTTP en
`127.0.0.1` y el frontend se abre en una ventana `pywebview` o, si eso falla, en
el navegador.

## Fuentes de datos

La app tiene **tres fuentes**, y se eligen con `--port` o con un argumento propio:

| Fuente | Cómo se activa | Para qué |
|---|---|---|
| **Puerto serie** | `--port /dev/ttyUSB0` | la placa conectada |
| **Simulador** | `--sim` o `--port SIM` | probar la interfaz sin placa; emite ventanas de evento v2 sintéticas |
| **Captura** | `--replay fichero.log`, o el botón **Captura…** | revivir una sesión guardada |

El botón **Captura…** de la barra superior abre un diálogo para elegir el fichero;
si `tkinter` no está disponible, cae a un `<input type=file>` del navegador.

## Instalación

```bash
cd BusAccel-8266-v2/app
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Ejecución

```bash
# contra la placa
python3 app.py --port /dev/ttyUSB0 --baud 115200

# sin placa
python3 app.py --sim

# reviviendo una captura, 4 veces más rápido que el original
python3 app.py --replay captura.log --speed 4
```

### Opciones

| Argumento | Descripción |
|---|---|
| `--port` | Puerto serial (por defecto `/dev/ttyUSB0`; `SIM` = simulador) |
| `--baud` | Baud rate (por defecto `115200`) |
| `--sim` | Arrancar en modo simulador |
| `--replay FICHERO` | Reproducir un `.log` en vez de leer el puerto |
| `--speed` | Multiplicador de velocidad del replay (0.1 … 50) |
| `--browser` | Forzar modo navegador (sin pywebview) |
| `--http-port` | Puerto del servidor local (por defecto `8765`) |
| `--debug` | Devtools de pywebview |

## Formatos de línea que entiende

El parser acepta los **dos** formatos, para que la app sirva tanto con v1 como
con v2. Cualquier otra línea (banner, diagnóstico, JSON, ruido) se descarta
automáticamente.

### Formato A — serie continuo (v1, `MODE_SERIAL_CSV 1`)

```
t,X,Y,Z,setX,setY,setZ,sym
123456,0.14,0.28,0.88,0.14,0.28,0.88,.
```

Ocho campos. Los tres ejes vienen **ya en g**, como float.

### Formato B — ventana de evento (v2, modo normal)

```
EV,t0,Xraw,Yraw,Zraw,dt,sym
EV,37795,30,33,235,10,.
```

| Campo | Significado |
|---|---|
| `t0` | **inicio de la ventana**, en ms de uptime. **Se repite en las 200 líneas** |
| `Xraw` `Yraw` `Zraw` | **cuentas crudas** del ADXL345 (÷256 = g) |
| `dt` | ms **reales** entre esta muestra y la anterior |
| `sym` | el símbolo va **solo en la muestra que disparó** |

> **La reconstrucción del tiempo.** Como `t0` se repite en toda la ventana, el
> tiempo real de cada muestra se reconstruye **sumando los `dt`** hacia adelante
> desde `t0`. Por eso el gráfico reproduce la duración real del golpe y no una
> línea plana: si se usara `t0` como eje, los 200 puntos se apilarían en el mismo
> instante.

### Las líneas JSON `[MQTT-OUT]`

El firmware en ensayo en seco imprime cada mensaje MQTT que *enviaría*. La app
las parsea con `json.loads` y las clasifica:

| Tipo | Cómo se reconoce | Qué hace la app |
|---|---|---|
| **setpoint** | `x`, `y`, `z` escalares | cachea el setpoint de Z para dibujarlo |
| **summary** | tiene `sym` **y** `id` | lo añade al panel **Eventos detectados** |
| **chunk** | tiene `id`, `i`, `up0` | lo reconoce y lo ignora (los datos ya llegan por `EV,`) |

Los resúmenes se muestran con su `id`, su `up` y su pico en g.

## Qué se ve

- **Gráfico del eje Z** en g, con ventana deslizante (10–120 s).
  - **Línea punteada**: el setpoint de Z.
  - **Zonas sombreadas**: ámbar para la banda leve (±0.20 g), rojo para la severa
    (≥ 0.24 g), por encima y por debajo.
  - Ejes X e Y opcionales con los toggles.
- **Última detección**: badge grande con el símbolo y su significado.
- **Estadísticas**: Hz, número de muestras, tiempo y setpoint Z.
- **Contadores** por símbolo.
- **Últimas detecciones**: los cruces, agrupados por renglón cuando son
  consecutivos.
- **Eventos detectados**: un renglón por evento completo, con `id`, `up` y pico.

### Sobre la medida de Hz

La frecuencia solo se calcula **cuando el hueco entre muestras es menor de 200
ms**. En v2 no llega nada durante el reposo, y si se midiera la cadencia contando
los huecos, el indicador marcaría un valor falsísimo.

### Sobre la retención

Los datos se guardan **por tiempo** (los últimos 300 s), con un suelo de 4000
puntos y un techo duro de 40000. No se cuenta por muestras a propósito: **durante
el reposo no llega nada**, así que un buffer contado en muestras perdería los
eventos anteriores en cuestión de minutos.

## Exportar una captura de CuTTeM

Para guardar una sesión y verla después:

1. En CuTTeM, **Recording → Start** (o el icono de grabar).
2. Reproduce la sesión que te interese.
3. **Recording → Stop** y guarda el `.log`.
4. Ábrelo con **Captura…** en la app, o con `python3 app.py --replay captura.log`.

### Las marcas de tiempo del log

El replay usa la marca que CuTTeM pone delante de cada línea para **reproducir el
ritmo original**: si el golpe duró 200 ms, se dibuja en 200 ms. Se aceptan varios
formatos:

| Formato | Ejemplo | Se interpreta como |
|---|---|---|
| `T+` con milisegundos | `[T+004.971]` | 4971 ms |
| `T+` con minutos | `[T+1:02.345]` | 62345 ms |
| Hora `HH:MM:SS.mmm` | `01:02:03.456 ` | 3723456 ms |
| Segundos con `s` | `[62.345s]` | 62345 ms |

Una línea **sin** marca se reproduce tal cual, sin romper el ritmo. El separador
entre dos líneas marcadas se divide por `--speed`, de modo que `--speed 20`
reproduce la sesión veinte veces más rápido.

## API HTTP local

Servida en `127.0.0.1`, pensada para depurar, no para exponerse:

| Ruta | Qué hace |
|---|---|
| `GET /api/state?since=N` | estado + puntos nuevos desde el índice N |
| `GET /api/ports` | puertos series disponibles |
| `GET /api/connect?port=&baud=` | conectar (o `SIM`) |
| `GET /api/disconnect` | desconectar |
| `GET /api/clear` | vaciar buffer y estadísticas |
| `GET /api/replay?src=FICHERO` | reproducir un log |
| `POST /api/replay-text` | reproducir el cuerpo de la petición como log |
| `GET /api/speed?speed=N` | cambiar la velocidad del replay |
| `GET /api/open` | diálogo nativo para elegir fichero |

> El servidor escucha **solo en `127.0.0.1`**. No lo expongas: no hay
> autenticación, y `/api/replay-text` acepta escritura.

## Solución de problemas

- **`No se pudo abrir /dev/ttyUSB0: … PermissionError`** — añade tu usuario al
  grupo `dialout` y vuelve a iniciar sesión:
  ```bash
  sudo usermod -aG dialout $USER
  ```
- **`Device or resource busy`** — otro programa tiene el puerto (CuTTeM,
  minicom, un `arduino-cli monitor`). Ciérralo.
- **El gráfico no recibe datos** — comprueba el formato. En v2 las muestras llegan
  **solo dentro de una ventana de evento**, así que sin un golpe contra la mesa
  la gráfica se queda vacía: es el comportamiento correcto, no un fallo.
- **El replay va a destiempo** — el log no tiene marcas de tiempo reconocibles, o
  el fichero está muy fragmentado. Prueba con `--speed 1`.
- **pywebview no abre ventana** — instala `PyQt5` y `PyQtWebEngine` (ya están en
  `requirements.txt`) o usa `--browser`.
