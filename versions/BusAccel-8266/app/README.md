# BusAccel-8266 — Monitor de escritorio (GY-291 / ADXL345)

Aplicación de escritorio que grafica **en tiempo real** las lecturas del
acelerómetro **GY-291 (ADXL345)** conectado a una **NodeMCU ESP8266** que las
envía por el puerto serial a **115200 baud**.

La interfaz es **oscura, con neón/glow**, y muestra:

- Curva del **eje Z** (g) sobre una ventana deslizante de tiempo (default 60 s).
- **Línea punteada** = setpoint (reposo) del eje Z.
- **Zonas de magnitud** sobre/debajo del setpoint:
  - **Ámbar** = desviación leve (0.20–0.24 g) → símbolos `+` / `-`
  - **Rojo/magenta** = desviación severa (≥ 0.24 g) → símbolos `*` / `!`
- **Marcadores con glow** en cada detección, **badge** del último símbolo,
  contadores y lista de eventos recientes.

## Stack

- **Backend**: Python + `pyserial` (lectura del puerto y parseo CSV).
- **Ventana**: `pywebview` con backend Qt (`PyQt5` + `PyQtWebEngine`).
  Si pywebview no puede abrir ventana, la app **cae automáticamente** al
  navegador por defecto.
- **Frontend**: HTML/CSS/JS con **canvas nativo**, sin CDN (funciona sin
  internet). Servido por un `ThreadingHTTPServer` local en `127.0.0.1`.

## Formato de datos esperado (firmware ciclo 16)

El firmware envía al arranque texto libre (banner, diagnóstico, puntos `.`,
`setpoint establecido: …`) y luego una línea CSV por tick a **20 Hz**:

```
t,X,Y,Z,setX,setY,setZ,sym
123456,0.14,0.28,0.88,0.14,0.28,0.88,.
```

- `t` = millis() en ms (entero)
- `X,Y,Z` = aceleración en g (float)
- `setX,setY,setZ` = setpoint por eje en g
- `sym` = `.` reposo · `+` elevación leve · `*` elevación severa ·
  `-` caída leve · `!` caída severa

**Cualquier línea que no sea CSV (banner, diagnóstico, puntos de adquisición)
se ignora automáticamente** (solo se parsean líneas de 8 campos cuyo 1º campo
es un entero).

## Instalación

```bash
cd BusAccel-8266
python3 -m venv venv
source venv/bin/activate
pip install -r app/requirements.txt
```

## Ejecución

```bash
# Con la placa en /dev/ttyUSB0 (default)
python3 app/app.py

# Puerto específico
python3 app/app.py --port /dev/ttyUSB1

# Modo simulador (datos sintéticos a 20 Hz, sin placa)
python3 app/app.py --sim

# Forzar navegador en lugar de ventana nativa
python3 app/app.py --browser
```

### Opciones

| Argumento | Descripción |
|---|---|
| `--port` | Puerto serial (default `/dev/ttyUSB0`; `SIM` = simulador) |
| `--baud` | Baud rate (default `115200`) |
| `--sim` | Arrancar en modo simulador |
| `--browser` | Forzar modo navegador (sin pywebview) |
| `--http-port` | Puerto del servidor local (default `8765`) |
| `--debug` | Devtools de pywebview |

## Uso en la interfaz

1. Selecciona el puerto (botón desplegable) y pulsa **Conectar**.
2. Si el puerto no existe / está ocupado / hay error de permisos, el estado
   se muestra en rojo en la cabecera y puedes **reconectar** o elegir otro
   puerto. El botón **Simulador** permite probar la interfaz sin placa.
3. Ajusta la **ventana de tiempo** (10–120 s) con el deslizador.
4. Muestra/oculta ejes X e Y con los toggles; pausa con ❚❚; limpia con
   **Limpiar**.

## Solución de problemas

- **`No se pudo abrir /dev/ttyUSB0: could not open port ... PermissionError`**:
  añade tu usuario al grupo `dialout`:
  ```bash
  sudo usermod -aG dialout $USER
  # (cerrar sesión y volver a entrar)
  ```
- **El gráfico no recibe datos pero cutecom sí los muestra**: comprueba que el
  firmware instalado sea el ciclo 16 (formato CSV `t,X,Y,Z,setX,setY,setZ,sym`);
  el firmware ciclo 15 (caracteres sueltos) no es parseable.
- **pywebview no abre ventana**: instala `PyQt5` y `PyQtWebEngine` (ya están en
  `requirements.txt`) o usa `--browser`.