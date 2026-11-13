"""Lector de la fuente de datos para BusAccel-8266-v2.

Se encarga de:
- Abrir/cerrar el puerto serial (pyserial) en un hilo en segundo plano.
- Leer lineas de los DOS formatos que emite el firmware:
  * ``t,X,Y,Z,setX,setY,setZ,sym``      (v1 / ``MODE_SERIAL_CSV 1``, 20 Hz)
  * ``EV,t0,Xraw,Yraw,Zraw,dt,sym``     (v2, ventana de evento a 100 Hz)
- Interpretar las lineas JSON ``[MQTT-OUT] {...}`` del ensayo en seco para
  extraer el **resumen de cada evento** (id, simbolo, pico, desviacion) y el
  **setpoint**, de modo que la interfaz pueda listar eventos y no solo muestras.
- IGNORAR cualquier linea que no encaje en ninguno de los formatos.
- Mantener un buffer circular con los ultimos puntos y estadisticas de
  deteccion por simbolo.
- Proporcionar tres fuentes alternativas para trabajar sin placa fisica:
  ``SIM`` (sintetico en formato v2), ``REPLAY:<fichero>`` (captura de sesion
  de CuTTeM reproducida linea a linea) y el puerto real.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional

try:
    import serial
    from serial.tools import list_ports

    _SERIAL_AVAILABLE = True
except Exception:  # pragma: no cover - pyserial no instalado
    _SERIAL_AVAILABLE = False

# Simbolos que el firmware (ciclo 16) puede emitir
VALID_SYMBOLS = (".", "+", "-", "*", "!")

# Desviacion (g) usada por el firmware para clasificar leve/severo
LEVE_THRESHOLD_G = 0.20
SEVERO_THRESHOLD_G = 0.24

# Maximo duro de puntos retenidos en memoria (techo de seguridad)
MAX_POINTS = 40000

# Retencion por tiempo: se descartan los puntos mas viejos que esto, pero
# nunca se baja de ``MIN_KEEP_POINTS`` (para no perder eventos completos).
KEEP_SECONDS = 300.0
MIN_KEEP_POINTS = 4000

# Tamaño tipico de una ventana de evento del firmware v2 (PRE_N + POST_N)
EVENT_POINTS = 200

# Numero de resúmenes de evento conservados para la lista lateral
MAX_SUMMARIES = 40


class Point:
    """Un punto de datos parseado de una linea CSV."""

    __slots__ = ("t", "x", "y", "z", "set_x", "set_y", "set_z", "sym")

    def __init__(
        self,
        t: int,
        x: float,
        y: float,
        z: float,
        set_x: float,
        set_y: float,
        set_z: float,
        sym: str,
    ) -> None:
        self.t = t          # millis() del dispositivo
        self.x = x          # g
        self.y = y          # g
        self.z = z          # g
        self.set_x = set_x  # g
        self.set_y = set_y  # g
        self.set_z = set_z  # g
        self.sym = sym      # '.' '+' '-' '*' '!'

    def to_dict(self) -> Dict:
        """Serializacion compacta para el frontend."""
        return {
            "t": self.t,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(self.z, 3),
            "sx": round(self.set_x, 3),
            "sy": round(self.set_y, 3),
            "sz": round(self.set_z, 3),
            "c": self.sym,
        }


def parse_line(line: str, state: Optional[Dict] = None) -> Optional[Point]:
    """Parsea una linea del firmware. Acepta los DOS formatos de v2.

    **Formato A - serie puro** (``MODE_SERIAL_CSV 1``, 8 campos)::

        t,X,Y,Z,setX,setY,setZ,sym
        123456,0.14,0.28,0.88,0.14,0.28,0.88,.

    **Formato B - ventana de evento** (``MODE_SERIAL_CSV 0``, 6 campos + prefijo)::

        EV,t0,Xraw,Yraw,Zraw,dt,sym
        EV,37795,30,33,235,10,.

    En el formato B los tres ejes llegan como **cuentas crudas del ADXL345**
    (se dividen por 256 para obtener g), el campo ``t0`` es el mismo en las 200
    lineas de la ventana, y el tiempo real de cada muestra se reconstruye
    sumando los ``dt``. Para eso hace falta ``state``, un dict que el lector
    mantiene entre lineas; si es ``None`` el formato B no se interpreta.

    Cualquier linea que no cumpla el formato (banner, diagnostico, puntos de
    adquisicion, JSON de MQTT, etc.) devuelve ``None`` y se descarta.
    """
    if line.startswith("EV,"):
        return _parse_event_line(line, state)

    parts = line.split(",")
    if len(parts) != 8:
        return None
    try:
        t = int(parts[0].strip())
        x = float(parts[1].strip())
        y = float(parts[2].strip())
        z = float(parts[3].strip())
        set_x = float(parts[4].strip())
        set_y = float(parts[5].strip())
        set_z = float(parts[6].strip())
        sym = parts[7].strip()
    except ValueError:
        return None
    if sym not in VALID_SYMBOLS:
        return None
    return Point(t, x, y, z, set_x, set_y, set_z, sym)


def _parse_event_line(line: str, state: Optional[Dict]) -> Optional[Point]:
    """Interpreta una linea ``EV,`` de la ventana de evento.

    El instante ``t0`` es el de INICIO de la ventana y se repite en las 200
    lineas, asi que no sirve como eje temporal por si solo: se avanza un
    acumulador con los ``dt``. Cuando cambia ``t0`` estamos ante una ventana
    nueva, y el acumulador vuelve a cero.
    """
    if state is None:
        return None

    parts = line.split(",")
    if len(parts) != 7:
        return None
    try:
        t0 = int(parts[1].strip())
        x_raw = float(parts[2].strip())
        y_raw = float(parts[3].strip())
        z_raw = float(parts[4].strip())
        dt = int(parts[5].strip())
        sym = parts[6].strip()
    except ValueError:
        return None
    if sym not in VALID_SYMBOLS:
        return None

    # Ventana nueva -> reiniciar el acumulador de tiempo
    if state.get("t0") != t0:
        state["t0"] = t0
        state["acc"] = 0

    t = t0 + state.get("acc", 0)
    state["acc"] = state.get("acc", 0) + dt

    set_z = state.get("spz", 0.0)
    return Point(t, x_raw / 256.0, y_raw / 256.0, z_raw / 256.0,
                 0.0, 0.0, set_z, sym)


def new_event_state() -> Dict:
    """Estado inicial para el parser de la ventana de evento."""
    return {"t0": None, "acc": 0, "spz": 0.0}


# CuTTeM y otros monitores anteponen la marca de tiempo a cada linea
# recibida, por ejemplo ``[T+12.345] EV,37795,...``. Se separa para poder
# calibrar el ritmo de la reproduccion.
_STAMP_TPLUS = re.compile(r"^\s*[\[(]?\s*T\s*\+\s*(.+)$")
_STAMP_HMS = re.compile(r"^\s*[\[(]?\s*(\d+):(\d+):(\d+)[.,](\d+)\s*[\])]?\s?")
_STAMP_SIMPLE = re.compile(r"^\s*[\[(]?\s*(\d+)[.,](\d+)\s*s[\])]?\s?")
_TIME_PARTS = re.compile(r"^(?:(\d+):)?(\d+):(\d+)[.,](\d+)$")
_TIME_SECS = re.compile(r"^(\d+)[.,](\d+)$")


def _time_to_ms(text: str):
    """Convierte ``h:mm:ss.mmm``, ``mm:ss.mmm`` o ``ss.mmm`` a milisegundos."""
    m = _TIME_PARTS.match(text)
    if m:
        h = int(m.group(1) or 0)
        return ((h * 60 + int(m.group(2))) * 60 + int(m.group(3))) * 1000 \
            + int(m.group(4))
    m = _TIME_SECS.match(text)
    if m:
        return int(m.group(1)) * 1000 + int(m.group(2))
    return None


def _split_replay_stamp(line: str):
    """Separa la marca de tiempo inicial de una linea de captura.

    Devuelve ``(texto, milisegundos)``; ``milisegundos`` es ``None`` si la
    linea no lleva marca. Se aceptan los formatos habituales::

        [T+1:02.345] EV,...      -> 62345 ms
        [T+62.345] EV,...        -> 62345 ms
        T+1:02:03.456 EV,...     -> 3723456 ms
        01:02:03.456 EV,...      -> 3723456 ms
        [62.345s] EV,...         -> 62345 ms

    Si el prefijo no encaja con ninguno, la linea se devuelve intacta y sin
    marca: el replay sigue funcionando, solo que a velocidad maxima.
    """
    line = line.rstrip("\r\n")

    m = _STAMP_TPLUS.match(line)
    if m:
        rest = m.group(1)
        # El token de tiempo termina en el cierre del corchete o en el primer
        # espacio, lo que aparezca primero.
        cut = len(rest)
        for sep in ("]", " "):
            i = rest.find(sep)
            if 0 <= i < cut:
                cut = i
        token = [REDACTED]
        tail = rest[cut + 1:] if cut < len(rest) else ""
        ms = _time_to_ms(token.strip())
        if ms is not None:
            return tail.lstrip(), ms

    m = _STAMP_HMS.match(line)
    if m:
        ms = ((int(m.group(1)) * 60 + int(m.group(2))) * 60 + int(m.group(3))) * 1000 \
            + int(m.group(4))
        return line[m.end():], ms

    m = _STAMP_SIMPLE.match(line)
    if m:
        ms = int(m.group(1)) * 1000 + int(m.group(2))
        return line[m.end():], ms

    return line, None


def list_serial_ports() -> List[str]:
    """Devuelve la lista de puertos seriales disponibles."""
    if not _SERIAL_AVAILABLE:
        return []
    try:
        return sorted({p.device for p in list_ports.comports()})
    except Exception:
        return []


REPLAY_PREFIX = "REPLAY:"
REPLAY_PORT_HINT = "REPLAY:<fichero.log>"


class SerialReader:
    """Hilo que lee la fuente de datos y alimenta un buffer circular.

    Fuentes admitidas en :meth:`connect`:

    ``/dev/ttyUSB0``
        Puerto serial real (pyserial).
    ``SIM``
        Simulador sintetico que emite ventanas de evento en formato v2.
    ``REPLAY:<fichero>``
        Reproduce linea a linea una captura de sesion guardada (por ejemplo
        el log que exporta CuTTeM), sin tocar ningun puerto. Es la via para
        ver el grafico de una jornada ya registrada.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self._points: Deque[Point] = deque(maxlen=MAX_POINTS)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._serial = None  # type: ignore[var-annotated]
        self.status: str = "idle"          # idle|connecting|connected|error|simulator|replay
        self.error: Optional[str] = None
        self.last_raw: str = ""
        self.last_point: Optional[Point] = None
        self._t0: Optional[int] = None
        self._wrap_offset: int = 0
        self._last_t: Optional[int] = None
        self._ev_state: Dict = new_event_state()
        self._counts: Dict[str, int] = {s: 0 for s in VALID_SYMBOLS}
        self._connected_at: float = 0.0
        self._bytes_read: int = 0
        self._sim_mode: bool = False
        self._replay_path: Optional[str] = None
        self._replay_total: int = 0
        self._replay_pos: int = 0
        self._summaries: Deque[Dict] = deque(maxlen=MAX_SUMMARIES)
        self._summaries_seen: int = 0
        self._replay_rerun = threading.Event()
        self._replay_speed: float = 1.0   # multiplicador de velocidad del replay

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------
    def connect(self, port: Optional[str] = None, baud: Optional[int] = None) -> None:
        """Conecta a la fuente indicada.

        Si ya hay un hilo activo, primero lo detiene. Acepta un puerto real,
        ``SIM`` o ``REPLAY:<fichero>``.
        """
        self.disconnect()
        if port:
            self.port = port
        if baud:
            self.baud = baud

        if self.port == "SIM":
            self._sim_mode = True
            self._replay_path = None
            self.status = "simulator"
            self.error = None
            self._start_thread(self._simulator_loop)
            return

        if self.port.startswith(REPLAY_PREFIX):
            path = self.port[len(REPLAY_PREFIX):] or self._replay_path
            if not path:
                self.status = "error"
                self.error = "REPLAY sin fichero: " + REPLAY_PORT_HINT
                return
            if not os.path.isfile(path):
                self.status = "error"
                self.error = f"No existe el fichero de captura: {path}"
                return
            self._sim_mode = False
            self._replay_path = path
            self.status = "connecting"
            self.error = None
            self._start_thread(self._replay_loop)
            return

        self._sim_mode = False
        self._replay_path = None
        if not _SERIAL_AVAILABLE:
            self.status = "error"
            self.error = "pyserial no esta instalado (pip install pyserial)"
            return
        self.status = "connecting"
        self.error = None
        self._start_thread(self._serial_loop)

    def disconnect(self) -> None:
        """Detiene el hilo y cierra el puerto."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._close_serial()
        if self.status not in ("error",):
            self.status = "idle"

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def clear(self) -> None:
        """Limpia el buffer de puntos y las estadisticas.

        En modo ``REPLAY:`` reinicia además el cursor del fichero, para poder
        volver a ver la misma captura desde el principio.
        """
        with self._lock:
            self._points.clear()
            self._counts = {s: 0 for s in VALID_SYMBOLS}
            self._t0 = None
            self._wrap_offset = 0
            self._last_t = None
            self._ev_state = new_event_state()
            self.last_point = None
            self._summaries.clear()
            self._summaries_seen = 0
        self._replay_pos = 0
        self._replay_rerun.set()

    def get_points_since(self, index: int) -> List[Dict]:
        """Devuelve los puntos nuevos desde ``index``.

        ``index`` es el indice absoluto del ultimo punto que ya tiene el
        frontend (``since``). Devuelve tambien el nuevo indice absoluto
        para que el frontend lo use en la siguiente consulta.
        """
        with self._lock:
            start = max(0, index)
            pts = list(self._points)
            if start >= len(pts):
                return {"since": len(pts), "points": []}
            new_pts = pts[start:]
            return {
                "since": len(pts),
                "points": [p.to_dict() for p in new_pts],
            }

    def get_state(self) -> Dict:
        """Estado completo para el frontend (una sola llamada)."""
        with self._lock:
            pts = list(self._points)
            counts = dict(self._counts)
            last = self.last_point.to_dict() if self.last_point else None
            t0 = self._t0
            wrap = self._wrap_offset
            last_t = self._last_t
            summaries = list(self._summaries)
        return {
            "status": self.status,
            "port": self.port,
            "baud": self.baud,
            "error": self.error,
            "sim": self._sim_mode,
            "raw": self.last_raw[-160:],
            "last": last,
            "counts": counts,
            "total": len(pts),
            "since": len(pts),
            "points": [],  # los puntos van por get_points_since
            "t0": t0,
            "wrap": wrap,
            "last_t": last_t,
            "bytes": self._bytes_read,
            "uptime": round(time.time() - self._connected_at, 1) if self._connected_at else 0.0,
            "ports": list_serial_ports(),
            "summaries": summaries,
            "summaries_seen": self._summaries_seen,
            "setpoint": self._ev_state.get("spz", 0.0) or None,
            "replay": {
                "path": self._replay_path,
                "line": self._replay_pos,
                "total": self._replay_total,
            },
            "thresholds": {
                "leve": LEVE_THRESHOLD_G,
                "severo": SEVERO_THRESHOLD_G,
            },
        }

    def set_replay_speed(self, speed: float) -> float:
        """Fija el multiplicador de velocidad del replay (0.1 .. 50)."""
        self._replay_speed = max(0.1, min(50.0, float(speed)))
        return self._replay_speed

    # ------------------------------------------------------------------
    # Internos
    # ------------------------------------------------------------------
    def _start_thread(self, target) -> None:
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=target, name="serial-reader", daemon=True
        )
        self._thread.start()

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def _push(self, p: Point) -> None:
        with self._lock:
            self._points.append(p)
            self._counts[p.sym] = self._counts.get(p.sym, 0) + 1
            self.last_point = p
            self._trim_locked(p.t)

    def _trim_locked(self, newest_t: int) -> None:
        """Descarta puntos mas viejos que ``KEEP_SECONDS``.

        La retencion es **por tiempo**, no por numero de puntos: en v2 los
        eventos llegan a 100 Hz y en reposo no llega nada, asi que un buffer
        contado en muestras perderia eventos completos. El suelo
        ``MIN_KEEP_POINTS`` garantiza que nunca se queda con menos de unos
        20 eventos aunque el tiempo sea muy largo.
        """
        if len(self._points) <= MIN_KEEP_POINTS:
            return
        cutoff = newest_t - int(KEEP_SECONDS * 1000)
        n = 0
        for q in self._points:
            if q.t >= cutoff:
                break
            n += 1
        if n:
            dropped = min(n, len(self._points) - MIN_KEEP_POINTS)
            if dropped > 0:
                for _ in range(dropped):
                    self._points.popleft()

    def _handle_json(self, line: str) -> bool:
        """Interpreta una linea JSON del ensayo en seco. Devuelve True si la
        consumio (para no seguir buscando datos de muestra en ella).

        Distingue tres cosas:
        - ``accel/setpoint``      -> ``{"x":..,"y":..,"z":..}``  (3 escalares)
        - ``accel/event/summary`` -> lleva ``id``, ``sym``, ``spz``, ``peak``,
          ``dev``, ``n``, ``up``  -> se guarda como resumen de evento.
        - ``accel/event/chunk``   -> lleva ``id`` y vectores ``z``/``x``/``y``/
          ``dt`` -> no aporta nada nuevo al grafico (los datos ya llegan como
          lineas ``EV,``), se ignora.
        """
        if not line.startswith("[MQTT-OUT] {"):
            return False
        blob = line[len("[MQTT-OUT] "):]
        try:
            obj = json.loads(blob)
        except ValueError:
            return False
        if not isinstance(obj, dict):
            return True

        # --- resumen de evento ---
        if "sym" in obj and "id" in obj:
            self._summaries.append({
                "id": obj.get("id"),
                "sym": obj.get("sym"),
                "up": obj.get("up"),
                "spz": obj.get("spz"),
                "dev": obj.get("dev"),
                "peak": obj.get("peak"),
                "n": obj.get("n"),
                "ms": obj.get("ms"),
            })
            self._summaries_seen += 1
            if isinstance(obj.get("spz"), (int, float)):
                self._ev_state["spz"] = float(obj["spz"])
            return True

        # --- setpoint (escalares sueltos, sin vectores) ---
        if all(k in obj for k in ("x", "y", "z")) and not isinstance(obj["z"], list):
            try:
                self._ev_state["spz"] = float(obj["z"])
            except (TypeError, ValueError):
                pass
            return True

        # --- chunk u otro mensaje: nada que hacer ---
        return True

    def _handle_line(self, raw: str) -> None:
        self.last_raw = raw.strip()

        if self._handle_json(self.last_raw):
            return

        p = parse_line(self.last_raw, self._ev_state)
        if p is None:
            return  # banner / diagnostico / ruido: se ignora
        # Manejo del wrap de millis() (~49.7 dias)
        with self._lock:
            if self._t0 is None:
                self._t0 = p.t
            if self._last_t is not None and p.t < self._last_t - 1000000:
                self._wrap_offset += 1 << 32
            self._last_t = p.t
        # t corregido (absoluto en ms)
        p.t += self._wrap_offset
        self._push(p)

    def _serial_loop(self) -> None:
        """Bucle principal leyendo del puerto."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.2,
                write_timeout=0.2,
            )
            self._serial.reset_input_buffer()
        except Exception as exc:  # puerto no existe / ocupado / permisos
            self.status = "error"
            self.error = f"No se pudo abrir {self.port}: {exc}"
            return

        self._connected_at = time.time()
        self.status = "connected"
        self.error = None
        try:
            while not self._stop_event.is_set():
                try:
                    line = self._serial.readline()
                except Exception as exc:
                    self.status = "error"
                    self.error = f"Error de lectura: {exc}"
                    self._close_serial()
                    return
                if not line:
                    continue
                self._bytes_read += len(line)
                try:
                    text = line.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                if text:
                    self._handle_line(text)
        finally:
            self._close_serial()
            if not self._stop_event.is_set():
                self.status = "error"
                self.error = "Puerto cerrado inesperadamente"

    def _replay_loop(self) -> None:
        """Reproduce una captura de sesion guardada, linea a linea.

        No abre ningun puerto: lee el fichero y alimenta las mismas rutas de
        parseo que el puerto real. Se calibra el ritmo con las marcas de
        tiempo del propio log (el prefijo ``[T+123.456]`` que antepone
        CuTTeM) para que la animacion sea proportionally fiel; si el log no
        las lleva, se reproduce a velocidad maxima. ``self._replay_speed``
        multiplica ese ritmo.
        """
        path = self._replay_path
        if not path:
            self.status = "error"
            self.error = "REPLAY sin fichero"
            return
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                lines = fh.readlines()
        except OSError as exc:
            self.status = "error"
            self.error = f"No se pudo leer la captura: {exc}"
            return

        self._replay_total = len(lines)
        self._replay_pos = 0
        self._replay_rerun.clear()
        self._connected_at = time.time()
        self.status = "replay"
        self.error = None

        prev_t = None
        for line in lines:
            if self._stop_event.is_set():
                return
            if self._replay_rerun.is_set():   # clear() -> empezar de nuevo
                self._replay_rerun.clear()
                self._replay_pos = 0
                prev_t = None
                with self._lock:
                    self._points.clear()
                    self._counts = {s: 0 for s in VALID_SYMBOLS}
                    self._t0 = None
                    self._wrap_offset = 0
                    self._last_t = None
                    self._ev_state = new_event_state()
                    self.last_point = None
                    self._summaries.clear()
                    self._summaries_seen = 0

            text, stamp = _split_replay_stamp(line)
            if stamp is not None and prev_t is not None:
                delay = (stamp - prev_t) / 1000.0 / max(0.01, self._replay_speed)
                if delay > 0:
                    if self._stop_event.wait(min(delay, 1.0)):
                        return
            if stamp is not None:
                prev_t = stamp

            if text:
                self._bytes_read += len(text) + 1
                self._handle_line(text)
            self._replay_pos += 1

        self.status = "idle"

    def _simulator_loop(self) -> None:
        """Simulador que emite el **formato v2** (ventanas de evento).

        Reproduce la secuencia real del firmware en ensayo en seco:

        1. una linea JSON de setpoint al arrancar;
        2. cada pocos segundos, un resumen JSON de evento;
        3. las 200 lineas ``EV,`` de la ventana (100 antes + 100 despues).

        Entre eventos el ``t0`` avanza como si el dispositivo siguiera
        corriendo, de modo que el eje temporal muestra los huecos reales.
        """
        import random

        self._connected_at = time.time()
        setpoint = 0.92
        t_ms = 0
        event_id = 1
        # ritmo: ~8 s entre eventos (comprimido, para no esperar 11 h)
        self._handle_line(f'[MQTT-OUT] {{"x":0.12,"y":0.13,"z":{setpoint:.2f}}}')

        while not self._stop_event.is_set():
            gap_s = random.uniform(5.0, 12.0)
            if self._stop_event.wait(gap_s):
                return
            t_ms += int(gap_s * 1000)

            up_dir = random.random() < 0.5
            amp = random.uniform(0.22, 0.75)
            sym = ("*" if amp >= SEVERO_THRESHOLD_G else "+") if up_dir else \
                  ("!" if amp >= SEVERO_THRESHOLD_G else "-")
            t0 = t_ms
            raw_spz = int(setpoint * 256)

            samples = []
            for k in range(EVENT_POINTS):
                dev = random.uniform(-0.03, 0.03)
                if 95 <= k <= 130:   # el golpe, en el centro de la ventana
                    shape = math.sin(math.pi * (k - 95) / 35.0)
                    dev += amp * shape * (1 if up_dir else -1)
                dev += random.uniform(-0.015, 0.015)
                z = int((setpoint + dev) * 256)
                x = int(random.uniform(-0.05, 0.05) * 256)
                y = int(random.uniform(-0.05, 0.05) * 256)
                s = "." if abs(z - raw_spz) < LEVE_THRESHOLD_G * 256 else sym
                samples.append((x, y, z, s))

            dev_first = (samples[EVENT_POINTS // 2][2] - raw_spz) / 256.0
            peak = max(abs((z - raw_spz) / 256.0) for _, _, z, _ in samples)
            summary = {
                "id": event_id,
                "sym": sym,
                "up": t0,
                "spz": setpoint,
                "dev": round(dev_first, 3),
                "peak": round(peak, 3),
                "n": EVENT_POINTS,
                "ms": EVENT_POINTS * 10,
            }
            self._handle_line("[MQTT-OUT] " + json.dumps(summary, separators=(",", ":")))

            for x, y, z, s in samples:
                self._handle_line(f"EV,{t0},{x},{y},{z},10,{s}")

            t_ms += EVENT_POINTS * 10
            event_id += 1

        self.status = "idle"