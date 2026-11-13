"""Lector del puerto serial para BusAccel-8266.

Se encarga de:
- Abrir/cerrar el puerto serial (pyserial) en un hilo en segundo plano.
- Leer lineas CSV ``t,X,Y,Z,setX,setY,setZ,sym`` a 20 Hz.
- IGNORAR cualquier linea que no sea CSV (banner/diagnostico del arranque).
- Mantener un buffer circular con los ultimos puntos y estadisticas de
  deteccion por simbolo.
- Proporcionar un modo SIMULADOR (datos sinteticos a 20 Hz) para poder
  probar la interfaz sin placa fisica.
"""

from __future__ import annotations

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

# Maximo de puntos retenidos en memoria (~300 s a 20 Hz)
MAX_POINTS = 6000


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


def parse_line(line: str) -> Optional[Point]:
    """Parsea una linea CSV del firmware.

    Formato esperado (8 campos, 1ro entero)::

        t,X,Y,Z,setX,setY,setZ,sym
        123456,0.14,0.28,0.88,0.14,0.28,0.88,.

    Cualquier linea que no cumpla el formato (banner, diagnostico, puntos
    de adquisicion, etc.) devuelve ``None`` y se descarta.
    """
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


def list_serial_ports() -> List[str]:
    """Devuelve la lista de puertos seriales disponibles."""
    if not _SERIAL_AVAILABLE:
        return []
    try:
        return sorted({p.device for p in list_ports.comports()})
    except Exception:
        return []


class SerialReader:
    """Hilo que lee el puerto serial y alimenta un buffer circular."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self._points: Deque[Point] = deque(maxlen=MAX_POINTS)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._serial = None  # type: ignore[var-annotated]
        self.status: str = "idle"          # idle|connecting|connected|error|simulator
        self.error: Optional[str] = None
        self.last_raw: str = ""
        self.last_point: Optional[Point] = None
        self._t0: Optional[int] = None
        self._wrap_offset: int = 0
        self._last_t: Optional[int] = None
        self._counts: Dict[str, int] = {s: 0 for s in VALID_SYMBOLS}
        self._connected_at: float = 0.0
        self._bytes_read: int = 0
        self._sim_mode: bool = False

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------
    def connect(self, port: Optional[str] = None, baud: Optional[int] = None) -> None:
        """Conecta al puerto (o entra en modo simulador si port == 'SIM').

        Si ya hay un hilo activo, primero lo detiene.
        """
        self.disconnect()
        if port:
            self.port = port
        if baud:
            self.baud = baud

        if self.port == "SIM":
            self._sim_mode = True
            self.status = "simulator"
            self.error = None
            self._start_thread(self._simulator_loop)
            return

        self._sim_mode = False
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
        """Limpia el buffer de puntos y las estadisticas."""
        with self._lock:
            self._points.clear()
            self._counts = {s: 0 for s in VALID_SYMBOLS}
            self._t0 = None
            self._wrap_offset = 0
            self._last_t = None
            self.last_point = None

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
            "thresholds": {
                "leve": LEVE_THRESHOLD_G,
                "severo": SEVERO_THRESHOLD_G,
            },
        }

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

    def _handle_line(self, raw: str) -> None:
        self.last_raw = raw.strip()
        p = parse_line(self.last_raw)
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

    def _simulator_loop(self) -> None:
        """Genera datos sinteticos a 20 Hz con eventos ocasionales."""
        import math
        import random

        self._connected_at = time.time()
        t_ms = 0
        event_timer = 0.0
        event_kind = None  # ('+',0.35) -> duracion y amplitud
        event_remaining = 0.0
        while not self._stop_event.is_set():
            time.sleep(0.05)  # 20 Hz
            t_ms += 50
            # ruido base
            z = 1.00 + random.uniform(-0.02, 0.02)
            x = random.uniform(-0.03, 0.03)
            y = random.uniform(-0.03, 0.03)
            sym = "."

            event_timer -= 0.05
            if event_timer <= 0 and event_remaining <= 0:
                # programar un evento aleatorio
                event_timer = random.uniform(3.0, 9.0)
                r = random.random()
                if r < 0.4:
                    event_kind = "+" if random.random() < 0.5 else "-"
                    event_remaining = random.uniform(0.15, 0.45)
                    event_amp = random.uniform(0.25, 0.6)
                else:
                    event_kind = None

            if event_remaining > 0:
                event_remaining -= 0.05
                amp = event_amp * (event_remaining / 0.3)
                if event_kind == "+":
                    z += max(0.0, amp)
                    sym = "*" if amp >= SEVERO_THRESHOLD_G else "+"
                else:
                    z -= max(0.0, amp)
                    sym = "!" if amp >= SEVERO_THRESHOLD_G else "-"
                if event_remaining <= 0:
                    event_remaining = 0.0

            line = (
                f"{t_ms},{x:.2f},{y:.2f},{z:.2f},"
                f"{1.00:.2f},{1.00:.2f},{1.00:.2f},{sym}"
            )
            self._handle_line(line)
        self.status = "idle"