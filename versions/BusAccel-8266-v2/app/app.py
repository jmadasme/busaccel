"""BusAccel-8266-v2 — Monitor de escritorio para el acelerometro GY-291 (ADXL345).

Backend:
  - Abre la fuente de datos: puerto real (pyserial), ``SIM`` (simulador en
    formato v2) o ``REPLAY:<fichero>`` (captura de sesion reproducida).
  - Parsea las lineas CSV del firmware, tanto el formato continuo de v1
    (``t,X,Y,Z,setX,setY,setZ,sym``) como las ventanas de evento de v2
    (``EV,t0,Xraw,Yraw,Zraw,dt,sym``), y las lineas JSON ``[MQTT-OUT]`` del
    ensayo en seco (setpoint y resumen de cada evento).
  - Sirve el frontend (index.html + style.css + app.js) y una API JSON
    local sobre un ThreadingHTTPServer en 127.0.0.1.
  - Muestra la interfaz en una ventana nativa con pywebview (backend Qt),
    y si pywebview falla en runtime, abre el navegador por defecto.

Uso:
    python3 app.py [--port /dev/ttyUSB0] [--baud 115200] [--sim]
                   [--replay captura.log] [--speed 4.0] [--browser]
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import socket
import tempfile
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional
from urllib.parse import parse_qs, urlparse

from serial_reader import REPLAY_PREFIX, SerialReader, list_serial_ports

APP_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HTTP_PORT = 8765


class ApiHandler(BaseHTTPRequestHandler):
    """Sirve el frontend estatico y la API JSON local."""

    reader: SerialReader  # inyectado por la clase Server
    server_version = "BusAccel/1.0"

    # ------------------------------------------------------------------
    # Helpers HTTP
    # ------------------------------------------------------------------
    def _send_json(self, payload: Dict, code: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, rel_path: str) -> None:
        path = os.path.join(APP_DIR, rel_path)
        if not os.path.isfile(path):
            self._send_json({"error": "not found"}, 404)
            return
        ctype, _ = mimetypes.guess_type(path)
        with open(path, "rb") as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    # ------------------------------------------------------------------
    # Rutas
    # ------------------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802 (API de BaseHTTPRequestHandler)
        self._route("")

    def do_POST(self) -> None:  # noqa: N802
        # El cuerpo se lee aqui y se pasa a _route: las capturas de una
        # jornada ocupan varios MB y no caben en una query string.
        length = 0
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        body = ""
        if length > 0:
            raw = self.rfile.read(length)
            try:
                body = raw.decode("utf-8", errors="replace")
            except Exception:
                body = ""
        self._route(body)

    def _route(self, body: str) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path in ("/", "/index.html"):
            self._send_file("index.html")
        elif path == "/style.css":
            self._send_file("style.css")
        elif path == "/app.js":
            self._send_file("app.js")
        elif path == "/api/state":
            since = 0
            try:
                since = int(query.get("since", ["0"])[0])
            except ValueError:
                since = 0
            state = self.reader.get_state()
            inc = self.reader.get_points_since(since)
            state["points"] = inc["points"]
            state["since"] = inc["since"]
            self._send_json(state)
        elif path == "/api/ports":
            self._send_json({"ports": list_serial_ports()})
        elif path == "/api/connect":
            port = query.get("port", [self.reader.port])[0]
            baud = int(query.get("baud", ["115200"])[0])
            speed = query.get("speed", [None])[0]
            if speed is not None:
                self.reader.set_replay_speed(float(speed))
            self.reader.connect(port, baud)
            self._send_json({"ok": True, "status": self.reader.status})
        elif path == "/api/disconnect":
            self.reader.disconnect()
            self._send_json({"ok": True, "status": self.reader.status})
        elif path == "/api/clear":
            self.reader.clear()
            self._send_json({"ok": True})
        elif path == "/api/replay":
            # Atajo del frontend: reproducir un fichero del disco.
            src = query.get("src", [""])[0]
            if not src:
                self._send_json({"error": "falta src"}, 400)
                return
            if not src.startswith(REPLAY_PREFIX):
                src = REPLAY_PREFIX + src
            self.reader.connect(src)
            self._send_json({"ok": True, "status": self.reader.status,
                             "error": self.reader.error})
        elif path == "/api/replay-text":
            # El navegador no puede dar la ruta local: se manda el texto de la
            # captura en el cuerpo del POST y el backend lo guarda en un
            # temporal para reproducirlo linea a linea.
            text = body or query.get("lines", [""])[0]
            if not text.strip():
                self._send_json({"error": "captura vacia"}, 400)
                return
            fd, tmp = tempfile.mkstemp(prefix="busaccel-captura-", suffix=".log")
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                fh.write(text if text.endswith("\n") else text + "\n")
            self.server.cleanup_capture()  # type: ignore[attr-defined]
            self.server.replay_tmp = tmp  # type: ignore[attr-defined]
            self.reader.connect(REPLAY_PREFIX + tmp)
            self._send_json({"ok": True, "status": self.reader.status,
                             "error": self.reader.error, "lines": text.count("\n") + 1})
        elif path == "/api/speed":
            try:
                speed = self.reader.set_replay_speed(float(query.get("speed", ["1"])[0]))
            except ValueError:
                self._send_json({"error": "speed no numerico"}, 400)
                return
            self._send_json({"ok": True, "speed": speed})
        elif path == "/api/open":
            # Selector de fichero nativo de pywebview (opcional).
            path_sel = self.server.select_file_dialog()  # type: ignore[attr-defined]
            self._send_json({"path": path_sel})
        else:
            self._send_json({"error": f"no route: {path}"}, 404)

    def log_message(self, fmt: str, *args) -> None:  # silenciar logs
        pass


def _free_port(preferred: int) -> int:
    """Devuelve ``preferred`` si esta libre, si no un puerto aleatorio libre."""
    for candidate in (preferred, 0):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("127.0.0.1", candidate))
                return s.getsockname()[1]
        except OSError:
            continue
    return 0


class LocalServer(ThreadingHTTPServer):
    """Servidor HTTP local con el selector de fichero y el temporal de captura."""

    daemon_threads = True

    def __init__(self, addr, handler, reader: SerialReader) -> None:
        super().__init__(addr, handler)
        self.reader = reader
        self.replay_tmp: Optional[str] = None

    def select_file_dialog(self) -> Optional[str]:
        """Abre un selector nativo de fichero y devuelve la ruta (o None)."""
        try:
            import tkinter as tk
            from tkinter import filedialog
        except Exception:
            return None
        try:
            root = tk.Tk()
            root.withdraw()
            root.update()
            path = filedialog.askopenfilename(
                title="Abrir captura de sesión",
                filetypes=[("Logs de sesión", "*.log *.txt"), ("Todos", "*.*")],
            )
            root.destroy()
            return path or None
        except Exception:
            return None

    def cleanup_capture(self) -> None:
        """Borra el temporal de la captura anterior subida por el navegador."""
        if self.replay_tmp:
            try:
                os.unlink(self.replay_tmp)
            except OSError:
                pass
            self.replay_tmp = None

    def cleanup(self) -> None:
        self.cleanup_capture()


def start_server(reader: SerialReader, preferred_port: int) -> LocalServer:
    """Arranca el servidor HTTP local en un hilo."""
    port = _free_port(preferred_port)
    server = LocalServer(("127.0.0.1", port), ApiHandler, reader)
    ApiHandler.reader = reader
    thread = threading.Thread(target=server.serve_forever, name="http-server", daemon=True)
    thread.start()
    return server


def launch_gui(url: str, debug: bool = False) -> bool:
    """Lanza pywebview; devuelve False si no esta disponible/falla."""
    try:
        import webview  # type: ignore[import-untyped]
    except Exception:
        return False
    try:
        window = webview.create_window(
            "BusAccel-8266-v2 — Monitor de aceleración (GY-291 / ADXL345)",
            url=url,
            width=1320,
            height=860,
            min_size=(1024, 660),
            background_color="#0b0e14",
        )
        webview.start(debug=debug, private_mode=False)
        return True
    except Exception:
        return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="BusAccel-8266-v2 — Monitor de escritorio del acelerometro GY-291"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Puerto serial (default: /dev/ttyUSB0). Use 'SIM' para simulador.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--sim", action="store_true",
                        help="Arrancar en modo simulador (ventanas de evento sinteticas)")
    parser.add_argument("--replay", metavar="FICHERO", default=None,
                        help="Reproducir una captura de sesion guardada (.log de CuTTeM)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Multiplicador de velocidad del replay (default: 1.0)")
    parser.add_argument("--browser", action="store_true",
                        help="Forzar modo navegador (no usar pywebview)")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT,
                        help=f"Puerto del servidor local (default: {DEFAULT_HTTP_PORT})")
    parser.add_argument("--debug", action="store_true", help="Habilitar devtools de pywebview")
    args = parser.parse_args()

    source = args.port
    if args.replay:
        source = REPLAY_PREFIX + os.path.abspath(args.replay)
        if not os.path.isfile(source[len(REPLAY_PREFIX):]):
            parser.error(f"no existe la captura: {args.replay}")

    reader = SerialReader(port=source, baud=args.baud)
    reader.set_replay_speed(args.speed)
    server = start_server(reader, args.http_port)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    url = f"{base}/?port={source}&baud={args.baud}"
    print(f"[BusAccel] Frontend servido en {url}")

    if args.sim:
        reader.connect("SIM")
    elif args.replay:
        reader.connect(source)
    else:
        reader.connect(source, args.baud)
    print(f"[BusAccel] Fuente: {source} · estado: {reader.status}"
          + (f" · {reader.error}" if reader.error else ""))

    if not args.browser and launch_gui(url, debug=args.debug):
        server.shutdown()
        server.cleanup()
        return

    # Fallback: navegador
    print("[BusAccel] pywebview no disponible; abriendo navegador por defecto.")
    webbrowser.open(url)
    try:
        while True:
            threading.Event().wait(3600)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.cleanup()


if __name__ == "__main__":
    main()