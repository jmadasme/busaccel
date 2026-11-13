"""BusAccel-8266 — Monitor de escritorio para el acelerometro GY-291 (ADXL345).

Backend:
  - Abre el puerto serial (pyserial) y parsea las lineas CSV
    ``t,X,Y,Z,setX,setY,setZ,sym`` emitidas por el firmware (ciclo 16).
  - Sirve el frontend (index.html + style.css + app.js) y una API JSON
    local sobre un ThreadingHTTPServer en 127.0.0.1.
  - Muestra la interfaz en una ventana nativa con pywebview (backend Qt),
    y si pywebview falla en runtime, abre el navegador por defecto.

Uso:
    python3 app.py [--port /dev/ttyUSB0] [--baud 115200] [--sim] [--browser]
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import socket
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional
from urllib.parse import parse_qs, urlparse

from serial_reader import SerialReader, list_serial_ports

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
            self.reader.connect(port, baud)
            self._send_json({"ok": True, "status": self.reader.status})
        elif path == "/api/disconnect":
            self.reader.disconnect()
            self._send_json({"ok": True, "status": self.reader.status})
        elif path == "/api/clear":
            self.reader.clear()
            self._send_json({"ok": True})
        else:
            self._send_json({"error": f"no route: {path}"}, 404)

    def do_POST(self) -> None:  # noqa: N802
        self.do_GET()

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


def start_server(reader: SerialReader, preferred_port: int) -> ThreadingHTTPServer:
    """Arranca el servidor HTTP local en un hilo."""
    port = _free_port(preferred_port)
    server = ThreadingHTTPServer(("127.0.0.1", port), ApiHandler)
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
            "BusAccel-8266 — Monitor de aceleración (GY-291 / ADXL345)",
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
        description="BusAccel-8266 — Monitor de escritorio del acelerometro GY-291"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Puerto serial (default: /dev/ttyUSB0). Use 'SIM' para simulador.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--sim", action="store_true",
                        help="Arrancar en modo simulador (datos sinteticos a 20 Hz)")
    parser.add_argument("--browser", action="store_true",
                        help="Forzar modo navegador (no usar pywebview)")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT,
                        help=f"Puerto del servidor local (default: {DEFAULT_HTTP_PORT})")
    parser.add_argument("--debug", action="store_true", help="Habilitar devtools de pywebview")
    args = parser.parse_args()

    reader = SerialReader(port=args.port, baud=args.baud)
    server = start_server(reader, args.http_port)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    url = f"{base}/?port={args.port}&baud={args.baud}"
    print(f"[BusAccel] Frontend servido en {url}")

    if args.sim or args.port == "SIM":
        reader.connect("SIM")
    else:
        reader.connect(args.port, args.baud)

    if not args.browser and launch_gui(url, debug=args.debug):
        server.shutdown()
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


if __name__ == "__main__":
    main()