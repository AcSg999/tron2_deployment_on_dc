"""Loopback browser view for an explicitly started calibration CLI session."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.parse import urlsplit

WEB_ROOT = Path(__file__).with_name("web")
ASSETS = {"/": ("calibration.html", "text/html; charset=utf-8"),
          "/calibration.js": ("calibration.js", "text/javascript; charset=utf-8"),
          "/calibration.css": ("calibration.css", "text/css; charset=utf-8")}


def make_server(guide, port=8790):
    """Opening the page only reads session state; hardware reads require POST."""
    if not isinstance(port, int) or not 0 <= port <= 65535:
        raise ValueError("port must be between 0 and 65535")
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, value, content_type="application/json; charset=utf-8"):
            body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def _local_request(self):
            actual_port = self.server.server_address[1]
            hosts = {f"127.0.0.1:{actual_port}", f"localhost:{actual_port}"}
            host = self.headers.get("Host")
            if host not in hosts:
                self._send(403, {"error": "Open the localhost URL printed by the CLI."})
                return False
            origin = self.headers.get("Origin")
            if origin is not None and origin != f"http://{host}":
                self._send(403, {"error": "Calibration actions require the same local page."})
                return False
            return True

        def do_GET(self):
            if not self._local_request():
                return
            path = urlsplit(self.path).path
            if path in ASSETS:
                name, kind = ASSETS[path]
                self._send(200, (WEB_ROOT / name).read_bytes(), kind)
            elif path == "/api/status":
                with lock:
                    self._send(200, guide.status())
            else:
                self._send(404, {"error": "Not found"})

        def do_POST(self):
            if not self._local_request():
                return
            action = {"/api/preview": guide.preview, "/api/save": guide.save,
                      "/api/solve": guide.solve}.get(self.path)
            if action is None:
                self._send(404, {"error": "Not found"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Send a small JSON object from the calibration page.")
                if json.loads(self.rfile.read(size)) != {}:
                    raise ValueError("Calibration actions use the fixed CLI session settings.")
                with lock:
                    state = action()
                self._send(200, state)
            except (ValueError, OSError, RuntimeError) as error:
                self._send(400, {"error": str(error)})

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def serve(profile_path, *, stage, side=None, pattern=(9, 6), square_m=.025, target=None,
          output, mock=False, port=8790):
    from .calibration_guide import CalibrationGuide
    from .config import load_profile

    guide = CalibrationGuide(load_profile(profile_path), output, stage=stage, side=side,
                             pattern=pattern, square_m=square_m, target=target,
                             mock=mock, profile_path=profile_path)
    server = make_server(guide, port)
    print(f"Calibration helper / 标定辅助: http://127.0.0.1:{server.server_address[1]}", flush=True)
    print("Open this URL. Preview reads the camera; Save captures a fresh sample. Ctrl+C stops the helper.\n"
          "打开此地址。预览读取相机，保存会重新采集样本。按 Ctrl+C 结束。", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
