"""The dashboard's web server: static files, a live stream and controls.

Standard library only.  Every URL is relative, so the dashboard also works
behind a path prefix such as Home Assistant's ingress.

* ``GET  /``                  the dashboard
* ``GET  api/state``          firmware list and, while a device runs, its snapshot
* ``GET  api/stream``         server-sent events: ``hello`` once, then ``tick``
                              (snapshot, new events, new history samples)
* ``POST api/control``        a command for the running device (JSON, see
                              :meth:`B2500Session._apply`)
* ``POST api/firmware?name=`` upload an image (the request body)
* ``POST api/start``          ``{"image": NAME, "pv": [W, W], "load": W, ...}``
* ``POST api/stop``
"""
from __future__ import annotations

import json
import logging
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

from .session import SESSION_TYPES, B2500Session

log = logging.getLogger("marsemu.ui")

STATIC = os.path.join(os.path.dirname(__file__), "static")
MAX_IMAGE = 4 * 1024 * 1024
_NAME_OK = re.compile(r"[^A-Za-z0-9._-]+")
ROUTES = ("/api/state", "/api/stream", "/api/control", "/api/firmware", "/api/start", "/api/stop",
          "/app.js", "/app.css")
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".json": "application/json"}


class UiApp:
    """Firmware images in ``firmware_dir`` and at most one running device.

    ``network``: called with no arguments for every new device, returns its
    :class:`~marsemu.fc41d.net.Network` (an MQTT broker, say).  ``defaults``:
    session options used when :meth:`start` does not give them."""

    def __init__(self, firmware_dir: str, *, state_dir: Optional[str] = None,
                 network: Optional[Callable[[], object]] = None, meter: str = "local",
                 defaults: Optional[dict] = None, allow_upload: bool = True):
        self.firmware_dir = firmware_dir
        self.state_dir = state_dir
        self.network = network
        self.meter = meter
        self.defaults = dict(defaults or {})
        self.allow_upload = allow_upload
        self.session: Optional[B2500Session] = None
        self.lock = threading.Lock()
        self.message: Optional[str] = None
        os.makedirs(firmware_dir, exist_ok=True)
        self._info: Dict[str, dict] = {}

    # -------------------------------------------------------------- firmware
    def images(self) -> List[dict]:
        from ..cli import image_info
        out = []
        for name in sorted(os.listdir(self.firmware_dir)):
            path = os.path.join(self.firmware_dir, name)
            if not os.path.isfile(path) or not name.lower().endswith(".bin"):
                continue
            st = os.stat(path)
            key = f"{name}:{st.st_size}:{st.st_mtime_ns}"
            if key not in self._info:
                try:
                    info = image_info(path)
                except Exception as exc:                         # noqa: BLE001 - shown to the user
                    info = {"profile": None, "error": str(exc)}
                self._info[key] = {"name": name, "size": st.st_size, "profile": info.get("profile"),
                                   "supported": info.get("profile") in SESSION_TYPES,
                                   "device_type": info.get("device_type_default"),
                                   "error": info.get("error")}
            out.append(self._info[key])
        return out

    def save_image(self, name: str, data: bytes) -> dict:
        if not self.allow_upload:
            raise PermissionError("uploads are switched off")
        name = _NAME_OK.sub("_", os.path.basename(name or "")).strip("._") or "firmware"
        if not name.lower().endswith(".bin"):
            name += ".bin"
        if not data or len(data) > MAX_IMAGE:
            raise ValueError(f"an image is between 1 byte and {MAX_IMAGE // 1024 // 1024} MB")
        path = os.path.join(self.firmware_dir, name)
        with open(path, "wb") as f:
            f.write(data)
        info = next((i for i in self.images() if i["name"] == name), None)
        return info or {"name": name}

    # ---------------------------------------------------------------- device
    def start(self, opts: dict) -> dict:
        name = os.path.basename(str(opts.get("image", "")))
        info = next((i for i in self.images() if i["name"] == name), None)
        if info is None:
            raise FileNotFoundError(f"no image {name!r} in the firmware folder")
        if not info["supported"]:
            raise ValueError(f"{name}: profile {info['profile']} is not in the dashboard yet (B2500 only for now)")
        o = {**self.defaults, **{k: v for k, v in opts.items() if k != "image"}}
        cls = SESSION_TYPES[info["profile"]]
        self.stop()
        state = os.path.join(self.state_dir, os.path.splitext(name)[0]) if self.state_dir else None
        sess = cls(os.path.join(self.firmware_dir, name),
                   pv=o.get("pv", (300.0, 300.0)), load=float(o.get("load", 400.0)), soc=float(o.get("soc", 50.0)),
                   ambient_c=float(o.get("ambient", 22.0)), extension_packs=int(o.get("extension_packs", 0)),
                   state_dir=state, network=self.network() if self.network else None, meter=self.meter,
                   speed=float(o.get("speed", 1.0)), device_type=o.get("device_type") or None,
                   setup=bool(o.get("setup", True)))
        with self.lock:
            self.session = sess
        sess.start()
        log.info("started %s (%s)", name, info["profile"])
        return {"started": name}

    def stop(self) -> None:
        with self.lock:
            sess, self.session = self.session, None
        if sess:
            sess.stop()

    def state(self) -> dict:
        s = self.session
        return {"images": self.images(), "running": s is not None, "upload": self.allow_upload,
                "image": os.path.basename(s.firmware) if s else None,
                "snapshot": s.read(10 ** 12, 10 ** 12)["snapshot"] if s else None}


def make_handler(app: UiApp):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "marsemu-ui"

        def log_message(self, fmt, *args):
            log.debug("%s " + fmt, self.address_string(), *args)

        # ------------------------------------------------------------ helpers
        def _send(self, code: int, body: bytes, ctype: str, extra: Optional[dict] = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode(), "application/json")

        def _error(self, code: int, msg: str) -> None:
            self._json({"error": msg}, code)

        def _route(self) -> str:
            # everything is relative to the page, so a path prefix (Home
            # Assistant's ingress) only changes what comes before these
            path = urlparse(self.path).path
            for known in ROUTES:
                if path.endswith(known):
                    return known
            return "/"

        def _body(self) -> bytes:
            n = int(self.headers.get("Content-Length") or 0)
            if n > MAX_IMAGE + 1024:
                raise ValueError("request too large")
            return self.rfile.read(n) if n else b""

        # ------------------------------------------------------------- routes
        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            route = self._route()
            if route == "/api/state":
                return self._json(app.state())
            if route == "/api/stream":
                return self._stream()
            if route.startswith("/api/"):
                return self._error(404, "not found")
            name = {"/": "index.html", "/app.js": "app.js", "/app.css": "app.css"}.get(route)
            if not name:
                return self._error(404, "not found")
            with open(os.path.join(STATIC, name), "rb") as f:
                body = f.read()
            self._send(200, body, TYPES[os.path.splitext(name)[1]])

        def do_POST(self):
            route = self._route()
            try:
                if route == "/api/firmware":
                    q = parse_qs(urlparse(self.path).query)
                    return self._json(app.save_image(q.get("name", ["firmware.bin"])[0], self._body()))
                body = json.loads(self._body() or b"{}")
                if route == "/api/control":
                    if not app.session:
                        return self._error(409, "no device is running")
                    app.session.command(body)
                    return self._json({"ok": True})
                if route == "/api/start":
                    return self._json(app.start(body))
                if route == "/api/stop":
                    app.stop()
                    return self._json({"stopped": True})
                return self._error(404, "not found")
            except PermissionError as exc:
                return self._error(403, str(exc))
            except FileNotFoundError as exc:
                return self._error(404, str(exc))
            except (ValueError, TypeError, KeyError) as exc:
                return self._error(400, str(exc))
            except Exception as exc:                                 # noqa: BLE001 - the device failed to build
                log.exception("request failed")
                return self._error(500, f"{type(exc).__name__}: {exc}")

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seq, t, sess, last_idle = 0, -1.0, None, 0.0

            def send(kind: str, obj) -> None:
                self.wfile.write(f"event: {kind}\ndata: {json.dumps(obj, separators=(',', ':'))}\n\n".encode())
                self.wfile.flush()
            try:
                send("hello", app.state())
                while True:
                    cur = app.session
                    if cur is not sess:
                        sess, seq, t = cur, 0, -1.0
                        send("hello", app.state())
                    if sess is not None:
                        r = sess.read(seq, t)
                        if r["events"]:
                            seq = r["events"][-1]["seq"]
                        if r["history"]:
                            t = r["history"][-1]["t"]
                        send("tick", r)
                        time.sleep(0.25)
                    else:
                        now = time.monotonic()
                        if now - last_idle > 15:
                            send("idle", {"images": app.images()})
                            last_idle = now
                        time.sleep(0.5)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

    return Handler


def serve(app: UiApp, host: str = "127.0.0.1", port: int = 8080) -> ThreadingHTTPServer:
    """Serve ``app`` on ``host:port`` in a background thread; returns the server."""
    srv = ThreadingHTTPServer((host, port), make_handler(app))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, name="marsemu-ui-http", daemon=True).start()
    return srv


__all__ = ["UiApp", "serve", "make_handler"]
