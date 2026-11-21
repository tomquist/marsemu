"""Bluetooth over WebSocket: a browser as the BLE central.

A WebSocket client stands in for the phone. Each binary message it sends is
one write to the device's command characteristic (FF01); each notification
the firmware sends on FF02 comes back as one binary message. Message framing
keeps notifications apart, which a TCP stream cannot do for the B2500 replies
whose length byte is wrong.

Text messages carry JSON about the device, sent when the client connects
and whenever it changes::

    {"type": "device", "name": "MST_VNSE3_0002", "advertising": true,
     "mac": "c8:47:8c:00:00:02", "services": ["ff00"]}

This is what hmjs' emulator transport (``createEmulatorBluetooth``) speaks.
The server is plain ``http.server`` with an RFC 6455 handshake, so it adds no
dependency; :func:`serve_ble_ws` runs it on its own port and the dashboard
mounts the same link at ``api/ble``.
"""
from __future__ import annotations

import base64
import hashlib
import json
import logging
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

log = logging.getLogger("emu.blews")

_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_TEXT, _BINARY, _CLOSE, _PING, _PONG, _CONT = 0x1, 0x2, 0x8, 0x9, 0xA, 0x0
_MAX_MESSAGE = 1 << 16


class WebSocketClosed(Exception):
    pass


class WebSocket:
    """The server side of one WebSocket connection (RFC 6455), on the
    socket files an ``http.server`` handler already holds."""

    def __init__(self, rfile, wfile):
        self.rfile, self.wfile = rfile, wfile
        self._send_lock = threading.Lock()
        self.closed = False

    def _read(self, n: int) -> bytes:
        data = self.rfile.read(n)
        if len(data) < n:
            raise WebSocketClosed()
        return data

    def _frame(self, opcode: int, payload: bytes) -> None:
        n = len(payload)
        head = bytes([0x80 | opcode])
        if n < 126:
            head += bytes([n])
        elif n < 1 << 16:
            head += bytes([126]) + struct.pack(">H", n)
        else:
            head += bytes([127]) + struct.pack(">Q", n)
        with self._send_lock:
            if self.closed:
                raise WebSocketClosed()
            try:
                self.wfile.write(head + payload)
                self.wfile.flush()
            except OSError as exc:
                self.closed = True
                raise WebSocketClosed() from exc

    def send(self, message) -> None:
        """Send ``bytes`` as a binary message, ``str`` as a text message."""
        if isinstance(message, str):
            self._frame(_TEXT, message.encode())
        else:
            self._frame(_BINARY, bytes(message))

    def recv(self):
        """The next data message: ``bytes`` (binary) or ``str`` (text).
        Answers pings; raises :class:`WebSocketClosed` at the end."""
        parts, kind = [], None
        while True:
            b0, b1 = self._read(2)
            fin, opcode = b0 & 0x80, b0 & 0x0F
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._read(8))[0]
            if n > _MAX_MESSAGE:
                raise WebSocketClosed()
            mask = self._read(4) if b1 & 0x80 else b"\0\0\0\0"
            payload = bytes(c ^ mask[i & 3] for i, c in enumerate(self._read(n)))
            if opcode == _CLOSE:
                try:
                    self._frame(_CLOSE, payload[:2])
                except WebSocketClosed:
                    pass
                self.closed = True
                raise WebSocketClosed()
            if opcode == _PING:
                self._frame(_PONG, payload)
                continue
            if opcode == _PONG:
                continue
            if opcode in (_TEXT, _BINARY):
                kind, parts = opcode, [payload]
            elif opcode == _CONT and kind is not None:
                parts.append(payload)
            if fin and kind is not None:
                data = b"".join(parts)
                return data.decode() if kind == _TEXT else data

    def close(self) -> None:
        try:
            self._frame(_CLOSE, struct.pack(">H", 1000))
        except WebSocketClosed:
            pass
        self.closed = True


def accept(handler: BaseHTTPRequestHandler) -> Optional[WebSocket]:
    """Answer a WebSocket upgrade request on ``handler``; ``None`` (and a 400
    reply) if the request is not one."""
    key = handler.headers.get("Sec-WebSocket-Key")
    if "websocket" not in (handler.headers.get("Upgrade") or "").lower() or not key:
        body = b"expected a WebSocket upgrade\n"
        handler.send_response(400)
        handler.send_header("Content-Type", "text/plain")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
        return None
    accept_key = base64.b64encode(hashlib.sha1((key + _GUID).encode()).digest()).decode()
    handler.send_response(101, "Switching Protocols")
    handler.send_header("Upgrade", "websocket")
    handler.send_header("Connection", "Upgrade")
    handler.send_header("Sec-WebSocket-Accept", accept_key)
    handler.end_headers()
    handler.wfile.flush()
    handler.close_connection = True
    return WebSocket(handler.rfile, handler.wfile)


class BleLink:
    """Connects WebSocket centrals to an emulator's Wi‑Fi/BLE module.

    ``emulator`` is the :class:`~marsemu.emulator.Emulator` (or a function
    returning the current one, for the dashboard, whose device changes). One
    central at a time, as on the device: a new connection replaces the
    previous one."""

    def __init__(self, emulator):
        self._get = emulator if callable(emulator) else (lambda: emulator)
        self._lock = threading.Lock()
        self._current: Optional[WebSocket] = None

    @staticmethod
    def device_info(module) -> dict:
        return {"type": "device", "name": module.ble_name, "advertising": bool(module.ble_adv),
                "mac": module.cfg.ble_mac, "services": [s.lower() for s in module.ble_services]}

    def serve(self, ws: WebSocket) -> None:
        """Run one central until it disconnects (blocks the calling thread)."""
        emu = self._get()
        if emu is None:
            ws.send(json.dumps({"type": "error", "message": "no device is running"}))
            ws.close()
            return
        module = emu.fc41d
        with self._lock:
            old, self._current = self._current, ws
        if old is not None:
            old.close()

        def info(*_):
            try:
                ws.send(json.dumps(self.device_info(module)))
            except WebSocketClosed:
                pass

        def notify(uuid, data):
            try:
                ws.send(bytes(data))
            except WebSocketClosed:
                pass

        handlers = (("ble_notify", notify), ("ble_name", info), ("ble_adv", info))
        for event, fn in handlers:
            module.on(event, fn)
        info()
        module.post(module.ble_connect)
        log.info("BLE central connected over WebSocket")
        try:
            while True:
                msg = ws.recv()
                if isinstance(msg, bytes) and msg:
                    module.post(lambda d=msg: module.ble_write(d))
                elif isinstance(msg, str) and msg.strip() == '{"type":"device"}':
                    info()
        except WebSocketClosed:
            pass
        finally:
            for event, fn in handlers:
                try:
                    module.listeners.get(event, []).remove(fn)
                except ValueError:
                    pass
            with self._lock:
                if self._current is ws:
                    self._current = None
                    module.post(module.ble_disconnect)
            log.info("BLE central disconnected")


def serve_ble_ws(emulator, port: int, host: str = "127.0.0.1") -> ThreadingHTTPServer:
    """Serve ``emulator``'s Bluetooth on ``ws://host:port/`` (any path) in a
    background thread; returns the server."""
    link = BleLink(emulator)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            log.debug("%s " + fmt, self.address_string(), *args)

        def do_GET(self):
            ws = accept(self)
            if ws is not None:
                link.serve(ws)

    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, name="marsemu-ble-ws", daemon=True).start()
    log.warning("BLE WebSocket on ws://%s:%d/", host, srv.server_address[1])
    return srv


__all__ = ["BleLink", "WebSocket", "WebSocketClosed", "accept", "serve_ble_ws"]
