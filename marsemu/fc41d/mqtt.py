"""Minimal MQTT 3.1.1 client (no dependencies) used to bridge the emulated
FC41D to a real broker.  Supports CONNECT, SUBSCRIBE, PUBLISH (QoS 0/1),
PINGREQ and TLS (with optional client certificates)."""
from __future__ import annotations

import logging
import socket
import ssl
import struct
import threading
import time
from typing import Callable, Optional

log = logging.getLogger("fc41d.mqtt")


def _enc_str(s: bytes) -> bytes:
    return struct.pack(">H", len(s)) + s


def _enc_len(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        if n:
            b |= 0x80
        out.append(b)
        if not n:
            return bytes(out)


class MqttClient:
    def __init__(self, host: str, port: int, *, tls: Optional[ssl.SSLContext] = None,
                 on_message: Callable[[str, bytes], None], on_close: Callable[[str], None]):
        self.host, self.port, self.tls = host, port, tls
        self.on_message = on_message
        self.on_close = on_close
        self.sock: Optional[socket.socket] = None
        self.lock = threading.Lock()
        self.pid = 1
        self.alive = False
        self.keepalive = 60

    # -- connection -----------------------------------------------------
    def connect(self, client_id: str, user: Optional[str], password: Optional[str], keepalive: int = 60,
                timeout: float = 10.0) -> int:
        """Blocking connect; returns the CONNACK return code (or -1 on socket error)."""
        self.keepalive = keepalive
        try:
            s = socket.create_connection((self.host, self.port), timeout=timeout)
            if self.tls is not None:
                s = self.tls.wrap_socket(s, server_hostname=self.host)
        except OSError as e:
            log.warning("mqtt connect %s:%d failed: %s", self.host, self.port, e)
            return -1
        flags = 0x02
        payload = _enc_str(client_id.encode())
        if user:
            flags |= 0x80
        if password:
            flags |= 0x40
        if user:
            payload += _enc_str(user.encode())
        if password:
            payload += _enc_str(password.encode())
        vh = _enc_str(b"MQTT") + bytes([4, flags]) + struct.pack(">H", keepalive)
        pkt = bytes([0x10]) + _enc_len(len(vh) + len(payload)) + vh + payload
        try:
            s.sendall(pkt)
            hdr = self._recv_exact(s, 4)
        except OSError as e:
            log.warning("mqtt handshake failed: %s", e)
            return -1
        if hdr[0] != 0x20:
            return -1
        rc = hdr[3]
        if rc != 0:
            s.close()
            return rc
        s.settimeout(None)
        self.sock = s
        self.alive = True
        threading.Thread(target=self._reader, daemon=True, name="mqtt-rx").start()
        threading.Thread(target=self._pinger, daemon=True, name="mqtt-ping").start()
        return 0

    @staticmethod
    def _recv_exact(s, n):
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                raise OSError("connection closed")
            buf += chunk
        return buf

    def _send(self, pkt: bytes) -> bool:
        with self.lock:
            if not self.sock:
                return False
            try:
                self.sock.sendall(pkt)
                return True
            except OSError:
                return False

    def _next_pid(self) -> int:
        self.pid = self.pid % 65535 + 1
        return self.pid

    def subscribe(self, topic: str, qos: int = 0) -> bool:
        body = struct.pack(">H", self._next_pid()) + _enc_str(topic.encode()) + bytes([qos])
        return self._send(bytes([0x82]) + _enc_len(len(body)) + body)

    def unsubscribe(self, topic: str) -> bool:
        body = struct.pack(">H", self._next_pid()) + _enc_str(topic.encode())
        return self._send(bytes([0xA2]) + _enc_len(len(body)) + body)

    def publish(self, topic: str, payload: bytes, qos: int = 0, retain: bool = False) -> bool:
        qos = min(qos, 1)
        body = _enc_str(topic.encode())
        if qos:
            body += struct.pack(">H", self._next_pid())
        body += payload
        return self._send(bytes([0x30 | (qos << 1) | (1 if retain else 0)]) + _enc_len(len(body)) + body)

    def close(self) -> None:
        self.alive = False
        with self.lock:
            if self.sock:
                try:
                    self.sock.sendall(b"\xe0\x00")
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None

    # -- background --------------------------------------------------------
    def _pinger(self):
        while self.alive:
            time.sleep(max(5, self.keepalive * 0.6))
            if self.alive and not self._send(b"\xc0\x00"):
                break

    def _reader(self):
        s = self.sock
        reason = "closed"
        try:
            while self.alive:
                h = self._recv_exact(s, 1)[0]
                mult, length = 1, 0
                while True:
                    b = self._recv_exact(s, 1)[0]
                    length += (b & 0x7F) * mult
                    mult *= 128
                    if not b & 0x80:
                        break
                body = self._recv_exact(s, length) if length else b""
                kind = h >> 4
                if kind == 3:  # PUBLISH
                    tl = struct.unpack(">H", body[:2])[0]
                    topic = body[2:2 + tl].decode("utf-8", "replace")
                    pos = 2 + tl
                    qos = (h >> 1) & 3
                    if qos:
                        pid = body[pos:pos + 2]
                        pos += 2
                        self._send(bytes([0x40, 2]) + pid)
                    self.on_message(topic, body[pos:])
        except OSError as e:
            reason = str(e)
        if self.alive:
            self.alive = False
            self.on_close(reason)
