"""Bluetooth over WebSocket (marsemu.blews) against a stand-in module."""
import base64
import json
import os
import socket
import struct
import threading
import time

from marsemu.blews import serve_ble_ws


class FakeModule:
    """What BleLink uses of the FC41D model; answers each write with a
    notification carrying the same bytes reversed."""

    def __init__(self):
        self.listeners = {}
        self.ble_name = "MST_VNSE3_0002"
        self.ble_adv = True
        self.ble_services = ["FF00"]
        self.cfg = type("Cfg", (), {"ble_mac": "c8:47:8c:00:00:02"})()
        self.connected = False
        self.writes = []

    def on(self, event, fn):
        self.listeners.setdefault(event, []).append(fn)

    def emit(self, event, *args):
        for fn in list(self.listeners.get(event, [])):
            fn(*args)

    def post(self, fn):                     # the emulator thread, here inline
        fn()

    def ble_connect(self):
        self.connected = True

    def ble_disconnect(self):
        self.connected = False

    def ble_write(self, data):
        self.writes.append(bytes(data))
        self.emit("ble_notify", "ff02", bytes(reversed(data)))


class Client:
    """A minimal WebSocket client (masked frames, as browsers send)."""

    def __init__(self, port, path="/ble"):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall((f"GET {path} HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            head += self.s.recv(1)
        assert b" 101 " in head.split(b"\r\n")[0]
        # the accept key as RFC 6455 computes it (browsers reject anything else)
        import hashlib
        want = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
        assert b"Sec-WebSocket-Accept: " + want in head

    def send(self, data, opcode=2):
        mask = os.urandom(4)
        n = len(data)
        head = bytes([0x80 | opcode]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n))
        self.s.sendall(head + mask + bytes(c ^ mask[i & 3] for i, c in enumerate(data)))

    def _read(self, n):
        out = b""
        while len(out) < n:
            chunk = self.s.recv(n - len(out))
            if not chunk:
                raise EOFError
            out += chunk
        return out

    def recv(self):
        b0, b1 = self._read(2)
        n = b1 & 0x7F
        if n == 126:
            n = struct.unpack(">H", self._read(2))[0]
        payload = self._read(n)
        return (payload.decode() if b0 & 0x0F == 1 else payload), b0 & 0x0F

    def close(self):
        self.send(struct.pack(">H", 1000), opcode=8)
        self.s.close()


def test_writes_and_notifications_travel_as_messages():
    module = FakeModule()
    emu = type("Emu", (), {"fc41d": module})()
    srv = serve_ble_ws(emu, 0)
    try:
        c = Client(srv.server_address[1])
        info, kind = c.recv()
        assert kind == 1
        assert json.loads(info) == {"type": "device", "name": "MST_VNSE3_0002", "advertising": True,
                                    "mac": "c8:47:8c:00:00:02", "services": ["ff00"]}
        assert module.connected
        frame = bytes.fromhex("7305230356")
        c.send(frame)
        reply, kind = c.recv()
        assert kind == 2 and reply == bytes(reversed(frame))
        assert module.writes == [frame]
        # a long write arrives whole (16-bit length)
        big = bytes(range(200))
        c.send(big)
        assert c.recv()[0] == bytes(reversed(big))
        # name changes are announced
        module.ble_name = "MST_VNSE3_1234"
        module.emit("ble_name", module.ble_name)
        assert json.loads(c.recv()[0])["name"] == "MST_VNSE3_1234"
        c.close()
        for _ in range(50):
            if not module.connected:
                break
            time.sleep(0.02)
        assert not module.connected
        assert not module.listeners.get("ble_notify")
    finally:
        srv.shutdown()


def test_a_new_central_replaces_the_previous_one():
    module = FakeModule()
    emu = type("Emu", (), {"fc41d": module})()
    srv = serve_ble_ws(emu, 0)
    try:
        a = Client(srv.server_address[1])
        a.recv()
        b = Client(srv.server_address[1])
        b.recv()
        # the first connection is closed by the server
        _, kind = a.recv()
        assert kind == 8
        b.send(b"\x01\x02")
        assert b.recv()[0] == b"\x02\x01"
        b.close()
    finally:
        srv.shutdown()


def test_plain_http_is_refused():
    module = FakeModule()
    srv = serve_ble_ws(type("Emu", (), {"fc41d": module})(), 0)
    try:
        s = socket.create_connection(("127.0.0.1", srv.server_address[1]), timeout=5)
        s.sendall(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n")
        assert b" 400 " in s.recv(200)
        s.close()
        assert not module.connected
    finally:
        srv.shutdown()
