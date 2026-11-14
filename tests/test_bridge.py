"""MQTT broker bridge (``--mqtt broker``) against a tiny in-process broker."""
import socket
import struct
import threading
import time

from marsemu.core.machine import Machine
from marsemu.fc41d.module import Fc41d, Fc41dConfig
from marsemu.fc41d.net import Network
from marsemu.periph import n32g45x


class TinyBroker:
    """Accepts one client: CONNACK, SUBACK, records PUBLISH, can push PUBLISH."""

    def __init__(self):
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.client_id = None
        self.subs = []
        self.pubs = []
        self.conn = None
        threading.Thread(target=self._run, daemon=True).start()

    def _recv(self, n):
        b = b""
        while len(b) < n:
            c = self.conn.recv(n - len(b))
            if not c:
                raise OSError
            b += c
        return b

    def _run(self):
        self.conn, _ = self.srv.accept()
        try:
            while True:
                h = self._recv(1)[0]
                ln, mult = 0, 1
                while True:
                    b = self._recv(1)[0]
                    ln += (b & 0x7F) * mult
                    mult *= 128
                    if not b & 0x80:
                        break
                body = self._recv(ln) if ln else b""
                kind = h >> 4
                if kind == 1:
                    n = struct.unpack(">H", body[10:12])[0]
                    self.client_id = body[12:12 + n].decode()
                    self.conn.sendall(b"\x20\x02\x00\x00")
                elif kind == 8:
                    pid = body[:2]
                    n = struct.unpack(">H", body[2:4])[0]
                    self.subs.append(body[4:4 + n].decode())
                    self.conn.sendall(b"\x90\x03" + pid + b"\x00")
                elif kind == 3:
                    n = struct.unpack(">H", body[:2])[0]
                    self.pubs.append((body[2:2 + n].decode(), body[2 + n:]))
        except OSError:
            pass

    def publish(self, topic, payload):
        t = topic.encode()
        body = struct.pack(">H", len(t)) + t + payload
        self.conn.sendall(bytes([0x30, len(body)]) + body)


class FakeUart:
    name = "FAKE"

    def __init__(self):
        self.rx = bytearray()

    def inject(self, d):
        self.rx.extend(d)


def wait(m, cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        m.advance(0.05)
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_broker_bridge_roundtrip():
    broker = TinyBroker()
    m = Machine()
    n32g45x.build(m)
    f = Fc41d(m, Network(mqtt="broker", broker=("127.0.0.1", broker.port)), Fc41dConfig(reply_ms=0))
    u = FakeUart()
    f.attach(u)
    f.got_ip = True
    f.on_tx(b'AT+QMTOPEN=0,"x.amazonaws.com",8883\r\n')
    f.on_tx(b'AT+QMTCONN=0,"mst_abc"\r\n')
    assert wait(m, lambda: b"+QMTCONN: 0,0,0" in u.rx)
    assert broker.client_id == "mst_abc"
    f.on_tx(b'AT+QMTSUB=0,1,"marstek_energy/HMA-1/App/abc/ctrl",0\r\n')
    assert wait(m, lambda: broker.subs == ["marstek_energy/HMA-1/App/abc/ctrl"])
    f.on_tx(b'AT+QMTPUB=0,0,0,0,"marstek_energy/HMA-1/device/abc/ctrl",4,"p1=0"\r\n')
    assert wait(m, lambda: broker.pubs == [("marstek_energy/HMA-1/device/abc/ctrl", b"p1=0")])
    broker.publish("marstek_energy/HMA-1/App/abc/ctrl", b"cd=1")
    assert wait(m, lambda: b'+QMTRECV: 0,1,"marstek_energy/HMA-1/App/abc/ctrl",4,"cd=1"' in u.rx)
