"""Modbus gateway: TCP and RTU-over-TCP clients reach a slave on the RS485 UART."""
import socket
import struct
import threading
import time
import types

from marsemu.modbus import ModbusGateway, crc16, response_length


def test_crc16_matches_the_modbus_reference():
    assert crc16(bytes.fromhex("01030000000A")) == bytes.fromhex("C5CD")


def test_response_length():
    assert response_length(bytes.fromhex("010314")) == 3 + 0x14 + 2
    assert response_length(bytes.fromhex("0106")) == 8
    assert response_length(bytes.fromhex("0183")) == 5
    assert response_length(b"\x01") is None


class FakeUart:
    """An RS485 line with one slave (address 1) that answers FC03 with
    register numbers as values and echoes FC06; it ignores other addresses."""

    name = "USART2"

    def __init__(self):
        self.tx_listeners = []
        self.injected = []

    def baud(self):
        return 115200

    def inject(self, frame: bytes) -> None:
        self.injected.append(frame)
        addr, fc = frame[0], frame[1]
        if addr != 1:
            return
        if fc == 3:
            start, n = struct.unpack(">HH", frame[2:6])
            body = bytes([1, 3, 2 * n]) + b"".join(struct.pack(">H", (start + i) & 0xFFFF) for i in range(n))
            reply = body + crc16(body)
        else:
            reply = frame
        # some console noise first, then the answer in two pieces
        for piece in (b"\x1b[2J", reply[:3], reply[3:]):
            for listener in self.tx_listeners:
                listener(piece)


class FakeEmu:
    def __init__(self):
        self.uart = FakeUart()
        self.clock = types.SimpleNamespace(seconds=0.0, after_ms=lambda ms, fn, name: None)
        self.machine = types.SimpleNamespace(clock=self.clock, bus=types.SimpleNamespace(get=lambda n: self.uart))
        self.image = b"VNSE3-0"


def drive(gw, emu, until, limit_s=5.0):
    """Run the gateway's emulator-side poll as the emulator's clock would."""
    end = time.time() + limit_s
    while not until() and time.time() < end:
        emu.clock.seconds += gw.POLL_MS / 1000
        gw._poll()
        time.sleep(0.001)


def test_modbus_tcp_read_write_and_timeout():
    emu = FakeEmu()
    gw = ModbusGateway(emu, port=0, mode="tcp", host="127.0.0.1")
    answers = []

    def client():
        with socket.create_connection(("127.0.0.1", gw.port), timeout=5) as s:
            for tid, unit, pdu in ((1, 1, bytes.fromhex("0379180002")),      # read 31000-31001
                                   (2, 1, bytes.fromhex("06a41055aa")),      # write 42000 = 0x55AA
                                   (3, 7, bytes.fromhex("0300000001"))):     # nobody at unit 7
                s.sendall(struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit) + pdu)
                head = s.recv(7)
                tid_r, _, length, unit_r = struct.unpack(">HHHB", head)
                answers.append((tid_r, unit_r, s.recv(length - 1)))
    t = threading.Thread(target=client)
    t.start()
    drive(gw, emu, lambda: len(answers) == 3)
    t.join(5)
    gw.close()
    assert answers[0] == (1, 1, bytes.fromhex("03047918") + bytes.fromhex("7919"))
    assert answers[1] == (2, 1, bytes.fromhex("06a41055aa"))
    assert answers[2] == (3, 7, bytes.fromhex("830b"))                   # gateway target failed to respond
    assert emu.uart.injected[0] == bytes.fromhex("0103791800 02".replace(" ", "")) + crc16(bytes.fromhex("010379180002"))
    assert gw.timeouts == 1


def test_modbus_rtu_over_tcp_passes_frames_through():
    emu = FakeEmu()
    gw = ModbusGateway(emu, port=0, mode="rtu", host="127.0.0.1")
    got = []
    req = bytes.fromhex("010300000002")
    req += crc16(req)

    def client():
        with socket.create_connection(("127.0.0.1", gw.port), timeout=5) as s:
            s.sendall(req[:3])
            time.sleep(0.05)                                                 # split over two segments
            s.sendall(req[3:])
            got.append(s.recv(64))
    t = threading.Thread(target=client)
    t.start()
    drive(gw, emu, lambda: bool(got))
    t.join(5)
    gw.close()
    body = bytes.fromhex("01030400000001")
    assert got == [body + crc16(body)]
