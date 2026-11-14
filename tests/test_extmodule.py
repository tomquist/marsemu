"""A real module on the FC41D UART: MCU bytes go to the serial port, the
module's answers reach the MCU, and the AT conversation is reported."""
import queue
import time
import types

from marsemu.extmodule import ExternalModule, parse_port


class FakeSerial:
    def __init__(self):
        self.written = bytearray()
        self.incoming: "queue.Queue[bytes]" = queue.Queue()

    def write(self, data):
        self.written.extend(data)

    def read(self, n):
        try:
            return self.incoming.get(timeout=0.02)
        except queue.Empty:
            return b""

    def close(self):
        pass


class FakeUart:
    def __init__(self):
        self.received = bytearray()

    def inject(self, data):
        self.received.extend(data)


def test_parse_port():
    assert parse_port("/dev/ttyUSB0") == ("/dev/ttyUSB0", 115200)
    assert parse_port("/dev/cu.usbserial-1@921600") == ("/dev/cu.usbserial-1", 921600)
    assert parse_port("socket://10.0.0.5:7000") == ("socket://10.0.0.5:7000", 115200)


def test_both_directions_and_trace():
    events = []
    emu = types.SimpleNamespace(
        machine=types.SimpleNamespace(clock=types.SimpleNamespace(after_ms=lambda ms, fn, name: None)),
        fc41d=types.SimpleNamespace(emit=lambda ev, *a: events.append((ev,) + a)))
    ser = FakeSerial()
    mod = ExternalModule(emu, "/dev/null", serial_port=ser)
    uart = FakeUart()
    mod.attach(uart)

    mod.on_tx(b"AT+QVERSION\r")          # the firmware sends, in pieces
    mod.on_tx(b"\n")
    assert bytes(ser.written) == b"AT+QVERSION\r\n"

    ser.incoming.put(b"+QVERSION:FC41D\r\nOK\r\n")   # the module answers
    for _ in range(100):
        mod._poll()                       # what the emulator's clock does
        if uart.received:
            break
        time.sleep(0.01)
    mod.close()
    assert bytes(uart.received) == b"+QVERSION:FC41D\r\nOK\r\n"
    assert ("at", "AT+QVERSION") in events
    rx = [e for e in events if e[0] == "at_rx"]
    assert [e[1] for e in rx] == ["+QVERSION:FC41D", "OK"]
    assert all(e[2] is not None and 0 <= e[2] < 5000 for e in rx)  # latency in ms


def test_wifi_password_redacted_in_transcript():
    from marsemu.cli import redact_wifi
    assert redact_wifi("AT+QSTAAPINFODEF=home,s3cret,x") == "AT+QSTAAPINFODEF=home,<password>"
    assert redact_wifi("s #home<.,.>s3cret[") == "s #home<.,.><password>"
    assert redact_wifi("AT+QMTOPEN=0,\"h\",1883") == "AT+QMTOPEN=0,\"h\",1883"
