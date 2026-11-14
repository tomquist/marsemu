"""UART on a TCP port: firmware output reaches clients, client bytes reach the firmware."""
import socket
import time
import types

from marsemu.uartbridge import UartBridge


class FakeUart:
    name = "UART5"

    def __init__(self):
        self.tx_listeners = []
        self.received = bytearray()

    def inject(self, data):
        self.received.extend(data)


def test_bridge_both_directions():
    uart = FakeUart()
    emu = types.SimpleNamespace(machine=types.SimpleNamespace(
        bus=types.SimpleNamespace(get=lambda n: uart),
        clock=types.SimpleNamespace(after_ms=lambda ms, fn, name: None)))
    br = UartBridge(emu, "UART5", 0, host="127.0.0.1")
    with socket.create_connection(("127.0.0.1", br.port), timeout=5) as c:
        for _ in range(100):                       # wait until the bridge has registered the client
            if br.clients:
                break
            time.sleep(0.01)
        for listener in uart.tx_listeners:         # the firmware sends
            listener(b"\x73\x00\x0e\x31")
        assert c.recv(16) == b"\x73\x00\x0e\x31"
        c.sendall(b"hello")                        # a client sends
        for _ in range(100):
            br._poll()                             # what the emulator's clock does
            if uart.received:
                break
            time.sleep(0.01)
    br.close()
    assert bytes(uart.received) == b"hello"
