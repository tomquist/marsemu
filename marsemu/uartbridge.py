"""Any UART of the emulated MCU on a TCP port (like ser2net).

Bytes the firmware sends go to every connected client; bytes a client sends
are received by the firmware at line speed.  Use it to look at a device's
serial ports with your own tools, or to play the device on the other end
(e.g. a B2500 extension pack on UART5)::

    from marsemu.uartbridge import UartBridge
    UartBridge(emu, "UART5", 7005)       # nc 127.0.0.1 7005 | xxd
"""
from __future__ import annotations

import logging
import queue
import socket
import threading
from typing import List

log = logging.getLogger(__name__)


class UartBridge:
    POLL_MS = 2.0

    def __init__(self, emu, uart: str, port: int, host: str = "0.0.0.0"):
        self.emu = emu
        self.uart = emu.machine.bus.get(uart)
        if self.uart is None or not hasattr(self.uart, "inject"):
            raise ValueError(f"no UART named {uart}")
        self.clients: List[socket.socket] = []
        self._lock = threading.Lock()
        self._rx: "queue.Queue[bytes]" = queue.Queue()
        self.uart.tx_listeners.append(self._on_tx)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(4)
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True, name=f"uart-{uart}").start()
        self._schedule()
        log.info("%s on %s:%d", uart, host, self.port)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    # emulator side
    def _schedule(self) -> None:
        self.emu.machine.clock.after_ms(self.POLL_MS, self._poll, "uart-bridge")

    def _poll(self) -> None:
        self._schedule()
        while True:
            try:
                data = self._rx.get_nowait()
            except queue.Empty:
                return
            self.uart.inject(data)

    def _on_tx(self, data: bytes) -> None:
        with self._lock:
            clients = list(self.clients)
        for c in clients:
            try:
                c.sendall(data)
            except OSError:
                self._drop(c)

    # network side
    def _drop(self, c) -> None:
        with self._lock:
            if c in self.clients:
                self.clients.remove(c)
        try:
            c.close()
        except OSError:
            pass

    def _accept(self) -> None:
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            log.info("%s client %s:%d", self.uart.name, *addr)
            with self._lock:
                self.clients.append(conn)
            threading.Thread(target=self._read, args=(conn,), daemon=True).start()

    def _read(self, conn) -> None:
        while True:
            try:
                data = conn.recv(1024)
            except OSError:
                data = b""
            if not data:
                self._drop(conn)
                return
            self._rx.put(data)


__all__ = ["UartBridge"]
