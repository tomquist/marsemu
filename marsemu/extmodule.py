"""A real Wi-Fi module on the device's FC41D UART instead of the model.

The firmware then talks to actual hardware, e.g. a Quectel FC41D (stock, or
running ESPHome's hame_at) on the FC41D-TE-B eval kit, through a USB serial
adapter: everything the MCU sends goes out on the serial port, everything the
module answers is received by the MCU at line speed. Run with --realtime, so
the firmware's timeouts match the module's.

    from marsemu.extmodule import ExternalModule
    Emulator(fw, external_module="/dev/ttyUSB0")          # or "socket://host:port" (ser2net)

``port`` is anything pyserial's serial_for_url() opens; "PORT@BAUD" sets the
baud rate (default 115200, the FC41D's).
"""
from __future__ import annotations

import logging
import queue
import threading
import time

from .periph.n32g45x import UartDevice

log = logging.getLogger(__name__)


def parse_port(spec: str):
    """"/dev/ttyUSB0@115200" -> ("/dev/ttyUSB0", 115200)."""
    port, sep, baud = spec.rpartition("@")
    if sep and baud.isdigit():
        return port, int(baud)
    return spec, 115200


class ExternalModule(UartDevice):
    POLL_MS = 1.0

    def __init__(self, emu, spec: str, serial_port=None):
        self.emu = emu
        self.uart = None
        self.port, self.baud = parse_port(spec)
        if serial_port is None:
            try:
                import serial  # pyserial
            except ImportError as e:  # pragma: no cover
                raise RuntimeError("an external module needs pyserial: pip install pyserial") from e
            serial_port = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05)
        self.ser = serial_port
        self._rx: "queue.Queue[bytes]" = queue.Queue()
        self._line = {">": bytearray(), "<": bytearray()}
        #: wall-clock time the MCU last ended a line; module lines report how
        #: long after it they arrived (the module's real latency, whatever
        #: the emulator's speed)
        self._tx_wall = None
        self.tx_total = self.rx_total = 0
        self.closed = False
        threading.Thread(target=self._reader, daemon=True, name="ext-module").start()
        log.info("FC41D UART goes to %s (%d baud)", self.port, self.baud)

    # emulator side
    def attach(self, uart) -> None:
        self.uart = uart
        self._schedule()

    def power_on(self) -> None:
        """The real module boots on its own."""

    def on_tx(self, data: bytes) -> None:  # the MCU sends
        self.tx_total += len(data)
        if b"\n" in data or b"\r" in data:
            self._tx_wall = time.monotonic()
        self._trace(">", data)
        try:
            self.ser.write(data)
        except Exception as e:  # noqa: BLE001 - a vanished adapter must not stop the firmware
            log.warning("external module: write failed: %s", e)

    def _schedule(self) -> None:
        self.emu.machine.clock.after_ms(self.POLL_MS, self._poll, "ext-module")

    def _poll(self) -> None:
        self._schedule()
        while True:
            try:
                wall, data = self._rx.get_nowait()
            except queue.Empty:
                return
            self.rx_total += len(data)
            self._trace("<", data, wall)
            if self.uart is not None:
                self.uart.inject(data)

    def _trace(self, direction: str, data: bytes, wall: float = None) -> None:
        # The AT conversation as events, like the model's ("at" for MCU lines,
        # "at_rx" for module lines), so --at and listeners keep working.
        buf = self._line[direction]
        buf.extend(data)
        while b"\n" in buf:
            i = buf.index(b"\n")
            line = bytes(buf[:i]).rstrip(b"\r").decode("latin1")
            del buf[: i + 1]
            if not line:
                continue
            if direction == ">":
                self.emu.fc41d.emit("at", line)
            else:
                # only for what arrived after the MCU's last line
                ms = None
                if wall is not None and self._tx_wall is not None and wall >= self._tx_wall:
                    ms = (wall - self._tx_wall) * 1000
                self.emu.fc41d.emit("at_rx", line, ms)
        if len(buf) > 4096:  # binary payloads without line ends
            buf.clear()

    # serial side
    def _reader(self) -> None:
        while not self.closed:
            try:
                # Hand bytes on as soon as they arrive: read(512) would wait
                # for 512 bytes or the whole timeout, delaying every answer.
                chunk = self.ser.read(1)
                if chunk:
                    try:
                        waiting = self.ser.in_waiting
                    except Exception:  # noqa: BLE001 - not every port can tell
                        waiting = 0
                    if waiting:
                        chunk += self.ser.read(waiting)
            except Exception as e:  # noqa: BLE001
                log.warning("external module: read failed: %s", e)
                return
            if chunk:
                self._rx.put((time.monotonic(), bytes(chunk)))

    def close(self) -> None:
        self.closed = True
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
