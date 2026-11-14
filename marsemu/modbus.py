"""Modbus on the emulated device's RS485 port, served over TCP.

The Venus E 3.0, D and A control firmwares contain a complete Modbus RTU slave
on their RS485 UART (USART2 on the E 3.0 and D, USART1 on the A; slave address
1 by default, stored in the control EEPROM).  :class:`ModbusGateway` connects a
virtual RS485 line to that UART and serves it on a TCP port, so Modbus clients
such as Home Assistant's Modbus integration can talk to the real firmware:

* ``mode="tcp"``: a Modbus TCP server; each request is passed on as an RTU
  frame (unit id = slave address) and the slave's answer is sent back.  No
  answer within ``timeout_s`` of emulated time gives exception 0x0B (gateway
  target device failed to respond), as from a real RS485 gateway.
* ``mode="rtu"``: RTU over TCP (Home Assistant ``type: rtuovertcp``): frames
  go to the line as they are, answers come back as they are.

Requests arrive on network threads and are handed to the emulator on its own
clock, so the firmware sees them at line speed in emulated time and it does
not matter whether the emulation runs faster or slower than real time.

    from marsemu import VenusE3
    from marsemu.modbus import ModbusGateway
    v = VenusE3("control.bin", "bms.bin")
    gw = ModbusGateway(v.control, port=5020)          # Modbus TCP on :5020
    v.run(600)
"""
from __future__ import annotations

import logging
import queue
import socket
import struct
import threading
from typing import Optional

log = logging.getLogger(__name__)


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC (little endian, as sent on the line)."""
    c = 0xFFFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return bytes([c & 0xFF, c >> 8])


def rs485_uart(emu) -> str:
    """The UART the firmware's Modbus RTU slave listens on."""
    img = emu.image
    # every control image names all three models once; its own model id ("VNSA-0") only it carries
    if b"VNSA-0" in img:
        return "USART1"
    if b"VNSE3-0" in img or b"VNSD-0" in img:
        return "USART2"
    raise ValueError(f"no known RS485 Modbus port for profile {emu.profile.name}")


def response_length(frame: bytes) -> Optional[int]:
    """Length of an RTU response once its header is known, else None."""
    if len(frame) < 2:
        return None
    fc = frame[1]
    if fc & 0x80:
        return 5
    if fc in (1, 2, 3, 4, 0x17):
        return 3 + frame[2] + 2 if len(frame) >= 3 else None
    if fc in (5, 6, 15, 16):
        return 8
    return None


class _Request:
    def __init__(self, frame: bytes, expect_reply: bool):
        self.frame = frame
        self.expect_reply = expect_reply
        self.reply: Optional[bytes] = None
        self.done = threading.Event()


class ModbusGateway:
    """Serves the firmware's RS485 Modbus slave on a TCP port (see module doc)."""

    POLL_MS = 5.0

    def __init__(self, emu, port: int = 502, mode: str = "tcp", host: str = "0.0.0.0",
                 uart: Optional[str] = None, timeout_s: float = 1.0):
        if mode not in ("tcp", "rtu"):
            raise ValueError("mode must be 'tcp' or 'rtu'")
        self.emu = emu
        self.mode = mode
        self.timeout_s = timeout_s
        self.uart = emu.machine.bus.get(uart or rs485_uart(emu))
        self.uart.tx_listeners.append(self._on_tx)
        self.requests: "queue.Queue[_Request]" = queue.Queue()
        self._current: Optional[_Request] = None
        self._rx = bytearray()
        self._deadline = 0.0
        self.served = 0
        self.timeouts = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True, name=f"modbus-{self.port}").start()
        self._schedule()
        log.info("Modbus %s on %s:%d -> %s", "TCP" if mode == "tcp" else "RTU over TCP", host, self.port,
                 self.uart.name)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    # -- emulator side (runs on the emulator's clock) ----------------------
    def _schedule(self) -> None:
        self.emu.machine.clock.after_ms(self.POLL_MS, self._poll, "modbus")

    def _now(self) -> float:
        return self.emu.machine.clock.seconds

    def _poll(self) -> None:
        self._schedule()
        cur = self._current
        if cur is not None:
            if self._reply_complete():
                self._finish(bytes(self._rx))
            elif self._now() >= self._deadline:
                self.timeouts += 1
                self._finish(None)
            return
        try:
            req = self.requests.get_nowait()
        except queue.Empty:
            return
        self._rx.clear()
        if not req.expect_reply:              # broadcast: nobody answers
            self.uart.inject(req.frame)
            req.done.set()
            return
        # pending before the frame goes out, so even an immediate answer is caught;
        # the request itself takes len * 10 bits on the line before the timeout starts
        self._current = req
        self._deadline = self._now() + self.timeout_s + len(req.frame) * 10.0 / max(self.uart.baud(), 1200)
        self.uart.inject(req.frame)

    def _on_tx(self, data: bytes) -> None:
        if self._current is not None:
            self._rx.extend(data)

    def _reply_complete(self) -> bool:
        rx, addr = self._rx, self._current.frame[0]
        while rx and rx[0] != addr:           # bytes before the answer (e.g. console output)
            del rx[0]
        n = response_length(bytes(rx))
        if n is None or len(rx) < n:
            return False
        if crc16(bytes(rx[:n - 2])) != bytes(rx[n - 2:n]):
            del rx[0]                          # not a frame after all: resynchronise
            return self._reply_complete() if rx else False
        del rx[n:]
        return True

    def _finish(self, reply: Optional[bytes]) -> None:
        req, self._current = self._current, None
        req.reply = reply
        self.served += 1
        req.done.set()

    # -- network side --------------------------------------------------------
    def _accept(self) -> None:
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            log.info("Modbus client %s:%d", *addr)
            target = self._serve_tcp if self.mode == "tcp" else self._serve_rtu
            threading.Thread(target=target, args=(conn,), daemon=True).start()

    def _call(self, frame: bytes) -> Optional[bytes]:
        req = _Request(frame, expect_reply=frame[0] != 0)
        self.requests.put(req)
        # generous wall-clock bound: the emulated timeout normally decides
        req.done.wait(30.0)
        return req.reply

    @staticmethod
    def _recv_exact(conn, n: int) -> Optional[bytes]:
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _serve_tcp(self, conn) -> None:
        with conn:
            while True:
                try:
                    head = self._recv_exact(conn, 7)
                    if head is None:
                        return
                    tid, proto, length, unit = struct.unpack(">HHHB", head)
                    pdu = self._recv_exact(conn, length - 1)
                    if pdu is None:
                        return
                except OSError:
                    return
                if proto != 0 or not pdu:
                    continue
                frame = bytes([unit]) + pdu
                reply = self._call(frame + crc16(frame))
                if unit == 0:
                    continue                    # broadcast: no answer
                if reply is None:
                    out = bytes([pdu[0] | 0x80, 0x0B])
                else:
                    out = reply[1:-2]
                try:
                    conn.sendall(struct.pack(">HHHB", tid, 0, len(out) + 1, unit) + out)
                except OSError:
                    return

    def _serve_rtu(self, conn) -> None:
        with conn:
            buf = bytearray()
            while True:
                try:
                    chunk = conn.recv(512)
                except OSError:
                    return
                if not chunk:
                    return
                buf.extend(chunk)
                while len(buf) >= 4:
                    n = _request_length(bytes(buf))
                    if n is None or len(buf) < n:
                        break
                    frame, buf = bytes(buf[:n]), buf[n:]
                    if crc16(frame[:-2]) != frame[-2:]:
                        log.warning("Modbus RTU: bad CRC from client, dropped %s", frame.hex())
                        buf.clear()
                        break
                    reply = self._call(frame)
                    if reply:
                        try:
                            conn.sendall(reply)
                        except OSError:
                            return


def _request_length(frame: bytes) -> Optional[int]:
    """Length of an RTU request once its header is known."""
    if len(frame) < 2:
        return None
    fc = frame[1]
    if fc in (1, 2, 3, 4, 5, 6):
        return 8
    if fc in (15, 16):
        return 9 + frame[6] if len(frame) >= 7 else None
    if fc == 0x17:
        return 13 + frame[10] if len(frame) >= 11 else None
    return 8


__all__ = ["ModbusGateway", "crc16", "rs485_uart"]
