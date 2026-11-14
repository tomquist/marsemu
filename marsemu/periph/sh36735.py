"""SH36735 battery front end (cell monitor) on an SPI bus, as used by the
Venus A BMS (13 cells).

Wire format (CRC-8, polynomial 0x07, initial value 0)::

    write:  MOSI 01 reg value crc(01 reg value) xx      MISO FF 01 reg value A5
    read:   MOSI 02 reg len  xx … (len + 2)             MISO FF 02 reg len data… crc(FF 02 reg len data…)

The chip echoes the command one byte late and then clocks out the data.
Registers are bytes; multi-byte values are big endian.
"""
from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

from .n32g45x import SpiDevice

log = logging.getLogger(__name__)


def crc8_07(data, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Sh36735(SpiDevice):
    def __init__(self, cs=None, cs_active_high: bool = True):
        self.regs = bytearray(256)
        self.cmd: List[int] = []
        self.selected = False
        self.measure: Optional[Callable[["Sh36735"], None]] = None
        self.write_hook: Optional[Callable[[int, int], None]] = None
        self.reads: Dict[int, int] = {}
        self.writes: Dict[int, int] = {}
        self._out: List[int] = []
        if cs is not None:
            gpio, pin = cs
            gpio.watchers.append(lambda p, lvl: self._cs(bool(lvl) == cs_active_high) if p == pin else None)

    def _cs(self, active: bool) -> None:
        if active and not self.selected:
            self.cmd = []
            self._out = []
        self.selected = active

    def xfer(self, b: int) -> int:
        c = self.cmd
        c.append(b)
        n = len(c)
        if n == 1:
            return 0xFF
        if c[0] == 0x02:                       # read
            if n <= 3:
                if n == 3:
                    self._prepare_read(c[1], c[2])
                return c[n - 2]
            if n == 4:
                return c[2]
            return self._out.pop(0) if self._out else 0xFF
        if c[0] == 0x01:                       # write
            if n <= 4:
                if n == 4 and crc8_07(c[:3]) == c[3]:
                    self._write(c[1], c[2])
                return c[n - 2]
            return 0xA5
        return 0xFF

    def _prepare_read(self, reg: int, length: int) -> None:
        self.reads[reg] = self.reads.get(reg, 0) + 1
        if self.measure and reg >= 0x58:
            self.measure(self)
        data = list(self.regs[reg: reg + length]) + [0] * max(0, reg + length - 256)
        # the CRC covers everything the chip clocked out, the leading FF included
        self._out = data + [crc8_07([0xFF, 0x02, reg, length] + data)]

    def _write(self, reg: int, value: int) -> None:
        self.writes[reg] = self.writes.get(reg, 0) + 1
        self.regs[reg] = value
        if self.write_hook:
            self.write_hook(reg, value)

    # helpers for models
    def set16(self, reg: int, value: int) -> None:
        value &= 0xFFFF
        self.regs[reg] = value >> 8
        self.regs[reg + 1] = value & 0xFF
