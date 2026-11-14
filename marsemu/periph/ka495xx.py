"""KA495XX-family battery front end (cell monitor) on an SPI bus.

Wire format, as used by the Venus E 3.0 BMS (all bytes MSB first, CRC-8
polynomial 0xD5, initial value 0xD5):

* write:       ``E0  reg<<1  hi lo  crc(E0 reg<<1 hi lo)``
* read:        ``E1  reg<<1  crc``  then the chip clocks out ``hi lo crc``
* burst read:  ``E1  reg<<1|1  n-1  crc`` then ``n`` words and one CRC

Measurement registers are filled from a :class:`CellSource` (normally the
battery model of a plant) whenever the firmware starts a conversion.
"""
from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

from .n32g45x import SpiDevice

log = logging.getLogger(__name__)


def crc8(data, crc: int = 0xD5) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Ka495xx(SpiDevice):
    def __init__(self, cs=None, cs_active_high: bool = True):
        self.cmd: List[int] = []
        self.out: List[int] = []
        self.selected = False
        self.cs_active_high = cs_active_high
        self.trace: Optional[Callable[[str, int, int], None]] = None
        self.reads: Dict[int, int] = {}
        self.writes: Dict[int, int] = {}
        self.read_hook: Optional[Callable[[int], Optional[int]]] = None
        self.write_hook: Optional[Callable[[int, int], None]] = None
        self.resp_crc_mode = "data"
        self.measure: Optional[Callable[["Ka495xx"], None]] = None
        self.reset()
        if cs is not None:
            gpio, pin = cs
            gpio.watchers.append(lambda p, lvl: self._cs(bool(lvl) == cs_active_high) if p == pin else None)

    # -- framing
    def _cs(self, active: bool) -> None:
        if active and not self.selected:
            self.cmd.clear()
            self.out.clear()
        self.selected = active

    def xfer(self, b: int) -> int:
        if self.out:
            r = self.out.pop(0)
            self.cmd.append(b)
            return r
        self.cmd.append(b)
        c = self.cmd
        if len(c) >= 3 and c[0] == 0xE1 and not c[1] & 1:
            if len(c) == 3:
                reg = c[1] >> 1
                if crc8(c[:2]) != c[2]:
                    log.debug("KA495XX read crc error %s", bytes(c).hex())
                v = self.read(reg)
                hi, lo = v >> 8, v & 0xFF
                crc = crc8([c[0], c[1], hi, lo]) if self.resp_crc_mode == "full" else crc8([hi, lo])
                self.out = [hi, lo, crc]
        elif len(c) == 4 and c[0] == 0xE1 and c[1] & 1:
            reg, n = c[1] >> 1, c[2] + 1
            data = []
            for i in range(n):
                v = self.read(reg + i)
                data += [v >> 8, v & 0xFF]
            crc = crc8(c[:3] + data) if self.resp_crc_mode == "full" else crc8(data)
            self.out = data + [crc]
        elif len(c) == 5 and c[0] == 0xE0:
            if crc8(c[:4]) == c[4]:
                self.write(c[1] >> 1, (c[2] << 8) | c[3])
            else:
                log.debug("KA495XX write crc error %s", bytes(c).hex())
        return 0x00

    # -- chip behaviour
    REG_ID = 0x09        # reads 0x000A; checked against the firmware's expected-register table
    REG_CMD = 0x0C       # bit 0/1: start a conversion, self-clearing
    REG_STATUS = 0x1C    # bits 0-4 state (1 = active), bit 7 / bits 5-6: conversion done (write 1 to clear)
    # The chip scans continuously: a cleared "done" flag is set again (with
    # fresh measurements) by the next read of the status register.

    def reset(self) -> None:
        self.regs = [0] * 128
        self.regs[self.REG_ID] = 0x000A
        self.regs[self.REG_STATUS] = 0x0001

    def _command(self, value: int) -> None:
        if value & 3:
            if self.measure:
                self.measure(self)
            self.regs[self.REG_STATUS] |= 0xE0
        self.regs[self.REG_CMD] = 0

    # -- registers
    def read(self, reg: int) -> int:
        reg &= 0x7F
        self.reads[reg] = self.reads.get(reg, 0) + 1
        if reg == self.REG_STATUS and (self.regs[reg] & 0xE0) != 0xE0:
            if self.measure:
                self.measure(self)
            self.regs[reg] |= 0xE0
        if self.read_hook:
            v = self.read_hook(reg)
            if v is not None:
                return v & 0xFFFF
        return self.regs[reg]

    def write(self, reg: int, value: int) -> None:
        reg &= 0x7F
        self.writes[reg] = self.writes.get(reg, 0) + 1
        if reg == self.REG_CMD:
            self._command(value)
        elif reg == self.REG_STATUS:
            self.regs[reg] &= ~(value & 0xE0)
        else:
            self.regs[reg] = value
        if self.write_hook:
            self.write_hook(reg, value)
