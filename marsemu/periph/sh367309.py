"""SH367309 battery front end (5-16 cell monitor and protector) on I2C, as used
by the B2500.

Slave address 0x1A (0x34 write / 0x35 read).  CRC-8, polynomial 0x07, initial
value 0, over every byte on the wire after START, addresses included::

    write:  S 34 reg value crc P                     (one byte; NACKed on a bad CRC)
    read:   S 34 reg len  Sr 35 data… crc P          (crc over 34 reg len 35 data…)
    reset:  S 34 EA C0 crc P

EEPROM 0x00-0x19 holds the configuration, RAM 0x40-0x72 the status and the
measurements (16-bit big endian, signed).  Scales from the datasheet:

* cells 0x4E.. (CELL1 near VSS): mV = raw * 5 / 32
* temperatures 0x46/0x48/0x4A: raw = 32768 * Rt / (Rt + Rref), Rref = 6.8 kΩ + 0.05 kΩ * TR
* current 0x4C (VADC, 10 Hz) mA = 200 * raw / (26837 * Rsense),
  0x6E (CADC, 4 Hz) mA = 200 * raw / (21470 * Rsense); negative = discharge
* BSTATUS3 bit 6 DSGING / bit 7 CHGING: current flowing beyond the state
  detection voltage (200 uV across Rsense by default); the B2500 counts its
  SOC only while one of them is set
"""
from __future__ import annotations

import logging
import struct
from typing import Callable, Dict, List, Optional

from .n32g45x import I2cDevice
from .sh36735 import crc8_07

log = logging.getLogger(__name__)

ADDR = 0x1A
CONF, BALANCEH, BALANCEL, BSTATUS1, BSTATUS2, BSTATUS3 = 0x40, 0x41, 0x42, 0x43, 0x44, 0x45
TEMP1, CUR, CELL1, CADCD, BFLAG1, BFLAG2, RSTSTAT = 0x46, 0x4C, 0x4E, 0x6E, 0x70, 0x71, 0x72

#: EEPROM defaults: 14 cells LFP-like thresholds (the firmware rewrites what it wants)
DEFAULT_EEPROM = bytes([
    0x7E, 0x00, 0xF2, 0x34, 0xF2, 0xB0, 0x5A, 0x6E, 0xA8, 0x00, 0x00, 0xE6,
    0x84, 0x84, 0x83, 0x84, 0x52, 0x3A, 0x45, 0xC7, 0xBE, 0x30, 0x3A, 0xC7,
    0xBE, 0x28,
])


class Sh367309(I2cDevice):
    address = ADDR

    def __init__(self, eeprom: bytes = DEFAULT_EEPROM):
        self.regs = bytearray(0x80)
        self.regs[:len(eeprom)] = eeprom
        self.regs[CONF] = 0x70
        self.regs[BFLAG2] = 0x80                     # RST_FLG
        #: called before measurement registers are read: fn(chip)
        self.measure: Optional[Callable[["Sh367309"], None]] = None
        #: called after a RAM/EEPROM register write: fn(reg, value)
        self.write_hook: Optional[Callable[[int, int], None]] = None
        self.wire: List[int] = []
        self.reading = False
        self._out: List[int] = []
        self.reads: Dict[int, int] = {}
        self.writes: Dict[int, int] = {}
        self.bad_crc = 0
        #: charge/discharge state detection voltage across the sense resistor, V
        self.state_v = 200e-6
        self._dir = 0

    # -- measurements
    def set16(self, reg: int, value: int) -> None:
        struct.pack_into(">h", self.regs, reg, max(-32768, min(32767, int(round(value)))))

    def set_cells_mv(self, mv: List[float]) -> None:
        for i in range(16):
            self.set16(CELL1 + 2 * i, (mv[i] if i < len(mv) else 0.0) * 32 / 5)

    def rref_kohm(self) -> float:
        return 6.8 + 0.05 * (self.regs[0x19] & 0x7F)

    def set_ntc_kohm(self, k: int, r_kohm: float) -> None:
        self.set16(TEMP1 + 2 * k, 32768 * r_kohm / (r_kohm + self.rref_kohm()))

    def set_current(self, amps: float, rsense_ohm: float) -> None:
        self.set16(CUR, amps * 1000 * 26837 * rsense_ohm / 200)
        self.set16(CADCD, amps * 1000 * 21470 * rsense_ohm / 200)
        v = amps * rsense_ohm
        self._dir = 0x80 if v > self.state_v else 0x40 if v < -self.state_v else 0
        self._update_fets()

    def fets(self) -> int:
        """BSTATUS3 bits: 1 discharge FET, 2 charge FET."""
        return self.regs[BSTATUS3] & 3

    # -- I2C
    def start(self, read: bool) -> bool:
        if read:
            if len(self.wire) == 3:
                self.reading = True
                reg, n = self.wire[1], self.wire[2]
                self._prepare_read(reg, n)
            self.wire.append(ADDR << 1 | 1)
        else:
            self.wire = [ADDR << 1]
            self.reading = False
        return True

    def write(self, b: int) -> bool:
        if self.reading:
            return True
        w = self.wire
        w.append(b)
        if len(w) == 4:
            if crc8_07(w[:3]) != b:
                self.bad_crc += 1
                return False
            if w[1] == 0xEA and w[2] == 0xC0:
                self.soft_reset()
            else:
                self._write(w[1], w[2])
        return True

    def read(self) -> int:
        return self._out.pop(0) if self._out else 0xFF

    def stop(self) -> None:
        self.reading = False

    def _prepare_read(self, reg: int, n: int) -> None:
        self.reads[reg] = self.reads.get(reg, 0) + 1
        if self.measure and reg + n > TEMP1:
            self.measure(self)
        data = [self.regs[r] if (r <= 0x19 or 0x40 <= r <= 0x72) else 0xFF for r in range(reg, reg + n)]
        if reg <= BFLAG2 < reg + n:
            self.regs[BFLAG2] &= ~0x30                # VADC/CADC flags clear on read
        self._out = data + [crc8_07(self.wire[:3] + [ADDR << 1 | 1] + data)]

    def _write(self, reg: int, value: int) -> None:
        self.writes[reg] = self.writes.get(reg, 0) + 1
        if reg <= 0x18:
            self.regs[reg] = value
        elif reg == CONF:
            self.regs[CONF] = value & 0x7F           # OCRC reads back as written minus the strobe
            self._update_fets()
        elif reg in (BALANCEH, BALANCEL):
            self.regs[reg] = value
        elif reg in (BFLAG1, BFLAG2, RSTSTAT):
            self.regs[reg] &= value                  # write-0-to-clear
        else:
            return
        if self.write_hook:
            self.write_hook(reg, value)

    def _update_fets(self) -> None:
        c = self.regs[CONF]
        st = self.regs[BSTATUS3] & ~0xC7
        st |= (1 if c & 0x20 else 0) | (2 if c & 0x10 else 0) | (4 if c & 0x40 and self.regs[0] & 0x80 else 0) | self._dir
        self.regs[BSTATUS3] = st

    def soft_reset(self) -> None:
        self.regs[CONF] = 0x70
        self.regs[BFLAG2] |= 0x80
        self._update_fets()

    def tick(self) -> None:
        """A measurement cycle completed (VADC 10 Hz)."""
        self.regs[BFLAG2] |= 0x30
        self._update_fets()


__all__ = ["Sh367309", "ADDR"]
