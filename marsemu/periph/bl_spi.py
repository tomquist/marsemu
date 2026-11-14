"""Belling BL09xx energy-metering IC on bit-banged SPI (the CT002/CT003 meter).

The CT00x firmware drives the chip in software on GPIOA: PA4 chip select,
PA5 clock, PA7 data to the chip, PA6 data from it, most significant bit first.
It writes a bit, raises the clock, lowers it and then reads a bit, so the chip
presents each bit from the rising edge of its own clock.

Frames are six bytes:

* read:  ``0x82, addr`` then the chip sends ``d2 d1 d0 sum``
* write: ``0x81, addr, d2, d1, d0, sum``

with ``sum = ~(cmd + addr + d2 + d1 + d0) & 0xFF``.  Registers are 24 bits.
The register layout is the BL0906/BL0910 family's: 0x9E is the write-protect
register (writing 0x5555 unlocks the others and reads back as 0x5555), 0x9F
is soft reset (0x5A5A5A).  What the measurement registers mean for the
CT firmware is in :class:`BlMeter`'s docstring.
"""
from __future__ import annotations

import logging
import math
from typing import Callable, Dict, List, Optional

log = logging.getLogger(__name__)

READ, WRITE = 0x82, 0x81
WRPROT, SOFT_RESET = 0x9E, 0x9F
UNLOCK, RESET_MAGIC = 0x5555, 0x5A5A5A


def checksum(cmd: int, addr: int, value: int) -> int:
    return ~(cmd + addr + (value >> 16 & 0xFF) + (value >> 8 & 0xFF) + (value & 0xFF)) & 0xFF


class BlChip:
    """The register file and the frame protocol, independent of the pins."""

    def __init__(self):
        self.regs: Dict[int, int] = {}
        #: called with (addr) before a read is answered: lets a model refresh
        #: measurement registers on demand
        self.on_read: List[Callable[[int], None]] = []
        self.writes: List[tuple] = []        # (addr, value) the firmware wrote, in order
        self.reads = 0
        self.bad_frames = 0
        self.unlocked = False

    def read(self, addr: int) -> int:
        for fn in self.on_read:
            fn(addr)
        self.reads += 1
        if addr == WRPROT:
            return UNLOCK if self.unlocked else 0
        return self.regs.get(addr, 0) & 0xFFFFFF

    def write(self, addr: int, value: int) -> None:
        self.writes.append((addr, value))
        if len(self.writes) > 256:
            del self.writes[:128]
        if addr == WRPROT:
            self.unlocked = value == UNLOCK
            return
        if not self.unlocked:
            log.debug("BL write 0x%02x=0x%06x while locked: ignored", addr, value)
            return
        if addr == SOFT_RESET:
            if value == RESET_MAGIC:
                log.debug("BL soft reset")
            return
        self.regs[addr] = value & 0xFFFFFF


class GpioSpiSlave:
    """Puts a :class:`BlChip` on four GPIO pins."""

    def __init__(self, gpio, chip: BlChip, *, cs: int = 4, sck: int = 5, mosi: int = 7, miso: int = 6):
        self.g, self.chip = gpio, chip
        self.cs_p, self.sck_p, self.mosi_p, self.miso_p = cs, sck, mosi, miso
        self.cs = 1
        self.mosi = 0
        self.rx: List[int] = []      # bits received this frame
        self.tx: List[int] = []      # bits to send, in order
        self.frames = 0
        gpio.watchers.append(self._pin)
        self._drive(1)

    def _drive(self, level: int) -> None:
        self.g.inputs[self.miso_p] = level & 1

    def _pin(self, pin: int, lvl: int) -> None:
        if pin == self.cs_p:
            if lvl == 0 and self.cs:
                self.rx, self.tx = [], []
            elif lvl and not self.cs:
                self._end()
            self.cs = lvl
        elif pin == self.mosi_p:
            self.mosi = lvl
        elif pin == self.sck_p and lvl and not self.cs:
            self.rx.append(self.mosi)
            n = len(self.rx)
            if n == 16 and self._byte(0) == READ:
                addr = self._byte(1)
                v = self.chip.read(addr)
                out = [v >> 16 & 0xFF, v >> 8 & 0xFF, v & 0xFF, checksum(READ, addr, v)]
                self.tx = [(b >> (7 - i)) & 1 for b in out for i in range(8)]
            k = n - 17                   # data bits are clocks 17-48
            if 0 <= k < len(self.tx):
                self._drive(self.tx[k])
            else:
                self._drive(1)

    def _byte(self, i: int) -> int:
        bits = self.rx[8 * i:8 * i + 8]
        return int("".join(map(str, bits)), 2) if len(bits) == 8 else -1

    def _end(self) -> None:
        self._drive(1)
        self.frames += 1
        if len(self.rx) < 48:
            return
        b = [self._byte(i) for i in range(6)]
        if b[0] == WRITE:
            value = b[2] << 16 | b[3] << 8 | b[4]
            if checksum(WRITE, b[1], value) != b[5]:
                self.chip.bad_frames += 1
                return
            self.chip.write(b[1], value)


class BlMeter:
    """What the CT002's clamps see, written into the chip's registers.

    Found by setting one register at a time on CT002 V120/V124 and reading the
    meter's answers to a battery:

    * 0x22, 0x23, 0x24: active power of phases A, B and C, signed 24 bits,
      134.193 counts per watt (the firmware's scale with the calibration it
      writes at start-up; 1000 W starts at 134193 counts, 2000 W at 268386).
      The firmware cuts each phase toward zero to whole watts and adds the
      unrounded phases for the total.  Linear to at least +-29800 W; the
      full-scale codes 0x7FFFFF and 0x800000 come out as 3025 W.
    * Every other register the firmware reads (RMS currents and voltages,
      energy counters, frequency) feeds only its cloud report.
    """

    POWER_REGS = {"A": 0x22, "B": 0x23, "C": 0x24}
    COUNTS_PER_W = 134.193

    def __init__(self, chip: BlChip):
        self.chip = chip
        self.power = {"A": 0.0, "B": 0.0, "C": 0.0}
        self.set_power(0.0, 0.0, 0.0)

    def set_power(self, a: float, b: float, c: float) -> None:
        """Grid power per phase in watts, + import, - export.  A whole number
        of watts is reported as exactly that: the register gets the smallest
        count the firmware reads as that many watts."""
        for ph, w in zip("ABC", (a, b, c)):
            self.power[ph] = float(w)
            counts = math.ceil(abs(w) * self.COUNTS_PER_W - 1e-6)
            counts = min(counts, 0x7FFFFE) * (1 if w >= 0 else -1)
            self.chip.regs[self.POWER_REGS[ph]] = counts & 0xFFFFFF


def attach_ct_meter(emu) -> Optional[BlChip]:
    """Give a CT00x its metering chip (GPIOA PA4-PA7)."""
    gpio = emu.machine.bus.peripherals.get("GPIOA")
    if gpio is None:
        return None
    chip = BlChip()
    emu.meter_chip = chip
    emu.meter_spi = GpioSpiSlave(gpio, chip)
    emu.ct_meter = BlMeter(chip)
    return chip
