"""Venus A: control and BMS firmware, inverter board (with on-board MPPT) modelled.

Differences to the Venus E 3.0 / D (from the Venus A RE notes and this
emulator):

* **BMS** (A 109-115, N32G45x): 13 LFP cells (2080 Wh), an **SH36735** front end
  (:mod:`~marsemu.periph.sh36735`) instead of the KA495XX, a bit-banged I2C
  EEPROM (PB10 SCL, PB11 SDA) that holds the master marker ``00 AA FF 55`` at
  0x01F6 — a pack only answers the control's poll as master.  In a stack every
  pack carries 1 at EEPROM 0x0202, and the master codes the others over the
  coding line (:class:`~marsemu.plant.venus3.CodingLine`) 2 s after boot.  Scaling measured
  against BMS 115: cells raw = mV x 6.4 (0x69), pack raw = mV x 0.256 (0x93),
  current 0.013045 A per count (0x91, read when the conversion-done flag
  0x59 bit 0 is set; 0x67 is the fast current channel, 0.1373 A per count),
  NTCs 0x5D..0x64 via the table below.
* **Inverter board** (Micro 1211, NS800RT Cortex-M7, no emulation): same class-4
  protocol, 48-byte telemetry, 1450 W limits (0x03/0x04 accept < 1451 W),
  charge limit from the BMS current x pack voltage x 1.06.  It also *is* the
  class-2 node: its MPPT stage (four PV inputs, 2400 W total) answers the
  control's MPPT commands.  The class-2 0xCB reply is a single index-0 frame
  (``65 00 BB 04``), so the control never stores an MPPT version (30205 = 0).
"""
from __future__ import annotations

import struct
from typing import Sequence

from ..periph.i2c_gpio import GpioI2cBus
from ..periph.n32g45x import I2cEeprom
from ..periph.sh36735 import Sh36735
from .battery import _interp
from .venus3 import CodingLine, MicroSpec, VenusBoard
from .venusd import MpptNode

#: Venus A inverter board, Micro v1211
MICRO_A_1211 = MicroSpec("VNSA-0 Micro 1211", 1211, 48, 1450, unsupported=(0x08, 0x0A, 0x14, 0x15, 0x17),
                         limit_accept_below=1451, max_power_w=1450.0, charge_bms_factor=1.06, state_style="a")

# NTC register value (0x5D..) -> degC, measured against BMS 115 (CAN 0x1840 frame)
A_NTC_RAW_TO_C = [
    (1536, 120.0), (2560, 105.7), (3584, 91.3), (4608, 80.8), (5632, 72.5), (6656, 65.6), (7680, 59.6),
    (8704, 54.3), (9728, 49.6), (10752, 45.2), (11776, 41.1), (12800, 37.3), (13824, 33.6), (14848, 30.1),
    (15872, 26.7), (16896, 23.3), (17920, 20.1), (18944, 16.8), (19968, 13.6), (20992, 10.3), (22016, 6.9),
    (23040, 3.5), (24064, -0.1), (25088, -3.9), (26112, -8.0), (27136, -12.4), (28160, -17.3), (29184, -23.1),
    (30208, -30.0),
]
_A_C_TO_RAW = sorted((c, r) for r, c in A_NTC_RAW_TO_C)
A_AMPS_PER_COUNT = 0.013045        # register 0x91, + = charge
A_FAST_AMPS_PER_COUNT = 0.13731    # register 0x67
MASTER_MARKER = bytes([0x00, 0xAA, 0xFF, 0x55])
#: BMS EEPROM byte: 1 = the pack is part of a stack (blank: a stand-alone unit)
STACK_FLAG = 0x202


class Sh36735FrontEnd(CodingLine):
    """Venus A BMS board: SH36735 front end fed from the pack, EEPROM with the master marker."""

    def __init__(self, emu, pack, master: bool = True, stacked: bool = False):
        self.emu = emu
        self.pack = pack
        m = emu.machine
        self.afe = Sh36735(cs=(m.bus.get("GPIOA"), 4))
        self.afe.measure = self._measure
        m.bus.get("SPI1").device = self.afe
        gb = m.bus.get("GPIOB")
        self.i2c = GpioI2cBus(gb, 10, gb, 11)
        self.eeprom = I2cEeprom(0x50, 65536, 2, 128)
        if master:                       # factory/pack-coding state of a single-pack unit
            self.eeprom.mem[0x1F6:0x1FA] = MASTER_MARKER
        if stacked:                      # part of a coded stack: the master codes the others at boot
            self.eeprom.mem[STACK_FLAG] = 1
        self.i2c.devices[0x50] = self.eeprom

    def _measure(self, c: Sh36735) -> None:
        p = self.pack
        cells = p.cell_voltages
        for i, v in enumerate(cells[:13]):
            c.set16(0x69 + 2 * i, int(round(v * 1000 * 6.4)))
        pack_mv = sum(cells[:13]) * 1000
        c.set16(0x93, int(round(pack_mv * 0.256)))
        c.set16(0x95, int(round(pack_mv * 0.256)))
        raw = int(_interp(_A_C_TO_RAW, p.temp_c))
        for k in range(4):
            c.set16(0x5D + 2 * k, raw)
        c.set16(0x67, int(round(p.current_a / A_FAST_AMPS_PER_COUNT)))
        c.set16(0x91, int(round(p.current_a / A_AMPS_PER_COUNT)))
        c.regs[0x59] |= 0x01                 # coulomb-counter conversion done

    def mos(self):
        m = self.emu.machine
        c = m.bus.get("GPIOC").regs.get(0x0C, 0)
        b = m.bus.get("GPIOB").regs.get(0x0C, 0)
        return {"charge": bool(c >> 5 & 1), "discharge": bool(b & 1)}


class AMppt(MpptNode):
    """The MPPT stage of the A inverter board, answering class-2 commands."""

    REF_WORD = 54                      # 30036 reads ~0.54 V on a Venus A
    POWER_CLAMP = (50, 2400)
    CMD_VOLTAGE_IS_CV = False          # the voltage field is an MPPT voltage setting

    def _version_reply(self, dest: int) -> None:
        # one frame with index 0: the control never commits it (30205 stays 0)
        self.bus.send(self, (MPPT_ADDR_A << 16) | (dest << 8) | 0xCB, struct.pack("<HH", 101, self.version), True,
                      t=self.sim.now)


MPPT_ADDR_A = 0x21


class VenusA(VenusBoard):
    """Venus A: Control 148-151, BMS 109-115 (13 cells, 2.08 kWh), Micro 1211 (modelled, with MPPT)."""

    MICRO = MICRO_A_1211
    CAPACITY_AH = 50.0
    CELLS = 13
    MAX_PACKS = 6

    def __init__(self, control, bms, *, pv: Sequence[float] = (0, 0, 0, 0), **kw):
        super().__init__(control, bms, **kw)
        self.mppt = self.bus.attach(AMppt(self.sim, self.pack, pv, version=1211))
        self.mppt.cv_source = lambda: self.micro.bms_chg_v or None
        self.dc_sources.append(self.mppt)

    def make_bms_front(self, emu, pack, index: int = 0):
        return Sh36735FrontEnd(emu, pack, master=index == 0, stacked=len(self.packs) > 1)

    def status(self) -> dict:
        st = super().status()
        st["pv_w"] = round(sum(self.mppt.pv_w), 1)
        st["pv_limit_w"] = self.mppt.limit_w
        return st


__all__ = ["VenusA", "Sh36735FrontEnd", "AMppt", "MICRO_A_1211"]
