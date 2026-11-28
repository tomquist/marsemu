"""Venus D: Venus E 3.0 hardware plus a PV/MPPT controller on the CAN bus.

The MPPT controller is a separate MCU (a TMS320F28035).  Its firmware (MPPT
102.5/104) runs on a model of its board when given (:mod:`marsemu.plant.mppt`);
otherwise :class:`MpptNode` models it from what the control firmware sends and
parses (Control v151):

* control -> MPPT (class 2, node 1): ``0x00012103`` poll (every 0.5 s),
  ``0x000121CB`` version query, ``0x000121CC`` date/time, ``0x00012102`` PV
  power limit (one 4-byte frame and one 8-byte frame, frame index 0 and 1)
* MPPT -> control: replies ``index<<24 | 0x21<<16 | dest<<8 | func`` like the
  inverter's; the poll answer is 56 bytes in 7 frames that the control copies
  verbatim to its MPPT block (v151: 0x2001776C):

  ======  ===================================================  =========
  word    meaning (Modbus register)                            scale
  ======  ===================================================  =========
  1       error word (37023)                                   bits
  3       warning word (37024)                                 bits
  4+3k    PV string k+1 voltage (30020..30023)                 0.1 V
  5+3k    PV string k+1 current (30024..30027)                 0.1 A
  6+3k    PV string k+1 power (30037..30040)                   0.1 W
  24      battery-side voltage (30028)                         0.1 V
  25      battery-side current, + into the battery (30029)     0.1 A
  26      "base voltage" (30036), 2.19 V on real units         0.01 V
  ======  ===================================================  =========

  The ``0xCB`` answer is ``<HHHH`` model code, version (104 on real units), …
  (the MPPT firmware: ``<101, 104, 0xFFFF, 1>``, after a header frame), and
  the poll answer of the firmware has a header frame (index 0, the number of
  data frames) in front of the seven; the control takes it either way.

The BMS board (:class:`DBmsFrontEnd`) has the KA49503 front end and a 24C512
EEPROM on bit-banged I2C (PB10/PB11).  Several packs form a stack (see
:class:`~marsemu.plant.venus3.CodingLine`): a pack whose EEPROM byte 0x202 is
not 1 claims the master role at boot; the others carry 1 there, announce
themselves on CAN and are coded by the master.
"""
from __future__ import annotations

import logging
import struct
from typing import List, Optional, Sequence

from ..periph.i2c_gpio import GpioI2cBus
from ..periph.n32g45x import I2cEeprom
from .canbus import Frame, Node
from .venus3 import AMPS_PER_COUNT, MICRO_D_116, BmsFrontEnd, VenusBoard

log = logging.getLogger(__name__)

MPPT_ADDR = 0x21
#: BMS v117/v118 current scale, A per AFE count: 54931.64 / 250 x 0.1 mA (180 mV
#: / 32768 per count over a 0.25 mOhm shunt) times a fixed 0.92 the D firmware
#: applies; the Jupiter C Plus BMS has the same board and formula without the 0.92
D_BMS_AMPS_PER_COUNT = 22.5 / 1024 * 0.92


class MpptNode(Node):
    """PV charger: up to four strings into the battery DC bus."""

    name = "mppt"

    #: word 26 of the status block (Modbus 30036, 0.01 V)
    REF_WORD = 219
    #: (min, max) of the commanded PV power (None: not clamped)
    POWER_CLAMP = None
    #: the PV limit command's voltage field is the charge voltage limit (D) or not (A)
    CMD_VOLTAGE_IS_CV = True

    def __init__(self, sim, pack, strings: Sequence[float] = (0, 0, 0, 0), string_v: float = 40.0,
                 efficiency: float = 0.97, slew_w_per_s: float = 300.0, version: int = 104):
        self.sim = sim
        self.pack = pack
        #: where a charge voltage limit comes from when the command has none: fn() -> volts
        self.cv_source = None
        self.available = [float(x) for x in strings] + [0.0] * (4 - len(strings))
        self.string_v = string_v
        self.efficiency = efficiency
        self.slew = slew_w_per_s
        self.version = version
        # PV limit commanded by the control (W, None = no limit); the stage stays
        # off until the control's first command, like the real converter
        self.limit_w: Optional[float] = 0.0
        self.pv_w = [0.0] * 4                         # what each string actually delivers
        self.dc_power = 0.0                           # into the battery, W
        self.limit_frames: List[tuple] = []
        self._pending02: Optional[bytes] = None
        self.vmax: Optional[float] = None
        self.mode = 0
        self.polls = 0

    # -- CAN
    def deliver(self, f: Frame) -> None:
        ident = f.ident
        if not f.ext or (ident & 0xF000) != 0x2000 or (ident >> 8) & 0xF != 1 or (ident >> 16) & 0xFF == MPPT_ADDR:
            return
        func, dest, idx = ident & 0xFF, (ident >> 16) & 0xFF, ident >> 24
        self.sim.at(f.t + 0.0005, lambda: self._command(func, dest, idx, f.data))

    def _send(self, func: int, dest: int, payload: bytes) -> None:
        t = self.sim.now
        n = max(1, (len(payload) + 7) // 8)
        for i in range(n):
            chunk = payload[8 * i: 8 * i + 8]
            ident = ((i + 1) << 24) | (MPPT_ADDR << 16) | (dest << 8) | func
            self.sim.at(t + 0.001 * i, lambda ident=ident, chunk=chunk: self.bus.send(self, ident, chunk, True, t=self.sim.now))

    def _command(self, func: int, dest: int, idx: int, data: bytes) -> None:
        if func == 0x03:
            self.polls += 1
            self._send(func, dest, self.block())
        elif func == 0xCB:
            self._version_reply(dest)
        elif func == 0x02:
            self.limit_frames.append((round(self.sim.now, 3), idx, data.hex()))
            del self.limit_frames[:-50]
            if idx == 0:
                self._pending02 = data
            elif self._pending02 is not None:
                self._limit(self._pending02, data)
                self._pending02 = None

    def _version_reply(self, dest: int) -> None:
        self._send(0xCB, dest, struct.pack("<HHHH", 0x0E, self.version, 0xFFFF, 1))

    def _limit(self, head: bytes, body: bytes) -> None:
        """PV limit (Grid_Export_Power_Limiter -> Inverter_CAN_PowerCmd_Send).

        head: u32 1 = limit active; body: u16 mode, u16 PV power limit (W),
        u16 charge voltage limit (0.1 V), two further bytes (not decoded)."""
        on = struct.unpack_from("<I", head.ljust(4, b"\0"))[0]
        mode, lim, vmax = struct.unpack_from("<HHH", body.ljust(8, b"\0"))
        if self.POWER_CLAMP and lim:
            lim = min(max(lim, self.POWER_CLAMP[0]), self.POWER_CLAMP[1])
        self.limit_w = float(lim) if on == 1 else None
        if self.CMD_VOLTAGE_IS_CV:
            self.vmax = vmax / 10.0 if vmax else None
        self.mode = mode

    # -- data
    def block(self) -> bytes:
        w = [0] * 28
        for k in range(4):
            p = self.pv_w[k]
            v = self.string_v if self.available[k] > 0 else 0.0
            w[4 + 3 * k] = int(v * 10)
            w[5 + 3 * k] = int(p / v * 10) if v else 0
            w[6 + 3 * k] = int(p * 10)
        vb = self.pack.voltage
        w[24] = int(vb * 10)
        w[25] = int(self.dc_power / vb * 10) & 0xFFFF if vb else 0
        w[26] = self.REF_WORD
        return struct.pack("<28H", *[x & 0xFFFF for x in w])

    # -- power
    def set_pv(self, strings: Sequence[float]) -> None:
        self.available = [float(x) for x in strings] + [0.0] * (4 - len(strings))

    def step(self, now: float, dt: float) -> None:
        avail = sum(self.available)
        target = avail if self.limit_w is None else min(avail, max(0.0, self.limit_w))
        vmax = self.vmax or (self.cv_source() if self.cv_source else None)
        if vmax:
            # constant-voltage phase: back off over the last 0.5 V below the limit
            headroom = (vmax - self.pack.voltage) / 0.5
            target = min(target, max(0.0, headroom) * avail)
        cur = sum(self.pv_w)
        step = self.slew * dt
        new = cur + max(-step, min(step, target - cur))
        self.pv_w = [new * a / avail for a in self.available] if avail > 0 else [0.0] * 4
        self.dc_power = new * self.efficiency


#: BMS EEPROM byte that keeps a pack from claiming the master role at boot (1 = slave pack)
D_BMS_SLAVE_FLAG = 0x202


class DBmsFrontEnd(BmsFrontEnd):
    """Venus D BMS board: KA49503 front end plus the 24C512 EEPROM (PB10 SCL,
    PB11 SDA).  A blank EEPROM makes the pack claim the master role at boot;
    the packs behind it carry ``1`` at :data:`D_BMS_SLAVE_FLAG`."""

    def __init__(self, emu, pack, amps_per_count: float, master: bool = True):
        super().__init__(emu, pack, amps_per_count)
        gb = emu.machine.bus.get("GPIOB")
        self.i2c = GpioI2cBus(gb, 10, gb, 11)
        self.eeprom = I2cEeprom(0x50, 65536, 2, 128)
        if not master:
            self.eeprom.mem[D_BMS_SLAVE_FLAG] = 1
        self.i2c.devices[0x50] = self.eeprom


class VenusD(VenusBoard):
    """Venus D: Control 147-151, BMS 117/118 (2.56 kWh pack), Micro 115/116 (modelled), and the
    MPPT: its firmware (102.5/104, ``mppt``) on a model of its board, or a model."""

    MICRO = MICRO_D_116
    BMS_AMPS_PER_COUNT = D_BMS_AMPS_PER_COUNT
    CAPACITY_AH = 50.0
    MAX_PACKS = 9

    def __init__(self, control, bms, *, pv: Sequence[float] = (0, 0, 0, 0), mppt=None,
                 mppt_kwargs: Optional[dict] = None, **kw):
        """``mppt``: the MPPT board's firmware image, run instead of the model."""
        super().__init__(control, bms, **kw)
        if mppt is not None:
            from .mppt import firmware_mppt
            self.mppt = firmware_mppt(self.sim, self.bus, mppt, self.pack, pv, parallel=self.parallel,
                                      kwargs=mppt_kwargs)
        else:
            self.mppt = self.bus.attach(MpptNode(self.sim, self.pack, pv))
        self.dc_sources.append(self.mppt)

    def make_bms_front(self, emu, pack, index: int = 0):
        return DBmsFrontEnd(emu, pack, self.BMS_AMPS_PER_COUNT, master=index == 0)

    def status(self) -> dict:
        st = super().status()
        st["pv_w"] = round(sum(self.mppt.pv_w), 1)
        st["pv_limit_w"] = self.mppt.limit_w
        return st


__all__ = ["MpptNode", "VenusD", "D_BMS_AMPS_PER_COUNT", "AMPS_PER_COUNT"]
