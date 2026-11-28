"""Jupiter C/E and C Plus: the real control firmware with inverter, MPPT and BMS models.

The Jupiter control board (N32G45x, FC41D) talks to three units over its CAN
bus (500 kbit/s).  The inverter firmware is not available, so the inverter is
modelled from what the control sends and parses (HMN-1 V142 and JPLS-8H V143,
mapped in this emulator by answering with numbered values and reading the
device's own ``cd=14`` / ``cd=1`` status reports); so is the MPPT, unless the
C Plus's MPPT firmware is given (JPLS-8H 213/217, a C2000 image: see
:mod:`marsemu.plant.mppt`).  The BMS is a model too, or for the C Plus the
real BMS firmware (JPLS-8H V37/V38, see :class:`CPlusBmsFrontEnd`):

* **inverter** - class 1 node 1 (``0x0001110x``), replies from ``0x11``
* **MPPT** - class 2 node 1 (``0x0001210x``), replies from ``0x21``
* **BMS** - the Jupiter C/E control asks with a standard id ``0x305``
  request, the C Plus control broadcasts ``0x1801FFAA`` (``00 00 58 00 ..``)
  every 0.45 s.

  - The real C Plus BMS (V37) answers the broadcast with extended frames
    ``0x18nnAApp`` (``nn`` the frame, ``pp`` the pack address, 1 = master):
    ``0x1801`` voltage (0.01 V) / current (0.1 A, + = charge) / - / SOC (%),
    ``0x1821`` charge voltage (0.1 V) / charge and discharge current limits
    (0.1 A), ``0x1822`` max/min cell (mV) and temperatures, ``0x1830``-
    ``0x1833`` 16 cell voltages, ``0x1840``/``0x1841`` temperatures, and
    ``0x1802``-``0x1804``, ``0x1820``, ``0x1823`` (not decoded).  It answers
    only once it has a pack address (EEPROM ``0x1F6``: ``00 AA`` and its
    complement for the master).
  - The model (:class:`PylonBms`) answers both requests with the frame set
    the Venus E 3.0 BMS sends on its second port, which the C Plus control
    parses as well: ``0x351`` charge voltage / charge and discharge current
    limits, ``0x355`` SOC (%) / SOH / capacity / version, ``0x356`` voltage
    (0.01 V) / current (0.1 A, signed) / temperatures, ``0x359`` protection,
    ``0x35C`` flags (0xC0 = charge and discharge allowed), ``0x360``-
    ``0x363`` 16 cell voltages, ``0x36A``/``0x36B`` temperatures.

The control copies the battery voltage and current of both the MPPT (words
24/25) and the BMS into its ``cd=14`` report unchanged, as
``mppt:...b_vol,b_cur`` and ``bms:...b_vol,b_cur``.

Commands to the inverter and MPPT (frame index in id bits 24-28):

* ``0x01`` start: 5 bytes, byte 4 = 1
* ``0x02`` power command: a 4-byte head (u32 1) and a body frame; inverter
  body ``<u16 mode, i16 power W, u16, u16>`` (+ = discharge; the phase
  detection probe is 800 W), MPPT body ``<u16 mode, u16 PV limit W, u16 charge
  voltage 0.1 V>``
* ``0x03`` status poll: the answer is a header frame (index 0, u16 number of
  data frames) followed by that many 8-byte frames, four words each:

  ========  ==================================================================
  inverter  4 frames: state, error, warning, grid V (0.1), grid I, pf, grid f
            (0.01 Hz), battery V (0.1), grid power W (+ export), temperature,
            then two u32 energy counters (day, month) at words 10/11 and 14/15
  MPPT      7 frames: state, error, temperature, warning, PV string k: V
            (0.1), I (0.1), P (0.1 W) at words 4+3k..6+3k, then battery V
            (0.1 V) and the current it delivers into the battery (0.1 A) at
            words 24/25, ``base_v`` and ``pe_v`` at words 26/27 (meaning not
            known; real units report about 220 and 165)
  ========  ==================================================================

* ``0xCB`` version: ``index 1`` frame, version in byte 4 (the model; the C
  Plus's MPPT firmware answers ``<u16 210, u16 version, 0xFFFF, 1>``)
* ``0x04``: restart (the C Plus control sends it to the MPPT about 11 s after
  boot; the MPPT firmware restarts through its watchdog)
"""
from __future__ import annotations

import struct
from typing import Optional, Sequence

from .battery import LfpPack
from .canbus import CanBus, Frame, Node
from .cosim import CoSim, EmulatorCanNode
from .venus3 import MicroSpec, PowerStage, VenusMicro
from .venusd import DBmsFrontEnd, MpptNode

INV_ADDR = 0x11
MPPT_ADDR = 0x21
#: the C Plus control's request to the BMS (extended id, every 0.45 s)
CPLUS_BMS_REQUEST = 0x1801FFAA
#: C Plus BMS V37 current scale: 54931.64 / 250 x 0.1 mA per AFE count, i.e.
#: 180 mV / 32768 per count over a 0.25 mOhm shunt (the Venus D BMS uses the same)
CPLUS_BMS_AMPS_PER_COUNT = 22.5 / 1024

#: the inverter: Jupiter limits (2500 W), no Venus-specific commands
JUPITER_INV = MicroSpec("Jupiter inverter", 106, 32, 2500, limit_accept_below=2501, max_power_w=2500.0)


def _send_block(node, func: int, dest: int, addr: int, words) -> None:
    """Header frame (index 0, frame count) + one 8-byte data frame per four words."""
    n = (len(words) + 3) // 4
    words = list(words) + [0] * (4 * n - len(words))
    payload = struct.pack(f"<{4 * n}H", *[int(w) & 0xFFFF for w in words])
    t = node.sim.now
    node.bus.send(node, (addr << 16) | (dest << 8) | func, struct.pack("<HHHH", n, 0, 0, 0), True, t=t)
    for i in range(n):
        ident = ((i + 1) << 24) | (addr << 16) | (dest << 8) | func
        chunk = payload[8 * i: 8 * i + 8]
        node.sim.at(t + 0.001 * (i + 1), lambda ident=ident, chunk=chunk: node.bus.send(node, ident, chunk, True, t=node.sim.now))


class JupiterInverter(VenusMicro):
    """Power stage of the Jupiter inverter; reuses the Venus power-stage model."""

    name = "inverter"

    def __init__(self, sim, pack, stage=None, version: int = 106):
        super().__init__(sim, pack, stage, spec=JUPITER_INV)
        self.version = version
        self.started = False
        self._head02 = None
        self.energy_day_wh = 0.0
        self.energy_month_wh = 0.0
        # the Pylontech-style BMS frames give the limits
        self.bms_flags = 0
        self.bms_seen = -1e9

    def deliver(self, f: Frame) -> None:
        ident = f.ident
        if not f.ext:
            self._bms_std(ident, f.data)
            return
        if (ident & 0xFF00FF00) == 0x1800AA00 and (ident & 0xFF) == 1:
            self._bms_ext(ident >> 16 & 0xFF, f.data)
            return
        if (ident & 0xF000) != 0x1000 or (ident >> 8) & 0xF != 1 or (ident >> 16) & 0xFF == INV_ADDR:
            return
        if f.t < self.reboot_until:
            return
        func, dest, idx = ident & 0xFF, (ident >> 16) & 0xFF, ident >> 24
        self.sim.at(f.t + 0.0005, lambda: self._jcommand(func, dest, idx, f.data))

    def _bms_std(self, ident: int, data: bytes) -> None:
        d = data + bytes(8)
        if ident == 0x351:
            cv, ci, di = struct.unpack_from("<HHH", d)
            self.bms_chg_v, self.bms_chg_i, self.bms_dis_i = cv / 10.0, ci / 10.0, di / 10.0
        elif ident == 0x355:
            self.bms_soc = float(struct.unpack_from("<H", d)[0])
        elif ident == 0x356:
            v, i = struct.unpack_from("<Hh", d)
            self.bms_v, self.bms_i, self.bms_seen = v / 100.0, i / 10.0, self.sim.now
        elif ident == 0x35C:
            f = d[0]
            self.bms_flags = (1 if f & 0x80 else 0) | (2 if f & 0x40 else 0)

    def _bms_ext(self, frame: int, data: bytes) -> None:
        """The C Plus BMS's own frames (master pack)."""
        d = data + bytes(8)
        if frame == 0x01:
            v, i, _, soc = struct.unpack_from("<HhHH", d)
            self.bms_v, self.bms_i, self.bms_soc, self.bms_seen = v / 100.0, i / 10.0, float(soc), self.sim.now
        elif frame == 0x21:
            cv, ci, di = struct.unpack_from("<HHH", d)
            self.bms_chg_v, self.bms_chg_i, self.bms_dis_i = cv / 10.0, ci / 10.0, di / 10.0
            self.bms_flags = (1 if ci else 0) | (2 if di else 0)

    def _jcommand(self, func: int, dest: int, idx: int, data: bytes) -> None:
        self.log.append((self.sim.now, func, data.hex()))
        if func == 0x01:
            self.started = len(data) > 4 and data[4] == 1
        elif func == 0x02:
            if idx == 0:
                self._head02 = data
            elif self._head02 is not None:
                on = struct.unpack_from("<I", self._head02.ljust(4, b"\0"))[0] == 1
                _mode, power = struct.unpack_from("<Hh", data.ljust(4, b"\0"))
                self.reference = self.clamp(float(power)) if on else 0.0
                for cb in self.on_reference:
                    cb(float(power), self.reference)
                self._head02 = None
        elif func == 0x03:
            _send_block(self, 0x03, dest, INV_ADDR, self.status_words())
        elif func == 0xCB:
            self.bus.send(self, (1 << 24) | (INV_ADDR << 16) | (dest << 8) | 0xCB,
                          bytes([0x0E, 0, 0x65, 0, self.version & 0xFF, 0, 1, 0]), True, t=self.sim.now)

    def running(self) -> bool:
        return self.started and super().running()

    def step(self, now: float, dt: float) -> None:
        super().step(now, dt)
        self.energy_day_wh += abs(self.ac_power) * dt / 3600
        self.energy_month_wh += abs(self.ac_power) * dt / 3600

    def status_words(self):
        st = self.stage
        grid_i = abs(self.ac_meas) / st.grid_v * 100 if st.grid_v else 0
        day, month = int(self.energy_day_wh), int(self.energy_month_wh)
        return [1 if self.running() else 0, self.err1 & 0xFFFF, 0, st.grid_v * 10, grid_i, 100, st.grid_hz * 100,
                self.pack.voltage * 10, int(round(self.ac_meas)), self.temp_inv * 10,
                day & 0xFFFF, day >> 16, 0, 0, month & 0xFFFF, month >> 16]


class JupiterMppt(MpptNode):
    """Jupiter PV charger: same physics as the Venus D MPPT, Jupiter framing."""

    name = "mppt"

    #: words 26/27 (``base_v``, ``pe_v`` in ``cd=14``); typical values of real units
    REF_WORD = 220
    PE_WORD = 165

    def deliver(self, f: Frame) -> None:
        ident = f.ident
        if not f.ext or (ident & 0xF000) != 0x2000 or (ident >> 8) & 0xF != 1 or (ident >> 16) & 0xFF == MPPT_ADDR:
            return
        func, dest, idx = ident & 0xFF, (ident >> 16) & 0xFF, ident >> 24
        self.sim.at(f.t + 0.0005, lambda: self._command(func, dest, idx, f.data))

    def _command(self, func: int, dest: int, idx: int, data: bytes) -> None:
        if func == 0x03:
            self.polls += 1
            _send_block(self, 0x03, dest, MPPT_ADDR, self.words())
        elif func == 0xCB:
            self.bus.send(self, (1 << 24) | (MPPT_ADDR << 16) | (dest << 8) | 0xCB,
                          bytes([0x0E, 0, 0x65, 0, self.version & 0xFF, 0, 1, 0]), True, t=self.sim.now)
        elif func == 0x02:
            super()._command(func, dest, idx, data)

    def words(self):
        """The 28-word status block (see the module docstring)."""
        w = list(struct.unpack("<28H", self.block()))
        w[0:4] = [1, 0, 300, 0]
        w[27] = self.PE_WORD
        return w


class PylonBms(Node):
    """Battery BMS with the Marstek/Pylontech-style frame set; answers the
    Jupiter C/E's ``0x305`` request and the C Plus's ``0x1801FFAA`` broadcast."""

    name = "bms"

    def __init__(self, sim, pack: LfpPack, capacity_wh: int = 5120, version: int = 209,
                 max_current_a: float = 50.0, charge_voltage: float = 57.1):
        self.sim = sim
        self.pack = pack
        self.capacity_wh = capacity_wh
        self.version = version
        self.max_current_a = max_current_a
        self.charge_voltage = charge_voltage
        self.full = False
        self.answers = 0

    def deliver(self, f: Frame) -> None:
        if (not f.ext and f.ident == 0x305) or (f.ext and f.ident == CPLUS_BMS_REQUEST):
            self.answers += 1
            self.sim.at(f.t + 0.002, self._answer)

    def limits(self):
        soc = self.pack.soc * 100
        vmax = max(self.pack.cell_voltages)
        vmin = min(self.pack.cell_voltages)
        if vmax >= 3.60:
            self.full = True
        elif soc < 95:
            self.full = False
        chg = 0.0 if self.full else self.max_current_a * (0.5 if soc > 95 else 1.0)
        dis = 0.0 if vmin <= 2.80 or soc <= 1 else self.max_current_a
        return chg, dis

    def _answer(self) -> None:
        p = self.pack
        chg, dis = self.limits()
        soc = int(round(p.soc * 100))
        cells = [int(round(v * 1000)) for v in p.cell_voltages][:16] + [0] * 16
        t10 = int(round(p.temp_c * 10))
        flags = (0x80 if chg > 0 else 0) | (0x40 if dis > 0 else 0)
        frames = [
            (0x351, struct.pack("<HHHH", int(self.charge_voltage * 10), int(chg * 10), int(dis * 10), 0)),
            (0x355, struct.pack("<HHHBB", soc, 100, self.capacity_wh, self.version & 0xFF, 0)),
            (0x356, struct.pack("<Hhhh", int(round(p.voltage * 100)), int(round(p.current_a * 10)), t10, t10)),
            (0x359, bytes(8)),
            (0x35C, bytes([flags, 0, 0, 0, 0, 0, 0, 0])),
        ]
        for k in range(4):
            frames.append((0x360 + k, struct.pack("<4H", *cells[4 * k: 4 * k + 4])))
        frames += [(0x36A, struct.pack("<4h", t10, t10, t10, t10)), (0x36B, struct.pack("<4h", t10, t10, 0, 0))]
        t = self.sim.now
        for n, (sid, d) in enumerate(frames):
            self.sim.at(t + 0.005 * n, lambda sid=sid, d=d: self.bus.send(self, sid, d, False, t=self.sim.now))


class CPlusBmsFrontEnd(DBmsFrontEnd):
    """The Jupiter C Plus BMS board around its MCU (V37): the Venus D board's
    KA49503 front end and 24C512 EEPROM, with the EEPROM holding the master
    pack address so that the BMS answers the control."""

    #: EEPROM: pack address 0x00AA (the master) and its complement
    ADDRESS = 0x1F6

    def __init__(self, emu, pack, amps_per_count: float = CPLUS_BMS_AMPS_PER_COUNT):
        super().__init__(emu, pack, amps_per_count)
        self.eeprom.mem[self.ADDRESS:self.ADDRESS + 4] = bytes([0x00, 0xAA, 0xFF, 0x55])


class Jupiter:
    """A Jupiter (C/E, C Plus) on one timeline: control firmware + inverter, MPPT
    and BMS models.  ``bms``: the C Plus BMS image, run instead of the BMS model;
    ``mppt``: the C Plus MPPT image, run instead of the MPPT model."""

    CAPACITY_AH = 100.0
    CELLS = 16

    def __init__(self, control, bms=None, *, pv: Sequence[float] = (0, 0, 0, 0), pack: Optional[LfpPack] = None,
                 stage: Optional[PowerStage] = None, control_kwargs: Optional[dict] = None,
                 bms_kwargs: Optional[dict] = None, slice_s: float = 0.005, realtime: bool = False,
                 mppt=None, mppt_kwargs: Optional[dict] = None, parallel: bool = True, **_ignored):
        from ..emulator import Emulator
        self.sim = CoSim(slice_s=slice_s, realtime=realtime)
        self.bus = CanBus(500_000)
        self.bus._now = lambda: self.sim.now
        self.pack = pack or LfpPack(cells_n=self.CELLS, capacity_ah=self.CAPACITY_AH)
        self.control = Emulator(control, **(control_kwargs or {}))
        self.sim.add_emulator(self.control)
        self.control_can = self.bus.attach(EmulatorCanNode(self.control, "CAN1", "control"))
        self.micro = self.bus.attach(JupiterInverter(self.sim, self.pack, stage))
        if mppt is not None:
            # the C Plus's MPPT board with its firmware (JPLS-8H 213/217), see plant/mppt.py
            from .mppt import firmware_mppt
            self.mppt = firmware_mppt(self.sim, self.bus, mppt, self.pack, pv, parallel=parallel,
                                      kwargs=mppt_kwargs)
        else:
            self.mppt = self.bus.attach(JupiterMppt(self.sim, self.pack, pv, version=206))
        self.bms = self.bms_emu = self.front = None
        if bms is None:
            self.bms = self.bus.attach(PylonBms(self.sim, self.pack))
        else:
            kw = {"profile": "aux", **(bms_kwargs or {})}
            self.bms_emu = Emulator(bms, **kw)
            self.front = CPlusBmsFrontEnd(self.bms_emu, self.pack)
            self.sim.add_emulator(self.bms_emu)
            self.bms_can = self.bus.attach(EmulatorCanNode(self.bms_emu, "CAN1", "bms"))
        self.mppt.cv_source = lambda: self.micro.bms_chg_v or None
        self.dc_sources = [self.mppt]
        self.sim.tickers.append(self._tick)
        self._acc = 0.0

    def _tick(self, now: float, dt: float) -> None:
        self._acc += dt
        if self._acc >= 0.01:
            dt, self._acc = self._acc, 0.0
            self.micro.step(now, dt)
            for s in self.dc_sources:
                s.step(now, dt)
            dc = self.micro.dc_power + sum(s.dc_power for s in self.dc_sources)
            self.pack.step(dt, self.pack.current_for_power(dc))

    def run(self, seconds: float, until=None) -> None:
        self.sim.run(seconds, until)

    def close(self) -> None:
        self.sim.close()
        self.control.save_state()
        if self.bms_emu:
            self.bms_emu.save_state()

    @property
    def time(self) -> float:
        return self.sim.now

    @property
    def ac_power(self) -> float:
        return self.micro.ac_power

    def status(self) -> dict:
        mi = self.micro
        return {
            "t": round(self.sim.now, 3), "ac_w": round(mi.ac_power, 1), "ref_w": round(mi.reference, 1),
            "dc_w": round(mi.dc_power, 1), "soc": round(self.pack.soc * 100, 2), "bms_soc": mi.bms_soc,
            "pack_v": round(self.pack.voltage, 2), "i_a": round(self.pack.current_a, 2), "bms_i_a": mi.bms_i,
            "limits": [round(x) for x in mi.limits()], "connected": mi.connected,
            "mos": self.front.mos() if self.front else {},
            "temp_c": round(self.pack.temp_c, 1), "pv_w": round(sum(self.mppt.pv_w), 1),
            "pv_limit_w": self.mppt.limit_w,
        }


__all__ = ["CPlusBmsFrontEnd", "Jupiter", "JupiterInverter", "JupiterMppt", "PylonBms",
           "CPLUS_BMS_AMPS_PER_COUNT"]
