"""Venus E 3.0: the real control and BMS firmware around a modelled power stage.

Three MCUs share the internal CAN bus (500 kbit/s, 29-bit ids):

========  =====================  ======================================
node      MCU                    here
========  =====================  ======================================
control   N32G457 (FC41D, CT)    real firmware (``venus3`` profile)
BMS       N32G45x + KA495XX AFE  real firmware, AFE modelled from a
                                 :class:`~.battery.LfpPack`
inverter  STM32G474 ("micro")    :class:`VenusMicro`, a Python model of
                                 its CAN protocol and power stage
========  =====================  ======================================

The control firmware runs the user-visible logic: Wi-Fi/MQTT/BLE, the CT
meter poll and self-consumption steering, schedules, limits.  The BMS
firmware computes SOC, limits and protections from the cell voltages,
current and temperatures the AFE model reports.  The inverter model answers
the control's commands with the frames the real micro firmware sends (layout
captured from Micro v119 running in this emulator) and moves power between
grid and battery with configurable dynamics.

Protocol facts used here (Micro v119 / BMS v115 / Control v151):

* control -> inverter: id ``0x00014000 | node<<8 | func`` (class 4)
* inverter replies: ``index<<24 | 0x41<<16 | dest<<8 | func``, dest = request
  id bits 16-23; payloads longer than 8 bytes go out as frames 1..n, 1 ms apart
* telemetry (func 0x10) is 56 bytes; the BMS sniffs frame 4 to integrate its
  reported SOC (battery power * 10000 / battery voltage*10 -> mA)
* control -> BMS: ``0x1801FFAA`` poll; BMS answers 0x1801..0x1804AA01 and a
  per-pack burst
"""
from __future__ import annotations

import logging
import struct
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

from ..periph.ka495xx import Ka495xx
from .battery import LfpPack, _interp
from .canbus import CanBus, Frame, Node
from .cosim import CoSim, EmulatorCanNode

log = logging.getLogger(__name__)

MICRO_ADDR = 0x41


# --------------------------------------------------------------------------- inverter model
@dataclass
class PowerStage:
    """Dynamics of the inverter's power stage (estimates; see README)."""

    slew_w_per_s: float = 1000.0      # how fast AC power follows the reference
    start_delay_s: float = 3.0        # grid-connect sequence when leaving standby
    idle_disconnect_s: float = 60.0   # stays grid-connected this long at 0 W
    deadband_w: float = 5.0           # references below this are treated as 0
    standby_w: float = 6.0            # drawn from the battery while grid-connected
    loss_fixed_w: float = 12.0        # conversion losses: fixed + k*P^2
    loss_quadratic: float = 1.0e-5
    meter_tau_s: float = 0.2          # filter of the reported AC/battery power
    grid_v: float = 230.0
    grid_hz: float = 50.0
    ambient_c: float = 25.0


@dataclass
class MicroSpec:
    """What differs between the inverter boards (captured from their firmware)."""

    name: str
    version: int
    telemetry_len: int
    default_limit_w: int
    unsupported: tuple = ()
    model_code: int = 0x0D
    #: CAN 0x03/0x04 accept limits below this (W)
    limit_accept_below: int = 0x9C5
    #: hard cap of the AC power reference (W)
    max_power_w: float = 3000.0
    #: charge limit from the BMS current limit: I x V x this factor
    charge_bms_factor: float = 1.0
    #: telemetry byte 0: "run" (1 running / 0 off) or "a" (A micro inv_state 0/1/2/3)
    state_style: str = "run"


#: Venus E 3.0 inverter, Micro v119 (STM32G474)
MICRO_E3_119 = MicroSpec("VNSE3-0 Micro 119", 0x77, 56, 3000)
#: Venus D inverter, Micro v116: no 0x08/0x0A/0x14/0x17, 48-byte telemetry, 2500 W defaults
MICRO_D_116 = MicroSpec("VNSD-0 Micro 116", 0x74, 48, 2500, unsupported=(0x08, 0x0A, 0x14, 0x17))

# delay of the empty acknowledgement frame after a command, as measured on the
# real firmware (the longer ones write EEPROM first)
ACK_DELAY = {0x02: 0.0075, 0x03: 0.0145, 0x04: 0.0145, 0x08: 0.0075, 0x0A: 0.0075}


class VenusMicro(Node):
    """CAN behaviour and power stage of the Venus E 3.0 / D inverter board.

    Replies follow the real firmware: data replies are frame-indexed
    (``index<<24 | 0x41<<16 | dest<<8 | func``), commands without data are
    acknowledged by an empty frame with index 0."""

    name = "micro"

    def __init__(self, sim: CoSim, pack: LfpPack, stage: Optional[PowerStage] = None,
                 spec: MicroSpec = MICRO_E3_119, hw_variant: int = 0xFFFF):
        self.sim = sim
        self.pack = pack
        self.stage = stage or PowerStage()
        self.spec = spec
        self.version = spec.version
        self.hw_variant = hw_variant
        # configuration (EEPROM of the real board; factory defaults)
        self.work_mode = 0
        self.standby_latch = 0
        self.backup_flag = 0
        self.battery_mode = 1
        self.charge_limit_w = spec.default_limit_w
        self.discharge_limit_w = spec.default_limit_w
        self.max_soc = 100
        self.cutoff_soc = 12
        self.parallel_mode = 0
        self.ems_seq = 0
        # BMS data sniffed from the bus
        self.bms_seen = -1e9
        self.bms_v = 0.0
        self.bms_soc = 50.0
        self.bms_chg_i = 0.0
        self.bms_dis_i = 0.0
        self.bms_chg_v = 0.0
        self.bms_i = 0.0
        self.bms_flags = 0
        self.bms_sleep = 0
        self.bms_protect = 0
        # power stage state
        self.reference = 0.0          # clamped setpoint, + = discharge to grid
        self.ac_power = 0.0           # actual AC power, + = export
        self.dc_power = 0.0           # battery power, + = charging
        self.ac_meas = 0.0
        self.dc_meas = 0.0
        self.connected = False
        self._connect_at: Optional[float] = None
        self._idle_since = 0.0
        self.low_soc_stop = False
        self._low_soc_ticks = 0
        self.err1 = 0
        self.err2 = 0
        self.warn1 = 0
        self.temp_llc = self.stage.ambient_c
        self.temp_inv = self.stage.ambient_c
        self.charge_wh = 0.0
        self.discharge_wh = 0.0
        self.reboot_until = 0.0
        self.log: List[tuple] = []
        self.on_reference: List[Callable[[float, float], None]] = []

    # ---------------------------------------------------------------- CAN
    def deliver(self, f: Frame) -> None:
        if not f.ext:
            return
        ident = f.ident
        if (ident & 0x1FF0FF00) == 0x1800AA00:
            self._bms_frame(ident, f.data)
            return
        if (ident & 0xF000) != 0x4000 or ((ident >> 8) & 0xF) not in (0, 1) or (ident >> 16) & 0xFF == MICRO_ADDR:
            return
        if f.t < self.reboot_until:
            return
        func = ident & 0xFF
        dest = (ident >> 16) & 0xFF
        self.sim.at(f.t + 0.0005, lambda: self._command(func, dest, f.data))

    def _ack(self, func: int, dest: int) -> None:
        ident = (MICRO_ADDR << 16) | (dest << 8) | func
        delay = ACK_DELAY.get(func, 0.0)
        if delay:
            self.sim.at(self.sim.now + delay, lambda: self.bus.send(self, ident, b"", True, t=self.sim.now))
        else:
            self.bus.send(self, ident, b"", True, t=self.sim.now)

    def _send(self, func: int, dest: int, payload: bytes) -> None:
        t = self.sim.now
        if len(payload) <= 8:
            self.bus.send(self, (1 << 24) | (MICRO_ADDR << 16) | (dest << 8) | func, payload, True, t=t)
            return
        n = (len(payload) + 7) // 8
        for i in range(n):
            chunk = payload[8 * i: 8 * i + 8]
            ident = ((i + 1) << 24) | (MICRO_ADDR << 16) | (dest << 8) | func
            self.sim.at(t + 0.001 * i, lambda ident=ident, chunk=chunk: self.bus.send(self, ident, chunk, True, t=self.sim.now))

    def _command(self, func: int, dest: int, data: bytes) -> None:
        d = data + bytes(8)
        u32 = struct.unpack_from("<i", d)[0]
        self.log.append((self.sim.now, func, data.hex()))
        if func in self.spec.unsupported:
            return
        if func == 0x01:
            if self.work_mode == 0:
                self.reference = self.clamp(float(u32))
                for cb in self.on_reference:
                    cb(float(u32), self.reference)
                echo = int(self.reference)
            else:
                echo = u32
            self._send(func, dest, struct.pack("<i", echo))
        elif func == 0x02:
            if self.work_mode == 0:
                self.backup_flag = d[0]
            self._ack(func, dest)
        elif func in (0x03, 0x04):
            if u32 < self.spec.limit_accept_below:
                if func == 0x03:
                    self.discharge_limit_w = u32
                else:
                    self.charge_limit_w = u32
            self._ack(func, dest)
        elif func == 0x05:   # save statistics, watchdog reboot of the micro
            self._ack(func, dest)
            self._reboot()
        elif func == 0x06:
            self.standby_latch = 1
            self._ack(func, dest)
        elif func == 0x07:
            self.ems_seq = (d[0] + 1) & 0xFF
            self._ack(func, dest)
        elif func == 0x08:
            if self.work_mode == 0 and d[0] <= 2:
                self.parallel_mode = d[0]
            self._ack(func, dest)
        elif func == 0x0A:
            if self.work_mode == 0 and 6 <= d[0] <= 79:
                self.cutoff_soc = d[0]
            self._ack(func, dest)
        elif func == 0x10:
            self._send(func, dest, self.telemetry())
        elif func == 0x11:
            self._send(func, dest, struct.pack("<BBBBHH", self.work_mode, self.standby_latch, self.battery_mode, 0,
                                               self.charge_limit_w, self.discharge_limit_w))
        elif func == 0x12:
            self._send(func, dest, bytes(6))
        elif func == 0x13:
            self._send(func, dest, struct.pack("<HH", self.charge_limit_w, self.discharge_limit_w))
        elif func == 0x14:
            self._send(func, dest, struct.pack("<HHBB", self.charge_limit_w, self.discharge_limit_w,
                                               self.max_soc, self.cutoff_soc))
        elif func in (0x16, 0x17):
            pass
        elif func == 0x50:
            if d[0] == 0:
                self.reference = 0.0
        elif func == 0x57:
            self.backup_flag = int(self.work_mode == 1 and d[0] == 1)
            self._ack(func, dest)
        elif func == 0x58:
            if self.work_mode == 1 and -2500 <= u32 <= 2500:
                self.reference = self.clamp(float(u32))
            self._send(func, dest, data)
        elif func == 0xC1:
            self.sim.at(self.sim.now + 0.009, lambda: self._send(func, dest, struct.pack("<I", 1)))
        elif func == 0xC2:
            self._ack(func, dest)
        elif func == 0xCB:
            self._send(func, dest, struct.pack("<HHHH", self.spec.model_code, self.version, self.hw_variant, 1))

    def _reboot(self) -> None:
        self.reboot_until = self.sim.now + 1.5
        self.reference = 0.0
        self.ac_power = 0.0
        self.connected = False
        self._connect_at = None

    def _bms_frame(self, ident: int, data: bytes) -> None:
        d = data + bytes(8)
        pf = (ident & 0xFFFFF) >> 16
        if ident & 0xFF != 0x01:
            return
        if pf == 1:
            v, i, _t, soc = struct.unpack_from("<HhhH", d)
            self.bms_v = v / 100.0
            self.bms_i = i / 10.0
            self.bms_soc = soc / 10.0
            self.bms_seen = self.sim.now
        elif pf == 2:
            self.bms_sleep = d[7]
        elif pf == 3:
            cv, ci, di = struct.unpack_from("<HHH", d)
            self.bms_chg_v = cv / 10.0
            self.bms_chg_i = ci / 10.0
            self.bms_dis_i = di / 10.0
            self.bms_flags = d[6]
        elif pf == 4:
            self.bms_protect = struct.unpack_from("<I", d)[0]

    # ---------------------------------------------------------------- limits
    def limits(self):
        """(max discharge W >= 0, max charge W <= 0) like Ctrl_SetPowerReferenceClamped."""
        v = self.pack.voltage
        cap = self.spec.max_power_w
        dis = min(self.discharge_limit_w, cap, self.bms_dis_i * v) if self.bms_flags & 2 else 0.0
        chg = min(self.charge_limit_w, cap, self.bms_chg_i * v * self.spec.charge_bms_factor) if self.bms_flags & 1 else 0.0
        return max(0.0, dis), -max(0.0, chg)

    def clamp(self, req: float) -> float:
        if self.low_soc_stop and req > 0:
            return self.reference
        hi, lo = self.limits()
        if req > 0:
            return min(req, hi)
        if req < 0:
            return max(req, lo)
        return 0.0

    def running(self) -> bool:
        return (self.sim.now >= self.reboot_until and not self.standby_latch and not self.bms_sleep
                and self.err1 == 0 and self.err2 == 0)

    # ---------------------------------------------------------------- dynamics
    def step(self, now: float, dt: float) -> None:
        st = self.stage
        bms_ok = now - self.bms_seen < 10.0
        self.err1 = 0 if bms_ok or now < 12.0 else (1 << 20)
        if self.pack.soc * 100 < 6:
            self._low_soc_ticks += 1
            if self._low_soc_ticks > 15000:
                self.low_soc_stop = True
        elif self.low_soc_stop and self.pack.soc * 100 >= self.cutoff_soc - 1:
            self.low_soc_stop = False
            self._low_soc_ticks = 0
        target = self.reference if self.running() else 0.0
        hi, lo = self.limits()
        target = min(hi, max(lo, target))
        if abs(target) < st.deadband_w:
            target = 0.0
        # grid connection sequence
        if target != 0.0 and not self.connected:
            if self._connect_at is None:
                self._connect_at = now + st.start_delay_s
            if now >= self._connect_at:
                self.connected = True
                self._connect_at = None
        elif target == 0.0:
            self._connect_at = None
            if self.connected and abs(self.ac_power) < 1:
                if now - self._idle_since > st.idle_disconnect_s:
                    self.connected = False
            else:
                self._idle_since = now
        if not self.running():
            self.connected = False
        if target != 0.0:
            self._idle_since = now
        goal = target if self.connected else 0.0
        step = st.slew_w_per_s * dt
        self.ac_power += max(-step, min(step, goal - self.ac_power))
        # battery side: losses always come out of the battery
        losses = (st.loss_fixed_w + st.loss_quadratic * self.ac_power ** 2 + st.standby_w) if self.connected else 0.0
        self.dc_power = -self.ac_power - losses
        if self.ac_power > 0:
            self.discharge_wh += self.ac_power * dt / 3600
        else:
            self.charge_wh -= self.ac_power * dt / 3600
        a = min(1.0, dt / st.meter_tau_s) if st.meter_tau_s > 0 else 1.0
        self.ac_meas += a * (self.ac_power - self.ac_meas)
        self.dc_meas += a * (self.dc_power - self.dc_meas)
        tau = 300.0
        heat = losses * 0.02
        self.temp_llc += dt * (st.ambient_c + heat - self.temp_llc) / tau
        self.temp_inv += dt * (st.ambient_c + heat * 0.8 - self.temp_inv) / tau
        self.warn1 = 0 if self.connected or self.ac_power == 0 else 1

    # ---------------------------------------------------------------- telemetry
    def telemetry(self) -> bytes:
        st = self.stage
        b = bytearray(56)
        run = self.running()
        if self.spec.state_style == "a":
            b[0] = 0 if not run else 3 if self.ac_power > 10 else 2 if self.ac_power < -10 else 1
            b[2] = int(bool(self.bms_flags & 2))
        else:
            b[0] = 1 if run else 0
        b[1] = 0
        b[2] = 1                              # grid present
        b[3] = self.backup_flag
        struct.pack_into("<III", b, 4, 0 if self.connected else 1, self.err1, self.err2)
        struct.pack_into("<HHHH", b, 0x10, int(st.grid_v * 10), int(st.grid_hz * 10), 0, int(self.connected))
        hi, lo = self.limits()
        struct.pack_into("<hhhH", b, 0x18, int(round(self.ac_meas)), 0, int(round(self.dc_meas)),
                         int(self.pack.voltage * 10))
        struct.pack_into("<hhhh", b, 0x20, int(self.temp_llc * 10), int(self.temp_inv * 10), int(hi), int(lo))
        struct.pack_into("<IIII", b, 0x28, int(self.charge_wh / 1000), int(self.discharge_wh / 1000),
                         int(self.charge_wh), int(self.discharge_wh))
        return bytes(b[:self.spec.telemetry_len])


# --------------------------------------------------------------------------- BMS front end
# Cell temperature sensor reading vs. the ratio of the NTC register (0x3F..0x42)
# to the reference register 0x44, measured by sweeping the ratio against BMS
# v115 and reading its CAN reports (degC).
NTC_RATIO_TO_C = [
    (0.09, 117.5), (0.12, 104.7), (0.15, 95.1), (0.18, 87.5), (0.21, 81.0), (0.24, 75.5), (0.27, 70.6),
    (0.30, 66.3), (0.33, 62.3), (0.36, 58.6), (0.39, 55.3), (0.42, 52.1), (0.45, 49.1), (0.48, 46.3),
    (0.51, 43.6), (0.54, 41.0), (0.57, 38.6), (0.60, 36.2), (0.63, 33.8), (0.66, 31.6), (0.69, 29.3),
    (0.72, 27.2), (0.75, 25.0), (0.78, 22.9), (0.81, 20.8), (0.84, 18.7), (0.87, 16.6), (0.90, 14.5),
    (0.93, 12.5), (0.96, 10.3), (0.99, 8.2), (1.00, 7.5), (1.06, 3.1), (1.12, -1.6), (1.18, -6.6),
    (1.24, -12.1), (1.30, -18.6), (1.36, -26.5),
]
_C_TO_RATIO = sorted((c, r) for r, c in NTC_RATIO_TO_C)
NTC_REF = 0x4000
CELL_REGS = [0x28 + i for i in range(14)] + [0x37, 0x38]   # AFE channel 14 (0x36) unused with 16 cells
NTC_REGS = [0x3F, 0x40, 0x41, 0x42]
CURRENT_REG = 0x48
AMPS_PER_COUNT = 0.023725                                     # + = charge


class CodingLine:
    """The pack-coding line between stacked packs (Venus D and A): PB12 of a
    pack's BMS drives, through an inverting stage, PB13 of the next pack's.

    A pack that claims the master role pulses its output and then holds it;
    the next pack sees a short low pulse and then a held low on its input, and
    takes the address the master sends next on CAN (class 3, function 3).
    Once addressed it codes the pack behind it the same way."""

    CHAIN_OUT = ("GPIOB", 12)
    CHAIN_IN = ("GPIOB", 13)

    def chain_out(self) -> int:
        port, pin = self.CHAIN_OUT
        return self.emu.machine.bus.get(port).pin_out(pin)

    def set_chain_in(self, driven: int) -> None:
        """``driven``: the previous pack's output (high pulls this input low)."""
        port, pin = self.CHAIN_IN
        g = self.emu.machine.bus.get(port)
        if g.inputs.get(pin) != 1 - driven:
            g.set_input(pin, 1 - driven)


class BmsFrontEnd(CodingLine):
    """Feeds a :class:`Ka495xx` from the battery model and reads the MOSFET gates."""

    def __init__(self, emu, pack: LfpPack, amps_per_count: float = AMPS_PER_COUNT):
        self.emu = emu
        self.pack = pack
        self.amps_per_count = amps_per_count
        m = emu.machine
        self.afe = Ka495xx(cs=(m.bus.get("GPIOA"), 4))
        self.afe.measure = self._measure
        m.bus.get("SPI1").device = self.afe
        m.bus.get("GPIOA").set_input(6, 1)          # AFE ready line
        self.cell_offsets_mv: List[float] = [0.0] * 16

    def _measure(self, chip: Ka495xx) -> None:
        p = self.pack
        for reg, v, off in zip(CELL_REGS, p.cell_voltages, self.cell_offsets_mv):
            chip.regs[reg] = max(0, min(0xFFFF, int(round((v * 1000 + off) * 16384 / 5000))))
        ratio = _interp(_C_TO_RATIO, p.temp_c)
        for reg in NTC_REGS:
            chip.regs[reg] = int(ratio * NTC_REF)
        for reg in (0x3E, 0x43, 0x44, 0x49, 0x4A, 0x4B):
            chip.regs[reg] = NTC_REF
        chip.regs[CURRENT_REG] = int(round(p.current_a / self.amps_per_count)) & 0xFFFF

    def mos(self) -> Dict[str, bool]:
        m = self.emu.machine
        c = m.bus.get("GPIOC").regs.get(0x0C, 0)
        b = m.bus.get("GPIOB").regs.get(0x0C, 0)
        return {"charge": bool(c >> 5 & 1), "discharge": bool(b & 1), "precharge": bool(b >> 2 & 1)}



class PackSnapshot:
    """What the BMS front end needs from the pack, copied into the BMS process."""

    def __init__(self, cell_voltages=(), temp_c=25.0, current_a=0.0):
        self.cell_voltages = list(cell_voltages) or [3.3] * 16
        self.temp_c = temp_c
        self.current_a = current_a


class _BmsSync:
    def __init__(self, front: "BmsFrontEnd"):
        self.front = front

    def apply(self, state) -> None:
        p = self.front.pack
        p.cell_voltages, p.temp_c, p.current_a, chain_in = state
        if chain_in is not None:
            self.front.set_chain_in(chain_in)

    def report(self):
        return {**self.front.mos(), "chain_out": self.front.chain_out()}


def _build_bms(image, kwargs, make_front, index=0):
    from ..emulator import Emulator
    emu = Emulator(image, **kwargs)
    front = make_front(emu, PackSnapshot(), index)
    return emu, _BmsSync(front)


# --------------------------------------------------------------------------- assembly
class VenusBoard:
    """A Venus E 3.0 / D on one timeline: control and BMS firmware, inverter
    model, battery pack, plus optional extra DC sources (the D's MPPT).

    ``control`` and ``bms`` are firmware images (paths or bytes); keyword
    arguments for the control board's :class:`~marsemu.emulator.Emulator`
    (network, state_dir, ...) go in ``control_kwargs``.

    Stacked packs (Venus D and A, ``packs=[...]``): every pack has its own BMS
    board running the same image with its own MCU UID, all on the CAN bus, and
    the packs are chained by the coding line (:class:`CodingLine`).  The first
    pack claims the master role, codes the others (addresses 2, 3, ...) and
    reports the whole stack to the control; the DC power is shared among the
    packs whose MOSFETs for that direction are on.
    """

    #: inverter board, BMS current scale (A per AFE count) and pack capacity
    MICRO: MicroSpec = MICRO_E3_119
    BMS_AMPS_PER_COUNT = AMPS_PER_COUNT
    CAPACITY_AH = 100.0
    CELLS = 16
    #: packs the BMS firmware manages in one stack
    MAX_PACKS = 1

    def __init__(self, control, bms, *, pack: Optional[LfpPack] = None, stage: Optional[PowerStage] = None,
                 control_kwargs: Optional[dict] = None, bms_kwargs: Optional[dict] = None,
                 slice_s: float = 0.005, realtime: bool = False, micro: Optional[MicroSpec] = None,
                 parallel: bool = True, packs: Optional[Sequence[LfpPack]] = None):
        """``packs``: several battery packs, each with its own BMS (the same
        image, its own MCU UID) on the shared CAN bus; ``pack`` is the first."""
        micro = micro or self.MICRO
        import hashlib
        from ..emulator import Emulator

        self.sim = CoSim(slice_s=slice_s, realtime=realtime)
        self.bus = CanBus(500_000)
        self.bus._now = lambda: self.sim.now
        self.packs: List[LfpPack] = list(packs) if packs else [pack or LfpPack(cells_n=self.CELLS,
                                                                          capacity_ah=self.CAPACITY_AH)]
        self.pack = self.packs[0]
        if len(self.packs) > self.MAX_PACKS:
            raise ValueError(f"{type(self).__name__} takes at most {self.MAX_PACKS} pack(s)")
        #: other things that move DC power into (+) or out of the pack: objects with ``dc_power``
        self.dc_sources: List = []
        self.control = Emulator(control, **(control_kwargs or {}))
        self.sim.add_emulator(self.control)
        self.control_can = self.bus.attach(EmulatorCanNode(self.control, "CAN1", "control"))
        bms_kwargs = dict(bms_kwargs or {})
        bms_kwargs.setdefault("profile", "aux")
        self.parallel = parallel
        if isinstance(bms, str):
            bms = open(bms, "rb").read()
        self.bms_cans: List = []
        self.fronts: List = []
        self.bms = None
        for k, p in enumerate(self.packs):
            kw = dict(bms_kwargs)
            if k:                                 # every BMS board has its own MCU (and UID)
                kw.setdefault("uid", hashlib.sha1(bms + bytes([k])).digest()[:12])
            name = "bms" if not k else f"bms{k + 1}"
            if parallel:
                # each BMS runs in its own process, concurrently with the control
                from .remote import RemoteMcu
                node = self.bus.attach(RemoteMcu(
                    name, _build_bms, (bms, kw, self.make_bms_front, k), "CAN1",
                    state=lambda p=p, k=k: (p.cell_voltages, p.temp_c, p.current_a, self._chain_in(k))))
                self.sim.add_remote(node)
                self.fronts.append(None)
            else:
                emu = Emulator(bms, **kw)
                self.fronts.append(self.make_bms_front(emu, p, k))
                self.sim.add_emulator(emu)
                node = self.bus.attach(EmulatorCanNode(emu, "CAN1", name))
                if not k:
                    self.bms = emu
            self.bms_cans.append(node)
        self.bms_can = self.bms_cans[0]
        self.front = self.fronts[0]
        if not parallel:
            # pack coding: the line from each pack to the next one in the stack
            for k in range(1, len(self.fronts)):
                prev, nxt = self.fronts[k - 1], self.fronts[k]
                port, pin = prev.CHAIN_OUT
                nxt.set_chain_in(prev.chain_out())
                prev.emu.machine.bus.get(port).watchers.append(
                    lambda p, lvl, pin=pin, nxt=nxt: p == pin and nxt.set_chain_in(lvl))
        self.micro = self.bus.attach(VenusMicro(self.sim, self.pack, stage, spec=micro))
        self.sim.tickers.append(self._tick)
        self._acc = 0.0

    def make_bms_front(self, emu, pack, index: int = 0):
        """The model of the BMS board around its MCU (front end, EEPROM, ...);
        ``index`` is the pack's place in the stack (0 = the master pack)."""
        return BmsFrontEnd(emu, pack, self.BMS_AMPS_PER_COUNT)

    def _tick(self, now: float, dt: float) -> None:
        self._acc += dt
        if self._acc >= 0.01:            # power stage at 100 Hz
            dt, self._acc = self._acc, 0.0
            self.micro.step(now, dt)
            for src in self.dc_sources:
                src.step(now, dt)
            dc = self.micro.dc_power + sum(src.dc_power for src in self.dc_sources)
            if len(self.packs) == 1:
                self.pack.step(dt, self.pack.current_for_power(dc))
                return
            # packs on the DC bus: the power goes through the packs whose relay/MOSFET for
            # that direction is closed (the BMS rotates one active pack at a time)
            want = "charge" if dc > 0 else "discharge"
            mos = self.mos_all()
            live = [p for p, m in zip(self.packs, mos) if m.get(want)] or self.packs
            total_ah = sum(p.capacity_ah for p in live)
            for p in self.packs:
                share = dc * p.capacity_ah / total_ah if p in live else 0.0
                p.step(dt, p.current_for_power(share))

    def run(self, seconds: float, until=None) -> None:
        self.sim.run(seconds, until)

    def close(self) -> None:
        """Stop the BMS process (when running in parallel) and save state."""
        self.sim.close()
        self.control.save_state()

    def mos(self) -> dict:
        return self.mos_all()[0]

    def mos_all(self) -> List[dict]:
        """MOSFET/relay state of every pack's BMS."""
        out = []
        for f, n in zip(self.fronts, self.bms_cans):
            r = f.mos() if f is not None else dict(n.report or {})
            r.pop("chain_out", None)
            out.append(r)
        return out

    def _chain_in(self, k: int) -> Optional[int]:
        """What drives pack ``k``'s coding input: the previous pack's output
        as of the last slice (None for the first pack: nothing connected)."""
        if k == 0:
            return None
        return (self.bms_cans[k - 1].report or {}).get("chain_out", 0)

    @property
    def time(self) -> float:
        return self.sim.now

    @property
    def ac_power(self) -> float:
        """Actual AC power, W, positive = feeding the house/grid (discharging)."""
        return self.micro.ac_power

    @property
    def soc(self) -> float:
        """State of charge of the whole stack (capacity-weighted)."""
        return sum(p.soc * p.capacity_ah for p in self.packs) / sum(p.capacity_ah for p in self.packs)

    def status(self) -> dict:
        mi = self.micro
        return {
            "t": round(self.sim.now, 3), "ac_w": round(mi.ac_power, 1), "ref_w": round(mi.reference, 1),
            "dc_w": round(mi.dc_power, 1), "soc": round(self.soc * 100, 2), "bms_soc": mi.bms_soc,
            "pack_v": round(self.pack.voltage, 2), "i_a": round(sum(p.current_a for p in self.packs), 2),
            "bms_i_a": mi.bms_i,
            "limits": [round(x) for x in mi.limits()], "connected": mi.connected, "mos": self.mos(),
            "temp_c": round(self.pack.temp_c, 1),
            **({"packs": [{"soc": round(p.soc * 100, 2), "pack_v": round(p.voltage, 2), "i_a": round(p.current_a, 2),
                           "mos": m} for p, m in zip(self.packs, self.mos_all())]} if len(self.packs) > 1 else {}),
        }


class VenusE3(VenusBoard):
    """Venus E 3.0: Control 144-151, BMS 106-115 (one 5.12 kWh pack), Micro 119."""

    MICRO = MICRO_E3_119
    BMS_AMPS_PER_COUNT = AMPS_PER_COUNT
    CAPACITY_AH = 100.0
