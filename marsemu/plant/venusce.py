"""Venus C/E (HMG-50): the control MCU runs the power stage itself.

The control board (STM32G474, control firmware V146-V158) drives a
bidirectional inverter: an LLC converter between the 16-cell LFP pack and a
~430 V DC bus, and a full bridge between the bus and the grid.  The BMS
(N32G45x, BMS firmware 114-216, KA495XX front end) is a separate MCU that
answers the control's CAN poll (``0x305``, 500 kbit/s on FDCAN2) with
Pylontech-style frames (``0x351`` limits, ``0x355`` SOC, ``0x356`` voltage /
current, ``0x359`` alarms, ``0x35C`` flags, ``0x360``.. cells).  Here both run
their real firmware; :class:`PowerStage` stands in for the analog side.

What the control firmware does, from the V151 pseudo-C (addresses for V151,
the HRTIM/ADC/DMA set-up is identical in V154):

* HRTIM master, 30 kHz: its compare 1 is ADC trigger 1, which starts ADC1-4
  (DMA1 channels 1-4, circular).  ADC1's transfer-complete interrupt runs the
  30 kHz control loop from CCM RAM: SOGI-PLL on the grid voltage (sin/cos on
  the CORDIC coprocessor), SOGI P/Q, a resonant current controller.
* Inverter bridge: HRTIM timers A and B (30 kHz, up-down), compare 1/2 =
  c +- 680 with c = m x 45334 and m = 0.5 + 0.5 v_cmd / Vbus.
* LLC: timer D (~104 kHz, soft start ramps its period 5700 -> 6550) resets
  timers C (HV side), E and F (battery side); timer D's compare interrupt
  rewrites their compares (0xFFFF = off).
* Grid zero crossings: a comparator into TIM20 channel 1 (1 MHz, both
  edges) measures the grid frequency the PLL is checked against.
* Relays: PD3 grid relay (closed at a zero crossing once the grid is good),
  PA12 bus precharge bypass, PD7.
* ADC channels and scaling (``ref`` = ADC1 IN3, the mid-rail reference):

  =====  ====  =====================  =========================
  ADC    IN    signal                 scaling in the firmware
  =====  ====  =====================  =========================
  1      9     inverter-side voltage  (raw - ref) x 0.21606 V
  1      3     mid-rail reference
  1      8     grid voltage           (raw - ref) x 0.21606 V
  2      1     grid current           (raw - ref) x 0.018311 A
  2      4     off-grid current       (raw - ref) x -0.017 A
  3      3     battery voltage        raw x 0.019293 V
  3      1     DC bus voltage         raw x 0.15271 V
  3      2     battery current        (raw - ref) x 0.06 A
  4      4     off-grid voltage       (raw - ref) x 0.21606 V
  4      2/1   voltage/current offset references
  5      10-13 NTCs (inverter, heat sinks)
  =====  ====  =====================  =========================

The plant is an averaged model, one step per ADC trigger: the bridge puts
(2m - 1) x Vbus on an inductor into the grid while the grid relay is closed,
the bus is a capacitor between bridge and LLC, and the LLC couples the bus to
n x Vbat (its gain follows the soft-start period) through a small
resistance, or a large one while the precharge bypass is open.  Inductance,
capacitance, the LLC ratio and the losses are estimates.

Timer D's interrupt only rewrites the LLC compares; it runs at a fraction of
its rate (:attr:`PowerStage.llc_irq_divider`) because at 104 kHz it alone
makes the emulation several times slower than real time.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger(__name__)

# ADC scaling (V151)
V_PER_COUNT = 0.21606
I_GRID_PER_COUNT = 0.018311        # hardware variant 1 (PD2 high)
I_GRID_PER_COUNT_V0 = -0.017577     # hardware variant 0 (PD2 low)
VBUS_PER_COUNT = 0.15271
VBAT_PER_COUNT = 0.019293064
IBAT_PER_COUNT = 0.06
REF = 2048
TWO_PI = 2 * math.pi
SQRT2 = math.sqrt(2)
#: HRTIM inverter compare offset around the duty (CMP1 = c + 680, CMP2 = c - 680)
CMP_OFFSET = 680


@dataclass
class PowerStageSpec:
    grid_v: float = 230.0
    grid_hz: float = 50.0
    l_h: float = 1.5e-3            # bridge-to-grid inductance (estimate)
    r_ohm: float = 0.15            # inductor and bridge resistance
    c_bus_f: float = 1.0e-3        # DC bus capacitance (estimate)
    llc_ratio: float = 8.35        # bus / battery voltage at full LLC gain (firmware expects 8.2-8.5)
    llc_r_ohm: float = 0.6         # LLC output resistance referred to the bus
    precharge_r_ohm: float = 60.0  # while the precharge bypass (PA12) is open
    llc_efficiency: float = 0.96
    temp_c: float = 30.0
    #: model the bridge's dead time (the gap between CMP1 and CMP2): while both
    #: switches of a leg are off, the current flows through the diode that opposes it
    dead_time: bool = False
    #: board variant strap on PD2; the firmware picks its grid-current sensor scale from it
    variant: int = 0


class PowerStage:
    """The analog side of a Venus C/E control board (see the module doc)."""

    def __init__(self, emu, pack, spec: Optional[PowerStageSpec] = None, llc_irq_divider: int = 30):
        self.emu = emu
        self.m = m = emu.machine
        self.pack = pack
        self.spec = spec or PowerStageSpec()
        self.hrtim = m.hrtim
        self.adc = {n: m.bus.get(n) for n in ("ADC1", "ADC2", "ADC3", "ADC4", "ADC5")}
        self.gpio = {p: m.bus.get(f"GPIO{p}") for p in "ADE"}
        self.zero_cross = m.bus.get("TIM20")
        # state
        self.theta = 0.0
        self.i = 0.0              # bridge current, + = into the grid
        self.vbus = 0.0
        self.v_inv = 0.0
        self.p_ac_avg = 0.0       # grid power, + = into the grid (low-pass, ~1 cycle)
        self.p_bat_avg = 0.0      # battery power, + = into the battery
        self.bat_ah = 0.0         # charge moved since the last step_pack(), Ah (+ = into the battery)
        self.bat_s = 0.0          # ... over this much time (the control's own clock)
        self.samples = 0
        self._vbat = pack.voltage
        self._ibat = 0.0
        self.llc_irq_divider = llc_irq_divider
        # fault inputs (hardware trip latches on PE7/8/10/11): inactive
        ge = self.gpio["E"]
        for pin in (7, 8, 10, 11):
            ge.set_input(pin, 0)
        # PE12: the hardware protection latch's status (1 = no fault; PB3 resets it)
        ge.set_input(12, 1)
        # PD2: board variant strap, read once at boot
        self.gpio["D"].set_input(2, 1 if self.spec.variant else 0)
        self._gain_i = I_GRID_PER_COUNT if self.spec.variant else I_GRID_PER_COUNT_V0
        self._ntc_raw = 2048
        self.hrtim.units["D"].divider = llc_irq_divider
        # ADC1's DMA: the firmware does nothing on half transfer; skipping it halves
        # the interrupts of the 30 kHz loop
        m.bus.get("DMA1").ch[1].half_transfer = False
        # cached for the 30 kHz step
        self._hz = m.clock.hz
        self._gpio_a, self._gpio_d = self.gpio["A"], self.gpio["D"]
        self._a1, self._a2, self._a3 = (self.adc[n].values for n in ("ADC1", "ADC2", "ADC3"))
        # channels that only carry the mid-rail reference or the NTCs never change
        a4, a5 = self.adc["ADC4"].values, self.adc["ADC5"].values
        self._a1[3] = self._a2[4] = a4[4] = a4[2] = a4[1] = REF
        a5[10] = a5[12] = a5[13] = self._ntc_raw
        self.hrtim.adc_listeners.insert(0, self._on_trigger)
        self.hrtim.active = True

    # -- inputs from the firmware
    def relay_closed(self) -> bool:
        return bool(self.gpio["D"].pin_out(3))

    def bypass_closed(self) -> bool:
        return bool(self.gpio["A"].pin_out(12))

    def bridge_on(self) -> bool:
        return (self.hrtim.outputs & 0xF) == 0xF

    def llc_on(self) -> bool:
        u = self.hrtim.units
        if not self.hrtim.outputs & 0xF30:
            return False
        return u["E"].compare(1) != 0xFFFF or u["C"].compare(1) != 0xFFFF

    def llc_gain(self) -> float:
        per = self.hrtim.units["D"].period_ticks()
        return max(0.3, min(1.0, (per - 5000) / 1550.0)) if per else 0.0

    def duty(self) -> float:
        a = self.hrtim.units["A"]
        per = a.period_ticks()
        if not per:
            return 0.5
        c = (a.compare(1) + a.compare(2)) / 2.0
        return min(1.0, max(0.0, c / per))

    def dead_fraction(self) -> float:
        """Share of a PWM period each bridge leg spends with both switches off.
        The timers count up and down; the high-side output is set above CMP1 and
        the low side's below CMP2, so the leg is open while the counter is
        between them, on the way up and on the way down."""
        a = self.hrtim.units["A"]
        per = a.period_ticks()
        return max(0, a.compare(1) - a.compare(2)) / per if per else 0.0

    # -- one sample
    def _on_trigger(self, trg: int) -> None:
        # Runs at 30 kHz: locals instead of attribute and method lookups.
        if trg != 1:
            return
        s = self.spec
        hrtim = self.hrtim
        units = hrtim.units
        master = units["M"]
        ts = master.period_cycles() / self._hz or 1 / 30000.0
        self.samples += 1
        if self.samples % 300 == 1:       # battery voltage changes slowly
            self._vbat = self.pack.voltage
        vbat = self._vbat
        old = self.theta
        theta = old + TWO_PI * s.grid_hz * ts
        if theta >= math.pi and old < math.pi or theta >= TWO_PI:
            self.zero_cross.capture(1)            # zero-crossing detector -> TIM20 CH1 (both edges)
        theta %= TWO_PI
        self.theta = theta
        v_grid = SQRT2 * s.grid_v * math.sin(theta)

        # bridge and inductor
        vbus = self.vbus
        i = self.i
        bridge = (hrtim.outputs & 0xF) == 0xF and vbus > 1.0
        if bridge:
            a = units["A"]
            per = a.period_ticks()
            c1, c2 = a.compare(1), a.compare(2)
            duty = min(1.0, max(0.0, (c1 + c2) / 2.0 / per)) if per else 0.5
            v_inv = (2 * duty - 1) * vbus
            if s.dead_time and i and per:
                # each leg's open time conducts through the diode against the current:
                # half of it is lost relative to the midpoint duty, per leg
                v_inv -= math.copysign(max(0, c1 - c2) / per * vbus, i)
        else:
            v_inv = 0.0
        self.v_inv = v_inv
        relay = self._gpio_d.pin_out(3)
        if relay and bridge:
            i += (v_inv - v_grid - s.r_ohm * i) * ts / s.l_h
        else:
            i = 0.0
        self.i = i
        i_dc = (v_inv * i / vbus) if bridge else 0.0     # out of the bus

        # LLC: couples the bus to n x Vbat
        i_llc = 0.0
        if self.llc_on():
            target = s.llc_ratio * self.llc_gain() * vbat
            r = s.llc_r_ohm if self._gpio_a.pin_out(12) else s.precharge_r_ohm
            i_llc = (target - vbus) / r
        vbus = max(0.0, vbus + (i_llc - i_dc) * ts / s.c_bus_f)
        self.vbus = vbus
        p_llc = i_llc * vbus                                        # into the bus
        p_bat = -(p_llc / s.llc_efficiency if p_llc > 0 else p_llc * s.llc_efficiency)
        ibat = p_bat / vbat if vbat > 1 else 0.0                    # + = into the battery
        self._ibat = ibat
        self.bat_ah += ibat * ts / 3600.0
        self.bat_s += ts
        k = ts / 0.02
        self.p_ac_avg += (v_grid * i - self.p_ac_avg) * k
        self.p_bat_avg += (p_bat - self.p_bat_avg) * k

        # ADC inputs
        v_out = v_grid if relay else v_inv
        a1, a2, a3 = self._a1, self._a2, self._a3
        a1[9] = _clip(REF + v_out / V_PER_COUNT)
        a1[8] = _clip(REF + v_grid / V_PER_COUNT)
        a2[1] = _clip(REF + i / self._gain_i)
        a3[3] = _clip(vbat / VBAT_PER_COUNT)
        a3[1] = _clip(vbus / VBUS_PER_COUNT)
        a3[2] = _clip(REF + ibat / IBAT_PER_COUNT)

    def step_pack(self) -> None:
        """Move the charge of the samples since the last call into the pack, at
        their average current over their own duration (the co-simulation's
        slices and the control's clock do not line up exactly)."""
        if self.bat_s <= 0:
            return
        self.pack.step(self.bat_s, self.bat_ah * 3600.0 / self.bat_s)
        self.bat_ah = self.bat_s = 0.0

    def status(self) -> dict:
        return {"ac_w": round(self.p_ac_avg, 1), "bat_w": round(self.p_bat_avg, 1), "vbus": round(self.vbus, 1),
                "relay": self.relay_closed(), "bridge": self.bridge_on(), "llc": self.llc_on(),
                "bypass": self.bypass_closed(), "i_peak_a": round(abs(self.i), 2)}


class VenusCE:
    """A whole Venus C/E: control firmware (HMG-50 V146-V158), BMS firmware
    (114-216) on the CAN bus with the KA495XX front end, and :class:`PowerStage`.

    Same interface as the other device classes (``control``, ``sim``,
    ``pack``, ``ac_power``, ``run``, ``status``, ``close``), so
    :func:`~marsemu.plant.venus3_cli.connect_house` and the CLI work with it.
    The BMS runs in its own process unless ``parallel=False``.
    """

    CAPACITY_AH = 100.0
    CELLS = 16
    BMS_AMPS_PER_COUNT = None   # the E 3.0 BMS scale (set below)
    #: configure_self_consumption: cd=18 (meter) resets the working mode to manual
    METER_BEFORE_MODE = True

    def __init__(self, control, bms, *, pack=None, spec: Optional[PowerStageSpec] = None,
                 control_kwargs: Optional[dict] = None, bms_kwargs: Optional[dict] = None,
                 slice_s: float = 0.002, realtime: bool = False, parallel: bool = True,
                 llc_irq_divider: int = 30, **_ignored):
        from ..emulator import Emulator
        from .battery import LfpPack
        from .canbus import CanBus
        from .cosim import CoSim, EmulatorCanNode
        from .venus3 import AMPS_PER_COUNT, BmsFrontEnd, _build_bms

        self.sim = CoSim(slice_s=slice_s, realtime=realtime)
        self.bus = CanBus(500_000)
        self.bus._now = lambda: self.sim.now
        self.pack = pack or LfpPack(cells_n=self.CELLS, capacity_ah=self.CAPACITY_AH)
        self.packs = [self.pack]
        self.control = Emulator(control, **(control_kwargs or {}))
        # the 30 kHz loop unmasks interrupts ~2000 times a second, which keeps the
        # machine's adaptive policy switching exits (and flushing) ~150 times a second
        self.control.machine.keep_exits = True
        self.sim.add_emulator(self.control)
        self.control_can = self.bus.attach(EmulatorCanNode(self.control, "FDCAN2", "control"))
        amps = self.BMS_AMPS_PER_COUNT or AMPS_PER_COUNT
        make_front = lambda emu, p, index=0: BmsFrontEnd(emu, p, amps)
        bms_kwargs = dict(bms_kwargs or {})
        bms_kwargs.setdefault("profile", "aux")
        if isinstance(bms, str):
            bms = open(bms, "rb").read()
        self.parallel = parallel
        if parallel:
            from .remote import RemoteMcu
            p = self.pack
            self.bms = None
            self.front = None
            self.bms_can = self.bus.attach(RemoteMcu(
                "bms", _build_bms, (bms, bms_kwargs, make_front, 0), "CAN1",
                state=lambda: (p.cell_voltages, p.temp_c, p.current_a, None)))
            self.sim.add_remote(self.bms_can)
        else:
            self.bms = Emulator(bms, **bms_kwargs)
            self.front = make_front(self.bms, self.pack)
            self.sim.add_emulator(self.bms)
            self.bms_can = self.bus.attach(EmulatorCanNode(self.bms, "CAN1", "bms"))
        self.stage = PowerStage(self.control, self.pack, spec, llc_irq_divider=llc_irq_divider)
        self.sim.tickers.append(lambda now, dt: self.stage.step_pack())

    # -- device interface
    def run(self, seconds: float, until=None) -> None:
        self.sim.run(seconds, until)

    def close(self) -> None:
        self.sim.close()
        self.control.save_state()

    @property
    def time(self) -> float:
        return self.sim.now

    @property
    def ac_power(self) -> float:
        """AC power, W, positive = feeding the house/grid (discharging)."""
        return self.stage.p_ac_avg

    @property
    def soc(self) -> float:
        return self.pack.soc

    def mos(self) -> dict:
        if self.front is not None:
            return self.front.mos()
        return dict(self.bms_can.report or {})

    def status(self) -> dict:
        st = self.stage.status()
        return {
            "t": round(self.sim.now, 3), "ac_w": st["ac_w"], "dc_w": st["bat_w"], "vbus": st["vbus"],
            "grid_relay": st["relay"], "inverter": st["bridge"], "llc": st["llc"],
            "soc": round(self.pack.soc * 100, 2), "pack_v": round(self.pack.voltage, 2),
            "i_a": round(self.pack.current_a, 2), "temp_c": round(self.pack.temp_c, 1), "mos": self.mos(),
            "limits": [2500, -2500],
        }


def _clip(x: float) -> int:
    return 0 if x < 0 else 4095 if x > 4095 else int(x)


__all__ = ["PowerStage", "PowerStageSpec", "VenusCE"]
