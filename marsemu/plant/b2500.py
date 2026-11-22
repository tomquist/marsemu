"""B2500 (HMA/HMJ, hardware v2): one N32G452 runs everything, so the power
stage is modelled at the level of its ADC inputs and its analog references.

Read from the firmware (HMA V232):

* Battery: SH367309 front end on I2C2 (14 LFP cells), see
  :mod:`marsemu.periph.sh367309`.  CADC current 12 mA per count.
* Two identical sides, each a PV input and an output to a microinverter.
  ADC1 (DMA, ch 1 2 3 6 7) and ADC2 (DMA, ch 5 4 12 8 9):

  ======================  =========  =========  ================================
  quantity                side 1     side 2     scale
  ======================  =========  =========  ================================
  PV voltage              ADC1 ch3   ADC2 ch5   25 mV/count + 32 mV
  PV current              ADC2 ch4   ADC2 ch12  8.25 mA/count (8.0: V119-V214, V241-V255)
  output voltage          ADC1 ch6   ADC1 ch7   25 mV/count + 32 mV
  output current          ADC1 ch1   ADC1 ch2   5.9 mA/count above a zero
                                                point (~610 counts, calibrated)
  heatsink NTC            ADC2 ch8   ADC2 ch9   table, 3129 counts = 25 degC
  ======================  =========  =========  ================================

* The PV converters are analog controllers whose references are filtered
  PWM (36 kHz): the PV voltage setpoint (MPPT) ``CCR = 4374 - 0.062 * mV``
  (TIM8 CH3 / TIM2 CH4) and a current limit ``CCR = 4224 - 0.2793 * mA``
  (TIM8 CH4 / TIM2 CH3).
* The outputs are current sources whose reference is the DAC, in the units of
  the current-sense ADC: ``DAC = zero + (mA - 5) / 5.9``; 0xFFA switches the
  output off.  The firmware either steps that current to reach a power target
  or makes it follow a synthetic PV curve (MPP at 35.2 V), so that the
  microinverter's own MPP tracker settles at the wanted power; the inverter is
  modelled with a perturb-and-observe tracker for that reason.
"""
from __future__ import annotations

import logging
import math
from typing import List, Optional, Sequence

from ..periph.sh367309 import Sh367309
from .battery import LfpPack

log = logging.getLogger(__name__)

#: SH367309 sense resistor implied by the firmware's 12 mA per CADC count
RSENSE_OHM = 200.0 / (21470 * 12.0)
#: output current sensor zero point (ADC counts, firmware default 610/614)
OUT_ZERO = 610


#: ADC channels of HMA V232 (HMJ-2 V118 moves PV2 and the output currents)
HMA_V232_LAYOUT = {
    "pv_v0": ("ADC1", 3), "pv_i0": ("ADC2", 4), "pv_v1": ("ADC2", 5), "pv_i1": ("ADC2", 12),
    "out_v0": ("ADC1", 6), "out_i0": ("ADC1", 1), "out_v1": ("ADC1", 7), "out_i1": ("ADC1", 2),
}


#: B2500 v1 (HMB V121-V144): no output voltage channels; output current 31 mA/count, PV current 8 mA/count
HMB_LAYOUT = {
    "pv_v0": ("ADC1", 3), "pv_i0": ("ADC2", 4), "pv_v1": ("ADC2", 5), "pv_i1": ("ADC2", 12),
    "out_i0": ("ADC1", 1), "out_i1": ("ADC1", 2),
}


def v_to_adc(volts: float) -> int:
    return max(0, min(4095, int(round((volts * 1000 - 32) / 25))))


def pv_i_to_adc(amps: float, ma_per_count: float = 8.25) -> int:
    return max(0, min(4095, int(round(amps * 1000 / ma_per_count))))


def out_i_to_adc(amps: float, ma_per_count: float = 5.9) -> int:
    off = 5 if ma_per_count == 5.9 else 0
    return max(0, min(4095, int(round(OUT_ZERO + (amps * 1000 - off) / ma_per_count))))


def heatsink_adc(temp_c: float) -> int:
    """NTC 10 kOhm (B 3950) to ground against 3.09 kOhm (fits the firmware's table)."""
    r = 10.0 * math.exp(3950 * (1 / (temp_c + 273.15) - 1 / 298.15))
    return int(round(4096 * r / (r + 3.09)))


def afe_ntc_kohm(temp_c: float) -> float:
    return 10.0 * math.exp(3435 * (1 / (temp_c + 273.15) - 1 / 298.15))


class PvArray:
    """A PV string with an exponential I-V curve, scaled to a peak power."""

    def __init__(self, peak_w: float = 0.0, voc: float = 45.0):
        self.voc = voc
        self.a = 0.075 * voc
        self._norm = 1.0 / (math.exp(voc / self.a) - 1)
        vs = [voc * k / 400 for k in range(401)]
        self._pmax_unit, self.vmp = max((v * self._shape(v), v) for v in vs)
        self.set_peak(peak_w)

    def _shape(self, v: float) -> float:
        return max(0.0, 1.0 - (math.exp(v / self.a) - 1) * self._norm)

    def set_peak(self, peak_w: float) -> None:
        self.peak_w = max(0.0, peak_w)
        self.isc = self.peak_w / self._pmax_unit

    def current(self, v: float) -> float:
        return self.isc * self._shape(v) if v < self.voc else 0.0

    def voltage_for(self, i: float) -> float:
        """Voltage where the curve delivers ``i`` (bisection)."""
        lo, hi = 0.0, self.voc
        for _ in range(30):
            mid = (lo + hi) / 2
            if self.current(mid) > i:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2


class PvConverter:
    """PV input: regulates the PV voltage to the PWM setpoint under the PWM current limit."""

    def __init__(self, array: PvArray, efficiency: float = 0.97, tau_s: float = 0.02):
        self.array = array
        self.efficiency = efficiency
        self.tau = tau_s
        self.v = 0.0
        self.i = 0.0
        self.v_ref = 0.0
        self.i_lim = 0.0
        self.enabled = False

    def set_refs(self, ccr_v: int, ccr_i: int) -> None:
        self.v_ref = max(0.0, (4374 - ccr_v) * 500 / 31 / 1000)
        self.i_lim = max(0.0, (4224 - ccr_i) / 0.2793 / 1000)

    def step(self, dt: float, v_batt: float) -> None:
        arr = self.array
        if arr.isc <= 0:
            tv, ti = 0.0, 0.0
        elif not self.enabled or self.v_ref >= arr.voc:
            tv, ti = arr.voc, 0.0
        else:
            tv = self.v_ref
            ti = arr.current(tv)
            # the current limit is on the battery side: past it the converter
            # backs off towards open circuit
            p_max = self.i_lim * v_batt / self.efficiency
            if tv * ti > p_max:
                lo, hi = max(tv, arr.vmp), arr.voc
                for _ in range(30):
                    mid = (lo + hi) / 2
                    if mid * arr.current(mid) > p_max:
                        lo = mid
                    else:
                        hi = mid
                tv = hi
                ti = arr.current(tv)
        k = 1 - math.exp(-dt / self.tau)
        self.v += (tv - self.v) * k
        self.i += (ti - self.i) * k

    @property
    def power(self) -> float:
        return self.v * self.i

    @property
    def dc_power(self) -> float:
        """Into the battery bus, W."""
        return self.power * self.efficiency


class MicroInverterInput:
    """One DC input of a microinverter: an MPP tracker on its input voltage
    (a sweep of the whole range at start-up, and every ``rescan_s`` if set,
    then perturb-and-observe on the power averaged over each step), an AC
    power cap, and a dropout below ``drop_w``."""

    def __init__(self, max_w: float = 400.0, v_min: float = 16.0, v_max: float = 58.0,
                 step_v: float = 0.5, period_s: float = 0.5, scan_step_v: float = 1.0,
                 rescan_s: Optional[float] = None, efficiency: float = 0.955):
        self.max_w = max_w
        self.v_min = v_min
        self.v_max = v_max
        self.step_v = step_v
        self.period = period_s
        self.scan_step = scan_step_v
        self.rescan_s = rescan_s
        self.efficiency = efficiency
        self.v_set = v_max
        #: seconds of input voltage before the inverter connects
        self.start_s = 10.0
        #: below this DC power for ``drop_s`` the inverter disconnects (and later
        #: restarts with a fresh sweep); an assumption, not measured
        self.drop_w = 10.0
        self.drop_s = 10.0
        self._low_s = 0.0
        self.dir = -1.0
        self.p_prev = 0.0
        self.p_dc = 0.0
        self._t = 0.0
        self._e = 0.0
        self._since_scan = 0.0
        self.scanning = False
        self._best = (0.0, v_max)

    def starved(self, dt: float, p_dc: float) -> bool:
        """True once the input power has been below ``drop_w`` for ``drop_s``."""
        self._low_s = self._low_s + dt if p_dc < self.drop_w and not self.scanning else 0.0
        if self._low_s >= self.drop_s:
            self._low_s = 0.0
            return True
        return False

    def start(self, v_open: float) -> None:
        """Input voltage appeared: sweep down from the open-circuit voltage."""
        self._low_s = 0.0
        self.v_set = min(self.v_max, v_open)
        self.scanning = True
        self._best = (0.0, self.v_set)
        self._t = self._e = self._since_scan = 0.0

    def track(self, dt: float, p_dc: float) -> None:
        self.p_dc = p_dc
        self._t += dt
        self._e += p_dc * dt
        self._since_scan += dt
        if self._t < self.period:
            return
        p = self._e / self._t
        self._t = self._e = 0.0
        if self.scanning:
            if p > self._best[0]:
                self._best = (p, self.v_set)
            if self.v_set - self.scan_step >= self.v_min:
                self.v_set -= self.scan_step
                return
            self.scanning = False
            self.v_set = self._best[1]
            self.p_prev, self.dir = 0.0, 1.0
            self._since_scan = 0.0
            return
        if self.rescan_s and self._since_scan >= self.rescan_s:
            self.start(self.v_max)
            return
        if p > self.max_w:
            self.v_set -= self.step_v                # limit: move off the MPP
            self.dir = -1.0
        else:
            if p <= self.p_prev:
                self.dir = -self.dir
            self.v_set += self.dir * self.step_v
        self.v_set = min(self.v_max, max(self.v_min, self.v_set))
        self.p_prev = p

    @property
    def ac_power(self) -> float:
        return self.p_dc * self.efficiency


class OutputStage:
    """B2500 output: a current source (DAC reference) into the inverter input.

    ``offset_a`` is the current loop's offset (DAC against current-sense amplifier;
    an estimate).  The firmware computes the DAC value rounding down, so with an
    exact loop the measured current settles one count below the target.  The
    old-layout builds wait for the measured current to reach their 2 A start
    current without a margin (the new-layout builds aim 70 mA above it), and
    stay at 2 A unless the loop errs on the high side."""

    def __init__(self, inverter: MicroInverterInput, v_max: float = 60.0, slew_a_per_s: float = 40.0,
                 efficiency: float = 0.97, offset_a: float = 0.01):
        self.inv = inverter
        self.offset_a = offset_a
        self.v_max = v_max
        self.slew = slew_a_per_s
        self.efficiency = efficiency
        self.i_lim = 0.0
        self.i = 0.0
        self.v = 0.0
        self.enabled = False
        self.powered_s = 0.0

    def set_dac(self, dac: int, zero: int = OUT_ZERO) -> None:
        self.i_lim = max(0.0, ((dac - zero) * 5.9 + 5) / 1000 + self.offset_a)

    def step(self, dt: float) -> None:
        inv = self.inv
        if not self.enabled:
            self.powered_s = 0.0
            self.i = 0.0
            self.v *= math.exp(-dt / 0.5)             # input capacitors discharge
            inv.track(dt, 0.0)
            return
        if self.powered_s < inv.start_s <= self.powered_s + dt:
            # connects: tracking starts from the open-circuit voltage downwards
            inv.start(self.v_max)
        self.powered_s += dt
        if self.powered_s < inv.start_s:
            # the inverter waits before it starts drawing power
            want, v = 0.0, self.v_max
        else:
            # CC/CV source: up to the current limit below v_max, tailing off above
            v = min(inv.v_set, self.v_max)
            want = self.i_lim * max(0.0, min(1.0, 1.0 - (inv.v_set - self.v_max) / 2))
        d = self.slew * dt
        self.i += max(-d, min(d, want - self.i))
        self.v = v
        inv.track(dt, self.v * self.i)
        if self.powered_s >= inv.start_s and inv.starved(dt, self.v * self.i):
            self.powered_s = 0.0                      # disconnected: restart sequence

    @property
    def power(self) -> float:
        return self.v * self.i

    @property
    def dc_power(self) -> float:
        """Taken from the battery bus, W (negative)."""
        return -self.power / self.efficiency


class SwitchedOutput:
    """B2500 v1 (HMB) output: no converter, just switches.  The battery
    (PB15 output 1 / PB14 output 2) or, on output 2, the PV2 panel in
    pass-through mode (PB12) is connected straight to the microinverter."""

    def __init__(self, inverter: MicroInverterInput, ramp_w_per_s: float = 50.0):
        self.inv = inverter
        self.ramp = ramp_w_per_s
        self.battery_on = False
        self.passthrough: Optional[PvArray] = None
        self.enabled = False                   # compatibility with OutputStage users
        self.i_lim = 0.0
        self.v = 0.0
        self.i = 0.0
        self.p = 0.0
        self.powered_s = 0.0
        self._source = None

    def set_dac(self, dac: int, zero: int = OUT_ZERO) -> None:
        pass

    def step(self, dt: float, v_batt: float = 0.0) -> None:
        inv = self.inv
        arr = self.passthrough if self.passthrough is not None and self.passthrough.isc > 0 else None
        source = "pv" if arr is not None else ("battery" if self.battery_on else None)
        if source != self._source:
            self._source = source
            self.powered_s = 0.0
            self.p = 0.0
        if source is None:
            self.i = self.p = 0.0
            self.v *= math.exp(-dt / 0.5)
            inv.track(dt, 0.0)
            return
        if self.powered_s < inv.start_s <= self.powered_s + dt:
            inv.start(arr.voc if arr is not None else inv.v_max)
        self.powered_s += dt
        if source == "pv":
            if self.powered_s < inv.start_s:
                self.v, self.i = arr.voc, 0.0
            else:
                self.v = min(inv.v_set, arr.voc)
                self.i = arr.current(self.v)
            self.p = self.v * self.i
        else:
            # a battery is a stiff source: the inverter simply ramps to its limit
            self.v = v_batt
            target = inv.max_w if self.powered_s >= inv.start_s else 0.0
            self.p += max(-self.ramp * dt * 4, min(self.ramp * dt, target - self.p))
            self.i = self.p / v_batt if v_batt else 0.0
        inv.p_dc = self.p
        if source == "pv":
            inv.track(dt, self.p)

    @property
    def power(self) -> float:
        return self.p

    @property
    def dc_power(self) -> float:
        """Taken from the battery, W (negative); pass-through power does not touch it."""
        return -self.p if self._source == "battery" else 0.0


class ExtensionPack:
    """An extension battery pack on the B2500's UART5 bus: it answers the main
    unit's poll for its index (1 or 2) from its own pack model.  The power it
    carries is shared out by :class:`B2500Plant` (packs in parallel on the bus)."""

    REPLY_MS = 20.0

    def __init__(self, emu, index: int, pack: LfpPack, uart: str = "UART5"):
        from ..b2500bus import FrameParser
        self.emu, self.index, self.pack = emu, index, pack
        self.uart = emu.machine.bus.get(uart)
        self.parser = FrameParser()
        self.answered = 0
        self.uart.tx_listeners.append(self._on_tx)

    def _on_tx(self, data: bytes) -> None:
        for f in self.parser.feed(data):
            if f[3] == 0x30 + self.index and len(f) == 14:
                self.emu.machine.clock.after_ms(self.REPLY_MS, self._reply, "ext-pack")
            elif f[3] == 0x35 + self.index and len(f) >= 8:
                self.emu.machine.clock.after_ms(self.REPLY_MS, lambda req=f: self._link(req), "ext-pack")

    def _link(self, request: bytes) -> None:
        from ..b2500bus import link_reply
        self.uart.inject(link_reply(request, self.pack.current_a * 1000))

    def _reply(self) -> None:
        from ..b2500bus import pack_reply
        p = self.pack
        v = p.voltage
        self.uart.inject(pack_reply(self.index, [c * 1000 for c in p.cell_voltages], v * 1000, p.current_a * 1000,
                                    v * p.current_a, [p.temp_c] * 5, p.soc * 100))
        self.answered += 1


class B2500Plant:
    """Everything around the MCU: battery + AFE, two PV inputs, two outputs."""

    STANDBY_W = 4.0

    def __init__(self, emu, pack: Optional[LfpPack] = None, pv: Sequence[float] = (0.0, 0.0),
                 inverter_w: Sequence[float] = (400.0, 400.0), heatsink_c: float = 25.0,
                 extension_packs: Sequence[LfpPack] = ()):
        self.emu = emu
        m = emu.machine
        self.m = m
        self.pack = pack or LfpPack(cells_n=14, capacity_ah=50.0, soc=0.5)
        # the firmware keeps its SOC in the settings record the profile seeded
        from ..profiles import set_b2500_soc
        set_b2500_soc(emu, self.pack.soc * 100)
        #: extension packs on UART5, in parallel with the main pack on the battery bus
        self.ext_packs = list(extension_packs)[:2]
        self.ext = [ExtensionPack(emu, k + 1, p) for k, p in enumerate(self.ext_packs)]
        self.afe = Sh367309()
        self.afe.measure = self._measure
        m.bus.get("I2C2").devices[self.afe.address] = self.afe
        self.arrays = [PvArray(p) for p in pv]
        self.pv = [PvConverter(a) for a in self.arrays]
        self.inverters = [MicroInverterInput(w) for w in inverter_w]
        #: B2500 v1 (HMB): switched outputs, no DAC, no output voltage sensing
        self.v1 = emu.profile.name == "b2500-v1"
        if self.v1:
            self.out = [SwitchedOutput(inv) for inv in self.inverters]
        else:
            self.out = [OutputStage(inv) for inv in self.inverters]
        self.heatsink_c = heatsink_c
        self.adc = {}
        #: signal -> (ADC, channel); resolved from the image once the ADC DMA runs
        self.layout: Optional[dict] = HMB_LAYOUT if self.v1 else None
        #: PV current scale, mA per ADC count (8.0 in HMB and in HMA V119-V214, V241-V255; else 8.25)
        self.pv_i_ma = [8.0, 8.0] if self.v1 else [8.25, 8.25]
        self._layout_pending = not self.v1
        self._refresh_adc()
        for name in ("ADC1", "ADC2"):
            m.bus.get(name).provider = self._sample
        self.tim8 = m.bus.get("TIM8")
        self.tim2 = m.bus.get("TIM2")
        self.dac = m.bus.get("DAC")
        self.gpiob = m.bus.get("GPIOB")
        self.dt = 0.01
        self._schedule()

    # -- MCU side
    def _sample(self, adc: str, ch: int) -> Optional[int]:
        return self.adc.get((adc, ch))

    def _refresh_adc(self) -> None:
        if self.layout is None and self._layout_pending:
            self._resolve_layout()
        pv, out = self.pv, self.out
        hs = heatsink_adc(self.heatsink_c)
        values = {
            "pv_v0": v_to_adc(pv[0].v), "pv_i0": pv_i_to_adc(pv[0].i, self.pv_i_ma[0]),
            "pv_v1": v_to_adc(pv[1].v), "pv_i1": pv_i_to_adc(pv[1].i, self.pv_i_ma[1]),
        }
        if self.v1:
            values.update(out_i0=out_i_to_adc(out[0].i, 31.0), out_i1=out_i_to_adc(out[1].i, 31.0))
        else:
            values.update(out_v0=v_to_adc(out[0].v), out_i0=out_i_to_adc(out[0].i),
                          out_v1=v_to_adc(out[1].v), out_i1=out_i_to_adc(out[1].i))
        layout = self.layout or HMA_V232_LAYOUT
        adc = {pin: values[sig] for sig, pin in layout.items()}
        adc[("ADC2", 8)] = adc[("ADC2", 9)] = hs
        self.adc = adc

    def _resolve_layout(self) -> None:
        """Which ADC channel carries which signal in this build (see
        :mod:`marsemu.analysis.b2500`), once the firmware has set up the
        ADC DMA so the buffers and channel sequences are known."""
        dma = self.m.bus.peripherals.get("DMA1")
        if dma is None:
            self._layout_pending = False
            return
        bufs = {}
        for i in range(8):
            pa = dma.regs.get(0x10 + 0x14 * i, 0)
            for name, dr in (("ADC1", 0x4002084C), ("ADC2", 0x40020C4C)):
                if pa == dr and dma.regs.get(0x08 + 0x14 * i, 0) & 1:
                    bufs[dma.regs.get(0x14 + 0x14 * i, 0)] = name
        if len(bufs) < 2:
            return
        self._layout_pending = False
        from ..analysis.b2500 import find_adc_slots
        scales: dict = {}
        try:
            slots = find_adc_slots(self.emu.image, self.emu.base, tuple(bufs), scales)
        except Exception as e:                                  # pragma: no cover - analysis failure
            log.warning("B2500 ADC layout analysis failed (%s); using the HMA V232 layout", e)
            return
        layout = {}
        for (buf, slot), sig in slots.items():
            seq = self.m.bus.get(bufs[buf]).sequence()
            if slot < len(seq):
                layout[sig] = (bufs[buf], seq[slot])
        if set(layout) != set(HMA_V232_LAYOUT):
            log.warning("B2500 ADC layout incomplete (%s); using the HMA V232 layout for the rest", sorted(layout))
            layout = {**HMA_V232_LAYOUT, **layout}
        self.layout = layout
        for k in range(2):
            self.pv_i_ma[k] = scales.get(f"pv_i{k}", self.pv_i_ma[k])
        log.info("B2500 ADC layout: %s; PV current %s mA/count",
                 ", ".join(f"{k}={a}:{c}" for k, (a, c) in sorted(layout.items())), self.pv_i_ma)

    def _read_refs(self) -> None:
        t8, t2 = self.tim8.regs, self.tim2.regs
        self.pv[0].set_refs(t8.get(0x3C, 4374), t8.get(0x40, 4224))
        self.pv[1].set_refs(t2.get(0x40, 4374), t2.get(0x3C, 4224))
        d = self.dac.regs
        en = d.get(0x00, 0)
        self.out[0].set_dac(d.get(0x08, 0) if en & 1 else 0)
        self.out[1].set_dac(d.get(0x14, 0) if en & 0x10000 else 0)
        gb = self.gpiob
        for k in range(2):
            self.pv[k].enabled = bool(gb.pin_out(6 + k))       # PB6/PB7: PV converters
        if self.v1:
            self.out[0].battery_on = bool(gb.pin_out(15))      # PB15: battery to output 1
            self.out[1].battery_on = bool(gb.pin_out(14))      # PB14: battery to output 2
            self.out[1].passthrough = self.arrays[1] if gb.pin_out(12) else None   # PB12: PV2 to output 2
        else:
            for k in range(2):
                self.out[k].enabled = bool(gb.pin_out(14 + k))     # PB14/PB15: outputs

    def _measure(self, afe: Sh367309) -> None:
        afe.set_cells_mv([v * 1000 for v in self.pack.cell_voltages])
        r = afe_ntc_kohm(self.pack.temp_c)
        for k in range(3):
            afe.set_ntc_kohm(k, r)
        afe.set_current(self.pack.current_a, RSENSE_OHM)
        afe.tick()

    # -- physics
    def _schedule(self) -> None:
        self.m.clock.after_ms(self.dt * 1000, self._tick, "b2500-plant")

    def _tick(self) -> None:
        self._schedule()
        dt = self.dt
        self._read_refs()
        vb = self.pack.voltage
        for c in self.pv:
            c.step(dt, vb)
        for o in self.out:
            o.step(dt, vb) if self.v1 else o.step(dt)
        # the battery FETs gate everything (BSTATUS3 bit 0 discharge, bit 1 charge)
        p = sum(c.dc_power for c in self.pv) + sum(o.dc_power for o in self.out) - self.STANDBY_W
        # packs in parallel share the bus power by capacity (the main pack's share is what its front end sees)
        packs = [self.pack] + self.ext_packs
        total_ah = sum(q.capacity_ah for q in packs)
        for q in packs:
            q.step(dt, q.current_for_power(p * q.capacity_ah / total_ah))
        self._refresh_adc()

    # -- user side
    def set_pv(self, watts: Sequence[float]) -> None:
        for a, w in zip(self.arrays, watts):
            a.set_peak(w)

    @property
    def ac_power(self) -> float:
        return sum(inv.ac_power for inv in self.inverters)

    def status(self) -> dict:
        return {
            "pv_w": [round(c.power, 1) for c in self.pv],
            "pv_v": [round(c.v, 2) for c in self.pv],
            "pv_ref_v": [round(c.v_ref, 2) for c in self.pv],
            "pv_ilim_a": [round(c.i_lim, 2) for c in self.pv],
            "out_w": [round(o.power, 1) for o in self.out],
            "out_v": [round(o.v, 2) for o in self.out],
            "out_i_lim": [round(o.i_lim, 2) for o in self.out],
            "inv_v": [round(i.v_set, 1) for i in self.inverters],
            "ac_w": round(self.ac_power, 1),
            "soc": round(self.pack.soc * 100, 2),
            "pack_v": round(self.pack.voltage, 2),
            "i_a": round(self.pack.current_a, 2),
            "fets": self.afe.fets(),
            **({"ext": [{"soc": round(q.soc * 100, 2), "pack_v": round(q.voltage, 2), "i_a": round(q.current_a, 2)}
                        for q in self.ext_packs]} if self.ext_packs else {}),
        }


class B2500:
    """A whole B2500 (hardware v2): its firmware on the emulated MCU, the
    plant around it and two microinverter inputs.  Same interface as the
    Venus boards (``sim``, ``control``, ``run``, ``status``, ...)."""

    CAPACITY_AH = 50.0
    CELLS = 14
    #: configure_self_consumption waits this long after the subscription: for
    #: 3 s after it subscribes the firmware (HMJ-2 110 and 116) holds an
    #: incoming command and then drops it, whatever it is (a cd=10 restart, or
    #: the cd=27 meter setting, which left HMJ-2 110 without a CT)
    SETUP_DELAY_S = 5.0

    def __init__(self, firmware, *, pack: Optional[LfpPack] = None, pv: Sequence[float] = (0.0, 0.0),
                 inverter_w: Sequence[float] = (400.0, 400.0), control_kwargs: Optional[dict] = None,
                 slice_s: float = 0.02, realtime: bool = False, extension_packs: Sequence = (), **_):
        """``extension_packs``: up to two :class:`LfpPack`, or a number (packs like the main one, same SOC)."""
        from ..emulator import Emulator
        from .cosim import CoSim

        self.sim = CoSim(slice_s=slice_s, realtime=realtime)
        self.control = Emulator(firmware, **(control_kwargs or {}))
        self.sim.add_emulator(self.control)
        if isinstance(extension_packs, int):
            soc = pack.soc if pack is not None else 0.5
            extension_packs = [LfpPack(cells_n=self.CELLS, capacity_ah=self.CAPACITY_AH, soc=soc)
                               for _ in range(extension_packs)]
        self.plant = B2500Plant(self.control, pack, list(pv) + [0.0] * (2 - len(pv)), inverter_w,
                                extension_packs=extension_packs)
        self.pack = self.plant.pack
        from ..b2500bus import BusMonitor
        #: the battery status the firmware broadcasts on its extension-pack bus (UART5)
        self.bus = BusMonitor(self.control)

    def set_pv(self, watts: Sequence[float]) -> None:
        self.plant.set_pv(list(watts) + [0.0] * (2 - len(watts)))

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
        return self.plant.ac_power

    @property
    def soc(self) -> float:
        return self.pack.soc

    def status(self) -> dict:
        st = {"t": round(self.sim.now, 3), **self.plant.status(), "temp_c": round(self.pack.temp_c, 1)}
        if self.bus.status:
            st["bus"] = {k: v for k, v in self.bus.status.items() if k not in ("raw", "cmd", "len", "type")}
        return st


__all__ = ["B2500", "B2500Plant", "ExtensionPack", "SwitchedOutput", "PvArray", "PvConverter", "OutputStage", "MicroInverterInput"]
