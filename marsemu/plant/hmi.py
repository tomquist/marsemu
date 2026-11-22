"""Saturn M microinverter (HMI): the real firmware with its power stage.

The HMI-1/2/5/6/9/350/500 run one TMS320F28035 (C28x + CLA, profile ``hmi``)
that does everything: the Wi-Fi/BLE/MQTT side through the FC41D on SCI-A, the
grid-connect state machine, MPPT, and the 50 kHz current loop on the CLA.
This module is the hardware around it, as the firmware sees it:

=================  ==========================================================
signal             here
=================  ==========================================================
PV1 / PV2 voltage  ADCINA3 / ADCINB4 (the CLA reads volts as raw x 0.0166)
PV1 / PV2 current  ADCINA1 / ADCINA0 (6.5 mA per count)
grid voltage       ADCINA6 - ADCINB6, differential, mid-scale biased
grid zero crossing ECAP1 (GPIO24): falling and rising edges, the period
                   gives ``grd_f``
flyback clamps     ADCINA7 / ADCINB0 (``fb1_v`` / ``fb2_v``, 41 mV per count)
PE voltage         ADCINB2 (``pe1_v``; high means a wiring fault, 1057)
chip temperature   the F28035's own sensor (``chp_t``)
grid relay         GPIO21 (``rel_s`` is its latch)
flybacks           EPWM1/2 (PV1) and EPWM3/4 (PV2), 100 kHz, interleaved
                   above a threshold; GPIO17/18 drive the unfolding bridge
=================  ==========================================================

Grid-connect sequence (1 ms task, state in RAM ``0x8684``): 0 checks the grid
and clears the outputs, 1 tells 50 from 60 Hz by the ECAP period (45-55 Hz
and its 60 Hz counterpart), 2 observes the grid, 9/3
arm the relay and close it inside the ADC interrupt as the CLA's PLL sine
passes -0.95, 4 waits, 5 runs MPPT per input.

The power stage is averaged per switching period: each input is a DCM
flyback whose input current is ``V * D^2 * T / 2L`` with the duty the ePWM
outputs actually have when the ADC samples, so power flows only while the
firmware switches; the PV voltage follows the input capacitor against the
array's I-V curve.
"""

from __future__ import annotations

import math
import struct
from typing import List, Optional, Sequence

from .b2500 import PvArray

#: CLA scale of the PV voltage channels (float 0x3c880303), V per count
PV_V_PER_COUNT = struct.unpack("<f", struct.pack("<I", 0x3C880303))[0]
#: PV current, A per count (the firmware reports raw x 0.065 in 0.1 A)
PV_A_PER_COUNT = 0.0065
#: grid voltage, counts of (A6 - B6) per volt; ``grd_v`` reads 230.0 V at 3.787
GRID_COUNTS_PER_V = 3.787
#: flyback clamp voltage sensing, V per count (``fb1_v`` is raw x 0.4106 in 0.1 V)
FB_V_PER_COUNT = 0.04106

CH_PV1_V, CH_PV2_V = 3, 12        # A3, B4
CH_PV1_I, CH_PV2_I = 1, 0         # A1, A0
CH_GRID_P, CH_GRID_N = 6, 14      # A6, B6
CH_FB1, CH_FB2 = 7, 8             # A7, B0
CH_PE = 10                        # B2
RELAY_GPIO = 21
#: conversion noise, +/- counts
ADC_NOISE = 2
#: what status() and the AC power average over, s (the flybacks draw in pulses)
AVG_S = 0.04
AC_AVG_S = 0.2


class Flyback:
    """One PV input: array, input capacitor and a DCM flyback."""

    def __init__(self, array: PvArray, epwms, c_in: float = 2.0e-3, l_mag: float = 4e-6,
                 efficiency: float = 0.955, hz: float = 60e6, buffer_s: float = 0.0):
        self.array = array
        self.epwms = epwms
        self.c_in = c_in
        self.l_mag = l_mag
        #: ePWM time base clock (SYSCLKOUT)
        self.hz = hz
        #: the board's intermediate stage takes the flybacks' 100 Hz power
        #: pulsation; the string sees their demand averaged over this long
        self.buffer_s = buffer_s
        self.g_avg = 0.0
        self.efficiency = efficiency
        self.v = array.voc if array.isc > 0 else 0.0
        self.i = 0.0          # converter input current (switching-period average), A
        self.p = 0.0          # converter input power now, W
        self.energy = 0.0     # J drawn since start, for averages
        #: averages over a few grid cycles: the flybacks draw in pulses
        self.p_avg = 0.0
        self.i_avg = 0.0

    def conductance(self) -> float:
        """Input current per volt the converter draws now: each phase stores
        (V t_on)^2 / 2L a period, so draws V D^2 T / 2L on average.  The
        firmware sets period and on-time both (variable frequency)."""
        g = 0.0
        for e in self.epwms:
            d = e.duty("A")
            if d > 0:
                g += d * d * e.period_cycles() / (self.hz * 2 * self.l_mag)
        return g

    def step(self, dt: float, g: float) -> None:
        arr = self.array
        if dt <= 0:
            return
        if arr.isc <= 0:          # no light: no voltage either
            self.v = self.i = self.p = self.p_avg = self.i_avg = 0.0
            return
        k = min(1.0, dt / self.buffer_s) if self.buffer_s > 0 else 1.0
        self.g_avg += (g - self.g_avg) * k
        g = self.g_avg
        # the capacitor against the array: a few explicit substeps, or the
        # equilibrium when the step is long against the time constant
        tau = self.c_in / (g + arr.isc / max(0.1 * arr.voc, 1e-3) + 1e-9)
        if dt > 4 * tau:
            lo, hi = 0.0, arr.voc
            for _ in range(30):
                mid = (lo + hi) / 2
                if arr.current(mid) > g * mid:
                    lo = mid
                else:
                    hi = mid
            self.v = (lo + hi) / 2
        else:
            n = max(1, int(dt / (0.25 * tau)) + 1)
            h = dt / n
            v = self.v
            for _ in range(n):
                v += h * (arr.current(v) - g * v) / self.c_in
                v = min(max(v, 0.0), arr.voc)
            self.v = v
        self.i = g * self.v
        self.p = self.i * self.v
        self.energy += self.p * dt
        k = min(1.0, dt / AVG_S)
        self.p_avg += (self.p - self.p_avg) * k
        self.i_avg += (self.i - self.i_avg) * k


class HmiPlant:
    """Grid, relay, two PV inputs and the flyback/unfolder power stage of an HMI."""

    def __init__(self, emu, pv: Sequence[float] = (400.0, 400.0), voc: Sequence[float] = (49.0, 49.0),
                 grid_v: float = 230.0, grid_hz: float = 50.0, ambient_c: float = 25.0):
        self.emu = emu
        m = self.m = emu.machine
        self.clock = m.clock
        self.grid_v = grid_v
        self.grid_hz = grid_hz
        self.grid_on = True
        self.ambient_c = ambient_c
        self.arrays = [PvArray(p, voc=v) for p, v in zip(pv, voc)]
        ep = m.epwm
        self.inputs: List[Flyback] = [Flyback(self.arrays[0], (ep[1], ep[2]), hz=m.clock.hz),
                                      Flyback(self.arrays[1], (ep[3], ep[4]), hz=m.clock.hz)]
        self.relay = False
        self._t = self.clock.seconds
        self._phase0 = 0.0       # grid phase at clock 0, rad
        #: AC energy fed in since start, J
        self.ac_energy = 0.0
        self.ac_w = 0.0          # AC power, averaged over the last ~0.2 s
        self.ecap = m.get("ECAP1")
        #: what each ADC channel reads; mid-scale where nothing is modelled
        self._adc_vals = [2048] * 16
        self._adc_vals[CH_FB1] = self._adc_vals[CH_FB2] = int(83.2 / FB_V_PER_COUNT)
        self._adc_vals[CH_PE] = 52
        self._adc_cyc = -1
        self._rng = 12345
        m.adc.source = self._adc
        m.adc.temp_raw = self._temp_raw(ambient_c)
        m.gpio.listeners.append(self._gpio)
        self._zc_ev = None
        self._schedule_zero_crossing()

    # -- grid
    def grid_now(self) -> float:
        """Instantaneous grid voltage, V."""
        if not self.grid_on:
            return 0.0
        t = self.clock.seconds
        return self.grid_v * math.sqrt(2) * math.sin(2 * math.pi * self.grid_hz * t + self._phase0)

    def _schedule_zero_crossing(self) -> None:
        clk = self.clock
        t = clk.seconds
        half = 1.0 / (2 * self.grid_hz)
        k = math.floor(t / half) + 1
        when = k * half
        rising = k % 2 == 0
        self._zc_ev = clk.at(max(clk.cycles + 1, int(round(when * clk.hz))),
                             lambda: self._zero_crossing(rising), "grid-zc")

    def _zero_crossing(self, rising: bool) -> None:
        if self.grid_on:
            self.ecap.edge(rising)
        self._schedule_zero_crossing()

    def set_grid(self, on: bool = True, volts: Optional[float] = None, hz: Optional[float] = None) -> None:
        self.grid_on = on
        if volts is not None:
            self.grid_v = volts
        if hz is not None and hz != self.grid_hz:
            # keep the phase continuous
            t = self.clock.seconds
            self._phase0 += 2 * math.pi * (self.grid_hz - hz) * t
            self.grid_hz = hz

    # -- relay
    def _gpio(self, n: int, level: int) -> None:
        if n == RELAY_GPIO:
            self.relay = bool(level)

    # -- PV
    def set_pv(self, watts: Sequence[float]) -> None:
        for a, w in zip(self.arrays, watts):
            a.set_peak(w)

    # -- power stage
    def _advance(self) -> None:
        now = self.clock.seconds
        dt = now - self._t
        if dt <= 0:
            return
        self._t = now
        connected = self.relay and self.grid_on
        p_ac = 0.0
        for fb in self.inputs:
            g = fb.conductance() if connected else 0.0
            fb.step(dt, g)
            p_ac += fb.p * fb.efficiency
        self.ac_energy += p_ac * dt
        self.ac_w += (p_ac - self.ac_w) * min(1.0, dt / AC_AVG_S)

    def ac_power(self) -> float:
        """AC power fed in, averaged over the last ~0.2 s, W."""
        self._advance()
        return self.ac_w

    # -- ADC
    @staticmethod
    def _temp_raw(c: float) -> int:
        from ..c28x import f2803x
        return f2803x.temp_raw(c)

    def _adc(self, ch: int) -> int:
        # a sequence converts its channels back to back at one instant:
        # work the plant out once per sequence
        cyc = self.clock.cycles
        if cyc != self._adc_cyc:
            self._adc_cyc = cyc
            self._refresh_adc()
        return self._adc_vals[ch]

    def _refresh_adc(self) -> None:
        self._advance()
        v = self._adc_vals
        a, b = self.inputs
        n = self._noise
        v[CH_PV1_V] = int(a.v / PV_V_PER_COUNT) + n()
        v[CH_PV2_V] = int(b.v / PV_V_PER_COUNT) + n()
        v[CH_PV1_I] = max(0, int(a.i / PV_A_PER_COUNT) + n()) if a.v > 0 else 0
        v[CH_PV2_I] = max(0, int(b.i / PV_A_PER_COUNT) + n()) if b.v > 0 else 0
        half = GRID_COUNTS_PER_V * self.grid_now() / 2
        v[CH_GRID_P] = int(2048 + half) + n()
        v[CH_GRID_N] = int(2048 - half) + n()

    def _noise(self) -> int:
        """A few counts of conversion noise, as a real ADC has.  The firmware's
        MPPT starts an input only once the string voltage is seen above its
        running minimum; a perfectly steady reading never is."""
        self._rng = (self._rng * 1103515245 + 12345) & 0x7FFFFFFF
        return (self._rng >> 16) % (2 * ADC_NOISE + 1) - ADC_NOISE

    def status(self) -> dict:
        self._advance()
        return {
            "pv_v": [round(f.v, 2) for f in self.inputs],
            "pv_a": [round(f.i_avg, 3) for f in self.inputs],
            "pv_w": [round(f.p_avg, 1) for f in self.inputs],
            "relay": self.relay,
            "grid_v": self.grid_v if self.grid_on else 0.0,
            "grid_hz": self.grid_hz,
        }


class Hmi:
    """A whole HMI: its firmware on the emulated F28035 and the plant around it.
    Same shape as the other whole devices (``sim``, ``control``, ``run``,
    ``status``, ``ac_power``)."""

    def __init__(self, firmware, *, pv: Sequence[float] = (400.0, 400.0), voc: Sequence[float] = (49.0, 49.0),
                 grid_v: float = 230.0, grid_hz: float = 50.0, ambient_c: float = 25.0,
                 control_kwargs: Optional[dict] = None, slice_s: float = 0.02, realtime: bool = False, **_):
        from ..emulator import Emulator
        from .cosim import CoSim

        self.sim = CoSim(slice_s=slice_s, realtime=realtime)
        self.control = Emulator(firmware, **(control_kwargs or {}))
        if self.control.profile.soc != "f2803x":
            raise ValueError(f"{self.control.profile.name}: not an HMI firmware")
        self.sim.add_emulator(self.control)
        pv = list(pv) + [0.0] * (2 - len(pv))
        self.plant = HmiPlant(self.control, pv[:2], list(voc) + [voc[-1]] * (2 - len(voc)),
                              grid_v=grid_v, grid_hz=grid_hz, ambient_c=ambient_c)

    def set_pv(self, watts: Sequence[float]) -> None:
        self.plant.set_pv(list(watts) + [0.0] * (2 - len(watts)))

    def set_grid(self, on: bool = True, volts: Optional[float] = None, hz: Optional[float] = None) -> None:
        self.plant.set_grid(on, volts, hz)

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
        return self.plant.ac_power()

    def status(self) -> dict:
        return {"t": round(self.sim.now, 3), "ac_w": round(self.plant.ac_power(), 1), **self.plant.status()}


__all__ = ["Flyback", "Hmi", "HmiPlant"]
