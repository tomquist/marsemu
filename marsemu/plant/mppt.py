"""The PV charger (MPPT) board of the Venus D and the Jupiter C Plus, with its real firmware.

The board has a TMS320F28035 (C28x + CLA, profile ``mppt``) and four PV
inputs, each a synchronous boost stage into the battery.  The firmware (VNSD-0
MPPT 102.5/104, JPLS-8H 213/217) runs the MPPT and the stages' control loops,
protects the strings and the battery, and talks to the control board on
eCAN-A as CAN node ``0x21``: it answers the control's status poll (``0x03``),
takes its PV power limit and charge voltage (``0x02``), and reports its
version (``0xCB``).  Its debug console (Letter shell: ``adcshow``,
``mpptshow``, ``canshow``, ``prtshow``, ...) is on LIN-A at 115200 baud.

This module is the hardware around the MCU, as the firmware sees it (mapped
by feeding each ADC channel its own value and reading the firmware's
``adcshow``):

==================  =====================================================
signal              here
==================  =====================================================
PV1..PV4 voltage    ADCINB0, B2, B4, B6: 0.016603 V per count
PV1..PV4 current    ADCINB1, B3, B5, B7: 11/1024 A per count
reference           ADCINA0: the current sensors' mid point (1.65 V)
battery voltage     ADCINA2: 0.017947 V per count
battery current     ADCINA4: (A0 - A4) x 0.0393 A, + = into the battery
PE voltage          ADCINA6: 0.0324 V per count
boost stages        EPWM1..EPWM4 (PV1..PV4): 50 kHz, interleaved by 90
                    degrees; A drives the low-side switch, B the
                    synchronous rectifier (off until the firmware enables it)
chip temperature    the F28035's own sensor
==================  =====================================================

The firmware's PV1..PV4 are the control's strings 4..1: the status block it
sends lists the inputs in the opposite order.  Strings here are numbered as
the control and the app number them.

Each stage is averaged over a switching period: in continuous conduction the
low-side duty ``D`` holds the string at ``(1 - D) x V_bat`` (filtered by
the input capacitor), and the string delivers what its I-V curve gives at
that voltage.  Without the synchronous rectifier the diode lets current flow
into the battery only.  Inductance, capacitance and losses are estimates.
"""
from __future__ import annotations

import logging
import struct
from typing import List, Optional, Sequence

from .b2500 import PvArray
from .canbus import Frame
from .cosim import EmulatorCanNode
from .remote import RemoteMcu

log = logging.getLogger(__name__)

PV_V_PER_COUNT = 0.016603
PV_A_PER_COUNT = 11 / 1024
BAT_V_PER_COUNT = 0.01794745
BAT_A_PER_COUNT = 0.0393
PE_V_PER_COUNT = 0.0324
#: the current sensors' reference, counts (1.65 V)
VREF_COUNTS = 2048
CH_VREF, CH_BAT_V, CH_BAT_I, CH_PE = 0, 2, 4, 6
#: firmware input k (PV1..PV4) -> its (voltage, current) ADC channels
CH_PV = ((8, 9), (10, 11), (12, 13), (14, 15))
ADC_NOISE = 2
#: how often the board's readings are worked out afresh, s
REFRESH_S = 50e-6
MPPT_ADDR = 0x21


def _clip(x: float) -> int:
    return 0 if x < 0 else 4095 if x > 4095 else int(x)


class BoostInput:
    """One PV input: the string, the input capacitor and a synchronous boost
    stage driven by one ePWM (A low-side switch, B synchronous rectifier)."""

    def __init__(self, array: PvArray, epwm, efficiency: float = 0.975, tau_s: float = 0.0005,
                 r_reverse: float = 0.2):
        self.array = array
        self.epwm = epwm
        self.efficiency = efficiency
        #: how fast the string voltage follows the duty (input capacitor and loop), s
        self.tau = tau_s
        #: path resistance when the synchronous rectifier drives current back into the string
        self.r_reverse = r_reverse
        self.v = array.voc if array.isc > 0 else 0.0
        self.i = 0.0
        self.p = 0.0

    def switching(self):
        """(low-side duty, synchronous rectifier on)."""
        e = self.epwm
        if not e.outputs_enabled():
            return 0.0, False
        return e.duty("A"), e.duty("B") > 0.0

    def step(self, dt: float, v_bat: float) -> None:
        arr = self.array
        d, sync = self.switching()
        voc = arr.voc if arr.isc > 0 else 0.0
        if d <= 0.0 and not sync:
            # stage off: the string sits at open circuit, or feeds the
            # battery through inductor and diode if it is above it
            target = min(voc, v_bat)
        else:
            target = (1.0 - d) * v_bat
            if not sync:
                target = min(target, voc)
        k = min(1.0, dt / self.tau) if self.tau > 0 else 1.0
        self.v += (target - self.v) * k
        if sync and self.v > voc:
            # the rectifier pushes the string above its open-circuit voltage:
            # current flows back from the battery
            self.i = -(self.v - voc) / self.r_reverse
        else:
            self.v = min(self.v, voc)
            self.i = arr.current(self.v)
        self.p = self.v * self.i


class MpptPlant:
    """The MPPT board around the F28035: four strings and boost stages, the
    battery terminals, and the board's sensors."""

    def __init__(self, emu, strings: Sequence[float] = (0, 0, 0, 0), voc: float = 48.0,
                 v_bat: float = 52.0, temp_c: float = 25.0):
        self.emu = emu
        m = self.m = emu.machine
        self.clock = m.clock
        #: per string as the control numbers them
        self.arrays = [PvArray(float(w), voc=voc) for w in list(strings) + [0.0] * (4 - len(strings))][:4]
        # firmware input k drives EPWM(k+1) and is the control's string 4-k
        self.inputs: List[BoostInput] = [BoostInput(self.arrays[3 - k], m.epwm[k + 1]) for k in range(4)]
        self.v_bat = v_bat
        self.i_bat = 0.0           # into the battery, A
        self.dc_power = 0.0        # into the battery, W
        self.energy = 0.0          # J into the battery since start
        self._t = self.clock.seconds
        self._adc_vals = [0] * 16
        self._adc_cyc = -1 << 62
        self._refresh_cycles = max(1, int(REFRESH_S * self.clock.hz))
        self._rng = 4321
        def source(ch: int) -> int:
            return self._adc(ch)
        source.batch = self._adc_batch      # all 16 at once (the native ADC samples a batch)
        m.adc.source = source
        self.set_temp(temp_c)

    # -- inputs from the board
    def set_pv(self, watts: Sequence[float]) -> None:
        for a, w in zip(self.arrays, list(watts) + [0.0] * (4 - len(watts))):
            a.set_peak(w)

    def set_battery(self, volts: float) -> None:
        self.advance()
        self.v_bat = volts

    def set_temp(self, c: float) -> None:
        from ..c28x import f2803x
        self.m.adc.temp_raw = f2803x.temp_raw(c)

    # -- power stage
    def advance(self) -> None:
        now = self.clock.seconds
        dt = now - self._t
        if dt <= 0:
            return
        self._t = now
        p = 0.0
        for s in self.inputs:
            s.step(dt, self.v_bat)
            p += s.p * (s.efficiency if s.p > 0 else 1.0 / s.efficiency)
        self.dc_power = p
        self.i_bat = p / self.v_bat if self.v_bat > 1 else 0.0
        self.energy += p * dt

    # -- ADC
    def _adc(self, ch: int) -> int:
        # the ADC converts at about 100 kHz; the stages move on a scale of
        # their input capacitors (~0.5 ms), so the readings are worked out
        # afresh every REFRESH_S
        cyc = self.clock.cycles
        if not 0 <= cyc - self._adc_cyc < self._refresh_cycles:
            self._adc_cyc = cyc
            self._refresh_adc()
        return self._adc_vals[ch]

    def _adc_batch(self, mask: int) -> List[int]:
        cyc = self.clock.cycles
        if not 0 <= cyc - self._adc_cyc < self._refresh_cycles:
            self._adc_cyc = cyc
            self._refresh_adc()
        return self._adc_vals

    def _noise(self) -> int:
        self._rng = (self._rng * 1103515245 + 12345) & 0x7FFFFFFF
        return (self._rng >> 16) % (2 * ADC_NOISE + 1) - ADC_NOISE

    def _refresh_adc(self) -> None:
        self.advance()
        v, n = self._adc_vals, self._noise
        vref = VREF_COUNTS + n()
        v[CH_VREF] = vref
        v[CH_BAT_V] = _clip(self.v_bat / BAT_V_PER_COUNT + n())
        v[CH_BAT_I] = _clip(vref - self.i_bat / BAT_A_PER_COUNT + n())
        v[CH_PE] = _clip(15 + n())
        for s, (cv, ci) in zip(self.inputs, CH_PV):
            v[cv] = _clip(s.v / PV_V_PER_COUNT + n())
            v[ci] = _clip(s.i / PV_A_PER_COUNT + n())

    def status(self) -> dict:
        self.advance()
        strings = list(reversed(self.inputs))
        return {"pv_v": [round(s.v, 2) for s in strings], "pv_a": [round(s.i, 3) for s in strings],
                "pv_w": [round(s.p, 1) for s in strings], "dc_w": round(self.dc_power, 1),
                "bat_v": round(self.v_bat, 2)}


# ------------------------------------------------------------------ on a CAN bus
class _MpptSync:
    """State from the device to the MPPT process and back (see remote.RemoteMcu)."""

    def __init__(self, plant: MpptPlant):
        self.plant = plant

    def apply(self, state) -> None:
        v_bat, avail = state
        self.plant.set_battery(v_bat)
        self.plant.set_pv(avail)

    def report(self):
        return self.plant.status()


def _build_mppt(image, kwargs, strings, voc):
    from ..emulator import Emulator
    emu = Emulator(image, **kwargs)
    plant = MpptPlant(emu, strings, voc=voc)
    return emu, _MpptSync(plant)


class _MpptView:
    """What the device models and the dashboard read from an MPPT (the same
    attributes as :class:`~marsemu.plant.venusd.MpptNode`).  The PV limit,
    mode and charge voltage are decoded from the control's commands; the rest
    comes from the board model."""

    name = "mppt"
    pack = None
    cv_source = None

    def _init_view(self, pack, strings):
        self.pack = pack
        self.available = [float(x) for x in strings] + [0.0] * (4 - len(strings))
        self.limit_w: Optional[float] = 0.0
        self.vmax: Optional[float] = None
        self.mode = 0
        self.polls = 0
        self.version: Optional[int] = None
        self.limit_frames: List[tuple] = []
        self._pending02: Optional[bytes] = None
        self._snap = {"pv_v": [0.0] * 4, "pv_a": [0.0] * 4, "pv_w": [0.0] * 4, "dc_w": 0.0}

    @property
    def pv_w(self) -> List[float]:
        return [max(0.0, w) for w in self._snap["pv_w"]]

    @property
    def pv_v(self) -> List[float]:
        return list(self._snap["pv_v"])

    @property
    def dc_power(self) -> float:
        return float(self._snap["dc_w"])

    def set_pv(self, strings: Sequence[float]) -> None:
        self.available = [float(x) for x in strings] + [0.0] * (4 - len(strings))

    def _watch(self, f: Frame) -> None:
        """Decode the control's commands to the MPPT, and its version reply."""
        if not f.ext:
            return
        ident = f.ident
        func, idx = ident & 0xFF, ident >> 24
        src, dst = (ident >> 16) & 0xFF, (ident >> 8) & 0xFF
        if src == MPPT_ADDR and func == 0xCB:
            if idx == 1 and len(f.data) >= 4:      # <u16 model, u16 version, ...>
                self.version = struct.unpack_from("<H", f.data, 2)[0]
            return
        if dst != MPPT_ADDR or src == MPPT_ADDR:
            return
        if func == 0x03:
            self.polls += 1
        elif func == 0x02:
            self.limit_frames.append((round(f.t, 3), idx, f.data.hex()))
            del self.limit_frames[:-50]
            if idx == 0:
                self._pending02 = f.data
            elif self._pending02 is not None:
                on = struct.unpack_from("<I", self._pending02.ljust(4, b"\0"))[0]
                self.mode, lim, vmax = struct.unpack_from("<HHH", f.data.ljust(8, b"\0"))
                self.limit_w = float(lim) if on == 1 else None
                self.vmax = vmax / 10.0 if vmax else None
                self._pending02 = None


def firmware_mppt(sim, bus, image, pack, strings: Sequence[float] = (0, 0, 0, 0), *, voc: float = 48.0,
                  parallel: bool = True, kwargs: Optional[dict] = None):
    """Attach the MPPT board with its firmware ``image`` to ``bus``: in its
    own process (``parallel``) or in this one.  Returns the CAN node, which
    has the attributes of :class:`~marsemu.plant.venusd.MpptNode` (``pv_w``,
    ``dc_power``, ``limit_w``, ``set_pv``, ``step``)."""
    kwargs = {"profile": "mppt", **(kwargs or {})}
    if isinstance(image, str):
        image = open(image, "rb").read()
    strings = list(strings) + [0.0] * (4 - len(strings))
    if parallel:
        node = bus.attach(ProcessMppt(image, kwargs, pack, strings, voc))
        sim.add_remote(node)
    else:
        node = bus.attach(LocalMppt(image, kwargs, pack, strings, voc))
        sim.add_emulator(node.emu)
    bus.monitors.append(node._watch)
    return node


class ProcessMppt(RemoteMcu, _MpptView):
    """The MPPT firmware and board model in a child process."""

    def __init__(self, image, kwargs, pack, strings, voc):
        self._init_view(pack, strings)
        RemoteMcu.__init__(self, "mppt", _build_mppt, (image, kwargs, strings, voc), "ECANA",
                           state=lambda: (self.pack.voltage, list(self.available)), keep_spacing=True)
        self.name = "mppt"

    def finish(self) -> None:
        super().finish()
        if self.report:
            self._snap = self.report

    def step(self, now: float, dt: float) -> None:
        pass


class LocalMppt(EmulatorCanNode, _MpptView):
    """The MPPT firmware and board model in this process (``--serial``)."""

    def __init__(self, image, kwargs, pack, strings, voc):
        from ..emulator import Emulator
        self._init_view(pack, strings)
        self.emu = Emulator(image, **kwargs)
        self.plant = MpptPlant(self.emu, strings, voc=voc)
        EmulatorCanNode.__init__(self, self.emu, "ECANA", "mppt")

    def step(self, now: float, dt: float) -> None:
        self.plant.set_battery(self.pack.voltage)
        self.plant.set_pv(self.available)
        self._snap = self.plant.status()


__all__ = ["BoostInput", "MpptPlant", "firmware_mppt", "ProcessMppt", "LocalMppt"]
