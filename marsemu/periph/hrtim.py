"""STM32G4 HRTIM: master and timers A-F as periodic counters.

What firmware driving a power stage needs from it, not the waveforms: each
running timer unit counts its period (``PERxR`` at f_HRTIM x 32 / 2^CKPSC),
sets its compare/period/repetition flags once per period and raises its
interrupt when enabled, and fires the ADC trigger outputs (``ADC1R..ADC4R``)
it is a source of, which start the conversions of the ADCs whose regular
trigger is that HRTIM output.  Compare and output registers are plain
registers a plant model reads (duty cycles, output enable).

Only units somebody listens to (an enabled interrupt or an ADC trigger
source) get clock events, so idle units cost nothing.  The whole timer is
off until a power-stage model sets :attr:`Hrtim.active` (tens of kHz of
interrupts slow the emulation down a lot); until then the ADCs convert once
when started, as without a trigger.
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional

from .n32g45x import Base

UNITS = "MABCDEF"
#: ADC trigger register bits per source unit: compare 1-4, period (master: bits 0-4)
_TRG_BITS_13 = {"M": (0, 1, 2, 3, 4), "A": (None, 10, 11, 12, 13), "B": (None, 15, 16, 17, 18),
                "C": (None, 20, 21, 22, 23), "D": (None, 24, 25, 26, 27), "E": (None, 28, 29, 30, 31)}
_TRG_BITS_24 = {"M": (0, 1, 2, 3, 4), "A": (None, 10, 11, 12, None), "B": (None, 14, 15, 16, None),
                "C": (None, 18, 19, 20, None), "D": (None, 21, 22, 23, None), "E": (None, 24, 25, 26, None),
                "F": (None, 28, 29, 30, 31)}


class HrtimUnit(Base):
    """One HRTIM timer unit (``HRTIM_Master`` or ``HRTIM_TIMx``)."""

    def __init__(self, machine, name, base, key: str):
        self.key = key
        self.ev = None
        self.periods = 0
        self.hrtim: Optional["Hrtim"] = None
        #: clock events every ``divider`` periods (a slower stand-in for a fast unit)
        self.divider = 1
        super().__init__(machine, name, base)
        self.register_levels()

    # timer registers: CR 0x00, ISR 0x04, ICR 0x08, DIER 0x0C, CNT 0x10, PER 0x14, REP 0x18, CMP1 0x1C,
    # CMP1C 0x20, CMP2 0x24, CMP3 0x28, CMP4 0x2C
    CMP_OFF = (0x1C, 0x24, 0x28, 0x2C)

    def period_ticks(self) -> int:
        return self.regs.get(0x14, 0) & 0xFFFF

    def ckpsc(self) -> int:
        return self.regs.get(0x00, 0) & 7

    def period_cycles(self) -> float:
        """Period in f_HRTIM (= CPU) cycles."""
        per = self.period_ticks()
        return per * (1 << self.ckpsc()) / 32.0 if per else 0.0

    def frequency(self) -> float:
        c = self.period_cycles()
        return self.m.clock.hz / c if c else 0.0

    def compare(self, n: int) -> int:
        return self.regs.get(self.CMP_OFF[n - 1], 0) & 0xFFFF

    def duty(self, n: int) -> float:
        """Compare n as a fraction of the period."""
        per = self.period_ticks()
        return min(1.0, self.compare(n) / per) if per else 0.0

    def level(self):
        isr = self.regs.get(0x04, 0)
        ier = self.regs.get(0x0C, 0)
        return {n: bool(isr & ier & 0x7F) for n in self.irqs}

    def read32(self, off):
        if off == 0x10 and self.ev is not None:  # counter: position in the current period
            c = self.period_cycles()
            if c:
                frac = ((self.m.clock.cycles - self._t0) % c) / c
                return int(frac * self.period_ticks())
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x08:  # ICR: write 1 to clear
            self.regs[0x04] = self.regs.get(0x04, 0) & ~v
            return
        if off == 0x04:
            return
        self.regs[off] = v
        if off in (0x00, 0x0C, 0x14) and self.hrtim:
            self.hrtim.update()

    # clock
    def running(self) -> bool:
        return bool(self.hrtim and self.hrtim.active and self.hrtim.enabled(self.key))

    def start(self) -> None:
        if self.ev is not None:
            return
        self._t0 = self.m.clock.cycles
        self._acc = 0.0
        self._schedule()

    def stop(self) -> None:
        if self.ev is not None:
            self.ev.cancel()
            self.ev = None

    def _schedule(self) -> None:
        c = self.period_cycles()
        if c <= 0:
            self.ev = None
            return
        self._acc += c * self.divider
        n = int(self._acc)
        self._acc -= n
        self.ev = self.m.clock.after(max(n, 1), self._period, self.name)

    def _period(self) -> None:
        # The hot path of a power stage (tens of kHz): keep it to dict lookups.
        self.ev = None
        hrtim = self.hrtim
        if not (hrtim and hrtim._active and hrtim.enabled(self.key)):
            return
        self.periods += 1
        # compare 1-4 (bits 0-3), repetition (bit 4), update (bit 6) once per period
        regs = self.regs
        regs[0x04] = regs.get(0x04, 0) | 0x5F
        irq = regs.get(0x0C, 0) & 0x7F
        if irq:
            self.update_irq()
        mask = hrtim.trigger_mask(self.key)
        if mask:
            hrtim.fire_triggers(mask)
        listeners = hrtim.period_listeners
        for fn in listeners:
            fn(self)
        if irq or mask or listeners:
            self._schedule()


class Hrtim:
    """Ties the units together with ``HRTIM_Common``: enables (master MCR),
    ADC triggers (ADC1R-ADC4R) and listeners for plant models."""

    def __init__(self, machine):
        self.m = machine
        self._active = False
        self._mask_state = None
        self._masks: Dict[str, int] = {}
        bus = machine.bus
        self.units: Dict[str, HrtimUnit] = {}
        for k in UNITS:
            u = bus.get("HRTIM_Master" if k == "M" else f"HRTIM_TIM{k}")
            u.hrtim = self
            self.units[k] = u
        self.common = bus.get("HRTIM_Common")
        #: called with (trigger number 1-4) when an HRTIM ADC trigger fires
        self.adc_listeners: List[Callable[[int], None]] = []
        #: called with the unit after each period of a running unit
        self.period_listeners: List[Callable[[HrtimUnit], None]] = []
        #: output enable bits (OENR/ODISR): TA1 bit 0, TA2 1, TB1 2, ... TF2 11
        self.outputs = 0
        common_w, common_r = self.common.write32, self.common.read32

        def write_common(off, v):
            if off == 0x14:      # OENR: enable outputs
                self.outputs |= v & 0xFFF
                return
            if off == 0x18:      # ODISR: disable outputs
                self.outputs &= ~v & 0xFFF
                return
            common_w(off, v)
            if 0x3C <= off <= 0x48:
                self.update()

        def read_common(off):
            if off == 0x14:      # OENR reads the enable state
                return self.outputs
            if off in (0x18, 0x1C):  # ODISR reads 0; ODSR: no output disabled by a fault
                # (not through the generic register file: HAL's ``ODISR |= x`` reads it
                # every time, and its busy-wait breaker would eventually answer all
                # ones and disable every output)
                return 0
            return common_r(off)
        self.common.write32, self.common.read32 = write_common, read_common

    @property
    def active(self) -> bool:
        """Whether the timer units run (see the module doc)."""
        return self._active

    @active.setter
    def active(self, on: bool) -> None:
        self._active = bool(on)
        self.update()

    def enabled(self, key: str) -> bool:
        mcr = self.units["M"].regs.get(0x00, 0)
        return bool(mcr >> (16 + UNITS.index(key)) & 1)

    def _trg_regs(self):
        return [self.common.regs.get(0x3C + 4 * i, 0) for i in range(4)]

    def _trg_mask(self, key: str) -> int:
        mask = 0
        for i, reg in enumerate(self._trg_regs()):
            bits = (_TRG_BITS_13 if i in (0, 2) else _TRG_BITS_24).get(key, ())
            for b in bits:
                if b is not None and reg >> b & 1:
                    mask |= 1 << i
        return mask

    def trigger_mask(self, key: str) -> int:
        """ADC triggers (bit i = ADC(i+1)R) unit ``key`` is a source of.

        Cached per value of the four trigger registers: they are set once at
        start-up, and this runs every period."""
        r = self.common.regs
        state = (r.get(0x3C, 0), r.get(0x40, 0), r.get(0x44, 0), r.get(0x48, 0))
        if state != self._mask_state:
            self._mask_state = state
            self._masks = {k: self._trg_mask(k) for k in UNITS}
        return self._masks[key]

    def fire_triggers(self, mask: int) -> None:
        listeners = self.adc_listeners
        for i in range(4):
            if mask >> i & 1:
                for fn in listeners:
                    fn(i + 1)

    def listened(self, u: HrtimUnit) -> bool:
        return bool(u.regs.get(0x0C, 0) & 0x7F) or bool(self.trigger_mask(u.key)) or bool(self.period_listeners)

    def update(self) -> None:
        for u in self.units.values():
            if self._active and u.running() and self.listened(u):
                u.start()
            else:
                u.stop()

    def unit_period(self, u: HrtimUnit) -> None:
        mask = self.trigger_mask(u.key)
        if mask:
            self.fire_triggers(mask)
        for fn in self.period_listeners:
            fn(u)


__all__ = ["Hrtim", "HrtimUnit"]
