"""F2803x ADC (type 3, SOC based) and the analog comparators."""
from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

from .machine import C28xPeripheral

log = logging.getLogger("emu.f2803x")

#: ADCRESULT0-15: read-only registers, kept as a RAM shadow that the ADC
#: writes (the CPU and the CLA read them like memory)
RESULT_BASE = 0x0B00

#: SOC trigger sources (ADCSOCxCTL.TRIGSEL)
TRIG_SW, TRIG_TINT0, TRIG_TINT1, TRIG_TINT2, TRIG_XINT2 = 0, 1, 2, 3, 4


def trig_epwm(n: int, b: bool) -> int:
    """TRIGSEL of ePWMn SOCA (b=False) or SOCB."""
    return 5 + 2 * (n - 1) + (1 if b else 0)


class AdcResult(C28xPeripheral):
    """ADCRESULT0-15 (0x0B00), read-only to the CPU (and the CLA)."""

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.adc: Optional["Adc"] = None
        self.reg_names = {i: f"ADCRESULT{i}" for i in range(16)}

    def read(self, off):
        return self.adc.result[off] if off < 16 else 0

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        pass


class Adc(C28xPeripheral):
    """The ADC (0x7100): 16 SOCs, each with a trigger, a channel and an
    acquisition window; round-robin conversion; ADCINT1-9 from EOC events.

    Conversions take (ACQPS + 1 + 13) ADC clocks each.  A batch of triggered
    SOCs is converted back to back; each result is sampled from
    :attr:`source` (``fn(channel) -> 12-bit``, channels 0-7 = A0-A7, 8-15 =
    B0-B7) when its conversion starts."""

    reg_names = {0x0: "ADCCTL1", 0x1: "ADCCTL2", 0x4: "ADCINTFLG", 0x5: "ADCINTFLGCLR", 0x6: "ADCINTOVF",
                 0x7: "ADCINTOVFCLR", 0x8: "INTSEL1N2", 0x9: "INTSEL3N4", 0xA: "INTSEL5N6", 0xB: "INTSEL7N8",
                 0xC: "INTSEL9N10", 0x10: "SOCPRICTL", 0x12: "ADCSAMPLEMODE", 0x14: "ADCINTSOCSEL1",
                 0x15: "ADCINTSOCSEL2", 0x18: "ADCSOCFLG1", 0x1A: "ADCSOCFRC1", 0x1C: "ADCSOCOVF1",
                 0x1E: "ADCSOCOVFCLR1", **{0x20 + i: f"ADCSOC{i}CTL" for i in range(16)}, 0x40: "ADCREFTRIM",
                 0x41: "ADCOFFTRIM", 0x4C: "COMPHYSTCTL", 0x4F: "ADCREV"}
    protected = False   # (most registers are EALLOW protected; the firmware sets EALLOW anyway)

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.result = [0] * 16
        self.socctl = [0] * 16
        self.pending = 0          # ADCSOCFLG1
        self.intflg = 0
        self.intovf = 0
        self.rr = 15              # round-robin pointer: last converted SOC
        self.busy_until = 0
        self._ev = None
        #: analog input: fn(channel 0..15) -> raw 12-bit value
        self.source: Callable[[int], int] = lambda ch: 0
        #: listeners for ADCINTn: fn(n) (the CLA's task triggers)
        self.int_listeners: List[Callable[[int], None]] = []
        self.conversions = 0
        self._eoc_map = None
        self._trig_map = None
        #: the internal temperature sensor's reading
        self.temp_raw = 1700
        #: ADCINT -> (PIE group, index): ADCINT1/2 on 1.1/1.2 (and 10.1/10.2), 9 on 1.6, 3-8 on 10.3-10.8
        self.pie_map = {1: (1, 1), 2: (1, 2), 9: (1, 6), 3: (10, 3), 4: (10, 4), 5: (10, 5), 6: (10, 6),
                        7: (10, 7), 8: (10, 8)}

    def reset(self):
        super().reset()
        self._eoc_map = None
        self._trig_map = None
        self.result = [0] * 16
        self.m.cpu.mem[RESULT_BASE:RESULT_BASE + 16] = __import__("array").array("H", bytes(32))
        self.socctl = [0] * 16
        self.pending = self.intflg = self.intovf = 0
        self.rr = 15
        self.regs[0x0] = 0x4000   # ADCENABLE reads 1
        self.regs[0x4F] = 0x0003
        if self._ev:
            self._ev.cancel()
            self._ev = None

    # --- triggers ---------------------------------------------------------------
    def trigger(self, src: int) -> None:
        """A trigger source fired (see TRIG_*): flag every SOC listening to it."""
        tm = self._trig_map
        if tm is None:
            tm = self._trig_map = {}
            for i, ctl in enumerate(self.socctl):
                tm[ctl >> 11] = tm.get(ctl >> 11, 0) | 1 << i
        hit = tm.get(src, 0)
        if hit:
            self._flag(hit)

    def _flag(self, bits: int) -> None:
        self.pending |= bits
        if self._ev is None:
            now = self.m.clock.cycles
            if self.busy_until <= now:
                # idle: sample now; the results and EOCs come when the batch is done
                self._convert()
            else:
                self._ev = self.m.clock.at(self.busy_until, self._convert, "adc")

    def _clk_div(self) -> int:
        return 2 if self.r(0x1) & 1 else 1

    def _convert(self) -> None:
        """Convert everything pending, back to back, in round-robin order."""
        self._ev = None
        t = 0
        regs = self.regs
        div = 2 if regs.get(0x1, 0) & 1 else 1
        ctl1 = regs.get(0x0, 0)
        off = regs.get(0x41, 0) & 0x1FF
        if off & 0x100:
            off -= 0x200
        eocs = []
        pend = self.pending
        socctl = self.socctl
        source = self.source
        result = self.result
        mem = self.m.cpu.mem
        start = self.rr + 1
        for k in range(16):
            i = (start + k) & 15
            if not pend >> i & 1:
                continue
            ctl = socctl[i]
            ch = (ctl >> 6) & 0xF
            if ch == 13 and ctl1 & 0x0002:     # VREFLOCONV: B5 reads VREFLO
                v = 0
            elif ch == 5 and ctl1 & 0x0001:    # TEMPCONV: A5 reads the temperature sensor
                v = self.temp_raw
            else:
                v = source(ch)
            v = int(v) + off
            v = 0 if v < 0 else (4095 if v > 4095 else v)
            result[i] = v
            mem[RESULT_BASE + i] = v
            self.rr = i
            t += ((ctl & 0x3F) + 14) * div
            eocs.append(i)
        self.conversions += len(eocs)
        self.pending = 0
        if not eocs:
            return
        self.busy_until = self.m.clock.cycles + max(1, t)
        self.m.clock.at(self.busy_until, lambda e=eocs: self._eoc(e), "adc-eoc")

    def _int_map(self):
        """EOC number -> [(ADCINT n, continuous)] from INTSELxNy."""
        m = self._eoc_map
        if m is None:
            m = self._eoc_map = {}
            for n in range(1, 10):
                isel = self.r(0x8 + (n - 1) // 2)
                byte = (isel >> 8) if n % 2 == 0 else isel & 0xFF
                if byte & 0x20:
                    m.setdefault(byte & 0x1F, []).append((n, bool(byte & 0x40)))
        return m

    def _eoc(self, eocs) -> None:
        imap = self._int_map()
        for i in eocs:
            for n, cont in imap.get(i, ()):
                bit = 1 << (n - 1)
                if self.intflg & bit:
                    self.intovf |= bit
                    if not cont:   # no pulse until the flag is cleared
                        continue
                self.intflg |= bit
                g = self.pie_map.get(n)
                if g:
                    self.m.pie.raise_irq(*g)
                    if n in (1, 2):
                        self.m.pie.raise_irq(10, n)
                for fn in self.int_listeners:
                    fn(n)
                if n in (1, 2):   # ADCINT1/2 pulses start the SOCs that select them
                    socsel = self.r(0x14) | (self.r(0x15) << 16)
                    start = 0
                    for j in range(16):
                        if socsel >> (2 * j) & 3 == n:
                            start |= 1 << j
                    if start:
                        self._flag(start)

    # --- registers --------------------------------------------------------------
    def read(self, off):
        if off == 0x0:
            return (self.r(0x0) & 0x40FF) | 0x4000
        if off == 0x4:
            return self.intflg
        if off == 0x6:
            return self.intovf
        if off == 0x18:
            return self.pending
        if 0x20 <= off < 0x30:
            return self.socctl[off - 0x20]
        if off in (0x5, 0x7, 0x1A, 0x1E):
            return 0
        return super().read(off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        if off == 0x0:
            if v & 0x8000:
                self.reset()
                return
            self.regs[0] = v & 0x40FF
            return
        if off == 0x5:
            self.intflg &= ~v
            return
        if off == 0x7:
            self.intovf &= ~v
            return
        if off == 0x1A:
            if v:
                self._flag(v)
            return
        if off == 0x1E:
            return
        if 0x20 <= off < 0x30:
            self.socctl[off - 0x20] = v
            self._trig_map = None
            return
        if 0x8 <= off <= 0xC:
            self._eoc_map = None
        super().write(off, v)


class Comparator(C28xPeripheral):
    """Analog comparator with 10-bit DAC (0x6400/0x6420/0x6440).  COMPSTS
    follows :attr:`level` (set by the plant)."""

    reg_names = {0: "COMPCTL", 2: "COMPSTS", 4: "DACCTL", 6: "DACVAL"}

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.level = 0

    def read(self, off):
        if off == 2:
            return self.level & 1
        return super().read(off)
