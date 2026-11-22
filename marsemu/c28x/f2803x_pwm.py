"""F2803x ePWM (time base, counter compare, action qualifier, event trigger,
trip zone) and eCAP.

The counter is computed from the time of its last turning point (zero or
period); an event at every turning point loads the shadow registers, fires
the event trigger (interrupt, ADC start of conversion) and passes the sync
pulse down the chain ePWM1 -> ePWM2 -> ... -> ePWM7.  The outputs are not
simulated edge by edge: :meth:`Epwm.duty` gives each output's duty cycle
from the compare values and action-qualifier settings, which is what the
averaged power-stage models need.
"""
from __future__ import annotations

import logging
from typing import Callable, List, Optional

from .f2803x_adc import trig_epwm
from .machine import C28xPeripheral

log = logging.getLogger("emu.f2803x")

UP, DOWN, UPDOWN, FREEZE = 0, 1, 2, 3


class Epwm(C28xPeripheral):
    reg_names = {0x00: "TBCTL", 0x01: "TBSTS", 0x02: "TBPHSHR", 0x03: "TBPHS", 0x04: "TBCTR", 0x05: "TBPRD",
                 0x06: "TBPRDHR", 0x07: "CMPCTL", 0x08: "CMPAHR", 0x09: "CMPA", 0x0A: "CMPB", 0x0B: "AQCTLA",
                 0x0C: "AQCTLB", 0x0D: "AQSFRC", 0x0E: "AQCSFRC", 0x0F: "DBCTL", 0x10: "DBRED", 0x11: "DBFED",
                 0x12: "TZSEL", 0x13: "TZDCSEL", 0x14: "TZCTL", 0x15: "TZEINT", 0x16: "TZFLG", 0x17: "TZCLR",
                 0x18: "TZFRC", 0x19: "ETSEL", 0x1A: "ETPS", 0x1B: "ETFLG", 0x1C: "ETCLR", 0x1D: "ETFRC",
                 0x1E: "PCCTL", 0x20: "HRCNFG", 0x21: "HRPWR", 0x26: "HRMSTEP", 0x28: "HRPCTL", 0x2A: "TBPRDHRM",
                 0x2B: "TBPRDM", 0x2C: "CMPAHRM", 0x2D: "CMPAM", 0x30: "DCTRIPSEL", 0x31: "DCACTL", 0x32: "DCBCTL",
                 0x33: "DCFCTL", 0x34: "DCCAPCTL", 0x35: "DCFOFFSET", 0x36: "DCFOFFSETCNT", 0x37: "DCFWINDOW",
                 0x38: "DCFWINDOWCNT", 0x39: "DCCAP"}

    def __init__(self, m, name, base, size, index: int = 1):
        super().__init__(m, name, base, size)
        self.index = index
        self.next_in_chain: Optional["Epwm"] = None
        self.tz_listeners: List[Callable] = []
        #: called at every turning point: fn(epwm, "zero"|"prd")
        self.boundary_listeners: List[Callable] = []
        self._ev = None
        self._reset_state()

    def _reset_state(self):
        self.prd = 0
        self.prd_shadow = 0
        self.cmpa = self.cmpa_sh = 0
        self.cmpb = self.cmpb_sh = 0
        self.cmpa_pend = self.cmpb_pend = False
        self.prd_pend = False
        self.anchor_t = self.m.clock.cycles
        self.anchor_ctr = 0
        self.dir = 1
        self.et_int_cnt = self.et_soca_cnt = self.et_socb_cnt = 0
        self.etflg = 0
        self.tzflg = 0
        self.tz_forced = False
        self.periods = 0
        self._sync_state = None
        self._idle = True
        self.ints = 0
        self.socas = 0

    def reset(self):
        super().reset()
        if self._ev:
            self._ev.cancel()
        self._ev = None
        self._reset_state()
        self.regs[0x00] = 0x0083   # TBCTL: CTRMODE=3 (stopped), HSPCLKDIV=/2? (reset 0x0083)

    # ---------------------------------------------------------------- time base
    def _tbclk_div(self) -> int:
        t = self.r(0x00)
        hsp = (t >> 7) & 7
        clk = (t >> 10) & 7
        return (2 * hsp if hsp else 1) * (1 << clk)

    def mode(self) -> int:
        return self.r(0x00) & 3

    def counter(self) -> int:
        """TBCTR now."""
        self._catch_up()
        mode = self.mode()
        if mode == FREEZE:
            return self.anchor_ctr
        ticks = (self.m.clock.cycles - self.anchor_t) // self._tbclk_div()
        prd = self.prd
        c = self.anchor_ctr
        if mode == UP:
            if prd == 0:
                return 0
            return (c + ticks) % (prd + 1)
        if mode == DOWN:
            if prd == 0:
                return 0
            return (c - ticks) % (prd + 1)
        # up-down
        if prd == 0:
            return 0
        pos = (c if self.dir > 0 else 2 * prd - c) + ticks
        pos %= 2 * prd
        return pos if pos <= prd else 2 * prd - pos

    def direction(self) -> int:
        mode = self.mode()
        if mode == UP:
            return 1
        if mode == DOWN:
            return -1
        if mode == FREEZE or self.prd == 0:
            return self.dir
        ticks = (self.m.clock.cycles - self.anchor_t) // self._tbclk_div()
        pos = ((self.anchor_ctr if self.dir > 0 else 2 * self.prd - self.anchor_ctr) + ticks) % (2 * self.prd)
        return 1 if pos < self.prd else -1

    def period_cycles(self) -> int:
        """Length of one PWM period in SYSCLKOUT cycles."""
        div = self._tbclk_div()
        if self.mode() == UPDOWN:
            return 2 * self.prd * div
        return (self.prd + 1) * div

    def frequency(self) -> float:
        pc = self.period_cycles()
        return self.m.clock.hz / pc if pc and self.mode() != FREEZE else 0.0

    def _restart(self, ctr: int, direction: int) -> None:
        self.anchor_t = self.m.clock.cycles
        self.anchor_ctr = ctr
        self.dir = direction
        self._schedule()

    def _needs_events(self) -> bool:
        """Turning points need events only for the event trigger, boundary
        listeners and a sync output whose slave is not in step yet.  Shadow
        loads are applied lazily (see :meth:`_catch_up`)."""
        if self.boundary_listeners:
            return True
        regs = self.regs
        etsel, etps = regs.get(0x19, 0), regs.get(0x1A, 0)
        if (etsel & 0x8 and etps & 3) or (etsel & 0x800 and etps & 0x300) or (etsel & 0x8000 and etps & 0x3000):
            return True
        if (regs.get(0x00, 0) >> 4) & 3 == 1:
            nxt = self.next_in_chain
            while nxt is not None:
                if nxt.regs.get(0x00, 0) & 0x4 and nxt._sync_state != (self.period_cycles(), nxt._sync_key()):
                    return True
                if (nxt.regs.get(0x00, 0) >> 4) & 3 != 0:
                    break
                nxt = nxt.next_in_chain
        return False

    def _sync_key(self):
        return (self.period_cycles(), self.regs.get(0x03, 0), self.regs.get(0x00, 0) & 0x2007)

    def _next_turn(self):
        """(cycle, kind) of the next turning point after the anchor."""
        mode = self.mode()
        if mode == FREEZE or self.prd == 0:
            return None, None
        c = self.anchor_ctr
        prd = self.prd
        if mode == UP:
            if c >= prd:
                dt, kind = prd + 1 - c, "zero"
            else:
                dt, kind = prd - c, "prd"
        elif mode == DOWN:
            if c == 0:
                dt, kind = 1, "prd"   # wraps to PRD
            else:
                dt, kind = c, "zero"
        else:
            if self.dir > 0:
                dt, kind = prd - c, "prd"
            else:
                dt, kind = c, "zero"
        if dt <= 0:
            dt = 1 if mode != UPDOWN else 2 * prd
            kind = "zero" if kind == "prd" else "prd"
        return self.anchor_t + dt * self._tbclk_div(), kind

    def _catch_up(self) -> None:
        """Apply the shadow loads of the turning points that went by while
        no events ran."""
        if self._ev is not None or not (self.cmpa_pend or self.cmpb_pend or self.prd_pend):
            return
        now = self.m.clock.cycles
        for _ in range(8):
            t, kind = self._next_turn()
            if t is None or t > now:
                return
            self._turn(t, kind)
            if not (self.cmpa_pend or self.cmpb_pend or self.prd_pend):
                return
        # far behind: the loads happened long ago; re-anchor from the counter
        self._apply_loads("zero")
        self._apply_loads("prd")

    def _turn(self, t: int, kind: str) -> None:
        """Move the anchor to the turning point at ``t`` and do its shadow loads."""
        self.anchor_t = t
        if kind == "zero":
            self.anchor_ctr = 0
            self.dir = 1
            self.periods += 1
        else:
            self.anchor_ctr = self.prd
            if self.mode() == UPDOWN:
                self.dir = -1
        self._apply_loads(kind)

    def _apply_loads(self, kind: str) -> None:
        cmpctl = self.regs.get(0x07, 0)
        lda, ldb = cmpctl & 3, (cmpctl >> 2) & 3
        zero = kind == "zero"
        if self.cmpa_pend and (lda == 2 or (lda == 0 and zero) or (lda == 1 and not zero)):
            self.cmpa = self.cmpa_sh
            self.cmpa_pend = False
        if self.cmpb_pend and (ldb == 2 or (ldb == 0 and zero) or (ldb == 1 and not zero)):
            self.cmpb = self.cmpb_sh
            self.cmpb_pend = False
        if self.prd_pend and zero:
            self.prd = self.prd_shadow
            self.prd_pend = False

    def _schedule(self) -> None:
        if self._ev:
            self._ev.cancel()
            self._ev = None
        mode = self.mode()
        if mode == FREEZE or self.prd == 0:
            return
        if not self._needs_events():
            self._idle = True
            return
        self._idle = False
        div = self._tbclk_div()
        c = self.anchor_ctr
        prd = self.prd
        if mode == UP:
            if c >= prd:
                dt, kind = prd + 1 - c, "zero"
            else:
                dt, kind = prd - c, "prd"
        elif mode == DOWN:
            if c == 0:
                dt, kind = 1, "prd"   # wraps to PRD
            else:
                dt, kind = c, "zero"
        else:
            if self.dir > 0:
                dt, kind = prd - c, "prd"
            else:
                dt, kind = c, "zero"
        if dt <= 0:
            dt = 1 if mode != UPDOWN else 2 * prd
            kind = "zero" if kind == "prd" else "prd"
        self._ev = self.m.clock.at(self.anchor_t + dt * div, lambda k=kind: self._boundary(k),
                                   f"{self.name}-{kind}")

    def _boundary(self, kind: str) -> None:
        self._ev = None
        self._turn(self.m.clock.cycles, kind)
        zero = kind == "zero"
        self._event_trigger(kind)
        if zero:
            sel = (self.r(0x00) >> 4) & 3
            if sel == 1:
                self._sync_out()
        for fn in self.boundary_listeners:
            fn(self, kind)
        self._schedule()

    def _sync_out(self) -> None:
        nxt = self.next_in_chain
        while nxt is not None:
            nxt.sync_in()
            if (nxt.r(0x00) >> 4) & 3 != 0:   # not pass-through: the chain stops here
                break
            nxt = nxt.next_in_chain

    def sync_in(self) -> None:
        t = self.r(0x00)
        if not t & 0x4:   # PHSEN
            return
        self._catch_up()
        phs = self.r(0x03)
        direction = 1
        if self.mode() == UPDOWN:
            direction = 1 if t & 0x2000 else -1   # PHSDIR: 1 = count up after sync
        master = self._master()
        self._sync_state = (master.period_cycles() if master else 0, self._sync_key())
        self._restart(min(phs, self.prd) if self.prd else phs, direction)

    def _master(self) -> Optional["Epwm"]:
        for e in self.m.epwm[1:]:
            if e is not None and e.next_in_chain is self:
                return e
        return None

    # ---------------------------------------------------------- event trigger
    def _et_match(self, sel: int, kind: str) -> bool:
        return sel == 3 and kind in ("zero", "prd") or (sel == 1 and kind == "zero") or (sel == 2 and kind == "prd")

    def _event_trigger(self, kind: str) -> None:
        etsel = self.r(0x19)
        if not etsel & 0x8908:
            return
        etps = self.r(0x1A)
        if etsel & 0x8 and self._et_match(etsel & 7, kind):
            prd = etps & 3
            if prd:
                self.et_int_cnt += 1
                if self.et_int_cnt >= prd:
                    self.et_int_cnt = 0
                    if not self.etflg & 1:
                        self.etflg |= 1
                        self.ints += 1
                        self.m.pie.raise_irq(3, self.index)
                        self.m.cla.peripheral_irq("epwm", self.index)
        if etsel & 0x800 and self._et_match((etsel >> 8) & 7, kind):
            prd = (etps >> 8) & 3
            if prd:
                self.et_soca_cnt += 1
                if self.et_soca_cnt >= prd:
                    self.et_soca_cnt = 0
                    self.etflg |= 4
                    self.socas += 1
                    self.m.adc.trigger(trig_epwm(self.index, False))
        if etsel & 0x8000 and self._et_match((etsel >> 12) & 7, kind):
            prd = (etps >> 12) & 3
            if prd:
                self.et_socb_cnt += 1
                if self.et_socb_cnt >= prd:
                    self.et_socb_cnt = 0
                    self.etflg |= 8
                    self.m.adc.trigger(trig_epwm(self.index, True))

    # ----------------------------------------------------------- trip zone
    def trip(self, oneshot: bool = True) -> None:
        """A trip event (from a TZ pin or the digital compare): force the
        outputs per TZCTL, set the flags and interrupt if enabled."""
        self.tzflg |= 1 | (4 if oneshot else 2)
        self.tz_forced = True
        if self.r(0x15) & (4 if oneshot else 2):
            self.m.pie.raise_irq(2, self.index)
        for fn in self.tz_listeners:
            fn(self)

    def outputs_enabled(self) -> bool:
        return not self.tz_forced

    # ---------------------------------------------------------- outputs
    def duty(self, ch: str = "A") -> float:
        """Fraction of the period output ``ch`` ("A"/"B") is high, from the
        action qualifier and the compare values (ignoring dead band)."""
        self._catch_up()
        if self.tz_forced:
            tzctl = self.r(0x14)
            act = (tzctl >> (0 if ch == "A" else 2)) & 3
            return {1: 1.0, 2: 0.0}.get(act, 0.0)
        csfrc = self.r(0x0E) >> (0 if ch == "A" else 2) & 3
        if csfrc == 1:
            return 0.0
        if csfrc == 2:
            return 1.0
        prd = self.prd
        if prd == 0 or self.mode() == FREEZE:
            return 0.0
        aq = self.r(0x0B if ch == "A" else 0x0C)
        zro, prd_a, cau, cad, cbu, cbd = aq & 3, (aq >> 2) & 3, (aq >> 4) & 3, (aq >> 6) & 3, (aq >> 8) & 3, (aq >> 10) & 3
        ca, cb = self.cmpa, self.cmpb
        mode = self.mode()
        # walk one period as a list of (time in ticks, action)
        events = []
        if mode == UPDOWN:
            T = 2 * prd
            events.append((0, zro))
            if 0 < ca <= prd:
                events.append((ca, cau))
                events.append((T - ca, cad))
            if 0 < cb <= prd:
                events.append((cb, cbu))
                events.append((T - cb, cbd))
            events.append((prd, prd_a))
        else:
            T = prd + 1
            events.append((0, zro))
            if ca <= prd:
                events.append((ca, cau if mode == UP else cad))
            if cb <= prd:
                events.append((cb, cbu if mode == UP else cbd))
            events.append((prd, prd_a))
        events.sort(key=lambda e: e[0])
        # find the level at the start of the period: run twice
        level = 0
        for _ in range(2):
            hi_time = 0
            last_t = 0
            for t, act in events:
                if level:
                    hi_time += t - last_t
                last_t = t
                if act == 1:
                    level = 0
                elif act == 2:
                    level = 1
                elif act == 3:
                    level ^= 1
            if level:
                hi_time += T - last_t
        return hi_time / T

    # ---------------------------------------------------------- registers
    def read(self, off):
        self._catch_up()
        if off == 0x04:
            return self.counter()
        if off == 0x01:
            v = self.r(0x01) & 0x06
            return v | (1 if self.direction() > 0 else 0)
        if off == 0x05:
            return self.prd
        if off == 0x09:
            return self.cmpa
        if off == 0x0A:
            return self.cmpb
        if off == 0x07:
            return (self.r(0x07) & 0x5F) | (0x100 if self.cmpa_pend else 0) | (0x200 if self.cmpb_pend else 0)
        if off == 0x1B:
            return self.etflg
        if off == 0x16:
            return self.tzflg
        if off == 0x1A:
            v = self.r(0x1A) & 0x3333
            return v | (self.et_int_cnt & 3) << 2 | (self.et_soca_cnt & 3) << 10 | (self.et_socb_cnt & 3) << 14
        if off in (0x1C, 0x1D, 0x17, 0x18):
            return 0
        return super().read(off)

    def peek(self, off):
        return self.read(off)

    def _rearm(self) -> None:
        """Something that needs turning points was set up while the counter
        ran without events: continue from where it is now."""
        if self._ev is None and self.mode() != FREEZE and self.prd and self._needs_events():
            self._restart(self.counter(), self.direction())

    def write(self, off, v):
        self._catch_up()
        self._write(off, v)
        if off in (0x05, 0x09, 0x0A, 0x19, 0x1A, 0x00, 0x07):
            self._rearm()

    def _write(self, off, v):
        if off == 0x05:
            if self.r(0x00) & 0x8:   # PRDLD = 1: immediate
                self.prd = v
                self._restart(min(self.counter(), v), self.direction())
            else:
                if self.prd == 0 or self.mode() == FREEZE:
                    self.prd = v
                    self._schedule()
                self.prd_shadow = v
                self.prd_pend = True
            return
        if off == 0x09:
            if self.r(0x07) & 0x10:   # SHDWAMODE = 1: immediate
                self.cmpa = v
            else:
                self.cmpa_sh = v
                self.cmpa_pend = True
                if (self.r(0x07) & 3) == 3:
                    pass
            return
        if off == 0x0A:
            if self.r(0x07) & 0x40:
                self.cmpb = v
            else:
                self.cmpb_sh = v
                self.cmpb_pend = True
            return
        if off == 0x04:
            self._restart(v, self.direction())
            return
        if off == 0x00:
            old = self.r(0x00)
            ctr = self.counter()
            d = self.direction()
            self.regs[0x00] = v & ~0x40
            if v & 0x40:   # SWFSYNC
                self.sync_in()
            if (old ^ v) & 0x1F83 or True:
                self._restart(ctr, d)
            return
        if off == 0x1C:   # ETCLR
            self.etflg &= ~v
            return
        if off == 0x1D:   # ETFRC
            if v & 1 and not self.etflg & 1:
                self.etflg |= 1
                self.m.pie.raise_irq(3, self.index)
            if v & 4:
                self.etflg |= 4
                self.m.adc.trigger(trig_epwm(self.index, False))
            if v & 8:
                self.etflg |= 8
                self.m.adc.trigger(trig_epwm(self.index, True))
            return
        if off == 0x17:   # TZCLR
            self.tzflg &= ~v
            if v & 4:   # OST cleared: outputs released
                self.tz_forced = False
            if not self.tzflg & 6:
                self.tzflg &= ~1
            return
        if off == 0x18:   # TZFRC
            if v & 6:
                self.trip(oneshot=bool(v & 4))
            return
        super().write(off, v)


class Ecap(C28xPeripheral):
    """eCAP1 (0x6A00).  In APWM mode its counter is modelled; capture inputs
    are events the plant raises with :meth:`capture`."""

    reg_names = {0x00: "TSCTR", 0x01: "TSCTR_H", 0x02: "CTRPHS", 0x03: "CTRPHS_H", 0x04: "CAP1", 0x05: "CAP1_H",
                 0x06: "CAP2", 0x07: "CAP2_H", 0x08: "CAP3", 0x09: "CAP3_H", 0x0A: "CAP4", 0x0B: "CAP4_H",
                 0x14: "ECCTL1", 0x15: "ECCTL2", 0x16: "ECEINT", 0x17: "ECFLG", 0x18: "ECCLR", 0x19: "ECFRC"}

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.start = 0
        self.caps = [0, 0, 0, 0]
        self.ptr = 0
        self.flg = 0

    def reset(self):
        super().reset()
        self.start = self.m.clock.cycles
        self.caps = [0, 0, 0, 0]
        self.ptr = 0
        self.flg = 0

    def tsctr(self) -> int:
        return (self.m.clock.cycles - self.start) & 0xFFFFFFFF

    def edge(self, rising: bool) -> None:
        """An edge on the capture input (ECAP1 pin).  Captured when it has the
        polarity the current capture stage waits for (CAPxPOL: 0 = rising)."""
        ctl2 = self.r(0x15)
        if not ctl2 & 0x10 or ctl2 & 0x200:   # counter stopped, or APWM mode
            return
        ctl1 = self.r(0x14)
        evt = self.ptr + 1
        pol_falling = (ctl1 >> (2 * (evt - 1))) & 1
        if bool(pol_falling) == rising:
            return
        now = self.tsctr()
        if ctl1 & 0x100:   # CAPLDEN
            self.caps[self.ptr] = now
        if (ctl1 >> (2 * (evt - 1) + 1)) & 1:   # CTRRSTx: delta mode
            self.start = self.m.clock.cycles
        self.flg |= (1 << evt) | 1
        stop = (ctl2 >> 1) & 3
        if self.ptr >= stop:
            self.ptr = 0
        else:
            self.ptr += 1
        if self.r(0x16) & (1 << evt):
            self.m.pie.raise_irq(4, 1)

    def capture(self) -> None:
        """An edge of whatever polarity the current stage waits for."""
        ctl1 = self.r(0x14)
        self.edge(not ((ctl1 >> (2 * self.ptr)) & 1))

    def read(self, off):
        if off in (0, 1):
            v = self.tsctr()
            return v >> 16 if off else v & 0xFFFF
        if 4 <= off < 12:
            v = self.caps[(off - 4) // 2]
            return v >> 16 if off & 1 else v & 0xFFFF
        if off == 0x17:
            return self.flg
        if off in (0x18, 0x19):
            return 0
        return super().read(off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        if off == 0x18:
            self.flg &= ~v
            return
        if off in (0, 1):
            cur = self.tsctr()
            new = (cur & 0xFFFF0000) | v if off == 0 else (cur & 0xFFFF) | (v << 16)
            self.start = self.m.clock.cycles - new
            return
        if off == 0x15 and v & 0x20:   # SWSYNC
            self.start = self.m.clock.cycles
        if off == 0x15 and v & 0x08:   # REARM
            self.ptr = 0
        super().write(off, v)
