"""F2803x system peripherals: clocks/PLL/watchdog, PIE, CPU timers, GPIO, XINT."""
from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

from .machine import C28xPeripheral

log = logging.getLogger("emu.f2803x")

OSCCLK = 10_000_000          # internal oscillator


class SysCtrl(C28xPeripheral):
    """Clocking, PLL and the watchdog (0x7010).  The PLL locks at once; the
    core clock of the virtual timeline stays at the machine's frequency."""

    reg_names = {0x0: "XCLK", 0x1: "PLLSTS", 0x2: "CLKCTL", 0x3: "PLLLOCKPRD", 0x4: "INTOSC1TRIM",
                 0x6: "INTOSC2TRIM", 0x9: "PCLKCR2", 0xB: "LOSPCP", 0xC: "PCLKCR0", 0xD: "PCLKCR1",
                 0xE: "LPMCR0", 0x10: "PCLKCR3", 0x11: "PLLCR", 0x12: "SCSR", 0x13: "WDCNTR", 0x15: "WDKEY",
                 0x19: "WDCR"}
    protected = True
    #: watchdog counter clock: OSCCLK / 512 / prescaler
    WD_BASE_HZ = OSCCLK / 512

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.reset_values = {0x1: 0x0001, 0x2: 0x6400, 0x3: 0xFFFF, 0xB: 0x0002, 0x11: 0, 0x12: 0x0001,
                             0x19: 0x0000}
        self.wd_enabled = True      # the emulator option; WDDIS still applies
        self._wd_ev = None
        self._wd_start = 0
        self._key = 0
        self.wd_resets = 0

    def reset(self):
        super().reset()
        self._key = 0
        self._wd_restart()

    # --- clocks ---------------------------------------------------------------
    @property
    def sysclk(self) -> int:
        return self.m.clock.hz

    @property
    def lspclk(self) -> int:
        div = self.r(0xB) & 7
        return self.sysclk // (2 * div) if div else self.sysclk

    # --- watchdog -------------------------------------------------------------
    def _wd_period_cycles(self) -> int:
        ps = self.r(0x19) & 7
        div = 1 if ps <= 1 else 1 << (ps - 1)
        return int(256 * div * self.m.clock.hz / self.WD_BASE_HZ)

    def _wd_restart(self):
        if self._wd_ev is not None:
            self._wd_ev.cancel()
            self._wd_ev = None
        self._wd_start = self.m.clock.cycles
        if not self.wd_enabled or self.r(0x19) & 0x40:
            return
        self._wd_ev = self.m.clock.after(self._wd_period_cycles(), self._wd_expire, "watchdog")

    def _wd_expire(self):
        self._wd_ev = None
        if self.r(0x12) & 2:   # WDENINT: interrupt instead of reset
            self.m.wake_nmi_wd()
            self._wd_restart()
            return
        self.wd_resets += 1
        self.m.request_reset("watchdog")

    def read(self, off):
        if off == 0x13:   # WDCNTR
            per = self._wd_period_cycles()
            return min(255, (self.m.clock.cycles - self._wd_start) * 256 // max(1, per))
        if off == 0x1:
            return self.r(0x1) | 1
        if off == 0x19:
            return self.r(0x19) & 0xBF | (self.r(0x19) & 0x40) | 0x80 * 0
        if off == 0x12:
            return self.r(0x12)
        return super().read(off)

    def write(self, off, v):
        if off == 0x15:   # WDKEY
            if v == 0x55:
                self._key = 0x55
            elif v == 0xAA and self._key == 0x55:
                self._key = 0
                self._wd_restart()
            else:
                self._key = 0
            return
        if off == 0x19:   # WDCR: WDCHK must be 101
            if (v >> 3) & 7 != 5:
                if self.wd_enabled:
                    log.warning("WDCR written with a bad WDCHK (0x%04x): watchdog reset", v)
                    self.m.request_reset("WDCR check bits")
                return
            self.regs[0x19] = v & 0x47
            self._wd_restart()
            return
        if off == 0x12:   # SCSR: WDOVERRIDE is write-1-to-clear
            cur = self.r(0x12)
            ov = cur & 1
            if v & 1:
                ov = 0
            self.regs[0x12] = ov | (v & 2)
            return
        if off == 0x1:    # PLLSTS: MCLKCLR etc.; PLL lock stays set
            self.regs[0x1] = (v & 0x0180) | 1
            return
        if off == 0x11:   # PLLCR: relock (instant)
            self.regs[0x11] = v & 0xF
            self.regs[0x1] = self.r(0x1) | 1
            return
        super().write(off, v)


class Pie(C28xPeripheral):
    """Peripheral interrupt expansion (0x0CE0): 12 groups of 8 interrupts
    multiplexed onto CPU INT1..INT12, vectors in the PIE RAM at 0x0D00."""

    reg_names = {0: "PIECTRL", 1: "PIEACK", **{2 + 2 * i: f"PIEIER{i + 1}" for i in range(12)},
                 **{3 + 2 * i: f"PIEIFR{i + 1}" for i in range(12)}}

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.ier = [0] * 13
        self.ifr = [0] * 13
        self.ack = 0
        self.ctrl = 0
        self.counts: Dict[tuple, int] = {}
        m.cpu.vector_hook = self.vector

    def reset(self):
        super().reset()
        self.ier = [0] * 13
        self.ifr = [0] * 13
        self.ack = 0
        self.ctrl = 0

    def raise_irq(self, group: int, idx: int) -> None:
        """Peripheral interrupt INTgroup.idx (both 1-based)."""
        self.ifr[group] |= 1 << (idx - 1)
        self._update(group)

    def clear_irq(self, group: int, idx: int) -> None:
        self.ifr[group] &= ~(1 << (idx - 1))

    def _update(self, g: int) -> None:
        if self.ifr[g] & self.ier[g] and not (self.ack >> (g - 1)) & 1:
            self.ack |= 1 << (g - 1)
            self.m.interrupt(g)

    def vector(self, k: int) -> int:
        cpu = self.m.cpu
        if not self.ctrl & 1:
            return cpu.rd32(0x3FFFC0 + 2 * k)
        if 1 <= k <= 12:
            pend = self.ifr[k] & self.ier[k]
            if pend:
                y = (pend & -pend).bit_length()
                self.ifr[k] &= ~(1 << (y - 1))
                self.counts[(k, y)] = self.counts.get((k, y), 0) + 1
                self.ctrl = (self.ctrl & 1) | ((0xD40 + ((k - 1) * 8 + y - 1) * 2) & 0xFFFE)
                return cpu.rd32(0xD40 + ((k - 1) * 8 + y - 1) * 2)
            # the flag was cleared before the CPU took it: PIE gives INTx.1's vector
            log.debug("INT%d taken with no PIE flag pending", k)
            return cpu.rd32(0xD40 + (k - 1) * 16)
        return cpu.rd32(0xD00 + 2 * k)

    def read(self, off):
        if off == 0:
            return self.ctrl
        if off == 1:
            return self.ack
        g = off // 2
        if 1 <= g <= 12:
            return self.ier[g] if off % 2 == 0 else self.ifr[g]
        return super().read(off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        if off == 0:
            self.ctrl = (self.ctrl & 0xFFFE) | (v & 1)
            return
        if off == 1:   # write 1 to clear
            self.ack &= ~v
            for g in range(1, 13):
                if v >> (g - 1) & 1:
                    self._update(g)
            return
        g = off // 2
        if 1 <= g <= 12:
            if off % 2 == 0:
                self.ier[g] = v & 0xFF
            else:
                self.ifr[g] = v & 0xFF
            self._update(g)
            return
        super().write(off, v)


class CpuTimers(C28xPeripheral):
    """CPU timers 0-2 (0x0C00): 32-bit down counters clocked by SYSCLKOUT /
    (TPR+1).  Timer 0 interrupts through PIE 1.7, timer 1 on INT13, timer 2
    on INT14."""

    reg_names = {}
    for _t in range(3):
        for _o, _n in ((0, "TIM"), (1, "TIMH"), (2, "PRD"), (3, "PRDH"), (4, "TCR"), (6, "TPR"), (7, "TPRH")):
            reg_names[_t * 8 + _o] = f"TIMER{_t}{_n}"

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.t = [dict(prd=0xFFFFFFFF, tpr=0, tcr=0x0001, start=0, run=True, base=0xFFFFFFFF, ev=None)
                  for _ in range(3)]

    def reset(self):
        super().reset()
        for i, t in enumerate(self.t):
            if t["ev"]:
                t["ev"].cancel()
            t.update(prd=0xFFFFFFFF, tpr=0, tcr=0x0001, start=self.m.clock.cycles, run=True, base=0xFFFFFFFF,
                     ev=None)
            self._schedule(i)

    def _div(self, t):
        """SYSCLKOUT cycles per count: TDDRH:TDDR + 1"""
        tpr = t["tpr"]
        return (((tpr >> 16) & 0xFF) << 8 | (tpr & 0xFF)) + 1

    def _count(self, i) -> int:
        t = self.t[i]
        if not t["run"]:
            return t["base"]
        ticks = (self.m.clock.cycles - t["start"]) // self._div(t)
        per = t["prd"] + 1
        if ticks <= t["base"]:
            return t["base"] - ticks
        return t["prd"] - ((ticks - t["base"] - 1) % per)

    def _schedule(self, i):
        t = self.t[i]
        if t["ev"]:
            t["ev"].cancel()
            t["ev"] = None
        if not t["run"]:
            return
        # next zero: the counter reaches 0 base ticks after start, then every prd+1
        div = self._div(t)
        ticks = (self.m.clock.cycles - t["start"]) // div
        if ticks <= t["base"]:
            nxt = t["base"] + 1
        else:
            per = t["prd"] + 1
            nxt = t["base"] + 1 + ((ticks - t["base"] - 1) // per + 1) * per
        when = t["start"] + nxt * div
        if nxt > 1 << 40:
            return
        t["ev"] = self.m.clock.at(when, lambda i=i: self._expire(i), f"cputimer{i}")

    def _expire(self, i):
        t = self.t[i]
        t["ev"] = None
        t["tcr"] |= 0x8000   # TIF
        if t["tcr"] & 0x4000:   # TIE
            if i == 0:
                self.m.pie.raise_irq(1, 7)
                self.m.cla.peripheral_irq("tint0", 0)
            else:
                self.m.interrupt(13 if i == 1 else 14)
        self._schedule(i)

    def read(self, off):
        i, r = divmod(off, 8)
        if i > 2:
            return super().read(off)
        t = self.t[i]
        if r == 0:
            return self._count(i) & 0xFFFF
        if r == 1:
            return self._count(i) >> 16
        if r == 2:
            return t["prd"] & 0xFFFF
        if r == 3:
            return t["prd"] >> 16
        if r == 4:
            return (t["tcr"] & ~0x10 & ~0x20) | (0 if t["run"] else 0x10)
        if r == 6:
            return t["tpr"] & 0xFFFF
        if r == 7:
            return t["tpr"] >> 16
        return super().read(off)

    def peek(self, off):
        return self.read(off)

    def _restart(self, i, count):
        t = self.t[i]
        t["base"] = count
        t["start"] = self.m.clock.cycles
        self._schedule(i)

    def write(self, off, v):
        i, r = divmod(off, 8)
        if i > 2:
            return super().write(off, v)
        t = self.t[i]
        if r == 2:
            t["prd"] = (t["prd"] & 0xFFFF0000) | v
        elif r == 3:
            t["prd"] = (t["prd"] & 0xFFFF) | (v << 16)
        elif r in (0, 1):
            cur = self._count(i)
            cur = (cur & 0xFFFF0000) | v if r == 0 else (cur & 0xFFFF) | (v << 16)
            self._restart(i, cur)
        elif r == 6:
            cur = self._count(i)
            t["tpr"] = (t["tpr"] & 0xFFFF0000) | v
            self._restart(i, cur)
        elif r == 7:
            cur = self._count(i)
            t["tpr"] = (t["tpr"] & 0xFFFF) | (v << 16)
            self._restart(i, cur)
        elif r == 4:
            tif = t["tcr"] & 0x8000
            if v & 0x8000:
                tif = 0
            cur = self._count(i)
            t["tcr"] = (v & 0x4C0F) | tif
            run = not (v & 0x10)
            if v & 0x20:   # TRB: reload
                cur = t["prd"]
            if run != t["run"]:
                t["run"] = run
            self._restart(i, cur)
        else:
            super().write(off, v)


class GpioPorts:
    """State shared by the GPIO control and data blocks."""

    def __init__(self, m):
        self.m = m
        self.out = [0, 0, 0]       # A, B, AIO output latches
        self.dir = [0, 0, 0]
        self.mux = [0, 0, 0, 0]    # GPAMUX1/2, GPBMUX1, AIOMUX1 (32 bits each)
        self.inputs: Dict[int, int] = {}
        #: level of inputs nobody drives (pull-ups are on by default on most pins)
        self.float_level = 1
        self.listeners: List[Callable[[int, int], None]] = []
        self.aio_inputs: Dict[int, int] = {}

    def level(self, port: int, pin: int) -> int:
        n = port * 32 + pin
        if self.dir[port] >> pin & 1:
            return self.out[port] >> pin & 1
        return self.inputs.get(n, self.float_level)

    def dat(self, port: int) -> int:
        v = 0
        for pin in range(32):
            v |= self.level(port, pin) << pin
        return v

    def set_out(self, port: int, new: int) -> None:
        old = self.out[port]
        self.out[port] = new & 0xFFFFFFFF
        ch = old ^ new
        if ch and self.listeners:
            for pin in range(32):
                if ch >> pin & 1:
                    for fn in self.listeners:
                        fn(port * 32 + pin, new >> pin & 1)

    def pin_out(self, n: int) -> int:
        """Output latch of GPIOn (port A/B) or AIO (64+n)."""
        return self.out[n // 32] >> (n % 32) & 1


class GpioCtrl(C28xPeripheral):
    protected = True
    reg_names = {0x0: "GPACTRL", 0x2: "GPAQSEL1", 0x4: "GPAQSEL2", 0x6: "GPAMUX1", 0x8: "GPAMUX2", 0xA: "GPADIR",
                 0xC: "GPAPUD", 0x10: "GPBCTRL", 0x12: "GPBQSEL1", 0x16: "GPBMUX1", 0x1A: "GPBDIR", 0x1C: "GPBPUD",
                 0x36: "AIOMUX1", 0x3A: "AIODIR"}
    reg_names.update({k + 1: v + "_H" for k, v in list(reg_names.items())})

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.ports: GpioPorts = m.gpio

    def _get32(self, off):
        lo = off & ~1
        if lo == 0xA:
            return self.ports.dir[0]
        if lo == 0x1A:
            return self.ports.dir[1]
        if lo == 0x3A:
            return self.ports.dir[2]
        if lo == 0x6:
            return self.ports.mux[0]
        if lo == 0x8:
            return self.ports.mux[1]
        if lo == 0x16:
            return self.ports.mux[2]
        if lo == 0x36:
            return self.ports.mux[3]
        return self.regs.get(lo, 0) | (self.regs.get(lo + 1, 0) << 16)

    def read(self, off):
        v = self._get32(off)
        return (v >> 16) if off & 1 else (v & 0xFFFF)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        lo = off & ~1
        cur = self._get32(off)
        new = (cur & 0xFFFF) | (v << 16) if off & 1 else (cur & 0xFFFF0000) | v
        p = self.ports
        if lo == 0xA:
            p.dir[0] = new
        elif lo == 0x1A:
            p.dir[1] = new
        elif lo == 0x3A:
            p.dir[2] = new
        elif lo == 0x6:
            p.mux[0] = new
        elif lo == 0x8:
            p.mux[1] = new
        elif lo == 0x16:
            p.mux[2] = new
        elif lo == 0x36:
            p.mux[3] = new
        else:
            self.regs[lo] = new & 0xFFFF
            self.regs[lo + 1] = new >> 16


class GpioData(C28xPeripheral):
    reg_names = {0x0: "GPADAT", 0x2: "GPASET", 0x4: "GPACLEAR", 0x6: "GPATOGGLE", 0x8: "GPBDAT", 0xA: "GPBSET",
                 0xC: "GPBCLEAR", 0xE: "GPBTOGGLE", 0x18: "AIODAT", 0x1A: "AIOSET", 0x1C: "AIOCLEAR",
                 0x1E: "AIOTOGGLE"}
    reg_names.update({k + 1: v + "_H" for k, v in list(reg_names.items())})

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.ports: GpioPorts = m.gpio

    @staticmethod
    def _port(off):
        lo = off & ~1
        if lo < 8:
            return 0, lo
        if lo < 0x10:
            return 1, lo - 8
        if 0x18 <= lo < 0x20:
            return 2, lo - 0x18
        return None, None

    def read(self, off):
        port, r = self._port(off)
        if port is None:
            return super().read(off)
        v = self.ports.dat(port) if r == 0 else 0
        return (v >> 16) if off & 1 else (v & 0xFFFF)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        port, r = self._port(off)
        if port is None:
            return super().write(off, v)
        p = self.ports
        bits = (v << 16) if off & 1 else v
        cur = p.out[port]
        if r == 0:
            new = (cur & 0xFFFF) | (v << 16) if off & 1 else (cur & 0xFFFF0000) | v
        elif r == 2:
            new = cur | bits
        elif r == 4:
            new = cur & ~bits
        else:
            new = cur ^ bits
        p.set_out(port, new)
