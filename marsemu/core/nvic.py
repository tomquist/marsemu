"""Cortex-M4 private peripheral bus: NVIC, SCB, SysTick, DWT, ITM, FPU, DBGMCU.

Unicorn ships the Cortex-M CPU core of QEMU but none of its system devices,
so the whole exception controller lives here.  The actual stacking and
unstacking of exception frames is done by :mod:`marsemu.core.machine`.
"""
from __future__ import annotations

import logging

from .bus import Peripheral

log = logging.getLogger("emu.nvic")

# exception numbers
NMI, HARDFAULT, MEMMANAGE, BUSFAULT, USAGEFAULT = 2, 3, 4, 5, 6
SVCALL, DEBUGMON, PENDSV, SYSTICK = 11, 12, 14, 15
EXC_NAMES = {1: "Reset", 2: "NMI", 3: "HardFault", 4: "MemManage", 5: "BusFault", 6: "UsageFault",
             11: "SVCall", 12: "DebugMon", 14: "PendSV", 15: "SysTick"}

NUM_EXC = 16 + 128


class Nvic(Peripheral):
    size = 0x100000  # 0xE0000000 .. 0xE00FFFFF

    def __init__(self, machine, base=0xE0000000):
        super().__init__(machine, "PPB", base)
        self.irq_names: dict[int, str] = {}
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        super().reset()
        self.enabled = [False] * NUM_EXC
        for e in (NMI, HARDFAULT, SVCALL, PENDSV, SYSTICK, DEBUGMON):
            self.enabled[e] = True
        self.pending = _Pending(self)
        self.active: list[int] = []  # stack of active exception numbers
        self.prio = [0] * NUM_EXC
        self.prio[1], self.prio[2], self.prio[3] = -3, -2, -1
        self.prigroup = 0
        self.vtor = 0
        self.shcsr = 0
        self.ccr = 0x200
        self.cpacr = 0
        self.cfsr = 0
        self.hfsr = 0
        self.scr = 0
        # SysTick
        self.st_ctrl = 0
        self.st_load = 0
        self.st_val_base = 0      # cycle at which counter was (re)loaded
        self.st_event = None
        self.st_countflag = False
        self.dwt_ctrl = 0x40000000
        self.demcr = 0
        self.itm_buf = bytearray()
        self.irq_count = [0] * NUM_EXC

    def name_of(self, exc: int) -> str:
        if exc < 16:
            return EXC_NAMES.get(exc, f"EXC{exc}")
        return self.irq_names.get(exc - 16, f"IRQ{exc - 16}")

    # ------------------------------------------------------------ priorities
    def group_prio(self, p: int) -> int:
        if p < 0:
            return p
        mask = (~((1 << (self.prigroup + 1)) - 1)) & 0xFF
        return p & mask

    def exec_priority(self, primask: int, basepri: int, faultmask: int) -> int:
        prio = 256
        for e in self.active:
            prio = min(prio, self.group_prio(self.prio[e]))
        if basepri & 0xFF:
            prio = min(prio, self.group_prio(basepri & 0xFF))
        if primask & 1:
            prio = min(prio, 0)
        if faultmask & 1:
            prio = min(prio, -1)
        return prio

    def set_pending(self, exc: int) -> None:
        self.pending[exc] = True
        self.m.wake()
        if self.m.running and self.enabled[exc]:
            self.m.preempt_check(exc)

    def clear_pending(self, exc: int) -> None:
        self.pending[exc] = False

    @property
    def pend_set(self):
        """Set of exceptions that are pending *and* enabled."""
        return [e for e in self.pending.s if self.enabled[e]] if self.pending.s else ()

    def best_pending(self, exec_prio: int):
        best = None
        bp = 1 << 30
        for e in self.pending.s:
            if self.enabled[e]:
                p = self.prio[e]
                if p < bp or (p == bp and e < best):
                    best, bp = e, p
        if best is None:
            return None
        if self.group_prio(bp) < exec_prio:
            return best
        return None

    def any_pending(self) -> bool:
        return bool(self.pend_set)

    # --------------------------------------------------------------- SysTick
    def _st_period(self) -> int:
        per = (self.st_load & 0xFFFFFF) + 1
        if not (self.st_ctrl & 4):  # external reference clock = HCLK/8
            per *= 8
        return per

    def _st_restart(self) -> None:
        if self.st_event:
            self.st_event.cancel()
            self.st_event = None
        if self.st_ctrl & 1 and self.st_load:
            self.st_val_base = self.m.clock.cycles
            self.st_event = self.m.clock.after(self._st_period(), self._st_fire, "systick")

    def _st_fire(self) -> None:
        self.st_countflag = True
        if self.st_ctrl & 2:
            self.set_pending(SYSTICK)
        self.st_val_base = self.m.clock.cycles
        self.st_event = self.m.clock.after(self._st_period(), self._st_fire, "systick")

    def _st_val(self) -> int:
        if not (self.st_ctrl & 1) or not self.st_load:
            return 0
        div = 1 if self.st_ctrl & 4 else 8
        elapsed = (self.m.clock.cycles - self.st_val_base) // div
        return (self.st_load & 0xFFFFFF) - (elapsed % ((self.st_load & 0xFFFFFF) + 1))

    # -------------------------------------------------------------- registers
    def read32(self, off: int) -> int:
        a = self.base + off
        if a == 0xE000E010:
            v = self.st_ctrl | (0x10000 if self.st_countflag else 0)
            self.st_countflag = False
            return v
        if a == 0xE000E014:
            return self.st_load
        if a == 0xE000E018:
            return self._st_val()
        if a == 0xE000E01C:
            return 0x40000000 | (self.m.clock.hz // 8 // 100)
        if 0xE000E100 <= a < 0xE000E180 or 0xE000E180 <= a < 0xE000E200:
            idx = ((a - 0xE000E100) % 0x80) // 4
            return self._bits(self.enabled, idx)
        if 0xE000E200 <= a < 0xE000E300:
            idx = ((a - 0xE000E200) % 0x80) // 4
            return self._bits(self.pending, idx)
        if 0xE000E300 <= a < 0xE000E380:
            idx = (a - 0xE000E300) // 4
            act = [False] * NUM_EXC
            for e in self.active:
                act[e] = True
            return self._bits(act, idx)
        if 0xE000E400 <= a < 0xE000E500:
            n = 16 + (a - 0xE000E400)
            return sum((self.prio[n + i] & 0xFF) << (8 * i) for i in range(4) if n + i < NUM_EXC)
        if a == 0xE000ED00:
            return 0x410FC241
        if a == 0xE000ED04:
            return self._icsr()
        if a == 0xE000ED08:
            return self.vtor
        if a == 0xE000ED0C:
            return 0xFA050000 | (self.prigroup << 8)
        if a == 0xE000ED10:
            return self.scr
        if a == 0xE000ED14:
            return self.ccr
        if 0xE000ED18 <= a < 0xE000ED24:
            n = 4 + (a - 0xE000ED18)
            return sum((self.prio[n + i] & 0xFF) << (8 * i) for i in range(4))
        if a == 0xE000ED24:
            v = self.shcsr & 0x70000
            for e in self.active:
                v |= {4: 1, 5: 2, 6: 8, 11: 0x80, 12: 0x100, 14: 0x400, 15: 0x800}.get(e, 0)
            return v
        if a == 0xE000ED28:
            return self.cfsr
        if a == 0xE000ED2C:
            return self.hfsr
        if a == 0xE000ED88:
            return self.cpacr
        if a == 0xE000EDFC:
            return self.demcr
        if a == 0xE0001000:
            return self.dwt_ctrl
        if a == 0xE0001004:
            return self.m.clock.cycles & 0xFFFFFFFF
        if 0xE0000000 <= a < 0xE0000080:  # ITM stimulus ports: always ready
            return 1
        if a == 0xE0042000:  # DBGMCU IDCODE
            return self.m.idcode
        return self.regs.get(off, 0)

    def write32(self, off: int, v: int) -> None:
        a = self.base + off
        if a == 0xE000E010:
            self.st_ctrl = v & 7
            self._st_restart()
        elif a == 0xE000E014:
            self.st_load = v & 0xFFFFFF
            if self.st_ctrl & 1:
                self._st_restart()
        elif a == 0xE000E018:
            self.st_countflag = False
            self._st_restart()
        elif 0xE000E100 <= a < 0xE000E180:
            self._setbits(self.enabled, (a - 0xE000E100) // 4, v, True)
            self.m.wake()
            if self.m.running and self.pending.s:
                self.m.preempt_check(None)
        elif 0xE000E180 <= a < 0xE000E200:
            self._setbits(self.enabled, (a - 0xE000E180) // 4, v, False)
        elif 0xE000E200 <= a < 0xE000E280:
            self._setbits(self.pending, (a - 0xE000E200) // 4, v, True)
            self.m.wake()
        elif 0xE000E280 <= a < 0xE000E300:
            self._setbits(self.pending, (a - 0xE000E280) // 4, v, False)
        elif 0xE000E400 <= a < 0xE000E500:
            n = 16 + (a - 0xE000E400)
            for i in range(4):
                if n + i < NUM_EXC:
                    self.prio[n + i] = (v >> (8 * i)) & 0xF0
        elif a == 0xE000ED04:
            if v & (1 << 28):
                self.set_pending(PENDSV)
            if v & (1 << 27):
                self.pending[PENDSV] = False
            if v & (1 << 26):
                self.set_pending(SYSTICK)
            if v & (1 << 25):
                self.pending[SYSTICK] = False
            if v & (1 << 31):
                self.set_pending(NMI)
        elif a == 0xE000ED08:
            self.vtor = v & 0xFFFFFF80
            log.info("VTOR = 0x%08x", self.vtor)
        elif a == 0xE000ED0C:
            if (v >> 16) == 0x05FA:
                self.prigroup = (v >> 8) & 7
                if v & 4:
                    log.warning("SYSRESETREQ from pc=0x%08x", self.m.pc)
                    self.m.request_reset("SYSRESETREQ")
        elif a == 0xE000ED10:
            self.scr = v
        elif a == 0xE000ED14:
            self.ccr = v
        elif 0xE000ED18 <= a < 0xE000ED24:
            n = 4 + (a - 0xE000ED18)
            for i in range(4):
                self.prio[n + i] = (v >> (8 * i)) & 0xF0
        elif a == 0xE000ED24:
            self.shcsr = v
            for bit, e in ((16, 4), (17, 5), (18, 6)):
                self.enabled[e] = bool(v & (1 << bit))
        elif a == 0xE000ED28:
            self.cfsr &= ~v
        elif a == 0xE000ED2C:
            self.hfsr &= ~v
        elif a == 0xE000ED88:
            self.cpacr = v
        elif a == 0xE000EDFC:
            self.demcr = v
        elif a == 0xE000EF00:  # STIR
            self.set_pending(16 + (v & 0x1FF))
        elif a == 0xE0001000:
            self.dwt_ctrl = v
        elif a == 0xE0001004:
            pass
        elif 0xE0000000 <= a < 0xE0000080:
            self._itm(v & 0xFF)
        else:
            self.regs[off] = v

    def write(self, off: int, size: int, value: int) -> None:
        a = self.base + off
        # byte-addressed priority registers
        if size == 1 and (0xE000E400 <= a < 0xE000E500 or 0xE000ED18 <= a < 0xE000ED24):
            n = 16 + (a - 0xE000E400) if a < 0xE000E500 else 4 + (a - 0xE000ED18)
            if n < NUM_EXC:
                self.prio[n] = value & 0xF0
            return
        if 0xE0000000 <= a < 0xE0000080:
            self._itm(value & 0xFF)
            return
        super().write(off, size, value)

    def _itm(self, ch: int) -> None:
        if ch == 10:
            self.m.console("itm", self.itm_buf.decode("latin1"))
            self.itm_buf.clear()
        elif ch != 13:
            self.itm_buf.append(ch)

    def _icsr(self) -> int:
        cur = self.active[-1] if self.active else 0
        v = cur & 0x1FF
        best = None
        for e in sorted(self.pending.s):
            if self.enabled[e]:
                if best is None or self.prio[e] < self.prio[best]:
                    best = e
        if best is not None:
            v |= (best & 0x1FF) << 12
            if best >= 16:
                v |= 1 << 22
        if len(self.active) <= 1:
            v |= 1 << 11
        if self.pending[PENDSV]:
            v |= 1 << 28
        if self.pending[SYSTICK]:
            v |= 1 << 26
        return v

    @staticmethod
    def _bits(arr, idx):
        base = 16 + idx * 32
        v = 0
        for i in range(32):
            if base + i < NUM_EXC and arr[base + i]:
                v |= 1 << i
        return v

    @staticmethod
    def _setbits(arr, idx, v, state):
        base = 16 + idx * 32
        for i in range(32):
            if v & (1 << i) and base + i < NUM_EXC:
                arr[base + i] = state


class _Pending:
    """Pending flags with list-like indexing, backed by a set for speed."""

    __slots__ = ("s", "nv")

    def __init__(self, nv):
        self.s: set = set()
        self.nv = nv

    def __getitem__(self, e: int) -> bool:
        return e in self.s

    def __setitem__(self, e: int, v: bool) -> None:
        if v:
            self.s.add(e)
        else:
            self.s.discard(e)

    def __iter__(self):
        return (e in self.s for e in range(NUM_EXC))
