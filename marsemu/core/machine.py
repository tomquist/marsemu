"""The emulated microcontroller: CPU, memory map, exception model, run loop."""
from __future__ import annotations

import ctypes
from collections import deque
import logging
import struct
from typing import Callable, Dict, List, Optional

from unicorn import (UC_ARCH_ARM, UC_HOOK_INTR, UC_HOOK_MEM_FETCH_UNMAPPED, UC_HOOK_MEM_READ_UNMAPPED,
                     UC_HOOK_MEM_WRITE_UNMAPPED, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB, UC_PROT_ALL, Uc,
                     UcError)
from unicorn import arm_const as A

from unicorn.unicorn_py3.unicorn import uclib as _uclib

from .bus import Bus
from .clock import Clock
from .nvic import HARDFAULT, SVCALL, USAGEFAULT, Nvic
from .ucount import Counter

log = logging.getLogger("emu.cpu")

EXCP_UDEF, EXCP_SWI, EXCP_BKPT, EXCP_EXCEPTION_EXIT = 1, 2, 7, 8
EXCP_NOCP, EXCP_INVSTATE, EXCP_UNALIGNED = 17, 18, 22

FRAME_REGS = (A.UC_ARM_REG_R0, A.UC_ARM_REG_R1, A.UC_ARM_REG_R2, A.UC_ARM_REG_R3, A.UC_ARM_REG_R12, A.UC_ARM_REG_LR)
FP_REGS = tuple(getattr(A, f"UC_ARM_REG_S{i}") for i in range(16))
FP_FRAME_REGS = FP_REGS + (A.UC_ARM_REG_FPSCR,)
ENTRY_REGS = (A.UC_ARM_REG_IPSR, A.UC_ARM_REG_CONTROL, A.UC_ARM_REG_SP) + FRAME_REGS + (A.UC_ARM_REG_XPSR,)
IPSR_LR = (A.UC_ARM_REG_IPSR, A.UC_ARM_REG_LR)


class EmulationStopped(Exception):
    pass


class Machine:
    """A Cortex-M4 microcontroller emulated with Unicorn.

    The machine owns memory, the clock and the NVIC.  An SoC module
    (e.g. :mod:`marsemu.periph.n32g45x`) attaches the peripherals.
    """

    def __init__(self, *, flash_base=0x08000000, flash_size=512 * 1024, sram_base=0x20000000,
                 sram_size=256 * 1024, hz=144_000_000, slice_cycles=8000, cpi=2):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(A.UC_CPU_ARM_CORTEX_M4)
        self.clock = Clock(hz)
        self.slice_cycles = slice_cycles
        #: core cycles accounted per executed instruction (flash wait states etc.)
        self.cpi = cpi
        self._u32 = ctypes.c_uint32()
        self._u32p = ctypes.byref(self._u32)
        self._uch = self.uc._uch
        self._batches: dict = {}
        self.counter = Counter(self.uc)
        self.unmask_table: Dict[int, tuple] = {}
        self._exits_on = False
        self.unmask_hits = 0
        self.exit_toggles = 0
        self._exits_busy = 0
        self._exits_hold = hz // 20  # 50 ms
        self._exits_sticky = False
        self._exits_off_at = -(1 << 62)
        self._exits_enables: deque = deque()
        self._exits_on_at = (0, 0)
        self._exit_hit_rate = 0.0
        self.flash_base, self.flash_size = flash_base, flash_size
        self.sram_base, self.sram_size = sram_base, sram_size
        self.idcode = 0x10036414
        self.consoles: List[Callable[[str, str], None]] = []
        self.reset_hooks: List[Callable[[], None]] = []
        self.slice_hooks: List[Callable[[], None]] = []
        self.stop_reason: Optional[str] = None
        self._stop = False
        self._reset_req: Optional[str] = None
        self._sleeping = False
        self.exc_count = 0           # exceptions taken (for the idle-loop check)
        #: heads of wait loops that reload the IWDG every iteration (see set_idle_loop)
        self.watchdog_waits: set = set()
        self._idle_stop = False
        self.idle_sleeps = 0
        self.reset_count = 0
        self.fault: Optional[str] = None
        self.fault_policy = "hardfault"  # or "stop"
        self.exc_trace = False
        #: vector table used at reset (a bootloader would have set VTOR)
        self.boot_vtor = 0
        self.level_sources: Dict[int, list] = {}
        self.running = False
        self._early = False
        self.early_stops = 0
        self.early_stop_cycles = 128
        self._early_pc = None
        self._stall_req = False
        self._stalled = False
        self._stall_until = 0
        self._stall_sp = 0
        self._stall_resume = False

        # --- memory -------------------------------------------------------
        self.flash = ctypes.create_string_buffer(b"\xff" * flash_size, flash_size)
        self.uc.mem_map_ptr(flash_base, flash_size, UC_PROT_ALL, ctypes.addressof(self.flash))
        # boot alias at 0x00000000 (BOOT0=0 maps main flash there)
        self.uc.mem_map_ptr(0, flash_size, UC_PROT_ALL, ctypes.addressof(self.flash))
        self.uc.mem_map(sram_base, sram_size)
        self._auto_pages: set = set()

        self.bus = Bus(self)
        self.nvic = self.bus.attach(Nvic(self))
        self.bus.map_window(0xE0000000, 0x100000)

        self.uc.hook_add(UC_HOOK_INTR, self._on_intr)
        self.uc.hook_add(UC_HOOK_MEM_READ_UNMAPPED | UC_HOOK_MEM_WRITE_UNMAPPED, self._on_unmapped)
        self.uc.hook_add(UC_HOOK_MEM_FETCH_UNMAPPED, self._on_fetch_unmapped)

    # ------------------------------------------------------------- utilities
    def map_ram(self, base: int, size: int) -> None:
        self.uc.mem_map(base, size)

    def load_flash(self, image: bytes, offset: int = 0) -> None:
        ctypes.memmove(ctypes.addressof(self.flash) + offset, image, len(image))

    def flash_bytes(self) -> bytes:
        return self.flash.raw

    def rr(self, regid: int) -> int:
        """Fast 32-bit register read (bypasses unicorn's slow Python wrapper)."""
        _uclib.uc_reg_read(self._uch, regid, self._u32p)
        return self._u32.value

    def rw(self, regid: int, value: int) -> None:
        self._u32.value = value & 0xFFFFFFFF
        _uclib.uc_reg_write(self._uch, regid, self._u32p)

    def _batch(self, regs: tuple):
        b = self._batches.get(regs)
        if b is None:
            n = len(regs)
            ids = (ctypes.c_int * n)(*regs)
            buf = (ctypes.c_uint64 * n)()
            ptrs = (ctypes.c_void_p * n)(*[ctypes.addressof(buf) + 8 * i for i in range(n)])
            b = self._batches[regs] = (ids, buf, ptrs, n)
        return b

    def rr_many(self, regs: tuple) -> list:
        """Read several 32-bit registers in one call (``regs`` must be a tuple)."""
        ids, buf, ptrs, n = self._batch(regs)
        _uclib.uc_reg_read_batch(self._uch, ids, ptrs, n)
        return [v & 0xFFFFFFFF for v in buf]

    def rw_many(self, regs: tuple, values) -> None:
        ids, buf, ptrs, n = self._batch(regs)
        for i, v in enumerate(values):
            buf[i] = v & 0xFFFFFFFF
        _uclib.uc_reg_write_batch(self._uch, ids, ptrs, n)

    @property
    def pc(self) -> int:
        return self.rr(A.UC_ARM_REG_PC)

    def reg(self, name: str) -> int:
        return self.rr(getattr(A, "UC_ARM_REG_" + name.upper()))

    def set_reg(self, name: str, value: int) -> None:
        self.rw(getattr(A, "UC_ARM_REG_" + name.upper()), value)

    def read(self, addr: int, n: int) -> bytes:
        return bytes(self.uc.mem_read(addr, n))

    def write(self, addr: int, data: bytes) -> None:
        self.uc.mem_write(addr, bytes(data))

    def r32(self, addr: int) -> int:
        return struct.unpack("<I", self.read(addr, 4))[0]

    def w32(self, addr: int, v: int) -> None:
        self.write(addr, struct.pack("<I", v & 0xFFFFFFFF))

    def cstr(self, addr: int, maxlen: int = 256) -> str:
        out = bytearray()
        for i in range(maxlen):
            b = self.read(addr + i, 1)[0]
            if b == 0:
                break
            out.append(b)
        return out.decode("latin1")

    def console(self, channel: str, line: str) -> None:
        for c in self.consoles:
            c(channel, line)

    def wake(self) -> None:
        self._sleeping = False

    def stop(self, reason: str = "stopped") -> None:
        self.stop_reason = reason
        self._stop = True
        self.uc.emu_stop()

    def request_reset(self, why: str) -> None:
        self._reset_req = why
        self.uc.emu_stop()

    def advance(self, seconds: float) -> None:
        """Advance virtual time running only timed events (no CPU)."""
        end = self.clock.cycles + int(seconds * self.clock.hz)
        while True:
            for h in self.slice_hooks:
                h()
            nd = self.clock.next_deadline()
            if nd is None or nd > end:
                break
            self.clock.cycles = max(self.clock.cycles, nd)
            self.clock.run_due()
        self.clock.cycles = end

    def add_code_hook(self, addr: int, fn: Callable[["Machine"], None], end: Optional[int] = None):
        """Call ``fn(machine)`` whenever execution reaches ``addr`` (or the range)."""
        addr &= ~1
        end = end if end is not None else addr
        h = self.uc.hook_add(UC_HOOK_CODE, lambda uc, a, s, ud: fn(self), begin=addr, end=end)
        # code that was already translated would not see the new hook
        self.uc.ctl_flush_tb()
        return h

    def set_idle_loop(self, addr: int, feeds_watchdog: bool = False) -> None:
        """Treat reaching ``addr`` (the head of an idle or wait loop) like WFI.

        The first arrival after an exception lets one iteration run; arriving
        again with no exception in between means the iteration neither
        yielded nor was interrupted, so the CPU sleeps until the next event.
        Each loop keeps its own mark, so leaving one loop for another never
        skips the other's first iteration.

        ``feeds_watchdog``: the loop reloads the IWDG on every iteration, so a
        watchdog that expires while the CPU sleeps there counts as reloaded
        (see :attr:`watchdog_waits`)."""
        mark = [-1]

        def hit(m):
            if mark[0] == m.exc_count and not m.nvic.pend_set:
                m._idle_stop = True
                m.uc.emu_stop()
            else:
                mark[0] = m.exc_count
        if feeds_watchdog:
            self.watchdog_waits.add(addr)
        self.add_code_hook(addr, hit)

    def feeding_watchdog(self) -> bool:
        """Asleep in a loop that reloads the watchdog on every iteration."""
        return self._sleeping and bool(self.watchdog_waits) and self.pc in self.watchdog_waits

    # ---------------------------------------------------------------- reset
    def reset(self) -> None:
        self.reset_count += 1
        for p in self.bus.peripherals.values():
            p.reset()
        self.nvic.vtor = vt = self.boot_vtor
        sp = self.r32(vt)
        entry = self.r32(vt + 4)
        for r in range(13):
            self.uc.reg_write(A.UC_ARM_REG_R0 + r, 0)
        self.uc.reg_write(A.UC_ARM_REG_IPSR, 0)
        self.uc.reg_write(A.UC_ARM_REG_CONTROL, 0)
        self.uc.reg_write(A.UC_ARM_REG_PRIMASK, 0)
        self.uc.reg_write(A.UC_ARM_REG_BASEPRI, 0)
        self.uc.reg_write(A.UC_ARM_REG_FAULTMASK, 0)
        self.uc.reg_write(A.UC_ARM_REG_MSP, sp)
        self.uc.reg_write(A.UC_ARM_REG_SP, sp)
        self.uc.reg_write(A.UC_ARM_REG_LR, 0xFFFFFFFF)
        self.uc.reg_write(A.UC_ARM_REG_PC, entry | 1)
        self._sleeping = False
        self._stall_until = 0
        self._stalled = False
        for h in self.reset_hooks:
            h()
        log.info("reset #%d: SP=0x%08x PC=0x%08x", self.reset_count, sp, entry)

    # ----------------------------------------------------- exception model
    def _exec_priority(self) -> int:
        return self.nvic.exec_priority(self.rr(A.UC_ARM_REG_PRIMASK), self.rr(A.UC_ARM_REG_BASEPRI),
                                       self.rr(A.UC_ARM_REG_FAULTMASK))

    def _take(self, exc: int, return_pc: int) -> None:
        """Exception entry: stack a frame and branch to the vector."""
        self.exc_count += 1
        uc = self.uc
        nv = self.nvic
        r = self.rr_many(ENTRY_REGS)
        ipsr, control, sp = r[0], r[1], r[2]
        thread = ipsr == 0
        use_psp = thread and bool(control & 2)
        fpca = bool(control & 4)
        vals = r[3:9] + [return_pc & ~1, r[9] | (1 << 24)]
        frame_size = 0x68 if fpca else 0x20
        sp -= frame_size
        realign = bool(sp & 4)
        if realign:
            sp -= 4
        if realign:
            vals[7] |= 1 << 9
        else:
            vals[7] &= ~(1 << 9)
        data = struct.pack("<8I", *vals)
        if fpca:
            fp = self.rr_many(FP_FRAME_REGS) + [0]
            data += struct.pack("<18I", *fp)
        try:
            uc.mem_write(sp, data)
        except UcError:
            log.error("stack overflow while entering %s (sp=0x%08x)", nv.name_of(exc), sp)
            self.stop("stack fault")
            return
        self.rw(A.UC_ARM_REG_SP, sp)
        # handler mode always uses MSP; clear FPCA/SPSEL like the hardware does
        if control & 6:
            self.rw(A.UC_ARM_REG_CONTROL, control & ~6)
        exc_return = 0xFFFFFFE1 if fpca else 0xFFFFFFF1
        if thread:
            exc_return |= 0x8
            if use_psp:
                exc_return |= 0x4
        self.rw_many(IPSR_LR, (exc, exc_return))
        nv.pending[exc] = False
        nv.active.append(exc)
        nv.irq_count[exc] += 1
        vector = self.r32(nv.vtor + 4 * exc)
        if self.exc_trace:
            log.debug("-> %s (ret=0x%08x) vector 0x%08x", nv.name_of(exc), return_pc, vector)
        self.rw(A.UC_ARM_REG_PC, vector | 1)
        self._sleeping = False
        self._stalled = False

    def _exception_return(self, exc_return: int) -> None:
        nv = self.nvic
        if nv.active:
            done = nv.active.pop()
        else:
            log.warning("exception return with no active exception (0x%08x)", exc_return)
            done = 0
        for src in self.level_sources.get(done, ()):
            if src():
                nv.pending[done] = True
                break
        to_thread = bool(exc_return & 8)
        use_psp = bool(exc_return & 4)
        extended = not (exc_return & 0x10)
        if to_thread:
            self.rw(A.UC_ARM_REG_IPSR, 0)
            control = self.rr(A.UC_ARM_REG_CONTROL)
            if use_psp:
                control |= 2
            else:
                control &= ~2
            if extended:
                control |= 4
            self.rw(A.UC_ARM_REG_CONTROL, control)
        sp = self.rr(A.UC_ARM_REG_SP)
        raw = self.read(sp, 0x68 if extended else 0x20)
        vals = struct.unpack_from("<8I", raw)
        self.rw_many(FRAME_REGS, vals[:6])
        if extended:
            self.rw_many(FP_FRAME_REGS, struct.unpack_from("<17I", raw, 0x20))
        xpsr = vals[7]
        sp += 0x68 if extended else 0x20
        if xpsr & (1 << 9):
            sp += 4
        self.rw(A.UC_ARM_REG_SP, sp)
        if not to_thread:
            self.rw(A.UC_ARM_REG_IPSR, xpsr & 0x1FF)
            # the hardware sets CONTROL.FPCA = !EXC_RETURN[4] on every return; left
            # clear, the handler's next FP instruction makes the core open a new FP
            # context with FPSCR = FPDSCR, losing its condition flags
            control = self.rr(A.UC_ARM_REG_CONTROL)
            want = (control | 4) if extended else (control & ~4)
            if want != control:
                self.rw(A.UC_ARM_REG_CONTROL, want)
        self.rw(A.UC_ARM_REG_XPSR, (xpsr & ~0x1FF & ~(1 << 9)) | (1 << 24) | (0 if to_thread else (xpsr & 0x1FF)))
        self.rw(A.UC_ARM_REG_PC, vals[6] | 1)
        if self.exc_trace:
            log.debug("<- %s to 0x%08x", nv.name_of(done), vals[6])
        # tail-chain anything that became pending meanwhile
        nxt = nv.best_pending(self._exec_priority())
        if nxt is not None:
            self._take(nxt, vals[6])
        elif self._stall_until and sp == self._stall_sp and self.clock.cycles < self._stall_until:
            # back in code that is still waiting for a peripheral
            self._stall_resume = True
            self.uc.emu_stop()

    def _fault(self, kind: str, exc: int = HARDFAULT) -> None:
        pc = self.pc
        msg = f"{kind} at pc=0x{pc:08x} lr=0x{self.reg('LR'):08x} sp=0x{self.reg('SP'):08x}"
        self.fault = msg
        if self.fault_policy == "stop" or HARDFAULT in self.nvic.active:
            log.error("fatal: %s", msg)
            self.stop("fault: " + msg)
            return
        log.warning("fault: %s -> %s", msg, self.nvic.name_of(exc))
        self.nvic.hfsr |= 1 << 30
        self._take(exc if self.nvic.enabled[exc] else HARDFAULT, pc)

    def _on_intr(self, uc, intno, _ud):
        if intno == EXCP_EXCEPTION_EXIT:
            self._exception_return(self.pc | 1)
        elif intno == EXCP_SWI:
            self._take(SVCALL, self.pc)
        elif intno == EXCP_BKPT:
            pc = self.pc
            log.warning("BKPT at 0x%08x", pc)
            self.stop(f"bkpt at 0x{pc:08x}")
        elif intno == EXCP_NOCP:
            self._fault("NOCP (coprocessor)", USAGEFAULT)
        elif intno == EXCP_UDEF:
            self._fault("undefined instruction", USAGEFAULT)
        elif intno in (EXCP_INVSTATE, EXCP_UNALIGNED):
            self._fault(f"usage fault {intno}", USAGEFAULT)
        else:
            self._fault(f"cpu exception {intno}")

    def _on_unmapped(self, uc, access, addr, size, value, _ud):
        page = addr & ~0xFFF
        if page in self._auto_pages:
            return True
        if not (0x20000000 <= addr < 0x40000000 or 0x60000000 <= addr < 0xA0000000 or addr < 0x20000000):
            log.warning("bus error: %s 0x%08x at pc=0x%08x", "write" if access == 20 else "read", addr, self.pc)
        else:
            log.warning("access to unmapped 0x%08x at pc=0x%08x -> mapping zero page", addr, self.pc)
        try:
            uc.mem_map(page, 0x1000)
            self._auto_pages.add(page)
        except UcError:
            return False
        return True

    def _on_fetch_unmapped(self, uc, access, addr, size, value, _ud):
        if addr >= 0xFFFFFF00:
            # bx lr with EXC_RETURN while unicorn did not see handler mode
            self._exception_return(addr | 1)
            return True
        self._fault(f"instruction fetch from unmapped 0x{addr:08x}")
        return True

    # -------------------------------------------------------------- run loop
    def preempt_check(self, exc: Optional[int]) -> None:
        """Called while emulating when something became pending: if it can
        preempt the running code, stop at the next instruction boundary so
        it is taken right away (RTOS yields rely on this)."""
        if self._early:
            return
        # stop both when the exception can preempt now and when it is masked:
        # in the latter case the run loop steps to the instruction that
        # unmasks it
        # We are inside an MMIO callback.  Unicorn stops with PC still on the
        # accessing instruction even though the access already happened, so
        # only stop on stores (and step over them afterwards); a load would
        # have to be re-executed, which repeats its side effects.
        pc = self.rr(A.UC_ARM_REG_PC)
        size = self._store_size(pc)
        if size:
            self._early = True
            self._early_pc = (pc, size)
            self.uc.emu_stop()

    def stall(self, cycles: int) -> None:
        """The running code waits ``cycles`` for a peripheral (a polled bus
        transfer that the model completes at once).  Called from an MMIO store:
        the CPU sleeps until then, an interrupt wakes it early, so polling
        loops cost neither wall time nor a distorted poll rate."""
        if not self.running or self._early or cycles <= 0:
            return
        pc = self.rr(A.UC_ARM_REG_PC)
        size = self._store_size(pc)
        if not size:
            return
        self._early = True
        self._early_pc = (pc, size)
        self._stall_req = True
        # interrupts run during the wait; returning to this context waits on
        self._stall_until = self.clock.cycles + cycles
        self._stall_sp = self.rr(A.UC_ARM_REG_SP)
        self._stall_ev = self.clock.after(cycles, self._stall_end, "stall")
        self.uc.emu_stop()

    def _stall_end(self) -> None:
        self._stall_until = 0
        if self._stalled:
            self._stalled = False
            self._sleeping = False

    def _store_size(self, pc: int) -> int:
        """Size of the store instruction at ``pc`` (0 if it is not a store)."""
        try:
            hw = struct.unpack("<H", self.read(pc, 2))[0]
        except UcError:
            return 0
        if (hw >> 11) in (0x1D, 0x1E, 0x1F):  # 32-bit Thumb-2
            # STR{B,H} (imm/reg): 1111 1000 xxx0 ; STM/STMDB/PUSH.W: 1110 100x x0x0
            if (hw & 0xFE10) == 0xF800:
                return 4
            if (hw & 0xFE50) == 0xE800 and (hw & 0x0010) == 0:
                return 4
            if (hw & 0xFE50) == 0xE840 and (hw & 0x0010) == 0:  # STRD/STREX
                return 4
            return 0
        top5 = hw >> 11
        if top5 in (0x0C, 0x0E, 0x10, 0x12):  # STR/STRB/STRH/STR(sp) imm
            return 2
        if (hw >> 9) in (0x28, 0x29, 0x2A):  # STR/STRH/STRB reg
            return 2
        if (hw >> 11) == 0x18 or (hw & 0xFE00) == 0xB400:  # STMIA / PUSH
            return 2
        return 0

    def install_unmask_exits(self, image: bytes, base: int) -> int:
        """Stop right *before* every ``cpsie i`` / ``msr primask|basepri|faultmask``.

        These instructions can unmask an interrupt that is already pending
        (e.g. a FreeRTOS yield requested inside a critical section) which the
        hardware takes immediately afterwards.  They are emulated in Python
        (see :meth:`_emulate_unmask`) and the interrupt is taken on the spot.
        Unicorn "exits" cost nothing until they are hit, unlike code hooks.
        """
        table = {}
        for off in range(0, len(image) - 4, 2):
            hw = image[off] | (image[off + 1] << 8)
            if hw == 0xB662:  # cpsie i
                table[base + off] = ("cpsie", 0, 2)
            elif hw & 0xFFF0 == 0xF380:
                hw2 = image[off + 2] | (image[off + 3] << 8)
                if hw2 in (0x8810, 0x8811, 0x8812, 0x8813):
                    table[base + off] = (("primask", "basepri", "basepri_max", "faultmask")[hw2 - 0x8810],
                                         hw & 0xF, 4)
        if not table or not self.counter.ok:
            return 0
        self.unmask_table = table
        self.uc.ctl_exits_enabled(True)
        self.uc.ctl_set_exits(sorted(table))
        self.uc.ctl_exits_enabled(False)
        self.uc.ctl_flush_tb()
        self._exits_on = False
        log.debug("%d unmask exits", len(table))
        return len(table)

    def _set_exits(self, on: bool) -> None:
        """Exits are checked when code is translated, so switching them on or
        off needs a translation-cache flush; it only happens around long
        critical sections."""
        if on != self._exits_on and self.unmask_table:
            self.uc.ctl_exits_enabled(on)
            self.uc.ctl_flush_tb()
            self._exits_on = on
            self.exit_toggles += 1
            if not on:
                self._exits_off_at = self.clock.cycles
                t0, h0 = self._exits_on_at
                dt = self.clock.cycles - t0
                if dt > 0:
                    # unmask hits per virtual second while exits were on
                    self._exit_hit_rate = (self.unmask_hits - h0) * self.clock.hz / dt

    def _masked_pending(self) -> bool:
        return bool(self.nvic.pend_set) and self.nvic.best_pending(self._exec_priority()) is None

    def _step_until_unmasked(self, max_steps: int = 48) -> bool:
        """Single-step while an interrupt is pending but masked, taking it the
        instruction after it becomes unmasked.  Returns False if the critical
        section is longer than ``max_steps`` (caller then uses exits)."""
        PC = A.UC_ARM_REG_PC
        for _ in range(max_steps):
            pc = self.rr(PC)
            if pc in self.unmask_table:
                self._emulate_unmask(pc)
            else:
                self._early = False
                self.running = True
                try:
                    self.uc.emu_start(pc | 1, 0 if self._exits_on else 1, count=1)
                except UcError:
                    self.running = False
                    return True  # let the main loop report it
                self.running = False
                if self._early and self._early_pc and self.rr(PC) == self._early_pc[0]:
                    self.rw(PC, (self._early_pc[0] + self._early_pc[1]) | 1)
                self._early_pc = None
                if self._stop or self._reset_req or self._sleeping:
                    return True
            self.clock.advance(self.cpi)
            nxt = self.nvic.best_pending(self._exec_priority()) if self.nvic.pend_set else None
            if nxt is not None:
                self._take(nxt, self.rr(PC))
                return True
            if not self.nvic.pend_set:
                return True
        return False

    def _emulate_unmask(self, pc: int) -> None:
        kind, rn, size = self.unmask_table[pc]
        if self.rr(A.UC_ARM_REG_XPSR) & 0x0600FC00:
            # inside an IT block: rare; the condition is ignored
            log.debug("msr/cpsie inside IT block at 0x%08x", pc)
        if kind == "cpsie":
            self.rw(A.UC_ARM_REG_PRIMASK, 0)
        else:
            v = self.rr(A.UC_ARM_REG_R0 + rn) if rn < 13 else (self.rr(A.UC_ARM_REG_SP) if rn == 13 else self.rr(A.UC_ARM_REG_LR))
            if kind == "primask":
                self.rw(A.UC_ARM_REG_PRIMASK, v & 1)
            elif kind == "faultmask":
                self.rw(A.UC_ARM_REG_FAULTMASK, v & 1)
            elif kind == "basepri":
                self.rw(A.UC_ARM_REG_BASEPRI, v & 0xFF)
            else:  # basepri_max only raises
                cur = self.rr(A.UC_ARM_REG_BASEPRI)
                if v & 0xFF and (cur == 0 or (v & 0xFF) < cur):
                    self.rw(A.UC_ARM_REG_BASEPRI, v & 0xFF)
        self.rw(A.UC_ARM_REG_PC, (pc + size) | 1)
        self.unmask_hits += 1

    def _check_interrupts(self) -> None:
        if not self.nvic.pend_set:
            return
        nxt = self.nvic.best_pending(self._exec_priority())
        if nxt is not None:
            self._take(nxt, self.pc)

    def run(self, seconds: Optional[float] = None, cycles: Optional[int] = None,
            until: Optional[Callable[[], bool]] = None) -> str:
        """Run until the time budget is used up, ``until()`` is true or stop()."""
        end = None
        if seconds is not None:
            end = self.clock.cycles + int(seconds * self.clock.hz)
        if cycles is not None:
            end = self.clock.cycles + cycles
        self._stop = False
        self.stop_reason = None
        clock = self.clock
        uc = self.uc
        cpi = self.cpi
        counter = self.counter
        table = self.unmask_table
        PC = A.UC_ARM_REG_PC
        while not self._stop:
            if self._reset_req:
                why, self._reset_req = self._reset_req, None
                log.warning("system reset (%s) at t=%.3fs", why, clock.seconds)
                self.reset()
            clock.run_due()
            for h in self.slice_hooks:
                h()
            if until is not None and until():
                self.stop_reason = "condition"
                break
            if end is not None and clock.cycles >= end:
                self.stop_reason = "time"
                break
            self._check_interrupts()
            if self._stop:
                break
            nd = clock.next_deadline()
            if self._sleeping:
                # WFI: jump to the next timed event (or end of budget)
                target = nd if nd is not None else end
                if target is None:
                    self.stop_reason = "deadlock (WFI with nothing scheduled)"
                    break
                if end is not None:
                    target = min(target, end)
                clock.cycles = max(clock.cycles + 1, target)
                continue
            budget = self.slice_cycles
            if nd is not None:
                budget = min(budget, max(1, nd - clock.cycles))
            if end is not None:
                budget = min(budget, max(1, end - clock.cycles))
            insns = max(1, budget // cpi)
            pc = self.rr(PC)
            if pc in table:
                self._emulate_unmask(pc)
                clock.advance(cpi)
                continue
            if table:
                masked = self._masked_pending()
                if masked:
                    if not self._exits_on:
                        if self._step_until_unmasked():
                            continue
                        # long critical section; if this keeps happening, keep
                        # exits on for a while instead of flushing every time
                        en = self._exits_enables
                        en.append(clock.cycles)
                        while en and clock.cycles - en[0] > 2 * self._exits_hold:
                            en.popleft()
                        # sticky only pays off when the firmware rarely hits an
                        # unmask instruction (each hit costs a restart)
                        self._exits_sticky = len(en) >= 10 and self._exit_hit_rate < 2000
                        self._exits_on_at = (clock.cycles, self.unmask_hits)
                        self._set_exits(True)
                        pc = self.rr(PC)
                        if pc in table:
                            continue
                    # exits on: run normally, the unmask instruction stops us
                    self._exits_busy = clock.cycles
                if self._exits_on and self._exits_sticky and clock.cycles - self._exits_on_at[0] > self._exits_hold:
                    # re-measure while sticky: when the firmware hits unmask
                    # instructions often, stopping at each costs more than
                    # the occasional flush
                    t0, h0 = self._exits_on_at
                    self._exit_hit_rate = (self.unmask_hits - h0) * clock.hz / (clock.cycles - t0)
                    self._exits_on_at = (clock.cycles, self.unmask_hits)
                    if self._exit_hit_rate >= 2000:
                        self._exits_sticky = False
                if self._exits_on and not masked and clock.cycles - self._exits_busy > (
                        self._exits_hold if self._exits_sticky else 0):
                    # firmware with frequent long critical sections would make
                    # us flush the translation cache constantly: switch off
                    # only after a quiet period
                    self._set_exits(False)
            # inner loop: keep emulating through unmask exits while nothing
            # else needs attention (no events due, nothing pending)
            while True:
                self._early = False
                self._idle_stop = False
                self.running = True
                try:
                    uc.emu_start(pc | 1, 0 if self._exits_on else 1, count=insns)
                except UcError as e:
                    self.running = False
                    self.fault = f"{e} at pc=0x{self.pc:08x}"
                    log.error("emulation error: %s", self.fault)
                    if self.fault_policy == "stop":
                        self.stop_reason = "error: " + self.fault
                        self._stop = True
                        break
                    self._fault(str(e))
                    clock.advance(budget)
                    break
                self.running = False
                done = counter.value()
                if done is None:
                    done = insns if not self._early else self.early_stop_cycles // cpi
                done = min(done, insns)
                clock.advance(max(1, done) * cpi)
                if done >= insns:
                    break
                # stopped early: find out why
                npc = self.rr(PC)
                if self._early:
                    self.early_stops += 1
                    if self._early_pc and npc == self._early_pc[0]:
                        # the store already happened: step over it
                        self.rw(PC, (npc + self._early_pc[1]) | 1)
                    self._early_pc = None
                    if self._stall_req:
                        self._stall_req = False
                        if not self.nvic.pend_set and not self._stop and not self._reset_req:
                            self._sleeping = self._stalled = True
                    break
                if npc in table:
                    self._emulate_unmask(npc)
                    insns -= done + 1
                    if insns <= 0 or self._stop or self._reset_req or self.nvic.pend_set:
                        break
                    pc = self.rr(PC)
                    continue
                if self._stall_resume:
                    self._stall_resume = False
                    if self._stall_until and not self.nvic.pend_set and not self._stop and not self._reset_req:
                        self._sleeping = self._stalled = True
                    break
                if self._idle_stop:
                    self._idle_stop = False
                    if not self.nvic.pend_set and not self._stop and not self._reset_req:
                        self._sleeping = True
                        self.idle_sleeps += 1
                    break
                if not self._stop and not self._reset_req:
                    # WFI/WFE stop emulation with PC after the instruction
                    try:
                        prev = struct.unpack("<H", self.read(npc - 2, 2))[0]
                    except UcError:
                        prev = 0
                    if prev in (0xBF30, 0xBF20) and not self.nvic.pend_set:
                        self._sleeping = True
                break
        return self.stop_reason or "stopped"
