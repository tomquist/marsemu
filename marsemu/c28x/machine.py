"""A C28x microcontroller with the run-loop interface of :class:`marsemu.core.machine.Machine`.

The Cortex-M devices run on Unicorn; the C28x has no such engine, so its CPU
is interpreted (:mod:`.cpu`).  This class gives it the same surroundings: the
virtual :class:`~marsemu.core.clock.Clock` with its timed events, peripherals
by name, console channels, slice and reset hooks, sleep on IDLE (and on
detected wait loops), so the FC41D model, the plant models and the dashboard
drive it like any other machine.
"""
from __future__ import annotations

import logging
from array import array
from typing import Callable, Dict, List, Optional

from ..core.clock import Clock
from .cpu import C28x, CpuFault

log = logging.getLogger("emu.c28x")


class C28xPeripheral:
    """A bank of 16-bit registers at word address ``base`` (``size`` words).

    Subclasses override :meth:`read` / :meth:`write` (offsets in words).
    Accesses to registers a model does not handle read back what was
    written, like :class:`marsemu.core.bus.UnknownPeripheral`."""

    #: word offset -> register name (for traces)
    reg_names: Dict[int, str] = {}
    #: registers that need EALLOW to be written
    protected: bool = False

    def __init__(self, machine: "C28xMachine", name: str, base: int, size: int):
        self.m = machine
        self.name = name
        self.base = base
        self.size = size
        self.regs: Dict[int, int] = {}
        self.reset_values: Dict[int, int] = {}
        self._seen: set = set()

    def reset(self) -> None:
        self.regs = dict(self.reset_values)

    def r(self, off: int) -> int:
        return self.regs.get(off, 0)

    def reg_name(self, off: int) -> str:
        return self.reg_names.get(off, f"+0x{off:02x}")

    def read(self, off: int) -> int:
        if off not in self.reg_names and ("r", off) not in self._seen:
            self._seen.add(("r", off))
            log.debug("unmodelled %s read %s (pc=0x%06x)", self.name, self.reg_name(off), self.m.pc)
        return self.regs.get(off, 0)

    def write(self, off: int, v: int) -> None:
        if off not in self.reg_names and ("w", off) not in self._seen:
            self._seen.add(("w", off))
            log.debug("unmodelled %s write %s = 0x%04x (pc=0x%06x)", self.name, self.reg_name(off), v, self.m.pc)
        self.regs[off] = v

    def peek(self, off: int) -> int:
        return self.regs.get(off, 0)

    # --- the CPU's interface (absolute word addresses) ------------------------
    def read16(self, a: int) -> int:
        off = a - self.base
        v = self.read(off)
        if self.m.tracers:
            for t in self.m.tracers:
                t("r", self, off, 2, v)
        return v

    def write16(self, a: int, v: int) -> None:
        off = a - self.base
        if self.m.tracers:
            for t in self.m.tracers:
                t("w", self, off, 2, v)
        if self.protected and not self.m.cpu.eallow:
            return
        self.write(off, v)

    def peek16(self, a: int) -> int:
        return self.peek(a - self.base)

    def __repr__(self) -> str:
        return f"<{type(self).__name__} {self.name}@0x{self.base:06x}>"


class _PageSplitter:
    """Dispatches one 64-word page to the peripherals that share it; words
    no peripheral claims read 0."""

    def __init__(self, m, base):
        self.m = m
        self.base = base
        self.parts: List[C28xPeripheral] = []

    def _find(self, a):
        for p in self.parts:
            if p.base <= a < p.base + p.size:
                return p
        return None

    def read16(self, a):
        p = self._find(a)
        return p.read16(a) if p is not None else 0

    def write16(self, a, v):
        p = self._find(a)
        if p is not None:
            p.write16(a, v)

    def peek16(self, a):
        p = self._find(a)
        return p.peek16(a) if p is not None else 0


class VirtualClockTime:
    """Calendar time on the virtual timeline (the board has no RTC; the
    cloud and network models ask for ``now()``)."""

    def __init__(self, m):
        import datetime as _dt
        self.m = m
        self.epoch = _dt.datetime.now().replace(microsecond=0)
        self.offset = 0.0

    def now(self):
        import datetime as _dt
        return self.epoch + _dt.timedelta(seconds=self.m.clock.seconds + self.offset)

    def reset(self):
        pass


class C28xMachine:
    """Clock + C28x CPU + peripherals, with the run loop of the Cortex-M machine."""

    #: word address of the flash (set by the SoC)
    flash_base = 0

    def __init__(self, hz: int = 60_000_000, slice_cycles: int = 20000):
        self.clock = Clock(hz)
        self.cpu = C28x(self.clock)
        self.slice_cycles = slice_cycles
        self.peripherals: Dict[str, C28xPeripheral] = {}
        self.consoles: List[Callable[[str, str], None]] = []
        self.reset_hooks: List[Callable[[], None]] = []
        self.slice_hooks: List[Callable[[], None]] = []
        self.tracers: List[Callable] = []
        self.stop_reason: Optional[str] = None
        self._stop = False
        self._reset_req: Optional[str] = None
        self.reset_count = 0
        self.fault: Optional[str] = None
        self.fault_policy = "stop"
        self.running = False
        #: entry point at reset (the boot ROM's jump target)
        self.entry = 0x3F7FF6
        #: wait loops slept through like IDLE: addr -> mark (see set_idle_loop)
        self.idle_loops: Dict[int, list] = {}
        self.idle_sleeps = 0
        self.exc_count = 0
        #: interrupts taken per CPU line (1..14 INT1..INT14, 15 DLOGINT, 16 RTOSINT)
        self.int_count = [0] * 32
        self.cpu.ack_hook = self._on_ack
        self.flash_size = 0
        self.peripherals["RTC"] = VirtualClockTime(self)
        #: the native peripherals (f2803x_native): run their due events, the time
        #: of the next one, route IO through the Python objects (tracers)
        self.native_poll: Optional[Callable[[], object]] = None
        self.native_next: Optional[Callable[[], int]] = None
        self.native_trace: Optional[Callable[[bool], object]] = None
        self._native_traced = False

    # ---------------------------------------------------------- peripherals
    def attach(self, p: C28xPeripheral) -> C28xPeripheral:
        from .cpu import IO, PAGE_SHIFT
        self.peripherals[p.name] = p
        cpu = self.cpu
        for pg in range(p.base >> PAGE_SHIFT, (p.base + p.size + 63) >> PAGE_SHIFT):
            cur = cpu.io.get(pg) if cpu.ptype[pg] == IO else None
            lo, hi = pg << PAGE_SHIFT, (pg + 1) << PAGE_SHIFT
            whole = p.base <= lo and p.base + p.size >= hi
            if cur is None and whole:
                cpu.map(lo, hi, IO, p)
                continue
            if not isinstance(cur, _PageSplitter):
                sp = _PageSplitter(self, lo)
                if cur is not None:
                    sp.parts.append(cur)
                cpu.map(lo, hi, IO, sp)
                cur = sp
            cur.parts.append(p)
        return p

    def get(self, name: str) -> C28xPeripheral:
        return self.peripherals[name]

    @property
    def bus(self):
        return self

    # ------------------------------------------------------------- helpers
    @property
    def pc(self) -> int:
        return self.cpu.pc

    def reg(self, name: str) -> int:
        return self.cpu.regs()[name.upper()]

    def read(self, addr: int, n: int) -> bytes:
        """``n`` bytes from word address ``addr`` (little endian words)."""
        out = bytearray()
        for i in range((n + 1) // 2):
            v = self.cpu.peek(addr + i)
            out += bytes((v & 0xFF, v >> 8))
        return bytes(out[:n])

    def write(self, addr: int, data: bytes) -> None:
        if len(data) % 2:
            data = bytes(data) + bytes([self.cpu.peek(addr + len(data) // 2) >> 8])
        for i in range(0, len(data), 2):
            self.cpu.mem[addr + i // 2] = data[i] | (data[i + 1] << 8)
        self.cpu.flush()

    def load_flash(self, data: bytes, offset: int = 0) -> None:
        """Program flash contents: ``data`` little-endian words, ``offset``
        in bytes from :attr:`flash_base`."""
        if len(data) % 2:
            data = bytes(data) + b"\xff"
        words = array("H", bytes(data))
        a = self.flash_base + offset // 2
        n = min(len(words), self.flash_base + self.flash_size - a)
        self.cpu.mem[a:a + n] = words[:n]
        self.cpu.flush()

    def flash_bytes(self) -> bytes:
        return self.cpu.mem[self.flash_base:self.flash_base + self.flash_size].tobytes()

    def console(self, channel: str, line: str) -> None:
        for c in self.consoles:
            c(channel, line)

    def wake(self) -> None:
        self.cpu.idle_wait = False

    def stop(self, reason: str = "stopped") -> None:
        self.stop_reason = reason
        self._stop = True

    def request_reset(self, why: str) -> None:
        self._reset_req = why

    def interrupt(self, k: int) -> None:
        """Raise CPU interrupt line INTk (1..14)."""
        self.cpu.ifr |= 1 << (k - 1)
        if self.cpu.idle_wait and self.cpu.ier & (1 << (k - 1)):
            self.cpu.idle_wait = False

    def wake_nmi_wd(self) -> None:
        """Watchdog interrupt (WAKEINT, PIE 1.8)."""
        self.pie.raise_irq(1, 8)

    def _earlier(self, when: int) -> None:
        self.cpu.limit = when

    def _on_ack(self, k: int) -> None:
        self.exc_count += 1
        self.int_count[k] += 1

    def set_idle_loop(self, addr: int, feeds_watchdog: bool = False) -> None:
        """Sleep at ``addr`` (head of a wait loop) when an iteration went by
        without an interrupt, until the next timed event."""
        mark = [-1]
        m = self

        def hit(cpu):
            if mark[0] == m.exc_count and not cpu.pending():
                cpu.idle_wait = True
                m.idle_sleeps += 1
                cpu.pc = addr
                return False
            mark[0] = m.exc_count
            return True
        self.idle_loops[addr] = mark
        self.cpu.add_code_hook(addr, hit)

    def install_delay_loops(self, start: int, end: int) -> List[int]:
        """Fast-forward the TI compiler's calibrated delay loops::

            L: RPT #k || NOP ; INC loc ; CMP loc,#N ; SB L,LO

        Arriving at ``L`` the remaining iterations become a wait (during
        which interrupts run as usual) and the counter is set to ``N``."""
        mem = self.cpu.mem
        found = []
        for a in range(start, end - 6):
            if (mem[a] >> 8) != 0xF6 or mem[a + 1] != 0x7700:
                continue
            inc, cmp_, sb = mem[a + 2], mem[a + 3], mem[a + 5]
            if inc >> 8 != 0x0A or cmp_ >> 8 != 0x1B or (cmp_ & 0xFF) != (inc & 0xFF) or sb >> 8 != 0x68:
                continue
            off = sb & 0xFF
            if a + 5 + (off - 256 if off & 0x80 else off) != a:
                continue
            L = inc & 0xFF
            if L >> 6 not in (0, 1):    # direct or stack-relative counter only
                continue
            self._hook_delay(a, mem[a] & 0xFF, L, mem[a + 4])
            found.append(a)
        return found

    def _hook_delay(self, a: int, k: int, L: int, n: int) -> None:
        per = (k + 1) + 1 + 1 + 1 + 4     # RPT, the NOPs, INC, CMP, taken SB

        def hit(cpu):
            addr = ((cpu.dp << 6) | (L & 63)) if L >> 6 == 0 else (cpu.sp - (L & 63)) & 0xFFFF
            v = cpu.rd16(addr)
            if v >= n:
                return True
            cpu.wr16(addr, n)
            cpu.z, cpu.n, cpu.c = 1, 0, 1
            cpu.pc = a + 6
            cpu.wait_pc = a + 6
            cpu.wait_left = (n - v) * per
            return False
        self.cpu.add_code_hook(a, hit)

    def feeding_watchdog(self) -> bool:
        return False

    def _next_deadline(self) -> Optional[int]:
        """The next clock or native event."""
        nd = self.clock.next_deadline()
        if self.native_next is not None:
            nn = self.native_next()
            if nn < (1 << 63) - 1:
                nd = nn if nd is None else min(nd, nn)
        return nd

    def advance(self, seconds: float) -> None:
        end = self.clock.cycles + int(seconds * self.clock.hz)
        while True:
            for h in self.slice_hooks:
                h()
            if self.native_poll is not None:
                self.native_poll()
            nd = self._next_deadline()
            if nd is None or nd > end:
                break
            self.clock.cycles = max(self.clock.cycles, nd)
            self.clock.run_due()
        self.clock.cycles = end
        if self.native_poll is not None:
            self.native_poll()

    # ---------------------------------------------------------------- reset
    def reset(self) -> None:
        self.reset_count += 1
        cpu = self.cpu
        cpu.reset()
        for p in self.peripherals.values():
            p.reset()
        # the state the boot ROM leaves when it branches to flash
        cpu.objmode = 1
        cpu.amode = 0
        cpu.m0m1map = 1
        cpu.sp = 0x400
        cpu.pc = self.entry
        for h in self.reset_hooks:
            h()
        log.info("reset #%d: PC=0x%06x", self.reset_count, cpu.pc)

    # -------------------------------------------------------------- run loop
    def run(self, seconds: Optional[float] = None, cycles: Optional[int] = None,
            until: Optional[Callable[[], bool]] = None) -> str:
        end = None
        if seconds is not None:
            end = self.clock.cycles + int(seconds * self.clock.hz)
        if cycles is not None:
            end = self.clock.cycles + cycles
        self._stop = False
        self.stop_reason = None
        clock = self.clock
        cpu = self.cpu
        native_poll = self.native_poll
        if self.native_trace is not None and bool(self.tracers) != self._native_traced:
            self._native_traced = bool(self.tracers)
            self.native_trace(self._native_traced)
        while not self._stop:
            if self._reset_req:
                why, self._reset_req = self._reset_req, None
                log.warning("system reset (%s) at t=%.3fs", why, clock.seconds)
                self.reset()
            clock.run_due()
            if native_poll is not None:
                native_poll()
            for h in self.slice_hooks:
                h()
            if until is not None and until():
                self.stop_reason = "condition"
                break
            if end is not None and clock.cycles >= end:
                self.stop_reason = "time"
                break
            nd = clock.next_deadline()
            if cpu.idle_wait or (cpu.wait_left and cpu.pc == cpu.wait_pc):
                # time skips ahead here: not past the native peripherals' next event
                nd = self._next_deadline() if native_poll is not None else nd
            if cpu.idle_wait:
                if cpu.ifr & cpu.ier:
                    cpu.idle_wait = False
                else:
                    target = nd if nd is not None else end
                    if target is None:
                        self.stop_reason = "deadlock (IDLE with nothing scheduled)"
                        break
                    if end is not None:
                        target = min(target, end)
                    clock.cycles = max(clock.cycles + 1, target)
                    continue
            if cpu.wait_left and cpu.pc == cpu.wait_pc:
                if cpu.ifr & cpu.ier and not cpu.intm:
                    pass   # take the interrupt; the wait resumes when it returns here
                else:
                    target = clock.cycles + cpu.wait_left
                    if nd is not None:
                        target = min(target, nd)
                    if end is not None:
                        target = min(target, end)
                    dt = max(1, target - clock.cycles)
                    clock.cycles += dt
                    cpu.wait_left = max(0, cpu.wait_left - dt)
                    continue
            if cpu.halted:
                self.fault = cpu.halted
                self.stop_reason = "fault: " + cpu.halted
                log.error("CPU halted: %s", cpu.halted)
                break
            budget = self.slice_cycles
            if nd is not None:
                budget = min(budget, max(1, nd - clock.cycles))
            if end is not None:
                budget = min(budget, max(1, end - clock.cycles))
            self.running = True
            # an event scheduled during the slice, before its end, ends it there
            clock.horizon = clock.cycles + budget
            clock.on_earlier = self._earlier
            try:
                cpu.run(budget)
            except CpuFault as e:
                self.fault = str(e)
                self.stop_reason = "fault: " + str(e)
                log.error("%s", e)
                break
            finally:
                self.running = False
                clock.horizon = 1 << 62
                clock.on_earlier = None
        return self.stop_reason or "stopped"
