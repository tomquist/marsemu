"""The C28x CPU: registers, word-addressed memory, interrupts and the run loop.

Memory is one array of 2^22 16-bit words.  Each 64-word page has a type:
plain RAM, memory-mapped I/O (dispatched to a peripheral), read-only (flash,
OTP, boot ROM; writes go to ``rom_write`` if set) or RAM that holds decoded
code (a write drops the decoded instructions there, for code copied to RAM).
"""
from __future__ import annotations

import logging
from array import array
from typing import Callable, Dict, List, Optional

from .isa import M16, M22, M32, REPEATABLE, Insn, Unimplemented, disasm, match, render
from .semantics import build

log = logging.getLogger("emu.c28x")

PAGE_SHIFT = 6
RAM, IO, ROM, CODE = 0, 1, 2, 3
#: vector numbers (SPRU430F table 3-1)
V_RESET, V_INT1, V_DLOGINT, V_RTOSINT, V_NMI, V_ILLEGAL = 0, 1, 15, 16, 18, 19


class CpuFault(Exception):
    pass


class C28x:
    """Register file and execution of a C28x core (OBJMODE=1, AMODE=0)."""

    def __init__(self, clock):
        self.clock = clock
        self.mem = array("H", bytes(2 << 22))
        self.ptype = bytearray(1 << (22 - PAGE_SHIFT))
        self.io: Dict[int, object] = {}            # page -> peripheral (read16/write16 by address)
        self.icache: Dict[int, Insn] = {}
        #: write to a ROM page: fn(addr, value) (flash programming); None = ignored
        self.rom_write: Optional[Callable[[int, int], None]] = None
        #: vector fetch for interrupt k: fn(k) -> handler address (the PIE); None = vector table at 0xD00
        self.vector_hook: Optional[Callable[[int], int]] = None
        #: called when the CPU acknowledges interrupt k (the PIE clears and re-raises lines)
        self.ack_hook: Optional[Callable[[int], None]] = None
        self.code_hooks: Dict[int, Callable] = {}
        #: IACK #16bit: fn(value) (starts CLA tasks)
        self.iack_hook: Optional[Callable[[int], None]] = None
        self.trace: Optional[Callable[[int, str], None]] = None
        self.insn_count = 0
        self._io_list: Optional[list] = None
        self._core = None
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.acc = self.p = self.xt = 0
        self.xar = [0] * 8
        self.pc = 0x3FFFC0
        self.rpc = 0
        self.sp = 0x400
        self.dp = 0
        self.sxm = self.ovm = self.tc = self.c = self.z = self.n = self.v = 0
        self.pm = 0
        self.ovc = 0
        self.intm = 1
        self.dbgm = 1
        self.page0 = 0
        self.vmap = 1
        self.spa = 0
        self.loop = 0
        self.eallow = 0
        self.idlestat = 0
        self.amode = 0
        self.objmode = 0
        self.m0m1map = 1
        self.xf = 0
        self.arp = 0
        self.ier = self.ifr = self.dbgier = 0
        self.dbgstat = 0
        self.rptc = 0
        self.rpt_pending = False
        self.rpt_i = 0
        self.rpt_last = True
        self.bio = 0
        self.idle_wait = False
        self.halted: Optional[str] = None
        #: a fast-forwarded delay loop: the CPU waits at ``wait_pc`` for
        #: ``wait_left`` cycles (interrupts still run meanwhile)
        self.wait_pc = -1
        self.wait_left = 0

    @property
    def st0(self) -> int:
        return (self.sxm | self.ovm << 1 | self.tc << 2 | self.c << 3 | self.z << 4 | self.n << 5 | self.v << 6
                | self.pm << 7 | (self.ovc & 63) << 10)

    @st0.setter
    def st0(self, v: int) -> None:
        self.sxm, self.ovm, self.tc, self.c = v & 1, v >> 1 & 1, v >> 2 & 1, v >> 3 & 1
        self.z, self.n, self.v = v >> 4 & 1, v >> 5 & 1, v >> 6 & 1
        self.pm = v >> 7 & 7
        o = v >> 10 & 63
        self.ovc = o - 64 if o & 32 else o

    @property
    def st1(self) -> int:
        return (self.intm | self.dbgm << 1 | self.page0 << 2 | self.vmap << 3 | self.spa << 4 | self.loop << 5
                | self.eallow << 6 | self.idlestat << 7 | self.amode << 8 | self.objmode << 9
                | self.m0m1map << 11 | self.xf << 12 | self.arp << 13)

    @st1.setter
    def st1(self, v: int) -> None:
        self.intm, self.dbgm, self.page0, self.vmap = v & 1, v >> 1 & 1, v >> 2 & 1, v >> 3 & 1
        self.spa, self.loop, self.eallow, self.idlestat = v >> 4 & 1, v >> 5 & 1, v >> 6 & 1, v >> 7 & 1
        self.amode, self.objmode, self.m0m1map, self.xf = v >> 8 & 1, v >> 9 & 1, v >> 11 & 1, v >> 12 & 1
        self.arp = v >> 13 & 7

    def regs(self) -> dict:
        r = {"PC": self.pc, "RPC": self.rpc, "ACC": self.acc, "P": self.p, "XT": self.xt, "SP": self.sp,
             "DP": self.dp, "ST0": self.st0, "ST1": self.st1, "IER": self.ier, "IFR": self.ifr}
        for i, v in enumerate(self.xar):
            r[f"XAR{i}"] = v
        return r

    # ----------------------------------------------------------------- memory
    def map(self, start: int, end: int, kind: int, periph=None) -> None:
        """Set the type of the pages in [start, end) (64-word granularity)."""
        for pg in range(start >> PAGE_SHIFT, (end + 63) >> PAGE_SHIFT):
            self.ptype[pg] = kind
            if kind == IO:
                self.io[pg] = periph
            else:
                self.io.pop(pg, None)
            if self._io_list is not None:
                self._io_list[pg] = periph if kind == IO else None

    def rd16(self, a: int) -> int:
        if self.ptype[a >> 6] == IO:
            return self.io[a >> 6].read16(a) & M16
        return self.mem[a]

    def wr16(self, a: int, v: int) -> None:
        t = self.ptype[a >> 6]
        if t == RAM:
            self.mem[a] = v
        elif t == IO:
            self.io[a >> 6].write16(a, v & M16)
        elif t == CODE:
            self.mem[a] = v
            ic = self.icache
            ic.pop(a, None)
            ic.pop(a - 1, None)
            if self._core is not None:
                self._core.invalidate(a)
        elif self.rom_write is not None:
            self.rom_write(a, v)

    def rd32(self, a: int) -> int:
        a &= 0x3FFFFE
        if self.ptype[a >> 6] == IO:
            p = self.io[a >> 6]
            if hasattr(p, "read32"):
                return p.read32(a) & M32
            return (p.read16(a) & M16) | ((p.read16(a + 1) & M16) << 16)
        m = self.mem
        return m[a] | (m[a + 1] << 16)

    def wr32(self, a: int, v: int) -> None:
        a &= 0x3FFFFE
        t = self.ptype[a >> 6]
        if t == RAM:
            m = self.mem
            m[a] = v & M16
            m[a + 1] = v >> 16
        elif t == IO:
            p = self.io[a >> 6]
            if hasattr(p, "write32"):
                p.write32(a, v & M32)
            else:
                p.write16(a, v & M16)
                p.write16(a + 1, (v >> 16) & M16)
        else:
            self.wr16(a, v & M16)
            self.wr16(a + 1, (v >> 16) & M16)

    def peek(self, a: int) -> int:
        """Read without side effects (debugger, disassembly)."""
        if self.ptype[a >> 6] == IO:
            p = self.io[a >> 6]
            return p.peek16(a) & M16 if hasattr(p, "peek16") else 0
        return self.mem[a]

    def load(self, addr: int, words) -> None:
        self.mem[addr:addr + len(words)] = array("H", words)
        for a in list(self.icache):
            if addr <= a < addr + len(words):
                del self.icache[a]
        if self._core is not None:
            self._core.flush()

    def flush(self) -> None:
        self.icache.clear()
        if self._core is not None:
            self._core.flush()

    # ----------------------------------------------------------------- decode
    def decode(self, pc: int) -> Insn:
        w1 = self.mem[pc]
        w2 = self.mem[(pc + 1) & M22]
        m = match(w1, w2)
        if m is None or (w1 in (0x0000, 0xFFFF)):
            ins = Insn(pc, 1, self._illegal_fn(pc, w1), f".word 0x{w1:04x}", flow="trap")
        else:
            k, f, size, syn = m
            text = render(k, f, syn, pc, size)
            try:
                fn = build(k, f, pc, size)
            except Unimplemented as e:
                fn = self._unimpl_fn(pc, text, str(e))
            ins = Insn(pc, size, fn, text, repeatable=k in REPEATABLE, key=k)
        hook = self.code_hooks.get(pc)
        if hook is not None:
            inner = ins.fn

            def hooked(c, inner=inner, hook=hook):
                if hook(c) is not False:
                    inner(c)
            ins.fn = hooked
        pg = pc >> PAGE_SHIFT
        if self.ptype[pg] == RAM:
            self.ptype[pg] = CODE
        if self.ptype[(pc + 1) >> PAGE_SHIFT] == RAM:
            self.ptype[(pc + 1) >> PAGE_SHIFT] = CODE
        self.icache[pc] = ins
        return ins

    def add_code_hook(self, addr: int, fn: Callable) -> None:
        """Call ``fn(cpu)`` before the instruction at ``addr``; returning False skips it."""
        self.code_hooks[addr] = fn
        self.icache.pop(addr, None)
        if self._core is not None:
            self._core.invalidate(addr)
            self._core.invalidate(addr + 1)

    def _illegal_fn(self, pc, w):
        def fn(c):
            log.error("illegal instruction 0x%04x at 0x%06x", w, pc)
            c.pc = pc
            c.trap(V_ILLEGAL, pc + 1, False)
        return fn

    def _unimpl_fn(self, pc, text, why):
        def fn(c):
            c.pc = pc
            raise CpuFault(f"unimplemented instruction at 0x{pc:06x}: {text} ({why})")
        return fn

    def disasm(self, addr: int):
        return disasm(self.peek, addr)

    # ------------------------------------------------------------- interrupts
    def _push32(self, v):
        self.wr32(self.sp, v)
        self.sp = (self.sp + 2) & M16

    def _pop32(self):
        self.sp = (self.sp - 2) & M16
        return self.rd32(self.sp)

    def _vector(self, k: int) -> int:
        if self.vector_hook is not None:
            return self.vector_hook(k)
        if self.vmap:
            return self.rd32(0x3FFFC0 + 2 * k)
        return self.rd32(0x000000 + 2 * k)

    def enter(self, k: int, ret: int) -> None:
        """Automatic context save and vector fetch for interrupt/trap k."""
        self.sp = (self.sp + 1) & M16
        self._push32((self.xt & 0xFFFF0000) | self.st0)
        self._push32(self.acc)
        self._push32(self.p)
        self._push32(((self.xar[1] & M16) << 16) | (self.xar[0] & M16))
        self._push32((self.dp << 16) | self.st1)
        self._push32((self.dbgstat << 16) | self.ier)
        self._push32(ret)
        vec = self._vector(k)
        if 1 <= k <= 16:
            self.ier &= ~(1 << (k - 1)) & M16
        self.intm = 1
        self.dbgm = 1
        self.loop = 0
        self.eallow = 0
        self.idlestat = 0
        self.idle_wait = False
        self.pc = vec & M22

    def iret(self) -> None:
        self.pc = self._pop32() & M22
        v = self._pop32()
        self.ier = v & M16
        self.dbgstat = v >> 16
        v = self._pop32()
        st1 = v & M16
        # LOOP, IDLESTAT and M0M1MAP are not restored
        keep = self.loop << 5 | self.idlestat << 7 | self.m0m1map << 11
        self.st1 = (st1 & ~((1 << 5) | (1 << 7) | (1 << 11))) | keep
        self.dp = v >> 16
        v = self._pop32()
        self.xar[0] = (self.xar[0] & 0xFFFF0000) | (v & M16)
        self.xar[1] = (self.xar[1] & 0xFFFF0000) | (v >> 16)
        self.p = self._pop32()
        self.acc = self._pop32()
        v = self._pop32()
        self.st0 = v & M16
        self.xt = (self.xt & M16) | (v & 0xFFFF0000)
        self.sp = (self.sp - 1) & M16

    def trap(self, k: int, ret: int, as_intr: bool) -> None:
        if k == V_ILLEGAL:
            self.halted = f"illegal instruction at 0x{self.pc:06x}"
        self.enter(k, ret)

    def idle(self) -> None:
        self.idlestat = 1
        self.idle_wait = True

    def pending(self) -> int:
        """Highest-priority interrupt the CPU would take now (1..16), or 0."""
        if self.intm:
            return 0
        bits = self.ifr & self.ier
        if not bits:
            return 0
        return (bits & -bits).bit_length()

    def take(self, k: int) -> None:
        self.ifr &= ~(1 << (k - 1)) & M16
        if self.ack_hook is not None:
            self.ack_hook(k)
        self.enter(k, self.pc)

    # ---------------------------------------------------------------- running
    def run(self, cycles: int) -> int:
        """Execute about ``cycles`` cycles (one per instruction, N+1 per
        repeated instruction); returns the cycles used.  Stops early on IDLE
        or when :attr:`halted` is set."""
        clock = self.clock
        start = clock.cycles
        end = start + cycles
        cache = self.icache
        decode = self.decode
        n = 0
        while clock.cycles < end:
            if self.ifr and not self.intm:
                k = self.pending()
                if k:
                    self.take(k)
            if self.idle_wait or self.halted:
                break
            pc = self.pc
            if pc == self.wait_pc and self.wait_left:
                break
            ins = cache.get(pc)
            if ins is None:
                ins = decode(pc)
            nxt = (pc + ins.size) & M22
            self.pc = nxt
            ins.fn(self)
            n += 1
            if self.pc != nxt:
                clock.cycles += 3   # a taken branch refills the pipeline
            if self.rpt_pending:
                self.rpt_pending = False
                pc = self.pc
                ins2 = cache.get(pc) or decode(pc)
                self.pc = (pc + ins2.size) & M22
                cnt = self.rptc
                if ins2.key == "NOP" and ins2.size == 1 and (self.mem[pc] & 0xFF) == 0:
                    clock.cycles += cnt + 1
                elif ins2.repeatable and cnt:
                    for i in range(cnt + 1):
                        self.rpt_i = i
                        self.rpt_last = i == cnt
                        ins2.fn(self)
                    self.rpt_i = 0
                    self.rpt_last = True
                    clock.cycles += cnt + 1
                else:
                    ins2.fn(self)
                    clock.cycles += 1
                n += 1
                continue
            clock.cycles += 1
        self.insn_count += n
        return clock.cycles - start


# ------------------------------------------------------------------ accelerator
REGISTERS = ("acc", "p", "xt", "pc", "rpc", "sp", "dp", "sxm", "ovm", "tc", "c", "z", "n", "v", "pm", "ovc",
             "intm", "dbgm", "page0", "vmap", "spa", "loop", "eallow", "idlestat", "amode", "objmode", "m0m1map",
             "xf", "arp", "ier", "ifr", "dbgier", "dbgstat", "rptc", "rpt_pending", "rpt_i", "rpt_last", "bio",
             "idle_wait", "wait_pc", "wait_left", "insn_count")
_BOOLS = ("rpt_pending", "rpt_last", "idle_wait")


class _Xar:
    """``cpu.xar`` of an accelerated CPU: the registers live in the C core."""
    __slots__ = ("core",)

    def __init__(self, core):
        self.core = core

    def __getitem__(self, i):
        return self.core.get_xar(i)

    def __setitem__(self, i, v):
        self.core.set_xar(i, v & M32)

    def __len__(self):
        return 8

    def __iter__(self):
        return (self.core.get_xar(i) for i in range(8))

    def __eq__(self, other):
        return list(self) == list(other)


def _prop(name):
    def get(self):
        return getattr(self._core, name)

    def put(self, v):
        setattr(self._core, name, int(v))
    return property(get, put)


class AcceleratedC28x(C28x):
    """A C28x whose register file and execution live in the native core
    (``marsemu.c28x._c28x``); decoding, unported instructions, interrupt
    entry and all peripherals stay in Python."""

    for _n in REGISTERS:
        locals()[_n] = _prop(_n)
    del _n

    @property
    def xar(self):
        return self._xar

    @xar.setter
    def xar(self, values):
        for i, v in enumerate(values):
            self._core.set_xar(i, v & M32)

    @property
    def st0(self):
        return self._core.st0

    @st0.setter
    def st0(self, v):
        self._core.st0 = v

    @property
    def st1(self):
        return self._core.st1

    @st1.setter
    def st1(self, v):
        self._core.st1 = v

    @property
    def halted(self):
        return self._halted

    @halted.setter
    def halted(self, v):
        self._halted = v
        self._core.halted = 1 if v else 0

    def iret(self) -> None:
        self._core.iret()

    def run(self, cycles: int) -> int:
        return self._core.run(cycles)

    def _c_decode(self, pc: int):
        """What the core needs of the instruction at ``pc``: (key, size, L, A,
        C, S, n, m, B, K, flags)."""
        ins = self.icache.get(pc) or self.decode(pc)
        w1 = self.mem[pc]
        w2 = self.mem[(pc + 1) & M22]
        m = match(w1, w2)
        flags = 0
        if m is None or w1 in (0x0000, 0xFFFF):
            return (-1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 4)
        k, f, size, _ = m
        if ins.repeatable:
            flags |= 1
        if pc in self.code_hooks:
            flags |= 2
        return (_KEY_INDEX[k], size, f.get("L", 0), f.get("A", 0), f.get("C", 0), f.get("S", 0), f.get("n", 0),
                f.get("m", 0), f.get("B", 0), f.get("K", 0), flags)

    def _c_take(self, k: int) -> int:
        """Interrupt k is taken (the core saves the context): acknowledge it
        and return its vector."""
        if self.ack_hook is not None:
            self.ack_hook(k)
        return self._vector(k)

    def _c_py(self, pc: int) -> None:
        ins = self.icache.get(pc) or self.decode(pc)
        ins.fn(self)


def _key_index():
    from .isa import TABLE
    return {k: i for i, (k, *_) in enumerate(TABLE)}


_KEY_INDEX = _key_index()


def accelerate(cpu: C28x) -> bool:
    """Move ``cpu`` onto the native core if it is built; returns whether it was."""
    if isinstance(cpu, AcceleratedC28x):
        return True
    try:
        from . import _c28x
    except ImportError:
        return False
    from .isa import TABLE
    state = {n: getattr(cpu, n) for n in REGISTERS}
    xar = list(cpu.xar)
    halted = cpu.halted
    io_list = [None] * len(cpu.ptype)
    for pg, p in cpu.io.items():
        io_list[pg] = p
    cpu._io_list = io_list
    cpu.__class__ = AcceleratedC28x
    for n in REGISTERS + ("xar", "halted"):
        cpu.__dict__.pop(n, None)
    core = _c28x.Core(cpu.mem, cpu.ptype, io_list, cpu.clock, cpu._c_decode, cpu._c_py, None,
                      cpu._c_take, lambda a, v: cpu.rom_write(a, v) if cpu.rom_write is not None else None,
                      lambda v: cpu.iack_hook(v) if cpu.iack_hook is not None else None, None,
                      [k for k, *_ in TABLE])
    cpu._core = core
    cpu._xar = _Xar(core)
    for n, v in state.items():
        setattr(core, n, int(v))
    for i, v in enumerate(xar):
        core.set_xar(i, v)
    cpu._halted = halted
    core.halted = 1 if halted else 0
    return True
