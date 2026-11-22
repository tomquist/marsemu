"""The F2803x Control Law Accelerator (CLA, type 0).

An independent 32-bit floating-point coprocessor sharing the C28x address
map.  Its program sits in L3 (0x9000-0x9FFF); every instruction is 32 bits,
the opcode in the upper word (at the odd address).  Tasks 1-8 start on a
peripheral interrupt (ADCINTx, ePWMx, CPU timer 0) or a software force
(``MIFRC`` or the CPU's ``IACK``), run to ``MSTOP`` and then raise
CLA1_INTx (PIE 11.x).

A task here runs to completion when it starts; its memory effects appear at
once and its end-of-task interrupt after the cycles it took.  The pipeline
effects that matter to compiled code are modelled: the three instructions
after ``MBCNDD``/``MCCNDD``/``MRCNDD`` always execute, those branches decide
on the flags as they were four instructions earlier, and loads of MAR0/MAR1
take effect for the fourth instruction after the load (TRM 10.5.2).
Encodings follow the TRM's CLA instruction set (SPRUI10 ch. 10).
"""
from __future__ import annotations

import logging
import math
import struct
from typing import Callable, Dict, List, Optional

from .machine import C28xPeripheral

log = logging.getLogger("emu.cla")

PROG_BASE = 0x9000
_F = struct.Struct("<f")
_I = struct.Struct("<I")
FLT_MAX_BITS = 0x7F7FFFFF
FLT_MAX = _F.unpack(_I.pack(FLT_MAX_BITS))[0]
FLT_MIN = 1.1754943508222875e-38
M32 = 0xFFFFFFFF


def tobits_(v):
    """Register value (an int, or a float left by compiled code) as bits."""
    if type(v) is int:
        return v
    from .cla_jit import tobits
    return tobits(v)


def b2f(b: int) -> float:
    """CLA float from bits; denormals read as zero."""
    if not (b & 0x7F800000):
        return -0.0 if b & 0x80000000 else 0.0
    return _F.unpack(_I.pack(b & M32))[0]


class _Res:
    __slots__ = ("lvf", "luf")


def f2b(cla, x: float) -> int:
    """Bits of a result, saturating on overflow (LVF) and flushing
    underflow to zero (LUF)."""
    if x != x:   # NaN cannot come out of the hardware; treat as overflow
        cla.lvf = 1
        return FLT_MAX_BITS
    ax = abs(x)
    if ax > FLT_MAX:
        try:
            r = _I.unpack(_F.pack(x))[0]
            if (r & 0x7F800000) != 0x7F800000:
                return r
        except OverflowError:
            pass
        cla.lvf = 1
        return FLT_MAX_BITS | (0x80000000 if x < 0 else 0)
    if ax and ax < FLT_MIN:
        cla.luf = 1
        return 0x80000000 if x < 0 else 0
    try:
        r = _I.unpack(_F.pack(x))[0]
    except OverflowError:
        cla.lvf = 1
        return FLT_MAX_BITS | (0x80000000 if x < 0 else 0)
    if (r & 0x7F800000) == 0x7F800000:
        cla.lvf = 1
        return FLT_MAX_BITS | (r & 0x80000000)
    return r


def _cond(k: int) -> Callable:
    """CNDF test on a flags tuple (nf, zf, tf, luf, lvf)."""
    return {
        0: lambda f: not f[1], 1: lambda f: f[1], 2: lambda f: not f[1] and not f[0], 3: lambda f: not f[0],
        4: lambda f: f[0], 5: lambda f: f[1] or f[0], 10: lambda f: f[2], 11: lambda f: not f[2],
        12: lambda f: f[3], 13: lambda f: f[4], 14: lambda f: True, 15: lambda f: True,
    }.get(k, lambda f: False)


COND = ("NEQ", "EQ", "GT", "GEQ", "LT", "LEQ", "?6", "?7", "?8", "?9", "TF", "NTF", "LU", "LV", "UNC", "UNCF")


def _sat_int(x: float, lo: int, hi: int, rnd: bool) -> int:
    if x != x:
        return 0
    v = math.floor(x + 0.5) if rnd else math.trunc(x)
    return max(lo, min(hi, int(v)))


class ClaInsn:
    __slots__ = ("fn", "text", "kind", "target", "cond")

    def __init__(self, fn, text, kind=None, target=0, cond=None):
        self.fn = fn
        self.text = text
        self.kind = kind          # None, "br", "call", "ret", "stop"
        self.target = target
        self.cond = cond


class Cla(C28xPeripheral):
    """CLA1 control registers (0x1400) and the execution engine."""

    reg_names = {**{i: f"MVECT{i + 1}" for i in range(8)}, 0x10: "MCTL", 0x11: "MMEMCFG", 0x14: "MPISRCSEL1",
                 0x15: "MPISRCSEL1_H", 0x20: "MIFR", 0x21: "MIOVF", 0x22: "MIFRC", 0x23: "MICLR",
                 0x24: "MICLROVF", 0x25: "MIER", 0x26: "MIRUN", 0x28: "MPC", 0x2A: "MAR0", 0x2B: "MAR1",
                 0x2E: "MSTF", 0x2F: "MSTF_H", 0x30: "MR0", 0x31: "MR0_H", 0x34: "MR1", 0x35: "MR1_H", 0x38: "MR2",
                 0x39: "MR2_H", 0x3C: "MR3", 0x3D: "MR3_H"}
    protected = True
    MAX_STEPS = 200000

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.cache: Dict[int, ClaInsn] = {}
        self.mr = [0, 0, 0, 0]
        self.mar = [0, 0]
        self.nf = self.zf = self.tf = self.luf = self.lvf = 0
        self.rndf32 = 0
        self.meallow = 0
        self.mpc = 0
        self.mrpc = 0
        self.mifr = self.miovf = self.mier = self.mirun = 0
        self.running: Optional[int] = None
        self.task_counts = [0] * 9
        self.task_cycles = [0] * 9
        self._end_ev = None
        self.faults = 0
        #: compile tasks to Python (see cla_jit); False = always interpret
        self.jit = True
        #: run tasks in the native core when the CPU has one
        self.native = True
        self._in_core = False
        self._compiled: Dict[int, object] = {}
        self._fallback_pc = None
        raw = memoryview(m.cpu.mem).cast("B")
        self._fv = raw.cast("f")
        self._uv = raw.cast("I")

    def reset(self):
        super().reset()
        self.cache.clear()
        self._compiled.clear()
        self.mr = [0, 0, 0, 0]
        self.mar = [0, 0]
        self.nf = self.zf = self.tf = self.luf = self.lvf = 0
        self.mifr = self.miovf = self.mier = self.mirun = 0
        self.running = None
        if self._end_ev:
            self._end_ev.cancel()
        self._end_ev = None

    # ------------------------------------------------------------- triggers
    def peripheral_irq(self, source: str, n: int) -> None:
        """A peripheral interrupt that may start a task: ``source`` is
        "adc" (ADCINTn), "epwm" (ePWMn INT) or "tint0"."""
        if not 1 <= n <= 8 and source != "tint0":
            return
        sel = self.r(0x14) | (self.r(0x15) << 16)
        if source == "tint0":
            if (sel >> 28) & 0xF == 2:
                self.force(1 << 7)
            return
        s = (sel >> (4 * (n - 1))) & 0xF
        if (source == "adc" and s == 0) or (source == "epwm" and s == 2 and n <= 7):
            self.force(1 << (n - 1))

    def force(self, bits: int) -> None:
        ov = self.mifr & bits
        self.miovf |= ov
        self.mifr |= bits & 0xFF
        self._dispatch()

    def iack(self, value: int) -> None:
        if self.r(0x10) & 0x4:   # MCTL.IACKE
            self.force(value & 0xFF)

    def _dispatch(self) -> None:
        if self.running is not None:
            return
        ready = self.mifr & self.mier
        if not ready:
            return
        bit = ready & -ready
        task = bit.bit_length()
        self.mifr &= ~bit
        self.mirun = bit
        self.running = task
        cycles = self._run_task(task)
        self.task_counts[task] += 1
        self.task_cycles[task] += cycles
        pie = self.m.pie
        if not (pie.ier[11] >> (task - 1)) & 1 and not self.mifr & self.mier:
            # nobody waits for the end-of-task interrupt: the task is done at once
            self._task_end()
            return
        self._end_ev = self.m.clock.after(max(1, cycles + 4), self._task_end, "cla-end")

    def _task_end(self) -> None:
        self._end_ev = None
        task = self.running
        self.running = None
        self.mirun = 0
        if task:
            self.m.pie.raise_irq(11, task)
        self._dispatch()

    # ------------------------------------------------------------ registers
    def read(self, off):
        if off >= 0x28:
            self.sync_from_core()
        if off == 0x20:
            return self.mifr
        if off == 0x21:
            return self.miovf
        if off == 0x25:
            return self.mier
        if off == 0x26:
            return self.mirun
        if off == 0x28:
            return self.mpc
        if off in (0x2A, 0x2B):
            return self.mar[off - 0x2A]
        if off in (0x2E, 0x2F):
            v = self.mstf()
            return v >> 16 if off & 1 else v & 0xFFFF
        if 0x30 <= off < 0x40:
            i, r = divmod(off - 0x30, 4)
            if r < 2:
                v = tobits_(self.mr[i])
                return (v >> 16) & 0xFFFF if r else v & 0xFFFF
        if off in (0x22, 0x23, 0x24):
            return 0
        return super().read(off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        if off == 0x22:   # MIFRC
            self.force(v & 0xFF)
            return
        if off == 0x23:   # MICLR
            self.mifr &= ~v
            return
        if off == 0x24:
            self.miovf &= ~v
            return
        if off == 0x25:
            self.mier = v & 0xFF
            self._dispatch()
            return
        if off == 0x10:   # MCTL
            if v & 1 or v & 2:   # HARDRESET / SOFTRESET
                self.mier = 0
                if v & 1:
                    self.mifr = self.miovf = 0
            self.regs[0x10] = v & 0x4
            return
        if off == 0x11:
            self.cache.clear()
            self._compiled.clear()
        if off < 8:
            v &= 0xFFF
        super().write(off, v)

    def mstf(self) -> int:
        return (self.lvf | self.luf << 1 | self.nf << 2 | self.zf << 3 | self.tf << 6 | self.rndf32 << 9
                | self.meallow << 11 | (self.mrpc & 0xFFF) << 12)

    def set_mstf(self, v: int) -> None:
        self.lvf, self.luf, self.nf, self.zf = v & 1, v >> 1 & 1, v >> 2 & 1, v >> 3 & 1
        self.tf, self.rndf32, self.meallow = v >> 6 & 1, v >> 9 & 1, v >> 11 & 1
        self.mrpc = (v >> 12) & 0xFFF

    # ------------------------------------------------------------ execution
    # helpers for compiled tasks (non-RAM addresses)
    @property
    def _adcr(self):
        return self.m.adc.result

    def _ldf(self, a):
        a &= 0xFFFE
        cpu = self.m.cpu
        return b2f(cpu.rd16(a) | (cpu.rd16(a + 1) << 16))

    def _ldi(self, a):
        a &= 0xFFFE
        cpu = self.m.cpu
        return cpu.rd16(a) | (cpu.rd16(a + 1) << 16)

    def _st32(self, a, v):
        a &= 0xFFFE
        cpu = self.m.cpu
        cpu.wr16(a, v & 0xFFFF)
        cpu.wr16(a + 1, (v >> 16) & 0xFFFF)

    def _run_task(self, task: int) -> int:
        """Run a task to MSTOP; returns the cycles it took."""
        entry = PROG_BASE + (self.r(task - 1) & 0xFFF)
        core = getattr(self.m.cpu, "_core", None)
        if core is not None and self.native:
            from . import _c28x
            if not self._in_core:
                _c28x.cla_state(core, [tobits_(v) for v in self.mr], list(self.mar),
                                [self.nf, self.zf, self.tf, self.luf, self.lvf, self.rndf32], self.mrpc)
                self._in_core = True
            steps, unknown = _c28x.cla_run(core, entry)
            if unknown >= 0:
                self.faults += 1
                log.error("CLA: unknown instruction at 0x%04x", unknown)
            return steps
        if self.jit:
            fn = self._compiled.get(entry)
            if fn is None:
                from .cla_jit import Compiler, Unsupported
                try:
                    fn = Compiler(self.m.cpu).compile(entry)
                except Unsupported as e:
                    log.warning("CLA task at 0x%04x is interpreted: %s", entry, e)
                    fn = False
                self._compiled[entry] = fn
            if fn:
                self._fallback_pc = None
                cyc = fn(self, self.m.cpu)
                if self._fallback_pc is None:
                    return cyc
                return cyc + self._interpret(self._fallback_pc)
        return self._interpret(entry)

    def sync_from_core(self) -> None:
        """Bring the registers kept in the native core back into this object."""
        if self._in_core:
            from . import _c28x
            mr, mar, fl, mrpc = _c28x.cla_state(self.m.cpu._core)
            self.mr, self.mar = mr, mar
            self.nf, self.zf, self.tf, self.luf, self.lvf, self.rndf32 = fl
            self.mrpc = mrpc
            self._in_core = False

    def _interpret(self, pc: int) -> int:
        self.sync_from_core()
        cpu = self.m.cpu
        self.mr = [tobits_(v) for v in self.mr]
        hist = [self.flags()] * 4
        pending_mar: List[list] = []
        branch = None       # [remaining slots, target]
        steps = 0
        cache = self.cache
        while steps < self.MAX_STEPS:
            steps += 1
            if pending_mar:
                for p in list(pending_mar):
                    p[0] -= 1
                    if p[0] <= 0:
                        self.mar[p[1]] = p[2]
                        pending_mar.remove(p)
            ins = cache.get(pc)
            if ins is None:
                ins = self._decode(pc)
            hist.append(self.flags())
            del hist[0]
            self.mpc = (pc - PROG_BASE) & 0xFFF
            if ins.kind is None:
                r = ins.fn(self, cpu)
                if r is not None:       # a MAR load: takes effect three instructions later
                    pending_mar.append([3, r[0], r[1]])
            elif ins.kind == "stop":
                if branch is not None:
                    log.warning("CLA: MSTOP in a delay slot at 0x%04x", pc)
                for p in pending_mar:   # loads already in the pipeline complete
                    self.mar[p[1]] = p[2]
                return steps
            else:
                if branch is not None:
                    log.warning("CLA: branch in a delay slot at 0x%04x", pc)
                taken = ins.cond(hist[0])
                if taken:
                    if ins.kind == "ret":
                        tgt = PROG_BASE + self.mrpc
                    else:
                        tgt = ins.target
                    if ins.kind == "call":
                        self.mrpc = (pc + 8 - PROG_BASE) & 0xFFF
                    branch = [4, tgt]
            if branch is not None:
                branch[0] -= 1
                if branch[0] == 0:
                    pc = branch[1]
                    branch = None
                    continue
            pc += 2
        self.faults += 1
        log.error("CLA task %d ran away (no MSTOP after %d instructions)", task, steps)
        return steps

    def flags(self):
        return (self.nf, self.zf, self.tf, self.luf, self.lvf)

    def disasm(self, addr: int) -> str:
        return self._decode(addr).text

    def _decode(self, pc: int) -> ClaInsn:
        cpu = self.m.cpu
        lsw = cpu.mem[pc]
        msw = cpu.mem[pc + 1]
        ins = decode(msw, lsw, pc)
        self.cache[pc] = ins
        return ins


# ---------------------------------------------------------------- decoder
def _mem(mode: int, lsw: int, size: int):
    """(address function with post-increment, text) for a 4-bit address mode."""
    if mode == 0:
        return (lambda c: lsw), f"@0x{lsw:04x}"
    if mode in (1, 2):
        r = mode - 1
        inc = lsw - 0x10000 if lsw & 0x8000 else lsw

        def ea(c):
            a = c.mar[r]
            c.mar[r] = (a + inc) & 0xFFFF
            return a
        return ea, (f"*MAR{r}++" if inc == size // 16 else f"*MAR{r}[{inc}]++")
    if mode in (5, 6):
        r = mode - 5
        return (lambda c: (c.mar[r] + lsw) & 0xFFFF), f"*MAR{r}+[{lsw}]"
    return None, f"?addr{mode}"


def _rd32(cpu, a):
    a &= 0xFFFE
    return cpu.rd16(a) | (cpu.rd16(a + 1) << 16)


def _wr32(cpu, a, v):
    a &= 0xFFFE
    cpu.wr16(a, v & 0xFFFF)
    cpu.wr16(a + 1, (v >> 16) & 0xFFFF)


def _zn_f(c, bits):
    c.zf = 1 if not (bits & 0x7FFFFFFF) or not (bits & 0x7F800000) else 0
    c.nf = 1 if (bits & 0x80000000) and not c.zf else 0


def _zn_i(c, v):
    c.zf = 0 if v else 1
    c.nf = v >> 31


def decode(msw: int, lsw: int, pc: int) -> ClaInsn:   # noqa: C901
    top = msw >> 12
    if top <= 6:
        return _parallel_mem(msw, lsw)
    a = (msw >> 4) & 3
    addr = msw & 0xF
    hi10 = msw >> 6
    if hi10 == 0x1C0 or (msw & 0xFC00) == 0x7000:   # MMOV32 MRa,mem32,CNDF: 0111 00cn dfaa addr
        cnd = (msw >> 6) & 0xF
        ea, t = _mem(addr, lsw, 32)
        cond = _cond(cnd)

        def fn(c, cpu):
            ad = ea(c)
            if cond(c.flags()):
                v = _rd32(cpu, ad)
                c.mr[a] = v
                if cnd == 15:
                    _zn_f(c, v)
        return ClaInsn(fn, f"MMOV32 MR{a},{t},{COND[cnd]}")
    grp = msw & 0xFFC0
    if grp in (0x7400, 0x7440, 0x7480, 0x74C0, 0x7500, 0x7540, 0x7580, 0x75C0):
        ea, t = _mem(addr, lsw, 32 if grp < 0x7500 else 16)
        if grp == 0x7400:
            def fn(c, cpu):
                ad = ea(c)
                v = _rd32(cpu, ad)
                c.mr[a] = v
                _wr32(cpu, ad + 2, v)
                _zn_f(c, v)
            return ClaInsn(fn, f"MMOVD32 MR{a},{t}")
        if grp == 0x7440:
            def fn(c, cpu):
                v = _rd32(cpu, ea(c))
                c.mr[a] = f2b(c, float(v - (1 << 32) if v >> 31 else v))
            return ClaInsn(fn, f"MI32TOF32 MR{a},{t}")
        if grp == 0x7480:
            def fn(c, cpu):
                c.mr[a] = f2b(c, float(_rd32(cpu, ea(c))))
            return ClaInsn(fn, f"MUI32TOF32 MR{a},{t}")
        if grp == 0x74C0:
            def fn(c, cpu):
                _wr32(cpu, ea(c), c.mr[a])
            return ClaInsn(fn, f"MMOV32 {t},MR{a}")
        if grp == 0x7500:
            def fn(c, cpu):
                v = cpu.rd16(ea(c))
                c.mr[a] = f2b(c, float(v - 0x10000 if v & 0x8000 else v))
            return ClaInsn(fn, f"MI16TOF32 MR{a},{t}")
        if grp == 0x7540:
            def fn(c, cpu):
                c.mr[a] = f2b(c, float(cpu.rd16(ea(c))))
            return ClaInsn(fn, f"MUI16TOF32 MR{a},{t}")
        if grp == 0x7580:
            def fn(c, cpu):
                v = cpu.rd16(ea(c))
                c.mr[a] = v
                _zn_i(c, v)
            return ClaInsn(fn, f"MMOVZ16 MR{a},{t}")

        def fn(c, cpu):
            cpu.wr16(ea(c), c.mr[a] & 0xFFFF)
        return ClaInsn(fn, f"MMOV16 {t},MR{a}")
    if (msw & 0xFF00) == 0x7600:   # MMOV16 MARx,mem16 / mem16,MARx
        r = (msw >> 6) & 1
        ea, t = _mem(addr, lsw, 16)
        if msw & 0x80:
            def fn(c, cpu):
                cpu.wr16(ea(c), c.mar[r])
            return ClaInsn(fn, f"MMOV16 {t},MAR{r}")

        def fn(c, cpu):
            return (r, cpu.rd16(ea(c)))
        return ClaInsn(fn, f"MMOV16 MAR{r},{t}")
    if (msw & 0xFFF0) == 0x7700:
        ea, t = _mem(addr, lsw, 32)

        def fn(c, cpu):
            c.set_mstf(_rd32(cpu, ea(c)))
        return ClaInsn(fn, f"MMOV32 MSTF,{t}")
    if (msw & 0xFFF0) == 0x7740:
        ea, t = _mem(addr, lsw, 32)

        def fn(c, cpu):
            _wr32(cpu, ea(c), c.mstf())
        return ClaInsn(fn, f"MMOV32 {t},MSTF")
    b2 = (msw >> 2) & 3
    a0 = msw & 3
    imm = lsw << 16
    fimm = b2f(imm)
    if (msw & 0xFFF0) in (0x7780, 0x77C0, 0x7800):
        op = {0x7780: "MMPYF32", 0x77C0: "MADDF32", 0x7800: "MSUBF32"}[msw & 0xFFF0]

        def fn(c, cpu):
            x = b2f(c.mr[b2])
            r = fimm * x if op == "MMPYF32" else (fimm + x if op == "MADDF32" else fimm - x)
            c.mr[a0] = f2b(c, r)
        return ClaInsn(fn, f"{op} MR{a0},#0x{lsw:04x},MR{b2}")
    if (msw & 0xFFFC) == 0x7840:
        def fn(c, cpu):
            c.mr[a0] = imm
        return ClaInsn(fn, f"MMOVIZ MR{a0},#0x{lsw:04x}")
    if (msw & 0xFFFC) == 0x7880:
        def fn(c, cpu):
            c.mr[a0] = (c.mr[a0] & 0xFFFF0000) | lsw
        return ClaInsn(fn, f"MMOVXI MR{a0},#0x{lsw:04x}")
    if (msw & 0xFFFC) in (0x78C0, 0x7900, 0x7940):
        kind = {0x78C0: "MCMPF32", 0x7900: "MMAXF32", 0x7940: "MMINF32"}[msw & 0xFFFC]

        def fn(c, cpu):
            x = b2f(c.mr[a0])
            c.zf = 1 if x == fimm else 0
            c.nf = 1 if x < fimm else 0
            if (kind == "MMAXF32" and x < fimm) or (kind == "MMINF32" and x > fimm):
                c.mr[a0] = imm
        return ClaInsn(fn, f"{kind} MR{a0},#0x{lsw:04x}")
    if (msw & 0xFFF0) in (0x7980, 0x7990):
        cnd = msw & 0xF
        off = lsw - 0x10000 if lsw & 0x8000 else lsw
        tgt = (pc + off - PROG_BASE) % 0x1000 + PROG_BASE
        kind = "br" if (msw & 0xFFF0) == 0x7980 else "call"
        return ClaInsn(None, f"{'MBCNDD' if kind == 'br' else 'MCCNDD'} 0x{tgt:04x},{COND[cnd]}", kind, tgt,
                       _cond(cnd))
    if (msw & 0xFFF0) == 0x79A0:
        cnd = msw & 0xF
        return ClaInsn(None, f"MRCNDD {COND[cnd]}", "ret", 0, _cond(cnd))
    if msw == 0x79C0:
        fl, va = lsw >> 8, lsw & 0xFF

        def fn(c, cpu):
            for bit, name in ((0, "lvf"), (1, "luf"), (2, "nf"), (3, "zf"), (6, "tf"), (7, "rndf32")):
                if fl >> bit & 1:
                    setattr(c, name, va >> bit & 1)
        return ClaInsn(fn, f"MSETFLG 0x{fl:02x},0x{va:02x}")
    la, lb, lc = lsw & 3, (lsw >> 2) & 3, (lsw >> 4) & 3
    ld, le, lf = (lsw >> 6) & 3, (lsw >> 8) & 3, (lsw >> 10) & 3
    cndl = (lsw >> 4) & 0xF
    if msw in (0x7A00, 0x7A40):
        sub = msw == 0x7A40

        def fn(c, cpu):
            mr = c.mr
            p = b2f(mr[lb]) * b2f(mr[lc])
            s = b2f(mr[le]) - b2f(mr[lf]) if sub else b2f(mr[le]) + b2f(mr[lf])
            mr[la] = f2b(c, p)
            mr[ld] = f2b(c, s)
        return ClaInsn(fn, f"MMPYF32 MR{la},MR{lb},MR{lc}||{'MSUBF32' if sub else 'MADDF32'} MR{ld},MR{le},MR{lf}")
    if msw == 0x7A80:
        cond = _cond(cndl)

        def fn(c, cpu):
            v = c.mr[lb]
            if cond(c.flags()):
                v ^= 0x80000000
            c.mr[la] = v
            if cndl == 15:
                _zn_f(c, v)
        return ClaInsn(fn, f"MNEGF32 MR{la},MR{lb},{COND[cndl]}")
    if msw == 0x7AC0:
        cond = _cond(cndl)

        def fn(c, cpu):
            if cond(c.flags()):
                c.mr[la] = c.mr[lb]
                if cndl == 15:
                    _zn_f(c, c.mr[la])
        return ClaInsn(fn, f"MMOV32 MR{la},MR{lb},{COND[cndl]}")
    if msw == 0x7B00:
        cond = _cond(cndl)

        def fn(c, cpu):
            if cond(c.flags()):
                c.mr[la], c.mr[lb] = c.mr[lb], c.mr[la]
        return ClaInsn(fn, f"MSWAPF MR{la},MR{lb},{COND[cndl]}")
    if msw in (0x7B40, 0x7B80, 0x7BC0):
        n = ((lsw >> 2) & 0x1F) + 1
        kind = {0x7B40: "MASR32", 0x7B80: "MLSR32", 0x7BC0: "MLSL32"}[msw]

        def fn(c, cpu):
            v = c.mr[la]
            if kind == "MASR32":
                v = ((v - (1 << 32) if v >> 31 else v) >> n) & M32
            elif kind == "MLSR32":
                v = v >> n
            else:
                v = (v << n) & M32
            c.mr[la] = v
            _zn_i(c, v)
        return ClaInsn(fn, f"{kind} MR{la},#{n}")
    if (msw & 0xFFE0) == 0x7C00 or (msw & 0xFFE0) == 0x7CE0 or msw in (0x7C20, 0x7C40, 0x7C60, 0x7C80, 0x7CA0,
                                                                         0x7CC0):
        kind = {0x7C00: "MMPYF32", 0x7C20: "MADDF32", 0x7C40: "MSUBF32", 0x7C60: "MAND32", 0x7C80: "MOR32",
                0x7CA0: "MXOR32", 0x7CC0: "MADD32", 0x7CE0: "MSUB32"}.get(msw)
        if kind is not None:
            def fn(c, cpu):
                mr = c.mr
                x, y = mr[lb], mr[lc]
                if kind == "MMPYF32":
                    mr[la] = f2b(c, b2f(x) * b2f(y))
                elif kind == "MADDF32":
                    mr[la] = f2b(c, b2f(x) + b2f(y))
                elif kind == "MSUBF32":
                    mr[la] = f2b(c, b2f(x) - b2f(y))
                else:
                    if kind == "MAND32":
                        v = x & y
                    elif kind == "MOR32":
                        v = x | y
                    elif kind == "MXOR32":
                        v = x ^ y
                    elif kind == "MADD32":
                        v = (x + y) & M32
                    else:
                        v = (x - y) & M32
                    mr[la] = v
                    _zn_i(c, v)
            return ClaInsn(fn, f"{kind} MR{la},MR{lb},MR{lc}")
    if 0x7D00 <= msw <= 0x7FE0 and not msw & 0x1F or msw in (0x7F90, 0x7FB0) or (msw & 0xFFC0) == 0x7FC0:
        return _unary(msw, lsw, la, lb)
    return ClaInsn(_bad(msw, lsw, pc), f".long 0x{msw:04x}{lsw:04x}")


def _bad(msw, lsw, pc):
    def fn(c, cpu):
        c.faults += 1
        log.error("CLA: unknown instruction 0x%04x%04x at 0x%04x", msw, lsw, pc)
    return fn


def _unary(msw, lsw, la, lb):   # noqa: C901
    if msw in (0x7D00, 0x7D20, 0x7D40):
        kind = {0x7D00: "MCMPF32", 0x7D20: "MMAXF32", 0x7D40: "MMINF32"}[msw]

        def fn(c, cpu):
            x, y = b2f(c.mr[la]), b2f(c.mr[lb])
            c.zf = 1 if x == y else 0
            c.nf = 1 if x < y else 0
            if (kind == "MMAXF32" and x < y) or (kind == "MMINF32" and x > y):
                c.mr[la] = c.mr[lb]
        return ClaInsn(fn, f"{kind} MR{la},MR{lb}")
    if msw == 0x7F20:
        def fn(c, cpu):
            x, y = c.mr[la], c.mr[lb]
            sx = x - (1 << 32) if x >> 31 else x
            sy = y - (1 << 32) if y >> 31 else y
            c.zf = 1 if sx == sy else 0
            c.nf = 1 if sx < sy else 0
        return ClaInsn(fn, f"MCMP32 MR{la},MR{lb}")
    conv = {
        0x7D60: ("MF32TOI32", lambda c, x: _sat_int(b2f(x), -(1 << 31), (1 << 31) - 1, False) & M32),
        0x7DA0: ("MF32TOUI32", lambda c, x: _sat_int(b2f(x), 0, M32, False)),
        0x7DE0: ("MF32TOI16", lambda c, x: _sat_int(b2f(x), -32768, 32767, False) & M32),
        0x7E60: ("MF32TOI16R", lambda c, x: _sat_int(b2f(x), -32768, 32767, True) & M32),
        0x7EA0: ("MF32TOUI16", lambda c, x: _sat_int(b2f(x), 0, 65535, False)),
        0x7EC0: ("MF32TOUI16R", lambda c, x: _sat_int(b2f(x), 0, 65535, True)),
        0x7D80: ("MI32TOF32", lambda c, x: f2b(c, float(x - (1 << 32) if x >> 31 else x))),
        0x7DC0: ("MUI32TOF32", lambda c, x: f2b(c, float(x))),
        0x7E80: ("MI16TOF32", lambda c, x: f2b(c, float((x & 0xFFFF) - 0x10000 if x & 0x8000 else x & 0xFFFF))),
        0x7EE0: ("MUI16TOF32", lambda c, x: f2b(c, float(x & 0xFFFF))),
        0x7E00: ("MFRACF32", lambda c, x: f2b(c, math.fmod(b2f(x), 1.0))),
        0x7E20: ("MABSF32", None),
        0x7E40: ("MEISQRTF32", lambda c, x: f2b(c, 1.0 / math.sqrt(abs(b2f(x)))) if b2f(x) else _ovf(c, x)),
        0x7F00: ("MEINVF32", lambda c, x: f2b(c, 1.0 / b2f(x)) if b2f(x) else _ovf(c, x)),
    }
    if msw in conv:
        name, f = conv[msw]
        if msw == 0x7E20:
            def fn(c, cpu):
                v = c.mr[lb] & 0x7FFFFFFF
                c.mr[la] = v
                _zn_f(c, v)
        else:
            def fn(c, cpu):
                c.mr[la] = f(c, c.mr[lb])
        return ClaInsn(fn, f"{name} MR{la},MR{lb}")
    if msw == 0x7F40:
        cnd = lsw & 0xF
        cond = _cond(cnd)

        def fn(c, cpu):
            c.tf = 1 if cond(c.flags()) else 0
        return ClaInsn(fn, f"MTESTTF {COND[cnd]}")
    if msw == 0x7F80:
        return ClaInsn(None, "MSTOP", "stop")
    if msw == 0x7F60:
        return ClaInsn(lambda c, cpu: None, "MDEBUGSTOP")
    if msw == 0x7F90:
        def fn(c, cpu):
            c.meallow = 1
        return ClaInsn(fn, "MEALLOW")
    if msw == 0x7FB0:
        def fn(c, cpu):
            c.meallow = 0
        return ClaInsn(fn, "MEDIS")
    if msw == 0x7FA0:
        return ClaInsn(lambda c, cpu: None, "MNOP")
    if (msw & 0xFFC0) == 0x7FC0:
        r = (msw >> 5) & 1
        if msw & 0x10:
            a = msw & 3

            def fn(c, cpu):
                return (r, (c.mr[a] + lsw) & 0xFFFF)
            return ClaInsn(fn, f"MMOV16 MAR{r},MR{a},#0x{lsw:04x}")

        def fn(c, cpu):
            return (r, lsw)
        return ClaInsn(fn, f"MMOVI16 MAR{r},#0x{lsw:04x}")
    return ClaInsn(_bad(msw, lsw, 0), f".long 0x{msw:04x}{lsw:04x}")


def _ovf(c, x):
    c.lvf = 1
    return FLT_MAX_BITS | (x & 0x80000000)


def _parallel_mem(msw, lsw):
    """Math with a parallel 32-bit move: 0xxx ffee ddaa addr."""
    op = msw >> 12
    ff, ee, dd, aa = (msw >> 10) & 3, (msw >> 8) & 3, (msw >> 6) & 3, (msw >> 4) & 3
    ea, t = _mem(msw & 0xF, lsw, 32)
    store = op >= 4
    kind = ("MMPYF32", "MADDF32", "MSUBF32", "MMACF32", "MMPYF32", "MADDF32", "MSUBF32")[op]
    base = op % 4 if op < 4 else op - 4

    def fn(c, cpu):
        mr = c.mr
        e, f = b2f(mr[ee]), b2f(mr[ff])
        if op == 3:
            acc = b2f(mr[3]) + b2f(mr[2])
            prod = e * f
            ad = ea(c)
            v = _rd32(cpu, ad)
            mr[3] = f2b(c, acc)
            mr[dd] = f2b(c, prod)
            mr[aa] = v
            _zn_f(c, v)
            return
        r = e * f if base == 0 else (e + f if base == 1 else e - f)
        ad = ea(c)
        if store:
            _wr32(cpu, ad, mr[aa])
            mr[dd] = f2b(c, r)
        else:
            v = _rd32(cpu, ad)
            mr[dd] = f2b(c, r)
            mr[aa] = v
            _zn_f(c, v)
    if op == 3:
        text = f"MMACF32 MR3,MR2,MR{dd},MR{ee},MR{ff}||MMOV32 MR{aa},{t}"
    elif store:
        text = f"{kind} MR{dd},MR{ee},MR{ff}||MMOV32 {t},MR{aa}"
    else:
        text = f"{kind} MR{dd},MR{ee},MR{ff}||MMOV32 MR{aa},{t}"
    return ClaInsn(fn, text)
