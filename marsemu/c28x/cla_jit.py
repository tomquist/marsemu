"""Compile CLA tasks to Python functions.

The control tasks run tens of thousands of times a second; interpreted one
instruction at a time they would cost tens of seconds per emulated second.
Here a task becomes one generated Python function:

* registers are locals; a register holds a Python ``float`` after a float
  operation and an ``int`` (the 32-bit pattern) after an integer one, and is
  converted where an instruction needs the other kind;
* float32 RAM is read and written through ``float``/``uint32`` views of the
  memory array (the store rounds to single precision; arithmetic in between is
  done in double precision, which is more precise than the hardware but never
  less);
* the status flags are SSA-named locals, so a delayed branch tests the flags
  as they were four instructions before it at no cost, and the history crosses
  block boundaries as a tuple;
* MAR0/MAR1 loads take effect for the fourth instruction after the load.

Anything the compiler does not handle makes the task fall back to the
interpreter in :mod:`.cla` (both are checked against each other in the tests).
"""
from __future__ import annotations

import logging
import math
import struct
from typing import Dict, List, Optional, Tuple

from .cla import PROG_BASE, FLT_MAX_BITS, b2f

log = logging.getLogger("emu.cla")

_F = struct.Struct("<f")
_I = struct.Struct("<I")


class Unsupported(Exception):
    pass


def tofloat(x):
    """A register value as a float (bits are reinterpreted)."""
    if type(x) is float:
        return x
    return b2f(x)


def tobits(x):
    """A register value as its 32-bit pattern (floats are rounded to single)."""
    if type(x) is int:
        return x
    try:
        r = _I.unpack(_F.pack(x))[0]
    except OverflowError:
        return FLT_MAX_BITS | (0x80000000 if x < 0 else 0)
    if (r & 0x7F800000) == 0x7F800000 and x == x and abs(x) != math.inf and abs(x) < 3.5e38:
        # a finite double that rounds past FLT_MAX saturates, as the hardware does
        return FLT_MAX_BITS | (r & 0x80000000)
    return r


def sat_int(x, lo, hi, rnd):
    if x != x:
        return 0
    if x == math.inf or x == -math.inf:
        return hi if x > 0 else lo
    v = math.floor(x + 0.5) if rnd else math.trunc(x)
    return lo if v < lo else (hi if v > hi else int(v))


def frac(x):
    return math.fmod(x, 1.0)


def einv(x):
    if x == 0:
        return math.copysign(3.4028234663852886e38, x)
    return 1.0 / x


def eisqrt(x):
    if x == 0:
        return 3.4028234663852886e38
    return 1.0 / math.sqrt(abs(x))


COND = {0: "not {z}", 1: "{z}", 2: "not {z} and not {n}", 3: "not {n}", 4: "{n}", 5: "{z} or {n}",
        10: "{t}", 11: "not {t}", 12: "{lu}", 13: "{lv}", 14: "True", 15: "True"}


# ------------------------------------------------------------------ parsing
def parse(msw: int, lsw: int, pc: int) -> tuple:
    """Instruction -> IR tuple ``(op, ...)``.  Mirrors :func:`.cla.decode`."""
    top = msw >> 12
    if top <= 6:
        ops = ("mpy", "add", "sub", "mac", "mpy", "add", "sub")
        return ("par", ops[top], top >= 4, (msw >> 6) & 3, (msw >> 8) & 3, (msw >> 10) & 3, (msw >> 4) & 3,
                msw & 0xF, lsw)
    a = (msw >> 4) & 3
    mode = msw & 0xF
    if (msw & 0xFC00) == 0x7000:
        return ("ld32", a, (msw >> 6) & 0xF, mode, lsw)
    grp = msw & 0xFFC0
    memops = {0x7400: "movd32", 0x7440: "i32tof_m", 0x7480: "ui32tof_m", 0x74C0: "st32", 0x7500: "i16tof_m",
              0x7540: "ui16tof_m", 0x7580: "movz16", 0x75C0: "st16"}
    if grp in memops:
        return (memops[grp], a, mode, lsw)
    if (msw & 0xFF00) == 0x7600:
        return ("st_mar" if msw & 0x80 else "ld_mar", (msw >> 6) & 1, mode, lsw)
    if (msw & 0xFFF0) == 0x7700:
        return ("ld_mstf", mode, lsw)
    if (msw & 0xFFF0) == 0x7740:
        return ("st_mstf", mode, lsw)
    b2, a0 = (msw >> 2) & 3, msw & 3
    if (msw & 0xFFF0) in (0x7780, 0x77C0, 0x7800):
        return ("fimm", {0x7780: "*", 0x77C0: "+", 0x7800: "-"}[msw & 0xFFF0], a0, b2, b2f(lsw << 16))
    if (msw & 0xFFFC) == 0x7840:
        return ("moviz", a0, lsw)
    if (msw & 0xFFFC) == 0x7880:
        return ("movxi", a0, lsw)
    if (msw & 0xFFFC) in (0x78C0, 0x7900, 0x7940):
        return ({0x78C0: "cmpfi", 0x7900: "maxfi", 0x7940: "minfi"}[msw & 0xFFFC], a0, b2f(lsw << 16), lsw << 16)
    if (msw & 0xFFF0) in (0x7980, 0x7990):
        off = lsw - 0x10000 if lsw & 0x8000 else lsw
        tgt = (pc + off - PROG_BASE) % 0x1000 + PROG_BASE
        return ("br" if (msw & 0xFFF0) == 0x7980 else "call", msw & 0xF, tgt)
    if (msw & 0xFFF0) == 0x79A0:
        return ("ret", msw & 0xF)
    if msw == 0x79C0:
        return ("setflg", lsw >> 8, lsw & 0xFF)
    la, lb, lc = lsw & 3, (lsw >> 2) & 3, (lsw >> 4) & 3
    ld, le, lf = (lsw >> 6) & 3, (lsw >> 8) & 3, (lsw >> 10) & 3
    cnd = (lsw >> 4) & 0xF
    if msw in (0x7A00, 0x7A40):
        return ("mpy_addsub", "-" if msw == 0x7A40 else "+", la, lb, lc, ld, le, lf)
    if msw == 0x7A80:
        return ("negf", la, lb, cnd)
    if msw == 0x7AC0:
        return ("movr", la, lb, cnd)
    if msw == 0x7B00:
        return ("swap", la, lb, cnd)
    if msw in (0x7B40, 0x7B80, 0x7BC0):
        return ({0x7B40: "asr", 0x7B80: "lsr", 0x7BC0: "lsl"}[msw], la, ((lsw >> 2) & 0x1F) + 1)
    ops3 = {0x7C00: "fmul", 0x7C20: "fadd", 0x7C40: "fsub", 0x7C60: "and", 0x7C80: "or", 0x7CA0: "xor",
            0x7CC0: "iadd", 0x7CE0: "isub"}
    if msw in ops3:
        return (ops3[msw], la, lb, lc)
    un = {0x7D00: "cmpf", 0x7D20: "maxf", 0x7D40: "minf", 0x7F20: "cmp32", 0x7D60: "f2i32", 0x7DA0: "f2ui32",
          0x7DE0: "f2i16", 0x7E60: "f2i16r", 0x7EA0: "f2ui16", 0x7EC0: "f2ui16r", 0x7D80: "i32tof",
          0x7DC0: "ui32tof", 0x7E80: "i16tof", 0x7EE0: "ui16tof", 0x7E00: "frac", 0x7E20: "absf",
          0x7E40: "eisqrt", 0x7F00: "einv"}
    if msw in un:
        return (un[msw], la, lb)
    if msw == 0x7F40:
        return ("testtf", lsw & 0xF)
    if msw == 0x7F80:
        return ("stop",)
    if msw in (0x7F60, 0x7FA0, 0x7F90, 0x7FB0):
        return ({0x7F60: "nop", 0x7FA0: "nop", 0x7F90: "eallow", 0x7FB0: "edis"}[msw],)
    if (msw & 0xFFC0) == 0x7FC0:
        r = (msw >> 5) & 1
        if msw & 0x10:
            return ("mar_reg", r, msw & 3, lsw)
        return ("mar_imm", r, lsw)
    raise Unsupported(f"0x{msw:04x}{lsw:04x}")


def reg_use(ir) -> Tuple[dict, set]:
    """(register -> "f"/"i"/"r" read kind, registers written) of an IR."""
    op = ir[0]
    R: dict = {}
    W: set = set()
    if op == "par":
        _, kop, store, dd, ee, ff, aa, mode, lsw = ir
        R[ee] = R[ff] = "f"
        if kop == "mac":
            R[2] = R[3] = "f"
            W |= {3, dd, aa}
        elif store:
            R.setdefault(aa, "r")
            W.add(dd)
        else:
            W |= {dd, aa}
    elif op in ("ld32", "movd32", "i32tof_m", "ui32tof_m", "i16tof_m", "ui16tof_m", "movz16", "moviz"):
        W.add(ir[1])
        if op == "ld32" and ir[2] not in (14, 15):
            R[ir[1]] = "r"
    elif op == "st32":
        R[ir[1]] = "r"
    elif op == "st16" or op == "movxi":
        R[ir[1]] = "i"
        if op == "movxi":
            W.add(ir[1])
    elif op == "mar_reg":
        R[ir[2]] = "i"
    elif op == "fimm":
        R[ir[3]] = "f"
        W.add(ir[2])
    elif op in ("cmpfi", "maxfi", "minfi"):
        R[ir[1]] = "f"
        if op != "cmpfi":
            W.add(ir[1])
    elif op == "mpy_addsub":
        _, sym, la, lb, lc, ld, le, lf = ir
        for r in (lb, lc, le, lf):
            R[r] = "f"
        W |= {la, ld}
    elif op in ("negf",):
        R[ir[2]] = "f"
        W.add(ir[1])
    elif op in ("movr", "swap"):
        R[ir[2]] = "r"
        R.setdefault(ir[1], "r")
        W.add(ir[1])
    elif op in ("asr", "lsr", "lsl"):
        R[ir[1]] = "i"
        W.add(ir[1])
    elif op in ("fmul", "fadd", "fsub"):
        R[ir[2]] = R[ir[3]] = "f"
        W.add(ir[1])
    elif op in ("and", "or", "xor", "iadd", "isub"):
        R[ir[2]] = R[ir[3]] = "i"
        W.add(ir[1])
    elif op in ("cmpf", "maxf", "minf"):
        R[ir[1]] = R[ir[2]] = "f"
    elif op == "cmp32":
        R[ir[1]] = R[ir[2]] = "i"
    elif op in ("f2i32", "f2ui32", "f2i16", "f2i16r", "f2ui16", "f2ui16r", "frac", "absf", "eisqrt", "einv"):
        R[ir[2]] = "f"
        W.add(ir[1])
    elif op in ("i32tof", "ui32tof", "i16tof", "ui16tof"):
        R[ir[2]] = "i"
        W.add(ir[1])
    return R, W


def wanted_kind(insns, start: int, reg: int) -> str:
    """How the code after ``start`` first uses ``reg``: "i" if as an integer."""
    for _, ir in insns[start:]:
        R, W = reg_use(ir)
        if reg in R:
            return "i" if R[reg] == "i" else "f"
        if reg in W:
            return "f"
    return "f"


# ------------------------------------------------------------------ compiler
class _Block:
    def __init__(self, start):
        self.start = start
        self.insns: List[Tuple[int, tuple]] = []
        self.exit = None       # ("fall", addr) | ("br", cond, tgt, fall) | ("call", cond, tgt, ret) | ("ret", cond, fall) | ("stop",)


class Compiler:
    MAX_INSNS = 4000

    def __init__(self, cpu, trace: bool = False, adc=None):
        self.cpu = cpu
        self.trace = trace
        self.io_env: dict = {}
        self.adc = adc

    def _ram(self, a: int) -> bool:
        from .cpu import RAM
        return self.cpu.ptype[a >> 6] == RAM

    def discover(self, entry: int) -> Dict[int, _Block]:
        mem = self.cpu.mem
        leaders = {entry}
        todo = [entry]
        seen = set()
        branch_ends = {}
        count = 0
        while todo:
            a = todo.pop()
            while a not in seen:
                seen.add(a)
                count += 1
                if count > self.MAX_INSNS:
                    raise Unsupported("task too large")
                ir = parse(mem[a + 1], mem[a], a)
                op = ir[0]
                if op == "stop":
                    break
                if op in ("br", "call", "ret"):
                    after = a + 8
                    for k in range(1, 4):
                        seen.add(a + 2 * k)
                    branch_ends[a] = after
                    nxt = []
                    if op in ("br", "call"):
                        nxt.append(ir[2])
                    if op != "br" or ir[1] not in (14, 15):
                        nxt.append(after)
                    if op == "call":
                        leaders.add(after)
                    for t in nxt:
                        leaders.add(t)
                        todo.append(t)
                    if op == "ret":
                        # where it returns is dynamic: the call sites' return points are leaders
                        if ir[1] not in (14, 15):
                            leaders.add(after)
                            todo.append(after)
                    break
                a += 2
        # build blocks
        blocks: Dict[int, _Block] = {}
        for start in sorted(leaders):
            b = _Block(start)
            a = start
            while True:
                if a != start and a in leaders:
                    b.exit = ("fall", a)
                    break
                ir = parse(mem[a + 1], mem[a], a)
                op = ir[0]
                if op == "stop":
                    b.insns.append((a, ir))
                    b.exit = ("stop",)
                    break
                if op in ("br", "call", "ret"):
                    b.insns.append((a, ir))
                    for k in range(1, 4):
                        s = a + 2 * k
                        sir = parse(mem[s + 1], mem[s], s)
                        if sir[0] in ("br", "call", "ret", "stop"):
                            raise Unsupported("branch or stop in a delay slot")
                        b.insns.append((s, sir))
                    after = a + 8
                    if op == "br":
                        b.exit = ("br", ir[1], ir[2], after)
                    elif op == "call":
                        b.exit = ("call", ir[1], ir[2], after)
                    else:
                        b.exit = ("ret", ir[1], after)
                    break
                b.insns.append((a, ir))
                a += 2
                if len(b.insns) > self.MAX_INSNS:
                    raise Unsupported("runaway block")
            blocks[start] = b
        return blocks

    # ---------------------------------------------------------- emission
    def compile(self, entry: int):
        blocks = self.discover(entry)
        lines = ["def task(c, cpu):",
                 "    mem = cpu.mem; fv = c._fv; uv = c._uv; rd16 = cpu.rd16; wr16 = cpu.wr16; ptype = cpu.ptype",
                 "    adcr = c._adcr",
                 "    r0, r1, r2, r3 = c.mr",
                 "    mar0, mar1 = c.mar",
                 "    n, z, t, lu, lv = c.nf, c.zf, c.tf, c.luf, c.lvf",
                 "    mrpc = c.mrpc",
                 "    H = ((n, z, t, lu, lv),) * 4",
                 "    cyc = 0",
                 f"    pc = {entry}",
                 "    while True:"]
        starts = sorted(blocks)
        bodies = {}
        self.need_hist = False
        # register kinds at block entries: a fixpoint over the control flow
        rets = [blk.exit[3] for blk in blocks.values() if blk.exit[0] == "call"]
        succ = {}
        for st, blk in blocks.items():
            ex = blk.exit
            if ex[0] == "fall":
                succ[st] = [ex[1]]
            elif ex[0] in ("br", "call"):
                succ[st] = [ex[2], ex[3]]
            elif ex[0] == "ret":
                succ[st] = rets + [ex[2]]
            else:
                succ[st] = []
        TOP = "?"
        entry_k = {st: [TOP] * 4 for st in starts}
        entry_k[entry] = [None] * 4
        for _ in range(50):
            changed = False
            for st in starts:
                ek = [None if k == TOP else k for k in entry_k[st]]
                if all(k == TOP for k in entry_k[st]) and st != entry:
                    continue
                self._block(blocks[st], ek)
                xk = self.exit_kinds
                for t in succ[st]:
                    if t not in entry_k:
                        continue
                    cur_k = entry_k[t]
                    new = [x if c_ == TOP else (c_ if c_ == x else None) for c_, x in zip(cur_k, xk)]
                    if new != cur_k:
                        entry_k[t] = new
                        changed = True
            if not changed:
                break
        for s in starts:
            ek = [None if k == "?" else k for k in entry_k[s]]
            bodies[s] = self._block(blocks[s], ek)
        if not self.need_hist:   # no branch looks further back than its own block: drop the history
            for s in starts:
                bodies[s] = [ln for ln in bodies[s] if not ln.startswith("H = ")]
        hi = lines.index("    H = ((n, z, t, lu, lv),) * 4")
        lines[hi] = "    H = ((n, z, t, lu, lv),) * 4" if self.need_hist else "    pass"

        def tree(keys, ind):
            if len(keys) <= 3:
                for i, k in enumerate(keys):
                    lines.append(" " * ind + f"{'if' if i == 0 else 'elif'} pc == {k}:")
                    if self.trace:
                        lines.append(" " * (ind + 4) + "c._trace.append((pc, tuple(tobits(x) for x in (r0, r1, r2, r3)), n, z))")
                    lines.extend(" " * (ind + 4) + ln for ln in bodies[k])
                lines.append(" " * ind + "else:")
                lines.append(" " * (ind + 4) + "c._fallback_pc = pc")
                lines.append(" " * (ind + 4) + "break")
                return
            mid = len(keys) // 2
            lines.append(" " * ind + f"if pc < {keys[mid]}:")
            tree(keys[:mid], ind + 4)
            lines.append(" " * ind + "else:")
            tree(keys[mid:], ind + 4)
        tree(starts, 8)
        lines.append("    c.mr = [r0, r1, r2, r3]")
        lines.append("    c.mar = [mar0, mar1]")
        lines.append("    c.nf, c.zf, c.tf, c.luf, c.lvf = n, z, t, lu, lv")
        lines.append("    c.mrpc = mrpc")
        lines.append("    return cyc")
        lines = self._dce(lines)
        src = "\n".join(lines)
        env = {"b2f_": b2f, "tofloat": tofloat, "tobits": tobits, "sat_int": sat_int, "frac": frac, "einv": einv,
               "eisqrt": eisqrt, "math": math, "PROG_BASE": PROG_BASE}
        env.update(self.io_env)
        code = compile(src, f"<cla task @0x{entry:04x}>", "exec")
        exec(code, env)
        fn = env["task"]
        fn.source = src
        return fn

    def _periph(self, a: int):
        from .cpu import IO
        cpu = self.cpu
        if cpu.ptype[a >> 6] != IO:
            return None
        p = cpu.io[a >> 6]
        if hasattr(p, "_find"):   # a page shared by several blocks
            p = p._find(a)
        return p

    def _io_read(self, a: int) -> str:
        """A direct peripheral read bound at compile time (no dispatch)."""
        p = self._periph(a)
        if p is None:
            return f"rd16({a})"
        from .f2803x_adc import AdcResult
        if isinstance(p, AdcResult) and a - p.base < 16:
            return f"adcr[{a - p.base}]"
        name = f"io_{len(self.io_env)}"
        self.io_env[name] = p.read
        return f"{name}({a - p.base})"

    def _io_write(self, a: int, expr: str) -> str:
        p = self._periph(a)
        if p is None:
            return f"wr16({a}, {expr})"
        name = f"io_{len(self.io_env)}"
        self.io_env[name] = p.write
        return f"{name}({a - p.base}, {expr})"

    @staticmethod
    def _dce(lines: List[str]) -> List[str]:
        """Drop assignments to SSA flag names (``n_3``, ``z_7``...) nobody reads."""
        import re
        pat = re.compile(r"^(\s*)([nzt]_\d+) = (.*)$")
        while True:
            used = {}
            for ln in lines:
                m = pat.match(ln)
                body = m.group(3) if m else ln
                for name in re.findall(r"\b[nzt]_\d+\b", body):
                    used[name] = True
            out = []
            changed = False
            for ln in lines:
                m = pat.match(ln)
                if m and m.group(2) not in used:
                    changed = True
                    continue
                out.append(ln)
            lines = out
            if not changed:
                return lines

    def _block(self, b: _Block, entry_kinds=(None, None, None, None)) -> List[str]:   # noqa: C901
        out: List[str] = []
        kind = list(entry_kinds)   # static kind of r0..r3: "f", "i" or None (unknown)
        self.exit_kinds = kind
        # flags after each instruction of the block: tuple of expressions
        flags_hist: List[tuple] = []
        cur = ("n", "z", "t", "lu", "lv")
        version = [0]
        mar_pending: List[list] = []   # [remaining, reg, temp name]

        def F(r):
            """expression for register r as float"""
            if kind[r] == "f":
                return f"r{r}"
            if kind[r] == "i":
                return f"b2f_(r{r})"
            return f"tofloat(r{r})"

        def B(r):
            if kind[r] == "i":
                return f"r{r}"
            return f"tobits(r{r})"

        def setf(r, expr):
            out.append(f"r{r} = {expr}")
            kind[r] = "f"

        def seti(r, expr):
            out.append(f"r{r} = {expr}")
            kind[r] = "i"

        def newflags(nexpr=None, zexpr=None, texpr=None):
            nonlocal cur
            version[0] += 1
            v = version[0]
            nn, zz, tt, ll, vv = cur
            if nexpr is not None:
                out.append(f"n_{v} = {nexpr}")
                nn = f"n_{v}"
            if zexpr is not None:
                out.append(f"z_{v} = {zexpr}")
                zz = f"z_{v}"
            if texpr is not None:
                out.append(f"t_{v} = {texpr}")
                tt = f"t_{v}"
            cur = (nn, zz, tt, ll, vv)

        def fl_zn_f(expr_var):
            newflags(f"1 if {expr_var} < 0.0 else 0", f"1 if {expr_var} == 0.0 else 0")

        def fl_zn_i(var):
            newflags(f"{var} >> 31", f"0 if {var} else 1")

        def cond_expr(k, fl):
            return COND[k].format(n=fl[0], z=fl[1], t=fl[2], lu=fl[3], lv=fl[4]) if k in COND else "False"

        def addr32(mode, lsw):
            """(expression for the word address, is_static_ram)"""
            if mode == 0:
                return str(lsw), self._ram(lsw)
            if mode in (1, 2):
                r = mode - 1
                inc = lsw - 0x10000 if lsw & 0x8000 else lsw
                tmp = f"a{len(out)}"
                out.append(f"{tmp} = mar{r}; mar{r} = (mar{r} + {inc}) & 0xFFFF")
                return tmp, False
            if mode in (5, 6):
                r = mode - 5
                return f"((mar{r} + {lsw}) & 0xFFFF)", False
            raise Unsupported(f"address mode {mode}")

        def ld32f(mode, lsw):
            a, ram = addr32(mode, lsw)
            if ram:
                return f"fv[{int(a) >> 1}]"
            if mode == 0:
                return f"c._ldf({a})"
            return f"(fv[{a} >> 1] if not ptype[{a} >> 6] else c._ldf({a}))"

        def ld32i(mode, lsw):
            a, ram = addr32(mode, lsw)
            if ram:
                return f"uv[{int(a) >> 1}]"
            if mode == 0:
                return f"c._ldi({a})"
            return f"(uv[{a} >> 1] if not ptype[{a} >> 6] else c._ldi({a}))"

        def st32(mode, lsw, r):
            a, ram = addr32(mode, lsw)
            if ram:
                if kind[r] == "f":
                    out.append(f"fv[{int(a) >> 1}] = r{r}")
                else:
                    out.append(f"uv[{int(a) >> 1}] = {B(r)}")
            else:
                out.append(f"c._st32({a}, {B(r)})")

        def ld16(mode, lsw):
            a, ram = addr32(mode, lsw)
            if ram:
                return f"mem[{a}]"
            if mode == 0:
                return self._io_read(lsw)
            return f"rd16({a})"

        def st16(mode, lsw, expr):
            a, ram = addr32(mode, lsw)
            if ram:
                out.append(f"mem[{a}] = {expr}")
            elif mode == 0:
                out.append(self._io_write(lsw, expr))
            else:
                out.append(f"wr16({a}, {expr})")

        n_insn = len(b.insns)
        branch_pos = None
        for idx, (pc, ir) in enumerate(b.insns):
            # MAR loads become visible three instructions later
            for p in list(mar_pending):
                p[0] -= 1
                if p[0] <= 0:
                    out.append(f"mar{p[1]} = {p[2]}")
                    mar_pending.remove(p)
            op = ir[0]
            if op == "stop":
                flags_hist.append(cur)
                continue
            if op in ("br", "call", "ret"):
                branch_pos = idx
                # the decision uses the flags after the instruction four back
                back = idx - 4
                if back >= 0:
                    fl = flags_hist[back]
                else:
                    fl = tuple(f"H[{4 + back}][{i}]" for i in range(5))
                    self.need_hist = True
                out.append(f"take = {cond_expr(ir[1], fl)}")
                if op == "ret":
                    out.append("ret_to = PROG_BASE + mrpc")
                if op == "call":
                    out.append(f"if take: mrpc = {(pc + 8 - PROG_BASE) & 0xFFF}")
                flags_hist.append(cur)
                continue
            self._want = lambda r, i=idx: wanted_kind(b.insns, i + 1, r)
            self._emit(op, ir, out, F, B, setf, seti, newflags, fl_zn_f, fl_zn_i, cond_expr, ld32f, ld32i, st32,
                       ld16, st16, kind, lambda: cur, mar_pending)
            flags_hist.append(cur)
        for p in mar_pending:
            out.append(f"mar{p[1]} = {p[2]}")
        # history for the next block: flags after its last four instructions
        hist = []
        for k in range(4, 0, -1):
            i = n_insn - k
            hist.append(flags_hist[i] if i >= 0 else tuple(f"H[{4 + i}][{j}]" for j in range(5)))
        out.append("H = (" + ", ".join("(" + ", ".join(h) + ")" for h in hist) + ",)")
        out.append(f"n, z, t, lu, lv = {', '.join(cur)}")
        out.append(f"cyc += {n_insn}")
        ex = b.exit
        if ex[0] == "fall":
            out.append(f"pc = {ex[1]}")
        elif ex[0] == "stop":
            out.append("break")
        elif ex[0] in ("br", "call"):
            out.append(f"pc = {ex[2]} if take else {ex[3]}")
        else:
            out.append(f"pc = ret_to if take else {ex[2]}")
        return out

    def _emit(self, op, ir, out, F, B, setf, seti, newflags, fl_zn_f, fl_zn_i, cond_expr, ld32f, ld32i, st32,  # noqa: C901
              ld16, st16, kind, cur, mar_pending):
        if op == "nop" or op in ("eallow", "edis"):
            return
        if op == "par":
            _, kop, store, dd, ee, ff, aa, mode, lsw = ir
            e, f = F(ee), F(ff)
            if kop == "mac":
                out.append(f"_acc = {F(3)} + {F(2)}; _p = {e} * {f}; _v = {ld32f(mode, lsw)}")
                out.append("r3 = _acc")
                kind[3] = "f"
                setf(dd, "_p")
                setf(aa, "_v")
                fl_zn_f("_v")
                return
            sym = {"mpy": "*", "add": "+", "sub": "-"}[kop]
            out.append(f"_p = {e} {sym} {f}")
            if store:
                st32(mode, lsw, aa)
                setf(dd, "_p")
            else:
                out.append(f"_v = {ld32f(mode, lsw)}")
                setf(dd, "_p")
                setf(aa, "_v")
                fl_zn_f("_v")
            return
        if op == "ld32":
            _, a, cnd, mode, lsw = ir
            if cnd in (14, 15):
                if self._want(a) == "i":
                    out.append(f"_v = {ld32i(mode, lsw)}")
                    seti(a, "_v")
                    if cnd == 15:
                        newflags(f"1 if _v & 0x80000000 and _v & 0x7FFFFFFF else 0", "0 if _v & 0x7FFFFFFF else 1")
                    return
                out.append(f"_v = {ld32f(mode, lsw)}")
                setf(a, "_v")
                if cnd == 15:
                    fl_zn_f("_v")
                return
            # conditional: register keeps its value (and kind) when false
            c_ = cond_expr(cnd, cur())
            ex = ld32f(mode, lsw)
            out.append(f"if {c_}: r{a} = {ex}")
            if kind[a] != "f":
                kind[a] = None
            return
        if op == "movd32":
            _, a, mode, lsw = ir
            if mode != 0:
                raise Unsupported("MMOVD32 indirect")
            if self._ram(lsw):
                out.append(f"_v = fv[{lsw >> 1}]; fv[{(lsw >> 1) + 1}] = _v")
            else:
                out.append(f"_v = c._ldf({lsw}); c._st32({lsw + 2}, tobits(_v))")
            setf(a, "_v")
            fl_zn_f("_v")
            return
        if op == "i32tof_m":
            _, a, mode, lsw = ir
            out.append(f"_v = {ld32i(mode, lsw)}")
            setf(a, "float(_v - 4294967296 if _v >> 31 else _v)")
            return
        if op == "ui32tof_m":
            _, a, mode, lsw = ir
            setf(a, f"float({ld32i(mode, lsw)})")
            return
        if op == "st32":
            _, a, mode, lsw = ir
            st32(mode, lsw, a)
            return
        if op == "i16tof_m":
            _, a, mode, lsw = ir
            out.append(f"_v = {ld16(mode, lsw)}")
            setf(a, "float(_v - 65536 if _v & 0x8000 else _v)")
            return
        if op == "ui16tof_m":
            _, a, mode, lsw = ir
            setf(a, f"float({ld16(mode, lsw)})")
            return
        if op == "movz16":
            _, a, mode, lsw = ir
            seti(a, ld16(mode, lsw))
            fl_zn_i(f"r{a}")
            return
        if op == "st16":
            _, a, mode, lsw = ir
            st16(mode, lsw, f"{B(a)} & 0xFFFF")
            return
        if op == "ld_mar":
            _, r, mode, lsw = ir
            tmp = f"m{len(out)}"
            out.append(f"{tmp} = {ld16(mode, lsw)}")
            mar_pending.append([3, r, tmp])
            return
        if op == "st_mar":
            _, r, mode, lsw = ir
            st16(mode, lsw, f"mar{r}")
            return
        if op in ("mar_imm", "mar_reg"):
            if op == "mar_imm":
                _, r, v = ir
                expr = str(v)
            else:
                _, r, a, v = ir
                tmp = f"m{len(out)}"
                out.append(f"{tmp} = ({B(a)} + {v}) & 0xFFFF")
                expr = tmp
            mar_pending.append([3, r, expr])
            return
        if op in ("ld_mstf", "st_mstf"):
            raise Unsupported("MSTF moves")
        if op == "fimm":
            _, sym, a, b, imm = ir
            setf(a, f"{imm!r} {sym} {F(b)}")
            return
        if op == "moviz":
            _, a, lsw = ir
            seti(a, str(lsw << 16))
            return
        if op == "movxi":
            _, a, lsw = ir
            seti(a, f"({B(a)} & 0xFFFF0000) | {lsw}")
            return
        if op in ("cmpfi", "maxfi", "minfi"):
            _, a, imm, bits = ir
            out.append(f"_x = {F(a)}")
            newflags(f"1 if _x < {imm!r} else 0", f"1 if _x == {imm!r} else 0")
            if op == "maxfi":
                out.append(f"if _x < {imm!r}: _x = {imm!r}")
                setf(a, "_x")
            elif op == "minfi":
                out.append(f"if _x > {imm!r}: _x = {imm!r}")
                setf(a, "_x")
            return
        if op == "setflg":
            _, fl, va = ir
            nn = str(va >> 2 & 1) if fl >> 2 & 1 else None
            zz = str(va >> 3 & 1) if fl >> 3 & 1 else None
            tt = str(va >> 6 & 1) if fl >> 6 & 1 else None
            if nn or zz or tt:
                newflags(nn, zz, tt)
            if fl & 3 or fl & 0x80:
                pass   # LVF/LUF/RNDF32: not tracked by the compiled code
            return
        if op == "mpy_addsub":
            _, sym, la, lb, lc, ld, le, lf = ir
            out.append(f"_p = {F(lb)} * {F(lc)}; _s = {F(le)} {sym} {F(lf)}")
            setf(la, "_p")
            setf(ld, "_s")
            return
        if op in ("negf", "movr", "swap"):
            _, la, lb, cnd = ir
            always = cnd in (14, 15)
            c_ = "True" if always else cond_expr(cnd, cur())
            if op == "negf":
                out.append(f"_v = -{F(lb)} if {c_} else {F(lb)}")
                setf(la, "_v")
                if cnd == 15:
                    fl_zn_f("_v")
            elif op == "movr":
                if always:
                    out.append(f"r{la} = r{lb}")
                    kind[la] = kind[lb]
                    if cnd == 15:
                        out.append(f"_v = {F(la)}")
                        fl_zn_f("_v")
                else:
                    out.append(f"if {c_}: r{la} = r{lb}")
                    if kind[la] != kind[lb]:
                        kind[la] = None
            else:
                out.append(f"if {c_}: r{la}, r{lb} = r{lb}, r{la}")
                if kind[la] != kind[lb]:
                    kind[la] = kind[lb] = None
            return
        if op in ("asr", "lsr", "lsl"):
            _, a, sh = ir
            v = B(a)
            if op == "asr":
                seti(a, f"((({v}) - 4294967296 if ({v}) >> 31 else ({v})) >> {sh}) & 0xFFFFFFFF")
            elif op == "lsr":
                seti(a, f"({v}) >> {sh}")
            else:
                seti(a, f"(({v}) << {sh}) & 0xFFFFFFFF")
            fl_zn_i(f"r{a}")
            return
        if op in ("fmul", "fadd", "fsub"):
            _, la, lb, lc = ir
            sym = {"fmul": "*", "fadd": "+", "fsub": "-"}[op]
            setf(la, f"{F(lb)} {sym} {F(lc)}")
            return
        if op in ("and", "or", "xor", "iadd", "isub"):
            _, la, lb, lc = ir
            x, y = B(lb), B(lc)
            expr = {"and": f"{x} & {y}", "or": f"{x} | {y}", "xor": f"{x} ^ {y}",
                    "iadd": f"({x} + {y}) & 0xFFFFFFFF", "isub": f"({x} - {y}) & 0xFFFFFFFF"}[op]
            seti(la, expr)
            fl_zn_i(f"r{la}")
            return
        if op in ("cmpf", "maxf", "minf"):
            _, la, lb = ir
            out.append(f"_x = {F(la)}; _y = {F(lb)}")
            newflags("1 if _x < _y else 0", "1 if _x == _y else 0")
            if op == "maxf":
                out.append(f"if _x < _y: r{la} = r{lb}")
            elif op == "minf":
                out.append(f"if _x > _y: r{la} = r{lb}")
            if op != "cmpf" and kind[la] != kind[lb]:
                kind[la] = None
            return
        if op == "cmp32":
            _, la, lb = ir
            out.append(f"_x = {B(la)}; _y = {B(lb)}")
            out.append("_x = _x - 4294967296 if _x >> 31 else _x; _y = _y - 4294967296 if _y >> 31 else _y")
            newflags("1 if _x < _y else 0", "1 if _x == _y else 0")
            return
        conv = {"f2i32": "sat_int({x}, -2147483648, 2147483647, False) & 0xFFFFFFFF",
                "f2ui32": "sat_int({x}, 0, 4294967295, False)",
                "f2i16": "sat_int({x}, -32768, 32767, False) & 0xFFFFFFFF",
                "f2i16r": "sat_int({x}, -32768, 32767, True) & 0xFFFFFFFF",
                "f2ui16": "sat_int({x}, 0, 65535, False)",
                "f2ui16r": "sat_int({x}, 0, 65535, True)"}
        if op in conv:
            _, la, lb = ir
            seti(la, conv[op].format(x=F(lb)))
            return
        if op in ("i32tof", "ui32tof", "i16tof", "ui16tof"):
            _, la, lb = ir
            x = B(lb)
            expr = {"i32tof": f"float(({x}) - 4294967296 if ({x}) >> 31 else ({x}))",
                    "ui32tof": f"float({x})",
                    "i16tof": f"float((({x}) & 0xFFFF) - 65536 if ({x}) & 0x8000 else ({x}) & 0xFFFF)",
                    "ui16tof": f"float(({x}) & 0xFFFF)"}[op]
            setf(la, expr)
            return
        if op in ("frac", "absf", "eisqrt", "einv"):
            _, la, lb = ir
            x = F(lb)
            if op == "absf":
                out.append(f"_v = abs({x})")
                setf(la, "_v")
                fl_zn_f("_v")
            else:
                setf(la, {"frac": f"frac({x})", "eisqrt": f"eisqrt({x})", "einv": f"einv({x})"}[op])
            return
        if op == "testtf":
            _, cnd = ir
            newflags(None, None, f"1 if {cond_expr(cnd, cur())} else 0")
            return
        raise Unsupported(op)
