"""TMS320C28x instruction set: decoder, semantics and disassembler.

The C28x (TI C2000 MCUs such as the F2803x) is a 32-bit fixed-point DSP with
a *word*-addressed memory (every address holds 16 bits), 16- and 32-bit
instructions, and one shared operand field, ``loc16``/``loc32``, that covers
direct, stack, indirect and register addressing (SPRU430F, table 5-1).

Each instruction is decoded once into an :class:`Insn` whose ``fn(cpu)``
executes it.  The run loop sets ``cpu.pc`` to the next instruction before
calling ``fn``; branches overwrite it.  Encodings and semantics follow TI's
"TMS320C28x CPU and Instruction Set Reference Guide" (SPRU430F); only the C28x
object mode (OBJMODE = 1, AMODE = 0) that the TI compiler uses is supported.
Instructions the compiler never emits (the C2xLP ARP forms, IN/OUT ports) are
decoded for the disassembly but raise :class:`Unimplemented` when executed.
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

M16, M32, M22 = 0xFFFF, 0xFFFFFFFF, 0x3FFFFF


class Unimplemented(Exception):
    pass


def s16(v: int) -> int:
    return v - 0x10000 if v & 0x8000 else v


def s32(v: int) -> int:
    return v - 0x100000000 if v & 0x80000000 else v


class Insn:
    __slots__ = ("addr", "size", "fn", "text", "flow", "repeatable", "target", "cycles", "key")

    def __init__(self, addr, size, fn, text, flow=None, repeatable=False, target=None, cycles=1, key=""):
        self.addr = addr
        self.size = size
        self.fn = fn
        self.text = text
        #: None (falls through), "jump", "call", "ret", "cond", "rpt", "idle", "trap"
        self.flow = flow
        self.repeatable = repeatable
        self.target = target
        self.cycles = cycles
        self.key = key

    def __repr__(self):
        return f"<{self.addr:06x}: {self.text}>"


# ---------------------------------------------------------------- conditions
COND_NAMES = ("NEQ", "EQ", "GT", "GEQ", "LT", "LEQ", "HI", "HIS", "LO", "LOS", "NOV", "OV", "NTC", "TC",
              "NBIO", "UNC")


def cond_fn(k: int) -> Callable:
    """Condition test; NOV/OV clear V after testing it, as the hardware does."""
    if k == 0:
        return lambda c: not c.z
    if k == 1:
        return lambda c: c.z
    if k == 2:
        return lambda c: not c.z and not c.n
    if k == 3:
        return lambda c: not c.n
    if k == 4:
        return lambda c: c.n
    if k == 5:
        return lambda c: c.z or c.n
    if k == 6:
        return lambda c: c.c and not c.z
    if k == 7:
        return lambda c: c.c
    if k == 8:
        return lambda c: not c.c
    if k == 9:
        return lambda c: not c.c or c.z
    if k == 10:
        def nov(c):
            r = not c.v
            c.v = 0
            return r
        return nov
    if k == 11:
        def ov(c):
            r = c.v
            c.v = 0
            return r
        return ov
    if k == 12:
        return lambda c: not c.tc
    if k == 13:
        return lambda c: c.tc
    if k == 14:
        return lambda c: c.bio   # BIO pin low
    return lambda c: True


# ---------------------------------------------------------------- arithmetic helpers
def nz16(c, v):
    c.n = v >> 15
    c.z = 0 if v else 1


def nz32(c, v):
    c.n = v >> 31
    c.z = 0 if v else 1


def ovc_count(c, delta):
    o = c.ovc + delta
    if o > 31:
        o -= 64
    elif o < -32:
        o += 64
    c.ovc = o


def acc_add(c, b, cin=0, cmode=0, ovc=True):
    """ACC = ACC + b (+ carry in) with C, V, N, Z, OVC and OVM saturation.
    ``cmode`` 1: C can be set but not cleared (ADD ACC,loc16<<#16)."""
    a = c.acc
    r = a + b + cin
    carry = r >> 32
    r &= M32
    if cmode == 1:
        if carry:
            c.c = 1
    else:
        c.c = carry
    if (~(a ^ b) & (a ^ r)) & 0x80000000:
        c.v = 1
        if c.ovm:
            r = 0x80000000 if a & 0x80000000 else 0x7FFFFFFF
        elif ovc:
            ovc_count(c, -1 if a & 0x80000000 else 1)
    c.acc = r
    c.n = r >> 31
    c.z = 0 if r else 1
    return r


def acc_sub(c, b, bin_=0, cmode=0, ovc=True):
    """ACC = ACC - b (- borrow in); C = no borrow.  ``cmode`` 2: C can be
    cleared but not set (SUB ACC,loc16<<#16)."""
    a = c.acc
    r = a - b - bin_
    nob = 0 if r < 0 else 1
    r &= M32
    if cmode == 2:
        if not nob:
            c.c = 0
    else:
        c.c = nob
    if ((a ^ b) & (a ^ r)) & 0x80000000:
        c.v = 1
        if c.ovm:
            r = 0x80000000 if a & 0x80000000 else 0x7FFFFFFF
        elif ovc:
            ovc_count(c, -1 if a & 0x80000000 else 1)
    c.acc = r
    c.n = r >> 31
    c.z = 0 if r else 1
    return r


def add16(c, a, b):
    r = a + b
    c.c = r >> 16
    r &= M16
    if (~(a ^ b) & (a ^ r)) & 0x8000:
        c.v = 1
    c.n = r >> 15
    c.z = 0 if r else 1
    return r


def sub16(c, a, b):
    r = a - b
    c.c = 0 if r < 0 else 1
    r &= M16
    if ((a ^ b) & (a ^ r)) & 0x8000:
        c.v = 1
    c.n = r >> 15
    c.z = 0 if r else 1
    return r


def pshift(c, p=None):
    """P shifted by the product shift mode (PM)."""
    if p is None:
        p = c.p
    pm = c.pm
    if pm == 1:
        return p
    if pm == 0:
        return (p << 1) & M32
    if pm == 5 and c.amode:
        return (p << 4) & M32
    return (s32(p) >> (pm - 1)) & M32


def ext16(c, v):
    """16-bit value extended to 32 bits according to SXM."""
    if c.sxm and v & 0x8000:
        return v | 0xFFFF0000
    return v


# ---------------------------------------------------------------- loc16 / loc32
REG16 = {0xA8: "AH", 0xA9: "AL", 0xAA: "PH", 0xAB: "PL", 0xAC: "TH", 0xAD: "SP"}
REG32 = {0xA9: "ACC", 0xAB: "P", 0xAC: "XT"}


def loc_text(L: int, size: int) -> str:
    mode = L >> 6
    if mode == 0:
        return f"@0x{L & 63:x}"
    if mode == 1:
        return f"*-SP[{L & 63}]"
    if mode == 3:
        return f"*+XAR{L & 7}[{(L >> 3) & 7}]"
    sub, n = (L >> 3) & 7, L & 7
    if sub == 0:
        return f"*XAR{n}++"
    if sub == 1:
        return f"*--XAR{n}"
    if sub == 2:
        return f"*+XAR{n}[AR0]"
    if sub == 3:
        return f"*+XAR{n}[AR1]"
    if sub == 4:
        return f"@XAR{n}" if size == 32 else f"@AR{n}"
    if sub == 5:
        if size == 32 and L in REG32:
            return "@" + REG32[L]
        if L in REG16:
            return "@" + REG16[L]
        return "*BR0++" if L == 0xAE else "*BR0--"
    if sub == 6:
        return f"*,ARP{n}"
    return ("*", "*++", "*--", "*0++", "*0--", "*SP++", "*--SP", "*AR6%++")[n]


def make_ea(L: int, size: int) -> Optional[Callable]:
    """Effective-address function (with the mode's side effects), or None
    for the register-direct modes.  Addresses are 22-bit word addresses;
    32-bit accesses ignore bit 0 (done by the memory accessors)."""
    inc = 2 if size == 32 else 1
    mode = L >> 6
    if mode == 0:
        off = L & 63

        def ea(c):
            return (c.dp << 6) | off
        return ea
    if mode == 1:
        off = L & 63

        def ea(c):
            return (c.sp - off) & M16
        return ea
    n = L & 7
    if mode == 3:
        k = (L >> 3) & 7

        def ea(c):
            return (c.xar[n] + k) & M22
        return ea
    sub = (L >> 3) & 7
    if sub == 0:
        def ea(c):
            x = c.xar
            a = x[n]
            x[n] = (a + inc) & M32
            return a & M22
        return ea
    if sub == 1:
        def ea(c):
            x = c.xar
            a = (x[n] - inc) & M32
            x[n] = a
            return a & M22
        return ea
    if sub == 2:
        def ea(c):
            x = c.xar
            return (x[n] + (x[0] & M16)) & M22
        return ea
    if sub == 3:
        def ea(c):
            x = c.xar
            return (x[n] + (x[1] & M16)) & M22
        return ea
    if sub == 4:
        return None
    if sub == 5:
        if L in (0xAE, 0xAF):
            def ea(c):
                raise Unimplemented("bit-reversed addressing *BR0++/--")
            return ea
        return None
    if sub == 6:   # *,ARPn: use the current ARP register, then ARP = n
        def ea(c):
            a = c.xar[c.arp] & M22
            c.arp = n
            return a
        return ea
    # sub == 7
    if n == 5:   # *SP++
        def ea(c):
            a = c.sp
            c.sp = (a + inc) & M16
            return a
        return ea
    if n == 6:   # *--SP
        def ea(c):
            a = (c.sp - inc) & M16
            c.sp = a
            return a
        return ea
    if n == 7:   # *AR6%++
        def ea(c):
            x = c.xar
            a = x[6]
            if (a & 0xFF) == (x[1] & 0xFF):
                x[6] = a & 0xFFFFFF00
            else:
                x[6] = (a & 0xFFFF0000) | ((a + inc) & M16)
            return a & M22
        return ea
    # C2xLP ARP modes: *, *++, *--, *0++, *0--
    delta = (0, inc, -inc, None, None)[n]

    def ea(c):
        x = c.xar
        r = c.arp
        a = x[r]
        if n == 3:
            x[r] = (a + (x[0] & M16)) & M32
        elif n == 4:
            x[r] = (a - (x[0] & M16)) & M32
        elif delta:
            x[r] = (a + delta) & M32
        return a & M22
    return ea


def reg16_get(L):
    if (L >> 3) == 0x14:
        n = L & 7
        return lambda c: c.xar[n] & M16
    return {
        0xA8: lambda c: c.acc >> 16,
        0xA9: lambda c: c.acc & M16,
        0xAA: lambda c: c.p >> 16,
        0xAB: lambda c: c.p & M16,
        0xAC: lambda c: c.xt >> 16,
        0xAD: lambda c: c.sp,
    }[L]


def reg16_set(L):
    if (L >> 3) == 0x14:
        n = L & 7

        def s(c, v):
            x = c.xar
            x[n] = (x[n] & 0xFFFF0000) | v
        return s

    def ah(c, v):
        c.acc = (c.acc & M16) | (v << 16)

    def al(c, v):
        c.acc = (c.acc & 0xFFFF0000) | v

    def ph(c, v):
        c.p = (c.p & M16) | (v << 16)

    def pl(c, v):
        c.p = (c.p & 0xFFFF0000) | v

    def th(c, v):
        c.xt = (c.xt & M16) | (v << 16)

    def sp(c, v):
        c.sp = v
    return {0xA8: ah, 0xA9: al, 0xAA: ph, 0xAB: pl, 0xAC: th, 0xAD: sp}[L]


def reg32_get(L):
    if (L >> 3) == 0x14:
        n = L & 7
        return lambda c: c.xar[n]
    return {0xA9: lambda c: c.acc, 0xAB: lambda c: c.p, 0xAC: lambda c: c.xt}[L]


def reg32_set(L):
    if (L >> 3) == 0x14:
        n = L & 7

        def s(c, v):
            c.xar[n] = v
        return s

    def acc(c, v):
        c.acc = v

    def p(c, v):
        c.p = v

    def xt(c, v):
        c.xt = v
    return {0xA9: acc, 0xAB: p, 0xAC: xt}[L]


class Loc:
    """Accessors for one loc16/loc32 field value."""
    __slots__ = ("L", "size", "text", "ea", "rd", "wr", "rmw", "is_ax", "is_acc", "reg")

    def __init__(self, L: int, size: int):
        self.L = L
        self.size = size
        self.text = loc_text(L, size)
        ea = make_ea(L, size)
        self.ea = ea
        self.is_ax = size == 16 and L in (0xA8, 0xA9)
        self.is_acc = size == 32 and L == 0xA9
        self.reg = ea is None
        if ea is None:
            if size == 16:
                if L not in REG16 and (L >> 3) != 0x14:
                    raise Unimplemented(f"loc16 0x{L:02x}")
                get, put = reg16_get(L), reg16_set(L)
            else:
                if L not in REG32 and (L >> 3) != 0x14:
                    raise Unimplemented(f"loc32 0x{L:02x}")
                get, put = reg32_get(L), reg32_set(L)
            self.rd = get
            self.wr = put

            def rmw(c, f):
                v = f(get(c))
                put(c, v)
                return v
            self.rmw = rmw
            return
        if size == 16:
            def rd(c):
                return c.rd16(ea(c))

            def wr(c, v):
                c.wr16(ea(c), v)

            def rmw(c, f):
                a = ea(c)
                v = f(c.rd16(a))
                c.wr16(a, v)
                return v
        else:
            def rd(c):
                return c.rd32(ea(c))

            def wr(c, v):
                c.wr32(ea(c), v)

            def rmw(c, f):
                a = ea(c)
                v = f(c.rd32(a))
                c.wr32(a, v)
                return v
        self.rd, self.wr, self.rmw = rd, wr, rmw


_LOCS: Dict[Tuple[int, int], Loc] = {}


def loc(L: int, size: int) -> Loc:
    k = (L, size)
    v = _LOCS.get(k)
    if v is None:
        v = _LOCS[k] = Loc(L, size)
    return v


# ---------------------------------------------------------------- encoding table
# (key, word 1 pattern, word 2 pattern or None, syntax).  Pattern letters:
# L loc field, A AH/AL select, C immediate (both words, msb first),
# S shift, n/m register numbers, B bit number, K condition.
TABLE = [
    ("ABORTI", "0000 0000 0000 0001", None, "ABORTI"),
    ("ABS", "1111 1111 0101 0110", None, "ABS ACC"),
    ("ABSTC", "0101 0110 0101 1111", None, "ABSTC ACC"),
    ("ADD_ACC_IMM", "1111 1111 0001 SSSS", "CCCC CCCC CCCC CCCC", "ADD ACC,#{C}<<#{S}"),
    ("ADD_ACC_LOC_T", "0101 0110 0010 0011", "0000 0000 LLLL LLLL", "ADD ACC,{L16}<<T"),
    ("ADD_ACC_LOC", "1000 0001 LLLL LLLL", None, "ADD ACC,{L16}"),
    ("ADD_ACC_LOC_S", "0101 0110 0000 0100", "0000 SSSS LLLL LLLL", "ADD ACC,{L16}<<#{S}"),
    ("ADD_ACC_LOC_16", "0000 0101 LLLL LLLL", None, "ADD ACC,{L16}<<#16"),
    ("ADD_AX_LOC", "1001 010A LLLL LLLL", None, "ADD {A},{L16}"),
    ("ADD_LOC_AX", "0111 001A LLLL LLLL", None, "ADD {L16},{A}"),
    ("ADD_LOC_IMM", "0000 1000 LLLL LLLL", "CCCC CCCC CCCC CCCC", "ADD {L16},#{Cs}"),
    ("ADDB_ACC", "0000 1001 CCCC CCCC", None, "ADDB ACC,#{C}"),
    ("ADDB_AX", "1001 110A CCCC CCCC", None, "ADDB {A},#{Cs8}"),
    ("ADDB_SP", "1111 1110 0CCC CCCC", None, "ADDB SP,#{C}"),
    ("ADDB_XAR", "1101 1nnn 0CCC CCCC", None, "ADDB XAR{n},#{C}"),
    ("ADDCL", "0101 0110 0100 0000", "xxxx xxxx LLLL LLLL", "ADDCL ACC,{L32}"),
    ("ADDCU", "0000 1100 LLLL LLLL", None, "ADDCU ACC,{L16}"),
    ("ADDL_ACC_LOC", "0000 0111 LLLL LLLL", None, "ADDL ACC,{L32}"),
    ("ADDL_ACC_P", "0001 0000 1010 1100", None, "ADDL ACC,P<<PM"),
    ("ADDL_LOC_ACC", "0101 0110 0000 0001", "0000 0000 LLLL LLLL", "ADDL {L32},ACC"),
    ("ADDU", "0000 1101 LLLL LLLL", None, "ADDU ACC,{L16}"),
    ("ADDUL_P", "0101 0110 0101 0111", "0000 0000 LLLL LLLL", "ADDUL P,{L32}"),
    ("ADDUL_ACC", "0101 0110 0101 0011", "xxxx xxxx LLLL LLLL", "ADDUL ACC,{L32}"),
    ("ADRK", "1111 1100 CCCC CCCC", None, "ADRK #{C}"),
    ("AND_ACC_IMM", "0011 1110 0000 SSSS", "CCCC CCCC CCCC CCCC", "AND ACC,#{C}<<#{S}"),
    ("AND_ACC_IMM16", "0101 0110 0000 1000", "CCCC CCCC CCCC CCCC", "AND ACC,#{C}<<#16"),
    ("AND_ACC_LOC", "1000 1001 LLLL LLLL", None, "AND ACC,{L16}"),
    ("AND_AX_LOC_IMM", "1100 110A LLLL LLLL", "CCCC CCCC CCCC CCCC", "AND {A},{L16},#{C}"),
    ("AND_IER", "0111 0110 0010 0110", "CCCC CCCC CCCC CCCC", "AND IER,#{C}"),
    ("AND_IFR", "0111 0110 0010 1111", "CCCC CCCC CCCC CCCC", "AND IFR,#{C}"),
    ("AND_LOC_AX", "1100 000A LLLL LLLL", None, "AND {L16},{A}"),
    ("AND_AX_LOC", "1100 111A LLLL LLLL", None, "AND {A},{L16}"),
    ("AND_LOC_IMM", "0001 1000 LLLL LLLL", "CCCC CCCC CCCC CCCC", "AND {L16},#{C}"),
    ("ANDB", "1001 000A CCCC CCCC", None, "ANDB {A},#{C}"),
    ("ASP", "0111 0110 0001 1011", None, "ASP"),
    ("ASR_AX_IMM", "1111 1111 101A SSSS", None, "ASR {A},#{S1}"),
    ("ASR_AX_T", "1111 1111 0110 010A", None, "ASR {A},T"),
    ("ASR64_IMM", "0101 0110 1000 SSSS", None, "ASR64 ACC:P,#{S1}"),
    ("ASR64_T", "0101 0110 0010 1100", None, "ASR64 ACC:P,T"),
    ("ASRL", "0101 0110 0001 0000", None, "ASRL ACC,T"),
    ("B", "1111 1111 1110 KKKK", "CCCC CCCC CCCC CCCC", "B {T16},{K}"),
    ("BANZ", "0000 0000 0000 1nnn", "CCCC CCCC CCCC CCCC", "BANZ {T16},AR{n}--"),
    ("BAR", "1000 1111 1Bnn nmmm", "CCCC CCCC CCCC CCCC", "BAR {T16},AR{n},AR{m},{BAR}"),
    ("BF", "0101 0110 1100 KKKK", "CCCC CCCC CCCC CCCC", "BF {T16},{K}"),
    ("CLRC_AMODE", "0101 0110 0001 0110", None, "CLRC AMODE"),
    ("CLRC_M0M1MAP", "0101 0110 0011 1111", None, "CLRC M0M1MAP"),
    ("CLRC_OBJMODE", "0101 0110 0011 0110", None, "CLRC OBJMODE"),
    ("CLRC_OVC", "0101 0110 0101 1100", None, "CLRC OVC"),
    ("CLRC_XF", "0101 0110 0001 1011", None, "CLRC XF"),
    ("CLRC", "0010 1001 CCCC CCCC", None, "CLRC {MODE}"),
    ("CMP_AX_LOC", "0101 010A LLLL LLLL", None, "CMP {A},{L16}"),
    ("CMP_LOC_IMM", "0001 1011 LLLL LLLL", "CCCC CCCC CCCC CCCC", "CMP {L16},#{Cs}"),
    ("CMP64", "0101 0110 0101 1110", None, "CMP64 ACC:P"),
    ("CMPB", "0101 001A CCCC CCCC", None, "CMPB {A},#{C}"),
    ("CMPL_ACC_LOC", "0000 1111 LLLL LLLL", None, "CMPL ACC,{L32}"),
    ("CMPL_ACC_P", "1111 1111 0101 1001", None, "CMPL ACC,P<<PM"),
    ("CMPR", "0101 0110 0001 1KKK", None, "CMPR {CMPR}"),
    ("CSB", "0101 0110 0011 0101", None, "CSB ACC"),
    ("DEC", "0000 1011 LLLL LLLL", None, "DEC {L16}"),
    ("DMAC", "0101 0110 0100 1011", "1n00 0111 LLLL LLLL", "DMAC ACC:P,{L32},{X7}"),
    ("DMOV", "1010 0101 LLLL LLLL", None, "DMOV {L16}"),
    ("EALLOW", "0111 0110 0010 0010", None, "EALLOW"),
    ("EDIS", "0111 0110 0001 1010", None, "EDIS"),
    ("ESTOP0", "0111 0110 0010 0101", None, "ESTOP0"),
    ("ESTOP1", "0111 0110 0010 0100", None, "ESTOP1"),
    ("FFC", "0000 0000 11CC CCCC", "CCCC CCCC CCCC CCCC", "FFC XAR7,{T22}"),
    ("FLIP", "0101 0110 0111 000A", None, "FLIP {A}"),
    ("IACK", "0111 0110 0011 1111", "CCCC CCCC CCCC CCCC", "IACK #{C}"),
    ("IDLE", "0111 0110 0010 0001", None, "IDLE"),
    ("IMACL", "0101 0110 0100 1101", "1n00 0111 LLLL LLLL", "IMACL P,{L32},{X7}"),
    ("IMPYAL", "0101 0110 0100 1100", "0000 0000 LLLL LLLL", "IMPYAL P,XT,{L32}"),
    ("IMPYL_ACC", "0101 0110 0100 0100", "0000 0000 LLLL LLLL", "IMPYL ACC,XT,{L32}"),
    ("IMPYL_P", "0101 0110 0000 0101", "0000 0000 LLLL LLLL", "IMPYL P,XT,{L32}"),
    ("IMPYSL", "0101 0110 0100 0011", "0000 0000 LLLL LLLL", "IMPYSL P,XT,{L32}"),
    ("IMPYXUL", "0101 0110 0110 0101", "0000 0000 LLLL LLLL", "IMPYXUL P,XT,{L32}"),
    ("IN", "1011 0100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "IN {L16},*(0x{C:x})"),
    ("INC", "0000 1010 LLLL LLLL", None, "INC {L16}"),
    ("INTR", "0000 0000 0001 CCCC", None, "INTR {INTR}"),
    ("INTR_NMI", "0111 0110 0001 0110", None, "INTR NMI"),
    ("INTR_EMU", "0111 0110 0001 1100", None, "INTR EMUINT"),
    ("IRET", "0111 0110 0000 0010", None, "IRET"),
    ("LB_XAR7", "0111 0110 0010 0000", None, "LB *XAR7"),
    ("LB", "0000 0000 01CC CCCC", "CCCC CCCC CCCC CCCC", "LB {T22}"),
    ("LC_XAR7", "0111 0110 0000 0100", None, "LC *XAR7"),
    ("LC", "0000 0000 10CC CCCC", "CCCC CCCC CCCC CCCC", "LC {T22}"),
    ("LCR", "0111 0110 01CC CCCC", "CCCC CCCC CCCC CCCC", "LCR {T22}"),
    ("LCR_XARN", "0011 1110 0110 0nnn", None, "LCR *XAR{n}"),
    ("LOOPNZ", "0010 1110 LLLL LLLL", "CCCC CCCC CCCC CCCC", "LOOPNZ {L16},#{C}"),
    ("LOOPZ", "0010 1100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "LOOPZ {L16},#{C}"),
    ("LPADDR", "0101 0110 0001 1110", None, "LPADDR"),
    ("LRET", "0111 0110 0001 0100", None, "LRET"),
    ("LRETE", "0111 0110 0001 0000", None, "LRETE"),
    ("LRETR", "0000 0000 0000 0110", None, "LRETR"),
    ("LSL_ACC_IMM", "1111 1111 0011 SSSS", None, "LSL ACC,#{S1}"),
    ("LSL_ACC_T", "1111 1111 0101 0000", None, "LSL ACC,T"),
    ("LSL_AX_IMM", "1111 1111 100A SSSS", None, "LSL {A},#{S1}"),
    ("LSL_AX_T", "1111 1111 0110 011A", None, "LSL {A},T"),
    ("LSL64_IMM", "0101 0110 1010 SSSS", None, "LSL64 ACC:P,#{S1}"),
    ("LSL64_T", "0101 0110 0101 0010", None, "LSL64 ACC:P,T"),
    ("LSLL", "0101 0110 0011 1011", None, "LSLL ACC,T"),
    ("LSR_AX_IMM", "1111 1111 110A SSSS", None, "LSR {A},#{S1}"),
    ("LSR_AX_T", "1111 1111 0110 001A", None, "LSR {A},T"),
    ("LSR64_IMM", "0101 0110 1001 SSSS", None, "LSR64 ACC:P,#{S1}"),
    ("LSR64_T", "0101 0110 0101 1011", None, "LSR64 ACC:P,T"),
    ("LSRL", "0101 0110 0010 0010", None, "LSRL ACC,T"),
    ("MAC_PMA", "0001 0100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MAC P,{L16},0:0x{C:x}"),
    ("MAC_XAR7", "0101 0110 0000 0111", "1n00 0111 LLLL LLLL", "MAC P,{L16},{X7}"),
    ("MAX", "0101 0110 0111 001A", "0000 0000 LLLL LLLL", "MAX {A},{L16}"),
    ("MAXCUL", "0101 0110 0101 0001", "0000 0000 LLLL LLLL", "MAXCUL P,{L32}"),
    ("MAXL", "0101 0110 0110 0001", "0000 0000 LLLL LLLL", "MAXL ACC,{L32}"),
    ("MIN", "0101 0110 0111 010A", "0000 0000 LLLL LLLL", "MIN {A},{L16}"),
    ("MINCUL", "0101 0110 0101 1001", "xxxx xxxx LLLL LLLL", "MINCUL P,{L32}"),
    ("MINL", "0101 0110 0101 0000", "0000 0000 LLLL LLLL", "MINL ACC,{L32}"),
    ("MOV_ABS_LOC", "1111 0100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MOV *(0:0x{C:04x}),{L16}"),
    ("MOV_ACC_IMM", "1111 1111 0010 SSSS", "CCCC CCCC CCCC CCCC", "MOV ACC,#{C}<<#{S}"),
    ("MOV_ACC_LOC_T", "0101 0110 0000 0110", "0000 0000 LLLL LLLL", "MOV ACC,{L16}<<T"),
    ("MOV_ACC_LOC", "1000 0101 LLLL LLLL", None, "MOV ACC,{L16}"),
    ("MOV_ACC_LOC_S", "0101 0110 0000 0011", "0000 SSSS LLLL LLLL", "MOV ACC,{L16}<<#{S}"),
    ("MOV_ACC_LOC_16", "0010 0101 LLLL LLLL", None, "MOV ACC,{L16}<<#16"),
    ("MOV_AR6_LOC", "0101 1110 LLLL LLLL", None, "MOV AR6,{L16}"),
    ("MOV_AR7_LOC", "0101 1111 LLLL LLLL", None, "MOV AR7,{L16}"),
    ("MOV_AX_LOC", "1001 001A LLLL LLLL", None, "MOV {A},{L16}"),
    ("MOV_DP_IMM", "1111 10CC CCCC CCCC", None, "MOV DP,#0x{C:x}"),
    ("MOV_IER_LOC", "0010 0011 LLLL LLLL", None, "MOV IER,{L16}"),
    ("MOV_LOC_IMM", "0010 1000 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MOV {L16},#0x{C:x}"),
    ("MOV_LOC_ABS", "1111 0101 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MOV {L16},*(0:0x{C:04x})"),
    ("MOV_LOC_0", "0010 1011 LLLL LLLL", None, "MOV {L16},#0"),
    ("MOV_LOC_ACC_1", "1011 0001 LLLL LLLL", None, "MOV {L16},ACC<<1"),
    ("MOV_LOC_ACC_S", "0101 0110 0010 1101", "0000 0SSS LLLL LLLL", "MOV {L16},ACC<<{S}"),
    ("MOV_LOC_ARN", "0111 1nnn LLLL LLLL", None, "MOV {L16},AR{n}"),
    ("MOV_LOC_AX", "1001 011A LLLL LLLL", None, "MOV {L16},{A}"),
    ("MOV_LOC_AX_COND", "0101 0110 0010 101A", "0000 KKKK LLLL LLLL", "MOV {L16},{A},{K}"),
    ("MOV_LOC_IER", "0010 0000 LLLL LLLL", None, "MOV {L16},IER"),
    ("MOV_LOC_OVC", "0101 0110 0010 1001", "0000 0000 LLLL LLLL", "MOV {L16},OVC"),
    ("MOV_LOC_P", "0011 1111 LLLL LLLL", None, "MOV {L16},P"),
    ("MOV_LOC_T", "0010 0001 LLLL LLLL", None, "MOV {L16},T"),
    ("MOV_OVC_LOC", "0101 0110 0000 0010", "0000 0000 LLLL LLLL", "MOV OVC,{L16}"),
    ("MOV_PH_LOC", "0010 1111 LLLL LLLL", None, "MOV PH,{L16}"),
    ("MOV_PL_LOC", "0010 0111 LLLL LLLL", None, "MOV PL,{L16}"),
    ("MOV_PM_AX", "0101 0110 0011 100A", None, "MOV PM,{A}"),
    ("MOV_T_LOC", "0010 1101 LLLL LLLL", None, "MOV T,{L16}"),
    ("MOV_TL_0", "0101 0110 0101 0110", None, "MOV TL,#0"),
    ("MOV_XARN_PC", "0011 1110 0101 1nnn", None, "MOV XAR{n},PC"),
    ("MOVA", "0001 0000 LLLL LLLL", None, "MOVA T,{L16}"),
    ("MOVAD", "1010 0111 LLLL LLLL", None, "MOVAD T,{L16}"),
    ("MOVB_ACC", "0000 0010 CCCC CCCC", None, "MOVB ACC,#{C}"),
    ("MOVB_AR6", "1101 0110 CCCC CCCC", None, "MOVB AR6,#{C}"),
    ("MOVB_AR7", "1101 0111 CCCC CCCC", None, "MOVB AR7,#{C}"),
    ("MOVB_AX", "1001 101A CCCC CCCC", None, "MOVB {A},#{C}"),
    ("MOVB_AX_LSB", "1100 011A LLLL LLLL", None, "MOVB {A}.LSB,{L16}"),
    ("MOVB_AX_MSB", "0011 100A LLLL LLLL", None, "MOVB {A}.MSB,{L16}"),
    ("MOVB_LOC_IMM_COND", "0101 0110 1011 KKKK", "CCCC CCCC LLLL LLLL", "MOVB {L16},#{C},{K}"),
    ("MOVB_LOC_AX_LSB", "0011 110A LLLL LLLL", None, "MOVB {L16},{A}.LSB"),
    ("MOVB_LOC_AX_MSB", "1100 100A LLLL LLLL", None, "MOVB {L16},{A}.MSB"),
    ("MOVB_XARN", "1101 0nnn CCCC CCCC", None, "MOVB XAR{n},#{C}"),
    ("MOVB_XAR6", "1011 1110 CCCC CCCC", None, "MOVB XAR6,#{C}"),
    ("MOVB_XAR7", "1011 0110 CCCC CCCC", None, "MOVB XAR7,#{C}"),
    ("MOVDL", "1010 0110 LLLL LLLL", None, "MOVDL XT,{L32}"),
    ("MOVH_LOC_ACC_1", "1011 0011 LLLL LLLL", None, "MOVH {L16},ACC<<1"),
    ("MOVH_LOC_ACC_S", "0101 0110 0010 1111", "0000 0SSS LLLL LLLL", "MOVH {L16},ACC<<{S}"),
    ("MOVH_LOC_P", "0101 0111 LLLL LLLL", None, "MOVH {L16},P"),
    ("MOVL_ACC_LOC", "0000 0110 LLLL LLLL", None, "MOVL ACC,{L32}"),
    ("MOVL_ACC_P", "0001 0110 1010 1100", None, "MOVL ACC,P<<PM"),
    ("MOVL_LOC_ACC", "0001 1110 LLLL LLLL", None, "MOVL {L32},ACC"),
    ("MOVL_LOC_ACC_COND", "0101 0110 0100 1000", "0000 KKKK LLLL LLLL", "MOVL {L32},ACC,{K}"),
    ("MOVL_LOC_P", "1010 1001 LLLL LLLL", None, "MOVL {L32},P"),
    ("MOVL_LOC_XAR0", "0011 1010 LLLL LLLL", None, "MOVL {L32},XAR0"),
    ("MOVL_LOC_XAR1", "1011 0010 LLLL LLLL", None, "MOVL {L32},XAR1"),
    ("MOVL_LOC_XAR2", "1010 1010 LLLL LLLL", None, "MOVL {L32},XAR2"),
    ("MOVL_LOC_XAR3", "1010 0010 LLLL LLLL", None, "MOVL {L32},XAR3"),
    ("MOVL_LOC_XAR4", "1010 1000 LLLL LLLL", None, "MOVL {L32},XAR4"),
    ("MOVL_LOC_XAR5", "1010 0000 LLLL LLLL", None, "MOVL {L32},XAR5"),
    ("MOVL_LOC_XAR6", "1100 0010 LLLL LLLL", None, "MOVL {L32},XAR6"),
    ("MOVL_LOC_XAR7", "1100 0011 LLLL LLLL", None, "MOVL {L32},XAR7"),
    ("MOVL_LOC_XT", "1010 1011 LLLL LLLL", None, "MOVL {L32},XT"),
    ("MOVL_P_ACC", "1111 1111 0101 1010", None, "MOVL P,ACC"),
    ("MOVL_P_LOC", "1010 0011 LLLL LLLL", None, "MOVL P,{L32}"),
    ("MOVL_XAR0_LOC", "1000 1110 LLLL LLLL", None, "MOVL XAR0,{L32}"),
    ("MOVL_XAR1_LOC", "1000 1011 LLLL LLLL", None, "MOVL XAR1,{L32}"),
    ("MOVL_XAR2_LOC", "1000 0110 LLLL LLLL", None, "MOVL XAR2,{L32}"),
    ("MOVL_XAR3_LOC", "1000 0010 LLLL LLLL", None, "MOVL XAR3,{L32}"),
    ("MOVL_XAR4_LOC", "1000 1010 LLLL LLLL", None, "MOVL XAR4,{L32}"),
    ("MOVL_XAR5_LOC", "1000 0011 LLLL LLLL", None, "MOVL XAR5,{L32}"),
    ("MOVL_XAR6_LOC", "1100 0100 LLLL LLLL", None, "MOVL XAR6,{L32}"),
    ("MOVL_XAR7_LOC", "1100 0101 LLLL LLLL", None, "MOVL XAR7,{L32}"),
    ("MOVL_XAR0_IMM", "1000 1101 00CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR0,#0x{C:x}"),
    ("MOVL_XAR1_IMM", "1000 1101 01CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR1,#0x{C:x}"),
    ("MOVL_XAR2_IMM", "1000 1101 10CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR2,#0x{C:x}"),
    ("MOVL_XAR3_IMM", "1000 1101 11CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR3,#0x{C:x}"),
    ("MOVL_XAR4_IMM", "1000 1111 00CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR4,#0x{C:x}"),
    ("MOVL_XAR5_IMM", "1000 1111 01CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR5,#0x{C:x}"),
    ("MOVL_XAR6_IMM", "0111 0110 10CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR6,#0x{C:x}"),
    ("MOVL_XAR7_IMM", "0111 0110 11CC CCCC", "CCCC CCCC CCCC CCCC", "MOVL XAR7,#0x{C:x}"),
    ("MOVL_XT_LOC", "1000 0111 LLLL LLLL", None, "MOVL XT,{L32}"),
    ("MOVP", "0001 0110 LLLL LLLL", None, "MOVP T,{L16}"),
    ("MOVS", "0001 0001 LLLL LLLL", None, "MOVS T,{L16}"),
    ("MOVU_ACC_LOC", "0000 1110 LLLL LLLL", None, "MOVU ACC,{L16}"),
    ("MOVU_LOC_OVC", "0101 0110 0010 1000", "0000 0000 LLLL LLLL", "MOVU {L16},OVC"),
    ("MOVU_OVC_LOC", "0101 0110 0110 0010", "0000 0000 LLLL LLLL", "MOVU OVC,{L16}"),
    ("MOVW_DP", "0111 0110 0001 1111", "CCCC CCCC CCCC CCCC", "MOVW DP,#0x{C:x}"),
    ("MOVX_TL", "0101 0110 0010 0001", "xxxx xxxx LLLL LLLL", "MOVX TL,{L16}"),
    ("MOVZ_ARN", "0101 1nnn LLLL LLLL", None, "MOVZ AR{n},{L16}"),
    ("MOVZ_AR6", "1000 1000 LLLL LLLL", None, "MOVZ AR6,{L16}"),
    ("MOVZ_AR7", "1000 0000 LLLL LLLL", None, "MOVZ AR7,{L16}"),
    ("MOVZ_DP", "1011 10CC CCCC CCCC", None, "MOVZ DP,#0x{C:x}"),
    ("MPY_ACC_LOC_IMM", "0011 0100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MPY ACC,{L16},#{Cs}"),
    ("MPY_ACC_T_LOC", "0001 0010 LLLL LLLL", None, "MPY ACC,T,{L16}"),
    ("MPY_P_LOC_IMM", "1000 1100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MPY P,{L16},#{Cs}"),
    ("MPY_P_T_LOC", "0011 0011 LLLL LLLL", None, "MPY P,T,{L16}"),
    ("MPYA_P_LOC_IMM", "0001 0101 LLLL LLLL", "CCCC CCCC CCCC CCCC", "MPYA P,{L16},#{Cs}"),
    ("MPYA_P_T_LOC", "0001 0111 LLLL LLLL", None, "MPYA P,T,{L16}"),
    ("MPYB_ACC", "0011 0101 CCCC CCCC", None, "MPYB ACC,T,#{C}"),
    ("MPYB_P", "0011 0001 CCCC CCCC", None, "MPYB P,T,#{C}"),
    ("MPYS", "0001 0011 LLLL LLLL", None, "MPYS P,T,{L16}"),
    ("MPYU_P", "0011 0111 LLLL LLLL", None, "MPYU P,T,{L16}"),
    ("MPYU_ACC", "0011 0110 LLLL LLLL", None, "MPYU ACC,T,{L16}"),
    ("MPYXU_ACC", "0011 0000 LLLL LLLL", None, "MPYXU ACC,T,{L16}"),
    ("MPYXU_P", "0011 0010 LLLL LLLL", None, "MPYXU P,T,{L16}"),
    ("NASP", "0111 0110 0001 0111", None, "NASP"),
    ("NEG_ACC", "1111 1111 0101 0100", None, "NEG ACC"),
    ("NEG_AX", "1111 1111 0101 110A", None, "NEG {A}"),
    ("NEG64", "0101 0110 0101 1000", None, "NEG64 ACC:P"),
    ("NEGTC", "0101 0110 0011 0010", None, "NEGTC ACC"),
    ("NOP", "0111 0111 LLLL LLLL", None, "NOP {L16}"),
    ("NORM_ACC_IND", "0101 0110 0010 0100", None, "NORM ACC,*"),
    ("NORM_XARN", "1111 1111 0111 Bnnn", None, "NORM ACC,XAR{n}{NORM}"),
    ("NOT_ACC", "1111 1111 0101 0101", None, "NOT ACC"),
    ("NOT_AX", "1111 1111 0101 111A", None, "NOT {A}"),
    ("OR_ACC_LOC", "1010 1111 LLLL LLLL", None, "OR ACC,{L16}"),
    ("OR_ACC_IMM", "0011 1110 0001 SSSS", "CCCC CCCC CCCC CCCC", "OR ACC,#{C}<<#{S}"),
    ("OR_ACC_IMM16", "0101 0110 0100 1010", "CCCC CCCC CCCC CCCC", "OR ACC,#{C}<<#16"),
    ("OR_AX_LOC", "1100 101A LLLL LLLL", None, "OR {A},{L16}"),
    ("OR_IER", "0111 0110 0010 0011", "CCCC CCCC CCCC CCCC", "OR IER,#{C}"),
    ("OR_IFR", "0111 0110 0010 0111", "CCCC CCCC CCCC CCCC", "OR IFR,#{C}"),
    ("OR_LOC_IMM", "0001 1010 LLLL LLLL", "CCCC CCCC CCCC CCCC", "OR {L16},#{C}"),
    ("OR_LOC_AX", "1001 100A LLLL LLLL", None, "OR {L16},{A}"),
    ("ORB", "0101 000A CCCC CCCC", None, "ORB {A},#{C}"),
    ("OUT", "1011 1100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "OUT *(0x{C:x}),{L16}"),
    ("POP_AR1_AR0", "0111 0110 0000 0111", None, "POP AR1:AR0"),
    ("POP_AR3_AR2", "0111 0110 0000 0101", None, "POP AR3:AR2"),
    ("POP_AR5_AR4", "0111 0110 0000 0110", None, "POP AR5:AR4"),
    ("POP_AR1H_AR0H", "0000 0000 0000 0011", None, "POP AR1H:AR0H"),
    ("POP_DBGIER", "0111 0110 0001 0010", None, "POP DBGIER"),
    ("POP_DP", "0111 0110 0000 0011", None, "POP DP"),
    ("POP_DP_ST1", "0111 0110 0000 0001", None, "POP DP:ST1"),
    ("POP_IFR", "0000 0000 0000 0010", None, "POP IFR"),
    ("POP_LOC", "0010 1010 LLLL LLLL", None, "POP {L16}"),
    ("POP_P", "0111 0110 0001 0001", None, "POP P"),
    ("POP_RPC", "0000 0000 0000 0111", None, "POP RPC"),
    ("POP_ST0", "0111 0110 0001 0011", None, "POP ST0"),
    ("POP_ST1", "0111 0110 0000 0000", None, "POP ST1"),
    ("POP_T_ST0", "0111 0110 0001 0101", None, "POP T:ST0"),
    ("PREAD", "0010 0100 LLLL LLLL", None, "PREAD {L16},*XAR7"),
    ("PUSH_AR1_AR0", "0111 0110 0000 1101", None, "PUSH AR1:AR0"),
    ("PUSH_AR3_AR2", "0111 0110 0000 1111", None, "PUSH AR3:AR2"),
    ("PUSH_AR5_AR4", "0111 0110 0000 1100", None, "PUSH AR5:AR4"),
    ("PUSH_AR1H_AR0H", "0000 0000 0000 0101", None, "PUSH AR1H:AR0H"),
    ("PUSH_DBGIER", "0111 0110 0000 1110", None, "PUSH DBGIER"),
    ("PUSH_DP", "0111 0110 0000 1011", None, "PUSH DP"),
    ("PUSH_DP_ST1", "0111 0110 0000 1001", None, "PUSH DP:ST1"),
    ("PUSH_IFR", "0111 0110 0000 1010", None, "PUSH IFR"),
    ("PUSH_LOC", "0010 0010 LLLL LLLL", None, "PUSH {L16}"),
    ("PUSH_P", "0111 0110 0001 1101", None, "PUSH P"),
    ("PUSH_RPC", "0000 0000 0000 0100", None, "PUSH RPC"),
    ("PUSH_ST0", "0111 0110 0001 1000", None, "PUSH ST0"),
    ("PUSH_ST1", "0111 0110 0000 1000", None, "PUSH ST1"),
    ("PUSH_T_ST0", "0111 0110 0001 1001", None, "PUSH T:ST0"),
    ("PWRITE", "0010 0110 LLLL LLLL", None, "PWRITE *XAR7,{L16}"),
    ("QMACL", "0101 0110 0100 1111", "1n00 0111 LLLL LLLL", "QMACL P,{L32},{X7}"),
    ("QMPYAL", "0101 0110 0100 0110", "0000 0000 LLLL LLLL", "QMPYAL P,XT,{L32}"),
    ("QMPYL_P", "0101 0110 0110 0111", "0000 0000 LLLL LLLL", "QMPYL P,XT,{L32}"),
    ("QMPYL_ACC", "0101 0110 0110 0011", "0000 0000 LLLL LLLL", "QMPYL ACC,XT,{L32}"),
    ("QMPYSL", "0101 0110 0100 0101", "0000 0000 LLLL LLLL", "QMPYSL P,XT,{L32}"),
    ("QMPYUL", "0101 0110 0100 0111", "0000 0000 LLLL LLLL", "QMPYUL P,XT,{L32}"),
    ("QMPYXUL", "0101 0110 0100 0010", "0000 0000 LLLL LLLL", "QMPYXUL P,XT,{L32}"),
    ("ROL", "1111 1111 0101 0011", None, "ROL ACC"),
    ("ROR", "1111 1111 0101 0010", None, "ROR ACC"),
    ("RPT_IMM", "1111 0110 CCCC CCCC", None, "RPT #{C}"),
    ("RPT_LOC", "1111 0111 LLLL LLLL", None, "RPT {L16}"),
    ("SAT", "1111 1111 0101 0111", None, "SAT ACC"),
    ("SAT64", "0101 0110 0011 1110", None, "SAT64 ACC:P"),
    ("SB", "0110 KKKK CCCC CCCC", None, "SB {T8},{K}"),
    ("SBBU", "0001 1101 LLLL LLLL", None, "SBBU ACC,{L16}"),
    ("SBF_EQ", "1110 1100 CCCC CCCC", None, "SBF {T8},EQ"),
    ("SBF_NEQ", "1110 1101 CCCC CCCC", None, "SBF {T8},NEQ"),
    ("SBF_TC", "1110 1110 CCCC CCCC", None, "SBF {T8},TC"),
    ("SBF_NTC", "1110 1111 CCCC CCCC", None, "SBF {T8},NTC"),
    ("SBRK", "1111 1101 CCCC CCCC", None, "SBRK #{C}"),
    ("SETC", "0011 1011 CCCC CCCC", None, "SETC {MODE}"),
    ("SETC_M0M1MAP", "0101 0110 0001 1010", None, "SETC M0M1MAP"),
    ("SETC_OBJMODE", "0101 0110 0001 1111", None, "SETC OBJMODE"),
    ("SETC_XF", "0101 0110 0010 0110", None, "SETC XF"),
    ("SFR_IMM", "1111 1111 0100 SSSS", None, "SFR ACC,#{S1}"),
    ("SFR_T", "1111 1111 0101 0001", None, "SFR ACC,T"),
    ("SPM", "1111 1111 0110 1SSS", None, "SPM {PM}"),
    ("SQRA", "0101 0110 0001 0101", "0000 0000 LLLL LLLL", "SQRA {L16}"),
    ("SQRS", "0101 0110 0001 0001", "xxxx xxxx LLLL LLLL", "SQRS {L16}"),
    ("SUB_ACC_LOC", "1010 1110 LLLL LLLL", None, "SUB ACC,{L16}"),
    ("SUB_ACC_LOC_S", "0101 0110 0000 0000", "0000 SSSS LLLL LLLL", "SUB ACC,{L16}<<#{S}"),
    ("SUB_ACC_LOC_16", "0000 0100 LLLL LLLL", None, "SUB ACC,{L16}<<#16"),
    ("SUB_ACC_LOC_T", "0101 0110 0010 0111", "0000 0000 LLLL LLLL", "SUB ACC,{L16}<<T"),
    ("SUB_ACC_IMM", "1111 1111 0000 SSSS", "CCCC CCCC CCCC CCCC", "SUB ACC,#{C}<<#{S}"),
    ("SUB_AX_LOC", "1001 111A LLLL LLLL", None, "SUB {A},{L16}"),
    ("SUB_LOC_AX", "0111 010A LLLL LLLL", None, "SUB {L16},{A}"),
    ("SUBB_ACC", "0001 1001 CCCC CCCC", None, "SUBB ACC,#{C}"),
    ("SUBB_SP", "1111 1110 1CCC CCCC", None, "SUBB SP,#{C}"),
    ("SUBB_XAR", "1101 1nnn 1CCC CCCC", None, "SUBB XAR{n},#{C}"),
    ("SUBBL", "0101 0110 0101 0100", "0000 0000 LLLL LLLL", "SUBBL ACC,{L32}"),
    ("SUBCU", "0001 1111 LLLL LLLL", None, "SUBCU ACC,{L16}"),
    ("SUBCUL", "0101 0110 0001 0111", "0000 0000 LLLL LLLL", "SUBCUL ACC,{L32}"),
    ("SUBL_ACC_LOC", "0000 0011 LLLL LLLL", None, "SUBL ACC,{L32}"),
    ("SUBL_ACC_P", "0001 0001 1010 1100", None, "SUBL ACC,P<<PM"),
    ("SUBL_LOC_ACC", "0101 0110 0100 0001", "0000 0000 LLLL LLLL", "SUBL {L32},ACC"),
    ("SUBR_LOC_AX", "1110 101A LLLL LLLL", None, "SUBR {L16},{A}"),
    ("SUBRL", "0101 0110 0100 1001", "0000 0000 LLLL LLLL", "SUBRL {L32},ACC"),
    ("SUBU", "0000 0001 LLLL LLLL", None, "SUBU ACC,{L16}"),
    ("SUBUL_ACC", "0101 0110 0101 0101", "0000 0000 LLLL LLLL", "SUBUL ACC,{L32}"),
    ("SUBUL_P", "0101 0110 0101 1101", "0000 0000 LLLL LLLL", "SUBUL P,{L32}"),
    ("TBIT_IMM", "0100 BBBB LLLL LLLL", None, "TBIT {L16},#{B}"),
    ("TBIT_T", "0101 0110 0010 0101", "0000 0000 LLLL LLLL", "TBIT {L16},T"),
    ("TCLR", "0101 0110 0000 1001", "0000 BBBB LLLL LLLL", "TCLR {L16},#{B}"),
    ("TEST", "1111 1111 0101 1000", None, "TEST ACC"),
    ("TRAP", "0000 0000 001C CCCC", None, "TRAP #{C}"),
    ("TSET", "0101 0110 0000 1101", "0000 BBBB LLLL LLLL", "TSET {L16},#{B}"),
    ("UOUT", "1011 0000 LLLL LLLL", "CCCC CCCC CCCC CCCC", "UOUT *(0x{C:x}),{L16}"),
    ("XB_AL", "0101 0110 0001 0100", None, "XB *AL"),
    ("XB_PMA_COND", "0101 0110 1101 KKKK", "CCCC CCCC CCCC CCCC", "XB 0x3F{C:04x},{K}"),
    ("XCALL_AL", "0101 0110 0011 0100", None, "XCALL *AL"),
    ("XCALL_PMA_COND", "0101 0110 1110 KKKK", "CCCC CCCC CCCC CCCC", "XCALL 0x3F{C:04x},{K}"),
    ("XMAC", "1000 0100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "XMAC P,{L16},*(0x3F{C:04x})"),
    ("XMACD", "1010 0100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "XMACD P,{L16},*(0x3F{C:04x})"),
    ("XOR_ACC_LOC", "1011 0111 LLLL LLLL", None, "XOR ACC,{L16}"),
    ("XOR_ACC_IMM", "0011 1110 0010 SSSS", "CCCC CCCC CCCC CCCC", "XOR ACC,#{C}<<#{S}"),
    ("XOR_ACC_IMM16", "0101 0110 0100 1110", "CCCC CCCC CCCC CCCC", "XOR ACC,#{C}<<#16"),
    ("XOR_AX_LOC", "0111 000A LLLL LLLL", None, "XOR {A},{L16}"),
    ("XOR_LOC_AX", "1111 001A LLLL LLLL", None, "XOR {L16},{A}"),
    ("XOR_LOC_IMM", "0001 1100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "XOR {L16},#{C}"),
    ("XORB", "1111 000A CCCC CCCC", None, "XORB {A},#{C}"),
    ("XPREAD_PMA", "1010 1100 LLLL LLLL", "CCCC CCCC CCCC CCCC", "XPREAD {L16},*(0x3F{C:04x})"),
    ("XPREAD_AL", "0101 0110 0011 1100", "0000 0000 LLLL LLLL", "XPREAD {L16},*AL"),
    ("XPWRITE_AL", "0101 0110 0011 1101", "0000 0000 LLLL LLLL", "XPWRITE *AL,{L16}"),
    ("XRETC", "0101 0110 1111 KKKK", None, "XRETC {K}"),
    ("ZALR", "0101 0110 0001 0011", "0000 0000 LLLL LLLL", "ZALR ACC,{L16}"),
    ("ZAPA", "0101 0110 0011 0011", None, "ZAPA"),
]

#: instructions that may follow RPT and execute N+1 times
REPEATABLE = {
    "ADD_ACC_LOC", "ADD_ACC_LOC_S", "ADD_ACC_LOC_16", "ADDCU", "ADDL_ACC_LOC", "ADDU", "ADDUL_P", "ADDUL_ACC",
    "ADDCL", "ASR64_IMM", "ASR64_T", "DMAC", "IMACL", "IMPYAL", "IMPYSL", "IMPYXUL", "MAC_PMA", "MAC_XAR7",
    "MAXCUL", "MINCUL", "MOV_LOC_IMM", "MOV_LOC_0", "MOV_LOC_AX", "MOV_LOC_ACC_1", "MOV_LOC_ACC_S", "MOVH_LOC_ACC_1",
    "MOVH_LOC_ACC_S", "MOVL_LOC_ACC", "NOP", "NORM_ACC_IND", "NORM_XARN", "PREAD", "PWRITE", "QMACL", "QMPYAL",
    "QMPYSL", "ROL", "ROR", "SQRA", "SQRS", "SUB_ACC_LOC", "SUB_ACC_LOC_S", "SUB_ACC_LOC_16", "SUBBL",
    "SUBCU", "SUBCUL", "SUBL_ACC_LOC", "SUBU", "SUBUL_ACC", "SUBUL_P", "XMAC", "XMACD", "XPREAD_PMA",
    "XPREAD_AL", "XPWRITE_AL", "LSL64_IMM", "LSL64_T", "LSR64_IMM", "LSR64_T", "LSL_ACC_IMM", "LSL_ACC_T",
    "SFR_IMM", "SFR_T", "MOV_ABS_LOC", "MOV_LOC_ABS", "MOV_LOC_P", "MOVH_LOC_P", "MOVL_LOC_XT", "MOVL_LOC_P",
    "SBBU", "MOVDL", "MOVAD", "DMOV", "MPYA_P_T_LOC", "MPYS", "MOVA", "MOVS", "MOVP",
}


def _parse(pat: Optional[str]):
    if pat is None:
        return None
    bits = pat.replace(" ", "")
    assert len(bits) == 16, pat
    mask = val = 0
    for i, ch in enumerate(bits):
        b = 15 - i
        if ch in "01":
            mask |= 1 << b
            if ch == "1":
                val |= 1 << b
    return mask, val, bits


def _fields(bits1: str, w1: int, bits2: Optional[str], w2: int) -> Dict[str, int]:
    f: Dict[str, int] = {}
    for bits, w in ((bits1, w1), (bits2, w2)):
        if bits is None:
            continue
        for i, ch in enumerate(bits):
            if ch in "01x":
                continue
            f[ch] = (f.get(ch, 0) << 1) | ((w >> (15 - i)) & 1)
    return f


_ENTRIES = []
for _k, _p1, _p2, _syn in TABLE:
    _m1 = _parse(_p1)
    _m2 = _parse(_p2)
    _spec = bin(_m1[0]).count("1") * 2 + (bin(_m2[0]).count("1") if _m2 else 0)
    _ENTRIES.append((_spec, _k, _m1, _m2, _syn))
_ENTRIES.sort(key=lambda e: -e[0])
_CAND: Dict[int, list] = {}


def candidates(w1: int) -> list:
    c = _CAND.get(w1)
    if c is None:
        c = _CAND[w1] = [e for e in _ENTRIES if (w1 & e[2][0]) == e[2][1]]
    return c


def match(w1: int, w2: int):
    """(key, fields, size, syntax) for the instruction words, or None."""
    for spec, k, m1, m2, syn in candidates(w1):
        if m2 is not None and (w2 & m2[0]) != m2[1]:
            continue
        f = _fields(m1[2], w1, m2[2] if m2 else None, w2)
        return k, f, 2 if m2 else 1, syn
    return None


# ---------------------------------------------------------------- disassembly text
MODE_BITS = ("SXM", "OVM", "TC", "C", "INTM", "DBGM", "PAGE0", "VMAP")


def render(key, f, syn, addr, size):
    out = syn
    if "{L16}" in out:
        out = out.replace("{L16}", loc_text(f["L"], 16))
    if "{L32}" in out:
        out = out.replace("{L32}", loc_text(f["L"], 32))
    if "{A}" in out:
        out = out.replace("{A}", "AH" if f.get("A") else "AL")
    if "{K}" in out:
        out = out.replace("{K}", COND_NAMES[f["K"]])
    if "{T16}" in out:
        out = out.replace("{T16}", f"0x{(addr + s16(f['C'])) & M22:06x}")
    if "{T8}" in out:
        c = f["C"]
        out = out.replace("{T8}", f"0x{(addr + (c - 256 if c & 0x80 else c)) & M22:06x}")
    if "{T22}" in out:
        out = out.replace("{T22}", f"0x{f['C'] & M22:06x}")
    if "{Cs8}" in out:
        c = f["C"]
        out = out.replace("{Cs8}", str(c - 256 if c & 0x80 else c))
    if "{Cs}" in out:
        out = out.replace("{Cs}", str(s16(f["C"])))
    if "{S1}" in out:
        out = out.replace("{S1}", str(f["S"] + 1))
    if "{MODE}" in out:
        out = out.replace("{MODE}", ",".join(n for i, n in enumerate(MODE_BITS) if f["C"] >> i & 1) or "0")
    if "{X7}" in out:
        out = out.replace("{X7}", "*XAR7" if f["n"] else "*XAR7++")
    if "{BAR}" in out:
        out = out.replace("{BAR}", "NEQ" if f["B"] else "EQ")
    if "{NORM}" in out:
        out = out.replace("{NORM}", "++" if f["B"] else "--")
    if "{PM}" in out:
        out = out.replace("{PM}", ("+1", "0", "-1", "-2", "-3", "-4", "-5", "-6")[f["S"]])
    if "{CMPR}" in out:
        out = out.replace("{CMPR}", str({5: 0, 1: 1, 0: 2, 4: 3}.get(f["K"], "?")))
    if "{INTR}" in out:
        out = out.replace("{INTR}", f"INT{f['C'] + 1}" if f["C"] < 14 else ("DLOGINT", "RTOSINT")[f["C"] - 14])
    try:
        return out.format(**{k: v for k, v in f.items() if k in "CSnmB"})
    except (KeyError, ValueError, IndexError):
        return out


def disasm(read16: Callable[[int], int], addr: int) -> Tuple[str, int]:
    w1 = read16(addr)
    w2 = read16((addr + 1) & M22)
    m = match(w1, w2)
    if m is None:
        return f".word 0x{w1:04x}", 1
    k, f, size, syn = m
    return render(k, f, syn, addr, size), size
