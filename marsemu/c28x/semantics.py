"""Executable semantics of the C28x instructions (see :mod:`.isa`).

``build(key, f, addr, size)`` returns ``fn(cpu)``.  ``cpu.pc`` already
points at the next instruction when ``fn`` runs; control flow overwrites it.
Flag behaviour follows the "Flags and Modes" tables of SPRU430F.
"""
from __future__ import annotations

from .isa import (M16, M22, M32, Unimplemented, acc_add, acc_sub, add16, cond_fn, ext16, loc, nz16, nz32,
                  ovc_count, pshift, s16, s32, sub16)


def _ax_get(A):
    if A:
        return lambda c: c.acc >> 16
    return lambda c: c.acc & M16


def _ax_set(A):
    if A:
        def s(c, v):
            c.acc = (c.acc & M16) | (v << 16)
    else:
        def s(c, v):
            c.acc = (c.acc & 0xFFFF0000) | v
    return s


def _store16(lo, c, v):
    """Store to a loc16; a store to @AH/@AL sets N and Z."""
    lo.wr(c, v)
    if lo.is_ax:
        nz16(c, v)


def _store32(lo, c, v):
    lo.wr(c, v)
    if lo.is_acc:
        nz32(c, v)


def _shl(v, n, width):
    """(result, carry) of a logical left shift by n (n >= 1)."""
    m = (1 << width) - 1
    return (v << n) & m, (v >> (width - n)) & 1


def _push16(c, v):
    c.wr16(c.sp, v)
    c.sp = (c.sp + 1) & M16


def _pop16(c):
    c.sp = (c.sp - 1) & M16
    return c.rd16(c.sp)


def _push32(c, v):
    c.wr32(c.sp, v)
    c.sp = (c.sp + 2) & M16


def _pop32(c):
    c.sp = (c.sp - 2) & M16
    return c.rd32(c.sp)


def _mul_ss(a, b):
    return (s16(a) * s16(b)) & M32


def _q64(a, b):
    """signed 32 x signed 32 -> 64-bit two's complement"""
    return (s32(a) * s32(b)) & 0xFFFFFFFFFFFFFFFF


def _p_from_temp(c, temp):
    """IMPYAL/IMPYSL/IMACL: the PM-selected 32 bits of the low 38 product bits."""
    pm = c.pm
    if pm == 1:
        return temp & M32
    if pm == 0:
        return (temp << 1) & M32
    if pm == 5 and c.amode:
        return (temp << 4) & M32
    return (temp >> (pm - 1)) & M32


def build(key: str, f: dict, addr: int, size: int):   # noqa: C901 - one big table
    nxt = (addr + size) & M22
    L = f.get("L")
    A = f.get("A", 0)
    C = f.get("C", 0)

    def l16():
        return loc(L, 16)

    def l32():
        return loc(L, 32)

    # ------------------------------------------------------------ moves
    if key == "MOV_AX_LOC":
        lo, put = l16(), _ax_set(A)

        def fn(c):
            v = lo.rd(c)
            put(c, v)
            nz16(c, v)
        return fn
    if key == "MOV_LOC_AX":
        lo, get = l16(), _ax_get(A)

        def fn(c):
            v = get(c)
            lo.wr(c, v)
            if lo.is_ax:
                nz16(c, v)
        return fn
    if key == "MOV_LOC_AX_COND":
        lo, get, cond = l16(), _ax_get(A), cond_fn(f["K"])

        def fn(c):
            if cond(c):
                _store16(lo, c, get(c))
            elif lo.ea is not None:
                lo.ea(c)   # the address mode's side effect happens either way
        return fn
    if key == "MOV_LOC_IMM":
        lo, v = l16(), C

        def fn(c):
            _store16(lo, c, v)
        return fn
    if key == "MOV_LOC_0":
        lo = l16()

        def fn(c):
            _store16(lo, c, 0)
        return fn
    if key == "MOV_ACC_LOC":
        lo = l16()

        def fn(c):
            v = ext16(c, lo.rd(c))
            c.acc = v
            nz32(c, v)
        return fn
    if key in ("MOV_ACC_LOC_S", "MOV_ACC_LOC_16"):
        lo, sh = l16(), (16 if key.endswith("16") else f["S"])

        def fn(c):
            v = (ext16(c, lo.rd(c)) << sh) & M32
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MOV_ACC_LOC_T":
        lo = l16()

        def fn(c):
            v = (ext16(c, lo.rd(c)) << ((c.xt >> 16) & 15)) & M32
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MOV_ACC_IMM":
        sh = f["S"]

        def fn(c):
            v = (ext16(c, C) << sh) & M32
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MOVU_ACC_LOC":
        lo = l16()

        def fn(c):
            v = lo.rd(c)
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MOVB_ACC":
        def fn(c):
            c.acc = C
            nz32(c, C)
        return fn
    if key == "MOVB_AX":
        put = _ax_set(A)

        def fn(c):
            put(c, C)
            nz16(c, C)
        return fn
    if key in ("MOVB_XARN", "MOVB_XAR6", "MOVB_XAR7"):
        n = f.get("n", 6 if key == "MOVB_XAR6" else 7)

        def fn(c):
            c.xar[n] = C
        return fn
    if key in ("MOVB_AR6", "MOVB_AR7"):
        n = 6 if key == "MOVB_AR6" else 7

        def fn(c):
            c.xar[n] = (c.xar[n] & 0xFFFF0000) | C
        return fn
    if key.startswith("MOVL_XAR") and key.endswith("_IMM"):
        n = int(key[8])

        def fn(c):
            c.xar[n] = C
        return fn
    if key.startswith("MOVL_XAR") and key.endswith("_LOC"):
        n, lo = int(key[8]), l32()

        def fn(c):
            c.xar[n] = lo.rd(c)
        return fn
    if key.startswith("MOVL_LOC_XAR"):
        n, lo = int(key[12]), l32()

        def fn(c):
            _store32(lo, c, c.xar[n])
        return fn
    if key == "MOVL_ACC_LOC":
        lo = l32()

        def fn(c):
            v = lo.rd(c)
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MOVL_LOC_ACC":
        lo = l32()

        def fn(c):
            _store32(lo, c, c.acc)
        return fn
    if key == "MOVL_LOC_ACC_COND":
        lo, cond = l32(), cond_fn(f["K"])

        def fn(c):
            if cond(c):
                _store32(lo, c, c.acc)
            elif lo.ea is not None:
                lo.ea(c)
        return fn
    if key == "MOVL_P_LOC":
        lo = l32()

        def fn(c):
            c.p = lo.rd(c)
        return fn
    if key == "MOVL_LOC_P":
        lo = l32()

        def fn(c):
            _store32(lo, c, c.p)
        return fn
    if key == "MOVL_P_ACC":
        def fn(c):
            c.p = c.acc
        return fn
    if key == "MOVL_XT_LOC":
        lo = l32()

        def fn(c):
            c.xt = lo.rd(c)
        return fn
    if key == "MOVL_LOC_XT":
        lo = l32()

        def fn(c):
            _store32(lo, c, c.xt)
        return fn
    if key == "MOVL_ACC_P":
        def fn(c):
            v = pshift(c)
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MOVDL":
        lo = l32()

        def fn(c):
            a = lo.ea(c)
            v = c.rd32(a)
            c.xt = v
            c.wr32((a & ~1) + 2, v)
        return fn
    if key == "MOVW_DP":
        def fn(c):
            c.dp = C
        return fn
    if key in ("MOV_DP_IMM", "MOVZ_DP"):
        def fn(c):
            c.dp = C
        return fn
    if key == "MOVZ_ARN":
        n, lo = f["n"], l16()

        def fn(c):
            c.xar[n] = lo.rd(c)
        return fn
    if key in ("MOVZ_AR6", "MOVZ_AR7"):
        n, lo = int(key[-1]), l16()

        def fn(c):
            c.xar[n] = lo.rd(c)
        return fn
    if key in ("MOV_AR6_LOC", "MOV_AR7_LOC"):
        n, lo = int(key[6]), l16()

        def fn(c):
            c.xar[n] = (c.xar[n] & 0xFFFF0000) | lo.rd(c)
        return fn
    if key == "MOV_LOC_ARN":
        n, lo = f["n"], l16()

        def fn(c):
            _store16(lo, c, c.xar[n] & M16)
        return fn
    if key == "MOV_XARN_PC":
        n = f["n"]

        def fn(c):
            c.xar[n] = addr
        return fn
    if key == "MOV_T_LOC":
        lo = l16()

        def fn(c):
            c.xt = (c.xt & M16) | (lo.rd(c) << 16)
        return fn
    if key == "MOV_LOC_T":
        lo = l16()

        def fn(c):
            _store16(lo, c, c.xt >> 16)
        return fn
    if key == "MOV_TL_0":
        def fn(c):
            c.xt &= 0xFFFF0000
        return fn
    if key == "MOVX_TL":
        lo = l16()

        def fn(c):
            v = lo.rd(c)
            c.xt = v | (0xFFFF0000 if v & 0x8000 else 0)
        return fn
    if key == "MOV_PH_LOC":
        lo = l16()

        def fn(c):
            c.p = (c.p & M16) | (lo.rd(c) << 16)
        return fn
    if key == "MOV_PL_LOC":
        lo = l16()

        def fn(c):
            c.p = (c.p & 0xFFFF0000) | lo.rd(c)
        return fn
    if key == "MOV_LOC_P":
        lo = l16()

        def fn(c):
            _store16(lo, c, pshift(c) & M16)
        return fn
    if key == "MOVH_LOC_P":
        lo = l16()

        def fn(c):
            _store16(lo, c, pshift(c) >> 16)
        return fn
    if key in ("MOV_LOC_ACC_1", "MOV_LOC_ACC_S"):
        lo, sh = l16(), (1 if key.endswith("_1") else f["S"])

        def fn(c):
            _store16(lo, c, (c.acc << sh) & M16)
        return fn
    if key in ("MOVH_LOC_ACC_1", "MOVH_LOC_ACC_S"):
        lo, sh = l16(), (1 if key.endswith("_1") else f["S"])

        def fn(c):
            _store16(lo, c, ((c.acc << sh) >> 16) & M16)
        return fn
    if key == "MOV_LOC_IER":
        lo = l16()

        def fn(c):
            _store16(lo, c, c.ier)
        return fn
    if key == "MOV_IER_LOC":
        lo = l16()

        def fn(c):
            c.ier = lo.rd(c)
        return fn
    if key in ("MOV_LOC_OVC", "MOVU_LOC_OVC"):
        lo = l16()

        def fn(c):
            _store16(lo, c, (c.ovc & 63) << 10)
        return fn
    if key == "MOV_OVC_LOC":
        lo = l16()

        def fn(c):
            v = lo.rd(c) >> 10
            c.ovc = v - 64 if v & 32 else v
        return fn
    if key == "MOVU_OVC_LOC":
        lo = l16()

        def fn(c):
            v = lo.rd(c) & 63
            c.ovc = v - 64 if v & 32 else v
        return fn
    if key == "MOV_PM_AX":
        get = _ax_get(A)

        def fn(c):
            c.pm = get(c) & 7
        return fn
    if key == "MOV_ABS_LOC":   # *(0:16bit) = [loc16]
        lo = l16()

        def fn(c):
            c.wr16(C, lo.rd(c))
        return fn
    if key == "MOV_LOC_ABS":
        lo = l16()

        def fn(c):
            _store16(lo, c, c.rd16(C))
        return fn
    if key == "MOVB_LOC_IMM_COND":
        lo, cond = l16(), cond_fn(f["K"])

        def fn(c):
            if cond(c):
                _store16(lo, c, C)
            elif lo.ea is not None:
                lo.ea(c)
        return fn
    if key in ("MOVB_AX_LSB", "MOVB_AX_MSB", "MOVB_LOC_AX_LSB", "MOVB_LOC_AX_MSB"):
        return _movb(key, L, A)
    if key == "DMOV":
        lo = l16()

        def fn(c):
            a = lo.ea(c)
            c.wr16((a + 1) & M22, c.rd16(a))
        return fn
    # ------------------------------------------------------------ 16-bit AX arithmetic / logic
    if key in ("ADD_AX_LOC", "SUB_AX_LOC", "AND_AX_LOC", "OR_AX_LOC", "XOR_AX_LOC"):
        lo, get, put = l16(), _ax_get(A), _ax_set(A)
        op = key[:-7]

        def fn(c):
            b = lo.rd(c)
            a = get(c)
            if op == "ADD":
                r = add16(c, a, b)
            elif op == "SUB":
                r = sub16(c, a, b)
            else:
                r = a & b if op == "AND" else (a | b if op == "OR" else a ^ b)
                nz16(c, r)
            put(c, r)
        return fn
    if key == "AND_AX_LOC_IMM":
        lo, put = l16(), _ax_set(A)

        def fn(c):
            r = lo.rd(c) & C
            put(c, r)
            nz16(c, r)
        return fn
    if key in ("ADD_LOC_AX", "SUB_LOC_AX", "AND_LOC_AX", "OR_LOC_AX", "XOR_LOC_AX", "SUBR_LOC_AX"):
        lo, get = l16(), _ax_get(A)
        op = key[:-7]

        def fn(c):
            ax = get(c)

            def m(v):
                if op == "ADD":
                    return add16(c, v, ax)
                if op == "SUB":
                    return sub16(c, v, ax)
                if op == "SUBR":
                    return sub16(c, ax, v)
                r = v & ax if op == "AND" else (v | ax if op == "OR" else v ^ ax)
                nz16(c, r)
                return r
            lo.rmw(c, m)
        return fn
    if key in ("ADD_LOC_IMM", "AND_LOC_IMM", "OR_LOC_IMM", "XOR_LOC_IMM"):
        lo = l16()
        op = key[:-8]

        def fn(c):
            def m(v):
                if op == "ADD":
                    return add16(c, v, C)
                r = v & C if op == "AND" else (v | C if op == "OR" else v ^ C)
                nz16(c, r)
                return r
            lo.rmw(c, m)
        return fn
    if key in ("ANDB", "ORB", "XORB"):
        get, put = _ax_get(A), _ax_set(A)

        def fn(c):
            a = get(c)
            r = a & C if key == "ANDB" else (a | C if key == "ORB" else a ^ C)
            put(c, r)
            nz16(c, r)
        return fn
    if key == "ADDB_AX":
        get, put = _ax_get(A), _ax_set(A)
        b = (C - 256 if C & 0x80 else C) & M16

        def fn(c):
            put(c, add16(c, get(c), b))
        return fn
    if key == "CMP_AX_LOC":
        lo, get = l16(), _ax_get(A)

        def fn(c):
            a, b = get(c), lo.rd(c)
            r = s16(a) - s16(b)
            c.n = 1 if r < 0 else 0
            c.z = 1 if a == b else 0
            c.c = 1 if a >= b else 0
        return fn
    if key == "CMPB":
        get = _ax_get(A)

        def fn(c):
            a = get(c)
            r = s16(a) - C
            c.n = 1 if r < 0 else 0
            c.z = 1 if a == C else 0
            c.c = 1 if a >= C else 0
        return fn
    if key == "CMP_LOC_IMM":
        lo = l16()
        sb = s16(C)

        def fn(c):
            a = lo.rd(c)
            r = s16(a) - sb
            c.n = 1 if r < 0 else 0
            c.z = 1 if a == C else 0
            c.c = 1 if a >= C else 0
        return fn
    if key in ("INC", "DEC"):
        lo = l16()

        def fn(c):
            if key == "INC":
                lo.rmw(c, lambda v: add16(c, v, 1))
            else:
                lo.rmw(c, lambda v: sub16(c, v, 1))
        return fn
    if key == "NEG_AX":
        get, put = _ax_get(A), _ax_set(A)

        def fn(c):
            a = get(c)
            if a == 0x8000:
                c.v = 1
                r = 0x8000
            else:
                r = (-a) & M16
            c.c = 1 if a == 0 else 0
            put(c, r)
            nz16(c, r)
        return fn
    if key == "NOT_AX":
        get, put = _ax_get(A), _ax_set(A)

        def fn(c):
            r = get(c) ^ M16
            put(c, r)
            nz16(c, r)
        return fn
    if key == "FLIP":
        get, put = _ax_get(A), _ax_set(A)

        def fn(c):
            v = get(c)
            r = int(f"{v:016b}"[::-1], 2)
            put(c, r)
            nz16(c, r)
        return fn
    if key in ("MAX", "MIN"):
        lo, get, put = l16(), _ax_get(A), _ax_set(A)

        def fn(c):
            a, b = s16(get(c)), s16(lo.rd(c))
            # flags as for AX - [loc16]
            r = a - b
            c.n = 1 if r < 0 else 0
            c.z = 1 if r == 0 else 0
            c.c = 1 if (a & M16) >= (b & M16) else 0
            if (key == "MAX" and b > a) or (key == "MIN" and b < a):
                put(c, b & M16)
        return fn
    if key in ("ASR_AX_IMM", "LSL_AX_IMM", "LSR_AX_IMM", "ASR_AX_T", "LSL_AX_T", "LSR_AX_T"):
        get, put = _ax_get(A), _ax_set(A)
        kind = key[:3]
        fixed = f["S"] + 1 if key.endswith("IMM") else None

        def fn(c):
            n = fixed if fixed is not None else (c.xt >> 16) & 15
            v = get(c)
            if n == 0:
                c.c = 0
                r = v
            elif kind == "LSL":
                r = (v << n) & M16
                c.c = (v >> (16 - n)) & 1
            elif kind == "LSR":
                r = v >> n
                c.c = (v >> (n - 1)) & 1
            else:
                sv = s16(v)
                r = (sv >> n) & M16
                c.c = (sv >> (n - 1)) & 1
            put(c, r)
            nz16(c, r)
        return fn
    # ------------------------------------------------------------ 32-bit ACC arithmetic
    if key in ("ADD_ACC_LOC", "ADD_ACC_LOC_S", "ADD_ACC_LOC_16"):
        lo = l16()
        sh = 0 if key == "ADD_ACC_LOC" else (16 if key.endswith("16") else f["S"])
        cm = 1 if sh == 16 else 0

        def fn(c):
            acc_add(c, (ext16(c, lo.rd(c)) << sh) & M32, cmode=cm)
        return fn
    if key == "ADD_ACC_LOC_T":
        lo = l16()

        def fn(c):
            acc_add(c, (ext16(c, lo.rd(c)) << ((c.xt >> 16) & 15)) & M32)
        return fn
    if key == "ADD_ACC_IMM":
        sh = f["S"]

        def fn(c):
            acc_add(c, (ext16(c, C) << sh) & M32)
        return fn
    if key in ("SUB_ACC_LOC", "SUB_ACC_LOC_S", "SUB_ACC_LOC_16"):
        lo = l16()
        sh = 0 if key == "SUB_ACC_LOC" else (16 if key.endswith("16") else f["S"])
        cm = 2 if sh == 16 else 0

        def fn(c):
            acc_sub(c, (ext16(c, lo.rd(c)) << sh) & M32, cmode=cm)
        return fn
    if key == "SUB_ACC_LOC_T":
        lo = l16()

        def fn(c):
            acc_sub(c, (ext16(c, lo.rd(c)) << ((c.xt >> 16) & 15)) & M32)
        return fn
    if key == "SUB_ACC_IMM":
        sh = f["S"]

        def fn(c):
            acc_sub(c, (ext16(c, C) << sh) & M32)
        return fn
    if key == "ADDB_ACC":
        def fn(c):
            acc_add(c, C)
        return fn
    if key == "SUBB_ACC":
        def fn(c):
            acc_sub(c, C)
        return fn
    if key == "ADDU":
        lo = l16()

        def fn(c):
            acc_add(c, lo.rd(c))
        return fn
    if key == "SUBU":
        lo = l16()

        def fn(c):
            acc_sub(c, lo.rd(c))
        return fn
    if key == "ADDCU":
        lo = l16()

        def fn(c):
            acc_add(c, lo.rd(c), cin=c.c)
        return fn
    if key == "SBBU":
        lo = l16()

        def fn(c):
            acc_sub(c, lo.rd(c), bin_=1 - c.c)
        return fn
    if key == "ADDL_ACC_LOC":
        lo = l32()

        def fn(c):
            acc_add(c, lo.rd(c))
        return fn
    if key == "SUBL_ACC_LOC":
        lo = l32()

        def fn(c):
            acc_sub(c, lo.rd(c))
        return fn
    if key == "ADDCL":
        lo = l32()

        def fn(c):
            acc_add(c, lo.rd(c), cin=c.c)
        return fn
    if key == "SUBBL":
        lo = l32()

        def fn(c):
            acc_sub(c, lo.rd(c), bin_=1 - c.c)
        return fn
    if key == "ADDUL_ACC":
        lo = l32()

        def fn(c):
            a = c.acc
            r = a + lo.rd(c)
            c.c = r >> 32
            r &= M32
            if c.c:
                ovc_count(c, 1)
            c.acc = r
            nz32(c, r)
        return fn
    if key == "SUBUL_ACC":
        lo = l32()

        def fn(c):
            a = c.acc
            r = a - lo.rd(c)
            c.c = 0 if r < 0 else 1
            r &= M32
            if not c.c:
                ovc_count(c, -1)
            c.acc = r
            nz32(c, r)
        return fn
    if key == "ADDUL_P":
        lo = l32()

        def fn(c):
            r = c.p + lo.rd(c)
            c.c = r >> 32
            r &= M32
            if c.c:
                ovc_count(c, 1)
            c.p = r
            nz32(c, r)
        return fn
    if key == "SUBUL_P":
        lo = l32()

        def fn(c):
            r = c.p - lo.rd(c)
            c.c = 0 if r < 0 else 1
            r &= M32
            if not c.c:
                ovc_count(c, -1)
            c.p = r
            nz32(c, r)
        return fn
    if key == "ADDL_LOC_ACC":
        lo = l32()

        def fn(c):
            def m(v):
                a = v
                b = c.acc
                r = a + b
                c.c = r >> 32
                r &= M32
                if (~(a ^ b) & (a ^ r)) & 0x80000000:
                    c.v = 1
                    if c.ovm:
                        r = 0x80000000 if a & 0x80000000 else 0x7FFFFFFF
                    else:
                        ovc_count(c, -1 if a & 0x80000000 else 1)
                nz32(c, r)
                return r
            lo.rmw(c, m)
        return fn
    if key in ("SUBL_LOC_ACC", "SUBRL"):
        lo = l32()

        def fn(c):
            def m(v):
                a, b = (v, c.acc) if key == "SUBL_LOC_ACC" else (c.acc, v)
                r = a - b
                c.c = 0 if r < 0 else 1
                r &= M32
                if ((a ^ b) & (a ^ r)) & 0x80000000:
                    c.v = 1
                    if c.ovm:
                        r = 0x80000000 if a & 0x80000000 else 0x7FFFFFFF
                    else:
                        ovc_count(c, -1 if a & 0x80000000 else 1)
                nz32(c, r)
                return r
            lo.rmw(c, m)
        return fn
    if key == "ADDL_ACC_P":
        def fn(c):
            acc_add(c, pshift(c))
        return fn
    if key == "SUBL_ACC_P":
        def fn(c):
            acc_sub(c, pshift(c))
        return fn
    if key in ("CMPL_ACC_LOC", "CMPL_ACC_P"):
        lo = l32() if key == "CMPL_ACC_LOC" else None

        def fn(c):
            a = c.acc
            b = lo.rd(c) if lo is not None else pshift(c)
            r = s32(a) - s32(b)
            c.n = 1 if r < 0 else 0
            c.z = 1 if a == b else 0
            c.c = 1 if a >= b else 0
        return fn
    if key == "CMP64":
        def fn(c):
            v = (c.acc << 32) | c.p
            if v & (1 << 63):
                v -= 1 << 64
            if c.v:
                c.n = 0 if v < 0 else 1
            else:
                c.n = 1 if v < 0 else 0
            c.z = 1 if v == 0 else 0
            c.v = 0
        return fn
    if key in ("AND_ACC_LOC", "OR_ACC_LOC", "XOR_ACC_LOC"):
        lo = l16()

        def fn(c):
            b = lo.rd(c)
            r = c.acc & b if key[0] == "A" else (c.acc | b if key[0] == "O" else c.acc ^ b)
            c.acc = r
            nz32(c, r)
        return fn
    if key in ("AND_ACC_IMM", "OR_ACC_IMM", "XOR_ACC_IMM", "AND_ACC_IMM16", "OR_ACC_IMM16", "XOR_ACC_IMM16"):
        sh = 16 if key.endswith("16") else f["S"]
        b = (C << sh) & M32
        op = key[0]

        def fn(c):
            r = c.acc & b if op == "A" else (c.acc | b if op == "O" else c.acc ^ b)
            c.acc = r
            nz32(c, r)
        return fn
    if key == "NEG_ACC":
        def fn(c):
            a = c.acc
            if a == 0x80000000:
                c.v = 1
                r = 0x7FFFFFFF if c.ovm else 0x80000000
            else:
                r = (-a) & M32
            c.c = 1 if a == 0 else 0
            c.acc = r
            nz32(c, r)
        return fn
    if key == "NEGTC":
        def fn(c):
            if c.tc:
                a = c.acc
                if a == 0x80000000:
                    c.v = 1
                    r = 0x7FFFFFFF if c.ovm else 0x80000000
                else:
                    r = (-a) & M32
                c.c = 1 if a == 0 else 0
                c.acc = r
            nz32(c, c.acc)
        return fn
    if key == "NOT_ACC":
        def fn(c):
            r = c.acc ^ M32
            c.acc = r
            nz32(c, r)
        return fn
    if key in ("ABS", "ABSTC"):
        def fn(c):
            a = c.acc
            if a == 0x80000000:
                c.v = 1
                r = 0x7FFFFFFF if c.ovm else 0x80000000
                if key == "ABSTC":
                    c.tc ^= 1
            elif a & 0x80000000:
                r = (-a) & M32
                if key == "ABSTC":
                    c.tc ^= 1
            else:
                r = a
            c.c = 0
            c.acc = r
            nz32(c, r)
        return fn
    if key == "SAT":
        def fn(c):
            if c.ovc > 0:
                c.acc = 0x7FFFFFFF
                c.v = 1
            elif c.ovc < 0:
                c.acc = 0x80000000
                c.v = 1
            else:
                c.v = 0
            c.ovc = 0
            c.c = 0
            nz32(c, c.acc)
        return fn
    if key == "TEST":
        def fn(c):
            nz32(c, c.acc)
        return fn
    if key == "CSB":
        def fn(c):
            a = c.acc
            sign = a >> 31
            n = 0
            for i in range(30, -1, -1):
                if (a >> i) & 1 == sign:
                    n += 1
                else:
                    break
            c.xt = (c.xt & M16) | (n << 16)
            nz32(c, a)
            c.tc = sign
        return fn
    if key in ("LSL_ACC_IMM", "LSL_ACC_T", "LSLL", "SFR_IMM", "SFR_T", "LSRL", "ASRL"):
        fixed = f["S"] + 1 if key.endswith("IMM") else None
        mask5 = key in ("LSLL", "LSRL", "ASRL")

        def fn(c):
            n = fixed if fixed is not None else ((c.xt >> 16) & (31 if mask5 else 15))
            a = c.acc
            if n == 0:
                c.c = 0
                r = a
            elif key.startswith("LSL"):
                r = (a << n) & M32
                c.c = (a >> (32 - n)) & 1
            elif key == "LSRL" or (key.startswith("SFR") and not c.sxm):
                r = a >> n
                c.c = (a >> (n - 1)) & 1
            else:
                sa = s32(a)
                r = (sa >> n) & M32
                c.c = (sa >> (n - 1)) & 1
            c.acc = r
            nz32(c, r)
        return fn
    if key in ("ROL", "ROR"):
        def fn(c):
            a = c.acc
            if key == "ROL":
                r = ((a << 1) | c.c) & M32
                c.c = a >> 31
            else:
                r = (a >> 1) | (c.c << 31)
                c.c = a & 1
            c.acc = r
            nz32(c, r)
        return fn
    if key in ("LSL64_IMM", "LSL64_T", "LSR64_IMM", "ASR64_IMM", "LSR64_T", "ASR64_T"):
        fixed = f["S"] + 1 if key.endswith("IMM") else None

        def fn(c):
            n = fixed if fixed is not None else (c.xt >> 16) & 63
            v = (c.acc << 32) | c.p
            if n == 0:
                c.c = 0
                r = v
            elif key.startswith("LSL"):
                r = (v << n) & 0xFFFFFFFFFFFFFFFF
                c.c = (v >> (64 - n)) & 1 if n <= 64 else 0
            elif key.startswith("LSR"):
                r = v >> n
                c.c = (v >> (n - 1)) & 1
            else:
                sv = v - (1 << 64) if v >> 63 else v
                r = (sv >> n) & 0xFFFFFFFFFFFFFFFF
                c.c = (sv >> (n - 1)) & 1
            c.acc = r >> 32
            c.p = r & M32
            c.n = r >> 63
            c.z = 0 if r else 1
        return fn
    if key == "NEG64":
        def fn(c):
            v = (c.acc << 32) | c.p
            if v == 1 << 63:
                c.v = 1
                r = ((1 << 63) - 1) if c.ovm else v
            else:
                r = (-v) & 0xFFFFFFFFFFFFFFFF
            c.c = 1 if v == 0 else 0
            c.acc, c.p = r >> 32, r & M32
            c.n = r >> 63
            c.z = 0 if r else 1
        return fn
    if key == "SAT64":
        def fn(c):
            if c.ovc > 0:
                c.acc, c.p, c.v = 0x7FFFFFFF, M32, 1
            elif c.ovc < 0:
                c.acc, c.p, c.v = 0x80000000, 0, 1
            c.ovc = 0
            c.c = 0
            r = (c.acc << 32) | c.p
            c.n = r >> 63
            c.z = 0 if r else 1
        return fn
    if key == "SUBCU":
        lo = l16()

        def fn(c):
            b = lo.rd(c)
            a2 = c.acc << 1
            temp = a2 - (b << 16)
            if temp >= 0:
                c.acc = (temp + 1) & M32
                c.c = 1
            else:
                c.acc = a2 & M32
                c.c = 0
            nz32(c, c.acc)
        return fn
    if key == "SUBCUL":
        lo = l32()

        def fn(c):
            b = lo.rd(c)
            temp = (c.acc << 1) + (c.p >> 31) - b
            if temp >= 0:
                c.acc = temp & M32
                c.p = ((c.p << 1) + 1) & M32
                c.c = 1
            else:
                v = ((c.acc << 32) | c.p) << 1
                c.acc = (v >> 32) & M32
                c.p = v & M32
                c.c = 0
            nz32(c, c.acc)
        return fn
    if key in ("MAXL", "MINL"):
        lo = l32()

        def fn(c):
            a, b = s32(c.acc), s32(lo.rd(c))
            r = a - b
            c.n = 1 if r < 0 else 0
            c.z = 1 if r == 0 else 0
            c.c = 1 if (a & M32) >= (b & M32) else 0
            if (key == "MAXL" and b > a) or (key == "MINL" and b < a):
                c.acc = b & M32
        return fn
    if key in ("MAXCUL", "MINCUL"):
        lo = l32()
        mx = key == "MAXCUL"

        def fn(c):
            # the upper halves were compared by MAXL/MINL: N and Z say how
            b = lo.rd(c)
            if not c.z:
                if mx == bool(c.n):
                    c.p = b
            elif (c.p < b) if mx else (c.p > b):
                c.p = b
                c.v = 1
        return fn
    # ------------------------------------------------------------ multiply
    if key == "MPY_P_T_LOC":
        lo = l16()

        def fn(c):
            c.p = _mul_ss(c.xt >> 16, lo.rd(c))
        return fn
    if key == "MPY_ACC_T_LOC":
        lo = l16()

        def fn(c):
            v = _mul_ss(c.xt >> 16, lo.rd(c))
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MPY_ACC_LOC_IMM":
        lo = l16()

        def fn(c):
            t = lo.rd(c)
            c.xt = (c.xt & M16) | (t << 16)
            v = _mul_ss(t, C)
            c.acc = v
            nz32(c, v)
        return fn
    if key == "MPY_P_LOC_IMM":
        lo = l16()

        def fn(c):
            c.p = _mul_ss(lo.rd(c), C)
        return fn
    if key in ("MPYA_P_T_LOC", "MPYS"):
        lo = l16()

        def fn(c):
            b = lo.rd(c)
            if key == "MPYS":
                acc_sub(c, pshift(c))
            else:
                acc_add(c, pshift(c))
            c.p = _mul_ss(c.xt >> 16, b)
        return fn
    if key == "MPYA_P_LOC_IMM":
        lo = l16()

        def fn(c):
            t = lo.rd(c)
            acc_add(c, pshift(c))
            c.xt = (c.xt & M16) | (t << 16)
            c.p = _mul_ss(t, C)
        return fn
    if key in ("MPYB_P", "MPYB_ACC"):
        def fn(c):
            v = (s16(c.xt >> 16) * C) & M32
            if key == "MPYB_P":
                c.p = v
            else:
                c.acc = v
                nz32(c, v)
        return fn
    if key in ("MPYU_P", "MPYU_ACC"):
        lo = l16()

        def fn(c):
            v = ((c.xt >> 16) * lo.rd(c)) & M32
            if key == "MPYU_P":
                c.p = v
            else:
                c.acc = v
                nz32(c, v)
        return fn
    if key in ("MPYXU_P", "MPYXU_ACC"):
        lo = l16()

        def fn(c):
            v = (s16(c.xt >> 16) * lo.rd(c)) & M32
            if key == "MPYXU_P":
                c.p = v
            else:
                c.acc = v
                nz32(c, v)
        return fn
    if key in ("MOVA", "MOVS", "MOVP"):
        lo = l16()

        def fn(c):
            t = lo.rd(c)
            ps = pshift(c)
            c.xt = (c.xt & M16) | (t << 16)
            if key == "MOVA":
                acc_add(c, ps)
            elif key == "MOVS":
                acc_sub(c, ps)
            else:
                c.acc = ps
                nz32(c, ps)
        return fn
    if key == "MOVAD":
        lo = l16()

        def fn(c):
            a = lo.ea(c)
            t = c.rd16(a)
            c.xt = (c.xt & M16) | (t << 16)
            c.wr16((a + 1) & M22, t)
            acc_add(c, pshift(c))
        return fn
    if key in ("IMPYL_P", "IMPYL_ACC", "QMPYL_P", "QMPYL_ACC", "QMPYUL", "QMPYXUL", "IMPYXUL", "QMPYAL", "QMPYSL",
               "IMPYAL", "IMPYSL"):
        lo = l32()

        def fn(c):
            b = lo.rd(c)
            xt = c.xt
            if key in ("QMPYUL",):
                prod = (xt * b)
            elif key in ("QMPYXUL", "IMPYXUL"):
                prod = (s32(xt) * b) & 0xFFFFFFFFFFFFFFFF
            elif key == "IMPYAL":
                prod = (s32(xt) * s32(b)) & 0xFFFFFFFFFFFFFFFF
            else:
                prod = _q64(xt, b)
            hi, lo32 = prod >> 32, prod & M32
            if key == "IMPYL_P" or key == "IMPYXUL":
                c.p = lo32
            elif key == "IMPYL_ACC":
                c.acc = lo32
                nz32(c, lo32)
            elif key in ("QMPYL_P", "QMPYUL", "QMPYXUL"):
                c.p = hi
            elif key == "QMPYL_ACC":
                c.acc = hi
                nz32(c, hi)
            elif key == "QMPYAL":
                acc_add(c, pshift(c))
                c.p = hi
            elif key == "QMPYSL":
                acc_sub(c, pshift(c))
                c.p = hi
            elif key == "IMPYAL":
                r = c.acc + c.p
                c.c = r >> 32
                r &= M32
                if c.c:
                    ovc_count(c, 1)
                c.acc = r
                nz32(c, r)
                c.p = _p_from_temp(c, prod & ((1 << 38) - 1))
            else:   # IMPYSL
                acc_sub(c, c.p)
                c.p = _p_from_temp(c, prod & ((1 << 38) - 1))
        return fn
    if key in ("MAC_XAR7", "MAC_PMA", "XMAC", "XMACD"):
        lo = l16()
        post = key == "MAC_XAR7" and not f["n"]

        def fn(c):
            acc_add(c, pshift(c))
            a = lo.ea(c) if lo.ea is not None else None
            t = c.rd16(a) if a is not None else lo.rd(c)
            c.xt = (c.xt & M16) | (t << 16)
            if key == "MAC_XAR7":
                pa = (c.xar[7] + c.rpt_i) & M22 if post else c.xar[7] & M22
            elif key == "MAC_PMA":
                pa = (C + c.rpt_i) & M22
            else:
                pa = (0x3F0000 | C) + c.rpt_i
            c.p = _mul_ss(t, c.rd16(pa))
            if key == "XMACD" and a is not None:
                c.wr16((a + 1) & M22, t)
            if post and c.rpt_last:
                c.xar[7] = (c.xar[7] + c.rpt_i + 1) & M32
        return fn
    if key in ("IMACL", "QMACL", "DMAC"):
        lo = l32()
        post = not f["n"]

        def fn(c):
            b = lo.rd(c)
            pa = (c.xar[7] + 2 * c.rpt_i) & M22 if post else c.xar[7] & M22
            x = c.rd32(pa)
            if key == "IMACL":
                r = c.acc + c.p
                c.c = r >> 32
                c.acc = r & M32
                if c.c:
                    ovc_count(c, 1)
                nz32(c, c.acc)
                c.p = _p_from_temp(c, (s32(x) * s32(b)) & ((1 << 38) - 1))
            elif key == "QMACL":
                acc_add(c, pshift(c))
                c.p = _q64(x, b) >> 32
            else:   # DMAC ACC:P: ACC += hi*hi, P += lo*lo (16x16 signed pairs)
                hi = s16(x >> 16) * s16(b >> 16)
                low = s16(x & M16) * s16(b & M16)
                c.acc = (c.acc + (hi >> (c.pm - 1) if c.pm > 1 else hi)) & M32
                c.p = (c.p + (low >> (c.pm - 1) if c.pm > 1 else low)) & M32
            if post and c.rpt_last:
                c.xar[7] = (c.xar[7] + 2 * (c.rpt_i + 1)) & M32
        return fn
    if key in ("SQRA", "SQRS"):
        lo = l16()

        def fn(c):
            t = lo.rd(c)
            ps = pshift(c)
            if key == "SQRA":
                acc_add(c, ps)
            else:
                acc_sub(c, ps)
            c.xt = (c.xt & M16) | (t << 16)
            c.p = _mul_ss(t, t)
        return fn
    # ------------------------------------------------------------ bits
    if key == "TBIT_IMM":
        lo, b = l16(), f["B"]

        def fn(c):
            c.tc = (lo.rd(c) >> b) & 1
        return fn
    if key == "TBIT_T":
        lo = l16()

        def fn(c):
            c.tc = (lo.rd(c) >> (15 - ((c.xt >> 16) & 15))) & 1
        return fn
    if key in ("TSET", "TCLR"):
        lo, b = l16(), f["B"]

        def fn(c):
            def m(v):
                c.tc = (v >> b) & 1
                r = v | (1 << b) if key == "TSET" else v & ~(1 << b) & M16
                nz16(c, r)
                return r
            lo.rmw(c, m)
        return fn
    # ------------------------------------------------------------ address registers, stack
    if key == "ADDB_XAR":
        n = f["n"]

        def fn(c):
            c.xar[n] = (c.xar[n] + C) & M32
        return fn
    if key == "SUBB_XAR":
        n = f["n"]

        def fn(c):
            c.xar[n] = (c.xar[n] - C) & M32
        return fn
    if key == "ADDB_SP":
        def fn(c):
            c.sp = (c.sp + C) & M16
        return fn
    if key == "SUBB_SP":
        def fn(c):
            c.sp = (c.sp - C) & M16
        return fn
    if key in ("ADRK", "SBRK"):
        def fn(c):
            r = c.arp
            c.xar[r] = (c.xar[r] + (C if key == "ADRK" else -C)) & M32
        return fn
    if key == "ASP":
        def fn(c):
            if c.sp & 1:
                c.sp = (c.sp + 1) & M16
                c.spa = 1
            else:
                c.spa = 0
        return fn
    if key == "NASP":
        def fn(c):
            if c.spa:
                c.sp = (c.sp - 1) & M16
                c.spa = 0
        return fn
    if key == "PUSH_LOC":
        lo = l16()

        def fn(c):
            _push16(c, lo.rd(c))
        return fn
    if key == "POP_LOC":
        lo = l16()

        def fn(c):
            _store16(lo, c, _pop16(c))
        return fn
    if key.startswith("PUSH_AR") or key.startswith("POP_AR"):
        pairs = {"AR1_AR0": (1, 0), "AR3_AR2": (3, 2), "AR5_AR4": (5, 4), "AR1H_AR0H": (1, 0)}
        hi, lo_ = pairs[key.split("_", 1)[1]]
        high = "H_" in key

        def fn(c):
            x = c.xar
            if key.startswith("PUSH"):
                if high:
                    v = ((x[hi] >> 16) << 16) | (x[lo_] >> 16)
                else:
                    v = ((x[hi] & M16) << 16) | (x[lo_] & M16)
                _push32(c, v)
            else:
                v = _pop32(c)
                if high:
                    x[hi] = (x[hi] & M16) | (v & 0xFFFF0000)
                    x[lo_] = (x[lo_] & M16) | ((v & M16) << 16)
                else:
                    x[hi] = (x[hi] & 0xFFFF0000) | (v >> 16)
                    x[lo_] = (x[lo_] & 0xFFFF0000) | (v & M16)
        return fn
    if key in ("PUSH_DP", "POP_DP"):
        def fn(c):
            if key == "PUSH_DP":
                _push16(c, c.dp)
            else:
                c.dp = _pop16(c)
        return fn
    if key in ("PUSH_ST0", "POP_ST0", "PUSH_ST1", "POP_ST1", "PUSH_IFR", "POP_IFR", "PUSH_DBGIER", "POP_DBGIER",
               "PUSH_DP_ST1", "POP_DP_ST1", "PUSH_T_ST0", "POP_T_ST0", "PUSH_P", "POP_P", "PUSH_RPC", "POP_RPC"):
        what = key.split("_", 1)[1]

        def fn(c):
            if key.startswith("PUSH"):
                if what == "ST0":
                    _push16(c, c.st0)
                elif what == "ST1":
                    _push16(c, c.st1)
                elif what == "IFR":
                    _push16(c, c.ifr)
                elif what == "DBGIER":
                    _push16(c, c.dbgier)
                elif what == "DP_ST1":
                    _push32(c, (c.dp << 16) | c.st1)
                elif what == "T_ST0":
                    _push32(c, (c.xt & 0xFFFF0000) | c.st0)
                elif what == "P":
                    _push32(c, c.p)
                else:   # RPC, two 16-bit pushes
                    _push16(c, c.rpc & M16)
                    _push16(c, c.rpc >> 16)
            else:
                if what == "ST0":
                    c.st0 = _pop16(c)
                elif what == "ST1":
                    c.st1 = _pop16(c)
                elif what == "IFR":
                    c.ifr = _pop16(c)
                elif what == "DBGIER":
                    c.dbgier = _pop16(c)
                elif what == "DP_ST1":
                    v = _pop32(c)
                    c.st1 = v & M16
                    c.dp = v >> 16
                elif what == "T_ST0":
                    v = _pop32(c)
                    c.st0 = v & M16
                    c.xt = (c.xt & M16) | (v & 0xFFFF0000)
                elif what == "P":
                    c.p = _pop32(c)
                else:
                    h = _pop16(c)
                    c.rpc = ((h << 16) | _pop16(c)) & M22
        return fn
    if key in ("PREAD", "PWRITE"):
        lo = l16()

        def fn(c):
            pa = (c.xar[7] + c.rpt_i) & M22
            if key == "PREAD":
                _store16(lo, c, c.rd16(pa))
            else:
                c.wr16(pa, lo.rd(c))
        return fn
    if key == "XPREAD_PMA":
        lo = l16()

        def fn(c):
            _store16(lo, c, c.rd16((0x3F0000 | C) + c.rpt_i))
        return fn
    if key in ("XPREAD_AL", "XPWRITE_AL"):
        lo = l16()

        def fn(c):
            pa = (0x3F0000 | (c.acc & M16)) + c.rpt_i
            if key == "XPREAD_AL":
                _store16(lo, c, c.rd16(pa))
            else:
                c.wr16(pa, lo.rd(c))
        return fn
    # ------------------------------------------------------------ status
    if key in ("SETC", "CLRC"):
        val = 1 if key == "SETC" else 0
        names = [n for i, n in enumerate(("sxm", "ovm", "tc", "c", "intm", "dbgm", "page0", "vmap")) if C >> i & 1]

        def fn(c):
            for n in names:
                setattr(c, n, val)
            if "page0" in names and val:
                raise Unimplemented("PAGE0 stack addressing mode")
        return fn
    if key in ("CLRC_AMODE", "SETC_OBJMODE", "CLRC_OBJMODE", "SETC_M0M1MAP", "CLRC_M0M1MAP", "SETC_XF", "CLRC_XF",
               "CLRC_OVC"):
        def fn(c):
            if key == "CLRC_AMODE":
                c.amode = 0
            elif key == "SETC_OBJMODE":
                c.objmode = 1
            elif key == "CLRC_OBJMODE":
                c.objmode = 0
            elif key == "SETC_M0M1MAP":
                c.m0m1map = 1
            elif key == "CLRC_M0M1MAP":
                c.m0m1map = 0
            elif key == "SETC_XF":
                c.xf = 1
            elif key == "CLRC_XF":
                c.xf = 0
            else:
                c.ovc = 0
        return fn
    if key == "SPM":
        pm = f["S"]

        def fn(c):
            c.pm = pm
        return fn
    if key == "EALLOW":
        def fn(c):
            c.eallow = 1
        return fn
    if key == "EDIS":
        def fn(c):
            c.eallow = 0
        return fn
    if key in ("AND_IER", "OR_IER", "AND_IFR", "OR_IFR"):
        def fn(c):
            if key == "AND_IER":
                c.ier &= C
            elif key == "OR_IER":
                c.ier |= C
            elif key == "AND_IFR":
                c.ifr &= C
            else:
                c.ifr |= C
        return fn
    if key == "IACK":
        def fn(c):
            if c.iack_hook is not None:
                c.iack_hook(C)
        return fn
    if key in ("NOP", "ESTOP0", "ESTOP1", "ABORTI", "ZAPA", "LPADDR"):
        lo = l16() if key == "NOP" and L else None

        def fn(c):
            if lo is not None and lo.ea is not None:
                lo.ea(c)
            elif key == "ZAPA":
                c.acc = c.p = 0
                c.ovc = 0
                c.n, c.z = 0, 1
            elif key == "LPADDR":
                c.amode = 1
                raise Unimplemented("LPADDR (C2xLP addressing)")
        return fn
    # ------------------------------------------------------------ control flow
    if key == "B" or key == "BF":
        tgt = (addr + s16(C)) & M22
        cond = cond_fn(f["K"])
        if f["K"] == 15:
            def fn(c):
                c.pc = tgt
        else:
            def fn(c):
                if cond(c):
                    c.pc = tgt
        return fn
    if key == "SB" or key.startswith("SBF_"):
        tgt = (addr + (C - 256 if C & 0x80 else C)) & M22
        k = f["K"] if key == "SB" else {"SBF_EQ": 1, "SBF_NEQ": 0, "SBF_TC": 13, "SBF_NTC": 12}[key]
        cond = cond_fn(k)
        if k == 15:
            def fn(c):
                c.pc = tgt
        else:
            def fn(c):
                if cond(c):
                    c.pc = tgt
        return fn
    if key == "BANZ":
        n = f["n"]
        tgt = (addr + s16(C)) & M22

        def fn(c):
            x = c.xar
            v = x[n] & M16
            x[n] = (x[n] & 0xFFFF0000) | ((v - 1) & M16)
            if v:
                c.pc = tgt
        return fn
    if key == "BAR":
        n, m, ne = f["n"], f["m"], f["B"]
        tgt = (addr + s16(C)) & M22

        def fn(c):
            eq = (c.xar[n] & M16) == (c.xar[m] & M16)
            if eq != bool(ne):
                c.pc = tgt
        return fn
    if key == "LB":
        def fn(c):
            c.pc = C
        return fn
    if key == "LB_XAR7":
        def fn(c):
            c.pc = c.xar[7] & M22
        return fn
    if key == "LCR":
        def fn(c):
            _push16(c, c.rpc & M16)
            _push16(c, c.rpc >> 16)
            c.rpc = nxt
            c.pc = C
        return fn
    if key == "LCR_XARN":
        n = f["n"]

        def fn(c):
            _push16(c, c.rpc & M16)
            _push16(c, c.rpc >> 16)
            c.rpc = nxt
            c.pc = c.xar[n] & M22
        return fn
    if key == "LRETR":
        def fn(c):
            c.pc = c.rpc
            h = _pop16(c)
            c.rpc = ((h << 16) | _pop16(c)) & M22
        return fn
    if key in ("LC", "LC_XAR7"):
        def fn(c):
            _push16(c, nxt & M16)
            _push16(c, nxt >> 16)
            c.pc = C if key == "LC" else c.xar[7] & M22
        return fn
    if key in ("LRET", "LRETE"):
        def fn(c):
            h = _pop16(c)
            c.pc = ((h << 16) | _pop16(c)) & M22
            if key == "LRETE":
                c.intm = 0
        return fn
    if key == "FFC":
        def fn(c):
            c.xar[7] = nxt
            c.pc = C
        return fn
    if key in ("XB_AL", "XCALL_AL"):
        def fn(c):
            if key == "XCALL_AL":
                _push16(c, nxt & M16)
                _push16(c, nxt >> 16)
            c.pc = 0x3F0000 | (c.acc & M16)
        return fn
    if key in ("XB_PMA_COND", "XCALL_PMA_COND"):
        cond = cond_fn(f["K"])
        tgt = 0x3F0000 | C

        def fn(c):
            if cond(c):
                if key == "XCALL_PMA_COND":
                    _push16(c, nxt & M16)
                    _push16(c, nxt >> 16)
                c.pc = tgt
        return fn
    if key == "XRETC":
        cond = cond_fn(f["K"])

        def fn(c):
            if cond(c):
                h = _pop16(c)
                c.pc = ((h << 16) | _pop16(c)) & M22
        return fn
    if key in ("LOOPZ", "LOOPNZ"):
        lo = l16()

        def fn(c):
            a = lo.ea(c) if lo.ea else None
            v = c.rd16(a) if a is not None else lo.rd(c)
            r = v & C
            c.loop = 0
            nz16(c, r)
            if (key == "LOOPZ" and r != 0) or (key == "LOOPNZ" and r == 0):
                c.loop = 1
                c.pc = addr   # keep polling
        return fn
    if key == "IRET":
        def fn(c):
            c.iret()
        return fn
    if key in ("TRAP", "INTR", "INTR_NMI", "INTR_EMU"):
        vec = {"TRAP": C, "INTR": (C + 1) if C < 14 else C + 1, "INTR_NMI": 18, "INTR_EMU": 0}[key]

        def fn(c):
            c.trap(vec, nxt, key == "INTR")
        return fn
    if key == "IDLE":
        def fn(c):
            c.idle()
        return fn
    if key in ("RPT_IMM", "RPT_LOC"):
        lo = l16() if key == "RPT_LOC" else None

        def fn(c):
            c.rptc = lo.rd(c) if lo is not None else C
            c.rpt_pending = True
        return fn
    if key in ("NORM_XARN", "NORM_ACC_IND"):
        n = f.get("n")
        inc = f.get("B", 0)

        def fn(c):
            a = c.acc
            if a == 0:
                c.tc = 0
            elif (a >> 31) == ((a >> 30) & 1):
                a = (a << 1) & M32
                c.acc = a
                c.tc = 0
                if n is not None:
                    c.xar[n] = (c.xar[n] + (1 if inc else -1)) & M32
            else:
                c.tc = 1
            nz32(c, c.acc)
        return fn
    if key in ("IN", "OUT", "UOUT", "CMPR"):
        def fn(c):
            raise Unimplemented(key)
        return fn
    raise Unimplemented(key)


def _movb(key, L, A):
    """Byte moves: *+XARn[AR0/AR1/3bit] address bytes (word XARn + off/2, the
    byte selected by the offset's parity); other modes the low byte."""
    lo = loc(L, 16)
    get, put = _ax_get(A), _ax_set(A)
    mode = L >> 6
    sub = (L >> 3) & 7
    n = L & 7
    indexed = mode == 3 or (mode == 2 and sub in (2, 3))
    if indexed:
        if mode == 3:
            k = (L >> 3) & 7

            def where(c):
                return (c.xar[n] + (k >> 1)) & M22, k & 1
        else:
            r = 0 if sub == 2 else 1

            def where(c):
                off = c.xar[r] & M16
                return (c.xar[n] + (off >> 1)) & M22, off & 1
    else:
        where = None

    def read_byte(c):
        if where is not None:
            a, hi = where(c)
            v = c.rd16(a)
            return (v >> 8) if hi else (v & 0xFF)
        return lo.rd(c) & 0xFF

    def write_byte(c, b):
        if where is not None:
            a, hi = where(c)
            v = c.rd16(a)
            c.wr16(a, (v & 0xFF) | (b << 8) if hi else (v & 0xFF00) | b)
        elif lo.reg:
            v = lo.rd(c)
            lo.wr(c, (v & 0xFF00) | b)
        else:
            a = lo.ea(c)
            v = c.rd16(a)
            c.wr16(a, (v & 0xFF00) | b)

    if key == "MOVB_AX_LSB":
        def fn(c):
            v = read_byte(c)
            put(c, v)
            nz16(c, v)
    elif key == "MOVB_AX_MSB":
        def fn(c):
            v = (get(c) & 0xFF) | (read_byte(c) << 8)
            put(c, v)
            nz16(c, v)
    elif key == "MOVB_LOC_AX_LSB":
        def fn(c):
            write_byte(c, get(c) & 0xFF)
    else:
        def fn(c):
            write_byte(c, get(c) >> 8)
    return fn
