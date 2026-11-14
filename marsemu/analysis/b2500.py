"""Which ADC sample is which signal in a B2500 (hardware v2) image.

The two ADCs fill DMA buffers in a fixed channel sequence, but which buffer
slot carries which measurement differs between builds (HMA V232 and HMJ-2
V118 swap PV2 and the output currents between the ADCs).  The conversions
themselves are the same everywhere, so they identify the slots:

* voltages: ``x * 25 + 32`` (``add r, x, x, lsl #3`` / ``add r, r, x, lsl #4``
  / ``adds #0x20``), stored at +0x32 / +0x7a (PV1/PV2) of the PV block or
  +0x16 / +0x4e (output 1/2) of the output block (V119: PV2 at +0x76, its
  current at +0x7a, output 2 at +0x44)
* PV currents: 128 samples summed, then ``sum >> 4`` (8 mA/count), in most
  builds ``+ (>> 5)`` on top (8.25 mA/count), stored at +0x36 / +0x7e (some
  builds add part of the offset to the base register first)
* output currents: summed in the DMA interrupt next to a sample history
  (``strh x, [hist, n, lsl #1]``), output 1 first

:func:`find_adc_slots` returns ``{(buffer_address, slot): signal}`` with
signals ``pv_v0 pv_v1 pv_i0 pv_i1 out_v0 out_v1 out_i0 out_i1``; pass a
dict as ``scales`` to get the PV current scales (mA per count) as well.
"""
from __future__ import annotations

import re
import struct
from typing import Dict, Optional, Tuple

PV_V = {0x32: "pv_v0", 0x7A: "pv_v1", 0x76: "pv_v1"}      # V119 keeps side 2 at +0x76/+0x7a/+0x44
OUT_V = {0x16: "out_v0", 0x4E: "out_v1", 0x44: "out_v1"}
PV_I = {0x36: "pv_i0", 0x7E: "pv_i1", 0x7A: "pv_i1"}

_IMM = re.compile(r"#(-?0x[0-9a-f]+|-?\d+)")


def _imm(s: str) -> Optional[int]:
    m = _IMM.search(s)
    return int(m.group(1), 0) if m else None


def find_adc_slots(image: bytes, base: int = 0x08000000,
                   buffers: Optional[Tuple[int, ...]] = None,
                   scales: Optional[Dict[str, float]] = None) -> Dict[Tuple[int, int], str]:
    """``buffers``: the DMA destination addresses, when known; other matches are dropped.
    ``scales``: filled with ``{"pv_i0": mA_per_count, ...}`` for the PV currents found."""
    scales = {} if scales is None else scales
    from capstone import CS_ARCH_ARM, CS_MODE_MCLASS, CS_MODE_THUMB, Cs

    md = Cs(CS_ARCH_ARM, CS_MODE_THUMB | CS_MODE_MCLASS)
    md.skipdata = True
    ins = list(md.disasm(image, base))
    regs: Dict[str, int] = {}
    found: Dict[Tuple[int, int], str] = {}
    out_i_order = []

    def lit(x) -> Optional[int]:
        m = re.match(r"(\w+), \[pc, #(0x[0-9a-f]+|\d+)\]", x.op_str)
        if not m:
            return None
        t = ((x.address + 4) & ~3) + int(m.group(2), 0) - base
        if 0 <= t <= len(image) - 4:
            return struct.unpack_from("<I", image, t)[0]
        return None

    for k, x in enumerate(ins):
        mn, ops = x.mnemonic, x.op_str
        if mn.startswith("push") or mn in ("bx", "b") or (mn.startswith("pop") and "pc" in ops):
            regs.clear()
            continue
        dst = ops.split(",")[0].strip()
        if mn.startswith("ldr") and mn not in ("ldrh", "ldrh.w", "ldrb", "ldrb.w", "ldrsh", "ldrsh.w") and "[pc" in ops:
            v = lit(x)
            if v is not None:
                regs[dst] = v
            continue
        if mn in ("adds", "add", "add.w", "adds.w") and ops.count(",") == 1 and dst in regs:
            imm = _imm(ops)
            if imm is not None:
                regs[dst] = (regs[dst] + imm) & 0xFFFFFFFF
            continue
        if mn in ("subs", "sub", "sub.w") and ops.count(",") == 1 and dst in regs:
            imm = _imm(ops)
            if imm is not None:
                regs[dst] = (regs[dst] - imm) & 0xFFFFFFFF
            continue
        if mn.startswith("ldrh"):
            m = re.match(r"(\w+), \[(\w+)(?:, #(0x[0-9a-f]+|\d+))?\]$", ops)
            if m and m.group(2) in regs and 0x20000000 <= regs[m.group(2)] < 0x20020000:
                buf = regs[m.group(2)]
                off = int(m.group(3), 0) if m.group(3) else 0
                scale: Dict[str, float] = {}
                sig = _classify(ins, k, m.group(1), scale)
                key = (buf, off // 2)
                if buffers is not None and buf not in buffers:
                    sig = None
                if sig == "out_i":
                    if key not in out_i_order:
                        out_i_order.append(key)
                elif sig and key not in found:
                    found[key] = sig
                    if sig in scale:
                        scales[sig] = scale[sig]
            if dst in regs:
                del regs[dst]
            continue
        if dst in regs and not mn.startswith(("str", "cmp", "tst", "cb", "b")):
            del regs[dst]
    for n, key in enumerate(out_i_order[:2]):
        found.setdefault(key, f"out_i{n}")
    return found


def board_family(image: bytes, base: int = 0x08000000) -> Optional[str]:
    """Which B2500 v2 board a build is for, from its ADC layout: ``"HMJ"`` (PV 2
    is sampled by the same ADC as PV 1) or ``"HMA"`` (by the other one), or None
    when the layout isn't found.  The image itself can't say more: HMA and HMJ
    builds carry the same device-type table, and the type of a unit (HMA-1,
    HMJ-2, ...) is factory data."""
    try:
        slots = find_adc_slots(image, base)
    except Exception:
        return None
    where = {sig: key[0] for key, sig in slots.items()}
    if "pv_v0" not in where or "pv_v1" not in where:
        return None
    return "HMJ" if where["pv_v0"] == where["pv_v1"] else "HMA"


def _classify(ins, k: int, reg: str, scales: Optional[Dict[str, float]] = None) -> Optional[str]:
    """What the half-word loaded into ``reg`` at ins[k] becomes (PV current
    scales go into ``scales``)."""
    window = ins[k + 1:k + 24]
    times9 = None
    for n, y in enumerate(window):
        ops = y.op_str.replace(" ", "")
        if y.mnemonic == "bl" and times9 is None:
            # passed through a filter function: follow its result
            if reg in ("r0", "r1", "r2", "r3"):
                reg = "r0"
            continue
        if y.mnemonic in ("mov", "movs") and times9 is None and ops.endswith(f",{reg}"):
            reg = ops.split(",")[0]
            continue
        # x*25+32: add rA, x, x, lsl #3 ; add rB, rA, x, lsl #4 ; adds rB, #0x20 ; str rB, [rP, #off]
        if times9 is None and y.mnemonic.startswith("add") and ops.endswith(f",{reg},{reg},lsl#3"):
            times9 = ops.split(",")[0]
            continue
        if times9 and y.mnemonic.startswith(("str", "strh")):
            off = _imm(y.op_str)
            if off in PV_V:
                return PV_V[off]
            if off in OUT_V:
                return OUT_V[off]
        # history + running sum in the DMA interrupt: strh x, [hist, n, lsl #1]
        if n < 5 and y.mnemonic.startswith("strh") and re.match(rf"{reg},\[\w+,\w+,lsl#1\]", ops):
            return "out_i"
        # 128-sample PV current average: sum >> 4 (8 mA/count), mostly + (that >> 5)
        if y.mnemonic.startswith("lsrs") and ops.endswith(",#4"):
            after = ins[k + 1 + n:k + 1 + n + 8]
            adj: Dict[str, int] = {}
            for z in after[:6]:
                zo = z.op_str.replace(" ", "")
                if z.mnemonic in ("adds", "add", "subs", "sub") and zo.count(",") == 1:
                    imm = _imm(z.op_str)
                    if imm is not None:
                        r = zo.split(",")[0]
                        adj[r] = adj.get(r, 0) + (imm if z.mnemonic.startswith("add") else -imm)
                    continue
                if z.mnemonic.startswith(("str", "strh")):
                    mm = re.match(r"\w+,\[(\w+)(?:,#(-?0x[0-9a-f]+|-?\d+))?\]$", zo)
                    if not mm:
                        continue
                    off = (int(mm.group(2), 0) if mm.group(2) else 0) + adj.get(mm.group(1), 0)
                    if off in PV_I:
                        if scales is not None:
                            plus = any(w.op_str.replace(" ", "").endswith("lsr#5") for w in after)
                            scales[PV_I[off]] = 8.25 if plus else 8.0
                        return PV_I[off]
    return None


__all__ = ["find_adc_slots"]
