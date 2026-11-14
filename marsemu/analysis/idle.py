"""Find the FreeRTOS idle loop so it can be treated like WFI.

Most Marstek firmwares run FreeRTOS without a WFI in the idle task: when every
task is blocked the CPU spins in ``prvIdleTask``.  Emulating that spin is most
of the work for a mostly idle firmware, so the emulator fast-forwards to the
next interrupt when execution reaches the idle loop, exactly as it does for WFI.

The idle task is found from ``vTaskStartScheduler``: the code that loads the
task name ``"IDLE"`` also loads the entry point ``prvIdleTask``; its first
backward loop is the idle loop.  The machine only sleeps there after a whole
iteration ran without an exception (no yield to another idle-priority task, no
interrupt), so skipping further iterations until the next interrupt is
indistinguishable from running them.
"""
from __future__ import annotations

import struct
from typing import Optional

try:
    import capstone
except ImportError:  # pragma: no cover
    capstone = None


def _thumb():
    md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
    md.detail = False
    return md


def _literal_refs(image: bytes, base: int, value: int):
    """Addresses of 4-byte aligned literal words equal to ``value``."""
    pat = struct.pack("<I", value)
    i = image.find(pat)
    while i >= 0:
        if i % 4 == 0:
            yield base + i
        i = image.find(pat, i + 1)


def find_idle_loop(image: bytes, base: int) -> Optional[int]:
    """Return the address of the idle loop's head, or None."""
    if capstone is None:
        return None
    md = _thumb()
    md.skipdata = True
    end = base + len(image)
    names = []
    s = image.find(b"IDLE\x00")
    while s >= 0:
        names.append(base + s)
        s = image.find(b"IDLE\x00", s + 1)
    if not names:
        return None
    targets = set(names)
    for n in names:
        targets.update(_literal_refs(image, base, n))
    insns = list(md.disasm(image, base))
    for k, ins in enumerate(insns):
        t = None
        if ins.mnemonic in ("ldr", "ldr.w") and "[pc" in ins.op_str:
            t = _ldr_target(ins)
        elif (ins.mnemonic in ("adr", "adr.w") or ins.mnemonic in ("add", "addw") and ", pc," in ins.op_str) \
                and "#" in ins.op_str:
            try:
                t = ((ins.address + 4) & ~3) + int(ins.op_str.split("#")[-1], 0)
            except ValueError:
                t = None
        if t not in targets:
            continue
        for near in insns[max(0, k - 6): k + 6]:
            if near.mnemonic in ("ldr", "ldr.w") and "[pc" in near.op_str and near.op_str.startswith("r0,"):
                lt = _ldr_target(near)
                if base <= lt < end - 4:
                    v = struct.unpack_from("<I", image, lt - base)[0]
                    if v & 1 and base <= v < end:
                        head = _plain_loop(image, base, v & ~1, _thumb())
                        if head:
                            return head
    return None


def _ldr_target(ins) -> Optional[int]:
    try:
        off = int(ins.op_str.split("#")[-1].rstrip("]"), 0)
    except ValueError:
        off = 0
    return ((ins.address + 4) & ~3) + off


def _plain_loop(image, base, entry, md) -> Optional[int]:
    """Head of the first backward loop in the function if it only calls and branches."""
    insns = list(md.disasm(image[entry - base: entry - base + 160], entry))
    for n, ins in enumerate(insns):
        if ins.mnemonic in ("b", "b.w") and ins.op_str.startswith("#"):
            tgt = int(ins.op_str[1:], 0)
            if entry <= tgt <= ins.address:
                body = [i for i in insns if tgt <= i.address <= ins.address]
                if len(body) <= 24 and all(_harmless(i) for i in body):
                    return tgt
                return None
        if ins.mnemonic in ("bx", "pop") and "pc" in ins.op_str or ins.mnemonic == "bx":
            return None
    return None


def _harmless(ins) -> bool:
    return not ins.mnemonic.startswith(("svc", "bkpt", "cps", "msr", "udf", "pop", "bx"))
