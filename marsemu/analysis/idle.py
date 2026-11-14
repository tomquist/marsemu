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


# ----------------------------------------------------------------- watchdog waits
#: IWDG base address on the N32G45x, STM32F0/F1/G4
IWDG_BASE = 0x40003000
SRAM = (0x20000000, 0x20100000)
#: what a wait loop may contain besides the call: loads, compares, extensions
_WAIT_OK = ("ldr", "ldrh", "ldrb", "ldrsh", "ldrsb", "ldr.w", "ldrh.w", "ldrb.w", "cmp", "cmp.w", "cmn", "tst",
            "tst.w", "uxth", "uxtb", "sxth", "sxtb", "bl", "nop")
_COND_BRANCH = ("beq", "bne", "bhs", "bcs", "blo", "bcc", "bhi", "bls", "bge", "blt", "bgt", "ble", "bmi", "bpl")


def _strip(m: str) -> str:
    return m[:-2] if m.endswith(".w") or m.endswith(".n") else m


def watchdog_reload_functions(image: bytes, base: int) -> list:
    """Entry points of ``IWDG_ReloadCounter``-style functions: load the IWDG
    base from a literal, write 0xAAAA to its key register, return."""
    if capstone is None:
        return []
    md = _thumb()
    found = []
    for lit in _literal_refs(image, base, IWDG_BASE):
        for start in range(lit - 16, lit - 6, 2):
            if start < base:
                continue
            ins = list(md.disasm(image[start - base: lit - base], start))
            for k in range(len(ins) - 3):
                w = ins[k:k + 4]
                ops = [_strip(i.mnemonic) for i in w]
                if ops[3] != "bx" or w[3].op_str != "lr":
                    continue
                ld = next((i for i in w[:3] if _strip(i.mnemonic) == "ldr" and "[pc" in i.op_str), None)
                mv = next((i for i in w[:3] if _strip(i.mnemonic) in ("movw", "mov", "movs")
                           and i.op_str.endswith("#0xaaaa")), None)
                st = next((i for i in w[:3] if _strip(i.mnemonic) == "str"), None)
                if not (ld and mv and st) or _ldr_target(ld) != lit:
                    continue
                ra, rb = ld.op_str.split(",")[0], mv.op_str.split(",")[0]
                if st.op_str.replace(" ", "") in (f"{rb},[{ra}]", f"{rb},[{ra},#0]"):
                    found.append(w[0].address)
            if found and found[-1] >= start:
                break
    return sorted(set(found))


def _ram_base(insns, k_head, reg: str, image: bytes, base: int) -> Optional[int]:
    """The literal a register was loaded with shortly before a loop, if any."""
    for prev in reversed(insns[max(0, k_head - 12): k_head]):
        if prev.op_str.split(",")[0] != reg:
            continue
        if _strip(prev.mnemonic) == "ldr" and "[pc" in prev.op_str:
            lt = _ldr_target(prev)
            if base <= lt <= base + len(image) - 4:
                return struct.unpack_from("<I", image, lt - base)[0]
        return None
    return None


def find_watchdog_waits(image: bytes, base: int) -> list:
    """Heads of busy-wait loops that only reload the watchdog and poll RAM.

    The typical one is a ``delay_ms()``: ``while (ticks < n) IWDG_ReloadCounter();``
    with ``ticks`` advanced by an interrupt.  Such a loop can only end after an
    interrupt, and an iteration changes nothing but the watchdog, so the CPU
    may sleep there like in WFI (see :meth:`Machine.set_idle_loop`).  Loops
    that read a peripheral are not taken: a flag a peripheral sets without an
    interrupt would otherwise wait for the next one."""
    funcs = set(watchdog_reload_functions(image, base))
    if not funcs:
        return []
    md = _thumb()
    md.skipdata = True
    insns = list(md.disasm(image, base))
    heads = []
    for k, ins in enumerate(insns):
        if _strip(ins.mnemonic) != "bl" or not ins.op_str.startswith("#") or int(ins.op_str[1:], 0) not in funcs:
            continue
        for j in range(k + 1, min(k + 8, len(insns))):
            br = insns[j]
            m = _strip(br.mnemonic)
            if m not in _COND_BRANCH and m != "b":
                continue
            if not br.op_str.startswith("#"):
                break
            tgt = int(br.op_str[1:], 0)
            if not (tgt <= ins.address and ins.address - tgt <= 16):
                break
            ks = [i for i, x in enumerate(insns) if tgt <= x.address <= br.address]
            body = [insns[i] for i in ks]
            if any(_strip(x.mnemonic) not in _WAIT_OK and x is not br for x in body):
                break
            if any(_strip(x.mnemonic) == "bl" and int(x.op_str[1:], 0) not in funcs for x in body
                   if x.op_str.startswith("#")):
                break
            ok = True
            for x in body:
                if _strip(x.mnemonic).startswith("ldr") and "[pc" not in x.op_str:
                    reg = x.op_str.split("[")[1].split(",")[0].rstrip("]").strip()
                    addr = _ram_base(insns, ks[0], reg, image, base)
                    if addr is None or not SRAM[0] <= addr < SRAM[1]:
                        ok = False
            if ok:
                heads.append(tgt)
            break
    return sorted(set(heads))
