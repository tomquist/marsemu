"""Exact instruction count of the last ``emu_start``.

Unicorn implements ``emu_start(count=N)`` with an internal counter but does
not expose it, and we need it whenever emulation stops early (interrupt
preemption, exits, WFI) to keep virtual time exact.  The counter lives next
to the requested count inside ``struct uc_struct``; its offset depends on the
Unicorn build, so it is discovered once at runtime with a probe program and
verified before use.
"""
from __future__ import annotations

import ctypes
import logging
import struct
from typing import Optional

from unicorn import UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, Uc
from unicorn import arm_const as A

log = logging.getLogger("emu.ucount")
_OFFSET: Optional[int] = None
_PROBED = False


def _probe() -> Optional[int]:
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0x08000000, 0x1000)
    # 20x "adds r0, #1" followed by a branch back to the start
    off = -(40 + 4) // 2
    uc.mem_write(0x08000000, bytes.fromhex("0130" * 20) + struct.pack("<H", 0xE000 | (off & 0x7FF)))
    base = uc._uch.value

    def snap():
        return ctypes.string_at(base, 0x4000)

    def find(val, buf):
        return {i for i in range(0, len(buf) - 7, 4) if struct.unpack_from("<Q", buf, i)[0] == val}

    uc.emu_start(0x08000001, 1, count=7771)
    a = find(7771, snap())
    uc.emu_start(0x08000001, 1, count=123457)
    cands = a & find(123457, snap())
    uc.ctl_exits_enabled(True)
    uc.ctl_set_exits([0x08000010])
    uc.ctl_flush_tb()
    for c in sorted(cands):
        for delta in (-8, 8):
            uc.reg_write(A.UC_ARM_REG_R0, 0)
            uc.emu_start(0x08000001, 0, count=99991)
            got = struct.unpack("<Q", ctypes.string_at(base + c + delta, 8))[0]
            want = uc.reg_read(A.UC_ARM_REG_R0)  # one add per instruction before the exit
            if got == want == 8:
                return c + delta
    return None


def counter_offset() -> Optional[int]:
    global _OFFSET, _PROBED
    if not _PROBED:
        _PROBED = True
        try:
            _OFFSET = _probe()
        except Exception as e:  # pragma: no cover
            log.warning("instruction counter probe failed: %s", e)
        if _OFFSET is None:
            log.warning("could not locate unicorn's instruction counter; early stops are estimated")
    return _OFFSET


class Counter:
    """Reads the executed-instruction count of the last emu_start of ``uc``."""

    def __init__(self, uc):
        off = counter_offset()
        self.ok = off is not None
        self._v = ctypes.c_uint64.from_address(uc._uch.value + off) if self.ok else None

    def value(self) -> Optional[int]:
        return self._v.value if self.ok else None
