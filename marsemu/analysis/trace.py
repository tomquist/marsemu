"""Tracing helpers: MMIO access log with register names, basic-block coverage."""
from __future__ import annotations

from unicorn import UC_HOOK_BLOCK


class MmioTracer:
    """Logs every peripheral register access as ``t pc kind periph.reg size value``.

    Note: Unicorn reports the PC of the accessing instruction for MMIO, so
    the PC column is exact."""

    def __init__(self, emu, path: str, skip=("PPB",)):
        self.emu = emu
        self.f = open(path, "w")
        self.skip = set(skip)
        emu.machine.bus.tracers.append(self.trace)

    def trace(self, kind, p, off, size, value):
        if p.name in self.skip:
            return
        m = self.emu.machine
        self.f.write(f"{m.clock.seconds:.6f} {m.pc:08x} {kind} {p.name}.{p.reg_name(off)} {size} {value:08x}\n")


class Coverage:
    """Records the start address of every executed basic block.

    This adds a Python callback per block and slows emulation down noticeably;
    use it for focused runs.  The output (one hex address per line) can be
    loaded into Ghidra/IDA with the usual coverage plugins (e.g. Lighthouse's
    "address list" format)."""

    def __init__(self, emu, lo: int = 0x08000000, hi: int = 0x08100000):
        self.blocks = {}
        emu.machine.uc.hook_add(UC_HOOK_BLOCK, self._hook, begin=lo, end=hi)

    def _hook(self, uc, addr, size, _ud):
        self.blocks[addr] = size

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            for a in sorted(self.blocks):
                f.write(f"0x{a:08x} {self.blocks[a]}\n")
