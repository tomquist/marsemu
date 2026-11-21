"""Memory-mapped I/O dispatch.

Every peripheral is a :class:`Peripheral` that owns an address window.  The
bus maps large MMIO windows into Unicorn once and dispatches each access to
the peripheral that owns the 1 KiB page, falling back to a plain register
file (:class:`UnknownPeripheral`) so that firmware touching hardware we do
not model reads back what it wrote instead of crashing.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Callable, Dict, List, Optional

if TYPE_CHECKING:  # pragma: no cover
    from .machine import Machine

log = logging.getLogger("emu.bus")

PAGE = 0x400


class Peripheral:
    """Base class: a bank of 32-bit registers with optional side effects.

    Subclasses override :meth:`read32` / :meth:`write32` (word aligned
    offsets).  Byte and half-word accesses are merged into word accesses by
    the base class, which is what the AHB/APB bridges of these MCUs do too.
    """

    size = PAGE
    #: register offset -> name, used for traces (filled from the SVD database)
    reg_names: Dict[int, str] = {}

    def __init__(self, machine: "Machine", name: str, base: int, irqs: Optional[Dict[str, int]] = None):
        self.m = machine
        self.name = name
        self.base = base
        self.irqs = irqs or {}
        self.regs: Dict[int, int] = {}
        self.reset_values: Dict[int, int] = {}

    # -- helpers -------------------------------------------------------------
    def reset(self) -> None:
        self.regs = dict(self.reset_values)

    def r(self, off: int) -> int:
        return self.regs.get(off, 0)

    def set_bits(self, off: int, mask: int) -> None:
        self.regs[off] = self.regs.get(off, 0) | mask

    def clr_bits(self, off: int, mask: int) -> None:
        self.regs[off] = self.regs.get(off, 0) & ~mask & 0xFFFFFFFF

    def irq(self, which: str = "", level: bool = True) -> None:
        """Pend (level=True) the named interrupt line of this peripheral."""
        n = self.irqs.get(which) if which else (next(iter(self.irqs.values())) if self.irqs else None)
        if n is not None and level:
            self.m.nvic.set_pending(16 + n)

    def reg_name(self, off: int) -> str:
        return self.reg_names.get(off, f"+0x{off:03x}")

    # -- word access (override) ------------------------------------------------
    def read32(self, off: int) -> int:
        return self.regs.get(off, 0)

    def write32(self, off: int, value: int) -> None:
        self.regs[off] = value

    # -- sub-word plumbing -------------------------------------------------------
    def read(self, off: int, size: int) -> int:
        aligned = off & ~3
        val = self.read32(aligned)
        shift = (off & 3) * 8
        return (val >> shift) & ((1 << (size * 8)) - 1)

    def write(self, off: int, size: int, value: int) -> None:
        aligned = off & ~3
        if size == 4 and not (off & 3):
            self.write32(aligned, value & 0xFFFFFFFF)
            return
        shift = (off & 3) * 8
        mask = ((1 << (size * 8)) - 1) << shift
        old = self.regs.get(aligned, 0)
        self.write32(aligned, ((old & ~mask) | ((value << shift) & mask)) & 0xFFFFFFFF)

    def tick(self) -> None:  # optional per-slice hook
        pass

    def __repr__(self) -> str:
        return f"<{type(self).__name__} {self.name}@0x{self.base:08x}>"


class PollResolver:
    """Breaks busy-wait loops on registers nobody models.

    When the same register is read many times in a row without any write to
    the peripheral, the firmware is almost certainly waiting for a status
    bit.  We then return the value with all bits inverted once, which
    satisfies both "wait until set" and "wait until clear" loops, and log it
    so a proper model can be written."""

    threshold = 200

    def __init__(self):
        self.last = None
        self.count = 0
        self.resolved: set = set()

    def note_write(self):
        self.last = None
        self.count = 0

    def read(self, periph, off, value):
        if self.last == off:
            self.count += 1
            if self.count >= self.threshold:
                self.count = 0
                if off not in self.resolved:
                    self.resolved.add(off)
                    log.info("%s %s polled %d times without change: returning 0x%08x once "
                             "(unmodelled status bit?)", periph.name, periph.reg_name(off), self.threshold,
                             value ^ 0xFFFFFFFF)
                return value ^ 0xFFFFFFFF
        else:
            self.last = off
            self.count = 1
        return value


class UnknownPeripheral(Peripheral):
    """Register file for hardware we don't model.  Logs the first accesses."""

    def __init__(self, machine, name, base):
        super().__init__(machine, name, base)
        self._seen: set = set()
        self.poll = PollResolver()

    def _note(self, kind: str, off: int, value: Optional[int] = None) -> None:
        key = (kind, off)
        if key in self._seen:
            return
        self._seen.add(key)
        pc = self.m.pc
        if value is None:
            log.debug("unmodelled %s read %s (pc=0x%08x)", self.name, self.reg_name(off), pc)
        else:
            log.debug("unmodelled %s write %s = 0x%08x (pc=0x%08x)", self.name, self.reg_name(off), value, pc)

    def read32(self, off):
        self._note("r", off)
        return self.poll.read(self, off, self.regs.get(off, 0))

    def write32(self, off, value):
        self._note("w", off, value)
        self.poll.note_write()
        self.regs[off] = value


Tracer = Callable[[str, "Peripheral", int, int, int], None]


class Bus:
    """Dispatches MMIO windows to peripherals."""

    def __init__(self, machine: "Machine"):
        self.m = machine
        self.pages: Dict[int, Peripheral] = {}
        self.peripherals: Dict[str, Peripheral] = {}
        self.tracers: List[Tracer] = []
        self.windows: List[tuple] = []
        self.unknown_names: Dict[int, str] = {}
        #: address -> (peripheral, offset), filled on first access
        self._routes: Dict[int, tuple] = {}
        #: ctypes callbacks of the directly mapped windows (kept alive here)
        self._mmio_callbacks: List[tuple] = []

    def attach(self, p: Peripheral) -> Peripheral:
        self._routes.clear()
        self.peripherals[p.name] = p
        for a in range(p.base & ~(PAGE - 1), p.base + p.size, PAGE):
            cur = self.pages.get(a)
            if cur is None or (cur.base == a and cur.size >= PAGE and p.size >= PAGE):
                self.pages[a] = p
            else:
                # several small blocks share this page (e.g. STM32G4 ADC1/ADC2/common)
                if not isinstance(cur, _Splitter):
                    cur = _Splitter(self.m, a, [cur])
                    self.pages[a] = cur
                cur.add(p)
        return p

    def get(self, name: str) -> Peripheral:
        return self.peripherals[name]

    def owner(self, addr: int) -> Peripheral:
        page = addr & ~(PAGE - 1)
        p = self.pages.get(page)
        if p is None:
            name = self.unknown_names.get(page, f"MMIO_{page:08x}")
            p = UnknownPeripheral(self.m, name, page)
            self.pages[page] = p
            self.peripherals[name] = p
        return p

    def route(self, addr: int):
        """(peripheral, offset) that serves ``addr``, through any page splitter."""
        r = self._routes.get(addr)
        if r is None:
            p = self.owner(addr)
            if isinstance(p, _Splitter):
                p = p._find(addr - p.base) or p
            r = self._routes[addr] = (p, addr - p.base)
        return r

    def map_window(self, base: int, size: int) -> None:
        uc = self.m.uc
        routes = self._routes

        def rd(uc_, offset, sz, _ud):
            addr = base + offset
            r = routes.get(addr) or self.route(addr)
            p, off = r
            v = p.read32(off) if sz == 4 and not off & 3 else p.read(off, sz)
            if self.tracers:
                for t in self.tracers:
                    t("r", p, off, sz, v)
            return v

        def wr(uc_, offset, sz, value, _ud):
            addr = base + offset
            r = routes.get(addr) or self.route(addr)
            p, off = r
            if self.tracers:
                for t in self.tracers:
                    t("w", p, off, sz, value)
            if sz == 4 and not off & 3:
                p.write32(off, value & 0xFFFFFFFF)
            else:
                p.write(off, sz, value)

        if not self._map_direct(uc, base, size, routes):
            uc.mmio_map(base, size, rd, None, wr, None)
        self.windows.append((base, size, rd, wr))

    def _map_direct(self, uc, base: int, size: int, routes) -> bool:
        """Register the window's callbacks with unicorn's C API directly.

        Firmware driving a power stage touches peripheral registers hundreds
        of thousands of times per second; the binding's ``mmio_map`` puts two
        Python layers in front of each access. These callbacks do the same
        routing in one, with the binding's error handling: an exception stops
        the emulation and is raised from ``emu_start``. Returns False (use the
        binding) if its internals are not as expected."""
        try:
            from unicorn.unicorn_py3 import unicorn as ucmod
            read_t, write_t, lib = ucmod.MMIO_READ_CFUNC, ucmod.MMIO_WRITE_CFUNC, ucmod.uclib
            handle = uc._uch
            uc._hook_exception  # noqa: B018 - the binding reports hook errors here
        except (ImportError, AttributeError):
            return False
        route = self.route
        bus = self

        def fail(e):
            if uc._hook_exception is None:
                uc._hook_exception = e
            uc.emu_stop()

        def rd(_h, offset, sz, _ud):
            try:
                addr = base + offset
                p, off = routes.get(addr) or route(addr)
                v = p.read32(off) if sz == 4 and not off & 3 else p.read(off, sz)
                if bus.tracers:
                    for t in bus.tracers:
                        t("r", p, off, sz, v)
                return v
            except BaseException as e:
                fail(e)
                return 0

        def wr(_h, offset, sz, value, _ud):
            try:
                addr = base + offset
                p, off = routes.get(addr) or route(addr)
                if bus.tracers:
                    for t in bus.tracers:
                        t("w", p, off, sz, value)
                if sz == 4 and not off & 3:
                    p.write32(off, value & 0xFFFFFFFF)
                else:
                    p.write(off, sz, value)
            except BaseException as e:
                fail(e)

        cb_r, cb_w = read_t(rd), write_t(wr)
        if lib.uc_mmio_map(handle, base, size, cb_r, None, cb_w, None) != 0:
            return False
        self._mmio_callbacks.append((cb_r, cb_w))   # the C side holds no reference
        return True

    # direct access helpers (no tracing), used by DMA etc.
    def read(self, addr: int, size: int = 4) -> int:
        p = self.owner(addr)
        return p.read(addr - p.base, size)

    def write(self, addr: int, value: int, size: int = 4) -> None:
        p = self.owner(addr)
        p.write(addr - p.base, size, value)


class _Splitter(Peripheral):
    """Dispatches one 1 KiB page to several sub-page peripherals."""

    def __init__(self, machine, page, parts):
        super().__init__(machine, f"PAGE_{page:08x}", page)
        self.parts = []
        for p in parts:
            self.add(p)

    def add(self, p):
        self.parts.append(p)
        self.parts.sort(key=lambda x: -x.base)

    def _find(self, off):
        addr = self.base + off
        for p in self.parts:
            if p.base <= addr < p.base + p.size:
                return p
        return None

    def read(self, off, size):
        p = self._find(off)
        if p is None:
            return 0
        return p.read(self.base + off - p.base, size)

    def write(self, off, size, value):
        p = self._find(off)
        if p is not None:
            p.write(self.base + off - p.base, size, value)
