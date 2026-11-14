"""Peripheral models for the Nations N32G45x (Cortex-M4F) used by Marstek.

The register layout is close to an STM32F10x with renamed registers.  Models
are behavioural: they implement what the firmware observes (flags, interrupts,
data movement), not cycle-exact hardware.  Everything not modelled here falls
back to :class:`~marsemu.core.bus.UnknownPeripheral` (a register file).
"""
from __future__ import annotations

import struct
import ctypes
import datetime as _dt
import json
import logging
import os
from collections import deque
from typing import Callable, Dict, List, Optional

from ..core.bus import Peripheral

log = logging.getLogger("emu.periph")

_DB = json.load(open(os.path.join(os.path.dirname(__file__), "..", "data", "n32g45x.json")))["peripherals"]


def regnames(name: str) -> Dict[int, str]:
    return {int(k): v for k, v in _DB.get(name, {}).get("regs", {}).items()}


# --------------------------------------------------------------------------- base
class Base(Peripheral):
    """Peripheral with SVD register names and level-sensitive interrupts."""

    def __init__(self, machine, name, base, irqs=None):
        db = getattr(machine, "periph_db", None) or _DB
        if irqs is None:
            irqs = dict(db.get(name, {}).get("irqs", {}))
        super().__init__(machine, name, base, irqs)
        self.reg_names = {int(k): v for k, v in db.get(name, {}).get("regs", {}).items()}
        for irq_name, n in self.irqs.items():
            machine.nvic.irq_names[n] = irq_name
        self.reset()

    def level(self) -> Dict[str, bool]:
        """Return {irq-name: asserted}.  Override for level interrupts."""
        return {}

    # Plain register file for blocks without a dedicated model; busy-wait
    # loops on them are resolved (see PollResolver).
    def read32(self, off):
        if type(self) is Base:
            pr = self.__dict__.get("_poll")
            if pr is None:
                from ..core.bus import PollResolver
                pr = self._poll = PollResolver()
            return pr.read(self, off, self.regs.get(off, 0))
        return self.regs.get(off, 0)

    def write32(self, off, value):
        if type(self) is Base and "_poll" in self.__dict__:
            self._poll.note_write()
        self.regs[off] = value

    def update_irq(self) -> None:
        for name, on in self.level().items():
            if on:
                n = self.irqs.get(name)
                if n is not None and not self.m.nvic.pending[16 + n]:
                    self.m.nvic.set_pending(16 + n)

    def register_levels(self) -> None:
        for name in self.level().keys():
            n = self.irqs.get(name)
            if n is not None:
                self.m.level_sources.setdefault(16 + n, []).append(self._level_of(name))

    def _level_of(self, name):
        return lambda: self.level().get(name, False)


# --------------------------------------------------------------------------- RCC
class Rcc(Base):
    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x83, 0x04: 0, 0x14: 0x14, 0x24: 0x0C000003, 0x2C: 0x3800, 0x30: 0x3840})

    def write32(self, off, v):
        if off == 0x00:  # CTRL: ready flags follow enables
            v &= ~((1 << 1) | (1 << 17) | (1 << 25))
            if v & 1:
                v |= 1 << 1
            if v & (1 << 16):
                v |= 1 << 17
            if v & (1 << 24):
                v |= 1 << 25
        elif off == 0x04:  # CFG: switch status follows switch
            v = (v & ~0xC) | ((v & 3) << 2)
        elif off == 0x08:  # CLKINT: clear bits
            v &= 0x1F00
        elif off == 0x20:  # BDCTRL: LSE ready
            v = (v & ~2) | (2 if v & 1 else 0)
            if v & (1 << 16):
                v = 0
        elif off == 0x24:  # CTRLSTS: LSI ready, RMRSTF clears reset flags
            keep = self.regs.get(0x24, 0) & 0xFFF80000
            if v & (1 << 24):
                keep = 0
            v = (v & 0x1) | (2 if v & 1 else 0) | keep
        self.regs[off] = v

    def note_reset(self, why: str) -> None:
        flag = {"IWDG": 29, "WWDG": 30, "SYSRESETREQ": 28}.get(why)
        if flag:
            self.regs[0x24] = self.regs.get(0x24, 0) | (1 << flag)

    def sysclk(self) -> int:
        return self.m.clock.hz


# --------------------------------------------------------------------------- FLASH controller
class FlashCtl(Base):
    PAGE = 0x800

    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x30, 0x10: 0x80, 0x1C: 0x03FFFFFC, 0x20: 0xFFFFFFFF})
        self.key_stage = 0
        self.erases = 0

    def read32(self, off):
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x04:  # KEY
            if v == 0x45670123:
                self.key_stage = 1
            elif v == 0xCDEF89AB and self.key_stage == 1:
                self.regs[0x10] = self.regs.get(0x10, 0) & ~0x80
                self.key_stage = 0
            return
        if off == 0x0C:  # STS: write-1-to-clear
            self.regs[0x0C] = self.regs.get(0x0C, 0) & ~v
            return
        if off == 0x10:  # CTRL
            if v & 0x40:  # START
                if v & 0x4:
                    self._erase(self.m.flash_base, self.m.flash_size)
                elif v & 0x2:
                    a = self.regs.get(0x14, 0)
                    a = (a & ~(self.PAGE - 1))
                    if a < self.m.flash_base:
                        a += self.m.flash_base
                    self._erase(a, self.PAGE)
                v &= ~0x40
                self.regs[0x0C] = (self.regs.get(0x0C, 0) | 0x20) & ~1  # EOP
            self.regs[0x10] = v
            return
        self.regs[off] = v

    def _erase(self, addr: int, size: int) -> None:
        off = addr - self.m.flash_base
        if not (0 <= off < self.m.flash_size):
            log.warning("flash erase outside flash: 0x%08x", addr)
            return
        size = min(size, self.m.flash_size - off)
        ctypes.memset(ctypes.addressof(self.m.flash) + off, 0xFF, size)
        try:
            self.m.uc.ctl_remove_cache(addr, addr + size)
            self.m.uc.ctl_remove_cache(off, off + size)
        except Exception:  # pragma: no cover
            pass
        self.erases += 1
        log.debug("flash erase 0x%08x +0x%x (pc=0x%08x)", addr, size, self.m.pc)


# --------------------------------------------------------------------------- GPIO
class Gpio(Base):
    def __init__(self, machine, name, base, port: str):
        self.port = port
        self.inputs: Dict[int, int] = {}  # pin -> forced input level
        #: level read on floating inputs; boards mostly use external pull-ups
        #: (buttons, open-drain status lines), so idle-high is the safe default
        self.float_level = 1
        self.watchers: List[Callable[[int, int], None]] = []
        super().__init__(machine, name, base)

    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x44444444, 0x04: 0x44444444})

    def mode(self, pin: int) -> int:
        cfg = self.regs.get(0x00 if pin < 8 else 0x04, 0)
        return (cfg >> (4 * (pin & 7))) & 0xF

    def pin_out(self, pin: int) -> int:
        return (self.regs.get(0x0C, 0) >> pin) & 1

    def _pid(self) -> int:
        odr = self.regs.get(0x0C, 0)
        v = 0
        for pin in range(16):
            if pin in self.inputs:
                bit = self.inputs[pin]
            else:
                md = self.mode(pin)
                if md & 3:  # output: reads back the driven level
                    bit = (odr >> pin) & 1
                elif (md >> 2) == 2:  # input with pull-up/down
                    bit = (odr >> pin) & 1
                elif md == 0:  # analog
                    bit = 0
                else:
                    bit = self.float_level
            v |= (bit & 1) << pin
        return v

    def read32(self, off):
        if off == 0x08:
            return self._pid()
        return self.regs.get(off, 0)

    def write32(self, off, v):
        old = self.regs.get(0x0C, 0)
        if off == 0x10:  # PBSC: set/reset
            new = (old | (v & 0xFFFF)) & ~(v >> 16)
            self.regs[0x0C] = new
        elif off == 0x14:  # PBC
            self.regs[0x0C] = old & ~(v & 0xFFFF)
        else:
            self.regs[off] = v
        new = self.regs.get(0x0C, 0)
        if new != old and self.watchers:
            for pin in range(16):
                if (new ^ old) >> pin & 1:
                    for w in self.watchers:
                        w(pin, (new >> pin) & 1)

    def set_input(self, pin: int, level: int) -> None:
        self.inputs[pin] = level & 1
        self.m.bus.get("EXTI").gpio_edge(self.port, pin, level)


# --------------------------------------------------------------------------- EXTI
class Exti(Base):
    LINE_IRQ = {0: 6, 1: 7, 2: 8, 3: 9, 4: 10, **{i: 23 for i in range(5, 10)},
                **{i: 40 for i in range(10, 16)}, 16: 1, 17: 41, 18: 42}

    def __init__(self, machine, name, base):
        super().__init__(machine, name, base, irqs={})

    def write32(self, off, v):
        if off == 0x14:  # PEND: write-1-to-clear
            self.regs[0x14] = self.regs.get(0x14, 0) & ~v
            return
        if off == 0x10:  # SWIE
            self.regs[0x10] = v
            for line in range(20):
                if v & (1 << line):
                    self.trigger(line)
            return
        self.regs[off] = v

    def trigger(self, line: int) -> None:
        self.regs[0x14] = self.regs.get(0x14, 0) | (1 << line)
        if self.regs.get(0x00, 0) & (1 << line):
            irq = self.LINE_IRQ.get(line)
            if irq is not None:
                self.m.nvic.set_pending(16 + irq)

    SELECTOR = "AFIO"  # STM32: SYSCFG (EXTICR at the same offsets)

    def gpio_edge(self, port: str, pin: int, level: int) -> None:
        afio = self.m.bus.peripherals.get(self.SELECTOR)
        if afio is None:
            return
        cfg = afio.regs.get(0x08 + 4 * (pin // 4), 0)
        sel = (cfg >> (4 * (pin % 4))) & 0xF
        if sel != "ABCDEFG".index(port):
            return
        edge = 0x08 if level else 0x0C
        if self.regs.get(edge, 0) & (1 << pin):
            self.trigger(pin)


# --------------------------------------------------------------------------- USART
class UartDevice:
    """Something on the other end of a UART (FC41D, BMS, a console, ...)."""

    def attach(self, uart: "Usart") -> None:
        self.uart = uart

    def on_tx(self, data: bytes) -> None:  # bytes sent by the MCU
        raise NotImplementedError


class Usart(Base):
    """USART with an STM32F1-style register file (N32G45x).  Subclasses
    change the offsets/bits for the newer STM32 USART (ISR/ICR/RDR/TDR)."""

    O_STS, O_RDR, O_TDR, O_BRR, O_CR1, O_CR3 = 0x00, 0x04, 0x04, 0x08, 0x0C, 0x14
    CR1_UE, CR1_RE = 1 << 13, 1 << 2
    STS_RESET = 0xC0
    TXE, TC, RXNE, IDLE, ORE = 0x80, 0x40, 0x20, 0x10, 0x08
    CR3_DMAR = 0x40

    def __init__(self, machine, name, base):
        self.device: Optional[UartDevice] = None
        self.rx: deque = deque()
        self.rx_event = None
        self.idle_event = None
        self.tx_listeners: List[Callable[[bytes], None]] = []
        self.dma_rx = None
        self.tx_total = 0
        self.rx_total = 0
        self.pclk_div = 1 if name in ("USART1", "UART6", "UART7") else 2
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs[self.O_STS] = self.STS_RESET
        self.rx.clear()
        self.dma_rx = None

    @property
    def addr_dat(self) -> int:
        return self.base + self.O_TDR

    def baud(self) -> int:
        brr = self.regs.get(self.O_BRR, 0)
        if not brr:
            return 115200
        pclk = self.m.clock.hz // self.pclk_div
        return max(1200, int(pclk / brr))

    def byte_cycles(self) -> int:
        # 10 bits per frame at the configured baud
        b = min(max(self.baud(), 9600), 2_000_000)
        return self.m.clock.us_to_cycles(10_000_000 / b)

    def level(self):
        sts = self.regs.get(self.O_STS, 0)
        c1 = self.regs.get(self.O_CR1, 0)
        on = bool((sts & 0x80 and c1 & 0x80) or (sts & 0x40 and c1 & 0x40) or
                  (sts & 0x20 and c1 & 0x20) or (sts & 0x10 and c1 & 0x10) or (sts & 0x08 and c1 & 0x20))
        return {self.name: on}

    def _set(self, bits):
        self.regs[self.O_STS] = self.regs.get(self.O_STS, 0) | bits

    def _clr(self, bits):
        self.regs[self.O_STS] = self.regs.get(self.O_STS, 0) & ~bits

    def read32(self, off):
        if off == self.O_RDR:
            v = self.regs.get(self.O_RDR, 0)
            # reading the data register clears RXNE (and ORE/IDLE on F1-style parts)
            self._clr(self.RXNE | (self.IDLE | self.ORE if self.O_RDR == self.O_TDR else 0))
            self._schedule_rx()
            return v
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == self.O_TDR:
            self._tx(bytes([v & 0xFF]))
            self._set(self.TXE | self.TC)
            self.update_irq()
            return
        if off == self.O_STS:
            # F1-style: writing 0 clears TXC/RXDNE (rc_w0)
            self.regs[self.O_STS] = (self.regs.get(self.O_STS, 0) & (v | ~0x360)) & 0x3FF
            return
        self.regs[off] = v
        if off in (self.O_CR1, self.O_CR3):
            self.update_irq()
            self._schedule_rx()

    def _tx(self, data: bytes) -> None:
        self.tx_total += len(data)
        if self.device:
            self.device.on_tx(data)
        for l in self.tx_listeners:
            l(data)

    # -- receive side ----------------------------------------------------
    def inject(self, data: bytes) -> None:
        """Queue bytes to be received by the MCU at line speed."""
        self.rx.extend(data)
        self._schedule_rx()

    def rx_enabled(self) -> bool:
        c1 = self.regs.get(self.O_CR1, 0)
        return bool(c1 & self.CR1_UE) and bool(c1 & self.CR1_RE)

    def _schedule_rx(self) -> None:
        if self.rx and self.rx_event is None and self.rx_enabled():
            self.rx_event = self.m.clock.after(self.byte_cycles(), self._rx_byte, self.name + "-rx")

    def _rx_byte(self) -> None:
        self.rx_event = None
        if not self.rx or not self.rx_enabled():
            return
        dma = self.dma_rx if (self.regs.get(self.O_CR3, 0) & self.CR3_DMAR) else None
        if dma is not None and dma.active():
            b = self.rx.popleft()
            dma.push(b)
            self.rx_total += 1
        else:
            if self.regs.get(self.O_STS, 0) & self.RXNE:
                # previous byte not consumed: hold the line (flow control)
                self.rx_event = self.m.clock.after(self.byte_cycles(), self._rx_byte, self.name + "-rx")
                return
            b = self.rx.popleft()
            self.rx_total += 1
            self.regs[self.O_RDR] = b
            self._set(self.RXNE)
        if self.rx:
            self.rx_event = self.m.clock.after(self.byte_cycles(), self._rx_byte, self.name + "-rx")
        else:
            if self.idle_event:
                self.idle_event.cancel()
            self.idle_event = self.m.clock.after(self.byte_cycles(), self._idle, self.name + "-idle")
        self.update_irq()

    def _idle(self) -> None:
        self.idle_event = None
        if not self.rx:
            self._set(self.IDLE)
            self.update_irq()

    def dma_from_memory(self, ch, data: bytes) -> int:
        self._clr(self.TC)
        self._tx(data)

        def done():
            self._set(self.TXE | self.TC)
            self.update_irq()
        delay = self.byte_cycles() * len(data)
        self.m.clock.after(delay, done, self.name + "-txc")
        return delay

    def dma_to_memory(self, ch) -> None:
        self.dma_rx = ch
        self._schedule_rx()


# --------------------------------------------------------------------------- DMA
_PACK = {1: "<%dB", 2: "<%dH", 4: "<%dI"}


class DmaChannel:
    def __init__(self, dma: "Dma", idx: int):
        self.dma = dma
        self.idx = idx  # 1-based
        self.off = 0x08 + (idx - 1) * 0x14
        self.pos = 0
        self.total = 0
        #: raise the half-transfer flag (a board model may switch it off when the
        #: firmware does nothing on it, to save an interrupt per transfer)
        self.half_transfer = True

    def cfg(self) -> int:
        return self.dma.regs.get(self.off, 0)

    def active(self) -> bool:
        return bool(self.cfg() & 1)

    @property
    def ndtx(self) -> int:
        return self.dma.regs.get(self.off + 4, 0) & 0xFFFF

    @property
    def paddr(self) -> int:
        return self.dma.regs.get(self.off + 8, 0)

    @property
    def maddr(self) -> int:
        return self.dma.regs.get(self.off + 12, 0)

    def msize(self) -> int:
        return 1 << ((self.cfg() >> 10) & 3)

    def psize(self) -> int:
        return 1 << ((self.cfg() >> 8) & 3)

    def flag(self, bits: int) -> None:
        sh = 4 * (self.idx - 1)
        regs = self.dma.regs
        regs[0x00] = regs.get(0x00, 0) | ((bits | 1) << sh)
        if regs.get(self.off, 0) & 0xE:     # only an enabled flag can raise the interrupt
            self.dma.update_irq(self.idx)

    def push(self, value: int) -> None:
        """Peripheral -> memory transfer of one item (USART RX, ADC, ...)."""
        if not self.active():
            return
        n = self.ndtx
        if n == 0:
            return
        size = self.msize()
        step = size if self.cfg() & 0x80 else 0
        addr = self.maddr + self.pos * step
        try:
            self.dma.m.write(addr, (value & ((1 << (8 * size)) - 1)).to_bytes(size, "little"))
        except Exception:
            log.warning("%s ch%d bad memory address 0x%08x", self.dma.name, self.idx, addr)
        self.pos += 1
        n -= 1
        self.dma.regs[self.off + 4] = n
        if n == self.total // 2 and self.half_transfer:
            self.flag(0x4)
        if n == 0:
            self.flag(0x2)
            if self.cfg() & 0x20:  # circular
                self.dma.regs[self.off + 4] = self.total
                self.pos = 0

    def push_many(self, values) -> None:
        """Peripheral -> memory transfer of several items (an ADC sequence):
        the same as :meth:`push` for each, with one memory write per run of
        items that doesn't wrap."""
        cfg = self.cfg()
        if not cfg & 1:
            return
        size = 1 << ((cfg >> 10) & 3)
        if not cfg & 0x80 or len(values) == 1:   # no memory increment: item by item
            for v in values:
                self.push(v)
            return
        regs = self.dma.regs
        mask = (1 << (8 * size)) - 1
        i = 0
        while i < len(values):
            n = regs.get(self.off + 4, 0) & 0xFFFF
            if n == 0:
                return
            k = min(n, len(values) - i)
            half = self.total // 2
            if self.half_transfer and n > half > n - k:
                k = n - half                     # stop at the half-transfer point
            chunk = values[i:i + k]
            addr = regs.get(self.off + 12, 0) + self.pos * size
            data = struct.pack(_PACK[size] % len(chunk), *[v & mask for v in chunk])
            try:
                self.dma.m.write(addr, data)
            except Exception:
                log.warning("%s ch%d bad memory address 0x%08x", self.dma.name, self.idx, addr)
            self.pos += k
            n -= k
            i += k
            regs[self.off + 4] = n
            if n == self.total // 2 and self.half_transfer:
                self.flag(0x4)
            if n == 0:
                self.flag(0x2)
                if cfg & 0x20:  # circular
                    regs[self.off + 4] = self.total
                    self.pos = 0

    def start(self) -> None:
        self.pos = 0
        self.total = self.ndtx
        cfg = self.cfg()
        owner = self.dma.m.bus.owner(self.paddr) if self.paddr else None
        if owner is not None and hasattr(owner, "_find"):  # page shared by several blocks
            owner = owner._find(self.paddr - owner.base) or owner
        if cfg & 0x4000:  # mem2mem
            n = self.ndtx * self.msize()
            src, dst = (self.maddr, self.paddr) if cfg & 0x10 else (self.paddr, self.maddr)
            self.dma.m.write(dst, self.dma.m.read(src, n))
            self.dma.regs[self.off + 4] = 0
            self.flag(0x2)
            return
        if owner is None:
            return
        if cfg & 0x10:  # memory -> peripheral
            sink = getattr(owner, "dma_from_memory", None)
            if sink is None:
                return
            n = self.ndtx
            size = self.msize()
            step = size if cfg & 0x80 else 0
            if step == 1:
                data = self.dma.m.read(self.maddr, n)
            else:
                data = bytes(self.dma.m.read(self.maddr + i * step, 1)[0] for i in range(n))
            delay = sink(self, data)
            self.dma.regs[self.off + 4] = 0

            def done():
                self.flag(0x2 | 0x4)
                if self.cfg() & 0x20:
                    self.dma.regs[self.off + 4] = self.total
            if delay:
                self.dma.m.clock.after(delay, done, f"{self.dma.name}-ch{self.idx}")
            else:
                done()
        else:  # peripheral -> memory
            src = getattr(owner, "dma_to_memory", None)
            if src is not None:
                src(self)


class Dma(Base):
    def __init__(self, machine, name, base):
        self.ch = [None] + [DmaChannel(self, i) for i in range(1, 9)]
        super().__init__(machine, name, base)
        self.register_levels()

    def ch_level(self, i: int) -> bool:
        f = (self.regs.get(0x00, 0) >> (4 * (i - 1))) & 0xF
        return bool((f >> 1) & (self.regs.get(0x08 + (i - 1) * 0x14, 0) >> 1) & 0x7)

    def level(self):
        return {self._irq_for(i): self.ch_level(i) for i in range(1, 9)}

    def update_irq(self, idx: Optional[int] = None) -> None:
        """Pend the interrupt of channel ``idx`` (all channels if None) if its flags are up."""
        for i in ((idx,) if idx else range(1, 9)):
            if self.ch_level(i):
                n = self.irqs.get(self._irq_for(i))
                if n is not None and not self.m.nvic.pending[16 + n]:
                    self.m.nvic.set_pending(16 + n)

    def _level_of(self, name):
        for i in range(1, 9):
            if self._irq_for(i) == name:
                return lambda i=i: self.ch_level(i)
        return lambda: False

    def _irq_for(self, i: int) -> str:
        cache = self.__dict__.setdefault("_irq_cache", {})
        name = cache.get(i)
        if name is None:
            name = cache[i] = self._irq_name(i)
        return name

    def _irq_name(self, i: int) -> str:
        for name in self.irqs:
            if name.endswith((f"Channel{i}", f"CH{i}", f"_Ch{i}")) and name.startswith(self.name):
                return name
        return f"{self.name}_Channel{i}"

    def write32(self, off, v):
        if off == 0x04:  # INTCLR
            self.regs[0x00] = self.regs.get(0x00, 0) & ~v
            return
        if off == 0x00:
            return
        old = self.regs.get(off, 0)
        self.regs[off] = v
        if off >= 0x08 and off < 0xA8 and (off - 0x08) % 0x14 == 0:
            ch = self.ch[(off - 0x08) // 0x14 + 1]
            if v & 1 and not old & 1:
                ch.start()
            self.update_irq()


# --------------------------------------------------------------------------- Timers
class Timer(Base):
    def __init__(self, machine, name, base):
        self.ev = None
        self.t0 = 0
        self.overflows = 0
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs[0x2C] = 0xFFFF
        if getattr(self, "ev", None):
            self.ev.cancel()
        self.ev = None

    def _irqname(self, kind: str) -> str:
        if self.name in ("TIM1", "TIM8"):
            return f"{self.name}_{'UP' if kind == 'up' else 'CC'}"
        return self.name

    def level(self):
        sts = self.regs.get(0x10, 0)
        en = self.regs.get(0x0C, 0)
        up = bool(sts & en & 1)
        cc = bool(sts & en & 0x1E)
        if len(self.irqs) <= 1:
            return {n: up or cc for n in self.irqs} or {self.name: up or cc}
        out = {}
        for n in self.irqs:
            if "_UP" in n:
                out[n] = up
            elif "_CC" in n:
                out[n] = cc
            else:
                out[n] = False
        return out

    def period(self) -> int:
        psc = self.regs.get(0x28, 0) & 0xFFFF
        arr = self.regs.get(0x2C, 0xFFFF) & 0xFFFF
        return (psc + 1) * (arr + 1)

    def _restart(self) -> None:
        if self.ev:
            self.ev.cancel()
            self.ev = None
        if self.regs.get(0x00, 0) & 1:
            self.t0 = self.m.clock.cycles
            # only schedule real events when someone listens (IRQ or DMA);
            # otherwise the update flag is derived lazily from elapsed time
            if self.regs.get(0x0C, 0) & 0x1F1F:
                self.ev = self.m.clock.after(self.period(), self._update, self.name)

    def _update(self) -> None:
        self.overflows += 1
        self.regs[0x10] = self.regs.get(0x10, 0) | 1 | (0x1E & self._cc_mask())
        self.t0 = self.m.clock.cycles
        self.ev = None
        if self.regs.get(0x00, 0) & 0x8:  # one-pulse mode
            self.regs[0x00] &= ~1
        elif self.regs.get(0x0C, 0) & 0x1F1F:
            self.ev = self.m.clock.after(self.period(), self._update, self.name)
        self.update_irq()

    def _cc_mask(self) -> int:
        # capture/compare channels whose interrupts are enabled fire once per period,
        # except inputs a board model drives (see capture())
        return self.regs.get(0x0C, 0) & 0x1E & ~self.__dict__.get("external_cc", 0)

    def capture(self, ch: int) -> None:
        """An edge on input capture channel ``ch`` (1-4): latch the counter into
        CCRch and raise the capture flag.  Marks the channel as driven by a
        model, so it no longer fires once per period on its own."""
        self.external_cc = self.__dict__.get("external_cc", 0) | (1 << ch)
        self.regs[0x34 + 4 * (ch - 1)] = self.read32(0x24)
        self.regs[0x10] = self.regs.get(0x10, 0) | (1 << ch)
        self.update_irq()

    def read32(self, off):
        if off == 0x24:
            if self.regs.get(0x00, 0) & 1:
                psc = (self.regs.get(0x28, 0) & 0xFFFF) + 1
                arr = (self.regs.get(0x2C, 0xFFFF) & 0xFFFF) + 1
                return ((self.m.clock.cycles - self.t0) // psc) % arr
            return self.regs.get(0x24, 0)
        if off == 0x10 and self.ev is None and self.regs.get(0x00, 0) & 1:
            if self.m.clock.cycles - self.t0 >= self.period():
                self.regs[0x10] = self.regs.get(0x10, 0) | 1
                self.t0 += ((self.m.clock.cycles - self.t0) // self.period()) * self.period()
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x10:  # STS rc_w0
            self.regs[0x10] = self.regs.get(0x10, 0) & v
            return
        if off == 0x14:  # EVTGEN
            if v & 1:
                self.regs[0x10] = self.regs.get(0x10, 0) | 1
                self.t0 = self.m.clock.cycles
                self.update_irq()
            return
        old = self.regs.get(off, 0)
        self.regs[off] = v
        if off == 0x00 and (old ^ v) & 1:
            self._restart()
        elif off == 0x0C:
            if (old ^ v) & 0x1F1F:
                self._restart() if self.regs.get(0x00, 0) & 1 else None
            self.update_irq()


# --------------------------------------------------------------------------- ADC
class Adc(Base):
    def __init__(self, machine, name, base):
        self.values: Dict[int, int] = {}
        self.dma = None
        self.default = 2048
        self.provider: Optional[Callable[[str, int], Optional[int]]] = None
        self.conv_event = None
        super().__init__(machine, name, base, irqs={})

    def reset(self):
        super().reset()
        self.regs[0x58] = 0x60  # CTRL3: RDY | PDRDY

    def sample(self, ch: int) -> int:
        if self.provider:
            v = self.provider(self.name, ch)
            if v is not None:
                return v & 0xFFF
        if ch == 16:  # temperature sensor ~25 C
            return 1750
        if ch == 17:  # internal reference 1.2 V @ 3.3 V
            return 1490
        return self.values.get(ch, self.default)

    def sequence(self) -> List[int]:
        n = ((self.regs.get(0x2C, 0) >> 20) & 0xF) + 1
        seq = []
        for i in range(n):
            reg = 0x34 if i < 6 else (0x30 if i < 12 else 0x2C)
            seq.append((self.regs.get(reg, 0) >> (5 * (i % 6))) & 0x1F)
        return seq

    def read32(self, off):
        if off == 0x4C:
            self.regs[0x00] = self.regs.get(0x00, 0) & ~2
            return self.regs.get(0x4C, 0)
        if off == 0x58:
            return self.regs.get(0x58, 0) | 0x60
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x00:
            self.regs[0x00] = self.regs.get(0x00, 0) & v
            return
        if off == 0x08:
            v &= ~(1 << 2) & ~(1 << 3)  # calibration completes instantly
            if v & ((1 << 22) | (1 << 21)) and v & 1:
                self.regs[0x08] = v & ~((1 << 22) | (1 << 21))
                self._convert(injected=bool(v & (1 << 21)))
                return
        self.regs[off] = v
        if off == 0x08 and v & 1 and v & 2 and v & (1 << 8):  # continuous + DMA
            self._convert()

    def _convert(self, injected: bool = False) -> None:
        if injected:
            jl = ((self.regs.get(0x38, 0) >> 20) & 3) + 1
            for i in range(jl):
                ch = (self.regs.get(0x38, 0) >> (5 * (4 - jl + i))) & 0x1F
                self.regs[0x3C + 4 * i] = self.sample(ch)
            self.regs[0x00] = self.regs.get(0x00, 0) | 0x4 | 0x8
            return
        seq = self.sequence()
        dma_on = self.regs.get(0x08, 0) & (1 << 8)
        for ch in seq:
            val = self.sample(ch)
            self.regs[0x4C] = val
            if dma_on and self.dma is not None:
                self.dma.push(val)
        self.regs[0x00] = self.regs.get(0x00, 0) | 0x2 | 0x10
        if self.regs.get(0x08, 0) & 2 and self.conv_event is None:  # continuous: refresh every ms
            def again():
                self.conv_event = None
                if self.regs.get(0x08, 0) & 3 == 3:
                    self._convert()
            self.conv_event = self.m.clock.after_ms(1, again, self.name)

    def dma_to_memory(self, ch) -> None:
        self.dma = ch
        if self.regs.get(0x08, 0) & 1:
            self._convert()


# --------------------------------------------------------------------------- I2C
class I2cDevice:
    """Slave on an I2C bus.  Override what you need."""
    address = 0

    def start(self, read: bool) -> bool:  # return False to NACK
        return True

    def write(self, b: int) -> bool:
        return True

    def read(self) -> int:
        return 0xFF

    def stop(self) -> None:
        pass


class RegI2cDevice(I2cDevice):
    """Typical register-addressed device: first byte written selects a register."""

    def __init__(self, address: int, regs: Optional[Dict[int, int]] = None, size: int = 256):
        self.address = address
        self.mem = bytearray(size)
        for k, v in (regs or {}).items():
            self.mem[k] = v
        self.ptr = 0
        self.first = True

    def start(self, read):
        if not read:
            self.first = True
        return True

    def write(self, b):
        if self.first:
            self.ptr = b
            self.first = False
        else:
            self.mem[self.ptr % len(self.mem)] = b
            self.ptr += 1
        return True

    def read(self):
        v = self.mem[self.ptr % len(self.mem)]
        self.ptr += 1
        return v


class I2cEeprom(I2cDevice):
    """24Cxx serial EEPROM: address bytes then data, sequential read/write.

    Blank cells read 0xFF.  Writes wrap inside a page like the real part."""

    def __init__(self, address: int = 0x50, size: int = 32768, addr_bytes: int = 2, page: int = 64,
                 path: Optional[str] = None):
        self.address = address
        self.size = size
        self.addr_bytes = addr_bytes
        self.page = page
        self.path = path
        self.mem = bytearray(b"\xff" * size)
        if path:
            try:
                data = open(path, "rb").read()
                self.mem[:len(data)] = data[:size]
            except OSError:
                pass
        self.ptr = 0
        self.naddr = 0
        self.dirty = False

    def start(self, read: bool) -> bool:
        if not read:
            self.naddr = 0
        return True

    def write(self, b: int) -> bool:
        if self.naddr < self.addr_bytes:
            self.ptr = ((self.ptr << 8) | b) if self.naddr else b
            self.naddr += 1
            if self.naddr == self.addr_bytes:
                self.ptr %= self.size
            return True
        self.mem[self.ptr] = b
        self.dirty = True
        base = self.ptr - self.ptr % self.page
        self.ptr = base + (self.ptr + 1) % self.page
        return True

    def read(self) -> int:
        v = self.mem[self.ptr]
        self.ptr = (self.ptr + 1) % self.size
        return v

    def save(self) -> None:
        if self.path and self.dirty:
            open(self.path, "wb").write(self.mem)
            self.dirty = False


class I2c(Base):
    def __init__(self, machine, name, base):
        self.devices: Dict[int, I2cDevice] = {}
        self.cur: Optional[I2cDevice] = None
        self.reading = False
        self.default_ack = True
        self.unknown_seen: set = set()
        #: bytes on the wire since the last STOP (the CPU waits for them at STOP)
        self.txn_bytes = 0
        super().__init__(machine, name, base)
        self.register_levels()

    def bit_seconds(self) -> float:
        """SCL period from the clock control register and the APB clock (CTRL2.FREQ)."""
        mhz = self.regs.get(0x04, 0) & 0x3F
        cc = self.regs.get(0x1C, 0)
        ccr = cc & 0xFFF
        if not mhz or not ccr:
            return 10e-6
        k = (25 if cc & 0x4000 else 3) if cc & 0x8000 else 2
        return k * ccr / (mhz * 1e6)

    def level(self):
        s1 = self.regs.get(0x14, 0)
        c2 = self.regs.get(0x04, 0)
        ev = bool(c2 & 0x200 and s1 & 0x1F) or bool(c2 & 0x600 == 0x600 and s1 & 0xC0)
        er = bool(c2 & 0x100 and s1 & 0xDF00)
        evn, ern = f"{self.name}_EV", f"{self.name}_ER"
        return {evn: ev, ern: er}

    def read32(self, off):
        if off == 0x18:
            # reading STS2 after STS1 clears ADDRF
            s1 = self.regs.get(0x14, 0)
            if s1 & 2:
                self.regs[0x14] = s1 & ~2
                if self.reading and self.cur is not None:
                    self._load_rx()
            s2 = self.regs.get(0x18, 0)
            return s2
        if off == 0x10:
            v = self.regs.get(0x10, 0)
            if self.reading and self.cur is not None:
                self._load_rx()
            else:
                self.regs[0x14] = self.regs.get(0x14, 0) & ~0x44   # last byte taken: RXNE/BTF clear
            return v
        if off == 0x14:
            return self.regs.get(0x14, 0)
        return self.regs.get(off, 0)

    def _load_rx(self) -> None:
        self.txn_bytes += 1
        self.regs[0x10] = self.cur.read() & 0xFF
        self.regs[0x14] = self.regs.get(0x14, 0) | 0x40 | 0x04

    def write32(self, off, v):
        if off == 0x00:
            if v & (1 << 15):  # software reset
                self.regs = {0x00: 0}
                self.cur = None
                return
            if v & (1 << 8):  # START
                v &= ~(1 << 8)
                self.regs[0x14] = (self.regs.get(0x14, 0) | 1) & ~0x0400
                self.regs[0x18] = self.regs.get(0x18, 0) | 3
            if v & (1 << 9):  # STOP
                v &= ~(1 << 9)
                if self.cur:
                    self.cur.stop()
                # a byte already shifted in during a read stays readable (RXNE)
                keep = self.regs.get(0x14, 0) & 0x44 if self.reading else 0
                if self.txn_bytes:
                    # the transfer took bus time the model skipped: let the CPU wait
                    n, self.txn_bytes = self.txn_bytes, 0
                    self.m.stall(int((9 * n + 2) * self.bit_seconds() * self.m.clock.hz))
                self.cur = None
                self.reading = False
                self.regs[0x18] = 0
                self.regs[0x14] = (self.regs.get(0x14, 0) & ~0xC6) | keep
            self.regs[0x00] = v
            self.update_irq()
            return
        if off == 0x10:
            s1 = self.regs.get(0x14, 0)
            self.txn_bytes += 1
            if s1 & 1:  # address phase
                addr7 = (v >> 1) & 0x7F
                rd = bool(v & 1)
                dev = self.devices.get(addr7)
                if dev is None and self.default_ack:
                    if addr7 not in self.unknown_seen:
                        self.unknown_seen.add(addr7)
                        log.info("%s: transaction with unmodelled device 0x%02x (pc=0x%08x)", self.name, addr7, self.m.pc)
                    dev = self.devices.setdefault(addr7, RegI2cDevice(addr7))
                s1 &= ~1
                if dev is not None and dev.start(rd):
                    self.cur = dev
                    self.reading = rd
                    s1 |= 2
                    if rd:
                        self.regs[0x18] = 3
                    else:
                        self.regs[0x18] = 7
                        s1 |= 0x80
                else:
                    s1 |= 0x400
                self.regs[0x14] = s1
            else:
                if self.cur is not None and not self.reading:
                    if not self.cur.write(v & 0xFF):
                        self.regs[0x14] = self.regs.get(0x14, 0) | 0x400
                self.regs[0x14] = self.regs.get(0x14, 0) | 0x84
            self.regs[0x10] = v & 0xFF
            self.update_irq()
            return
        if off == 0x14:
            self.regs[0x14] = self.regs.get(0x14, 0) & (v | 0xFF)
            return
        self.regs[off] = v
        if off == 0x04:
            self.update_irq()

    def dma_from_memory(self, ch, data: bytes) -> int:
        for b in data:
            if self.cur is not None:
                self.cur.write(b)
        return 0

    def dma_to_memory(self, ch) -> None:
        if self.cur is not None:
            for _ in range(ch.ndtx):
                ch.push(self.cur.read())


# --------------------------------------------------------------------------- SPI
class SpiDevice:
    def select(self, active: bool) -> None:
        pass

    def xfer(self, b: int) -> int:
        return 0xFF


class Spi(Base):
    def __init__(self, machine, name, base):
        self.device: Optional[SpiDevice] = None
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs[0x08] = 0x02

    def level(self):
        sts = self.regs.get(0x08, 0)
        c2 = self.regs.get(0x04, 0)
        return {self.name: bool((c2 & 0x80 and sts & 2) or (c2 & 0x40 and sts & 1))}

    def read32(self, off):
        if off == 0x0C:
            self.regs[0x08] = self.regs.get(0x08, 0) & ~1
            return self.regs.get(0x0C, 0)
        if off == 0x08:
            return self.regs.get(0x08, 0) | 2
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x0C:
            out = self.device.xfer(v & 0xFF) if self.device else 0xFF
            self.regs[0x0C] = out
            self.regs[0x08] = self.regs.get(0x08, 0) | 3
            self.update_irq()
            return
        self.regs[off] = v

    def dma_from_memory(self, ch, data: bytes) -> int:
        last = 0xFF
        for b in data:
            last = self.device.xfer(b) if self.device else 0xFF
        self.regs[0x0C] = last
        self.regs[0x08] = self.regs.get(0x08, 0) | 3
        return 0

    def dma_to_memory(self, ch) -> None:
        # RX DMA is paired with a TX DMA; values are produced by xfer()
        pass


# --------------------------------------------------------------------------- IWDG
class Iwdg(Base):
    def __init__(self, machine, name, base):
        self.ev = None
        self.enabled = True  # can be disabled from the CLI
        self.started = False
        self.fired = 0
        self.last_kick = 0
        super().__init__(machine, name, base, irqs={})

    def reset(self):
        super().reset()
        self.regs.update({0x08: 0xFFF})
        if getattr(self, "ev", None):
            self.ev.cancel()
        self.ev = None
        self.started = False

    def timeout_cycles(self) -> int:
        pd = self.regs.get(0x04, 0) & 7
        rl = self.regs.get(0x08, 0xFFF) & 0xFFF
        secs = (4 << pd) * (rl + 1) / 40000.0
        return int(secs * self.m.clock.hz)

    def _kick(self) -> None:
        # cheap: just remember the time; the pending event re-arms itself lazily
        self.last_kick = self.m.clock.cycles
        if self.ev is None:
            self.ev = self.m.clock.after(self.timeout_cycles(), self._expire, "iwdg")

    def _expire(self) -> None:
        self.ev = None
        if self.m.feeding_watchdog():
            # asleep in a loop that reloads the watchdog on every iteration
            self.last_kick = self.m.clock.cycles
        due = self.last_kick + self.timeout_cycles()
        if due > self.m.clock.cycles:
            self.ev = self.m.clock.at(due, self._expire, "iwdg")
            return
        self.fired += 1
        if self.enabled:
            log.warning("IWDG expired (pc=0x%08x) -> reset", self.m.pc)
            self.m.bus.get("RCC").note_reset("IWDG")
            self.m.request_reset("IWDG")
        else:
            log.info("IWDG expired (ignored)")

    def read32(self, off):
        if off == 0x0C:
            return 0
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x00:
            k = v & 0xFFFF
            if k == 0xCCCC:
                self.started = True
                self._kick()
            elif k == 0xAAAA and self.started:
                self._kick()
            return
        self.regs[off] = v


# --------------------------------------------------------------------------- RTC (calendar)
def _bcd(n: int) -> int:
    return ((n // 10) << 4) | (n % 10)


def _unbcd(b: int) -> int:
    return (b >> 4) * 10 + (b & 0xF)


class Rtc(Base):
    """Calendar RTC.  Runs on virtual time starting at a configurable epoch."""

    def __init__(self, machine, name, base):
        self.epoch = _dt.datetime.now().replace(microsecond=0)
        self.offset = 0.0
        self.wkup_ev = None
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs.update({0x0C: 0x07 | 0x20 | 0x10, 0x10: 0x007F00FF, 0x14: 0xFFFF})

    def now(self) -> _dt.datetime:
        return self.epoch + _dt.timedelta(seconds=self.m.clock.seconds + self.offset)

    def level(self):
        sts = self.regs.get(0x0C, 0)
        ctrl = self.regs.get(0x08, 0)
        return {"RTC": bool(sts & 0x400 and ctrl & 0x4000), "RTCAlarm": bool(sts & 0x300 and ctrl & 0x3000)}

    def read32(self, off):
        t = self.now()
        if off == 0x00:
            return _bcd(t.second) | (_bcd(t.minute) << 8) | (_bcd(t.hour) << 16)
        if off == 0x04:
            return _bcd(t.day) | (_bcd(t.month) << 8) | ((t.isoweekday()) << 13) | (_bcd(t.year % 100) << 16)
        if off == 0x0C:
            return self.regs.get(0x0C, 0) | 0x20 | 0x04 | 0x10
        if off == 0x28:
            return 0xFF - int((self.m.clock.seconds % 1) * 256)
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x0C:
            cur = self.regs.get(0x0C, 0)
            if v & 0x80:
                cur |= 0xC0
            else:
                cur &= ~0xC0
            cur &= ~(~v & 0x1F00)  # clear event flags written as 0
            self.regs[0x0C] = cur
            return
        if off in (0x00, 0x04):
            self.regs[off] = v
            tsh, date = self.regs.get(0x00, 0), self.regs.get(0x04, 0)
            try:
                newt = _dt.datetime(2000 + _unbcd((date >> 16) & 0xFF), max(1, _unbcd((date >> 8) & 0x1F)),
                                    max(1, _unbcd(date & 0x3F)), _unbcd((tsh >> 16) & 0x3F),
                                    _unbcd((tsh >> 8) & 0x7F), _unbcd(tsh & 0x7F))
                self.offset += (newt - self.now()).total_seconds()
            except ValueError:
                pass
            return
        self.regs[off] = v
        if off in (0x08, 0x14):
            self._wkup()

    def _wkup(self) -> None:
        if self.wkup_ev:
            self.wkup_ev.cancel()
            self.wkup_ev = None
        ctrl = self.regs.get(0x08, 0)
        if ctrl & 0x400:
            sel = ctrl & 7
            n = (self.regs.get(0x14, 0) & 0xFFFF) + 1
            secs = n if sel & 4 else n * (16 >> sel) / 32768.0
            secs = max(secs, 0.001)

            def fire():
                self.regs[0x0C] = self.regs.get(0x0C, 0) | 0x400
                self.m.bus.get("EXTI").trigger(20)
                self.update_irq()
                self._wkup()
            self.wkup_ev = self.m.clock.after(int(secs * self.m.clock.hz), fire, "rtc-wkup")


# --------------------------------------------------------------------------- CRC
class Crc(Base):
    def reset(self):
        super().reset()
        self.crc = 0xFFFFFFFF
        self.crc16 = 0

    def read32(self, off):
        if off == 0x00:
            return self.crc
        if off == 0x10:
            return self.crc16
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x00:
            # STM32-style CRC32 (poly 0x04C11DB7, MSB first, 32-bit words)
            crc = self.crc ^ v
            for _ in range(32):
                crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
                crc &= 0xFFFFFFFF
            self.crc = crc
            return
        if off == 0x08 and v & 1:
            self.crc = 0xFFFFFFFF
            return
        if off == 0x14:  # CRC16D: one byte at a time (CRC16/MODBUS style)
            ctrl = self.regs.get(0x0C, 0)
            crc = self.crc16
            b = v & 0xFF
            if ctrl & 2:  # little endian input
                crc ^= b
                for _ in range(8):
                    crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
            else:
                crc ^= b << 8
                for _ in range(8):
                    crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            self.crc16 = crc & 0xFFFF
            return
        if off == 0x10:
            self.crc16 = v & 0xFFFF
            return
        self.regs[off] = v


# --------------------------------------------------------------------------- CAN (bxCAN compatible)
class Can(Base):
    def __init__(self, machine, name, base):
        self.tx_listeners: List[Callable[[int, bytes, bool], None]] = []
        self.rx: deque = deque()
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x10002, 0x04: 0xC02, 0x08: 0x1C000000})
        if hasattr(self, "rx"):
            self.rx.clear()

    def level(self):
        ie = self.regs.get(0x14, 0)
        rf0 = self.regs.get(0x0C, 0)
        tsts = self.regs.get(0x08, 0)
        return {f"{self.name}_RX0": bool(ie & 2 and rf0 & 3),
                f"{self.name}_TX": bool(ie & 1 and tsts & 0x10101)}

    def write32(self, off, v):
        if off == 0x00:  # MCTRL: INRQ/SLEEP handshake
            self.regs[0x00] = v
            msts = self.regs.get(0x04, 0) & ~3
            if v & 1:
                msts |= 1
            elif v & 2:
                msts |= 2
            self.regs[0x04] = msts
            return
        if off == 0x08:  # TSTS: rc_w1
            self.regs[0x08] = self.regs.get(0x08, 0) & ~(v & 0x00838383)
            return
        if off == 0x0C:  # RFF0: release mailbox
            if v & 0x20 and self.rx:
                self.rx.popleft()
                self._load_rx()
            return
        self.regs[off] = v
        if off in (0x180, 0x190, 0x1A0) and v & 1:
            mb = (off - 0x180) // 0x10
            tmi = v
            ide = bool(tmi & 4)
            ident = (tmi >> 3) if ide else (tmi >> 21)
            dlc = self.regs.get(off + 4, 0) & 0xF
            data = (self.regs.get(off + 8, 0).to_bytes(4, "little") + self.regs.get(off + 12, 0).to_bytes(4, "little"))[:dlc]
            self.regs[off] = v & ~1
            self.regs[0x08] = self.regs.get(0x08, 0) | (3 << (8 * mb)) | (1 << (26 + mb))
            for l in self.tx_listeners:
                l(ident, data, ide)
            self.update_irq()
        elif off == 0x14:
            self.update_irq()

    def inject(self, ident: int, data: bytes, ide: bool = False) -> bool:
        if len(self.rx) >= 3:  # FIFO 0 is three messages deep: overrun
            self.regs[0x0C] = self.regs.get(0x0C, 0) | 0x10
            return False
        self.rx.append((ident, bytes(data), ide))
        self._load_rx()
        return True

    def _load_rx(self) -> None:
        if not self.rx:
            self.regs[0x0C] = 0
            self.update_irq()
            return
        ident, data, ide = self.rx[0]
        self.regs[0x1B0] = ((ident << 3) | 4) if ide else (ident << 21)
        self.regs[0x1B4] = len(data)
        d = data.ljust(8, b"\0")
        self.regs[0x1B8] = int.from_bytes(d[:4], "little")
        self.regs[0x1BC] = int.from_bytes(d[4:], "little")
        self.regs[0x0C] = min(len(self.rx), 3) | (self.regs.get(0x0C, 0) & 0x18)
        self.update_irq()


# --------------------------------------------------------------------------- misc
class Pwr(Base):
    def reset(self):
        super().reset()
        self.regs[0x08] = 0x6E4


class Qspi(Base):
    def read32(self, off):
        if off == 0x28:
            return 0x2 | 0x4
        return self.regs.get(off, 0)


class Bkp(Base):
    """Backup registers survive soft resets (they are battery backed)."""

    def reset(self):
        if not hasattr(self, "regs") or not self.regs:
            super().reset()


class Wwdg(Base):
    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x7F, 0x04: 0x7F})


CLASSES = {
    "RCC": Rcc, "FLASH": FlashCtl, "EXTI": Exti, "DMA1": Dma, "DMA2": Dma, "IWDG": Iwdg, "RTC": Rtc,
    "CRC": Crc, "PWR": Pwr, "QSPI": Qspi, "BKP": Bkp, "WWDG": Wwdg,
    "CAN1": Can, "CAN2": Can,
}
for _n in ("USART1", "USART2", "USART3", "UART4", "UART5", "UART6", "UART7"):
    CLASSES[_n] = Usart
for _n in ("TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM6", "TIM7", "TIM8"):
    CLASSES[_n] = Timer
for _n in ("ADC1", "ADC2", "ADC3", "ADC4"):
    CLASSES[_n] = Adc
for _n in ("I2C1", "I2C2", "I2C3", "I2C4"):
    CLASSES[_n] = I2c
for _n in ("SPI1", "SPI2", "SPI3"):
    CLASSES[_n] = Spi


def build(machine, uid: bytes = bytes.fromhex("4d5354454d55303132333435"), flash_kb: int = 512) -> None:
    """Attach all N32G45x peripherals to ``machine``."""
    machine.level_sources = getattr(machine, "level_sources", {})
    for name, info in _DB.items():
        base = info["base"]
        if name.startswith("GPIO"):
            p = Gpio(machine, name, base, name[-1])
        else:
            cls = CLASSES.get(name, Base)
            p = cls(machine, name, base)
        machine.bus.attach(p)
    # peripheral windows (APB1/APB2/AHB) and the QSPI/XMC control block
    machine.bus.map_window(0x40000000, 0x30000)
    machine.bus.map_window(0xA0000000, 0x2000)
    # external SRAM window used by some Venus builds (XMC bank 1)
    machine.map_ram(0x60000000, 0x100000)
    # system memory: UID (0x1FFFF7F0) and flash size (0x1FFFF7E0)
    machine.map_ram(0x1FFF0000, 0x10000)
    machine.write(0x1FFFF7E0, (flash_kb).to_bytes(2, "little"))
    uid12 = uid[:12].ljust(12, b"\0")
    machine.write(0x1FFFF7F0, uid12)  # UID (reference manual)
    machine.write(0x1FFFF270, uid12)  # where Marstek firmware actually reads its 96-bit ID
    machine.write(0x1FFFF7C0, (uid12 + b"N32G").ljust(16, b"\0"))  # UCID
    machine.write(0x1FFFF7E8, uid12)  # STM32F1-style location, too
    _map_bitband(machine)


def _map_bitband(machine) -> None:
    """Bit-band alias regions for SRAM (0x22000000) and peripherals (0x42000000)."""
    uc = machine.uc

    def sram_rd(uc_, off, size, _):
        addr = 0x20000000 + (off >> 5)
        bit = (off >> 2) & 7
        return (machine.read(addr, 1)[0] >> bit) & 1

    def sram_wr(uc_, off, size, value, _):
        addr = 0x20000000 + (off >> 5)
        bit = (off >> 2) & 7
        b = machine.read(addr, 1)[0]
        b = (b | (1 << bit)) if value & 1 else (b & ~(1 << bit))
        machine.write(addr, bytes([b & 0xFF]))

    def per_rd(uc_, off, size, _):
        addr = 0x40000000 + (off >> 5)
        bit = (off >> 2) & 31
        word = addr & ~3
        bitpos = bit + 8 * (addr & 3)
        return (machine.bus.read(word, 4) >> bitpos) & 1

    def per_wr(uc_, off, size, value, _):
        addr = 0x40000000 + (off >> 5)
        bit = (off >> 2) & 7
        word = addr & ~3
        bitpos = bit + 8 * (addr & 3)
        p = machine.bus.owner(word)
        cur = p.regs.get(word - p.base, 0)
        new = (cur | (1 << bitpos)) if value & 1 else (cur & ~(1 << bitpos))
        p.write32(word - p.base, new)

    uc.mmio_map(0x22000000, 0x02000000, sram_rd, None, sram_wr, None)
    uc.mmio_map(0x42000000, 0x00600000, per_rd, None, per_wr, None)
    machine._bitband_cbs = (sram_rd, sram_wr, per_rd, per_wr)


# --------------------------------------------------------------------------- SPI NOR flash
class SpiNorFlash(SpiDevice):
    """W25Qxx-style serial NOR flash on an SPI bus.

    Commands: 9F (JEDEC ID), 90 (manufacturer/device ID), 05/35 (status),
    06/04 (write enable/disable), 03/0B (read / fast read), 02 (page program),
    20/52/D8 (4K/32K/64K erase), C7/60 (chip erase), B9/AB (power down/up).

    Chip select: if ``cs`` = (gpio_peripheral, pin) is given, transactions are
    framed by that pin; otherwise any change of a GPIO output ends the
    current command (firmwares toggle CS between transactions)."""

    def __init__(self, machine, size: int = 4 << 20, jedec: bytes = b"\xef\x40\x16", path: str = None,
                 cs=None):
        self.m = machine
        self.mem = bytearray(b"\xff" * size)
        self.jedec = jedec
        self.path = path
        if path:
            try:
                data = open(path, "rb").read()
                self.mem[:len(data)] = data[:size]
            except OSError:
                pass
        self.cmd = []
        self.wel = False
        self.selected = False
        if cs is not None:
            gpio, pin = cs
            gpio.watchers.append(lambda p, lvl: self._cs(lvl == 0) if p == pin else None)
        else:
            for name, p in machine.bus.peripherals.items():
                if name.startswith("GPIO") and hasattr(p, "watchers"):
                    p.watchers.append(lambda pin, lvl: self._end())

    def _cs(self, active: bool) -> None:
        if not active:
            self._end()
        self.selected = active

    def _end(self) -> None:
        if self.cmd:
            self._finish()
        self.cmd = []

    def _addr(self) -> int:
        return ((self.cmd[1] << 16) | (self.cmd[2] << 8) | self.cmd[3]) % len(self.mem)

    def xfer(self, b: int) -> int:
        self.cmd.append(b)
        op = self.cmd[0]
        n = len(self.cmd)
        if op == 0x9F:
            return self.jedec[n - 2] if 2 <= n <= 1 + len(self.jedec) else 0xFF
        if op == 0x90:
            return (self.jedec[0], self.jedec[2])[(n - 5) % 2] if n >= 5 else 0
        if op in (0x05, 0x35):
            return (0x02 if self.wel else 0x00) if op == 0x05 else 0x00
        if op == 0x03 and n > 4:
            return self.mem[(self._addr() + n - 5) % len(self.mem)]
        if op == 0x0B and n > 5:
            return self.mem[(self._addr() + n - 6) % len(self.mem)]
        if op == 0x02 and n > 4 and self.wel:
            a = self._addr()
            page = a & ~0xFF
            self.mem[page | ((a + n - 5) & 0xFF)] &= b
        return 0xFF

    def _finish(self) -> None:
        op = self.cmd[0]
        if op == 0x06:
            self.wel = True
        elif op == 0x04:
            self.wel = False
        elif op in (0x20, 0x52, 0xD8) and len(self.cmd) >= 4 and self.wel:
            size = {0x20: 0x1000, 0x52: 0x8000, 0xD8: 0x10000}[op]
            a = self._addr() & ~(size - 1)
            self.mem[a:a + size] = b"\xff" * size
            self.wel = False
        elif op in (0xC7, 0x60) and self.wel:
            self.mem[:] = b"\xff" * len(self.mem)
            self.wel = False
        elif op == 0x02:
            self.wel = False

    def save(self) -> None:
        if self.path:
            open(self.path, "wb").write(self.mem)
