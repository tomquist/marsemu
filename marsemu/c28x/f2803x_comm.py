"""F2803x serial peripherals: SCI-A, LIN-A (in SCI mode) and I2C-A.

The UARTs speak the interface of :class:`marsemu.periph.n32g45x.Usart` towards
the device on their pins (``device.on_tx(bytes)``, ``inject(bytes)``,
``baud()``, ``byte_cycles()``), so the FC41D model and UART taps attach to
them unchanged.
"""
from __future__ import annotations

import logging
from collections import deque
from typing import Callable, Dict, List, Optional

from .machine import C28xPeripheral

log = logging.getLogger("emu.f2803x")


class _Uart(C28xPeripheral):
    """Byte timing shared by the SCI and the LIN: received bytes arrive one
    byte time apart, transmitted bytes leave one byte time after the write."""

    RX_DEPTH = 4

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.device = None
        self.rx_q: deque = deque()          # bytes on the wire towards us
        self.rx_fifo: deque = deque()       # received, not yet read
        self.tx_fifo: deque = deque()
        self._rx_ev = None
        self._tx_ev = None
        self.tx_listeners: List[Callable[[bytes], None]] = []
        self.tx_total = 0
        self.rx_total = 0
        self.overruns = 0

    def reset(self):
        super().reset()
        self.rx_q.clear()
        self.rx_fifo.clear()
        self.tx_fifo.clear()
        for ev in (self._rx_ev, self._tx_ev):
            if ev:
                ev.cancel()
        self._rx_ev = self._tx_ev = None

    def baud(self) -> int:
        return 115200

    def byte_cycles(self) -> int:
        return max(1, int(self.m.clock.hz * 10 / max(1, self.baud())))

    # --- receive side -----------------------------------------------------------
    def inject(self, data: bytes) -> None:
        """Bytes sent to the MCU by the device on the pins."""
        self.rx_q.extend(data)
        if self._rx_ev is None:
            self._rx_ev = self.m.clock.after(self.byte_cycles(), self._rx_byte, "uart-rx")

    def _rx_byte(self) -> None:
        self._rx_ev = None
        if not self.rx_q:
            return
        b = self.rx_q.popleft()
        self.rx_total += 1
        if self.rx_enabled():
            if len(self.rx_fifo) >= self.rx_capacity():
                self.overruns += 1
                self.on_overrun()
            else:
                self.rx_fifo.append(b)
            self.rx_update()
        if self.rx_q:
            self._rx_ev = self.m.clock.after(self.byte_cycles(), self._rx_byte, "uart-rx")

    def rx_capacity(self) -> int:
        return 1

    def rx_enabled(self) -> bool:
        return True

    def rx_update(self) -> None:
        pass

    def on_overrun(self) -> None:
        pass

    # --- transmit side ----------------------------------------------------------
    def tx_put(self, b: int) -> None:
        self.tx_fifo.append(b & 0xFF)
        if self._tx_ev is None:
            self._tx_ev = self.m.clock.after(self.byte_cycles(), self._tx_byte, "uart-tx")
        self.tx_update()

    def _tx_byte(self) -> None:
        self._tx_ev = None
        if not self.tx_fifo:
            return
        b = self.tx_fifo.popleft()
        self.tx_total += 1
        data = bytes([b])
        if self.device is not None:
            self.device.on_tx(data)
        for fn in self.tx_listeners:
            fn(data)
        if self.tx_fifo:
            self._tx_ev = self.m.clock.after(self.byte_cycles(), self._tx_byte, "uart-tx")
        self.tx_update()

    def tx_update(self) -> None:
        pass


class Sci(_Uart):
    """SCI-A (0x7050) with its 4-level FIFOs.  Interrupts: PIE 9.1 (RX), 9.2 (TX)."""

    reg_names = {0x0: "SCICCR", 0x1: "SCICTL1", 0x2: "SCIHBAUD", 0x3: "SCILBAUD", 0x4: "SCICTL2", 0x5: "SCIRXST",
                 0x6: "SCIRXEMU", 0x7: "SCIRXBUF", 0x9: "SCITXBUF", 0xA: "SCIFFTX", 0xB: "SCIFFRX", 0xC: "SCIFFCT",
                 0xF: "SCIPRI"}
    pie_rx = (9, 1)
    pie_tx = (9, 2)

    def reset(self):
        super().reset()
        self.regs[0xA] = 0xA000
        self.regs[0xB] = 0x201F

    def baud(self) -> int:
        brr = (self.r(0x2) << 8) | self.r(0x3)
        lsp = self.m.sysctrl.lspclk
        return lsp // ((brr + 1) * 8) if brr else lsp // 16

    def _fifo(self) -> bool:
        return bool(self.r(0xA) & 0x4000)

    def rx_capacity(self):
        return 4 if self._fifo() else 1

    def rx_enabled(self):
        return bool(self.r(0x1) & 0x01) and bool(self.r(0x1) & 0x20)

    def on_overrun(self):
        if self._fifo():
            self.regs[0xB] = self.r(0xB) | 0x8000
        else:
            self.regs[0x5] = self.r(0x5) | 0x08

    def rx_update(self):
        if self._fifo():
            il = self.r(0xB) & 0x1F
            if len(self.rx_fifo) >= max(1, il) and self.r(0xB) & 0x20:
                self.regs[0xB] = self.r(0xB) | 0x80
                self.m.pie.raise_irq(*self.pie_rx)
        elif self.rx_fifo and self.r(0x4) & 0x02:
            self.m.pie.raise_irq(*self.pie_rx)

    def tx_update(self):
        if self._fifo():
            if len(self.tx_fifo) <= (self.r(0xA) & 0x1F) and self.r(0xA) & 0x20:
                self.m.pie.raise_irq(*self.pie_tx)
        elif not self.tx_fifo and self.r(0x4) & 0x01:
            self.m.pie.raise_irq(*self.pie_tx)

    def read(self, off):
        if off == 0x7:
            b = self.rx_fifo.popleft() if self.rx_fifo else 0
            return b
        if off == 0x5:
            v = self.r(0x5) & ~0x40
            if self.rx_fifo and not self._fifo():
                v |= 0x40
            return v
        if off == 0x4:
            v = self.r(0x4) & 0x03
            if len(self.tx_fifo) < (4 if self._fifo() else 1):
                v |= 0x80
            if not self.tx_fifo:
                v |= 0x40
            return v
        if off == 0xA:
            return (self.r(0xA) & 0xE03F) | (len(self.tx_fifo) << 8) | (0x80 if len(self.tx_fifo) <= (self.r(0xA) & 0x1F) else 0)
        if off == 0xB:
            v = self.r(0xB) & 0xE03F
            v |= len(self.rx_fifo) << 8
            if len(self.rx_fifo) >= max(1, self.r(0xB) & 0x1F):
                v |= 0x80
            return v
        return super().read(off)

    def peek(self, off):
        if off == 0x7:
            return self.rx_fifo[0] if self.rx_fifo else 0
        return self.read(off)

    def write(self, off, v):
        if off == 0x9:
            self.tx_put(v)
            return
        if off == 0xA:
            if not v & 0x2000:
                self.tx_fifo.clear()
            self.regs[0xA] = v & ~0x40
            return
        if off == 0xB:
            if not v & 0x2000:
                self.rx_fifo.clear()
            cur = self.r(0xB)
            if v & 0x4000:
                cur &= ~0x8000
            self.regs[0xB] = (cur & 0x8000) | (v & 0x203F)
            return
        if off == 0x1 and not v & 0x20:
            self.rx_fifo.clear()
        super().write(off, v)


class Lin(_Uart):
    """LIN-A (0x6C00) used as a UART (SCI-compatible mode, LINMODE = 0).
    32-bit registers, low half at the lower address.  Interrupts: PIE 9.3
    (level 0) and 9.4 (level 1)."""

    reg_names = {0x00: "SCIGCR0", 0x02: "SCIGCR1", 0x04: "SCIGCR2", 0x06: "SCISETINT", 0x08: "SCICLEARINT",
                 0x0A: "SCISETINTLVL", 0x0C: "SCICLEARINTLVL", 0x0E: "SCIFLR", 0x10: "SCIINTVECT0",
                 0x12: "SCIINTVECT1", 0x14: "SCIFORMAT", 0x16: "BRSR", 0x18: "SCIED", 0x1A: "SCIRD", 0x1C: "SCITD",
                 0x22: "SCIPIO2", 0x30: "LINCOMP", 0x48: "IODFTCTRL"}
    reg_names.update({k + 1: v + "_H" for k, v in list(reg_names.items())})
    #: SCIFLR bits
    TXRDY, RXRDY, TXEMPTY, OE = 1 << 8, 1 << 9, 1 << 11, 1 << 25
    #: registers whose reads have no side effect and do not depend on time
    #: (the firmware polls SCIFLR in its main loop): see f2803x_native
    cacheable = frozenset((0x0E, 0x0F))

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.r32: Dict[int, int] = {}
        self.flr = self.TXRDY | self.TXEMPTY
        self.inten = 0
        self.intlvl = 0

    def reset(self):
        super().reset()
        self.r32 = {}
        self.flr = self.TXRDY | self.TXEMPTY
        self.inten = 0
        self.intlvl = 0

    def baud(self) -> int:
        brsr = self.r32.get(0x16, 0)
        p = brsr & 0xFFFFFF
        mfrac = (brsr >> 24) & 0xF
        clk = self.m.clock.hz // 2
        return int(clk / (16 * (p + 1 + mfrac / 16))) if p or mfrac else clk // 16

    def rx_enabled(self):
        return bool(self.r32.get(0x02, 0) >> 24 & 1)

    def on_overrun(self):
        self.flr |= self.OE
        self._irq()

    def rx_update(self):
        if self.rx_fifo:
            self.flr |= self.RXRDY
        self._irq()

    def tx_update(self):
        if self.tx_fifo:
            self.flr &= ~(self.TXRDY | self.TXEMPTY)
            if len(self.tx_fifo) < 2:
                self.flr |= self.TXRDY
        else:
            self.flr |= self.TXRDY | self.TXEMPTY
        self._irq()

    def _vector(self, lvl: int) -> int:
        """SCIINTVECTx: the highest-priority enabled flag on that line."""
        order = ((0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 0x0C), (9, 0x0B),
                 (13, 0x0D), (14, 0x0E), (24, 0x09), (25, 0x0A), (26, 0x0C))
        bits = {8: 0x0C, 9: 0x0B, 24: 0x09, 25: 0x0A, 26: 0x0C, 0: 0x01}
        # TI's vector numbers: RX = 11, TX = 12, OE = 9 ... (table 15-x)
        active = self.flr & self.inten
        for bit in (0, 25, 26, 24, 9, 8):
            if active >> bit & 1 and ((self.intlvl >> bit) & 1) == lvl:
                return {0: 1, 25: 9, 26: 10, 24: 8, 9: 11, 8: 12}[bit]
        return 0

    def _irq(self):
        if self._vector(0):
            self.m.pie.raise_irq(9, 3)
        if self._vector(1):
            self.m.pie.raise_irq(9, 4)

    def _get(self, off):
        lo = off & ~1
        if lo == 0x0E:
            return self.flr
        if lo == 0x06 or lo == 0x08:
            return self.inten
        if lo == 0x0A or lo == 0x0C:
            return self.intlvl
        if lo == 0x10:
            return self._vector(0)
        if lo == 0x12:
            return self._vector(1)
        if lo == 0x1A:
            return self.rx_fifo[0] if self.rx_fifo else 0
        return self.r32.get(lo, 0)

    def read(self, off):
        lo = off & ~1
        v = self._get(off)
        if lo == 0x1A and not off & 1:
            if self.rx_fifo:
                self.rx_fifo.popleft()
            if not self.rx_fifo:
                self.flr &= ~self.RXRDY
        if lo == 0x10 or lo == 0x12:
            # reading the vector clears the flag of RX/TX? (RX is cleared by reading SCIRD)
            pass
        return (v >> 16) & 0xFFFF if off & 1 else v & 0xFFFF

    def peek(self, off):
        v = self._get(off)
        return (v >> 16) & 0xFFFF if off & 1 else v & 0xFFFF

    def write(self, off, v):
        lo = off & ~1
        cur = self.r32.get(lo, 0)
        if lo in (0x06, 0x08, 0x0A, 0x0C, 0x0E):
            cur = 0
        word = (v << 16) if off & 1 else v
        if lo == 0x06:
            self.inten |= word
            self._irq()
            return
        if lo == 0x08:
            self.inten &= ~word
            return
        if lo == 0x0A:
            self.intlvl |= word
            return
        if lo == 0x0C:
            self.intlvl &= ~word
            return
        if lo == 0x0E:   # write 1 to clear
            self.flr &= ~word | (self.TXRDY | self.TXEMPTY | self.RXRDY)
            return
        if lo == 0x1C:
            if not off & 1:
                self.tx_put(v & 0xFF)
            return
        new = (cur & 0xFFFF) | (v << 16) if off & 1 else (cur & 0xFFFF0000) | v
        self.r32[lo] = new
        if lo == 0x02 and not new & 0x80:   # SWnRST = 0: hold in reset
            self.rx_fifo.clear()
            self.flr = self.TXRDY | self.TXEMPTY


class I2c(C28xPeripheral):
    """I2C-A (0x7900) as a master with its 4-byte FIFOs.  Slaves are
    :class:`marsemu.periph.n32g45x.I2cDevice` objects in :attr:`devices`.
    A transfer takes about one byte time (~100 kbit/s) per byte."""

    reg_names = {0x0: "I2COAR", 0x1: "I2CIER", 0x2: "I2CSTR", 0x3: "I2CCLKL", 0x4: "I2CCLKH", 0x5: "I2CCNT",
                 0x6: "I2CDRR", 0x7: "I2CSAR", 0x8: "I2CDXR", 0x9: "I2CMDR", 0xA: "I2CISRC", 0xB: "I2CEMDR",
                 0xC: "I2CPSC", 0x20: "I2CFFTX", 0x21: "I2CFFRX"}
    # I2CSTR / I2CIER bits
    ARBL, NACK, ARDY, RRDY, XRDY, SCD, XSMT, BB = 1, 2, 4, 8, 16, 32, 1 << 10, 1 << 12
    # I2CMDR bits
    STT, STP, MST, TRX, IRS = 1 << 13, 1 << 11, 1 << 10, 1 << 9, 1 << 5

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.devices: Dict[int, object] = {}
        self.txf: deque = deque()
        self.rxf: deque = deque()
        self.str = self.XSMT | self.XRDY
        self.cnt_left = 0
        self._ev = None
        self.cur = None
        self.transfers = 0

    def reset(self):
        super().reset()
        self.txf.clear()
        self.rxf.clear()
        self.str = self.XSMT | self.XRDY
        if self._ev:
            self._ev.cancel()
        self._ev = None
        self.cur = None

    def _byte_cycles(self) -> int:
        return self.m.clock.hz // 11000   # 9 bit times at ~100 kbit/s

    def _fifo(self):
        return bool(self.r(0x20) & 0x4000)

    def _status(self):
        v = self.str & ~(self.RRDY | self.XRDY)
        if self.rxf:
            v |= self.RRDY
        if self._fifo() or not self.txf:
            v |= self.XRDY
        return v

    def _irq(self):
        if self._status() & self.r(0x1) & 0x3F:
            self.m.pie.raise_irq(8, 1)

    def read(self, off):
        if off == 0x2:
            return self._status()
        if off == 0x6:
            return self.rxf.popleft() if self.rxf else 0
        if off == 0x5:
            return self.cnt_left if self._ev is not None else self.r(0x5)
        if off == 0x9:
            return self.r(0x9)
        if off == 0xA:
            st = self._status() & self.r(0x1)
            for code, bit in ((1, 1), (2, 2), (3, 4), (4, 8), (5, 16), (6, 32)):
                if st & bit:
                    if bit in (self.ARDY, self.SCD, self.NACK, self.ARBL):
                        self.str &= ~bit
                    return code
            return 0
        if off == 0x20:
            return (self.r(0x20) & 0xE03F) | (len(self.txf) << 8)
        if off == 0x21:
            il = self.r(0x21) & 0x1F
            return (self.r(0x21) & 0xE03F) | (len(self.rxf) << 8) | (0x80 if self.rxf and len(self.rxf) >= il else 0)
        return super().read(off)

    def peek(self, off):
        if off == 0x6:
            return self.rxf[0] if self.rxf else 0
        if off == 0xA:
            return 0
        return self.read(off)

    def write(self, off, v):
        if off == 0x8:
            self.txf.append(v & 0xFF)
            return
        if off == 0x2:   # write 1 to clear
            self.str &= ~(v & (self.ARBL | self.NACK | self.ARDY | self.SCD))
            return
        if off == 0x20:
            if not v & 0x2000:
                self.txf.clear()
            self.regs[0x20] = v & ~0x40
            return
        if off == 0x21:
            if not v & 0x2000:
                self.rxf.clear()
            self.regs[0x21] = v & ~0x40
            return
        if off == 0x9:
            self.regs[0x9] = v
            if not v & self.IRS:   # module held in reset
                if self._ev:
                    self._ev.cancel()
                self._ev = None
                self.txf.clear()
                self.rxf.clear()
                self.str = self.XSMT | self.XRDY
                if self.cur is not None:
                    self.cur.stop()
                self.cur = None
                return
            if v & self.STT and self._ev is None:
                self._start(v)
            elif v & self.STP and self._ev is None and not v & self.STT:
                self._stop()
            return
        super().write(off, v)

    def _start(self, mdr):
        sar = self.r(0x7) & 0x7F
        read = not mdr & self.TRX
        n = self.r(0x5) or 0x10000
        self.cnt_left = n
        self.str |= self.BB
        self.str &= ~(self.NACK | self.SCD | self.ARDY)
        dev = self.devices.get(sar)
        if self.cur is not None and self.cur is not dev:
            self.cur.stop()
        ack = dev is not None and dev.start(read)
        self.cur = dev if ack else None
        self.transfers += 1
        cyc = self._byte_cycles() * (1 + (min(n, 64) if ack else 0))
        self._ev = self.m.clock.after(cyc, lambda: self._done(ack, read, n), "i2c")

    def _done(self, ack, read, n):
        self._ev = None
        mdr = self.r(0x9)
        if not ack:
            self.str |= self.NACK
        else:
            dev = self.cur
            for _ in range(n if n < 0x10000 else 0):
                if read:
                    self.rxf.append(dev.read() & 0xFF)
                else:
                    dev.write(self.txf.popleft() if self.txf else 0)
            self.cnt_left = 0
        self.regs[0x9] = mdr & ~self.STT
        if mdr & self.STP:
            self._stop()
        else:
            self.str |= self.ARDY
        self._irq()
        if self.r(0x21) & 0x20 and self.rxf and len(self.rxf) >= (self.r(0x21) & 0x1F):
            self.regs[0x21] = self.r(0x21) | 0x80
            self.m.pie.raise_irq(8, 2)

    def _stop(self):
        if self.cur is not None:
            self.cur.stop()
        self.cur = None
        self.str &= ~self.BB
        self.str |= self.SCD
        self.regs[0x9] = self.r(0x9) & ~(self.STP | self.STT | self.MST)
        self._irq()
