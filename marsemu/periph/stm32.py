"""STM32 peripheral models for the non-Nations boards.

* STM32G4 (Cortex-M4F): original Venus (HMG-x) control board.
* STM32F0-class (Cortex-M0): CT002/CT003 smart meters.

Both use the "new" STM32 peripheral generation (GPIO MODER/IDR/BSRR, USART
with ISR/ICR/RDR/TDR, I2C with TIMINGR/NBYTES, ADC with ADEN/ADRDY ...).
Timers, IWDG, SPI, DMA channels and EXTI keep the classic layout, so those
models are shared with :mod:`marsemu.periph.n32g45x`.
"""
from __future__ import annotations

import ctypes
import json
import logging
import os
from typing import Dict, Optional

from . import n32g45x as n32
from .n32g45x import Base, I2cDevice, RegI2cDevice
from .hrtim import Hrtim, HrtimUnit

log = logging.getLogger("emu.periph")


def _db(name: str) -> dict:
    path = os.path.join(os.path.dirname(__file__), "..", "data", name)
    return json.load(open(path))["peripherals"] if os.path.exists(path) else {}


# --------------------------------------------------------------------------- RCC
class RccG4(Base):
    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x00000500, 0x04: 0x40000000, 0x08: 0x5, 0x0C: 0x1000, 0x94: 0x0C000000,
                          0x98: 0})

    def write32(self, off, v):
        if off == 0x00:  # CR: HSI(8->10) HSE(16->17) PLL(24->25)
            v &= ~((1 << 10) | (1 << 17) | (1 << 25))
            if v & (1 << 8):
                v |= 1 << 10
            if v & (1 << 16):
                v |= 1 << 17
            if v & (1 << 24):
                v |= 1 << 25
        elif off == 0x08:  # CFGR: SWS follows SW
            v = (v & ~0xC) | ((v & 3) << 2)
        elif off == 0x90:  # BDCR: LSERDY
            v = (v & ~2) | (2 if v & 1 else 0)
        elif off == 0x94:  # CSR: LSIRDY, RMVF clears reset flags
            keep = self.regs.get(0x94, 0) & 0xFE000000
            if v & (1 << 23):
                keep = 0
            v = (v & 1) | (2 if v & 1 else 0) | keep
        elif off == 0x98:  # CRRCR: HSI48RDY
            v = (v & ~2) | (2 if v & 1 else 0)
        elif off == 0x1C:  # CICR
            return
        self.regs[off] = v

    def note_reset(self, why: str) -> None:
        flag = {"IWDG": 29, "WWDG": 30, "SYSRESETREQ": 28}.get(why)
        if flag:
            self.regs[0x94] = self.regs.get(0x94, 0) | (1 << flag)


class RccF0(n32.Rcc):
    """STM32F0: like F1 plus CR2 (HSI14/HSI48)."""

    def write32(self, off, v):
        if off == 0x34:
            v = (v & ~((1 << 1) | (1 << 17))) | ((v & 1) << 1) | ((v & (1 << 16)) << 1)
            self.regs[off] = v
            return
        super().write32(off, v)


class PwrG4(Base):
    def read32(self, off):
        if off == 0x14:  # SR2: VOSF=0, REGLPF=0
            return 0
        return self.regs.get(off, 0)


# --------------------------------------------------------------------------- FLASH
class FlashG4(Base):
    """G4 flash: unlock via KEYR, page erase via PER/PNB/BKER/STRT."""

    def __init__(self, machine, name, base, page=0x800, bank_size=0x40000):
        self.page = page
        self.bank_size = bank_size
        super().__init__(machine, name, base)

    def reset(self):
        super().reset()
        self.regs.update({0x00: 0x600, 0x14: 0xC0000000, 0x20: 0xFFEFF8AA})
        self.stage = 0

    def write32(self, off, v):
        if off == 0x08:  # KEYR
            if v == 0x45670123:
                self.stage = 1
            elif v == 0xCDEF89AB and self.stage == 1:
                self.regs[0x14] = self.regs.get(0x14, 0) & ~(1 << 31)
            return
        if off == 0x10:  # SR: w1c
            self.regs[0x10] = self.regs.get(0x10, 0) & ~v
            return
        if off == 0x14:
            if v & (1 << 16):  # STRT
                if v & (1 << 2):
                    self._erase(self.m.flash_base, self.m.flash_size)
                elif v & (1 << 1):
                    pnb = (v >> 3) & 0x7F
                    bank = (v >> 11) & 1
                    self._erase(self.m.flash_base + bank * self.bank_size + pnb * self.page, self.page)
                v &= ~(1 << 16)
                self.regs[0x10] = self.regs.get(0x10, 0) | 1  # EOP
            self.regs[0x14] = v
            return
        self.regs[off] = v

    def _erase(self, addr, size):
        off = addr - self.m.flash_base
        if 0 <= off < self.m.flash_size:
            ctypes.memset(ctypes.addressof(self.m.flash) + off, 0xFF, min(size, self.m.flash_size - off))
            self.m.uc.ctl_remove_cache(addr, addr + size)


# --------------------------------------------------------------------------- GPIO
class GpioV2(n32.Gpio):
    """MODER/OTYPER/OSPEEDR/PUPDR/IDR/ODR/BSRR/LCKR/AFR/BRR."""

    def reset(self):
        Base.reset(self)
        if self.port == "A":
            self.regs[0x00] = 0xABFFFFFF
        elif self.port == "B":
            self.regs[0x00] = 0xFFFFFEBF
        else:
            self.regs[0x00] = 0xFFFFFFFF

    def _idr(self) -> int:
        moder = self.regs.get(0x00, 0)
        pupdr = self.regs.get(0x0C, 0)
        odr = self.regs.get(0x14, 0)
        key = (moder, pupdr, odr, tuple(self.inputs.items()), self.float_level)
        if key == getattr(self, "_idr_key", None):
            return self._idr_val
        self._idr_key = key
        self._idr_val = v = self._compute_idr(moder, pupdr, odr)
        return v

    def _compute_idr(self, moder, pupdr, odr) -> int:
        v = 0
        for pin in range(16):
            if pin in self.inputs:
                bit = self.inputs[pin]
            else:
                mode = (moder >> (2 * pin)) & 3
                pull = (pupdr >> (2 * pin)) & 3
                if mode == 1 or mode == 2:
                    bit = (odr >> pin) & 1
                elif mode == 3:
                    bit = 0
                elif pull == 1:
                    bit = 1
                elif pull == 2:
                    bit = 0
                else:
                    bit = self.float_level
            v |= (bit & 1) << pin
        return v

    def pin_out(self, pin):
        return (self.regs.get(0x14, 0) >> pin) & 1

    def read32(self, off):
        if off == 0x10:
            return self._idr()
        return self.regs.get(off, 0)

    def write32(self, off, v):
        old = self.regs.get(0x14, 0)
        if off == 0x18:  # BSRR
            self.regs[0x14] = (old | (v & 0xFFFF)) & ~(v >> 16)
        elif off == 0x28:  # BRR
            self.regs[0x14] = old & ~(v & 0xFFFF)
        else:
            self.regs[off] = v
        new = self.regs.get(0x14, 0)
        if new != old and self.watchers:
            for pin in range(16):
                if (new ^ old) >> pin & 1:
                    for w in self.watchers:
                        w(pin, (new >> pin) & 1)


# --------------------------------------------------------------------------- USART
class UsartV2(n32.Usart):
    O_CR1, O_CR3, O_BRR, O_STS, O_RDR, O_TDR = 0x00, 0x08, 0x0C, 0x1C, 0x24, 0x28
    CR1_UE, CR1_RE = 1 << 0, 1 << 2
    STS_RESET = 0xC0 | (1 << 21) | (1 << 22)  # TXE TC TEACK REACK
    O_ICR, O_RQR = 0x20, 0x18

    def __init__(self, machine, name, base):
        super().__init__(machine, name, base)
        self.pclk_div = 1

    def baud(self) -> int:
        brr = self.regs.get(self.O_BRR, 0)
        if not brr:
            return 115200
        if self.name.startswith("LPUART"):
            return max(300, int(256 * self.m.clock.hz / brr))
        return max(1200, int(self.m.clock.hz / brr))

    RXFT, TXFT = 1 << 26, 1 << 27

    def _fifo_flags(self, v):
        # FIFO mode: our receive "FIFO" holds one byte, so the RX threshold
        # flag follows RXFNE and the TX threshold flag follows TXFNF
        if self.regs.get(self.O_CR1, 0) & (1 << 29):
            v &= ~(self.RXFT | self.TXFT)
            if v & self.RXNE:
                v |= self.RXFT
            if v & self.TXE:
                v |= self.TXFT
        return v

    def read32(self, off):
        if off == self.O_STS:
            v = self._fifo_flags(self.regs.get(off, 0))
            c1 = self.regs.get(self.O_CR1, 0)
            v = (v & ~((1 << 21) | (1 << 22))) | ((c1 & 8) << 18) | ((c1 & 4) << 20)
            return v
        return super().read32(off)

    def write32(self, off, v):
        if off == self.O_ICR:
            self._clr(v & 0x1B5F)
            self.update_irq()
            return
        if off == self.O_RQR:
            if v & (1 << 3):  # RXFRQ
                self._clr(self.RXNE)
            return
        if off == self.O_STS:
            return
        super().write32(off, v)

    def level(self):
        sts = self._fifo_flags(self.regs.get(self.O_STS, 0))
        c1 = self.regs.get(self.O_CR1, 0)
        c3 = self.regs.get(self.O_CR3, 0)
        on = bool((sts & 0x80 and c1 & 0x80) or (sts & 0x40 and c1 & 0x40) or
                  (sts & 0x20 and c1 & 0x20) or (sts & 0x10 and c1 & 0x10) or (sts & 0x08 and c1 & 0x20) or
                  (sts & self.RXFT and c3 & (1 << 28)) or (sts & self.TXFT and c3 & (1 << 23)) or
                  (sts & 0x08 and c3 & 1))
        if self.name in self.irqs:
            irq = self.name
        else:
            cands = [n for n in self.irqs if "UART" in n.upper()] or list(self.irqs) or [self.name]
            irq = cands[0]
        return {irq: on}


# --------------------------------------------------------------------------- I2C (v2)
class I2cV2(Base):
    def __init__(self, machine, name, base):
        self.devices: Dict[int, I2cDevice] = {}
        self.cur: Optional[I2cDevice] = None
        self.reading = False
        self.left = 0
        self.unknown_seen: set = set()
        self.default_ack = True
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs[0x18] = 0x1  # TXE

    def level(self):
        isr = self.regs.get(0x18, 0)
        cr1 = self.regs.get(0x00, 0)
        ev = bool((cr1 & 2 and isr & 2) or (cr1 & 4 and isr & 4) or (cr1 & 0x40 and isr & 0xC0) or
                  (cr1 & 0x20 and isr & 0x20) or (cr1 & 0x10 and isr & 0x10))
        out = {}
        for n in self.irqs:
            out[n] = ev if n.endswith("EV") or n == self.name else False
        return out

    def _isr(self, set_=0, clr=0):
        self.regs[0x18] = (self.regs.get(0x18, 0) | set_) & ~clr

    def write32(self, off, v):
        if off == 0x04:  # CR2
            self.regs[0x04] = v & ~((1 << 13) | (1 << 14))
            if v & (1 << 13):
                self._start(v)
            if v & (1 << 14):
                self._stop()
            self.update_irq()
            return
        if off == 0x1C:  # ICR
            self._isr(clr=v & 0x3F38)
            self.update_irq()
            return
        if off == 0x28:  # TXDR
            if self.cur is not None and not self.reading:
                self.cur.write(v & 0xFF)
            self._byte_done()
            return
        if off == 0x00 and not v & 1:
            self.regs[0x18] = 1
            self.cur = None
        self.regs[off] = v
        self.update_irq()

    def read32(self, off):
        if off == 0x24:
            v = self.regs.get(0x24, 0)
            self._isr(clr=4)
            self._byte_done()
            return v
        return self.regs.get(off, 0)

    def _start(self, cr2):
        addr7 = (cr2 >> 1) & 0x7F
        rd = bool(cr2 & (1 << 10))
        self.left = (cr2 >> 16) & 0xFF
        dev = self.devices.get(addr7)
        if dev is None and self.default_ack:
            if addr7 not in self.unknown_seen:
                self.unknown_seen.add(addr7)
                log.info("%s: transaction with unmodelled device 0x%02x", self.name, addr7)
            dev = self.devices.setdefault(addr7, RegI2cDevice(addr7))
        self._isr(set_=1 << 15, clr=0xFE)
        if dev is None or not dev.start(rd):
            self._isr(set_=0x10 | 0x20)  # NACKF, STOPF
            self._isr(clr=1 << 15)
            return
        self.cur = dev
        self.reading = rd
        self._next()

    def _next(self):
        if self.left == 0:
            self._complete()
            return
        if self.reading:
            self.regs[0x24] = self.cur.read() & 0xFF
            self._isr(set_=4)
        else:
            self._isr(set_=2 | 1)

    def _byte_done(self):
        if self.cur is None:
            return
        self.left = max(0, self.left - 1)
        self._isr(clr=2)
        self._next()
        self.update_irq()

    def _complete(self):
        cr2 = self.regs.get(0x04, 0)
        if cr2 & (1 << 24):  # RELOAD
            self._isr(set_=0x80)
        elif cr2 & (1 << 25):  # AUTOEND
            self._stop()
        else:
            self._isr(set_=0x40)  # TC

    def _stop(self):
        if self.cur:
            self.cur.stop()
        self.cur = None
        self._isr(set_=0x20 | 1, clr=(1 << 15) | 2 | 4)


# --------------------------------------------------------------------------- ADC
class AdcV2(Base):
    """G4/F0 ADC: ADEN/ADRDY handshake, ADCAL, software start, DMA."""

    def __init__(self, machine, name, base, f0: bool = False):
        self.values: Dict[int, int] = {}
        self.default = 2048
        self.dma = None
        self.f0 = f0
        self.provider = None
        self.ev = None
        super().__init__(machine, name, base, irqs={})
        self.size = 0x100 if not f0 else 0x400

    def sample(self, ch):
        if self.provider:
            v = self.provider(self.name, ch)
            if v is not None:
                return v & 0xFFF
        return self.values.get(ch, self.default)

    def sequence(self):
        key = (self.regs.get(0x28, 0), self.regs.get(0x30, 0), self.regs.get(0x34, 0),
               self.regs.get(0x38, 0), self.regs.get(0x3C, 0))
        if key != getattr(self, "_seq_key", None):
            self._seq_key, self._seq = key, self._sequence()
        return self._seq

    def _sequence(self):
        if self.f0:
            sel = self.regs.get(0x28, 0)
            return [c for c in range(19) if sel >> c & 1] or [0]
        sq1 = self.regs.get(0x30, 0)
        n = (sq1 & 0xF) + 1
        seq = []
        for i in range(n):
            if i < 4:
                seq.append((sq1 >> (6 + 6 * i)) & 0x1F)
            else:
                reg = 0x34 + 4 * ((i - 4) // 5)
                seq.append((self.regs.get(reg, 0) >> (6 * ((i - 4) % 5))) & 0x1F)
        return seq

    def read32(self, off):
        if off == 0x40:
            self.regs[0x00] = self.regs.get(0x00, 0) & ~4
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x00:  # ISR w1c
            self.regs[0x00] = self.regs.get(0x00, 0) & ~v
            return
        if off == 0x08:  # CR
            if v & (1 << 31):  # ADCAL completes instantly
                v &= ~(1 << 31)
            if v & 1:
                self.regs[0x00] = self.regs.get(0x00, 0) | 1  # ADRDY
            if v & 2:  # ADDIS
                v &= ~3
            start = v & 4
            if v & 0x10:  # ADSTP: disarms a hardware-triggered sequence
                start = 0
            v &= ~(4 | 0x10)
            hrtim = getattr(self.m, "hrtim", None)
            hw = not self.f0 and (self._cfgr() >> 10) & 3 and hrtim is not None and hrtim.active
            if start and hw:
                v |= 4  # ADSTART stays set: armed, converts on each trigger
            self.regs[0x08] = v
            if start and not hw:
                self._convert()
            return
        self.regs[off] = v

    #: regular-group EXTSEL values of the HRTIM ADC triggers (RM0440 tables 163/164)
    HRTIM_EXTSEL = {"ADC1": {21: 1, 22: 3}, "ADC2": {21: 1, 22: 3},
                    "ADC3": {21: 1, 22: 2}, "ADC4": {21: 1, 22: 2}, "ADC5": {21: 1, 22: 2}}

    def hrtim_trigger(self, trg: int) -> None:
        """HRTIM ADC trigger ``trg`` fired: convert if armed for it."""
        if not self.regs.get(0x08, 0) & 4:
            return
        cfgr = self._cfgr()
        if not (cfgr >> 10) & 3 or self.HRTIM_EXTSEL.get(self.name, {}).get((cfgr >> 5) & 0x1F) != trg:
            return
        self._convert()

    def _cfgr(self):
        return self.regs.get(0x0C, 0)

    def _convert(self):
        dma = self._cfgr() & 1
        vals = [self.sample(ch) for ch in self.sequence()]
        if vals:
            self.regs[0x40] = vals[-1]
            if dma and self.dma is not None:
                self.dma.push_many(vals)
        self.regs[0x00] = self.regs.get(0x00, 0) | 4 | 8
        if self._cfgr() & (1 << 13) and self.ev is None:  # continuous
            def again():
                self.ev = None
                if self.regs.get(0x08, 0) & 1:
                    self._convert()
            self.ev = self.m.clock.after_ms(1, again, self.name)

    def dma_to_memory(self, ch):
        self.dma = ch


# --------------------------------------------------------------------------- RTC (G4)
class RtcG4(n32.Rtc):
    """G4 RTC: TR/DR/SSR/ICSR/PRER/WUTR/CR/.../SR/SCR."""

    def reset(self):
        Base.reset(self)
        self.regs.update({0x0C: 0x07 | 0x20 | 0x10, 0x10: 0x007F00FF, 0x14: 0xFFFF})

    def level(self):
        sr = self.regs.get(0x50, 0)
        cr = self.regs.get(0x18, 0)
        return {"RTC_WKUP": bool(sr & 4 and cr & 0x4000), "RTC_ALARM": bool(sr & 3 and cr & 0x3000)}

    def read32(self, off):
        if off in (0x00, 0x04):
            return super().read32(off)
        if off == 0x08:
            return 0xFF - int((self.m.clock.seconds % 1) * 256)
        if off == 0x0C:
            return self.regs.get(0x0C, 0) | 0x20 | 0x04 | 0x10
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x0C:
            self.regs[0x0C] = (v & 0x80) | (0x40 if v & 0x80 else 0) | 0x37
            return
        if off == 0x5C:  # SCR
            self.regs[0x50] = self.regs.get(0x50, 0) & ~v
            self.update_irq()
            return
        if off in (0x00, 0x04):
            super().write32(off, v)
            return
        self.regs[off] = v
        if off in (0x18, 0x14):
            self._wkup_g4()

    def _wkup_g4(self):
        if self.wkup_ev:
            self.wkup_ev.cancel()
            self.wkup_ev = None
        cr = self.regs.get(0x18, 0)
        if cr & 0x400:
            sel = cr & 7
            n = (self.regs.get(0x14, 0) & 0xFFFF) + 1
            secs = max(n if sel & 4 else n * (16 >> sel) / 32768.0, 0.001)

            def fire():
                self.regs[0x50] = self.regs.get(0x50, 0) | 4
                self.m.bus.get("EXTI").trigger(20)
                self.update_irq()
                self._wkup_g4()
            self.wkup_ev = self.m.clock.after(int(secs * self.m.clock.hz), fire, "rtc-wkup")


# --------------------------------------------------------------------------- CORDIC (G4)
class Cordic(Base):
    """G4 CORDIC coprocessor: cos, sin, phase, modulus, arctan, sqrt (and the
    hyperbolic functions) on q1.31 or packed q1.15 arguments, computed in
    floating point.  CSR 0x00, WDATA 0x04, RDATA 0x08."""

    def reset(self):
        super().reset()
        self.regs[0x00] = 0x50  # PRECISION 5
        self.args = [0.0, 1.0]  # the second argument resets to 0x7FFFFFFF (~1)
        self.nargs_seen = 0
        self.results = []

    def _csr(self):
        return self.regs.get(0x00, 0)

    @staticmethod
    def _q31(v):
        v &= 0xFFFFFFFF
        return (v - (1 << 32) if v & 0x80000000 else v) / 2147483648.0

    @staticmethod
    def _q15(v):
        v &= 0xFFFF
        return (v - (1 << 16) if v & 0x8000 else v) / 32768.0

    @staticmethod
    def _to_q(x, bits):
        full = 1 << (bits - 1)
        return max(-full, min(full - 1, int(round(x * full)))) & ((1 << bits) - 1)

    def read32(self, off):
        if off == 0x00:
            return self._csr() | (0x80000000 if self.results else 0)
        if off == 0x08:
            return self.results.pop(0) if self.results else 0
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x00:
            self.regs[0x00] = v & 0x7FFFFFFF
            self.nargs_seen = 0
            return
        if off != 0x04:
            self.regs[off] = v
            return
        csr = self._csr()
        if csr >> 22 & 1:  # ARGSIZE: two q1.15 in one word
            self.args = [self._q15(v), self._q15(v >> 16)]
            self._compute()
            return
        self.args[self.nargs_seen] = self._q31(v)
        self.nargs_seen += 1
        if self.nargs_seen >= (2 if csr >> 20 & 1 else 1):
            self.nargs_seen = 0
            self._compute()

    def _compute(self):
        import math
        csr = self._csr()
        func, scale = csr & 0xF, (csr >> 8) & 7
        a, b = self.args
        r1 = r2 = 0.0
        try:
            if func in (0, 1):            # cos / sin of a*pi, times modulus b
                c, s_ = b * math.cos(a * math.pi), b * math.sin(a * math.pi)
                r1, r2 = (c, s_) if func == 0 else (s_, c)
            elif func in (2, 3):          # phase / modulus of (a, b)
                ph, mod = math.atan2(b, a) / math.pi, math.hypot(a, b)
                r1, r2 = (ph, mod) if func == 2 else (mod, ph)
            elif func == 4:               # arctan(a * 2^n) / pi * 2^-n
                r1 = math.atan(a * (1 << scale)) / math.pi / (1 << scale)
            elif func in (5, 6):          # cosh / sinh of a * 2^n, results * 2^-n
                x = a * (1 << scale)
                ch, sh = math.cosh(x) / (1 << scale), math.sinh(x) / (1 << scale)
                r1, r2 = (ch, sh) if func == 5 else (sh, ch)
            elif func == 7:
                r1 = math.atanh(a * (1 << scale)) / (1 << scale)
            elif func == 8:               # ln(a * 2^n) * 2^-(n+1)
                r1 = math.log(a * (1 << scale)) / (1 << (scale + 1))
            elif func == 9:               # sqrt(a * 2^n) * 2^-n
                r1 = math.sqrt(max(0.0, a * (1 << scale))) / (1 << scale)
        except (ValueError, OverflowError):
            r1 = r2 = 0.0
        if csr >> 21 & 1:  # RESSIZE: two q1.15 in one word
            self.results = [self._to_q(r1, 16) | self._to_q(r2, 16) << 16]
        else:
            self.results = [self._to_q(r1, 32)] + ([self._to_q(r2, 32)] if csr >> 19 & 1 else [])


# --------------------------------------------------------------------------- CRC (v2)
class CrcV2(Base):
    def reset(self):
        super().reset()
        self.regs.update({0x10: 0xFFFFFFFF, 0x14: 0x04C11DB7})
        self.crc = 0xFFFFFFFF

    def _cfg(self):
        cr = self.regs.get(0x08, 0)
        width = {0: 32, 1: 16, 2: 8, 3: 7}[(cr >> 3) & 3]
        return width, (cr >> 5) & 3, bool(cr & 0x80)

    def _feed(self, value, nbytes):
        width, rev_in, _ = self._cfg()
        poly = self.regs.get(0x14, 0x04C11DB7) & ((1 << width) - 1)
        data = value & ((1 << (8 * nbytes)) - 1)
        if rev_in == 1:  # by byte
            data = int.from_bytes(bytes(int(f"{b:08b}"[::-1], 2) for b in data.to_bytes(nbytes, "little")), "little")
        elif rev_in == 2:  # by half-word
            data = int(f"{data:0{8 * nbytes}b}"[::-1], 2) if nbytes == 1 else data
        elif rev_in == 3:  # by word
            data = int(f"{data:0{8 * nbytes}b}"[::-1], 2)
        crc = self.crc
        top = 1 << (width - 1)
        mask = (1 << width) - 1
        for i in range(8 * nbytes - 1, -1, -1):
            bit = (data >> i) & 1
            fb = bool(crc & top) ^ bit
            crc = ((crc << 1) & mask) ^ (poly if fb else 0)
        self.crc = crc

    def write(self, off, size, value):
        if off < 4:
            self._feed(value, size)
            return
        super().write(off, size, value)

    def write32(self, off, v):
        if off == 0x00:
            self._feed(v, 4)
            return
        if off == 0x08:
            if v & 1:
                self.crc = self.regs.get(0x10, 0xFFFFFFFF)
                v &= ~1
        if off == 0x10:
            self.crc = v
        self.regs[off] = v

    def read32(self, off):
        if off == 0x00:
            width, _, rev_out = self._cfg()
            if rev_out:
                return int(f"{self.crc:0{width}b}"[::-1], 2)
            return self.crc
        return self.regs.get(off, 0)


# --------------------------------------------------------------------------- FDCAN
# Message RAM layout of one STM32G4 FDCAN instance (0x350 bytes, fixed).
_STD_FLT, _EXT_FLT, _RXF0, _RXF1, _TXEF, _TXB = 0x000, 0x070, 0x0B0, 0x188, 0x260, 0x278
_ELEM = 0x48
_DLC_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


class Fdcan(Base):
    """STM32G4 FDCAN: classic-CAN frames, TX FIFO, RX FIFO 0/1 with the filter
    lists and global filter, interrupt lines 0/1.  Frames leave through
    ``tx_listeners`` and arrive through :meth:`inject`."""

    def __init__(self, machine, name, base, ram_base=0x4000A400, ram_index=0):
        self.ram = ram_base + ram_index * 0x350
        self.tx_listeners = []
        self.fifo = ([], [])  # frames waiting in RX FIFO 0/1 (element contents)
        super().__init__(machine, name, base)
        self.register_levels()

    def reset(self):
        super().reset()
        self.regs.update({0x18: 0x1, 0xC4: 0x3, 0x84: 0x1FFFFFFF})  # CCCR.INIT, TXFQS free level, XIDAM
        if hasattr(self, "fifo"):
            self.fifo[0].clear()
            self.fifo[1].clear()

    # -- message RAM helpers
    def _rd(self, a):
        return self.m.bus.read(self.ram + a, 4)

    def _wr(self, a, v):
        self.m.bus.write(self.ram + a, v & 0xFFFFFFFF, 4)

    def read32(self, off):
        if off in (0x90, 0x98):  # RXFnS
            q = self.fifo[(off - 0x90) // 8]
            n = min(len(q), 3)
            st = n | (0x1000000 if n >= 3 else 0)
            return st | (self.regs.get(off, 0) & 0x2000000)
        return self.regs.get(off, 0)

    def write32(self, off, v):
        if off == 0x18:  # CCCR: INIT/CCE read back
            self.regs[off] = v
            return
        if off == 0xCC:  # TXBAR: send buffers
            for i in range(3):
                if v & (1 << i):
                    self._send(_TXB + i * _ELEM)
            self.regs[0xD8] = self.regs.get(0xD8, 0) | v  # TXBTO
            self.regs[0x50] = self.regs.get(0x50, 0) | (1 << 7) | (1 << 9)  # IR.TC, IR.TFE
            self.update_irq()
            return
        if off == 0x50:  # IR w1c
            self.regs[0x50] = self.regs.get(0x50, 0) & ~v
            self.update_irq()
            return
        if off in (0x94, 0x9C):  # RXFnA: acknowledge up to the given index
            q = self.fifo[(off - 0x94) // 8]
            if q:
                q.pop(0)
            self._load(0 if off == 0x94 else 1)
            return
        self.regs[off] = v
        if off in (0x54, 0x58, 0x5C):
            self.update_irq()

    def _send(self, a):
        t0, t1 = self._rd(a), self._rd(a + 4)
        ln = _DLC_LEN[(t1 >> 16) & 0xF]
        data = b"".join(self._rd(a + 8 + 4 * k).to_bytes(4, "little") for k in range((ln + 3) // 4))[:ln]
        xtd = bool(t0 & (1 << 30))
        ident = t0 & 0x1FFFFFFF if xtd else (t0 >> 18) & 0x7FF
        for l in self.tx_listeners:
            l(ident, data, xtd)

    # -- reception
    def _match(self, ident: int, xtd: bool):
        """Return the target FIFO (0/1) or None, applying filters like the hardware."""
        gfc = self.regs.get(0x80, 0)
        if xtd:
            n, base = (gfc >> 24) & 0xF, _EXT_FLT
            mask = self.regs.get(0x84, 0x1FFFFFFF) & 0x1FFFFFFF
            ident_m = ident & mask
        else:
            n, base = (gfc >> 16) & 0x1F, _STD_FLT
            ident_m = ident
        for i in range(n):
            if xtd:
                f0, f1 = self._rd(base + 8 * i), self._rd(base + 8 * i + 4)
                fec, id1, ft, id2 = f0 >> 29, f0 & 0x1FFFFFFF, f1 >> 30, f1 & 0x1FFFFFFF
            else:
                f = self._rd(base + 4 * i)
                ft, fec, id1, id2 = f >> 30, (f >> 27) & 7, (f >> 16) & 0x7FF, f & 0x7FF
            if fec == 0:
                continue
            if ft == 0:
                hit = id1 <= ident_m <= id2
            elif ft == 1:
                hit = ident_m in (id1, id2)
            elif ft == 2:
                hit = (ident_m & id2) == (id1 & id2)
            else:
                hit = xtd and id1 <= ident <= id2
            if not hit:
                continue
            if fec in (1, 5):
                return 0
            if fec in (2, 6):
                return 1
            if fec == 3:
                return None
            if fec in (4, 7):
                continue
        anf = (gfc >> 2) & 3 if xtd else (gfc >> 4) & 3
        return anf if anf < 2 else None

    def inject(self, ident: int, data: bytes, xtd: bool = False) -> bool:
        if self.regs.get(0x18, 0) & 1:  # INIT: not on the bus
            return False
        f = self._match(ident, xtd)
        if f is None:
            return False
        q = self.fifo[f]
        if len(q) >= 3:
            self.regs[0x50] = self.regs.get(0x50, 0) | (1 << (2 + 3 * f))  # RFnL lost
            self.update_irq()
            return False
        q.append((ident, bytes(data)[:8], xtd))
        self.regs[0x50] = self.regs.get(0x50, 0) | (1 << (3 * f)) | ((1 << (1 + 3 * f)) if len(q) >= 3 else 0)
        self._load(f)
        self.update_irq()
        return True

    def _load(self, f):
        """Keep the FIFO head in element 0 of the FIFO area (get index stays 0)."""
        q = self.fifo[f]
        if not q:
            return
        ident, data, xtd = q[0]
        a = (_RXF0, _RXF1)[f]
        self._wr(a, (ident & 0x1FFFFFFF) | (1 << 30) if xtd else (ident & 0x7FF) << 18)
        self._wr(a + 4, len(data) << 16)
        d = data.ljust(8, b"\0")
        self._wr(a + 8, int.from_bytes(d[:4], "little"))
        self._wr(a + 12, int.from_bytes(d[4:], "little"))

    def level(self):
        ir = self.regs.get(0x50, 0) & self.regs.get(0x54, 0)
        ils = self.regs.get(0x58, 0)
        ile = self.regs.get(0x5C, 0)
        groups = [ir & 0x7, ir & 0x38, ir & 0x1C0, ir & 0x1E00, ir & 0x30000 | ir & 0xE000, ir & 0x1C0000, ir & 0xE00000]
        lines = [False, False]
        for g, active in enumerate(groups):
            if active:
                lines[(ils >> g) & 1] = True
        names = list(self.irqs)
        out = {}
        for i, n in enumerate(names[:2]):
            out[n] = bool(lines[i] and ile & (1 << i))
        return out


# --------------------------------------------------------------------------- SoC builders
G4_CLASSES = {
    "RCC": RccG4, "PWR": PwrG4, "FLASH": FlashG4, "RTC": RtcG4, "CRC": CrcV2,
    "EXTI": n32.Exti, "IWDG": n32.Iwdg, "WWDG": n32.Wwdg, "DMA1": n32.Dma, "DMA2": n32.Dma,
    "FDCAN1": Fdcan, "FDCAN2": Fdcan, "FDCAN3": Fdcan, "CORDIC": Cordic,
}


class ExtiG4(n32.Exti):
    SELECTOR = "SYSCFG"


def build_g4(machine, uid: bytes, flash_kb: int = 512) -> None:
    db = _db("stm32g474.json")
    machine.periph_db = db
    machine.level_sources = getattr(machine, "level_sources", {})
    for name, info in db.items():
        base = info["base"]
        if base >= 0xE0000000:
            continue
        if name.startswith("GPIO"):
            p = GpioV2(machine, name, base, name[-1])
        elif name.startswith(("USART", "UART", "LPUART")):
            p = UsartV2(machine, name, base)
        elif name.startswith("I2C"):
            p = I2cV2(machine, name, base)
        elif name.startswith("TIM"):
            p = n32.Timer(machine, name, base)
        elif name.startswith("SPI"):
            p = n32.Spi(machine, name, base)
        elif name in ("ADC1", "ADC2", "ADC3", "ADC4", "ADC5"):
            p = AdcV2(machine, name, base)
        elif name == "EXTI":
            p = ExtiG4(machine, name, base)
        elif name.startswith("FDCAN") and name != "FDCAN":
            p = Fdcan(machine, name, base, ram_index=int(name[-1]) - 1)
        elif name == "HRTIM_Master" or name.startswith("HRTIM_TIM"):
            p = HrtimUnit(machine, name, base, "M" if name == "HRTIM_Master" else name[-1])
        else:
            cls = G4_CLASSES.get(name, Base)
            p = cls(machine, name, base)
        if name.startswith("ADC") or name in ("VREFBUF", "COMP", "OPAMP") or name.startswith("HRTIM_") \
                or name.startswith("DAC"):
            p.size = 0x100 if not name.startswith("HRTIM_") else 0x80
            if name in ("VREFBUF", "COMP", "OPAMP"):
                p.size = 0x30 if name == "VREFBUF" else 0x100
        if name == "SYSCFG":
            p.size = 0x30
        if name == "FDCAN":  # message RAM
            p.size = 0xA00
        machine.bus.attach(p)
    machine.bus.get("FLASH").bank_size = flash_kb * 1024 // 2
    machine.hrtim = Hrtim(machine)
    adcs = [machine.bus.get(n) for n in ("ADC1", "ADC2", "ADC3", "ADC4", "ADC5")]
    machine.hrtim.adc_listeners.append(lambda trg: [a.hrtim_trigger(trg) for a in adcs])
    machine.bus.map_window(0x40000000, 0x30000)
    machine.bus.map_window(0x48000000, 0x2000)
    machine.bus.map_window(0x50000000, 0x2000)
    machine.bus.map_window(0x50060000, 0x1000)
    machine.bus.map_window(0xA0000000, 0x2000)
    machine.map_ram(0x10000000, 0x8000)  # CCM SRAM
    machine.map_ram(0x1FFF0000, 0x10000)
    machine.write(0x1FFF75E0, flash_kb.to_bytes(2, "little"))
    machine.write(0x1FFF7590, uid[:12].ljust(12, b"\0"))
    n32._map_bitband(machine)


def build_f0(machine, uid: bytes, flash_kb: int = 128) -> None:
    """STM32F0-class Cortex-M0 (CT002/CT003)."""
    machine.level_sources = getattr(machine, "level_sources", {})
    irq = {
        "WWDG": 0, "RTC": 2, "FLASH": 3, "RCC": 4, "EXTI0_1": 5, "EXTI2_3": 6, "EXTI4_15": 7,
        "DMA1_CH1": 9, "DMA1_CH2_3": 10, "DMA1_CH4_5": 11, "ADC": 12, "TIM1_UP": 13, "TIM1_CC": 14,
        "TIM2": 15, "TIM3": 16, "TIM6": 17, "TIM7": 18, "TIM14": 19, "TIM15": 20, "TIM16": 21, "TIM17": 22,
        "I2C1": 23, "I2C2": 24, "SPI1": 25, "SPI2": 26, "USART1": 27, "USART2": 28, "USART3": 29,
    }
    blocks = [
        ("TIM2", 0x40000000, n32.Timer, ["TIM2"]), ("TIM3", 0x40000400, n32.Timer, ["TIM3"]),
        ("TIM6", 0x40001000, n32.Timer, ["TIM6"]), ("TIM7", 0x40001400, n32.Timer, ["TIM7"]),
        ("TIM14", 0x40002000, n32.Timer, ["TIM14"]), ("RTC", 0x40002800, n32.Rtc, ["RTC"]),
        ("WWDG", 0x40002C00, n32.Wwdg, []), ("IWDG", 0x40003000, n32.Iwdg, []),
        ("SPI2", 0x40003800, n32.Spi, ["SPI2"]), ("USART2", 0x40004400, UsartV2, ["USART2"]),
        ("USART3", 0x40004800, UsartV2, ["USART3"]), ("USART4", 0x40004C00, UsartV2, []),
        ("I2C1", 0x40005400, I2cV2, ["I2C1"]), ("I2C2", 0x40005800, I2cV2, ["I2C2"]),
        ("PWR", 0x40007000, Base, []), ("SYSCFG", 0x40010000, Base, []),
        ("EXTI", 0x40010400, ExtiG4, []), ("ADC1", 0x40012400, None, ["ADC"]),
        ("TIM1", 0x40012C00, n32.Timer, ["TIM1_UP", "TIM1_CC"]), ("SPI1", 0x40013000, n32.Spi, ["SPI1"]),
        ("USART1", 0x40013800, UsartV2, ["USART1"]), ("TIM15", 0x40014000, n32.Timer, ["TIM15"]),
        ("TIM16", 0x40014400, n32.Timer, ["TIM16"]), ("TIM17", 0x40014800, n32.Timer, ["TIM17"]),
        ("DMA1", 0x40020000, n32.Dma, ["DMA1_CH1", "DMA1_CH2_3", "DMA1_CH4_5"]),
        ("RCC", 0x40021000, RccF0, ["RCC"]), ("FLASH", 0x40022000, n32.FlashCtl, ["FLASH"]),
        ("CRC", 0x40023000, CrcV2, []),
    ]
    machine.periph_db = {name: {"base": base, "regs": {}, "irqs": {n: irq[n] for n in irqs}}
                         for name, base, _, irqs in blocks}
    for name, base, cls, irqs in blocks:
        if name == "ADC1":
            p = AdcV2(machine, "ADC1", base, f0=True)
        else:
            p = cls(machine, name, base)
        if isinstance(p, n32.Dma):
            # F0 shares one vector between channels 2/3 and 4/5
            p._irq_for = lambda i: {1: "DMA1_CH1", 2: "DMA1_CH2_3", 3: "DMA1_CH2_3", 4: "DMA1_CH4_5",
                                    5: "DMA1_CH4_5"}.get(i, "DMA1_CH1")
        machine.bus.attach(p)
    for i, port in enumerate("ABCDF"):
        machine.bus.attach(GpioV2(machine, "GPIO" + port, 0x48000000 + 0x400 * i, port))
    machine.bus.map_window(0x40000000, 0x24000)
    machine.bus.map_window(0x48000000, 0x1800)
    machine.map_ram(0x1FFF0000, 0x10000)
    machine.write(0x1FFFF7CC, flash_kb.to_bytes(2, "little"))
    machine.write(0x1FFFF7AC, uid[:12].ljust(12, b"\0"))
