"""F2803x enhanced CAN controller (eCAN-A at 0x6000).

Register map and behaviour from SPRUI10, "Enhanced Controller Area Network
(eCAN)": 32 mailboxes, each configurable to transmit or receive, with a
local acceptance mask per mailbox, the global interrupt lines ECAN0INT and
ECAN1INT (PIE 9.5 and 9.6) and write-1-to-clear flag registers.

The CPU reaches the 32-bit registers as two 16-bit halves (low word first),
so every register is kept as 16-bit words and each half acts on its own 16
bits: a write of 1s to the low half of ``CANTA`` clears mailboxes 0-15, one
to the high half 16-31.

What is modelled: configuration mode (``CCR``/``CCE``), transmission of the
mailboxes whose ``TRS`` bit is set (one frame at a time, highest priority
first, ``TA`` when the frame is complete on the bus), reception into the
highest-numbered enabled receive mailbox whose identifier matches (with its
acceptance mask when ``AME`` is set), overwrite protection (``OPC``), lost
messages (``RML``), transmit abort (``TRR``/``AA``), both byte orders
(``DBO``), the mailbox interrupts and their vector (``MIV``).  Not modelled:
remote frames, error counters and bus-off, time stamps and time-outs,
self-test loopback.

The connection to a bus is that of the other CAN models
(:class:`~marsemu.plant.cosim.EmulatorCanNode`): ``tx_listeners`` get
``(ident, data, ext)`` for every frame sent, :meth:`inject` takes a frame
from the bus.
"""
from __future__ import annotations

import logging
from typing import Callable, List

from .machine import C28xPeripheral

log = logging.getLogger("emu.f2803x.can")

# 32-bit control registers, word offsets
CANME, CANMD, CANTRS, CANTRR, CANTA, CANAA, CANRMP, CANRML = 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E
CANRFP, CANGAM, CANMC, CANBTC, CANES, CANTEC, CANREC = 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C
CANGIF0, CANGIM, CANGIF1, CANMIM, CANMIL, CANOPC = 0x1E, 0x20, 0x22, 0x24, 0x26, 0x28
CANTIOC, CANRIOC, CANTSC, CANTOC, CANTOS = 0x2A, 0x2C, 0x2E, 0x30, 0x32
LAM_BASE, MOTS_BASE, MOTO_BASE, MBOX_BASE = 0x40, 0x80, 0xC0, 0x100

_NAMES = ["CANME", "CANMD", "CANTRS", "CANTRR", "CANTA", "CANAA", "CANRMP", "CANRML", "CANRFP", "CANGAM",
          "CANMC", "CANBTC", "CANES", "CANTEC", "CANREC", "CANGIF0", "CANGIM", "CANGIF1", "CANMIM", "CANMIL",
          "CANOPC", "CANTIOC", "CANRIOC", "CANTSC", "CANTOC", "CANTOS"]

# CANMC bits
MC_SRES, MC_DBO, MC_PDR, MC_CCR = 1 << 5, 1 << 10, 1 << 11, 1 << 12
# CANES bits
ES_PDA, ES_CCE = 1 << 3, 1 << 4
# CANGIFx bits
GIF_AAIF, GIF_RMLIF, GIF_GMIF = 1 << 14, 1 << 11, 1 << 15
# MSGID bits (high word)
ID_IDE, ID_AME = 0x8000, 0x4000
#: PIE (group, index) of ECAN0INTA and ECAN1INTA
PIE_LINES = ((9, 5), (9, 6))
#: frame time when the bit timing is not set up, Hz
DEFAULT_BITRATE = 500_000

#: write-1-to-clear flag registers
_W1C = (CANTA, CANAA, CANRMP)


def _names() -> dict:
    names = {}
    for i, n in enumerate(_NAMES):
        names[2 * i] = n
        names[2 * i + 1] = n + "_H"
    for k in range(32):
        names[LAM_BASE + 2 * k] = f"LAM{k}"
        names[LAM_BASE + 2 * k + 1] = f"LAM{k}_H"
        names[MOTS_BASE + 2 * k] = f"MOTS{k}"
        names[MOTS_BASE + 2 * k + 1] = f"MOTS{k}_H"
        names[MOTO_BASE + 2 * k] = f"MOTO{k}"
        names[MOTO_BASE + 2 * k + 1] = f"MOTO{k}_H"
        for j, n in enumerate(("MSGID", "MSGID_H", "MSGCTRL", "MSGCTRL_H", "MDL", "MDL_H", "MDH", "MDH_H")):
            names[MBOX_BASE + 8 * k + j] = f"MBOX{k}.{n}"
    return names


class ECan(C28xPeripheral):
    """eCAN-A: 32 mailboxes on one CAN bus."""

    reg_names = _names()

    def __init__(self, m, name, base, size):
        super().__init__(m, name, base, size)
        self.tx_listeners: List[Callable[[int, bytes, bool], None]] = []
        #: when the frame being sent is complete (clock cycles)
        self._busy_until = 0
        self._tx_ev = None
        self.tx_frames = 0
        self.rx_frames = 0
        self.rx_dropped = 0

    def reset(self):
        super().reset()
        self.regs[CANMC] = MC_CCR | 0x0000
        self.regs[CANES] = ES_CCE
        self._busy_until = 0
        if self._tx_ev is not None:
            self._tx_ev.cancel()
            self._tx_ev = None

    # -- 32-bit views
    def r32(self, off: int) -> int:
        return self.regs.get(off, 0) | (self.regs.get(off + 1, 0) << 16)

    def _set32(self, off: int, v: int) -> None:
        self.regs[off] = v & 0xFFFF
        self.regs[off + 1] = (v >> 16) & 0xFFFF

    def _mbox(self, k: int, j: int) -> int:
        return self.regs.get(MBOX_BASE + 8 * k + j, 0)

    # -- registers
    def read(self, off):
        if off in (CANES, CANES + 1):
            mc = self.regs.get(CANMC, 0)
            es = (ES_CCE if mc & MC_CCR else 0) | (ES_PDA if mc & MC_PDR else 0)
            return es if off == CANES else self.regs.get(CANES + 1, 0)
        if off in (CANGIF0, CANGIF0 + 1, CANGIF1, CANGIF1 + 1):
            self._update_gif()
        return self.regs.get(off, 0)

    def write(self, off, v):
        base = off & ~1
        if base in _W1C:
            self.regs[off] = self.regs.get(off, 0) & ~v
            if base == CANRMP:
                self.regs[CANRML + (off & 1)] = self.regs.get(CANRML + (off & 1), 0) & ~v
            self._irq_after_clear()
            return
        if base == CANTRS:
            v &= ~self.regs.get(CANMD + (off & 1), 0)          # transmit mailboxes only
            v &= self.regs.get(CANME + (off & 1), 0)
            if v:
                self.regs[off] = self.regs.get(off, 0) | v
                self._kick()
            return
        if base == CANTRR:
            pend = self.regs.get(CANTRS + (off & 1), 0) & v
            if pend:
                self.regs[CANTRS + (off & 1)] &= ~pend
                self.regs[CANAA + (off & 1)] = self.regs.get(CANAA + (off & 1), 0) | pend
            return
        if base in (CANGIF0, CANGIF1):
            # the flags other than GMIF/MIV are write-1-to-clear
            self.regs[off] = self.regs.get(off, 0) & ~(v & ~(GIF_GMIF | 0x1F)) if off == base else \
                self.regs.get(off, 0) & ~v
            return
        if base == CANES:
            self.regs[off] = self.regs.get(off, 0) & ~v
            return
        if base == CANMC and off == CANMC and v & MC_SRES:
            mc_h = self.regs.get(CANMC + 1, 0)
            self.reset()
            self.regs[CANMC + 1] = mc_h
            return
        self.regs[off] = v
        if base in (CANMIM, CANGIM, CANMIL):
            self._irq_after_clear()

    # -- transmission
    def _bitrate(self) -> int:
        """From CANBTC: BRP (bits 23:16), TSEG1 (6:3), TSEG2 (2:0); the bit
        clock is SYSCLKOUT / 2 on the F2803x."""
        btc = self.r32(CANBTC)
        brp = ((btc >> 16) & 0xFF) + 1
        tq = ((btc >> 3) & 0xF) + 1 + (btc & 7) + 1 + 1
        if btc == 0 or tq < 8:
            return DEFAULT_BITRATE
        return int(self.m.clock.hz / 2 / brp / tq)

    def _kick(self) -> None:
        if self._tx_ev is None:
            clk = self.m.clock
            self._tx_ev = clk.at(max(clk.cycles + 1, self._busy_until), self._tx_next, "ecan-tx")

    def _tx_next(self) -> None:
        self._tx_ev = None
        trs = self.r32(CANTRS) & self.r32(CANME) & ~self.r32(CANMD)
        if not trs or self.regs.get(CANMC, 0) & MC_CCR:
            return
        # highest transmit priority level first, then the highest mailbox number
        k = max((i for i in range(32) if trs >> i & 1), key=lambda i: (self._mbox(i, 3) >> 8 & 0x1F, i))
        ident, ext = self._mbox_id(k)
        data = self._mbox_data(k, self._mbox(k, 2) & 0xF)
        from ..plant.canbus import frame_bits
        clk = self.m.clock
        dur = int(frame_bits(len(data), ext) * clk.hz / self._bitrate())
        self._busy_until = clk.cycles + dur
        self.tx_frames += 1
        for fn in self.tx_listeners:
            fn(ident, data, ext)
        clk.at(self._busy_until, lambda: self._tx_done(k), "ecan-ta")

    def _tx_done(self, k: int) -> None:
        bit = 1 << (k & 15)
        hw = k >> 4
        if self.regs.get(CANTRS + hw, 0) & bit:
            self.regs[CANTRS + hw] &= ~bit
            self.regs[CANTA + hw] = self.regs.get(CANTA + hw, 0) | bit
            self._flag(k)
        self._kick()

    def _mbox_id(self, k: int):
        lo, hi = self._mbox(k, 0), self._mbox(k, 1)
        if hi & ID_IDE:
            return ((hi & 0x1FFF) << 16) | lo, True
        return (hi >> 2) & 0x7FF, False

    def _byte_order_msb_first(self) -> bool:
        return not self.regs.get(CANMC, 0) & MC_DBO

    def _mbox_data(self, k: int, dlc: int) -> bytes:
        mdl = self._mbox(k, 4) | (self._mbox(k, 5) << 16)
        mdh = self._mbox(k, 6) | (self._mbox(k, 7) << 16)
        if self._byte_order_msb_first():
            raw = mdl.to_bytes(4, "big") + mdh.to_bytes(4, "big")
        else:
            raw = mdl.to_bytes(4, "little") + mdh.to_bytes(4, "little")
        return raw[:min(dlc, 8)]

    # -- reception
    def inject(self, ident: int, data: bytes, ext: bool = True) -> bool:
        """A frame from the bus.  False if no mailbox took it."""
        if self.regs.get(CANMC, 0) & MC_CCR:
            return False
        me, md = self.r32(CANME), self.r32(CANMD)
        rx = me & md
        rmp, opc = self.r32(CANRMP), self.r32(CANOPC)
        target = None
        for k in range(31, -1, -1):
            if not rx >> k & 1 or not self._accepts(k, ident, ext):
                continue
            if rmp >> k & 1 and opc >> k & 1:
                continue
            target = k
            break
        if target is None:
            self.rx_dropped += 1
            return False
        k = target
        b = MBOX_BASE + 8 * k
        hi = self.regs.get(b + 1, 0)
        if ext:
            self.regs[b] = ident & 0xFFFF
            self.regs[b + 1] = (hi & (ID_AME | 0x2000)) | ID_IDE | ((ident >> 16) & 0x1FFF)
        else:
            self.regs[b] = 0
            self.regs[b + 1] = (hi & (ID_AME | 0x2000)) | ((ident & 0x7FF) << 2)
        self.regs[b + 2] = (self.regs.get(b + 2, 0) & ~0x1F) | min(len(data), 8)
        raw = bytes(data[:8]).ljust(8, b"\0")
        if self._byte_order_msb_first():
            mdl, mdh = int.from_bytes(raw[:4], "big"), int.from_bytes(raw[4:], "big")
        else:
            mdl, mdh = int.from_bytes(raw[:4], "little"), int.from_bytes(raw[4:], "little")
        self.regs[b + 4], self.regs[b + 5] = mdl & 0xFFFF, mdl >> 16
        self.regs[b + 6], self.regs[b + 7] = mdh & 0xFFFF, mdh >> 16
        bit, hw = 1 << (k & 15), k >> 4
        if self.regs.get(CANRMP + hw, 0) & bit:
            self.regs[CANRML + hw] = self.regs.get(CANRML + hw, 0) | bit
            line = CANGIF1 if self.r32(CANMIL) >> k & 1 else CANGIF0
            self.regs[line] = self.regs.get(line, 0) | GIF_RMLIF
        self.regs[CANRMP + hw] = self.regs.get(CANRMP + hw, 0) | bit
        self.rx_frames += 1
        self._flag(k)
        return True

    def _accepts(self, k: int, ident: int, ext: bool) -> bool:
        lo, hi = self._mbox(k, 0), self._mbox(k, 1)
        mid = ((hi & 0x1FFF) << 16) | lo                     # bits 28:0
        lam = self.regs.get(LAM_BASE + 2 * k, 0) | (self.regs.get(LAM_BASE + 2 * k + 1, 0) << 16)
        mask = (lam & 0x1FFFFFFF) if hi & ID_AME else 0
        if not (hi & ID_AME and lam >> 31) and bool(hi & ID_IDE) != ext:
            return False
        rid = ident & 0x1FFFFFFF if ext else (ident & 0x7FF) << 18
        if not ext:
            mid &= 0x1FFC0000
            mask &= 0x1FFC0000
        return ((rid ^ mid) & ~mask & 0x1FFFFFFF) == 0

    # -- interrupts
    def _pending(self) -> int:
        return (self.r32(CANTA) | self.r32(CANRMP)) & self.r32(CANMIM)

    def _update_gif(self) -> None:
        pend = self._pending()
        mil = self.r32(CANMIL)
        for line, off in ((0, CANGIF0), (1, CANGIF1)):
            p = pend & (mil if line else ~mil)
            g = self.regs.get(off, 0) & ~(GIF_GMIF | 0x1F)
            if p:
                g |= GIF_GMIF | (p.bit_length() - 1)
            self.regs[off] = g

    def _flag(self, k: int) -> None:
        if not self.r32(CANMIM) >> k & 1:
            return
        self._update_gif()
        line = self.r32(CANMIL) >> k & 1
        if self.regs.get(CANGIM, 0) >> line & 1:
            self.m.pie.raise_irq(*PIE_LINES[line])

    def _irq_after_clear(self) -> None:
        """A mailbox flag still pending after the CPU cleared another one
        keeps its interrupt line asserted: the PIE sees it again."""
        self._update_gif()
        gim = self.regs.get(CANGIM, 0)
        pend = self._pending()
        mil = self.r32(CANMIL)
        for line in (0, 1):
            if gim >> line & 1 and pend & (mil if line else ~mil):
                self.m.pie.raise_irq(*PIE_LINES[line])


__all__ = ["ECan"]
