"""I2C slave devices on bit-banged GPIO pins.

Many boards drive their EEPROM with software I2C.  :class:`GpioI2cBus` watches
the master's SCL/SDA output writes, decodes START/STOP and bytes, and drives
SDA (ACK and read data) through the GPIO input override, so the firmware's
bit-banging code reads what a real slave would put on the open-drain line.
Any :class:`~marsemu.periph.n32g45x.I2cDevice` (e.g. ``I2cEeprom``) can be
attached.
"""
from __future__ import annotations

import logging
from typing import Dict, Optional

log = logging.getLogger(__name__)


class GpioI2cBus:
    def __init__(self, scl_gpio, scl_pin: int, sda_gpio, sda_pin: int):
        self.scl_g, self.scl_p = scl_gpio, scl_pin
        self.sda_g, self.sda_p = sda_gpio, sda_pin
        self.devices: Dict[int, object] = {}
        self.scl = 1
        self.sda = 1                 # level the master drives
        self.state = "idle"          # idle, addr, write, read, ack_out, ack_in, ignore
        self.bits = 0
        self.byte = 0
        self.dev = None
        self.reading = False
        self.out_byte = 0
        self.transactions = 0
        scl_gpio.watchers.append(self._on_scl_gpio)
        if sda_gpio is not scl_gpio:
            sda_gpio.watchers.append(self._on_sda_gpio)

    # -- pin plumbing
    def _on_scl_gpio(self, pin: int, lvl: int) -> None:
        if pin == self.scl_p and (self.scl_g is not self.sda_g or pin != self.sda_p):
            self._scl(lvl)
        elif self.scl_g is self.sda_g and pin == self.sda_p:
            self._sda(lvl)

    def _on_sda_gpio(self, pin: int, lvl: int) -> None:
        if pin == self.sda_p:
            self._sda(lvl)

    def _drive(self, level: Optional[int]) -> None:
        if level is None:
            self.sda_g.inputs.pop(self.sda_p, None)
        else:
            self.sda_g.inputs[self.sda_p] = level & 1

    # -- protocol
    def _sda(self, lvl: int) -> None:
        old, self.sda = self.sda, lvl
        if not self.scl or old == lvl:
            return
        if lvl == 0:                                   # START (or repeated START)
            self._drive(None)
            self.state, self.bits, self.byte = "addr", 0, 0
            self.transactions += 1
        else:                                          # STOP
            self._drive(None)
            if self.dev is not None:
                self.dev.stop()
            self.dev, self.state = None, "idle"

    def _scl(self, lvl: int) -> None:
        if lvl == self.scl:
            return
        self.scl = lvl
        st = self.state
        if lvl:                                        # rising edge: sample
            if st in ("addr", "write"):
                self.byte = (self.byte << 1) | (self.sda & 1)
                self.bits += 1
            elif st == "ack_in":                       # master ACK/NACK after a read byte
                self.state = "read" if self.sda == 0 else "ignore"
                self._nack = self.sda
            return
        # falling edge: change what the slave drives
        if st in ("addr", "write") and self.bits == 8:
            ack = self._byte_done()
            self._drive(0 if ack else None)
            self.state = "ack_out_r" if (ack and self.reading and st == "addr") else ("ack_out" if ack else "ignore")
            return
        if st == "ack_out":                            # end of the ACK clock
            self._drive(None)
            self.state, self.bits, self.byte = "write", 0, 0
            return
        if st == "ack_out_r" or (st == "read" and self.bits == 0):
            self._load_read()
            st = self.state
        if st == "read":
            if self.bits < 8:
                self._drive((self.out_byte >> (7 - self.bits)) & 1)
                self.bits += 1
            else:                                      # release for the master's ACK
                self._drive(None)
                self.state = "ack_in"
                self.bits = 0
            return
        if st == "ignore":
            self._drive(None)

    def _load_read(self) -> None:
        self.out_byte = self.dev.read() & 0xFF if self.dev is not None else 0xFF
        self.state, self.bits = "read", 0

    def _byte_done(self) -> bool:
        b = self.byte & 0xFF
        self.bits, self.byte = 0, 0
        if self.state == "addr":
            addr, rd = b >> 1, bool(b & 1)
            dev = self.devices.get(addr)
            if dev is None or not dev.start(rd):
                self.dev = None
                return False
            self.dev, self.reading = dev, rd
            return True
        if self.dev is None:
            return False
        return bool(self.dev.write(b))
