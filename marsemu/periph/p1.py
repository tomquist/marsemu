"""A smart meter's P1 port (DSMR), as the CT003 (HME-3) reads it.

The CT003 is a P1 reader: it does not measure, it parses the telegrams the
utility meter sends.  Its firmware (V122) works like this:

* At start it watches PB11, USART3's receive pin.  Once that line has idled
  high for 2 s (a meter is plugged in) it sets up USART3 at 115200 baud, 8N1,
  and receives byte by byte.
* A telegram starts at ``/`` and ends at ``!``; the firmware keeps up to
  1800 bytes and then picks lines out by OBIS code with ``(%d.%d*kW)``-style
  patterns:

  ======================  =========================================
  ``1-0:1.7.0``/``2.7.0``  total import / export power, kW
  ``1-0:21.7.0``/``22.7.0`` phase L1 import / export power, kW (L2: 41/42, L3: 61/62)
  ``1-0:1.8.1``/``1.8.2``  import energy, tariff 1 / 2, kWh
  ``1-0:2.8.1``/``2.8.2``  export energy, tariff 1 / 2, kWh
  ======================  =========================================

  It reads the fraction as a whole number (``%d.%d``), so the model writes the
  three decimals real meters use.

:class:`P1Meter` holds PB11 high and sends a DSMR 5 telegram every second
(DSMR 5 meters send one a second) with the power you set and the energy that
power adds up to.
"""
from __future__ import annotations

from typing import Optional

#: what the CT003's firmware looks for
OBIS_MARKER = b"1-0:21.7.0"


def crc16(data: bytes) -> int:
    """DSMR 5 telegram CRC: CRC-16/ARC over ``/`` to ``!``."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def telegram(a: float, b: float, c: float, energy_kwh=(0.0, 0.0, 0.0, 0.0), tariff: int = 1,
             header: str = "/ISK5\\2M550T-1012") -> bytes:
    """A DSMR 5 telegram for phase powers in watts (+ import, - export)."""
    def kw(w: float) -> str:
        return f"{abs(w) / 1000:06.3f}"

    def imp(w: float) -> str:
        return kw(w) if w > 0 else "00.000"

    def exp(w: float) -> str:
        return kw(w) if w < 0 else "00.000"
    total = a + b + c
    e = energy_kwh
    lines = [header, "", "1-3:0.2.8(50)", "0-0:1.0.0(260101000000W)",
             f"1-0:1.8.1({e[0]:010.3f}*kWh)", f"1-0:1.8.2({e[1]:010.3f}*kWh)",
             f"1-0:2.8.1({e[2]:010.3f}*kWh)", f"1-0:2.8.2({e[3]:010.3f}*kWh)",
             f"0-0:96.14.0({tariff:04d})",
             f"1-0:1.7.0({imp(total)}*kW)", f"1-0:2.7.0({exp(total)}*kW)",
             f"1-0:21.7.0({imp(a)}*kW)", f"1-0:41.7.0({imp(b)}*kW)", f"1-0:61.7.0({imp(c)}*kW)",
             f"1-0:22.7.0({exp(a)}*kW)", f"1-0:42.7.0({exp(b)}*kW)", f"1-0:62.7.0({exp(c)}*kW)", "!"]
    body = "\r\n".join(lines).encode("ascii")
    return body + f"{crc16(body):04X}\r\n".encode("ascii")


class P1Meter:
    """The utility meter on a CT003's P1 port.

    ``set_power(a, b, c)`` in watts per phase, like the CT002's
    :class:`~marsemu.periph.bl_spi.BlMeter`.  ``energy_kwh`` (import T1, import
    T2, export T1, export T2) grows with that power in the current ``tariff``.
    """

    def __init__(self, emu, period_s: float = 1.0, uart: str = "USART3", sense=("GPIOB", 11)):
        self.emu = emu
        self.m = emu.machine
        self.uart = self.m.bus.get(uart)
        self.period_s = period_s
        self.power = {"A": 0.0, "B": 0.0, "C": 0.0}
        self.energy_kwh = [0.0, 0.0, 0.0, 0.0]
        self.tariff = 1
        self.sent = 0
        self.connected = True
        self._sense = self.m.bus.get(sense[0]), sense[1]
        self._sense[0].inputs[self._sense[1]] = 1          # the data line idles high: a meter is there
        self.m.clock.after(int(period_s * self.m.clock.hz), self._tick, "p1-telegram")

    def set_power(self, a: float, b: float, c: float) -> None:
        self.power = {"A": float(a), "B": float(b), "C": float(c)}

    def set_connected(self, on: bool) -> None:
        """Unplug (False) or plug in the meter: the line drops low, no telegrams."""
        self.connected = bool(on)
        self._sense[0].inputs[self._sense[1]] = 1 if on else 0

    def _tick(self) -> None:
        self.m.clock.after(int(self.period_s * self.m.clock.hz), self._tick, "p1-telegram")
        total = sum(self.power.values())
        hours = self.period_s / 3600.0
        k = self.tariff - 1
        if total > 0:
            self.energy_kwh[k] += total / 1000 * hours
        else:
            self.energy_kwh[2 + k] += -total / 1000 * hours
        if not self.connected or not self.uart.rx_enabled() or len(self.uart.rx) > 64:
            return                                          # nobody listening yet: the meter's bytes are lost
        p = self.power
        self.uart.inject(telegram(p["A"], p["B"], p["C"], self.energy_kwh, self.tariff))
        self.sent += 1


def attach_p1_meter(emu) -> Optional[P1Meter]:
    """Give a CT003 image its utility meter, if it is one (it parses P1 OBIS lines)."""
    if OBIS_MARKER not in getattr(emu, "image", b""):
        return None
    meter = P1Meter(emu)
    emu.ct_meter = meter
    return meter
