"""The B2500's extension-pack bus (UART5, 19200 baud, all hardware versions).

About once a second the main unit polls extension packs 1 and 2 and
broadcasts the status of its own battery.  Frames::

    0x73 | length (2 bytes, big endian, whole frame) | command | payload | XOR of all bytes before

* ``0x31`` / ``0x32``: poll of extension pack 1 / 2 (14 bytes); the pack
  answers with the same command, byte 4 = 5 + pack number, and 133 bytes
  laid out like the status frame from offset 5 (139 bytes in all; byte 5 must
  be non-zero, or the pack does not count as connected; see :func:`pack_reply`)
* ``0x36`` / ``0x37``: once the packs have answered, about 4 Hz to pack 1 / 2
  (10 bytes); the pack answers with the same frame, byte 4 = 5 + pack number
  and byte 6 = its state bits (:func:`link_reply`).  The main unit then stops
  broadcasting ``0x2a`` and keeps polling for the full records
* ``0x2a``: status of the main unit's battery (151 bytes; 150 in HMB and the
  HMA builds from V241, whose device id is 12 bytes instead of 13, so that
  everything after it sits one byte earlier)

V119 and V160 send no status frame, only the polls (V119 also an 8-byte 0x0c).

Status fields, by offset in the frame, as far as they are checked against the
firmware's ``cd=1`` report and the emulated pack (little endian):

==========  =====  ================================================
offset      type   meaning
==========  =====  ================================================
6           u8     direction: 0 idle, 1 charging, 2 discharging
12..39      u16    cell voltages, mV (14)
40..43      u16    0 in the emulator; temperatures in a capture from a real V1
48          u32    pack voltage, mV
52          u32    battery current, mA (magnitude; see direction)
56          u32    battery power, W (magnitude)
70..79      u16    temperatures, degC (5)
84          u8     state bits: 0x80 charging, 0x40 discharging, 0x03 (FETs?)
88          u8     state of charge, %
122         bytes  device id (12 or 13 bytes, see above)
len-3       u16    the cd=1 report's ``kn``: remaining energy, Wh, in 10 % steps of
                   2240 Wh per pack (summed over the extension packs)
==========  =====  ================================================

Everything else is returned by offset under ``"raw"``.  The port is a TTL
UART (not RS485); framing, cell voltages, SOC, device id and checksum agree
with noone2k's notes from a real V1 (noone2k/hm2500#2).
"""
from __future__ import annotations

import struct
from typing import Dict, Iterator, List, Optional

START = 0x73
STATUS, POLL1, POLL2 = 0x2A, 0x31, 0x32
DIRECTION = {0: "idle", 1: "charging", 2: "discharging"}
_KNOWN = set(range(0, 4)) | {6} | set(range(12, 40)) | set(range(48, 60)) | set(range(70, 80)) | {84, 88}


def xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


class FrameParser:
    """Splits a byte stream into frames (resynchronises on bad checksums)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes) -> Iterator[bytes]:
        buf = self.buf
        buf.extend(data)
        while True:
            i = buf.find(bytes([START]))
            if i < 0:
                buf.clear()
                return
            del buf[:i]
            if len(buf) < 3:
                return
            n = (buf[1] << 8) | buf[2]
            if not 5 <= n <= 512:
                del buf[0]
                continue
            if len(buf) < n:
                return
            frame = bytes(buf[:n])
            if xor(frame[:-1]) != frame[-1]:
                self.bad += 1
                del buf[0]
                continue
            del buf[:n]
            yield frame


def decode(frame: bytes) -> Dict:
    """A frame as a dict (``cmd`` plus the fields known for it)."""
    cmd = frame[3]
    out: Dict = {"cmd": cmd, "len": len(frame)}
    if cmd in (POLL1, POLL2) and len(frame) > 14 and frame[4] == 5 + cmd - 0x30:
        u16 = lambda o: struct.unpack_from("<H", frame, o)[0]
        u32 = lambda o: struct.unpack_from("<I", frame, o)[0]
        direction = DIRECTION.get(frame[6], frame[6])
        sign = -1 if direction == "discharging" else 1
        out.update(type="pack", pack=cmd - 0x30, direction=direction,
                   cells_mv=[u16(12 + 2 * k) for k in range(14)], pack_mv=u32(48),
                   current_ma=sign * u32(52), power_w=sign * u32(56),
                   temps_c=[u16(70 + 2 * k) for k in range(5)], soc=frame[88])
        return out
    if cmd in (POLL1, POLL2):
        out.update(type="poll", pack=frame[5] if len(frame) > 5 else cmd - 0x30)
        return out
    if cmd != STATUS or len(frame) < 150:
        out.update(type="unknown", hex=frame.hex())
        return out
    u16 = lambda o: struct.unpack_from("<H", frame, o)[0]
    u32 = lambda o: struct.unpack_from("<I", frame, o)[0]
    cells = [u16(12 + 2 * k) for k in range(14)]
    id_len = len(frame) - 138
    direction = DIRECTION.get(frame[6], frame[6])
    sign = -1 if direction == "discharging" else 1
    out.update(
        type="status",
        direction=direction,
        cells_mv=cells,
        pack_mv=u32(48),
        current_ma=sign * u32(52),
        power_w=sign * u32(56),
        temps_c=[u16(70 + 2 * k) for k in range(5)],
        state_bits=frame[84],
        soc=frame[88],
        kn_wh=u16(len(frame) - 3),
        device_id=frame[122:122 + id_len].hex(),
        raw={o: frame[o] for o in range(4, len(frame) - 3)
             if o not in _KNOWN and not 122 <= o < 122 + id_len and frame[o]},
    )
    return out


REPLY_LEN = 0x85
#: byte 5 of an answer; the main unit sends 0xe8 there, any non-zero value marks the pack present
PRESENT = 0xE8


def pack_reply(index: int, cells_mv, pack_mv: int, current_ma: int, power_w: int,
               temps_c, soc: int) -> bytes:
    """An extension pack's answer to its poll (``index`` 1 or 2); current and
    power are signed (+ = charging).  Fields the firmware is not known to read
    stay 0."""
    f = bytearray(5 + REPLY_LEN)                  # laid out like the status frame
    f[5] = PRESENT
    f[6] = 1 if current_ma > 0 else 2 if current_ma < 0 else 0
    for k, mv in enumerate(list(cells_mv)[:14]):
        struct.pack_into("<H", f, 12 + 2 * k, int(round(mv)))
    struct.pack_into("<III", f, 48, int(round(pack_mv)), abs(int(round(current_ma))), abs(int(round(power_w))))
    for k, t in enumerate(list(temps_c)[:5]):
        struct.pack_into("<h", f, 70 + 2 * k, int(round(t)))
    f[84] = state_bits(current_ma)
    f[88] = max(0, min(100, int(round(soc))))
    body = bytes([START, 0, 5 + REPLY_LEN + 1, 0x30 + index, 5 + index]) + bytes(f[5:])
    return body + bytes([xor(body)])


def state_bits(current_ma: float) -> int:
    """State byte of a pack: 0x03 (FETs on?) plus 0x80 charging / 0x40 discharging."""
    return 0x03 | (0x80 if current_ma > 0 else 0x40 if current_ma < 0 else 0)


def link_reply(request: bytes, current_ma: float) -> bytes:
    """An extension pack's answer to the main unit's 0x36/0x37 link frame:
    the request with byte 4 = 5 + pack number and byte 6 = the pack's state bits
    (the firmware stores byte 6 where the status record keeps them)."""
    f = bytearray(request)
    f[4] = 5 + (f[3] - 0x35)
    f[6] = state_bits(current_ma)
    f[-1] = xor(bytes(f[:-1]))
    return bytes(f)


def decode_stream(data: bytes) -> List[Dict]:
    """Decode every complete frame in ``data``."""
    return [decode(f) for f in FrameParser().feed(data)]


class BusMonitor:
    """Keeps the latest decoded status of a running B2500 (``.status``)."""

    def __init__(self, emu, uart: str = "UART5"):
        self.parser = FrameParser()
        self.status: Optional[Dict] = None
        self.polls = 0
        emu.machine.bus.get(uart).tx_listeners.append(self._on_tx)

    def _on_tx(self, data: bytes) -> None:
        for f in self.parser.feed(data):
            d = decode(f)
            if d["type"] == "status":
                self.status = d
            elif d["type"] == "poll":
                self.polls += 1


__all__ = ["FrameParser", "decode", "decode_stream", "pack_reply", "BusMonitor"]
