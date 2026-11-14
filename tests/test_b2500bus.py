"""B2500 extension-bus frames: framing, checksum, status fields."""
import struct

from marsemu.b2500bus import FrameParser, decode, decode_stream, xor


def frame(cmd: int, body: bytes) -> bytes:
    head = bytes([0x73]) + struct.pack(">H", 4 + len(body) + 1) + bytes([cmd])
    f = head + body
    return f + bytes([xor(f)])


def status_frame(direction=2, cell=3324, pack_mv=46536, current=7257, power=339, soc=80, kn=1792, bits=0x43):
    b = bytearray(151)
    b[0], b[1], b[2], b[3] = 0x73, 0, 151, 0x2A
    b[6] = direction
    for k in range(14):
        struct.pack_into("<H", b, 12 + 2 * k, cell)
    struct.pack_into("<III", b, 48, pack_mv, current, power)
    for k in range(5):
        struct.pack_into("<H", b, 70 + 2 * k, 25)
    b[84], b[88] = bits, soc
    struct.pack_into("<H", b, 148, kn)
    b[150] = xor(bytes(b[:150]))
    return bytes(b)


def test_poll_frames_as_the_firmware_sends_them():
    poll1 = bytes.fromhex("73000e3100010000000000a1a24e")
    assert xor(poll1[:-1]) == poll1[-1]
    assert decode(poll1) == {"cmd": 0x31, "len": 14, "type": "poll", "pack": 1}


def test_status_fields_and_sign_from_the_direction():
    d = decode(status_frame())
    assert d["type"] == "status" and d["direction"] == "discharging"
    assert d["cells_mv"] == [3324] * 14 and d["pack_mv"] == 46536
    assert d["current_ma"] == -7257 and d["power_w"] == -339
    assert d["soc"] == 80 and d["kn_wh"] == 1792 and d["temps_c"] == [25] * 5
    assert decode(status_frame(direction=1, bits=0x83))["current_ma"] == 7257


def test_short_status_variant_keeps_kn_before_the_checksum():
    f = bytearray(status_frame())
    del f[120]                                       # HMB / HMA V241+: one byte less after offset 88
    f[2] = 150
    f[-1] = xor(bytes(f[:-1]))
    d = decode(bytes(f))
    assert d["type"] == "status" and d["soc"] == 80 and d["kn_wh"] == 1792 and d["pack_mv"] == 46536
    assert len(bytes.fromhex(d["device_id"])) == 12


def test_parser_resynchronises_and_drops_bad_checksums():
    good = status_frame()
    bad = bytearray(frame(0x32, bytes(9)))
    bad[-1] ^= 0xFF
    stream = b"\x00\x73\x00" + bytes(bad) + good[:70]
    p = FrameParser()
    assert list(p.feed(stream)) == []
    got = list(p.feed(good[70:] + frame(0x31, bytes.fromhex("0001000000000000a1a2"))))
    assert [decode(f)["type"] for f in got] == ["status", "poll"]
    assert p.bad >= 1
    assert len(decode_stream(good * 3)) == 3


def test_pack_answer_is_accepted_framing():
    from marsemu.b2500bus import link_reply, pack_reply
    f = pack_reply(2, [3300] * 14, 46200, -6000, -277, [21] * 5, 40)
    assert len(f) == 139 and f[:5] == bytes([0x73, 0, 139, 0x32, 7]) and f[5] != 0, "byte 5 marks the pack present"
    assert xor(f[:-1]) == f[-1]
    d = decode(f)
    assert d["type"] == "pack" and d["pack"] == 2 and d["direction"] == "discharging"
    assert d["pack_mv"] == 46200 and d["current_ma"] == -6000 and d["power_w"] == -277 and d["soc"] == 40
    req = bytes.fromhex("73000a3600010111005e")
    r = link_reply(req, 5000)
    assert r[3] == 0x36 and r[4] == 6 and r[6] == 0x83 and xor(r[:-1]) == r[-1]
    assert link_reply(bytes.fromhex("73000a3700020000004c"), -1)[4] == 7
