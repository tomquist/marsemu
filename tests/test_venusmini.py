"""Venus E Mini board models: framing, the inverter's protocol and regulator,
the DC/DC's measurements (no firmware needed)."""
import struct

from marsemu.core.machine import Machine
from marsemu.periph import n32g45x
from marsemu.plant.battery import LfpPack
from marsemu.plant.cosim import CoSim
from marsemu.plant.venusmini import (CONTROL, DCDC, INVERTER, FrameParser, MiniDcdc, MiniInverter, crc16, frame,
                                     ntc_raw)


class FakeEmu:
    """Just enough of an Emulator for a board on the control's UART."""

    def __init__(self):
        self.machine = m = Machine()
        n32g45x.build(m)
        self.taps = {}

    @property
    def time(self):
        return self.machine.clock.seconds


def test_frames_carry_a_modbus_crc_and_a_big_endian_length():
    # a request the control sends to the DC/DC (cmd 0x80, no payload), as captured
    assert frame(CONTROL, DCDC, 0x80) == bytes.fromhex("aa808200088037d7")
    assert crc16(b"123456789") == 0x4B37             # CRC-16/MODBUS check value
    p = FrameParser()
    good = frame(INVERTER, CONTROL, 0x72, bytes(14))
    bad = bytearray(good)
    bad[7] ^= 1
    got = list(p.feed(b"\x00\xaa" + bytes(bad) + good[:5])) + list(p.feed(good[5:]))
    assert got == [(INVERTER, CONTROL, 0x72, bytes(14))]
    assert p.bad >= 1


def requests(board, *frames_):
    """Send control requests to a board; returns the replies it puts on the UART."""
    u = board.uart
    for f in frames_:
        board.on_tx(f)
    board.emu.machine.advance(0.01)
    out = list(FrameParser().feed(bytes(u.rx)))
    u.rx.clear()
    return out


def test_inverter_answers_polls_and_echoes_parameter_writes():
    emu = FakeEmu()
    inv = MiniInverter(emu, CoSim())
    replies = requests(inv, frame(CONTROL, INVERTER, 0x01), frame(CONTROL, INVERTER, 0x02),
                       frame(CONTROL, INVERTER, 0x40))
    assert [(c, len(p)) for _, _, c, p in replies] == [(0x01, 80), (0x02, 82), (0x40, 68)]
    status, realtime = replies[0][3], replies[1][3]
    assert struct.unpack_from(">H", status, 32)[0] == 270           # inv
    hz, volts = struct.unpack_from(">HH", realtime, 0)
    assert hz > 4000 and volts > 800                              # the control sees a grid
    block = bytearray(68)
    block[62:64] = struct.pack(">H", 1)                           # gps = 1: the 1500 W preset
    (_, _, cmd, echo), = requests(inv, frame(CONTROL, INVERTER, 0x60, bytes(block)))
    assert cmd == 0x60 and echo == bytes(block)
    assert inv.feed_in_limit() == 1500.0


def meter(inv, grid_w, n=1):
    for _ in range(n):
        inv.handle(0x21, struct.pack(">10h", grid_w, 0, 0, 0, 0, 0, 0, 0, 0, 1))


def test_inverter_regulates_on_the_forwarded_meter_reading():
    """Mode 0: Micro 270's regulator; discharge grows by half the import per
    reading, and it turns to charging on a sustained export at its floor."""
    inv = MiniInverter(FakeEmu(), CoSim())
    inv.handle(0x72, struct.pack(">8H", 0, 0, 0, 0, 0, 0, 0, 0))
    assert inv.target() == 0.0                                     # no meter reading yet
    meter(inv, 0)
    assert inv.target() == -20.0                                   # the floor: absorbs 20 W
    meter(inv, 600)
    assert inv.target() == -20 + 300 - 2
    meter(inv, 400, 20)
    assert inv.target() == 800.0                                   # the 800 W feed-in limit
    meter(inv, -2000, 2)
    assert inv.target() == -20.0 and not inv.charging_state
    meter(inv, -300)                                               # the third export reading at the floor
    assert inv.charging_state and inv.target() == 0.0
    meter(inv, -300)
    assert inv.target() == -150.0                                  # charging: half the export
    # the meter reading goes invalid: idle, and it starts over from its charging state
    inv.handle(0x21, struct.pack(">10h", 500, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    assert inv.target() == 0.0 and inv.charging_state
    meter(inv, 500, 3)                                             # import: back to discharging
    assert not inv.charging_state


def test_inverter_fixed_modes():
    inv = MiniInverter(FakeEmu(), CoSim())
    inv.handle(0x72, struct.pack(">HH", 1, 800) + bytes(12))      # phase detection: charge 800 W
    assert inv.target() == -800.0
    inv.handle(0x72, struct.pack(">HH", 2, 2000) + bytes(12))     # discharge, held to the limit
    assert inv.target() == 800.0


def test_dcdc_reports_the_two_cells_and_the_current():
    emu = FakeEmu()
    pack = LfpPack(cells_n=2, capacity_ah=314, soc=0.5, temp_c=40.0)
    pack.current_a = -95.5
    dc = MiniDcdc(emu, pack)
    (_, _, cmd, p), = requests(dc, frame(CONTROL, DCDC, 0x80))
    assert cmd == 0x80 and len(p) == 40
    v = struct.unpack_from(">I", p, 2)[0]
    i_coarse, i_fine = struct.unpack_from(">ii", p, 6)
    c1, c2 = struct.unpack_from(">HH", p, 14)
    assert i_coarse == i_fine == -95500
    assert abs(c1 - pack.cell_voltages[0] * 1000) < 1 and v == c1 + c2
    assert struct.unpack_from(">h", p, 18)[0] == ntc_raw(40.0)
    assert struct.unpack_from(">H", p, 36)[0] == 268                # dcdc
    assert not dc.on
    requests(dc, frame(CONTROL, DCDC, 0x90, struct.pack(">6H", 1, 1, 1, 0, 0, 0)))
    assert dc.on


def test_cell_temperature_encoding():
    """The control's conversion: identity up to 300 (30.0 degC), 0.9 x raw + 30 above."""
    for c in (-10.0, 0.0, 25.0, 30.0):
        assert ntc_raw(c) == round(c * 10)
    for c in (40.0, 55.5):
        assert abs(0.9 * ntc_raw(c) + 30 - c * 10) < 1
