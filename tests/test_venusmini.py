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
    block[62:64] = struct.pack(">H", 1)                           # gps = 1: the "1500 W" preset
    (_, _, cmd, echo), = requests(inv, frame(CONTROL, INVERTER, 0x60, bytes(block)))
    assert cmd == 0x60 and echo == bytes(block)
    assert inv.feed_in_limit() == 1250.0                          # Micro 270 caps it at 1250 W
    assert inv.charge_limit() == 1250.0
    block41 = bytearray(52)
    block41[6:8] = struct.pack(">H", 1)                           # ogpc = 1: grid charging capped at 800 W
    requests(inv, frame(CONTROL, INVERTER, 0x61, bytes(block41)))
    assert inv.charge_limit() == 800.0


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


def test_mini_cells_rest_at_the_controls_soc_table():
    """The Mini's cells take the control's resting-voltage tables (0, 20,
    30 degC) as their OCV, so its SOC estimate at start matches the pack."""
    from marsemu.plant.venusmini import MINI_OCV_MV, VenusMini, mini_ocv
    assert mini_ocv(0.5, 20.0) == MINI_OCV_MV[1][50] / 1000
    assert mini_ocv(0.5, 25.0) == (MINI_OCV_MV[1][50] + MINI_OCV_MV[2][50]) / 2000
    assert mini_ocv(0.5, -5.0) == mini_ocv(0.5, 0.0) and mini_ocv(0.5, 45.0) == mini_ocv(0.5, 30.0)
    assert mini_ocv(1.0, 20.0) > mini_ocv(0.5, 20.0) > mini_ocv(0.0, 20.0)
    pack = LfpPack(cells_n=2, capacity_ah=314, soc=0.42, temp_c=20.0)
    pack.cells[0].ocv_curve = pack.cells[1].ocv_curve = lambda soc: mini_ocv(soc, pack.temp_c)
    assert abs(pack.ocv - 2 * mini_ocv(0.42, 20.0)) < 1e-9
    assert VenusMini.CELLS == 2


def test_cells_take_a_curve_of_their_own():
    from marsemu.plant.battery import LFP_OCV, Cell, _interp
    assert Cell(100, 0.5).ocv() == _interp(LFP_OCV, 0.5)
    assert Cell(100, 0.5, ocv_curve=lambda soc: 3.0 + soc).ocv() == 3.5


def test_dcdc_carries_the_controls_permissions_and_limits():
    """0x90: word 1 allows discharging, word 2 charging, words 3/4 cap the
    charge and discharge power (W)."""
    emu = FakeEmu()
    dc = MiniDcdc(emu, LfpPack(cells_n=2, capacity_ah=314, soc=0.5))
    requests(dc, frame(CONTROL, DCDC, 0x90, struct.pack(">6H", 1, 0, 1, 300, 1500, 1)))
    assert dc.on and not dc.discharge_allowed and dc.charge_allowed
    assert dc.limit(-900.0) == -300.0 and dc.limit(600.0) == 600.0
    requests(dc, frame(CONTROL, DCDC, 0x90, struct.pack(">6H", 1, 1, 1, 1500, 400, 0)))
    assert dc.discharge_allowed and dc.limit(600.0) == 400.0 and dc.limit(-900.0) == -900.0


def test_cells_rise_at_the_end_of_a_charge_and_relax_at_rest():
    from marsemu.plant.battery import Cell
    c = Cell(314, 1.0, ocv_curve=lambda soc: 3.38, charge_knee_v=0.25)
    assert c.voltage(0.0) == 3.38                                   # resting: the table's full voltage
    assert abs(c.voltage(10.0) - (3.38 + 10 * c.r_ohm + 0.25)) < 1e-9
    c.soc = 0.5
    assert abs(c.voltage(10.0) - (3.38 + 10 * c.r_ohm)) < 1e-9      # no rise away from full
