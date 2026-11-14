"""B2500 plant: SH367309 front end, hardware I2C reads, PV and output stages."""
import math

from marsemu.core.machine import Machine
from marsemu.periph import n32g45x
from marsemu.periph.sh36735 import crc8_07
from marsemu.periph.sh367309 import BFLAG2, CELL1, CUR, Sh367309
from marsemu.plant.b2500 import (
    MicroInverterInput, OutputStage, PvArray, PvConverter, RSENSE_OHM, heatsink_adc, out_i_to_adc, pv_i_to_adc,
    v_to_adc,
)


def afe_read(chip, reg, n):
    assert chip.start(False)
    chip.write(reg)
    chip.write(n)
    chip.start(True)
    data = [chip.read() for _ in range(n + 1)]
    chip.stop()
    return data[:n], data[n]


def test_sh367309_read_crc_covers_both_addresses():
    chip = Sh367309()
    chip.set_cells_mv([3300] * 14)
    data, crc = afe_read(chip, CELL1, 4)
    raw = int(3300 * 32 / 5)
    assert data == [raw >> 8, raw & 0xFF] * 2
    assert crc == crc8_07([0x34, CELL1, 4, 0x35] + data)


def test_sh367309_write_needs_crc_and_flags_clear_on_read():
    chip = Sh367309()
    chip.start(False)
    assert chip.write(0x00) and chip.write(0x6E)
    assert not chip.write(0x00)                     # wrong CRC: NACK, not written
    assert chip.regs[0] != 0x6E
    chip.start(False)
    for b in (0x00, 0x6E, crc8_07([0x34, 0x00, 0x6E])):
        assert chip.write(b)
    assert chip.regs[0] == 0x6E
    chip.tick()
    assert chip.regs[BFLAG2] & 0x30
    afe_read(chip, BFLAG2, 1)
    assert not chip.regs[BFLAG2] & 0x30


def test_sh367309_current_matches_firmware_scale():
    chip = Sh367309()
    chip.set_current(-10.0, RSENSE_OHM)              # discharging 10 A
    raw = int.from_bytes(chip.regs[0x6E:0x70], "big", signed=True)
    assert raw < 0 and abs(-raw * 12 - 87 - 10000) < 120   # firmware: 12 mA/count - 87
    vadc = int.from_bytes(chip.regs[CUR:CUR + 2], "big", signed=True)
    assert vadc < 0


def test_sh367309_reports_the_current_direction():
    """BSTATUS3 DSGING/CHGING: the B2500 only counts its SOC while one is set."""
    chip = Sh367309()
    chip.regs[0x40] = 0x30                          # both FETs on
    chip._update_fets()
    for amps, bits in ((-10.0, 0x40), (10.0, 0x80), (0.1, 0)):
        chip.set_current(amps, RSENSE_OHM)
        assert chip.regs[0x45] & 0xC0 == bits
        assert chip.fets() == 3


def test_i2c_stop_keeps_the_last_received_byte():
    """The B2500 driver sets STOP before it reads the final (CRC) byte."""
    m = Machine()
    n32g45x.build(m, uid=b"\0" * 12, flash_kb=256)
    i2c = m.bus.get("I2C2")
    chip = Sh367309()
    chip.set_cells_mv([3200] * 14)
    i2c.devices[0x1A] = chip
    i2c.write32(0x00, 1 | 1 << 8)                   # START
    i2c.write32(0x10, 0x34)
    i2c.read32(0x14), i2c.read32(0x18)
    i2c.write32(0x10, CELL1)
    i2c.write32(0x10, 2)
    i2c.write32(0x00, 1 | 1 << 8)                   # repeated START
    i2c.write32(0x10, 0x35)
    i2c.read32(0x14), i2c.read32(0x18)
    got = [i2c.read32(0x10), i2c.read32(0x10)]
    i2c.write32(0x00, 1 | 1 << 9)                   # STOP, CRC byte still pending
    assert i2c.read32(0x14) & 0x40
    got.append(i2c.read32(0x10))
    assert got[2] == crc8_07([0x34, CELL1, 2, 0x35] + got[:2])
    assert not i2c.read32(0x14) & 0x40


def test_adc_scales_match_firmware_formulas():
    assert abs(v_to_adc(51.232) * 25 + 32 - 51232) <= 25
    assert abs(pv_i_to_adc(8.0) * 8.25 - 8000) <= 8.25
    raw = out_i_to_adc(4.0)
    assert abs((raw - 610) * 5.9 + 5 - 4000) <= 6
    assert abs(heatsink_adc(25.0) - 3129) <= 3
    assert abs(heatsink_adc(0.0) - 3743) <= 40


def test_pv_array_peak_and_converter_tracks_setpoint():
    arr = PvArray(300.0)
    best = max(v * arr.current(v) for v in [arr.voc * k / 1000 for k in range(1000)])
    assert abs(best - 300.0) < 1.0
    conv = PvConverter(arr)
    conv.enabled = True
    conv.set_refs(int(4374 - 0.062 * arr.vmp * 1000), 0)     # MPP voltage, no current limit
    for _ in range(100):
        conv.step(0.01, 46.0)
    assert abs(conv.v - arr.vmp) < 0.2 and conv.power > 295
    conv.set_refs(int(4374 - 0.062 * arr.vmp * 1000), int(4224 - 0.2793 * 3000))   # 3 A into the battery
    for _ in range(100):
        conv.step(0.01, 46.0)
    assert abs(conv.dc_power - 3.0 * 46.0) < 3 and conv.v > arr.vmp


def test_converter_off_leaves_pv_at_open_circuit():
    conv = PvConverter(PvArray(300.0))
    conv.set_refs(3199, 999)
    for _ in range(100):
        conv.step(0.01, 46.0)
    assert conv.power == 0 and abs(conv.v - conv.array.voc) < 0.1


def test_inverter_finds_the_mpp_of_a_synthetic_panel():
    """What the B2500 does: output current follows a PV-like curve of the voltage."""
    inv = MicroInverterInput(max_w=400)
    stage = OutputStage(inv)
    stage.enabled = True

    def panel_current(v, voc=40.0, vmp=35.2, imp=4.0):
        if v >= voc:
            return 0.0
        return imp * min(1.2, (voc - v) / (voc - vmp)) if v > vmp else imp * (1 + 0.05 * (vmp - v) / vmp)

    for _ in range(int(120 / 0.01)):
        stage.i_lim = panel_current(stage.v)
        stage.step(0.01)
    assert abs(stage.v - 35.2) < 1.5
    assert stage.power > 0.9 * 35.2 * 4.0


def test_inverter_caps_its_power():
    inv = MicroInverterInput(max_w=200)
    stage = OutputStage(inv)
    stage.enabled = True
    stage.i_lim = 10.0                               # stiff source: power only limited by the inverter
    for _ in range(int(120 / 0.01)):
        stage.step(0.01)
    assert stage.power <= 210
    assert not math.isnan(stage.power)


def test_v1_switched_output_from_battery_ramps_to_the_inverter_limit():
    from marsemu.plant.b2500 import SwitchedOutput
    out = SwitchedOutput(MicroInverterInput(max_w=400))
    out.battery_on = True
    for _ in range(int(5 / 0.01)):
        out.step(0.01, 48.0)
    assert out.power == 0, "inverter still in its start-up delay"
    for _ in range(int(20 / 0.01)):
        out.step(0.01, 48.0)
    assert abs(out.power - 400) < 1 and abs(out.v - 48.0) < 1e-9
    assert out.dc_power == -out.power
    out.battery_on = False
    out.step(0.01, 48.0)
    assert out.power == 0


def test_v1_pv2_pass_through_lets_the_inverter_track_the_panel():
    from marsemu.plant.b2500 import SwitchedOutput
    arr = PvArray(300.0)
    out = SwitchedOutput(MicroInverterInput(max_w=400))
    out.passthrough = arr
    for _ in range(int(120 / 0.01)):
        out.step(0.01, 48.0)
    assert out.power > 290 and abs(out.v - arr.vmp) < 2
    assert out.dc_power == 0, "pass-through power does not come from the battery"


def test_device_type_table_with_two_digit_types_and_a_second_copy():
    """B2500 v2 builds list HMF-0..HMJ-14 at a 24-byte stride, then the same
    table again; types past -9 and the copy must not cut it short or repeat."""
    import struct
    from marsemu.profiles import find_type_tables
    names = ["HMF-0", "HMA-1", "HMA-9", "HMA-10", "HMJ-1", "HMJ-14"]
    table = b"".join(n.encode().ljust(24, b"\0") for n in names)
    start = 0x40
    img = bytearray(start) + table + table + bytes(16)
    struct.pack_into("<I", img, 0, 0x08000000 + start)   # code refers to the table
    tabs = find_type_tables(bytes(img), 0x08000000)
    assert tabs and tabs[0][0] == start and tabs[0][1] == 24
    assert tabs[0][2] == names
