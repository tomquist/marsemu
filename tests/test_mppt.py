"""The F2803x eCAN model and the MPPT board model (no firmware needed)."""
import struct

import pytest

from marsemu.c28x.f2803x import build
from marsemu.c28x.f2803x_can import (CANGIF1, CANGIM, CANMC, CANMD, CANME, CANMIL, CANMIM, CANOPC, CANRML,
                                     CANRMP, CANTA, CANTRS, LAM_BASE, MBOX_BASE)
from marsemu.c28x.machine import C28xMachine


def can_machine():
    m = C28xMachine()
    build(m)
    m.reset()
    return m, m.get("ECANA")


def w32(can, off, v):
    """A 32-bit register write the way the CPU does it: low word, then high word."""
    can.write16(can.base + off, v & 0xFFFF)
    can.write16(can.base + off + 1, v >> 16)


def r32(can, off):
    return can.read16(can.base + off) | can.read16(can.base + off + 1) << 16


def mbox(can, k, msgid, ctrl=8, mdl=0, mdh=0):
    b = MBOX_BASE + 8 * k
    for j, v in enumerate((msgid, ctrl, mdl, mdh)):
        w32(can, b + 2 * j, v)


def configured(dbo=True):
    """eCAN set up like the MPPT firmware: mailboxes 0-15 transmit, 16-31
    receive frames for node 0x21 (bits 15:8, mask on the rest), interrupts
    on line 1."""
    m, can = can_machine()
    assert r32(can, 0x18) & 0x10                       # CCE: in configuration mode after reset
    w32(can, CANMC, 0x1000 | (0x400 if dbo else 0) | 0x2000)
    w32(can, 0x16, 0x000500AA)                         # 500 kbit/s at 60 MHz
    w32(can, CANMC, (0x400 if dbo else 0) | 0x2000)    # leave configuration mode
    assert not r32(can, 0x18) & 0x10
    for k in range(32):
        if k < 16:
            mbox(can, k, 0x80000000)
        else:
            mbox(can, k, 0xC0002100)
            w32(can, LAM_BASE + 2 * k, 0x1FFF00FF)
    w32(can, CANMD, 0xFFFF0000)
    w32(can, CANME, 0xFFFFFFFF)
    w32(can, CANMIM, 0xFFFF0000)
    w32(can, CANMIL, 0xFFFFFFFF)
    w32(can, CANGIM, 0x0002)
    return m, can


def test_ecan_sends_a_mailbox_and_flags_it_when_the_frame_is_done():
    m, can = configured()
    sent = []
    can.tx_listeners.append(lambda i, d, x: sent.append((m.clock.cycles, i, d, x)))
    mbox(can, 3, 0x80000000 | 0x00210103, ctrl=4, mdl=0x00000007)
    w32(can, CANTRS, 1 << 3)
    assert r32(can, CANTRS) == 1 << 3 and not sent
    m.clock.cycles += 10
    m.clock.run_due()
    assert sent and sent[0][1:] == (0x00210103, b"\x07\x00\x00\x00", True)    # DBO=1: least significant byte first
    assert not r32(can, CANTA)
    m.clock.cycles += 60_000                                                 # well past the frame time
    m.clock.run_due()
    assert r32(can, CANTA) == 1 << 3 and r32(can, CANTRS) == 0
    w32(can, CANTA, 1 << 3)                                                  # write 1 to clear
    assert r32(can, CANTA) == 0


def test_ecan_receives_into_the_highest_matching_mailbox_and_interrupts():
    m, can = configured()
    raised = []
    pie = m.pie
    orig = pie.raise_irq
    pie.raise_irq = lambda g, i: (raised.append((g, i)), orig(g, i))
    assert can.inject(0x00012103, b"\x01\x02\x03\x04", True)
    assert r32(can, CANRMP) == 1 << 31 and raised == [(9, 6)]                # ECAN1INT
    assert r32(can, CANGIF1) & 0x801F == 0x8000 | 31                         # GMIF1, MIV1 = 31
    b = MBOX_BASE + 8 * 31
    assert r32(can, b) == 0xC0012103 and r32(can, b + 2) & 0xF == 4          # the received id (AME kept), DLC
    assert r32(can, b + 4) == 0x04030201                                     # DBO=1
    # the next one goes to mailbox 31 again (no overwrite protection) and is flagged lost
    assert can.inject(0x00012102, b"\x00" * 8, True)
    assert r32(can, CANRML) == 1 << 31
    w32(can, CANRMP, 1 << 31)
    assert r32(can, CANRMP) == 0 and r32(can, CANRML) == 0
    # with overwrite protection on 31 the next frame lands in 30
    w32(can, CANOPC, 1 << 31)
    can.inject(0x00012103, b"", True)
    can.inject(0x00012103, b"", True)
    assert r32(can, CANRMP) == 3 << 30
    # frames for other nodes, and standard frames, are not taken
    w32(can, CANRMP, 0xFFFFFFFF)
    assert can.inject(0x00011103, b"", True) is False
    assert can.inject(0x123, b"", False) is False


def test_ecan_byte_order_msb_first():
    m, can = configured(dbo=False)
    can.inject(0x00012103, b"\x01\x02\x03\x04\x05", True)
    b = MBOX_BASE + 8 * 31
    assert r32(can, b + 4) == 0x01020304 and r32(can, b + 6) == 0x05000000
    sent = []
    can.tx_listeners.append(lambda i, d, x: sent.append(d))
    mbox(can, 0, 0x80000000 | 0x00210103, ctrl=6, mdl=0x11223344, mdh=0x55660000)
    w32(can, CANTRS, 1)
    m.clock.cycles += 10
    m.clock.run_due()
    assert sent == [bytes.fromhex("112233445566")]


def test_ecan_ignores_the_bus_in_configuration_mode():
    m, can = can_machine()
    assert can.inject(0x00012103, b"", True) is False


# ------------------------------------------------------------------ the board model
class FakeEpwm:
    def __init__(self, a=0.0, b=0.0, on=True):
        self.a, self.b, self.on = a, b, on

    def duty(self, ch="A"):
        return self.a if ch == "A" else self.b

    def outputs_enabled(self):
        return self.on


def test_boost_stage_holds_the_string_at_one_minus_d_times_the_battery():
    from marsemu.plant.b2500 import PvArray
    from marsemu.plant.mppt import BoostInput
    arr = PvArray(400, voc=48.0)
    pwm = FakeEpwm()
    s = BoostInput(arr, pwm)
    s.step(0.01, 53.0)
    assert s.v == pytest.approx(48.0) and s.i == 0.0                         # stage off: open circuit
    pwm.a, pwm.b = 0.27, 0.73
    for _ in range(50):
        s.step(0.001, 53.0)
    assert s.v == pytest.approx(0.73 * 53.0, abs=0.01)
    assert s.p == pytest.approx(arr.current(s.v) * s.v) and 380 < s.p <= 400
    # duty too small with the rectifier on: the battery drives current back into the string
    pwm.a = 0.0
    for _ in range(50):
        s.step(0.001, 53.0)
    assert s.i < 0
    # the same without the rectifier: the diode blocks it
    pwm.b = 0.0
    pwm.a = 0.01
    for _ in range(50):
        s.step(0.001, 53.0)
    assert s.i == 0.0 and s.v == pytest.approx(48.0)


def test_mppt_plant_adc_scales_and_string_order():
    """The board's sensors as the firmware's adcshow reads them; the
    firmware's PV1 is the control's string 4."""
    from marsemu.plant import mppt

    class Emu:
        pass
    m = C28xMachine()
    build(m)
    m.reset()
    emu = Emu()
    emu.machine = m
    plant = mppt.MpptPlant(emu, [400, 0, 0, 300], voc=48.0, v_bat=52.0)
    mppt.ADC_NOISE = 0
    try:
        m.clock.cycles += 10 ** 6
        vals = [m.adc.source(ch) for ch in range(16)]
    finally:
        mppt.ADC_NOISE = 2
    assert vals[mppt.CH_VREF] == 2048
    assert vals[mppt.CH_BAT_V] * mppt.BAT_V_PER_COUNT == pytest.approx(52.0, abs=0.02)
    assert vals[mppt.CH_BAT_I] == 2048                                       # nothing flowing
    # firmware PV1 (B0) is string 4 (300 W), firmware PV4 (B6) string 1 (400 W): both at open circuit
    assert vals[8] * mppt.PV_V_PER_COUNT == pytest.approx(48.0, abs=0.02)
    assert vals[14] * mppt.PV_V_PER_COUNT == pytest.approx(48.0, abs=0.02)
    assert vals[10] == vals[12] == 0                                         # no light on strings 2 and 3
    assert plant.inputs[0].array is plant.arrays[3] and plant.inputs[3].array is plant.arrays[0]


def test_mppt_view_decodes_the_controls_commands_and_the_version():
    from marsemu.plant.canbus import Frame
    from marsemu.plant.mppt import _MpptView
    v = _MpptView()
    v._init_view(None, [100, 200])
    v._watch(Frame(0x00012102, b"\x01\x00\x00\x00", True, None, 1.0))
    v._watch(Frame(0x01012102, struct.pack("<HHHH", 0, 2650, 576, 0x1919), True, None, 1.001))
    assert v.limit_w == 2650 and v.vmax == 57.6 and v.mode == 0
    v._watch(Frame(0x00012103, bytes(4), True, None, 1.1))
    assert v.polls == 1
    v._watch(Frame(0x002101CB, b"\x01\x00\x00\x00", True, None, 1.2))
    v._watch(Frame(0x012101CB, bytes.fromhex("65006800ffff0100"), True, None, 1.2))
    assert v.version == 104
    # the MPPT's own frames are not commands
    v._watch(Frame(0x00210103, b"\x07\x00\x00\x00", True, None, 1.3))
    assert v.polls == 1 and v.available == [100.0, 200.0, 0.0, 0.0]
