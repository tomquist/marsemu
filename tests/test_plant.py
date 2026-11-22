"""Plant models: AFE protocol, FDCAN reception, CAN bus timing, Venus inverter model."""
import struct

from marsemu.core.machine import Machine
from marsemu.periph import stm32
from marsemu.periph.ka495xx import Ka495xx, crc8
from marsemu.plant.battery import LfpPack
from marsemu.plant.canbus import CanBus, Frame, Node, frame_bits
from marsemu.plant.cosim import CoSim
from marsemu.plant.venus3 import VenusMicro


def spi(chip, data):
    return [chip.xfer(b) for b in data]


def test_ka495xx_crc_matches_bms_firmware_frames():
    # frames captured from the Venus E 3.0 BMS (v115) initialisation
    assert crc8(bytes.fromhex("e0020780")) == 0xAE
    assert crc8(bytes.fromhex("e1ae")) == 0xCA
    assert crc8(bytes.fromhex("e10358")) == 0xA3


def test_ka495xx_write_read_and_burst():
    chip = Ka495xx()
    chip._cs(True)
    spi(chip, [0xE0, 0x0B << 1, 0x33, 0x33, crc8([0xE0, 0x16, 0x33, 0x33])])
    chip._cs(False)
    assert chip.regs[0x0B] == 0x3333
    chip._cs(True)
    out = spi(chip, [0xE1, 0x0B << 1, crc8([0xE1, 0x16]), 0, 0, 0])
    chip._cs(False)
    assert out[3:5] == [0x33, 0x33] and out[5] == crc8([0x33, 0x33])
    # burst: registers 9..11, register 9 is the chip id 0x000A
    chip._cs(True)
    hdr = [0xE1, (9 << 1) | 1, 2]
    out = spi(chip, hdr + [crc8(hdr)] + [0] * 7)
    chip._cs(False)
    data = out[4:10]
    assert data == [0x00, 0x0A, 0, 0, 0x33, 0x33]
    assert out[10] == crc8(data)


def test_ka495xx_conversion_flags_and_measure_callback():
    calls = []
    chip = Ka495xx()
    chip.measure = lambda c: (calls.append(1), c.regs.__setitem__(0x28, 1234))
    assert chip.read(0x1C) & 0xE0 == 0xE0 and calls
    chip.write(0x1C, 0x80)                 # clear "done"
    assert chip.regs[0x1C] & 0x80 == 0
    chip.read(0x1C)                         # next scan finished
    assert chip.regs[0x1C] & 0x80 and chip.regs[0x28] == 1234
    chip.write(0x0C, 3)                     # start command self-clears
    assert chip.regs[0x0C] == 0


def _g4():
    m = Machine(hz=170_000_000, sram_size=128 * 1024)
    stm32.build_g4(m, uid=bytes(12))
    return m


def test_fdcan_rx_fifo_and_extended_filter():
    m = _g4()
    can = m.bus.get("FDCAN1")
    ram = can.ram
    can.write32(0x18, 0)                    # leave INIT
    # global filter: reject everything not matched; one extended filter
    can.write32(0x80, (1 << 24) | (2 << 4) | (2 << 2))
    m.bus.write(ram + 0x70, (1 << 29) | 0x4100)          # EFEC=1 (FIFO0), id (control -> node 1)
    m.bus.write(ram + 0x74, (2 << 30) | 0xFF00)          # classic filter, mask
    can.write32(0x54, 1)                                  # RF0NE
    can.write32(0x5C, 1)                                  # line 0
    assert can.inject(0x00014110, b"\x01\x02", True)
    assert not can.inject(0x00025110, b"", True)          # filtered out
    assert can.read32(0x90) & 0x7 == 1
    w0, w1 = m.bus.read(ram + 0xB0), m.bus.read(ram + 0xB4)
    assert w0 & 0x1FFFFFFF == 0x00014110 and w0 & (1 << 30) and (w1 >> 16) & 0xF == 2
    assert m.bus.read(ram + 0xB8) & 0xFFFF == 0x0201
    assert can.level()["FDCAN1_IT0"]
    can.write32(0x94, 0)                                   # acknowledge
    assert can.read32(0x90) & 0x7 == 0


class Rec(Node):
    def __init__(self, name):
        self.name = name
        self.got = []

    def deliver(self, f: Frame):
        self.got.append(f)


def test_canbus_serialises_frames():
    bus = CanBus(500_000)
    a, b = bus.attach(Rec("a")), bus.attach(Rec("b"))
    bus.send(a, 0x123, bytes(8), True, t=1.0)
    bus.send(a, 0x124, bytes(8), True, t=1.0)
    assert not a.got and len(b.got) == 2
    dt = frame_bits(8, True) / 500_000
    assert abs(b.got[0].t - (1.0 + dt)) < 1e-9 and abs(b.got[1].t - (1.0 + 2 * dt)) < 1e-9


def _micro():
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    ctl = bus.attach(Rec("control"))
    micro = bus.attach(VenusMicro(sim, LfpPack(soc=0.5)))
    return sim, bus, ctl, micro


def _bms_limits(bus, ctl, sim, chg_a=100.0, dis_a=100.0, flags=3):
    bus.send(ctl, 0x1803AA01, struct.pack("<HHHBB", 576, int(chg_a * 10), int(dis_a * 10), flags, 0), True, t=sim.now)
    bus.send(ctl, 0x1801AA01, struct.pack("<HhhH", 5280, 0, 250, 500), True, t=sim.now)


def test_micro_telemetry_is_seven_frames_like_the_real_firmware():
    sim, bus, ctl, micro = _micro()
    bus.send(ctl, 0x00014110, bytes(4), True, t=0.0)
    sim.run(0.02)
    ids = [f.ident for f in ctl.got]
    assert ids == [(i << 24) | 0x410110 for i in range(1, 8)]
    payload = b"".join(f.data for f in ctl.got)
    assert len(payload) == 56
    v10 = struct.unpack_from("<H", payload, 0x1E)[0]
    assert 500 < v10 < 560                     # 16S LFP pack voltage x10
    assert struct.unpack_from("<H", payload, 0x12)[0] == 500   # 50.0 Hz


def test_micro_setpoint_is_clamped_by_bms_limits_and_ramps():
    sim, bus, ctl, micro = _micro()
    def tick(now, dt):
        micro.step(now, dt)
        micro.pack.step(dt, micro.pack.current_for_power(micro.dc_power))
    sim.tickers.append(tick)
    _bms_limits(bus, ctl, sim, chg_a=10.0, dis_a=20.0)
    bus.send(ctl, 0x00014101, struct.pack("<i", 2500), True, t=0.0)
    sim.run(0.01)
    echo = [f for f in ctl.got if f.ident == 0x01410101]
    hi, lo = micro.limits()
    assert struct.unpack("<i", echo[-1].data)[0] == int(hi)
    assert 1000 < hi < 1100                      # 20 A x ~52.8 V
    assert -560 < lo < -500                      # 10 A charge
    sim.run(micro.stage.start_delay_s + 2.0)
    assert abs(micro.ac_power - micro.limits()[0]) < 1   # follows the limit as the pack voltage sags
    assert micro.pack.current_a < -15            # discharging
    # limits from the micro's own config (func 0x03) apply too
    bus.send(ctl, 0x00014103, struct.pack("<i", 300), True, t=sim.now)
    bus.send(ctl, 0x00014101, struct.pack("<i", 2500), True, t=sim.now + 0.01)
    sim.run(2.0)
    assert micro.reference == 300 and abs(micro.ac_power - 300) < 1


def test_pack_current_for_power_round_trip():
    p = LfpPack(soc=0.5)
    i = p.current_for_power(-1000.0)
    p.current_a = i
    assert abs(i * p.voltage + 1000.0) < 1.0


def test_micro_acknowledges_with_empty_frames_like_the_firmware():
    sim, bus, ctl, micro = _micro()
    bus.send(ctl, 0x00014102, b"\x01", True, t=0.0)          # backup flag: ack after 7.5 ms, index 0, no data
    bus.send(ctl, 0x00014103, struct.pack("<I", 2000), True, t=0.0)
    sim.run(0.05)
    acks = [(f.ident, f.data) for f in ctl.got]
    assert (0x00410102, b"") in acks and (0x00410103, b"") in acks
    assert micro.discharge_limit_w == 2000


def test_venus_d_inverter_spec():
    from marsemu.plant.venus3 import MICRO_D_116
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    ctl = bus.attach(Rec("control"))
    bus.attach(VenusMicro(sim, LfpPack(capacity_ah=50), spec=MICRO_D_116))
    bus.send(ctl, 0x00014110, bytes(4), True, t=0.0)
    bus.send(ctl, 0x00014114, bytes(4), True, t=0.1)          # not handled by Micro 116
    bus.send(ctl, 0x000141CB, bytes(4), True, t=0.2)
    sim.run(0.3)
    tele = [f for f in ctl.got if f.ident & 0xFFFFFF == 0x410110]
    assert len(tele) == 6                                       # 48 bytes
    assert not [f for f in ctl.got if f.ident & 0xFF == 0x14]
    ver = [f for f in ctl.got if f.ident & 0xFF == 0xCB][0]
    assert struct.unpack("<HH", ver.data[:4]) == (0x0D, 116)


def test_mppt_block_and_limit():
    from marsemu.plant.venusd import MpptNode
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    ctl = bus.attach(Rec("control"))
    pack = LfpPack(capacity_ah=50)
    mppt = bus.attach(MpptNode(sim, pack, [600, 300]))
    sim.tickers.append(lambda now, dt: mppt.step(now, dt))
    sim.run(1)
    assert sum(mppt.pv_w) == 0                                 # off until the control allows PV
    bus.send(ctl, 0x00012102, struct.pack("<I", 1), True, t=sim.now)
    bus.send(ctl, 0x01012102, struct.pack("<HHHH", 0, 2600, 576, 0x1919), True, t=sim.now + 0.001)
    sim.run(5)
    assert abs(sum(mppt.pv_w) - 900) < 1
    bus.send(ctl, 0x00012103, bytes(4), True, t=sim.now)
    sim.run(0.02)
    frames = [f for f in ctl.got if f.ident & 0xFFFFFF == 0x210103]
    assert [f.ident >> 24 for f in frames] == list(range(1, 8))
    w = struct.unpack("<28H", b"".join(f.data for f in frames))
    assert w[6] == 6000 and w[9] == 3000 and w[4] == 400       # 0.1 W / 0.1 V
    # control limits PV to 500 W
    bus.send(ctl, 0x00012102, struct.pack("<I", 1), True, t=sim.now)
    bus.send(ctl, 0x01012102, struct.pack("<HHHH", 0, 500, 576, 0x1919), True, t=sim.now + 0.001)
    sim.run(5)
    assert mppt.limit_w == 500 and abs(sum(mppt.pv_w) - 500) < 1


def test_sh36735_protocol():
    from marsemu.periph.sh36735 import Sh36735, crc8_07
    chip = Sh36735()
    chip._cs(True)
    out = spi(chip, [0x01, 0x40, 0x30, crc8_07([0x01, 0x40, 0x30]), 0x00])
    chip._cs(False)
    assert chip.regs[0x40] == 0x30 and out == [0xFF, 0x01, 0x40, 0x30, 0xA5]
    chip.set16(0x69, 0x5280)
    chip._cs(True)
    out = spi(chip, [0x02, 0x69, 0x02, 0, 0, 0, 0])
    chip._cs(False)
    assert out[:4] == [0xFF, 0x02, 0x69, 0x02] and out[4:6] == [0x52, 0x80]
    assert out[6] == crc8_07([0xFF, 0x02, 0x69, 0x02, 0x52, 0x80])


class FakeGpio:
    def __init__(self):
        self.watchers, self.inputs, self.odr = [], {}, {10: 1, 11: 1}

    def set(self, pin, lvl):
        if self.odr[pin] != lvl:
            self.odr[pin] = lvl
            for w in self.watchers:
                w(pin, lvl)

    def get(self, pin):
        return self.inputs.get(pin, self.odr[pin])


def _bb_byte(g, b):
    for i in range(8):
        g.set(11, (b >> (7 - i)) & 1)
        g.set(10, 1)
        g.set(10, 0)
    g.set(11, 1)
    g.set(10, 1)
    ack = g.get(11) == 0
    g.set(10, 0)
    return ack


def _bb_read(g, last):
    v = 0
    g.set(11, 1)
    for _ in range(8):
        g.set(10, 1)
        v = (v << 1) | g.get(11)
        g.set(10, 0)
    g.set(11, 1 if last else 0)
    g.set(10, 1)
    g.set(10, 0)
    g.set(11, 1)
    return v


def test_bitbanged_i2c_eeprom():
    from marsemu.periph.i2c_gpio import GpioI2cBus
    from marsemu.periph.n32g45x import I2cEeprom
    g = FakeGpio()
    bus = GpioI2cBus(g, 10, g, 11)
    ee = I2cEeprom(0x50, 65536, 2, 128)
    bus.devices[0x50] = ee

    def start():
        g.set(11, 1); g.set(10, 1); g.set(11, 0); g.set(10, 0)

    def stop():
        g.set(11, 0); g.set(10, 1); g.set(11, 1)
    start()
    assert _bb_byte(g, 0xA0) and _bb_byte(g, 0x01) and _bb_byte(g, 0xF6)
    assert _bb_byte(g, 0x12) and _bb_byte(g, 0x34)
    stop()
    assert ee.mem[0x1F6:0x1F8] == b"\x12\x34"
    start()
    assert _bb_byte(g, 0xA0) and _bb_byte(g, 0x01) and _bb_byte(g, 0xF6)
    start()                                                   # repeated start, read
    assert _bb_byte(g, 0xA1)
    assert [_bb_read(g, False), _bb_read(g, True)] == [0x12, 0x34]
    stop()
    start()
    assert not _bb_byte(g, 0xA4)                              # no device at 0x52: NACK
    stop()


def test_jupiter_inverter_framing_and_setpoint():
    from marsemu.plant.jupiter import JupiterInverter, PylonBms
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    ctl = bus.attach(Rec("control"))
    pack = LfpPack()
    inv = bus.attach(JupiterInverter(sim, pack))
    bus.attach(PylonBms(sim, pack))

    def tick(now, dt):
        inv.step(now, dt)
        pack.step(dt, pack.current_for_power(inv.dc_power))
    sim.tickers.append(tick)
    bus.send(ctl, 0x305, bytes(8), False, t=0.0)                     # BMS limits
    bus.send(ctl, 0x00011101, bytes([0, 0, 0, 0, 1]), True, t=0.01)  # start
    bus.send(ctl, 0x00011102, struct.pack("<I", 1), True, t=0.2)
    bus.send(ctl, 0x01011102, struct.pack("<HhHH", 0, 800, 0, 1), True, t=0.201)
    sim.run(inv.stage.start_delay_s + 2)
    assert inv.reference == 800 and abs(inv.ac_power - 800) < 1
    ctl.got.clear()
    bus.send(ctl, 0x00011103, bytes(4), True, t=sim.now)
    sim.run(0.02)
    frames = [f for f in ctl.got if f.ident & 0xFFFFFF == 0x110103]
    assert [f.ident >> 24 for f in frames] == [0, 1, 2, 3, 4]
    assert struct.unpack("<H", frames[0].data[:2])[0] == 4
    w = struct.unpack("<16H", b"".join(f.data for f in frames[1:]))
    assert w[3] == 2300 and w[6] == 5000 and abs(struct.unpack("<h", struct.pack("<H", w[8]))[0] - 800) < 5


def test_jupiter_mppt_block_carries_battery_voltage_and_current():
    from marsemu.plant.jupiter import JupiterMppt
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    ctl = bus.attach(Rec("control"))
    pack = LfpPack()
    mppt = bus.attach(JupiterMppt(sim, pack, [300, 300], version=206))
    mppt.limit_w = None
    for _ in range(300):                                             # 3 s at 10 ms
        mppt.step(sim.now, 0.01)
    bus.send(ctl, 0x00012103, bytes(4), True, t=0.0)
    sim.run(0.02)
    frames = [f for f in ctl.got if f.ident & 0xFFFFFF == 0x210103]
    assert [f.ident >> 24 for f in frames] == list(range(8))
    assert struct.unpack("<H", frames[0].data[:2])[0] == 7
    w = struct.unpack("<28H", b"".join(f.data for f in frames[1:]))
    assert w[4] == 400 and w[6] == 3000                              # PV1: 40.0 V, 300.0 W
    assert w[24] == int(pack.voltage * 10)
    assert w[25] == int(mppt.dc_power / pack.voltage * 10) and w[25] > 0
    assert (w[26], w[27]) == (JupiterMppt.REF_WORD, JupiterMppt.PE_WORD)


def test_jupiter_bms_answers_the_c_e_poll_and_the_c_plus_broadcast():
    from marsemu.plant.jupiter import CPLUS_BMS_REQUEST, PylonBms
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    ctl = bus.attach(Rec("control"))
    pack = LfpPack()
    pack.step(1.0, -12.3)                                            # some current to report
    bms = bus.attach(PylonBms(sim, pack))
    sim.run(2.0)
    assert not ctl.got                                               # nothing unasked
    bus.send(ctl, 0x305, bytes(8), False, t=sim.now)                 # Jupiter C/E
    bus.send(ctl, CPLUS_BMS_REQUEST, bytes.fromhex("0000580000000000"), True, t=sim.now + 0.5)  # C Plus
    sim.run(1.0)
    v356 = [f for f in ctl.got if not f.ext and f.ident == 0x356]
    assert len(v356) == 2 and bms.answers == 2
    v, i = struct.unpack_from("<Hh", v356[-1].data)
    assert v == round(pack.voltage * 100) and i == round(pack.current_a * 10) == -123


def test_jupiter_inverter_reads_the_c_plus_bms_frames():
    from marsemu.plant.jupiter import JupiterInverter
    sim = CoSim()
    bus = CanBus()
    bus._now = lambda: sim.now
    bms = bus.attach(Rec("bms"))
    inv = bus.attach(JupiterInverter(sim, LfpPack()))
    # captured from the C Plus BMS V37: 52.78 V, +21.9 A, SOC 50 %; 58.4 V, 50 A / 50 A
    bus.send(bms, 0x1801AA01, bytes.fromhex("9e14db0000003200"), True, t=0.0)
    bus.send(bms, 0x1821AA01, bytes.fromhex("4802f401f4010225"), True, t=0.001)
    bus.send(bms, 0x1801AA02, bytes.fromhex("0000000000000000"), True, t=0.002)   # another pack
    assert (inv.bms_v, inv.bms_i, inv.bms_soc) == (52.78, 21.9, 50.0)
    assert (inv.bms_chg_v, inv.bms_chg_i, inv.bms_dis_i, inv.bms_flags) == (58.4, 50.0, 50.0, 3)


class _FakeRemote:
    """Stands in for a RemoteMcu: records the order of start/finish calls."""

    def __init__(self, name, log):
        self.name, self.log = name, log

    def start(self, target):
        self.log.append(("start", self.name, round(target, 6)))

    def finish(self):
        self.log.append(("finish", self.name))

    def close(self):
        pass


def test_run_lockstep_shares_one_timeline():
    from marsemu.plant.cosim import run_lockstep
    a, b = CoSim(slice_s=0.005), CoSim(slice_s=0.02)
    seen = []
    a.at(0.012, lambda: seen.append(("a", a.now, b.now)))
    b.at(0.012, lambda: seen.append(("b", a.now, b.now)))
    ticks = {"a": [], "b": []}
    a.tickers.append(lambda now, dt: ticks["a"].append((now, b.now)))
    b.tickers.append(lambda now, dt: ticks["b"].append(now))
    run_lockstep([a, b], 0.05)
    assert abs(a.now - 0.05) < 1e-9 and abs(b.now - 0.05) < 1e-9
    # the shortest slice is used for both, and neither runs ahead of the other
    assert len(ticks["b"]) == 10
    assert all(abs(ta - tb) < 0.0051 for ta, tb in ticks["a"])
    assert [s[0] for s in seen] == ["a", "b"]
    assert all(abs(x - y) < 0.0051 for _, x, y in seen)
    run_lockstep([a, b], 0.01, slice_s=0.01)
    assert len(ticks["b"]) == 11


def test_run_lockstep_overlaps_remote_mcus_of_all_sims():
    from marsemu.plant.cosim import run_lockstep
    log = []
    a, b = CoSim(slice_s=0.01), CoSim(slice_s=0.01)
    a.add_remote(_FakeRemote("a", log))
    b.add_remote(_FakeRemote("b", log))
    run_lockstep([a, b], 0.01)
    assert log == [("start", "a", 0.01), ("start", "b", 0.01), ("finish", "a"), ("finish", "b")]


def test_run_lockstep_refuses_sims_at_different_times():
    import pytest
    from marsemu.plant.cosim import run_lockstep
    a, b = CoSim(), CoSim()
    a.run(0.01)
    with pytest.raises(ValueError):
        run_lockstep([a, b], 0.01)


def test_run_lockstep_until_stops_all():
    from marsemu.plant.cosim import run_lockstep
    a, b = CoSim(slice_s=0.01), CoSim(slice_s=0.01)
    run_lockstep([a, b], 1.0, until=lambda: a.now >= 0.0995)
    assert abs(a.now - 0.1) < 1e-9 and abs(b.now - 0.1) < 1e-9


def test_house_with_several_batteries():
    import pytest
    from marsemu.plant.venus3_cli import House
    h = House({"A": 100, "B": -500, "C": 300})
    a = h.add_battery("a", "A")
    c = h.add_battery("c", "C")
    a[1], c[1] = 80.0, -200.0
    assert h.grid() == {"A": 20.0, "B": -500.0, "C": 500.0}
    with pytest.raises(ValueError):
        h.add_battery("a", "B")
    with pytest.raises(ValueError):
        h.add_battery("x", "D")
    # the single-battery fields still work alongside
    h.battery_phase, h.battery_w = "B", -100.0
    assert h.grid()["B"] == -400.0


class _FakeNet:
    def __init__(self):
        self.udp_handler = None
        self.replies = []

    def udp_reply(self, sock, data, src):
        self.replies.append((sock, data, src))


class _FakeDevice:
    """Enough of a whole device for connect_house: a sim, a control
    emulator with a module, and its AC power."""

    def __init__(self, ip):
        from types import SimpleNamespace
        from marsemu.fc41d.module import Fc41dConfig
        self.sim = CoSim(slice_s=0.01)
        self.ac_power = 0.0
        net = _FakeNet()
        self.control = SimpleNamespace(fc41d=SimpleNamespace(net=net, cfg=Fc41dConfig(ip=ip),
                                                             later=lambda ms, fn, tag: fn()))
        self.net = net


def test_connect_house_several_devices_one_meter_function():
    from types import SimpleNamespace
    from marsemu.plant.venus3_cli import House, connect_house, run_together
    house = House({"A": 0, "B": 300, "C": 0})
    d1, d2 = _FakeDevice("192.168.1.51"), _FakeDevice("192.168.1.52")
    polls = []

    def meter(data, addr):
        polls.append((data, addr, house.grid()))
        return b"reply"
    m1 = connect_house(d1, house, meter=meter, phase="A", name="one")
    m2 = connect_house(d2, house, meter=meter, phase="B")
    assert set(house.batteries) == {"one", "bat2"}
    d1.ac_power, d2.ac_power = -400.0, 250.0
    run_together([d1, d2], 0.05)
    assert house.grid() == {"A": 400.0, "B": 50.0, "C": 0.0}
    # each device's polls reach the same function with its own address
    sock = SimpleNamespace(local_port=0)
    d1.net.udp_handler(sock, b"poll1", ("192.168.1.10", 12345))
    d2.net.udp_handler(sock, b"poll2", ("192.168.1.10", 12345))
    assert [(p[0], p[1]) for p in polls] == [(b"poll1", ("192.168.1.51", 12345)),
                                            (b"poll2", ("192.168.1.52", 12345))]
    assert d1.net.replies[0][1] == b"reply" and d2.net.replies[0][1] == b"reply"
    assert m1.polls == 1 and m2.polls == 1



def test_only_polls_to_the_ct_port_reach_the_meter():
    """A CT002/CT003 answers on UDP 12345 only; a Venus E Mini's broadcast to
    other Minis (60606) must not be answered as a meter poll."""
    from types import SimpleNamespace as NS

    from marsemu.plant.venus3_cli import House, connect_house

    replies = []
    net = NS(udp_handler=None, udp_reply=lambda sock, data, dest: replies.append((data, dest)))
    dev = NS(control=NS(fc41d=NS(net=net, cfg=NS(ip="192.168.1.50"))), sim=NS(tickers=[]), ac_power=0.0)
    m = connect_house(dev, House({"A": 100.0}), meter=lambda data, src: b"answer")
    net.udp_handler(NS(local_port=22222), b"poll", ("192.168.1.255", 12345))
    net.udp_handler(NS(local_port=22222), b"hello minis", ("192.168.1.255", 60606))
    assert m.polls == 1 and replies == [(b"answer", ("192.168.1.255", 12345))]


def test_b2500_setup_waits_out_the_firmware_hold_after_subscribing():
    """HMJ-2 110/116 hold a command arriving within 3 s of their subscription
    and then drop it: the setup's meter command (cd=27) came 2 s after it and
    left HMJ-2 110 without a CT (ct_t=7, no polls)."""
    from types import SimpleNamespace
    from marsemu.plant.b2500 import B2500
    from marsemu.plant.cosim import CoSim
    from marsemu.plant.venus3_cli import configure_self_consumption
    sim = CoSim()
    sent, handlers = [], {}
    control = SimpleNamespace(image=b"...meter=...", on=lambda ev, fn: handlers.setdefault(ev, fn),
                              mqtt_inject=lambda t, p: sent.append((sim.now, p)))
    dev = B2500.__new__(B2500)
    dev.sim, dev.control, dev.plant = sim, control, SimpleNamespace(v1=False)
    configure_self_consumption(dev)
    handlers["mqtt_subscribe"]("marstek_energy/HMJ-2/App/x/ctrl")
    sim.run(20)
    assert [p for _, p in sent] == ["cd=27,meter=3,mac=112233445566", "cd=18,md=1"]
    assert sent[0][0] > 3.0
