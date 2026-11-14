"""Boot real firmware images to the cloud-connected state.

These take 30-120 s each and need the images, so they only run with
``pytest -m firmware`` (or ``-m firmware -k b2500``).
"""
import glob
import os

import pytest

from marsemu.emulator import Emulator

#: firmware images laid out as <device type>/<version>/<file>.bin, e.g. HMA-1/V232/....bin
FW = os.environ.get("MARSEMU_FIRMWARE", "firmware")
#: a firmware archive with the Venus E 3.0/D/A control and BMS images and the CT00x
#: (e.g. sphings79/marstek-firmware-archiv's firmwares directory)
EXTRA = os.environ.get("MARSEMU_ARCHIVE", "")

CASES = [
    # (id, glob, virtual seconds budget)
    ("b2500-v2-HMA-V232", f"{FW}/HMA-1/V232/*.bin", 40),
    ("b2500-v2-HMA-V255", f"{FW}/HMA-1/V255/*V255.2.bin", 40),
    ("b2500-v1-HMB-V144", f"{FW}/HMB-1/V144/*.bin", 40),
    ("b2500-HMJ-V118", f"{FW}/HMJ-2/V118/*.bin", 40),
    ("jupiter-HMN-V142", f"{FW}/HMN-1/V142/*.bin", 70),
    ("venus-HMG-V154", f"{FW}/HMG-50/V154/*.bin", 70),
]
if EXTRA:
    CASES += [
        ("venus3-VNSE3-151", f"{EXTRA}/**/VNSE3-0/Control/151/*.bin", 70),
        ("ct002-HME4-124", f"{EXTRA}/**/HME-4/124/*.bin", 90),
        ("jupiter-JPLS-143", f"{EXTRA}/**/JPLS-8H/143/*.bin", 70),
    ]


@pytest.mark.firmware
@pytest.mark.parametrize("name,pattern,budget", CASES, ids=[c[0] for c in CASES])
def test_reaches_mqtt(name, pattern, budget):
    files = sorted(glob.glob(pattern, recursive=True))
    if not files:
        pytest.skip(f"firmware not available: {pattern}")
    emu = Emulator(files[0])
    subs, pubs = [], []
    emu.on("mqtt_subscribe", subs.append)
    emu.on("mqtt_publish", lambda t, p: pubs.append(t))
    emu.run(seconds=budget, until=lambda: bool(subs))
    assert subs, f"{name}: no MQTT subscription after {emu.time:.1f}s; AT tail:\n" + emu.fc41d.dump_transcript(20)
    assert emu.machine.reset_count == 1, "unexpected resets"
    topic = subs[0]
    assert "/App/" in topic and topic.endswith("/ctrl")


def _venus3_images():
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    ctl = sorted(glob.glob(f"{EXTRA}/**/VNSE3-0/Control/151/*.bin", recursive=True))
    bms = sorted(glob.glob(f"{EXTRA}/**/VNSE3-0/BMS/115/*.bin", recursive=True))
    if not ctl or not bms:
        pytest.skip("Venus E 3.0 control 151 / BMS 115 not in the archive")
    return ctl[0], bms[0]


@pytest.mark.firmware
def test_venus3_bms_and_inverter_on_can():
    """Control and BMS firmware talk over CAN with the inverter model."""
    from marsemu.plant.venus3 import VenusE3
    ctl, bms = _venus3_images()
    v = VenusE3(ctl, bms)
    try:
        v.run(15)
        mi = v.micro
        assert any(func == 0x10 for _, func, _ in mi.log), "control never polled the inverter"
        assert 50 < mi.bms_soc < 57, "BMS SOC should come from the pack's cell voltages (LFP at ~50 %)"
        hi, lo = mi.limits()
        assert hi > 2500 and lo < -2500, "BMS charge/discharge limits not received"
        assert v.mos().get("discharge"), "BMS did not close the discharge MOSFET"
    finally:
        v.close()


@pytest.mark.firmware
def test_venus3_steers_on_ct_reading():
    """Self-consumption: the control firmware detects its phase and covers the house load."""
    from marsemu.plant.venus3 import VenusE3
    from marsemu.plant.venus3_cli import CtMeter, House
    ctl, bms = _venus3_images()
    v = VenusE3(ctl, bms)
    house = House({"A": 600.0})
    v.sim.tickers.append(lambda now, dt: setattr(house, "battery_w", v.ac_power))
    v.control.fc41d.net.udp_handler = CtMeter(v.control, house, "112233445566")
    subs = []
    v.control.on("mqtt_subscribe", subs.append)
    try:
        v.run(70, until=lambda: bool(subs))
        assert subs
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=2,md=0")
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=18,meter=3,mac=112233445566")
        v.run(160, until=lambda: abs(house.grid()["A"]) < 20 and v.ac_power > 500)
        assert abs(house.grid()["A"]) < 20, f"grid not balanced: {house.grid()} battery {v.ac_power:.0f} W"
    finally:
        v.close()


@pytest.mark.firmware
def test_venus_d_with_pv_covers_the_load():
    """Venus D: control 151 + BMS 118 + inverter and MPPT models; PV covers the house."""
    from marsemu.plant.venusd import VenusD
    from marsemu.plant.venus3_cli import CtMeter, House
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    ctl = sorted(glob.glob(f"{EXTRA}/**/VNSD-0/Control/151/*.bin", recursive=True))
    bms = sorted(glob.glob(f"{EXTRA}/**/VNSD-0/BMS/118/*.bin", recursive=True))
    if not ctl or not bms:
        pytest.skip("Venus D control 151 / BMS 118 not in the archive")
    v = VenusD(ctl[0], bms[0], pv=[600, 0, 0, 0])
    house = House({"A": 400.0})
    v.sim.tickers.append(lambda now, dt: setattr(house, "battery_w", v.ac_power))
    v.control.fc41d.net.udp_handler = CtMeter(v.control, house, "112233445566")
    subs = []
    v.control.on("mqtt_subscribe", subs.append)
    try:
        v.run(70, until=lambda: bool(subs))
        assert subs
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=2,md=0")
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=18,meter=3,mac=112233445566")
        v.run(160, until=lambda: abs(house.grid()["A"]) < 20 and v.ac_power > 300)
        assert abs(house.grid()["A"]) < 20, f"grid not balanced: {house.grid()} battery {v.ac_power:.0f} W"
        assert v.pack.current_a > 0, "PV surplus should charge the battery"
        assert v.mppt.limit_w is not None and v.mppt.polls > 10
    finally:
        v.close()


@pytest.mark.firmware
def test_venus_a_with_pv_covers_the_load():
    """Venus A: control 151 + BMS 115 (SH36735, 13 cells) + inverter/MPPT model."""
    from marsemu.plant.venusa import VenusA
    from marsemu.plant.venus3_cli import CtMeter, House
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    ctl = sorted(glob.glob(f"{EXTRA}/**/VNSA-0/Control/151/*.bin", recursive=True))
    bms = sorted(glob.glob(f"{EXTRA}/**/VNSA-0/BMS/115/*.bin", recursive=True))
    if not ctl or not bms:
        pytest.skip("Venus A control 151 / BMS 115 not in the archive")
    v = VenusA(ctl[0], bms[0], pv=[600, 0, 0, 0])
    house = House({"A": 400.0})
    v.sim.tickers.append(lambda now, dt: setattr(house, "battery_w", v.ac_power))
    v.control.fc41d.net.udp_handler = CtMeter(v.control, house, "112233445566")
    subs = []
    v.control.on("mqtt_subscribe", subs.append)
    try:
        v.run(70, until=lambda: bool(subs))
        assert subs
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=2,md=0")
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=18,meter=3,mac=112233445566")
        v.run(160, until=lambda: abs(house.grid()["A"]) < 20 and v.ac_power > 300)
        assert abs(house.grid()["A"]) < 20, f"grid not balanced: {house.grid()} battery {v.ac_power:.0f} W"
        assert v.pack.current_a > 0, "PV surplus should charge the battery"
        hi, lo = v.micro.limits()
        assert hi == 1450 and lo == -1450, "Venus A limits"
        assert 40 < v.micro.bms_v < 46, "13-cell pack voltage from the BMS"
    finally:
        v.close()


@pytest.mark.firmware
@pytest.mark.parametrize("model,bms_version", [("VNSD-0", "118"), ("VNSA-0", "115")], ids=["venus-d", "venus-a"])
def test_venus_stack_of_two_packs(model, bms_version):
    """Two packs, each with its own BMS: the first claims master and codes the
    second over the coding line; the control reports both over Modbus."""
    import socket
    import struct
    import threading
    from marsemu import LfpPack, VenusA, VenusD, House, connect_house, configure_self_consumption
    from marsemu.modbus import ModbusGateway
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    ctl = sorted(glob.glob(f"{EXTRA}/**/{model}/Control/151/*.bin", recursive=True))
    bms = sorted(glob.glob(f"{EXTRA}/**/{model}/BMS/{bms_version}/*.bin", recursive=True))
    if not ctl or not bms:
        pytest.skip(f"{model} control 151 / BMS {bms_version} not in the archive")
    cls = VenusD if model == "VNSD-0" else VenusA
    packs = [LfpPack(cells_n=cls.CELLS, capacity_ah=cls.CAPACITY_AH, soc=s) for s in (0.7, 0.3)]
    v = cls(ctl[0], bms[0], packs=packs)
    house = House({"A": 400.0})
    connect_house(v, house)
    configure_self_consumption(v)
    gw = ModbusGateway(v.control, port=0, host="127.0.0.1")
    regs = {}

    def client():
        with socket.create_connection(("127.0.0.1", gw.port), timeout=60) as s:
            for tid, (addr, n) in enumerate(((32109, 1), (34000, 3), (34100, 3)), 1):
                s.sendall(struct.pack(">HHHBBHH", tid, 0, 6, 1, 3, addr, n))
                _, _, length, _ = struct.unpack(">HHHB", s.recv(7))
                body = s.recv(length - 1)
                regs[addr] = struct.unpack(f">{n}h", body[2:2 + 2 * n]) if body[0] == 3 else body
    try:
        v.run(200, until=lambda: abs(house.grid()["A"]) < 20 and v.ac_power > 300)
        assert abs(house.grid()["A"]) < 20, f"grid not balanced: {house.grid()} battery {v.ac_power:.0f} W"
        t = threading.Thread(target=client, daemon=True)
        t.start()
        v.run(20, until=lambda: len(regs) == 3)
        t.join(5)
        assert regs.get(32109) == (2,), f"pack count: {regs}"
        for addr in (34000, 34100):
            assert 4000 < regs[addr][0] < 6000, f"pack voltage (0.01 V) at {addr}: {regs}"
        assert packs[0].current_a < 0, "the fuller pack discharges"
    finally:
        gw.close()
        v.close()


@pytest.mark.firmware
def test_venus_ce_power_stage_syncs_and_connects():
    """Venus C/E (HMG-50): the control MCU runs the power stage itself.  Its
    PLL locks on the emulated grid, the LLC charges the DC bus from the
    battery, the real BMS firmware reports over CAN, and in self-consumption
    mode the firmware closes the grid relay (state 0xCE), detects the phase it
    is on with a 2 kW charge test against the CT and covers the house."""
    from marsemu import VenusCE, House, connect_house, configure_self_consumption
    ctl = sorted(glob.glob(f"{FW}/HMG-50/V154/*.bin"))
    bms = sorted(glob.glob(f"{EXTRA}/**/HMG-50/BMS/216/*.bin", recursive=True)) if EXTRA else []
    if not ctl or not bms:
        pytest.skip("Venus C/E control V154 (MARSEMU_FIRMWARE) and BMS 216 (MARSEMU_ARCHIVE) needed")
    v = VenusCE(ctl[0], bms[0])
    house = House({"A": 100.0, "B": 500.0}, battery_phase="B")
    meter = connect_house(v, house)
    configure_self_consumption(v)
    m = v.control.machine
    phase = lambda: next(iter(meter.reports.values()), ("", ""))[0]
    try:
        v.run(60, until=lambda: v.stage.relay_closed())
        assert v.stage.relay_closed(), f"grid relay never closed: {v.status()}"
        v.run(5)
        assert 400 < v.stage.vbus < 470, "DC bus charged by the LLC"
        # the 1 kHz state machine's state (struct at 0x200002A4 in V151 and V154)
        assert m.read(0x200002A8, 1)[0] == 0xCE, "connected state"
        v.run(40, until=lambda: phase() == "B" and abs(sum(house.grid().values())) < 100)
        assert phase() == "B", f"phase reported in the CT polls: {meter.reports}"
        assert abs(sum(house.grid().values())) < 100, f"house not covered: {house.grid()}, battery {v.ac_power:.0f} W"
    finally:
        v.close()


@pytest.mark.firmware
def test_jupiter_with_pv_covers_the_load():
    """Jupiter: control firmware + inverter, MPPT and BMS models on CAN."""
    from marsemu.plant.jupiter import Jupiter
    from marsemu.plant.venus3_cli import CtMeter, House
    files = sorted(glob.glob(f"{FW}/HMN-1/V142/*.bin"))
    if not files:
        pytest.skip("Jupiter HMN-1 V142 not available")
    v = Jupiter(files[0], pv=[600, 0, 0, 0])
    house = House({"A": 400.0})
    v.sim.tickers.append(lambda now, dt: setattr(house, "battery_w", v.ac_power))
    v.control.fc41d.net.udp_handler = CtMeter(v.control, house, "112233445566")
    subs = []
    v.control.on("mqtt_subscribe", subs.append)
    try:
        v.run(80, until=lambda: bool(subs))
        assert subs
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=2,md=1")
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=16,meter=3,mac=112233445566")
        v.run(160, until=lambda: abs(house.grid()["A"]) < 20 and v.ac_power > 300)
        assert abs(house.grid()["A"]) < 20, f"grid not balanced: {house.grid()} battery {v.ac_power:.0f} W"
        assert v.pack.current_a > 0
    finally:
        v.close()


@pytest.mark.firmware
def test_jupiter_c_plus_reports_mppt_and_bms_battery_current():
    """Jupiter C Plus (JPLS-8H V143) with the BMS model: the control copies the
    battery current of the MPPT and the BMS into cd=14 unchanged."""
    from marsemu.plant.jupiter import Jupiter
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive with JPLS-8H")
    files = sorted(glob.glob(f"{EXTRA}/**/JPLS-8H/143/*.bin", recursive=True))
    if not files:
        pytest.skip("Jupiter C Plus JPLS-8H V143 not available")
    v = Jupiter(files[0], pv=[300, 300, 0, 0])
    subs, pubs = [], []
    v.control.on("mqtt_subscribe", subs.append)
    v.control.on("mqtt_publish", lambda t, p: pubs.append(p.decode("latin1") if isinstance(p, bytes) else p))
    try:
        v.run(80, until=lambda: bool(subs))
        assert subs
        v.run(20)
        v.control.mqtt_inject(subs[0], "cd=14")
        v.run(3)
        report = next(p for p in reversed(pubs) if p.startswith("inv:"))
        mppt = dict(kv.split("=", 1) for kv in report.split("mppt:")[1].split("bms:")[0].strip(",").split(","))
        bms = dict(kv.split("=", 1) for kv in report.split("bms:")[1].split(","))
        assert v.bms.answers > 0
        assert int(bms["b_cur"]) > 50 and abs(int(bms["b_vol"]) / 100 - v.pack.voltage) < 0.1
        assert int(mppt["b_cur"]) > 50 and abs(int(mppt["b_vol"]) / 10 - v.pack.voltage) < 0.2
        assert abs(int(mppt["b_cur"]) - int(bms["b_cur"])) <= 2                  # inverter idle
    finally:
        v.close()


@pytest.mark.firmware
def test_jupiter_c_plus_with_its_bms_firmware():
    """Jupiter C Plus: control V143 and the real BMS V37 on one CAN bus; the
    BMS answers the control's broadcast and its readings reach cd=14."""
    from marsemu.plant.jupiter import Jupiter
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive with JPLS-8H")
    ctl = sorted(glob.glob(f"{EXTRA}/**/JPLS-8H/143/*.bin", recursive=True))
    bms = sorted(glob.glob(f"{EXTRA}/**/JPLS-8H/37/*.bin", recursive=True))
    if not ctl or not bms:
        pytest.skip("Jupiter C Plus JPLS-8H V143 / V37 not available")
    v = Jupiter(ctl[0], bms[0], pv=[300, 300, 0, 0])
    subs, pubs, frames = [], [], []
    v.control.on("mqtt_subscribe", subs.append)
    v.control.on("mqtt_publish", lambda t, p: pubs.append(p.decode("latin1") if isinstance(p, bytes) else p))
    v.bus.monitors.append(lambda f: frames.append(f.ident) if f.sender is v.bms_can else None)
    try:
        v.run(80, until=lambda: bool(subs))
        assert subs
        v.run(20)
        v.control.mqtt_inject(subs[0], "cd=14")
        v.run(3)
        assert 0x1801AA01 in frames and 0x1821AA01 in frames
        report = next(p for p in reversed(pubs) if p.startswith("inv:"))
        mppt = dict(kv.split("=", 1) for kv in report.split("mppt:")[1].split("bms:")[0].strip(",").split(","))
        bms = dict(kv.split("=", 1) for kv in report.split("bms:")[1].split(","))
        assert bms["c_vol"] == "584" and bms["b_cap"] == "2560"                  # the real BMS, not the model
        assert abs(int(bms["b_vol"]) / 100 - v.pack.voltage) < 0.1
        assert abs(int(bms["b_cur"]) - int(mppt["b_cur"])) <= 2 and int(bms["b_cur"]) > 50
        assert v.status()["mos"] == {"charge": True, "discharge": True, "precharge": False}
        assert v.micro.bms_flags == 3 and v.micro.limits()[0] > 0               # limits from the real BMS
    finally:
        v.close()


HMA_LAYOUT = {"pv_v1": ("ADC2", 5), "pv_i1": ("ADC2", 12), "out_i0": ("ADC1", 1), "out_i1": ("ADC1", 2)}
HMJ_LAYOUT = {"pv_v1": ("ADC1", 2), "pv_i1": ("ADC1", 1), "out_i0": ("ADC2", 5), "out_i1": ("ADC2", 12)}
B2500_LAYOUTS = {
    # build: (channels that differ between builds, ADC DMA buffers, PV current mA/count)
    # HMJ moves PV2 and the output currents to the other ADC
    "HMA-1/V232": (HMA_LAYOUT, (0x20007E78, 0x20007E98), 8.25),
    "HMJ-2/V118": (HMJ_LAYOUT, (0x20007D7C, 0x20007D9C), 8.25),
    "HMJ-1/V112": (HMJ_LAYOUT, (0x20007D18, 0x20007D38), 8.25),
    # old layout: 8 mA/count up to V214 and from V241; V119 keeps side 2 at other offsets
    "HMA-1/V255": (HMA_LAYOUT, (0x20002C24, 0x20002C44), 8.0),
    "HMA-1/V119": (HMA_LAYOUT, (0x20002C74, 0x20002C94), 8.0),
}
#: builds with CT meter support (the others only run timers)
B2500_CT = ("HMA-1/V232", "HMJ-2/V118")


@pytest.mark.firmware
@pytest.mark.parametrize("build", sorted(B2500_LAYOUTS))
def test_b2500_adc_layout_found_in_the_image(build):
    from marsemu.analysis.b2500 import find_adc_slots
    files = sorted(glob.glob(f"{FW}/{build}/*.bin"))
    if not files:
        pytest.skip(f"B2500 {build} not available")
    _, buffers, ma = B2500_LAYOUTS[build]
    scales = {}
    slots = find_adc_slots(open(files[-1], "rb").read(), 0x08000000, buffers, scales)
    assert sorted(slots.values()) == sorted(["pv_v0", "pv_v1", "pv_i0", "pv_i1", "out_v0", "out_v1", "out_i0", "out_i1"])
    assert scales == {"pv_i0": ma, "pv_i1": ma}


@pytest.mark.firmware
@pytest.mark.parametrize("build", B2500_CT)
def test_b2500_steers_against_the_ct_and_charges_from_pv(build):
    """B2500: firmware + SH367309, PV converters, outputs and microinverters."""
    from marsemu.plant.b2500 import B2500
    from marsemu.plant.venus3_cli import CtMeter, House
    files = sorted(glob.glob(f"{FW}/{build}/*.bin"))
    if not files:
        pytest.skip(f"B2500 {build} not available")
    v = B2500(files[0], pv=[300, 300])
    house = House({"A": 200.0})
    v.sim.tickers.append(lambda now, dt: setattr(house, "battery_w", v.ac_power))
    v.control.fc41d.net.udp_handler = CtMeter(v.control, house, "112233445566")
    subs = []
    v.control.on("mqtt_subscribe", subs.append)
    try:
        v.run(60, until=lambda: bool(subs))
        assert subs
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=27,meter=3,mac=112233445566")
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=18,md=1")
        v.run(150, until=lambda: abs(house.grid()["A"]) < 10 and v.ac_power > 150 and sum(v.plant.status()["pv_w"]) > 580)
        st = v.status()
        assert abs(house.grid()["A"]) < 10, f"grid not balanced: {house.grid()} {st}"
        assert sum(st["pv_w"]) > 580, "MPPT at the maximum power point"
        assert v.pack.current_a > 0, "PV surplus should charge the battery"
        assert all(30 < x < 40 for x in st["out_v"]), "microinverters near the synthetic panel's MPP (35.2 V)"
        for sig, pin in B2500_LAYOUTS[build][0].items():
            assert v.plant.layout[sig] == pin, f"{sig} channel"
    finally:
        v.close()


@pytest.mark.firmware
@pytest.mark.parametrize("build", ["HMA-1/V119", "HMA-1/V255", "HMA-1/V232", "HMJ-2/V118"])
def test_b2500_v2_timers_soc_and_pv(build):
    """B2500 v2, old and new layout: the firmware writes its own defaults over
    the seeded record (timer 1: 800 W all day), takes the pack's SOC from it and
    delivers the timer's power; PV is measured at the build's own scale."""
    from marsemu.plant.b2500 import B2500
    from marsemu.plant.battery import LfpPack
    files = sorted(glob.glob(f"{FW}/{build}/*.bin"))
    if not files:
        pytest.skip(f"B2500 {build} not available")
    v = B2500(files[-1], pv=[0, 0], pack=LfpPack(cells_n=14, capacity_ah=50.0, soc=0.37))
    subs, last = [], {}
    v.control.on("mqtt_subscribe", subs.append)
    v.control.on("mqtt_publish", lambda t, p: last.update(
        dict(kv.split("=", 1) for kv in p.decode("latin1").split(",") if "=" in kv)) if b"pe=" in p else None)
    try:
        v.run(90)
        assert subs and v.control.machine.reset_count == 1
        v.control.mqtt_inject(subs[0], "cd=1")
        v.run(3)
        assert last.get("pe") == "37", "SOC from the seeded record"
        assert last.get("do") == "100" and last.get("h1") == "800", "the firmware's own defaults"
        st = v.status()
        assert all(abs(w - 400) < 5 for w in st["out_w"]), "800 W timer: both inverters at their limit"
        assert all(abs(int(last[g]) - w) < 15 for g, w in zip(("g1", "g2"), st["out_w"])), "firmware measures the outputs"
        if build != "HMA-1/V119":                       # V119 only polls the extension packs
            bus = st["bus"]                             # status broadcast on the extension-pack bus (UART5)
            assert bus["soc"] == 37 and bus["direction"] == "discharging"
            assert abs(bus["pack_mv"] / 1000 - st["pack_v"]) < 0.15 and len(bus["cells_mv"]) == 14
            assert abs(-bus["current_ma"] / 1000 - (-st["i_a"])) < 1.0
        # PV: measured at this build's scale; the old layout charges first (factory setting)
        v.set_pv([300, 200])
        v.run(60)
        v.control.mqtt_inject(subs[0], "cd=1")
        v.run(3)
        st = v.status()
        assert abs(int(last["w1"]) - st["pv_w"][0]) < 6 and abs(int(last["w2"]) - st["pv_w"][1]) < 6
        if build in ("HMA-1/V119", "HMA-1/V255"):
            assert v.pack.current_a > 5
    finally:
        v.close()


@pytest.mark.firmware
@pytest.mark.parametrize("build", ["HMA-1/V232", "HMB-1/V144"])
def test_b2500_extension_pack_on_the_bus(build):
    """An emulated extension pack answers the main unit on UART5: the firmware
    reports it connected, averages the SOC and adds its energy, and shows its
    voltage and current in cd=26 (V2)."""
    from marsemu.plant.b2500 import B2500
    from marsemu.plant.battery import LfpPack
    files = sorted(glob.glob(f"{FW}/{build}/*.bin"))
    if not files:
        pytest.skip(f"B2500 {build} not available")
    mk = lambda soc: LfpPack(cells_n=14, capacity_ah=50.0, soc=soc)
    v = B2500(files[-1], pack=mk(0.5), extension_packs=[mk(0.7)])
    subs, reports = [], []
    v.control.on("mqtt_subscribe", subs.append)
    v.control.on("mqtt_publish", lambda t, p: reports.append(
        dict(kv.split("=", 1) for kv in p.decode("latin1").split(",") if "=" in kv)))
    try:
        v.run(60)
        for cmd in ("cd=1", "cd=26"):
            v.control.mqtt_inject(subs[0], cmd)
            v.run(3)
        one = next(r for r in reports if "pe" in r)
        assert one["b1"] == "1" and one["pe"] == "60" and one["kn"] == str(224 * (5 + 7))
        if build == "HMA-1/V232":
            ext = next(r for r in reports if "sv" in r and "bb" in r)
            pack = v.plant.ext_packs[0]
            assert abs(int(ext["sv"]) / 1000 - pack.voltage) < 0.1
            assert abs(int(ext["sc"]) / 1000 - abs(pack.current_a)) < 1.0
        assert v.plant.ext[0].answered > 10
    finally:
        v.close()


@pytest.mark.firmware
def test_b2500_v1_switched_outputs_and_pass_through():
    """HMB V144 (B2500 v1): settings record seeded with the factory defaults and
    the pack's SOC; outputs are switches, PV2 can pass straight through."""
    from marsemu.plant.b2500 import B2500
    from marsemu.plant.battery import LfpPack
    files = sorted(glob.glob(f"{FW}/HMB-1/V144/*.bin"))
    if not files:
        pytest.skip("B2500 HMB-1 V144 not available")
    v = B2500(files[0], pv=[0, 0], pack=LfpPack(cells_n=14, capacity_ah=50.0, soc=0.6))
    subs, last = [], {}
    v.control.on("mqtt_subscribe", subs.append)
    v.control.on("mqtt_publish", lambda t, p: last.update(
        dict(kv.split("=", 1) for kv in p.decode("latin1").split(",") if "=" in kv)) if b"pe=" in p else None)
    try:
        v.run(60, until=lambda: bool(subs))
        assert subs
        v.run(2)
        v.control.mqtt_inject(subs[0], "cd=4,md=3")          # both outputs on
        v.run(80)
        v.control.mqtt_inject(subs[0], "cd=1")
        v.run(3)
        assert last.get("pe") == "60", "SOC from the seeded record"
        st = v.status()
        assert all(abs(w - 400) < 5 for w in st["out_w"]), "battery switched onto both outputs, inverters at their limit"
        assert abs(int(last["g1"]) - st["out_w"][0]) < 15, "firmware measures output 1"
        assert v.pack.current_a < -15
        # PV above the output threshold: battery off, PV2 straight through to output 2
        v.set_pv([0, 300])
        v.run(80)
        v.control.mqtt_inject(subs[0], "cd=1")
        v.run(3)
        st = v.status()
        assert last.get("p2") == "2", "PV2 in pass-through (factory default charging mode)"
        assert st["out_w"][0] == 0 and st["out_w"][1] > 290
    finally:
        v.close()


@pytest.mark.firmware
@pytest.mark.parametrize("model,port", [("VNSE3-0", "USART2"), ("VNSD-0", "USART2"), ("VNSA-0", "USART1")])
def test_venus_modbus_tcp_reaches_the_rs485_slave(model, port):
    """The control firmware's Modbus RTU slave, served as Modbus TCP: read the
    model string and the slave address, switch RS485 control on and read it back."""
    import socket
    import struct
    import threading
    from marsemu.modbus import ModbusGateway
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    files = sorted(glob.glob(f"{EXTRA}/**/{model}/Control/151/*.bin", recursive=True))
    if not files:
        pytest.skip(f"{model} control 151 not in the archive")
    emu = Emulator(files[0])
    gw = ModbusGateway(emu, port=0, host="127.0.0.1")
    assert gw.uart.name == port
    answers = []

    def client():
        with socket.create_connection(("127.0.0.1", gw.port), timeout=60) as s:
            for tid, pdu in ((1, struct.pack(">BHH", 3, 31000, 10)), (2, struct.pack(">BHH", 3, 41100, 1)),
                             (3, struct.pack(">BHH", 6, 42000, 0x55AA)), (4, struct.pack(">BHH", 3, 42000, 1))):
                s.sendall(struct.pack(">HHHB", tid, 0, len(pdu) + 1, 1) + pdu)
                _, _, length, _ = struct.unpack(">HHHB", s.recv(7))
                answers.append(s.recv(length - 1))
    t = threading.Thread(target=client, daemon=True)
    emu.run(seconds=15)                     # boot; the slave answers once its RS485 task runs
    t.start()
    emu.run(seconds=30, until=lambda: len(answers) == 4)
    t.join(5)
    gw.close()
    assert len(answers) == 4, answers
    assert answers[0][:2] == bytes([3, 20]) and answers[0][2:].rstrip(b"\0") == model.encode()
    assert answers[1] == bytes([3, 2, 0, 1]), "slave address 1"
    assert answers[2] == struct.pack(">BHH", 6, 42000, 0x55AA)
    assert answers[3] == bytes([3, 2, 0x55, 0xAA])


def _venus_whole(kind, soc):
    """A whole Venus E 3.0 or D (151) with distinct images found in the archive."""
    from marsemu.plant.battery import LfpPack
    from marsemu.plant.venus3 import VenusE3
    from marsemu.plant.venusd import VenusD
    if kind == "e3":
        ctl, bms = _venus3_images()
        return VenusE3(ctl, bms, pack=LfpPack(soc=soc))
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    ctl = sorted(glob.glob(f"{EXTRA}/**/VNSD-0/Control/151/*.bin", recursive=True))
    bms = sorted(glob.glob(f"{EXTRA}/**/VNSD-0/BMS/118/*.bin", recursive=True))
    if not ctl or not bms:
        pytest.skip("Venus D control 151 / BMS 118 not in the archive")
    return VenusD(ctl[0], bms[0], pack=LfpPack(soc=soc, capacity_ah=100.0))


def _ct_reply(f, grid, polls, extra=None, ct_mac="112233445566"):
    """A CT002 relay reply to the poll fields ``f`` for per-phase ``grid``."""
    from marsemu.plant.venus3_cli import ct_payload
    reply = ["HME-4", ct_mac, f[0], f[1]] + [str(round(grid[p])) for p in "ABC"]
    reply += [str(round(sum(grid.values()))), "1", "1", "1", "0", "-50", str(polls % 256)] + ["0"] * 10
    for i, v in (extra or {}).items():
        reply[i] = v
    return ct_payload(reply)


@pytest.mark.firmware
@pytest.mark.parametrize("kind", ["e3", "d"])
def test_venus_stops_charging_while_the_ct_reports_another_battery_discharging(kind):
    """Cross-talk: a charging Venus (151) drops to 0 W as soon as the CT reply
    carries a non-zero *_dchrg_power (here A_dchrg_power, field 20, while it
    sits on phase C), and resumes when the field is 0 again.  A CT002
    emulator that reports a full battery's PV passthrough there makes its
    other batteries switch on and off (AstraMeter issue #763)."""
    from marsemu import House, connect_house, configure_self_consumption
    from marsemu.plant.venus3_cli import ct_fields
    v = _venus_whole(kind, 0.5)
    house = House({"A": 0.0, "B": 0.0, "C": -800.0})
    window = [None]
    polls = [0]

    def meter(data, addr):
        polls[0] += 1
        f = ct_fields(data)
        on = window[0] is not None and window[0][0] <= v.time < window[0][1]
        return _ct_reply(f, house.grid(), polls[0], {20: "300"} if on else None)
    connect_house(v, house, meter=meter, phase="C")
    configure_self_consumption(v)
    try:
        v.run(150, until=lambda: v.ac_power < -700 and abs(house.grid()["C"]) < 50)
        assert v.ac_power < -700, f"never charged from the export: {v.status()}"
        t = v.time
        window[0] = (t + 1, t + 21)
        v.run(12)
        assert abs(v.ac_power) < 20, f"still charging with A_dchrg_power set: {v.ac_power:.0f} W"
        v.run(25, until=lambda: v.ac_power < -700)
        assert v.ac_power < -700, f"did not resume after the field cleared: {v.ac_power:.0f} W"
    finally:
        v.close()


@pytest.mark.firmware
@pytest.mark.parametrize("soc,detects", [(0.66, False), (0.85, True)])
def test_venus3_skips_phase_detection_below_its_backup_soc(soc, detects):
    """Venus E 3.0 (151) with the depth of discharge at 30 (cut-off 70 %):
    below the cut-off it never runs its phase detection, keeps polling the CT
    as phase 0 (inspection) and does not charge even from an export; above
    it, it finds its phase and charges (AstraMeter issue #703)."""
    from marsemu import House, connect_house
    v = _venus_whole("e3", soc)
    house = House({"A": 0.0, "B": -800.0, "C": 0.0})
    meter = connect_house(v, house, phase="B")
    subs = []
    v.control.on("mqtt_subscribe", subs.append)
    try:
        v.run(60, until=lambda: bool(subs))
        assert subs
        for cmd in ("cd=56,dod=30", "cd=2,md=0", "cd=18,meter=3,mac=112233445566"):
            v.run(2)
            v.control.mqtt_inject(subs[0], cmd)
        v.run(120, until=lambda: v.ac_power < -700)
        phase = next(iter(meter.reports.values()), ("", ""))[0]
        if detects:
            assert phase == "B" and v.ac_power < -700, f"phase {phase!r}, {v.status()}"
        else:
            assert phase == "0", f"left inspection: phase {phase!r}"
            assert abs(v.ac_power) < 20, f"charged below the cut-off: {v.status()}"
    finally:
        v.close()


@pytest.mark.firmware
def test_two_b2500_detecting_at_once_both_give_up_until_rearmed_one_by_one():
    """Two B2500 HMJ-2 (116) on phases A and C behind one CT, both given the
    CT at the same moment (as when both are switched to self-adaptation at
    once): their phase probes overlap, each sees two phases move, and both
    are stuck in phase 0 in their polls, not steering.  Selecting the meter
    again, one battery at a time, lets each find its phase (AstraMeter issue
    #559).  Runs both devices on one timeline (run_together)."""
    from marsemu import B2500, House, connect_house, run_together
    from marsemu.fc41d.module import Fc41dConfig
    from marsemu.plant.venus3_cli import ct_fields
    files = sorted(glob.glob(f"{FW}/HMJ-2/V116/*.bin"))
    if not files:
        pytest.skip("B2500 HMJ-2 V116 not available")
    ct = "02b25060e884"

    def ident(n):
        return {"uid": bytes([0x40 + n] * 12),
                "fc41d": Fc41dConfig(wifi_mac=f"c8:47:8c:00:{n:02x}:01", ip=f"192.168.1.{50 + n}")}
    left = B2500(files[0], inverter_w=[300, 300], control_kwargs=ident(1))
    right = B2500(files[0], inverter_w=[300, 300], control_kwargs=ident(2))
    house = House({"A": 300.0, "B": 0.0, "C": 300.0})
    phases, polls = {}, [0]

    def meter(data, addr):                     # one CT for both, relaying the grid
        f = ct_fields(data)
        polls[0] += 1
        phases[addr[0]] = f[4]
        return _ct_reply(f, house.grid(), polls[0], ct_mac=ct)
    topics = {}
    for name, dev, phase in (("L", left, "A"), ("R", right, "C")):
        connect_house(dev, house, meter=meter, phase=phase, name=name)
        dev.control.on("mqtt_subscribe", lambda t, name=name: topics.setdefault(name, t))

    def send(name, payload):
        (left if name == "L" else right).control.mqtt_inject(topics[name], payload)
    devices = [left, right]

    def at(t):                                  # run until emulated time t
        run_together(devices, t - left.time)
    try:
        at(30)
        assert len(topics) == 2
        send("L", f"cd=27,meter=3,mac={ct}")    # both at once
        send("R", f"cd=27,meter=3,mac={ct}")
        at(32)
        send("L", "cd=18,md=1")
        send("R", "cd=18,md=1")
        at(150)
        # stuck: still phase 0 two minutes on (each either holds its probe
        # output or has switched off, depending on how the probes overlapped)
        assert phases == {"192.168.1.51": "0", "192.168.1.52": "0"}, phases
        for name, t in (("L", 150), ("R", 300)):  # one at a time
            send(name, "cd=27,meter=0")
            at(t + 3)
            send(name, f"cd=27,meter=3,mac={ct}")
            at(t + 5)
            send(name, "cd=18,md=1")
            at(t + 150)
        assert phases == {"192.168.1.51": "A", "192.168.1.52": "C"}, phases
        assert abs(sum(house.grid().values())) < 60, f"house not covered: {house.grid()}"
    finally:
        for d in devices:
            d.close()


@pytest.mark.firmware
def test_b2500_holds_its_output_while_a_detecting_battery_reports_discharge():
    """B2500 HMJ-2 (116): while the CT reply's x_dchrg_power (field 19, the
    batteries still detecting their phase) is non-zero, it does not raise its
    output however much import the reply shows; the per-phase *_dchrg_power
    fields have no such effect.  A neighbour stuck mid-probe at full output
    therefore freezes it until that probe times out."""
    from marsemu import B2500, House, connect_house, configure_self_consumption
    from marsemu.plant.venus3_cli import ct_fields
    files = sorted(glob.glob(f"{FW}/HMJ-2/V116/*.bin"))
    if not files:
        pytest.skip("B2500 HMJ-2 V116 not available")
    b = B2500(files[0])
    house = House({"A": 400.0})
    override = {}
    polls = [0]

    def meter(data, addr):
        polls[0] += 1
        f = ct_fields(data)
        g = dict(house.grid())
        if override.get("grid") is not None:
            g["A"] = override["grid"]
        return _ct_reply(f, g, polls[0], override.get("fields"))
    connect_house(b, house, meter=meter)
    configure_self_consumption(b)
    try:
        b.run(120)                  # well past its phase detection and settling
        assert abs(house.grid()["A"]) < 15, f"never covered the load: {b.status()}"
        steady = b.ac_power
        # a reply that keeps asking for 100 W more, with a detecting battery discharging
        override.update(grid=100.0, fields={19: "800"})
        b.run(40)
        assert abs(b.ac_power - steady) < 20, f"raised its output to {b.ac_power:.0f} W"
        override["fields"] = {20: "1000", 21: "800"}    # per-phase discharge: no hold
        b.run(40)
        assert b.ac_power > steady + 200, f"held at {b.ac_power:.0f} W without x_dchrg"
    finally:
        b.close()


@pytest.mark.firmware
def test_ui_session_b2500_steers_and_logs_every_bus():
    """The dashboard's session around a B2500: the firmware steers the grid to
    about 0 W, and every bus the dashboard draws shows up in its log."""
    import time
    from marsemu.ui.session import B2500Session
    files = sorted(glob.glob(f"{FW}/HMJ-2/V116/*.bin"))
    if not files:
        pytest.skip("B2500 HMJ-2 V116 not available")
    s = B2500Session(files[0], pv=(300, 280), load=420, speed=0)
    s.start()
    try:
        deadline = time.time() + 600
        while s.read()["snapshot"]["t"] < 80 and time.time() < deadline:
            time.sleep(0.5)
        r = s.read()
    finally:
        s.stop()
    snap = r["snapshot"]
    assert snap["error"] is None
    assert snap["version"] == "116" and snap["device_type"] == "HMJ-2"
    assert abs(snap["grid"]) < 60, snap["grid"]
    assert {"USART1", "RF", "MQTT", "I2C2", "UART5"} <= {e["ch"] for e in r["events"]}
    assert snap["mcu"]["regs"]["TIM8_CCR3"] and snap["mcu"]["irqs"]
    assert len(r["history"]) > 100


UI_SESSIONS = [
    ("venusd", "VNSD-0/Control/151", "VNSD-0/BMS/118"),
    ("venusa", "VNSA-0/Control/151", "VNSA-0/BMS/115"),
    ("jupiter-cplus", "JPLS-8H/143", "JPLS-8H/37"),
    ("jupiter-bms-model", "JPLS-8H/143", None),
]


@pytest.mark.firmware
@pytest.mark.parametrize("name,control,bms", UI_SESSIONS, ids=[u[0] for u in UI_SESSIONS])
def test_ui_session_steers(name, control, bms):
    """The dashboard's sessions for the Venus D and A and the Jupiter: the
    firmware steers the grid to about 0 W and the CAN bus is decoded."""
    import time
    from marsemu.ui import session as S
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    ctl = sorted(glob.glob(f"{EXTRA}/**/{control}/*.bin", recursive=True))
    bmi = sorted(glob.glob(f"{EXTRA}/**/{bms}/*.bin", recursive=True)) if bms else [None]
    if not ctl or not bmi:
        pytest.skip(f"{control} / {bms} not in the archive")
    cls = {"venusd": S.VenusDSession, "venusa": S.VenusASession}.get(name, S.JupiterSession)
    s = cls(ctl[0], bms=bmi[0], pv=(300, 300, 0, 0), load=500, speed=0)
    s.start()
    try:
        deadline = time.time() + 900
        while s.read()["snapshot"]["t"] < 90 and time.time() < deadline:
            time.sleep(0.5)
        r = s.read()
    finally:
        s.stop()
    snap = r["snapshot"]
    assert snap["error"] is None and snap["version"]
    assert abs(snap["grid"]) < 60, snap["grid"]
    assert snap["mppt"]["dc"] > 0
    dirs = snap["can"]["by_dir"]
    assert dirs.get("control→inverter") and dirs.get("BMS→control") and dirs.get("control→MPPT")
    assert snap["bms"]["real"] == (bms is not None)


@pytest.mark.firmware
def test_ui_ct_session_answers_batteries():
    """The CT meter's own firmware answers the virtual batteries' polls,
    counting them per phase, with the grid its clamps measure through the
    metering chip: 600 W of house over three phases, 300 W from each battery."""
    import time
    from marsemu.ui.session import CtSession
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    files = sorted(glob.glob(f"{EXTRA}/**/HME-4/124/*.bin", recursive=True))
    if not files:
        pytest.skip("CT002 HME-4 V124 not in the archive")
    s = CtSession(files[0], batteries=2, battery_w=300, load=600, speed=0)
    s.start()
    try:
        deadline = time.time() + 300
        while s.read()["snapshot"]["t"] < 30 and time.time() < deadline:
            time.sleep(0.5)
        snap = s.read()["snapshot"]
    finally:
        s.stop()
    assert snap["meter"]["replies"] >= 5
    assert [b["replies"] > 0 for b in snap["batteries"]] == [True, True]
    assert snap["meter"]["last"][8:10] == ["1", "1"]          # one battery on phase A, one on B
    assert snap["meter"]["clamps"] == {"A": -100.0, "B": -100.0, "C": 200.0}
    assert [int(v) for v in snap["meter"]["last"][4:8]] == [-100, -100, 200, 0]


@pytest.mark.firmware
def test_ui_ct003_session_reads_the_p1_meter():
    """The CT003 is a P1 reader: once its P1 line idles high it reads the
    utility meter's telegrams on USART3 and answers the batteries with them."""
    import time
    from marsemu.ui.session import CtSession
    if not EXTRA:
        pytest.skip("set MARSEMU_ARCHIVE to a firmware archive")
    files = sorted(glob.glob(f"{EXTRA}/**/HME-3/122/*.bin", recursive=True))
    if not files:
        pytest.skip("CT003 HME-3 V122 not in the archive")
    s = CtSession(files[0], batteries=2, battery_w=300, load=600, speed=0)
    s.start()
    try:
        deadline = time.time() + 300
        while s.read()["snapshot"]["t"] < 30 and time.time() < deadline:
            time.sleep(0.5)
        snap = s.read()["snapshot"]
    finally:
        s.stop()
    assert snap["meter"]["type"] == "HME-3" and snap["meter"]["source"] == "p1"
    assert snap["meter"]["last"][0] == "HME-3" and len(snap["meter"]["last"]) == 28
    assert [int(v) for v in snap["meter"]["last"][4:8]] == [-100, -100, 200, 0]
