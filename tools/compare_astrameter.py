"""Run real Marstek firmware in the emulator and AstraMeter's battery simulator
through the same house-load profile, and compare how they steer.

Both sides see a CT that reports the raw grid power (a CT002 in relay mode, not
AstraMeter's active control). Devices:

* ``b2500``: a B2500 (DC-coupled, steers its outputs through microinverters),
  no PV, pack at 50 %
* ``venus3``, ``venusd``, ``venusa``: a Venus E 3.0 / D / A (control and BMS
  firmware), pack at 50 %, phase detection on both sides; the profile includes
  an export, which it charges from

::

    # the firmware, in the emulator (about real time)
    python tools/compare_astrameter.py emulator --device b2500 emu.json FIRMWARE.bin [--ideal-inverter]
    python tools/compare_astrameter.py emulator --device venus3 emu.json CONTROL.bin BMS.bin
    # AstraMeter's model, with AstraMeter's own Python environment
    /path/to/AstraMeter/.venv/bin/python tools/compare_astrameter.py astrameter --device venus3 astra.json --poll 1.0
    # side by side
    python tools/compare_astrameter.py compare --device venus3 emu.json astra.json

The B2500 side includes a microinverter model (start-up delay, MPP sweep,
dropout at low power), which shapes the transients; ``--ideal-inverter`` makes
it start in 2 s, sweep fast, never drop out and convert without loss, to tell
firmware behaviour from plant assumptions.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

#: (start s, house load W, + = import); the last entry ends the run
PROFILES = {
    "b2500": [(0, 300), (90, 600), (180, 200), (270, 50), (360, 0), (450, 150), (540, 1000), (630, None)],
    "venus3": [(0, 300), (120, 800), (210, 150), (300, -500), (390, 30), (480, 0), (570, 3000), (660, 400),
               (780, None)],
}
PROFILES["venusd"] = PROFILES["venusa"] = PROFILES["venus3"]
METER_TYPE = {"b2500": "HMJ-2", "venus3": "VNSE3-0", "venusd": "VNSD-0", "venusa": "VNSA-0"}
#: pack energy (Wh) for AstraMeter's SOC bookkeeping
CAPACITY_WH = {"venus3": 5120.0, "venusd": 2560.0, "venusa": 2080.0}


def load_at(profile, t: float) -> float:
    w = 0
    for s, v in profile:
        if v is None or t < s:
            break
        w = v
    return w


def run_emulator(args) -> None:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from marsemu.plant.battery import LfpPack
    from marsemu.plant.venus3_cli import CtMeter, House

    profile = PROFILES[args.device]
    duration = profile[-1][0]
    if args.device != "b2500":
        from marsemu.plant.venus3 import VenusE3
        from marsemu.plant.venusa import VenusA
        from marsemu.plant.venusd import VenusD
        cls = {"venus3": VenusE3, "venusd": VenusD, "venusa": VenusA}[args.device]
        b = cls(args.firmware[0], args.firmware[1], pack=LfpPack(soc=0.5, capacity_ah=cls.CAPACITY_AH,
                                                                  cells_n=cls.CELLS))
        commands = ["cd=2,md=0", "cd=18,meter=3,mac=112233445566"]
    else:
        from marsemu.plant.b2500 import B2500
        b = B2500(args.firmware[0], pv=[0, 0], inverter_w=[args.inverter_w] * 2)
        # the meter command resets the discharge mode: adaptive (md=1) after it
        commands = ["cd=27,meter=3,mac=112233445566", "cd=18,md=1"]
    if args.ideal_inverter and args.device == "b2500":
        for inv in b.plant.inverters:
            inv.start_s, inv.scan_step, inv.period, inv.drop_w, inv.efficiency = 2.0, 2.0, 0.25, 0.0, 1.0
        for o in b.plant.out:
            o.efficiency = 1.0
    e = b.control
    house = House({"A": 0.0}, "A")
    meter = CtMeter(e, house, "112233445566")
    polls = []

    def ct(sock, data, dest):
        polls.append(b.time)
        meter(sock, data, dest)
    e.fc41d.net.udp_handler = ct
    topics, t0 = [], [None]

    def setup():
        t = topics[0]
        for k, c in enumerate(commands):
            b.sim.after(2.0 * k + 0.01, lambda c=c: e.mqtt_inject(t, c))
        b.sim.after(2.0 * len(commands) + 1.0, lambda: t0.__setitem__(0, b.time))
    e.on("mqtt_subscribe", topics.append)
    e.on("mqtt_subscribe", lambda t: b.sim.after(2.0, setup) if len(topics) == 1 else None)

    def track(now, dt):
        house.battery_w = b.ac_power
        if t0[0] is not None:
            house.loads["A"] = load_at(profile, now - t0[0])
    b.sim.tickers.append(track)
    b.run(120, until=lambda: t0[0] is not None)
    if t0[0] is None:
        sys.exit("the battery never subscribed to MQTT")
    rows = []
    b.sim.every(0.5, lambda now: rows.append({"t": round(now - t0[0], 2), "load": house.loads["A"],
                                             "ac_w": round(b.ac_power, 1), "grid": round(house.grid()["A"], 1)}))
    b.run(duration + 1)
    p = [x - t0[0] for x in polls if x >= t0[0]]
    st = b.status()
    json.dump({"side": "emulator" + ("-ideal" if args.ideal_inverter else ""), "device": args.device, "rows": rows,
               "poll": round((p[-1] - p[0]) / (len(p) - 1), 2) if len(p) > 1 else None,
               "limits": st.get("limits"), "bms_soc": st.get("bms_soc")}, open(args.out, "w"))
    b.close()


def run_astrameter(args) -> None:
    from astrameter.simulator.battery import BatterySimulator

    profile = PROFILES[args.device]
    duration = profile[-1][0]
    venus = args.device != "b2500"
    kw = dict(max_discharge_power=args.max_discharge, max_charge_power=args.max_charge)
    if args.soc is not None:
        kw["initial_soc"] = args.soc / 100.0
    if args.ramp is not None:
        kw["ramp_rate"] = args.ramp
    if args.startup_delay is not None:
        kw["startup_delay"] = args.startup_delay
    if venus:
        kw.update(phase_detection=True, capacity_wh=CAPACITY_WH[args.device])
    sim = BatterySimulator(mac="aabbccddeeff", phase="A", ct_mac="112233445566",
                           meter_dev_type=METER_TYPE[args.device], poll_interval=args.poll, **kw)
    rows, t, info_idx = [], 0.0, 0
    while t <= duration:
        sim._advance(args.poll)          # plant first, then the poll, like BatterySimulator.step()
        t += args.poll
        grid = load_at(profile, t) - sim.current_power
        if venus:
            # a CT002 reply in relay mode, as the emulator's meter sends it
            ph = sim.reported_phase
            counts = [1 if ph == p else 0 for p in "ABCD"]
            fields = ["HME-4", "112233445566", METER_TYPE[args.device], "AABBCCDDEEFF", str(round(grid)), "0", "0",
                      str(round(grid))] + [str(c) for c in counts] + ["-55", str(info_idx)] + ["0"] * 10
            info_idx = (info_idx + 1) % 256
            sim._receive_firmware(fields)
        else:
            sim._handle_ct_response(["HME-4", "112233445566", "", "", str(round(grid)), "0", "0"],
                                    grid_reading=round(grid))
        rows.append({"t": round(t, 3), "load": load_at(profile, t), "ac_w": round(sim.current_power, 1),
                     "grid": round(grid, 1)})
    side = "astrameter" + (f"-ramp{args.ramp:g}" if args.ramp else "")
    json.dump({"side": side, "device": args.device, "poll": args.poll, "rows": rows}, open(args.out, "w"))


def segment(rows, start, end, band=20.0, hold=10.0) -> dict:
    seg = [r for r in rows if start <= r["t"] < end]
    settle = None
    for i, r in enumerate(seg):
        later = [x for x in seg[i:] if x["t"] <= r["t"] + hold]
        if seg[-1]["t"] >= r["t"] + hold and all(abs(x["grid"]) <= band for x in later):
            settle = round(r["t"] - start, 1)
            break
    tail = [r for r in seg if r["t"] >= end - 30] or seg
    return {"settle_s": settle, "out": round(sum(r["ac_w"] for r in tail) / len(tail)),
            "grid": round(sum(r["grid"] for r in tail) / len(tail)),
            "mean_abs_grid": round(sum(abs(r["grid"]) for r in seg) / len(seg)),
            "peak": round(max(r["ac_w"] for r in seg))}


def compare(args) -> None:
    data = [json.load(open(p)) for p in args.files]
    profile = PROFILES[data[0].get("device", "b2500")]
    print("load W  from-to s | " + " | ".join(f"{d['side']} (poll {d.get('poll')} s)".center(38) for d in data))
    print("                  | " + " | ".join(f"{'settle':>7} {'out':>5} {'grid':>5} {'|g|':>5} {'peak':>5}      " for _ in data))
    for (s, load), (e, _) in zip(profile, profile[1:]):
        cells = []
        for d in data:
            st = segment(d["rows"], s, e)
            cells.append(f"{str(st['settle_s']):>7} {st['out']:>5} {st['grid']:>5} {st['mean_abs_grid']:>5} {st['peak']:>5}      ")
        print(f"{load:>6} {s:>4}-{e:<4} | " + " | ".join(cells))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    em = sub.add_parser("emulator")
    em.add_argument("--device", choices=sorted(PROFILES), default="b2500")
    em.add_argument("out")
    em.add_argument("firmware", nargs="+", help="B2500: one image; Venus: control and BMS")
    em.add_argument("--inverter-w", type=float, default=400.0)
    em.add_argument("--ideal-inverter", action="store_true")
    am = sub.add_parser("astrameter")
    am.add_argument("--device", choices=sorted(PROFILES), default="b2500")
    am.add_argument("out")
    am.add_argument("--poll", type=float, default=1.0)
    am.add_argument("--max-discharge", type=int, default=800)
    am.add_argument("--max-charge", type=int, default=800)
    am.add_argument("--soc", type=float, help="initial SOC %% (the emulated Venus BMS reports 53.5 for a 50 %% pack)")
    am.add_argument("--ramp", type=float, help="plant ramp, W/s (AstraMeter default 200; the emulated Venus inverter: 1000)")
    am.add_argument("--startup-delay", type=float, help="s (AstraMeter default 2; the emulated Venus inverter: 3)")
    cp = sub.add_parser("compare")
    cp.add_argument("files", nargs="+")
    args = ap.parse_args()
    {"emulator": run_emulator, "astrameter": run_astrameter, "compare": compare}[args.cmd](args)


if __name__ == "__main__":
    main()
