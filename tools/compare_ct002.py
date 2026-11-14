"""Answer the same battery polls with the real CT002 firmware and with
AstraMeter's CT002, on the same grid, and compare the replies field by field.

The firmware runs in the emulator with its metering chip modelled
(``marsemu/periph/bl_spi.py``): the grid power per phase goes into the chip's
power registers, and the firmware reads them the way it reads a real chip.
AstraMeter's ``CT002`` runs in the same process with ``active_control`` off
(a CT that reports, not one that steers), on the emulator's clock, and reads
the same grid from its ``before_send`` hook.

Both get every poll at the same emulated moment.  A scenario is a grid
profile plus batteries that poll on a fixed period, report a power, change
phase, join and leave::

    python tools/compare_ct002.py CT002.bin --astrameter /path/to/AstraMeter/src [--json out.json]

The report lists, per reply field, how many replies agreed, and the first
disagreements with their context.  Replies within ``--settle`` seconds of a
grid step are counted separately: the firmware reads the chip every 200 ms
and may filter, so a step reaches the two answers at slightly different polls.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

#: reply fields, numbered as in AstraMeter's docs/ct002-ct003-protocol.md (0-based here)
FIELDS = ["meter_dev_type", "meter_mac", "storage_type", "storage_mac", "A_power", "B_power", "C_power",
          "total_power", "A_nb", "B_nb", "C_nb", "ABC_nb", "rssi", "info_idx", "x_chrg", "A_chrg", "B_chrg",
          "C_chrg", "ABC_chrg", "x_dchrg", "A_dchrg", "B_dchrg", "C_dchrg", "ABC_dchrg",
          "low_in", "normal_in", "low_out", "normal_out"]
#: fields that differ by design: the meter's own id and radio, and a counter
IGNORED = {"meter_mac", "rssi", "info_idx"}

#: grid power per phase (W, + import) from a time on
GRID = [(0, (300, -150, 50)), (40, (120, -150, 50)), (80, (-400, 200, 0)), (120, (1500, 900, -60)),
        (160, (0, 0, 0)), (200, (35, -20, 5)), (240, (2500, -2500, 800))]


class Battery:
    def __init__(self, kind, mac, period, plan):
        """``plan``: [(from t, phase or None to stay silent, reported W)]"""
        self.kind, self.mac, self.period, self.plan = kind, mac, period, plan
        self.next = 0.0

    def state(self, t):
        cur = None
        for start, phase, w in self.plan:
            if t >= start:
                cur = (phase, w)
        return cur


def batteries() -> List[Battery]:
    return [
        Battery("HMG-50", "c8478c00aa01", 1.0, [(0, "A", 0), (60, "A", 150), (130, "A", -300), (230, "A", 600)]),
        Battery("VNSE3-0", "c8478c00aa02", 1.3, [(5, "B", 0), (90, "B", -800), (180, None, 0), (220, "B", 0)]),
        Battery("HMJ-2", "c8478c00aa03", 2.0, [(20, "0", 0), (70, "A", 0), (150, "C", 100), (200, "D", 250)]),
    ]


def grid_at(t):
    cur = GRID[0][1]
    for start, g in GRID:
        if t >= start:
            cur = g
    return cur


def settle_window(t, settle):
    return any(start <= t < start + settle for start, _ in GRID[1:])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("firmware", help="a CT002 (HME-4) image")
    ap.add_argument("--astrameter", default=os.environ.get("ASTRAMETER_SRC"),
                    help="AstraMeter's src directory (or set ASTRAMETER_SRC)")
    ap.add_argument("--seconds", type=float, default=280.0, help="scenario length after the meter is up")
    ap.add_argument("--settle", type=float, default=2.0, help="seconds after a grid step counted as transition")
    ap.add_argument("--json", help="write every poll and both replies here")
    ap.add_argument("--show", type=int, default=12, help="disagreements to print")
    args = ap.parse_args()
    if args.astrameter:
        sys.path.insert(0, args.astrameter)
    logging.disable(logging.WARNING)

    from marsemu.emulator import Emulator
    from marsemu.plant.venus3_cli import ct_fields, ct_payload
    try:
        from astrameter.ct002.ct002 import CT002
    except ImportError:
        print("cannot import astrameter: pass --astrameter /path/to/AstraMeter/src", file=sys.stderr)
        return 2

    emu = Emulator(open(args.firmware, "rb").read())
    meter = getattr(emu, "ct_meter", None)
    if meter is None:
        print("this image has no modelled metering chip (only the CT002, HME-4, has one)", file=sys.stderr)
        return 2
    mod = emu.fc41d
    ct_id = mod.cfg.ble_mac.replace(":", "").lower()

    fw_replies: Dict[str, List[tuple]] = defaultdict(list)     # mac -> [(emulated time, fields)]
    orig_send = mod.net.socket_send

    def send(s, data, extra):
        f = ct_fields(bytes(data))
        if f and len(f) > 4:
            fw_replies[f[3]].append((emu.time, f))
        return orig_send(s, data, extra)
    mod.net.socket_send = send

    def service():
        for sid, s in mod.sockets.items():
            if getattr(s, "kind", "") == "UDP SERVICE" and getattr(s, "local_port", 0) == 12345:
                return sid
        return None

    meter.set_power(*grid_at(0))                 # the grid is there from power-on
    print("booting the CT002 firmware ...", file=sys.stderr)
    while service() is None and emu.time < 90:
        emu.run(1.0)
    if service() is None:
        print("the firmware never opened its UDP service", file=sys.stderr)
        return 1
    emu.run(3.0)
    t0 = emu.time

    ct = CT002(ct_mac=ct_id, ct_type="HME-4", active_control=False, clock=lambda: 1.7e9 + emu.time)
    state = {"grid": grid_at(0)}

    async def before_send(addr, request, consumer_id):
        return list(state["grid"])
    ct.before_send = before_send
    loop = asyncio.new_event_loop()

    class Sink:
        reply = None

        def sendto(self, data, addr):
            self.reply = data

    def astra(data, addr):
        sink = Sink()
        loop.run_until_complete(ct._handle_request(data, addr, sink))
        return ct_fields(sink.reply) if sink.reply else None

    bats = batteries()
    polls = []
    step = 0.05
    while emu.time - t0 < args.seconds:
        t = emu.time - t0
        g = grid_at(t)
        if g != state["grid"]:
            state["grid"] = g
        meter.set_power(*g)
        for k, b in enumerate(bats):
            if t < b.next:
                continue
            b.next = t + b.period
            st = b.state(t)
            if st is None or st[0] is None:
                continue
            phase, w = st
            data = ct_payload([b.kind, b.mac, "HME-4", ct_id, phase, str(int(w))])
            addr = (f"192.168.1.{60 + k}", 12345)
            n_before = len(fw_replies[b.mac])
            sid = service()
            mod.post(lambda d=data, a=addr, sid=sid: mod.socket_received(sid, d, a))
            polls.append({"t": round(t, 2), "mac": b.mac, "phase": phase, "w": w, "grid": list(g),
                          "astra": astra(data, addr), "fw_index": n_before})
        emu.run(step)
    emu.run(1.0)

    # the firmware's answer to a poll is its next reply to that battery after it
    for p in polls:
        got = fw_replies[p["mac"]]
        if len(got) > p["fw_index"]:
            t_reply, p["fw"] = got[p["fw_index"]]
            p["fw_delay"] = round(t_reply - t0 - p["t"], 3)
        else:
            p["fw"], p["fw_delay"] = None, None

    events = sorted({start for start, _ in GRID[1:]} | {start for b in bats for start, _, _ in b.plan[1:]})
    agree = defaultdict(int)
    differ = defaultdict(int)
    transient = defaultdict(int)
    rounding = defaultdict(int)
    shown = []
    missing = {"fw": 0, "astra": 0}
    for p in polls:
        fw, aa = p["fw"], p["astra"]
        if fw is None or aa is None:
            missing["fw" if fw is None else "astra"] += 1
            continue
        # the firmware answers 0.5-1.5 s after a poll, with what it knows then: a
        # grid step, or a battery changing what it reports, in between (or just
        # before the poll, for the grid) makes the two answers differ by timing
        moving = settle_window(p["t"], args.settle) or any(
            p["t"] < start <= p["t"] + (p["fw_delay"] or 0) for start in events)
        for i, name in enumerate(FIELDS):
            if name in IGNORED:
                continue
            a = fw[i] if i < len(fw) else ""
            b = aa[i] if i < len(aa) else ""
            if a == b:
                agree[name] += 1
            elif name == "total_power" and a.lstrip("-").isdigit() and b.lstrip("-").isdigit() \
                    and abs(int(a) - int(b)) == 1:
                # the firmware adds the unrounded phases, AstraMeter whole watts
                rounding[name] += 1
            elif moving:
                transient[name] += 1
            else:
                differ[name] += 1
                if len(shown) < args.show:
                    shown.append((p, name, a, b))

    n = sum(1 for p in polls if p["fw"] is not None and p["astra"] is not None)
    print(f"{len(polls)} polls, {n} answered by both; no firmware reply to {missing['fw']},"
          f" no AstraMeter reply to {missing['astra']}")
    delays = sorted(p["fw_delay"] for p in polls if p.get("fw_delay") is not None)
    if delays:
        print(f"firmware reply delay: median {delays[len(delays) // 2] * 1000:.0f} ms,"
              f" max {delays[-1] * 1000:.0f} ms (AstraMeter answers at once)")
    print(f"{'field':<14}{'agree':>7}{'differ':>8}{'in transition':>15}{'1 W rounding':>14}")
    for name in FIELDS:
        if name in IGNORED:
            continue
        if differ[name] or transient[name] or agree[name] or rounding[name]:
            mark = "  <--" if differ[name] else ""
            print(f"{name:<14}{agree[name]:>7}{differ[name]:>8}{transient[name]:>15}{rounding[name]:>14}{mark}")
    if shown:
        print("\nfirst disagreements (steady grid):")
        for p, name, a, b in shown:
            print(f"  t={p['t']:>6.1f} {p['mac'][-4:]} phase {p['phase']} reports {p['w']:>5} W,"
                  f" grid {p['grid']}: {name} firmware {a!r} AstraMeter {b!r}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(polls, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
