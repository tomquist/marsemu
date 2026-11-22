#!/usr/bin/env python3
"""Send every ``cd=`` command to a firmware over MQTT and record what it does.

    python tools/command_sweep.py FIRMWARE.bin --cd 0-120 --jobs 4 > sweep.jsonl
    python tools/command_sweep.py FIRMWARE.bin --payload "cd=8,p1=200" --payload "cd=22,p1=1"
    python tools/command_sweep.py --render sweep.jsonl          # a Markdown table

The device boots once, until it has subscribed to its command topic and
settled; then each command runs in a fork of that booted emulator, so a
command that resets, reconfigures or switches the device off cannot change
what the next one sees.  For each command the result has the MQTT messages
the firmware published (topic, payload), its debug-console lines, the AT
commands it sent the module, whether it reset, and, compared with a ``cd=1``
status taken in the same fork, which status fields the command changed.

The HMI runs with its plant (grid and PV), so commands that act on the power
stage show it; other devices run their firmware alone.  Needs ``os.fork``
(Linux, macOS).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


def parse_range(spec: str):
    out = []
    for part in spec.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def parse_status(payload: str) -> dict:
    return dict(kv.split("=", 1) for kv in payload.split(",") if "=" in kv)


class Device:
    """The booted device and what it says, per fork."""

    def __init__(self, firmware: str, pv, state_dir=None):
        from marsemu.emulator import Emulator
        from marsemu.profiles import detect
        image = open(firmware, "rb").read()
        prof = detect(image, firmware)
        self.hmi = None
        if prof.name == "hmi":
            from marsemu.plant.hmi import Hmi
            self.hmi = Hmi(firmware, pv=pv, control_kwargs={"state_dir": state_dir})
            self.emu = self.hmi.control
        else:
            self.emu = Emulator(firmware, state_dir=state_dir)
        self.profile = prof.name
        self.subs = []
        self.log = []
        e = self.emu
        e.on("mqtt_subscribe", self.subs.append)
        e.on("mqtt_publish", lambda t, p: self.log.append(("pub", self.t, t, _text(p))))
        e.on("at", lambda line: self.log.append(("at", self.t, line)))
        orig = e.machine.console

        def console(ch, line):
            if ch not in ("SCIA", e.fc41d.uart.name if e.fc41d.uart else None):
                self.log.append(("con", self.t, ch, line))
            orig(ch, line)
        e.machine.console = console

    @property
    def t(self) -> float:
        return self.hmi.time if self.hmi else self.emu.time

    def run(self, seconds: float, until=None) -> None:
        if self.hmi:
            self.hmi.run(seconds, until)
        else:
            self.emu.run(seconds=seconds, until=until)

    def plant(self) -> dict:
        if not self.hmi:
            return {}
        st = self.hmi.status()
        return {k: st[k] for k in ("ac_w", "relay", "pv_w")}


def _text(p) -> str:
    return p.decode("latin1", "replace") if isinstance(p, (bytes, bytearray)) else str(p)


def status(dev: Device, topic: str, wait: float) -> dict:
    n = len(dev.log)
    dev.emu.mqtt_inject(topic, "cd=1")
    dev.run(wait)
    pubs = [x for x in dev.log[n:] if x[0] == "pub"]
    for p in reversed(pubs):
        st = parse_status(p[3])
        if len(st) > 10:
            return st
    return {}


def one(dev: Device, topic: str, payload: str, wait: float, status_wait: float) -> dict:
    before = status(dev, topic, status_wait)
    resets = dev.emu.machine.reset_count
    n = len(dev.log)
    t0 = dev.t
    dev.emu.mqtt_inject(topic, payload)
    dev.run(wait)
    seen = dev.log[n:]
    after = status(dev, topic, status_wait)
    changed = {k: [before.get(k), v] for k, v in after.items() if before.get(k) != v}
    # fields that move by themselves (energy, timers, measurements) are noise here
    for k in ("ele_d", "ele_h", "ele_s", "ele_m", "pv1_v", "pv1_i", "pv1_p", "pv2_v", "pv2_i", "pv2_p", "grd_f",
              "grd_v", "grd_o", "chp_t", "fb1_v", "fb2_v", "pe1_v", "rssi", "wif_s"):
        changed.pop(k, None)
    return {
        "payload": payload,
        "published": [{"t": round(x[1] - t0, 2), "topic": x[2], "payload": x[3]} for x in seen if x[0] == "pub"],
        "console": [x[3] for x in seen if x[0] == "con"],
        "at": [x[2] for x in seen if x[0] == "at" and not x[2].startswith(("AT+QIRD", "AT+QMTPUB"))][:40],
        "reset": dev.emu.machine.reset_count != resets,
        "status_changed": changed,
        "plant": dev.plant(),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("firmware", nargs="?")
    ap.add_argument("--cd", default="", metavar="RANGE", help="command numbers, e.g. 0-120 or 1,8,11,22")
    ap.add_argument("--payload", action="append", default=[], help="a whole payload to send (repeatable)")
    ap.add_argument("--wait", type=float, default=4.0, help="emulated seconds to watch after each command")
    ap.add_argument("--boot", type=float, default=45.0, help="emulated seconds of boot before sweeping, at most")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--pv", default="300,250", help="HMI: PV strings, W")
    ap.add_argument("--render", metavar="JSONL", help="print a Markdown table of a sweep's results")
    a = ap.parse_args()
    if a.render:
        return render(a.render)
    if not a.firmware:
        ap.error("FIRMWARE needed")
    payloads = [f"cd={n}" for n in parse_range(a.cd)] if a.cd else []
    payloads += a.payload
    if not payloads:
        ap.error("nothing to send: --cd or --payload")
    dev = Device(a.firmware, [float(x) for x in a.pv.split(",")])
    t0 = time.time()
    dev.run(a.boot, until=lambda: bool(dev.subs) and dev.t > 25)
    if not dev.subs:
        print(f"no MQTT subscription after {dev.t:.0f} s", file=sys.stderr)
        return 1
    dev.run(5.0)
    topic = dev.subs[0]
    print(f"booted in {time.time() - t0:.0f} s wall; topic {topic}; {len(payloads)} commands", file=sys.stderr)
    running = {}
    queue = list(payloads)
    while queue or running:
        while queue and len(running) < a.jobs:
            p = queue.pop(0)
            r, w = os.pipe()
            pid = os.fork()
            if pid == 0:
                os.close(r)
                try:
                    res = one(dev, topic, p, a.wait, 2.0)
                except Exception as e:                       # noqa: BLE001 - recorded, not fatal
                    res = {"payload": p, "error": f"{type(e).__name__}: {e}"}
                os.write(w, json.dumps(res).encode())
                os._exit(0)
            os.close(w)
            running[pid] = (p, r)
        pid, _ = os.wait()
        p, r = running.pop(pid)
        data = b""
        while True:
            chunk = os.read(r, 65536)
            if not chunk:
                break
            data += chunk
        os.close(r)
        print(data.decode() if data else json.dumps({"payload": p, "error": "no result"}), flush=True)
        print(f"  {p} done", file=sys.stderr)
    return 0


def render(path: str) -> int:
    rows = [json.loads(line) for line in open(path) if line.strip()]

    def key(r):
        m = parse_status(r["payload"])
        return int(m.get("cd", 0)) if m.get("cd", "0").isdigit() else 0
    print("| Payload | Published | Status changed | Console | Reset |")
    print("|---|---|---|---|---|")
    for r in sorted(rows, key=key):
        if "error" in r:
            print(f"| `{r['payload']}` | error: {r['error']} | | | |")
            continue
        pubs = "<br>".join(f"`{p['topic'].split('/')[-3]}`: {p['payload'][:120]}" for p in r["published"]) or "–"
        ch = ", ".join(f"{k} {v[0]}→{v[1]}" for k, v in r["status_changed"].items()) or "–"
        con = "<br>".join(r["console"][:3]) or "–"
        print(f"| `{r['payload']}` | {pubs} | {ch} | {con} | {'yes' if r['reset'] else ''} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
