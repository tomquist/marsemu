#!/usr/bin/env python3
"""Boot many firmware images and summarise how far each one gets.

    python tools/survey.py --seconds 60 --jobs 4 IMAGE... > docs/STATUS.md

Each image runs in its own process; the result is a Markdown table (or JSON
lines with --json).
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


#: directories firmware trees usually sit under; recorded names start below them
FIRMWARE_ROOTS = ("firmware/downloads/", "firmwares/", "firmware/")


def image_name(path: str) -> str:
    """The image's path below its firmware tree (``HMA-1/V232/x.bin``), so that
    results do not depend on where the tree is checked out."""
    p = path.replace(os.sep, "/")
    for root in FIRMWARE_ROOTS:
        i = p.rfind("/" + root) if not p.startswith(root) else -1
        if p.startswith(root):
            return p[len(root):]
        if i >= 0:
            return p[i + 1 + len(root):]
    return "/".join(p.split("/")[-3:])


def run_one(path: str, seconds: float) -> dict:
    sys.path.insert(0, ROOT)
    import logging

    logging.disable(logging.CRITICAL)
    from marsemu.emulator import Emulator

    t0 = time.time()
    res = {"file": image_name(path)}
    try:
        e = Emulator(path)
        res["profile"] = e.profile.name
        first = {}
        counts = {}

        def note(k, v=None):
            counts[k] = counts.get(k, 0) + 1
            first.setdefault(k, round(e.time, 1))
            if v is not None:
                res.setdefault(k + "_value", v)
        e.on("at", lambda l: note("at"))
        e.on("mqtt_connect", lambda s: note("mqtt", f"{s.host}:{s.port}"))
        e.on("mqtt_subscribe", lambda t: note("sub", t))
        e.on("mqtt_publish", lambda t, p: note("pub"))
        e.on("ble_name", lambda n: note("ble", n))
        e.on("http", lambda m, u, b: note("http"))
        reason = e.run(seconds)
        res.update(reason=reason, virtual=round(e.time, 1), wall=round(time.time() - t0, 1),
                   resets=e.machine.reset_count - 1, fault=e.machine.fault,
                   fc41d=e.fc41d.uart.name if e.fc41d.uart else None, first=first, counts=counts)
    except Exception as ex:  # noqa: BLE001
        res["error"] = repr(ex)
    return res


def child(path: str, seconds: float) -> dict:
    try:
        out = subprocess.run([sys.executable, __file__, "--child", path, "--seconds", str(seconds)],
                             capture_output=True, text=True, timeout=seconds * 15 + 120)
    except subprocess.TimeoutExpired:
        return {"file": image_name(path), "error": "timeout"}
    try:
        return json.loads(out.stdout.strip().splitlines()[-1])
    except Exception:  # noqa: BLE001
        return {"file": image_name(path), "error": (out.stderr or out.stdout)[-300:]}


def stage(r: dict) -> str:
    c = r.get("counts", {})
    if "error" in r:
        return "error"
    if c.get("sub"):
        return "MQTT subscribed"
    if c.get("mqtt"):
        return "MQTT connected"
    if c.get("ble"):
        return "BLE up"
    if c.get("at"):
        return "talks to FC41D"
    no_module = str(r.get("profile", "")).startswith("aux") or r.get("profile") == "mppt"
    return "runs (no FC41D)" if no_module else "no FC41D traffic"


def markdown(results) -> str:
    lines = ["| Image | Profile | Result | MQTT topic | t(sub) | Resets | Speed |", "|---|---|---|---|---|---|---|"]
    for r in results:
        name = "/".join(r["file"].split("/")[-3:])
        speed = f"{r['virtual'] / r['wall']:.2f}×" if r.get("wall") else ""
        lines.append(f"| `{name}` | {r.get('profile', '')} | {stage(r)} | `{r.get('sub_value', '')}` | "
                     f"{r.get('first', {}).get('sub', '')} | {r.get('resets', '')} | {speed} |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+", help="firmware images (or result files with --render)")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--render", action="store_true", help="images are JSON-lines result files: print the table")
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.render:
        latest = {}
        for f in a.images:  # later files override earlier results for the same image
            for line in open(f):
                if line.strip():
                    r = json.loads(line)
                    latest[r["file"]] = r
        rs = list(latest.values())
        rs.sort(key=lambda r: r["file"].split("/")[-3:])
        print(f"Survey: {len(rs)} images, up to {a.seconds:.0f} virtual seconds each.\n")
        print(markdown(rs))
        return
    if a.child:
        print(json.dumps(run_one(a.images[0], a.seconds)))
        return
    results = []
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        futs = {ex.submit(child, p, a.seconds): p for p in a.images}
        for fut in cf.as_completed(futs):
            r = fut.result()
            results.append(r)
            if a.json:  # stream, so partial runs are not lost
                print(json.dumps(r), flush=True)
    order = {image_name(p): i for i, p in enumerate(a.images)}
    results.sort(key=lambda r: order.get(r["file"], 0))
    if not a.json:
        print(f"Survey: {len(results)} images, {a.seconds:.0f} virtual seconds each.\n")
        print(markdown(results))


if __name__ == "__main__":
    main()
