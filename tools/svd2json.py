#!/usr/bin/env python3
"""Convert a CMSIS SVD file into the compact register map used for traces.

Usage: svd2json.py N32G457.svd marsemu/data/n32g45x.json
"""
import json
import sys
import xml.etree.ElementTree as ET


def main(src, dst):
    root = ET.parse(src).getroot()
    periphs = {p.findtext("name"): p for p in root.iter("peripheral")}
    out = {"device": root.findtext("name"), "peripherals": {}}
    for name, p in periphs.items():
        base = int(p.findtext("baseAddress"), 0)
        tmpl = periphs[p.get("derivedFrom")] if p.get("derivedFrom") else p
        regs = {}
        for r in tmpl.iter("register"):
            regs[str(int(r.findtext("addressOffset"), 0))] = r.findtext("name").split("_", 1)[-1]
        irqs = {i.findtext("name"): int(i.findtext("value")) for i in p.findall("interrupt")}
        out["peripherals"][name] = {"base": base, "regs": regs, "irqs": irqs}
    json.dump(out, open(dst, "w"), indent=0, sort_keys=True)


if __name__ == "__main__":
    main(*sys.argv[1:3])
