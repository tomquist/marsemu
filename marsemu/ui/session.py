"""A whole device run for the dashboard.

:class:`B2500Session` runs a :class:`~marsemu.plant.b2500.B2500` in a worker
thread, in short slices of emulated time.  Between slices it applies what the
dashboard asked for (house load, sun, faults, speed, MQTT commands) and takes
a snapshot of everything the dashboard draws: the plant, the cells, the front
end's flags, peripheral registers the firmware writes, interrupt rates, and a
log of the traffic on every bus (AT commands, MQTT, CT polls, the extension
bus, the front end).  The HTTP server (:mod:`marsemu.ui.server`) only reads
snapshots and queues commands, so nothing touches the emulator from another
thread.
"""
from __future__ import annotations

import collections
import json
import logging
import os
import re
import threading
import time
from typing import Callable, Deque, Dict, List, Optional

log = logging.getLogger("marsemu.ui")

#: exception number -> interrupt name, from the SVD map
_IRQ_NAMES: Dict[int, str] = {}


def _irq_names() -> Dict[int, str]:
    if not _IRQ_NAMES:
        path = os.path.join(os.path.dirname(__file__), "..", "data", "n32g45x.json")
        with open(path) as f:
            for p in json.load(f)["peripherals"].values():
                for name, n in p.get("irqs", {}).items():
                    _IRQ_NAMES[16 + n] = name
        _IRQ_NAMES.update({11: "SVC", 14: "PendSV", 15: "SysTick"})
    return _IRQ_NAMES


class Event(dict):
    """One line of bus traffic: ``seq``, ``t`` (emulated s), ``ch``, ``dir``, ``text``."""


class B2500Session:
    """A B2500 in a house, run for the dashboard.

    ``firmware``: path of the image.  ``network``: a
    :class:`~marsemu.fc41d.net.Network` (default: offline).  ``meter``:
    ``"local"`` (built-in CT002) or ``"HOST:PORT"`` (forward the polls, e.g. to
    AstraMeter).  ``speed``: emulated seconds per wall-clock second at most;
    0 runs as fast as the host allows."""

    MODEL = "B2500"
    SLICE_S = 0.25
    HIST_PERIOD_S = 0.5
    HIST_LEN = 600
    EVENTS_LEN = 600

    def __init__(self, firmware: str, *, pv=(300.0, 300.0), load: float = 400.0, soc: float = 50.0,
                 ambient_c: float = 22.0, extension_packs: int = 0, state_dir: Optional[str] = None,
                 network=None, meter: str = "local", ct_mac: Optional[str] = None, setup: bool = True,
                 speed: float = 1.0, device_type: Optional[str] = None, uid: Optional[bytes] = None,
                 poll_s: float = 30.0):
        from ..plant.b2500 import B2500
        from ..plant.battery import LfpPack
        from ..plant.venus3_cli import DEFAULT_CT_MAC, House, configure_self_consumption, connect_house
        from ..fc41d.net import Network

        self.firmware = firmware
        self.ct_mac = ct_mac or DEFAULT_CT_MAC
        self.lock = threading.Lock()
        self._cmds: Deque[dict] = collections.deque()
        self.events: Deque[Event] = collections.deque(maxlen=self.EVENTS_LEN)
        self.history: Deque[dict] = collections.deque(maxlen=self.HIST_LEN)
        self._seq = 0
        self._mark: Optional[str] = None
        self.speed = float(speed)
        self.paused = False
        self._step_s = 0.0
        self.faults = {"wifi": False, "ct": False, "heat": False}
        self.error: Optional[str] = None
        self.version: Optional[str] = None
        self.sub_topic: Optional[str] = None
        self.ct = {"polls": 0, "answered": 0, "last_grid": None, "last_t": None}
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._wall_rate = 0.0

        #: a small cell imbalance, as in a real pack, so balancing has something to do
        pack = LfpPack(cells_n=B2500.CELLS, capacity_ah=B2500.CAPACITY_AH, soc=soc / 100.0,
                       temp_c=ambient_c, ambient_c=ambient_c, spread=0.015)
        kwargs = {"network": network or Network()}
        if state_dir:
            kwargs["state_dir"] = state_dir
        if device_type:
            kwargs["device_type"] = device_type
        if uid:
            kwargs["uid"] = uid
        self.dev = B2500(firmware, pack=pack, pv=list(pv), control_kwargs=kwargs,
                         extension_packs=extension_packs)
        self.emu = self.dev.control
        if not device_type:
            try:
                from ..cli import image_info
                device_type = image_info(firmware).get("device_type_default")
            except Exception:                                    # noqa: BLE001 - only a label
                device_type = None
        self.device_type = device_type
        self.plant = self.dev.plant
        self.house = House({"A": float(load)})
        self.sun = [float(x) for x in pv] + [0.0] * (2 - len(pv))
        self.ambient_c = float(ambient_c)
        self.meter = connect_house(self.dev, self.house, meter, self.ct_mac)
        self._hook()
        if setup:
            configure_self_consumption(self.dev, self.ct_mac, say=lambda text: self._log("FW", "", text))
        self.dev.sim.every(self.HIST_PERIOD_S, self._sample_history)
        self.dev.sim.every(2.0, self._afe_summary)
        if getattr(self.emu.fc41d.net, "mqtt_mode", "offline") == "offline":
            # nobody else asks an offline device for its status: do what the app does
            self.dev.sim.every(poll_s, self._app_poll)
        self._irq_prev = list(self.emu.machine.nvic.irq_count)
        self._irq_t = 0.0
        self._irq_rates: Dict[int, float] = {}
        self._afe_reads = 0
        self.snapshot: dict = {}
        self._take_snapshot()

    # ------------------------------------------------------------------ hooks
    def _log(self, ch: str, direction: str, text: str) -> None:
        with self.lock:
            self._seq += 1
            self.events.append(Event(seq=self._seq, t=round(self.dev.time, 2), ch=ch, dir=direction, text=text))

    def _hook(self) -> None:
        from ..cli import redact_wifi
        e, mod = self.emu, self.emu.fc41d

        def at(line: str) -> None:
            self._log("USART1", "MCU→FC41D", redact_wifi(line)[:300])
        e.on("at", at)

        orig_line = mod.line

        def line(text: str, delay_ms=None) -> None:
            if text not in ("OK", "") and not text.startswith("+QIRD"):
                self._log("USART1", "FC41D→MCU", redact_wifi(text)[:300])
            orig_line(text, delay_ms)
        mod.line = line

        def pub(topic: str, payload) -> None:
            text = payload.decode("latin1", "replace") if isinstance(payload, (bytes, bytearray)) else str(payload)
            m = re.search(r"(?:^|,)vv=(\d+)", text)
            if m:
                self.version = m.group(1)
            self._log("MQTT", "FC41D→broker", f"{topic}  {text[:400]}")
        e.on("mqtt_publish", pub)

        def http(method: str, url: str, body=None) -> None:
            m = re.search(r"[?&]sv=(\d+)", url)
            if m and not self.version:
                self.version = m.group(1)
            self._log("RF", "FC41D→cloud", f"HTTP {method} {url[:200]}")
        e.on("http", http)

        def sub(topic: str) -> None:
            self.sub_topic = self.sub_topic or topic
            self._log("MQTT", "FC41D→broker", f"subscribe {topic}")
        e.on("mqtt_subscribe", sub)
        e.on("mqtt_connect", lambda *a: self._log("MQTT", "FC41D→broker", "connected " + " ".join(str(x) for x in a)[:200]))
        e.on("wifi", lambda state, *a: self._log("RF", "FC41D", f"Wi‑Fi {state}"))
        e.on("ble_name", lambda name, *a: self._log("BLE", "FC41D", f"BLE name {name}"))
        e.on("ble_state", lambda *a: self._log("BLE", "FC41D", "BLE " + " ".join(str(x) for x in a)))
        e.on("ble_notify", lambda data, *a: self._log("BLE", "FC41D→phone",
                                                       "notify " + (data.hex() if isinstance(data, (bytes, bytearray)) else str(data))[:120]))

        # the CT002: polls in, answers out; the fault drops them
        net = mod.net
        meter = net.udp_handler

        def udp(sock, data: bytes, dest) -> None:
            self.ct["polls"] += 1
            if self.faults["ct"] or self.faults["wifi"]:
                self._log("RF", "FC41D→CT002", f"UDP poll to {dest[0]}:{dest[1]}, no answer (meter offline)")
                return
            self._log("RF", "FC41D→CT002", f"UDP poll to {dest[0]}:{dest[1]}  {data[:60].decode('latin1', 'replace')}")
            meter(sock, data, dest)
        net.udp_handler = udp
        if hasattr(self.meter, "_answer"):
            orig_answer = self.meter._answer

            def answer(sock, f, dest) -> None:
                g = self.house.grid()
                self.ct.update(answered=self.ct["answered"] + 1, last_grid=round(g["A"], 1), last_t=self.dev.time)
                self._log("RF", "CT002→FC41D", f"A {g['A']:+.0f} W  B {g['B']:+.0f} W  C {g['C']:+.0f} W")
                orig_answer(sock, f, dest)
            self.meter._answer = answer

        # the extension bus (UART5)
        from ..b2500bus import FrameParser, decode
        parser = FrameParser()

        def uart5(data: bytes) -> None:
            for fr in parser.feed(data):
                d = decode(fr)
                if d["type"] == "status":
                    self._log("UART5", "MCU→bus", f"0x73 status  {d.get('direction', '')} · {d.get('pack_mv', 0) / 1000:.2f} V"
                              f" · {d.get('current_ma', 0) / 1000:+.2f} A · SOC {d.get('soc', '?')} %")
        self.emu.machine.bus.get("UART5").tx_listeners.append(uart5)

        # faults that need the emulator's own time
        def heat(now, dt) -> None:
            if self.faults["heat"]:
                self.dev.pack.temp_c = min(75.0, self.dev.pack.temp_c + 4.0 * dt)
        self.dev.sim.tickers.append(heat)

    def _app_poll(self, now: float) -> None:
        if self.sub_topic and not self.faults["wifi"]:
            self._log("MQTT", "broker→FC41D", f"{self.sub_topic}  cd=1  (status poll, as the app does)")
            self.emu.mqtt_inject(self.sub_topic, "cd=1")

    def _afe_summary(self, now: float) -> None:
        afe = self.plant.afe
        n = sum(afe.reads.values()) if isinstance(afe.reads, dict) else 0
        new, self._afe_reads = n - self._afe_reads, n
        mv = [round(v * 1000) for v in self.dev.pack.cell_voltages]
        self._log("I2C2", "AFE→MCU", f"{new} register reads in 2 s · cells {' '.join(map(str, mv))} mV")

    def _sample_history(self, now: float) -> None:
        st = self.plant
        cells = self.dev.pack.cell_voltages
        bal = (self.plant.afe.regs[0x41] << 8) | self.plant.afe.regs[0x42]
        sample = {
            "t": round(now, 2), "grid": round(self.house.grid()["A"], 1), "ac": round(st.ac_power, 1),
            "pv": round(sum(c.power for c in st.pv), 1), "bat": round(self._battery_w(), 1),
            "soc": round(self.dev.pack.soc * 100, 2), "v": round(self.dev.pack.voltage, 2),
            "cmin": round(min(cells) * 1000, 1), "cmax": round(max(cells) * 1000, 1), "bal": bool(bal),
        }
        if self._mark:
            sample["ev"], self._mark = self._mark, None
        with self.lock:
            self.history.append(sample)

    def _battery_w(self) -> float:
        p = self.dev.pack
        return p.current_a * p.voltage

    # --------------------------------------------------------------- commands
    def command(self, cmd: dict) -> None:
        """Queue a command from the dashboard; applied between slices."""
        with self.lock:
            self._cmds.append(cmd)

    def _apply(self, c: dict) -> None:
        op = c.get("op")
        if op == "load":
            self.house.loads["A"] = max(0.0, float(c["w"]))
        elif op == "pv":
            self.sun = [max(0.0, float(x)) for x in c["w"]][:2]
            self.dev.set_pv(self.sun)
        elif op == "ambient":
            self.ambient_c = float(c["c"])
            self.dev.pack.ambient_c = self.ambient_c
            self.plant.heatsink_c = self.ambient_c + 3.0
        elif op == "fault":
            name, on = c["name"], bool(c["on"])
            if name not in self.faults:
                raise ValueError(f"unknown fault {name!r}")
            self.faults[name] = on
            mod = self.emu.fc41d
            if name == "wifi":
                mod.post(mod.wifi_disconnect if on else mod._wifi_connected)
            if name == "heat" and not on:
                self.dev.pack.temp_c = self.ambient_c
            label = {"wifi": "Wi‑Fi down", "ct": "CT002 offline", "heat": "pack over-temperature"}[name]
            self._log("ALARM" if on else "EVT", "", f"{label} {'injected' if on else 'cleared'}")
            self._mark = label + ("" if on else " cleared")
        elif op == "pause":
            self.paused = bool(c.get("on", True))
        elif op == "step":
            self.paused = True
            self._step_s += max(0.05, min(60.0, float(c.get("s", 1.0))))
        elif op == "speed":
            self.speed = max(0.0, float(c["x"]))
        elif op == "mqtt":
            if not self.sub_topic:
                raise ValueError("the device has not subscribed to its command topic yet")
            payload = str(c["payload"])
            self._log("MQTT", "broker→FC41D", f"{self.sub_topic}  {payload}")
            self.emu.mqtt_inject(self.sub_topic, payload)
        elif op == "scenario":
            self._scenario(c["name"])
        else:
            raise ValueError(f"unknown command {op!r}")

    def _scenario(self, name: str) -> None:
        sim = self.dev.sim
        if name == "load_step":
            base = self.house.loads["A"]
            self.house.loads["A"] = base + 800
            self._log("EVT", "", f"scenario: load step {base:.0f} → {base + 800:.0f} W for 60 s")
            self._mark = "load +800"

            def back():
                self.house.loads["A"] = base
                self._log("EVT", "", f"scenario: load back to {base:.0f} W")
                self._mark = "load back"
            sim.after(60.0, back)
        elif name == "cloud":
            sun = list(self.sun)
            self.dev.set_pv([w * 0.15 for w in sun])
            self._log("EVT", "", "scenario: passing cloud, PV to 15 % for 40 s")
            self._mark = "cloud"

            def sun_back():
                self.dev.set_pv(self.sun)
                self._log("EVT", "", "scenario: cloud passed")
                self._mark = "sun back"
            sim.after(40.0, sun_back)
        elif name == "surplus":
            self.house.loads["A"] = 120.0
            self.sun = [420.0, 410.0]
            self.dev.set_pv(self.sun)
            self._log("EVT", "", "scenario: solar surplus, 830 W PV against 120 W load")
            self._mark = "surplus"
        else:
            raise ValueError(f"unknown scenario {name!r}")

    # ---------------------------------------------------------------- running
    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="marsemu-ui-device", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=10)
        try:
            self.dev.close()
        except Exception:                                        # noqa: BLE001 - closing is best effort
            log.exception("closing the device")

    def _loop(self) -> None:
        wall0, virt0 = time.monotonic(), self.dev.time
        last_snap = 0.0
        try:
            while not self._stop.is_set():
                with self.lock:
                    cmds = list(self._cmds)
                    self._cmds.clear()
                for c in cmds:
                    try:
                        self._apply(c)
                    except (KeyError, TypeError, ValueError) as exc:
                        self._log("EVT", "", f"command {c.get('op')!r} failed: {exc}")
                if self.paused and self._step_s <= 0:
                    wall0, virt0 = time.monotonic(), self.dev.time
                    self._take_snapshot()
                    time.sleep(0.05)
                    continue
                dt = min(self.SLICE_S, self._step_s) if self.paused else self.SLICE_S
                w0 = time.monotonic()
                self.dev.run(dt)
                w1 = time.monotonic()
                if self.paused:
                    self._step_s -= dt
                rate = dt / max(1e-6, w1 - w0)
                self._wall_rate = rate if not self._wall_rate else 0.85 * self._wall_rate + 0.15 * rate
                if self.speed > 0 and not self.paused:
                    ahead = (self.dev.time - virt0) / self.speed - (time.monotonic() - wall0)
                    if ahead > 0:
                        time.sleep(min(ahead, 0.5))
                    elif ahead < -2.0:                       # fell behind: don't race to catch up
                        wall0, virt0 = time.monotonic(), self.dev.time
                if w1 - last_snap > 0.1:
                    self._take_snapshot()
                    last_snap = w1
        except Exception as exc:                                 # noqa: BLE001 - shown in the dashboard
            log.exception("device stopped")
            self.error = f"{type(exc).__name__}: {exc}"
            self._take_snapshot()

    # ---------------------------------------------------------------- reading
    def _registers(self) -> dict:
        bus = self.emu.machine.bus
        t8, t2, dac = bus.get("TIM8").regs, bus.get("TIM2").regs, bus.get("DAC").regs
        en = dac.get(0x00, 0)
        return {
            "TIM8_CCR3": t8.get(0x3C, 0), "TIM8_CCR4": t8.get(0x40, 0),
            "TIM2_CCR4": t2.get(0x40, 0), "TIM2_CCR3": t2.get(0x3C, 0),
            "DAC_DHR12R1": dac.get(0x08, 0) if en & 1 else None,
            "DAC_DHR12R2": dac.get(0x14, 0) if en & 0x10000 else None,
        }

    def _irqs(self) -> List[list]:
        nv = self.emu.machine.nvic
        now = self.dev.time
        if now - self._irq_t >= 1.0:
            cur = list(nv.irq_count)
            span = now - self._irq_t
            self._irq_rates = {i: (c - p) / span for i, (c, p) in enumerate(zip(cur, self._irq_prev)) if c > p}
            self._irq_prev, self._irq_t = cur, now
        names = _irq_names()
        top = sorted(self._irq_rates.items(), key=lambda kv: -kv[1])[:8]
        return [[names.get(i, f"exc {i}"), round(r, 1)] for i, r in top]

    def _take_snapshot(self) -> None:
        d, p, st, mod = self.dev, self.dev.pack, self.plant, self.emu.fc41d
        m = self.emu.machine
        afe = st.afe
        cells = p.cell_voltages
        sessions = [s for s in mod.mqtt.values() if s.open or s.connected]
        try:
            pc = m.reg("pc")
        except Exception:                                        # noqa: BLE001 - not every state has a PC
            pc = 0
        snap = {
            "model": self.MODEL,
            "t": round(d.time, 2), "running": not self.paused and self.error is None, "error": self.error,
            "speed": self.speed, "rate": round(self._wall_rate, 2),
            "image": os.path.basename(self.firmware), "profile": self.emu.profile.name,
            "device_type": self.emu.device_type or self.device_type, "version": self.version,
            "load": round(self.house.loads["A"], 1), "grid": round(self.house.grid()["A"], 1),
            "ac": round(st.ac_power, 1), "ambient": self.ambient_c, "heatsink": round(st.heatsink_c, 1),
            "pv": [{"avail": round(a, 1), "w": round(c.power, 1), "v": round(c.v, 2), "a": round(c.i, 3),
                    "v_ref": round(c.v_ref, 2), "i_lim": round(c.i_lim, 2), "on": bool(c.enabled)}
                   for a, c in zip(self.sun, st.pv)],
            "out": [{"w": round(o.power, 1), "v": round(o.v, 2), "a": round(o.i, 3),
                     "i_lim": round(o.i_lim, 3), "on": bool(o.enabled)} for o in st.out],
            "inv": [{"v_set": round(i.v_set, 2)} for i in st.inverters],
            "pack": {"soc": round(p.soc * 100, 2), "v": round(p.voltage, 3), "a": round(p.current_a, 3),
                     "w": round(self._battery_w(), 1), "temp": round(p.temp_c, 1),
                     "cells": [round(v * 1000, 1) for v in cells],
                     "capacity_ah": p.capacity_ah, "ah_in": round(p.charged_ah, 3), "ah_out": round(p.discharged_ah, 3)},
            "afe": {"fets": afe.fets(), "bal": (afe.regs[0x41] << 8) | afe.regs[0x42],
                    "bstatus": [afe.regs[0x43], afe.regs[0x44], afe.regs[0x45]],
                    "reads": sum(afe.reads.values()) if isinstance(afe.reads, dict) else 0},
            "ext": [{"soc": round(q.soc * 100, 2), "v": round(q.voltage, 2), "a": round(q.current_a, 2)}
                    for q in st.ext_packs],
            "bus": {k: v for k, v in (d.bus.status or {}).items() if k not in ("raw", "cmd", "len", "type")},
            "mcu": {"pc": pc, "cycles": m.clock.cycles, "hz": m.clock.hz, "irqs": self._irqs(),
                    "regs": self._registers(), "adc": {f"{a} ch{c}": v for (a, c), v in sorted(st.adc.items())}},
            "fc41d": {"wifi": bool(mod.wifi_up), "ble": mod.ble_name, "version": mod.cfg.version,
                      "mqtt": [{"host": s.host, "port": s.port, "connected": s.connected, "client_id": s.client_id,
                                "subs": list(s.subs)} for s in sessions]},
            "ct": dict(self.ct, mac=self.ct_mac, age=None if self.ct["last_t"] is None else round(d.time - self.ct["last_t"], 1)),
            "faults": dict(self.faults),
            "sub_topic": self.sub_topic,
        }
        with self.lock:
            self.snapshot = snap

    def read(self, since_seq: int = 0, since_t: float = -1.0) -> dict:
        """The latest snapshot, plus events and history samples newer than the given ones."""
        with self.lock:
            return {
                "snapshot": self.snapshot,
                "events": [e for e in self.events if e["seq"] > since_seq],
                "history": [h for h in self.history if h["t"] > since_t],
            }


SESSION_TYPES: Dict[str, Callable[..., B2500Session]] = {
    "b2500-v1": B2500Session, "b2500-v2": B2500Session, "b2500-v3": B2500Session,
}

__all__ = ["B2500Session", "SESSION_TYPES", "Event"]
