"""Whole devices run for the dashboard.

A session runs a whole device (:class:`B2500Session`, :class:`VenusE3Session`)
in a worker thread, in short slices of emulated time.  Between slices it
applies what the dashboard asked for (house load, sun, faults, speed, MQTT
commands) and takes a snapshot of everything the dashboard draws: the plant,
the cells, the protection state, peripheral registers the firmware writes,
interrupt rates, and a log of the traffic on every bus (AT commands, MQTT,
CT polls, CAN, the extension bus, the front end).  The HTTP server
(:mod:`marsemu.ui.server`) only reads snapshots and queues commands, so
nothing touches the emulator from another thread.
"""
from __future__ import annotations

import collections
import json
import logging
import os
import re
import struct
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


class DeviceSession:
    """What every device run for the dashboard shares: the worker thread,
    commands, the event log, the history, the Wi‑Fi module, the CT002 and the
    faults.  Subclasses build the device (:meth:`_build`) and describe it
    (:meth:`_device_snapshot`, :meth:`_sample`).

    ``network``: a :class:`~marsemu.fc41d.net.Network` (default: offline).
    ``meter``: ``"local"`` (built-in CT002) or ``"HOST:PORT"`` (forward the
    polls, e.g. to AstraMeter).  ``speed``: emulated seconds per wall-clock
    second at most; 0 runs as fast as the host allows."""

    MODEL = "?"
    #: console channels this device has, in display order
    CHANNELS = ("FW", "MQTT", "RF", "BLE", "EVT", "ALARM")
    #: a battery in a house (False: the CT meter)
    HAS_PACK = True
    HAS_HOUSE = True
    SLICE_S = 0.25
    HIST_PERIOD_S = 0.5
    HIST_LEN = 600
    EVENTS_LEN = 600

    def __init__(self, firmware: str, *, load: float = 400.0, soc: float = 50.0, ambient_c: float = 22.0,
                 state_dir: Optional[str] = None, network=None, meter: str = "local", ct_mac: Optional[str] = None,
                 setup: bool = True, speed: float = 1.0, device_type: Optional[str] = None,
                 uid: Optional[bytes] = None, poll_s: float = 30.0, **device_opts):
        from ..fc41d.net import Network
        from ..plant.venus3_cli import DEFAULT_CT_MAC, House, configure_self_consumption, connect_house

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
        self.ambient_c = float(ambient_c)

        kwargs = {"network": network or Network()}
        if state_dir:
            kwargs["state_dir"] = state_dir
        if device_type:
            kwargs["device_type"] = device_type
        if uid:
            kwargs["uid"] = uid
        self.dev = self._build(kwargs, soc=soc, **device_opts)
        self.emu = self.dev.control
        if not device_type:
            try:
                from ..cli import image_info
                info = image_info(firmware)
                device_type = info.get("device_type_default") or next(iter(info.get("device_types") or []), None)
            except Exception:                                    # noqa: BLE001 - only a label
                device_type = None
        self.device_type = device_type
        self.house = House({"A": float(load)})
        self.meter = connect_house(self.dev, self.house, meter, self.ct_mac) if self.HAS_HOUSE else None
        self._hook_module()
        self._hook_device()
        if setup and self.HAS_HOUSE:
            configure_self_consumption(self.dev, self.ct_mac, say=lambda text: self._log("FW", "", text))
        self.dev.sim.every(self.HIST_PERIOD_S, self._sample_history)
        if getattr(self.emu.fc41d.net, "mqtt_mode", "offline") == "offline":
            # nobody else asks an offline device for its status: do what the app does
            self.dev.sim.every(poll_s, self._app_poll)
        self._irq_prev = list(self.emu.machine.nvic.irq_count)
        self._irq_t = 0.0
        self._irq_rates: Dict[int, float] = {}
        self.snapshot: dict = {}
        self._take_snapshot()

    # ------------------------------------------------------------ per device
    def _build(self, control_kwargs: dict, soc: float, **opts):
        raise NotImplementedError

    def _hook_device(self) -> None:
        """Taps on the device's own buses."""

    def _device_snapshot(self) -> dict:
        raise NotImplementedError

    def _sample(self) -> dict:
        """Chart values of this moment, besides grid, AC, battery, SOC and cells."""
        return {"pv": 0.0}

    def _apply_device(self, c: dict) -> bool:
        """Device-specific commands; True if handled."""
        return False

    @property
    def pack(self):
        return getattr(self.dev, "pack", None)

    @property
    def fc41d_uart(self) -> str:
        u = self.emu.fc41d.uart
        return getattr(u, "name", None) or "UART"

    # ------------------------------------------------------------------ hooks
    def _log(self, ch: str, direction: str, text: str) -> None:
        with self.lock:
            self._seq += 1
            self.events.append(Event(seq=self._seq, t=round(self.dev.time, 2), ch=ch, dir=direction, text=text))

    def _hook_module(self) -> None:
        from ..cli import redact_wifi
        e, mod = self.emu, self.emu.fc41d

        def at(line: str) -> None:
            self._log(self.fc41d_uart, "MCU→FC41D", redact_wifi(line)[:300])
        e.on("at", at)

        orig_line = mod.line

        def line(text: str, delay_ms=None) -> None:
            if text not in ("OK", "") and not text.startswith("+QIRD"):
                self._log(self.fc41d_uart, "FC41D→MCU", redact_wifi(text)[:300])
            orig_line(text, delay_ms)
        mod.line = line

        def pub(topic: str, payload) -> None:
            text = payload.decode("latin1", "replace") if isinstance(payload, (bytes, bytearray)) else str(payload)
            m = re.search(r"(?:^|,)vv=(\d+)", text)
            if m and not self.version:
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

        if self.HAS_HOUSE:
            self._hook_meter(mod.net)

        # faults that need the emulator's own time
        def heat(now, dt) -> None:
            if self.faults["heat"] and self.pack is not None:
                self.pack.temp_c = min(75.0, self.pack.temp_c + 4.0 * dt)
        self.dev.sim.tickers.append(heat)

    def _hook_meter(self, net) -> None:
        # the CT002: polls in, answers out; the fault drops them
        meter = net.udp_handler

        def udp(sock, data: bytes, dest) -> None:
            self.ct["polls"] += 1
            self._ct_last_poll = self.dev.time
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
        self._ct_last_poll = None
        self._ct_quiet = False
        self.dev.sim.every(1.0, self._watch_ct)

    def _watch_ct(self, now: float) -> None:
        """Say in the console when the firmware stops polling its CT002, and why it
        might have: the module's Wi-Fi, its UDP socket, the faults."""
        last = self._ct_last_poll
        if last is None:
            return
        quiet = now - last > 10.0
        if quiet and not self._ct_quiet:
            mod = self.emu.fc41d
            udp = [s for s in mod.sockets.values() if str(getattr(s, "kind", "")).startswith("UDP")]
            why = [f"Wi-Fi {'up' if mod.wifi_up else 'down'}",
                   f"UDP socket {'open' if udp else 'closed'}",
                   f"MQTT {'connected' if any(m.connected for m in mod.mqtt.values()) else 'not connected'}"]
            why += [f"fault: {k}" for k, on in self.faults.items() if on]
            self._log("ALARM", "", f"CT002: no poll from the firmware for 10 s (last at t={last:.1f} s) · " + " · ".join(why))
            self._ct_quiet_since = last
        elif not quiet and self._ct_quiet:
            self._log("EVT", "", f"CT002: the firmware polls again, after {now - self._ct_quiet_since:.0f} s without")
        self._ct_quiet = quiet

    def _app_poll(self, now: float) -> None:
        if self.sub_topic and not self.faults["wifi"]:
            self._log("MQTT", "broker→FC41D", f"{self.sub_topic}  cd=1  (status poll, as the app does)")
            self.emu.mqtt_inject(self.sub_topic, "cd=1")

    def _battery_w(self) -> float:
        p = self.pack
        return p.current_a * p.voltage if p is not None else 0.0

    def _sample_history(self, now: float) -> None:
        if not self.HAS_PACK:
            sample = {"t": round(now, 2), **self._sample()}
            with self.lock:
                self.history.append(sample)
            return
        cells = self.pack.cell_voltages
        sample = {
            "t": round(now, 2), "grid": round(self.house.grid()["A"], 1), "ac": round(self.dev.ac_power, 1),
            "bat": round(self._battery_w(), 1), "soc": round(self.pack.soc * 100, 2), "v": round(self.pack.voltage, 2),
            "cmin": round(min(cells) * 1000, 1), "cmax": round(max(cells) * 1000, 1), "bal": False,
            **self._sample(),
        }
        if self._mark:
            sample["ev"], self._mark = self._mark, None
        with self.lock:
            self.history.append(sample)

    # --------------------------------------------------------------- commands
    def command(self, cmd: dict) -> None:
        """Queue a command from the dashboard; applied between slices."""
        with self.lock:
            self._cmds.append(cmd)

    def _apply(self, c: dict) -> None:
        op = c.get("op")
        if self._apply_device(c):
            return
        if op == "load":
            # negative: the house exports on its own (a separate solar system feeding it)
            self.house.loads["A"] = float(c["w"])
        elif op == "ambient":
            self.ambient_c = float(c["c"])
            self.pack.ambient_c = self.ambient_c
            self._set_ambient(self.ambient_c)
        elif op == "fault":
            name, on = c["name"], bool(c["on"])
            if name not in self.faults:
                raise ValueError(f"unknown fault {name!r}")
            self.faults[name] = on
            mod = self.emu.fc41d
            if name == "wifi":
                mod.post(mod.wifi_disconnect if on else mod._wifi_connected)
            if name == "heat" and not on:
                self.pack.temp_c = self.ambient_c
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
            self._repace = True
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

    def _set_ambient(self, c: float) -> None:
        """Device-specific parts of an ambient temperature change."""

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
        elif name == "export":
            base = self.house.loads["A"]
            self.house.loads["A"] = -600.0
            self._log("EVT", "", "scenario: 600 W solar export from the house for 60 s")
            self._mark = "export"

            def back():
                self.house.loads["A"] = base
                self._log("EVT", "", f"scenario: load back to {base:.0f} W")
                self._mark = "load back"
            sim.after(60.0, back)
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
        rate_w, rate_t = wall0, virt0
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
                if getattr(self, "_repace", False):              # a new speed: pace from here
                    self._repace = False
                    wall0, virt0 = time.monotonic(), self.dev.time
                if self.paused and self._step_s <= 0:
                    wall0, virt0 = time.monotonic(), self.dev.time
                    rate_w, rate_t = wall0, virt0
                    self._take_snapshot()
                    time.sleep(0.05)
                    continue
                if self.speed > 0 and not self.paused:
                    ahead = (self.dev.time - virt0) / self.speed - (time.monotonic() - wall0)
                    if ahead > 0.002:
                        # ahead of the clock: wait in short steps, so commands and snapshots
                        # keep coming (at 0.1x a slice is ten times longer in wall time)
                        time.sleep(min(ahead, 0.05))
                        if time.monotonic() - last_snap > 0.1:
                            self._take_snapshot()
                            last_snap = time.monotonic()
                        continue
                    if ahead < -2.0:                         # fell behind: don't race to catch up
                        wall0, virt0 = time.monotonic(), self.dev.time
                # slower than real time: shorter slices, so the picture moves smoothly
                slice_s = self.SLICE_S if not (0 < self.speed < 1) else max(0.01, self.SLICE_S * self.speed)
                dt = min(slice_s, self._step_s) if self.paused else slice_s
                w0 = time.monotonic()
                self.dev.run(dt)
                w1 = time.monotonic()
                if self.paused:
                    self._step_s -= dt
                # the pace you see (emulated per wall second, waits included), over the last second
                if w1 - rate_w >= 1.0:
                    rate = (self.dev.time - rate_t) / (w1 - rate_w)
                    self._wall_rate = rate if not self._wall_rate else 0.5 * self._wall_rate + 0.5 * rate
                    rate_w, rate_t = w1, self.dev.time
                if w1 - last_snap > 0.1:
                    self._take_snapshot()
                    last_snap = w1
        except Exception as exc:                                 # noqa: BLE001 - shown in the dashboard
            log.exception("device stopped")
            self.error = f"{type(exc).__name__}: {exc}"
            self._take_snapshot()

    # ---------------------------------------------------------------- reading
    def _irqs(self) -> List[list]:
        nv = self.emu.machine.nvic
        now = self.dev.time
        if now - self._irq_t >= 1.0:
            cur = list(nv.irq_count)
            span = now - self._irq_t
            self._irq_rates = {i: (c - p) / span for i, (c, p) in enumerate(zip(cur, self._irq_prev)) if c > p}
            self._irq_prev, self._irq_t = cur, now
        names = {16 + n: name for n, name in getattr(nv, "irq_names", {}).items()} or _irq_names()
        names.update({11: "SVC", 14: "PendSV", 15: "SysTick"})
        top = sorted(self._irq_rates.items(), key=lambda kv: -kv[1])[:8]
        return [[names.get(i, f"exc {i}"), round(r, 1)] for i, r in top]

    def _take_snapshot(self) -> None:
        d, p, mod = self.dev, self.pack, self.emu.fc41d
        m = self.emu.machine
        sessions = [s for s in mod.mqtt.values() if s.open or s.connected]
        try:
            pc = m.reg("pc")
        except Exception:                                        # noqa: BLE001 - not every state has a PC
            pc = 0
        snap = {
            "model": self.MODEL, "channels": list(self.CHANNELS),
            "t": round(d.time, 2), "running": not self.paused and self.error is None, "error": self.error,
            "speed": self.speed, "rate": round(self._wall_rate, 2),
            "image": os.path.basename(self.firmware), "profile": self.emu.profile.name,
            "device_type": self.emu.device_type or self.device_type, "version": self.version,
            "load": round(self.house.loads["A"], 1), "grid": round(self.house.grid()["A"], 1),
            "ac": round(d.ac_power, 1), "ambient": self.ambient_c,
            "pack": None if p is None else {
                "soc": round(p.soc * 100, 2), "v": round(p.voltage, 3), "a": round(p.current_a, 3),
                "w": round(self._battery_w(), 1), "temp": round(p.temp_c, 1),
                "cells": [round(v * 1000, 1) for v in p.cell_voltages],
                "capacity_ah": p.capacity_ah, "ah_in": round(p.charged_ah, 3), "ah_out": round(p.discharged_ah, 3)},
            "mcu": {"pc": pc, "cycles": m.clock.cycles, "hz": m.clock.hz, "irqs": self._irqs()},
            "fc41d": {"wifi": bool(mod.wifi_up), "ble": mod.ble_name, "version": mod.cfg.version, "uart": self.fc41d_uart,
                      "mqtt": [{"host": s.host, "port": s.port, "connected": s.connected, "client_id": s.client_id,
                                "subs": list(s.subs)} for s in sessions]},
            "ct": dict(self.ct, mac=self.ct_mac, age=None if self.ct["last_t"] is None else round(d.time - self.ct["last_t"], 1)),
            "faults": dict(self.faults),
            "sub_topic": self.sub_topic,
        }
        dev = self._device_snapshot()
        snap["mcu"].update(dev.pop("mcu", {}))
        snap.update(dev)
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


# =============================================================================== B2500
class B2500Session(DeviceSession):
    """A B2500 in a house: its one MCU, the SH367309 front end and 14 cells,
    two PV inputs, two outputs into a microinverter, extension packs on UART5."""

    MODEL = "B2500"
    CHANNELS = ("FW", "I2C2", "USART1", "UART5", "MQTT", "RF", "BLE", "EVT", "ALARM")

    def __init__(self, firmware: str, *, pv=(300.0, 300.0), extension_packs: int = 0, **kw):
        self.sun = [float(x) for x in pv] + [0.0] * (2 - len(pv))
        self._afe_reads = 0
        super().__init__(firmware, extension_packs=extension_packs, **kw)

    def _build(self, control_kwargs: dict, soc: float, extension_packs: int = 0):
        from ..plant.b2500 import B2500
        from ..plant.battery import LfpPack
        #: a small cell imbalance, as in a real pack, so balancing has something to do
        pack = LfpPack(cells_n=B2500.CELLS, capacity_ah=B2500.CAPACITY_AH, soc=soc / 100.0,
                       temp_c=self.ambient_c, ambient_c=self.ambient_c, spread=0.015)
        dev = B2500(self.firmware, pack=pack, pv=list(self.sun), control_kwargs=control_kwargs,
                    extension_packs=extension_packs)
        self.plant = dev.plant
        return dev

    def _hook_device(self) -> None:
        from ..b2500bus import FrameParser, decode
        parser = FrameParser()

        def uart5(data: bytes) -> None:
            for fr in parser.feed(data):
                d = decode(fr)
                if d["type"] == "status":
                    self._log("UART5", "MCU→bus", f"0x73 status  {d.get('direction', '')} · {d.get('pack_mv', 0) / 1000:.2f} V"
                              f" · {d.get('current_ma', 0) / 1000:+.2f} A · SOC {d.get('soc', '?')} %")
        self.emu.machine.bus.get("UART5").tx_listeners.append(uart5)
        self.dev.sim.every(2.0, self._afe_summary)

    def _afe_summary(self, now: float) -> None:
        afe = self.plant.afe
        n = sum(afe.reads.values()) if isinstance(afe.reads, dict) else 0
        new, self._afe_reads = n - self._afe_reads, n
        mv = [round(v * 1000) for v in self.pack.cell_voltages]
        self._log("I2C2", "AFE→MCU", f"{new} register reads in 2 s · cells {' '.join(map(str, mv))} mV")

    def _sample(self) -> dict:
        afe = self.plant.afe
        return {"pv": round(sum(c.power for c in self.plant.pv), 1), "bal": bool(afe.regs[0x41] | afe.regs[0x42])}

    def _set_ambient(self, c: float) -> None:
        self.plant.heatsink_c = c + 3.0

    def _apply_device(self, c: dict) -> bool:
        op = c.get("op")
        if op == "pv":
            self.sun = [max(0.0, float(x)) for x in c["w"]][:2]
            self.dev.set_pv(self.sun)
            return True
        if op == "scenario" and c.get("name") == "cloud":
            sun = list(self.sun)
            self.dev.set_pv([w * 0.15 for w in sun])
            self._log("EVT", "", "scenario: passing cloud, PV to 15 % for 40 s")
            self._mark = "cloud"

            def sun_back():
                self.dev.set_pv(self.sun)
                self._log("EVT", "", "scenario: cloud passed")
                self._mark = "sun back"
            self.dev.sim.after(40.0, sun_back)
            return True
        if op == "scenario" and c.get("name") == "surplus":
            self.house.loads["A"] = 120.0
            self.sun = [420.0, 410.0]
            self.dev.set_pv(self.sun)
            self._log("EVT", "", "scenario: solar surplus, 830 W PV against 120 W load")
            self._mark = "surplus"
            return True
        return False

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

    def _device_snapshot(self) -> dict:
        st, afe, d = self.plant, self.plant.afe, self.dev
        return {
            "heatsink": round(st.heatsink_c, 1),
            "pv": [{"avail": round(a, 1), "w": round(c.power, 1), "v": round(c.v, 2), "a": round(c.i, 3),
                    "v_ref": round(c.v_ref, 2), "i_lim": round(c.i_lim, 2), "on": bool(c.enabled)}
                   for a, c in zip(self.sun, st.pv)],
            "out": [{"w": round(o.power, 1), "v": round(o.v, 2), "a": round(o.i, 3),
                     "i_lim": round(o.i_lim, 3), "on": bool(o.enabled)} for o in st.out],
            "inv": [{"v_set": round(i.v_set, 2)} for i in st.inverters],
            "afe": {"fets": afe.fets(), "bal": (afe.regs[0x41] << 8) | afe.regs[0x42],
                    "bstatus": [afe.regs[0x43], afe.regs[0x44], afe.regs[0x45]],
                    "reads": sum(afe.reads.values()) if isinstance(afe.reads, dict) else 0},
            "ext": [{"soc": round(q.soc * 100, 2), "v": round(q.voltage, 2), "a": round(q.current_a, 2)}
                    for q in st.ext_packs],
            "bus": {k: v for k, v in (d.bus.status or {}).items() if k not in ("raw", "cmd", "len", "type")},
            "mcu": {"regs": self._registers(),
                    "adc": {f"{a} ch{c}": v for (a, c), v in sorted(st.adc.items())}},
        }


# ======================================================================= CAN decoding
#: what the control asks of an inverter board, by function code (see VenusMicro._command)
MICRO_FUNCS = {0x01: "setpoint", 0x02: "backup flag", 0x03: "discharge limit", 0x04: "charge limit",
               0x05: "save statistics", 0x06: "standby", 0x07: "EMS sequence", 0x08: "parallel mode",
               0x0A: "cut-off SOC", 0x10: "telemetry", 0x11: "configuration", 0x12: "status", 0x13: "limits",
               0x14: "limits and SOC", 0x50: "stop", 0x57: "backup mode", 0x58: "setpoint (mode 1)",
               0xC1: "self-test", 0xC2: "clear", 0xCB: "version"}
MPPT_FUNCS = {0x01: "PV limit", 0x02: "charge voltage", 0x03: "status block", 0xCB: "version"}
#: CAN node addresses of the boards marsemu models: Venus inverter, Jupiter inverter, MPPT
NODES = {0x41: "inverter", 0x11: "inverter", 0x21: "MPPT"}
#: Pylontech-style frames of the Jupiter C/E's BMS
PYLON = {0x351: "limits", 0x355: "SOC", 0x356: "pack", 0x359: "protection", 0x35C: "flags", 0x35E: "name"}


def decode_venus_can(ident: int, data: bytes, sender: str, ext: bool = True) -> tuple:
    """(direction, short label, text) for a frame on a Venus or Jupiter internal CAN bus."""
    d = data + bytes(8)
    if not ext:
        if ident == 0x305:
            return "control→BMS", "BMS poll", "0x305  BMS request"
        if ident == 0x351:
            cv, ci, di = struct.unpack_from("<HHH", d)
            text = f"limits: charge to {cv / 10:.1f} V at {ci / 10:.1f} A, discharge {di / 10:.1f} A"
        elif ident == 0x355:
            text = f"SOC {struct.unpack_from('<H', d)[0]} % · SOH {struct.unpack_from('<H', d, 2)[0]} %"
        elif ident == 0x356:
            v, i, t = struct.unpack_from("<Hhh", d)
            text = f"{v / 100:.2f} V · {i / 10:+.1f} A · {t / 10:.1f} °C"
        elif 0x360 <= ident <= 0x363:
            first = 4 * (ident - 0x360) + 1
            text = f"cells {first}–{first + 3}: " + " ".join(f"{v} mV" for v in struct.unpack_from("<4H", d))
        else:
            text = PYLON.get(ident, "") + (" " if ident in PYLON else "") + data.hex()
        label = "cells" if 0x360 <= ident <= 0x363 else PYLON.get(ident, f"0x{ident:03X}")
        return "BMS→control", label, f"0x{ident:03X}  {text}"
    if ident == 0x1801FFAA:
        return "control→BMS", "BMS poll", "0x1801FFAA  BMS poll"
    if (ident & 0x1F00FFFF) == 0x1800AA01:
        pf = (ident >> 16) & 0xFF
        if 0x30 <= pf <= 0x3F:
            first = 4 * (pf - 0x30) + 1
            mv = struct.unpack_from("<4H", d)
            return "BMS→control", "cells", f"0x{ident:08X}  cells {first}–{first + 3}: " + " ".join(f"{v} mV" for v in mv)
        if pf == 0x40:
            return "BMS→control", "temps", f"0x{ident:08X}  temperatures " + " ".join(
                f"{v / 10:.1f} °C" for v in struct.unpack_from("<4h", d))
        if pf == 1:
            v, i, t, soc = struct.unpack_from("<HhhH", d)
            text = f"{v / 100:.2f} V · {i / 10:+.1f} A · {t / 10:.1f} °C · SOC {soc / 10:.1f} %"
        elif pf == 3:
            cv, ci, di = struct.unpack_from("<HHH", d)
            text = f"limits: charge to {cv / 10:.1f} V at {ci / 10:.1f} A, discharge {di / 10:.1f} A · flags 0x{d[6]:02x}"
        elif pf == 4:
            text = f"protection 0x{struct.unpack_from('<I', d)[0]:08x}"
        elif pf == 2:
            text = f"status {data.hex()}"
        else:
            text = data.hex()
        return "BMS→control", f"BMS 0x{pf:02X}", f"0x{ident:08X}  {text}"
    func, src, dst = ident & 0xFF, (ident >> 16) & 0xFF, (ident >> 8) & 0xFF
    if src in NODES:
        node = NODES[src]
        names = MPPT_FUNCS if node == "MPPT" else MICRO_FUNCS
        name = names.get(func, f"func 0x{func:02X}")
        if func == 0x01 and len(data) >= 4 and node == "inverter":
            text = f"setpoint echo {struct.unpack_from('<i', d)[0]} W"
        elif not data:
            text = f"ack {name}"
        else:
            text = f"{name} part {ident >> 24}: {data.hex()}"
        return f"{node}→control", name if data else "ack", f"0x{ident:08X}  {text}"
    if dst in NODES:
        node = NODES[dst]
        names = MPPT_FUNCS if node == "MPPT" else MICRO_FUNCS
        name = names.get(func, f"func 0x{func:02X}")
        if func == 0x01 and len(data) >= 4:
            text = f"{'setpoint' if node == 'inverter' else 'PV limit'} {struct.unpack_from('<i', d)[0]} W"
        elif func in (0x03, 0x04) and len(data) >= 4 and node == "inverter":
            text = f"{name} {struct.unpack_from('<i', d)[0]} W"
        else:
            text = name + (f" {data.hex()}" if data else "")
        return f"control→{node}", name, f"0x{ident:08X}  {text}"
    return f"{sender}→bus", f"0x{ident:X}", f"0x{ident:08X}  {data.hex()}"


# ================================================================== the CAN family
class CanBoardSession(DeviceSession):
    """Devices with a control MCU on an internal CAN bus: the Venus E 3.0, D
    and A, the Jupiter and the Venus C/E.  Taps the bus, decodes every frame,
    and keeps what the BMS last reported."""

    CHANNELS = ("FW", "CAN", "UART6", "MQTT", "RF", "BLE", "EVT", "ALARM")

    def __init__(self, firmware: str, *, bms: Optional[str] = None, device_type: Optional[str] = None, **kw):
        self.bms_image = bms
        self.can = {"frames": 0, "by_dir": collections.Counter()}
        self._can_last: Dict[str, float] = {}
        self.bms_seen: Dict[str, float] = {}
        super().__init__(firmware, **kw)
        if device_type:
            self.device_type = device_type

    def _hook_device(self) -> None:
        def frame(f) -> None:
            self.can["frames"] += 1
            sender = getattr(f.sender, "name", "?")
            direction, label, text = decode_venus_can(f.ident, f.data, sender, f.ext)
            self.can["by_dir"][direction] += 1
            if direction == "BMS→control":
                self._note_bms(f.ident, f.data, f.ext)
            # the bus carries hundreds of frames a second: log each kind at most every 2 s
            key = f"{direction}:{label}"
            if f.t - self._can_last.get(key, -1e9) >= 2.0:
                self._can_last[key] = f.t
                self._log("CAN", direction, text)
        self.dev.bus.monitors.append(frame)

    def _note_bms(self, ident: int, data: bytes, ext: bool) -> None:
        d, b = data + bytes(8), self.bms_seen
        if ext and (ident >> 16) & 0xFF == 1:
            v, i, t, soc = struct.unpack_from("<HhhH", d)
            b.update(v=v / 100, a=i / 10, temp=t / 10, soc=soc / 10, t=self.dev.time)
        elif ext and (ident >> 16) & 0xFF == 3:
            cv, ci, di = struct.unpack_from("<HHH", d)
            b.update(charge_v=cv / 10, charge_a=ci / 10, discharge_a=di / 10, flags=d[6])
        elif ext and (ident >> 16) & 0xFF == 4:
            b["protect"] = struct.unpack_from("<I", d)[0]
        elif not ext and ident == 0x356:
            v, i, t = struct.unpack_from("<Hhh", d)
            b.update(v=v / 100, a=i / 10, temp=t / 10, t=self.dev.time)
        elif not ext and ident == 0x355:
            b["soc"] = float(struct.unpack_from("<H", d)[0])
        elif not ext and ident == 0x351:
            cv, ci, di = struct.unpack_from("<HHH", d)
            b.update(charge_v=cv / 10, charge_a=ci / 10, discharge_a=di / 10)

    def _bms_snapshot(self) -> dict:
        b = self.bms_seen
        return {"soc": b.get("soc"), "v": b.get("v"), "a": b.get("a"), "charge_v": b.get("charge_v"),
                "charge_a": b.get("charge_a"), "discharge_a": b.get("discharge_a"), "flags": b.get("flags", 0),
                "protect": b.get("protect", 0), "seen": round(self.dev.time - b["t"], 1) if "t" in b else None,
                "image": os.path.basename(self.bms_image) if self.bms_image else None}

    def _set_ambient(self, c: float) -> None:
        stage = getattr(self, "stage", None)
        if stage is not None and hasattr(stage, "ambient_c"):
            stage.ambient_c = c

    def _can_snapshot(self) -> dict:
        return {"frames": self.can["frames"], "by_dir": dict(self.can["by_dir"])}


class VenusBoardSession(CanBoardSession):
    """A Venus E 3.0, D or A: control and BMS firmware (each BMS in its own
    process), the inverter board model, the MPPT model on the D and A, and one
    or more packs."""

    DEVICE = "VenusE3"
    PV_INPUTS = 0

    def __init__(self, firmware: str, *, bms: str, pv=(300.0, 300.0, 0.0, 0.0), packs: int = 1, **kw):
        self.sun = [float(x) for x in pv][:4] + [0.0] * (4 - len(pv[:4]))
        self.n_packs = max(1, int(packs))
        super().__init__(firmware, bms=bms, **kw)

    def _build(self, control_kwargs: dict, soc: float, **_):
        from ..plant import venus3, venusa, venusd
        from ..plant.battery import LfpPack
        cls = {"VenusE3": venus3.VenusE3, "VenusD": venusd.VenusD, "VenusA": venusa.VenusA}[self.DEVICE]
        n = min(self.n_packs, cls.MAX_PACKS)
        packs = [LfpPack(cells_n=cls.CELLS, capacity_ah=cls.CAPACITY_AH, soc=soc / 100.0, temp_c=self.ambient_c,
                         ambient_c=self.ambient_c, spread=0.01) for _ in range(n)]
        self.stage = venus3.PowerStage(ambient_c=self.ambient_c)
        kw = {"pv": self.sun} if self.PV_INPUTS else {}
        dev = cls(self.firmware, self.bms_image, pack=packs[0], packs=packs if n > 1 else None, stage=self.stage,
                  control_kwargs=control_kwargs, parallel=True, **kw)
        self.micro = dev.micro
        return dev

    def _apply_device(self, c: dict) -> bool:
        if c.get("op") == "pv" and self.PV_INPUTS:
            self.sun = [max(0.0, float(x)) for x in c["w"]][:4]
            self.sun += [0.0] * (4 - len(self.sun))
            self.dev.mppt.set_pv(self.sun)
            return True
        if c.get("op") == "scenario" and c.get("name") == "cloud" and self.PV_INPUTS:
            sun = list(self.sun)
            self.dev.mppt.set_pv([w * 0.15 for w in sun])
            self._log("EVT", "", "scenario: passing cloud, PV to 15 % for 40 s")
            self._mark = "cloud"

            def sun_back():
                self.dev.mppt.set_pv(self.sun)
                self._log("EVT", "", "scenario: cloud passed")
                self._mark = "sun back"
            self.dev.sim.after(40.0, sun_back)
            return True
        return False

    def _sample(self) -> dict:
        mppt = getattr(self.dev, "mppt", None)
        return {"pv": round(sum(mppt.pv_w), 1) if mppt else 0.0}

    def _micro_snapshot(self) -> dict:
        mi = self.micro
        hi, lo = mi.limits()
        return {"reference": round(mi.reference, 1), "ac": round(mi.ac_power, 1), "dc": round(mi.dc_power, 1),
                "connected": bool(mi.connected), "limit_discharge": round(hi), "limit_charge": round(lo),
                "charge_limit_w": mi.charge_limit_w, "discharge_limit_w": mi.discharge_limit_w,
                "cutoff_soc": mi.cutoff_soc, "max_soc": mi.max_soc, "work_mode": mi.work_mode,
                "temp_llc": round(mi.temp_llc, 1), "temp_inv": round(mi.temp_inv, 1),
                "errors": [mi.err1, mi.err2, mi.warn1], "version": mi.version}

    def _mppt_snapshot(self) -> Optional[dict]:
        mppt = getattr(self.dev, "mppt", None)
        if mppt is None:
            return None
        return {"avail": [round(x, 1) for x in self.sun], "pv": [round(x, 1) for x in mppt.pv_w],
                "limit": mppt.limit_w, "dc": round(mppt.dc_power, 1), "mode": mppt.mode,
                "version": getattr(mppt, "version", None)}

    def _device_snapshot(self) -> dict:
        mi, d = self.micro, self.dev
        bms = self._bms_snapshot()
        bms.update({k: v for k, v in {"soc": mi.bms_soc, "v": mi.bms_v, "a": mi.bms_i, "charge_v": mi.bms_chg_v,
                                       "charge_a": mi.bms_chg_i, "discharge_a": mi.bms_dis_i, "flags": mi.bms_flags,
                                       "protect": mi.bms_protect}.items()})
        bms.update(mos=d.mos(), process="own process", real=True)
        packs = [{"soc": round(p.soc * 100, 1), "v": round(p.voltage, 2), "a": round(p.current_a, 2),
                  "mos": m} for p, m in zip(d.packs, d.mos_all())]
        return {"inverter": self._micro_snapshot(), "bms": bms, "packs": packs, "mppt": self._mppt_snapshot(),
                "can": self._can_snapshot()}


class VenusE3Session(VenusBoardSession):
    """A Venus E 3.0: control and BMS firmware, the inverter board model, one pack."""
    MODEL = "Venus E 3.0"
    DEVICE = "VenusE3"

    def __init__(self, firmware: str, *, device_type: Optional[str] = None, **kw):
        kw.pop("pv", None)
        super().__init__(firmware, device_type=device_type or "VNSE3-0", **kw)


class VenusDSession(VenusBoardSession):
    """A Venus D: control and BMS firmware, the inverter and MPPT models, its packs."""
    MODEL = "Venus D"
    DEVICE = "VenusD"
    PV_INPUTS = 4

    def __init__(self, firmware: str, *, device_type: Optional[str] = None, **kw):
        super().__init__(firmware, device_type=device_type or "VNSD-0", **kw)


class VenusASession(VenusBoardSession):
    """A Venus A: control and BMS firmware (SH36735 front end, 13 cells), the
    inverter model with its four PV inputs, its packs."""
    MODEL = "Venus A"
    DEVICE = "VenusA"
    PV_INPUTS = 4

    def __init__(self, firmware: str, *, device_type: Optional[str] = None, **kw):
        super().__init__(firmware, device_type=device_type or "VNSA-0", **kw)


# ============================================================================ Jupiter
class JupiterSession(VenusBoardSession):
    """A Jupiter C/E or C Plus: the control firmware, the inverter and MPPT
    models, and the BMS: the C Plus's real BMS firmware when its image is
    given, else a BMS model speaking the Pylontech-style set."""

    MODEL = "Jupiter"
    PV_INPUTS = 4
    CHANNELS = ("FW", "CAN", "USART1", "MQTT", "RF", "BLE", "EVT", "ALARM")

    def __init__(self, firmware: str, *, bms: Optional[str] = None, device_type: Optional[str] = None, **kw):
        kw.pop("packs", None)
        self.cplus = bool(bms) or "JPLS" in os.path.basename(firmware).upper()
        self.MODEL = "Jupiter C Plus" if self.cplus else "Jupiter C/E"
        self.sun = [float(x) for x in kw.pop("pv", (300.0, 300.0, 0.0, 0.0))][:4]
        self.sun += [0.0] * (4 - len(self.sun))
        self.n_packs = 1
        CanBoardSession.__init__(self, firmware, bms=bms, device_type=device_type, **kw)
        if not device_type:
            # the image lists the meters' types too: its own is a Jupiter one
            try:
                from ..cli import image_info
                types = image_info(firmware).get("device_types") or []
            except Exception:                                    # noqa: BLE001 - only a label
                types = []
            own = ("JPLS",) if self.cplus else ("HMM", "HMN")
            self.device_type = next((t for t in types if t.startswith(own)), self.device_type)

    def _build(self, control_kwargs: dict, soc: float, **_):
        from ..plant.battery import LfpPack
        from ..plant.jupiter import Jupiter
        from ..plant.venus3 import PowerStage
        pack = LfpPack(cells_n=Jupiter.CELLS, capacity_ah=Jupiter.CAPACITY_AH, soc=soc / 100.0,
                       temp_c=self.ambient_c, ambient_c=self.ambient_c, spread=0.01)
        self.stage = PowerStage(ambient_c=self.ambient_c)
        dev = Jupiter(self.firmware, self.bms_image, pv=self.sun, pack=pack, stage=self.stage,
                      control_kwargs=control_kwargs)
        self.micro = dev.micro
        return dev

    def _device_snapshot(self) -> dict:
        mi, d = self.micro, self.dev
        bms = self._bms_snapshot()
        bms.update({k: v for k, v in {"soc": mi.bms_soc, "v": mi.bms_v, "a": mi.bms_i, "charge_v": mi.bms_chg_v,
                                       "charge_a": mi.bms_chg_i, "discharge_a": mi.bms_dis_i,
                                       "flags": mi.bms_flags}.items() if v})
        real = d.front is not None
        mos = d.front.mos() if real else {"charge": True, "discharge": True}
        bms.update(mos=mos, real=real, process="this process" if real else "model (Pylontech-style frames)")
        return {"inverter": self._micro_snapshot(), "bms": bms, "packs": [], "mppt": self._mppt_snapshot(),
                "can": self._can_snapshot()}


# ========================================================================== Venus C/E
class VenusCESession(CanBoardSession):
    """A Venus C/E: the control MCU (STM32G474) runs the power stage itself;
    marsemu models it at the control's pins (HRTIM, ADC, relays). The BMS
    firmware runs in its own process on CAN.  About a sixth of real time."""

    MODEL = "Venus C/E"
    CHANNELS = ("FW", "CAN", "LPUART1", "MQTT", "RF", "BLE", "EVT", "ALARM")

    def __init__(self, firmware: str, *, bms: str, device_type: Optional[str] = None, **kw):
        kw.pop("pv", None)
        kw.pop("packs", None)
        super().__init__(firmware, bms=bms, device_type=device_type, **kw)
        if not device_type:
            self.device_type = "HMG-50"

    def _build(self, control_kwargs: dict, soc: float, **_):
        from ..plant.battery import LfpPack
        from ..plant.venusce import VenusCE
        pack = LfpPack(cells_n=VenusCE.CELLS, capacity_ah=VenusCE.CAPACITY_AH, soc=soc / 100.0,
                       temp_c=self.ambient_c, ambient_c=self.ambient_c, spread=0.01)
        dev = VenusCE(self.firmware, self.bms_image, pack=pack, control_kwargs=control_kwargs, parallel=True)
        self.stage = dev.stage
        return dev

    def _battery_w(self) -> float:
        # the pack sees the power stage's current in bursts: its average is the battery power
        return self.stage.p_bat_avg

    def _device_snapshot(self) -> dict:
        st = self.stage.status()
        bms = self._bms_snapshot()
        bms.update(mos=self.dev.mos(), process="own process", real=True)
        return {"stage": st, "bms": bms, "can": self._can_snapshot(),
                "inverter": {"reference": None, "ac": st["ac_w"], "dc": -st["bat_w"], "connected": bool(st["relay"])}}


# ======================================================================= CT002 / CT003
class _MeterDevice:
    """The CT meter on its own timeline: one MCU, no plant."""

    def __init__(self, firmware: str, control_kwargs: dict):
        from ..emulator import Emulator
        from ..plant.cosim import CoSim
        self.sim = CoSim(slice_s=0.01)
        self.control = Emulator(firmware, **control_kwargs)
        self.sim.add_emulator(self.control)
        self.ac_power = 0.0

    @property
    def time(self) -> float:
        return self.sim.now

    def run(self, seconds: float, until=None) -> None:
        self.sim.run(seconds, until)

    def close(self) -> None:
        self.sim.close()
        self.control.save_state()


class CtSession(DeviceSession):
    """A CT002/CT003 meter: its firmware with the Wi‑Fi module model, polled by
    virtual batteries the way real ones poll it; its answers are the firmware's
    own.  The grid it measures is the house (its load spread over the three
    phases) minus what each battery delivers on its phase: on a CT002 through
    its modelled metering chip, on a CT003 (a P1 reader) as the utility
    meter's telegrams."""

    MODEL = "CT002 / CT003"
    HAS_PACK = False
    HAS_HOUSE = False
    CHANNELS = ("FW", "USART1", "UDP", "MQTT", "RF", "BLE", "EVT", "ALARM")
    POLL_S = 2.7

    def __init__(self, firmware: str, *, batteries: int = 2, battery_w: float = 300.0, **kw):
        for k in ("pv", "packs", "bms", "extension_packs"):
            kw.pop(k, None)
        self.batteries = [{"type": "HMJ-2" if k == 0 else "VNSE3-0", "mac": f"c8478c00aa{k + 1:02x}",
                           "phase": "ABC"[k % 3], "w": float(battery_w), "replies": 0, "last": None}
                          for k in range(3)]
        self.n_batteries = max(0, min(3, int(batteries)))
        self.replies = 0
        self.last_reply: Optional[List[str]] = None
        self.house_load = float(kw.get("load", 400.0))
        kw["load"] = 0.0                                        # the clamps below are the house
        super().__init__(firmware, **kw)

    def _build(self, control_kwargs: dict, soc: float, **_):
        return _MeterDevice(self.firmware, control_kwargs)

    @property
    def ct_type(self) -> str:
        """What the batteries call this meter: HME-3 for the CT003 (a P1 reader), HME-4 for the CT002."""
        from ..periph.p1 import P1Meter
        return "HME-3" if isinstance(getattr(self.emu, "ct_meter", None), P1Meter) else "HME-4"

    @property
    def ct_id(self) -> str:
        # the meter answers polls addressed to the module's BLE MAC (its id towards the cloud)
        return self.emu.fc41d.cfg.ble_mac.replace(":", "").lower()

    def _hook_device(self) -> None:
        from ..plant.venus3_cli import ct_fields
        net = self.emu.fc41d.net
        orig = net.socket_send

        def send(s, data, extra):
            f = ct_fields(bytes(data))
            if f and len(f) > 4:
                self.replies += 1
                self.last_reply = f
                bat = next((b for b in self.batteries if b["mac"] == f[3]), None)
                if bat is not None:
                    bat["replies"] += 1
                    bat["last"] = f
                self._log("UDP", "CT→battery", f"to {f[3]}: A {f[4]} W · B {f[5]} W · C {f[6]} W · total {f[7]} W"
                          f" · batteries per phase {'/'.join(f[8:11])} · RSSI {f[12] if len(f) > 12 else '?'}")
            return orig(s, data, extra)
        net.socket_send = send
        self.dev.sim.every(self.POLL_S, self._poll)
        self.clamps = getattr(self.emu, "ct_meter", None)
        if self.clamps is not None:
            self.dev.sim.every(0.2, self._feed_clamps)
        from ..periph.p1 import P1Meter
        if isinstance(self.clamps, P1Meter):
            # the CT003 reads the utility meter's telegrams on USART3: log each one
            self.CHANNELS = self.CHANNELS[:2] + ("P1",) + self.CHANNELS[2:]
            self.clamps.listeners.append(self._log_telegram)
        elif self.clamps is not None:
            # the CT002 reads its metering chip five times a second: log what it read every 2 s
            self.CHANNELS = self.CHANNELS[:2] + ("SPI",) + self.CHANNELS[2:]
            self._spi_reads = 0
            self.clamps.chip.on_read.append(self._count_spi)
            self.dev.sim.every(2.0, self._log_spi)

    #: the OBIS lines the CT003's firmware parses
    P1_LINES = ("1-0:1.7.0", "1-0:2.7.0", "1-0:21.7.0", "1-0:22.7.0", "1-0:41.7.0", "1-0:42.7.0",
                "1-0:61.7.0", "1-0:62.7.0", "1-0:1.8.1", "1-0:1.8.2", "1-0:2.8.1", "1-0:2.8.2")

    def _log_telegram(self, data: bytes) -> None:
        lines = data.decode("ascii", "replace").split("\r\n")
        head = lines[0] if lines else "/"
        obis = [ln for ln in lines if ln.startswith(self.P1_LINES)]
        self._log("P1", "meter→CT", f"{head}  " + "  ".join(obis) + f"  ({len(data)} bytes, {lines[-2] if len(lines) > 1 else ''})")

    def _count_spi(self, addr: int) -> None:
        self._spi_reads += 1

    def _log_spi(self, now: float) -> None:
        if not self._spi_reads:
            return
        regs = self.clamps.chip.regs

        def watts(addr: int) -> str:
            v = regs.get(addr, 0)
            v = v - (1 << 24) if v & 0x800000 else v
            return f"{v} ({v / self.clamps.COUNTS_PER_W:+.0f} W)"
        self._log("SPI", "BL09xx→MCU", f"0x22 {watts(0x22)}  0x23 {watts(0x23)}  0x24 {watts(0x24)}"
                  f"  · {self._spi_reads} register reads in 2 s")
        self._spi_reads = 0

    def clamp_power(self) -> Dict[str, float]:
        """Grid per phase as the clamps see it: house load minus battery output."""
        g = {ph: self.house_load / 3 for ph in "ABC"}
        for b in self.batteries[:self.n_batteries]:
            if b["phase"] in g:
                g[b["phase"]] -= b["w"]
        return g

    def _feed_clamps(self, now: float) -> None:
        g = self.clamp_power()
        self.clamps.set_power(g["A"], g["B"], g["C"])

    def _service_socket(self) -> Optional[int]:
        for sid, s in self.emu.fc41d.sockets.items():
            if getattr(s, "kind", "") == "UDP SERVICE" and getattr(s, "local_port", 0) == 12345:
                return sid
        return None

    def _poll(self, now: float) -> None:
        from ..plant.venus3_cli import ct_payload
        sid = self._service_socket()
        if sid is None or self.faults["wifi"]:
            return
        mod = self.emu.fc41d
        for k, b in enumerate(self.batteries[:self.n_batteries]):
            data = ct_payload([b["type"], b["mac"], self.ct_type, self.ct_id, b["phase"], str(int(b["w"])), "1"])
            self._log("UDP", "battery→CT", f"{b['type']} {b['mac']} polls: phase {b['phase']}, delivering {b['w']:.0f} W")
            mod.post(lambda d=data, k=k: mod.socket_received(sid, d, (f"192.168.1.{60 + k}", 12345)))

    def _apply_device(self, c: dict) -> bool:
        op = c.get("op")
        if op == "batteries":
            self.n_batteries = max(0, min(3, int(c["n"])))
            return True
        if op == "battery_w":
            for b in self.batteries:
                b["w"] = float(c["w"])
            return True
        if op == "load":
            self.house_load = float(c["w"])                    # negative: the house exports
            return True
        if op in ("pv", "ambient", "scenario"):
            return True                                         # nothing to drive on a meter
        return False

    def _sample(self) -> dict:
        f = self.last_reply or []
        num = lambda i: float(f[i]) if len(f) > i and f[i].lstrip("-").isdigit() else 0.0
        # the charts' other series have nothing to show on a meter: zeros keep them drawable
        return {"grid": num(7), "ac": sum(b["w"] for b in self.batteries[:self.n_batteries]), "pv": 0.0,
                "load": self.house_load if self.clamps is not None else 0.0,
                "bat": 0.0, "soc": 0.0, "v": 0.0, "cmin": 0.0, "cmax": 0.0, "bal": False, "replies": self.replies}

    def _device_snapshot(self) -> dict:
        return {"meter": {"id": self.ct_id, "replies": self.replies, "last": self.last_reply,
                          "socket": self._service_socket() is not None,
                          "clamps": ({k: round(v, 1) for k, v in self.clamp_power().items()}
                                     if self.clamps is not None else None),
                          "source": "p1" if self.ct_type == "HME-3" else "chip",
                          "type": self.ct_type,
                          "house_load": self.house_load},
                "load": self.house_load,
                "batteries": [{k: v for k, v in b.items()} for b in self.batteries[:self.n_batteries]]}


SESSION_TYPES: Dict[str, Callable[..., DeviceSession]] = {
    "b2500-v1": B2500Session, "b2500-v2": B2500Session, "b2500-v3": B2500Session, "ct00x": CtSession,
}
#: control profile -> [(session, device types it covers)]: the control image's types tell them apart
PAIRED_TYPES: Dict[str, list] = {
    "venus3": [(VenusE3Session, ("VNSE3-0", "VNSEE3-0")), (VenusDSession, ("VNSD-0",)),
               (VenusASession, ("VNSA-0",))],
    "venus": [(VenusCESession, ("HMG-1", "HMG-25", "HMG-50"))],
    "jupiter": [(JupiterSession, ("JPLS-8H", "HMM-1", "HMM-2", "HMN-1", "HMN-2"))],
}
#: sessions that run without a BMS image too
BMS_OPTIONAL = (JupiterSession,)

__all__ = ["DeviceSession", "B2500Session", "VenusE3Session", "VenusDSession", "VenusASession", "JupiterSession",
           "VenusCESession", "CtSession", "SESSION_TYPES", "PAIRED_TYPES", "BMS_OPTIONAL", "Event",
           "decode_venus_can"]
