"""Start the dashboard as a Home Assistant add-on (``python -m marsemu.ui.addon``).

Reads the add-on options from ``/data/options.json``, takes the MQTT broker
from Home Assistant's MQTT service (the Mosquitto add-on) through the
Supervisor, serves the dashboard on the ingress port and, when
``house_load_entity`` names a power sensor, keeps the emulated house load
equal to it.  Firmware images live in the add-on's config folder
(``/addon_configs/<slug>/firmware`` on the host, ``/config/firmware`` here).
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.error
import urllib.request
from typing import Optional

from ..fc41d.net import Network
from .server import UiApp, serve

log = logging.getLogger("marsemu.addon")
OPTIONS = "/data/options.json"
SUPERVISOR = "http://supervisor"


def supervisor(path: str) -> Optional[dict]:
    token = os.environ.get("SUPERVISOR_TOKEN")
    if not token:
        return None
    r = urllib.request.Request(SUPERVISOR + path, headers={"Authorization": f"Bearer {token}"})
    try:
        with urllib.request.urlopen(r, timeout=10) as resp:
            body = json.loads(resp.read() or b"{}")
    except (urllib.error.URLError, OSError, ValueError) as exc:
        log.warning("Supervisor %s: %s", path, exc)
        return None
    return body.get("data", body) if isinstance(body, dict) else None


def mqtt_network(opts: dict):
    """A factory for the devices' network: Home Assistant's broker, or offline."""
    if opts.get("mqtt", "home_assistant") != "home_assistant":
        log.info("MQTT: offline (the device's MQTT traffic is shown, not sent)")
        return lambda: Network()
    svc = supervisor("/services/mqtt") or {}
    if not svc.get("host"):
        log.warning("MQTT: Home Assistant has no MQTT service (install the Mosquitto broker add-on); running offline")
        return lambda: Network()
    host, port = svc["host"], int(svc.get("port", 1883))
    auth = (svc["username"], svc.get("password", "")) if svc.get("username") else None
    log.info("MQTT: %s:%s", host, port)
    return lambda: Network(mqtt="broker", broker=(host, port), broker_auth=auth)


def follow_load(app: UiApp, entity: str, period: float = 5.0) -> None:
    """Keep the emulated house load at the value of a Home Assistant power sensor."""
    last = None
    while True:
        st = supervisor(f"/core/api/states/{entity}")
        sess = app.session
        if st and sess is not None:
            try:
                w = float(st.get("state"))
                unit = (st.get("attributes") or {}).get("unit_of_measurement", "W")
                w *= 1000.0 if unit == "kW" else 1.0
            except (TypeError, ValueError):
                w = None
            if w is not None and w != last:
                sess.command({"op": "load", "w": max(0.0, w)})
                last = w
        time.sleep(period)


def main() -> int:
    # the emulator's own loggers are chatty at INFO and slow it down: only ours
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    for name in ("marsemu.addon", "marsemu.ui"):
        logging.getLogger(name).setLevel(logging.INFO)
    logging.getLogger("emu.cpu").setLevel(logging.ERROR)
    opts = {}
    if os.path.exists(OPTIONS):
        with open(OPTIONS) as f:
            opts = json.load(f)
    fw_dir = opts.get("firmware_dir") or "/config/firmware"
    app = UiApp(fw_dir, state_dir="/data/state", network=mqtt_network(opts), meter=opts.get("meter") or "local",
                defaults={"pv": [float(opts.get("pv1_w", 300)), float(opts.get("pv2_w", 300))],
                          "load": float(opts.get("load_w", 400)), "soc": float(opts.get("soc", 50)),
                          "speed": float(opts.get("speed", 1)),
                          "extension_packs": int(opts.get("extension_packs", 0))})
    port = int(os.environ.get("INGRESS_PORT", 8099))
    serve(app, "0.0.0.0", port)
    log.info("dashboard on port %d; firmware folder %s", port, fw_dir)
    image = opts.get("image") or ""
    supported = [i["name"] for i in app.images() if i["supported"]]
    if not image and len(supported) == 1:
        image = supported[0]
    if image:
        try:
            app.start({"image": image})
        except (OSError, ValueError) as exc:
            log.error("could not start %s: %s", image, exc)
    if opts.get("house_load_entity"):
        threading.Thread(target=follow_load, args=(app, opts["house_load_entity"]), daemon=True).start()
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
