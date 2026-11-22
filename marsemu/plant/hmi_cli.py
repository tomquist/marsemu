"""``marsemu hmi``: a whole Saturn M microinverter.

The HMI firmware runs on the emulated TMS320F28035 (CPU and CLA); the plant
model supplies the grid, the two PV strings and the flyback power stage, so
the firmware detects the grid, observes it, closes its relay and tracks the
maximum power point of each string as the real one does.  Power flows about
36 s of emulated time after boot.

Examples::

    marsemu hmi HMI.bin --pv 400,350 -t 300
    marsemu hmi HMI.bin --pv 300 --mqtt broker --broker 127.0.0.1:1883   # for hm2mqtt
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread

log = logging.getLogger("emu.hmi")


def add_parser(sub) -> None:
    v = sub.add_parser("hmi", help="run a whole Saturn M microinverter (HMI firmware, grid and PV models)",
                       description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    v.add_argument("firmware", metavar="FIRMWARE", help="HMI firmware image")
    v.add_argument("--pv", default="400,400", metavar="W[,W]", help="peak power of each PV string, W")
    v.add_argument("--voc", default="49", metavar="V[,V]", help="open-circuit voltage of each PV string, V")
    v.add_argument("--grid-v", type=float, default=230.0, help="grid voltage, V rms")
    v.add_argument("--grid-hz", type=float, default=50.0, help="grid frequency, Hz")
    v.add_argument("--temp", type=float, default=25.0, help="ambient temperature, degC")
    v.add_argument("-t", "--seconds", type=float, default=None, help="virtual seconds to run (default: forever)")
    v.add_argument("--state-dir", help="persist the EEPROM and module state here")
    v.add_argument("--http-port", type=int, default=8090,
                   help="serve /status, POST /pv {\"pv\": [W, W]} and POST /grid {\"on\": .., \"v\": .., \"hz\": ..}; 0 = off")
    v.add_argument("--ble-port", type=int, help="TCP port bridging a BLE central to the MCU (raw bytes in/out)")
    v.add_argument("--ble-ws", metavar="[HOST:]PORT", help="serve Bluetooth over WebSocket on [HOST:]PORT")
    v.add_argument("--ble-peripheral", metavar="TRANSPORT",
                   help="be a Bluetooth LE peripheral (vhci, corebluetooth, or a Bumble transport; needs marsemu[ble])")
    v.add_argument("--serve-uart", action="append", default=[], metavar="NAME:PORT",
                   help="serve SCIA or LINA (the debug console) on a TCP port")
    v.add_argument("--realtime", action="store_true", help="never run ahead of wall-clock time")
    v.add_argument("--at", action="store_true", help="print the AT conversation with the Wi-Fi module")
    v.add_argument("--module-serial", metavar="PORT[@BAUD]",
                   help="a real Wi-Fi module on this serial port instead of the FC41D model")
    v.add_argument("--mqtt", choices=["offline", "broker"], default="offline",
                   help="broker: connect the device to --broker (e.g. for hm2mqtt)")
    v.add_argument("--broker", default="127.0.0.1:1883")
    v.add_argument("--broker-auth", help="user:password")
    v.add_argument("--status-every", type=float, default=5.0, help="print a status line every N virtual seconds")
    v.add_argument("--json-log", metavar="FILE", help="write status lines as JSON")
    v.add_argument("-v", "--verbose", action="count", default=0)


def serve_http(port: int, device) -> ThreadingHTTPServer:
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            return json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")

        def do_GET(self):
            if self.path.startswith("/status"):
                self._json(device.status())
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            try:
                body = self._body()
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            if self.path.startswith("/pv"):
                pv = body.get("pv") if isinstance(body, dict) else body
                try:
                    pv = [float(pv)] if isinstance(pv, (int, float)) else [float(x) for x in pv]
                except (TypeError, ValueError):
                    return self._json({"error": 'expected {"pv": [W, W]}'}, 400)
                device.set_pv(pv)
                return self._json({"pv": pv})
            if self.path.startswith("/grid"):
                try:
                    device.set_grid(bool(body.get("on", True)),
                                    float(body["v"]) if "v" in body else None,
                                    float(body["hz"]) if "hz" in body else None)
                except (TypeError, ValueError, AttributeError):
                    return self._json({"error": 'expected {"on": true, "v": 230, "hz": 50}'}, 400)
                p = device.plant
                return self._json({"on": p.grid_on, "v": p.grid_v, "hz": p.grid_hz})
            self._json({"error": "not found"}, 404)

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def cmd_hmi(args) -> int:
    from ..fc41d.net import Network
    from .hmi import Hmi

    logging.basicConfig(level=logging.WARNING - 10 * min(args.verbose, 2),
                        format="%(levelname)-7s %(name)s: %(message)s")
    logging.getLogger("emu.cpu").setLevel(logging.ERROR)
    host, _, port = args.broker.rpartition(":")
    net = Network(mqtt=args.mqtt, broker=(host or args.broker, int(port or 1883)),
                  broker_auth=tuple(args.broker_auth.split(":", 1)) if args.broker_auth else None)
    dev = Hmi(args.firmware, pv=[float(x) for x in args.pv.split(",")], voc=[float(x) for x in args.voc.split(",")],
              grid_v=args.grid_v, grid_hz=args.grid_hz, ambient_c=args.temp, realtime=args.realtime,
              control_kwargs={"network": net, "state_dir": args.state_dir, "external_module": args.module_serial})
    e = dev.control
    if args.ble_port:
        from ..cli import BleBridge
        BleBridge(e, args.ble_port)
        print(f"BLE on TCP port {args.ble_port}", flush=True)
    if args.ble_ws:
        from ..cli import start_ble_ws
        start_ble_ws(e, args.ble_ws)
    if args.ble_peripheral:
        from ..bleperiph import start_ble_peripheral
        start_ble_peripheral(e, args.ble_peripheral)
    for spec in args.serve_uart:
        from ..uartbridge import UartBridge
        name, _, uport = spec.rpartition(":")
        UartBridge(e, name, int(uport))
        print(f"{name} on TCP port {uport}", flush=True)
    if args.http_port:
        serve_http(args.http_port, dev)
        print(f"status at http://127.0.0.1:{args.http_port}/status", flush=True)
    e.on("mqtt_publish", lambda t, p: log.info("MQTT> %s %s", t, p[:200]))
    if args.at:
        e.on("at", lambda line: print(f"[{dev.time:8.1f}] AT  {line}", flush=True))
        e.on("at_rx", lambda line, ms=None: print(
            f"[{dev.time:8.1f}] AT< {line}" + ("" if ms is None else f"  (+{ms:.0f} ms)"), flush=True))
    jlog = open(args.json_log, "a") if args.json_log else None

    def report(now):
        st = dev.status()
        pv = "  ".join(f"pv{k + 1} {v:5.1f} V {a:5.2f} A {w:6.1f} W"
                       for k, (v, a, w) in enumerate(zip(st["pv_v"], st["pv_a"], st["pv_w"])))
        print(f"[{now:8.1f}] ac {st['ac_w']:6.1f} W  {pv}  relay {'on ' if st['relay'] else 'off'}  "
              f"grid {st['grid_v']:.0f} V {st['grid_hz']:.2f} Hz", flush=True)
        if jlog:
            jlog.write(json.dumps(st) + "\n")
            jlog.flush()
    dev.sim.every(args.status_every, report)
    t0 = time.time()
    try:
        if args.seconds is None:
            while True:
                dev.run(60)
        else:
            dev.run(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        dev.close()
    print(f"{dev.time:.1f} virtual seconds in {time.time() - t0:.1f} s", flush=True)
    return 0
