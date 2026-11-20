"""``python -m marsemu venus3``: a whole Venus E 3.0 for testing EMS software.

The emulated battery polls a CT002 meter like the real one.  The meter is
either built in (a house with constant loads) or an AstraMeter instance the
polls are forwarded to; in both cases the house's grid power is the load minus
what the emulated battery actually delivers.  The grid power is also served
over HTTP in the format AstraMeter's ``[JSON_HTTP]`` powermeter reads, so an
unmodified AstraMeter closes the loop::

    [JSON_HTTP]
    URL = http://127.0.0.1:8090/power
    JSON_PATHS = $.phase_a,$.phase_b,$.phase_c

    [CT002]
    UDP_PORT = 12345
    ACTIVE_CONTROL = True
"""
from __future__ import annotations

import argparse
import json
import logging
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional

log = logging.getLogger("emu.venus3")

SOH, STX, ETX = 1, 2, 3
PHASES = "ABC"


# --------------------------------------------------------------------------- CT002 wire format
def ct_payload(fields: List[str]) -> bytes:
    body = ("|" + "|".join(fields)).encode("ascii")
    base = 1 + 1 + len(body) + 1 + 2
    n = next(base + d for d in range(1, 5) if len(str(base + d)) == d)
    out = bytearray([SOH, STX]) + str(n).encode() + body + bytes([ETX])
    x = 0
    for b in out:
        x ^= b
    return bytes(out + f"{x:02x}".encode())


def ct_fields(data: bytes) -> Optional[List[str]]:
    if len(data) < 10 or data[0] != SOH or data[1] != STX or data[-3] != ETX:
        return None
    sep = data.find(b"|", 2)
    try:
        return data[sep:-3].decode("ascii").split("|")[1:]
    except UnicodeDecodeError:
        return None


# --------------------------------------------------------------------------- house
class House:
    """Loads per phase; the grid sees the load minus what the batteries
    deliver on their phases.

    One battery: :func:`connect_house` keeps ``battery_w`` (on
    ``battery_phase``) up to date.  Several batteries on one house: give each
    its phase, ``connect_house(dev, house, phase="B")``; each then has an
    entry in :attr:`batteries`."""

    def __init__(self, loads: Dict[str, float], battery_phase: str = "A"):
        self.loads = {p: float(loads.get(p, 0.0)) for p in PHASES}
        self.battery_phase = battery_phase
        self.battery_w = 0.0
        #: battery name -> [phase, AC power delivered, W] (see :meth:`add_battery`)
        self.batteries: Dict[str, list] = {}

    def add_battery(self, name: str, phase: str) -> list:
        """Register a battery on ``phase``; returns its ``[phase, watts]``
        entry, which the caller keeps up to date."""
        if phase not in PHASES:
            raise ValueError(f"phase must be one of {PHASES}, not {phase!r}")
        if name in self.batteries:
            raise ValueError(f"battery {name!r} is already in this house")
        entry = self.batteries[name] = [phase, 0.0]
        return entry

    def grid(self) -> Dict[str, float]:
        g = dict(self.loads)
        g[self.battery_phase] -= self.battery_w
        for phase, w in self.batteries.values():
            g[phase] -= w
        return g


def _after(emu, seconds: float, fn) -> None:
    """Run ``fn`` now, or ``seconds`` of the device's emulated time from now."""
    if seconds > 0:
        emu.fc41d.later(seconds * 1000.0, fn, "ct-reply")
    else:
        fn()


class CtMeter:
    """Answers the battery's CT002 polls itself (no AstraMeter needed), the
    way a CT002 in relay mode does: grid power per phase and in total, the
    number of batteries reporting on each phase, the Wi-Fi RSSI and a reply
    counter that advances with every answer."""

    def __init__(self, emu, house: House, mac: str, rssi: int = -55, reply_delay: float = 0.0):
        self.emu, self.house, self.mac = emu, house, mac
        self.rssi = rssi
        #: seconds between a poll and its answer (see :func:`connect_house`)
        self.reply_delay = reply_delay
        self.polls = 0
        self.info_idx = 0
        #: battery MAC -> (phase it reports, power it reports)
        self.reports: Dict[str, tuple] = {}

    def __call__(self, sock, data: bytes, dest) -> None:
        f = ct_fields(data)
        if not f or len(f) < 4:
            return
        self.polls += 1
        _after(self.emu, self.reply_delay, lambda: self._answer(sock, f, dest))

    def _answer(self, sock, f: List[str], dest) -> None:
        if len(f) >= 6:
            participates = len(f) < 7 or f[6] != "0"
            self.reports[f[1]] = (f[4] if participates else "", f[5])
        g = self.house.grid()
        counts = [sum(1 for ph, _ in self.reports.values() if ph == p) for p in PHASES]
        counts.append(sum(1 for ph, _ in self.reports.values() if ph == "D"))
        reply = [f[2] or "HME-4", self.mac, f[0], f[1]] + [str(int(round(g[p]))) for p in PHASES]
        reply += [str(int(round(sum(g.values()))))] + [str(c) for c in counts]
        reply += [str(self.rssi), str(self.info_idx)] + ["0"] * 10
        self.info_idx = (self.info_idx + 1) % 256
        self.emu.fc41d.net.udp_reply(sock, ct_payload(reply), (dest[0], dest[1]))


class CtForwarder:
    """Sends the battery's CT002 polls to a real meter (e.g. AstraMeter) and
    hands the replies back to the firmware."""

    def __init__(self, emu, host: str, port: int):
        self.emu = emu
        self.target = (host, port)
        self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.so.bind(("0.0.0.0", 0))
        self.sock = None
        self.dest = None
        self.polls = 0
        threading.Thread(target=self._rx, daemon=True).start()

    def __call__(self, sock, data: bytes, dest) -> None:
        self.sock, self.dest = sock, dest
        self.polls += 1
        try:
            self.so.sendto(data, self.target)
        except OSError as e:
            log.warning("CT forward failed: %s", e)

    def _rx(self) -> None:
        while True:
            try:
                data, addr = self.so.recvfrom(2048)
            except OSError:
                return
            if self.sock is not None:
                self.emu.fc41d.net.udp_reply(self.sock, data, (self.dest[0], self.dest[1]))


class CtCallback:
    """Answers the battery's CT002 polls with a function called in emulated
    time: ``fn(request, (host, port))`` returns the reply datagram (or None
    for no answer).  ``(host, port)`` is the battery's address as a meter on
    the LAN would see it (the module's IP), so one function can serve several
    batteries.  Lets a CT002 implementation run in-process against the
    firmware without real sockets or wall-clock timing, e.g. AstraMeter's
    ``CT002._handle_request`` in a test."""

    def __init__(self, emu, fn, reply_delay: float = 0.0):
        self.emu, self.fn = emu, fn
        self.polls = 0
        #: seconds between a poll and the call that answers it (see :func:`connect_house`)
        self.reply_delay = reply_delay

    def __call__(self, sock, data: bytes, dest) -> None:
        self.polls += 1
        data = bytes(data)
        _after(self.emu, self.reply_delay, lambda: self._answer(sock, data, dest))

    def _answer(self, sock, data: bytes, dest) -> None:
        src = (self.emu.fc41d.cfg.ip, getattr(sock, "local_port", 0) or dest[1])
        reply = self.fn(data, src)
        if reply:
            self.emu.fc41d.net.udp_reply(sock, reply, (dest[0], dest[1]))


# --------------------------------------------------------------------------- HTTP
def serve_http(port: int, house: House, status, set_pv=None) -> ThreadingHTTPServer:
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

        def do_GET(self):
            if self.path.startswith("/power"):
                g = house.grid()
                self._json({f"phase_{p.lower()}": round(g[p], 1) for p in PHASES}
                           | {"total": round(sum(g.values()), 1)})
            elif self.path.startswith("/status"):
                self._json(status())
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            if self.path.startswith("/load"):
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                except ValueError:
                    return self._json({"error": "bad json"}, 400)
                for p in PHASES:
                    for k in (p, p.lower(), f"phase_{p.lower()}"):
                        if k in body:
                            house.loads[p] = float(body[k])
                return self._json({p: house.loads[p] for p in PHASES})
            if self.path.startswith("/pv") and set_pv is not None:
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                    pv = body.get("pv", body.get("strings"))
                    pv = [float(pv)] if isinstance(pv, (int, float)) else [float(x) for x in pv]
                except (ValueError, TypeError, AttributeError):
                    return self._json({"error": 'expected {"pv": [W, W, W, W]}'}, 400)
                set_pv(pv)
                return self._json({"pv": pv})
            self._json({"error": "not found"}, 404)

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# --------------------------------------------------------------------------- library helpers
DEFAULT_CT_MAC = "112233445566"
#: UDP port the CT002/CT003 answers polls on
CT_UDP_PORT = 12345


def connect_house(device, house: House, meter="local", ct_mac: str = DEFAULT_CT_MAC,
                  reply_delay: Optional[float] = None, phase: Optional[str] = None,
                  name: Optional[str] = None):
    """Put ``device`` (a whole device: VenusE3, VenusD, VenusA, Jupiter, B2500,
    VenusCE) into ``house``: the house sees what the device actually delivers,
    and the device's CT polls are answered from the house's grid power
    (``meter="local"``), forwarded to a CT002 emulator such as AstraMeter
    (``meter="HOST:PORT"``, replies arrive in wall-clock time) or answered by a
    function in emulated time (``meter=fn``, see :class:`CtCallback`).

    ``phase``: the phase the device is connected to.  Without it the device is
    the house's one battery (``house.battery_w`` on ``house.battery_phase``);
    with it the device is added to ``house.batteries`` under ``name``
    (default ``"bat1"``, ``"bat2"``, ...), so several devices can share a house
    (run them with :func:`run_together`).  ``meter="local"`` gives each device
    its own CT model; for one shared CT002 that sees every battery (as the real
    one does), pass the same function as ``meter`` to every device.

    ``reply_delay``: seconds of emulated time between a poll and its answer for
    the local and function meters (default: the device's ``CT_REPLY_DELAY_S``,
    else at once), e.g. to see how a firmware copes with a slow meter.
    Returns the meter (``.polls`` counts the polls)."""
    if phase is None:
        device.sim.tickers.append(lambda now, dt: setattr(house, "battery_w", device.ac_power))
    else:
        entry = house.add_battery(name or f"bat{len(house.batteries) + 1}", phase)
        device.sim.tickers.append(lambda now, dt: entry.__setitem__(1, device.ac_power))
    if reply_delay is None:
        reply_delay = getattr(device, "CT_REPLY_DELAY_S", 0.0)
    if callable(meter):
        m = CtCallback(device.control, meter, reply_delay)
    elif meter == "local":
        m = CtMeter(device.control, house, ct_mac, reply_delay=reply_delay)
    else:
        host, _, port = meter.rpartition(":")
        m = CtForwarder(device.control, host or meter, int(port or 12345))
    def to_meter(sock, data: bytes, dest) -> None:
        # a CT002/CT003 listens on its UDP port only; other broadcasts (the
        # Venus E Mini announces itself to other Minis on 60606) go unanswered
        if dest[1] == CT_UDP_PORT:
            m(sock, data, dest)
    device.control.fc41d.net.udp_handler = to_meter
    return m


def run_together(devices, seconds: float, until=None, slice_s: Optional[float] = None) -> None:
    """Run several whole devices (e.g. three batteries in one house) for
    ``seconds`` of emulated time on one shared timeline: they advance in
    lockstep slices (the shortest of their slices, or ``slice_s``), so a
    shared meter and house see all of them at the same moment.  The devices
    must be at the same emulated time, i.e. created together and only ever
    run with this function.  See :func:`marsemu.plant.cosim.run_lockstep`."""
    from .cosim import run_lockstep
    run_lockstep([d.sim for d in devices], seconds, until, slice_s)


def configure_self_consumption(device, ct_mac: str = DEFAULT_CT_MAC, say=None) -> None:
    """Once the device has subscribed to its MQTT command topic, send the
    commands the app would: self-consumption with the CT ``ct_mac`` (B2500 v2:
    adaptive mode; HMB and old-layout HMA have no CT support and are left on
    their own outputs/timers)."""
    from .b2500 import B2500
    from .jupiter import Jupiter
    e = device.control
    say = say or (lambda text: None)
    topics: List[str] = []

    def setup():
        t = topics[0]
        if isinstance(device, B2500) and device.plant.v1:
            # v1 (HMB) has no meter support: switched outputs; turn both on
            say("B2500 v1: no CT support; switching both outputs on (cd=4,md=3)")
            e.mqtt_inject(t, "cd=4,md=3")
            return
        if isinstance(device, B2500) and b"meter=" not in e.image:
            # old-layout HMA (V119-V220, V241-V255) and HMJ V100: timers only
            say("B2500: this build has no CT support; it runs its timers (factory default: 800 W all day)")
            return
        say(f"configuring self-consumption mode and CT {ct_mac} via {t}")
        if getattr(device, "TWO_DIGIT_COMMANDS", False):
            # Venus E Mini: two-digit codes, the meter as ct_t (3 = CT002), and a
            # phase detection (cd=58): it regulates the grid on its own phase only
            e.mqtt_inject(t, "cd=02,md=0")
            device.sim.after(2.0, lambda: e.mqtt_inject(t, f"cd=18,ct_t=3,mac={ct_mac}"))
            device.sim.after(8.0, lambda: e.mqtt_inject(t, "cd=58,dt=1"))
            return
        if isinstance(device, B2500):
            # the meter command resets the discharge mode: adaptive (md=1) after it
            e.mqtt_inject(t, f"cd=27,meter=3,mac={ct_mac}")
            device.sim.after(2.0, lambda: e.mqtt_inject(t, "cd=18,md=1"))
            return
        # Jupiter: automatic = md=1, meter via cd=16; Venus: md=0, cd=18
        mode, meter_cd = ("1", 16) if isinstance(device, Jupiter) else ("0", 18)
        if getattr(device, "METER_BEFORE_MODE", False):
            # Venus C/E: setting the meter drops it back to manual mode, so the mode goes last
            e.mqtt_inject(t, f"cd={meter_cd},meter=3,mac={ct_mac}")
            device.sim.after(2.0, lambda: e.mqtt_inject(t, f"cd=2,md={mode}"))
            return
        e.mqtt_inject(t, f"cd=2,md={mode}")
        device.sim.after(2.0, lambda: e.mqtt_inject(t, f"cd={meter_cd},meter=3,mac={ct_mac}"))

    def subscribed(t):
        topics.append(t)
        if len(topics) == 1:
            device.sim.after(2.0, setup)
    e.on("mqtt_subscribe", subscribed)


# --------------------------------------------------------------------------- command
def add_parser(sub) -> None:
    for cmd, model in (("venus3", "Venus E 3.0"), ("venusd", "Venus D"), ("venusa", "Venus A"),
                       ("jupiter", "Jupiter C/E/C Plus"), ("venusmini", "Venus E Mini"), ("b2500", "B2500")):
        _add(sub, cmd, model)


def _add(sub, cmd: str, model: str) -> None:
    what = "firmware, power stage and microinverter models" if cmd == "b2500" else \
        "control firmware, inverter, DC/DC and battery models" if cmd == "venusmini" else \
        "control + BMS firmware, inverter model"
    v = sub.add_parser(cmd, help=f"run a whole {model} ({what})",
                       description=f"Run a whole {model} ({what}).\n\n" + __doc__.split("\n\n", 1)[1],
                       formatter_class=argparse.RawDescriptionHelpFormatter)
    v.add_argument("control", metavar="FIRMWARE" if cmd == "b2500" else "CONTROL",
                   help=f"{model} {'' if cmd == 'b2500' else 'control '}firmware image")
    if cmd == "venusmini":
        pass
    elif cmd == "jupiter":
        v.add_argument("bms", nargs="?", metavar="BMS", help="Jupiter C Plus BMS firmware image (JPLS-8H V37); "
                       "without it a BMS model answers the control")
    elif cmd != "b2500":
        v.add_argument("bms", metavar="BMS", help=f"{model} BMS firmware image")
    if cmd in ("venusd", "venusa", "jupiter", "b2500"):
        v.add_argument("--pv", default="0", metavar="W[,W,...]", help="PV power available per input, W")
    if cmd in ("venusd", "venusa"):
        v.add_argument("--packs", type=int, default=1, metavar="N",
                       help="battery packs in the stack, each with its own BMS (same SOC as --soc)")
    if cmd == "b2500":
        v.add_argument("--inverter-w", default="400,400", metavar="W,W",
                       help="AC limit of the microinverter input on each output, W")
        v.add_argument("--extension-packs", type=int, default=0, choices=[0, 1, 2],
                       help="extension battery packs on the bus (same SOC as --soc)")
    v.add_argument("-t", "--seconds", type=float, default=None, help="virtual seconds to run (default: forever)")
    v.add_argument("--state-dir", help="persist control flash/EEPROM and module state here")
    v.add_argument("--soc", type=float, default=50.0, help="initial state of charge, %%")
    v.add_argument("--temp", type=float, default=25.0, help="pack and ambient temperature, degC")
    v.add_argument("--load", default="400", metavar="A[,B,C]", help="house load per phase, W")
    v.add_argument("--phase", default="A", choices=list(PHASES), help="phase the battery is connected to")
    v.add_argument("--meter", default="local", metavar="local|HOST:PORT",
                   help="answer CT polls locally, or forward them to a CT002 emulator such as AstraMeter")
    v.add_argument("--ct-mac", default=DEFAULT_CT_MAC, help="MAC of the CT the battery is configured to use")
    v.add_argument("--no-setup", action="store_true",
                   help="don't configure self-consumption mode and the CT over MQTT after first connect")
    v.add_argument("--http-port", type=int, default=8090, help="serve /power (grid) and /status; 0 = off")
    v.add_argument("--ble-port", type=int, help="TCP port bridging a BLE central to the control MCU (raw bytes in/out)")
    v.add_argument("--serve-uart", action="append", default=[], metavar="NAME:PORT",
                   help="serve a UART of the control MCU on a TCP port"
                        + (" (UART5: the extension-pack bus)" if cmd == "b2500" else ""))
    if cmd in ("venus3", "venusd", "venusa"):
        v.add_argument("--modbus-port", type=int, help="serve the RS485 Modbus slave as Modbus TCP on this port")
        v.add_argument("--modbus-rtu-port", type=int, help="serve the RS485 Modbus slave as RTU over TCP on this port")
    v.add_argument("--realtime", action="store_true", help="never run ahead of wall-clock time")
    v.add_argument("--at", action="store_true", help="print the AT conversation with the Wi-Fi module")
    v.add_argument("--module-serial", metavar="PORT[@BAUD]",
                   help="a real Wi-Fi module on this serial port instead of the FC41D model "
                        "(e.g. /dev/ttyUSB0, socket://host:port); use with --realtime")
    v.add_argument("--slew", type=float, help="inverter slew rate, W/s (model default 1000)")
    v.add_argument("--start-delay", type=float, help="inverter grid-connect delay from standby, s (default 3)")
    v.add_argument("--mqtt", choices=["offline", "broker"], default="offline",
                   help="broker: connect the device to --broker (e.g. for hm2mqtt)")
    v.add_argument("--broker", default="127.0.0.1:1883")
    v.add_argument("--broker-auth", help="user:password")
    v.add_argument("--status-every", type=float, default=5.0, help="print a status line every N virtual seconds")
    v.add_argument("--json-log", metavar="FILE", help="write status lines as JSON")
    v.add_argument("--serial", action="store_true", help="run BMS and control in one process (slower)")
    v.add_argument("-v", "--verbose", action="count", default=0)


def cmd_venus3(args) -> int:
    from ..fc41d.net import Network
    from .battery import LfpPack
    from .venus3 import PowerStage, VenusE3
    from .jupiter import Jupiter
    from .venusa import VenusA
    from .venusd import VenusD
    from .b2500 import B2500
    from .venusmini import VenusMini, mini_stage

    logging.basicConfig(level=logging.WARNING - 10 * min(args.verbose, 2),
                        format="%(levelname)-7s %(name)s: %(message)s")
    logging.getLogger("emu.cpu").setLevel(logging.ERROR)
    loads = [float(x) for x in args.load.split(",")]
    house = House(dict(zip(PHASES, loads)), args.phase)
    host, _, port = args.broker.rpartition(":")
    net = Network(mqtt=args.mqtt, broker=(host or args.broker, int(port or 1883)),
                  broker_auth=tuple(args.broker_auth.split(":", 1)) if args.broker_auth else None)
    stage = mini_stage(args.temp) if args.cmd == "venusmini" else PowerStage(ambient_c=args.temp)
    if args.slew:
        stage.slew_w_per_s = args.slew
    if args.start_delay is not None:
        stage.start_delay_s = args.start_delay
    cls = {"venusd": VenusD, "venusa": VenusA, "jupiter": Jupiter, "b2500": B2500,
           "venusmini": VenusMini}.get(args.cmd, VenusE3)
    pack = LfpPack(soc=args.soc / 100.0, temp_c=args.temp, ambient_c=args.temp, capacity_ah=cls.CAPACITY_AH,
                   cells_n=cls.CELLS)
    extra = {"pv": [float(x) for x in args.pv.split(",")]} if hasattr(args, "pv") else {}
    if getattr(args, "packs", 1) > 1:
        extra["packs"] = [pack] + [LfpPack(soc=args.soc / 100.0, temp_c=args.temp, ambient_c=args.temp,
                                           capacity_ah=cls.CAPACITY_AH, cells_n=cls.CELLS)
                                   for _ in range(args.packs - 1)]
    if args.cmd == "b2500":
        extra["inverter_w"] = [float(x) for x in args.inverter_w.split(",")]
        extra["extension_packs"] = args.extension_packs
    images = (args.control,) if args.cmd in ("b2500", "venusmini") or (args.cmd == "jupiter" and not args.bms) \
        else (args.control, args.bms)
    v = cls(*images, pack=pack, stage=stage, realtime=args.realtime,
            control_kwargs={"network": net, "state_dir": args.state_dir, "external_module": args.module_serial},
            parallel=not args.serial, **extra)
    mppt = getattr(v, "mppt", None)
    set_pv = v.set_pv if hasattr(v, "set_pv") else (mppt.set_pv if mppt else None)
    e = v.control
    meter = connect_house(v, house, args.meter, args.ct_mac)
    if args.ble_port:
        from ..cli import BleBridge
        BleBridge(e, args.ble_port)
        print(f"BLE on TCP port {args.ble_port}", flush=True)
    for spec in args.serve_uart:
        from ..uartbridge import UartBridge
        name, _, uport = spec.rpartition(":")
        UartBridge(e, name, int(uport))
        print(f"{name} on TCP port {uport}", flush=True)
    for port, mode in ((getattr(args, "modbus_port", None), "tcp"), (getattr(args, "modbus_rtu_port", None), "rtu")):
        if port:
            from ..modbus import ModbusGateway
            gw = ModbusGateway(e, port, mode)
            print(f"Modbus {'TCP' if mode == 'tcp' else 'RTU over TCP'} on port {gw.port} ({gw.uart.name}, slave 1)",
                  flush=True)
    status = lambda: {**v.status(), "grid": house.grid(), "ct_polls": meter.polls}
    if args.http_port:
        serve_http(args.http_port, house, status, set_pv)
        print(f"grid power at http://127.0.0.1:{args.http_port}/power, status at /status", flush=True)

    e.on("mqtt_publish", lambda t, p: log.info("MQTT> %s %s", t, p[:200]))
    if args.at:
        e.on("at", lambda line: print(f"[{v.time:8.1f}] AT  {line}", flush=True))
        e.on("at_rx", lambda line, ms=None: print(
            f"[{v.time:8.1f}] AT< {line}" + ("" if ms is None else f"  (+{ms:.0f} ms)"), flush=True))
    if not args.no_setup:
        configure_self_consumption(v, args.ct_mac, say=lambda text: print(f"[{v.time:8.1f}] {text}", flush=True))
    if args.cmd == "venusmini" and args.mqtt == "offline":
        # offline nobody asks for its state: poll like the app, for its SOC in the status line
        topics: List[str] = []
        e.on("mqtt_subscribe", topics.append)
        v.sim.every(30.0, lambda now: topics and e.mqtt_inject(topics[0], "cd=01"))
    jlog = open(args.json_log, "a") if args.json_log else None

    def report(now):
        st = status()
        pvw = st.get("pv_w")
        pv = f"pv {sum(pvw) if isinstance(pvw, list) else pvw:6.1f} W  " if pvw is not None else ""
        ref = f"ref {st['ref_w']:7.1f} W  " if "ref_w" in st else ""
        if args.cmd == "b2500":
            tail = f"out {st['out_w']} W at {st['out_v']} V"
            bms = ""
        else:
            tail = f"limits {st['limits']}"
            bms = f" (bms {st['bms_soc']:.1f}%)" if st.get("bms_soc") is not None else " (bms –)"
        print(f"[{now:8.1f}] ac {st['ac_w']:7.1f} W  {ref}{pv}grid {st['grid'][args.phase]:7.1f} W  "
              f"soc {st['soc']:5.1f}%{bms}  {st['pack_v']:.2f} V  {st['i_a']:6.1f} A  "
              f"{tail}  polls {st['ct_polls']}", flush=True)
        if jlog:
            jlog.write(json.dumps(st) + "\n")
            jlog.flush()
    v.sim.every(args.status_every, report)
    t0 = time.time()
    try:
        if args.seconds is None:
            while True:
                v.run(60)
        else:
            v.run(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        v.close()
    print(f"{v.time:.1f} virtual seconds in {time.time() - t0:.1f} s", flush=True)
    return 0
