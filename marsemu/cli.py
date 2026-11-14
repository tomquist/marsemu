"""Command line interface: ``python -m marsemu ...``"""
from __future__ import annotations

import argparse
import json
import logging
import re
import socket
import struct
import sys
import threading
import time

from . import __version__
from .emulator import Emulator
from .fc41d.module import Fc41dConfig
from .fc41d.net import Network
from .profiles import PROFILES, default_device_type, detect, find_type_tables, load_base

#: --realtime: how far behind wall-clock time the emulator may fall before it
#: stops trying to catch up (see throttle in cmd_run and CoSim.run)
REALTIME_SLACK_S = 0.05

#: Wi-Fi passwords in the AT transcript: AT+QSTAAPINFO[DEF]=<ssid>,<pwd>, and
#: the app's BLE frame "<ssid><.,.><pwd>" that a real module passes on raw.
_WIFI_AT = re.compile(r"(AT\+QSTAAPINFO(?:DEF)?=[^,]*,)(.*)", re.IGNORECASE)
_WIFI_BLE = re.compile(r"(<\.,\.>).*")


def redact_wifi(text: str) -> str:
    text = _WIFI_AT.sub(r"\1<password>", text)
    return _WIFI_BLE.sub(r"\1<password>", text)


log = logging.getLogger("emu.cli")


def _hostport(s: str, default_port: int):
    host, _, port = s.rpartition(":")
    if not host:
        return s, default_port
    return host, int(port)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="marsemu", description="Run Marstek/Hame device firmware in an emulator")
    p.add_argument("--version", action="version", version=__version__)
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="run a firmware image")
    r.add_argument("firmware")
    r.add_argument("--profile", choices=sorted(PROFILES), help="board profile (default: auto-detect)")
    r.add_argument("-t", "--seconds", type=float, default=None, help="virtual seconds to run (default: forever)")
    r.add_argument("--state-dir", help="persist flash and module state (certificates, Wi-Fi) here")
    r.add_argument("--device-type", help="factory device type to seed, e.g. HMA-1 or HMJ-2 (B2500 profiles; default: HMJ-2 for builds for the HMJ board, else the profile's)")
    r.add_argument("--uid", help="MCU unique ID as 24 hex digits (default: derived from the image)")
    r.add_argument("--ble-mac", help=f"BLE MAC reported by the FC41D (default: {DEFAULT_BLE_MAC})")
    r.add_argument("--wifi-mac", help=f"Wi-Fi MAC reported by the FC41D (default: {DEFAULT_WIFI_MAC})")
    r.add_argument("--ssid", default="EmuNet")
    r.add_argument("--no-wifi", action="store_true", help="module starts without stored Wi-Fi credentials")
    r.add_argument("--fc41d-version", default="FC41DAAR18A04M02_HM.bin_202409090159")
    r.add_argument("--fc41d-uart", help="UART the FC41D is on, e.g. USART1 (default: profile / auto-detect)")
    r.add_argument("--module-serial", metavar="PORT[@BAUD]",
                   help="a real Wi-Fi module on this serial port instead of the FC41D model "
                        "(e.g. /dev/ttyUSB0, socket://host:port); use with --realtime")
    r.add_argument("--mqtt", choices=["offline", "broker", "cloud"], default="offline",
                   help="offline: record only; broker: redirect to --broker; cloud: the vendor's broker with the firmware's certificates (needs --confirm-own-device)")
    r.add_argument("--broker", default="127.0.0.1:1883", help="host:port for --mqtt broker")
    r.add_argument("--broker-auth", help="user:password for --mqtt broker")
    r.add_argument("--http", choices=["offline", "real"], default="offline",
                   help="offline: answer locally (--http-rule); real: send the firmware's requests to the vendor's servers")
    r.add_argument("--confirm-own-device", action="store_true",
                   help="required for --mqtt cloud and --http real: confirms that --uid, --wifi-mac and "
                        "--ble-mac are those of a device you own, registered to your own account")
    r.add_argument("--http-rule", action="append", default=[], metavar="REGEX=STATUS:BODY|@FILE",
                   help="answer matching HTTP requests (offline mode), e.g. 'getDeviceVersion=200:{}'")
    r.add_argument("--sockets", choices=["offline", "real"], default="offline",
                   help="real: TCP/UDP client sockets connect, listeners bind on the host")
    r.add_argument("--listen-offset", type=int, default=0, help="host port = device port + offset")
    r.add_argument("--ble-port", type=int, help="TCP port bridging a BLE central (raw bytes in/out)")
    r.add_argument("--serve-uart", action="append", default=[], metavar="NAME:PORT",
                   help="serve a UART on a TCP port, raw bytes both ways (e.g. UART5:7005 = B2500 extension bus)")
    r.add_argument("--modbus-port", type=int, help="serve the RS485 Modbus slave as Modbus TCP on this port (Venus)")
    r.add_argument("--modbus-rtu-port", type=int, help="serve the RS485 Modbus slave as RTU over TCP on this port")
    r.add_argument("--gdb", metavar="PORT", type=int, help="halt at reset and serve the GDB remote protocol")
    r.add_argument("--no-watchdog", action="store_true", help="IWDG expiry does not reset (handy when debugging)")
    r.add_argument("--realtime", action="store_true", help="never run ahead of wall-clock time")
    r.add_argument("--interactive", "-i", action="store_true", help="read commands from stdin (type 'help')")
    r.add_argument("--inject", action="append", default=[], metavar="T:TOPIC=PAYLOAD",
                   help="at virtual second T deliver an MQTT message, e.g. '30:+/App/+/ctrl=cd=1' "
                        "(+ in the topic is replaced by the firmware's subscription)")
    r.add_argument("--at", action="store_true", help="print the AT command transcript")
    r.add_argument("--uart", action="append", default=[], help="print only these UART consoles (default: all)")
    r.add_argument("--quiet-uart", action="store_true", help="don't print UART console output")
    r.add_argument("--trace-mmio", metavar="FILE", help="write every peripheral access to FILE")
    r.add_argument("--trace-irq", action="store_true", help="log exception entry/exit")
    r.add_argument("--coverage", metavar="FILE", help="write executed basic blocks to FILE at exit")
    r.add_argument("--json-log", metavar="FILE", help="write device events (MQTT, BLE, HTTP, AT) as JSON lines")
    r.add_argument("--gpio", action="append", default=[], metavar="PORTPIN=LEVEL", help="force a GPIO input, e.g. B14=0")
    r.add_argument("--adc", action="append", default=[], metavar="ADCn:CH=VALUE", help="ADC reading, e.g. ADC1:3=1800")
    r.add_argument("--spi-flash", action="append", default=[], metavar="SPIn=FILE",
                   help="attach a SPI NOR flash (W25Q32) backed by FILE, e.g. SPI2=extflash.bin")
    r.add_argument("-v", "--verbose", action="count", default=0)

    from .plant import venus3_cli
    venus3_cli.add_parser(sub)

    i = sub.add_parser("info", help="identify a firmware image")
    i.add_argument("firmware", nargs="+")
    sub.add_parser("profiles", help="list board profiles")
    d = sub.add_parser("decode-bus", help="print B2500 extension-bus frames from a TCP port (marsemu --serve-uart, ser2net)")
    d.add_argument("address", metavar="HOST:PORT")
    d.add_argument("--raw", action="store_true", help="include fields of unknown meaning, by offset")
    d.add_argument("--polls", action="store_true", help="also print the polls of the extension packs")
    return p


def cmd_decode_bus(args) -> int:
    import socket
    from .b2500bus import FrameParser, decode
    host, _, port = args.address.rpartition(":")
    parser = FrameParser()
    with socket.create_connection((host or "127.0.0.1", int(port))) as s:
        while True:
            data = s.recv(1024)
            if not data:
                return 0
            for f in parser.feed(data):
                d = decode(f)
                if d["type"] == "poll" and not args.polls:
                    continue
                if not args.raw:
                    d.pop("raw", None)
                print(json.dumps(d), flush=True)


# --------------------------------------------------------------------------- helpers
class EventPrinter:
    def __init__(self, emu: Emulator, args):
        self.emu = emu
        self.args = args
        self.json = open(args.json_log, "a") if args.json_log else None

    def ts(self) -> str:
        return f"[{self.emu.time:9.3f}]"

    def out(self, kind: str, text: str, **extra) -> None:
        if kind.startswith("AT"):
            text = redact_wifi(text)
        print(f"{self.ts()} {kind:<8} {text}", flush=True)
        if self.json:
            self.json.write(json.dumps({"t": round(self.emu.time, 4), "kind": kind, "text": text, **extra}) + "\n")
            self.json.flush()

    def attach(self) -> None:
        e = self.emu
        a = self.args
        e.on("mqtt_publish", lambda t, p: self.out("MQTT>", f"{t} {p.decode('latin1')}", topic=t))
        e.on("mqtt_subscribe", lambda t: self.out("MQTT-SUB", t))
        e.on("mqtt_connect", lambda s: self.out("MQTT", f"connected #{s.idx} {s.host}:{s.port} id={s.client_id}"))
        e.on("ble_notify", lambda u, d: self.out("BLE>", f"{u} {d.hex()}", data=d.hex()))
        e.on("ble_name", lambda n: self.out("BLE", f"name {n}"))
        e.on("http", lambda m, u, b: self.out("HTTP", f"{m} {u}" + (f" body={b[:200]!r}" if b else "")))
        e.on("ota", lambda u: self.out("OTA", f"module update requested from {u}"))
        e.on("wifi_config", lambda s, pw: self.out("WIFI", f"credentials set ssid={s!r}"))
        e.on("tcp_send", lambda sid, d, extra: self.out("SOCK>", f"#{sid} {d.hex() if any(b < 32 and b not in (10, 13) for b in d) else d.decode('latin1')} {extra or ''}"))
        if a.at:
            e.on("at", lambda l: self.out("AT", l))
            # answers of a real module (--module-serial), with how long after
            # the MCU's last line they arrived (wall clock)
            e.on("at_rx", lambda l, ms=None: self.out("AT<", l if ms is None else f"{l}  (+{ms:.0f} ms)"))
        if not a.quiet_uart:
            def con(ch, line):
                if ch in ("USART1", "USART2", "USART3", "UART4", "UART5", "UART6", "UART7", "itm"):
                    if a.uart and ch not in a.uart:
                        return
                    self.out(ch, line)
            e.console_listeners.append(con)


class BleBridge:
    """TCP server: a connected client is a BLE central.  Bytes it sends are
    GATT writes, notifications are sent back raw."""

    def __init__(self, emu: Emulator, port: int):
        self.emu = emu
        self.client = None
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        emu.on("ble_notify", self._notify)
        threading.Thread(target=self._accept, daemon=True).start()
        log.warning("BLE bridge listening on 127.0.0.1:%d", port)

    def _notify(self, uuid, data):
        if self.client:
            try:
                self.client.sendall(data)
            except OSError:
                pass

    def _accept(self):
        mod = self.emu.fc41d
        while True:
            c, addr = self.srv.accept()
            self.client = c
            mod.post(mod.ble_connect)
            while True:
                try:
                    d = c.recv(512)
                except OSError:
                    d = b""
                if not d:
                    break
                mod.post(lambda d=d: mod.ble_write(d))
            self.client = None
            mod.post(mod.ble_disconnect)


HELP = """commands:
  mqtt <topic> <payload>   deliver an MQTT message (topic '+' = first subscription)
  ble <hex>                BLE GATT write (connects first)
  ble-connect | ble-disconnect
  wifi-down | wifi-up      drop / restore the access point
  gpio <port><pin> <0|1>   drive a GPIO input, e.g. gpio B14 0
  adc <ADCn> <ch> <value>  set an ADC reading
  at                       print the last 40 lines of the AT transcript
  status                   time, PC, resets, MQTT state
  quit"""


def interactive(emu: Emulator, stop_evt: threading.Event):
    mod = emu.fc41d
    subs = []
    emu.on("mqtt_subscribe", subs.append)
    for line in sys.stdin:
        parts = line.strip().split(None, 2)
        if not parts:
            continue
        cmd = parts[0]
        try:
            if cmd == "help":
                print(HELP)
            elif cmd == "quit":
                stop_evt.set()
                mod.post(lambda: emu.machine.stop("quit"))
                return
            elif cmd == "mqtt":
                topic = parts[1]
                if topic == "+":
                    if not subs:
                        print("the firmware has not subscribed to anything yet")
                        continue
                    topic = subs[0]
                payload = parts[2].encode() if len(parts) > 2 else b""
                mod.post(lambda: mod.mqtt_deliver(topic, payload))
            elif cmd == "ble":
                data = bytes.fromhex("".join(parts[1:]))
                mod.post(lambda: mod.ble_write(data))
            elif cmd == "ble-connect":
                mod.post(mod.ble_connect)
            elif cmd == "ble-disconnect":
                mod.post(mod.ble_disconnect)
            elif cmd == "wifi-down":
                mod.post(mod.wifi_disconnect)
            elif cmd == "wifi-up":
                mod.post(mod._wifi_connected)
            elif cmd == "gpio":
                port, pin = parts[1][0].upper(), int(parts[1][1:])
                lvl = int(parts[2])
                mod.post(lambda: emu.machine.bus.get("GPIO" + port).set_input(pin, lvl))
            elif cmd == "adc":
                a, ch, v = parts[1], int(parts[2].split()[0]), int(parts[2].split()[1])
                emu.machine.bus.get(a.upper()).values[ch] = v
            elif cmd == "at":
                print(mod.dump_transcript(40))
            elif cmd == "status":
                m = emu.machine
                print(f"t={emu.time:.3f}s pc=0x{m.pc:08x} resets={m.reset_count} fault={m.fault} "
                      f"wifi={mod.got_ip} ble={mod.ble_connected} mqtt="
                      f"{[(s.idx, s.host, s.connected, s.subs) for s in mod.mqtt.values()]}")
            else:
                print("unknown command; type 'help'")
        except (IndexError, ValueError) as e:
            print(f"bad command: {e}")


DEFAULT_WIFI_MAC = "c8:47:8c:00:00:01"
DEFAULT_BLE_MAC = "c8:47:8c:00:00:02"

VENDOR_CLOUD_TERMS = """\
--mqtt cloud and --http real connect the emulated device to the manufacturer's
servers, where it presents the identity given with --uid, --wifi-mac and
--ble-mac. Use them only with the identity of a device you own, registered to
your own account, and within the manufacturer's terms; posing as another
device or account is unauthorised access. --confirm-own-device confirms that."""


def check_vendor_cloud(args) -> bool:
    """The backends that reach the vendor's servers need an explicit identity
    (the defaults are the same for every marsemu user) and the user's
    confirmation that it is their own device."""
    if args.mqtt != "cloud" and args.http != "real":
        return True
    missing = [f for f, v in (("--uid", args.uid), ("--wifi-mac", args.wifi_mac),
                              ("--ble-mac", args.ble_mac), ("--confirm-own-device", args.confirm_own_device))
               if not v]
    if missing:
        print(f"marsemu: error: contacting the vendor's servers needs {', '.join(missing)}.\n\n"
              f"{VENDOR_CLOUD_TERMS}", file=sys.stderr)
        return False
    print(f"marsemu: connecting to the vendor's servers as your own device: UID {args.uid}, "
          f"Wi-Fi MAC {args.wifi_mac}, BLE MAC {args.ble_mac}", file=sys.stderr)
    return True


def cmd_run(args) -> int:
    if not check_vendor_cloud(args):
        return 2
    args.wifi_mac = args.wifi_mac or DEFAULT_WIFI_MAC
    args.ble_mac = args.ble_mac or DEFAULT_BLE_MAC
    level = logging.WARNING - 10 * min(args.verbose, 2)
    logging.basicConfig(level=level, format="%(levelname)-7s %(name)s: %(message)s")
    if args.verbose < 2:
        logging.getLogger("emu.bus").setLevel(logging.WARNING)
    net = Network(mqtt=args.mqtt, broker=_hostport(args.broker, 1883),
                  broker_auth=tuple(args.broker_auth.split(":", 1)) if args.broker_auth else None,
                  http=args.http, sockets=args.sockets, listen_offset=args.listen_offset)
    for rule in args.http_rule:
        pat, _, resp = rule.partition("=")
        if resp.startswith("@"):
            status, body = 200, open(resp[1:], "rb").read()
        else:
            st, _, body = resp.partition(":")
            status, body = int(st or 200), body.encode()
        net.add_http_rule(pat, status, body)
    cfg = Fc41dConfig(version=args.fc41d_version, ble_mac=args.ble_mac, wifi_mac=args.wifi_mac, ssid=args.ssid,
                      wifi_provisioned=not args.no_wifi)
    emu = Emulator(args.firmware, profile=args.profile, network=net, fc41d=cfg, state_dir=args.state_dir,
                   uid=bytes.fromhex(args.uid) if args.uid else None, watchdog=not args.no_watchdog,
                   fc41d_uart=args.fc41d_uart, device_type=args.device_type,
                   external_module=args.module_serial)
    m = emu.machine
    print(f"firmware {args.firmware}: profile {emu.profile.name} ({emu.profile.description}), "
          f"linked at 0x{emu.base:08x}", flush=True)
    for g in args.gpio:
        k, _, v = g.partition("=")
        m.bus.get("GPIO" + k[0].upper()).inputs[int(k[1:])] = int(v)
    for a in args.adc:
        k, _, v = a.partition("=")
        adc, _, ch = k.partition(":")
        m.bus.get(adc.upper()).values[int(ch)] = int(v)
    flashes = []
    for spec in args.spi_flash:
        bus, _, path = spec.partition("=")
        from .periph.n32g45x import SpiNorFlash
        fl = SpiNorFlash(m, path=path)
        m.bus.get(bus.upper()).device = fl
        flashes.append(fl)
    printer = EventPrinter(emu, args)
    printer.attach()
    if args.trace_irq:
        m.exc_trace = True
        logging.getLogger("emu.cpu").setLevel(logging.DEBUG)
    if args.trace_mmio:
        from .analysis.trace import MmioTracer
        MmioTracer(emu, args.trace_mmio)
    cov = None
    if args.coverage:
        from .analysis.trace import Coverage
        cov = Coverage(emu)
    if args.ble_port:
        BleBridge(emu, args.ble_port)
    for spec in args.serve_uart:
        from .uartbridge import UartBridge
        name, _, port = spec.rpartition(":")
        UartBridge(emu, name, int(port))
    if args.modbus_port or args.modbus_rtu_port:
        from .modbus import ModbusGateway
        if args.modbus_port:
            ModbusGateway(emu, args.modbus_port, "tcp")
        if args.modbus_rtu_port:
            ModbusGateway(emu, args.modbus_rtu_port, "rtu")
    subs = []
    emu.on("mqtt_subscribe", subs.append)
    pending = []
    for spec in args.inject:
        t, _, rest = spec.partition(":")
        topic, _, payload = rest.partition("=")
        pending.append((float(t), topic, payload.encode()))
    pending.sort()

    def injector():
        while pending and emu.time >= pending[0][0]:
            _, topic, payload = pending.pop(0)
            if "+" in topic and subs:
                from .fc41d.module import topic_matches
                cands = [s for s in subs if topic_matches(topic, s)] or subs
                topic = cands[0]
            printer.out("MQTT<", f"{topic} {payload.decode('latin1')}")
            emu.mqtt_inject(topic, payload)
    m.slice_hooks.append(injector)
    if args.realtime:
        base = [time.time() - emu.time]

        def throttle():
            ahead = emu.time - (time.time() - base[0])
            if ahead > 0.005:
                time.sleep(ahead)
            elif ahead < -REALTIME_SLACK_S:
                # Behind (a slow phase or a stall): carry on from here rather
                # than run flat out to catch up, which would make everything
                # outside (a real module, a broker) look slow to the firmware.
                base[0] = time.time() - emu.time
        m.slice_hooks.append(throttle)
    stop_evt = threading.Event()
    if args.interactive:
        threading.Thread(target=interactive, args=(emu, stop_evt), daemon=True).start()
    if args.gdb:
        from .analysis.gdbstub import GdbServer
        GdbServer(emu, args.gdb).serve()  # returns when gdb detaches
    t0 = time.time()
    try:
        reason = emu.run(args.seconds)
    except KeyboardInterrupt:
        reason = "interrupted"
        emu.save_state()
    wall = time.time() - t0
    print(f"stopped after {emu.time:.3f}s virtual ({wall:.1f}s wall): {reason}; resets={m.reset_count}"
          + (f"; last fault: {m.fault}" if m.fault else ""), flush=True)
    if cov:
        cov.save(args.coverage)
    for fl in flashes:
        fl.save()
    return 0


def cmd_info(args) -> int:
    for f in args.firmware:
        img = open(f, "rb").read()
        prof = detect(img, f)
        sp, rst = struct.unpack_from("<II", img, 0)
        base = load_base(img)
        info = {
            "file": f, "size": len(img), "profile": prof.name, "linked_at": hex(base),
            "initial_sp": hex(sp), "reset": hex(rst), "fc41d": b"AT+Q" in img,
        }
        tables = find_type_tables(img, base) if prof.name.startswith("b2500") else []
        if tables:
            # B2500: the build accepts every type in its table; a unit's type is factory data
            info["device_types_supported"] = max((t[2] for t in tables), key=len)
            info["device_type_default"] = default_device_type(img, base, prof)
        else:
            names = sorted(set(re.findall(rb"(?:HM[A-Z]-\d+|VNS[A-Z0-9]+-\d|JPLS-\w+)", img)))
            info["device_types"] = [n.decode() for n in names]
        info["ble_name"] = sorted(set(m.decode() for m in re.findall(rb"AT\+QBLENAME=([^\r\n\x00]{1,24})", img)))
        info["mqtt_prefixes"] = sorted(set(m.decode() for m in re.findall(rb"(hame_energy|marstek_energy)/", img)))
        print(json.dumps(info, indent=1))
    return 0


def main(argv=None) -> int:
    parser = build_parser()
    if not (sys.argv[1:] if argv is None else argv):
        parser.print_help()
        return 0
    args = parser.parse_args(argv)
    try:
        return _dispatch(args)
    except (FileNotFoundError, IsADirectoryError, PermissionError) as e:
        if not e.filename:
            raise
        parser.exit(2, f"marsemu: error: {e.strerror}: {e.filename}\n")


def _dispatch(args) -> int:
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "info":
        return cmd_info(args)
    if args.cmd in ("venus3", "venusd", "venusa", "jupiter", "b2500"):
        from .plant.venus3_cli import cmd_venus3
        return cmd_venus3(args)
    if args.cmd == "decode-bus":
        try:
            return cmd_decode_bus(args)
        except KeyboardInterrupt:
            return 0
    if args.cmd == "profiles":
        for n, p in sorted(PROFILES.items()):
            print(f"{n:<10} {p.description}")
        return 0
    return 1
