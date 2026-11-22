"""Emulated Quectel FC41D Wi-Fi/BLE module (Hame "HM" AT firmware flavour).

The MCU firmware talks to the FC41D over a UART with Quectel AT commands.
This class sits on that UART and answers like the module would, while the
"outside world" (Wi-Fi, BLE central, MQTT broker, HTTP servers, sockets) is
provided by :class:`marsemu.fc41d.net.Network`.

Everything the firmware does through the module is recorded in
:attr:`Fc41d.log` and reported through listener callbacks so scripts and the
CLI can observe and drive the device.
"""
from __future__ import annotations

import binascii
import logging
import queue
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from ..periph.n32g45x import UartDevice
from .net import Network

log = logging.getLogger("fc41d")
atlog = logging.getLogger("fc41d.at")


def topic_matches(pattern: str, topic: str) -> bool:
    p = pattern.split("/")
    t = topic.split("/")
    for i, part in enumerate(p):
        if part == "#":
            return True
        if i >= len(t):
            return False
        if part != "+" and part != t[i]:
            return False
    return len(p) == len(t)


def cert_checksum(data: bytes) -> int:
    """16-bit checksum the Hame FC41D firmware reports for stored certificates.

    The exact algorithm is not known yet (CRC-16/CCITT-FALSE is used); with a
    wrong value firmwares simply delete and re-upload their certificate."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def split_args(s: str) -> List[str]:
    """Split an AT argument list on commas, honouring double quotes."""
    out, cur, q = [], [], False
    for ch in s:
        if ch == '"':
            q = not q
            cur.append(ch)
        elif ch == "," and not q:
            out.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
    out.append("".join(cur))
    return [a.strip() for a in out]


def unq(s: str) -> str:
    s = s.strip()
    return s[1:-1] if len(s) >= 2 and s[0] == '"' and s[-1] == '"' else s


#: module builds that answer AT+QSSLCERT? without a checksum
OLD_CERT_FORMAT_BUILDS = ("202406141323", "202409090159")
#: virtual seconds between a central connecting and its first write reaching
#: the firmware: a phone discovers the services and enables notifications
#: first, which takes about this long. Firmware drops a write that comes too
#: soon after the connection event: the Venus E 3.0 one within 0.45 s.
BLE_SETTLE_S = 1.0


@dataclass
class Fc41dConfig:
    version: str = "FC41DAAR18A04M02_HM.bin_202409090159"
    wifi_mac: str = "c8:47:8c:00:00:01"
    ble_mac: str = "c8:47:8c:00:00:02"
    ssid: str = "EmuNet"
    password: str = "emulator"
    rssi: int = -48
    ip: str = "192.168.1.50"
    gateway: str = "192.168.1.1"
    netmask: str = "255.255.255.0"
    dns: str = "192.168.1.1"
    #: module has stored Wi-Fi credentials and auto-joins after boot
    wifi_provisioned: bool = True
    #: Wi-Fi association delay after boot / AT+QSTAAPINFO
    wifi_connect_ms: int = 1500
    #: auto-join only once the MCU has not sent a command for this long:
    #: several firmwares drop URCs that arrive while they wait for a response
    wifi_idle_ms: int = 2500
    #: ...but never later than this after power-on
    wifi_max_ms: int = 20000
    #: if the firmware does nothing with the network this long after GOT_IP
    #: (it probably missed the URC), announce the connection again the way
    #: a module does after a reconnect (at most wifi_reannounce_max times)
    wifi_reannounce_ms: int = 8000
    wifi_reannounce_max: int = 3
    echo: bool = False
    #: BSSID reported by AT+QGETWIFISTATE (devices report it as wifi_mac)
    bssid: str = "02:00:00:00:00:01"
    #: how long the module takes to answer a command
    reply_ms: float = 3.0
    #: shortest gap between two +QMTRECV URCs: messages that arrive at the
    #: same instant (two injects in one go) reach the MCU one after the
    #: other, as they would through a broker; back to back on the UART the
    #: firmwares keep only the first
    mqtt_recv_gap_ms: float = 200.0
    #: a URC waits until the reply to a command has left the UART, plus this
    #: gap, as on a real module (URCs are held while a command executes);
    #: glued to a reply, the firmwares parse the reply and drop the URC
    urc_gap_ms: float = 20.0
    #: delay of the +QHTTPGET/+QHTTPPOST result after OK (a network round trip)
    http_ms: float = 300.0
    #: "ready" banner after power-on / AT+QRST
    boot_banner: str = "ready"
    boot_ms: int = 300
    #: answer AT+QSSLCERT? with "<kind>",<exists>,<checksum>; None: by module
    #: version (the CT00x firmwares expect no checksum from module builds
    #: 202406141323 and 202409090159 and one from later builds)
    cert_checksum: Optional[bool] = None
    #: certificates the module claims to hold from the factory (kind ->
    #: checksum shown by AT+QSSLCERT?), for firmwares that only verify them
    preloaded_certs: Dict[str, str] = field(default_factory=dict)
    #: how BLE writes from the phone are delivered: "raw" (Hame transparent
    #: mode, AT+QBLETRANMODE) or "urc" (+QBLEGATTSWRITE style)
    ble_rx_mode: str = "raw"


@dataclass
class MqttSession:
    idx: int
    host: str = ""
    port: int = 0
    open: bool = False
    connected: bool = False
    client_id: str = ""
    user: str = ""
    password: str = ""
    subs: List[str] = field(default_factory=list)
    #: QoS each topic filter in ``subs`` was subscribed with
    sub_qos: Dict[str, int] = field(default_factory=dict)
    cfg: Dict[str, List[str]] = field(default_factory=dict)
    conn: object = None
    #: the broker connection being made or in use (net.mqtt_connect)
    client: object = None
    msgid: int = 0


@dataclass
class Socket:
    sid: int
    kind: str
    host: str
    port: int
    local_port: int
    mode: int
    conn: object = None
    rxbuf: bytearray = field(default_factory=bytearray)
    #: a UDP service socket in buffer access mode keeps whole datagrams with their sender
    datagrams: list = field(default_factory=list)
    rx_total: int = 0
    tx_total: int = 0


class Fc41d(UartDevice):
    """AT command engine of the FC41D."""

    PROMPT_CMDS = ("AT+QBLEGATTSNTFY=", "AT+QISEND=", "AT+QMTPUB=", "AT+QSSLSEND=")

    def __init__(self, machine, network: Optional[Network] = None, config: Optional[Fc41dConfig] = None):
        self.m = machine
        self.cfg = config or Fc41dConfig()
        self.net = network or Network()
        self.net.attach(self)
        self.uart = None
        self.buf = bytearray()
        self._skip_lf = False
        self.raw_wait: Optional[Tuple[int, Callable[[bytes], None]]] = None
        self.events: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self.listeners: Dict[str, List[Callable]] = {}
        self.transcript: List[Tuple[float, str, bytes]] = []
        self.keep_transcript = True
        self.echo = self.cfg.echo
        self.reset_state()
        machine.slice_hooks.append(self._drain_events)

    # ------------------------------------------------------------------ state
    def later(self, ms: float, fn: Callable[[], None], name: str = "fc41d") -> None:
        """Schedule ``fn`` unless the module is reset (AT+QRST) in between."""
        epoch = self.epoch

        def run():
            if epoch == self.epoch:
                fn()
        self.m.clock.after_ms(ms, run, name)

    def reset_state(self) -> None:
        self.epoch = getattr(self, "epoch", 0) + 1
        self.wifi_up = False
        self.got_ip = False
        self.ble_init = 0
        self.ble_name = "Quec_BLE"
        self.ble_adv = False
        self.ble_connected = False
        self.ble_connect_t = 0.0
        self._ble_held: List[Tuple[bytes, str]] = []
        self.ble_services: List[str] = []
        self.ble_chars: List[str] = []
        self.ble_tran_uuid: Optional[str] = None
        self.mqtt: Dict[int, MqttSession] = {}
        self.sockets: Dict[int, Socket] = {}
        self.http_cfg: Dict[str, object] = {"headers": {}, "url": "", "output": 0}
        self.http_resp: Optional[bytes] = None
        self.ssl_cfg: Dict[str, List[str]] = {}
        self.certs: Dict[str, bytes] = getattr(self, "certs", {})  # persisted by the module (flash)
        self.urc_cfg: List[str] = []
        self.booted = False
        self.last_cmd_t = 0.0
        self.boot_t = self.m.clock.seconds
        self.wifi_asked = False
        self.net_activity_t = 0.0
        self.ip_t = 0.0
        self.reannounced = 0

    # ---------------------------------------------------------------- events
    def on(self, event: str, fn: Callable) -> None:
        """Subscribe to module events.

        Events: ``at`` (line), ``mqtt_publish`` (topic, payload), ``mqtt_subscribe``
        (topic), ``mqtt_connect`` (session), ``ble_notify`` (uuid, data),
        ``ble_name`` (name), ``http`` (method, url, body), ``tcp_send`` (sid, data),
        ``ota`` (url), ``wifi_config`` (args)."""
        self.listeners.setdefault(event, []).append(fn)

    def emit(self, event: str, *args) -> None:
        for fn in self.listeners.get(event, []):
            try:
                fn(*args)
            except Exception:  # pragma: no cover - user callbacks
                log.exception("listener for %s failed", event)

    def post(self, fn: Callable[[], None]) -> None:
        """Thread-safe: run ``fn`` on the emulation thread (used by network threads)."""
        self.events.put(fn)
        self.m.wake()

    def _drain_events(self) -> None:
        if self.events.empty():
            return
        while True:
            try:
                fn = self.events.get_nowait()
            except queue.Empty:
                return
            fn()

    # ------------------------------------------------------------- uart side
    def attach(self, uart) -> None:
        self.uart = uart

    def send(self, data: bytes, delay_ms: Optional[float] = None) -> None:
        """Queue bytes from the module to the MCU."""
        if self.uart is None:
            return
        d = self.cfg.reply_ms if delay_ms is None else delay_ms
        now_ms = self.m.clock.seconds * 1000.0
        # when this output has left the UART: a held URC waits for it (send_after_reply)
        self._out_end_ms = (max(getattr(self, "_out_end_ms", 0.0), now_ms + max(d, 0.0))
                            + len(data) * self._byte_ms())
        if self.keep_transcript:
            self.transcript.append((self.m.clock.seconds, "<", bytes(data)))

        epoch = self.epoch

        def go():
            if epoch == self.epoch:  # a module reset discards queued output
                self.uart.inject(data)
        if d <= 0:
            go()
        else:
            self.m.clock.after_ms(d, go, "fc41d-tx")

    def _byte_ms(self) -> float:
        bc = getattr(self.uart, "byte_cycles", None)
        return bc() * 1000.0 / self.m.clock.hz if bc else 10_000 / 115200

    def send_after_reply(self, data: bytes, delay_ms: float) -> None:
        """Send an unsolicited result after ``delay_ms``, but not before the
        output already queued (a command's reply) has left the UART plus
        ``urc_gap_ms``."""
        epoch = self.epoch

        def go():
            if epoch != self.epoch:
                return
            wait = getattr(self, "_out_end_ms", 0.0) + self.cfg.urc_gap_ms - self.m.clock.seconds * 1000.0
            if wait > 0:
                self.m.clock.after_ms(wait, go, "fc41d-urc")
            else:
                self.send(data, 0)
        self.m.clock.after_ms(max(delay_ms, 0.0), go, "fc41d-urc")

    def line(self, text: str, delay_ms: Optional[float] = None) -> None:
        atlog.debug("<< %s", text)
        self.send(b"\r\n" + text.encode("latin1") + b"\r\n", delay_ms)

    def ok(self, delay_ms: Optional[float] = None) -> None:
        self.line("OK", delay_ms)

    def error(self, delay_ms: Optional[float] = None) -> None:
        self.line("ERROR", delay_ms)

    def urc(self, text: str, delay_ms: float = 0.0) -> None:
        self.line(text, delay_ms if delay_ms else self.cfg.reply_ms)

    def on_tx(self, data: bytes) -> None:
        if self.keep_transcript:
            t = self.m.clock.seconds
            if self.transcript and self.transcript[-1][1] == ">" and t - self.transcript[-1][0] < 0.002 \
                    and not self.transcript[-1][2].endswith(b"\n"):
                self.transcript[-1] = (self.transcript[-1][0], ">", self.transcript[-1][2] + bytes(data))
            else:
                self.transcript.append((t, ">", bytes(data)))
            if len(self.transcript) > 20000:
                del self.transcript[:10000]
        if self.echo and self.raw_wait is None:
            self.send(data, 0.1)
        self.buf.extend(data)
        self._parse()

    def _parse(self) -> None:
        while self.buf:
            if self._skip_lf:
                # the LF of the CR LF that ended the previous command line
                self._skip_lf = False
                if self.buf[:1] == b"\n":
                    del self.buf[:1]
                    continue
            if self.raw_wait is not None:
                n, cb = self.raw_wait
                if len(self.buf) < n:
                    return
                chunk = bytes(self.buf[:n])
                del self.buf[:n]
                self.raw_wait = None
                cb(chunk)
                continue
            # skip stray line terminators / garbage before "AT"
            while self.buf and self.buf[:1] in (b"\r", b"\n", b"\0", b" "):
                del self.buf[:1]
            if not self.buf:
                return
            if self.buf[:2].upper() != b"AT" and len(self.buf) >= 2:
                # not a command: data in transparent mode or noise
                i = self.buf.find(b"AT")
                junk = bytes(self.buf[: i if i >= 0 else len(self.buf)])
                del self.buf[: len(junk)]
                if junk.strip():
                    self.emit("raw", junk)
                    log.debug("ignoring non-AT bytes from MCU: %r", junk[:40])
                continue
            # commands with inline length-prefixed payloads
            head = bytes(self.buf[:24]).upper()
            handled = False
            for pfx in self.PROMPT_CMDS:
                if head.startswith(pfx.encode()):
                    r = self._parse_inline(pfx)
                    if r is None:
                        return  # need more bytes
                    handled = True
                    break
            if handled:
                continue
            end = -1
            for i, b in enumerate(self.buf):
                if b in (13, 10):
                    end = i
                    break
            if end < 0:
                if len(self.buf) > 4096:
                    self.buf.clear()
                return
            line = bytes(self.buf[:end]).decode("latin1")
            self._skip_lf = self.buf[end] == 13
            del self.buf[: end + 1]
            self.command(line)

    def _parse_inline(self, pfx: str) -> Optional[bool]:
        """Parse ``AT+X=a,b,...,<len>,<payload>`` where payload may be binary."""
        buf = bytes(self.buf)
        nfields = {"AT+QBLEGATTSNTFY=": 2, "AT+QISEND=": 2, "AT+QMTPUB=": 6, "AT+QSSLSEND=": 2}[pfx]
        pos = len(pfx)
        fields = []
        inq = False
        start = pos
        while len(fields) < nfields:
            if pos >= len(buf):
                return None
            ch = buf[pos]
            if ch == 34:
                inq = not inq
            elif ch in (13, 10) and not inq:
                # no payload: plain command (e.g. AT+QISEND=0,0 query)
                fields.append(buf[start:pos].decode("latin1"))
                line = buf[:pos].decode("latin1")
                del self.buf[: pos + 1]
                self.command(line)
                return True
            elif ch == 44 and not inq:
                fields.append(buf[start:pos].decode("latin1"))
                start = pos + 1
            pos += 1
        try:
            n = int(fields[-1])
        except ValueError:
            # unexpected syntax: fall back to line mode
            end = buf.find(b"\r")
            if end < 0:
                return None
            del self.buf[: end + 1]
            self.command(buf[:end].decode("latin1"))
            return True
        quoted = pos < len(buf) and buf[pos] == 34
        if pos >= len(buf):
            return None
        if pfx == "AT+QBLEGATTSNTFY=" and not quoted:
            # payload is either <n> raw bytes or 2*<n> hex digits (Hame FW
            # converts hex text to binary before notifying)
            if len(buf) < pos + min(2 * n, n + 1):
                return None
            hexpart = buf[pos:pos + 2 * n]
            if len(hexpart) == 2 * n and all(c in b"0123456789abcdefABCDEF" for c in hexpart) and \
                    (len(buf) == pos + 2 * n or buf[pos + 2 * n] in (13, 10)):
                if len(buf) == pos + 2 * n:
                    return None
                rest = pos + 2 * n
                while rest < len(buf) and buf[rest] in (13, 10):
                    rest += 1
                del self.buf[:rest]
                header = [unq(f) for f in fields]
                self.emit("at", pfx + ",".join(fields) + f",<{n} bytes hex>")
                self.last_cmd_t = self.m.clock.seconds
                self.inline_QBLEGATTSNTFY(header, bytes.fromhex(hexpart.decode()))
                return True
        if quoted:
            # payload is quoted: take <n> bytes after the quote, tolerate mismatch
            if len(buf) < pos + 1 + n + 1:
                return None
            payload = buf[pos + 1: pos + 1 + n]
            rest = pos + 1 + n
            if buf[rest:rest + 1] != b'"':
                q = buf.find(b'"\r', pos + 1)
                if q < 0:
                    return None
                payload = buf[pos + 1:q]
                rest = q
            rest += 1
        else:
            if len(buf) < pos + n:
                return None
            payload = buf[pos:pos + n]
            rest = pos + n
        # trailing arguments after the payload (e.g. ,"ip",port for UDP)
        eol = rest
        while eol < len(buf) and buf[eol] not in (13, 10):
            eol += 1
        if eol >= len(buf):
            return None
        extra = buf[rest:eol].decode("latin1").lstrip(",")
        rest = eol
        while rest < len(buf) and buf[rest] in (13, 10):
            rest += 1
        del self.buf[:rest]
        header = [unq(f) for f in fields] + ([unq(a) for a in split_args(extra)] if extra else [])
        atlog.debug(">> %s<%d bytes>", pfx + ",".join(fields), len(payload))
        self.emit("at", pfx + ",".join(fields) + f",<{len(payload)} bytes>")
        self.last_cmd_t = self.m.clock.seconds
        getattr(self, "inline_" + pfx[3:-1])(header, payload)
        return True

    # ------------------------------------------------------------- commands
    def command(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        atlog.debug(">> %s", line)
        self.emit("at", line)
        self.last_cmd_t = self.m.clock.seconds
        up = line.upper()
        if up == "AT":
            self.ok()
            return
        if up in ("ATE0", "ATE1"):
            self.echo = up == "ATE1"
            self.ok()
            return
        m = re.match(r"AT\+([A-Z0-9_]+)(=\?|\?|=(.*))?$", line, re.I | re.S)
        if not m:
            log.info("unknown AT syntax: %r", line)
            self.error()
            return
        name = m.group(1).upper()
        if m.group(2) == "=?":
            mode, args = "test", []
        elif m.group(2) == "?":
            mode, args = "query", []
        elif m.group(2):
            mode, args = "set", split_args(m.group(3))
        else:
            mode, args = "exec", []
        fn = getattr(self, "at_" + name, None)
        if fn is None:
            log.info("unimplemented AT command %s (%s) -> OK", name, line)
            self.ok()
            return
        try:
            fn(mode, args)
        except (IndexError, ValueError) as e:
            log.warning("bad arguments for %s: %s (%s)", name, line, e)
            self.error()

    # ---- system --------------------------------------------------------
    def power_on(self) -> None:
        """Module powered up (MCU toggled its enable/reset line, or at start)."""
        self.reset_state()
        if self.cfg.boot_banner:
            self.urc(self.cfg.boot_banner, self.cfg.boot_ms)
        self.booted = True
        if self.cfg.wifi_provisioned:
            self.later(self.cfg.boot_ms + self.cfg.wifi_connect_ms, self._wifi_autojoin, "wifi")

    def _wifi_autojoin(self) -> None:
        if self.wifi_up:
            return
        idle = (self.m.clock.seconds - self.last_cmd_t) * 1000
        since_boot = (self.m.clock.seconds - self.boot_t) * 1000
        if (idle < self.cfg.wifi_idle_ms or self.raw_wait is not None) and since_boot < self.cfg.wifi_max_ms \
                and not self.wifi_asked:
            self.later(100, self._wifi_autojoin, "wifi")
            return
        self._wifi_connected()

    def _net_used(self) -> None:
        self.net_activity_t = self.m.clock.seconds

    def _reannounce_check(self) -> None:
        if not self.got_ip or self.reannounced >= self.cfg.wifi_reannounce_max:
            return
        if self.net_activity_t >= self.ip_t:
            return  # firmware is using the network: it saw the URC
        idle = (self.m.clock.seconds - self.last_cmd_t) * 1000
        if idle < self.cfg.wifi_idle_ms or self.raw_wait is not None:
            self.later(500, self._reannounce_check, "wifi-re")
            return
        self.reannounced += 1
        log.info("firmware ignored the Wi-Fi connection; announcing it again (reconnect)")
        self.urc("+QSTASTAT:WLAN_CONNECTED")
        self.later(300, lambda: self.urc("+QSTASTAT:GOT_IP"), "dhcp")
        self.ip_t = self.m.clock.seconds
        self.later(self.cfg.wifi_reannounce_ms, self._reannounce_check, "wifi-re")

    def _wifi_connected(self) -> None:
        if self.wifi_up:
            return
        self.wifi_up = True
        self.urc("+QSTASTAT:WLAN_CONNECTED")
        self.later(300, self._got_ip, "dhcp")
        self.emit("wifi", "connected")

    def _got_ip(self) -> None:
        self.got_ip = True
        self.urc("+QSTASTAT:GOT_IP")
        self.emit("wifi", "got_ip")
        self.ip_t = self.m.clock.seconds
        self.later(self.cfg.wifi_reannounce_ms, self._reannounce_check, "wifi-re")

    def wifi_disconnect(self) -> None:
        """Simulate losing the access point."""
        self.wifi_up = self.got_ip = False
        self.urc("+QSTASTAT:WLAN DISCONNECTED")
        for s in list(self.mqtt.values()):
            if s.open:
                self._mqtt_drop(s, 1)

    def at_QRST(self, mode, args):
        self.ok()
        for s in self.mqtt.values():
            self.net.mqtt_close(s)
        self.m.clock.after_ms(50, self.power_on, "fc41d-reset")

    def at_QVERSION(self, mode, args):
        self.line(f"+QVERSION: {self.cfg.version}")
        self.ok()

    def at_QECHO(self, mode, args):
        self.echo = args and args[0] == "1"
        self.ok()

    def at_QURCCFG(self, mode, args):
        self.urc_cfg = args
        self.ok()

    def at_QWLMAC(self, mode, args):
        self.line(f"+QWLMAC: {self.cfg.wifi_mac}")
        self.ok()

    def at_QGETIP(self, mode, args):
        self.wifi_asked = True
        if self.got_ip:
            c = self.cfg
            # no blank line before it: the Venus C/E keeps the IP only if the first
            # CRLF of the answer comes after "ip:" (else it asks again on every pass
            # of its meter task, and that command's 300 ms read swallows CT replies)
            text = f"+QGETIP:ip:{c.ip},gate:{c.gateway},mask:{c.netmask},dns:{c.dns}"
            atlog.debug("<< %s", text)
            self.send(text.encode() + b"\r\n")
            self.ok()
        else:
            self.error()

    def at_QWLANOTA(self, mode, args):
        url = unq(args[0]) if args else ""
        log.warning("firmware asked the module to OTA-update itself from %s", url)
        self.emit("ota", url)
        self.ok()

    def at_QSETBAUD(self, mode, args):
        self.ok()

    at_QSETBAND = at_QSETBAUD

    def at_QSTASTOP(self, mode, args):
        self.ok()
        if self.wifi_up:
            self.wifi_disconnect()

    def at_QSTAAPINFO(self, mode, args):
        if mode != "set":
            self.line(f'+QSTAAPINFO: "{self.cfg.ssid}"')
            self.ok()
            return
        self.cfg.ssid = unq(args[0])
        if len(args) > 1:
            self.cfg.password = unq(args[1])
        self.emit("wifi_config", self.cfg.ssid, self.cfg.password)
        log.info("firmware configured Wi-Fi: ssid=%r", self.cfg.ssid)
        self.ok()
        self.wifi_up = self.got_ip = False
        self.later(self.cfg.wifi_connect_ms, self._wifi_connected, "wifi")

    at_QSTAAPINFODEF = at_QSTAAPINFO

    def at_QSTAST(self, mode, args):
        self.wifi_asked = True
        self.line("+QSTAST:" + ("STATION_UP" if self.wifi_up else "STATION_DOWN"))
        self.ok()

    def at_QGETWIFISTATE(self, mode, args):
        self.wifi_asked = True
        if self.wifi_up:
            # the Hame module firmware answers in key=value form (not the
            # manual's positional one); the device firmwares look for "ssid="
            # and "rssi=", and keep asking every 80 ms when they don't find them
            self.line(f'+QGETWIFISTATE: ssid={self.cfg.ssid},bssid={self.cfg.bssid},rssi={self.cfg.rssi},channel=6')
        else:
            self.line("+QGETWIFISTATE: 0")
        self.ok()

    def at_QWSCAN(self, mode, args):
        self.line(f'+QWSCAN:"{self.cfg.ssid}",3,{self.cfg.rssi},"aa:bb:cc:dd:ee:ff",6')
        self.ok()

    def at_QSTADHCP(self, mode, args):
        self.ok()

    at_QSTADHCPDEF = at_QSTADHCP

    def at_QSOFTAP(self, mode, args):
        self.ok()

    at_QSOFTAPSTOP = at_QSOFTAP

    def at_QPING(self, mode, args):
        self.ok()
        self.urc("+QPING: 0,4,4,0,1,1,1", 20)

    def at_QNTP(self, mode, args):
        self.ok()

    def at_QTIME(self, mode, args):
        t = time.gmtime()
        self.line(time.strftime('+QTIME: "%Y-%m-%d %H:%M:%S"', t))
        self.ok()

    # ---- BLE -----------------------------------------------------------
    def at_QBLEINIT(self, mode, args):
        if mode == "query":
            self.line(f"+QBLEINIT: {self.ble_init}")
            self.ok()
            return
        self.ble_init = int(args[0]) if args else 2
        self.ok()

    def at_QBLEADDR(self, mode, args):
        self.line(f"+QBLEADDR:{self.cfg.ble_mac}")
        self.ok()

    def at_QBLENAME(self, mode, args):
        if mode == "query":
            self.line(f"+QBLENAME:{self.ble_name}")
            self.ok()
            return
        self.ble_name = unq(",".join(args))
        log.info("BLE name: %s", self.ble_name)
        self.emit("ble_name", self.ble_name)
        self.ok()

    def at_QBLEGATTSSRV(self, mode, args):
        self.ble_services.append(unq(args[0]))
        self.ok()

    def at_QBLEGATTSCHAR(self, mode, args):
        self.ble_chars.append(unq(args[0]))
        self.ok()

    def at_QBLEADVPARAM(self, mode, args):
        self.ok()

    def at_QBLEADVDATA(self, mode, args):
        self.ok()

    def at_QBLEADVSTART(self, mode, args):
        self.ble_adv = True
        self.ok()
        self.emit("ble_adv", True)

    def at_QBLEADVSTOP(self, mode, args):
        self.ble_adv = False
        self.ok()
        self.emit("ble_adv", False)

    def at_QBLETRANMODE(self, mode, args):
        self.ble_tran_uuid = unq(args[0]) if args else None
        self.ok()

    def at_QBLESTAT(self, mode, args):
        if self.ble_connected:
            st = "CONNECTED"
        elif self.ble_adv:
            st = "ADVERTISING"
        elif self.ble_init:
            st = "INIT"
        else:
            st = "NOINIT"
        self.line(f"+QBLESTAT:{st}")
        self.ok()

    def at_QBLEDISCONN(self, mode, args):
        self.ok()
        if self.ble_connected:
            self.ble_disconnect()

    def at_QBLECFGMTU(self, mode, args):
        if mode == "query":
            self.line("+QBLECFGMTU: 247")
        self.ok()

    def at_QBLEGATTSNTFY(self, mode, args):
        # without inline payload (data given as plain argument)
        data = unq(args[-1]).encode("latin1") if args else b""
        self.inline_QBLEGATTSNTFY([unq(a) for a in args[:-1]] + [str(len(data))], data)

    def inline_QBLEGATTSNTFY(self, header, payload: bytes):
        uuid = header[0] if header else "ff02"
        self.emit("ble_notify", uuid, payload)
        self.ok()

    # driven by the outside world
    def ble_connect(self) -> None:
        self.ble_connected = True
        self.ble_connect_t = self.m.clock.seconds
        self.urc("+QBLESTAT:CONNECTED")
        self.emit("ble_state", "connected")

    def ble_disconnect(self) -> None:
        self.ble_connected = False
        self._ble_held = []
        self.urc("+QBLESTAT:DISCONNECTED")
        self.emit("ble_state", "disconnected")

    def ble_write(self, data: bytes, uuid: str = "ff01") -> None:
        """The phone writes ``data`` to the GATT characteristic."""
        if not self.ble_connected:
            self.ble_connect()
        wait = self.ble_connect_t + BLE_SETTLE_S - self.m.clock.seconds
        if wait > 0 or self._ble_held:
            # too soon after connecting: hold it, and what follows, in order
            if not self._ble_held:
                self.later(max(wait, 0.0) * 1000.0, self._ble_release, "ble-settle")
            self._ble_held.append((bytes(data), uuid))
            return
        self._ble_deliver(data, uuid)

    def _ble_release(self) -> None:
        held, self._ble_held = self._ble_held, []
        if self.ble_connected:
            for data, uuid in held:
                self._ble_deliver(data, uuid)

    def _ble_deliver(self, data: bytes, uuid: str) -> None:
        if self.cfg.ble_rx_mode == "urc":
            self.send(f"\r\n+QBLEGATTSWRITE: {uuid},{len(data)},".encode() + data + b"\r\n")
        else:
            self.send(bytes(data))

    # ---- SSL -----------------------------------------------------------
    def cert_checksums(self) -> bool:
        if self.cfg.cert_checksum is not None:
            return self.cfg.cert_checksum
        return not self.cfg.version.endswith(OLD_CERT_FORMAT_BUILDS)

    def at_QSSLCERT(self, mode, args):
        if mode == "query":
            # one block, lines separated by a single CRLF: the CT00x firmwares
            # compare the whole reply with a string
            lines = []
            for kind in ("CA", "User Cert", "User Key"):
                data = self.certs.get(kind)
                claim = self.cfg.preloaded_certs.get(kind)
                if self.cert_checksums():
                    # a 16-bit checksum that firmwares compare with the value of their own certificate
                    ck = claim if data is None and claim else (f"{cert_checksum(data):x}" if data else "0")
                    lines.append(f'+QSSLCERT: "{kind}",{1 if data or claim else 0},{ck}')
                else:
                    lines.append(f'+QSSLCERT: "{kind}",{1 if data or claim else 0}')
            atlog.debug("<< %s", lines)
            self.send(("\r\n" + "\r\n".join(lines) + "\r\n").encode("latin1"))
            self.ok()
            return
        kind = unq(args[0])
        op = int(args[1]) if len(args) > 1 else 1
        if op == 0:
            self.certs.pop(kind, None)
            self.ok()
        elif op == 1:
            data = self.certs.get(kind)
            if data is None:
                self.error()
                return
            self.line("CONNECT")
            self.send(data)
            self.ok()
        elif op == 2:
            n = int(args[2])
            self.line("CONNECT")

            def got(data: bytes):
                self.certs[kind] = data
                log.info("firmware uploaded SSL %s (%d bytes)", kind, len(data))
                self.emit("cert", kind, data)
                self.ok()
            self.raw_wait = (n, got)
        else:
            self.error()

    def at_QSSLCFG(self, mode, args):
        if args:
            self.ssl_cfg[unq(args[0])] = args[1:]
        self.ok()

    # ---- MQTT ----------------------------------------------------------
    def _sess(self, idx) -> MqttSession:
        idx = int(idx)
        if idx not in self.mqtt:
            self.mqtt[idx] = MqttSession(idx)
        return self.mqtt[idx]

    def at_QMTCFG(self, mode, args):
        if mode == "set":
            key = unq(args[0])
            if len(args) >= 2:
                s = self._sess(args[1])
                s.cfg[key] = args[2:]
        self.ok()

    def at_QMTOPEN(self, mode, args):
        self._net_used()
        if mode == "query":
            for s in self.mqtt.values():
                if s.open:
                    self.line(f'+QMTOPEN: {s.idx},"{s.host}",{s.port}')
            self.ok()
            return
        s = self._sess(args[0])
        s.host, s.port = unq(args[1]), int(args[2])
        self.ok()
        if not self.got_ip:
            self.urc(f"+QMTOPEN: {s.idx},-1", 100)
            return
        log.info("MQTT open #%d %s:%d", s.idx, s.host, s.port)
        self.net.mqtt_open(s, lambda rc: self._mqtt_opened(s, rc))

    def _mqtt_opened(self, s: MqttSession, rc: int) -> None:
        s.open = rc == 0
        self.urc(f"+QMTOPEN: {s.idx},{rc}", 30)

    def at_QMTCONN(self, mode, args):
        self._net_used()
        if mode == "query":
            for s in self.mqtt.values():
                if s.open:
                    self.line(f"+QMTCONN: {s.idx},{3 if s.connected else 1}")
            self.ok()
            return
        s = self._sess(args[0])
        s.client_id = unq(args[1])
        s.user = unq(args[2]) if len(args) > 2 else ""
        s.password = unq(args[3]) if len(args) > 3 else ""
        self.ok()
        log.info("MQTT connect #%d client_id=%s user=%s", s.idx, s.client_id, s.user)
        self.net.mqtt_connect(s, lambda rc: self._mqtt_connected(s, rc))

    def _mqtt_connected(self, s: MqttSession, rc: int) -> None:
        s.connected = rc == 0
        if rc == 0:
            self.urc(f"+QMTCONN: {s.idx},0,0", 20)
            self.emit("mqtt_connect", s)
        elif rc < 0:
            self.urc(f"+QMTCONN: {s.idx},2", 20)
        else:
            self.urc(f"+QMTCONN: {s.idx},0,{rc}", 20)

    def at_QMTSUB(self, mode, args):
        s = self._sess(args[0])
        msgid = int(args[1])
        topics = [unq(args[i]) for i in range(2, len(args), 2)]
        qos = [int(args[i]) for i in range(3, len(args), 2)] or [0]
        self.ok()
        for t in topics:
            if t not in s.subs:
                s.subs.append(t)
            s.sub_qos[t] = qos[0] if qos else 0
            log.info("MQTT subscribe #%d %s", s.idx, t)
            self.emit("mqtt_subscribe", t)
            self.net.mqtt_subscribe(s, t, qos[0] if qos else 0)
        self.urc(f"+QMTSUB: {s.idx},{msgid},0,{qos[0] if qos else 0}", 20)

    def at_QMTUNS(self, mode, args):
        s = self._sess(args[0])
        msgid = int(args[1])
        for a in args[2:]:
            t = unq(a)
            if t in s.subs:
                s.subs.remove(t)
            s.sub_qos.pop(t, None)
            self.net.mqtt_unsubscribe(s, t)
        self.ok()
        self.urc(f"+QMTUNS: {s.idx},{msgid},0", 20)

    def at_QMTPUB(self, mode, args):
        # AT+QMTPUB=<id>,<msgid>,<qos>,<retain>,"<topic>" -> '>' prompt, payload, Ctrl-Z
        self.send(b"\r\n>")
        self._ctrlz_wait(lambda payload: self.inline_QMTPUB(args[:5] + [str(len(payload))], payload))

    def _ctrlz_wait(self, cb) -> None:
        # wait for Ctrl-Z terminated data
        def check(_=None):
            i = self.buf.find(b"\x1a")
            if i < 0:
                self.m.clock.after_ms(1, check)
                return
            data = bytes(self.buf[:i])
            del self.buf[: i + 1]
            cb(data)
        check()

    def inline_QMTPUB(self, header, payload: bytes):
        s = self._sess(header[0])
        msgid, qos, retain = int(header[1]), int(header[2]), int(header[3])
        topic = unq(header[4])
        self.ok()
        log.info("MQTT publish #%d %s (%d bytes)", s.idx, topic, len(payload))
        self.emit("mqtt_publish", topic, payload)
        if s.connected:
            self.net.mqtt_publish(s, topic, payload, qos, retain)
            self.urc(f"+QMTPUB: {s.idx},{msgid},0", 10)
            if self.net.mqtt_mode == "offline" and any(topic_matches(p, topic) for p in s.subs):
                # an MQTT 3.1.1 broker delivers a message to every matching
                # subscription, the publisher's own included; Venus firmware
                # publishes to its own ctrl topic and treats the echo as
                # proof that the connection works
                self.later(60, lambda: s.connected and self.mqtt_deliver(topic, payload, s.idx))
        else:
            self.urc(f"+QMTPUB: {s.idx},{msgid},2", 10)

    def at_QMTDISC(self, mode, args):
        s = self._sess(args[0])
        self.ok()
        self.net.mqtt_close(s)
        s.connected = False
        self.urc(f"+QMTDISC: {s.idx},0", 10)

    def at_QMTCLOSE(self, mode, args):
        s = self._sess(args[0])
        self.ok()
        self.net.mqtt_close(s)
        s.connected = s.open = False
        self.urc(f"+QMTCLOSE: {s.idx},0", 10)

    def at_QMTRECV(self, mode, args):
        self.ok()

    def mqtt_deliver(self, topic: str, payload: bytes, idx: Optional[int] = None) -> bool:
        """Deliver a message from the broker to the MCU (+QMTRECV URC), at
        least ``mqtt_recv_gap_ms`` after the previous one."""
        delivered = False
        now_ms = self.m.clock.seconds * 1000.0
        at_ms = max(now_ms, getattr(self, "_mqtt_recv_next_ms", 0.0))
        for s in self.mqtt.values():
            if idx is not None and s.idx != idx:
                continue
            if not s.connected:
                continue
            matches = [p for p in s.subs if topic_matches(p, topic)]
            if matches:
                # A QoS-0 message has no packet identifier: the FC41D reports
                # msgid 0, and the CT00x firmwares accept nothing else.
                if max(s.sub_qos.get(p, 0) for p in matches) > 0:
                    s.msgid = s.msgid % 65535 + 1
                    msgid = s.msgid
                else:
                    msgid = 0
                txt = payload.decode("latin1")
                self.send_after_reply(f'\r\n+QMTRECV: {s.idx},{msgid},"{topic}",{len(payload)},"{txt}"\r\n'.encode("latin1"),
                                      at_ms - now_ms + self.cfg.reply_ms)
                at_ms += self.cfg.mqtt_recv_gap_ms
                delivered = True
        if delivered:
            self._mqtt_recv_next_ms = at_ms
        if not delivered:
            log.info("MQTT message for %s dropped (not subscribed/connected)", topic)
        return delivered

    def _mqtt_drop(self, s: MqttSession, reason: int) -> None:
        s.connected = s.open = False
        self.urc(f"+QMTSTAT: {s.idx},{reason}")

    # ---- HTTP ----------------------------------------------------------
    def at_QHTTPCFG(self, mode, args):
        if mode == "set":
            key = unq(args[0])
            if key == "url":
                self.http_cfg["url"] = unq(",".join(args[1:])) if len(args) > 1 else self.http_cfg["url"]
            elif key == "header":
                if len(args) > 2:
                    self.http_cfg["headers"][unq(args[1])] = unq(",".join(args[2:]))
                elif len(args) == 2:
                    self.http_cfg["headers"].pop(unq(args[1]), None)
            elif key == "response/output" and len(args) > 1:
                self.http_cfg["output"] = int(args[1])
            else:
                self.http_cfg[key] = args[1:]
        self.ok()

    def at_QHTTPGET(self, mode, args):
        self._net_used()
        url = self.http_cfg["url"]
        self.ok()
        self._http("GET", url, b"")

    def at_QHTTPPOST(self, mode, args):
        self._net_used()
        n = int(args[0])
        url = self.http_cfg["url"]
        self.line("CONNECT")

        def got(body: bytes):
            self.ok()
            self._http("POST", url, body)
        self.raw_wait = (n, got)

    def _http(self, method: str, url: str, body: bytes) -> None:
        log.info("HTTP %s %s (%d bytes body)", method, url, len(body))
        self.emit("http", method, url, body)
        if not self.got_ip:
            self.urc(f"+QHTTP{method}: 1", 50)
            return

        def done(status: Optional[int], data: bytes):
            if status is None:
                self.urc(f"+QHTTP{method}: 4", 10)
                return
            self.http_resp = data
            if self.http_cfg.get("output") == 1:
                self.urc(f"+QHTTP{method}: 0,{status},{len(data)}", self.cfg.http_ms)
                self.send(f'\r\n+QHTTPURC: "recv",{len(data)}\r\n'.encode() + data + b"\r\n")
            else:
                self.urc(f"+QHTTP{method}: 0,{status},{len(data)}", self.cfg.http_ms)
        self.net.http(method, url, dict(self.http_cfg["headers"]), body, lambda st, d: self.post(lambda: done(st, d)))

    def at_QHTTPREAD(self, mode, args):
        data = self.http_resp or b""
        self.line("CONNECT")
        self.send(data)
        self.ok()
        self.urc("+QHTTPREAD: 0", 5)

    # ---- TCP/UDP sockets ----------------------------------------------
    def at_QICFG(self, mode, args):
        self.ok()

    def at_QIOPEN(self, mode, args):
        self._net_used()
        sid = int(args[0])
        kind = unq(args[1]).upper()
        host = unq(args[2]) if len(args) > 2 else ""
        port = int(args[3]) if len(args) > 3 and args[3] else 0
        lport = int(args[4]) if len(args) > 4 and args[4] else 0
        amode = int(args[5]) if len(args) > 5 and args[5] else 0
        sock = Socket(sid, kind, host, port, lport, amode)
        self.sockets[sid] = sock
        self.ok()
        if not self.got_ip:
            self.urc(f"+QIOPEN: {sid},553", 50)
            return
        log.info("socket %d open %s %s:%d (local %d, mode %d)", sid, kind, host, port, lport, amode)
        self.emit("tcp_open", sock)
        self.net.socket_open(sock, lambda err: self.urc(f"+QIOPEN: {sid},{err}", 20))

    def at_QISTATE(self, mode, args):
        for s in self.sockets.values():
            self.line(f'+QISTATE: {s.sid},"{s.kind}","{s.host}",{s.port},{s.local_port},'
                      f'{2 if s.conn is not None or s.kind != "TCP" else 0},{s.mode}')
        self.ok()

    def at_QISEND(self, mode, args):
        # AT+QISEND=<id>,0 query, or AT+QISEND=<id>,<len> followed by '>' prompt
        sid = int(args[0])
        s = self.sockets.get(sid)
        if len(args) >= 2 and args[1] == "0":
            tx = s.tx_total if s else 0
            self.line(f"+QISEND: {tx},{tx},0")
            self.ok()
            return
        n = int(args[1])
        self.send(b"\r\n> ")
        self.raw_wait = (n, lambda data: self.inline_QISEND([str(sid), str(n)] + args[2:], data))

    def inline_QISEND(self, header, payload: bytes):
        sid = int(header[0])
        s = self.sockets.get(sid)
        if s is None:
            self.error()
            return
        s.tx_total += len(payload)
        self.emit("tcp_send", sid, payload, header[2:])
        log.info("socket %d send %d bytes%s", sid, len(payload), f" to {header[2]}:{header[3]}" if len(header) > 3 else "")
        self.net.socket_send(s, payload, header[2:] if len(header) > 2 else None)
        self.line(f"+QISEND: {len(payload)}")
        self.ok()

    def at_QIRD(self, mode, args):
        sid = int(args[0])
        n = int(args[1]) if len(args) > 1 else 1500
        s = self.sockets.get(sid)
        if s is None:
            self.error()
            return
        if n == 0:
            self.line(f"+QIRD: {s.rx_total},{s.rx_total - len(s.rxbuf)},{len(s.rxbuf)}")
            self.ok()
            return
        if s.kind == "UDP SERVICE" and s.datagrams:
            # one datagram per read, with the address it came from (FC41D AT
            # manual: +QIRD: <len>,<remote IP>,<remote port> for UDP service)
            data, addr = s.datagrams.pop(0)
            data = data[:n]
            del s.rxbuf[:len(data)]
            head = f'+QIRD: {len(data)},"{addr[0]}",{addr[1]}' if addr else f"+QIRD: {len(data)}"
            self.send(f"\r\n{head}\r\n".encode() + data + b"\r\nOK\r\n")
            return
        data = bytes(s.rxbuf[:n])
        del s.rxbuf[:n]
        self.send(f"\r\n+QIRD: {len(data)}\r\n".encode() + data + b"\r\nOK\r\n")

    def at_QICLOSE(self, mode, args):
        sid = int(args[0])
        s = self.sockets.pop(sid, None)
        if s:
            self.net.socket_close(s)
        self.ok()

    def socket_received(self, sid: int, data: bytes, addr: Optional[Tuple[str, int]] = None) -> None:
        s = self.sockets.get(sid)
        if s is None:
            return
        s.rx_total += len(data)
        if s.mode == 1:
            hdr = f'+QIURC: "recv",{sid},{len(data)}'
            if s.kind == "UDP SERVICE" and addr:
                hdr += f',"{addr[0]}",{addr[1]}'
            self.send(f"\r\n{hdr}\r\n".encode() + data + b"\r\n")
        elif s.mode == 2:
            self.send(data)
        else:
            s.rxbuf.extend(data)
            if s.kind == "UDP SERVICE":
                s.datagrams.append((bytes(data), addr))
            self.urc(f'+QIURC: "recv",{sid}')

    def socket_closed(self, sid: int) -> None:
        if sid in self.sockets:
            self.urc(f'+QIURC: "closed",{sid}')

    # ---- Hame extension -------------------------------------------------
    def at_HMIOPEN(self, mode, args):
        self.ok()
        self.urc("+HMIOPEN FAIL", 50)

    # ------------------------------------------------------------- utilities
    def dump_transcript(self, limit: int = 0) -> str:
        out = []
        items = self.transcript[-limit:] if limit else self.transcript
        for t, d, data in items:
            txt = data.decode("latin1")
            if any(ord(c) < 9 or 13 < ord(c) < 32 or ord(c) > 126 for c in txt):
                txt = "hex:" + binascii.hexlify(data).decode()
            out.append(f"{t:9.3f} {d} {txt!r}")
        return "\n".join(out)
