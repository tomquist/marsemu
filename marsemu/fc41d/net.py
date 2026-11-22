"""The outside world as seen by the emulated FC41D.

``Network`` decides what happens when the firmware opens an MQTT session,
issues an HTTP request or opens a socket.  Three MQTT modes exist:

* ``offline`` (default): the session "connects" instantly, publishes are only
  recorded, messages can be injected from scripts / the CLI.
* ``broker``: every session is redirected to one plain-TCP broker (e.g. a
  local mosquitto), topics unchanged.  Use ``mosquitto_sub -t '#'`` to watch
  the device and ``mosquitto_pub`` to control it.
* ``cloud``: connect to the host/port the firmware asked for, with TLS and
  the CA/client certificate the firmware uploaded via ``AT+QSSLCERT``.

HTTP is ``offline`` (answered from rules, default 404) or ``real``.
Sockets are ``offline`` (client connects fail, listeners accept) or ``real``
(client sockets connect for real, listeners bind on the host at
``local_port + listen_offset``).
"""
from __future__ import annotations

import logging
import os
import re
import socket
import ssl
import tempfile
import threading
import urllib.error
import urllib.request
from typing import Callable, List, Optional, Tuple

from .mqtt import MqttClient

log = logging.getLogger("fc41d.net")


class Network:
    def __init__(self, mqtt: str = "offline", broker: Tuple[str, int] = ("127.0.0.1", 1883),
                 broker_auth: Optional[Tuple[str, str]] = None, http: str = "offline",
                 http_rules: Optional[List[Tuple[str, int, bytes]]] = None, sockets: str = "offline",
                 listen_offset: int = 0, mqtt_connect_ms: float = 150):
        if mqtt == "cloud" or http == "real":
            log.warning("connecting to the vendor's servers: use only your own device's identity and account")
        self.mqtt_mode = mqtt
        self.broker = broker
        self.broker_auth = broker_auth
        self.http_mode = http
        self.http_rules = list(http_rules or [])
        self.http_default = (404, b"")
        self.sock_mode = sockets
        self.listen_offset = listen_offset
        self.mqtt_connect_ms = mqtt_connect_ms
        self.module = None  # set by Fc41d
        #: in-process UDP peer: fn(socket, data, (dest_ip, dest_port))
        self.udp_handler = None
        #: destination port -> (host, port) to forward UDP datagrams to, e.g.
        #: {12345: ("127.0.0.1", 12345)} sends CT002 polls to a local AstraMeter
        self.udp_forward: dict = {}
        #: answer the Hamedata cloud endpoints locally (time handshake etc.)
        self.fake_cloud = http != "real"

    def attach(self, module) -> None:
        self.module = module
        if self.fake_cloud:
            from .cloud import hamedata_rules
            rtc = module.m.bus.get("RTC")
            self.http_rules.extend(hamedata_rules(rtc.now))

    # ------------------------------------------------------------------ helpers
    def _later(self, ms: float, fn: Callable[[], None]) -> None:
        self.module.m.clock.after_ms(ms, fn, "net")

    def _thread(self, fn: Callable[[], None]) -> None:
        threading.Thread(target=fn, daemon=True).start()

    # --------------------------------------------------------------------- MQTT
    def mqtt_open(self, sess, cb: Callable[[int], None]) -> None:
        self._later(self.mqtt_connect_ms, lambda: cb(0))

    def _tls_context(self, sess) -> Optional[ssl.SSLContext]:
        ssl_on = sess.cfg.get("ssl", ["0"])[0] == "1" or sess.port in (8883, 8884)
        if not ssl_on:
            return None
        ctx = ssl.create_default_context()
        certs = self.module.certs
        ctx.check_hostname = False
        if "CA" in certs:
            try:
                ctx.load_verify_locations(cadata=certs["CA"].decode("latin1"))
            except ssl.SSLError as e:
                log.warning("could not load firmware CA: %s", e)
                ctx.verify_mode = ssl.CERT_NONE
        else:
            ctx.verify_mode = ssl.CERT_NONE
        if "User Cert" in certs and "User Key" in certs:
            d = tempfile.mkdtemp(prefix="fc41d-")
            cp, kp = os.path.join(d, "cert.pem"), os.path.join(d, "key.pem")
            open(cp, "wb").write(certs["User Cert"])
            open(kp, "wb").write(certs["User Key"])
            try:
                ctx.load_cert_chain(cp, kp)
            except ssl.SSLError as e:
                log.warning("could not load firmware client certificate: %s", e)
        return ctx

    def mqtt_connect(self, sess, cb: Callable[[int], None]) -> None:
        mod = self.module
        if self.mqtt_mode == "offline":
            self._later(self.mqtt_connect_ms, lambda: cb(0))
            return
        if self.mqtt_mode == "broker":
            host, port = self.broker
            tls = None
            user, pw = self.broker_auth or (sess.user, sess.password)
        else:
            host, port = sess.host, sess.port
            tls = self._tls_context(sess)
            user, pw = sess.user, sess.password

        def on_message(topic, payload):
            mod.post(lambda: mod.mqtt_deliver(topic, payload, sess.idx))

        def on_close(reason):
            # A connection the firmware has since replaced (it opened and
            # connected the session again) is not this session's any more:
            # its loss, often the broker closing it for the new one's client
            # ID, must not drop the new connection.
            if sess.client is not client:
                return
            log.warning("MQTT #%d connection lost: %s", sess.idx, reason)
            mod.post(lambda: sess.client is client and mod._mqtt_drop(sess, 1))

        old = sess.client
        if old is not None:
            sess.client = sess.conn = None
            old.close()
        client = MqttClient(host, port, tls=tls, on_message=on_message, on_close=on_close)
        sess.client = client
        keepalive = int((sess.cfg.get("keepalive") or ["60"])[0] or 60)
        t0 = mod.m.clock.seconds

        def done(rc):
            # A real module takes a while to connect to the cloud broker, and
            # some firmware (B2500 HMJ) misses a +QMTCONN that comes sooner:
            # never answer before the modelled connect time, however fast
            # the local broker is.
            if sess.client is not client:
                return
            wait = self.mqtt_connect_ms - (mod.m.clock.seconds - t0) * 1000.0
            self._later(max(wait, 0.0), lambda: sess.client is client and cb(rc))

        def run():
            rc = client.connect(sess.client_id, user or None, pw or None, keepalive)
            if rc == 0:
                sess.conn = client
                for t in sess.subs:
                    client.subscribe(t)
            log.info("MQTT #%d -> %s:%d rc=%d", sess.idx, host, port, rc)
            mod.post(lambda: done(rc))
        self._thread(run)

    def mqtt_subscribe(self, sess, topic: str, qos: int) -> None:
        if isinstance(sess.conn, MqttClient):
            sess.conn.subscribe(topic, qos)

    def mqtt_unsubscribe(self, sess, topic: str) -> None:
        if isinstance(sess.conn, MqttClient):
            sess.conn.unsubscribe(topic)

    def mqtt_publish(self, sess, topic: str, payload: bytes, qos: int, retain: int) -> None:
        if isinstance(sess.conn, MqttClient):
            sess.conn.publish(topic, payload, qos, bool(retain))

    def mqtt_close(self, sess) -> None:
        client = sess.client
        sess.client = None
        if isinstance(client, MqttClient):
            client.close()
        if isinstance(sess.conn, MqttClient) and sess.conn is not client:
            sess.conn.close()
        sess.conn = None

    # --------------------------------------------------------------------- HTTP
    def add_http_rule(self, pattern: str, status: int, body: bytes) -> None:
        self.http_rules.insert(0, (pattern, status, body))

    def http(self, method: str, url: str, headers: dict, body: bytes,
             cb: Callable[[Optional[int], bytes], None]) -> None:
        for pat, status, data in self.http_rules:
            if re.search(pat, url):
                if callable(data):
                    data = data(url, body)
                log.info("HTTP %s %s -> %d (rule)", method, url, status)
                cb(status, data)
                return
        if self.http_mode != "real":
            log.info("HTTP %s %s -> %d (offline default)", method, url, self.http_default[0])
            cb(*self.http_default)
            return

        def run():
            req = urllib.request.Request(url, data=body if method == "POST" else None, method=method,
                                         headers=headers)
            try:
                with urllib.request.urlopen(req, timeout=20) as r:
                    cb(r.status, r.read())
            except urllib.error.HTTPError as e:
                cb(e.code, e.read() if e.fp else b"")
            except Exception as e:  # noqa: BLE001
                log.warning("HTTP %s %s failed: %s", method, url, e)
                cb(None, b"")
        self._thread(run)

    # ------------------------------------------------------------------ sockets
    def socket_open(self, s, cb: Callable[[int], None]) -> None:
        mod = self.module
        if s.kind in ("TCP LISTENER", "UDP SERVICE"):
            if self.sock_mode == "real":
                self._listen(s)
            self._later(10, lambda: cb(0))
            return
        if self.sock_mode != "real":
            self._later(200, lambda: cb(566))
            return

        def run():
            try:
                if s.kind == "UDP":
                    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    so.connect((s.host, s.port))
                else:
                    so = socket.create_connection((s.host, s.port), timeout=10)
                s.conn = so
                mod.post(lambda: cb(0))
                while True:
                    data = so.recv(1500)
                    if not data:
                        break
                    mod.post(lambda d=data: mod.socket_received(s.sid, d))
            except OSError as e:
                log.info("socket %d: %s", s.sid, e)
                if s.conn is None:
                    mod.post(lambda: cb(566))
            mod.post(lambda: mod.socket_closed(s.sid))
        self._thread(run)

    def _listen(self, s) -> None:
        mod = self.module
        port = s.local_port + self.listen_offset
        if s.kind == "UDP SERVICE":
            so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            so.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                so.bind(("127.0.0.1", port))
            except (OSError, OverflowError) as e:   # OverflowError: port + offset outside 0-65535
                log.warning("cannot bind UDP %d: %s", port, e)
                return
            s.conn = so
            log.info("device UDP service %d reachable on host udp/127.0.0.1:%d", s.local_port, port)

            def run():
                while True:
                    try:
                        data, addr = so.recvfrom(2048)
                    except OSError:
                        return
                    s.peer = addr
                    mod.post(lambda d=data, a=addr: mod.socket_received(s.sid, d, a))
            self._thread(run)

    def socket_send(self, s, data: bytes, extra) -> None:
        if s.kind.startswith("UDP"):
            dest = (s.host, s.port)
            if extra and len(extra) >= 2:
                try:
                    dest = (extra[0].strip('"'), int(extra[1]))
                except ValueError:
                    pass
            if self.udp_handler is not None and self.udp_handler(s, data, dest) is not False:
                # in-process peer (e.g. a CT002/CT003 emulator); it answers by
                # calling udp_reply().  A handler returns False for datagrams
                # that aren't for it (a Local API reply, say): those go on below.
                return
            fwd = self.udp_forward.get(dest[1])
            if fwd is not None and s.conn is not None:
                try:
                    s.conn.sendto(data, fwd)
                except OSError as e:
                    log.info("socket %d forward failed: %s", s.sid, e)
                return
        so = s.conn
        if so is None:
            return
        try:
            if s.kind == "UDP SERVICE":
                host, port = (extra[0].strip('"'), int(extra[1])) if extra and len(extra) >= 2 else getattr(s, "peer", None)
                so.sendto(data, (host, port))
            else:
                so.sendall(data)
        except (OSError, TypeError) as e:
            log.info("socket %d send failed: %s", s.sid, e)

    def udp_reply(self, s, data: bytes, src: Tuple[str, int]) -> None:
        """Deliver a datagram to the firmware as if it came from ``src``.
        Thread-safe."""
        mod = self.module
        mod.post(lambda: mod.socket_received(s.sid, bytes(data), src))

    def socket_close(self, s) -> None:
        if s.conn is not None:
            try:
                s.conn.close()
            except OSError:
                pass
