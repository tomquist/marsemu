"""FC41D AT engine tests (no firmware needed)."""
from marsemu.core.machine import Machine
from marsemu.fc41d.module import Fc41d, Fc41dConfig, split_args, topic_matches
from marsemu.fc41d.net import Network
from marsemu.periph import n32g45x


class FakeUart:
    def __init__(self):
        self.rx = bytearray()

    def inject(self, data):
        self.rx.extend(data)

    name = "FAKE"


def module(**cfg):
    m = Machine()
    n32g45x.build(m)
    f = Fc41d(m, Network(), Fc41dConfig(reply_ms=0, **cfg))
    u = FakeUart()
    f.attach(u)
    return m, f, u


def test_split_args_and_topics():
    assert split_args('0,"a,b",3') == ["0", '"a,b"', "3"]
    assert topic_matches("a/+/c", "a/b/c")
    assert topic_matches("a/#", "a/b/c")
    assert not topic_matches("a/+", "a/b/c")


def test_basic_commands():
    m, f, u = module()
    f.on_tx(b"AT+QVERSION\r\n")
    assert b"+QVERSION: FC41DAAR18A04M02_HM.bin_202409090159" in u.rx and u.rx.endswith(b"OK\r\n")
    u.rx.clear()
    f.on_tx(b"AT+QBLEADDR?\r\n")
    assert b"+QBLEADDR:" in u.rx


def test_mqtt_flow_and_inline_publish():
    m, f, u = module()
    f.got_ip = True
    pubs = []
    f.on("mqtt_publish", lambda t, p: pubs.append((t, p)))
    f.on_tx(b'AT+QMTOPEN=0,"broker",8883\r\n')
    m.advance(0.5)
    assert b"+QMTOPEN: 0,0" in u.rx
    f.on_tx(b'AT+QMTCONN=0,"client"\r\n')
    m.advance(0.5)
    assert b"+QMTCONN: 0,0,0" in u.rx
    f.on_tx(b'AT+QMTSUB=0,1,"x/App/1/ctrl",0\r\n')
    m.advance(0.1)
    # payload containing commas, quotes and CR/LF is length-delimited
    payload = b'a=1,b="2"\r\nc=3'
    f.on_tx(b'AT+QMTPUB=0,0,0,0,"x/device/1/ctrl",%d,"' % len(payload) + payload + b'"\r\n')
    assert pubs == [("x/device/1/ctrl", payload)]
    u.rx.clear()
    assert f.mqtt_deliver("x/App/1/ctrl", b"cd=1")
    m.advance(0.1)
    assert b'+QMTRECV: 0,1,"x/App/1/ctrl",4,"cd=1"' in u.rx


def test_udp_send_with_trailing_address():
    m, f, u = module()
    f.got_ip = True
    sent = []
    f.on("tcp_send", lambda sid, d, extra: sent.append((sid, d, extra)))
    f.on_tx(b'AT+QIOPEN=1,"UDP SERVICE","192.168.1.50",12345,10000,1\r\n')
    data = b"\x01\x02hello|x\x03"
    f.on_tx(b'AT+QISEND=1,%d,"' % len(data) + data + b'","192.168.1.255",12345\r\n')
    assert sent == [(1, data, ["192.168.1.255", "12345"])]


def test_udp_service_buffer_read_gives_the_sender():
    """A UDP service socket in buffer access mode (the Venus E Mini's CT002
    socket) reads one datagram at a time, with the address it came from:
    ``+QIRD: <len>,"<remote IP>",<remote port>`` (FC41D AT manual)."""
    m, f, u = module()
    f.got_ip = True
    f.on_tx(b'AT+QIOPEN=1,"UDP SERVICE","192.168.1.50",12345,22222,0\r\n')
    f.socket_received(1, b"first", ("192.168.1.77", 12345))
    f.socket_received(1, b"second!", ("192.168.1.78", 12345))
    assert b'+QIURC: "recv",1' in u.rx
    u.rx.clear()
    f.on_tx(b"AT+QIRD=1,0\r\n")
    assert b"+QIRD: 12,0,12" in u.rx
    u.rx.clear()
    f.on_tx(b"AT+QIRD=1,1500\r\n")
    assert bytes(u.rx) == b'\r\n+QIRD: 5,"192.168.1.77",12345\r\nfirst\r\nOK\r\n'
    u.rx.clear()
    f.on_tx(b"AT+QIRD=1,1500\r\n")
    assert bytes(u.rx).startswith(b'\r\n+QIRD: 7,"192.168.1.78",12345\r\nsecond!')
    u.rx.clear()
    f.on_tx(b"AT+QIRD=1,0\r\n")
    assert b"+QIRD: 12,12,0" in u.rx


def test_ble_notify_hex_and_raw():
    m, f, u = module()
    got = []
    f.on("ble_notify", lambda uuid, d: got.append(d))
    f.on_tx(b"AT+QBLEGATTSNTFY=ff02,3,730102\r\n")
    f.on_tx(b"AT+QBLEGATTSNTFY=ff02,3,\x00\x01\x02\r\n")
    assert got == [b"\x73\x01\x02", b"\x00\x01\x02"]



def test_ble_writes_wait_until_the_connection_settles():
    """A central's first writes reach the firmware BLE_SETTLE_S after it
    connects, in order, as a phone's do; later ones go straight through."""
    from marsemu.fc41d.module import BLE_SETTLE_S
    m, f, u = module(ble_rx_mode="raw")
    f.ble_connect()
    assert b"+QBLESTAT:CONNECTED" in u.rx
    u.rx.clear()
    f.ble_write(b"\x01")
    f.ble_write(b"\x02")
    assert u.rx == b""
    m.advance(BLE_SETTLE_S + 0.01)
    assert bytes(u.rx) == b"\x01\x02"
    f.ble_write(b"\x03")
    assert bytes(u.rx) == b"\x01\x02\x03"
    # a disconnect drops what is still held
    f.ble_disconnect()
    f.ble_connect()
    f.ble_write(b"\x04")
    f.ble_disconnect()
    u.rx.clear()
    m.advance(BLE_SETTLE_S + 0.01)
    assert b"\x04" not in u.rx

def test_cert_upload():
    m, f, u = module()
    f.on_tx(b'AT+QSSLCERT="CA",2,5\r\n')
    assert b"CONNECT" in u.rx
    f.on_tx(b"ABCDE")
    assert f.certs["CA"] == b"ABCDE"
    u.rx.clear()
    f.on_tx(b"AT+QSSLCERT?\r\n")
    # module build 202409090159: no checksums, all three lines in one block
    assert bytes(u.rx).startswith(b'\r\n+QSSLCERT: "CA",1\r\n+QSSLCERT: "User Cert",0\r\n+QSSLCERT: "User Key",0\r\n')
    u.rx.clear()
    f.cfg.version = "FC41DAAR18A04M02_HM.bin_202503010000"
    f.cfg.preloaded_certs["User Key"] = "236f"
    f.on_tx(b"AT+QSSLCERT?\r\n")
    assert b'+QSSLCERT: "CA",1,' in u.rx and b'+QSSLCERT: "User Key",1,236f\r\n' in u.rx


def test_wifi_autojoin_waits_for_idle():
    m, f, u = module(wifi_connect_ms=100, wifi_idle_ms=500)
    f.power_on()
    for _ in range(5):  # MCU busy: a command every 200 ms
        f.on_tx(b"AT\r\n")
        m.advance(0.2)
    assert not f.wifi_up
    m.advance(1.0)
    assert f.wifi_up and b"+QSTASTAT:WLAN_CONNECTED" in u.rx


def test_spi_nor_flash():
    from marsemu.periph.n32g45x import SpiNorFlash
    m = Machine()
    n32g45x.build(m)
    fl = SpiNorFlash(m, size=1 << 16)

    def tx(*bs):
        out = [fl.xfer(b) for b in bs]
        fl._end()
        return out
    assert tx(0x9F, 0, 0, 0)[1:] == [0xEF, 0x40, 0x16]
    tx(0x06)
    tx(0x02, 0, 0x10, 0, 1, 2, 3)
    assert tx(0x03, 0, 0x10, 0, 0, 0, 0)[4:] == [1, 2, 3]
    tx(0x06)
    tx(0x20, 0, 0x10, 0)
    assert tx(0x03, 0, 0x10, 0, 0)[4] == 0xFF


def test_mqtt_messages_at_the_same_instant_reach_the_mcu_one_after_the_other():
    m, f, u = module()
    f.got_ip = True
    f.on_tx(b'AT+QMTOPEN=0,"broker",8883\r\n')
    m.advance(0.5)
    f.on_tx(b'AT+QMTCONN=0,"client"\r\n')
    m.advance(0.5)
    f.on_tx(b'AT+QMTSUB=0,1,"x/App/1/ctrl",0\r\n')
    m.advance(0.1)
    u.rx.clear()
    assert f.mqtt_deliver("x/App/1/ctrl", b"cd=2,md=0")
    assert f.mqtt_deliver("x/App/1/ctrl", b"cd=56,dod=30")
    m.advance(0.1)
    assert b"cd=2,md=0" in u.rx and b"cd=56" not in u.rx
    m.advance(0.2)
    assert b'"cd=56,dod=30"' in u.rx


def test_mqtt_message_waits_for_a_command_reply_to_leave_the_uart():
    """A real module holds URCs while it answers a command: glued to the reply,
    the firmwares parse the reply and drop the +QMTRECV."""
    m = Machine()
    n32g45x.build(m)
    f = Fc41d(m, Network(), Fc41dConfig(reply_ms=3))
    log = []

    class TimedUart(FakeUart):
        def inject(self, data):
            log.append((m.clock.seconds * 1000.0, bytes(data)))
    f.attach(TimedUart())
    f.got_ip = True
    for cmd in (b'AT+QMTOPEN=0,"broker",8883\r\n', b'AT+QMTCONN=0,"client"\r\n',
                b'AT+QMTSUB=0,1,"x/App/1/ctrl",0\r\n'):
        f.on_tx(cmd)
        m.advance(0.5)
    log.clear()
    f.on_tx(b"AT+QVERSION\r\n")                       # its reply is due in 3 ms
    assert f.mqtt_deliver("x/App/1/ctrl", b"cd=14")   # and so would the message be
    m.advance(0.2)
    reply_end = max(t + len(d) * 10_000 / 115200 for t, d in log if b"QMTRECV" not in d)
    (t_recv,) = [t for t, d in log if b"+QMTRECV" in d]
    assert t_recv >= reply_end + f.cfg.urc_gap_ms - 0.01


class _Broker:
    """A minimal MQTT 3.1.1 broker on localhost: CONNACKs every CONNECT and,
    like mosquitto, closes an older connection that has the same client ID."""

    def __init__(self):
        import socket
        import threading
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen()
        self.port = self.srv.getsockname()[1]
        self.by_client = {}
        self.connects = 0
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        import threading
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _serve(self, c):
        try:
            data = c.recv(4096)
            if not data or data[0] >> 4 != 1:
                return
            # fixed header (1 byte type, 1 byte length for short packets),
            # protocol name, level, flags, keepalive, then the client ID
            i = 2 + 2 + data[3] + 4
            cid = data[i + 2:i + 2 + int.from_bytes(data[i:i + 2], "big")]
            old = self.by_client.get(cid)
            self.by_client[cid] = c
            if old is not None:
                import socket
                old.shutdown(socket.SHUT_RDWR)
                old.close()
            self.connects += 1
            c.sendall(b"\x20\x02\x00\x00")
            while c.recv(4096):
                pass
        except OSError:
            pass

    def close(self):
        self.srv.close()
        for c in self.by_client.values():
            c.close()


def _wait_for(m, u, text, seconds=5.0):
    import time
    end = time.time() + seconds
    while time.time() < end:
        m.advance(0.01)
        if text in u.rx:
            return True
        time.sleep(0.005)
    return False


def test_broker_connect_is_answered_after_the_modelled_connect_time():
    """A local broker answers within a millisecond; the module still takes
    its usual connect time, as B2500 (HMJ) firmware misses a +QMTCONN that
    comes sooner and starts the connection over."""
    b = _Broker()
    try:
        m = Machine()
        n32g45x.build(m)
        f = Fc41d(m, Network(mqtt="broker", broker=("127.0.0.1", b.port), mqtt_connect_ms=150),
                  Fc41dConfig(reply_ms=0))
        u = FakeUart()
        f.attach(u)
        f.got_ip = True
        f.on_tx(b'AT+QMTOPEN=0,"broker",8883\r\n')
        m.advance(0.5)
        t0 = m.clock.seconds
        f.on_tx(b'AT+QMTCONN=0,"client"\r\n')
        assert _wait_for(m, u, b"+QMTCONN: 0,0,0")
        assert m.clock.seconds - t0 >= 0.15
    finally:
        b.close()


def test_broker_reconnect_does_not_drop_the_new_connection():
    """Connecting a session again replaces its broker connection; the broker
    closing the old one for the shared client ID is no connection loss."""
    b = _Broker()
    try:
        m = Machine()
        n32g45x.build(m)
        f = Fc41d(m, Network(mqtt="broker", broker=("127.0.0.1", b.port), mqtt_connect_ms=0),
                  Fc41dConfig(reply_ms=0))
        u = FakeUart()
        f.attach(u)
        f.got_ip = True
        f.on_tx(b'AT+QMTOPEN=0,"broker",8883\r\n')
        m.advance(0.5)
        f.on_tx(b'AT+QMTCONN=0,"client"\r\n')
        assert _wait_for(m, u, b"+QMTCONN: 0,0,0")
        u.rx.clear()
        f.on_tx(b'AT+QMTCONN=0,"client"\r\n')
        assert _wait_for(m, u, b"+QMTCONN: 0,0,0")
        assert not _wait_for(m, u, b"+QMTSTAT", 0.5)
        assert b.connects == 2
        assert f.mqtt[0].connected
    finally:
        b.close()
