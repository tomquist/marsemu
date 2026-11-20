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
