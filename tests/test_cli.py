import pytest

from marsemu.cli import main


OWN = ["--uid", "00" * 12, "--wifi-mac", "c8:47:8c:12:34:01", "--ble-mac", "c8:47:8c:12:34:02"]


def test_vendor_cloud_needs_confirmation(capsys):
    assert main(["run", "missing.bin", "--mqtt", "cloud", *OWN]) == 2
    assert "needs --confirm-own-device" in capsys.readouterr().err
    assert main(["run", "missing.bin", "--http", "real", *OWN]) == 2


def test_vendor_cloud_needs_an_explicit_identity(capsys):
    """The default identity is the same for every marsemu user, so nobody's own."""
    assert main(["run", "missing.bin", "--mqtt", "cloud", "--confirm-own-device", "--uid", "00" * 12]) == 2
    assert "needs --wifi-mac, --ble-mac." in capsys.readouterr().err


def test_vendor_cloud_with_confirmation_gets_past_the_check(capsys):
    with pytest.raises(SystemExit):     # the missing image, after the check
        main(["run", "missing.bin", "--mqtt", "cloud", "--confirm-own-device", *OWN])
    assert "as your own device: UID 000000000000000000000000" in capsys.readouterr().err


def test_missing_image_is_a_clean_error(capsys):
    with pytest.raises(SystemExit) as e:
        main(["info", "missing.bin"])
    assert e.value.code == 2
    assert "missing.bin" in capsys.readouterr().err


# ---------------------------------------------------------------- driving a device for MQTT clients
import os
import signal
from types import SimpleNamespace

from marsemu.cli import _stop_on_sigterm, announce_identity, parse_phases
from marsemu.fc41d.net import Network
from marsemu.plant.venus3_cli import House, connect_house


class _Events:
    def __init__(self):
        self.listeners = {}

    def on(self, event, fn):
        self.listeners.setdefault(event, []).append(fn)

    def emit(self, event, *args):
        for fn in self.listeners.get(event, []):
            fn(*args)


def test_identity_gives_hm2mqtt_the_topic_id_or_the_mac_it_hashes_itself():
    said = []
    e = _Events()
    announce_identity(e, "c8:47:8c:00:00:07", said.append)
    e.emit("mqtt_subscribe", "marstek_energy/VNSE3-0/App/0123abcd/ctrl")
    e.emit("mqtt_subscribe", "hame_energy/VNSE3-0/App/0123abcd/ctrl")     # same device: said once
    e.emit("mqtt_subscribe", "marstek_energy/HMJ-2/App/fedc9876/ctrl")
    e.emit("mqtt_subscribe", "something/else")
    assert said == [
        "MQTT: VNSE3-0, topic id 0123abcd, BLE MAC c8478c000007; for hm2mqtt: DEVICE_0=VNSE3-0:0123abcd",
        "MQTT: HMJ-2, topic id fedc9876, BLE MAC c8478c000007; for hm2mqtt: DEVICE_0=HMJ-2:c8478c000007",
    ]


def test_parse_phases():
    assert parse_phases("500") == (500.0, 0.0, 0.0)
    assert parse_phases("500,-200,1000") == (500.0, -200.0, 1000.0)
    assert parse_phases("1 2") == (1.0, 2.0, 0.0)
    with pytest.raises(ValueError):
        parse_phases("1,2,3,4")


def test_whole_device_commands_take_an_identity_and_real_sockets():
    from marsemu.cli import build_parser
    args = build_parser().parse_args(["venusce", "ctl.bin", "bms.bin", "--ble-mac", "c8:47:8c:00:00:09",
                                      "--sockets", "real", "--listen-offset", "10000"])
    assert (args.cmd, args.ble_mac, args.sockets, args.listen_offset) == \
        ("venusce", "c8:47:8c:00:00:09", "real", 10000)


def test_sigterm_stops_like_ctrl_c():
    previous = _stop_on_sigterm()
    try:
        with pytest.raises(KeyboardInterrupt):
            os.kill(os.getpid(), signal.SIGTERM)
            for _ in range(1000):       # the handler runs between bytecodes
                pass
    finally:
        signal.signal(signal.SIGTERM, previous)


class _Sock:
    def __init__(self):
        self.sent = []

    def sendto(self, data, addr):
        self.sent.append((bytes(data), addr))


def _device_on(net):
    fc41d = SimpleNamespace(net=net, cfg=SimpleNamespace(ip="192.168.1.50"))
    return SimpleNamespace(sim=SimpleNamespace(tickers=[]), ac_power=0.0, control=SimpleNamespace(fc41d=fc41d))


def test_house_meter_takes_ct_polls_and_lets_other_datagrams_out():
    """A Local API reply is not a CT poll: with real sockets it must reach the
    client that asked, not vanish into the meter."""
    net = Network(sockets="real")
    polls = []
    connect_house(_device_on(net), House({"A": 100.0}), meter=lambda request, src: polls.append(request))
    api = SimpleNamespace(sid=1, kind="UDP SERVICE", host="", port=0, conn=_Sock(), peer=("127.0.0.1", 40000))
    net.socket_send(api, b'{"id":1,"result":{}}', ['"127.0.0.1"', "40000"])
    assert api.conn.sent == [(b'{"id":1,"result":{}}', ("127.0.0.1", 40000))]
    ct = SimpleNamespace(sid=2, kind="UDP", host="192.168.1.255", port=12345, conn=_Sock())
    net.socket_send(ct, b"poll", None)
    assert polls == [b"poll"] and ct.conn.sent == []


def test_listen_offset_out_of_range_is_a_warning_not_a_crash():
    net = Network(sockets="real", listen_offset=-70000)
    net.module = SimpleNamespace()
    s = SimpleNamespace(kind="UDP SERVICE", local_port=30000, conn=None)
    net._listen(s)
    assert s.conn is None
