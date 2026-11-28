"""The dashboard's server, without firmware: routes, uploads, errors.

The device side is covered by tests/test_firmware.py (``-m firmware``)."""
import json
import threading
import urllib.error
import urllib.request

import pytest

from marsemu.ui.server import UiApp, make_handler
from http.server import ThreadingHTTPServer


@pytest.fixture
def server(tmp_path):
    app = UiApp(str(tmp_path / "fw"))
    srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield app, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def req(url, data=None, method=None):
    r = urllib.request.Request(url, data=data, method=method or ("POST" if data is not None else "GET"))
    try:
        with urllib.request.urlopen(r, timeout=5) as resp:
            return resp.status, resp.headers.get("Content-Type"), resp.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get("Content-Type"), e.read()


def test_page_and_assets(server):
    _, base = server
    code, ctype, body = req(base + "/")
    assert code == 200 and ctype.startswith("text/html") and b"app.js" in body
    assert req(base + "/app.js")[0] == 200
    assert req(base + "/app.css")[1].startswith("text/css")


def test_ingress_prefix(server):
    """Home Assistant's ingress puts a path in front; every route still works."""
    _, base = server
    assert req(base + "/api/hassio_ingress/abc123/")[1].startswith("text/html")
    code, ctype, body = req(base + "/api/hassio_ingress/abc123/api/state")
    assert code == 200 and json.loads(body)["running"] is False


def test_upload_lists_and_sanitises(server, tmp_path):
    app, base = server
    code, _, body = req(base + "/api/firmware?name=" + "..%2F..%2Fevil%20name.bin", b"\x00" * 512)
    assert code == 200
    name = json.loads(body)["name"]
    assert "/" not in name and name.endswith(".bin")
    assert (tmp_path / "fw" / name).exists()
    state = json.loads(req(base + "/api/state")[2])
    assert [i["name"] for i in state["images"]] == [name]
    assert state["images"][0]["supported"] is False


def test_start_refuses_what_it_cannot_run(server):
    _, base = server
    req(base + "/api/firmware?name=blob.bin", b"\x00" * 512)
    code, _, body = req(base + "/api/start", json.dumps({"image": "blob.bin"}).encode())
    assert code == 400 and "not in the dashboard yet" in json.loads(body)["error"]
    assert req(base + "/api/start", json.dumps({"image": "missing.bin"}).encode())[0] == 404


def test_control_needs_a_device(server):
    _, base = server
    code, _, body = req(base + "/api/control", json.dumps({"op": "load", "w": 100}).encode())
    assert code == 409


def test_uploads_can_be_switched_off(tmp_path):
    app = UiApp(str(tmp_path), allow_upload=False)
    with pytest.raises(PermissionError):
        app.save_image("x.bin", b"\x00")


def test_stream_says_hello(server):
    _, base = server
    with urllib.request.urlopen(base + "/api/stream", timeout=5) as resp:
        assert resp.headers.get("Content-Type") == "text/event-stream"
        assert resp.readline() == b"event: hello\n"
        assert json.loads(resp.readline()[len(b"data: "):])["running"] is False


def test_venus_can_frames_are_decoded():
    import struct
    from marsemu.ui.session import decode_venus_can
    assert decode_venus_can(0x1801FFAA, b"", "control")[0] == "control→BMS"
    d, label, text = decode_venus_can(0x1801AA01, struct.pack("<HhhH", 5281, -118, 250, 535), "bms")
    assert d == "BMS→control" and "52.81 V" in text and "-11.8 A" in text and "SOC 53.5 %" in text
    d, label, text = decode_venus_can(0x140141 << 8 | 0x01, struct.pack("<i", 600), "control")
    assert d == "control→inverter" and label == "setpoint" and "600 W" in text
    d, label, text = decode_venus_can(0x01410101, struct.pack("<i", 600), "micro")
    assert d == "inverter→control" and "echo 600 W" in text
    d, label, text = decode_venus_can(0x1830AA01, struct.pack("<4H", 3299, 3300, 3301, 3302), "bms")
    assert d == "BMS→control" and "cells 1–4: 3299 mV" in text
    assert decode_venus_can(0x1841AA01, bytes(8), "bms")[0] == "BMS→control"


def test_images_are_classified():
    from marsemu.ui.server import describe
    b = describe("b.bin", 1, {"profile": "b2500-v3", "device_type_default": "HMJ-2"})
    assert b["supported"] and b["model"] == "B2500" and not b["needs_bms"]
    v = describe("151_control_VNSE3-0.bin", 1, {"profile": "venus3", "device_types": ["HME-4", "VNSE3-0"]})
    assert v["supported"] and v["model"] == "Venus E 3.0" and v["needs_bms"]
    d = describe("151_control_VNSD-0.bin", 1, {"profile": "venus3", "device_types": ["HME-4", "VNSD-0"]})
    assert d["model"] == "Venus D" and d["needs_bms"] and d["pv_inputs"] == 4 and d["packs"]
    a = describe("151_control_VNSA-0.bin", 1, {"profile": "venus3", "device_types": ["HME-4", "VNSA-0"]})
    assert a["model"] == "Venus A"
    ce = describe("158_control_HMG-50.bin", 1, {"profile": "venus", "device_types": ["HME-3", "HMG-1"]})
    assert ce["model"] == "Venus C/E" and ce["needs_bms"]
    j = describe("143_JPLS-8H_app.bin", 1, {"profile": "jupiter", "device_types": ["HME-3", "HMM-1", "JPLS-8H"]})
    assert j["model"] == "Jupiter C Plus" and not j["needs_bms"] and j["bms_optional"]
    mini = describe("309_control_VNSEMINI-0.bin", 1,
                    {"profile": "venus-mini", "device_types": ["HMC-1", "HME-3", "HME-4", "VNSEM-0", "VNSEMINI-0"]})
    assert mini["supported"] and mini["model"] == "Venus E Mini" and not mini["needs_bms"]
    assert mini["device_type"] == "VNSEMINI-0" and mini["pv_inputs"] == 0
    ct = describe("124_HME-4.bin", 1, {"profile": "ct00x", "device_types": ["HME-4"]})
    assert ct["supported"] and ct["model"] == "CT002 / CT003"
    hmi = describe("132_control_HMI-5_2025.bin", 1, {"profile": "hmi", "device_types": ["HMI-2", "HMI-1", "HMI-5"]})
    assert hmi["supported"] and hmi["model"] == "HMI" and hmi["pv_inputs"] == 2 and not hmi["needs_bms"]
    assert hmi["device_type"] == "HMI-5"
    assert describe("115_bms.bin", 1, {"profile": "aux"})["role"] == "bms"
    # the MPPT board's image goes with a Venus D or a Jupiter, as an option
    mp = describe("104_mppt_VNSD-0.bin", 1, {"profile": "mppt", "device_types": ["VNSD"]})
    assert mp["role"] == "mppt" and not mp["supported"]
    assert d["mppt_optional"] and j["mppt_optional"] and not a["mppt_optional"] and not v["mppt_optional"]



def test_ble_websocket_needs_a_running_device(server):
    """api/ble is the running device's Bluetooth over WebSocket; with no
    device it says so and closes."""
    from test_blews import Client
    _, url = server
    c = Client(int(url.rsplit(":", 1)[1]), "/api/ble")
    msg, kind = c.recv()
    assert kind == 1 and json.loads(msg) == {"type": "error", "message": "no device is running"}
    assert c.recv()[1] == 8
    c.s.close()


def test_ble_websocket_reaches_the_running_device(server):
    from test_blews import Client, FakeModule
    app, url = server
    module = FakeModule()
    app.session = type("Session", (), {"emu": type("Emu", (), {"fc41d": module})()})()
    c = Client(int(url.rsplit(":", 1)[1]), "/prefix/api/ble")      # behind ingress too
    assert json.loads(c.recv()[0])["name"] == "MST_VNSE3_0002"
    c.send(b"\x73\x01")
    assert c.recv()[0] == b"\x01\x73"
    c.close()
    app.session = None
