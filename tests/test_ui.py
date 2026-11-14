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
