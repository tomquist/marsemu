"""The add-on's choice of MQTT broker."""
from marsemu.ui.addon import mqtt_network


def test_custom_broker():
    net = mqtt_network({"mqtt": "custom", "mqtt_host": "192.168.1.5", "mqtt_port": 1884,
                        "mqtt_user": "bat", "mqtt_password": "pw"})()
    assert net.mqtt_mode == "broker" and net.broker == ("192.168.1.5", 1884) and net.broker_auth == ("bat", "pw")


def test_custom_broker_without_login():
    net = mqtt_network({"mqtt": "custom", "mqtt_host": "mqtt.lan", "mqtt_user": ""})()
    assert net.broker == ("mqtt.lan", 1883) and net.broker_auth is None


def test_custom_without_host_runs_offline():
    assert mqtt_network({"mqtt": "custom", "mqtt_host": " "})().mqtt_mode == "offline"


def test_home_assistant_without_supervisor_runs_offline(monkeypatch):
    monkeypatch.delenv("SUPERVISOR_TOKEN", raising=False)
    assert mqtt_network({"mqtt": "home_assistant"})().mqtt_mode == "offline"


def test_offline():
    assert mqtt_network({"mqtt": "offline"})().mqtt_mode == "offline"


def test_sigterm_stops_cleanly(tmp_path):
    """Home Assistant stops the add-on with SIGTERM.  In the container the add-on
    is PID 1, which ignores SIGTERM unless it handles it; it must exit 0."""
    import json
    import signal
    import subprocess
    import sys
    import time
    opts = tmp_path / "options.json"
    (tmp_path / "fw").mkdir()
    opts.write_text(json.dumps({"firmware_dir": str(tmp_path / "fw"), "mqtt": "offline"}))
    code = ("import sys; from marsemu.ui import addon; addon.OPTIONS = sys.argv[1]; "
            "raise SystemExit(addon.main())")
    env = {**__import__("os").environ, "INGRESS_PORT": "0"}
    p = subprocess.Popen([sys.executable, "-c", code, str(opts)], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.time() + 20
        out = ""
        while "dashboard on port" not in out and time.time() < deadline:
            out += p.stdout.readline()
        p.send_signal(signal.SIGTERM)
        assert p.wait(timeout=15) == 0
        assert "stopping (SIGTERM)" in out + p.stdout.read()
    finally:
        if p.poll() is None:
            p.kill()
