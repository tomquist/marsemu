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
