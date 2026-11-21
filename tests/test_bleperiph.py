"""The virtual BLE peripheral (marsemu.bleperiph), with a Bumble central on
an in-memory link: what a phone or a browser does over the air."""
import asyncio
import threading

import pytest

pytest.importorskip("bumble")

from bumble.controller import Controller  # noqa: E402
from bumble.core import UUID, AdvertisingData  # noqa: E402
from bumble.device import Device, Peer  # noqa: E402
from bumble.hci import Address  # noqa: E402
from bumble.host import Host  # noqa: E402
from bumble.link import LocalLink  # noqa: E402
from bumble.transport.common import AsyncPipeSink  # noqa: E402

from marsemu.bleperiph import BlePeripheral  # noqa: E402


class FakeModule:
    """What the peripheral uses of the FC41D model, set up as the firmware
    sets it up; answers each write with the bytes reversed (times three for
    a long reply)."""

    def __init__(self):
        self.listeners = {}
        self.ble_name = "Quec_BLE"
        self.ble_adv = False
        self.ble_services, self.ble_chars = [], []
        self.ble_connected = False
        self.cfg = type("Cfg", (), {"ble_mac": "c8:47:8c:00:00:02"})()
        self.writes = []
        self.events = []
        self.lock = threading.Lock()

    def on(self, event, fn):
        self.listeners.setdefault(event, []).append(fn)

    def emit(self, event, *args):
        for fn in list(self.listeners.get(event, [])):
            fn(*args)

    def post(self, fn):
        with self.lock:
            fn()

    def ble_connect(self):
        self.ble_connected = True
        self.events.append("connected")

    def ble_disconnect(self):
        self.ble_connected = False
        self.events.append("disconnected")
        self.emit("ble_state", "disconnected")

    def ble_write(self, data, uuid="ff01"):
        self.writes.append((uuid, bytes(data)))
        reply = bytes(reversed(data))
        self.emit("ble_notify", "ff02", reply * 3 if data[:1] == b"L" else reply)

    def firmware_setup(self):
        """AT+QBLENAME, QBLEGATTSSRV, QBLEGATTSCHAR ×3, QBLEADVSTART."""
        self.ble_name = "MST_VNSE3_0002"
        self.emit("ble_name", self.ble_name)
        self.ble_services = ["ff00"]
        self.ble_chars = ["ff01", "ff02", "ff06"]
        self.ble_adv = True
        self.emit("ble_adv", True)


@pytest.fixture
def peripheral():
    module = FakeModule()
    link = LocalLink()
    p = BlePeripheral(type("Emu", (), {"fc41d": module})(), None, link=link).start()
    yield p, module, link
    p.close()


def run(p, coro, timeout=20):
    return asyncio.run_coroutine_threadsafe(coro, p.loop).result(timeout)


async def scan(central, timeout=5.0):
    found = asyncio.get_running_loop().create_future()

    def on_adv(adv):
        name = adv.data.get(AdvertisingData.COMPLETE_LOCAL_NAME)
        if name and not found.done():
            found.set_result((adv.address, name))
    central.on(central.EVENT_ADVERTISEMENT, on_adv)
    await central.start_scanning()
    try:
        return await asyncio.wait_for(found, timeout)
    finally:
        central.remove_listener(central.EVENT_ADVERTISEMENT, on_adv)
        await central.stop_scanning()


def test_a_central_finds_connects_writes_and_gets_notified(peripheral):
    p, module, link = peripheral

    async def central_session():
        ctl = Controller("phone", link=link, public_address="F0:F0:F0:F0:F0:F0")
        central = Device(name="phone", address=Address("F0:F0:F0:F0:F0:F0"), host=Host(ctl, AsyncPipeSink(ctl)))
        await central.power_on()
        # nothing to find until the firmware starts advertising
        with pytest.raises(asyncio.TimeoutError):
            await scan(central, 0.5)
        module.firmware_setup()
        address, name = await scan(central)
        assert name == "MST_VNSE3_0002"
        assert str(address).startswith("C8:47:8C:00:00:02")
        conn = await central.connect(address)
        peer = Peer(conn)
        assert await peer.request_mtu(517) == 247
        await peer.discover_services()
        await peer.discover_characteristics()
        assert [str(s.uuid) for s in peer.services if "FF00" in str(s.uuid)]
        (ff01,) = peer.get_characteristics_by_uuid(UUID("FF01"))
        (ff02,) = peer.get_characteristics_by_uuid(UUID("FF02"))
        assert peer.get_characteristics_by_uuid(UUID("FF06"))
        got = asyncio.Queue()
        await peer.subscribe(ff02, lambda value: got.put_nowait(bytes(value)))
        await ff01.write_value(bytes.fromhex("7305230356"), with_response=False)
        first = await asyncio.wait_for(got.get(), 5)
        long = b"L" + bytes(range(60))
        await ff01.write_value(long, with_response=True)
        second = await asyncio.wait_for(got.get(), 5)
        connected_seen = module.ble_connected
        await conn.disconnect()
        await asyncio.sleep(0.2)
        # advertising again for the next central
        address2, _ = await scan(central)
        return first, second, connected_seen, address2

    first, second, connected_seen, again = run(p, central_session())
    assert connected_seen
    assert first == bytes.fromhex("5603230573")
    assert second == bytes(reversed(b"L" + bytes(range(60)))) * 3       # 183 bytes in one notification
    assert module.writes[0] == ("ff01", bytes.fromhex("7305230356"))
    assert module.events == ["connected", "disconnected"]
    assert again


def test_the_firmware_dropping_the_connection_disconnects_the_central(peripheral):
    p, module, link = peripheral
    module.firmware_setup()

    async def central_session():
        ctl = Controller("phone", link=link, public_address="F1:F1:F1:F1:F1:F1")
        central = Device(name="phone", address=Address("F1:F1:F1:F1:F1:F1"), host=Host(ctl, AsyncPipeSink(ctl)))
        await central.power_on()
        address, _ = await scan(central)
        conn = await central.connect(address)
        gone = asyncio.get_running_loop().create_future()
        conn.on(conn.EVENT_DISCONNECTION, lambda reason: gone.done() or gone.set_result(reason))
        await asyncio.sleep(0.1)
        module.post(module.ble_disconnect)            # AT+QBLEDISCONN
        await asyncio.wait_for(gone, 5)
        return True

    assert run(p, central_session())
    assert module.events == ["connected", "disconnected"]
