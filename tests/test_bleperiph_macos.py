"""The CoreBluetooth peripheral (marsemu.bleperiph_macos) against a stand-in
for CoreBluetooth that calls the delegate from its own thread, as macOS'
dispatch queue does. Runs anywhere; the real framework only on a Mac."""
import queue
import threading
import time
from types import SimpleNamespace

import pytest

from marsemu.bleperiph_macos import (ATT_SUCCESS, PERM_READABLE, PERM_WRITEABLE, PROP_NOTIFY, PROP_READ,
                                     PROP_WRITE, PROP_WRITE_NO_RESPONSE, CoreBluetoothPeripheral,
                                     make_delegate_class, uuid16)


# ----------------------------------------------------------- the stand-in
class NSObject:
    @classmethod
    def alloc(cls):
        return cls.__new__(cls)

    def init(self):
        return self


class CBUUID:
    def __init__(self, s):
        self.s = uuid16(s)          # CoreBluetooth shortens base UUIDs

    @classmethod
    def UUIDWithString_(cls, s):    # noqa: N802
        return cls(s)

    def UUIDString(self):           # noqa: N802
        return self.s


class CBMutableCharacteristic(NSObject):
    def initWithType_properties_value_permissions_(self, uuid, props, value, perms):  # noqa: N802
        self.uuid, self.props, self.value, self.perms = uuid, props, value, perms
        return self

    def UUID(self):                 # noqa: N802
        return self.uuid


class CBMutableService(NSObject):
    def initWithType_primary_(self, uuid, primary):  # noqa: N802
        self.uuid, self.primary, self.chars = uuid, primary, []
        return self

    def setCharacteristics_(self, chars):  # noqa: N802
        self.chars = list(chars)


class NSData:
    @staticmethod
    def dataWithBytes_length_(b, n):  # noqa: N802
        return bytes(b[:n])


class Central:
    def __init__(self, ident, mtu=247):
        self.ident, self.mtu = ident, mtu

    def identifier(self):
        return CBUUID(self.ident)

    def maximumUpdateValueLength(self):  # noqa: N802
        return self.mtu - 3


class Request:
    def __init__(self, central, char, value):
        self._c, self._ch, self._v = central, char, value
        self.response = None

    def central(self):
        return self._c

    def characteristic(self):
        return self._ch

    def value(self):
        return self._v

    def setValue_(self, v):  # noqa: N802
        self.response = v


class Manager(NSObject):
    """CBPeripheralManager: calls the delegate on its own thread."""
    state_on_start = 5

    def initWithDelegate_queue_(self, delegate, q):  # noqa: N802
        self.delegate, self.calls = delegate, []
        self.services, self.adverts, self.sent, self.responses = [], [], [], []
        self.advertising = False
        self.full = 0                                  # updateValue refuses this many times
        self.q = queue.Queue()
        threading.Thread(target=self._run, daemon=True).start()
        self.state = self.state_on_start
        self.dispatch(lambda: delegate.peripheralManagerDidUpdateState_(self))
        MANAGERS.append(self)
        return self

    def _run(self):
        while True:
            self.q.get()()

    def dispatch(self, fn):
        self.q.put(fn)

    def sync(self):
        done = threading.Event()
        self.dispatch(done.set)
        assert done.wait(2)

    # the API the peripheral calls
    def addService_(self, svc):  # noqa: N802
        self.services.append(svc)
        self.dispatch(lambda: self.delegate.peripheralManager_didAddService_error_(self, svc, None))

    def removeAllServices(self):  # noqa: N802
        self.services.clear()

    def startAdvertising_(self, data):  # noqa: N802
        self.adverts.append(dict(data))
        self.advertising = True
        self.dispatch(lambda: self.delegate.peripheralManagerDidStartAdvertising_error_(self, None))

    def stopAdvertising(self):  # noqa: N802
        self.advertising = False

    def updateValue_forCharacteristic_onSubscribedCentrals_(self, value, char, centrals):  # noqa: N802
        if self.full:
            self.full -= 1
            self.dispatch(lambda: self.delegate.peripheralManagerIsReadyToUpdateSubscribers_(self))
            return False
        self.sent.append((char.uuid.UUIDString(), bytes(value)))
        return True

    def respondToRequest_withResult_(self, request, result):  # noqa: N802
        self.responses.append((request, result))

    def state(self):
        return self._state

    # what a central does
    def subscribe(self, central, uuid):
        self.dispatch(lambda: self.delegate.peripheralManager_central_didSubscribeToCharacteristic_(
            self, central, self.char(uuid)))
        self.sync()

    def unsubscribe(self, central, uuid):
        self.dispatch(lambda: self.delegate.peripheralManager_central_didUnsubscribeFromCharacteristic_(
            self, central, self.char(uuid)))
        self.sync()

    def write(self, central, uuid, data):
        reqs = [Request(central, self.char(uuid), data)]
        self.dispatch(lambda: self.delegate.peripheralManager_didReceiveWriteRequests_(self, reqs))
        self.sync()
        return reqs[0]

    def char(self, uuid):
        return next(c for s in self.services for c in s.chars if c.uuid.UUIDString() == uuid)

    def set_state(self, st):
        self._state = st
        self.dispatch(lambda: self.delegate.peripheralManagerDidUpdateState_(self))
        self.sync()


Manager.state = property(lambda self: (lambda: self._state), lambda self, v: setattr(self, "_state", v))
MANAGERS = []
Delegate = make_delegate_class(NSObject)


def fake_cb():
    return SimpleNamespace(CBUUID=CBUUID, CBMutableService=CBMutableService,
                           CBMutableCharacteristic=CBMutableCharacteristic, CBPeripheralManager=Manager,
                           LocalNameKey="kCBAdvDataLocalName", NSData=NSData, new_queue=lambda: None,
                           new_delegate=lambda owner: Delegate.alloc().initWithOwner_(owner))


class FakeModule:
    """The FC41D model's BLE side, as the firmware sets it up."""

    def __init__(self):
        self.listeners = {}
        self.ble_name, self.ble_adv = "Quec_BLE", False
        self.ble_services, self.ble_chars = [], []
        self.ble_connected = False
        self.cfg = SimpleNamespace(ble_mac="c8:47:8c:00:00:02")
        self.writes, self.events = [], []

    def on(self, event, fn):
        self.listeners.setdefault(event, []).append(fn)

    def emit(self, event, *args):
        for fn in list(self.listeners.get(event, [])):
            fn(*args)

    def post(self, fn):
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

    def firmware_setup(self, name="MST_VNSE3_0002"):
        self.ble_name = name
        self.emit("ble_name", name)
        self.ble_services, self.ble_chars = ["ff00"], ["ff01", "ff02", "ff06"]
        self.ble_adv = True
        self.emit("ble_adv", True)

    def booted(self, mgr):
        """firmware_setup, then wait until macOS advertises it."""
        self.firmware_setup()
        assert until(lambda: mgr.advertising)


def until(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


@pytest.fixture
def setup():
    MANAGERS.clear()
    Manager.state_on_start = 5
    module = FakeModule()
    p = CoreBluetoothPeripheral(SimpleNamespace(fc41d=module), cb=fake_cb()).start(timeout=3)
    yield p, module, MANAGERS[0]
    p.close()


# ------------------------------------------------------------------ tests
def test_serves_what_the_firmware_sets_up(setup):
    p, module, mgr = setup
    assert not mgr.services and not mgr.adverts            # nothing before the firmware advertises
    module.firmware_setup()
    assert until(lambda: mgr.adverts)
    (svc,) = mgr.services
    assert svc.uuid.UUIDString() == "FF00" and svc.primary
    assert [c.uuid.UUIDString() for c in svc.chars] == ["FF01", "FF02", "FF06"]
    c = svc.chars[0]
    assert c.props == PROP_READ | PROP_WRITE | PROP_WRITE_NO_RESPONSE | PROP_NOTIFY and c.value is None
    assert c.perms == PERM_READABLE | PERM_WRITEABLE
    assert mgr.adverts == [{"kCBAdvDataLocalName": "MST_VNSE3_0002"}]
    # a new name: advertised again under it
    module.ble_name = "MST_VNSE3_1234"
    module.emit("ble_name", module.ble_name)
    assert until(lambda: mgr.adverts[-1] == {"kCBAdvDataLocalName": "MST_VNSE3_1234"})
    assert len(mgr.services) == 1


def test_a_central_connects_writes_is_notified_and_leaves(setup):
    p, module, mgr = setup
    module.firmware_setup()
    assert until(lambda: mgr.advertising)
    phone = Central("A1")
    mgr.subscribe(phone, "FF02")
    assert module.events == ["connected"]
    assert until(lambda: not mgr.advertising)               # one central at a time
    req = mgr.write(phone, "FF01", b"\x73\x05\x23\x03\x56")
    assert module.writes == [("ff01", b"\x73\x05\x23\x03\x56")]
    assert mgr.responses == [(req, ATT_SUCCESS)]
    module.emit("ble_notify", "ff02", b"reply")
    assert until(lambda: mgr.sent == [("FF02", b"reply")])
    # macOS' queue is full twice: sent when it says there is room
    mgr.full = 2
    module.emit("ble_notify", "ff02", b"later")
    assert until(lambda: mgr.sent[-1] == ("FF02", b"later"))
    # the last subscription goes: the central disconnected
    mgr.unsubscribe(phone, "FF02")
    assert module.events == ["connected", "disconnected"]
    assert until(lambda: mgr.advertising)
    assert mgr.adverts[-1] == {"kCBAdvDataLocalName": "MST_VNSE3_0002"}


def test_a_write_before_any_subscription_connects(setup):
    p, module, mgr = setup
    module.booted(mgr)
    phone = Central("B2")
    mgr.write(phone, "FF01", b"\x01")
    assert module.events == ["connected"] and module.writes == [("ff01", b"\x01")]
    mgr.subscribe(phone, "FF02")                           # the same central: no second connect
    assert module.events == ["connected"]


def test_the_firmware_cannot_drop_the_central(setup, caplog):
    p, module, mgr = setup
    module.booted(mgr)
    phone = Central("C3")
    mgr.subscribe(phone, "FF02")
    module.ble_disconnect()                                # AT+QBLEDISCONN
    assert "cannot disconnect a central" in caplog.text
    module.emit("ble_notify", "ff02", b"x")                # still served
    assert until(lambda: mgr.sent == [("FF02", b"x")])
    # the next write reconnects it on the module's side
    mgr.write(phone, "FF01", b"\x02")
    assert module.writes[-1] == ("ff01", b"\x02")


def test_bluetooth_off_and_on_again(setup):
    p, module, mgr = setup
    module.booted(mgr)
    phone = Central("D4")
    mgr.subscribe(phone, "FF02")
    mgr.services.clear()                                   # macOS forgets them when it powers off
    mgr.set_state(4)
    assert module.events == ["connected", "disconnected"]
    mgr.set_state(5)
    assert until(lambda: len(mgr.services) == 1 and mgr.advertising)


def test_long_notifications_warn_about_the_mtu(setup, caplog):
    p, module, mgr = setup
    module.booted(mgr)
    mgr.subscribe(Central("E5", mtu=23), "FF02")
    module.emit("ble_notify", "ff02", bytes(144))
    assert until(lambda: mgr.sent)
    assert "longer than the central takes (20 bytes)" in caplog.text


@pytest.mark.parametrize("state,words", [(3, "Privacy & Security"), (4, "Bluetooth is off"),
                                         (2, "cannot act as a peripheral")])
def test_explains_why_bluetooth_cannot_be_used(state, words):
    MANAGERS.clear()
    Manager.state_on_start = state
    try:
        with pytest.raises(RuntimeError, match=words):
            CoreBluetoothPeripheral(SimpleNamespace(fc41d=FakeModule()), cb=fake_cb()).start(timeout=0.3)
    finally:
        Manager.state_on_start = 5


def test_uuid16():
    assert uuid16("ff01") == uuid16("0xFF01") == uuid16("0000ff01-0000-1000-8000-00805f9b34fb") == "FF01"
    assert uuid16("6E400001-B5A3-F393-E0A9-E50E24DCCA9E") == "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"


def test_only_on_a_mac(monkeypatch):
    import sys
    from marsemu.bleperiph_macos import load_corebluetooth
    monkeypatch.setattr(sys, "platform", "linux")
    with pytest.raises(RuntimeError, match="macOS"):
        load_corebluetooth()
