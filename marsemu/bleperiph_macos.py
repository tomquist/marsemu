"""The emulated device as a Bluetooth LE peripheral on a Mac's own Bluetooth.

``--ble-peripheral corebluetooth`` serves the FC41D model's Bluetooth through
macOS' CoreBluetooth framework (``CBPeripheralManager``, via pyobjc: ``pip
install 'marsemu[ble]'`` on a Mac), on the Mac's built-in radio. Phones,
other computers and the Marstek app nearby then find the emulated device by
the name its firmware set and talk to it as to the device.

CoreBluetooth decides more than a Bluetooth controller of our own would
(:mod:`marsemu.bleperiph`), and that shows:

* The advertisement carries the name the firmware set and nothing else we
  choose; macOS picks the address (random, not the device's MAC) and the
  interval. After connecting, a central that reads the GAP device name gets
  the Mac's name; scanners show the advertised one.
* There is no connection event for a peripheral. A central counts as
  connected from its first subscription (to FF02, which apps and hmjs do
  before they write) or write, and as gone when it has no subscription left,
  which CoreBluetooth reports when it disconnects.
* A peripheral cannot drop a central, so the firmware's ``AT+QBLEDISCONN``
  is logged and otherwise has no effect.
* Notifications longer than a central's ``maximumUpdateValueLength`` (its
  ATT MTU − 3) are cut by macOS: a central must ask for an MTU large enough
  for the longest reply (147 for the Venus' 144-byte runtime info), which
  phones and Chrome do.
* The Mac cannot connect to itself: the central is another device.

The terminal (or whatever runs marsemu) needs the Bluetooth permission
(System Settings → Privacy & Security → Bluetooth); macOS asks the first
time.
"""
from __future__ import annotations

import logging
import queue
import sys
import threading
from types import SimpleNamespace
from typing import Dict, Optional

log = logging.getLogger("emu.bleperiph")

# CBManagerState
STATE_NAMES = {0: "unknown", 1: "resetting", 2: "unsupported", 3: "unauthorized", 4: "powered off",
               5: "powered on"}
POWERED_ON = 5
STATE_HELP = {
    2: "this Mac's Bluetooth cannot act as a peripheral",
    3: "marsemu may not use Bluetooth: allow the app running it (e.g. Terminal) in System Settings → "
       "Privacy & Security → Bluetooth",
    4: "Bluetooth is off: turn it on in System Settings → Bluetooth",
}
# CBCharacteristicProperties, CBAttributePermissions, CBATTError
PROP_READ, PROP_WRITE_NO_RESPONSE, PROP_WRITE, PROP_NOTIFY = 0x02, 0x04, 0x08, 0x10
PERM_READABLE, PERM_WRITEABLE = 0x01, 0x02
ATT_SUCCESS = 0
#: how long to wait for macOS to take a notification when its queue is full
NOTIFY_WAIT_S = 2.0


def uuid16(u: str) -> str:
    """``FF01`` from ``ff01``, ``0xFF01`` or the full base-UUID form, which is
    how CoreBluetooth reports 16-bit UUIDs (``UUIDString`` gives ``FF01``)."""
    s = str(u).strip().upper().removeprefix("0X")
    if len(s) == 36 and s.startswith("0000") and s.endswith("-0000-1000-8000-00805F9B34FB"):
        return s[4:8]
    return s


_DELEGATE_CLASS = None


def load_corebluetooth() -> SimpleNamespace:
    """What the peripheral uses of CoreBluetooth, Foundation and libdispatch,
    plus a delegate factory; raises RuntimeError off macOS or without pyobjc."""
    global _DELEGATE_CLASS
    if sys.platform != "darwin":
        raise RuntimeError("corebluetooth is macOS' Bluetooth; on Linux use vhci or usb:N")
    try:
        import objc
        import CoreBluetooth as CB
        from Foundation import NSData, NSObject
        from libdispatch import DISPATCH_QUEUE_SERIAL, dispatch_queue_create
    except ImportError as exc:                           # pragma: no cover - depends on the install
        raise RuntimeError("the CoreBluetooth peripheral needs pyobjc: pip install 'marsemu[ble]'") from exc
    if _DELEGATE_CLASS is None:
        _DELEGATE_CLASS = make_delegate_class(NSObject, objc.protocolNamed("CBPeripheralManagerDelegate"))
    return SimpleNamespace(
        CBUUID=CB.CBUUID, CBMutableService=CB.CBMutableService,
        CBMutableCharacteristic=CB.CBMutableCharacteristic, CBPeripheralManager=CB.CBPeripheralManager,
        LocalNameKey=CB.CBAdvertisementDataLocalNameKey, NSData=NSData,
        new_queue=lambda: dispatch_queue_create(b"marsemu-ble", DISPATCH_QUEUE_SERIAL),
        new_delegate=lambda owner: _DELEGATE_CLASS.alloc().initWithOwner_(owner))


def make_delegate_class(base, protocol=None):
    """A CBPeripheralManagerDelegate that hands every callback to its owner
    (a :class:`CoreBluetoothPeripheral`). Callbacks come on CoreBluetooth's
    dispatch queue."""
    kw = {"protocols": [protocol]} if protocol is not None else {}

    class MarsemuPeripheralDelegate(base, **kw):
        def initWithOwner_(self, owner):                 # noqa: N802 - Objective-C selector
            self = self.init()
            if self is None:
                return None
            self.owner = owner
            return self

        def peripheralManagerDidUpdateState_(self, manager):           # noqa: N802
            self.owner.on_state(int(manager.state()))

        def peripheralManager_didAddService_error_(self, manager, service, error):   # noqa: N802
            self.owner.on_service_added(error)

        def peripheralManagerDidStartAdvertising_error_(self, manager, error):     # noqa: N802
            self.owner.on_advertising_started(error)

        def peripheralManager_central_didSubscribeToCharacteristic_(self, manager, central, char):  # noqa: N802
            self.owner.on_subscribe(central, char)

        def peripheralManager_central_didUnsubscribeFromCharacteristic_(self, manager, central, char):  # noqa: N802,E501
            self.owner.on_unsubscribe(central, char)

        def peripheralManager_didReceiveReadRequest_(self, manager, request):      # noqa: N802
            self.owner.on_read(request)

        def peripheralManager_didReceiveWriteRequests_(self, manager, requests):   # noqa: N802
            self.owner.on_writes(requests)

        def peripheralManagerIsReadyToUpdateSubscribers_(self, manager):            # noqa: N802
            self.owner.on_ready_to_update()

    return MarsemuPeripheralDelegate


class CoreBluetoothPeripheral:
    """Serves an emulator's Bluetooth through CoreBluetooth. ``cb`` replaces
    :func:`load_corebluetooth` (tests)."""

    def __init__(self, emulator, *, cb: Optional[SimpleNamespace] = None):
        self.module = emulator.fc41d
        self._cb = cb
        self.manager = None
        self.service = None
        self.chars: Dict[str, object] = {}
        self.centrals: Dict[str, object] = {}       # identifier -> CBCentral
        self.subscriptions: Dict[str, set] = {}     # identifier -> characteristic UUIDs
        self.state: Optional[int] = None
        self.advertising: Optional[str] = None      # the name advertised, while advertising
        self._lock = threading.RLock()
        self._powered = threading.Event()
        self._service_added = threading.Event()
        self._ready = threading.Event()
        self._jobs: "queue.Queue" = queue.Queue()
        self._worker = threading.Thread(target=self._work, name="marsemu-corebluetooth", daemon=True)
        self._said_no_drop = False
        self._warned_mtu = False

    # -------------------------------------------------------------- lifecycle
    def start(self, timeout: float = 10.0) -> "CoreBluetoothPeripheral":
        cb = self._cb = self._cb or load_corebluetooth()
        self._delegate = cb.new_delegate(self)
        self.manager = cb.CBPeripheralManager.alloc().initWithDelegate_queue_(self._delegate, cb.new_queue())
        if not self._powered.wait(timeout):
            st = self.state
            why = STATE_HELP.get(st, f"Bluetooth is {STATE_NAMES.get(st, 'not answering')}")
            raise RuntimeError(f"cannot use this Mac's Bluetooth: {why}")
        self._worker.start()
        for event, fn in (("ble_adv", self._on_module), ("ble_name", self._on_module),
                          ("ble_state", self._on_module_state), ("ble_notify", self._on_module_notify)):
            self.module.on(event, fn)
        self._job(self._sync)
        log.warning("BLE peripheral on this Mac's Bluetooth (CoreBluetooth)")
        return self

    def close(self) -> None:
        if self.manager is None:
            return
        self._jobs.put(None)
        self._worker.join(5)
        with self._lock:
            self.manager.stopAdvertising()
            self.manager.removeAllServices()
            self.advertising = None

    def _job(self, fn, *args) -> None:
        self._jobs.put((fn, args))

    def _work(self) -> None:
        while True:
            item = self._jobs.get()
            if item is None:
                return
            fn, args = item
            try:
                fn(*args)
            except Exception:                            # noqa: BLE001 - keep serving
                log.exception("BLE peripheral")

    # ------------------------------------------------ module -> CoreBluetooth
    def _on_module(self, *_):
        self._job(self._sync)

    def _on_module_state(self, state):
        if state == "disconnected" and self.centrals and not self._said_no_drop:
            self._said_no_drop = True
            log.warning("the firmware dropped the BLE connection; a CoreBluetooth peripheral cannot "
                        "disconnect a central, so it stays connected")

    def _on_module_notify(self, uuid, data):
        self._job(self._notify, uuid16(uuid), bytes(data))

    def _sync(self) -> None:
        """GATT database and advertising as the firmware has set them up."""
        m, cb = self.module, self._cb
        if m.ble_adv and self.service is None and m.ble_services and m.ble_chars:
            self._add_service(uuid16(m.ble_services[-1]), list(dict.fromkeys(uuid16(c) for c in m.ble_chars)))
        name = m.ble_name or "Quec_BLE"
        with self._lock:
            want = name if (m.ble_adv and self.service is not None and not self.centrals) else None
            if want == self.advertising:
                return
            if self.advertising is not None:
                self.manager.stopAdvertising()
                self.advertising = None
            if want is not None:
                self.manager.startAdvertising_({cb.LocalNameKey: want})
                self.advertising = want
                log.info("BLE advertising as %s", want)

    def _add_service(self, service: str, chars) -> None:
        cb = self._cb
        props = PROP_READ | PROP_WRITE | PROP_WRITE_NO_RESPONSE | PROP_NOTIFY
        built = []
        for u in chars:
            c = cb.CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
                cb.CBUUID.UUIDWithString_(u), props, None, PERM_READABLE | PERM_WRITEABLE)
            self.chars[u] = c
            built.append(c)
        svc = cb.CBMutableService.alloc().initWithType_primary_(cb.CBUUID.UUIDWithString_(service), True)
        svc.setCharacteristics_(built)
        self._service_added.clear()
        self.manager.addService_(svc)
        if not self._service_added.wait(5):
            log.error("macOS did not confirm the GATT service")
        self.service = svc
        log.info("BLE GATT: service %s, characteristics %s", service, ", ".join(chars))

    def _notify(self, uuid: str, data: bytes) -> None:
        char = self.chars.get(uuid) or self.chars.get("FF02")
        if char is None or not self.centrals:
            return
        with self._lock:
            limits = [int(c.maximumUpdateValueLength()) for c in self.centrals.values()]
        if limits and len(data) > min(limits) and not self._warned_mtu:
            self._warned_mtu = True
            log.warning("a %d-byte notification is longer than the central takes (%d bytes): it asked for "
                        "too small an MTU, macOS cuts it", len(data), min(limits))
        value = self._cb.NSData.dataWithBytes_length_(data, len(data))
        tries = 3
        while not self.manager.updateValue_forCharacteristic_onSubscribedCentrals_(value, char, None):
            # macOS' transmit queue is full; it says when there is room again
            self._ready.clear()
            tries -= 1
            if tries < 0 or not self._ready.wait(NOTIFY_WAIT_S):
                log.error("dropped a %d-byte notification: macOS did not take it", len(data))
                return

    # ------------------------------------------------ CoreBluetooth -> module
    def on_state(self, state: int) -> None:
        self.state = state
        log.info("Bluetooth %s", STATE_NAMES.get(state, state))
        if state == POWERED_ON:
            self._powered.set()
            if self.service is not None:               # back after Bluetooth was off: start over
                self.service, self.chars, self.advertising = None, {}, None
                self._job(self._sync)
        else:
            self._powered.clear()
            with self._lock:
                gone = bool(self.centrals)
                self.centrals.clear()
                self.subscriptions.clear()
                self.advertising = None
            if gone:
                self._central_gone()

    def on_service_added(self, error) -> None:
        if error is not None:
            log.error("macOS refused the GATT service: %s", error)
        self._service_added.set()

    def on_advertising_started(self, error) -> None:
        if error is not None:
            log.error("macOS did not start advertising: %s", error)
            with self._lock:
                self.advertising = None

    def _central(self, central) -> str:
        """Note ``central`` (a subscription or a write from it); the first
        central is the connection the firmware hears about. Returns its
        identifier."""
        ident = str(central.identifier().UUIDString())
        with self._lock:
            first = not self.centrals
            self.centrals[ident] = central
            self.subscriptions.setdefault(ident, set())
        if first:
            log.info("BLE central %s connected", ident)
            m = self.module
            m.post(m.ble_connect)
            self._job(self._sync)                      # one central at a time: stop advertising
        return ident

    def on_subscribe(self, central, char) -> None:
        ident = self._central(central)
        with self._lock:
            self.subscriptions[ident].add(uuid16(char.UUID().UUIDString()))

    def on_unsubscribe(self, central, char) -> None:
        ident = str(central.identifier().UUIDString())
        with self._lock:
            subs = self.subscriptions.get(ident)
            if subs is None:
                return
            subs.discard(uuid16(char.UUID().UUIDString()))
            if subs:
                return
            # no subscription left: CoreBluetooth's sign that it disconnected
            del self.subscriptions[ident]
            self.centrals.pop(ident, None)
            last = not self.centrals
        if last:
            log.info("BLE central %s disconnected", ident)
            self._central_gone()

    def _central_gone(self) -> None:
        m = self.module

        def gone():
            if m.ble_connected:
                m.ble_disconnect()
        m.post(gone)
        self._job(self._sync)                          # the FC41D advertises again

    def on_read(self, request) -> None:
        request.setValue_(self._cb.NSData.dataWithBytes_length_(b"", 0))
        self.manager.respondToRequest_withResult_(request, ATT_SUCCESS)

    def on_writes(self, requests) -> None:
        m = self.module
        for request in requests:
            self._central(request.central())
            uuid = uuid16(request.characteristic().UUID().UUIDString()).lower()
            value = request.value()
            data = bytes(value) if value is not None else b""
            m.post(lambda d=data, u=uuid: m.ble_write(d, u))
        if len(requests):
            self.manager.respondToRequest_withResult_(requests[0], ATT_SUCCESS)

    def on_ready_to_update(self) -> None:
        self._ready.set()


__all__ = ["CoreBluetoothPeripheral", "load_corebluetooth", "make_delegate_class", "uuid16"]
