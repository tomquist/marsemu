"""A virtual Bluetooth LE peripheral: the emulated device as Bluetooth sees it.

The FC41D model's BLE side (the name the firmware sets, its GATT service,
advertising, writes and notifications) becomes a real BLE peripheral on a
Bluetooth controller, so that phones, browsers (Web Bluetooth) and the
Marstek app find and connect to the emulated device as to a real one. It is
built on Bumble, Google's Bluetooth stack in Python (``pip install
'marsemu[ble]'``), and runs where Bumble has a controller:

``vhci``
    A virtual adapter in the Linux kernel (``/dev/vhci``, root or write
    access to it). The host's own Bluetooth stack (BlueZ, so Chrome on Linux)
    gets a new adapter, ``hciN``, through which it sees the emulated device
    advertising; no radio is involved. ``vhci:/path`` names another device
    node.
``usb:N`` / ``serial:…`` / ``tcp-client:…`` / ``android-netsim`` / …
    Any other Bumble transport (https://google.github.io/bumble/transports/):
    a USB Bluetooth dongle the host's stack does not use (on air, for a phone
    nearby), an HCI UART, or the Android emulator's virtual Bluetooth, which
    its Chrome and apps then see.

What it serves follows the firmware's own setup over AT commands: the
service (``FF00``) and characteristics (``FF01``, ``FF02``, ``FF06``) it
registers, the name it sets, advertising when it starts it (flags and the
complete name, which is what the module sends when the firmware gives it no
advertising data, every 93.75 ms as the firmware asks). A central's writes
reach the firmware as the module's write events; its notifications go to
subscribed centrals. Connecting and disconnecting reach the firmware as the
module's status events; when the firmware drops the connection, the central
is disconnected. The ATT MTU goes up to 247, as on the FC41D.
"""
from __future__ import annotations

import asyncio
import logging
import threading
from typing import Dict, Optional

log = logging.getLogger("emu.bleperiph")

#: what the FC41D offers in an MTU exchange (AT+QBLECFGMTU?)
MTU = 247
#: advertising interval the firmwares set (AT+QBLEADVPARAM=150,150), in ms
ADV_INTERVAL_MS = 150 * 0.625


def static_address(mac: str) -> str:
    """The module's MAC as a static random address (top two bits set), which
    the peripheral advertises with: the Marstek MACs (C8:47:8C:…) already
    are one, so centrals see the device's own address."""
    b = bytes.fromhex(mac.replace(":", ""))
    return ":".join(f"{x:02X}" for x in bytes([b[0] | 0xC0]) + b[1:])


def _uuid16(u: str) -> str:
    return u.strip().upper().removeprefix("0X")


class BlePeripheral:
    """Serves an emulator's Bluetooth as a BLE peripheral on a Bumble
    controller. ``transport`` is ``"vhci"`` or a Bumble transport name; with
    ``link`` (a ``bumble.link.LocalLink``) the peripheral's controller is put
    on that link instead, which is how tests connect a Bumble central to it.
    """

    def __init__(self, emulator, transport: Optional[str] = "vhci", *, link=None):
        try:
            import bumble  # noqa: F401
        except ImportError as exc:                       # pragma: no cover - depends on the install
            raise RuntimeError("the BLE peripheral needs Bumble: pip install 'marsemu[ble]'") from exc
        self.module = emulator.fc41d
        self.transport_name = transport
        self.link = link
        self.loop = asyncio.new_event_loop()
        self.device = None
        self.connection = None
        self.chars: Dict[str, object] = {}
        self._transport = None
        self._sync_lock: Optional[asyncio.Lock] = None
        self._ready = threading.Event()
        self._error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name="marsemu-ble-peripheral", daemon=True)

    # -------------------------------------------------------------- lifecycle
    def start(self, timeout: float = 15.0) -> "BlePeripheral":
        """Open the controller and start following the module; raises if the
        transport cannot be opened."""
        self._thread.start()
        if not self._ready.wait(timeout):
            raise RuntimeError(f"Bluetooth controller {self.transport_name!r} did not come up")
        if self._error is not None:
            raise RuntimeError(f"cannot open Bluetooth on {self.transport_name!r}: {self._error}") from self._error
        m = self.module
        for event, fn in (("ble_adv", self._on_module), ("ble_name", self._on_module),
                          ("ble_state", self._on_state), ("ble_notify", self._on_notify)):
            m.on(event, fn)
        self._call(self._sync)
        return self

    def close(self) -> None:
        if not self._thread.is_alive():
            return

        async def stop():
            try:
                if self.connection is not None:
                    await self.connection.disconnect()
            except Exception:                            # noqa: BLE001 - best effort on the way out
                pass
            if self._transport is not None:
                await self._transport.close()
            self.loop.stop()
        asyncio.run_coroutine_threadsafe(stop(), self.loop)
        self._thread.join(5)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._open())
        except BaseException as exc:                     # noqa: BLE001 - reported by start()
            self._error = exc
            self._ready.set()
            return
        self._ready.set()
        self.loop.run_forever()

    async def _open(self) -> None:
        from bumble.controller import Controller
        from bumble.device import Device
        from bumble.hci import Address
        from bumble.host import Host
        from bumble.link import LocalLink
        from bumble.transport import open_transport
        from bumble.transport.common import AsyncPipeSink

        address = Address(static_address(self.module.cfg.ble_mac))
        name = self.module.ble_name or "Quec_BLE"
        link = self.link
        if link is None and self.transport_name.split(":", 1)[0] == "vhci":
            # a virtual adapter for the host's stack, and the device's own
            # controller, both on one in-memory link
            self._transport = await open_transport(self.transport_name)
            link = LocalLink()
            Controller("marsemu-adapter", host_source=self._transport.source,
                       host_sink=self._transport.sink, link=link)
        if link is not None:
            ctl = Controller("marsemu-device", link=link)
            self.device = Device(name=name, address=address, host=Host(ctl, AsyncPipeSink(ctl)))
        else:
            self._transport = await open_transport(self.transport_name)
            self.device = Device.with_hci(name, address, self._transport.source, self._transport.sink)
        self.device.gatt_server.max_mtu = MTU
        self.device.on(self.device.EVENT_CONNECTION, self._on_connection)
        await self.device.power_on()
        log.warning("BLE peripheral on %s, address %s", self.transport_name or "a local link", address)

    # ------------------------------------------------ module -> peripheral
    def _call(self, coro_fn, *args) -> None:
        """Run ``coro_fn(*args)`` on the Bluetooth thread (from any thread)."""
        def go():
            task = self.loop.create_task(coro_fn(*args))
            task.add_done_callback(self._report)
        self.loop.call_soon_threadsafe(go)

    @staticmethod
    def _report(task) -> None:
        if not task.cancelled() and task.exception() is not None:
            log.error("BLE peripheral: %s", task.exception())

    def _on_module(self, *_):
        self._call(self._sync)

    def _on_state(self, state):
        if state == "disconnected":
            self._call(self._drop)

    def _on_notify(self, uuid, data):
        self._call(self._notify, _uuid16(uuid), bytes(data))

    async def _sync(self) -> None:
        """Bring the GATT database, name and advertising in line with what
        the firmware has set up."""
        if self._sync_lock is None:
            self._sync_lock = asyncio.Lock()
        async with self._sync_lock:
            await self._sync_locked()

    async def _sync_locked(self) -> None:
        from bumble.core import AdvertisingData
        m, d = self.module, self.device
        # the firmware registers its service and characteristics before it
        # starts advertising, and the same ones again after a module reset
        if m.ble_adv and not self.chars and m.ble_services and m.ble_chars:
            self._build_gatt(_uuid16(m.ble_services[-1]),
                             list(dict.fromkeys(_uuid16(c) for c in m.ble_chars)))
        name = m.ble_name or "Quec_BLE"
        d.name = name
        want = bool(m.ble_adv) and self.connection is None
        if d.is_advertising:
            await d.stop_advertising()
        if want:
            adv = bytes(AdvertisingData([(AdvertisingData.FLAGS, bytes([0x06])),
                                         (AdvertisingData.COMPLETE_LOCAL_NAME, name.encode())]))
            await d.start_advertising(auto_restart=False,
                                      advertising_data=adv, advertising_interval_min=ADV_INTERVAL_MS,
                                      advertising_interval_max=ADV_INTERVAL_MS)
            log.info("BLE advertising as %s", name)

    def _build_gatt(self, service: str, chars) -> None:
        from bumble.gatt import Characteristic, CharacteristicValue, Service
        P = Characteristic.Properties
        module = self.module
        built = []
        for uuid in chars:
            def write(connection, value, uuid=uuid):
                data = bytes(value)
                module.post(lambda: module.ble_write(data, uuid.lower()))
            c = Characteristic(uuid, P.READ | P.WRITE | P.WRITE_WITHOUT_RESPONSE | P.NOTIFY,
                               Characteristic.READABLE | Characteristic.WRITEABLE,
                               CharacteristicValue(read=lambda connection: b"", write=write))
            self.chars[uuid] = c
            built.append(c)
        self.device.gatt_server.add_service(Service(service, built))
        log.info("BLE GATT: service %s, characteristics %s", service, ", ".join(chars))

    async def _notify(self, uuid: str, data: bytes) -> None:
        c = self.chars.get(uuid) or self.chars.get("FF02")
        if c is not None and self.connection is not None:
            await self.device.notify_subscribers(c, data)

    async def _drop(self) -> None:
        if self.connection is not None:
            await self.connection.disconnect()

    # ------------------------------------------------ peripheral -> module
    def _on_connection(self, connection) -> None:
        self.connection = connection
        log.info("BLE central %s connected", connection.peer_address)
        m = self.module
        m.post(m.ble_connect)
        connection.on(connection.EVENT_DISCONNECTION, lambda reason: self._on_disconnection(connection))

    def _on_disconnection(self, connection) -> None:
        if self.connection is connection:
            self.connection = None
        log.info("BLE central disconnected")
        m = self.module

        def gone():
            if m.ble_connected:
                m.ble_disconnect()
        m.post(gone)
        # the FC41D advertises again once the central is gone
        self.loop.create_task(self._sync())


def start_ble_peripheral(emulator, transport: str) -> BlePeripheral:
    """``--ble-peripheral TRANSPORT``: serve the emulator's Bluetooth on a
    Bumble controller; returns the running peripheral."""
    try:
        p = BlePeripheral(emulator, transport).start()
    except RuntimeError as exc:
        hint = (" (a virtual adapter needs the hci_vhci kernel module, `sudo modprobe hci_vhci`, and write"
                " access to /dev/vhci)") if transport.split(":", 1)[0] == "vhci" else ""
        raise SystemExit(f"marsemu: {exc}{hint}") from exc
    print(f"BLE peripheral on {transport}: the device advertises there once its firmware starts advertising",
          flush=True)
    return p


__all__ = ["BlePeripheral", "start_ble_peripheral"]
