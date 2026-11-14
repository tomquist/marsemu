"""marsemu: run Marstek/Hame firmware without the hardware.

See README.md.  One MCU, the FC41D module emulated::

    from marsemu import Emulator
    emu = Emulator("firmware.bin")
    emu.on("mqtt_publish", lambda topic, payload: print(topic, payload))
    emu.run(seconds=30)

A whole device in a house, steering against its CT::

    from marsemu import B2500, House, connect_house, configure_self_consumption
    b = B2500("b2500.bin", pv=[300, 300])
    house = House({"A": 400})
    connect_house(b, house)               # or meter="127.0.0.1:12345" for AstraMeter
    configure_self_consumption(b)
    b.run(120)
    print(b.status(), house.grid())
    b.close()
"""
__version__ = "0.1.0"

_EXPORTS = {
    "Emulator": ".emulator",
    "B2500": ".plant.b2500",
    "VenusE3": ".plant.venus3",
    "PowerStage": ".plant.venus3",
    "VenusD": ".plant.venusd",
    "VenusA": ".plant.venusa",
    "Jupiter": ".plant.jupiter",
    "VenusCE": ".plant.venusce",
    "LfpPack": ".plant.battery",
    "House": ".plant.venus3_cli",
    "CtMeter": ".plant.venus3_cli",
    "CtCallback": ".plant.venus3_cli",
    "CtForwarder": ".plant.venus3_cli",
    "connect_house": ".plant.venus3_cli",
    "run_together": ".plant.venus3_cli",
    "configure_self_consumption": ".plant.venus3_cli",
    "ModbusGateway": ".modbus",
    "UartBridge": ".uartbridge",
}
__all__ = sorted(_EXPORTS)


def __getattr__(name):
    mod = _EXPORTS.get(name)
    if mod is None:
        raise AttributeError(name)
    import importlib
    return getattr(importlib.import_module(mod, __name__), name)
