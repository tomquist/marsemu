# Python API

Everything the command line does is available from Python: whole devices in a
house, several devices on one grid, single MCUs with hooks, and your own board
models on the peripheral buses. Times are virtual seconds, so scripts run
faster than real time and give the same result every run.

## A whole device

```python
from marsemu import B2500, House, connect_house, configure_self_consumption

b = B2500("firmwares/HMJ-2/116/116_HMJ-2_202508051050536604e3965.bin", pv=[300, 300])
house = House({"A": 400})
connect_house(b, house)                 # built-in CT; meter="127.0.0.1:12345" forwards to AstraMeter,
                                        # meter=fn answers in-process (see interfaces.md#astrameter)
configure_self_consumption(b)           # what the app would send once the device is online
b.run(120)                              # virtual seconds, faster than real time
print(b.status(), house.grid())
house.loads["A"] = 800                  # a load step
b.run(30)
b.close()
```

`VenusE3`, `VenusD`, `VenusA`, `VenusCE` (control + BMS image), `VenusMini` (control image) and `Jupiter`
work the same way.

## Several devices in one house

Give each device its phase and run them together, so
they share one emulated timeline (lockstep slices of a few milliseconds) and
see one grid. Two devices built from the same image need their own identity
(MCU UID, the module's Wi-Fi and BLE MACs, and IP), or the meter cannot tell
them apart. A Venus names itself in its CT polls by the BLE MAC, so two Venus
left on the default BLE MAC look like one battery to the meter:

```python
from marsemu import VenusD, VenusE3, House, connect_house, run_together
from marsemu.fc41d.module import Fc41dConfig

def ident(n):
    return {"uid": bytes([n] * 12),
            "fc41d": Fc41dConfig(wifi_mac=f"c8:47:8c:00:{n:02x}:01", ble_mac=f"c8:47:8c:00:{n:02x}:02",
                                 ip=f"192.168.1.{50 + n}")}

d = VenusD(D_CONTROL, D_BMS, pv=[750, 750], control_kwargs=ident(1))
e = VenusE3(E_CONTROL, E_BMS, control_kwargs=ident(2))
house = House({"A": -800, "B": 300})
connect_house(d, house, meter=answer, phase="A")   # one CT002 function answers both,
connect_house(e, house, meter=answer, phase="B")   # called with each battery's address
run_together([d, e], 300)
print(house.grid(), house.batteries)
```

With `meter=fn` the function gets each poll with the polling battery's
address, as a CT002 on the LAN would; `meter="local"` gives each device a CT
of its own that only counts that device.

## A single MCU

`Emulator` runs a single MCU:

```python
from marsemu import Emulator

emu = Emulator("HMN-1_app_0142.bin", state_dir="state/jupiter")
emu.on("mqtt_publish", lambda topic, payload: print(topic, payload))
emu.on("at", lambda line: None)                    # every AT command
emu.machine.add_code_hook(0x0800E8D8, lambda m: print("send_at", m.cstr(m.reg("r0"))))
emu.run(until=lambda: emu.fc41d.mqtt and any(s.connected for s in emu.fc41d.mqtt.values()))
emu.mqtt_inject("marstek_energy/HMN-1/App/<id>/ctrl", "cd=1")
emu.run(seconds=2)
print(emu.fc41d.dump_transcript(20))
```

Useful objects: `emu.machine` (CPU, memory, `clock`, `nvic`, `bus` with every
peripheral by name), `emu.fc41d` (module state, `transcript`, `certs`, `mqtt`
sessions), `emu.consoles` (text printed on each UART).

## Extending: board models

Board models plug into the peripheral models without touching the core:

```python
from marsemu.periph.n32g45x import UartDevice, RegI2cDevice

class Bms(UartDevice):
    def on_tx(self, data):            # frame from the MCU
        self.uart.inject(reply_for(data))

emu.machine.bus.get("UART5").device = bms = Bms(); bms.attach(emu.machine.bus.get("UART5"))
emu.machine.bus.get("I2C1").devices[0x50] = RegI2cDevice(0x50, size=8192)   # EEPROM
emu.machine.bus.get("ADC1").values[3] = 2900                                   # a voltage
emu.machine.bus.get("GPIOB").set_input(14, 0)                                  # a button
```

Profiles (`profiles.py`) collect such settings per device family, including
factory data (e.g. the device-type index the B2500 derives its MQTT topics
from). A B2500 v2 build accepts every type in its table (HMA, HMJ, HMK, ...), so
the image alone doesn't say which unit it ran on: builds for the HMJ board
(recognised by their ADC wiring) get HMJ-2, the others HMA-1, and
`--device-type` picks another. `--state-dir` keeps flash and module state between runs, so a device can
be "upgraded" by running an older and then a newer image on the same state.
`plant/cosim.py` (lockstep timeline, CAN delivery at exact arrival times),
`plant/canbus.py` and `plant/remote.py` (an MCU in its own process) are
generic, so other multi-MCU devices can be assembled the same way.
