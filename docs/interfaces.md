# Interfaces

How to reach the emulated device from other software. The whole-device
commands (`venus3`, `venusd`, `venusa`, `venusmini`, `jupiter`, `b2500`) and `run` take
most of these options; `marsemu COMMAND --help` lists them.

## MQTT broker (hm2mqtt, hame-relay, mosquitto)

```bash
mosquitto -p 1883 &
marsemu run FIRMWARE.bin --mqtt broker --broker 127.0.0.1:1883 --realtime
mosquitto_sub -t '#' -v                                                  # watch the device
mosquitto_pub -t 'marstek_energy/HMA-1/App/<id>/ctrl' -m 'cd=1'          # poll it
```

MQTT sessions go to the broker with their original topics and client IDs, so
tools that talk to the cloud topics work unchanged. `run --mqtt cloud`
connects to the vendor's broker the firmware asked for, with the certificates
it uploaded, under the identity you give with `--uid`, `--wifi-mac` and
`--ble-mac`. It only starts with all three and `--confirm-own-device`, which
confirms they are those of a device you own, registered to your own account;
posing as anyone else's device is unauthorised access.

## BLE

A TCP client on `--ble-port` acts as the phone: what it sends are GATT writes,
notifications come back as raw bytes. A whole B2500 (300 W on each PV input,
200 W house load), asked for its runtime info:

```bash
marsemu b2500 HMA-1/V232/*.bin --pv 300,300 --load 200 --realtime --ble-port 7001 &
sleep 150; printf '\x73\x06\x23\x03\x01\x54' | nc -q1 127.0.0.1 7001 | xxd
```

```text
00000000: 7310 2303 0101 2b01 2b01 f401 e800 0103  s.#...+.+.......
00000010: 0101 64d0 0000 6004 6800 6800 0000 0011  ..d...`.h.h.....
00000020: 1619 0019 0000 0002 0d00 0000 0000 0000  ................
00000030: 1500 0000 0700 0000 6c00 0001 00         ........l....
```

In the layout [esphome-b2500](https://github.com/tomquist/esphome-b2500) decodes
(`RuntimeInfoPacket`, after the 4-byte header `73 10 23 03`): both PV inputs
active at `2b01` = 299 W each, SOC `f401` = 50.0 %, firmware version `e8` = 232,
DoD `64` = 100 %, `6004` = 1120 Wh remaining, both outputs on at `6800` = 104 W,
which is what the plant shows at that moment (`/status`: PV 299.9 W per input,
outputs 104.2 W each, grid 1 W). `run` takes `--ble-port` too, but without a
power-stage model the ADCs read mid-scale and the numbers mean nothing.

### Over WebSocket

`--ble-ws [HOST:]PORT` (on `run` and every whole-device command) serves the
same link over WebSocket: each binary message the client sends is one write
to FF01, and each notification on FF02 comes back as one binary message. A
TCP stream cannot keep notifications apart when a reply's length byte is
wrong, as some B2500 replies' are; WebSocket messages can. Text messages
carry JSON about the device, when the client connects and whenever it
changes:

```json
{"type": "device", "name": "MST_VNSE3_0002", "advertising": true, "mac": "c8:47:8c:00:00:02", "services": ["ff00"]}
```

One client is connected at a time, as on the device: a new one replaces the
previous one. This is what hmjs' emulator transport speaks, so its demo
works with the emulated device as with a real one, with no Bluetooth
adapter:

```bash
marsemu venus3 VNSE3-0/Control/151/*.bin VNSE3-0/BMS/115/*.bin --realtime --ble-ws 7101
# then open https://tomquist.github.io/hmjs/?emulator=ws://127.0.0.1:7101/
```

Browsers let an HTTPS page open `ws://localhost`, so the hosted demo
reaches a local emulator. The dashboard serves the running device's
Bluetooth the same way at `api/ble`, and its **Open in hmjs** button opens
the demo connected to it. Writes in the first second (emulated) after a
central connects are held until then, as a phone takes about that long
(service discovery, notifications) before it writes; the firmware drops a
write that arrives too soon after the connection event. While it brings up
Wi‑Fi and MQTT after booting it also drops some writes, as the device does;
hmjs sends every command twice for that reason.

### As a Bluetooth device

`--ble-peripheral TRANSPORT` makes the emulated device a real Bluetooth LE
peripheral, which the Marstek app, a browser's device picker or any other
BLE tool finds and connects to as to the device. It needs Bumble, Google's
Bluetooth stack in Python: `pip install 'marsemu[ble]'`. It serves what the
firmware sets up: its name, service `FF00` with `FF01`, `FF02` and `FF06`,
advertising once the firmware starts it, an MTU of up to 247. The transport
decides who sees it:

| `TRANSPORT` | Who sees the device |
|---|---|
| `vhci` | This Linux host: a virtual adapter appears (`hciN`, `sudo modprobe hci_vhci` and write access to `/dev/vhci`), and BlueZ, so Chrome and `bluetoothctl`, sees the device through it. No radio |
| `corebluetooth` | Everything in radio range, through the Mac's own Bluetooth (below) |
| `usb:0` | Everything in radio range, through a USB Bluetooth dongle that the host's own stack does not use (on Linux, `sudo hciconfig hciN down` first) |
| `android-netsim` | The Android emulator: its Chrome and apps, the Marstek app among them |

Any other [Bumble transport](https://google.github.io/bumble/transports/)
works too (`serial:…` for an HCI UART controller, `tcp-client:…`).

```bash
sudo modprobe hci_vhci
marsemu venus3 VNSE3-0/Control/151/*.bin VNSE3-0/BMS/115/*.bin --realtime --ble-peripheral vhci
bluetoothctl scan le      # [NEW] Device C8:47:8C:00:00:02 MST_VNSE3_0002
```

Chrome uses the system's default adapter, so with a real adapter present,
power it off (`bluetoothctl power off` after `select` on it) or use a
machine without one. The device advertises with its own MAC as a static
random address.

#### On a Mac

`--ble-peripheral corebluetooth` uses the Mac's built-in Bluetooth through
macOS' CoreBluetooth (pyobjc, which `marsemu[ble]` installs on a Mac). A
phone nearby, with the Marstek app or Chrome, finds the emulated device by
its name. Run marsemu natively (Docker on a Mac has no Bluetooth), and allow
the terminal to use Bluetooth when macOS asks (System Settings → Privacy &
Security → Bluetooth). macOS keeps some things to itself:

- The advertisement holds the device's name only; macOS chooses the address
  (not the device's MAC) and the interval. After connecting, a central
  reading the GAP device name gets the Mac's name.
- A central counts as connected from its first subscription or write, and
  as gone when it has no subscription left (CoreBluetooth reports no
  connections to a peripheral). The firmware cannot drop a central
  (`AT+QBLEDISCONN` is logged).
- A notification longer than the central's MTU allows is cut by macOS; the
  Venus' longest reply needs an MTU of 147, which phones and Chrome ask for.
- The Mac cannot connect to itself; Chrome on the same Mac does not see it.
  A USB dongle (`usb:0`) works for that: the Mac's own Bluetooth then sees
  the dongle's device.

## Interactive and GDB

`-i` reads commands from stdin while the device runs:

```text
mqtt + cd=1                 deliver to the firmware's first subscription
ble 730623030154            BLE write
wifi-down / wifi-up         lose / regain the access point
gpio B14 0                  drive an input (a button)
adc ADC1 3 1800             change a sensor reading
at                          last AT transcript lines
status
```

```bash
marsemu run FIRMWARE.bin --gdb 3333 --no-watchdog
gdb-multiarch -ex 'set architecture armv7e-m' -ex 'target remote :3333'
```

The CPU halts at reset. Breakpoints, watchpoints, stepping, register and memory
access and Ctrl‑C work; `monitor time`, `monitor at` and `monitor reset` are
available. Peripheral registers read through the models. Ghidra and IDA can
attach as GDB clients too.

## HTTP house

The whole-device commands serve the house on `--http-port` (8090):

* `GET /power` → `{"phase_a": …, "phase_b": …, "phase_c": …, "total": …}`:
  grid power = house load − what the battery actually delivers
* `POST /load` with `{"A": 1500}` changes the load (negative = solar export)
* `POST /pv` with `{"pv": [W, …]}` changes the available PV power
* `GET /status` → the device's state (AC/DC power, SOC, pack, BMS limits,
  MOSFETs, CT polls)

## Modbus (Home Assistant and other clients)

The Venus E 3.0, D and A control firmwares contain a complete Modbus RTU slave
on their RS485 port (slave address 1). `--modbus-port` serves it as
**Modbus TCP**, `--modbus-rtu-port` as **RTU over TCP**, both against the real
firmware's register map, writes included:

```bash
marsemu venus3 VNSE3-0/Control/151/*.bin VNSE3-0/BMS/115/*.bin --modbus-port 5020 --realtime
```

```yaml
# Home Assistant configuration.yaml (or point a Venus Modbus integration at the same host/port)
modbus:
  - name: venus_emulated
    type: tcp                 # rtuovertcp with --modbus-rtu-port
    host: 127.0.0.1
    port: 5020
    sensors:
      - name: Venus SOC
        slave: 1
        address: 32104
        input_type: holding
        unit_of_measurement: "%"
      - name: Venus battery power
        slave: 1
        address: 30001
        input_type: holding
        data_type: int16
        unit_of_measurement: W
```

Requests reach the firmware on the emulator's clock at line speed; an address
nobody answers within 1 s of emulated time gets exception 0x0B, as from a real
RS485 gateway. From Python: `ModbusGateway(device.control, port=5020)`. Not
covered: the Venus D's Ethernet Modbus (CH395) and the Venus C/E.

## Serial ports (B2500 extension bus)

Any UART can be served on a TCP port, raw bytes both ways, like ser2net. On the
B2500, UART5 is the extension-pack bus (`0x73` frames: polls for packs 1/2 and
a status broadcast of the main battery); `marsemu.b2500bus` decodes it, and a
running device shows the decoded status under `bus` in `/status`:

```bash
marsemu b2500 FIRMWARE.bin --serve-uart UART5:7005 &
marsemu decode-bus 127.0.0.1:7005          # also works against a real B2500 behind ser2net
```

```json
{"type": "status", "direction": "charging", "cells_mv": [3307, …], "pack_mv": 46298, "current_ma": 7347,
 "power_w": 336, "temps_c": [25, 25, 25, 25, 25], "state_bits": 131, "soc": 50, "kn_wh": 1792, "device_id": "…"}
```

## A real Wi-Fi module

`--module-serial PORT` connects the FC41D UART to a serial port instead of
the module model: the firmware then talks to real hardware, for example a
Quectel FC41D on the FC41D-TE-B eval kit (stock, or a replacement firmware
such as ESPHome's `hame_at`) through its USB serial adapter. Wi-Fi, MQTT,
HTTP, sockets and BLE are then the module's; `--mqtt`, `--http`, `--meter`
and the automatic self-consumption setup no longer apply.

```bash
pip install pyserial
marsemu b2500 FIRMWARE.bin --realtime --pv 300,300 --module-serial /dev/ttyUSB0
marsemu run FIRMWARE.bin --realtime --at --module-serial socket://192.168.1.20:7000   # ser2net
```

Use `--realtime`, so the firmware's timeouts run on the module's clock (the
emulator may still run slower than real time; the firmware then simply sees
a fast module, and after a stall it carries on rather than racing to catch
up). `PORT@BAUD` sets another baud rate than the FC41D's 115200. `--at`
prints the conversation in both directions; the module's lines show how long
after the MCU's last line they arrived, in wall-clock milliseconds
(`OK  (+12 ms)`), which is the module's real response time. The firmware uploads its
certificates and connects to the cloud through the module: keep the module
off the real cloud (or use your own device's identity).

## AstraMeter

`--meter HOST:PORT` sends the device's CT polls to a CT002 emulator such as
AstraMeter instead of the built-in meter, which closes the loop without
changing AstraMeter:

```ini
# AstraMeter config.ini
[GENERAL]
DEVICE_TYPE = ct002

[CT002]
UDP_PORT = 12345
ACTIVE_CONTROL = True

[JSON_HTTP]
URL = http://127.0.0.1:8090/power
JSON_PATHS = $.phase_a,$.phase_b,$.phase_c
```

```bash
marsemu venus3 CONTROL.bin BMS.bin --realtime --meter 127.0.0.1:12345 --load 700
astrameter -c config.ini
```

AstraMeter reads the house from `/power`, the device polls AstraMeter for the
grid reading, and what it does with it is the real firmware's decision. From
Docker, use `--network host` or the host's address.

In-process, without sockets or wall-clock timing, `meter=` also takes a
function that gets each poll and returns the reply, called in the device's
emulated time. AstraMeter's CT002 can answer it directly, which makes a
deterministic, faster-than-real-time test of AstraMeter against the real
firmware:

```python
import asyncio
from astrameter.ct002.ct002 import CT002

ct = CT002(ct_mac="112233445566", active_control=True, clock=lambda: 1.7e9 + dev.time)

async def grid(addr, request, consumer_id):
    g = house.grid()
    return [g["A"], g["B"], g["C"]]

ct.before_send = grid
loop = asyncio.new_event_loop()

class Sink:
    def sendto(self, data, addr):
        self.reply = data

def answer(data, addr):
    sink = Sink()
    loop.run_until_complete(ct._handle_request(data, addr, sink))
    return getattr(sink, "reply", None)

connect_house(dev, house, meter=answer)
```

`tools/compare_astrameter.py` runs AstraMeter's battery simulator and the real
firmware through the same house-load profile and compares them:

```bash
python tools/compare_astrameter.py emulator --device venus3 emu.json CONTROL.bin BMS.bin
/path/to/AstraMeter/.venv/bin/python tools/compare_astrameter.py astrameter --device venus3 astra.json \
    --poll 1.23 --ramp 1000 --startup-delay 3 --soc 53.5
python tools/compare_astrameter.py compare emu.json astra.json
```

With the plant's timing matched, the Venus E 3.0, D and A agree with
AstraMeter's steering model on every step, the charge from an export and the
800 W cap. The main difference is just after phase detection: the firmware
steers on the next poll with the reading taken during the probe and swings to
the opposite of the probe for 2–6 s, which the model does later and shorter.

### The meter side: the CT002 and CT003 firmware against AstraMeter's CT002

The meters' firmware sees whatever grid you give it through `emu.ct_meter`.
Both models take `emu.ct_meter.set_power(a, b, c)`, in watts per phase.

* **CT002 (HME-4)** measures with clamps and a Belling BL09xx metering chip.
  Its firmware drives the chip over bit-banged SPI (`marsemu/periph/bl_spi.py`).
  * Phase power is registers 0x22–0x24, at 134.193 counts per watt.
  * The firmware cuts each phase to whole watts and adds the unrounded phases
    for the total.
  * The RMS currents, voltages and energy counters only reach its cloud report
    (every 300 s). They are not mapped yet.
* **CT003 (HME-3)** is a P1 reader: it parses the utility meter's DSMR
  telegrams (`marsemu/periph/p1.py`).
  * It waits for its P1 line (PB11, USART3's receive pin) to idle high for 2 s,
    which means a meter is connected.
  * Then it reads USART3 at 115200 baud, from `/` to `!`.
  * It picks out total and per-phase import and export power (`1-0:1.7.0`,
    `2.7.0`, `21.7.0`/`22.7.0`, `41.7.0`/`42.7.0`, `61.7.0`/`62.7.0`) and the
    four tariff energy registers (`1.8.1`, `1.8.2`, `2.8.1`, `2.8.2`). The
    energy registers go into reply fields 25–28.
  * The model sends a DSMR 5 telegram every second with the power you set and
    the energy it adds up to. `set_connected(False)` unplugs it.

`tools/compare_ct002.py` puts AstraMeter's `CT002` (with `active_control`
off, so it reports and doesn't steer) next to the firmware in one process, on
the emulator's clock. It sends both the same polls from three batteries over a
280 s scenario: grid steps, imports and exports, a battery changing phase, an
inspection-mode reporter, the combined `D` phase, and a battery that goes
quiet for 40 s. Then it compares every reply field:

```bash
python tools/compare_ct002.py CT002.bin --astrameter /path/to/AstraMeter/src [--json polls.json]
```

Against CT002 V124 and AstraMeter at the time of writing, all 589 replies agree
on every field (phase and total power, batteries per phase, charge and
discharge sums, the energy fields) except:

* **Reply timing.** The firmware answers 0.5–1.5 s after a poll (median
  0.7 s), with what it knows by then, while AstraMeter answers at once. A grid
  step, or another battery's new report, inside that window shows up in one
  answer and not the other. The tool counts these as transition.
* **A battery that goes quiet.** The firmware still counted the silent battery,
  with its last reported power, when it came back 40 s later. AstraMeter's
  adaptive eviction drops it after about two of its poll periods.

Against CT003 V122, the same 589 polls agree on the same fields, with the same
two differences. The CT003's reply also has 28 fields, not 24. Fields 25–28
are the P1 meter's tariff energy registers (`1.8.1`, `1.8.2`, `2.8.1`,
`2.8.2`), in units of 0.1 Wh: 1234.567 kWh is sent as 12345670. AstraMeter
leaves these four fields empty, because it has no smart meter to read them
from.
