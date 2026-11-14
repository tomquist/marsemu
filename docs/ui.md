# Dashboard

`marsemu ui` runs a device and shows its inside in the browser, with live
data from the running firmware:

- **X-ray view.** The device drawn as marsemu models it, at three depths:
  - *Shell:* the closed enclosure.
  - *Boards:* the parts and their wiring, with power flowing between them and
    bus messages travelling along the wires.
  - *Silicon:* the registers the firmware writes, the cell voltages and the
    front end's status.

  A lens follows the pointer and shows one layer deeper (key `L`).
- **Inspector.** Click any part: what it is, whether it runs real firmware or
  is a model, its live values, its registers and its recent traffic.
- **Trace.** "Trace a CT poll" walks through one steering cycle: from the
  meter, through the Wi‑Fi module and the firmware's decision, to the outputs,
  the microinverter, the house and the pack (key `T`, arrows to step, `Esc`).
- **Telemetry and console.** Charts of the last five minutes and every bus
  message (AT commands, MQTT, CT polls, the extension bus, the front end),
  with filters.
- **Controls.** House load, sun on each PV input, ambient temperature,
  scenarios (load step; on the B2500 a passing cloud and solar surplus, on the
  Venus a house that exports), faults (Wi‑Fi down,
  CT002 offline, pack over-temperature), a status poll (`cd=1`), pause, step
  and a speed limit.

Every device marsemu models. Pick the image on the start screen; devices
with a BMS image of their own take it too.

| Device | Images | What the view shows |
|---|---|---|
| **B2500** | one (`b2500-v1`, `-v2`, `-v3`) | the one MCU, the SH367309 front end and 14 cells, both PV inputs and outputs, the microinverter, extension packs on UART5 |
| **Venus E 3.0** | control (`venus3`, VNSE3) + BMS | control and BMS firmware (the BMS in its own process), the KA495XX and 16 cells, the inverter board model, EEPROM, RS485 |
| **Venus D** | control (VNSD) + BMS | as the E 3.0, plus the MPPT model with four PV strings; stacked packs (`packs` on the start screen) |
| **Venus A** | control (VNSA) + BMS | the SH36735 front end and 13 cells, the inverter with its four PV inputs; stacked packs |
| **Jupiter C/E, C Plus** | control (`jupiter`), BMS optional | the inverter and MPPT models; the C Plus's real BMS firmware with its image, else the BMS model (Pylontech-style frames) |
| **Venus C/E** | control (`venus`, HMG) + BMS | the control MCU runs the power stage itself: the LLC, the DC bus, the bridge and the grid relay it drives, the BMS firmware on CAN. It runs at about a sixth of real time |
| **CT002 / CT003** | one (`ct00x`) | the meter's firmware answering virtual batteries that poll it over UDP (how many, and what power they report, are controls). The grid it measures is the house load, spread over the three phases, minus what each battery delivers: on a CT002 through a model of its metering chip, on a CT003 (a P1 reader) as the utility meter's telegrams on its P1 port |

On every CAN device the internal bus is decoded: setpoints and limits to
the inverter, its replies, the MPPT's status, and what the BMS reports
(voltage, current, SOC, limits, protection, cells, temperatures).

## Running it

```bash
marsemu ui B2500.bin                         # start this image; http://127.0.0.1:8080/
marsemu ui --firmware-dir ~/venus            # a Venus E 3.0: pick the control image and its BMS image
marsemu ui --firmware-dir ~/firmware         # pick an image (or drop one in) on the page
marsemu ui B2500.bin --mqtt broker --broker 127.0.0.1:1883   # its MQTT session to your broker
```

`--host 0.0.0.0` makes it reachable from other machines, and `--port` changes
the port. `--speed 0` runs as fast as the computer can; the default is real
time at most. `--meter HOST:PORT` sends the CT002 polls to AstraMeter instead
of the built-in meter. `marsemu ui --help` lists the rest.

With Docker, the firmware folder must be writable if you want to drop images
into the page:

```bash
docker run --rm -p 8080:8080 -v "$PWD/firmware:/firmware" ghcr.io/tomquist/marsemu:main ui --host 0.0.0.0
```

## Home Assistant add-on

1. In Home Assistant, open **Settings → Add-ons → Add-on store → ⋮ →
   Repositories**, and add `https://github.com/tomquist/marsemu`.
2. Install **marsemu** and start it. It appears as **marsemu** in the sidebar.
3. Drop your device's firmware image into the panel, or copy it into
   `/addon_configs/<id>_marsemu/firmware`.

With the Mosquitto broker add-on installed, or with MQTT set to `custom` and
your own broker's address, the emulated battery connects to that broker with
its real topics, so hm2mqtt sees it like a real
device. Set **House load sensor** to your household consumption sensor, and
the emulated house draws what your house draws. The add-on's documentation
tab describes every option.

The add-on builds marsemu from the repository's `main` branch when you
install it.

## Speed

A B2500 runs at about 3–4× real time on a desktop computer once it has
booted; its first 10 seconds are busier and take about 15 s. It is online
(Wi‑Fi, MQTT, CT002) after about 20 s of emulated time. A Raspberry Pi is
several times slower.

## API

Every URL is relative to the page, so the dashboard also works behind a path
prefix (Home Assistant's ingress).

| Method and path | What it does |
|---|---|
| `GET api/state` | The firmware images and, while a device runs, its latest snapshot |
| `GET api/stream` | Server-sent events: `hello` (the state) once, then `tick` four times a second with the snapshot, new bus messages and new chart samples |
| `POST api/control` | A command for the running device, e.g. `{"op": "load", "w": 800}`, `{"op": "pv", "w": [300, 200]}`, `{"op": "fault", "name": "wifi", "on": true}`, `{"op": "mqtt", "payload": "cd=1"}`, `{"op": "speed", "x": 0}`, `{"op": "pause", "on": true}`, `{"op": "step", "s": 1}`, `{"op": "scenario", "name": "cloud"}`; for the CT `{"op": "batteries", "n": 2}` and `{"op": "battery_w", "w": 300}` |
| `POST api/firmware?name=FILE` | Save the request body as a firmware image |
| `POST api/start` | `{"image": "FILE", "pv": [W, W], "load": W, "soc": %}`; devices with a BMS image take `"bms": "FILE"`, the Venus D and A `"packs": N`, the CT `"batteries": N` |
| `POST api/stop` | Stop the device |

From Python, the classes in `marsemu.ui.session` (`B2500Session`,
`VenusE3Session`, `VenusDSession`, `VenusASession`, `JupiterSession`,
`VenusCESession`, `CtSession`) are the same devices without the web server: `command()` takes the same commands and `read()` returns the
same snapshot.
