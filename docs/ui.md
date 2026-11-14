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

Two devices for now:

- **B2500** (every image `marsemu info` reports as `b2500-v1`, `-v2` or `-v3`):
  the one MCU, the SH367309 front end and 14 cells, both PV inputs, both
  outputs and the microinverter, extension packs on UART5.
- **Venus E 3.0** (a `venus3` control image for the E 3.0, plus its BMS
  image): the control and the BMS firmware (the BMS in its own process), the
  KA495XX front end and 16 cells, and the inverter board model. The internal
  CAN bus is decoded: setpoints and limits to the inverter board, its
  replies, and the BMS's voltage, current, SOC, limits, protection, cells and
  temperatures.

The other devices follow.

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
| `POST api/control` | A command for the running device, e.g. `{"op": "load", "w": 800}`, `{"op": "pv", "w": [300, 200]}`, `{"op": "fault", "name": "wifi", "on": true}`, `{"op": "mqtt", "payload": "cd=1"}`, `{"op": "speed", "x": 0}`, `{"op": "pause", "on": true}`, `{"op": "step", "s": 1}`, `{"op": "scenario", "name": "cloud"}` |
| `POST api/firmware?name=FILE` | Save the request body as a firmware image |
| `POST api/start` | `{"image": "FILE", "pv": [W, W], "load": W, "soc": %}`; a Venus also takes `"bms": "FILE"` |
| `POST api/stop` | Stop the device |

From Python, `marsemu.ui.session.B2500Session` and `VenusE3Session` are the
same devices without the web server: `command()` takes the same commands and `read()` returns the
same snapshot.
