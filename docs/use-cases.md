# What you can do with marsemu

Examples by task, with the options that do it. `CONTROL.bin`, `BMS.bin` and
`FIRMWARE.bin` stand for your device's images.

## Integrations without touching the real device

* *Develop or test an MQTT integration* (hm2mqtt, hame-relay, your own) against
  your device's firmware, or the version it is about to get: the device talks
  to your broker with its real topics and payloads.
  ```bash
  marsemu venus3 CONTROL.bin BMS.bin --mqtt broker --broker 127.0.0.1:1883 --realtime
  ```
* *Point a Home Assistant Modbus integration at a Venus*: the real firmware's
  register map, writes included (force charge, RS485 control mode, …).
  ```bash
  marsemu venusd CONTROL.bin BMS.bin --modbus-port 5020 --realtime
  ```
* *Run an EMS or CT emulator against the real control loop*: AstraMeter (or
  anything that speaks CT002) answers the battery's CT polls, and the battery
  reacts as the firmware decides, not as a simulator guesses.
  ```bash
  marsemu b2500 FIRMWARE.bin --meter 127.0.0.1:12345 --load 700 --realtime
  ```
* *Test BLE tools* through a TCP socket that stands in for the phone
  ([example with output](interfaces.md#ble)).
  ```bash
  marsemu b2500 FIRMWARE.bin --pv 300,300 --load 200 --realtime --ble-port 7001
  ```
* *Exercise a Venus D/A stack* with several packs and look at what the app,
  MQTT and Modbus report for each pack (`--packs 3`).

## Scenarios that are hard to set up on a real device

* *Load steps, solar export, PV changes* while it runs: `POST /load` and
  `POST /pv` on the HTTP house, or a script:
  ```python
  house.loads["A"] = -600          # export: the battery should charge
  v.run(60); print(v.status(), house.grid())
  ```
* *Edge cases*: start empty or full (`--soc 3`, `--soc 99`), cold (`--temp -5`),
  lose Wi‑Fi (`wifi-down` in `-i`), press buttons (`--gpio B14=0`), fake a
  sensor (`--adc ADC1:3=1800`), answer cloud requests your way
  (`--http-rule 'getDeviceVersion=200:{...}'`).
* *Upgrade paths*: run an old image, then a new one on the same `--state-dir`,
  and see what survives in flash and EEPROM.
* *Firmware regressions*: run two versions through the same scenario with
  `--json-log` and diff what they publish, or put them in a CI job with the
  Docker image.

## Reverse engineering

* *What does a field mean?* Change one input (an ADC channel, a cell
  voltage, a BMS frame) and watch which field of the `cd=1` report moves.
* *Breakpoints and hooks*: `--gdb 3333` for GDB, Ghidra or IDA;
  `machine.add_code_hook(addr, fn)` from Python.
* *What does the firmware touch?* `--trace-mmio FILE` logs every peripheral
  access with register names, `--coverage FILE` the executed code, `--at` the
  full AT conversation with the Wi‑Fi module, `--json-log` every MQTT, BLE and
  HTTP exchange.
* *Look at the wires between MCUs*: CAN frames between the Venus control, BMS
  and inverter, or a serial port served on TCP (`--serve-uart UART5:7005`,
  then `marsemu decode-bus 127.0.0.1:7005` for the B2500 extension bus).
* *Play the other side*: answer a UART or CAN bus from Python as a device
  that does not exist (an extension pack, a BMS, an inverter).
