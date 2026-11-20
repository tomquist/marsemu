# How marsemu works

The emulator under the device models: the CPU core and exceptions, the
peripheral models, the FC41D Wi‑Fi/BLE module and what the network looks like
from the firmware's side.

## CPU and exceptions

Unicorn provides QEMU's Cortex-M4 core but none of the system devices, so the
NVIC/SCB/SysTick/DWT/ITM live in `core/nvic.py` and exception entry and return
in `core/machine.py`: basic and FPU frames, MSP/PSP, EXC_RETURN, tail-chaining,
priorities with PRIGROUP/BASEPRI/PRIMASK, SVC, faults. FreeRTOS, CMSIS-RTOS2,
uC/OS-III (Venus E Mini) and bare-metal loops run unmodified. Details that matter:

* **Preemption is immediate.** `PENDSVSET` or unmasking BASEPRI/PRIMASK enters
  the pending handler before the next instruction, or FreeRTOS corrupts its
  lists. Such writes stop emulation right after the store, and
  `msr basepri/primask`/`cpsie i` are Unicorn exits (free until hit).
* **Virtual time is exact.** Time is executed instructions × CPI (2 by default,
  1 on the Venus E 3.0/A/D control, as on its cached flash). `core/ucount.py`
  finds Unicorn's instruction counter with a self-calibrating probe. WFI and the
  FreeRTOS or uC/OS-III idle loop (`analysis/idle.py`) fast-forward to the next event, which
  makes mostly idle firmware 3–6× faster. So do delay loops that only reload
  the watchdog while they wait for a tick counter in RAM (the B2500's
  `delay_ms()`): such a loop can only end after an interrupt, and a watchdog
  that expires while the CPU sleeps there counts as reloaded. Loops that poll a
  peripheral are left alone.
* **FPU state survives nesting.** Exception return restores CONTROL.FPCA from
  EXC_RETURN, as the hardware does; otherwise a handler preempted inside FP
  code loses its condition flags.

## Peripherals

Behavioural models (`periph/n32g45x.py`, `periph/stm32.py`, `periph/hrtim.py`)
implement what firmware observes: ready flags, level-triggered interrupts, UART
byte timing with IDLE detection and DMA, DMA routed by peripheral address,
timers with input capture (on the N32G45x counting at their APB clock, doubled
when the APB is divided, as on the chip; the UARTs' baud rates follow the same
APB clocks), timer- and HRTIM-triggered ADC sequences, I2C state
machines with pluggable slaves (a transfer costs its bus time), CAN/FDCAN,
CORDIC, flash with persistence, IWDG resets, an RTC on virtual time. Register
names come from the vendor SVD files (`data/*.json`) and show up in traces.

Anything not modelled is a plain register file. Busy-wait loops on such
registers are broken automatically (after 200 identical reads without a write
the value is returned inverted once and logged).

## FC41D

`fc41d/module.py` parses the AT stream incrementally, including binary and
length-prefixed payloads (`AT+QISEND`, `AT+QMTPUB`, `AT+QBLEGATTSNTFY`,
`AT+QSSLCERT` uploads), and answers in the format the firmware parsers expect:
system, Wi‑Fi, BLE (GATT, advertising, notifications), SSL, MQTT, HTTP,
sockets and Hame's `AT+HMIOPEN`.

Two details only show up after minutes of running. The Hame module answers
`AT+QGETWIFISTATE` as `ssid=…,bssid=…,rssi=…,channel=…`, not in the manual's
positional form; and the `+QHTTPGET`/`+QHTTPPOST` result comes a network round
trip after `OK`. Either one wrong, and a B2500 loses its CT meter. The offline
cloud answers what [Marsrelay](https://github.com/tomquist/marsrelay) answers.

The CT00x meters do not upload certificates: theirs are loaded into the module
at the factory, and the meter compares the whole `AT+QSSLCERT?` reply with what
it expects (with or without checksums, depending on the module build). The
`ct00x` profile has the module claim them; real ones exist only on the devices,
so `--mqtt cloud` cannot work for a meter.

## Network backends (`fc41d/net.py`)

| | default | alternatives |
|---|---|---|
| MQTT | `offline`: sessions connect, publishes are recorded, messages injected | `broker` (redirect to one broker), `cloud` (vendor's host + firmware certificates; needs `--confirm-own-device`) |
| HTTP | `offline`: stand-in for the Hamedata endpoints (time handshake etc.), 404 otherwise, `--http-rule` overrides | `real` (the vendor's servers; needs `--confirm-own-device`) |
| Sockets | `offline`: clients fail, listeners and UDP services accept | `real` (listeners bind on the host at `port + --listen-offset`) |

## Limits

* The N32 ROM (ISP loader, crypto library) is not emulated; the profiles seed
  the factory records so firmware never needs it.
* Builds with an external Ethernet controller (CH395) only see a register file
  there.
* GDB single-steps in thread context; exceptions taken while stopped are
  delivered on the next continue.

## Repository layout

```text
marsemu/
  core/       machine.py (CPU, exceptions, run loop) · nvic.py · bus.py · clock.py · ucount.py
  periph/     n32g45x.py · stm32.py · hrtim.py   (peripheral models + SoC builders)
              ka495xx.py · sh36735.py · sh367309.py (battery front ends) · i2c_gpio.py (bit-banged I2C)
  fc41d/      module.py (AT engine) · net.py · mqtt.py · cloud.py
  analysis/   gdbstub.py · trace.py · idle.py · b2500.py
  plant/      cosim.py · canbus.py · remote.py   (multi-MCU co-simulation)
              venus3.py · venusd.py · venusa.py · venusce.py · jupiter.py · b2500.py · battery.py
              venus3_cli.py (whole-device commands, CT meter, HTTP house)
  modbus.py · uartbridge.py · b2500bus.py · extmodule.py (a real Wi-Fi module on a serial port)
  ui/        session.py (a device run for the dashboard) · server.py · addon.py · static/ (the page)
  profiles.py · emulator.py · cli.py
  data/       register maps generated from SVD files
ha_addon/     the Home Assistant add-on (config.yaml, Dockerfile)
docs/         ui · use-cases · devices · interfaces · library (Python API) · internals · STATUS (firmware survey)
tests/        unit tests · test_firmware.py (pytest -m firmware)
tools/        svd2json.py · survey.py · compare_astrameter.py
```
