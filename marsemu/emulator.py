"""High level entry point: firmware image -> running, observable device."""
from __future__ import annotations

import hashlib
import json
import logging
import os
from typing import Callable, Dict, List, Optional

from .core.machine import Machine
from .fc41d.module import Fc41d, Fc41dConfig
from .fc41d.net import Network
from .periph import n32g45x
from .periph.n32g45x import UartDevice
from .analysis.idle import find_idle_loop, find_watchdog_waits
from .profiles import PROFILES, Profile, detect, load_base, strip_header

log = logging.getLogger("emu")

UARTS = ("USART1", "USART2", "USART3", "UART4", "UART5", "UART6", "UART7", "USART4", "LPUART1")


class UartTap(UartDevice):
    """Default device on every UART: collects output and turns printable
    output into console lines, binary output into hex frames.  The first UART
    that sends an AT command gets the FC41D attached."""

    def __init__(self, emu: "Emulator", name: str):
        self.emu = emu
        self.name = name
        self.buf = bytearray()
        self.total = 0
        self.flush_ev = None
        self.frames: List[bytes] = []

    def on_tx(self, data: bytes) -> None:
        self.total += len(data)
        if self.emu.fc41d.uart is None and self.emu.autodetect_fc41d:
            probe = bytes(self.buf[-8:]) + data
            if b"AT+" in probe or probe.startswith(b"AT\r"):
                self.emu.bind_fc41d(self.name, pending=bytes(self.buf) + data)
                self.buf.clear()
                return
        self.buf.extend(data)
        while b"\n" in self.buf:
            i = self.buf.index(b"\n")
            line = bytes(self.buf[:i]).rstrip(b"\r")
            del self.buf[: i + 1]
            self._emit(line)
        m = self.emu.machine
        if self.flush_ev:
            self.flush_ev.cancel()
        # binary protocols have no newlines: flush after an idle gap
        self.flush_ev = m.clock.after_ms(5, self._flush, "uart-flush")
        if len(self.buf) > 512:
            self._flush()

    def _flush(self) -> None:
        self.flush_ev = None
        if self.buf:
            self._emit(bytes(self.buf))
            self.buf.clear()

    def _emit(self, chunk: bytes) -> None:
        self.frames.append(chunk)
        if len(self.frames) > 2000:
            del self.frames[:1000]
        printable = chunk and all(32 <= b < 127 or b in (9,) for b in chunk)
        text = chunk.decode("latin1") if printable else "hex:" + chunk.hex()
        self.emu.machine.console(self.name, text)


class Emulator:
    """A Marstek device: firmware + MCU + board + FC41D + network."""

    def __init__(self, firmware, *, profile: Optional[str] = None, network: Optional[Network] = None,
                 fc41d: Optional[Fc41dConfig] = None, state_dir: Optional[str] = None,
                 uid: Optional[bytes] = None, watchdog: bool = True, cpi: Optional[int] = None,
                 fc41d_uart: Optional[str] = None, device_type: Optional[str] = None,
                 idle_skip: bool = True, external_module: Optional[str] = None):
        if isinstance(firmware, (str, os.PathLike)):
            self.firmware_path = str(firmware)
            image = open(firmware, "rb").read()
        else:
            self.firmware_path = ""
            image = bytes(firmware)
        #: the application as flashed (an OTA header in front of it removed)
        image = strip_header(image)
        self.image = image
        self.profile: Profile = PROFILES[profile] if profile else detect(image, self.firmware_path)
        p = self.profile
        #: device type the board is "factory programmed" as (e.g. HMA-1)
        self.device_type = device_type
        if p.soc == "f2803x":
            self._init_c28x(network, fc41d, state_dir, uid, watchdog, fc41d_uart, external_module, idle_skip)
            return
        self.machine = m = Machine(hz=p.hz, flash_size=p.flash_kb * 1024, sram_size=max(p.sram_kb, 256) * 1024,
                                   cpi=cpi or p.cpi)
        self.uid = uid or hashlib.sha1(image).digest()[:12]
        if p.soc == "stm32g4":
            from .periph import stm32
            stm32.build_g4(m, uid=self.uid, flash_kb=p.flash_kb)
        elif p.soc == "stm32f0":
            from .periph import stm32
            stm32.build_f0(m, uid=self.uid, flash_kb=p.flash_kb)
        else:
            n32g45x.build(m, uid=self.uid, flash_kb=p.flash_kb)
        self.state_dir = state_dir
        #: devices with a save() method that keep state across runs (EEPROMs, ...)
        self.persist: List = []
        self.base = load_base(image, m.flash_base)
        m.boot_vtor = self.base if self.base != m.flash_base else 0
        self._load_flash()
        self.consoles: Dict[str, List[str]] = {n: [] for n in UARTS}
        m.consoles.append(self._console)
        self.console_listeners: List[Callable[[str, str], None]] = []

        # UART taps + FC41D
        self.taps = {}
        for n in UARTS:
            u = m.bus.peripherals.get(n)
            if u is None or not hasattr(u, "inject"):
                continue
            tap = UartTap(self, n)
            u.device = tap
            self.taps[n] = tap
        self.fc41d = Fc41d(m, network or Network(), fc41d or Fc41dConfig())
        self._load_module_state()
        #: a real module on a serial port instead of the model (see extmodule)
        self.external = None
        if external_module:
            from .extmodule import ExternalModule
            self.external = ExternalModule(self, external_module)
        uart = fc41d_uart or p.fc41d_uart
        self.autodetect_fc41d = uart is None and not p.name.startswith("aux")
        self._fc41d_uart = uart

        # board
        for (port, pin), lvl in p.gpio_inputs.items():
            m.bus.get("GPIO" + port).inputs[pin] = lvl
        for name in "ABCDEFG":
            if "GPIO" + name in m.bus.peripherals:
                m.bus.get("GPIO" + name).float_level = p.gpio_float
        for n in ("ADC1", "ADC2", "ADC3", "ADC4", "ADC5"):
            if n in m.bus.peripherals:
                m.bus.get(n).default = p.adc_default
        for (adc, ch), v in p.adc.items():
            m.bus.get(adc).values[ch] = v
        m.bus.get("IWDG").enabled = watchdog
        for fn in p.setup:
            fn(self)
        if not os.environ.get('EMU_NO_UNMASK'):
            m.install_unmask_exits(image, self.base)
        #: head of the RTOS idle loop, skipped like WFI (see analysis.idle)
        self.idle_loop = find_idle_loop(image, self.base) if idle_skip else None
        if self.idle_loop:
            m.set_idle_loop(self.idle_loop)
        #: delay loops that only reload the watchdog, skipped the same way
        self.watchdog_waits = find_watchdog_waits(image, self.base) if idle_skip else []
        for addr in self.watchdog_waits:
            m.set_idle_loop(addr, feeds_watchdog=True)
        m.reset()
        if uart:
            self.bind_fc41d(uart)
        if p.fc41d_power and self.external is None:
            port, pin = p.fc41d_power
            self.fc41d.power_from_gpio(m.bus.get("GPIO" + port), pin)
        log.info("profile %s: %s", p.name, p.description)

    def _init_c28x(self, network, fc41d, state_dir, uid, watchdog, fc41d_uart, external_module, idle_skip):
        """A TI C2000 device (F2803x): interpreted C28x core + CLA, the SoC
        peripherals, the FC41D on its SCI and an EEPROM on I2C."""
        from .c28x import f2803x
        from .c28x.machine import C28xMachine
        from .periph.n32g45x import I2cEeprom
        p = self.profile
        image = self.image
        self.machine = m = C28xMachine(hz=p.hz)
        f2803x.build(m)
        if not os.environ.get("MARSEMU_PURE_PYTHON"):
            from .c28x.cpu import accelerate
            if not accelerate(m.cpu):
                log.warning("the native C28x core is not built (no C compiler at install time?): "
                            "this device runs at a small fraction of real time")
            else:
                from .c28x.f2803x_native import accelerate_peripherals
                accelerate_peripherals(m)
        self.uid = uid or hashlib.sha1(image).digest()[:12]
        self.state_dir = state_dir
        self.persist: List = []
        #: word address the application is linked for
        self.base = f2803x.FLASH_BASE
        m.entry = self.base
        self._load_flash()
        self.consoles: Dict[str, List[str]] = {}
        m.consoles.append(self._console)
        self.console_listeners: List[Callable[[str, str], None]] = []
        self.taps = {}
        for n in ("SCIA", "LINA"):
            tap = UartTap(self, n)
            m.get(n).device = tap
            self.taps[n] = tap
            self.consoles[n] = []
        self.fc41d = Fc41d(m, network or Network(), fc41d or Fc41dConfig())
        self._load_module_state()
        self.external = None
        if external_module:
            from .extmodule import ExternalModule
            self.external = ExternalModule(self, external_module)
        self.autodetect_fc41d = False
        uart = fc41d_uart or p.fc41d_uart
        self._fc41d_uart = uart
        # settings EEPROM (24C32 at 0x50)
        path = os.path.join(state_dir, "eeprom.bin") if state_dir else None
        self.eeprom = I2cEeprom(0x50, 4096, 2, 32, path)
        m.get("I2CA").devices[0x50] = self.eeprom
        self.persist.append(self.eeprom)
        m.sysctrl.wd_enabled = watchdog
        for fn in p.setup:
            fn(self)
        if idle_skip:
            self.delay_loops = m.install_delay_loops(self.base, self.base + len(image) // 2)
        m.reset()
        if uart:
            self.bind_fc41d(uart)
        log.info("profile %s: %s", p.name, p.description)

    # ----------------------------------------------------------------- fc41d
    def bind_fc41d(self, uart: str, pending: bytes = b"") -> None:
        u = self.machine.bus.get(uart)
        if self.external is not None:
            u.device = self.external
            self.external.attach(u)
            log.info("%s connected to the module on %s", uart, self.external.port)
            if pending:
                i = pending.find(b"AT")
                self.external.on_tx(pending[i:] if i >= 0 else pending)
            return
        u.device = self.fc41d
        self.fc41d.attach(u)
        log.info("FC41D attached to %s (%d baud)", uart, u.baud())
        self.fc41d.power_on()
        if pending:
            i = pending.find(b"AT")
            self.fc41d.on_tx(pending[i:] if i >= 0 else pending)

    # ------------------------------------------------------------- consoles
    def _console(self, channel: str, line: str) -> None:
        self.consoles.setdefault(channel, []).append(line)
        if len(self.consoles[channel]) > 5000:
            del self.consoles[channel][:2500]
        for l in self.console_listeners:
            l(channel, line)

    # -------------------------------------------------------------- running
    def run(self, seconds: Optional[float] = None, until: Optional[Callable[[], bool]] = None) -> str:
        r = self.machine.run(seconds=seconds, until=until)
        self.save_state()
        return r

    @property
    def time(self) -> float:
        return self.machine.clock.seconds

    # --------------------------------------------------------- interaction
    def mqtt_inject(self, topic: str, payload) -> bool:
        if isinstance(payload, str):
            payload = payload.encode()
        return self.fc41d.mqtt_deliver(topic, payload)

    def ble_connect(self) -> None:
        self.fc41d.ble_connect()

    def ble_write(self, data: bytes) -> None:
        self.fc41d.ble_write(data)

    def on(self, event: str, fn: Callable) -> None:
        self.fc41d.on(event, fn)

    # ---------------------------------------------------------- persistence
    def _paths(self):
        if not self.state_dir:
            return None, None
        os.makedirs(self.state_dir, exist_ok=True)
        return os.path.join(self.state_dir, "flash.bin"), os.path.join(self.state_dir, "fc41d.json")

    def _load_flash(self) -> None:
        m = self.machine
        if self.profile.soc == "f2803x":
            flash_path, _ = self._paths()
            if flash_path and os.path.exists(flash_path):
                m.load_flash(open(flash_path, "rb").read())
                log.info("restored flash state from %s", flash_path)
            m.load_flash(self.image, (self.base - m.flash_base) * 2)
            return
        flash_path, _ = self._paths()
        if flash_path and os.path.exists(flash_path):
            data = open(flash_path, "rb").read()[: m.flash_size]
            m.load_flash(data)
            log.info("restored flash state from %s", flash_path)
        m.load_flash(self.image, self.base - m.flash_base)
        if self.base != m.flash_base:
            log.info("application image linked at 0x%08x (no bootloader in file)", self.base)

    def _load_module_state(self) -> None:
        _, mod_path = self._paths()
        if mod_path and os.path.exists(mod_path):
            st = json.load(open(mod_path))
            self.fc41d.certs = {k: bytes.fromhex(v) for k, v in st.get("certs", {}).items()}
            for k in ("ssid", "password"):
                if k in st:
                    setattr(self.fc41d.cfg, k, st[k])

    def save_state(self) -> None:
        flash_path, mod_path = self._paths()
        if not flash_path:
            return
        for dev in self.persist:
            dev.save()
        open(flash_path, "wb").write(self.machine.flash_bytes())
        json.dump({"certs": {k: v.hex() for k, v in self.fc41d.certs.items()},
                   "ssid": self.fc41d.cfg.ssid, "password": self.fc41d.cfg.password},
                  open(mod_path, "w"), indent=1)
