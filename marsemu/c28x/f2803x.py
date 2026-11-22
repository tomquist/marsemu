"""TMS320F2803x (Piccolo) SoC: memory map and peripherals.

Register maps and behaviour from the "TMS320F2803x Microcontrollers
Technical Reference Manual" (SPRUI10).  The models cover what the firmware
here uses; registers nobody models read back what was written.
"""
from __future__ import annotations

import logging
from array import array
from typing import Dict, Optional

from .cpu import CODE, IO, RAM, ROM
from .machine import C28xMachine, C28xPeripheral

log = logging.getLogger("emu.f2803x")

FLASH_BASE, FLASH_END = 0x3E8000, 0x3F8000
OTP_BASE, OTP_END = 0x3D7800, 0x3D8000
BOOT_ROM, BOOT_ROM_END = 0x3FE000, 0x400000
#: TI's calibration routine in OTP
DEVICE_CAL = 0x3D7C80
#: temperature sensor calibration (degC = (raw - offset) * slope / 32768)
TEMP_SLOPE, TEMP_OFFSET = 5184, 1784
#: TI OTP getter functions -> the value they return
OTP_GETTERS = {
    0x3D7E82: TEMP_SLOPE,     # getTempSlope()
    0x3D7E85: TEMP_OFFSET,    # getTempOffset()
    0x3D7E90: 0,              # getOsc1FineTrimSlope()
    0x3D7E93: 0,              # getOsc1FineTrimOffset()
    0x3D7E96: 0,              # getOsc1CoarseTrim()
    0x3D7E99: 0,              # getOsc2FineTrimSlope()
    0x3D7E9C: 0,              # getOsc2FineTrimOffset()
    0x3D7E9F: 0,              # getOsc2CoarseTrim()
    0x3D7EA2: TEMP_OFFSET,    # getRefTempOffset()
}


def temp_raw(deg_c: float) -> int:
    """Temperature sensor reading for ``deg_c`` (with the calibration above)."""
    return int(round(TEMP_OFFSET + deg_c * 32768 / TEMP_SLOPE))

#: (name, base, size in words)
BLOCKS = [
    ("DEVEMU", 0x0880, 0x105),
    ("SYSPWR", 0x0985, 3),
    ("FLASH", 0x0A80, 0x60),
    ("CSM", 0x0AE0, 0x10),
    ("ADCRESULT", 0x0B00, 0x20),
    ("CPUTIMER", 0x0C00, 0x40),
    ("PIE", 0x0CE0, 0x20),
    ("CLA1", 0x1400, 0x80),
    ("ECANA", 0x6000, 0x200),
    ("COMP1", 0x6400, 0x20),
    ("COMP2", 0x6420, 0x20),
    ("COMP3", 0x6440, 0x20),
    ("EPWM1", 0x6800, 0x40),
    ("EPWM2", 0x6840, 0x40),
    ("EPWM3", 0x6880, 0x40),
    ("EPWM4", 0x68C0, 0x40),
    ("EPWM5", 0x6900, 0x40),
    ("EPWM6", 0x6940, 0x40),
    ("EPWM7", 0x6980, 0x40),
    ("ECAP1", 0x6A00, 0x20),
    ("HRCAP1", 0x6AC0, 0x20),
    ("HRCAP2", 0x6AE0, 0x20),
    ("EQEP1", 0x6B00, 0x40),
    ("LINA", 0x6C00, 0x80),
    ("GPIOCTRL", 0x6F80, 0x40),
    ("GPIODAT", 0x6FC0, 0x20),
    ("GPIOINT", 0x6FE0, 0x20),
    ("SYSCTRL", 0x7010, 0x20),
    ("SPIA", 0x7040, 0x10),
    ("SCIA", 0x7050, 0x10),
    ("NMIWD", 0x7060, 0x10),
    ("XINT", 0x7070, 0x10),
    ("ADC", 0x7100, 0x80),
    ("SPIB", 0x7740, 0x10),
    ("I2CA", 0x7900, 0x40),
]


def build(m: C28xMachine, models: Optional[Dict[str, type]] = None) -> None:
    """Map memory and attach a peripheral (model or plain register file) for every block."""
    cpu = m.cpu
    cpu.map(0, 1 << 22, ROM)            # unimplemented space: reads as 0, writes ignored
    cpu.map(0x0000, 0x0800, RAM)        # M0, M1
    cpu.map(0x0D00, 0x0E00, RAM)        # PIE vector table
    cpu.map(0x1480, 0x1580, RAM)        # CLA message RAMs
    cpu.map(0x8000, 0xA000, RAM)        # L0-L3
    cpu.map(FLASH_BASE, FLASH_END, ROM)
    cpu.map(OTP_BASE, OTP_END, ROM)
    cpu.map(BOOT_ROM, BOOT_ROM_END, ROM)
    m.flash_base = FLASH_BASE
    m.flash_size = FLASH_END - FLASH_BASE
    cpu.mem[FLASH_BASE:FLASH_END] = array("H", b"\xff\xff" * (FLASH_END - FLASH_BASE))
    # TI's Device_cal() in OTP trims the oscillators and the ADC; the trims
    # change nothing here, so the routine just returns
    cpu.mem[DEVICE_CAL:DEVICE_CAL + 3] = array("H", (0x7622, 0x761A, 0x0006))   # EALLOW; EDIS; LRETR
    # TI's OTP getters (DSP2803x_TempSensorConv.c / _OscComp.c): MOV @AL,#value; LRETR
    for addr, value in OTP_GETTERS.items():
        cpu.mem[addr:addr + 3] = array("H", (0x28A9, value & 0xFFFF, 0x0006))
    from . import iqmath
    iqmath.install(cpu.mem)
    from . import f2803x_sys as sysp
    from . import f2803x_adc as adcp
    from . import f2803x_comm as comm
    from .cla import Cla
    from . import f2803x_pwm as pwm
    m.gpio = sysp.GpioPorts(m)
    std = {"SYSCTRL": sysp.SysCtrl, "PIE": sysp.Pie, "CPUTIMER": sysp.CpuTimers, "GPIOCTRL": sysp.GpioCtrl,
           "GPIODAT": sysp.GpioData, "ADC": adcp.Adc, "ADCRESULT": adcp.AdcResult, "COMP1": adcp.Comparator,
           "COMP2": adcp.Comparator, "COMP3": adcp.Comparator, "SCIA": comm.Sci, "LINA": comm.Lin,
           "I2CA": comm.I2c, "CLA1": Cla, "ECAP1": pwm.Ecap}
    std.update(models or {})
    for name, base, size in BLOCKS:
        if name.startswith("EPWM") and name not in (models or {}):
            m.attach(pwm.Epwm(m, name, base, size, index=int(name[4:])))
            continue
        cls = std.get(name, C28xPeripheral)
        m.attach(cls(m, name, base, size))
    m.epwm = [None] + [m.get(f"EPWM{i}") for i in range(1, 8)]
    for i in range(1, 7):
        m.epwm[i].next_in_chain = m.epwm[i + 1]
    m.pie = m.get("PIE")
    m.sysctrl = m.get("SYSCTRL")
    m.adc = m.get("ADC")
    m.adc.temp_raw = temp_raw(30.0)
    m.cla = m.get("CLA1")
    cpu.iack_hook = m.cla.iack
    m.nvic = sysp.InterruptCounts(m)
    m.adc.int_listeners.append(lambda n: m.cla.peripheral_irq("adc", n))
    m.get("ADCRESULT").adc = m.adc
    # ADCRESULT0-15 are read-only: served from the memory the ADC writes
    cpu.map(0x0B00, 0x0B40, RAM)
