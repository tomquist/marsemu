"""Firmware family detection and board profiles.

A profile describes the board around the MCU: clock, which UART the FC41D
is on (or ``None`` to auto-detect the UART that speaks AT), GPIO input
levels, models for chips on I2C/SPI/UART, and default ADC readings.

Profiles are deliberately data-driven so new firmware families only need a
new entry here.  ``detect()`` picks a profile from the image itself.
"""
from __future__ import annotations

import logging
import os
import re
import struct
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

log = logging.getLogger("emu.profile")


@dataclass
class Profile:
    name: str
    description: str
    #: SoC model: "n32g45x", "stm32g4" or "stm32f0"
    soc: str = "n32g45x"
    hz: int = 144_000_000
    flash_kb: int = 512
    sram_kb: int = 144
    #: UART the FC41D sits on (e.g. "USART1"); None = first UART that sends "AT"
    fc41d_uart: Optional[str] = None
    #: (port, pin) -> level for GPIO inputs that matter
    gpio_inputs: Dict[Tuple[str, int], int] = field(default_factory=dict)
    #: default level of floating GPIO inputs
    gpio_float: int = 1
    #: ADC channel -> raw 12-bit value
    adc: Dict[Tuple[str, int], int] = field(default_factory=dict)
    adc_default: int = 2048
    #: hooks run after the machine is built: fn(emulator)
    setup: List[Callable] = field(default_factory=list)
    #: device type string used by the cloud (for documentation / defaults)
    device_type: str = ""
    notes: str = ""
    #: clock cycles per emulated instruction (timing of the virtual clock)
    cpi: int = 2


PROFILES: Dict[str, Profile] = {}


def register(p: Profile) -> Profile:
    PROFILES[p.name] = p
    return p


GENERIC = register(Profile("generic", "Any N32G45x firmware; FC41D auto-detected"))


def find_type_tables(image: bytes, base: int = 0x08000000):
    """Find B2500 device-type name tables: ``"HMx-n\0"`` strings at a
    constant stride whose start address is referenced by code.
    Returns a list of (offset, stride, [names])."""
    hits = [m.start() for m in re.finditer(rb"HM[A-Z]-\d\d?\x00", image)]
    words = set(struct.unpack_from("<%dI" % (len(image) // 4), image))
    out = []
    for i, a in enumerate(hits):
        if base + a not in words:
            continue
        for b in hits[i + 1:i + 2]:
            stride = b - a
            if not 8 <= stride <= 64:
                continue
            names = []
            off = a
            while (m := re.match(rb"(HM[A-Z]-\d\d?)\x00", image[off:off + 7])):
                name = m.group(1).decode()
                if name in names:   # some builds keep two copies back to back
                    break
                names.append(name)
                off += stride
            if len(names) >= 4:
                out.append((a, stride, names))
    return out


#: device type an HMJ-board build gets when none is given (newer HMJ builds are only published for HMJ-2)
HMJ_DEFAULT_TYPE = "HMJ-2"


def default_device_type(image: bytes, base: int, profile) -> str:
    """The factory device type to give a B2500 v2 build when the user names
    none: the profile's (HMA-1), or HMJ-2 for a build for the HMJ board."""
    if profile.device_type == "HMA-1":
        from .analysis.b2500 import board_family
        if board_family(image, base) == "HMJ":
            return HMJ_DEFAULT_TYPE
    return profile.device_type


def seed_b2500_config(emu) -> None:
    """Factory data for B2500 firmwares.

    The device-type index is programmed at the factory into the config page
    at 0x0803F800 (offset 0x0A).  Several builds copy it from flash even when
    the page is invalid, so an erased page yields a garbage type (and MQTT
    topic).  If the page is erased we write the index of the profile's
    device type, like the factory does."""
    m = emu.machine
    page = 0x0803F800
    if page + 0x54 > m.flash_base + m.flash_size:
        return
    cur = m.read(page, 0x54)
    if cur != b"\xff" * 0x54:
        return
    want = emu.device_type or default_device_type(emu.image, emu.base, emu.profile)
    tabs = [t for t in find_type_tables(emu.image, emu.base) if want in t[2]]
    if not tabs:
        return
    idx = tabs[0][2].index(want)
    off = page - m.flash_base
    data = bytearray(m.flash_bytes()[off:off + 0x54])
    data[0x0A] = idx
    data[0x04] = 5
    m.load_flash(bytes(data), off)
    log.info("seeded factory device type %s (index %d) at 0x%08x", want, idx, page + 0x0A)

def seed_b2500_v1_config(emu) -> None:
    """B2500 v1 (HMB V12x-V14x) keeps its settings in a record at
    0x0803F000: a version marker half-word, fields, and a 32-bit half-word
    sum (some builds store its complement) right after the record.  On an
    erased page it copies a flag from 0xFF and jumps into the ROM ISP loader,
    so write a valid record with the factory defaults.  Record size, marker
    and checksum style are read from the code that validates it."""
    from capstone import CS_ARCH_ARM, CS_MODE_MCLASS, CS_MODE_THUMB, Cs

    m = emu.machine
    page = 0x0803F000
    if m.read(page, 0x100) != b"\xff" * 0x100:
        return
    img = emu.image
    lit = struct.pack("<I", page)
    lits = {emu.base + i for i in range(0, len(img) - 3, 4) if img[i:i + 4] == lit}
    if not lits:
        return
    md = Cs(CS_ARCH_ARM, CS_MODE_THUMB | CS_MODE_MCLASS)
    md.skipdata = True
    ins = list(md.disasm(img[:0x30000], emu.base))
    by_addr = {x.address: k for k, x in enumerate(ins)}
    for k, x in enumerate(ins):
        if not (x.mnemonic == "ldr" and "pc" in x.op_str and "#" in x.op_str):
            continue
        try:
            tgt = ((x.address + 4) & ~3) + int(x.op_str.split("#")[-1].rstrip("]"), 16)
        except ValueError:
            continue
        if tgt not in lits:
            continue
        window = ins[k:k + 14]
        size = marker = csum_fn = None
        for y in window:
            if y.mnemonic.startswith("movs") and "#" in y.op_str and size is None:
                v = int(y.op_str.split("#")[1], 16)
                if 0x20 <= v <= 0x200 and v % 2 == 0:
                    size = v
            if y.mnemonic == "bl" and csum_fn is None:
                csum_fn = int(y.op_str.lstrip("#"), 16)
            if y.mnemonic == "cmp" and y.op_str.startswith("r0, #") and size is not None:
                v = int(y.op_str.split("#")[1], 16)
                if 0x40 <= v < 0x100:
                    marker = v
                    break
        if size is None or marker is None:
            continue
        complement = False
        if csum_fn is not None and csum_fn in by_addr:
            for y in ins[by_addr[csum_fn]:by_addr[csum_fn] + 24]:
                if (y.mnemonic.startswith("rsb") and "#-1" in y.op_str) or y.mnemonic.startswith("mvn"):
                    complement = True
                if y.mnemonic.startswith("pop") or y.mnemonic == "bx":
                    break
        cfg = bytearray(size + 4)
        cfg[0:2] = struct.pack("<H", marker)
        if emu.profile.name == "b2500-v1" and size == HMB_RECORD_SIZE:
            # what the firmware's own factory reset (cd=11) writes: discharge
            # depth 100 %, output threshold 100 W, output 1 on, pack 50 Ah
            cfg[2:0x10] = HMB_RECORD_DEFAULTS
        emu.v1_record = (page, size, complement)
        write_v1_record(emu, cfg)
        log.info("seeded legacy config record (marker 0x%02x, %d bytes%s) at 0x%08x", marker, size,
                 ", complemented sum" if complement else "", page)
        return


#: HMB (B2500 v1, V121-V144) settings record: size, factory defaults of bytes 2..15, stored SOC
HMB_RECORD_SIZE = 0x3C
HMB_RECORD_DEFAULTS = bytes.fromhex("640080d1f0080095ba0a00640001")
HMB_RECORD_SOC = 0x14

#: B2500 v2 (HMA V119-V255, HMJ V100-V118, old and new layout): settings record at
#: 0x0803F000, marker 0x68, 0x58 bytes + sum.  Byte 0x12 holds flags (bit 0:
#: the SOC in byte 0x14 is valid, bit 6: enter the ROM bootloader); with bit 0
#: set, V119 also keeps bytes 2, 0x0D-0x0F, 0x2B and 0x56 of an invalid record.
HMA_RECORD_PAGE = 0x0803F000
HMA_RECORD_FLAGS = 0x12
HMA_RECORD_SOC = 0x14


def seed_hma_record(emu) -> None:
    """B2500 v2 builds write their own defaults (discharge depth 100 %,
    timer 1 at 800 W all day) over an invalid settings record, but first read
    the flags byte 0x12 from it: erased, its bit 6 sends V119 into the ROM
    bootloader.  So leave an invalid record whose flags are clear and whose
    kept settings hold the firmware's defaults."""
    m = emu.machine
    page = HMA_RECORD_PAGE
    if page + 0x100 > m.flash_base + m.flash_size or m.read(page, 0x100) != b"\xff" * 0x100:
        return
    rec = bytearray(b"\xff" * 0x58)
    rec[2:4] = (100).to_bytes(2, "little")
    rec[0x0D:0x10] = bytes([100, 1, 0])
    rec[HMA_RECORD_FLAGS:HMA_RECORD_FLAGS + 8] = bytes(8)
    rec[0x2B] = 0
    rec[0x56:0x58] = bytes(2)
    m.load_flash(bytes(rec), page - m.flash_base)
    emu.hma_record = page
    log.info("seeded an invalid settings record at 0x%08x (the firmware writes its defaults)", page)


def write_v1_record(emu, cfg: bytearray) -> None:
    """Write a legacy config record (see :func:`seed_b2500_v1_config`) with its checksum."""
    page, size, complement = emu.v1_record
    total = sum(struct.unpack_from("<%dH" % (size // 2), cfg, 0))
    if complement:
        total = ~total
    struct.pack_into("<I", cfg, size, total & 0xFFFFFFFF)
    emu.machine.load_flash(bytes(cfg), page - emu.machine.flash_base)


def set_b2500_soc(emu, soc_pct: float) -> bool:
    """Store a state of charge in a seeded settings record (before the
    firmware runs), so the firmware starts from the pack's real SOC.

    * HMB: byte 0x12 is the SOC estimator's state: 0 = never calibrated (the
      firmware starts at 0 % and only calibrates at full charge), 1 = the
      stored value is used as is.
    * B2500 v2: bit 0 of byte 0x12 marks the stored SOC valid; without
      it the firmware estimates the SOC from the lowest cell voltage, which
      reads 85 % or more from 3.30 V up (the middle of an LFP pack)."""
    soc = max(0, min(100, int(round(soc_pct))))
    page = getattr(emu, "hma_record", None)
    if page is not None:
        emu.machine.load_flash(bytes([1, 0, soc]), page + HMA_RECORD_FLAGS - emu.machine.flash_base)
        return True
    rec = getattr(emu, "v1_record", None)
    if rec is None or rec[1] != HMB_RECORD_SIZE:
        return False
    page, size, _ = rec
    cfg = bytearray(emu.machine.read(page, size + 4))
    cfg[HMB_RECORD_SOC - 2] = 1
    cfg[HMB_RECORD_SOC] = soc
    write_v1_record(emu, cfg)
    return True


register(Profile(
    "b2500-v1", "Saturn/B2500 v1 (HMB-x), N32G452, bootloader + app at 0x08002800",
    device_type="HMB-1", flash_kb=256, setup=[seed_b2500_v1_config, seed_b2500_config]))
register(Profile(
    "b2500-v2", "Saturn/B2500 v2 (HMA/HMF/HMK-x), N32G452, bootloader + app at 0x08002800",
    device_type="HMA-1", flash_kb=256, setup=[seed_hma_record, seed_b2500_config]))
register(Profile(
    "b2500-v3", "B2500 v2 hardware revision with app at 0x08007800 (HMA V222+, HMJ-x)",
    device_type="HMA-1", flash_kb=256, setup=[seed_hma_record, seed_b2500_config]))
register(Profile(
    "venus", "Venus C/E (HMG-x) control board, STM32G474", soc="stm32g4", hz=170_000_000, sram_kb=96,
    device_type="HMG-50"))
register(Profile(
    "venus3", "Venus E 3.0 / A / D control board (VNSE3/VNSA/VNSD)", device_type="VNSE3-0", cpi=1,
    setup=[lambda emu: control_eeprom(emu)]))
register(Profile(
    "venus-mini", "Venus E Mini (VNSEMINI) control board (PMU), N32G45x, app at 0x08008000",
    device_type="VNSEMINI-0"))
register(Profile(
    "jupiter", "Jupiter C/E (HMM/HMN-x) and C Plus (JPLS-8H) control board", device_type="HMN-1"))
def preloaded_module_certs(emu) -> None:
    """The CT00x firmwares expect the FC41D to hold factory-loaded TLS
    certificates and only check that it does: they compare the module's whole
    ``AT+QSSLCERT?`` reply with a string, ``+QSSLCERT: "CA",1,7b67`` ... from
    newer module builds, ``+QSSLCERT: "CA",1`` ... from 202406141323 and
    202409090159.  With all three missing they upload certificates from their
    SPI flash, with anything else they delete them and ask again.  Let the
    module claim exactly the certificates the image expects."""
    for kind, ck in re.findall(rb'\+QSSLCERT: "(CA|User Cert|User Key)",1,([0-9a-f]{4})', emu.image):
        emu.fc41d.cfg.preloaded_certs.setdefault(kind.decode(), ck.decode())


def ct_spi_flash(emu) -> None:
    """CT00x keep factory data (TLS client certificate, key) in an external
    SPI NOR flash on SPI2.  Attach an empty one; load a dump with --spi-flash."""
    from .periph.n32g45x import SpiNorFlash
    spi = emu.machine.bus.peripherals.get("SPI2")
    if spi is not None and spi.device is None:
        spi.device = SpiNorFlash(emu.machine)


def ct_meter_chip(emu) -> None:
    """What the meter measures, as ``emu.ct_meter`` (``set_power(a, b, c)``):

    * CT003 (HME-3), a P1 reader: a utility meter on its P1 port (USART3),
      a DSMR 5 telegram a second;
    * CT002 (HME-4): the BL09xx metering chip on bit-banged SPI (GPIOA PA4-PA7).

    Both start at 0 W on every phase."""
    from .periph.bl_spi import attach_ct_meter
    from .periph.p1 import attach_p1_meter
    if attach_p1_meter(emu) is None:
        attach_ct_meter(emu)


register(Profile(
    "ct00x", "CT002/CT003 smart meter (HME-x), STM32F0-class Cortex-M0", soc="stm32f0", hz=48_000_000,
    flash_kb=128, sram_kb=16, device_type="HME-4", fc41d_uart="USART1", setup=[ct_spi_flash, preloaded_module_certs, ct_meter_chip],
    notes="USART2 is a debug console that also echoes AT commands"))
def control_eeprom(emu) -> None:
    """Venus control boards keep their settings in a 24Cxx EEPROM at 0x50 on I2C1
    (settings blocks up to 0x36BD, i.e. at least a 24C128)."""
    from .periph.n32g45x import I2cEeprom
    i2c = emu.machine.bus.peripherals.get("I2C1")
    if i2c is None or 0x50 in i2c.devices:
        return
    path = os.path.join(emu.state_dir, "eeprom.bin") if emu.state_dir else None
    i2c.devices[0x50] = dev = I2cEeprom(0x50, 32768, 2, 64, path)
    emu.persist.append(dev)


register(Profile(
    "aux", "Secondary MCU without FC41D (BMS, inverter, ...)", fc41d_uart=None))
register(Profile(
    "hmi", "Saturn M microinverter (HMI-x), TMS320F28035 (C28x + CLA), app at 0x3E8000", soc="f2803x",
    hz=60_000_000, flash_kb=128, sram_kb=20, fc41d_uart="SCIA", device_type="HMI-1",
    notes="LIN-A runs a debug console (115200 baud); settings in a 24C32 EEPROM on I2C"))
register(Profile(
    "aux-g4", "Secondary STM32G474 MCU without FC41D (Venus D/E 3.0 inverter board)", soc="stm32g4",
    hz=170_000_000, sram_kb=128, fc41d_uart=None))


#: header of the Venus E Mini's OTA images: 32 ASCII bytes
#: ``<device type>_<board>_V<version, hex>_<payload size>_<checksum>_FF``
#: (e.g. ``VNSEMINI-0_PMU_V135_186556_42_FF``) in front of the application
OTA_HEADER = re.compile(rb"([A-Z0-9]+-\d)_([A-Z]+)_V([0-9A-F]{3,4})_(\d+)_([0-9A-F]{2})_([0-9A-F]{2})")
OTA_HEADER_SIZE = 32


def ota_header(image: bytes) -> Optional[dict]:
    """The fields of an OTA header in front of the image, or None.  Only a
    header whose size field matches the payload counts."""
    m = OTA_HEADER.fullmatch(image[:OTA_HEADER_SIZE])
    if m is None or int(m.group(4)) != len(image) - OTA_HEADER_SIZE:
        return None
    return {"device_type": m.group(1).decode(), "board": m.group(2).decode(),
            "version": int(m.group(3), 16), "size": int(m.group(4)), "checksum": int(m.group(5), 16)}


def strip_header(image: bytes) -> bytes:
    """The application without an OTA header (see :data:`OTA_HEADER`): what the
    bootloader writes to flash."""
    return image[OTA_HEADER_SIZE:] if ota_header(image) else image


def _literal_count(image: bytes, words) -> int:
    return sum(image.count(struct.pack("<I", w)) for w in words)


def strings_of(image: bytes) -> List[bytes]:
    return re.findall(rb"[\x20-\x7e]{6,}", image)


def is_c28x_image(image: bytes) -> bool:
    """A TI C2000 (C28x) application image: 16-bit characters (the AT
    command strings are stored one character per word) and a long branch
    (``LB 0x3Fxxxx``) as its first instruction."""
    if len(image) < 8:
        return False
    w0, w1 = struct.unpack_from("<HH", image, 0)
    return (w0 & 0xFFC0) == 0x0040 and b"A\x00T\x00+\x00Q\x00" in image


def detect(image: bytes, filename: str = "") -> Profile:
    image = strip_header(image)
    if is_c28x_image(image):
        return PROFILES["hmi"]
    sp, reset = struct.unpack_from("<II", image, 0)
    has_at = b"AT+QBLE" in image or b"AT+QVERSION" in image
    name = filename.lower()
    if not has_at:
        g4 = _literal_count(image, (0x48000000, 0x48000400, 0x48000800))
        n32 = _literal_count(image, (0x40010800, 0x40010C00, 0x40011000))
        return PROFILES["aux-g4" if g4 > 10 and n32 == 0 else "aux"]
    if (reset & ~0x7FF) == 0x08006800:
        return PROFILES["ct00x"]
    if b"VNSEMINI-" in image or b"VNSEM-" in image:
        return PROFILES["venus-mini"]
    if b"VNSE" in image or b"VNSA" in image or b"VNSD" in image or (reset & ~0xF) == 0x08004A60:
        return PROFILES["venus3"]
    if b"MST_AIOS" in image or b"HMN-" in image or b"HMM-" in image:
        return PROFILES["jupiter"]
    if (reset & ~0xF) == 0x08002AA0 or (reset & ~0xF) == 0x08002AB0 or b"MST_ACCP" in image:
        return PROFILES["venus"]
    if len(image) > 0x7808:
        sp2, rst2 = struct.unpack_from("<II", image, 0x7800)
        if 0x20000000 <= sp2 < 0x20040000 and 0x08007800 <= rst2 < 0x08080000:
            return PROFILES["b2500-v3"]
    if b"HMA-1" not in image or b"B2500_All" in image or "b2500_all" in name:
        return PROFILES["b2500-v1"]
    return PROFILES["b2500-v2"]


def load_base(image: bytes, flash_base: int = 0x08000000) -> int:
    """Address the image is linked for.

    Full images (bootloader + app) start at the flash base.  Application-only
    OTA images have their vector table at the start of the file (after an OTA
    header, if any) and a reset handler a few hundred bytes after it, e.g.
    0x08002a6d -> 0x08002800.
    """
    image = strip_header(image)
    sp, reset = struct.unpack_from("<II", image, 0)
    if not (flash_base <= reset < flash_base + 0x100000):
        return flash_base
    off = reset - flash_base
    if off < 0x800 and off < len(image):
        return flash_base
    base = (reset & ~0x7FF)
    if reset - base < 0x100:  # vector table is at least 0x100 bytes
        base -= 0x800
    return base
