"""Images with an OTA header (Venus E Mini) are recognised and placed like the bootloader does."""
import struct

from marsemu.cli import image_info
from marsemu.profiles import detect, load_base, ota_header, strip_header


def mini_image(board: str = "PMU", version: int = 309, size_field=None) -> bytes:
    """A Venus E Mini control application linked at 0x08008000, with the OTA header in front."""
    app = struct.pack("<II", 0x20020660, 0x080081AD) + b"\x00" * 0x1A0
    app += b"AT+QVERSION\x00VNSEMINI-0\x00VNSEM-0\x00marstek_energy/%s/App/%s/ctrl\x00"
    app = app.ljust(186556, b"\xff")             # the real one's size: the header is exactly 32 bytes
    size = len(app) if size_field is None else size_field
    header = f"VNSEMINI-0_{board}_V{version:03X}_{size}_42_FF".encode()
    assert len(header) == 32
    return header + app


def test_header_fields():
    img = mini_image()
    assert ota_header(img) == {"device_type": "VNSEMINI-0", "board": "PMU", "version": 309,
                               "size": len(img) - 32, "checksum": 0x42}
    assert strip_header(img)[:8] == struct.pack("<II", 0x20020660, 0x080081AD)


def test_header_must_match_the_payload_size():
    img = mini_image(size_field=123456)
    assert ota_header(img) is None
    assert strip_header(img) == img


def test_images_without_a_header_are_unchanged():
    img = struct.pack("<II", 0x20021DF8, 0x08004A71) + b"VNSE3-0\x00" + b"\x00" * 64
    assert ota_header(img) is None and strip_header(img) == img


def test_mini_control_is_its_own_profile_at_its_link_address():
    """It names "VNSE..." too, so it must not be taken for a Venus E 3.0."""
    img = mini_image()
    assert detect(img).name == "venus-mini"
    assert load_base(img) == 0x08008000


def test_info_reports_the_header(tmp_path):
    path = tmp_path / "309_control_VNSEMINI-0_PMU.bin"
    path.write_bytes(mini_image())
    info = image_info(str(path))
    assert info["profile"] == "venus-mini" and info["linked_at"] == "0x8008000"
    assert info["ota_header"]["board"] == "PMU" and info["ota_header"]["version"] == 309
    assert info["reset"] == "0x80081ad"
    assert "VNSEMINI-0" in info["device_types"]


class _Board:
    """What a profile's setup hook sees of the emulator."""

    def __init__(self, state_dir=None, soc="n32g45x"):
        from marsemu.core.machine import Machine
        from marsemu.periph import n32g45x, stm32
        self.machine = Machine()
        if soc == "stm32g4":
            stm32.build_g4(self.machine, uid=bytes(12))
        else:
            n32g45x.build(self.machine)
        self.state_dir = state_dir
        self.persist = []


def test_venus_mini_settings_flash_survives_a_reset(tmp_path):
    """The Mini's settings live in an SPI NOR flash on SPI1 (chip select PA4)."""
    from marsemu.profiles import PROFILES
    p = PROFILES["venus-mini"]
    assert p.fc41d_power == ("A", 1) and p.fc41d_uart == "USART2"
    b = _Board(str(tmp_path))
    for fn in p.setup:
        fn(b)
    m = b.machine
    spi, gpioa = m.bus.get("SPI1"), m.bus.get("GPIOA")

    def txn(*data):
        gpioa.write32(0x14, 1 << 4)                     # select
        out = []
        for x in data:
            spi.write32(0x0C, x)
            out.append(spi.read32(0x0C))
        gpioa.write32(0x10, 1 << 4)                     # deselect
        return out
    txn(0x06)
    txn(0x20, 0x00, 0xA0, 0x00)                         # erase the sector at 0xA000
    txn(0x06)
    txn(0x02, 0x00, 0xA0, 0x00, 0x46, 0x00)             # program two bytes
    m.reset()
    assert txn(0x03, 0x00, 0xA0, 0x00, 0, 0)[4:] == [0x46, 0x00]
    for dev in b.persist:
        dev.save()
    assert (tmp_path / "spiflash.bin").read_bytes()[0xA000] == 0x46
