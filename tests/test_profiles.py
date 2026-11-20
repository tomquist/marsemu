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
