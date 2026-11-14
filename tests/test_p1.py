"""The utility meter on a CT003's P1 port: the telegram it sends."""
from marsemu.periph.p1 import crc16, telegram


def test_telegram_lines_and_crc():
    t = telegram(300, -120, 45, (1234.5678, 2.0, 3.25, 0.0))
    body, crc = t.rsplit(b"!", 1)
    assert body.startswith(b"/") and crc.endswith(b"\r\n")
    assert int(crc[:4], 16) == crc16(body + b"!")
    text = t.decode()
    # import and export per phase, in kW with three decimals (the firmware reads %d.%d)
    assert "1-0:21.7.0(00.300*kW)" in text and "1-0:22.7.0(00.000*kW)" in text
    assert "1-0:41.7.0(00.000*kW)" in text and "1-0:42.7.0(00.120*kW)" in text
    assert "1-0:61.7.0(00.045*kW)" in text
    assert "1-0:1.7.0(00.225*kW)" in text and "1-0:2.7.0(00.000*kW)" in text
    assert "1-0:1.8.1(001234.568*kWh)" in text and "1-0:2.8.1(000003.250*kWh)" in text


def test_crc16_is_crc16_arc():
    assert crc16(b"123456789") == 0xBB3D
