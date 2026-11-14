import pytest

from marsemu.cli import main


OWN = ["--uid", "00" * 12, "--wifi-mac", "c8:47:8c:12:34:01", "--ble-mac", "c8:47:8c:12:34:02"]


def test_vendor_cloud_needs_confirmation(capsys):
    assert main(["run", "missing.bin", "--mqtt", "cloud", *OWN]) == 2
    assert "needs --confirm-own-device" in capsys.readouterr().err
    assert main(["run", "missing.bin", "--http", "real", *OWN]) == 2


def test_vendor_cloud_needs_an_explicit_identity(capsys):
    """The default identity is the same for every marsemu user, so nobody's own."""
    assert main(["run", "missing.bin", "--mqtt", "cloud", "--confirm-own-device", "--uid", "00" * 12]) == 2
    assert "needs --wifi-mac, --ble-mac." in capsys.readouterr().err


def test_vendor_cloud_with_confirmation_gets_past_the_check(capsys):
    with pytest.raises(SystemExit):     # the missing image, after the check
        main(["run", "missing.bin", "--mqtt", "cloud", "--confirm-own-device", *OWN])
    assert "as your own device: UID 000000000000000000000000" in capsys.readouterr().err


def test_missing_image_is_a_clean_error(capsys):
    with pytest.raises(SystemExit) as e:
        main(["info", "missing.bin"])
    assert e.value.code == 2
    assert "missing.bin" in capsys.readouterr().err
