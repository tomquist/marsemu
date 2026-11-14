"""The CT002's metering chip on bit-banged SPI, driven the way its firmware does."""
from marsemu.periph.bl_spi import READ, WRITE, BlChip, GpioSpiSlave, checksum

CS, SCK, MOSI, MISO = 4, 5, 7, 6


class Port:
    def __init__(self):
        self.watchers, self.inputs = [], {}

    def set(self, pin, lvl):
        for w in self.watchers:
            w(pin, lvl)


def xfer(port, out: bytes) -> bytes:
    """The firmware's byte loop: data out, clock high, clock low, read a bit."""
    port.set(CS, 0)
    got = bytearray()
    for byte in out:
        v = 0
        for i in range(8):
            port.set(MOSI, byte >> (7 - i) & 1)
            port.set(SCK, 1)
            port.set(SCK, 0)
            v = v << 1 | port.inputs[MISO]
        got.append(v)
    port.set(CS, 1)
    return bytes(got)


def write(port, addr, value):
    xfer(port, bytes([WRITE, addr, value >> 16 & 0xFF, value >> 8 & 0xFF, value & 0xFF,
                      checksum(WRITE, addr, value)]))


def read(port, addr):
    got = xfer(port, bytes([READ, addr, 0, 0, 0, 0]))
    value = got[2] << 16 | got[3] << 8 | got[4]
    assert got[5] == checksum(READ, addr, value)
    return value


def test_registers_need_the_unlock_and_read_back():
    port, chip = Port(), BlChip()
    GpioSpiSlave(port, chip)
    write(port, 0x93, 0xC3)
    assert read(port, 0x93) == 0                 # locked: ignored
    write(port, 0x9E, 0x5555)
    assert read(port, 0x9E) == 0x5555
    write(port, 0x93, 0xC3)
    assert read(port, 0x93) == 0xC3
    chip.regs[0x22] = 0xFF0000                   # a measurement the model sets
    assert read(port, 0x22) == 0xFF0000


def test_a_bad_checksum_is_dropped():
    port, chip = Port(), BlChip()
    GpioSpiSlave(port, chip)
    write(port, 0x9E, 0x5555)
    xfer(port, bytes([WRITE, 0x93, 0, 0, 0xC3, 0x00]))
    assert chip.bad_frames == 1 and chip.regs.get(0x93) is None
