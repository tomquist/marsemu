"""Wait loops that only reload the watchdog are found, and only those."""
import struct

from marsemu.analysis.idle import find_watchdog_waits, watchdog_reload_functions

BASE = 0x08000000


def bl(src: int, dst: int) -> bytes:
    off = dst - (src + 4)
    s, i1, i2 = (off >> 24) & 1, (off >> 23) & 1, (off >> 22) & 1
    j1, j2 = (~i1 ^ s) & 1, (~i2 ^ s) & 1
    hw1 = 0xF000 | (s << 10) | ((off >> 12) & 0x3FF)
    hw2 = 0xD000 | (j1 << 13) | (j2 << 11) | ((off >> 1) & 0x7FF)
    return struct.pack("<HH", hw1, hw2)


def image(counter_addr: int) -> bytes:
    """IWDG_ReloadCounter at 0x00 and ``while (*counter < r5) reload();`` at 0x10."""
    code = b"".join([
        struct.pack("<H", 0x4902),               # 00 ldr  r1, [pc, #8]      (IWDG)
        struct.pack("<HH", 0xF64A, 0x20AA),      # 02 movw r0, #0xaaaa
        struct.pack("<H", 0x6008),               # 06 str  r0, [r1]
        struct.pack("<H", 0x4770),               # 08 bx   lr
        struct.pack("<H", 0xBF00),               # 0a nop
        struct.pack("<I", 0x40003000),           # 0c literal: IWDG
        struct.pack("<H", 0x4C03),               # 10 ldr  r4, [pc, #12]     (counter)
        bl(BASE + 0x12, BASE + 0x00),            # 12 bl   IWDG_ReloadCounter
        struct.pack("<H", 0x8821),               # 16 ldrh r1, [r4]
        struct.pack("<H", 0x42A9),               # 18 cmp  r1, r5
        struct.pack("<H", 0xD3FA),               # 1a blo  0x12
        struct.pack("<H", 0x4770),               # 1c bx   lr
        struct.pack("<H", 0xBF00),               # 1e nop
        struct.pack("<I", counter_addr),         # 20 literal: the counter
    ])
    return code + b"\xff" * 0x40


def test_reload_function_is_found():
    assert watchdog_reload_functions(image(0x20000100), BASE) == [BASE]


def test_delay_loop_on_a_ram_counter_is_a_wait():
    assert find_watchdog_waits(image(0x20000100), BASE) == [BASE + 0x12]


def test_loop_polling_a_peripheral_is_not():
    # a flag a peripheral sets without an interrupt must not wait for the next one
    assert find_watchdog_waits(image(0x40010808), BASE) == []


def test_no_watchdog_no_waits():
    assert find_watchdog_waits(b"\x00\xbf" * 64, BASE) == []
