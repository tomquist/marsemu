"""Wait loops that only reload the watchdog are found, and only those."""
import struct

from marsemu.analysis.idle import find_idle_loop, find_watchdog_waits, watchdog_reload_functions

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


def test_counter_address_loaded_inside_the_loop():
    """The B2500 V232 form: the counter's address is loaded inside the loop, and
    a store just before it uses the same register (a store does not set it)."""
    code = image(0x20000100)[:0x10] + b"".join([
        struct.pack("<H", 0x2000),               # 10 movs r0, #0
        struct.pack("<H", 0x4905),               # 12 ldr  r1, [pc, #20]     (counter)
        struct.pack("<H", 0x8008),               # 14 strh r0, [r1]          (counter = 0)
        struct.pack("<H", 0xE001),               # 16 b    0x1c
        bl(BASE + 0x18, BASE + 0x00),            # 18 bl   IWDG_ReloadCounter
        struct.pack("<H", 0x4802),               # 1c ldr  r0, [pc, #8]      (counter)
        struct.pack("<H", 0x8800),               # 1e ldrh r0, [r0]
        struct.pack("<H", 0x42A0),               # 20 cmp  r0, r4
        struct.pack("<H", 0xDBF9),               # 22 blt  0x18
        struct.pack("<H", 0x4770),               # 24 bx   lr
        struct.pack("<H", 0xBF00),               # 26 nop
        struct.pack("<I", 0x20000100),           # 28 literal: the counter
    ]) + b"\xff" * 0x40
    assert find_watchdog_waits(code, BASE) == [BASE + 0x18]


def ucos_image() -> bytes:
    """uC/OS-III: ``OS_IdleTask`` at 0x00 (loop at 0x02), another function
    with a loop at 0x06, and ``OS_IdleTaskInit`` at 0x10 calling
    ``OSTaskCreate(r0 = TCB, r1 = "uC/OS-III Idle Task", r2 = OS_IdleTask)``;
    here r0 holds a code address too, which must not be taken for the entry."""
    code = b"".join([
        struct.pack("<H", 0x2400),               # 00 movs r4, #0          OS_IdleTask
        struct.pack("<H", 0xBF00),               # 02 nop                  (idle loop)
        struct.pack("<H", 0xE7FD),               # 04 b    0x02
        struct.pack("<H", 0xBF00),               # 06 nop                  (another loop)
        struct.pack("<H", 0xE7FD),               # 08 b    0x06
        struct.pack("<HHH", 0xBF00, 0xBF00, 0xBF00),  # 0a nops
        struct.pack("<H", 0x4802),               # 10 ldr  r0, [pc, #8]    (0x1c)
        struct.pack("<H", 0x4A01),               # 12 ldr  r2, [pc, #4]    (0x18: OS_IdleTask)
        struct.pack("<H", 0xA102),               # 14 adr  r1, #8          (0x20: the name)
        struct.pack("<H", 0x4770),               # 16 bx   lr
        struct.pack("<I", BASE + 0x01),          # 18 literal: OS_IdleTask | 1
        struct.pack("<I", BASE + 0x07),          # 1c literal: the other function | 1
        b"uC/OS-III Idle Task\x00",              # 20 the task name, after the code
    ])
    return code + b"\xff" * (-len(code) % 4) + b"\xff" * 0x40


def test_ucos_idle_task_loop_is_found():
    assert find_idle_loop(ucos_image(), BASE) == BASE + 0x02


def test_no_idle_task_name_no_loop():
    assert find_idle_loop(ucos_image().replace(b"uC/OS-III Idle Task", b"uC/OS-III Fast Task"), BASE) is None
