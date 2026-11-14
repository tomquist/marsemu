"""Unit tests of the CPU/exception model with small hand-assembled programs."""
import struct

from marsemu.core.machine import Machine
from marsemu.core.ucount import counter_offset
from marsemu.periph import n32g45x

BASE = 0x08000000


def thumb(*halfwords):
    return b"".join(struct.pack("<H", h) for h in halfwords)


def make(program: bytes, vectors: dict = None, sp=0x20008000) -> Machine:
    m = Machine()
    n32g45x.build(m)
    vt = [sp, BASE + 0x400 + 1] + [0] * 62
    for exc, addr in (vectors or {}).items():
        vt[exc] = addr | 1
    image = bytearray(b"\xff" * 0x1000)
    image[0:256] = struct.pack("<64I", *vt)
    image[0x400:0x400 + len(program)] = program
    return m, image


def boot(m, image):
    m.load_flash(bytes(image))
    m.reset()
    return m


def test_instruction_counter_found():
    assert counter_offset() is not None


def test_svc_and_exception_return():
    # main: movs r4,#0 ; loop: adds r4,#1 ; svc #0 ; b loop
    main = thumb(0x2400, 0x3401, 0xDF00, 0xE7FC)
    # SVC handler at 0x500: adds r5,#1 ; bx lr
    m, img = make(main, {11: BASE + 0x500})
    img[0x500:0x504] = thumb(0x3501, 0x4770)
    boot(m, img)
    m.run(cycles=2000)
    r4, r5 = m.reg("r4"), m.reg("r5")
    assert r4 > 10
    assert r5 in (r4, r4 - 1)
    assert m.reg("ipsr") in (0, 11)


def test_systick_interrupt_and_wfi_fast_forward():
    # main: cpsie i ; loop: wfi ; b loop
    main = thumb(0xB662, 0xBF30, 0xE7FD)
    m, img = make(main, {15: BASE + 0x500})
    img[0x500:0x504] = thumb(0x3601, 0x4770)  # adds r6,#1 ; bx lr
    boot(m, img)
    # SysTick every 1 ms at 144 MHz, core clock source
    m.bus.write(0xE000E014, 144_000 - 1)
    m.bus.write(0xE000E010, 7)
    m.run(seconds=0.1)
    assert 95 <= m.reg("r6") <= 101
    assert m.nvic.irq_count[15] == m.reg("r6")


def test_pendsv_is_taken_right_after_the_store():
    """FreeRTOS relies on PendSV running immediately after PENDSVSET."""
    # main: ldr r0,=ICSR ; ldr r1,=PENDSVSET ; str r1,[r0] ; movs r2,r6 ; b .
    main = thumb(0x4802, 0x4903, 0x6001, 0x0032, 0xE7FE, 0xBF00) + struct.pack("<II", 0xE000ED04, 1 << 28)
    m, img = make(main, {14: BASE + 0x500})
    img[0x500:0x504] = thumb(0x2607, 0x4770)  # PendSV: movs r6,#7 ; bx lr
    boot(m, img)
    m.run(cycles=10_000)
    # r2 = r6 copied *after* the store: PendSV must already have run
    assert m.reg("r2") == 7
    assert m.nvic.irq_count[14] == 1


def test_basepri_unmask_takes_pending_interrupt_immediately():
    # movs r0,#0x50 ; msr basepri,r0 ; ldr r1,=ICSR ; ldr r2,=PENDSVSET ; str r2,[r1]
    # movs r0,#0 ; msr basepri,r0 ; movs r3,r6 ; b .
    main = thumb(0x2050, 0xF380, 0x8811, 0x4904, 0x4A04, 0x600A, 0x2000, 0xF380, 0x8811, 0x0033, 0xE7FE,
                 0xBF00) + struct.pack("<II", 0xE000ED04, 1 << 28)
    m, img = make(main, {14: BASE + 0x500})
    img[0x500:0x504] = thumb(0x2609, 0x4770)  # PendSV: movs r6,#9 ; bx lr
    # PendSV priority 0xF0 (masked by BASEPRI 0x50)
    boot(m, img)
    m.bus.write(0xE000ED20, 0x00F00000)
    m.install_unmask_exits(bytes(img[0x400:0x500]), BASE + 0x400)
    m.run(cycles=10_000)
    assert m.reg("r3") == 9


def test_dma_push_many_matches_single_pushes():
    """An ADC sequence written in one go leaves memory, NDTR, the position and
    the flags (and their interrupt checks) exactly as item-by-item pushes."""
    import random
    from marsemu.periph.n32g45x import DmaChannel

    class Mem:
        def __init__(self):
            self.bytes = {}

        def write(self, a, d):
            for i, b in enumerate(d):
                self.bytes[a + i] = b

    class Dma:
        name = "DMA"

        def __init__(self):
            self.regs, self.m, self.irq_checks = {}, Mem(), []

        def update_irq(self, idx):
            self.irq_checks.append(self.regs[0])

    def channel(cfg, n, ht):
        d = Dma()
        ch = DmaChannel(d, 1)
        ch.half_transfer = ht
        d.regs.update({ch.off: cfg, ch.off + 4: n, ch.off + 12: 0x20000000})
        ch.total = n
        return d, ch

    rnd = random.Random(1)
    for _ in range(500):
        n = rnd.randint(1, 40)
        cfg = 1 | rnd.choice([0, 0x80]) | rnd.choice([0, 0x20]) | rnd.choice([0, 0x400, 0x800]) | rnd.choice([0, 0x2, 0x6])
        ht = rnd.random() < 0.5
        seqs = [[rnd.randint(0, 4095) for _ in range(rnd.randint(1, 12))] for _ in range(rnd.randint(1, 8))]
        a, ca = channel(cfg, n, ht)
        b, cb = channel(cfg, n, ht)
        for seq in seqs:
            for v in seq:
                ca.push(v)
            cb.push_many(seq)
        assert (a.m.bytes, a.regs, ca.pos, a.irq_checks) == (b.m.bytes, b.regs, cb.pos, b.irq_checks)
