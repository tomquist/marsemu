"""TMS320C28x CPU and CLA: instruction semantics on hand-assembled code."""
import math
import struct

import pytest

from marsemu.c28x import isa
from marsemu.c28x.cla import b2f, tobits_
from marsemu.c28x.cpu import RAM
from marsemu.c28x.f2803x import build
from marsemu.c28x.machine import C28xMachine

CODE = 0x8000


def machine(words, entry=CODE):
    m = C28xMachine()
    build(m)
    m.cpu.mem[entry:entry + len(words)] = __import__("array").array("H", words)
    m.entry = entry
    m.reset()
    return m


def _run_one(words, steps, setup, native):
    m = machine(words)
    if setup:
        setup(m)
    if native:
        from marsemu.c28x.cpu import accelerate
        assert accelerate(m.cpu)
    end = CODE + len(words) - 1
    for _ in range(steps or 2000):
        if m.cpu.pc == end:
            break
        m.cpu.run(1)
    assert m.cpu.pc == end, f"did not finish, pc=0x{m.cpu.pc:06x}"
    return m.cpu


def _have_native():
    try:
        from marsemu.c28x import _c28x  # noqa: F401
        return True
    except ImportError:
        return False


def run(words, steps=None, setup=None):
    """Run ``words`` (ending in ``SB $,UNC``) until the CPU spins there, on
    the Python core and, when it is built, on the native one: both must end
    in the same registers and memory."""
    words = list(words) + [0x6F00]   # SB $,UNC
    c = _run_one(words, steps, setup, False)
    if _have_native():
        from marsemu.c28x.cpu import REGISTERS
        n = _run_one(words, steps, setup, True)
        n.flush()
        for r in REGISTERS:
            if r != "insn_count":
                assert getattr(n, r) == getattr(c, r), r
        assert list(n.xar) == list(c.xar)
        assert n.mem[0:0x10000] == c.mem[0:0x10000]
    return c


def f32(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]


def test_disassembly():
    rd = {0: 0x761F, 1: 0x01A1, 2: 0x9201, 3: 0x56C4, 4: 0x0010}.get
    assert isa.disasm(rd, 0) == ("MOVW DP,#0x1a1", 2)
    assert isa.disasm(rd, 2) == ("MOV AL,@0x1", 1)
    assert isa.disasm(lambda a: [0x8F00, 0x8301][a], 0)[0] == "MOVL XAR4,#0x8301"


def test_mov_add_flags():
    # MOVB AL,#0xFF ; ADDB AL,#1 -> AL = 0x100, no carry from 16 bits
    c = run([0x9AFF, 0x9C01])
    assert c.acc & 0xFFFF == 0x100 and c.c == 0 and c.z == 0
    # MOV AL,#0xFFFF (MOV @AL,#imm) ; ADDB AL,#1 -> 0, carry, zero
    c = run([0x28A9, 0xFFFF, 0x9C01])
    assert c.acc & 0xFFFF == 0 and c.c == 1 and c.z == 1


def test_cmp_infinite_precision():
    # AL = 0x8000, CMPB AL,#1 -> negative (0x8000 - 1 < 0 signed)
    c = run([0x28A9, 0x8000, 0x5201])
    assert c.n == 1 and c.c == 1 and c.z == 0


def test_subcu_division():
    # ACC = 1000 (AH=0, AL=1000), divide by 7 with RPT #15 || SUBCU ACC,*-SP[1]
    # stack: MOV *-SP[1] needs SP space: ADDB SP,#2 ; MOV *-SP[1],#7
    c = run([0xFE02, 0x2841, 0x0007, 0xFF20, 1000, 0xF60F, 0x1F41])
    assert c.acc & 0xFFFF == 1000 // 7 and c.acc >> 16 == 1000 % 7


def test_subcul_division():
    # P = 123456789, ACC = 0, divisor at *-SP[2] (32 bit) = 1000
    words = [0xFE04,                    # ADDB SP,#4
             0x8F00, 1000 >> 0,          # MOVL XAR4,#1000
             0xA842,                    # MOVL *-SP[2],XAR4
             0xFF20, 123456789 >> 16, 0xFF3F,   # MOV ACC,#hi<<#0 ; LSL ACC,#16
             0x18A9, 0x0000]            # placeholder replaced below
    # simpler: build P via MOV PH/PL from immediates through AL
    words = [0xFE04, 0x8F00, 1000, 0xA842,
             0x28A9, 123456789 >> 16, 0x2FA9,          # MOV AL,#hi ; MOV PH,@AL
             0x28A9, 123456789 & 0xFFFF, 0x27A9,       # MOV AL,#lo ; MOV PL,@AL
             0x0200,                                   # MOVB ACC,#0
             0xF61F, 0x5617, 0x0042]                   # RPT #31 || SUBCUL ACC,*-SP[2]
    c = run(words)
    assert c.p == 123456789 // 1000 and c.acc == 123456789 % 1000


def test_lcr_lretr_nest():
    # LCR sub ; SB $ ... sub: LCR sub2 ; LRETR ; sub2: MOVB AL,#5 ; LRETR
    words = [0x7640, CODE + 4,          # 0: LCR 0x8004
             0x6F07,                    # 2: SB +7 (to the end)
             0x7700,                    # 3: NOP
             0x7640, CODE + 7,          # 4: LCR 0x8007
             0x0006,                    # 6: LRETR
             0x9A05, 0x0006]            # 7: MOVB AL,#5 ; LRETR
    m = machine(words + [0x6F00])
    sp0 = m.cpu.sp
    for _ in range(50):
        m.cpu.run(1)
    assert m.cpu.acc & 0xFFFF == 5 and m.cpu.sp == sp0 and m.cpu.pc == CODE + 9


def test_qmpyl_impyl():
    a, b = -123456789, 987654
    words = [0x8F00 | ((a & 0xFFFFFFFF) >> 16 & 0x3F), (a & 0xFFFF)]   # placeholder
    prod = a * b

    def setup(m):
        m.cpu.xt = a & 0xFFFFFFFF
        m.cpu.mem[0x8800] = b & 0xFFFF
        m.cpu.mem[0x8801] = (b >> 16) & 0xFFFF
        m.cpu.dp = 0x8800 >> 6
    c = run([0x5663, 0x0000], setup=setup)          # QMPYL ACC,XT,@0
    assert c.acc == (prod >> 32) & 0xFFFFFFFF
    c = run([0x5605, 0x0000], setup=setup)          # IMPYL P,XT,@0
    assert c.p == prod & 0xFFFFFFFF


def test_addressing_post_increment_and_32bit_alignment():
    def setup(m):
        m.cpu.xar[4] = 0x8801     # odd: 32-bit accesses use 0x8800
        m.cpu.mem[0x8800] = 0x1111
        m.cpu.mem[0x8801] = 0x2222
    c = run([0x0684], setup=setup)                  # MOVL ACC,*XAR4++
    assert c.acc == 0x22221111 and c.xar[4] == 0x8803


def test_interrupt_roundtrip():
    """Context save and IRET restore everything the hardware saves."""
    words = [0x6F00]
    m = machine(words)
    c = m.cpu
    # ISR at 0x8100: MOVB AL,#0 ; MOVB ACC,#0 ; IRET
    c.mem[0x8100:0x8103] = __import__("array").array("H", [0x9A00, 0x0200, 0x7602])
    m.pie.ctrl = 0     # use the CPU vector table in RAM... route INT1 via our own hook
    c.vector_hook = lambda k: 0x8100
    c.acc, c.p, c.xt, c.dp = 0x12345678, 0xCAFEBABE, 0x55AA0000, 0x123
    c.xar[0], c.xar[1] = 0xAAAA1111, 0xBBBB2222
    c.intm, c.ier = 0, 1
    sp0 = c.sp
    st0, st1 = c.st0, c.st1
    m.interrupt(1)
    for _ in range(10):
        c.run(1)
    assert c.pc == CODE and c.acc == 0x12345678 and c.p == 0xCAFEBABE and c.sp == sp0
    assert c.st0 == st0 and c.st1 == st1 and c.ier == 1 and c.dp == 0x123
    assert c.xar[0] == 0xAAAA1111 and c.xar[1] == 0xBBBB2222


def test_rpt_add_accumulates():
    # 4 words of 10 at 0x8800; XAR4 = 0x8800; MOVB ACC,#0 ; RPT #3 || ADD ACC,*XAR4++
    def setup(m):
        m.cpu.xar[4] = 0x8800
        for i in range(4):
            m.cpu.mem[0x8800 + i] = 10
    c = run([0x0200, 0xF603, 0x8184], setup=setup)
    assert c.acc == 40 and c.xar[4] == 0x8804


def test_movb_byte_addressing():
    def setup(m):
        m.cpu.xar[4] = 0x8800
        m.cpu.mem[0x8801] = 0xABCD
    # MOVB AL.LSB,*+XAR4[3] -> word 0x8801, high byte
    c = run([0xC6DC], setup=setup)
    assert c.acc & 0xFFFF == 0xAB


def test_cla_task_delay_slots_and_flags():
    """A CLA branch decides on the flags from four instructions back and
    always executes its three delay slots."""
    m = machine([0x6F00])
    cla = m.cla
    prog = [
        (0x7840, 0x3F80),    # MMOVIZ MR0,#1.0
        (0x7FA0, 0), (0x7FA0, 0), (0x7FA0, 0),
        (0x78C0, 0x4000),    # MCMPF32 MR0,#2.0 -> NF=1 (I1)
        (0x7FA0, 0), (0x7FA0, 0), (0x7FA0, 0),
        (0x7984, 8),         # MBCNDD +8 (to after the slots + 2), LT
        (0x7841, 0x4040),    # slot: MMOVIZ MR1,#3.0
        (0x7FA0, 0), (0x7FA0, 0),
        (0x7842, 0x4080),    # skipped if taken: MR2 = 4.0
        (0x7F80, 0),         # target: MSTOP
    ]
    # branch at index 8 (0x9010): target = 0x9010 + 10 = 0x901a (index 13)
    prog[8] = (0x7984, 10)
    for i, (msw, lsw) in enumerate(prog):
        m.cpu.mem[0x9000 + 2 * i] = lsw
        m.cpu.mem[0x9001 + 2 * i] = msw
    cla.regs[1] = 0      # MVECT2
    cla.mier = 2
    cla.force(2)
    assert b2f(tobits_(cla.mr[1])) == 3.0 and tobits_(cla.mr[2]) == 0
    m.clock.cycles += 100
    m.clock.run_due()
    assert m.pie.ifr[11] & 2


def test_cla_float_ops():
    m = machine([0x6F00])
    cla = m.cla
    cla.mr = [f32(1.5), f32(-2.25), 0, 0]
    prog = [(0x7C00, 0x0004 | 0 | (1 << 2) | (0 << 4)),   # MMPYF32 MR0,MR1,MR0
            (0x7DE0, 0x0001),                             # MF32TOI16 MR1,MR0
            (0x7F80, 0)]
    for i, (msw, lsw) in enumerate(prog):
        m.cpu.mem[0x9000 + 2 * i] = lsw
        m.cpu.mem[0x9001 + 2 * i] = msw
    cla.mier = 1
    cla.force(1)
    assert math.isclose(b2f(tobits_(cla.mr[0])), -3.375)
    assert tobits_(cla.mr[1]) == (-3) & 0xFFFFFFFF


@pytest.mark.parametrize("acc,p,want", [
    (0x00000001, 0x00000005, (0x00000000, 0xFFFFFFFF)),   # above the maximum
    (0x00000000, 0x00000007, (0x00000000, 0x00000007)),   # inside
    (0xFFFFFFFE, 0x00000003, (0xFFFFFFFF, 0x00000000)),   # below the minimum
    (0xFFFFFFFF, 0x80000000, (0xFFFFFFFF, 0x80000000)),   # inside, negative
])
def test_64bit_saturation(acc, p, want):
    # IQmath's 64-bit clamp: MINL ACC,*+XAR7[2]; MINCUL P,*+XAR7[0];
    # MAXL ACC,*+XAR7[6]; MAXCUL P,*+XAR7[4] against max 0:FFFFFFFF, min FFFFFFFF:0
    def setup(m):
        lim = (0xFFFFFFFF, 0, 0, 0xFFFFFFFF)
        for k, v in enumerate(lim):
            m.cpu.mem[0x8800 + 2 * k] = v & 0xFFFF
            m.cpu.mem[0x8801 + 2 * k] = v >> 16
        m.cpu.xar[7] = 0x8800
        m.cpu.acc, m.cpu.p = acc, p
    c = run([0x5650, 0x00D7, 0x5659, 0x00C7, 0x5661, 0x00F7, 0x5651, 0x00E7], setup=setup)
    assert (c.acc, c.p) == want


def test_iqmath_tables():
    from marsemu.c28x import iqmath
    t = iqmath.tables()
    assert iqmath.END == 0x3FEB50          # SPRUI10's IQTABLES length
    q30 = 1 << 30
    assert t[iqmath.SIN + 2 * 128] == q30   # sin(pi/2)
    # division seeds: 1/x in Q29 for the normalized divisor's top bits
    assert abs(t[iqmath.INV + 16] / (1 << 29) - 1 / (257 / 512)) < 1e-6
    assert t[iqmath.INV] == 2 << 29
    # Q15 rounding entry: round 1<<14, max 2^46-1, min -2^46
    e = iqmath.ROUND + 12 * 15
    assert (t[e], t[e + 2], t[e + 6], t[e + 10]) == (1 << 14, 0, (1 << 14) - 1, 0xFFFFC000)
