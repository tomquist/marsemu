"""The IQmath lookup tables of the F2803x boot ROM, computed from their definitions.

TI's IQmath library, linked into application flash, takes its Newton-Raphson
seeds and its rounding and saturation limits from tables in the boot ROM
(SPRUI10 section 2.1.1, "IQmathTables" at 0x3FE000).  An empty ROM makes
every ``_IQdiv`` return 0, every ``_IQsqrt`` 0, and so on.  The tables are
plain mathematics, so they are generated here rather than copied: the layout
follows SPRUI10's table sizes, and the meaning of each word follows how the
library code indexes it.

Only the tables the library's division, square root and rounding multiply
use are filled; arctan, exp and asin stay 0.
"""

from __future__ import annotations

import math
from typing import Dict

BASE = 0x3FE000
SIN = BASE                      # 1282 words: sin(2*pi*i/512), i = 0..640, Q30
INV = SIN + 1282                # 528 words: inverse seeds plus limits, Q29
SQRT = INV + 528                # 274 words: inverse square root seeds plus limits, Q30
ATAN = SQRT + 274               # 452 words (not generated)
ROUND = ATAN + 452              # 360 words: rounding and saturation for Q30 down to Q1
END = ROUND + 360               # 0x3FEB50

M32 = 0xFFFFFFFF


def _q(v: float, n: int) -> int:
    return int(round(v * (1 << n))) & M32


def _limits(round_bit: int, top: int) -> list:
    """Round, max and min of a 64-bit intermediate whose result is bits
    [top+31 .. top]: {round lo, hi, max lo, hi, min lo, hi}."""
    rnd = 1 << round_bit if round_bit >= 0 else 0
    hi = (1 << (top + 31)) - 1
    lo = (-(1 << (top + 31))) & ((1 << 64) - 1)
    return [rnd & M32, rnd >> 32, hi & M32, hi >> 32, lo & M32, lo >> 32]


def tables() -> Dict[int, int]:
    """32-bit values by (even) word address."""
    t: Dict[int, int] = {}
    for i in range(641):
        t[SIN + 2 * i] = _q(math.sin(2 * math.pi * i / 512), 30)
    # division: y <- y * (2 - x*y) from a seed for the normalized divisor's
    # top bits; the result is the 64-bit product >> 1
    t[INV] = _q(2.0, 29)
    for k, v in enumerate(_limits(0, 1)):
        t[INV + 2 + 2 * k] = v
    for w in range(0, 512, 2):   # word offset = (AH >> 6) - 0x100 of the normalized divisor
        lo = (256 + w) / 512 if w < 256 else -1 + (w - 256) / 512
        t[INV + 16 + w] = _q(1 / (lo + 1 / 512), 29)
    # square root: y <- y * (1.5 - x*y*y/2) from a seed, then scaled by a
    # constant chosen by the parity of the normalization shift
    t[SQRT] = _q(1.5, 30)
    lim = _limits(15, 16)
    for k, v in enumerate(lim[:4]):
        t[SQRT + 2 + 2 * k] = v
    t[SQRT + 10] = _q(1.0, 30)
    t[SQRT + 12] = _q(0.5, 30)
    t[SQRT + 14] = _q(math.sqrt(0.5), 30)
    for w in range(0, 258, 2):   # word offset = (AH >> 6) - 254
        x = (w + 255) / 512
        t[SQRT + 16 + w] = _q(1 / math.sqrt(x), 30)
    # IQmpy with rounding and saturation, entry i for Q = 30 - i
    for i in range(30):
        q = 30 - i
        for k, v in enumerate(_limits(q - 1, q)):
            t[ROUND + 12 * i + 2 * k] = v
    return t


def install(mem) -> None:
    """Write the tables into a word array covering the boot ROM."""
    for a, v in tables().items():
        mem[a] = v & 0xFFFF
        mem[a + 1] = v >> 16
