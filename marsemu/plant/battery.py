"""Electrical/thermal model of an LFP battery pack.

Deliberately simple but physical: coulomb counting per cell (cells can be
given different capacities and start SOCs to model imbalance), a typical LFP
open-circuit-voltage curve, an ohmic plus one RC polarisation term, and a
lumped thermal model.  Current is positive when charging (BMS convention).
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import List

# Typical LFP (graphite/LiFePO4) OCV curve at 25 degC, SOC -> V.
LFP_OCV = [
    (0.00, 2.500), (0.01, 2.900), (0.02, 3.050), (0.05, 3.180), (0.10, 3.220),
    (0.15, 3.245), (0.20, 3.260), (0.30, 3.280), (0.40, 3.292), (0.50, 3.300),
    (0.60, 3.310), (0.65, 3.320), (0.70, 3.328), (0.80, 3.333), (0.90, 3.338),
    (0.95, 3.345), (0.97, 3.360), (0.98, 3.390), (0.99, 3.450), (0.995, 3.520),
    (1.00, 3.600),
]


def _interp(table, x):
    xs = [a for a, _ in table]
    i = bisect.bisect_left(xs, x)
    if i <= 0:
        return table[0][1]
    if i >= len(table):
        return table[-1][1]
    (x0, y0), (x1, y1) = table[i - 1], table[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


@dataclass
class Cell:
    capacity_ah: float
    soc: float
    r_ohm: float = 0.0008          # DC resistance per cell
    rc_r: float = 0.0006           # polarisation
    rc_tau: float = 60.0
    v_rc: float = 0.0

    def ocv(self) -> float:
        return _interp(LFP_OCV, min(1.0, max(0.0, self.soc)))

    def voltage(self, current_a: float) -> float:
        return self.ocv() + current_a * self.r_ohm + self.v_rc


@dataclass
class LfpPack:
    """``cells`` LFP cells in series."""

    cells_n: int = 16
    capacity_ah: float = 100.0
    soc: float = 0.5
    temp_c: float = 25.0
    ambient_c: float = 25.0
    thermal_mass_j_per_k: float = 40_000.0   # ~ 50 kg pack
    cooling_w_per_k: float = 2.0
    spread: float = 0.0                     # +-fraction of SOC imbalance across cells
    cells: List[Cell] = field(default_factory=list)
    current_a: float = 0.0
    charged_ah: float = 0.0
    discharged_ah: float = 0.0

    def __post_init__(self):
        if not self.cells:
            for i in range(self.cells_n):
                off = 0.0
                if self.spread and self.cells_n > 1:
                    off = self.spread * (2 * i / (self.cells_n - 1) - 1)
                self.cells.append(Cell(self.capacity_ah, min(1.0, max(0.0, self.soc + off))))

    # -- state
    @property
    def cell_voltages(self) -> List[float]:
        return [c.voltage(self.current_a) for c in self.cells]

    @property
    def voltage(self) -> float:
        return sum(self.cell_voltages)

    @property
    def ocv(self) -> float:
        return sum(c.ocv() for c in self.cells)

    @property
    def mean_soc(self) -> float:
        return sum(c.soc for c in self.cells) / len(self.cells)

    def max_cell_v(self) -> float:
        return max(self.cell_voltages)

    def min_cell_v(self) -> float:
        return min(self.cell_voltages)

    # -- dynamics
    def current_for_power(self, power_w: float) -> float:
        """Current (A, + = charge) that moves ``power_w`` (+ = into the pack)."""
        # P = I * (OCV + Vrc + I*R)  ->  solve the quadratic for I
        r = sum(c.r_ohm for c in self.cells)
        e = self.ocv + sum(c.v_rc for c in self.cells)
        disc = e * e + 4 * r * power_w
        if disc <= 0:
            return -e / (2 * r)
        return (-e + math.sqrt(disc)) / (2 * r)

    def step(self, dt: float, current_a: float) -> None:
        """Advance by ``dt`` seconds with ``current_a`` flowing (+ = charge)."""
        self.current_a = current_a
        for c in self.cells:
            c.soc = min(1.0, max(0.0, c.soc + current_a * dt / 3600.0 / c.capacity_ah))
            a = math.exp(-dt / c.rc_tau)
            c.v_rc = c.v_rc * a + current_a * c.rc_r * (1 - a)
        if current_a > 0:
            self.charged_ah += current_a * dt / 3600.0
        else:
            self.discharged_ah -= current_a * dt / 3600.0
        heat = current_a * current_a * sum(c.r_ohm + c.rc_r for c in self.cells)
        self.temp_c += dt * (heat - self.cooling_w_per_k * (self.temp_c - self.ambient_c)) / self.thermal_mass_j_per_k
        self.soc = self.mean_soc
