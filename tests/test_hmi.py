"""HMI (Saturn M) plant around the F2803x model: no firmware needed."""
import math

import pytest

from marsemu.c28x import f2803x
from marsemu.c28x.machine import C28xMachine
from marsemu.plant.b2500 import PvArray
from marsemu.plant.hmi import (CH_GRID_N, CH_GRID_P, CH_PV1_V, GRID_COUNTS_PER_V, PV_V_PER_COUNT, RELAY_GPIO,
                               Flyback, HmiPlant)


class _Emu:
    def __init__(self):
        self.machine = m = C28xMachine()
        f2803x.build(m)


def _advance(m, seconds):
    clk = m.clock
    end = clk.cycles + int(seconds * clk.hz)
    while True:
        nxt = clk.next_deadline()
        if nxt is None or nxt > end:
            break
        clk.cycles = max(clk.cycles, nxt)
        clk.run_due()
    clk.cycles = end


class _Pwm:
    def __init__(self, d):
        self.d = d

    def duty(self, ch="A"):
        return self.d


def test_open_circuit_and_grid_sine():
    emu = _Emu()
    plant = HmiPlant(emu, pv=(300.0, 0.0), voc=(45.0, 45.0))
    m = emu.machine
    assert plant._adc(CH_PV1_V) * PV_V_PER_COUNT == pytest.approx(45.0, abs=0.05)
    # a quarter period in: the grid at its positive peak, differential around mid-scale
    _advance(m, 0.005)
    p, n = plant._adc(CH_GRID_P), plant._adc(CH_GRID_N)
    assert p + n == pytest.approx(4096, abs=2)
    assert (p - n) / GRID_COUNTS_PER_V == pytest.approx(230 * math.sqrt(2), rel=0.01)


def test_zero_crossings_reach_ecap_with_the_grid_period():
    emu = _Emu()
    plant = HmiPlant(emu, grid_hz=50.0)
    m = emu.machine
    ecap = m.get("ECAP1")
    # what the HMI firmware sets: falling and rising edges alternately, each resetting the counter
    ecap.write(0x14, 0x1BB)
    ecap.write(0x15, 0x16)
    _advance(m, 0.2)
    assert all(c == pytest.approx(m.clock.hz / 100, rel=0.001) for c in ecap.caps)
    plant.set_grid(False)
    before = list(ecap.caps)
    _advance(m, 0.1)
    assert ecap.caps == before


def test_relay_follows_gpio21():
    emu = _Emu()
    plant = HmiPlant(emu)
    gp = emu.machine.gpio
    gp.dir[0] |= 1 << RELAY_GPIO
    gp.set_out(0, gp.out[0] | 1 << RELAY_GPIO)
    assert plant.relay
    gp.set_out(0, gp.out[0] & ~(1 << RELAY_GPIO))
    assert not plant.relay


def test_flyback_settles_on_the_iv_curve():
    arr = PvArray(400.0, voc=49.0)
    fb = Flyback(arr, (_Pwm(0.15), _Pwm(0.15)))
    for _ in range(2000):
        fb.step(20e-6, fb.conductance())
    # input current matches what the array delivers at that voltage
    assert fb.i == pytest.approx(arr.current(fb.v), rel=0.01)
    assert 0 < fb.p <= 400.0 + 1e-6
    # no switching: back to open circuit
    fb.epwms = (_Pwm(0.0), _Pwm(0.0))
    fb.step(1.0, fb.conductance())
    assert fb.v == pytest.approx(49.0, abs=0.01) and fb.p == 0.0
