"""The native F2803x peripherals (PIE, ePWM, ADC, CLA dispatch) against the
Python models they mirror: the same register script on both, the same
observations.  No firmware needed; skipped without the native core."""
import pytest

from marsemu.c28x.f2803x import build
from marsemu.c28x.machine import C28xMachine


def machines():
    from marsemu.c28x.cpu import accelerate
    from marsemu.c28x.f2803x_native import accelerate_peripherals
    py = C28xMachine()
    build(py)
    py.reset()
    nat = C28xMachine()
    build(nat)
    if not accelerate(nat.cpu) or not accelerate_peripherals(nat):
        pytest.skip("the native C28x core is not built")
    nat.reset()
    return py, nat


EPWM1, EPWM2, ADC, PIE, RES = 0x6800, 0x6840, 0x7100, 0x0CE0, 0x0B00


def script(m):
    """Set up ePWM1 -> SOCA -> ADC -> ADCINT1/2 -> PIE, ePWM2 synced to it,
    and return what the CPU would read at points along the way."""
    w = m.cpu.wr16
    r = m.cpu.rd16
    obs = []
    m.adc.source = lambda ch: 100 * ch + 7
    m.cpu.eallow = 1

    def look(tag):
        obs.append((tag, m.clock.cycles, [r(a) for a in (EPWM1 + 4, EPWM1 + 1, EPWM1 + 0x1B, EPWM2 + 4, EPWM2 + 1,
                                                          ADC + 4, ADC + 6, PIE + 3, PIE + 7, PIE + 1, EPWM1 + 9,
                                                          EPWM1 + 7, EPWM1 + 0x16)],
                    [r(RES + i) for i in range(4)], m.cpu.ifr,
                    round(m.epwm[1].duty("A"), 6), round(m.epwm[2].duty("A"), 6), m.epwm[1].outputs_enabled(),
                    m.epwm[1].period_cycles(), m.epwm[2].counter()))

    w(PIE, 1)                                    # ENPIE
    w(PIE + 2, 0x03)                             # PIEIER1: ADCINT1, ADCINT2
    w(PIE + 6, 0x01)                             # PIEIER3: EPWM1_INT
    w(ADC + 0x20, (5 << 11) | (3 << 6) | 6)      # SOC0: ePWM1 SOCA, A3
    w(ADC + 0x21, (5 << 11) | (8 << 6) | 6)      # SOC1: ePWM1 SOCA, B0
    w(ADC + 0x22, (2 << 6) | 6)                  # SOC2: A2, started by ADCINT1
    w(ADC + 0x23, (5 << 6) | 9)                  # SOC3: A5 (the temperature sensor)
    w(ADC + 0x14, 1 << 4)                        # ADCINTSOCSEL1: SOC2 <- ADCINT1
    w(ADC + 0x8, 0x21 | (0x22 << 8))             # INT1 <- EOC1, INT2 <- EOC2
    w(ADC + 0x0, 0x4001)                         # ADCENABLE, TEMPCONV
    w(EPWM1 + 5, 599)                            # TBPRD
    w(EPWM1 + 7, 0)                              # CMPCTL: shadow, load at zero
    w(EPWM1 + 9, 150)                            # CMPA
    w(EPWM1 + 0x0B, 0x12)                        # AQCTLA: set at zero, clear at CMPA up
    w(EPWM1 + 0x14, 0x0A)                        # TZCTL: force low
    w(EPWM1 + 0x19, 0x0900 | 0x9)                # ETSEL: SOCA and INT at zero
    w(EPWM1 + 0x1A, 0x0101)                      # ETPS: every period
    w(EPWM2 + 5, 299)
    w(EPWM2 + 3, 100)                            # TBPHS
    w(EPWM2 + 0x0B, 0x60 | 0x2)                  # AQCTLA: set at zero, toggle down ...
    w(EPWM2 + 9, 120)
    w(EPWM2 + 0, 0x0002 | 0x0004 | 0x2000)       # up-down, PHSEN, PHSDIR up
    w(EPWM1 + 0, 0x0010)                         # up, SYNCOSEL = CTR=0
    look("start")
    for k in range(6):
        m.advance(3.3e-6)
        look(f"t{k}")
    w(ADC + 5, 3)                                # ADCINTFLGCLR
    w(PIE + 1, 0xFFF)                            # PIEACK
    w(EPWM1 + 0x1C, 1)                           # ETCLR
    w(EPWM1 + 9, 400)                            # CMPA, loaded at the next zero
    look("cmpa written")
    m.advance(10e-6)
    look("next period")
    w(EPWM1 + 0x18, 4)                           # TZFRC: one-shot trip
    look("tripped")
    w(EPWM1 + 0x17, 5)                           # TZCLR
    w(ADC + 0x1A, 0x8)                           # ADCSOCFRC1: SOC3
    m.advance(2e-6)
    look("cleared, SOC3 forced")
    w(EPWM1 + 0, 0x0010 | (1 << 7))              # HSPCLKDIV /2
    w(EPWM1 + 4, 17)                             # TBCTR
    m.advance(50e-6)
    look("divided")
    obs.append(("pie counts", sorted(m.pie.counts.items()), m.adc.conversions, m.epwm[1].ints, m.epwm[1].socas))
    return obs


def test_native_peripherals_match_the_python_models():
    py, nat = machines()
    a, b = script(py), script(nat)
    for x, y in zip(a, b):
        assert x == y, f"{x[0]}:\n python {x}\n native {y}"
    assert len(a) == len(b)
    # and it went through the chain: conversions, ePWM interrupts, results of
    # SOC0/1 (ePWM1 SOCA), SOC2 (started by ADCINT1), SOC3 (the temperature sensor)
    assert a[-1][2] >= 10 and a[-1][3] == 2
    assert a[-2][3][:3] == [307, 807, 207] and a[-2][3][3] > 1000


def test_native_cla_dispatch_matches():
    """A CLA task started by ADCINT1 (MPISRCSEL1 = 0) loads MR0 and ends;
    the end-of-task interrupt reaches PIE group 11 once its time is up."""
    from marsemu.c28x.cla import tobits_
    py, nat = machines()
    res = []
    for m in (py, nat):
        prog = [(0x7840, 0x3F80), (0x7FA0, 0), (0x7FA0, 0), (0x7F80, 0)]    # MMOVIZ MR0,#1.0; MNOPs; MSTOP
        for i, (msw, lsw) in enumerate(prog):
            m.cpu.mem[0x9000 + 2 * i] = lsw
            m.cpu.mem[0x9001 + 2 * i] = msw
        m.cpu.eallow = 1
        cla = m.get("CLA1")
        cla.write16(0x1400 + 0x00, 0x000)                  # MVECT1 -> 0x9000
        cla.write16(0x1400 + 0x25, 0x01)                   # MIER: task 1
        m.cpu.wr16(PIE, 1)
        m.cpu.wr16(PIE + 2 * 11, 0x01)                     # PIEIER11: task 1's end
        m.cla.peripheral_irq("adc", 1)
        before = (m.cla.running, m.pie.ifr[11])
        m.advance(1e-5)
        m.cla.sync_from_core()
        res.append((before, m.cla.task_counts[1], m.pie.ifr[11], m.cla.mifr, m.cla.mirun, m.cla.running,
                    tobits_(m.cla.mr[0]), m.cpu.ifr, cla.read16(0x1400 + 0x26)))
    assert res[0] == res[1]
    assert res[0][1] == 1 and res[0][6] == 0x3F800000 and res[0][2] == 1
