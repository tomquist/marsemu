"""The F2803x peripherals the control loops hammer, in the native core.

The C2000 firmware's control loops run at tens of kHz: every period an ePWM
starts the ADC, the end of conversion interrupts the CPU and starts a CLA
task, and both write the ePWM compare and trip-zone registers.  Done by the
Python models, each of those accesses and timed events leaves the native core.
With the core built, :func:`accelerate_peripherals` moves the PIE, ePWM1-7,
the ADC and the CLA's task dispatch into it (``_f2803x.h``, a line-by-line
port of the Python models, which stay the reference and run when there is no
compiler) and turns these objects into facades of the native state: the
plants and the dashboard use them as before (``duty()``, ``cmpa``,
``adc.source``, ``pie.counts``, ...).

The board model's analog inputs are sampled once per conversion batch
(``adc.source(ch)`` for each channel the batch converts), as before.
"""
from __future__ import annotations

import os
from typing import List

from . import f2803x_adc, f2803x_pwm, f2803x_sys
from .cla import Cla
from .machine import C28xMachine, C28xPeripheral

(OP_POLL, OP_NEXT, OP_PIE_RAISE, OP_PIE_READ, OP_PIE_WRITE, OP_EP_READ, OP_EP_WRITE, OP_EP_DUTY, OP_EP_TRIP,
 OP_EP_RESET, OP_EP_COUNTER, OP_EP_DIRECTION, OP_EP_PERIOD, OP_EP_FLAGS, OP_ADC_READ, OP_ADC_WRITE, OP_ADC_RESET,
 OP_ADC_TRIGGER, OP_ADC_TEMP, OP_CLA_READ, OP_CLA_WRITE, OP_CLA_FORCE, OP_CLA_IACK, OP_CLA_IRQ, OP_CLA_RESET,
 OP_TRACE, OP_PIE_RESET, OP_PIE_CLEAR) = range(1, 29)
FP_PIE, FP_ADC, FP_CLA, FP_EPWM = 1, 2, 3, 4
#: "no event" as the core reports it
NONE_T = (1 << 63) - 1


class _NotifyList(list):
    """A list that tells its owner when it changes (listeners the native side must call)."""

    def __init__(self, items, changed):
        super().__init__(items)
        self._changed = changed

    def _wrap(name):
        def f(self, *a, **k):
            r = getattr(list, name)(self, *a, **k)
            self._changed()
            return r
        return f

    for _n in ("append", "extend", "insert", "remove", "pop", "clear", "__setitem__", "__delitem__", "__iadd__"):
        locals()[_n] = _wrap(_n)
    del _n, _wrap


_MOD = None


def _fp():
    global _MOD
    if _MOD is None:
        from . import _c28x
        _MOD = _c28x
    return _MOD


# ------------------------------------------------------------------------- PIE
class NativePie(f2803x_sys.Pie):
    def _st(self):
        return _fp().fp_state(self._core, "pie")

    ier = property(lambda self: self._st()["ier"])
    ifr = property(lambda self: self._st()["ifr"])
    ack = property(lambda self: self._st()["ack"])
    ctrl = property(lambda self: self._st()["ctrl"])
    counts = property(lambda self: self._st()["counts"])

    def reset(self):
        C28xPeripheral.reset(self)
        _fp().fp(self._core, OP_PIE_RESET)

    def raise_irq(self, group: int, idx: int) -> None:
        _fp().fp(self._core, OP_PIE_RAISE, group, idx)

    def clear_irq(self, group: int, idx: int) -> None:
        _fp().fp(self._core, OP_PIE_CLEAR, group, idx)

    def read(self, off):
        return _fp().fp(self._core, OP_PIE_READ, off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        _fp().fp(self._core, OP_PIE_WRITE, off, v)


# ------------------------------------------------------------------------ ePWM
_EP_FIELDS = ("prd", "prd_shadow", "cmpa", "cmpa_sh", "cmpb", "cmpb_sh", "prd_pend", "cmpa_pend", "cmpb_pend",
              "anchor_t", "anchor_ctr", "dir", "et_int_cnt", "et_soca_cnt", "et_socb_cnt", "etflg", "tzflg",
              "periods", "ints", "socas")


class NativeEpwm(f2803x_pwm.Epwm):
    def _st(self):
        return _fp().fp_state(self._core, "ep", self.index)

    for _f in _EP_FIELDS:
        locals()[_f] = property(lambda self, _f=_f: self._st()[_f])
    del _f
    tz_forced = property(lambda self: bool(self._st()["tz_forced"]))

    @property
    def regs(self):
        return dict(enumerate(self._st()["regs"]))

    @regs.setter
    def regs(self, v):
        pass

    def r(self, off):
        return self._st()["regs"][off] if off < 0x40 else 0

    def _flags(self):
        _fp().fp(self._core, OP_EP_FLAGS, self.index, int(bool(self.boundary_listeners)), int(bool(self.tz_listeners)))

    def reset(self):
        _fp().fp(self._core, OP_EP_RESET, self.index)

    def _catch_up(self):
        pass

    def mode(self):
        return self.r(0x00) & 3

    def counter(self):
        return _fp().fp(self._core, OP_EP_COUNTER, self.index)

    def direction(self):
        return _fp().fp(self._core, OP_EP_DIRECTION, self.index)

    def period_cycles(self):
        return _fp().fp(self._core, OP_EP_PERIOD, self.index)

    def duty(self, ch: str = "A") -> float:
        return _fp().fp(self._core, OP_EP_DUTY, self.index, 0 if ch == "A" else 1)

    def outputs_enabled(self) -> bool:
        return not _fp().fp(self._core, OP_EP_READ, self.index, 0x40)

    def trip(self, oneshot: bool = True) -> None:
        _fp().fp(self._core, OP_EP_TRIP, self.index, 1 if oneshot else 0)

    def read(self, off):
        return _fp().fp(self._core, OP_EP_READ, self.index, off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        _fp().fp(self._core, OP_EP_WRITE, self.index, off, v)

    def _on_boundary(self, kind: str) -> None:
        for fn in self.boundary_listeners:
            fn(self, kind)

    def _on_trip(self) -> None:
        for fn in self.tz_listeners:
            fn(self)


# ------------------------------------------------------------------------- ADC
class NativeAdc(f2803x_adc.Adc):
    def _st(self):
        return _fp().fp_state(self._core, "adc")

    result = property(lambda self: self._st()["result"])
    pending = property(lambda self: self._st()["pending"])
    intflg = property(lambda self: self._st()["intflg"])
    intovf = property(lambda self: self._st()["intovf"])
    rr = property(lambda self: self._st()["rr"])
    conversions = property(lambda self: self._st()["conversions"])
    busy_until = property(lambda self: self._st()["busy_until"])
    socctl = property(lambda self: self._st()["socctl"])

    @property
    def source(self):
        return self._source

    @source.setter
    def source(self, fn):
        self._source = fn
        self._callbacks()

    @property
    def temp_raw(self):
        return self._st()["temp_raw"]

    @temp_raw.setter
    def temp_raw(self, v):
        _fp().fp(self._core, OP_ADC_TEMP, int(v))

    def _sample(self, mask: int) -> List[int]:
        batch = getattr(self._source, "batch", None)
        if batch is not None:
            return batch(mask)
        src = self._source
        return [src(ch) if mask >> ch & 1 else 0 for ch in range(16)]

    def _listen(self, n: int) -> None:
        for fn in self.int_listeners:
            fn(n)

    def _callbacks(self):
        self._owner.callbacks()

    def reset(self):
        C28xPeripheral.reset(self)
        _fp().fp(self._core, OP_ADC_RESET)

    def trigger(self, src: int) -> None:
        _fp().fp(self._core, OP_ADC_TRIGGER, src)

    def _flag(self, bits: int) -> None:
        _fp().fp(self._core, OP_ADC_WRITE, 0x1A, bits)

    def read(self, off):
        return _fp().fp(self._core, OP_ADC_READ, off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        _fp().fp(self._core, OP_ADC_WRITE, off, v)


# ------------------------------------------------------------------------- CLA
class NativeCla(Cla):
    def _st(self):
        return _fp().fp_state(self._core, "cla")

    mifr = property(lambda self: self._st()["mifr"])
    miovf = property(lambda self: self._st()["miovf"])
    mier = property(lambda self: self._st()["mier"])
    mirun = property(lambda self: self._st()["mirun"])
    running = property(lambda self: self._st()["running"] or None)
    task_counts = property(lambda self: self._st()["task_counts"])
    task_cycles = property(lambda self: self._st()["task_cycles"])
    faults = property(lambda self: self._st()["faults"])

    def reset(self):
        C28xPeripheral.reset(self)
        self.cache.clear()
        self._compiled.clear()
        _fp().fp(self._core, OP_CLA_RESET)
        self._in_core = True

    def sync_from_core(self) -> None:
        mr, mar, fl, mrpc = _fp().cla_state(self._core)
        self.__dict__.update(mr=mr, mar=mar, mrpc=mrpc)
        (self.__dict__["nf"], self.__dict__["zf"], self.__dict__["tf"], self.__dict__["luf"], self.__dict__["lvf"],
         self.__dict__["rndf32"]) = fl

    def force(self, bits: int) -> None:
        _fp().fp(self._core, OP_CLA_FORCE, bits)

    def iack(self, value: int) -> None:
        _fp().fp(self._core, OP_CLA_IACK, value)

    def peripheral_irq(self, source: str, n: int) -> None:
        _fp().fp(self._core, OP_CLA_IRQ, {"adc": 1, "epwm": 0, "tint0": -1}[source], n)

    def _dispatch(self) -> None:
        pass

    def read(self, off):
        return _fp().fp(self._core, OP_CLA_READ, off)

    def peek(self, off):
        return self.read(off)

    def write(self, off, v):
        _fp().fp(self._core, OP_CLA_WRITE, off, v)


# --------------------------------------------------------------------- machine
class NativeC28xMachine(C28xMachine):
    """The machine with the native peripherals: interrupts are taken (and
    counted) in the core."""

    @property
    def exc_count(self):
        return _fp().fp_state(self.cpu._core, "counts")["exc_count"]

    @exc_count.setter
    def exc_count(self, v):
        pass

    @property
    def int_count(self):
        return _fp().fp_state(self.cpu._core, "counts")["int_count"]

    @int_count.setter
    def int_count(self, v):
        pass


class _Owner:
    def __init__(self, m):
        self.m = m

    def callbacks(self):
        m = self.m
        adc = m.adc
        extra = [fn for fn in adc.int_listeners if not getattr(fn, "cla_trigger", False)]
        _fp().fp_callbacks(m.cpu._core, adc._sample if adc._source is not None else None,
                           adc._listen if extra else None, self._boundary, self._trip)

    def _boundary(self, k, kind):
        self.m.epwm[k]._on_boundary(kind)

    def _trip(self, k):
        self.m.epwm[k]._on_trip()


def accelerate_peripherals(m: C28xMachine) -> bool:
    """Move the PIE, ePWM1-7, the ADC and the CLA dispatch of ``m`` into its
    native core (after :func:`~marsemu.c28x.cpu.accelerate`); False if the
    core has no native peripherals or ``MARSEMU_PY_PERIPHERALS`` is set."""
    core = getattr(m.cpu, "_core", None)
    if core is None or os.environ.get("MARSEMU_PY_PERIPHERALS"):
        return False
    try:
        fpm = _fp()
        fpm.fp_enable
    except (ImportError, AttributeError):
        return False
    routes = []
    pie, adc, cla = m.get("PIE"), m.get("ADC"), m.get("CLA1")
    routes.append((pie.base >> 6, FP_PIE))
    for pg in range(adc.base >> 6, (adc.base + adc.size + 63) >> 6):
        routes.append((pg, FP_ADC))
    for pg in range(cla.base >> 6, (cla.base + 0x80) >> 6):
        routes.append((pg, FP_CLA))
    for k in range(1, 8):
        routes.append((m.epwm[k].base >> 6, FP_EPWM + k - 1))
    fpm.fp_enable(core, routes)
    owner = _Owner(m)
    pie.__class__ = NativePie
    for n in ("ier", "ifr", "ack", "ctrl", "counts"):
        pie.__dict__.pop(n, None)
    for k in range(1, 8):
        e = m.epwm[k]
        if e._ev is not None:
            e._ev.cancel()
        e.__class__ = NativeEpwm
        for n in _EP_FIELDS + ("tz_forced", "regs"):
            e.__dict__.pop(n, None)
        e._core = core
        e.boundary_listeners = _NotifyList(e.boundary_listeners, e._flags)
        e.tz_listeners = _NotifyList(e.tz_listeners, e._flags)
    temp = adc.__dict__.get("temp_raw", 1700)
    src = adc.__dict__.pop("source", None)
    if adc._ev is not None:
        adc._ev.cancel()
    adc.__class__ = NativeAdc
    for n in ("result", "pending", "intflg", "intovf", "rr", "conversions", "busy_until", "socctl", "temp_raw"):
        adc.__dict__.pop(n, None)
    adc._core = core
    adc._owner = owner
    adc._source = src
    adc.int_listeners = _NotifyList(adc.int_listeners, owner.callbacks)
    cla.__class__ = NativeCla
    for n in ("mifr", "miovf", "mier", "mirun", "running", "task_counts", "task_cycles", "faults"):
        cla.__dict__.pop(n, None)
    cla._core = core
    pie._core = core
    adc.temp_raw = temp
    owner.callbacks()
    for k in range(1, 8):
        m.epwm[k]._flags()
    for p in m.peripherals.values():
        for off in getattr(p, "cacheable", ()):
            fpm.io_cacheable(core, p.base + off, True)
    m.__class__ = NativeC28xMachine
    m.__dict__.pop("exc_count", None)
    m.__dict__.pop("int_count", None)
    m.native_poll = lambda: fpm.fp(core, OP_POLL)
    m.native_next = lambda: fpm.fp(core, OP_NEXT)
    m.native_trace = lambda on: fpm.fp(core, OP_TRACE, 1 if on else 0)
    return True


__all__ = ["accelerate_peripherals", "NativePie", "NativeEpwm", "NativeAdc", "NativeCla"]
