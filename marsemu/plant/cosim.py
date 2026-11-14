"""Run several emulated MCUs and Python models on one timeline.

Every emulator is advanced in short lockstep slices (1 ms by default); Python
models schedule callbacks on the shared timeline.  Cross-MCU traffic (CAN)
reaches an emulator at its exact arrival time through that emulator's own
event clock, so the slice length only bounds how late a reply to a frame sent
within the same slice can be seen by the sender.
"""
from __future__ import annotations

import heapq
import itertools
import logging
import time as _time
from typing import Callable, List, Optional, Tuple

from .canbus import CanBus, Frame, Node

log = logging.getLogger(__name__)


class CoSim:
    #: --realtime: how far behind wall-clock time the simulation may fall
    #: before it stops trying to catch up
    REALTIME_SLACK_S = 0.05

    def __init__(self, slice_s: float = 0.001, realtime: bool = False):
        self.slice_s = slice_s
        self.realtime = realtime
        self.now = 0.0
        self.emulators: List = []
        #: MCUs in other processes (see remote.RemoteMcu); they run concurrently
        self.remotes: List = []
        self._events: List[Tuple[float, int, Callable[[], None]]] = []
        self._seq = itertools.count()
        self.tickers: List[Callable[[float, float], None]] = []
        self._wall0: Optional[float] = None

    # -- composition
    def add_emulator(self, emu) -> None:
        self.emulators.append(emu)

    def add_remote(self, node) -> None:
        self.remotes.append(node)

    def close(self) -> None:
        for r in self.remotes:
            r.close()

    def at(self, t: float, fn: Callable[[], None]) -> None:
        heapq.heappush(self._events, (t, next(self._seq), fn))

    def after(self, dt: float, fn: Callable[[], None]) -> None:
        self.at(self.now + dt, fn)

    def every(self, period: float, fn: Callable[[float], None]) -> None:
        """Call ``fn(t)`` every ``period`` seconds of simulated time."""
        def tick():
            fn(self.now)
            self.after(period, tick)
        self.after(period, tick)

    # -- running
    def run(self, seconds: float, until: Optional[Callable[[], bool]] = None) -> None:
        run_lockstep([self], seconds, until)

    # One slice, in three parts so that several co-simulations can share a
    # slice (see run_lockstep): the MCUs in child processes of every
    # co-simulation run while the local ones run.
    def _begin(self, target: float) -> None:
        while self._events and self._events[0][0] <= target:
            t, _, fn = heapq.heappop(self._events)
            self.now = max(self.now, t)
            fn()
        for r in self.remotes:
            r.start(target)

    def _run_local(self, target: float) -> None:
        for e in self.emulators:
            m = e.machine
            cyc = int(round(target * m.clock.hz)) - m.clock.cycles
            if cyc > 0:
                m.run(cycles=cyc)

    def _end(self, target: float) -> None:
        for r in self.remotes:
            r.finish()
        dt = target - self.now
        self.now = target
        for tk in self.tickers:
            tk(self.now, dt)


def run_lockstep(sims, seconds: float, until: Optional[Callable[[], bool]] = None,
                 slice_s: Optional[float] = None) -> None:
    """Advance several co-simulations (e.g. the ``sim`` of several devices in
    one house) by ``seconds`` on one shared timeline, slice by slice.

    Each slice runs every co-simulation's due events, then all of their MCUs
    (those in child processes concurrently with the local ones), then their
    tickers, so nothing in one device can run ahead of another by more than a
    slice.  The slice is the shortest of the co-simulations' ``slice_s``
    unless given; the co-simulations must be at the same time when called.
    ``--realtime`` pacing follows the first co-simulation's ``realtime``."""
    sims = list(sims)
    if not sims:
        return
    t0 = sims[0].now
    if any(abs(s.now - t0) > 1e-9 for s in sims):
        raise ValueError("co-simulations are at different times: "
                         + ", ".join(f"{s.now:.3f}" for s in sims))
    step = slice_s or min(s.slice_s for s in sims)
    lead = sims[0]
    end = t0 + seconds
    while lead.now < end - 1e-12:
        target = min(end, lead.now + step)
        for s in sims:
            s._begin(target)
        for s in sims:
            s._run_local(target)
        for s in sims:
            s._end(target)
        if lead.realtime:
            if lead._wall0 is None:
                lead._wall0 = _time.monotonic() - lead.now
            ahead = lead.now - (_time.monotonic() - lead._wall0)
            if ahead > 0.002:
                _time.sleep(ahead)
            elif ahead < -CoSim.REALTIME_SLACK_S:
                # Behind (a slow phase or a stall): carry on from here
                # instead of running flat out to catch up, which would
                # make anything outside (a real Wi-Fi module, a broker)
                # look slow to the firmware and trip its timeouts.
                lead._wall0 = _time.monotonic() - lead.now
        if until is not None and until():
            break


class EmulatorCanNode(Node):
    """Connects the CAN controller of an emulated MCU to a :class:`CanBus`."""

    def __init__(self, emu, periph: str, name: str = ""):
        self.emu = emu
        self.name = name or periph
        self.can = emu.machine.bus.get(periph)
        self.can.tx_listeners.append(self._tx)
        self.dropped = 0

    def _tx(self, ident: int, data: bytes, ext: bool) -> None:
        self.bus.send(self, ident, data, ext, t=self.emu.time)

    def deliver(self, f: Frame) -> None:
        m = self.emu.machine
        when = int(f.t * m.clock.hz)
        if when <= m.clock.cycles:
            self._inject(f)
        else:
            m.clock.at(when, lambda: self._inject(f), "can-rx")

    def _inject(self, f: Frame) -> None:
        ok = self.can.inject(f.ident, f.data, f.ext)
        if ok is False:
            self.dropped += 1


__all__ = ["CoSim", "run_lockstep", "EmulatorCanNode", "CanBus", "Frame", "Node"]
