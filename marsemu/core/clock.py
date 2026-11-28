"""Virtual time.

The emulated CPU executes one instruction per cycle.  Virtual time is the
number of executed cycles divided by the core clock, so it is deterministic
and independent of how fast the host is.  Peripherals schedule callbacks on
this timeline (timer overflows, UART byte times, SysTick, ...).
"""
from __future__ import annotations

import heapq
import itertools
from typing import Callable, Optional


class Event:
    __slots__ = ("when", "seq", "fn", "cancelled", "name")

    def __init__(self, when: int, seq: int, fn: Callable[[], None], name: str):
        self.when = when
        self.seq = seq
        self.fn = fn
        self.cancelled = False
        self.name = name

    def cancel(self) -> None:
        self.cancelled = True

    def __lt__(self, other: "Event") -> bool:
        return (self.when, self.seq) < (other.when, other.seq)


class Clock:
    def __init__(self, hz: int = 144_000_000):
        self.hz = hz
        self.cycles = 0
        self._q: list[Event] = []
        self._seq = itertools.count()
        #: while a CPU runs a slice: where the slice ends, and what to call when
        #: an event is scheduled before that (from inside the slice: a
        #: peripheral access, an interrupt), so that the CPU stops there
        self.horizon = 1 << 62
        self.on_earlier: Optional[Callable[[int], None]] = None

    # -- conversions -----------------------------------------------------
    @property
    def seconds(self) -> float:
        return self.cycles / self.hz

    @property
    def ms(self) -> float:
        return self.cycles * 1000.0 / self.hz

    def us_to_cycles(self, us: float) -> int:
        return max(1, int(us * self.hz / 1_000_000))

    def ms_to_cycles(self, ms: float) -> int:
        return max(1, int(ms * self.hz / 1000))

    # -- scheduling ------------------------------------------------------
    def at(self, when: int, fn: Callable[[], None], name: str = "") -> Event:
        ev = Event(max(when, self.cycles), next(self._seq), fn, name)
        heapq.heappush(self._q, ev)
        if self.on_earlier is not None and ev.when < self.horizon:
            self.horizon = ev.when
            self.on_earlier(ev.when)
        return ev

    def after(self, cycles: int, fn: Callable[[], None], name: str = "") -> Event:
        return self.at(self.cycles + max(1, int(cycles)), fn, name)

    def after_us(self, us: float, fn: Callable[[], None], name: str = "") -> Event:
        return self.after(self.us_to_cycles(us), fn, name)

    def after_ms(self, ms: float, fn: Callable[[], None], name: str = "") -> Event:
        return self.after(self.ms_to_cycles(ms), fn, name)

    def next_deadline(self) -> Optional[int]:
        while self._q and self._q[0].cancelled:
            heapq.heappop(self._q)
        return self._q[0].when if self._q else None

    def advance(self, cycles: int) -> None:
        self.cycles += cycles

    def run_due(self) -> int:
        """Fire every event whose deadline has passed. Returns number fired."""
        n = 0
        while self._q and self._q[0].when <= self.cycles:
            ev = heapq.heappop(self._q)
            if ev.cancelled:
                continue
            ev.fn()
            n += 1
        return n
