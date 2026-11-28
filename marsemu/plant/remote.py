"""Run an emulated MCU in a separate process, in lockstep with the co-simulation.

Each MCU is single threaded Python + Unicorn, so a board with several MCUs
only uses several CPU cores if they live in different processes.  A
:class:`RemoteMcu` runs one :class:`~marsemu.emulator.Emulator` in a child
process; per co-simulation slice the parent sends "run until t" with the CAN
frames that arrived for it and a snapshot of whatever the board model needs,
and collects the frames the MCU sent.  The child runs while the parent runs
the other MCUs, so slices overlap.

Frames from the remote MCU reach the others at their own timestamps when
those are still ahead, otherwise at the start of the next slice: at most one
slice (default 1 ms) of extra latency.
"""
from __future__ import annotations

import logging
import multiprocessing as mp
import signal
import traceback
from typing import Any, Callable, List, Optional

from .canbus import Frame, Node

log = logging.getLogger(__name__)


def _child(conn, build: Callable, args: tuple, can_periph: str, parent_end=None, keep_spacing: bool = False) -> None:
    # The fork inherits the parent's end of the pipe: closed here, so that the
    # parent dying (SIGKILL, a crash) ends recv() with EOFError instead of
    # leaving this process blocked in it for good.
    if parent_end is not None:
        parent_end.close()
    # Ctrl-C reaches the whole process group; the parent stops this MCU itself
    # ("stop"), after it has saved its state, rather than it dying mid-slice.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        emu, sync = build(*args)
        m = emu.machine
        can = m.bus.get(can_periph)
        out: List[tuple] = []
        can.tx_listeners.append(lambda i, d, x: out.append((emu.time, i, bytes(d), x)))
        conn.send(("ready", None))
        while True:
            try:
                msg = conn.recv()
            except EOFError:        # the parent is gone
                return
            if msg[0] == "stop":
                emu.save_state()
                conn.send(("stopped", None))
                return
            _, target, frames, state = msg
            if state is not None:
                sync.apply(state)
            # frames that went by while this MCU was waiting for the slice: at
            # once, or (keep_spacing) as late as the first of them but still as
            # far apart as on the bus, for controllers without a receive FIFO
            lag = 0
            if keep_spacing and frames:
                lag = max(0, m.clock.cycles - int(frames[0][0] * m.clock.hz))
            for (t, ident, data, ext) in frames:
                when = int(t * m.clock.hz) + lag
                if when <= m.clock.cycles:
                    can.inject(ident, data, ext)
                else:
                    m.clock.at(when, lambda i=ident, d=data, x=ext: can.inject(i, d, x), "can-rx")
            cyc = int(round(target * m.clock.hz)) - m.clock.cycles
            if cyc > 0:
                m.run(cycles=cyc)
            conn.send(("done", out[:], sync.report()))
            out.clear()
    except Exception:  # pragma: no cover - reported to the parent
        conn.send(("error", traceback.format_exc()))


class RemoteMcu(Node):
    """CAN node backed by an emulator in a child process.

    ``build(*args)`` runs in the child and returns ``(emulator, sync)``;
    ``sync.apply(state)`` takes what :meth:`state` produced in the parent and
    ``sync.report()`` returns what :attr:`report` should hold afterwards.

    ``keep_spacing``: frames that arrive late (during the slice before) reach
    the MCU as far apart as they were on the bus, not all at once; a CAN
    controller that has mailboxes and no receive FIFO (the C2000's eCAN)
    would otherwise overwrite one with the next."""

    def __init__(self, name: str, build: Callable, args: tuple = (), can_periph: str = "CAN1",
                 state: Optional[Callable[[], Any]] = None, keep_spacing: bool = False):
        self.name = name
        self.state = state
        self.report: Any = None
        self._pending: List[tuple] = []
        ctx = mp.get_context("fork")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_child, args=(child, build, args, can_periph, self.conn, keep_spacing),
                                daemon=True)
        self.proc.start()
        child.close()
        msg = self.conn.recv()
        if msg[0] != "ready":
            raise RuntimeError(f"{name} failed to start:\n{msg[1]}")
        self._busy = False

    def deliver(self, f: Frame) -> None:
        self._pending.append((f.t, f.ident, f.data, f.ext))

    def start(self, target: float) -> None:
        try:
            self.conn.send(("run", target, self._pending, self.state() if self.state else None))
        except (BrokenPipeError, ConnectionResetError):
            raise RuntimeError(f"{self.name} exited (exit code {self.proc.exitcode})") from None
        self._pending = []
        self._busy = True

    def finish(self) -> None:
        try:
            msg = self.conn.recv()
        except EOFError:
            self._busy = False
            raise RuntimeError(f"{self.name} exited (exit code {self.proc.exitcode})") from None
        self._busy = False
        if msg[0] == "error":
            raise RuntimeError(f"{self.name} crashed:\n{msg[1]}")
        _, frames, self.report = msg
        for (t, ident, data, ext) in frames:
            self.bus.send(self, ident, data, ext, t=t)

    def close(self) -> None:
        if self.proc.is_alive():
            try:
                if self._busy:
                    self.finish()
                self.conn.send(("stop",))
                self.conn.recv()
            except (EOFError, OSError):
                pass
            self.proc.join(2)
