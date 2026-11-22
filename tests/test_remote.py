"""RemoteMcu's child process: it must never outlive the run that started it,
and it saves its state when the parent stops it, Ctrl-C or not."""
import os
import signal
import time
from types import SimpleNamespace

import pytest

from marsemu.plant.remote import RemoteMcu


def _fake_build(saved_path):
    """A stand-in MCU with the surface _child uses (no firmware needed)."""
    def save_state():
        open(saved_path, "w").write("saved")
    can = SimpleNamespace(tx_listeners=[], inject=lambda *a: None)
    clock = SimpleNamespace(hz=1_000_000, cycles=0, at=lambda *a: None)
    machine = SimpleNamespace(bus=SimpleNamespace(get=lambda name: can), clock=clock,
                              run=lambda cycles: setattr(clock, "cycles", clock.cycles + cycles))
    emu = SimpleNamespace(machine=machine, time=0.0, save_state=save_state)
    sync = SimpleNamespace(apply=lambda state: None, report=lambda: None)
    return emu, sync


def _wait_dead(proc, timeout=5.0):
    proc.join(timeout)
    return not proc.is_alive()


@pytest.mark.skipif(not hasattr(os, "fork"), reason="needs fork")
def test_child_exits_when_the_parent_end_of_the_pipe_closes(tmp_path):
    """The parent dying (SIGKILL, a crash) closes its end of the pipe; the
    child must see EOF then, which it can't while it holds a copy of that end
    itself (inherited through the fork)."""
    r = RemoteMcu("bms", _fake_build, (str(tmp_path / "state"),))
    r.start(0.001)
    r.finish()
    r.conn.close()
    assert _wait_dead(r.proc), "child kept running after the parent end closed"


@pytest.mark.skipif(not hasattr(os, "fork"), reason="needs fork")
def test_child_ignores_ctrl_c_and_saves_its_state_when_stopped(tmp_path):
    """Ctrl-C reaches the whole process group; the child must leave stopping
    to the parent, which then has it save its state."""
    saved = tmp_path / "state"
    r = RemoteMcu("bms", _fake_build, (str(saved),))
    os.kill(r.proc.pid, signal.SIGINT)
    time.sleep(0.2)
    assert r.proc.is_alive()
    r.start(0.001)
    r.finish()
    r.close()
    assert _wait_dead(r.proc)
    assert saved.read_text() == "saved"


@pytest.mark.skipif(not hasattr(os, "fork"), reason="needs fork")
def test_a_child_that_died_is_reported_not_waited_for(tmp_path):
    r = RemoteMcu("bms", _fake_build, (str(tmp_path / "state"),))
    r.proc.kill()
    assert _wait_dead(r.proc)
    with pytest.raises(RuntimeError, match="bms exited"):
        r.start(0.001)
        r.finish()
