"""A shared CAN bus between emulated MCUs and Python device models.

Frames are serialised like on the wire: one at a time, each occupying the bus
for its bit time (arbitration is first come, first served, which is what a
lightly loaded bus amounts to).  A frame becomes visible to every other node
once it has been transmitted completely.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, List, Optional

log = logging.getLogger(__name__)


@dataclass
class Frame:
    ident: int
    data: bytes
    ext: bool = True
    sender: Optional["Node"] = None
    t: float = 0.0          # time the frame was complete on the bus

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        idf = f"{self.ident:08x}" if self.ext else f"{self.ident:03x}"
        return f"<{self.t:.4f} {idf} {self.data.hex()} from {getattr(self.sender, 'name', '?')}>"


def frame_bits(dlc: int, ext: bool) -> int:
    """Approximate frame length on the wire, including typical bit stuffing."""
    raw = (67 if ext else 47) + 8 * dlc
    return raw + raw // 5 // 2


class Node:
    """Something attached to the bus."""

    name = "node"
    bus: "CanBus"

    def deliver(self, frame: Frame) -> None:
        raise NotImplementedError


class CanBus:
    def __init__(self, bitrate: int = 500_000):
        self.bitrate = bitrate
        self.nodes: List[Node] = []
        self.free_at = 0.0
        self.monitors: List[Callable[[Frame], None]] = []
        self.frames = 0
        self._now: Callable[[], float] = lambda: 0.0

    def attach(self, node: Node) -> Node:
        node.bus = self
        self.nodes.append(node)
        return node

    def send(self, sender: Node, ident: int, data: bytes, ext: bool = True, t: Optional[float] = None) -> Frame:
        """Queue a frame from ``sender`` at time ``t`` (default: now)."""
        t = self._now() if t is None else t
        start = max(t, self.free_at)
        done = start + frame_bits(len(data), ext) / self.bitrate
        self.free_at = done
        f = Frame(ident, bytes(data), ext, sender, done)
        self.frames += 1
        for m in self.monitors:
            m(f)
        for n in self.nodes:
            if n is not sender:
                n.deliver(f)
        return f
