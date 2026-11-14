"""Minimal GDB remote serial protocol server for the emulated MCU.

Connect with ``gdb-multiarch`` / ``arm-none-eabi-gdb``::

    (gdb) set architecture armv7e-m
    (gdb) target remote :3333

Supported: register and memory read/write, software breakpoints (Z0/Z1),
watchpoints (Z2/Z3/Z4), continue, single step, Ctrl-C, detach/kill and an
ARM M-profile target description.  While stopped, virtual time does not
advance; the device's peripherals (UART receive, timers) continue when the
CPU does.  Also usable from IDA/Ghidra/radare2 as a generic GDB target.
"""
from __future__ import annotations

import logging
import socket
import threading
from typing import Dict, Optional

from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE, UcError
from unicorn import arm_const as A

log = logging.getLogger("emu.gdb")

TARGET_XML = """<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32"/><reg name="r1" bitsize="32"/><reg name="r2" bitsize="32"/>
    <reg name="r3" bitsize="32"/><reg name="r4" bitsize="32"/><reg name="r5" bitsize="32"/>
    <reg name="r6" bitsize="32"/><reg name="r7" bitsize="32"/><reg name="r8" bitsize="32"/>
    <reg name="r9" bitsize="32"/><reg name="r10" bitsize="32"/><reg name="r11" bitsize="32"/>
    <reg name="r12" bitsize="32"/><reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32"/><reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" regnum="25"/>
  </feature>
  <feature name="org.gnu.gdb.arm.m-system">
    <reg name="msp" bitsize="32" regnum="26"/><reg name="psp" bitsize="32"/>
    <reg name="primask" bitsize="32"/><reg name="basepri" bitsize="32"/>
    <reg name="faultmask" bitsize="32"/><reg name="control" bitsize="32"/>
  </feature>
</target>"""

REGS = [A.UC_ARM_REG_R0 + i for i in range(13)] + [A.UC_ARM_REG_SP, A.UC_ARM_REG_LR, A.UC_ARM_REG_PC]
SYSREGS = [A.UC_ARM_REG_MSP, A.UC_ARM_REG_PSP, A.UC_ARM_REG_PRIMASK, A.UC_ARM_REG_BASEPRI,
           A.UC_ARM_REG_FAULTMASK, A.UC_ARM_REG_CONTROL]


def _h32(v: int) -> str:
    return (v & 0xFFFFFFFF).to_bytes(4, "little").hex()


class GdbServer:
    def __init__(self, emu, port: int, host: str = "127.0.0.1"):
        self.emu = emu
        self.m = emu.machine
        self.port = port
        self.host = host
        self.bps: Dict[int, int] = {}
        self.wps: Dict[tuple, int] = {}
        self.stop_signal = 5
        self.hit: Optional[str] = None
        self.conn: Optional[socket.socket] = None
        self.interrupt = threading.Event()

    # ------------------------------------------------------------ transport
    def _send(self, data: str) -> None:
        cs = sum(data.encode("latin1")) & 0xFF
        self.conn.sendall(f"${data}#{cs:02x}".encode("latin1"))

    def _recv_packet(self) -> Optional[str]:
        buf = b""
        while True:
            ch = self.conn.recv(1)
            if not ch:
                return None
            if ch == b"\x03":
                return "\x03"
            if ch == b"$":
                break
        while True:
            ch = self.conn.recv(1)
            if not ch:
                return None
            if ch == b"#":
                self.conn.recv(2)
                self.conn.sendall(b"+")
                return buf.decode("latin1")
            buf += ch

    # ------------------------------------------------------------- serving
    def serve(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        print(f"waiting for gdb on {self.host}:{self.port} (target remote :{self.port})", flush=True)
        self.conn, _ = srv.accept()
        log.warning("gdb connected")
        while True:
            pkt = self._recv_packet()
            if pkt is None:
                break
            reply = self.handle(pkt)
            if reply is False:
                break
            if reply is not None:
                self._send(reply)
        self._clear()
        try:
            self.conn.close()
        except OSError:
            pass

    def handle(self, p: str):
        m = self.m
        if p == "\x03":
            return f"S{self.stop_signal:02x}"
        if p.startswith("qSupported"):
            return "PacketSize=4000;qXfer:features:read+;swbreak+;hwbreak+"
        if p.startswith("qXfer:features:read:target.xml:"):
            off, ln = (int(x, 16) for x in p.split(":")[-1].split(","))
            chunk = TARGET_XML[off:off + ln]
            return ("l" if off + ln >= len(TARGET_XML) else "m") + chunk
        if p == "?":
            return f"S{self.stop_signal:02x}"
        if p in ("qAttached",):
            return "1"
        if p.startswith("qC"):
            return "QC1"
        if p in ("qfThreadInfo",):
            return "m1"
        if p in ("qsThreadInfo",):
            return "l"
        if p.startswith(("Hg", "Hc", "vMustReplyEmpty", "qTStatus", "qOffsets", "qSymbol")):
            return "OK" if p.startswith(("Hg", "Hc")) else ""
        if p == "g":
            vals = [m.rr(r) for r in REGS] + [m.rr(A.UC_ARM_REG_XPSR) | (1 << 24)]
            return "".join(_h32(v) for v in vals)
        if p.startswith("G"):
            data = bytes.fromhex(p[1:])
            for i, r in enumerate(REGS):
                if 4 * i + 4 <= len(data):
                    m.rw(r, int.from_bytes(data[4 * i:4 * i + 4], "little") | (1 if r == A.UC_ARM_REG_PC else 0))
            return "OK"
        if p.startswith("p"):
            n = int(p[1:], 16)
            if n < 16:
                return _h32(m.rr(REGS[n]))
            if n == 25:
                return _h32(m.rr(A.UC_ARM_REG_XPSR) | (1 << 24))
            if 26 <= n < 26 + len(SYSREGS):
                return _h32(m.rr(SYSREGS[n - 26]))
            return "00000000"
        if p.startswith("P"):
            n, v = p[1:].split("=")
            n = int(n, 16)
            v = int.from_bytes(bytes.fromhex(v), "little")
            if n < 16:
                m.rw(REGS[n], v | (1 if n == 15 else 0))
            elif 26 <= n < 26 + len(SYSREGS):
                m.rw(SYSREGS[n - 26], v)
            return "OK"
        if p.startswith("m"):
            addr, ln = (int(x, 16) for x in p[1:].split(","))
            try:
                return self._read_mem(addr, ln).hex()
            except UcError:
                return "E01"
        if p.startswith("M"):
            head, data = p[1:].split(":")
            addr, ln = (int(x, 16) for x in head.split(","))
            try:
                m.write(addr, bytes.fromhex(data))
                m.uc.ctl_remove_cache(addr, addr + ln)
            except UcError:
                return "E01"
            return "OK"
        if p.startswith(("Z0", "Z1")):
            addr = int(p.split(",")[1], 16) & ~1
            self._add_bp(addr)
            return "OK"
        if p.startswith(("z0", "z1")):
            addr = int(p.split(",")[1], 16) & ~1
            h = self.bps.pop(addr, None)
            if h is not None:
                m.uc.hook_del(h)
                m.uc.ctl_flush_tb()
            return "OK"
        if p[:2] in ("Z2", "Z3", "Z4"):
            kind, addr, ln = p[1], int(p.split(",")[1], 16), int(p.split(",")[2], 16)
            self._add_wp(kind, addr, ln)
            return "OK"
        if p[:2] in ("z2", "z3", "z4"):
            key = (p[1], int(p.split(",")[1], 16), int(p.split(",")[2], 16))
            h = self.wps.pop(key, None)
            if h is not None:
                m.uc.hook_del(h)
            return "OK"
        if p.startswith("c") or p.startswith("vCont;c"):
            return self._continue()
        if p.startswith("s") or p.startswith("vCont;s"):
            return self._step()
        if p.startswith("vCont?"):
            return "vCont;c;s"
        if p.startswith("qRcmd,"):
            cmd = bytes.fromhex(p[6:]).decode()
            out = self._monitor(cmd)
            return out.encode().hex() if out else "OK"
        if p == "k":
            m.stop("gdb kill")
            return False
        if p.startswith("D"):
            self._clear()
            self._send("OK")
            return False
        return ""

    def _clear(self) -> None:
        for h in list(self.bps.values()) + list(self.wps.values()):
            self.m.uc.hook_del(h)
        self.bps.clear()
        self.wps.clear()
        self.m.uc.ctl_flush_tb()

    def _read_mem(self, addr: int, ln: int) -> bytes:
        """Reads memory; peripheral space is read through the bus models."""
        out = bytearray()
        for a in range(addr, addr + ln):
            if 0x40000000 <= a < 0x60000000 or a >= 0xE0000000:
                out.append(self.m.bus.read(a, 1) & 0xFF)
            else:
                out += self.m.read(a, 1)
        return bytes(out)

    def _monitor(self, cmd: str) -> str:
        e = self.emu
        if cmd.strip() == "reset":
            self.m.request_reset("gdb")
            return "reset requested\n"
        if cmd.strip() == "time":
            return f"virtual time {e.time:.6f}s\n"
        if cmd.strip() == "at":
            return e.fc41d.dump_transcript(30) + "\n"
        return "monitor commands: reset, time, at\n"

    # --------------------------------------------------------- execution
    def _add_bp(self, addr: int) -> None:
        if addr in self.bps:
            return

        def hit(uc, a, size, _ud):
            if self._stepping_over == a:
                return
            self.hit = "T05swbreak:;"
            self.m.stop("breakpoint")
        self.bps[addr] = self.m.uc.hook_add(UC_HOOK_CODE, hit, begin=addr, end=addr)
        self.m.uc.ctl_flush_tb()

    _stepping_over = None

    def _add_wp(self, kind: str, addr: int, ln: int) -> None:
        typ = {"2": UC_HOOK_MEM_WRITE, "3": UC_HOOK_MEM_READ, "4": UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE}[kind]
        name = {"2": "watch", "3": "rwatch", "4": "awatch"}[kind]

        def hit(uc, access, a, size, value, _ud):
            self.hit = f"T05{name}:{a:x};"
            self.m.stop("watchpoint")
        self.wps[(kind, addr, ln)] = self.m.uc.hook_add(typ, hit, begin=addr, end=addr + ln - 1)

    def _watch_interrupt(self):
        """While the CPU runs, a Ctrl-C (0x03) from gdb stops it."""
        self.conn.settimeout(0.1)
        try:
            while self._running:
                try:
                    ch = self.conn.recv(1)
                except socket.timeout:
                    continue
                if ch == b"\x03" or not ch:
                    self.hit = "S02"
                    self.m.stop("gdb interrupt")
                    return
        finally:
            self.conn.settimeout(None)

    def _continue(self) -> str:
        self.hit = None
        pc = self.m.pc
        if pc in self.bps:
            self._step()
        self._running = True
        t = threading.Thread(target=self._watch_interrupt, daemon=True)
        t.start()
        try:
            while self.hit is None:
                reason = self.emu.run(None)
                if self.hit is None and reason not in ("breakpoint", "watchpoint", "gdb interrupt"):
                    self.hit = "S05"
                    break
        finally:
            self._running = False
            t.join()
        return self.hit

    def _step(self) -> str:
        m = self.m
        pc = m.pc
        if pc in m.unmask_table:  # stepping onto an "exit" instruction: emulate it
            m._emulate_unmask(pc)
            m.clock.advance(m.cpi)
            return "S05"
        self._stepping_over = pc
        try:
            m.uc.emu_start(pc | 1, 0, count=1)
        except UcError as e:
            log.warning("step: %s", e)
        finally:
            self._stepping_over = None
        m.clock.advance(m.cpi)
        return "S05"
