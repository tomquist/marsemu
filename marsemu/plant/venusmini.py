"""Venus E Mini: the real control firmware with inverter, DC/DC and battery models.

The Mini (VNSEMINI-0) has three boards.  The control board ("PMU", N32G45x
running uC/OS-III, FC41D on USART2) runs the Wi-Fi/MQTT/BLE side, the meter
poll, the battery management (SOC, protections) and the operating modes; it
talks to the other two over UARTs, not CAN:

=========  ======  ======  ======  ===========================================
board      addr    UART    baud    here
=========  ======  ======  ======  ===========================================
control    0x80    -       -       real firmware (``venus-mini`` profile)
inverter   0x81    UART4   19200   :class:`MiniInverter` (Micro 270 is an
                                   STM32G474; it boots here, but at 1/40 of
                                   real time, so its protocol and regulator
                                   are modelled)
DC/DC      0x82    UART5   115200  :class:`MiniDcdc`, the battery side: cell
                                   voltages, current, temperatures (no
                                   firmware)
=========  ======  ======  ======  ===========================================

(Baud rates from the firmware's BRR with APB1 at 36 MHz, HCLK/4.)

The battery is two LFP cells in series (3.2 V, 314 Ah, 2009 Wh: the
firmware's rated capacity); the DC/DC boosts it to the inverter's DC bus.

Frames (both links)::

    aa | src | dst | length (2 bytes, big endian, whole frame) | cmd | payload | CRC-16/MODBUS (little endian)

Payload fields are big endian.  The control polls; a board answers with the
same command.  What is known, from the control firmware's request builders
and reply handlers (Control 309) and the inverter firmware (Micro 270):

Inverter (0x81)

* ``0x01`` status, 80 bytes: word 0 grid state (``gs``), words 2..13 six
  32-bit counters (``e1``..``e6``), word 16 firmware version (``inv``),
  word 38 a state the control watches, word 39 device state (``dev_sta``:
  0 standby, 1 charging, 2 discharging).
* ``0x02`` realtime, 82 bytes: word 0 grid frequency (0.01 Hz), word 1 grid
  voltage (0.1 V) - the control counts the grid as present above 40 Hz and
  80 V -, word 3 AC power (W, + = into the grid; ``gp``), words 10/11 the same
  as a 32-bit value (``ig``), word 14 backup-socket power (``lp``).
* ``0x40``..``0x46`` read and ``0x60``..``0x66`` write the inverter's
  parameter blocks (68, 52, 116, 94, 102, 60 bytes); a write is answered with
  the block as stored.  Block 0x40 word 31 is the feed-in limit (``gps``:
  0 = 800 W, 1 = 1500 W).
* ``0x21`` (20 bytes, every 0.6 s): word 0 the grid power the battery is to
  cover (W, + = import, the meter's reading on the battery's phase), word 9
  1 while the meter reading is valid.
* ``0x72`` (16 bytes): word 0 mode, word 1 power.  Mode 0: the inverter
  regulates the grid power of ``0x21`` itself (self-consumption); mode 1:
  charge at the power; mode 2: discharge at the power (phase detection,
  backup/UPS mode, the forced charge below 5 % SOC, schedules).
* ``0x70``/``0x71``: settings, answered with 4/10 bytes.

DC/DC (0x82)

* ``0x80`` realtime, 40 bytes: +0x02 pack voltage (u32, mV), +0x06/+0x0a pack
  current (i32, mA, + = charging; a coarse and a fine sensor, the control uses
  the fine one below 70 A), +0x0e/+0x10 the two cell voltages (mV),
  +0x12/+0x14 the two cell temperatures, +0x1a..+0x1e board temperatures
  (0.1 degC), +0x24 firmware version (``dcdc``).
* ``0x81``/``0x91`` read/write a 68-byte parameter block (current-sensor
  zero calibration among others).
* ``0x90`` (12 bytes): three switches and three values; ``1,1,1,0,0,0`` while
  the battery is fine.  Taken here as: word 0 converter on.

The inverter's self-consumption regulator follows Micro 270
(``0x080180fc``): discharging, its power grows by half the grid import per
reading and stops at 0 and at the feed-in limit; charging, the charge power
grows by half the export.  It turns to charging when it sits at its floor with
more than 15 W exported for three readings, and back when the charge power is
0 with more than 20 W imported for three readings.
"""
from __future__ import annotations

import logging
import struct
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from ..periph.n32g45x import UartDevice
from .battery import LfpPack
from .cosim import CoSim
from .venus3 import PowerStage

log = logging.getLogger(__name__)

SYNC = 0xAA
CONTROL, INVERTER, DCDC = 0x80, 0x81, 0x82
#: board -> the control's UART it hangs on
UART_OF = {INVERTER: "UART4", DCDC: "UART5"}
#: the board's answer follows the request after this long (ms)
REPLY_DELAY_MS = 3.0

#: reply payload lengths the control's handlers read
INV_STATUS_LEN, INV_REALTIME_LEN = 80, 82
INV_BLOCKS = {0x40: 68, 0x41: 52, 0x43: 116, 0x44: 94, 0x45: 102, 0x46: 60}
DCDC_REALTIME_LEN, DCDC_PARAMS_LEN = 40, 68

INVERTER_VERSION = 270      # Micro 270
DCDC_VERSION = 268          # the DC/DC version offered with Micro 270
#: the battery: two LFP cells of 314 Ah in series
CELLS, CAPACITY_AH = 2, 314.0
CELL_R_OHM = 0.00025


def crc16(data: bytes) -> int:
    """CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(src: int, dst: int, cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([SYNC, src, dst]) + struct.pack(">H", 8 + len(payload)) + bytes([cmd]) + payload
    return body + struct.pack("<H", crc16(body))


class FrameParser:
    """Splits a byte stream into checked frames ``(src, dst, cmd, payload)``."""

    MAX = 1100

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes):
        buf = self.buf
        buf.extend(data)
        while True:
            i = buf.find(bytes([SYNC]))
            if i < 0:
                buf.clear()
                return
            del buf[:i]
            if len(buf) < 5:
                return
            n = struct.unpack_from(">H", buf, 3)[0]
            if not 8 <= n <= self.MAX:
                del buf[0]
                continue
            if len(buf) < n:
                return
            f = bytes(buf[:n])
            if crc16(f[:-2]) != struct.unpack_from("<H", f, n - 2)[0]:
                self.bad += 1
                del buf[0]
                continue
            del buf[:n]
            yield f[1], f[2], f[5], f[6:-2]


#: The control's resting-voltage tables (Control 309, at 0x08030656): the mean
#: cell voltage (mV) at 100, 99, ..., 0 % SOC, at 0, 20 and 30 degC.  The
#: control picks or interpolates them by its hottest cell's temperature and
#: estimates its starting SOC from them; the Mini's cells take them as their
#: open-circuit voltage, so that estimate matches the pack's SOC.  Copied from
#: the firmware with the maintainer's permission.
MINI_OCV_TEMPS = (0.0, 20.0, 30.0)
MINI_OCV_MV = (
    (3380, 3334, 3333, 3332, 3331, 3330, 3330, 3329, 3328, 3327, 3326, 3325, 3324, 3323, 3322, 3322, 3321, 3320, 3319,
     3318, 3317, 3316, 3315, 3314, 3314, 3313, 3312, 3311, 3310, 3309, 3308, 3307, 3306, 3305, 3305, 3304, 3303, 3302,
     3301, 3300, 3299, 3298, 3297, 3297, 3296, 3295, 3294, 3293, 3292, 3291, 3290, 3289, 3289, 3288, 3287, 3286, 3285,
     3284, 3283, 3282, 3281, 3281, 3280, 3279, 3278, 3277, 3274, 3272, 3269, 3267, 3264, 3261, 3259, 3256, 3253, 3249,
     3246, 3243, 3239, 3235, 3231, 3227, 3223, 3219, 3215, 3210, 3205, 3200, 3195, 3190, 3184, 3178, 3172, 3166, 3160,
     3150, 3108, 3060, 2992, 2897, 2787),
    (3380, 3334, 3333, 3333, 3332, 3331, 3330, 3330, 3329, 3328, 3327, 3327, 3326, 3325, 3324, 3324, 3323, 3322, 3321,
     3321, 3320, 3319, 3318, 3318, 3317, 3316, 3315, 3315, 3314, 3313, 3312, 3312, 3311, 3310, 3309, 3309, 3308, 3307,
     3306, 3306, 3305, 3304, 3303, 3303, 3302, 3301, 3300, 3300, 3299, 3298, 3297, 3297, 3296, 3295, 3294, 3294, 3293,
     3292, 3291, 3291, 3290, 3289, 3288, 3288, 3287, 3286, 3285, 3283, 3280, 3278, 3275, 3272, 3269, 3266, 3262, 3259,
     3255, 3251, 3248, 3244, 3240, 3235, 3231, 3227, 3223, 3219, 3215, 3211, 3207, 3203, 3199, 3195, 3191, 3188, 3184,
     3178, 3139, 3095, 3032, 2948, 2838),
    (3360, 3333, 3332, 3332, 3331, 3330, 3330, 3329, 3328, 3328, 3327, 3326, 3325, 3325, 3324, 3323, 3323, 3322, 3321,
     3321, 3320, 3319, 3319, 3318, 3317, 3317, 3316, 3315, 3314, 3314, 3313, 3312, 3312, 3311, 3310, 3310, 3309, 3308,
     3308, 3307, 3306, 3305, 3305, 3304, 3303, 3303, 3302, 3301, 3301, 3300, 3299, 3299, 3298, 3297, 3297, 3296, 3295,
     3294, 3294, 3293, 3292, 3292, 3291, 3290, 3290, 3289, 3287, 3285, 3283, 3280, 3277, 3274, 3271, 3267, 3264, 3260,
     3256, 3252, 3248, 3244, 3240, 3236, 3232, 3228, 3228, 3219, 3215, 3211, 3207, 3203, 3200, 3196, 3192, 3189, 3186,
     3180, 3141, 3097, 3035, 2950, 2840),
)


def mini_ocv(soc: float, temp_c: float) -> float:
    """Open-circuit voltage (V) of a Mini cell at ``soc`` (0..1) and ``temp_c``,
    from :data:`MINI_OCV_MV`: linear in SOC between the 1 % points and in
    temperature between the tables (held at the 0 and 30 degC ones outside)."""
    t = min(MINI_OCV_TEMPS[-1], max(MINI_OCV_TEMPS[0], temp_c))
    k = 1 if t <= MINI_OCV_TEMPS[1] else 2
    t0, t1 = MINI_OCV_TEMPS[k - 1], MINI_OCV_TEMPS[k]
    w = (t - t0) / (t1 - t0)
    x = (1.0 - min(1.0, max(0.0, soc))) * 100.0        # table index: 0 = 100 %
    i = min(99, int(x))
    f = x - i

    def at(row):
        return row[i] + (row[i + 1] - row[i]) * f
    return ((1 - w) * at(MINI_OCV_MV[k - 1]) + w * at(MINI_OCV_MV[k])) / 1000.0


def mini_stage(ambient_c: float = 25.0) -> PowerStage:
    """Power-stage dynamics of the Mini (estimates: 800 W slew, 2 s grid
    connect, about 6 % conversion losses at full power through the DC/DC
    from 6.4 V)."""
    return PowerStage(slew_w_per_s=800.0, start_delay_s=2.0, loss_fixed_w=10.0, loss_quadratic=4e-5,
                      standby_w=4.0, ambient_c=ambient_c)


def ntc_raw(temp_c: float) -> int:
    """The cell temperature reading the control converts to ``temp_c``: its
    conversion (Control 309, ``0x0802272c``) is the identity up to 30.0 degC
    and 0.9 x raw + 30 above (0.1 degC).  Below 0 degC it is taken to stay the
    identity (the field is signed; only 0..409.5 was measured)."""
    t10 = temp_c * 10.0
    return int(round(t10 if t10 <= 300 else (t10 - 30) / 0.9))


class MiniBoard(UartDevice):
    """A board on one of the control's UARTs: answers the control's requests."""

    addr = 0
    name = "board"

    def __init__(self, emu):
        self.emu = emu
        self.parser = FrameParser()
        #: (time, cmd, payload hex) of the last requests
        self.log: List[tuple] = []
        self.requests: Dict[int, int] = {}
        u = emu.machine.bus.get(UART_OF[self.addr])
        u.device = self
        self.attach(u)
        emu.taps.pop(u.name, None)

    def on_tx(self, data: bytes) -> None:
        for src, dst, cmd, payload in self.parser.feed(data):
            if src != CONTROL or dst != self.addr:
                continue
            self.requests[cmd] = self.requests.get(cmd, 0) + 1
            self.log.append((round(self.emu.time, 3), cmd, payload.hex()))
            if len(self.log) > 500:
                del self.log[:250]
            reply = self.handle(cmd, payload)
            if reply is not None:
                f = frame(self.addr, CONTROL, cmd, reply)
                self.emu.machine.clock.after_ms(REPLY_DELAY_MS, lambda f=f: self.uart.inject(f), f"{self.name}-reply")

    def handle(self, cmd: int, payload: bytes) -> Optional[bytes]:
        raise NotImplementedError


@dataclass
class MiniLimits:
    """Inverter limits (W)."""

    #: charge power limit (grid charging)
    charge_w: float = 800.0
    #: feed-in limits by the ``gps`` preset (block 0x40 word 31)
    feed_in_w: tuple = (800.0, 1500.0)


class MiniInverter(MiniBoard):
    """The inverter board: its protocol, regulator and power stage."""

    addr = INVERTER
    name = "inverter"

    def __init__(self, emu, sim: CoSim, stage: Optional[PowerStage] = None, limits: Optional[MiniLimits] = None,
                 version: int = INVERTER_VERSION):
        super().__init__(emu)
        self.sim = sim
        self.stage = stage or mini_stage()
        self.limits = limits or MiniLimits()
        self.version = version
        self.blocks: Dict[int, bytearray] = {c: bytearray(n) for c, n in INV_BLOCKS.items()}
        self.mode = 0
        self.mode_power = 0
        self.grid_import = 0         # from 0x21 word 0
        self.meter_ok = False        # 0x21 word 9
        self.mode_seen = False
        # regulator (Micro 270): discharging state with a power that may dip to -20 W
        self.charging_state = False
        self.discharge_ref = -20.0
        self.charge_ref = 0.0
        self._switch = 0
        # power stage
        self.reference = 0.0         # + = into the grid
        self.ac_power = 0.0
        self.dc_power = 0.0          # + = into the battery
        self.connected = False
        self._connect_at: Optional[float] = None
        self.enabled: Callable[[], bool] = lambda: True
        self.temp_c = self.stage.ambient_c
        self.on_reference: List[Callable[[float, float], None]] = []

    # ------------------------------------------------------------- protocol
    def feed_in_limit(self) -> float:
        gps = struct.unpack_from(">H", self.blocks[0x40], 62)[0]
        return self.limits.feed_in_w[1 if gps == 1 else 0]

    def handle(self, cmd: int, p: bytes) -> Optional[bytes]:
        if cmd == 0x01:
            return self.status()
        if cmd == 0x02:
            return self.realtime()
        if cmd in INV_BLOCKS:
            return bytes(self.blocks[cmd])
        if cmd - 0x20 in INV_BLOCKS:
            blk = self.blocks[cmd - 0x20]
            blk[:] = p[:len(blk)].ljust(len(blk), b"\0")
            return bytes(blk)
        if cmd == 0x21:
            w = struct.unpack(">10h", p[:20].ljust(20, b"\0"))
            self.grid_import, self.meter_ok = w[0], bool(w[9])
            if not self.meter_ok:
                # no valid reading: Micro 270 drops to idle, in its charging state at 0 W
                self.charging_state, self.charge_ref, self.discharge_ref, self._switch = True, 0.0, -20.0, 0
            self._regulate()
            return b""
        if cmd == 0x72:
            self.mode, self.mode_power = struct.unpack(">HH", p[:4].ljust(4, b"\0"))
            self.mode_seen = True
            return p[:14].ljust(14, b"\0")
        if cmd == 0x70:
            return p[:4].ljust(4, b"\0")
        if cmd == 0x71:
            return p[:10].ljust(10, b"\0")
        return None                       # 0xa0..0xa3, 0xb0: firmware update, not modelled

    def _regulate(self) -> None:
        """One step of the self-consumption regulator per meter reading."""
        if self.mode != 0 or not self.meter_ok:
            return
        g = self.grid_import
        if self.charging_state:
            self.charge_ref = max(0.0, min(self.limits.charge_w, self.charge_ref - g / 2))
            self._switch = self._switch + 1 if (self.charge_ref == 0 and g > 20) else 0
            if self._switch >= 3:
                self.charging_state, self._switch, self.discharge_ref = False, 0, -20.0
        else:
            self.discharge_ref = max(-20.0, min(self.feed_in_limit(), self.discharge_ref + g / 2 - 2))
            self._switch = self._switch + 1 if (self.discharge_ref == -20 and g < -15) else 0
            if self._switch >= 3:
                self.charging_state, self._switch, self.charge_ref = True, 0, 0.0

    def target(self) -> float:
        if self.mode == 1:
            ref = -min(float(self.mode_power), self.limits.charge_w)
        elif self.mode == 2:
            ref = min(float(self.mode_power), self.feed_in_limit())
        elif self.meter_ok:
            ref = -self.charge_ref if self.charging_state else self.discharge_ref
        else:
            ref = 0.0
        return ref

    # ------------------------------------------------------------- dynamics
    def step(self, now: float, dt: float) -> None:
        st = self.stage
        target = self.target() if self.enabled() else 0.0
        if target != self.reference:
            for cb in self.on_reference:
                cb(self.target(), target)
        self.reference = target
        if abs(target) < st.deadband_w:
            target = 0.0
        if target != 0.0 and not self.connected:
            if self._connect_at is None:
                self._connect_at = now + st.start_delay_s
            if now >= self._connect_at:
                self.connected, self._connect_at = True, None
        elif target == 0.0:
            self._connect_at = None
        goal = target if self.connected else 0.0
        step = st.slew_w_per_s * dt
        self.ac_power += max(-step, min(step, goal - self.ac_power))
        losses = (st.loss_fixed_w + st.loss_quadratic * self.ac_power ** 2 + st.standby_w) if self.connected else 0.0
        self.dc_power = -self.ac_power - losses
        self.temp_c += dt * (st.ambient_c + losses * 0.05 - self.temp_c) / 300.0

    # ------------------------------------------------------------- replies
    def device_state(self) -> int:
        if self.ac_power > 5:
            return 2
        if self.ac_power < -5:
            return 1
        return 0

    def status(self) -> bytes:
        b = bytearray(INV_STATUS_LEN)
        struct.pack_into(">H", b, 0, 5)                # gs: 5 on real units
        struct.pack_into(">H", b, 32, self.version)
        struct.pack_into(">hh", b, 34, int(self.temp_c * 10), int(self.temp_c * 10))
        struct.pack_into(">H", b, 78, self.device_state())
        return bytes(b)

    def realtime(self) -> bytes:
        st = self.stage
        b = bytearray(INV_REALTIME_LEN)
        ac = int(round(self.ac_power))
        struct.pack_into(">HH", b, 0, int(st.grid_hz * 100), int(st.grid_v * 10))
        struct.pack_into(">h", b, 6, ac)
        struct.pack_into(">i", b, 20, ac)
        return bytes(b)


class MiniDcdc(MiniBoard):
    """The DC/DC board: measures the battery for the control's BMS."""

    addr = DCDC
    name = "dcdc"

    def __init__(self, emu, pack: LfpPack, version: int = DCDC_VERSION, ambient_c: float = 25.0):
        super().__init__(emu)
        self.pack = pack
        self.version = version
        self.params = bytearray(DCDC_PARAMS_LEN)
        self.switches = (0, 0, 0)
        self.values = (0, 0, 0)
        self.board_temp_c = ambient_c
        #: offset of the current sensors, mA (the control calibrates it out)
        self.current_offset_ma = 0

    @property
    def on(self) -> bool:
        return bool(self.switches[0])

    def handle(self, cmd: int, p: bytes) -> Optional[bytes]:
        if cmd == 0x80:
            return self.realtime()
        if cmd == 0x81:
            return bytes(self.params)
        if cmd == 0x91:
            self.params[:] = p[:DCDC_PARAMS_LEN].ljust(DCDC_PARAMS_LEN, b"\0")
            return bytes(self.params)
        if cmd == 0x90:
            w = struct.unpack(">6H", p[:12].ljust(12, b"\0"))
            self.switches, self.values = w[:3], w[3:]
            return b""
        return None

    def realtime(self) -> bytes:
        p = self.pack
        cells = [int(round(v * 1000)) for v in p.cell_voltages[:2]] + [0, 0]
        ma = int(round(p.current_a * 1000)) + self.current_offset_ma
        b = bytearray(DCDC_REALTIME_LEN)
        struct.pack_into(">I", b, 2, cells[0] + cells[1])
        struct.pack_into(">ii", b, 6, ma, ma)
        struct.pack_into(">HH", b, 14, cells[0], cells[1])
        raw = ntc_raw(p.temp_c)
        struct.pack_into(">hh", b, 18, raw, raw)    # read as signed
        t10 = int(round(self.board_temp_c * 10))
        struct.pack_into(">hhh", b, 26, t10, t10, t10)
        struct.pack_into(">H", b, 36, self.version)
        return bytes(b)


class VenusMini:
    """A Venus E Mini on one timeline: control firmware, inverter, DC/DC and
    battery models.  ``control_kwargs`` go to the control's
    :class:`~marsemu.emulator.Emulator` (network, state_dir, ...)."""

    CAPACITY_AH = CAPACITY_AH
    CELLS = CELLS
    #: the Mini answers two-digit commands only (``cd=01``, ``cd=02``)
    TWO_DIGIT_COMMANDS = True

    def __init__(self, control, *, pack: Optional[LfpPack] = None, stage: Optional[PowerStage] = None,
                 limits: Optional[MiniLimits] = None, control_kwargs: Optional[dict] = None,
                 slice_s: float = 0.005, realtime: bool = False, **_ignored):
        from ..emulator import Emulator
        self.sim = CoSim(slice_s=slice_s, realtime=realtime)
        self.pack = pack or LfpPack(cells_n=CELLS, capacity_ah=CAPACITY_AH)
        if all(c.r_ohm == 0.0008 for c in self.pack.cells) and self.pack.capacity_ah == CAPACITY_AH:
            # two large cells (unless given their own): lower resistance than
            # the 100 Ah default, about 12 kg
            for c in self.pack.cells:
                c.r_ohm, c.rc_r = CELL_R_OHM, c.rc_r * 100.0 / CAPACITY_AH
            self.pack.thermal_mass_j_per_k = 12_000.0
        if self.pack.capacity_ah == CAPACITY_AH and all(c.ocv_curve is None for c in self.pack.cells):
            # the Mini's cells rest at the voltages the control's SOC tables give
            for c in self.pack.cells:
                c.ocv_curve = lambda soc, pack=self.pack: mini_ocv(soc, pack.temp_c)
        kw = dict(control_kwargs or {})
        kw.setdefault("profile", "venus-mini")
        self.control = Emulator(control, **kw)
        self.sim.add_emulator(self.control)
        stage = stage or mini_stage(self.pack.ambient_c)
        self.micro = self.inverter = MiniInverter(self.control, self.sim, stage, limits)
        self.dcdc = MiniDcdc(self.control, self.pack, ambient_c=stage.ambient_c)
        self.inverter.enabled = self._battery_allows
        #: the control's own view, from its last ``cd=01`` report (SOC in %, battery W)
        self.reported: Dict[str, float] = {}
        self.control.on("mqtt_publish", self._note_report)
        self.sim.tickers.append(self._tick)
        self._acc = 0.0

    def _note_report(self, topic: str, payload) -> None:
        text = bytes(payload).decode("latin1", "replace")
        if not text.startswith("gp="):
            return
        f = dict(kv.split("=", 1) for kv in text.split(",") if "=" in kv)
        try:
            self.reported.update(soc=int(f["soc"]) / 10.0, battery_w=float(f["dpt"]), t=self.sim.now)
        except (KeyError, ValueError):
            pass

    def _battery_allows(self) -> bool:
        """The DC/DC carries power while the control has it on; the cells'
        hard limits stop it otherwise (the control's BMS acts long before)."""
        if not self.dcdc.on:
            return False
        t = self.inverter.target()
        if t > 0 and self.pack.min_cell_v() <= 2.5:
            return False
        if t < 0 and self.pack.max_cell_v() >= 3.65:
            return False
        return True

    def _tick(self, now: float, dt: float) -> None:
        self._acc += dt
        if self._acc >= 0.01:
            dt, self._acc = self._acc, 0.0
            self.inverter.step(now, dt)
            self.pack.step(dt, self.pack.current_for_power(self.inverter.dc_power))

    def run(self, seconds: float, until=None) -> None:
        self.sim.run(seconds, until)

    def close(self) -> None:
        self.sim.close()
        self.control.save_state()

    @property
    def time(self) -> float:
        return self.sim.now

    @property
    def ac_power(self) -> float:
        """AC power, W, + = into the house/grid (discharging)."""
        return self.inverter.ac_power

    @property
    def soc(self) -> float:
        return self.pack.soc

    def status(self) -> dict:
        inv = self.inverter
        soc = self.reported.get("soc")
        return {
            "t": round(self.sim.now, 3), "ac_w": round(inv.ac_power, 1), "ref_w": round(inv.reference, 1),
            "dc_w": round(inv.dc_power, 1), "soc": round(self.pack.soc * 100, 2), "bms_soc": soc,
            "pack_v": round(self.pack.voltage, 3), "i_a": round(self.pack.current_a, 2),
            "cells_v": [round(v, 3) for v in self.pack.cell_voltages],
            "limits": [round(inv.feed_in_limit()), -round(inv.limits.charge_w)], "connected": inv.connected,
            "mode": inv.mode, "mode_w": inv.mode_power, "grid_import_w": inv.grid_import, "meter_ok": inv.meter_ok,
            "dcdc_on": self.dcdc.on, "temp_c": round(self.pack.temp_c, 1),
        }


__all__ = ["VenusMini", "mini_stage", "mini_ocv", "MiniInverter", "MiniDcdc", "MiniLimits", "FrameParser", "frame", "crc16", "ntc_raw"]
