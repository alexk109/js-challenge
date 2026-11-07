"""Pin-level I2C reference models on the tick-based bus in verif/vip/core.py.

Three devices, all on two open-drain nets named "scl" and "sda": a device drives 0 or
releases (None), never 1. Because of the core's two-phase tick, every device here reacts
one tick after it sees an edge, exactly like a flop sampling its input, so bit times below
carry a fixed 1 to 3 tick latency that never changes which limit is met.

  I2cController      bit-level master for queued transactions; drives both lines, NACKs
                     and clock stretching handled, bit rate and every setup/hold time from
                     the bus tick rate and the I2C spec minimums.
  I2cEepromTarget    24C02-style slave: 7-bit address, 256 bytes, 8-byte pages, byte /
                     page write with in-page wrap, current-address, random and sequential
                     read, optional clock stretching, NACKs every other address.
  I2cTimingChecker   passive monitor: decodes START / repeated START / STOP, bytes and
                      ACKs, checks every spec minimum for a chosen mode and its fSCL
                      maximum, logging each violation with the tick it happened on.

Typical use:

    bus = Bus(ns_per_tick=20)                       # 50 MHz
    ctl = attach_controller(bus, address=0x50, mode="fast")
    ee = attach_eeprom(bus, address=0x50)
    chk = attach_checker(bus, mode="fast")          # optional, passive
    ctl.queue_write(b"\\xa5")
    ctl.run(bus, limit=200_000)
    assert not ctl.errors and chk.violations == []
    assert ee.memory[0] == 0xA5

Spec minimums (NXP UM10204 timing table, ns) and the nominal bit rate each mode must not
exceed. tLOW and tHIGH are floors: the controller lengthens them so that one bit is
exactly 1 / f, which is slower than the floor but never faster. Those floors alone do not
bound the rate from above -- a master can shorten both halves and still meet every one of
them above the mode's nominal rate -- so `mode_minimums` also returns "min_period", the
shortest SCL period the mode allows (125 ticks for fast mode at 20 ns/tick), and
`I2cTimingChecker` flags every SCL period shorter than it.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Optional

from .core import Bus, Device


@dataclass(frozen=True)
class I2cTimings:
    """One I2C speed class: its fSCL maximum in Hz and every minimum in nanoseconds.

    `freq` is the one timing that is a maximum rather than a floor (UM10204 `## Table 11`
    lists it as fSCL, 100 kHz for standard mode and 400 kHz for fast mode): the
    half-periods below are the minimums a device may shorten towards, the nominal rate is
    the one it must not exceed. `mode_minimums` turns it into the shortest legal SCL
    period, which is what `I2cTimingChecker` checks.
    """

    name: str
    freq: float
    t_low: float        # LOW period
    t_high: float       # HIGH period
    t_su_sta: float     # START setup: SDA stable high before the SCL rise
    t_hd_sta: float     # START hold: SDA stable low after the SCL fall
    t_su_dat: float     # data setup: SDA stable before the SCL rise
    t_hd_dat: float     # data hold: SDA stable after the SCL fall
    t_su_sto: float     # STOP setup: SCL rise to SDA release
    t_buf: float        # bus free time between a STOP and the next START
    t_ack: float = 0.0  # ACK hold; not checked by I2cTimingChecker, used by the target


I2C_MODES: dict[str, I2cTimings] = {
    "standard": I2cTimings("standard", 100e3, 4_700, 4_000, 4_700, 4_000, 250, 300, 4_700, 4_700, 300),
    "fast": I2cTimings("fast", 400e3, 1_300, 600, 600, 260, 100, 0, 1_300, 1_300, 0),
}


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


def mode_minimums(
    ns_per_tick: float, mode: str = "standard", overrides: Optional[dict] = None
) -> dict:
    """Every spec minimum for `mode` as whole ticks, rounded up, plus its fSCL maximum.

    `overrides` replaces individual nanosecond values by their long names, which is how a
    test drives a controller or a checker off-spec on purpose:
    {"t_low": 500} means "low half-periods must last at least 500 ns".

    The one override that is not a time is "freq", the fSCL maximum in Hz. It is how a
    test asks for a controller faster than its mode's nominal rate (450 kHz on a fast-mode
    bus still meets every minimum, which is why the nominal needs a check of its own), and
    how a checker can be pointed at a rate other than its mode's.

    "min_period" is the result the fSCL check uses: 1/fSCL rounded up to whole ticks. One
    tick of quantisation only ever shortens a legal period's budget by less than a tick,
    so the check is at worst a hair stricter than the exact 1/f and never lenient.
    """
    try:
        m = I2C_MODES[mode]
    except KeyError:
        raise ValueError(f"unknown I2C mode {mode!r}; pick one of {sorted(I2C_MODES)}") from None
    ns = {
        "t_low": m.t_low,
        "t_high": m.t_high,
        "t_su_sta": m.t_su_sta,
        "t_hd_sta": m.t_hd_sta,
        "t_su_dat": m.t_su_dat,
        "t_hd_dat": m.t_hd_dat,
        "t_su_sto": m.t_su_sto,
        "t_buf": m.t_buf,
        "t_ack": m.t_ack,
    }
    overrides = dict(overrides or {})
    freq = float(overrides.pop("freq", m.freq))
    if not freq > 0:
        raise ValueError(f"the fSCL maximum must be positive, got {freq!r}")
    for key, value in overrides.items():
        if key not in ns:
            raise ValueError(
                f"unknown timing override {key!r}; pick from {sorted(ns) + ['freq']}"
            )
        ns[key] = value
    out = {key: ns_to_ticks(ns_per_tick, value) for key, value in ns.items()}
    out["freq"] = freq
    out["min_period"] = ns_to_ticks(ns_per_tick, 1e9 / freq)
    out["mode"] = mode
    return out


@dataclass
class I2cOp:
    """One queued transaction. Filled in by the controller as it runs."""

    kind: str                      # "write" | "read" | "write_read"
    write: bytes = b""
    count: int = 0
    data: bytearray = field(default_factory=bytearray)   # bytes received
    acked: int = 0                 # bytes the slave ACKed in the write phase
    nack: bool = False             # a send was NACKed, or a read ended early
    error: str = ""
    start_tick: int = -1
    stop_tick: int = -1

    @property
    def ok(self) -> bool:
        return not self.nack and not self.error and len(self.data) >= (
            self.count if self.kind != "write" else 0
        )


class I2cController(Device):
    """Bit-level I2C master: queued transactions on the open-drain scl/sda nets.

    Every timing comes from the bus tick rate (bind it with `bus=`, see
    attach_controller) and the spec minimums for `mode`, so the same class gives a legal
    100 kHz or 400 kHz master. Each bit is one `low` phase then one `high` phase; SDA only
    ever changes inside a low phase, at the hold point for that byte's next bit, which is
    what makes the checker's tHD;DAT and tSU;DAT pass. Clock stretching is honoured: the
    high phase does not start counting until the bus really reads SCL high, and a slave
    that never releases SCL trips stretch_timeout instead of hanging the sim.
    """

    name = "i2c_ctrl"

    def __init__(
        self,
        address: int = 0x50,
        mode: str = "standard",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        scl_net: str = "scl",
        sda_net: str = "sda",
        stretch_timeout: int = 10_000,
        name: Optional[str] = None,
    ):
        if not 0 <= address <= 0x7F:
            raise ValueError(f"I2C address must be 7 bits, got {address:#x}")
        self.address = address
        self.mode = mode
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.scl_net = scl_net
        self.sda_net = sda_net
        self.stretch_timeout = stretch_timeout
        if name:
            self.name = name
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        """Turn the spec minimums into tick counts, then stretch the bit to the mode rate."""
        m = mode_minimums(self.ns_per_tick, self.mode, self.timing_overrides)
        self.min = m
        low, high = m["t_low"], m["t_high"]
        period = ns_to_ticks(self.ns_per_tick, 1e9 / m["freq"])
        slack = period - (low + high)
        if slack > 0:            # keep 1/f but spread the slack over both halves
            low += slack // 2
            high += slack - slack // 2
        self.low, self.high = low, high
        self.stop_high = max(high, m["t_su_sto"])
        # What the bus shows is the number of tick() calls a phase spans, minus the ticks
        # it spends waiting for a drive to appear in the next snapshot. The high phases
        # burn two (entry tick, stretched-SCL tick), the low phase none, so pre-compensate.
        self.low_ticks = low
        self.high_ticks = max(1, high - 2)
        self.stop_ticks = max(m["t_su_sto"], max(1, self.stop_high - 2))
        self.su_sta = m["t_su_sta"]
        self.first_hold = self._hold_point(m["t_hd_sta"])
        self.data_hold = self._hold_point(m["t_hd_dat"])
        # tSU;DAT: a low half-period has to fit the data hold time plus the data setup
        # time, so lengthen it if the minimums leave no room (odd timing overrides).
        need = self.data_hold + m["t_su_dat"] + 1
        if self.low_ticks < need:
            self.low_ticks = need
        if self.low < self.low_ticks:
            self.low = self.low_ticks

    def _hold_point(self, min_ticks: int) -> int:
        """Tick, counted from the SCL falling edge, at which SDA may change.

        One tick on top of the minimum, because a device reacts one tick after it sees
        the edge; clamped so an off-spec t_low can neither deadlock the state machine nor
        put the change while the bus still reads SCL high (which would look like a START).
        """
        return max(2, min(max(1, min_ticks) + 1, max(3, self.low - 1)))

    def bind(self, bus: Bus) -> "I2cController":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self._reset()
        return self

    def _reset(self) -> None:
        self.ops: list[I2cOp] = []          # queued, not started yet
        self.errors: list[str] = []
        self.nacks: list[tuple] = []        # (tick, what was being sent)
        self.transfers: list[tuple] = []    # (start_tick, stop_tick, kind, nbytes, nack)
        self.phase = "idle"
        self.op: Optional[I2cOp] = None
        self.plan: list[tuple] = []
        self.pi = 0
        self.send: list[int] = []
        self.si = 0
        self.cnt = 0
        self.after_low = "high"
        self.first_bit = False
        self.bit_sda: Optional[int] = 0
        self.clock_is_ack = False
        self.cur_byte: Optional[int] = None
        self.nbits = 0
        self.sampled = 1
        self.ack_seen = False
        self.ack_sampled = False
        self.wait_scl = False
        self.stretch_ticks = 0
        self.recving = False
        self.rx_shift = 0
        self.rx_left = 0
        self.clocks = 0
        self.free_left = self.su_sta   # nothing on the bus yet: still wait out tSU;STA

    # ---- the queue ---------------------------------------------------------------
    def queue_write(self, data) -> I2cOp:
        """START, address+W, data, STOP. Every byte must be ACKed."""
        op = I2cOp("write", write=bytes(data))
        self.ops.append(op)
        return op

    def queue_read(self, count: int) -> I2cOp:
        """START, address+R, `count` bytes (NACK on the last), STOP. Result in op.data."""
        op = I2cOp("read", count=count)
        self.ops.append(op)
        return op

    def queue_write_read(self, data, count: int) -> I2cOp:
        """START, address+W, data, repeated START, address+R, count bytes, STOP."""
        op = I2cOp("write_read", write=bytes(data), count=count)
        self.ops.append(op)
        return op

    @property
    def busy(self) -> bool:
        return self.phase != "idle" or bool(self.ops)

    @property
    def idle(self) -> bool:
        return not self.busy

    def run(self, bus: Bus, limit: int = 500_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- drives ------------------------------------------------------------------
    def _scl(self) -> Optional[int]:
        return 0 if self.phase in ("low", "rstart_rel") else None

    def _sda(self) -> Optional[int]:
        if self.phase in ("start_cond", "stop_prep"):
            return 0
        if self.phase in ("idle", "start_prep", "stop_cond", "rstart_rel"):
            return None
        return self.bit_sda          # "low" and "high": the bit being clocked

    def _drive(self) -> dict:
        return {self.scl_net: self._scl(), self.sda_net: self._sda()}

    def tick(self, t: int, pins) -> dict:
        drv = self._drive()
        self._advance(t, pins)
        return drv

    # ---- the state machine --------------------------------------------------------
    def _advance(self, t: int, pins) -> None:
        phase = self.phase
        if phase == "idle":
            self._idle(t)
        elif phase == "start_prep":
            self.cnt += 1
            if self.cnt >= self.su_sta:
                self.phase, self.cnt = "start_cond", 0
                self.bit_sda = 0      # SDA is low from here until the first hold point
        elif phase == "start_cond":
            self.cnt += 1
            if self.cnt >= 2:
                self._begin_low(t, first_bit=True)
        elif phase == "low":
            self._low()
        elif phase == "high":
            self._high(t, pins)
        elif phase == "stop_prep":
            self._high(t, pins, stop=True)
        elif phase == "stop_cond":
            # Both lines stay released from here: the bus goes free and stays high, so
            # there is no SCL low pulse after the STOP that could read as a short period.
            self.cnt += 1
            if self.cnt >= 2:
                self._to_idle(t)
        elif phase == "rstart_rel":
            self.cnt += 1
            if self.cnt >= self.su_sta:
                self.phase, self.cnt = "start_prep", 0
        else:
            raise RuntimeError(f"{self.name}: bad phase {self.phase!r}")

    def _idle(self, t: int) -> None:
        if self.free_left > 0:
            self.free_left -= 1
        elif self.ops:
            self._start_op(t)

    def _start_op(self, t: int) -> None:
        op = self.ops.pop(0)
        self.op = op
        op.start_tick = t
        w_addr, r_addr = self.address << 1, (self.address << 1) | 1
        if op.kind == "write":
            self.plan = [("send", [w_addr] + list(op.write)), ("stop",)]
        elif op.kind == "read":
            self.plan = [("send", [r_addr]), ("recv", op.count), ("stop",)]
        else:
            self.plan = [
                ("send", [w_addr] + list(op.write)),
                ("restart", [r_addr]),
                ("recv", op.count),
                ("stop",),
            ]
        self.pi, self.cnt, self.recving, self.rx_left, self.rx_shift = 0, 0, False, 0, 0
        self.send = list(self.plan[0][1])
        self.si = 1
        self.cur_byte, self.nbits = self.send[0], 0
        self.phase = "start_prep"

    def _begin_low(self, t: int, first_bit: bool = False, sda: Optional[int] = None,
                   after: str = "high") -> None:
        """Enter the low half-period before the next clock."""
        self.phase, self.cnt, self.after_low = "low", 0, after
        self.first_bit = first_bit
        self.clock_is_ack = self.nbits >= 8
        self.pending_sda = self._bit_value() if sda is None else sda

    def _bit_value(self) -> Optional[int]:
        """What SDA must read for the clock about to happen: 0 -> drive 0, 1 -> release."""
        if self.nbits >= 8:
            # The ACK clock. We are the receiver while reading, so we drive the ACK (0)
            # for every byte but the last, and release it to NACK the last one. While
            # sending, the slave drives instead and we stay off the bus. rx_left is only
            # decremented once this clock is over, so it still counts this byte here.
            if self.recving:
                return 0 if self.rx_left > 1 else None
            return None
        if self.recving:
            return None                  # the slave drives this bit
        return None if (self.cur_byte >> (7 - self.nbits)) & 1 else 0

    def _low(self) -> None:
        if self.cnt == (self.first_hold if self.first_bit else self.data_hold):
            self.bit_sda = self.pending_sda      # the only place SDA may change
        self.cnt += 1
        if self.cnt >= self.low_ticks:
            if self.after_low == "high":
                self.phase, self.cnt = "high", 0
                self.wait_scl, self.ack_sampled, self.stretch_ticks = True, False, 0
            else:
                self.phase, self.cnt = self.after_low, 0
                self.wait_scl, self.stretch_ticks = self.after_low == "stop_prep", 0

    def _high(self, t: int, pins, stop: bool = False) -> None:
        want = self.stop_ticks if stop else self.high_ticks
        if self.wait_scl:           # clock stretching: wait for the bus to read high
            if pins[self.scl_net] == 1:
                self.wait_scl = False
                self.cnt = 0
            else:
                self.stretch_ticks += 1
                if self.stretch_ticks > self.stretch_timeout:
                    self._error(f"SCL still low {self.stretch_ticks} ticks after release")
                    self.wait_scl = False
                    self.cnt = 0
            return
        self.cnt += 1
        if not self.ack_sampled and self.cnt >= max(1, want // 2):
            self.ack_sampled = True
            self.sampled = pins[self.sda_net]
        if self.cnt >= want:
            if stop:
                self.phase, self.cnt = "stop_cond", 0
            else:
                self.clocks += 1
                self._clock_done(t)

    def _clock_done(self, t: int) -> None:
        """One clock period finished: latch the bit, then pick what comes next."""
        if self.clock_is_ack:
            self.ack_seen = self.sampled == 0
            if self.recving:
                self._rx_byte(t)
                return
            if not self.ack_seen:
                self._nack(t, "byte %d of %d" % (self.si, len(self.send)))
                return
            if self.op:
                self.op.acked += 1
            if self.si < len(self.send):
                self._begin_byte(t)
            else:
                self._send_done(t)
            return
        if self.recving:
            self.rx_shift = (self.rx_shift << 1) | self.sampled
        self.nbits += 1
        self._begin_low(t)

    def _begin_byte(self, t: int) -> None:
        """The low half-period before the next byte's first clock."""
        self.cur_byte = self.send[self.si]
        self.si += 1
        self.nbits = 0
        self._begin_low(t)

    def _send_done(self, t: int) -> None:
        """Every byte of the current send or restart step came back ACKed."""
        self.pi += 1
        step = self.plan[self.pi]
        kind, arg = step[0], (step[1] if len(step) > 1 else None)
        if kind == "restart":                       # repeated START, then the next bytes
            self.send, self.si = list(arg), 0
            self.cur_byte, self.nbits = self.send[0], 0
            self.si = 1
            self._begin_low(t, sda=0, after="rstart_rel")
        elif kind == "recv" and arg > 0:            # the slave drives from here on
            self.recving, self.rx_shift, self.rx_left = True, 0, arg
            self.cur_byte, self.nbits = None, 0
            self._begin_low(t)
        elif kind == "recv":                        # a zero-length read: step straight over
            self._send_done(t)
        else:                                       # stop
            self._begin_low(t, sda=0, after="stop_prep")

    def _rx_byte(self, t: int) -> None:
        """A byte the master read has been clocked out and answered."""
        self.op.data.append(self.rx_shift)
        self.rx_shift = 0
        self.rx_left -= 1
        if self.rx_left <= 0:
            self._recv_done(t)                       # NACK on the last byte is normal
            return
        if not self.ack_seen:
            self._nack(t, "read ended after %d byte(s)" % (self.op.count - self.rx_left))
            return
        self.nbits = 0
        self._begin_low(t)

    def _recv_done(self, t: int) -> None:
        while True:
            self.pi += 1
            step = self.plan[self.pi]
            kind, arg = step[0], (step[1] if len(step) > 1 else None)
            if kind == "stop":
                self._begin_low(t, sda=0, after="stop_prep")
                return
            if kind == "recv" and arg > 0:
                self.rx_left = arg
                self.nbits = 0
                self._begin_low(t)
                return

    def _nack(self, t: int, what: str) -> None:
        """The slave did not ACK: record it and shut the transaction down with a STOP."""
        self.nacks.append((t, what))
        if self.op:
            self.op.nack = True
        self._begin_low(t, sda=0, after="stop_prep")

    def _error(self, msg: str) -> None:
        self.errors.append(f"{self.name}: {msg}")
        if self.op is not None:
            self.op.error = msg

    def _to_idle(self, t: int) -> None:
        """STOP is on the bus: SCL low, both lines free again."""
        if self.op is not None:
            self.op.stop_tick = t
            self.transfers.append(
                (self.op.start_tick, t, self.op.kind, len(self.op.data), self.op.nack)
            )
            self.op = None
        self.phase, self.cnt, self.recving, self.rx_left = "idle", 0, False, 0
        self.after_low, self.bit_sda, self.cur_byte, self.nbits = "high", None, None, 0
        self.free_left = self.min["t_buf"]


class I2cEepromTarget(Device):
    """24C02-style I2C EEPROM slave: 7-bit address, `size` bytes in `page_size`-byte pages.

    What it implements, all of it from the 24C02 datasheet: the control byte
    1010 A2 A1 A0 R/W with every other address NACKed, a byte write, a page write that
    rolls over inside the page when the counter passes its last address, a
    current-address read, a random read (control byte, repeated START, read control byte)
    and a sequential read that runs the counter on without page rollover. The word address
    byte is the full 8-bit memory address: a 2 Kbit part has no address above A7 and its
    pages are 8 bytes, so the rollover is A2:A0.

    It reacts one tick after it sees an edge, like any device on this bus: START and STOP
    come from SDA moving while SCL reads high, bits are sampled on the SCL rising edge,
    and SDA is driven from the falling edge that precedes its clock, never while SCL reads
    high. With `stretch_ticks` set it holds SCL low that many ticks after every byte it
    ACKs, which is where the controller's clock stretching gets exercised.
    """

    name = "i2c_eeprom"

    def __init__(
        self,
        address: int = 0x50,
        size: int = 256,
        page_size: int = 8,
        mode: str = "standard",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        stretch_ticks: int = 0,
        timing: Optional[dict] = None,
        scl_net: str = "scl",
        sda_net: str = "sda",
        name: Optional[str] = None,
    ):
        if not 0 <= address <= 0x7F:
            raise ValueError(f"I2C address must be 7 bits, got {address:#x}")
        self.address, self.size, self.page_size = address, size, page_size
        self.mode = mode
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.stretch_ticks = stretch_ticks
        self.scl_net, self.sda_net = scl_net, sda_net
        if name:
            self.name = name
        self.min = mode_minimums(self.ns_per_tick, self.mode, self.timing_overrides)
        self.data_hold = max(1, self.min["t_hd_dat"]) + 1   # SDA hold after SCL falls
        self.ack_hold = max(1, self.min["t_ack"]) + 1      # hold time of our own ACK
        self.memory = bytearray(size)
        self.write_log: list[tuple] = []            # (tick, address, value)
        self.read_log: list[tuple] = []             # (tick, address)
        self.nacked_addresses: list[tuple] = []     # (tick, byte)
        self._reset()

    def bind(self, bus: Bus) -> "I2cEepromTarget":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = mode_minimums(self.ns_per_tick, self.mode, self.timing_overrides)
        self.data_hold = max(1, self.min["t_hd_dat"]) + 1
        self.ack_hold = max(1, self.min["t_ack"]) + 1
        return self

    def _reset(self) -> None:
        self.state = "idle"      # idle | addr | word | data | read | ignore
        self.prev_scl = self.prev_sda = 1
        self.nbits = 0           # bits of the current byte already seen (9: ACK done)
        self.shift = 0
        self.ack = False         # what to drive for the ACK clock that comes next
        self.counter = 0         # where the next byte read or written lands
        self.rw = 0
        self.tx_byte = 0
        self.pending_tx = False
        self.tx_done = False
        self.sda_out: Optional[int] = None
        self.sda_pending: Optional[tuple] = None
        self.stretch_left = 0
        self.bytes_seen = 0
        self.starts = self.stops = 0

    # ---- drives ------------------------------------------------------------------
    def _drive(self, t: int) -> dict:
        if self.sda_pending is not None and t >= self.sda_pending[0]:
            self.sda_out = self.sda_pending[1]
            self.sda_pending = None
        drv = {self.sda_net: self.sda_out}
        if self.stretch_left > 0:
            drv[self.scl_net] = 0        # clock stretching
        return drv

    def tick(self, t: int, pins) -> dict:
        drv = self._drive(t)
        self._advance(t, pins)
        return drv

    def _schedule(self, t: int, value: Optional[int], hold: Optional[int] = None) -> None:
        """Drive `value` on SDA from `hold` ticks after now (default: the data hold time),
        so SDA never moves while the bus reads SCL high."""
        if value == self.sda_out and self.sda_pending is None:
            return
        self.sda_pending = (t + (self.data_hold if hold is None else hold), value)

    # ---- edges --------------------------------------------------------------------
    def _advance(self, t: int, pins) -> None:
        if self.stretch_left > 0:
            self.stretch_left -= 1
        scl, sda = pins[self.scl_net], pins[self.sda_net]
        prev_scl, prev_sda = self.prev_scl, self.prev_sda
        self.prev_scl, self.prev_sda = scl, sda
        if scl == 1 and prev_sda != sda:
            self._start(t) if sda == 0 else self._stop(t)
            return
        if self.state == "idle":
            return
        if scl == 1 and prev_scl == 0:
            self._rising(t, sda)
        elif scl == 0 and prev_scl == 1:
            self._falling(t)

    def _start(self, t: int) -> None:
        """SDA fell while SCL was high: a START, or a repeated START from a read."""
        self.state = "addr"
        self.nbits = self.shift = 0
        self.ack = False
        self.pending_tx = False
        self.tx_done = False
        self.sda_out = self.sda_pending = None
        self.starts += 1

    def _stop(self, t: int) -> None:
        """SDA rose while SCL was high: the transaction is over, the address pointer stays
        where it is, which is what makes a current-address read work."""
        self.state = "idle"
        self.nbits = 0
        self.sda_out = self.sda_pending = None
        self.stops += 1

    def _rising(self, t: int, sda: int) -> None:
        if self.nbits < 8:                       # data bits, MSB first
            self.shift = (self.shift << 1) | sda
            self.nbits += 1
            if self.nbits == 8 and self.state != "read":
                self._decode(t)           # sending: nothing to decode
            return
        if self.nbits != 8:
            return
        acked = sda == 0
        self.nbits = 9                           # the ACK clock is over
        if self.state == "read":
            # The read pointer runs on after every byte; the master's NACK only says
            # this was the last one. No page rollover on a read, unlike a write.
            self.counter = (self.counter + 1) % self.size
            if acked:
                self.tx_byte = self.memory[self.counter]
            else:
                self.tx_done = True              # let go of SDA: the master owns the bus
                self.sda_out = self.sda_pending = None
            return
        if acked and self.pending_tx:
            self.state = "read"
            self.pending_tx = False
            self.read_log.append((t, self.counter))

    def _decode(self, t: int) -> None:
        """Eight bits in: decide the ACK and what they mean."""
        byte = self.shift
        self.bytes_seen += 1
        if self.state == "addr":
            if (byte >> 1) != self.address:
                self.ack = False
                self.state = "ignore"
                self.nacked_addresses.append((t, byte))
                return
            self.ack, self.rw = True, byte & 1
            if self.rw:                          # read: the data byte follows the ACK
                self.tx_byte = self.memory[self.counter]
                self.pending_tx = True
            else:
                self.state = "word"              # write: the word address byte follows
            return
        if self.state == "ignore":
            self.ack = False
            return
        if self.state == "word":
            self.counter = byte % self.size
            self.ack = True
            self.state = "data"
            return
        # write data byte: store it, then roll the pointer over inside the page
        self.memory[self.counter] = byte
        self.write_log.append((t, self.counter, byte))
        self.counter = self._roll(self.counter)
        self.ack = True

    def _roll(self, addr: int) -> int:
        """Next address after a write: +1, but from the last address of a page straight
        back to the first address of the same page."""
        if addr % self.page_size == self.page_size - 1:
            return addr - (self.page_size - 1)
        return (addr + 1) % self.size

    def _falling(self, t: int) -> None:
        """SCL is about to go high: put the right value on SDA for the next clock."""
        if self.nbits == 8:                      # the ACK clock comes next
            if self.ack:
                self.stretch_left = self.stretch_ticks
            if self.state == "read":             # our ACK clock: the master drives
                self._schedule(t, None)
            else:
                self._schedule(t, 0 if self.ack else None, hold=self.ack_hold)
            return
        if self.nbits >= 8:                      # the ACK clock just ended
            self.nbits = 0                       # count the next byte from bit 0
            self.shift = 0
        if self.state != "read" or self.tx_done:
            self._schedule(t, None)              # receiving, or done: stay off the bus
            return
        bit = 0 if self.nbits >= 8 else self.nbits    # next data bit of the byte we send
        value = (self.tx_byte >> (7 - bit)) & 1
        self._schedule(t, None if value else 0)


# ---- harness helpers -------------------------------------------------------------
def _ensure_nets(bus: Bus) -> None:
    """Add the two open-drain I2C nets if they are not there yet."""
    for net in ("scl", "sda"):
        if net not in bus.nets:
            bus.add_net(net)


def attach_controller(bus: Bus, **kwargs) -> I2cController:
    """Build an I2cController that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    ctl = I2cController(bus=bus, **kwargs)
    bus.attach(ctl)
    return ctl


def attach_eeprom(bus: Bus, **kwargs) -> I2cEepromTarget:
    """Build an I2cEepromTarget that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    ee = I2cEepromTarget(bus=bus, **kwargs)
    bus.attach(ee)
    return ee


def attach_checker(bus: Bus, **kwargs) -> "I2cTimingChecker":
    """Build a passive I2cTimingChecker for `bus` and attach it."""
    _ensure_nets(bus)
    chk = I2cTimingChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk


@dataclass(frozen=True)
class Violation:
    """One spec minimum that the bus did not meet, with the tick it was missed on."""

    tick: int
    rule: str            # e.g. "tLOW", "tHD;STA", "tSU;DAT"
    measured: int        # ticks the bus gave it
    required: int        # ticks the mode asks for
    detail: str = ""

    def __str__(self) -> str:
        return (f"t={self.tick} {self.rule} {self.measured} ticks < {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


@dataclass
class ByteTransfer:
    """One byte the checker saw on the bus, with the tick its ACK clock began."""

    tick: int
    value: int
    ack: bool
    is_address: bool = False
    rw: Optional[int] = None      # address byte only: 0 = write, 1 = read

    def __str__(self) -> str:
        what = f"0x{self.value:02x}"
        if self.is_address:
            what += f" (address, {'read' if self.rw else 'write'})"
        return f"t={self.tick} {what} {'ACK' if self.ack else 'NACK'}"


class I2cTimingChecker(Device):
    """Passive I2C bus monitor: decodes the traffic and checks the spec, both ways.

    It drives nothing, so it sees the bus exactly as a real analyser would, including
    the two-phase tick's sampling latency: it measures the resolved pin values and does
    not add or subtract anything for a device reacting a tick late. It measures you, not
    the devices.

    Checks, all against `mode`: tLOW, tHIGH, tSU;STA and tHD;STA (both the first START
    and a repeated one), tSU;DAT, tHD;DAT, tSU;STO and tBUF, plus fSCL, the mode's
    maximum SCL frequency. Every miss is appended to `violations` with its tick. It also
    decodes START, repeated START, STOP, every byte and every ACK/NACK into `starts`,
    `repeated_starts`, `stops`, `bytes`, `acks`, `nacks` and `transfers`.

    fSCL is the only maximum, so it is checked differently from the rest: a period is
    *too short* rather than too short by a floor. `mode_minimums` turns fSCL into
    "min_period" (1/fSCL rounded up to whole ticks) and every SCL period shorter than
    that is a violation. The periods are the ones between consecutive SCL rising edges,
    and the START and STOP pulses are left out of it: a master holds SCL high for only
    tHD;STA + tSU;STA around a START, which is not a clock cycle, and counting it would
    flag every legal master. Since the difference only shows up once SDA has moved during
    the high period *after* the rise, a rise is a clock edge only once the next rise has
    come and no START or STOP has appeared in between -- so the first bit time of a
    transaction, and the last one before every START and STOP, are not measured.
    `shortest_period` and `periods` say what was measured.

    One caveat: a period that began before the first tick cannot be measured, so the
    first SCL high period and the first START are measured from tick 0. That only makes
    the checker stricter, never lenient.
    """

    name = "i2c_check"

    def __init__(
        self,
        mode: str = "standard",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        scl_net: str = "scl",
        sda_net: str = "sda",
        name: Optional[str] = None,
    ):
        self.mode = mode
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.min = mode_minimums(self.ns_per_tick, self.mode, self.timing_overrides)
        self.scl_net, self.sda_net = scl_net, sda_net
        if name:
            self.name = name
        self.violations: list[Violation] = []
        self.transfers: list[ByteTransfer] = []
        self.start_ticks: list[int] = []
        self.stop_ticks: list[int] = []
        self._reset()

    def bind(self, bus: Bus) -> "I2cTimingChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = mode_minimums(self.ns_per_tick, self.mode, self.timing_overrides)
        return self

    def _reset(self) -> None:
        self.prev_scl = self.prev_sda = 1
        self.low_since = None      # tick the current SCL low period began
        self.high_since = 0        # tick the current SCL high period began
        self.sda_high_since = 0    # last SDA rise: bounds tSU;STA
        self.sda_low_since = 0     # last SDA fall: bounds tSU;STO
        self.dat_since = None      # last SDA move while SCL low: tSU;DAT
        self.hold_since = 0        # last SCL fall: bounds tHD;DAT / tHD;STA
        self.hold_req = 0
        self.hold_rule = "tHD;DAT"
        self.start_hold = False    # the high period that just ended was a START
        self.last_stop = None
        self.last_condition = None
        self.rise_pending = None    # last SCL rise, not yet known to be a clock edge
        self.rise_confirmed = None  # the last rise known to be one
        self.shortest_period = None # shortest SCL period measured, in ticks
        self.periods = 0            # SCL periods measured, the fSCL sample count
        self.nbits = 0
        self.shift = 0
        self.first_byte = True
        self.starts = 0
        self.repeated_starts = 0
        self.stops = 0
        self.bytes = 0
        self.acks = 0
        self.nacks = 0

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def max_scl_hz(self) -> Optional[float]:
        """The fastest SCL the bus actually ran at, from the shortest period measured."""
        if self.shortest_period is None:
            return None
        return 1e9 / (self.ns_per_tick * self.shortest_period)

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 10) -> str:
        head = (f"{self.mode} mode at {self.ns_per_tick} ns/tick: "
                f"{self.starts} START ({self.repeated_starts} repeated), {self.stops} STOP, "
                f"{self.bytes} bytes, {self.acks} ACK, {self.nacks} NACK, "
                f"{len(self.violations)} violations")
        if self.shortest_period is not None:
            head += (f", shortest SCL period {self.shortest_period} ticks "
                     f"({self.max_scl_hz / 1e3:.1f} kbit/s, fSCL max "
                     f"{self.min['freq'] / 1e3:.1f} kbit/s)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   detail: str = "") -> None:
        if measured < required:
            self.violations.append(Violation(t, rule, measured, required, detail))

    # ---- the monitor --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        scl, sda = pins[self.scl_net], pins[self.sda_net]
        prev_scl, prev_sda = self.prev_scl, self.prev_sda
        self.prev_scl, self.prev_sda = scl, sda
        # SCL first: a falling edge starts the data hold window, so an SDA move in the
        # same tick is a zero-length hold and must be caught as one.
        if prev_scl != scl:
            if scl == 0:
                self._fall(t)
            else:
                self._rise(t, sda)
        if prev_sda != sda:
            # SDA moves while SCL reads high: START or STOP. Anything else is data.
            if scl == 1:
                self._start(t) if sda == 0 else self._stop(t)
            else:
                self._data(t, sda)
        return {}

    def _high_start(self, t: int) -> int:
        """Tick the current SCL high period began, or t if SCL rose on this very tick."""
        return self.high_since if self.high_since is not None else t

    def _start(self, t: int) -> None:
        self.starts += 1
        if self.last_condition == "start":
            self.repeated_starts += 1
        self.last_condition = "start"
        self.start_ticks.append(t)
        self.start_hold = True
        self.nbits = self.shift = 0
        self.first_byte = True
        self.sda_low_since = t
        # SDA moved during the high period of the rise before this: that rise is the SCL
        # pulse of the START (tHD;STA + tSU;STA long), not a clock edge, so it is not one
        # end of a bit time either.
        self.rise_pending = None
        # tSU;STA: SDA has to be stable high for tSU;STA before the SCL rise that goes
        # with this START, so the window starts at whichever came later.
        self._violation(t, "tSU;STA", t - max(self._high_start(t), self.sda_high_since),
                        self.min["t_su_sta"], "SDA high before the START's SCL rise")
        # tBUF: only between a STOP and the next START, not after a repeated START.
        if self.last_stop is not None:
            self._violation(t, "tBUF", t - self.last_stop, self.min["t_buf"],
                            "bus free time after the previous STOP")
            self.last_stop = None

    def _stop(self, t: int) -> None:
        self.stops += 1
        self.last_condition = "stop"
        self.last_stop = t
        self.stop_ticks.append(t)
        self.nbits = self.shift = 0
        self.first_byte = False
        self.sda_high_since = t
        # The SCL pulse that carries this STOP is the rise before it, not a clock edge.
        self.rise_pending = None
        # tSU;STO: SDA has to be stable low for tSU;STO after the SCL rise that goes
        # with this STOP.
        self._violation(t, "tSU;STO", t - max(self._high_start(t), self.sda_low_since),
                        self.min["t_su_sto"], "SDA low before the STOP's SDA release")

    def _data(self, t: int, sda: int) -> None:
        """SDA moved while SCL was low: a data bit."""
        if t < self.hold_since + self.hold_req:
            self._violation(t, self.hold_rule, t - self.hold_since, self.hold_req,
                            "SDA moved after the SCL fall")
        self.dat_since = t
        if sda == 1:
            self.sda_high_since = t
        else:
            self.sda_low_since = t

    def _fall(self, t: int) -> None:
        if self.high_since is not None:
            self._violation(t, "tHIGH", t - self.high_since, self.min["t_high"],
                            "SCL high period")
        self.high_since = None
        self.low_since = t
        # After SCL falls, SDA is held for tHD;STA if a START just ended, else tHD;DAT.
        if self.start_hold:
            self.hold_rule, self.hold_req = "tHD;STA", self.min["t_hd_sta"]
            self.start_hold = False
        else:
            self.hold_rule, self.hold_req = "tHD;DAT", self.min["t_hd_dat"]
        self.hold_since = t
        if self.nbits == 9:
            self.nbits = 0
            self.shift = 0

    def _rise(self, t: int, sda: int) -> None:
        if self.low_since is not None:
            self._violation(t, "tLOW", t - self.low_since, self.min["t_low"],
                            "SCL low period")
        self.low_since = None
        self.high_since = t
        self._period(t)
        if self.dat_since is not None:      # tSU;DAT: SDA stable before SCL goes high
            self._violation(t, "tSU;DAT", t - self.dat_since, self.min["t_su_dat"],
                            "SDA changed at t=%d" % self.dat_since)
            self.dat_since = None
        if self.nbits < 8:                 # decode the byte being sent
            self.shift = (self.shift << 1) | sda
            self.nbits += 1
        elif self.nbits == 8:
            ack = sda == 0
            self.bytes += 1
            self.acks += int(ack)
            self.nacks += int(not ack)
            self.transfers.append(
                ByteTransfer(t, self.shift, ack, self.first_byte,
                             (self.shift & 1) if self.first_byte else None)
            )
            self.first_byte = False
            self.nbits = 9

    def _period(self, t: int) -> None:
        """fSCL: the SCL period ending at the rising edge seen on tick `t`.

        A rise is only a clock edge if nothing moved SDA while its high period lasted, and
        that is not known until SCL falls again, so the rise is held in `rise_pending`
        until the next rise turns it into a confirmed edge. `_start` and `_stop` throw a
        pending rise away, which is how the START and STOP pulses -- SCL high for only
        tHD;STA + tSU;STA -- stay out of the measurement. The price is that the first bit
        time of a transaction and the last one before every condition are not measured;
        `periods` is the number that were.
        """
        pending, self.rise_pending = self.rise_pending, t
        if pending is None:
            return
        period = pending - self.rise_confirmed if self.rise_confirmed is not None else 0
        if period <= 0:                     # the first clock edge there is nothing to pair
            self.rise_confirmed = pending
            return
        self.rise_confirmed = pending
        self.periods += 1
        self.shortest_period = (period if self.shortest_period is None
                                else min(self.shortest_period, period))
        self._violation(t, "fSCL", period, self.min["min_period"],
                        "SCL period is %.1f kbit/s, %s caps fSCL at %.1f kbit/s"
                        % (1e6 / (self.ns_per_tick * period), self.min["mode"],
                           self.min["freq"] / 1e3))
