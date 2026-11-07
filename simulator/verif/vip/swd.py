"""Pin-level Serial Wire Debug (SWD) reference models on the tick bus in verif/vip/core.py.

Two nets, named "swclk" and "swdio", both push-pull with a pull-up. SWCLK is driven by the host
alone; SWDIO is driven by whichever side owns the line at that moment, because ADIv5 lets both
sides drive SWDIO high or low or release it. That also means `core.Bus` catches the one electrical
fault a two-wire debug link can have: two devices driving opposite values on the same tick.

One transfer is the three phases of ADIv5 chapter 5, with a turnaround cycle wherever the
direction of the data changes:

    REQ(8)  Trn  ACK(3)  [ Trn ]  DATA(32)  PAR(1)  Trn

* REQ is the 8-bit request byte, first bit on the wire is bit 0: start (1), APnDP, RnW, A[3:2],
  parity, stop (0), park (1). An IDCODE read is therefore 0xA5.
* ACK is three bits and nothing else: OK 0b001, WAIT 0b010, FAULT 0b100, with no parity bit of
  its own. A line nobody drives reads 1,1,1, which is how "no response" looks on the wire.
* DATA is 32 bits LSB first plus one even-parity bit. A read is target-to-host and follows the
  ACK with no turnaround; a write is host-to-target and has one.

Two timing conventions, both from core.py's docstring and both stated once here:

* A device that returns a value for a net at tick `t` has that net read it from the end of tick
  `t`; every device sees it at tick `t+1`. So both sides here react one tick after an edge, like a
  flop sampling its input. `sample_delay` (default 2) is the tick, measured from the rising
  SWCLK edge, at which a receiver reads the line; it must leave room for that one tick and for the
  half period, so `1 <= sample_delay <= ticks_per_bit // 2 - 1`.
* Real SWD hardware has the target launch a bit *at* a rising edge and the host sample it at the
  *next* rising edge. On this bus a launch at tick `t` is visible at `t+1`, so the host reads the
  line `sample_delay` ticks after each rising edge and the checker measures setup and hold from
  that same instant. Both numbers are therefore one to two ticks tighter than the wire's own
  edges: the checker is stricter than reality, never lenient.

Typical use (50 MHz chip clock, 20 ns per tick, 5 MHz SWCLK):

    bus = Bus(ns_per_tick=20)
    host = attach_host(bus, ticks_per_bit=10)          # the debug probe
    tgt = attach_target(bus, idcode=0x2BA01477)        # the SW-DP under test
    chk = attach_checker(bus)                          # optional, passive
    host.line_reset()
    idc = host.read_idcode()
    host.write_ctrl_stat(0x40000003)                  # power up, clear sticky overrun
    cs = host.read_ctrl_stat()
    assert host.run(bus, limit=200_000)
    assert idc.ok and idc.data == 0x2BA01477
    assert chk.clean, chk.report()

Every protocol constant below cites `research/ref/protocols/swd/`, which holds the quoted text it
came from; `README.md` in that directory maps each constant to its section.

What an implementation of the target side has to do, for whoever writes the firmware against
this bench (`SwdTarget` is the executable version of the list):

1. Bit-bang SWCLK, and drive SWDIO low or release it. The line is open drain with a pull-up on
   the MVP, so "release" is `pindirs 0` and "drive low" is `pindirs 1` (`docs/mvp-isa.md`,
   `## Open drain`). SWDIO changes at the falling edge of SWCLK, never while SWCLK is high [S19].
2. Count 50 consecutive clocks with SWDIO high: that is a line reset, it returns the registers
   to their power-on values, and from then on only an IDCODE read is answered [S12, S13].
3. Answer every other request: OK in 3 bits, then 32 data bits LSB first plus one even-parity
   bit for a read, or wait out one turnaround cycle and take 33 bits for a write [S3, S17]. A
   request with a bad start, stop, park or parity bit gets no answer at all [S10].
4. Wait exactly one clock cycle before driving the ACK, and let go of the line one cycle after
   the data phase [S3]. A hold-up here is the one thing this bench cannot see on the wire, so
   watch `bus.errors` for the contention it causes.

One number to check before porting to the MVP: this bench gives every device **one** tick of
reaction latency, while the MVP's input path is `T_IN = 3` ticks and its output path `T_OUT = 1`
(`docs/mvp-isa.md`, `## Timing`). A target that reacts the way `SwdTarget` does therefore has
`ticks_per_bit - 1 - T_IN` ticks of margin before the host's sampling point: 6 ticks (120 ns) at
the default `ticks_per_bit = 10`, against the 1 tick (20 ns) the DSTREAM table asks for. A
firmware author who bit-bangs both sides of the link in one engine has to add the two latencies
by hand.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Mapping, Optional

from .core import Bus, Device

# ---- the DP register map: A[3:2] on the wire, four registers, address bits [1:0] are 0b00 ----
DP_IDCODE = 0x0        # read-only on SW-DP                                 [S5, S6]
DP_ABORT = 0x0         # write-only on SW-DP                                [S5, S6]
DP_CTRL_STAT = 0x4     # read-write                                         [S7]
DP_SELECT = 0x8        # read-write                                         [S6]
DP_RDBUFF = 0xC        # read-only                                          [S6]

DP_NAMES = {0x0: "IDCODE/ABORT", 0x4: "CTRL/STAT", 0x8: "SELECT", 0xC: "RDBUFF"}

# ---- the three ACK codes, sent LSB first, no parity bit                              [S17]
ACK_OK = 0b001
ACK_WAIT = 0b010
ACK_FAULT = 0b100
ACK_NONE = 0b111       # nobody drove the line: the pull-up makes all three bits 1

ACK_NAMES = {ACK_OK: "OK", ACK_WAIT: "WAIT", ACK_FAULT: "FAULT", ACK_NONE: "no response"}

# ---- request byte, wire bit 0 first                                                 [S15, S16]
REQ_START = 1 << 0     # always 1
REQ_APNDP = 1 << 1     # 1 = Access Port access, 0 = Debug Port access
REQ_RNW = 1 << 2       # 1 = read, 0 = write
REQ_A32 = 0b11 << 3    # register address bits A[3:2]
REQ_PARITY = 1 << 5    # even parity over APnDP | RnW | A[3:2]
REQ_STOP = 1 << 6      # the stop bit's wire position; its value is always 0
REQ_PARK = 1 << 7      # always 1

REQUEST_BITS = 8       # the request phase is eight bits, start bit included
ACK_BITS = 3           # the ACK phase is three bits, parity bit not included
DATA_BITS = 32         # the data phase is 32 bits plus one parity bit

# ---- IDCODE field layout, Arm's documented SW-DP part number and designer         [S5]
IDCODE_DEFAULT = 0x2BA01477
IDCODE_VERSION = 0x2           # [31:28], implementation defined
IDCODE_PARTNO = 0xBA01         # [27:12], one of the SW-DP part numbers Arm lists
IDCODE_DESIGNER = 0x23B        # [11:1], Arm's registered JEP-106 designer ID

# ---- CTRL/STAT fields                                                            [S7, S8]
CTRL_CSYSPWRUPACK = 1 << 31    # RO, follows CSYSPWRUPREQ
CTRL_CSYSPWRUPREQ = 1 << 30    # RW
CTRL_CDBGRSTACK = 1 << 27      # RO, follows CDBGRSTREQ
CTRL_CDBGRSTREQ = 1 << 26      # RW
CTRL_WDATAERR = 1 << 7         # RO sticky: a write-data parity error was seen
CTRL_READOK = 1 << 6           # RO: the previous AP or RDBUFF read was OK
CTRL_STICKYERR = 1 << 5        # RO sticky
CTRL_STICKYCMP = 1 << 4        # RO sticky
CTRL_STICKYORUN = 1 << 1       # RO sticky: a response other than OK
CTRL_ORUNDETECT = 1 << 0       # RW: require a data phase on every response

# ---- ABORT fields. The saved text names the four clear bits (OpenOCD's
# swd_clear_sticky_errors writes STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR) but not their
# positions, so these use the order every public host uses. Only these four flags exist here.
ABORT_STKCMPCLR = 1 << 0
ABORT_STKERRCLR = 1 << 1
ABORT_WDERRCLR = 1 << 2
ABORT_ORUNERRCLR = 1 << 3

# ---- SELECT fields                                                              [S6]
SELECT_APBANKSEL = 0x00FF0000
SELECT_APSEL = 0xFF000000
SELECT_DPBANKSEL = 0x0000000F

MIN_TICKS_PER_BIT = 6      # sample_delay must fit before the falling edge: see the docstring
DEFAULT_SAMPLE_DELAY = 2
EPS = 1e-9


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as `core.Bus.ticks()`."""
    return int(ceil(max(0.0, float(ns)) / float(ns_per_tick)))


def swd_timings(ns_per_tick: float = 20.0, timing: Optional[dict] = None) -> dict:
    """Every limit the models and the checker work to, as whole ticks and whole clock cycles.

    The four nanosecond defaults are the DSTREAM SWD timing table [S19]: `t_setup` is Tis (4 ns),
    `t_hold` is Tih (1 ns), `min_high` and `min_low` are the Thigh and Tlow minimums (10 ns each).
    At 20 ns per tick all four round to one tick, so they are floors rather than a rate: a caller
    that wants a real clock-rate rule passes `min_period`, which is `min_high + min_low`.

    The three cycle counts are not nanoseconds: `turnaround` is the Trn default of exactly one
    serial clock cycle [S3], `reset_high` is the 50 clocks with SWDIO high that make a line reset
    [S12], and `reset_low` is the low phase that follows it, two clocks in the sources we could
    capture [S14]. `reset_detect` is how many consecutive high clocks make the checker decide the
    host is attempting a line reset rather than sending a frame: a request byte has at most eight
    ones in a row (its stop bit is always 0), so anything longer is a reset attempt.

    `overrides` replaces individual values by these long names, which is how a test drives one part
    of the bench off-spec without touching the others -- {"min_period": 40} makes the checker
    demand a 800 ns SWCLK period, {"t_setup": 100} makes it demand 5 ticks of setup.
    """
    ns = {"t_setup": 4.0, "t_hold": 1.0, "min_high": 10.0, "min_low": 10.0}
    cycles = {"turnaround": 1, "reset_high": 50, "reset_low": 2, "reset_detect": 10}
    overrides = dict(timing or {})
    unknown = set(overrides) - set(ns) - set(cycles) - {"min_period"}
    if unknown:
        raise ValueError(
            f"unknown timing override(s) {sorted(unknown)}; pick from "
            f"{sorted(set(ns) | set(cycles) | {'min_period'})}"
        )
    for key, value in overrides.items():
        if key in ns:
            ns[key] = value
        else:
            cycles[key] = int(value)
    for key, value in cycles.items():
        if value < 0:
            raise ValueError(f"{key} must be >= 0, got {value!r}")
    out = {key: ns_to_ticks(ns_per_tick, value) for key, value in ns.items()}
    out.update(cycles)
    out["min_period"] = int(overrides.get("min_period", out["min_high"] + out["min_low"]))
    if out["min_period"] < 1:
        raise ValueError(f"min_period must be >= 1 tick, got {out['min_period']!r}")
    if cycles["turnaround"] < 0:
        raise ValueError("turnaround must be >= 0 clock cycles")
    if cycles["reset_high"] < 1:
        raise ValueError("reset_high must be at least one clock cycle")
    if cycles["reset_detect"] >= cycles["reset_high"]:
        raise ValueError(
            f"reset_detect ({cycles['reset_detect']}) must be below reset_high "
            f"({cycles['reset_high']}), or every reset attempt would also count as a frame"
        )
    out["ns_per_tick"] = float(ns_per_tick)
    return out


def header_parity(byte: int) -> int:
    """The request byte's parity bit: 1 when APnDP, RnW and A[3:2] hold an odd number of ones.

    ADIv5 calls this even parity over those fields [S2]; OpenOCD sets the bit exactly when
    `parity_u32()` over the same fields is 1 [S15].
    """
    fields = (int(byte) & (REQ_APNDP | REQ_RNW | REQ_A32))
    return fields.bit_count() & 1


def header_byte(ap: bool = False, read: bool = True, addr: int = 0) -> int:
    """The 8-bit request byte, first bit on the wire being bit 0 [S15, S16].

    `addr` is the register's A[3:2] field: 0x0 IDCODE (read) or ABORT (write), 0x4 CTRL/STAT,
    0x8 SELECT, 0xC RDBUFF. An IDCODE read is 0xA5 and an ABORT write is 0x81, which is the
    byte pair every public implementation sends.
    """
    if not 0 <= addr <= 0xC or addr % 4:
        raise ValueError(
            f"SWD addresses are the A[3:2] field of a DP or AP register: 0x0, 0x4, 0x8 or 0xC "
            f"(got {addr!r})"
        )
    byte = REQ_START | REQ_PARK | (((addr >> 2) & 0b11) << 3)   # A[3:2] goes to wire bits 4:3
    if ap:
        byte |= REQ_APNDP
    if read:
        byte |= REQ_RNW
    if header_parity(byte):
        byte |= REQ_PARITY
    return byte


def data_parity(value: int) -> int:
    """The data phase's parity bit: even parity over the 32 data bits [S2, S17]."""
    return (int(value) & 0xFFFFFFFF).bit_count() & 1


def decode_header(byte: int) -> dict:
    """What a request byte asks for, and whether its framing bits are right [S15, S16].

    Returns the decoded fields plus `framing_ok`, which is False when the start, stop or park bit
    is not what the spec says, and `parity_ok`. A target ignores such a request entirely [S10], and
    `SwdChecker` reports it as a START, STOP, PARK or PARITY;REQ violation.
    """
    byte = int(byte) & 0xFF
    fields = byte & (REQ_APNDP | REQ_RNW | REQ_A32)
    addr = ((byte >> 3) & 0b11) << 2
    return {
        "byte": byte,
        "ap": bool(byte & REQ_APNDP),
        "read": bool(byte & REQ_RNW),
        "addr": addr,
        "start": bool(byte & REQ_START),
        "stop": bool(byte & REQ_STOP),
        "park": bool(byte & REQ_PARK),
        "parity_ok": bool(byte & REQ_PARITY) == bool(header_parity(byte)),
        "framing_ok": bool(byte & REQ_START) and not (byte & REQ_STOP) and bool(byte & REQ_PARK),
        "is_idcode_read": not (byte & REQ_APNDP) and bool(byte & REQ_RNW) and addr == DP_IDCODE,
    }


def data_bits(value: int, parity: Optional[int] = None) -> list[int]:
    """The 33 levels of one data phase, LSB first, parity last [S2, S17].

    This is what a host drives for WDATA and what it reads back for RDATA. Override it in a
    subclass to model a transmitter with a fault, the way `verif/vip/uart.py` does it.
    """
    value = int(value) & 0xFFFFFFFF
    bits = [(value >> k) & 1 for k in range(DATA_BITS)]
    bit = data_parity(value) if parity is None else int(parity) & 1
    return bits + [bit]


@dataclass
class SwdOp:
    """One thing the host was asked to do. Filled in while the wire runs.

    `kind` is "line_reset", "read" or "write". `ack` is the three-bit response, `data` is the
    32-bit data phase, and `waits` counts how many times the target answered WAIT before this
    transfer came back. `parity_error` means the data phase's parity bit did not match, so for a
    read `data` is garbage the host must not believe [S4].

    `queued_tick` is when the host picked the operation up, `start_tick` is the tick it observed
    the frame's first rising SWCLK edge on and `end_tick` the last one -- both one tick after the
    edge it drove, which is how this bus reports an edge to everyone but the driver. `SwdChecker`
    takes its own one tick of observation delay out, so its frame ticks are the drive ticks.
    """

    kind: str
    addr: int = 0
    cycles: int = 0            # line_reset only: high clocks to send instead of the default
    queued_tick: int = -1      # when the host picked this operation up
    value: int = 0
    ap: bool = False
    header: int = 0
    start_tick: int = -1
    end_tick: int = -1
    ack: Optional[int] = None
    data: Optional[int] = None
    parity_error: bool = False
    no_ack: bool = False
    waits: int = 0
    retries: int = 0
    error: str = ""

    def __str__(self) -> str:
        what = self.kind if self.kind == "line_reset" else (
            f"{'AP' if self.ap else 'DP'} {'read' if self.kind == 'read' else 'write'} "
            f"{DP_NAMES.get(self.addr, hex(self.addr))}")
        ack = "-" if self.ack is None else ACK_NAMES.get(self.ack, f"0b{self.ack:03b}")
        out = f"t={self.start_tick}..{self.end_tick} {what} ACK={ack}"
        if self.kind != "line_reset":
            out += f" data=0x{(self.data or 0):08x}"
        if self.parity_error:
            out += " PARITY-ERROR"
        if self.waits:
            out += f" waits={self.waits}"
        if self.retries:
            out += f" retries={self.retries}"
        return out

    @property
    def ok(self) -> bool:
        """True when the target answered OK and the data phase, if any, checked out."""
        if self.kind == "line_reset":
            return self.ack is None and not self.error
        return (self.ack == ACK_OK and not self.parity_error and not self.error)


class SwdHost(Device):
    """The debug probe: clocks SWCLK, owns SWDIO for the request and WDATA phases, and reads
    the ACK and RDATA phases back.

    Every queued operation becomes one frame on the wire, and the state machine follows the ADIv5
    layout in the module docstring: eight request bits, a turnaround, three ACK bits, then the data
    phase (33 bits, with a turnaround in front of it only for a write) and one more turnaround to
    hand the line back. `SwdOp` objects come back filled in as the bus runs.

    It is deliberately a *good* host: framing and parity are always right, the turnaround is
    always the full one cycle, and `overrun_detect` mirrors what it has written into CTRL/STAT, so
    a data phase follows a non-OK ACK exactly when the spec says one should [S1, S4]. Two hooks,
    `request_byte()` and `data_phase()`, are what a test subclasses to model a faulty probe.

    `data_offset` moves the moment SWDIO changes away from the falling edge of SWCLK: 0 (the
    default) is where the spec puts it [S19], positive is later -- less setup to the next rising
    edge -- and negative is earlier, which eats into the hold time of the bit being sampled. It is
    a fault-injection knob, and the only reason the model has a data phase it does not otherwise
    need.

    `retries` (default 0) is how many times a WAIT or a missing ACK is repeated, which is what a
    real debugger does [S4]: a WAIT is simply repeated, and a missing ACK is retried after a line
    reset, because the DP may have lost synchronisation and only an IDCODE read or 50 high clocks
    bring it back [S11, S12]. `idle_cycles` must be at least 1: a SW-DP resynchronises on a new
    header only after a cycle with the line low [S11], so a host that never idles is not heard.
    """

    name = "swd_host"

    def __init__(
        self,
        ticks_per_bit: int = 10,
        ns_per_tick: float = 20.0,
        bus: Optional[Bus] = None,
        timing: Optional[dict] = None,
        clk_net: str = "swclk",
        io_net: str = "swdio",
        sample_delay: int = DEFAULT_SAMPLE_DELAY,
        idle_cycles: int = 1,
        idle_level: Optional[int] = 0,
        overrun_detect: bool = False,
        retries: int = 0,
        data_offset: int = 0,
        warmup_ticks: Optional[int] = None,
        name: Optional[str] = None,
    ):
        self._tpb = int(ticks_per_bit)
        if self._tpb < MIN_TICKS_PER_BIT:
            raise ValueError(
                f"ticks_per_bit must be >= {MIN_TICKS_PER_BIT} (got {ticks_per_bit!r}): a device "
                f"reacts one tick after an edge, so the line has to stay put for a moment after "
                f"the sample point and before the falling edge"
            )
        self._half = self._tpb // 2
        self.sample_delay = int(sample_delay)
        if not 1 <= self.sample_delay <= self._half - 1:
            raise ValueError(
                f"sample_delay must be 1..{self._half - 1} for ticks_per_bit={self._tpb} "
                f"(got {sample_delay!r}): the sampled value has to be on the line before the host "
                f"changes it for the next period"
            )
        if int(idle_cycles) < 1:
            raise ValueError(
                f"idle_cycles must be >= 1 (got {idle_cycles!r}): the SW-DP only resynchronises on "
                f"a header after a cycle with the line low"
            )
        self.data_offset = int(data_offset)
        if not -self._half <= self.data_offset <= self._half:
            raise ValueError(
                f"data_offset must be -{self._half}..{self._half} ticks (got {data_offset!r}): it "
                f"moves the SWDIO change away from the falling edge of SWCLK, and there is only "
                f"one half period to move it in"
            )
        self.idle_cycles = int(idle_cycles)
        # Ticks the line is set up and the clock held low before a frame's first bit, so even
        # that bit has a half period of setup like every other one. Zero would give it one tick.
        self.warmup_ticks = self._half if warmup_ticks is None else int(warmup_ticks)
        if self.warmup_ticks < 0:
            raise ValueError(f"warmup_ticks must be >= 0 (got {warmup_ticks!r})")
        self.idle_level = None if idle_level is None else int(idle_level)
        self.overrun_detect = bool(overrun_detect)
        self.retries = int(retries)
        self.clk_net, self.io_net = clk_net, io_net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing = dict(timing or {})
        self.tim = swd_timings(self.ns_per_tick, self.timing)
        if name:
            self.name = name
        self.queue: list[SwdOp] = []
        self.transfers: list[SwdOp] = []      # every frame that reached the wire
        self.errors: list[str] = []
        self._reset_state()

    # ---- configuration -------------------------------------------------------
    @property
    def ticks_per_bit(self) -> int:
        return self._tpb

    @property
    def half_period(self) -> int:
        """Ticks SWCLK stays high (and low) inside one period."""
        return self._half

    @property
    def swclk_hz(self) -> float:
        """SWCLK frequency in Hz, from the bus tick rate and the period."""
        return 1e9 / (self.ns_per_tick * self._tpb)

    def bind(self, bus: Bus) -> "SwdHost":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.tim = swd_timings(self.ns_per_tick, self.timing)
        return self

    # ---- the queue -----------------------------------------------------------
    def queue_line_reset(self, cycles: Optional[int] = None) -> SwdOp:
        """Clock `cycles` SWDIO-high clocks (default 50) then `reset_low` clocks with it low [S12].

        This is what brings a DP out of reset, and an IDCODE read has to follow it [S12, S13].
        """
        if cycles is not None and int(cycles) < 1:
            raise ValueError(f"a line reset needs at least one clock, got {cycles!r}")
        op = SwdOp("line_reset", cycles=int(cycles or 0))
        self.queue.append(op)
        return op

    def queue_read(self, addr: int, ap: bool = False) -> SwdOp:
        """Queue a DP or AP register read. The result lands in `op.data`."""
        op = SwdOp("read", addr=addr, ap=ap)
        self.queue.append(op)
        return op

    def queue_write(self, addr: int, value: int, ap: bool = False) -> SwdOp:
        """Queue a DP or AP register write. Validity is confirmed by the next read [S4]."""
        op = SwdOp("write", addr=addr, value=int(value) & 0xFFFFFFFF, ap=ap,
                   data=int(value) & 0xFFFFFFFF)
        self.queue.append(op)
        return op

    def line_reset(self, cycles: Optional[int] = None) -> SwdOp:
        """Alias for `queue_line_reset`, for the common "reset then read IDCODE" opening."""
        return self.queue_line_reset(cycles)

    def read_idcode(self) -> SwdOp:
        """Queue the DPIDR read. 0xA5 on the wire, always answered, even after a line reset [S5]."""
        return self.queue_read(DP_IDCODE)

    def read_ctrl_stat(self) -> SwdOp:
        return self.queue_read(DP_CTRL_STAT)

    def write_ctrl_stat(self, value: int) -> SwdOp:
        """Write CTRL/STAT, e.g. CSYSPWRUPREQ (1<<30) or ORUNDETECT (1<<0) [S7]."""
        return self.queue_write(DP_CTRL_STAT, value)

    def write_abort(self, flags: int = 0xF) -> SwdOp:
        """Write the AP Abort register: 1 in a clear bit clears that sticky flag [S8]."""
        return self.queue_write(DP_ABORT, flags)

    def read_select(self) -> SwdOp:
        return self.queue_read(DP_SELECT)

    def write_select(self, value: int) -> SwdOp:
        return self.queue_write(DP_SELECT, value)

    def read_rdbuff(self) -> SwdOp:
        return self.queue_read(DP_RDBUFF)

    @property
    def busy(self) -> bool:
        """True while a frame is on the wire or the queue still has work."""
        return self._clocking or bool(self.queue)

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def pending(self) -> int:
        return len(self.queue)

    def run(self, bus: Bus, limit: int = 500_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- what gets driven, and what a test may break ------------------------
    def request_byte(self, op: SwdOp) -> int:
        """The 8-bit request byte for `op`. Override to model a faulty host."""
        return header_byte(op.ap, op.kind == "read", op.addr)

    def data_phase(self, value: int, write: bool) -> list[int]:
        """The 33 levels of a data phase. Override to model a faulty host."""
        return data_bits(value)

    # ---- internals -----------------------------------------------------------
    def _reset_state(self) -> None:
        self._clocking = False
        self._warmup = 0
        self._state = "idle"
        self._n = 0
        self._phase = 0
        self._cur: Optional[int] = None
        self._next: Optional[int] = None
        self._prev_clk = 0
        self._pending_sample: Optional[int] = None
        self._ack_bits: list[int] = []
        self._data_bits: list[int] = []
        self._req_bits = 0
        self._wdata: list[int] = []
        self._op: Optional[SwdOp] = None
        self._trn: Optional[int] = None      # what the host drives while the line is free
        self._idle_left = 0
        self.periods = 0

    def _begin(self, t: int) -> None:
        op = self.queue.pop(0)
        self._op = op
        self._n = 0
        op.queued_tick = t
        if op.kind == "line_reset":
            self._reset_high = op.cycles or self.tim["reset_high"]
            # both levels start equal: `_cur` is what this period drives, `_next` what the next
            # one will, and the first rising edge adopts it (see `tick`)
            self._state, self._cur, self._next = "reset_hi", 1, 1
        else:
            op.header = self.request_byte(op)
            self._req_bits = op.header
            self._wdata = self.data_phase(op.value, True) if op.kind == "write" else []
            # bit 0 of the request byte is the start bit, and it goes on the wire like any other
            # bit [S15]: a header from a faulty `request_byte()` really does start with a 0
            self._state = "req"
            self._cur = self._next = self._req_bits & 1
        self._trn = self.turnaround_level()      # what this host drives while the line is free
        self._ack_bits = []
        self._data_bits = []
        self._pending_sample = None
        self._phase = 0
        self._warmup = self.warmup_ticks
        self._clocking = True

    def _end_frame(self, t: int) -> None:
        """The trailing turnaround period is over: finish, retry or recover."""
        op = self._op
        assert op is not None
        op.end_tick = t
        self.transfers.append(op)
        self._op = None
        self._clocking = False
        self._state = "idle"
        self._pending_sample = None
        if op.kind == "line_reset":
            self._idle_left = self.idle_cycles
            return
        op.no_ack = op.ack == ACK_NONE
        if op.ack == ACK_WAIT:
            op.waits += 1
        if (op.ack == ACK_WAIT or op.no_ack) and op.retries < self.retries:
            op.retries += 1
            if op.no_ack:
                self.errors.append(
                    f"t={t} no response to {op}: line reset and retry, as a lost target needs [S11]"
                )
                self.queue.insert(0, SwdOp("line_reset"))
            self.queue.insert(0, op)
            return
        if op.no_ack:
            op.error = "no ACK"
            self.errors.append(f"t={t} no ACK for {op}")
        elif op.parity_error:
            op.error = "data parity"
            self.errors.append(f"t={t} data parity error for {op}")
        self._idle_left = 0

    def _rise(self, t: int) -> None:
        """A rising SWCLK edge: the period that just ended is done, decide the next period."""
        self.periods += 1
        self._pending_sample = t + self.sample_delay
        st = self._state
        if st == "reset_hi":
            self._n += 1
            if self._n >= self._reset_high:
                self._state, self._n, self._next = "reset_lo", 0, 0
            else:
                self._next = 1
        elif st == "reset_lo":
            self._n += 1
            if self._n >= self.tim["reset_low"]:
                self._next = None
                self._end_frame(t)
            else:
                self._next = 0
        elif st == "req":
            self._n += 1
            if self._n < REQUEST_BITS:
                self._next = (self._req_bits >> self._n) & 1
            else:
                self._state, self._n, self._next = "trn", 0, self._trn
        elif st == "trn":
            self._state, self._n, self._next = "ack", 0, self._trn
        elif st == "ack":
            self._n += 1
            self._next = None                    # the decision waits for the three samples
        elif st == "wtrn":
            # The turnaround belongs to the line, not to the data: a host that lets go of SWDIO
            # in it (the default) starts WDATA one clock later than a host that does not.
            self._state, self._n = "wdata", 0
            self._next = self._wdata[0] if self._trn is None else self._trn
        elif st == "wdata":
            self._n += 1
            if self._n < DATA_BITS + 1:
                self._next = self._wdata[self._n]
            else:
                self._state, self._n, self._next = "trn2", 0, self._trn
        elif st == "rdata":
            self._n += 1
            self._next = None                    # the samples end this phase
        elif st == "trn2":
            self._state, self._n, self._next = "idle_n", 0, self.idle_level
        elif st == "idle_n":
            # the trailing turnaround and the idle cycles are over: the frame is finished
            self._n += 1
            self._end_frame(t)

    def _sample(self, t: int, level: int) -> None:
        """Read the line `sample_delay` ticks after a rising edge: the ACK and the data phase."""
        self._pending_sample = None
        st = self._state
        if st == "ack":
            self._ack_bits.append(level)
            if len(self._ack_bits) < ACK_BITS:
                return
            op = self._op
            assert op is not None
            op.ack = sum(bit << k for k, bit in enumerate(self._ack_bits))
            self._ack_bits = []
            want_data = op.ack == ACK_OK or self.overrun_detect
            if want_data and op.kind == "read":
                self._state, self._n, self._next = "rdata", 0, self._trn
            elif want_data and op.kind == "write":
                self._state, self._n, self._next = "wtrn", 0, self._trn
            else:
                self._state, self._n, self._next = "trn2", 0, self._trn
        elif st == "rdata":
            self._data_bits.append(level)
            if len(self._data_bits) < DATA_BITS + 1:
                return
            op = self._op
            assert op is not None
            value = sum(bit << k for k, bit in enumerate(self._data_bits[:DATA_BITS]))
            parity = self._data_bits[DATA_BITS]
            op.data = value
            op.parity_error = parity != data_parity(value)
            self._data_bits = []
            self._state, self._n, self._next = "trn2", 0, None

    # ---- the device ----------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        clk = pins.get(self.clk_net, 0)
        io = pins.get(self.io_net, 1)
        rose = clk == 1 and self._prev_clk == 0
        self._prev_clk = clk
        if not self._clocking:
            if self._idle_left > 0:
                self._idle_left -= 1
            elif self.queue:
                self._begin(t)
            else:
                return {self.clk_net: 0, self.io_net: self.idle_level}
        if self._warmup > 0:
            # the data is on the line and the clock is still low, so the very first bit of the
            # frame has a half period of setup like every other bit
            self._warmup -= 1
            return {self.clk_net: 0, self.io_net: self._cur}
        if not self._clocking:
            return {self.clk_net: 0, self.io_net: self.idle_level}
        if self._op is not None and self._op.start_tick < 0:
            self._op.start_tick = t          # the first rising edge of this frame
        if rose:
            self._rise(t)
        if not self._clocking:                  # the last period of the frame just ended
            return {self.clk_net: 0, self.io_net: self.idle_level}
        if self._pending_sample is not None and t >= self._pending_sample:
            self._sample(t, io)
        # The probe writes SWDIO on the falling edge of SWDCLK [S19], so the level for the coming
        # period goes on the line now (during the low half) and is what the next rising edge
        # samples. `_cur` is what this period drives, `_next` what the period after it will.
        if self._phase == 0:
            self._cur = self._next
        high = self._phase < self._half
        if high:
            level = self._cur
        else:
            level = self._next if self._phase >= self._half + self.data_offset else self._cur
        out = {self.clk_net: 1 if high else 0, self.io_net: level}
        self._phase += 1
        if self._phase >= self._tpb:
            self._phase = 0
        return out

    def turnaround_level(self) -> Optional[int]:
        """What the host drives during a turnaround cycle: released (None), as the spec wants.

        Override to model a host that does not let go of the line. A checker watching the pins
        reports a drive during a turnaround as a Trn violation.
        """
        return None


class SwdTarget(Device):
    """The SW-DP under test: answers requests on SWDIO, drives nothing else.

    It implements the DP an emulator needs and no more: IDCODE (read-only), ABORT (write-only,
    clearing the sticky flags), CTRL/STAT (read-write, with the power-up and debug-reset
    acknowledge bits following their request bits), SELECT and RDBUFF. Access Port accesses are
    answered with OK and a zeroed data phase: there is no AP in this model, and pretending
    otherwise would be worse than saying so.

    What it does about the awkward parts of the spec, all of it from the saved text:

    * A request whose start, stop, park or parity bit is wrong gets **no response at all** [S10].
      The DP then ignores everything until the host has clocked a cycle with the line low [S11].
    * 50 clocks with SWDIO high put it in line-reset state, and from there it answers only an
      IDCODE read, which is what the connection sequence requires [S12, S13]. The line reset also
      returns the DP registers to their power-on values, except SELECT: ADIv5 does not clear
      SELECT on a line reset (OpenOCD's arm_adi_v5.h says so next to the ADIv6 code that does).
    * A write-data parity error is not applied and sets the WDATAERR sticky flag [S10].
    * A response other than OK sets the STICKYORUN sticky flag [S9].

    The fault knobs exist so a test can make a real target misbehave: `ignore` frames get no
    response, `wait` frames get WAIT, `fault` frames get FAULT, `bad_ack` frames get an ACK code
    that is not OK/WAIT/FAULT, `bad_read_parity` reads come back with the wrong parity bit, and
    `bad_write_parity` writes are accepted but flagged.
    """

    name = "swd_target"

    def __init__(
        self,
        idcode: int = IDCODE_DEFAULT,
        ns_per_tick: float = 20.0,
        bus: Optional[Bus] = None,
        timing: Optional[dict] = None,
        clk_net: str = "swclk",
        io_net: str = "swdio",
        turnaround: Optional[int] = None,
        reset_high: Optional[int] = None,
        ignore_after_reset: bool = True,
        overrun_detect: bool = False,
        ignore: int = 0,
        wait: int = 0,
        fault: int = 0,
        bad_ack: int = 0,
        bad_ack_code: int = 0b000,
        bad_read_parity: int = 0,
        bad_write_parity: int = 0,
        name: Optional[str] = None,
    ):
        if int(idcode) & 1 != 1:
            raise ValueError(f"IDCODE bit 0 is always 1 [S5], got {idcode:#x}")
        self.idcode = int(idcode) & 0xFFFFFFFF
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing = dict(timing or {})
        self.tim = swd_timings(self.ns_per_tick, self.timing)
        self.turnaround = self.tim["turnaround"] if turnaround is None else int(turnaround)
        self.reset_high = self.tim["reset_high"] if reset_high is None else int(reset_high)
        self.ignore_after_reset = bool(ignore_after_reset)
        self.overrun_detect = bool(overrun_detect)
        self.ignore, self.wait, self.fault = int(ignore), int(wait), int(fault)
        self.bad_ack, self.bad_ack_code = int(bad_ack), int(bad_ack_code) & 0b111
        self.bad_read_parity, self.bad_write_parity = int(bad_read_parity), int(bad_write_parity)
        self.clk_net, self.io_net = clk_net, io_net
        if name:
            self.name = name
        self.reads: list[tuple] = []       # (tick, addr, value) for every read answered
        self.writes: list[tuple] = []      # (tick, addr, value) for every write applied
        self.protocol_errors: list[tuple] = []   # (tick, header byte, why)
        self.ignored: list[tuple] = []     # (tick, header byte) dropped after a line reset
        self.silent: list[int] = []        # ticks of frames dropped on purpose
        self.resets = 0
        self._power_on()

    # ---- registers -----------------------------------------------------------
    def _power_on(self) -> None:
        self._ctrl = CTRL_ORUNDETECT if self.overrun_detect else 0
        self._select = 0
        self._readok = False
        self.in_line_reset = False
        self._state = "listen"
        self._out: Optional[int] = None
        self._prev_clk = 0
        self._high = 0
        self._low_since = True
        self._n = 0
        self._req_bits: list[int] = []
        self._wbits: list[int] = []
        self._ack = ACK_OK
        self._req: dict = {}
        self._read_value = 0

    def bind(self, bus: Bus) -> "SwdTarget":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.tim = swd_timings(self.ns_per_tick, self.timing)
        if self.turnaround is None:
            self.turnaround = self.tim["turnaround"]
        if self.reset_high is None:
            self.reset_high = self.tim["reset_high"]
        return self

    def ctrl_stat(self) -> int:
        """The CTRL/STAT value as a read would return it: ACKs following their requests."""
        value = self._ctrl & ~(CTRL_CSYSPWRUPACK | CTRL_CDBGRSTACK)
        if value & CTRL_CSYSPWRUPREQ:
            value |= CTRL_CSYSPWRUPACK
        if value & CTRL_CDBGRSTREQ:
            value |= CTRL_CDBGRSTACK
        if self._readok:
            value |= CTRL_READOK
        return value & 0xFFFFFFFF

    def sticky(self) -> int:
        """Just the sticky flags, for a test to assert on."""
        return self.ctrl_stat() & (CTRL_STICKYORUN | CTRL_STICKYERR | CTRL_WDATAERR)

    def _read_reg(self, info: dict) -> int:
        if info["ap"]:
            return 0                    # no Access Port here; see the class docstring
        addr = info["addr"]
        if addr == DP_IDCODE:
            return self.idcode
        if addr == DP_CTRL_STAT:
            return self.ctrl_stat()
        if addr == DP_SELECT:
            return self._select
        return 0                        # RDBUFF: nothing was posted

    # ---- the device ----------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        clk = pins.get(self.clk_net, 0)
        io = pins.get(self.io_net, 1)
        if t == 0:
            self._prev_clk = clk           # the pull-up is a level, not an edge
            return {self.io_net: self._out}
        rose = clk == 1 and self._prev_clk == 0
        self._prev_clk = clk
        if rose:
            self._rise(t, io)
        return {self.io_net: self._out}

    def _rise(self, t: int, io: int) -> None:
        self._high = self._high + 1 if io == 1 else 0
        st = self._state
        if st == "req":
            self._req_bits.append(io)
            if len(self._req_bits) >= REQUEST_BITS:
                self._finish_request(t)
        elif st == "trn":
            self._n += 1
            if self._n >= self.turnaround:
                self._out = self._ack & 1
                self._state, self._n = "ack", 0     # bit 0 is on the wire; _n is its index
        elif st == "ack":
            if self._n < ACK_BITS - 1:
                self._n += 1
                self._out = (self._ack >> self._n) & 1
            else:
                self._after_ack(t)
        elif st == "wtrn":
            # The host writes WDATA on the falling edge of the preceding period [S19], so the
            # first data bit is already on the line by this rising edge: take it now.
            self._n += 1
            if self._n >= self.turnaround:
                self._state = "wdata"
                self._wbits = [io]
                if len(self._wbits) >= DATA_BITS + 1:
                    self._finish_write(t)
        elif st == "rdata":
            self._n += 1
            if self._n < DATA_BITS:
                self._out = (self._read_value >> self._n) & 1
            elif self._n == DATA_BITS:
                self._out = self._read_parity()
            else:
                self._out = None
                self._state = "tail"
        elif st == "wdata":
            self._wbits.append(io)
            if len(self._wbits) >= DATA_BITS + 1:
                self._finish_write(t)
        else:                       # "listen", "tail", "drop", "reset_run"
            self._listen(t, io)

    def _listen(self, t: int, io: int) -> None:
        """No frame in progress: watch for a line reset or for the next request header."""
        if io == 0:
            self._low_since = True
            if self._state in ("tail", "drop", "reset_run"):
                self._state = "listen"
            return
        if self._high >= self.reset_high:
            if self._high == self.reset_high:
                self._line_reset(t)
            return
        if self._high >= self.tim["reset_detect"]:
            self._state = "reset_run"       # too many ones to be a request: a reset attempt
            return
        if not self._low_since:
            return                          # wait for a low cycle before a new header [S11]
        self._state = "req"
        self._low_since = False
        self._req_bits = [io]               # this period's bit is the start bit

    def _line_reset(self, t: int) -> None:
        """50 clocks with the line high: back to line-reset state, registers to power-on [S12]."""
        self.resets += 1
        keep_select = self._select          # ADIv5 does not clear SELECT on a line reset
        self._power_on()
        self.in_line_reset = True
        self._select = keep_select
        self._high = self.reset_high        # stay in reset_run until the line goes low
        self._state = "reset_run"
        self._low_since = False

    def _drop(self, t: int, byte: int, why: str, log: bool = True) -> None:
        # The high run is deliberately not cleared here: a dropped header is exactly what a line
        # reset looks like while the counter is still counting up to `reset_high` [S12].
        self._out = None
        self._state = "drop"
        self._low_since = False
        if log:
            self.protocol_errors.append((t, byte, why))

    def _finish_request(self, t: int) -> None:
        byte = sum(bit << k for k, bit in enumerate(self._req_bits))
        info = decode_header(byte)
        if not info["framing_ok"]:
            if all(self._req_bits) or self._high >= self.tim["reset_detect"]:
                # An all-ones stretch is a line reset being counted, not a header that went
                # wrong: the stop bit is 0 in every real header, so the reset preamble looks
                # like one [S12]. Not an error, and the run keeps counting.
                self._state = "reset_run"
                self._low_since = False
                return
            self._drop(t, byte, "framing")          # bad start, stop or park bit [S10]
            return
        if not info["parity_ok"]:
            self._drop(t, byte, "parity")
            return
        if self.in_line_reset and self.ignore_after_reset and not info["is_idcode_read"]:
            self.ignored.append((t, byte))          # only IDCODE is answered after a reset [S12]
            self._drop(t, byte, "after line reset", log=False)
            return
        if self.ignore > 0:
            self.ignore -= 1
            self.silent.append(t)
            self._drop(t, byte, "fault injection", log=False)
            return
        ack = ACK_OK
        if self.bad_ack > 0:
            self.bad_ack -= 1
            ack = self.bad_ack_code
        elif self.wait > 0:
            self.wait -= 1
            ack = ACK_WAIT
        elif self.fault > 0:
            self.fault -= 1
            ack = ACK_FAULT
        if ack != ACK_OK:
            self._ctrl |= CTRL_STICKYORUN          # any response other than OK [S9]
        if (info["ap"] and info["read"]) or (not info["ap"] and info["read"]
                                             and info["addr"] == DP_RDBUFF):
            self._readok = ack == ACK_OK           # READOK follows the ACK [S3]
        self._ack, self._req = ack, info
        self._read_value = self._read_reg(info) if info["read"] else 0
        if info["is_idcode_read"]:
            self.in_line_reset = False             # the IDCODE read takes us out of reset
        if info["read"]:
            self.reads.append((t, info["addr"], self._read_value))
        self._n = 0
        if self.turnaround < 1:
            self._out = ack & 1                    # no turnaround at all: the faulty case
            self._state, self._n = "ack", 1
        else:
            self._out = None
            self._state = "trn"

    def _after_ack(self, t: int) -> None:
        """The three ACK bits are on the wire: send read data, or wait for the host's write."""
        self._high = 0
        info = self._req
        if self._ack != ACK_OK and not self.overrun_detect:
            self._out = None                       # no data phase unless ORUNDETECT [S1]
            self._state = "tail"
            return
        if info["read"]:
            self._out = self._read_value & 1   # data bit 0; _n is the index of the bit on the wire
            self._state = "rdata"
            self._n = 0
        else:
            self._out = None
            self._state = "wtrn"
            self._n = 0
            self._wbits = []

    def _read_parity(self) -> int:
        """The parity bit of the read data phase, wrong on request for the fault tests."""
        bit = data_parity(self._read_value)
        if self.bad_read_parity > 0:
            self.bad_read_parity -= 1
            return bit ^ 1
        return bit

    def _finish_write(self, t: int) -> None:
        self._high = 0
        value = sum(bit << k for k, bit in enumerate(self._wbits[:DATA_BITS]))
        parity = self._wbits[DATA_BITS]
        info = self._req
        if parity != data_parity(value):
            self._ctrl |= CTRL_WDATAERR          # not applied, and flagged [S10]
        elif self.bad_write_parity > 0:
            self.bad_write_parity -= 1
            self._ctrl |= CTRL_WDATAERR          # applied but flagged: the other fault to test
            self._apply_write(t, info, value)
        else:
            self._apply_write(t, info, value)
        self._out = None
        self._state = "tail"

    def _apply_write(self, t: int, info: dict, value: int) -> None:
        addr = info["addr"]
        self.writes.append((t, addr, value))
        if info["ap"]:
            return                               # no Access Port here
        if addr == DP_ABORT:
            if value & ABORT_STKCMPCLR:
                self._ctrl &= ~CTRL_STICKYCMP
            if value & ABORT_STKERRCLR:
                self._ctrl &= ~CTRL_STICKYERR
            if value & ABORT_WDERRCLR:
                self._ctrl &= ~CTRL_WDATAERR
            if value & ABORT_ORUNERRCLR:
                self._ctrl &= ~CTRL_STICKYORUN
        elif addr == DP_CTRL_STAT:
            keep = self._ctrl & (CTRL_STICKYORUN | CTRL_STICKYERR | CTRL_STICKYCMP
                                 | CTRL_WDATAERR)
            self._ctrl = (value & (CTRL_CSYSPWRUPREQ | CTRL_CDBGRSTREQ | CTRL_ORUNDETECT)) | keep
            if value & CTRL_STICKYORUN:           # SW-DP clears it by writing 1 to it [S9]
                self._ctrl &= ~CTRL_STICKYORUN
            self.overrun_detect = bool(value & CTRL_ORUNDETECT)
        elif addr == DP_SELECT:
            self._select = value & 0xFFFFFFFF


def _ensure_nets(bus: Bus) -> None:
    """Add the two SWD nets if they are not there yet.

    Both are push-pull with a pull-up: SWCLK is the host's alone, and SWDIO is shared, so a tick
    on which the two sides drive opposite values shows up in `bus.errors` as a `Contention` --
    the one electrical fault a two-wire debug link can have.
    """
    for net in ("swclk", "swdio"):
        if net not in bus.nets:
            bus.add_net(net, push_pull=True)


def attach_host(bus: Bus, host_class: type = None, **kwargs) -> SwdHost:
    """Build an SwdHost that takes its timing from `bus` and attach it.

    `host_class` is how a test swaps in a subclass with a fault in it.
    """
    _ensure_nets(bus)
    host = (host_class or SwdHost)(bus=bus, **kwargs)
    bus.attach(host)
    return host


def attach_target(bus: Bus, **kwargs) -> SwdTarget:
    """Build an SwdTarget that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    target = SwdTarget(bus=bus, **kwargs)
    bus.attach(target)
    return target


def attach_checker(bus: Bus, **kwargs) -> "SwdChecker":
    """Build a passive SwdChecker for `bus` and attach it."""
    _ensure_nets(bus)
    checker = SwdChecker(bus=bus, **kwargs)
    bus.attach(checker)
    return checker


@dataclass
class Violation:
    """One spec rule the wire did not meet, with the tick it was missed on.

    `measured` and `required` are in ticks for a timing rule and in clock cycles for a framing
    rule, and `detail` says which phase or which bit it was. Ticks are in *drive* time: the
    checker's own `latency` ticks of observation delay are already taken out, so a tick here is
    the tick the driver really drove that edge on.
    """

    tick: int
    rule: str
    measured: int = 0
    required: int = 0
    detail: str = ""

    def __str__(self) -> str:
        text = f"t={self.tick} {self.rule}"
        if self.required:
            # a missed minimum reads "5 < 8"; a wrong parity bit reads "0, wanted 1"
            text += (f" {self.measured} < {self.required}" if self.measured < self.required
                     else f" {self.measured}, wanted {self.required}")
        if self.detail:
            text += f" ({self.detail})"
        return text


@dataclass
class SwdFrame:
    """One transfer as the checker decoded it from the pins."""

    tick: int                       # rising edge the request started on
    end_tick: int = 0
    header: int = 0
    ap: bool = False
    read: bool = False
    addr: int = 0
    ack: Optional[int] = None       # 0, 1, 2, 4 or 7 (no response)
    data: Optional[int] = None
    parity_error: bool = False
    no_ack: bool = False
    periods: int = 0                 # clock periods the frame took
    reset_after: bool = False       # this frame followed a line reset

    def __str__(self) -> str:
        what = f"{'AP' if self.ap else 'DP'} {'read' if self.read else 'write'} " \
               f"{DP_NAMES.get(self.addr, hex(self.addr))}"
        out = f"t={self.tick} {self.header:#04x} {what} ACK=" \
              f"{ACK_NAMES.get(self.ack, str(self.ack))}"
        if self.data is not None:
            out += f" data=0x{self.data:08x}"
        if self.parity_error:
            out += " PARITY-ERROR"
        if self.reset_after:
            out += " after-line-reset"
        return out


class SwdChecker(Device):
    """Passive bus monitor: decodes the wire from the pins and checks every rule it can see.

    It drives nothing, so it sees the bus exactly as an analyser would, and it judges the traffic
    rather than the devices. `violations` holds every rule the wire broke, each with its tick;
    `frames` holds every transfer it decoded. `clean` and `report()` are the two things a test
    normally looks at.

    The rules, and where each one comes from:

    ==============  ====================================================================
    Rule            What it catches
    ==============  ====================================================================
    `tCK`           an SWCLK period shorter than `min_period` [S19]
    `tHIGH`         an SWCLK high period shorter than `min_high` [S19]
    `tLOW`          an SWCLK low period shorter than `min_low` [S19]
    `tSU;DATA`      SWDIO not settled `t_setup` ticks before the edge that samples it [S19]
    `tHD;DATA`      SWDIO moving again within `t_hold` ticks of that edge [S19]
    `STOP`          the request's stop bit is not 0 [S15, S16]
    `PARK`          the request's park bit is not 1 [S15, S16]
    `PARITY;REQ`    the request parity bit does not match APnDP|RnW|A[3:2] [S2, S15]
    `Trn`           the line not released during a turnaround cycle [S3]
    `ACK`           an ACK code that is not OK, WAIT or FAULT [S17]
    `NO-ACK`        a well-formed request that nobody answered at all [S10]
    `PARITY;DATA`   the data phase's parity bit wrong, or a data phase of the wrong length [S2]
    `LINE-RESET`    a run of SWDIO-high clocks that looks like a line reset but is too short,
                    or an IDCODE read that does not follow one [S12, S13]
    ==============  ====================================================================

    Three things it cannot know from the wire, all of them documented rather than guessed:

    * **How long the turnaround is.** The spec default is one clock [S3] and that is what this
      decoder assumes, because a released line reads 1 whichever side is holding it. A target
      configured for more wait states (CTRL/STAT.TRNCTL, not modelled) would be reported here.
    * **Whether ORUNDETECT is set.** A data phase is only required after a non-OK ACK when it is
      [S1], and a passive monitor cannot read a register. A frame that ends after a WAIT or FAULT
      is taken at its word; a target that then sends data anyway is reported.
    * **Whether a frame is truncated.** Nothing on the wire says "the data phase stops here", so
      a data phase of the wrong length shows up as a parity error rather than as its own rule.
    * **A start bit that is not 1.** There is no START rule, and that is not an oversight: a
      frame is located by its start bit, so a header whose first bit is 0 leaves the wire
      unframed. The checker then latches onto the next high clock, decodes a byte shifted by one
      and reports that byte's STOP, PARK or PARITY;REQ -- which is what an analyser sees too. The
      bit is still enforced where it can be: `SwdTarget` refuses such a header outright [S10],
      and so does a real DP.

    Setup and hold are measured from `sample_delay` ticks after the rising edge rather than at the
    edge itself, because a device on this bus reacts one tick late. That makes every measurement
    one to two ticks tighter than the wire's own edges: stricter than reality, never lenient.
    """

    name = "swd_check"

    def __init__(
        self,
        ns_per_tick: float = 20.0,
        bus: Optional[Bus] = None,
        timing: Optional[dict] = None,
        clk_net: str = "swclk",
        io_net: str = "swdio",
        sample_delay: int = DEFAULT_SAMPLE_DELAY,
        latency: int = 1,
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing = dict(timing or {})
        self.tim = swd_timings(self.ns_per_tick, self.timing)
        self.clk_net, self.io_net = clk_net, io_net
        self.sample_delay = int(sample_delay)
        self.latency = int(latency)
        if self.latency < 0:
            raise ValueError(f"latency must be >= 0 (got {latency!r})")
        if name:
            self.name = name
        self.violations: list[Violation] = []
        self.frames: list[SwdFrame] = []
        self.resets = 0                # line resets seen
        self.shortest_period: Optional[int] = None
        self._reset_state()

    def bind(self, bus: Bus) -> "SwdChecker":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.tim = swd_timings(self.ns_per_tick, self.timing)
        return self

    # ---- what the tests ask for ------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def max_swclk_hz(self) -> Optional[float]:
        """The fastest SWCLK the wire actually ran at, from the shortest period measured."""
        if self.shortest_period is None:
            return None
        return 1e9 / (self.ns_per_tick * self.shortest_period)

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def rules(self) -> set[str]:
        """Every rule name that has been broken."""
        return {v.rule for v in self.violations}

    def report(self, limit: int = 10) -> str:
        head = (f"{self.ns_per_tick} ns/tick, {len(self.frames)} frames, {self.resets} line "
                f"reset(s), {len(self.violations)} violations")
        if self.shortest_period is not None:
            head += (f", shortest SWCLK period {self.shortest_period} ticks "
                     f"({self.max_swclk_hz / 1e6:.2f} MHz)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    # ---- internals -------------------------------------------------------------
    def _reset_state(self) -> None:
        self._prev_clk = 0
        self._state = "wait"           # wait | req | trn | ack | wtrn | data | trn2
        self._n = 0
        self._frame: Optional[SwdFrame] = None
        self._bits: list[int] = []
        self._last_rise: Optional[int] = None
        self._last_fall: Optional[int] = None
        self._io_change: int = 0       # observed tick of the last SWDIO change
        self._high_run = 0
        self._need_idcode = False
        self._pending_reset = False
        self._pending_trn: Optional[tuple] = None
        self._last_io = 1

    def _violation(self, tick: int, rule: str, measured: int, required: int,
                   detail: str = "") -> None:
        """Record a timing rule the wire did not meet, if it really did not meet it."""
        if measured < required:
            self.violations.append(Violation(tick - self.latency, rule, measured, required,
                                             detail))

    def _flag(self, tick: int, rule: str, detail: str = "",
              measured: int = 0, required: int = 0) -> None:
        """Record a framing rule the wire broke.

        `measured` and `required` are carried through where they say something useful -- the
        parity rules put the bit on the wire and the bit that was expected there -- and are zero
        otherwise.
        """
        self.violations.append(Violation(tick - self.latency, rule, measured, required, detail))

    # ---- the device ------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        clk = pins.get(self.clk_net, 0)
        io = pins.get(self.io_net, 1)
        if t == 0:
            # the bus starts at the pull-up: that is a level, not an edge
            self._prev_clk, self._last_io = clk, io
            return {}
        prev_clk = self._prev_clk
        if io != self._last_io:
            # Hold time: the line must stay put for t_hold ticks *after* the rising edge that
            # samples the bit [S19]. A device that writes on the rising edge changes the line one
            # tick later, which is exactly the one tick Tih asks for.
            if self._last_rise is not None and self._last_rise < t \
                    and t - self._last_rise < self.tim["t_hold"]:
                self._violation(t, "tHD;DATA", t - self._last_rise, self.tim["t_hold"],
                                "SWDIO moved while the sampled bit was still being read")
            self._io_change = t
            self._last_io = io
        if self._pending_trn is not None and t >= self._pending_trn[0] + self.sample_delay:
            where = self._pending_trn[1]
            self._pending_trn = None
            if io == 0:
                self._violation(t, "Trn", 0, self.tim["turnaround"],
                                f"line still driven low during the {where} turnaround")
        self._prev_clk = clk
        if clk != prev_clk:
            if clk == 0:
                self._fall(t)
            else:
                self._rise(t, io)
        return {}

    def _fall(self, t: int) -> None:
        if self._last_fall is not None and self._last_rise is not None:
            high = self._last_rise - self._last_fall
            self._violation(t, "tHIGH", high, self.tim["min_high"], "SWCLK high period")
        self._last_fall = t

    def _turnaround(self, t: int, where: str, check: bool = True) -> None:
        """Note a turnaround cycle, to be judged at its sample point.

        Nobody may be holding the line during a turnaround [S3], but a driver that releases *at*
        the rising edge -- which is what the target does [S19] -- only lets go one tick later on
        this bus. So the level is judged `sample_delay` ticks in, like every other measurement
        here, not at the edge itself.
        """
        self._pending_trn = (t, where) if check else None

    def _begin_frame(self, t: int, io: int) -> None:
        self._frame = SwdFrame(tick=t - self.latency, reset_after=self._pending_reset)
        self._pending_reset = False
        self._state = "req"
        self._n = 0
        self._bits = [io]

    def _rise(self, t: int, io: int) -> None:
        if self._last_rise is not None and self._state != "wait":
            # Only inside a frame: between frames the host has stopped the clock, and that gap
            # is not a clock period, so it is neither measured nor judged.
            period = t - self._last_rise
            self._violation(t, "tLOW", t - self._last_fall, self.tim["min_low"],
                            "SWCLK low period")
            self.shortest_period = (period if self.shortest_period is None
                                    else min(self.shortest_period, period))
            self._violation(t, "tCK", period, self.tim["min_period"], "SWCLK period")
        self._last_rise = t
        self._violation(t, "tSU;DATA", t - self._io_change, self.tim["t_setup"],
                        "SWDIO setup before the rising edge")
        self._count_high(t, io)
        st = self._state
        if st == "wait":
            # A request starts with a 1 (the start bit) and a run shorter than `reset_detect`
            # clocks cannot be a line reset, so this is a header [S15, S16].
            if io == 1 and self._high_run < self.tim["reset_detect"]:
                self._begin_frame(t, io)
        elif st == "req":
            self._bits.append(io)
            if len(self._bits) >= REQUEST_BITS:
                self._decode_request(t)
        elif st == "trn":
            self._state, self._n = "ack", 0
        elif st == "ack":
            self._bits.append(io)
            if len(self._bits) >= ACK_BITS:
                self._decode_ack(t)
        elif st == "wtrn":
            # The host writes WDATA on the falling edge of the previous period [S19], so this
            # cycle's level is already data bit 0: take it here, not one clock later.
            self._state, self._n, self._bits = "data", 0, [io]
        elif st == "data":
            self._bits.append(io)
            if len(self._bits) >= DATA_BITS + 1:
                self._decode_data(t)
        elif st == "trn2":
            self._end_frame(t)

    def _count_high(self, t: int, io: int) -> None:
        """Between frames, count the clocks that make a line reset [S12].

        A run of `reset_high` SWDIO-high clocks is a line reset, and an IDCODE read has to
        follow it [S12, S13]. A run that starts looking like a reset (`reset_detect` clocks) and
        then stops is reported, because a reset that is too short does not reset anything. A
        shorter run is ordinary framing: a request byte has at most eight ones in a row, since its
        stop bit is always 0.
        """
        if io == 1:
            self._high_run += 1
            if self._high_run == self.tim["reset_high"]:
                self.resets += 1
                self._need_idcode = True
                self._pending_reset = True
            return
        if (self._state == "wait"
                and self.tim["reset_detect"] <= self._high_run < self.tim["reset_high"]):
            self._violation(t, "LINE-RESET", self._high_run, self.tim["reset_high"],
                            "run of SWDIO-high clocks stopped short of a line reset")
        self._high_run = 0

    def _decode_request(self, t: int) -> None:
        assert self._frame is not None
        byte = sum(bit << k for k, bit in enumerate(self._bits))
        if all(self._bits):
            # Eight clocks with the line high are the start of a line reset, not a header: every
            # real header has a 0 stop bit [S12, S15]. Go back to counting, do not report it.
            self._frame = None
            self._state, self._bits = "wait", []
            return
        info = decode_header(byte)
        self._frame.header = byte
        self._frame.ap, self._frame.read = info["ap"], info["read"]
        self._frame.addr = info["addr"]
        self._frame.periods = REQUEST_BITS + (0 if info["read"] else 1)   # a write has a Trn
        if self._frame.reset_after and not info["is_idcode_read"]:
            self._flag(t, "LINE-RESET",
                       f"the first request after a line reset was not an IDCODE read "
                       f"(header {byte:#04x})")
        if info["stop"]:
            self._flag(t, "STOP", "the request's stop bit was not 0")
        if not info["park"]:
            self._flag(t, "PARK", "the request's park bit was not 1")
        if not info["parity_ok"]:
            self._flag(t, "PARITY;REQ",
                       "the request parity bit does not match APnDP|RnW|A[3:2]")
        # The turnaround cycle starts with this one: the host has to let go of SWDIO [S3].
        self._turnaround(t, "request/ACK")
        self._state, self._n, self._bits = "trn", 0, []

    def _decode_ack(self, t: int) -> None:
        assert self._frame is not None
        ack = sum(bit << k for k, bit in enumerate(self._bits))
        self._frame.ack = ack
        self._frame.periods += ACK_BITS
        if ack == ACK_NONE:
            self._frame.no_ack = True
            byte = self._frame.header
            if decode_header(byte)["framing_ok"] and decode_header(byte)["parity_ok"]:
                self._flag(t, "NO-ACK", "nobody answered a well-formed request")
        elif ack not in (ACK_OK, ACK_WAIT, ACK_FAULT):
            self._flag(t, "ACK", f"ACK code 0b{ack:03b} is not OK, WAIT or FAULT")
        self._bits = []
        if ack == ACK_OK and self._frame.read:
            # A read's data follows the ACK with no turnaround at all [S3].
            self._state, self._n = "data", 0
        elif ack == ACK_OK:
            # A write has one, and its cycle starts now [S3].
            self._turnaround(t, "ACK/WDATA")
            self._state, self._n = "wtrn", 0
        else:
            self._turnaround(t, "ACK/idle")
            self._state, self._n = "trn2", 0

    def _decode_data(self, t: int) -> None:
        assert self._frame is not None
        value = sum(bit << k for k, bit in enumerate(self._bits[:DATA_BITS]))
        parity = self._bits[DATA_BITS]
        self._frame.data = value
        self._frame.parity_error = parity != data_parity(value)
        if self._frame.parity_error:
            self._flag(t, "PARITY;DATA",
                       f"parity bit {parity} on the wire, the 32 data bits need "
                       f"{data_parity(value)}")
        self._frame.periods += DATA_BITS + 1
        # The target hands the line back at this edge: its turnaround cycle starts now [S3].
        # Only a read can be checked: after a write it is the host that owns the line, so the
        # wire cannot show the hand-back -- a target that held on would show up as a Contention
        # in `bus.errors` instead, which is where that fault belongs.
        self._turnaround(t, "data/idle", check=self._frame.read)
        self._state, self._n, self._bits = "trn2", 0, []

    def _end_frame(self, t: int) -> None:
        assert self._frame is not None
        self._frame.end_tick = t - self.latency
        self._frame.periods += 2                    # the two turnarounds
        self.frames.append(self._frame)
        self._frame = None
        self._state, self._n = "wait", 0
        self._high_run = 0