"""Pin-level IEEE 1149.1 JTAG reference models on the tick-based bus in `core.py`.

Four nets, all push-pull: TCK, TMS, TDI, TDO. A TAP is a 16-state machine clocked by
TCK: TDI and TMS are sampled on the rising edge, TDO changes on the falling edge and
is read on the rising edge. That is an SPI mode 1 transfer with TMS acting as the
per-cycle "chip select" (`research/ref/protocols/jtag/wikipedia-jtag.txt`).

  JtagMaster    TAP master. Queued IR and DR scans, `reset()`, samples TDO on the
                rising edge of TCK, logs every scan with its bits and its ticks.
  JtagTarget    a TAP: an instruction register of `ir_len` bits, IDCODE (32 bits),
                BYPASS (1 bit) and any user data registers, each with its opcode.
  JtagChecker   passive monitor. Decodes the wire and flags every rule it breaks,
                with the tick it happened on. 15 rules, listed in `JtagChecker`.
  attach_*      build a device and attach it to a `Bus` in one line.

Typical use:

    bus = jtag_bus()                                   # 50 MHz, four push-pull nets
    m = attach_master(bus)
    tap = attach_target(bus)
    chk = attach_checker(bus)
    m.reset()                                          # 5 TCK cycles with TMS high
    m.scan_ir(tap.opcode("IDCODE"))
    m.scan_dr(0xFFFFFFFF)                             # 32 bits out
    bus.run_until(lambda b: not m.busy, 100_000)       # `busy`, not `idle`: `idle()`
    assert chk.clean, chk.report()                     # queues idle TCK cycles

Every timing number this file uses is in `JtagTimings`, and every field of it cites
the text saved under `research/ref/protocols/jtag/`: the AC table comes from the IAR
I-jet probe's JTAG port (`ac-timing-iar-ijet.txt`), because IEEE 1149.1 itself fixes
no AC numbers -- "data presented to TDI must be valid for some chip-specific Setup
time before and Hold time after the relevant (here, rising) clock edge"
(`wikipedia-jtag.txt`). So `jtag_timings()` converts a *named* profile into whole
ticks the way `verif/vip/i2c.py` does its speed modes, and any profile can be passed
instead. Read that README before trusting a number here.

## The tick model, in one paragraph

One TCK cycle is `tck_low + tck_high` ticks. For a cycle whose first tick is `t` the
master drives TCK low on ticks `t .. t+tck_low-1` and high on `t+tck_low ..
t+tck_low+tck_high-1`, with this cycle's TMS and TDI on every one of them. Because
`core.py` resolves drives at the end of a tick, the bus then reads

  * TCK falling at tick `f = t + 1`, and the new TMS/TDI on the bus from `f` too, so
    the master's data launch is *zero* ticks after the falling edge it caused;
  * TCK rising at tick `r = f + tck_low`, low for `tck_low` ticks and high for
    `tck_high` ticks, exactly;
  * the master samples TDO at tick `r`, the first tick the bus reads TCK high.

A target that reacts to the falling edge at tick `f` drives its next TDO bit at `f+1`
and the bus shows it from `f+2`, which is `core.py`'s flop convention, so the master has
`tck_low - 2` ticks of margin on it. That is why `MIN_HALF = 2` is a floor and not 1:
at one tick of low time the bit arrives on the very tick the master samples it, and no
target could be read at all -- the same caveat `verif/vip/spi.py` gives for a degenerate
half period. The default master uses a half period of 3 at 20 ns/tick, which is 12.5 MHz
TCK with a tick of margin.

## Rules the checker enforces

Timing (the master side, from the probe's AC table):

  TCK.low, TCK.high      TCK may not be shorter than the profile's low/high period
  TCK.period             TCK may not be faster than the profile's maximum frequency
  TDI.setup, TMS.setup   TDI/TMS must settle within `tdi_tms_valid` ticks of the
                         falling TCK edge, not later
  TDO.setup              TDO must stay valid for the whole low phase once it changes,
                         within the profile's Ttdo
  TDI/TMS/TDO.change-on-high  none of the three may change while TCK reads high

Framing (both sides):

  TDO.inactive           TDO must be released outside Shift-IR and Shift-DR
  IR.capture             the two bits shifted out of Capture-IR must be 0, 1
  IR.length              an IR scan must shift exactly `ir_len` bits
  IDCODE.lsb             bit 0 of a captured IDCODE must be 0
  DR.length              a DR scan must shift exactly the selected DR's length
  RESET.tms-high         five rising edges with TMS high must leave the TAP in
                         Test-Logic-Reset, whatever state it started in

A violation is a `Violation(tick, rule, measured, required, detail)`; `checker.clean`
is the one-liner a test asserts, and `checker.report()` prints them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional

from .core import Bus, Device

TCK = "tck"
TMS = "tms"
TDI = "tdi"
TDO = "tdo"
NETS = (TCK, TMS, TDI, TDO)

# TCK idles low and every device releases TDO it is not shifting, so the pull-ups on
# the bus are what make an idle or released line read 1.
IDLE_LEVEL = 0
RELEASED_LEVEL = 1

# -- the 16 states, in the order of the transition table below -------------------
TEST_LOGIC_RESET = "Test-Logic-Reset"
RUN_TEST_IDLE = "Run-Test/Idle"
SELECT_DR_SCAN = "Select-DR-Scan"
CAPTURE_DR = "Capture-DR"
SHIFT_DR = "Shift-DR"
EXIT1_DR = "Exit1-DR"
PAUSE_DR = "Pause-DR"
EXIT2_DR = "Exit2-DR"
UPDATE_DR = "Update-DR"
SELECT_IR_SCAN = "Select-IR-Scan"
CAPTURE_IR = "Capture-IR"
SHIFT_IR = "Shift-IR"
EXIT1_IR = "Exit1-IR"
PAUSE_IR = "Pause-IR"
EXIT2_IR = "Exit2-IR"
UPDATE_IR = "Update-IR"

TAP_STATES = (
    TEST_LOGIC_RESET, RUN_TEST_IDLE, SELECT_DR_SCAN, CAPTURE_DR, SHIFT_DR,
    EXIT1_DR, PAUSE_DR, EXIT2_DR, UPDATE_DR, SELECT_IR_SCAN, CAPTURE_IR,
    SHIFT_IR, EXIT1_IR, PAUSE_IR, EXIT2_IR, UPDATE_IR,
)

#: `TAP_TRANSITIONS[state][tms]` is the next state. The table is
#: `tap-state-machine-pbeirne.txt` ("Default State Transitions"), cross-checked
#: against the state descriptions in `segger-kb-jtag.txt`.
TAP_TRANSITIONS: dict[str, tuple[str, str]] = {
    TEST_LOGIC_RESET: (RUN_TEST_IDLE, TEST_LOGIC_RESET),
    RUN_TEST_IDLE: (RUN_TEST_IDLE, SELECT_DR_SCAN),
    SELECT_DR_SCAN: (CAPTURE_DR, SELECT_IR_SCAN),
    CAPTURE_DR: (SHIFT_DR, EXIT1_DR),
    SHIFT_DR: (SHIFT_DR, EXIT1_DR),
    EXIT1_DR: (PAUSE_DR, UPDATE_DR),
    PAUSE_DR: (PAUSE_DR, EXIT2_DR),
    EXIT2_DR: (SHIFT_DR, UPDATE_DR),
    UPDATE_DR: (RUN_TEST_IDLE, SELECT_DR_SCAN),
    SELECT_IR_SCAN: (CAPTURE_IR, TEST_LOGIC_RESET),
    CAPTURE_IR: (SHIFT_IR, EXIT1_IR),
    SHIFT_IR: (SHIFT_IR, EXIT1_IR),
    EXIT1_IR: (PAUSE_IR, UPDATE_IR),
    PAUSE_IR: (PAUSE_IR, EXIT2_IR),
    EXIT2_IR: (SHIFT_IR, UPDATE_IR),
    UPDATE_IR: (RUN_TEST_IDLE, SELECT_DR_SCAN),
}

SHIFT_STATES = (SHIFT_DR, SHIFT_IR)
CAPTURE_STATES = (CAPTURE_DR, CAPTURE_IR)
UPDATE_STATES = (UPDATE_DR, UPDATE_IR)

#: opcodes the built-in target uses. BYPASS is all ones whatever the IR width
#: ("an opcode of all ones regardless of the TAP's instruction register size",
#: wikipedia-jtag.txt); IDCODE's opcode is implementer-defined, and these two are the
#: values OpenOCD's own documentation prints for an IDCODE TAP
#: (openocd-tap-declaration.txt: "jtag newtap chip1 cpu -irlen 4 -expected-id
#: 0x3ba00477").
OPCODE_IDCODE = 0x02
OPCODE_SCRATCH = 0xE0
OPCODE_BYPASS = 0xFF

#: What the built-in target answers with: version 1, JEP-106 manufacturer 0xBA0, part
#: 0x023, bit 0 clear -- the layout `idcode_fields` packs. Real parts' IDCODEs are
#: often odd in bit 0 and OpenOCD masks that bit out (`bit-order-svf-tutorial.txt`,
#: openocd-tap-declaration.txt).
DEFAULT_IDCODE = 0x10BA0046

#: "JTAG requires the two LSBs of this value to be 01" (openocd-tap-declaration.txt):
#: bit 0 is the first bit shifted out of the IR and is 1, bit 1 is 0.
IR_CAPTURE_LSB_PATTERN = 0b01

#: How many rising edges with TMS high force Test-Logic-Reset ("if TMS is high for at
#: least five clock cycles", segger-kb-jtag.txt).
RESET_CLOCKS = 5

#: The shortest legal TCK half period, in ticks. A target that sees the falling edge at
#: tick `f` drives the next TDO bit at `f + 1` and the bus shows it from `f + 2`
#: (`core.py`'s flop convention), and the master samples on the rising edge at
#: `f + tck_low`: so the low time has to be at least two ticks for the bit to be there.
MIN_HALF = 2

#: opcode -> (name, length in bits), the built-in target's instruction table. An
#: opcode that is not in a table selects BYPASS ("a single-bit register that passes
#: information from TDI to TDO", segger-kb-jtag.txt), so both defaults map everything
#: else onto it.
DEFAULT_INSTRUCTIONS = {
    OPCODE_IDCODE: ("IDCODE", 32),
    OPCODE_SCRATCH: ("SCRATCH", 8),
    OPCODE_BYPASS: ("BYPASS", 1),
}

#: opcode -> data-register length, which is all `JtagMaster` needs to size a scan.
DEFAULT_DR_LENGTHS = {OPCODE_IDCODE: 32, OPCODE_SCRATCH: 8, OPCODE_BYPASS: 1, None: 1}

def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as `Bus.ticks()`."""
    return max(0, -int(-float(ns) // float(ns_per_tick)))


def next_state(state: str, tms: int) -> str:
    """The state a TAP goes to from `state` with `tms` sampled on a rising TCK edge."""
    return TAP_TRANSITIONS[state][1 if tms else 0]


def scan_bits(value: int, width: int) -> list[int]:
    """`width` bits of `value`, LSB first: the order a JTAG scan puts them on TDI.

    "Least significant bit (LSB) comes first", in both directions
    (`bit-order-svf-tutorial.txt`), which is what the hardware does: the shift
    register moves towards TDO, so the first bit to arrive ends up at the TDO end of
    the register, which is its bit 0.
    """
    value = int(value) & ((1 << int(width)) - 1)
    return [(value >> i) & 1 for i in range(int(width))]


def scan_value(bits) -> int:
    """A LSB-first bit sequence -- the order TDO arrives -- as an integer."""
    return sum((1 if bit else 0) << k for k, bit in enumerate(bits))


def mask_opcode(opcode: int, ir_len: int) -> int:
    """An opcode as it is stored in an `ir_len`-bit instruction register."""
    return int(opcode) & ((1 << int(ir_len)) - 1)


def idcode_fields(manufacturer: int, part: int, version: int = 0) -> int:
    """Pack an IDCODE: bit 0 is 0, bits 11:1 the part number, bits 27:12 the
    manufacturer (JEP-106) code, bits 31:28 the version (`wikipedia-jtag.txt`;
    the version field is "bit 28-31 of the IDCODE", openocd-tap-declaration.txt)."""
    value = ((int(version) & 0xF) << 28) | ((int(manufacturer) & 0xFFFF) << 12)
    value |= (int(part) & 0x7FF) << 1      # bits 11:1, which leaves bit 0 at 0
    return value


def jtag_bus(ns_per_tick: float = 20.0, trace: bool = False) -> Bus:
    """A `Bus` with the four JTAG nets, all push-pull.

    TCK's pull-up is 0 and the other three are 1, which is what an idle JTAG bus looks
    like: the clock sits low, and TMS, TDI and a released TDO are pulled high.
    """
    bus = Bus(ns_per_tick=ns_per_tick, trace=trace)
    for net in NETS:
        bus.add_net(net, push_pull=True,
                    pullup=IDLE_LEVEL if net == TCK else RELEASED_LEVEL)
    return bus


@dataclass(frozen=True)
class JtagTimings:
    """One JTAG AC timing profile, every field in nanoseconds (or Hz for the rate).

    IEEE 1149.1 fixes the *function* and leaves every time to the device
    ("chip-specific", wikipedia-jtag.txt), so a profile names the part or probe whose
    datasheet the numbers come from. `tck_low`, `tck_high` and `tdo_valid` are floors;
    `tdi_tms_valid` and `max_tck_hz` are ceilings.
    """

    name: str
    tck_low: float          # TCK LOW period, minimum (Ttckl)
    tck_high: float         # TCK HIGH period, minimum (Ttckh)
    tdi_tms_valid: float    # TDI/TMS valid from the falling TCK edge, maximum (Tsod)
    tdo_valid: float        # TDO valid length after it changes, minimum (Ttdo)
    max_tck_hz: float       # fastest TCK the chain may run at (ceil of a typical range)
    source: str = ""        # file under research/ref/protocols/jtag/ it is quoted from


IJET_TIMINGS = JtagTimings(
    "ijet", 15.6, 15.6, 2.0, 15.6, 10e6,
    "ac-timing-iar-ijet.txt (Table 1: Ttckl, Ttckh, Tsod, Ttdo); max_tck_hz from "
    "wikipedia-jtag.txt, 'typically 10-100 MHz', taking the low end",
)

JTAG_PROFILES = {"ijet": IJET_TIMINGS}

_TIMING_KEYS = ("tck_low", "tck_high", "tdi_tms_valid", "tdo_valid")


def jtag_timings(
    ns_per_tick: float = 20.0, profile: str = "ijet", overrides: Optional[dict] = None
) -> dict:
    """Every spec minimum and ceiling for `profile`, as whole ticks.

    The nanosecond fields are rounded *up* to whole ticks, so a tick grid can only
    ever make a check stricter than the datasheet, never lenient. `overrides` replaces
    individual fields by their names and is how a test drives one device off-spec on
    purpose (`{"tck_low": 1}` asks for a clock faster than the profile allows).

    `min_period` is the TCK period floor that `max_tck_hz` implies, and it is what
    `JtagChecker` checks: the slowest device in a daisy chain sets the rate
    ("varies depending on all chips in the chain (the lowest speed must be used)",
    wikipedia-jtag.txt).

    At 20 ns/tick this profile asks for one tick of TDO validity after TDO changes, so a
    master has to give the low phase at least two ticks to meet it and three to leave the
    default master a margin. Below about 5 ns/tick the numbers stop being representable and
    this function reports small values rather than complaining, exactly as
    `verif/vip/i2c.py` does.
    """
    try:
        p = JTAG_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown JTAG timing profile {profile!r}; pick one of {sorted(JTAG_PROFILES)}"
        ) from None
    ns = {key: getattr(p, key) for key in _TIMING_KEYS}
    overrides = dict(overrides or {})
    freq = float(overrides.pop("max_tck_hz", p.max_tck_hz))
    for key, value in overrides.items():
        if key not in ns:
            raise ValueError(
                f"unknown timing override {key!r}; pick from {sorted(ns) + ['max_tck_hz']}"
            )
        ns[key] = float(value)
    if freq <= 0:
        raise ValueError(f"max_tck_hz must be positive, got {freq!r}")
    out = {key: ns_to_ticks(ns_per_tick, value) for key, value in ns.items()}
    out["max_tck_hz"] = freq
    out["min_period"] = ns_to_ticks(ns_per_tick, 1e9 / freq)
    out["profile"] = profile
    out["source"] = p.source
    out["ns"] = ns
    return out


@dataclass
class Scan:
    """One scan the master was asked to run. Filled in as the bus runs.

    `out_bits` are the TDO bits in the order they arrived, the register's LSB first, so
    `value` is the register's contents as the master read it. `sent` is what went out on
    TDI, and both directions are LSB first ("Least significant bit (LSB) comes first",
    bit-order-svf-tutorial.txt).
    """

    kind: str                                   # "ir" | "dr" | "reset" | "walk"
    length: int = 0                             # data bits shifted (0 for reset/walk)
    sent: int = 0                               # value driven on TDI
    out_bits: list[int] = field(default_factory=list)
    t_start: Optional[int] = None               # first TCK cycle of the scan
    t_end: Optional[int] = None                 # first TCK cycle after the scan
    cycles: int = 0                             # TCK cycles it took

    @property
    def value(self) -> int:
        """The bits read out of the target, as an integer (bit 0 first off TDO)."""
        return scan_value(self.out_bits)

    @property
    def complete(self) -> bool:
        return self.t_end is not None


class JtagMaster(Device):
    """TAP master: a queue of IR scans, DR scans and resets, driven on TCK/TMS/TDI.

    Timing comes from `jtag_timings()` plus the half period, so the default master
    meets every rule `JtagChecker` enforces at the profile it was given. A short `half`
    is the off-spec switch: `half=2` drives a 4-tick clock, which meets every floor but
    is over the profile's `max_tck_hz` -- exactly what `TCK.period` is there to catch.
    `MIN_HALF` is the floor below which no target could be read at all.

    The master never drives TDO, and it does not model the one-tick latency of a real
    probe: it samples TDO from the snapshot on the first tick the bus reads TCK high,
    which is the rising edge it caused (`tck_low + 1` ticks into a cycle, see the
    module docstring). `dr_lengths` is the target's instruction table, so
    `scan_dr(value)` knows how many bits to shift; `ir_len` is the target's IR width.
    """

    name = "jtag_master"

    def __init__(
        self,
        ir_len: int = 8,
        dr_lengths: Optional[dict] = None,
        half: Optional[int] = None,
        launch_delay: int = 0,
        timing: Optional[dict] = None,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        tck_net: str = TCK,
        tms_net: str = TMS,
        tdi_net: str = TDI,
        tdo_net: str = TDO,
        name: Optional[str] = None,
    ):
        if ir_len < 2:
            raise ValueError(
                f"ir_len must be >= 2 bits ('at least two shift-register cells', "
                f"segger-kb-jtag.txt), got {ir_len}"
            )
        self.ir_len = int(ir_len)
        self.dr_lengths = (dict(DEFAULT_DR_LENGTHS) if dr_lengths is None
                           else dict(dr_lengths))
        self.half = half
        #: Ticks between the falling TCK edge and the new TMS/TDI going on the wire.
        #: 0 is what a real adapter does; see `_values()` for what a bigger one breaks.
        self.launch_delay = int(launch_delay)
        self.timing_overrides = dict(timing or {})
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.tck_net, self.tms_net, self.tdi_net, self.tdo_net = tck_net, tms_net, tdi_net, tdo_net
        if name:
            self.name = name
        self._compute_timings()
        #: Where the master believes the TAP is. A modelling choice, not a citation:
        #: with TMS high for five clocks the choice does not matter, because five
        #: clocks reach Test-Logic-Reset from *any* state (segger-kb-jtag.txt).
        self.state = TEST_LOGIC_RESET
        self.instruction: Optional[int] = None
        self.queue: list[Scan] = []
        self.log: list[Scan] = []
        self.rises = 0                       # rising TCK edges driven so far
        self._plan: list[tuple[int, int, Optional[Scan]]] = []
        #: Cycles queued but not anchored to bus ticks yet; see `_plan_queue`.
        self._pending: list[tuple[int, int, Optional[Scan]]] = []
        self._t0 = 0
        self._bus_t = 0

    def _compute_timings(self) -> None:
        self.min = jtag_timings(self.ns_per_tick, "ijet", self.timing_overrides)
        floor = max(MIN_HALF, self.min["tck_low"], self.min["tck_high"])
        half = self.half if self.half is not None else max(
            MIN_HALF, -(-self.min["min_period"] // 2)
        )
        if half < MIN_HALF:
            raise ValueError(
                f"half must be >= {MIN_HALF} ticks, got {half}: a target needs a tick to "
                f"react to the falling edge and the bus a tick to resolve, so a shorter "
                f"low time than that cannot carry a TDO bit"
            )
        self.low = max(floor, half)
        self.high = max(floor, half)
        self.period = self.low + self.high

    def bind(self, bus: Bus) -> "JtagMaster":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self.state = TEST_LOGIC_RESET
        self.instruction = None
        self.queue, self.log, self._plan, self._pending = [], [], [], []
        self.rises = 0
        self._t0 = bus.t
        self._bus_t = bus.t
        return self

    @property
    def max_tck_hz(self) -> float:
        """The rate this master drives, as the checker measures it on the wire."""
        return 1e9 / (self.ns_per_tick * self.period)

    # -- queueing -----------------------------------------------------------------
    def reset(self, clocks: int = RESET_CLOCKS) -> Scan:
        """Queue `clocks` TCK cycles with TMS high, then one with TMS low.

        Five clocks with TMS high is what forces Test-Logic-Reset from any state
        (segger-kb-jtag.txt); the extra cycle with TMS low lands in Run-Test/Idle,
        where every scan starts.
        """
        if clocks < RESET_CLOCKS:
            raise ValueError(
                f"a synchronous reset needs {RESET_CLOCKS} TCK cycles with TMS high "
                f"to reach Test-Logic-Reset, got {clocks}"
            )
        scan = Scan(kind="reset", length=0)
        self.queue.append(scan)
        for _ in range(clocks):
            self._push(scan, 1, 0)
        self._push(scan, 0, 0)
        # A TAP's instruction register holds IDCODE (or BYPASS) out of reset, so that is
        # what a following scan_dr() sizes itself from: "the instruction register is
        # preloaded with either BYPASS or IDCODE" (wikipedia-jtag.txt).
        self.instruction = OPCODE_IDCODE
        return scan

    def idle(self, clocks: int = 1) -> Scan:
        """Queue `clocks` TCK cycles in Run-Test/Idle (TMS low)."""
        scan = Scan(kind="idle", length=0)
        self.queue.append(scan)
        for _ in range(clocks):
            self._push(scan, 0, 0)
        return scan

    def scan_ir(self, instruction: int, length: Optional[int] = None) -> Scan:
        """Queue an instruction scan of `instruction` and return the Scan.

        Bits go out LSB first, so pass the plain opcode: `scan_ir(0x02)` puts 0x02 in
        the instruction register whatever width that register has.
        """
        length = self.ir_len if length is None else int(length)
        value = int(instruction) & ((1 << length) - 1)
        scan = Scan(kind="ir", length=length, sent=value)
        self.queue.append(scan)
        self._push(scan, 1, 0)          # Run-Test/Idle -> Select-DR-Scan
        self._push(scan, 1, 0)          # Select-DR-Scan  -> Select-IR-Scan
        self._push(scan, 0, 0)          # Select-IR-Scan  -> Capture-IR
        self._push(scan, 0, 0)          # Capture-IR      -> Shift-IR
        for bit in scan_bits(value, length)[:-1]:
            self._push(scan, 0, bit, sample=True)
        # The clock that leaves Shift-IR is a shifting clock too, so the last bit goes
        # in on it: `length` clocks with Shift-IR as their starting state, exactly as the
        # worked sequence in tap-state-machine-pbeirne.txt shows.
        self._push(scan, 1, scan_bits(value, length)[-1], sample=True)   # -> Exit1-IR
        self._push(scan, 1, 0)          # Exit1-IR  -> Update-IR
        self._push(scan, 0, 0)          # Update-IR -> Run-Test/Idle
        self.instruction = value
        return scan

    def scan_dr(self, value: Optional[int] = None, length: Optional[int] = None) -> Scan:
        """Queue a data scan; `value` is what goes out on TDI, LSB first.

        `length` defaults to the length of the DR the current instruction selects, from
        `dr_lengths`. A scan always ends in Run-Test/Idle, so the next one can follow
        straight on.
        """
        if length is None:
            length = self.dr_lengths.get(self.instruction)
            if length is None:
                raise ValueError(
                    f"no data-register length for instruction "
                    f"{self.instruction!r}: pass length= or set dr_lengths"
                )
        length = int(length)
        if length < 1:
            raise ValueError(f"a DR scan needs at least 1 bit, got {length}")
        sent = 0 if value is None else int(value) & ((1 << length) - 1)
        scan = Scan(kind="dr", length=length, sent=sent)
        self.queue.append(scan)
        self._push(scan, 1, 0)          # Run-Test/Idle -> Select-DR-Scan
        self._push(scan, 0, 0)          # Select-DR-Scan -> Capture-DR
        self._push(scan, 0, 0)          # Capture-DR     -> Shift-DR
        for bit in scan_bits(sent, length)[:-1]:
            self._push(scan, 0, bit, sample=True)
        self._push(scan, 1, scan_bits(sent, length)[-1], sample=True)   # -> Exit1-DR
        self._push(scan, 1, 0)          # Exit1-DR  -> Update-DR
        self._push(scan, 0, 0)          # Update-DR -> Run-Test/Idle
        return scan

    # -- driving ------------------------------------------------------------------
    def pending(self) -> int:
        """Scans queued whose TCK cycles are not on the plan yet."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return bool(self.queue) or self._plan_left(self._bus_t) > 0

    def _plan_left(self, t: int) -> int:
        """Ticks of plan still ahead of bus tick `t`."""
        return max(0, self._t0 + len(self._plan) * self.period - t)

    def pending_ticks(self, t: Optional[int] = None) -> int:
        """Ticks of driving left, so a caller can size a `bus.run()`."""
        now = self._bus_t if t is None else t
        if self.queue:
            self._plan_queue(now)
        return self._plan_left(now)

    def run(self, bus: Optional[Bus] = None, limit: Optional[int] = None) -> int:
        """Run the bus until the queue is empty. Returns the ticks it took.

        More scans may be queued between runs: the plan is indexed by absolute bus
        tick, so the cycles already planned keep their ticks.
        """
        if bus is None:
            bus = self.bus
        if bus is None:
            raise ValueError("JtagMaster.run needs a bus (pass bus= or run(bus))")
        if limit is None:
            limit = self.pending_ticks(bus.t) + 2
        start = bus.t
        bus.run_until(lambda _b: not self.busy, limit)
        return bus.t - start

    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        self._bus_t = t
        if self.queue:
            self._plan_queue(t)
        i = t - self._t0
        cycle = i // self.period if i >= 0 else -1
        phase = i % self.period if i >= 0 else 0
        if cycle < 0 or cycle >= len(self._plan):
            # Nothing left to drive: park in Run-Test/Idle with TCK low, which is the
            # idle state a real adapter leaves the chain in.
            return {self.tck_net: IDLE_LEVEL, self.tms_net: 0, self.tdi_net: 0}
        tms, tdi = self._values(cycle, i % self.period)
        # The rising edge of cycle `c` reaches the bus one tick after the last low drive,
        # which for a one-tick high phase lands in the next cycle, so the sample is timed
        # from the cycle it belongs to rather than from the tick's own phase.
        due = i - self.low - 1
        if due >= 0 and due % self.period == 0 and due // self.period < len(self._plan):
            self.rises += 1
            scan = self._plan[due // self.period][2]
            if scan is not None:
                self._sample(scan, pins.get(self.tdo_net, RELEASED_LEVEL), t)
        return {
            self.tck_net: IDLE_LEVEL if phase < self.low else 1,
            self.tms_net: tms,
            self.tdi_net: tdi,
        }

    def _sample(self, scan: Scan, level: int, t: int) -> None:
        """Bank one sampled TDO bit into the scan it belongs to."""
        if scan.t_start is None:
            scan.t_start = t
        scan.out_bits.append(level & 1)

    def _push(self, scan: Scan, tms: int, tdi: int, sample: bool = False) -> None:
        """Queue one TCK cycle, walking the state machine so `state` stays honest.

        Only the cycles inside Shift-IR / Shift-DR carry a `Scan` to sample, so a
        prefix or suffix cycle never banks a TDO bit.
        """
        self.state = next_state(self.state, tms)
        scan.cycles += 1
        self._pending.append((tms, tdi, scan if sample else None))

    def _values(self, cycle: int, phase: int) -> tuple:
        """TMS and TDI for one tick, optionally `launch_delay` ticks late.

        A real adapter drives TDI and TMS on the falling edge of TCK, which is what
        `launch_delay=0` does here. `launch_delay=n` holds the previous cycle's values for
        the first n ticks of a cycle: later than the profile's Tsod that is a setup
        violation (`TDI.setup`, `TMS.setup`), and past the low time it puts a data change
        inside the high phase, where the three `*.change-on-high` rules live. It is the
        fault hook for those five rules.
        """
        if self.launch_delay and phase < self.launch_delay:
            cycle = max(0, cycle - 1)
        return self._plan[cycle][0], self._plan[cycle][1]

    def walk_to_idle(self) -> Scan:
        """Queue enough TMS-high cycles to reach Run-Test/Idle from wherever we are.

        From any state five cycles with TMS high reach Test-Logic-Reset
        (segger-kb-jtag.txt), and one more cycle with TMS low leaves Run-Test/Idle, so
        six is always enough and the master never depends on where it started.
        """
        scan = Scan(kind="walk", length=0)
        self.queue.append(scan)
        for _ in range(RESET_CLOCKS):
            if self.state == RUN_TEST_IDLE:
                break
            self._push(scan, 1, 0)
        if self.state != RUN_TEST_IDLE:
            self._push(scan, 0, 0)
        return scan

    def _plan_queue(self, t: int) -> None:
        """Anchor the queued cycles to absolute bus ticks, starting at or after `t`.

        The plan is a list of whole TCK cycles indexed by absolute bus tick, so scans may
        be queued before the first tick or between runs: whatever is already planned keeps
        its ticks, idle cycles fill any gap, and the new cycles start where the bus is.
        """
        if not self._pending:
            return
        if not self._plan:
            self._t0 = t
        while self._t0 + len(self._plan) * self.period < t:
            self._plan.append((0, 0, None))     # whole idle cycles, so the plan stays indexed
        base = self._t0 + len(self._plan) * self.period
        for scan in self.queue:
            scan.t_start = base
            base += scan.cycles * self.period
            scan.t_end = base
            self.log.append(scan)
        self._plan += self._pending
        self._pending = []
        self.queue = []


class JtagTarget(Device):
    """A TAP: an instruction register of `ir_len` bits and a set of data registers.

    The built-in instruction table is IDCODE (32 bits), SCRATCH (8 bits, a user
    register the tests read and write) and BYPASS (1 bit); `instructions` replaces it,
    and any opcode outside it selects BYPASS. `has_idcode=False` drops IDCODE, which
    then is not in the table either, so entering Test-Logic-Reset selects BYPASS --
    "the instruction register is preloaded with either BYPASS or IDCODE"
    (wikipedia-jtag.txt).

    What it does on each rising TCK edge, in the order of the state machine:
    Capture-IR loads the capture pattern (bit 0 = 1, bit 1 = 0), Capture-DR loads the
    selected register's capture value (IDCODE, or a zero bit for BYPASS), a Shift state
    shifts one bit towards TDO and one in from TDI, and an Update state latches the
    shift register into the register's parallel output. TDO changes on the falling
    edge, as it must ("TDO should be sent on the falling edge of TCK",
    ac-timing-iar-ijet.txt), and is released outside the two Shift states, which is
    what makes the bus read 1 there.

    `ir_capture`, `capture_values` and `idcode` are the hooks a test plants a fault in:
    subclass and override, or pass a wrong value, to see the checker catch it.
    """

    name = "jtag_target"

    def __init__(
        self,
        ir_len: int = 8,
        idcode: Optional[int] = None,
        has_idcode: bool = True,
        instructions: Optional[dict] = None,
        ir_capture: Optional[int] = None,
        registers: Optional[dict] = None,
        tdo_delay: int = 0,
        idle_tdo: Optional[int] = None,
        tck_net: str = TCK,
        tms_net: str = TMS,
        tdi_net: str = TDI,
        tdo_net: str = TDO,
        name: Optional[str] = None,
    ):
        if ir_len < 2:
            raise ValueError(f"ir_len must be >= 2 bits, got {ir_len}")
        self.ir_len = int(ir_len)
        self.tck_net, self.tms_net, self.tdi_net, self.tdo_net = tck_net, tms_net, tdi_net, tdo_net
        if name:
            self.name = name
        table = dict(DEFAULT_INSTRUCTIONS if instructions is None else instructions)
        if not has_idcode:
            table.pop(OPCODE_IDCODE, None)
        self.instructions = {int(op): (str(n), int(nb)) for op, (n, nb) in table.items()}
        if not any(nb == 1 for _n, nb in self.instructions.values()):
            self.instructions[mask_opcode(OPCODE_BYPASS, self.ir_len)] = ("BYPASS", 1)
        self.has_idcode = any(n == "IDCODE" for n, _nb in self.instructions.values())
        # Nothing is masked here on purpose: a captured IDCODE has a 0 in bit 0 (the
        # bypass bit), so an odd idcode is a fault the checker should be able to see.
        self.idcode = DEFAULT_IDCODE if idcode is None else int(idcode) & 0xFFFFFFFF
        #: Bits loaded into the IR on entry to Capture-IR. Its two LSBs are fixed by the
        #: standard (IR_CAPTURE_LSB_PATTERN); the rest is implementer-defined.
        self.ir_capture = (IR_CAPTURE_LSB_PATTERN if ir_capture is None
                           else int(ir_capture) & ((1 << self.ir_len) - 1))
        #: Parallel contents of the user data registers, by name.
        self.registers = dict(registers or {"SCRATCH": 0})
        #: Ticks between the falling TCK edge and the next TDO bit appearing. 0 is what a
        #: device that puts TDO out on the falling edge does; a target that needs more of
        #: the low phase than the profile leaves it trips `TDO.setup`.
        self.tdo_delay = int(tdo_delay)
        #: What TDO should read when the TAP is not shifting. None releases it, which is
        #: the legal answer; 0 or 1 drives it, which is the `TDO.inactive` fault.
        self.idle_tdo = idle_tdo
        #: What registers with no storage of their own capture: the IDCODE and the zero
        #: bit BYPASS holds. A user register in `registers` captures its own contents
        #: unless an entry here overrides it, which is how a test plants an illegal
        #: capture (an odd IDCODE, say).
        self.capture_values = {"IDCODE": self.idcode, "BYPASS": 0}
        self.dr_reads: list[tuple] = []      # (tick, name, value read out)
        self.ir_reads: list[tuple] = []      # (tick, value read out)
        self.updates: list[tuple] = []       # (tick, name, value latched)
        self.unknown_instructions: list[tuple] = []   # (tick, opcode)
        self.rises = 0
        self._init()

    # -- helpers ------------------------------------------------------------------
    def opcode(self, name: str) -> int:
        """The opcode of an instruction in the table, e.g. `target.opcode("IDCODE")`."""
        for op, (nm, _nb) in self.instructions.items():
            if nm == name:
                return op
        raise KeyError(f"no instruction called {name!r} in {sorted(self.instructions.values())}")

    def name_of(self, opcode: Optional[int]) -> str:
        """The instruction name an opcode selects; anything unknown selects BYPASS."""
        entry = self.instructions.get(self._mask(opcode))
        return entry[0] if entry else "BYPASS"

    def dr_length(self, opcode: Optional[int]) -> int:
        """Bits in the data register an opcode selects."""
        entry = self.instructions.get(self._mask(opcode))
        return entry[1] if entry else 1

    def _mask(self, opcode: Optional[int]) -> Optional[int]:
        if opcode is None:
            return None
        return int(opcode) & ((1 << self.ir_len) - 1)

    def _init(self) -> None:
        self.state = TEST_LOGIC_RESET
        self.instruction = self._reset_instruction()
        self._ir = 0
        self._dr = 0
        self._tdo_bit = 0
        self._tdo_valid = False
        self._pending_out = 0
        self._tdo_at = 0
        self._out_bits = {SHIFT_IR: [], SHIFT_DR: []}
        self._prev_tck: Optional[int] = None

    def _reset_instruction(self) -> int:
        """What the IR holds while the TAP is in Test-Logic-Reset."""
        return OPCODE_IDCODE if self.has_idcode else mask_opcode(OPCODE_BYPASS, self.ir_len)

    # -- the device ----------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        drv = self._drive(t)
        self._advance(t, pins)
        return drv

    def _drive(self, t: int = 0) -> dict:
        # TDO only while shifting ("Data comes back from this pin, changing on the
        # falling edge of TCLK, during the I_SHIFT and D_SHIFT states",
        # tap-state-machine-pbeirne.txt). Released, the bus pull-up reads 1.
        if not self._tdo_valid:
            return {self.tdo_net: self.idle_tdo}
        if self.tdo_delay and t < self._tdo_at:
            return {self.tdo_net: None}
        return {self.tdo_net: self._tdo_bit}

    def _advance(self, t: int, pins: Mapping[str, int]) -> None:
        tck = pins[self.tck_net]
        if self._prev_tck is None:
            self._prev_tck = tck       # the first tick is a level, not an edge
            return
        if tck == self._prev_tck:
            return
        if tck == 1:
            self._rising(t, pins[self.tms_net] & 1, pins[self.tdi_net] & 1)
        else:
            self._falling(t)
        self._prev_tck = tck

    def _rising(self, t: int, tms: int, tdi: int) -> None:
        """One rising TCK edge: act in the current state, then move on."""
        self.rises += 1
        state = self.state
        if state in SHIFT_STATES:
            reg = self._ir if state == SHIFT_IR else self._dr
            nb = self.ir_len if state == SHIFT_IR else self.dr_length(self.instruction)
            out = reg & 1
            self._out_bits[state].append(out)
            reg = (reg >> 1) | (tdi << (nb - 1)) & ((1 << nb) - 1)
            if state == SHIFT_IR:
                self._ir = reg
            else:
                self._dr = reg
            # `out` is the bit the master is sampling on this edge, and it was put on
            # TDO at the falling edge before it; the bit that goes out next is the LSB
            # of the register as it now stands.
            self._pending_out = reg & 1
        elif state in CAPTURE_STATES:
            self._capture(t)
        elif state in UPDATE_STATES:
            self._update(t)
        self.state = next_state(state, tms)
        if self.state in SHIFT_STATES and state in CAPTURE_STATES:
            # The bit that goes out at the next falling edge is bit 0 of what the
            # capture just loaded.
            self._pending_out = self._shift_reg() & 1
        if self.state == TEST_LOGIC_RESET and state != TEST_LOGIC_RESET:
            # "the instruction register is preloaded with either BYPASS or IDCODE"
            # (wikipedia-jtag.txt): a reset puts the reset instruction back, so the next
            # data scan sees IDCODE without an instruction scan.
            self.instruction = self._reset_instruction()

    def _falling(self, t: int) -> None:
        """One falling TCK edge: TDO is launched here, or released outside a shift."""
        if self.state in SHIFT_STATES:
            self._tdo_bit, self._tdo_valid = self._pending_out, True
            self._tdo_at = t + self.tdo_delay
        else:
            self._tdo_valid = False

    def _capture(self, t: int) -> None:
        if self.state == CAPTURE_IR:
            self._ir = self.ir_capture & ((1 << self.ir_len) - 1)
            self._out_bits[SHIFT_IR] = []
            return
        nm = self.name_of(self.instruction)
        self._dr = self._capture_value(nm) & ((1 << self.dr_length(self.instruction)) - 1)
        self._out_bits[SHIFT_DR] = []

    def _capture_value(self, name: str) -> int:
        """What a data register loads on entry to Capture-DR."""
        if name in self.capture_values:
            return int(self.capture_values[name])
        if name in self.registers:
            return int(self.registers[name])
        return 0

    def _update(self, t: int) -> None:
        if self.state == UPDATE_IR:
            value = self._ir & ((1 << self.ir_len) - 1)
            self.instruction = value
            self.ir_reads.append((t, self._out_value(SHIFT_IR)))
            if self._mask(value) not in self.instructions:
                self.unknown_instructions.append((t, value))
            return
        nm = self.name_of(self.instruction)
        nb = self.dr_length(self.instruction)
        value = self._dr & ((1 << nb) - 1)
        self.dr_reads.append((t, nm, self._out_value(SHIFT_DR)))
        if nm in self.registers:
            self.registers[nm] = value
            self.updates.append((t, nm, value))

    # -- shift bookkeeping ---------------------------------------------------------
    def _shift_reg(self) -> int:
        return self._ir if self.state in (SHIFT_IR, CAPTURE_IR) else self._dr

    def _out_value(self, kind: str) -> int:
        """Bits shifted out during the scan that just ended, LSB of the register first."""
        out = 0
        for k, bit in enumerate(self._out_bits.get(kind, [])):
            out |= (bit & 1) << k
        return out

    @property
    def last_idcode(self) -> Optional[int]:
        """The last IDCODE this target shifted out, or None if it never did."""
        for t, nm, value in reversed(self.dr_reads):
            if nm == "IDCODE":
                return value
        return None


#: Every rule `JtagChecker` can flag, in the order the module docstring lists them. The
#: self-tests plant one fault per entry; `test_every_rule_has_a_test` checks that.
CHECKER_RULES = (
    "TCK.low", "TCK.high", "TCK.period",
    "TDI.setup", "TMS.setup", "TDO.setup",
    "TDI.change-on-high", "TMS.change-on-high", "TDO.change-on-high",
    "TDO.inactive",
    "IR.capture", "IR.length", "IDCODE.lsb", "DR.length",
    "RESET.tms-high",
)


@dataclass
class Violation:
    """One rule of the JTAG spec that the wire did not satisfy, and when."""

    tick: int
    rule: str                 # e.g. "TDI.setup", "DR.length", "IR.capture"
    measured: int
    required: int
    detail: str = ""
    #: What the rule wants: "min" measured >= required, "max" measured <= required,
    #: "eq" measured == required, "event" no measurement, just the flag.
    sense: str = "min"

    def __str__(self) -> str:
        if self.sense == "event":
            body = f"{self.rule} happened"
        elif self.sense == "max":
            body = f"{self.rule} {self.measured} ticks > {self.required}"
        else:
            body = f"{self.rule} {self.measured} vs {self.required}"
        return f"t={self.tick} {body}" + (f" ({self.detail})" if self.detail else "")


@dataclass
class ObservedScan:
    """One scan the checker decoded off the wire."""

    kind: str                  # "ir" | "dr"
    length: int                # bits it saw shifted
    value: int                 # bits read out of TDO, LSB first
    sent: int                  # bits the master drove on TDI, MSB first
    t_start: int
    t_end: int
    state: str = ""            # the instruction in force when the scan started

    def __str__(self) -> str:
        return (f"t={self.t_start} {self.kind} scan {self.length} bits: "
                f"out 0x{self.value:x} in 0x{self.sent:x}")


class JtagChecker(Device):
    """Passive JTAG monitor: decodes the wire and flags every rule it breaks.

    It drives nothing, so it sees the pins as an analyser would and measures you, not
    the devices: every number it reports is a tick count between two edges it observed.
    The 15 rules are listed in the module docstring; each appears in `violations` as a
    `Violation` carrying its tick, the measurement and what the profile asks for.

    Configuration, all of which a test may set to make the checker stricter or to point
    it at a different device:

    ir_len           the target's instruction register width (`IR.length`)
    instructions     opcode -> (name, bits), the target's table (`DR.length`)
    has_idcode       whether the target implements IDCODE, which decides the
                     instruction a reset selects (`DR.length` after a reset)
    require_idcode_lsb
                     whether a captured IDCODE must have a 0 in bit 0. Real parts
                     often do not, which is what OpenOCD's `-ignore-bypass` is for
    timing           a `jtag_timings` override dict, for the five timing rules

    The state machine starts in `initial_state`, Test-Logic-Reset by default. That is a
    modelling choice rather than a citation -- no source saved here states what a TAP does
    before its first clock -- and it cannot make the checker lenient: five rising edges
    with TMS high reach Test-Logic-Reset from any state, and every master in these tests
    resets first.
    """

    name = "jtag_check"

    def __init__(
        self,
        ir_len: int = 8,
        instructions: Optional[dict] = None,
        has_idcode: bool = True,
        require_idcode_lsb: bool = True,
        transitions: Optional[dict] = None,
        initial_state: str = TEST_LOGIC_RESET,
        timing: Optional[dict] = None,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        tck_net: str = TCK,
        tms_net: str = TMS,
        tdi_net: str = TDI,
        tdo_net: str = TDO,
        name: Optional[str] = None,
    ):
        if ir_len < 2:
            raise ValueError(f"ir_len must be >= 2 bits, got {ir_len}")
        self.ir_len = int(ir_len)
        table = dict(DEFAULT_INSTRUCTIONS if instructions is None else instructions)
        if not has_idcode:
            table.pop(OPCODE_IDCODE, None)
        self.instructions = {int(op): (str(n), int(nb)) for op, (n, nb) in table.items()}
        self.has_idcode = has_idcode
        self.require_idcode_lsb = bool(require_idcode_lsb)
        #: The decode table. Settable only so that a test can show a table without the
        #: five-clocks-to-reset guarantee is flagged (`RESET.tms-high`); leave it alone.
        self.transitions = dict(transitions or TAP_TRANSITIONS)
        #: The state the checker assumes before it has seen a rising edge. A monitor
        #: attached to a live bus cannot know it, so this is a parameter; the default is
        #: the same choice `JtagTarget` makes.
        self.initial_state = str(initial_state)
        self.timing_overrides = dict(timing or {})
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.tck_net, self.tms_net, self.tdi_net, self.tdo_net = tck_net, tms_net, tdi_net, tdo_net
        if name:
            self.name = name
        self.violations: list[Violation] = []
        self.scans: list[ObservedScan] = []
        self.rises = 0
        self.resets = 0
        self.periods = 0
        self.shortest_period: Optional[int] = None
        self._reset()

    def bind(self, bus: Bus) -> "JtagChecker":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self._reset()
        return self

    def _compute_timings(self) -> None:
        self.min = jtag_timings(self.ns_per_tick, "ijet", self.timing_overrides)

    def _reset(self) -> None:
        self.min = jtag_timings(self.ns_per_tick, "ijet", self.timing_overrides)
        self.state = self.initial_state
        # The same modelling choice the target makes: a TAP's instruction register holds
        # IDCODE (or BYPASS) out of reset, so that is what the first data scan after a
        # reset is measured against. Five TMS-high clocks make it true whatever the
        # starting state was.
        self.instruction = (OPCODE_IDCODE if self.has_idcode
                            else mask_opcode(OPCODE_BYPASS, self.ir_len))
        self._prev: Optional[tuple] = None
        self._prev_tck: Optional[int] = None
        self._prev_state: Optional[str] = None
        self._rise_tick = 0
        self._fall_tick = 0
        self._low_seen = False
        self._prev_rise: Optional[int] = None
        self._chg = {"TMS": None, "TDI": None, "TDO": None}
        self._tms_high = 0
        self._tdo_low = False
        self._ir_out = self._ir_in = self._ir_seen = 0
        self._dr_out = self._dr_in = self._dr_seen = 0
        self._pending_ir = 0
        self._scan_start = 0

    # -- what the tests ask for ----------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def max_tck_hz(self) -> Optional[float]:
        """The fastest TCK actually seen, from the shortest measured period."""
        if self.shortest_period is None:
            return None
        return 1e9 / (self.ns_per_tick * self.shortest_period)

    def name_of(self, opcode: Optional[int]) -> str:
        """The instruction name an opcode selects; anything unknown selects BYPASS."""
        entry = self.instructions.get(mask_opcode(opcode, self.ir_len)) if opcode is not None else None
        return entry[0] if entry else "BYPASS"

    def dr_length(self, opcode: Optional[int]) -> int:
        """Bits in the data register an opcode selects."""
        entry = self.instructions.get(mask_opcode(opcode, self.ir_len)) if opcode is not None else None
        return entry[1] if entry else 1

    def report(self, limit: int = 10) -> str:
        head = (f"{self.min['profile']} profile at {self.ns_per_tick} ns/tick: "
                f"{self.rises} rising TCK edges, {self.resets} resets, "
                f"{len(self.scans)} scans, {len(self.violations)} violations")
        if self.shortest_period is not None:
            head += (f", shortest TCK period {self.shortest_period} ticks "
                     f"({self.max_tck_hz() / 1e6:.2f} MHz, cap "
                     f"{self.min['max_tck_hz'] / 1e6:.2f} MHz)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   detail: str = "", sense: str = "min") -> None:
        bad = {"min": measured < required, "max": measured > required,
               "eq": measured != required}.get(sense, True)
        if bad:
            self.violations.append(Violation(t, rule, measured, required, detail, sense))

    # -- the monitor ----------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        tck = pins[self.tck_net]
        tms, tdi, tdo = pins[self.tms_net] & 1, pins[self.tdi_net] & 1, pins[self.tdo_net] & 1
        if self._prev is None:
            # The first tick only tells the monitor the levels, so there is no edge and
            # no previous value to compare against yet.
            self._prev = (tck, tms, tdi, tdo)
            self._prev_tck = tck
            self._prev_state = self.state
            self._rise_tick = self._fall_tick = t
            return {}
        ptck, ptms, ptdi, ptdo = self._prev
        changed = (tms != ptms, tdi != ptdi, tdo != ptdo)
        falling = ptck == 1 and tck == 0
        rising = ptck == 0 and tck == 1

        # TDO outside a shift state: the target must have released it, so the bus reads
        # the pull-up. One violation per low episode, not one per tick, and the tick the
        # release itself takes is not held against the target -- on the tick the TAP
        # leaves a Shift state the bus still shows the bit that was being shifted.
        was_shifting = self.state in SHIFT_STATES
        if (tdo == 0 and not was_shifting and not self._tdo_low
                and self._prev_state is not None
                and self._prev_state not in SHIFT_STATES):
            self._violation(t, "TDO.inactive", 0, 1,
                            f"TDO read low in {self.state}", "eq")
        self._tdo_low = tdo == 0 and not was_shifting

        if tck == 1 and any(changed):
            for name, did in zip(("TMS", "TDI", "TDO"), changed):
                if did:
                    self._violation(
                        t, f"{name}.change-on-high", 1, 0,
                        f"{name} changed while TCK read high; TMS/TDI are sampled and "
                        f"TDO is sent on the rising edge", "event",
                    )

        if falling:
            self._violation(t, "TCK.high", t - self._rise_tick, self.min["tck_high"],
                            "TCK high time")
            self._fall_tick = t
            self._chg = {"TMS": None, "TDI": None, "TDO": None}
        # When each data line first moved since the falling edge, so the rising edge can
        # judge setup time and TDO validity. A move during the high phase is recorded too:
        # that is `*.change-on-high`, and its lateness is also a setup miss.
        for name, did in zip(("TMS", "TDI", "TDO"), changed):
            if did and self._chg[name] is None:
                self._chg[name] = t
        if rising:
            if self._low_seen:
                self._violation(t, "TCK.low", t - self._fall_tick, self.min["tck_low"],
                                "TCK low time")
            else:
                # The low time before the first rising edge began before the checker saw
                # the bus, so there is nothing to measure it against.
                self._low_seen = True
            self._violation_setup(t)
            self._prev_state = self.state
            self._decode_rise(t, tms, tdi, tdo)
            self.rises += 1
            self._rise_tick = t
        self._prev = (tck, tms, tdi, tdo)
        self._prev_tck = tck
        return {}

    def _violation_setup(self, t: int) -> None:
        """The checks that only a rising TCK edge can settle."""
        if self._prev_rise is not None:
            period = t - self._prev_rise
            self.periods += 1
            if self.shortest_period is None:
                self.shortest_period = period
            else:
                self.shortest_period = min(self.shortest_period, period)
            hz = 1e9 / (self.ns_per_tick * period)
            self._violation(
                t, "TCK.period", period, self.min["min_period"],
                "TCK period is %.2f MHz, %s caps it at %.2f MHz"
                % (hz / 1e6, self.min["profile"], self.min["max_tck_hz"] / 1e6),
            )
        self._prev_rise = t
        chg_tdo = self._chg["TDO"]
        if chg_tdo is not None:
            self._violation(
                t, "TDO.setup", t - chg_tdo, self.min["tdo_valid"],
                "TDO must stay valid for the whole TCK low time once it has changed",
            )
        for name in ("TDI", "TMS"):
            changed_at = self._chg[name]
            if changed_at is not None:
                self._violation(
                    t, f"{name}.setup", changed_at - self._fall_tick,
                    self.min["tdi_tms_valid"],
                    f"{name} must be valid within the profile's Tsod of the falling "
                    f"TCK edge", "max",
                )

    def _decode_rise(self, t: int, tms: int, tdi: int, tdo: int) -> None:
        """Track the TAP on one rising edge and check the framing rules."""
        state = self.state
        if state == SHIFT_IR:
            self._ir_out |= (tdo & 1) << self._ir_seen
            self._ir_in |= (tdi & 1) << self._ir_seen
            self._ir_seen += 1
            if self._ir_seen == 2:
                self._violation(
                    t, "IR.capture", self._ir_out, IR_CAPTURE_LSB_PATTERN,
                    "the two bits shifted out of Capture-IR must read 0, 1", "eq",
                )
        elif state == SHIFT_DR:
            self._dr_out |= (tdo & 1) << self._dr_seen
            self._dr_in |= (tdi & 1) << self._dr_seen
            if (self._dr_seen == 0 and self.name_of(self.instruction) == "IDCODE"
                    and self.require_idcode_lsb):
                self._violation(
                    t, "IDCODE.lsb", tdo & 1, 0,
                    "bit 0 of a captured IDCODE is the bypass bit and is always 0",
                    "eq",
                )
            self._dr_seen += 1
        new = self.transitions[state][1 if tms else 0]
        if state in CAPTURE_STATES and new in SHIFT_STATES:
            self._scan_start = t
            if new == SHIFT_IR:
                self._ir_out = self._ir_in = self._ir_seen = 0
            else:
                self._dr_out = self._dr_in = self._dr_seen = 0
        if state == SHIFT_IR and new != SHIFT_IR:
            self._violation(
                t, "IR.length", self._ir_seen, self.ir_len,
                "bits shifted while in Shift-IR", "eq",
            )
            self.scans.append(ObservedScan("ir", self._ir_seen, self._ir_out,
                                           self._ir_in, self._scan_start, t,
                                           self.name_of(self.instruction)))
            self._pending_ir = self._ir_in
            self._ir_out = self._ir_seen = 0
        elif state == SHIFT_DR and new != SHIFT_DR:
            want = self.dr_length(self.instruction)
            nm = self.name_of(self.instruction)
            self._violation(
                t, "DR.length", self._dr_seen, want,
                f"bits shifted while in Shift-DR; {nm} is {want} bits", "eq",
            )
            self.scans.append(ObservedScan("dr", self._dr_seen, self._dr_out,
                                           self._dr_in, self._scan_start, t, nm))
            self._dr_out = self._dr_seen = 0
        if state == UPDATE_IR:
            self.instruction = self._pending_ir
        if new == TEST_LOGIC_RESET and state != TEST_LOGIC_RESET:
            self.resets += 1
            self.instruction = (OPCODE_IDCODE if self.has_idcode
                                else mask_opcode(OPCODE_BYPASS, self.ir_len))
        self.state = new
        self._tms_high = self._tms_high + 1 if tms else 0
        if self._tms_high >= RESET_CLOCKS and self.state != TEST_LOGIC_RESET:
            # A conditional, not a floor: once TMS has been high for five rising edges the
            # state has to be Test-Logic-Reset, whatever state it started from.
            self._violation(
                t, "RESET.tms-high", self._tms_high, RESET_CLOCKS,
                f"five rising edges with TMS high reached {self.state}, "
                f"not Test-Logic-Reset", "event",
            )


# ---- harness helpers -------------------------------------------------------------
def _ensure_nets(bus: Bus) -> None:
    """Add the four JTAG nets if they are not there yet."""
    for net in NETS:
        if net not in bus.nets:
            bus.add_net(net, push_pull=True,
                        pullup=IDLE_LEVEL if net == TCK else RELEASED_LEVEL)


def attach_master(bus: Bus, **kwargs) -> JtagMaster:
    """Build a JtagMaster that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    m = JtagMaster(bus=bus, **kwargs)
    bus.attach(m)
    return m


def attach_target(bus: Bus, **kwargs) -> JtagTarget:
    """Build a JtagTarget and attach it."""
    _ensure_nets(bus)
    tap = JtagTarget(**kwargs)
    bus.attach(tap)
    return tap


def attach_checker(bus: Bus, **kwargs) -> JtagChecker:
    """Build a passive JtagChecker for `bus` and attach it."""
    _ensure_nets(bus)
    chk = JtagChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk
