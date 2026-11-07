"""Pin-level MDIO/MDC (IEEE 802.3 clause 22) reference models on the tick bus.

Three devices on two nets named "mdc" and "mdio". MDC is a push-pull clock the
station drives; MDIO is a bidirectional line with a pull-up, so a device drives 0
or releases it (None) and never needs to drive 1. Both nets are wired-AND with
`pullup=1` *and* `push_pull=True`, which is how the core's contention detector
catches a station that drives data while the PHY is still driving the turnaround.

  MdioStation   bit-level station management entity (STA): MDC plus the MDIO
                bit stream, a queue of register reads and writes, the 32-bit
                preamble, ST / OP / PHYAD / REGAD / TA / DATA, and every timing
                below taken from the spec minimums and rounded to whole ticks.
  MdioPhy       clause 22 PHY with a 32 x 16-bit register file: BMCR, BMSR with
                its link-status latch, PHY ID, auto-negotiation registers, a
                soft reset, an optional post-reset silent window, and a link
                that a test can raise and drop. It only ever drives the second
                turnaround bit and the data bits of a read addressed to it.
  MdioChecker   passive monitor: it drives nothing, decodes every frame off the
                pins, and appends a `Violation` with its tick for each timing or
                framing rule the bus breaks.

Typical use, with the models talking to each other:

    bus = Bus(ns_per_tick=20)                       # 50 MHz chip clock
    st  = attach_station(bus)
    phy = attach_phy(bus, phyad=0)
    chk = attach_checker(bus, expect_phyad={0})
    assert st.read(bus, 1) & 0x0004                 # BMSR, link status
    assert not chk.violations, chk.report()

Both nets are `push_pull=True` with `pullup=1`: see `attach_*` and README notes.

The spec numbers, and where each one is saved in `research/ref/protocols/mdio/`:

    MDC period >= 400 ns, i.e. MDC <= 2.5 MHz     lan8720a-mdio-timing.txt
    MDC high >= 160 ns, MDC low >= 160 ns         lan8720a-mdio-timing.txt
    station data setup >= 10 ns                   lan8720a-mdio-timing.txt
    station data hold >= 10 ns                    lan8720a-mdio-timing.txt
    PHY read data valid 0 .. 300 ns after the     lan8720a-mdio-timing.txt
      rising MDC edge
    MDIO sampled on the rising MDC edge, so a     lan8720a-mdio-timing.txt,
      device may only change MDIO while MDC is low   ti-dp83tc811-smi.txt
    32 ones of preamble, then ST=01, OP=10 read / ti-dp83tc811-smi.txt,
      01 write, PHYAD[4:0], REGAD[4:0], TA, DATA[15:0]  wikipedia-mdio-clause22.txt
    read TA is <Z0> driven by the PHY, write TA is ti-dp83tc811-smi.txt
      <10> driven by the station

Every minimum is converted to whole ticks with `ns_to_ticks`, rounding up, so a
10 ns minimum is 1 tick (20 ns) at 50 MHz: at worst a hair stricter than the
spec, never lenient. Both the models and the checker take `timing=` overrides in
nanoseconds, which is how a test drives one of them off-spec on purpose.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Optional

from .core import Bus, Device

# ---- frame constants, all of them clause 22 -------------------------------
# 32 ones, ST=01, OP, 5 PHYAD, 5 REGAD, TA=2, then 16 data bits: 64 bit times.
PREAMBLE_BITS = 32                     # wikipedia-mdio-clause22.txt PRE_32
ST_BITS = 0b01                         # "always contains the combination '01'"
OP_READ = 0b10                         # ti-dp83tc811-smi.txt Table 7-15
OP_WRITE = 0b01                        # "The Opcode consists of 2 bits ..."
ADDRESS_BITS = 5                       # PA5: up to 32 PHYs
REGISTER_BITS = 5                      # RA5: 32 registers of 16 bits
DATA_BITS = 16
TURNAROUND_BITS = 2
#: Bit indexes at which the control fields are complete, i.e. the index of their
#: last bit. The checker uses them to fire the ST and OP rules as soon as the
#: field has been read, rather than at the end of the frame.
ST_END = PREAMBLE_BITS + 1
OP_END = ST_END + 2
REGAD_END = OP_END + ADDRESS_BITS + REGISTER_BITS
CONTROL_BITS = 2 + 2 + ADDRESS_BITS + REGISTER_BITS + TURNAROUND_BITS   # = 16
FRAME_BITS = PREAMBLE_BITS + CONTROL_BITS + DATA_BITS   # = 64

#: Which part of the frame each bit index belongs to, in wire order. Both the
#: models and the checker decode with this, so there is one definition of the
#: frame in the file. `bit_role` and `bit_offset` are its two accessors.
FIELDS = (
    ("preamble", PREAMBLE_BITS),
    ("st", 2),
    ("op", 2),
    ("phyad", ADDRESS_BITS),
    ("regad", REGISTER_BITS),
    ("ta1", 1),
    ("ta2", 1),
    ("data", DATA_BITS),
)
IDLE = 1                               # the pull-up value of MDIO and MDC


def bit_role(index: int) -> str:
    """The field name bit `index` of a frame belongs to ("preamble", "data", ...)."""
    n = 0
    for name, width in FIELDS:
        if index < n + width:
            return name
        n += width
    return "past-frame"


def bit_offset(index: int) -> int:
    """How many bits into its field bit `index` is (0 for the first bit)."""
    n = 0
    for name, width in FIELDS:
        if index < n + width:
            return index - n
        n += width
    return index - n


def _bits(value: int, width: int) -> list[int]:
    """`value` as `width` bits, most significant first, as the wire sends them."""
    return [(value >> (width - 1 - i)) & 1 for i in range(width)]


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))

# ---- timing ----------------------------------------------------------------
@dataclass(frozen=True)
class MdioTimings:
    """Every clause 22 timing this model uses, in nanoseconds.

    `t_period` is the only one that is a *maximum rate* rather than a floor: it
    is the shortest legal MDC period, which is the same statement as "MDC may not
    exceed 2.5 MHz" (LAN8720A Table 5.12 `tclkp` 400 ns min, and Section 3.5 "the
    minimum cycle time (time between two consecutive rising or two consecutive
    falling edges) is 400 ns"). `mdio_minimums` turns it into `period_ticks`,
    which is what both the station's bit time and the checker's `MDC` rule use.

    The others are floors, except `t_phy_max`, the deadline by which a PHY must
    have its read data on the line (LAN8720A `tval` 300 ns max, Wikipedia "the
    PHY has to provide the MDIO signal between 0 and 300 ns after the rising
    edge"), and `t_phy_min`, its hold floor (LAN8720A `tohold` 0 ns min).
    """

    t_period: float = 400.0    # MDC period floor, so the fMDC ceiling of 2.5 MHz
    t_low: float = 160.0       # MDC low time floor
    t_high: float = 160.0      # MDC high time floor
    t_setup: float = 10.0      # station data stable before the rising MDC edge
    t_hold: float = 10.0       # station data stable after the rising MDC edge
    t_phy_max: float = 300.0   # PHY read data valid, a deadline after the rise
    t_phy_min: float = 0.0     # PHY read data hold, a floor after the rise


#: The one timing class of clause 22. Named so that a test can spell it out:
#: `mdio_minimums(20, timing=MDIO_CLAUSE22.as_dict(t_period=200))`.
MDIO_CLAUSE22 = MdioTimings()

_TIMING_NAMES = tuple(MdioTimings.__dataclass_fields__)


def mdio_minimums(ns_per_tick: float, timing: Optional[dict] = None) -> dict:
    """Every spec limit as whole ticks, rounded up, for a bus of `ns_per_tick`.

    `timing` replaces individual nanosecond values by their field names, which is
    how a test drives a station or a checker off-spec on purpose:
    `{"t_period": 200}` means "MDC periods of 200 ns are legal here", which at
    20 ns/tick is 5 MHz and makes the station faster than clause 22 allows.

    Three derived values come back as well, because "how long is a bit" is not
    one of the spec's own numbers:

    `period_ticks`      `t_period` rounded up: the shortest legal bit time.
    `low_ticks`         `t_low` rounded up: the shortest MDC low phase.
    `high_ticks`        `t_high` rounded up: the shortest MDC high phase.
    `max_mdc_hz`        1 / t_period, the fMDC ceiling in Hz.

    Rounding up means one tick of quantisation never shortens a legal period's
    budget by more than a tick, so these checks are at worst a hair stricter
    than the exact spec and never lenient.
    """
    values = {name: getattr(MDIO_CLAUSE22, name) for name in _TIMING_NAMES}
    overrides = dict(timing or {})
    unknown = set(overrides) - set(_TIMING_NAMES)
    if unknown:
        raise ValueError(
            f"unknown MDIO timing(s) {sorted(unknown)}; pick from {list(_TIMING_NAMES)}")
    values.update(overrides)
    for name, value in values.items():
        if value < 0:
            raise ValueError(f"{name} must not be negative, got {value!r}")
    if values["t_period"] <= 0:
        raise ValueError(f"t_period must be positive, got {values['t_period']!r}")
    ticks = {name: ns_to_ticks(ns_per_tick, ns) for name, ns in values.items()}
    ticks["period_ticks"] = ticks["t_period"]
    ticks["low_ticks"] = ticks["t_low"]
    ticks["high_ticks"] = ticks["t_high"]
    ticks["max_mdc_hz"] = 1e9 / values["t_period"]
    return ticks


# ---- frames ----------------------------------------------------------------
@dataclass(frozen=True)
class MdioFrame:
    """One decoded access: what was on the wire, as the checker saw it.

    `phyad`, `reg` and `data` are None while the frame is still incomplete, and
    for a write `data` is what the station sent. `preamble_bits` counts the ones
    the station sent immediately before ST, `unanswered` is True for a read whose
    turnaround came back as ones (no PHY on the bus claimed the address), and
    `mdc_periods` lists every MDC period in ticks that the frame contains, so a
    test can see what the checker's `MDC` rule measured.
    """

    start: int                  # tick of the first MDC rising edge of the frame
    is_write: Optional[bool] = None
    phyad: Optional[int] = None
    reg: Optional[int] = None
    data: Optional[int] = None
    ta: Optional[int] = None
    st_bits: Optional[int] = None
    op_bits: Optional[int] = None
    preamble_bits: int = 0
    unanswered: bool = False
    mdc_periods: tuple = ()
    error: str = ""             # why the frame could not be decoded, if it could not

    def __str__(self) -> str:
        what = "?" if self.is_write is None else ("write" if self.is_write else "read")
        return (f"t={self.start} {what} PHYAD={self.phyad} REGAD={self.reg} "
                f"DATA=0x{self.data:04x}" if self.data is not None
                else f"t={self.start} {what} incomplete ({self.error})")


def encode_frame(is_write: bool, phyad: int, reg: int, data: int = 0,
                 preamble_bits: int = PREAMBLE_BITS) -> list[int]:
    """The 64 bit times of one clause 22 frame, most significant bit first.

    Turnaround comes out of this too: a write sends `<10>` (the station drives
    both bits, ti-dp83tc811-smi.txt), a read sends `<Z Z>` and then 16 Z data
    bits, because the station has released MDIO and the PHY drives the second
    turnaround bit and the data ("no device may actively drive the MDIO signal
    during the first bit of turnaround"). `None` here means "released": the
    caller must not drive that bit time.
    """
    for name, value, limit in (("phyad", phyad, (1 << ADDRESS_BITS) - 1),
                               ("reg", reg, (1 << REGISTER_BITS) - 1),
                               ("data", data, (1 << DATA_BITS) - 1)):
        if not 0 <= value <= limit:
            raise ValueError(f"{name} {value!r} does not fit in {limit + 1} bits")
    bits = [1] * preamble_bits + _bits(ST_BITS, 2) \
        + _bits(OP_WRITE if is_write else OP_READ, 2) \
        + _bits(phyad, ADDRESS_BITS) + _bits(reg, REGISTER_BITS)
    if is_write:
        bits += _bits(0b10, TURNAROUND_BITS) + _bits(data, DATA_BITS)
    else:
        bits += [None] * (TURNAROUND_BITS + DATA_BITS)
    assert len(bits) == preamble_bits + CONTROL_BITS + DATA_BITS, len(bits)
    return bits


@dataclass
class MdioOp:
    """The result of one queued station access, filled in as the frame goes out.

    `value` is what the station wrote, or what it read back; `unanswered` is a
    read that came back as 0xFFFF, which on a wired-AND bus means no PHY on the
    bus drove anything, so the station learned there is no PHY at that address.
    `start` and `end` are the ticks of the frame's first and last MDC rising
    edges, and `periods` the MDC periods in ticks it measured.
    """

    is_write: bool
    reg: int
    value: Optional[int] = None
    phyad: int = 0
    start: Optional[int] = None
    end: Optional[int] = None
    periods: list = field(default_factory=list)
    errors: list = field(default_factory=list)

    @property
    def unanswered(self) -> bool:
        return not self.is_write and self.value == 0xFFFF

    def __str__(self) -> str:
        verb = "write" if self.is_write else "read"
        tail = "" if self.value is None else f" 0x{self.value:04x}"
        if self.unanswered:
            tail += " (no PHY answered)"
        return f"{verb} REGAD={self.reg}{tail}"


# ---- the clause 22 register file ------------------------------------------
# Names and bits from lan8720a-clause22-registers.txt (LAN8720A datasheet rev 1.4
# Sections 4.2.1, 4.2.2, 4.2.3, 4.2.5). Registers 7..17 get no names and no
# behaviour: that datasheet documents the IEEE registers 0..6 and then its own
# vendor registers from 18, so nothing saved here describes them (see the README).
BMCR = 0          # Basic Mode Control Register
BMSR = 1          # Basic Mode Status Register
PHYIDR1 = 2       # PHY Identifier 1
PHYIDR2 = 3       # PHY Identifier 2
ANAR = 4          # Auto-Negotiation Advertisement
ANLPR = 5         # Auto-Negotiation Link Partner Ability
ANER = 6          # Auto-Negotiation Expansion

REGISTER_NAMES = {
    BMCR: "BMCR", BMSR: "BMSR", PHYIDR1: "PHYIDR1", PHYIDR2: "PHYIDR2",
    ANAR: "ANAR", ANLPR: "ANLPR", ANER: "ANER",
}

BMCR_RESET = 0x8000      # 15, self-clearing (type R/WSC in the datasheet)
BMCR_LOOPBACK = 0x4000   # 14
BMCR_SPEED100 = 0x2000   # 13, "0 = 10Mbps, 1 = 100Mbps"
BMCR_ANEG_EN = 0x1000    # 12, overrides 13 and 8
BMCR_POWERDOWN = 0x0800  # 11
BMCR_ISOLATE = 0x0400    # 10
BMCR_ANRESTART = 0x0200  # 9, self-clearing
BMCR_DUPLEX = 0x0100     # 8, "0 = half duplex, 1 = full duplex"
BMCR_MASK = (BMCR_LOOPBACK | BMCR_SPEED100 | BMCR_ANEG_EN | BMCR_POWERDOWN
             | BMCR_ISOLATE | BMCR_DUPLEX)
BMCR_STRAPS = BMCR_SPEED100 | BMCR_ANEG_EN | BMCR_DUPLEX  # defaults come from straps

BMSR_T4 = 0x8000         # 15, 100BASE-T4 ability
BMSR_100TX_FD = 0x4000   # 14
BMSR_100TX_HD = 0x2000   # 13
BMSR_10T_FD = 0x1000     # 12
BMSR_10T_HD = 0x0800     # 11
BMSR_T2_FD = 0x0400      # 10
BMSR_T2_HD = 0x0200      # 9
BMSR_EXT_STATUS = 0x0100  # 8
BMSR_ANEG_DONE = 0x0020  # 5
BMSR_REMOTE_FAULT = 0x0010  # 4, latch high
BMSR_ANEG_CAP = 0x0008   # 3
BMSR_LINK = 0x0004       # 2, "0 = link is down, 1 = link is up", type RO/LL
BMSR_JABBER = 0x0002     # 1, latch high
BMSR_EXT_CAPS = 0x0001   # 0

ANAR_REMOTE_FAULT = 0x2000   # 13
ANAR_PAUSE_SHIFT = 10        # 11:10
ANAR_PAUSE_MASK = 0x0C00
ANAR_100TX_FD = 0x0100       # 8
ANAR_100TX = 0x0080          # 7
ANAR_10T_FD = 0x0040          # 6
ANAR_10T = 0x0020            # 5
ANAR_SELECTOR = 0x001F        # 4:0, "00001 = IEEE 802.3"

#: Power-on contents of the whole register file, from the datasheet's DEFAULT
#: column. BMSR bit 2 (LINK) is not in here: the link state is a model input, so
#: it is added when BMSR is read. The four BMCR/ANAR bits the datasheet marks
#: "-" (determined by the MODE[2:0] straps) are taken as 1, the "all abilities"
#: power-on setting a 10/100 PHY straps to.
POWER_ON_DEFAULTS = {
    BMCR: BMCR_SPEED100 | BMCR_ANEG_EN | BMCR_DUPLEX,
    BMSR: (BMSR_100TX_FD | BMSR_100TX_HD | BMSR_10T_FD | BMSR_10T_HD
           | BMSR_ANEG_CAP | BMSR_EXT_CAPS),
    PHYIDR1: 0x0007,
    PHYIDR2: 0x0000,
    ANAR: (ANAR_100TX_FD | ANAR_100TX | ANAR_10T_FD | ANAR_10T | 0b00001),
    ANLPR: 0x0000,
    ANER: 0x0000,
}


class MdioStation(Device):
    """Bit-level station management entity: MDC plus the MDIO bit stream.

    A station access is one 64-bit-time frame: 32 ones, ST=01, OP, five PHYAD
    bits, five REGAD bits, the two turnaround bits, and sixteen data bits
    (`encode_frame` lays them out). The station queues accesses and shifts them
    out one MDC period at a time, driven from `bus.ns_per_tick`.

    Timing, and how it maps onto the two-phase tick of `verif/vip/core.py`. A bit
    time is `period_ticks` ticks: `low_ticks` with MDC low, `high_ticks` with
    MDC high. Because every device reacts one tick after it sees a change, the
    values the checker measures on the pins are the station's own schedule plus a
    fixed one-tick shift, which changes no interval: MDC high lasts exactly
    `high_ticks` ticks and low exactly `low_ticks`, so the period is exactly
    `period_ticks`. MDIO is changed `setup_ticks` ticks before the MDC rising
    edge that samples it, which leaves `period_ticks - setup_ticks` ticks of hold
    after the previous edge. The two add up to one bit time, which is why the
    station can only trade setup against hold, never have both: a test that
    wants `tSU` to fire sets `setup_ticks=0`, and one that wants `tH` to fire
    sets `setup_ticks=period_ticks` (which also drives MDIO one tick before the
    previous rise). `setup_ticks` defaults to half a bit time, the middle of the
    window the two spec minimums leave open.

    On a read the station releases MDIO for the second turnaround bit and for
    all sixteen data bits and samples them on the rising MDC edges, because the
    PHY owns the bus from there ("no device may actively drive the MDIO signal
    during the first bit of turnaround"; the PHY drives 0 for the second).
    A read that comes back as 0xFFFF is `MdioOp.unanswered`: with the pull-up
    holding MDIO high and nobody driving, no PHY claimed the address.

    Typical use:

        st = attach_station(bus)
        st.write(bus, 0, 0x2100)          # BMCR: 100 Mbit full duplex, autoneg off
        bmsr = st.read(bus, 1)            # BMSR, bit 2 is the link status
        assert not st.errors, st.errors
    """

    name = "mdio_sta"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        period_ticks: Optional[int] = None,
        low_ticks: Optional[int] = None,
        high_ticks: Optional[int] = None,
        setup_ticks: Optional[int] = None,
        frame_bits: Optional[int] = None,
        preamble_bits: int = PREAMBLE_BITS,
        mdc_net: str = "mdc",
        mdio_net: str = "mdio",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.min = mdio_minimums(self.ns_per_tick, self.timing_overrides)
        self.mdc_net, self.mdio_net = mdc_net, mdio_net
        if name:
            self.name = name
        #: Half a bit time low and half high by default: the spec floors
        #: (`t_low`, `t_high`) are floors, not the duty cycle, and 160 ns is 40%
        #: of 400 ns, so an even split is legal at every legal rate.
        self.period_ticks = int(period_ticks if period_ticks is not None
                                else self.min["period_ticks"])
        half = self.period_ticks // 2
        self.low_ticks = int(low_ticks if low_ticks is not None else half)
        self.high_ticks = int(high_ticks if high_ticks is not None
                              else self.period_ticks - self.low_ticks)
        if min(self.low_ticks, self.high_ticks, self.period_ticks) < 1:
            raise ValueError("a bit time needs at least one tick of low, high and period")
        self.setup_ticks = int(setup_ticks if setup_ticks is not None
                               else self.period_ticks // 2)
        #: How many bit times to clock out. Clause 22 is the preamble plus 32
        #: control and data bit times; fewer stops the frame early, which is what
        #: the checker's `frame` rule is there for.
        #: How many 1s to send before ST. Clause 22 is 32, and a shorter one
        #: shifts the whole frame, because a monitor has no other way to find
        #: where the frame began.
        self.preamble_bits = int(preamble_bits)
        self.frame_bits = int(frame_bits if frame_bits is not None
                              else self.preamble_bits + CONTROL_BITS + DATA_BITS)
        self.queue: list[MdioOp] = []
        self.errors: list[str] = []
        self._reset()

    def bind(self, bus: Bus) -> "MdioStation":
        """Take the tick rate from `bus` and recompute the derived timings."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = mdio_minimums(self.ns_per_tick, self.timing_overrides)
        return self

    def _reset(self) -> None:
        self.op: Optional[MdioOp] = None
        self.bits: list = []
        self.bit = 0               # the bit time currently being clocked out
        self.mdio_bit = 0          # bit times whose MDIO value has been sent
        self.t0 = 0                # drive-tick of bit 0's MDC low phase
        self.mdio_at: Optional[int] = None   # next drive-tick that moves MDIO
        self.mdio_drive: Optional[int] = None
        self.prev_mdc = 1
        self.idle_low_since: Optional[int] = None
        self.clocked_elsewhere = False
        self.last_rise: Optional[int] = None
        self.rise_seen = 0

    # ---- what the tests ask for ------------------------------------------------
    @property
    def busy(self) -> bool:
        """True while a frame is being shifted out."""
        return self.op is not None

    @property
    def max_mdc_hz(self) -> float:
        """The rate this station actually clocks MDC at."""
        return 1e9 / (self.ns_per_tick * self.period_ticks)

    @property
    def bit_ns(self) -> float:
        """The bit time in nanoseconds, for a firmware author to compare with."""
        return self.ns_per_tick * self.period_ticks

    def queue_read(self, reg: int, phyad: int = 0) -> MdioOp:
        """Queue a read of `reg` on the PHY at `phyad`; returns the pending result."""
        op = MdioOp(is_write=False, reg=reg, phyad=phyad)
        self.queue.append(op)
        return op

    def queue_write(self, reg: int, value: int, phyad: int = 0) -> MdioOp:
        """Queue a write of `value` to `reg` on the PHY at `phyad`."""
        encode_frame(True, phyad, reg, value)      # range-check now, not mid-frame
        op = MdioOp(is_write=True, reg=reg, value=value & 0xFFFF, phyad=phyad)
        self.queue.append(op)
        return op

    def run(self, bus: Optional[Bus] = None, limit: int = 1_000_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks run out first."""
        if bus is not None and bus is not self.bus:
            self.bind(bus)
        target = bus if bus is not None else self.bus
        if target is None:
            raise RuntimeError("MdioStation.run needs a Bus")
        for _ in range(limit):
            if not self.busy and not self.queue:
                return True
            target.step()
        return not self.busy and not self.queue

    def read(self, bus: Optional[Bus], reg: int, phyad: int = 0,
             limit: int = 1_000_000) -> int:
        """Queue one read, run the bus until it is out, and return the 16-bit value."""
        op = self.queue_read(reg, phyad)
        if not self.run(bus, limit):
            raise RuntimeError(f"the station never finished reading REGAD={reg}")
        return op.value

    def write(self, bus: Optional[Bus], reg: int, value: int, phyad: int = 0,
              limit: int = 1_000_000) -> int:
        """Queue one write, run the bus until it is out, and return the value sent."""
        op = self.queue_write(reg, value, phyad)
        if not self.run(bus, limit):
            raise RuntimeError(f"the station never finished writing REGAD={reg}")
        return op.value

    # ---- the bit engine ----------------------------------------------------
    def _frame_bits(self, op: MdioOp) -> list:
        """The bit times of `op` as this station sends them. Override to patch."""
        return encode_frame(op.is_write, op.phyad, op.reg, op.value or 0,
                            self.preamble_bits)

    def _begin(self, op: MdioOp, t: int) -> None:
        """Start `op` on the drive-tick `t`: MDIO first, then MDC, then bit times."""
        self.op = op
        self.bits = self._frame_bits(op)
        self.rd = 0
        self.rd_bits = 0
        self.ta = None
        # MDIO for bit 0 has to move `setup_ticks` before its rising edge, which
        # is further back than the low phase is long when setup plus hold would
        # not fit in one bit time; MDIO then moves `lead` ticks before MDC does,
        # during the idle gap, which is legal because there is no edge out there.
        self.lead = max(0, self.setup_ticks - self.low_ticks)
        self.t0 = t + self.lead
        self.mdio_bit = 0
        self.mdio_drive = None
        self.last_rise = None
        self.first_rise: Optional[int] = None
        self.idle_low_since: Optional[int] = None
        self.clocked_elsewhere = False
        self.rise_seen = 0
        op.start = None
        op.end = None
        op.periods = []

    def _on_rise(self, t: int, bit: int) -> None:
        """Sample the bit MDIO held when MDC rose, which is the bit just clocked."""
        if self.first_rise is None:
            self.first_rise = t
        else:
            expected = self.first_rise + self.rise_seen * self.period_ticks
            if t != expected and not self.clocked_elsewhere:
                self.clocked_elsewhere = True
                self.errors.append(
                    f"t={t}: MDC rose {t - expected:+d} ticks off the station's "
                    f"schedule: something else is clocking this bus")
        self.rise_seen += 1
        index = self.rise_seen - 1
        if self.last_rise is not None:
            self.op.periods.append(t - self.last_rise)
        else:
            self.op.start = t
        self.last_rise = t
        if index >= self.frame_bits:
            return
        role = bit_role(index)
        if role == "ta2":
            self.ta = bit
        elif role == "data" and not self.op.is_write:
            self.rd = (self.rd << 1) | bit
            self.rd_bits += 1
            if self.rd_bits == DATA_BITS:
                self.op.value = self.rd
        if self.rise_seen == self.frame_bits:
            self.op.end = t
            if not self.op.is_write and self.rd_bits < DATA_BITS:
                self.op.errors.append(
                    f"read ended after {self.rd_bits} of {DATA_BITS} data bits")

    def tick(self, t: int, pins) -> dict:
        mdc, mdio = pins[self.mdc_net], pins[self.mdio_net]
        if self.busy and self.prev_mdc == 0 and mdc == 1:
            self._on_rise(t, mdio)
        elif not self.busy and not self.queue:
            # Only one station per bus: MDC belongs to this device alone, and an
            # idle MDC is the pull-up. MDC that stays low with nothing queued is
            # somebody else clocking, which the off-schedule rise would only
            # notice once a frame started.
            if mdc == 0:
                self.idle_low_since = self.idle_low_since if self.idle_low_since is not None else t
                if (t - self.idle_low_since >= self.min["period_ticks"]
                        and not self.clocked_elsewhere):
                    self.clocked_elsewhere = True
                    self.errors.append(
                        f"t={t}: MDC low for {t - self.idle_low_since} ticks with "
                        f"nothing queued: something else is clocking this bus")
            else:
                self.idle_low_since = None
        self.prev_mdc = mdc
        if not self.busy:
            if not self.queue:
                return {}
            self._begin(self.queue.pop(0), t)
        return self._drive(t)

    def _drive(self, t: int) -> dict:
        """The drives this station issues on drive-tick `t`."""
        # MDIO: one value per bit time, the value for bit `self.mdio_bit`.
        if self.mdio_bit <= self.frame_bits:
            # One MDIO change per bit time, `setup_ticks` ticks before the MDC
            # rise that samples it. Bit 0's change can therefore fall before MDC
            # even starts, which is what `lead` allows for in `_begin`.
            if self.mdio_bit < self.frame_bits:
                due = (self.t0 + self.mdio_bit * self.period_ticks
                       + self.low_ticks - self.setup_ticks)
            else:
                due = self.t0 + self.period_ticks * self.frame_bits
            if t >= due:
                self.mdio_drive = (self.bits[self.mdio_bit]
                                   if self.mdio_bit < self.frame_bits else None)
                self.mdio_bit += 1
        d = t - self.t0
        if d < 0:
            # Still inside the lead: MDIO has moved to bit 0, MDC has not started.
            return {self.mdc_net: None, self.mdio_net: self.mdio_drive}
        if d >= self.period_ticks * self.frame_bits:
            # The frame is over: release both lines and go idle.
            if self.rise_seen < self.frame_bits:
                self.op.errors.append(
                    f"only {self.rise_seen} of {self.frame_bits} bit times clocked")
            self.op = None
            self.mdio_drive = None
            self.mdio_bit = self.frame_bits + 1
            return {}
        return {self.mdc_net: 0 if d % self.period_ticks < self.low_ticks else 1,
                self.mdio_net: self.mdio_drive}


class MdioPhy(Device):
    """Clause 22 PHY: a 32 x 16-bit register file behind the turnaround.

    It watches MDC rising edges and counts the bit times, exactly as the station
    and the checker do, so `bit_role` gives all three the same frame. It answers
    only a read whose PHYAD is its own: for the second turnaround bit it drives 0
    and for the sixteen data bits it drives the register value, starting
    `drive_delay_ticks` after the rising edge that begins the bit time (1 tick by
    default, so its data is valid 20 ns after the rise, well inside the 300 ns
    `tval` deadline; a test can raise it past the deadline to break `tPHY`).
    Everything else it leaves released: a write, a read addressed elsewhere, and
    the first turnaround bit of a read all read as 1 on the wire.

    The register file is the IEEE one: BMCR (0) with its self-clearing reset,
    BMSR (1) whose link bit latches low until read, the two PHY ID words, and the
    auto-negotiation registers, all from `lan8720a-clause22-registers.txt`.
    Registers 7..17 are plain storage, because no saved source names them.

    Setting `link` is how a test makes the link go down: the falling edge latches
    BMSR bit 2 low, so a station that polls once and misses the outage sees 0 on
    the next poll, and reading BMSR clears the latch. A write of BMCR bit 15
    (`soft_reset`) restores the power-on contents and re-arms the post-reset
    window, during which the PHY drives nothing at all and every read comes back
    0xFFFF -- the DP83848 asks for 3 us and a 32-bit preamble after a reset
    (`ti-dp83848-smc-timing.txt`). The power-up window of the same table is not
    modelled: a fresh `MdioPhy` starts with its window already elapsed, because
    nothing in a test powers a PHY up mid-run.
    """

    name = "mdio_phy"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        phyad: int = 0,
        regs: Optional[dict] = None,
        link: bool = True,
        timing: Optional[dict] = None,
        drive_delay_ticks: Optional[int] = None,
        init_ticks: Optional[int] = None,
        mdc_net: str = "mdc",
        mdio_net: str = "mdio",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.min = mdio_minimums(self.ns_per_tick, self.timing_overrides)
        self.mdc_net, self.mdio_net = mdc_net, mdio_net
        if name:
            self.name = name
        if not 0 <= phyad < (1 << ADDRESS_BITS):
            raise ValueError(f"PHYAD {phyad!r} does not fit in {ADDRESS_BITS} bits")
        self.phyad = phyad
        self.regs: list[int] = [0] * 32
        for reg, value in POWER_ON_DEFAULTS.items():
            self.regs[reg] = value
        for reg, value in (regs or {}).items():
            if not 0 <= reg < len(self.regs):
                raise ValueError(f"register {reg} is outside the 32-word file")
            self.regs[reg] = value & 0xFFFF
        #: The PHY's read-data latency in ticks after the rising MDC edge.
        self.drive_delay_ticks = int(
            drive_delay_ticks if drive_delay_ticks is not None else 1)
        #: Ticks the PHY stays silent after a soft reset (3 us at 50 MHz = 150).
        self.init_ticks = int(init_ticks if init_ticks is not None
                              else ns_to_ticks(self.ns_per_tick, 3_000))
        self._link = bool(link)
        self.link_lost = False
        self.errors: list[str] = []
        self.accesses: list = []
        self.resets = 0
        self.frames = 0
        self._reset_frame()

    def bind(self, bus: Bus) -> "MdioPhy":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = mdio_minimums(self.ns_per_tick, self.timing_overrides)
        return self

    def _reset_frame(self) -> None:
        self.prev_mdc = 1
        self.in_frame = False
        self.idle_high = 0
        self.prev_rise: Optional[int] = None
        self.low_run = 0
        self.observed_period: Optional[int] = None
        self.nbits = 0
        self.bits: list = [0] * FRAME_BITS
        self.st = 0
        self.op = 0
        self.phyad_seen = 0
        self.reg = 0
        self.wr = 0
        self.read_value = 0
        self.silent = False
        self.overlong = False
        self.drive_at: Optional[int] = None
        self.drive_value: Optional[int] = None
        self.busy_until = 0

    # ---- the register file --------------------------------------------------
    @property
    def link(self) -> bool:
        """Whether the link is up. Assigning False latches BMSR bit 2 low."""
        return self._link

    @link.setter
    def link(self, value) -> None:
        value = bool(value)
        if self._link and not value:
            self.link_lost = True      # BMSR bit 2 is RO/LL: latches 0 until read
        self._link = value

    def read_reg(self, reg: int) -> int:
        """The value a read of `reg` returns, with the BMSR link latch applied."""
        if reg == BMSR:
            value = self.regs[BMSR] & ~BMSR_LINK
            if self._link and not self.link_lost:
                value |= BMSR_LINK
            self.link_lost = False    # reading BMSR releases the latch
            return value
        return self.regs[reg] & 0xFFFF

    def write_reg(self, reg: int, value: int, t: Optional[int] = None) -> None:
        """Apply a write to `reg`; BMCR bit 15 resets the whole file instead."""
        value &= 0xFFFF
        if reg == BMCR and value & BMCR_RESET:
            self.soft_reset(t)
            return
        if reg == BMSR:
            return      # every BMSR bit is RO, RO/LH or RO/LL in the datasheet
        self.regs[reg] = value

    def soft_reset(self, t: Optional[int] = None) -> None:
        """Restore the power-on register contents and re-arm the silent window."""
        self.resets += 1
        for reg, value in POWER_ON_DEFAULTS.items():
            self.regs[reg] = value
        self.link_lost = False
        if t is not None:
            self.busy_until = t + self.init_ticks

    def dump(self) -> dict:
        """The register file as {register index: value}, for a test to look at."""
        return {i: self.read_reg(i) for i in range(len(self.regs))}

    # ---- the bus side -------------------------------------------------------
    def quiet_ticks(self) -> int:
        """Ticks MDC has to stay put before this PHY calls the frame over.

        Three times the spec's shortest bit time and never less than twice the bit
        time measured so far, so a station slower than 2.5 MHz does not look like
        it stopped clocking half way through a frame.
        """
        return max(3 * self.min["period_ticks"], 2 * (self.observed_period or 0))

    def _start_frame(self, t: int) -> None:
        """A frame begins at the rising MDC edge that follows a quiet bus."""
        self.in_frame = True
        self.nbits = 0
        self.idle_high = 0
        self.bits = [0] * FRAME_BITS
        self.st = self.op = self.phyad_seen = self.reg = self.wr = 0
        self.read_value = 0
        self.overlong = False
        self.silent = t < self.busy_until
        self.drive_at = self.drive_value = None
        if self.silent:
            self.errors.append(f"t={t}: frame arrived during the post-reset window")

    def _drive_for(self, t: int, index: int) -> Optional[int]:
        """What this PHY must drive during bit time `index`, or None to release.

Only a read of this PHY's own address is answered: the second turnaround bit
        becomes 0 and the data bits carry the register. The decision is reached
        from the fields decoded so far, which are complete by the time the
        turnaround is armed, so no "am I answering?" flag is needed. `silent` is
        the state latched when the frame started: a frame that arrived inside the
        post-reset window is ignored from end to end, rather than half-answered
        once the window ran out in the middle of it.
        """
        if index >= FRAME_BITS or self.silent:
            return None
        if self.op != OP_READ or self.phyad_seen != self.phyad:
            return None
        if index == FRAME_BITS - DATA_BITS - 1:          # the ta2 bit
            return 0
        if index >= FRAME_BITS - DATA_BITS:             # the 16 data bits, MSB first
            return (self.read_value >> (FRAME_BITS - 1 - index)) & 1
        return None

    def _arm(self, t: int, index: int) -> None:
        """Present bit `index`'s value `drive_delay_ticks` after the edge at `t`.

        The arm happens once per bit time and once more for the bit time past the
        frame, where `_drive_for` says whether to release ("then releases the MDIO
        line") or, for a subclass that gets it wrong, to keep holding the bus.
        """
        self.drive_at = t + self.drive_delay_ticks
        self.drive_value = self._drive_for(t, index)

    def _on_rise(self, t: int, bit: int) -> None:
        index = self.nbits
        if self.prev_rise is not None and index:
            self.observed_period = max(self.observed_period or 0, t - self.prev_rise)
        self.prev_rise = t
        if index >= FRAME_BITS:
            if not self.overlong:
                self.errors.append(f"t={t}: MDC kept clocking after "
                                   f"{FRAME_BITS} bit times")
                self.overlong = True
            self.nbits = index + 1
            return
        self.bits[index] = bit
        role = bit_role(index)
        if role == "st":
            self.st = (self.st << 1) | bit
        elif role == "op":
            self.op = (self.op << 1) | bit
        elif role == "phyad":
            self.phyad_seen = (self.phyad_seen << 1) | bit
        elif role == "regad":
            self.reg = (self.reg << 1) | bit
        elif role == "ta2":
            self._on_turnaround(t, bit)
        elif role == "data" and self.op == OP_WRITE:
            self.wr = (self.wr << 1) | bit
        self.nbits = index + 1
        self._arm(t, index + 1)
        if self.nbits == FRAME_BITS:
            self._end_frame(t)

    def _on_turnaround(self, t: int, bit: int) -> None:
        """The second turnaround bit: the last chance to latch a read's value."""
        if self.silent:
            return
        if self.op == OP_READ and self.phyad_seen == self.phyad:
            if bit != 0:
                self.errors.append(
                    f"t={t}: turnaround read {bit}, this PHY drives 0 there")
            self.read_value = self.read_reg(self.reg)

    def _end_frame(self, t: int) -> None:
        """The frame's 64th bit time is over: check it and apply a write."""
        self.frames += 1
        self.in_frame = False
        if self.st != ST_BITS:
            self.errors.append(f"t={t}: start code {self.st:02b}, clause 22 is 01")
        if self.op not in (OP_READ, OP_WRITE):
            self.errors.append(f"t={t}: opcode {self.op:02b} is neither read nor write")
            return
        addressed = self.phyad_seen == self.phyad
        if not addressed:
            return
        verb = "write" if self.op == OP_WRITE else "read"
        self.accesses.append((t, verb, self.reg, self.wr if self.op == OP_WRITE
                              else self.read_value))
        if self.op == OP_WRITE:
            self.write_reg(self.reg, self.wr, t)

    def tick(self, t: int, pins) -> dict:
        mdc, mdio = pins[self.mdc_net], pins[self.mdio_net]
        # MDC high for a whole bit time means no bit time is being clocked, so
        # the frame that was in progress is over -- finished, or abandoned.
        if mdc == 0:
            self.low_run += 1
        elif self.low_run:
            self.observed_period = max(self.observed_period or 0, 2 * self.low_run)
            self.low_run = 0
        self.idle_high = self.idle_high + 1 if mdc == 1 else 0
        if self.in_frame and self.idle_high == self.quiet_ticks():
            if self.nbits < FRAME_BITS:
                self.errors.append(f"t={t}: frame abandoned after {self.nbits} of "
                                   f"{FRAME_BITS} bit times")
            self.in_frame = False
        if self.prev_mdc == 0 and mdc == 1:
            if not self.in_frame:
                self._start_frame(t)
            self._on_rise(t, mdio)
        self.prev_mdc = mdc
        # The armed value is held until the next `_arm`: a driver owns the line
        # for the whole bit time, which is what "stable across the MDC rising
        # edge" means on the wire.
        if self.drive_at is not None and t >= self.drive_at and t >= self.busy_until:
            return {self.mdio_net: self.drive_value}
        return {}


# ---- the checker -----------------------------------------------------------
@dataclass(frozen=True)
class Violation:
    """One spec rule the bus did not meet, with the tick it was missed on."""

    tick: int
    rule: str            # "MDC", "MDC-half", "tSU", "tH", "tPHY", "preamble", ...
    measured: int        # ticks the bus gave it
    required: int        # ticks the spec asks for
    detail: str = ""
    kind: str = "min"    # "min": measured must be >= required; "max": <=

    def __str__(self) -> str:
        sign = "<" if self.kind == "min" else ">"
        return (f"t={self.tick} {self.rule} {self.measured} ticks {sign} {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


def _station_owns(index: int, is_write: Optional[bool]) -> Optional[bool]:
    """Whether the station, not a PHY, is the driver during bit time `index`.

    True for the fields the station always sends, and for a write's turnaround
    and data. False for the parts of a read the PHY owns (the second turnaround
    bit and the data), and None for the first turnaround bit of a read, where the
    rule is that *nobody* drives. `None` is what keeps the checker's setup and
    hold checks off the PHY's read data, which is free to move as soon after the
    rising edge as it likes (`tohold` 0 ns min).
    """
    role = bit_role(index)
    if role in ("preamble", "st", "op", "phyad", "regad"):
        return True
    if role == "ta1":
        return None if is_write is False else True   # nobody drives this one
    if role == "ta2":
        return False if is_write is False else True
    if role == "data":
        return False if is_write is False else True
    return False


class MdioChecker(Device):
    """Passive MDIO monitor: decodes the wire and flags every rule it breaks.

    It drives nothing, so it sees the bus exactly as an analyser would, sampling
    MDIO when MDC rises because the pins it is handed are the resolved values.
    Every miss goes into `violations` with the tick it happened on, and every
    decoded access into `frames`, so a test can assert either on the rule names
    or on what was read and written.

    The timing rules, and what each one measures:

    `MDC`       the MDC period, floor `t_period` (400 ns), i.e. the fMDC ceiling of
                2.5 MHz. Measured between consecutive rising edges inside a frame.
    `MDC-half`  MDC low time (floor `t_low`) and MDC high time (floor `t_high`),
                both 160 ns: the spec's "minimum time between edges of the MDC".
    `tSU`       an MDIO change that the station made less than `t_setup` (10 ns)
                ticks before the MDC rising edge that sampled it.
    `tH`        an MDIO change the station made less than `t_hold` (10 ns) ticks
                after an MDC rising edge.
    `tPHY`      an MDIO change in a read's data field more than `t_phy_max`
                (300 ns) ticks after the rising edge that started that bit, i.e.
                read data that missed its deadline.

    `tSU` and `tH` only apply where the station is the driver (`_station_owns`):
    in a read's turnaround and data the PHY owns the line, and the only hard edge
    there is the 300 ns deadline of `tPHY`. Two more limits are framing rules
    rather than measurements: `release` wants MDIO back at the pull-up value 1
    within one bit time of the last data bit, and `idle` wants a bus that has
    stopped clocking to leave both nets at 1.

    The framing rules are `preamble` (32 ones before ST after an idle gap),
    `ST` (01), `OP` (10 read or 01 write), `TA1` (the first turnaround bit must
    read 1: the station drives it on a write, nobody may drive it on a read),
    `TA2` (the second must read 0: the PHY drives it on a read, the station
    drives it on a write), `frame` (MDC stopped clocking before the frame's 64
    bit times were out) and `contention` (two devices drove the same net with
    opposite values, read straight off `Bus.errors`).

    `expect_phyad` is how the checker knows which addresses a PHY answers. A read
    whose turnaround comes back as ones is the clause 22 "no PHY there" answer
    (the pull-up holding MDIO high), so without `expect_phyad` it is recorded as
    `MdioFrame.unanswered` rather than flagged; with it, `TA2` fires if the
    addressed PHY did not drive its 0. Two frames back to back with no dead time
    between them may skip the preamble, which is the one place this checker is
    lenient, and `prev_frame_end`/`idle_ticks` say what it decided.
    """

    name = "mdio_check"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        expect_phyad: Optional[object] = None,
        mdc_net: str = "mdc",
        mdio_net: str = "mdio",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.min = mdio_minimums(self.ns_per_tick, self.timing_overrides)
        self.mdc_net, self.mdio_net = mdc_net, mdio_net
        if name:
            self.name = name
        if expect_phyad is None:
            self.expect_phyad: Optional[set] = None
        elif isinstance(expect_phyad, int):
            self.expect_phyad = {expect_phyad}
        else:
            self.expect_phyad = set(expect_phyad)
        self.violations: list[Violation] = []
        self.frames: list[MdioFrame] = []
        self.mdc_periods: list[int] = []
        self._seen_errors = 0
        self._reset()

    def bind(self, bus: Bus) -> "MdioChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = mdio_minimums(self.ns_per_tick, self.timing_overrides)
        self._seen_errors = len(bus.errors)
        return self

    def _reset(self) -> None:
        self.prev_mdc = self.prev_mdio = IDLE
        self.nbits = 0
        self.in_frame = False
        self.aborted = False
        self.idle_low_ticks = 0
        self.idle_high_ticks = 0
        self.reported_idle = False
        self.reported_release = False
        self.bits = [IDLE] * FRAME_BITS
        self.preamble_ones = 0
        self.st = self.op = self.phyad = self.reg = 0
        self.ta = 0
        self.data = 0
        self.data_bits = 0
        self.frame_start = 0
        self.frame_periods: list[int] = []
        self.prev_rise: Optional[int] = None
        self.prev_fall: Optional[int] = None
        self.mdio_change: Optional[int] = None
        self.prev_frame_end: Optional[int] = None
        self.idle_ticks = 0
        self.shortest_period: Optional[int] = None
        self.observed_period: Optional[int] = None
        self.preamble_required = True

    # ---- what the tests ask for ------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def max_mdc_hz(self) -> Optional[float]:
        """The fastest MDC the bus ran at, from the shortest period measured."""
        if self.shortest_period is None:
            return None
        return 1e9 / (self.ns_per_tick * self.shortest_period)

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 10) -> str:
        head = (f"MDIO at {self.ns_per_tick} ns/tick: {len(self.frames)} frames "
                f"({sum(1 for f in self.frames if f.is_write)} writes, "
                f"{sum(1 for f in self.frames if f.unanswered)} unanswered), "
                f"{len(self.violations)} violations")
        if self.shortest_period is not None:
            head += (f", shortest MDC period {self.shortest_period} ticks "
                     f"({self.max_mdc_hz / 1e6:.3f} MHz, ceiling "
                     f"{self.min['max_mdc_hz'] / 1e6:.3f} MHz)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   detail: str = "") -> None:
        if measured < required:
            self.violations.append(Violation(t, rule, measured, required, detail))

    def _late(self, t: int, rule: str, measured: int, limit: int,
              detail: str = "") -> None:
        """Record a rule that is a ceiling rather than a floor."""
        if measured > limit:
            self.violations.append(Violation(t, rule, measured, limit, detail, "max"))

    def _contention(self) -> None:
        """Turn the core's push-pull contention records into `contention` violations."""
        if self.bus is None:
            return
        for err in self.bus.errors[self._seen_errors:]:
            if err.net in (self.mdc_net, self.mdio_net):
                self.violations.append(Violation(
                    err.t, "contention", 0, 0,
                    f"{err.net} driven {err.drivers} with opposite values"))
        self._seen_errors = len(self.bus.errors)

    def _begin_frame(self, t: int) -> None:
        """The rise that starts a frame: measure its gap to the previous one."""
        self.in_frame = True
        self.nbits = 0
        self.aborted = False
        self.bits = [IDLE] * FRAME_BITS
        self.preamble_ones = 0
        self.st = self.op = self.phyad = self.reg = self.ta = 0
        self.data = 0
        self.data_bits = 0
        self.frame_start = t
        self.frame_periods = []
        self.idle_ticks = 0
        self.idle_high_ticks = 0
        # Two frames back to back, with the next bit time starting where the last
        # one ended, may skip the preamble; anything with dead time in front of
        # it needs the full 32 ones (see the README's judgement calls).
        gap = None if self.prev_frame_end is None else t - self.prev_frame_end
        self.preamble_required = not (
            gap is not None and gap <= self.min["period_ticks"] + 1)

    def _mdc_edge(self, t: int, mdc: int, mdio: int, moved: bool) -> None:
        if mdc == 0:
            if self.prev_rise is not None:
                self._violation(t, "MDC-half", t - self.prev_rise,
                                self.min["high_ticks"], "MDC high time")
            if self.prev_fall is not None:
                # An MDC low phase is half a bit time or more, so it is a lower
                # bound on this bus's bit time even before a whole period is out.
                self.observed_period = max(self.observed_period or 0,
                                           2 * (t - self.prev_fall))
            self.prev_fall = t
            return
        if self.prev_fall is not None:
            self._violation(t, "MDC-half", t - self.prev_fall,
                            self.min["low_ticks"], "MDC low time")
        if self.in_frame and self.prev_rise is not None:
            period = t - self.prev_rise
            self.mdc_periods.append(period)
            self.frame_periods.append(period)
            self.observed_period = max(self.observed_period or 0, period)
            self.shortest_period = (period if self.shortest_period is None
                                    else min(self.shortest_period, period))
            self._violation(
                t, "MDC", period, self.min["period_ticks"],
                "MDC period is %.3f MHz, the ceiling is %.3f MHz"
                % (1e9 / (self.ns_per_tick * period),
                   self.min["max_mdc_hz"] / 1e6))
        self.prev_rise = t
        if not self.in_frame:
            self._begin_frame(t)
        self._sample(t, mdio, moved)

    def _sample(self, t: int, mdio: int, moved: bool = False) -> None:
        """MDIO is read on the rising MDC edge; check the setup of the bit it ends.

        `moved` says MDIO also changed on this very tick. A change that lands on
        the edge that samples it has no setup for the new bit and no hold for the
        old one, so both rules fire, which is what `setup_ticks=0` produces.
        """
        index = self.nbits
        if index >= FRAME_BITS:
            return
        self.bits[index] = mdio
        is_write = True if self.op == OP_WRITE else (
            False if self.op == OP_READ else None)
        owner = _station_owns(index, is_write)
        if owner is True:
            since = t if moved else self.mdio_change
            if since is not None:
                self._violation(t, "tSU", t - since, self.min["t_setup"],
                                "MDIO moved this close to the MDC rise")
        role = bit_role(index)
        if role == "preamble":
            self.preamble_ones = self.preamble_ones + 1 if mdio == IDLE else 0
        elif role == "st":
            self.st = (self.st << 1) | mdio
        elif role == "op":
            self.op = (self.op << 1) | mdio
        elif role == "phyad":
            self.phyad = (self.phyad << 1) | mdio
        elif role == "regad":
            self.reg = (self.reg << 1) | mdio
        elif role == "ta1":
            self.ta = mdio << 1
            if mdio != IDLE:
                self.violations.append(Violation(
                    t, "TA1", mdio, IDLE,
                    "the first turnaround bit must read 1: the station drives it "
                    "on a write and nobody may drive it on a read"))
        elif role == "ta2":
            self.ta |= mdio
            self._check_ta2(t, mdio, is_write)
        elif role == "data":
            self.data = (self.data << 1) | mdio
            self.data_bits += 1
        if index == PREAMBLE_BITS:
            self._check_preamble(t)
        if index == ST_END:
            if self.st != ST_BITS:
                self.violations.append(Violation(
                    t, "ST", self.st, ST_BITS,
                    f"start code read {self.st:02b}, clause 22 is 01"))
        if index == OP_END:
            if self.op not in (OP_READ, OP_WRITE):
                self.violations.append(Violation(
                    t, "OP", self.op, 0,
                    f"opcode read {self.op:02b}, clause 22 is 10 read or 01 write"))
        self.nbits = index + 1
        if index == FRAME_BITS - 1:
            self._end_frame(t, True)

    def _check_preamble(self, t: int) -> None:
        """The 32 bit times before ST must be 32 ones unless the frame is contiguous."""
        if self.preamble_required and self.preamble_ones < PREAMBLE_BITS:
            self.violations.append(Violation(
                t, "preamble", self.preamble_ones, PREAMBLE_BITS,
                "only %d of %d ones before ST (a frame with dead time in front "
                "of it needs the full preamble)" % (self.preamble_ones,
                                                    PREAMBLE_BITS)))

    def _check_ta2(self, t: int, mdio: int, is_write: Optional[bool]) -> None:
        """The second turnaround bit must read 0, whoever is supposed to drive it."""
        if is_write is False:
            if self.expect_phyad is not None and self.phyad not in self.expect_phyad:
                return          # no PHY claimed it: the ones are the answer
            who = "the PHY at PHYAD %d" % self.phyad
        elif is_write is True:
            who = "the station"
        else:
            return
        if mdio != 0:
            self.violations.append(Violation(
                t, "TA2", mdio, 0,
                f"the second turnaround bit read {mdio}; {who} must drive 0"))

    def _end_frame(self, t: int, complete: bool) -> None:
        """Record a decoded (or truncated) frame and wait for the next one."""
        is_write = True if self.op == OP_WRITE else (
            False if self.op == OP_READ else None)
        data = self.data & 0xFFFF if self.data_bits == DATA_BITS else None
        frame = MdioFrame(
            start=self.frame_start, is_write=is_write, phyad=self.phyad, reg=self.reg,
            data=data, ta=self.ta, st_bits=self.st, op_bits=self.op,
            preamble_bits=self.preamble_ones,
            unanswered=(is_write is False and self.ta == 0b11),
            mdc_periods=tuple(self.frame_periods),
            error="" if complete else
            f"MDC stopped clocking after {self.nbits} of {FRAME_BITS} bit times")
        self.frames.append(frame)
        if not complete:
            self.violations.append(Violation(
                t, "frame", self.nbits, FRAME_BITS, frame.error))
        self.prev_frame_end = t
        self.in_frame = False
        self.nbits = 0
        self.frame_periods = []
        self.reported_idle = self.reported_release = False

    def _mdio_change(self, t: int, mdio: int) -> None:
        """MDIO moved: check it against the edge whose window it landed in."""
        if self.in_frame:
            owner = _station_owns(self.nbits, self.op == OP_WRITE)
            if owner is True and self.prev_rise is not None:
                self._violation(t, "tH", t - self.prev_rise, self.min["t_hold"],
                                "MDIO moved this soon after the MDC rise")
            elif owner is False and self.prev_rise is not None:
                self._late(t, "tPHY", t - self.prev_rise, self.min["t_phy_max"],
                           "read data valid only this long after the MDC rise")
        self.mdio_change = t

    def quiet_ticks(self) -> int:
        """How long MDC has to stay put before the bus counts as idle.

        Three times the spec's shortest bit time, and never less than twice the
        bit time this bus has actually been running. A frame's own MDC high phase
        is half a bit time, so that much quiet cannot happen inside a frame, while
        a station slower than 2.5 MHz has a long high phase that must not be
        mistaken for the end of the frame. The bus's own rate is learned from the
        periods measured so far (and from the MDC low phases, which give a lower
        bound on the bit time before the first period has been measured), so the
        checker works at any legal rate without being told the rate.
        """
        return max(3 * self.min["period_ticks"], 2 * (self.observed_period or 0))

    def _idle(self, t: int, mdc: int, mdio: int) -> None:
        """Count how long MDC has been still, which is what ends a frame.

        MDC high for a whole bit time means no bit time is being clocked, so a
        frame is over: complete after its 64th bit time, or truncated, which is
        the `frame` rule. Between frames the same quiet bus has to leave both
        nets at 1, which is `idle` (MDC parked low) and `release` (MDIO left
        low with nobody driving).
        """
        if mdc == 0:
            self.idle_low_ticks += 1
            self.idle_high_ticks = 0
        else:
            self.idle_low_ticks = 0
            self.idle_high_ticks += 1
        quiet = self.quiet_ticks()
        if self.in_frame and self.idle_high_ticks == quiet and 0 < self.nbits < FRAME_BITS:
            self._end_frame(t, False)
            self.aborted = True
        # A parked MDC is wrong whatever the checker thinks it is decoding: half
        # a bit time of MDC low is normal inside a frame, three bit times is not.
        if self.idle_low_ticks == quiet and not self.reported_idle:
            self.reported_idle = True
            self.violations.append(Violation(
                t, "idle", self.idle_low_ticks, quiet,
                "MDC stayed low with no bit times being clocked"))
        if self.in_frame:
            return
        if self.prev_frame_end is not None:
            self.idle_ticks = t - self.prev_frame_end
        if (self.nbits == 0 and mdio != IDLE and not self.reported_release
                and self.idle_high_ticks >= quiet):
            self.reported_release = True
            self.violations.append(Violation(
                t, "release", 0, IDLE,
                "MDIO stayed low with the bus idle; the pull-up owns the line "
                "once every driver has released it"))

    def tick(self, t: int, pins) -> dict:
        mdc, mdio = pins[self.mdc_net], pins[self.mdio_net]
        prev_mdc, prev_mdio = self.prev_mdc, self.prev_mdio
        self.prev_mdc, self.prev_mdio = mdc, mdio
        self._contention()
        if mdc != prev_mdc:
            self._mdc_edge(t, mdc, mdio, mdio != prev_mdio)
        if mdio != prev_mdio:
            self._mdio_change(t, mdio)
        self._idle(t, mdc, mdio)
        return {}


# ---- harness helpers --------------------------------------------------------
def _ensure_nets(bus: Bus) -> None:
    """Add the two MDIO nets if they are not there yet.

    MDIO is `push_pull=True` so that two devices driving opposite values in the
    same tick is recorded as contention, which is how a station that drives the
    wire while the PHY is still driving the turnaround gets caught. Both nets
    pull up to 1, which is the idle state of the bus and what makes a released
    MDIO read 1 (`ti-dp83tc811-smi.txt`: "MDIO pin requires a pullup resistor
    ... which pulls MDIO high during IDLE and turnaround").
    """
    for net in ("mdc", "mdio"):
        if net not in bus.nets:
            bus.add_net(net, push_pull=True, pullup=IDLE)


def attach_station(bus: Bus, **kwargs) -> MdioStation:
    """Build an MdioStation that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    st = MdioStation(bus=bus, **kwargs)
    bus.attach(st)
    return st


def attach_phy(bus: Bus, **kwargs) -> MdioPhy:
    """Build an MdioPhy that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    phy = MdioPhy(bus=bus, **kwargs)
    bus.attach(phy)
    return phy


def attach_checker(bus: Bus, **kwargs) -> MdioChecker:
    """Build a passive MdioChecker for `bus` and attach it."""
    _ensure_nets(bus)
    chk = MdioChecker(bus=bus, **kwargs)
    chk._seen_errors = len(bus.errors)
    bus.attach(chk)
    return chk
