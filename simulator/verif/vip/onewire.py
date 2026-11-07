"""Pin-level 1-Wire reference models on the tick-based bus in verif/vip/core.py.

One net, named "dq" by default, open drain: every device returns 0 to pull it low
or None to release it, never 1, and every test asserts `bus.errors == []`. The
line is idle high because `Bus.add_net` gives it a pull-up of 1.

Three devices:

  OneWireMaster       bit-level master for queued transactions, standard or
                      overdrive speed: reset + presence, write slots (LSB
                      first), read slots, and the ROM/function command helpers
                      (read_rom, match_rom, skip_rom, search_rom, convert_t,
                      read_scratchpad, write_scratchpad) built on those.
  Ds18b20Target       DS18B20-like slave: 64-bit ROM (family 28h, 48-bit serial,
                      CRC-8), a 9-byte scratchpad (temperature, TH, TL,
                      configuration, reserved, CRC), the five ROM commands, the
                      six function commands, and the presence pulse.
  OneWireTimingChecker  passive monitor: it drives nothing, decodes resets,
                      presence pulses, time slots and bytes off the pins, checks
                      every timing rule of the datasheet timing tables below, and
                      flags every framing error (missing presence, an unknown ROM
                      command, a slot before any reset, a slave driving inside a
                      write slot, a CRC mismatch) with the tick it happened on.

Typical use:

    bus = Bus(ns_per_tick=20)                       # 50 MHz
    mst = attach_master(bus)
    ds = attach_ds18b20(bus, serial=b"\\x01\\x02\\x03\\x04\\x05\\x06")
    chk = attach_checker(bus)                        # optional, passive

    mst.queue_convert_t()                           # skip ROM + Convert T
    mst.run(bus, limit=2_000_000)
    op = mst.queue_read_scratchpad()                # skip ROM + Read Scratchpad
    mst.run(bus, limit=2_000_000)
    chk.finish()                                    # flush end-of-traffic rules
    assert op.present and bytes(op.data) == bytes(ds.scratchpad)
    assert chk.violations == [], chk.report()

Every microsecond in this file comes from one of two saved sources, quoted with
their line numbers in `research/ref/protocols/onewire/` (and the judgement calls
are listed in the README next to them):

  DS18B20 datasheet (AC electrical characteristics, read/write time slots)
    tSLOT 60..120 us   tREC 1 us     tLOW0 60..120 us   tLOW1 1..15 us
    tRDV 15 us         tRSTH 480 us  tRSTL 480 us        tPDHIGH 15..60 us
    tPDLOW 60..240 us, tCONV 750 ms at 12 bits
  AN126 Table 2, the master timing table, as min/recommended/max
    A 5/6/15   and 1/1.5/1.85      B 59/64/-   and 7.5/7.5/-
    C 60/60/120 and 7/7.5/14        D 8/10/-   and 2.5/2.5/-
    E 5/9/12   and 0.5/0.75/0.85    F 50/55/-  and 6.75/7/-
    G 0/0/0    and 2.5/2.5/-        H 480/480/640 and 68/70/80
    I 63/70/78 and 7.2/8.5/8.8      J 410/410/- and 39.5/40/-

`RULES` lists every violation the checker can report, and `test_onewire.py` breaks
each one on purpose. `finish()` has to be called when the traffic stops: it judges
what the last edge could not (a reset still waiting for its presence pulse, a byte
half-way through).

One warning before you read the numbers: 1-Wire is slow and this bus counts in
ticks. At 20 ns/tick one time slot is 3000 to 6000 ticks and one byte is 8 of
them, so a full "reset, read 9 scratchpad bytes" transaction is about 410000
ticks and takes most of a second of wall clock in this simulator. Run long
protocol tests on a coarser bus (`Bus(ns_per_tick=100)`, 10 MHz) and keep 20 ns
for the tests that measure timing precisely; every timing in the model and the
checker is derived from `ns_per_tick`, so nothing is hard-wired to one rate. At
100 ns/tick every standard-speed limit is still at least 10 ticks.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Optional

from .core import Bus, Device

# ---- protocol constants, all from the DS18B20 datasheet -------------------------
# "There are five ROM commands, and each command is 8 bits long. The master device
#  must issue an appropriate ROM command before issuing a DS18B20 function
#  command."  (research/ref/protocols/onewire/ds18b20-commands.txt)
ROM_COMMANDS = {0xF0: "Search ROM", 0x33: "Read ROM", 0x55: "Match ROM",
                0xCC: "Skip ROM", 0xEC: "Alarm Search"}
# DS18B20 Function Command Set, Table 4 (same file)
FUNCTION_COMMANDS = {0x44: "Convert T", 0xBE: "Read Scratchpad",
                     0x4E: "Write Scratchpad", 0x48: "Copy Scratchpad",
                     0xB8: "Recall E2", 0xB4: "Read Power Supply"}
FAMILY_DS18B20 = 0x28             # "the DS18B20's 1-Wire family code: 28h"

DS18B20_POWERUP_TEMP = 0x0550      # "* The power-on reset value of the temperature
                                   #   register is +85C." (85 << 4 = 0x550)
DS18B20_RESERVED_05 = (0xFF, 0xFF, 0x10)   # "Byte 5 Reserved (FFh) ... Byte 7
                                           #  Reserved (10h)"


def crc8(data, crc: int = 0) -> int:
    """The 1-Wire CRC-8: x^8 + x^5 + x^4 + 1, MSB first, seed 0, no reflection.

    "The equivalent polynomial function of the CRC (ROM or scratchpad) is:
    CRC = X 8 + X 5 + X 4 + 1 ... the shift register bits are initialized to 0."
    """
    for byte in data:
        for bit in range(8):
            mix = (crc ^ byte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            byte >>= 1
    return crc & 0xFF


def rom_from_serial(serial, family: int = FAMILY_DS18B20) -> bytes:
    """The 8 ROM bytes in wire order: family, 6 serial bytes, CRC.

    "The least significant 8 bits of the ROM code contain the DS18B20's 1-Wire
    family code: 28h. The next 48 bits contain a unique serial number. The most
    significant 8 bits contain a cyclic redundancy check (CRC) byte that is
    calculated from the first 56 bits of the ROM code."
    """
    serial = bytes(serial)
    if len(serial) != 6:
        raise ValueError(f"a 1-Wire serial number is 6 bytes, got {len(serial)}")
    rom = bytes([family]) + serial
    return rom + bytes([crc8(rom)])


def rom_from_id(device_id: int, family: int = FAMILY_DS18B20) -> bytes:
    """A ROM code from a small integer, so tests can say `rom_from_id(1)`."""
    return rom_from_serial(device_id.to_bytes(6, "little"), family)
# ---- the timing tables, in nanoseconds ------------------------------------------
@dataclass(frozen=True)
class OneWireSpeeds:
    """One 1-Wire speed class: the limits a checker enforces and the waveform a
    master builds, both in nanoseconds. `None` means "the spec does not give this
    one", and the checker then does not check it -- see the README next to these
    numbers for which source each is from and why a few are missing.

    The `t_*` fields are limits (a min or a (min, max) pair), `w_*` fields are
    what `OneWireMaster` actually drives: the recommended column of AN126 Table 2
    where AN126 has one, chosen to sit in the middle of the datasheet window
    otherwise.
    """

    name: str
    # ---- limits the checker enforces ----
    t_rstl: tuple          # master reset low, (min, max): H + DS18B20 tRSTL
    t_pdhigh: tuple        # reset release -> presence falls, (min, max)
    t_pdlow: tuple         # presence low, (min, max)
    t_rsth: float          # reset release -> first slot of that transaction, min
    t_low1: tuple          # master low inside a write-1 slot, (min, max)
    t_low0: tuple          # master low inside a write-0 slot, (min, max)
    t_slot: tuple          # slot period, falling edge to falling edge, (min, max)
    t_rec: float           # bus high between slots, min
    t_rdv: float           # a slave may not drive a read slot earlier than this
    t_tx0: tuple           # a read-0 ends this long after the slot start, (min, max)
    t_read_low: tuple      # master low inside a read slot, (min, max)
    # ---- what OneWireMaster drives ----
    w_rstl: float          # AN126 H, recommended
    w_low1: float          # AN126 A, recommended
    w_low0: float          # AN126 C, recommended
    w_read_low: float      # AN126 A, recommended (Figure 1: a read slot is A then E)
    w_sample: float        # when inside a read slot the master samples the line
    w_slot: float          # slot period the master uses for every slot
    w_post_presence: float # AN126 J, the wait after the presence pulse
    w_presence_wait: float # how long the master waits for the presence edge


ONEWIRE_SPEEDS: dict[str, OneWireSpeeds] = {
    # Standard speed. Every limit is the DS18B20 AC table; the maximum of the
    # reset low (640 us) is AN126 H, because the DS18B20 table has only the
    # 480 us minimum. w_slot is 70 us = AN126 A+B (write 1), C+D (write 0) and
    # A+E+F (read) all come to 70 with the recommended column, so one number
    # serves every slot.
    "standard": OneWireSpeeds(
        name="standard",
        t_rstl=(480_000, 640_000), t_pdhigh=(15_000, 60_000),
        t_pdlow=(60_000, 240_000), t_rsth=480_000,
        t_low1=(1_000, 15_000), t_low0=(60_000, 120_000),
        t_slot=(60_000, 120_000), t_rec=1_000, t_rdv=15_000,
        t_tx0=(60_000, 120_000), t_read_low=(1_000, 15_000),
        w_rstl=480_000, w_low1=6_000, w_low0=60_000, w_read_low=6_000,
        w_sample=20_000, w_slot=70_000, w_post_presence=410_000,
        w_presence_wait=60_000,
    ),
    # Overdrive speed. The DS18B20 datasheet has no overdrive AC table, so these
    # are AN126's overdrive column: A for the master's low in a write-1 and in a
    # read slot, C for a write-0, H for the reset low, E+F for the read slot's
    # length, I+J for tRSTH. tPDHIGH/tPDLOW stay at the DS18B20's standard-speed
    # numbers, because a slave's presence pulse is not a scaled signal and no
    # source gives an overdrive version. tSLOT's maximum, tREC, tRDV and tTX0
    # have no overdrive source and are therefore not checked.
    "overdrive": OneWireSpeeds(
        name="overdrive",
        t_rstl=(68_000, 80_000), t_pdhigh=(15_000, 60_000),
        t_pdlow=(60_000, 240_000), t_rsth=47_000,
        t_low1=(1_000, 1_850), t_low0=(7_000, 14_000),
        t_slot=(7_000, None), t_rec=0.0, t_rdv=0.0,
        t_tx0=(None, None), t_read_low=(1_000, 1_850),
        w_rstl=70_000, w_low1=1_500, w_low0=7_500, w_read_low=1_500,
        w_sample=2_250, w_slot=9_250, w_post_presence=40_000,
        w_presence_wait=60_000,
    ),
}


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


def _limit(ns_per_tick: float, value) -> Optional[tuple]:
    """One (min, max) limit in ns as (min_ticks, max_ticks); either end may be None,
    which means "the spec does not give this one" and the checker skips it."""
    if value is None:
        return None
    lo, hi = value
    return (None if lo is None else ns_to_ticks(ns_per_tick, lo),
            None if hi is None else ns_to_ticks(ns_per_tick, hi))


def speed_minimums(ns_per_tick: float, speed: str = "standard",
                   overrides: Optional[dict] = None) -> dict:
    """Every limit of `speed` as whole ticks, rounded up, plus the master's own
    waveform. `overrides` replaces individual nanosecond values by their field
    names, which is how a test drives one part of the bench off-spec on purpose:
    {"t_low1": (40_000, 90_000)} means "write-1 lows must last 40..90 us".

    The keys are the field names of `OneWireSpeeds` plus "speed". A limit that
    comes out as None stays None, so a test can see which rules the chosen speed
    has no source for and leave those alone.
    """
    try:
        spec = ONEWIRE_SPEEDS[speed]
    except KeyError:
        raise ValueError(
            f"unknown 1-Wire speed {speed!r}; pick one of {sorted(ONEWIRE_SPEEDS)}"
        ) from None
    out = {}
    for field_name in OneWireSpeeds.__dataclass_fields__:
        value = getattr(spec, field_name)
        if field_name == "name":
            out[field_name] = value
        elif isinstance(value, tuple):
            out[field_name] = _limit(ns_per_tick, value)
        else:
            out[field_name] = ns_to_ticks(ns_per_tick, value)
    out["speed"] = speed
    out["ns_per_tick"] = ns_per_tick
    for key, value in (overrides or {}).items():
        if key not in out or key in ("name", "speed", "ns_per_tick"):
            raise ValueError(
                f"unknown timing override {key!r}; pick from "
                f"{sorted(k for k in out if k not in ('name', 'speed', 'ns_per_tick'))}"
            )
        out[key] = _limit(ns_per_tick, value) if isinstance(value, tuple) else \
            ns_to_ticks(ns_per_tick, value)
    return out
@dataclass
class Violation:
    """One spec rule the bus did not meet, with the tick it was missed on.

    `bound` says which way the rule bites: "min" when the bus was too short,
    "max" when it was too long, "framing" when there is no measurement at all
    (a missing presence pulse, a CRC mismatch, an unknown ROM command).
    """

    tick: int
    rule: str
    bound: str                       # "min" | "max" | "framing"
    measured: Optional[int] = None    # ticks the bus gave it
    required: Optional[int] = None    # ticks the spec asks for
    detail: str = ""

    def __str__(self) -> str:
        if self.bound == "framing":
            return f"t={self.tick} {self.rule}" + (f" ({self.detail})" if self.detail else "")
        if self.bound == "max":
            return (f"t={self.tick} {self.rule} {self.measured} ticks > {self.required}"
                    + (f" ({self.detail})" if self.detail else ""))
        return (f"t={self.tick} {self.rule} {self.measured} ticks < {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


#: How many bytes each decoded group is: a ROM code is 8, a scratchpad 9, the three
#: bytes of a Write Scratchpad are 3, and everything else is one command byte.
GROUP_BYTES = {"rom": 8, "search": 8, "search_rom": 8, "scratchpad": 9,
               "scratchpad_write": 3, "rom_command": 1, "function": 1}

#: Every rule `OneWireTimingChecker` can report, in the order it documents them.
#: `test_onewire.py` breaks each of these in turn, so this tuple is the checklist.
RULES = (
    "tRSTL",        # master reset low: 480 us min, 640 us max (AN126 H)
    "presence",     # a reset must be answered by a presence pulse
    "tPDHIGH",      # presence falls 15..60 us after the reset release
    "tPDLOW",       # presence low lasts 60..240 us
    "tRSTH",        # line high for 480 us from the reset release to the first slot
    "tLOW1",        # write-1 low 1..15 us
    "tLOW0",        # write-0 low 60..120 us
    "tSLOT",        # slot period 60..120 us
    "tREC",         # at least 1 us high between slots
    "tRDV",         # no slave drive before 15 us into a read slot
    "tTX0",         # a read-0 ends 60..120 us after its slot start
    "CRC",          # CRC-8 over the 8 ROM bytes or the 8 scratchpad bytes
    "rom-command",  # the first byte after a reset is one of the five ROM commands
    "no-reset",     # no time slot before any reset
    "slave-drive",  # a slave drove inside a write slot, or too late in a read slot
)


@dataclass
class OneWireOp:
    """One queued master operation. Filled in by the master as it runs."""

    kind: str                      # "reset" | "write" | "read" | "idle"
    count: int = 0                 # bytes to read, or ticks to idle
    write: bytes = b""             # bytes to write
    data: bytearray = field(default_factory=bytearray)   # bytes received
    sent: int = 0                  # bytes written
    present: Optional[bool] = None  # presence seen by the reset that opened this
    error: str = ""
    start_tick: int = -1
    stop_tick: int = -1

    @property
    def ok(self) -> bool:
        if self.error:
            return False
        if self.kind == "read":
            return len(self.data) >= self.count
        return True


class OneWireMaster(Device):
    """Bit-level 1-Wire master for queued transactions on the open-drain dq net.

    Every timing comes from `OneWireSpeeds`: the reset pulse is the recommended
    column of AN126 H, a write-1 low is A, a write-0 low is C, a read slot is A
    then E with the sample inside the datasheet's 15..60 us window, and every
    slot is one `w_slot` long so the whole stream is one `tSLOT` per slot.

    The presence pulse is handled the way a real master has to handle it rather
    than with a fixed delay: after the reset the master releases the line and
    waits for the *edge* (up to `w_presence_wait`, the tPDHIGH maximum). A slave
    that starts its presence pulse later than that is reported as absent, which
    is the spec's own boundary, and it also keeps the overdrive speed working
    with a slave whose presence pulse is still standard speed.

    Bits go out least significant bit first, so `queue_write(0x33)` puts 1,1,0,0,1,1,0,0
    on the wire; that is the byte order the datasheet means by "the data must be
    transmitted to the DS18B20 starting with the least significant bit of byte 2".

    It refuses to queue a write or a read that does not follow a reset: "the
    DS18B20 will not respond if any steps in the sequence are missing or out of
    order". Use `queue_reset()` first, as the helpers below do.
    """

    name = "ow_master"

    def __init__(
        self,
        speed: str = "standard",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        net: str = "dq",
        name: Optional[str] = None,
    ):
        self.speed = speed
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.net = net
        if name:
            self.name = name
        self.min = speed_minimums(self.ns_per_tick, speed, self.timing_overrides)
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        """Turn the speed class into the tick counts this master drives."""
        m = self.min
        self.rst_low = max(1, m["w_rstl"])
        self.presence_wait = max(1, m["w_presence_wait"])
        # How long to wait for the presence pulse to end before giving up on it.
        self.presence_max = max(1, (m["t_pdlow"][1] or m["w_presence_wait"]) * 2)
        self.post_presence = max(1, m["w_post_presence"])
        self.low1 = max(1, m["w_low1"])
        self.low0 = max(1, m["w_low0"])
        self.read_low = max(1, m["w_read_low"])
        self.rdv = max(1, m["w_sample"])
        # One slot length for every slot: it has to be at least the longest pulse
        # the master drives inside it, and it is AN126's A+B = C+D = A+E+F = 70 us
        # at standard speed.
        self.slot = max(m["w_slot"], self.low0, self.low1, self.read_low)

    def bind(self, bus: Bus) -> "OneWireMaster":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = speed_minimums(self.ns_per_tick, self.speed, self.timing_overrides)
        self._compute_timings()
        self._reset()
        return self

    def _reset(self) -> None:
        self.ops: list[OneWireOp] = []
        self.errors: list[str] = []
        self.transactions: list[tuple] = []     # (start, stop, kind, nbytes, present)
        self.phase = "idle"
        self.op: Optional[OneWireOp] = None
        self.plan: list[str] = []               # "w1" | "w0" | "rd" for this op
        self.pi = 0
        self.cnt = 0
        self.slot_call = 0                      # call index that started the slot
        self.sampled = False
        self.rd_shift = 0
        self.rd_bits = 0
        self.presence = False
        self.presence_tick = -1
        self.presences: list[tuple] = []        # (tick, presence)
        self.seq_open = False                   # a reset has been sent or queued
        self.last_present: Optional[bool] = None

    # ---- the queue ---------------------------------------------------------------
    def queue_reset(self) -> OneWireOp:
        """A reset pulse, then wait for the presence pulse. Result in op.present."""
        op = OneWireOp("reset")
        self.ops.append(op)
        self.seq_open = True
        return op

    def queue_write(self, data) -> OneWireOp:
        """Write bytes into the open transaction, least significant bit first."""
        self._need_transaction("queue_write")
        op = OneWireOp("write", write=bytes(data))
        op.sent = len(op.write)
        self.ops.append(op)
        return op

    def queue_read(self, count: int) -> OneWireOp:
        """Read `count` bytes into op.data, least significant bit first."""
        self._need_transaction("queue_read")
        if count < 0:
            raise ValueError(f"a 1-Wire read cannot be {count} bytes")
        op = OneWireOp("read", count=count)
        self.ops.append(op)
        return op

    def queue_idle(self, ticks: int) -> OneWireOp:
        """Hold the line released for `ticks`: the strong-pullup window that a
        conversion or an EEPROM write needs ("the bus must be held high by the
        pullup for the duration of the conversion")."""
        op = OneWireOp("idle")
        op.count = ticks
        self.ops.append(op)
        return op

    def _need_transaction(self, what: str) -> None:
        if not self.seq_open:
            raise ValueError(
                f"{what} needs an open transaction: queue_reset() first, because a "
                "DS18B20 ignores a function command that no ROM command opened"
            )

    # ---- command helpers, all of them reset + ROM command + function command ----
    def queue_read_rom(self) -> OneWireOp:
        """reset, Read ROM [33h], then the 8 ROM bytes. Only legal with one slave."""
        self.queue_reset()
        self.queue_write(b"\x33")
        return self.queue_read(8)

    def queue_match_rom(self, rom) -> OneWireOp:
        """reset, Match ROM [55h], then the 8 ROM bytes to address."""
        self.queue_reset()
        self.queue_write(b"\x55")
        return self.queue_write(bytes(rom))

    def queue_skip_rom(self) -> OneWireOp:
        """reset, Skip ROM [CCh]: address every slave at once."""
        self.queue_reset()
        return self.queue_write(b"\xcc")

    def queue_search_rom(self, bits: int = -1) -> OneWireOp:
        """reset, Search ROM [F0h], then the 64 search bits and 64 read slots.

        `bits` is the 64-bit search state the master is following: a 0 bit makes
        every slave whose ROM has a 0 in that position answer, a 1 bit makes every
        slave whose ROM has a 1 there answer. -1 (the default) is all ones, which
        returns the ROM of one slave. Only the bit choice is modelled, not the
        process of elimination from AN937.
        """
        if not -1 <= bits <= (1 << 64) - 1:
            raise ValueError("a 1-Wire search state is 64 bits")
        self.queue_reset()
        self.queue_write(b"\xf0")
        branch = (1 << 64) - 1 if bits < 0 else bits
        self._search_bits(branch)
        return self.queue_read(8)

    def queue_alarm_search(self, bits: int = -1) -> OneWireOp:
        """reset, Alarm Search [ECh], then the 64 search bits and 64 read slots."""
        self.queue_reset()
        self.queue_write(b"\xec")
        self._search_bits((1 << 64) - 1 if bits < 0 else bits)
        return self.queue_read(8)

    def _search_bits(self, branch: int) -> OneWireOp:
        """Queue the 64 search bits a Search ROM command needs before its reads.

        They go out least significant bit first, so bit 0 of the search state is
        the first bit of the first ROM byte on the wire.
        """
        return self.queue_write(branch.to_bytes(8, "little"))

    def queue_convert_t(self, conv_ticks: int = 0) -> OneWireOp:
        """reset, Skip ROM, Convert T [44h], then hold the bus high for the
        conversion. `conv_ticks` is the strong-pullup window; 0 means the target
        converts instantly (see the README in research/ref/protocols/onewire)."""
        self.queue_reset()
        self.queue_write(b"\xcc")
        op = self.queue_write(b"\x44")
        if conv_ticks:
            self.queue_idle(conv_ticks)
        return op

    def queue_read_scratchpad(self) -> OneWireOp:
        """reset, Skip ROM, Read Scratchpad [BEh], then the 9 scratchpad bytes."""
        self.queue_reset()
        self.queue_write(b"\xcc")
        self.queue_write(b"\xbe")
        return self.queue_read(9)

    def queue_write_scratchpad(self, th: int, tl: int, config: int) -> OneWireOp:
        """reset, Skip ROM, Write Scratchpad [4Eh], then TH, TL and configuration.

        All three bytes have to be written before the next reset, so this helper
        always sends exactly three.
        """
        self.queue_reset()
        self.queue_write(b"\xcc")
        self.queue_write(b"\x4e")
        return self.queue_write(bytes([th & 0xFF, tl & 0xFF, config & 0xFF]))

    def queue_copy_scratchpad(self) -> OneWireOp:
        """reset, Skip ROM, Copy Scratchpad [48h]: scratchpad bytes 2..4 to EEPROM."""
        self.queue_reset()
        self.queue_write(b"\xcc")
        return self.queue_write(b"\x48")

    def queue_recall_e2(self) -> OneWireOp:
        """reset, Skip ROM, Recall E2 [B8h]: EEPROM back into the scratchpad."""
        self.queue_reset()
        self.queue_write(b"\xcc")
        return self.queue_write(b"\xb8")

    @property
    def busy(self) -> bool:
        return self.phase != "idle" or bool(self.ops)

    @property
    def idle(self) -> bool:
        return not self.busy

    def run(self, bus: Bus, limit: int = 8_000_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out."""
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- drives ------------------------------------------------------------------
    def _drive(self) -> dict:
        # Open drain: 0 pulls the line low, None releases it. Never 1.
        low = self.phase in ("rst_low", "w1_low", "w0_low", "rd_low")
        return {self.net: 0 if low else None}

    def tick(self, t: int, pins) -> dict:
        # Advance first, then report the drive: a phase that spans N calls is then
        # exactly N ticks of that level on the bus, which is how every length below
        # was derived. The one-tick latency of core.py's two-phase tick is in the
        # `+ 1` of the read sample and nowhere else.
        self._advance(t, pins)
        return self._drive()
# ---- the state machine --------------------------------------------------------
    def _advance(self, t: int, pins) -> None:
        line = pins[self.net]
        phase = self.phase
        if phase == "idle":
            if self.ops:
                self._begin_op(t)
        elif phase == "rst_low":
            self.cnt += 1
            if self.cnt >= self.rst_low:
                self.phase, self.cnt = "rst_presence", 0
        elif phase == "rst_presence":
            # Wait for the presence *edge*, not for a delay: that is the only way
            # one master works at both speeds with a slave whose presence pulse is
            # standard speed. tPDHIGH's maximum is how long we are willing to wait.
            if line == 0:
                self.presence = True
                self.presence_tick = t
                self.phase, self.cnt = "rst_presence_low", 0
            else:
                self.cnt += 1
                if self.cnt >= self.presence_wait:
                    self.presence = False
                    self.phase, self.cnt = "rst_recover", 0
        elif phase == "rst_presence_low":
            if line == 1:
                self.phase, self.cnt = "rst_recover", 0
            else:
                self.cnt += 1
                if self.cnt >= self.presence_max:
                    self._error(f"the presence pulse did not end within "
                                f"{self.presence_max} ticks")
                    self.phase, self.cnt = "rst_recover", 0
        elif phase == "rst_recover":
            self.cnt += 1
            if self.cnt >= self.post_presence:
                if self.op is not None:
                    self.op.present = self.presence
                self._end_op(t)
        elif phase == "idle_high":
            self.cnt += 1
            if self.cnt >= (self.op.count if self.op else 0):
                self._end_op(t)
        elif phase == "w1_low":
            self.cnt += 1
            if self.cnt >= self.low1:
                self.phase, self.cnt = "slot_high", 0
        elif phase == "w0_low":
            self.cnt += 1
            if self.cnt >= self.low0:
                self.phase, self.cnt = "slot_high", 0
        elif phase == "rd_low":
            self.cnt += 1
            if self.cnt >= self.read_low:
                self.phase, self.cnt = "rd_wait", 0
        elif phase == "rd_wait":
            # `+ 1` for the one tick it takes a drive to reach the pins: the slot's
            # falling edge is at pin time slot_call + 1, and pins[t] is what the
            # master reads at call t.
            if not self.sampled and t - self.slot_call >= 1 + self.rdv:
                self._sample(t, line)
                self.sampled = True
            if self.sampled and t - self.slot_call >= self.slot:
                self._end_slot(t)
        elif phase in ("slot_high", "rd_high"):
            if t - self.slot_call >= self.slot:
                self._end_slot(t)
        else:
            raise RuntimeError(f"{self.name}: bad phase {self.phase!r}")

    def _begin_op(self, t: int) -> None:
        op = self.ops.pop(0)
        self.op = op
        op.start_tick = t
        op.present = self.last_present
        if op.kind == "reset":
            self.plan, self.pi = [], 0
            self.presence = False
            self.presence_tick = -1
            self.phase, self.cnt = "rst_low", 0
            return
        if op.kind == "idle":
            self.plan, self.pi = [], 0
            self.phase, self.cnt = "idle_high", 0
            return
        if op.kind == "write":
            plan: list[str] = []
            for byte in op.write:
                for bit in range(8):           # least significant bit first
                    plan.append("w1" if (byte >> bit) & 1 else "w0")
        elif op.kind == "read":
            plan = ["rd"] * (8 * op.count)
        else:
            raise RuntimeError(f"{self.name}: bad op kind {op.kind!r}")
        self.plan, self.pi = plan, 0
        self.rd_shift, self.rd_bits = 0, 0
        if not plan:                            # a zero-length read or write
            self._end_op(t)
            return
        self._begin_slot(t, plan[0])

    def _begin_slot(self, t: int, kind: str) -> None:
        self.slot_call = t
        self.sampled = False
        self.phase = {"w1": "w1_low", "w0": "w0_low", "rd": "rd_low"}[kind]
        self.cnt = 0

    def _sample(self, t: int, line: int) -> None:
        bit = 1 if line else 0
        self.rd_shift |= bit << self.rd_bits
        self.rd_bits += 1
        if self.rd_bits == 8:
            if self.op is not None and self.op.kind == "read":
                self.op.data.append(self.rd_shift)
            self.rd_shift, self.rd_bits = 0, 0

    def _end_slot(self, t: int) -> None:
        self.pi += 1
        if self.pi < len(self.plan):
            self._begin_slot(t, self.plan[self.pi])
        else:
            self._end_op(t)

    def _end_op(self, t: int) -> None:
        if self.op is not None:
            self.op.stop_tick = t
            if self.op.kind == "reset":
                self.last_present = self.presence
                self.presences.append((t, self.presence))
            self.transactions.append(
                (self.op.start_tick, t, self.op.kind,
                 len(self.op.data) if self.op.kind == "read" else self.op.sent,
                 self.op.present)
            )
            self.op = None
        self.phase, self.cnt = "idle", 0
        self.plan, self.pi = [], 0

    def _error(self, msg: str) -> None:
        self.errors.append(f"{self.name}: {msg}")
        if self.op is not None:
            self.op.error = msg
class Ds18b20Target(Device):
    """DS18B20-like 1-Wire slave: 64-bit ROM, 9-byte scratchpad, real commands.

    It implements what the datasheet describes: the reset-triggered presence pulse
    ("waits 15 µs to 60 µs and then transmits a presence pulse"), the five ROM
    commands with a Match ROM that only one device answers, the six function
    commands, the read-only temperature bytes, TH/TL and the configuration register
    with their power-on contents, the scratchpad CRC, and the read slot where the
    device answers 0 while a conversion is still running.

    Its own waveform timings are the model's, chosen inside the datasheet windows
    and listed in `presence_delay`, `presence_low`, `sample`, `tx0_start` and
    `tx0_end`: presence after 30 µs (inside tPDHIGH's 15..60) for 120 µs (inside
    tPDLOW's 60..240), write slots sampled at 30 µs (the datasheet's own "DS18B20
    Samples 15 typ 30" figure), and a read-0 driven from 25 µs to 65 µs after the
    slot start so that it ends inside tTX0's 60..120 and leaves tREC's 1 µs before
    the next slot. Every one of them is measured by `OneWireTimingChecker`.

    `conv_ticks` is the one thing a test may want to change: 0 (the default) means
    a conversion finishes immediately, because the real 750 ms is 37.5 million
    ticks at 20 ns/tick. Set it to a few thousand ticks and the device answers
    read slots with 0 until the conversion is done, which is the datasheet's own
    status protocol and catches firmware that reads too early.
    """

    name = "ow_ds18b20"

    def __init__(
        self,
        serial=b"\x01\x02\x03\x04\x05\x06",
        speed: str = "standard",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        net: str = "dq",
        family: int = FAMILY_DS18B20,
        rom: Optional[bytes] = None,
        alarm: bool = False,
        parasite_power: bool = False,
        conv_ticks: int = 0,
        presence_delay: Optional[float] = None,
        presence_low: Optional[float] = None,
        sample: Optional[float] = None,
        tx0_start: Optional[float] = None,
        tx0_end: Optional[float] = None,
        name: Optional[str] = None,
    ):
        self.speed = speed
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.net = net
        self.family = family
        self.rom = bytes(rom) if rom is not None else rom_from_serial(serial, family)
        if len(self.rom) != 8:
            raise ValueError(f"a 1-Wire ROM code is 8 bytes, got {len(self.rom)}")
        self.alarm = alarm
        self.parasite_power = parasite_power
        self.conv_ticks = conv_ticks
        if name:
            self.name = name
        self.timing = speed_minimums(self.ns_per_tick, speed)
        # The slave's own waveform: datasheet windows at standard speed, scaled to
        # the same shape at overdrive (no DS18B20 overdrive table exists to quote).
        m = self.timing
        # The shortest a reset can be: how a device tells one from a time slot.
        self.rst_low = m["t_rstl"][0]
        period = m["w_slot"]                  # a slot, in ticks: the frame for all of this
        # The five waveform knobs below are the *model's* own timings, in
        # nanoseconds, chosen inside the datasheet windows: how long after a reset
        # the presence pulse starts and how long it lasts, where a write slot is
        # sampled, and when a read-0 is driven and released. A test passes them to
        # make the model break a rule on purpose.
        self.presence_delay = self._ns(presence_delay, 30_000)
        self.presence_low = self._ns(presence_low, 120_000)
        # The datasheet samples a write slot 15 us min, 30 us typ, 60 us max.
        self.sample = min(self._ns(sample, 30_000), max(1, period // 2))
        # A read-0 starts as soon as the slave's data is valid (tRDV, with a
        # microsecond of slack so the ceil() never lands one tick short) and before
        # the middle of the slot, so any master that samples inside the datasheet's
        # 15..60 us window sees it. Everything here is in ticks.
        start = (ns_to_ticks(self.ns_per_tick, tx0_start) if tx0_start is not None else
                 min((m["t_rdv"] or max(1, period // 8))
                     + ns_to_ticks(self.ns_per_tick, 1_000), max(2, period // 2)))
        end = (ns_to_ticks(self.ns_per_tick, tx0_end) if tx0_end is not None else
               int((m["t_tx0"][0] or period - max(1, period // 8)) * 1.1))
        if tx0_end is not None and end <= start:
            raise ValueError("a read-0 needs tx0_end after tx0_start")
        self.tx0_start, self.tx0_end = start, max(start + 1, end)
        self.temperature = DS18B20_POWERUP_TEMP       # 16-bit, byte 0 = LSB
        self.th = 0x4B                                 # 75 C, see the README
        self.tl = 0x00                                 # 0 C
        self.config = 0x7F                             # R1 R0 = 11, 12-bit
        self.scratchpad = bytearray(9)
        self.eeprom = bytearray(3)                     # TH, TL, configuration
        self.converting = False
        self.convert_left = 0
        self.write_log: list[tuple] = []               # (tick, byte index, value)
        self.resets = 0
        self.rom_commands: list[tuple] = []
        self.function_commands: list[tuple] = []
        self.match_failures = 0
        self._sync_eeprom()
        self._refresh_scratchpad()
        self._reset()

    def _ns(self, value: Optional[float], default: float) -> int:
        """Nanoseconds to ticks, or the default in nanoseconds to ticks."""
        return ns_to_ticks(self.ns_per_tick, default if value is None else value)

    # ---- memory ----------------------------------------------------------------
    def _sync_eeprom(self) -> None:
        self.eeprom[0], self.eeprom[1], self.eeprom[2] = self.th, self.tl, self.config

    def _refresh_scratchpad(self) -> None:
        """Bytes 0..7 plus the CRC in byte 8, recomputed from whatever is in RAM."""
        sp = self.scratchpad
        sp[0] = self.temperature & 0xFF
        sp[1] = (self.temperature >> 8) & 0xFF
        sp[2] = self.th & 0xFF
        sp[3] = self.tl & 0xFF
        # "Bits 0-4 are don't cares on a write but will always read out '1'. Bit 7
        # is a don't care on a write but will always read out '0'."
        self.config = 0x1F | (self.config & 0x60)
        sp[4] = self.config
        sp[5], sp[6], sp[7] = DS18B20_RESERVED_05
        sp[8] = crc8(sp[:8])

    @property
    def resolution(self) -> int:
        """Bits of temperature resolution, from R1:R0 of the configuration register."""
        return 9 + ((self.config >> 5) & 0x03)

    @resolution.setter
    def resolution(self, bits: int) -> None:
        if bits not in (9, 10, 11, 12):
            raise ValueError(f"a DS18B20 resolution is 9, 10, 11 or 12 bits, got {bits}")
        self.config = (self.config & ~0x60) | ((bits - 9) << 5)
        self._sync_eeprom()
        self._refresh_scratchpad()

    @classmethod
    def conv_time_ticks(cls, resolution: int, ns_per_tick: float) -> int:
        """tCONV for a resolution: 750 ms at 12 bits, /2 per bit below (Table 2).

        The model does not wait this long; the number is here so a test can state
        what it is not waiting for. At 20 ns/tick the 12-bit conversion is 37.5M
        ticks, which is why `conv_ticks` exists.
        """
        table = {9: 93_750_000, 10: 187_500_000, 11: 375_000_000, 12: 750_000_000}
        if resolution not in table:
            raise ValueError(f"a DS18B20 resolution is 9, 10, 11 or 12 bits, got {resolution}")
        return ns_to_ticks(ns_per_tick, table[resolution])

    def temperature_raw(self) -> int:
        """The temperature register as it would be read: 16-bit, LSB first."""
        self._refresh_scratchpad()
        return self.temperature & 0xFFFF

    def set_temperature(self, raw: int) -> None:
        """Set the temperature register directly (16-bit two's complement, °C x 16)."""
        self.temperature = raw & 0xFFFF
        self._refresh_scratchpad()
# ---- the target's protocol state ---------------------------------------------
    def _reset(self) -> None:
        self.state = "idle"      # idle | low | presence | presence_low | slots | own
        self.line = 1            # what the pins read last tick
        self.low_start = 0       # pin time the current low began
        self.low_len = 0         # how long the last low lasted, in ticks
        self.slot_call = 0       # pin time the current slot's falling edge was seen
        self.own_from = None     # call index of a pull this device scheduled
        self.own_to = 0
        self.drive_low = False
        self.expect = "none"     # "rx" | "tx" | "none" for the next slot
        self.stage = "rom_command"
        self.cmd = None          # the ROM command this transaction started with
        self.matched = True      # Match ROM / Search ROM outcome
        self.search_branch: Optional[int] = None
        self.rx = bytearray()    # bytes being received
        self.rx_left = 0         # how many are still to come
        self.rxn = 0
        self.rxb = 0
        self.tx = bytearray()    # bytes being transmitted
        self.tx_left = 0
        self.tx_index = 0
        self.tx_bits = 0
        self.sample_due = False
        self.errors: list[str] = []

    @property
    def addressed(self) -> bool:
        """True when this transaction is one this device answers.

        Skip ROM addresses everybody; Read ROM is answered by everybody; Match ROM
        only by the device whose 64-bit code matches; Search ROM and Alarm Search
        only by a device whose ROM agrees with the search branch. Alarm Search
        answers only while `alarm` is true, which is what makes the alarm useful.
        """
        if self.cmd is None:
            return True
        if self.cmd == 0xCC or self.cmd == 0x33:
            return True
        if self.cmd == 0xEC and not self.alarm:
            return False                # Alarm Search [ECh]: only a device in alarm
        return self.matched

    # ---- drives ------------------------------------------------------------------
    def _drive(self) -> dict:
        return {self.net: 0 if self.drive_low else None}

    def tick(self, t: int, pins) -> dict:
        self._advance(t, pins)
        if self.own_from is not None and t >= self.own_from:
            self.drive_low = t < self.own_to
            if t >= self.own_to:
                self.own_from = None
        return self._drive()

    def _pull(self, t: int, start: int, stop: int) -> None:
        """Pull the line low from `start` to `stop` ticks after now.

        The offsets are counts of calls and the drive appears on the pins one tick
        later, which is why every measurement here is a pin time and not a call
        time.
        """
        self.own_from, self.own_to = t + max(0, start), t + max(0, stop)

    # ---- edges --------------------------------------------------------------------
    def _advance(self, t: int, pins) -> None:
        line = pins[self.net]
        if line != self.line:
            if line == 0:
                self.low_start = t
                self._fall(t)
            else:
                self.low_len = t - self.low_start
                self._rise(t)
            self.line = line
        if self.sample_due and t - self.slot_call >= self.sample:
            self.sample_due = False
            self._sample_bit(t, pins[self.net])
        if self.convert_left > 0:
            self.convert_left -= 1
            if self.convert_left == 0:
                self.converting = False

    def _fall(self, t: int) -> None:
        """The line fell. Either this device is the one driving it -- the presence
        pulse or a read-0 -- or the master has just started a time slot."""
        if self.own_from is not None and self.own_from + 1 <= t <= self.own_to:
            self.state = "own"                  # our own drive reached the pins
            return
        self.slot_call = t                     # this fall begins a time slot
        self._arm(t)

    def _rise(self, t: int) -> None:
        """The line rose: its length says whether a reset pulse or a master pulse
        inside a time slot just ended."""
        if self.state == "own":
            self.state = "slots"               # our own drive ended
            return
        self.state = "slots"
        if self.low_len >= self.rst_low:
            # A reset pulse: its low is at least tRSTL and no time slot can be that
            # long, so the two are never confused.
            self._reset_seen(t)
            return
        self._fire(t)

    def _arm(self, t: int) -> None:
        """A time slot just began at `t`, so its direction is decided now, from what
        the command decoder had waiting when it began. A write slot is sampled where
        the datasheet samples it -- 15..60 us after the slot began, which for a
        write-0 is before the master's pulse has even ended -- and that sample can
        finish the byte, which is why the direction has to be latched here rather
        than looked up again when the master's pulse ends."""
        if self.rx_left > 0:
            self.expect, self.sample_due = "rx", True
        elif self.tx_left > 0 or self.stage in ("status", "supply"):
            # A queued byte, or one of the two answers the datasheet repeats on
            # every read slot: "transmitting 0 while the temperature conversion is in
            # progress and 1 when the conversion is done".
            self.expect, self.sample_due = "tx", False
        else:
            self.expect, self.sample_due = "none", False

    def _fire(self, t: int) -> None:
        """The master released the bus inside the slot that began at
        `self.slot_call`. A read-0 has to start after the master let go of the line,
        so the bit to send is only decided now."""
        if self.expect != "tx":
            return
        bit = (self._level_bit() if self.stage in ("status", "supply")
               else self._next_tx_bit())
        if bit == 0 and self.addressed:
            self._pull(t, self.slot_call + self.tx0_start - t,
                       self.slot_call + self.tx0_end - t)

    def _reset_seen(self, t: int) -> None:
        """A reset pulse just ended: answer it with a presence pulse 15..60 us later
        and forget everything about the previous transaction."""
        self.resets += 1
        self.sample_due = False
        self.cmd, self.matched, self.search_branch = None, True, None
        self.rx, self.tx = bytearray(), bytearray()
        self.rx_left = self.tx_left = 0
        self._start_rx(1, "rom_command")        # the next byte is a ROM command
        self._pull(t, self.presence_delay, self.presence_delay + self.presence_low)



    def _sample_bit(self, t: int, line: int) -> None:
        """One bit of a write slot, sampled at `self.sample` ticks into the slot."""
        self.rxb |= (1 if line else 0) << self.rxn
        self.rxn += 1
        if self.rxn < 8:
            return
        value, self.rxn, self.rxb = self.rxb, 0, 0
        self.rx.append(value)
        self.rx_left -= 1
        if self.rx_left == 0:
            self._after_byte(value, t)

    def _next_tx_bit(self) -> int:
        """The next bit of the bytes queued to send, least significant bit first."""
        if self.tx_index >= len(self.tx):
            self.tx_left = 0
            return 1
        bit = (self.tx[self.tx_index] >> self.tx_bits) & 1
        self.tx_bits += 1
        if self.tx_bits == 8:
            self.tx_bits, self.tx_index = 0, self.tx_index + 1
            if self.tx_index >= len(self.tx):
                self.tx_left = 0
        return bit

    def _start_rx(self, nbytes: int, stage: str) -> None:
        # `expect` is not touched here: it is latched per slot in _arm(), because a
        # byte that finishes mid-slot must not change what the slot being received
        # was. See _arm().
        self.rx, self.rx_left = bytearray(), nbytes
        self.rxn = self.rxb = 0
        self.stage = stage

    def _start_tx(self, data, stage: str) -> None:
        self.tx = bytearray(data)
        self.tx_index = self.tx_bits = 0
        self.tx_left = len(self.tx)
        self.stage = stage

    def _after_byte(self, value: int, t: int) -> None:
        """A whole byte has arrived: decide what it means and what comes next."""
        stage = self.stage
        if stage == "rom_command":
            self.cmd = value
            self.rom_commands.append((t, value))
            if value == 0x55:
                self._start_rx(8, "match_rom")
            elif value in (0xF0, 0xEC):
                self._start_rx(8, "search_branch")
            elif value == 0x33:
                self._start_tx(self.rom, "done")
            elif value == 0xCC:
                self._start_rx(1, "function")
            else:
                self._expect_none("ignore")
        elif stage == "match_rom":
            self.matched = bytes(self.rx) == self.rom
            if not self.matched:
                self.match_failures += 1        # "all other slaves ... wait for a reset"
                self._expect_none("ignore")
            else:
                self._start_rx(1, "function")
        elif stage == "search_branch":
            self.search_branch = int.from_bytes(bytes(self.rx), "little")
            self.matched = True
            self._start_tx(self._search_bytes(), "done")
        elif stage == "function":
            self.function_commands.append((t, value))
            self._function(value)
        elif stage == "write_scratchpad":
            th, tl, config = bytes(self.rx)
            self.th, self.tl, self.config = th, tl, config
            self._sync_eeprom()
            self._refresh_scratchpad()
            self.write_log.append((t, th, tl, config))
            self._expect_none("done")

    def _level_bit(self) -> int:
        """The bit a repeating answer transmits on every read slot: the conversion
        status, or the power-supply mode -- "parasite powered DS18B20s will pull the
        bus low, and externally powered DS18B20s will let the bus remain high"."""
        if self.stage == "status":
            return 0 if self.converting else 1
        return 0 if self.parasite_power else 1

    def _function(self, value: int) -> None:
        """A DS18B20 Function Command, Table 4."""
        if value == 0xBE:                       # Read Scratchpad: 9 bytes out
            self._refresh_scratchpad()
            self._start_tx(bytes(self.scratchpad), "done")
        elif value == 0x4E:                     # Write Scratchpad: TH, TL, config in
            self._start_rx(3, "write_scratchpad")
        elif value == 0x44:                     # Convert T, then a status bit
            self.converting = self.conv_ticks > 0
            self.convert_left = self.conv_ticks
            self._expect_none("status")
        elif value == 0x48:                     # Copy Scratchpad: bytes 2..4 to EEPROM
            self._sync_eeprom()
            self._expect_none("done")
        elif value == 0xB8:                     # Recall E2, then a status bit
            self.th, self.tl, self.config = bytes(self.eeprom)
            self._refresh_scratchpad()
            self._expect_none("status")
        elif value == 0xB4:                     # Read Power Supply
            self._expect_none("supply")
        else:
            self._expect_none("ignore")

    def _expect_none(self, stage: str) -> None:
        """Nothing more is expected in this transaction."""
        self.expect, self.stage = "none", stage
        self.rx_left = self.tx_left = 0
        self.sample_due = False

    def _search_bytes(self) -> bytes:
        """This device's ROM as a Search ROM answers it: a position where the ROM
        bit and the branch bit agree transmits the ROM bit, and every other
        position transmits 1 (the device stays out of the way)."""
        branch = self.search_branch or 0
        rom = int.from_bytes(self.rom, "little")     # bit 0 is the first bit on the wire
        out = bytearray()
        for i in range(64):
            rom_bit = (rom >> i) & 1
            bit = rom_bit if rom_bit == ((branch >> i) & 1) else 1
            if i % 8 == 0:
                out.append(0)
            out[i // 8] |= bit << (i % 8)
        return bytes(out)
@dataclass
class ByteTransfer:
    """One byte the checker decoded, with the tick its eighth slot began."""

    tick: int
    value: int
    group: str            # "rom_command" | "rom" | "function" | "scratchpad" | ...
    direction: str        # "write" (the master sent it) | "read" (a slave sent it)

    def __str__(self) -> str:
        return f"t={self.tick} 0x{self.value:02x} {self.group} {self.direction}"


class OneWireTimingChecker(Device):
    """Passive 1-Wire monitor: decodes the wire and checks the datasheet.

    It drives nothing, so it sees the bus exactly as a real analyser would. It
    measures resolved pin values and adds no latency of its own for the two-phase
    tick: what it reports is what the pins did.

    Timing rules, all from the tables in `speed_minimums`: tRSTL (a reset low has
    to be 480 us and no more than AN126's 640 us), tPDHIGH and tPDLOW around the
    presence pulse, tRSTH from the reset release to the first slot of that
    transaction, tLOW1 and tLOW0 for the master's low in a write slot, tSLOT for
    the slot period, tREC for the recovery time between slots, tRDV for a drive
    that comes too early inside a read slot, and tTX0 for a read-0 that ends
    outside its window. Every miss is appended to `violations` with its tick.

    Framing rules: `presence` (a reset nobody answered), `rom-command` (the first
    byte after a reset is not one of the five ROM commands, or a function command
    was sent with no ROM command in front of it), `no-reset` (a time slot before
    any reset), `slave-drive` (a slave pulled the bus low inside a write slot, or
    so late in a read slot that it looks like the next slot), and `CRC` (the
    CRC-8 of a 64-bit ROM or a 9-byte scratchpad does not match the last byte).

    Two limits are worth stating plainly:

    * A master samples the bus inside its own chip, which no pin-level monitor can
      see. What this checker can check about a read slot is the master's own low
      (`t_read_low`) and everything either side does to the line, which is why the
      tRDV rule is written as "nobody may pull the bus low before 15 us into a
      read slot" and not as "the master sampled too early".
    * `finish()` has to be called when the traffic stops. A trace that ends while a
      reset is still waiting for its presence pulse, or in the middle of a byte,
      cannot be judged from the last edge alone; `finish()` closes both. It is
      idempotent.
    """

    name = "ow_check"

    def __init__(
        self,
        speed: str = "standard",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        net: str = "dq",
        name: Optional[str] = None,
    ):
        self.speed = speed
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.net = net
        if name:
            self.name = name
        self.min = speed_minimums(self.ns_per_tick, speed, self.timing_overrides)
        self.violations: list[Violation] = []
        self.transfers: list[ByteTransfer] = []
        self.crc_checks = 0
        self.resets = 0
        self.presences = 0
        self.slots = 0
        self.write_slots = 0
        self.read_slots = 0
        self.bytes = 0
        self._reset()
        self._finished = False

    def bind(self, bus: Bus) -> "OneWireTimingChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = speed_minimums(self.ns_per_tick, self.speed, self.timing_overrides)
        return self

    def _reset(self) -> None:
        m = self.min
        # The limits, unpacked for readability. `t_rstl_min` doubles as the length
        # that tells a reset from a time slot: no slot can be that long.
        self.t_rstl_min, self.t_rstl_max = m["t_rstl"]
        self.t_pdhigh_min, self.t_pdhigh_max = m["t_pdhigh"]
        self.t_pdlow_min, self.t_pdlow_max = m["t_pdlow"]
        self.t_rsth = m["t_rsth"]
        self.t_low1_min, self.t_low1_max = m["t_low1"]
        self.t_low0_min, self.t_low0_max = m["t_low0"]
        self.t_slot_min, self.t_slot_max = m["t_slot"]
        self.t_rec = m["t_rec"]
        self.t_rdv = m["t_rdv"]
        self.t_tx0_min, self.t_tx0_max = m["t_tx0"]
        self.t_read_min, self.t_read_max = m["t_read_low"]
        # A low longer than this is a write-0, shorter is a write-1 or a read slot.
        self.split = max(1, (self.t_low1_max + self.t_low0_min) // 2)
        # ---- wire state ----
        self.phase = "idle"        # idle | await_presence | presence_low | slots
        self.line = 1
        self.low_start = 0         # pin time the current low began
        self.low_fall = 0          # the same, kept for the rise to use
        self.low_len = 0           # how long the last low lasted, in ticks
        self.low_is_drive = False  # the current low is a slave's, not the master's
        self.last_rise = 0
        self.prev_rise = 0         # the rise before the current fall
        self.reset_release = -1
        self.presence_start = 0
        # ---- the open slot ----
        self.slot_open = False
        self.slot_start = 0
        self.slot_low = 0
        self.slot_read = False
        self.slot_bit = 1
        self.slot_has_second = False
        self.pending_period = None   # (slot start, next fall) awaiting a tSLOT check
        # ---- the byte being decoded ----
        self.grp = "none"
        self.grp_dir: Optional[str] = None
        self.grp_left = 0
        self.grp_bytes: list[int] = []
        self.bits = 0
        self.value = 0
        self.first_slot_pending = False
        self.plan: list[tuple] = []   # the groups the current transaction owes

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def rules_seen(self) -> set:
        return {v.rule for v in self.violations}

    def report(self, limit: int = 10) -> str:
        head = (f"{self.speed} speed at {self.ns_per_tick} ns/tick: "
                f"{self.resets} reset(s), {self.presences} presence pulse(s), "
                f"{self.slots} slot(s) ({self.write_slots} written, "
                f"{self.read_slots} read), {self.bytes} byte(s), "
                f"{self.crc_checks} CRC check(s), {len(self.violations)} violation(s)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, bound: str, measured=None,
                   required=None, detail: str = "") -> None:
        self.violations.append(Violation(t, rule, bound, measured, required, detail))

    def _window(self, t: int, rule: str, measured: int, window,
                detail: str = "") -> None:
        """Check one measurement against a (min, max) window, in ticks."""
        lo, hi = window
        if lo is not None and measured < lo:
            self._violation(t, rule, "min", measured, lo, detail)
        if hi is not None and measured > hi:
            self._violation(t, rule, "max", measured, hi, detail)

    # ---- the monitor --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        line = pins[self.net]
        if line != self.line:
            if line == 0:
                self.prev_rise = self.last_rise
                self.low_start = t
                self._fall(t)
            else:
                self.last_rise = t
                self._rise(t)
            self.line = line
        return {}

    def finish(self) -> list[Violation]:
        """Close the open slot and judge anything the last edge could not. Idempotent."""
        if self._finished:
            return self.violations
        self._finished = True
        self._bank_bit()
        if self.phase == "await_presence":
            self._violation(self.reset_release, "presence", "framing",
                            detail=f"no presence pulse within {self.t_pdhigh_max} "
                                   f"ticks of the reset release")
        self.phase = "finished"
        return self.violations
# ---- edges ----------------------------------------------------------------------
    def _fall(self, t: int) -> None:
        """The line fell. Almost nothing is decided here: a low that lasts tRSTL is a
        reset and no time slot can be that long, so the length has to be measured
        first. The two exceptions are a fall inside the open slot that comes too
        early to be the next slot -- that can only be a slave answering a read --
        and the presence pulse, whose position after the reset release is itself a
        rule (tPDHIGH)."""
        self.low_start, self.low_fall = t, t
        self.low_is_drive = False
        if self.phase == "await_presence":
            waited = t - self.reset_release
            if self.t_pdhigh_max is None or waited <= self.t_pdhigh_max:
                self.presences += 1
                self.presence_start = t
                self.phase = "presence_low"
                self._window(t, "tPDHIGH", waited, self.min["t_pdhigh"],
                             "reset release to the presence pulse")
                return
            # The presence pulse is late. Both facts are reported: nothing answered
            # the reset, and the pulse that did arrive was outside tPDHIGH.
            self._violation(t, "presence", "framing",
                            detail=f"no presence pulse within {self.t_pdhigh_max} "
                                   f"ticks of the reset release")
            self._window(t, "tPDHIGH", waited, self.min["t_pdhigh"],
                         "reset release to the presence pulse")
        if self.slot_open and not self.slot_has_second and self.slot_bit \
                and (self.t_slot_min is None or t - self.slot_start < self.t_slot_min):
            self.low_is_drive = True
            if self.grp_dir == "write":
                self._violation(t, "slave-drive", "framing",
                                detail=f"a slave pulled the bus low inside a write "
                                       f"slot that began at t={self.slot_start}")
            if self.t_rdv:
                self._window(t, "tRDV", t - self.slot_start, (self.t_rdv, None),
                             "drive inside a read slot")
            self.slot_bit = 0
            self.slot_has_second = True
            return
        # A new low. It may still turn out to be a reset, so the open slot's period
        # is banked now but only *checked* once this low is known to be a slot.
        self.pending_period = (self.slot_start, t) if self.slot_open else None
        self._bank_bit()
        if not self.slot_open and self.resets == 0:
            self._violation(t, "no-reset", "framing",
                            detail="a time slot before any reset pulse")
        self.slot_open = True
        self.slot_start, self.slot_low = t, 0
        self.slot_bit, self.slot_has_second = 1, False
        self.slot_read = self.grp_dir == "read"
        self.phase = "slots"
        self.slots += 1
        if self.first_slot_pending and self.reset_release >= 0:
            self._window(t, "tRSTH", t - self.reset_release, (self.t_rsth, None),
                         "reset release to the first slot of the transaction")
        self.first_slot_pending = False

    def _rise(self, t: int) -> None:
        """The line rose. Its length is what says which signal just ended."""
        low = t - self.low_start
        self.low_len = low
        if self.low_is_drive:
            # A slave finished driving a read-0 inside the open slot.
            if self.t_tx0_min is not None:
                self._window(t, "tTX0", t - self.slot_start, self.min["t_tx0"],
                             "read-0 ends this long after its slot began")
            return
        if self.phase == "presence_low":
            # The presence pulse, checked before anything else: a presence pulse can
            # be longer than the shortest reset (120 us against 68 us at overdrive
            # speed), so it is the phase that says what this low was, not its length.
            self.slot_open = False               # the presence pulse is not a slot
            self.phase = "slots"
            self._window(t, "tPDLOW", low, self.min["t_pdlow"], "presence pulse length")
            return
        if low >= self.t_rstl_min:
            # A reset pulse: the slot this fall opened was not a slot at all, and a
            # reset that started before the checker was watching is not a slot
            # before any reset either.
            self._drop_reset_fall_violations()
            self.slot_open, self.pending_period = False, None
            self.slots -= 1              # the fall that opened it was not a slot
            self.resets += 1
            self.reset_release = t
            self.phase = "await_presence"
            self._window(t, "tRSTL", low, self.min["t_rstl"], "reset pulse")
            self._new_transaction()
            self.first_slot_pending = True
            return
        if self.t_slot_max is not None and low > self.t_slot_max:
            # This low is too long to be a time slot (tSLOT) and was not a reset
            # either, because it is shorter than tRSTL. Both readings are wrong and
            # the checker says so twice, which is the only way a passive monitor can
            # catch a reset pulse that is too short: on the wire, a short reset and
            # a long slot are the same signal.
            detail = "low of %d ticks: too long for a time slot, too short for a " \
                     "reset" % low
            self._violation(t, "tSLOT", "max", low, self.t_slot_max, detail)
            self._violation(t, "tRSTL", "min", low, self.t_rstl_min, detail)
        self._master_low_ended(t, low)
        if self.pending_period:
            start, end = self.pending_period
            self.pending_period = None
            self._window(end, "tSLOT", end - start, self.min["t_slot"],
                         "time slot period")
            self._window(end, "tREC", end - self.prev_rise, (self.t_rec, None),
                         "recovery time between slots")

    def _drop_reset_fall_violations(self) -> None:
        """The fall that opened the last "slot" was a reset, so anything it
        triggered is about a signal that is not a time slot at all."""
        while self.violations and self.violations[-1].tick == self.low_fall \
                and self.violations[-1].rule in ("tRSTH", "no-reset"):
            self.violations.pop()

    # ---- slots ---------------------------------------------------------------------
    def _new_transaction(self) -> None:
        """A reset pulse ended: the next byte on the wire is a ROM command."""
        self.slot_open = False
        self.grp, self.grp_dir, self.grp_left = "rom_command", "write", 1
        self.grp_bytes, self.bits, self.value = [], 0, 0
        self.plan = []

    def _master_low_ended(self, t: int, low: int) -> None:
        """The master's pulse inside the open slot ended, at pin time `t`, `low` long.

        A long low is a write-0 (the master held the line down for the whole slot);
        a short one is a write-1 or a read slot, and which of the two it is comes
        from the command the checker decoded. `split` is the midpoint of the gap
        between tLOW1's maximum and tLOW0's minimum, so an override that moves
        either window moves the split with it.
        """
        self.slot_low = low
        if self.slot_read:
            self.read_slots += 1
            self._window(t, "tLOW1", low, self.min["t_read_low"],
                         "master low inside a read slot")
            return
        self.write_slots += 1
        if low >= self.split:
            self.slot_bit = 0                      # the master held it low: a zero
            self._window(t, "tLOW0", low, self.min["t_low0"],
                         "master low inside a write-0 slot")
        else:
            self._window(t, "tLOW1", low, self.min["t_low1"],
                         "master low inside a write-1 slot")

    def _bank_bit(self) -> None:
        """The open slot is over: bank its bit and finish any byte it completed."""
        if not self.slot_open or self.grp_left <= 0:
            return
        self.value |= self.slot_bit << self.bits
        self.bits += 1
        if self.bits < 8:
            return
        value, self.value, self.bits = self.value, 0, 0
        self.bytes += 1
        self.grp_bytes.append(value)
        self.grp_left -= 1
        self.transfers.append(ByteTransfer(self.slot_start, value, self.grp,
                                           "read" if self.grp_dir == "read" else "write"))
        if self.grp_left == 0:
            self._group_done(value)

    # ---- the group decoder ----------------------------------------------------------
    def _group_done(self, value: int) -> None:
        """Eight slots are one byte of a group: check the CRC, then work out what
        the next group is from the ROM and function commands. The tick reported is
        the start of the byte's last slot."""
        name = self.grp
        t = self.slot_start
        if name in ("rom", "scratchpad"):
            # "the CRC is calculated from the first 56 bits of the ROM code" and
            # "byte 8 ... contains the CRC code for bytes 0 through 7": in both cases
            # the CRC is over every byte but the last one.
            self.crc_checks += 1
            data = bytes(self.grp_bytes[:-1])
            if crc8(data) != self.grp_bytes[-1]:
                self._violation(self.slot_start, "CRC", "framing",
                                detail=f"{name}: 0x{crc8(data):02x} computed, "
                                       f"0x{self.grp_bytes[-1]:02x} on the wire")
            self.plan = [("function", "write")] if name == "rom" else []
            self._take_plan()
            return
        if name == "rom_command":
            if value not in ROM_COMMANDS:
                self._violation(self.slot_start, "rom-command", "framing",
                                detail=f"0x{value:02x} is not one of "
                                       + ", ".join(f"{k:02x}h" for k in sorted(ROM_COMMANDS)))
                self.plan = []
                self._take_plan()
                return
            if value in (0x33, 0x55):
                self.plan = [("rom", "read" if value == 0x33 else "write")]
            elif value == 0xCC:
                self.plan = [("function", "write")]
            else:
                # Search ROM or Alarm Search: 64 branch bits out, 64 bits back. The
                # answer is not a ROM code -- a device that is not on the branch
                # transmits 1s -- so it gets its own group and no CRC check.
                self.plan = [("search", "write"), ("search_rom", "read")]
            self._take_plan()
            return
        if name == "function":
            if value not in FUNCTION_COMMANDS:
                self._violation(self.slot_start, "rom-command", "framing",
                                detail=f"0x{value:02x} is not a DS18B20 function "
                                       "command, and no ROM command preceded it")
            elif value == 0xBE:
                self.plan = [("scratchpad", "read")]     # 9 bytes out
            elif value == 0x4E:
                self.plan = [("scratchpad_write", "write")]   # TH, TL, config in
            self._take_plan()
            return
        if name != "search":
            # The 64 branch bits of a Search ROM are followed by the 64 answer bits
            # already in the plan; any other group ends the transaction.
            self.plan = []
        self._take_plan()

    def _take_plan(self) -> None:
        """Start the next group the transaction owes, if there is one."""
        if not self.plan:
            # Nothing is expected and the direction of what follows is not known
            # either -- a status bit after Convert T is one slot, not a byte, so the
            # checker cannot line it up with a group. An unknown direction is not a
            # *write* direction: the slave-drive rule must not fire on a slot the
            # checker has no reason to believe the master is writing.
            self.grp, self.grp_dir, self.grp_left = "none", None, 0
            self.grp_bytes = []
            return
        name, direction = self.plan.pop(0)
        self.grp, self.grp_dir = name, direction
        self.grp_bytes = []
        self.grp_left = GROUP_BYTES.get(name, 1)


def _ensure_net(bus: Bus, net: str) -> None:
    """Add the open-drain 1-Wire net if it is not there yet."""
    if net not in bus.nets:
        bus.add_net(net)


def attach_master(bus: Bus, **kwargs) -> OneWireMaster:
    """Build an OneWireMaster that takes its timing from `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "dq"))
    master = OneWireMaster(bus=bus, **kwargs)
    bus.attach(master)
    return master


def attach_ds18b20(bus: Bus, **kwargs) -> Ds18b20Target:
    """Build a Ds18b20Target that takes its timing from `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "dq"))
    target = Ds18b20Target(bus=bus, **kwargs)
    bus.attach(target)
    return target


def attach_checker(bus: Bus, **kwargs) -> OneWireTimingChecker:
    """Build a passive OneWireTimingChecker for `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "dq"))
    checker = OneWireTimingChecker(bus=bus, **kwargs)
    bus.attach(checker)
    return checker