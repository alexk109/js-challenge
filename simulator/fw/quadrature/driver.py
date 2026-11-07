"""Host side of the MVP quadrature-decoder firmware: load program.pio, run one engine.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order, the per-engine fields) and
`docs/mvp-isa.md` (the divider, `SMe_INSTR` forced instructions, autopush).  Host accesses go
through `iss.pio_iss.ChipDevice`'s write/read queue, one per tick, so every wait here is the
simulation running -- the same thing a real host does over the SPI port of
`docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                 # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}")                  # p0 = B, p1 = A, p2 = I
    dev = bus.attach(ChipDevice())
    drv = QuadratureDriver(dev, bus)          # engine 0 on p0/p1/p2
    drv.load(); drv.start()
    drv.pump(20_000)                          # run, draining the RX FIFO as it goes
    drv.flush()                               # empty the ISR's partial word
    assert drv.position == 196                # the encoder turned 0.05 turns at 10 000 RPM

The chip core clock is 50 MHz: one `verif.vip.core.Bus` tick at `ns_per_tick = 20`, the
configuration every VIP in verif/vip/ documents in its own docstring.

Two things about this chip that the RP2040 decoder does not have to think about, and that this
module exists to get right:

* **The pin window is the sample.**  `in pins, 3` reads three *contiguous* pins from IN_BASE,
  so the three wires of an ABI interface have to sit next to each other: B at IN_BASE, A one
  above it, I two above.  `Config` checks that, and the sample comes out of the chip as
  `(index_level << 2) | (A << 1) | B`, which is the same packing the reference model uses for
  `ab` and `index_level` (`verif/vip/quadrature.py`), so the two can be compared directly.
* **The chip logs, the host counts.**  The engine pushes ten samples per FIFO word and has no
  idea what a count is: direction, the x1/x2/x4 factor, the impossible-transition check and index
  homing are `HostDecoder`'s job here, exactly as they are `QuadratureDecoder`'s job on the
  model side.  One copy of the Gray rule (`verif.vip.quadrature.step_delta`, imported below) is
  what both sides use.

The RX FIFO is four words deep and the engine is *not* stopped by a full FIFO: the `in pins, 3`
that reaches PUSH_THRESH stalls atomically and re-samples its source when it completes
(`docs/mvp-isa.md` `### Instruction semantics`, IN), so no sample is lost but the chip falls
behind, and if two edges pass while it is stalled the host sees a two-bit jump and reports it
as an impossible transition.  That is what `FDEBUG.RXSTALL` means on this chip, and
`fw/quadrature/test_quadrature.py` `test_a_slow_host_is_reported_not_silently_wrong` is about it.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

from asm.pioasm import Program, assemble
from verif.vip.quadrature import step_delta

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
IMEM_SIZE = 32                 # docs/mvp-isa.md `### Engine state`
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "quadrature"     # the .program name inside program.pio
SAMPLE_BITS = 3               # `in pins, 3`: one sample is three pins
DEFAULT_CHANNEL = "b"         # which line x1/x2 count, as QuadratureDecoder counts it

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF = regmap.txf_addrs()
A_RXF = regmap.rxf_addrs()
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_INSTR_MEM = regmap.imem_bases()    # engine e, word k: A_INSTR_MEM[e] + k
A_SM_BASE = regmap.sm_base()              # SMe_CLKDIV = A_SM_BASE + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

# CTRL fields (docs/mvp-regmap.md `**CTRL**`)
CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100
# FSTAT fields, engine e (bit +e of each field)
FSTAT_TXEMPTY, FSTAT_TXFULL = 0x0100_0000, 0x0001_0000
FSTAT_RXEMPTY, FSTAT_RXFULL = 0x0000_0100, 0x0000_0001
# FDEBUG, engine e
FDEBUG_TXSTALL, FDEBUG_RXOVER = 0x0100_0000, 0x0001_0000
FDEBUG_RXUNDER, FDEBUG_RXSTALL = 0x0000_0100, 0x0000_0001

# Instruction words the host forces (docs/mvp-isa.md `## Encoding`).
PUSH_BLOCK = 0x8020            # PUSH opcode 100, IfFull 0, Block 1, bits 4:0 0
JMP_ALWAYS = 0b000


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (0b000 << 13) | (JMP_ALWAYS << 5) | addr


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def ticks_per_sm_cycle(clkdiv: int) -> float:
    """The average core cycles (ticks) one SM cycle lasts at this CLKDIV."""
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    return integer + ((clkdiv >> 8) & 0xFF) / 256.0


# --------------------------------------------------------------------------------------------
# The program: assemble it, and read its shape back out of the words
# --------------------------------------------------------------------------------------------

def load_program(path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio (asm/pioasm.py) and return its one program."""
    programs = assemble(Path(path).read_text(encoding="utf-8"))
    names = [p.name for p in programs]
    if names != [PROTO_NAME]:
        raise ValueError(f"{path}: expected one .program named {PROTO_NAME}, got {names}")
    return programs[0]


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


def decode_word(word: int) -> dict:
    """Split one 16-bit word into the fields docs/mvp-isa.md `## Encoding` defines.

    The program uses no side-set and no delay, so the delay/side-set field is all delay and the
    split is not interesting here; what the driver needs is the opcode, the destination, the
    MOV source and the bit count.
    """
    return {
        "op": word >> 13,
        "delay": (word >> 8) & 0x1F,
        "hi": (word >> 5) & 0x7,            # bits 7:5: MOV/IN/OUT destination, JMP condition
        "op2": (word >> 3) & 0x3,           # bits 4:3: MOV invert / bit-reverse
        "lo": word & 0x7,                   # bits 2:0: MOV source
        "count": (word & 0x1F) or 32,        # bits 4:0, and 00000 encodes 32
    }


def describe(program: Program, push_thresh: int = 30) -> dict:
    """What the assembled words say the chip does, derived rather than assumed.

    `fw/uart_tx/driver.py` derives its bit time from the words the same way, and for the same
    reason: if program.pio ever changes shape, the numbers this driver puts in the registers
    have to change with it, or the chip is being configured for a program it is not running.
    The labels are the contract:

        start    `mov y, pins`      the baseline
        wrap     `mov x, pins`      the poll, and WRAP_TARGET
        moved    `jmp x!=y, changed` the change test, and WRAP_TOP
        changed  `in pins, N`       the log

    Returns a dict with:

    `sample_bits`    N of that `in pins, N` (3)
    `unchanged`      SM cycles a poll with nothing moving costs: the WRAP_TARGET..WRAP_TOP
                     window, which the PC runs over without a jump (2)
    `changed`        SM cycles one logged sample costs: the same window plus everything from
                     `changed` to the end of the program (5)
    `baseline_word`  the `mov y, pins` that arms the change detector, which the host forces after
                     a forced `push block` to re-prime it
    `sync_word`      the `in pins, N` at `start`: the sample that says where the shaft was
    `push_thresh`    PUSH_THRESH that makes one word hold exactly ten samples (30)
    """
    labels = program.labels
    for name in ("start", "wrap", "moved", "changed"):
        if name not in labels:
            raise ValueError(f"{PROTO_NAME}: program.pio has no `{name}:` label")
    start, wrap, moved, changed = (labels[n] for n in ("start", "wrap", "moved", "changed"))
    if not start < wrap <= moved < changed < len(program.instrs):
        raise ValueError(f"{PROTO_NAME}: labels out of order: {labels}")
    at = lambda a: decode_word(program.instrs[a])                  # noqa: E731
    is_in_pins = lambda s: s["op"] == 0b010 and s["hi"] == 0b000  # noqa: E731
    is_mov_pins = lambda s: s["op"] == 0b101 and s["lo"] == 0b000  # noqa: E731
    want = [
        (start, is_in_pins, "`in pins, N` (the sync) at `start`"),
        (start + 1, is_mov_pins, "`mov ., pins` (the baseline) just after `start`"),
        (wrap, is_mov_pins, "`mov ., pins` at `wrap`"),
        (moved, lambda s: s["op"] == 0b000 and s["hi"] == 0b101, "`jmp x!=y` at `moved`"),
        (changed, is_in_pins, "`in pins, N` at `changed`"),
    ]
    for addr, ok, what in want:
        if not ok(at(addr)):
            raise ValueError(
                f"{PROTO_NAME}: word {addr} is {program.instrs[addr]:#06x}, which is not the "
                f"{what} the driver's timing assumes"
            )
    if at(start)["count"] != at(changed)["count"]:
        raise ValueError(
            f"{PROTO_NAME}: the sync at `start` samples {at(start)['count']} pins and the log at "
            f"`changed` samples {at(changed)['count']}: they must be the same width"
        )
    if program.wrap != moved or program.wrap_target != wrap:
        raise ValueError(
            f"{PROTO_NAME}: .wrap/.wrap_target are {program.wrap}/{program.wrap_target}, but the "
            f"poll loop needs them on `moved`/`wrap` ({moved}/{wrap})"
        )
    bits = at(changed)["count"]
    unchanged = moved - wrap + 1
    tail = len(program.instrs) - changed
    per_word = samples_per_word(bits, push_thresh)
    return {
        "sample_bits": bits,
        "unchanged": unchanged,
        "changed": unchanged + tail,
        "sync_word": program.instrs[start],
        "log_word": program.instrs[changed],
        "baseline_word": program.instrs[start + 1],
        "push_thresh": per_word * bits,
        "samples_per_word": per_word,
    }


def samples_per_word(sample_bits: int, push_thresh: int = 30) -> int:
    """How many samples one FIFO word carries: PUSH_THRESH bits / the sample width.

    AUTOPUSH empties the ISR when `isr_cnt` reaches the threshold, and every sample shifts
    `sample_bits` bits in, so a word holds exactly this many samples and no sample can straddle
    two words (`docs/mvp-isa.md` `### Instruction semantics`, IN).
    """
    if sample_bits not in (1, 2, 3):
        raise ValueError(f"this driver decodes {sample_bits}-bit samples (I, A, B)")
    if not 1 <= push_thresh <= 32:
        raise ValueError(f"PUSH_THRESH {push_thresh} out of range 1..32")
    if push_thresh % sample_bits:
        raise ValueError(
            f"PUSH_THRESH {push_thresh} is not a multiple of the sample width {sample_bits}: a "
            f"word would then hold a fraction of a sample"
        )
    return push_thresh // sample_bits


def sample_positions(program: Program, push_thresh: int = 30) -> list[tuple]:
    """Where each sample of a word sits in it, oldest first, as `(shift, width)` pairs.

    SHIFTCTRL.IN_SHIFTDIR = 0 means the data enters at the LSB and the ISR shifts left, so after
    exactly `samples_per_word` samples the word holds them in bits `3*(n-1) : 0` -- the newest in
    bits 2:0, the oldest in bits 29:27 for three-bit samples -- and bits 31:30 are zero.  Each
    sample's bit 0 is the LSB of its own field, which is the pin at IN_BASE: `sample & 1` is B,
    `sample >> 1 & 1` is A and `sample >> 2` is I.
    """
    shape = describe(program, push_thresh)
    bits, per_word = shape["sample_bits"], shape["samples_per_word"]
    return [(i * bits, bits) for i in reversed(range(per_word))]


def unpack_word(word: int, program: Program, push_thresh: int = 30) -> list[int]:
    """One RX FIFO word as its samples, oldest first, each an `(I << 2) | ab` triple."""
    return [(word >> shift) & ((1 << width) - 1)
            for shift, width in sample_positions(program, push_thresh)]


# --------------------------------------------------------------------------------------------
# The host-side decoder: the part of the model that lives in software here
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Config:
    """One decoder setup: which engine, which three pins, which decoding factor, which divider.

    `x` and `channel` mean exactly what they mean in `verif.vip.quadrature.QuadratureDecoder`:
    x=4 counts every edge, x=2 every edge of `channel`, x=1 that channel's rising edges.  They are
    host-side choices on purpose: the engine pushes the same sample stream either way, and the
    model counts the same wire the same way.
    """
    engine: int = 0
    b_pin: int = 0               # IN_BASE: the sample's bit 0
    a_pin: int = 1               # the sample's bit 1
    i_pin: int = 2               # the sample's bit 2
    x: int = 4
    channel: str = DEFAULT_CHANNEL
    clkdiv: int = 0x0001_0000    # 1.0: the fastest the program can go, and fast enough
    counts_per_rev: int = 4000   # the AS5047P's 4 x 1000 (verif/vip/quadrature.py)
    initial_position: int = 0    # where the counter starts; see HostDecoder

    def __post_init__(self) -> None:
        if self.engine not in (0, 1):
            raise ValueError(f"engine {self.engine} out of range (the MVP has two)")
        if self.x not in (1, 2, 4):
            raise ValueError(f"decoding factor x must be 1, 2 or 4, got {self.x!r}")
        if self.channel not in ("a", "b"):
            raise ValueError(f"channel must be 'a' or 'b', got {self.channel!r}")
        want = {"b": self.b_pin, "a": self.a_pin}
        if not 0 <= self.b_pin < NUM_PINS:
            raise ValueError(f"IN_BASE {self.b_pin} is outside 0..{NUM_PINS - 1}")
        if self.a_pin != self.b_pin + 1 or self.i_pin != self.b_pin + 2:
            raise ValueError(
                f"`in pins, 3` reads three contiguous pins from IN_BASE: with B={self.b_pin} "
                f"this needs A={self.b_pin + 1} and I={self.b_pin + 2}, got A={self.a_pin}, "
                f"I={self.i_pin}"
            )
        if self.b_pin + 3 > NUM_PINS:
            raise ValueError(
                f"pins {self.b_pin}..{self.b_pin + 2} run past the last real pin {NUM_PINS - 1}"
            )
        if want[self.channel] not in (self.a_pin, self.b_pin):
            raise ValueError(f"channel {self.channel!r} is not one of this decoder's lines")

    @property
    def sample_base(self) -> int:
        """The protocol pin that is the sample's bit 0: IN_BASE."""
        return self.b_pin

    def ab_of(self, sample: int) -> int:
        """The A/B pair of one sample, packed as the model packs it: `(a << 1) | b`."""
        return sample & 0b11

    def index_of(self, sample: int) -> int:
        """The index level of one sample."""
        return (sample >> 2) & 0b1


@dataclass
class DecodeError:
    """One sample the Gray cycle cannot explain, at the sample index that broke.

    Two lines moved at once, which no state in the cycle can do (the AS5047P's four states per
    A pulse are one line at a time).  The position does not move, exactly as
    `QuadratureDecoder` refuses to count one, and the word index and sample index say which word
    of the log it came from.
    """
    word: int
    sample: int
    tick: int
    old_ab: int
    new_ab: int

    def __str__(self) -> str:
        return (f"word {self.word} sample {self.sample} (t={self.tick}): impossible A/B "
                f"transition {self.old_ab:02b} -> {self.new_ab:02b}")


@dataclass
class IndexEvent:
    """One index edge as the host decoder saw it, with the position it happened at."""
    tick: int
    level: int
    position: int
    word: int


class HostDecoder:
    """Turns the chip's sample words into a position, a direction and index events.

    This is the half of `verif.vip.quadrature.QuadratureDecoder` that a PIO cannot do: it sees
    every A/B edge and then applies the Gray successor table (`step_delta`, imported from the
    model so there is one copy of it), the x1/x2/x4 factor and the index rule.  The chip's log is
    the wire's own history, so the two decoders see the same sequence of states and must agree
    count for count -- which is what `fw/quadrature/test_quadrature.py` asserts.

    Rules, all of them the model's:

    * the first sample of a stream sets the baseline and counts nothing: there is no edge yet.
    * a sample whose A/B pair equals the previous one is not a step.  It happens when only the
      index moved (`mov x, pins` watches every pin, see program.pio), and it is how the index is
      captured without spending a word on it.
    * a sample that is not a legal successor of the previous one is an impossible transition:
      it is *not* counted, `errors` gets it, and the next legal edge carries on counting.
    * `x=2` counts the edges of `channel`, `x=1` that channel's rising edges, `x=4` every edge.

    `initial_position` is the count the counter starts at, which is how a host says "the shaft is
    here": the model's `RotaryEncoder` starts one index window below zero, so passing
    `encoder.initial_counts` makes the two agree from the first edge.
    """
    name = "host_dec"

    def __init__(self, config: Optional[Config] = None, *, initial_position: Optional[int] = None,
                 program: Optional[Program] = None, push_thresh: int = 30) -> None:
        self.config = config if config is not None else Config()
        self.program = program if program is not None else load_program()
        self.push_thresh = push_thresh
        self.position = (self.config.initial_position if initial_position is None
                         else initial_position)
        self.initial_position = self.position
        self.direction = 0
        self.valid = False
        self.counted = 0             # edges that went into the position
        self.samples = 0             # samples fed in, steps or not
        self.steps = 0               # legal edges seen, counted or not
        self.words = 0
        self.edges: list[tuple] = []     # (tick, ab, index, delta, counted)
        self.repeats: list[tuple] = []   # (word, index, tick, sample) logged with no state change
        self.errors: list[DecodeError] = []
        self.index_events: list[IndexEvent] = []
        self._ab: Optional[int] = None
        self._index: Optional[int] = None
        self._index_changed: Optional[int] = None
        self._word = 0

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        """True while no impossible transition has been seen."""
        return not self.errors

    @property
    def index_count(self) -> int:
        """Rising edges of the index seen so far."""
        return sum(1 for e in self.index_events if e.level == 1)

    @property
    def revolutions(self) -> float:
        return self.position / self.config.counts_per_rev

    def pop_error(self) -> Optional[DecodeError]:
        """Oldest decode error not yet popped, or None."""
        return self.errors.pop(0) if self.errors else None

    def resync(self, position: Optional[int] = None) -> None:
        """Start a new stream: forget the baseline, the direction and the index level.

        `QuadratureDriver.restart()` clears the chip's ISR, so the samples after it are a new
        stream whose first entry is a baseline and not an edge -- exactly what this puts the
        decoder back to.  `position` defaults to where the counter stands, so a restart costs no
        counts; a host that means "the shaft is here" passes it.
        """
        self.position = self.position if position is None else int(position)
        self.initial_position = self.position
        self.direction = 0
        self.valid = False
        self._ab = None
        self._index = None
        self._index_changed = None

    def homed(self) -> Optional[int]:
        """Where the counter stood at the last index rise, or None if it has not seen one.

        This is what a host does with the index: the position at the index edge *is* the
        mechanical angle, so a host that wants absolute counts zeroes its counter there.  The
        model's encoder opens its index window at count 0 (`_index_at()`), so on a wire that
        starts one window early the answer is 0.
        """
        rises = [e for e in self.index_events if e.level == 1]
        return rises[-1].position if rises else None

    # ---- the device -------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        """A `verif.vip.core.Device`, so a HostDecoder can sit on the Bus like any model."""
        return {}

    def feed_word(self, word: int, tick: int = 0) -> list[tuple]:
        """One RX FIFO word: ten samples, oldest first.  Returns the edges it produced."""
        self._word += 1
        out = []
        for i, sample in enumerate(unpack_word(word, self.program, self.push_thresh)):
            out.append(self._on_sample(sample, tick, self._word, i))
        return out

    def _on_sample(self, sample: int, tick: int, word: int, index: int) -> tuple:
        ab, level = self.config.ab_of(sample), self.config.index_of(sample)
        self.samples += 1
        prev, prev_index = self._ab, self._index
        self._ab, self._index = ab, level
        self._index_changed = level if (prev is not None and level != prev_index) else None
        if prev is None:
            return (tick, ab, level, None, False)          # the baseline: not an edge
        if ab == prev:
            # Only the index (or nothing) moved.  program.pio logs a sample whenever *any* pin
            # in the IN window changed, so this is how an index edge is captured; it also catches
            # the rare case where the same state is logged twice (see program.pio's note on the
            # two-cycle gap between the detector's read and the log's read).  Either way the Gray
            # rule says a repeated state is not a step, so the position does not move.
            self.repeats.append((word, index, tick, sample))
            return (tick, ab, level, None, False)
        delta = step_delta(prev, ab)
        self.direction = delta or self.direction
        # The index edge is recorded *after* the position has moved, which is the order
        # `QuadratureDecoder.tick` uses: the encoder raises I on the same edge that crosses count
        # 0, and the model's event carries the position *after* that step.  It is recorded even
        # when the A/B transition turned out to be impossible, because the index really did move.
        if self._index_changed is not None:
            counted = False
            if delta is not None:
                net = "a" if (ab >> 1) != (prev >> 1) else "b"
                net_level = (ab >> (1 if net == "a" else 0)) & 1
                counted = self._counts_this_edge(net, net_level)
                if counted:
                    self.position += delta
                    self.counted += 1
            self.index_events.append(IndexEvent(tick, self._index_changed, self.position, word))
            edge = (tick, ab, level, delta, counted)
            if delta is None:
                self.valid = False
                self.errors.append(DecodeError(word, index, tick, prev, ab))
            else:
                self.steps += 1
                self.valid = self.valid or counted
            self.edges.append(edge)
            return edge
        if delta is None:
            self.valid = False
            self.errors.append(DecodeError(word, index, tick, prev, ab))
            edge = (tick, ab, level, None, False)
            self.edges.append(edge)
            return edge
        self.steps += 1
        net = "a" if (ab >> 1) != (prev >> 1) else "b"
        net_level = (ab >> (1 if net == "a" else 0)) & 1
        counted = self._counts_this_edge(net, net_level)
        if counted:
            self.position += delta
            self.counted += 1
            self.valid = True
        edge = (tick, ab, level, delta, counted)
        self.edges.append(edge)
        return edge

    def _counts_this_edge(self, net: str, level: int) -> bool:
        """Which edges this decoder's `x` counts (the model's rule, verbatim)."""
        if self.config.x == 4:
            return True
        if self.config.x == 2:
            return net == self.config.channel
        return net == self.config.channel and level == 1


# --------------------------------------------------------------------------------------------
# Register values this program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, side_en: int = 0) -> int:
    """EXECCTRL: the wrap window the assembler worked out (.wrap_target / .wrap).

    The program side-sets nothing, so SIDE_EN is 0 and SIDESET_COUNT stays 0: with a side-set
    count of 0 there is never a side-set (`docs/mvp-isa.md` `### Delay/side-set field`).
    """
    return (side_en << 30) | (program.wrap << 12) | (program.wrap_target << 7)


def shiftctrl_value(*, push_thresh: int = 30, in_shiftdir: int = 0, autopush: int = 1) -> int:
    """SHIFTCTRL: shift the ISR left (data enters at the LSB) and autopush at 30 bits.

    `in_shiftdir` is the *value of the field*, not a direction to ask for: bit 18 is
    `IN_SHIFTDIR`, and 1 there means "shift right, the data enters at the MSB"
    (`docs/mvp-regmap.md` `**SMe_SHIFTCTRL**`).  The default here is 0, i.e. shift left, because
    that is what makes the packing exact: after ten samples they fill bits 29:0 of the word,
    newest in bits 2:0, oldest in bits 29:27, and bits 31:30 are zero.  With the reset value 1
    the same ten samples sit in bits 31:2, which is still a fixed layout but two bits further up
    and, worse, *not* what `unpack_word()` reads.

    Bit 18 is also easy to confuse with bit 19, which is `OUT_SHIFTDIR`; the MVP resets both to 1
    (0x000C_0000), so leaving SHIFTCTRL alone gives the wrong direction on the input side.  This
    program never shifts anything out, so `OUT_SHIFTDIR` is left at its reset value.
    """
    if not 1 <= push_thresh <= 32:
        raise ValueError(f"PUSH_THRESH {push_thresh} out of range 1..32")
    if in_shiftdir not in (0, 1):
        raise ValueError(f"IN_SHIFTDIR is one bit, got {in_shiftdir!r}")
    return (push_thresh << 20) | (in_shiftdir << 18) | (autopush << 16)


def pinctrl_value(b_pin: int, *, sample_bits: int = SAMPLE_BITS) -> int:
    """PINCTRL: IN_BASE on the B pin and *nothing else open*.

    No OUT, SET or side-set window: the program writes no pin at all, and this is what makes
    "the chip drives nothing" a register-level fact rather than a promise.  The driver also
    refuses a pin the engine cannot read (`docs/mvp-isa.md `## Pins``: IN_BASE is a 5-bit field
    that wraps mod 32, so pins 18..31 are the phantom ones and read 0).
    """
    if not 0 <= b_pin < NUM_PINS:
        raise ValueError(f"B pin {b_pin} is outside 0..{NUM_PINS - 1} (18..31 are phantom)")
    if b_pin + sample_bits > NUM_PINS:
        raise ValueError(
            f"the sample needs pins {b_pin}..{b_pin + sample_bits - 1} and {NUM_PINS - 1} is "
            f"the last real pin: `in pins, {sample_bits}` would read past the end"
        )
    return b_pin << 15                        # IN_BASE only: SIDESET/SET/OUT bases and counts 0


# --------------------------------------------------------------------------------------------
# The driver: one engine of a ChipDevice as a quadrature decoder
# --------------------------------------------------------------------------------------------

class QuadratureDriver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as an incremental-encoder decoder.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are queued on
    `dev` and land one per tick as the simulation runs, so `read()`, `pump()` and `flush()` are
    the simulation running, not a sleep.  `decoder` is the `HostDecoder` the RX FIFO words are fed
    to; it is public so a test can read `decoder.position` without touching the chip's internals.
    """

    MAX_TICKS = 4_000_000

    @property
    def RXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)

    def __init__(self, dev, bus, *, engine: int = 0, b_pin: int = 0, a_pin: Optional[int] = None,
                 i_pin: Optional[int] = None, x: int = 4, channel: str = DEFAULT_CHANNEL,
                 clkdiv: int = 0x0001_0000, counts_per_rev: int = 4000,
                 initial_position: int = 0, program: Optional[Program] = None,
                 path: Path | str = PROGRAM_PATH, config: Optional[Config] = None,
                 auto_read: bool = True) -> None:
        self.dev, self.bus = dev, bus
        self.program = program if program is not None else load_program(path)
        self.shape = describe(self.program)
        self.config = config if config is not None else Config(
            engine=engine, b_pin=b_pin, a_pin=b_pin + 1 if a_pin is None else a_pin,
            i_pin=b_pin + 2 if i_pin is None else i_pin, x=x, channel=channel, clkdiv=clkdiv,
            counts_per_rev=counts_per_rev, initial_position=initial_position)
        self.engine = self.config.engine
        self.auto_read = auto_read
        self.decoder = HostDecoder(self.config, program=self.program)
        self.words_read = 0
        self._reads: list = []
        self._loaded = False

    # ---- addresses and register values ----
    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    @property
    def start_addr(self) -> int:
        return self.program.labels["start"]

    @property
    def clkdiv(self) -> int:
        return self.config.clkdiv

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value(push_thresh=self.shape["push_thresh"])

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.config.b_pin, sample_bits=self.shape["sample_bits"])

    @property
    def words_used(self) -> int:
        return words_used(self.program)

    # ---- the speed the program can reach, from the words and the divider ----
    @property
    def sm_cycles_per_sample(self) -> int:
        """SM cycles one logged sample costs (5): the `changed` path of program.pio."""
        return self.shape["changed"]

    @property
    def ticks_per_sample(self) -> float:
        """Core cycles between two logged samples at this CLKDIV."""
        return self.sm_cycles_per_sample * ticks_per_sm_cycle(self.clkdiv)

    @property
    def max_sample_rate(self) -> float:
        """Samples per second the chip can log at this CLKDIV (the `changed` path, no stalls)."""
        return CORE_HZ / self.ticks_per_sample

    @property
    def min_step_ticks(self) -> float:
        """The shortest gap between two edges the chip can tell apart, in core cycles.

        The poll grid is the `unchanged` path, so two edges closer together than this fall
        between two reads of the pins and the chip never sees the state in between.  This is the
        number to compare with a profile's `min_edge`.
        """
        return self.shape["unchanged"] * ticks_per_sm_cycle(self.clkdiv)

    @property
    def max_rpm(self, counts_per_rev: int = 4000) -> float:
        """The fastest shaft this configuration can follow, in RPM, at counts_per_rev.

        A shaft at `rpm` turns one count every `60e9 / (rpm * counts_per_rev)` ns, which at
        20 ns per tick is `750000 / rpm` ticks per count -- the same arithmetic the reference
        model uses (`RotaryEncoder.ticks_per_count`).  The binding limit is `min_step_ticks`.
        """
        return 750_000.0 * CORE_HZ / (self.min_step_ticks * counts_per_rev)

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write (it takes effect at the end of its bus access cycle `h`)."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def read(self, addr: int) -> int:
        """Queue one host read and run the bus until its data comes back.

        The value is the register's state in the access cycle, so an `RXFn` pop happens as this
        returns (`docs/mvp-regmap.md` `### Cycle-exact mapping to bus accesses`).
        """
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host read of {addr:#04x} unanswered after {ticks} ticks")
        return int(future.result())

    def _settle(self, extra: int = 2) -> None:
        """Run the bus until this driver's queue is empty, then `extra` more ticks."""
        ticks = 0
        while not self.dev.idle or self._reads:
            self.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host accesses still queued after {ticks} ticks")
        for _ in range(extra):
            self.bus.step()

    # ---- register reads ----
    def ctrl(self) -> int:
        return self.read(A_CTRL)

    def fstat(self) -> int:
        return self.read(A_FSTAT)

    def flevel(self) -> int:
        return self.read(A_FLEVEL)

    def fdebug(self) -> int:
        return self.read(A_FDEBUG)

    def clear_fdebug(self) -> int:
        """Write FDEBUG back to clear the sticky flags (W1C)."""
        value = self.fdebug()
        if value:
            self.write(A_FDEBUG, value)
            self._settle()
        return value

    def padout(self) -> int:
        return self.read(A_DBG_PADOUT)

    def padoe(self) -> int:
        return self.read(A_DBG_PADOE)

    def cfginfo(self) -> int:
        return self.read(A_DBG_CFGINFO)

    def pc(self) -> int:
        return self.read(self.sm_addr(SM_ADDR))

    def rx_level(self) -> int:
        """This engine's RX FIFO occupancy, 0..4, from `FLEVEL` (bits 7:4 RX0, 15:12 RX1)."""
        return (self.flevel() >> (8 * self.engine + 4)) & 0xF

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`) ----
    def load(self) -> None:
        """Steps 1-3: stop the engine, burst-write the program, set the four registers."""
        self.stop()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self._settle()
        self._loaded = True

    def arm(self) -> None:
        """Step 5: force `jmp start` through `SMe_INSTR`, the only way to set the PC.

        The host-forced instruction runs in the very next core cycle whatever the divider and
        `SM_ENABLE` say (`docs/mvp-isa.md` `### EXEC`).  Its side-set applies and its delay is
        ignored; this program uses neither.
        """
        self.write(self.sm_addr(SM_INSTR), jmp_abs(self.start_addr))
        self._settle(2)

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in the same CTRL write.

        The engine's first SM cycle is then two core cycles after this write
        (`docs/mvp-isa.md` `### Clock divider`), and SM_RESTART clears the ISR and isr_cnt, so the
        log starts empty.
        """
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def start(self) -> None:
        """Arm and enable.  Nothing has to be in the FIFO: this program never pulls."""
        if not self._loaded:
            self.load()
        self.arm()
        self.enable()

    def stop(self) -> None:
        """Disable the engine, keeping every register (`docs/mvp-isa.md` `### Enable, restart`)."""
        ctrl = self.read(A_CTRL)
        self.write(A_CTRL, ctrl & ~self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def restart(self) -> None:
        """SM_RESTART + CLKDIV_RESTART, keeping the enable.

        This clears the ISR, so the sample log restarts: the host must then expect the next word
        it reads to be the beginning of a new stream, and `HostDecoder` must be re-seeded (a new
        `HostDecoder`, or `decoder.resync()`).
        """
        ctrl = self.read(A_CTRL)
        keep = self.ctrl_bits(CTRL_SM_ENABLE)
        self.write(A_CTRL, ctrl | (self.ctrl_bits(*self.CTRL_BITS) & ~keep))
        self._settle()

    def flush(self) -> int:
        """Get the ISR's partial word out, as a *full* word.  Ten forced `in pins, 3`.

        AUTOPUSH only sends a word when ten samples have accumulated, so a run that stops with
        three samples in the ISR would lose them.  The obvious fix -- a host-forced
        `push block` -- is wrong here, and the reason is worth stating because it is not obvious:
        the word it produces has three samples in bits 8:0 and nine zero samples above them, and
        the host cannot tell those zeros from real samples of state 00 (ab = 00 is a legal Gray
        state, and the index is low outside its window).  A forced push therefore hands the
        decoder nine samples that could be either.

        Forcing the program's own log instruction (`in pins, 3`, the word at `changed`) exactly
        `samples_per_word` times does not: each one adds three bits to the ISR and the one that
        reaches PUSH_THRESH autopushes a *complete* ten-sample word, whose last samples are copies
        of the state the wires are in at that moment.  Copies of the current state are no-change
        samples, which the Gray rule already says are not steps, so the tail costs the position
        nothing.  With `k` samples in the ISR the push happens on the (10 - k)'th forced
        instruction, so one word always comes out and the ISR is left with the same `k` samples.

        Returns the number of words the host read while doing this (normally 0 or 1: the word
        itself is drained by `_settle`, and `words_read` has it already).
        """
        before = self.words_read
        for _ in range(self.shape["samples_per_word"]):
            self.write(self.sm_addr(SM_INSTR), self.shape["log_word"])
            self._settle(1)
        # The pushed word reaches the host over four ticks: two for `rx_valid` to rise out of
        # the flop that carries it, then the FLEVEL read and the RXF read that follow from it.
        self.pump(8)
        return self.drain() if self.rx_level() else self.words_read - before

    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask

    # ---- running, and draining the RX FIFO as it goes ----
    def step(self) -> None:
        """Advance one tick and service the host side of the drain.

        The drain is the two-step dance `fw/uart_rx/driver.py` uses: `rx_valid` (irq0) says
        *some* engine's RX FIFO has data, which is a two-flop signal two cycles behind the FIFO
        (`docs/mvp-regmap.md` `### tx_ready and rx_valid`), so it is a trigger and never the
        answer.  The answer is a read of `FLEVEL`, and only then is `RXFn` read -- reading it on
        the strength of irq0 alone sets `FDEBUG.RXUNDER` and hands the decoder a word of zeros
        that no encoder ever sent.
        """
        chip = self.dev.chip
        if self.auto_read and not self._reads and chip.irq0:
            self._reads.append((self.dev.read(A_FLEVEL), False))
        self.bus.step()
        while self._reads and self._reads[0][0].done():
            future, is_word = self._reads.pop(0)
            value = int(future.result())
            if is_word:
                self.words_read += 1
                self.decoder.feed_word(value, self.bus.t)
            elif (value >> (8 * self.engine + 4)) & 0xF:
                self._reads.append((self.dev.read(A_RXF[self.engine]), True))

    def pump(self, ticks: int) -> None:
        """Run `ticks` core cycles with the drain following along."""
        for _ in range(ticks):
            self.step()

    def run_until(self, cond, limit: int = MAX_TICKS) -> bool:
        """Step until `cond()` or `limit` ticks have passed; True if `cond()` ended it."""
        ticks = 0
        while ticks < limit:
            if cond():
                return True
            self.step()
            ticks += 1
        return bool(cond())

    def drain(self, limit: int = 4096) -> int:
        """Read every word the chip has already pushed.  Returns how many.

        Only safe once the shaft has stopped or the run has ended: a word that AUTOPUSH has not
        sent yet is not in the FIFO, and `flush()` is what gets it out.
        """
        words = 0
        while self.rx_level() and words < limit:
            self.decoder.feed_word(self.read(A_RXF[self.engine]), self.bus.t)
            self.words_read += 1
            words += 1
        return words

    # ---- what the decoder has made of the log ----
    @property
    def position(self) -> int:
        return self.decoder.position

    @property
    def direction(self) -> int:
        return self.decoder.direction

    @property
    def index_count(self) -> int:
        return self.decoder.index_count

    @property
    def errors(self) -> list:
        return self.decoder.errors

    def describe_run(self) -> str:
        """One line a test or a note can quote: words, samples, position, speed."""
        shape = self.shape
        return (f"{self.words_used} of {IMEM_SIZE} IMEM words, {shape['unchanged']} SM cycles per "
                f"poll and {shape['changed']} per sample, CLKDIV {self.clkdiv:#010x} "
                f"({ticks_per_sm_cycle(self.clkdiv):.3f} core cycles per SM cycle): "
                f"{self.words_read} words, {self.decoder.samples} samples, "
                f"{self.decoder.steps} edges, position {self.decoder.position}, "
                f"{self.decoder.counted} counted, {len(self.decoder.errors)} errors")
