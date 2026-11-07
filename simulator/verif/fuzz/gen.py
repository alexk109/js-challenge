"""Seeded, spec-only stimulus for the RTL/ISS lockstep fuzz.

Everything here is derived from `docs/mvp-isa.md` (`## Encoding`, `## Semantics`, `## Timing`)
and `docs/mvp-regmap.md` (`### Address map`, `## Registers`). Nothing is read out of `src/` or
`iss/`: the harness must not encode an implementation detail, or the fuzz would agree with a bug
that both sides share.

A :class:`Stimulus` is

* a seed, which fixes `pin_in` for every cycle (`pin_vector`), independently of everything else
  so that shrinking the host-op list does not change the pins;
* a list of bus accesses, at most one per core cycle, each ``(cycle, "w", addr, data)`` or
  ``(cycle, "r", addr)``;
* a human-readable summary of what was generated (programs, config, coverage counters), which
  the repro file carries so a bug report says what was running.

It serializes to JSON and back, so a repro is self-contained: ``seed`` + ``ops`` + ``cycles``
replay byte for byte, because the encoder functions and `pin_vector` are versioned with it.
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass, field
from typing import Any, Optional

M32 = 0xFFFF_FFFF
IMEM_SIZE = 32
NUM_ENGINES = 2
NUM_PINS = 18
#: pins the harness can drive from outside (uio 0..7 and input-only 8..12); 13..17 are the chip's
#: own output-only pins, which the chip feeds back itself.
EXT_PINS = 13
UIO_MASK = 0x000FF
IN_ONLY_MASK = 0x01F00
OUT_ONLY_MASK = 0x3E000
PIN_MASK = UIO_MASK | IN_ONLY_MASK | OUT_ONLY_MASK

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_TXF = (0x04, 0x05)
A_RXF = (0x08, 0x09)
A_PADOUT, A_PADOE, A_CFGINFO = 0x0F, 0x10, 0x11
A_IMEM = (0x12, 0x52)          # engine e, word k at A_IMEM[e] + k
A_SM_BASE = 0x32               # SMe_* at 0x32 + 6*e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)
A_INTR = 0x4A
A_IRQ0_INTE, A_IRQ0_INTF, A_IRQ1_INTE, A_IRQ1_INTF = 0x4B, 0x4C, 0x4E, 0x4F

CTRL_SM_ENABLE = 0x3
CTRL_SM_RESTART = 0x30         # bits 5:4
CTRL_CLKDIV_RESTART = 0x300    # bits 9:8
INTE_MASK = 0x33               # bits 5:4 (TXNFULL), 1:0 (RXNEMPTY)

# ---- instruction field values (docs/mvp-isa.md `### Field values`) ----
JMP_ALWAYS, JMP_NOT_X, JMP_DEC_X, JMP_NOT_Y, JMP_DEC_Y, JMP_XNE_Y, JMP_PIN, JMP_NO_OSRE = range(8)
WAIT_GPIO, WAIT_PIN_SRC, WAIT_IRQ, WAIT_RSVD = range(4)
IN_PINS, IN_X, IN_Y, IN_NULL, IN_ISR, IN_OSR = 0, 1, 2, 3, 6, 7
OUT_PINS, OUT_X, OUT_Y, OUT_NULL, OUT_PINDIRS, OUT_PC, OUT_ISR, OUT_EXEC = range(8)
MOV_PINS, MOV_X, MOV_Y, MOV_RSVD_DST, MOV_EXEC, MOV_PC, MOV_ISR, MOV_OSR = range(8)
MOV_NONE, MOV_INVERT, MOV_REVERSE = 0, 1, 2
MOV_SRC_PINS, MOV_SRC_X, MOV_SRC_Y, MOV_SRC_NULL, MOV_SRC_STATUS, MOV_SRC_ISR, MOV_SRC_OSR = (
    0, 1, 2, 3, 5, 6, 7)
SET_PINS, SET_X, SET_Y, SET_PINDIRS = 0, 1, 2, 4


def enc(op: int, ds: int = 0, f75: int = 0, f40: int = 0) -> int:
    """One instruction word from the table in docs/mvp-isa.md `## Encoding`."""
    assert 0 <= op < 8 and 0 <= ds < 32 and 0 <= f75 < 8 and 0 <= f40 < 32
    return (op << 13) | (ds << 8) | (f75 << 5) | f40


def jmp(cond: int, addr: int, ds: int = 0) -> int:
    return enc(0, ds, cond, addr)


def wait_(pol: int, source: int, index: int, ds: int = 0) -> int:
    return enc(1, ds, (pol << 2) | source, index)


def _count(n: int) -> int:
    """Bit count 1..32; `00000` encodes 32 (docs/mvp-isa.md `## Encoding`)."""
    assert 1 <= n <= 32
    return 0 if n == 32 else n


def in_(source: int, count: int, ds: int = 0) -> int:
    return enc(2, ds, source, _count(count))


def out(dst: int, count: int, ds: int = 0) -> int:
    return enc(3, ds, dst, _count(count))


def push(if_full: bool = False, block: bool = False, ds: int = 0) -> int:
    return enc(4, ds, (0 << 2) | (int(if_full) << 1) | int(block))


def pull(if_empty: bool = False, block: bool = False, ds: int = 0) -> int:
    return enc(4, ds, (1 << 2) | (int(if_empty) << 1) | int(block))


def mov(dst: int, op: int, src: int, ds: int = 0) -> int:
    return enc(5, ds, dst, (op << 3) | src)


def irq(ret: bool = False, clr: bool = False, ds: int = 0) -> int:
    return enc(6, ds, (int(ret) << 1) | int(clr))


def set_(dst: int, data: int, ds: int = 0) -> int:
    return enc(7, ds, dst, data)


# The spec's own worked encodings (docs/mvp-isa.md `### Delay/side-set field`): gen_test_asserts
# below checks every encoder against them, so the harness cannot drift from the spec table.
SPEC_WORDS = {
    "nop": 0xA042,                    # mov y, y
    "jmp 0": 0x0000,
    "set pindirs, 1": 0xE081,
    "set pins, 1 [1]": 0xE101,
    "pull block": 0x80A0,
    "pull side 1 [7]": 0x9FA0,        # .side_set 1 opt: SC = 2, SIDE_EN = 1
    "out pins, 1": 0x6001,
    "in pins, 1": 0x4001,
    "wait 0 pin 0": 0x2020,
    "jmp x-- 3 [6]": 0x0643,
    "mov pins, x": 0xA001,
}


def spec_encoders() -> dict[str, int]:
    """The same table, built with this module's encoders."""
    return {
        "nop": mov(MOV_Y, MOV_NONE, MOV_SRC_Y),
        "jmp 0": jmp(JMP_ALWAYS, 0),
        "set pindirs, 1": set_(SET_PINDIRS, 1),
        "set pins, 1 [1]": enc(7, 1, SET_PINS, 1),              # delay 1, no side-set
        "pull block": pull(block=True),
        "pull side 1 [7]": enc(4, 31, 5),                      # SC = 2 opt: side 1, delay 7
        "out pins, 1": out(OUT_PINS, 1),
        "in pins, 1": in_(IN_PINS, 1),
        "wait 0 pin 0": wait_(0, WAIT_PIN_SRC, 0),
        "jmp x-- 3 [6]": enc(0, 6, JMP_DEC_X, 3),
        "mov pins, x": mov(MOV_PINS, MOV_NONE, MOV_SRC_X),
    }


def gen_test_asserts() -> None:
    mine = spec_encoders()
    for name, want in SPEC_WORDS.items():
        got = mine[name]
        assert got == want, f"{name}: encoder gives {got:#06x}, spec says {want:#06x}"


# ---------------------------------------------------------------------------------------
# pin_in
# ---------------------------------------------------------------------------------------

def pin_vector(seed: int, cycle: int) -> int:
    """The value the *world* drives on the input pins in `cycle`, 18 bits.

    Pins 0..12 are the ones a host or board can drive (0..7 `uio`, 8..12 input-only); the
    harness overrides 0..7 with the chip's own driven level wherever the chip drives them, which
    is the pad model of docs/mvp-isa.md `### The 18 protocol pins` ("`uio_in` follows the pad").
    Pins 13..17 are driven anyway (the chip ignores them and feeds its own level back).

    A function of (seed, cycle) only, never of the op list, so that shrinking a repro's host
    ops leaves the pin sequence untouched. Values come in bursts of 8 cycles, with quiet and
    all-equal stretches, so a program that samples its pins sees runs and levels to lock onto.
    """
    r = random.Random((seed * 0x9E3779B1 + cycle * 0x85EBCA6B) & M32)
    burst = random.Random((seed * 0xC2B2AE35 + cycle // 8) & M32)
    word = burst.getrandbits(EXT_PINS)
    roll = r.random()
    if roll >= 0.30:
        word ^= r.getrandbits(EXT_PINS)
    if roll >= 0.85:
        word = EXT_PINS if r.random() < 0.5 else 0
    return (word | (r.getrandbits(5) << EXT_PINS)) & 0x3FFFF


# ---------------------------------------------------------------------------------------
# Stimulus
# ---------------------------------------------------------------------------------------

@dataclass
class Stimulus:
    """A whole run: the seed that fixes the pins, and one bus access per cycle at most.

    `build` records which RTL the run was made against ("rtl", or "planted1"/"planted2" for the
    planted copies), so that replaying a repro does not need to be told again.
    """
    seed: int
    cycles: int
    ops: list[tuple[int, str, int, int]] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)
    kind: str = "fuzz-lockstep-1"
    build: str = "rtl"
    minimized: bool = False
    #: how the run was compared and what the world offered: `{"deep": bool, "pullup": bool}`, or
    #: `{}` for a stimulus that does not record one (the older committed repros), in which case
    #: the harness defaults govern. A repro written by the deep lockstep replays in its own mode, so
    #: a deep finding cannot come back clean.
    mode: dict[str, bool] = field(default_factory=dict)
    #: pins the outside world holds at 0 whatever `pin_vector` says, like pull-downs on a board.
    #: Only the stimuli that *read* pins need it; the random ones leave it at 0.
    ext_low: int = 0

    def op_at(self, cycle: int) -> Optional[tuple[str, int, int]]:
        for c, kind, addr, data in self.ops:
            if c == cycle:
                return kind, addr, data
            if c > cycle:
                break
        return None

    def to_json(self) -> str:
        return json.dumps({
            "kind": self.kind,
            "build": self.build,
            "minimized": self.minimized,
            "seed": self.seed,
            "cycles": self.cycles,
            "ops": [list(o) for o in self.ops],
            "summary": self.summary,
            "mode": self.mode,
            "ext_low": self.ext_low,
        }, indent=1, sort_keys=False)

    @staticmethod
    def from_json(text: str) -> "Stimulus":
        d = json.loads(text)
        return Stimulus(seed=int(d["seed"]), cycles=int(d["cycles"]),
                        ops=[tuple(o) for o in d["ops"]],
                        summary=d.get("summary", {}), kind=d.get("kind", "fuzz-lockstep-1"),
                        build=d.get("build", "rtl"), minimized=d.get("minimized", False),
                        mode=d.get("mode", {}),
                        ext_low=int(d.get("ext_low", 0)))

    def save(self, path) -> None:
        with open(path, "w") as fh:
            fh.write(self.to_json() + "\n")

    @staticmethod
    def load(path) -> "Stimulus":
        with open(path) as fh:
            return Stimulus.from_json(fh.read())

    def copy(self, cycles: Optional[int] = None, ops=None) -> "Stimulus":
        return Stimulus(seed=self.seed,
                        cycles=self.cycles if cycles is None else cycles,
                        ops=list(self.ops if ops is None else ops),
                        summary=dict(self.summary), kind=self.kind, build=self.build,
                        mode=dict(self.mode), ext_low=self.ext_low)


class Sched:
    """Hands out one bus access per core cycle, in order."""

    def __init__(self, start: int = 0) -> None:
        self.cursor = start
        self.ops: list[tuple[int, str, int, int]] = []

    def _next(self) -> int:
        self.cursor += 1
        return self.cursor

    def write(self, addr: int, data: int) -> int:
        c = self._next()
        self.ops.append((c, "w", addr, data & M32))
        return c

    def read(self, addr: int) -> int:
        c = self._next()
        self.ops.append((c, "r", addr, 0))
        return c


# ---------------------------------------------------------------------------------------
# program generation
# ---------------------------------------------------------------------------------------

class SideCfg:
    """A program's side-set configuration, and the delay/side-set field of its instructions.

    docs/mvp-isa.md `### Delay/side-set field, bits 12:8`: the top `SC` bits are side-set, the
    low `5 - SC` are the delay, so the delay range and the side-set width both follow the
    engine's SIDESET_COUNT (0..5, with 6 and 7 behaving as 5).
    """

    def __init__(self, count: int, side_en: bool, pindir: bool, rng: random.Random) -> None:
        self.count = count            # as stored in PINCTRL.SIDESET_COUNT
        self.sc = min(count, 5)       # 6 and 7 behave as 5
        self.side_en = side_en
        self.pindir = pindir
        self.rng = rng

    def describe(self) -> dict[str, Any]:
        return {"sideset_count": self.count, "side_en": int(self.side_en),
                "side_pindir": int(self.pindir)}

    def ds(self, side: Optional[int] = None, delay: Optional[int] = None) -> int:
        """The 5-bit delay/side-set field for one instruction."""
        rng = self.rng
        if delay is None:
            delay = rng.randrange(1 << (5 - self.sc))
        if self.sc == 0:
            return delay
        if side is None:
            if self.side_en:
                data = rng.randrange(1 << (self.sc - 1))
                enable = rng.random() < 0.75          # side-set most, but not all, instructions
                side = ((1 if enable else 0) << (self.sc - 1)) | data
            else:
                side = rng.randrange(1 << self.sc)
        assert delay < (1 << (5 - self.sc)) and side < (1 << self.sc)
        return (side << (5 - self.sc)) | delay


def instr_form(word: int) -> int:
    """Which form of an instruction a word is: opcode, bits 7:5 and bits 4:0.

    The delay/side-set field (bits 12:8) is configuration, not form, so it is dropped; and
    PUSH/PULL ignore bits 4:0 (docs/mvp-isa.md `### Removed and reserved encodings`, item 3), so
    for them the form stops at bit 5.
    """
    return word & 0x1FE0 if (word >> 13) == 4 else word & 0x1FFF


def build_program(bodies: list[int], scfg: SideCfg) -> list[int]:
    """Attach a random delay/side-set field to each body and patch JMP targets into range."""
    rng = scfg.rng
    n = len(bodies)
    words = []
    for body in bodies:
        assert body & 0x1F00 == 0, "bodies carry no delay/side-set field"
        word = body | (scfg.ds() << 8)
        if word >> 13 == 0:                 # JMP: keep the target inside this program
            word = (word & ~0x1F) | rng.randrange(n)
        words.append(word)
    return words


def cover_bodies(rng: random.Random) -> list[int]:
    """Instruction bodies (delay/side-set field = 0) covering the MVP instruction space.

    Every MVP instruction, operand class and reserved form that docs/mvp-isa.md
    `### Removed and reserved encodings` defines as a NOP. `wait 1 irq` is left out here: it
    never completes in the MVP, so it would wedge the engine for the rest of the run. It gets a
    program of its own (:func:`wedge_bodies`), used only occasionally.
    """
    body: list[int] = []
    for cond in range(8):                                    # JMP, all 8 conditions
        body.append(jmp(cond, 0))
    for src in (WAIT_GPIO, WAIT_PIN_SRC, WAIT_IRQ, WAIT_RSVD):   # WAIT, both polarities
        for pol in (0, 1):
            if (src, pol) == (WAIT_IRQ, 1):
                continue
            body.append(wait_(pol, src, rng.randrange(32)))
    for src in (IN_PINS, IN_X, IN_Y, IN_NULL, IN_ISR, IN_OSR):   # IN, every source
        for cnt in (1, 2, 7, 8, 31, 32):
            body.append(in_(src, cnt))
    body += [in_(4, 8), in_(5, 8)]                            # IN reserved sources: NOPs
    for dst in range(8):                                      # OUT, every destination
        for cnt in (1, 8, 32):
            body.append(out(dst, cnt))
    for if_full in (0, 1):                                    # PUSH and PULL, all four forms
        for block in (0, 1):
            body.append(push(bool(if_full), bool(block)))
            body.append(pull(bool(if_full), bool(block)))
    for dst in range(8):                                      # MOV, every destination
        if dst == MOV_RSVD_DST:
            body.append(mov(dst, MOV_NONE, MOV_SRC_X))       # reserved destination: NOP
            continue
        body.append(mov(dst, MOV_NONE, MOV_SRC_PINS))
    for src in (MOV_SRC_PINS, MOV_SRC_X, MOV_SRC_Y, MOV_SRC_NULL, MOV_SRC_STATUS,
                MOV_SRC_ISR, MOV_SRC_OSR):                    # MOV, every source
        for op in (MOV_NONE, MOV_INVERT, MOV_REVERSE):
            body.append(mov(MOV_X, op, src))
    body += [mov(MOV_Y, 3, MOV_SRC_X), mov(MOV_X, MOV_NONE, 4)]   # MOV reserved op/source: NOPs
    for dst in (SET_PINS, SET_X, SET_Y, SET_PINDIRS):         # SET, every destination
        body.append(set_(dst, rng.randrange(32)))
    body += [set_(3, 1), set_(5, 1), set_(6, 1), set_(7, 1)]  # SET reserved: NOPs
    for ret in (0, 1):                                        # IRQ: a NOP carrying side/delay
        for clr in (0, 1):
            body.append(irq(bool(ret), bool(clr)))
    return body


def wedge_bodies(rng: random.Random) -> list[int]:
    """A program that never finishes: `wait 1 irq`, which the MVP defines as stalling forever.

    It only exercises the stall paths if something later un-wedges the engine, so the schedule
    always follows it with a forced instruction or a restart.
    """
    return [wait_(1, WAIT_IRQ, rng.randrange(32)), wait_(1, WAIT_GPIO, rng.randrange(32)),
            jmp(JMP_ALWAYS, 0)]


def random_words(rng: random.Random, n: int) -> list[int]:
    """n random legal instruction words, delay/side-set field included."""
    return [rng.getrandbits(16) for _ in range(n)]


# ---------------------------------------------------------------------------------------
# register setup
# ---------------------------------------------------------------------------------------

def gen_clkdiv(rng: random.Random) -> int:
    """SMe_CLKDIV: INT in 0..65535 (0 means 65536, and is defined with FRAC != 0), FRAC in 0..255."""
    roll = rng.random()
    if roll < 0.25:
        integer = 1
    elif roll < 0.60:
        integer = rng.randrange(2, 9)
    elif roll < 0.75:
        integer = rng.randrange(9, 65)
    elif roll < 0.90:
        integer = 0                                   # 65536, with and without a fraction
    else:
        integer = rng.randrange(65, 65536)
    frac = rng.randrange(256) if rng.random() < 0.35 else 0
    return (integer << 16) | (frac << 8)              # bits 7:0 are reserved


def gen_execctrl(rng: random.Random, wrap: Optional[tuple[int, int]] = None) -> int:
    """SMe_EXECCTRL: side-set mode, JMP_PIN, the wrap window, STATUS."""
    side_en = rng.random() < 0.35
    side_pindir = rng.random() < 0.30
    jmp_pin = rng.randrange(NUM_PINS) if rng.random() < 0.8 else rng.randrange(32)
    if wrap is None:
        bottom = rng.randrange(32)
        top = min(31, bottom + rng.randrange(0, 8))
    else:
        bottom, top = wrap
    return ((int(side_en) << 30) | (int(side_pindir) << 29) | (jmp_pin << 24)
            | (top << 12) | (bottom << 7) | (rng.randrange(2) << 4) | rng.randrange(16))


def gen_shiftctrl(rng: random.Random) -> int:
    """SMe_SHIFTCTRL: thresholds (0 means 32), shift directions, autopush/autopull."""
    pull_thresh = 0 if rng.random() < 0.4 else rng.randrange(1, 33)
    push_thresh = 0 if rng.random() < 0.4 else rng.randrange(1, 33)
    return ((pull_thresh << 25) | (push_thresh << 20) | (rng.randrange(2) << 19)
            | (rng.randrange(2) << 18) | (rng.randrange(2) << 17) | (rng.randrange(2) << 16))


def gen_pinctrl(rng: random.Random, scfg: SideCfg) -> int:
    """SMe_PINCTRL: SIDESET_COUNT (6 and 7 behave as 5), SET/OUT windows and counts, IN_BASE.

    OUT_COUNT above 32 behaves as 32 and SET_COUNT 6 or 7 writes zeros to the extra pins, so
    both are generated at their odd values on purpose. Bases are 5-bit indices into the 32-pin
    virtual space, phantom pins included: they are legal, they just have no effect.
    """
    out_count = 32 if rng.random() < 0.4 else rng.randrange(33)
    if rng.random() < 0.1:
        out_count = rng.randrange(33, 64)              # above 32: behaves as 32
    return ((scfg.count << 29) | (rng.choice([0, 1, 5, 6, 7]) << 26) | (out_count << 20)
            | (rng.randrange(32) << 15) | (rng.randrange(32) << 10) | (rng.randrange(32) << 5)
            | rng.randrange(32))


def gen_config(rng: random.Random) -> dict[str, int]:
    """One engine's whole setup: the five configuration registers, spec-consistent."""
    scfg = SideCfg(rng.choice([0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7]), rng.random() < 0.35,
                   rng.random() < 0.30, rng)
    wrap_bottom = rng.randrange(32)
    wrap = (wrap_bottom, min(31, wrap_bottom + rng.randrange(0, 8)))
    cfg = {
        SM_CLKDIV: gen_clkdiv(rng),
        SM_EXECCTRL: gen_execctrl(rng, wrap),
        SM_SHIFTCTRL: gen_shiftctrl(rng),
        SM_PINCTRL: gen_pinctrl(rng, scfg),
    }
    return {"regs": cfg, "side": scfg, "wrap": wrap}


# ---------------------------------------------------------------------------------------
# the run
# ---------------------------------------------------------------------------------------

#: what a random host access is, with weights (docs/mvp-interfaces.md `## Make targets` lists the
#: traffic a driver needs to exercise: TXF writes, RXF reads, forced SMx_INSTR, IMEM writes while
#: running, SM_RESTART / CLKDIV_RESTART).
HOST_MIX = [
    ("idle", 39),
    ("txf", 11),
    ("rxf", 8),
    ("read_any", 9),      # read a random word address: covers every register's read path,
    ("write_any", 4),     # and a write to a reserved address, which the spec says is ignored
    ("instr", 14),
    ("imem", 5),
    ("smcfg", 5),
    ("irq", 3),
    ("fdebug", 3),
    ("ctrl", 3),
]

#: instructions the host forces through SMe_INSTR (docs/mvp-regmap.md `### Typical host
#: sequences`): set the pins before the program starts, drain a TX FIFO, move the PC.
FORCED = [
    ("set pins", lambda r: set_(SET_PINS, r.randrange(32))),
    ("set pindirs", lambda r: set_(SET_PINDIRS, r.randrange(32))),
    ("out null 32", lambda r: out(OUT_NULL, 32)),
    ("pull noblock", lambda r: pull(block=False)),
    ("pull block", lambda r: pull(block=True)),
    ("push block", lambda r: push(block=True)),
    ("nop", lambda r: mov(MOV_Y, MOV_NONE, MOV_SRC_Y)),
    ("mov x, 0", lambda r: mov(MOV_X, MOV_NONE, MOV_SRC_NULL)),
    ("jmp 0", lambda r: jmp(JMP_ALWAYS, 0)),
    ("wait 1 gpio 0", lambda r: wait_(1, WAIT_GPIO, r.randrange(32))),
    ("random", lambda r: r.getrandbits(16)),
    # The regmap's own advice for a stuck engine (docs/mvp-regmap.md `### EXEC`): "writing a NOP
    # (0xA042) clears a stuck instruction", and `pull noblock`/`out null, 32` drain the TX FIFO.
    # They are repeated in the list on purpose: un-wedging an engine is what lets the rest of the
    # program run at all, and a program stuck on one blocking instruction exercises nothing else.
    ("nop", lambda r: mov(MOV_Y, MOV_NONE, MOV_SRC_Y)),
    ("pull noblock", lambda r: pull(block=False)),
    ("push noblock", lambda r: push(block=False)),
    ("out null 32", lambda r: out(OUT_NULL, 32)),
]


def _weighted(rng: random.Random, mix: list[tuple[str, int]]) -> str:
    total = sum(w for _, w in mix)
    roll = rng.randrange(total)
    for name, w in mix:
        roll -= w
        if roll < 0:
            return name
    return mix[-1][0]


def _programs(rng: random.Random, scfg: SideCfg, wedge: bool) -> tuple[list[list[int]], list[int]]:
    """The IMEM images one engine runs through, in order: coverage first, then random.

    Each image is a full 32-word IMEM (docs/mvp-regmap.md: INSTR_MEM0..31), so the wrap window
    can cover the whole thing. Rewriting IMEM while the engine runs is legal and is how the
    schedule reaches later images.

    The second return value is the list of instruction forms the coverage images ask for, so that
    the harness can report how many of them the run actually executed.
    """
    images: list[list[int]] = []
    cover_forms: list[int] = []
    bodies = cover_bodies(rng)
    while bodies:
        chunk, bodies = bodies[:IMEM_SIZE], bodies[IMEM_SIZE:]
        words = build_program(chunk, scfg)
        cover_forms += [instr_form(w) for w in words]
        while len(words) < IMEM_SIZE:
            words.append(mov(MOV_Y, MOV_NONE, MOV_SRC_Y))       # pad: `nop`
        images.append(words)
    if wedge:
        images.append(build_program(wedge_bodies(rng), scfg))
    for _ in range(rng.randrange(1, 4)):
        images.append(random_words(rng, IMEM_SIZE))
    return images, cover_forms


def gen(seed: int, cycles: int, conflict: int = 0) -> Stimulus:
    """A whole legal run: setup, start, then `cycles` of host traffic and pins.

    `conflict` > 0 appends a guaranteed two-engine pin conflict of that many cycles at the end of
    the run (see :func:`conflict_phase`). It is off by default, so `conflict=0` produces exactly
    the stimulus this function always produced; `metrics/fuzz.csv` and every committed repro stay
    reproducible. The deep campaign turns it on: without it the cross-engine
    pin rule of docs/mvp-isa.md `### Pin writes` is reached on only 16 of the first 20 seeds, and
    plant 5 (engine 0 winning that conflict) escapes on the other 4.
    """
    gen_test_asserts()
    rng = random.Random((seed * 0x27D4EB2F + 0x9E3779B9) & M32)
    s = Sched()
    total = cycles
    # the conflict phase is part of the run, not extra: the random part gives up the cycles
    conflict = max(0, min(conflict, total // 2))
    cycles = total - conflict
    summary: dict[str, Any] = {"engines": [], "host_ops": {}, "notes": []}
    notes = summary["notes"]

    # --- phase 1: both engines off, then setup -------------------------------------------------
    s.write(A_CTRL, 0)
    engines = []
    for e in range(NUM_ENGINES):
        conf = gen_config(rng)
        wedge = rng.random() < 0.15
        images, cover_forms = _programs(rng, conf["side"], wedge)
        engines.append({"conf": conf, "images": images, "gen": 0, "done": False})
        summary["engines"].append({
            "engine": e,
            "registers": {SM_CLKDIV: "CLKDIV", SM_EXECCTRL: "EXECCTRL",
                          SM_SHIFTCTRL: "SHIFTCTRL", SM_PINCTRL: "PINCTRL",
                          SM_INSTR: "INSTR"},
            "config": {k: f"{v:#010x}" for k, v in conf["regs"].items()},
            "wrap": list(conf["wrap"]),
            "side": conf["side"].describe(),
            "images": [["%04x" % w for w in img] for img in images],
            "cover_forms": ["%04x" % f for f in cover_forms],
            "wedged_image": wedge,
        })

    def load_image(e: int, index: int) -> None:
        """Write one 32-word IMEM image of engine `e` (while it runs, which is legal)."""
        eng = engines[e]
        for k, word in enumerate(eng["images"][index]):
            s.write(A_IMEM[e] + k, word)

    for e, eng in enumerate(engines):
        load_image(e, 0)
        for i, value in eng["conf"]["regs"].items():
            s.write(A_SM_BASE + 6 * e + i, value)
        if rng.random() < 0.5:                       # the host can remap the IRQ lines
            s.write(A_IRQ0_INTE, rng.randrange(64) & INTE_MASK)
            s.write(A_IRQ1_INTE, rng.randrange(64) & INTE_MASK)
    notes.append("per-engine PINCTRL.SIDESET_COUNT: "
                 + ", ".join(str(en["conf"]["side"].count) for en in engines))

    # --- phase 2: start both engines ------------------------------------------------------------
    ctrl = CTRL_SM_ENABLE | CTRL_CLKDIV_RESTART
    if rng.random() < 0.5:
        ctrl |= CTRL_SM_RESTART
    s.write(A_CTRL, ctrl)

    # --- phase 3: host traffic -----------------------------------------------------------------
    forced_count = 0
    counts: dict[str, int] = {}
    reserve = 4
    while s.cursor < cycles - reserve:
        what = _weighted(rng, HOST_MIX)
        counts[what] = counts.get(what, 0) + 1
        e = rng.randrange(NUM_ENGINES)
        if what == "idle":
            s.cursor += 1
        elif what == "txf":
            s.write(A_TXF[e], rng.getrandbits(32))
        elif what == "rxf":
            s.read(A_RXF[e])
        elif what == "read_any":
            s.read(rng.randrange(128))
        elif what == "write_any":
            s.write(rng.randrange(128), rng.getrandbits(32))
        elif what == "instr":
            name, build = FORCED[rng.randrange(len(FORCED))]
            forced_count += 1
            s.write(A_SM_BASE + 6 * e + SM_INSTR, build(rng))
            counts["forced:" + name] = counts.get("forced:" + name, 0) + 1
        elif what == "imem":
            k = rng.randrange(IMEM_SIZE)
            s.write(A_IMEM[e] + k, rng.getrandbits(16))
        elif what == "smcfg":
            i = rng.choice([SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_PINCTRL])
            if i == SM_CLKDIV:
                value = gen_clkdiv(rng)
            elif i == SM_EXECCTRL:
                value = gen_execctrl(rng)
            elif i == SM_SHIFTCTRL:
                value = gen_shiftctrl(rng)
            else:
                # a PINCTRL write may change SIDESET_COUNT under the running program
                side = engines[e]["conf"]["side"] if rng.random() < 0.5 else SideCfg(
                    rng.choice([0, 1, 2, 3, 4, 5]), rng.random() < 0.35, rng.random() < 0.3, rng)
                value = gen_pinctrl(rng, side)
            s.write(A_SM_BASE + 6 * e + i, value)
        elif what == "irq":
            if rng.random() < 0.5:
                s.write(A_IRQ0_INTE if rng.random() < 0.5 else A_IRQ1_INTE,
                        rng.randrange(64) & INTE_MASK)
            else:
                s.write(A_IRQ0_INTF if rng.random() < 0.5 else A_IRQ1_INTF,
                        rng.randrange(64) & INTE_MASK)
        elif what == "fdebug":
            s.write(A_FDEBUG, rng.getrandbits(26))
        elif what == "ctrl":
            s.write(A_CTRL, rng.getrandbits(10) & (CTRL_SM_ENABLE | CTRL_SM_RESTART
                                                   | CTRL_CLKDIV_RESTART))
        if s.cursor >= cycles - reserve:
            break
        # A new image for both engines from time to time: IMEM writes while running.
        if rng.random() < 1.0 / 400:
            advance = True
            for other in range(NUM_ENGINES):
                eng = engines[other]
                if eng["gen"] + 1 < len(eng["images"]):
                    eng["gen"] += 1
                    load_image(other, eng["gen"])
                elif not eng["done"]:
                    eng["done"] = True
                else:
                    advance = False
            if advance and rng.random() < 0.5:
                s.write(A_CTRL, (CTRL_SM_ENABLE | rng.getrandbits(2)
                                 | (CTRL_SM_RESTART if rng.random() < 0.7 else 0)
                                 | (CTRL_CLKDIV_RESTART if rng.random() < 0.7 else 0)))
            counts["image_advance"] = counts.get("image_advance", 0) + 1

    while s.cursor < cycles - reserve:
        s.cursor += 1
    summary["host_ops"] = dict(sorted(counts.items()))
    summary["forced_instructions"] = forced_count
    summary["notes"].append(f"pins: external value per cycle from pin_vector(seed={seed}); "
                            "the harness forces pin_in[k] = pin_out[k] while the chip drives "
                            "uio pin k")
    if conflict > 0:
        conflict_phase(s, summary, conflict)
    summary["ops"] = len(s.ops)
    stim = Stimulus(seed=seed, cycles=total, ops=s.ops, summary=summary)
    return stim


# ---------------------------------------------------------------------------------------
# the guaranteed pin conflict, for the cross-engine pin rule
# ---------------------------------------------------------------------------------------

#: PINCTRL for the conflict phase: SIDESET_COUNT 1 (side-set to pin 2), SET_COUNT 1, OUT_COUNT 1,
#: SIDESET_BASE 2, SET_BASE and OUT_BASE 0. The bases decide *which* pins are contested, so both
#: engines get the same one; the data each engine writes decides *whether* they disagree.
CONFLICT_PINCTRL = (1 << 29) | (1 << 26) | (1 << 20) | (2 << 10)
CONFLICT_EXECCTRL = 5 << 12                  # wrap 0..5, no side-set enable, no side-set pindir

#: the two halves of the conflict, one instruction per PC: `set pins 0/1` (a level write to pin 0
#: through the SET window), `set pindirs 1/0` (a direction write to the same pin) and a jump back.
#: Engine 0 and engine 1 run the same shape with every data bit and every side-set bit flipped, so
#: the two engines ask for different things to the same pin in the same cycle, every cycle of the
#: loop, and the rule that decides it (docs/mvp-isa.md `### Pin writes`, "engine 1 wins over
#: engine 0") is the only thing that can tell them apart.
CONFLICT_BODIES = (
    [set_(SET_PINS, 0), set_(SET_PINS, 1), set_(SET_PINDIRS, 1), jmp(JMP_ALWAYS, 0)],
    [set_(SET_PINS, 1), set_(SET_PINS, 0), set_(SET_PINDIRS, 0), jmp(JMP_ALWAYS, 0)],
)
#: the side-set data that rides along on each of those four instructions, per engine
CONFLICT_SIDE = ((0, 1, 0, 0), (1, 0, 1, 1))


def conflict_phase(s: Sched, summary: dict[str, Any], cycles: int) -> None:
    """Make both engines write the same pin with different values, for `cycles` cycles.

    The random programs of a seed reach the cross-engine pin rule only when their clock dividers
    happen to line up (measured on seeds 1-20 at 10k cycles: 16 of 20, and seeds 2, 7, 12 and 17
    never), so a bug in that rule can hide in a run that reports itself clean. This phase removes
    the luck: both engines get divide-by-1, a four-instruction loop that only writes pins, and a
    PINCTRL that puts both windows on the same pin. It runs at the end of the run, so nothing it
    writes has to be restored.
    """
    for e in range(NUM_ENGINES):
        program = [word | (CONFLICT_SIDE[e][i] << 12) for i, word in enumerate(CONFLICT_BODIES[e])]
        program += [NOP] * (IMEM_SIZE - len(program))
        for k, word in enumerate(program):
            s.write(A_IMEM[e] + k, word)
        s.write(A_SM_BASE + 6 * e + SM_CLKDIV, 1 << 16)             # 1.0, every cycle
        s.write(A_SM_BASE + 6 * e + SM_EXECCTRL, CONFLICT_EXECCTRL)  # wrap 0..5
        s.write(A_SM_BASE + 6 * e + SM_SHIFTCTRL, 0)                 # no autopush, no autopull
        s.write(A_SM_BASE + 6 * e + SM_PINCTRL, CONFLICT_PINCTRL)
    s.write(A_CTRL, CTRL_SM_ENABLE | CTRL_CLKDIV_RESTART | CTRL_SM_RESTART)
    end = s.cursor + cycles                                       # the phase's own cycles
    while s.cursor < end:
        s.cursor += 1
    summary["notes"].append(f"pin conflict phase: {cycles} cycles, both engines divide by 1 and "
                            "write pin 0 with opposite data and opposite directions")
    summary["host_ops"]["conflict_phase"] = cycles


# ---------------------------------------------------------------------------------------
# the FIFO-fill stimulus: the FIFO storage compare
# ---------------------------------------------------------------------------------------

def fifofill(seed: int = 0, cycles: int = 120) -> Stimulus:
    """A run that fills both TX FIFOs and never empties them, so that the *contents* are visible.

    The FIFO levels of a random run say how full a FIFO is, never what is in it: words come out
    of the FIFO straight into OSR, so a word that is stored wrongly shows up later as a wrong pin
    or a wrong register, if the run lives long enough to reach it. This one fills each TX FIFO
    with four words (plus a fifth, which is the write to a full FIFO that the spec says is
    dropped), and the program never pulls or shifts anything out, so the words stay in the storage
    for the whole run. A fault in the storage therefore has nowhere to hide: what is stored is the
    only thing that can show it. `verif/fuzz/planted/` plant 7 is exactly such a fault.

    It also carries what the other deep negative controls need, so that one short run can exercise
    all of them: a fractional clock divider (the divider accumulator and its carry), an `OUT PINS`
    (a pin write request), a PINCTRL the program does not use (a configuration register), and 31
    IMEM words of which only the first two are ever fetched.
    """
    prog = [
        out(OUT_PINS, 1, 0),                          # 0  one pin write request per loop
        jmp(JMP_ALWAYS, 0),                           # 1
    ]

    def image(words: list[int]) -> list[int]:
        return words + [NOP] * (IMEM_SIZE - len(words))

    # INT = 3 with FRAC = 0x41: the divider reloads every third cycle and the accumulator carries
    # every second one, so div_cnt, div_acc and the SM cadence all move
    conf = {SM_CLKDIV: (3 << 16) | (0x41 << 8), SM_EXECCTRL: 0, SM_SHIFTCTRL: 0,
            SM_PINCTRL: (1 << 26) | (1 << 20)}         # SET and OUT windows of one pin on pin 0
    s = Sched()
    s.write(A_CTRL, 0)
    for e in range(NUM_ENGINES):
        for k, word in enumerate(image(prog)):
            s.write(A_IMEM[e] + k, word)
        for i, value in conf.items():
            s.write(A_SM_BASE + 6 * e + i, value)
    s.write(A_CTRL, CTRL_SM_ENABLE | CTRL_CLKDIV_RESTART)
    for e in range(NUM_ENGINES):                       # four words each, then one too many
        for k in range(4):
            s.write(A_TXF[e], 0x1000_0000 | (e << 20) | (k << 8) | 0x5A)
        s.write(A_TXF[e], 0xDEAD_BEEF)                # TXOVER: the spec drops this one
    s.read(A_FLEVEL)
    s.read(A_FSTAT)
    while s.cursor < cycles - 4:
        s.cursor += 1
    summary = {
        "engines": [{"engine": e, "program": ["%04x" % w for w in prog], "config": conf,
                     "wrap": [0, 1],
                     "side": {"sideset_count": 0, "side_en": 0, "side_pindir": 0}}
                    for e in range(NUM_ENGINES)],
        "host_ops": {"fifofill": 8},
        "notes": ["the FIFO storage stimulus: fixed, no random stimulus",
                  "four words in each TX FIFO and a fifth that the FIFO is full to refuse",
                  "the program never pulls or shifts, so the words stay in the storage"],
    }
    return Stimulus(seed=seed, cycles=cycles, ops=s.ops, summary=summary)


# ---------------------------------------------------------------------------------------
# the pin-loopback stimulus: the pad model and the pull-up mode
# ---------------------------------------------------------------------------------------

def pinloop(seed: int = 0, cycles: int = 300) -> Stimulus:
    """A run that reads back the pins, so that the harness's pad model is not unobservable.

    Everything else the lockstep compares is a function of the chip alone; the pad model
    (`lockstep.pad_value`) is a function of the *world* and of the chip's own driven level, so a
    program that only ever reads a pin nobody drives cannot tell a right pad model from a wrong
    one. This one reads both kinds:

    * engine 0 drives pin 0 high and holds pin 1 as an output (side-set), so `pin_in[0:2]` has to
      follow the chip's own level and direction -- the rule of docs/mvp-isa.md
      `### The 18 protocol pins` ("`uio_in` follows the pad"), and what `FUZZ_PAD_FAULT` (a pad
      model that forgets it) breaks;
    * engine 1 reads pin 0 (the driven one, so it must see what engine 0 drives), reads pin 2 (a
      `uio` pin nobody drives: 1 under `--pullup`, 0 without it, because the world holds every pin
      it can drive low here), and moves both into X and Y, so the two are compared as architectural
      state and not only as a coverage counter.
    """
    prog0 = [
        set_(SET_PINDIRS, 0b01),                       # 0  pin 0 becomes an output
        enc(7, 0, SET_PINS, 0b01),                      # 1  pin 0 high; the world holds it low
        jmp(JMP_ALWAYS, 0),                             # 2  ... forever, so the pin is steady
    ]
    prog1 = [
        wait_(1, WAIT_GPIO, 0),                         # 0  until pin 0 (engine 0 drives it) is high
        mov(MOV_X, MOV_NONE, MOV_SRC_PINS),             # 1  X = the pad: bit 0 from engine 0,
                                                         #     bit 2 from the board or its pull-up
        jmp(JMP_ALWAYS, 0),                             # 2
    ]

    def image(words: list[int]) -> list[int]:
        return words + [NOP] * (IMEM_SIZE - len(words))

    conf0 = {SM_CLKDIV: 1 << 16, SM_EXECCTRL: 1 << 12,  # wrap 0..1: the two SETs and nothing else
             SM_SHIFTCTRL: 0,
             SM_PINCTRL: (1 << 26) | (1 << 20)}         # SET window on pin 0, one pin wide
    conf1 = {SM_CLKDIV: 1 << 16, SM_EXECCTRL: 2 << 12,  # wrap 0..2
             SM_SHIFTCTRL: 0, SM_PINCTRL: 0}
    s = Sched()
    s.write(A_CTRL, 0)
    for e, (prog, conf) in enumerate([(prog0, conf0), (prog1, conf1)]):
        for k, word in enumerate(image(prog)):
            s.write(A_IMEM[e] + k, word)
        for i, value in conf.items():
            s.write(A_SM_BASE + 6 * e + i, value)
    s.write(A_CTRL, CTRL_SM_ENABLE | CTRL_CLKDIV_RESTART)
    # the world holds every pin it can drive low, whatever the seed says, so that a pad model
    # which ignored the chip's own driven level would read 0 where the chip drives 1
    while s.cursor < cycles - 4:
        s.cursor += 1
    summary = {
        "engines": [
            {"engine": 0, "program": ["%04x" % w for w in prog0], "config": conf0,
             "wrap": [0, 2], "side": {"sideset_count": 0, "side_en": 0, "side_pindir": 0}},
            {"engine": 1, "program": ["%04x" % w for w in prog1], "config": conf1,
             "wrap": [0, 2], "side": {"sideset_count": 0, "side_en": 0, "side_pindir": 0}},
        ],
        "host_ops": {"pinloop": 1},
        "notes": ["the pad-model stimulus: fixed, no random stimulus",
                  "engine 0 drives pin 0 high as an output; the world holds every pin it can "
                  "drive low",
                  "engine 1 waits for pin 0 to read high and then reads the pad into X"],
    }
    return Stimulus(seed=seed, cycles=cycles, ops=s.ops, summary=summary, ext_low=PIN_MASK)


# ---------------------------------------------------------------------------------------
# the trivial stimulus, for `make fuzz-selftest`
# ---------------------------------------------------------------------------------------

#: `nop`, the one encoding the spec tells the host to write to clear a stuck instruction
NOP = 0xA042


def trivial(seed: int = 0, cycles: int = 300) -> Stimulus:
    """A short hand-written run with no random stimulus in it.

    Selftest check (a) runs this against the real RTL and the real ISS and requires a match,
    while (b) and (c) run the *same* stimulus against a planted RTL bug and a planted ISS bug and
    require a mismatch. So it has to touch, on both sides, whatever the plants touch: the wrap
    window (`pc_update` step 3), `jmp x--` (which always writes X-1), side-set with an enable
    bit, a stall (blocking PULL) and a pin conflict between the two engines.
    """
    prog0 = [
        set_(SET_PINDIRS, 1),                                  # 0
        enc(7, 12, SET_PINS, 1),                               # 1  side 1 (enabled)
        set_(SET_X, 2),                                        # 2
        enc(0, 0, JMP_DEC_X, 4),                               # 3  jmp x-- 4: decrements X
        pull(block=True),                                      # 4  stalls until the host writes TXF0
        out(OUT_PINS, 1, 9),                                   # 5  side 1, delay 1
    ]
    # Engine 1 writes pin 0 the other way round, so both the level and the direction register of
    # pin 0 are contested every cycle and the resolution order (docs/mvp-isa.md `### Pin writes`,
    # engine 1 wins) decides pin_out[0] and pin_oe[0].
    prog1 = [
        set_(SET_PINDIRS, 0),                                  # 0  engine 1 releases pin 0
        enc(7, 12, SET_PINS, 0),                               # 1  and drives it low; side 1 enabled
        jmp(JMP_ALWAYS, 1),                                    # 2
    ]

    def image(words: list[int]) -> list[int]:
        return words + [NOP] * (IMEM_SIZE - len(words))

    conf0 = {
        SM_CLKDIV: (1 << 16),                                              # CLKDIV = 1.0
        SM_EXECCTRL: (1 << 30) | (1 << 29) | (5 << 12) | (0 << 7) | 2,     # side opt + pindir,
        SM_SHIFTCTRL: (8 << 25) | (8 << 20) | (1 << 19) | (1 << 18) | (1 << 17),
        SM_PINCTRL: (2 << 29) | (1 << 26) | (1 << 20) | (1 << 10),        # SC 2 opt, 1 pin each
    }
    conf1 = {
        SM_CLKDIV: (1 << 16),
        SM_EXECCTRL: (1 << 30) | (4 << 24) | (2 << 12) | (0 << 7),         # JMP_PIN = 4
        SM_SHIFTCTRL: (4 << 20) | (1 << 19) | (1 << 18),
        SM_PINCTRL: (1 << 29) | (1 << 26) | (1 << 20) | (0 << 15),
    }
    s = Sched()
    s.write(A_CTRL, 0)
    for e, (prog, conf) in enumerate([(prog0, conf0), (prog1, conf1)]):
        for k, word in enumerate(image(prog)):
            s.write(A_IMEM[e] + k, word)
        for i, value in conf.items():
            s.write(A_SM_BASE + 6 * e + i, value)
    s.write(A_CTRL, CTRL_SM_ENABLE | CTRL_CLKDIV_RESTART)
    # host traffic: TXF data for the blocking PULL, a forced SET, and a read of RXF0
    s.write(A_TXF[0], 0x1234_5678)
    s.write(A_SM_BASE + 6 * 0 + SM_INSTR, set_(SET_PINS, 3))      # forced set pins, 3
    s.write(A_TXF[0], 0x9ABC_DEF0)
    s.read(A_RXF[0])
    s.read(A_RXF[0])
    s.write(A_FDEBUG, 0x0303_0303)
    s.read(A_FSTAT)
    s.write(A_CTRL, CTRL_SM_ENABLE | CTRL_SM_RESTART)
    while s.cursor < cycles - 4:
        s.cursor += 1
    summary = {
        "engines": [
            {"engine": 0, "program": ["%04x" % w for w in prog0], "config": conf0,
             "wrap": [0, 5], "side": {"sideset_count": 2, "side_en": 1, "side_pindir": 1}},
            {"engine": 1, "program": ["%04x" % w for w in prog1], "config": conf1,
             "wrap": [0, 2], "side": {"sideset_count": 1, "side_en": 1, "side_pindir": 0}},
        ],
        "host_ops": {"trivial": 1},
        "notes": ["the selftest stimulus: fixed, no random stimulus",
                  "engine 0: set pindirs/set pins/set x/jmp x--/pull block/out pins, "
                  "wrap 0..5, side-set 2 bits with enable, side-set to pin 1 as pindir",
                  "engine 1: set pindirs,0/set pins,0/jmp, contests pin 0 with engine 0 "
                  "(engine 1 wins)"],
    }
    return Stimulus(seed=seed, cycles=cycles, ops=s.ops, summary=summary)


def _main(argv: list[str]) -> int:
    """`python -m verif.fuzz.gen --seed 3 --cycles 500` writes a stimulus to stdout or a file."""
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--cycles", type=int, default=1000)
    ap.add_argument("--trivial", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    stim = trivial(cycles=args.cycles) if args.trivial else gen(args.seed, args.cycles)
    text = stim.to_json()
    if args.out:
        Stimulus(seed=stim.seed, cycles=stim.cycles, ops=stim.ops,
                 summary=stim.summary).save(args.out)
        print(f"wrote {args.out}: {len(stim.ops)} bus ops, {stim.cycles} cycles")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_main(sys.argv[1:]))