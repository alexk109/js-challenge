"""Host side of the MVP I2S firmware: load program.pio, set both engines, stream slots.

Everything here is written against `docs/mvp-isa.md` (the reference step,
autopull, the divider, the pin windows) and `docs/mvp-regmap.md` (word addresses, the
startup order).  Host accesses go through `iss.pio_iss.ChipDevice`'s queue, one per tick, so
every wait in this module is the simulation running -- the same thing a host does over the
SPI port of `docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                  # 50 MHz core clock
    dev = bus.attach(ChipDevice())             # nets p0..p17
    drv = I2sDriver(dev, bus)                   # SCK p0, WS p1, SD p2
    drv.stream([0x0123, -0x0456, 0x0007, ...]) # one stereo frame per pair of samples
    chk.flush()                                # close the last slot, as §6.2's receiver would

### What the chip plays, and why

The chip is the **I2S controller**: the device that generates SCK and WS (UM11732 §3, "the
device generating SCK and WS is the controller").  So the firmware drives all three nets and
the bench's `I2sReceiver` is the target that latches them.

### Two engines

`program.pio` holds two programs.  `i2s` runs on engine 0 and does the bit clock (one
side-set bit, `SC = 1`) and the data (one OUT pin); it is 32 words, one slot of 16 cells,
wrapping on itself.  `i2s_ws` runs on engine 1 and writes WS once per slot through its own
OUT window.  Splitting them is what keeps a 16-bit slot inside 32 words: WS has to change
on the same SM cycle as the edge that raises SCK, and an instruction's side-set field is an
immediate, so one engine would need a second copy of every HIGH/LOW pair per channel (the
reasoning is in `fw/i2s/program.pio`, "Why two engines and not one").

Both engines must run the same number of SM cycles per slot, and `slot_cycles()` checks
that against the assembled words rather than trusting the comment.

### Speed

The program fixes the shape of a bit clock period -- `HALF_DELAY` SM cycles per half, 50%
duty -- and the 16.8 divider scales it, exactly as an RP2040 program does.  Only *integer*
divisors are used: a fractional one would jitter the period, and `fSCK` in
`I2sChecker` compares every period against the nominal one.  `bit_clock_ticks()`,
`bit_clock_hz()` and `sample_rate()` are the numbers the bench configures `verif.vip.i2s`
with, all derived from the words and the divider, so the test cannot check the firmware
against a rate the firmware does not produce.

### Stopping

`stream()` does not need to know a cycle boundary.  Both engines run out of TX FIFO words
at the same slot boundary and stall there -- engine 0 on the `out pins, 1` that follows an
empty OSR, engine 1 on the `pull block` that is its last instruction of the slot, one cycle
*before* its wrap.  The final rising edge of the stream therefore happens with no WS
change, which is what leaves the last slot complete for `I2sChecker.flush()`
(`### Stopping the stream` in `program.pio`).  The cost is two `FDEBUG.TXSTALL` flags at the
end of a stream, which is a stall on an empty FIFO and not an error;
`drain()` waits for exactly those two flags.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

from asm.pioasm import Program, assemble

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
TICK_NS = 20.0                 # ns per core cycle at CORE_HZ
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
PROGRAM_PATH = Path(__file__).with_name("program.pio")
DATA_PROGRAM = "i2s"          # engine 0: bit clock + serial data
WS_PROGRAM = "i2s_ws"         # engine 1: word select

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
# FSTAT/FDEBUG fields, engine e (bits 25:24 and 17:16 of each)
FSTAT_TXEMPTY, FSTAT_TXFULL = 0x0100_0000, 0x0001_0000
FDEBUG_TXSTALL, FDEBUG_TXOVER = 0x0100_0000, 0x0001_0000


def __getattr__(name):  # TXFIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "TXFIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


# Instruction encodings the host forces (docs/mvp-isa.md `## Encoding`, `### EXEC`).
OP_JMP = 0b000
JMP_ALWAYS = 0b000
PULL_BLOCK = 0x80A0           # the worked encoding in `### Delay/side-set field`

#: The `i2s` program must be exactly this shape, or the driver cannot derive the timing from
#: it.  Checked by `check_data_program()` instead of being assumed.
EXPECTED_CELLS = 16


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address.

    The host cannot write the PC (docs/mvp-isa.md `### Enable, restart`); a forced JMP
    through `SMe_INSTR` is the only way to set it.
    """
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


# --------------------------------------------------------------------------------------------
# The programs: assemble them, and read the timing back out of the words
# --------------------------------------------------------------------------------------------

def load_program(path: Path | str = PROGRAM_PATH) -> tuple[Program, Program]:
    """Assemble `program.pio` and return its two programs, (engine 0, engine 1)."""
    programs = assemble(Path(path).read_text(encoding="utf-8"))
    names = [p.name for p in programs]
    if names != [DATA_PROGRAM, WS_PROGRAM]:
        raise ValueError(
            f"{path}: expected .program {DATA_PROGRAM} then {WS_PROGRAM}, got {names}"
        )
    data, ws = programs
    check_data_program(data)
    check_ws_program(ws, data)
    return data, ws


def decode_word(program: Program, word: int) -> dict:
    """Split one word into the fields `docs/mvp-isa.md` `## Encoding` defines.

    Only what this driver needs: the opcode and how the delay/side-set field splits, whose
    layout depends on `PINCTRL.SIDESET_COUNT` rather than on the instruction
    (`### Delay/side-set field`).  `sc` counts the side-set bits including the `opt` enable,
    so the delay is the low `5 - sc` bits.
    """
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    low = word & 0xFF
    return {
        "op": word >> 13,
        "sideset": field >> (5 - sc) if sc else 0,
        "delay": field & ((1 << (5 - sc)) - 1),
        "dest": (low >> 5) & 0b111,
        "count": low & 0x1F,
    }


def is_high_half(program: Program, word: int) -> bool:
    """True for `nop side 1`: the HIGH half of a bit clock period, which raises SCK.

    A `nop` is `mov y, y` (docs/mvp-isa.md `### Delay/side-set field`, the worked encoding
    table), so the opcode is MOV and the destination is Y.
    """
    fields = decode_word(program, word)
    return (fields["op"] == 0b101 and fields["dest"] == 0b010
            and fields["sideset"] == 1 and fields["count"] == 0b010)


def is_low_half(program: Program, word: int) -> bool:
    """True for `out pins, 1 side 0`: the LOW half, which lowers SCK and writes one data bit."""
    fields = decode_word(program, word)
    return (fields["op"] == 0b011 and fields["dest"] == 0b000 and fields["count"] == 1
            and fields["sideset"] == 0)


def check_data_program(program: Program) -> None:
    """Refuse a `i2s` program whose timing this driver cannot read off the words.

    Every half period has to be `1 + delay` SM cycles (docs/mvp-isa.md `### Cycle
    accounting`), so a delay that is not the same everywhere would make one CLKDIV mean two
    different things.  The pattern must also alternate HIGH/LOW and fill one whole slot, so
    that "one wrap is one slot" is a property of the words and not of this comment.
    """
    if program.sideset_bits != 1 or program.sideset_opt:
        raise ValueError(
            f"{DATA_PROGRAM}: expected `.side_set 1` with no `opt` (one side-set data bit for "
            f"SC), got {program.sideset_bits} bit(s), opt={program.sideset_opt}"
        )
    if len(program.instrs) != 2 * EXPECTED_CELLS:
        raise ValueError(
            f"{DATA_PROGRAM}: expected {2 * EXPECTED_CELLS} words (one HIGH/LOW pair per cell "
            f"of a {EXPECTED_CELLS}-cell slot), got {len(program.instrs)}"
        )
    for k, word in enumerate(program.instrs):
        want_high = k % 2 == 0
        if (is_high_half if want_high else is_low_half)(program, word):
            continue
        raise ValueError(
            f"{DATA_PROGRAM}: word {k} ({word:#06x}) is not "
            f"`{'nop side 1' if want_high else 'out pins, 1 side 0'}`; the driver reads the bit "
            f"clock timing off these words"
        )
    delays = {decode_word(program, w)["delay"] for w in program.instrs}
    if len(delays) != 1:
        raise ValueError(f"{DATA_PROGRAM}: the halves have different delays {sorted(delays)}")


def check_ws_program(ws: Program, data: Program) -> None:
    """The two engines must wrap at the same time, and WS must ride the first edge.

    Engine 1 writes WS with the *first* instruction of its slot, because WS has to change on
    the same SM cycle as the edge that raises SCK (the checker's WS-EDGE rule allows no
    slack).  The two slots must therefore be the same number of SM cycles long, or the WS
    edges drift away from the clock edges one slot at a time.
    """
    if ws.sideset_bits:
        raise ValueError(
            f"{WS_PROGRAM}: expected `.side_set 0` (this engine writes one pin through its OUT "
            f"window and must not side-set anything), got {ws.sideset_bits} bit(s)"
        )
    if len(ws.instrs) < 2:
        raise ValueError(f"{WS_PROGRAM}: needs the WS write and the `pull block` after it")
    first = decode_word(ws, ws.instrs[0])
    if not (first["op"] == 0b011 and first["dest"] == 0b000 and first["count"] == 1):
        raise ValueError(
            f"{WS_PROGRAM}: instruction 0 ({ws.instrs[0]:#06x}) is not `out pins, 1`, the WS "
            f"write that has to land on the slot's opening edge"
        )
    last = ws.instrs[-1]
    if (last & 0xFF) != (PULL_BLOCK & 0xFF) or last >> 13 != 0b100:
        raise ValueError(
            f"{WS_PROGRAM}: the last instruction ({last:#06x}) is not `pull block`, which is "
            f"what stalls one cycle before the wrap when the TX FIFO runs dry"
        )
    got, want = program_cycles(ws), program_cycles(data)
    if got != want:
        raise ValueError(
            f"{WS_PROGRAM} is {got} SM cycles per slot but {DATA_PROGRAM} is {want}; the two "
            f"engines have to wrap together or the WS edges leave the clock edges"
        )


def program_cycles(program: Program) -> int:
    """SM cycles one pass through the program takes: `sum(1 + delay)`.

    `docs/mvp-isa.md` `### Cycle accounting`: an instruction from IMEM that does not stall
    completes in one SM cycle, and the next one runs `1 + delay` SM cycles later.
    """
    return sum(1 + decode_word(program, w)["delay"] for w in program.instrs)


def half_cycles(program: Program) -> int:
    """SM cycles one half of a bit clock period lasts, read off the first HIGH half."""
    return 1 + decode_word(program, program.instrs[0])["delay"]


def slot_cycles(data: Program, ws: Optional[Program] = None) -> int:
    """SM cycles one slot lasts -- one wrap of `data`, checked against `ws` when given."""
    cycles = program_cycles(data)
    if ws is not None and program_cycles(ws) != cycles:
        raise ValueError(f"engine 1 wraps in {program_cycles(ws)} SM cycles, engine 0 in {cycles}")
    return cycles


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


# --------------------------------------------------------------------------------------------
# Register values the programs need (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, side_en: int = 0, wrap_bottom: int = 0) -> int:
    """EXECCTRL: the wrap window the assembler worked out, plus SIDE_EN.

    `side_en` is 0 for both programs here.  `.side_set 1` without `opt` means the one
    side-set bit is *data*, so SCK's level rides on it directly; with SIDE_EN = 1 that bit
    would be the enable and `side 1` would drive two pins instead of one
    (`docs/mvp-isa.md` `### Delay/side-set field`).
    """
    if side_en and program.sideset_opt:
        raise ValueError(f"{program.name}: `.side_set ... opt` with SIDE_EN = 1 needs a data bit")
    if program.sideset_bits == 0 and side_en:
        raise ValueError(f"{program.name}: no side-set bit to use as an enable")
    return (side_en << 30) | (program.wrap << 12) | (wrap_bottom << 7)


def shiftctrl_value(*, pull_thresh: int = EXPECTED_CELLS, out_right: int = 0,
                    autopull: int = 1) -> int:
    """SHIFTCTRL for engine 0: shift *left*, autopull every 16 bits.

    `OUT_SHIFTDIR = 0` is what makes the sample go out MSB first ("Serial data is
    transmitted in two's complement with the MSB first", UM11732 §3.1): `out pins, 1` then
    takes OSR[31] and shifts the OSR left.  The RP2040 examples leave this bit at its reset
    value of 1, which sends an I2S word backwards -- so the host pushes each sample
    left-justified in bits 31:16 (see `pack_sample()`).

    `PULL_THRESH = 16` is one slot: the sixteenth `out pins, 1` brings the OSR count to the
    threshold and refills from the TX FIFO in the same SM cycle
    (`docs/mvp-isa.md` `### Instruction semantics`, OUT).
    """
    if not 1 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 1..31")
    return (pull_thresh << 25) | (out_right << 19) | (autopull << 17)


def ws_shiftctrl_value() -> int:
    """SHIFTCTRL for engine 1: no autopull, and the reset shift direction.

    The level of the next slot is fetched by an explicit `pull block`, which is one SM cycle
    in the slot's last cycle and -- unlike an autopull refill -- *stalls* when the FIFO runs
    dry.  That stall is how the stream stops (`program.pio`, "Stopping the stream").

    `OUT_SHIFTDIR` is written as 1, which is its reset value: the level is in bit 0 and the
    OUT is one bit wide, so the shift direction does not change what reaches the pin.  It has
    to be written explicitly because the host port moves whole words and a write of 0 here
    would clear it -- and with `OUT_SHIFTDIR = 0` the OUT would take OSR[31], which is where
    engine 0 puts the MSB of a sample and where no level ever goes.
    """
    return 1 << 19


def pinctrl_value(*, out_base: int, out_count: int = 1, sideset_base: int = 0,
                  sideset_count: int = 0, set_base: int = 0, set_count: int = 0,
                  in_base: int = 0) -> int:
    """PINCTRL: one window per write channel, all of them one pin wide.

    Engine 0 needs two windows on two different pins -- SCK through the side-set window and
    SD through the OUT window -- so this takes both bases.  Engine 1 writes WS through its
    OUT window and side-sets nothing.

    `SET_BASE`/`SET_COUNT` are the window a forced `set pindirs, 7` reaches, so they cover
    all three nets on both engines; nothing else in either program writes through it.
    """
    for name, value in (("out_base", out_base), ("sideset_base", sideset_base),
                        ("set_base", set_base), ("in_base", in_base)):
        if not 0 <= value < NUM_PINS:
            raise ValueError(f"{name} {value} is outside 0..{NUM_PINS - 1}")
    if not all(0 <= c <= 32 for c in (out_count, sideset_count, set_count)):
        raise ValueError("window counts must be 0..32")
    if sideset_count and out_count:
        out_pins = {(out_base + k) % NUM_PINS for k in range(out_count)}
        side_pins = {(sideset_base + k) % NUM_PINS for k in range(sideset_count)}
        if out_pins & side_pins:
            raise ValueError(
                f"the OUT window {sorted(out_pins)} and the side-set window {sorted(side_pins)} "
                f"share pin(s): the side-set channel wins over OUT/SET on the same pin "
                f"(docs/mvp-isa.md `### Pin writes`)"
            )
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (in_base << 15)
            | (sideset_base << 10) | (set_base << 5) | out_base)


def pack_sample(sample: int, word_bits: int = EXPECTED_CELLS) -> int:
    """The TX FIFO word for one slot: the sample, two's complement, MSB first in bits 31:0.

    Engine 0 shifts left (`OUT_SHIFTDIR = 0`), so the first bit on the wire is bit 31.  A
    16-bit sample therefore goes in bits 31:16, and the low half of the word is never
    shifted out -- 16 `out pins, 1` of a slot empty exactly the OSR count of 16.
    """
    if not 4 <= word_bits <= 32:
        raise ValueError(f"word_bits must be 4..32 (got {word_bits})")
    pattern = int(sample) & ((1 << word_bits) - 1)
    return (pattern << (32 - word_bits)) & 0xFFFF_FFFF


def pack_ws(level: int) -> int:
    """The TX FIFO word for engine 1: the WS level in bit 0.

    Engine 1's `out pins, 1` shifts right (its reset `OUT_SHIFTDIR`), so it takes OSR[0].
    """
    return 1 if int(level) else 0


@dataclass(frozen=True)
class Config:
    """One I2S setup: which pins, which engine plays what, and how fast.

    `clkdiv` is the only knob that changes the rate; see `### Speed`.  `word_bits` must be
    the program's slot length, because a slot is exactly one sample and the checker judges
    `SLOT` and `WORD` against the geometry it is configured with.
    """
    clkdiv: int = 1
    bclk_pin: int = 0             # SCK, side-set window of engine 0
    ws_pin: int = 1               # WS, OUT window of engine 1
    sd_pin: int = 2               # SD, OUT window of engine 0
    word_bits: int = EXPECTED_CELLS
    slots: int = 2                # 2 = stereo: left then right, WS alternating
    left_level: int = 0           # "WS = 0; channel 1 (left)" (UM11732 §3.2)

    def __post_init__(self) -> None:
        pins = (self.bclk_pin, self.ws_pin, self.sd_pin)
        for name, pin in zip(("bclk_pin", "ws_pin", "sd_pin"), pins):
            if pin not in UIO_PINS:
                raise ValueError(
                    f"{name} {pin} is not a uio pin 0..7: the MVP can only *drive* the "
                    f"bidirectional pins (docs/mvp-isa.md `### The 18 protocol pins`)"
                )
        if len(set(pins)) != 3:
            raise ValueError(f"the three nets need three pins, got {pins}")
        if self.word_bits != EXPECTED_CELLS:
            raise ValueError(
                f"word_bits {self.word_bits} does not match the program's {EXPECTED_CELLS}-cell "
                f"slot: a slot is exactly one sample"
            )
        if self.slots not in (1, 2):
            raise ValueError(f"slots {self.slots} is not 1 (mono) or 2 (stereo)")
        if self.clkdiv < 1:
            raise ValueError(f"clkdiv {self.clkdiv} must be at least 1")

    @property
    def slot_bits(self) -> int:
        """Cells a slot occupies, which is what the clock counts (UM11732 §6.1)."""
        return self.word_bits

    @property
    def bits_per_frame(self) -> int:
        return self.slot_bits * self.slots

    def ws_levels(self, slots: int) -> list[int]:
        """The WS level of each of `slots` consecutive slots: left, right, left, ...

        §3.2: "WS = 0; channel 1 (left)", "WS = 1; channel 2 (right)", and `### WS` in
        UM11732 §6.1 loads the shift register at *every* WS change, so the level alternates
        with a period of two slots.
        """
        if slots < 0:
            raise ValueError(f"slots {slots} is negative")
        first = 1 if self.left_level else 0
        return [first if i % 2 == 0 else 1 - first for i in range(slots)]


def set_pindirs(mask: int) -> int:
    """`set pindirs, MASK` -- the instruction the host forces to make pins outputs.

    SET with destination PINDIRS and a 5-bit immediate, so the mask is `SET_COUNT` pins wide
    starting at `SET_BASE` (`docs/mvp-regmap.md` `### Typical host sequences`, step 4: "Set
    the initial pins with forced `set pins`/`set pindirs` through `SMe_INSTR`").  The chip has
    no host-writable direction register, so this is the only way to turn a pin into an
    output -- and the reason both programs need no word for it.
    """
    if not 0 <= mask <= 0x1F:
        raise ValueError(f"set pindirs mask {mask} does not fit in 5 bits")
    return 0xE080 | mask          # opcode 111, dest PINDIRS (100), data -- the table's
                                   # worked encoding `set pindirs, 1` is 0xE081


def set_pins(value: int) -> int:
    """`set pins, VALUE` -- the instruction the host forces to park the pins before they
    become outputs.

    SET with destination PINS and the same 5-bit immediate, so it writes `SET_COUNT` pins
    starting at `SET_BASE`, exactly like `set_pindirs()` writes their directions.  A uio pin
    that is not being driven sits at its reset level of 0, so without this the moment the
    driver turns the direction on would put that 0 on the net -- and WS would change from its
    pull-up to 0 *before* the first clock edge, which the checker reads as a WS transition
    that no clock edge justifies (WS-EDGE, then WS-1T).
    """
    if not 0 <= value <= 0x1F:
        raise ValueError(f"set pins value {value} does not fit in 5 bits")
    return 0xE000 | value          # opcode 111, dest PINS (000), data


@dataclass(frozen=True)
class Pins:
    """Where the three nets sit, and the idle level each one has before the chip drives it.

    The net pull-ups are not decoration: the checker adopts whatever the pins idle at on its
    first tick instead of inventing a transition (`I2sChecker.tick`), so the bit clock has to
    idle low and WS has to idle at the level of the channel that is *not* selected --
    otherwise the first WS transition is not a transition and WS-EDGE has nothing to check.
    """
    bclk: int = 0
    ws: int = 1
    sd: int = 2

    def __post_init__(self) -> None:
        for name, pin in (("bclk", self.bclk), ("ws", self.ws), ("sd", self.sd)):
            if pin not in UIO_PINS:
                raise ValueError(f"{name} pin {pin} is not a uio pin 0..7")
        span = (self.bclk, self.bclk + 1, self.bclk + 2)
        if (self.ws, self.sd) != span[1:]:
            raise ValueError(
                f"the three nets must be three contiguous uio pins ({self.bclk}, "
                f"{self.bclk + 1}, {self.bclk + 2}), got {self.bclk}, {self.ws}, {self.sd}: "
                f"one forced `set pindirs` covers a SET window, so the window has to reach all "
                f"three"
            )

    @property
    def set_base(self) -> int:
        return self.bclk

    @property
    def set_count(self) -> int:
        return 3

    def nets(self) -> tuple[str, str, str]:
        """The three net names on a `verif.vip.core.Bus`, in `verif.vip.i2s.NETS` order."""
        return tuple(f"p{pin}" for pin in (self.bclk, self.ws, self.sd))

    def idle_levels(self, left_level: int = 0) -> tuple[int, int, int]:
        """(bclk, ws, sd) before anything drives: clock low, WS on the other channel, SD low."""
        return (0, 0 if left_level else 1, 0)


class I2sDriver:
    """Drives both engines of an `iss.pio_iss.ChipDevice` as an I2S controller.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are
    queued on `dev` and land one per tick as the simulation runs, so `read()` and `stream()`
    pump the bus while they wait.

    The two engines are started by one CTRL write (both `SM_ENABLE` bits with both
    `SM_RESTART` and `CLKDIV_RESTART`), which is what keeps them in lockstep
    (`docs/mvp-isa.md` `### Clock divider`), and both are fed one word per slot: a sample on
    engine 0, the WS level for the slot after it on engine 1 (`### Two engines`).
    """

    MAX_TICKS = 4_000_000
    STALL_POLL = 32              # ticks between `FDEBUG` polls while draining
    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)

    def __init__(self, dev, bus, *, config: Optional[Config] = None,
                 program: Optional[tuple[Program, Program]] = None,
                 path: Path | str = PROGRAM_PATH) -> None:
        self.dev, self.bus = dev, bus
        self.data, self.ws = program if program is not None else load_program(path)
        self.config = config if config is not None else Config()
        if self.config.word_bits != len(self.data.instrs) // 2:
            raise ValueError(
                f"word_bits {self.config.word_bits} does not match the program's "
                f"{len(self.data.instrs) // 2}-cell slot"
            )

    # ---- geometry, all of it read off the assembled words ----
    @property
    def half_cycles(self) -> int:
        """SM cycles per half bit clock period."""
        return half_cycles(self.data)

    @property
    def period_cycles(self) -> int:
        """SM cycles per bit clock period: two halves at a 50% duty cycle."""
        return 2 * self.half_cycles

    @property
    def slot_cycles(self) -> int:
        """SM cycles per slot (one word on engine 0, one level on engine 1)."""
        return slot_cycles(self.data, self.ws)

    @property
    def bit_clock_ticks(self) -> int:
        """Bit clock period in core cycles: `2 * half_cycles * CLKDIV`."""
        return self.period_cycles * self.config.clkdiv

    @property
    def slot_ticks(self) -> int:
        return self.slot_cycles * self.config.clkdiv

    @property
    def frame_ticks(self) -> int:
        return self.slot_ticks * self.config.slots

    @property
    def bit_clock_hz(self) -> float:
        return 1e9 / (self.bit_clock_ticks * TICK_NS)

    @property
    def sample_rate(self) -> float:
        """Frames per second: the bit clock divided by the bits in a frame.

        `bit_clock_hz` is "the product of the sample rate, the number of bits per channel and
        the number of channels" (Wikipedia, "Details"), so this is the rate a checker has to
        be configured with for the stream to be in spec.
        """
        return self.bit_clock_hz / self.config.bits_per_frame

    @property
    def words_used(self) -> tuple[int, int]:
        return words_used(self.data), words_used(self.ws)

    # ---- addresses ----
    @property
    def imem_base(self) -> tuple[int, int]:
        return A_INSTR_MEM

    def sm_addr(self, engine: int, index: int) -> int:
        return A_SM_BASE + 6 * engine + index

    # ---- the register values this configuration asks for ----
    @property
    def clkdiv(self) -> int:
        return clkdiv_value(self.config.clkdiv)

    def execctrl(self, engine: int) -> int:
        return execctrl_value(self.data if engine == 0 else self.ws)

    def shiftctrl(self, engine: int) -> int:
        return shiftctrl_value() if engine == 0 else ws_shiftctrl_value()

    def pinctrl(self, engine: int) -> int:
        """PINCTRL for one engine: one window per write channel, all on the three I2S pins.

        The SET window is three pins wide on both engines, because `set pindirs, 7` is forced
        through `SMe_INSTR` to make the nets outputs and it writes the SET window.
        """
        pins = self.pins
        return pinctrl_value(
            out_base=pins.sd if engine == 0 else pins.ws,
            out_count=1,
            sideset_base=pins.bclk,
            sideset_count=self.data.sideset_bits if engine == 0 else 0,
            set_base=pins.set_base,
            set_count=pins.set_count,
            in_base=pins.bclk,
        )

    @property
    def pins(self) -> Pins:
        return Pins(self.config.bclk_pin, self.config.ws_pin, self.config.sd_pin)

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write. It takes effect at the end of its bus access cycle `h`
        (`docs/mvp-regmap.md` `### Cycle-exact mapping`)."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def read(self, addr: int) -> int:
        """Queue one host read and run the bus until its data comes back.

        The value is the register's state in the access cycle, so a FIFO pop or a W1C
        happens as this returns.
        """
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host read of {addr:#04x} unanswered after {ticks} ticks")
        return int(future.result())

    def settle(self, extra: int = 2) -> None:
        """Run the bus until the host queue is empty, then `extra` more ticks."""
        ticks = 0
        while not self.dev.idle:
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host writes still queued after {ticks} ticks")
        self.run(extra)

    def run(self, ticks: int) -> None:
        """Run the bus for `ticks` core cycles."""
        for _ in range(max(0, int(ticks))):
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
        """Read FDEBUG and write it back to clear the sticky flags (W1C)."""
        value = self.fdebug()
        if value:
            self.write(A_FDEBUG, value)
            self.settle()
        return value

    def padout(self) -> int:
        return self.read(A_DBG_PADOUT)

    def padoe(self) -> int:
        return self.read(A_DBG_PADOE)

    def cfginfo(self) -> int:
        return self.read(A_DBG_CFGINFO)

    def pc(self, engine: int) -> int:
        return self.read(self.sm_addr(engine, SM_ADDR))

    def ctrl_bits(self, *fields: int) -> int:
        """The CTRL fields named, every engine's bit set -- both engines always move together."""
        mask = 0
        for base in fields:
            mask |= base | (base << 1)
        return mask

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`) ----
    def load(self) -> None:
        """Steps 1-3 for both engines: stop them, load the programs, set the four registers.

        The engines are left disabled at PC 0, so the caller can arm, preload and prime them
        before enabling: a disabled engine never drains its FIFO.
        """
        self.stop()
        for engine, program in ((0, self.data), (1, self.ws)):
            for k, word in enumerate(program.instrs):
                self.write(A_INSTR_MEM[engine] + k, word)
            self.write(self.sm_addr(engine, SM_CLKDIV), self.clkdiv)
            self.write(self.sm_addr(engine, SM_EXECCTRL), self.execctrl(engine))
            self.write(self.sm_addr(engine, SM_SHIFTCTRL), self.shiftctrl(engine))
            self.write(self.sm_addr(engine, SM_PINCTRL), self.pinctrl(engine))
        self.settle()

    def arm(self) -> None:
        """Steps 4 and 5: park the pins, make them outputs, then a forced `jmp start`.

        The chip has no host-writable direction register, so the three nets become outputs
        through a forced `set pindirs, 7`.  A forced `set pins` comes *first*, with the idle
        levels the bench's nets pull up to (clock low, WS on the channel that is not
        selected, SD low): a uio pin that is not being driven reads 0, so turning the
        direction on without parking the level first would put 0 on WS while the checker is
        looking at an idle bus -- a WS transition with no clock edge behind it.  Both
        instructions write the same three-pin SET window on either engine, so only engine 0
        does them; engine 0's `set pins` also side-sets SCK low, the idle clock level.

        The forced `jmp start` is the only way to set the PC (`docs/mvp-isa.md` `### Enable,
        restart`).  Both forced cycles run the background autopull of their engine, so this
        must happen *before* the FIFOs are filled (as in fw/uart_tx's `arm()`): a forced
        cycle on engine 0 would pop a stale word into its OSR.
        """
        level = self.idle_ws_level << 1        # bit 0 = SCK, bit 1 = WS, bit 2 = SD
        self.write(self.sm_addr(0, SM_INSTR), set_pins(level))
        # A forced instruction runs in the cycle after the host write that armed it.
        self.settle(2)
        self.write(self.sm_addr(0, SM_INSTR), set_pindirs(0b111))
        self.settle(2)
        for engine in (0, 1):
            self.write(self.sm_addr(engine, SM_INSTR), jmp_abs(self.start_addr(engine)))
            self.settle(2)

    @property
    def idle_ws_level(self) -> int:
        """The level WS has to idle at: the channel that is *not* selected (UM11732 §3.2), so
        that the first slot boundary is a real transition."""
        return 0 if self.config.left_level else 1

    def start_addr(self, engine: int) -> int:
        program = self.data if engine == 0 else self.ws
        return program.labels["start"]

    def enable(self) -> None:
        """Step 6: both engines' `SM_ENABLE` with `SM_RESTART` and `CLKDIV_RESTART` in one write.

        Written together on purpose: engines restarted together stay in lockstep
        (`docs/mvp-isa.md` `### Clock divider`), and engine 1's WS edges have to stay on
        engine 0's clock edges.  Each engine's first SM cycle is two core cycles after this
        write.
        """
        bits = self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, (self.ctrl() & ~bits) | bits)
        self.settle()

    def stop(self) -> None:
        """Disable both engines, keeping every register (`### Enable, restart`)."""
        ctrl = self.read(A_CTRL)
        keep = self.ctrl_bits(CTRL_SM_ENABLE)
        self.write(A_CTRL, ctrl & ~keep)
        self.settle()

    def restart(self) -> None:
        """`SM_RESTART` + `CLKDIV_RESTART` on both engines, keeping the enable."""
        ctrl = self.ctrl()
        bits = self.ctrl_bits(*self.CTRL_BITS) & ~self.ctrl_bits(CTRL_SM_ENABLE)
        self.write(A_CTRL, ctrl | bits)
        self.settle()

    # ---- the TX FIFOs: one word per slot on each engine ----
    def room(self, engine: int, fstat: Optional[int] = None) -> bool:
        value = self.fstat() if fstat is None else fstat
        return not value & (FSTAT_TXFULL << engine)

    def push_word(self, engine: int, word: int, timeout: Optional[int] = None) -> None:
        """Push one word, waiting for room first so nothing is dropped (TXOVER).

        `timeout` is in core cycles and defaults to `MAX_TICKS`: a FIFO that stays full for
        four million cycles is not going to drain, so a test can pass a small one and get the
        exception instead of the wait.
        """
        deadline = self.MAX_TICKS if timeout is None else int(timeout)
        while not self.room(engine):
            self.run(8)
            deadline -= 8
            if deadline <= 0:
                raise TimeoutError(
                    f"engine {engine}'s TX FIFO stayed full for {timeout or self.MAX_TICKS} "
                    f"core cycles"
                )
        self.write(A_TXF[engine], word)
        self.settle(1)

    def push_slot(self, sample: int, level: int, *, strict: bool = True,
                  timeout: Optional[int] = None) -> None:
        """Push one slot's two words: the sample to engine 0, the WS level to engine 1.

        The words go in on consecutive ticks and, with `strict`, only when *both* FIFOs have
        room.  That matters because the two engines stall on their own TX FIFOs: a stream that
        ran engine 0 one slot further than engine 1 would leave WS frozen mid-stream, which is
        a framing fault and not a slow start.  `strict=False` pushes to whichever engine has
        room, which is what a caller wants when it is priming one engine only.

        Engine 1's word is the level for the slot *after* this one: its `pull block` is the
        last instruction of its slot and the `out pins, 1` that writes WS is the first
        instruction of the next one (`fw/i2s/program.pio`).
        """
        words = {0: pack_sample(sample, self.config.word_bits), 1: pack_ws(level)}
        deadline = self.MAX_TICKS if timeout is None else int(timeout)
        while True:
            fstat = self.fstat()
            free = [engine for engine in (0, 1) if self.room(engine, fstat)]
            if len(free) == 2 or (free and not strict):
                break
            self.run(8)
            deadline -= 8
            if deadline <= 0:
                raise TimeoutError(
                    f"an engine's TX FIFO stayed full for {timeout or self.MAX_TICKS} core "
                    f"cycles: level {self.flevel():#06x}, TXFULL is bit {FSTAT_TXFULL:#x} plus "
                    f"the engine index"
                )
        for engine in free:
            self.write(A_TXF[engine], words[engine])
        self.settle(1)

    def prime_ws(self) -> None:
        """Force one `pull block` on engine 1 so its OSR holds the first WS level.

        Engine 0 needs nothing: `CTRL.SM_RESTART` clears `osr_cnt` to 32, "empty"
        (`docs/mvp-isa.md` `### Engine state`), so the very first SM cycle after the restart
        runs the background autopull and fills the OSR by itself.  Engine 1 has
        `AUTOPULL = 0` -- its `pull block` is what makes a dry FIFO *stall* instead of
        refilling silently -- so without this forced pull its OSR would still be empty when
        slot 0's `out pins, 1` ran, and that OUT would write the reset value 0 forever.

        This has to run *after* the FIFOs are filled (a forced `pull` pops a word) and
        *before* the enable, whose `SM_RESTART` keeps the OSR value but not the count.
        """
        if self.fstat() & (FSTAT_TXEMPTY << 1):
            raise RuntimeError(
                "engine 1's TX FIFO is empty: `prime_ws()` pops the level for the first slot, "
                "so preload at least one word before calling it"
            )
        self.write(self.sm_addr(1, SM_INSTR), PULL_BLOCK)
        self.settle(2)

    def stalled(self) -> bool:
        """True when both engines show `FDEBUG.TXSTALL`: the stream has run out of words.

        That is the normal end of a stream (`### Stopping the stream`), not an error, but it
        is worth checking: an underrun in the middle of a stream would stop it just as
        quietly, and only the FIFO flags distinguish the two.
        """
        value = self.fdebug()
        return all(value & (FDEBUG_TXSTALL << engine) for engine in (0, 1))

    def stream(self, samples: Sequence[int], *, preload: Optional[int] = None) -> None:
        """Send `samples`, one per slot, and leave both engines stalled at the end.

        Fills both FIFOs while the engines are stopped, primes engine 1's OSR, enables both with
        one CTRL write, feeds the remaining samples as the stream runs, and finally drains the
        bus until both engines have stalled on their empty FIFOs -- so the call returns with
        the last slot complete and the clock parked, with no cycle-accurate host action.

        `preload` words go in before the engines start (the FIFO is `fifo_depth` deep, 4 at the MVP), which is the
        runway that keeps the two engines in step; the rest are pushed as slots are consumed.
        """
        words = [int(s) & ((1 << self.config.word_bits) - 1) for s in samples]
        levels = self.config.ws_levels(len(words))
        if not words:
            raise ValueError("stream() needs at least one slot")
        self.load()
        self.arm()
        depth = regmap.fifo_depth()
        head = max(1, min(depth if preload is None else int(preload), len(words), depth))
        for sample, level in zip(words[:head], levels[:head]):
            self.push_word(0, pack_sample(sample, self.config.word_bits))
            self.push_word(1, pack_ws(level))
        self.prime_ws()
        self.enable()
        for sample, level in zip(words[head:], levels[head:]):
            self.push_slot(sample, level)
        self.drain(limit=self.drain_limit(len(words)))

    def drain_limit(self, slots: int) -> int:
        """A tick budget for a stream of `slots` slots that is generous but finite.

        A slot is `slot_ticks` core cycles, and the last slot can still be in a FIFO when the
        last push lands, so the budget is two whole streams plus a thousand ticks of slack.
        Without one, a driver bug would spin in `drain()` until the machine gave up instead
        of failing with a number in the message.
        """
        return 2 * slots * self.slot_ticks + 1000

    def drain(self, until: Optional[Callable[[], bool]] = None,
              limit: Optional[int] = None) -> bool:
        """Run the bus until `until()` holds or both engines stall on an empty TX FIFO.

        `stalled()` is polled every `STALL_POLL` ticks: a stalled clock produces no further
        edges, so a stream that has run dry simply stops changing the bus, and without this
        the drain would have to be a pure timeout.  True when either condition was reached.
        """
        limit = self.drain_limit(4) if limit is None else int(limit)
        ticks = 0
        while ticks < limit:
            if until is not None and until():
                return True
            self.bus.step()
            ticks += 1
            if ticks % self.STALL_POLL == 0 and self.stalled():
                return True
        return until is not None and until()