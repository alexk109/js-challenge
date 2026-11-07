"""Host side of the MVP 1-Wire firmware: the register writes that run fw/onewire/program.pio.

Written against docs/mvp-regmap.md, not against the ISS: every address and field below is
the word address and bit position of the register map, so the same driver drives the RTL
once it exists. Host accesses go through the queue of `iss.pio_iss.ChipDevice` (one access
per core cycle, in order) and the driver is what steps the `verif.vip.core.Bus` while it
waits for a read to come back.

The chip is the **master** of the open-drain `dq` line and `Ds18b20Target` in
`verif/vip/onewire.py` is the slave, so everything here is the host side of a master:
it starts a reset, hands the program one 32-bit word per byte and takes the bytes back
out of the RX FIFO.

One PIO program, four host entry points, all forced through `SMx_INSTR` because
`SMx_ADDR` is read-only and there is no IRQ to wake the engine with:

    set pindirs, 0     release the line; the engine then idles in `init`
    set y, n           unused: the program paces itself with `pull block`
    jmp reset          reset pulse, presence wait, then straight into the bit loop
    jmp w_byte         the bit loop on its own, for a phase with no reset in front

The interesting property of this program is that **writing a byte and reading a byte are
the same loop**. A read slot is a write-1 slot that happens to sample the line and keep
the sample, so the host chooses by pushing a word of `0x00000000` (release every slot,
sample all of them) instead of a word of the byte it wants to send, and by turning
AUTOPUSH on so the samples reach the RX FIFO:

    reset                     reset pulse, then the line is free for the ROM command
    write(0xCC, 0x44)         Skip ROM, Convert T: one word per byte
    read(9)                   nine words of 0x00000000 come back as nine RX words

Both directions are LSB first, which is what `docs/mvp-isa.md` calls 1-Wire's own order,
and the host pushes the *inverted* byte because `out pindirs, 1` writes a direction:
1 pulls the line low, which is a 0-bit on the wire.
"""
from __future__ import annotations

from fw import regmap

from typing import Callable, List, Optional, Sequence

from asm.pioasm import Program
from verif.vip.onewire import speed_minimums

# One bus tick is one core cycle of the chip: 20 ns, i.e. 50 MHz. Every timing in this
# file is derived from it, the same way `verif/vip/onewire.py` derives its own.
TICK_NS = 20.0

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) -----------------
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF0, A_RXF0 = regmap.txf_addrs()[0], regmap.rxf_addrs()[0]
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_IMEM = regmap.imem_bases()              # engine e, IMEM word k at A_IMEM[e] + k
A_SM = regmap.sm_base()                        # SMe_* at A_SM + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

FLEVEL_TX0, FLEVEL_RX0 = 0, 4      # shift of the engine-0 field of FLEVEL 3:0 / 7:4


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


# FDEBUG bits of engine 0 (docs/mvp-regmap.md, `FDEBUG`).
FDEBUG_RXSTALL0 = 1 << 0
FDEBUG_RXUNDER0 = 1 << 8
FDEBUG_TXOVER0 = 1 << 16
FDEBUG_TXSTALL0 = 1 << 24

# Pin assignment of the program. fw/onewire/program.pio carries the same number in its
# `.define` line; `check_program` makes sure the two files still agree.
DEFAULT_DQ_PIN = 0

# The timing the program is built around, in SM cycles. These are not free parameters:
# they are what program.pio's delay fields add up to, `layout()` re-derives them from the
# assembled words and `check_program` fails if the two disagree.
DEFAULT_CLKDIV = 50                # 1 us per SM cycle at 50 MHz, which makes every
                                   # SM-cycle number below a microsecond
SLOT_LOW_SHORT = 6                 # `set pindirs, 1 [5]`: tLOW1 is 1..15 us
SLOT_LOW_LONG = 65                 # the write-0 low: tLOW0 is 60..120 us
SLOT_SAMPLE = 25                   # `in pins, 1`: a read slot is sampled 15..60 us in
SLOT_PERIOD = 75                   # fall to fall, both branches: tSLOT is 60..120 us
BYTE_SLOTS = 8                     # one TX word per byte, PULL_THRESH 8
RESET_LOW = 547                    # `set pindirs, 1 [1]` + `set x, 16` + 17 x 32
PRESENCE_SAMPLE = 69               # release .. `in pins, 1` at CLKDIV 50 (see below)
#: Where the presence sample wants to land, in ns after the reset release: the middle of
#: PRESENCE_WINDOW, which is 68..69 us over the whole legal divider range. The sample
#: reads the pad three core cycles before its own SM cycle, which `presence_cycles` adds
#: in, and the divider phase moves it by up to one SM cycle either way.
PRESENCE_TARGET_NS = 69_000.0
PRESENCE_HOLD = 6                  # `in pins, 1 [4]` + `push block`, after the sample
RESET_WAIT = 545                   # `set x, 16` + 17 x 32, the tRSTH wait
# The window the presence sample has to fall in, from the DS18B20's own two numbers:
# a pulse may start as late as tPDHIGH's 60 us maximum and lasts at least tPDLOW's
# 60 us minimum, so it is still low everywhere in 60..75 us after the release.
PRESENCE_WINDOW = (60_000.0, 75_000.0)
# Release .. the fall of the first slot: the presence sample, the three instructions that
# follow it, the tRSTH wait and the two instructions that get the bit loop going.
# `divider_window()` uses the nominal 623 here. That is safe: retargeting moves the
# presence part by at most 14 SM cycles, which at the fastest divider is 13 us on a total
# that is already 109 us over the 480 us minimum, and tRSTH has no maximum to break.
RESET_TO_SLOT = PRESENCE_SAMPLE + PRESENCE_HOLD + RESET_WAIT + 3

# The clock divider window the DS18B20 timing tables allow for this program, as
# (fastest, slowest). `test_the_divider_window_is_where_the_tables_say` re-measures both
# ends on the ISS; the numbers come from `limits()`.
PHASE_LIMIT = 400_000              # core cycles one phase may take before we give up


class OneWireError(RuntimeError):
    """The engine or the bus did something the driver cannot make sense of."""


# ---- the 16-bit words the host hands to the engine ---------------------------------
def tx_word(byte: int) -> int:
    """One byte to send, as the program consumes it: bit k is slot k of the byte.

    Bits go out least significant bit first (OUT_SHIFTDIR = 1), so bit 0 of the word is
    the first slot. `out pindirs, 1` writes a *direction*, and on an open-drain line the
    level register stays 0, so a 1 in the word pulls the line low: a 1 here is a 0-bit on
    the wire. The byte is therefore inverted here, which is the inversion
    docs/mvp-isa.md `### Open drain` asks of the host.
    """
    if not 0 <= byte <= 0xFF:
        raise ValueError(f"byte {byte!r} does not fit in 8 bits")
    return ~byte & 0xFF


def read_word(count: int = BYTE_SLOTS) -> int:
    """The TX word for a byte the program *reads*: every slot released, every slot sampled.

    A zero word makes `out x, 1` put a 0 in X every time, so every slot takes the short
    low and releases the line, and every slot's `in pins, 1` sample is kept. `count` is
    there so a caller that wants a word of `BYTE_SLOTS` slots can say so out loud; a
    count other than 8 would desynchronise `jmp !osre`, so it is refused.
    """
    if count != BYTE_SLOTS:
        raise ValueError(
            f"a read word is {BYTE_SLOTS} slots, one per bit of a byte; got {count}"
        )
    return 0


def jmp_word(address: int) -> int:
    """`jmp <address>` with no condition, no side-set and no delay: JMP is opcode 000
    and bits 12:0 are then all zero, so the word is the address."""
    return address & 0x1F


def mov_isr_null_word() -> int:
    """`mov isr, null`: ISR = 0 and isr_cnt = 0, forced into the EXEC latch.

    The host needs this before every read phase. The samples of the write slots in front
    of it are already in the ISR (the program samples in both branches, because it cannot
    tell a read slot from a write-1 slot) and AUTOPUSH is off during a write phase, so
    those bits are still sitting there when the read phase starts. `MOV ISR` is the only
    way to clear it: the register map has no write to the ISR
    (docs/mvp-regmap.md `### Address map`), so the host forces the instruction.
    """
    return (5 << 13) | (6 << 5) | 3          # MOV, destination ISR, no op, source NULL


def rx_byte(word: int) -> int:
    """The data byte of an RX word: bits 31:24.

    The program samples eight times with IN_SHIFTDIR = 1, so every sample shifts the ISR
    right and puts the new bit in at the top: the first bit sampled ends in bit 24 and the
    eighth in bit 31. 1-Wire sends a byte least significant bit first, so bit 24 is the
    byte's bit 0 and the byte is the right-aligned, un-reversed value. (IN_SHIFTDIR = 0
    would put the first bit sampled in bit 7 instead, which reverses the byte: measured,
    not guessed, see the test that checks this against the model's ROM.)
    """
    return (word >> RX_BYTE_SHIFT) & 0xFF


#: Where a sampled byte sits in an RX word, and where the presence bit sits: the ISR
#: shifts right, so the first bit sampled is the lowest of the byte and the last one is
#: the top bit of the word.
RX_BYTE_SHIFT = 24
RX_PRESENT_BIT = 31


def decode_word(instr: int) -> tuple:
    """(opcode name, destination/source, count, delay) of one assembled word.

    The program has no side-set, so bits 12:8 are the delay and nothing else
    (docs/mvp-isa.md `### Delay/side-set field`), which is all `layout()` needs to walk
    the program and add up its SM cycles.
    """
    op = instr >> 13
    delay = (instr >> 8) & 0x1F
    field = (instr >> 5) & 7
    count = (instr & 0x1F) or 32
    names = {0: "jmp", 1: "wait", 2: "in", 3: "out", 4: "pull", 5: "mov", 6: "irq", 7: "set"}
    dests = {0: "pins", 1: "x", 2: "y", 3: "null", 4: "pindirs", 5: "pc", 6: "isr", 7: "exec"}
    jmp_conds = {0: "", 1: "!x", 2: "x--", 3: "!y", 4: "y--", 5: "x!=y", 6: "pin", 7: "!osre"}
    if op == 0:
        return ("jmp", jmp_conds[field], count, delay)
    if op in (1, 2, 3):
        return (names[op], dests[field], count, delay)
    if op == 4:
        return ("pull" if (instr >> 7) & 1 else "push", "block" if (instr >> 5) & 1 else "noblock",
                0, delay)
    if op == 5:
        # `nop` is `mov y, y` and nothing else (docs/mvp-isa.md `## Encoding`, the worked
        # encodings table), so say so: the timing arithmetic below is all about nops, and
        # `retarget` will only rewrite a `nop`.
        if dests[field] == "y" and (instr & 7) == 2 and ((instr >> 3) & 3) == 0:
            return ("nop", "y", 0, delay)
        return ("mov", dests[field], (instr >> 3) & 3, delay)
    return ("set", dests[field], instr & 0x1F, delay)


# ---- walking the assembled program: where the SM cycles go -------------------------
def cycles(instr: int) -> int:
    """How many SM cycles an instruction occupies: one, plus its delay."""
    return 1 + ((instr >> 8) & 0x1F)


def layout(program: Program) -> dict:
    """Walk the assembled words and measure the slot, in SM cycles.

    Everything the host, the tests and the README quote about the waveform comes from
    here rather than from the constants at the top of this file, so editing a delay in
    program.pio cannot leave the numbers behind: `check_program` compares the two.

    The walk follows the control flow the program really takes for the two kinds of slot.
    Both start at `w_slot` with `out x, 1` and `jmp x-- w_zero`, and the `jmp` costs the
    same SM cycle taken as not taken, so the two `set pindirs, 1` that start the low land
    at the same offset of the slot. The two branches then have to be the same length again
    before `w_join`, or a 0-slot and a 1-slot would be different lengths and tSLOT would
    measure two different periods on one bus.
    """
    labels = program.labels
    for name in ("init", "reset", "w_byte", "w_slot", "w_one", "w_zero", "w_join",
                 "next_byte"):
        if name not in labels:
            raise OneWireError(f"program.pio has no `{name}` label for the host to use")
    words = program.instrs
    if program.sideset_bits:
        raise OneWireError(
            f"program.pio side-sets {program.sideset_bits} bit(s): the open-drain line is "
            "written by `set pindirs`, and a side-set would leave only 4 delay bits"
        )

    def is_jmp_to(addr: int) -> bool:
        return (words[addr] >> 13) == 0 and ((words[addr] >> 5) & 7) == 0 \
            and (words[addr] & 0x1F) == labels["w_join"]

    branches = {}
    for branch in ("w_one", "w_zero"):
        start = labels[branch]
        op, field, count, _ = decode_word(words[start])
        if (op, field) != ("set", "pindirs") or (words[start] & 0x1F) != 1:
            raise OneWireError(
                f"{branch} starts with `{op} {field} {words[start] & 0x1F}`, so the low of "
                "a slot does not start on the first instruction of the branch"
            )
        length = 0
        sample = release = None
        k = start
        while k != labels["w_join"] and not is_jmp_to(k):
            op, field, count, _ = decode_word(words[k])
            if op == "in" and sample is None:
                sample = length
            if op == "set" and field == "pindirs" and not count and release is None:
                release = length
            length += cycles(words[k])
            k += 1
            if k > labels["w_join"]:
                raise OneWireError(f"{branch} never reaches `w_join`")
        if release is None:
            raise OneWireError(
                f"{branch} never releases the line with `set pindirs, 0`, so the slot "
                "would leave the bus low and the next reset would look like one long slot"
            )
        if k != labels["w_join"]:
            length += cycles(words[k])                # w_one ends with `jmp w_join`
        branches[branch] = {"length": length, "sample": sample, "release": release}
    # The prefix, from the top of the slot to the branch: `out x, 1` and `jmp x-- w_zero`.
    prefix = sum(cycles(words[k]) for k in range(labels["w_slot"], labels["w_one"]))
    tail = cycles(words[labels["w_join"]])            # `jmp !osre w_slot`
    one, zero = branches["w_one"], branches["w_zero"]
    out = {
        "prefix": prefix,
        "fall": prefix,
        "sample": None if one["sample"] is None else prefix + one["sample"],
        # The low is the distance from the `set pindirs, 1` at the top of the branch to
        # its `set pindirs, 0`: both offsets are relative to the branch, so it is just the
        # offset of the release. It is also exactly what the checker measures, because the
        # pad follows the direction register one core cycle after each of the two.
        "low_short": one["release"],
        "low_long": zero["release"],
        "len_w_one": one["length"],
        "len_w_zero": zero["length"],
        "tail": tail,
        "period": prefix + max(one["length"], zero["length"]) + tail,
        # The stride from one byte's first slot to the next byte's. One more SM cycle than
        # `period` because the body ends on `jmp !osre`, which falls through into
        # `jmp w_byte` -> `pull block` -> the next `out x, 1`. One more again when the
        # engine actually stalls at that `pull block`, which is what the host's pacing does
        # on a write byte: the host pushes the next word after the byte it just sent, so the
        # pull blocks for one SM cycle (docs/mvp-isa.md `### Stalls, delay and side-set`,
        # "a blocking instruction takes stall + 1 + delay SM cycles"). A read byte is
        # different: `read()` pushes the next word while the byte is still going out, so its
        # pull does not stall.
        "word_period": prefix + max(one["length"], zero["length"]) + tail + 1,
        "word_period_stalled": prefix + max(one["length"], zero["length"]) + tail + 2,
    }
    return out


def presence_cycles(clkdiv: int) -> int:
    """SM cycles from the release instruction of the reset to its `in pins, 1`.

    The sample has to be where every legal presence pulse is still low, and that is a time,
    not a number of SM cycles: the same 69 SM cycles are 68.9 us at CLKDIV 50 and 80 us at
    CLKDIV 58, and 80 us is past the 60 us at which a pulse may still start, so a master
    that sampled there would miss a slave whose pulse was short and early. The delay fields
    cannot scale themselves, so the host scales them: this is the count `retarget` builds.
    """
    if not 1 <= clkdiv <= 0xFFFF:
        raise ValueError(f"CLKDIV integer part {clkdiv} is out of range")
    cycles_ = int(round(PRESENCE_TARGET_NS / (clkdiv * TICK_NS)))
    if not 4 <= cycles_ <= 97:
        raise OneWireError(
            f"the presence sample cannot be placed at CLKDIV {clkdiv}: it needs "
            f"{cycles_} SM cycles and the program's pad holds 4..97"
        )
    return cycles_


def _split_cycles(total: int, count: int = 3) -> List[int]:
    """`total` SM cycles spread over `count` instructions of at most 32 each, front loaded.

    Returns the delays, so an instruction given `d` occupies `1 + d` cycles. Every
    instruction gets at least one, which is what keeps a `nop` a `nop`.
    """
    if not count <= total <= 32 * count:
        raise OneWireError(
            f"{total} SM cycles do not fit in {count} instructions of 32 (needs "
            f"{count}..{32 * count})"
        )
    delays = []
    for i in range(count):
        left = count - i - 1
        take = min(32, total - left)
        delays.append(take - 1)
        total -= take
    return delays


def retarget(program: Program, clkdiv: int) -> Program:
    """The same program with its presence pad scaled to `clkdiv`.

    Exactly three instructions carry the pad: the `nop`s between the reset's release and
    its `in pins, 1`. Their delays are the only thing in program.pio that has to change
    with the clock, because the presence sample is the one timing that is a *time* rather
    than a number of SM cycles (see `presence_cycles`). Nothing else moves: the reset pulse
    is in front of the pad, the slot body is behind the tRSTH wait, and the engine's other
    timing is set by CLKDIV alone.

    At CLKDIV 50 this is the identity: the delays in program.pio are already
    `_split_cycles(presence_cycles(50) - 1)`.
    """
    words = list(program.instrs)
    span = list(range(program.labels["reset"], program.labels["w_byte"]))
    release = sample = None
    for k in span:
        op, field, count, delay = decode_word(words[k])
        if op == "set" and field == "pindirs" and not count and release is None:
            release = k
        elif op == "in" and release is not None and sample is None:
            sample = k
    if release is None or sample is None:
        raise OneWireError(
            "program.pio's reset has to release the line and then sample it, or there is "
            "nothing to retarget"
        )
    pad = list(range(release + 1, sample))
    for k in pad:
        op = decode_word(words[k])[0]
        if op != "nop":
            raise OneWireError(
                f"the presence pad of program.pio is {[decode_word(words[k])[0] for k in pad]}, "
                f"and only `nop` instructions can carry a scaled delay (word {k} is `{op}`)"
            )
    for k, delay in zip(pad, _split_cycles(presence_cycles(clkdiv) - 1, len(pad))):
        words[k] = (words[k] & ~(0x1F << 8)) | (delay << 8)
    return Program(
        name=program.name, instrs=words, origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap,
        sideset_bits=program.sideset_bits, sideset_opt=program.sideset_opt,
        sideset_pindirs=program.sideset_pindirs, labels=program.labels,
    )


def reset_layout(program: Program) -> dict:
    """Walk the reset sequence the same way, and measure it in SM cycles.

    Four things matter and none of them is visible in the source: how long the line is
    pulled low (tRSTL), how long after the release the presence bit is sampled (which has
    to be inside PRESENCE_WINDOW), that the bit really is pushed to the host, and how long
    it is from the release to the first slot of the transaction (tRSTH).

    The walk is a static one over the addresses from `reset` to `w_byte`, which is the
    sequence exactly once. The two `jmp x--` self-loops are then run for their trip count:
    the count is whatever the `set x, n` in front of them loaded, so the walk reads the
    same words the engine does instead of being told the numbers.
    """
    labels = program.labels
    words = program.instrs
    span = list(range(labels["reset"], labels["w_byte"]))
    # offset of every instruction once the loops have been unrolled, keyed by address
    at = {}
    length = 0
    for pos, k in enumerate(span):
        at[k] = length
        op, field, count, delay = decode_word(words[k])
        length += cycles(words[k])
        if op == "jmp" and field == "x--" and count == k and pos and span[pos - 1] == k - 1:
            # a self loop: `set x, n` immediately in front of it gives the trip count
            set_op, set_field, trips, _ = decode_word(words[k - 1])
            if (set_op, set_field) != ("set", "x") or not 1 <= trips <= 31:
                raise OneWireError(
                    f"the loop at {k} is `jmp x-- {k}` but the instruction in front of it "
                    f"is `{set_op} {set_field} {trips}`, so its trip count is not a "
                    "literal `set x, n` and the reset cannot be measured from the words"
                )
            # `jmp x-- L [d]` runs `trips + 1` times, not `trips`: the iteration where X
            # has reached 0 does not branch, but it still retires in 1 + d SM cycles before
            # falling through (docs/mvp-isa.md `### Instruction semantics`, JMP). Measured,
            # not assumed: the ISS trace of a reset shows 17 entries of 32 SM cycles for
            # `set x, 16` and this loop.
            length += trips * cycles(words[k])
    low = release = sample = None
    for k in span:
        op, field, count, delay = decode_word(words[k])
        if op == "set" and field == "pindirs":
            if count and low is None:
                low = at[k]
            elif not count and release is None:
                release = at[k]
        elif op == "in" and sample is None:
            sample = at[k]
    if low is None or release is None:
        raise OneWireError(
            "the reset sequence has to pull the line low and release it again; "
            f"program.pio does low={low}, release={release}"
        )
    if sample is None:
        raise OneWireError(
            "the reset sequence never samples the line, so a master would send a ROM "
            "command without checking for a presence pulse"
        )
    pushed = any(decode_word(words[k])[:2] == ("push", "block") for k in span)
    if not pushed:
        raise OneWireError(
            "the reset sequence never pushes the presence bit, so the host could not "
            "tell whether a slave answered"
        )
    return {
        "low": release - low,                 # the reset pulse itself
        "sample": sample - release,           # after the release
        "to_slot": (length - release) + 3,    # ... and on to the fall of the first slot
        "pushed": pushed,
    }


def check_program(program: Program, clkdiv: int = DEFAULT_CLKDIV,
                  timing: bool = True) -> dict:
    """Fail loudly if program.pio no longer says what this driver assumes it says.

    Three things have to hold and none of them is visible in the source: the program fits
    the IMEM, the two branches of a slot are the same length in SM cycles, and every
    timing below lands inside its datasheet window at this divider. The windows are the
    DS18B20 AC characteristics at standard speed, the same ones
    `verif.vip.onewire.speed_minimums("standard", 20.0)` produces, so they can be compared
    with what the checker measured without a second table to keep in step.

    The presence sample is the one layout number that depends on the divider, because it
    is the one timing that is a time rather than a count of SM cycles; it is checked against
    `presence_cycles(clkdiv)`, which is what `retarget` builds. program.pio as written is
    therefore the CLKDIV 50 program (its pad is `_split_cycles(68)`), and the driver
    retargets it for anything else.

    `timing=False` keeps the layout checks and drops the window ones, which is how a test
    builds a driver at a divider that is deliberately off spec: the layout is a property of
    the program, the windows are a property of the program *and* the divider, and the test
    wants the second kind of failure from the bus checker rather than from the driver.
    """
    if len(program.instrs) > 32:
        raise OneWireError(
            f"the program needs {len(program.instrs)} words, an engine has 32 "
            "(docs/mvp-isa.md `### Engine state`)"
        )
    words = program.instrs
    lay = layout(program)
    if lay["len_w_one"] != lay["len_w_zero"]:
        raise OneWireError(
            f"a write-1 slot is {lay['len_w_one']} SM cycles and a write-0 slot is "
            f"{lay['len_w_zero']}: the two branches of the slot have to be the same "
            "length, or tSLOT measures two different periods on one bus"
        )
    rst = reset_layout(program)
    for what, got, want in (("the write-1 low", lay["low_short"], SLOT_LOW_SHORT),
                            ("the write-0 low", lay["low_long"], SLOT_LOW_LONG),
                            ("the slot sample point", lay["sample"], SLOT_SAMPLE),
                            ("the slot period", lay["period"], SLOT_PERIOD),
                            ("the reset pulse", rst["low"], RESET_LOW),
                            ("the presence sample", rst["sample"], presence_cycles(clkdiv))):
        if got != want:
            raise OneWireError(
                f"program.pio puts {what} at {got} SM cycles, driver.py says {want}"
            )
    lay = dict(lay, reset=rst, reset_to_slot=rst["to_slot"])
    op, field, _, _ = decode_word(words[program.labels["w_byte"]])
    if (op, field) != ("pull", "block"):
        raise OneWireError(
            f"program.pio's byte boundary is `{op} {field}`, the host relies on it being "
            "`pull block`: it is the one instruction the engine stops on"
        )
    if not timing:
        return lay
    # The checker measures in ticks and `speed_minimums` hands its windows back in ticks,
    # so this compares ticks with ticks: no rounding of its own to disagree about.
    where = limits(clkdiv)
    t = speed_minimums(TICK_NS, "standard")
    for what, value, window in (
        ("tRSTL", where["reset_low_ns"], t["t_rstl"]),
        ("tRSTH", where["reset_to_slot_ns"], (t["t_rsth"], None)),
        ("tLOW1", where["low1_ns"], t["t_low1"]),
        ("tLOW0", where["low0_ns"], t["t_low0"]),
        ("tSLOT", where["slot_ns"], t["t_slot"]),
    ):
        lo, hi = window
        ticks = value / TICK_NS
        if (lo is not None and ticks < lo) or (hi is not None and ticks > hi):
            raise OneWireError(
                f"{what} is {value / 1000.0:.1f} us at CLKDIV {clkdiv}, outside the "
                f"{(lo or 0) * TICK_NS / 1000.0:.0f}..{(hi or 0) * TICK_NS / 1000.0:.0f} us "
                "window"
            )
    return lay


def limits(clkdiv: int) -> dict:
    """Every timing this program produces at `clkdiv`, in ns and in slots per second.

    One SM cycle is `clkdiv` core cycles and one tick is `TICK_NS`, so an SM-cycle count
    becomes a time by `sm * clkdiv * TICK_NS`. These are the numbers the README quotes
    and the ones the tests compare with the checker's measurements.
    """
    if not 1 <= clkdiv <= 0xFFFF:
        raise ValueError(f"CLKDIV integer part {clkdiv} is out of range")
    scale = clkdiv * TICK_NS
    return {
        "clkdiv": clkdiv,
        "sm_ns": scale,
        "reset_low_ns": RESET_LOW * scale,
        "reset_wait_ns": RESET_WAIT * scale,
        "reset_to_slot_ns": RESET_TO_SLOT * scale,
        "presence_sample_ns": (presence_cycles(clkdiv) * clkdiv - 3) * TICK_NS,
        "low1_ns": SLOT_LOW_SHORT * scale,
        "low0_ns": SLOT_LOW_LONG * scale,
        "slot_ns": SLOT_PERIOD * scale,
        "sample_ns": SLOT_SAMPLE * scale,
        "slot_rate_hz": 1e9 / (SLOT_PERIOD * scale),
        "byte_ns": SLOT_PERIOD * BYTE_SLOTS * scale,
        "byte_rate_hz": 1e9 / (SLOT_PERIOD * BYTE_SLOTS * scale),
    }


def divider_window() -> tuple:
    """The CLKDIV values whose timings all land inside the standard-speed windows.

    Every timing is a whole number of SM cycles, so each datasheet window becomes a range
    of CLKDIV and the program is legal exactly where all of them overlap: tRSTL
    480..640 us, tLOW1 1..15 us, tLOW0 60..120 us, tSLOT 60..120 us and tRSTH >= 480 us.
    `test_the_divider_window_is_where_the_tables_say` measures both ends on the ISS and
    compares them with this.
    """
    windows = (
        (RESET_LOW, 480_000.0, 640_000.0),        # tRSTL, AN126 H for the maximum
        (SLOT_LOW_SHORT, 1_000.0, 15_000.0),      # tLOW1
        (SLOT_LOW_LONG, 60_000.0, 120_000.0),      # tLOW0
        (SLOT_PERIOD, 60_000.0, 120_000.0),       # tSLOT
        (RESET_TO_SLOT, 480_000.0, float("inf")),  # tRSTH
    )
    fastest, slowest = 1, 0xFFFF
    for sm, lo_ns, hi_ns in windows:
        span = sm * TICK_NS
        fastest = max(fastest, int(-(-int(lo_ns) // span)))    # ceil(lo_ns / span)
        if hi_ns != float("inf"):
            slowest = min(slowest, int(hi_ns) // span)
    return int(fastest), int(slowest)


class OneWireDriver:
    """Load fw/onewire/program.pio into one engine and drive 1-Wire transactions with it.

    The engine is the master. The host's whole job is to hand the program one 32-bit word
    per byte and to read the bytes back, because the program does the waveform: it cannot
    be asked to do anything else, since there is no IRQ and no way to rewrite IMEM while it
    runs (docs/mvp-isa.md `## Differences from RP2040`).

    Every phase ends with the engine stopped on `pull block` with the line released, and
    that is the only synchronisation point in the driver: `wait_boundary()` watches
    `SMx_ADDR` for it. Nothing here needs to know how long a slot is, which is why the
    driver keeps working when the divider is changed for a different speed.
    """

    def __init__(
        self,
        bus,
        device,
        program: Program,
        clkdiv: int = DEFAULT_CLKDIV,
        engine: int = 0,
        dq_pin: int = DEFAULT_DQ_PIN,
        check_timing: bool = True,
    ) -> None:
        if not 0 <= dq_pin <= 7:
            raise ValueError(
                f"dq_pin={dq_pin}: open drain needs a uio pin, and the bidirectional pins "
                "are 0..7 (docs/mvp-isa.md `### Open drain`, `### The 18 protocol pins`)"
            )
        if clkdiv not in range(1, 0x10000):
            raise ValueError(f"CLKDIV integer part {clkdiv} is out of range")
        # program.pio is written for CLKDIV 50; the presence pad is scaled to whatever
        # divider this driver was asked for before anything is loaded (see `retarget`).
        self.prog = program = retarget(program, clkdiv)
        self.layout = check_program(program, clkdiv, timing=check_timing)
        self.bus = bus
        self.dev = device
        self.engine = engine
        self.dq_pin = dq_pin
        self.clkdiv = clkdiv
        self.peak_tx = self.peak_rx = 0
        self.stalls = 0
        self._pull = program.labels["w_byte"]
        self._reset = program.labels["reset"]
        self._park = program.labels["init"]
        # The states in which the engine generates no slots: stopped on `pull block`
        # between two bytes, or idling in the park before the first reset. Both have the
        # line released, so the host may change its mind in either.
        self._stopped = frozenset((self._pull, self._park,
                                   program.labels["next_byte"]))

    # ---- the register words the program needs --------------------------------------
    @property
    def clkdiv_word(self) -> int:
        return (self.clkdiv & 0xFFFF) << 16

    @property
    def execctrl_word(self) -> int:
        """The wrap window and nothing else: no side-set (the program side-sets nothing,
        so SIDE_EN would be a NOP bit) and no SIDE_PINDIR. Running off either end of the
        program lands in `init`, where the line is released."""
        return ((self.prog.wrap & 0x1F) << 12) | ((self.prog.wrap_target & 0x1F) << 7)

    @property
    def shiftctrl_base_word(self) -> int:
        """PULL_THRESH 8, PUSH_THRESH 8, OUT_SHIFTDIR 1, IN_SHIFTDIR 0, AUTOPULL off.

        PULL_THRESH 8 is what makes `jmp !osre` count eight slots per TX word, OUT_SHIFTDIR
        1 walks the OSR from bit 0, which is 1-Wire's least-significant-bit-first order, and
        IN_SHIFTDIR 1 leaves a sampled byte right-aligned in bits 31:24 in wire order.
        AUTOPULL is off
        because the program pulls explicitly and `pull block` is a no-op with autopull on
        and a full OSR (docs/mvp-isa.md `### Instruction semantics`, PULL).
        """
        return (8 << 25) | (8 << 20) | (1 << 19) | (1 << 18)

    def shiftctrl_word(self, autopush: bool = False) -> int:
        """The same word with AUTOPUSH set for a read phase.

        The program samples the line in both branches of a slot, so with AUTOPUSH on it
        would push a byte of 1s for every eight write-1 slots. A read phase therefore
        turns it on and a write phase turns it off; see the module docstring.
        """
        return self.shiftctrl_base_word | ((1 << 16) if autopush else 0)

    @property
    def pinctrl_word(self) -> int:
        """One pin in the SET window and one pin in the IN window, both at `dq_pin`.

        SET_COUNT 1 is all the program needs: every pin write in it is `set pindirs`, and
        the open-drain line is a direction, so the level register is never written and
        stays 0. There is no OUT window (the program writes no pins with `out`, it uses
        `out x, 1`) and no side-set, so both of those counts are 0.
        """
        return (1 << 26) | (self.dq_pin << 15) | (self.dq_pin << 5)

    @property
    def sm(self) -> int:
        """Word address of this engine's `SMe_CLKDIV`."""
        return A_SM + 6 * self.engine

    # ---- register access -----------------------------------------------------------
    def step(self) -> None:
        """One core cycle: the ChipDevice performs at most one queued host access."""
        self.bus.step()

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.step()

    def reg_write(self, addr: int, data: int) -> None:
        """Queue a register write; it is performed on the next cycle the driver steps."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def reg_read(self, addr: int) -> int:
        """Queue a register read and step the bus until the answer is back."""
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.step()
            ticks += 1
            if ticks > PHASE_LIMIT:
                raise OneWireError(f"read of word address {addr:#04x} never came back")
        return future.result()

    def wait_until(self, what: Callable[[], bool], limit: int = PHASE_LIMIT,
                   what_for: str = "condition") -> None:
        ticks = 0
        while not what():
            self.step()
            ticks += 1
            if ticks > limit:
                raise OneWireError(f"{what_for} did not come true within {ticks} core cycles")

    # ---- engine setup --------------------------------------------------------------
    def load(self) -> None:
        """The host sequence of docs/mvp-regmap.md `### Typical host sequences`.

        Nothing is forced into the engine first: at reset the PC is 0, which is `init`, and
        every pin is an input, so the line is already released. The engine therefore starts
        in the park by itself, and there is no window in which it could drive the bus.
        """
        self.reg_write(A_CTRL, 0)                                        # stop the engine
        for k, word in enumerate(self.prog.instrs):                      # IMEM burst
            self.reg_write(A_IMEM[self.engine] + k, word)
        self.reg_write(self.sm + SM_CLKDIV, self.clkdiv_word)
        self.reg_write(self.sm + SM_EXECCTRL, self.execctrl_word)
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word())
        self.reg_write(self.sm + SM_PINCTRL, self.pinctrl_word)
        self.reg_write(A_CTRL, self._ctrl_start)                         # enable + restart
        self.wait_boundary()

    @property
    def _ctrl_start(self) -> int:
        """CTRL with this engine enabled and its divider restarted, so the first SM cycle
        is two core cycles after this access (docs/mvp-isa.md `### Clock divider`)."""
        e = self.engine
        return (1 << (8 + e)) | (1 << (4 + e)) | (1 << e)

    def force(self, word: int) -> None:
        """Push one instruction into the engine's EXEC latch.

        It executes in the next core cycle whatever SM_ENABLE and the divider say, it
        does not advance the PC and its delay is ignored (docs/mvp-isa.md `### EXEC`).
        """
        self.reg_write(self.sm + SM_INSTR, word & 0xFFFF)

    @property
    def pc(self) -> int:
        """`SMx_ADDR`, the engine's PC. There is no IRQ in the MVP, so this and the FIFO
        levels are all the host can see of the program's progress."""
        return self.reg_read(self.sm + SM_ADDR) & 0x1F

    @property
    def at_boundary(self) -> bool:
        """True when the engine is generating no slots: stopped on `pull block` between
        two bytes, or idling in the park.

        This is the only state in which it is safe for the host to change its mind: the line
        is released, no slot is half way through, and the next thing that happens is a pull
        of whatever word the host pushes next. Before the first transaction the engine is in
        the park rather than on `pull block`, so both count.
        """
        return self.pc in self._stopped

    def wait_boundary(self, limit: int = PHASE_LIMIT) -> None:
        """Step until the engine is generating no slots. The one synchronisation point of
        the whole driver: every phase ends here and every phase starts from here."""
        self.wait_until(lambda: self.at_boundary, limit,
                        "the engine reaching a byte boundary")

    def wait_left_boundary(self, limit: int = PHASE_LIMIT) -> None:
        """Step until the engine has left the boundary, i.e. a forced jump landed.

        A forced instruction runs in the *next* core cycle, so reading SM0_ADDR right after
        writing SM0_INSTR still shows the old PC: without this the caller would go on
        believing the phase it just started has finished.
        """
        self.wait_until(lambda: not self.at_boundary, limit,
                        "the engine leaving the byte boundary")

    def fdebug(self) -> int:
        return self.reg_read(A_FDEBUG)

    def clear_fdebug(self) -> None:
        self.reg_write(A_FDEBUG, 0xFFFF_FFFF)

    def stall_flags(self) -> dict:
        """FDEBUG of engine 0, split into the four reasons.

        `tx_stall` is expected here and not a defect: the program blocks on `pull block`
        between bytes by design, and that is a TXSTALL (docs/mvp-isa.md `### Stalls, delay
        and side-set, summarised`). The other three are the host's own mistakes.
        """
        flags = self.fdebug()
        return {
            "rx_stall": bool(flags & FDEBUG_RXSTALL0),
            "rx_under": bool(flags & FDEBUG_RXUNDER0),
            "tx_over": bool(flags & FDEBUG_TXOVER0),
            "tx_stall": bool(flags & FDEBUG_TXSTALL0),
        }

    # ---- the FIFOs ------------------------------------------------------------------
    def _tx_level(self) -> int:
        level = (self.reg_read(A_FLEVEL) >> FLEVEL_TX0) & 0xF
        self.peak_tx = max(self.peak_tx, level)
        return level

    def _rx_level(self) -> int:
        level = (self.reg_read(A_FLEVEL) >> FLEVEL_RX0) & 0xF
        self.peak_rx = max(self.peak_rx, level)
        return level

    @property
    def peak_fifo_depth(self) -> int:
        """The deepest either FIFO got. Both are four deep (docs/mvp-regmap.md), so
        anything below four means the host kept up with the engine instead of leaning on
        its stalls."""
        return max(self.peak_tx, self.peak_rx)

    def _push_word(self, word: int) -> None:
        """Queue one byte for the program, waiting for room in the TX FIFO first."""
        self.wait_until(lambda: self._tx_level() < regmap.fifo_full_level(), what_for="a free TX FIFO slot")
        self.reg_write(A_TXF0, word)

    def _take_word(self) -> int:
        """Wait for one word in the RX FIFO and take it."""
        self.wait_until(lambda: self._rx_level() > 0, what_for="a word in the RX FIFO")
        return self.reg_read(A_RXF0)

    # ---- the transaction steps ------------------------------------------------------
    def reset(self) -> bool:
        """A reset pulse, then report whether a slave answered with a presence pulse.

        The pulse itself is the slave's: it starts 15..60 us after the release and lasts
        60..240 us, so a master that samples the line once has to do it where every legal
        pulse is still low, which is 60..75 us after the release -- the union of "starts no
        later than 60" and "lasts at least 60". The program samples at 65 us and pushes the
        bit into the RX FIFO, which is the only way the host learns about it: the MVP has no
        IRQ, no status register for a pin and no way to write the ISR from outside
        (docs/mvp-regmap.md `### Address map`).

        Returns True when the line was low at the sample point, i.e. a device answered.
        The pushed word holds one sampled bit, at bit 31: the ISR shifts right, so the only
        bit it holds is the one just sampled, at the top.
        """
        self.wait_boundary()
        # A reset is not a read phase: with AUTOPUSH on, the sample this sequence makes
        # would autopush a second word into the RX FIFO.
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word())
        self.force(jmp_word(self._reset))
        self.wait_left_boundary()        # the forced jump landed; the pulse is running
        self.wait_boundary()             # pulse, sample, push and the tRSTH wait are done
        return ((self._take_word() >> RX_PRESENT_BIT) & 1) == 0

    def write(self, data: Sequence[int]) -> None:
        """Send bytes, one 32-bit word each, and stop on the byte boundary.

        The engine blocks on `pull block` until the next word is there, so this waits for
        the byte it pushed to be *transmitted* rather than filling the FIFO and hoping.
        Waiting for the boundary alone would not do: a word pushed while the engine is
        still sitting on `pull block` leaves the PC there for another 50 core cycles, so
        the boundary would read as reached before the byte had started. Waiting for the
        engine to leave the boundary first, and only then for it to come back, is what
        makes this exact for any divider.

        It is also what keeps the gap between two slots legal. The engine's own tail is
        72 SM cycles after the last fall, and the host is a couple of core cycles behind
        that, so the period between two slots is the 76 SM cycles of `word_period` with a
        little on top, comfortably inside tSLOT's 60..120 us.
        """
        if not data:
            return
        for byte in data:
            self._push_word(tx_word(byte))
            self.wait_left_boundary()      # the word was taken and the byte started
            self.wait_boundary()           # and the byte is on the wire

    def read(self, count: int) -> bytes:
        """Read `count` bytes: `count` words of 0x00000000 in, `count` words out.

        AUTOPUSH is on for the whole phase and the ISR is cleared first, because the
        samples the write slots in front of this one left eight 1-bits per byte in it (the
        program cannot tell a read slot from a write-1 slot: the difference is only whether
        the host looks at the RX FIFO).
        """
        if count < 0:
            raise ValueError(f"a 1-Wire read cannot be {count} bytes")
        if not count:
            return b""
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word(autopush=True))
        self.force(mov_isr_null_word())          # one core cycle, no pin write, no PC change
        received: List[int] = []
        for _ in range(count):
            self._push_word(read_word())
            received.append(self._take_word())
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word())
        return bytes(rx_byte(word) for word in received)

    # ---- the command helpers, each one a reset plus a ROM and a function command -----
    def skip_rom(self, function: int) -> None:
        """reset, Skip ROM [CCh], one function command: addresses every slave at once."""
        self.reset()
        self.write((0xCC, function & 0xFF))

    def read_rom(self) -> bytes:
        """reset, Read ROM [33h], the 8 ROM bytes. Only legal with one slave on the bus."""
        self.reset()
        self.write((0x33,))
        return self.read(8)

    def match_rom(self, rom: Sequence[int]) -> None:
        """reset, Match ROM [55h], the 8 bytes to address: only that slave answers."""
        rom = bytes(rom)
        if len(rom) != 8:
            raise ValueError(f"a 1-Wire ROM code is 8 bytes, got {len(rom)}")
        self.reset()
        self.write((0x55,))
        self.write(rom)

    def read_scratchpad(self) -> bytes:
        """reset, Skip ROM [CCh], Read Scratchpad [BEh], the 9 scratchpad bytes."""
        self.skip_rom(0xBE)
        return self.read(9)

    def convert_t(self) -> bool:
        """reset, Skip ROM [CCh], Convert T [44h]. Returns the presence of a slave."""
        present = self.reset()
        self.write((0xCC, 0x44))
        return present

    def write_scratchpad(self, th: int, tl: int, config: int) -> bool:
        """reset, Skip ROM [CCh], Write Scratchpad [4Eh], then TH, TL and the config byte.

        All three bytes have to be written before the next reset, so this always sends
        exactly three.
        """
        present = self.reset()
        self.write((0xCC, 0x4E, th & 0xFF, tl & 0xFF, config & 0xFF))
        return present

    def copy_scratchpad(self) -> bool:
        """reset, Skip ROM [CCh], Copy Scratchpad [48h]: scratchpad bytes 2..4 to EEPROM."""
        present = self.reset()
        self.write((0xCC, 0x48))
        return present

    def recall_e2(self) -> bool:
        """reset, Skip ROM [CCh], Recall E2 [B8h]: the EEPROM back into the scratchpad."""
        present = self.reset()
        self.write((0xCC, 0xB8))
        return present

    @property
    def park(self) -> None:
        """Leave the bit loop and let the engine idle in `init`, with the line released."""
        self.wait_boundary()
        self.force(jmp_word(self._park))
        self.wait_until(lambda: self.pc in (self._park, self.prog.labels["next_byte"]),
                        what_for="the engine parking")
