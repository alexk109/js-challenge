# SPDX-License-Identifier: Apache-2.0
"""The directed stimulus: two programs, their pins, their cycle counts.

`test/test.py`'s four chip-level tests pass the SDF-annotated gate-level netlist at the slow
corner at 20 ns, where STA says the chip misses by 9.895 ns.
The reason is that nothing in them measures a *cycle count*: every observation there is a
poll of a status line and an assertion about a value, so a path that arrives one core cycle
late is invisible.  This module is the part that measures cycle counts.

Which paths, and why these instructions: A script outside this repository translated the slow corner's 2 000 reported setup paths
from `_27307_/Q -> _29046_/D` into RTL registers.  The four that matter:

| path | STA slack at 20 ns | SDF cell delay | at 20 ns | at 30 ns |
|---|---|---|---|---|
| `sm_en[1] -> u_rxf.mem[40]` (the RX FIFO write) | -9.895 ns | 26.211 ns | misses by 6.211 | fits |
| `sm_en[1] -> ir[12]` (the fetch) | -9.514 ns | 25.721 ns | misses by 5.721 | fits |
| `div_en[0] -> u_rxf.mem[78]` (the RX FIFO write) | -5.012 ns | 21.605 ns | misses by 1.605 | fits |
| `div_en[0] -> ir[4]` (the fetch) | -4.307 ns | 20.996 ns | misses by 0.996 | fits |

Two launch registers, and that is the whole design of the stimulus:

* `sm_en` is the registered `CTRL.SM_ENABLE[e]`, so it transitions **once**, when the host
  starts the engine.  `CLKDIV = 1` makes that the only transition into the engine's datapath,
  which isolates it.
* `div_en` is the divider's `en` flop, so with an integer `CLKDIV = D > 1` it transitions
  **every D core cycles** -- once per SM cycle.  That is the recurring path, and a directed
  test that runs its engine at `CLKDIV = 1` never touches it.

So every check below is run at several divisors, and each one is measured from an edge the
*host* chooses: a `wait 1 pin` instruction, which parks the engine until a pin the host owns
goes high.  That matters because the alternative anchor -- an engine-produced pin -- sits on
the same late paths (`sm_en -> uo_out[6]` is -9.523 ns at the slow corner), so it would move
with the thing being measured.  The host's own write to `ui_in` moves with nothing.

The cycle counts below are derived from `docs/mvp-isa.md` (`### When an engine executes`,
`### Reference step`, `### Clock divider`, `### Cycle accounting`) and `docs/mvp-regmap.md`
(`### tx_ready and rx_valid`), with `IN_PIPE` and `OUT_PIPE` for `pio_chip`'s input and output
register stages.  They are exact, not bounds: a whole sweep of divider phases is run, so the
*set* of latencies is known, not just its spread.
"""

from pathlib import Path
from typing import List, Sequence

from asm.pioasm import Program, assemble

PROGRAM_PIO = Path(__file__).with_name("program.pio")

# ---------------------------------------------------------------------------------------------
# The programs
# ---------------------------------------------------------------------------------------------

_PROGRAMS = {p.name: p for p in assemble(PROGRAM_PIO.read_text())}


def _add(name: str, source: str) -> Program:
    """Assemble `source` and remember it under `name`, so `program(name)` finds it too."""
    found = assemble(source)
    if len(found) != 1:
        raise ValueError(f"expected one program in the {name} source, got {len(found)}")
    _PROGRAMS[name] = found[0]
    return found[0]


def program(name: str) -> Program:
    """One of this package's assembled programs, by name.

    The four in `program.pio` are assembled when the module is imported; `anchor` is the
    `docs/mvp-isa.md` `### Anchor trace` program, which is quoted from the document rather than
    written out twice, so it is assembled on first use.
    """
    if name == "anchor":
        return anchor_values()
    try:
        return _PROGRAMS[name]
    except KeyError:
        raise KeyError(f"no program {name!r} in {PROGRAM_PIO.name}: "
                       f"{sorted(_PROGRAMS)}") from None


#: The word `rxf_probe` pushes, and the words the host puts in engine 1's TX FIFO for it.
PUSH_WORD = 0x1122_3344
#: What the host writes to `TXF1` before starting: one word per request it will make.
TXF_WORDS = [0xA000_0000 | (i + 1) for i in range(4)]

# ---------------------------------------------------------------------------------------------
# Pins (docs/mvp-regmap.md `### Address map`: 0..7 uio, 8..12 ui_in[7:3], 13..17 uo_out[7:3])
# ---------------------------------------------------------------------------------------------

#: The pin each program waits on, and the top-level port bit it is.
#: `fetch_probe` and `fetch_control` are engine 0's; `rxf_probe` is engine 1's.
WAIT_PIN = {"fetch_probe": 8, "fetch_probe_late": 8, "fetch_control": 8, "rxf_probe": 9,
            "rxf_probe_late": 9}
#: `wait 1 pin, k` counts from `IN_BASE`, so `k` is 0 and `IN_BASE` is the pin number.
WAIT_INDEX = 0
#: The pin each program answers on.  `rxf_probe` answers with a push instead, so it has none.
ANSWER_PIN = {"fetch_probe": 16, "fetch_probe_late": 16, "anchor": 0, "fetch_control": 16}

#: pio_chip's input and output register stages (`src/pio_chip.sv`, T_IN = 3, T_OUT = 1).
#: The input path is what makes the host's write visible to an engine three core cycles late;
#: the output path is what makes a `set pins` visible on the pad one core cycle late.
IN_PIPE = 3
OUT_PIPE = 1

#: `uo_out[2]` = IRQ0 = `rx_valid`, the line a FIFO write shows up on
#: (docs/mvp-regmap.md `### tx_ready and rx_valid`).
BIT_RX_VALID = 2


def pin_port(pin: int, kind: str) -> tuple:
    """`(port, bit)` of one protocol pin as `kind`: "in" (what the host drives), "out" (what
    the chip drives onto a pad) or "oe" (a `uio` pin's direction).

    Pins 0..7 are the `uio` pins and have all three; 8..12 are input-only and 13..17 are
    output-only.  Both of those run *down* the port, in opposite directions, which is easy to
    get backwards: `src/tt_um_protocol_emulator.v` assigns
    `pin_in = {5'b00000, ui_in[7:3], uio_in_resolved}` and
    `uo_out = {pin_out[17:13], irq0, irq1, spi_miso}`, so

        pin 8 -> ui_in[3]  ... pin 12 -> ui_in[7]        (bit = pin - 5)
        pin 13 -> uo_out[3] ... pin 17 -> uo_out[7]      (bit = pin - 10)

    `docs/mvp-regmap.md` `### Address map` says the same in prose -- "`ui_in[7:3]` protocol pins
    12..8", "`uo_out[7:3]` protocol pins 17..13" -- and `test/test.py`'s
    `test_protocol_pin_map` has `bit = pin - IN_ONLY_BASE + 3` for the input side.  Note that
    `uo_out[2]` and `uo_out[1]` are the two interrupt lines and `uo_out[0]` is MISO, so the
    five PIO output pins are `uo_out[3]` upwards.
    """
    if pin < 8:
        ports = {"in": "uio_in", "out": "uio_out", "oe": "uio_oe"}
        if kind not in ports:
            raise ValueError(f"kind must be one of {sorted(ports)}")
        return ports[kind], pin
    if pin < 13:
        if kind != "in":
            raise ValueError(f"pin {pin} is input-only, so it has no {kind!r} signal")
        return "ui_in", pin - 5
    if kind != "out":
        raise ValueError(f"pin {pin} is output-only, so it has no {kind!r} signal")
    return "uo_out", pin - 10


def net_of(pin: int) -> str:
    """The `verif.vip.core.Bus` net name of one protocol pin: `p<pin>` (`docs/mvp-interfaces.md`)."""
    return f"p{pin}"


# ---------------------------------------------------------------------------------------------
# The register words (docs/mvp-regmap.md `### Address map` and the per-register field tables).
# The field positions are the firmware drivers' (`fw/uart_rx/driver.py`); nothing is imported
# from them, because these programs are not theirs and their programs set fields these do not.
# ---------------------------------------------------------------------------------------------

CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100


def ctrl_bits(engine: int, *fields: int) -> int:
    """The `CTRL` bits named, each moved up to this engine's bit."""
    mask = 0
    for field in fields:
        mask |= field << engine
    return mask


def sm_clkdiv(divisor: int) -> int:
    """`SMe_CLKDIV` for an integer number of core cycles per SM cycle (`INT` in 31:16)."""
    if not 1 <= divisor <= 0xFFFF:
        raise ValueError(f"CLKDIV must be 1..65535 core cycles per SM cycle, got {divisor}")
    return divisor << 16


def sm_execctrl(wrap_bottom: int = 0, wrap_top: int = 31, jmp_pin: int = 0) -> int:
    """`SMe_EXECCTRL`: `JMP_PIN` in 28:24 (an absolute pin index) and the wrap window in 12:7.

    `docs/mvp-isa.md` `### Field values`: "**EXECCTRL.JMP_PIN**, absolute index", and the JMP
    `PIN` condition reads `IN_VEC[JMP_PIN]` -- which is what lets a program branch on a pin the
    host drives.
    """
    return (jmp_pin << 24) | (wrap_top << 12) | (wrap_bottom << 7)


def sm_shiftctrl() -> int:
    """`SMe_SHIFTCTRL`: right shifts, no autopush, no autopull, both thresholds 32 (never)."""
    return (1 << 19) | (1 << 18)


def sm_pinctrl(in_base: int = 0, set_base: int = 0, set_count: int = 0) -> int:
    """`SMe_PINCTRL`: the IN window at 14:15, the SET window at 26:5, no OUT or side-set."""
    return (set_count << 26) | (in_base << 15) | (set_base << 5)


def sm_addr(engine: int, index: int) -> int:
    """Word address of `SMe_<register>` (`0x32 + 6 * engine + index`, index 0 = CLKDIV)."""
    return 0x32 + 6 * engine + index


def imem_addr(engine: int, word: int) -> int:
    """Word address of `INSTR_MEM<word>` of `engine` (`0x12 + 0x40 * engine + word`)."""
    return 0x12 + 0x40 * engine + word


def config_for(name: str, divisor: int, wrap: tuple = (0, 31)) -> dict:
    """The four `SMe_*` words one of these programs needs, plus the CTRL mask to start it.

    `wrap` is `(WRAP_BOTTOM, WRAP_TOP)`: the `anchor` program needs the documented window
    1..2 and the loops wrap 31 -> 0 (the reset values), which is the same thing here.
    """
    if name == "anchor":
        return {"execctrl": sm_execctrl(*wrap), "pinctrl": sm_pinctrl(set_base=0, set_count=1)}
    wait_pin = WAIT_PIN[name]
    jmp_pin = wait_pin if name == "fetch_control" else 0
    return {"execctrl": sm_execctrl(*wrap, jmp_pin=jmp_pin),
            "pinctrl": sm_pinctrl(in_base=wait_pin, set_base=ANSWER_PIN.get(name, 0),
                                  set_count=1 if name != "rxf_probe" else 0)}


# ---------------------------------------------------------------------------------------------
# The cycle counts.  Every one of them is derived here, from the ISA, in one place, so that the
# two front-ends (test/test_slowpath.py on the pins, fw/slowpath/test_slowpath.py on the ISS)
# cannot disagree about what they expect.  `D` is an integer CLKDIV: core cycles per SM cycle.
#
# The shape of all of them is the same.  The host drives its pin high for the first time during
# core cycle `T`.  `pio_chip`'s input pipeline puts that level in `IN_VEC` for core cycle
# `T + IN_PIPE`, so the `wait` completes in the first SM cycle `S >= T + IN_PIPE`, and
# `S - T` is one of `IN_PIPE .. IN_PIPE + D - 1` -- which one depends on where `T` falls
# against the divider, and the host cannot know that, because the divider's phase is set by the
# CTRL write and the host port does not tell it when that write landed.  So a check does not
# assert one latency: it drives at every phase and asserts the whole *set*.
# ---------------------------------------------------------------------------------------------


def phases(divisor: int) -> int:
    """How many releases it takes to cover every divider phase: one per core cycle of the
    divider's period (`docs/mvp-isa.md` `### Clock divider`: `en` is high every `D` cycles)."""
    return divisor


#: How many extra SM cycles each mutant program puts in front of its answer, and so how much
#: later its answer must be.  This is the negative control's whole claim: one `nop` is `D` core
#: cycles, and the measurement has to show exactly that.
MUTANT_EXTRA_SM_CYCLES = {"fetch_probe_late": 1, "rxf_probe_late": 1}


def fetch_latency_set(divisor: int, name: str = "fetch_probe") -> set:
    """Core cycles from the host's pin edge to the answer's pin edge, for `fetch_probe`.

    `wait` completes in SM cycle `S`; `set pins, 1` executes in SM cycle `S+1`, which is `D`
    core cycles later; the level register is written at the end of that core cycle and the pad
    follows `OUT_PIPE` core cycles after it.  So `(S - T) + D + OUT_PIPE`, over all phases.
    A mutant program's extra `nop`s shift the whole set by that many SM cycles.
    """
    extra = MUTANT_EXTRA_SM_CYCLES.get(name, 0) * divisor
    low = divisor + IN_PIPE + OUT_PIPE + extra
    return set(range(low, low + divisor))


def fetch_pulse_width(divisor: int) -> int:
    """Core cycles the answer pin is high: `set pins, 1` and `set pins, 0` are one SM cycle
    apart (`fetch_probe`'s addresses 1 and 2), so the pulse is `D` core cycles wide."""
    return divisor


def rxf_irq_latency_set(divisor: int, name: str = "rxf_probe") -> set:
    """Core cycles from the host's pin edge to `rx_valid`, for `rxf_probe`.

    `wait` completes in SM cycle `S`; `pull` in `S+1`, `mov isr, osr` in `S+2` and `push` in
    `S+3`, so the FIFO write happens at the end of core cycle `S + 3 * D`.
    `docs/mvp-regmap.md` `### tx_ready and rx_valid`: the interrupt pins are flops, so a FIFO
    change at the end of cycle `n` shows on `rx_valid` from cycle `n + 2`.  So
    `(S - T) + 3 * D + 2`, over all phases.
    """
    extra = MUTANT_EXTRA_SM_CYCLES.get(name, 0) * divisor
    low = 3 * divisor + IN_PIPE + 2 + extra
    return set(range(low, low + divisor))


def control_period(taken: bool, divisor: int) -> int:
    """Core cycles between two rising edges of `fetch_control`'s answer pin.

    `fetch_control` reads the host's own pin into `x` (`in x, 1`) and branches on it, so the
    *same* program takes a different path depending on what the host drives, and the two paths
    have different lengths:

        0: set pins, 1      the answer rises
        1: set pins, 0      and falls, so the next iteration is an edge too
        2: jmp pin, 5        taken when the host's pin is high -> 5, not taken -> 3
        3: nop              the two instructions a taken branch skips
        4: nop
        5: jmp 0

    taken (the host's pin high): 0, 1, 2, 5 -- four SM cycles per iteration.
    not taken (the pin low): 0, 1, 2, 3, 4, 5 -- six.  A taken JMP costs one SM cycle like any
    other instruction (`docs/mvp-isa.md` `### Cycle accounting`), so the difference is exactly
    the two instructions the branch skips, and the answer's period is the loop's length.
    """
    return (4 if taken else 6) * divisor


#: The `docs/mvp-isa.md` `### Anchor trace` program, and its table.  This is the document's own
#: cycle-by-cycle example, so it is the one expectation here that is not derived but quoted.
#: Engine 0, `CLKDIV = 1.0`, `WRAP_BOTTOM = 1`, `WRAP_TOP = 2`, IMEM
#: `0: set pindirs,1 / 1: set pins,1 / 2: set pins,0`, and `CTRL = 0x101` written in cycle `h`:
#:
#:   | Cycle | en | executes            | uio_oe[0] | uio_out[0] |
#:   | h+1   | 0 | --                  | 0         | 0          |
#:   | h+2   | 1 | PC 0 set pindirs,1  | 0         | 0          |
#:   | h+3   | 1 | PC 1 set pins,1     | 1         | 0          |
#:   | h+4   | 1 | PC 2 set pins,0     | 1         | 1          |
#:   | h+5   | 1 | PC 1 set pins,1     | 1         | 0          |
ANCHOR_PIO = """
.program anchor
set pindirs, 1
set pins, 1
set pins, 0
"""
#: `(uio_oe[0], uio_out[0])` for cycles `h+1 .. h+5`, as the table above.
ANCHOR_TABLE = ((0, 0), (0, 0), (1, 0), (1, 1), (1, 0))
#: The SM cycle each of those is, relative to `h`: `h+1` executes nothing and `h+2..h+5` are the
#: first four SM cycles, so the values belong to SM cycles 0..3 as listed here.
ANCHOR_SM_CYCLES = (None, 0, 1, 2, 3)


def anchor_values() -> "Program":
    """The anchor program, assembled (it is not in `program.pio`: it is the doc's, verbatim)."""
    return _add("anchor", ANCHOR_PIO)


def anchor_expected(divisor: int, cycles: int) -> List[tuple]:
    """`(uio_oe[0], uio_out[0])` for the `cycles` core cycles from the anchor program's own
    first transition, at an integer `CLKDIV`.

    The anchor program writes the direction once (`set pindirs,1` at its first SM cycle) and then
    alternates the level on every SM cycle, because `WRAP_BOTTOM = 1` sends it back to address 1
    after address 2 -- no JMP at all.  The pad is `OUT_PIPE` core cycles behind the instruction,
    so the first transition (`uio_oe[0]` rising) is one core cycle after that first SM cycle and
    the level then changes every `divisor` core cycles from there.
    """
    return [(1, (i // divisor) % 2) for i in range(int(cycles))]


def anchor_gaps(divisor: int) -> List[int]:
    """Core cycles between consecutive transitions of the anchor program's two signals.

    `ANCHOR_SM_CYCLES` says which SM cycle each documented value belongs to, and the pad is
    `OUT_PIPE` core cycles behind the instruction, so a transition lands `OUT_PIPE` core
    cycles after the start of its SM cycle.  The gaps between transitions are therefore
    `divisor` times the gaps between the SM cycles, which is the part of the table that does
    not depend on where the CTRL write landed.
    """
    sm = [s for s in ANCHOR_SM_CYCLES if s is not None]
    return [divisor * (b - a) for a, b in zip(sm, sm[1:])]


# ---------------------------------------------------------------------------------------------
# Measuring: one recorder, and the arithmetic both front-ends use on what it recorded
# ---------------------------------------------------------------------------------------------


def latencies_from(releases: Sequence[int], edges: Sequence[int]) -> List[int]:
    """`edge - release` for each pair, refusing to pair up events that cannot be paired."""
    if len(releases) != len(edges):
        raise AssertionError(
            f"{len(releases)} releases but {len(edges)} edges: a release went unanswered, or an "
            f"edge arrived that no release asked for")
    bad = [(r, e) for r, e in zip(releases, edges) if e < r]
    if bad:
        raise AssertionError(f"edges before their release: {bad[:3]}")
    return [e - r for r, e in zip(releases, edges)]


def rising_edges(samples: Sequence[int]) -> List[int]:
    """The indices at which a sampled level went 0 -> 1.  Index 0 counts as no edge."""
    return [i for i in range(1, len(samples)) if samples[i - 1] == 0 and samples[i] == 1]


def check_set(what: str, got: Sequence[int], wanted: set) -> None:
    """Assert the *set* of measured latencies is exactly `wanted`, and say what differed.

    A set, not a sorted list: the order in which the phases come out depends on where the host's
    first release fell against the divider, and two releases at different phases can legitimately
    land on the same latency.  What must hold is that the *distinct* latencies observed are the
    ones the ISA says are possible, and that every one of the phases produced one.
    """
    got_set = set(got)
    if got_set != wanted or len(got) != len(wanted):
        raise AssertionError(
            f"{what}: measured latencies {sorted(got)} (distinct {sorted(got_set)}), "
            f"expected the set {sorted(wanted)} from {len(wanted)} releases; "
            f"missing {sorted(wanted - got_set)}, unexpected {sorted(got_set - wanted)}")


def check_exact(what: str, got, wanted) -> None:
    """Assert one measured number equals what the ISA says, with the measurement in the message."""
    if got != wanted:
        raise AssertionError(f"{what}: measured {got}, the ISA says {wanted}")

# ---------------------------------------------------------------------------------------------
# The host's release schedule, shared by both front-ends
# ---------------------------------------------------------------------------------------------


#: SM cycles from the answering `wait` back to the next `wait`: `rxf_probe` answers on its
#: fourth instruction (`wait`, `pull`, `mov isr, osr`, `push`, `jmp 0`), so the pin has to be
#: low again by then, or the `wait` completes a second time off one release.
LOOP_SM_CYCLES = 4


def hold_cycles(divisor: int) -> int:
    """How long the host holds its release pin high, in core cycles.

    The pin has to be high in the SM cycle that answers the release and low again in the SM
    cycle `LOOP_SM_CYCLES` later, when the program loops back round to its `wait`: high, and the
    `wait` answers twice off one release; low too early, and it never answers at all.

    The answering SM cycle `S` sits at `T + IN_PIPE + j` with `j` in `0 .. D-1` (which phase of
    the divider the release landed on), and the next `wait` is `LOOP_SM_CYCLES * D` core cycles
    after `S`.  So the pin's high window, measured from `T`, has to be inside

        [IN_PIPE + j + 1,  IN_PIPE + j + LOOP_SM_CYCLES * D]   for every j in 0 .. D-1

    and `IN_PIPE + 2 * D` is inside that for every divisor.  Being one cycle out is enough to go
    wrong, in both directions, and both were measured on the ISS while this was written: a hold
    one cycle too long let `fetch_probe` answer again on its own, so the second release's "first"
    edge belonged to the first release, and a hold too short left the engine free-running with two
    words in the RX FIFO per release, so `rx_valid` never fell again.
    """
    return IN_PIPE + 2 * divisor


def shift_hold(divisor: int) -> int:
    """How long the host holds the pin high for a `shift_probe` release.

    `shift_probe` pushes four words per release, so its high window has to cover the *fourth*
    shift -- SM cycle `S + 4`, which reads the pin `IN_PIPE` core cycles earlier, i.e. core cycle
    `S + 1` at worst `IN_PIPE + D` after the release -- and still end before the engine loops back
    to its `wait` (`shift_probe` takes five SM cycles, so the loop-back reads the pin at
    `IN_PIPE + 5 * D` or later).  The low level is *driven* in core cycle `release + hold`, so the
    pin is high during `release .. release + hold - 2` -- one cycle less than the arithmetic
    above suggests, and measured: one cycle short reads as the last two words being `0,0`.

    So `IN_PIPE + D + 2`, which is inside both windows.  It is shorter than `hold_cycles` because
    the window is bounded by the loop-back rather than by the answer.
    """
    return IN_PIPE + divisor + 2


def release_after(cycle_now: int, residue: int, divisor: int, lead: int = 2) -> int:
    """The first core cycle at or after `cycle_now + lead` whose index is `residue` mod `D`.

    The divider's SM cycles sit on one residue class of core cycles modulo `D`, and the host
    cannot know which -- the CTRL write that set the phase landed at a cycle the host port does
    not report -- but it can choose its own residue, and a release in core cycle `T` is answered
    by the first SM cycle at or after `T + IN_PIPE`.  So `T mod D` alone decides which phase of
    the divider answers it: one residue class gives one latency, and `D` releases with `D`
    different residues give every latency the ISA allows, which is what `check_set` then holds
    the measurement to.
    """
    want = residue % divisor
    cycle = int(cycle_now) + lead
    while cycle % divisor != want:
        cycle += 1
    return cycle


# ---------------------------------------------------------------------------------------------
# The host-side register addresses the tests reach for (docs/mvp-regmap.md `### Address map`)
# ---------------------------------------------------------------------------------------------

A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03


def TXF(engine: int) -> int:
    """`TXFn`: engine `e`'s TX FIFO data register (0x04 for engine 0, 0x05 for engine 1)."""
    return 0x04 + engine


def RXF(engine: int) -> int:
    """`RXFn`: engine `e`'s RX FIFO data register (0x08 for engine 0, 0x09 for engine 1)."""
    return 0x08 + engine


def FLEVEL_RX(engine: int) -> int:
    """The `FLEVEL` bit shift that puts engine `e`'s RX FIFO occupancy at bits 0."""
    return 8 * engine + 4
