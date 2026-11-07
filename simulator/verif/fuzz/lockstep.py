"""Lockstep fuzz: the RTL (`pio_chip`) against the golden ISS, cycle for cycle.

This module is
both

* the cocotb test that drives `src/pio_*.sv` under the simulator and applies the *same*
  stimulus to `iss.pio_iss.Chip`, and
* the command line driver (`python -m verif.fuzz.lockstep --seed 3 --cycles 10000`) that builds
  the simulator through `verif/fuzz/Makefile` and runs it once.

Every cycle it compares, after the values have settled and before the next edge:

* `pin_out`, `pin_oe`, `irq0`, `irq1` (chip outputs),
* `bus_rdata`, in the cycle of a host read (combinational, docs/mvp-interfaces.md
  `## Register bus`),
* every key of `Chip.state(e)`, for each engine, against the RTL signal of the same name inside
  `g_eng[e].u_eng` (docs/mvp-interfaces.md `### Lockstep visibility`),
* and, when `--deep` is on (the default, because seeds 1-60 at 20k cycles
  are clean), everything `Chip.state()` and `Chip.peek()` between them leave unobserved --
  the five holes `docs/mvp-verification-report.md` `### Known holes in the stimulus` lists:

  | deep field | RTL signal | ISS side |
  |---|---|---|
  | `sm_enable` | `dut.sm_enable` | `peek(A_CTRL)` |
  | `fdebug` | `dut.fdebug` | `peek(A_FDEBUG)`, lanes unpacked |
  | `inte0/1`, `intf0/1` | `dut.inte0` ... `dut.intf1` | `peek(A_IRQx_INTE/INTF)`, lanes unpacked |
  | `in_vec`, `in_pipe0` | `dut.in_vec`, `dut.in_pipe[0]` | `deep_state()["in_vec"]`, `sync[0]` |
  | `eng{e}.clkdiv/execctrl/shiftctrl/pinctrl` | `u_eng.r_clkdiv` ... | `peek(A_SM_BASE + 6e + i)` |
  | `eng{e}.div_cnt/div_acc/div_en` | `u_eng.u_div.cnt/acc/en` | `deep_state()[e]["div"]` |
  | `eng{e}.tx[i]`, `eng{e}.rx[i]` | `u_eng.u_txf.mem[(rp+i)%4]` | `deep_state()[e]["tx"/"rx"]` |
  | `eng{e}.imem[k]` | `u_eng.imem[k]` | `deep_state()[e]["imem"]` |
  | `eng{e}.os_we/os_dir/os_mask/os_data`, `ss_*` | `u_eng.os_*`, `u_eng.ss_*` | `deep_state()["req"]` |
  | `res_level`, `res_dir` | `dut.lvl_n`, `dut.dir_n` | `deep_state()["res_level"/"res_dir"]` |

  The last five rows are compared in the *second* half of the cycle (after the host access has
  been applied and the ISS has stepped), because they are combinational outputs of the requests
  the two sides make in that cycle; everything above it is flop state, compared where the current
  code compares it.

`--pullup` adds a board to the stimulus: a `uio` pin the chip releases reads 1 (an open-drain net
with a pull-up). `FUZZ_PAD_FAULT` is the negative control of the pad model and must stay unset.

On the first mismatch it stops, prints the cycle, the field and both values, the last
`FUZZ_WINDOW` cycles of both sides, and writes a self-contained repro (the seed, which fixes the
pins, plus the whole bus-op list) to `verif/fuzz/repros/`. `verif/fuzz/minimize.py` shrinks it.

Nothing in `src/` or `iss/` is touched here, and no mismatch is fixed here: a mismatch is a
finding to hand to the orchestrator.

Environment (all optional, read inside the simulator process):

| Variable | Meaning |
|---|---|
| `FUZZ_SEED` | seed for `verif.fuzz.gen.gen` (default 1) |
| `FUZZ_CYCLES` | core cycles to run (default 1000) |
| `FUZZ_REPRO` | path to a stimulus JSON to replay instead of generating one |
| `FUZZ_TRIVIAL` | if set, run the selftest stimulus `gen.trivial()` instead |
| `FUZZ_PINLOOP` | if set, run the pad-model stimulus `gen.pinloop()` instead |
| `FUZZ_FIFOFILL` | if set, run the FIFO-storage stimulus `gen.fifofill()` instead |
| `FUZZ_CONFLICT` | if set, `gen.gen(..., conflict=N)`: append a guaranteed pin-conflict phase |
| `FUZZ_DEEP` | `0`/`1`, force the deep comparison off/on (default `DEEP_DEFAULT`) |
| `FUZZ_PULLUP` | if set, an un-driven `uio` pin reads 1 |
| `FUZZ_PAD_FAULT` | if set, a *wrong* pad model (the negative control of `pad_value`) |
| `FUZZ_PAD_TRACE` | if set, record the per-cycle pad and `in_vec` in the result JSON |
| `FUZZ_WINDOW` | cycles of context in the report (default 20) |
| `FUZZ_RESULT` | path for the machine-readable result JSON |
| `FUZZ_REPRO_DIR` | where repros go (default `verif/fuzz/repros/`) |
| `FUZZ_QUIET` | if set, only print the verdict |
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cocotb                                                    # noqa: E402
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge    # noqa: E402

from verif.fuzz import gen                                       # noqa: E402

#: every key of `Chip.state(e)` (docs/mvp-interfaces.md `## ISS API`), which is also the list of
#: lockstep signal names in `g_eng[e].u_eng`.
STATE_KEYS = ("pc", "ir", "x", "y", "osr", "isr", "osr_cnt", "isr_cnt", "delay_cnt", "stalled",
              "latch", "latch_pend", "latch_host", "forced", "tx_level", "rx_level")
#: chip outputs compared every cycle, in report order
OUTPUT_KEYS = ("pin_out", "pin_oe", "irq0", "irq1")
READ_KEY = "bus_rdata"

#: The deep comparison is the default. It became one once seeds 1-60 at 20k cycles ran clean
#: with it (60/60 seeds matched over 157 fields); `--no-deep` keeps the 37 shallow signals, which is what the
#: deep negative controls are compared against.
DEEP_DEFAULT = True

#: pins the outside world can drive; the harness models the pad on top of this
UIO_MASK = gen.UIO_MASK
IN_ONLY_MASK = gen.IN_ONLY_MASK
OUT_ONLY_MASK = gen.OUT_ONLY_MASK
PIN_MASK = 0x3FFFF
#: bits of the register file that hold a pin level / a pin direction (docs/mvp-isa.md
#: `### The 18 protocol pins`); writes outside them are dropped by the spec, so the harness masks
#: them away on both sides rather than comparing a value the chip is allowed to ignore.
LEVEL_WRITABLE = UIO_MASK | OUT_ONLY_MASK
DIR_WRITABLE = UIO_MASK

CLOCK_NS = 10.0                # a 100 MHz core clock; the period is irrelevant, the edges are not


# ---------------------------------------------------------------------------------------
# stimulus
# ---------------------------------------------------------------------------------------

def env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    return default if value in (None, "") else int(value)


def deep_enabled() -> bool:
    """Whether this run compares the deep fields: `FUZZ_DEEP` wins over `DEEP_DEFAULT`."""
    raw = os.environ.get("FUZZ_DEEP")
    if raw in (None, ""):
        return DEEP_DEFAULT
    return raw not in ("0", "no", "false")


def pullup_enabled() -> bool:
    return bool(os.environ.get("FUZZ_PULLUP"))


def apply_mode(stim: gen.Stimulus) -> None:
    """A repro carries the mode it was found in (`Stimulus.mode`); an explicit flag still wins.

    Without this, a deep finding replayed with `make fuzz-repro` would come back clean, because the
    shallow comparison cannot see the field that diverged.
    """
    if os.environ.get("FUZZ_DEEP") in (None, ""):
        if "deep" in stim.mode:
            os.environ["FUZZ_DEEP"] = "1" if stim.mode["deep"] else "0"
    if os.environ.get("FUZZ_PULLUP") in (None, "") and stim.mode.get("pullup"):
        os.environ["FUZZ_PULLUP"] = "1"


def load_stimulus() -> gen.Stimulus:
    """The stimulus to run: a repro file if FUZZ_REPRO is set, else a generated one."""
    path = os.environ.get("FUZZ_REPRO")
    if path:
        stim = gen.Stimulus.load(path)
        apply_mode(stim)
        if os.environ.get("FUZZ_CYCLES"):
            stim.cycles = min(stim.cycles, env_int("FUZZ_CYCLES", stim.cycles))
        return stim
    seed = env_int("FUZZ_SEED", 1)
    cycles = env_int("FUZZ_CYCLES", 1000)
    if os.environ.get("FUZZ_TRIVIAL"):
        return gen.trivial(seed=seed, cycles=cycles)
    if os.environ.get("FUZZ_PINLOOP"):
        return gen.pinloop(seed=seed, cycles=cycles)
    if os.environ.get("FUZZ_FIFOFILL"):
        return gen.fifofill(seed=seed, cycles=cycles)
    stim = gen.gen(seed, cycles, conflict=env_int("FUZZ_CONFLICT", 0))
    stim.mode = {"deep": deep_enabled(), "pullup": pullup_enabled()}
    return stim


def label() -> str:
    """What this build is: the real RTL, or which planted copy of it."""
    plant = os.environ.get("FUZZ_PLANTED")
    return "rtl" if not plant else f"planted{plant}"


# ---------------------------------------------------------------------------------------
# comparison
# ---------------------------------------------------------------------------------------

class Mismatch:
    """The first cycle in which the two sides disagree, and where."""

    def __init__(self, cycle: int, field: str, engine: Optional[int], rtl: int, iss: int) -> None:
        self.cycle = cycle
        self.field = field
        self.engine = engine
        self.rtl = rtl
        self.iss = iss

    @property
    def name(self) -> str:
        return f"eng{self.engine}.{self.field}" if self.engine is not None else self.field

    def as_dict(self) -> dict[str, Any]:
        return {"cycle": self.cycle, "field": self.field, "engine": self.engine,
                "name": self.name, "rtl": self.rtl, "rtl_hex": f"{self.rtl:#x}",
                "iss": self.iss, "iss_hex": f"{self.iss:#x}"}


#: deep chip-level fields, compared with the settled flop state (first half of the cycle)
CHIP_DEEP_KEYS = ("sm_enable", "fdebug", "inte0", "inte1", "intf0", "intf1", "in_vec", "in_pipe0")
#: deep per-engine flop state: the configuration registers, then the 16.8 divider
ENGINE_DEEP_KEYS = ("clkdiv", "execctrl", "shiftctrl", "pinctrl", "div_cnt", "div_acc", "div_en")
#: deep per-engine pin write requests, compared in the second half of the cycle
ENGINE_REQ_KEYS = ("os_we", "os_dir", "os_mask", "os_data", "ss_we", "ss_dir", "ss_mask", "ss_data")
#: the pin resolution of this cycle's requests, before the output register (T_OUT = 1)
COMBI_DEEP_KEYS = ("res_level", "res_dir")
#: how many words each FIFO holds (docs/mvp-isa.md `### Engine state`: a 4-deep FIFO per engine
#: and direction), so that the contents compare knows how many words are in the queue
FIFO_WORDS = 4


def field_names(deep: bool = True) -> list[str]:
    """Every field a run compares, in report order; `DeepProbe` appends in exactly this order."""
    names = list(OUTPUT_KEYS)
    names += [f"eng{e}.{k}" for e in range(gen.NUM_ENGINES) for k in STATE_KEYS]
    if deep:
        names += list(CHIP_DEEP_KEYS)
        for e in range(gen.NUM_ENGINES):
            names += [f"eng{e}.{k}" for k in ENGINE_DEEP_KEYS]
            names += [f"eng{e}.tx[{i}]" for i in range(FIFO_WORDS)]
            names += [f"eng{e}.rx[{i}]" for i in range(FIFO_WORDS)]
            names += [f"eng{e}.imem[{k}]" for k in range(gen.IMEM_SIZE)]
    names += [READ_KEY]
    if deep:
        # the requests before what they add up to: a report should name the cause before the effect
        names += [f"eng{e}.{k}" for e in range(gen.NUM_ENGINES) for k in ENGINE_REQ_KEYS]
        names += list(COMBI_DEEP_KEYS)
    return names


def unpack_lanes(word: int, count: int) -> int:
    """The low two bits of each of `count` 8-bit lanes, back to back (`lanes()` in `src/pio_chip.sv`)."""
    out = 0
    for i in range(count):
        out |= ((word >> (8 * i)) & 0x3) << (2 * i)
    return out


def unpack_fdebug(word: int) -> int:
    """The eight FDEBUG flags out of the 32-bit read value, in `dut.fdebug`'s bit order.

    The read mux packs them as four 8-bit lanes (`src/pio_chip.sv`: `lanes(fdebug[7:6],
    fdebug[5:4], fdebug[3:2], fdebug[1:0])`), lane `i` at bits `8i+1:8i`, holding TXSTALL, TXOVER,
    RXUNDER and RXSTALL in that order from the top; `Chip.peek(A_FDEBUG)` builds the same word
    straight from the flags. So this is the inverse of a documented packing, not a second
    implementation of the flags.
    """
    return unpack_lanes(word, 4)


def unpack_irq(word: int) -> int:
    """The four bits of IRQx_INTE or IRQx_INTF out of the 32-bit read value, in `dut.inte0`'s order.

    The chip stores each of those registers in four bits, `{TXNFULL[1:0], RXNEMPTY[1:0]}`
    (`src/pio_chip.sv`), and the read mux puts TXNFULL at bits 5:4 and RXNEMPTY at bits 1:0
    (`irq_word()`), which is where docs/mvp-regmap.md `## Registers` puts them, so a read of
    `IRQ1_INTE` returns 0x010 after reset. `Chip.peek` returns the register as stored, in those
    same bit positions, so putting the two back together is a shift rather than a guess.
    """
    return ((word >> 4) & 0x3) << 2 | (word & 0x3)


class History:
    """The last `window` cycles of both sides, for the report."""

    def __init__(self, window: int, deep: bool = True) -> None:
        self.window = window
        self.deep = deep
        self.names = field_names(deep)
        self.cycles: deque[int] = deque(maxlen=window)
        self.rtl: deque[list[int]] = deque(maxlen=window)
        self.iss: deque[list[int]] = deque(maxlen=window)
        self.bus: deque[str] = deque(maxlen=window)

    def add(self, cycle: int, rtl: list[int], iss: list[int], bus: str) -> None:
        assert len(rtl) == len(self.names) and len(iss) == len(self.names), (
            f"cycle {cycle}: {len(rtl)}/{len(iss)} values for {len(self.names)} fields")
        self.cycles.append(cycle)
        self.rtl.append(rtl)
        self.iss.append(iss)
        self.bus.append(bus)

    def add_partial(self, cycle: int, rtl: list[int], iss: list[int], bus: str) -> None:
        """A cycle that stopped half way through its fields (the run ends there): pad the rest with
        zeros on both sides, so that the report's row has every field and shows no difference in the
        ones that were never read."""
        pad = len(self.names) - len(rtl)
        self.add(cycle, rtl + [0] * pad, iss + [0] * pad, bus)

    def field_names(self) -> list[str]:
        return list(self.names)

    def as_dict(self) -> dict[str, Any]:
        names = self.names
        rows = []
        for cycle, rtl, iss, bus in zip(self.cycles, self.rtl, self.iss, self.bus):
            row = {"cycle": cycle, "bus": bus}
            for name, r, i in zip(names, rtl + [0] * (len(names) - len(rtl)),
                                   iss + [0] * (len(names) - len(iss))):
                row[name] = r if r == i else {"rtl": r, "iss": i}
            rows.append(row)
        return {"fields": names, "cycles": rows}


# ---------------------------------------------------------------------------------------
# the deep comparison
# ---------------------------------------------------------------------------------------

class DeepProbe:
    """The fields `--deep` adds, read out of the RTL and out of the ISS.

    Every read goes through a handle collected once (cocotb 2's `handle.value` is a snapshot), and
    every value goes through `check`, so a field cannot be read and then forgotten. The values come
    back in `field_names()` order.

    Two halves, because the RTL signals are of two kinds:

    * `state()` reads flop state in the first half of the cycle, where the shallow comparison
      reads its own: the configuration registers, the divider, the FIFO storage, the IMEM, FDEBUG,
      the input pipeline and the interrupt enable/flag registers.
    * `requests()` reads what the two sides are *asking for in this cycle*, in the second half,
      after the host access has been applied and the ISS has stepped: the pin write requests and
      the pin resolution they add up to. A re-issued same-value side-set is invisible on `pin_out`
      and visible here, which is the hole `docs/mvp-verification-report.md` `### Known holes in the
      stimulus` names.
    """

    def __init__(self, dut, engines: list) -> None:
        self.chip = {k: getattr(dut, k) for k in CHIP_DEEP_KEYS if k != "in_pipe0"}
        self.in_pipe0 = dut.in_pipe[0]
        self.lvl_n = dut.lvl_n
        self.dir_n = dut.dir_n
        self.eng: list[dict[str, Any]] = []
        for eng in engines:
            self.eng.append({
                "cfg": [getattr(eng, f"r_{name}") for name in
                        ("clkdiv", "execctrl", "shiftctrl", "pinctrl")],
                "div": [eng.u_div.cnt, eng.u_div.acc, eng.u_div.en],
                "tx_rp": eng.u_txf.rp, "tx_mem": [eng.u_txf.mem[i] for i in range(FIFO_WORDS)],
                "rx_rp": eng.u_rxf.rp, "rx_mem": [eng.u_rxf.mem[i] for i in range(FIFO_WORDS)],
                "imem": [(eng.g_imem_flop.imem if hasattr(eng, "g_imem_flop") else eng.imem)[k]
                         for k in range(gen.IMEM_SIZE)],
                "req": {k: getattr(eng, k) for k in ENGINE_REQ_KEYS},
            })

    # ---- first half: flop state -------------------------------------------------------------
    def state(self, iss) -> list[tuple[str, Optional[int], int, int]]:
        """(name, engine, rtl, iss) for every deep flop field, in `field_names()` order."""
        out: list[tuple[str, Optional[int], int, int]] = [
            ("sm_enable", None, int(self.chip["sm_enable"].value), iss.peek(gen.A_CTRL)),
            ("fdebug", None, int(self.chip["fdebug"].value), unpack_fdebug(iss.peek(gen.A_FDEBUG))),
            ("inte0", None, int(self.chip["inte0"].value), unpack_irq(iss.peek(gen.A_IRQ0_INTE))),
            ("inte1", None, int(self.chip["inte1"].value), unpack_irq(iss.peek(gen.A_IRQ1_INTE))),
            ("intf0", None, int(self.chip["intf0"].value), unpack_irq(iss.peek(gen.A_IRQ0_INTF))),
            ("intf1", None, int(self.chip["intf1"].value), unpack_irq(iss.peek(gen.A_IRQ1_INTF))),
        ]
        deep = iss.deep_state()
        out.append(("in_vec", None, int(self.chip["in_vec"].value), deep["in_vec"]))
        # sync[0] is the pad of the cycle that just ended: the newest stage of the T_IN pipeline
        out.append(("in_pipe0", None, int(self.in_pipe0.value), iss.sync[0]))
        for e, handles in enumerate(self.eng):
            eng = deep[str(e)]
            for i, key in enumerate(("clkdiv", "execctrl", "shiftctrl", "pinctrl")):
                out.append((key, e, int(handles["cfg"][i].value),
                            iss.peek(gen.A_SM_BASE + 6 * e + (0, 1, 2, 5)[i])))
            for i, key in enumerate(("div_cnt", "div_acc", "div_en")):
                out.append((key, e, int(handles["div"][i].value), eng["div"][i]))
            for kind, rp_key, mem_key, words in (("tx", "tx_rp", "tx_mem", eng["tx"]),
                                                 ("rx", "rx_rp", "rx_mem", eng["rx"])):
                rp = int(handles[rp_key].value)
                for i in range(FIFO_WORDS):
                    # the queue is a ring: the ISS lists it oldest first, the RTL stores it from
                    # rp; a word outside the queue has no counterpart and is not compared (the
                    # level itself is compared in the shallow set)
                    if i < len(words):
                        rtl_v = int(handles[mem_key][(rp + i) & (FIFO_WORDS - 1)].value)
                        iss_v = words[i]
                    else:
                        rtl_v = iss_v = 0
                    out.append((f"{kind}[{i}]", e, rtl_v, iss_v))
            for k in range(gen.IMEM_SIZE):
                out.append((f"imem[{k}]", e, int(handles["imem"][k].value), eng["imem"][k]))
        return out

    # ---- second half: this cycle's requests --------------------------------------------------
    def requests(self, iss) -> list[tuple[str, Optional[int], int, int]]:
        """(name, engine, rtl, iss) for the pin write requests and the pin resolution."""
        deep = iss.deep_state()
        out: list[tuple[str, Optional[int], int, int]] = []
        for e, handles in enumerate(self.eng):
            req = deep["req"][e]
            for channel, prefix in (("out", "os"), ("side", "ss")):
                want = req[channel]
                rtl = handles["req"]
                if want is None:
                    # The channel did not write, and `we` is the whole request: the other three are
                    # combinational echoes that are only meaningful together with it. `ss_dir` is
                    # `SIDE_PINDIR` itself (`assign ss_dir = cfg_side_pindir`) and `ss_mask` is the
                    # mask the side-set *would* have used, which is non-zero whenever
                    # SIDESET_COUNT is non-zero even if this instruction's enable bit is 0 -- so
                    # comparing them against 0 would be comparing the configuration, which the
                    # EXECCTRL compare above already does.
                    out.append((f"{prefix}_we", e, int(rtl[f"{prefix}_we"].value), 0))
                    out.append((f"{prefix}_dir", e, 0, 0))
                    out.append((f"{prefix}_mask", e, 0, 0))
                    out.append((f"{prefix}_data", e, 0, 0))
                    continue
                is_dir, mask, val = want
                out.append((f"{prefix}_we", e, int(rtl[f"{prefix}_we"].value), 1))
                out.append((f"{prefix}_dir", e, int(rtl[f"{prefix}_dir"].value), is_dir))
                out.append((f"{prefix}_mask", e, int(rtl[f"{prefix}_mask"].value), mask))
                out.append((f"{prefix}_data", e, int(rtl[f"{prefix}_data"].value), val))
        out.append(("res_level", None, int(self.lvl_n.value) & LEVEL_WRITABLE & PIN_MASK,
                    deep["res_level"]))
        out.append(("res_dir", None, int(self.dir_n.value) & DIR_WRITABLE & PIN_MASK,
                    deep["res_dir"]))
        return out


# ---------------------------------------------------------------------------------------
# the cocotb test
# ---------------------------------------------------------------------------------------

def pad_value(pin_in_ext: int, pin_out: int, pin_oe: int, pullup: bool = False,
              fault: bool = False) -> int:
    """What the pads actually carry in this cycle.

    docs/mvp-isa.md `### The 18 protocol pins`: "`uio_in` follows the pad, and an unloaded pad
    follows `uio_out`", and for open drain "the level stays 0 ... the program toggles the
    direction". So a `uio` pin the chip drives carries the chip's own driven level, whatever the
    outside world happens to offer; the rest carry `pin_in_ext`. Pins 13..17 are the chip's own
    output-only pins, which it feeds back itself and ignores here.

    `pullup` adds the board: a `uio` pin the chip *releases* reads 1, which is what an open-drain
    net with a pull-up resistor does, and is the mode the MVP's open-drain instructions need. It
    changes only what the world offers, never what the chip is asked to do, so it cannot by itself
    make the two sides disagree; `verif/fuzz/gen.py pinloop()` is the stimulus that reads the pin
    back, so that the mode is not vacuous.

    `fault` is the negative control (`FUZZ_PAD_FAULT`): a deliberately wrong pad model that
    forgets to follow the driven level. It cannot make the two sides disagree -- both get the same
    `pin_in` -- so it is caught by `pad_violations()` below instead, which is the only thing that
    can catch a wrong pad model at all.
    """
    pad = pin_in_ext & (UIO_MASK | IN_ONLY_MASK)
    for k in range(8):
        if (pin_oe >> k) & 1:
            if not fault:
                pad = (pad & ~(1 << k)) | (((pin_out >> k) & 1) << k)
        elif pullup:
            pad |= 1 << k
    return pad


def pad_rule(pin_in_ext: int, pin_out: int, pin_oe: int, pullup: bool) -> list[int]:
    """What each of the eight `uio` pads has to carry this cycle, from the rule rather than from
    `pad_value`: the chip's own level where the chip drives, the world (or 1 under `--pullup`)
    where it does not."""
    out = []
    for k in range(8):
        if (pin_oe >> k) & 1:
            out.append((pin_out >> k) & 1)
        else:
            out.append(1 if pullup else (pin_in_ext >> k) & 1)
    return out


def pad_violations(pin_in_ext: int, pin_in: int, pin_out: int, pin_oe: int,
                   pullup: bool) -> list[tuple[int, int, int]]:
    """Where the `pin_in` this run drives breaks the pad rule: `(pin, driven, required)`.

    The pad model is part of the *stimulus*, and a stimulus error is the one error the lockstep
    cannot see: the RTL and the ISS are handed the same `pin_in`, so both would happily agree on a
    pad that never happened. So the rule is checked instead of trusted, every cycle, and
    `--pad-fault` (a pad model that forgets to follow the driven level) is what proves the check
    bites.
    """
    return [(k, (pin_in >> k) & 1, want)
            for k, want in enumerate(pad_rule(pin_in_ext, pin_out, pin_oe, pullup))
            if ((pin_in >> k) & 1) != want]


@cocotb.test()
async def lockstep(dut) -> None:
    """Drive one stimulus (or a batch of them) into both the RTL and the ISS, cycle by cycle.

    `FUZZ_STIMULI` (a JSON list of stimuli, or one stimulus) runs several stimuli in one
    simulator process, which is what the minimizer needs; each starts from a fresh reset, which
    is what a reset is.
    """
    quiet = bool(os.environ.get("FUZZ_QUIET"))
    batch = load_batch()
    results = []
    for stim in batch:
        results.append(await _one(dut, stim, quiet=quiet))
    if len(results) == 1 and not os.environ.get("FUZZ_STIMULI"):
        _write_result(results[0])
    else:
        _write_result({"status": "batch", "results": results})
    bad = [r for r in results if r["status"] != "pass"]
    if bad:
        first = bad[0]
        m = first.get("mismatch", {})
        raise AssertionError(
            f"{len(bad)}/{len(results)} stimuli mismatched; first at cycle "
            f"{first['first_divergence_cycle']}, field {m.get('name')}: "
            f"rtl={m.get('rtl_hex')} iss={m.get('iss_hex')} (see {first.get('report')})")


def load_batch() -> list:
    """The stimuli this process must run: one from the environment, or the FUZZ_STIMULI list."""
    path = os.environ.get("FUZZ_STIMULI")
    if not path:
        return [load_stimulus()]
    if Path(path).exists():
        data = json.loads(Path(path).read_text())
    else:
        data = json.loads(path)                    # inline JSON, for short lists
    if isinstance(data, dict):
        data = [data]
    out = []
    for item in data:
        stim = gen.Stimulus.from_json(json.dumps(item))
        if os.environ.get("FUZZ_CYCLES"):
            stim.cycles = min(stim.cycles, env_int("FUZZ_CYCLES", stim.cycles))
        out.append(stim)
    if out:
        apply_mode(out[0])
    return out


async def _one(dut, stim: gen.Stimulus, quiet: bool = False) -> dict[str, Any]:
    """One stimulus from reset to its end. Returns the result dict; raises nothing."""
    from cocotb.clock import Clock
    from iss.pio_iss import Chip

    log = dut._log
    window = env_int("FUZZ_WINDOW", 20)
    deep = deep_enabled()
    pullup = pullup_enabled()
    pad_fault = bool(os.environ.get("FUZZ_PAD_FAULT"))
    iss = Chip()
    engines = [dut.g_eng[e].u_eng for e in range(gen.NUM_ENGINES)]
    # handles, not values: cocotb 2's `handle.value` is a snapshot, so re-read it every cycle
    handles = {k: [getattr(eng, k) for eng in engines] for k in STATE_KEYS}
    out = {k: getattr(dut, k) for k in OUTPUT_KEYS}
    rdata_handle = dut.bus_rdata
    probe = DeepProbe(dut, engines) if deep else None
    history = History(window, deep)
    coverage = Coverage()
    # FUZZ_PAD_TRACE: record, per cycle, what the pad carries and what the engines read out of it.
    # `--pullup` is not self-certifying (`pad_value()` forces the bit and the coverage counts that
    # same bit), so the evidence that the mode is non-vacuous has to be a number the pad model
    # cannot force by construction: how many cycles the *compared* `in_vec`/`in_pipe0` differ
    # between a --pullup run and a run without it. See verif/fuzz/pullup_measure.py.
    pad_trace = bool(os.environ.get("FUZZ_PAD_TRACE"))
    trace_pad: list[int] = []
    trace_in_vec: list[int] = []
    result: dict[str, Any] = {
        "status": "pass", "cycles": stim.cycles, "seed": stim.seed, "rtl": label(),
        "sim": os.environ.get("SIM", "?"), "visibility_flag": os.environ.get("FUZZ_VIS", "?"),
        "ops": len(stim.ops), "first_divergence_cycle": None,
        "deep": deep, "pullup": pullup, "pad_fault": pad_fault,
        "fields": len(history.names),
    }
    t0 = time.time()
    bus_ops = {cycle: (kind, addr, data) for cycle, kind, addr, data in stim.ops}

    def say(msg: str) -> None:
        if not quiet:
            log.info(msg)

    # ---- reset: hold rst_n low for two cycles, release it at a falling edge, so the first
    # ---- rising edge after that closes core cycle 0 (docs/mvp-isa.md `## Conventions`).
    # ---- The first FallingEdge comes before any write: a previous stimulus (batch mode) ends in
    # ---- the ReadOnly phase, and cocotb refuses writes there.
    cocotb.start_soon(Clock(dut.clk, CLOCK_NS, unit="ns").start())
    await FallingEdge(dut.clk)
    dut.rst_n.value = 0
    for name in ("bus_we", "bus_re", "bus_addr", "bus_wdata", "pin_in"):
        getattr(dut, name).value = 0
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await ReadOnly()

    mismatch: Optional[Mismatch] = None

    def check(cycle: int, field: str, engine: Optional[int], rtl: int, iss: int) -> None:
        """Record the first disagreement, and only the first (stop at the first)."""
        nonlocal mismatch
        if mismatch is None and rtl != iss:
            mismatch = Mismatch(cycle, field, engine, rtl, iss)

    for cycle in range(stim.cycles):
        iss_out = {"pin_out": iss.pin_out, "pin_oe": iss.pin_oe,
                   "irq0": iss.irq0, "irq1": iss.irq1}
        # ---- (1) state of this cycle: the DUT settled after the last edge, the ISS stepped once.
        # Every compared signal goes through `check`, so none can be forgotten.
        rtl_vals: list[int] = []
        iss_vals: list[int] = []
        for name in OUTPUT_KEYS:
            rtl_v = int(out[name].value)
            rtl_vals.append(rtl_v)
            iss_vals.append(iss_out[name])
            check(cycle, name, None, rtl_v, iss_out[name])
        for e, eng in enumerate(engines):
            state = iss.state(e)
            for k in STATE_KEYS:
                rtl_v = int(handles[k][e].value)
                rtl_vals.append(rtl_v)
                iss_vals.append(state[k])
                check(cycle, k, e, rtl_v, state[k])
        if probe is not None:                 # the five holes state() and peek() leave between them
            for name, engine, rtl_v, iss_v in probe.state(iss):
                rtl_vals.append(rtl_v)
                iss_vals.append(iss_v)
                check(cycle, name, engine, rtl_v, iss_v)
        if pad_trace:
            # the value the engines use in this cycle; T_IN stages back it is the pad of cycle-3
            trace_in_vec.append(int(probe.chip["in_vec"].value) if probe else 0)

        bus = bus_ops.get(cycle)
        if mismatch is not None:
            history.add_partial(cycle, rtl_vals, iss_vals, _bus_text(bus))
            break

        # ---- (2) drive the stimulus for this cycle
        pad_oe = iss.pin_oe
        ext = gen.pin_vector(stim.seed, cycle) & ~stim.ext_low
        pin_in = pad_value(ext, iss.pin_out, pad_oe, pullup=pullup, fault=pad_fault)
        if pad_trace:
            trace_pad.append(pin_in)
        # the one thing the RTL/ISS comparison cannot see: whether the pad itself is the pad
        violations = pad_violations(ext, pin_in, iss.pin_out, pad_oe, pullup)
        if mismatch is None and violations:
            pin, driven, want = violations[0]
            mismatch = Mismatch(cycle, f"pad[{pin}]", None, driven, want)
            log.error(f"pad rule broken at cycle {cycle}: pin {pin} driven {driven}, "
                      f"the rule requires {want} (pin_oe {pad_oe:#06x}, pin_out {iss.pin_out:#06x}, "
                      f"pull-up {pullup}, pad fault {pad_fault})")
        await FallingEdge(dut.clk)
        dut.pin_in.value = pin_in
        if bus is None:
            dut.bus_we.value = 0
            dut.bus_re.value = 0
            dut.bus_addr.value = 0
            dut.bus_wdata.value = 0
        elif bus[0] == "w":
            dut.bus_we.value = 1
            dut.bus_re.value = 0
            dut.bus_addr.value = bus[1]
            dut.bus_wdata.value = bus[2]
        else:
            dut.bus_we.value = 0
            dut.bus_re.value = 1
            dut.bus_addr.value = bus[1]
            dut.bus_wdata.value = 0

        # ---- (3) the read data is combinational in this cycle: compare it before the edge
        await ReadOnly()
        rtl_rdata = int(rdata_handle.value)
        if bus is None:
            iss_rdata = iss.step(pin_in, None)
        elif bus[0] == "r":
            iss_rdata = iss.step(pin_in, ("r", bus[1]))
        else:
            iss_rdata = iss.step(pin_in, ("w", bus[1], bus[2]))
        coverage.note(iss, cycle, bus, pin_in, pad_oe)
        rtl_vals.append(rtl_rdata)
        iss_vals.append(iss_rdata if iss_rdata is not None else 0)
        if iss_rdata is not None:
            check(cycle, READ_KEY, None, rtl_rdata, iss_rdata)
        if probe is not None:
            # the requests both sides made in this cycle, and what they add up to: read after the
            # ISS has stepped, which is the only moment the two are the same cycle's requests
            for name, engine, rtl_v, iss_v in probe.requests(iss):
                rtl_vals.append(rtl_v)
                iss_vals.append(iss_v)
                check(cycle, name, engine, rtl_v, iss_v)
        history.add(cycle, rtl_vals, iss_vals, _bus_text(bus))
        if mismatch is not None:
            break

        # ---- (4) the edge that closes this cycle
        await RisingEdge(dut.clk)
        await ReadOnly()

    runtime = time.time() - t0
    result["runtime_s"] = round(runtime, 2)
    result["cycles_run"] = cycle + 1
    result["coverage"] = coverage.as_dict(stim)
    if pad_trace:
        result["pad_trace"] = trace_pad
        result["in_vec_trace"] = trace_in_vec
    if mismatch is None:
        say(f"{label()} seed {stim.seed}: PASS, {stim.cycles} cycles, {runtime:.1f}s "
            f"({stim.cycles / max(runtime, 1e-9):.0f} cycles/s)")
        say("  covered: " + coverage.summary(stim))
        return result
    result["status"] = "mismatch"
    result["first_divergence_cycle"] = mismatch.cycle
    result["mismatch"] = mismatch.as_dict()
    result["history"] = history.as_dict()
    if os.environ.get("FUZZ_STIMULI"):
        return result                      # a batch run only wants the verdict, not the files
    report = _report(stim, mismatch, history, result)
    log.error(report)
    paths = _write_findings(stim, mismatch, result, report)
    result["repro"] = str(paths["repro"])
    result["report"] = str(paths["report"])
    log.error(f"MISMATCH: {paths['report']}")
    log.error(f"repro:    {paths['repro']}")
    return result


def _bus_text(bus: Optional[tuple]) -> str:
    if bus is None:
        return "-"
    if bus[0] == "w":
        return f"w[{bus[1]:#04x}]<={bus[2]:#010x}"
    return f"r[{bus[1]:#04x}]"


#: opcode name for the coverage counters
OP_NAMES = ("JMP", "WAIT", "IN", "OUT", "PUSH/PULL", "MOV", "IRQ", "SET")


class Coverage:
    """What the run actually exercised. Proof that a "PASS" is not a vacuous PASS.

    Read out of the ISS only, and only after the comparison: these counters never influence what
    the two sides see.
    """

    def __init__(self) -> None:
        self.sm_cycles = [0, 0]
        self.stall_cycles = [0, 0]
        self.delay_cycles = [0, 0]
        self.forced_cycles = [0, 0]
        self.executed: dict[str, int] = {}
        self.forms: set[int] = set()          # distinct instruction forms actually executed
        self.bus_reads = 0
        self.bus_writes = 0
        self.imem_writes = 0
        self.read_addrs = 0        # which of the 128 word addresses the host read/wrote
        self.write_addrs = 0
        self.pin_out_seen = 0
        self.pin_out_changes = 0
        self.pad_seen = 0
        self.pad_changes = 0
        self.uio_released = 0       # pin-cycles in which the chip released a `uio` pin
        self.uio_released_high = 0  # ... and that pin read 1: with --pullup always, without it
        self.cycles = 0             # whatever the outside world happened to offer
        self._last_out: Optional[int] = None
        self._last_pad: Optional[int] = None

    def note(self, iss, cycle: int, bus: Optional[tuple], pin_in: int, pin_oe: int) -> None:
        self.cycles += 1
        for e, eng in enumerate(iss.engines):
            if eng.force or (((iss.sm_enable >> e) & 1) and eng.en):
                self.sm_cycles[e] += 1
                if eng.force:
                    self.forced_cycles[e] += 1
                elif eng.delay > 0:
                    self.delay_cycles[e] += 1
                else:
                    word = eng.latch if eng.latch_pend else eng.ir
                    name = OP_NAMES[word >> 13]
                    if word >> 13 == 4:
                        name = "PULL" if (word >> 7) & 1 else "PUSH"
                    self.executed[name] = self.executed.get(name, 0) + 1
                    self.forms.add(gen.instr_form(word))
        for e in range(gen.NUM_ENGINES):
            if iss.engines[e].stalled:
                self.stall_cycles[e] += 1
        if bus is not None:
            if bus[0] == "r":
                self.bus_reads += 1
                self.read_addrs |= 1 << bus[1]
            else:
                self.bus_writes += 1
                self.write_addrs |= 1 << bus[1]
                if gen.A_IMEM[0] <= bus[1] < gen.A_IMEM[0] + gen.IMEM_SIZE or \
                        gen.A_IMEM[1] <= bus[1] < gen.A_IMEM[1] + gen.IMEM_SIZE:
                    self.imem_writes += 1
        out = iss.pin_out
        self.pin_out_seen |= out
        if self._last_out is not None and out != self._last_out:
            self.pin_out_changes += 1
        self._last_out = out
        self.pad_seen |= pin_in & (UIO_MASK | IN_ONLY_MASK)
        if self._last_pad is not None and pin_in != self._last_pad:
            self.pad_changes += 1
        self._last_pad = pin_in
        # the pad was built from *this* cycle's pin_oe, which is the one before the step
        for k in range(8):
            if not (pin_oe >> k) & 1:
                self.uio_released += 1
                self.uio_released_high += (pin_in >> k) & 1

    def cover_hit(self, stim: gen.Stimulus) -> dict[str, str]:
        """How many of the instruction forms the stimulus asked for the run executed."""
        out: dict[str, str] = {}
        for eng in stim.summary.get("engines", []):
            wanted = {int(f, 16) for f in eng.get("cover_forms", [])}
            if not wanted:
                continue
            out[f"engine{eng.get('engine')}"] = f"{len(wanted & self.forms)}/{len(wanted)}"
        return out

    def as_dict(self, stim: Optional[gen.Stimulus] = None) -> dict[str, Any]:
        out = {
            "cycles": self.cycles,
            "sm_cycles": self.sm_cycles,
            "stall_cycles": self.stall_cycles,
            "delay_cycles": self.delay_cycles,
            "forced_cycles": self.forced_cycles,
            "instructions_executed": dict(sorted(self.executed.items())),
            "bus_writes": self.bus_writes, "bus_reads": self.bus_reads,
            "imem_writes": self.imem_writes,
            "read_addrs": f"{self.read_addrs:#034x}", "write_addrs": f"{self.write_addrs:#034x}",
            "pin_out_changes": self.pin_out_changes, "pin_out_pattern": f"{self.pin_out_seen:#07x}",
            "pad_changes": self.pad_changes, "pad_pattern": f"{self.pad_seen:#06x}",
            "uio_released_pin_cycles": self.uio_released,
            "uio_released_high_pin_cycles": self.uio_released_high,
            "distinct_instruction_forms": len(self.forms),
        }
        if stim is not None and self.cover_hit(stim):
            out["cover_forms_executed"] = self.cover_hit(stim)
        return out

    def summary(self, stim: Optional[gen.Stimulus] = None) -> str:
        kinds = len(self.executed)
        cover = self.cover_hit(stim) if stim is not None else {}
        return (f"{self.cycles} cycles, SM cycles e0/e1 {self.sm_cycles[0]}/{self.sm_cycles[1]}, "
                f"{kinds} instruction kinds executed, stalls {self.stall_cycles[0]}/"
                f"{self.stall_cycles[1]}, forced {self.forced_cycles[0]}/"
                f"{self.forced_cycles[1]}, host w/r {self.bus_writes}/{self.bus_reads} "
                f"({self.read_addrs.bit_count()}/{self.write_addrs.bit_count()} distinct "
                f"addresses), pin_out changes {self.pin_out_changes}, pad pattern "
                f"{self.pad_seen:#06x}, {len(self.forms)} instruction forms"
                f", released uio pin-cycles {self.uio_released_high}/{self.uio_released} high"
                + (", cover forms " + " ".join(f"{k[-1]}:{v}" for k, v in sorted(cover.items()))
                   if cover else ""))


# ---------------------------------------------------------------------------------------
# findings
# ---------------------------------------------------------------------------------------

def git_stamp() -> str:
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty"], cwd=ROOT,
                              capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception:                                     # pragma: no cover
        return "?"


def _report(stim: gen.Stimulus, mismatch: Mismatch, history: History,
            result: dict[str, Any]) -> str:
    """The human-readable mismatch report: cycle, field, both values, and the context."""
    names = history.field_names()
    rows = list(zip(history.cycles, history.rtl, history.iss, history.bus))
    width = max(len(n) for n in names)
    interesting = [i for i in range(len(names))
                   if any(rtl[i] != iss[i] for _, rtl, iss, _ in rows if i < len(rtl) and i < len(iss))]
    lines = [
        "=" * 78,
        f"lockstep mismatch: cycle {mismatch.cycle}, field {mismatch.name}",
        f"  RTL ({label()}) = {mismatch.rtl} ({mismatch.rtl:#010x})",
        f"  ISS            = {mismatch.iss} ({mismatch.iss:#010x})",
        f"seed {stim.seed}, {stim.cycles} cycles, {len(stim.ops)} bus ops, "
        f"simulator {os.environ.get('SIM', '?')}",
        f"compared {len(names)} fields (deep {deep_enabled()}, pull-up {pullup_enabled()}, "
        f"pad fault {bool(os.environ.get('FUZZ_PAD_FAULT'))})",
        f"git {git_stamp()}, {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        f"last {history.window} cycles, RTL/ISS; every field that differs in the window:",
        "  " + f"{'cycle':>6}  {'bus':<15}" + "".join(f"{names[i]:>{max(width, 12)}}" for i in interesting),
    ]
    for cycle, rtl, iss, bus in rows:
        cells = []
        for i in interesting:
            cells.append(f"{rtl[i]:#x}/{iss[i]:#x}" if rtl[i] != iss[i] else f"{rtl[i]:#x}")
        lines.append(f"{cycle:6d}  {bus:<15}" + "".join(f"{c:>{max(width, 12)}}" for c in cells))
    lines += [
        "",
        f"every compared field in the divergence cycle {mismatch.cycle} (RTL vs ISS):",
        "  " + f"{'field':>{width}}  {'RTL':>12}  {'ISS':>12}",
    ]
    for cycle, rtl, iss, _ in rows:
        if cycle != mismatch.cycle:
            continue
        for i, name in enumerate(names):
            if i >= len(rtl) or i >= len(iss):
                continue
            flag = "  <-- differs" if rtl[i] != iss[i] else ""
            lines.append(f"  {name:>{width}}  {rtl[i]:>12}  {iss[i]:>12}{flag}")
    lines += [
        "",
        "host ops in the window (cycle, op):",
    ]
    for cycle, kind, addr, data in stim.ops:
        if history.cycles and history.cycles[0] <= cycle <= mismatch.cycle:
            lines.append(f"  {cycle:6d}  {kind} {addr:#04x} {data:#010x}")
    lines += [
        "",
        "engine setup (from the stimulus):",
    ]
    for eng in stim.summary.get("engines", []):
        lines.append(f"  engine {eng.get('engine')}: wrap {eng.get('wrap')} "
                     f"side {eng.get('side')} config {eng.get('config')}")
        for image in eng.get("images", [])[:1]:
            lines.append(f"    image 0: {' '.join(image)}")
        if "program" in eng:
            lines.append(f"    program: {' '.join(eng['program'])}")
    lines += [
        "",
        "repro: the JSON beside this file (seed fixes the pins, `ops` is the bus traffic)",
        "  shrink it with: make fuzz-repro REPRO=<file>",
    ]
    return "\n".join(lines)


def _write_findings(stim: gen.Stimulus, mismatch: Mismatch, result: dict[str, Any],
                    report: str) -> dict[str, Path]:
    """Write the repro stimulus and the report into verif/fuzz/repros/."""
    repro_dir = Path(os.environ.get("FUZZ_REPRO_DIR") or (HERE / "repros"))
    repro_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{label()}-seed{stim.seed}-c{mismatch.cycle:06d}-{mismatch.name.replace('.', '_')}"
    if stim.minimized:            # a minimized replay must not overwrite the repro it came from
        stem += ".min"
    repro = repro_dir / f"{stem}.json"
    txt = repro_dir / f"{stem}.txt"
    stim.build = label()          # so that replaying it picks the same RTL build
    stim.mode = {"deep": deep_enabled(), "pullup": pullup_enabled()}
    stim.save(repro)
    txt.write_text(report + "\n")
    return {"repro": repro, "report": txt}


def _write_result(result: dict[str, Any]) -> None:
    path = os.environ.get("FUZZ_RESULT")
    if not path:
        return
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(result, fh, indent=1)


# ---------------------------------------------------------------------------------------
# the command line: build with verif/fuzz/Makefile, run once, decide the exit status
# ---------------------------------------------------------------------------------------

def build_dir(sim: str = "verilator", planted: bool = False, plant: int = 1) -> Path:
    return HERE / "sim_build" / (sim + (f"-planted{plant}" if planted else ""))


def build(sim: str = "verilator", planted: bool = False, plant: int = 1, jobs: str = "-j2",
          force: bool = False) -> None:
    """Throw away the simulator build (used before a re-run, or by `--force`)."""
    if force:
        target = build_dir(sim, planted, plant)
        if target.exists():
            shutil.rmtree(target)


def run(seed: Optional[int] = None, cycles: Optional[int] = None, sim: str = "verilator",
        planted: bool = False, repro: Optional[str] = None, trivial: bool = False,
        window: int = 20, quiet: bool = False, jobs: str = "-j2", force: bool = False,
        modules: Optional[str] = None, plant: int = 1,
        repro_dir: Optional[str] = None, stimuli: Optional[Path] = None,
        deep: Optional[bool] = None, pullup: bool = False, conflict: int = 0,
        pinloop: bool = False, fifofill: bool = False, pad_fault: bool = False,
        extra_env: Optional[dict[str, str]] = None) -> dict[str, Any]:
    """Run one lockstep run and return its result dict.

    Builds through `verif/fuzz/Makefile` (only if the sources changed), runs the cocotb test
    with the stimulus in the environment, and reads back the machine-readable result the test
    wrote. The simulator's own output goes to `verif/fuzz/sim_build/<sim>[.log]`, which is where
    the report is printed.

    `deep=None` means "whatever `DEEP_DEFAULT` and the stimulus say"; `deep=False` is the
    shallow comparison, which the deep negative controls run against.
    """
    if force:
        build(sim=sim, planted=planted, plant=plant, jobs=jobs, force=True)
    if repro and not planted:
        # a repro records the RTL build it was made against, so replaying needs no extra flags
        recorded = json.loads(Path(repro).read_text()).get("build", "rtl")
        if recorded.startswith("planted"):
            planted = True
            plant = int(recorded.removeprefix("planted") or 1)
    with tempfile.TemporaryDirectory(prefix="fuzzlock-") as tmp:
        result_path = Path(tmp) / "result.json"
        env = dict(os.environ)
        env.update({
            "FUZZ_SEED": str(seed if seed is not None else 1),
            "FUZZ_CYCLES": str(cycles if cycles is not None else 1000),
            "FUZZ_WINDOW": str(window),
            "FUZZ_RESULT": str(result_path),
            "PYTHONPATH": os.pathsep.join(p for p in (str(ROOT), str(HERE),
                                                      env.get("PYTHONPATH")) if p),
        })
        for gone in ("FUZZ_REPRO", "FUZZ_TRIVIAL", "FUZZ_QUIET", "FUZZ_PLANTED", "FUZZ_VIS",
                     "FUZZ_STIMULI", "FUZZ_DEEP", "FUZZ_PULLUP", "FUZZ_CONFLICT",
                     "FUZZ_PINLOOP", "FUZZ_FIFOFILL", "FUZZ_PAD_FAULT", "FUZZ_DEEP_PLANT",
                     "FUZZ_PAD_TRACE"):
            env.pop(gone, None)
        if stimuli is not None:
            env["FUZZ_STIMULI"] = str(stimuli)
        if repro:
            env["FUZZ_REPRO"] = str(Path(repro).resolve())
        if trivial:
            env["FUZZ_TRIVIAL"] = "1"
        if pinloop:
            env["FUZZ_PINLOOP"] = "1"
        if fifofill:
            env["FUZZ_FIFOFILL"] = "1"
        if quiet:
            env["FUZZ_QUIET"] = "1"
        if deep is not None:
            env["FUZZ_DEEP"] = "1" if deep else "0"
        if pullup:
            env["FUZZ_PULLUP"] = "1"
        if conflict:
            env["FUZZ_CONFLICT"] = str(conflict)
        if pad_fault:
            env["FUZZ_PAD_FAULT"] = "1"
        if extra_env:
            env.update({k: str(v) for k, v in extra_env.items()})
        if planted:
            env["FUZZ_PLANTED"] = str(plant)
        if repro_dir:
            env["FUZZ_REPRO_DIR"] = str(Path(repro_dir).resolve())
        tag = sim + (f"-planted{plant}" if planted else "")
        log_path = HERE / "sim_build" / f"{tag}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["make", jobs, "-C", str(HERE), f"SIM={sim}", f"PLANTED={1 if planted else 0}",
               f"PLANT={plant}", "sim"]
        if modules:
            cmd.append(f"COCOTB_TEST_MODULES={modules}")
        with open(log_path, "w") as log:
            proc = subprocess.run(cmd, env=env, cwd=HERE, stdout=log, stderr=subprocess.STDOUT)
        if not result_path.exists():
            print(log_path.read_text(errors="replace"), file=sys.stderr)
            raise RuntimeError(f"the simulator produced no result file (see {log_path})")
        result = json.loads(result_path.read_text())
    result["simulator_log"] = str(log_path)
    result["simulator_exit"] = proc.returncode
    return result


def _main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="run the RTL/ISS lockstep fuzz")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--cycles", type=int, default=None)
    ap.add_argument("--sim", default="verilator", choices=["verilator", "icarus"])
    ap.add_argument("--planted", action="store_true", help="build verif/fuzz/planted with a bug")
    ap.add_argument("--plant", type=int, default=1, help="which FUZZ_PLANT_RTL plant to build")
    ap.add_argument("--repro", default=None, help="replay a stimulus JSON instead of generating")
    ap.add_argument("--trivial", action="store_true", help="run the selftest stimulus")
    ap.add_argument("--window", type=int, default=20)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--jobs", default="-j2")
    ap.add_argument("--force", action="store_true", help="rebuild from scratch")
    ap.add_argument("--repro-dir", default=None, help="where repros go")
    ap.add_argument("--deep", dest="deep", action="store_true", default=None,
                    help="compare the deep fields too (the default)")
    ap.add_argument("--no-deep", dest="deep", action="store_false",
                    help="only the 37 signals of the original lockstep")
    ap.add_argument("--pullup", action="store_true",
                    help="a uio pin the chip releases reads 1 (an open-drain net with a pull-up)")
    ap.add_argument("--conflict", type=int, default=0, metavar="N",
                    help="append a guaranteed two-engine pin conflict of N cycles")
    ap.add_argument("--pinloop", action="store_true", help="run the pad-model stimulus")
    ap.add_argument("--fifofill", action="store_true", help="run the FIFO-storage stimulus")
    ap.add_argument("--pad-fault", action="store_true",
                    help="negative control: drive a deliberately wrong pad model")
    ap.add_argument("--pad-trace", action="store_true",
                    help="record the per-cycle pad and in_vec in the result, so the pull-up's "
                         "non-vacuity can be measured (verif/fuzz/pullup_measure.py)")
    args = ap.parse_args(argv)
    result = run(seed=args.seed, cycles=args.cycles, sim=args.sim, planted=args.planted,
                 repro=args.repro, trivial=args.trivial, window=args.window, quiet=args.quiet,
                 jobs=args.jobs, force=args.force, repro_dir=args.repro_dir, plant=args.plant,
                 deep=args.deep, pullup=args.pullup, conflict=args.conflict,
                 pinloop=args.pinloop, fifofill=args.fifofill, pad_fault=args.pad_fault,
                 extra_env={"FUZZ_PAD_TRACE": "1"} if args.pad_trace else None)
    if args.quiet and result.get("report"):
        print(Path(result["report"]).read_text(errors="replace"))
    if result["status"] == "pass":
        print(f"fuzz: seed {result['seed']} {result['cycles']} cycles match "
              f"({result['fields']} fields, deep {result.get('deep')}, "
              f"{result['runtime_s']}s, simulator log {result['simulator_log']})")
        return 0
    m = result["mismatch"]
    print(f"fuzz: MISMATCH seed {result['seed']} cycle {m['cycle']} field {m['name']}: "
          f"rtl={m['rtl_hex']} iss={m['iss_hex']}")
    print(f"  report {result['report']}\n  repro  {result['repro']}")
    return 1


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))