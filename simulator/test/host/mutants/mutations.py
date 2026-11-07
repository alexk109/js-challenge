# SPDX-License-Identifier: Apache-2.0
#
# The 7 mutations of docs/mvp-verification-report.md `### 7. The blind host-port bench`,
# expressed as edits against the *real* `src/host_spi.sv` instead of against
# `test/host/host_spi_stub.sv`, which is what the report's table was measured on.
#
# Each mutation is a list of (old, new) text edits applied to the pristine RTL.  Two rules
# make a mutation trustworthy, and `apply` enforces both:
#   * every `old` must occur **exactly once** in the source, otherwise the edit is
#     ambiguous and the build fails instead of silently mutating the wrong place;
#   * the result must differ from the source (a mutation that changes nothing is a
#     negative control, `NEGATIVE_CONTROL`, and only `selftest.py` may produce one).
# Nothing here is read by the bench: the mutants are compiled as the DUT, so a wrong
# mutation is a hole in *this* file, not in the bench.
#
# The mutant sources themselves are committed next to this file, one directory per
# mutation, because they are the negative controls of the whole exercise: they are the
# evidence that the 16 tests bite.  `generate.py --check` re-derives them from
# `src/host_spi.sv` and fails if the committed copies have drifted.
#
# Every mutation keeps the module name and port list, so `test/host/tb.sv` and the cocotb
# bench compile it unchanged; the only difference is the behaviour under test.

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
#: The pristine source the mutants are copies of.
SRC = HERE.parents[2] / "src" / "host_spi.sv"
#: Where the copies go: <HERE>/<id>/host_spi.sv
DUT_NAME = "host_spi.sv"


@dataclass(frozen=True)
class Mutation:
    """One behavioural fault, as edits on the RTL text."""

    id: str
    title: str
    #: Why this is the same fault the report's row 1..7 measured, on the other file.
    note: str = ""
    edits: Sequence[tuple[str, str]] = field(default_factory=tuple)

    def path(self) -> Path:
        return HERE / self.id / DUT_NAME


def apply(source: str, mut: Mutation, *, allow_noop: bool = False) -> str:
    """`source` with `mut` applied.  Raises if an anchor is missing or ambiguous."""
    out = source
    for i, (old, new) in enumerate(mut.edits):
        n = out.count(old)
        if n != 1:
            raise ValueError(
                f"{mut.id}: edit {i} matches {n} times, expected exactly 1:\n"
                f"  --- {old!r}\n  +++ {new!r}")
        out = out.replace(old, new, 1)
    if out == source and not allow_noop:
        raise ValueError(f"{mut.id}: the edits leave the source unchanged")
    return out


# ---------------------------------------------------------------------------- the 7

#: Row 1: the bus access lands one cycle late (h = n + 2 instead of n + 1).
#: Faithful rather than minimal: the pulse itself gets one more pipeline stage, and the
#: two things that happen "at the access of cycle h" (the burst address step and the load
#: of the read word into MISO) key off the moved pulse, so the whole cycle h is late and
#: not just the output.
LATE_ACCESS = Mutation(
    id="M1_access_one_cycle_late",
    title="bus access one cycle late (h = n + 2 instead of n + 1)",
    note="report row 1 (7/16 failed on the stub): one more pipeline stage on the access pulse",
    edits=(
        # the pipeline register the pulse has to pass through, and its reset
        ("  logic        we_q, re_q;   // the access pulse, valid in h\n",
         "  logic        we_q, re_q;   // the access pulse, valid in h\n"
         "  logic        we_d, re_d;   // MUTATION M1: an extra stage in front of it\n"),
        ("      wdata_q    <= 32'd0;\n    end else if (!sel) begin\n",
         "      wdata_q    <= 32'd0;\n"
         "      we_d       <= 1'b0;    // MUTATION M1\n"
         "      re_d       <= 1'b0;    // MUTATION M1\n"
         "    end else if (!sel) begin\n"),
        ("      wdata_q    <= 32'd0;\n    end else begin\n",
         "      wdata_q    <= 32'd0;\n"
         "      we_d       <= 1'b0;    // MUTATION M1\n"
         "      re_d       <= 1'b0;    // MUTATION M1\n"
         "    end else begin\n"),
        ("      we_q <= 1'b0;\n      re_q <= 1'b0;\n",
         "      we_q <= we_d;          // MUTATION M1: the pulse is one cycle older\n"
         "      re_q <= re_d;          // MUTATION M1\n"
         "      we_d <= 1'b0;\n      re_d <= 1'b0;\n"),
        ("            wdata_q   <= wdata_next;\n            we_q      <= 1'b1;\n",
         "            wdata_q   <= wdata_next;\n"
         "            we_d      <= 1'b1;   // MUTATION M1\n"),
        ("          if (bit_cnt_q == 5'd0) re_q <= 1'b1;\n",
         "          if (bit_cnt_q == 5'd0) re_d <= 1'b1;   // MUTATION M1\n"),
    ),
)

#: Row 2: the read fires on the last turnaround bit instead of the first one.
LATE_TA = Mutation(
    id="M2_read_on_last_ta_bit",
    title="read fires on the last turnaround bit instead of the first",
    note="report row 2 (10/16 failed on the stub): bit_cnt_q == 5'd0 -> 5'd7",
    edits=(
        ("          if (bit_cnt_q == 5'd0) re_q <= 1'b1;\n",
         "          if (bit_cnt_q == 5'd7) re_q <= 1'b1;  // MUTATION M2\n"),
    ),
)

#: Row 3: the burst address always increments, so the FIFO data registers walk away.
NO_FIFO_HOLD = Mutation(
    id="M3_burst_address_always_increments",
    title="burst address always increments, ignoring the FIFO data registers",
    note="report row 3 (5/16 failed on the stub): the 0x04-0x0b exception is gone",
    edits=(
        ("    if ((a >= FIFO_LO) && (a <= FIFO_HI)) addr_step = a;\n",
         "    if (1'b0) addr_step = a;                     // MUTATION M3\n"),
    ),
)

#: Row 4: a word the host abandoned half way still reaches the bus when CSn rises.
PARTIAL_WRITE_ON_ABORT = Mutation(
    id="M4_partial_word_survives_csn",
    title="a partial word is not discarded when CSn rises",
    note="report row 4 (7/16 failed on the stub): the idle branch fires the write",
    edits=(
        ("      wdata_q    <= 32'd0;\n    end else begin\n",
         "      wdata_q    <= 32'd0;\n"
         "      // MUTATION M4: the word the host walked away from still goes out\n"
         "      if ((phase_q == PH_WR) && (bit_cnt_q != 5'd0)) begin\n"
         "        wdata_q <= {wdata_sr_q, mosi_s2};\n"
         "        we_q    <= 1'b1;\n"
         "      end\n"
         "    end else begin\n"),
    ),
)

#: Row 5: the MISO shift register also moves during the turnaround byte.
MISO_SHIFTS_IN_TA = Mutation(
    id="M5_miso_shifts_during_ta",
    title="miso_sr also shifts during the turnaround byte",
    note="report row 5 (10/16 failed on the stub): the shift is not RD-phase only",
    edits=(
        ("        PH_TA: if (rise) begin\n          if (bit_cnt_q == 5'd0) re_q <= 1'b1;\n",
         "        PH_TA: if (rise) begin\n"
         "          miso_sr_q <= {miso_sr_q[30:0], 1'b0};  // MUTATION M5\n"
         "          if (bit_cnt_q == 5'd0) re_q <= 1'b1;\n"),
    ),
)

#: Row 6: MISO is not cleared while the port is idle.
NO_IDLE_MISO_CLEAR = Mutation(
    id="M6_miso_not_cleared_when_idle",
    title="MISO is not cleared while the port is idle",
    note="report row 6 (1/16 failed on the stub): the reset of miso_sr_q in the idle branch",
    edits=(
        ("      wdata_sr_q <= 31'd0;\n      miso_sr_q  <= 32'd0;\n      we_q       <= 1'b0;\n"
         "      re_q       <= 1'b0;\n      wdata_q    <= 32'd0;\n    end else begin\n",
         "      wdata_sr_q <= 31'd0;\n"
         "      // MUTATION M6: miso_sr_q is not reset here\n"
         "      we_q       <= 1'b0;\n"
         "      re_q       <= 1'b0;\n"
         "      wdata_q    <= 32'd0;\n    end else begin\n"),
    ),
)

#: Row 7: the command byte is decoded LSB first, so RW and ADDR are swapped.
CMD_LSB_FIRST = Mutation(
    id="M7_command_byte_lsb_first",
    title="command byte decoded LSB first (RW and ADDR swapped)",
    note="report row 7 (13/16 failed on the stub): ADDR = cmd[7:1], RW = cmd[0]",
    edits=(
        ("            addr_q    <= cmd_byte[ADDR_W-1:0];\n"
         "            if (cmd_byte[CMD_BITS-1]) phase_q <= PH_TA;\n",
         "            // MUTATION M7: decoded LSB first\n"
         "            addr_q    <= cmd_byte[7:1];\n"
         "            if (cmd_byte[0]) phase_q <= PH_TA;\n"),
    ),
)

MUTATIONS: tuple[Mutation, ...] = (
    LATE_ACCESS,
    LATE_TA,
    NO_FIFO_HOLD,
    PARTIAL_WRITE_ON_ABORT,
    MISO_SHIFTS_IN_TA,
    NO_IDLE_MISO_CLEAR,
    CMD_LSB_FIRST,
)

#: The negative control for `run.py` (see selftest.py): a "mutation" whose edits leave the
#: source byte-identical, so the mutant *is* the pristine RTL and must be reported as a
#: survivor.  It is never part of a PASS run and generate.py refuses to write it.
NEGATIVE_CONTROL = Mutation(
    id="X0_noop_negative_control",
    title="no-op: identical to src/host_spi.sv, so the bench must report a survivor",
    note="negative control for the mutation driver, never part of an accepted run",
    edits=(),
)


def source_text() -> str:
    """The pristine `src/host_spi.sv`."""
    return SRC.read_text()


def diffstat(source: str, mutated: str) -> tuple[int, int]:
    """(lines removed, lines added) between the pristine source and a mutant."""
    import difflib
    removed = added = 0
    for line in difflib.ndiff(source.splitlines(), mutated.splitlines()):
        if line.startswith("- "):
            removed += 1
        elif line.startswith("+ "):
            added += 1
    return removed, added


if __name__ == "__main__":       # a quick self-check, no simulator needed
    src = source_text()
    for mut in MUTATIONS:
        removed, added = diffstat(src, apply(src, mut))
        print(f"{mut.id:38s} edits={len(mut.edits)} -{removed} +{added} -> {mut.path()}")