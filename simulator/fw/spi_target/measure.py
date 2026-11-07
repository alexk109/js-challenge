"""Measure the MVP SPI *target* firmware and write the sweep to a log.

    .venv/bin/python -m fw.spi_target.measure                       # print the report
    .venv/bin/python -m fw.spi_target.measure --log sweep.log

What it measures, and why each number is here:

* **the ceiling.**  The fastest `half_period` at which a 0x9F and a 0x03 transaction are both
  served with the right bytes off the wire, no `SpiMonitor` violation and no `FDEBUG` flag --
  the headline number, and `MAX_HALF_CLEAN` in `test_spi_target.py`.
* **where the ceiling comes from**, as three separate rows rather than one story:
  * *instruction term*: the same program with one `nop` between the launch wait and the
    `out pins, 1`.  One more SM cycle on the launch path must cost exactly one half period.
  * *divider term*: the ceiling at CLKDIV 1.0, 2.0, 3.0, 4.0.  At 1.0 the answer is
    `T_IN + 1 + T_OUT`; from 2.0 on the loop's own length binds too, so this is a measured
    table.
  * *input synchroniser term*: read off the chip by the latency decomposition
    (`test_the_latency_decomposition_adds_up_to_the_ceiling`), not by this script.
* **instruction count**: 9 words of the 32 an engine has.

Every number comes from running the same bench the tests run
(`fw/spi_target/test_spi_target.py`, `sweep_half_period`), so the log and the assertions cannot
drift apart.  Nothing here writes to `metrics/fw.csv`: that ledger is shared, and this script
only writes the log named by `--log`.
"""
from __future__ import annotations

import argparse
import datetime
import pathlib
import subprocess
import sys

from fw.spi_target import assemble_program
from fw.spi_target.test_spi_target import (
    F_CLK_HZ,
    HALVES_TOO_FAST,
    LOOP_CYCLES,
    MAX_HALF_CLEAN,
    NS_PER_TICK,
    why_too_fast,
    late_out_program,
    served,
    sweep_half_period,
)

DIVIDERS = ((1, 0), (2, 0), (3, 0), (4, 0))
#: `half_period` values the ceiling search walks for each divider.  Wide enough to bracket the
#: slowest divider's ceiling (15) on both sides.
SEARCH_HALVES = tuple(range(2, 18))


def git_describe() -> str:
    try:
        out = subprocess.run(["git", "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def rate_table(clkdiv: tuple[int, int], program=None) -> list[dict]:
    """Every `half_period` in the search, at one divider, with what the wire carried."""
    return [sweep_half_period(half, clkdiv=clkdiv, program=program)
            for half in SEARCH_HALVES]


def ceiling(clkdiv: tuple[int, int], program=None) -> tuple[int | None, list[dict]]:
    """The fastest `half_period` that serves everything, and the whole table behind it."""
    table = rate_table(clkdiv, program)
    clean = [r["half"] for r in table if served(r)]
    return (min(clean) if clean else None), table


def _row(r: dict) -> str:
    ok = "served" if served(r) else "FAILED"
    why = ""
    if not served(r):
        if r["violations"]:
            why = f"  monitor: {r['violations'][0]}"
        elif r["fdebug"]:
            why = f"  FDEBUG {r['fdebug']:#04x}"
        elif not r["jedec_ok"] or not r["read_ok"]:
            why = (f"  miso 0x9F -> {[hex(b) for b in r['miso']]}"
                   if not r["jedec_ok"] else
                   f"  miso 0x03 -> {[hex(b) for b in r['read_miso']]}")
        else:
            why = "  bus contention"
    return (f"  half_period {r['half']:2d}  {r['period_ticks']:3d} core cycles/bit  "
            f"{r['rate_hz'] / 1e6:7.3f} MHz  {ok}{why}")


def report() -> str:
    out: list[str] = []
    program = assemble_program()
    add = out.append
    add("SPI target (device mode) firmware measurement -- fw/spi_target")
    add(f"date            {datetime.date.today().isoformat()}")
    add(f"commit          {git_describe()}")
    add(f"target          FW_TARGET={__import__('os').environ.get('FW_TARGET', 'iss')}")
    add(f"clock           {NS_PER_TICK:g} ns per core cycle ({F_CLK_HZ / 1e6:.0f} MHz), "
        f"50 MHz core")
    add(f"program         {program.name}, {len(program.instrs)} of 32 IMEM words, "
        f"{LOOP_CYCLES} SM cycles per bit")
    add("judge           verif.vip.spi.SpiController drives, SpiMonitor decodes; a rate counts")
    add("                as served only when the decoded MISO bytes are right AND there is no")
    add("                monitor violation AND FDEBUG is clear AND the bus has no contention")
    add("")

    # ---- the ceiling at the shipped divider -------------------------------------------------
    best, table = ceiling((1, 0))
    add(f"1. The ceiling at CLKDIV 1.0   (half_period, i.e. ticks per SCK half period)")
    add("   The whole sweep, fastest first is at the bottom:")
    for r in sorted(table, key=lambda r: r["half"]):
        add(_row(r))
    add("")
    if best is None:
        add("   NOTHING SERVED at CLKDIV 1.0")
    else:
        add(f"   fastest served: half_period {best} = {2 * best} core cycles per bit = "
            f"{F_CLK_HZ / (2 * best) / 1e6:.2f} MHz")
        add(f"   arithmetic:      T_IN 3 + the `out` instruction 1 + T_OUT 1 = {3 + 1 + 1} "
            f"= half_period {MAX_HALF_CLEAN}")
        add(f"   matches the recorded MAX_HALF_CLEAN {MAX_HALF_CLEAN}: "
            f"{best == MAX_HALF_CLEAN}")
        for half in HALVES_TOO_FAST:
            add(f"   half_period {half} ({F_CLK_HZ / (2 * half) / 1e6:.2f} MHz): {why_too_fast(half)}")
    add("")

    # ---- the instruction term ----------------------------------------------------------------
    late_best, _ = ceiling((1, 0), program=late_out_program())
    add("2. The instruction term: the same program plus one `nop` before the `out pins, 1`")
    add(f"   fastest served: half_period {late_best} "
        f"({F_CLK_HZ / (2 * late_best) / 1e6:.2f} MHz)"
        if late_best else "   nothing served")
    if best is not None and late_best is not None:
        add(f"   one more SM cycle on the launch path costs {late_best - best} half period(s): "
            f"{best} -> {late_best}")
    add("   (this is the only term the program can pay; T_IN and T_OUT are hardware)")
    add("")

    # ---- the divider term ---------------------------------------------------------------------
    add("3. The divider term: the ceiling at four integer dividers")
    add("   INT   fastest served   core cycles/bit   MHz   T_IN + INT + T_OUT")
    ceilings = {}
    for clkdiv in DIVIDERS:
        got, _ = ceiling(clkdiv)
        ceilings[clkdiv[0]] = got
        if got is None:
            add(f"  {clkdiv[0]:3d}   nothing served")
            continue
        add(f"  {clkdiv[0]:3d}   {got:14d}   {2 * got:15d}   "
            f"{F_CLK_HZ / (2 * got) / 1e6:5.2f}   {3 + clkdiv[0] + 1:17d}")
    add("   At INT 1 the deadline (T_IN + INT + T_OUT = 5) is the whole story and the two")
    add("   agree.  From INT 2 on the loop's own length binds as well -- a byte boundary costs")
    add("   eight SM cycles (five in the loop, three at the boundary) and a bit period is")
    add("   2 * half_period / INT of them -- so the two effects are not a sum and the table is")
    add("   what it is.  `test_the_measured_ceiling_at_each_divider` records it as")
    add("   DIVIDER_CEILINGS and fails if it moves.")
    add("")

    # ---- the summary --------------------------------------------------------------------------
    add("4. Summary")
    add(f"   words used      {len(program.instrs)} of 32")
    add(f"   fastest served  half_period {best} = {2 * best} core cycles per bit = "
        f"{F_CLK_HZ / (2 * best) / 1e6:.2f} MHz at 50 MHz"
        if best else "   fastest served  nothing")
    add(f"   decomposition   T_IN 3 (input synchroniser, 60%) + 1 instruction (20%) + "
        f"T_OUT 1 (20%)")
    add("   the limit is the input synchroniser: three of the five core cycles are T_IN, which")
    add("   no program can remove.  That 100 ns of edge-to-answer latency is the sampled-lane")
    add("   reflex thesis C1 is about (research/ideation/round2/research/C1.md, F3).")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--log", type=pathlib.Path, default=None,
                        help="write the report here as well as to stdout")
    args = parser.parse_args(argv)
    text = report()
    sys.stdout.write(text)
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        args.log.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())