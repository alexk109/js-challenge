#!/usr/bin/env python
# SPDX-License-Identifier: Apache-2.0
#
# The negative controls for the mutation machinery of this directory.
#
# `run.py` claims "7/7 mutations killed".  That claim is only worth reading if the driver
# itself can fail, so this self-test shows two ways it can, plus one way `mutations.py` can:
#
#   1. NO-OP MUTANT.  A "mutation" whose edits leave src/host_spi.sv byte-identical is
#      compiled as the DUT.  The bench must report 16/16 pass, i.e. the mutant SURVIVES, and
#      run.py must report that as the expectation it was given.  A driver that printed KILLED
#      for everything, or that skipped the run, would be caught here.
#   2. A MUTATION THAT IS NOT THERE.  An edit whose anchor does not occur exactly once must
#      make mutations.apply raise, not silently patch the wrong line or leave the source
#      alone.  This is the "deliberately wrong input that the check rejects" control, and it
#      is the reason apply() counts its anchors.
#   3. A MUTANT THAT DOES NOT COMPILE.  iverilog rejects it, make stops, and run.py must
#      report an ERROR rather than counting a kill: the bench never ran, so nothing was
#      shown about it.
#   4. THE COMMITTED COPIES.  generate.py --check re-derives all 7 from src/host_spi.sv, so a
#      hand-edited or stale mutant cannot masquerade as a faithful copy of the RTL.
#
# Every control runs its own simulation, so:
#   .venv/bin/python test/host/mutants/selftest.py
# Exit 0 iff all controls behaved as they must.

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))

import generate  # noqa: E402
import mutations  # noqa: E402
from mutations import NEGATIVE_CONTROL, apply, source_text  # noqa: E402

PY = ROOT / ".venv" / "bin" / "python"
failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"{'ok  ' if ok else 'FAIL'} {name}" + (f" -- {detail}" if detail else ""))
    if not ok:
        failures.append(name)


def control_noop_mutant_must_survive() -> None:
    """A mutant identical to the RTL must pass every test, and the driver must say so."""
    src = source_text()
    with tempfile.TemporaryDirectory(prefix="hostmut-") as tmp:
        root = Path(tmp)
        path = generate.build(NEGATIVE_CONTROL, src, root, allow_noop=True)
        check("no-op mutant is byte-identical to src/host_spi.sv",
              path.read_text() == src, f"{path}")
        proc = subprocess.run(
            [str(PY), str(HERE / "run.py"), "--dir", str(root), "--only",
             NEGATIVE_CONTROL.id, "--negative-control"],
            cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = proc.stdout
        print(out)
        check("run.py exits 0 when the no-op mutant survives as expected",
              proc.returncode == 0, f"exit {proc.returncode}")
        check("run.py reports the no-op mutant as SURVIVED",
              "SURVIVED" in out and "RESULT: PASS" in out)
        check("run.py did not report it as killed",
              ", 16 pass, 0 fail," in out,
              "all 16 tests must pass against a copy of the clean RTL")


def control_missing_anchor_is_rejected() -> None:
    """An edit that cannot be placed must raise, not pass quietly."""
    src = source_text()
    bad_anchor = mutations.Mutation(
        id="BAD_anchor", title="anchor that is not in the RTL", edits=(("module host_spi (" + "\n" * 99 + "x", "y"),))
    try:
        apply(src, bad_anchor)
        check("an unplaceable edit is rejected", False, "apply() returned instead of raising")
    except ValueError as exc:
        check("an unplaceable edit is rejected", "matches 0 times" in str(exc),
              str(exc).splitlines()[0])
    ambiguous = mutations.Mutation(
        id="BAD_ambiguous", title="anchor that occurs more than once",
        edits=(("      we_q       <= 1'b0;\n", "      we_q       <= 1'b0;\n"),))
    try:
        apply(src, ambiguous)
        check("an ambiguous edit is rejected", False, "apply() returned instead of raising")
    except ValueError as exc:
        check("an ambiguous edit is rejected", "matches 2 times" in str(exc),
              str(exc).splitlines()[0])
    noop = mutations.Mutation(id="BAD_noop", title="edits that change nothing",
                              edits=(("module host_spi (", "module host_spi ("),))
    try:
        apply(src, noop)
        check("a mutation that changes nothing is rejected", False,
              "apply() returned instead of raising")
    except ValueError as exc:
        check("a mutation that changes nothing is rejected",
              "leave the source unchanged" in str(exc), str(exc))


def control_broken_mutant_is_an_error_not_a_kill() -> None:
    """A mutant that does not compile must be an ERROR, never a kill.

    Without this, `failed > 0` would be the only signal and a mutant that iverilog rejects
    would be indistinguishable from one the bench caught -- or, worse, from one the bench
    never ran.  No simulation here: make stops at the compile.
    """
    src = source_text()
    broken = mutations.Mutation(
        id="BAD_no_compile", title="a mutant iverilog cannot compile", edits=(
            ("endmodule", "this is not verilog\nendmodule"),))
    with tempfile.TemporaryDirectory(prefix="hostmut-") as tmp:
        root = Path(tmp)
        path = generate.build(broken, src, root)
        proc = subprocess.run(
            [str(PY), str(HERE / "run.py"), "--dir", str(root), "--extra", broken.id],
            cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = proc.stdout
    print(out)
    check("a mutant that does not compile exits non-zero", proc.returncode != 0,
          f"exit {proc.returncode}")
    check("run.py reports it as an ERROR, not as a kill",
          "RESULT: FAIL" in out and "could not be measured" in out and "is not a kill" in out)
    row = [l for l in out.splitlines() if l.startswith("BAD_no_compile ")]
    check("run.py does not count it as killed", bool(row) and " NO " in row[0],
          row[0] if row else "no row for BAD_no_compile in the table")


def control_committed_copies_match_the_manifest() -> None:
    proc = subprocess.run([str(PY), str(HERE / "generate.py"), "--check"],
                          cwd=ROOT, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)
    print(proc.stdout)
    check("generate.py --check: the 7 committed copies are the manifest applied to src/",
          proc.returncode == 0, f"exit {proc.returncode}")


def main() -> int:
    print("=" * 78)
    print("control 1: the no-op mutant must SURVIVE the bench (one simulation)")
    control_noop_mutant_must_survive()
    print("=" * 78)
    print("control 2: a mutation that cannot be placed must be rejected (no simulation)")
    control_missing_anchor_is_rejected()
    print("=" * 78)
    print("control 3: a mutant that does not compile must be an ERROR, not a kill "
          "(no simulation)")
    control_broken_mutant_is_an_error_not_a_kill()
    print("=" * 78)
    print("control 4: the committed copies must be the manifest applied to src/ (no simulation)")
    control_committed_copies_match_the_manifest()
    print("=" * 78)
    if failures:
        print(f"RESULT: FAIL -- {len(failures)} control(s) did not behave: {failures}")
        return 1
    print("RESULT: PASS -- every negative control of the mutation driver behaved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())