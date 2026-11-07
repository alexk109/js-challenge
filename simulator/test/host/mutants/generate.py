#!/usr/bin/env python
# SPDX-License-Identifier: Apache-2.0
#
# Write (or check) the committed copies of the mutants in mutations.py.
#
#   test/host/mutants/generate.py            rewrite every test/host/mutants/<id>/host_spi.sv
#   test/host/mutants/generate.py --check    fail if a committed copy is not what the edits
#                                            in mutations.py make of src/host_spi.sv
#   test/host/mutants/generate.py --only ID  one mutation
#   test/host/mutants/generate.py --out DIR  write somewhere else (selftest.py uses this for
#                                            the no-op negative control, which is never
#                                            committed)
#
# The copies are committed because they are the negative controls of the whole exercise:
# they are the evidence that the 16 tests of test/host/test_host_spi.py bite.  Keeping them
# derived from a manifest instead of hand-edited means `--check` can prove, at any commit,
# that each copy is the pristine RTL plus exactly the stated fault and nothing else.
#
# No simulator: this is file surgery.  The bench is `test/host/mutants/run.py`.

from __future__ import annotations

import argparse
import difflib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mutations import (  # noqa: E402
    MUTATIONS,
    NEGATIVE_CONTROL,
    DUT_NAME,
    SRC,
    Mutation,
    apply,
    source_text,
)


def build(mut: Mutation, src: str, outdir: Path, *, allow_noop: bool = False) -> Path:
    """Write `mut` into `outdir/<id>/host_spi.sv` and return the path."""
    if mut is NEGATIVE_CONTROL and outdir.resolve() == Path(__file__).resolve().parent:
        raise SystemExit("refusing to commit the negative control; pass --out")
    text = apply(src, mut, allow_noop=allow_noop or mut is NEGATIVE_CONTROL)
    path = outdir / mut.id / DUT_NAME
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def check(mut: Mutation, src: str, root: Path) -> bool:
    """True if the committed copy is exactly what the edits make of `src`."""
    path = root / mut.id / DUT_NAME
    if not path.exists():
        print(f"DRIFT {mut.id}: {path} does not exist")
        return False
    want = apply(src, mut)
    got = path.read_text()
    if want == got:
        return True
    print(f"DRIFT {mut.id}: {path} is not what mutations.py makes of {SRC}")
    for line in list(difflib.unified_diff(got.splitlines(), want.splitlines(),
                                          "committed", "expected", lineterm="", n=1))[:40]:
        print("  " + line)
    return False


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="fail if a committed copy has drifted from the manifest")
    ap.add_argument("--only", default="",
                    help="comma-separated mutation ids, or id prefixes (M4, M4_partial)")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent,
                    help="where the <id>/%s copies go" % DUT_NAME)
    ap.add_argument("--negative-control", action="store_true",
                    help="also build the no-op mutation (selftest.py; never committed)")
    args = ap.parse_args(argv)

    only = {i for i in args.only.split(",") if i}
    muts = [m for m in MUTATIONS
            if not only or any(m.id == i or m.id.startswith(i) for i in only)]
    if args.negative_control:
        muts = list(muts) + [NEGATIVE_CONTROL]
    if only and not muts:
        print(f"no mutation matches {sorted(only)}; known ids: "
              f"{[m.id for m in MUTATIONS]}", file=sys.stderr)
        return 2

    src = source_text()
    print(f"source: {SRC} ({len(src.splitlines())} lines, "
          f"sha256 {__import__('hashlib').sha256(src.encode()).hexdigest()[:16]})")
    bad = 0
    for mut in muts:
        try:
            if args.check:
                ok = check(mut, src, args.out)
                bad += not ok
                print(f"{'ok  ' if ok else 'DRIFT'} {mut.id}")
            else:
                path = build(mut, src, args.out)
                print(f"wrote {path} ({len(path.read_text().splitlines())} lines)")
        except ValueError as exc:
            bad += 1
            print(f"ERROR {mut.id}: {exc}")
    if args.check and not bad:
        print(f"{len(muts)} committed mutant copies are in sync with {SRC.name}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())