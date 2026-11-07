"""`make fuzz-selftest`: prove that the lockstep harness is not vacuous.

Every check must behave,
or this exits 1:

| check | what it runs | what it must do |
|---|---|---|
| 0 | `python -m verif.fuzz.planted.generate --check` | `verif/fuzz/planted/` is still a copy of `src/` + the plants |
| (a) | the trivial stimulus, real RTL, real ISS | every compared signal matches |
| (b) | the trivial stimulus, `verif/fuzz/planted` with plants 1, 2 and 5 | a mismatch, in the field the plant breaks |
| (c) | the trivial stimulus, real RTL, ISS monkeypatched by `verif/fuzz/plant_iss.py` | a mismatch, in the field the patch breaks |

The negative control for every field `--deep` adds, which is the only way to
claim the deep comparison sees more than the 37 shallow signals do:

| check | stimulus | build / patch | what it must do |
|---|---|---|---|
| (d) | `gen.fifofill()` | real RTL | matches, deep and shallow |
| (e) | `gen.fifofill()` | plant 7 (the FIFO stores a word one slot along) | a mismatch in `eng0.tx[i]` **with** `--deep`, and **no** mismatch without it: that pair is the claim that only a FIFO-contents compare can catch it |
| (f) | `gen.fifofill()` | `plant_iss_deep` `fifo`, `config`, `div`, `imem`, `fdebug`, `pinreq` | a mismatch, in the deep field that plant breaks |
| (g) | `gen.pinloop()` | real RTL | matches, with and without `--pullup` |
| (h) | `gen.pinloop()` | `FUZZ_PAD_FAULT`, a deliberately wrong pad model | a mismatch: the pad model is checked, not assumed |

(b), (c) and (f) also require the mismatch to be in the field the plant breaks, so a check cannot
pass because the harness found some other difference first.

    python -m verif.fuzz.selftest [--cycles 200] [--sim verilator|icarus]
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verif.fuzz import lockstep                                   # noqa: E402

#: What each planted bug must break, so that a mismatch in the wrong place is a failure too. More
#: than one name where the deep comparison finds the same fault earlier than the shallow one: the
#: pin resolution of a cycle is seen in that cycle (`res_dir`) and on the register in the next
#: (`pin_oe`), and the request before both.
EXPECTED = {
    "plant 1 (jmp x-- does not write X-1)": ("eng0.x",),
    "plant 2 (the wrap ignores WRAP_BOTTOM)": ("eng0.pc",),
    "plant 3 (side-set repeats while an instruction is stalled)": ("pin_out", "eng0.ss_we"),
    "plant_iss (the ISS does not wrap to WRAP_BOTTOM)": ("eng0.pc",),
    "plant 5 (engine 0 wins a pin conflict instead of engine 1)": ("pin_oe", "res_dir",
                                                                   "res_level"),
    "plant 7 (the FIFO stores a word one slot along)": ("eng0.tx[0]",),
    "fifo (an ISS TX FIFO word is not the word the host pushed)": ("eng0.tx[1]",),
    "config (the ISS stores PINCTRL.IN_BASE one pin off)": ("eng0.pinctrl",),
    "div (the ISS divider accumulator loses its low bit)": ("eng0.div_acc",),
    "imem (the ISS drops a host IMEM write)": ("eng0.imem[17]",),
    "fdebug (the ISS sets an FDEBUG flag nothing set)": ("fdebug",),
    "pinreq (an ISS OUT request claims the wrong direction)": ("eng0.os_dir",),
    "pad fault (the harness drives a wrong pad model)": ("pad[0]",),
}
#: the deep negative controls: `FUZZ_DEEP_PLANT` value -> (check name, stimulus)
DEEP_PLANTS = {
    "fifo": "fifo (an ISS TX FIFO word is not the word the host pushed)",
    "config": "config (the ISS stores PINCTRL.IN_BASE one pin off)",
    "div": "div (the ISS divider accumulator loses its low bit)",
    "imem": "imem (the ISS drops a host IMEM write)",
    "fdebug": "fdebug (the ISS sets an FDEBUG flag nothing set)",
    "pinreq": "pinreq (an ISS OUT request claims the wrong direction)",
}


class Check:
    def __init__(self, name: str, want: str) -> None:
        self.name = name
        self.want = want
        self.ok = False
        self.detail = ""

    def __str__(self) -> str:
        return f"[{'ok' if self.ok else 'FAIL'}] {self.name}: {self.want}; {self.detail}"


def _verdict(result: dict[str, Any], check: Check, want: str) -> None:
    """Turn one run into a pass/fail, with the reason in `detail`."""
    if want == "match" and result["status"] == "pass":
        check.ok = True
        check.detail = (f"{result['cycles_run']} cycles match over {result['fields']} fields "
                        f"(deep {result.get('deep')}); covered "
                        f"{result['coverage']['sm_cycles']} SM cycles e0/e1")
        return
    if want == "mismatch" and result["status"] == "mismatch":
        name = result["mismatch"]["name"]
        cycle = result["mismatch"]["cycle"]
        expected = EXPECTED[check.name]
        check.ok = name in expected
        check.detail = (f"mismatch at cycle {cycle}, {name}: rtl={result['mismatch']['rtl_hex']} "
                        f"iss={result['mismatch']['iss_hex']}"
                        + ("" if check.ok else f"; expected one of {expected}"))
        check.detail += f"; report {result['report']}"
        return
    if want == "match":
        check.detail = (f"expected a clean run, but {result['status']} at cycle "
                        f"{result.get('first_divergence_cycle')} "
                        f"{result.get('mismatch', {}).get('name')}: "
                        f"rtl={result.get('mismatch', {}).get('rtl_hex')} "
                        f"iss={result.get('mismatch', {}).get('iss_hex')} "
                        f"(report {result.get('report')})")
    else:
        check.detail = f"expected a mismatch, but {result['cycles_run']} cycles matched"
    check.ok = False


def main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="the lockstep harness selftest")
    ap.add_argument("--cycles", type=int, default=200)
    ap.add_argument("--sim", default="verilator", choices=["verilator", "icarus"])
    ap.add_argument("--jobs", default="-j2")
    args = ap.parse_args(argv)

    checks: list[Check] = []
    stamp = lockstep.git_stamp()
    print(f"fuzz-selftest: simulator {args.sim}, trivial stimulus, {args.cycles} cycles, "
          f"deep {lockstep.DEEP_DEFAULT}, git {stamp}, {time.strftime('%Y-%m-%d %H:%M:%S')}")

    stale = subprocess.run([sys.executable, "-m", "verif.fuzz.planted.generate", "--check"],
                           cwd=ROOT, capture_output=True, text=True)
    sync = Check("planted copies are in sync with src/", "in sync")
    sync.ok = stale.returncode == 0
    sync.detail = (stale.stdout or stale.stderr).strip().splitlines()[-1] if not sync.ok else \
        stale.stdout.strip()
    checks.append(sync)

    plan = [
        ("(a) clean run on the trivial program", "match", {}),
        ("plant 1 (jmp x-- does not write X-1)", "mismatch", {"planted": True, "plant": 1}),
        ("plant 2 (the wrap ignores WRAP_BOTTOM)", "mismatch", {"planted": True, "plant": 2}),
        ("plant 5 (engine 0 wins a pin conflict instead of engine 1)", "mismatch",
         {"planted": True, "plant": 5}),
        ("plant_iss (the ISS does not wrap to WRAP_BOTTOM)", "mismatch",
         {"modules": "lockstep,plant_iss"}),
    ]
    for name, want, kwargs in plan:
        check = Check(name, want)
        print(f"  running {name} ...", flush=True)
        result = lockstep.run(seed=1, cycles=args.cycles, sim=args.sim, trivial=True, quiet=True,
                              jobs=args.jobs, repro_dir=lockstep.HERE / "repros" / "selftest",
                              **kwargs)
        _verdict(result, check, want)
        checks.append(check)

    # ---- the negative controls of the deep comparison ---------------------
    fifo_kwargs = dict(seed=1, cycles=120, sim=args.sim, quiet=True, jobs=args.jobs,
                       repro_dir=lockstep.HERE / "repros" / "selftest")

    def run_fifo(label: str, want: str, extra: dict[str, Any]) -> Check:
        check = Check(label, want)
        print(f"  running {label} ...", flush=True)
        result = lockstep.run(fifofill=True, **fifo_kwargs, **extra)
        _verdict(result, check, want)
        return check

    checks.append(run_fifo("(d) clean deep run on the FIFO-fill program", "match", {"deep": True}))
    checks.append(run_fifo("(d) clean shallow run on the FIFO-fill program", "match",
                           {"deep": False}))
    # The pair that matters: plant 7 is invisible to the 37 shallow signals and visible to the
    # FIFO-contents compare, in the same run.
    checks.append(run_fifo("plant 7 (the FIFO stores a word one slot along)", "mismatch",
                           {"planted": True, "plant": 7, "deep": True}))
    checks.append(run_fifo("plant 7 with --no-deep (the shallow 37 must miss it)", "match",
                           {"planted": True, "plant": 7, "deep": False}))
    for plant, name in DEEP_PLANTS.items():
        checks.append(run_fifo(name, "mismatch",
                               {"deep": True, "modules": "lockstep,plant_iss_deep",
                                "extra_env": {"FUZZ_DEEP_PLANT": plant}}))
    checks.append(run_fifo("(g) clean run on the pin-loopback program", "match",
                           {"pinloop": True, "deep": True}))
    checks.append(run_fifo("(g) the same with --pullup", "match",
                           {"pinloop": True, "deep": True, "pullup": True}))
    checks.append(run_fifo("pad fault (the harness drives a wrong pad model)", "mismatch",
                           {"pinloop": True, "deep": True, "pad_fault": True}))

    print("")
    for check in checks:
        print(str(check))
    failed = [c for c in checks if not c.ok]
    print("")
    print(f"fuzz-selftest: {'PASS' if not failed else 'FAIL'} "
          f"({len(checks) - len(failed)}/{len(checks)} checks behaved as required)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
