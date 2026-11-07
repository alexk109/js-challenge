"""The measured MVP performance table: sweep every protocol, write `research/perf/b/perf.csv`.

    python -m verif.perf.measure                       # all 14 protocols on the ISS
    python -m verif.perf.measure --protocols spi,i2c  # two of them
    FW_TARGET=rtl python -m verif.perf.measure --protocols spi  # the four RTL-targetable rigs

What it does, per protocol: walk that protocol's ladder of speed settings fastest first
(`verif/perf/rigs.py` `LADDER`), run one measured trial on each (`rigs.<name>_trial`), and stop
at the first setting the protocol's own VIP checker accepts.  That setting is the reported
maximum, and the rung above it -- the fastest setting that was *refused* -- is written to the
CSV too, so every maximum in the table has its boundary next to it.

Every number is a reading of a simulation: bits and cycles off an instrumented
`verif.vip.core.Bus` (`verif/perf/probe.py`), the verdict off the protocol's own checker.  The
CSV carries a `source` column naming the window, the nets and the counts the value came from,
so a reader can find the measurement rather than trust the table.

`main()` also puts the requested target's chip behind `iss.pio_iss.Chip` (`install_target`) and
then *checks* it did, because this bench runs outside pytest and so nothing else had: an
`FW_TARGET=rtl` run that had not installed the backend would measure the ISS and write
`target=rtl` into the CSV, which is the one failure this table cannot be allowed to have.

Nothing outside `verif/perf/` and the two output paths is written, and `src/` is not touched.
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

from verif.perf.hostport import AccessCost

from verif.perf import hostport, rigs
from verif.perf.probe import Trial

ROOT = Path(__file__).resolve().parents[2]
CSV_PATH = ROOT / "research" / "perf" / "b" / "perf.csv"
COLUMNS = ["protocol", "metric", "value", "unit", "target", "knob", "knob_value", "source"]

#: The order the table is written in: bus-shaped protocols first (they share a vocabulary),
#: then the point-to-point ones, then the two that only receive and the three that only send.
ORDER = ("spi", "i2c", "jtag", "swd", "ps2", "onewire", "dmx512", "wiegand", "quadrature",
         "necir", "uart_tx", "uart_rx", "manchester", "ws2812")


def git_stamp() -> str:
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty"], cwd=ROOT,
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):       # pragma: no cover
        return "unknown"


#: Set to anything to *skip* the backend install below.  Nothing sets it in normal use; it exists
#: so the negative control in `verif/perf/selftest.py` and `make perf-guard`
#: can show the guard refusing a bench that was
#: never given the target it says it is measuring, instead of that claim being taken on trust.
SKIP_INSTALL_ENV = "PERF_SKIP_TARGET_INSTALL"


def backend_problem(target: str, installed: type, wanted: type) -> Optional[str]:
    """`""` when `iss.pio_iss.Chip` will really build the target's chip, else why it will not.

    A pure predicate, so the refusal can be tested without a simulator.  It has to be a *pure*
    comparison of class identity rather than a request for confirmation: `fw/<proto>/test_<proto>.py`
    binds `Chip` at import time, so a bench that silently measured the ISS would look perfectly
    healthy, report plausible numbers and label every one of them `target=rtl`.
    """
    if installed is wanted:
        return None
    from iss import pio_iss

    if pio_iss.Chip is wanted:
        return None                            # installed, but under a class object of its own
    return (f"verif/perf/measure.py: FW_TARGET={target!r} but iss.pio_iss.Chip is "
            f"{pio_iss.Chip.__module__}.{pio_iss.Chip.__name__} and {target!r} needs "
            f"{wanted.__module__}.{wanted.__name__}; the rigs would measure "
            f"{pio_iss.Chip.__name__} and write it to the CSV as target={target!r}. "
            f"Refusing rather than publishing that.")


def install_target(target: str) -> type:
    """Put `target`'s chip behind `iss.pio_iss.Chip` and insist it took; return the class.

    `fw/targets.py` is the one registry (`fw/conftest.py` is its pytest caller), and this bench
    runs outside pytest, so nothing else had installed anything: at HEAD an `FW_TARGET=rtl` run of
    this bench measured the ISS and wrote `target=rtl` into the CSV.  The install has to happen
    here, before `measure_all()`, because the rigs import `fw.<proto>.test_<proto>` lazily and
    those modules bind `Chip` when they are imported.

    The check runs whether or not the install did, which is the whole point: with
    `PERF_SKIP_TARGET_INSTALL` set the install is skipped and this has to refuse rather than let
    a run go on to publish ISS numbers as RTL.

    Returns the class `iss.pio_iss.Chip` is *now*, whatever installed it, so the caller can print
    which chip the rows came from.
    """
    from iss import pio_iss

    from fw import targets as fw_targets

    if target not in fw_targets.TARGETS:
        raise SystemExit(f"verif/perf/measure.py: {target!r} is not a target; fw/targets.py knows "
                         f"{', '.join(fw_targets.NAMES)}")
    wanted = fw_targets.TARGETS[target].load()      # raises TargetUnavailable if not built here
    if not os.environ.get(SKIP_INSTALL_ENV):
        fw_targets.install(target)
    problem = backend_problem(target, pio_iss.Chip, wanted)
    if problem:
        raise SystemExit(problem)
    return pio_iss.Chip


def _sweep_row(protocol: rigs.Protocol, trial: Trial, target: str) -> dict:
    """One rung of the ladder: what was asked for, and what the checker said."""
    return dict(protocol=protocol.name, metric="sweep_point",
                value="pass" if trial.ok else "fail", unit=protocol.unit, target=target,
                knob=protocol.knob, knob_value=f"{trial.value:g}",
                source=(f"fastest is {protocol.faster}; " + (trial.why[:200] if trial.why
                        else "the protocol checker passed with no violations")))


def sweep(name: str, target: str = "iss", verbose: bool = True,
          speeds: Optional[Sequence[float]] = None) -> tuple[Trial, list[Trial]]:
    """One protocol: walk its ladder fastest first and stop at the first accepted setting.

    `speeds` replaces the ladder, which is what the RTL run uses: the ISS run has already found
    each boundary, so the RTL run only has to re-measure the *same setting* to compare, and the
    protocols whose payload is hundreds of thousands of core cycles (PS/2, 1-Wire) are not worth
    re-sweeping at 6 000 cycles a second.
    """
    protocol = rigs.ladder_for(name)
    tried: list[Trial] = []
    for speed in (protocol.speeds if speeds is None else speeds):
        started = time.perf_counter()
        trial = protocol.trial(speed)
        tried.append(trial)
        if verbose:
            print(f"  {name:11s} {protocol.knob}={trial.value:<12g} "
                  f"{'pass' if trial.ok else 'FAIL':4s} "
                  f"{'' if trial.ok else trial.why[:90]}"
                  f"  ({time.perf_counter() - started:.1f}s)", flush=True)
        if trial.ok:
            return trial, tried
    raise SystemExit(f"verif/perf/measure.py: {name}: no setting on its ladder passed "
                     f"({protocol.speeds}); the ladder or the firmware is wrong")


def host_port_rows(target: str, ticks_per_bit: int = hostport.TICKS_PER_BIT
                    ) -> tuple[list[dict], list[AccessCost]]:
    costs = hostport.measure(ticks_per_bit)
    rows = []
    for cost in costs:
        rows.append(dict(protocol="spi_host_port", metric=cost.name, value=f"{cost.cycles:d}",
                         unit="core cycles per transaction", target=target,
                         knob="ticks_per_sck_bit", knob_value=f"{ticks_per_bit}",
                         source=cost.describe() + f"; framing check "
                                f"{'passes' if not hostport.check_framing(cost) else 'FAILS: ' + str(hostport.check_framing(cost))}"))
    return rows, costs


def measure_all(names: Sequence[str], target: str = "iss", with_host_port: bool = True,
                verbose: bool = True,
                pinned: Optional[dict[str, Sequence[float]]] = None) -> list[dict]:
    """Every row of the table, in the order `ORDER` gives."""
    rows: list[dict] = []
    host_costs: list[AccessCost] = []
    if with_host_port:
        host_rows, host_costs = host_port_rows(target)
        rows.extend(host_rows)
    for name in names:
        protocol = rigs.ladder_for(name)
        if target not in protocol.targetable and target != "iss":
            raise SystemExit(f"verif/perf/measure.py: {name} is not run on target {target!r}; "
                             f"its targetable targets are {protocol.targetable or ('iss',)}")
        print(f"{name} ({protocol.knob}, faster is {protocol.faster}):", flush=True)
        best, tried = sweep(name, target=target, verbose=verbose,
                            speeds=(pinned or {}).get(name))
        for trial in tried:
            rows.append(_sweep_row(protocol, trial, target))
        best.extra.setdefault("responds", protocol.responds)   # so an "n/a" row says why
        rows.extend(best.rows(target=target))
        # Whatever else the run measured about itself, as rows of their own: a number that is
        # only in a Python variable is not evidence, and these are the ones a reader wants to
        # check the headline rows against (the slot period behind a 1-Wire bit rate, the divider
        # behind a requested baud, the violation rules a rejected rung tripped).
        for key, value in best.extra.items():
            if isinstance(value, (int, float, str)) and str(value):
                rows.append(dict(protocol=name, metric=f"measured_{key}",
                                 value=f"{value}" if isinstance(value, str)
                                 else f"{value:g}", unit="see source", target=target,
                                 knob=protocol.knob, knob_value=f"{best.value:g}",
                                 source=f"also measured in this run ({name} at "
                                        f"{protocol.knob}={best.value:g} {protocol.unit})"
                                        + (f"; {best.extra_notes[key]}"
                                           if key in best.extra_notes else "")))
        # Metric 6 is a property of the host port, so every protocol carries the same measured
        # number and says where it came from rather than pretending to have measured it again.
        for cost in _host_costs(host_costs):
            rows.append(dict(protocol=name, metric=f"host_port_{cost.name}_cycles",
                             value=f"{cost.cycles:d}", unit="core cycles per transaction",
                             target=target, knob="ticks_per_sck_bit",
                             knob_value=f"{cost.ticks_per_bit}",
                             source=cost.describe() + "; the SPI host port is one thing, so "
                                    "this is the spi_host_port row, not a per-protocol one"))
    return rows


def _host_costs(costs: list[AccessCost]) -> list[AccessCost]:
    """The two shapes a protocol's host actually performs: one write, one read."""
    return [c for c in costs if c.name in ("write", "read")]


def write_csv(rows: list[dict], path: Path = CSV_PATH) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in COLUMNS})
    return path


def print_table(rows: list[dict]) -> None:
    """The same numbers as the CSV, one line per protocol and metric, for the log."""
    print("\n" + "=" * 100)
    print(f"{'protocol':12s} {'metric':26s} {'value':>14s}  {'unit':22s} target  knob")
    for row in rows:
        if row["metric"] == "sweep_point":
            continue
        print(f"{row['protocol']:12s} {row['metric']:26s} {row['value']:>14s}  "
              f"{row['unit']:22s} {row['target']:6s}  {row['knob']}={row['knob_value']}")


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--protocols", default="",
                    help="comma separated; default is every protocol in fw/")
    ap.add_argument("--target", default=os.environ.get("FW_TARGET", "iss").lower(),
                    help="iss or rtl (the default is FW_TARGET, i.e. iss)")
    ap.add_argument("--csv", default=str(CSV_PATH), help="where the table goes")
    ap.add_argument("--no-host-port", action="store_true",
                    help="skip the SPI host-port rows")
    ap.add_argument("--speeds", default="",
                    help="pin the settings instead of walking the ladder: "
                         "\"spi=1,ps2=94\" (the RTL run does this: the ISS run has the boundary)")
    ap.add_argument("--append", action="store_true",
                    help="add to the CSV instead of replacing it (for an RTL run)")
    args = ap.parse_args(argv)

    names = [n.strip() for n in args.protocols.split(",") if n.strip()] or list(ORDER)
    stamp, started = git_stamp(), time.time()
    print(f"verif/perf/measure.py: {len(names)} protocol(s) on FW_TARGET={args.target}, "
          f"git {stamp}, {time.strftime('%F %T')}")
    print(f"command: {' '.join([sys.executable, '-m', 'verif.perf.measure'] + (argv or sys.argv[1:]))}")
    # Before the first `fw.<proto>.test_<proto>` import, which binds `Chip`: see install_target.
    chip = install_target(args.target)
    print(f"backend: iss.pio_iss.Chip is {chip.__module__}.{chip.__name__} (target "
          f"{args.target!r})")
    pinned: dict[str, tuple[float, ...]] = {}
    for item in (p.strip() for p in args.speeds.split(",") if p.strip()):
        name, _, values = item.partition("=")
        if name not in rigs.PROTOCOLS:
            raise SystemExit(f"verif/perf/measure.py: --speeds: no such protocol: {name}")
        pinned[name] = tuple(float(v) for v in values.split("|"))
    rows = measure_all(names, target=args.target, with_host_port=not args.no_host_port,
                       pinned=pinned)
    path = Path(args.csv)
    if args.append and path.exists():
        with open(path, "a", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=COLUMNS)
            for row in rows:
                writer.writerow({k: row.get(k, "") for k in COLUMNS})
    else:
        write_csv(rows, path)
    print_table(rows)
    print(f"\n{len(rows)} rows written to {path.relative_to(ROOT) if path.is_relative_to(ROOT) else path}"
          f" in {time.time() - started:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())