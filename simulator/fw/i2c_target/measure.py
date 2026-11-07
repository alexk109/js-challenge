"""Measure the MVP I2C target firmware: the rate sweep, the margins, and metrics/fw.csv.

    .venv/bin/python -m fw.i2c_target.measure            # measure and append the rows
    .venv/bin/python -m fw.i2c_target.measure --dry-run  # measure only
    .venv/bin/python -m fw.i2c_target.measure --sweep    # the rate sweep only

This is what fw/i2c_target/README.md quotes. Every number it prints comes out of the same
bench fw/i2c_target/test_i2c_target.py uses, so the numbers and the tests cannot drift
apart, and every one of them is written to metrics/fw.csv with the git describe and the
date of the tree it was measured in.

What is measured
----------------
* the **rate sweep**: the same three transactions (write two bytes across a page boundary,
  then a current-address sequential read of five) at a list of SCL rates, each with the
  controller *and* the checker configured for it. A rate passes when every transaction is
  correct, `I2cTimingChecker` finds no violation at all, *and* the engine's `FDEBUG` is zero
  — a rate can satisfy the checker and still have stalled on an empty FIFO halfway through,
  which is what 800 kHz does. The sweep test applies the same three checks
  (`test_i2c_target._run_rate`), so the two cannot disagree about what "clean" means.
* the **margin probe**: a passive device on the bus that measures, from the wire, the
  smallest interval each rule ever got, each the way `I2cTimingChecker` takes it —
  `tHD;DAT` armed on the SCL falling edge and sampled at the next SDA move, which is
  "SDA valid after SCL falling". The checker only records violations, so a pass
  tells you nothing about how close it came; this does. The tightest rule is the answer to
  "the margin in core cycles at the tightest point".
* **whose** margin: the bus-wide minimum of an SDA-move rule is on this bus set by the
  *master's* bit placement (`I2cController._hold_point` plus one tick of bus resolution),
  not the firmware's, so the probe attributes every sample to whoever moved SDA (through
  `verif.vip.core.Bus.tap`, the bus's read-only observer) and keeps two minima per rule:
  the bus-wide one, which is what the checker would fail on, and the PIO's own, which is
  the design's. `tightest()` judges the PIO's own; both are printed so the next reader does
  not have to take the attribution on trust.
* the **reference cross-check**: the same transactions against `verif.vip.i2c`'s own
  `I2cEepromTarget`, and the two memory images compared. The firmware is judged by the
  VIP; this says whether the thing it emulated is the thing the VIP emulates.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys
from typing import List, Tuple

from iss import pio_iss
from verif.vip.core import Bus, Device
from verif.vip.i2c import (I2C_MODES, I2cEepromTarget, I2cTimingChecker, mode_minimums,
                           ns_to_ticks)

from . import assemble_program, clkdiv_max, poll_for
from .driver import I2cTargetDriver
from .test_i2c_target import TICK_NS, Bus2, scripts_for

CSV_PATH = pathlib.Path("metrics/fw.csv")
CSV_HEADER = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
              "peak_fifo_depth", "cycles_per_bit"]

#: The rates the sweep asks for. 100 kHz is standard mode's nominal, 400 kHz fast mode's,
#: and the two above them need minimums that no mode in `I2C_MODES` carries, so they are
#: spelled out here (see `bus_for`).
SWEEP_HZ = [50_000, 100_000, 200_000, 300_000, 400_000, 600_000, 800_000, 1_000_000]

#: Fast mode plus (UM10204's Fm+ row) as timing overrides, because `I2C_MODES` has no entry
#: for it and the sweep needs one: above 400 kHz the minimums that matter are the
#: half-periods, and fast mode's own (1300 ns / 600 ns) are stricter than Fm+'s.
FMP = {"t_low": 400, "t_high": 400, "t_su_sta": 400, "t_hd_sta": 400, "t_su_dat": 100,
       "t_hd_dat": 0, "t_su_sto": 400, "t_buf": 400}


def bus_for(rate_hz: float, clkdiv: int | None = None) -> Bus2:
    """A bench at `rate_hz`, with the divider and the poll counter worked out for it.

    The mode is the lowest one whose nominal rate covers `rate_hz`, and above fast mode's
    400 kHz the fast-mode-plus minimums are applied, because that is the class the rate
    belongs to. The divider is then the largest the bus allows (`clkdiv_max`, which is
    bounded both by the master's low half-period and by the START window `idle` polls for),
    and the poll counter is the smallest that outlasts one master high period (`poll_for`).
    """
    mode = "standard" if rate_hz <= I2C_MODES["standard"].freq else "fast"
    overrides = dict(FMP, freq=rate_hz) if rate_hz > I2C_MODES["fast"].freq else {"freq": rate_hz}
    return Bus2(mode=mode, overrides=overrides, clkdiv=clkdiv, address=0x50)


def measure(rate_hz: float) -> dict:
    """Run every script at `rate_hz` and report what the bus and the chip saw.

    A script that fails is recorded and the next one still runs, so one bad rate reports
    its whole story instead of hiding the rates after it.
    """
    bench = bus_for(rate_hz)
    problems: List[str] = []
    for name, script in scripts_for(bench):
        try:
            script(bench)
        except Exception as exc:                       # noqa: BLE001 - it is the measurement
            problems.append(f"{name}: {type(exc).__name__} {exc}")
            continue
        if bench.checker.violations:
            problems += [f"{name}: {v}" for v in bench.checker.violations]
        # One read, into a local: `reg_read` steps the bus until the read lands, so a
        # second call for the f-string would shift the master's phase for the next
        # transaction and make this bench differ from `_run_rate`'s by a tick per stalling
        # transaction. `_run_rate` and `one_mhz_owner` both read FDEBUG once per script.
        fd = bench.driver.fdebug()
        if fd:
            problems.append(f"{name}: FDEBUG {fd:#010x}")
        if bench.errors:
            problems.append(f"{name}: bus contention {bench.errors}")
    drv, chk = bench.driver, bench.checker
    return {
        "rate_hz": rate_hz,
        "mode": bench.mode,
        "clkdiv": drv.clkdiv,
        "poll": drv.poll,
        "shortest_period": chk.shortest_period,
        "measured_hz": chk.max_scl_hz,
        "violations": [str(v) for v in chk.violations],
        "rules": sorted({v.rule for v in chk.violations}),
        "problems": problems,
        "starts": chk.starts,
        "repeated_starts": chk.repeated_starts,
        "stops": chk.stops,
        "bytes": chk.bytes,
        "acks": chk.acks,
        "nacks": chk.nacks,
        "markers": drv.bus_free,
        "bytes_in": drv.bytes_in,
        "fdebug": drv.fdebug(),
        "peak_fifo_depth": drv.peak_fifo_depth,
        "margins": bench.probe.margins(),
        "tightest": bench.probe.tightest(),
        "tightest_bus": bench.probe.tightest(scope="bus"),
        "pio_name": bench.probe.own_name,
        "image": bytes(drv.eeprom.memory).hex(),
    }


def sweep(rates=None) -> List[dict]:
    return [measure(r) for r in (rates or SWEEP_HZ)]


def cross_check(rate_hz: float = 400_000) -> dict:
    """The same scripts against the VIP's own EEPROM, and both images compared."""
    bench = bus_for(rate_hz)
    scripts = scripts_for(bench)
    for _, script in scripts:
        script(bench)
    ours = bytes(bench.driver.eeprom.memory)

    ref = Bus2(mode=bench.mode, overrides=bench.overrides, address=0x50)
    for _, script in scripts_for(ref):
        script(ref)
    theirs = bytes(ref.eeprom.memory)
    return {"rate_hz": rate_hz, "identical": ours == theirs, "ours": ours.hex(),
            "theirs": theirs.hex()}


def reference_only(rate_hz: float, address: int) -> dict:
    """The VIP's EEPROM answering an address nobody claims, for the NAK comparison.

    `queue_write` returns the op and `I2cController.ops` is only its *queue*, popped when
    an op starts (`verif/vip/i2c.py`), so the answer has to be read off the object it hands
    back -- which is what the tests do with `_write_script`.
    """
    bench = Bus2(mode="fast", overrides={"freq": rate_hz}, address=0x50)
    ee = I2cEepromTarget(address=address, mode="fast", bus=bench.bus, **bench.nets)
    bench.bus.attach(ee)
    op = bench.ctl.queue_write([0x00, 0x5A])
    bench.ctl.run(bench.bus, limit=200_000)
    for _ in range(400):
        bench.driver.serve(1)
    return {"address": address, "acked": bench.driver.bytes_in > 0,
            "reference_acked": not op.nack,
            "memory": bytes(bench.driver.eeprom.memory[:4]).hex()}


def git_describe() -> str:
    try:
        out = subprocess.run(["git", "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


#: What goes in `metrics/fw.csv`'s `harness` column: the thing that produced the row, in
#: the style the other protocols use (see the existing rows).
HARNESS = ("fw/i2c_target/measure.py (ChipDevice + I2cController/I2cTimingChecker, "
           "20 ns/tick) clkdiv={clkdiv} poll={poll}")


def _row(now: str, desc: str, r: dict) -> list:
    """One row of metrics/fw.csv, in `CSV_HEADER` order — all nine of its columns.

    `peak_fifo_depth` and `cycles_per_bit` come out of `r`, the row `measure()` actually
    measured, and not off a fresh `bus_for()` bench that never ran (which would report 0).
    """
    return [now, desc, HARNESS.format(clkdiv=r["clkdiv"], poll=r["poll"]), "i2c_target",
            "pass", "skip", len(assemble_program().instrs), r["peak_fifo_depth"],
            f"{r['shortest_period']:.1f}"]


def _fmt(value) -> str:
    """A number or a dash for "no sample / no minimum", so the columns stay readable."""
    return "  -" if value is None else f"{value:3d}"


def report_hold_mechanism(rate: dict) -> str:
    """What the PIO's own tightest tHD;DAT is made of, in core cycles.

    The program puts the next bit on SDA after `wait 0 gpio SCL` returns, and an engine
    sees the fall three core cycles late (`iss.pio_iss.T_IN`); `out pindirs` is then the
    next instruction the divider releases, one `CLKDIV + 1` period later. What is left over
    is the phase between the master's fall and the divider's release grid, which the
    minimum over the run can only shrink: at CLKDIV 5 it was zero, so the tightest point
    measured is exactly T_IN + one divider period.
    """
    hold = rate["margins"]["tHD;DAT"]["own"]
    if hold is None:
        return ""
    period = rate["clkdiv"] + 1
    phase = hold - pio_iss.T_IN - period
    return (f"           tHD;DAT, {rate['pio_name']}'s own {hold} = T_IN "
            f"{pio_iss.T_IN} + one divider period {period} (CLKDIV {rate['clkdiv']})"
            f" + phase {phase}")


def report_margin(rule: str, m: dict, own_name: str) -> str:
    """One rule's line: the wire-wide minimum, the PIO's own, and who owned what.

    Both minima are printed because they are different questions. `bus` is what
    `I2cTimingChecker` would fail on and, on this bus, is set by the master's own bit
    placement; `own` is the minimum over the samples the PIO moved SDA in, which is the
    margin that matters. The tally is who owned how many samples and how tight each
    one's was, so a reader can see which of the two minima is the firmware's.
    """
    tally = ", ".join(f"{name}: {count} samples, min {low}"
                       for name, (count, low) in sorted(m["tally"].items())) or "no samples"
    req = _fmt(m["required"])
    bus_margin = "  -" if m["measured"] is None or m["required"] is None \
        else f"{m['measured'] - m['required']:3d}"
    own_margin = "  -" if m["own"] is None or m["required"] is None \
        else f"{m['own'] - m['required']:3d}"
    return (f"           {rule:9s} bus {_fmt(m['measured'])}/{req} -> {bus_margin} cycles"
            f" | {own_name}'s own {_fmt(m['own'])}/{req} -> {own_margin} cycles"
            f" | {tally}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dry-run", action="store_true", help="do not touch metrics/fw.csv")
    ap.add_argument("--sweep", action="store_true", help="only run the rate sweep")
    args = ap.parse_args(argv)

    rows = sweep()
    for r in rows:
        m = r["margins"]
        tight, bus_tight = r["tightest"], r["tightest_bus"]
        print(f"{r['rate_hz'] / 1e3:7.1f} kbit/s {r['mode']:8s} clkdiv {r['clkdiv']:3d} "
              f"poll {r['poll']:2d} -> {r['measured_hz'] / 1e3:7.1f} kbit/s "
              f"({r['shortest_period']:.0f} ticks/bit), {len(r['violations'])} violations "
              f"{r['rules'] or ''}")
        for rule in sorted(m):
            print(report_margin(rule, m[rule], r["pio_name"]))
        print(f"           tightest, {r['pio_name']}'s own: {tight['rule']} "
              f"{tight['measured']} vs {tight['required']} -> {tight['margin']} cycles")
        print(f"           tightest, bus-wide (the master's): {bus_tight['rule']} "
              f"{bus_tight['measured']} vs {bus_tight['required']} -> "
              f"{bus_tight['margin']} cycles")
        print(report_hold_mechanism(r))
        for p in r["problems"]:
            print("           PROBLEM " + p)
    check = cross_check()
    print(f"reference cross-check at {check['rate_hz'] / 1e3:.0f} kbit/s: images identical "
          f"{check['identical']}")
    nack = reference_only(400_000, 0x51)
    print(f"foreign address {nack['address']:#04x}: firmware acked {nack['acked']}, "
          f"reference acked {nack['reference_acked']}, memory {nack['memory']}")
    if args.sweep or args.dry_run:
        return 0

    now, desc = datetime.date.today().isoformat(), git_describe()
    csv_rows = [_row(now, desc, r) for r in rows
                if not r["violations"] and not r["problems"]]
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    new_file = not CSV_PATH.exists()
    with CSV_PATH.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if new_file:
            writer.writerow(CSV_HEADER)
        writer.writerows(csv_rows)
    print(f"appended {len(csv_rows)} row(s) to {CSV_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())