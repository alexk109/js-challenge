"""The master's own START hold, measured off the wire, against fast mode's tHD;STA.

    .venv/bin/python -m fw.i2c_target.start_hold          # every fast-mode rate in SWEEP

What it is for. It helps explain why `fw/i2c_target` failed at 200 kHz
between two passing rates. That turned out to be a real firmware bug -- `clkdiv_max` sized the divider by
the master's low half-period and left the START window unbounded -- and a real trigger: the
`I2cController` master holds SDA low for **300 ns** after the SCL fall of a START, where fast
mode specifies **600 ns**, because `verif/vip/i2c.py:72` carries 260 ns in the fast row and
260 ns is the *fast-mode-plus* column of the spec table saved at
`research/ideation/round2/research/sources/C12/ti-snls650-i2c-timing-minima.md:24`
("tHD;STA -- Hold time for a start or a repeated start condition: 4 / 0.6 / 0.26 us").

`verif/vip/` is treated as read-only here, so nothing here patches a model: this reads
the bus through `Bus.tap` and prints the numbers the report quotes, and
`test_i2c_target.test_the_masters_start_hold_meets_the_fast_mode_minimum` is the same
measurement as a guard. `I2cTimingChecker` cannot see the violation because it tests this
hold against the same understated constant -- a short hold cancels out -- which is why every
`SWEEP` row above 400 kHz reports zero violations; the last line of every block says so.

Exits 0 always: this is a measurement, not a gate. What *fails* is the test's business.
"""
from __future__ import annotations

import sys

from fw.i2c_target.driver import clkdiv_max, start_window_ticks
from fw.i2c_target.measure import bus_for
from fw.i2c_target.test_i2c_target import (FAST_MODE_RATES, SPEC_FAST_THD_STA_NS,
                                           SPEC_FAST_THD_STA_TICKS, TICK_NS, start_holds)

#: The other column of the same spec line, for the record: what `verif/vip/i2c.py:72` carries.
SPEC_FAST_PLUS_THD_STA_NS = 260.0

#: The pre-fix divider at 200 kHz, `clkdiv_max(142)` as it was before the START-window
#: bound. A 23-core-cycle look is what missed the START.
PREFIX_CLK_DIV_200K = 22

def report_rate(rate_hz: int) -> dict:
    """One fast-mode rate: every START's tHD;STA measured off the wire, and what it means."""
    bench = bus_for(rate_hz)
    rows = start_holds(bench)
    model_ns = bench.ctl.min["t_hd_sta"] * TICK_NS
    print(f"\n{rate_hz / 1e3:.0f} kHz (mode {bench.mode}): "
          f"I2C_MODES['fast'].t_hd_sta = {model_ns:.0f} ns "
          f"({bench.ctl.min['t_hd_sta']} ticks), first_hold = {bench.ctl.first_hold} ticks, "
          f"CLK_DIV {bench.driver.clkdiv} (a {bench.driver.clkdiv + 1}-cycle look)")
    for i, row in enumerate(rows, 1):
        hold = row["hold"]
        if hold is None:
            print(f"   START {i}: SDA falls {row['sda_fell']}, SDA rises {row['sda_rise']} "
                  f"-- SCL never fell, so there is no tHD;STA to measure")
            continue
        verdict = ("meets fast mode" if hold >= SPEC_FAST_THD_STA_TICKS
                   else f"SHORT of fast mode by {SPEC_FAST_THD_STA_TICKS - hold} ticks")
        print(f"   START {i}: SDA falls {row['sda_fell']}, SCL falls {row['scl_fell']}, "
              f"SDA rises {row['sda_rise']}")
        print(f"            tHD;STA (SCL fall -> SDA rise) = {hold} ticks "
              f"= {hold * TICK_NS:.0f} ns   {verdict}")
        print(f"            the whole SDA-low window, which is what the divider is sized "
              f"against = {row['window']} ticks = {row['window'] * TICK_NS:.0f} ns")
    holds = [r["hold"] for r in rows if r["hold"] is not None]
    tightest = min(holds) if holds else None
    windows = [r["window"] for r in rows]
    print(f"   tightest tHD;STA here: {tightest} ticks ({tightest * TICK_NS:.0f} ns) against "
          f"fast mode's {SPEC_FAST_THD_STA_TICKS} ticks ({SPEC_FAST_THD_STA_NS:.0f} ns); "
          f"windows {min(windows)}-{max(windows)} ticks")
    print(f"   checker violations reported at this rate: {len(bench.checker.violations)} -- "
          f"the checker tests this hold against the same {model_ns:.0f} ns, so a short hold "
          f"cancels out")
    return {"rate_hz": rate_hz, "model_ns": model_ns, "tightest": tightest,
            "window": min(windows), "low_ticks": bench.ctl.low_ticks,
            "clkdiv": bench.driver.clkdiv, "first_hold": bench.ctl.first_hold}


def what_the_spec_column_would_give(rate_hz: int = 200_000) -> None:
    """The same bench with fast mode's own 600 ns instead of the fast-plus 260 ns.

    Pure arithmetic on the model's own formulas -- `I2cController._hold_point` and
    `start_window_ticks` -- so it says what a *spec-compliant* fast-mode master would put on
    this wire, and what the pre-fix divider would then have done about it. Nothing is
    rebuilt on the bus and no constant is changed; this is the counterfactual the report's
    verdict rests on.
    """
    bench = bus_for(rate_hz)
    hold = bench.ctl._hold_point(SPEC_FAST_THD_STA_TICKS)
    window = start_window_ticks(hold)
    look = PREFIX_CLK_DIV_200K + 1
    print(f"\n{rate_hz / 1e3:.0f} kHz with fast mode's own tHD;STA "
          f"({SPEC_FAST_THD_STA_NS:.0f} ns = {SPEC_FAST_THD_STA_TICKS} ticks):")
    print(f"   first_hold = {hold}, so the START window would be {window} ticks "
          f"= {window * TICK_NS:.0f} ns")
    print(f"   clkdiv_max({bench.ctl.low_ticks}, {hold}) = "
          f"{clkdiv_max(bench.ctl.low_ticks, hold)} -- and the pre-fix divider "
          f"{PREFIX_CLK_DIV_200K} is a {look}-cycle look, which is "
          f"{'wider than' if look > window else 'inside'} that window, so the pre-fix "
          f"firmware {'would have missed STARTs' if look > window else 'would NOT have failed'}"
          f" at {rate_hz / 1e3:.0f} kHz against a compliant master")


def main() -> int:
    print("# fw/i2c_target: the VIP master's START hold against fast mode's tHD;STA")
    print(f"#   python -m fw.i2c_target.start_hold")
    print(f"#   fast mode specifies {SPEC_FAST_THD_STA_NS:.0f} ns of tHD;STA; "
          f"verif/vip/i2c.py:72 carries {SPEC_FAST_PLUS_THD_STA_NS:.0f} ns in its fast row, "
          f"which is the fast-mode-plus column of the same table")
    print("#   research/ideation/round2/research/sources/C12/"
          "ti-snls650-i2c-timing-minima.md:24")
    for rate_hz in FAST_MODE_RATES:
        report_rate(int(rate_hz))
    what_the_spec_column_would_give()
    return 0


if __name__ == "__main__":
    sys.exit(main())
