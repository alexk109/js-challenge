# verif/perf: the measured performance bench

This is the machinery behind `research/perf/b/perf.csv`: the MVP's performance table, measured
rather than derived. Four files, and one rule each.

| File | What it is | Rule |
|---|---|---|
| `probe.py` | the instruments: an instrumented `verif.vip.core.Bus` (edge log), the register-access log, per-cycle engine samples | reads a bus, never changes one; a signal it cannot read becomes `n/a`, never `0` |
| `rigs.py` | the 14 protocol benches, one measured `Trial` per speed setting | reuses each protocol's own bench and checker; adds no cycle and removes none |
| `hostport.py` | what a register access costs over the SPI host port at `SCK = core/8`, and the four host timing phases measured with it | one measurement, quoted into every protocol's row |
| `measure.py` | the sweep, the target install and its check, and the CSV writer | walks each ladder fastest-first and stops at the first accepted setting; refuses to label a run with a target it is not on |
| `selftest.py` | the instruments on a hand-checkable bus, plus six negative controls | every check must be able to fail |

## How to run it

```
make perf                        # all 14 protocols on the ISS -> research/perf/b/perf.csv
make perf PROTO=spi,i2c          # only those
make perf-rtl                    # spi, ps2, swd, onewire against src/pio_chip.sv, appended
make perf-selftest               # the instruments and the six negative controls
make perf-guard                  # the negative control for perf-rtl's target guard: must refuse
```

All five are simulations: the sweeps are long, the
self-test is short. The commands behind them are
`.venv/bin/python -m verif.perf.measure`, `.venv/bin/python -m verif.perf.selftest` and
`.venv/bin/python -m verif.perf.measure` with `PERF_SKIP_TARGET_INSTALL=1`
(never run them outside the queue on a shared machine). A full ISS sweep takes a few minutes,
almost all of it Wiegand: one 26-bit frame is 780 000 core cycles of Python stepping. The RTL
sweep takes ~12 minutes of wall clock and needs a `medium` slot twice over.

The results are in `research/perf/b/perf.csv`; the definitions and boundaries are below.

## Three things this bench refuses to do

These are the failure modes that would make the table look fine and be worthless, so each is
refused in code and checked by a negative control.

1. **Publish a number nobody read.** `Probe.sm_available` is False when the chip's engine object
   does not expose `sm_cycle` (which is the case on `FW_TARGET=rtl`: `fw/rtl_harness.py`'s
   `_RtlEngine` answers `STATE_KEYS`, the SM configuration words, `imem`, `tx` and `rx`, and
   raises for the rest), and `Trial.rows()` then writes `n/a` and the reason. Control:
   `test_a_target_that_hides_sm_cycle_reports_n_a_not_zero`.
2. **Label a run with a target it is not on.** `measure.py` `main()` installs the requested
   target through `fw/targets.py` before the rigs' first `fw.<proto>` import, then compares class
   identity and refuses to start if `iss.pio_iss.Chip` is not the class that target asks for.
   Nothing but pytest's `fw/conftest.py` installs a backend, and this bench is not run under
   pytest. Control: `test_the_target_guard_refuses_a_bench_that_was_never_installed`, and
   `make perf-guard` for the same thing from the command line.
3. **Reconstruct a measurement from a formula.** Where a rig wanted a number its driver would not
   give it -- `SwdDriver` has no `swclk_hz`, so the SCK frequency row is `50 MHz / period` -- the
   row says `FALLBACK, not a measurement` and names the driver refusal, rather than arriving in
   the shape of a reading. The same reasoning is why the RTL rows of `sm_cycles_per_bit` are
   `n/a` and not rebuilt from `CTRL` and `CLKDIV` inside the probe.

## What "measured" means here, exactly

* **Every bench is the firmware's own.** Each rig calls the `Bench`/`Rig`/`build()` from
  `fw/<proto>/test_<proto>.py`, loads the `program.pio` that ships, drives it with that
  protocol's own driver, and is judged by that protocol's own VIP checker. `ok` is the checker's
  verdict (`violations == []`, the decoder agreeing with the payload, the FIFOs clean) -- never
  this bench's opinion of a waveform. `rigs.py` says so per protocol, in the docstring.
* **Every cycle count comes off the bus.** `probe.attach()` wraps the *instance* methods of the
  `Bus` and the `ChipDevice` the bench already built. An edge is timestamped with the cycle its
  new level is first visible on the net (the cycle after the one that produced it, because
  `core.Bus` resolves drives at the end of a tick); an access is timestamped with the bus access
  cycle `h` it was performed in. No cycle is added or skipped anywhere.
* **No number is a formula from `docs/`.** A datasheet limit may appear in a *verdict* (the
  checkers use them) but never in a measurement. Where a window's end cannot be an edge -- the
  last bits of a UART frame, the last slot of a 1-Wire frame -- it is taken from a period
  measured elsewhere in the same run, and the rig says which.

## The two definitions worth arguing about

**The payload window.** `max_bit_rate` is `bits on the wire / core cycles the wire took`, where
the window runs from the protocol's first payload event to the last one. Each rig picks its own
window because each protocol's last bit is a different thing: for SPI it is the CSn rise, for
I2C the STOP, for JTAG/PS/2/SWD/DMX512 the last clock of the last frame, for Manchester half a
bit after the last mid-bit transition, for 1-Wire and Wiegand one measured slot or pulse period
after the last pull-low. `research/perf/b/README.md` lists the window each protocol used, so a
reader who disagrees with one of them can see exactly what was measured.

**The speed knob.** Each protocol gets the knob it really has: a divider integer where the
divider is the knob (SPI, I2C, JTAG, PS/2, 1-Wire), a bit period in core cycles (SWD), a
requested rate (UART baud, Manchester, DMX512, NEC), a pulse width (Wiegand), a program choice
(WS2812) or an outside-world rate (quadrature RPM). `rigs.LADDER` is the table, with a note per
protocol on what stops it going further.