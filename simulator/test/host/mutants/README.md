# test/host/mutants -- the 7 mutations of `src/host_spi.sv`, and what kills them

**What this is.** `docs/mvp-verification-report.md` `### 7` claims the blind host-port bench
(`../test_host_spi.py`, 16 tests) has teeth, on the strength of 7 mutations of
`../host_spi_stub.sv` -- a stub the bench's own author wrote, never the RTL. This directory
re-applies the same 7 faults to a **copy of the real `src/host_spi.sv`** and runs the bench
against each one, so the claim is about the module that will go to silicon.

**Result (measured in a bench run).** 7/7 killed; the bench itself passes 16/16 on the unmutated
RTL. The negative controls of the driver are described below.

| # | mutation | report's count on the stub | here, on the real RTL | killed by |
|---|---|---:|---:|---|
| M1 | bus access one cycle late (`h = n + 2`) | 7 fail | **15 fail** | 15 of 16, incl. every read test |
| M2 | read fires on the last turnaround bit | 10 | **10** | 10 of 16 |
| M3 | burst address always increments (no FIFO hold) | 5 | **5** | 5 of 16 |
| M4 | a partial word survives the CSn rise | 7 | **1** | `test_cs_n_abort_mid_word` |
| M5 | `miso_sr` also shifts during the turnaround byte | 10 | **10** | 10 of 16 |
| M6 | MISO is not cleared while the port is idle | 1 | **1** | `test_cs_n_abort_during_a_turnaround_byte` |
| M7 | command byte decoded LSB first | 13 | **13** | 13 of 16 |

The counts are read from `test/host/results.xml` of each run (one `<testcase>` per test), not
from cocotb's summary line. Two counts differ from the report's, and neither is a hole:

* **M1, 7 -> 15.** The report's M1 delayed `bus_we`/`bus_re` on their way out. M1 here
  delays the whole cycle-`h` event -- the pulse gains one pipeline stage, and the two things
  that happen "at the access of cycle h" (the burst address step and the load of the read
  word into MISO) key off the moved pulse -- so it is the same fault expressed faithfully
  against RTL whose `h` is spread over four statements, and a stronger one. Every read test
  dies, because the word is sampled a bit-period late as well as a cycle late.
* **M4, 7 -> 1.** On the real RTL the fault is only reachable when CSn rises in the middle
  of a write word, and `test_cs_n_abort_mid_word` is the only test that does that (the other
  two CSn-abort tests abort during a turnaround byte and during a read word, where
  `phase_q != PH_WR`). One test reaching the fault and failing it is the correct result; the
  bench would have a hole if that test passed.

## How to run it

```sh
# all 7; exit 1 if any mutant survives the bench.  It runs 8 simulations.
.venv/bin/python test/host/mutants/run.py
make -C test/host host-mutate            # the same, from the repo root
make -C test/host host-mutate MUT=M4     # one of them (id or id prefix)
make -C test/host host-mutant-selftest   # the driver's own negative controls

# file surgery only, no simulator:
.venv/bin/python test/host/mutants/mutations.py     # what each mutation changes
.venv/bin/python test/host/mutants/generate.py --check   # committed copies == manifest
.venv/bin/python test/host/mutants/generate.py       # rewrite the copies
```

The full simulator output of every run is left in `../results-mutants/<id>.log` (gitignored
build output); `run.py` writes `results-mutants/<id>.xml` next to it.

## The files

| File | What it is |
|---|---|
| `mutations.py` | the manifest: each fault as `(old, new)` edits on the RTL text, with the report row it comes from. Importable and runnable; prints one line per mutation. |
| `<id>/host_spi.sv` | the committed mutant: the pristine RTL plus exactly that fault. These are the negative controls of the whole exercise, so they are in git. |
| `generate.py` | writes the copies from the manifest, or `--check`s that the committed copies still match it. |
| `run.py` | compiles each copy as the DUT, runs the 16 tests, parses `../results.xml`, prints the table, writes a CSV. Exits 1 if a mutant survives. |
| `selftest.py` | the negative controls (below). |

## The stale build that made the first campaign meaningless

Worth reading before reusing this: the first run of `run.py` reported **7/7 killed with the
same 10 failing tests for all seven mutants**, which was nonsense. The cause is
`cocotb_tools/makefiles/simulators/Makefile.icarus`:

```make
$(SIM_BUILD)/sim.vvp: $(VERILOG_SOURCES) ...
```

`sim.vvp` is rebuilt only when a source is **newer** than it. All seven mutant files are
written by `generate.py` in one go, so after the first mutant is built, every later mutant
looks up to date: `make` prints nothing, `vvp` keeps the first mutant, and the bench
re-reports the first mutant's failures -- which look exactly like "the bench kills
everything". `test/host/Makefile`'s `DUTFILE=` switch is therefore *not* enough on its own
to swap the DUT.

`run.py` defends against it three ways, and any of the three failing is an ERROR rather
than a row in the table:

1. `make clean` before every run;
2. the run's own build log must contain an `iverilog` command line naming the mutant -- with
   a stale build, make prints no recipe at all;
3. the compiled image `../sim_build/sim.vvp` must name exactly the mutant's path and no other
   `host_spi*.sv`. iverilog records each source's path in the vvp image, so this says what
   the tests *ran against*, not what was requested.

The evidence for each row is in the campaign log: the `iverilog` line, the `sim.vvp` line,
the file's sha256, and the bench's own `host-test: compiled ...` line.

## The negative controls (`selftest.py`)

Four, so that "7/7 killed" is not a claim the driver could make no matter what:

1. **A no-op mutation.** A "mutation" whose edits leave `src/host_spi.sv` byte-identical is
   built into a scratch directory and run through the same driver. The bench must report
   16/16 pass -- the mutant *survives* -- and `run.py` must say `RESULT: PASS` because it
   was told to expect a survivor. A driver that reported KILLED for everything, or skipped
   the run, fails here. It is the `NEGATIVE_CONTROL` entry in `mutations.py`, and
   `generate.py` refuses to commit it.
2. **An unplaceable mutation.** `mutations.apply()` raises if an edit's anchor does not
   occur exactly once in the source, if two anchors are ambiguous, or if the edits leave the
   file unchanged. Without that, a mutation whose anchor drifts (because `src/` changed)
   would silently patch nothing and be reported as a survivor -- or worse, patch the wrong
   line.
3. **A mutant that does not compile.** iverilog rejects it, `make` stops before the bench
   runs, and `run.py` must report an **ERROR** and exit non-zero. This is why `Result.killed`
   counts failed *tests* only: a mutant the simulator would not build has not been shown to
   be detectable, and must never be counted as one. (`run.py --extra <id>` is how the
   control gets a mutant that is not in the manifest past the `--only` filter.)
4. **The committed copies.** `generate.py --check` re-derives all seven from the current
   `src/host_spi.sv`, so a hand-edited or stale mutant cannot pass as a faithful copy of the
   RTL. It runs in the mutation campaign log too.

Controls 1 and 3 run a simulation each (control 1: 16 tests; control 3: none, `make` stops at
the compile); 2 and 4 are file surgery.

## What this does not do

* It does not check the register map: the register file behind the bench is
  `../regfile_model.py`, a dict with the documented reset values. Everything here is about
  the transport.
* It does not widen the stimulus. 16 tests, icarus, `T = 20 ns` -- the same scope as
  `docs/mvp-verification-report.md` sec. 7. No gate-level, no other clock period, no Verilator.
* It says nothing about `src/pio_*.sv` or the rest of the chip.