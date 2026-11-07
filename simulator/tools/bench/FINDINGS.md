# Bench findings

What the evidence bench measured once every level was wired up. Every number below is a record in the store
(`python -m tools.bench compare --metric <m>` or `trace <id>` reproduces it). The committed snapshot
`tools/bench/snapshots/committed.jsonl` holds the 2,051 records the bench itself produced (every level except `import`, whose
sources are in the repo; the physical-flow records in it were produced elsewhere and cannot be re-run from this repository without the EDA toolchain); `python -m tools.bench export --html wave4.html --snapshot tools/bench/snapshots/committed.jsonl`
renders it as one static page, and `make bench-report` rebuilds the full store, `latest.jsonl` and `latest.html`.
Fmax = 1000 / (20 ns - setup WNS) at the typical (`nom_typ_1p20V_25C`) and slow (`nom_slow_1p08V_125C`) corners.

## 1. Chip-level results per configuration (full LibreLane flow, IHP SG13CMOS5L, 6x4 tiles)

| config | synth area um2 | flow std-cell um2 | util % | fmax typ / slow MHz | DRC / LVS | TT precheck | power typ mW (idle) | gate level |
|---|---|---|---|---|---|---|---|---|
| MVP (2 engines, flop IMEM, FIFO 4) | 265,057 | 345,485 | 38.3 | 57.8 / 36.5 | 0 / 0 | pass | 8.71 | test.py 4/4; gl_rig = rtl |
| `imem_tech=sram2p` | 260,078 (incl. 78,768 macro) | 239,299 + 78,768 macro | 29.1 | **66.6 / 41.1** | 0 / 0 | pass | 6.41 incl. macro estimate (4.95 without) | test.py 4/4; gl_rig = rtl |
| `n_engines=3, fifo_depth=8, engine_route` | 460,866 | 599,910 | 66.5 | 56.1 / 35.0 | 0 / 0 | pass | 15.30 | gl_rig = rtl (test.py is MVP-map only) |
| `n_engines=3, engine_route` (FIFO 4) | 393,866 | 511,733 | 56.7 | 57.0 / 35.9 | 0 / 0 | pass | 12.62 | gl_rig = rtl (test.py is MVP-map only) |

The TT precheck column comes from the precheck logs, not from bench records (the `flow` backend stores `precheck_pass` as n/a: it is a separate step, `phys/run_tt_precheck.py --rundir`). The snapshot also keeps the superseded failures (e.g. an sram2p `gl` 3/4 run before the macro model fix, the two flows that timed out); queries take the latest run. The SRAM run's precheck result was reproduced number for number by the bench flow.
**A third engine costs area, not speed:** fmax moves 57.8 -> 57.0 typ; std-cell area 345k -> 512k (FIFO 4) or 600k (FIFO 8).

- Power on the MVP across workloads: idle 8.71, uart_tx 8.79, i2c 8.87, both engines 8.88, host SPI burst 8.77 mW
  (flops ~73 %, clock tree ~27 %); SRAM 6.41-6.50 mW, i.e. **-26 %**, the same as the research estimate. Off the MVP
  register map only `idle` power is valid (the activity testbench speaks the MVP map); see section 4.
- Synth area is linear in engines: 140,729 / 265,057 / 391,600 / 513,700 um2 for 1-4 engines (~124k per engine).
  FIFO depth 8 adds ~62k for three engines; the engine route adds 2-8k.
- Synth is not post-layout area: the flow adds 1.22-1.32x area and ~1.45x cells (buffers, resizing, clock tree, taps).
  The analytic model targets post-layout area. Its MVP value (342k vs 345k) agrees by construction (pnr_factor is fitted
  to the MVP flow); the out-of-sample predictions are sram2p 321k vs 318k (+1 %), 3 engines FIFO 4 504k vs 512k (-1.5 %,
  analytic has no engine_route) and FIFO 8 586k vs 600k (-2.3 %), all inside the model's band.

## 2. The SRAM floorplan (research/imem-sram/FLOORPLAN.md)

Two stacked macros at x=12 um left a 35 um channel facing 264 Metal2 pins: 1206 global-routing overflow and a detailed
router that never got below 54 violations (two independent 6 h runs). Side by side (x=7.5 and 707.9, on the PDN column
lattice, 5 um halo) gives 0 overflow, 0 DRT, DRC/LVS 0, precheck pass. A three-macro SRAM build cannot be placed side by
side (3 x 526 um > 1289 um die width), so "3 engines + SRAM" needs a new floorplan idea.

## 3. Behaviour (ISS / RTL / gate level agree)

- Bridge UART RX 115200 -> UART TX 9600 over the engine route, 22-byte burst: 11 / 3 / 0 bytes lost at FIFO depth
  4 / 8 / 16 (12-byte burst: 2 / 0 / 0). The loss path is only a non-blocking PUSH into a full FIFO.
- FIFO depth changes pass/fail and rate only through back-pressure; `fifo_headroom` is the rig's deepest point, not a
  rate-driven demand (0 at depth <= 4, payload-bound at 8/16).
- The spi rig needs two engines; the bench refuses it at `n_engines=1`.
- Parametric RTL and parametric ISS agree on every shared metric at all swept configs; the post-layout netlist
  (`gl_rig`) agrees with RTL on spi, swd, ps2 and onewire at the MVP, sram2p and the 3-engine layout; a netlist with
  ~90 inverters swapped for buffers fails, so the agreement is not vacuous.

## 4. Coverage limits found (and how the bench now reports them)

- `test/test.py` and the power activity testbench speak only the MVP register map. On the 3-engine map they measured an
  idle chip (gl 0/4, every power workload = idle). `gl` and non-idle `power` now refuse such configs; the 35 bad records
  are in `tools/bench/results/invalidated.jsonl` with the reason (44 lines in all: those 35, plus 9 `gl_rig` runs that died on a harness build race, two parallel jobs compiling into one build dir; fixed, re-run passes).
- `gl_rig` forces the internal register bus, so host_spi's SPI decode is not exercised on the netlist; for wide-map
  configs (more than 2 engines) the 10-bit host address path has no gate-level coverage yet.
- The SRAM IMEM is undefined until written in silicon; every simulation zeroes it (RTL `PIO_SRAM_MODEL`, gl_rig).
  `test_protocol_pin_map` used to rely on the flop's zero reset and now writes IMEM word 0 itself.

## 5. Corrections to earlier documents

- An earlier estimate quoted "fmax 61.6 typ / 38.8 slow" for an older design (`metrics/flow.csv` row mvp-slow).
  The MVP then gave 57.8 / 36.5 with the same formula; the bench flow is deterministic (identical to the earlier run).
- An earlier prediction that the 3-engine/FIFO 8 layout would not route (12,184 global-routing overflow, ~31k first-pass
  violations) was wrong: detailed routing converged to 0. Global-routing overflow is not a pass/fail predictor here.
