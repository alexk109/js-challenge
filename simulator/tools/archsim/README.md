# archsim: architecture / performance simulator

A performance model of the PIO-style protocol engine chip, in one offline HTML page. It answers: **which protocols can
this architecture serve, what limits it, what would it take to serve one that fails, and how far to believe the answer.**
Performance only: it does not simulate RTL, route, or time anything at gate level. The model is **hybrid and analytic**:
closed-form limits per (architecture point x protocol), every constant carried in `data/inputs.json` with its source, and
the model **calibrated against measured points** (golden ISS, perf-a/perf-b, LibreLane flow runs, probes) in Trust.

Normative text: `docs/archsim-spec.md` (what it must do) and `tools/archsim/PLAN-v2.md` / `PLAN.md` (how, with every formula).

## Open it

`tools/archsim/archsim.html`, any browser, offline (one self-contained file). Everything you do is in the URL hash, so a
view can be shared as a link. Start from the MVP ("S0: MVP as built"); change one knob at a time and watch the chip card.

## Five-minute tour

1. Open the page; it starts on the MVP ("S0: MVP as built"). Capability shows 4 served, 15 impossible.
2. In Configure pick the preset `A: lane on the bus clock` and press **pin**; pick `Wide host bus + 4 KB SRAM buffer` and press **pin** again. Capability now has three columns.
3. Read the "N of 19 protocols differ" note under the tabs, then tick **only rows that differ across points** (the diff toggle).
4. Click a cell: the side panel ranks every limit and draws the derivation down to parameters and sources.
5. Click the protocol name, then the **Timing** tab: the same protocol's reflex/response budget, edge placement and receive sampling for the focused point (switch the point in the focus bar).
6. Open **Explore**, "what would it take": the smallest knob change(s) that serve that protocol, including ones that break the chip check (labelled, with the fix) and ISA-based ones (labelled `estimate`).
7. Back in Capability choose **Scenario (at the same time)**, tick several protocols (or press a reference mix such as *Sensor hub*), and read the shared-resource check and the "at most N can run together" line.
8. Scroll to **Pairwise compatibility**: the "N of M pairs run together" tally and the matrix of every served-alone protocol with every other; hover a cell for the failing rows, click it for the side panel.

## How to use it

**Set up a chip (Configure, left column).** The architecture is a *choice*, not an outcome. Pick a preset, move knobs
(sliders, numbers, quick buttons), pin up to five points to compare, reset to the MVP. The live block diagram shows the
choice (box size ~ sqrt(area), not a floorplan; click a box for its detail). The **chip check** card above the knobs says
whether the chip itself can be built: core clock vs fmax, die area vs usable, board clock, pins (and, with ISA levers on,
opcode room). Click a row for its derivation; "breakdown" jumps to the tab that composes it.

**What can it do (Capability).** *Single protocols*: the matrix of protocols x points, each cell one protocol on the chip
loaded with only its firmware: verdict (`served` / `marginal` / `impossible`, never colour alone), the binding limit and
its margin with a pess..opt bar. A hatched column means the chip check of that point fails or is marginal: the verdicts
assume the chip could be built. *Scenario (at the same time)*: tick several protocols to check them together (engines,
pins, instruction memory, host loop, link bandwidth). Click a cell for the ranked limits and the full derivation.

**Can X and Y run at the same time? Which reference mixes does this point run?** Capability, *Scenario (at the same time)*.
*Reference mixes on each point* is a table of typical products (debug probe, sensor hub, maker I/O, ...) against every pinned
point, each cell pass / marginal / fail with the failing shared resource and "N of M can run together". *Pairwise
compatibility* checks every pair of protocols that are served alone on the focused point (a protocol with itself = two instances)
and prints "N of M pairs run together". Tick a set (or press a reference mix) and the page gives the largest number of the ticked
protocols that can run together (an exact search; capped at 3000 checks, and it says so when the cap bites). Pass means the
shared-resource check passes: engines, pins, program memory, link bandwidth and time share, host decision loop. It does **not**
check host CPU time, arbitration latency between engines, differing core-clock needs or interference on shared pins (see Trust, "Not modelled").

**Does a bigger buffer buy protocols?** Only for rows whose frame is larger than the FIFO. The `frames` tag holds the large-frame
rows (SD 512 B blocks, 10BASE-T 1518 B, USB isochronous 1023 B, logic-analyzer captures of 1/4/16 KB). Pick tag `frames` in Single
protocols and change *Deep SRAM-backed FIFO* and its direction in Configure: a capture needs an RX (or both) deep FIFO, a TX one does nothing for it.

**Why does protocol X fail, and what would it take?** Pick X in the focus bar (it persists across tabs), then:
Timing (reflex/response budget, edge placement, receive sampling, core clock vs fmax), Throughput (host link, decision loop,
buffer fill against the need), Area & pins (waterfall against the die, pin allocation). In **Explore**: *per-knob impact*
(which protocols flip, and which way, when a knob moves to its neighbouring values), *what would it take* (the smallest
knob change(s) that serve X, including changes that break the chip check, which are labelled and come with the fix that
restores it), sweeps of any metric, and a Pareto scatter of area vs served count.

**How much to trust it (Trust).** Calibration (model vs measured, with `off` rows as findings, not tuning targets), the
parameters we are guessing (with status and sources, editable), the ISA effect rows, and the full "Not modelled" list.

## What each area owns (single-home rule)

Every concept is shown in exactly one place; elsewhere there is a link, not a copy (`tests/test_v2_no_dup.py` enforces it).

| area | owns |
|---|---|
| Configure (rail + chip card) | knobs, presets, points, live diagram (no area figures), the four/five chip-check verdicts and margins |
| Capability | per-protocol verdict, binding limit, margin; the served/marginal/impossible tally; scenario table (need and margin), reference mixes per point, pairwise compatibility and the max-concurrent count; how to read it |
| Timing | cross-point strip (reflex/response margin and binding timing limit per point), reflex/response segments, edge placement, receive sampling, the core clock path D against T (no margin number: that is the chip card) |
| Area & pins | the area waterfall and its total/usable, per-block lines, pin allocation |
| Throughput | host link MB/s, decision loop, IMEM load time, demand vs link, FIFO fill |
| Explore | per-knob impact, what-would-it-take, sweeps, Pareto (its base counts are context for the flip lists) |
| Trust | calibration rows, parameter statuses, ISA effect rows, the "Not modelled" list |

**Shared focus and side panel.** The focus bar holds the protocol and the point (live or a pinned one); every tab shows
that focus, knobs always edit the live point. Anything underlined or outlined is clickable (mouse or Enter/Space): it opens
the **derivation side panel**: the value, its formula, every term down to the parameter with `method`, `status` and source
file. Esc closes it. The panel is the only place derivations are drawn.

## The chip check vs protocol limits

Some limits are properties of the chip (core clock vs fmax, die area, board clock, pin budget, ISA opcode room); they were
repeated in every protocol cell in v1, so every row turned `impossible` when the die overflowed. v2 judges them **once per
point** in the chip card (and a chip badge in each Capability column header), and the protocol cells hold only
protocol-specific limits (TX edge, reflex, program size, host stream, per-protocol pin need ...). A cell never changes
because of the chip check; read "this verdict assumes the chip can be built" when its column is hatched.

## ISA levers are estimates

The ISA knob group (IRQ restored, XOR/CRC step, instruction width 16/24/32, more conditionals, wider MOV sources) changes
instructions per unit, program words, area and fmax per protocol. Measured counts exist only for the current ISA, so every
effect row in `inputs.json["isa"]` is an **estimate** (`judgement`, with a rationale and the closest measured neighbour).
The evidence rule: in *what would it take*, a candidate is labelled `estimate` (and ranked after the others) when it introduces a judgement-status input that the base point's blocking limits did not already use, or when any ISA effect row is used in its cell; the label lists which inputs are judged. Candidates backed only by measured/derived/spec inputs, or by judged inputs the base point already relied on, are not penalised. Trust lists every effect row; the chip check item "ISA encoding" shows opcode room. An ISA lever at margin 0 (e.g.
IRQ alone) means exactly full, not failing.

## Run

All commands from the repo root (they use `.venv`).

| command | does |
|---|---|
| `make ui-archsim` | build if stale, serve and open `archsim.html` |
| `python3 tools/archsim/build_html.py` | rebuild `archsim.html` from `data/inputs.json` and `ui/` |
| `pytest tools/archsim/tests` | the JS model through QuickJS, every view's render function, the page, this README |

`inputs.json` and `archsim.html` are committed.

Layout: `model/archsim-model.js` (the model; pure functions, including `ArchSim.mix`, `ArchSim.pairs`, `ArchSim.maxConcurrent`), `ui/archsim-ui.js` (core: layout, registry, side panel,
helpers; its header is the contract for view authors), `ui/view-*.js` + `ui/css/*.css` (one view each), `extract/` (data
extractors), `tests/`. Views are pure render functions (state in, HTML out) so they run headless under QuickJS.

## Method and status vocabulary

Every value carries a `method` (how obtained) and a `status` (how far to trust it). The definitions below are
transcribed from PLAN 3.3 into `inputs.json` (`vocab`); `tests/test_readme.py` fails if a term in the data is missing here.

| method | meaning |
|---|---|
| `spec` | normative repo document |
| `iss` | golden ISS measurement |
| `rtl` | RTL simulation measurement |
| `formula` | closed form in this model |
| `python_model` | a repo Python model |
| `synth` | Yosys synthesis probe |
| `flow` | full LibreLane flow |
| `spice` | SPICE |
| `literature` | external published value |
| `judgement` | our estimate |
| `derived` | computed by archsim from other values |

| status | meaning |
|---|---|
| `measured` | read from a committed measurement of this design or block |
| `derived` | arithmetic on measured or spec values |
| `estimate` | judgement anchored on measured neighbours |
| `judgement` | no measurement behind it |
| `unverified` | literature value not checked against the primary source, or for another process (sky130) |
| `disputed` | two repo sources disagree (both values are listed in `assumptions`) |

Calibration classes (PLAN 3.5): `match` (|err| <= 1 %), `close` (<= 10 %), `off` (> 10 %), `definitional` (not comparable, not scored),
`not_modelled` (measured only), `identity` (used to fit a parameter, agreement by construction), `brackets` (the measured value lies between the
model's bounding cases), `info` (a context item inside a point, not scored).

## Uncertainty (vertex intervals)

Every uncertain parameter has `lo / nom / hi`; these are bounds or judgement ranges, **not confidence intervals**. For each limit the model probes each
parameter it uses one at a time to find which end lowers the margin, then evaluates the margin at the all-pessimistic and all-optimistic vertices.
Verdict: `served` if the pessimistic margin is >= 0, `impossible` if the optimistic margin is < 0, otherwise `marginal`. This is exact because every
limit is monotone in each parameter (a test checks it), deterministic, and instant in the browser. The `cycles` and `rate_ceiling` limits have no
uncertain inputs, so a miss there is always `impossible`. Details: PLAN 3.2.


## Known limits

- **Not modelled** (the list Trust shows in full and the side panel shows where relevant, from PLAN 3.6): protocol-specific state machines beyond words and cycles per unit; open-drain rise times; FIFO joining; engine-to-engine coupling on shared pins; pad receiver thresholds; IR drop; hold-buffer area cliffs (captured only in the P&R factor band); the I2C-target host-feed limit (K6); armed triggers (C2); CRC computation cost in cycles; context-switching engines (stack D is area-only); silicon pad delay on IHP (the largest single unknown); host MCU CPU time per stream, host-port arbitration latency between engines, engines needing different core clocks and timing interference between engines on shared pins (all four: the Scenario check counts engines, pins, memory and link only).
- Bands are ranges, not distributions; "marginal" means the verdict depends on where in the band reality sits.
- The model contradicts several numbers in the project briefs on purpose (PLAN Part 0): those are findings, shown with their derivations.
- Tested headless only: the model, the render functions (HTML/SVG strings) and the page's structure run under QuickJS in pytest. Nothing executes the page in a real browser in CI, so layout, theme (light/dark), hover tooltips, click handlers, the URL hash and `localStorage` are checked by eye.
- The ISA levers are estimates (see above) until each effect is measured on a real program; the evidence rule keeps them behind measured options.
- The pin map is illustrative: the model counts pins, it does not route them.
