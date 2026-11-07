# `fw/i2c_target` — I2C target (device mode) for the MVP chip

The MVP answering as a **24C02-style EEPROM**: 7-bit address, 256 bytes in 8-byte pages, a
word-address pointer, sequential read, ACK and NAK. This is the mirror image of `fw/i2c`,
which is a controller: there the host starts every byte, here the *bus* does, and
`verif.vip.i2c.I2cController` is the master while `I2cTimingChecker` judges the wire.

    FW_TARGET=iss .venv/bin/python -m pytest -q fw/i2c_target      # 27 passed, 3 xfailed
    .venv/bin/python -m fw.i2c_target.measure --dry-run           # the numbers below
    .venv/bin/python -m fw.i2c_target.start_hold                  # the master's START hold
    .venv/bin/python -m fw.i2c_target.hold_owner                 # who owns the tightest one
    python -m fw.targets                                         # which targets work here

Every line above is read-only: `measure --dry-run` writes nothing, `hold_owner` and
`start_hold` have no side effects at all, and neither touches `metrics/fw.csv`. Drop the
`--dry-run` and `measure` also **appends five rows** (one per
clean rate) to the shared, tracked `metrics/fw.csv` — they are already committed, so a plain
run duplicates them, which is why the reproduction above and the log table below are the
dry-run.

| | |
|---|---|
| Program size | **29 of 32 IMEM words** (`fw/i2c_target/program.pio`, one engine) |
| Fastest SCL that serves every transaction | **600 kHz** nominal, **595.2 kbit/s** measured on the wire |
| Tightest margin at that rate | **tHD;DAT, 9 core cycles of room**, measured on the **PIO's own** SDA moves (9 ticks after the master's SCL fall, 0 required). The wire-wide minimum at that rate is 3 and belongs to the *master*: see `### Margins` |
| Clock stretching | **not used**, and cannot be — see below |
| Engine words per byte | 2 (a DATA word and an ACK word), one per 9 bit times |

Measured on the golden ISS at 20 ns/tick.

## What is where

| File | |
|---|---|
| `program.pio` | the whole wire engine: START, bit times, the ACK drive, STOP detection |
| `driver.py` | the EEPROM behind it: memory, pointer, page rollover, address, ACK/NAK |
| `test_i2c_target.py` | the bench, the rate sweep, and a passive margin probe that attributes every sample to whoever moved SDA |
| `measure.py` | the sweep, the margins (both minima per rule), and the rows in `metrics/fw.csv` |
| `rate_bisect.py` | any rate range in arbitrary steps, with the START window measured off the wire |
| `start_hold.py` | the master's own START hold (tHD;STA, SCL fall to SDA rise) off the wire at every fast-mode rate, against the spec's 600 ns |
| `hold_owner.py` | the attribution cross-check: the probe run three ways, plus the tick decomposition |
| `one_mhz_owner.py` | whose move the 1 MHz `tSU;DAT` violation is, with and without a slave |
| `__init__.py` | `assemble_program()` and the re-exports |

Every number below can be regenerated with the commands at the top: the rate sweep and margins
with `measure.py --dry-run`, the master's START hold with `start_hold.py`, the attribution
cross-check with `hold_owner.py`, and the 1 MHz attribution with `one_mhz_owner.py`.

## How the two halves fit together

The program owns the wire and the host owns the protocol, and they talk through the FIFOs
in **two 32-bit words per byte**:

    DATA word, pulled before the byte      bits 31:24  the eight bit times, as SDA
                                           directions (1 pulls low, 0 releases)
    ACK word,  pulled after the eight       bit 31      the ninth bit time, the ACK clock
                                           bits 30:0   not read

The ACK word is pulled *after* the data bits on purpose. That is the whole reason a target
can NAK an address that is not its own: the host has the byte in its hands (PUSH_THRESH 8
hands it over at the eighth sample, a whole bit time before the ACK clock) and only then
can it answer. The cost is that the program parks in `pull block` while the host answers,
which lengthens the master's tLOW — legal, and priced in the rate table below.

Ordering is the part that is easy to get wrong, and the rules are in `driver.py`'s `_queue`:
nothing is queued before its answer exists (so a slow host is a stall, not a wrong byte), a
byte the slave *sends* contributes its DATA and ACK words together (its answer is known
before it starts — the ACK clock is the master's), and an end-of-bus marker empties the
queue.

## The one rule: a target must not drive SCL

Every SCL edge on the bus has to be the master's. A controller can pull SCL low the moment
it has sampled and stretch as it likes; a slave may not, because the high period it
shortens is the master's **tHIGH**, and `I2cTimingChecker` measures the wire, not the
intent. So `program.pio` has **no `.side_set` at all** — SCL is only ever read, with
`wait 0/1 gpio SCL_PIN` — and `check_program` refuses any program that has one. That is
also why **this firmware cannot stretch the clock**: a slave may pull SCL low only *after*
the master has released it, and a program with no side-set and no timer has nowhere to
distinguish that moment. The `MVP` says clock stretching is the target's to use
(`docs/mvp-isa.md` `### Open drain`), and for I2C the MVP takes the opposite trade: it
answers inside the master's bit time instead of holding the bit, and the measurement below
is what that costs.

## Rates

Every row is the three transactions (`page_write`, `sequential_read`, `byte_write`) run at
that SCL rate, with the controller *and* the checker configured for it. A rate is clean when
all three complete correctly, `I2cTimingChecker` finds **no violation at all**, and the
engine's `FDEBUG` is zero — the last one because a rate can satisfy the checker and still
have the engine stall on an empty FIFO halfway through. The failing rows carry
`pytest.mark.xfail(strict=True)` **as a mark on the parameter** (`SWEEP_PARAMS` in
`test_i2c_target.py`), with the measured reason from `XFAIL` as the mark's reason, and
nothing in the test body turns a clean run into an xfail. So a rate that starts passing is
`XPASS(strict)` and fails the run instead of quietly improving (the 50 kHz row was once
recorded here as failing, and the strict mark is what caught it).

| SCL rate | mode | CLKDIV | poll | on the wire | result |
|---|---|---|---|---|---|
| 50 kHz | standard | 84 | 2 | 50.0 kbit/s | clean (0 violations; tSU;DAT 183 vs 13 → 170 cycles) |
| 100 kHz | standard | 43 | 2 | 100.0 kbit/s | clean |
| 200 kHz | fast | 16 | 3 | 200.0 kbit/s | clean — failed before the START-window bound described below |
| 300 kHz | fast | 15 | 2 | 299.4 kbit/s | clean |
| 400 kHz | fast | 12 | 2 | 400.0 kbit/s | clean |
| 600 kHz | fast (Fm+ minimums) | 5 | 3 | 595.2 kbit/s | clean, and the fastest |
| 800 kHz | fast (Fm+ minimums) | 3 | 3 | 793.7 kbit/s | **fails**: `FDEBUG 0x01000000`, the engine stalls on an empty TX FIFO |
| 1 MHz | fast (Fm+ minimums) | 2 | 4 | 1000.0 kbit/s | **fails**: `tSU;DAT 3 ticks < 5`, and the move is the **master's** — see `### Rates` |

`I2C_MODES` carries standard and fast only, so the three rates above 400 kHz are driven
with the fast-mode-plus half-periods (400 ns, 20 ticks) as an explicit override
(`measure.FMP`) and with `freq` set to the rate, which is how `verif.vip.i2c` is meant to
be pointed off-nominal.

**Why the firmware stops where it does.** Three budgets. The first is what the tHD;DAT column
of the margin table measures, and it is what sets the *tightest point*; the other two decide
where the firmware stops:

1. *The first bit of a byte has to be on SDA before the master releases SCL.* From the
   master's SCL fall that is six SM cycles (`clkdiv_max`), so the divider is bounded by the
   master's **low half-period**, not by the bit time.
2. *The bit the program samples has to be sampled inside the master's high period.* The
   sample is three core cycles (`T_IN`) plus one divider period after the rise, so the
   divider is bounded above by the high period too — and the margin left over is what the
   table calls tSU;DAT.
3. *A START has to be seen at all.* `idle` recognises a START by **polling**
   `wait 0 gpio SDA_PIN`, so the engine looks at SDA once per `CLKDIV + 1` core cycles; a
   START whose SDA-low window is narrower than that falls between two looks and is missed
   outright. The window is `I2cController.first_hold + 3` ticks — **17 ticks in fast mode**
   against standard mode's 204 — so in fast mode this is the binding bound **from just above
   100 kHz up to about 290 kHz**: the lower edge is the standard/fast boundary, where the
   window falls from 204 ticks to 17, and the upper one is where the master's low half-period
   becomes the tighter bound again (the divider holds at 16 from 150 kHz through 290 kHz and
   drops to 15 at 295 kHz; `rate_bisect.py` reproduces this). That is why
   `clkdiv_max` takes the minimum of (1) and (3) and 200 kHz now
   runs at CLKDIV 16 rather than the 22 the low half-period alone would allow. There is no
   program-side alternative: a polled `wait` cannot see a pulse shorter than one
   instruction. The short version is that this budget was once missing, and it showed up as a
   rate that failed between two passing ones rather than as a rate limit.

At 800 kHz the *host* is the limit — the engine outruns the FIFO pipeline and stalls on an
empty TX FIFO, which `FDEBUG` records and both `measure.py` and the sweep test check.

The 1 MHz row is subtler than the two budgets above, and the attribution is what settles it.
Its symptom is `tSU;DAT 3 ticks < 5`, and that sample is **the master's**: the bus-wide
tightest tSU;DAT belongs to `i2c_ctrl`, which holds SDA low for
`first_hold = _hold_point(t_hd_sta) = 21` ticks into a 25-tick low period and then releases
it three ticks before its own rise, where Fm+ asks for five. A bus with **no slave on it at
all** shows the same violation at the same tick
(`one_mhz_owner.py`), and the PIO's own tSU;DAT at that rate
is 14 against 5. So the row legitimately fails — the checker finds a violation either way —
but no firmware could pass it with this master at 1 MHz, which is a statement about
`I2cController`'s first-bit placement and not about this design.

**Which of the two 1 MHz figures is which.** The **14** above is the sweep's own bench
(`measure.py`), and it is the number quoted here, in `XFAIL` and in the test's reason string.
`one_mhz_owner.py` prints the same 14 for the same rate because both benches now read FDEBUG
**once per transaction**. Before this round they did not: `measure.py` called `fdebug()`
twice per transaction — once in the `if` and once again inside the f-string — and a host read
is a host read (`reg_read` steps the bus until the read lands), so the second one shifted the
master's phase for the next transaction by a tick and the sweep reported the PIO's own tSU;DAT
as 15, so the two numbers for one thing used to disagree. The row's
verdict does not depend on which one you read — 14 and 15 both clear the required 5, and the
row fails on the master's own 3 — but they are the same number now.

### Margins, measured off the wire, and whose margin they are

`test_i2c_target.py`'s `MarginProbe` is a passive device that records the **smallest**
interval each rule ever got, because `I2cTimingChecker` only records violations and a clean
run says nothing about how close it came. Each rule is taken the way the checker takes it:
**tHD;DAT** is armed on an SCL falling edge and sampled at the next SDA move — the
"SDA valid after SCL falling" — and **tSU;DAT** is the last SDA move before an SCL rise.

**Every sample is attributed to whoever moved SDA**, through `verif.vip.core.Bus.tap` — the
bus's own read-only observer, which reports what each device drove on the tick that was just
resolved and is called after every device has ticked, so it cannot change what any device
sees. So there are **two minima per rule**, and they are different questions:

* the **wire-wide** one is what `I2cTimingChecker` would fail on, and on this bus it belongs
  to the **master**: `I2cController._hold_point` is `max(2, min(max(1, t_hd_dat) + 1,
  low − 1))`, i.e. 16 ticks in standard mode and 2 in fast, plus one tick of bus resolution
  — **17 and 3**, constant from 50 kHz to 1 MHz while the divider goes 84 → 2. A bus with
  *no slave on it at all* reaches the same two numbers.
* the **PIO's own** one is the minimum over the samples this firmware moved the line in, and
  that is the margin that matters.

`tHD;DAT` is the tightest of the two rules on the PIO's own minima at every clean rate, so
it is the answer to "the margin in core cycles at the tightest point":

| SCL rate | tHD;DAT, PIO's own | required | **room** | tHD;DAT, wire-wide | whose | tSU;DAT, PIO's own | required | room |
|---|---|---|---|---|---|---|---|---|
| 50 kHz | **94** | 15 | **79 cycles** | 17 | master | 183 | 13 | 170 cycles |
| 100 kHz | **56** | 15 | **41 cycles** | 17 | master | 92 | 13 | 79 cycles |
| 200 kHz | **21** | 0 | **21 cycles** | 3 | master | 75 | 5 | 70 cycles |
| 300 kHz | **20** | 0 | **20 cycles** | 3 | master | 39 | 5 | 34 cycles |
| 400 kHz | **19** | 0 | **19 cycles** | 3 | master | 27 | 5 | 22 cycles |
| 600 kHz | **9** | **0** | **9 cycles** | 3 | master | 19 | 5 | 14 cycles |

The setup rule has room either way — the PIO's own minimum is 92 at 100 kHz where the
wire-wide one is 65, because the master's own bit placement is what sets the tightest
sample — so tHD;DAT is the tightest of the two rules on the PIO's own minima at every clean
rate.

So at the fastest clean rate the design has **9 core cycles** of hold room after the master's
SCL fall, not the 3 the wire-wide column shows and not the 14 the setup rule has.

**Why 9 and not 3.** It is `T_IN` **plus one divider period**, not `T_IN` alone: an engine
sees the fall three core cycles late (`iss/pio_iss.py`'s `T_IN`), and `out pindirs` is the
*next instruction the divider releases* — the ISS calls `sm_step` only when the divider
asserts `en` — so the bit reaches SDA one whole divider period (CLKDIV + 1) later still. At
CLKDIV 5 that is 3 + 6 = **9** with no phase left over, and `measure.py` prints the
decomposition at every rate: 94 = 3 + 85 + 6 at 50 kHz, 56 = 3 + 44 + 9 at 100 kHz, 21 = 3 +
17 + 1 at 200 kHz, 20 = 3 + 16 + 1 at 300 kHz, 19 = 3 + 13 + 3 at 400 kHz. The last term is
where the master's fall lands in the divider's phase, which only the minimum over the run can
shrink. The 200 kHz row is a consequence of the START-window bound: that rate now runs CLKDIV 16
rather than 22 (see `### Rates`), so its divider period is 17 ticks and not 23, and its hold
of 21 cycles is a little smaller than the 27 it would otherwise have had — of which fast mode
requires zero.

That is why `program.pio` keeps `LOW_HOLD` at 0 — and why the design is **not** out of hold
to spend. Fast mode and Fm+ require *zero* hold, so the 9 cycles are real; `LOW_HOLD` counts
SM cycles, so a single extra one would cost a whole divider period (6 core cycles at CLKDIV
5) and leave 3.

`SDA→fall` (how long SDA had *already* been stable when SCL fell) is reported the same way:
2 cycles wire-wide at every rate, which is the master's own bit placement, and 61–666 cycles
for the PIO across the five clean rates, whose `out pindirs` is nowhere near a fall. It is the
**only** one of the three that carries an owner tally, because it is the only one of the three
that hangs off an SDA move.

`tLOW` and `tHIGH` hang off **SCL** edges, and a PIO may not make one at all, so they have no
owner and no `own` column on any bench — the two rows are the master's own SCL periods,
measured between the two edges that bound each of them (fall to rise, rise to fall):

| SCL rate | tLOW, wire-wide | required | room | tHIGH, wire-wide | required | room |
|---|---|---|---|---|---|---|
| 50 kHz | 517 | 235 | 282 cycles | 474 | 200 | 274 cycles |
| 100 kHz | 267 | 235 | 32 cycles | 233 | 200 | 33 cycles |
| 300 kHz | 101 | 65 | 36 cycles | 64 | 30 | 34 cycles |
| 400 kHz | 80 | 65 | 15 cycles | 45 | 30 | 15 cycles |
| 600 kHz | 42 | 20 | 22 cycles | 42 | 20 | 22 cycles |

Every margin is positive, and the smallest of the ten is 15 cycles — on **both** rules at
400 kHz — which is why `I2cTimingChecker` finds **no violation at all** at any of these rates,
and why nothing in these two rows is a firmware budget. (An earlier version of this probe
filed the high period under `tLOW` and printed it against `t_low`; the four negative margins it
reported were the high period measured against the low minimum. `verif/vip/i2c.py`'s
`_fall`/`_rise` are the reference, and the log's rows now match them.) The probe judges only
tHD;DAT and tSU;DAT, the two the firmware's budget decides. The one guard these two rules have is
`test_each_scl_period_is_measured_under_the_rule_that_bounds_it`, which ticks the probe by
hand on a clock whose low and high periods are 37 and 91 ticks, so the two names cannot be
told apart by symmetry;
That test fails with the
two labels swapped back, which is how this mix-up would show up again.

`hold_owner.py` is the cross-check that the attribution
is right: the same probe run three ways — this firmware as the slave, the VIP's own
`I2cEepromTarget` in its place (a different drive path again), and the controller alone on a
bus with **no slave on it at all** — plus the tick-by-tick decomposition of every clean
rate. `python -m fw.i2c_target.hold_owner` reproduces it.

Two things about the numbers themselves. Standard mode requires 300 ns = 15 ticks of hold,
which is `I2C_MODES`'s `t_hd_dat` used as a *minimum* hold where the spec quotes tHD;DAT as a
maximum with a zero minimum (`verif/vip/` is treated as read-only here), so that row is the
VIP's rule rather than a datasheet's. And the PIO's own minimum is **one tick lower** here
than in a run without the sweep's per-transaction FDEBUG poll (95 / 57 / 20 / 20 / 9): the
poll is a host read, so it steps the bus and shifts the master's phase for the next
transaction by a tick. The table quotes the sweep's bench, because that is where every other
number in this README comes from; `hold_owner.py` prints both. That is the *only* difference
between the two benches — the sweep polls FDEBUG once per transaction and so does
`hold_owner.py`'s "sweep bench" run, which is what the double `fdebug()` in `measure.py`
used to break (see `### Rates` above).
## What it does not do, measured

**A repeated START shifts the byte after it by one bit.** This is the honest limit of the
design and it is worth stating plainly. A repeated START is SDA falling while SCL is high,
and between the ACK clock and the address byte that follows it the master releases SCL
exactly as it releases SCL for a bit time: one rise, one fall, SDA high. Nothing on the
wire distinguishes that from a data bit except *when* the fall comes, and telling the two
apart needs a timer whose value depends on the bus rate — which is precisely the assumption
this program refuses to make anywhere else.

What follows on the bus is therefore correct but one bit out, so **a random read (write the
pointer, repeated START, read) does not work**, and the firmware's own EEPROM state machine
is not told a new address byte is coming: it carries on writing data bytes. Every other
24C02 access does work, because every other one starts with a START from an idle bus, where
`idle` sees SDA fall and the alignment is exact:

| access | works | note |
|---|---|---|
| byte write | yes | |
| page write across a page boundary | yes | the host's pointer rolls over inside the page |
| current-address read | yes | a STOP, then a fresh START |
| sequential read | yes | the pointer runs on without a rollover |
| NAK of a foreign address | yes | and the VIP's own `I2cEepromTarget` NAKs the same one |
| **random read (repeated START)** | **no** | the byte after it is one bit out |

The host also cannot see a repeated START for the same reason, so its state machine resets
on the end-of-bus marker the program pushes (`Y = 0x8000001f`, bit 31 set, which no byte
word can be) rather than on a repeated START. This was **not** what the 200 kHz row's NACK
was, though it was recorded here as if it were: the address byte after a repeated START is
one bit out, and so was the address byte of `SWEEP`'s third transaction — but for a different
reason, and the two cannot be told apart from
the `nack=True` alone. What is true is that both come from the same missing signal, this
time a START too narrow to be polled for.

**A START narrower than one engine look is missed entirely**, and no divider value can
prevent it — only bound it. Measured at 200 kHz, where the master's START window is 17 ticks
and the program now runs a 17-tick look; before the fix the divider was 22, i.e. a 23-tick
look, and **15 of the 41 rates in 150–350 kHz failed on it**
(compare `rate_bisect.py` on the current and the earlier divider rule). A slave with an edge-sensitive START detect
would catch every one of them; this program has a polled `wait` and no other option, which is
the honest limit.

**And 17 ticks is half what a compliant fast-mode master would give.** Fast mode specifies
600 ns of `tHD;STA` — hold SDA low after the SCL fall of a START — but
`verif/vip/i2c.py:72` carries **260 ns** in its fast row, which is the *fast-mode-plus*
column of the same table
(`research/ideation/round2/research/sources/C12/ti-snls650-i2c-timing-minima.md:24`: "4 / 0.6 /
0.26 us" for standard / fast / fast-plus). So this model's master measures **300 ns** of hold
where fast mode asks 600 — measured off the wire at 200, 300 and 400 kHz, every START, by
`python -m fw.i2c_target.start_hold` — and the
START window is 17 ticks rather than the 34 a compliant master would give.

That is **why the sweep never caught the firmware out**: against a spec-compliant 200 kHz
master the pre-fix CLKDIV 22, a 23-cycle look, sits *inside* a 34-tick window and would have
caught every START. So the 200 kHz failure was triggered by the model's constant, and the
firmware's missing START-window budget, real as it was, was latent rather than exercised.
`verif/vip/` is treated as read-only here, so the constant is reported and not patched, and
`test_the_masters_start_hold_meets_the_fast_mode_minimum` guards it: a strict xfail that
**fails at 300 ns today** and would become an XPASS failure the day the model is corrected.
The three firmware-side guards are `test_the_start_window_is_longer_than_one_engine_look`,
`test_clkdiv_max_is_bounded_by_the_start_window_and_not_only_by_the_low_phase` and
`test_every_start_on_the_bus_is_served_by_the_program`; with the old `clkdiv_max` back, all
three bite and 100 kHz stays green. See `### Rates` for why the
sweep's guard is a mark and not a call.

**No clock stretching**, as above: the MVP's answer inside the bit time instead of holding
it. For the VIP's master that is invisible — stretching would only *lower* the rate — but a
real slave that needs to hold SDA for a slow host cannot.

**The host is in the loop.** One word per byte through a four-deep TX FIFO means the host
answers each byte inside the master's low half-period. At 800 kHz it cannot (that is the
`FDEBUG 0x01000000` row) — a target whose memory is on-chip would not have that limit, and
that is the whole argument for C1's bus-clocked lane
(`research/ideation/round2/research/C1.md`): the data has to be *there*, not in a host.

## What I did not verify

* **Clock stretching on real silicon.** The VIP's master would have to ask for it; the
  program has no side-set to do it with, so this is a design decision, not a measurement.
* ~~**The 200 kHz NACK is diagnosed, not explained.**~~ Resolved: it was a missed START, the
  divider being coarser than the master's START window. The repeated START and this share a
  missing signal, but they are not the same mechanism.
* **The non-monotonicity is measured but not fully explained.** The bisect says a rate fails
  exactly when `CLKDIV + 1 > 17`, and it says *which* transaction fails; it does not say why
  one rate with a 20-cycle look misses a START and the rate 5 kHz above it with a 21-cycle
  look does not. That is the divider's phase relative to the master's, and I did not model
  it. It does not matter for the fix — the bound makes every rate in the band clean — but it
  means the failure could not have been predicted from the rate alone, only found by
  bisecting.
* **`FW_TARGET=rtl` is not verified.** The RTL build on this machine fails before any of
  this firmware runs — `fw/rtl_harness.py` raises `no IMEM array in g_eng[e].u_eng` in
  `fw/sim_build/verilator-fw-*.log`, so every test on that target errors with "the simulator
  died". The same failures appear without
  the START-window change, so this is a property of the build here rather than of this
  firmware. The change is a divider value computed in Python and written to `SM_CLKDIV`,
  which both ISSs and the RTL read from the same register, so it is expected to hold — but
  that is an expectation and not a measurement, and the guard tests have not run on RTL.
* **Nothing was run at a non-20 ns tick**, and no pin pair other than the default SDA 0 /
  SCL 1 pair was simulated (the pin arithmetic is unit-tested, not simulated).
* **The 24C02 rules are the VIP's**, not a datasheet's: the page rollover, the pointer and
  the address decode are checked against `verif.vip.i2c.I2cEepromTarget` and agree
  (`test_the_firmware_agrees_with_the_vip_eeprom`), which is a self-consistency check
  between two models of the same part rather than against the part.
