# fw/jtag — MVP JTAG firmware (task B1-FW-jtag)

The chip is the **controller**: it generates TCK, drives TMS and TDI, and samples TDO. The
far end is a TAP (`verif.vip/jtag.JtagTarget` on the bench), and `verif/vip/jtag.JtagChecker`
decodes the wire and checks all 15 of its rules. Zero violations, in every configuration
`test_jtag.py` runs.

| | |
|---|---|
| Chip role | TAP **controller** (TAP master / probe side) |
| Program | `program.pio`, **17 of 32** IMEM words, one engine, no host IRQ |
| Rate on the ISS at 50 MHz | **5.56 MHz TCK** (9 core cycles per TCK cycle at CLKDIV 1.0) |
| Scans supported | reset, IR scan (2..32 bits), DR scan (1..32 bits), idle clocks |
| Peak FIFO depth | 1 of 4 (the host queues each scan's words before starting it) |
| Acceptance | `make fw PROTO=jtag TARGET=iss` — 11 tests |

## Files

| File | What it is |
|---|---|
| `program.pio` | the whole program: 17 words, `.side_set 1` on TCK, three loops and a park |
| `driver.py` | the host side: register words, the TAP walks, the TCK rate the program implies |
| `test_jtag.py` | the bench and the 11 cases, judged by `verif/vip/jtag.py` |
| `measure.py` | `python -m fw.jtag.measure` — the numbers above, appended to `metrics/fw.csv` |

`program.pio`'s header comment is the design document: which pins, why TCK idles high, why
one TCK cycle is nine core cycles, and why the shift phase counts X while the framing phase
counts Y. `driver.py`'s module docstring covers the other half — why the driver exists at
all and why every scan is padded to a whole number of TX words.

## How to run it

```sh
make fw PROTO=jtag TARGET=iss              # the acceptance command
.venv/bin/python -m fw.jtag.measure        # the numbers in metrics/fw.csv
.venv/bin/python -m fw.jtag.measure --dry-run   # without touching the CSV
```

`FW_TARGET=rtl` runs the same bench against the RTL chip under cocotb once that exists
(`fw/rtl_harness.py`); the tests skip themselves for any other value.

Measured output: rows in `metrics/fw.csv`.

## Why the chip is the controller and not a TAP

`verif/vip/jtag.py` splits JTAG into `JtagMaster` (a bit pump: it is handed TCK cycles with
a TMS and a TDI level and does nothing else) and `JtagTarget` (the TAP: the 16-state
machine, the instruction register, the data registers). A real link splits the same way: the
probe shifts bits and OpenOCD decides which bits make a scan. So the PIO takes the master
role, which is the part a state machine can hold in 32 words, and `driver.py` takes the
driver role.

Building the TAP *inside* the chip does not fit, and the reasons are worth recording because
they are limits of the MVP, not of this program:

- **The state machine needs a jump table.** The 16 states are only reachable through
  conditional branches, and the MVP has no way to read IMEM (the host can write it and
  cannot read it, `docs/mvp-isa.md `### Engine state``), so a computed `mov pc, y` has
  nothing to compute *from*. Sixteen states of two or three instructions each is 32 words
  before the instruction register, the capture pattern and the shift logic exist.
- **A scan result has nowhere to go.** There is no IRQ (the MVP removes it, difference 3)
  and no `SMx_ADDR` write, so a 32-bit IDCODE read would have to be handed over in the RX
  FIFO — which works, but only after the whole scan has run, and the driver still has to
  know when. Here the *host* decides when a scan is finished, because it queued it.
- **The pin windows are not enough.** A TAP has to hold a shift register and a parallel
  register per data register, and it has to load them from the pin vector. The MVP gives one
  IN window and one OUT window, which is what the bit pump needs and not what a TAP needs.

## The timing, and where the floor is

One TCK cycle is `LOW_HOLD + HIGH_HOLD + 4` SM cycles: `out pins, 2` (1 + LOW_HOLD),
`nop [HIGH_HOLD]`, `in pins, 1`, `jmp x--`. `docs/mvp-isa.md `## Timing`` puts a floor
under both delays, and `measure.py` sweeps them to show the floors are the checker's, not
this program's:

- **HIGH_HOLD = 3** is T_IN = 3. The `in pins, 1` has to execute exactly three cycles after
  the rising-edge write, which is where the input pipeline puts the sample *on* the edge —
  the same tick the VIP's own `JtagMaster` samples on. HIGH_HOLD = 2 reaches 6.25 MHz and
  the checker accepts it, because it samples one tick *before* the edge with no TDO margin,
  and the checker cannot see a chip's sampling point. That is a different program.
- **LOW_HOLD = 2** is the TDO floor: `verif/vip/core.py` resolves drives at the end of a
  tick, so the target's next TDO bit reaches the wire two ticks after the falling edge, and
  the ijet profile's Ttdo (15.6 ns = 1 tick at 20 ns) wants one tick of validity before the
  rising edge. LOW_HOLD = 1 makes the change land on the very tick the edge is measured
  against, and `JtagChecker` reports `TDO.setup` — that is
  `test_a_tck_phase_that_is_too_short_is_caught`, and the sweep in the log shows it for every
  combination below the floor.

9 core cycles at 50 MHz is 5.56 MHz, against the 10 MHz the ijet profile caps TCK at. The
sweep also finds that LOW_HOLD = 0 breaks `IR.capture` and `IDCODE.lsb` as well: with no low
phase at all the TAP sees the wrong state and the scan decodes as garbage.

## What the tests cover

| Test | What it pins down |
|---|---|
| `test_idcode_at_the_fastest_divider` | reset + IDCODE at CLKDIV 1.0, the checker's two decoded scans, 9-tick period, FDEBUG clean, no contention |
| `test_a_slower_divider_scales_the_wire_exactly` | CLKDIV 4: the period scales by exactly 4, and the timing is at the floor in both |
| `test_another_tap_on_another_pin_map` | 4-bit IR, 16-bit user register, no IDCODE, pins 2..5: the program knows none of it |
| `test_bypass_and_a_user_register_on_the_built_in_tap` | the 1-bit scan (X = 0 is one clock, not none) and an 8-bit write/read |
| `test_idle_clocks_and_a_second_reset` | six IDCODE reads and two resets: TMS high for five clocks works from any state |
| `test_the_tms_sequences_are_the_state_table_s_own` | every TMS level replayed through `jtag.next_state`, and the shift lengths |
| `test_the_driver_packs_bits_the_way_the_wire_does` | LSB-first bit order against the VIP's independent copy; the pair packing |
| `test_a_tck_phase_that_is_too_short_is_caught` | the harness can fail: `TDO.setup` at LOW_HOLD 1, clean again at 3 |
| `test_the_program_fits_the_instruction_memory` | 17 words, the wrap window, the side-set shape, the loop arithmetic |
| `test_the_driver_addresses_match_the_register_map_and_the_iss` | no renumbering |
| `test_the_pin_map_has_to_match_the_windows_the_program_writes` | the pin map is refused, not mis-wired |

## Things a reader should know

- **TCK idles high here, which a real adapter does not.** `JtagChecker` measures TDI.setup
  and TMS.setup from the last falling edge, so on this bus a data change is only legal in
  the tick of a falling edge — which means the parked state has to leave TCK high for the
  next scan's first pair to fall on. The chip also starts with every pin an input on a
  pulled-up bus, so turning the directions on is itself a data change; `init_pin` writes
  the levels before the directions to make that invisible. See program.pio, "WHY TCK IDLES
  HIGH".
- **Every scan is padded to a whole number of TX words.** The OSR is refilled by autopull
  exactly when it empties, so a scan that ends mid-word leaves bits in it that belong to the
  next scan, and the host cannot see them. The padding is real TCK with TMS = 0 in
  Run-Test/Idle, which is what a probe does between scans anyway.
- **A scan with nothing to shift still clocks once.** A PIO loop cannot be told "zero
  times" (`jmp x--` tests before it decrements, so a loaded 0 is one clock), so a reset
  clocks once through the shift phase and pushes the RX word for it. The driver reads that
  word and throws it away.
- **A 32-bit DR scan has to be one Shift-DR phase.** Pausing in the middle of a long scan
  is what SVF drivers do, and the checker accepts the pause as a pause — but it ends and
  checks *every* Shift-DR phase against the register's full length, so two halves read as
  two short scans. This is why the shift phase counts X: `set x, 32` does not exist, and
  X = 31 is 32 clocks.
- **The chip emulates a controller, so the emulator's "device" side of JTAG is not covered
  here.** A chip that had to *answer* a probe would need the TAP above; that is the limit
  this README's first table is about, and it is a limit of 32 words and no IRQ.