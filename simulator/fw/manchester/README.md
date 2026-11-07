# `fw/manchester/` -- MVP Manchester transmitter firmware

MVP firmware for the Manchester (biphase) line code. The chip is the
**transmitter**; the judge is `verif/vip/manchester.py` -- its `ManchesterRx` for the data and
its passive `ManchesterTimingChecker` for the wire -- which is written from the
published specifications (IEEE 802.3, Atmel AN2358, Freescale AN3015).

## How to run it

```bash
make fw PROTO=manchester TARGET=iss      # the acceptance command: 94 tests
make fw PROTO=manchester TARGET=rtl      # the same checks on src/pio_chip.sv under cocotb
python -m fw.manchester.sweep            # the rate/timing characterisation table
```

`TARGET=rtl` swaps `iss.pio_iss.Chip` for a chip on the RTL and changes nothing else
(`fw/rtl_harness.py`). Every test goes through the shared slot queue:
`make fw PROTO=manchester TARGET=iss`.

## What is in here

| file | what it is |
|---|---|
| `program.pio` | two `.program`s, one per chip split, with the line code and every number cited |
| `driver.py` | assembles either program, derives the timing from the words, builds the registers, and refuses a bit rate the split cannot carry |
| `test_manchester.py` | 94 tests: twelve configurations on the ISS, plus the non-vacuity group |
| `sweep.py` | the rate ladder per program, and one row in `metrics/fw.csv` |
| `__init__.py` | re-exports the driver API |

## Results

Test run on the golden ISS:

```
$ make fw PROTO=manchester TARGET=iss
94 passed
```

The characterisation table from `python -m fw.manchester.sweep` reports:

| | `manchester` | `manchester_even` |
|---|---|---|
| IMEM words used | **11 of 32** | **11 of 32** |
| chips of a cell | 3 and 2 SM cycles | 2 and 2 SM cycles |
| SM cycles per bit | 5 | 4 |
| fastest at `CLKDIV = 1.0` | **10 Mbit/s** (AN1120a's 10BASE-T) | 12.5 Mbit/s |
| fastest the judge can certify | 10 Mbit/s | **12 Mbit/s** |
| rates measured clean | 10 and 5 Mbit/s | all 18 on the ladder, 12 Mbit/s .. 25 kbit/s |
| worst bit-grid error measured | 0.50 tick at 10 Mbit/s, 1.00 at 5 | 0.83 at 12 Mbit/s, **0.00** at 8.33 Mbit/s and every slower whole-number rate |

Both programs are one message per 32-bit TX word: eight data bits, two instructions each, plus
the sync preamble, the pull and the idle gap. Sixteen data bits would need 32 words before any
loop overhead, which does not fit the 32-word IMEM.

## The two programs, and why there are two

A Manchester cell is two half-bit level periods, and they should be *equal*: half a bit each.
Three instructions make a bit -- `mov pins, osr !` writes the complement of the bit and holds
it, `out pins, 1` writes the bit and shifts the next one in, and the loop's `jmp` holds the
second level while it runs -- so

```
first chip  = 1 + delay(`mov pins, osr !`)  SM cycles
second chip = 1 + delay(`out pins, 1`) + 1  SM cycles
```

`manchester` picks delays 2 and 0, so the chips are 3 and 2 SM cycles and the bit is 5. At
`CLKDIV = 1.0` that is 5 ticks of 20 ns, a 100 ns bit time, 10 Mbit/s: AN1120a's 10BASE-T rate
and the profile `verif/vip/manchester.py` defaults to. The price is a half tick of skew per
tick of `CLKDIV`: the bit-boundary transition lands 0.5 ticks from its place on the judge's
grid at 10 Mbit/s and a whole tick -- the entire `tJIT` allowance -- at 5 Mbit/s, which is the
slowest rate that program can carry. Below that the chips are outside the window a receiver can
classify and the wire stops being Manchester; `driver.split_is_legal` refuses those rates and
`test_a_rate_the_split_cannot_carry_is_refused_and_the_judge_says_why` shows the monitor
agreeing, by name (`tHALF` and `tMID-BIT`), when the refusal is overridden.

`manchester_even` picks delays 1 and 0: chips of 2 and 2 SM cycles, a bit of 4. The skew is
zero, so the wire sits on the judge's bit grid to the tick at *any* rate the divider can
express, and the sweep measures exactly 0.00 ticks of grid error from 8.33 Mbit/s down to
25 kbit/s. Its ceiling is 12.5 Mbit/s at `CLKDIV = 1.0`, which is precisely the rate at which
the judge's half-bit window [1.5, 3.5] ticks and bit window [4, 6] stop being disjoint and
`profile_timings` refuses to classify anything -- so 12 Mbit/s is the fastest it can be shown.

## What the firmware had to get right

* **The line code is two instructions, not one.** `mov pins, osr !` writes `~OSR` (a MOV never
  stalls and never shifts, so the bit is still there) and `out pins, 1` writes `OSR` bit 0 and
  shifts. There is no single instruction that does both: only OUT shifts the OSR, and only MOV
  can write a pin level inverted. The bit-boundary rule falls out of the chip list rather than
  being implemented -- a boundary transition appears exactly when two adjacent bits are equal,
  which is AN2358's "additional transitions are inserted between bits to satisfy the mid-bit
  transition rule".
* **The sync preamble is what puts a receiver on the right bit grid.** A line code has no
  start bit, so AN2358's rule ("always start messages with a '0' bit when the idle state is
  low. Or, send a 0 to 1 preamble") is the only phase on the wire. `program.pio` generates the
  two-bit pattern with two `set pins`, and the delay of the first one is one bit time, because
  the judge's grid is anchored on the preamble's first mid-bit transition and advances by
  exactly one bit. Getting that half bit one SM cycle wrong is invisible at 10 Mbit/s (the
  error is one tick and the allowance is one tick) and a `tMID-BIT` violation at any other
  rate; `test_a_broken_program_fails_the_check` breaks it on purpose.
* **`set x, 7` sits inside the preamble's last cell.** The `set pins, 1` and the counter load
  together are that cell's second half, one half bit of SM cycles, and the data loop's first
  chip has to start on the cell boundary. That is why the counter is between the preamble and
  the loop and not before the preamble.
* **`PINCTRL.SIDESET_COUNT` has to be 0.** These programs have no `.side_set`, so all five bits
  of bits 12:8 are delay -- but `SIDESET_COUNT` *resets* to 2, and with the reset
  `EXECCTRL.SIDE_EN = 0` that makes every instruction side-set level 0 into `SIDESET_BASE` and
  `SIDESET_BASE + 1`, and the side-set channel beats OUT/SET on the same pin
  (`docs/mvp-isa.md` `### Pin writes`). The line would sit low for ever. The driver takes the
  count from the assembler rather than writing a number.
* **The level register resets to 0 and the idle level is 1.** Forcing `set pindirs, 1` before
  `set pins, 1` would drive the line low for one core cycle -- a one-tick runt edge, shorter
  than the half bit the monitor allows, which is a `tHALF` violation on a line that is
  supposed to be idle. Both the driver (`arm()`) and the program (`start`) put the level first.
* **The convention is a host-side bit complement.** `manchester` emits IEEE 802.3 chips, whose
  sync preamble happens to drive the same two levels in the same two slots as G.E. Thomas's, so
  `convention="thomas"` is exactly "push the complement of each byte" and nothing in the program
  changes. The two conventions are inverses and a receiver using the wrong one reads every bit
  the other way round while reporting *no* timing error at all, which is the trap Freescale
  AN3015 walks into (`research/ref/protocols/manchester/README.md`).

## What the tests check, and what they cannot

`test_the_line_is_legal_and_the_payload_comes_back` is the main pass criterion: `chk.clean`, which
covers all eight of the monitor's rules, plus `rx.clean` and the bytes coming back. Around it:
the preamble on the wire (`tSYNC`), the idle gap (`tIDLE`, `tGAP`), the bit grid (`tJIT`), the
chip left clean (no `TXOVER`, `RXUNDER` or `RXSTALL`, only the TX pin driven, the other engine
untouched, no contention on the push-pull net), and the IMEM holding exactly the words
`asm/pioasm.py` produced.

Two honest limits, both asserted as tests rather than papered over:

* **A line code cannot report a bit-order error.** A host that reverses each byte's bits
  produces a perfectly legal Manchester waveform of a different message -- same cells, same
  transitions, same timing -- so the monitor is silent and only the receiver's bytes catch it
  (`test_a_host_that_reverses_its_bytes_gets_a_legal_wire_carrying_the_wrong_bits`).
* **No idle line means no boundary, and the monitor says nothing.** Two messages with no gap
  between them are one stream to it; the violation is that the boundary has gone, not an
  interval. The mutation test asserts the stream count, which is what notices.

## Not done here

* **The chip is a transmitter only.** A receiver has to tell a half-bit interval from a
  whole-bit one, which with no IRQ means counting in a loop against a pin that arrives three
  cycles late (`docs/mvp-isa.md` `### Timing`); 32 words is not enough to do that and still
  frame anything. That is a different program.
* **Nothing here runs against the RTL.** `FW_TARGET=rtl` is wired up and the test file skips
  unless it is set, but the main command is `TARGET=iss`; `verif/vip/cocotb_bridge.py`
  knows about the UART and SPI devices and has no Manchester device.
* **No waveform dump.** `verif/vip/manchester.py` documents `manchester_bus(trace=True)` and
  `bus.write_vcd(...)`; nothing in `fw/manchester/` does it.
* **The idle level is compiled in.** `IDLE_LEVEL = 1` and the two `set pins` words assume a
  link that idles high. AN2358 shows links that idle low too; those would need the two words
  swapped, and `program.pio` says so.