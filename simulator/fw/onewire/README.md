# fw/onewire — MVP 1-Wire (DS18B20) master firmware

The chip plays the **master** of the open-drain `dq` line; `verif/vip/onewire.py`'s
`Ds18b20Target` is the DS18B20. One `program.pio` is the whole master: a reset pulse with
the presence check, and one time-slot loop that writes a byte, reads a byte or does both
depending only on the 32-bit words the host pushes.

## Run it

```bash
make fw PROTO=onewire TARGET=iss      # the accept command: 14 tests, ~50 s
.venv/bin/python -m fw.onewire.measure        # the numbers below, into metrics/fw.csv
```

Both are simulations of the tick-based bus (`verif.vip.core.Bus` at 20 ns/tick, i.e. 50 MHz)
with the chip behind `iss.pio_iss.ChipDevice`. They are slow because 1-Wire is slow: one
time slot is 70 to 87 us, which is 3 500 to 4 350 ticks, and one "reset, read nine
scratchpad bytes" transaction is about 390 000 ticks.

## The files

| File | What it is |
|---|---|
| `program.pio` | 29 of 32 IMEM words. The header explains every pin window, every delay and how this differs from an RP2040 one-wire program. |
| `driver.py` | The register map side: `OneWireDriver`, the word packing, and `layout()` / `reset_layout()` / `check_program()`, which measure the assembled words and refuse a program whose timing moved. `retarget()` scales the presence pad to the clock divider. |
| `measure.py` | The row for `metrics/fw.csv`, plus the CLKDIV scan. |
| `test_onewire.py` | The bench: the chip on a bus with a DS18B20 and `OneWireTimingChecker`, in three configurations, judged by the checker alone. |

## Results

Measured on the golden ISS: `make fw PROTO=onewire TARGET=iss` gives **14 passed in 51.96 s**.
The per-configuration numbers come from `python -m fw.onewire.measure`:

| What | Value |
|---|---|
| IMEM words used | **29 of 32** |
| Chip role | master (the model is the DS18B20) |
| Max speed on the ISS at 50 MHz | **CLKDIV 47: 14 184 slots/s, 1 773 bytes/s, 70.5 us per slot** |
| Speed the tests run at | CLKDIV 50: 13 333 slots/s, 1 667 bytes/s, 75.0 us per slot |
| Legal CLKDIV window | **47 .. 58** (14 184 .. 11 494 slots/s), measured at both ends and one step outside each |
| Peak FIFO depth | 1 of 4, and never a RXSTALL, a TXOVER or a RXUNDER |
| Violations | **zero**, at every legal divider |

The window is not a choice. Every timing is a whole number of SM cycles, so each DS18B20
window becomes a range of CLKDIV, and the program is legal exactly where they overlap. The
reset pulse is the longest timing and the one with a ceiling (AN126 H's 640 us), so it sets
both ends: 547 SM cycles of low is 514 us at CLKDIV 47 and 635 us at CLKDIV 58. Below 47 the
write-0 low drops under tLOW0's 60 us; above 58 the reset pulse grows past 640 us. Both
failures are in `test_the_divider_window_is_where_the_tables_say`, and both keep the bytes
correct, which is why they are worth quoting.

## The waveform, in SM cycles at CLKDIV 50 (1 us each)

| What | SM cycles | ns at CLKDIV 50 | Window |
|---|---|---|---|
| reset pulse (tRSTL) | 547 | 547 000 | 480 .. 640 us |
| release -> presence sample | 69 | 68 900 (the pad reads 60 ns earlier) | 60 .. 75 us, so every legal pulse is still low |
| release -> first slot (tRSTH) | 623 | 623 000 | >= 480 us |
| slot period (tSLOT) | 75 | 75 000 | 60 .. 120 us |
| write-1 low (tLOW1) | 6 | 6 000 | 1 .. 15 us |
| write-0 low (tLOW0) | 65 | 65 000 | 60 .. 120 us |
| sample point | 25 | 25 000 | 15 .. 60 us (tRDV .. tTX0) |
| byte to byte | 76, or 77 when the `pull block` stalls | 76 000 | tSLOT |

## Three things the ISS caught that reading the spec did not

1. **A `jmp x--` self-loop retires `n + 1` times.** The iteration where X has reached 0
   does not branch, but it still costs its delay, so a 16-iteration loop of `jmp x-- [31]`
   is 544 SM cycles and not 512. The reset pulse is 547 SM cycles, not 515, which moves
   both ends of the divider window.
2. **`IN_SHIFTDIR` has to be 1.** 1-Wire sends a byte least significant bit first, so a
   left-shifting ISR puts the first bit on the wire in bit 7 and the bytes come back
   *bit-reversed*: the model's ROM `28 01 02 ...` read as `14 80 40 ...`. The byte is now
   right-aligned in bits 31:24 and the presence bit is bit 31.
3. **The presence sample is a time, not a count of SM cycles.** A fixed 69 SM cycles is
   68.9 us at CLKDIV 50 but 80 us at CLKDIV 58, and 80 us is past the 60 us at which a
   presence pulse may still start, so a slave whose pulse was short and early would be
   missed. `driver.retarget()` rewrites the three `nop` delays of the pad, which is the only
   thing in `program.pio` that depends on the clock.

## What is not verified here

- **The RTL.** `FW_TARGET=rtl` skips, as allowed; nothing has been run against
  `src/pio_*.sv`. The driver only uses `ChipDevice`'s access queue, so the RTL path needs a
  Device that does the same over the SPI host port.
- **Overdrive speed.** The program is a standard-speed master: 70 to 87 us slots. The slot
  part would survive: at CLKDIV 6 the 75 SM cycle slot is 9.0 us (the overdrive w_slot is
  9.25 us) and the 65 SM cycle write-0 low is 7.8 us (overdrive tLOW0 is 7..14 us). Two
  things would not, and neither is a divider away:
  `presence_cycles(6)` wants 575 SM cycles of pad and the program's three `nop`s hold 97
  ("the presence sample cannot be placed at CLKDIV 6"), and the 547 SM cycle reset pulse
  would be 65.6 us against AN126 H's 68 us floor. Overdrive needs a bigger program -- a
  longer pad and one more loop iteration -- not a different divider.
- **The SPI host port.** Every register access here is one core cycle. Over SPI each costs
  ~34, which only lengthens the host's turnaround between phases (the bus idles longer, so
  the timing minimums can only improve) but was not measured.
- **Two engines.** The driver takes an `engine=` argument written against the register map
  and uses engine 0 in every test; engine 1 is untouched.
- **A real DS18B20's conversion time.** `Convert T` is issued but never waited for: the
  model's 750 ms conversion is 37.5 M ticks at 20 ns/tick, so the test never asks for one.
- **Parasite power and the strong pull-up.** `queue_idle` on the model's side (the bus held
  high for the conversion) has no firmware counterpart here: there is no instruction to hold
  the line in anything but released or pulled low.