# fw/dmx512 -- MVP DMX512-A controller firmware

MVP firmware for DMX512-A, judged by the reference models in `verif/vip/dmx512.py`:
`DmxRx` decodes the line the way a receiver does and `DmxChecker` measures the wire and flags
every timing or framing rule ANSI E1.11 states.  Neither is modified here; they are the judge.

    make fw PROTO=dmx512 TARGET=iss      # the accept command, 54 tests, ~32 s
    .venv/bin/python -m fw.dmx512.report # what the firmware achieves, measured

**The chip is the controller** (the transmitter).  Every rule the checker enforces is a
transmitter's -- tables 6 and 7 of the standard, clauses 8.1 to 8.10 and 9.1 -- and a receiver
role would need the chip to sample an asynchronous line at 250 kbit/s through a 3-cycle input
pipeline (`T_IN = 3`) with no IRQ to tell the host when a slot has landed.

## What is here

| file | what it is |
|---|---|
| `program.pio` | the whole transmitter: BREAK, MAB, START Code, data slots, mark before the next break. 28 of the 32 words of one engine's IMEM. |
| `driver.py` | assembles the program for a bit rate and a frame length, derives every duration from the words it produced, programs an `iss.pio_iss.ChipDevice` and feeds it one FIFO word per slot. |
| `test_dmx512.py` | the eight configurations on a `verif.vip.core.Bus` at 20 ns/tick, plus the checks that the judge is not vacuous. |
| `report.py` | the measured numbers next to the derived ones, and the three headline numbers. |

The `.pio` header carries the argument for every register the program needs; `driver.py`'s
module docstring carries the argument for the design as a whole.

## The frame, and why the program looks like this

```
BREAK (space, >= 92 us) | MAB (mark, >= 12 us) | slot 0 = START Code (0x00)
| slots 1..N, each a start bit, 8 data bits LSB first, 2 mark stop bits
| mark until 1204 us have passed since the last BREAK's falling edge
```

* **No side-set**, so the whole 5-bit delay/side-set field is delay: one instruction is up to
  32 SM cycles, which is what a 4 us bit at 50 MHz needs (`(1 + BIT_DELAY) * CLKDIV INT` core
  cycles, exact, because the divider is an integer).
* **One instruction per bit.** A slot is 12 instructions: start bit, eight `out pins, 1`, two
  stop bits, and the loop's own jump. The jump is a bit time too, so a slot is 12 bit times on
  the wire rather than 11 -- slower than the tables' minimum update time, and legal, because
  clause 8.8 wants the line at mark between slots. There is no room in 32 words for a
  zero-delay jump, which would be the only way to spend a bit time without an instruction.
* **The break and the pad are loops, not unrolled instructions.** A 92 us break at CLKDIV 8 is
  23 bit times; unrolled that would be 23 words. Two loops (break, mark-before-break) do it in
  five, and each loop's counter is a `set` immediate, which is also what limits a frame to
  1..31 slots.
* **The host keeps the TX FIFO fed.** The eighth `out pins, 1` of a slot pulls the next slot's
  byte in the same SM cycle; an empty FIFO costs a whole SM cycle of jitter, where the tables'
  tolerance is two ticks. `driver.feed` tops the FIFO up while the bus runs.

## Results

Measured with `python -m fw.dmx512.report`:

* 28 words of 32; frames of 1..31 slots; every whole-tick bit length from 196 to 204 is
  expressible, so **every** bit rate the tables allow at 20 ns/tick.
* 255.1 kbit/s (196 ticks, delay 27, CLKDIV 7) is the fastest legal bit, and 50 Mbit/s the
  fastest the program shape could express at all (one instruction per bit, CLKDIV 1.0) --
  200x the nominal rate, and not a DMX512 transmitter.
* Frame time 64 337 .. 80 176 ticks against table 6's 60 200 minimum; refresh 623.6 .. 818.6 Hz
  against the tables' "approximately 830 Hz".
* `DmxChecker`: **zero violations** on all eight configurations, against table 7 (the
  receiver's) and, for one of them, against table 6 (the transmitter's) as well.
* The checker's measured break, MAB, bit period and break-to-break equal the numbers the driver
  derived from the instruction words, exactly, in every configuration.

## What the tests hold it to

`test_dmx512.py` is 54 tests in five groups: the transaction (every slot, in order, with a NULL
START Code, judged by both models); the numbers (the checker's measurements against the derived
ones); the chip left clean (no TXOVER/RXUNDER/TXSTALL/RXSTALL, one pin driven, the other engine
untouched, no contention); the program itself (28 words, the shape read back out of the words,
`frame_cycles` an accounting identity); and **the judge is not vacuous** -- eight one-word
mutations each have to be caught, the pad loop is shown to be load-bearing, and both halves of a
starved TX FIFO are shown (a word a few ticks late is a BIT-RATE violation; a FIFO empty for
good just stops the frame, which only FDEBUG.TXSTALL and the receiver notice).

## Known limits

* **31 slots per frame.** A 512-slot frame needs a 9-bit counter and `SET`'s data field is 5
  bits; the only 32-bit load is a FIFO word, so the host would have to force `pull` and
  `mov x, osr` for every frame, and the program would lose the word that reloads X. See
  `driver.slots_limit`.
* **No IRQ.** The controller cannot be told a frame has started, so a host that wants to change
  the frame length mid-run has to restart the engine -- and `SM_RESTART` clears `osr_cnt` without
  touching the OSR, which makes the background autopull take one FIFO word. Start the engine
  with an empty FIFO, or use `send`, which fills it first.
* **Not run on the RTL.** `FW_TARGET=rtl` skips the file: this firmware is tested on `TARGET=iss`. The
  driver and the program are target-independent (only the `ChipDevice` differs).