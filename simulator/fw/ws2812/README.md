# WS2812 / WS2812B controller firmware for the MVP chip

The chip plays the **controller** of a WS2812
chain: it drives the single data line, and the pixels are the targets -- which is the side
`verif/vip/ws2812.py` puts its `Ws2812Tx` (controller) and `Ws2812Pixel` (strip) on.

    make fw PROTO=ws2812 TARGET=iss          # the main command
    .venv/bin/python -m fw.ws2812.measure    # the measured table below, into metrics/fw.csv

## What is here

| File | What it is |
|---|---|
| `program.pio` | two `.program`s, both made entirely of the side-set channel: the datasheet's own rate and the fastest this ISA can legally go |
| `driver.py` | the host side on `iss.pio_iss.ChipDevice`: IMEM, the four per-engine registers, the pin direction, the GRB word packing, and the waits between frames |
| `test_ws2812.py` | five end-to-end configurations judged by `verif.vip.ws2812.py`, plus eleven mutations that each have to be caught |
| `measure.py` | measures the wire and appends rows to `metrics/fw.csv` |
| `__init__.py` | the module's public names |

## Results

`make fw PROTO=ws2812 TARGET=iss` -- **54 passed, 1 skipped** in 9.7 s
(the skip is a limit of the *model*, not of the
firmware -- see "What this firmware cannot be judged on" below).

Measured on the ISS at 50 MHz, one `verif.vip.core.Bus` tick per core cycle:

| case | program | CLKDIV | pixels | words | FIFO | bit period | bit rate | violations |
|---|---|---|---|---|---|---|---|---|
| `794k-1px` | `ws2812` | 1.0 | 1 | 9/32 | 1 | 63 ticks | 793.7 kbit/s | 0 |
| `747k-frac-4px` | `ws2812` | 1+16/256 | 4 | 9/32 | 3 | 66..68 | 735.3..757.6 kbit/s | 0 |
| `1250k-fast-1px` | `ws2812_fast` | 1.0 | 1 | 8/32 | 1 | 40 ticks | 1250.0 kbit/s | 0 |
| `1250k-fast-8px` | `ws2812_fast` | 1.0 | 8 | 8/32 | 4 | 40..41 | 1219.5..1250.0 kbit/s | 0 |
| `794k-engine1-up` | `ws2812` | 1.0 | 2 (engine 1, pulled-up line) | 9/32 | 1 | 63..64 | 781.2..793.7 kbit/s | 0 |

The four high and low times, as measured on the wire, against the windows
`ws2812_timings()` enforces at 20 ns per tick (T0H 13..27, T1H 33..47, T0L <= 50,
T1L <= 30, TBIT 33..92, reset >= 2500 ticks):

| case | T0H | T1H | T0L | T1L |
|---|---|---|---|---|
| `ws2812`, CLKDIV 1.0 | 20 | 40 | 43 | 23 |
| `ws2812`, CLKDIV 1+16/256 | 21..22 | 42..43 | 45..47 | 24..26 |
| `ws2812_fast`, CLKDIV 1.0 | 13 | 33 | 27 | 7 |

The nominal program lands on the datasheet's own numbers to within a tick: T0H 400 ns
(nominal 400), T1H 800 ns (nominal 800), T0L 860 ns (nominal 850), T1L 460 ns (nominal
450), and a 1.26 us bit period against "TH+TL = 1.25 us +/- 600 ns" -- i.e. 793.7 kbit/s
against page 1's "Send data at speeds of 800Kbps".

**FDEBUG** is `0x0100_0000` (`TXSTALL` on engine 0) or `0x0200_0000` on engine 1 in every
case, and nothing else: the engine is parked on `pull block` with an empty FIFO, which is
where a WS2812 transmitter spends the time between frames.  `TXOVER`, `RXUNDER` and
`RXSTALL` are never set, so no word was dropped and no read underran.

## How it works

The wire carries no clock, no handshake and no reply: a bit is *only* the length of a high
pulse, so every edge in this firmware is made by the side-set channel.

* `.side_set 1` with `SIDESET_COUNT = 1` and `EXECCTRL.SIDE_EN = 0` (its reset value) makes
  `side 0`/`side 1` the level of one pin, on every instruction, with the delay field of the
  same word saying for how long.  That leaves 4 bits of delay, so one instruction is 1..16 SM
  cycles -- the granularity every chunk below is a multiple of.
* `out x, 1` shifts the next bit of the OSR into X *and* starts the high pulse;
  `jmp !x zero` reads that X on the next SM cycle and chooses the pulse length.  The branch
  has one side-set value for both paths, so both bit values' pulses *start* on the same grid.
* `jmp !osre bitloop` is the bit counter: `!osre` is `osr_cnt < PULL_THRESH`, and with
  `SHIFTCTRL.PULL_THRESH = 24` and one bit per `out x, 1` it loops over a word's other 23 bits
  and then falls through to the `pull block`.  Nothing else in the MVP ISA counts bits.
* The `pull block` at the end of the loop is where the engine waits for the next word, and it
  carries `side 0`: a low of more than 50 us *is* the reset code, so a transmitter that has
  run out of data must leave the line low.  A stalled instruction keeps the level it last
  wrote, which is what holds the line low between frames.  No instruction that can stall
  carries `side 1`.
* `OUT_SHIFTDIR = 0` sends OSR bit 31 first, which is the datasheet's order (most significant
  bit first, green byte first, page 5), so the host pushes each pixel's 24 bits left-justified
  into bits 31:8.
* The host, not the engine, decides where a frame ends: nothing but the reset code marks it,
  so `send()` waits out the frame and then `RESET_TICKS` (2500) of low before the next frame
  is pushed.

`driver.py` derives the per-bit high and low times **out of the assembled words**, by walking
the program's labels, and `timing()` raises rather than guessing if a program is not the shape
it can time (a missing `jmp !osre`, a low chunk that is not a straight-line `nop`, a pull that
side-sets high, two bit values of different lengths, ...).  So CLKDIV, the frame length and
the reset wait all follow from `program.pio`, not from a constant written twice.

## The two speeds, and why there are only two

The bit period is a sum of instruction durations, so it is quantised to whole SM cycles and
then to whole core cycles by CLKDIV.  The datasheet's own rate needs 62.5 ticks per bit and the
grid can express 63, which is 1.26 us -- inside "TH+TL = 1.25 us +/- 600 ns" -- so `ws2812`
runs at 793.7 kbit/s.  The fastest anything can go is set by the windows, not by taste:

* a 1 bit's high must be at least `t1h_min` = 33 ticks, and no instruction is longer than
  `max_cycles_per_instruction` = 16 SM cycles, so the 1-bit path needs at least three high
  chunks;
* a 0 bit's high must be at least `t0h_min` = 13 ticks and it shares its first chunks with the
  1 bit, so those have to sum to at least 17 for the third to finish the job;
* the low after a 1 bit has no floor in the datasheet but needs at least one instruction.

33 + 1 = **34 ticks, i.e. 1.47 Mbit/s, is the ISA's ceiling** for this protocol
(`test_the_fastest_speed_this_isa_can_drive_the_line_at`).  `ws2812_fast` reaches 40 ticks =
0.8 us = 1.25 Mbit/s: both its high times sit exactly on their floors (260 ns and 660 ns
against nominals of 400 and 800 ns), which is as tight as the windows allow and still legal.
Fractional CLKDIV can slow both programs down (up to FRAC = 41 before a 1 bit's high leaves
its window), but not speed them up: the fastest period is already at the high-time floors.

## What this firmware cannot be judged on

One case, `1250k-fast-8px`, does not assert the strip's colours, and the reason is in
`verif/vip/ws2812.py` rather than here.  `Ws2812Pixel` forwards on its DOUT one tick after it
sampled on its DIN, so the first pulse it hands downstream is one tick shorter than the one it
received.  A 1 bit's high is 33 ticks at `ws2812_fast`, which is exactly `t1h_min`, so that
tick lands in the dead band between the two high-time windows, the bit is undecodable, and the
far pixel of a chain never receives its 24 bits.  A real WS2812's `tPLZ` is sub-microsecond and
does not shorten a pulse.

The controller's own net is clean in that same run -- 384 bits, 2 frames, 0 violations -- and
`test_the_chain_model_cannot_follow_the_fastest_speed` asserts both halves, so the gap in the
coverage is a documented model limit rather than an unexplained hole.  At the datasheet's rate
the chain model follows all eight pixels correctly (63 ticks per bit leaves the one lost tick
inside every window).

## Not verified here

* `TARGET=rtl` is wired the same way as `fw/uart_tx` and `fw/i2c` (`FW_TARGET=rtl` installs
  `fw/rtl_harness.py` and runs the same checks on `src/pio_chip.sv`), but only `TARGET=iss` was
  run (the main command), and the RTL build is a heavy job.
* Only the 9-Apr-2013 Worldsemi datasheet revision is covered.
* The strip model is the VIP's: this firmware has never been measured against real LEDs.