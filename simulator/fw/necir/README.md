# `fw/necir/` -- MVP NEC infrared receiver

MVP firmware for the NEC remote-control protocol, judged by the pin-level models in
`verif/vip/necir.py`.

## What this is

| file | what it is |
|---|---|
| `program.pio` | the whole receiver: 27 words of PIO, assembled by `asm/pioasm.py` |
| `driver.py` | the host side: register packing from `docs/mvp-regmap.md`, the sample-window arithmetic, and `NecirReceiver`, which loads, arms, starts and drains the engine |
| `test_necir.py` | the end-to-end test: the ISS on a `verif.vip.core.Bus` with `NecirTx`, `NecirRx` and `NecirChecker` on the wire |
| `__init__.py` | re-exports, and the three numbers worth knowing before reading `program.pio` |

**The chip is the target.** NEC is one-way, so the chip is a *receiver*: it never drives the net,
it listens on one pin, and every frame it decodes arrives as one 32-bit word in its RX FIFO --
exactly the 32 bits that were on the wire, LSB of word 0 first. A repeat code, which NEC defines
as carrying no data, pushes nothing. So the firmware's output is words, and the judgement is
whether those words equal `verif.vip.necir.encode_frame(address, command)`.

## How to run it

```
make fw PROTO=necir TARGET=iss          # the main command: 43 tests, ~13 s
FW_TARGET=rtl make fw PROTO=necir       # the same on src/pio_chip.sv, if the RTL exists
```

Both go through the shared slot queue when the machine is busy:

```
make fw PROTO=necir TARGET=iss
```

The test run passes 43 tests, and the tables below are printed by the driver itself.

## The design in one page

Every element of NEC is a whole multiple of one 562.5 us burst -- the leader is 16, a frame's
leader space 8, a repeat code's 4, a data burst 1, a data space 1 or 3 -- so the host divides the
clock until one SM cycle is a fraction of a burst (`S` = how many SM cycles a burst takes) and the
program measures the wire in SM cycles. There is no arithmetic in the program, no counter except
one, and no pin write at all.

Three samples, each placed by a delay and each inside the window the checker's tolerance leaves it
(`driver.py:sample_window` computes the windows from the model's own `NECIR_TIMING`):

| sample | at | judges |
|---|---|---|
| leader mark | +`MARK_CYCLES` (102) | is this burst a 9 ms leader, or noise / a data burst? |
| leader space | +`SPACE_CYCLES` (49) | 8 bursts means a frame, 4 means a repeat code |
| data space | +`K` (12) | high means the space is over, so the bit is a `0` |

The frame ends on `jmp !osre bit`: `out null, 1` counts the bits in the OSR, and `!osre` is false
once 32 have gone by. That is what makes a frame *exactly* 32 data bits, and it means the program
never has to recognise the trailing burst or the idle after it -- the one thing a level sample
cannot do at this tolerance, because a `1` space of 2.25 bursts and a space of 3.75 bursts bracket
every sample point in between.

## Results

All numbers below are measured and printed by `fw/necir/driver.py`.

**Words used: 27 of 32.**  Five spare.

**Sample margins**, read back out of the assembled words and compared with the windows at the
checker's default 25 % tolerance, S = 9 (one burst = 562.5 ns = 28.125 core cycles at 50 MHz, one
SM cycle = 3.125 core cycles = 62.5 ns). Quoted in bursts, which is the scale-free unit:

| sample | at | room below | room above |
|---|---|---|---|
| leader mark | 102 SM cycles (11.3 bursts) | 10.05 bursts | 0.52 bursts |
| leader space | 49 SM cycles (5.4 bursts) | 0.48 bursts | 0.41 bursts |
| data space | 12 SM cycles (1.33 bursts) | 0.12 bursts | 0.19 bursts |

The leader-mark sample is placed as late as its window allows on purpose -- it has to accept every
legal leader, whose shortest legal length is 12 bursts at this tolerance -- so the room *below* it
is large and the room above it is what protects the shortest legal leader. At a real remote's
scale the same three samples have 5.67/0.31 ms, 250/250 us and 109/62 us of room.

The program also builds, and decodes, at the sources' own spread of 5 % tolerance, and for every
`S` from 7 to 12 (`test_every_usable_grid_decodes_and_the_useless_ones_are_refused`).

**Usable grids.** `S` may not be 1: the data-space sample has to be late enough that a `0` space
(1.25 bursts at 25 % tolerance) has certainly ended and early enough that the bit loop is back at
`wait 1 pin, 0` while the burst after it is still on the wire, since a `0` bit is only two bursts
long. Those bounds cross at S = 7, and the 5-bit delay field ends the range at S = 12. The driver
refuses 1..6 and 13.. rather than loading a program whose sample cannot be placed.

**Max speed.** NEC's rate is fixed, so the only speed in a receiver is how finely the SM cycle
divides the burst: the fastest is `S = 12`, CLKDIV = 2.34375 core cycles (one SM cycle every
46.9 ns), and the slowest that still decodes is `S = 7`, CLKDIV = 4.0178. The default `S = 9` is
CLKDIV = 3.125.

**The model runs NEC 1000x faster than a real remote.** `verif/vip/necir.py` states its element
timings in nanoseconds, so its burst is 562.5 ns where a real NEC burst is 562.5 us (and its own
`research/ref/protocols/necir/README.md` does the arithmetic at that scale: "one tick is 3.56 % of
a burst" = 20 ns / 562.5 ns). Nothing in the firmware depends on the scale -- the samples are SM
cycles -- so `build_program(9, unit_ns=REAL_NEC_UNIT_NS)` gives the same 27 words a real remote's
timings, with CLKDIV = 3125, which still fits the 16-bit INT with 2.1x to spare
(`test_the_same_program_decodes_a_real_nec_remote_too`).

**Minimum core clock.** A `ChipDevice` steps one core cycle per bus tick, so the tick rate *is* the
core clock, and CLKDIV cannot go below 1.0: one SM cycle is at least one core cycle, so a burst has
to be at least `S` core cycles long. At the model's scale that is 16 MHz for `S = 9` (9 / 562.5 ns);
the driver refuses a slower bench and says by how much, which is why every test here runs at 20 ns
per tick and the 110 ms repeat raster is scaled rather than the clock.

## What is not implemented, and why

* **The leader mark is validated from below only.** A burst shorter than `MARK_CYCLES` is
  rejected, so noise and data bursts cannot sync the program mid-frame and a 4 ms or 6 ms "leader"
  is not decoded -- but a leader that is too *long* (12 ms, which the checker flags) is still
  decoded. Catching that would need a bound from above, i.e. checking that the mark *ended* within
  some time, which no `wait` can express: the engine only learns about the fall when it happens.
* **The extended (16-bit address) form needs no firmware change** -- the wire is the same 32 bits
  and word 1 is simply not the inverse of word 0 -- but the *checker* and the *decoder* have to be
  told (`extended=True`), and the test suite exercises the standard form. The chip pushes the raw
  word either way.
* **No carrier, no AGC, no ambient light, no NECx or NEC2.** The models are demodulated and say so
  (`verif/vip/necir.py`'s docstring, judgement call 8), and a 38 kHz carrier is not something this
  instruction set can generate.
* **The RX FIFO is four deep and there is no flow control.** A host that stops reading loses
  frames; a `push block` stalls the engine, which is the only back pressure there is.

## The interesting negative results

Two of them, both from the tests, both about the checker rather than the firmware:

* On an illegal wire -- a 1 ms data space, in the gap between the `0` and `1` windows -- the
  firmware recovers the sender's intent (its sample sits inside the `1` window, so a 1.78 burst
  space still reads as a `1`), while `NecirRx`, which re-derives its phase from every edge, loses
  its place in the stream and reports zeros. The checker fires `bit-space` 16 times either way.
  A phase-tracking decoder and a re-synchronising one fail differently on a broken wire; only the
  checker says so.
* A *receiver* cannot be judged by a passive protocol checker on the wire it listens to: the wire
  is the remote's. So `test_the_checker_sees_the_same_wire_with_the_chip_attached` drives the same
  press twice, once with the chip on the bus and once with it removed, and requires the two
  checkers to report identically. That is what makes "zero violations" in the other tests a
  statement about the firmware.
