# `fw/wiegand/` — MVP Wiegand reader firmware

| File | What it is |
|---|---|
| `program.pio` | the PIO program, 8 words of the 32-word IMEM, with the host configuration it needs written out and why |
| `driver.py` | the host side: register values, the SIA timing window, the 26-bit layout and its parity, `clkdiv_for_pulse()`, the startup sequence, sending a card |
| `test_wiegand.py` | the end-to-end test: `iss.pio_iss.ChipDevice` on a `verif.vip.core.Bus` at 20 ns/tick, judged by `verif/vip/wiegand.py`'s `WiegandReceiver` and `WiegandChecker` |
| `report.py` | the tables below, regenerated from the assembled words and the divider words |

## Run it

```
make fw PROTO=wiegand TARGET=iss          # 23 tests, about 2.5 minutes (see "The cost" below)
.venv/bin/python -m fw.wiegand.report     # the numbers, no simulation
.venv/bin/python -m asm.pioasm -m fw/wiegand/program.pio
```

`TARGET=rtl` runs the same program and driver against `src/pio_chip.sv` under cocotb, but skips
the tests that need a whole frame: one frame is 312 000 to 1 575 000 core cycles plus a million
ticks of quiet time, and the RTL harness talks to the simulator over pipes
(`fw/rtl_harness.py`, `## The cost is one pipe round trip`).

## Which side of the wire the chip plays

The **reader**: the box at the door, the side that initiates.  There is no clock, no handshake
and no reply.  A reader emits 26 low pulses on two open-drain lines, D0 and D1, where a bit is
a pulse on one line and both lines are high again between bits
(research/ref/protocols/wiegand/03_wikipedia_wiegand_interface.md), and then the lines go quiet.
The only frame boundary a receiver can see is that quiet time
(research/ref/protocols/wiegand/05_accessgrid_26bit_decode.md), so a frame is "26 bits, then a
pause longer than any pause inside it may be".

That makes the models in `verif/vip/wiegand.py` fall on both sides of this firmware:

| Model | Role here |
|---|---|
| `WiegandReceiver` | the door panel: passive, counts falling edges, decodes facility code, ID number and both parity bits. **The judge of the data.** |
| `WiegandChecker` | an analyser: passive, measures every pulse and pause against the SIA window and checks the frame length and the parity. **The judge of the timing.** |
| `WiegandReader` | the counterpart: a model of *this* firmware, used by the VIP's own self-tests, not by these tests |

The credential is the host's.  A real reader reads the card and formats the frame itself; this
chip has no card input at all, so the host sends the facility code and ID number over the SPI
port and the engine is the wire-timing machine.  The frame layout and both parity bits are
computed in `driver.py` from research/ref/protocols/wiegand/ — a *second* transcription of
Pyramid's PR-25 Table 2, the reference model having been written separately from the same
sources — and `test_the_two_transcriptions_of_the_timing_window_agree` /
`test_the_layout_is_the_bit_map_in_the_sources` check the two against each other in one
direction only, so a mistake in either shows up as a parity or frame-length violation instead of
cancelling out.

## The program

```
0: idle:  set pindirs, 0        both lines released
1:         jmp idle              and idle until the host forces `jmp send`
2: send:  set y, 25              26 pairs to shift out
3:         pull block            the first 16 pairs (32 OSR bits) from the TX FIFO
4: loop:  out pindirs, 2         the pulse: OSR[0] on D0, OSR[1] on D1, 1 SM cycle
5:         set pindirs, 0        the pause: both lines released, 10 SM cycles
6:         jmp y-- loop
7: last:  jmp idle               the frame is over; WRAP_TOP, so a PC off the end lands in `idle`
```

**8 words of 32.**  `idle` is first on purpose: there is no IRQ in the MVP to park an engine
with, so the program comes up doing nothing and the host starts a frame by forcing `jmp send`
through `SMx_INSTR`, the only way to set the PC (`docs/mvp-regmap.md`, `**SMe_INSTR**`).

### Open drain on a chip whose pins have no output-enable inversion

`docs/mvp-isa.md` `### Open drain` fixes the sense: the *level* register of an open-drain pin
stays 0 and the *direction* is what moves.  So the program writes both directions and never a
level, and two register values make that work:

* `PINCTRL.SIDESET_COUNT = 0`.  The reset value is 2 with `SIDE_EN = 0`, which makes **every**
  instruction side-set level 0 into `SIDESET_BASE` and its neighbour on every SM cycle, and the
  side-set channel wins over the OUT channel on the same pin
  (`docs/mvp-isa.md` `### Pin writes`).  Left at its reset value the two lines would be held
  low for ever.  It also has to be 0, not 1, because the delay field is what is left: five bits
  is what lets a phase be 32 SM cycles long.
* `OUT_COUNT = SET_COUNT = 2`, on `OUT_BASE = SET_BASE = D0` with `D1 = D0 + 1`, both `uio`
  pins.  `out pindirs, 2` pulls one line and `set pindirs, 0` has to release **both**; a
  one-pin SET window would leave the line of the previous pulse driven, and the specification
  forbids overlapping pulses outright.

### Why the data is two bits per frame bit

A frame bit chooses *which of two pins* to pull low, and `out pindirs, n` takes its value from
the OSR, so one frame bit would naturally be one OUT.  But `out pindirs, 1` writes OSR[0] to D0
and 0 to D1, and a direction of 0 means *released*: one bit can say "pull D0" or "release both",
never "pull D1".  The MVP has no instruction that branches on a data bit either — `jmp` tests X,
Y, OSR-emptiness and one input pin, and the only way to move a bit out of the OSR is to shift it,
which moves it to a pin as it goes — so the program shifts out **direction pairs**: `0b01` for a
data 0 (D0 low), `0b10` for a data 1 (D1 low), never both bits set, so the two lines are never
low together.  `driver.py`'s `direction_pairs()` and `frame_words()` build them.

A 26-bit frame is therefore 26 pairs = 52 OSR bits = two TX words.  `AUTOPULL` with
`PULL_THRESH = 0` (which the ISA defines as 32) refills the OSR when a whole word has been
shifted out, so the program pulls by hand exactly once.

## The timing, and what the chip cannot reach

`pw_ns` is what a host asks for; CLKDIV follows from it and from the two phase lengths read back
out of the assembled words, and `clkdiv_for_pulse()` **refuses** a speed whose pulse or pause
would fall outside the SIA window.  A configuration that cannot be expressed is therefore an
error, not a silent violation.  The window (pulse width 20..100 µs, pulse interval 200 µs ..
20 ms) is transcribed in `driver.py` from the sources cited in its docstring, and
`test_the_two_transcriptions_of_the_timing_window_agree` checks it against the model's own table.

At 50 MHz (20 ns per tick) with the three configurations the tests run:

| Configuration | CLKDIV | pulse | pause | bit | frame on the wire |
|---|---|---|---|---|---|
| `floor-20us`, engine 0, pins 0/1 | `0x03e8_0000` (1000) | 1000 ticks = 20 µs | 11000 = 220 µs | 240 µs | 312 000 ticks = 6.24 ms |
| `reader-50us`, engine 0, pins 0/1 | `0x09c4_0000` (2500) | 2500 = 50 µs | 27500 = 550 µs | 600 µs | 787 508 = 15.75 ms |
| `ceiling-100us`, engine 1, pins 5/6 | `0x1388_0000` (5000) | 5000 = 100 µs | 55000 = 1.1 ms | 1200 µs | 1 575 008 = 31.5 ms |

**Max speed on the ISS at 50 MHz: a 1000-tick pulse and an 11 000-tick pause, 4166.7 bit/s, a
frame in 6.24 ms.**  Two limits stack and neither is the chip's speed: the pulse is one SM cycle,
so its width *is* the divider period and the floor of the window (20 µs = 1000 ticks) is the
smallest one; and the pause is 10 SM cycles plus the `jmp` back into the loop, so at that
divider it is 220 µs, just inside its own 200 µs floor.

### What the 32-word IMEM and one divider do not reach

* **The pause/pulse ratio is at most 32.**  A phase is at most 32 SM cycles (the delay field is
  five bits) and at least one, so with a single shared divider the ratio cannot exceed 32: 1.
  BALTECH's shipped defaults, PulseWidth 50 µs and PulseInterval 2 ms, are a ratio of **40** and
  are therefore *not* reachable from this program; 50 µs and 1.6 ms (32:1) is, and is inside the
  same window.  Pyramid's typical Tpw 100 µs / Tpi 1 ms (40:1) is out of reach for the same
  reason — this program can do 100 µs with a 1.1 ms pause.  Reaching a 40:1 reader takes either
  a second pause instruction in the loop (a longer loop body, the same 32-SM-cycle ceiling per
  phase) or a host that rewrites CLKDIV between the phases, which moves the timing out of the
  engine and into the host.
* **The program cannot check parity, so the host must.**  Parity over twelve bits is a
  reduction, and the MVP has no ALU: `mov` shifts nothing, `set` writes an immediate, and the
  only shifter in the block is the OSR/ISR pair, which would need one bit time per bit.  26 data
  bits of parity would cost 26 bit times per frame — 26 × 240 µs = 6.2 ms of extra wire time on a
  6.24 ms frame, on a bus where a parity bit is *not* allowed to appear as a pulse because the
  panel would read it as data.  A PIO that shifts a frame out cannot also reduce it.
* **The panel side (the decoder) is not what this firmware does.**  Decoding needs a frame
  boundary, which is a timeout; the MVP has no IRQ and no timer an engine can wait on, so a
  door controller would have to be hosted, not programmed.  That is a real limit of the chip for
  the target role, and it is why the chip plays the reader here.
* **A frame cannot be shorter than 26 bit times plus a quiet time**, which at the fastest legal
  speed is 6.24 ms + 20 ms.  That is the protocol's shape, not a limit of the chip, but it is why
  a Wiegand reader is cheap and a Wiegand *bus* is slow.

## The cost of testing it

Every full-frame test runs at least the frame plus the frame gap, because a receiver closes a
frame when the lines have been quiet for longer than a pause may be — 1 000 000 ticks at 20 ns per
tick.  So one full-frame configuration is 1.3 to 2.6 million bus ticks, about 15 to 30 seconds of
Python at the ~85 000 ticks/s this bus does with four devices on it.  The long runs are
module-scoped fixtures and the assertions read their results; the mutation tests stop after a few
bit times, because a timing rule fires while it is being measured.

## What is judged, and by whom

Nothing in `test_wiegand.py` decides pass or fail by looking inside the ISS.  The models measure
the pins; the ISS is only asked for what a host can read over SPI (FDEBUG, DBG_PADOE, the PC, the
FIFO levels) and those are checked for cleanliness.  The three configurations each assert, from
the models alone:

* `chk.clean` — every timing rule, the frame length and both parity bits;
* `rcv.clean` and `(facility, number) == (123, 4567)` — the panel decoded the credential the
  host was asked to send;
* the measured pulse width and pause equal to what `phase_cycles()` and CLKDIV say they should
  be, and sit inside the model's own tick limits.

and then that the reader left an idle bus behind: both lines released, no uio pin driven except
the pair, no flag in FDEBUG, the engine parked in `idle`, the other engine untouched, the IMEM
words exactly what `asm/pioasm.py` produced, and no contention on any net.

The judge is not vacuous: three program/divider mutations (a divider far below the window, a
pulse 32 SM cycles wide, a pause that pulls D0 low instead of releasing it), an inverted data
frame and a frame one bit short all have to be caught, and an idle engine has to say nothing at
all.  `test_two_cards_a_frame_gap_apart_are_two_frames` shows the frame boundary is a quiet time
and not the end of a bit, and that two cards a gap apart are two frames with both credentials
intact.