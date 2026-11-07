# fw/swd -- SWD controller firmware for the MVP chip

Task **B1-FW-swd**.  The chip is the **controller**: it generates SWCLK and drives SWDIO
open drain, and the SW-DP on the other end of the two wires is `verif.vip.swd.SwdTarget`.
The judge is `verif.vip.swd.SwdChecker`, which decodes the wire and holds it to every rule
it knows; a violation fails the test.

Run it:

    make fw PROTO=swd TARGET=iss        # the accept command, 13 tests
    python -m fw.swd.measure            # the numbers below, from the ISS

## What is here

| File | What it is |
|---|---|
| `program.pio` | the PIO program: 31 of the 32 words, three bit-bang loops and four entry points |
| `driver.py` | the host side: register words from docs/mvp-regmap.md, the CLKDIV/PINCTRL/SHIFTCTRL/EXECCTRL values, one `transfer()` per frame, the line reset |
| `test_swd.py` | the bench: chip, DP and checker on a `verif.vip.core.Bus` at 20 ns/tick, judged by the checker |
| `measure.py` | prints the word budget, the bit-rate range and a whole session per configuration |
| `LIMITS.md` | what the 32-word IMEM bought, what it did not, and the three places the firmware had to be written around a missing instruction |

## How it fits together

A transfer is one 46-period SWD frame and the program is five phases, each a run of whole
bit periods:

    xfer      9 periods   the request byte from the TX FIFO, then the turnaround
    wack/ck   2 periods   the ACK, clocked and not sampled
    rd/rx    32 periods   the RDATA bits, sampled into the ISR
    p2        3 periods   RDATA parity, the DP's turnaround, the pull-down that resyncs it

A write replaces `rd` with one turnaround period and 32 periods of WDATA, and `p2` carries
its parity bit instead.  The host starts each transfer by forcing two instructions --
`set pins, n` (read or write) and `jmp xfer` -- and the TX FIFO carries the three words the
program pulls.

The word budget and why each word is there is in `LIMITS.md`; the two numbers worth
repeating here are that a bit period costs **three instructions plus two delays** (which
is why a 46-period frame fits in 31 words at all) and that the fastest SWCLK is therefore
**5.56 MHz** at a 50 MHz core clock.

## Results

Measured on the golden ISS, six configurations, each running a line reset, an
IDCODE read, a CTRL/STAT write and read and a SELECT write and read:

| HIGH | LOW | period (core cycles) | measured SWCLK | IDCODE | SELECT | frames | violations | contention |
|---|---|---|---|---|---|---|---|---|
| 6 | 0 | 9 | 5.556 MHz | ok | ok | 5 | 0 | 0 |
| 6 | 1 | 10 | 5.000 MHz | ok | ok | 5 | 0 | 0 |
| 6 | 5 | 14 | 3.571 MHz | ok | ok | 5 | 0 | 0 |
| 6 | 14 | 23 | 2.174 MHz | ok | ok | 5 | 0 | 0 |
| 8 | 9 | 20 | 2.500 MHz | ok | ok | 5 | 0 | 0 |
| 13 | 14 | 30 | 1.667 MHz | ok | ok | 5 | 0 | 0 |

The "measured SWCLK" column is `SwdChecker.shortest_period`, i.e. the shortest period the
wire actually carried: it equals the driver's period in every configuration, which is the
check that the phase boundaries (`wack`'s `[LOW - 1]` padding) do not run the clock short.

`make fw PROTO=swd TARGET=iss` -- 13 passed.

## Pin map

`SWDIO` is a `uio` pin (open drain is a *direction* on this chip, docs/mvp-isa.md
`### Open drain`, and only the eight bidirectional pins have one), `SWCLK` is an
output-only pin (always driven, so the side-set only writes its level and no direction has
to be programmed anywhere), and a third pin carries the host's read/write flag for
`jmp pin`.  The defaults are SWDIO 0, SWCLK 13, FLAG 14; all three are PINCTRL fields, so
`test_two_speeds_and_a_second_pin_set` runs the same words on pins 4, 16 and 17.

## Related

* `docs/mvp-isa.md` -- the instruction set, the timing constants (T_OUT, T_IN, the divider)
  and the pin kinds this firmware depends on.
* `docs/mvp-regmap.md` -- the word addresses and field positions the driver writes.
* `research/ref/protocols/swd/` -- the ADIv5 text the VIP cites; the program's phase
  lengths and the parity rules come from there.
* `fw/i2c/` -- the same shape of problem (a bit-bang protocol with open drain and no IRQ)
  on the other of the two engines.