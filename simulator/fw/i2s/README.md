# fw/i2s — MVP I2S controller firmware

Task **B3-FW-i2s**.  The chip plays the I2S **controller**: the device that generates SCK and
WS (UM11732 §3, "the device generating SCK and WS is the controller").  The bench holds no
`I2sTransmitter`; the firmware's own pins are the source on the wire, and `verif.vip.i2s`'s
`I2sReceiver` (UM11732 §6.2's receiver) and `I2sChecker` judge them.

```
make fw PROTO=i2s TARGET=iss          # the full test command
.venv/bin/python -m fw.i2s.measure    # the rates, the envelope
```

## What is here

| file | what it is |
|---|---|
| `program.pio` | two `.program`s: `i2s` (engine 0, 32 words) and `i2s_ws` (engine 1, 5 words) |
| `driver.py` | assembles them, builds every register, streams slots, stops the stream |
| `test_i2s.py` | six configurations on a `Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice` |
| `measure.py` | the CLKDIV sweep, the timing envelope, and the headline numbers |

`docs/mvp-isa.md` and `docs/mvp-regmap.md` are the only sources for what the hardware does;
the protocol rules come from `research/ref/protocols/i2s/` (NXP UM11732 §3, §4, §6.1, §6.2).
Nothing here is from memory of an RP2040 program — the one thing an RP2040 example would have
done differently (`OUT_SHIFTDIR`) is written out in `driver.py` `shiftctrl_value()`.

## The wire, and which pin moves when

```
tick      ...   w        w + T/2        w + T  ...
bclk           _/‾\_/‾‾\_/‾‾‾\_            rising at w and w+T
ws             changes at w; next change at w + 16T
sd             the previous word's LSB until w + T/2, the MSB from w + T/2
```

* `nop side 1` is the HIGH half: it raises SCK through the side-set window and writes no data,
  because SD must not move on the edge that latches it.
* `out pins, 1 side 0` is the LOW half: it lowers SCK and puts the next data bit on SD through
  the OUT window, one half period before the edge that latches it.
* Engine 1's `out pins, 1` is the first instruction of its slot, so WS moves on the same SM
  cycle as the edge that raises SCK, which is what `WS-EDGE` (slack 0) and `WS-1T` (one whole
  clock period) ask for.

## Why two engines

WS has to change in the same SM cycle as the edge that raises SCK, and an instruction's
side-set field is an immediate — so one engine needs a second copy of every HIGH/LOW pair per
channel.  That is 4 instructions per bit cell for a stereo frame, and with 32 words of IMEM
(docs/mvp-isa.md `### Engine state`) a one-engine program caps the slot at 8 cells, i.e.
8-bit samples.  Splitting the job leaves one copy: engine 0 keeps a single HIGH/LOW pair and
shifts the data, engine 1 writes WS once per slot.  Both engines wrap on the same slot length
(128 SM cycles) and are started by one CTRL write, which is what keeps them in lockstep
(`### Clock divider`: "Engines restarted together stay in lockstep").  `slot_cycles()` checks
that against the assembled words, so the lockstep property is measured, not assumed.

The price is one word of bookkeeping per slot and the lockstep risk of a dry FIFO, which the
stop mechanism exploits (below).

## Stopping without knowing a cycle boundary

Engine 0 stalls on the `out pins, 1` that follows an empty OSR; engine 1 stalls on the
`pull block` that is its last instruction of the slot, one cycle *before* its wrap.  Both run
out of TX FIFO words at the same boundary, so the final rising edge of the stream happens with
no WS change: the last slot is complete, and `I2sChecker.flush()` closes it without a
violation.  The two `FDEBUG.TXSTALL` flags that come with it are a stall on an empty FIFO, not
an error, and the test asserts they are the *only* flags set.

## Results

The numbers below come from the standard test command and the `measure.py` sweep (also
runnable with `--slots 1` for mono).

| what | value |
|---|---|
| words used | **32 of 32** (engine 0), **5 of 32** (engine 1) |
| slot | 16 cells, 128 SM cycles, 50% duty, one wrap = one slot |
| fastest clean stream of the shipped program | **6.2500 MHz** SCK, **195312.500 Hz** sample rate (CLKDIV 1, 8 ticks per bit clock) |
| slowest configuration measured | CLKDIV 8 → 0.7812 MHz SCK, 24414.062 Hz |
| other CLKDIVs measured | 3.1250, 2.0833, 1.5625, 1.2500, 1.0417, 0.8929 MHz — all clean |
| violations | **0** in every configuration and in every pattern |
| fastest stream the judge accepts at all | 12.5 MHz SCK / 390625 Hz (`HALF_DELAY 1`, 4 ticks per bit clock) |
| what stops it going faster | `verif.vip.i2s.MIN_TICKS_PER_BIT_CLOCK = 4`: a 2-tick period (25 MHz) is a geometry the models refuse to be configured for, so no checker can judge it |

The `HALF_DELAY` sweep in the log is a measurement, not a shipped configuration: `measure.py`
re-assembles the same file with that one `.define` rewritten.  The shipped firmware keeps
`HALF_DELAY 3`, which keeps two whole ticks of margin on `tHC`/`tLC` (0.35 × 8 = 3 ticks of a
4-tick half) instead of sitting exactly on the limit at `HALF_DELAY 1`.

## What is not covered

* **The target side.** The chip is the controller.  An I2S *target* would sample SD on SCK's
  leading edge and push the words into its RX FIFO — expressible on this chip (an RP2040 does
  SPI slaves with it), but it is a second program, and only the controller side is built here.
* **The `abs25` profile.** `I2S_PROFILES["abs25"]` adds the absolute limits of Tables 1 and 2
  (360 ns ≤ T ≤ 440 ns, tHC ≥ 160 ns).  The firmware's fastest bit clock is 160 ns at CLKDIV 1
  and everything slower is inside that window, so the profile is reachable; it is not part of
  the acceptance run.
* **`FW_TARGET=iss_b` and `FW_TARGET=rtl` were not run.** They are wired for this protocol the
  same way as for every other `fw/` one: `fw/conftest.py` swaps the backend behind
  `iss.pio_iss.Chip` for the whole directory, and `fw/rtl_harness.py` is protocol-agnostic
  (`fw/Makefile`: "There is nothing protocol-specific here").  `python -m fw.targets` lists both
  `iss_b` and `rtl` as available on this machine.  Only `iss` was run for this protocol, because
  that is the standard test command — so the test file here has never been run against the
  RTL or the second ISS, and this README does not claim it works there.
