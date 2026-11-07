# fw/quadrature -- MVP firmware for an incremental encoder (ABI) interface

Task **B1-FW-quadrature**. The chip is the **target** (decoder) end of the quadrature-encoder
protocol: an external encoder drives three wires -- `A`, `B` and the index `I`, the ABI outputs
of an ams/Infineon AS5047P -- and this firmware watches them, logs every change into the RX FIFO
and hands the host a sample stream. Nothing in the chip drives a pin, and a test says so at the
register level.

| File | What it is |
|---|---|
| `program.pio` | the PIO program, **7 of the 32 IMEM words**: a sync sample, a change detector and a sample log |
| `driver.py` | the host side: `QuadratureDriver` loads the program into one engine over `iss.pio_iss.ChipDevice` and drains the RX FIFO while the simulation runs; `HostDecoder` turns the sample log into a position |
| `test_quadrature.py` | 46 pytest cases against `verif/vip/quadrature.py`, the reference model and checker written independently of the firmware |
| `CANNOT-EXPRESS.md` | not needed: the protocol fits, with room to spare |

## How to run it

```
make fw PROTO=quadrature TARGET=iss      # the main command: 46 passed in 6.5 s
make fw PROTO=quadrature TARGET=iss
make fw PROTO=quadrature TARGET=rtl      # the same 46 cases on src/pio_chip.sv: 46 passed in 93 s
```

The chip core clock is 50 MHz, i.e. one `verif.vip.core.Bus` tick at `ns_per_tick = 20`, which is
what every VIP in `verif/vip/` uses. `FW_TARGET=rtl` swaps `iss.pio_iss.Chip` for the RTL through
`fw/rtl_harness.py` and changes nothing else; it needs a bigger simulator pool than the harness
default because this file keeps one bench per case alive, so the test defaults `FW_RTL_MAX_SIMS`
to 12 (the harness's own knob).

## What the program does, in one paragraph

`in pins, 3` samples three contiguous pins from IN_BASE, so the driver puts `B` at IN_BASE, `A`
one pin up and `I` two pins up, and one sample is `(I << 2) | (A << 1) | B` -- the same packing the
reference model uses for `ab` and `index_level`. The loop reads the pin vector into X, compares it
with the vector it last logged, and on any change shifts the sample into the ISR; AUTOPUSH at 30
bits sends ten samples per 32-bit FIFO word. Two SM cycles per poll, five per logged sample, and
the words come out at the speed of the shaft and not at the speed of the poll, because a poll that
sees nothing changes never reaches the `in`.

Direction, the x1/x2/x4 factor, the impossible-transition check and index homing are host-side,
which is exactly where `QuadratureDecoder` keeps them in the reference model: one copy of the Gray
rule (`verif.vip.quadrature.step_delta`, imported by `driver.py`), not two.

## Results

All of these were measured on the golden ISS; the speed numbers are in
`metrics/quadrature_max_speed.csv`, which `test_max_speed_on_the_iss` writes.

| Number | Value | Where it comes from |
|---|---|---|
| IMEM words used | **7 of 32** | `test_the_program_is_seven_words_and_says_what_it_does` |
| sampling grid | 2 SM cycles (40 ns at CLKDIV 1.0) | `describe()["unchanged"]` |
| cost per logged edge | 5 SM cycles (100 ns) | `describe()["changed"]` |
| fastest encoder followed, ISS at 50 MHz | **112 000 RPM**, 4.0x the AS5047P's 28 000 RPM | `metrics/quadrature_max_speed.csv`, the sweep loses counts from 224 000 RPM |
| arithmetic ceiling | 10 M logged samples/s = 150 000 RPM at 4000 counts/rev | `CORE_HZ / ticks_per_sample` |
| counts on the plain case | 200 counts at 10 000 RPM, chip == model == encoder, 0 errors, FDEBUG 0 | `test_the_chip_counts_the_shaft[x4-10krpm]` |
| checker | 0 violations in every configuration | `test_the_checker_reports_no_violation` |
| pins driven | none: `DBG_PADOE` has no uio bit set | `test_the_chip_drives_nothing` |

Configurations covered (`CASES` in the test file): x4 forward at 10 000 RPM, x4 at 300 RPM, x2,
x1, x4 in reverse, and a second encoder on engine 1 over pins p3/p4/p5 at a different speed and
decoding factor.

## Things a reader should know before trusting it

* **The chip logs, the host counts.** The two halves of the decoder are split on purpose, and
  program.pio says why in as many words: seven words for the part that has to run at wire rate,
  `HostDecoder` for the part that is arithmetic.
* **`mov x, pins` is unmasked.** The change detector watches all 32 pin positions, so traffic on
  an unrelated pin is logged as a sample whose A/B pair is unchanged. It cannot become motion --
  the Gray rule already says a repeated state is not a step -- but it costs a FIFO word.
  `test_a_moving_pin_that_is_not_an_encoder_is_logged_but_not_counted` is the proof.
* **A word must be full.** A host-forced `push block` produces a word whose unused high bits are
  zeros, and a zero sample is indistinguishable from a real state 00. `flush()` avoids that by
  forcing the program's own log instruction ten times, so AUTOPUSH sends a complete word whose
  tail is copies of the current state. program.pio and `flush()` both explain this.
* **The VIP's glitch injector only knows the nets "a", "b", "i".** `RotaryEncoder._tick_faults`
  looks the net up in a hard-coded tuple, so `glitch()` raises `ValueError` on any other name.
  verif/vip is read-only for this task, so the test drives that fault through a two-device
  `NetAlias` instead. Everything else in the model honours `a_net`/`b_net`/`i_net` and is wired to
  `p0..p17` directly.
