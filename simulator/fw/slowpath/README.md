# `fw/slowpath/` -- the directed stimulus for the chip's worst slow-corner paths

This directory holds the *stimulus*; the two tests that run it live
beside it and in `test/`:

| what | where | what it runs on |
|---|---|---|
| the programs, the pin map, the register words, every expected cycle count | `program.pio`, `stimulus.py` | -- |
| the same stimulus on every firmware target | `test_slowpath.py` (pytest) | `FW_TARGET=iss`, `iss_b`, `rtl`, through `iss.pio_iss.ChipDevice` |
| the same stimulus on the Tiny Tapeout pins | `test/test_slowpath.py` (cocotb) | the RTL and the SDF-annotated gate-level netlist (`sim/sdf/run.py --module test_slowpath`) |

## Why these programs

The slow corner (`nom_slow_1p08V_125C`) fails STA by **-9.895 ns** at a 20 ns clock, and
`test/test.py`'s four chip-level tests pass on the SDF-annotated netlist there anyway. The
reason is visible in the numbers: none of them measures a cycle count. Every observation is a
poll of a status line and every assertion is about a value, so a path that settles one core
cycle late cannot show up.

A script outside this repository translated the slow corner's 2 000 reported setup paths from
`_27307_/Q -> _29046_/D` into RTL registers.
All 527 violators come from just **two launch registers**, and the table names both of the
paths they feed:

| launch | endpoint | STA slack at 20 ns | SDF cell delay (typ) | at 20 ns | at 30 ns |
|---|---|---|---|---|---|
| `g_eng[1].u_eng.sm_en` | `g_eng[1].u_eng.u_rxf.mem[40]` | -9.895 ns | 26.211 ns | **misses by 6.211** | fits |
| `g_eng[1].u_eng.sm_en` | `g_eng[1].u_eng.ir[12]` | -9.514 ns | 25.721 ns | **misses by 5.721** | fits |
| `g_eng[0].u_eng.div_en` | `g_eng[0].u_eng.u_rxf.mem[78]` | -5.012 ns | 21.605 ns | **misses by 1.605** | fits |
| `g_eng[0].u_eng.div_en` | `g_eng[0].u_eng.ir[4]` | -4.307 ns | 20.996 ns | **misses by 0.996** | fits |

("cell delay" is what `iverilog` actually waits: the sum of the SDF's own `IOPATH` values along
the path, which is a lower bound -- `INTERCONNECT` is 4.0 to 4.3 ns of the STA total and iverilog
annotates none of it.)

* `sm_en` is the **registered** `CTRL.SM_ENABLE[e]`, so it moves **once**, when the host starts
  the engine. `CLKDIV = 1` makes it the only transition into the engine's datapath, which
  isolates it.
* `div_en` is the divider's `en` flop, so with an integer `CLKDIV = D > 1` it moves every `D`
  core cycles -- **once per SM cycle**. That is the recurring path, and a test that runs its
  engine at `CLKDIV = 1` never touches it.

So both divisors are exercised, 1, 2 and 3, and the stimulus is built around a path that recurs.

## The programs

| program | engine | what it does |
|---|---|---|
| `fetch_probe` | 0 | `wait 1 pin, 0` / `set pins, 1` / `set pins, 0` / `jmp 0` -- a pulse on protocol pin 16 every time the host releases it |
| `rxf_probe` | 1 | `wait 1 pin, 0` / `pull` / `mov isr, osr` / `push` / `jmp 0` -- one word per release into engine 1's RX FIFO, read back afterwards |
| `fetch_probe_late`, `rxf_probe_late` | 0, 1 | the same with one extra `nop` before the answer: the **negative control** |
| `fetch_control` | 0 | `set pins,1` / `set pins,0` / `jmp pin, 5` / `nop` / `nop` / `jmp 0` -- a **taken** JMP when the host's pin is high and a **not-taken** one when it is low, in the same program |
| `anchor` (in `stimulus.py`) | 0 | the `docs/mvp-isa.md` `### Anchor trace` program verbatim, with the documented wrap window 1..2 |

Both wait on a pin **the host owns** (`ui_in[3]` for engine 0, `ui_in[4]` for engine 1). That is
the whole measurement trick: the host knows the exact core cycle it drove the pin, so every
latency is measured from an edge that moves with nothing in the chip. An engine-produced anchor
would not do -- `sm_en -> uo_out[6]` is -9.523 ns at the slow corner, so that anchor would move
with the thing being measured.
## What is measured, and why it is an exact set

The divider's SM cycles sit on one residue class of core cycles modulo `D`, and the host cannot
know which: the CTRL write that set the phase landed at a cycle the host port does not report.
But it can choose its own residue, and a release in core cycle `T` is answered by the first SM
cycle at or after `T + IN_PIPE`, so `T mod D` alone decides which phase answers it. Each sweep
therefore issues **`D` releases, one per phase** (`sp.release_after`), and the test asserts the
whole *set* of latencies the ISA allows -- not one number with a tolerance:

| quantity | value | from |
|---|---|---|
| `fetch_probe`'s answer latency | `{D + 4 .. 2D + 3}` | `wait` completes in SM cycle `S`; `set pins,1` in `S+1` = `D` core cycles later; the level register is written at the end of that cycle and the pad follows `OUT_PIPE = 1` later; `S - T` is one of `IN_PIPE .. IN_PIPE + D - 1` |
| `fetch_probe`'s pulse width | `D` | the two SETs are one SM cycle apart |
| `rxf_probe`'s `rx_valid` latency | `{3D + 5 .. 3D + 4}` | `wait` in `S`, `pull` in `S+1`, `mov isr, osr` in `S+2`, `push` in `S+3`; the FIFO change at the end of cycle `n` shows on `rx_valid` from `n + 2` (`docs/mvp-regmap.md` `### tx_ready and rx_valid`) |
| `fetch_control`'s period | `4D` taken, `6D` not taken | a taken JMP costs one SM cycle like any other instruction (`### Cycle accounting`), so the period is exactly the loop length, which the branch changes by two instructions |
| the anchor trace | the document's own table | `docs/mvp-isa.md` `### Anchor trace`, per-cycle values, aligned on its first transition |

A path that settles one core cycle late moves the whole set by one, which is what makes these
checks sensitive: `test_slowpath_negative_control` measures the mutants and requires exactly
that shift, so a checker that was reading something else -- a constant offset, the length of an
SPI transaction, the divider's phase -- would fail.

## Running it

```sh
# the firmware matrix (fast; the ISS and ISS-B are Python models)
FW_TARGET=iss    .venv/bin/python -m pytest -q fw/slowpath
FW_TARGET=iss_b  .venv/bin/python -m pytest -q fw/slowpath
FW_TARGET=rtl    .venv/bin/python -m pytest -q fw/slowpath

# the chip-level tests on the TT pins, on the RTL
source .venv/bin/activate && make -C test COCOTB_TEST_MODULES=test_slowpath FST=

# the same three tests on the SDF-annotated hardened netlist, at a corner and a core clock
make sdf-slowpath SDF_CORNER=slow SDF_TICK=20     # 3 passed (a failure would have been expected from the STA numbers)
make sdf-slowpath SDF_CORNER=slow SDF_TICK=21     # 3 passed -- the crossover run
make sdf-slowpath SDF_CORNER=slow SDF_TICK=30     # 3 passed
make sdf-slowpath SDF_CORNER=typ SDF_TICK=20      # 3 passed
make sdf-slowpath-matrix                          # all three, one after another
```

## ISA details that are easy to get wrong

## What the ISA settled while this was written

Four things, each of which was a wrong first guess here and is now a comment where it is used:

1. **`EXECCTRL` bit 31 is `latch_pend & latch_host`**, so `EXEC_STALLED` reports a stalled
   *host-forced* instruction only, never one fetched from IMEM. `fetch_probe` is parked on an
   IMEM `wait`, and what proves it is parked is `SMe_ADDR` (a stalled IMEM instruction leaves the
   PC alone) plus the answer pin not moving.
2. **`push` pushes the ISR, not the OSR**, so the word has to be moved across first:
   `pull` then `mov isr, osr` then `push`.
3. **The MVP's `IN` has no destination field** -- it always shifts into the ISR -- so a program
   cannot load a pin into X with `in x, 1`. `fetch_control` branches on `EXECCTRL.JMP_PIN`
   instead, which is the pin condition the ISA does define.
4. **Protocol pins 8..12 are `ui_in[3..7]` and pins 13..17 are `uo_out[3..7]`** (`pin - 5` and
   `pin - 10`), running in *opposite* directions along their ports. `sp.pin_port()` is the one
   place that knows it, and `uo_out[2]` is `rx_valid`, so the PIO output pins are `uo_out[3]` up.
