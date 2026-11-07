# `src/`: the RTL of the protocol emulator

Everything Tiny Tapeout hardens, plus two probe top levels and the retired UART stub. The
chip is `tt_um_protocol_emulator.v` (the TT top level: pads, reset synchroniser, wiring) →
`pio_chip.sv` (register file, pin resolution, two engines) → `pio_engine.sv` (the PIO state
machine). The design of record is `docs/mvp-isa.md`, `docs/mvp-regmap.md` and
`docs/mvp-interfaces.md`; this file only says what is here and how to build it.

## What each file is

| file | what it is |
|---|---|
| `tt_um_protocol_emulator.v` | the TT top level: the 43 TT pins, `rst_n` async-assert/sync-release, `host_spi` + `pio_chip` |
| `pio_chip.sv` | the chip: two `pio_engine`s, the global registers, `T_IN = 3` input pipeline, registered pin level/direction with engine-1-wins resolution |
| `pio_engine.sv` | one engine: IMEM (32 × 16 flops), the IR fetch register, PC/X/Y/OSR/ISR, counters, the EXEC latch, both FIFOs, the 16.8 divider, `SMe_*` |
| `pio_imem_sram.sv` | the same IMEM in an IHP SRAM macro. Selected by `-DIMEM_SRAM`; the flop array is the default |
| `pio_clkdiv.sv`, `pio_fifo.sv` | the divider and the 4-deep FIFO, both instantiated per engine |
| `host_spi.sv` | the SPI host target: `ui_in[2:0]` / `uo_out[0]` → register-bus accesses |
| `config.json`, `info.yaml` → `info.yaml` | TT's project config; `phys/flow.py` overrides both per flow config |
| `lane.sdc` | timing constraints for the gate-level sim |
| `RM_IHPSG13_1P_64x16_c2.v`, `RM_IHPSG13_2P_64x22_c2_bm_bist.v` | port-only blackboxes for the two IHP SRAM macros (single-port probe and two-port IMEM). Synthesis sees these; the simulation builds use the PDK's own behavioural model instead |
| `sram_tester.v`, `tt_um_sram_probe*.v` | the SRAM probe: a macro write/read tester and the two TT top levels it was hardened in |
| `tt_um_lane_probe.v` | the pin-mapping probe |
| `stub_pkg.sv`, `stub_uart_tx.*` | the retired flow stub, kept because `info.yaml` and older flow configs still list it |

## The two IMEM implementations

`pio_engine.IMEM_SRAM` (default `1'b0`) picks where the 32 instruction words live:

* **flops** — `logic [15:0] imem[32]` inside the engine, reset to zero, `ir <= imem[pc_n]` every
  cycle. This is what every default build gets and what `docs/mvp-isa.md` describes.
* **SRAM** — `-DIMEM_SRAM`, `pio_imem_sram.sv`: one `RM_IHPSG13_2P_64x22_c2_bm_bist` per engine,
  port A the fetch (its `A_DOUT` *is* `ir`) and port B the host write port. It has to be a
  two-port macro because the flop IMEM is read and written on the same clock edge, and it has
  no reset, so the program image must be written before `CTRL.SM_ENABLE`.

The two builds are the same design cycle for cycle once the program image is written (a macro has
no reset, so a pin reset does not clear IMEM the way it clears the flop array — see the Reset
bullet in `research/imem-sram/RESULT.md` "## Cycle-exactness"); that file has the argument, the
measurements and what the second build costs.

## How to build and check

```sh
make lint                       # verilator -Wall over the sources info.yaml hardens
make rtl-lint                   # the pio_chip hierarchy alone (quick check)
make test                       # the TT cocotb test of the chip on its pads
make fuzz SEED=1 CYCLES=2000    # RTL/ISS lockstep
make fw PROTO=spi TARGET=rtl    # the firmware on the RTL
```

With `SRAM_IMEM=1` the same four commands build the SRAM variant: the three harness Makefiles
(`verif/fuzz/`, `fw/`, `test/`) add `src/pio_imem_sram.sv`, the macro's port-only blackbox is
replaced by the PDK behavioural model, and `-DIMEM_SRAM -DPIO_SRAM_MODEL` go to the compiler.
`PIO_SRAM_MODEL` is what tells `pio_imem_sram.sv` that the model's array is there to be given
the flop build's power-up value; the synthesis builds compile the blackbox instead and must not
see it. `verilator --lint-only -Wall -Isrc --top-module pio_imem_sram src/pio_imem_sram.sv
src/RM_IHPSG13_2P_64x22_c2_bm_bist.v` is the standalone lint of the module, and the same
command with the PDK model added and `-DPIO_SRAM_MODEL` lints the simulation build.

`info.yaml`'s `source_files` does **not** list `pio_imem_sram.sv` or the blackboxes: the flop
IMEM is the default, and `phys/cfg/mvp-sram.json` lists the SRAM variant's own sources. That
is why the lint above is a separate command rather than part of `make lint`.