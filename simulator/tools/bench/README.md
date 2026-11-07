# tools/bench: evidence bench user guide

## Start here

The evidence bench runs chip questions on several simulators and models and keeps every number as a record (`results/records.jsonl`).

```
python -m tools.bench knobs                                   # every knob and which backend honours it
python -m tools.bench run --backend analytic --workload chip  # run, store and print a table
python -m tools.bench check --all                             # compare levels against each other
python -m tools.bench serve                                   # browser front end (--store X works before or after the command)
make bench-report                                             # quick levels, check, snapshots/latest.html
```

Findings so far: `FINDINGS.md`; the committed result set: `snapshots/committed.jsonl`. Tour by task: section 2.

Commands are
written `python -m tools.bench ...` and need the repository root as the working directory (`./bench ...` works anywhere).
Some examples mention files that are not part of this repository (`docs/`, `research/ideation/`); the
levels `flow`, `gl`, `gl_rig`, `power`, `synth` also need an EDA toolchain (see the top-level README, Optional parts).

The evidence bench runs the same chip questions on several simulators and models, stores every number as a record
with its provenance, and compares levels against each other. Use it to ask "what does config X do under workload Y"
and to see how far each number can be trusted. Contract (normative rules): `CONTRACT.md`. Plan: `PLAN.md`.

## 1. What it is

- **One environment.** One CLI, `python -m tools.bench <command>` (or `make bench ARGS="<command>"`), and one browser
  front end (`serve`) for every level below.
- **Several simulators and models.** An analytic model (archsim, in `tools/archsim/`), software models of the chip core
  (ISS, instruction-set simulator: a fast model that runs the firmware), a Verilog simulation (RTL), synthesis (Yosys
  area), gate-level simulation (gl), the full place-and-route flow (flow), power, and past results (import).
- **One record store.** Every number is a record (section 3) in `tools/bench/results/records.jsonl` (override with
  `--store` or `BENCH_STORE`). The front end and the write-up read the same store.
- **Config and workload are the inputs.** A **config** is a set of knob values, one per chip choice. The **MVP** is the
  chip as built: 2 engines (PIO blocks, each with its own state machines and program memory), 32-word program memory per engine (flop), FIFO depth 4 (the 32-bit
  queues between engines and host), 50 MHz core clock, SPI host link. A **workload** is what runs: a protocol at a
  speed, a bridge (one protocol's input feeding another's output), a mix (several protocols at once), or the whole chip.
- **Cross-level checks.** The same quantity from two levels is compared (`check`). A disagreement is a finding about a
  model, not a bug to hide (section 3).

Other terms: **SM cycle** = one state-machine clock tick; **cap19** = the 19 protocols tagged `cap19` in
`tools/archsim/data/inputs.json`; **flow** = LibreLane place and route; **VCD** = a waveform file used as power input.

### Levels

| level | what it is | what it measures | speed | knobs it honours |
|---|---|---|---|---|
| `analytic` | archsim formulas over the chip description | protocol verdict (served / marginal / impossible), margin, binding limit; chip area, clock margins, pins | under 1 s for all 58 workloads | F: f_core_mhz, n_engines, imem_tech, fifo_depth, host_link, lane, die, isa_*. P: imem_words (16 refused). -: t_in, engine_route |
| `iss` | golden ISS (`iss/pio_iss.py`, the verified reference) | rate and timing of the shipped firmware (max_bit_rate, sm_cycles_per_bit, pin response) | about 0.5 s per protocol | MVP only |
| `iss_b` | second, independent ISS (`verif/iss_b/`) | the same metrics as `iss`; agreement is checked | about 0.5 s per protocol | MVP only |
| `iss_p` | parametric ISS (`tools/bench/sim/`) | as `iss`, plus bridges and mixes; FIFO depth and engine routing are real here | about 8 s per bridge run at 115200 baud | F: fifo_depth, engine_route. P: n_engines, imem_words, t_in |
| `rtl` | the Verilog design, simulated cycle by cycle (Verilator) | the same metrics as `iss`, from the hardware | spi smoke about 6 s with the build; other protocols long | F: imem_tech, t_in, n_engines, imem_words, fifo_depth, engine_route (built into the Verilator model; `RTL_KNOBS` in `backends/sim.py`). Others MVP only |
| `synth` | Yosys synthesis, no place and route | area (standard cells, macros), cell counts | 19 to 25 s for the chip | F: t_in, n_engines, imem_words, fifo_depth, engine_route, imem_tech |
| `gl` | gate-level simulation of the hardened netlist, with the chip's cocotb tests | pass / fail on the post-layout netlist | 92 to 151 s (long) | F: f_core_mhz, t_in, imem_tech. n_engines, imem_words, fifo_depth, engine_route: MVP value only (test/test.py speaks the MVP register map) |
| `flow` | full LibreLane place and route | area, fmax (typical and slow), DRC and LVS counts | about 3.5 h (long; timeout 6 h) | F: f_core_mhz, t_in, n_engines, imem_words, fifo_depth, engine_route, imem_tech |
| `power` | OpenROAD `report_power` on gate-level activity | power at three corners, split into clock, sequential, combinational, leakage | 36 s per workload once a flow exists (long) | F: f_core_mhz, imem_tech. P: t_in; n_engines, imem_words, fifo_depth, engine_route (off the MVP register map only `idle` runs) |
| `gl_rig` | the perf rigs (spi, swd, ps2, onewire) on the post-layout netlist, internal bus forced | `rtl`'s metric names at level `gl`; a mismatch with `rtl` is a netlist finding | 4 s (spi, swd) to 12 min (onewire) per rig (long) | F: t_in, n_engines, imem_words, fifo_depth, engine_route, imem_tech: any config with a finished flow |
| `iss_isa` | instruction-set variants measured on the parametric ISS (`backends/isa.py`, `tools/bench/sim/ISA_EXT.md`) | program words, cycles per bit and IMEM margin of the xor / cond / irq variants (workloads can_1m, i2c_target_fmplus, uart_3m) | under 1 s | P: isa_* and imem_words only |
| `import` | past evidence: `metrics/*.csv`, `research/perf/`, `RESULT.md` | the numbers those files record (method `imported`, or `derived`) | 2 s for all 6475 records | none |

Speeds are from this guide's runs unless marked otherwise. `fake` and `fake2` are test backends; their numbers are
fixed and are not evidence. F = full, P = partial (reasons below the table), - = MVP only.

Every registered backend (`tools/bench/backends/__init__.py`), in one line each:

- `analytic`: archsim formulas (`tools/archsim/`); instant, mostly estimates and identities.
- `iss`, `iss_b`: the two independent reference instruction-set simulators running the shipped firmware (MVP only).
- `iss_p`: the parametric ISS in `tools/bench/sim/`; honours FIFO depth, engine count, routing, and runs bridges and mixes.
- `iss_isa`: measured effects of instruction-set variants (xor, conditional jump, irq) on the parametric ISS.
- `rtl`: the Verilog design under Verilator through the fw cocotb harness; a handful of protocol rigs.
- `synth`: Yosys synthesis area, no place and route.
- `flow`: the full LibreLane place and route (hours); its finished run is the input of `gl`, `gl_rig` and `power`.
- `gl`: `test/test.py` (the chip's 4 cocotb tests) on the hardened netlist **through the real host SPI pins**, so it
  covers the host decode, but speaks the MVP register map only.
- `gl_rig`: the perf rigs on the netlist with the internal bus **forced** (host SPI decode bypassed), for any config
  with a finished flow, including ones off the MVP register map.
- `power`: OpenROAD `report_power` on gate-level activity (needs a finished flow).
- `import`: past evidence files, unchanged.
- `fake`, `fake2`: test backends.

`gl` and every `power` workload except `idle` refuse configs off the MVP register map (`n_engines`, `imem_words`,
`fifo_depth`, `engine_route` at non-default values: their testbenches' host writes would miss and measure an idle
chip); use `gl_rig` for gate-level checks there.

Backend file names: `physical.py` = `synth` + `flow`; `gatelevel.py` = `gl`; `gl_rig.py` = `gl_rig`; `sim.py` =
`iss`/`iss_b`/`rtl`; `sim_p.py` = `iss_p`; `isa.py` = `iss_isa`. `tools/bench/sim/` is the parametric ISS itself;
`tools/bench/bridges/` holds the `.pio` programs used by `bridges.py`.

Knob support, from the code (`python -m tools.bench knobs`):

```
knob          kind   default  domain             unit          analytic                                                                            fake     fake2    flow     gl                                                      gl_rig   import   iss      iss_b    iss_isa                                                       iss_p                                                                            power                                                                                           rtl      synth
------------  -----  -------  -----------------  ------------  ----------------------------------------------------------------------------------  -------  -------  -------  ------------------------------------------------------  -------  -------  -------  -------  ------------------------------------------------------------  -------------------------------------------------------------------------------  ----------------------------------------------------------------------------------------------  -------  -------
f_core_mhz    float  50.0     10/80              MHz           full                                                                                full     full     full     full                                                    default  default  default  default  default                                                       default                                                                          full                                                                                            default  default
t_in          int    3        2/4                cycles        default                                                                             full     full     full     full                                                    full     default  default  default  default                                                       partial: a pipeline depth in the ISS; protocol firmware assumes T_IN=3           partial: the activity firmware is the MVP's; power is measured, not asserted correct            full     full
n_engines     int    2        1/4                              full                                                                                full     full     full     default_only: test/test.py speaks the MVP register map  full     default  default  default  default                                                       partial: protocol rigs use engines 0-1; mixes need 2-3                           partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)  full     full
imem_words    enum   32       16/32/64/128       16-bit words  partial: values outside archsim's domain (imem_words 16) are refused by supports()  full     full     full     default_only: test/test.py speaks the MVP register map  full     default  default  default  partial: only sets the IMEM size the words are compared with  partial: the shipped firmware is up to 29 words, so 16 fails for most protocols  partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)  full     full
imem_tech     enum   flop     flop/sram2p                      full                                                                                default  default  full     full                                                    full     default  default  default  default                                                       default                                                                          full                                                                                            full     full
fifo_depth    int    4        2/16               32-bit words  full                                                                                full     full     full     default_only: test/test.py speaks the MVP register map  full     default  default  default  default                                                       full                                                                             partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)  full     full
engine_route  bool   False                                     default                                                                             full     full     full     default_only: test/test.py speaks the MVP register map  full     default  default  default  default                                                       full                                                                             partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)  full     full
host_link     enum   spi      spi/spi_fast/qspi                full                                                                                full     full     default  default                                                 default  default  default  default  default                                                       default                                                                          default                                                                                         default  default
lane          bool   False                                     full                                                                                default  default  default  default                                                 default  default  default  default  default                                                       default                                                                          default                                                                                         default  default
die           enum   6x4      6x4/8x4                          full                                                                                full     full     default  default                                                 default  default  default  default  default                                                       default                                                                          default                                                                                         default  default
isa_xor       bool   False                                     full                                                                                default  default  default  default                                                 default  default  default  default  full                                                          default                                                                          default                                                                                         default  default
isa_cond      bool   False                                     full                                                                                default  default  default  default                                                 default  default  default  default  full                                                          default                                                                          default                                                                                         default  default
isa_irq       bool   False                                     full                                                                                default  default  default  default                                                 default  default  default  default  full                                                          default                                                                          default                                                                                         default  default

analytic.imem_words: partial: values outside archsim's domain (imem_words 16) are refused by supports()

iss_isa.imem_words: partial: only sets the IMEM size the words are compared with

iss_p.t_in: partial: a pipeline depth in the ISS; protocol firmware assumes T_IN=3

iss_p.n_engines: partial: protocol rigs use engines 0-1; mixes need 2-3

iss_p.imem_words: partial: the shipped firmware is up to 29 words, so 16 fails for most protocols

power.t_in: partial: the activity firmware is the MVP's; power is measured, not asserted correct

power.n_engines: partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)

power.imem_words: partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)

power.fifo_depth: partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)

power.engine_route: partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)
```

Workloads per backend (`python -m tools.bench workloads` lists each with its parameters):

| backend | count | workloads |
|---|---|---|
| analytic | 58 | the 19 cap19 protocols, the 15 `fw_*` firmware protocols, `chip`, `spi_host_port`, others |
| iss, iss_b | 15 each | spi, i2c, jtag, swd, ps2, onewire, dmx512, wiegand, quadrature, necir, uart_tx, uart_rx, manchester, ws2812, spi_host_port |
| iss_p | 20 | the 15 above, 3 bridges (`bridge.*`), 2 mixes (`mix.*`) |
| rtl, power | 5 each | rtl: spi, swd, ps2, onewire, spi_host_port. power: idle, uart_tx, i2c, both_engines, host_spi_burst |
| gl_rig | 4 | spi, swd, ps2, onewire |
| iss_isa | 3 | can_1m, i2c_target_fmplus, uart_3m |
| synth, flow, gl, import | 1 each | chip, chip, chip, all |

## 2. Five-minute tour, by task

Each example is the command and output from this guide's runs (trimmed where the text says so). The runs used
`BENCH_STORE=/tmp/my-store/records.jsonl`; leave the variable unset for the repo default. An identical
repeat run is served from the cache (`[cache_hit]`). **The cache belongs to a directory, not to a store file:** it is
`cache/` next to `records.jsonl`, so two store files in one directory share it (a "fresh" `tour.jsonl` beside an old
`records.jsonl` still prints `[cache_hit]`). For a clean start give each store its own directory, or add `--no-cache`.

### 2.1 What can the MVP do?

The MVP against the analytic model (chip, then four protocols), the 19 cap19 protocols (a tally: 4 served,
15 impossible, 0 refused), and four protocols under the golden ISS (`max_bit_rate` rows shown).

```
$ python -m tools.bench run --backend analytic --workload chip,can_1m,spi_target_10m,swd_host_10m
id            workload        metric                  value                         unit  level     backend   method    config
404badb1346e  chip            chip_verdict            pass                                analytic  analytic  estimate  MVP
7c8dd573bed9  chip            area_um2                342472 [384681..295536]       um2   analytic  analytic  estimate  MVP
b628ee7962ad  chip            area_utilization_pct    37.9505 [42.6278..32.7493]    %     analytic  analytic  estimate  MVP
db50e0b52c71  chip            fmax_mhz                61.6 [61.6..61.6]             MHz   analytic  analytic  estimate  MVP
ffb5b8a0a674  chip            die_area_margin_um2     289220 [156769..408350]       um2   analytic  analytic  estimate  MVP
99a78f4ab565  chip            core_clock_margin_mhz   11.6 [11.6..11.6]             MHz   analytic  analytic  estimate  MVP
b1eaddd1c081  chip            board_clock_margin_mhz  16.5 [16.5..16.5]             MHz   analytic  analytic  derived   MVP
448cdd2316b9  chip            pins_left               18 [18..18]                   pins  analytic  analytic  derived   MVP
868ce269410f  chip            pins_fixed              6                             pins  analytic  analytic  derived   MVP
1c3ebe717228  can_1m          verdict                 impossible                          analytic  analytic  derived   MVP
ca095121f8ff  can_1m          binding_limit           isa_xor                             analytic  analytic  derived   MVP
1c9ddb09eb9f  can_1m          margin                  -1 [-1..-1]                   flag  analytic  analytic  derived   MVP
$ python -m tools.bench run --backend iss --workload uart_tx,spi,i2c,jtag
id            workload                 metric                        value        unit                      level  backend  method    config
0876939304b6  uart_tx:speed=6250000.0  max_bit_rate                  6.10687e+06  bit/s                     iss    iss      measured  MVP
5989fcae3cab  spi:speed=1.0            max_bit_rate                  6.00601e+06  bit/s                     iss    iss      measured  MVP
4a79c219dcac  i2c:speed=13.0           max_bit_rate                  320999       bit/s                     iss    iss      measured  MVP
1793e3980058  jtag:speed=1.0           max_bit_rate                  3.83142e+06  bit/s                     iss    iss      measured  MVP
```

`chip_verdict pass` with `core_clock_margin_mhz` 11.6 means the chip fits the MVP budget on the model. `impossible`
with a negative `margin` names the limit that broke (`binding_limit`): `can_1m` needs the XOR operation (`isa_xor`).

### 2.2 Does the analytic model agree with simulation?

Run the same protocols on both sides, import the perf-b results, then compare. The analytic ids `fw_*` join the
simulator names through `analytic.ANALYTIC_TO_SIM`.

```
$ python -m tools.bench run --backend analytic --workload fw_uart_tx,fw_spi_m0,fw_i2c,fw_jtag
$ python -m tools.bench import
1 spec(s), 6475 record(s), 0 error(s)
$ python -m tools.bench check --metric max_bit_rate --all
verdict    workload                     metric        a                                     b                                   detail
---------  ---------------------------  ------------  ------------------------------------  ----------------------------------  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
off        jtag:speed=1.0               max_bit_rate  analytic/analytic=11111122.222233308  iss/iss=3831420.0                   bands [1.11111e+07, 1.11111e+07] and [3.83142e+06, 3.83142e+06] do not overlap
off        jtag:speed=1.0               max_bit_rate  analytic/analytic=11111122.222233308  import/import:perf_b:iss=3831420.0  bands [1.11111e+07, 1.11111e+07] and [3.83142e+06, 3.83142e+06] do not overlap
match      jtag:speed=1.0               max_bit_rate  iss/iss=3831420.0                     import/import:perf_b:iss=3831420.0  equal
identity   spi:speed=1.0                max_bit_rate  analytic/analytic=6250006.249999998   iss/iss=6006010.0                   analytic/analytic carries an archsim input unchanged (judgement inputs: param:t_mcu_txn_ns); not a model prediction
identity   spi:speed=1.0                max_bit_rate  analytic/analytic=6250006.249999998   import/import:perf_b:iss=6006010.0  analytic/analytic carries an archsim input unchanged (judgement inputs: param:t_mcu_txn_ns); not a model prediction
identity   spi:speed=1.0                max_bit_rate  analytic/analytic=6250006.249999998   import/import:perf_b:rtl=6006010.0  analytic/analytic carries an archsim input unchanged (judgement inputs: param:t_mcu_txn_ns); not a model prediction
match      spi:speed=1.0                max_bit_rate  iss/iss=6006010.0                     import/import:perf_b:iss=6006010.0  equal
match      spi:speed=1.0                max_bit_rate  iss/iss=6006010.0                     import/import:perf_b:rtl=6006010.0  equal
match      spi:speed=1.0                max_bit_rate  import/import:perf_b:iss=6006010.0    import/import:perf_b:rtl=6006010.0  equal
off        uart_tx:speed=6250000.0      max_bit_rate  analytic/analytic=6250006.250006249   iss/iss=6106870.0                   bands [6.25001e+06, 6.25001e+06] and [6.10687e+06, 6.10687e+06] do not overlap
off        uart_tx:speed=6250000.0      max_bit_rate  analytic/analytic=6250006.250006249   import/import:perf_b:iss=6106870.0  bands [6.25001e+06, 6.25001e+06] and [6.10687e+06, 6.10687e+06] do not overlap
match      uart_tx:speed=6250000.0      max_bit_rate  iss/iss=6106870.0                     import/import:perf_b:iss=6106870.0  equal
identity   i2c:speed=13.0               max_bit_rate  analytic/analytic=400000.3999999995   iss/iss=320999.0                    analytic/analytic carries an archsim input unchanged (input copied from archsim rate_ceiling_hz (the protocol ceiling binds: its value is perf-a); not a model prediction
identity   i2c:speed=13.0               max_bit_rate  analytic/analytic=400000.3999999995   import/import:perf_b:iss=320999     analytic/analytic carries an archsim input unchanged (input copied from archsim rate_ceiling_hz (the protocol ceiling binds: its value is perf-a); not a model prediction
match      i2c:speed=13.0               max_bit_rate  iss/iss=320999.0                      import/import:perf_b:iss=320999     equal
match      swd:speed=9.0                max_bit_rate  import/import:perf_b:iss=5245230.0    import/import:perf_b:rtl=5245230.0  equal
match      ps2:speed=94.0               max_bit_rate  import/import:perf_b:iss=10390.6      import/import:perf_b:rtl=10390.6    equal
match      onewire:speed=47.0           max_bit_rate  import/import:perf_b:iss=13004.1      import/import:perf_b:rtl=13004.1    equal
one-sided  dmx512:speed=255000.0        max_bit_rate  import/import:perf_b:iss=255102       -                                   only import/import:perf_b:iss has it
one-sided  wiegand:speed=20000.0        max_bit_rate  import/import:perf_b:iss=4166.67      -                                   only import/import:perf_b:iss has it
one-sided  quadrature:speed=112000.0    max_bit_rate  import/import:perf_b:iss=32653100.0   -                                   only import/import:perf_b:iss has it
one-sided  necir:speed=7.0              max_bit_rate  import/import:perf_b:iss=1031.87      -                                   only import/import:perf_b:iss has it
one-sided  uart_rx:speed=8333330.0      max_bit_rate  import/import:perf_b:iss=8333330.0    -                                   only import/import:perf_b:iss has it
one-sided  manchester:speed=12000000.0  max_bit_rate  import/import:perf_b:iss=8064520.0    -                                   only import/import:perf_b:iss has it
one-sided  ws2812:speed=1.0             max_bit_rate  import/import:perf_b:iss=1250000.0    -                                   only import/import:perf_b:iss has it
one-sided  spi:speed=1.0                max_bit_rate  rtl/rtl=6006010.0                     -                                   only rtl/rtl has it

match: 9  close: 0  off: 4  identity: 5  one-sided: 8
$ python -m tools.bench check --strict; echo $?
match: 84  close: 0  off: 4  identity: 6  one-sided: 5169
1
```

The last rows are `one-sided` (8 here, 5169 in the whole store: imported verdicts with no simulator counterpart). Reading the verdicts (rules in section 3):

- `match`: equal values. `iss` and imported perf-b agree to the digit for jtag and spi.
- `off`: the analytic max rates for jtag (11.1 MHz) and uart_tx (6.25 MHz) differ from the simulators (3.8, 6.1 MHz).
  archsim gives the steady-state rate; perf-b counts frame gaps and other bit conventions. These are allowlisted with
  that reason in `tools/bench/smoke_allow.json`. An `off` is a result about a model, not a tuning target.
- `identity`: one side is an archsim input copied through (`rate_ceiling_hz`), so agreement proves nothing. spi and i2c are identities.
- `close` (within tolerance, or overlapping bands) does not appear in this run. `--strict` exits 1 on any `off`.

### 2.3 Sweep FIFO depth for a bridge

The bridge `uart_rx->uart_tx` takes 22 bytes back to back at 115200 baud and forwards them at 9600 baud, through
engine 0's RX FIFO and engine 1's TX FIFO (`engine_route=true`). Sweep the depth (30 records; the comparisons are the summary):

```
$ python -m tools.bench sweep --vary fifo_depth=4,8,16 --set engine_route=true --workload 'bridge.uart_rx->uart_tx:burst=22' --backend iss_p
$ python -m tools.bench compare --metric bytes_lost
config                            workload                                                                      iss/iss_p
--------------------------------  ----------------------------------------------------------------------------  ---------
engine_route=True, fifo_depth=16  bridge.uart_rx->uart_tx:burst=22:gap=0:in_rate=115200:out_rate=9600:search=0  0
engine_route=True, fifo_depth=8   bridge.uart_rx->uart_tx:burst=22:gap=0:in_rate=115200:out_rate=9600:search=0  3
engine_route=True                 bridge.uart_rx->uart_tx:burst=22:gap=0:in_rate=115200:out_rate=9600:search=0  11
$ python -m tools.bench compare --metric peak_buffer_words
config                            workload                                                                      iss/iss_p
--------------------------------  ----------------------------------------------------------------------------  ---------
engine_route=True, fifo_depth=16  bridge.uart_rx->uart_tx:burst=22:gap=0:in_rate=115200:out_rate=9600:search=0  20
engine_route=True, fifo_depth=8   bridge.uart_rx->uart_tx:burst=22:gap=0:in_rate=115200:out_rate=9600:search=0  16
engine_route=True                 bridge.uart_rx->uart_tx:burst=22:gap=0:in_rate=115200:out_rate=9600:search=0  8
```

At depth 4 the route holds 8 words and 11 of 22 bytes are lost; at depth 8, 3; at depth 16, none. Losses are whole
bytes. `sustained_rate_no_loss` needs `search=1`. Bridge and mix definitions: docstring of `tools/bench/bridges.py`.

### 2.4 Where did this number come from? (`trace`)

Take a record id from any table. **Ids are per run:** an id is a hash of the whole record including
`provenance.started`, so a real re-run of the same spec gives new ids (a cached run keeps the old ones). An id you
saved earlier still resolves while its record is in the store, but it is then the older record. To find the latest
record for a key, use `compare --metric M --workload W` (or the Compare tab; the last record wins per cell, and
`compare --json` carries each cell's `id`), then `trace` that id.

`trace` shows what was measured and how, the note, the provenance, and the command that reproduces it.

```
$ python -m tools.bench trace 7c8dd573bed9
record   7c8dd573bed9
what     chip / area_um2 = 342472 [384681..295536] um2
how      level analytic, backend analytic, method estimate, status ok
note     input copied from flow (archsim K10: pnr_factor is fitted to the MVP flow area, so the MVP agrees by construction; other configs are predictions); judgement inputs: block:host_spi, block:pinmux_2, param:macro_halo
config   MVP  (hash a20770391769cb40)
provenance
  command    python -m tools.bench run --backend analytic --workload chip
  commit     4628644febec8ae59a01cb6bdbef67ab60ce499b
  dirty      False
  duration_s 0.0019
  input_hashes {'archsim-model.js': 'a8d062ec95cadae5', 'inputs.json': '16abf4069d0d292b'}
  started    2026-10-31T01:21:25
reproduce  python -m tools.bench run --backend analytic --workload chip
```

The note says the MVP area is copied from the flow (archsim K10 fits its place-and-route factor to it), so the MVP
agreement holds by construction. `method estimate` means a judgement input was used (here `block:host_spi`,
`block:pinmux_2`, `param:macro_halo`).

### 2.5 Area of a variant

Synthesis gives the area as Yosys sees it. Compare the MVP with the variant that uses the IHP two-port SRAM macro for
program memory (`imem_tech=sram2p`), next to the analytic model.

```
$ python -m tools.bench run --backend synth --workload chip
$ python -m tools.bench run --backend synth --workload chip --set imem_tech=sram2p
$ python -m tools.bench run --backend analytic --workload chip --set imem_tech=sram2p
$ python -m tools.bench compare --metric area_um2 --workload chip
config            workload  analytic/analytic        synth/synth
----------------  --------  -----------------------  -----------
imem_tech=sram2p  chip      321431 [373323..276569]  260078
MVP               chip      342472 [384681..295536]  265057
```

The sram2p variant has 78,768 um2 of macro, and its standard cells are 181,000 um2 against 265,000 um2 for the MVP.
Synth leaves out the buffers and tap cells the full flow adds: the flow reports 342,960 um2 for the MVP
(`metrics/flow.csv`), 29 % above synth. Compare synth with synth, and the analytic model with the flow.

### 2.6 Open the front end

```
$ python -m tools.bench serve --port 8765      # prints the URL; leave it running (Ctrl-C stops)
  open http://127.0.0.1:8765    (Configure, Results, Compare, Trace, Help)
$ curl -s -o /dev/null -w "%{http_code}" http://127.0.0.1:8765/    -> 200  (/api/knobs, /api/records, /api/trace/<id> also 200)
$ python -m tools.bench export --html OUT.html
wrote OUT.html (6622 records)
```

The server binds to 127.0.0.1 and rejects Host or Origin headers other than 127.0.0.1 or localhost. Configure launches
runs and sweeps; Results, Compare and Trace read the store. `export --html` writes one self-contained page with
Configure disabled; `--snapshot FILE` exports a committed snapshot instead. A client that disconnects
mid-response (for example `curl ... | head`) is logged as one short line.

### 2.7 Long runs

`flow`, `gl` and `power` run only with `--allow-long`. Without the flag a spec is skipped and listed with its reason
(RTL specs other than the spi smoke are long too).

```
$ python -m tools.bench run --backend flow,gl --workload chip
2 spec(s), 0 record(s), 0 error(s)

skipped (2):
backend  workload  reason
-------  --------  ----------------------------------
flow     chip      long run (flow); pass --allow-long
gl       chip      long run (gl); pass --allow-long
$ python -m tools.bench run --backend power --workload idle --allow-long
id            workload  metric         value     unit  level  backend  method    config
------------  --------  -------------  --------  ----  -----  -------  --------  ------
ee2ccfd4ca70  idle      power_mw_typ   8.7083    mW    power  power    measured  MVP
c5e1532b675f  idle      power_mw_slow  6.908     mW    power  power    measured  MVP
187fba9c7358  idle      power_mw_fast  10.8299   mW    power  power    measured  MVP
f6b678f59d26  idle      seq_mw         6.3295    mW    power  power    measured  MVP
1577400e6ba6  idle      comb_mw        0.0033    mW    power  power    measured  MVP
dc059b5f6988  idle      clock_tree_mw  2.3755    mW    power  power    measured  MVP
926299c31988  idle      leakage_mw     0.037178  mW    power  power    measured  MVP
40097d14c7b3  idle      macro_mw       0         mW    power  power    measured  MVP
$ python -m tools.bench run --backend gl --workload chip --allow-long
id            workload  metric           value  unit   level  backend  method    config
------------  --------  ---------------  -----  -----  -----  -------  --------  ------
204bb5203607  chip      gl_tests_passed  4      tests  gl     gl       measured  MVP
2361abec9dd6  chip      gl_tests_failed  0      tests  gl     gl       measured  MVP
bca8790fec02  chip      gl_sim_seconds   92.1   s      gl     gl       measured  MVP
$ python -m tools.bench run --backend flow --workload chip --allow-long      (about 3.5 h; not run for this guide)
```

`power`: 36 s per workload once a finished flow exists for the config; without one the spec is refused ("run the flow
backend for this config first"). `gl`: 92 to 151 s over several runs (machine load). `flow`: about 3.5 h here (`flow.csv` mvp-slow: 13355 s),
timeout 6 h. Power and gl hash the flow's artifacts, so a re-run flow invalidates their cache. A crashed flow stores
one `run_error` record and no partial results.

#### `gl_rig`: the protocol rigs on the netlist

`gl` runs `test/test.py`, which speaks the MVP register map only, so a config off that map (e.g. `n_engines=3`) had no
gate-level check. `gl_rig` runs the same perf rigs as `rtl` (spi, swd, ps2, onewire) with the config's finished flow
netlist behind the fw harness instead of `src/*.sv`: Icarus, the PDK io/udp/stdcell models (plus the SRAM macro model for
`imem_tech=sram2p`), `-DGL_TEST -DFUNCTIONAL -DSIM`. It emits `rtl`'s metric names at level `gl`, so
`python -m tools.bench check --all` compares gl_rig, rtl and iss_p; same design, so they should agree cycle for cycle.
A mismatch or a failed rig is a netlist (or flow) finding; the harness side is `fw/gl_wrap.py`, which forces the netlist's
bus nets and reads host reads from host_spi's capture register (the host SPI port itself is idle, so `spi_host_port` is
not offered). Only FIFO levels and FDEBUG are visible inside the flat netlist, which is all the rigs need.

What it verifies: the whole chip core (engines, FIFOs, register file, pin logic, reset synchronizer, real pad
cells) as placed and routed. What it bypasses: `host_spi`'s SPI decode, because the bus nets are forced. So on a
wide-map config (`n_engines=3`: 10-bit host address, 2-byte command) the host address path has no gate-level coverage
at all; the MVP and sram2p get it from the `gl` backend (`test/test.py`). Like `rtl`, `gl_rig` zeroes the SRAM macro
arrays at start as a simulation convention; on silicon the IMEM is undefined until written. The build dir is keyed on
the config hash and the netlist's content hash, so a re-run flow gets a fresh build. `gl_rig` and `rtl` refuse to share
one process (the target install is process-global).

```
$ python -m tools.bench run --backend gl_rig --workload spi,swd --allow-long        # also --set n_engines=3 ...
```

Cost (measured with 2 jobs in parallel on a loaded 12-core machine; Icarus on the 4 to 6 MB netlist, one
`fw/sim_build/gl-<config hash>` per config, built on first use): spi and swd 4 to 11 s, ps2 2.5 to 8 min, onewire 4 to 12
min (MVP 283 s and 401 s; the 3-engine config 491 s and 695 s), about 1.1 to 2 times the same rig on `rtl`. Needs a finished
flow for the config, like `gl` and `power`. Measured agreement: spi, swd, ps2, onewire on the MVP, `imem_tech=sram2p`
and `n_engines=3 fifo_depth=8 engine_route=true`: every metric equal to `rtl` (and to `iss_p` where it models the config).

## 3. How to read results

**Store, cache, ids.** Records are appended to the store file (`--store` / `BENCH_STORE`). `--store X` works before or after the subcommand (`python -m tools.bench --store X serve` or `serve --store X`). The run cache is the
`cache/` directory beside it, shared by every store file in that directory. Record ids change when a run really
re-executes (see 2.4); to get the newest value for a key use `compare`, not an old id.

**Record fields.** `workload` (id, plus parameters when not default), `metric` (snake_case, the same at every level),
`value`, `unit`, `level`, `backend` (for example `iss_p`, or `import:perf_b:iss`), `method`, `status`, `band`,
`config` and `config_hash` (`MVP` = the default), `workload_params`, `note`, `provenance` (`command`, `commit`, `dirty`,
`started`, `duration_s`, `input_hashes` or `source`), and `run_key`. `id` (CLI and front end) is the first 12 hex characters
of a hash of the record; `trace` accepts it or any prefix.

| method | meaning |
|---|---|
| `measured` | a simulator or tool produced the value |
| `derived` | a formula over measured or fixed inputs |
| `estimate` | a judgement input was used, or the tool cannot observe the value. A model value |
| `spec` | from a specification, not computed |
| `imported` | taken unchanged from a past result file |

| status | meaning |
|---|---|
| `ok` | a value or verdict |
| `fail` | the run worked and the chip failed the check it was given (a bridge that loses bytes, a checker that rejects a setting) |
| `n/a` | nobody observed it: `value` is null, `note` says why. Never 0 |
| `error` | the run broke (crash, timeout, unreadable output, unknown metric). Never cached or cross-checked |

**Bands and margins.** `band: [pessimistic, optimistic]` shows as `value [pess..opt]` (for area the pessimistic value is
the larger). `margin` is the slack of the binding limit: negative means a miss.

**Check verdicts.** Joins use `(config_hash, workload, workload_params, metric)`; only `ok` records with a value take part.

| verdict | rule |
|---|---|
| `match` | equal values |
| `close` | not equal, within tolerance (2 % relative for rates; exact for cycle, tick, count and verdict metrics; 5 % for `margin`), or overlapping bands |
| `off` | outside tolerance: a result about a model, allowlisted in `smoke_allow.json` with its reason when known |
| `identity` | one side was handed the number (an archsim input copied through). Not a check |
| `one-sided` | only one side has it, or the other side is n/a (the note gives the reason). Not a disagreement |

Compare shows, per row, the worst verdict over all pairs (`off` > `close` > `identity` > `match` > `one-sided`).

## 4. Adding things

Contract rules: `CONTRACT.md`. After any change run `make bench-test` (under 3 minutes); `make bench-test-long` runs the
long tests; `make bench-smoke` runs the end-to-end set (about 8 s with a fresh store, including the RTL build; about 5 s with everything cached). `make bench-report` runs the quick levels (analytic, iss/iss_b/iss_p, bridges, mixes, synth MVP and sram2p), `check`, writes `tools/bench/snapshots/latest.html` and the snapshot `latest`, and prints a summary (about 1.5 minutes cold).

- **A knob.** Add a `Knob(...)` to `KNOBS` in `tools/bench/knobs.py`. That file and `types.py` are the contract; change them deliberately.
  Every backend must then answer for the id in `knob_support()` (`full`,
  `default_only`, or `partial: <why>`), and `supports()` must refuse unsupported non-default values. Config hashes cover
  all knobs, so a new knob changes every config hash: re-run stored evidence, or snapshot first, before comparing.
- **A backend.** A class with `name`, `level` (one of `types.LEVELS`), `supports(config, workload)` (None or a reason),
  `knob_support()`, `workloads()` (a list of `Workload(id, kind, params, label)`), `run(spec) -> list[Record]`, and
  `code_hash()`. Optional: `is_long(spec)` (True means `--allow-long` is needed) and `input_hash(spec)` (a hash of external
  files the run reads, such as a finished flow). Register it with one line in `tools/bench/backends/__init__.py`
  (`REGISTRY`, imported lazily). `run()` executes in a worker process: it must not write the store or change the tree.
  Tests go in `tools/bench/tests/test_<area>.py`.
- **A workload.** Return it from `workloads()`. Parameters are part of the join key, so level-specific tuning belongs in the
  config. On the command line: `id` or `id:param=value:param=value`. Workloads a backend does not offer are refused.
- **A metric.** Add `_add(unit, description, name)` to `tools/bench/metrics.py` in your own section, at the end. Never
  rename or reorder: the same quantity must carry the same name at every level. If the value does not depend on the
  speed setting, add it to `SPEED_INDEPENDENT`. Unknown names are rejected by the runner.

## 5. Known limits

- **Analytic predictions are narrow.** Real predictions: 7 `max_bit_rate` values (uart_tx, jtag, swd, manchester,
  manchester_even, dmx512, necir: the cycles limit binds) and 2 host-port windows (`write`, `txf0_burst_4`, weak: calibrated
  on perf-a and perf-b). The other 8 rates are identities. Reflex response and FIFO headroom are not modelled.
- **Chip area is fitted.** Archsim's place-and-route factor is fitted to the MVP flow (K10): MVP agreement holds by
  construction. Other configs are predictions.
- **Analytic has no `t_in` or `engine_route`.** `imem_words=16` is refused (archsim's domain starts at 32).
- **`fifo_headroom_at_max_rate` is not a rate-driven demand measure.** The perf rigs read the active `fifo_depth` (drivers and
  `verif/perf/probe.py`), but they preload up to `fifo_depth` words with the engine stopped and use a fixed payload, so it is
  the words free at the rig transaction's deepest point: 0 at depth <= 4, bounded by the payload size at 8/16. FLEVEL nibbles
  saturate at 15, so drivers wait on `min(fifo_depth, 15)` at depth 16. Pass/fail and rates respond to depth through back-pressure.
- **cocotb builds the RTL simulator against whichever `python3` is first on `PATH`.** The bench runner puts its own
  interpreter's bin directory first for its workers, so `bench run` is safe. Running the fw/verif makefiles directly with
  `/usr/bin/python3` first leaves a `fw/sim_build/verilator` that dies with `No module named pygpi` (the harness reports
  `EOFError ... the simulator died`); delete `fw/sim_build/verilator` to rebuild.
- **Host-port cost metrics are n/a on the wide register map** (more than 2 engines or more than 32 program words).
- **Rigs use engines 0 and 1;** `imem_words=16` fails for most protocols (firmware is up to 29 words). Only uart_tx, spi
  and i2c were run with a non-MVP config (FIFO depth 8, 64 words, 3 engines).
- **The I2C target rig ACKs any address,** has no clock stretching and no underrun protection. An empty TX FIFO mid-byte
  loses bit phase, so the uart_rx-to-i2c bridge depends on the master's timing.
- **Bridge losses can be garbage bytes:** a lost bit phase can yield a spurious byte (`bytes_spurious`). Sustained rate
  assumes monotone loss, resolved to one ladder step (about 1.3 to 2 times in rate).
- **Engine routing is exercised only by bridges.** No firmware uses it yet; no three-engine chain or 1-Wire bridge end.
- **Power is approximate:** about plus or minus 20 to 30 %, excluding pads and the TT infrastructure. For sram2p,
  `power_mw_typ` excludes macro dynamic power; read `power_mw_typ_incl_macro` (report total + estimated macro dynamic,
  method `estimate`) as the headline total, and `macro_mw` for the macro share. Non-MVP configs run the MVP activity
  firmware, so power is not checked for correctness (`gl` covers the chip's own tests).
- **Gate level** runs the chip's own cocotb suite (4 tests on the MVP netlist). It does not prove the protocols.
- **Flow results are partly verified.** The flow config has been checked through step 1 only; a bench-owned full flow has
  not finished end to end. Precheck is n/a; Magic DRC is off (only `route__drc_errors` and LVS counts).
  `utilization_pct` differs from flow.csv's `util_pct` (a global-placement figure).
- **Synth area is not flow area:** synth leaves out repair and CTS buffers and tap cells.
- **Imported records.** `import` gives 6461 records on this tree. Identical synth
  rows are both kept. Perf-b iss and rtl share a join key (the target is in the note), so they are separate sides.
- **Front end.** Ticking a workload with parameters shows a `k=v` field (empty = defaults); the page builds `id:k=v`. Export has no support matrix and no run button.
- **Cache is per directory.** Two stores in one directory share `cache/`; use one directory per store for clean runs.
- **Bridge runs are slow at 9600 baud:** the ISS runs about 100 k cycles per second, so each byte costs about 1 s of
  simulation. Tests use 57600 and 115200.

## Technical reference

## Python API

The front end calls these directly; the CLI is a thin layer over them.

```python
from tools.bench import store, runner, crosscheck, knobs
from tools.bench.types import Record, RunSpec, Workload

# store.py
store.load(path=None) -> list[Record]
store.query(path=None, *, records=None, **filters) -> list[Record]
    # filters: workload, metric, backend, level, method, status, config_hash, run_key (value or list),
    #          id_prefix, workload_params (dict). Unknown filter -> TypeError.
store.append(records, path=None, skip_existing=False) -> int
store.record_id(rec) -> str            # sha(record json)[:12]
store.as_dicts(records) -> list[dict]  # asdict + "id", JSON-ready
store.snapshot(name, records, directory=None) -> Path   # tools/bench/snapshots/NAME.jsonl
store.get_cached(run_key) / store.put_cached(run_key, records)

# runner.py
runner.expand_sweep(base: dict, vary: dict[str, list], workloads: list[str], backends: list[str])
    -> (specs: list[RunSpec], refused: list[{backend, workload, config, reason}])
    # workload strings: "id" or "id:param=value:param=value" (overrides the backend's listed params)
runner.run_specs(specs, jobs=4, allow_long=False, use_cache=True, on_event=None, timeouts=None,
                 store_path=None, save=True) -> list[Record]
    # on_event(dict): {"event": "cache_hit"|"start"|"done"|"skipped", "backend", "workload", "params", "key", ...}
    # called from worker threads, serialized; exceptions are logged and ignored. Callers needing ordering
    # across calls should use their own lock/queue. Records reach the store as each run finishes. Skipped (long gate) specs yield no records.
runner.repro_command(spec) -> str      # shell command that re-runs one spec
runner.parse_workload("id:k=v") -> (id, params)

# crosscheck.py
crosscheck.findings(records) -> list[dict]   # {config_hash, workload, workload_params, metric,
                                             #  verdict: match|close|off|identity|one-sided, detail, a, b}
crosscheck.summary(findings) -> {"match": n, "close": n, "off": n, "identity": n, "one-sided": n}
crosscheck.TOLERANCES   # per-metric ("exact",) | ("rel", f) | ("abs", x); default: exact for cycle/tick/count units, 2 % else
```

Backends: `fake` (level iss) and `fake2` (level rtl) are deterministic test backends; behaviour (delay, mode
fail/na/crash/raise/hang/noise/badmetric, long, refuse) is set by workload params. See `backends/fake.py`.

## What the bench checks today

`make bench-smoke` (`tools/bench/smoke.py`, about 8 s fresh, 5 s cached) runs: import all; analytic on `chip`, the cap19
workloads, the 15 firmware workloads and `spi_host_port`; iss and iss_b on uart_tx, spi, i2c, spi_host_port; one rtl
smoke (spi); then `check`, and fails on any `off` not listed with its reason in `smoke_allow.json`.

Join rule (`crosscheck.py`): `(config_hash, workload, workload_params, metric)`, after aligning workload ids
(`analytic.ANALYTIC_TO_SIM`, exact matches only; NEAR_MATCHES never join; analytic `chip` area joins flow cfg `mvp`).
A record with no params (analytic is speed-free) joins the sims' record at the workload's reference point (the speed
perf-b accepts; 8 ticks for the host port) only for metrics in `metrics.SPEED_INDEPENDENT`. Each backend is its own
side (imports are `import:<source>`, perf-b is `import:perf_b:iss` / `import:perf_b:rtl`). A value against n/a is
`one-sided (other side n/a: reason)`. A pair where one side was handed the number is `identity`, not a check.

Which analytic-vs-simulation pairs are real predictions:

| pair | status |
|---|---|
| `max_bit_rate`, 7 of 15 firmware protocols (uart_tx, jtag, swd, manchester, manchester_even, dmx512, necir): the cycles limit `f_core/ipu` binds | real prediction (archsim's copy of perf-b's value, `measured.b_max_bit_rate`, is not read) |
| `max_bit_rate`, the other 8 (uart_rx, i2c, spi, onewire, ps2, quadrature, wiegand, ws2812_fast): the protocol ceiling binds | `identity`: `rate_ceiling_hz` is an archsim input (perf-a), copied through |
| `spi_host_port` `write`, `txf0_burst_4`: archsim link windows vs sim | real, but the framing constants were calibrated on perf-a/perf-b host-port runs (weak) |
| `spi_host_port` `read`, `rxf0_burst_4` | real; differs because perf-b's read harness is defective (allowlisted) |
| `area_um2`, analytic `chip` vs flow `mvp` (MVP config only) | identity: pnr_factor is fitted to this run (archsim K10) |
| `verdict`, `margin`, `binding_limit`, chip metrics other than area | no simulation measures them: analytic only |
| `sm_cycles_per_bit`, `pin_event_to_response` | not emitted by analytic: archsim `ipu` is core cycles per bit at the fastest legal CLKDIV (perf-a), and no fw cell has a reflex limit |

So today the genuine analytic-vs-simulation predictions are 7 max rates plus 2 host-port windows (`write`, `txf0_burst_4`).
The Compare page shows, per row, the worst verdict over all pairs (off > close > identity > match > one-sided) and names that pair.

Verdict counts of the last smoke run (5381 findings; 5152 one-sided because the imports cover far more than the smoke runs):
match 195, close 1, identity 17, off 16. All 16 `off` are allowlisted (7 entries, each pinned to config and operating point,
with the perf-a/perf-b class that explains it).

The `off` rows are findings about the models (archsim's max rate is the steady-state rate; perf-b includes frame
gaps, re-arm cycles and different counting conventions: classes SUS, DEF, OP in `tools/archsim/notes/perf-a-vs-b.md`).
