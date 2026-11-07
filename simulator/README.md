# Protocol Emulator ASIC simulator

```
make ui
```

Simulator for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/) design on Tiny Tapeout, IHP SG13CMOS5L: two RP2040-PIO-compatible engines behind an SPI host port. Each result is stored as a record with provenance; `check` compares the same quantity across levels.

## Install

Python 3.10+ and `make`.

```
git clone https://github.com/alexk109/js-challenge.git && cd js-challenge/simulator
make setup
```

## Run

| command | action |
|---|---|
| `make quickstart` | two example runs (analytic, ISS) |
| `make smoke` | all backends on a small workload, then cross-level checks |
| `make test` | `pytest -m "not long" tools/bench/tests` |
| `make test-all` | `make test` plus `pytest iss asm tools/archsim/tests` |
| `make bench-test-long` | bench tests marked `long` |
| `make bench-test-isa` | `pytest -m "not long" tools/bench/isa_fw` |
| `make bench-equiv` | Yosys equivalence proof |
| `make bench-report` | quick levels, check, `snapshots/latest.html` |
| `make ui-archsim` | open the archsim UI |
| `./bench --help` | list commands |
| `./bench knobs` | list knobs and the backends that honour them |
| `./bench run --backend B --workload W` | run and store records |
| `./bench check --workload W` | compare levels |
| `./bench trace <record id>` | provenance of one record |
| `./bench sweep --help` | parameter sweeps |
| `./bench serve` | browser front end |

Other suites, run directly: `pytest verif`, `pytest fw` (`FW_TARGET=iss|iss_b|rtl`),
`make -C test/host host-mutate`, `python -m verif.fuzz.selftest`.

Options: `--no-cache`, `--store PATH` (or `BENCH_STORE`), `--allow-long`. Default store: `tools/bench/results/`.
Reference: `tools/bench/README.md`, `tools/bench/CONTRACT.md`, `tools/bench/FINDINGS.md`.

## Levels

| level | backends |
|---|---|
| analytic | `analytic` |
| ISS | `iss`, `iss_b`, `iss_p` |
| RTL | `rtl` |
| physical | `synth`, `flow`, `gl`, `gl_rig`, `power` |
| imported | `import` |

## Layout

```
bench               CLI entry point (python -m tools.bench)
tools/bench/        CLI, backends, record store, checks, browser UI
tools/archsim/      analytic model (JavaScript, run by quickjs)
iss/  verif/iss_b/  instruction-set simulators
asm/                assembler
fw/                 firmware per protocol
verif/              bus models, performance rigs, fuzzing
src/  test/         Verilog design and cocotb tests
phys/               synthesis and place-and-route configs
metrics/  research/ recorded results for the import level
```

## Optional dependencies

| dependency | enables |
|---|---|
| Verilator 5.x | `rtl` level, RTL tests |
| Yosys | `synth` level |
| LibreLane, OpenROAD, open PDK (`PDK_ROOT`), tt-support-tools in `tt/` | `flow`, `gl`, `gl_rig`, `power` |
| SymbiYosys | `make bench-equiv` |

`BENCH_TOOLCHAIN_WRAPPER` sets a wrapper command for `tools/ll`.

## License

Apache License 2.0. See `../LICENSE`.
