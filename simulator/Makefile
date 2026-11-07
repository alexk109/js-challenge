# Top-level entry point. Run `make help`.
# PY is the Python to use; a local .venv (made by `make setup`) is picked up automatically.
PY ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
export PYTHONPATH := .

.PHONY: help setup quickstart smoke test test-all ui ui-archsim
help:
	@echo "make ui           open the front end (--archsim variant: make ui-archsim)"
	@echo "make setup        create .venv and install requirements.txt"
	@echo "make quickstart   run two tiny examples and print the results (about 2 s)"
	@echo "make smoke        the whole bench end to end, then the cross-level checks (about 10 s)"
	@echo "make test         fast tests (about 2 min)"
	@echo "make test-all     fast tests plus the ISS, assembler and archsim unit tests"
	@echo "make bench ARGS='knobs'   any bench command, e.g. ARGS='run --backend iss --workload spi'"
	@echo "or use ./bench <command> directly; ./bench --help lists the commands"

setup:
	python3 -m venv .venv
	.venv/bin/pip install -r requirements.txt

ui:
	python3 tools/ui_launch.py $(UI_ARGS)

ui-archsim:
	python3 tools/ui_launch.py --archsim $(UI_ARGS)

quickstart:
	$(PY) -m tools.bench run --backend analytic --workload chip
	$(PY) -m tools.bench run --backend iss --workload uart_tx

smoke: bench-smoke

test: bench-test

test-all: bench-test
	$(PY) -m pytest -q iss asm tools/archsim/tests

include tools/bench/bench.mk
-include verif/fuzz/fuzz.mk

SEED      ?= 1
CYCLES    ?= 1000
FUZZ_SIM  ?= verilator
FUZZ_ARGS ?=

fuzz:
	$(PY) -m verif.fuzz.lockstep --seed $(SEED) --cycles $(CYCLES) --sim $(FUZZ_SIM) $(FUZZ_ARGS)

fuzz-selftest:
	$(PY) -m verif.fuzz.selftest --sim $(FUZZ_SIM)

fuzz-repro:
	@test -n "$(REPRO)" || { echo "make fuzz-repro: set REPRO=<verif/fuzz/repros/*.json>"; false; }
	$(PY) -m verif.fuzz.minimize $(REPRO) --quiet
	$(PY) -m verif.fuzz.lockstep --repro $(if $(MINIMIZE),$(basename $(REPRO)).min.json,$(REPRO))

host-mutate:
	$(MAKE) -C test/host host-mutate
