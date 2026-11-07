# tools/bench/bench.mk -- the evidence bench (tools/bench/CONTRACT.md, tools/bench/PLAN.md)
#   make bench ARGS="knobs"       any CLI command (python -m tools.bench ...)
#   make bench-test               fast tests (< ~3 min)
#   make bench-test-long          the long-marked tests (flow, RTL sweeps, gate level, power)
PY ?= python3
.PHONY: bench bench-test bench-test-long
bench:
	$(PY) -m tools.bench $(ARGS)
bench-test:
	$(PY) -m pytest -q -m "not long" tools/bench/tests
bench-test-long:
	$(PY) -m pytest -q -m long tools/bench/tests
-include $(sort $(wildcard tools/bench/mk/*.mk))
