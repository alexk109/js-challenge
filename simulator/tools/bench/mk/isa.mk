# tools/bench/mk/isa.mk -- instruction-set lever bench (tools/bench/sim/ISA_EXT.md)
.PHONY: bench-test-isa bench-test-isa-long
bench-test-isa:
	$(PY) -m pytest -q -m "not long" tools/bench/isa_fw
bench-test-isa-long:
	$(PY) -m pytest -q -m long tools/bench/isa_fw
