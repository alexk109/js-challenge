# make bench-equiv: formal proof that the parametric RTL at default parameters is the original MVP pio_chip
# (tools/bench/formal/equiv_mvp/: all-time Yosys equivalence + negative control + SymbiYosys BMC miter).
.PHONY: bench-equiv
bench-equiv:
	$(PY) tools/bench/formal/equiv_mvp/run.py $(if $(EQUIV_BMC),--bmc) $(EQUIV_ARGS)
