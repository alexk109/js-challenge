# Cocotb bench for the host SPI port (test/host).
#
#   make            run the whole bench
#   make DUT=stub   always the behavioural stand-in host_spi_stub.sv
#   make DUT=rtl    always ../../src/host_spi.sv (an error if it is not there)
#   make DUT=auto   (default) src/host_spi.sv if it exists, the stub if it does not
#   make clean
#
# Waves: VCD (test/host/host_spi.vcd), dumped by tb.sv. WAVES=0 stops cocotb from
# adding its own dump module, which is the one that wants FST.

SIM           ?= icarus
TOPLEVEL_LANG ?= verilog
TOPLEVEL       = tb
WAVES          ?=
DUT           ?= auto

RTL := ../../src/host_spi.sv

# DUTFILE stays relative to this directory; VERILOG_SOURCES makes it absolute.
#
# CAREFUL when you point this at a different DUT by hand: cocotb's Makefile.icarus
# rebuilds sim_build/sim.vvp only when a source is *newer* than it, so a DUT file that was
# written before the last build looks up to date and the bench silently keeps running
# against the previous one -- with no error anywhere.  `make clean` first, or use
# `make -C test/host host-mutate`, which does it and then checks that sim.vvp itself names
# the file it was asked to compile (test/host/mutants/README.md).
ifeq ($(DUT),auto)
  ifeq ($(wildcard $(RTL)),)
    DUTFILE := host_spi_stub.sv
  else
    DUTFILE := $(RTL)
  endif
else ifeq ($(DUT),stub)
  DUTFILE := host_spi_stub.sv
else ifeq ($(DUT),rtl)
  ifeq ($(wildcard $(RTL)),)
    $(error DUT=rtl, but $(RTL) does not exist)
  endif
  DUTFILE := $(RTL)
else
  $(error DUT must be auto, stub or rtl, not "$(DUT)")
endif

VERILOG_SOURCES  = $(abspath $(PWD)/tb.sv) $(abspath $(PWD)/$(DUTFILE))
COCOTB_TEST_MODULES = test_host_spi

# Which module the bench compiled, so a log says whether it saw the RTL or the stub.
export HOST_DUT = $(notdir $(DUTFILE))

include $(shell cocotb-config --makefiles)/Makefile.sim

# ---------------------------------------------------------------- RTL host bench
# The 7 mutations of docs/mvp-verification-report.md sec. 7, re-applied to a copy of the
# real src/host_spi.sv and run through this bench (test/host/mutants/README.md).  Both
# targets are added *after* the include on purpose: Makefile.sim's `all` has to stay the
# default goal, so a plain `make` in this directory still runs the bench.
#
#   make -C test/host host-mutate            all 7; exit 1 if any survives
#   make -C test/host host-mutate MUT=M4     one of them (id prefix or full id)
#   make -C test/host host-mutant-selftest   the negative controls of the mutant driver
PY  := $(if $(wildcard ../../.venv/bin/python),../../.venv/bin/python,python3)
MUT ?=

host-mutate:
	$(PY) mutants/run.py $(if $(MUT),--only $(MUT))

host-mutant-selftest:
	$(PY) mutants/selftest.py

.PHONY: host-mutate host-mutant-selftest
