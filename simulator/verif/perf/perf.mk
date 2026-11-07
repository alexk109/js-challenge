# verif/perf/perf.mk: the measured MVP performance table.
#
#   make perf                     every protocol in fw/ on the ISS -> research/perf/b/perf.csv
#   make perf PROTO=spi,i2c       only those
#   make perf-rtl                 the four RTL-targetable rigs against src/pio_chip.sv, appended
#   make perf-selftest            the instruments and the four negative controls
#
# The numbers come out of simulations: the sweep is long, the self-test is short.
# Nothing here writes outside `research/perf/b/`, and `src/` is never touched.

PERF_PROTO  ?=
PERF_TARGET ?= iss
PERF_CSV    ?= research/perf/b/perf.csv
#: The four rigs that also run on `FW_TARGET=rtl` (`verif/perf/rigs.py` `LADDER.targetable`),
#: with the settings the ISS run found to be their maxima: the RTL run re-measures the same
#: setting to compare, rather than sweeping again at a few thousand cycles a second.  The `|`
#: that separates one protocol's settings is quoted in the recipe below: make hands the recipe
#: to `/bin/bash`, and unquoted it is a pipe, not an argument.
PERF_RTL_PROTOS ?= spi,ps2,swd,onewire
PERF_RTL_SPEEDS ?= spi=1|2|4,ps2=94,swd=9|11|20,onewire=47|50

#: Which IMEM the RTL build of `src/pio_chip.sv` gets.  `SRAM_IMEM=1` (the default here) builds
#: each engine's IMEM in one two-port IHP SRAM macro; `SRAM_IMEM=0` builds the 32 x 16 flop
#: array.  It is 1 because the flop build cannot be measured at all at the moment:
#: `fw/rtl_harness.py`'s `_imem_words` still looks for the array at `u_eng.imem`, and
#: `src/pio_engine.sv` holds it inside a generate block (`g_imem_flop.imem`, or the macro's
#: array under `g_imem_sram`) since the SRAM IMEM option was added, so the harness raises "no IMEM array in
#: g_eng[e].u_eng" and the simulator exits.  That is a
#: limitation of the harness, not of this bench, so this bench selects the build the harness
#: does support.  `verif/fuzz/Makefile` uses the same environment variable for the same reason.
PERF_SRAM_IMEM ?= 1

.PHONY: perf perf-rtl perf-selftest perf-guard

perf:
	FW_TARGET=$(PERF_TARGET) $(PY) -m verif.perf.measure \
	    --target $(PERF_TARGET) --csv $(PERF_CSV) \
	    $(if $(PERF_PROTO),--protocols $(PERF_PROTO),)

perf-rtl:
	SRAM_IMEM=$(PERF_SRAM_IMEM) FW_TARGET=rtl $(PY) -m verif.perf.measure \
	    --target rtl --csv $(PERF_CSV) --append --protocols $(PERF_RTL_PROTOS) \
	    --speeds '$(PERF_RTL_SPEEDS)'

perf-selftest:
	$(PY) -m verif.perf.selftest

# The negative control for `perf-rtl`'s target guard: the same bench, with the one switch that
# stops it installing a backend, has to refuse rather than measure the ISS and label it RTL.
# Expected to exit 1; `make perf-guard` is the command whose output that log holds.
perf-guard:
	@FW_TARGET=rtl PERF_SKIP_TARGET_INSTALL=1 $(PY) -m verif.perf.measure \
	    --target rtl --protocols spi --csv $(PERF_CSV).guard ; \
	  rc=$$? ; rm -f $(PERF_CSV).guard ; \
	  test $$rc -ne 0 || { echo "perf-guard: the bench ran without a backend: the guard does not bite"; exit 1; } ; \
	  echo "perf-guard: refused as it must (exit $$rc)"