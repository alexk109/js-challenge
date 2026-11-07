# The deep lockstep targets.
#
# The top-level Makefile includes every verif/*/*.mk, so this file adds targets without anyone
# editing the top Makefile. `fuzz`, `fuzz-selftest` and `fuzz-repro` stay where they are.
#
# Simulations are slow; the targets below do not
# run in parallel by themselves, exactly like `make fuzz`.

# make fuzz-deep SEED=3 CYCLES=10000      one run with every deep field compared (the default)
# make fuzz-deep FUZZ_ARGS="--no-deep"    the same run with only the 37 shallow signals
# make fuzz-deep FUZZ_ARGS="--pullup"     ... with an un-driven uio pin reading 1
# make fuzz-deep FUZZ_ARGS="--fifofill"   ... on the FIFO-storage stimulus
FUZZ_DEEP_ARGS ?= --deep

fuzz-deep:
	$(PY) -m verif.fuzz.lockstep --seed $(SEED) --cycles $(CYCLES) --sim $(FUZZ_SIM) \
	  $(FUZZ_DEEP_ARGS) $(FUZZ_ARGS)

# make fuzz-deep-campaign SEEDS=1-60 DEEP_CYCLES=20000 CONFLICT=256
#
# The full run: 60 seeds, 20k cycles each, every deep field, and the guaranteed
# two-engine pin conflict at the end of every run, into metrics/fuzz-deep.csv. Repro dir
# FUZZ_REPRO_DIR (default /tmp/fuzz-repros), so a clean run leaves the committed tree alone
# and a mismatch that *is* a finding can be re-run with FUZZ_REPRO_DIR=verif/fuzz/repros.
SEEDS      ?= 1-60
DEEP_CYCLES ?= 20000
CONFLICT   ?= 256
FUZZ_JOBS  ?= -j2
DEEP_CSV    = metrics/fuzz-deep.csv

# A planted run is a sensitivity record, not evidence about the chip: every row of it is `mismatch`
# by construction. It goes in its own CSV for the same reason metrics/fuzz-sensitivity.csv holds the
# other planted-bug runs, and metrics/fuzz-deep.csv stays the clean-campaign record the datasheet's
# lockstep facts are read from (docs/derive_datasheet_facts.py).
PLANT5_CSV ?= metrics/fuzz-plant5.csv

fuzz-deep-campaign:
	$(PY) -m verif.fuzz.campaign --seeds $(SEEDS) --cycles $(DEEP_CYCLES) --sim $(FUZZ_SIM) \
	  --jobs=$(FUZZ_JOBS) --deep --conflict $(CONFLICT) --csv $(DEEP_CSV) \
	  --repro-dir $(FUZZ_REPRO_DIR)/deep-campaign --label lockstep-deep $(FUZZ_ARGS)

# A campaign writes one repro per mismatching seed. The default is the committed
# verif/fuzz/repros/, which is right for a finding and wrong for a sensitivity run: 20 planted
# repros are not a finding, so the two targets below send theirs to FUZZ_REPRO_DIR, which defaults
# outside the tree. Override it with FUZZ_REPRO_DIR=<dir> if you want to keep them.
FUZZ_REPRO_DIR ?= /tmp/fuzz-repros

# make fuzz-plant5 SEEDS=1-20 DEEP_CYCLES=20000  the same campaign against planted copy 5
# (engine 0 winning a cross-engine pin conflict), which is how the "20/20" detection rate is measured.
# Its repros go to FUZZ_REPRO_DIR (default /tmp/fuzz-repros), not into verif/fuzz/repros/,
# and its rows go to PLANT5_CSV (default metrics/fuzz-plant5.csv), not into metrics/fuzz-deep.csv.
fuzz-plant5:
	$(PY) -m verif.fuzz.campaign --seeds $(SEEDS) --cycles $(DEEP_CYCLES) --sim $(FUZZ_SIM) \
	  --jobs=$(FUZZ_JOBS) --deep --conflict $(CONFLICT) --plant 5 --csv $(PLANT5_CSV) \
	  --repro-dir $(FUZZ_REPRO_DIR)/plant5 --label lockstep-deep $(FUZZ_ARGS)

# make fuzz-deep-pytest     the pure-Python parts: pad model, unpacking, field list, stimuli.
fuzz-deep-pytest:
	$(PY) -m pytest -q verif/fuzz/test_deep.py

# make fuzz-pullup-measure SEED=1 CYCLES=300   is --pullup non-vacuous?
#
# Two runs of the same stimulus, one with --pullup and one without, and the number of cycles in
# which the compared in_vec differs. Simulation: medium slot. It can fail, which is the point.
fuzz-pullup-measure:
	$(PY) -m verif.fuzz.pullup_measure --seed $(SEED) --cycles $(CYCLES) --sim $(FUZZ_SIM) \
	  --jobs=$(FUZZ_JOBS)

# make fuzz-conflict-measure SEEDS=1-20 CYCLES=10000   how often do the two engines contest one pin?
#
# The ISS only, no simulator (light slot): it is a property of the stimulus, and it is what says
# how much of a campaign's coverage the guaranteed conflict phase is doing.
fuzz-conflict-measure:
	$(PY) -m verif.fuzz.conflict_measure --seeds $(SEEDS) --cycles $(CYCLES)
