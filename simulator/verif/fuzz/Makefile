# The lockstep fuzz harness's simulator build.
#
#   make -C verif/fuzz                     build and run one lockstep run (see lockstep.py)
#   make -C verif/fuzz PLANTED=1           build verif/fuzz/planted/ instead of src/
#   make -C verif/fuzz SIM=icarus          same run under Icarus
#   make -C verif/fuzz MODULES=lockstep,plant_iss
#
# The stimulus comes in through the environment (FUZZ_SEED, FUZZ_CYCLES, FUZZ_REPRO,
# FUZZ_TRIVIAL, FUZZ_RESULT), which is how verif/fuzz/lockstep.py drives it.
SIM            ?= verilator
TOPLEVEL_LANG  ?= verilog
ROOT           := $(abspath $(CURDIR)/../..)
PLANTED        ?= 0
PLANT          ?= 1
ifeq ($(strip $(PLANTED)),0)
RTL_DIR  := $(ROOT)/src
PLANTDEF :=
PLANT_TAG :=
else
RTL_DIR  := $(CURDIR)/planted
PLANTDEF := $(if $(filter 1,$(PLANT)),-DFUZZ_PLANT_RTL,) $(if $(filter 2,$(PLANT)),-DFUZZ_PLANT_RTL2,) $(if $(filter 3,$(PLANT)),-DFUZZ_PLANT_RTL3,) $(if $(filter 4,$(PLANT)),-DFUZZ_PLANT_RTL4,) $(if $(filter 5,$(PLANT)),-DFUZZ_PLANT_RTL5,) $(if $(filter 6,$(PLANT)),-DFUZZ_PLANT_RTL6,) $(if $(filter 7,$(PLANT)),-DFUZZ_PLANT_RTL7,)
# one build directory per plant: make cannot see a change of a variable on the
# command line, so switching PLANT would otherwise reuse the previous binary
PLANT_TAG := -planted$(PLANT)
endif
VERILOG_SOURCES = $(RTL_DIR)/pio_chip.sv $(RTL_DIR)/pio_clkdiv.sv $(RTL_DIR)/pio_engine.sv \
                  $(RTL_DIR)/pio_fifo.sv
TOPLEVEL        = pio_chip
COCOTB_TOPLEVEL = pio_chip
COCOTB_TEST_MODULES ?= lockstep

# SRAM_IMEM=1 builds the same DUT with each engine's
# IMEM in one two-port IHP SRAM macro (src/pio_imem_sram.sv) -- two macros in the chip, one per
# engine -- instead of the flop array. Nothing else changes -- same stimulus, same
# comparison, same lockstep signals -- and without
# SRAM_IMEM this file builds exactly what it built before. The macro is simulated with the
# PDK's own behavioural model, not with src/RM_IHPSG13_1P_64x16_c2.v, which is the
# port-only blackbox synthesis uses. PIO_SRAM_MODEL tells src/pio_imem_sram.sv that the
# model's array is there to give the flop build's power-up value.
# It is an environment variable on purpose: `SRAM_IMEM=1 make fuzz SEED=1` reaches this file
# through the harness, which passes its environment to the make it spawns.
SRAM_IMEM   ?= 0
PDK_ROOT    ?= $(HOME)/pdk
SRAM_PDK     = $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/verilog
SRAM_TAG     =
ifeq ($(SRAM_IMEM),1)
VERILOG_SOURCES += $(ROOT)/src/pio_imem_sram.sv
VERILOG_SOURCES += $(SRAM_PDK)/RM_IHPSG13_2P_64x22_c2_bm_bist.v
VERILOG_SOURCES += $(SRAM_PDK)/RM_IHPSG13_2P_core_behavioral_bm_bist_ideal.v
COMPILE_ARGS    += -DIMEM_SRAM -DPIO_SRAM_MODEL
ifeq ($(SIM),verilator)
# The PDK model has a `#0` and a specify block; cocotb's Makefile.verilator passes no timing
# option and the DUT has no timing controls, so tell Verilator to ignore both. (Without this
# the model is an error: NEEDTIMINGOPT.)
COMPILE_ARGS    += --no-timing
# ... and the two warnings that leaves behind come from the PDK's own file, which this
# harness does not edit. -Wno-fatal keeps a future PDK warning from failing the build.
COMPILE_ARGS    += -Wno-SPECIFYIGN -Wno-STMTDLY -Wno-fatal
endif
SRAM_TAG        = -sram
endif
SIM_BUILD       = sim_build/$(SIM)$(PLANT_TAG)$(SRAM_TAG)
COMPILE_ARGS    += $(PLANTDEF)
# Changing a compile flag does not by itself make cocotb's build stale; make Vtop.mk depend on
# this Makefile so that switching PLANT (or the visibility flag) really rebuilds.
CUSTOM_COMPILE_DEPS += $(CURDIR)/Makefile

# Lockstep visibility: docs/mvp-interfaces.md `### Lockstep visibility` asks for
# --public-flat-rd so that g_eng[e].u_eng.* stay readable. Verilator 5.050 has no such flag; the
# nearest is --public-flat-rw, which cocotb's own Makefile.verilator already passes and which is
# enough to read them. Use -rd when the installed verilator knows it, and tell the report which
# one the build got (lockstep.py reads FUZZ_VIS).
VLT_RD := $(shell verilator --help 2>/dev/null | grep -c -- '--public-flat-rd')
ifeq ($(VLT_RD),1)
EXTRA_ARGS  += --public-flat-rd
export FUZZ_VIS := --public-flat-rd
else
export FUZZ_VIS := --public-flat-rw (cocotb default; verilator has no --public-flat-rd)
endif

ifeq ($(SIM),icarus)
COMPILE_ARGS += -g2012
endif
include $(shell cocotb-config --makefiles)/Makefile.sim