# The FW_TARGET=rtl simulator build.
#
#   make -C fw                       build and run one served session (fw/rtl_harness.py drives it)
#   make -C fw SIM=icarus            same under Icarus
#
# There is nothing protocol-specific here: the top level is `pio_chip` itself, and the one
# cocotb test is the cycle-by-cycle server in `fw/rtl_harness.py`, which the firmware tests
# drive over a pair of pipes. `fw/rtl_harness.py` builds this and runs it; nothing else does.
#
# The sources and the visibility flag are the ones verif/fuzz/Makefile already uses, because
# the firmware harness reads the same `g_eng[e].u_eng.*` signals (docs/mvp-interfaces.md
# `### Lockstep visibility`) to answer `Chip.peek()` and `Chip.state()` from the RTL rather
# than from a model of it.
SIM            ?= verilator
TOPLEVEL_LANG  ?= verilog
ROOT           := $(abspath $(CURDIR)/..)
VERILOG_SOURCES = $(ROOT)/src/pio_chip.sv $(ROOT)/src/pio_clkdiv.sv $(ROOT)/src/pio_engine.sv \
                  $(ROOT)/src/pio_fifo.sv
TOPLEVEL        = pio_chip
COCOTB_TOPLEVEL = pio_chip
COCOTB_TEST_MODULES ?= fw.rtl_harness

# SRAM_IMEM=1 builds the same chip with each
# engine's IMEM in one two-port IHP SRAM macro (src/pio_imem_sram.sv), two macros in the chip.
# See verif/fuzz/Makefile, which has the same block; fw/rtl_harness.py reads the IMEM words out
# of the RTL either way (from the flop array, or from the load macro's array when IMEM_SRAM is
# set).
# SRAM_IMEM=1 FW_TARGET=rtl make -C fw   is how it is selected.
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
# Makefile does not edit. -Wno-fatal keeps a future PDK warning from failing the build.
COMPILE_ARGS    += -Wno-SPECIFYIGN -Wno-STMTDLY -Wno-fatal
endif
SRAM_TAG        = -sram
endif
# Parametric chip: env BENCH_CONFIG (JSON of a bench config, the variable
# the parametric ISS reads) selects N_ENGINES / IMEM_WORDS / FIFO_DEPTH / ENGINE_ROUTE / T_IN of pio_chip
# through tools/bench/rtlparams.py. Unset (the MVP) adds nothing and builds where it always built; any
# other config gets its own sim_build directory, named by the hash of those parameters.
PYTHON         ?= python3
RTL_CFG_TAG    := $(strip $(shell cd $(ROOT) && PYTHONPATH=$(ROOT) $(PYTHON) -m tools.bench.rtlparams tag))
ifneq ($(RTL_CFG_TAG),)
ifeq ($(SIM),icarus)
COMPILE_ARGS   += $(shell cd $(ROOT) && PYTHONPATH=$(ROOT) $(PYTHON) -m tools.bench.rtlparams pflags)
else
EXTRA_ARGS     += $(shell cd $(ROOT) && PYTHONPATH=$(ROOT) $(PYTHON) -m tools.bench.rtlparams gflags)
endif
endif
SIM_BUILD       = sim_build/$(SIM)$(SRAM_TAG)$(if $(RTL_CFG_TAG),-$(RTL_CFG_TAG))

# docs/mvp-interfaces.md `### Lockstep visibility` asks for --public-flat-rd; Verilator 5.050 has
# no such flag and cocotb's own Makefile.verilator passes --public-flat-rw, which is enough to
# read those signals. Use -rd when the installed verilator knows it, and record which one the
# build got (fw/rtl_harness.py reads FW_RTL_VIS).
VLT_RD := $(shell verilator --help 2>/dev/null | grep -c -- '--public-flat-rd')
ifeq ($(VLT_RD),1)
EXTRA_ARGS  += --public-flat-rd
export FW_RTL_VIS := --public-flat-rd
else
export FW_RTL_VIS := --public-flat-rw (cocotb default; verilator has no --public-flat-rd)
endif

ifeq ($(SIM),icarus)
COMPILE_ARGS += -g2012
endif

# Gate level (tools/bench/backends/gl_rig.py): GL_NETLIST=<finished flow's final/nl/*.nl.v> runs the same harness on
# the post-layout netlist instead of src/*.sv -- same PDK models and defines as test/Makefile `GATES=yes`
# (-DGL_TEST -DFUNCTIONAL -DSIM; SRAM_IMEM=1 adds the SRAM macro models, which the block above already put in
# VERILOG_SOURCES).  Icarus only.  The top level is fw/gl_wrap.py's `gl_top`, which gives the flat netlist the
# bus/pin ports the harness drives (see its docstring); FW_RTL_GL tells fw/rtl_harness.py to use them.
# GL_TAG names the build directory (one per config's netlist).
GL_NETLIST ?=
ifneq ($(GL_NETLIST),)
GL_TAG         ?= default
SIM_BUILD       = sim_build/gl-$(GL_TAG)$(SRAM_TAG)
GL_TOP_V        = $(CURDIR)/$(SIM_BUILD)/gl_top.v
GL_GEN         := $(shell cd $(ROOT) && PYTHONPATH=$(ROOT) $(PYTHON) -m fw.gl_wrap $(GL_NETLIST) $(GL_TOP_V) 2>&1)
ifneq ($(strip $(GL_GEN)),)
$(error gate-level wrapper: $(GL_GEN))
endif
GL_PDK          = $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref
VERILOG_SOURCES = $(GL_PDK)/sg13cmos5l_io/verilog/sg13cmos5l_io.v \
                  $(GL_PDK)/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v \
                  $(GL_PDK)/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v \
                  $(if $(filter 1,$(SRAM_IMEM)),$(SRAM_PDK)/RM_IHPSG13_2P_64x22_c2_bm_bist.v $(SRAM_PDK)/RM_IHPSG13_2P_core_behavioral_bm_bist_ideal.v) \
                  $(GL_NETLIST) $(GL_TOP_V)
TOPLEVEL        = gl_top
COCOTB_TOPLEVEL = gl_top
COMPILE_ARGS    = -g2012 -DGL_TEST -DFUNCTIONAL -DSIM
export FW_RTL_GL := $(GL_NETLIST)
endif

# Changing a compile flag does not by itself make cocotb's build stale; depend on this file so
# that switching SIM or the visibility flag really rebuilds.
CUSTOM_COMPILE_DEPS += $(CURDIR)/Makefile

include $(shell cocotb-config --makefiles)/Makefile.sim
# The build alone, without running the simulation. fw/rtl_harness.py runs it under a lock before `make sim`:
# parallel runs of one config share SIM_BUILD, and two compiles into it at once leave a corrupt sim.vvp
# ("Program not runnable, N errors"), which surfaces as "the simulator died".
.PHONY: build
build: $(SIM_BUILD)/$(if $(filter icarus,$(SIM)),sim.vvp,Vtop)
