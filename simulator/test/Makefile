# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
# Must list the same files, in the same order, as info.yaml `project.source_files`:
# the TT top level, the two modules it instantiates, and the three more that make up
# pio_chip.  Anything else in ../src (the retired UART stub) is not part of the chip.
PROJECT_SOURCES ?= tt_um_protocol_emulator.v pio_chip.sv pio_clkdiv.sv pio_engine.sv \
                   pio_fifo.sv host_spi.sv

# SRAM_IMEM=1 builds the same chip with each
# engine's IMEM in one two-port IHP SRAM macro (src/pio_imem_sram.sv) -- two macros in the
# chip -- instead of the flop array. Same tests, same programs, same drivers; see
# verif/fuzz/Makefile for the same block.
# The macro is simulated with the PDK's own behavioural model -- hence PDK_ROOT, which the
# RTL path otherwise does not need -- and PIO_SRAM_MODEL tells src/pio_imem_sram.sv that the
# model's array is there to give the flop build's power-up value.
SRAM_IMEM   ?= 0
PDK_ROOT    ?= $(HOME)/pdk
SRAM_PDK     = $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/verilog
SRAM_TAG     =
ifeq ($(SRAM_IMEM),1)
# GATES=yes: the netlist already holds the SRAM wrapper's logic, so only the PDK macro model goes in
ifneq ($(GATES),yes)
COMPILE_ARGS    += -DIMEM_SRAM -DPIO_SRAM_MODEL
VERILOG_SOURCES += $(SRC_DIR)/pio_imem_sram.sv
endif
VERILOG_SOURCES += $(SRAM_PDK)/RM_IHPSG13_2P_64x22_c2_bm_bist.v
VERILOG_SOURCES += $(SRAM_PDK)/RM_IHPSG13_2P_core_behavioral_bm_bist_ideal.v
ifeq ($(SIM),verilator)
# The PDK model has a `#0` and a specify block; cocotb's Makefile.verilator passes no timing
# option and the DUT has no timing controls, so tell Verilator to ignore both.
COMPILE_ARGS    += --no-timing
# ... and the two warnings that leaves behind come from the PDK's own file, which this
# bench does not edit. -Wno-fatal keeps a future PDK warning from failing the build.
COMPILE_ARGS    += -Wno-SPECIFYIGN -Wno-STMTDLY -Wno-fatal
endif
SRAM_TAG        = -sram
endif

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl$(SRAM_TAG)
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl$(SRAM_TAG)
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_io/verilog/sg13cmos5l_io.v
# sg13cmos5l_stdcell.v instantiates the PDK's UDP primitives (ihp_dff_r, ihp_mux2, ihp_mux4,
# ...) but does not `include them; they live in this sibling file, and icarus fails with
# "Unknown module type: ihp_mux2" without it.  Harmless if the netlist uses no such cell.
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
