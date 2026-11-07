"""The metric vocabulary: the only metric names a backend may emit (CONTRACT.md section 2).

Why a closed list: cross-checks join records on `(config_hash, workload, workload_params, metric)`, so
the same quantity must carry the same name at every level. Other tasks append their names in their
own section below; never reorder or rename an existing entry.
"""
from __future__ import annotations

#: {name: (unit, description)}
METRICS: dict[str, tuple[str, str]] = {}


def _add(unit: str, desc: str, *names: str) -> None:
    for n in names:
        assert n not in METRICS, n
        METRICS[n] = (unit, desc)


# --- task A: the metric column of research/perf/b/perf.csv (verif/perf rigs) -------------------------
_add("core cycles per transaction", "Host SPI port transaction length.",
     "write", "read", "read_rxf0_pop", "txf0_burst_4", "rxf0_burst_4", "host_port_read_cycles", "host_port_write_cycles")
_add("Hz", "Rate or frequency observed on a pin or computed from a pin trace.",
     "measured_bit_rate_hz", "measured_checker_bit_rate_hz", "measured_line_rate_hz", "measured_max_scl_hz",
     "measured_swclk_hz", "max_bit_rate", "measured_achieved_baud", "measured_requested_baud")
_add("ticks", "Duration or period in core clock ticks.",
     "measured_bit_period_ticks", "measured_checker_bit_ticks", "measured_clock_period_ticks",
     "measured_frame_period_ticks", "measured_half_bit_ticks", "measured_min_step_ticks",
     "measured_pulse_period_ticks", "measured_pulse_width_ticks", "measured_shortest_scl_period",
     "measured_slot_period_ticks", "measured_slot_ticks", "measured_steady_tck_period",
     "measured_ticks_per_count", "measured_ticks_per_sample", "regwrite_to_first_pin_edge",
     "pin_event_to_response", "measured_high", "measured_low")
_add("cycles", "State-machine cycles per protocol unit.",
     "sm_cycles_per_bit", "measured_sm_cycles_per_bit", "measured_sm_cycles_per_slot",
     "measured_sm_cycles_bounds", "cmd_only")
_add("count", "Items observed on the wire or in a model during the rig run.",
     "measured_bytes_on_the_wire", "measured_counts", "measured_frame_bits", "measured_frames",
     "measured_frames_to_device", "measured_frames_to_host", "measured_packets", "measured_preamble_bits",
     "measured_presence_pulses", "measured_read_slots", "measured_write_slots", "measured_streams",
     "measured_violations", "measured_chip_position", "measured_encoder_position", "measured_model_position",
     "measured_clkdiv", "measured_responds", "measured_program", "measured_unit_ns_bench",
     "measured_unit_ns_real", "measured_profile_max_rpm", "measured_tolerance", "sweep_point")
_add("words", "FIFO words free at the deepest point of the rig transaction (rigs preload up to fifo_depth words with the engine stopped and use a fixed payload: not a rate-driven demand measure).", "fifo_headroom_at_max_rate")

# --- task A: archsim cell outputs ------------------------------------------------------------------
_add("", "Verdict of one archsim cell: served / marginal / impossible.", "verdict")
_add("", "Name of the limit that binds in an archsim cell.", "binding_limit")
_add("varies", "Slack of the binding limit (negative = miss); may carry a band.", "margin")

# --- task A: runner bookkeeping ---------------------------------------------------------------------
_add("", "Placeholder metric on the status=error record of a run that crashed, timed out or was refused at run time.",
     "run_error")


# --- task G: cross-check support --------------------------------------------------------------------
#: Metrics whose value does not depend on the workload's speed param when read at the workload's reference point
#: (the speed perf-b accepts; the host port's default ticks). A record with no params (analytic is speed-free) joins
#: the sims' reference-point record for these metrics only (crosscheck.py).
SPEED_INDEPENDENT = frozenset({
    "max_bit_rate", "sm_cycles_per_bit", "pin_event_to_response",
    "write", "read", "txf0_burst_4", "rxf0_burst_4", "cmd_only",
})

# --- analytic chip workload ------------------------------------------------
_add("", "Chip-level check verdict (pass / fail) over area, core clock, pins.", "chip_verdict")
_add("%", "Core utilization.", "area_utilization_pct")
_add("MHz", "Maximum core clock at the signoff corner (analytic).", "fmax_mhz")
_add("um2", "Die area left over the design area (negative = does not fit).", "die_area_margin_um2")
_add("MHz", "Slack of the core clock against the achievable one.", "core_clock_margin_mhz")
_add("MHz", "Slack of the board clock against the achievable one.", "board_clock_margin_mhz")
_add("pins", "Project pins left after the fixed host link.", "pins_left")
_add("pins", "Pins fixed by the host link (in + out + reserved); not firmware pin usage.", "pins_fixed")
_add("slots", "Free opcode slots when an ISA lever is on.", "isa_opcode_room")

# --- task G: importers: metrics/flow.csv and metrics/synth.csv --------------------------------------
_add("", "Flow probe parameter column: state machines in the probe.", "flow_n_sm")
_add("words", "Flow probe parameter column: instruction memory depth.", "flow_imem_depth")
_add("ns", "Flow clock period target.", "flow_clock_period_ns")
_add("count", "Cell count after the flow.", "flow_cells")
_add("count", "Sequential cell count after the flow.", "flow_seq_cells")
_add("um2", "Core area of the flow floorplan.", "flow_core_area_um2")
_add("%", "Core utilization reported by the flow.", "flow_util_pct")
_add("ns", "Worst negative slack (setup) of the flow.", "wns_ns")
_add("ns", "Total negative slack (setup) of the flow.", "flow_tns")
_add("ns", "Worst hold slack of the flow.", "flow_hold_wns")
_add("count", "Design-rule violations after the flow.", "flow_drc_count")
_add("count", "Clock-tree buffers inserted by the flow.", "flow_cts_buffers")
_add("count", "Antenna violations after the flow.", "flow_antenna_viol")
_add("count", "LVS errors after the flow.", "flow_lvs_errors")
_add("s", "Wall-clock runtime of the flow.", "flow_runtime_s")
_add("count", "Cell count of the Yosys synthesis probe.", "synth_cells")
_add("count", "Sequential cell count of the synthesis probe.", "synth_seq_cells")
_add("um2", "Sequential cell area of the synthesis probe.", "synth_seq_area_um2")
_add("um2", "Combinational cell area of the synthesis probe.", "synth_comb_area_um2")

# --- task G: importers: research/perf/a/perf.csv (no-simulation protocol model) ---------------------
for _n, _u, _d in (
    ("wire_unit", "text", "Name of the protocol's wire unit (bit, slot, frame)."),
    ("wire_bits_per_unit", "wire bits", "Wire bits in one wire unit."),
    ("bit_loop_instructions", "instructions per slot", "Instructions in the bit loop."),
    ("bit_loop_sm_cycles", "SM cycles per slot", "State-machine cycles in the bit loop."),
    ("wire_time_sm_cycles_per_unit", "SM cycles per slot", "SM cycles one wire unit takes."),
    ("sm_cycles_per_wire_bit", "SM cycles per wire bit", "SM cycles per wire bit (perf-a)."),
    ("sm_cycles_per_wire_bit_longest_shape", "SM cycles per wire bit", "SM cycles per wire bit of the longest shape."),
    ("instructions_per_wire_bit", "instructions per wire bit", "Instructions per wire bit."),
    ("wire_period_core_cycles_max_rate", "core cycles per slot", "Wire unit period at the maximum rate."),
    ("wire_period_core_cycles_nominal", "core cycles per slot", "Wire unit period at the nominal rate."),
    ("wire_unit_rate_max", "slot/s", "Wire unit rate at the maximum."),
    ("wire_unit_rate_nominal", "slot/s", "Wire unit rate at nominal."),
    ("bit_period_core_cycles_max_rate", "core cycles per wire bit", "Bit period at the maximum rate."),
    ("bit_period_core_cycles_nominal", "core cycles per wire bit", "Bit period at the nominal rate."),
    ("bit_rate_max", "wire bits/s", "Maximum wire bit rate (perf-a)."),
    ("bit_rate_nominal", "wire bits/s", "Nominal wire bit rate (perf-a)."),
    ("protocol_ceiling", "slot/s", "Protocol's own rate ceiling."),
    ("clkdiv_nominal", "SMe_CLKDIV word", "Nominal clock divider word."),
    ("clkdiv_min_legal", "SMe_CLKDIV word", "Smallest legal clock divider word."),
    ("engine_duty_nominal", "% of core cycles", "Engine duty at the nominal rate."),
    ("engine_duty_max_rate", "% of core cycles", "Engine duty at the maximum rate."),
    ("tx_bits_per_fifo_word", "wire bits per 32-bit FIFO word", "TX wire bits one FIFO word carries."),
    ("rx_bits_per_fifo_word", "text", "RX wire bits one FIFO word carries."),
    ("tx_fifo_headroom_max_rate", "core cycles", "TX FIFO headroom at the maximum rate."),
    ("tx_fifo_headroom_nominal", "core cycles", "TX FIFO headroom at the nominal rate."),
    ("tx_fifo_headroom_spi_writes_max_rate", "register writes", "TX FIFO headroom in SPI register writes at max rate."),
    ("rx_fifo_headroom_max_rate", "core cycles", "RX FIFO headroom at the maximum rate."),
    ("rx_fifo_headroom_nominal", "core cycles", "RX FIFO headroom at the nominal rate."),
    ("rx_fifo_headroom_spi_writes_max_rate", "register writes", "RX FIFO headroom in SPI register writes at max rate."),
    ("tx_fifo_sustainable_over_spi", "yes/no", "Whether SPI writes keep the TX FIFO fed."),
    ("host_ctrl_write_to_first_pin_write", "core cycles", "Host control write to first pin write."),
    ("host_ctrl_write_to_first_pin_write_max_rate", "core cycles", "Same at the maximum rate."),
    ("host_forced_write_to_pad_edge", "core cycles", "Forced host write to the pad edge."),
    ("host_forced_write_to_pad_edge_over_spi", "core cycles", "Same including the SPI transaction."),
    ("pin_event_to_pin_edge_min", "core cycles", "Pin event to output edge, minimum."),
    ("pin_event_to_pin_edge_max", "core cycles", "Pin event to output edge, maximum."),
    ("pin_event_jitter", "core cycles", "Spread of the pin event response."),
    ("pin_event_to_sample_in_isr_min", "core cycles", "Pin event to the sample in the ISR, minimum."),
    ("pin_event_to_sample_in_isr_max", "core cycles", "Pin event to the sample in the ISR, maximum."),
    ("pin_event_to_fifo_push_min", "core cycles", "Pin event to FIFO push, minimum."),
    ("pin_event_to_fifo_push_max", "core cycles", "Pin event to FIFO push, maximum."),
    ("spi_bit_core_cycles", "core cycles per SCK bit", "SPI host port: core cycles per SCK bit."),
    ("spi_reg_write_core_cycles", "core cycles", "SPI host port: register write."),
    ("spi_reg_read_commit_core_cycles", "core cycles", "SPI host port: register read, commit."),
    ("spi_reg_read_data_core_cycles", "core cycles", "SPI host port: register read, data."),
    ("spi_burst_write_4_words_core_cycles", "core cycles", "SPI host port: 4-word burst write."),
    ("spi_imem_burst_28_words_core_cycles", "core cycles", "SPI host port: 28-word IMEM burst."),
    ("spi_access_back_to_back_core_cycles", "core cycles", "SPI host port: back-to-back access."),
    ("spi_host_access_rate_max", "register accesses/s", "SPI host port: maximum access rate."),
):
    _add(_u, _d, _n)

# --- task G: importers: metrics/{fw,fuzz,mutate,coverage}.csv ---------------------------------------
_add("count", "Instructions in the firmware program.", "fw_instr_count")
_add("count", "Peak FIFO depth reached by the firmware sweep.", "fw_peak_fifo_depth")
_add("cycles", "Firmware cycles per bit in the sweep.", "fw_cycles_per_bit")
_add("cycles", "Firmware cycles per slot in the sweep.", "fw_cycles_per_slot")
_add("cycles", "Cycles simulated by a fuzz run.", "fuzz_cycles")
_add("count", "Programs run by a fuzz run.", "fuzz_programs")
_add("s", "Wall-clock runtime of a fuzz run.", "fuzz_runtime_s")
_add("count", "Mutants generated for a feature.", "mutate_mutants")
_add("count", "Mutants killed by the tests.", "mutate_killed")
_add("fraction", "Killed mutants over all mutants.", "mutate_kill_rate")
_add("", "Weight of a coverage bin.", "coverage_weight")
_add("count", "Hits of a coverage bin by the fuzz.", "coverage_fuzz")
_add("count", "Hits of a coverage bin by firmware tests.", "coverage_fw")
_add("count", "Hits of a coverage bin by directed tests.", "coverage_directed")
_add("count", "Hits of a coverage bin by all sources together.", "coverage_union")

# --- task I: physical backends (synth, flow) ------------------------------------------------------------
_add("um2", "Total standard-cell + macro area (synth: mapped; flow: placed, incl. repair/CTS buffers and tap cells, fill excluded).",
     "area_um2")
_add("um2", "Standard-cell area only (same scope as area_um2 minus macros).", "stdcell_area_um2")
_add("um2", "Hard-macro (SRAM) area: synth from the macro liberty, flow from design__instance__area__macros.", "macro_area_um2")
_add("um2", "Area of sequential cells (flops, latches, clock gates), synthesis.", "seq_area_um2")
_add("um2", "Area of combinational cells, synthesis.", "comb_area_um2")
_add("cells", "Standard-cell instances.", "cell_count")
_add("cells", "Sequential cell instances.", "seq_cell_count")
_add("%", "Standard-cell area over core area after the flow (fill excluded).", "utilization_pct")
_add("MHz", "1000 / (clock period - setup WNS) at the typical corner.", "fmax_mhz_typ")
_add("MHz", "1000 / (clock period - setup WNS) at the slow corner.", "fmax_mhz_slow")
_add("ns", "Worst setup slack, typical corner nom_typ_1p20V_25C.", "wns_ns_typ")
_add("ns", "Worst setup slack, slow corner nom_slow_1p08V_125C.", "wns_ns_slow")
_add("count", "Detailed-route DRC errors (route__drc_errors).", "drc_errors")
_add("count", "LVS errors (design__lvs_error__count).", "lvs_errors")
_add("bool", "TT precheck result; not part of the LibreLane run, so n/a from the flow backend.", "precheck_pass")

# --- task L: instruction-set levers (backends/isa.py, tools/bench/sim/ISA_EXT.md) -------------------------------
_add("words", "IMEM words of the workload's whole program set (one engine).", "program_words")
_add("words", "Change in program words caused by an ISA lever (lever on minus off).", "isa_words_delta")
_add("bool", "The program set fits one engine's IMEM.", "fits_imem")
# --- task K: power and gate-level backends ------------------------------------------------------------
_add("mW", "Total chip power from OpenROAD report_power, typical corner 1.20 V 25 C, 50 MHz, gate-level VCD activity, post-route SPEF.",
     "power_mw_typ")
_add("mW", "Same at the slow corner 1.08 V 125 C (separate liberty; leakage can read lower than typ).", "power_mw_slow")
_add("mW", "Same at the fast corner 1.32 V -40 C.", "power_mw_fast")
_add("mW", "SRAM configs only: power_mw_typ (report_power total, macro leakage only) + estimated macro dynamic power; the headline typical-corner total when the chip has SRAM macros.",
     "power_mw_typ_incl_macro")
_add("mW", "report_power Clock group (clock network cells), typical corner.", "clock_tree_mw")
_add("mW", "report_power Sequential group (flop clock-pin internal + data power), typical corner.", "seq_mw")
_add("mW", "report_power Combinational group, typical corner.", "comb_mw")
_add("mW", "SRAM macro power, typical corner: report_power Macro group (leakage only) plus access-count dynamic energy from the liberty (estimate); 0 without macros.",
     "macro_mw")
_add("mW", "Total leakage power from report_power, typical corner.", "leakage_mw")
_add("tests", "cocotb tests of test/test.py that passed on the hardened gate-level netlist.", "gl_tests_passed")
_add("tests", "cocotb tests that failed or errored on the hardened gate-level netlist.", "gl_tests_failed")
_add("s", "Wall-clock time of the gate-level test run (compile + simulate).", "gl_sim_seconds")
# --- task J: bridge and mix workloads (tools/bench/bridges.py; definitions in its docstring) ------------
_add("bytes", "Bytes the source VIP sent into a bridge.", "bytes_in")
_add("bytes", "Bytes the sink VIP received from a bridge.", "bytes_out")
_add("bytes", "Input bytes missing from the output, in order (LCS); bytes_in = bytes_out + bytes_lost - bytes_spurious.", "bytes_lost")
_add("bytes", "Output bytes that match no input byte (a receiver that lost its bit phase).", "bytes_spurious")
_add("words", "Max over cycles of len(source RX FIFO) + len(sink TX FIFO): the route's buffering in use.",
     "peak_buffer_words")
_add("cycles", "Sink finish tick minus source finish tick of the first matched byte (includes the output frame).",
     "latency_first_byte_cycles")
_add("cycles", "99th percentile of the same latency over all matched bytes.", "latency_p99_cycles")
_add("bytes/s", "Highest source byte rate of a burst with no loss, from stepping the inter-byte gap down a fixed ladder (gallop + bisect); assumes loss is monotone in the gap; resolution = one ladder step (about 1.3-2x in rate).",
     "sustained_rate_no_loss")
_add("fraction", "Host-port busy fraction: register accesses x single-access cycles / run cycles (>1 = infeasible).",
     "host_port_utilization")
_add("bool", "Every protocol of a mix passed its own checker.", "all_pass")
_add("bool", "One protocol of a mix passed its checker.", "pass_uart_tx", "pass_uart_rx", "pass_spi_rx", "pass_i2c_rd")
_add("cycles", "Core cycles the bridge or mix run took.", "run_cycles")
_add("Hz", "Bit rate of the isa_xor CRC-15 loop ALONE (crc15_xor.pio at CLKDIV 1); not a chip- or workload-level max_bit_rate.",
     "crc_loop_max_bit_rate")
_add("cycles", "SM cycles per bit of the isa_xor CRC-15 loop alone (both branch paths).", "crc_loop_sm_cycles_per_bit")
