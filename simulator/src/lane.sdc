# SPDX-License-Identifier: Apache-2.0
#
# lane.sdc: two clocks for the lane probe (phys/cfg/lane-probe*.json).
#
# Derived from the SDC the Tiny Tapeout FAQ publishes for exactly this case
# ("How can I map an additional external clock to one of the GPIOs",
# https://tinytapeout.com/faq/, fetched 2026-10-05 and kept verbatim at
# research/ideation/round2/research/sources/C1/tt-faq-additional-external-clock.md)
# and from this flow's own scripts/base.sdc (LibreLane 3.1.0.dev3, the version in
# metrics/flow.csv column flow_version). Differences from the FAQ version, and why:
#
#  1. Env var names are LibreLane 3's, not OpenLane 1's: IO_DELAY_CONSTRAINT
#     (the FAQ's IO_PCT), TIME_DERATING_CONSTRAINT in percent (the FAQ's
#     SYNTH_TIMING_DERATE as a fraction), CLOCK_UNCERTAINTY_CONSTRAINT and
#     CLOCK_TRANSITION_CONSTRAINT (the FAQ's SYNTH_CLOCK_*).
#  2. The lane clock gets its own period, lane_period, because it is not the same
#     frequency as clk (the FAQ's SDC is explicitly organised for the equal-
#     frequency case only: "This works if and only if the two clocks have the
#     same frequency!").
#  3. uo_out[3] gets its output delay against the lane clock and nothing else,
#     so the ui_in[3] -> uo_out[3] path is a real timed path and its slack says
#     how much of the bit time the lane's own logic eats.
#  4. set_clock_groups -asynchronous cuts core->lane and lane->core, i.e. it
#     declares the two-flop crossings in src/tt_um_lane_probe.v to be
#     unconstrained, which is the assumption an SBY proof would have to
#     discharge.
#
# THREE FACTS ABOUT THIS TOOLCHAIN THAT THE FAQ GETS WRONG OR DOES NOT SAY.
# All three are behaviours of the OpenSTA inside LibreLane 3.1.0.dev3, measured
# on the probe's own pre-PnR netlist;
# the FAQ's recipe is written for OpenLane 1.x, whose port lookup was Python's.
#
#  A. Do NOT escape the brackets. `get_ports {ui_in\[3\]}` returns an EMPTY
#     collection in this OpenSTA: its pattern matcher knows `*` and `?` but not
#     Tcl's backslash escape, so the pattern is compared literally, brackets and
#     all. `get_ports {ui_in[3]}` returns exactly the one bit. So does
#     `get_port {ui_in[3]}`. Spelling it the FAQ's way is neither an error nor
#     silence — it is worse than loud, because the checks that look for it miss
#     it. What negative-control runs measured:
#       * `create_clock` accepts the empty collection WITHOUT error or warning and
#         creates a clock OBJECT named lane_clk whose `sources` is empty, so kill
#         criterion A2 ("lane_clk never appears in the signoff STA") does NOT fire;
#       * OpenSTA does print `port 'ui_in\[3\]' not found`, as a WARNING, at the
#         get_ports. The third warning says the rest: "virtual clock lane_clk can
#         not be propagated". The same line appears in the flow log of the run that used this spelling;
#       * the run finishes green, and the flow's own check_setup then reports 36
#         unclocked register/latch pins and 36 unconstrained endpoints. A3's
#         "unconstrained endpoints" half is the check that catches this; A2 is not.
#     The assertion below turns it into a hard failure instead: with it, that same
#     escaped spelling aborts the SDC read.
#  B. `[all_inputs]` does not return port NAMES, it returns opaque object handles
#     (`_10fc1f5655550000_p_Port`). So `lsearch $all_inputs "clk"` is -1 for
#     every design, and `lreplace $all_inputs -1 -1` then deletes the LAST input
#     instead (Tcl counts from the end), leaving the real clock port in the list:
#     that is the source of base.sdc's `set_input_delay relative to a clock
#     defined on the same port/pin not allowed`. The fix is the idiom base.sdc
#     itself uses: search with the HANDLE from get_port, which stringifies to the
#     same handle. Below, a clock port that is not found is a hard error rather
#     than a silent -1, because a silently mis-constrained clock is the exact
#     failure this probe exists to detect.
#  C. SYNTH_CLK_DRIVING_CELL is not exported to the SDC environment
#     (the run's generated environment file has 74 ::env(...)
#     lines, and it is not one of them).
#     scripts/base.sdc lines 47-49 default it from SYNTH_DRIVING_CELL; a custom
#     SDC replaces that file wholesale, so it has to repeat the default or the
#     read dies with `can't read "::env(SYNTH_CLK_DRIVING_CELL)"`, which is what
#     killed pre-PnR STA in the previous attempt of this probe.
# --- shared constants, copied from scripts/base.sdc -----------------------------------
set input_delay_value  [expr $::env(CLOCK_PERIOD) * $::env(IO_DELAY_CONSTRAINT) / 100]
set output_delay_value [expr $::env(CLOCK_PERIOD) * $::env(IO_DELAY_CONSTRAINT) / 100]
set cap_load           [expr $::env(OUTPUT_CAP_LOAD) / 1000.0]

# The lane period cannot come from the config. LibreLane exports only its own declared
# variables into the SDC environment: the generated _env_*.tcl of this run lists 74
# ::env(...) lines and a LANE_PERIOD among them is not one,
# so the key that used to sit in
# phys/cfg/lane-probe.json never arrived here and was removed from it. Reading
# $::env(LANE_PERIOD) anyway is what failed STA in attempt 4 of this probe.
# 30 ns = 33 MHz is the pad output toggle limit (research/models/limits.py:29), i.e. the
# fastest bit time a lane could ever be asked for; it sets only the period the lane
# output is timed against, not t_int, which is read as the report's data arrival time.
set lane_period 30.0

set_max_fanout $::env(MAX_FANOUT_CONSTRAINT) [current_design]
if { [info exists ::env(MAX_TRANSITION_CONSTRAINT)] } {
    set_max_transition $::env(MAX_TRANSITION_CONSTRAINT) [current_design]
}
if { [info exists ::env(MAX_CAPACITANCE_CONSTRAINT)] } {
    set_max_capacitance $::env(MAX_CAPACITANCE_CONSTRAINT) [current_design]
}

# --- the two clock ports, looked up ONCE, and asserted ------------------------------
# get_port/get_ports never fail: a pattern that matches nothing returns an EMPTY
# collection (fact A). So every port this file constrains is resolved here, once, and
# the assertion below covers ALL of them: create_clock, set_driving_cell and the
# all_inputs removal all take $clk_port / $lane_port from here and never re-query.
# A port that does not exist must abort the SDC read, not create a source-less clock.
set clk_port  [get_port "clk"]
set lane_port [get_port {ui_in[3]}]
foreach p [list $clk_port $lane_port] {
    if { [llength $p] == 0 } {
        error "lane.sdc: clock port not found; the lane clock would be missing"
    }
}

# --- inputs, with BOTH clock ports removed (fact B above) -----------------------------
set all_inputs [all_inputs]
foreach clock_port [list $clk_port $lane_port] {
    set idx [lsearch $all_inputs $clock_port]
    if { $idx < 0 } {
        error "lane.sdc: clock port '[get_property $clock_port full_name]' is not in all_inputs"
    }
    set all_inputs [lreplace $all_inputs $idx $idx]
}

# --- clk: the board's clock -------------------------------------------------------------
create_clock $clk_port -name core_clk -period $::env(CLOCK_PERIOD)
set_input_delay  $input_delay_value  -clock [get_clocks core_clk] $all_inputs
set_output_delay $output_delay_value -clock [get_clocks core_clk] [all_outputs]

# --- ui_in[3]: the bus clock ------------------------------------------------------------
create_clock $lane_port -name lane_clk -period $lane_period

# The measurement: uo_out[3] is written by nothing else, so the ui_in[3] -> uo_out[3]
# path is the lane. Output delay 0 against lane_clk means the path must fit inside
# the lane's own bit time, i.e. its slack is lane_period - t_int, and the report
# prints the arrival time, which is t_int itself.
set_output_delay 0.0 -clock [get_clocks lane_clk] [get_ports {uo_out[3]}]

# The two clocks are unrelated: the crossings are the two-flop pairs in the RTL.
set_clock_groups -asynchronous -group {core_clk} -group {lane_clk}

# --- miscellanea, copied from scripts/base.sdc ------------------------------------------
# Fact C: this default is scripts/base.sdc:47-49, repeated because this file replaces it.
if { ![info exists ::env(SYNTH_CLK_DRIVING_CELL)] } {
    set ::env(SYNTH_CLK_DRIVING_CELL) $::env(SYNTH_DRIVING_CELL)
}

set_driving_cell \
    -lib_cell [lindex [split $::env(SYNTH_DRIVING_CELL) "/"] 0] \
    -pin      [lindex [split $::env(SYNTH_DRIVING_CELL) "/"] 1] \
    $all_inputs

# Both clock ports get the clock driving cell, as base.sdc gives it to clk: the lane
# clock's slew at the pad is then the same number the core clock's is.
set_driving_cell \
    -lib_cell [lindex [split $::env(SYNTH_CLK_DRIVING_CELL) "/"] 0] \
    -pin      [lindex [split $::env(SYNTH_CLK_DRIVING_CELL) "/"] 1] \
    $clk_port

set_driving_cell \
    -lib_cell [lindex [split $::env(SYNTH_CLK_DRIVING_CELL) "/"] 0] \
    -pin      [lindex [split $::env(SYNTH_CLK_DRIVING_CELL) "/"] 1] \
    $lane_port

set_load $cap_load [all_outputs]

set_clock_uncertainty $::env(CLOCK_UNCERTAINTY_CONSTRAINT) [get_clocks core_clk]
set_clock_uncertainty $::env(CLOCK_UNCERTAINTY_CONSTRAINT) [get_clocks lane_clk]
set_clock_transition  $::env(CLOCK_TRANSITION_CONSTRAINT)  [get_clocks core_clk]
set_clock_transition  $::env(CLOCK_TRANSITION_CONSTRAINT)  [get_clocks lane_clk]
set_timing_derate -early [expr 1-[expr $::env(TIME_DERATING_CONSTRAINT) / 100.0]]
set_timing_derate -late  [expr 1+[expr $::env(TIME_DERATING_CONSTRAINT) / 100.0]]

if { [info exists ::env(OPENLANE_SDC_IDEAL_CLOCKS)] && $::env(OPENLANE_SDC_IDEAL_CLOCKS) } {
    unset_propagated_clock [all_clocks]
} else {
    set_propagated_clock [all_clocks]
}
