# SPDX-License-Identifier: Apache-2.0
#
# adpll.sdc: constraints for phys/cfg/adpll-probe.json — a design whose clock is generated on die.
#
# The question: can LibreLane/OpenROAD time a DCO-generated clock?
# This file is the experiment. It is not a product SDC and it makes no attempt to be one.
#
# The shape of the problem, stated first so the log can be read against it:
#
#  * `clk` is the only clock the board supplies (research/models/limits.py:27, <= 66 MHz).
#  * `uo_out[0]` carries the ring oscillator's output. The path from `clk` to `uo_out[0]` does not
#    exist: the ring free-runs, is trimmed by a counter-based loop, and has no timing relationship
#    to `clk` that STA could derive. Worse, the path *through* the ring is a **combinational loop**
#    (verilator reports UNOPTFLAT on it, which is the correct report), so the clock cannot even be
#    traced from its own output.
#  * Therefore the DCO clock has to be *declared*, not *derived*. Three ways to do that are tried
#    below, each inside `catch`, so one flow run reports which the tool accepts:
#      (A) `create_clock` on the ring net at its slowest guaranteed period. This is the pattern the
#          one published OpenLane/OpenROAD ring-oscillator project uses:
#            https://github.com/opentrng/ptrng/blob/4ce474b7/hardware/config/digitalnoise/templates/constraints.xdc.jinja
#            "Ring-oscillator maximal output frequency (for proper timing verifications on ROs
#             clock output): create_clock -name ringN_clk -period 1/fmax -add [get_nets ...]"
#      (B) `create_generated_clock` from `clk`. This is the *semantically wrong* answer — the ring is
#          not derived from `clk` — but it is the form the experiment asks about, and whether OpenSTA
#          accepts it and what it reports is the datum.
#      (C) nothing at all: leave the domain unconstrained. This is the failure mode
#          `tt_um_nuatlabs_fifo_pwm` demonstrated going unnoticed (C1.md:240) and the one C1's kill
#          criterion A3 exists to catch. Run B above is the *detection*: the run's own STA report has
#          to show the DCO flops with setup **and hold** slack under a clock.
#
# Two more facts this file encodes, both from committed repo files:
#
#  * The period to declare. research/ideation/round2/research/adpll/adpll-physics.py, section 3
#    grid-searches (tap, fine) over the **127-stage** ring (N_MAX = 127, a 7-level tap tree) and
#    reaches every integer multiple of 50 MHz at all three corners. The tap words it writes down are
#    the **N_MAX-wide** column of log section 3, i.e. the model this file names -- at N=8
#    (400 MHz) fast tap 22 / typ 15 / **slow 1**; at N=4 (200 MHz) fast 83 / typ 19 / slow 3
#    (The model also prints the per-tap-tree
#    column, 20/12/11 and 61/17/13; both reach all nine targets.)
#    (The *flow probe* is a different, smaller thing: N=31 with a 5-level tree, a
#    deliberate concession to flow runtime, so its taps are not the ones quoted here.)
#    The *guaranteed* floor is the slow corner, so the declared period must be the slow-corner period,
#    not the typical one. DCO_PERIOD below is an environment variable so the experiment can be run
#    at both and the difference read off.
#  * What is deliberately left unconstrained. `tap` and `fine` are reference-domain registers that
#    drive the mux select pins inside the ring. They are quasi-static control, not data: the loop
#    is dithering between taps by design, so any intermediate value is legal
#    (sources/ADPLL/govardhana-adpll-ttsky26c.md). `set_clock_groups -asynchronous` cuts those paths
#    and this file says so in a comment, because a cut path with no recorded reason is the thing
#    C1.md:243-253 warns about. An SBY proof discharging "a tap change cannot glitch the ring into
#    a wrong frequency" is *not* attempted here and is listed as unverified in the write-up.
#
# Base.sdc for LibreLane 3.1.0.dev3 constrains one clock and warns about the rest
# (C1.md:239), so as with src/lane.sdc everything below re-does what base.sdc would have done.

# --- the slow-corner floor of the DCO, from adpll-physics.py section 3 -------------------------
# 5.0 ns = 200 MHz = 4x the 50 MHz reference. Not typed in by hand: it is the N=4 row of that log.
if { [info exists ::env(DCO_PERIOD)] } {
  set dco_period $::env(DCO_PERIOD)
} else {
  set dco_period 5.0
}

set input_delay_value  [expr $::env(CLOCK_PERIOD) * $::env(IO_DELAY_CONSTRAINT) / 100]
set output_delay_value [expr $::env(CLOCK_PERIOD) * $::env(IO_DELAY_CONSTRAINT) / 100]
set cap_load           [expr $::env(OUTPUT_CAP_LOAD) / 1000.0]

set_max_fanout $::env(MAX_FANOUT_CONSTRAINT) [current_design]
if { [info exists ::env(MAX_TRANSITION_CONSTRAINT)] } {
  set_max_transition $::env(MAX_TRANSITION_CONSTRAINT) [current_design]
}
if { [info exists ::env(MAX_CAPACITANCE_CONSTRAINT)] } {
  set_max_capacitance $::env(MAX_CAPACITANCE_CONSTRAINT) [current_design]
}

# --- inputs, with clk removed -------------------------------------------------------------------
set all_inputs [all_inputs]
set idx [lsearch $all_inputs "clk"]
set all_inputs [lreplace $all_inputs $idx $idx]

# --- (0) the one real clock ----------------------------------------------------------------------
create_clock [get_ports "clk"] -name core_clk -period $::env(CLOCK_PERIOD)
set_input_delay  $input_delay_value  -clock [get_clocks core_clk] $all_inputs
set_output_delay $output_delay_value -clock [get_clocks core_clk] [all_outputs]

# --- (A) independent clock on the ring net, at its slow-corner floor -----------------------------
# uo_out[0] and the internal net are the same driver, so constraining the port is enough for the
# flops: OpenSTA propagates a clock backwards from a port only if it can reach the sinks, and here
# it reaches them *through a combinational loop*, so the port form is the reliable one and the net
# form is attempted as well.
set dco_obj {}
foreach cand [list [list get_ports {uo_out\[0\]}] [list get_nets {uo_out[0\]}] \
                   [list get_nets {dco_clk}] [list get_pins {uo_out\[0\]}]] {
  if { [catch { set o [eval $cand] }] == 0 && [llength $o] > 0 } {
    set dco_obj $o
    puts "ADPLL-SDC: ring clock object resolved by \[$cand\] -> $o"
    break
  }
}

set have_dco_clock 0
set have_dco_gen    0
if { [llength $dco_obj] > 0 } {
  if { [catch { create_clock -name dco_clk -period $dco_period -add $dco_obj }] } {
    puts "ADPLL-SDC: (A) create_clock on the ring FAILED"
  } else {
    set have_dco_clock 1
    puts "ADPLL-SDC: (A) create_clock dco_clk -period $dco_period ACCEPTED"
  }

  # (B) the generated-clock form, which is the one the experiment asks about. -source is `clk` because that
  # is the only clock that exists in the design; the ring's independence from it is exactly what
  # this line pretends, and the report is what tells us whether the tool noticed.
  if { [catch { create_generated_clock -name dco_gen -source [get_ports "clk"] \
                -multiply_by 4 -add $dco_obj }] } {
    puts "ADPLL-SDC: (B) create_generated_clock FAILED"
  } else {
    set have_dco_gen 1
    puts "ADPLL-SDC: (B) create_generated_clock dco_gen ACCEPTED"
  }
} else {
  puts "ADPLL-SDC: no object for the ring clock resolved; the DCO domain will be UNCONSTRAINED"
}

# --- the two domains are unrelated by construction -----------------------------------------------
# core -> dco: the tap and fine buses (quasi-static control, dithering by design, see the header).
# dco -> core: the two 2-flop synchronisers in phys/probes/rtl/probe_adpll_flow.sv.
# Both are cut here rather than timed. A CDC tool or an SBY proof would have to discharge this; none
# is attempted in this run and the write-up records that as unverified.
if { $have_dco_clock || $have_dco_gen } {
  set dco_clocks {}
  if { $have_dco_clock } { lappend dco_clocks [get_clocks dco_clk] }
  if { $have_dco_gen }   { lappend dco_clocks [get_clocks dco_gen] }
  set_clock_groups -asynchronous -group [get_clocks core_clk] -group $dco_clocks
}

# --- miscellanea, copied from scripts/base.sdc -------------------------------------------------
set_driving_cell \
    -lib_cell [lindex [split $::env(SYNTH_DRIVING_CELL) "/"] 0] \
    -pin      [lindex [split $::env(SYNTH_DRIVING_CELL) "/"] 1] \
    $all_inputs

set_driving_cell \
    -lib_cell [lindex [split $::env(SYNTH_CLK_DRIVING_CELL) "/"] 0] \
    -pin      [lindex [split $::env(SYNTH_CLK_DRIVING_CELL) "/"] 1] \
    [get_ports "clk"]

set_load $cap_load [all_outputs]

set_clock_uncertainty $::env(CLOCK_UNCERTAINTY_CONSTRAINT) [get_clocks core_clk]
set_clock_transition  $::env(CLOCK_TRANSITION_CONSTRAINT)  [get_clocks core_clk]
set_timing_derate -early [expr 1-[expr $::env(TIME_DERATING_CONSTRAINT) / 100.0]]
set_timing_derate -late  [expr 1+[expr $::env(TIME_DERATING_CONSTRAINT) / 100.0]]

if { [info exists ::env(OPENLANE_SDC_IDEAL_CLOCKS)] && $::env(OPENLANE_SDC_IDEAL_CLOCKS) } {
  unset_propagated_clock [all_clocks]
} else {
  set_propagated_clock [all_clocks]
}

# --- what the run has to report, printed so a grep finds it in the STA log ----------------------
puts "ADPLL-SDC: SUMMARY have_dco_clock=$have_dco_clock have_dco_gen=$have_dco_gen"
foreach c [all_clocks] {
  puts "ADPLL-SDC: clock [get_name $c]"
}