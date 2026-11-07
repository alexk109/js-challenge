// SPDX-License-Identifier: Apache-2.0
//
// adpll_loop + adpll_capture: the digital half of the ADPLL probe.
// See phys/probes/rtl/probe_adpll_dco.v for why this is probe RTL and not product RTL,
// and research/ideation/round2/research/adpll/adpll-physics.py for every number quoted here.

`default_nettype none

// Parameter-driven widths in a probe are worth more than -Wall purity: `int` parameters compared
// against narrow registers are the one thing that reads differently in slang, yosys and verilator,
// and a probe whose width behaviour is tool-dependent is not a measurement. DECLFILENAME is waived
// too: these files hold several modules each, which is what phys/probes/rtl/probe_imem.sv does.
/* verilator lint_off DECLFILENAME */
/* verilator lint_off WIDTH */
/* verilator lint_off DECLFILENAME */

// ---------------------------------------------------------------------------------------------
// adpll_loop: counter-based phase detector + PI loop filter.
//
// Structure taken from the only standard-cell ADPLL on Tiny Tapeout, verbatim from its README
// (research/ideation/round2/research/sources/ADPLL/govardhana-adpll-ttsky26c.md):
//   "A Gray-coded counter in the DCO clock domain counts oscillator cycles; the count crosses
//    into the reference domain through a two-stage synchronizer and is sampled once every 16
//    reference cycles. The difference between consecutive samples is the measured cycle count
//    for that window, and comparing it against N * 16 gives a signed frequency error."
//   "A PI filter integrates the error into the tuning word. The coarse bits of the word select
//    one of 16 taps along the ring ... the integrator averages this dither into the intended
//    mean frequency."
//   "the lock detector ... accumulates the error over groups of four windows and asserts lock
//    only after eight consecutive groups land within two counts of target."
//
// Two deviations from that description, both deliberate and both argued in the write-up:
//  1. The count is held *stable* while it crosses instead of free-running. A Gray-coded counter
//     crossing free-running gives a value that is metastable at the instant of the crossing and
//     then keeps counting, so the sampled value is a race. Here the DCO-domain counter freezes on
//     request and the request/acknowledge handshake is what guarantees at least two destination
//     clocks of stability. Same CDC, one extra flop pair, no ambiguity.
//  2. The proportional path moves the *fine* delay-cell word and the integral path moves the tap.
//     This is the assignment the source design uses -- coarse bits on the tap, the remaining bits
//     on a finer chain -- and it is kept because this file's structure is copied rather than
//     re-derived. **It is a property of this probe, not a recommendation, and the library's own
//     delay table argues the other way round.** Comparing the two words' half-step bounds: the tap is one
//     `sg13cmos5l_inv_1` at 32.6 ps typ, so half a step is 16.30 ps = 0.652 % of the 2.5 ns
//     half-loop at 200 MHz and 1.304 % at 400 MHz; the delay word is one
//     `sg13cmos5l_dlygate4sd1_1` at 140.3 ps, so half a step is 70.17 ps = 2.807 % of the same
//     half-loop and 5.614 % at 400 MHz -- 4.3x too coarse by the very test the tap fails. A
//     memoryless path can only null an error down to its own step, so the *smaller* LSB is the only
//     candidate for it: the tap. That is the ordering `probe_adpll_dco.v:103-106` states in its own
//     words ("the log's fine trim is the inverter tap and this bank is the *coarse* trim on top of
//     it"), so what this file does is inverted with respect to the delay table, and the honest
//     consequence is that the proportional term here moves the *coarse* word: it quantises to
//     2.807 % of the half-loop at 200 MHz, cannot null a smaller error, and below one delay cell
//     does not move at all. What this file's arithmetic check tests is the *other* property -- that
//     the term is exactly proportional, with no offset and no dead zone -- which is a different
//     question from which word it is on. See research/ideation/round2/research/adpll/
//     propshift-check.py, which reads the line below out of this file rather than transcribing it.
//
// WIN = 16 reference cycles, as in the source.
// ---------------------------------------------------------------------------------------------
module adpll_loop #(
    parameter int NB       = 5,     // multiplier input width: N up to 2**NB-1
    parameter int WIN      = 16,    // reference cycles per measurement window
    parameter int NTAPW    = 7,     // tap word width, must match adpll_dco
    parameter int NFINE    = 4,     // fine word width, must match adpll_dco
    parameter int LOCKGRP  = 4,     // windows per lock group
    parameter int LOCKOK   = 8      // consecutive good groups before lock
) (
    input  wire                 ref_clk,      // the 50 MHz project clock
    input  wire                 rst_n,
    input  wire                 dco_clk,      // generated on die
    input  wire [NB-1:0]        n_cfg,        // requested multiplier
    input  wire [1:0]           gain_sel,     // 00 default, 01 fast, 10 slow, 11 aggressive
    output logic [NTAPW-1:0]    tap,
    output logic [NFINE-1:0]    fine,
    output logic                locked,
    output logic                meas_valid,   // one ref cycle pulse: `meas` below is fresh
    output logic signed [NB+3:0] meas_err     // measured - target, in DCO counts per window
);

  // tap 0 would be a zero-length loop, so reset to all ones (shortest reachable loop is 1)
  localparam logic [NTAPW-1:0] TAP_RESET = {NTAPW{1'b1}};

  // --- target ------------------------------------------------------------------------------
  wire [NB-1:0] target = (n_cfg == {NB{1'b0}}) ? NB'(4) : n_cfg;   // 0 selects N = 4
  // N * WIN in a NB+4-bit word: 16 * (2**NB-1) < 2**(NB+4) for every NB, so this cannot wrap.
  // Sized via localparam vectors rather than N'(expr) casts: a cast here reads as an addition in
  // some tools, and a probe whose width behaviour is ambiguous is not a probe.
  // NB+4 bits: WIN * target summed WIN times. Written as a loop rather than a multiply so the
  // width of every intermediate is the width of the result; yosys folds the constant WIN anyway.
  logic [NB+3:0] target_win;
  always_comb begin
    target_win = '0;
    for (int k = 0; k < WIN; k++) target_win = target_win + {{(NB + 2){1'b0}}, target};
  end

  // --- the three gains, as a shift amount ---------------------------------------------------
  // Source: "two further pins select one of four PI gain presets, trading lock speed against
  // settling behaviour".
  logic [3:0] prop_shift;
  always_comb begin
    case (gain_sel)
      2'd1:    prop_shift = 4'd3;
      2'd2:    prop_shift = 4'd4;
      2'd3:    prop_shift = 4'd2;
      default: prop_shift = 4'd3;
    endcase
  end

  // --- reference-domain window counter -------------------------------------------------------
  logic [4:0] wcnt;
  logic       win_stb;
  always_ff @(posedge ref_clk or negedge rst_n) begin
    if (!rst_n) wcnt <= 5'd0;
    else if (wcnt == 5'(WIN - 1)) wcnt <= 5'd0;
    else wcnt <= wcnt + 5'd1;
  end
  assign win_stb = (wcnt == 5'(WIN - 1));

  // --- CDC request/acknowledge, ref -> dco ---------------------------------------------------
  logic req, req_s1, req_s2, dco_busy;
  always_ff @(posedge ref_clk or negedge rst_n) begin
    if (!rst_n) begin req <= 1'b0; req_s1 <= 1'b0; req_s2 <= 1'b0; end
    else begin req <= win_stb; req_s1 <= req; req_s2 <= req_s1; end
  end

  // --- DCO-domain Gray counter, frozen while a sample is in flight ---------------------------
  // Width NB+5 so it cannot wrap inside one window at N = 2**NB-1 with margin: 16*(N+1) < 2**(NB+5)
  // for every NB this probe uses.
  localparam int CW = NB + 5;
  logic [CW-1:0] dco_cnt, dco_cnt_hold;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) dco_cnt <= {CW{1'b0}};
    else if (!dco_busy) dco_cnt <= dco_cnt + {{CW - 1{1'b0}}, 1'b1};
  end
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) dco_busy <= 1'b0;
    else dco_busy <= req_s2;             // freeze on the request, release when it drops
  end
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) dco_cnt_hold <= {CW{1'b0}};
    else if (dco_busy) dco_cnt_hold <= dco_cnt;   // held >= 2 dco cycles before `done`
  end

  // --- CDC acknowledge, dco -> ref, with the held count through two flops --------------------
  logic done, done_s1, done_s2;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) done <= 1'b0;
    else done <= dco_busy && !req_s2;    // one dco cycle after the freeze
  end
  logic [CW-1:0] cnt_s1, cnt_s2;
  always_ff @(posedge ref_clk or negedge rst_n) begin
    if (!rst_n) begin done_s1 <= 1'b0; done_s2 <= 1'b0; cnt_s1 <= {CW{1'b0}}; cnt_s2 <= {CW{1'b0}}; end
    else begin
      done_s1 <= done; done_s2 <= done_s1;
      cnt_s1  <= dco_cnt_hold; cnt_s2 <= cnt_s1;
    end
  end
  // Gray decode of the held count (the source uses a Gray counter; the decode belongs here).
  // CW = NB+5 is two bits wider than the error word on purpose: the counter must not wrap inside a
  // window, and the top two bits are then simply never read.
  /* verilator lint_off UNUSEDSIGNAL */
  wire [CW-1:0] cnt_dec = {^cnt_s2[CW-1:5], cnt_s2[CW-2:0]};
  /* verilator lint_on UNUSEDSIGNAL */

  // --- error, PI filter, tap dither ---------------------------------------------------------
  // Declared *before* the combinational block that assigns them, which is not a style preference:
  // any Verilog-2001 reader, and Verilator, accept use-before-declaration inside a module, but the
  // flow's own frontend does not. `read_slang` rejected this file outright with five errors of the
  // form "identifier 'prop_term' used before its declaration" (2026-10-05, the queued
  // `make synth-probe` run: adpll-loop, adpll-capture-8, adpll-capture-18 and adpll-all-8 all FAILED
  // with "ERROR: Compilation failed" while the three adpll-dco-* rows, which do not include this
  // file, synthesised). Moving the declarations up is the whole fix; nothing else about the logic
  // changes.
  logic signed [NB+3:0] err_r;
  logic                  use_sample;
  logic [NB+3:0]        prev_cnt;
  logic [3:0]           grp_ok;
  logic [3:0]           grp_nok;
  // Proportional term: the error scaled by the selected gain, used as the fine word's delta.
  // err_r is NB+4 bits and prop_shift is 2..4, so the shift below is the only arithmetic here.
  // The width waiver below is still needed and is why: prop_term is CW+1 = NB+6 bits wide because
  // the sign extension has to reach the top of the word, but only its low NFINE bits reach the DCO,
  // so the rest are unused by construction. (There used to be a PSMAX_V parameter here; it computed the wrong shift amount and
  // nothing else needed it, so it is removed and the waiver stays.)
  /* verilator lint_off UNUSEDSIGNAL */
  logic signed [CW:0] prop_term;
  /* verilator lint_on UNUSEDSIGNAL */
  logic [NFINE-1:0]    prop_step;
  wire logic [4:0]       sh_v    = {1'b0, prop_shift};

  always_comb begin
    // Arithmetic right shift by sh_v: the NB+4-bit signed error scaled by 2**-sh_v, which is what a
    // proportional term *is*. Two wrong shifts came out of this line, both found by lint-plus-read
    // and neither of which lint could have caught:
    //  * the first shifted *left* by 7..9 in an NB+6-bit word, which zeroes the low NFINE bits for
    //    every representable err_r, so prop_step was identically zero and the fine word never
    //    moved;
    //  * the second replaced `<<<` with `>>>` but left the amount at (PSMAX_V - sh_v) == NB+6-sh_v,
    //    i.e. 7..9 again. That is a *saturated* term: prop_step took two values (0 or 15; four at
    //    the slowest gain) instead of sixteen, so the fine path was a bang-bang, not a
    //    proportional one.
    //    `PSMAX_V` existed only to compute that amount and is gone.
    // `$signed` is load-bearing, not decoration: a concatenation is *unsigned*, so a bare >>> on it
    // is a logical shift. The low NFINE bits come out the same either way (NB+6-sh_v >= NFINE for
    // every gain here, so the sign fill lands above the bits that are read), but the word that
    // feeds the DCO should be arithmetic by construction rather than by coincidence.
    //
    // What it computes, tabulated for every representable err_r and every gain:
    // prop_step takes all 16 values at all three
    // shifts, and is exactly `(err_r >> sh_v)` truncated to the fine word, with no offset and no
    // dead zone. It is monotone -- and so never wraps -- for |err_r| <= 15 << sh_v; past that the
    // 4-bit word wraps rather than saturating, so err_r = +-64 at the aggressive gain (sh_v = 2)
    // gives prop_step = 0. That wrap is a truncation of a proportional term into a 4-bit trim,
    // which is what this trim is; it is not the saturated two-valued term of the earlier version. A product
    // design would saturate here; this probe does not, and saying so is cheaper than pretending the
    // fine word's range is unbounded.
    prop_term = $signed({{2{err_r[NB + 3]}}, err_r}) >>> sh_v;
    prop_step = prop_term[NFINE - 1:0];
  end

  assign use_sample = done_s2 && !done_s1;    // the single cycle the sample is fresh

  always_ff @(posedge ref_clk or negedge rst_n) begin
    if (!rst_n) begin
      meas_err <= '0; meas_valid <= 1'b0; tap <= TAP_RESET; fine <= '0; locked <= 1'b0;
      prev_cnt <= '0; grp_ok <= '0; grp_nok <= '0;
    end else begin
      meas_valid <= use_sample;
      if (use_sample) begin
        // measured cycles in this window, corrected for the counter's own +/-1 sampling phase
        // all three terms are NB+4 bits, so the subtraction is bit-identical in two's complement
        err_r    <= cnt_dec[NB + 3:0] - prev_cnt - {1'b0, target_win};
        prev_cnt <= cnt_dec[NB + 3:0];
        // proportional path: the fine word. err > 0 means "too few DCO cycles in the window",
        // i.e. too slow, so the loop must be lengthened.
        // NB this is the *coarse* word in this library -- one step is 140.3 ps, 4.3x the tap's
        // 32.6 ps -- so the delay table puts the proportional term on `tap` instead. The header
        // says why this file keeps the source design's assignment anyway; what is checked here is
        // that the step is proportional to the error, not which word carries it.
        fine <= fine - prop_step;
        // integral path: bang-bang on the tap, which dithers (see the log, section 3). This is the
        // word the delay table would put the *proportional* term on.
        if (err_r > 0)      tap <= tap + {{(NTAPW - 1){1'b0}}, 1'b1};
        else if (err_r < 0) tap <= tap - {{(NTAPW - 1){1'b0}}, 1'b1};
        // lock detector: LOCKGRP windows per group, LOCKOK consecutive good groups
        // NOT a masked-and-zeros test: `x & 0` is 0 for every x, so this branch was dead and
        // `locked` could never assert. What the predicate now says is
        // |err_r| >= 4 **or** err_r < 0, because err_r[NB+3] is the sign bit: a two's-complement
        // error of -1..-3 has it set. The header's "within two counts" is `!err_r[NB+2:2]`, which is
        // a strictly narrower predicate. The code deliberately keeps the wider one.
        if (|(err_r[NB + 3:2] & {(NB + 2){1'b1}})) begin
          if (grp_ok == 4'(LOCKGRP - 1)) begin
            grp_ok <= 4'd0;
            if (grp_nok == 4'(LOCKOK - 1)) begin grp_nok <= 4'd0; locked <= 1'b1; end
            else grp_nok <= grp_nok + 4'd1;
          end else grp_ok <= grp_ok + 4'd1;
        end else begin
          grp_ok  <= '0;
          grp_nok <= '0;
          locked  <= 1'b0;
        end
        meas_err <= err_r;
      end
    end
  end

endmodule
