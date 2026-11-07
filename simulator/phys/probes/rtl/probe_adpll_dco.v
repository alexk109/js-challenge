// SPDX-License-Identifier: Apache-2.0
//
// Probe RTL for the ADPLL experiment.
// This is **probe RTL, not product RTL**: it prices the block for
// `make synth-probe` (phys/probes/probes.yaml) and it is the top of the flow probe
// (phys/cfg/adpll-probe.json), whose whole job is to find out whether LibreLane/OpenSTA
// will time a DCO-generated clock at all. Nothing here is intended to be taped out as written.
//
// Three blocks, matching research/ideation/round2/research/adpll/adpll-physics.py:
//   adpll_dco       N-stage inverter ring closed through one N:1 tap-select mux tree,
//                   plus a bank of dlygate4sd1_1 cells switched into the loop as a fine trim.
//                   Model B of the log. N=127 needs 7 mux levels; the log prices that tree at
//                   15-16 % of the loop delay, which is the finding.
//   adpll_loop      counter-based phase detector + PI loop filter, exactly the structure the
//                   only existing TT ADPLL describes (Gray-coded DCO counter, two-stage
//                   synchroniser, one sample every 16 reference cycles, compare with N*16,
//                   coarse taps + dither, lock after 8 groups of 4 windows within 2 counts).
//                   Source: the published Tiny Tapeout ADPLL README.
//   adpll_capture   the thing the probe actually wants: NP input pins sampled on dco_clk,
//                   any change latched with (ref_count, dco_phase) so an edge is located to
//                   one DCO period. A sampling TDC with no delay line.
//
// Two things this file deliberately does NOT do, because both are load-bearing findings:
//   * no phase alignment / no PLL-style phase detector. The log (section 4b) shows the capture
//     use case needs a *calibrated ratio*, not a locked phase, and phase alignment is what buys
//     jitter specifications this application cannot use.
//   * no ring oscillator is instantiated in the loop, so this block is safe to simulate. The DCO
//     is a separate top (`adpll_dco`) and is only instantiated by the flow probe, which never
//     simulates it -- the same decision the govardhana README documents ("the free-running ring
//     forms a zero-delay loop in the PDK functional models under Icarus, which freezes
//     simulation time and makes any GL run hang regardless of the testbench").
//
// Clocked logic on `dco_clk` is written for the flow, not for a linter: the DCO clock is
// generated on die, so a linter cannot recognise it. govardhana's build reports 26 such
// warnings and calls them expected.

`default_nettype none

// UNOPTFLAT is *expected and wanted* here: a ring oscillator is a combinational loop, and that is
// the whole question the flow probe asks. Everything else stays on.
/* verilator lint_off DECLFILENAME */
/* verilator lint_off GENUNNAMED */   // the generate-ifs inside g_fine and g_pad

// ---------------------------------------------------------------------------------------------
// adpll_dco: N inverters in a chain, closed through an N:1 tap-select mux tree, plus a fine
// trim of dlygate4sd1_1 cells. Nothing else. Deliberately free of any flop, so that synthesis
// maps it to pure combinational cells and the area number is the oscillator's area.
// ---------------------------------------------------------------------------------------------
module adpll_dco #(
    parameter int N      = 127,  // inverters in the chain; the tap selects 1..N of them
    parameter int NFINE  = 4,    // fine-trim width, one dlygate4sd1_1 per bit
    parameter int NTAPW  = 7     // tap-select width; N+1 taps must fit
) (
    input  wire  [NTAPW-1:0] tap,    // 1..N inverters in the loop
    input  wire  [NFINE-1:0]  fine,   // delay cells inserted in the loop (see FMAX below)
    output wire               dco_clk,
    output wire [NTAPW-1:0]    tap_o // the tap, on an output, so the flow probe can name a pin
);

  // The chain, closed: tap mux tree -> fine trim -> st[0] -> N inverters -> tree input st[N].
  // sg13cmos5l_inv_1 is the smallest inverter in the library (5.44 um2); the log uses its liberty
  // delay of 22.0 / 32.6 / 52.0 ps, which reproduces docs/solution-space.md:41 (22 / 33 / 52 ps
  // FO1) exactly.
  wire [N:0] st;
  wire       fb;      // tree output, feeds the fine trim
  wire       fine_out; // fine-trim output, feeds the chain head: this is st[0]
  assign st[0] = fine_out;

  genvar g, L;

  genvar i;
  generate
    for (i = 0; i < N; i = i + 1) begin : g_inv
      sg13cmos5l_inv_1 u_inv (.A(st[i]), .Y(st[i + 1]));
    end
  endgenerate

  // Loop-closing tap select: a 2**NTAPW:1 mux tree, NTAPW levels of sg13cmos5l_mux2_1, built as
  // a halving reduction so the depth is a function of NTAPW and visible in the netlist. The log
  // (section 2) prices this tree at 15-16 % of the loop delay and 127 extra cells at NTAPW=7;
  // that is the finding, so it is written as ordinary structure rather than dodged.
  localparam int LEAFW = 1 << NTAPW;
  wire [LEAFW-1:0] lv [0:NTAPW];

  // Pad the leaves up to 2**NTAPW. A generate-if, not a ternary: a ternary on an out-of-range
  // index elaborates st[g] anyway and every tool then warns about it.
  generate
    for (g = 0; g < LEAFW; g = g + 1) begin : g_pad
      if (g <= N) assign lv[0][g] = st[g];
      else        assign lv[0][g] = 1'b0;
    end
    for (L = 0; L < NTAPW; L = L + 1) begin : g_lvl
      for (g = 0; g < (LEAFW >> (L + 1)); g = g + 1) begin : g_mux
        sg13cmos5l_mux2_1 u (.A0(lv[L][2 * g]), .A1(lv[L][2 * g + 1]),
                            .S(tap[L]), .X(lv[L + 1][g]));
      end
    end
  endgenerate

  assign fb = lv[NTAPW][0];

  // Fine trim: one dlygate4sd1_1 per fine bit, in series with the loop, each bypassed by the
  // mux it is selected through. Liberty delay 92.3 / 140.3 / 225.9 ps (the log's table in
  // section 1) -- the smallest element the library has above one inverter, which is why the
  // log's fine trim is the inverter tap and this bank is the *coarse* trim on top of it.
  // sg13cmos5l_dlygate4sd1_1 has pins A -> X (verified against
  // ~/ttsetup/pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/*.lib by
  // research/models/liberty.py's Cell.pin()).
  // NOTE: this bank sits *after* the tap tree on purpose -- putting delay in the tap-select path
  // would put a mux in front of the tap bits and make the loop untimeable.
  // All FMAX banks are elaborated so ch[FMAX] is always driven. A bank with g >= NFINE has its
  // select tied low, which is a bypass: it still costs a delay gate and a mux in the loop. NFINE=4
  // in the area probes, so nothing is bypassed there.
  localparam int FMAX = 4;   // banks elaborated; FMAX - NFINE of them are bypassed
  wire [FMAX:0] ch;
  assign ch[0] = fb;
  generate
    for (g = 0; g < FMAX; g = g + 1) begin : g_fine
      wire via;
      sg13cmos5l_dlygate4sd1_1 u_dly (.A(ch[g]), .X(via));
      // generate-if rather than `wire sel = (g < NFINE) ? fine[g] : 1'b0`: a ternary elaborates
      // fine[g] even when g is past NFINE, and every tool then reports an out-of-range read.
      if (g < NFINE) begin : g_sel
        sg13cmos5l_mux2_1 u_mux (.A0(ch[g]), .A1(via), .S(fine[g]), .X(ch[g + 1]));
      end else begin : g_byp
        sg13cmos5l_mux2_1 u_mux (.A0(ch[g]), .A1(via), .S(1'b0), .X(ch[g + 1]));
      end
    end
  endgenerate

  assign fine_out = ch[FMAX];
  assign dco_clk  = ch[FMAX];
  assign tap_o    = tap;

endmodule
