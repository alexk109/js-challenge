// SPDX-License-Identifier: Apache-2.0
//
// adpll_flow_probe: the top for phys/cfg/adpll-probe.json. Its only job is to answer the flow
// question: **can LibreLane/OpenSTA time a clock this design generates
// on die?** Nothing else about it is a product decision.
//
// What it contains, and why:
//   * one `adpll_dco` (phys/probes/rtl/probe_adpll_dco.v): an N=31 inverter ring closed through a
//     5-level tap-select mux tree. N=31 rather than 127 so the probe routes inside a normal flow
//     runtime; the area probe prices the real 127-stage version separately (metrics/synth.csv,
//     row adpll-dco-127).
//   * 8 flops clocked by `dco_clk`: a shift register plus a synchroniser. Enough that the flow
//     cannot decide the question is vacuous ("no flops in that domain"), few enough to read.
//   * a 2-flop synchroniser in each direction between the reference and DCO domains, so the run
//     produces a CDC structure rather than a claim about one.
//   * `tap` and `fine` held in reference-domain registers. That is the ADPLL's real structure and
//     it is a **quasi-static** crossing: the bits change slowly and the DCO tolerates any
//     intermediate value because the loop is dithering. STA cannot tell that from a data crossing,
//     so src/adpll.sdc declares the domains asynchronous and the write-up records that the tap bus
//     is left unconstrained on purpose.
//
// `dco_clk` is brought out on `uo_out[0]` so the SDC has a name it can reach in a *flattened*
// netlist, where hierarchical paths from the pre-synthesis top are gone. Two clocks are then
// attempted on it in src/adpll.sdc: an independent `create_clock` (the opentrng/ptrng pattern) and
// a `create_generated_clock`. Both are in `catch` blocks, so one run reports which the tool
// accepts instead of stopping at the first refusal.
//
// Simulation note: this top contains a combinational loop and will not simulate to time. That is
// not a defect of this probe; it is the behaviour govardhana's README documents ("the free-running
// ring forms a zero-delay loop in the PDK functional models under Icarus, which freezes simulation
// time and makes any GL run hang regardless of the testbench").

`default_nettype none

module adpll_flow_probe #(
    parameter int N     = 31,   // ring length; needs 2**NTAPW > N, i.e. NTAPW >= 5 here
    parameter int NTAPW = 5,
    parameter int NFINE = 2
) (
    input  wire                clk,      // the 50 MHz project clock, the only real clock
    input  wire                rst_n,
    input  wire [7:0]          ui_in,
    output wire [7:0]          uo_out
);

  wire [NTAPW-1:0] tap_w;      // the DCO's copy of the tap, driven by the reference-domain register
  wire             dco_clk;

  // --- reference domain: the tap/fine registers and a slow counter ----------------------------
  reg [NTAPW-1:0] tap_r;
  reg [NFINE-1:0] fine_r;
  reg [3:0]       cnt_r;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tap_r  <= {NTAPW{1'b1}};   // shortest reachable loop; a real boot sequence sweeps this
      fine_r <= {NFINE{1'b0}};
      cnt_r  <= 4'd0;
    end else begin
      tap_r  <= ui_in[NTAPW-1:0];
      fine_r <= ui_in[NTAPW +: NFINE];   // every ui_in bit is used; see the ports above
      cnt_r  <= cnt_r + 4'd1;
    end
  end

  // --- the oscillator ------------------------------------------------------------------------
  adpll_dco #(.N(N), .NTAPW(NTAPW), .NFINE(NFINE)) u_dco (
      .tap(tap_r), .fine(fine_r), .dco_clk(dco_clk), .tap_o(tap_w));

  // --- DCO domain: 8 flops on the generated clock --------------------------------------------
  // A shift register, so the net has real fanout and the domain has real endpoints to time.
  reg [7:0] shf;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) shf <= 8'd0;
    else        shf <= {shf[6:0], shf[7] ^ shf[5]};
  end

  // --- 2-flop synchroniser, reference -> DCO --------------------------------------------------
  reg [3:0] cnt_s1, cnt_s2;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) begin cnt_s1 <= 4'd0; cnt_s2 <= 4'd0; end
    else begin cnt_s1 <= cnt_r; cnt_s2 <= cnt_s1; end
  end

  // --- 2-flop synchroniser, DCO -> reference --------------------------------------------------
  reg [7:0] shf_s1, shf_s2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin shf_s1 <= 8'd0; shf_s2 <= 8'd0; end
    else begin shf_s1 <= shf; shf_s2 <= shf_s1; end
  end

  // All of shf, cnt_s2 and shf_s2 reach uo_out, so no bit is dangling; the two synchronisers are the
  // CDC the SDC cuts.
  wire _unused_ok = &{1'b0, ui_in[NTAPW + NFINE +: 8 - NTAPW - NFINE], cnt_s2[3:1], shf_s2[6:0]};
  assign uo_out[0] = dco_clk;        // the clock itself, so the SDC has a stable name for it
  assign uo_out[1] = shf[0];         // DCO domain
  assign uo_out[2] = shf[7];         // DCO domain
  assign uo_out[3] = cnt_s2[0];      // DCO domain, arrived through a synchroniser
  assign uo_out[4] = cnt_r[0];       // reference domain
  assign uo_out[5] = tap_r[0];       // reference domain
  assign uo_out[6] = fine_r[0];      // reference domain
  assign uo_out[7] = shf_s2[7];      // reference domain, arrived through a synchroniser
  // tap_o is brought out on a wire so the DCO's tap-output port has a real connection; it is the
  // same value as tap_r and is not timed separately.
  wire _unused = &{1'b0, tap_w, 1'b0};

endmodule
