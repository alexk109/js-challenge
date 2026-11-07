// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Probe: the witness block of a "self-attesting
// emulator".
//
// Three things a test engineer wants on the die, not only in the host:
//   1. an image digest over everything the host wrote (which program is in there)
//   2. an execution digest over the fetch stream and the pin edges (what ran)
//   3. a pin-edge shadow: the value the datapath intended beside the value on the
//      pad, with a first-violation log naming pin and tick
// No protocol logic. The library has no enable flop (docs/2026-09-22-setup-and-
// architecture-review.md:69), so every register here is updated every cycle with
// a mux in front, which is what this probe measures.

module probe_witness #(
    parameter int NP = 24,  // pins watched (8 uio + 8 ui_in + 8 uo_out)
    parameter int TW = 16,  // tick timestamp width
    parameter int W  = 32   // digest width
) (
    input  logic              clk,
    input  logic              rst_n,
    // --- digest configuration: polynomial and seed are registers, so the primitive
    //     is parameterized by firmware (docs/design-notes.md sec. 1, point 3)
    input  logic              cfg_we,
    input  logic [W-1:0]      cfg_poly,
    input  logic [W-1:0]      cfg_seed,
    // --- image digest port: one word per cycle from the host loader
    input  logic              img_init,
    input  logic              img_we,
    input  logic [W-1:0]      img_din,
    input  logic [W-1:0]      img_expect,   // the value the host computed
    input  logic              img_check,    // one-cycle compare request
    // --- execution digest port: one fetched instruction word per cycle
    input  logic              exe_we,
    input  logic [W-1:0]      exe_din,
    // --- pin shadow
    input  logic [NP-1:0]     pad_level,    // what the pad receiver sees
    input  logic [NP-1:0]     want_level,   // what the datapath intended
    input  logic [NP-1:0]     want_oe,      // and its output enables
    input  logic [TW-1:0]     tick,
    input  logic              shadow_en,    // run the comparison (0 = witness off)
    // --- outputs
    output logic [W-1:0]      img_digest,
    output logic [W-1:0]      exe_digest,
    output logic              img_bad,
    output logic              viol_any,
    output logic [$clog2(NP)-1:0] viol_pin,
    output logic [TW-1:0]     viol_tick,
    output logic [7:0]        viol_count,   // saturating
    output logic [NP*TW-1:0]  edge_ts_flat  // per-pin last-edge records, read out so the
                                            // synthesis tool cannot delete them
);
  localparam int PIW = (NP > 1) ? $clog2(NP) : 1;

  // --- 1 + 2. two digests, same linear update, one word per cycle
  logic [W-1:0] poly_q, seed_q;
  logic [W-1:0] img_q, exe_q;
  logic [W-1:0] img_fb, img_shift, exe_fb, exe_shift;
  // declared before use: slang rejects an identifier used before its declaration
  logic [NP-1:0] prev_level, edge_flag, mismatch, exe_vec;

  assign img_fb    = img_q[W-1] ^ img_din[0];
  assign img_shift = {img_q[W-2:0], 1'b0} ^ {img_din[W-1:1], 1'b0} ^ (img_fb ? poly_q : {W{1'b0}});
  // the execution digest covers the pin stream as well as the fetch stream: one bit per pin
  // per cycle, zero-extended into the word
  wire [W-1:0] exe_word = exe_din ^ {{(W-NP){1'b0}}, exe_vec};
  assign exe_fb    = exe_q[W-1] ^ exe_word[0];
  assign exe_shift = {exe_q[W-2:0], 1'b0} ^ {exe_word[W-1:1], 1'b0} ^ (exe_fb ? poly_q : {W{1'b0}});

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      poly_q <= '0;
      seed_q <= '0;
      img_q  <= '0;
      exe_q  <= '0;
    end else begin
      if (cfg_we) begin
        poly_q <= cfg_poly;
        seed_q <= cfg_seed;
      end
      if (img_init) img_q <= seed_q;
      else if (img_we) img_q <= img_shift;
      if (exe_we)   exe_q <= exe_shift;
    end
  end

  assign img_digest = img_q;
  assign exe_digest = exe_q;

  // --- image digest verdict, registered one cycle after the request
  logic img_bad_q;
  always_ff @(posedge clk) begin
    if (!rst_n) img_bad_q <= 1'b0;
    else        img_bad_q <= img_check & (img_q != img_expect);
  end
  assign img_bad = img_bad_q;

  // --- 3. per-pin last-edge record (timestamp + level in bit 0) and mismatch detection
  logic [TW-1:0] edge_ts [NP];
  integer i;

  always_comb begin
    prev_level = '0;
    edge_flag  = '0;
    mismatch   = '0;
    for (i = 0; i < NP; i = i + 1) begin
      prev_level[i] = edge_ts[i][0];
      edge_flag[i]  = shadow_en & (pad_level[i] != prev_level[i]);
      mismatch[i]   = shadow_en & want_oe[i] & (pad_level[i] != want_level[i]);
    end
    // one bit per pin per cycle folded into the execution digest, so it covers the
    // pin stream as well as the fetch stream
    exe_vec = edge_flag | mismatch | (want_oe & (want_level ^ prev_level));
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (i = 0; i < NP; i = i + 1) edge_ts[i] <= '0;
    end else begin
      for (i = 0; i < NP; i = i + 1)
        if (shadow_en && (edge_flag[i] || mismatch[i]))
          edge_ts[i] <= {tick[TW-1:1], pad_level[i]};
    end
  end

  // --- first-violation log: lowest-numbered mismatching pin wins, sticky, saturating count
  logic [PIW-1:0] viol_pin_d;
  always_comb begin
    viol_pin_d = '0;
    for (i = NP - 1; i >= 0; i = i - 1)
      if (mismatch[i]) viol_pin_d = PIW'(i);
  end

  logic viol_any_q, viol_pin_q, viol_tick_q, viol_count_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      viol_any_q   <= 1'b0;
      viol_pin_q   <= '0;
      viol_tick_q  <= '0;
      viol_count_q <= 8'd0;
    end else if (shadow_en && (|mismatch)) begin
      viol_any_q <= 1'b1;
      if (!viol_any_q) begin
        viol_pin_q  <= viol_pin_d;
        viol_tick_q <= tick;
      end
      if (viol_count_q != 8'hff) viol_count_q <= viol_count_q + 8'd1;
    end
  end

  assign viol_any   = viol_any_q;
  assign viol_pin   = viol_pin_q;
  assign viol_tick  = viol_tick_q;
  assign viol_count = viol_count_q;

  genvar g;
  generate
    for (g = 0; g < NP; g = g + 1) begin : g_flat
      assign edge_ts_flat[g*TW +: TW] = edge_ts[g];
    end
  endgenerate
endmodule
