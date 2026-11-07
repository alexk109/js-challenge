// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Probe: the two on-die parts of a reduced replay module
// ("always-on attestation, windowed replay"):
//
//   1. probe_replay_sig     a 32-bit signature folded once per cycle over the
//                           pin stream and the fetch stream. Zero bandwidth.
//   2. probe_replay_shadow  the pin shadow: every pin's registered intent,
//                           delayed by the measured settling window D, against
//                           the level the pad receiver sees; plus the window
//                           buffer that holds the receipt for one window.
//   3. probe_replay_window  the window buffer alone, so the receipt's storage
//                           is measured rather than assumed.
//   4. probe_replay_module  1 + 2 together: the block the kill criterion is
//                           about.
//
// Three house rules, all from the repo's own area history:
//   * the sg13cmos5l standard-cell library has no enable flop
//     (docs/2026-09-22-setup-and-architecture-review.md:69-70), so every
//     register here updates every cycle behind a mux. That is what is being
//     measured; do not "optimise" it into an enable and the number will move.
//   * every register is read out on a host port. probe_witness.sv under-counted
//     by 3x once because its edge timestamps were write-only and synthesis
//     deleted them (research/ideation/round2/testeng-1.md:239-243). Anyone
//     re-measuring a block with an array here owes that check: compare
//     seq_cells with the flops the block obviously needs.
//   * slang rejects an identifier used before its declaration, so every
//     signal is declared before first use.

// ---------------------------------------------------------------- part 1
// The 32-bit per-cycle signature.
module probe_replay_sig #(
    parameter int NP = 18,  // pins folded into the signature (18 = MVP pin map)
    parameter int W  = 32   // signature width
) (
    input  logic         clk,
    input  logic         rst_n,
    // polynomial and seed are registers so the primitive is programmable
    input  logic         cfg_we,
    input  logic [W-1:0] cfg_poly,
    input  logic [W-1:0] cfg_seed,
    // window control: a receipt is checked against a signature that restarts
    // at the window boundary
    input  logic         sig_clear,
    input  logic         sig_snapshot,
    // one fetched instruction word per cycle (the execution half)
    input  logic         exe_we,
    input  logic [W-1:0] exe_din,
    // the pin stream: one bit per pin per cycle, edge and mismatch (zero-extended)
    input  logic [NP-1:0] pin_stream,
    // read out so synthesis cannot delete the accumulator
    output logic [W-1:0] sig_live,
    output logic [W-1:0] sig_snap
);
  logic [W-1:0] poly_q, seed_q, sig_q, snap_q;
  logic [W-1:0] word, fb, shifted;

  assign word    = exe_din ^ {{(W - NP) {1'b0}}, pin_stream};
  assign fb      = sig_q[W-1] ^ word[0];
  assign shifted = {sig_q[W-2:0], 1'b0} ^ {word[W-1:1], 1'b0}
                 ^ (fb ? poly_q : {W {1'b0}});

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      poly_q <= '0;
      seed_q <= '0;
      sig_q  <= '0;
      snap_q <= '0;
    end else begin
      if (cfg_we) begin
        poly_q <= cfg_poly;
        seed_q <= cfg_seed;
      end
      // always-on: the fold happens every cycle, behind a mux, not behind an
      // enable. sig_clear restarts at a window boundary.
      if (sig_clear)     sig_q <= seed_q;
      else if (exe_we)   sig_q <= shifted;
      else               sig_q <= shifted;
      if (sig_snapshot) snap_q <= sig_q;
    end
  end

  assign sig_live = sig_q;
  assign sig_snap = snap_q;
endmodule
// ---------------------------------------------------------------- part 2
// The pin shadow: the primary engine's registered (oe, val) intent for every
// pin, delayed by the measured settling window D, against the level the pad
// receiver sees. Same comparison the a3-safety probe makes, minus the second
// (shadow) engine: C3's shadow compares one engine against the wire, which is
// what makes MATCH mean "the silicon did what the spec says, given what it
// read" (C3.md Fact 1).
//
// The window buffer: while the capture window is armed, the pin vector is
// written into a DEPTH-deep RAM with a cycle counter per entry. That is the
// storage a dumped receipt has to come out of; C3.md prices the receipt in
// bytes, not in flops, so this is the number the byte figures need.
module probe_replay_shadow #(
    parameter int NP  = 18,   // pins watched
    parameter int D   = 3,    // input-synchronizer latency / settling window
    parameter int TW  = 20,   // cycle-counter width
    parameter int DEPTH = 512, // window buffer entries (one pin vector + tick each)
    localparam int AW = (DEPTH < 2) ? 1 : $clog2(DEPTH)
) (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          shadow_en,
    input  logic          cnt_clear,
    input  logic [NP-1:0] want_oe,     // datapath intent (what it is driving)
    input  logic [NP-1:0] want_val,
    input  logic [NP-1:0] line_level,  // sampled pad level, post-synchronizer
    // --- window buffer
    input  logic          win_arm,      // open the capture window
    input  logic          win_close,
    output logic          win_full,
    output logic [15:0]   win_count,    // entries written
    // --- shadow status
    output logic [NP-1:0] div_line,
    output logic          any_div,
    output logic [7:0]    div_count,    // saturating
    // --- first-divergence record
output logic [4:0]    div_pin,
  output logic          div_tick_any,
  // --- window-buffer read port: see the note at the bottom of this module
  input  logic [AW-1:0] win_raddr,
  output logic [TW+NP-1:0] win_rdata
);
  localparam int PW = (NP < 2) ? 1 : $clog2(NP);

  logic [NP-1:0] want_oe_q, want_val_q, frozen_oe, frozen_val;
  logic [NP-1:0] want_oe_d [D+1];
  logic [NP-1:0] want_val_d [D+1];
  logic [NP-1:0] line_neq;
  logic          new_div;
  logic [TW-1:0] cyc;
  logic [4:0]    pin_d;
  logic          found_d;

  assign new_div = |line_neq;
  assign any_div = new_div;

  always_comb begin
    line_neq = '0;
    for (int p = 0; p < NP; p++) begin
      // only meaningful where the engine was actually driving
      line_neq[p] = want_oe_d[D][p] & (want_val_d[D][p] != line_level[p]);
    end
  end

  // lowest-numbered mismatching pin wins
  always_comb begin
    found_d = 1'b0;
    pin_d   = '0;
    for (int p = NP - 1; p >= 0; p--) begin
      if (line_neq[p]) begin
        found_d = 1'b1;
        pin_d   = PW'(p);
      end
    end
  end

  // --- window buffer: DEPTH entries of {tick, pin vector}, written while armed
  logic [TW+NP-1:0] wbuf [DEPTH];
  logic [AW:0]      wptr;
  logic             win_arm_q;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      want_oe_q <= '0; want_val_q <= '0; frozen_oe <= '0; frozen_val <= '0;
      div_line  <= '0; div_count <= '0; cyc <= '0;
      div_pin   <= '0; div_tick_any <= 1'b0;
      wptr <= '0; win_count <= '0; win_full <= 1'b0; win_arm_q <= 1'b0;
      for (int d = 0; d <= D; d++) begin want_oe_d[d] <= '0; want_val_d[d] <= '0; end
    end else begin
      want_oe_q  <= want_oe;
      want_val_q <= want_val;
      frozen_oe  <= want_oe;
      frozen_val <= want_val;

      // the D-tap delay line the line comparison needs
      for (int d = 0; d < D; d++) begin
        want_oe_d[d]  <= (d == 0) ? want_oe_q  : want_oe_d[d-1];
        want_val_d[d] <= (d == 0) ? want_val_q : want_val_d[d-1];
      end
      want_oe_d[D]  <= want_oe_d[D-1];
      want_val_d[D] <= want_val_d[D-1];

      if (cnt_clear) begin
        div_line <= '0; div_count <= '0; cyc <= '0;
      end else begin
        cyc <= cyc + 1'b1;
      end

      if (shadow_en && new_div) begin
        div_line  <= div_line | line_neq;
        div_count <= (div_count == 8'hff) ? 8'hff : div_count + 8'd1;
      end
      if (shadow_en && new_div && found_d && !div_tick_any) begin
        div_pin      <= pin_d;
        div_tick_any <= 1'b1;
      end

      // window buffer
      if (win_arm && !win_arm_q) begin
        wptr      <= '0;
        win_count <= '0;
        win_full  <= 1'b0;
      // wptr is AW+1 bits, so wptr[AW] goes high exactly at wptr == DEPTH: that
      // single bit is the "buffer not full" test. Do NOT write it as
      // (wptr[AW-1:0] < AW'(DEPTH)) -- that truncates DEPTH to zero in AW bits,
      // the write enable is then never true, and opt_clean deletes the whole
      // array (measured: 58 um2 instead of ~1 M um2, this probe, 2026-10-05).
      end else if (win_arm && (wptr[AW] == 1'b0)) begin
        wbuf[wptr[AW-1:0]] <= {cyc, line_level};
        wptr      <= wptr + 1'b1;
        win_count <= win_count + 16'd1;
      end
      win_arm_q <= win_arm;
      if (win_full || win_count == DEPTH) win_full <= 1'b1;
    end
  end

  // The buffer is written-only above, so export a host read port over it: without
  // one, opt_clean deletes the whole array and the area comes back 3x too small
  // (the bug probe_witness.sv had, testeng-1.md:239-243).
  assign win_rdata = wbuf[win_raddr];
endmodule

// ---------------------------------------------------------------- part 3
// The window buffer alone, so the receipt's storage cost is measured on its own
// and not inferred from the combined block. Same array as part 2's, exposed
// with nothing else in the way.
module probe_replay_window #(
    parameter int TW    = 20,  // cycle-counter width
    parameter int NP    = 18,  // pin vector width stored beside the tick
    parameter int DEPTH = 512, // entries
    localparam int AW   = (DEPTH < 2) ? 1 : $clog2(DEPTH)
) (
    input  logic           clk,
    input  logic           rst_n,
    input  logic           win_arm,
    input  logic [TW-1:0]  cyc,
    input  logic [NP-1:0]  din,
    output logic           win_full,
    output logic [15:0]    win_count,
    input  logic [AW-1:0]  win_raddr,
    output logic [TW+NP-1:0] win_rdata
);
  logic [TW+NP-1:0] wbuf [DEPTH];
  logic [AW:0]      wptr;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wptr <= '0; win_count <= '0; win_full <= 1'b0;
    end else if (!win_arm) begin
      wptr <= '0; win_count <= '0; win_full <= 1'b0;
    end else if (!wptr[AW]) begin   // wptr[AW] is set at wptr == DEPTH; see the note
                                 // in probe_replay_shadow about AW'(DEPTH)
      wbuf[wptr[AW-1:0]] <= {cyc, din};
      wptr      <= wptr + 1'b1;
      win_count <= win_count + 16'd1;
    end else begin
      win_full <= 1'b1;
    end
  end

  assign win_rdata = wbuf[win_raddr];
endmodule

// ---------------------------------------------------------------- part 4
// signature + pin shadow, the block the kill criterion prices. The two are the
// only always-on parts of C3's reduced form; the window buffer is part 3 and is
// reported separately, because C3.md's 40,777 um2 estimate is the witness block
// (two digests + a 24-pin edge file + a violation log) and not this pair.
module probe_replay_module #(
    parameter int NP  = 18,
    parameter int D   = 3,
    parameter int TW  = 20,
    parameter int W   = 32
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic             cfg_we,
    input  logic [W-1:0]     cfg_poly,
    input  logic [W-1:0]     cfg_seed,
    input  logic             sig_clear,
    input  logic             sig_snapshot,
    input  logic             exe_we,
    input  logic [W-1:0]     exe_din,
    input  logic             shadow_en,
    input  logic             cnt_clear,
    input  logic [NP-1:0]    want_oe,
    input  logic [NP-1:0]    want_val,
    input  logic [NP-1:0]    line_level,
    output logic [W-1:0]     sig_live,
    output logic [W-1:0]     sig_snap,
    output logic [NP-1:0]    div_line,
    output logic             any_div,
    output logic [7:0]       div_count,
    output logic [4:0]       div_pin,
    output logic             div_tick_any
);
  logic [NP-1:0] pin_stream;
  always_comb pin_stream = want_val ^ want_oe;

  probe_replay_sig #(.NP(NP), .W(W)) u_sig (
      .clk(clk), .rst_n(rst_n),
      .cfg_we(cfg_we), .cfg_poly(cfg_poly), .cfg_seed(cfg_seed),
      .sig_clear(sig_clear), .sig_snapshot(sig_snapshot),
      .exe_we(exe_we), .exe_din(exe_din), .pin_stream(pin_stream),
      .sig_live(sig_live), .sig_snap(sig_snap)
  );

  probe_replay_shadow #(.NP(NP), .D(D), .TW(TW), .DEPTH(2)) u_shadow (
      .clk(clk), .rst_n(rst_n), .shadow_en(shadow_en), .cnt_clear(cnt_clear),
      .want_oe(want_oe), .want_val(want_val), .line_level(line_level),
      .win_arm(1'b0), .win_close(1'b0), .win_full(), .win_count(),
      .div_line(div_line), .any_div(any_div), .div_count(div_count),
      .div_pin(div_pin), .div_tick_any(div_tick_any),
      .win_raddr(1'b0), .win_rdata()
  );
endmodule

// ---------------------------------------------------------------- part 5
// The window buffer, second reading: a per-pin edge file instead of a RAM of
// pin vectors. One {tick, level} record per pin, overwritten when that pin
// moves. It costs NP x TW flops instead of DEPTH x (TW + NP), and it is what
// probe_witness.sv already contains; what it cannot do is reconstruct a window
// receipt, because it keeps only the *last* edge of each pin. Priced separately
// because "the pin shadow (window buffer)" does not say which of the
// two is meant.
module probe_replay_edgefile #(
    parameter int NP = 18,
    parameter int D  = 3,
    parameter int TW = 20
) (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          shadow_en,
    input  logic [NP-1:0] want_oe,
    input  logic [NP-1:0] want_val,
    input  logic [NP-1:0] line_level,
    input  logic [TW-1:0] tick,
    output logic [NP-1:0] div_line,
    output logic [7:0]    div_count,
    // per-pin last-edge records, read out so synthesis keeps them
    output logic [NP*TW-1:0] edge_ts_flat
);
  localparam int PW = (NP < 2) ? 1 : $clog2(NP);

  logic [NP-1:0] want_oe_q, want_val_q;
  logic [NP-1:0] want_oe_d [D+1];
  logic [NP-1:0] want_val_d [D+1];
  logic [NP-1:0] line_neq, edge_flag;
  logic [TW-1:0] edge_ts [NP];
  logic [7:0]    count_q;

  always_comb begin
    line_neq  = '0;
    edge_flag = '0;
    for (int p = 0; p < NP; p++) begin
      line_neq[p]  = want_oe_d[D][p] & (want_val_d[D][p] != line_level[p]);
      edge_flag[p] = (line_level[p] != edge_ts[p][0]);
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      want_oe_q <= '0; want_val_q <= '0; div_line <= '0; count_q <= '0;
      for (int d = 0; d <= D; d++) begin want_oe_d[d] <= '0; want_val_d[d] <= '0; end
      for (int k = 0; k < NP; k++) edge_ts[k] <= '0;
    end else begin
      want_oe_q  <= want_oe;
      want_val_q <= want_val;
      for (int d = 0; d < D; d++) begin
        want_oe_d[d]  <= (d == 0) ? want_oe_q  : want_oe_d[d-1];
        want_val_d[d] <= (d == 0) ? want_val_q : want_val_d[d-1];
      end
      want_oe_d[D]  <= want_oe_d[D-1];
      want_val_d[D] <= want_val_d[D-1];

      if (shadow_en) begin
        div_line <= div_line | line_neq;
        if ((count_q != 8'hff) && (|line_neq)) count_q <= count_q + 8'd1;
        for (int p = 0; p < NP; p++)
          if (edge_flag[p] || line_neq[p]) edge_ts[p] <= {tick[TW-1:1], line_level[p]};
      end
    end
  end

  genvar g;
  generate
    for (g = 0; g < NP; g++) begin : g_flat
      assign edge_ts_flat[g*TW +: TW] = edge_ts[g];
    end
  endgenerate
endmodule
