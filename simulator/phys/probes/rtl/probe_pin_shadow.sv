// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Probe: probe_pin_shadow.
//
// One comparator per pad between a primary engine's pad decision and a second,
// independently programmed "shadow" engine's decision. The shadow engine never
// reaches a pad; it only reaches these comparators.
//
// Two independent comparisons per pin:
//   DECISION  primary (oe,val) vs shadow (oe,val), same clock edge.
//             Catches a fault in the primary datapath, program store, decoder or
//             FIFO that changes what the chip would have driven.
//   LINE      the registered primary decision, delayed by the known input
//             synchronizer latency D, vs the sampled pad level.
//             Catches everything the decision comparison cannot: a pad or mux
//             fault, a short, another device fighting us, a broken clock.
//             D is a compile-time constant because our own measurement says the
//             synchronizer latency is fixed (an echo table follows ui_in[0] by
//             exactly 3 ticks at 50 MHz).
// Reaction: sticky per-pin flags, a first-divergence record {pin, kind, cycle,
// both values}, a saturating counter, and an optional freeze of the pad
// registers so a detected divergence can be made visible on the wire instead of
// only in a status register.
// Method and results: research/ideation/round2/README-safety.md.
module probe_pin_shadow #(
    parameter int NP  = 18,   // pads compared
    parameter int D   = 3,    // input synchronizer latency, ticks
    parameter int TW  = 20,   // cycle-counter width
    parameter int LOGN = 4    // first-divergence log entries
) (
    input  logic            clk,
    input  logic            rst_n,
    input  logic            cfg_we,
    input  logic            cfg_freeze,   // 1: a divergence freezes the pad regs
    input  logic            cnt_clear,
    // primary engine decisions (these reach the pad)
    input  logic [NP-1:0]   a_oe,
    input  logic [NP-1:0]   a_val,
    // shadow engine decisions (never reach a pad)
    input  logic [NP-1:0]   b_oe,
    input  logic [NP-1:0]   b_val,
    // sampled pad levels, post-synchronizer
    input  logic [NP-1:0]   line_level,
    // pad registers owned by this module
    output logic [NP-1:0]   pad_oe,
    output logic [NP-1:0]   pad_val,
    // status
    output logic [NP-1:0]   div_decision,
    output logic [NP-1:0]   div_line,
    output logic            any_divergence,
    output logic [7:0]      div_count,      // saturating
    // first-divergence log
    output logic            log_wr,
    output logic            log_overwrite,
    output logic [4:0]      log_pin,
    output logic [1:0]      log_kind,
    output logic [TW-1:0]   log_cycle,
    output logic [1:0]      log_ab
);
  localparam int PW = (NP < 2) ? 1 : $clog2(NP);  // $clog2(1) is 0: a zero-width port
  localparam int CI = $clog2(LOGN);

  logic [NP-1:0] a_oe_q, a_val_q, frozen_oe, frozen_val;
  logic [NP-1:0] a_oe_d [D+1];
  logic [NP-1:0] a_val_d [D+1];
  logic [NP-1:0] dec_neq, line_neq;
  logic          new_div;
  logic [TW-1:0] cyc;
  logic [CI:0]   wp;
  logic          seen   [LOGN];
  logic [PW-1:0] pin_q  [LOGN];
  logic [1:0]    kind_q [LOGN];
  logic [TW-1:0] cyc_q  [LOGN];
  logic [1:0]    ab_q   [LOGN];
  logic          freeze_q;

  assign new_div      = |(dec_neq | line_neq);
  assign any_divergence = new_div | (|div_decision) | (|div_line);
  assign log_wr       = new_div;
  assign log_pin      = pin_q[wp[CI-1:0]];
  assign log_kind     = kind_q[wp[CI-1:0]];
  assign log_cycle    = cyc_q[wp[CI-1:0]];
  assign log_ab       = ab_q[wp[CI-1:0]];
  assign log_overwrite = seen[wp[CI-1:0]];

  always_comb begin
    for (int p = 0; p < NP; p++) begin
      dec_neq[p]  = (a_oe_q[p] != b_oe[p]) || (a_val_q[p] != b_val[p]);
      // only meaningful where the primary was actually driving
      line_neq[p] = a_oe_d[D][p] & (a_val_d[D][p] != line_level[p]);
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      a_oe_q <= '0; a_val_q <= '0; frozen_oe <= '0; frozen_val <= '0;
      pad_oe <= '0; pad_val <= '0;
      div_decision <= '0; div_line <= '0;
      div_count <= '0; cyc <= '0; wp <= '0; freeze_q <= 1'b0;
      for (int d = 0; d <= D; d++) begin a_oe_d[d] <= '0; a_val_d[d] <= '0; end
      for (int k = 0; k < LOGN; k++) begin
        seen[k] <= 1'b0; pin_q[k] <= '0; kind_q[k] <= '0; cyc_q[k] <= '0; ab_q[k] <= '0;
      end
    end else begin
      if (cfg_we) freeze_q <= cfg_freeze;

      // pad registers, with the optional fail-silent freeze
      if (freeze_q && new_div) begin
        pad_oe  <= frozen_oe;
        pad_val <= frozen_val;
      end else begin
        pad_oe      <= a_oe;
        pad_val     <= a_val;
        frozen_oe   <= a_oe;
        frozen_val  <= a_val;
      end
      a_oe_q  <= pad_oe;      // what the pad actually was given last tick
      a_val_q <= pad_val;

      // the D-tap delay line used by the line comparison
      for (int d = 0; d < D; d++) begin
        a_oe_d[d]  <= (d == 0) ? pad_oe      : a_oe_d[d-1];
        a_val_d[d] <= (d == 0) ? pad_val     : a_val_d[d-1];
      end
      a_oe_d[D]  <= a_oe_d[D-1];
      a_val_d[D] <= a_val_d[D-1];

      // sticky status, saturating counter
      if (new_div) begin
        div_decision <= div_decision | dec_neq;
        div_line     <= div_line     | line_neq;
        div_count    <= (div_count == 8'hff) ? 8'hff : div_count + 8'd1;
      end
      if (cnt_clear) begin
        div_decision <= '0; div_line <= '0; div_count <= '0; cyc <= '0;
      end else begin
        cyc <= cyc + 1'b1;
      end

      // log the lowest-numbered new divergence, once per tick
      if (new_div) begin
        seen[wp[CI-1:0]] <= 1'b1;
        wp <= wp + 1'b1;
        for (int p = 0; p < NP; p++) begin
          if (dec_neq[p] || line_neq[p]) begin
            pin_q[wp[CI-1:0]] <= PW'(p);
            kind_q[wp[CI-1:0]] <= dec_neq[p] ? 2'd0 : 2'd1;
            cyc_q[wp[CI-1:0]] <= cyc;
            ab_q[wp[CI-1:0]] <= {a_val_q[p], a_oe_q[p]};
          end
        end
      end
    end
  end
endmodule
