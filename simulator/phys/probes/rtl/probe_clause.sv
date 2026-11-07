// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Probe: the clause evaluator of a "contract in, verdict out" tester.
//
// A reactive tester that stores vectors needs about 10.79 Kbit; the alternative is to
// store no vectors at
// all: one row per *timing clause of the device under test*, and both the stimulus
// and the compare are derived from the clause. This probe is the silicon that
// reads such a table - NCL clause rows, the per-pin edge timestamps they compare,
// a min/max margin tracker for the clause under test, and a bisection controller
// that walks the swept parameter to the edge of the pass region in log2(range)
// bus periods instead of range.
//
// The clause table itself is expected to live in SRAM (4.3-6.0 um2/bit,
// docs/solution-space.md:51), not in flops, so it is not in this probe; the rows
// here are flop registers, so the evaluator can be measured on its own.

module probe_clause #(
    parameter int NP  = 24,  // pins watched
    parameter int NCL = 32,  // clause rows
    parameter int TW  = 16,  // tick width
    parameter int RW  = 32   // clause row width
) (
    input  logic                    clk,
    input  logic                    rst_n,
    // --- clause table write port (the loader, or a protocol program writing it)
    input  logic                    cfg_we,
    input  logic [$clog2(NCL)-1:0]  cfg_addr,
    input  logic [RW-1:0]           cfg_data,
    input  logic                    limit_we,     // write `point` into row `sel`
    input  logic [TW-1:0]           point,
    // --- the bus under test: a one-cycle pulse per pin that changed, plus its level
    input  logic [NP-1:0]           pin_ev,
    input  logic [NP-1:0]           pin_lvl,
    input  logic [TW-1:0]           tick,
    // --- campaign control
    input  logic [$clog2(NCL)-1:0]  sel,          // clause under test
    input  logic                    arm,          // start the campaign
    input  logic                    eval_en,      // evaluate now (once per bus period)
    // --- outputs
    output logic [TW-1:0]           obs,          // measured interval of the selected row
    output logic                    viol,         // selected row's verdict
    output logic [NCL-1:0]          viol_vec,     // every row's verdict, for the host
    output logic [TW-1:0]           min_obs,
    output logic [TW-1:0]           max_obs,
    output logic [TW-1:0]           lo,
    output logic [TW-1:0]           hi,
    output logic [TW-1:0]           probe_pt,
    output logic [1:0]              state,
    output logic                    done,
    output logic [7:0]              evals,        // saturating count of evaluations
    output logic [$clog2(NCL)-1:0]  viol_id,
    output logic [TW-1:0]           viol_tick
);
  localparam int PIW = (NP > 1) ? $clog2(NP) : 1;

  // row layout, MSB first: {spare[1:0], limit[TW-1:0], pin_b, pin_a, op[2:0], en}
  localparam int OP_MSB  = 3 + PIW + PIW + 1;              // bit index of op[2]
  localparam int PINA_LSB = 3 + PIW;                       // bit index of pin_a[0]
  localparam int PINB_LSB = 3;                             // bit index of pin_b[0]

  logic [RW-1:0] rows [NCL];
  integer k;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (k = 0; k < NCL; k = k + 1) rows[k] <= '0;
    end else if (limit_we) begin
      rows[sel][TW-1:0] <= point;
    end else if (cfg_we) begin
      rows[cfg_addr] <= cfg_data;
    end
  end

  // --- per-pin last-edge timestamp, with the level in bit 0
  logic [TW-1:0] ts [NP];
  integer p;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (p = 0; p < NP; p = p + 1) ts[p] <= '0;
    end else begin
      for (p = 0; p < NP; p = p + 1)
        if (pin_ev[p]) ts[p] <= {tick[TW-1:1], pin_lvl[p]};
    end
  end

  // --- every clause evaluated in parallel: the interval between two pins' last
  //     edges, compared with that row's limit under that row's operator
  logic [TW-1:0] iv [NCL];
  logic          vrow [NCL];
  logic [NCL-1:0] viol_vec_c;
  integer c;

  always_comb begin
    viol_vec_c = '0;
    for (c = 0; c < NCL; c = c + 1) begin
      iv[c] = {1'b0, ts[rows[c][OP_MSB -: PIW]][TW-1:1]} -
              {1'b0, ts[rows[c][PINA_LSB +: PIW]][TW-1:1]};
      case (rows[c][OP_MSB-2 -: 3])
        3'd0: vrow[c] = rows[c][0] & (iv[c][TW-1:0] >= rows[c][TW-1:0]);  // min width
        3'd1: vrow[c] = rows[c][0] & (iv[c][TW-1:0] <= rows[c][TW-1:0]);  // max width
        3'd2: vrow[c] = rows[c][0] & (iv[c][TW-1:0] >  rows[c][TW-1:0]);  // strictly inside
        3'd3: vrow[c] = rows[c][0] & (iv[c][TW-1:0] == rows[c][TW-1:0]);  // exact width
        3'd4: vrow[c] = rows[c][0] & pin_lvl[rows[c][PINA_LSB +: PIW]];  // level clause
        default: vrow[c] = 1'b0;
      endcase
      viol_vec_c[c] = eval_en & vrow[c];
    end
  end

  assign viol_vec = viol_vec_c;
  assign viol     = viol_vec_c[sel];
  assign obs      = iv[sel];

  // --- min/max margin trackers, for the selected clause only: one pair of
  //     registers rather than one pair per row (that is the area decision here)
  logic [TW-1:0] min_q, max_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      min_q <= {TW{1'b1}};
      max_q <= '0;
    end else if (eval_en) begin
      if (iv[sel][TW-1:0] < min_q) min_q <= iv[sel][TW-1:0];
      if (iv[sel][TW-1:0] > max_q) max_q <= iv[sel][TW-1:0];
    end
  end
  assign min_obs = min_q;
  assign max_obs = max_q;

  // --- bisection controller. Assumption, stated in the thesis: the pass/fail
  //     boundary is monotone in the swept parameter, so halving the bracket finds
  //     it in ceil(log2(range)) evaluations.
  logic [TW-1:0] lo_q, hi_q, pt_q;
  logic [1:0]    state_q;
  logic          done_q;
  logic [7:0]    evals_q;
  logic [$clog2(NCL)-1:0] viol_id_q;
  logic [TW-1:0] viol_tick_q;

  logic [TW-1:0] mid;
  assign mid = (lo_q + hi_q) >> 1;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      lo_q        <= '0;
      hi_q        <= {TW{1'b1}};
      pt_q        <= '0;
      state_q     <= 2'd0;
      done_q      <= 1'b0;
      evals_q     <= 8'd0;
      viol_id_q   <= '0;
      viol_tick_q <= '0;
    end else if (arm) begin
      lo_q    <= '0;
      hi_q    <= {TW{1'b1}};
      pt_q    <= '0;
      state_q <= 2'd1;
      done_q  <= 1'b0;
      evals_q <= 8'd0;
    end else if (eval_en && !done_q) begin
      if (evals_q != 8'hff) evals_q <= evals_q + 8'd1;
      if (viol) begin
        viol_id_q   <= sel;
        viol_tick_q <= tick;
        hi_q        <= pt_q;
      end else begin
        lo_q <= pt_q;
      end
      if ((hi_q - lo_q) <= {{(TW-1){1'b0}}, 1'b1} || mid == pt_q) begin
        done_q  <= 1'b1;
        state_q <= 2'd3;
        pt_q    <= viol ? lo_q : hi_q;
      end else begin
        pt_q <= mid;
      end
    end
  end

  assign lo      = lo_q;
  assign hi      = hi_q;
  assign probe_pt = pt_q;
  assign state   = state_q;
  assign done    = done_q;
  assign evals   = evals_q;
  assign viol_id   = viol_id_q;
  assign viol_tick = viol_tick_q;
endmodule
