// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// The data-DPLL block of research/ideation/round2/research/DPLL.md.
//
// One block per PIO engine. It sees only what the silicon could see: the synchronised pin value
// (the same `sync[-1]` the engine's IN_VEC comes from, `T_IN = 3`) and the engine's own divider
// registers. It writes the engine's existing 16.8 CLKDIV; it adds no register to the map and needs
// no ISA change.
//
// Structure, from the report's Mechanism section:
//   * a frame-boundary detector: a falling edge at least `frame_bits - 1/2` bit periods after the
//     last accepted one is a start bit; anything sooner is a data transition and is dropped;
//   * an interval register plus a running minimum over `WIN` intervals, which rejects an
//     inter-frame gap and a missed frame and averages the +-1 tick arrival quantisation;
//   * a 16.8 divider write port, `int_q` and `frac_q`, from `interval * 2**FRAC_BITS / (frame_bits
//     * sm_cycles_per_bit)` -- a shift and an add, not a multiplier;
//   * optionally the coarse rate ladder a real block needs for the fast side: `LADDER` candidate
//     bit periods, one step down every `SEARCH_PERIOD` bit periods without a boundary. Each rung
//     is a multiply by 1/1.5 and 1/1.2247, which synthesise to shift-adds.
//
// No sub-cycle phase adjust is modelled: the actuator in the report for the tracking half is the
// engine's enable pulse, which is `probe_dpll`'s `nudge_adv` / `nudge_del` outputs, and they are
// one flop each.
module probe_dpll #(
    parameter int CW        = 32,   // core-cycle counter width
    parameter int FRAME_BITS = 10,  // bits per frame (8N1)
    parameter int WIN       = 8,    // intervals kept for the running minimum
    parameter int LADDER    = 0,    // 0 = no coarse search; >0 = number of ladder rungs
    parameter int FRAC_BITS = 8,   // CLKDIV.FRAC width, 16.8 divider
    // SM_CYCLES = 0 keeps `sm_cycles` a runtime input (a host-programmable S, which costs a
    // combinational divider); SM_CYCLES > 0 makes it a constant and the divide folds to a shift.
    parameter int SM_CYCLES = 0
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 pin,          // synchronised pin value
    input  logic                 clkdiv_restart,
    input  logic [31:0]          clkdiv,       // current 16.8 setting (INT 31:16, FRAC 15:8)
      input  logic [CW-1:0]        sm_cycles,   // SM cycles per bit time (S), host-programmed
    input  logic                 late,        // phase detector: the last boundary arrived late
    output logic [31:0]          clkdiv_o,    // the new 16.8 setting, or `clkdiv` unchanged
    output logic                 locked,      // an interval has been measured and accepted
    output logic                 nudge_adv,   // tracking actuator: take an extra SM cycle
    output logic                 nudge_del    // tracking actuator: skip an SM cycle
);
  // ---- how many core cycles one bit is expected to last, from the current divider ----
  // CLKDIV is 16.8: one SM cycle lasts INT + FRAC/256 core cycles, and a bit is `sm_cycles` of
  // them, so t_bit = sm_cycles * (INT + FRAC/256).
  localparam int EST_W  = CW + 8;
  localparam int RUNG_W = (LADDER < 2) ? 1 : $clog2(LADDER + 1);

  logic [15:0]      div_int;
  logic [7:0]       div_frac;
  logic [EST_W-1:0] sm_period;
  logic [EST_W-1:0] t_bit;

  assign div_int  = clkdiv[31:16];
  assign div_frac = clkdiv[15:8];
  // one SM cycle lasts INT + FRAC/256 core cycles (8 fractional bits), and a bit is `sm_cycles`
  // of them, so t_bit = sm_cycles * (INT + FRAC/256).
  always_comb begin
    sm_period = (EST_W'(div_int) << 8) + EST_W'(div_frac);
    t_bit     = (sm_period * sm_cycles) >> 8;
  end

  // ---- the coarse ladder: t_bit / 1.5^(rung/2) / 1.2247^(rung%2) ----
  logic [RUNG_W-1:0] rung;          // which candidate is in force
  // One step is a shift-add; written as a case so the probe measures a real table, not a multiply.
  logic [EST_W-1:0] cand;
  logic [EST_W-1:0] half, three_eighths;
  always_comb begin
    half          = (t_bit >> 1) + (t_bit >> 3);   // x 0.6250
    three_eighths = (t_bit >> 2) + (t_bit >> 4);   // x 0.3125
    case (rung)
      // Each rung divides the candidate bit period by about 1.225 (half of 1.5^2), so the ladder
      // walks down by a factor of ~45 over 12 rungs and every rung is a shift and an add.
      default: cand = t_bit;
      1:  cand = half - (half >> 4);
      2:  cand = half + (t_bit >> 5);
      3:  cand = half + (t_bit >> 5) - ((half + (t_bit >> 5)) >> 3);
      4:  cand = three_eighths;
      5:  cand = three_eighths - (three_eighths >> 3);
      6:  cand = (t_bit >> 3) + (t_bit >> 5);
      7:  cand = (t_bit >> 3) + (t_bit >> 5) - ((t_bit >> 5));
      8:  cand = (t_bit >> 4) + (t_bit >> 6);
      9:  cand = (t_bit >> 4) + (t_bit >> 7);
      10: cand = (t_bit >> 5) + (t_bit >> 6);
      11: cand = (t_bit >> 5) + (t_bit >> 7);
    endcase
  end

  // ---- the guard: half a bit of slack either side of the frame length ----
  // A start bit is FRAME_BITS periods after the last one; a data transition to 0 is at most
  // FRAME_BITS - 2 periods after it (the stop bit is high), so (FRAME_BITS - 1/2) periods
  // separates them with half a bit of margin on each side.
  logic [EST_W-1:0] guard_bits;
  // `boundary` and `take_min` are the frame-boundary accept strobe and its write enable; `hist`
  // is the last `WIN` accepted intervals, from which the running minimum is taken.
  logic          boundary, take_min;
  logic [WIN-1:0][CW-1:0] hist;
  always_comb guard_bits = (cand * EST_W'(2 * FRAME_BITS - 1)) >> 1;

  // The minimum of the window is a chain of comparators -- that is what the probe is measuring.
  logic [CW-1:0] best;
  always_comb begin
    best = hist[0];
    for (int i = 1; i < WIN; i++) if (hist[i] < best) best = hist[i];
  end

  // ---- the 16.8 write: interval / (frame_bits * sm_cycles), in 8 fractional bits ----
  // shift by FRAC_BITS, then divide by the frame length and by S. Both divisors are host-set
  // constants, so this is a shift and two small divisions rather than a divider.
  logic [CW+FRAC_BITS:0] scaled;
  logic [CW+FRAC_BITS:0] per_bit;
  always_comb begin
    scaled  = ({{(FRAC_BITS + 1) {1'b0}}, best} << FRAC_BITS);
    per_bit = (FRAME_BITS == 0) ? '0 : (scaled / EST_W'(FRAME_BITS));
  end

  logic [CW+FRAC_BITS:0] per_sm;
  logic [CW-1:0]         s_eff;
  always_comb begin
    s_eff  = (SM_CYCLES == 0) ? ((sm_cycles == 0) ? 1 : sm_cycles) : CW'(SM_CYCLES);
    per_sm = per_bit / s_eff;
  end

  // Only an accepted measurement differs from the input, so the mux is one gate wide per field.
  wire [31:0] div_next = take_min ? {16'd0, per_sm[FRAC_BITS + 15:FRAC_BITS],
                                     per_sm[FRAC_BITS-1:0]}
                                  : clkdiv;

  // ---- edge detection on the synchronised pin ----
  logic pin_q;
  always_ff @(posedge clk) pin_q <= pin;
  wire falling = pin_q & ~pin;

  // ---- the interval counter ----
  logic [CW-1:0] t_start;
  logic [CW-1:0] cycle_q;
  always_ff @(posedge clk) cycle_q <= cycle_q + 1'b1;

  wire timed_out = (cycle_q - t_start) > guard_bits;

  // ---- the running minimum over WIN intervals ----
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      boundary <= 1'b0;
      take_min <= 1'b0;
      t_start  <= '0;
      locked   <= 1'b0;
      rung     <= '0;
      cycle_q  <= '0;
      for (int i = 0; i < WIN; i++) hist[i] <= {CW{1'b1}};
      clkdiv_o <= clkdiv;
    end else begin
      clkdiv_o <= div_next;
      boundary <= 1'b0;
      take_min <= 1'b0;
      locked   <= 1'b0;
      if (clkdiv_restart) begin
        t_start <= cycle_q;
      end
      // accept a boundary: first edge after reset, or one past the guard
      if (falling && ((t_start == '0) || ((cycle_q - t_start) >= guard_bits))) begin
        boundary <= 1'b1;
        t_start  <= cycle_q;
        for (int i = 0; i < WIN - 1; i++) hist[i] <= hist[i + 1];
        hist[WIN - 1] <= cycle_q - t_start;
        take_min     <= 1'b1;
      end
      // the coarse search: no boundary for a whole frame period means the candidate is too slow
      if (LADDER > 0) begin
        if (timed_out) begin
          if (rung < LADDER - 1) rung <= rung + 1'b1;
        end
      end
    end
  end


  // ---- the tracking actuator: one flop each, one core cycle of authority ----
  // A bang-bang phase detector with a fast-then-slow vote threshold (S3, S4 in the sources). The
  // "signed" input is the sign of the phase error, i.e. whether the last boundary arrived before
  // or after the prediction; the vote is a saturating up/down counter.
  logic [3:0]  votes;
  logic [3:0]  threshold;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      votes <= '0;
      threshold <= 4'd1;
      nudge_adv <= 1'b0;
      nudge_del <= 1'b0;
    end else begin
      nudge_adv <= 1'b0;
      nudge_del <= 1'b0;
      if (votes >= threshold) begin
        if (late) nudge_del <= 1'b1; else nudge_adv <= 1'b1;   // peer late -> we sampled early
        votes <= '0;
        if (threshold != 4'd7) threshold <= threshold + 1'b1;  // fast, then slow
      end else if (votes != 0) begin
        if (late) votes <= votes - 1'b1; else votes <= votes + 1'b1;
      end
    end
  end
endmodule
