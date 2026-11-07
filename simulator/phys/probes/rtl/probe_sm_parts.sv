// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// The per-SM blocks whose size decides N_SM / DATA_W / FIFO_DEPTH at Gate 2.

// One FIFO direction (TX or RX).
module probe_fifo #(
    parameter int DEPTH = 4,
    parameter int W     = 32
) (
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       push,
    input  logic [W-1:0]               din,
    input  logic                       pop,
    output logic [W-1:0]               dout,
    output logic                       full,
    output logic                       empty,
    output logic [$clog2(DEPTH+1)-1:0] level
);
  localparam int AW = (DEPTH > 1) ? $clog2(DEPTH) : 1;
  logic [W-1:0] mem[DEPTH];
  logic [AW-1:0] wp, rp;

  assign full  = (level == DEPTH[$clog2(DEPTH+1)-1:0]);
  assign empty = (level == '0);
  assign dout  = mem[rp];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wp <= '0;
      rp <= '0;
      level <= '0;
    end else begin
      if (push && !full) begin
        mem[wp] <= din;
        wp <= (wp == AW'(DEPTH - 1)) ? '0 : wp + 1'b1;
      end
      if (pop && !empty) rp <= (rp == AW'(DEPTH - 1)) ? '0 : rp + 1'b1;
      level <= level + (push && !full) - (pop && !empty);
    end
  end
endmodule

// OSR and ISR with variable shift amount 1..W in either direction, shift
// counters, and the IN/OUT data paths (PIO-style semantics).
module probe_shiftregs #(
    parameter int W = 32
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 osr_load,
    input  logic [W-1:0]         osr_din,
    input  logic                 out_en,
    input  logic [$clog2(W)-1:0] out_n,      // 0 encodes W
    input  logic                 out_right,
    output logic [W-1:0]         out_data,
    input  logic                 in_en,
    input  logic [$clog2(W)-1:0] in_n,
    input  logic                 in_right,
    input  logic [W-1:0]         in_data,
    input  logic                 isr_clear,
    output logic [W-1:0]         isr_q,
    output logic [$clog2(W):0]   osr_cnt,
    output logic [$clog2(W):0]   isr_cnt
);
  localparam int NW = $clog2(W) + 1;
  logic [W-1:0] osr_q;
  logic [NW-1:0] on, inn;
  logic [W-1:0] omask, imask;

  assign on    = (out_n == '0) ? NW'(W) : NW'(out_n);
  assign inn   = (in_n == '0) ? NW'(W) : NW'(in_n);
  assign omask = (on == NW'(W)) ? '1 : ((W'(1) << on) - 1'b1);
  assign imask = (inn == NW'(W)) ? '1 : ((W'(1) << inn) - 1'b1);

  assign out_data = out_right ? (osr_q & omask) : ((osr_q >> (NW'(W) - on)) & omask);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      osr_q <= '0;
      isr_q <= '0;
      osr_cnt <= '0;
      isr_cnt <= '0;
    end else begin
      if (osr_load) begin
        osr_q   <= osr_din;
        osr_cnt <= '0;
      end else if (out_en) begin
        osr_q   <= out_right ? (osr_q >> on) : (osr_q << on);
        osr_cnt <= (osr_cnt + on > NW'(W)) ? NW'(W) : osr_cnt + on;
      end
      if (isr_clear) begin
        isr_q   <= '0;
        isr_cnt <= '0;
      end else if (in_en) begin
        isr_q   <= in_right ? ((isr_q >> inn) | ((in_data & imask) << (NW'(W) - inn)))
                            : ((isr_q << inn) | (in_data & imask));
        isr_cnt <= (isr_cnt + inn > NW'(W)) ? NW'(W) : isr_cnt + inn;
      end
    end
  end
endmodule

// X/Y scratch registers with decrement, zero and X!=Y tests, plus the
// sequencer state (PC with wrap, 5-bit delay counter).
module probe_seq_xy #(
    parameter int W     = 32,
    parameter int DEPTH = 32
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     x_load,
    input  logic                     y_load,
    input  logic [W-1:0]             din,
    input  logic                     x_dec,
    input  logic                     y_dec,
    output logic                     x_zero,
    output logic                     y_zero,
    output logic                     x_ne_y,
    output logic [W-1:0]             x_q,
    output logic [W-1:0]             y_q,
    input  logic                     jmp,
    input  logic [$clog2(DEPTH)-1:0] jmp_target,
    input  logic [$clog2(DEPTH)-1:0] wrap_top,
    input  logic [$clog2(DEPTH)-1:0] wrap_bottom,
    input  logic                     advance,
    input  logic [4:0]               delay_in,
    output logic [$clog2(DEPTH)-1:0] next_pc,
    output logic                     busy
);
  logic [$clog2(DEPTH)-1:0] pc;
  logic [4:0] delay_q;

  assign x_zero = (x_q == '0);
  assign y_zero = (y_q == '0);
  assign x_ne_y = (x_q != y_q);
  assign busy = (delay_q != '0);
  assign next_pc = jmp ? jmp_target : (pc == wrap_top) ? wrap_bottom : pc + 1'b1;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      x_q <= '0;
      y_q <= '0;
      pc <= '0;
      delay_q <= '0;
    end else begin
      if (x_load) x_q <= din;
      else if (x_dec) x_q <= x_q - 1'b1;
      if (y_load) y_q <= din;
      else if (y_dec) y_q <= y_q - 1'b1;
      if (busy) delay_q <= delay_q - 1'b1;
      else if (advance) begin
        pc <= next_pc;
        delay_q <= delay_in;
      end
    end
  end
endmodule

// 16.8 fractional divider with the SM enable computed one cycle ahead and
// registered (plan sec. 6.1).
module probe_clkdiv (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [15:0] div_int,
    input  logic [7:0]  div_frac,
    output logic        en
);
  logic [15:0] cnt;
  logic [7:0] acc;
  logic carry, fire_next;
  logic [8:0] acc_sum;

  assign acc_sum = {1'b0, acc} + {1'b0, div_frac};
  assign carry = acc_sum[8];
  assign fire_next = (cnt <= 16'd1);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      cnt <= 16'd1;
      acc <= '0;
      en  <= 1'b0;
    end else begin
      en <= fire_next;
      if (fire_next) begin
        acc <= acc_sum[7:0];
        cnt <= div_int + {15'd0, carry};
      end else begin
        cnt <= cnt - 1'b1;
      end
    end
  end
endmodule

// EN_LFSR: programmable LFSR/CRC (poly, width, reflect as registers), one bit
// per enabled cycle. The plan estimates ~150 cells/SM.
module probe_lfsr #(
    parameter int W = 32
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 cfg_we,
    input  logic [W-1:0]         cfg_poly,
    input  logic [$clog2(W)-1:0] cfg_msb,
    input  logic                 cfg_reflect,
    input  logic                 load,
    input  logic [W-1:0]         load_val,
    input  logic                 step,
    input  logic                 din,
    output logic [W-1:0]         state
);
  logic [W-1:0] poly;
  logic [$clog2(W)-1:0] msb;
  logic reflect, fb;
  logic [W-1:0] mask;

  assign mask = {W{1'b1}} >> (W - 1 - msb);
  assign fb = (reflect ? state[0] : state[msb]) ^ din;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      poly <= '0;
      msb <= '0;
      reflect <= 1'b0;
      state <= '0;
    end else begin
      if (cfg_we) begin
        poly <= cfg_poly;
        msb <= cfg_msb;
        reflect <= cfg_reflect;
      end
      if (load) state <= load_val;
      else if (step)
        state <= (reflect ? (state >> 1) : (state << 1)) & mask ^ ({W{fb}} & poly);
    end
  end
endmodule
