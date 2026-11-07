// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Private per-SM instruction memory options, each including the fetch-stage
// instruction register (plan sec. 6.1) so the read path is comparable.

// Flops: host write port, registered read addressed by the next-PC mux.
module probe_imem_ff #(
    parameter int DEPTH = 32,
    parameter int W     = 24
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     we,
    input  logic [$clog2(DEPTH)-1:0] waddr,
    input  logic [W-1:0]             wdata,
    input  logic [$clog2(DEPTH)-1:0] next_pc,
    input  logic                     hold,
    output logic [W-1:0]             ir
);
  logic [W-1:0] mem[DEPTH];

  always_ff @(posedge clk) if (we) mem[waddr] <= wdata;

  always_ff @(posedge clk)
    if (!rst_n) ir <= '0;
    else if (!hold) ir <= mem[next_pc];
endmodule

// Latches + one ICG per word (the tinyQV / DFFRAM style). Write data is
// registered; row r's latches open while clk is low in the cycle after the
// write, so they are stable again before the next rising edge.
module probe_imem_latch #(
    parameter int DEPTH = 32,
    parameter int W     = 24
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     we,
    input  logic [$clog2(DEPTH)-1:0] waddr,
    input  logic [W-1:0]             wdata,
    input  logic [$clog2(DEPTH)-1:0] next_pc,
    input  logic                     hold,
    output logic [W-1:0]             ir
);
  logic [W-1:0] wdata_q;
  logic [$clog2(DEPTH)-1:0] waddr_q;
  logic we_q;
  logic [W-1:0] mem[DEPTH];

  always_ff @(posedge clk) begin
    wdata_q <= wdata;
    waddr_q <= waddr;
    we_q    <= rst_n & we;
  end

  for (genvar r = 0; r < DEPTH; r++) begin : g_row
    logic gclk;
    sg13cmos5l_lgcp_1 u_icg (
        .CLK (~clk),
        .GATE(we_q && waddr_q == r),
        .GCLK(gclk)
    );
    always_latch if (gclk) mem[r] = wdata_q;
  end

  always_ff @(posedge clk)
    if (!rst_n) ir <= '0;
    else if (!hold) ir <= mem[next_pc];
endmodule

// Shared (RP2040 PIO style): one array, NPORT independent registered read
// ports (one per SM). Reads never conflict, so this keeps the one-fetch-per-
// cycle timing model; programs share the DEPTH slots.
module probe_imem_shared #(
    parameter int DEPTH = 64,
    parameter int W     = 24,
    parameter int NPORT = 4,
    parameter bit LATCH = 1'b0
) (
    input  logic                                clk,
    input  logic                                rst_n,
    input  logic                                we,
    input  logic [$clog2(DEPTH)-1:0]            waddr,
    input  logic [W-1:0]                        wdata,
    input  logic [NPORT-1:0][$clog2(DEPTH)-1:0] next_pc,
    input  logic [NPORT-1:0]                    hold,
    output logic [NPORT-1:0][W-1:0]             ir
);
  logic [W-1:0] mem[DEPTH];

  if (LATCH) begin : g_latch
    logic [W-1:0] wdata_q;
    logic [$clog2(DEPTH)-1:0] waddr_q;
    logic we_q;
    always_ff @(posedge clk) begin
      wdata_q <= wdata;
      waddr_q <= waddr;
      we_q    <= rst_n & we;
    end
    for (genvar r = 0; r < DEPTH; r++) begin : g_row
      logic gclk;
      sg13cmos5l_lgcp_1 u_icg (
          .CLK (~clk),
          .GATE(we_q && waddr_q == r),
          .GCLK(gclk)
      );
      always_latch if (gclk) mem[r] = wdata_q;
    end
  end else begin : g_ff
    always_ff @(posedge clk) if (we) mem[waddr] <= wdata;
  end

  for (genvar p = 0; p < NPORT; p++) begin : g_port
    always_ff @(posedge clk)
      if (!rst_n) ir[p] <= '0;
      else if (!hold[p]) ir[p] <= mem[next_pc[p]];
  end
endmodule

// Cross-coupled A21OI storage (after Yann Guidon's HDSISO8RS): per bit
//   q  = !((we_row & dn) | qn)      qn = !((we_row & d) | q)
// is a gated D latch in 2 x 9.07 um2, with d/dn driven once per column.
// Row write pulses come from registered write data + a clock-low window, as
// in probe_imem_latch. STA needs the q/qn loop disabled and a pulse check;
// this probe only measures area.
module probe_imem_srlatch #(
    parameter int DEPTH = 32,
    parameter int W     = 24
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     we,
    input  logic [$clog2(DEPTH)-1:0] waddr,
    input  logic [W-1:0]             wdata,
    input  logic [$clog2(DEPTH)-1:0] next_pc,
    input  logic                     hold,
    output logic [W-1:0]             ir
);
  logic [W-1:0] d_q, dn;
  logic [$clog2(DEPTH)-1:0] waddr_q;
  logic we_q;
  logic [W-1:0] q[DEPTH];

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      d_q <= '0;
      waddr_q <= '0;
      we_q <= 1'b0;
    end else begin
      d_q <= wdata;
      waddr_q <= waddr;
      we_q <= we;
    end
  assign dn = ~d_q;

  for (genvar r = 0; r < DEPTH; r++) begin : g_row
    logic wl;
    assign wl = we_q & ~clk & (waddr_q == r);
    for (genvar b = 0; b < W; b++) begin : g_bit
      logic qn;
      (* keep *) sg13cmos5l_a21oi_1 u_q  (.A1(wl), .A2(dn[b]),  .B1(qn),      .Y(q[r][b]));
      (* keep *) sg13cmos5l_a21oi_1 u_qn (.A1(wl), .A2(d_q[b]), .B1(q[r][b]), .Y(qn));
    end
  end

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) ir <= '0;
    else if (!hold) ir <= q[next_pc];
endmodule
