// SPDX-License-Identifier: Apache-2.0
//
// pio_chip: the MVP chip behind the register bus (docs/mvp-interfaces.md): two pio_engine
// instances (g_eng[e].u_eng), the global registers of docs/mvp-regmap.md, the input
// synchronizer pipeline (T_IN = 3), and the registered, resolved pin level/direction state
// (T_OUT = 1) with engine-1-wins resolution (docs/mvp-isa.md "### Pin writes").
//
// rst_n is an async active-low reset that the top level has already synchronized.

// Both engines' IMEM implementation: undefined, which is
// every default build, is the flop IMEM; `-DIMEM_SRAM` puts the same 32 words per engine in one
// two-port IHP SRAM macro each -- two macros in this chip, one per engine -- and changes
// nothing else.
`ifdef IMEM_SRAM
  `define PIO_IMEM_SRAM_DEFAULT 1'b1
`else
  `define PIO_IMEM_SRAM_DEFAULT 1'b0
`endif

// Structural parameters of the bench's parametric chip (tools/bench/sim/REGMAP.md is normative;
// task H of tools/bench/PLAN.md). The defaults are the MVP and elaborate to exactly the MVP chip.
//   N_ENGINES     1..4   engines (SMe_* registers, FIFO windows, IRQ bits for e < N_ENGINES)
//   IMEM_WORDS    16/32/64/128 instruction words per engine (PAGE field above 32)
//   FIFO_DEPTH    2..16  words per TX / RX FIFO
//   ENGINE_ROUTE  1      CTRL[16+e] links engine e's RX FIFO to engine e+1's TX FIFO
// The register bus is 7 bits wide unless the MVP map cannot hold the config (N_ENGINES > 2 or
// IMEM_WORDS > 32); then it is 10 bits and the IMEM windows move to 0x80 + 0x80*e (host_spi
// then uses a 2-byte command).
module pio_chip #(
    parameter int T_IN = 3,          // docs/mvp-isa.md "### Constants"; T_OUT = 1 is structural
    parameter bit IMEM_SRAM = `PIO_IMEM_SRAM_DEFAULT,
    parameter bit IMEM_INIT_ZERO = 1'b1,  // simulation only, see pio_engine
    parameter int N_ENGINES = 2,
    parameter int IMEM_WORDS = 32,
    parameter int FIFO_DEPTH = 4,
    parameter bit ENGINE_ROUTE = 1'b0,
    localparam bit WIDE = (N_ENGINES > 2) || (IMEM_WORDS > 32),
    localparam int AW = WIDE ? 10 : 7,             // bus_addr width
    localparam int LW = $clog2(FIFO_DEPTH + 1),    // FIFO level width
    localparam int PCW = $clog2(IMEM_WORDS)        // IMEM address width
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        bus_we,
    input  logic        bus_re,
    input  logic [AW-1:0] bus_addr,
    input  logic [31:0] bus_wdata,
    output logic [31:0] bus_rdata,
    input  logic [17:0] pin_in,
    output logic [17:0] pin_out,
    output logic [17:0] pin_oe,
    output logic        irq0,
    output logic        irq1
);


  localparam int N = N_ENGINES;
  localparam logic [AW-1:0] A_CTRL = AW'(7'h00), A_FSTAT = AW'(7'h01), A_FDEBUG = AW'(7'h02);
  localparam logic [AW-1:0] A_FLEVEL = AW'(7'h03);
  localparam logic [AW-1:0] A_TXF0 = AW'(7'h04), A_RXF0 = AW'(7'h08);
  localparam logic [AW-1:0] A_PADOUT = AW'(7'h0F), A_PADOE = AW'(7'h10), A_CFGINFO = AW'(7'h11);
  localparam logic [AW-1:0] A_SM0 = AW'(7'h32);
  localparam logic [AW-1:0] A_INTR = AW'(7'h4A);
  localparam logic [AW-1:0] A_IRQ0_INTE = AW'(7'h4B), A_IRQ0_INTF = AW'(7'h4C), A_IRQ0_INTS = AW'(7'h4D);
  localparam logic [AW-1:0] A_IRQ1_INTE = AW'(7'h4E), A_IRQ1_INTF = AW'(7'h4F), A_IRQ1_INTS = AW'(7'h50);

  if (ENGINE_ROUTE && N_ENGINES < 2) begin : g_bad_route
    $error("ENGINE_ROUTE needs at least 2 engines");
  end

  // ---------------------------------------------------------------------------------------
  // Global registers
  // ---------------------------------------------------------------------------------------

  logic [N-1:0] sm_enable;      // CTRL.SM_ENABLE
  logic [N-1:0] route_en;       // CTRL.ROUTE_EN[16+e]: link e -> e+1 (bit N-1 is constant 0)
  logic [4*N-1:0] fdebug;       // {TXSTALL, TXOVER, RXUNDER, RXSTALL}, N bits each
  logic [2*N-1:0] inte0, inte1; // IRQx_INTE bits {TXNFULL[N-1:0], RXNEMPTY[N-1:0]}
  logic [2*N-1:0] intf0, intf1; // IRQx_INTF, same packing

  logic         wr_ctrl;
  logic [N-1:0] sm_restart, div_restart;

  assign wr_ctrl     = bus_we && (bus_addr == A_CTRL);
  assign sm_restart  = wr_ctrl ? bus_wdata[4+:N] : '0;
  assign div_restart = wr_ctrl ? bus_wdata[8+:N] : '0;

  // ---------------------------------------------------------------------------------------
  // Input path: T_IN register stages per pin (2-flop synchronizer + one register).
  // Output-only pins 13..17 feed back their driven level.
  // ---------------------------------------------------------------------------------------

  logic [17:0] lvl;             // level register (bits 8..12 are constant 0)
  logic [7:0]  dir;             // direction register of the uio pins
  logic [17:0] in_raw;
  logic [17:0] in_pipe[T_IN];
  logic [31:0] in_vec;

  assign in_raw = {lvl[17:13], pin_in[12:0]};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < T_IN; i++) in_pipe[i] <= '0;
    end else begin
      in_pipe[0] <= in_raw;
      for (int i = 1; i < T_IN; i++) in_pipe[i] <= in_pipe[i-1];
    end
  end

  assign in_vec = {14'd0, in_pipe[T_IN-1]};

  logic unused_pin_in;
  assign unused_pin_in = ^pin_in[17:13];

  // ---------------------------------------------------------------------------------------
  // Engines
  // ---------------------------------------------------------------------------------------

  logic [31:0] r_clkdiv[N], r_execctrl[N], r_shiftctrl[N], r_addr[N], r_instr[N], r_pinctrl[N];
  logic [31:0] rx_head[N];
  logic [LW-1:0] tx_level[N], rx_level[N];
  logic [N-1:0] txstall_set, rxstall_set;
  logic [N-1:0] txover_set, rxunder_set;
  logic [N-1:0] os_we, os_dir, ss_we, ss_dir;
  logic [17:0] os_mask[N], os_data[N], ss_mask[N], ss_data[N];
  logic [N-1:0] txf_wr, rxf_rd;
  logic [N-1:0] host_push;      // an accepted host write to TXFe this cycle
  logic [N-1:0] link_mv;        // link e -> e+1 moves a word this cycle (bit N-1 constant 0)

  // engine_route (REGMAP.md section 3): decided on start-of-cycle levels; the host takes a TX
  // slot first, so the link needs room for the host's word as well.
  for (genvar e = 0; e < N; e++) begin : g_link
    if (e < N - 1) begin : g_has
      assign link_mv[e] = route_en[e] && (rx_level[e] != '0)
                          && (((LW + 1)'(tx_level[e+1]) + (LW + 1)'(host_push[e+1])) < (LW + 1)'(FIFO_DEPTH));
    end else begin : g_none
      assign link_mv[e] = 1'b0;
    end
  end

  for (genvar e = 0; e < N; e++) begin : g_eng
    logic [5:0]   reg_we;
    logic         imem_we;
    logic [PCW-1:0] imem_addr;
    logic         link_in;          // this engine's TX FIFO is the sink of link e-1
    logic [31:0]  link_data;

    for (genvar i = 0; i < 6; i++) begin : g_we
      assign reg_we[i] = bus_we && (bus_addr == A_SM0 + AW'(6 * e + i));
    end

    if (WIDE) begin : g_wide
      // engine e word k at 0x80 + 0x80*e + k
      assign imem_we   = bus_we && (bus_addr[AW-1:7] == 3'(e + 1)) && (IMEM_WORDS == 128 || bus_addr[6:0] < 7'(IMEM_WORDS));
      assign imem_addr = bus_addr[PCW-1:0];
    end else begin : g_narrow
      // engine e word k at 0x12 + 0x40*e + k (MVP); words >= IMEM_WORDS are unmapped
      localparam logic [AW-1:0] IMEM_BASE = AW'(7'h12 + 7'h40 * e);
      assign imem_we   = bus_we && (bus_addr >= IMEM_BASE) && (bus_addr <= IMEM_BASE + AW'(IMEM_WORDS - 1));
      assign imem_addr = PCW'(bus_addr[4:0] + 5'd14);  // (addr - 0x12) mod 32, same for 0x52
    end

    assign txf_wr[e]     = bus_we && (bus_addr == A_TXF0 + AW'(e));
    // a routed RX FIFO belongs to its link: the host cannot pop it
    assign rxf_rd[e]     = bus_re && (bus_addr == A_RXF0 + AW'(e)) && !route_en[e];
    assign host_push[e]  = txf_wr[e] && (tx_level[e] != LW'(FIFO_DEPTH));

    if (e > 0) begin : g_in
      assign link_in   = link_mv[e-1];
      assign link_data = rx_head[e-1];
    end else begin : g_noin
      assign link_in   = 1'b0;
      assign link_data = 32'd0;
    end

    pio_engine #(
        .IMEM_SRAM     (IMEM_SRAM),
        .IMEM_INIT_ZERO(IMEM_INIT_ZERO),
        .FIFO_DEPTH    (FIFO_DEPTH),
        .IMEM_WORDS    (IMEM_WORDS),
        .TX_DUAL       (ENGINE_ROUTE && (e > 0))
    ) u_eng (
        .clk        (clk),
        .rst_n      (rst_n),
        .sm_en      (sm_enable[e]),
        .sm_restart (sm_restart[e]),
        .div_restart(div_restart[e]),
        .in_vec     (in_vec),
        .wdata      (bus_wdata),
        .reg_we     (reg_we),
        .imem_we    (imem_we),
        .imem_addr  (imem_addr),
        .txf_push   (txf_wr[e]),
        .rxf_pop    (rxf_rd[e] || link_mv[e]),
        .txf_push2  (link_in),
        .txf_data2  (link_data),
        .r_clkdiv   (r_clkdiv[e]),
        .r_execctrl (r_execctrl[e]),
        .r_shiftctrl(r_shiftctrl[e]),
        .r_addr     (r_addr[e]),
        .r_instr    (r_instr[e]),
        .r_pinctrl  (r_pinctrl[e]),
        .rx_head    (rx_head[e]),
        .tx_level   (tx_level[e]),
        .rx_level   (rx_level[e]),
        .txstall_set(txstall_set[e]),
        .rxstall_set(rxstall_set[e]),
        .os_we      (os_we[e]),
        .os_dir     (os_dir[e]),
        .os_mask    (os_mask[e]),
        .os_data    (os_data[e]),
        .ss_we      (ss_we[e]),
        .ss_dir     (ss_dir[e]),
        .ss_mask    (ss_mask[e]),
        .ss_data    (ss_data[e])
    );

    // host-side FIFO flags, from start-of-cycle levels
    assign txover_set[e]  = txf_wr[e] && (tx_level[e] == LW'(FIFO_DEPTH));
    assign rxunder_set[e] = rxf_rd[e] && (rx_level[e] == '0);
  end

  // ---------------------------------------------------------------------------------------
  // Pin resolution: within an engine side-set beats OUT/SET; the higher engine beats the lower;
  // unwritten pins hold. Writes to pins without a register are dropped afterwards.
  // ---------------------------------------------------------------------------------------

  logic [17:0] lvl_n, dir_n;

  always_comb begin
    lvl_n = lvl;
    dir_n = {10'd0, dir};
    for (int e = 0; e < N; e++) begin
      if (os_we[e] && !os_dir[e]) lvl_n = (lvl_n & ~os_mask[e]) | (os_data[e] & os_mask[e]);
      if (os_we[e] && os_dir[e]) dir_n = (dir_n & ~os_mask[e]) | (os_data[e] & os_mask[e]);
      if (ss_we[e] && !ss_dir[e]) lvl_n = (lvl_n & ~ss_mask[e]) | (ss_data[e] & ss_mask[e]);
      if (ss_we[e] && ss_dir[e]) dir_n = (dir_n & ~ss_mask[e]) | (ss_data[e] & ss_mask[e]);
    end
  end

  logic [7:0] lvl_uio;
  logic [4:0] lvl_uo;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lvl_uio <= '0;
      lvl_uo  <= '0;
      dir     <= '0;
    end else begin
      lvl_uio <= lvl_n[7:0];
      lvl_uo  <= lvl_n[17:13];
      dir     <= dir_n[7:0];
    end
  end

  logic unused_hp;              // host_push[0] feeds no link
  assign unused_hp = host_push[0];

  logic unused_res;
  assign unused_res = ^{lvl_n[12:8], dir_n[17:8]};

  assign lvl     = {lvl_uo, 5'd0, lvl_uio};
  assign pin_out = lvl;
  assign pin_oe  = {5'b11111, 5'd0, dir};

  // ---------------------------------------------------------------------------------------
  // Interrupt lines (docs/mvp-regmap.md "### tx_ready and rx_valid")
  // ---------------------------------------------------------------------------------------

  logic [2*N-1:0] intr4;        // {TXNFULL[N-1:0], RXNEMPTY[N-1:0]}
  logic [2*N-1:0] ints0, ints1;
  logic [31:0]    intr_w, ints0_w, ints1_w;
  logic [N-1:0]   txnfull, rxnempty;

  for (genvar e = 0; e < N; e++) begin : g_intr
    assign txnfull[e]  = tx_level[e] != LW'(FIFO_DEPTH);
    assign rxnempty[e] = rx_level[e] != '0;
  end
  assign intr4   = {txnfull, rxnempty};
  assign ints0   = (intr4 & inte0) | intf0;
  assign ints1   = (intr4 & inte1) | intf1;

  // RXNEMPTY[e] at bit e, TXNFULL[e] at bit 4+e
  function automatic logic [31:0] irq_word(input logic [2*N-1:0] b);
    irq_word = 32'(b[N-1:0]) | (32'(b[2*N-1:N]) << 4);
  endfunction

  assign intr_w  = irq_word(intr4);
  assign ints0_w = irq_word(ints0);
  assign ints1_w = irq_word(ints1);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      irq0 <= 1'b0;
      irq1 <= 1'b0;
    end else begin
      irq0 <= |ints0;
      irq1 <= |ints1;
    end
  end

  // ---------------------------------------------------------------------------------------
  // Global register writes
  // ---------------------------------------------------------------------------------------

  logic [4*N-1:0] fdebug_set, fdebug_w1c;
  assign fdebug_set = {txstall_set, txover_set, rxunder_set, rxstall_set};
  for (genvar f = 0; f < 4; f++) begin : g_fd
    assign fdebug_w1c[N*f+:N] = bus_wdata[8*f+:N];
  end

  // IRQx_INTE resets: the MVP's 0x003 / 0x010 masked to the engines that exist, i.e. IRQ0 =
  // RXNEMPTY of engines 0 and 1, IRQ1 = TXNFULL of engine 0
  localparam logic [2*N-1:0] INTE0_RESET = (2 * N)'((1 << ((N < 2) ? N : 2)) - 1);
  localparam logic [2*N-1:0] INTE1_RESET = (2 * N)'(1 << N);

  if (ENGINE_ROUTE) begin : g_route_reg
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) route_en[N-2:0] <= '0;
      else if (wr_ctrl) route_en[N-2:0] <= bus_wdata[16+:N-1];
    end
    assign route_en[N-1] = 1'b0;
  end else begin : g_no_route_reg
    assign route_en = '0;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sm_enable <= '0;
      fdebug    <= '0;
      inte0     <= INTE0_RESET;
      inte1     <= INTE1_RESET;
      intf0     <= '0;
      intf1     <= '0;
    end else begin
      if (wr_ctrl) sm_enable <= bus_wdata[N-1:0];
      // W1C; a flag set in the same cycle stays set
      if (bus_we && bus_addr == A_FDEBUG) fdebug <= (fdebug & ~fdebug_w1c) | fdebug_set;
      else fdebug <= fdebug | fdebug_set;
      if (bus_we && bus_addr == A_IRQ0_INTE) inte0 <= {bus_wdata[4+:N], bus_wdata[N-1:0]};
      if (bus_we && bus_addr == A_IRQ0_INTF) intf0 <= {bus_wdata[4+:N], bus_wdata[N-1:0]};
      if (bus_we && bus_addr == A_IRQ1_INTE) inte1 <= {bus_wdata[4+:N], bus_wdata[N-1:0]};
      if (bus_we && bus_addr == A_IRQ1_INTF) intf1 <= {bus_wdata[4+:N], bus_wdata[N-1:0]};
    end
  end

  // ---------------------------------------------------------------------------------------
  // Register reads: combinational, the state in this cycle
  // ---------------------------------------------------------------------------------------

  // FLEVEL nibble: the level, saturating at 15 (FIFO_DEPTH = 16 reads 15 when full)
  function automatic logic [3:0] nib(input logic [LW-1:0] l);
    nib = (LW > 4 && l > LW'(15)) ? 4'hF : 4'(l);
  endfunction

  logic [31:0] fstat_w, fdebug_w, flevel_w;

  always_comb begin
    fstat_w  = 32'd0;
    fdebug_w = 32'd0;
    flevel_w = 32'd0;
    for (int e = 0; e < N; e++) begin
      fstat_w[24+e] = tx_level[e] == '0;
      fstat_w[16+e] = tx_level[e] == LW'(FIFO_DEPTH);
      fstat_w[8+e]  = rx_level[e] == '0;
      fstat_w[e]    = rx_level[e] == LW'(FIFO_DEPTH);
      for (int f = 0; f < 4; f++) fdebug_w[8*f+e] = fdebug[N*f+e];
      flevel_w[8*e+:4]   = nib(tx_level[e]);
      flevel_w[8*e+4+:4] = nib(rx_level[e]);
    end
  end

  logic [31:0] rdata;

  always_comb begin
    rdata = 32'd0;
    unique case (bus_addr)
      A_CTRL:      rdata = 32'(sm_enable) | (32'(route_en) << 16);
      A_FSTAT:     rdata = fstat_w;
      A_FDEBUG:    rdata = fdebug_w;
      A_FLEVEL:    rdata = flevel_w;
      A_PADOUT:    rdata = {14'd0, pin_out};
      A_PADOE:     rdata = {14'd0, pin_oe};
      A_CFGINFO:   rdata = 32'(FIFO_DEPTH) | (32'(N_ENGINES) << 8) | (32'(IMEM_WORDS) << 16);
      A_INTR:      rdata = intr_w;
      A_IRQ0_INTE: rdata = irq_word(inte0);
      A_IRQ0_INTF: rdata = irq_word(intf0);
      A_IRQ0_INTS: rdata = ints0_w;
      A_IRQ1_INTE: rdata = irq_word(inte1);
      A_IRQ1_INTF: rdata = irq_word(intf1);
      A_IRQ1_INTS: rdata = ints1_w;
      default: begin
        for (int e = 0; e < N; e++) begin
          if (bus_addr == A_RXF0 + AW'(e))
            rdata = (rx_level[e] != '0 && !route_en[e]) ? rx_head[e] : 32'd0;
          if (bus_addr == A_SM0 + AW'(6 * e + 0)) rdata = r_clkdiv[e];
          if (bus_addr == A_SM0 + AW'(6 * e + 1)) rdata = r_execctrl[e];
          if (bus_addr == A_SM0 + AW'(6 * e + 2)) rdata = r_shiftctrl[e];
          if (bus_addr == A_SM0 + AW'(6 * e + 3)) rdata = r_addr[e];
          if (bus_addr == A_SM0 + AW'(6 * e + 4)) rdata = r_instr[e];
          if (bus_addr == A_SM0 + AW'(6 * e + 5)) rdata = r_pinctrl[e];
        end
      end
    endcase
  end

  // Reads only return data in a read cycle; a write cycle drives 0.
  assign bus_rdata = bus_re ? rdata : 32'd0;
endmodule
