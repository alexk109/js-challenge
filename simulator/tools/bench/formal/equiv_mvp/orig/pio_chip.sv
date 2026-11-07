// SPDX-License-Identifier: Apache-2.0
//
// pio_chip: the baseline chip behind the register bus (docs/mvp-interfaces.md): two pio_engine
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

module pio_chip #(
    parameter int T_IN = 3,          // docs/mvp-isa.md "### Constants"; T_OUT = 1 is structural
    parameter bit IMEM_SRAM = `PIO_IMEM_SRAM_DEFAULT,
    parameter bit IMEM_INIT_ZERO = 1'b1   // simulation only, see pio_engine
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        bus_we,
    input  logic        bus_re,
    input  logic [6:0]  bus_addr,
    input  logic [31:0] bus_wdata,
    output logic [31:0] bus_rdata,
    input  logic [17:0] pin_in,
    output logic [17:0] pin_out,
    output logic [17:0] pin_oe,
    output logic        irq0,
    output logic        irq1
);

  localparam logic [6:0] A_CTRL = 7'h00, A_FSTAT = 7'h01, A_FDEBUG = 7'h02, A_FLEVEL = 7'h03;
  localparam logic [6:0] A_TXF0 = 7'h04, A_TXF1 = 7'h05, A_RXF0 = 7'h08, A_RXF1 = 7'h09;
  localparam logic [6:0] A_PADOUT = 7'h0F, A_PADOE = 7'h10, A_CFGINFO = 7'h11;
  localparam logic [6:0] A_SM0 = 7'h32, A_SM1 = 7'h38;
  localparam logic [6:0] A_INTR = 7'h4A;
  localparam logic [6:0] A_IRQ0_INTE = 7'h4B, A_IRQ0_INTF = 7'h4C, A_IRQ0_INTS = 7'h4D;
  localparam logic [6:0] A_IRQ1_INTE = 7'h4E, A_IRQ1_INTF = 7'h4F, A_IRQ1_INTS = 7'h50;

  // ---------------------------------------------------------------------------------------
  // Global registers
  // ---------------------------------------------------------------------------------------

  logic [1:0] sm_enable;        // CTRL.SM_ENABLE
  logic [7:0] fdebug;           // {TXSTALL, TXOVER, RXUNDER, RXSTALL} x 2 engines
  logic [3:0] inte0, inte1;     // IRQx_INTE bits {5:4, 1:0}
  logic [3:0] intf0, intf1;     // IRQx_INTF bits {5:4, 1:0}

  logic       wr_ctrl;
  logic [1:0] sm_restart, div_restart;

  assign wr_ctrl     = bus_we && (bus_addr == A_CTRL);
  assign sm_restart  = wr_ctrl ? bus_wdata[5:4] : 2'b00;
  assign div_restart = wr_ctrl ? bus_wdata[9:8] : 2'b00;

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

  logic [31:0] r_clkdiv[2], r_execctrl[2], r_shiftctrl[2], r_addr[2], r_instr[2], r_pinctrl[2];
  logic [31:0] rx_head[2];
  logic [2:0]  tx_level[2], rx_level[2];
  logic [1:0]  txstall_set, rxstall_set;
  logic [1:0]  txover_set, rxunder_set;
  logic [1:0]  os_we, os_dir, ss_we, ss_dir;
  logic [17:0] os_mask[2], os_data[2], ss_mask[2], ss_data[2];
  logic [1:0]  txf_wr, rxf_rd;

  for (genvar e = 0; e < 2; e++) begin : g_eng
    logic [6:0] sm_base;
    logic [6:0] imem_base;
    logic [5:0] reg_we;
    logic       imem_we;
    logic [4:0] imem_addr;

    assign sm_base   = (e == 0) ? A_SM0 : A_SM1;
    assign imem_base = (e == 0) ? 7'h12 : 7'h52;
    for (genvar i = 0; i < 6; i++) begin : g_we
      assign reg_we[i] = bus_we && (bus_addr == sm_base + 7'(i));
    end
    assign imem_we   = bus_we && (bus_addr >= imem_base) && (bus_addr <= imem_base + 7'd31);
    assign imem_addr = bus_addr[4:0] + 5'd14;  // (addr - 0x12) mod 32, same for 0x52

    assign txf_wr[e] = bus_we && (bus_addr == ((e == 0) ? A_TXF0 : A_TXF1));
    assign rxf_rd[e] = bus_re && (bus_addr == ((e == 0) ? A_RXF0 : A_RXF1));

    pio_engine #(
        .IMEM_SRAM    (IMEM_SRAM),
        .IMEM_INIT_ZERO(IMEM_INIT_ZERO)
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
        .rxf_pop    (rxf_rd[e]),
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
    assign txover_set[e]  = txf_wr[e] && (tx_level[e] == 3'd4);
    assign rxunder_set[e] = rxf_rd[e] && (rx_level[e] == 3'd0);
  end

  // ---------------------------------------------------------------------------------------
  // Pin resolution: within an engine side-set beats OUT/SET; engine 1 beats engine 0;
  // unwritten pins hold. Writes to pins without a register are dropped afterwards.
  // ---------------------------------------------------------------------------------------

  logic [17:0] lvl_n, dir_n;

  always_comb begin
    lvl_n = lvl;
    dir_n = {10'd0, dir};
    for (int e = 0; e < 2; e++) begin
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

  logic unused_res;
  assign unused_res = ^{lvl_n[12:8], dir_n[17:8]};

  assign lvl     = {lvl_uo, 5'd0, lvl_uio};
  assign pin_out = lvl;
  assign pin_oe  = {5'b11111, 5'd0, dir};

  // ---------------------------------------------------------------------------------------
  // Interrupt lines (docs/mvp-regmap.md "### tx_ready and rx_valid")
  // ---------------------------------------------------------------------------------------

  logic [3:0]  intr4;           // {TXNFULL[1:0], RXNEMPTY[1:0]}
  logic [3:0]  ints0, ints1;
  logic [31:0] intr_w, ints0_w, ints1_w;

  assign intr4   = {tx_level[1] != 3'd4, tx_level[0] != 3'd4,
                    rx_level[1] != 3'd0, rx_level[0] != 3'd0};
  assign ints0   = (intr4 & inte0) | intf0;
  assign ints1   = (intr4 & inte1) | intf1;

  function automatic logic [31:0] irq_word(input logic [3:0] b);
    irq_word = {26'd0, b[3:2], 2'd0, b[1:0]};
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

  logic [7:0] fdebug_set;
  assign fdebug_set = {txstall_set, txover_set, rxunder_set, rxstall_set};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sm_enable <= '0;
      fdebug    <= '0;
      inte0     <= 4'b0011;  // IRQ0_INTE = 0x003
      inte1     <= 4'b0100;  // IRQ1_INTE = 0x010
      intf0     <= '0;
      intf1     <= '0;
    end else begin
      if (wr_ctrl) sm_enable <= bus_wdata[1:0];
      // W1C; a flag set in the same cycle stays set
      if (bus_we && bus_addr == A_FDEBUG)
        fdebug <= (fdebug & ~{bus_wdata[25:24], bus_wdata[17:16], bus_wdata[9:8], bus_wdata[1:0]})
                  | fdebug_set;
      else fdebug <= fdebug | fdebug_set;
      if (bus_we && bus_addr == A_IRQ0_INTE) inte0 <= {bus_wdata[5:4], bus_wdata[1:0]};
      if (bus_we && bus_addr == A_IRQ0_INTF) intf0 <= {bus_wdata[5:4], bus_wdata[1:0]};
      if (bus_we && bus_addr == A_IRQ1_INTE) inte1 <= {bus_wdata[5:4], bus_wdata[1:0]};
      if (bus_we && bus_addr == A_IRQ1_INTF) intf1 <= {bus_wdata[5:4], bus_wdata[1:0]};
    end
  end

  // ---------------------------------------------------------------------------------------
  // Register reads: combinational, the state in this cycle
  // ---------------------------------------------------------------------------------------

  function automatic logic [31:0] lanes(input logic [1:0] b3, input logic [1:0] b2,
                                        input logic [1:0] b1, input logic [1:0] b0);
    lanes = {6'd0, b3, 6'd0, b2, 6'd0, b1, 6'd0, b0};
  endfunction

  logic [1:0] tx_empty2, tx_full2, rx_empty2, rx_full2;
  assign tx_empty2 = {tx_level[1] == 3'd0, tx_level[0] == 3'd0};
  assign tx_full2  = {tx_level[1] == 3'd4, tx_level[0] == 3'd4};
  assign rx_empty2 = {rx_level[1] == 3'd0, rx_level[0] == 3'd0};
  assign rx_full2  = {rx_level[1] == 3'd4, rx_level[0] == 3'd4};

  logic [31:0] rdata;

  always_comb begin
    rdata = 32'd0;
    unique case (bus_addr)
      A_CTRL:      rdata = {30'd0, sm_enable};
      A_FSTAT:     rdata = lanes(tx_empty2, tx_full2, rx_empty2, rx_full2);
      A_FDEBUG:    rdata = lanes(fdebug[7:6], fdebug[5:4], fdebug[3:2], fdebug[1:0]);
      A_FLEVEL:    rdata = {16'd0, 1'b0, rx_level[1], 1'b0, tx_level[1],
                            1'b0, rx_level[0], 1'b0, tx_level[0]};
      A_RXF0:      rdata = (rx_level[0] != 3'd0) ? rx_head[0] : 32'd0;
      A_RXF1:      rdata = (rx_level[1] != 3'd0) ? rx_head[1] : 32'd0;
      A_PADOUT:    rdata = {14'd0, pin_out};
      A_PADOE:     rdata = {14'd0, pin_oe};
      A_CFGINFO:   rdata = 32'h0020_0204;
      A_SM0 + 7'd0: rdata = r_clkdiv[0];
      A_SM0 + 7'd1: rdata = r_execctrl[0];
      A_SM0 + 7'd2: rdata = r_shiftctrl[0];
      A_SM0 + 7'd3: rdata = r_addr[0];
      A_SM0 + 7'd4: rdata = r_instr[0];
      A_SM0 + 7'd5: rdata = r_pinctrl[0];
      A_SM1 + 7'd0: rdata = r_clkdiv[1];
      A_SM1 + 7'd1: rdata = r_execctrl[1];
      A_SM1 + 7'd2: rdata = r_shiftctrl[1];
      A_SM1 + 7'd3: rdata = r_addr[1];
      A_SM1 + 7'd4: rdata = r_instr[1];
      A_SM1 + 7'd5: rdata = r_pinctrl[1];
      A_INTR:      rdata = intr_w;
      A_IRQ0_INTE: rdata = irq_word(inte0);
      A_IRQ0_INTF: rdata = irq_word(intf0);
      A_IRQ0_INTS: rdata = ints0_w;
      A_IRQ1_INTE: rdata = irq_word(inte1);
      A_IRQ1_INTF: rdata = irq_word(intf1);
      A_IRQ1_INTS: rdata = ints1_w;
      default:     rdata = 32'd0;
    endcase
  end

  // Reads only return data in a read cycle; a write cycle drives 0.
  assign bus_rdata = bus_re ? rdata : 32'd0;
endmodule
