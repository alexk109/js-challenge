/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SRAM integration probe for TT CMOS5L: FOUR
 * RM_IHPSG13_1P_64x16_c2 side by side, one per SM, each with its own
 * write/read tester. phys/cfg/sram-probe-4.json.
 *
 * The single-macro probe is src/tt_um_sram_probe.v (phys/cfg/sram-probe.json).
 * Pin map is the same as there.
 */

`default_nettype none

// -----------------------------------------------------------------------------
// Four macros side by side, one per SM, all on the same Metal4 stripe phase.
// x = 12 + k*337.2 um (337.2 = 5 x the 67.44 um stripe pitch) is what
// phys/macro_geom.py --check-stripes verified; the last macro's right edge is
// 1260.4 um, the core xMax is 1286.4 um.
// -----------------------------------------------------------------------------
module tt_um_sram_probe4 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  RM_IHPSG13_1P_64x16_c2 u_imem0 (
      .A_CLK (clk), .A_MEN (men0), .A_WEN (wen0), .A_REN (ren0),
      .A_ADDR(addr0), .A_DIN (din0), .A_DLY (1'b1), .A_DOUT(dout0)
  );
  RM_IHPSG13_1P_64x16_c2 u_imem1 (
      .A_CLK (clk), .A_MEN (men1), .A_WEN (wen1), .A_REN (ren1),
      .A_ADDR(addr1), .A_DIN (din1), .A_DLY (1'b1), .A_DOUT(dout1)
  );
  RM_IHPSG13_1P_64x16_c2 u_imem2 (
      .A_CLK (clk), .A_MEN (men2), .A_WEN (wen2), .A_REN (ren2),
      .A_ADDR(addr2), .A_DIN (din2), .A_DLY (1'b1), .A_DOUT(dout2)
  );
  RM_IHPSG13_1P_64x16_c2 u_imem3 (
      .A_CLK (clk), .A_MEN (men3), .A_WEN (wen3), .A_REN (ren3),
      .A_ADDR(addr3), .A_DIN (din3), .A_DLY (1'b1), .A_DOUT(dout3)
  );

  wire        men0, wen0, ren0, busy0, done0, fail0;
  wire        men1, wen1, ren1, busy1, done1, fail1;
  wire        men2, wen2, ren2, busy2, done2, fail2;
  wire        men3, wen3, ren3, busy3, done3, fail3;
  wire [5:0]  addr0, addr1, addr2, addr3;
  wire [15:0] din0, din1, din2, din3, dout0, dout1, dout2, dout3;
  wire [7:0]  errs0, errs1, errs2, errs3;

  sram_tester #(.ADDR_W(6), .DATA_W(16)) u_tester0 (
      .clk(clk), .rst_n(rst_n), .restart(ui_in[0]), .busy(busy0), .done(done0),
      .failed(fail0), .errors(errs0),
      .sram_addr(addr0), .sram_din(din0), .sram_men(men0), .sram_wen(wen0),
      .sram_ren(ren0), .sram_dout(dout0)
  );
  sram_tester #(.ADDR_W(6), .DATA_W(16)) u_tester1 (
      .clk(clk), .rst_n(rst_n), .restart(ui_in[0]), .busy(busy1), .done(done1),
      .failed(fail1), .errors(errs1),
      .sram_addr(addr1), .sram_din(din1), .sram_men(men1), .sram_wen(wen1),
      .sram_ren(ren1), .sram_dout(dout1)
  );
  sram_tester #(.ADDR_W(6), .DATA_W(16)) u_tester2 (
      .clk(clk), .rst_n(rst_n), .restart(ui_in[0]), .busy(busy2), .done(done2),
      .failed(fail2), .errors(errs2),
      .sram_addr(addr2), .sram_din(din2), .sram_men(men2), .sram_wen(wen2),
      .sram_ren(ren2), .sram_dout(dout2)
  );
  sram_tester #(.ADDR_W(6), .DATA_W(16)) u_tester3 (
      .clk(clk), .rst_n(rst_n), .restart(ui_in[0]), .busy(busy3), .done(done3),
      .failed(fail3), .errors(errs3),
      .sram_addr(addr3), .sram_din(din3), .sram_men(men3), .sram_wen(wen3),
      .sram_ren(ren3), .sram_dout(dout3)
  );

  wire all_done = done0 & done1 & done2 & done3;
  wire any_fail = fail0 | fail1 | fail2 | fail3;
  wire any_errs = (errs0 != 8'd0) | (errs1 != 8'd0) | (errs2 != 8'd0) | (errs3 != 8'd0);

  assign uo_out  = {1'b0, fail3, fail2, fail1, fail0, any_errs, any_fail, all_done};
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  wire _unused = &{ena, uio_in, ui_in[7:1], busy0, busy1, busy2, busy3, 1'b0};

endmodule

`default_nettype wire

`default_nettype wire
