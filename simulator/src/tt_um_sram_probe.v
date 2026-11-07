/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SRAM integration probe for TT CMOS5L: ONE
 * RM_IHPSG13_1P_64x16_c2 with its write/read tester.
 * The four-macro variant is src/tt_um_sram_probe4.v (phys/cfg/sram-probe-4.json).
 *
 * Each macro gets its own sram_tester, which fills the macro, reads it back and
 * counts mismatches. The macro instances are named in phys/cfg/sram-probe*.json
 * under MACROS[..].instances, and the placement there is what the PDN stripe
 * grid has to line up with (see src/pdn_cfg.tcl and phys/macro_geom.py).
 *
 * Not product RTL and not a TT submission candidate: it exists to answer
 * "does a private single-port SRAM IMEM fit, power and route inside a 6x4 tile
 * block", which is the G1 decision behind IMEM_W in contracts/isa.yaml.
 *
 * Pin map (deviation from the stub, for a bench, not for the host port):
 *   ui_in[0]    restart the sweep (level)
 *   uo_out[0]   every tester finished its sweep
 *   uo_out[1]   at least one tester saw a mismatch (sticky)
 *   uo_out[2]   at least one tester counted an error (sticky)
 *   uo_out[3:6] per-macro sticky mismatch flags, macro 0 first
 */

`default_nettype none

module tt_um_sram_probe (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Instance names must match MACROS[RM_IHPSG13_1P_64x16_c2].instances in
  // phys/cfg/sram-probe.json. OpenSTA sees the flattened design, so these are
  // the names the flow looks for; no submodule wrapper on purpose.
  RM_IHPSG13_1P_64x16_c2 u_imem (
      .A_CLK (clk),
      .A_MEN (sram_men),
      .A_WEN (sram_wen),
      .A_REN (sram_ren),
      .A_ADDR(sram_addr),
      .A_DIN (sram_din),
      .A_DLY (1'b1),          // MANDATORY per the macro datasheet
      .A_DOUT(sram_dout)
  );

  wire        sram_men, sram_wen, sram_ren;
  wire [5:0]  sram_addr;
  wire [15:0] sram_din, sram_dout;
  wire        busy, done, failed;
  wire [7:0]  errors;

  sram_tester #(
      .ADDR_W(6),
      .DATA_W(16)
  ) u_tester (
      .clk      (clk),
      .rst_n    (rst_n),
      .restart  (ui_in[0]),
      .busy     (busy),
      .done     (done),
      .failed   (failed),
      .errors   (errors),
      .sram_addr(sram_addr),
      .sram_din (sram_din),
      .sram_men (sram_men),
      .sram_wen (sram_wen),
      .sram_ren (sram_ren),
      .sram_dout(sram_dout)
  );

  assign uo_out  = {4'b0000, failed, (errors != 8'd0), failed, done};
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  wire _unused = &{ena, uio_in, ui_in[7:1], busy, 1'b0};

endmodule
