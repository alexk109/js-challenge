/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Port-only blackbox for the IHP hard macro RM_IHPSG13_2P_64x22_c2_bm_bist
 * (64 words x 22 bits, two ports).
 *
 * The macro's real timing comes from the PDK liberty and its geometry from the LEF
 * (phys/cfg/mvp-sram.json MACROS[..].lib); this file only tells yosys the port list so the
 * macro survives synthesis as a blackbox. Same trick as the 1-port one
 * (src/RM_IHPSG13_1P_64x16_c2.v) and the same reason it is not the PDK's behavioural
 * model in .../sg13cmos5l_sram/verilog/: that one would be synthesised.
 *
 * Bus names and widths are taken verbatim from the macro LEF
 * (lef/RM_IHPSG13_2P_64x22_c2_bm_bist.lef): 264 signal pins, 22 pins each for the eight data
 * buses, 6 for the four address buses, 1 for the eighteen control pins, plus VDD!/VDDARRAY!/
 * VSS!. The power pins are not listed: LibreLane connects them with PDN_MACRO_CONNECTIONS,
 * not in the Verilog. *_BIST_* and *_BIST_EN are tied low in src/pio_imem_sram.sv
 * (datasheet: "When A_BIST_EN is disabled A_ADDR is used as address input").
 */

`default_nettype none

/* verilator lint_off UNUSED */
/* verilator lint_off UNDRIVEN */
(* blackbox *)
module RM_IHPSG13_2P_64x22_c2_bm_bist (
    // port A: the fetch read
    A_CLK,
    A_MEN,
    A_REN,
    A_WEN,
    A_DLY,
    A_ADDR,
    A_DIN,
    A_BM,
    A_DOUT,
    // port A BIST mode, unused (A_BIST_EN low)
    A_BIST_EN,
    A_BIST_CLK,
    A_BIST_MEN,
    A_BIST_REN,
    A_BIST_WEN,
    A_BIST_ADDR,
    A_BIST_DIN,
    A_BIST_BM,
    // port B: the host write
    B_CLK,
    B_MEN,
    B_REN,
    B_WEN,
    B_DLY,
    B_ADDR,
    B_DIN,
    B_BM,
    B_DOUT,
    // port B BIST mode, unused (B_BIST_EN low)
    B_BIST_EN,
    B_BIST_CLK,
    B_BIST_MEN,
    B_BIST_REN,
    B_BIST_WEN,
    B_BIST_ADDR,
    B_BIST_DIN,
    B_BIST_BM
);

  input        A_CLK;
  input        A_MEN;        // memory enable; low leaves the macro deactivated
  input        A_REN;        // read enable
  input        A_WEN;        // write enable (with A_REN high: write-through)
  input        A_DLY;        // MANDATORY: tie to 1
  input  [5:0] A_ADDR;
  input  [21:0] A_DIN;
  input  [21:0] A_BM;        // write bit mask
  output [21:0] A_DOUT;      // registered: the word read on the previous clock edge

  input        A_BIST_EN;    // 0 = normal operation, A_ADDR/A_DIN/A_BM are used
  input        A_BIST_CLK;
  input        A_BIST_MEN;
  input        A_BIST_REN;
  input        A_BIST_WEN;
  input  [5:0] A_BIST_ADDR;
  input  [21:0] A_BIST_DIN;
  input  [21:0] A_BIST_BM;

  input        B_CLK;
  input        B_MEN;
  input        B_REN;
  input        B_WEN;
  input        B_DLY;        // MANDATORY: tie to 1
  input  [5:0] B_ADDR;
  input  [21:0] B_DIN;
  input  [21:0] B_BM;
  output [21:0] B_DOUT;

  input        B_BIST_EN;
  input        B_BIST_CLK;
  input        B_BIST_MEN;
  input        B_BIST_REN;
  input        B_BIST_WEN;
  input  [5:0] B_BIST_ADDR;
  input  [21:0] B_BIST_DIN;
  input  [21:0] B_BIST_BM;

endmodule
/* verilator lint_on UNDRIVEN */
/* verilator lint_on UNUSED */

`default_nettype wire
