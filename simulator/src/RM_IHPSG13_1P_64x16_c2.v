/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Port-only blackbox for the IHP hard macro RM_IHPSG13_1P_64x16_c2
 * (64 words x 16 bits, single port).
 *
 * The macro's real timing and cell information comes from the PDK liberty in
 * phys/cfg/sram-probe.json (MACROS[..].lib) and its geometry from the LEF; this
 * file only has to tell yosys the port list so the macro survives synthesis as
 * a blackbox. Same trick as the reference recipe
 * (research/ref/loom-sram-smoke/), and deliberately NOT the PDK's behavioural
 * model in .../sg13cmos5l_sram/verilog/, which would be synthesised.
 *
 * Port order and names are taken verbatim from the macro LEF:
 *   lef/RM_IHPSG13_1P_64x16_c2.lef, 43 signal pins + VDD!/VDDARRAY!/VSS!.
 * The power pins are not listed: LibreLane connects them with
 * PDN_MACRO_CONNECTIONS, not in the Verilog.
 */

`default_nettype none

/* verilator lint_off UNUSED */
/* verilator lint_off UNDRIVEN */
(* blackbox *)
module RM_IHPSG13_1P_64x16_c2 (
    A_CLK,
    A_MEN,
    A_WEN,
    A_REN,
    A_ADDR,
    A_DIN,
    A_DLY,
    A_DOUT
);

  input        A_CLK;
  input        A_MEN;   // memory enable; low leaves the macro deactivated
  input        A_WEN;   // write enable (with A_REN high: write-through)
  input        A_REN;   // read enable
  input  [5:0] A_ADDR;
  input  [15:0] A_DIN;
  input        A_DLY;   // MANDATORY: tie to 1
  output [15:0] A_DOUT; // registered: the word read on the previous clock edge

endmodule
/* verilator lint_on UNDRIVEN */
/* verilator lint_on UNUSED */

`default_nettype wire
