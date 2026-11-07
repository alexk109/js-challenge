// SPDX-License-Identifier: Apache-2.0
//
// pio_imem_sram: one MVP engine's IMEM in an IHP sg13cmos5l SRAM macro.
//
// WHICH MACRO, AND WHY IT HAS TO HAVE TWO PORTS
// ----------------------------------------------
// The flop IMEM in pio_engine is 32 words x 16 flops, and it is read and written *in the same
// cycle*:
//
//   always_ff ir <= imem[pc_n];                                  // every cycle
//   always_ff if (imem_we) imem[imem_addr] <= wdata[15:0];      // on a host INSTR_MEMk write
//
// So this memory needs two accesses per clock, at two different addresses,
// and the read is one of them in every cycle. A single-port 1RW macro cannot do that: one
// A_ADDR, and its operation table (datasheet "### Operation Tables") offers Read, Write or
// Write-Through -- never "read A and write B". Write-Through is not a way out either: it
// returns the *new* word at the written address, where the flop build's read-first behaviour
// (`ir <= imem[pc_n]` reads the old word, nonblocking) returns the old one, and only for the
// one address the port is on.
//
// That is not a theory here. The first version of this module used two
// RM_IHPSG13_1P_64x16_c2 macros, one fetched from every cycle and one written by the host,
// and the lockstep fuzz caught it in four cycles:
//
//   fuzz: MISMATCH seed 1 cycle 4 field eng0.ir: rtl=0x0 iss=0x508
//   (research/imem-sram/1p-pair-failure/, the harness's own repro, committed as it was)
//
// because the fetch macro is the one that must receive every host write, and it is the one that
// has no cycle to receive them in. The pair would have been the cheap answer in area
// (2 x 15,240.45 = 30,480.90 um2); it cannot be made to work at any area.
//
// The PDK has ten two-port macros (datasheet "Area (h x w)"; phys/sram_pdn_sweep.py tabulates
// all of them with the stripe grid each one needs):
//
//   RM_IHPSG13_2P_64x22_c2_bm_bist   64 x 22   39,383.87 um2   526.03 x  74.87 um
//   RM_IHPSG13_2P_64x32_c2           64 x 32   52,620.88 um2   702.83 x  74.87 um
//   RM_IHPSG13_2P_256x8_c2_bm_bist  256 x  8   38,147.51 um2   278.51 x 136.97 um
//   RM_IHPSG13_2P_256x16_c2_bm_bist 256 x 16   57,520.55 um2   419.95 x 136.97 um
//   RM_IHPSG13_2P_512x16_c2_bm_bist 512 x 16   88,481.60 um2   402.61 x 219.77 um
//   ... the rest are deeper or wider and larger still
//
// This module uses RM_IHPSG13_2P_64x22_c2_bm_bist: it is the only two-port macro that is
// smaller than the 42,627 um2 of mapped flops it replaces (metrics/synth.csv imem-ff-32x16,
// which includes the 32-way read mux the macro also replaces), it is 64 words deep so the
// upper 32 are free, and 16 of its 22 data bits are exactly one instruction word wide
// (docs/mvp-isa.md "### Engine state"). 16 bits from two 8-bit macros would cost
// 76,295.02 um2 per engine, and 512 words would cost 88,481.60 um2.
//
// What the choice costs is the block's power grid, and that is not visible in this file.
// The cmos5l block PDN can only power a macro by letting the Metal4 stripes run through the
// macro's own Metal4 power columns (src/pdn_cfg.tcl), and
// there is one such grid for the whole block. The single-port RM_IHPSG13_1P_64x16_c2 keeps
// its 67.44 um FP_PDN_VPITCH, i.e. 19 stripe pairs over the 6x4 core; *every* two-port macro
// needs 175.10 um or more, i.e. 8 pairs or fewer, with 226.60 um for the 256-word and 512x16
// parts and 350.20 um for 2P_256x16/2P_256x32/2P_64x32
// (measured in a bench run, which reproduces the single-port macro's known numbers as its
// control). phys/cfg/mvp-sram.json uses this macro's 175.10 / 6.74 / 18.32. That is the price
// of the second port, and research/imem-sram/RESULT.md weighs it against the area and the
// program space the macro buys.
//
// THE TWO PORTS
// -------------
//   port A  the fetch. A_ADDR = pc_n, A_REN = 1 every cycle, A_WEN = 0 forever. A_DOUT *is*
//           the IR register: one clock of address-to-output latency, which is exactly the flop
//           line it replaces. `pc_n` is combinational (docs/mvp-isa.md "### Reference step"),
//           so the address is on the port before the edge and A_DOUT holds the word that
//           address holds at the edge.
//   port B  the host write port. B_ADDR = imem_addr, B_WEN = imem_we, B_REN = 0: this build
//           never reads through B.
//
// The two ports share one array and one clock, and the datasheet says "Both ports operate
// independly from each other, there is no conflict handling included". Because port B's write
// and port A's read of the same address land on the same edge, the read sees the word as it
// was *before* the write, which is what `ir <= imem[pc_n]` does. That is the read-first
// behaviour T_IMEM depends on, and it is why a host write to the word the engine is fetching
// takes effect from the *next* fetch, not this one.
//
// WHAT IS DIFFERENT FROM THE FLOP BUILD
// -------------------------------------
// A macro has no reset, so its power-up words are undefined where the flop array's are zero --
// and, more than that, a pin reset clears the flop array (always_ff @(posedge clk or negedge
// rst_n) in pio_engine.sv) and the ISS (iss/pio_iss.py Engine.reset) but leaves the macro's words
// standing. That is the behavioural difference, and it is inherent to any SRAM IMEM: the program
// image has to be written before CTRL.SM_ENABLE, which every firmware and the fuzz stimulus
// already do (a full 32-word image first, verif/fuzz/gen.py `_programs`). INIT_ZERO below
// gives the *simulation* the same power-up state as the flop build, so the two builds are
// comparable word for word from cycle 0; it is a time-0 poke, not a reset response, and the
// lockstep harness resets once before cycle 0, so no committed log covers a mid-run reset.
// research/imem-sram/RESULT.md has the details.

`default_nettype none

module pio_imem_sram #(
    // Words the engine addresses (its PC is AW bits; docs/mvp-isa.md "### Engine state").
    // The macro is MW words deep, so AW < MW leaves the top words unreachable -- which is what
    // the flop build does too, and what makes the extra depth free (see RESULT.md).
    parameter int AW = 5,
    // Give the array the flop build's power-up value (all zeros). Simulation only: the one
    // build that has the model's array to zero is the one that defines PIO_SRAM_MODEL, and
    // under SYNTHESIS there is no initial block at all; in silicon the words are undefined
    // until the host writes them.
    /* verilator lint_off UNUSEDPARAM */
    parameter bit INIT_ZERO = 1'b1
    /* verilator lint_on UNUSEDPARAM */
) (
    input  wire           clk,
    input  wire           rst_n,
    input  wire [AW-1:0]  rd_addr,   // pio_engine.pc_n: the word that runs next
    input  wire           wr_en,     // host INSTR_MEMk write
    input  wire [AW-1:0]  wr_addr,
    input  wire [15:0]    wr_data,
    output logic [15:0]   ir         // the instruction register: this is port A's A_DOUT
);

  // RM_IHPSG13_2P_64x22_c2_bm_bist: 64 words x 22 bits, two independent 1RW ports
  // (datasheet "### Datasheet": Size 64 words x 22 bits, Number of Ports 2).
  localparam int MW = 6;      // address bits: 64 words
  localparam int DW = 22;     // data bits; the low 16 of them are the instruction word

  localparam logic [DW-1:0] DATA_ALL_ONES = {DW{1'b1}};

  logic [DW-1:0] dout_a;
  logic [DW-1:0] dout_b;

  // A_DLY/B_DLY are timing-trim inputs the datasheet says must be tied to 1 ("IMPORTANT: Input
  // port *_DLY must always be tied to '1'"), and the PDK model checks them at t=0. The BIST
  // pins are tied low: with *_BIST_EN = 0 the macro uses the normal pins
  // (datasheet "When A_BIST_EN is disabled A_ADDR is used as address input").
  RM_IHPSG13_2P_64x22_c2_bm_bist u_imem (
      .A_CLK       (clk),
      .A_MEN       (1'b1),                     // always enabled: fetch every cycle
      .A_REN       (1'b1),
      .A_WEN       (1'b0),                     // never: every write goes to port B
      .A_DLY       (1'b1),
      .A_ADDR      ({{(MW - AW) {1'b0}}, rd_addr}),
      .A_DIN       ({DW{1'b0}}),               // a read ignores A_DIN
      .A_BM        (DATA_ALL_ONES),
      .A_DOUT      (dout_a),
      .A_BIST_EN   (1'b0),
      .A_BIST_CLK  (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_ADDR ({MW{1'b0}}),
      .A_BIST_DIN  ({DW{1'b0}}),
      .A_BIST_BM   ({DW{1'b0}}),

      .B_CLK       (clk),
      .B_MEN       (1'b1),
      .B_REN       (1'b0),                     // never reads, so it never needs write-through
      .B_WEN       (wr_en & rst_n),            // rst_n: the flop build ignores writes in reset
      .B_DLY       (1'b1),
      .B_ADDR      ({{(MW - AW) {1'b0}}, wr_addr}),
      .B_DIN       ({6'b0, wr_data}),
      .B_BM        (DATA_ALL_ONES),
      .B_DOUT      (dout_b),
      .B_BIST_EN   (1'b0),
      .B_BIST_CLK  (1'b0),
      .B_BIST_MEN  (1'b0),
      .B_BIST_REN  (1'b0),
      .B_BIST_WEN  (1'b0),
      .B_BIST_ADDR ({MW{1'b0}}),
      .B_BIST_DIN  ({DW{1'b0}}),
      .B_BIST_BM   ({DW{1'b0}})
  );

  // The six data bits above the instruction word are tied to 0 on write and never read: the
  // PDK has no 2-port macro of 64 x 16 (doc/, "Number of Ports 2" list), and the instruction
  // word is 16 bits wide (docs/mvp-isa.md "### Engine state"), so the extras are not IMEM.
  assign ir = dout_a[15:0];

  logic unused_dout;
  assign unused_dout = ^{dout_a[21:16], dout_b};

  // Power-up state, simulation only (see the header). The PDK's behavioural model keeps the
  // array in `mem_arr` and the two output registers in `dr_a_r` / `dr_b_r`, all
  // uninitialised, so without this every never-written word -- and `ir` before the first clock
  // edge -- reads X, where the flop build reads 0. `i_SRAM_2P_behavioral_bm_bist` is the
  // instance name in both branches of the PDK's RM_IHPSG13_2P_64x22_c2_bm_bist.v (with and
  // without `FUNCTIONAL`), and if the PDK ever renames it this stops compiling rather than
  // quietly going back to X.
  //
  // PIO_SRAM_MODEL is defined by the builds that compile the PDK model (verif/fuzz/Makefile,
  // fw/Makefile, test/Makefile, all with SRAM_IMEM=1); the synthesis builds compile
  // src/RM_IHPSG13_2P_64x22_c2_bm_bist.v, the port-only blackbox, which has no such scope to
  // write to -- hence the define, and `ifndef SYNTHESIS around both.
`ifdef PIO_SRAM_MODEL
`ifndef SYNTHESIS
  initial begin
    if (INIT_ZERO) begin
      for (int w = 0; w < (1 << MW); w++) begin
        u_imem.i_SRAM_2P_behavioral_bm_bist.mem_arr[w] = {DW{1'b0}};
      end
      u_imem.i_SRAM_2P_behavioral_bm_bist.dr_a_r = {DW{1'b0}};
      u_imem.i_SRAM_2P_behavioral_bm_bist.dr_b_r = {DW{1'b0}};
    end
  end
`endif
`endif

endmodule

`default_nettype wire
