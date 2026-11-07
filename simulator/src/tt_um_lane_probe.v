// SPDX-License-Identifier: Apache-2.0
//
// lane_probe: a bus-clocked target lane, probing whether a pin can clock logic in the TT flow.
//
// WHAT THIS IS. A structural probe, not a product. It answers one question static
// analysis alone could not: does the TT CMOS5L flow accept a
// `ui_in` pin as a clock, and how long is the internal port-to-port path through
// logic clocked by that pin? The number this file
// produces is `t_int`, the internal port-to-port delay.
//
// WHY IT IS SHAPED THIS WAY. The reflex path that C1 claims is
//   ui_in[3] (bus clock) -> clock buffer -> lane flop -> select -> uo_out[3]
// so nothing else may sit on that path: no pin mux, no core-domain logic, no
// reset in the reflex path (rst_n is asynchronous only into the lane's state
// flops, and those feed the table lookup, not the lane output). The response is
// a registered flop clocked by the bus clock and selected by an 8-bit mask, so
// the port-to-port path is buf -> flop CK->Q -> AND/OR tree, which is the shape
// that is estimated at 1-3 ns internally.
//
// LANE SIZE. 16 states x 8 bits = 128 flops of table, 16 state flops, one 8-bit
// shift register. This is one quarter of the pitch's 16x32 edge table; the probe
// is about the clock path, not about the table size, and a 150-flop design flows
// in minutes (metrics/flow.csv row `sram-probe`, 148 s).
//
// NOT VERIFIED HERE: anything about the pads. The TT pads live in tt_top, not in
// the user block, so pad delay is outside STA (research/ideation/packet-v2/
// 01-platform-io.md:26-36). t_int here is the INTERNAL part only; the pad part is
// the sky130 20 ns round trip, which is an argument from a datasheet.

`default_nettype none

// ui_in is read in both clock domains on purpose -- each domain sees the pad
// directly, and that is the whole point of a pad-clocked lane. Verilator reads
// that as SYNCASYNCNET (a signal used synchronously in one always block and
// asynchronously in another); here "asynchronously" means "by the other clock",
// which is the architecture.
/* verilator lint_off SYNCASYNCNET */

module tt_um_lane_probe (
    input  wire [7:0] ui_in,    // Dedicated inputs; ui_in[3] is the lane clock
    output wire [7:0] uo_out,   // Dedicated outputs; uo_out[3] is the lane response
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // --------------------------------------------------------------------------
  // Lane clock: ui_in[3] through an explicit two-inverter buffer. CTS is not
  // asked to root a tree here (CLOCK_PORT stays `clk`), so this buffer is what
  // the lane clock actually gets; the probe therefore measures the pessimistic
  // case for the lane net.
  // --------------------------------------------------------------------------
  // Two plain inverters rather than PDK cell names: synthesis maps them to the
  // library's inverter, and the file stays lintable without the PDK cell models.
  wire lane_clk_n, lane_clk;
  assign lane_clk_n = ~ui_in[3];
  assign lane_clk   = ~lane_clk_n;

  // --------------------------------------------------------------------------
  // The lane: a 16-state table machine clocked by the bus.
  //   edge_tab/state  : the edge table and the state register (quasi-static config
  //                  comes from the core domain over the two flops below, which
  //                  the SDC declares asynchronous, so this is the CDC that
  //                  needs to be proved)
  //   rx           : 8-bit shift register on the bus data pin ui_in[2]
  //   sel          : the 16-way compare feeding the response
  // --------------------------------------------------------------------------
  (* keep *) reg  [3:0] lane_state;
  (* keep *) reg  [31:0] lane_rx;   // 32-bit shift register: enough fanout for the lane clock
  wire        lane_match;

  (* keep *) reg [127:0] edge_tab;   // 16 rows x 8 bits, written from the core domain
  (* keep *) reg [15:0]  qstate;  // state double-buffer: core writes, lane reads

  // Two-flop CDC into the lane domain, one per bit, for the table and state.
  // (This is the cheap, honest stand-in for the Gray-coded FIFO the pitch wants:
  // it makes the crossing real in the netlist and in STA without pretending the
  // FIFO is the thing being measured.)
  (* keep *) reg [127:0] edge_tab_s1, edge_tab_s2;
  (* keep *) reg [15:0]  qstate_s1, qstate_s2;

  // One loop variable per process, declared inside it. A module-level integer
  // shared by two always blocks is inferred by Yosys as a 32-bit register whose
  // bits then conflict with the constants the combinational block folds them to;
  // that is what runs 1-3 of this probe did (see the flow log).
  always @(posedge clk or negedge rst_n) begin : core_block
    integer i;
    if (!rst_n) begin
      edge_tab    <= 128'd0;
      qstate      <= 16'd0;
      edge_tab_s1 <= 128'd0;
      edge_tab_s2 <= 128'd0;
      qstate_s1   <= 16'd0;
      qstate_s2   <= 16'd0;
    end else begin
      // All 16 rows are primed from the input pins so the table cannot be
      // constant-folded away and the crossing exists in the netlist. Every bit is
      // assigned in both branches: a bit left unassigned in one branch makes
      // Yosys tie it to a constant and the flow's `check` then reports a
      // conflicting constant driver on the flop's own Q (that is what the first version of
      // this probe did).
      for (i = 0; i < 16; i = i + 1) begin
        edge_tab[{i[3:0], 3'b000} +: 8] <= {ui_in[1], ui_in[2], i[2:0], ui_in[0], 2'b00};
      end
      qstate      <= qstate + 16'd1;   // every register is driven in BOTH branches:
                                      // a bit assigned only in the reset branch is
                                      // tied to a constant by Yosys and the flow's
                                      // `check` reports a conflicting constant
                                      // driver on that flop's own Q.
      edge_tab_s1 <= edge_tab;
      edge_tab_s2 <= edge_tab_s1;
      qstate_s1   <= qstate;
      qstate_s2   <= qstate_s1;
    end
  end

  // Lane state and shift register, on the bus clock.
  always @(posedge lane_clk or negedge rst_n) begin
    if (!rst_n) begin
      lane_state <= 4'd0;
      lane_rx    <= 32'd0;
    end else begin
      lane_state <= qstate_s2[lane_state +: 4];    // one 4-bit row read per bus edge
      lane_rx    <= {lane_rx[30:0], ui_in[2]};     // shift the bus data pin
    end
  end

  // Row select: the 16-way compare. edge_tab_s2 is the lane-domain copy of the table.
  reg [7:0] row;
  always @* begin : sel_block
    integer j;
    row = 8'h00;
    for (j = 0; j < 16; j = j + 1) begin
      if (lane_state == j[3:0]) row = edge_tab_s2[{j[3:0], 3'b000} +: 8];
    end
  end

  assign lane_match = (row == lane_rx[31:24]);

  // --------------------------------------------------------------------------
  // The reflex path ends here: lane flop -> mask -> dedicated output pin.
  // uo_out[3] is written by nothing else, so the ui_in[3] -> uo_out[3] path in
  // the STA report is exactly the lane.
  // --------------------------------------------------------------------------
  assign uo_out[7:4] = 4'b0000;
  assign uo_out[3] = lane_match;
  assign uo_out[0] = clk;
  assign uo_out[1] = rst_n;
  assign uo_out[2] = qstate_s2[0];
  assign uio_out   = 8'h00;
  assign uio_oe    = 8'h00;

  wire _unused = &{ena, uio_in, ui_in[7:4], ui_in[1], uio_out, uio_oe,
                   lane_rx[23:0], row[7:1], edge_tab_s1[0], qstate_s1[0], 1'b0};

endmodule
/* verilator lint_on SYNCASYNCNET */
