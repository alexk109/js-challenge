/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flow bring-up stub: 8N1 UART transmitter that sends `data` back to back.
 * Verilog-2005 variant, used by CFG=stub. stub_uart_tx.sv is the
 * SystemVerilog twin used by CFG=stub-sv to check the slang frontend.
 */

`default_nettype none

module stub_uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    output reg        tx,
    output wire       busy
);

  localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;
  localparam integer CW = $clog2(CLKS_PER_BIT);

  reg [CW-1:0] clk_cnt;
  reg [   3:0] bits_left;  // bits still to send after the one on `tx`
  reg [   8:0] shreg;      // {stop, data[7:0]}, LSB first

  wire tick = (clk_cnt == CLKS_PER_BIT[CW-1:0] - 1'b1);

  assign busy = (bits_left != 4'd0);

  always @(posedge clk) begin
    if (!rst_n) begin
      clk_cnt   <= {CW{1'b0}};
      bits_left <= 4'd0;
      shreg     <= 9'h1ff;
      tx        <= 1'b1;
    end else if (!tick) begin
      clk_cnt <= clk_cnt + 1'b1;
    end else begin
      clk_cnt <= {CW{1'b0}};
      if (bits_left == 4'd0) begin
        tx        <= 1'b0;  // start bit
        shreg     <= {1'b1, data};
        bits_left <= 4'd9;
      end else begin
        tx        <= shreg[0];
        shreg     <= {1'b1, shreg[8:1]};
        bits_left <= bits_left - 1'b1;
      end
    end
  end

endmodule
