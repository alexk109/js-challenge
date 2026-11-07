// SPDX-License-Identifier: Apache-2.0
//
// SystemVerilog twin of stub_uart_tx.v (CFG=stub-sv). Same ports and
// the same 10-bit 8N1 frame on `tx`, plus one idle bit between frames.

module stub_uart_tx
  import stub_pkg::*;
#(
    parameter int unsigned CLK_HZ = 50_000_000,
    parameter int unsigned BAUD   = 115_200
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic [7:0] data,
    output logic       tx,
    output logic       busy
);

  localparam int unsigned CPB = clks_per_bit(CLK_HZ, BAUD);
  localparam int unsigned CW = $clog2(CPB);

  tx_state_e state_q;
  logic [CW-1:0] cnt_q;
  logic [2:0] bit_q;
  logic [7:0] sh_q;
  logic tick;

  assign tick = (cnt_q == CW'(CPB - 1));
  assign busy = (state_q != TX_IDLE);

  always_ff @(posedge clk) begin
    if (!rst_n) cnt_q <= '0;
    else cnt_q <= tick ? '0 : cnt_q + 1'b1;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state_q <= TX_IDLE;
      bit_q   <= '0;
      sh_q    <= '0;
      tx      <= 1'b1;
    end else if (tick) begin
      unique case (state_q)
        TX_IDLE: begin
          state_q <= TX_START;
          sh_q    <= data;
          tx      <= 1'b0;
        end
        TX_START: begin
          state_q <= TX_DATA;
          bit_q   <= '0;
          tx      <= sh_q[0];
          sh_q    <= sh_q >> 1;
        end
        TX_DATA: begin
          bit_q <= bit_q + 1'b1;
          if (bit_q == 3'd7) begin
            state_q <= TX_STOP;
            tx      <= 1'b1;
          end else begin
            tx   <= sh_q[0];
            sh_q <= sh_q >> 1;
          end
        end
        TX_STOP: begin
          state_q <= TX_IDLE;
          tx      <= 1'b1;
        end
      endcase
    end
  end

endmodule
