// SPDX-License-Identifier: Apache-2.0
//
// pio_fifo: 4 x 32 FIFO, one direction (docs/mvp-isa.md "### Engine state", DS 3.2.3.5).
// Push and pop are qualified by the start-of-cycle full/empty state, so a push to a full FIFO
// and a pop of an empty one are ignored even if the other side acts in the same cycle
// (docs/mvp-isa.md "### Host-to-engine timing"). A push and a pop in the same cycle both
// happen. Async active-low reset on every flop, storage included.

module pio_fifo (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        push,
    input  logic [31:0] din,
    input  logic        pop,
    output logic [31:0] head,
    output logic [2:0]  level
);
  logic [31:0] mem[4];
  logic [1:0] wp, rp;
  logic do_push, do_pop;

  assign do_push = push && (level != 3'd4);
  assign do_pop  = pop && (level != 3'd0);
  assign head    = mem[rp];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 4; i++) mem[i] <= '0;
      wp    <= '0;
      rp    <= '0;
      level <= '0;
    end else begin
      if (do_push) begin
        mem[wp] <= din;
        wp      <= wp + 2'd1;
      end
      if (do_pop) rp <= rp + 2'd1;
      if (do_push && !do_pop) level <= level + 3'd1;
      else if (do_pop && !do_push) level <= level - 3'd1;
    end
  end
endmodule
