/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny write/read tester for one single-port IHP SRAM macro.
 *
 * Fills every word of the macro with an address-derived pattern, reads every
 * word back and counts mismatches. It is the smallest thing that proves the
 * macro is really connected in a placed-and-routed block: a macro that floats,
 * is shorted to the wrong rail, or is fed by a stripe that misses its power
 * columns shows up here as errors or as an unconnected macro.
 *
 * The macro is single port with a registered read: A_DOUT holds the word read
 * on the previous clock edge (see RM_IHPSG13_1P_64x16_c2_core_behavioral.v in
 * the PDK, `dr_r`), so the compare pipeline below is one stage deep.
 *
 * Not product RTL. It never leaves ST_DONE; drive `restart` to sweep again.
 */

`default_nettype none

module sram_tester #(
    parameter integer ADDR_W = 6,
    parameter integer DATA_W = 16
) (
    input  wire              clk,
    input  wire              rst_n,    // async, active low
    input  wire              restart,  // level: clear the sticky result and resweep

    output wire              busy,
    output reg               done,      // one-cycle pulse at the end of a sweep
    output reg               failed,    // sticky
    output reg  [7:0]        errors,    // sticky, saturating at 255

    // macro interface
    output wire [ADDR_W-1:0] sram_addr,
    output wire [DATA_W-1:0] sram_din,
    output wire              sram_men,
    output wire              sram_wen,
    output wire              sram_ren,
    input  wire [DATA_W-1:0] sram_dout
);

  // pattern(a) = {MARKER, a, ~a}: every address bit appears, and so does its
  // complement, so an aliased address, a swapped address bit or a stuck data
  // column all show up as a mismatch. Needs DATA_W >= 2*ADDR_W + 1.
  localparam integer MARK_W = DATA_W - 2 * ADDR_W;
  localparam [MARK_W-1:0] MARKER = {MARK_W{1'b1}};

  localparam [1:0] ST_FILL   = 2'd0,
                   ST_VERIFY = 2'd1,
                   ST_LAST   = 2'd2,  // the read of the last word is in flight
                   ST_DONE   = 2'd3;

  function [DATA_W-1:0] pattern;
    input [ADDR_W-1:0] a;
    begin
      pattern = {MARKER, a, ~a};
    end
  endfunction

  reg [1:0]        state;
  reg [ADDR_W-1:0] addr;
  reg [ADDR_W-1:0] addr_q;  // address of the access in flight
  reg              rd_q;    // 1 if the access in flight was a read

  assign busy     = (state != ST_DONE);
  assign sram_addr = addr;
  assign sram_din  = pattern(addr);
  // no access at all while in reset: the macro has no reset of its own, so A_MEN
  // has to hold it off or the reset state itself writes word 0.
  assign sram_men  = rst_n && ((state == ST_FILL) || (state == ST_VERIFY));
  assign sram_wen  = (state == ST_FILL);
  assign sram_ren  = (state == ST_VERIFY);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state  <= ST_FILL;
      addr   <= {ADDR_W{1'b0}};
      addr_q <= {ADDR_W{1'b0}};
      rd_q   <= 1'b0;
      done   <= 1'b0;
      failed <= 1'b0;
      errors <= 8'd0;
    end else begin
      done <= 1'b0;

      if (restart) begin
        state  <= ST_FILL;
        addr   <= {ADDR_W{1'b0}};
        rd_q   <= 1'b0;
        done   <= 1'b0;
        failed <= 1'b0;
        errors <= 8'd0;
      end else if (state == ST_FILL) begin
        // A_DOUT does not move on a write, and rd_q is low, so nothing to check.
        rd_q   <= 1'b0;
        addr_q <= addr;
        if (addr == {ADDR_W{1'b1}}) begin
          addr  <= {ADDR_W{1'b0}};
          state <= ST_VERIFY;
        end else begin
          addr <= addr + 1'b1;
        end
      end else if (state == ST_VERIFY) begin
        // A_DOUT now holds the word read one cycle ago, from address addr_q.
        rd_q   <= 1'b1;
        addr_q <= addr;
        if (rd_q && (sram_dout != pattern(addr_q))) begin
          failed <= 1'b1;
          if (errors != 8'hff) errors <= errors + 1'b1;
        end
        if (addr == {ADDR_W{1'b1}}) begin
          // one more idle cycle so the read of the last word can be compared
          addr  <= {ADDR_W{1'b0}};
          state <= ST_LAST;
        end else begin
          addr <= addr + 1'b1;
        end
      end else if (state == ST_LAST) begin
        // compare the read of the last word, which landed during ST_LAST
        rd_q   <= 1'b0;
        addr_q <= addr;
        if (rd_q && (sram_dout != pattern(addr_q))) begin
          failed <= 1'b1;
          if (errors != 8'hff) errors <= errors + 1'b1;
        end
        state <= ST_DONE;
        done  <= 1'b1;
      end
      // ST_DONE: hold the result until `restart`
    end
  end

endmodule

`default_nettype wire
