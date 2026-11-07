// SPDX-License-Identifier: Apache-2.0
//
// ---------------------------------------------------------------------------------
// STUB -- NOT the RTL. Not silicon. Not part of the chip.
//
// This is a behavioural stand-in for `src/host_spi.sv`, written from
// `docs/mvp-regmap.md` `## SPI transaction format` so that `test/host` (the cocotb
// bench) can be debugged without the real module. It is written independently from the
// same spec text; this file never reads the real module and must not be mistaken for it.
//
// `test/host/Makefile` compiles `../../src/host_spi.sv` whenever that file is present
// and only falls back to this stub when it is not. Delete this file once the real
// module lands.
//
// It is deliberately the most literal thing the spec allows: no pipelining, no
// lookahead, one state machine that does exactly what the "Cycle-exact mapping to bus
// accesses" section says. That is the point -- the bench checks the spec, and a stub
// that "improves" on it would hide spec violations.
// ---------------------------------------------------------------------------------

module host_spi (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         spi_cs_n,   // raw ui_in[0], synchronized here
    input  wire         spi_sck,    // raw ui_in[1]
    input  wire         spi_mosi,   // raw ui_in[2],
    output wire         spi_miso,   // uo_out[0]
    output wire         bus_we,
    output wire         bus_re,
    output wire [6:0]   bus_addr,
    output wire [31:0]  bus_wdata,
    input  wire  [31:0]  bus_rdata
);

  // ---------------------------------------------------------------- synchronizers
  // "CSn, SCK and MOSI each pass a 2-flop synchronizer (s1, s2), and SCK has one more
  // flop (sck_d = the previous s2)."
  reg cs_s1, cs_s2;
  reg sck_s1, sck_s2;
  reg mosi_s1, mosi_s2;
  reg sck_d;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cs_s1 <= 1'b1;   cs_s2 <= 1'b1;   // idle: CS_N high
      sck_s1 <= 1'b0;  sck_s2 <= 1'b0;
      mosi_s1 <= 1'b0; mosi_s2 <= 1'b0;
      sck_d <= 1'b0;
    end else begin
      cs_s1 <= spi_cs_n;  cs_s2 <= cs_s1;
      sck_s1 <= spi_sck;  sck_s2 <= sck_s1;
      mosi_s1 <= spi_mosi; mosi_s2 <= mosi_s1;
      sck_d <= sck_s2;
    end
  end

  wire sel = ~cs_s2;                     // "sel = !cs_s2"
  wire rise = sel & sck_s2 & ~sck_d;     // "rise = sel & sck_s2 & !sck_d"

  // ---------------------------------------------------------------------- framing
  localparam [1:0] P_CMD = 2'd0,  // the command byte
                   P_WR  = 2'd1,  // a write data word, 32 bits
                   P_TA  = 2'd2,  // the turnaround byte of a read
                   P_RD  = 2'd3;  // the 32 bits of a read word coming out

  reg [1:0]  phase;
  reg [2:0]  cmd_cnt;    // CMD bits received, 0..7
  reg [2:0]  ta_cnt;     // TA bits received, 0..7
  reg [4:0]  word_cnt;   // data bits received of the current word, 0..31
  reg [7:0]  cmd_sr;
  reg [31:0] wr_sr;      // the word coming in, MSB first
  reg [31:0] miso_sr;    // MISO comes straight from bit 31 of this
  reg [6:0]  addr_q;     // word address of the next word access

  // The command byte as of this rising edge: the bits already shifted in plus the one
  // arriving now, in the natural order, so `CMD = { RW, ADDR[6:0] }` reads off it
  // directly. The rising edge that completes the byte has to be decoded with the
  // arriving bit included, so the byte is formed here and not one cycle later.
  wire [7:0] cmd_byte = {cmd_sr[6:0], mosi_s2};

  // A request made in cycle n drives the bus in cycle h = n + 1. The register below is
  // what makes that true: it is clocked by the `rise` in cycle n, so the bus carries it
  // in cycle n + 1 and not one cycle later.
  reg        wr_req, rd_req;
  reg [6:0]  req_addr_q;
  reg [31:0] req_wdata_q;

  assign spi_miso = miso_sr[31];
  assign bus_we   = wr_req;
  assign bus_re   = rd_req;
  assign bus_addr = req_addr_q;
  assign bus_wdata= req_wdata_q;

  // "After each word access, the address increments by 1, wrapping mod 128. The
  //  exception is the FIFO data registers (word addresses 0x04-0x0B), where the address
  //  stays put."
  function [6:0] next_addr(input [6:0] a);
    begin
      if (a >= 7'h04 && a <= 7'h0b) next_addr = a;
      else                         next_addr = a + 7'd1;   // 7 bits wrap mod 128
    end
  endfunction

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      phase      <= P_CMD;
      cmd_cnt    <= 3'd0;
      ta_cnt     <= 3'd0;
      word_cnt   <= 5'd0;
      cmd_sr     <= 8'd0;
      wr_sr      <= 32'd0;
      miso_sr    <= 32'd0;
      addr_q     <= 7'd0;
      wr_req     <= 1'b0;
      rd_req     <= 1'b0;
      req_addr_q <= 7'd0;
      req_wdata_q<= 32'd0;
    end else begin
      wr_req     <= 1'b0;
      rd_req     <= 1'b0;

      // ---- MISO ----
      // "In each RD-phase cycle with a rise, miso_sr shifts left by one at the end of
      //  that cycle." and "the value is loaded into the MISO shift register at the end
      //  of h" (h = the cycle bus_re is asserted in, i.e. the cycle rd_req is valid).
      if (sel && rise && phase == P_RD)
        miso_sr <= {miso_sr[30:0], 1'b0};
      if (rd_req)
        miso_sr <= bus_rdata;

      // ---- the frame ----
      if (!sel) begin
        // "While sel = 0, the port is idle and is reset at the end of the cycle: phase
        //  CMD, bit counter 0, MISO shift register 0."
        phase    <= P_CMD;
        cmd_cnt  <= 3'd0;
        ta_cnt   <= 3'd0;
        word_cnt <= 5'd0;
        cmd_sr   <= 8'd0;
        wr_sr    <= 32'd0;
        miso_sr  <= 32'd0;
        addr_q   <= 7'd0;
      end else if (rise) begin
        case (phase)
          P_CMD: begin
            cmd_sr <= cmd_byte;
            if (cmd_cnt == 3'd7) begin
              addr_q   <= cmd_byte[6:0];        // ADDR
              phase    <= cmd_byte[7] ? P_TA : P_WR;   // RW = 1 read, 0 write
              word_cnt <= 5'd0;
              ta_cnt   <= 3'd0;
            end else begin
              cmd_cnt <= cmd_cnt + 3'd1;
            end
          end
          P_WR: begin
            wr_sr <= {wr_sr[30:0], mosi_s2};
            if (word_cnt == 5'd31) begin
              // "if rise for bit 0 (the 32nd bit) of a data word is detected in cycle
              //  n, the write is performed in cycle h = n + 1"
              wr_req      <= 1'b1;
              req_addr_q  <= addr_q;
              req_wdata_q <= {wr_sr[30:0], mosi_s2};
              addr_q      <= next_addr(addr_q);
              word_cnt    <= 5'd0;
            end else begin
              word_cnt <= word_cnt + 5'd1;
            end
          end
          P_TA: begin
            // "Each word is preceded by its own turnaround byte. The read itself, and
            //  its side effects ... happen at the first bit of the TA byte", so the
            //  request goes out on the first of the eight bits and the 32 data bits
            //  only start after the eighth.
            if (ta_cnt == 3'd0) begin
              // "if rise for the first TA bit is detected in cycle n, the read is
              //  performed in cycle h = n + 1"
              rd_req      <= 1'b1;
              req_addr_q  <= addr_q;
              req_wdata_q <= 32'd0;
              addr_q      <= next_addr(addr_q);
            end
            if (ta_cnt == 3'd7) begin
              phase    <= P_RD;
              word_cnt <= 5'd0;
              ta_cnt   <= 3'd0;
            end else begin
              ta_cnt <= ta_cnt + 3'd1;
            end
          end
          P_RD: begin
            // 32 data bits shift out, then the next word of the burst wants its own
            // turnaround byte, and keeps wanting one until CS_N rises.
            if (word_cnt == 5'd31) begin
              phase    <= P_TA;
              word_cnt <= 5'd0;
              ta_cnt   <= 3'd0;
            end else begin
              word_cnt <= word_cnt + 5'd1;
            end
          end
          default: phase <= P_CMD;
        endcase
      end
    end
  end

endmodule
