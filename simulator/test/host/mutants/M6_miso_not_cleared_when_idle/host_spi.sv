// SPDX-License-Identifier: Apache-2.0
//
// host_spi: the SPI target that turns host transactions into register-bus accesses.
//
// Ports: docs/mvp-interfaces.md `### RTL ports`.  Behaviour: docs/mvp-regmap.md
// `## SPI transaction format`, followed literally.  The open points are in
// src/SPEC-QUESTIONS-host.md; each one is marked Q<n> below.
//
// Frame (docs/mvp-regmap.md `### Framing`):
//     write:  CMD | W0[31:0] | W1 | ...
//     read:   CMD | TA | R0[31:0] | TA | R1 | ...
//     CMD = { RW, ADDR[6:0] }, RW = 1 read / 0 write, ADDR = word address,
//     TA  = one turnaround byte, MOSI ignored.
//
// The bus access cycle `h` (docs/mvp-regmap.md `### Cycle-exact mapping`):
//     write: rise of bit 0 (the 32nd bit) of a word seen in cycle n -> write in h = n+1;
//     read:  rise of the first TA bit seen in cycle n              -> read in h = n+1,
//            bus_rdata of cycle h loaded into miso_sr at the end of h.
//
// "Cycle n" always means the cycle whose start-of-cycle state drives the logic, so an
// access scheduled in cycle n is on the bus in the next cycle, h.

module host_spi (
  input  logic        clk, rst_n,
  input  logic        spi_cs_n, spi_sck, spi_mosi,   // raw ui_in[0..2], synchronized here
  output logic        spi_miso,                       // uo_out[0]
  output logic        bus_we, bus_re,
  output logic [6:0]  bus_addr,
  output logic [31:0] bus_wdata,
  input  logic [31:0] bus_rdata
);

  // ------------------------------------------------------------------
  // Physical layer: 2-flop synchronizers, one more flop on SCK
  // ------------------------------------------------------------------
  logic cs_s1, cs_s2;           // s1 = first flop, s2 = second
  logic sck_s1, sck_s2, sck_d;  // sck_d = the previous sck_s2
  logic mosi_s1, mosi_s2;

  // Q1: the spec does not give the synchronizers' reset values.  CSn resets high
  // (deselected), so the port is idle from the first cycle out of reset; SCK resets
  // low, which is its idle level in mode 0, and MOSI resets low.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cs_s1   <= 1'b1;
      cs_s2   <= 1'b1;
      sck_s1  <= 1'b0;
      sck_s2  <= 1'b0;
      sck_d   <= 1'b0;
      mosi_s1 <= 1'b0;
      mosi_s2 <= 1'b0;
    end else begin
      cs_s1   <= spi_cs_n;
      cs_s2   <= cs_s1;
      sck_s1  <= spi_sck;
      sck_s2  <= sck_s1;
      sck_d   <= sck_s2;
      mosi_s1 <= spi_mosi;
      mosi_s2 <= mosi_s1;
    end
  end

  logic sel, rise;
  assign sel  = !cs_s2;
  assign rise = sel & sck_s2 & !sck_d;      // a rising SCK edge with the port selected

  // ------------------------------------------------------------------
  // Framing state
  // ------------------------------------------------------------------
  // Q2: TA is a phase of its own, so the 8 turnaround bits are not counted as the
  // first 8 bits of the word: MISO holds (showing bit 31) through TA and shifts only
  // in the RD phase.
  typedef enum logic [1:0] {
    PH_CMD = 2'd0,   // collecting the command byte
    PH_WR  = 2'd1,   // collecting a 32-bit data word
    PH_TA  = 2'd2,   // turnaround byte before a read word
    PH_RD  = 2'd3    // shifting a 32-bit read word out
  } phase_e;

  localparam logic [6:0] FIFO_LO = 7'h04;   // word addresses whose address stays put
  localparam logic [6:0] FIFO_HI = 7'h0b;   // in a burst: TXF0/1, reserved, RXF0/1

  phase_e     phase_q;
  logic [4:0] bit_cnt_q;     // bits seen in this byte (CMD, TA) or word (WR, RD)
  // Q6: the shift registers hold everything but the bit arriving this cycle, which is
  // `mosi_s2`; the arriving bit is combined in combinationally (`cmd_byte`, `wdata_next`)
  // so that the value for the access cycle h is complete at the end of cycle n.
  logic [6:0]  cmd_sr_q;     // command byte bits 7..1
  logic [6:0]  addr_q;       // address of the next word access
  logic [30:0] wdata_sr_q;   // write word bits 31..1
  logic [31:0] miso_sr_q;    // MISO shift register, bit 31 is the pin
  logic        we_q, re_q;   // the access pulse, valid in h
  logic [31:0] wdata_q;      // the write data of that pulse

  logic [7:0]  cmd_byte;     // the command byte including the bit arriving this cycle
  logic [31:0] wdata_next;   // the write word including the bit arriving this cycle
  assign cmd_byte   = {cmd_sr_q, mosi_s2};
  assign wdata_next = {wdata_sr_q, mosi_s2};

  // Burst address step: +1 mod 128 after every word access, except the FIFO data
  // registers 0x04-0x0b, where one write burst streams into TXFn and one read burst
  // drains RXFn.  (Q7: this runs for unmapped and reserved addresses too; the port
  // cannot know what the register file did with the access.)
  function automatic logic [6:0] addr_step(input logic [6:0] a);
    if ((a >= FIFO_LO) && (a <= FIFO_HI)) addr_step = a;
    else                                  addr_step = a + 7'h01;
  endfunction

  // While `sel` is 0 the port is idle and is reset at the end of the cycle: phase CMD,
  // bit counter 0, MISO shift register 0.  The shift registers and the address counter
  // are cleared too, so a transaction always starts from a defined state (Q4).
  // Q8: a pending access is cancelled by the idle reset, so an access is only ever put
  // on the bus in a cycle that is still selected.  A legal host holds CSn low for at
  // least 2 core cycles past the last SCK rise, so cycle h is always selected.
  // The async reset and the synchronous idle reset are separate branches: synthesis
  // frontends (slang, yosys proc_dff) reject a sync term inside the async-reset condition.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      phase_q    <= PH_CMD;
      bit_cnt_q  <= 5'd0;
      cmd_sr_q   <= 7'd0;
      addr_q     <= 7'd0;
      wdata_sr_q <= 31'd0;
      miso_sr_q  <= 32'd0;
      we_q       <= 1'b0;
      re_q       <= 1'b0;
      wdata_q    <= 32'd0;
    end else if (!sel) begin
      phase_q    <= PH_CMD;
      bit_cnt_q  <= 5'd0;
      cmd_sr_q   <= 7'd0;
      addr_q     <= 7'd0;
      wdata_sr_q <= 31'd0;
      // MUTATION M6: miso_sr_q is not reset here
      we_q       <= 1'b0;
      re_q       <= 1'b0;
      wdata_q    <= 32'd0;
    end else begin
      // The access pulse is one cycle wide: it is set at the end of the detecting cycle
      // n and so is valid in h = n+1, and cleared again at the end of h.  we_q and re_q
      // are never both 1.
      we_q <= 1'b0;
      re_q <= 1'b0;

      // After the access of cycle h the burst address moves on (Q3), and the word read in
      // h goes on MISO at the end of h.
      if (we_q || re_q) addr_q    <= addr_step(addr_q);
      if (re_q)        miso_sr_q <= bus_rdata;

      case (phase_q)
        PH_CMD: if (rise) begin
          cmd_sr_q <= {cmd_sr_q[5:0], mosi_s2};
          if (bit_cnt_q == 5'd7) begin
            // RW = cmd_byte[7]: 1 reads, 0 writes.  ADDR = cmd_byte[6:0].
            addr_q    <= cmd_byte[6:0];
            if (cmd_byte[7]) phase_q <= PH_TA;
            else             phase_q <= PH_WR;
            bit_cnt_q <= 5'd0;
          end else begin
            bit_cnt_q <= bit_cnt_q + 5'd1;
          end
        end

        // A word takes effect only after all 32 of its bits have arrived, so the access
        // is scheduled by the rise of the 32nd bit (bit_cnt_q == 31 here).
        PH_WR: if (rise) begin
          wdata_sr_q <= {wdata_sr_q[29:0], mosi_s2};
          if (bit_cnt_q == 5'd31) begin
            wdata_q   <= wdata_next;
            we_q      <= 1'b1;
            bit_cnt_q <= 5'd0;
          end else begin
            bit_cnt_q <= bit_cnt_q + 5'd1;
          end
        end

        // The read happens at the rise of the first TA bit; the rest of the byte is
        // clocked while the data settles.  MISO does not move during TA.
        PH_TA: if (rise) begin
          if (bit_cnt_q == 5'd0) re_q <= 1'b1;
          if (bit_cnt_q == 5'd7) begin
            phase_q   <= PH_RD;
            bit_cnt_q <= 5'd0;
          end else begin
            bit_cnt_q <= bit_cnt_q + 5'd1;
          end
        end

        // MISO is miso_sr_q[31]; each rise shifts left at the end of that cycle, so the
        // word goes out MSB first.  The next word of a burst gets its own TA byte.
        PH_RD: if (rise) begin
          miso_sr_q <= {miso_sr_q[30:0], 1'b0};
          if (bit_cnt_q == 5'd31) begin
            phase_q   <= PH_TA;
            bit_cnt_q <= 5'd0;
          end else begin
            bit_cnt_q <= bit_cnt_q + 5'd1;
          end
        end

        default: ;   // unreachable: every phase_e value is handled above (Q9)
      endcase
    end
  end

  // MISO comes straight from a flop, so it changes at the end of the cycle in which the
  // rise was seen.  Q5: between read words it reads 0, which is what the 32 left shifts
  // of zero leave behind.
  assign spi_miso  = miso_sr_q[31];
  assign bus_we    = we_q;
  assign bus_re    = re_q;
  assign bus_addr  = addr_q;
  assign bus_wdata = wdata_q;

endmodule
