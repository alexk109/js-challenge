// SPDX-License-Identifier: Apache-2.0
//
// Testbench wrapper for the host SPI port (test/host).
//
// `host_spi` is not a Tiny Tapeout top level, but the cocotb bridge of
// verif/vip/cocotb_bridge.py maps its nets onto TT pins, so this wrapper gives it the
// TT port list and puts `host_spi` behind it the way tt_um_protocol_emulator.sv does
// for the chip: ui_in[0..2] are CSn/SCK/MOSI (docs/mvp-regmap.md `### Physical layer`),
// uo_out[0] is MISO.
//
// The register bus is brought out as plain top-level ports so the Python register-file
// model can drive bus_rdata and watch the accesses (docs/mvp-interfaces.md
// `## Register bus`): the "chip" side of the bench is Python, not RTL.
//
// Deliberately no `default_nettype`: the file under test is somebody else's module and
// its own compilation environment is its own business. Everything here is typed.
//
// Waves: VCD (not FST), the format icarus here is built with can write.

module tb (
    input  wire [7:0] ui_in,      // [0] CSn, [1] SCK, [2] MOSI
    output wire [7:0] uo_out,     // [0] MISO
    input  wire [7:0] uio_in,     // unused by this bench
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n,

    // the register bus, to the Python register-file model
    output wire       bus_we,
    output wire       bus_re,
    output wire [6:0] bus_addr,
    output wire [31:0] bus_wdata,
    input  wire [31:0] bus_rdata
);

  initial begin
    $dumpfile("host_spi.vcd");
    $dumpvars(0, tb);
  end

  wire miso;

  host_spi u_host_spi (
      .clk      (clk),
      .rst_n    (rst_n),
      .spi_cs_n (ui_in[0]),
      .spi_sck  (ui_in[1]),
      .spi_mosi (ui_in[2]),
      .spi_miso (miso),
      .bus_we   (bus_we),
      .bus_re   (bus_re),
      .bus_addr (bus_addr),
      .bus_wdata(bus_wdata),
      .bus_rdata(bus_rdata)
  );

  assign uo_out  = {7'd0, miso};
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // uio_* and ena belong to the chip top level, not to this module.
  wire _unused = &{ena, uio_in, uio_out, uio_oe, 1'b0};

endmodule
