/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * tt_um_protocol_emulator: the Tiny Tapeout top level of the baseline chip.
 *
 * It is two modules and the wiring between them, exactly as docs/mvp-interfaces.md
 * `### RTL ports` says:
 *
 *   host_spi   the SPI target on ui_in[2:0] / uo_out[0], which turns host
 *              transactions into register-bus accesses (docs/mvp-regmap.md
 *              `## SPI transaction format`)
 *   pio_chip   the two engines, the FIFOs, the register file and the 18 protocol
 *              pins (docs/mvp-regmap.md, docs/mvp-isa.md)
 *
 * Pin use (docs/mvp-regmap.md `### Physical layer` and docs/mvp-interfaces.md
 * `### Protocol pins`), all 43 Tiny Tapeout signals accounted for:
 *
 *   ui_in[2:0]   CSn, SCK, MOSI of the host port, raw: host_spi synchronizes them
 *   uo_out[0]    MISO
 *   uo_out[1]    irq1 = tx_ready   (PIO interrupt line IRQ1)
 *   uo_out[2]    irq0 = rx_valid   (PIO interrupt line IRQ0)
 *   uo_out[7:3]  protocol pins 17..13, the five output-only pins
 *   ui_in[7:3]   protocol pins 12..8, the five input-only pins
 *   uio[7:0]     protocol pins 7..0, the eight bidirectional pins
 *
 * `pin_in[17:13]` is left at 0 on purpose: the chip ignores those bits and feeds back
 * its own driven level (docs/mvp-interfaces.md `### Protocol pins`), so driving them
 * from a pin that does not exist would only add a fanout cone.
 *
 * rst_n comes straight off a pad, so it is asserted asynchronously and released
 * synchronously: `rst_n_int` is the two-flop reset synchronizer below, and it is the
 * only thing both modules see. Releasing it synchronously is what keeps the engines,
 * the FIFOs and the SPI port out of reset on different cycles.
 */

`default_nettype none

module tt_um_protocol_emulator (
    input  wire [7:0] ui_in,    // Dedicated inputs: [2:0] CSn/SCK/MOSI, [7:3] protocol pins 8..12
    output wire [7:0] uo_out,   // Dedicated outputs: [0] MISO, [1] tx_ready, [2] rx_valid, [7:3] pins 13..17
    input  wire [7:0] uio_in,   // IOs: Input path -- protocol pins 0..7
    output wire [7:0] uio_out,  // IOs: Output path -- protocol pins 0..7
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ---------------------------------------------------------------------------------
  // Reset: async assert, sync release. The first flop catches the asynchronous
  // assertion, the second one is what the rest of the chip sees, so rst_n_int rises on
  // a clock edge and both modules leave reset together.
  // ---------------------------------------------------------------------------------
  reg rst_meta, rst_int;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_meta <= 1'b0;
      rst_int  <= 1'b0;
    end else begin
      rst_meta <= 1'b1;
      rst_int  <= rst_meta;
    end
  end

  // ---------------------------------------------------------------------------------
  // The register bus between the two modules, and the three signals the two of them hand
  // to the pads: MISO, tx_ready (irq1) and rx_valid (irq0).
  // ---------------------------------------------------------------------------------
  wire        bus_we, bus_re;
  wire [6:0]  bus_addr;
  wire [31:0] bus_wdata;
  wire [31:0] bus_rdata;
  wire        spi_miso;
  wire        irq0, irq1;      // irq0 = rx_valid, irq1 = tx_ready

  // ---------------------------------------------------------------------------------
  // The 18 protocol pins, as docs/mvp-interfaces.md `### Protocol pins` numbers them:
  // 0..7 are the uio pins, 8..12 the five input-only ones on ui_in[7:3], 13..17 the
  // five output-only ones on uo_out[7:3].
  // ---------------------------------------------------------------------------------
  wire [17:0] pin_in, pin_out, pin_oe;

  // The eight uio pads are bidirectional, so the input path has to see the resolved
  // level of each pin: a pin the chip drives reads back what it drives, a pin it
  // releases reads what the outside world drives.  That is what the pad itself does --
  // `sg13cmos5l_IOPadInOut` says `assign pad = c2p_en ? c2p : 1'bz; assign p2c = pad;`
  // -- and without it open drain cannot work (docs/mvp-isa.md `### Open drain`: the
  // level register stays 0 and only the direction changes, so `in pins` has to see the
  // chip's own pull-down).  docs/mvp-interfaces.md `### Protocol pins` calls `uio_in`
  // the input of pins 0..7, which on silicon it is: the pad's input.  Doing the same
  // resolution here as well is idempotent (with `uio_oe` high the pad resolves to
  // `uio_out` whatever feeds its input, with `uio_oe` low this resolves to the outside
  // world), so the chip behaves the same with pads, without them, and in simulation.
  wire [7:0] uio_in_resolved;

  assign uio_in_resolved = (uio_oe & uio_out) | (~uio_oe & uio_in);
  assign pin_in          = {5'b00000, ui_in[7:3], uio_in_resolved};
  assign uio_out         = pin_out[7:0];
  assign uio_oe          = pin_oe[7:0];
  assign uo_out          = {pin_out[17:13], irq0, irq1, spi_miso};

  // ---------------------------------------------------------------------------------
  // The host port
  // ---------------------------------------------------------------------------------
  host_spi u_host (
      .clk      (clk),
      .rst_n    (rst_int),
      .spi_cs_n (ui_in[0]),
      .spi_sck  (ui_in[1]),
      .spi_mosi (ui_in[2]),
      .spi_miso (spi_miso),
      .bus_we   (bus_we),
      .bus_re   (bus_re),
      .bus_addr (bus_addr),
      .bus_wdata(bus_wdata),
      .bus_rdata(bus_rdata)
  );

  // ---------------------------------------------------------------------------------
  // The chip
  // ---------------------------------------------------------------------------------
  pio_chip u_chip (
      .clk      (clk),
      .rst_n    (rst_int),
      .bus_we   (bus_we),
      .bus_re   (bus_re),
      .bus_addr (bus_addr),
      .bus_wdata(bus_wdata),
      .bus_rdata(bus_rdata),
      .pin_in   (pin_in),
      .pin_out  (pin_out),
      .pin_oe   (pin_oe),
      .irq0     (irq0),
      .irq1     (irq1)
  );

  // `ena` is held at 1 whenever the chip is selected. `pin_out[12:8]` is the level
  // register of the five input-only pins, which reads 0 and goes nowhere, and
  // `pin_oe[17:8]` is 0 for the input-only pins and 1 for the output-only ones, so
  // only its low eight bits (the uio enables) are a signal here.
  wire _unused = &{ena, pin_out[12:8], pin_oe[17:8], 1'b0};

endmodule
