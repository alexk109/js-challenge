// Self-checking bench for the wide-map host port (2-byte command), bench task H.
//   iverilog -g2012 -s wide_host_tb -o /tmp/w.vvp tools/bench/sim/rtl_tb/wide_host_tb.sv \
//       src/tt_um_protocol_emulator.v src/host_spi.sv src/pio_chip.sv src/pio_clkdiv.sv \
//       src/pio_engine.sv src/pio_fifo.sv && vvp /tmp/w.vvp
// The DUT's N_ENGINES is overridden here (3 engines make the map wide); the sources stay untouched.
`timescale 1ns/1ps
module wide_host_tb;
  parameter NE = 3;
  reg clk = 0, rst_n = 0;
  reg [7:0] ui_in = 8'b0000_0001;      // CSn high
  wire [7:0] uo_out, uio_out, uio_oe;
  tt_um_protocol_emulator #(.N_ENGINES(NE), .IMEM_WORDS(32), .FIFO_DEPTH(4), .ENGINE_ROUTE(0)) dut (
      .ui_in(ui_in), .uo_out(uo_out), .uio_in(8'h00), .uio_out(uio_out), .uio_oe(uio_oe),
      .ena(1'b1), .clk(clk), .rst_n(rst_n));
  always #10 clk = ~clk;
  integer errors = 0;
  reg m;
  reg [7:0] rb;
  reg [31:0] v;

  task spi_bit(input b);
    begin
      ui_in[2] = b; repeat (4) @(posedge clk);
      m = uo_out[0];                       // MISO shows the bit before the SCK rise that shifts it out
      ui_in[1] = 1; repeat (4) @(posedge clk);
      ui_in[1] = 0; repeat (4) @(posedge clk);
    end
  endtask
  task spi_byte(input [7:0] x);
    integer i;
    begin
      for (i = 7; i >= 0; i = i - 1) begin spi_bit(x[i]); rb[i] = m; end
    end
  endtask
  task cmd16(input rw, input [9:0] addr);
    begin
      spi_byte({rw, 5'b0, addr[9:8]});
      spi_byte(addr[7:0]);
    end
  endtask
  task begin_txn;
    begin ui_in[0] = 0; repeat (6) @(posedge clk); end
  endtask
  task end_txn;
    begin repeat (8) @(posedge clk); ui_in[0] = 1; repeat (8) @(posedge clk); end
  endtask
  task wr(input [9:0] addr, input [31:0] d);
    integer i;
    begin
      begin_txn; cmd16(0, addr);
      for (i = 3; i >= 0; i = i - 1) spi_byte(d[8*i +: 8]);
      end_txn;
    end
  endtask
  task rd(input [9:0] addr);
    integer i;
    begin
      begin_txn; cmd16(1, addr);
      spi_byte(8'h00);                       // turnaround
      for (i = 3; i >= 0; i = i - 1) begin spi_byte(8'h00); v[8*i +: 8] = rb; end
      end_txn;
    end
  endtask

  initial begin
    repeat (6) @(posedge clk); rst_n = 1; repeat (10) @(posedge clk);
    rd(10'h011);
    $display("CFGINFO = %08x", v);
    if (v !== (32'd4 | (NE << 8) | (32'd32 << 16))) begin errors = errors + 1; $display("FAIL cfginfo"); end
    wr(10'h03E, 32'h1234_5600);            // SM2_CLKDIV (0x32 + 12)
    rd(10'h03E);
    $display("SM2_CLKDIV = %08x", v);
    if (v !== 32'h1234_5600) begin errors = errors + 1; $display("FAIL sm2 clkdiv"); end
    wr(10'h0B2, 32'hdeadbeef);             // bit 7 set: aliases SM0_CLKDIV (0x32) if the address were 7 bits
    rd(10'h032);
    $display("SM0_CLKDIV = %08x", v);
    if (v !== 32'h0001_0000) begin errors = errors + 1; $display("FAIL sm0 clkdiv (address aliasing)"); end
    begin_txn; cmd16(0, 10'h006);          // burst of two words to TXF2; the address holds on the FIFO window
    spi_byte(8'h00); spi_byte(8'h00); spi_byte(8'h00); spi_byte(8'h11);
    spi_byte(8'h00); spi_byte(8'h00); spi_byte(8'h00); spi_byte(8'h22);
    end_txn;
    rd(10'h003);                           // FLEVEL: engine 2 TX level = bits 19:16
    $display("FLEVEL = %08x", v);
    if (v[19:16] !== 4'd2) begin errors = errors + 1; $display("FAIL flevel"); end
    if (errors == 0) $display("WIDE HOST PASS"); else $display("WIDE HOST FAIL %0d", errors);
    $finish;
  end
  initial begin #50000000; $display("TIMEOUT"); $finish; end
endmodule
