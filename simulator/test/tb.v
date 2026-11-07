`default_nettype none
`timescale 1ns / 1ps

/* This testbench instantiates the module and makes the convenient wires that the cocotb
   test.py drives and observes.  It sees nothing of the chip: test.py speaks to the design
   only through the Tiny Tapeout top-level ports (ui_in, uo_out, uio_*), because the same
   test has to run on the hardened gate-level netlist through TT's gl_test.

   `uio_in` is what the outside world puts on each bidirectional pin; the design resolves
   it against its own drive (see `src/tt_um_protocol_emulator.v`), which is what the pad
   cell does on silicon, so nothing is modelled here.
*/
module tb ();

  // Dump the signals to a VCD file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  tt_um_protocol_emulator user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

endmodule
