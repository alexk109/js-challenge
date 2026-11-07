// Miter: the original MVP pio_chip (orig_pio_chip, git 45efee26) against the current pio_chip at its default
// parameters, fed the same inputs. Every output is asserted equal in every cycle. `rst_n` is a free input
// (asynchronous reset in both), so reset can fire at any time.
module miter (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        bus_we,
    input  logic        bus_re,
    input  logic [6:0]  bus_addr,
    input  logic [31:0] bus_wdata,
    input  logic [17:0] pin_in
);
  logic [31:0] rd_a, rd_b;
  logic [17:0] po_a, po_b, oe_a, oe_b;
  logic i0_a, i1_a, i0_b, i1_b;

  orig_pio_chip a (.clk(clk), .rst_n(rst_n), .bus_we(bus_we), .bus_re(bus_re), .bus_addr(bus_addr),
                   .bus_wdata(bus_wdata), .bus_rdata(rd_a), .pin_in(pin_in), .pin_out(po_a), .pin_oe(oe_a),
                   .irq0(i0_a), .irq1(i1_a));
  pio_chip      b (.clk(clk), .rst_n(rst_n), .bus_we(bus_we), .bus_re(bus_re), .bus_addr(bus_addr),
                   .bus_wdata(bus_wdata), .bus_rdata(rd_b), .pin_in(pin_in), .pin_out(po_b), .pin_oe(oe_b),
                   .irq0(i0_b), .irq1(i1_b));

  // reset is asserted in the first cycle (both designs start from their reset state), free afterwards
  logic first = 1'b1;
  always_ff @(posedge clk) first <= 1'b0;
  always_comb if (first) assume (!rst_n);

  always_comb begin
    assert (rd_a == rd_b);      // combinational read data, every address, every cycle
    assert (po_a == po_b);
    assert (oe_a == oe_b);
    assert (i0_a == i0_b);
    assert (i1_a == i1_b);
  end
endmodule
