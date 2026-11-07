// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Plan sec. 7 probe 1: pin mapping for N_SM machines onto NP protocol pins.
// Both variants include their configuration registers (host-written) and
// the registered pad value/OE, so the comparison is like for like.

// Windowed: each SM writes OUT/SET/SIDE values into base+count windows (a
// mod-NP rotate, PIO style) and reads pins rotated by IN base. Within an SM
// side-set beats OUT/SET; across SMs the highest index wins.
module probe_pinmux_win #(
    parameter int N_SM = 4,
    parameter int NP   = 20,
    parameter int DW   = 32
) (
    input  logic                        clk,
    input  logic                        rst_n,
    // PINCTRL writes from the host
    input  logic                        cfg_we,
    input  logic [$clog2(N_SM)-1:0]     cfg_sm,
    input  logic [31:0]                 cfg_data,
    // per-SM requests this cycle
    input  logic [N_SM-1:0]             out_we,
    input  logic [N_SM-1:0]             out_to_dir,
    input  logic [N_SM-1:0][DW-1:0]     out_data,
    input  logic [N_SM-1:0]             set_we,
    input  logic [N_SM-1:0]             set_to_dir,
    input  logic [N_SM-1:0][4:0]        set_data,
    input  logic [N_SM-1:0]             side_we,
    input  logic [N_SM-1:0]             side_to_dir,
    input  logic [N_SM-1:0][4:0]        side_data,
    input  logic [NP-1:0]               pins_in,
    output logic [N_SM-1:0][NP-1:0]     in_view,
    output logic [NP-1:0]               pad_out,
    output logic [NP-1:0]               pad_oe
);
  typedef struct packed {
    logic [4:0] in_base;
    logic [2:0] side_count;
    logic [4:0] side_base;
    logic [2:0] set_count;
    logic [4:0] set_base;
    logic [5:0] out_count;
    logic [4:0] out_base;
  } pinctrl_t;

  pinctrl_t pinctrl[N_SM];

  // Rotate an NP-bit vector left by `sh` (0..NP-1), mod NP.
  function automatic logic [NP-1:0] rotl(logic [NP-1:0] v, logic [4:0] sh);
    logic [2*NP-1:0] d;
    d = {v, v} << sh;
    return d[2*NP-1:NP];
  endfunction

  function automatic logic [NP-1:0] rotr(logic [NP-1:0] v, logic [4:0] sh);
    logic [2*NP-1:0] d;
    d = {v, v} >> sh;
    return d[NP-1:0];
  endfunction

  function automatic logic [NP-1:0] lowmask(logic [5:0] n);
    return (n >= NP) ? '1 : NP'((32'd1 << n) - 1);
  endfunction

  logic [NP-1:0] nxt_out, nxt_oe;

  always_comb begin
    nxt_out = pad_out;
    nxt_oe  = pad_oe;
    for (int s = 0; s < N_SM; s++) begin
      logic [NP-1:0] m, v;
      // OUT then SET, then SIDE on top (side-set has priority within an SM)
      m = rotl(lowmask(pinctrl[s].out_count), pinctrl[s].out_base);
      v = rotl(out_data[s][NP-1:0], pinctrl[s].out_base);
      if (out_we[s] && !out_to_dir[s]) nxt_out = (nxt_out & ~m) | (v & m);
      if (out_we[s] && out_to_dir[s]) nxt_oe = (nxt_oe & ~m) | (v & m);
      m = rotl(lowmask({3'b0, pinctrl[s].set_count}), pinctrl[s].set_base);
      v = rotl({{(NP - 5) {1'b0}}, set_data[s]}, pinctrl[s].set_base);
      if (set_we[s] && !set_to_dir[s]) nxt_out = (nxt_out & ~m) | (v & m);
      if (set_we[s] && set_to_dir[s]) nxt_oe = (nxt_oe & ~m) | (v & m);
      m = rotl(lowmask({3'b0, pinctrl[s].side_count}), pinctrl[s].side_base);
      v = rotl({{(NP - 5) {1'b0}}, side_data[s]}, pinctrl[s].side_base);
      if (side_we[s] && !side_to_dir[s]) nxt_out = (nxt_out & ~m) | (v & m);
      if (side_we[s] && side_to_dir[s]) nxt_oe = (nxt_oe & ~m) | (v & m);
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      pad_out <= '0;
      pad_oe  <= '0;
      for (int s = 0; s < N_SM; s++) pinctrl[s] <= '0;
    end else begin
      pad_out <= nxt_out;
      pad_oe  <= nxt_oe;
      if (cfg_we) pinctrl[cfg_sm] <= pinctrl_t'(cfg_data);
    end
  end

  for (genvar s = 0; s < N_SM; s++) begin : g_in
    assign in_view[s] = rotr(pins_in, pinctrl[s].in_base);
  end
endmodule

// Crossbar: each SM keeps its own DW-bit output and OE vectors (no windows);
// every pad picks (owner SM, bit index) and every SM input bit picks any pad.
module probe_pinmux_xbar #(
    parameter int N_SM = 4,
    parameter int NP   = 20,
    parameter int DW   = 32
) (
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic                        cfg_we,
    input  logic [7:0]                  cfg_addr,
    input  logic [7:0]                  cfg_data,
    input  logic [N_SM-1:0]             out_we,
    input  logic [N_SM-1:0]             out_to_dir,
    input  logic [N_SM-1:0][DW-1:0]     out_data,
    input  logic [N_SM-1:0][DW-1:0]     out_mask,
    input  logic [NP-1:0]               pins_in,
    output logic [N_SM-1:0][NP-1:0]     in_view,
    output logic [NP-1:0]               pad_out,
    output logic [NP-1:0]               pad_oe
);
  localparam int IW = $clog2(DW);
  localparam int SW = (N_SM > 1) ? $clog2(N_SM) : 1;

  logic [N_SM-1:0][DW-1:0] loc_out, loc_oe;
  logic [SW-1:0] pad_owner[NP];
  logic [IW-1:0] pad_bit[NP];
  logic [4:0] in_sel[N_SM][NP];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      loc_out <= '0;
      loc_oe  <= '0;
      for (int p = 0; p < NP; p++) begin
        pad_owner[p] <= '0;
        pad_bit[p]   <= '0;
      end
      for (int s = 0; s < N_SM; s++) for (int i = 0; i < NP; i++) in_sel[s][i] <= '0;
    end else begin
      for (int s = 0; s < N_SM; s++) begin
        if (out_we[s] && !out_to_dir[s])
          loc_out[s] <= (loc_out[s] & ~out_mask[s]) | (out_data[s] & out_mask[s]);
        if (out_we[s] && out_to_dir[s])
          loc_oe[s] <= (loc_oe[s] & ~out_mask[s]) | (out_data[s] & out_mask[s]);
      end
      if (cfg_we) begin
        // addr 0..NP-1: pad map; NP.. : input selects (one byte each)
        if (cfg_addr < NP) begin
          pad_owner[cfg_addr] <= cfg_data[IW+SW-1:IW];
          pad_bit[cfg_addr]   <= cfg_data[IW-1:0];
        end else begin
          in_sel[(cfg_addr-NP)/NP][(cfg_addr-NP)%NP] <= cfg_data[4:0];
        end
      end
    end
  end

  for (genvar p = 0; p < NP; p++) begin : g_pad
    assign pad_out[p] = loc_out[pad_owner[p]][pad_bit[p]];
    assign pad_oe[p]  = loc_oe[pad_owner[p]][pad_bit[p]];
  end

  for (genvar s = 0; s < N_SM; s++) begin : g_in
    for (genvar i = 0; i < NP; i++) begin : g_bit
      assign in_view[s][i] = pins_in[in_sel[s][i]];
    end
  end
endmodule
