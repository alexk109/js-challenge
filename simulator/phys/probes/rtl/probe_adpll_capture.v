// SPDX-License-Identifier: Apache-2.0
//
// adpll_capture + adpll_top: the block the probe is actually about, and the probe tops.
//
// adpll_capture is the interesting module. It is a *sampling* TDC with no delay line: NP pins are
// sampled on dco_clk, and the first dco cycle on which any of them differs from the previous
// sample latches a (ref_count, dco_phase) pair. ref_count counts synchronised reference periods,
// dco_phase is the free-running DCO phase inside one reference period. The pair therefore locates
// an input edge to one DCO period -- which is the log's whole claim: the resolution is the grid,
// not a delay element.
//
// Two crossings, both 2-flop, both one-bit:
//   ref_clk -> dco domain: `ref_sync2` gates ref_phase. A reference edge can arrive at any point in
//     a DCO cycle, so ref_phase jumps by whatever amount of the phase was left; the log (4b)
//     says this is the only place the reference contributes error, and it is bounded by one DCO
//     period, which is the grid we already charged.
//   dco domain -> ref_clk: the capture FIFO is read by the reference domain. A *multi-bit*
//     crossing, so it is not a plain 2-flop synchroniser: `wr_ptr`/`rd_ptr` are Gray counters,
//     which is the standard structure, and the SDC has to bound the pointer paths with
//     set_max_delay rather than false-path them (see src/adpll.sdc).
//
// Not modelled here, and each is a real risk listed in the write-up: metastability of the first
// synchroniser flop, the pad receiver's lack of hysteresis
// (research/ideation/packet-v2/07-unknowns.md:52), and the TT input path's own jitter.

`default_nettype none

/* verilator lint_off DECLFILENAME */  // three modules in one probe file, like probe_imem.sv

module adpll_capture #(
    parameter int NP    = 8,     // input pins watched
    parameter int PW    = 5,     // ref_phase width: must hold N, the multiplier
    parameter int FW    = 8      // FIFO depth, power of two
) (
    input  wire                ref_clk,
    input  wire                rst_n,
    input  wire                dco_clk,
    input  wire [NP-1:0]       pins,        // async, straight off the pads
    input  wire                rd_en,       // ref domain pops one entry
    output logic [PW:0]        rd_phase,    // dco phase of the oldest unconsumed entry
    output logic [7:0]         rd_refcnt,   // reference period of that entry
    output logic               rd_valid,
    output logic [5:0]         wr_level,    // how full, for the host
    output logic               ovf          // sticky: an entry was lost
);

  // --- ref_clk -> dco_clk: the only reference edge the fast domain sees ---------------------
  logic ref_s1, ref_s2;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) begin ref_s1 <= 1'b0; ref_s2 <= 1'b0; end
    else begin ref_s1 <= ref_clk; ref_s2 <= ref_s1; end
  end

  // free-running phase and period counter, both in the DCO domain
  logic [PW:0]  phase;
  logic [7:0]   refcnt;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) begin phase <= '0; refcnt <= '0; end
    else begin
      phase  <= phase + {{PW{1'b0}}, 1'b1};
      refcnt <= refcnt + 8'd1;
    end
  end
  wire [PW:0] phase_wrap = ref_s2 ? '0 : phase;   // restart at a reference edge

  // --- change detect ------------------------------------------------------------------------
  logic [NP-1:0] prev, cur, changed;
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) prev <= '0;
    else begin prev <= pins; cur <= pins; end
  end
  assign changed = cur ^ prev;
  wire any_change = |changed;

  // --- the FIFO: Gray-coded pointers, the standard multi-bit CDC ----------------------------
  localparam int AW = $clog2(FW);
  logic [AW:0] wr_ptr, rd_ptr;
  logic [PW:0] mem_ph [0:FW-1];
  logic [7:0]  mem_rc [0:FW-1];

  wire [AW:0] wr_gray = wr_ptr ^ (wr_ptr >> 1);
  wire [AW:0] rd_gray = rd_ptr ^ (rd_ptr >> 1);

  // pointer sync, dco -> ref and ref -> dco, two flops each
  logic [AW:0] rd_gray_s1, rd_gray_s2;                     // in the ref domain
  logic [AW:0] wr_gray_s1, wr_gray_s2;                     // in the dco domain

  // `rd_valid_full` and the two pointer increments are declared in dependency order rather than in
  // narrative order. That is not a style preference: verilator and any Verilog-2001 reader accept
  // use-before-declaration inside a module, but the flow's own slang frontend does not, and the
  // queued `make synth-probe` run of 2026-10-05 failed adpll-capture-8, adpll-capture-18 and
  // adpll-all-8 with "ERROR: Compilation failed" for exactly this reason. The same defect and the
  // same fix are in probe_adpll_loop.v.
  wire rd_valid_full = (wr_gray_s2 != rd_gray);

  wire [AW:0] wr_bin_nxt = wr_ptr + {{AW{1'b0}}, (any_change & ~rd_valid_full)};
  wire [AW:0] rd_bin_nxt = rd_ptr + {{AW{1'b0}}, (rd_en & rd_valid)};
  wire [AW:0] wr_gray_nxt = wr_bin_nxt ^ (wr_bin_nxt >> 1);

  wire full          = (wr_gray_nxt == {~rd_gray_s2[AW:AW - 1], rd_gray_s2[AW - 2:0]});

  always_ff @(posedge ref_clk or negedge rst_n) begin
    if (!rst_n) begin rd_gray_s1 <= '0; rd_gray_s2 <= '0; rd_ptr <= '0; end
    else begin rd_gray_s1 <= rd_gray; rd_gray_s2 <= rd_gray_s1; rd_ptr <= rd_bin_nxt; end
  end
  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) begin wr_gray_s1 <= '0; wr_gray_s2 <= '0; wr_ptr <= '0; end
    else begin wr_gray_s1 <= wr_gray; wr_gray_s2 <= wr_gray_s1; wr_ptr <= wr_bin_nxt; end
  end

  always_ff @(posedge dco_clk or negedge rst_n) begin
    if (!rst_n) ovf <= 1'b0;
    else if (any_change && full) ovf <= 1'b1;
  end

  // write on the cycle the change is seen; read side is combinational out of the array
  always_ff @(posedge dco_clk) begin
    if (any_change && !full) begin
      mem_ph[wr_ptr[AW - 1:0]] <= phase_wrap;
      mem_rc[wr_ptr[AW - 1:0]] <= refcnt;
    end
  end

  assign rd_valid  = rd_valid_full;
  assign rd_phase  = mem_ph[rd_ptr[AW - 1:0]];
  assign rd_refcnt = mem_rc[rd_ptr[AW - 1:0]];

  // how full, for the host to drain: pointer difference, synchronised into the ref domain
  logic [5:0] level;
  always_ff @(posedge ref_clk or negedge rst_n) begin
    if (!rst_n) level <= 6'd0;
    else level <= 6'(wr_gray_s2[AW:0]) - 6'(rd_ptr[AW:0]);
  end
  assign wr_level = level;

endmodule

// ---------------------------------------------------------------------------------------------
// adpll_top: loop + capture, no oscillator. This is the top for `make synth-probe PROBE=adpll-all`
// and it is what a first design would contain: the DCO is the only part that cannot be
// synthesised and timed, and it is priced separately by `adpll-dco`.
// ---------------------------------------------------------------------------------------------
module adpll_top #(
    parameter int NP   = 8,
    parameter int NB   = 5,
    parameter int NTAPW = 7,
    parameter int NFINE = 4,
    parameter int PW   = 5
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [NP-1:0]         uio_in,
    input  wire [NB-1:0]         uio_n_cfg,
    input  wire [1:0]            uio_gain,
    input  wire                  uio_rd_en,
    output wire [NTAPW-1:0]      uio_tap,
    output wire [NFINE-1:0]      uio_fine,
    output wire                  uio_locked,
    output wire                  uio_dco_clk,
    output wire [PW:0]           uio_rd_phase,
    output wire [7:0]            uio_rd_refcnt,
    output wire                  uio_rd_valid,
    output wire [5:0]            uio_level,
    output wire                  uio_ovf,
    output wire                  uio_meas_valid,
    output wire [NB+3:0]         uio_meas_err
);
  // In this top the "DCO clock" is a divided copy of clk: enough to elaborate, time and price the
  // digital half in the normal flow. The real oscillator is adpll_dco, probed on its own.
  reg dco_div;
  always_ff @(posedge clk or negedge rst_n) if (!rst_n) dco_div <= 1'b0; else dco_div <= ~dco_div;

  wire [NTAPW-1:0] tap;
  wire [NFINE-1:0] fine;
  wire             locked;
  wire             meas_valid;
  wire signed [NB+3:0] meas_err;

  adpll_loop #(.NB(NB), .NTAPW(NTAPW), .NFINE(NFINE)) u_loop (
      .ref_clk(clk), .rst_n(rst_n), .dco_clk(dco_div), .n_cfg(uio_n_cfg), .gain_sel(uio_gain),
      .tap(tap), .fine(fine), .locked(locked), .meas_valid(meas_valid), .meas_err(meas_err));

  adpll_capture #(.NP(NP), .PW(PW)) u_cap (
      .ref_clk(clk), .rst_n(rst_n), .dco_clk(dco_div), .pins(uio_in), .rd_en(uio_rd_en),
      .rd_phase(uio_rd_phase), .rd_refcnt(uio_rd_refcnt), .rd_valid(uio_rd_valid),
      .wr_level(uio_level), .ovf(uio_ovf));

  assign uio_tap    = tap;
  assign uio_fine   = fine;
  assign uio_locked = locked;
  assign uio_dco_clk   = dco_div;
  assign uio_meas_valid = meas_valid;
  assign uio_meas_err   = meas_err[NB+3:0];
endmodule
