// SPDX-License-Identifier: Apache-2.0
//
// AREA PROBE RTL - representative, not verified, not product RTL.
// Probe: area probes for two safety datapaths:
//   probe_pad_safety : per-pin safety governor (lease + legality + fail-silent)
//   probe_pin_shadow : per-pin decision comparator for a shadow ("second opinion") engine
// Both are generic: no block is named after a protocol, every per-pin parameter is
// host-loaded, and every mechanism has a firmware fallback (release / ignore).
// Method and results: research/ideation/round2/README-safety.md.

// ---------------------------------------------------------------------------
// probe_pad_safety
//
// One governor per pad. It sits between the engine's output request and the pad
// registers and is the only thing that can drive `pad_oe`.
//
// Per pin, host-loaded contract (probe purpose, not product spec):
//   legal_hi  driving a hard 1 on this pin is legal
//   legal_lo  driving a hard 0 on this pin is legal
//   od        open-drain pin: only "drive 0" / "release" are legal, so legal_hi = 0
// Mechanisms, in the order they can fire:
//   1. LEASE  a counter set whenever this pin becomes driven; if the engine does
//             not re-arm it within LW ticks the governor releases the pin, so a
//             hung program cannot hold a bus line forever (liveness).
//   2. LEGAL  a drive request outside {legal_hi, legal_lo} is refused: the pad
//             keeps its previous safe state, the violation is sticky, the pad
//             releases (fail-silent), and the violation id + timestamp are logged.
//   3. CONTENTION the sampled line level disagrees with the value we are driving,
//             which on a shared bus means another device is driving it too.
//             Sticky, does NOT release by itself (a bus fight is not our bug to
//             resolve), but it is logged.
//   4. SAFE STATE out of reset, and after any release: oe = 0.
// Log: 4-entry ring of {pin id, kind, timestamp}. One shared free-running
// timestamp counter, TW bits.
// ---------------------------------------------------------------------------
module probe_pad_safety #(
    parameter int NP = 18,   // protocol pins governed (the MVP map's 18)
    parameter int LW = 12,   // lease width in ticks
    parameter int TW = 20,   // timestamp width in ticks
    parameter int LOGN = 4   // violation-log entries
) (
    input  logic            clk,
    input  logic            rst_n,
    // host configuration, one word per pin
    input  logic            cfg_we,
    input  logic [4:0]      cfg_pin,
    input  logic [2:0]      cfg_data,   // {legal_hi, legal_lo, od}
    input  logic            ts_clear,
    // engine requests (what the pin mux would have driven)
    input  logic [NP-1:0]   req_oe,
    input  logic [NP-1:0]   req_val,
    input  logic [NP-1:0]   req_rearm,  // engine says "I still own this pin"
    // observed line levels, post-synchronizer
    input  logic [NP-1:0]   line_level,
    // pad registers this module owns
    output logic [NP-1:0]   pad_oe,
    output logic [NP-1:0]   pad_val,
    // sticky status
    output logic [NP-1:0]   viol_legal,
    output logic [NP-1:0]   viol_lease,
    output logic [NP-1:0]   viol_contention,
    output logic            any_violation,
    // violation log, read by the host
    output logic            log_wr,
    output logic [1:0]      log_kind,
    output logic [4:0]      log_pin,
    output logic [TW-1:0]   log_ts,
    output logic            log_overwrite
);
  localparam int PW = (NP < 2) ? 1 : $clog2(NP);  // $clog2(1) is 0: a zero-width port
  localparam int LW_ = (LW < 2) ? 2 : LW;
  localparam int CI  = $clog2(LOGN);

  logic [2:0]      ctr [NP];          // {legal_hi, legal_lo, od}
  logic [LW_-1:0]  lease [NP];
  logic [NP-1:0]   owned;             // pad is currently driven by the engine
  logic            held [NP];         // engine asserted req_rearm this tick
  logic [NP-1:0]   illegal_req;
  logic [NP-1:0]   lease_zero;
  logic [NP-1:0]   legal_hi_v, legal_lo_v;
  logic [TW-1:0]   ts;
  logic [CI:0]     log_wp;
  logic            seen [LOGN];       // entry valid
  logic [1:0]      kind_q [LOGN];
  logic [PW-1:0]   pin_q  [LOGN];

  assign any_violation = |(viol_legal | viol_lease | viol_contention);
  assign log_wr    = any_violation;
  assign log_ts    = ts;
  assign log_pin   = pin_q[log_wp[CI-1:0]];
  assign log_kind  = kind_q[log_wp[CI-1:0]];
  assign log_overwrite = seen[log_wp[CI-1:0]];

  always_comb begin
    for (int p = 0; p < NP; p++) begin
      legal_hi_v[p] = ctr[p][2] | !ctr[p][0];   // a push-pull pin allows 1
      legal_lo_v[p] = ctr[p][1];
      held[p]       = |req_rearm[p];
      // a drive request is legal iff the requested value is in the legal set;
      // releasing (oe = 0) is always legal.
      illegal_req[p] = req_oe[p] &&
                        ((req_val[p] & !legal_hi_v[p]) || (!req_val[p] & !legal_lo_v[p]));
      lease_zero[p]  = (lease[p] == '0);
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int p = 0; p < NP; p++) begin
        ctr[p]       <= 3'b000;
        lease[p]     <= '0;
        owned[p]     <= 1'b0;
        pad_oe[p]    <= 1'b0;
        pad_val[p]   <= 1'b0;
        viol_legal[p]      <= 1'b0;
        viol_lease[p]      <= 1'b0;
        viol_contention[p] <= 1'b0;
      end
      ts <= '0;
      log_wp <= '0;
      for (int k = 0; k < LOGN; k++) begin
        seen[k] <= 1'b0; kind_q[k] <= 2'b00; pin_q[k] <= '0;
      end
    end else begin
      if (cfg_we) ctr[cfg_pin] <= cfg_data[2:0];
      if (ts_clear) ts <= '0;
      else if (any_violation) ts <= ts + 1'b1;

      for (int p = 0; p < NP; p++) begin
        // lease: loaded on acquire, refreshed while re-armed, expired on zero
        if (illegal_req[p] || (!held[p] && lease_zero[p])) begin
          owned[p]  <= 1'b0;                    // fail-silent: release
          lease[p]  <= '0;
          viol_legal[p] <= viol_legal[p] | illegal_req[p];
          viol_lease[p] <= viol_lease[p] | (!held[p] && lease_zero[p] && owned[p]);
        end else if (req_oe[p]) begin
          owned[p] <= 1'b1;
          if (!held[p] || (lease[p] == '0)) lease[p] <= {LW_-1{1'b1}};
          else lease[p] <= lease[p] - 1'b1;
        end else if (held[p] && owned[p] && !lease_zero[p]) begin
          lease[p] <= lease[p] - 1'b1;          // engine still owns it, no re-arm
        end else begin
          owned[p] <= 1'b0;
        end
        // the pad follows the request only while the pin is owned and legal
        pad_oe[p]  <= owned[p] & req_oe[p] & ~illegal_req[p];
        pad_val[p] <= pad_val[p];
        // contention: we drive, the line disagrees
        viol_contention[p] <= viol_contention[p] |
                              (owned[p] & req_oe[p] & ~illegal_req[p] &
                               (line_level[p] != req_val[p]));
      end

      // log the lowest-numbered pin with a new violation, once per tick
      if (any_violation) begin
        seen[log_wp[CI-1:0]] <= 1'b1;
        log_wp <= log_wp + 1'b1;
        for (int p = 0; p < NP; p++) begin
          if (viol_legal[p] || viol_lease[p] || viol_contention[p]) begin
            pin_q[log_wp[CI-1:0]] <= PW'(p);
            if (viol_legal[p])      kind_q[log_wp[CI-1:0]] <= 2'd0;
            else if (viol_lease[p]) kind_q[log_wp[CI-1:0]] <= 2'd1;
            else                    kind_q[log_wp[CI-1:0]] <= 2'd2;
          end
        end
      end
    end
  end
endmodule
