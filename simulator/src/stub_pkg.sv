// SPDX-License-Identifier: Apache-2.0
//
// Package for the SystemVerilog flow stub (CFG=stub-sv). Exercises the
// constructs the product RTL is expected to use: packages, enums, functions,
// typed parameters.

package stub_pkg;

  typedef enum logic [1:0] {
    TX_IDLE,
    TX_START,
    TX_DATA,
    TX_STOP
  } tx_state_e;

  function automatic int unsigned clks_per_bit(int unsigned clk_hz, int unsigned baud);
    return (clk_hz + baud / 2) / baud;
  endfunction

endpackage
