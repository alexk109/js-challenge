# equiv_mvp: the parametric RTL at default parameters is the original MVP chip

`make bench-equiv` (`run.py`; `EQUIV_BMC=1` adds the BMC run). Original = `src/` at commit `45efee26` (byte-identical to
wb/bench before task H), every module renamed `orig_*` by `prepare.py`; new = the current `src/`. Defaults, same inputs.

* `equiv.ys.in` (filled in by `run.py`): Yosys sequential equivalence, at two levels. **Top**: `tt_um_protocol_emulator`
  (host_spi + pio_chip; inputs `ui_in uio_in ena rst_n`, outputs `uo_out uio_out uio_oe`): 5924 equivalence points, all proved.
  **Chip**: `pio_chip` (pins and the register bus; `pin_out pin_oe irq0 irq1 bus_rdata`): 5452, all proved. Both designs are
  flattened (slang front end), state registers and outputs are paired by name and `equiv_induct` proves that paired
  registers stay equal and the outputs agree.
* Negative controls: `prepare.py --mutant --top top|chip` changes one bit of the new design (`uo_out[1]` / `pin_oe[0]`); the
  same script must then leave exactly 1 point unproven, and does.
* `bmc.sby` (opt-in): SymbiYosys miter of `pio_chip` (`miter.sv`: reset in the first cycle, free afterwards), engine `abc bmc3`,
  depth 40 requested. **It only reaches depth 4** (frame 4 takes 30 s, frame 5 did not finish in 30 minutes; `smtbmc yices`
  did not pass step 5 in 6 minutes; `abc pdr` stalled at frame 4): two full chips with free 32-bit buses are too hard
  for these solvers. It found no counterexample up to the depth reached and it is not evidence beyond that.

## Soundness note

`equiv_induct` is an induction step without a base case. It is sound here because reset precedes use: the registers all have an
asynchronous reset (`async2sync` models it as a mux on the next state), and the state right after reset is a state of the proven
invariant (every paired register equal), so every reachable state from reset satisfies it. What is **not** covered is power-up
without reset (unreset flops hold arbitrary, possibly different, values in the two designs). The paired-register
correspondence is by name, which is what makes the invariant strong enough; registers that exist in only one design (the new
`route_en` is constant 0 at the defaults, `cfg_page` is constant 0, the second FIFO write port is absent) are optimised away
before pairing and show up as no equivalence point.
