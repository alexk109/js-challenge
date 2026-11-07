# Instruction-set extensions of the parametric ISS (`isa_xor`, `isa_cond`, `isa_irq`)

Normative for `tools/bench/sim/pio_iss_p.py` (`Params.isa_*`) and `asm/pioasm.py` (`assemble(text, isa=...)`). Base:
`docs/mvp-isa.md`. Everything not listed here is as in the MVP; with all three knobs off the ISS is cycle-identical to the
golden ISS (lockstep tests) and the assembler's output is unchanged. All three use encodings the MVP leaves reserved
(`docs/mvp-isa.md` `### Removed and reserved encodings`), so an MVP chip runs a program that contains them as NOPs.
The words are 16 bits; the delay/side-set field (12:8) is as in every instruction.

## Conflict: `isa_cond` and `isa_irq` both need opcode `110`

At 16 bits the only free major opcode is `110`. IRQ is its RP2040 meaning; JMPC is a second JMP. They cannot both exist:
`Params(isa_cond=True, isa_irq=True)` raises, the `iss_isa` backend's `supports()` refuses it, and `assemble(..., isa=
("cond","irq"))` raises. (archsim's `isa.levers` slots say the same: 1 major shared.) Widths of 24/32 bits would lift it;
they are out of scope. `isa_xor` takes a minor slot (MOV Op `11`) and combines with either.

## isa_xor: `xor DST, SRC` (MOV Op `11`)

* **Encoding:** MOV (`101`), Op (4:3) = `11`, Destination (7:5), Source (2:0) as MOV. Mnemonic `xor x, isr` (dst is also
  the first operand: `dst = dst ^ src`).
* **Semantics:** `DST <- DST ^ SRC` over 32 bits. Destination `X`(001), `Y`(010), `ISR`(110), `OSR`(111); any other
  destination (PINS, EXEC, PC, 011) is a NOP. Source as MOV (PINS = the rotated IN window **unmasked**, NULL, STATUS, ...;
  reserved source 100 is a NOP). Writing ISR or OSR resets its shift counter to 0 exactly as MOV does.
* **Cost:** one SM cycle, like MOV; delay and side-set as usual.
* **RTL needed:** MOV decode of Op `11`; a 4:1 mux of the destination register as the ALU's second input (the source mux
  exists) and 32 XOR gates ahead of the write mux (archsim blocks `isa_mov_src_mux` + `isa_xor_alu`).

## isa_cond: `jmpc COND, ADDR` (opcode `110`)

* **Encoding:** `110`, delay/side-set (12:8), Condition (7:5), Address (4:0) (absolute, like JMP).
* **Conditions:**

| 7:5 | name | taken when |
|---|---|---|
| 000 | `x_eq_y` | X == Y |
| 001 | `x_lt_y` | X < Y (unsigned) |
| 010 | `y_lt_x` | Y < X (unsigned) |
| 011 | `pin_low` | `IN_VEC[EXECCTRL.JMP_PIN] == 0` (the complement of `jmp pin`) |
| 100 | `pin_rise` | the JMP_PIN pin had a 0->1 edge since the last `pin_rise` test (latched; the test clears the latch) |
| 101 | `pin_fall` | same, 1->0 |
| 110 | `osre` | `osr_cnt >= thresh(PULL_THRESH)` (the complement of `jmp !osre`) |
| 111 | `isr_full` | `isr_cnt >= thresh(PUSH_THRESH)` |

* **Edge latch:** per engine, the registered pin of the previous **core** cycle (not SM cycle), a rise latch and a fall
  latch. An edge in a core cycle sets its latch at the end of that cycle; a `jmpc pin_rise` executed in the same cycle
  clears it first, so the new edge wins. `SM_RESTART` clears both latches and re-arms the previous-value flop (its next cycle only loads it). Disabling
  the engine alone does not clear them. Because they run at core rate they keep a pulse shorter than the polling period
  (a `jmp pin` loop at CLKDIV 4 misses a one-cycle pulse; `pin_fall` does not).
* **Cost:** one SM cycle taken or not. `x == imm` and `x < imm` are **not** in this set: with a 5-bit address and the delay
  field kept, no bits are left for an immediate (archsim's "x == imm" does not fit at 16 bits; a 24/32-bit word could).
* **RTL needed:** decode of `110` into the JMP unit; a 32-bit unsigned comparator (X vs Y; `==` shares the existing `!=`);
  per engine 3 flops (previous pin, rise, fall) clocked every core cycle; the two complements are free.

## isa_irq: `irq` and `wait irq` (opcode `110`, WAIT source `10`)

* **Flags:** eight, chip-wide (`Chip.irq_flags`, reset 0). Index field (4:0) of `irq` / `wait`: bits 2:0 the flag, bit 4
  `rel` = add the **engine number** to bits 1:0, modulo 4 (RP2040 semantics; bit 2 unchanged), bit 3 reserved.
* **`irq [set|nowait] N [rel]`** (bit 6 = 0, bit 5 = 0): raise flag N. **`irq clear N`** (bit 6 = 1): lower it.
  **`irq wait N`** (bit 6 = 0, bit 5 = 1): raise it, then stall until the flag reads 0 in a later cycle (cleared by another
  engine's `wait 1 irq`, or an `irq clear`; the host has no register for it yet). Bit 7 = 1 is reserved: NOP. Clear wins over wait if both are given.
* **`wait POL irq N`** (WAIT source `10`): stall while flag != POL; a `wait 1 irq` that passes lowers the flag
  (RP2040). On the MVP chip this encoding is the hardwired-0 flag, unchanged.
* **Timing:** all flag reads use the start-of-cycle value; set/clear requests of all engines are applied at the end of the
  cycle, clears first, so a same-cycle set wins. Flag updates are visible to every engine one core cycle later.
* **Host side:** the ISS has `chip.irq_flags` and no host register. The MVP regmap has no free word for the RP2040 `IRQ`
  register at 0x030 (it is inside engine 0's IMEM window), and `INTR`/`IRQx_INTE/INTF/INTS` have no IRQ-flag bits; this lever
  would need a regmap change. No part of that is modelled or measured here.
* **RTL needed:** an 8-bit flag register with set/clear from four engines; WAIT source `10` mux; a stall-state flop per
  engine for `irq wait`; the host read/clear register and (optionally) the interrupt lines.

## Where the effects are measured

`tools/bench/isa_fw/` (programs, tests, `compare.py`), the `iss_isa` backend (`tools/bench/backends/isa.py`), the result
table in `tools/bench/notes/L.md`.
