# Generalized register map of the parametric ISS (`pio_iss_p`)

Normative for `tools/bench/sim/pio_iss_p.py` and `fw/regmap.py`; the future parametric RTL (task H)
should implement the same thing. Base: `docs/mvp-regmap.md`. Everything not listed here is as in the
MVP. Parameters: `n_engines` 1-4, `imem_words` 16/32/64/128, `fifo_depth` 2-16, `engine_route`,
`t_in` 2-4 (the last has no register effect).

## 1. The MVP map is unchanged where it fits

The MVP's 7-bit map already reserves the RP2040 positions for four engines, so these do **not** move:

| Register | Address | Generalization |
|---|---|---|
| `CTRL` | `0x00` | `SM_ENABLE` bit `e`, `SM_RESTART` bit `4+e`, `CLKDIV_RESTART` bit `8+e` for `e < n_engines`; higher bits ignored. New: `ROUTE_EN` bits `16+e` (section 3) |
| `FSTAT`, `FDEBUG` | `0x01`, `0x02` | field bit `+e` for `e < n_engines` (4-bit fields, 8-bit stride in `FDEBUG`); `TXFULL`/`RXFULL` mean "holds `fifo_depth` words" |
| `FLEVEL` | `0x03` | engine `e`: TX level bits `8e+3:8e`, RX level bits `8e+7:8e+4`. **A nibble saturates at 15**, so with `fifo_depth = 16` a full FIFO reads 15 (read `FSTAT` for full) |
| `TXF0..3` | `0x04..0x07` | engine `e` at `0x04+e`; addresses of absent engines read 0, ignore writes |
| `RXF0..3` | `0x08..0x0B` | engine `e` at `0x08+e` |
| `DBG_PADOUT/PADOE` | `0x0F`, `0x10` | unchanged |
| `DBG_CFGINFO` | `0x11` | `FIFO_DEPTH` bits 5:0, `SM_COUNT` bits 11:8, `IMEM_SIZE` bits 23:16 (MVP value `0x0020_0204`; bits 21:16 are the RP2040 field, 23:22 were reserved zero) |
| `SMe_CLKDIV..PINCTRL` | `0x32 + 6e + i`, `e < 4` (up to `0x49`) | unchanged; `EXECCTRL` gains the `PAGE` field (section 2) |
| `INTR`, `IRQx_INTE/INTF/INTS` | `0x4A..0x50` | `RXNEMPTY` bit `e`, `TXNFULL` bit `4+e`; stored `INTE/INTF` bits are those for `e < n_engines`. Reset values `0x003` / `0x010` masked to existing engines |

## 2. IMEM, and when the address space grows

`wide = (n_engines > 2) or (imem_words > 32)`.

* **Not wide** (all configs with `n_engines <= 2` and `imem_words <= 32`, including the MVP): 7-bit
  addresses, IMEM word `k` of engine `e` at `0x12 + 0x40*e + k`, exactly the MVP. With
  `imem_words = 16` words 16..31 are unmapped (write ignored). With `n_engines = 1` engine 1's
  registers, FIFOs and IMEM window are unmapped.
* **Wide**: 10-bit word addresses (0..0x3FF). Every register above keeps its address; IMEM moves
  out of the way: engine `e` word `k` at `0x80 + 0x80*e + k` (`k < imem_words`). The old IMEM windows
  (`0x12..0x31`, `0x52..0x71`) become reserved. Bursts wrap mod 1024 and the FIFO windows
  (`0x04..0x0B`) still hold the address.
* A bus access outside the address space is an error in the ISS (`ValueError`), as in the golden ISS.

**Host-port format implication for the RTL (task H).** The MVP command byte `{RW, ADDR[6:0]}` cannot
name a wide address. A wide chip needs a 2-byte command `{RW, 5'b0, ADDR[9:0]}` (first byte
`RW,0,0,0,0,0,A9,A8`, second byte `A7..A0`), burst/turnaround rules unchanged; the non-wide chip
keeps the 1-byte command. (The ISS does not model the serial framing; `fw`'s `ChipDevice` queue takes
the word address directly.)

### PC width and the `PAGE` field

`JMP`, `OUT PC` and `MOV PC` carry a 5-bit address, as does `WRAP_TOP`/`WRAP_BOTTOM`. With more than
32 words the PC is `log2(imem_words)` bits wide and these 5-bit values are *offsets inside a page*:

* `EXECCTRL[21:20] = PAGE` (stored only when `imem_words > 32`: 1 bit for 64 words, 2 bits for 128;
  otherwise it reads 0 and ignores writes; bits 23:17 were unused in the MVP).
* A taken `JMP`, `OUT PC`, `MOV PC` loads `PC = PAGE*32 + addr5`. The wrap compare is against
  `PAGE*32 + WRAP_TOP`, and the wrap target is `PAGE*32 + WRAP_BOTTOM`.
* A sequential PC increment carries across pages and wraps at `imem_words` (`pc = (pc+1) mod
  imem_words`). So a program longer than 32 words runs from `PAGE=0` straight into page 1, and a
  program that lives in page `p` sets `PAGE = p` and uses ordinary 5-bit addresses.
* With `imem_words = 16` the PC is 4 bits wide: it wraps mod 16, and **every PC-valued field is truncated to
  4 bits**: a 5-bit jump / `OUT PC` / `MOV PC` address and the wrap-bottom target lose their top bit. The
  wrap *compare* is not truncated (`pc == WRAP_TOP` with a `WRAP_TOP` of 16..31 never matches, so such a
  window never wraps and the PC runs mod 16). The RTL and the ISS agree (`lockstep_rtl_p` at 16 words).
* `SMe_ADDR` reads the full PC.
* At `imem_words <= 32` the PC is `& 0x1F`, as the MVP, whatever `PAGE` holds (it holds 0).

## 3. Engine route (`engine_route = true`)

Link `e` (for `e = 0 .. n_engines-2`) carries words from **engine `e`'s RX FIFO** to **engine `e+1`'s TX FIFO**.

* **Enable:** `CTRL[16+e]` = `ROUTE_EN` of link `e` (so bit 16 is link 0 -> 1, bit 17 is 1 -> 2, bit 18
  is 2 -> 3). Plain RW bits, reset 0, read back in `CTRL`. Without `engine_route` (or for a link that
  does not exist) the bit reads 0 and ignores writes.
* **Transfer:** each core cycle the link moves **at most one word**, the head of RX `e` to the tail of TX
  `e+1`, if at the **start of that cycle** RX `e` is non-empty and TX `e+1` has room. The decision uses
  start-of-cycle levels only (the ISS convention); the move lands at the end of the cycle. Latency for a
  word the engine pushes in cycle `n`: visible in RX `e` from `n+1`, moved at the end of `n+1`, in TX
  `e+1` from `n+2`, so an engine pulling in `n+2` can take it. Throughput is 1 word per cycle.
* **Room means:** `tx_level(e+1) + host_push < fifo_depth`, with the TX level taken at the start of the
  cycle (an engine pop in the same cycle does not free a slot for the link until the next cycle).
  A host write to `TXF(e+1)` in the same cycle takes the slot first (host wins); a host write to a full
  TX FIFO still sets `TXOVER` and is dropped as usual.
* **Backpressure:** the link itself never drops a word and sets no flag. When TX `e+1` is full the word stays at the
  head of RX `e`; RX `e` fills; engine `e`'s `PUSH` then stalls (`block`) or drops (non-blocking, `RXSTALL`
  in `FDEBUG`), or its autopush stalls, exactly as for a host that does not read. Engine `e+1`
  stalls on an empty TX (`TXSTALL`) as usual. A chain of links backpressures stage by stage.
  The only loss path: a non-blocking `PUSH` (or `PUSH` with `if_full` off) into a full RX `e` drops the
  word and sets `RXSTALL`, the same as for a host that does not read; a blocking `PUSH` or autopush stalls instead.
* **What the host sees:** engine `e`'s RX FIFO belongs to the link while `ROUTE_EN[e]` is 1.
  `RXFe` reads return 0, with no pop and no `RXUNDER`. `FSTAT.RXEMPTY/RXFULL`, `FLEVEL`, `INTR` and the
  IRQ pins still show the real RX FIFO of engine `e` (so a backlog is visible), and the `TXF(e+1)`
  level shows the words that arrived. The host may still write `TXF(e+1)`; its words and the link's
  words share the FIFO in arrival order.
* **Disable:** clearing the bit leaves any words already in RX `e` there for the host to read.
* Engines need not be enabled for a link to move words (it is plumbing between two FIFOs). `SM_RESTART`
  does not clear FIFOs (as the MVP) and does not touch `ROUTE_EN`.
