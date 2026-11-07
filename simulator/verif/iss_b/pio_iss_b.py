"""Independent cycle-accurate ISS of the MVP protocol-emulator chip.

Sources: `docs/mvp-isa.md` (M0) and `docs/mvp-regmap.md` (M0). Nothing here was
taken from `iss/`, `src/` or `asm/`, and nothing from memory of the RP2040: the
spec text is the only source. Ambiguities and the reading taken are recorded in
`verif/iss_b/SPEC-QUESTIONS.md`.

Cycle convention (ISA `### Conventions`)
----------------------------------------
`step()` runs one core cycle and the attributes describe the state at the end of
that cycle. The first `step()` after `reset()` is core cycle 0, in which every
flop still holds its reset value, so the first change is visible in cycle 1.

`pin_out`, `pin_oe`, `irq0` and `irq1` are registered pad outputs: after `step()`
for cycle `n` they hold what is on the pads in cycle `n+1` (T_OUT = 1, and the
interrupt pins are "registered once" from the state of cycle `n`). The same
vectors are what `DBG_PADOUT` / `DBG_PADOE` read combinationally in cycle `n+1`,
so the ISS and the register file cannot disagree.

Public API: exactly the `Chip` class of `docs/mvp-interfaces.md`.
"""

M32 = 0xFFFFFFFF
NUM_ENGINES = 2
FIFO_DEPTH = 4
T_IN = 3  # an engine in cycle n sees the pad value of cycle n-3
T_IMEM = 2  # a host IMEM write in cycle h is first executed in cycle h+2

# Pin kinds (ISA `## Pins` `### The 18 protocol pins`).
PINS_BIDIR = range(0, 8)  # uio[0..7], level and direction registers
PINS_IN_ONLY = range(8, 13)  # ui_in[3..7], writes dropped
PINS_OUT_ONLY = range(13, 18)  # uo_out[3..7], always driven

SRC_IMEM = "IMEM"
SRC_LATCH = "LATCH"
SRC_HOST = "HOST"


def thresh(f: int) -> int:
    """ISA `### Conventions`: `thresh(f)` = `f` if `f != 0`, else 32."""
    return f if f else 32


def mask(k: int) -> int:
    """ISA `### Conventions`: `mask(k)` = the low `k` bits set."""
    return M32 if k >= 32 else (1 << k) - 1


def rot32(v: int, k: int) -> int:
    """Rotate `v` right by `k`; bit 0 of the result is bit `k` of `v`."""
    k &= 31
    return v & M32 if k == 0 else ((v >> k) | (v << (32 - k))) & M32


def brev32(v: int) -> int:
    """MOV Op `10`: bit `n` <- bit `31-n`."""
    return int(format(v & M32, "032b")[::-1], 2)


def window(base: int, count: int, data: int = 0) -> tuple[int, int]:
    """A pin write request for the `(BASE, COUNT)` window, as `(mask, data)`.

    Data bit `k` goes to pin `(BASE + k) mod 32`, so the data is pre-aligned to
    absolute pin positions here (ISA `### Pin writes`).
    """
    mask_ = 0
    out = 0
    for k in range(max(0, count)):
        pin = (base + k) & 31
        mask_ |= 1 << pin
        out |= ((data >> k) & 1) << pin
    return mask_, out


def sideset_count(eng: "Engine") -> int:
    """`SC` = `PINCTRL.SIDESET_COUNT`, with 6 and 7 behaving as 5 (MVP)."""
    return min(eng.sideset_count, 5)


def delay_of(instr: int, eng: "Engine") -> int:
    """Low `5 - SC` bits of the delay/side-set field (ISA `### Field values`)."""
    return (instr >> 8) & mask(5 - sideset_count(eng))


class Engine:
    """One engine: its flops (ISA `### Engine state`) and its config registers.

    Every attribute holds the *start-of-cycle* value while `Chip.step()` runs; the
    engine's own updates are collected in a dict and committed at the end of the
    cycle, so the host access of that cycle always sees the start-of-cycle state.
    """

    def __init__(self, index: int) -> None:
        self.index = index
        self.imem = [0] * 32
        self.tx: list[int] = []  # TX FIFO: host -> engine
        self.rx: list[int] = []  # RX FIFO: engine -> host
        self.clkdiv = 0x00010000
        self.load_cfg()
        self.reset_state()

    # -- reset values (ISA `### Engine state`, regmap `### Per-engine registers`)

    def reset_state(self) -> None:
        self.pc = 0
        self.ir = 0
        self.x = 0
        self.y = 0
        self.osr = 0
        self.isr = 0
        self.osr_cnt = 32
        self.isr_cnt = 0
        self.delay = 0
        self.stalled = 0
        self.latch = 0
        self.latch_pend = 0
        self.latch_host = 0
        self.force = 0
        self.d_cnt = 1  # divider cnt (17 bits), reset 1
        self.d_acc = 0  # divider acc (8 bits), reset 0
        self.d_en = 0  # divider en, reset 0

    def restart_state(self) -> None:
        """`CTRL.SM_RESTART`: the fields the ISA table marks 0.

        PC, IR, X, Y, OSR, latch, the FIFOs and the divider are kept.
        """
        self.isr = 0
        self.osr_cnt = 32
        self.isr_cnt = 0
        self.delay = 0
        self.stalled = 0
        self.latch_pend = 0
        self.latch_host = 0
        self.force = 0

    def cfg_regs(self) -> dict:
        """The four configuration registers as raw values."""
        return {
            "clkdiv": self.clkdiv,
            "execctrl": self.execctrl,
            "shiftctrl": self.shiftctrl,
            "pinctrl": self.pinctrl,
        }

    def load_cfg(self, **kw) -> None:
        """Decode the configuration registers. Unlisted registers keep their value.

        Reset values: CLKDIV `0x0001_0000`, EXECCTRL `0x0001_F000`, SHIFTCTRL
        `0x000C_0000`, PINCTRL `0x1400_0000` (regmap `### Per-engine registers`).
        """
        clkdiv = kw.get("clkdiv", 0x00010000)
        execctrl = kw.get("execctrl", 0x0001F000)
        shiftctrl = kw.get("shiftctrl", 0x000C0000)
        pinctrl = kw.get("pinctrl", 0x14000000)
        self.clkdiv = clkdiv & 0xFFFFFF00  # INT is 31:16, FRAC 15:8; 7:0 reserved
        self.execctrl = execctrl
        self.shiftctrl = shiftctrl
        self.pinctrl = pinctrl
        self.div_int = (clkdiv >> 16) & 0xFFFF
        self.div_frac = (clkdiv >> 8) & 0xFF
        self.side_en = (execctrl >> 30) & 1
        self.side_pindir = (execctrl >> 29) & 1
        self.jmp_pin = (execctrl >> 24) & 0x1F
        self.wrap_top = (execctrl >> 12) & 0x1F
        self.wrap_bottom = (execctrl >> 7) & 0x1F
        self.status_sel = (execctrl >> 4) & 1
        self.status_n = execctrl & 0xF
        self.pull_thresh = (shiftctrl >> 25) & 0x1F
        self.push_thresh = (shiftctrl >> 20) & 0x1F
        self.out_shiftdir = (shiftctrl >> 19) & 1
        self.in_shiftdir = (shiftctrl >> 18) & 1
        self.autopull = (shiftctrl >> 17) & 1
        self.autopush = (shiftctrl >> 16) & 1
        self.sideset_count = (pinctrl >> 29) & 7
        self.set_count = (pinctrl >> 26) & 7
        self.out_count = (pinctrl >> 20) & 0x3F
        self.in_base = (pinctrl >> 15) & 0x1F
        self.sideset_base = (pinctrl >> 10) & 0x1F
        self.set_base = (pinctrl >> 5) & 0x1F
        self.out_base = pinctrl & 0x1F

    def state(self) -> dict:
        """The canonical engine state (`docs/mvp-interfaces.md` `### ISS API`)."""
        return {
            "pc": self.pc,
            "ir": self.ir,
            "x": self.x,
            "y": self.y,
            "osr": self.osr,
            "isr": self.isr,
            "osr_cnt": self.osr_cnt,
            "isr_cnt": self.isr_cnt,
            "delay_cnt": self.delay,
            "stalled": self.stalled,
            "latch": self.latch,
            "latch_pend": self.latch_pend,
            "latch_host": self.latch_host,
            "forced": self.force,
            "tx_level": len(self.tx),
            "rx_level": len(self.rx),
        }


class Chip:
    """The chip: two engines, the register file, the pin level/direction registers.

    `step()` runs one core cycle, exactly one host bus access at most, and
    resolves the engines' pin write requests. See the module docstring for the
    cycle convention.
    """

    def __init__(self) -> None:
        self.eng = [Engine(0), Engine(1)]
        self.lvl = 0  # pin level registers: pins 0..7 and 13..17 only
        self.dir = 0  # pin direction registers: pins 0..7 only
        # The pad value presented in cycle m reaches IN_VEC in cycle m + T_IN, so the
        # chain holds T_IN slots: this cycle's IN_VEC is the oldest one.
        self.in_pipe = [0] * T_IN
        self.sm_enable = 0
        self.fdebug = 0
        self.inte = [0x003, 0x010]  # MVP reset values (regmap `### tx_ready and rx_valid`)
        self.intf = [0, 0]
        self.pin_out = 0
        self.pin_oe = 0x3E000
        self.irq0 = 0
        self.irq1 = 0

    def reset(self) -> None:
        """Power-on state (regmap reset values, ISA `### Engine state`)."""
        self.__init__()  # noqa: PLC2801 - plain re-initialisation, no inheritance here

    def state(self, e: int) -> dict:
        """Canonical engine state after the last step."""
        return self.eng[e].state()

    # ------------------------------------------------------------------ cycle

    def step(self, pin_in: int, bus: tuple | None = None) -> int | None:
        """Advance one core cycle. See `docs/mvp-interfaces.md` `### ISS API`."""
        pin_in &= (1 << 18) - 1
        in_vec = self.in_pipe.pop(0)  # IN_VEC of this cycle: the pad of cycle n-3
        pad = (pin_in & 0x1FFF) | (self.lvl & 0x3E000)  # pad value of *this* cycle

        # Registered interrupt pins: the value in cycle n+1 from the state of n, so
        # a FIFO change at the end of cycle n shows on the pin from cycle n+2.
        intr = self._intr()
        irq0 = 1 if (intr & self.inte[0]) | self.intf[0] else 0
        irq1 = 1 if (intr & self.inte[1]) | self.intf[1] else 0

        # Engines: the SM-cycle decision uses the start-of-cycle force/enable/divider.
        ups: list[dict] = []
        reqs: list[dict] = []
        fdbg_set = 0
        for e, eng in enumerate(self.eng):
            if eng.force or (((self.sm_enable >> e) & 1) and eng.d_en):
                upd, req, flags = self._sm_cycle(eng, in_vec)
                fdbg_set |= flags
            else:
                upd, req = {}, {}
            ups.append(upd)
            reqs.append(req)

        # The one host bus access of this cycle, on the start-of-cycle state.
        w: dict = {}
        rdata = None
        if bus is not None:
            bus = tuple(bus)
            if bus[:1] == ("r",) and len(bus) == 2:
                rdata, rx_pop = self._read(bus[1] & 0x7F, w)
                w["rx_pop"] = rx_pop
            elif bus[:1] == ("w",) and len(bus) == 3:
                self._write(bus[1] & 0x7F, bus[2] & M32, w)
            # Anything else is not a bus access: the cycle runs with no access.

        self._resolve_pins(reqs)  # T_OUT = 1: visible from the next cycle
        self._commit(ups, w, fdbg_set | w.get("fdebug_set", 0))

        self.in_pipe.append(pad)
        self.irq0, self.irq1 = irq0, irq1
        self._sync_pins()
        return rdata

    def _commit(self, ups: list[dict], w: dict, fdbg_set: int) -> None:
        """Apply this cycle's state changes at its end, host after engine."""
        # FDEBUG: a flag set by an engine in this cycle stays set.
        self.fdebug = (self.fdebug & ~w.get("fdebug_clr", 0)) | fdbg_set

        for e, eng in enumerate(self.eng):
            upd = ups[e]
            # FIFO effects: host and engine each saw the start-of-cycle state, and a
            # push and a pop in the same cycle both happen (ISA `### Host-to-engine`).
            tx_pop = upd.pop("_tx_pop", None)
            rx_push = upd.pop("_rx_push", None)
            if tx_pop is not None:
                eng.tx.pop(0)
            if w.get("rx_pop") == e and eng.rx:
                eng.rx.pop(0)
            push = w.get("tx_push")
            if push is not None and push[0] == e and push[1]:
                eng.tx.append(push[2])
            if rx_push is not None:
                eng.rx.append(rx_push)

            for name, value in upd.items():
                setattr(eng, name, value)

            if w.get("instr") is not None and w["instr"][0] == e:
                # The host's value wins the latch, even against OUT/MOV EXEC.
                eng.latch = w["instr"][1]
                eng.latch_host = 1
                eng.force = 1

            if (w.get("sm_restart", 0) >> e) & 1:
                eng.restart_state()  # the restart wins for the fields it clears

            self._divider(eng, e, w)
            # IR updates in every core cycle, from the IMEM content of this cycle
            # and the PC value for the next cycle (a write takes effect at its end).
            eng.ir = eng.imem[eng.pc]

            if w.get("imem") is not None and w["imem"][0] == e:
                eng.imem[w["imem"][1]] = w["imem"][2]
            cfg = w.get("cfg", {}).get(e)
            if cfg is not None:
                eng.load_cfg(**{**eng.cfg_regs(), **cfg})  # other registers kept
        if w.get("sm_enable") is not None:
            self.sm_enable = w["sm_enable"]
        for i, mask_ in ((0, "inte0"), (1, "inte1")):
            if w.get(mask_) is not None:
                self.inte[i] = w[mask_] & 0x33  # bits 5:4 and 1:0 only
        for i, mask_ in ((0, "intf0"), (1, "intf1")):
            if w.get(mask_) is not None:
                self.intf[i] = w[mask_] & 0x33

    def _divider(self, eng: Engine, e: int, w: dict) -> None:
        """The clock divider, normative algorithm (ISA `### Clock divider`)."""
        if (w.get("clkdiv_restart", 0) >> e) & 1:
            eng.d_cnt, eng.d_acc, eng.d_en = 1, 0, 0
        elif eng.d_cnt == 1:
            eng.d_en = 1
            total = eng.d_acc + eng.div_frac
            eng.d_acc = total & 0xFF
            inte = eng.div_int if eng.div_int else 65536
            eng.d_cnt = inte + (1 if total > 0xFF else 0)
        else:
            eng.d_en = 0
            eng.d_cnt -= 1

    # -------------------------------------------------------------------- pins

    def _sync_pins(self) -> None:
        """`pin_out` / `pin_oe`: the registered, resolved pad state.

        Input-only pins 8..12 have no level register and no direction register
        (both read 0); output-only pins 13..17 read 1 in `pin_oe`.
        """
        self.pin_out = self.lvl & 0x3E0FF
        self.pin_oe = (self.dir & 0xFF) | 0x3E000

    def _resolve_pins(self, reqs: list[dict]) -> None:
        """Resolve the engines' write requests (ISA `### Pin writes`).

        Level and direction registers are resolved separately, per pin. Within an
        engine the side-set channel wins over the OUT/SET channel; between engines
        engine 1 wins. Writes to pins that cannot take them are dropped here.
        """
        order: list[tuple[str, int, int]] = []
        for e in (1, 0):
            for (channel, is_dir), (m, d) in sorted(
                reqs[e].items(), key=lambda kv: 0 if kv[0][0] == "side" else 1
            ):
                order.append(("dir" if is_dir else "lvl", m, d))
        lvl_order = [(m, d) for kind, m, d in order if kind == "lvl"]
        dir_order = [(m, d) for kind, m, d in order if kind == "dir"]
        lvl, dirs = self.lvl, self.dir
        for k in range(18):
            bit = 1 << k
            for m, d in lvl_order:
                if m & bit:
                    if k in PINS_BIDIR or k in PINS_OUT_ONLY:
                        lvl = (lvl & ~bit) | ((d >> k) & 1) * bit
                    break
            for m, d in dir_order:
                if m & bit:
                    if k in PINS_BIDIR:
                        dirs = (dirs & ~bit) | ((d >> k) & 1) * bit
                    break
        self.lvl, self.dir = lvl, dirs

    # ------------------------------------------------------- register file

    def _intr(self) -> int:
        """`INTR`: `SMn_TXNFULL` (5:4) and `SMn_RXNEMPTY` (1:0); IRQ flags read 0."""
        v = 0
        for e, eng in enumerate(self.eng):
            if len(eng.tx) < FIFO_DEPTH:
                v |= 1 << (4 + e)
            if eng.rx:
                v |= 1 << e
        return v

    def _read(self, addr: int, w: dict) -> tuple[int, int | None]:
        """One bus read of `addr` on the start-of-cycle state.

        Returns (data, engine whose RX FIFO is popped at the end of the cycle).
        """
        pop = None
        if addr == 0x00:  # CTRL: the SC bits read 0
            v = self.sm_enable & 3
        elif addr == 0x01:  # FSTAT
            v = _fstat(self)
        elif addr == 0x02:  # FDEBUG
            v = self.fdebug
        elif addr == 0x03:  # FLEVEL
            v = _flevel(self)
        elif 0x08 <= addr <= 0x09:  # RXF0, RXF1: RF
            e = addr - 0x08
            if self.eng[e].rx:
                v, pop = self.eng[e].rx[0], e
            else:
                v = 0  # MVP: an empty read returns 0 and sets RXUNDER
                w["fdebug_set"] = w.get("fdebug_set", 0) | (1 << (8 + e))
        elif addr == 0x0F:  # DBG_PADOUT
            v = self.lvl & 0x3E0FF
        elif addr == 0x10:  # DBG_PADOE
            v = (self.dir & 0xFF) | 0x3E000
        elif addr == 0x11:  # DBG_CFGINFO
            v = 0x00200204
        elif addr == 0x4A:  # INTR
            v = self._intr()
        elif addr == 0x4D:  # IRQ0_INTS
            v = (self._intr() & self.inte[0]) | self.intf[0]
        elif addr == 0x50:  # IRQ1_INTS
            v = (self._intr() & self.inte[1]) | self.intf[1]
        elif addr in (0x4B, 0x4E):  # IRQx_INTE
            v = self.inte[(addr - 0x4B) // 3] & 0x33
        elif addr in (0x4C, 0x4F):  # IRQx_INTF
            v = self.intf[(addr - 0x4C) // 3] & 0x33
        elif 0x12 <= addr <= 0x31 or 0x52 <= addr <= 0x71:  # INSTR_MEM: WO
            v = 0
        elif 0x32 <= addr <= 0x3D:  # SMe_*
            eng = self.eng[(addr - 0x32) // 6]
            i = (addr - 0x32) % 6
            if i == 0:  # CLKDIV
                v = eng.clkdiv
            elif i == 1:  # EXECCTRL
                v = (
                    ((eng.latch_pend & eng.latch_host) << 31)
                    | (eng.side_en << 30)
                    | (eng.side_pindir << 29)
                    | (eng.jmp_pin << 24)
                    | (eng.wrap_top << 12)
                    | (eng.wrap_bottom << 7)
                    | (eng.status_sel << 4)
                    | eng.status_n
                )
            elif i == 2:  # SHIFTCTRL
                v = (
                    (eng.pull_thresh << 25)
                    | (eng.push_thresh << 20)
                    | (eng.out_shiftdir << 19)
                    | (eng.in_shiftdir << 18)
                    | (eng.autopull << 17)
                    | (eng.autopush << 16)
                )
            elif i == 3:  # ADDR: the PC
                v = eng.pc & 0x1F
            elif i == 4:  # INSTR: reads IR
                v = eng.ir & 0xFFFF
            else:  # PINCTRL
                v = (
                    (eng.sideset_count << 29)
                    | (eng.set_count << 26)
                    | (eng.out_count << 20)
                    | (eng.in_base << 15)
                    | (eng.sideset_base << 10)
                    | (eng.set_base << 5)
                    | eng.out_base
                )
        else:  # TXF (WO), reserved and unmapped
            v = 0
        return v, pop

    def _write(self, addr: int, data: int, w: dict) -> None:
        """One bus write of cycle `h`; every effect lands at the end of the cycle.

        RO, WO and reserved addresses ignore writes (regmap `### Address map`).
        """
        if addr == 0x00:  # CTRL
            w["sm_enable"] = data & 0x3
            w["sm_restart"] = (data >> 4) & 0x3
            w["clkdiv_restart"] = (data >> 8) & 0x3
        elif addr == 0x02:  # FDEBUG: W1C
            w["fdebug_clr"] = data
        elif 0x04 <= addr <= 0x05:  # TXF0, TXF1: WF
            e = addr - 0x04
            room = len(self.eng[e].tx) < FIFO_DEPTH
            w["tx_push"] = (e, room, data)
            if not room:  # dropped, TXOVER
                w["fdebug_set"] = w.get("fdebug_set", 0) | (1 << (16 + e))
        elif 0x12 <= addr <= 0x31 or 0x52 <= addr <= 0x71:  # INSTR_MEM
            e = 0 if addr < 0x40 else 1
            w["imem"] = (e, addr - (0x12 + 0x40 * e), data & 0xFFFF)
        elif 0x32 <= addr <= 0x3D:  # SMe_*
            e = (addr - 0x32) // 6
            i = (addr - 0x32) % 6
            if i == 0:
                w.setdefault("cfg", {}).setdefault(e, {})["clkdiv"] = data & 0xFFFFFF00
            elif i == 1:
                w.setdefault("cfg", {}).setdefault(e, {})["execctrl"] = data
            elif i == 2:
                w.setdefault("cfg", {}).setdefault(e, {})["shiftctrl"] = data
            elif i == 4:  # INSTR: force the latch
                w["instr"] = (e, data & 0xFFFF)
            elif i == 5:
                w.setdefault("cfg", {}).setdefault(e, {})["pinctrl"] = data
        elif addr in (0x4B, 0x4E):  # IRQx_INTE
            w["inte0" if addr == 0x4B else "inte1"] = data
        elif addr in (0x4C, 0x4F):  # IRQx_INTF
            w["intf0" if addr == 0x4C else "intf1"] = data
        # SMe_ADDR (3), FSTAT, FLEVEL, DBG_*, INTR, IRQx_INTS: read-only
        # Everything else: reserved, ignored.

    # ----------------------------------------------------------- engine step

    def _sm_cycle(self, eng: Engine, in_vec: int) -> tuple[dict, dict, int]:
        """The normative `step(e)` of ISA `### Reference step`.

        Returns (updates, pin write requests, FDEBUG flags to set).
        """
        upd: dict = {}
        req: dict = {}
        fdbg = 0

        def cur(name: str):
            return upd[name] if name in upd else getattr(eng, name)

        if eng.force:
            instr, src = eng.latch, SRC_HOST
        elif eng.latch_pend:
            instr, src = eng.latch, SRC_LATCH
        elif eng.delay > 0:
            upd["delay"] = eng.delay - 1
            self._bg_autopull(eng, upd, cur)
            return upd, req, fdbg
        else:
            instr, src = eng.ir, SRC_IMEM

        # 1. side-set, on the first cycle of the instruction only. A host-forced
        #    instruction writes it every cycle it runs, a stalled one only the first.
        sc = sideset_count(eng)
        if sc and (src == SRC_HOST or not eng.stalled):
            side_bits = (instr >> 8) >> (5 - sc)
            if eng.side_en:
                # The MSB of the side-set bits is the enable, so only `SC - 1` data
                # bits are left and the window covers `SC - 1` pins.
                if (side_bits >> (sc - 1)) & 1:
                    data = side_bits & mask(sc - 1)
                    req[("side", bool(eng.side_pindir))] = window(
                        eng.sideset_base, sc - 1, data
                    )
            else:
                req[("side", bool(eng.side_pindir))] = window(
                    eng.sideset_base, sc, side_bits & mask(sc)
                )

        # 2. main operation
        stall, new_pc, flags = self._execute(eng, instr, in_vec, upd, req, cur)
        fdbg |= flags

        # 3. bookkeeping
        is_exec = _is_exec(instr)
        if stall:
            upd["stalled"] = 1
            if src != SRC_IMEM:
                upd["latch"] = instr
                upd["latch_pend"] = 1
                upd["latch_host"] = 1 if src == SRC_HOST else cur("latch_host")
            # PC unchanged, delay not loaded
        else:
            upd["stalled"] = 0
            if src != SRC_IMEM and not is_exec:
                upd["latch_pend"] = 0
            upd["pc"] = self._pc_update(eng, cur, instr, src, new_pc)
            if not is_exec and not (src != SRC_IMEM and cur("latch_host")):
                upd["delay"] = delay_of(instr, eng)
        if (instr >> 13) & 7 != 3:  # not OUT: OUT does its own autopull
            self._bg_autopull(eng, upd, cur)
        upd["force"] = 0
        return upd, req, fdbg

    def _bg_autopull(self, eng: Engine, upd: dict, cur) -> None:
        """`background_autopull()` (ISA `### Reference step`).

        Runs on every SM cycle that does not execute an OUT, including delay and
        stall cycles and forced cycles. It can never pop twice in one cycle: an
        instruction that popped already set `osr_cnt = 0`.
        """
        if not eng.autopull:
            return
        if cur("osr_cnt") >= thresh(eng.pull_thresh) and len(eng.tx):
            upd["_tx_pop"] = eng.tx[0]
            upd["osr"] = eng.tx[0]
            upd["osr_cnt"] = 0

    def _pc_update(self, eng: Engine, cur, instr: int, src: str, new_pc: int | None) -> int:
        """`pc_update(I, src)` (ISA `### Reference step`)."""
        if new_pc is not None:  # taken JMP, or MOV PC / OUT PC
            return new_pc & 0x1F
        if src != SRC_IMEM:  # an EXEC'd or forced instruction does not advance the PC
            return cur("pc")
        pc = cur("pc")
        if pc == eng.wrap_top:
            return eng.wrap_bottom
        return (pc + 1) & 0x1F

    # ------------------------------------------------------- main operations

    def _execute(
        self, eng: Engine, instr: int, in_vec: int, upd: dict, req: dict, cur
    ) -> tuple[bool, int | None, int]:
        """The main effect of one instruction (ISA `### Instruction semantics`).

        Returns (stall, next PC override or None, FDEBUG flags to set). All
        operands are read from the start-of-cycle state. Reserved field values and
        the removed IRQ opcode are NOPs: no effect, never a stall.
        """
        op = (instr >> 13) & 7
        stall = False
        new_pc: int | None = None
        fdbg = 0
        rx_stall = 1 << eng.index  # FDEBUG.RXSTALL[e]
        tx_stall = 1 << (24 + eng.index)  # FDEBUG.TXSTALL[e]

        if op == 0:  # ------------------------------------------------ JMP
            cond = (instr >> 5) & 7
            x, y, osr_cnt = cur("x"), cur("y"), cur("osr_cnt")
            if cond == 0:
                taken = True
            elif cond == 1:
                taken = x == 0
            elif cond == 2:
                taken = x != 0
                upd["x"] = (x - 1) & M32  # always decrements
            elif cond == 3:
                taken = y == 0
            elif cond == 4:
                taken = y != 0
                upd["y"] = (y - 1) & M32
            elif cond == 5:
                taken = x != y
            elif cond == 6:
                taken = ((in_vec >> eng.jmp_pin) & 1) == 1
            else:
                taken = osr_cnt < thresh(eng.pull_thresh)
            if taken:
                new_pc = instr & 0x1F

        elif op == 1:  # ---------------------------------------------- WAIT
            pol = (instr >> 7) & 1
            source = (instr >> 5) & 3
            index = instr & 0x1F
            if source == 0:  # GPIO, absolute pin
                bit = (in_vec >> index) & 1
            elif source == 1:  # PIN, relative to IN_BASE
                bit = (in_vec >> ((eng.in_base + index) & 31)) & 1
            elif source == 2:  # IRQ: hardwired 0 (MVP)
                bit = 0
            else:
                bit = None  # reserved: NOP
            if bit is not None and bit != pol:
                stall = True

        elif op == 2:  # ------------------------------------------------- IN
            n = (instr & 0x1F) or 32
            source = (instr >> 5) & 7
            if source in (4, 5):  # reserved: NOP
                pass
            else:
                if source == 0:
                    d = rot32(in_vec, eng.in_base) & mask(n)
                elif source == 1:
                    d = cur("x") & mask(n)
                elif source == 2:
                    d = cur("y") & mask(n)
                elif source == 3:
                    d = 0
                elif source == 6:
                    d = cur("isr") & mask(n)
                else:
                    d = cur("osr") & mask(n)
                isr = cur("isr")
                if n == 32:
                    shifted = d
                elif eng.in_shiftdir:
                    shifted = ((isr >> n) | (d << (32 - n))) & M32
                else:
                    shifted = ((isr << n) | d) & M32
                cnt = min(32, cur("isr_cnt") + n)
                if eng.autopush and cnt >= thresh(eng.push_thresh):
                    if len(eng.rx) == FIFO_DEPTH:
                        stall, fdbg = True, rx_stall  # atomic: nothing else changes
                    else:
                        upd["_rx_push"] = shifted
                        upd["isr"], upd["isr_cnt"] = 0, 0
                else:
                    upd["isr"], upd["isr_cnt"] = shifted, cnt

        elif op == 3:  # ------------------------------------------------ OUT
            n = (instr & 0x1F) or 32
            dst = (instr >> 5) & 7
            osr, osr_cnt = cur("osr"), cur("osr_cnt")
            if eng.autopull and osr_cnt >= thresh(eng.pull_thresh):
                stall = True
                if len(eng.tx):
                    upd["_tx_pop"] = eng.tx[0]
                    upd["osr"], upd["osr_cnt"] = eng.tx[0], 0
                else:
                    fdbg = tx_stall
            else:
                if eng.out_shiftdir:
                    v = osr & mask(n)
                    upd["osr"] = (osr >> n) & M32
                else:
                    v = (osr >> (32 - n)) & M32 if n < 32 else osr
                    upd["osr"] = (osr << n) & M32
                cnt = min(32, osr_cnt + n)
                upd["osr_cnt"] = cnt
                if dst == 0:
                    req[("out", False)] = window(
                        eng.out_base, min(eng.out_count, 32), v
                    )
                elif dst == 1:
                    upd["x"] = v
                elif dst == 2:
                    upd["y"] = v
                elif dst == 4:
                    req[("out", True)] = window(
                        eng.out_base, min(eng.out_count, 32), v
                    )
                elif dst == 5:
                    new_pc = v & 0x1F
                elif dst == 6:
                    upd["isr"], upd["isr_cnt"] = v, n
                elif dst == 7:
                    upd["latch"], upd["latch_pend"], upd["latch_host"] = v & 0xFFFF, 1, 0
                # dst 3 (NULL) discards the value
                if eng.autopull and cnt >= thresh(eng.pull_thresh) and len(eng.tx):
                    upd["_tx_pop"] = eng.tx[0]
                    upd["osr"], upd["osr_cnt"] = eng.tx[0], 0
        else:
            stall, new_pc, fdbg = self._execute_4567(eng, instr, in_vec, upd, req, cur)
        return stall, new_pc, fdbg

    def _execute_4567(
        self, eng: Engine, instr: int, in_vec: int, upd: dict, req: dict, cur
    ) -> tuple[bool, int | None, int]:
        """PUSH, PULL, MOV, IRQ and SET (ISA `### Instruction semantics`)."""
        op = (instr >> 13) & 7
        stall = False
        new_pc: int | None = None
        fdbg = 0
        rx_stall = 1 << eng.index
        tx_stall = 1 << (24 + eng.index)

        if op == 4:
            if (instr >> 7) & 1:  # --------------------------------- PULL
                if_empty = (instr >> 6) & 1
                block = (instr >> 5) & 1
                osr_cnt = cur("osr_cnt")
                if not (eng.autopull and osr_cnt == 0):  # else: no-op barrier
                    if not (if_empty and osr_cnt < thresh(eng.pull_thresh)):
                        if len(eng.tx) == 0:
                            if block:
                                # ISA `### PULL`: Block stalls and sets TXSTALL; the
                                # non-blocking form sets no flag (it does not stall), and
                                # the regmap's FDEBUG table says TXSTALL is set when the
                                # engine stalls on an empty TX FIFO.
                                fdbg, stall = tx_stall, True
                            else:
                                upd["osr"], upd["osr_cnt"] = cur("x"), 0
                        else:
                            upd["_tx_pop"] = eng.tx[0]
                            upd["osr"], upd["osr_cnt"] = eng.tx[0], 0
            else:  # ------------------------------------------------- PUSH
                if_full = (instr >> 6) & 1
                block = (instr >> 5) & 1
                if not (if_full and cur("isr_cnt") < thresh(eng.push_thresh)):
                    if len(eng.rx) == FIFO_DEPTH:
                        fdbg = rx_stall
                        if block:
                            stall = True
                        else:  # drops the data, clears ISR
                            upd["isr"], upd["isr_cnt"] = 0, 0
                    else:
                        upd["_rx_push"] = cur("isr")
                        upd["isr"], upd["isr_cnt"] = 0, 0

        elif op == 5:  # ---------------------------------------------- MOV
            dst = (instr >> 5) & 7
            opf = (instr >> 3) & 3
            src = instr & 7
            if dst == 3 or opf == 3 or src == 4:  # reserved: NOP
                pass
            else:
                if src == 0:
                    v = rot32(in_vec, eng.in_base)
                elif src == 1:
                    v = cur("x")
                elif src == 2:
                    v = cur("y")
                elif src == 3:
                    v = 0
                elif src == 5:
                    level = len(eng.rx if eng.status_sel else eng.tx)
                    v = M32 if level < eng.status_n else 0
                elif src == 6:
                    v = cur("isr")
                else:
                    v = cur("osr")
                if opf == 1:
                    v = ~v & M32
                elif opf == 2:
                    v = brev32(v)
                if dst == 0:
                    req[("out", False)] = window(
                        eng.out_base, min(eng.out_count, 32), v
                    )
                elif dst == 1:
                    upd["x"] = v
                elif dst == 2:
                    upd["y"] = v
                elif dst == 4:
                    upd["latch"], upd["latch_pend"], upd["latch_host"] = v & 0xFFFF, 1, 0
                elif dst == 5:
                    new_pc = v & 0x1F
                elif dst == 6:
                    upd["isr"], upd["isr_cnt"] = v, 0
                else:
                    upd["osr"], upd["osr_cnt"] = v, 0

        elif op == 7:  # ----------------------------------------------- SET
            dst = (instr >> 5) & 7
            data = instr & 0x1F
            if dst == 0:
                req[("out", False)] = window(eng.set_base, eng.set_count, data)
            elif dst == 1:
                upd["x"] = data
            elif dst == 2:
                upd["y"] = data
            elif dst == 4:
                req[("out", True)] = window(eng.set_base, eng.set_count, data)

        # op == 6 is IRQ: removed in the MVP, a NOP that carries side-set and delay.
        return stall, new_pc, fdbg


def _is_exec(instr: int) -> bool:
    """True for OUT EXEC and MOV EXEC: they ignore their delay field.

    A `MOV EXEC` with a reserved Op or Source is a NOP (ISA `### Removed and reserved
    encodings`, item 2: "The instruction executes as a NOP. It has no main effect, never
    stalls, and advances the PC as a non-jump would. Side-set and delay apply as
    normal."). A NOP is not an EXEC-er: it loads no latch, and its delay field applies.
    """
    op = (instr >> 13) & 7
    dst = (instr >> 5) & 7
    if op == 3:
        return dst == 7
    if op == 5:
        if dst != 4:
            return False
        if ((instr >> 3) & 3) == 3 or (instr & 7) == 4:      # reserved Op or Source
            return False
        return True
    return False

# ------------------------------------------------------------------ helpers


def _fstat(chip: "Chip") -> int:
    """`FSTAT` (regmap `### Global registers`): TX/RX EMPTY and FULL per engine."""
    v = 0
    for e, eng in enumerate(chip.eng):
        if not eng.tx:
            v |= 1 << (24 + e)
        if len(eng.tx) == FIFO_DEPTH:
            v |= 1 << (16 + e)
        if not eng.rx:
            v |= 1 << (8 + e)
        if len(eng.rx) == FIFO_DEPTH:
            v |= 1 << e
    return v


def _flevel(chip: "Chip") -> int:
    """`FLEVEL` (regmap `**FLEVEL**`): TX0 (3:0), RX0 (7:4), TX1 (11:8), RX1 (15:12).

    Eight bits of step per engine and four between an engine's two nibbles, so a FIFO
    occupancy never lands in a neighbour's field.
    """
    v = 0
    for e, eng in enumerate(chip.eng):
        v |= (len(eng.tx) & 0xF) << (8 * e)
        v |= (len(eng.rx) & 0xF) << (8 * e + 4)
    return v
