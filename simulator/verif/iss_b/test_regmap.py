"""Register-file behaviour: one test per register and per side effect
(`docs/mvp-regmap.md` `## Registers`).
"""

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import Bus, poke, set_, execs, sm_cycle_now
from verif.iss_b.enc import JMP_ALWAYS, SET_X

def step_when(b: Bus, e: int, word: int, bus: tuple | None = None):
    """Step until engine `e` is about to execute `word`, then run that SM cycle with
    the given bus access (if any). Returns the state after it, and the read data."""
    limit = b.cycle + 200
    while not (sm_cycle_now(b, e) and b.state(e)["ir"] == word):
        assert b.cycle < limit, f"engine {e} never reached {word:#06x}"
        b.step()
    rdata = b.step(bus=bus)
    return b.state(e), rdata


# --- address map (regmap `### Address map`)
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_TXF0, A_TXF1, A_RXF0, A_RXF1 = 0x04, 0x05, 0x08, 0x09
A_PADOUT, A_PADOE, A_CFGINFO = 0x0F, 0x10, 0x11
A_INTR = 0x4A
A_IRQ0_INTE, A_IRQ0_INTF, A_IRQ0_INTS = 0x4B, 0x4C, 0x4D
A_IRQ1_INTE, A_IRQ1_INTF, A_IRQ1_INTS = 0x4E, 0x4F, 0x50


# ---------------------------------------------------------------------- CTRL


def test_ctrl_reset_and_sm_enable():
    b = Bus()
    assert b.read(A_CTRL) == 0
    b.write(A_CTRL, enc.ctrl(sm_enable=0b11))
    assert b.read(A_CTRL) == 0b11  # bits 1:0
    b.write(A_CTRL, 0xFFFFFFFF)
    assert b.read(A_CTRL) == 0b11  # reserved bits read 0


def test_ctrl_enable_takes_effect_in_the_next_cycle():
    """`CTRL.SM_ENABLE[e]` written in cycle `h` takes effect from cycle `h+1`."""
    b = Bus()
    b.load_imem(0, [set_(SET_X, 1), set_(SET_X, 2)])
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    assert not sm_cycle_now(b, 0)  # cycle h+1: the divider output en is 0
    b.idle(1)
    assert sm_cycle_now(b, 0)  # cycle h+2: the first SM cycle
    b.step()  # run it
    assert b.state(0)["x"] == 1  # IMEM[0] executed in that cycle


def test_ctrl_restart_bits_are_self_clearing():
    b = Bus()
    b.write(A_CTRL, enc.ctrl(sm_enable=1, sm_restart=1, clkdiv_restart=1))
    assert b.read(A_CTRL) == 0b01  # the SC bits read 0


def test_ctrl_sm_restart_clears_the_listed_state_and_keeps_the_rest():
    """`The FIFOs, X, Y, OSR and PC are kept`; ISR, counts, delay, latch_pend."""
    b = Bus()
    b.load_imem(0, [set_(SET_X, 1), set_(SET_X, 2)])
    b.start(0)
    poke(
        b,
        0,
        x=0x11,
        y=0x22,
        osr=0x33,
        osr_cnt=5,
        isr=0x44,
        isr_cnt=7,
        delay=3,
        stalled=1,
        latch=0x5555,
        latch_pend=1,
        pc=9,
        tx=[1, 2],
        rx=[3],
    )
    b.write(A_CTRL, enc.ctrl(sm_enable=1, sm_restart=1))
    st = b.state(0)
    assert (st["x"], st["y"], st["osr"], st["pc"], st["latch"]) == (
        0x11,
        0x22,
        0x33,
        9,
        0x5555,
    )
    assert st["tx_level"] == 2 and st["rx_level"] == 1  # MVP: FIFOs are kept
    assert (st["isr"], st["isr_cnt"], st["osr_cnt"]) == (0, 0, 32)
    assert st["delay_cnt"] == 0 and st["stalled"] == 0 and st["latch_pend"] == 0


def test_ctrl_restart_wins_over_an_execution_in_the_same_cycle():
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.load_imem(0, [set_(SET_X, 5)])
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(1)
    poke(b, 0, isr=0x99, isr_cnt=8)
    assert sm_cycle_now(b, 0)
    b.step(bus=("w", A_CTRL, enc.ctrl(sm_enable=1, sm_restart=1)))
    assert b.state(0)["x"] == 5  # the instruction ran
    assert b.state(0)["isr"] == 0  # but the restart cleared the ISR


# --------------------------------------------------------------------- FSTAT


def test_fstat_reset_and_bits():
    b = Bus()
    assert b.read(A_FSTAT) == 0x03000300  # everything empty, nothing full
    poke(b, 0, tx=[1])  # only engine 0's TX FIFO has a word
    v = b.read(A_FSTAT)
    assert v & (1 << 24) == 0  # SM0 TXEMPTY clear
    assert v & (1 << 25) == 1 << 25  # SM1 TXEMPTY set
    assert v & (1 << 8) == 1 << 8  # both RX FIFOs empty
    for w in range(4):
        poke(b, 0, tx=[1, 2, 3, 4][: w + 1])
        poke(b, 0, rx=[9, 8, 7, 6][: w + 1])
    v = b.read(A_FSTAT)
    assert v & 0x1 == 0x1  # RX0 full
    assert v & 0x100 == 0  # RX0 not empty
    assert v & (1 << 16) == 1 << 16  # TX0 full
    assert v & (1 << 24) == 0  # SM0 TX not empty
    assert v & (1 << 25) == 1 << 25  # SM1's TX FIFO is still empty


def test_fstat_is_read_only():
    b = Bus()
    before = b.read(A_FSTAT)
    b.write(A_FSTAT, 0)
    assert b.read(A_FSTAT) == before


# -------------------------------------------------------------------- FDEBUG


def test_fdebug_reset_and_w1c():
    b = Bus()
    assert b.read(A_FDEBUG) == 0
    # Fill engine 0's RX FIFO and make a PUSH drop, which sets RXSTALL.
    b.cfg(0)
    b.load_imem(0, [enc.push()])
    b.start(0)
    poke(b, 0, rx=[1, 2, 3, 4], isr=0xAA)
    execs(b, 1)
    assert b.read(A_FDEBUG) & 0x1  # RXSTALL[0]
    b.write(A_CTRL, 0)  # stop the engine so it stops re-raising the flag
    b.write(A_FDEBUG, 0x1)
    assert b.read(A_FDEBUG) & 0x1 == 0  # write 1 to clear
    b.write(A_FDEBUG, 0x1)
    assert b.read(A_FDEBUG) & 0x1 == 0  # it was already clear
    # Set it again on a fresh chip and clear it with a write naming another flag.
    c = Bus()
    c.cfg(0)
    c.load_imem(0, [enc.push(block=1)])
    c.start(0)
    poke(c, 0, rx=[1, 2, 3, 4], isr=0xBB)
    execs(c, 1)
    assert c.read(A_FDEBUG) & 0x1
    c.write(A_CTRL, 0)
    c.write(A_FDEBUG, 0x100)  # another flag
    assert c.read(A_FDEBUG) & 0x1  # RXSTALL survives


def test_fdebug_txovern_and_rxunder_n():
    b = Bus()
    for _ in range(5):
        b.push_tx(0, 1)  # the fifth push overflows
    assert b.read(A_FDEBUG) & (1 << 16)  # TXOVER[0]
    b.write(A_FDEBUG, 0x1 << 16)
    assert b.pop_rx(1) == 0  # an empty read
    assert b.read(A_FDEBUG) & (1 << 9)  # RXUNDER[1]
    for _ in range(5):
        b.push_tx(1, 1)
    assert b.read(A_FDEBUG) & (1 << 17)  # TXOVER[1]


def test_fdebug_txstall_n_and_rxstall_n():
    b = Bus()
    b.cfg(0)
    b.cfg(1)
    b.load_imem(0, [enc.pull(block=1), enc.jmp(JMP_ALWAYS, 0)])
    b.load_imem(1, [enc.pull(block=1), enc.jmp(JMP_ALWAYS, 0)])
    b.write(A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    while not (sm_cycle_now(b, 0) and sm_cycle_now(b, 1)):
        b.step()
    b.step()
    assert b.read(A_FDEBUG) & (1 << 24)  # TXSTALL[0]
    assert b.read(A_FDEBUG) & (1 << 25)  # TXSTALL[1]


def test_a_flag_set_and_cleared_in_the_same_cycle_stays_set():
    """The engine sets it, and a FDEBUG clear in that same cycle does not take it."""
    b = Bus()
    b.cfg(0)
    b.load_imem(0, [enc.pull(block=1), enc.jmp(JMP_ALWAYS, 0)])
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(1)
    while not (sm_cycle_now(b, 0) and b.state(0)["ir"] == enc.pull(block=1)):
        b.step()
    b.step(bus=("w", A_FDEBUG, 0xFFFFFFFF))
    assert b.read(A_FDEBUG) & (1 << 24)


# -------------------------------------------------------------------- FLEVEL


def test_flevel_reports_the_four_occupancies():
    """regmap `**FLEVEL**`: TX0 3:0, RX0 7:4, TX1 11:8, RX1 15:12 — four bits per field."""
    b = Bus()
    assert b.read(A_FLEVEL) == 0
    poke(b, 0, tx=[1, 2, 3], rx=[4, 5])
    poke(b, 1, tx=[9], rx=[8, 9, 10, 11])
    assert b.read(A_FLEVEL) == (3 << 0) | (2 << 4) | (1 << 8) | (4 << 12)
    assert b.read(A_FLEVEL) >> 16 == 0  # bits 31:16 reserved


def test_flevel_is_read_only():
    b = Bus()
    poke(b, 0, tx=[1, 2])
    before = b.read(A_FLEVEL)
    b.write(A_FLEVEL, 0)
    assert b.read(A_FLEVEL) == before


def test_flevel_nibbles_are_four_bits_wide():
    """Regression (iss-diff, W3): the four fields are nibbles in the regmap's order.

    `FLEVEL` once packed its fields three bits apart (`FSTAT`'s step), which overlapped
    engine 1's occupancies into engine 0's nibble.
    """
    b = Bus()
    poke(b, 0, tx=[1, 2, 3, 4], rx=[5, 6])              # TX0 = 4, RX0 = 2
    poke(b, 1, tx=[7, 8, 9], rx=[10, 11, 12, 13])       # TX1 = 3, RX1 = 4
    v = b.read(A_FLEVEL)
    assert [(v >> 4 * i) & 0xF for i in range(4)] == [4, 2, 3, 4]


# ----------------------------------------------------------------- TXF / RXF


def test_txf_pushes_and_rxf_pops_in_order():
    b = Bus()
    b.push_tx(0, 0xA0)
    b.push_tx(0, 0xB0)
    assert b.state(0)["tx_level"] == 2
    assert b.pop_rx(0) == 0  # the wrong FIFO
    b.cfg(0)
    b.load_imem(0, [enc.pull()])
    b.start(0)
    assert execs(b, 1)[0]["osr"] == 0xA0  # FIFO order
    assert b.state(0)["tx_level"] == 1


def test_txf_is_write_only_and_rxf_is_read_only():
    b = Bus()
    assert b.read(A_TXF0) == 0  # WO reads 0
    b.push_tx(0, 0x1234)
    b.write(A_RXF0, 0x5678)  # ignored
    assert b.read(A_TXF0) == 0
    assert b.state(0)["tx_level"] == 1  # the write to RXF did nothing


def test_txf_drop_on_a_full_fifo_sets_txover():
    b = Bus()
    for i in range(4):
        b.push_tx(0, i)
    assert b.state(0)["tx_level"] == 4
    b.push_tx(0, 99)
    assert b.state(0)["tx_level"] == 4  # dropped
    assert b.read(A_FDEBUG) & (1 << 16)
    b.cfg(0)
    b.load_imem(0, [enc.pull()])
    b.start(0)
    execs(b, 1)  # pops one, so the FIFO has room again
    b.write(A_CTRL, 0)  # stop the engine so it does not pop again
    b.write(A_FDEBUG, 0x1 << 16)
    assert b.read(A_FDEBUG) & (1 << 16) == 0
    b.push_tx(0, 99)  # now accepted
    assert b.state(0)["tx_level"] == 4


def test_a_host_push_to_a_full_fifo_is_dropped_even_if_the_engine_pops():
    """`a host push to a full FIFO is dropped (TXOVER) even if the engine pops`."""
    b = Bus()
    b.cfg(0)
    b.load_imem(0, [enc.pull()])
    b.start(0)
    poke(b, 0, tx=[1, 2, 3, 4])
    b.write(A_FDEBUG, 0)
    # Run the cycle in which the engine pops, with the host pushing at the same time.
    before = b.state(0)["tx_level"]
    step_when(b, 0, enc.pull(), ("w", A_TXF0, 0x77))
    assert b.state(0)["tx_level"] == before - 1  # one popped, the host push dropped
    assert b.read(A_FDEBUG) & (1 << 16)  # TXOVER[0]
    # Without the host push in that cycle, the same thing happens minus the drop.
    poke(b, 0, tx=[1, 2, 3, 4])
    b.write(A_FDEBUG, 1 << 16)
    before = b.state(0)["tx_level"]
    step_when(b, 0, enc.pull())
    assert b.read(A_FDEBUG) & (1 << 16) == 0  # no TXOVER this time
    assert b.state(0)["tx_level"] == before - 1


def test_an_engine_cannot_pop_a_word_the_host_pushes_in_the_same_cycle():
    b = Bus()
    b.cfg(0)
    b.load_imem(0, [enc.pull(), enc.jmp(JMP_ALWAYS, 0)])
    b.start(0)
    step_when(b, 0, enc.pull(), ("w", A_TXF0, 0x77))
    assert b.state(0)["osr"] == 0  # the pull found the FIFO empty
    # `enc.pull()` is the non-blocking form, which sets no flag (ISA `### PULL`).
    assert b.read(A_FDEBUG) & (1 << 24) == 0
    assert b.state(0)["tx_level"] == 1  # the word is there now
    poke(b, 0, pc=0)
    assert execs(b, 1)[0]["osr"] == 0x77  # and it pops on the next SM cycle


def test_rxf_of_an_empty_fifo_returns_zero_and_pops_nothing():
    b = Bus()
    assert b.pop_rx(0) == 0
    assert b.read(A_FDEBUG) & (1 << 8)  # RXUNDER[0]
    assert b.state(0)["rx_level"] == 0


def test_a_host_read_of_an_empty_rx_fifo_returns_zero_even_if_the_engine_pushes():
    """`a host read of an empty RX FIFO returns 0 (RXUNDER) even if the engine
    pushes in that cycle`."""
    b = Bus()
    b.cfg(0)
    b.load_imem(0, [enc.push()])
    b.start(0)
    poke(b, 0, isr=0xAA)
    while not (sm_cycle_now(b, 0) and b.state(0)["ir"] == enc.push()):
        b.step()
    assert b.read(A_RXF0) == 0  # the read happened at the start of the cycle
    assert b.read(A_FDEBUG) & (1 << 8)  # RXUNDER[0]
    assert b.state(0)["rx_level"] == 1  # and the push still happened


def test_a_push_and_a_pop_in_the_same_cycle_both_happen():
    b = Bus()
    b.cfg(0)
    b.load_imem(0, [enc.push(), enc.jmp(JMP_ALWAYS, 0)])
    b.start(0)
    poke(b, 0, isr=0xBB)
    # The read happens in the same cycle as the push, so it sees an empty FIFO, while
    # the push still happens at the end of that cycle.
    state, rdata = step_when(b, 0, enc.push(), ("r", A_RXF0))
    assert rdata == 0
    assert b.read(A_FDEBUG) & (1 << 8)  # RXUNDER[0]
    assert state["rx_level"] == 1  # the engine's push still happened
    assert b.pop_rx(0) == 0xBB


# ---------------------------------------------------------- DBG_PADOUT / PADOE


def test_dbg_padout_and_padoe_reflect_the_pin_registers():
    b = Bus()
    b.cfg(0, pinctrl_v=enc.pinctrl(set_base=0, set_count=8))
    b.load_imem(0, [set_(enc.SET_PINS, 0xF5), set_(enc.SET_PINDIRS, 0xAA)])
    b.start(0)
    execs(b, 1)
    assert b.read(A_PADOUT) == b.chip.pin_out
    execs(b, 1)
    assert b.read(A_PADOE) == b.chip.pin_oe
    b.write(A_PADOUT, 0)  # RO
    assert b.read(A_PADOUT) == b.chip.pin_out


# ------------------------------------------------------------- DBG_CFGINFO


def test_dbg_cfginfo_fields():
    v = Bus().read(A_CFGINFO)
    assert v == 0x00200204
    assert (v >> 16) & 0x3F == 32  # IMEM_SIZE
    assert (v >> 8) & 0xF == 2  # SM_COUNT
    assert v & 0x3F == 4  # FIFO_DEPTH


# ----------------------------------------------------------------- INSTR_MEM


def test_instr_mem_is_engine_local_and_write_only():
    b = Bus()
    b.load_imem(0, {0: set_(SET_X, 5), 31: set_(SET_X, 6)})
    b.load_imem(1, {0: set_(enc.SET_Y, 9), 31: set_(enc.SET_Y, 10)})
    assert b.read(enc.a_imem(0, 0)) == 0  # WO reads 0
    assert b.read(enc.a_imem(1, 0)) == 0
    # The two engines' IMEM are independent: each runs its own words.
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.cfg(1, clkdiv_v=enc.clkdiv(1, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    limit = b.cycle + 20
    while not (sm_cycle_now(b, 0) and sm_cycle_now(b, 1)):
        assert b.cycle < limit
        b.step()
    b.step()
    assert b.state(0)["x"] == 5 and b.state(1)["y"] == 9
    assert b.chip.eng[0].imem[0] == set_(SET_X, 5)
    assert b.chip.eng[1].imem[0] == set_(enc.SET_Y, 9)
    # Jump each engine to word 31 with a forced JMP, which is how the host sets the PC.
    b.force(0, enc.jmp(enc.JMP_ALWAYS, 31))
    b.force(1, enc.jmp(enc.JMP_ALWAYS, 31))
    state0, _ = step_when(b, 0, set_(SET_X, 6))
    state1, _ = step_when(b, 1, set_(enc.SET_Y, 10))
    assert state0["x"] == 6 and state1["y"] == 10


def test_instr_mem_ignores_bits_31_16():
    b = Bus()
    b.write(enc.a_imem(0, 5), 0xFFFFA042)
    b.force(0, 0x0000)
    b.cfg(0)
    b.load_imem(0, {5: enc.set_(SET_X, 7)})
    b.start(0)
    poke(b, 0, pc=5)
    assert execs(b, 1)[0]["x"] == 7


def test_imem_write_reaches_execution_two_cycles_later():
    """`T_IMEM = 2`: a host IMEM write in cycle `h` is first executed in cycle `h+2`."""
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.write(enc.a_imem(0, 0), 0x0000)  # `jmp 0`: a self-loop
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(1)
    assert b.state(0)["ir"] == 0x0000  # IR fetched in this cycle
    h = b.cycle
    b.write(enc.a_imem(0, 0), set_(SET_X, 0x0B))
    # The word is in IMEM from cycle h+1, is fetched into IR at the end of cycle h+1,
    # and is executed in cycle h+2.
    assert b.state(0)["ir"] == 0x0000  # IR during cycle h still holds the old word
    b.idle(1)
    assert b.state(0)["ir"] == set_(SET_X, 0x0B)  # IR during cycle h+1
    b.idle(1)
    assert b.state(0)["x"] == 0x0B  # executed in cycle h+2


def test_sm_instr_reads_ir():
    b = Bus()
    b.load_imem(0, {3: 0xA042})
    poke(b, 0, pc=3)
    b.cfg(0)
    b.start(0)
    b.idle(1)
    assert b.read(enc.a_sm(0, 4)) == 0xA042  # IR = IMEM[PC]
    b.write(enc.a_sm(0, 4), 0xFFFFFFFF)
    assert b.read(enc.a_sm(0, 4)) & 0xFFFF0000 == 0  # bits 31:16 read 0


# ---------------------------------------------------- per-engine config regs


def test_sm_clkdiv_reads_back_and_ignores_bits_7_0():
    b = Bus()
    assert b.read(enc.a_sm(0, 0)) == 0x00010000  # reset: 1.0
    b.write(enc.a_sm(0, 0), enc.clkdiv(3, 0x40) | 0xFF)
    assert b.read(enc.a_sm(0, 0)) == enc.clkdiv(3, 0x40)
    b.write(enc.a_sm(1, 0), enc.clkdiv(2, 0))
    assert b.read(enc.a_sm(1, 0)) == enc.clkdiv(2, 0)


def test_sm_execctrl_reads_back_its_fields_and_the_exec_stalled_bit():
    b = Bus()
    assert b.read(enc.a_sm(0, 1)) == 0x0001F000  # reset: wrap 31:0
    v = enc.execctrl(
        side_en=1, side_pindir=1, jmp_pin=17, wrap_top=9, wrap_bottom=4,
        status_sel=1, status_n=7,
    )
    b.write(enc.a_sm(0, 1), v | 0x00FC0000)  # the n/i bits OUT_EN_SEL/OUT_STICKY
    got = b.read(enc.a_sm(0, 1))
    assert got == v  # so the read-back matches, and they read 0
    assert got & (1 << 31) == 0  # EXEC_STALLED is 0
    poke(b, 0, latch_pend=1, latch_host=1)
    assert b.read(enc.a_sm(0, 1)) & (1 << 31)


def test_sm_shiftctrl_reads_back_and_ignores_bits_31_16():
    b = Bus()
    assert b.read(enc.a_sm(0, 2)) == 0x000C0000  # reset: both shifts right
    v = enc.shiftctrl(
        pull_thresh=5, push_thresh=6, out_shiftdir=0, in_shiftdir=1, autopull=1, autopush=1
    )
    b.write(enc.a_sm(0, 2), v | 0xC000FFFF)  # FJOIN_RX/TX and the reserved 15:0
    got = b.read(enc.a_sm(0, 2))
    assert got == v  # they are n/i, so they read 0


def test_sm_addr_is_read_only_and_reads_the_pc():
    b = Bus()
    assert b.read(enc.a_sm(0, 3)) == 0
    poke(b, 0, pc=27)
    assert b.read(enc.a_sm(0, 3)) == 27
    poke(b, 0, pc=0)
    b.write(enc.a_sm(0, 3), 5)
    assert b.read(enc.a_sm(0, 3)) == 0  # RO: writes are ignored


def test_sm_pinctrl_reads_back_and_ignores_nothing():
    b = Bus()
    assert b.read(enc.a_sm(0, 5)) == 0x14000000  # reset: SET_COUNT 5
    v = enc.pinctrl(
        sideset_count=3, set_count=4, out_count=20, in_base=2,
        sideset_base=6, set_base=7, out_base=1,
    )
    b.write(enc.a_sm(0, 5), v)
    assert b.read(enc.a_sm(0, 5)) == v


def test_config_writes_apply_from_the_next_cycle():
    """`Configuration writes in cycle h apply from cycle h+1, even while the engine
    runs.`"""
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.load_imem(0, [set_(enc.SET_PINS, 1), enc.jmp(JMP_ALWAYS, 0)])
    b.start(0)
    new_pinctrl = enc.pinctrl(set_base=1, set_count=1)
    # The SET in the write's cycle used the old SET_BASE (0), so pin 0 was written.
    step_when(b, 0, set_(enc.SET_PINS, 1), ("w", enc.a_sm(0, 5), new_pinctrl))
    assert b.chip.lvl & 0b11 == 0b01  # pin 0, the old SET_BASE
    # The next SM cycle uses the new one, so pin 1 gets it and pin 0 keeps its value.
    step_when(b, 0, set_(enc.SET_PINS, 1))
    assert b.chip.lvl & 0b11 == 0b11


# ------------------------------------------------------------- INTR / IRQx


def test_intr_reset_and_bits():
    b = Bus()
    # Reset: both TX FIFOs have room (5:4), both RX FIFOs are empty (1:0).
    assert b.read(A_INTR) == 0x30
    poke(b, 0, tx=[1, 2, 3, 4])
    assert b.read(A_INTR) & (1 << 4) == 0  # SM0_TXNFULL
    assert b.read(A_INTR) & (1 << 5) == 1 << 5  # SM1_TXNFULL
    poke(b, 0, rx=[1])
    assert b.read(A_INTR) & 0x3 == 0b01  # SM0_RXNEMPTY


def test_intr_is_read_only_and_its_irq_bits_read_zero():
    b = Bus()
    before = b.read(A_INTR)
    b.write(A_INTR, 0xFFFFFFFF)
    assert b.read(A_INTR) == before
    assert b.read(A_INTR) & 0x300 == 0  # bits 9:8 are always 0 (no IRQ in the MVP)


def test_irq_inte_reset_values_are_the_mvp_ones():
    b = Bus()
    assert b.read(A_IRQ0_INTE) == 0x003
    assert b.read(A_IRQ1_INTE) == 0x010
    assert b.read(A_IRQ0_INTF) == 0
    assert b.read(A_IRQ1_INTF) == 0
    assert b.read(A_IRQ0_INTS) == 0  # INTR & INTE = 0x30 & 0x003 = 0
    assert b.read(A_IRQ1_INTS) == 0x10  # 0x30 & 0x010


def test_irqx_intf_forces_the_line_and_ints_is_the_or():
    b = Bus()
    b.write(A_IRQ0_INTF, 0x2)  # bits 1:0 only
    assert b.read(A_IRQ0_INTF) == 0x2
    assert b.read(A_IRQ0_INTS) & 0x2  # INTF appears in INTS
    b.write(A_IRQ1_INTE, 0x33)
    assert b.read(A_IRQ1_INTE) == 0x33  # bits 5:4 and 1:0
    b.write(A_IRQ0_INTE, 0xFFFFFFFF)
    assert b.read(A_IRQ0_INTE) == 0x33  # the other bits read 0


def test_ints_clears_when_inte_and_intf_are_cleared():
    b = Bus()
    b.write(A_IRQ1_INTE, 0x33)
    b.write(A_IRQ1_INTF, 0x1)
    assert b.read(A_IRQ1_INTS) & 0x1
    b.write(A_IRQ1_INTF, 0x1)  # write 1 to clear (it is RW, so a plain write)
    b.write(A_IRQ1_INTF, 0)
    assert b.read(A_IRQ1_INTS) & 0x1 == 0
    b.write(A_IRQ1_INTE, 0)
    assert b.read(A_IRQ1_INTS) == 0


def test_ints_is_read_only():
    b = Bus()
    b.write(A_IRQ0_INTF, 0x1)
    before = b.read(A_IRQ0_INTS)
    b.write(A_IRQ0_INTS, 0)
    assert b.read(A_IRQ0_INTS) == before


# ----------------------------------------------------- irq0 / irq1 pin lines


def test_irq0_is_rx_valid_and_irq1_is_tx_ready_by_default():
    """MVP resets: `IRQ0_INTE = 0x003` so rx_valid means "some RX FIFO has data",
    `IRQ1_INTE = 0x010` so tx_ready means "engine 0's TX FIFO has room"."""
    b = Bus()
    b.idle(2)  # the pins are registered once from the state of the previous cycle
    assert (b.chip.irq0, b.chip.irq1) == (0, 1)
    poke(b, 0, rx=[1])
    b.idle(2)
    assert b.chip.irq0 == 1
    poke(b, 0, rx=[])
    b.idle(2)
    assert b.chip.irq0 == 0


def test_the_fifos_show_on_the_pins_two_cycles_later():
    """`a FIFO change made at the end of cycle n shows on the pin from cycle n+2`."""
    b = Bus()
    b.idle(2)
    assert b.chip.irq1 == 1 and b.chip.irq0 == 0  # engine 0's TX FIFO has room
    for w in (1, 2, 3):
        b.push_tx(0, w)
    h = b.cycle - 1  # the write cycle that filled the FIFO
    b.push_tx(0, 4)  # the FIFO is full from cycle h + 1 onward
    assert b.chip.irq1 == 1  # the pin value of cycle h + 1: still the old state
    b.step()
    assert b.chip.irq1 == 0  # cycle h + 2: the pin follows the FIFO


def test_the_host_can_remap_the_two_lines():
    b = Bus()
    b.write(A_IRQ0_INTE, 0x30)  # rx_valid = both TX FIFOs have room
    b.write(A_IRQ1_INTE, 0x3)  # tx_ready = both RX FIFOs have data
    poke(b, 0, rx=[1], tx=[1, 2, 3, 4])
    b.idle(2)
    assert b.chip.irq0 == 1  # engine 0's TX FIFO is full
    assert b.chip.irq1 == 1  # engine 0's RX FIFO has data
    poke(b, 0, rx=[], tx=[])
    poke(b, 1, rx=[1], tx=[1, 2, 3, 4])
    b.idle(2)
    assert b.chip.irq0 == 1  # engine 1's TX FIFO is the full one
    assert b.chip.irq1 == 1


# ----------------------------------------------------------- reserved space


@pytest.mark.parametrize(
    "addr", [0x06, 0x07, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x3E, 0x3F, 0x49, 0x51, 0x72, 0x7F]
)
def test_reserved_addresses_read_zero_and_ignore_writes(addr):
    b = Bus()
    assert b.read(addr) == 0
    b.write(addr, 0xFFFFFFFF)
    assert b.read(addr) == 0


def test_unmapped_word_addresses_read_zero():
    b = Bus()
    for addr in (0x60, 0x61, 0x100 & 0x7F):
        assert b.read(addr) == 0


def test_the_address_bus_is_seven_bits():
    b = Bus()
    b.write(0x84, 0xFFFFFFFF)  # 0x84 & 0x7F = 0x04, TXF0
    assert b.state(0)["tx_level"] == 1
    # 0x81 & 0x7F = 0x01, FSTAT: everything empty except engine 0's TX FIFO.
    assert b.read(0x81) == 0x02000300
    # The same register is reachable by any address that differs only above bit 6.
    assert b.read(0x101) == b.read(0x01)