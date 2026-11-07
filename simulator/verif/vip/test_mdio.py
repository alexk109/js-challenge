"""Self-tests for the pin-level MDIO/MDC reference models: `make vip-selftest VIP=mdio`.

Everything runs at 50 MHz (20 ns per tick). A station and one or two PHYs talk to
each other while a passive checker watches the pins. The first half is legal
traffic: the models must agree and the checker must stay quiet. The second half
breaks one rule at a time and the checker must say which one, with the tick.

The rule names the checker uses are in `CHECKER_RULES`, and
`test_checker_rule_vocabulary` fails if this file ever exercises a rule the
checker does not implement, so renaming a rule cannot silently switch a test off.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.mdio import (
    ANAR,
    BMSR,
    BMCR,
    BMCR_ANEG_EN,
    BMCR_DUPLEX,
    BMCR_ISOLATE,
    BMCR_LOOPBACK,
    BMCR_POWERDOWN,
    BMCR_RESET,
    BMCR_SPEED100,
    BMSR_ANEG_CAP,
    BMSR_EXT_CAPS,
    BMSR_LINK,
    PHYIDR1,
    FRAME_BITS,
    OP_READ,
    OP_WRITE,
    POWER_ON_DEFAULTS,
    PREAMBLE_BITS,
    ST_BITS,
    ST_END,
    OP_END,
    REGAD_END,
    MdioChecker,
    MdioPhy,
    MdioStation,
    attach_checker,
    attach_phy,
    attach_station,
    bit_role,
    encode_frame,
    mdio_minimums,
    ns_to_ticks,
)

NS_PER_TICK = 20.0                    # 50 MHz chip clock

#: Every rule MdioChecker can report. The framing tests below each add to this
#: set, and `test_checker_rule_vocabulary` checks the checker knows them all.
CHECKER_RULES = {
    "MDC",         # MDC period shorter than 400 ns, i.e. faster than 2.5 MHz
    "MDC-half",    # MDC low or high time shorter than 160 ns
    "tSU",         # station moved MDIO too close before an MDC rising edge
    "tH",          # station moved MDIO too soon after an MDC rising edge
    "tPHY",        # PHY read data missed its 300 ns deadline
    "preamble",    # fewer than 32 ones before ST
    "ST",          # start code was not 01
    "OP",          # opcode was neither 10 (read) nor 01 (write)
    "TA1",         # the first turnaround bit was not 1
    "TA2",         # the second turnaround bit was not 0
    "frame",       # MDC stopped clocking before 64 bit times were out
    "release",     # MDIO left low with the bus idle
    "idle",        # MDC parked low with no bit times being clocked
    "contention",  # two devices drove one net with opposite values
}

SPEED_TICKS = [20, 40, 100]           # 400 ns, 800 ns, 2000 ns bit times


class PatchedStation(MdioStation):
    """A station whose outgoing frame bits are patched, to break one rule.

    `patch` maps a bit index to the bits to send there, so `{32: [1, 1]}` breaks
    the start code and `{47: 1}` breaks the second turnaround bit of a write. A
    patched station is still a legal driver otherwise: its timing is untouched,
    which keeps the test that breaks a framing rule from also breaking a timing
    rule by accident.
    """

    def __init__(self, patch=None, **kwargs):
        super().__init__(**kwargs)
        self.patch = patch or {}

    def _frame_bits(self, op):
        bits = super()._frame_bits(op)
        for index, values in self.patch.items():
            if isinstance(values, int):
                values = [values]
            bits[index:index + len(values)] = list(values)
        return bits


class SilentPhy(MdioPhy):
    """A PHY that never releases MDIO after a read, which is the `release` bug."""

    def _drive_for(self, t, index):
        if index >= FRAME_BITS and index < FRAME_BITS + 8:
            return 0            # keep holding the bus low past the last bit
        return super()._drive_for(t, index)


class MdcParkedStation(MdioStation):
    """A station that leaves MDC low after its frame, which is the `idle` bug."""

    def __init__(self, patch=None, **kwargs):
        super().__init__(**kwargs)

    def tick(self, t, pins):
        if self.mdio_bit > self.frame_bits and not self.queue:
            # The frame is out and there is nothing queued: park MDC low. A new
            # queued access releases it again, so only the tail is broken.
            return {self.mdc_net: 0, self.mdio_net: None}
        return super().tick(t, pins)


def rig(phyads=(0,), station_kw=None, phy_kw=None, chk_kw=None,
        station_cls=MdioStation, patch=None, check=True, ns_per_tick=NS_PER_TICK):
    """A bus with a station, one PHY per address in `phyads`, and a checker.

    `station_kw`/`phy_kw` go to the station/the PHYs, `chk_kw` to the checker,
    and `station_cls` picks a subclass (with `patch` handed to it) so a test can
    break one rule without touching the timing knobs.
    """
    bus = Bus(ns_per_tick=ns_per_tick)
    if station_cls is MdioStation:
        station = attach_station(bus, **(station_kw or {}))
    else:
        _ensure_nets(bus)
        station = station_cls(bus=bus, patch=patch, **(station_kw or {}))
        bus.attach(station)
    phys = [attach_phy(bus, phyad=addr, **(phy_kw or {})) for addr in phyads]
    checker = (attach_checker(bus, expect_phyad=set(phyads), **(chk_kw or {}))
               if check else None)
    return bus, station, phys, checker


def _ensure_nets(bus: Bus) -> None:
    """The same two nets `attach_*` would add, for a station subclass."""
    for net in ("mdc", "mdio"):
        if net not in bus.nets:
            bus.add_net(net, push_pull=True)


def settle(station, bus, limit=2_000_000):
    """Run the bus until the station is idle, then let the bus go quiet.

    The extra ticks matter: the checker only calls a frame finished (or
    abandoned) and the bus idle once MDC has stayed put for its own idea of a
    quiet window, which is three spec bit times. Too few ticks and a test would
    miss a `frame`, `release` or `idle` violation that is there.
    """
    assert station.run(bus, limit=limit), "the station never finished its queue"
    bus.run(4 * station.period_ticks + 4 * station.min["period_ticks"] + 8)
    return station


def rules(checker):
    return {v.rule for v in checker.violations}

# ---- legal traffic: the models agree and the checker stays quiet -------------
def test_power_on_register_file():
    """A fresh PHY answers with the datasheet's power-on register values."""
    bus, station, (phy,), chk = rig()
    assert len(phy.regs) == 32 and all(isinstance(r, int) for r in phy.regs)
    for reg, value in POWER_ON_DEFAULTS.items():
        assert phy.regs[reg] == value, (reg, hex(phy.regs[reg]))
    # BMSR is the one register a read does not return verbatim: bit 2 is the
    # link, which is a model input rather than a stored bit.
    assert phy.read_reg(BMSR) == POWER_ON_DEFAULTS[BMSR] | BMSR_LINK
    for reg in range(32):
        assert station.read(bus, reg) == phy.read_reg(reg)
    settle(station, bus)
    assert chk.clean, chk.report()


def test_all_thirty_two_registers_are_readable_and_writable():
    """Every one of the 32 words round-trips through the wire."""
    bus, station, (phy,), chk = rig()
    for reg in range(32):
        value = 0x5A00 ^ (reg << 7) ^ 0x0031
        assert station.write(bus, reg, value) == value
    settle(station, bus)
    assert chk.clean, chk.report()
    for reg in range(32):
        if reg == BMSR:
            continue        # every BMSR bit is read-only, so the write did nothing
        assert phy.read_reg(reg) == (0x5A00 ^ (reg << 7) ^ 0x0031), reg
    assert phy.read_reg(BMSR) == POWER_ON_DEFAULTS[BMSR] | BMSR_LINK
    assert len(chk.frames) == 32 and all(f.is_write for f in chk.frames)


def test_control_and_status_round_trip():
    """BMCR, BMSR and the auto-negotiation register behave as the datasheet says."""
    bus, station, (phy,), chk = rig()
    value = (BMCR_SPEED100 | BMCR_DUPLEX | BMCR_ANEG_EN | BMCR_LOOPBACK
             | BMCR_ISOLATE | BMCR_POWERDOWN)
    station.write(bus, BMCR, value)
    assert phy.read_reg(BMCR) == value
    assert station.read(bus, BMCR) == value

    bmsr = station.read(bus, BMSR)
    assert bmsr & BMSR_LINK, "the link is up, so BMSR bit 2 must be set"
    assert bmsr & BMSR_ANEG_CAP and bmsr & BMSR_EXT_CAPS

    station.write(bus, ANAR, 0x01FD)                # 802.3 selector plus 100/10
    assert station.read(bus, ANAR) == 0x01FD
    settle(station, bus)
    assert chk.clean, chk.report()
    assert [(f.is_write, f.reg) for f in chk.frames] == [
        (True, BMCR), (False, BMCR), (False, BMSR), (True, ANAR), (False, ANAR)]


def test_link_status_latches_until_read():
    """BMSR bit 2 is RO/LL: a falling edge latches it low until BMSR is read."""
    bus, station, (phy,), chk = rig()
    assert station.read(bus, BMSR) & BMSR_LINK
    phy.link = False
    phy.link = True                          # back up without polling in between
    assert station.read(bus, BMSR) & BMSR_LINK == 0, "the outage latched it low"
    assert station.read(bus, BMSR) & BMSR_LINK, "reading BMSR released the latch"
    settle(station, bus)
    assert chk.clean, chk.report()


def test_soft_reset_restores_the_register_file():
    """Writing BMCR bit 15 puts every register back to its power-on value."""
    bus, station, (phy,), chk = rig()
    station.write(bus, BMCR, 0x0000)
    station.write(bus, ANAR, 0x0000)
    assert phy.read_reg(BMCR) == 0 and phy.read_reg(ANAR) == 0
    station.write(bus, BMCR, BMCR_RESET)
    assert phy.resets == 1
    bus.run(phy.init_ticks + 2)              # out of the post-reset window
    assert phy.read_reg(BMCR) == POWER_ON_DEFAULTS[BMCR], "bit 15 is self-clearing"
    assert phy.read_reg(ANAR) == POWER_ON_DEFAULTS[ANAR]
    assert station.read(bus, ANAR) == POWER_ON_DEFAULTS[ANAR]


def test_reset_silent_window_then_answer():
    """A read inside the post-reset window comes back as ones, and then answers.

    The DP83848 asks for 3 us of silence after a reset (`ti-dp83848-smc-timing.txt`),
    so the model ignores accesses until `init_ticks` have passed and the station
    sees 0xFFFF. The checker, which cannot know the PHY is busy, flags the missing
    turnaround, which is the point: the station has to wait and try again.
    """
    bus, station, (phy,), chk = rig(phy_kw={"init_ticks": 150})
    station.write(bus, BMCR, BMCR_RESET)
    op = station.queue_read(BMSR)
    settle(station, bus)
    assert op.unanswered and op.value == 0xFFFF
    assert "TA2" in rules(chk), chk.report()
    assert any("post-reset window" in e for e in phy.errors)

    bus.run(200)                       # 200 ticks = 4 us, past the window
    assert station.read(bus, BMSR) == POWER_ON_DEFAULTS[BMSR] | BMSR_LINK
    # Only the read inside the window is on the checker's list, and nothing else.
    assert rules(chk) == {"TA2"}, chk.report()


def test_two_phys_answer_only_their_own_address():
    """Two PHYs on one bus: each answers its PHYAD and leaves the other alone."""
    bus, station, (phy0, phy7), chk = rig(phyads=(0, 7))
    station.write(bus, BMCR, 0x2100, phyad=0)
    station.write(bus, BMCR, 0x0040, phyad=7)
    assert phy0.read_reg(BMCR) == 0x2100
    assert phy7.read_reg(BMCR) == 0x0040
    assert station.read(bus, BMCR, phyad=0) == 0x2100
    assert station.read(bus, BMCR, phyad=7) == 0x0040
    settle(station, bus)
    assert chk.clean, chk.report()
    assert [f.phyad for f in chk.frames] == [0, 7, 0, 7]


def test_address_scan_finds_the_phys():
    """Scanning all 32 PHYADs answers 0xFFFF everywhere except the real PHYs."""
    bus, station, (phy0, phy3), chk = rig(phyads=(0, 3), check=False)
    found = [addr for addr in range(32)
             if station.read(bus, BMSR, phyad=addr) != 0xFFFF]
    assert found == [0, 3]
    # With the checker attached it sees 30 reads that no PHY answered, which is
    # the clause 22 "nobody there" answer and not a violation.
    bus2, station2, _, chk2 = rig(phyads=(0, 3))
    for addr in range(32):
        station2.read(bus2, BMSR, phyad=addr)
    settle(station2, bus2)
    assert chk2.clean, chk2.report()
    assert len(chk2.frames) == 32
    assert sum(1 for f in chk2.frames if f.unanswered) == 30


@pytest.mark.parametrize("period_ticks", SPEED_TICKS)
def test_legal_at_three_speeds(period_ticks):
    """A frame is legal at 2.5 MHz and at any slower rate the spec allows."""
    bus, station, (phy,), chk = rig(station_kw={"period_ticks": period_ticks})
    assert station.max_mdc_hz == pytest.approx(1e9 / (NS_PER_TICK * period_ticks))
    station.write(bus, BMCR, BMCR_SPEED100)
    assert station.read(bus, BMSR) & BMSR_LINK
    settle(station, bus)
    assert chk.clean, chk.report()
    assert chk.max_mdc_hz == pytest.approx(station.max_mdc_hz)
    assert chk.shortest_period == period_ticks
    assert set(chk.mdc_periods) == {period_ticks}


def test_back_to_back_frames_need_no_preamble():
    """Frames with no dead time in front of them may skip the 32 ones."""
    bus, station, (phy,), chk = rig()
    for reg in range(4):
        station.queue_read(reg)
    settle(station, bus)
    assert chk.clean, chk.report()
    assert len(chk.frames) == 4
    # The queue drained without ever leaving MDC high, so the checker waived the
    # preamble: `preamble_required` is False for every frame after the first.
    assert [f.preamble_bits for f in chk.frames][0] == PREAMBLE_BITS
    assert chk.frames[0].start is not None


def test_frames_after_an_idle_gap_need_the_preamble():
    """One idle tick between frames is enough to make the 32 ones mandatory."""
    bus, station, (phy,), chk = rig()
    station.queue_read(BMSR)
    station.read(bus, BMSR)          # the second frame waits for the bus to idle
    settle(station, bus)
    assert chk.clean, chk.report()
    assert len(chk.frames) == 2
    assert chk.frames[1].preamble_bits == PREAMBLE_BITS
    assert chk.idle_ticks > 0


def test_bus_returns_to_idle_after_traffic():
    """After the queue drains, both nets are back at their pull-up value."""
    bus, station, (phy,), chk = rig()
    station.read(bus, BMSR)
    settle(station, bus)
    bus.run(50)
    assert bus.values == {"mdc": 1, "mdio": 1}, bus.values
    assert chk.clean, chk.report()
    assert not station.busy and not station.queue


# ---- broken traffic: every rule has a case that breaks exactly it ----------
def _read_bmsr(bus, station):
    return station.read(bus, BMSR)


def _write_bmcr(bus, station):
    return station.write(bus, BMCR, BMCR_SPEED100)


def _break_mdc():
    """A station clocking MDC at 3.125 MHz: the period is 320 ns, not 400."""
    bus, station, phys, chk = rig(station_kw={"period_ticks": 16})
    return bus, station, phys, chk, _read_bmsr


def _break_mdc_half():
    """A 40 ns MDC low phase: the spec floor is 160 ns either side of an edge."""
    bus, station, phys, chk = rig(station_kw={"low_ticks": 2, "high_ticks": 18})
    return bus, station, phys, chk, _read_bmsr


def _break_tsu_checker():
    """A station that leaves 200 ns of setup against a 300 ns limit."""
    bus, station, phys, chk = rig(chk_kw={"timing": {"t_setup": 300}})
    return bus, station, phys, chk, _read_bmsr


def _break_th_checker():
    """A station that holds MDIO for 200 ns against a 400 ns limit."""
    bus, station, phys, chk = rig(chk_kw={"timing": {"t_hold": 400}})
    return bus, station, phys, chk, _read_bmsr


def _break_tphy_checker():
    """A PHY asked for read data on the spot, with no deadline at all."""
    bus, station, phys, chk = rig(chk_kw={"timing": {"t_phy_max": 0}})
    return bus, station, phys, chk, _read_bmsr


def _break_tphy_phy():
    """A PHY whose read data turns up 340 ns after the edge: 300 ns is the limit.

    Sixteen ticks is past the deadline and still inside the 400 ns bit time, so
    the read comes back right while the timing is broken: the checker has to be
    the thing that notices. A delay longer than a whole bit time would shift the
    data instead, which is a different (and unobservable) failure.
    """
    bus, station, phys, chk = rig(phy_kw={"drive_delay_ticks": 16})
    return bus, station, phys, chk, _read_bmsr


def _break_preamble():
    """One 0 in the middle of the 32 ones: 31 is not 32."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={5: 0})
    return bus, station, phys, chk, _read_bmsr


def _break_short_preamble():
    """Only 8 ones of preamble, which also shifts everything after it."""
    bus, station, phys, chk = rig(station_kw={"preamble_bits": 8})
    return bus, station, phys, chk, _read_bmsr


def _break_st():
    """A start code of 11 instead of 01."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={32: [1, 1]})
    return bus, station, phys, chk, _read_bmsr


def _break_op():
    """An opcode of 00, which is neither the read nor the write code."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={34: [0, 0]})
    return bus, station, phys, chk, _read_bmsr


def _break_ta1_read():
    """A station that drives 0 into the first turnaround bit of a read."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={46: [0]})
    return bus, station, phys, chk, _read_bmsr


def _break_ta1_write():
    """A write whose turnaround is 00 instead of 10."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={46: [0]})
    return bus, station, phys, chk, _write_bmcr


def _break_ta2_write():
    """A write whose turnaround is 11: the second bit must be a driven 0."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={47: [1]})
    return bus, station, phys, chk, _write_bmcr


def _break_frame():
    """A station that stops clocking after 20 of the frame's 64 bit times."""
    bus, station, phys, chk = rig(station_kw={"frame_bits": 20})
    return bus, station, phys, chk, _read_bmsr


def _break_release():
    """A PHY that never lets go of MDIO after the last data bit."""
    bus, station, (phy,), chk = rig()
    phy.__class__ = SilentPhy
    return bus, station, (phy,), chk, _read_bmsr


def _break_idle():
    """A station that leaves MDC low after its frame instead of parking it high."""
    bus, station, phys, chk = rig(station_cls=MdcParkedStation)
    return bus, station, phys, chk, _read_bmsr


def _break_contention():
    """A station that drives 1 into the data field while the PHY drives 0."""
    bus, station, phys, chk = rig(station_cls=PatchedStation, patch={48: [1], 49: [1]})
    return bus, station, phys, chk, _read_bmsr


def _break_tsu_device():
    """A station that moves MDIO on the very tick the edge samples it."""
    bus, station, phys, chk = rig(station_kw={"setup_ticks": 0})
    return bus, station, phys, chk, _read_bmsr


def _break_th_device():
    """A station that moves MDIO on the tick the previous edge sampled it."""
    bus, station, phys, chk = rig(station_kw={"setup_ticks": 20})
    return bus, station, phys, chk, _read_bmsr


def _break_ta2_read():
    """A read of a PHY that is inside its post-reset window: no turnaround."""
    bus, station, phys, chk = rig(phy_kw={"init_ticks": 150})

    def work(bus, station):
        station.write(bus, BMCR, BMCR_RESET)
        return station.read(bus, BMSR)
    return bus, station, phys, chk, work


#: Every checker rule, and the one case that breaks it. `test_every_rule_has_a_
#: breaking_case` walks this, so a rule that stops firing fails the suite.
BREAKERS = {
    "MDC": _break_mdc,
    "MDC-half": _break_mdc_half,
    "tSU": _break_tsu_checker,
    "tH": _break_th_checker,
    "tPHY": _break_tphy_phy,
    "preamble": _break_preamble,
    "ST": _break_st,
    "OP": _break_op,
    "TA1": _break_ta1_read,
    "TA2": _break_ta2_write,
    "frame": _break_frame,
    "release": _break_release,
    "idle": _break_idle,
    "contention": _break_contention,
}


@pytest.mark.parametrize("rule", sorted(BREAKERS))
def test_every_rule_has_a_breaking_case(rule):
    """Each rule is broken on purpose and the checker must name it, with a tick."""
    bus, station, phys, chk, work = BREAKERS[rule]()
    work(bus, station)
    settle(station, bus)
    assert rule in rules(chk), f"{rule} not flagged: {chk.report()}"
    flagged = chk.violations_by_rule(rule)
    assert flagged and all(v.tick >= 0 for v in flagged), flagged
    assert all(v.detail or v.rule for v in flagged)


def test_checker_rule_vocabulary_is_complete():
    """Every rule this file breaks is a rule the checker knows, and vice versa."""
    assert set(BREAKERS) == CHECKER_RULES, (set(BREAKERS) ^ CHECKER_RULES)
    seen = set()
    for rule in sorted(BREAKERS):
        bus, station, phys, chk, work = BREAKERS[rule]()
        work(bus, station)
        settle(station, bus)
        seen |= rules(chk)
    assert CHECKER_RULES <= seen, CHECKER_RULES - seen


@pytest.mark.parametrize("breaker,expected", [
    # A short preamble costs the monitor everything: the frame is 8 bit times
    # shorter than 64, so the ST and OP it finds are the wrong fields and the
    # frame looks truncated as well.
    (_break_short_preamble, {"preamble", "ST", "OP", "frame"}),
    (_break_tphy_checker, {"tPHY"}),
    (_break_tsu_device, {"tSU", "tH"}),
    (_break_ta1_write, {"TA1"}),
    (_break_ta2_read, {"TA2"}),
    (_break_mdc_half, {"MDC-half"}),
    (_break_frame, {"frame"}),
    (_break_release, {"release"}),
    (_break_idle, {"idle"}),
])
def test_a_broken_case_reports_what_it_breaks(breaker, expected):
    """Some cases are worth pinning down exactly: these rules and nothing else."""
    bus, station, phys, chk, work = breaker()
    work(bus, station)
    settle(station, bus)
    assert rules(chk) == expected, chk.report()


def test_a_station_with_no_hold_also_loses_its_own_frame():
    """Zero hold is not just a timing error: it shifts the bits the PHY receives.

    At 20 ns/tick the only way to break the 10 ns hold minimum is to move MDIO on
    the very tick the previous edge sampled it, and then the station reads back
    the *next* bit's value, so the ST and OP the checker decodes are garbage. That
    is what real hardware does with no hold at all, and it is why a test that only
    wants `tH` points the checker at a longer limit instead (`_break_th_checker`).
    """
    bus, station, phys, chk, work = _break_th_device()
    work(bus, station)
    settle(station, bus)
    assert {"tH", "tSU"} <= rules(chk), chk.report()
    assert "ST" in rules(chk) and "OP" in rules(chk), chk.report()
    assert station.errors == [], station.errors


# ---- the checker is not vacuous --------------------------------------------
def test_checker_is_passive():
    """The checker drives nothing, so the models behave the same without it."""
    with_chk = rig()
    without = rig(check=False)
    for bus, station, phys, chk in (with_chk, without):
        station.write(bus, BMCR, 0x2100)
        values = [station.read(bus, BMSR), station.read(bus, PHYIDR1),
                  station.read(bus, ANAR)]
        settle(station, bus)
        assert values == [0x780D, 0x0007, 0x01E1]
        assert phys[0].read_reg(BMCR) == 0x2100
    assert with_chk[3].frames, "the checker saw no frames at all"
    assert with_chk[3].mdc_periods, "the checker measured no MDC periods"
    assert with_chk[3].shortest_period == 20


def test_checker_decodes_the_same_frames_the_station_sent():
    """What the checker decoded matches what the station put on the wire."""
    bus, station, (phy,), chk = rig()
    ops = [station.queue_write(BMCR, 0x2100), station.queue_read(BMSR),
           station.queue_write(ANAR, 0x0125), station.queue_read(PHYIDR1)]
    settle(station, bus)
    assert len(chk.frames) == len(ops) == 4
    for frame, op in zip(chk.frames, ops):
        assert frame.is_write == op.is_write
        assert frame.reg == op.reg
        if op.is_write:
            assert frame.data == op.value
        else:
            assert frame.data == op.value and op.value != 0xFFFF
        assert frame.st_bits == ST_BITS
        assert frame.op_bits == (OP_WRITE if op.is_write else OP_READ)
        assert frame.preamble_bits == PREAMBLE_BITS
        assert frame.ta == (0b10 if op.is_write else 0b10), hex(frame.ta)
        assert len(frame.mdc_periods) == FRAME_BITS - 1


def test_checker_tolerates_a_long_idle_gap():
    """The spec has no maximum time between MDC edges, so idling is not an error."""
    bus, station, (phy,), chk = rig()
    station.read(bus, BMSR)
    bus.run(20_000)                       # 400 us of nothing at all
    assert chk.idle_ticks > 20_000, "the checker saw the bus go quiet"
    station.read(bus, BMSR)
    settle(station, bus)
    assert chk.clean, chk.report()
    assert len(chk.frames) == 2


def test_checker_mirrors_the_core_contention_records():
    """Contention shows up both in `Bus.errors` and in the checker's violations."""
    bus, station, phys, chk, work = _break_contention()
    work(bus, station)
    settle(station, bus)
    assert bus.errors, "the core saw no contention either"
    assert all(e.net == "mdio" for e in bus.errors)
    assert chk.violations_by_rule("contention")


def test_reads_with_no_phy_come_back_as_ones():
    """A read to an empty address is the pull-up reading 1, not a violation."""
    bus, station, (), chk = rig(phyads=())
    assert station.read(bus, BMSR) == 0xFFFF
    settle(station, bus)
    assert chk.clean, chk.report()
    assert chk.frames[-1].unanswered


# ---- the model's own arithmetic --------------------------------------------
def test_frame_layout_is_clause_22():
    """64 bit times: 32 ones, ST=01, OP, 5 + 5 address bits, TA, 16 data bits."""
    write = encode_frame(True, phyad=5, reg=17, data=0xBEEF)
    read = encode_frame(False, phyad=5, reg=17)
    assert len(write) == FRAME_BITS == 64
    assert write[:PREAMBLE_BITS] == [1] * 32
    assert write[ST_END - 1:ST_END + 1] == [0, 1], "ST is 01"
    assert write[OP_END - 1:OP_END + 1] == [0, 1], "a write opcode is 01"
    assert write[OP_END + 1:OP_END + 6] == [0, 0, 1, 0, 1], "PHYAD 5, MSB first"
    assert write[REGAD_END - 4:REGAD_END + 1] == [1, 0, 0, 0, 1], "REGAD 17"
    assert write[46:48] == [1, 0], "a write turnaround is <10>"
    assert write[48:] == [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1], "0xBEEF"
    assert read[34:36] == [1, 0], "a read opcode is 10"
    assert read[46:] == [None] * 18, "a read releases the turnaround and the data"
    for index in range(FRAME_BITS):
        assert bit_role(index) in ("preamble", "st", "op", "phyad", "regad",
                                   "ta1", "ta2", "data")


def test_timing_conversion_and_ceiling():
    """Every limit rounds up to whole ticks, and the ceiling is 2.5 MHz."""
    min_ = mdio_minimums(20.0)
    assert min_["t_period"] == 20 and min_["period_ticks"] == 20
    assert min_["t_low"] == min_["t_high"] == 8, "160 ns is 8 ticks"
    assert min_["t_setup"] == min_["t_hold"] == 1, "10 ns rounds up to one tick"
    assert min_["t_phy_max"] == 15, "300 ns"
    assert min_["t_phy_min"] == 0
    assert min_["max_mdc_hz"] == 2.5e6
    assert ns_to_ticks(20.0, 0) == 0 and ns_to_ticks(20.0, 21) == 2
    slower = mdio_minimums(20.0, {"t_period": 1000})
    assert slower["period_ticks"] == 50 and slower["max_mdc_hz"] == 1e6


def test_bad_timing_is_rejected():
    with pytest.raises(ValueError, match="unknown MDIO timing"):
        mdio_minimums(20.0, {"t_nonsense": 10})
    with pytest.raises(ValueError, match="must not be negative"):
        mdio_minimums(20.0, {"t_setup": -1})
    with pytest.raises(ValueError, match="t_period must be positive"):
        mdio_minimums(20.0, {"t_period": 0})


def test_out_of_range_fields_are_rejected():
    for kwargs in (dict(phyad=32), dict(phyad=-1)):
        with pytest.raises(ValueError, match="phyad"):
            encode_frame(True, kwargs["phyad"], 0)
    with pytest.raises(ValueError, match="reg "):
        encode_frame(True, 0, 32)
    with pytest.raises(ValueError, match="data"):
        encode_frame(True, 0, 0, 0x10000)
    with pytest.raises(ValueError, match="PHYAD"):
        MdioPhy(phyad=32)
    with pytest.raises(ValueError, match="outside the 32-word file"):
        MdioPhy(regs={32: 0})


def test_station_run_gives_up_on_a_short_limit():
    """`run` says so instead of hanging when the limit is too small."""
    bus, station, (phy,), chk = rig()
    station.queue_read(BMSR)
    assert station.run(bus, limit=50) is False
    assert station.busy or station.queue, "the access is still in flight"
    assert station.run(bus) is True


def test_station_reports_a_second_clock_driver():
    """Two things make the station say so: a foreign clock edge, and a parked MDC.

    Only one station per bus, so a device that pulses MDC behind the station's
    back makes its rises arrive off schedule, and a device that holds MDC low
    while the station has nothing queued means the same thing more quietly. Both
    are caught rather than silently producing a garbled frame.
    """
    class OtherClock(Device):
        """Pulses MDC out of phase with the station."""

        name = "other_clock"

        def __init__(self):
            self.phase = 0

        def tick(self, t, pins):
            self.phase ^= 1
            return {"mdc": 1 if self.phase else 0}

    class ParkedClock(Device):
        """Holds MDC low forever."""

        name = "parked_clock"

        def tick(self, t, pins):
            return {"mdc": 0}

    bus, station, (phy,), chk = rig(station_kw={"frame_bits": 8})
    bus.attach(OtherClock())
    station.queue_read(BMSR)
    station.run(bus)
    assert any("off the station's schedule" in e for e in station.errors), \
        station.errors[:3]

    bus, station, (phy,), chk = rig()
    bus.attach(ParkedClock())
    bus.run(100)
    assert any("nothing queued" in e for e in station.errors), station.errors[:3]
