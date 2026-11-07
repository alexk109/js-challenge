"""End-to-end test of the MVP Wiegand firmware (task B1-FW-wiegand).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes, the host driver loads
program.pio and sends a card, and two models from `verif/vip/wiegand.py` judge the wire --
`WiegandReceiver`, a passive door-controller decoder that reads the facility code, the ID number
and both parity bits back off the pins, and `WiegandChecker`, an analyser that measures every
pulse and pause against the SIA window.  Neither model is modified and neither is softened: a
violation fails the test.  The models look at the bus through the chip's own pin nets (`p0`,
`p1` or wherever a configuration puts them), so there is no coupling device in between and no
tick of latency that is not in the design, which is what fw/i2c/test_i2c.py does for the same
reason.

    make fw PROTO=wiegand TARGET=iss        # runs

The chip plays the **reader**, the side that initiates: it drives 26 low pulses on two
open-drain lines and then goes quiet, which is the only frame boundary a receiver can see.  The
model that *sends* cards, `WiegandReader`, is therefore this firmware's counterpart rather than
its judge, and the tests use the decoder and the analyser instead.

Three configurations, all at the chip's own 20 ns tick and all judged against the real SIA
window (pulse width 20..100 us, pulse interval 200 us .. 20 ms, quoted by three independent
documents under research/ref/protocols/wiegand/):

    floor      20 us pulse   the fastest legal frame the program can put on the wire
    reader     50 us pulse   BALTECH's shipped PulseWidth default
    ceiling   100 us pulse   the widest pulse the window allows, on engine 1 and pins 5/6

Every number the checker reports comes from the checker: nothing here decides pass or fail by
looking inside the ISS.  What the ISS is asked for is only the things a host can see --
FDEBUG, DBG_PADOE, the PC and the FIFO levels -- and those are checked for cleanliness, not for
correctness of the protocol.

The cost of this protocol in Python is the frame *gap*: a receiver closes a frame when the lines
have been quiet for longer than a pulse interval may be, which at 20 ns per tick is 1 000 000
ticks.  Every full-frame test therefore runs ~1.3 to 2.6 million ticks, so the long runs are
module-scoped fixtures and the assertions read their results rather than repeating them.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

import pytest

from fw import targets
from fw.wiegand import driver as driver_mod
from fw.wiegand.driver import (
    CORE_HZ,
    READER_DEFAULTS_NS,
    TICK_NS,
    WIEGAND_WINDOW_NS,
    Config,
    WiegandDriver,
    bits_from_pairs,
    ceil_ticks,
    clkdiv_for_pulse,
    divider_value,
    direction_pairs,
    frame_bits,
    frame_pairs,
    frame_words,
    layout,
    load_program,
    phase_cycles,
    ticks_per_sm_cycle,
    words_used,
)
from iss import pio_iss
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.wiegand import (
    LEGAL_DEFAULTS_NS,
    WiegandChecker,
    WiegandReceiver,
    decode_card,
    encode_card,
    spec_minimums,
)
from fw import regmap as _regmap

# One full frame is 1.3 to 2.6 million core cycles, and every one of them goes through the RTL
# harness's pipes on a slow target (fw/rtl_harness.py, `## The cost is one pipe round trip`), so
# on `rtl` a frame is minutes.  The tests below that need a whole frame therefore ask for a
# *model* target -- `fw/targets.py`'s `is_fast()`, which is speed, not identity: the second ISS
# passes too. The ones that need no bus run on every target.
LONG = pytest.mark.skipif(
    not targets.is_fast(),
    reason=f"FW_TARGET={targets.current()}: a full Wiegand frame is over a million core cycles, "
           f"which is minutes of cocotb; a Python model target is the reference for this "
           f"protocol's timing",
)

IMEM_SIZE = 32                       # docs/mvp-isa.md `### Engine state`
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_BAD = 0x0003_0303             # TXOVER, RXUNDER, RXSTALL, TXSTALL: nothing at all
UIO_MASK = 0x000FF                   # the bidirectional protocol pins
CARD = (123, 4567)                   # the credential the full-frame cases present
HAND_WORKED_FRAME = "10111101100010001110101110"   # Pyramid's Table 2, worked out in
# verif/vip/test_wiegand.py `test_the_frame_bits_are_the_bit_map_in_the_reference`: FC 123 =
# 0b01111011 into bits 2..9, ID 4567 = 0b0001000110100111 into bits 10..25.


@dataclass(frozen=True)
class Case:
    """One end-to-end configuration: a pulse width, an engine and a pin pair."""

    name: str
    pw_ns: float
    engine: int = 0
    d0_pin: int = 0


CASES = [
    Case("floor-20us", 20_000.0),
    Case("reader-50us", READER_DEFAULTS_NS["baltech_pw"]),
    Case("ceiling-100us-engine1", READER_DEFAULTS_NS["pyramid_pw"], engine=1, d0_pin=5),
]


class Bench:
    """A bus with the chip on it, the two Wiegand models watching, and the host driver.

    The models are constructed rather than attached through `verif/vip/wiegand.py`'s
    `attach_checker()` / `attach_receiver()` helpers because those add a "d0"/"d1" net of their
    own when the bus does not have one, and this bench points the models at the chip's own pin
    nets instead (p0, p1 or p5, p6).  Every device therefore sees the same two nets: there is no
    coupling device, no extra tick of latency, and no net in the bus that nobody drives.
    """

    def __init__(self, case: Case, program=None, clkdiv: int | None = None):
        self.case = case
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(pio_iss.NUM_PINS):
            self.bus.add_net(f"p{k}")                    # p0..p17, open drain, pulled up
        self.chip = pio_iss.Chip()
        self.device = self.bus.attach(ChipDevice(self.chip))
        self.driver = WiegandDriver(self.device, self.bus, engine=case.engine,
                                    d0_pin=case.d0_pin, pw_ns=case.pw_ns, program=program)
        if clkdiv is not None:
            self.driver.clkdiv = clkdiv          # a deliberate fault, see test_broken_...
        nets = {"d0_net": f"p{self.driver.d0_pin}", "d1_net": f"p{self.driver.d1_pin}"}
        self.receiver = self.bus.attach(WiegandReceiver(bus=self.bus, **nets))
        self.checker = self.bus.attach(WiegandChecker(bus=self.bus, **nets))

    def start(self, facility=CARD[0], number=CARD[1]) -> "Bench":
        """Load the program, enable the engine, send one card, and let the frame close."""
        self.driver.load()
        self.driver.start()
        self.driver.send_card(facility, number)
        self.driver.run()
        return self

    # ---- what the models saw ----
    def rules(self) -> set:
        return {v.rule for v in self.checker.violations}

    def card(self):
        """The credential the *decoder* read off the pins."""
        return self.receiver.frames[-1].card if self.receiver.frames else None

# --------------------------------------------------------------------------------------------
# The transaction, judged by the models.  One run per configuration, shared by the assertions.
# --------------------------------------------------------------------------------------------

@pytest.fixture(scope="module", params=CASES, ids=[c.name for c in CASES])
def bench(request) -> Bench:
    if not targets.is_fast():
        pytest.skip("a full Wiegand frame is over a million core cycles; see LONG above")
    return Bench(request.param).start()


@LONG
def test_the_card_reaches_the_panel(bench: Bench) -> None:
    """One card in, the same facility code and ID number out, and a clean wire.

    This is the round trip that matters: 26 pulses on two open-drain lines, decoded by a model
    this test did not write.  Both models have to agree with the credential the host was asked to
    send, the decoder must find the frame legal (`WiegandReceiver.clean`), and the analyser must
    have flagged nothing at all -- every timing rule, the frame length and both parity bits
    (research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md for the window,
    02_pyramid_26bit_format.md for the layout).
    """
    chk, rcv = bench.checker, bench.receiver
    assert chk.clean, chk.report()
    assert rcv.clean, rcv.report()
    assert len(chk.pulses) == bench.driver.pairs == 26, "one pulse per frame bit"
    assert len(chk.frames) == 1 and len(chk.frames[0].bits) == 26, chk.report()
    assert len(rcv.frames) == 1 and len(rcv.frames[0].bits) == 26, rcv.report()

    # The decoder's answer, and the analyser's, both the credential the host asked for.
    for who, card in (("decoder", rcv.frames[0].card), ("analyser", chk.frames[0].card)):
        assert (card.facility, card.number) == CARD, f"the {who} read {card}"
        assert card.parity_ok, f"the {who} read a card with the wrong parity: {card}"
        assert card.lead_ok and card.trail_ok

    # The wire carries the bits the layout table says, first bit first, and never both lines at
    # once: the pulses are strictly sequential by construction (one `out pindirs, 2` per bit, and
    # a pair never has two bits set), so a D0D1 violation would mean the program drove two pins.
    wire = "".join(str(b) for b in chk.frames[0].bits)
    assert wire == HAND_WORKED_FRAME and [int(b) for b in wire] == frame_bits(*CARD)
    assert {p.bit for p in chk.pulses} == {0, 1}, "both lines really carried bits"
    assert not bench.checker.both_low and rcv.collisions == 0


@LONG
def test_the_wire_timing_is_what_the_divider_asked_for(bench: Bench) -> None:
    """The measured pulse and pause are the ones the program and CLKDIV describe, in ticks.

    The prediction is built out of the assembled words (`phase_cycles`) and the divider word, not
    out of the program text, so this fails if the two files ever disagree.  The window it is
    checked against is the *model's* table, converted to ticks at this tick rate, so the two
    independent transcriptions of the SIA numbers meet here.
    """
    chk, drv = bench.checker, bench.driver
    window = spec_minimums(TICK_NS)
    assert (chk.min_pw, chk.max_pw) == (drv.pulse_ticks, drv.pulse_ticks), (
        f"the analyser measured {chk.min_pw}-{chk.max_pw} ticks of pulse, the program asks for "
        f"{drv.pulse_ticks:g}"
    )
    assert chk.min_pi == drv.pause_ticks, (
        f"the shortest pause on the wire was {chk.min_pi} ticks, the program asks for "
        f"{drv.pause_ticks:g}"
    )
    # Both ends of every window, from the model's own conversion of its nanosecond table.
    assert window["pw_min"] <= chk.min_pw <= chk.max_pw <= window["pw_max"]
    assert window["pi_min"] <= chk.min_pi <= window["pi_max"]
    # Every pause in the frame is the same, not just the shortest: one instruction per phase.
    pauses = [b.fall - a.rise for a, b in zip(chk.pulses, chk.pulses[1:])]
    assert set(pauses) == {drv.pause_ticks}, sorted(set(pauses))
    widths = [p.width for p in chk.pulses]
    assert set(widths) == {drv.pulse_ticks}, sorted(set(widths))
    # The first pulse is the only one with no pause in front of it, so the analyser has no
    # tPI;MIN to measure there -- and the frame is closed by the quiet time after the last one.
    assert chk.frames[0].pulses[0].index == 0 and len(chk.frames[0].pulses) == 26


@LONG
def test_the_chip_leaves_the_lines_released_and_the_registers_clean(bench: Bench) -> None:
    """Both lines released, nothing stalled, nothing dropped, only this engine used, IMEM exact.

    Everything here is something a host can read over the SPI port of `docs/mvp-regmap.md`, and
    all of it is what the reader has to leave behind after a frame: an idle bus is the only thing
    the far end can wait for.
    """
    drv, chip, case = bench.driver, bench.chip, bench.case
    d0, d1 = drv.d0_pin, drv.d1_pin
    other = 1 - case.engine

    padoe, padout = drv.padoe(), drv.padout()
    assert not padoe & ((1 << d0) | (1 << d1)), (
        f"DBG_PADOE {padoe:#06x}: D{d0}/D{d1} are still driven, so the bus is not idle"
    )
    assert not padoe & UIO_MASK & ~((1 << d0) | (1 << d1)), (
        f"DBG_PADOE {padoe:#06x}: a program window leaked onto another uio pin"
    )
    assert not padout & ((1 << d0) | (1 << d1)), (
        f"DBG_PADOUT {padout:#06x}: an open-drain line must keep its level register at 0"
    )
    # Not one flag: the words were queued before the jump, so the engine never parked on
    # `pull block` with an empty TX FIFO (FDEBUG.TXSTALL) and nothing was ever dropped.
    assert drv.fdebug() == 0, f"FDEBUG {drv.fdebug():#010x} & {FDEBUG_BAD:#010x}"
    assert drv.tx_empty, "the frame's TX words were never consumed"
    assert drv.idle and drv.pc() in (drv.idle_addr, drv.idle_addr + 1), (
        f"the engine is at PC {drv.pc()}, not parked in `idle`"
    )

    imem = chip.engines[case.engine].imem
    assert imem[: drv.words_used] == drv.program.instrs, "the words in IMEM are not the program's"
    assert set(imem[drv.words_used:]) <= {0}, "the driver wrote past the program"
    assert chip.state(other)["pc"] == 0 and not chip.engines[other].tx, "the other engine ran"
    assert not bench.bus.errors, f"contention on a net: {bench.bus.errors[:2]}"
    assert drv.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"
    assert bench.bus.values[f"p{d0}"] == bench.bus.values[f"p{d1}"] == 1, (
        "both nets are pulled up: an open-drain line idles high"
    )


@LONG
def test_an_idle_reader_puts_nothing_on_the_wire() -> None:
    """An engine that has been sent nothing says nothing: no edges, no frames, no violations.

    The program parks in `idle` with both lines released and only leaves it when the host forces
    `jmp send`, so the bus has to stay silent across a whole frame time.  One frame time is
    enough to prove it: every rule the checker has needs an edge, an edge needs a pulse, and a
    pulse needs the loop -- and if the engine ever fell out of `idle` into `send`, the blocking
    `pull block` would park it on FDEBUG.TXSTALL with an empty TX FIFO, which is asserted too.
    This is the other half of the checker's usefulness: a reader that chats on an idle bus would
    break every panel's framing, and a checker that flagged *this* would be unusable.
    """
    idle = Bench(CASES[0])
    idle.driver.load()
    idle.driver.start()
    idle.driver.run(idle.driver.frame_span_ticks)

    assert idle.checker.clean and idle.checker.frames == [] and idle.checker.pulses == []
    assert idle.receiver.frames == [] and idle.receiver.bits == []
    assert idle.driver.fdebug() == 0, "the engine left `idle` and parked on `pull block`"
    assert idle.driver.idle, f"PC {idle.driver.pc()} is not in `idle`"
    assert idle.bus.values[f"p{idle.driver.d0_pin}"] == 1


@LONG
def test_two_cards_a_frame_gap_apart_are_two_frames() -> None:
    """Two cards in a row: two frames, both decoded, both legal, and the gap between them.

    This is the property a panel actually depends on.  A frame has no marker: the other end only
    knows where one ended because the lines went quiet for longer than any pause may be
    (research/ref/protocols/wiegand/05_accessgrid_26bit_decode.md), so a reader that returned to
    the wires sooner would put one 52-bit frame on the wire instead of two 26-bit ones.  The
    driver therefore waits `frame_gap_ticks` -- one tick more than the pulse interval maximum --
    after a frame before it starts the next, and this measures that the two frames really are
    separate and in order.
    """
    bench = Bench(CASES[0])
    drv = bench.driver
    drv.load()
    drv.start()
    first = drv.send_card(1, 2)
    drv.run()                                  # long enough for the first frame to close
    quiet_before = len(bench.checker.frames)    # the first frame is closed by now
    second = drv.send_card(200, 40000)
    # Nothing has gone quiet yet, so the second frame cannot have closed: a frame boundary is a
    # quiet time, not the end of a bit.
    drv.run(drv.bit_ticks * 2)
    assert len(bench.checker.frames) == quiet_before == 1, (
        "the second frame closed before the lines went quiet"
    )
    drv.run()                                  # and now the quiet time closes it

    chk, rcv = bench.checker, bench.receiver
    assert chk.clean, chk.report()
    assert rcv.clean, rcv.report()
    assert len(chk.frames) == 2 and len(chk.pulses) == 52
    assert [(f.card.facility, f.card.number) for f in rcv.frames] == [(1, 2), (200, 40000)]
    assert [f.card.number for f in chk.frames] == [2, 40000], "the second card did not arrive"
    # The quiet time between them is at least one pulse interval maximum, and the gap the driver
    # waited is longer than the frame itself took, so this is not a near miss.
    assert chk.frames[1].start - chk.frames[0].end >= chk.min["pi_max"], (
        "the frames are not separated by a frame gap"
    )
    assert second - first >= drv.run_ticks, (
        "the driver did not wait a whole frame plus a frame gap before the second card"
    )
    assert drv.idle and drv.fdebug() == 0, "the reader did not park cleanly after the second card"


# --------------------------------------------------------------------------------------------
# What the program and the driver are, derived from the words rather than asserted by hand
# --------------------------------------------------------------------------------------------

def test_program_fits_the_imem() -> None:
    """program.pio assembles, and the driver reads its shape back out of the words."""
    program = load_program()
    assert program.name == "wiegand"
    assert words_used(program) == 8, [f"{w:#06x}" for w in program.instrs]
    assert words_used(program) <= IMEM_SIZE, "the IMEM holds 32 words"
    assert program.sideset_bits == 0, "the program must carry no side-set: see program.pio"
    assert program.labels["idle"] == 0, "the engine comes up idle, with nothing to send"
    assert program.labels["loop"] == 4 and program.labels["send"] == 2
    assert program.wrap == words_used(program) - 1, "WRAP_TOP must be the last instruction"
    assert program.wrap_target == 0, "running off the end must land back in the idle preamble"
    # The two phase lengths the divider is derived from, out of the words themselves.
    assert phase_cycles(program) == (1, 10), "a pulse is 1 SM cycle, the pause 10"
    assert frame_pairs(program) == 26, "`set y, 25` runs the loop 26 times"
    assert frame_pairs(program) == layout("sia26").bits, "one direction pair per frame bit"


def test_the_two_transcriptions_of_the_timing_window_agree() -> None:
    """The firmware's window and the checker's window are the same four numbers.

    They are two independent transcriptions of the same cited sources -- this module's from
    research/ref/protocols/wiegand/, the model's in verif/vip/wiegand.py -- and the only place
    they meet is here.  If either drifted, every other assertion in this file would still pass
    while the two disagreed about what legal is, so this is checked first and in both
    directions: the nanoseconds, and the reader defaults each cites from a vendor document.
    """
    assert WIEGAND_WINDOW_NS == {k: v for k, v in spec_minimums(TICK_NS)["ns"].items()}
    assert spec_minimums(TICK_NS)["ns"] == {
        "pw_min": 20_000, "pw_max": 100_000, "pi_min": 200_000, "pi_max": 20_000_000,
    }, "the SIA AC-01 window, in nanoseconds"
    assert (READER_DEFAULTS_NS["baltech_pw"], READER_DEFAULTS_NS["baltech_pi"]) == \
        LEGAL_DEFAULTS_NS, "BALTECH's shipped PulseWidth/PulseInterval defaults"
    window = WIEGAND_WINDOW_NS
    assert window["pw_min"] <= READER_DEFAULTS_NS["baltech_pw"] <= window["pw_max"]
    assert window["pi_min"] <= READER_DEFAULTS_NS["baltech_pi"] <= window["pi_max"]


def test_the_layout_is_the_bit_map_in_the_sources() -> None:
    """The 26-bit frame, against Pyramid's Table 2 and against the model's own encoder.

    Two independent things are checked.  First the frame itself: FC 123 and ID 4567 worked out
    by hand from the table, in the order the sources count positions -- bit 1 first on the wire,
    even parity over bits 2..13, the facility code MSB first in bits 2..9, the ID number MSB
    first in bits 10..25, odd parity over bits 14..25 in bit 26.
    Second, that the reference model -- written separately
    from the same sources -- agrees on every credential tried, which is what makes the
    decoder in the end-to-end tests an independent second implementation rather than a mirror.
    """
    facility, number = CARD
    bits = frame_bits(facility, number)
    assert "".join(str(b) for b in bits) == HAND_WORKED_FRAME
    assert bits[1:9] == [int(c) for c in f"{facility:08b}"]
    assert bits[9:25] == [int(c) for c in f"{number:016b}"]
    assert sum(bits[0:13]) % 2 == 0, "bit 1 is even parity over bits 2..13"
    assert sum(bits[13:26]) % 2 == 1, "bit 26 is odd parity over bits 14..25"

    for cred in ((0, 0), (255, 65535), (1, 1), (128, 32768), (123, 4567)):
        assert frame_bits(*cred) == encode_card("sia26", *cred), cred
        assert bits_from_pairs(direction_pairs(frame_bits(*cred))) == frame_bits(*cred)
        card = decode_card("sia26", frame_bits(*cred))
        assert card is not None and card.parity_ok and (card.facility, card.number) == cred

    # A single flipped bit anywhere in the frame breaks one of the two parity bits, which is what
    # makes the analyser's parity rules worth having on the wire.
    for index in range(26):
        broken = list(bits)
        broken[index] ^= 1
        assert not decode_card("sia26", broken).parity_ok, f"flipping bit {index} went unnoticed"


def test_direction_pairs_encode_the_two_lines_as_open_drain() -> None:
    """A data 0 is 0b01 on (D0, D1) and a data 1 is 0b10: one line low, never both.

    `out pindirs, 2` writes OSR[0] to D0 and OSR[1] to D1, and on this chip a direction of 1 is
    what pulls the line low (docs/mvp-isa.md `### Open drain`).  So a pair must never have both
    bits set -- that would put both lines low at once, which the specification forbids outright
    -- and the level registers stay at 0, so nothing ever drives a hard 1 against the pull-up.
    """
    for cred in ((0, 0), (255, 65535), CARD):
        pairs = direction_pairs(frame_bits(*cred))
        assert len(pairs) == 26
        assert set(pairs) <= {1, 2} and all(pair in (0b01, 0b10) for pair in pairs)
        assert bits_from_pairs(pairs) == frame_bits(*cred)
    assert direction_pairs([0]) == [1] and direction_pairs([1]) == [2]
    for bad in ([2], [-1], [0, 3]):                # a data bit is 0 or 1
        with pytest.raises(ValueError):
            direction_pairs(bad)
    for bad in ([0], [3], [-1]):                  # a direction pair is 0b01 or 0b10
        with pytest.raises(ValueError):
            bits_from_pairs(bad)
    # 26 pairs is 52 OSR bits: the first word carries 16 pairs, the second 10.
    words = frame_words(frame_bits(*CARD))
    assert len(words) == 2 and words[0] >> 32 == 0
    assert words[0] == sum(direction_pairs(frame_bits(*CARD))[i] << (2 * i) for i in range(16))
    assert words[1] == sum(p << (2 * i) for i, p in
                           enumerate(direction_pairs(frame_bits(*CARD))[16:]))


def test_every_speed_the_program_can_express_is_inside_the_window() -> None:
    """The whole legal pulse range, checked against the model's own tick limits.

    One divider serves both phases and the delay field is five bits, so a frame's shape is
    fixed by the program: a pulse of `n_pw` SM cycles, then a pause of `n_pi + 1` (the `jmp`
    costs one).  What the host chooses is the divider, and therefore the speed.  This walks the
    whole range a 20 ns tick can express -- the pulse grid is 20 ns, because one phase is one
    SM cycle -- and asks the *model* whether each of those frames would be legal, then asks the
    driver for the same verdict from its own table.  Both have to say yes for every speed, and
    no for the ones just outside, which is what makes "the configures are legal" a checked
    statement rather than an assertion about two numbers that happen to look right.
    """
    window = spec_minimums(TICK_NS)
    program = load_program()
    n_pw, n_pi = phase_cycles(program)
    step = TICK_NS

    def wire(clkdiv: int) -> tuple:
        per_sm = ticks_per_sm_cycle(clkdiv)
        return n_pw * per_sm, (n_pi + 1) * per_sm

    legal = 0
    for ns in range(window["ns"]["pw_min"], window["ns"]["pw_max"] + 1, int(step)):
        clkdiv = clkdiv_for_pulse(ns, n_pw, n_pi)
        pulse, pause = wire(clkdiv)
        assert window["pw_min"] <= pulse <= window["pw_max"], (ns, pulse)
        assert window["pi_min"] <= pause <= window["pi_max"], (ns, pause)
        assert pulse * step == pytest.approx(ns), (ns, pulse)
        legal += 1
    assert legal == 4001, f"{legal} legal pulse widths on the 20 ns grid"

    # Just outside: a pulse below the floor, one above the ceiling, and a phase length the delay
    # field cannot hold.  All three are refused by name rather than quietly accepted.
    for bad in (window["ns"]["pw_min"] - int(step), window["ns"]["pw_max"] + int(step), 0, -1):
        with pytest.raises(ValueError, match="window|pulse width"):
            clkdiv_for_pulse(bad, n_pw, n_pi)
    for bad_phase in ((0, 10), (1, 33), (33, 10)):
        with pytest.raises(ValueError):
            clkdiv_for_pulse(50_000, *bad_phase)


def test_max_speed_on_the_iss() -> None:
    """The fastest frame this program can legally put on the wire, and where the limit comes from.

    Two limits stack.  The pulse is one SM cycle long, so its width is the CLKDIV period and the
    smallest one the window allows is its floor, 20 us = 1000 ticks at 50 MHz; the pause is ten
    SM cycles plus the `jmp`, so at that divider it is 11 000 ticks = 220 us, comfortably inside
    the 200 us floor.  A frame is therefore 26 bits of 12 000 ticks, 312 000 ticks = 6.24 ms of
    wire time -- about 4.2 kbit/s of pulse rate -- and the quiet time after it, which is not a
    transmission cost but the only frame boundary a receiver has, is 20 ms more.

    The measurement that backs this is `CASES[0]`, the `floor-20us` end-to-end run: 26 pulses of
    exactly 1000 ticks with 11 000 ticks between them and no violation.
    """
    program = load_program()
    n_pw, n_pi = phase_cycles(program)
    floor = spec_minimums(TICK_NS)["pw_min"]
    fastest = clkdiv_for_pulse(floor * TICK_NS, n_pw, n_pi)
    per_sm = ticks_per_sm_cycle(fastest)

    assert per_sm == floor, "the fastest legal pulse is one SM cycle of the floor"
    pulse, pause = n_pw * per_sm, (n_pi + 1) * per_sm
    assert (pulse, pause) == (1_000, 11_000), (pulse, pause)
    bit = pulse + pause
    frame = 26 * bit
    assert (pulse, pause, bit, frame) == (1_000, 11_000, 12_000, 312_000)
    assert frame * TICK_NS == 6_240_000, "6.24 ms of wire time per card"
    assert CORE_HZ / bit == pytest.approx(4166.6667, rel=1e-4), \
        "4.1667 kbit/s of pulse rate, and the ceiling is the 200 us pause floor"
    # Nothing faster is reachable without changing the program: a phase is at least one SM cycle
    # long, and one SM cycle is at least one core cycle.
    with pytest.raises(ValueError):
        clkdiv_for_pulse((floor - int(TICK_NS)) * TICK_NS, n_pw, n_pi)
    with pytest.raises(ValueError):
        clkdiv_for_pulse(TICK_NS, n_pw, n_pi)          # one core cycle per SM cycle is the floor
    # The quiet time that separates two frames is not a transmission cost but is the price of
    # framing: a frame cannot be shorter than its 26 bits plus one pulse interval maximum.
    gap = ceil_ticks(WIEGAND_WINDOW_NS["pi_max"]) + 1
    assert gap == 1_000_001, "pi_max + 1 ticks, the only observable frame boundary"
    assert (frame + gap) * TICK_NS == pytest.approx(26.24e6, rel=1e-3), "26.24 ms per card, wire to wire"


def test_the_driver_refuses_setups_it_cannot_honour() -> None:
    """A pin pair PINDIRS cannot drive, an engine that does not exist, a layout of another width,
    a word that is not a word, and a FIFO that cannot hold what was asked of it."""
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for pin in (7, 8, 12, 17):        # D1 = D0 + 1 leaves the uio pins, or is not one
        with pytest.raises(ValueError, match="uio"):
            WiegandDriver(dev, bus, d0_pin=pin)
    with pytest.raises(ValueError, match="engine"):
        WiegandDriver(dev, bus, engine=2)
    with pytest.raises(ValueError, match="layout"):
        layout("h10301")
    with pytest.raises(ValueError, match="does not fit"):
        frame_bits(256, 0)
    with pytest.raises(ValueError, match="does not fit"):
        frame_bits(0, 65536)

    drv = WiegandDriver(dev, bus)
    drv.load()
    with pytest.raises(ValueError, match="TX FIFO"):
        drv.preload([1, 2, 3, 4, 5])
    with pytest.raises(ValueError):
        drv.push(1 << 32)
    # A program whose bit loop is not the one the driver can time is refused, not guessed at.
    for mutate in (
        lambda p: p.instrs.__setitem__(5, p.instrs[5] & ~0x00E0),      # `set pins, 0`, not pindirs
        lambda p: p.instrs.__setitem__(6, p.instrs[6] & ~0x00E0),      # `jmp idle`, not `jmp y--`
        lambda p: p.instrs.__setitem__(4, 0x6081),                     # `out pindirs, 1`
        lambda p: p.instrs.__setitem__(2, 0x6081),                     # `set y` replaced
    ):
        broken = copy.deepcopy(load_program())
        mutate(broken)
        with pytest.raises(ValueError):
            WiegandDriver(dev, bus, program=broken)


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: a broken program, or a divider the driver would have refused, has to
# make the models complain.  Timing rules fire while they are being measured, so most of these
# runs stop after a few bit times and cost a second or two.
# --------------------------------------------------------------------------------------------

MUTATION_CASE = CASES[0]                    # the cheapest configuration to run
SHORT_RUN = 60_000                          # ticks: a few bit times of the fastest configuration


def _mutated_bench(program=None, clkdiv=None) -> Bench:
    """A bench on the cheapest configuration with the engine's words or its divider replaced.

    Both are put in *after* the driver has been built, deliberately: a program the driver could
    not time and a divider outside the SIA window are both refused at construction, and the point
    of these tests is to put the fault on the wire and watch an independent model notice it.
    """
    bench = Bench(MUTATION_CASE)
    if program is not None:
        bench.driver.program = program
    if clkdiv is not None:
        bench.driver.clkdiv = clkdiv
    return bench


def _run_short(bench: Bench, words=None) -> Bench:
    bench.driver.load()
    bench.driver.start()
    for word in (frame_words(frame_bits(*CARD)) if words is None else words):
        bench.driver.push(word)
    bench.driver.force(driver_mod.jmp_abs(bench.driver.send_addr))
    bench.bus.run(SHORT_RUN)
    return bench


def _break_delay(program: Program, index: int, delay: int) -> Program:
    """One program with the delay field of instruction `index` replaced (bits 12:8, SC = 0)."""
    broken = copy.deepcopy(program)
    field = driver_mod.decode_word(program, broken.instrs[index])["delay"]
    broken.instrs[index] = (broken.instrs[index] & ~(0x1F << 8)) | (delay << 8)
    assert driver_mod.decode_word(broken, broken.instrs[index])["delay"] == delay != field
    return broken


def _break_set(program: Program) -> Program:
    """A program whose pause pulls D0 low instead of releasing it: `set pindirs, 1`.

    That is the open-drain mistake this firmware exists to avoid -- the pause has to leave both
    lines released (`set pindirs, 0`, two pins wide) or the previous pulse never ends and the
    next bit arrives on top of it.
    """
    broken = copy.deepcopy(program)
    assert driver_mod.is_set_pindirs(program, broken.instrs[5], 0)
    broken.instrs[5] = broken.instrs[5] | 1
    return broken


def _break_set_count(program: Program) -> Program:
    """A program one pair short of 26: `set y, 24` instead of `set y, 25`."""
    broken = copy.deepcopy(program)
    assert driver_mod.decode_word(program, broken.instrs[2])["low"] & 0x1F == 25
    broken.instrs[2] = broken.instrs[2] - 1
    return broken


MUTATIONS = [
    # (name, program, clkdiv, the rules the analyser must raise)
    ("a divider far below the window", None, divider_value(100), {"tPW;MIN", "tPI;MIN"}),
    ("a pulse wider than the ceiling", _break_delay(load_program(), 4, 31),
     divider_value(5000), {"tPW;MAX"}),
    ("a pause that pulls a line low instead of releasing it", _break_set(load_program()),
     divider_value(1000), {"tPW;MAX"}),
]


@pytest.mark.parametrize("name,program,clkdiv,rules", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_broken_timing_fails_the_check(name: str, program, clkdiv: int, rules: set) -> None:
    """Each mutation puts something on the wire that the specification forbids, and the analyser
    says so, with the tick and the reason.

    The divider case is one `clkdiv_for_pulse()` refuses to build -- a 2 us pulse and a 22 us
    pause -- and that is the point: the firmware's own guard and the independent judge agree, and
    this proves the judge would have noticed anyway.  The program cases change one instruction of
    the loop, which is what an off-by-one in the .pio file looks like: a pulse 32 SM cycles long,
    and a pause that pulls D0 low instead of releasing it, which is exactly the "both lines low
    at once" failure the specification forbids -- except that here it also shows up as a line
    held far past the pulse-width ceiling.
    """
    clean = _run_short(_mutated_bench())
    assert clean.checker.clean and len(clean.checker.pulses) > 0, (
        "the unmutated run must be clean and must really put pulses on the wire, or this test "
        f"proves nothing: {clean.checker.report()}"
    )

    bench = _run_short(_mutated_bench(program=program, clkdiv=clkdiv))
    assert rules <= bench.rules(), f"{name}: expected {sorted(rules)}, got {bench.checker.report()}"
    assert bench.checker.violations, f"{name}: nothing was flagged at all"
    for violation in bench.checker.violations:
        assert violation.tick > 0 and violation.detail, "every violation carries a tick and a reason"


@LONG
def test_inverted_pairs_are_caught_by_the_decoders() -> None:
    """Wrong data on the wire is caught, even though the timing is perfect.

    Every direction pair inverted -- 01 becomes 10 and back -- is a frame whose bits are all
    inverted, which is what a single swapped pin pair or a mis-encoded pair table looks like.
    Nothing about the *timing* is wrong, so only the two decoders can see it: the panel reads a
    different credential, and the parity bits the host computed no longer match the bits that
    arrived, so the analyser raises PARITY;LEAD or PARITY;TRAIL.  This is why both models sit on
    this wire and why the parity is computed on the host at all.
    """
    # 0b11 per pair, so 0x3333_3333 swaps 01 and 10 and nothing else
    words = [w ^ 0x3333_3333 for w in frame_words(frame_bits(*CARD))]
    bench = _run_short(_mutated_bench(), words=words)
    bench.driver.run()                      # let the quiet time close the frame

    chk, rcv = bench.checker, bench.receiver
    assert {"PARITY;LEAD", "PARITY;TRAIL"} & bench.rules(), chk.report()
    assert len(chk.pulses) == 26 and len(chk.frames) == 1
    assert not rcv.clean, "the receiver accepted a frame with the wrong parity"
    assert {p.bit for p in chk.pulses} == {0, 1} and len(chk.pulses) == 26, (
        "every bit is still there, on the other line: only its value changed"
    )
    assert (chk.frames[0].facility, chk.frames[0].number) != CARD, (
        "the inverted frame reads back as the credential it was built from"
    )
    assert chk.frames[0].card is not None and not chk.frames[0].card.parity_ok


@LONG
def test_a_bit_missing_is_measured_as_a_short_frame() -> None:
    """One pair short of 26, both models count 25 bits and the frame comes out too short.

    The frame length is a rule about the *data*, not about the timing, so it is the one that both
    models agree on and neither of them can see until the lines go quiet.  This sends 25 pairs
    (`set y, 24`), lets the quiet time close the frame, and asks for both BITS and the tPI;MAX
    that always comes with it: the lines went quiet to the frame timeout with a bit still
    missing, which on the wire is indistinguishable from a reader that is too slow.
    """
    bench = _run_short(_mutated_bench(program=_break_set_count(load_program())))
    bench.driver.run()

    chk, rcv = bench.checker, bench.receiver
    assert len(chk.pulses) == 25, f"the analyser counted {len(chk.pulses)} pulses, not 25"
    assert bench.rules() == {"BITS", "tPI;MAX"}, chk.report()
    assert "after only 25 of 26 bits" in chk.violations_by_rule("tPI;MAX")[0].detail
    assert len(chk.frames) == 1 and len(chk.frames[0].bits) == 25 and chk.frames[0].card is None
    assert not rcv.clean and "expected 26" in rcv.errors[0], rcv.report()
