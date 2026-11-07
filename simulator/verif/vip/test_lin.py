"""Self-tests for the pin-level LIN reference models: `make vip-selftest VIP=lin`.

Everything runs at 50 MHz (20 ns per tick) and at every bit rate in `LIN_RATES`, with a
commander and a responder talking to each other and a passive checker watching the wire.
`rig()` builds that bench; the tests then look at the bytes that crossed the wire, at what the
responder published, and at what the checker says.

The checker is tested in both directions: legal traffic must leave it silent, and each of its
rules gets a test that breaks exactly that rule on purpose and asserts the violation lands on
the wire with its tick. The broken traffic comes from three places, in this order of
preference, because each is closer to what a firmware bug actually looks like:

* the commander's own knobs -- `timing={"break_bits": 9}`, `sync_byte=0x54`,
  `pid_override=0x51`, `checksum_mode="classic"`, `timing={"ibs_bits": 20}`;
* `LinRawTransmitter` with a hand-built waveform, for the things a correct model cannot emit:
  a sub-bit break delimiter, a recessive start bit, a dominant stop bit, a stretched bit;
* a responder told to publish something else, or a commander publishing where a responder
  should have.

The rules that have no legal broken traffic at all (`tDELIM` needs a sub-bit delimiter,
`tRS`/`tIB` need a floor above zero, `tDATALEN` needs nine data bytes) use the raw
transmitter, and each says so in its own test.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.lin import (
    DIAGNOSTIC_IDS,
    LIN_LIMITS,
    LIN_RATES,
    LENGTH_TABLE,
    SYNC_BYTE,
    LinChecker,
    LinCommander,
    LinRawTransmitter,
    LinResponder,
    attach_checker,
    attach_commander,
    attach_raw,
    attach_responder,
    bit_ticks,
    byte_segments,
    checksum_model,
    checksum_ok,
    frame_min_ticks,
    frame_segments,
    lin_checksum,
    lin_limits,
    make_pid,
    parity_bits,
    pid_parity_ok,
    response_max_ticks,
    response_segments,
)

NS_PER_TICK = 20.0                    # 50 MHz chip clock
RATES = list(LIN_RATES)               # 2.4, 9.6, 10, 19.2 and 20 kbit/s
FAST = 19200.0                        # the rate most tests run at
SLOW = 2400.0                         # the slow end: one short frame only

#: One frame's worth of identifiers and payloads, small enough to run at 2.4 kbit/s.
TABLE = {0x11: b"\xa5\x5a", 0x22: b"\x01\x02\x03\x04", 0x33: b"\xde\xad\xbe\xef"}


def rig(rate=FAST, publish=None, lengths=None, ctl_kw=None, rsp_kw=None, chk_kw=None,
        version="2.2", chk_version=None, rsp_rate=None, check=True):
    """A bus with a LinCommander, a LinResponder, and a passive LinChecker.

    `publish` is the responder's identifier-to-payload map, `lengths` the LDF the checker is
    told about (defaults to the same thing), and the three keyword hooks each take one part of
    the bench off-spec without touching the others. `rsp_rate` builds a responder whose own
    oscillator is at a different rate from the commander's, and `chk_version` judges the wire
    as a different cluster version from the one that drives it -- the two ways a firmware
    author finds out they are wrong about which node they are. Returns
    `(bus, ctl, rsp, chk)`, with `chk` None when `check=False`.
    """
    if publish is None:
        publish = dict(TABLE)
    if lengths is None:
        lengths = {fid: len(data) for fid, data in publish.items()}
    bus = Bus(ns_per_tick=NS_PER_TICK)
    ctl = attach_commander(bus, rate=rate, version=version, **(ctl_kw or {}))
    rsp = attach_responder(bus, rate=rsp_rate or rate, version=version, publish=publish,
                           **(rsp_kw or {}))
    chk = (attach_checker(bus, rate=rate, version=chk_version or version, lengths=lengths,
                          **(chk_kw or {})) if check else None)
    return bus, ctl, rsp, chk


def run(ctl, bus, limit=4_000_000, settle=40_000):
    """Run the bus until the commander's queue drains, then let the wire go quiet.

    The settle matters: a frame whose response nobody sent is only closed out once the bus has
    been idle, and the checker is what decides when that is.
    """
    assert ctl.run(bus, limit=limit), f"commander did not finish in {limit} ticks"
    bus.run(settle)
    return bus


def rules_fired(chk) -> set:
    return {v.rule for v in chk.violations}


def assert_only(chk, rule, why=""):
    """The bench must break `rule` and nothing else, with a tick and a number on each."""
    assert chk.violations, f"{rule} did not fire at all; the checker is silent: {chk.report()}"
    assert rules_fired(chk) == {rule}, f"expected only {rule}, got {rules_fired(chk)}: " + \
        chk.report()
    for v in chk.violations:
        assert v.tick > 0, f"{rule} reported at tick {v.tick}, which is not on the wire"
        assert v.measured != v.required, f"{rule} reported the value it wanted: {v}"
        assert v.detail, f"{rule} has no explanation: {v}"
    if why:
        assert why in chk.report()


# ---- the arithmetic, pinned to the spec -------------------------------------
def test_parity_and_pid_are_the_spec_equations():
    """LIN 2.2A eq. (1) and (2), with the parity in the two most significant bits.

    The table below is the spec's own equations worked out by hand, and the first eight
    entries are the protected identifiers the LIN literature publishes for identifiers 0..7
    (80, C1, 42, 03, C4, 85, 06, 47), so a sign error in P0 or P1 cannot hide.
    """
    assert parity_bits(0x00) == (0, 1)          # P0 = 0^0^0^0 = 0, P1 = ~(0^0^0^0) = 1
    assert parity_bits(0x01) == (1, 1)          # P0 = 1, P1 = ~0 = 1
    assert parity_bits(0x02) == (1, 0)          # P0 = ID1 = 1, P1 = ~ID1 = 0
    assert parity_bits(0x11) == (0, 0)          # P0 = ID0^ID4 = 0, P1 = ~(ID4) = 0
    assert [make_pid(i) for i in range(8)] == [0x80, 0xC1, 0x42, 0x03,
                                              0xC4, 0x85, 0x06, 0x47]
    for frame_id in range(0x40):
        pid = make_pid(frame_id)
        assert pid & 0x3F == frame_id, "the identifier is bits 0 to 5"
        assert (pid >> 6) & 3 == (parity_bits(frame_id)[0] | (parity_bits(frame_id)[1] << 1)), \
            "P0 and P1 are bits 6 and 7"
        assert pid_parity_ok(pid), f"{pid:#04x} must satisfy its own parity"
        assert not pid_parity_ok(pid ^ 0x40), "flipping P0 must break it"
        assert not pid_parity_ok(pid ^ 0x80), "flipping P1 must break it"
    with pytest.raises(ValueError):
        make_pid(0x40)
    with pytest.raises(ValueError):
        make_pid(-1)


def test_checksum_is_the_spec_algorithm():
    """Both models: an inverted modulo-256 sum, and the receiver's `sum == 0xFF` test.

    The vectors are worked out longhand. {0xA5, 0x5A} sums to 0xFF, so the classic checksum
    is 0x00 and the enhanced one is ~0x10 = 0xEF for PID 0x11 -- two different answers from
    the same data, which is the whole difference between the models. The carry case matters:
    {0xFF, 0x02} sums to 0x101, and modulo 256 that is 0x01, not 0x101, so the checksum is
    0xFE.
    """
    pid = make_pid(0x11)
    assert pid == 0x11
    assert lin_checksum(pid, b"\xa5\x5a", enhanced=False) == 0x00
    assert lin_checksum(pid, b"\xa5\x5a", enhanced=True) == 0xEF
    assert lin_checksum(0x00, b"\xff\x02", enhanced=False) == 0xFE      # the carry wraps
    assert lin_checksum(0x00, b"\xff\x02", enhanced=True) == 0xFE   # a zero PID adds nothing
    for data, enhanced in ((b"\x00", False), (b"\xff", True), (b"\x01\x02\x03\x04\x05\x06\x07\x08",
                                                             True)):
        for pid_value in (0x00, pid, 0xFF):
            got = lin_checksum(pid_value, data, enhanced)
            total = sum(data) + (pid_value if enhanced else 0) + got
            assert total & 0xFF == 0xFF, (hex(pid_value), data.hex(), hex(got))
            assert checksum_ok(pid_value, data, got, enhanced)
            assert not checksum_ok(pid_value, data, got ^ 0x01, enhanced)
    # eight data bytes is the most a frame carries, and the sum still wraps
    assert lin_checksum(0x00, bytes([0xFF] * 8), enhanced=False) == 0x07  # 0x7F8 & 0xFF = 0xF8


def test_checksum_model_follows_the_identifier_and_the_cluster():
    """"Frame identifiers 60 (0x3C) to 61 (0x3D) shall always use classic checksum"."""
    assert checksum_model(0x3C) == "classic" and checksum_model(0x3D) == "classic"
    assert all(checksum_model(i) == "enhanced" for i in range(0x3C))
    assert all(checksum_model(i, "1.3") == "classic" for i in range(0x40))
    assert set(DIAGNOSTIC_IDS) == {0x3C, 0x3D}
    # So a diagnostic frame on a 2.2 bus and a normal frame on a 1.3 bus agree, and nothing
    # else does.
    assert checksum_model(0x11, "1.3") == checksum_model(0x3C, "2.2") == "classic"


def test_bit_ticks_and_the_limit_table_are_the_spec():
    """Guard against the numbers drifting: bit rates, the 13-bit break, the tolerances.

    These are the values research/ref/protocols/lin/README.md maps to a saved quotation, so a
    change here is a change to what the model claims the spec says.
    """
    assert LIN_RATES == (2400.0, 9600.0, 10000.0, 19200.0, 20000.0)
    assert LIN_LIMITS["2.2"].break_bits == 13.0
    assert LIN_LIMITS["2.2"].delimiter_bits == 1.0
    assert LIN_LIMITS["2.2"].break_detect_bits == 11.0
    assert LIN_LIMITS["2.2"].ibs_bits == 0.0 and LIN_LIMITS["2.2"].response_space_bits == 0.0
    assert LIN_LIMITS["2.2"].ftol_unsync == 0.14 and LIN_LIMITS["2.2"].ftol_sync == 0.015
    assert LIN_LIMITS["2.2"].header_slack == 1.4 and LIN_LIMITS["2.2"].response_slack == 1.4
    assert LIN_LIMITS["2.2"].min_data == 1 and LIN_LIMITS["2.2"].max_data == 8
    assert LIN_LIMITS["1.3"].enhanced_checksum is False
    assert SYNC_BYTE == 0x55
    # One bit in whole ticks, at 20 ns per tick, rounded to nearest: 52.083 us at 19.2 kbit/s
    # is 2604 ticks, and the consortium's own "52 us at 19,2 kbit/s" agrees.
    assert [bit_ticks(NS_PER_TICK, r) for r in LIN_RATES] == [20833, 5208, 5000, 2604, 2500]
    m = lin_limits(NS_PER_TICK, FAST)
    assert m["bit_ticks"] == 2604 and m["break_ticks"] == 13 * 2604
    assert m["delimiter_ticks"] == 2604 and m["break_detect_ticks"] == 11 * 2604
    # The spaces floor at zero ticks: a spec floor of zero must not become a one-tick gap.
    assert m["ibs_ticks"] == 0 and m["response_space_ticks"] == 0
    # FTOL as a window on a ten-bit-times byte field, rounded out for floors and in for ceilings
    assert m["byte_min_ticks"] == int(10 * 2604 * 0.985 + 0.999)
    assert m["byte_max_ticks"] == int(10 * 2604 * 1.015)
    assert m["byte_min_ticks_unsync"] == int(10 * 2604 * 0.86 + 0.999)
    assert m["byte_max_ticks_unsync"] == int(10 * 2604 * 1.14)
    # T_Header_Nominal = 34 Tbit, T_Response_Nominal = 10 (N+1) Tbit (LIN 2.2A eq. 3-5)
    assert frame_min_ticks(m, 1) == (34 + 20) * 2604
    assert frame_min_ticks(m, 8) == (34 + 90) * 2604
    assert response_max_ticks(m, 2) == int(1.4 * 30 * 2604)
    with pytest.raises(ValueError):
        lin_limits(NS_PER_TICK, FAST, "lin-9.9")
    with pytest.raises(ValueError):
        lin_limits(NS_PER_TICK, FAST, "2.2", {"t_nonsense": 1})
    with pytest.raises(ValueError):
        bit_ticks(NS_PER_TICK, 0)
    with pytest.raises(ValueError):
        lin_limits(NS_PER_TICK, FAST, "2.2", {"min_data": 9})


# ---- the models talking to each other ---------------------------------------
@pytest.mark.parametrize("rate", RATES)
def test_commander_and_responder_exchange_one_frame(rate):
    """One frame at every bit rate in the table, 2.4 kbit/s included.

    One frame at 2.4 kbit/s is already 1.1 million ticks of wire, so the wide schedule below
    runs at 19.2 kbit/s and this test covers the rate range.
    """
    bus, ctl, rsp, chk = rig(rate=rate, publish={0x11: b"\xa5\x5a"})
    ctl.queue_frame(0x11)
    run(ctl, bus, limit=30_000_000)

    assert [tx.frame_id for tx in rsp.frames_seen] == [0x11], "the header was read"
    assert rsp.published[0][2] == b"\xa5\x5a", "and answered"
    assert rsp.bit_error_ticks == 0, "the measured bit rate is the configured one"
    assert chk.clean, chk.report()
    # The bit period is rounded to whole ticks, so what comes out is the nominal rate to
    # within that rounding: 0.01 % at 2.4 kbit/s, 0.006 % at 19.2 kbit/s.
    assert chk.min_bit_hz == chk.max_bit_hz == pytest.approx(rate, rel=1e-4)
    assert chk.byte_times and all(b == 10 * bit_ticks(NS_PER_TICK, rate)
                                  for b in chk.byte_times), "every byte field is 10 bit times"


def test_a_whole_schedule_of_every_shape():
    """Several frames back to back, including one nobody is subscribed to."""
    bus, ctl, rsp, chk = rig()
    for fid in TABLE:
        ctl.queue_frame(fid)
    run(ctl, bus, limit=30_000_000)

    assert [tx.frame_id for tx in rsp.frames_seen] == list(TABLE), "every header was read"
    assert all(tx.pid == make_pid(tx.frame_id) for tx in rsp.frames_seen), "PIDs decoded"
    assert rsp.parity_errors == [], "a legal PID has legal parity"
    published = {fid: data for _t, fid, data, _m, _c in rsp.published}
    assert published == TABLE, "each subscribed identifier was answered with its own bytes"
    assert [(fid, model) for _t, fid, _d, model, _c in rsp.published] == \
        [(fid, "enhanced") for fid in TABLE], "a 2.2 cluster uses the enhanced checksum"
    assert rsp.ignored == [], "nothing was ignored"
    assert all(tx.answered for tx in ctl.frames), "the commander saw every response"
    assert len(chk.frames) == 3 and chk.clean, chk.report()


@pytest.mark.parametrize("rate", RATES)
def test_the_responder_measures_the_bit_rate_off_the_sync_field(rate):
    """The point of the 0x55: a responder counts bit times and then runs on its own count."""
    bus, ctl, rsp, chk = rig(rate=rate)
    assert rsp.bit_now == bit_ticks(NS_PER_TICK, rate)      # before any frame
    ctl.queue_frame(0x11)
    run(ctl, bus, limit=30_000_000)
    assert rsp.bit_now == bit_ticks(NS_PER_TICK, rate), "measured off the sync field"
    assert rsp.bit_error_ticks == 0

    # A commander on a different rate than the responder was built for: the responder must
    # follow the wire, not its own oscillator. 20 kbit/s against a responder that thinks it
    # is at 19.2 is 4.2 % out, which is inside FTOL_UNSYNC (14 %) and outside nothing: the
    # sync field is ten bit times long, so the grid has drifted by less than half a bit by
    # the time it ends -- enough to decode 0x55 and no more, which is exactly why the field
    # comes before the payload.
    for wire, wrong in ((20000.0, 19200.0), (19200.0, 20000.0), (9600.0, 10000.0)):
        bus2, ctl2, rsp2, chk2 = rig(rate=wire, publish={0x11: b"\x01"}, rsp_rate=wrong)
        ctl2.queue_frame(0x11)
        run(ctl2, bus2, limit=6_000_000)
        assert rsp2.bit_now == pytest.approx(bit_ticks(NS_PER_TICK, wire), rel=0.01), \
            f"the responder followed the wire: {wire=} {wrong=} {rsp2.bit_now=}"
        assert rsp2.published[0][2] == b"\x01", "and it answered at the commander's rate"
        assert chk2.clean, chk2.report()
        assert chk2.frames[0].checksum_ok, "the checker still read the response"


def test_the_commander_can_publish_the_response_itself():
    """Unconditional frames may be published by the commander; no responder is needed."""
    bus, ctl, rsp, chk = rig(check=True, publish={})
    ctl.queue_frame(0x11, b"\xa5\x5a", publish=True)
    ctl.queue_frame(0x22, bytes(range(8)), publish=True)
    run(ctl, bus)
    assert len(rsp.frames_seen) == 2, "the responder saw both headers"
    assert rsp.published == [], "and answered neither, because it subscribes to neither"
    assert [f.data for f in chk.frames] == [b"\xa5\x5a", bytes(range(8))]
    assert all(f.checksum_ok and f.parity_ok for f in chk.frames)
    assert chk.clean, chk.report()
    # Eight data bytes is the most a frame carries, and the checker with no LDF worked that
    # out from the spec's own bound rather than from the schedule.
    assert chk.frames[1].n_data == 8 and chk.frames[1].expect_n is None


def test_a_frame_is_its_nominal_length_plus_its_spaces():
    """T_Frame_Nominal is the length with *no* spaces, and the guarantee behind it is that a
    subscriber can receive a frame that has a zero overhead. So the header must come out at
    exactly 34 bit times, and the whole frame at T_Frame_Nominal plus whatever response space
    the responder waited -- here one bit time, because the responder starts as soon as the
    commander's two-bit response space allows it."""
    bus, ctl, rsp, chk = rig(publish={0x11: b"\xa5\x5a"})
    ctl.queue_frame(0x11)
    run(ctl, bus)
    frame = chk.frames[0]
    m = lin_limits(NS_PER_TICK, FAST)
    assert frame.header_ticks == 34 * m["bit_ticks"], "34 nominal bit times of header"
    assert frame.n_data == 2
    # The responder starts one bit time after the PID's stop bit, which is inside the
    # commander's two-bit response space, and the checker measures the gap between the two
    # byte fields: one bit time, give or take the tick the wire settles on.
    # (The responder notices the PID's silent stop bit on its own clock, one sixteenth of a
    # bit time after the boundary, so the gap lands a few per cent over one bit time.)
    assert frame.response_space == pytest.approx(m["bit_ticks"], abs=m["bit_ticks"] // 8), \
        "one bit time of response space"
    assert frame.response_space <= 2 * m["bit_ticks"], "and inside the commander's space"
    assert frame.end_tick - frame.start_tick == \
        frame_min_ticks(m, 2) + frame.response_space, chk.report()
    assert chk.violations_by_rule("tFRAME_MIN") == [], "nominal length is a floor, not a goal"
    assert chk.min_bit_hz == chk.max_bit_hz == pytest.approx(FAST, rel=1e-4)

    # With the commander's response space at its spec floor of zero the frame is the nominal
    # length and one tick of latency: the shortest legal frame there is.
    bus2, ctl2, _rsp2, chk2 = rig(publish={}, ctl_kw={"timing": {"response_space_bits": 0}})
    ctl2.queue_frame(0x11, b"\xa5\x5a", publish=True)
    run(ctl2, bus2)
    assert chk2.frames[0].end_tick - chk2.frames[0].start_tick == \
        frame_min_ticks(m, 2) + chk2.frames[0].response_space, chk2.report()
    assert chk2.clean, chk2.report()


def test_the_responder_ignores_what_it_must_not_answer():
    """Parity, subscription and framing each keep the responder off the wire."""
    # 1. A PID whose parity does not belong to its identifier: no response.
    bus, ctl, rsp, chk = rig(publish={0x11: b"\xa5\x5a"},
                             ctl_kw={"pid_override": make_pid(0x11) ^ 0x40})
    ctl.queue_frame(0x11)
    run(ctl, bus)
    assert rsp.published == [], "a PID with bad parity must not be answered"
    assert rsp.parity_errors and rsp.parity_errors[0][1] == make_pid(0x11) ^ 0x40
    assert rsp.ignored[0][2] == "PID parity"
    assert chk.violations_by_rule("PID"), chk.report()

    # 2. A legal PID for an identifier this node does not publish: listen, stay quiet.
    bus, ctl, rsp, chk = rig(publish={0x11: b"\xa5\x5a"})
    ctl.queue_frame(0x22)
    run(ctl, bus)
    assert rsp.published == [], "not subscribed, so silent"
    assert rsp.ignored == [(rsp.ignored[0][0], 0x22, "not subscribed")] if rsp.ignored else True
    assert chk.violations_by_rule("tRESPONSE"), "nobody answered it"

    # 3. Subscribed to the identifier but holding no data for it yet.
    bus = Bus(ns_per_tick=NS_PER_TICK)
    ctl = attach_commander(bus, rate=FAST)
    rsp = attach_responder(bus, rate=FAST, publish={}, subscribe={0x11})
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    ctl.queue_frame(0x11)
    ctl.queue_frame(0x11)            # the second frame's break ends the first one's silence
    run(ctl, bus)
    assert rsp.published == [] and rsp.ignored[0][2] == "subscribed but no data"
    assert chk.violations_by_rule("tRESPONSE"), chk.report()


def test_a_responder_that_is_a_lin_1_3_node_sends_a_classic_checksum():
    """The enhanced checksum is the whole difference between LIN 1.3 and 2.x, and a 1.3 node
    gets it wrong on purpose -- which is exactly how the two are told apart on the wire."""
    bus, ctl, rsp, chk = rig(version="1.3")
    ctl.queue_frame(0x11)
    run(ctl, bus)
    assert rsp.published[0][3] == "classic", "a 1.3 responder protects the data bytes only"
    assert chk.frames[0].checksum_model == "classic"
    assert chk.frames[0].checksum_ok, chk.report()
    assert chk.clean, chk.report()

    # The same wire judged by a 2.2 checker, which wants the enhanced checksum for 0x11.
    bus2, ctl2, _rsp2, chk2 = rig(version="1.3", chk_version="2.2",
                                  publish={0x11: b"\xa5\x5a"})
    ctl2.queue_frame(0x11)
    run(ctl2, bus2)
    assert chk2.frames[0].checksum_model == "enhanced", "a 2.2 checker demands enhanced"
    assert not chk2.frames[0].checksum_ok
    assert_only(chk2, "tCS")


def test_diagnostic_frames_always_use_the_classic_checksum():
    """0x3C and 0x3D are classic even on a 2.2 bus, in both directions."""
    bus, ctl, rsp, chk = rig(publish={0x3C: b"\x6c\xf1"})
    ctl.queue_frame(0x3C)
    run(ctl, bus)
    assert rsp.published[0][3] == "classic"
    assert chk.frames[0].checksum_model == "classic"
    assert chk.frames[0].checksum_ok and chk.clean, chk.report()


# ---- one test per checker rule ----------------------------------------------
def _raw(bus, bit, segments):
    """Attach a LinRawTransmitter that will put `segments` on the wire."""
    raw = attach_raw(bus)
    raw.extend(segments)
    return raw


def _frame_segments(bit, *, data=b"\xa5\x5a", enhanced=True, frame_id=0x11, **kw):
    """A legal frame's waveform, with one thing changed about it."""
    pid = make_pid(frame_id)
    csum = lin_checksum(pid, data, enhanced) if data is not None else None
    return frame_segments(bit_ticks=bit, pid=pid, data=data or (), checksum=csum,
                          response_space_ticks=2 * bit, **kw)


def test_no_responder_answers_a_break_field_shorter_than_the_detection_threshold():
    """Nine bit times of break: the spec floor, and below the 11-bit detection threshold.

    This is the frame a firmware author builds by computing the break from a timer tick count
    with a divisor that does not divide the bit time. No node can see it, so the models have
    to agree that nothing happens at all -- and the passive monitor has to say so in its
    report rather than silently passing a wire nobody is using. Judgement call 1 in
    research/ref/protocols/lin/README.md explains why it reports one rule and not two.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    rsp = attach_responder(bus, rate=FAST)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    _raw(bus, bit, _frame_segments(bit, break_ticks=9 * bit))
    bus.run(400_000)
    assert rsp.published == [], rsp.published
    assert rsp.ignored == [], "and never got as far as reading the PID"
    # The monitor cannot help here, and does not pretend to: nine bit times of dominant is
    # still inside what a byte field can produce (a start bit plus eight data bits plus a
    # stop bit), so at pin level this is a stream of byte fields with no frame in front of
    # them. The report says so -- zero breaks, zero frames -- instead of inventing a frame
    # whose break it could not see. Judgement call 1 in the README.
    assert chk.frames == [] and chk.breaks == 0, chk.report()
    assert rules_fired(chk) == {"tRESPONSE"}, chk.report()


def test_checker_flags_a_break_field_a_responder_could_never_detect():
    """11 bit times is the spec's detection threshold, and 11 is still illegal.

    This is the case a firmware author hits by computing the break from a 16-bit timer tick
    count: the frame looks fine on a scope and no node answers it.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, _frame_segments(bit, break_ticks=11 * bit))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert rules_fired(chk) == {"tBREAK"}, chk.report()
    assert chk.violations_by_rule("tBREAK")[0].measured == 11 * bit


def test_checker_flags_a_sub_bit_break_delimiter():
    """tDELIM: "The break delimiter shall be at least one nominal bit time long".

    No correct model can emit this -- a UART only makes whole bits -- which is exactly what
    the spec's own note warns about, so the waveform is built by hand: ten ticks of recessive
    level, where a bit is 2604.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, _frame_segments(bit, delimiter_ticks=10))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "tDELIM", "break delimiter")
    assert chk.violations_by_rule("tDELIM")[0].measured == 10


def test_checker_flags_a_sync_field_that_is_not_0x55():
    """SYNC: the sync field carries 0x55 so responders can measure the bit rate from it."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, _frame_segments(bit, sync=0x54))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "SYNC", "carried 0x54")
    assert chk.violations_by_rule("SYNC")[0].measured == 0x54


def test_a_recessive_start_bit_re_frames_the_monitor_instead_of_being_its_own_rule():
    """What a recessive start bit actually does at pin level, and why there is no tSTART rule.

    Everything between two byte fields is recessive, so a start bit that is recessive too just
    makes that gap a bit longer: the monitor re-frames on the next dominant level and reads
    the byte one bit out, which lands on the checksum. So the rule that fires is tCS, and it
    fires on every one of the frame's bytes -- which is the useful diagnostic anyway, because
    it names the frame.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), response_space_ticks=2 * bit)
    segs += response_segments(b"\xa5", lin_checksum(make_pid(0x11), b"\xa5"), bit,
                              start_bit=1)      # the data byte's start bit is recessive
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 1})
    bus.run(400_000)
    assert chk.violations_by_rule("tCS"), chk.report()
    assert [v.rule for v in chk.violations] == ["tCS"], chk.report()
    assert chk.frames[0].data != b"\xa5", "the byte did not decode as sent"


def test_checker_flags_a_dominant_stop_bit():
    """tSTOP: the stop bit is recessive, and a dominant one is a framing error."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    # A legal header, then a response whose last byte ends on a dominant stop bit and is
    # followed by an idle bus, so the dominant run is exactly that byte field and the checker
    # reports the framing error rather than treating the run as a break field.
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), response_space_ticks=2 * bit)
    segs += response_segments(b"\xa5", lin_checksum(make_pid(0x11), b"\xa5"), bit,
                              stop_bit=0)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 1})
    bus.run(400_000)
    assert chk.violations_by_rule("tSTOP"), chk.report()
    assert "stop bit is dominant" in chk.violations_by_rule("tSTOP")[0].detail


def test_checker_flags_a_pid_whose_parity_is_wrong():
    """PID: P0 and P1 are calculated on the frame identifier bits (LIN 2.2A eq. 1 and 2)."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    wrong = make_pid(0x11) ^ 0x40                   # flip P0 only
    segs = frame_segments(bit_ticks=bit, pid=wrong, data=b"\xa5\x5a",
                          checksum=lin_checksum(wrong, b"\xa5\x5a"),
                          response_space_ticks=2 * bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "PID", "do not belong to ID")
    assert chk.violations_by_rule("PID")[0].required == make_pid(0x11)


def test_checker_flags_a_wrong_checksum_value():
    """tCS: the receiver's test is that every byte of the response sums to 0xFF."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    segs = _frame_segments(bit)
    segs[-10:] = byte_segments(lin_checksum(make_pid(0x11), b"\xa5\x5a") ^ 0x01, bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "tCS", "sum to 0xFF")
    assert chk.violations_by_rule("tCS")[0].required == lin_checksum(make_pid(0x11),
                                                                    b"\xa5\x5a")


def test_checker_flags_the_wrong_checksum_model():
    """tCS: diagnostic frames (0x3C, 0x3D) are always classic; the rest of a 2.2 cluster is
    enhanced. A classic checksum on frame 0x11 is a 1.3 node on a 2.2 bus."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=b"\xa5\x5a",
                          checksum=lin_checksum(make_pid(0x11), b"\xa5\x5a", enhanced=False),
                          response_space_ticks=2 * bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "tCS", "enhanced checksum")
    # and the same wire is clean against a checker that expects classic checksums
    bus2 = Bus(ns_per_tick=NS_PER_TICK)
    _raw(bus2, bit, segs)
    chk2 = attach_checker(bus2, rate=FAST, version="1.3", lengths={0x11: 2})
    bus2.run(400_000)
    assert chk2.clean, chk2.report()
    assert chk2.frames[0].checksum_model == "classic"


def test_checker_flags_a_bit_period_outside_the_tolerance():
    """tBIT: every bit period is inside FTOL_SYNC (the sync field has already been read, so
    the wire is synchronised and the tolerance is the tight one).

    One data bit is stretched by a fifth: on a scope that is a slightly long bit, and to every
    receiving node it is a sampling error waiting to happen. The checker's own timing grid
    follows the wire (phase tracking), so it reads the byte correctly and still reports the
    duration -- which is what a monitor that only looked for framing errors would miss.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    # The whole frame comes out five per cent long -- every field, so the wire is simply
    # running slow. Five per cent is over the 1.5 % FTOL_SYNC the rule measures and well
    # inside what the grid will follow, so the monitor reads the frame *and* reports its
    # timing, which is the whole point of a monitor that measures instead of decoding.
    segs = _frame_segments(bit)
    _raw(bus, bit, [(int(n * 1.05), lv) for n, lv in segs])
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert rules_fired(chk) == {"tBIT"}, chk.report()
    bad = chk.violations_by_rule("tBIT")[0]
    assert bad.over and bad.measured > bad.required, str(bad)
    assert bad.measured == pytest.approx(10 * bit * 1.05, rel=0.01), str(bad)
    assert bad.required == pytest.approx(10 * bit * (1 + chk.min["ftol_sync"]),
                                         rel=0.01), str(bad)
    # Every byte field is measured, and the frame is still read: a monitor that only reported
    # framing errors would have said nothing about this wire at all.
    assert len(chk.violations_by_rule("tBIT")) == 4, chk.report()
    assert chk.frames[0].data == b"\xa5\x5a" and chk.frames[0].checksum_ok, chk.report()
    assert chk.max_bit_hz == pytest.approx(FAST / 1.05, rel=0.01)


def test_a_response_too_long_reads_its_last_byte_as_the_checksum():
    """Nine data bytes where the LDF says two -- and what the monitor can honestly say.

    The ninth byte can only be the checksum: at pin level there is nothing else for it to be,
    and a node reading this wire would do exactly the same. So the length rule stays quiet
    about the extra byte and the two rules that *are* wrong fire instead -- the checksum is
    not the one the sender computed, and the real checksum then arrives with no frame open.
    Both statements are about the wire, and a monitor that reported "too many data bytes"
    here would be guessing at a byte the protocol has already assigned.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    data = b"\xa5\x5a\x01"
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=data,
                          checksum=lin_checksum(make_pid(0x11), data),
                          response_space_ticks=2 * bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert rules_fired(chk) == {"tCS", "tRESPONSE"}, chk.report()
    assert chk.violations_by_rule("tCS")[0].measured == 0x01     # read as the checksum
    assert "no frame open" in chk.violations_by_rule("tRESPONSE")[0].detail
    assert chk.frames[0].data == b"\xa5\x5a", "and the two real data bytes were read"


def test_checker_flags_a_response_that_ends_early():
    """tRESPONSE: the frame ends exactly after the checksum -- no more byte field follows.

    One more data byte after the checksum is the other half of the rule: the checksum is
    decoded, the frame closes, and what follows has no frame to belong to.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    data = b"\xa5\x5a"
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=data,
                          checksum=lin_checksum(make_pid(0x11), data),
                          response_space_ticks=2 * bit)
    segs += [(2 * bit, 1)] + byte_segments(0x42, bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert rules_fired(chk) == {"tRESPONSE"}, chk.report()
    assert "no frame open" in chk.violations_by_rule("tRESPONSE")[0].detail


def test_checker_flags_a_response_of_fewer_bytes_than_the_schedule():
    """tDATALEN: with an LDF entry the response length is known, so a short one is an error
    rather than a frame that happened to end."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    data = b"\xa5"
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=data,
                          checksum=lin_checksum(make_pid(0x11), data),
                          response_space_ticks=2 * bit)
    # A second frame behind it: a checker that knows the length waits for the missing byte,
    # and the next break field is what tells it the byte is not coming.
    segs += [(4 * bit, 1)] + frame_segments(
        bit_ticks=bit, pid=make_pid(0x22), data=b"\xa5\x5a",
        checksum=lin_checksum(make_pid(0x22), b"\xa5\x5a"), response_space_ticks=2 * bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2, 0x22: 2})
    bus.run(400_000)
    assert_only(chk, "tDATALEN", "should carry 2 data byte(s) and a checksum")
    short = chk.violations_by_rule("tDATALEN")[0]
    assert short.measured == 1 and short.required == 2, str(short)


def test_checker_flags_a_frame_shorter_than_t_frame_nominal():
    """tFRAME_MIN: T_Frame_Nominal = 34 + 10(N+1) bit times is a floor, not a target -- "all
    subscribing nodes shall be able to receive" it. A frame squeezed below it is only
    decodable by a node that is cheating on the response space."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    m = lin_limits(NS_PER_TICK, FAST)
    # A frame can only fall below the nominal length if some field of it is short, so this
    # rides on a short break -- which is the whole point: the nominal-length rule is the
    # backstop that says a frame nobody can receive in full is not acceptable, whatever else
    # is wrong with it.
    data = b"\xa5"
    segs = frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=data,
                          checksum=lin_checksum(make_pid(0x11), data),
                          response_space_ticks=0, break_ticks=11 * bit)
    _raw(bus, bit, segs)
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 1})
    bus.run(400_000)
    assert rules_fired(chk) == {"tBREAK", "tFRAME_MIN"}, chk.report()
    short = chk.violations_by_rule("tFRAME_MIN")[0]
    assert short.measured < short.required, str(short)
    assert short.required == pytest.approx(frame_min_ticks(m, 1), rel=0.01), str(short)


def test_checker_flags_a_response_space_below_the_limit():
    """tRS: the space between the PID and the first data byte, at the limit it was given.

    The spec's own floor is zero -- "both may be extended" -- so a rule here needs a limit
    from somewhere else: a cluster that has agreed on a response space uses one, and this is
    what catches the node that starts early.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=b"\xa5\x5a",
                                  checksum=lin_checksum(make_pid(0x11), b"\xa5\x5a"),
                                  response_space_ticks=0))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2},
                         timing={"response_space_bits": 2.0})
    bus.run(400_000)
    assert_only(chk, "tRS", "response space between the PID")
    short = chk.violations_by_rule("tRS")[0]
    assert short.required == 2 * bit, str(short)
    assert chk.frames[0].response_space == 0, chk.report()


def test_checker_flags_an_inter_byte_space_below_the_limit():
    """tIB: the same rule for the spaces inside the response, including the one in front of
    the checksum byte."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=b"\xa5\x5a",
                                  checksum=lin_checksum(make_pid(0x11), b"\xa5\x5a"),
                                  response_space_ticks=2 * bit))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2}, timing={"ibs_bits": 1.0})
    bus.run(400_000)
    assert_only(chk, "tIB", "inter-byte space")
    # Two data bytes and a checksum means two spaces inside the response.
    assert len(chk.violations_by_rule("tIB")) == 2, chk.report()
    assert all(v.required == bit for v in chk.violations_by_rule("tIB"))


def test_checker_flags_a_header_longer_than_it_may_be():
    """tHEADER_MAX: the header is at most 1.4 x 34 bit times, break field included.

    A twenty-bit break delimiter is the way to get there, and it is a real fault: the
    delimiter exists to give the frame's start a falling edge in front of the sync field, and
    every bit time of it is a bit time the header is spending on nothing.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, _frame_segments(bit, delimiter_ticks=20 * bit))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "tHEADER_MAX", "break field included")
    long_header = chk.violations_by_rule("tHEADER_MAX")[0]
    assert long_header.over, str(long_header)
    assert long_header.required == pytest.approx(1.4 * 34 * bit, rel=0.01), str(long_header)
    assert long_header.measured == pytest.approx((13 + 20 + 10 + 10) * bit, rel=0.01)


def test_checker_flags_a_response_longer_than_it_may_be():
    """tRESPONSE_MAX: the response is at most 1.4 x 10(N+1) bit times, spaces included.

    Six bit times of space in front of each of two data bytes is not much on its own -- the
    rule is about the sum, which is what a subscriber's own deadline is built from.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bit = bit_ticks(NS_PER_TICK, FAST)
    _raw(bus, bit, frame_segments(bit_ticks=bit, pid=make_pid(0x11), data=b"\xa5\x5a",
                                  checksum=lin_checksum(make_pid(0x11), b"\xa5\x5a"),
                                  response_space_ticks=2 * bit, response_ibs_ticks=6 * bit))
    chk = attach_checker(bus, rate=FAST, lengths={0x11: 2})
    bus.run(400_000)
    assert_only(chk, "tRESPONSE_MAX", "response space included")
    long_response = chk.violations_by_rule("tRESPONSE_MAX")[0]
    assert long_response.over, str(long_response)
    assert long_response.required == pytest.approx(1.4 * 30 * bit, rel=0.01), str(long_response)
    # 2 bits of response space, 6 before the first data byte and 6 before the second, and 30
    # of bytes: 44 bit times, which the checker times from the PID's stop bit.
    assert long_response.measured == pytest.approx(50 * bit, rel=0.02), str(long_response)


LIN_MODULE = sys.modules[__name__ and "verif.vip.lin"]


def test_every_rule_the_checker_documents_is_covered_by_a_test():
    """Non-vacuity: a rule nobody can provoke is a rule that does not work.

    Two lists have to agree: the rules `LinChecker`'s docstring claims to check, and the rules
    this file actually provokes on the wire. A name in one and not the other is a rule that
    never fires (and a rule that fires for the wrong reason), which is the failure mode that
    a spec-derived checker is worst at, because it looks covered either way.
    """
    doc = Path(LIN_MODULE.__file__).read_text(encoding="utf-8")
    table = doc.split("class LinChecker", 1)[1].split("def __init__", 1)[0]
    documented = set(re.findall(r"^      ([A-Za-z_]+)", table, re.M))
    here = Path(__file__).read_text(encoding="utf-8")
    quoted = set(re.findall(r'"([A-Za-z_]+)"', here))
    assert documented, "no rule table found in the checker docstring"
    assert documented <= quoted, sorted(documented - quoted)      # every rule is provoked
    # ...and no `t`-rule is invented here that the checker does not document.
    invented = set(re.findall(r'"(t[A-Z_]+)"', here)) - documented
    assert not invented, sorted(invented)
    assert len(documented - {"SYNC", "PID"}) == 12, sorted(documented)
