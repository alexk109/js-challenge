"""End-to-end test of the MVP LIN firmware (task B3-FW-lin).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with the
18 protocol pin nets `p0`..`p17`, exactly as `docs/mvp-interfaces.md` `## ISS API` describes,
the host driver loads the program and puts frames on the wire, and `verif.vip.lin` -- the
independent judge, which this test does not modify -- answers them and checks them.

The judge is three devices from one module:

* `LinResponder` is the other node on the bus.  It has no schedule table and no knowledge of
  the chip: it finds the break field, measures the commander's bit period off the sync field,
  decodes the protected identifier, checks its parity and -- if it is subscribed -- publishes
  its own data bytes and the checksum the identifier calls for.  A commander that puts a
  malformed header on the wire simply goes unanswered.
* `LinChecker` is a passive monitor that decodes the wire and checks twelve rules, from
  `tBREAK` and `tDELIM` through `tBIT`, `SYNC`, `tSTOP`, `PID`, `tDATALEN`, `tCS`, `tRS`,
  `tIB`, `tRESPONSE`, `tFRAME_MIN` to `tHEADER_MAX` and `tRESPONSE_MAX`.  **Zero violations is
  the bar**: a frame that is too short, too long, mis-framed, mis-protected or mis-checksummed
  is a failure, and every miss carries the tick it was missed on.
* `LinCommander` is not used here: this chip *is* the commander.  `fw/lin/README.md` says why
  that is the side the MVP can play.

    make fw PROTO=lin TARGET=iss        # runs
    FW_TARGET=iss_b python -m pytest -q fw/lin     # the same bench on the second ISS

The models are attached to the chip's own pin nets (`attach_checker(bus, net="p0")`), so the
wire is the pin: there is no coupling device and no extra tick of latency anywhere.  The nets
are open drain with a pull-up (`verif/vip/lin.py` `_ensure_net`), which is also what the
firmware assumes -- the program drives the pin's *direction* and never its level
(`docs/mvp-isa.md` `### Open drain`), so a bus that a hard-driven high would flatten shows up
here as `bus.errors` or as a checker that never sees a dominant level.

Nothing in this file decides pass or fail from the ISS internals: the frame the chip sent is
read off the wire by `LinChecker`, the answer off the wire by `LinResponder`.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field

import pytest

import fw.lin.driver as driver_mod
from fw import targets
from fw.lin.driver import (
    CORE_HZ,
    LinDriver,
    clkdiv_for_rate,
    clkdiv_value,
    decode,
    frame_words,
    jmp_cond,
    jmp_target,
    load_program,
    make_pid,
    parity_bits,
    program_timing,
    ticks_per_bit,
    wire_bytes,
    wire_word,
    words_used,
)
from iss.pio_iss import NUM_PINS, ChipDevice
from verif.vip import lin as vip
from verif.vip.core import Bus
from fw import regmap as _regmap

TICK_NS = 20.0                    # 50 MHz, the clock every model here is built for
IMEM_SIZE = 32                    # docs/mvp-isa.md `### Engine state`
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_TXSTALL = 0x0100_0000      # bit 24 + engine (docs/mvp-regmap.md `**FDEBUG**`)
FDEBUG_BAD = 0x0003_0303          # TXOVER, RXUNDER, RXSTALL: nothing dropped or mis-timed
UIO_MASK = 0x000FF                # the bidirectional protocol pins
OEO_MASK = 0x03E000              # the output-only pins 13..17, always driven by the chip
FAST = 19200.0                    # the rate most tests run at
TABLE = {0x11: b"\xa5\x5a", 0x22: b"\x01\x02\x03\x04"}      # identifier -> payload


def rules_fired(chk) -> set:
    """The rules the checker fired, for a "this must be caught" assertion."""
    return {v.rule for v in chk.violations}


@dataclass
class Bench:
    """A bus with the chip as commander, a responder and a checker, all on one wire.

    `publish` is the responder's identifier-to-payload map and `lengths` the LDF the checker is
    told about.  `clkdiv` overrides the divisor the driver would derive from `rate`, and
    `program` replaces the one `program.pio` assembles to: both exist so that the negative
    controls at the bottom of this file can put a *broken* commander on the wire and check that
    the judge notices, which is the only way to tell a passing checker from a silent one.
    """
    rate: float = FAST
    pin: int = 0
    engine: int = 0
    version: str = "2.2"
    program: object = None
    clkdiv: int | None = None
    publish: dict | None = None
    lengths: dict | None = None
    chk_timing: dict | None = None
    chk_version: str | None = None
    responder: bool = True
    check: bool = True
    bus: Bus = field(init=False)
    dev: ChipDevice = field(init=False)
    drv: LinDriver = field(init=False)
    rsp: object = field(init=False, default=None)
    chk: object = field(init=False, default=None)
    sent: list[tuple[int, bytes | None]] = field(init=False, default_factory=list)

    def __post_init__(self) -> None:
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(NUM_PINS):                 # p0..p17, open drain and pulled up
            self.bus.add_net(f"p{k}")
        self.dev = self.bus.attach(ChipDevice())
        net = f"p{self.pin}"
        if self.responder:
            self.rsp = vip.attach_responder(self.bus, rate=self.rate, version=self.version,
                                            net=net, publish=self.publish or {})
        if self.check:
            self.chk = vip.attach_checker(self.bus, rate=self.rate,
                                          version=self.chk_version or self.version,
                                          net=net, lengths=self.lengths,
                                          timing=self.chk_timing)
        self.drv = LinDriver(self.dev, self.bus, pin=self.pin, engine=self.engine,
                             rate=self.rate, version=self.version, program=self.program)
        self.drv.load(self.clkdiv)

    # ---- driving ----
    def frame(self, frame_id: int, data=None, version: str | None = None) -> list[int]:
        """Put one frame on the wire and remember the words that went into the TX FIFO."""
        words = self.drv.start(frame_id, data, version)
        self.sent.append((frame_id, None if data is None else bytes(data)))
        return words

    def settle_ticks(self) -> int:
        """Idle time to leave after a frame, in core cycles.

        Four bit times and a little: with an LDF entry the checker closes a frame when its
        checksum byte arrives, and the responder starts one bit time after the PID's stop bit,
        so this is only slack.  It is derived from the *nominal* bit rate rather than from the
        program, so a mutated program still gets a fair amount of wire.
        """
        return int(6 * CORE_HZ / self.rate) + 64

    def run(self, n_data: int | None = None, extra_ticks: int | None = None) -> None:
        """Run the bus long enough for the frame and its response to be over.

        `n_data` is how many data bytes the wire is expected to carry after the header, so the
        same argument covers both roles: the chip's own published response and the responder's
        answer are the same length in byte fields.  `None` means a header-only frame and no
        answer, which is what a bench with no subscriber runs.
        """
        total = self.drv.frame_ticks(n_data)
        self.bus.run(total + (self.settle_ticks() if extra_ticks is None else extra_ticks))

    # ---- what the judge saw ----
    def frames(self) -> list:
        assert self.chk is not None, "this bench has no checker"
        return self.chk.frames

    def clean(self) -> None:
        assert self.chk is not None and self.chk.clean, self.chk.report()

    def report(self) -> str:
        return self.chk.report() if self.chk is not None else "(no checker on this bench)"

# --------------------------------------------------------------------------------------------
# The transaction, judged by verif/vip/lin.py
# --------------------------------------------------------------------------------------------

def nominal_bit_ticks(rate: float = FAST) -> int:
    """One bit time in ticks at `rate`, the way the models round it."""
    return vip.bit_ticks(TICK_NS, rate)


@pytest.fixture(scope="module")
def answered() -> Bench:
    """A header from the chip at 19.2 kbit/s, answered by `LinResponder` with two bytes.

    The interesting configuration, because nothing about it is inside the chip: a second node
    on the wire has to find the break field, measure the commander's bit period off the sync
    field, decode the protected identifier, check its parity and only then answer.
    """
    bench = Bench(rate=FAST, publish={0x11: TABLE[0x11]}, lengths={0x11: len(TABLE[0x11])})
    bench.frame(0x11)
    bench.run(len(TABLE[0x11]))
    return bench


@pytest.fixture(scope="module")
def published() -> Bench:
    """The chip publishes the whole frame: eight data bytes and the enhanced checksum.

    `publish={}` still attaches a responder, subscribed to nothing: it reads the header, finds
    that the identifier is not its own, and stays off the wire.  That is what a commander on a
    bus with other nodes looks like when it publishes, and it is checked below.
    """
    payload = bytes(range(1, 9))
    bench = Bench(rate=FAST, publish={}, lengths={0x22: len(payload)})
    bench.frame(0x22, payload)
    bench.run(len(payload))
    return bench


def test_a_header_from_the_chip_is_answered_and_the_checker_is_silent(answered: Bench) -> None:
    """The whole exchange, read off the wire: no rule broken, the right bytes answered."""
    chk, rsp, drv = answered.chk, answered.rsp, answered.drv
    assert chk.clean, chk.report()

    # The monitor saw one frame: break field, delimiter, sync, PID, two data bytes, checksum.
    assert chk.breaks == 1 and len(chk.frames) == 1, chk.report()
    assert chk.bytes == 5, f"{chk.bytes} byte fields: {chk.report()}"
    frame = chk.frames[0]
    assert frame.frame_id == 0x11 and frame.sync == vip.SYNC_BYTE
    assert frame.pid == make_pid(0x11) and frame.parity_ok, str(frame)
    assert frame.n_data == 2 and frame.data == TABLE[0x11], str(frame)
    assert frame.checksum_model == "enhanced" and frame.checksum_ok, str(frame)

    # Every byte field on the wire, the chip's and the responder's, is ten bit times long, and
    # the bit rate is the nominal one to within the tick the models round to.
    bit = nominal_bit_ticks()
    assert chk.byte_times and all(b == 10 * bit for b in chk.byte_times), \
        f"byte fields {chk.shortest_byte}..{chk.longest_byte} ticks, 10 bit times is {10 * bit}"
    assert chk.min_bit_hz == chk.max_bit_hz == pytest.approx(FAST, rel=1e-4), chk.report()

    # The header is exactly as long as the program says it is, break field included: the
    # driver's `program_timing()` read those bit times out of the assembled words, so this
    # compares the wire against the program rather than against a number typed in here.
    timing = drv.timing
    assert frame.header_ticks == pytest.approx(timing.header_bit_times() * bit, abs=2 * bit / 100)
    assert frame.response_space == pytest.approx(bit, abs=bit // 8), (
        f"the responder starts one bit time after the PID's stop bit, got "
        f"{frame.response_space} ticks against {bit}")

    # The responder's own account: it decoded the header, measured our bit period off the sync
    # field to the tick, passed the parity test and published the enhanced checksum.
    assert [tx.frame_id for tx in rsp.frames_seen] == [0x11], rsp.frames_seen
    assert rsp.parity_errors == [] and rsp.ignored == [], (rsp.parity_errors, rsp.ignored)
    assert rsp.bit_now == bit and rsp.bit_error_ticks == 0, (
        f"the responder measured {rsp.bit_now} ticks per bit against the {bit} it was built "
        f"for: the sync field is what carries the bit timing")
    assert rsp.published and rsp.published[0][1] == 0x11, rsp.published
    _, frame_id, payload, model, csum = rsp.published[0]
    assert payload == TABLE[0x11] and model == "enhanced" and csum == frame.checksum
    assert vip.checksum_ok(frame.pid, payload, csum, enhanced=True)


def test_the_chip_publishes_its_own_response(published: Bench) -> None:
    """Eight data bytes and the checksum, off the chip's own engine, with no rule broken."""
    chk, rsp, drv = published.chk, published.rsp, published.drv
    payload = bytes(range(1, 9))
    assert chk.clean, chk.report()
    assert chk.breaks == 1 and len(chk.frames) == 1, chk.report()
    assert chk.bytes == 11, f"2 header + 8 data + 1 checksum, got {chk.bytes}: {chk.report()}"
    frame = chk.frames[0]
    assert frame.frame_id == 0x22 and frame.parity_ok and frame.n_data == 8, str(frame)
    assert frame.data == payload, f"{frame.data.hex()} != {payload.hex()}"
    assert frame.checksum_model == "enhanced" and frame.checksum_ok, str(frame)
    # The chip's checksum is the one the spec asks for, and it is the byte the wire carried.
    assert frame.checksum == driver_mod.lin_checksum(frame.pid, payload, enhanced=True)

    # The two bit times of response space are what the program leaves between the PID's stop
    # bit and the first data byte's start bit.  The checker times that gap from its own bit
    # grid, which is re-armed at each byte field, so it carries a couple of ticks of grid
    # slack -- which is why the bar is a tolerance and not an equality.
    bit = nominal_bit_ticks()
    assert frame.response_space == pytest.approx(drv.timing.response_space_bit_times * bit,
                                                abs=4), str(frame)

    # The frame is the length the program says it is, and it is above the spec's floor.
    assert frame.end_tick - frame.start_tick == pytest.approx(
        drv.timing.frame_bit_times(len(payload)) * bit, abs=2 * bit / 100)
    assert chk.violations_by_rule("tFRAME_MIN") == [], chk.report()

    # A responder was on the wire, read the header, and kept quiet because the identifier was
    # not one it is subscribed to: publishing and answering are not both done at once.
    assert [tx.frame_id for tx in rsp.frames_seen] == [0x22]
    assert rsp.published == [], "the chip published, so no responder may answer"
    assert [why for _t, _f, why in rsp.ignored] == ["not subscribed"], rsp.ignored


def test_a_schedule_of_two_frames_back_to_back() -> None:
    """Two identifiers in a row, each answered, and one identifier nobody subscribed to."""
    bench = Bench(rate=FAST, publish={0x11: TABLE[0x11], 0x33: b"\xde\xad"},
                  lengths={0x11: 2, 0x22: 4, 0x33: 2})
    bench.frame(0x11)
    bench.run(len(TABLE[0x11]))
    bench.frame(0x22)
    bench.run(len(TABLE[0x22]))          # 0x22 is published by nobody: tRESPONSE would fire
    bench.frame(0x33)
    bench.run(2)
    chk = bench.chk
    # A frame nobody answered trips three rules, and all three are the same fact said three
    # ways: no response (tRESPONSE), no data bytes in it (tDATALEN), and a frame that ends at
    # the PID's stop bit is shorter than T_Frame_Nominal for a 4-byte payload (tFRAME_MIN).
    # Nothing else fires: the header itself is a legal header.
    assert rules_fired(chk) == {"tRESPONSE", "tDATALEN", "tFRAME_MIN"}, chk.report()
    assert len(chk.frames) == 3 and [f.frame_id for f in chk.frames] == [0x11, 0x22, 0x33]
    for frame in (chk.frames[0], chk.frames[2]):
        assert frame.parity_ok and frame.checksum_ok and not frame.violations, str(frame)
    assert chk.frames[1].violations and "no response" in chk.report()
    assert [f for _t, f, _d, _m, _c in bench.rsp.published] == [0x11, 0x33]
    assert len(bench.rsp.frames_seen) == 3, bench.rsp.frames_seen


@pytest.mark.parametrize("rate", [9600.0, 10000.0, 19200.0, 20000.0])
def test_every_bit_rate_the_chip_reaches_is_bit_exact(rate: float) -> None:
    """The whole rate table, at 50 MHz: one SM cycle per bit, so CLKDIV alone sets the rate."""
    data = b"\x5a"
    bench = Bench(rate=rate, publish={0x11: data}, lengths={0x11: 1})
    bench.frame(0x11)
    bench.run(1)
    bit = nominal_bit_ticks(rate)
    assert bench.chk.clean, bench.chk.report()
    assert bench.chk.breaks == 1 and len(bench.chk.frames) == 1, bench.chk.report()
    frame = bench.chk.frames[0]
    assert frame.data == data and frame.checksum_ok and frame.parity_ok, str(frame)
    # Every byte field is ten bit times of *this* rate: the divider is what sets it, and the
    # 16.8 rounding is a thousandth of a per cent at every rate in the table.
    assert all(b == 10 * bit for b in bench.chk.byte_times), \
        f"{bench.chk.shortest_byte}..{bench.chk.longest_byte} ticks, expected {10 * bit}"
    assert bench.chk.min_bit_hz == bench.chk.max_bit_hz == pytest.approx(rate, rel=1e-4)
    assert bench.rsp.bit_now == bit, f"the responder measured {bench.rsp.bit_now}, wanted {bit}"
    assert bench.drv.ticks_per_bit() == pytest.approx(CORE_HZ / rate, rel=1e-4)


@pytest.mark.skipif(not targets.is_fast(), reason="a 2.4 kbit/s frame is 1.2 million ticks")
def test_the_slowest_rate_in_the_table() -> None:
    """2.4 kbit/s, the low end of the range a cluster auto-bauds over, in one short frame.

    Nothing about this rate is special to the program -- one SM cycle per bit is one SM cycle
    per bit -- and that is the point: the divisor reaches 20833, where the 16-bit integer part
    of CLKDIV is still only a third of the way to its maximum.
    """
    data = b"\x81"
    bench = Bench(rate=vip.LIN_RATES[0], publish={0x11: data}, lengths={0x11: 1})
    bench.frame(0x11)
    bench.run(1)
    bit = nominal_bit_ticks(vip.LIN_RATES[0])
    assert bench.chk.clean, bench.chk.report()
    assert bench.chk.frames[0].data == data and bench.chk.frames[0].checksum_ok
    assert all(b == 10 * bit for b in bench.chk.byte_times), bench.chk.report()
    assert bench.chk.min_bit_hz == bench.chk.max_bit_hz == pytest.approx(vip.LIN_RATES[0],
                                                                       rel=1e-4)
    assert bench.rsp.bit_now == bit


def test_engine_one_and_another_pin() -> None:
    """The same program on engine 1 and a different uio pin, with the other engine idle."""
    bench = Bench(rate=FAST, pin=5, engine=1, publish={}, lengths={0x11: 2})
    bench.frame(0x11, b"\x01\x02")
    bench.run(2)
    assert bench.chk.clean, bench.chk.report()
    assert bench.chk.frames[0].data == b"\x01\x02", str(bench.chk.frames[0])
    chip = bench.dev.chip
    assert chip.engines[0].pc == 0, "the other engine ran"
    assert not chip.engines[0].tx, "the other engine got data"
    padoe = bench.drv.padoe()
    assert not padoe & (1 << 5), f"padoe {padoe:#06x}: the LIN pin is still driven"
    assert padoe & OEO_MASK == OEO_MASK, "the output-only pins are always driven"


def test_the_checksum_model_follows_the_identifier_and_the_cluster() -> None:
    """0x3C and 0x3D are classic even on a 2.2 cluster; a 1.3 cluster is classic everywhere."""
    payload = b"\x6c\xf1"
    bench = Bench(rate=FAST, publish={}, lengths={0x3C: 2})
    bench.frame(0x3C, payload)
    bench.run(2)
    assert bench.chk.clean, bench.chk.report()
    frame = bench.chk.frames[0]
    assert frame.checksum_model == "classic" and frame.checksum_ok, str(frame)
    assert frame.checksum == driver_mod.lin_checksum(frame.pid, payload, enhanced=False)

    bench13 = Bench(rate=FAST, version="1.3", publish={}, lengths={0x11: 2})
    bench13.frame(0x11, payload)
    bench13.run(2)
    assert bench13.chk.clean, bench13.chk.report()
    frame13 = bench13.chk.frames[0]
    assert frame13.checksum_model == "classic" and frame13.checksum_ok, str(frame13)


def test_a_cluster_that_agreed_on_a_response_space_gets_one() -> None:
    """The two bit times of response space satisfy a checker told to want two.

    The spec's own floor for the space is zero ("both of them must be non-negative"), so the
    default checker cannot tell a node that starts early from one that does not.  A cluster that
    has agreed on a space can, and this is the firmware answering that question.
    """
    payload = b"\x01\x02"
    bench = Bench(rate=FAST, publish={}, lengths={0x11: 2},
                  chk_timing={"response_space_bits": 2.0})
    bench.frame(0x11, payload)
    bench.run(2)
    assert bench.chk.clean, bench.chk.report()
    space = bench.drv.timing.response_space_bit_times
    assert space == 2, space


# --------------------------------------------------------------------------------------------
# What the program and the divisor are, derived from the words rather than asserted by hand
# --------------------------------------------------------------------------------------------

def test_program_fits_the_imem() -> None:
    """program.pio assembles, and the driver reads its shape back out of the words."""
    program = load_program()
    assert program.name == "lin"
    assert program.sideset_bits == 2 and program.sideset_opt and program.sideset_pindirs, (
        ".side_set 1 opt pindirs is 2 bits, one enable plus one data bit, and the data bit is "
        "a *direction*: that is the open-drain sense and the `opt` bit is what lets a data "
        "bit's instruction leave the pin alone")
    assert words_used(program) == 26, [f"{w:#06x}" for w in program.instrs]
    assert words_used(program) <= IMEM_SIZE, "the MVP has 32 words of IMEM per engine"
    assert program.labels == {"frame": 0, "hdr": 3, "idle": 14, "resp": 15}, program.labels
    assert program.wrap == program.wrap_target == program.labels["idle"], \
        "the wrap window is the idle loop, so an engine that wraps cannot start a frame"

    # The frame's shape, read out of the assembled words by `program_timing`:
    timing = program_timing(program)
    assert timing.sm_cycles_per_bit == 1, "one SM cycle per bit: CLKDIV alone sets the rate"
    assert timing.break_bit_times == 14, "8 + 6 SM cycles: the 13 the spec asks for, plus one"
    assert timing.delimiter_bit_times == 2, "the spec floor is one bit time"
    assert timing.field_bit_times == 10, "start bit, eight data bits, stop bit"
    assert timing.response_space_bit_times == 2
    assert timing.header_bit_times() == 36, "14 + 2 + two byte fields"
    assert timing.frame_bit_times(None) == 36, "a header-only frame ends at the PID's stop bit"
    assert timing.frame_bit_times(8) == 36 + 2 + 9 * 10 == 128

    # Every bit instruction is `out pindirs, 1` with no side-set operand, and there are eight
    # of them per byte field: that is what "one instruction per data bit" means in the words.
    for name, start in (("hdr", program.labels["hdr"]), ("resp", program.labels["resp"])):
        block = program.instrs[start:start + 10]
        words = [decode(program, w) for w in block]
        assert driver_mod.is_pull_block(words[0]) and words[0].side_data, f"{name}: start bit"
        data = words[1:9]
        assert all(driver_mod.is_out_pindirs_1(w) and not w.side_enabled for w in data), \
            f"{name}: the data bits are `out pindirs, 1` with the side-set field zero, so the " \
            f"OUT channel owns the pin (side-set wins within a cycle)"
        assert all(w.delay == 0 for w in data), f"{name}: one SM cycle per data bit"
        stop = words[9]
        assert stop.op == 0b000 and not stop.side_data, f"{name}: the stop bit is recessive"
        assert jmp_cond(stop) in (0b010, 0b100), f"{name}: the stop bit is the counter branch"
        assert jmp_target(stop) == start, f"{name}: and it branches back to the field's start"


def test_max_speed_on_the_iss() -> None:
    """The fastest the program can go: CLKDIV 1.0 and one SM cycle per instruction.

    A bit is one SM cycle, CLKDIV = 1.0 puts an SM cycle every core cycle, and 50 MHz gives
    50 Mbit/s -- 2500 times LIN's own 20 kbit/s ceiling, and about the ceiling of what a PIO
    can shift out at all (the RP2040's `out pins, 1` loop with no delay is the same number).
    Nothing above that is reachable without changing the program, because a delay is counted
    in SM cycles and CLKDIV cannot go below one core cycle per SM cycle.
    """
    timing = program_timing(load_program())
    fastest = ticks_per_bit(clkdiv_value(1), timing.sm_cycles_per_bit)
    assert fastest == 1.0 and timing.sm_cycles_per_bit == 1
    assert CORE_HZ / fastest == 50_000_000.0
    assert clkdiv_for_rate(CORE_HZ / fastest, timing.sm_cycles_per_bit) == clkdiv_value(1)
    with pytest.raises(ValueError):
        clkdiv_for_rate(CORE_HZ / fastest * 1.01, timing.sm_cycles_per_bit)

    # The LIN-relevant end of the same ladder: 20 kbit/s is 2500 ticks per bit and needs an
    # integer divisor of 2500 exactly, so the fastest legal LIN rate is reached with no
    # rounding at all.
    top = vip.LIN_RATES[-1]
    assert clkdiv_for_rate(top) == clkdiv_value(2500)
    assert ticks_per_bit(clkdiv_for_rate(top)) == CORE_HZ / top


@pytest.mark.parametrize("rate", list(vip.LIN_RATES) + [4000.0, 12500.0, 33333.0, 50000.0])
def test_clkdiv_hits_the_rate_within_the_divider_resolution(rate: float) -> None:
    """CLKDIV for a rate is the 16.8 rounding, and CLKDIV is all the program has to go on."""
    clkdiv = clkdiv_for_rate(rate)
    integer, frac = (clkdiv >> 16) & 0xFFFF, (clkdiv >> 8) & 0xFF
    assert 1 <= integer <= 0xFFFF and 0 <= frac <= 0xFF
    # One bit is one SM cycle and one SM cycle is integer + frac/256 core cycles, so the bit
    # length can only move in steps of 1/256 tick: half a step, rounded.
    assert abs(ticks_per_bit(clkdiv) - CORE_HZ / rate) <= 0.5 / 256 + 1e-9
    # ... and the error is orders of magnitude inside FTOL_SYNC's 1.5 % for every LIN rate.
    assert abs(CORE_HZ / ticks_per_bit(clkdiv) - rate) / rate < 1e-3
    for bad in (0.0, -19200.0):
        with pytest.raises(ValueError):
            clkdiv_for_rate(bad)


# --------------------------------------------------------------------------------------------
# The protocol arithmetic: this firmware's own copy against the judge's, and the spec's vectors
# --------------------------------------------------------------------------------------------

def test_the_two_pid_implementations_agree_with_the_specs_own_vectors() -> None:
    """P0 and P1 are LIN 2.2A eq. (1) and (2), computed here and in the checker separately.

    The first eight protected identifiers below are the ones the LIN literature publishes for
    identifiers 0..7 (80, C1, 42, 03, C4, 85, 06, 47), so a sign error in either
    implementation cannot hide behind the other.
    """
    assert [make_pid(i) for i in range(8)] == [0x80, 0xC1, 0x42, 0x03,
                                              0xC4, 0x85, 0x06, 0x47]
    assert parity_bits(0x11) == (0, 0), "P0 = ID0 ^ ID4, P1 = ~(ID4)"
    assert parity_bits(0x3F) == (0, 1), "every ID bit set: P0 = four ones = 0, P1 = ~0 = 1"
    for frame_id in range(0x40):
        p0, p1 = parity_bits(frame_id)
        assert make_pid(frame_id) == frame_id | (p0 << 6) | (p1 << 7)
        assert make_pid(frame_id) == vip.make_pid(frame_id), hex(frame_id)
        assert parity_bits(frame_id) == vip.parity_bits(frame_id), hex(frame_id)
        assert vip.pid_parity_ok(make_pid(frame_id)), hex(frame_id)
    for bad in (-1, 0x40, 0x100):
        with pytest.raises(ValueError):
            make_pid(bad)


def test_the_two_checksum_implementations_agree_with_the_specs_own_vectors() -> None:
    """Both models: an inverted modulo-256 sum, and the receiver's `sum == 0xFF` test.

    {0xA5, 0x5A} sums to 0xFF, so its classic checksum is 0x00 and the enhanced one is
    ~0x10 = 0xEF for PID 0x11 -- two different answers from the same data, which is the whole
    difference between the models.  The carry case matters: {0xFF, 0x02} sums to 0x101, and
    modulo 256 that is 0x01.
    """
    pid = make_pid(0x11)
    assert driver_mod.lin_checksum(pid, b"\xa5\x5a", enhanced=False) == 0x00
    assert driver_mod.lin_checksum(pid, b"\xa5\x5a", enhanced=True) == 0xEF
    assert driver_mod.lin_checksum(0x00, b"\xff\x02", enhanced=False) == 0xFE
    for data in (b"\x00", b"\xff", b"\x01\x02\x03\x04", bytes(range(1, 9)), bytes([0xFF] * 8)):
        for frame_id in (0x00, 0x11, 0x3C, 0x3F):
            the_pid = make_pid(frame_id)
            for enhanced in (False, True):
                got = driver_mod.lin_checksum(the_pid, data, enhanced=enhanced)
                assert got == vip.lin_checksum(the_pid, data, enhanced=enhanced), \
                    f"{the_pid:#04x} {data.hex()}"
                assert vip.checksum_ok(the_pid, data, got, enhanced=enhanced)


def test_the_checksum_model_follows_the_identifier_and_the_cluster() -> None:
    """0x3C and 0x3D are classic even on 2.2; a 1.3 cluster is classic everywhere."""
    for frame_id in range(0x40):
        for version in driver_mod.CLUSTER_VERSIONS:
            assert driver_mod.checksum_model(frame_id, version) == \
                vip.checksum_model(frame_id, version), (hex(frame_id), version)
    assert all(driver_mod.checksum_model(i, "2.2") == "enhanced" for i in range(0x3C))
    assert all(driver_mod.checksum_model(i, "1.3") == "classic" for i in range(0x40))
    with pytest.raises(ValueError):
        driver_mod.checksum_model(0x11, "lin-9.9")


def test_the_pushed_words_are_the_wire_bytes_inverted() -> None:
    """What the host pushes is the byte with its bits inverted, and nothing else.

    This is the whole open-drain contract between the host and the program: `out pindirs, 1`
    writes a direction, 1 drives the line to the level register's reset value 0, and 0 is a
    dominant LIN level.  Undoing the inversion is the receiver's job, and the checker reading
    `a55a` back off the wire in the tests above is what proves it happened.
    """
    assert wire_word(0x00) == 0xFF and wire_word(0xFF) == 0x00
    assert wire_word(0x55) == 0xAA and wire_word(0x11) == 0xEE
    words = frame_words(0x11, b"\xa5\x5a")
    assert wire_bytes(words) == b"\x55" + bytes([make_pid(0x11)]) + b"\xa5\x5a" \
        + bytes([driver_mod.lin_checksum(make_pid(0x11), b"\xa5\x5a", enhanced=True)])
    assert len(words) == 5, "sync, PID, two data bytes, checksum"
    assert frame_words(0x11) == words[:2], "no data means header only"
    assert len(frame_words(0x22, bytes(8))) == 11, "2 header + 8 data + checksum"
    for bad in (256, -1):
        with pytest.raises(ValueError):
            wire_word(bad)
    for bad in (b"", bytes(9)):
        with pytest.raises(ValueError):
            frame_words(0x11, bad)


def test_driver_rejects_setups_it_cannot_honour() -> None:
    """A LIN pin PINDIRS cannot drive, an impossible rate, an overfull preloaded FIFO."""
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(NUM_PINS):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for pin in (8, 12, 13, 17, 18):        # input-only, output-only, out of range
        with pytest.raises(ValueError):
            LinDriver(dev, bus, pin=pin)
    with pytest.raises(ValueError):
        LinDriver(dev, bus, engine=2)
    for bad in (0.0, -9600.0):
        with pytest.raises(ValueError):
            clkdiv_for_rate(bad)
    with pytest.raises(ValueError):
        clkdiv_for_rate(CORE_HZ * 1.01)   # one SM cycle per bit cannot beat one core cycle
    with pytest.raises(ValueError):
        clkdiv_value(0, 1)
    with pytest.raises(ValueError):
        clkdiv_value(1, 256)
    with pytest.raises(ValueError):
        driver_mod.jmp_abs(32)
    with pytest.raises(ValueError):
        driver_mod.set_abs(0b011, 1)      # SET cannot write ISR
    with pytest.raises(ValueError):
        driver_mod.set_abs(driver_mod.SET_DEST_X, 32)

    drv = LinDriver(dev, bus, pin=0, rate=FAST)
    drv.load()
    with pytest.raises(ValueError):
        drv.preload([1, 2, 3, 4, 5])
    with pytest.raises(ValueError):
        drv.push(256)
    with pytest.raises(ValueError):
        drv.arm(response_fields=32)


def test_program_timing_refuses_a_program_it_cannot_time() -> None:
    """The driver will not derive a bit rate from a program whose shape it cannot describe.

    Each mutation is a way the firmware could be wrong in one word; the refusal is the point,
    because a driver that guessed would put a wrong waveform on the wire instead.
    """
    program = load_program()

    def mutate(index: int, delta: int):
        bad = copy.deepcopy(program)
        bad.instrs[index] += delta
        return bad

    # A data bit with a side-set operand: side-set wins over the OUT channel in the same cycle,
    # so the OSR bit would never reach the pin.
    with pytest.raises(ValueError, match="side-set wins"):
        program_timing(mutate(4, 1 << 11))
    # A byte field with seven data bits, and with the field's own loop branch replaced.
    short = copy.deepcopy(program)
    short.instrs.pop(5)
    with pytest.raises(ValueError, match="8 data bits"):
        program_timing(short)
    with pytest.raises(ValueError, match="stop-bit branch"):
        program_timing(mutate(12, -1))
    # A break field below the 13 bit times no responder can see, and a missing delimiter.
    with pytest.raises(ValueError, match="break field is"):
        program_timing(mutate(1, -3 << 8))
    no_delim = copy.deepcopy(program)
    no_delim.instrs[2] = 0x6081          # an `out pindirs, 1` where the delimiter belongs
    with pytest.raises(ValueError, match="break delimiter is"):
        program_timing(no_delim)
    # A data bit a tick short of the others, so a bit is not a whole number of SM cycles.
    with pytest.raises(ValueError, match="different delays"):
        program_timing(mutate(6, 1 << 8))
    # Words past the end of the program, and a jump back from the idle loop to nowhere.
    extra = copy.deepcopy(program)
    extra.instrs.append(0x0000)
    with pytest.raises(ValueError, match="words follow"):
        program_timing(extra)
    with pytest.raises(ValueError, match="jmp idle"):
        program_timing(mutate(25, 1))
    # A `.side_set` with no room for a data bit once the `opt` enable has taken one, and a
    # side-set of *levels* instead of directions, which would drive the wire high rather than
    # release it: EXECCTRL refuses both rather than start an engine that cannot transmit.
    no_room = copy.deepcopy(program)
    no_room.sideset_bits = 1
    with pytest.raises(ValueError, match="no data bit"):
        driver_mod.execctrl_value(no_room)
    levels = copy.deepcopy(program)
    levels.sideset_pindirs = False
    with pytest.raises(ValueError, match="pindirs"):
        driver_mod.execctrl_value(levels)


# --------------------------------------------------------------------------------------------
# The chip itself: registers, pins, the other engine
# --------------------------------------------------------------------------------------------

def test_the_chip_is_left_clean_and_only_touches_its_own_pin() -> None:
    """No word dropped, no read underflow, no stall, and the open-drain invariant intact."""
    bench = Bench(rate=FAST, pin=0, engine=0, publish={}, lengths={0x11: 2})
    payload = b"\x01\x02"
    bench.frame(0x11, payload)
    ticks = bench.drv.frame_ticks(len(payload))
    # A tenth of the way into the frame is inside the break field (14 of 68 bit times), where
    # the program is holding the wire dominant with PINDIRS: the pin must be driven.
    bench.bus.run(ticks // 10)
    assert bench.drv.padoe() & 1, f"the LIN pin is not driven during the break field"
    bench.run(len(payload))
    drv, chip = bench.drv, bench.dev.chip
    assert bench.chk.clean, bench.chk.report()

    # TXOVER, RXUNDER and RXSTALL would mean a word was dropped or mis-timed.  TXSTALL is the
    # engine parked on `pull block` waiting for a word the host has not pushed yet; this driver
    # primes the FIFO before the break starts and keeps it topped up inside a frame, so a
    # commander with a host behind it must not see it either.
    assert drv.fdebug() == 0, f"FDEBUG {drv.fdebug():#010x}"
    assert drv.fdebug() & FDEBUG_BAD == 0
    assert drv.flevel() == 0, "the FIFO is not empty at the end of a frame"

    # The open-drain invariant: the pin's *level* register was never written, so the program
    # can only ever pull the line low, and it released it when the frame ended.
    assert drv.padout() & 1 == 0, f"padout {drv.padout():#06x}: the level register moved"
    padoe = drv.padoe()
    assert not padoe & 1, f"padoe {padoe:#06x}: the LIN pin is still driven"
    assert padoe & UIO_MASK & ~1 == 0, f"padoe {padoe:#06x}: a window leaked onto another uio pin"
    assert padoe & OEO_MASK == OEO_MASK, "the output-only pins 13..17 are always driven"

    assert drv.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"
    assert drv.pc() == drv.idle_addr, f"PC {drv.pc()}, the engine should be parked in `idle`"
    assert chip.engines[0].pc == drv.idle_addr
    assert chip.engines[1].pc == 0, "the other engine ran"
    assert not chip.engines[1].tx, "the other engine got data"
    assert not bench.bus.errors, f"contention on the wire: {bench.bus.errors[:2]}"


@pytest.mark.parametrize("pin,engine", [(0, 0), (7, 1)])
def test_loaded_what_was_assembled(pin: int, engine: int) -> None:
    """The words in the engine's IMEM are the ones `asm/pioasm.py` produced for program.pio."""
    bench = Bench(rate=FAST, pin=pin, engine=engine, publish={}, lengths={0x11: 1})
    bench.frame(0x11, b"\x5a")
    bench.run(1)
    assert bench.chk.clean, bench.chk.report()
    imem = bench.dev.chip.engines[engine].imem
    assert imem[: bench.drv.words_used] == bench.drv.program.instrs
    assert set(imem[bench.drv.words_used:]) <= {0}, "the driver wrote past the program"


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: a broken firmware must fail the transaction
# --------------------------------------------------------------------------------------------

#: One way the firmware could be wrong, and what has to notice.  `expected` is a rule the
#: checker must fire, or `None` when the evidence is the responder instead: it has to fail to
#: read the header at all.  `field` says whether the chip publishes its own response (so there
#: is a checksum to get wrong) or only sends the header (so there is a responder to answer).
CASE_NO_BREAK = "no break field"
CASE_DELIMITER = "dominant break delimiter"
CASE_STOP = "dominant stop bit"
CASE_SHORT_BREAK = "an 11-bit-time break field"
CASE_NO_BIT = "a data bit removed"
CASE_LEVELS = "out pins instead of out pindirs"
CASE_SYNC = "a sync field that is not 0x55"
CASE_CSUM = "a checksum one bit out"
CASE_MSB_FIRST = "MSB first instead of LSB first"
CASE_SLOW = "a bit period 5 % slow"
CASE_BITTIME = "a data bit one SM cycle long"

SIDESET_DATA_BIT = 1 << 11          # with SIDESET_COUNT = 2, the side-set data bit is bit 11

MUTATIONS = [
    # name, publish?, expected rule, mutate the Program, patch the host
    (CASE_NO_BREAK, False, "tRESPONSE",
     lambda p: p.instrs.__setitem__(0, p.instrs[0] - SIDESET_DATA_BIT), None),
    (CASE_DELIMITER, False, None,
     lambda p: p.instrs.__setitem__(2, p.instrs[2] + SIDESET_DATA_BIT), None),
    (CASE_STOP, False, "tSTOP",
     lambda p: p.instrs.__setitem__(12, p.instrs[12] + SIDESET_DATA_BIT), None),
    (CASE_SHORT_BREAK, False, "tBREAK",
     lambda p: p.instrs.__setitem__(1, p.instrs[1] - (2 << 8)), None),      # 14 -> 11 bit times
    (CASE_NO_BIT, False, "SYNC",
     lambda p: p.instrs.pop(5), None),                                       # seven data bits
    (CASE_LEVELS, False, "SYNC",
     lambda p: p.instrs.__setitem__(4, 0x6001), None),                       # `out pins, 1`
    (CASE_BITTIME, False, "SYNC",
     lambda p: p.instrs.__setitem__(6, p.instrs[6] + (1 << 8)), None),       # 2 SM cycles
    (CASE_SYNC, True, "SYNC", None, "sync"),
    (CASE_CSUM, True, "tCS", None, "corrupt"),
    (CASE_MSB_FIRST, True, "SYNC", None, "msb_first"),
    (CASE_SLOW, True, "tBIT", None, "slow"),
]


#: The divisor of a wire running 5 % slow: 2604 ticks per bit becomes 2734, which is one
#: whole INT with no fraction, so the only thing wrong with it is the rate.
SLOW_CLKDIV = clkdiv_value(2734)


class _HostPatch:
    """A host that gets the frame wrong, in the one way each mutation names.

    These are firmware bugs rather than protocol-model bugs: the wrong sync byte, a checksum
    with one bit flipped, `OUT_SHIFTDIR` the wrong way round, a divisor that is not the rate.
    """

    def __init__(self, kind: str | None) -> None:
        self.kind = kind
        self.saved: dict = {}

    def __enter__(self) -> "_HostPatch":
        if self.kind == "sync":
            self.saved["SYNC_BYTE"] = driver_mod.SYNC_BYTE
            driver_mod.SYNC_BYTE = 0x54       # the bit pattern one off from 0x55
        elif self.kind == "corrupt":
            original = driver_mod.lin_checksum
            self.saved["lin_checksum"] = original
            driver_mod.lin_checksum = lambda pid, data, **kw: original(pid, data, **kw) ^ 0x01
        elif self.kind == "msb_first":
            original = driver_mod.shiftctrl_value
            self.saved["shiftctrl_value"] = original
            driver_mod.shiftctrl_value = lambda **kw: original(out_right=0)
        elif self.kind == "slow":
            self.saved["clkdiv"] = driver_mod.clkdiv_for_rate(FAST)
            driver_mod.clkdiv_for_rate = lambda rate, sm=1: \
                driver_mod.clkdiv_value(2604 + 130, 0)      # 2604 -> 2734: about 5 % slow
        return self

    def __exit__(self, *exc) -> None:
        for name, value in self.saved.items():
            setattr(driver_mod, name, value)
        return None


def _run_mutation(name: str, publish: bool, expected: str | None, mutate, patch) -> tuple:
    """Put a broken commander on the wire and report what noticed.

    The driver's own `program_timing` would refuse several of these programs -- that refusal is
    its own test -- so the timing of the *good* program is stubbed in for the run, exactly as
    `fw/uart_tx/test_uart_tx.py` stubs `bit_cycles` for its mutations.  The question here is
    whether the wire is noticed, not whether the driver starts.
    """
    program = copy.deepcopy(load_program())
    if mutate is not None:
        mutate(program)
    payload = b"\x01\x02"
    clean_timing = driver_mod.program_timing(load_program())
    original = driver_mod.program_timing
    driver_mod.program_timing = lambda _p: clean_timing
    try:
        with _HostPatch(patch):
            bench = Bench(rate=FAST, program=program,
                          publish={} if publish else {0x11: b"\xa5\x5a"},
                          lengths={0x11: len(payload)},
                          clkdiv=SLOW_CLKDIV if patch == "slow" else None)
            bench.frame(0x11, payload)
            bench.run(len(payload))
    finally:
        driver_mod.program_timing = original
    return bench, expected


@pytest.fixture(scope="module")
def clean_run() -> Bench:
    """The unmutated firmware on the wire, once, for the mutations to be compared against."""
    bench = Bench(rate=FAST, publish={0x11: b"\xa5\x5a"}, lengths={0x11: 2})
    bench.frame(0x11)
    bench.run(2)
    assert bench.chk.clean and bench.rsp.published, bench.chk.report()
    return bench


@pytest.mark.parametrize("name,publish,expected,mutate,patch", MUTATIONS,
                         ids=[m[0] for m in MUTATIONS])
def test_broken_firmware_fails_the_check(name: str, publish: bool, expected: str | None,
                                        mutate, patch, clean_run: Bench) -> None:
    """Each way of being wrong is caught by the checker, or by the responder going quiet.

    Two of these are worth reading the expectation for:

    * `CASE_DELIMITER` is the one the checker cannot see.  With the delimiter driven, the sync
      field's start bit merges with the break field into one 16-bit-time dominant run, which is
      still a legal break field, and the byte grid re-frames on the next edge.  What the wire
      carries is a header a responder reads as the wrong identifier, and the responder's
      silence is the only evidence -- which is why this bench judges with the model *and* the
      checker and not with the checker alone.
    * `CASE_BITTIME` does *not* trip `tBIT`, and cannot: the checker re-arms its bit grid at
      every byte field's start bit and samples on a timer, so one stretched bit changes what
      the bytes decode to (SYNC, tSTOP) rather than how long the field measures.  `CASE_SLOW` is
      the mutation that moves the grid -- a whole frame 5 % slow -- and that is what `tBIT`
      measures.
    """
    bench, expected = _run_mutation(name, publish, expected, mutate, patch)
    chk = bench.chk
    fired = rules_fired(chk)
    answered = [fid for _t, fid, _d, _m, _c in bench.rsp.published]
    noticed = bool(fired) or answered != [0x11]
    assert noticed, (
        f"{name}: nothing noticed. checker {chk.report()!r}, responder answered {answered}, "
        f"decoded {[hex(tx.frame_id) for tx in bench.rsp.frames_seen]}")
    if expected is not None:
        assert expected in fired, (
            f"{name}: expected {expected}, the checker fired {sorted(fired)}: {chk.report()}")
    if publish:
        # The chip published the response, so the responder on the wire is subscribed to
        # nothing and must stay off it: two publishers would collide, which on an open-drain
        # wire is invisible by construction (`verif/vip/lin.py`, judgement call 8).
        assert answered == [], f"{name}: a responder answered a frame the chip published"
