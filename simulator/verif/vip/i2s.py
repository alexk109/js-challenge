r"""Pin-level I2S (Philips format) reference models on the tick bus in verif/vip/core.py.

Three push-pull nets -- ``bclk`` (the continuous bit clock, the spec's SCK), ``ws`` (word select,
a.k.a. LRCLK/FS) and ``sd`` (serial data, a.k.a. SDATA) -- and three devices:

    I2sTransmitter   drives all three: a continuous bit clock, WS toggling once per slot, and
                     two's-complement MSB-first words timed to the spec's rules. Queued
                     frames, 16/24/32-bit words, 8-96 kHz, and fault-injection knobs so a
                     test can make it break any one rule.
    I2sReceiver     drives nothing: latches SD on every leading bit-clock edge (UM11732 §6.2)
                     and assembles a word from the cells that follow each WS transition, with
                     the two's-complement value, the channel, the frame index and the tick.
    I2sChecker      drives nothing and judges nothing it did not measure: flags every framing
                     or timing violation with the tick it happened on.

Every timing and framing rule below is quoted from the primary specification, **NXP UM11732
"I2S bus specification" Rev. 3.0, 17 Feb 2022** (the reconstruction of the Philips
specification of Feb 1986 / rev. 5 Jun 1996), saved with its URL and fetch date in
`research/ref/protocols/i2s/`. Nothing here is from memory: `I2S_PROFILES` and the rule table in
`I2sChecker`'s docstring are the only places a number appears, and each cites the saved text.

    bus = Bus(ns_per_tick=20)                                   # 50 MHz chip clock
    for name, idle in zip(NETS, (0, 1, 0)):                    # clock low, WS high, SD low
        bus.add_net(name, push_pull=True, pullup=idle)         # §5: TTL levels, push-pull
    tx = I2sTransmitter(sample_rate=48_000, word_bits=32)
    rx = I2sReceiver(sample_rate=48_000, word_bits=32)
    chk = I2sChecker(sample_rate=48_000, word_bits=32)
    for dev in (tx, rx, chk):
        bus.attach(dev)
    tx.send(0x0123_4567, -0x0123_4567)                         # left, right: one frame
    bus.run_until(lambda b: not tx.busy, 200_000)
    rx.flush()                                                 # §6.2: the last word
    assert chk.violations == []
    assert rx.left == [0x0123_4567] and rx.right == [-0x0123_4567]

or, with the harness helpers, which take the tick rate from the bus, make the nets with the right
idle levels and run the stream out:

    bus = Bus(ns_per_tick=20)
    tx  = attach_transmitter(bus, sample_rate=48_000, word_bits=32)
    rx  = attach_receiver(bus, sample_rate=48_000, word_bits=32)
    chk = attach_checker(bus, sample_rate=48_000, word_bits=32)
    tx.send(0x0123_4567, -0x0123_4567)
    run_frames(bus, tx, rx)                                    # runs out, then rx.flush()
    chk.flush()
    assert chk.clean and rx.values == [0x0123_4567, -0x0123_4567]

The wire
--------
Three lines, and the device generating SCK and WS is the controller (UM11732 §3):

    "• Continuous Serial Clock (SCK);  • Word Select (WS);  • Serial Data (SD); and the device
     generating SCK and WS is the controller."

Data is "transmitted in two's complement with the MSB first" (§3.1), the MSB is transmitted "one
clock period after the WS changes" (§3.1), "the serial data must be latched into the receiver on
the leading edge of the serial clock signal" (§3.1), and WS is "0; channel 1 (left)", "1;
channel 2 (right)" (§3.2). The bit clock rate is the sample rate times the bits per slot times the
number of slots (Wikipedia, "Details": "The bit clock frequency is the product of the sample
rate, the number of bits per channel and the number of channels"), which is what
`bit_clock_hz()` computes.

The frame layout, and why it is what it is
-------------------------------------------
The spec fixes the layout in three sentences that have to agree, and they do only one way
(§6.1, §6.2, §3.1/§3.2 -- quoted in full in the research file):

  1. "At each WS-level change, a pulse WSP is derived for synchronously parallel-loading the
     shift register" (§6.1)  ->  **WS changes exactly once per word**, so one slot is one word
     long and the slots alternate left, right, left, right ...
  2. "Following the first WS-level change, WSP will reset the counter on the falling edge of
     SCK" (§6.2)  ->  the bit counter restarts one *trailing* edge after the WS change.
  3. "the MSB latch (B1) is enabled (EN1 = 1), and the first serial data bit (the MSB) is latched
     into B1 on the rising edge of SCK" (§6.2)  ->  the MSB is latched on the *leading* edge that
     comes after that falling edge.

Combined with "The transmitter always sends the MSB of the next word one clock period after the
WS changes" (§3.1), the WS transition and the leading edge that latches the MSB are **one full
clock period apart**, and the MSB's own data cell is the half-period in between (SD changes at
that trailing edge and holds the MSB across the leading edge). With a rising edge at every
integer multiple of T and a trailing edge at (k + 0.5)·T, for `slot_bits` = 16:

    tick      ...   w        w + T/2        w + T  ...
    bclk           _/‾\_/‾‾\_/‾‾‾\_            rising at w and w+T
    ws             changes at w; next change at w + 16T
    sd             the previous word's LSB until w + T/2, the MSB from w + T/2

so the LSB of a word is latched on the very rising edge where WS flips to the next slot -- which
is exactly §6.2's "On the next WS-level change, the contents of the n latches are written in
parallel". `I2sTransmitter` drives exactly this, and `I2sChecker` checks it as the `WS-1T` rule.

One honest ambiguity, made a parameter instead of a guess: §3.2 says "WS may change either on a
trailing or leading edge of the serial clock", and Wikipedia says it is "typically synchronized to
the falling edge". The two statements are not the same requirement, and the model keeps them apart:

* `WS-EDGE` accepts **either** clock edge, because §3.2 says so. A WS transition one tick off any
  edge is still a fault.
* `WS-1T` enforces §3.1/§3.2's **one clock period** between the WS change and the edge that
  latches the MSB. That is only reachable from a change on a *leading* edge: from a trailing edge
  just the low half-period is left (`low` ticks, 17 at the default 48 kHz / 16-bit geometry), and
  §6.2's receiver -- which resets its counter on the *next* falling edge -- would then start
  counting cells one cell late. So by default `ws_lag_ticks=None` means "the lead is one clock
  period" and the half-period placement is **flagged**, not accepted quietly.
* `ws_lag_ticks=<int>` states the lead in ticks for a part that really does shift WS earlier, so
  such a device can be checked on purpose. The cost is stated in `RULE_HELP["WS-1T"]`: the
  receiver and the checker's decoder count cells from the first leading edge after the WS change,
  so a stream whose lead is not one clock period is decoded shifted by one cell.

Timings
-------
UM11732 §4: "All timing requirements are specified relative to the clock period or to the minimum
allowed clock period of a device."  Two profiles, both in `I2S_PROFILES`:

    "relative"   Figure 2/3 and Table 3, the relative form that scales to any rate:
                 tHC >= 0.35T, tLC >= 0.35T, tdtr <= 0.8T, thtr >= 0, tsr >= 0.2Tr, tRC <= 0.15Tr
    "abs25"      the same, plus the absolute numbers of Table 1 (controller transmitter at
                 2.5 MHz) and Table 2 (target receiver at 2.5 MHz): tHC >= 160 ns, tLC >= 160 ns,
                 tdtr <= 300 ns, thtr >= 100 ns, tsr >= 60 ns, 360 ns <= T <= 440 ns

The 1986 parts' `Ttr = 360 ns` is *not* a bus rule -- Table 3 note [1] calls it "the minimum
allowed clock period" of a device -- so the VIP never imposes it unless asked: 96 kHz with 32-bit
words needs 6.144 MHz of SCK and no reading of the document forbids it. Pass
`max_sck_hz=1e9/360e-9` to a checker to make that figure bite anyway.

What is *not* modelled, and why
-------------------------------
* **MCLK.** "Master clock (typically 256 x LRCLK); not part of the standard" (Wikipedia). Three
  nets, no more.
* **tRC, the clock rise time** (Table 3: <= 0.15Tr, "only relevant for transmitters in target
  mode"). An analogue edge cannot be measured at one tick of resolution; the checker cannot see
  it and does not claim to.
* **A stalled clock.** A clock that stops produces no edges, so there is nothing to time.
  `I2sReceiver(max_gap_ticks=...)` notices a clock that runs *slow* — it is a gap threshold
  between two rising edges — and `busy`/`run_until` on the transmitter is how you see a clock that
  has stopped.
* **Voltages and cables.** §5 gives TTL levels; the wiki notes I2S "was not intended to be used
  via cables" and has "no error detection mechanism", so the checker reports timing and framing
  only -- there is no CRC or parity anywhere on this bus to check.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from math import ceil
from typing import Mapping, Optional

from .core import Bus, Device

NETS = ("bclk", "ws", "sd")            # SCK, WS/LRCLK, SD/SDATA
MIN_TICKS_PER_BIT_CLOCK = 4            # so that a half-period is two ticks and a sample decidable
WORD_BITS_CHOICES = (16, 24, 32)       # the supported set; 4..32 accepted in fact
LEFT = 0                               # "WS = 0; channel 1 (left)"  (UM11732 §3.2)
RIGHT = 1                              # "WS = 1; channel 2 (right)"

_EPS = 1e-9


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


def _check_word_bits(value: int) -> int:
    bits = int(value)
    if not 4 <= bits <= 32:
        raise ValueError(
            f"word_bits must be 4..32 (got {value!r}); UM11732 §3.1 fixes only the MSB's "
            f"position, so any width works, but {WORD_BITS_CHOICES} are the usual ones"
        )
    return bits


def _check_slots(value: int) -> int:
    slots = int(value)
    if not 1 <= slots <= 8:
        raise ValueError(f"slots must be 1..8 (got {value!r}): a frame is `slots` words")
    return slots


def _check_ticks_per_bit_clock(value: float) -> float:
    tpb = float(value)
    if tpb < MIN_TICKS_PER_BIT_CLOCK:
        raise ValueError(
            f"a bit clock period of {value!r} ticks is below MIN_TICKS_PER_BIT_CLOCK="
            f"{MIN_TICKS_PER_BIT_CLOCK}: at less than 4 ticks per period a half-period cannot "
            "hold the 0.35T of UM11732 Figure 2 and a sample point cannot be told from a glitch"
        )
    return tpb


def bit_clock_hz(sample_rate: float, word_bits: int = 16, slots: int = 2,
                 slot_bits: Optional[int] = None) -> float:
    """The bit clock (SCK/BCLK) frequency of an I2S stream, in Hz.

    Wikipedia, "Details": "The bit clock frequency is the product of the sample rate, the number
    of bits per channel and the number of channels. So, for example, CD Audio with a sample
    frequency of 44.1 kHz, with 16 bits of precision and two channels (stereo) has a bit clock
    frequency of: 44.1 kHz × 16 × 2 = 1.4112 MHz". ST's AN4309 states the same identity in the
    footnote of its I2S figure. `slot_bits` defaults to `word_bits`; a stream whose slot is longer
    than its word (24-bit data in a 32-bit slot, AN4309: "16-, 24- or 32-bit data length ... and to
    16- or 32-bit packet length") passes it, because the clock counts *slot* bits.
    """
    rate = float(sample_rate)
    if not rate > 0:
        raise ValueError(f"sample_rate must be positive, got {sample_rate!r}")
    per_frame = _check_word_bits(word_bits) if slot_bits is None else int(slot_bits)
    if per_frame < 1:
        raise ValueError(f"slot_bits must be >= 1 (got {slot_bits!r})")
    return rate * per_frame * _check_slots(slots)


def ticks_per_bit_clock(
    ns_per_tick: float,
    sample_rate: Optional[float] = None,
    word_bits: int = 16,
    slots: int = 2,
    slot_bits: Optional[int] = None,
) -> int:
    """Bit-clock period in whole ticks, rounded up, at least MIN_TICKS_PER_BIT_CLOCK.

    Give `sample_rate` (and the word/slot geometry) or `slot_bits`; `sample_rate` wins. Rounding
    up keeps the modelled clock at or below the nominal frequency, so a stream built from this
    never claims to be faster than it is.
    """
    if slot_bits is not None and int(slot_bits) < 1:
        raise ValueError(f"slot_bits must be >= 1 (got {slot_bits!r})")
    per_frame = _check_slots(slots) * (int(slot_bits) if slot_bits else _check_word_bits(word_bits))
    if sample_rate is not None:
        period_ns = 1e9 / bit_clock_hz(sample_rate, word_bits, slots, slot_bits)
    elif slot_bits:
        raise ValueError("ticks_per_bit_clock needs sample_rate (with word_bits/slots) or slot_bits")
    else:
        raise ValueError("ticks_per_bit_clock needs sample_rate or slot_bits")
    ticks = ns_to_ticks(ns_per_tick, period_ns)
    if ticks < MIN_TICKS_PER_BIT_CLOCK:
        raise ValueError(
            f"a {sample_rate} Hz {word_bits}-bit x{slots} stream needs {period_ns:.1f} ns per bit "
            f"clock period, which is {ticks} ticks at {ns_per_tick} ns/tick -- below "
            f"MIN_TICKS_PER_BIT_CLOCK={MIN_TICKS_PER_BIT_CLOCK}"
        )
    return ticks


@dataclass(frozen=True)
class I2sProfile:
    """One timing class: the relative limits of UM11732 §4 and, optionally, absolute ones.

    The `t_*` fields are fractions of a clock period `T` (`t_hc`, `t_lc`, `t_dtr`, `t_htr`) or of
    the minimum allowed period `Tr` (`t_su`, `t_thr`, `t_rc`), as Table 3 states them. The `ns_*`
    fields are the absolute nanosecond limits of Table 1 (a controller transmitter at 2.5 MHz)
    and Table 2 (a target receiver at 2.5 MHz); `0.0` means "this profile states no absolute
    limit", and the checker simply does not check it.
    """

    name: str
    t_hc: float          # min clock HIGH, fraction of T        (Figure 2/3, Table 3)
    t_lc: float          # min clock LOW, fraction of T         (Figure 2/3, Table 3)
    t_dtr: float         # max delay of SD/WS after the leading edge, fraction of T
    t_htr: float         # min hold of SD/WS before the leading edge, fraction of T
    t_su: float          # receiver min setup of SD, fraction of Tr
    t_thr: float         # receiver min hold of SD, fraction of Tr
    t_rc: float          # max clock rise time, fraction of Tr (not measurable at tick resolution)
    ns_hc: float = 0.0   # Table 1: min clock HIGH
    ns_lc: float = 0.0   # Table 1: min clock LOW
    ns_dtr: float = 0.0  # Table 1: max data/WS delay
    ns_htr: float = 0.0  # Table 1: min data/WS hold
    ns_su: float = 0.0   # Table 2: min receiver setup
    ns_t_min: float = 0.0  # Table 1/2: min clock period
    ns_t_max: float = 0.0  # Table 1/2: max clock period


#: UM11732 Figure 2 ("SD and WS ... tLC ≥ 0.35T, thtr ≥ 0, tdtr ≤ 0.8T, tHC ≥ 0.35T"), Figure 3
#: ("tsr ≥ 0.2T, thr ≥ 0") and Table 3 (which adds tRC ≤ 0.15Tr).
I2S_PROFILES: dict[str, I2sProfile] = {
    "relative": I2sProfile("relative", 0.35, 0.35, 0.8, 0.0, 0.2, 0.0, 0.15),
    "abs25": I2sProfile("abs25", 0.35, 0.35, 0.8, 0.0, 0.2, 0.0, 0.15,
                        ns_hc=160.0, ns_lc=160.0, ns_dtr=300.0, ns_htr=100.0, ns_su=60.0,
                        ns_t_min=360.0, ns_t_max=440.0),
}

#: rule name -> one line of what it is, for `I2sChecker.rule_help()`.
RULE_HELP: dict[str, str] = {
    "tHC": "clock HIGH half-period >= 0.35T (Figure 2/3, Table 3; >= 160 ns on the abs25 profile)",
    "tLC": "clock LOW half-period >= 0.35T (Figure 2/3, Table 3; >= 160 ns on the abs25 profile)",
    "tH;TR": "SD/WS must hold at least one whole tick across the leading edge (the spec's Table 3 "
             "'thtr >= 0' is below one tick of resolution, so the model rounds it up to 1; 0 ticks "
             "of hold means the change landed in the same tick as the edge, which a digital model "
             "cannot resolve and silicon would see as ambiguous setup/hold)",
    "tD;TR": "SD/WS may change no later than 0.8T after the leading edge it follows "
             "(Figure 2/Table 3 'tdtr'; <= 300 ns on the abs25 profile)",
    "tSU;SD": "SD must be stable >= 0.2Tr before the leading edge that latches it "
              "(Figure 3/Table 3 'tsr'; >= 60 ns on the abs25 profile)",
    "WS-EDGE": "a WS transition must coincide with a bit-clock edge (Table 3 'WS may change "
               "either on a trailing or leading edge')",
    "WS-1T": "the WS transition is one clock period before the leading edge that latches the "
             "MSB (UM11732 §3.1/§3.2, with the half-period of §6.2's counter reset); a change on "
             "a trailing edge leaves only the low half and is flagged, and ws_lag_ticks is the "
             "opt-in for a device that really lags that little -- but then the receiver and this "
             "checker both count cells from the first leading edge after the WS change, so such a "
             "stream is decoded shifted by one cell",
    "WS-CHAN": "WS is the left channel's level on even slots and the right channel's on odd ones "
               "(§3.2: WS = 0 channel 1 left, WS = 1 channel 2 right)",
    "SLOT": "a slot is exactly slot_bits clock periods and WS changes exactly once per slot "
            "(§6.1: 'At each WS-level change ... parallel-loading the shift register')",
    "WORD": "a slot carries at least word_bits data cells, MSB first (§3.1: 'If the receiver is "
            "sent fewer bits than its word length, the missing bits are set to zero internally')",
    "fSCK": "the bit clock runs at sample_rate x slots x slot_bits (Wikipedia 'Details', ST "
            "AN4309 footnote (a)); on the abs25 profile also 360 ns <= T <= 440 ns (Tables 1/2)",
}


def i2s_limits(
    ns_per_tick: float = 20.0,
    sample_rate: Optional[float] = None,
    word_bits: int = 16,
    slots: int = 2,
    slot_bits: Optional[int] = None,
    ticks_per_period: Optional[int] = None,
    profile: str = "relative",
    timing: Optional[dict] = None,
) -> dict:
    """Every spec limit for one stream geometry, as whole ticks, plus its nominal rates.

    `ticks_per_period` overrides the bit-clock period computed from `sample_rate`; pass it when a
    device is told to run at some other rate than the sample rate it is checked at. `timing`
    overrides individual limits by their long names (`{"t_hc": 0.5}`), or the absolute ones by
    their `ns_` names (`{"ns_hc": 200}`), which is how a test drives a checker off-spec on
    purpose. `sample_rate` None means "do not check the rate", and then `ticks_per_period` is
    required.

    The fractional limits are rounded so that they are *at least* as strict as the exact
    fraction: `ceil(f · P)` ticks for a floor and `floor(f · P)` for a ceiling, with `P` the
    nominal period in ticks. Because the checker applies the same rounding to the *measured*
    period, a legal stream is never flagged and an illegal one is never missed.
    """
    try:
        prof = I2S_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown I2S profile {profile!r}; pick one of {sorted(I2S_PROFILES)}"
        ) from None
    word_bits = _check_word_bits(word_bits)
    slots = _check_slots(slots)
    sb = int(slot_bits) if slot_bits else word_bits
    if sb < 1:
        raise ValueError(f"slot_bits must be >= 1 (got {slot_bits!r})")
    if sb < word_bits:
        raise ValueError(
            f"slot_bits ({sb}) is shorter than word_bits ({word_bits}): UM11732 §6.1 has the "
            f"transmitter send every bit of the word ('At each WS-level change ... parallel-loading "
            f"the shift register') and §6.2 latches 'n' bits of them"
        )
    if ticks_per_period is not None:
        period = int(_check_ticks_per_bit_clock(ticks_per_period))
        ns_period = period * ns_per_tick
        rate = 1e9 / (ns_period * slots * sb)
    elif sample_rate is not None:
        period = ticks_per_bit_clock(ns_per_tick, sample_rate, word_bits, slots, sb)
        ns_period = period * ns_per_tick
        rate = float(sample_rate)
    else:
        raise ValueError(
            "i2s_limits needs sample_rate (the frame rate to check the bit clock against) or "
            "ticks_per_period; pass sample_rate=None to check an unknown rate"
        )
    over = dict(timing or {})
    unknown = [k for k in over if k not in {f for f in I2sProfile.__dataclass_fields__}]
    if unknown:
        raise ValueError(
            f"unknown timing override(s) {sorted(unknown)}; pick from "
            f"{sorted(I2sProfile.__dataclass_fields__)}"
        )
    prof = replace(prof, **over)
    out = {
        "t_hc": int(ceil(prof.t_hc * period)),
        "t_lc": int(ceil(prof.t_lc * period)),
        "t_dtr": int(prof.t_dtr * period),          # ceiling: floor()
        "t_htr": max(int(ceil(prof.t_htr * period)), 1),
        "t_su": int(ceil(prof.t_su * period)),
        "t_thr": max(int(ceil(prof.t_thr * period)), 1),
        "t_rc": int(ceil(prof.t_rc * period)),      # recorded; not checkable at tick resolution
        "ns_hc": ns_to_ticks(ns_per_tick, prof.ns_hc),
        "ns_lc": ns_to_ticks(ns_per_tick, prof.ns_lc),
        "ns_dtr": ns_to_ticks(ns_per_tick, prof.ns_dtr),
        "ns_htr": ns_to_ticks(ns_per_tick, prof.ns_htr),
        "ns_su": ns_to_ticks(ns_per_tick, prof.ns_su),
        "ns_t_min": ns_to_ticks(ns_per_tick, prof.ns_t_min),
        "ns_t_max": ns_to_ticks(ns_per_tick, prof.ns_t_max) if prof.ns_t_max else 0,
        "period": period,
        "word_bits": word_bits,
        "slot_bits": sb,
        "slots": slots,
        "sample_rate": rate,
        "bit_clock_hz": 1e9 / (ns_period * 1.0),
        "ns_per_tick": ns_per_tick,
        "ns_period": ns_period,
        "profile": prof.name,
    }
    return out


# ---- word helpers -------------------------------------------------------------------
def sample_to_word(value: int, word_bits: int) -> int:
    """The `word_bits`-wide two's-complement bit pattern for a signed or unsigned sample."""
    mask = (1 << _check_word_bits(word_bits)) - 1
    return int(value) & mask


def word_to_sample(word: int, word_bits: int) -> int:
    """Sign-extend a received word to a Python int ("Data is signed", Wikipedia "Details")."""
    bits = _check_word_bits(word_bits)
    word = int(word) & ((1 << bits) - 1)
    if word & (1 << (bits - 1)):
        return word - (1 << bits)
    return word


def word_bits_msb_first(word: int, word_bits: int) -> list[int]:
    """The bits of a word, MSB first ("Serial data is transmitted in two's complement with the
    MSB first", UM11732 §3.1) -- the order `I2sTransmitter` shifts them out in."""
    bits = _check_word_bits(word_bits)
    return [(int(word) >> (bits - 1 - i)) & 1 for i in range(bits)]


# ---- the transmitter ----------------------------------------------------------------
def _half_period(period: int, ratio: float, min_high: int, min_low: int) -> int:
    """Ticks for the clock HIGH half, honouring `ratio` but never breaking either limit.

    UM11732 Figure 2 asks for tHC >= 0.35T and tLC >= 0.35T, so a half can be at most 0.65T; this
    clamps `ratio` into [min_high, period - min_low] rather than raising, so a legal device cannot
    be made illegal by a careless keyword. A test that wants an *illegal* duty cycle writes its
    own two-line Device (as `verif/vip/test_i2c.py`'s `SloppyMaster` does) or asks a checker for
    a stricter limit with `timing={"t_hc": 0.7}`.
    """
    high = max(int(round(period * ratio)), min_high, 1)
    high = min(high, period - min_low)
    if high < 1:
        raise ValueError(
            f"a bit-clock period of {period} ticks cannot hold a HIGH half: tHC needs "
            f"{min_high} and tLC needs {min_low}"
        )
    return high


@dataclass
class TxSlot:
    """One slot as the transmitter drove it."""

    index: int                    # slot index since the transmitter started (0 = first left word)
    channel: int                  # 0 = first slot of the frame (left, §3.2), 1 = right, ...
    frame: int                    # frame this slot belongs to
    word: int                     # the value driven, masked to word_bits
    bits: tuple[int, ...]         # the bits shifted out, MSB first
    ws_tick: Optional[int] = None   # tick WS changed for this slot (drive tick)
    msb_tick: Optional[int] = None   # tick of the leading edge that latches the MSB (drive tick)
    lsb_tick: Optional[int] = None   # ... and the LSB
    end_tick: Optional[int] = None   # the leading edge that starts the next slot


class I2sTransmitter(Device):
    """A bit-level I2S controller: a continuous bit clock, WS per slot, MSB-first words.

    Everything it does is timed from the bit-clock period, so the same class is a legal 8 kHz
    16-bit stream or a legal 96 kHz 32-bit one (1.024 MHz ... 6.144 MHz of SCK). Frames are
    queued with `send()`; a frame is `slots` samples in slot order (left, right by default), so
    `send(0x1234, -0x5678)` is one stereo frame.

    How it drives the three nets, and why (UM11732 §3.1, §3.2, §6.1, §6.2, and the frame layout
    worked out in the module docstring):

    * SCK is continuous while there is anything to send. One period is `period` ticks, split into
      a `high` and a `low` half of at least `ceil(0.35 · period)` ticks ("tHC ≥ 0.35T, tLC ≥ 0.35T",
      Figure 2). A 50% split unless `high_ratio` says otherwise, clamped to stay legal.
    * WS toggles at the leading edge that is a whole `slot_bits` periods before the slot's MSB
      ("At each WS-level change ... parallel-loading the shift register", §6.1); the MSB is then
      latched one period later (§6.1/§6.2). It idles at the level of the channel that is *not*
      selected, so the first slot boundary is a real transition and can be timed like any other.
    * SD changes on each trailing edge, so the bit it presents is the one the *next* rising edge
      latches: Figure 2's "ACTIVE RISING CLOCK EDGE / DATA", leaving `high` ticks of hold across
      that edge (≥ the "thtr ≥ 0" of §4) and `low + 1` ticks of delay after the previous one
      (≤ the 0.8T of Table 3). Cells past the LSB of a slot drive 0, as §6.1 says they do: "Since
      the serial data input is zero, all the bits after the LSB will also be zero."

    While idle it holds SCK low and leaves WS and SD frozen at their last levels, so an idle
    transmitter cannot look like a framing error and a receiver that is waiting for "the next WS
    change" (§6.2) never sees a phantom one.

    Every pin level is a pure function of the tick, not a queue of scheduled events, which is what
    lets the fault-injection offsets be exact for any value, positive or negative:

        ws_offset_ticks   shifts every WS transition. Positive: WS moves after the edge (a
                          half-period-late WS is a real hardware fault and `WS-EDGE`/`WS-1T`
                          catch it). Negative: WS moves before it -- `-low` puts the change on
                          the preceding trailing edge, since the LOW half is `low` ticks long (17
                          ticks, against `high`'s 16, at the default 48 kHz / 16-bit geometry),
                          and `-high` is one tick *past* that edge. Note that the shift applies to
                          every boundary but the first, which is pinned to the start of the
                          stream, so a negative offset skews the first slot: it is a fault injector,
                          not a way to build the alternative trailing-edge layout (that stream is
                          flagged by `WS-1T`; `test_a_trailing_edge_ws_layout_is_flagged_not_accepted`
                          shows it).
        sd_offset_ticks   shifts every SD change relative to its trailing edge. Negative enough
                          (`-(low + 1)`) and SD moves on the latching rising edge itself, which is
                          the zero-hold case `tH;TR` flags. Positive enough and it moves later
                          than 0.8T, which `tD;TR` flags.
        slot_bits         a slot shorter than `word_bits` truncates the word (the cells simply
                          stop): `WORD` flags it and a receiver zero-fills, as §3.1 says.
        high_ratio        clamped to the legal duty range; see `_half_period`.
    """

    name = "i2s_tx"

    def __init__(
        self,
        sample_rate: Optional[float] = 48_000.0,
        word_bits: int = 16,
        slots: int = 2,
        ns_per_tick: float = 20.0,
        bus: Optional[Bus] = None,
        ticks_per_period: Optional[int] = None,
        slot_bits: Optional[int] = None,
        profile: str = "relative",
        timing: Optional[dict] = None,
        left_level: int = LEFT,
        nets: tuple[str, str, str] = NETS,
        name: Optional[str] = None,
        *,
        cells_per_slot: Optional[int] = None,
        start_delay_ticks: int = 2,
        high_ratio: float = 0.5,
        ws_offset_ticks: int = 0,
        sd_offset_ticks: int = 0,
    ):
        self.bclk_net, self.ws_net, self.sd_net = nets
        if name:
            self.name = name
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.left_level = 1 if int(left_level) else 0
        if not 0.0 <= float(high_ratio) <= 1.0:
            raise ValueError(f"high_ratio must be 0..1 (got {high_ratio!r})")
        self.high_ratio = float(high_ratio)
        self._period_given = ticks_per_period is not None
        self.limits = i2s_limits(
            self.ns_per_tick, sample_rate=sample_rate, word_bits=word_bits, slots=slots,
            slot_bits=slot_bits, ticks_per_period=ticks_per_period, profile=profile, timing=timing,
        )
        self.word_bits = self.limits["word_bits"]
        self.slots = self.limits["slots"]
        self.slot_bits = self.limits["slot_bits"]
        #: Fault injection: drive fewer cells per slot than the geometry says, which truncates
        #: the word. `i2s_limits` refuses a geometry whose slot is shorter than its word -- a
        #: transmitter is allowed to do it, and that is exactly what `WORD` is there to catch.
        self.cells_per_slot = min(
            self.slot_bits if cells_per_slot is None else int(cells_per_slot),
            self.slot_bits)
        if self.cells_per_slot < 1:
            raise ValueError(f"cells_per_slot must be >= 1 (got {cells_per_slot!r})")
        self.sample_rate = self.limits["sample_rate"]
        self.period = self.limits["period"]
        self.high = _half_period(self.period, self.high_ratio,
                                 self.limits["t_hc"], self.limits["t_lc"])
        self.low = self.period - self.high
        self.ws_offset_ticks = int(ws_offset_ticks)
        self.sd_offset_ticks = int(sd_offset_ticks)
        self.start_delay_ticks = max(int(start_delay_ticks), 0)
        self._stream: list[int] = []      # every queued word, in slot order
        self.tx_log: list[TxSlot] = []
        self._reset()

    # -- configuration ----------------------------------------------------------
    @property
    def ticks_per_bit_clock(self) -> float:
        """The nominal bit-clock period in ticks (what `high_ratio` splits)."""
        return self.period

    @property
    def bit_clock_hz(self) -> float:
        """The bit clock this transmitter actually drives, in Hz."""
        return 1e9 / (self.ns_per_tick * self.period)

    @property
    def slot_span(self) -> int:
        """Ticks one slot takes: `cells_per_slot` bit-clock periods."""
        return self.cells_per_slot * self.period

    def bind(self, bus: Bus) -> "I2sTransmitter":
        """Take the tick rate from `bus` and recompute. Done by the constructor when bus=."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.limits = i2s_limits(
            self.ns_per_tick, sample_rate=self.sample_rate, word_bits=self.word_bits,
            slots=self.slots, slot_bits=self.slot_bits,
            ticks_per_period=self.period if self._period_given else None,
            profile=self.limits["profile"],
        )
        self.period = self.limits["period"]
        self.high = _half_period(self.period, self.high_ratio,
                                 self.limits["t_hc"], self.limits["t_lc"])
        self.low = self.period - self.high
        self._reset()
        return self

    # -- the queue --------------------------------------------------------------
    def send(self, *samples: int | Iterable[int]) -> "I2sTransmitter":
        """Queue samples in slot order: `send(left, right)` is one frame, `send([l, r, l2, r2])`
        two. A frame is `slots` samples, so with stereo (the default) samples are taken in
        left/right pairs."""
        flat: list[int] = []
        for item in samples:
            if isinstance(item, int):
                flat.append(item)
            else:
                flat.extend(item)
        if len(flat) % self.slots:
            raise ValueError(
                f"{len(flat)} samples is not a whole number of {self.slots}-sample frames; "
                f"send left/right pairs (or {self.slots} samples per frame)"
            )
        self._stream.extend(sample_to_word(v, self.word_bits) for v in flat)
        if not self.pending:
            self._started = False        # a new queue after a finished one: stream it again
        return self

    @property
    def pending(self) -> int:
        """Samples queued but not started yet."""
        return max(0, len(self._stream) - len(self.tx_log))

    @property
    def busy(self) -> bool:
        """True while the clock is running or a sample is still queued."""
        return self.pending > 0 or (self._running and self._t < self._end_tick())

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def sent(self) -> int:
        """Slots finished so far."""
        return sum(1 for s in self.tx_log if s.end_tick is not None)

    def stop(self) -> None:
        """Drop the queue; the clock stops after the slot it is in."""
        self._stream = self._stream[:len(self.tx_log)]

    def run(self, bus: Optional[Bus] = None, slots: Optional[int] = None,
            limit: int = 1_000_000) -> bool:
        """Step `bus` until `slots` slots are done (default: the whole queue). False on timeout."""
        bus = bus or self.bus
        if bus is None:
            raise ValueError("run() needs a Bus: pass bus= or attach the transmitter to one")
        want = slots if slots is not None else self.pending
        return bus.run_until(lambda b: self.sent >= want, limit)
# -- the device -------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        self._t = t
        if not self._running and not self._started and self.pending and t >= self._first_rise:
            self._running = self._started = True
            self._first_rise = t
            self._logged = 0
        if not self._running:
            return {self.bclk_net: 0, self.ws_net: self._ws_idle, self.sd_net: self._idle_sd}
        end = self._end_tick()
        if t >= end:
            self._running = False
            self._log_edges(end - self.high)      # the last rising edge is the last one driven
            if self.tx_log and self.tx_log[-1].end_tick is None:
                # the stream stops here: no further rising edge opens a slot, so the last slot
                # ends on the last rising edge, which is where the next one would have begun
                self.tx_log[-1].end_tick = end - self.high
            return {self.bclk_net: 0, self.ws_net: self._ws_level(end - 1),
                    self.sd_net: self._sd_level(end - 1)}
        self._log_edges(t)
        phase = t - self._first_rise
        return {
            self.bclk_net: 1 if phase % self.period < self.high else 0,
            self.ws_net: self._ws_level(t),
            self.sd_net: self._sd_level(t),
        }

    # -- the timeline -----------------------------------------------------------
    def _end_tick(self) -> int:
        """The tick after the last rising edge, i.e. where the clock stops (holding low).

        A slot is `slot_bits` periods long and its LSB rides out on the rising edge that starts
        the *next* slot, so `n` words need `n * slot_bits + 1` rising edges: the first boundary
        (where WS changes and the MSB is one period away) and one boundary per word.
        """
        if not self._stream:
            return 0
        return self._first_rise + len(self._stream) * self.slot_span + self.high

    def _ws_level(self, t: int) -> int:
        """WS as a function of the tick: it toggles every `slot_span` ticks from the first one,
        shifted by `ws_offset_ticks`, and freezes on the last slot's level when the stream ends."""
        k = (t - self._first_rise - self.ws_offset_ticks) // self.slot_span + 1
        k = max(1, min(k, len(self._stream)))
        return self._ws_idle ^ (k & 1)

    def _sd_level(self, t: int) -> int:
        """SD as a function of the tick: cell `k` goes out on the trailing edge that precedes the
        rising edge that latches it (`low` ticks before it), so the value on the wire is the one
        next to be latched (§3.1: "the serial data must be latched into the receiver on the
        leading edge"), leaving `high` ticks of hold across that edge. Cell 0 -- the MSB -- is the
        first cell after the WS change, so its data goes out on the trailing edge that follows
        it, which is §6.2's "WSP will reset the counter on the falling edge of SCK"."""
        idx = (t - self._first_rise + self.low - 1 - self.sd_offset_ticks) // self.period - 1
        idx = max(0, min(idx, len(self._stream) * self.cells_per_slot - 1))
        word = self._stream[idx // self.cells_per_slot]
        cell = idx % self.cells_per_slot
        bits = word_bits_msb_first(word, self.word_bits)
        return bits[cell] if cell < len(bits) else 0       # §6.1: zeros after the LSB

    # -- the log ----------------------------------------------------------------
    def _log_edges(self, t: int) -> None:
        """Fill in `tx_log` up to the rising edges driven by tick `t`.

        Each rising edge latches one cell: cell 0 (the MSB) one period after the WS change, and
        the last cell of the slot on the edge that WS changes on (§6.2). Only the log needs this
        bookkeeping; the pins are `tick()`'s pure functions.
        """
        total = len(self._stream) * self.cells_per_slot
        rises = min((t - self._first_rise) // self.period + 1, total)
        while self._logged < rises:
            index = self._logged
            tick = self._first_rise + index * self.period
            if index % self.cells_per_slot == 0:
                if self.tx_log:
                    self.tx_log[-1].end_tick = tick          # its LSB rode out on this edge
                    if self.tx_log[-1].lsb_tick is None:
                        self.tx_log[-1].lsb_tick = tick
                if index // self.cells_per_slot >= len(self._stream):
                    break                                   # the last slot is closed, no new one
                word = self._stream[index // self.cells_per_slot]
                self.tx_log.append(TxSlot(
                    index=index // self.cells_per_slot,
                    channel=(index // self.cells_per_slot) % self.slots,
                    frame=index // (self.cells_per_slot * self.slots),
                    word=word, bits=tuple(word_bits_msb_first(word, self.word_bits)),
                    ws_tick=tick + self.ws_offset_ticks,
                ))
            elif self.tx_log:
                slot = self.tx_log[-1]
                if slot.msb_tick is None:
                    slot.msb_tick = tick
                elif index % self.cells_per_slot == self.word_bits - 1:
                    slot.lsb_tick = tick
            self._logged += 1

    def _reset(self) -> None:
        self._stream.clear()
        self.tx_log.clear()
        self._reset_state()

    def _reset_state(self) -> None:
        self._running = False
        self._started = False
        self._t = 0
        self._first_rise = self.start_delay_ticks
        self._logged = 0
        self._ws_idle = 1 - self.left_level   # the level of the channel that is *not* selected
        self._idle_sd = 0


# ---- the receiver --------------------------------------------------------------------
@dataclass
class RxSample:
    """One word as the receiver decoded it."""

    value: int                     # the raw word_bits-wide pattern
    sample: int                    # it sign-extended to a Python int
    channel: int                   # 0 = first slot of a frame (left), 1 = right, ...
    frame: int                     # frames counted from the receiver's first WS transition
    slot: int                      # slot index since the receiver started
    ws_tick: int                   # tick of the WS transition that opened the slot
    msb_tick: Optional[int] = None # tick of the leading edge that latched the MSB
    bits: int = 0                  # how many cells were actually latched (word_bits when complete)

    def __str__(self) -> str:
        return (f"t={self.ws_tick} {'left' if self.channel == 0 else 'right'} "
                f"0x{self.value:0{(self.value.bit_length() + 3) // 4 or 1}x} ({self.sample})")


class I2sReceiver(Device):
    """A passive I2S target: latches SD on every leading edge and assembles words from WS.

    It is §6.2's receiver, literally: "Following the first WS-level change, WSP will reset the
    counter on the falling edge of SCK", and then "the first serial data bit (the MSB) is latched
    into B1 on the rising edge of SCK. As the counter increases by one every clock pulse,
    subsequent data bits are latched into B2 to Bn. On the next WS-level change, the contents of
    the n latches are written in parallel, depending on WSD, into either the left or the right
    data-word latch." So it

    * uses each WS transition as the start of a slot and the channel flag for it,
    * latches SD on every rising edge of the bit clock into a shift register,
    * keeps the first `word_bits` cells of the slot (the MSB is the first one, one clock period
      after the WS change), and
    * on the next WS transition writes them out, sign-extended, tagged with the channel and the
      frame it belongs to.

    Bits after the LSB are ignored ("If the receiver is sent more bits than its word length, the
    bits after the LSB are ignored") and a short slot is zero-filled ("If the receiver is sent
    fewer bits than its word length, the missing bits are set to zero internally"); both are
    counted in `ignored_bits` and `truncated_words` so a test can see them.

    Sampling latency: the bus resolves a drive on the tick it is returned, so the pins a device
    sees on tick `t` were driven on tick `t-1`. `latency` (default 1, matching `verif/vip/uart.py`)
    is subtracted from every tick reported, so the logs hold the ticks the transmitter really
    drove. Pass the device's own latency when the pins come from RTL.

    Silence: `max_gap_ticks` (0 = off) is the gap threshold, and it is a *gap* threshold --
    measured between two rising edges -- so it counts a clock that runs slow. `stalls` is how many
    edges arrived more than `max_gap_ticks` after the one before it, `max_gap` the longest gap
    seen. A clock that stops dead produces no edge at all and moves neither number; for that, poll
    `busy` on the transmitter from the harness.

    A receiver attached to a bus that is already running has no idea which channel comes first:
    the first WS transition it sees it calls slot 0, so `left`/`right` are "the first channel seen
    and then the other one" until a frame boundary reveals the phase. `first_channel` names it
    for the log.
    """

    name = "i2s_rx"

    def __init__(
        self,
        sample_rate: Optional[float] = 48_000.0,
        word_bits: int = 16,
        slots: int = 2,
        ns_per_tick: float = 20.0,
        bus: Optional[Bus] = None,
        ticks_per_period: Optional[int] = None,
        slot_bits: Optional[int] = None,
        profile: str = "relative",
        timing: Optional[dict] = None,
        latency: int = 1,
        first_channel: int = LEFT,
        nets: tuple[str, str, str] = NETS,
        max_gap_ticks: int = 0,
        name: Optional[str] = None,
    ):
        self.bclk_net, self.ws_net, self.sd_net = nets
        if name:
            self.name = name
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.limits = i2s_limits(
            self.ns_per_tick, sample_rate=sample_rate, word_bits=word_bits, slots=slots,
            slot_bits=slot_bits, ticks_per_period=ticks_per_period, profile=profile, timing=timing,
        )
        self._period_given = ticks_per_period is not None
        self.word_bits = self.limits["word_bits"]
        self.slots = self.limits["slots"]
        self.slot_bits = self.limits["slot_bits"]
        self.sample_rate = self.limits["sample_rate"]
        self.latency = int(latency)
        if self.latency < 0:
            raise ValueError(f"latency must be >= 0 (got {latency!r})")
        self.first_channel = 1 if int(first_channel) else 0
        #: Silence longer than this between two rising edges is counted as a stall
        #: (0 = never). A stopped clock produces no edges at all, so nothing else can see it.
        self.max_gap_ticks = int(max_gap_ticks)
        self.samples: list[RxSample] = []
        self.ignored_bits = 0
        self.truncated_words = 0
        self.stalls = 0
        self.max_gap = 0
        self._reset()

    # -- configuration ----------------------------------------------------------
    def bind(self, bus: Bus) -> "I2sReceiver":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.limits = i2s_limits(
            self.ns_per_tick, sample_rate=self.limits["sample_rate"], word_bits=self.word_bits,
            slots=self.slots, slot_bits=self.slot_bits,
            ticks_per_period=self.limits["period"] if self._period_given else None,
            profile=self.limits["profile"],
        )
        self._reset()
        return self

    # -- the log ---------------------------------------------------------------
    @property
    def values(self) -> list[int]:
        """Every decoded sample, sign-extended, in stream order."""
        return [s.sample for s in self.samples]

    @property
    def words(self) -> list[int]:
        """Every decoded word as the raw `word_bits`-wide pattern, in stream order."""
        return [s.value for s in self.samples]

    @property
    def left(self) -> list[int]:
        """The samples the receiver called channel 0, in order."""
        return [s.sample for s in self.samples if s.channel == 0]

    @property
    def right(self) -> list[int]:
        """The samples the receiver called channel 1, in order."""
        return [s.sample for s in self.samples if s.channel == 1]

    @property
    def frames(self) -> int:
        """Frames decoded: slots seen divided by `slots`. The slot still open does not count
        until it is written out, so this is the number of *whole* frames."""
        return len(self.samples) // self.slots

    @property
    def slots_decoded(self) -> int:
        """Words written out so far, which is what a test counts while a stream is running."""
        return len(self.samples)

    @property
    def pending(self) -> int:
        """Words decoded but not yet popped."""
        return len(self.samples) - self._consumed

    @property
    def busy(self) -> bool:
        """True while a slot is open (a WS change has been seen and its word not written out)."""
        return self._in_slot

    def pop(self) -> Optional[RxSample]:
        """Oldest word not yet popped, or None."""
        if self._consumed >= len(self.samples):
            return None
        item = self.samples[self._consumed]
        self._consumed += 1
        return item

    def flush(self) -> Optional[RxSample]:
        """Write out the slot that is still open, as a stream that ends does on real hardware.

        §6.2 writes a slot out "On the next WS-level change", which is an event a *finite*
        simulation never produces for the last word. A receiver in silicon has the same problem
        and codecs flush on a timeout; here the test says so explicitly, once, after the clock has
        stopped. Returns the sample written, or None if no slot was open.
        """
        if not self._in_slot:
            return None
        self._write_slot(self._ws_tick)
        self._in_slot = False
        return self.samples[-1]

    # -- the device -------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        bclk = pins.get(self.bclk_net, 0)
        ws = pins.get(self.ws_net, 1)
        if self._prev is None:
            self._prev = (bclk, ws)     # first tick: adopt the idle levels, see I2sChecker.tick
            return {}
        prev = self._prev
        self._prev = (bclk, ws)
        if bclk and not prev[0]:                       # a leading edge: latch SD as it reads
            self._rising(t, pins.get(self.sd_net, 0))
        if ws != prev[1]:                              # a WS transition opens/closes a slot
            self._ws_change(t)
        return {}

    # -- the state machine ------------------------------------------------------
    def _rising(self, t: int, sd: int) -> None:
        tick = t - self.latency
        if self._last_rise is not None:
            gap = tick - self._last_rise
            self.max_gap = max(self.max_gap, gap)
            if self.max_gap_ticks and gap > self.max_gap_ticks:
                self.stalls += 1
        self._last_rise = tick
        if not self._in_slot:
            return                                 # no WS transition yet: nothing to latch into
        self._cells += 1
        if self._cells <= self.word_bits:
            self._shift = (self._shift << 1) | sd
            if self._cells == 1:
                self._msb_tick = tick
        if self._cells > self.word_bits:
            self.ignored_bits += 1                  # "the bits after the LSB are ignored", §3.1

    def _ws_change(self, t: int) -> None:
        tick = t - self.latency
        if self._in_slot:
            self._write_slot(tick)
        self._in_slot = True
        self._ws_tick = tick
        self._cells = 0
        self._shift = 0
        self._msb_tick = None

    def _write_slot(self, tick: int) -> None:
        bits = self._cells
        if bits < self.word_bits:
            self.truncated_words += 1               # "the missing bits are set to zero", §3.1
        value = self._shift & ((1 << self.word_bits) - 1)
        channel = (self._slot + self.first_channel) % max(self.slots, 2)
        self.samples.append(RxSample(
            value=value, sample=word_to_sample(value, self.word_bits),
            channel=channel if self.slots > 1 else 0,
            frame=self._slot // self.slots, slot=self._slot, ws_tick=self._ws_tick,
            msb_tick=self._msb_tick, bits=bits,
        ))
        self._slot += 1

    def _reset(self) -> None:
        self.samples.clear()
        self.ignored_bits = 0
        self.truncated_words = 0
        self.stalls = 0
        self.max_gap = 0
        self._reset_state()

    def _reset_state(self) -> None:
        self._prev: Optional[tuple[int, int]] = None      # learned on the first tick
        self._in_slot = False
        self._cells = 0
        self._shift = 0
        self._msb_tick: Optional[int] = None
        self._ws_tick = 0
        self._slot = 0
        self._last_rise: Optional[int] = None
        self._consumed = 0


# ---- harness helpers ----------------------------------------------------------------
def _ensure_nets(bus: Bus, nets: tuple[str, str, str] = NETS,
                 left_level: int = LEFT) -> None:
    """Add the three push-pull I2S nets if they are not there yet.

    Push-pull, not open-drain: UM11732 §5 gives TTL output levels (VL < 0.4 V, VH > 2.4 V, both
    "able to drive one standard TTL input") and input thresholds (VIL = 0.8 V, VIH = 2.0 V), so
    every net drives both ways and `Bus` records contention instead of resolving a wired-AND.

    The pull-up values are the *idle* levels a controller holds: the bit clock low (it stops), WS
    at `1 - left_level` (the level of the channel that is not selected, so that the first slot
    boundary is a real transition, §3.2) and SD low. Getting these right matters: with the default
    pull-up of 1 the bit clock would read high before anything drives it and a checker would see
    a phantom trailing edge at tick 0. Whichever device creates the nets decides the WS pull-up,
    so attach the one whose `left_level` the bus should have first.
    """
    idle = {0: 0, 1: 1 - (1 if int(left_level) else 0), 2: 0}   # bclk low, ws idle, sd low
    for i, net in enumerate(nets):
        if net not in bus.nets:
            bus.add_net(net, push_pull=True, pullup=idle.get(i, 1))


def attach_transmitter(bus: Bus, **kwargs) -> I2sTransmitter:
    """Build an I2sTransmitter that takes its timing from `bus` and attach it."""
    _ensure_nets(bus, kwargs.get("nets", NETS), kwargs.get("left_level", LEFT))
    tx = I2sTransmitter(bus=bus, **kwargs)
    bus.attach(tx)
    return tx


def attach_receiver(bus: Bus, **kwargs) -> I2sReceiver:
    """Build an I2sReceiver that takes its timing from `bus` and attach it."""
    _ensure_nets(bus, kwargs.get("nets", NETS), kwargs.get("left_level", LEFT))
    rx = I2sReceiver(bus=bus, **kwargs)
    bus.attach(rx)
    return rx


def attach_checker(bus: Bus, **kwargs) -> "I2sChecker":
    """Build a passive I2sChecker for `bus` and attach it."""
    _ensure_nets(bus, kwargs.get("nets", NETS), kwargs.get("left_level", LEFT))
    chk = I2sChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk


def attach_bench(bus: Bus, tx: Optional[dict] = None, rx: Optional[dict] = None,
                 chk: Optional[dict] = None, nets: tuple[str, str, str] = NETS):
    """Transmitter + receiver + checker on one bus, from keyword dicts.

    `tx`, `rx` and `chk` are the keyword arguments of the three classes; None skips that device.
    All three take their geometry (`sample_rate`, `word_bits`, `slots`, `slot_bits`) from the
    dicts, so a test can give them different geometry on purpose -- which is how the checker's
    `tSU;SD` and `fSCK` rules are shown to be about the *receiver's* view of the bus and not
    just arithmetic. Every unspecified geometry keyword goes to all three.
    """
    _ensure_nets(bus, nets, (tx or rx or chk or {}).get("left_level", LEFT))
    geometry = {"sample_rate": 48_000.0, "word_bits": 16, "slots": 2}
    out = []
    for cls, spec in ((I2sTransmitter, tx), (I2sReceiver, rx), (I2sChecker, chk)):
        if spec is None:
            out.append(None)
            continue
        dev = cls(bus=bus, nets=nets, **{**geometry, **spec})
        bus.attach(dev)
        out.append(dev)
    return tuple(out)


def run_frames(bus: Bus, tx: I2sTransmitter, rx: Optional[I2sReceiver] = None,
               frames: Optional[int] = None, limit: int = 4_000_000) -> bool:
    """Run `tx`'s queue out, then `rx.flush()`, and check `frames` came out.

    False if the limit runs out first, or if `frames` were asked for and fewer arrived. The count
    is taken *after* the flush: §6.2 writes a slot out "On the next WS-level change", and the last
    word of a finite stream has none, so nothing can count it before the flush.
    """
    ok = bus.run_until(lambda b: not tx.busy, limit)
    if rx is not None:
        rx.flush()
        if frames is not None:
            ok = ok and rx.slots_decoded >= frames * rx.slots
    elif frames is not None:
        raise ValueError("run_frames(frames=) needs a receiver to count them")
    return ok


# ---- the checker --------------------------------------------------------------------
@dataclass(frozen=True)
class Violation:
    """One spec rule the bus did not meet, with the tick it was missed on."""

    tick: int
    rule: str            # see RULE_HELP: "tHC", "WS-1T", "SLOT", ...
    measured: int        # ticks the bus gave it
    required: int        # ticks the rule asks for (0 for a rule with no limit, e.g. a polarity)
    detail: str = ""

    def __str__(self) -> str:
        text = f"t={self.tick} {self.rule}"
        if self.required:
            text += f" {self.measured} ticks < {self.required}"
        else:
            text += f" {self.measured}"
        return text + (f" ({self.detail})" if self.detail else "")


@dataclass
class SlotEvent:
    """One slot the checker saw, decoded the way §6.2's receiver does."""

    tick: int                 # the WS transition that opened the slot
    index: int
    level: int
    cells: int                # data cells latched in the slot
    word: Optional[int]       # the word they make, None if the slot was not closed
    msb_tick: Optional[int] = None
    complete: bool = False

    def __str__(self) -> str:
        what = "?" if self.word is None else f"0x{self.word:x}"
        return f"t={self.tick} slot {self.index} ws={self.level} {self.cells} cells {what}"


class I2sChecker(Device):
    """Passive I2S bus monitor: decodes the traffic and checks the spec, every rule in `RULE_HELP`.

    It drives nothing, so it sees the bus exactly as an analyser would. It measures the resolved
    pin values and takes nothing off for a device reacting a tick late (see `verif/vip/uart.py`):
    every *interval* it measures is between two edges it saw, so the one-tick observation latency
    cancels. It measures you, not the devices.

    It is configured for the geometry it is judging -- `word_bits`, `slots`, `slot_bits`,
    `sample_rate`, `profile` -- and from that it derives: where every data cell must fall, where
    WS must change, how long every clock half may be, and how fast the bit clock must run. Points
    it cannot decide are documented rather than guessed:

    * Which channel comes first. The first WS transition it sees it calls slot 0, so `SLOT` and
      `WORD` are absolute from there on but the channel phase is only known once two slots have
      been seen; `first_slot_level` says what to expect on the first one.
    * tRC, the clock rise time (Table 3: <= 0.15Tr): an analogue edge is not observable at one
      tick of resolution, so the number is in the limits but no rule uses it.
    * A stalled clock: no edges means nothing to time. `I2sReceiver(max_gap_ticks=...)` counts a
      clock that runs *slow* (a gap between two rising edges); a clock that stops dead leaves both
      of its counters where they were.

    Every violation carries its tick, the measured number of ticks, the number the rule asks for,
    and a `detail` naming the part of the spec it came from. `clean`, `violations_by_rule()`,
    `report()` and the per-rule counters are what a test asks.
    """

    name = "i2s_check"

    def __init__(
        self,
        sample_rate: Optional[float] = 48_000.0,
        word_bits: int = 16,
        slots: int = 2,
        ns_per_tick: float = 20.0,
        bus: Optional[Bus] = None,
        ticks_per_period: Optional[int] = None,
        slot_bits: Optional[int] = None,
        profile: str = "relative",
        timing: Optional[dict] = None,
        left_level: int = LEFT,
        first_slot_level: Optional[int] = None,
        latency: int = 1,
        ws_slack_ticks: int = 1,
        ws_edge_slack_ticks: int = 0,
        ws_lag_ticks: Optional[int] = None,
        period_slack_ticks: int = 1,
        max_sck_hz: Optional[float] = None,
        max_rate_violations: int = 8,
        nets: tuple[str, str, str] = NETS,
        name: Optional[str] = None,
    ):
        self.bclk_net, self.ws_net, self.sd_net = nets
        if name:
            self.name = name
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.limits = i2s_limits(
            self.ns_per_tick, sample_rate=sample_rate, word_bits=word_bits, slots=slots,
            slot_bits=slot_bits, ticks_per_period=ticks_per_period, profile=profile, timing=timing,
        )
        self._period_given = ticks_per_period is not None
        self.word_bits = self.limits["word_bits"]
        self.slots = self.limits["slots"]
        self.slot_bits = self.limits["slot_bits"]
        self.sample_rate = self.limits["sample_rate"]
        self.profile = I2S_PROFILES[self.limits["profile"]]
        self.left_level = 1 if int(left_level) else 0
        self.first_slot_level = self.left_level if first_slot_level is None \
            else (1 if int(first_slot_level) else 0)
        self.latency = int(latency)
        if self.latency < 0:
            raise ValueError(f"latency must be >= 0 (got {latency!r})")
        self.ws_slack_ticks = int(ws_slack_ticks)
        self.ws_edge_slack_ticks = int(ws_edge_slack_ticks)
        self.ws_lag_ticks = None if ws_lag_ticks is None else int(ws_lag_ticks)
        self.period_slack_ticks = int(period_slack_ticks)
        self.max_sck_hz = None if max_sck_hz is None else float(max_sck_hz)
        self.max_rate_violations = int(max_rate_violations)
        self.violations: list[Violation] = []
        self.slots: list[SlotEvent] = []
        self.ws_ticks: list[int] = []
        self.periods: list[int] = []
        self.min_period: Optional[int] = None
        self.max_period: Optional[int] = None
        self.rate_violations = 0
        self._reset()

    # -- what the tests ask for --------------------------------------------------
    def bind(self, bus: Bus) -> "I2sChecker":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.limits = i2s_limits(
            self.ns_per_tick, sample_rate=self.limits["sample_rate"],
            word_bits=self.limits["word_bits"], slots=self.limits["slots"],
            slot_bits=self.limits["slot_bits"],
            ticks_per_period=self.limits["period"] if self._period_given else None,
            profile=self.limits["profile"],
        )
        self._reset()
        return self

    @staticmethod
    def rule_help() -> dict[str, str]:
        """The rules this checker enforces, one line each, for a test's failure message."""
        return dict(RULE_HELP)

    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def frames(self) -> int:
        """Frames the checker saw, from its first WS transition."""
        return len(self.slots) // self.limits["slots"]

    @property
    def max_sck_measured(self) -> Optional[float]:
        """The fastest bit clock the bus actually ran at, in Hz."""
        if self.min_period is None:
            return None
        return 1e9 / (self.ns_per_tick * self.min_period)

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def flush(self) -> Optional[SlotEvent]:
        """Close the slot that is still open, as a stream that ends does on real hardware.

        §6.2 writes a slot out "On the next WS-level change", which never comes for the last slot
        of a finite simulation; the SLOT and WORD checks still apply to it, and a slot that ran
        short is flagged here as it would be mid-stream. Returns the slot, or None if none was
        open.
        """
        if not self.in_slot:
            return None
        self._close_slot(self._now)
        self.in_slot = False
        return self.slots[-1]

    def report(self, limit: int = 10) -> str:
        head = (f"I2S {self.limits['word_bits']}-bit x{self.limits['slots']} in "
                f"{self.limits['slot_bits']}-bit slots, {self.limits['sample_rate']:.0f} Hz, "
                f"{self.limits['profile']} profile at {self.ns_per_tick} ns/tick: "
                f"{len(self.slots)} slots ({self.frames} frames), {len(self.periods)} periods, "
                f"{len(self.violations)} violations")
        if self.min_period is not None:
            head += (f", bit clock {1e9 / (self.ns_per_tick * self.max_period) / 1e6:.3f}"
                     f"-{1e9 / (self.ns_per_tick * self.min_period) / 1e6:.3f} MHz"
                     f" (nominal {self.limits['bit_clock_hz'] / 1e6:.3f})")
        if self.rate_violations > len(self.violations):
            head += f", {self.rate_violations} periods out of rate"
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: int, required: int, detail: str = "") -> None:
        if measured < required:
            self.violations.append(Violation(t, rule, measured, required, detail))

    def _exceeded(self, t: int, rule: str, measured: int, allowed: int, detail: str = "") -> None:
        if measured > allowed:
            self.violations.append(Violation(t, rule, measured, allowed, detail))

    # -- the monitor --------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        bclk = pins.get(self.bclk_net, 0)
        ws = pins.get(self.ws_net, 1)
        sd = pins.get(self.sd_net, 0)
        if self._prev is None:
            # first tick: adopt whatever the pins idle at instead of inventing a transition.
            # Same for a checker attached to a bus that is already running -- it has no history.
            self._prev = (bclk, ws, sd)
            return {}
        prev = self._prev
        self._prev = (bclk, ws, sd)
        now = t - self.latency
        rise_at = bclk == 1 and prev[0] == 0
        fall_at = bclk == 0 and prev[0] == 1
        self._now = now
        # The bit clock first: the cell latched by a rising edge that WS also changes on belongs
        # to the *outgoing* slot (§6.2 writes it out "On the next WS-level change").
        if rise_at:
            self._rise(now, sd)
        if fall_at:
            self._fall(now)
        if ws != prev[1]:
            self._ws_change(now, ws, rise_at=rise_at, fall_at=fall_at)
        if sd != prev[2]:
            self._data(now)
        return {}

    # -- measurement --------------------------------------------------------------
    def _p(self) -> int:
        """The period to judge fractional limits against: the last one measured, else nominal.

        Table 3 note [2]: "At all data rates in the controller mode, the transmitter or receiver
        generates a clock signal with a fixed mark/space ratio. For this reason tHC and tLC are
        specified with respect to T." So the 0.35T limits are judged against the period the bus
        is actually running at, not against a nominal one -- a legal 39-tick clock must not be
        flagged because it was configured for 40.
        """
        return self.period if self.period else self.limits["period"]

    def _frac(self, fraction: float, ceiling: bool, use_measured: bool) -> int:
        """One fractional limit in whole ticks, rounded so that it is never lenient.

        Table 3 note [2]: "At all data rates in the controller mode, the transmitter or receiver
        generates a clock signal with a fixed mark/space ratio. For this reason tHC and tLC are
        specified with respect to T" -- the period the bus is *running* at, which is what
        `use_measured` selects. The receiver's own `tsr` is a property of the device, so it is
        judged against the period it was configured for (`use_measured=False`).
        """
        period = self._p() if use_measured else self.limits["period"]
        return int(ceil(fraction * period)) if not ceiling else int(fraction * period)

    def _floor(self, t: int, rule: str, measured: int, fraction: str, ns_key: str,
               detail: str, ceiling: bool = False, use_measured: bool = True) -> None:
        """Check one limit: the fraction of the clock period, and any absolute ns limit."""
        ticks = self._frac(getattr(self.profile, fraction), ceiling, use_measured)
        if ceiling:
            if measured > ticks:
                self.violations.append(Violation(t, rule, measured, ticks, detail))
        else:
            self._violation(t, rule, measured, ticks, detail)
        limit_ns = self.limits[ns_key]
        if limit_ns:
            if ceiling:
                if measured > limit_ns:
                    self.violations.append(
                        Violation(t, rule, measured, limit_ns, detail + " (absolute limit)"))
            else:
                self._violation(t, rule, measured, limit_ns, detail + " (absolute limit)")

    def _rise(self, t: int, sd: int) -> None:
        """A leading edge: the receiver latches SD here ("the serial data must be latched into
        the receiver on the leading edge of the serial clock signal", §3.1)."""
        if self.last_fall is not None and self.period is not None:
            # Table 3 note [2] judges tHC/tLC "with respect to T", so the first two halves of a
            # stream are not judged: the period they belong to has not been measured yet.
            self._floor(t, "tLC", t - self.last_fall, "t_lc", "ns_lc", "bit clock LOW half")
        if self.last_rise is not None:
            self._period(t, t - self.last_rise)
        self.last_rise = t
        self.last_edge_tick, self.last_edge_level = t, 1
        if self._first_edge is None:
            self._first_edge = t
        self.edges += 1
        # tSU;SD: how long SD has been stable before the edge that latches it (Figure 3,
        # "tsr >= 0.2T"). This is the receiver's own limit, so it is judged against the nominal
        # period Tr, not against the measured one.
        self._floor(t, "tSU;SD", t - self.sd_since, "t_su", "ns_su",
                    f"SD last changed at t={self.sd_since}", use_measured=False)
        if self._ws_first_edge:
            self._ws_lead(t)
        if not self.in_slot:
            return
        self.cells += 1
        if self.cells <= self.limits["word_bits"]:
            self.shift = (self.shift << 1) | sd
            if self.cells == 1:
                self.msb_tick = t
        if self.cells > self.limits["slot_bits"] and not self.slot_overflow:
            # No WS change by the end of the slot: §6.1 loads the shift register at *every* WS
            # change, so a slot that runs on without one is not a slot any more.
            self.slot_overflow = True
            self.violations.append(Violation(
                t, "SLOT", self.cells, self.limits["slot_bits"],
                f"no WS transition within {self.limits['slot_bits']} clock periods; "
                "WS must change once per slot (§6.1)"))

    def _fall(self, t: int) -> None:
        """A trailing edge: the HIGH half just ended."""
        if self.last_rise is not None and self.period is not None:
            self._floor(t, "tHC", t - self.last_rise, "t_hc", "ns_hc", "bit clock HIGH half")
        self.last_fall = t
        self.last_edge_tick, self.last_edge_level = t, 0
        if self._first_edge is None:
            self._first_edge = t

    def _period(self, t: int, period: int) -> None:
        """fSCK: the bit-clock period ending at the leading edge seen on tick `t`."""
        self.period = period
        self.periods.append(period)
        self.min_period = period if self.min_period is None else min(self.min_period, period)
        self.max_period = period if self.max_period is None else max(self.max_period, period)
        nominal = self.limits["period"]
        bad = ""
        if abs(period - nominal) > self.period_slack_ticks:
            bad = (f"bit clock period {period} ticks "
                   f"({1e9 / (self.ns_per_tick * period) / 1e6:.3f} MHz), the stream should be "
                   f"{nominal} ticks ({self.limits['bit_clock_hz'] / 1e6:.3f} MHz) for "
                   f"{self.limits['sample_rate']:.0f} Hz")
        ns = period * self.ns_per_tick
        t_min = self.limits["ns_t_min"] * self.ns_per_tick
        t_max = self.limits["ns_t_max"] * self.ns_per_tick
        if t_min and ns < t_min:
            bad = bad or f"period {ns:.0f} ns below the {t_min:.0f} ns of Tables 1/2"
        if t_max and ns > t_max:
            bad = bad or f"period {ns:.0f} ns above the {t_max:.0f} ns of Tables 1/2"
        if self.max_sck_hz and ns < 1e9 / self.max_sck_hz:
            bad = bad or (f"period {ns:.0f} ns is faster than the "
                          f"{self.max_sck_hz / 1e6:.3f} MHz limit")
        if bad:
            self.rate_violations += 1
            if self._fsck_reported < self.max_rate_violations:
                self._fsck_reported += 1
                self.violations.append(Violation(t, "fSCK", period, nominal, bad))

    def _ws_lead(self, t: int) -> None:
        """WS-1T: how far ahead of this MSB-latching edge the WS change was.

        §3.1: "The transmitter always sends the MSB of the next word one clock period after the
        WS changes", with §6.2 putting the counter reset on the falling edge in between. One
        clock period is therefore the expected lead, always -- there is no second arm for a
        change that landed on a *trailing* edge, because that placement leaves only `t -
        last_fall` ticks (the low half) and is not one clock period at all: it is flagged.

        `ws_lag_ticks` is the opt-in for a part that really does shift WS earlier (Wikipedia: WS
        is "typically synchronized to the falling edge"). It states the lead in ticks, so
        `ws_lag_ticks=17` means "the lead is 17 ticks", not "expect WS early by some amount".
        """
        ws_tick = self._ws_tick
        self._ws_first_edge = False
        lag = t - ws_tick
        expected = self._p() if self.ws_lag_ticks is None else self.ws_lag_ticks
        if abs(lag - expected) > self.ws_slack_ticks:
            wanted = ("one clock period" if self.ws_lag_ticks is None
                      else f"ws_lag_ticks={self.ws_lag_ticks}")
            where = ("on a trailing edge, so only its low half was left"
                     if self._ws_on_trail else "on a leading edge")
            self.violations.append(Violation(
                t, "WS-1T", lag, expected,
                f"WS changed at t={ws_tick} ({where}), the MSB edge is {lag} ticks later, the spec "
                f"wants {expected} ({wanted}) ± {self.ws_slack_ticks} (§3.1/§3.2)"))

    def _ws_change(self, t: int, level: int, rise_at: bool, fall_at: bool) -> None:
        self._ws_on_trail = fall_at and not rise_at
        if self.last_edge_tick is None or \
                abs(t - self.last_edge_tick) > self.ws_edge_slack_ticks:
            self.violations.append(Violation(
                t, "WS-EDGE", abs(t - self.last_edge_tick) if self.last_edge_tick else -1, 0,
                "WS changed "
                f"{abs(t - self.last_edge_tick) if self.last_edge_tick else 'before any clock edge'} "
                "ticks from a bit-clock edge; §3.2: WS changes on a trailing or leading edge"))
        if self.in_slot:
            self._close_slot(t)
        self.in_slot = True
        self.cells = 0
        self.shift = 0
        self.msb_tick = None
        self.slot_overflow = False
        self._slot_level = level
        self._ws_tick = t
        self._ws_first_edge = True
        self.ws_ticks.append(t)
        if not self.ws_ticks[:-1] and self._first_edge is not None and \
                abs(t - self._first_edge) <= self.ws_edge_slack_ticks:
            self._aligned = True
        expected = self.first_slot_level if self.slot_index % 2 == 0 \
            else 1 - self.first_slot_level
        if level != expected and self._saw_from_start:
            self.violations.append(Violation(
                t, "WS-CHAN", level, expected,
                f"slot {self.slot_index} carries {'left' if expected == self.left_level else 'right'} "
                f"channel (WS={expected}, §3.2)"))

    @property
    def _saw_from_start(self) -> bool:
        """True when the checker's first slot is known to be a whole one.

        A checker attached to a bus that is already running cannot tell whether the first WS
        transition it sees opens a slot or closes one, nor which channel comes first (see the
        class docstring). It therefore judges SLOT, WORD and WS-CHAN only once it has seen a WS
        transition coincide with the *first* clock edge it saw -- which it has, for every slot,
        when it was attached before the stream began and the stream opens on a slot boundary.
        """
        return self._aligned

    def _close_slot(self, t: int) -> None:
        slot = SlotEvent(tick=self._ws_tick, index=self.slot_index, level=self._slot_level,
                         cells=self.cells, word=self.shift, msb_tick=self.msb_tick,
                         complete=(self.cells == self.limits["slot_bits"]
                                   and self.cells >= self.limits["word_bits"]))
        self.slot_index += 1
        if self._saw_from_start and not self.slot_overflow and \
                self.cells != self.limits["slot_bits"]:
            self.violations.append(Violation(
                t, "SLOT", self.cells, self.limits["slot_bits"],
                f"slot was {self.cells} clock periods, the geometry says "
                f"{self.limits['slot_bits']} (§6.1)"))
        if self._saw_from_start and self.cells < self.limits["word_bits"]:
            self.violations.append(Violation(
                t, "WORD", self.cells, self.limits["word_bits"],
                "the slot ended before the word did; §3.1 has the missing bits set to zero "
                "internally, so the word is short"))
        self.slots.append(slot)

    def _data(self, t: int) -> None:
        """SD moved. Checked only while a slot is open: an idle controller may stop driving SD."""
        self.sd_since = t
        if not self.in_slot:
            return
        delay = t - self.last_rise if self.last_rise is not None else self._p()
        if delay <= 0:
            # thtr >= 0 allows zero hold, but a change in the same tick as the latching edge is
            # a value the model cannot resolve and silicon would see as ambiguous.
            self.violations.append(Violation(
                t, "tH;TR", 0, self.limits["t_htr"],
                "SD changed in the same tick as the leading edge that latches it (§4 thtr >= 0, "
                "but a zero-tick hold cannot be resolved)"))
            return
        self._floor(t, "tD;TR", delay, "t_dtr", "ns_dtr", "SD delay after the leading edge",
                    ceiling=True)

    def _reset(self) -> None:
        self.violations.clear()
        self.slots.clear()
        self.ws_ticks.clear()
        self.periods.clear()
        self.min_period = None
        self.max_period = None
        self.rate_violations = 0
        self.edges = 0
        self._fsck_reported = 0
        self._reset_state()

    def _reset_state(self) -> None:
        self._prev: Optional[tuple[int, int, int]] = None   # learned on the first tick
        self.period: Optional[int] = None
        self.last_rise: Optional[int] = None
        self.last_fall: Optional[int] = None
        self.last_edge_tick: Optional[int] = None
        self.last_edge_level = 0
        # a very old SD change so that the setup time of the first latched bit (which the checker
        # cannot know about a bus it attached to mid-stream) is not reported as a violation
        self.sd_since = -10 ** 9
        self._first_edge: Optional[int] = None
        self._aligned = False
        self.in_slot = False
        self.cells = 0
        self.shift = 0
        self.msb_tick: Optional[int] = None
        self.slot_index = 0
        self.slot_overflow = False
        self._slot_level = self.first_slot_level
        self._ws_tick = 0
        self._ws_first_edge = False
        self._ws_on_trail = False
        self._now = 0
