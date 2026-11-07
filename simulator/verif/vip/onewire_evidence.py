"""Evidence for the 1-Wire VIP: what the models measured on the wire.

Run from the repo root:

    .venv/bin/python verif/vip/onewire_evidence.py

It prints, for every number the 1-Wire model documents, the measurement
that produced it, so the printout and the claim cannot drift apart. Nothing here is a
hand-typed constant except the spec numbers, which are quoted from
research/ref/protocols/onewire/ and printed next to what the model derived from them.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from verif.vip.core import Bus                                    # noqa: E402
from verif.vip.onewire import (                                   # noqa: E402
    ONEWIRE_SPEEDS,
    RULES,
    Ds18b20Target,
    attach_checker,
    attach_ds18b20,
    attach_master,
    crc8,
    speed_minimums,
)


def line(text=""):
    print(text)


def head(text):
    line()
    line(text)
    line("-" * len(text))


def bench(ns_per_tick, speed="standard", check_speed=None, master_timing=None,
          **target_kwargs):
    """The same bench the tests use, so a number here and a number there are the
    same number."""
    bus = Bus(ns_per_tick=ns_per_tick)
    master = attach_master(bus, speed=speed, timing=master_timing)
    target = attach_ds18b20(bus, speed=speed, **target_kwargs)
    checker = attach_checker(bus, speed=check_speed or speed)
    return bus, master, target, checker


def main() -> int:
    stamp = sys.argv[1] if len(sys.argv) > 1 else "(no stamp given)"
    line("1-Wire VIP evidence")
    line(f"code state: {stamp}")
    line(f"python: {sys.version.split()[0]}")

    head("1. The spec tables, and what they become in whole ticks")
    for speed in ("standard", "overdrive"):
        spec = ONEWIRE_SPEEDS[speed]
        line(f"{speed}:")
        line(f"  limits in ns: t_rstl={spec.t_rstl} t_pdhigh={spec.t_pdhigh} "
             f"t_pdlow={spec.t_pdlow} t_rsth={spec.t_rsth}")
        line(f"              t_low1={spec.t_low1} t_low0={spec.t_low0} "
             f"t_slot={spec.t_slot} t_rec={spec.t_rec} t_rdv={spec.t_rdv}")
        line(f"              t_tx0={spec.t_tx0} t_read_low={spec.t_read_low}")
        line(f"  master drives: w_rstl={spec.w_rstl} w_low1={spec.w_low1} "
             f"w_low0={spec.w_low0} w_read_low={spec.w_read_low}")
        line(f"                 w_sample={spec.w_sample} w_slot={spec.w_slot} "
             f"w_post_presence={spec.w_post_presence}")
        for ns in (20.0, 100.0):
            m = speed_minimums(ns, speed)
            line(f"  at {ns:5.1f} ns/tick: t_rstl={m['t_rstl']} t_pdhigh={m['t_pdhigh']} "
                 f"t_pdlow={m['t_pdlow']} t_rsth={m['t_rsth']}")
            line(f"                    t_low1={m['t_low1']} t_low0={m['t_low0']} "
                 f"t_slot={m['t_slot']} t_rec={m['t_rec']} t_rdv={m['t_rdv']}")
            line(f"                    t_tx0={m['t_tx0']} t_read_low={m['t_read_low']}")

    head("2. What the master actually puts on the wire, and what the checker measures")
    for speed in ("standard", "overdrive"):
        for ns in (20.0, 100.0):
            bus, master, target, checker = bench(ns, speed)
            target.set_temperature(0x0191)            # 25.0625 C
            master.queue_reset()
            op = master.queue_read_scratchpad()
            started = time.perf_counter()
            assert master.run(bus, limit=60_000_000)
            checker.finish()
            wall = time.perf_counter() - started
            t_rstl = checker.t_rstl_min
            line(f"{speed:9s} {ns:5.1f} ns/tick  reset low {master.rst_low} ticks "
                 f"({master.rst_low * ns / 1000:.1f} us, limit {t_rstl}.."
                 f"{checker.t_rstl_max})")
            line(f"{'':9s} {'':5s}          presence {target.presence_delay} ticks after "
                 f"the release, {target.presence_low} ticks long "
                 f"(limits {checker.t_pdhigh_min}..{checker.t_pdhigh_max} / "
                 f"{checker.t_pdlow_min}..{checker.t_pdlow_max})")
            line(f"{'':9s} {'':5s}          slot {master.slot} ticks "
                 f"({master.slot * ns / 1000:.1f} us, limits {checker.t_slot_min}.."
                 f"{checker.t_slot_max}); write-1 low {master.low1}, write-0 low "
                 f"{master.low0}, read low {master.read_low}, sample at {master.rdv}")
            line(f"{'':9s} {'':5s}          read-0 driven from {target.tx0_start} to "
                 f"{target.tx0_end} ticks into its slot "
                 f"(tRDV {checker.t_rdv}, tTX0 {checker.t_tx0_min}..{checker.t_tx0_max})")
            line(f"{'':9s} {'':5s}          {checker.slots} slots "
                 f"({checker.write_slots} written, {checker.read_slots} read), "
                 f"{checker.bytes} bytes, {checker.crc_checks} CRC check(s), "
                 f"{bus.t} ticks total")
            line(f"{'':9s} {'':5s}          scratchpad read back correctly: "
                 f"{bytes(op.data) == bytes(target.scratchpad)}  "
                 f"violations: {checker.violations}")
            line(f"{'':9s} {'':5s}          {wall:.2f} s of wall clock for those "
                 f"{bus.t} ticks")

    head("3. Every rule the checker documents, and the broken traffic that trips it")
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "verif" / "vip"))
    import test_onewire as T                                # noqa: E402

    assert set(T.BROKEN) == set(RULES), "BROKEN and RULES disagree"
    line(f"{len(RULES)} rules: {', '.join(RULES)}")
    for rule in RULES:
        chk = T.BROKEN[rule]()
        hits = chk.violations_by_rule(rule)
        first = hits[0]
        line(f"  {rule:12s} {len(hits):4d} hit(s), first at tick {first.tick}: {first}")

    head("4. The checker is not vacuous")
    bus, master, target, checker = bench(100.0, check_speed="overdrive")
    master.queue_read_scratchpad()
    assert master.run(bus, limit=60_000_000)
    checker.finish()
    line(f"legal standard-speed wire against overdrive limits: "
         f"{len(checker.violations)} violations, rules "
         f"{', '.join(sorted(checker.rules_seen()))}")
    line("  " + "\n  ".join(str(v) for v in checker.violations[:4]))
    bus2, master2, target2, checker2 = bench(100.0)
    op = master2.queue_read_scratchpad()
    assert master2.run(bus2, limit=60_000_000)
    checker2.finish()
    line(f"the same bench at its own speed: {len(checker2.violations)} violations, "
         f"data correct: {bytes(op.data) == bytes(target2.scratchpad)}")

    head("5. Constants the model quotes from the datasheets")
    target = Ds18b20Target()
    line(f"CRC-8/MAXIM of b'123456789' = 0x{crc8(b'123456789'):02X} "
         f"(the check value for x^8+x^5+x^4+1)")
    line(f"power-up scratchpad: {bytes(target.scratchpad).hex()} "
         f"(+85 C = 0x0550, TH 0x4B, TL 0x00, config 0x7F, reserved FF FF 10, CRC)")
    for bits in (9, 10, 11, 12):
        line(f"  tCONV at {bits:2d} bits = "
             f"{Ds18b20Target.conv_time_ticks(bits, 20.0) * 20.0 / 1e6:.2f} ms "
             f"({Ds18b20Target.conv_time_ticks(bits, 20.0)} ticks at 20 ns)")
    return 0


if __name__ == "__main__":
    sys.exit(main())