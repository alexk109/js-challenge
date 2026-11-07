"""Prints the measured PS/2 evidence: the spec table in ticks, a real session, every rule broken.

This is the script that produces the PS/2 evidence numbers, kept in the tree so
they can be regenerated instead of retyped:

    .venv/bin/python verif/vip/ps2_evidence.py > /tmp/ps2-evidence.log

It is a plain script, not a test: the tests in `test_ps2.py` are the assertions, this is the
measurement. Every limit it prints is the one in `research/ref/protocols/ps2/`.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from verif.vip.core import Bus
from verif.vip.ps2 import (FRAME_BITS, HOST_FRAME_BITS, attach_checker, attach_device,
                           attach_host, attach_keyboard, attach_mouse, frame_ticks,
                           ps2_frame_bits, rate_minimums, ns_to_ticks)

NS = 20.0
print("== spec table in whole ticks at 20 ns/tick (research/ref/protocols/ps2) ==")
for rate in ("fast", "slow"):
    m = rate_minimums(NS, rate)
    keys = ("clk_low_min clk_low_max clk_high_min clk_high_max data_after_rise_min "
            "data_after_rise_max data_before_edge_min data_before_edge_max rts_hold "
            "bus_free_min inhibit_window min_period max_period clk_low clk_high "
            "start_bit_hold").split()
    print(f"  {rate:5s} " + "  ".join(f"{k}={m[k]}" for k in keys))
print(f"  nominal: fast = 2*{ns_to_ticks(NS, 30000)} ticks = "
      f"{1e6/(NS*2*ns_to_ticks(NS,30000)):.2f} kbit/s, "
      f"slow = 2*{ns_to_ticks(NS, 50000)} ticks = "
      f"{1e6/(NS*2*ns_to_ticks(NS,50000)):.2f} kbit/s")

print("\n== a full keyboard session on the wire ==")
bus = Bus(ns_per_tick=NS)
host = attach_host(bus); kbd = attach_keyboard(bus); chk = attach_checker(bus)
host.send(0xFF, 0xF4, 0xF0, 0x02, 0xED, 0x07, 0xF2, 0xEE)
host.run(bus, 5_000_000); kbd.run(bus, 5_000_000); bus.run(2000)
for key in ("a", "ctrl_l", "c"):
    pass
kbd.press("ctrl_l"); kbd.press("c"); kbd.release("c"); kbd.release("ctrl_l")
kbd.run(bus, 5_000_000); bus.run(2000)
print(f"  to the device: {[hex(b.value) for b in kbd.received]}")
print(f"  to the host:   {[hex(v) for v in host.values]}")
print(f"  frames: {chk.frames_to_host} to the host, {chk.frames_to_device} to the device, "
      f"{chk.acks} acks, {len(chk.violations)} violations")
print(f"  clock period on the wire: {chk.shortest_period}-{chk.longest_period} ticks "
      f"({chk.fastest_khz:.2f}-{chk.slowest_khz:.2f} kbit/s) over {chk.periods} periods")
print(f"  bus contention errors: {len(bus.errors)}")
print(f"  first frame on the wire: {chk.frames[0]}")
print(f"  ticks in one frame: to_host ~{frame_ticks(NS)}, to_device ~{frame_ticks(NS, direction='to_device')}")

print("\n== a mouse session ==")
bus = Bus(ns_per_tick=NS)
host = attach_host(bus); mouse = attach_mouse(bus); chk = attach_checker(bus)
host.send(0xFF, 0xF3, 100, 0xE8, 0x02, 0xF4)
host.run(bus, 5_000_000); mouse.run(bus, 5_000_000); bus.run(2000)
host.flush_rx()
mouse.move(10, -5, buttons=1)
mouse.run(bus, 5_000_000); bus.run(2000)
print(f"  to the host: {[hex(v) for v in host.values]}  packet signed {mouse.reports[-1].signed}")
print(f"  frames: {chk.frames_to_host} to the host, {chk.frames_to_device} to the device, "
      f"{len(chk.violations)} violations, contention {len(bus.errors)}")

print("\n== every rule, broken on purpose (rule: count of violations) ==")
def bench(**kw):
    b = Bus(ns_per_tick=NS)
    h = attach_host(b, rate=kw.pop("host_rate", "fast"), timing=kw.pop("host_timing", None))
    d = kw.pop("device", "bare")
    dev = (attach_keyboard(b, rate=kw.pop("dev_rate", "fast"), timing=kw.pop("dev_timing", None))
           if d == "kbd" else attach_device(b, rate=kw.pop("dev_rate", "fast"),
                                            timing=kw.pop("dev_timing", None)))
    c = attach_checker(b, rate=kw.pop("check_rate", "fast"), timing=kw.pop("check_timing", None))
    return b, h, dev, c

cases = [
    ("tCLK-low min", dict(dev_timing={"clk_low": 10_000})),
    ("tCLK-low max", dict(dev_timing={"clk_low": 60_000})),
    ("tCLK-high min", dict(dev_timing={"clk_high": 10_000})),
    ("tCLK-high max", dict(dev_timing={"clk_high": 60_000})),
    ("tSU;DAT min (device->host)", dict(dev_timing={"data_after_rise": 1_000})),
    ("tHD;DAT max (device->host)", dict(dev_rate="slow", dev_timing={"data_after_rise": 20_000})),
    ("data-window", None),   # filled in below
    ("tRTS", dict(device="kbd", host_timing={"rts_hold": 50_000})),
    ("tHD;DAT min (host->device)", dict(device="kbd", host_timing={"data_before_edge_min": 28_000})),
    ("tBUS-FREE", dict(dev_timing={"bus_free": 0})),
    ("stop-bit", None),
    ("parity", None),
]
from verif.vip.test_ps2 import LowPhaseData, StartGlitch, BadFrame, attach_double
for label, kw in cases:
    if kw is None:
        continue
    b, h, dev, c = bench(**kw)
    # tBUS-FREE is a rule about the gap between two frames, so that case sends two bytes.
    dev.queue(0xA5, 0x5A) if label == "tBUS-FREE" else dev.queue(0xA5)
    if label.startswith("tRTS") or label.startswith("tHD;DAT min (host"):
        h.send(0xF4)
        h.run(b, 1_000_000)
    dev.run(b, 1_000_000); b.run(2000)
    counts = {rule: len(c.violations_by_rule(rule)) for rule in c.RULES
              if c.violations_by_rule(rule)}
    first = f"  first: {c.violations[0]}" if c.violations else "  (clean!)"
    print(f"  {label:32s} -> {counts}{first}")

b = Bus(ns_per_tick=NS); h = attach_host(b)
dev = attach_double(b, LowPhaseData); c = attach_checker(b)
dev.queue(0xC3); dev.run(b, 1_000_000); b.run(2000)
print(f"  {'data-window':32s} -> " + str({r: len(c.violations_by_rule(r)) for r in c.RULES if c.violations_by_rule(r)}))
b = Bus(ns_per_tick=NS); h = attach_host(b)
dev = attach_double(b, StartGlitch); c = attach_checker(b)
dev.queue(0xA5); dev.run(b, 1_000_000); b.run(2000)
print(f"  {'start-bit':32s} -> " + str({r: len(c.violations_by_rule(r)) for r in c.RULES if c.violations_by_rule(r)}))
b = Bus(ns_per_tick=NS); h = attach_host(b, retries=0)
dev = attach_double(b, __import__("verif.vip.test_ps2", fromlist=["x"]).NoAck)
c = attach_checker(b)
h.send(0x55); h.run(b, 1_000_000); dev.run(b, 1_000_000); b.run(2000)
print(f"  {'ack-bit':32s} -> " + str({r: len(c.violations_by_rule(r)) for r in c.RULES if c.violations_by_rule(r)}))
for fault, rule in (("stop", "stop-bit"), ("parity", "parity")):
    b = Bus(ns_per_tick=NS); h = attach_host(b)
    dev = attach_double(b, BadFrame); dev.fault = fault
    c = attach_checker(b); dev.queue(0xA5); dev.run(b, 1_000_000); b.run(2000)
    print(f"  {rule + ' (fault=' + fault + ')':32s} -> " +
          str({r: len(c.violations_by_rule(r)) for r in c.RULES if c.violations_by_rule(r)}))
b = Bus(ns_per_tick=NS); h = attach_host(b)
dev = attach_double(b, __import__("verif.vip.test_ps2", fromlist=["x"]).ShortFrame)
c = attach_checker(b); dev.queue(0x3C); b.run(3 * c.frame_timeout)
print(f"  {'frame-length':32s} -> " + str({r: len(c.violations_by_rule(r)) for r in c.RULES if c.violations_by_rule(r)}))
