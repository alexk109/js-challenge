"""Lockstep of the parametric RTL (`src/pio_chip.sv` at env BENCH_CONFIG) against the parametric ISS.

The RTL runs under the firmware harness (`fw/rtl_harness.py`, Verilator),
the ISS is `pio_iss_p.Chip(Params)`, and both get the same pins and the same one bus access per cycle.
`verif/fuzz/lockstep.py` does this for the MVP map; this file does it for any config, with a stimulus
generator that knows the config's register map (`fw.regmap.RegMap`), including the 10-bit wide map, PAGE
in EXECCTRL and the engine_route ROUTE_EN bits in CTRL.

Compared every cycle: the read data of a host read, `pin_out`, `pin_oe`, `irq0`, `irq1`, `state(e)` of every
engine, and (from the one RTL snapshot of that cycle) the FIFO contents, the IMEM words and the CLKDIV /
EXECCTRL / SHIFTCTRL / PINCTRL registers, the CTRL route bits, FDEBUG and the IRQ enables. Every host read is
also checked inside the harness against the read mux of the harness (`RtlChip.step`).

    .venv/bin/python -m tools.bench.sim.lockstep_rtl_p --config '{"n_engines":3,"fifo_depth":8}' \\
        --seeds 1-3 --cycles 3000

Exit status 1 on the first mismatch of any seed (printed with the last cycles of both sides).
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys
from typing import Optional

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

M32 = 0xFFFFFFFF


def _stimulus(seed: int, cycles: int, rm, route_on: bool) -> dict:
    """{cycle: ("r", addr) | ("w", addr, data)}: setup, start, then random host traffic.

    Addresses come from the config's `RegMap`. The engines run the MVP generator's programs (so the
    whole instruction set, FIFO use and pin writes are covered); with more than 32 IMEM words the extra
    pages are filled with random words and PAGE is set at random, so jumps and wraps land in them.
    """
    from verif.fuzz import gen
    rng = random.Random((seed * 0x27D4EB2F + 0x1F3D5B79) & M32)
    n, words = rm.n_engines, rm.imem_words
    ops: dict[int, tuple] = {}
    cur = [0]

    def put(op: tuple) -> None:
        ops[cur[0]] = op
        cur[0] += 1

    def w(addr: int, data: int) -> None:
        put(("w", addr, data & M32))

    def r(addr: int) -> None:
        put(("r", addr))

    page_bits = max(0, (words // 32).bit_length() - 1)

    def execctrl(base: int) -> int:
        # EXECCTRL[21:20] = PAGE, stored only above 32 words
        page = rng.randrange(1 << page_bits) if page_bits else 0
        return (base & ~(3 << 20)) | (page << 20) | (rng.getrandbits(2) << 20 if rng.random() < 0.1 else 0)

    w(0x00, 0)                                       # CTRL = 0
    images = []
    for e in range(n):
        scfg = gen.SideCfg(rng.choice([0, 1, 2, 3, 3, 4, 5]), rng.random() < 0.35, rng.random() < 0.3, rng)
        bodies = gen.cover_bodies(rng)
        chunk = bodies[: min(32, words)]
        prog = gen.build_program(chunk, scfg)
        prog += [gen.mov(gen.MOV_Y, gen.MOV_NONE, gen.MOV_SRC_Y)] * (words - len(prog))
        extra = [rng.getrandbits(16) for _ in range(words)]
        images.append((prog, extra, scfg))
    wrap_stress = words == 16 and seed % 3 == 0     # the engines keep their wrap window (no reconfiguration below)
    pipeline = route_on and seed % 2 == 1 and not wrap_stress     # engines run pull/push loops, so the links carry traffic
    nop = gen.mov(gen.MOV_Y, gen.MOV_NONE, gen.MOV_SRC_Y)
    loop = [gen.pull(block=True), gen.mov(gen.MOV_ISR, gen.MOV_NONE, gen.MOV_SRC_OSR),
            gen.push(block=True), gen.jmp(gen.JMP_ALWAYS, 0)]
    for e in range(n):
        prog, extra, scfg = images[e]
        if pipeline:
            prog = loop + [nop] * (words - 4)
        for k in range(min(words, 32)):
            w(rm.imem(e, k), prog[k] if k < len(prog) else 0)
        for k in range(32, words):
            w(rm.imem(e, k), extra[k])
        wrap_bottom = rng.randrange(32)
        if wrap_stress:                   # 16 words: wrap window whose bottom is 16..31 (truncated to 4 bits) and top < 16
            w(rm.sm(e, 0), 1 << 16)
            w(rm.sm(e, 1), (rng.randrange(3, 15) << 12) | (rng.randrange(16, 32) << 7))
            w(rm.sm(e, 2), 0)
            w(rm.sm(e, 5), 0)
            continue
        if pipeline:
            w(rm.sm(e, 0), 1 << 16)
            w(rm.sm(e, 1), 3 << 12)
            w(rm.sm(e, 2), 0)
            w(rm.sm(e, 5), 0)
            continue
        w(rm.sm(e, 0), gen.gen_clkdiv(rng))
        w(rm.sm(e, 1), execctrl(gen.gen_execctrl(rng, (wrap_bottom, min(31, wrap_bottom + rng.randrange(0, 12))))))
        w(rm.sm(e, 2), gen.gen_shiftctrl(rng))
        w(rm.sm(e, 5), gen.gen_pinctrl(rng, scfg))
    mask_n = (1 << n) - 1
    route = (rng.getrandbits(max(n - 1, 1)) | 1) if route_on else 0
    w(0x00, mask_n | (mask_n << 8) | (rng.getrandbits(n) << 4) | (route << 16))   # enable, clkdiv restart, route

    weights = [("idle", 8), ("txf", 22), ("rxf", 12), ("read_any", 6), ("write_any", 3), ("instr", 3),
               ("imem", 4), ("smcfg", 3), ("irq", 3), ("fdebug", 3), ("ctrl", 3), ("sweep", 1)]
    if wrap_stress:
        weights = [(k, 0 if k in ("write_any", "instr", "imem", "smcfg") else v) for k, v in weights]
    if pipeline:                               # keep the pipeline alive: no reconfiguration, drain only the last RX
        weights = [(k, 0 if k in ("write_any", "instr", "imem", "smcfg", "fdebug") else v) for k, v in weights]
    names = [k for k, _ in weights]
    wts = [v for _, v in weights]
    forced = gen.FORCED
    while cur[0] < cycles - 4:
        what = rng.choices(names, wts)[0]
        e = rng.randrange(n)
        if what == "idle":
            cur[0] += 1
        elif what == "txf":
            w(rm.txf(e), rng.getrandbits(32))
        elif what == "rxf":
            r(rm.rxf(n - 1 if pipeline and rng.random() < 0.7 else e))
        elif what == "read_any":
            r(rng.randrange(rm.addr_space))
        elif what == "write_any":
            w(rng.randrange(rm.addr_space), rng.getrandbits(32))
        elif what == "instr":
            _, build = forced[rng.randrange(len(forced))]
            w(rm.sm(e, 4), build(rng))
        elif what == "imem":
            w(rm.imem(e, rng.randrange(words)), rng.getrandbits(16))
        elif what == "smcfg":
            i = rng.choice([0, 1, 2, 5])
            if i == 0:
                v = gen.gen_clkdiv(rng)
            elif i == 1:
                v = execctrl(gen.gen_execctrl(rng))
            elif i == 2:
                v = gen.gen_shiftctrl(rng)
            else:
                v = gen.gen_pinctrl(rng, images[e][2])
            w(rm.sm(e, i), v)
        elif what == "irq":
            a = rng.choice([0x4B, 0x4C, 0x4E, 0x4F])
            w(a, rng.getrandbits(8))
        elif what == "fdebug":
            w(0x02, rng.getrandbits(32))
        elif what == "ctrl":
            route = (rng.getrandbits(max(n - 1, 1)) if rng.random() < 0.5 else (1 << max(n - 1, 1)) - 1) if route_on else 0
            if pipeline:
                w(0x00, mask_n | (((1 << (n - 1)) - 1) << 16))
                continue
            w(0x00, mask_n | (rng.getrandbits(n) << 4) | (rng.getrandbits(n) << 8) | (route << 16)
              | (rng.getrandbits(32) & 0xFFFE0000 if rng.random() < 0.05 else 0))
        elif what == "sweep":                           # a read of every register that has one
            for a in [0x00, 0x01, 0x02, 0x03, 0x0F, 0x10, 0x11, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50]:
                r(a)
            for ee in range(n):
                for i in range(6):
                    r(rm.sm(ee, i))
    return ops


def run_seed(rtl, iss_cls, params, seed: int, cycles: int, window: int = 12) -> Optional[str]:
    """None if RTL and ISS agree for `cycles` cycles, else a mismatch report."""
    from fw import rtl_harness as H
    from verif.fuzz import gen
    from verif.fuzz.lockstep import pad_value
    rm = params.regmap
    ops = _stimulus(seed, cycles, rm, params.engine_route)
    rtl.reset()
    iss = iss_cls(params)
    hist: list[str] = []
    stats = {"reads": 0, "writes": 0, "route_moves": 0, "dual_push": 0, "max_tx": 0, "max_rx": 0,
             "fdebug_seen": 0, "max_pc": 0, "wrap_trunc": 0}
    for c in range(cycles):
        pin = pad_value(gen.pin_vector(seed, c), iss.pin_out, iss.pin_oe)
        op = ops.get(c)
        bus = None if op is None else (("r", op[1]) if op[0] == "r" else ("w", op[1], op[2]))
        n_e = params.n_engines
        for e in range(n_e - 1):                        # coverage: link moves (ISS rule, start of cycle)
            if (iss.route >> e) & 1 and iss.engines[e].rx and len(iss.engines[e + 1].tx) < params.fifo_depth:
                stats["route_moves"] += 1
                if op is not None and op[0] == "w" and op[1] == rm.txf(e + 1):
                    stats["dual_push"] += 1
        for e in range(n_e):                            # coverage: a wrap whose bottom needed truncating
            en = iss.engines[e]
            if (iss.sm_enable >> e) & 1 and en.pc == en.wrap_top and en.wrap_bottom > en.pmask:
                stats["wrap_trunc"] += 1
        ra = iss.step(pin, bus)
        rb = rtl.step(pin, bus)
        stats["fdebug_seen"] |= iss.fdebug
        stats["max_pc"] = max([stats["max_pc"]] + [iss.engines[e].pc for e in range(n_e)])
        if op is not None:
            stats["reads" if op[0] == "r" else "writes"] += 1
        problems = []
        if op is not None and op[0] == "r" and ra != rb:
            problems.append(f"bus_rdata@{op[1]:#05x}: iss {ra:#010x} rtl {rb:#010x}")
        for k in ("pin_out", "pin_oe", "irq0", "irq1"):
            if getattr(iss, k) != getattr(rtl, k):
                problems.append(f"{k}: iss {getattr(iss, k):#x} rtl {getattr(rtl, k):#x}")
        snap = rtl._snapshot()
        for e in range(params.n_engines):
            sa, sb = iss.state(e), snap["engines"][e]
            for k in H.STATE_KEYS:
                if sa[k] != sb[k]:
                    problems.append(f"eng{e}.{k}: iss {sa[k]:#x} rtl {sb[k]:#x}")
            ea = iss.engines[e]
            for name, a, b in (("tx", ea.tx, sb["tx"]), ("rx", ea.rx, sb["rx"]), ("imem", ea.imem, sb["imem"]),
                               ("clkdiv", ea.clkdiv, sb["sm"][0]), ("shiftctrl", ea.shiftctrl, sb["sm"][2]),
                               ("pinctrl", ea.pinctrl, sb["sm"][5]),
                               ("execctrl", ea.execctrl, sb["sm"][1] & 0x7FFFFFFF)):
                if a != b:
                    problems.append(f"eng{e}.{name}: iss {a} rtl {b}")
            stats["max_tx"] = max(stats["max_tx"], len(ea.tx))
            stats["max_rx"] = max(stats["max_rx"], len(ea.rx))
        if snap["sm_enable"] != iss.sm_enable or snap["route_en"] != iss.route:
            problems.append(f"ctrl: iss {iss.sm_enable:#x}/{iss.route:#x} rtl {snap['sm_enable']:#x}/{snap['route_en']:#x}")
        if H.read_mux(0x02, snap) != iss.peek(0x02):
            problems.append(f"fdebug: iss {iss.peek(2):#x} rtl {H.read_mux(2, snap):#x}")
        for a in (0x4B, 0x4C, 0x4E, 0x4F):
            if H.read_mux(a, snap) != iss.peek(a):
                problems.append(f"{a:#x}: iss {iss.peek(a):#x} rtl {H.read_mux(a, snap):#x}")
        hist.append(f"c{c} op={op} pin={pin:#x} out={rtl.pin_out:#x}/{rtl.pin_oe:#x} "
                    + " ".join(f"e{e}:pc{snap['engines'][e]['pc']} tx{snap['engines'][e]['tx_level']} rx{snap['engines'][e]['rx_level']}"
                               for e in range(params.n_engines)))
        if problems:
            return f"seed {seed} cycle {c}: " + "; ".join(problems) + "\nlast cycles:\n  " + "\n  ".join(hist[-window:])
    print("    coverage: " + ", ".join(f"{k}={v:#x}" if k == "fdebug_seen" else f"{k}={v}" for k, v in stats.items()))
    return None


def _seeds(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=os.environ.get("BENCH_CONFIG", ""), help="JSON bench config (default: env BENCH_CONFIG, else the MVP)")
    ap.add_argument("--seeds", default="1-3")
    ap.add_argument("--cycles", type=int, default=3000)
    args = ap.parse_args(argv)
    if args.config.strip():
        os.environ["BENCH_CONFIG"] = args.config                # read by the harness, the Makefile, the ISS
    os.environ.setdefault("FW_TARGET", "rtl")
    os.environ.setdefault("PYTHON", sys.executable)
    from fw import rtl_harness as H
    from tools.bench.sim import pio_iss_p
    params = pio_iss_p.Params.from_env()
    print(f"lockstep_rtl_p: {params}  regmap wide={params.regmap.wide}")
    rtl = H.RtlChip()
    bad = 0
    for seed in _seeds(args.seeds):
        msg = run_seed(rtl, pio_iss_p.Chip, params, seed, args.cycles)
        print(f"  seed {seed}: " + ("OK" if msg is None else "MISMATCH"), flush=True)
        if msg:
            print(msg)
            bad += 1
            break
    print(f"lockstep_rtl_p: {'FAIL' if bad else 'PASS'} ({len(_seeds(args.seeds))} seeds x {args.cycles} cycles)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
