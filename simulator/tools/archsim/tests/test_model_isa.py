"""A4 (PLAN-v2 2.4.3, Part 4): ISA lever knobs, effect rows in cells, IMEM width scaling, ISA blocks and fmax terms,
the isa_encoding chip item and the isa_xor limit rule. Every ISA number is a judgement from inputs.json.isa."""
import json
import pytest

ISA_DEFAULTS = dict(isa_irq=False, isa_xor=False, isa_word_bits=16, isa_cond=False, isa_mov_src=False)
ISA_KNOBS = ["isa_irq", "isa_xor", "isa_word_bits", "isa_cond", "isa_mov_src"]


def A(**kw):
    return "Object.assign(ArchSim.defaultArch(), %s)" % json.dumps(kw)


def ap(x, rel=1e-3):
    return pytest.approx(x, rel=rel)


def cell(js, arch, wl):
    return js("ArchSim.evaluateCell(%s, '%s', DATA)" % (arch, wl))


def lim(c, lid):
    return next(r for r in c["limits"] if r["id"] == lid)["value"]


def row(data, wl, lever, value=None):
    return next(e for e in data["isa"]["effects"] if e["wl"] == wl and e["lever"] == lever and e.get("value") == value)


def chip(js, arch):
    return js("ArchSim.chipCheck(%s, DATA)" % arch)


def citem(ck, iid):
    return next(i for i in ck["items"] if i["id"] == iid)


def counts(js, arch, tag="cap19"):
    return js("ArchSim.evaluateAll(%s, DATA, '%s', {lite: true}).counts" % (arch, tag))


def verdicts(js, arch, tag="cap19"):
    return {k: v["verdict"] for k, v in js("ArchSim.evaluateAll(%s, DATA, '%s', {lite: true}).cells" % (arch, tag)).items()}


# ------------------------------------------------------------------ knobs
def test_isa_knobs_in_group(js):
    knobs = {k["id"]: k for k in js("ArchSim.KNOBS")}
    for kid in ISA_KNOBS:
        assert knobs[kid]["group"] == "ISA" and knobs[kid]["default"] == ISA_DEFAULTS[kid]
    assert knobs["isa_word_bits"]["options"] == [16, 24, 32] and knobs["isa_word_bits"]["type"] == "enum"
    assert "ISA" in js("ArchSim.GROUPS")
    arch = js("ArchSim.defaultArch()")
    assert {k: arch[k] for k in ISA_KNOBS} == ISA_DEFAULTS
    assert js("ArchSim.checkArch(%s)" % A(isa_word_bits=24)) == []
    assert js("ArchSim.checkArch(%s)" % A(isa_word_bits=20)) != []


# ------------------------------------------------------------------ all knobs off: Appendix V unchanged
def test_levers_off_reproduce_appendix_v(js, data):
    off = A(**ISA_DEFAULTS)
    ck = chip(js, off)
    assert [i["id"] for i in ck["items"]] == ["core_clock", "die_area", "board_clock", "pins_budget"]
    assert ck["verdict"] == "pass" and ck["driver"] == "core_clock"
    assert citem(ck, "core_clock")["value"]["nom"] == ap(11.6)
    v = citem(ck, "die_area")["value"]
    assert (v["nom"], v["pess"], v["opt"]) == (ap(289220), ap(156769), ap(408350))
    d = js("ArchSim.derive(%s, DATA)" % off)
    assert d["area"]["total"]["nom"] == ap(342472, 1e-3)
    assert [b["id"] for b in d["area"]["blocks"] if b["id"].startswith("isa_")] == []
    assert [p["id"] for p in js("ArchSim.clockBudget(%s, DATA)" % off)["parts"]] == ["base", "imem", "die"]
    assert dict(counts(js, off)) == {"served": 4, "marginal": 0, "impossible": 15}
    assert dict(counts(js, off, "all")) == {"served": 21, "marginal": 4, "impossible": 31}
    for pid, want in [("stack_a", (4, 0, 15)), ("stretch_usb_sd", (5, 0, 14)), ("wide_host", (9, 0, 10)), ("levers_60", (4, 0, 15)), ("usb_fs_48", (4, 0, 15))]:
        c = counts(js, "ArchSim.applyPreset('%s', DATA)" % pid)
        assert (c["served"], c["marginal"], c["impossible"]) == want, pid
    # no cell term refers to an ISA row, and no workload sums a non-zero delta
    bad = js("(function () { var n = 0; DATA.workloads.forEach(function (w) { var c = ArchSim.evaluateCell(%s, w, DATA);"
             " c.limits.forEach(function (r) { r.terms.forEach(function (t) { if (t.ref && t.ref.kind === 'isa_effect') n++; }); }); }); return n; })()" % off)
    assert bad == 0


# ------------------------------------------------------------------ instruction width: IMEM area
def test_word_bits_scales_imem_exactly(js, data):
    base = js("ArchSim.derive(ArchSim.defaultArch(), DATA)")
    pnr = data["params"]["pnr_factor"]["nom"]
    synth = data["blocks"]["imem_ff_32x16"]["area"]["nom"]
    assert synth == 42627
    imem = lambda d: next(b for b in d["area"]["blocks"] if b["id"] == "imem")
    d32 = js("ArchSim.derive(%s, DATA)" % A(isa_word_bits=32))
    assert imem(d32)["area_cell"]["nom"] == pytest.approx(2 * imem(base)["area_cell"]["nom"], rel=1e-12)
    dec = data["blocks"]["isa_decode_32"]["area"]["nom"]
    want = 2 * synth * pnr + 2 * dec * pnr
    got = d32["area"]["total"]["nom"] - base["area"]["total"]["nom"]
    assert got == pytest.approx(want, rel=1e-9)
    assert 2 * synth * pnr == ap(110273, 1e-4)             # PLAN V.6: +110,273 um2 of IMEM nominal
    assert [b["id"] for b in d32["area"]["blocks"] if b["id"].startswith("isa_")] == ["isa_decode_32"]
    assert next(b for b in d32["area"]["blocks"] if b["id"] == "isa_decode_32")["count"] == 2
    d24 = js("ArchSim.derive(%s, DATA)" % A(isa_word_bits=24))
    assert imem(d24)["area_cell"]["nom"] == pytest.approx(1.5 * imem(base)["area_cell"]["nom"], rel=1e-12)
    assert [b["id"] for b in d24["area"]["blocks"] if b["id"].startswith("isa_")] == ["isa_decode_24"]
    # latch scales the same way; an sram2p macro is picked for the wider word
    dl = js("ArchSim.derive(%s, DATA)" % A(imem_tech="latch")); dl32 = js("ArchSim.derive(%s, DATA)" % A(imem_tech="latch", isa_word_bits=32))
    assert imem(dl32)["area_cell"]["nom"] == pytest.approx(2 * imem(dl)["area_cell"]["nom"], rel=1e-12)
    ds = js("ArchSim.derive(%s, DATA)" % A(imem_tech="sram2p", isa_word_bits=32))
    mac = js("ArchSim.pickMacro(DATA, 2, 32, 32)")
    m = next(b for b in ds["area"]["blocks"] if b["id"] == "imem_macro")
    assert m["area_cell"]["nom"] == ap(2 * mac["area"], 1e-9) and mac["copies"] * mac["bits"] >= 32
    # the width terms are in the synth-area record
    assert any(t["sym"] == "bits" and t["ref"] == {"kind": "knob", "id": "isa_word_bits"} for t in d32["records"]["area_synth"]["terms"])


def test_isa_blocks_per_lever(js, data):
    for knob, blk, per in [("isa_irq", "isa_irq_logic", 1), ("isa_xor", "isa_xor_alu", 2), ("isa_cond", "isa_cond_logic", 2), ("isa_mov_src", "isa_mov_src_mux", 2)]:
        d = js("ArchSim.derive(%s, DATA)" % A(**{knob: True}))
        b = [x for x in d["area"]["blocks"] if x["id"].startswith("isa_")]
        assert [x["id"] for x in b] == [blk] and b[0]["count"] == per, knob
        assert data["blocks"][blk]["status"] == "judgement"
        assert b[0]["area_cell"]["nom"] == ap(per * data["blocks"][blk]["area"]["nom"], 1e-9)
        # the band enters the vertex: pessimistic cell area is the hi end
        assert b[0]["area_cell"]["pess"] == ap(per * data["blocks"][blk]["area"]["hi"], 1e-9)
    d = js("ArchSim.derive(%s, DATA)" % A(n_engines=3, isa_xor=True))
    assert next(x for x in d["area"]["blocks"] if x["id"] == "isa_xor_alu")["count"] == 3


# ------------------------------------------------------------------ fmax
def test_isa_fmax_terms(js, data):
    base = js("ArchSim.derive(ArchSim.defaultArch(), DATA)")
    r = data["params"]["r_typ_slow"]["nom"]
    for knob, pid in [("isa_xor", "d_isa_xor_ns"), ("isa_irq", "d_isa_irq_ns"), ("isa_cond", "d_isa_cond_ns"), ("isa_mov_src", "d_isa_mov_src_ns"),
                      ("isa_word_bits", "d_isa_word_ns")]:
        d = js("ArchSim.derive(%s, DATA)" % A(**{knob: 24 if knob == "isa_word_bits" else True}))
        p = data["params"][pid]
        assert p["status"] == "judgement" and p["lo"] == 0
        f0 = base["fmax"]["typ"]["nom"]
        assert d["fmax"]["typ"]["nom"] == ap(1000 / (1000 / f0 + p["nom"] / r), 1e-9), knob
        assert d["fmax"]["typ"]["pess"] == ap(1000 / (1000 / base["fmax"]["typ"]["pess"] + p["hi"] / r), 1e-3), knob
        assert any(t["ref"] == {"kind": "param", "id": pid} for t in d["records"]["fmax_typ"]["terms"])
        cb = js("ArchSim.clockBudget(%s, DATA)" % A(**{knob: 24 if knob == "isa_word_bits" else True}))
        part = next(x for x in cb["parts"] if x["id"] == "isa")
        assert part["ns"]["nom"] == ap(p["nom"] / r, 1e-9) and part["ref"] == {"kind": "param", "id": pid}
        assert cb["D"]["nom"] == ap(sum(x["ns"]["nom"] for x in cb["parts"]), 1e-9)
    slow = js("ArchSim.derive(%s, DATA)" % A(isa_xor=True, signoff_corner="slow"))
    assert slow["fmax"]["slow"]["nom"] == ap(1000 / (1000 / base["fmax"]["slow"]["nom"] + data["params"]["d_isa_xor_ns"]["nom"]), 1e-9)


# ------------------------------------------------------------------ isa_xor limit and can_1m
def test_isa_xor_on_can_1m(js, data):
    r = row(data, "can_1m", "isa_xor")
    ipu, words = data["workloads"][[w["id"] for w in data["workloads"]].index("can_1m")]["ipu"], 25
    assert ipu == 10 and r["status"] == "judgement" and r["enables"] == ["isa_xor"]
    off = cell(js, A(), "can_1m")
    assert off["verdict"] == "impossible" and "isa_xor" in off["blocked_by"] and lim(off, "isa_xor")["nom"] == -1
    on = cell(js, A(isa_xor=True), "can_1m")
    # the CRC need is met by the instruction: no isa_xor block; the cost is paid in cycles (and words)
    assert "isa_xor" not in on["blocked_by"] and lim(on, "isa_xor") == {"nom": 1, "pess": 1, "opt": 1}
    c = lim(on, "cycles")
    assert c["nom"] == 50 - (ipu + r["d_ipu"]["nom"]) and c["pess"] == 50 - (ipu + r["d_ipu"]["hi"]) and c["opt"] == 50 - (ipu + r["d_ipu"]["lo"])
    # +10 words (6..18) on 25 used do not fit 32 words: honest result, not tuned
    pw = lim(on, "program_words")
    assert (pw["nom"], pw["pess"], pw["opt"]) == (32 - (words + r["d_words"]["nom"]), 32 - (words + r["d_words"]["hi"]), 32 - (words + r["d_words"]["lo"]))
    assert pw["nom"] < 0 <= pw["opt"] and on["verdict"] == "marginal" and on["blocked_by"] == []
    # terms carry the isa_effect ref
    cyc = next(x for x in on["limits"] if x["id"] == "cycles")
    assert any(t["ref"] == {"kind": "isa_effect", "wl": "can_1m", "lever": "isa_xor"} for t in cyc["terms"])
    assert "ISA effect band" in cyc["note"]
    # with the CRC unit present the unit does the CRC: the row is not paid
    both = cell(js, A(isa_xor=True, crc_unit=True), "can_1m")
    assert both["verdict"] == "served" and lim(both, "cycles")["nom"] == 40 and lim(both, "cycles")["pess"] == 40
    assert cell(js, A(crc_unit=True), "can_1m")["verdict"] == "served"


def test_cost_only_row_pays_cycles_without_enabling(js, data):
    r = row(data, "usb_fs_device", "isa_xor")
    assert r["enables"] == []
    b, o = cell(js, A(), "usb_fs_device"), cell(js, A(isa_xor=True), "usb_fs_device")
    assert lim(o, "cycles")["nom"] == lim(b, "cycles")["nom"] - r["d_ipu"]["nom"]
    assert not any(x["id"] == "isa_xor" for x in o["limits"])


def test_cond_row_shortens_reflex_and_words(js, data):
    r = row(data, "spi_flash_read_decode", "isa_cond")
    b, o = cell(js, A(), "spi_flash_read_decode"), cell(js, A(isa_cond=True), "spi_flash_read_decode")
    T = 20.0
    assert lim(o, "reflex")["nom"] == ap(lim(b, "reflex")["nom"] - r["d_reflex_sm"]["nom"] * T, 1e-9)
    assert lim(o, "program_words")["nom"] == lim(b, "program_words")["nom"] - r["d_words"]["nom"]
    assert lim(o, "program_words")["pess"] == lim(b, "program_words")["nom"] - r["d_words"]["hi"]
    assert b["verdict"] == "impossible" and o["verdict"] == "marginal"
    t = next(x for x in o["limits"] if x["id"] == "reflex")["terms"]
    assert any(x["ref"]["kind"] == "isa_effect" and x["ref"]["lever"] == "isa_cond" for x in t if x["ref"])


def test_word_bits_row_selection(js, data):
    # 32 bits uses the 32 row if present, else the 24 row; 24 bits never uses a 32 row
    r32 = row(data, "usb_ls_device", "isa_word_bits", 32)
    b = cell(js, A(), "usb_ls_device")
    c24, c32 = cell(js, A(isa_word_bits=24), "usb_ls_device"), cell(js, A(isa_word_bits=32), "usb_ls_device")
    assert lim(c24, "program_words") == lim(b, "program_words")
    assert lim(c32, "program_words")["nom"] == lim(b, "program_words")["nom"] - r32["d_words"]["nom"]
    r24 = row(data, "fw_onewire", "isa_word_bits", 24)
    b, c24, c32 = cell(js, A(), "fw_onewire"), cell(js, A(isa_word_bits=24), "fw_onewire"), cell(js, A(isa_word_bits=32), "fw_onewire")
    assert lim(c24, "program_words")["nom"] == lim(b, "program_words")["nom"] - r24["d_words"]["nom"]
    assert lim(c32, "program_words") == lim(c24, "program_words")


# ------------------------------------------------------------------ encoding room
def test_isa_encoding_chip_item(js, data):
    assert "isa_encoding" not in [i["id"] for i in chip(js, A())["items"]]
    ck = chip(js, A(isa_irq=True, isa_cond=True))
    it = citem(ck, "isa_encoding")
    assert ck["verdict"] == "fail" and ck["driver"] == "isa_encoding" and it["verdict"] == "fail"
    assert (it["value"]["nom"], it["value"]["pess"], it["value"]["opt"]) == (-1, -1, -1) and it["unit"] == "slots"
    assert ck["records"]["isa_encoding"]["terms"]
    ck24 = chip(js, A(isa_irq=True, isa_cond=True, isa_word_bits=24))
    assert ck24["verdict"] == "pass" and citem(ck24, "isa_encoding")["value"]["nom"] == 0
    # a single lever fits at 16 bits; margin = min(major, minor) slots left
    assert citem(chip(js, A(isa_irq=True)), "isa_encoding")["value"]["nom"] == 0
    assert citem(chip(js, A(isa_xor=True)), "isa_encoding")["value"]["nom"] == 1
    assert citem(chip(js, A(isa_xor=True, isa_mov_src=True)), "isa_encoding")["value"]["nom"] == 1
    # irq + cond at 16 bits is a chip fail but the cells still evaluate (a chip fail is not a protocol limit)
    assert "isa_encoding" not in [r["id"] for r in cell(js, A(isa_irq=True, isa_cond=True), "can_1m")["limits"]]


def test_chip_fail_is_reported_by_knob_impact(js):
    # the encoding clash is a chip breakage the explore solvers report (never hidden)
    ki = js("ArchSim.knobImpact(%s, DATA)" % A(isa_irq=True))
    alt = next(a for k in ki["knobs"] if k["knob"] == "isa_cond" for a in k["alternatives"])
    assert alt["value"] is True and alt["chip"]["verdict"] == "fail" and alt["chip"]["driver"] == "isa_encoding"
    assert alt["chip"]["margin"]["nom"] == -1 and alt["chip"]["unit"] == "slots"
    # irq alone leaves 0 slots (margin 0, pass)
    base = js("ArchSim.chipCheck(%s, DATA)" % A(isa_irq=True))
    assert base["verdict"] == "pass"


# ------------------------------------------------------------------ refs and evidence
def test_every_isa_ref_resolves(js, data):
    res = js("""(function () {
      var archs = [%s, %s, %s, %s], bad = [], nIsa = 0;
      archs.forEach(function (a, ai) {
        var d = ArchSim.derive(a, DATA), ids = Object.keys(d.records), ck = ArchSim.chipCheck(a, DATA, {derived: d});
        function walk(owner, rec, extra) {
          rec.terms.forEach(function (t) {
            if (t.ref && t.ref.kind === 'isa_effect') nIsa++;
            if (!t.ref || !ArchSim.refResolves(t.ref, DATA, ids.concat(extra))) bad.push([ai, owner, rec.id, t.sym, t.ref]);
            if (t.value === undefined) bad.push([ai, owner, rec.id, t.sym, 'no value']);
          });
        }
        ids.forEach(function (k) { walk('arch', d.records[k], []); });
        Object.keys(ck.records).forEach(function (k) { walk('chip', ck.records[k], ids); });
        DATA.workloads.forEach(function (w) {
          var c = ArchSim.evaluateCell(a, w, DATA, {derived: d}), extra = Object.keys(c.records).concat(ids);
          c.limits.forEach(function (r) { walk(w.id, r, extra); });
        });
      });
      return {bad: bad, nIsa: nIsa};
    })()""" % (A(isa_irq=True, isa_xor=True, isa_cond=True, isa_mov_src=True, isa_word_bits=32), A(isa_xor=True, isa_word_bits=24),
               A(isa_cond=True, streaming="out_exec"), A(isa_irq=True, isa_cond=True)))
    assert res["bad"] == [] and res["nIsa"] > 20
    assert js("ArchSim.refResolves({kind: 'isa_effect', wl: 'can_1m', lever: 'isa_xor'}, DATA)") is True
    assert js("ArchSim.refResolves({kind: 'isa_effect', wl: 'can_1m', lever: 'isa_irq'}, DATA)") is False
    assert js("ArchSim.refResolves({kind: 'isa_effect', wl: 'usb_ls_device', lever: 'isa_word_bits', value: 32}, DATA)") is True
    assert js("ArchSim.refResolves({kind: 'isa_effect', wl: 'usb_ls_device', lever: 'isa_word_bits', value: 24}, DATA)") is False


# ------------------------------------------------------------------ what would it take (A3 solvers see the levers)
def test_what_would_it_take_can_1m(js):
    mvp = js("ArchSim.whatWouldItTake(ArchSim.defaultArch(), 'can_1m', DATA)")
    # honest: the XOR instruction needs +10 words on 25 used of 32, so it reaches only 'marginal', not 'served'
    assert [[x["knob"] for x in c["changes"]] for c in mvp["single"]] == [["crc_unit"]]
    assert mvp["single"][0]["estimate"] is False and mvp["single"][0]["judged_inputs"] == []
    # with room for the program (64 words) both routes serve: evidence-backed crc_unit first, the estimate isa_xor second
    r = js("ArchSim.whatWouldItTake(%s, 'can_1m', DATA)" % A(imem_words=64))
    order = [[x["knob"] for x in c["changes"]] for c in r["single"]]
    assert ["crc_unit"] in order and ["isa_xor"] in order
    assert order.index(["crc_unit"]) < order.index(["isa_xor"])
    xor = r["single"][order.index(["isa_xor"])]
    assert xor["estimate"] is True and "isa_effect:isa_xor:can_1m" in xor["judged_inputs"]
    assert "block:isa_xor_alu" in xor["judged_inputs"] and "param:d_isa_xor_ns" in xor["judged_inputs"]
    assert r["single"][order.index(["crc_unit"])]["estimate"] is False
    print("computed order for imem_words=64:", order)


# ------------------------------------------------------------------ what flips on the MVP
def test_flips_on_mvp(js):
    def flips(arch, tag):
        b, a = verdicts(js, A(), tag), verdicts(js, arch, tag)
        return sorted((k, b[k], a[k]) for k in b if a[k] != b[k])
    for tag in ("cap19", "all"):
        assert flips(A(isa_irq=True), tag) == []
        assert flips(A(isa_mov_src=True), tag) == []
        assert flips(A(isa_word_bits=24), tag) == [] and flips(A(isa_word_bits=32), tag) == []
    assert flips(A(isa_xor=True), "cap19") == [("can_1m", "impossible", "marginal")]
    assert flips(A(isa_cond=True), "cap19") == []
    assert flips(A(isa_cond=True), "all") == [("spi_flash_read_decode", "impossible", "marginal")]
    assert flips(A(isa_xor=True), "all") == [("can_1m", "impossible", "marginal")]
    # knobImpact sees the same flips
    ki = js("ArchSim.knobImpact(ArchSim.defaultArch(), DATA)")
    got = {(k["knob"], str(a["value"])): [(f["wl"], f["from"], f["to"]) for f in a["flips"]] for k in ki["knobs"] if k["knob"].startswith("isa_") for a in k["alternatives"]}
    assert got[("isa_xor", "True")] == [("can_1m", "impossible", "marginal")]
    assert all(v == [] for k, v in got.items() if k != ("isa_xor", "True"))
