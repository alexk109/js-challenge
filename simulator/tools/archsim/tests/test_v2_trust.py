"""Trust view (calibration, assumptions, ISA estimates, data) and side kinds calibration / isa_effect."""
import html
import json
import re

CLASSES_3_5 = {"K1": "match", "K2": "match", "K3": "match", "K4": "off", "K5": "match", "K6": "not_modelled",
               "K7": "match", "K8": "off", "K9": "match", "K10": "identity", "K11": "close", "K12": "off",
               "K13": "identity", "K14": "match", "K15": "match", "K16": "match", "K17": "match", "K18": "close",
               "K20": "match"}


def trust(js, raw="{}"):
    return js("ArchSimUI.renderTrust(ArchSimUI.makeState(%s, DATA))" % raw)


def side(js, ref):
    return js("ArchSimUI.renderSide(ArchSimUI.makeState({side: %s}, DATA))" % json.dumps(ref))


def test_registered_and_sections(js):
    assert js("ArchSimUI.views.trust.title") == "Trust"
    h = trust(js)
    for k in ("calibration", "assumptions", "isa", "data"):
        assert 'data-trust-sec="%s"' % k in h


def test_calibration(js):
    h = trust(js)
    rows = re.findall(r'<tr data-cal="(K\d+)" data-class="(\w+)"', h)
    assert len(rows) == 20 and [r[0] for r in rows] == ["K%d" % i for i in range(1, 21)]
    cls = dict(rows)
    for k, c in CLASSES_3_5.items():
        assert cls[k] == c, k
    assert cls["K19"] == "brackets"
    assert "8 items" not in h and "16 items" in h             # K8 breakdown is expandable
    assert 'data-class-badge="match"' in h and 'data-class-badge="brackets"' in h and "<svg" in h
    assert "perf-a against perf-b" in h and "data-class-legend" in h


def test_class_legend_from_vocab(js, data):
    h = trust(js)
    for c in data["vocab"]["classes"]:
        assert c["meaning"] in html.unescape(h)


def test_legacy_only_differing_rows(js):
    h = trust(js)
    m = re.search(r'data-legacy-diff="(\d+)"', h)
    assert m and int(m.group(1)) >= 1
    sect = h[h.index("Legacy 19-row model"):]
    sect = sect[:sect.index("</details>")]
    assert "CAN 1 Mb/s" in sect and "isa_xor" in sect       # the one known disagreement on the MVP
    assert "UART 3 Mbaud" not in sect and "WS2812" not in sect   # agreeing rows are not listed


def test_inputs(js, data):
    h = trust(js)
    rows = re.findall(r'<tr data-param="([\w]+)"', h)
    assert rows == list(data["params"])
    for pid in data["params"]:
        for f in ("lo", "nom", "hi"):
            assert re.search(r'<input type="number"[^>]*data-param="%s" data-field="%s"' % (pid, f), h)
    assert "Not modelled" in h and "Silicon pad delay" in h
    assert 'data-edited="1"' not in h


def test_isa_params_flagged(js, data):
    h = trust(js)
    ids = [p for p in data["params"] if p.startswith("d_isa_")]
    assert len(ids) == 5
    for pid in ids:
        assert re.search(r'<tr data-param="%s" data-status="judgement" data-isa-param="1"' % pid, h)
    g = trust(js, "{view: {trust: {pstatus: 'isa'}}}")
    assert re.findall(r'<tr data-param="([\w]+)"', g) == ids


def test_inputs_override_flows_through(js, data):
    pid = "f_toggle_out_mhz"
    p = data["params"][pid]
    new = p["nom"] - 1
    st = "{arch: {param_overrides: {%s: {lo: %r, nom: %r, hi: %r}}}}" % (pid, min(p["lo"], new), new, p["hi"])
    h = trust(js, st)
    assert 'data-edited="1"' in h and ">edited<" in h
    assert re.search(r'data-param="%s" data-field="nom" value="%s"' % (pid, re.escape(repr(new) if new != int(new) else str(int(new)))), h)
    base = js("JSON.stringify(ArchSimUI.makeState({}, DATA).all.cells)")
    edited = js("JSON.stringify(ArchSimUI.makeState(%s, DATA).all.cells)" % st)
    assert base != edited                                      # the override reaches the cells


def test_status_filter_guess(js, data):
    g = trust(js, "{view: {trust: {pstatus: 'guess'}}}")
    n = len(re.findall(r'<tr data-param=', g))
    assert 0 < n < len(data["params"])
    assert all(s in ("judgement", "unverified", "disputed", "estimate")
               for s in re.findall(r'<tr data-param="\w+" data-status="(\w*)"', g))
    assert "what we are guessing" in g and "Show" in g


def test_isa_section(js, data):
    h = trust(js)
    isa = data["isa"]
    sect = h[h.index('data-trust-sec="isa"'):h.index('data-trust-sec="data"')]
    assert "These are estimates" in sect
    assert len(re.findall(r'data-m0="1"', sect)) == len(isa["m0_present"]) == 2
    assert "autopush" in sect and "autopull" in sect and "docs/mvp-isa.md" in sect
    keys = re.findall(r'<tr data-isa-effect="([^"]+)"', sect)
    assert len(keys) == len(isa["effects"]) == len(set(keys))
    for e in isa["effects"]:
        assert e["lever"] + "@" + e["wl"] + ("@%s" % e["value"] if "value" in e else "") in keys
        assert e["rationale"][:40].replace("&", "&amp;").replace("'", "&#39;").replace('"', "&quot;") in sect or e["rationale"][:40] in sect
    assert 'st-judgement' in sect and 'st-estimate' in sect and "d_words" in sect
    for lv in isa["levers"]:
        assert ">%s<" % lv in sect


def test_isa_effect_active_flag(js):
    h = trust(js, "{arch: {isa_xor: true}}")
    assert re.search(r'<tr data-isa-effect="isa_xor@can_1m" data-active="1"', h)
    assert not re.search(r'<tr data-isa-effect="isa_xor@can_1m" data-active', trust(js))


def test_side_calibration(js):
    d = side(js, {"kind": "calibration", "id": "K8"})
    assert "16 items" in d and "<details" in d and "open" in d and "class=\"badge cls" in d
    assert "No calibration row" in side(js, {"kind": "calibration", "id": "K99"})


def test_side_isa_effect(js, data):
    e = data["isa"]["effects"][0]
    d = side(js, {"kind": "isa_effect", "wl": e["wl"], "lever": e["lever"]})
    assert "An estimate" in d and "Rationale" in d and "judgement" in d
    assert e["cite"][0]["path"] in d and "d_isa_xor_ns" in d
    w = [x for x in data["isa"]["effects"] if "value" in x][0]
    d2 = side(js, {"kind": "isa_effect", "wl": w["wl"], "lever": w["lever"], "value": w["value"]})
    assert "width %s bit" % w["value"] in d2


def test_data_section(js, data):
    h = trust(js)
    assert "Input files (%d) with sha256" % len(data["sources"]) in h
    for p, sha in list(data["sources"].items())[:3]:
        assert sha in h
    for k in ("Blocks (area)", "Host links", "Workloads", "ISS constants"):
        assert k in h


def test_links_not_duplicated_results(js):
    h = trust(js)
    assert "Budgets" not in h and "Summary" not in h
