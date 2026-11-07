"use strict";
// Evidence bench front end. Vanilla JS, no build step. Data comes from /api/* (live) or window.__BENCH__ (static export).
(function () {
  const STATIC = typeof window.__BENCH__ !== "undefined";
  const S = { knobs: [], support: {}, backends: [], levels: [], workloads: [], records: [], findings: [], jobs: [],
              filters: {}, cmp: { metric: "", workload: "" }, sel: { backends: new Set(), workloads: new Set() }, wparams: {} };

  const LEVEL_HELP = {
    analytic: "formula model (archsim)", iss: "instruction-set simulator, cycle-exact", rtl: "RTL simulation",
    gl: "gate-level simulation", flow: "synthesis and place-and-route", power: "power estimate",
    import: "past evidence read in from old files" };
  const METHOD_HELP = {
    measured: "a simulator or tool produced it", derived: "a formula over measured inputs",
    estimate: "a judgement, so treat as soft", spec: "taken from a specification", imported: "read from an old file" };
  const VERDICT_HELP = {
    match: "levels agree (within the metric's tolerance, or their bands overlap)",
    close: "levels differ by a small, tolerated amount",
    off: "levels disagree. This is a result, not something to tune away",
    identity: "the model was handed this number by the other side, so agreement proves nothing",
    "one-sided": "only one level has a value, so there is nothing to compare yet" };

  // ---------------------------------------------------------------- helpers
  function h(tag, props) {
    const el = document.createElement(tag);
    for (const [k, v] of Object.entries(props || {})) {
      if (v === undefined || v === null || v === false) continue;
      if (k === "class") el.className = v;
      else if (k === "text") el.textContent = v;
      else if (k.startsWith("on")) el.addEventListener(k.slice(2), v);
      else if (k === "value" || k === "checked" || k === "disabled" || k === "selected") el[k] = v;
      else el.setAttribute(k, v === true ? "" : v);
    }
    for (let i = 2; i < arguments.length; i++) add(el, arguments[i]);
    return el;
  }
  function add(el, ...cs) {
    for (const c of cs) {
      if (c === undefined || c === null || c === false) continue;
      if (Array.isArray(c)) c.forEach((x) => add(el, x));
      else el.appendChild(typeof c === "object" ? c : document.createTextNode(String(c)));
    }
  }
  const $ = (id) => document.getElementById(id);
  function stable(o) {
    if (Array.isArray(o)) return "[" + o.map(stable).join(",") + "]";
    if (o && typeof o === "object") return "{" + Object.keys(o).sort().map((k) => JSON.stringify(k) + ":" + stable(o[k])).join(",") + "}";
    return JSON.stringify(o);
  }
  const gkey = (r) => [r.config_hash, r.row_workload || r.workload, stable(r.row_params || r.workload_params || {}), r.metric].join("|");
  function num(v) {
    if (typeof v !== "number") return String(v);
    if (Number.isInteger(v)) return v.toLocaleString("en-US");
    return String(Number(v.toPrecision(4)));
  }
  const badge = (cls, text, title) => h("span", { class: "badge " + cls, title: title || "", text });
  const levelBadge = (lv) => badge("lv lv-" + lv, lv, LEVEL_HELP[lv] || "");
  const methodBadge = (m) => badge("m-" + m, m, METHOD_HELP[m] || "");
  function valueCell(r) {
    if (r.status === "n/a") return h("span", { class: "na", title: r.note || "" }, "n/a", r.note ? h("span", { class: "band", text: r.note }) : null);
    if (r.status === "error") return h("span", { class: "err", title: r.note || "" }, "error", r.note ? h("span", { class: "band", text: r.note.trim().split("\n").pop() }) : null);
    const v = r.value === null || r.value === undefined ? "n/a" : num(r.value);
    return h("span", null, v, r.unit ? h("span", { class: "band", text: r.unit }) : null,
      r.band ? h("span", { class: "band", title: "pessimistic .. optimistic", text: "[" + r.band.map(num).join(" .. ") + "]" }) : null,
      r.status === "fail" ? badge("s-fail", "fail") : null);
  }
  function uniq(list) { return Array.from(new Set(list)).sort(); }
  function option(v, t, sel) { return h("option", { value: v, text: t === undefined ? v : t, selected: sel }); }

  async function get(path) {
    if (STATIC) {
      const D = window.__BENCH__;
      if (path === "/api/knobs") return D.knobs;
      if (path === "/api/workloads") return D.workloads;
      if (path === "/api/records") return D.records;
      if (path === "/api/check") return D.findings;
      if (path === "/api/jobs") return [];
      throw new Error("not in export: " + path);
    }
    const r = await fetch(path);
    if (!r.ok) throw new Error(path + ": " + r.status);
    return r.json();
  }

  // ---------------------------------------------------------------- loading
  // BEGIN findIdx
  // One entry per row key: the WORST pairwise finding (off > close > identity > match > one-sided), so a row never reads
  // "match" while another pair of its sources is off. `detail` names the worst pair.
  const VERDICT_RANK = { off: 4, close: 3, identity: 2, match: 1, "one-sided": 0 };
  function indexFindings(findings) {
    const idx = {};
    findings.forEach((x) => {
      const k = [x.config_hash, x.workload, stable(x.workload_params || {}), x.metric].join("|");
      const cur = idx[k];
      if (!cur || (VERDICT_RANK[x.verdict] || 0) > (VERDICT_RANK[cur.verdict] || 0)) {
        const pair = x.b ? x.a.backend + " vs " + x.b.backend : "";
        idx[k] = Object.assign({}, x, { detail: (pair ? pair + ": " : "") + (x.detail || "") });
      }
    });
    return idx;
  }
  // END findIdx
  async function loadAll() {
    const [k, w, r, f, j] = await Promise.all([get("/api/knobs"), get("/api/workloads"), get("/api/records"), get("/api/check"), get("/api/jobs")]);
    const isFake = (b) => /^fake/.test(b) || b.includes(":");   // test backends and imported-result sources (import:x:y) cannot be launched
    S.knobs = k.knobs; S.support = k.support || {}; S.backends = (k.backends || []).filter((b) => !isFake(b)); S.levels = k.levels || [];
    S.workloads = w.map((x) => Object.assign({}, x, { backends: x.backends.filter((b) => !isFake(b)) })).filter((x) => x.backends.length); S.records = r; S.findings = f; S.jobs = j;
    S.findIdx = indexFindings(f);
    $("source").textContent = (STATIC ? "static export, " : "live store, ") + r.length + " records";
  }
  async function reloadData() {
    const [r, f] = await Promise.all([get("/api/records"), get("/api/check")]);
    S.records = r; S.findings = f; S.findIdx = indexFindings(f);
    $("source").textContent = (STATIC ? "static export, " : "live store, ") + r.length + " records";
  }

  // ---------------------------------------------------------------- configure
  const parseKnob = (k, s) => {
    s = String(s).trim();
    if (k.kind === "bool") return /^(true|yes|on|1)$/i.test(s);
    if (k.kind === "int" || k.kind === "float") return Number(s);
    if (k.kind === "enum") { const m = k.domain.find((d) => String(d) === s); return m === undefined ? s : m; }
    return s;
  };
  function parseVary(k, text) {
    text = text.trim();
    if (!text) return null;
    if (/^all$/i.test(text)) return k.kind === "bool" ? [false, true] : k.kind === "enum" ? k.domain.slice() : null;
    return text.split(/[\s,]+/).filter(Boolean).map((s) => parseKnob(k, s));
  }
  function readForm() {
    const base = {}, vary = {};
    for (const k of S.knobs) {
      const el = $("kv-" + k.id), vt = $("kvary-" + k.id).value;
      const v = k.kind === "bool" ? el.value === "true" : parseKnob(k, el.value);
      if (v !== k.default && !(typeof v === "number" && isNaN(v))) base[k.id] = v;
      const list = parseVary(k, vt);
      if (list && list.length) vary[k.id] = list;
    }
    return { base, vary };
  }
  function recompute() {
    if (!$("runcount")) return;
    const { base, vary } = readForm();
    const nv = Object.values(vary).reduce((a, l) => a * l.length, 1);
    const nw = S.sel.workloads.size, nb = S.sel.backends.size;
    $("runcount").textContent = nv * nw * nb + " runs (" + nv + " config" + (nv > 1 ? "s" : "") + " x " + nw + " workload" + (nw === 1 ? "" : "s") + " x " + nb + " backend" + (nb === 1 ? "" : "s") + "), before backends refuse configs they cannot run";
    const go = $("launch");
    go.disabled = STATIC || !nw || !nb;
    for (const k of S.knobs) {
      const touched = k.id in base || k.id in vary;
      S.backends.forEach((b) => {
        const c = $("sup-" + k.id + "-" + b); if (!c) return;
        const sup = (S.support[b] || {})[k.id] || "";
        c.style.fontWeight = touched && S.sel.backends.has(b) && sup === "default_only" ? "700" : "";
        c.style.color = touched && S.sel.backends.has(b) && sup === "default_only" ? "var(--bad)" : "";
      });
    }
  }
  function supCell(b, k) {
    const s = (S.support[b] || {})[k.id];
    let cls = "default_only", text = "not reported", title = "this backend did not report support for the knob";
    if (s === "full") { cls = "full"; text = "full"; title = "this backend runs any value"; }
    else if (s === "default_only") { text = "MVP only"; title = "this backend runs only the default value; it will refuse other values"; }
    else if (s && s.startsWith("partial")) { cls = "partial"; text = "partial"; title = s; }
    return h("td", { class: "sup " + cls, id: "sup-" + k.id + "-" + b, title, text });
  }
  function knobRow(k) {
    let input;
    if (k.kind === "bool") input = h("select", { id: "kv-" + k.id, onchange: recompute }, option("false", "off", !k.default), option("true", "on", k.default));
    else if (k.kind === "enum") input = h("select", { id: "kv-" + k.id, onchange: recompute }, k.domain.map((d) => option(String(d), String(d), d === k.default)));
    else input = h("input", { id: "kv-" + k.id, type: "number", step: k.kind === "int" ? 1 : "any", min: k.domain[0], max: k.domain[1], value: k.default, oninput: recompute });
    const ph = k.kind === "bool" ? "all, or off/on" : k.kind === "enum" ? "all, or " + k.domain.slice(0, 2).join(", ") : "e.g. " + [k.domain[0], k.default, k.domain[1]].filter((x, i, a) => a.indexOf(x) === i).join(", ");
    const range = k.kind === "int" || k.kind === "float" ? " (" + k.domain[0] + "-" + k.domain[1] + ")" : "";
    return h("tr", null,
      h("td", { title: k.help || "" }, k.label, h("span", { class: "muted", text: " " + k.id })),
      h("td", { class: "val" }, h("div", { style: "display:flex;gap:4px;align-items:center" }, input, h("span", { class: "muted", style: "white-space:nowrap", text: k.unit || "" }))),
      h("td", { class: "vary" }, h("input", { id: "kvary-" + k.id, type: "text", placeholder: ph, oninput: recompute })),
      h("td", { class: "muted", style: "white-space:nowrap", text: "MVP: " + (k.kind === "bool" ? (k.default ? "on" : "off") : k.default) + range }),
      S.backends.map((b) => supCell(b, k)));
  }
  const defParams = (w) => Object.entries(w.params || {}).map(([a, b]) => a + "=" + b).join(" ");
  function wlPick(w) {
    const has = Object.keys(w.params || {}).length > 0;
    const inp = has ? h("input", { type: "text", class: "wparam", id: "wp-" + w.id, placeholder: defParams(w), size: 38, title: "parameters of " + w.id + " as k=v pairs; empty = the defaults shown",
      disabled: STATIC, style: S.sel.workloads.has(w.id) ? "" : "display:none", oninput: (e) => { S.wparams[w.id] = e.target.value; } }) : null;
    if (inp) inp.value = S.wparams[w.id] || "";
    return h("span", { style: "display:inline-flex;gap:4px;align-items:center" },
      h("label", { title: (w.label ? w.label + ". " : "") + "runs on: " + (w.backends.join(", ") || "none yet") },
        h("input", { type: "checkbox", checked: S.sel.workloads.has(w.id), disabled: STATIC, onchange: (e) => {
          e.target.checked ? S.sel.workloads.add(w.id) : S.sel.workloads.delete(w.id);
          if (inp) inp.style.display = e.target.checked ? "" : "none";
          recompute(); } }), w.id), inp);
  }
  // "id" + the typed "k=v k2=v2" -> "id:k=v:k2=v2" (the CLI's workload syntax)
  function wlString(id) {
    const t = (S.wparams[id] || "").trim();
    return t ? id + ":" + t.split(/[\s,]+/).filter(Boolean).join(":") : id;
  }
  function renderConfigure() {
    const v = $("view-configure"); v.textContent = "";
    add(v, h("p", { class: "intro" }, "Set the architecture, choose what to run it on, launch. Leave a knob at its MVP value to keep the MVP. To sweep a knob, type several values in its Sweep column; every combination is run."));
    if (STATIC) add(v, h("div", { class: "note-box" }, "This is a static export: running is disabled here. Start the live front end (python -m tools.bench serve) to configure and run."));
    // 1 configuration
    add(v, h("h2", null, "1. Architecture"));
    const groups = []; S.knobs.forEach((k) => { if (!groups.includes(k.group)) groups.push(k.group); });
    const head = h("tr", null, h("th", { text: "Knob" }), h("th", { text: "Value" }), h("th", { text: "Sweep (several values)" }), h("th", { text: "" }),
      S.backends.map((b) => h("th", { class: "sup", title: "Run on " + b + ": " + (LEVEL_HELP[b] || "") },
        h("label", { style: "display:inline-flex;gap:3px;align-items:center;cursor:pointer" },
          h("input", { type: "checkbox", checked: S.sel.backends.has(b), disabled: STATIC, onchange: (e) => { e.target.checked ? S.sel.backends.add(b) : S.sel.backends.delete(b); recompute(); } }), b))));
    const tb = h("tbody");
    groups.forEach((g) => {
      add(tb, h("tr", { class: "grp" }, h("td", { colspan: 4 + S.backends.length, text: g })));
      S.knobs.filter((k) => k.group === g).forEach((k) => add(tb, knobRow(k)));
    });
    if (!S.backends.length) add(v, h("div", { class: "note-box", text: "No backend registered, so nothing can run yet." }));
    add(v, h("div", { class: "tablewrap" }, h("table", { class: "knobtbl" }, h("thead", null, head), tb)));
    add(v, h("p", { class: "muted" }, "Column headers choose the backends to run on; each cell says whether that backend can run the knob (MVP only = it refuses other values)."));
    // 2 workloads
    add(v, h("h2", null, "2. Workloads"));
    const kinds = uniq(S.workloads.map((w) => w.kind || "other"));
    kinds.forEach((kd) => {
      add(v, h("h3", { text: kd }));
      add(v, h("div", { class: "picks" }, S.workloads.filter((w) => (w.kind || "other") === kd).map(wlPick)));
    });
    add(v, h("p", { class: "muted" }, "Ticking a workload that takes parameters (speed, rates, burst...) shows a small field: leave it empty for the defaults, or type k=v pairs separated by spaces (for example in_rate=57600 gap=20). A bridge needs engine_route on and 2 or more engines in section 1."));
    // 3 launch
    add(v, h("h2", null, "3. Launch"));
    add(v, h("div", { class: "launch" },
      h("button", { id: "launch", class: "primary", onclick: launch }, "Run"),
      h("label", null, h("input", { type: "checkbox", id: "allowlong", disabled: STATIC }), " also run long levels (flow, gate level, power; minutes to hours)"),
      h("label", null, h("input", { type: "checkbox", id: "nocache", disabled: STATIC }), " ignore cached results"),
      h("span", { id: "runcount", class: "muted" })));
    add(v, h("div", { id: "launchmsg" }));
    // jobs
    add(v, h("h2", null, "Jobs"));
    add(v, h("div", { id: "jobs" }));
    recompute(); renderJobs();
  }
  async function launch() {
    const { base, vary } = readForm();
    const body = { workloads: Array.from(S.sel.workloads).map(wlString), backends: Array.from(S.sel.backends), allow_long: $("allowlong").checked, no_cache: $("nocache").checked };
    if (Object.keys(vary).length) body.sweep = { base, vary }; else body.configs = [base];
    const msg = $("launchmsg"); msg.textContent = "";
    try {
      const r = await fetch("/api/run", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
      const j = await r.json();
      if (!r.ok) throw new Error(j.error || r.status);
      add(msg, h("p", { class: "muted", text: "Started job " + j.job + "." }));
      pollJobs();
    } catch (e) { add(msg, h("div", { class: "note-box", text: "Could not start: " + e.message })); }
  }
  let pollTimer = null;
  function pollJobs() {
    clearTimeout(pollTimer);
    get("/api/jobs").then(async (j) => {
      const wasRunning = S.jobs.some((x) => x.state === "running");
      S.jobs = j; renderJobs();
      const running = j.some((x) => x.state === "running");
      if (wasRunning && !running) await reloadData();
      if (running && currentView() === "configure") pollTimer = setTimeout(pollJobs, 1200);
    }).catch(() => {});
  }
  function issues(label, list) {
    if (!list || !list.length) return null;
    const line = (x) => typeof x === "string" ? x : [x.backend, x.workload, x.config && Object.keys(x.config).length ? JSON.stringify(x.config) : "", x.reason].filter(Boolean).join(" | ");
    return h("details", null, h("summary", { text: list.length + " " + label + (label === "skipped" ? " (long run, needs the long-levels box)" : "") }), list.map((x) => h("div", { class: "muted", text: line(x) })));
  }
  function renderJobs() {
    const box = $("jobs"); if (!box) return; box.textContent = "";
    if (STATIC) return add(box, h("p", { class: "muted" }, "No jobs in a static export."));
    if (!S.jobs.length) return add(box, h("p", { class: "muted" }, "No jobs yet. Results of finished jobs appear in the Results view."));
    const rows = S.jobs.map((j) => {
      const r = j.request || {};
      const what = (r.sweep ? "sweep " + Object.keys(r.sweep.vary || {}).join(", ") : "run") + " | " + (r.workloads || []).join(", ") + " | " + (r.backends || []).join(", ");
      const res = j.result;
      return h("tr", null, h("td", { class: "mono", text: j.id }),
        h("td", null, badge(j.state === "done" ? "s-ok" : j.state === "error" ? "s-error" : "", j.state)),
        h("td", null, j.state === "running" ? h("progress", { max: j.total || 1, value: Math.min(j.events, j.total || 1) }) : null, " ", j.events + " / " + (j.total || "?") + " runs"),
        h("td", { text: what }),
        h("td", null, res ? res.records + " records " + Object.entries(res.status_counts || {}).map(([a, b]) => a + ":" + b).join(" ") : j.error ? h("span", { class: "err", text: j.error }) : ""),
        h("td", null, res ? [issues("refused", res.refused), issues("skipped", res.skipped)] : ""));
    });
    add(box, h("div", { class: "tablewrap" }, h("table", null, h("thead", null, h("tr", null, ["Job", "State", "Progress", "What", "Result", "Refused / skipped"].map((t) => h("th", { text: t })))), h("tbody", null, rows))));
  }

  // ---------------------------------------------------------------- results
  const FIELDS = [["workload", "Workload"], ["metric", "Metric"], ["level", "Level"], ["backend", "Backend"], ["status", "Status"]];
  function renderResults() {
    const v = $("view-results"); v.textContent = "";
    add(v, h("p", { class: "intro" }, "Every number the bench has, one row each. Filter, then click a row to see where it came from. A value nobody observed shows as n/a (with the reason) and a broken run as error; neither is ever blank or zero."));
    const bar = h("div", { class: "bar" });
    FIELDS.forEach(([f, t]) => {
      const sel = h("select", { onchange: (e) => { S.filters[f] = e.target.value; drawRows(); } }, option("", "all"), uniq(S.records.map((r) => r[f])).map((x) => option(x, x, S.filters[f] === x)));
      add(bar, h("label", null, t, sel));
    });
    const cfgs = new Map(); S.records.forEach((r) => cfgs.set(r.config_hash, r.config_label));
    add(bar, h("label", null, "Config", h("select", { onchange: (e) => { S.filters.config_hash = e.target.value; drawRows(); } }, option("", "all"), Array.from(cfgs).map(([hh, l]) => option(hh, l, S.filters.config_hash === hh)))));
    add(bar, h("label", null, "Search notes", h("input", { type: "text", value: S.filters.q || "", oninput: (e) => { S.filters.q = e.target.value; drawRows(); } })));
    add(v, bar, h("div", { id: "rescount", class: "muted" }), h("div", { id: "restbl", class: "tablewrap" }));
    drawRows();
  }
  function drawRows() {
    const f = S.filters, q = (f.q || "").toLowerCase();
    const rows = S.records.filter((r) => FIELDS.every(([k]) => !f[k] || r[k] === f[k]) && (!f.config_hash || r.config_hash === f.config_hash) &&
      (!q || (r.note || "").toLowerCase().includes(q) || r.metric.includes(q) || r.workload.includes(q)));
    const shown = rows.slice(0, 600);
    $("rescount").textContent = rows.length + " of " + S.records.length + " records" + (rows.length > shown.length ? " (first " + shown.length + " shown, narrow the filters)" : "");
    const tb = h("tbody");
    shown.forEach((r) => add(tb, h("tr", { class: "click", onclick: () => go("trace/" + r.id) },
      h("td", { text: r.config_label, title: r.config_hash }), h("td", { text: r.workload + (r.workload_params && Object.keys(r.workload_params).length ? " @ " + Object.values(r.workload_params).join(",") : "") }),
      h("td", { text: r.metric }), h("td", { class: "num" }, valueCell(r)), h("td", null, levelBadge(r.level)), h("td", null, methodBadge(r.method)),
      h("td", { class: "muted", text: r.status === "ok" ? r.note || "" : "" }))));
    const box = $("restbl"); box.textContent = "";
    add(box, h("table", null, h("thead", null, h("tr", null, ["Config", "Workload", "Metric", "Value", "Level", "Method", "Note"].map((t, i) => h("th", { class: i === 3 ? "num" : "", text: t })))), tb));
  }

  // ---------------------------------------------------------------- compare
  function renderCompare(arg) {
    if (arg) S.cmp.metric = arg;
    const metrics = uniq(S.records.map((r) => r.metric));
    if (!S.cmp.metric || !metrics.includes(S.cmp.metric)) {
      // default: the metric with the most two-sided findings (a real comparison), else the first
      const cnt = {};
      (S.findings || []).forEach((f) => { if (f.b) cnt[f.metric] = (cnt[f.metric] || 0) + 1; });
      S.cmp.metric = metrics.slice().sort((a, b) => (cnt[b] || 0) - (cnt[a] || 0))[0] || "";
    }
    const v = $("view-compare"); v.textContent = "";
    add(v, h("p", { class: "intro" }, "One quantity, side by side: each row is a configuration on a workload, each column a level (how closely the model looks at the chip). The last column says whether the levels agree. Where two simulators disagree, it is marked off; that is a finding, not a bug to tune away."));
    const wls = uniq(S.records.filter((r) => r.metric === S.cmp.metric).map((r) => r.workload));
    add(v, h("div", { class: "bar" },
      h("label", null, "Metric", h("select", { onchange: (e) => { S.cmp.metric = e.target.value; S.cmp.workload = ""; renderCompare(); } }, metrics.map((m) => option(m, m, m === S.cmp.metric)))),
      h("label", null, "Workload", h("select", { onchange: (e) => { S.cmp.workload = e.target.value; renderCompare(); } }, option("", "all"), wls.map((m) => option(m, m, m === S.cmp.workload)))),
      h("label", null, "Levels agree?", h("select", { onchange: (e) => { S.cmp.only = e.target.value; renderCompare(); } }, option("", "any", !S.cmp.only), ["off", "close", "match", "identity", "one-sided"].map((x) => option(x, x, S.cmp.only === x))))));
    const rs = S.records.filter((r) => r.metric === S.cmp.metric && (!S.cmp.workload || r.workload === S.cmp.workload));
    const lv = (S.levels.length ? S.levels : uniq(rs.map((r) => r.level))).filter((l) => rs.some((r) => r.level === l));
    const groups = new Map();
    rs.forEach((r) => { const k = gkey(r); if (!groups.has(k)) groups.set(k, []); groups.get(k).push(r); });
    let rows = Array.from(groups.entries()).map(([k, list]) => ({ k, list, f: S.findIdx[k] }));
    if (S.cmp.only) rows = rows.filter((x) => x.f && x.f.verdict === S.cmp.only);
    const cl = (x) => x.list[0].config_label, pk = (x) => x.list[0].workload + " " + stable(x.list[0].workload_params || {});
    rows.sort((a, b) => ((cl(a) === "MVP" ? 0 : 1) - (cl(b) === "MVP" ? 0 : 1)) || cl(a).localeCompare(cl(b)) || pk(a).localeCompare(pk(b), undefined, { numeric: true }));
    const tb = h("tbody");
    rows.forEach(({ list, f }) => {
      const r0 = list[0];
      const tr = h("tr", null, h("td", { text: r0.config_label, title: r0.config_hash }),
        h("td", { text: (r0.row_workload || r0.workload) + (Object.keys(r0.row_params || r0.workload_params || {}).length ? " @ " + Object.values(r0.row_params || r0.workload_params).join(",") : ""), title: list.some((x) => x.workload !== (x.row_workload || x.workload)) ? "joined: " + Array.from(new Set(list.map((x) => x.workload))).join(" = ") : "" }));
      lv.forEach((l) => {
        const here = list.filter((r) => r.level === l).sort((a, b) => (a.status === "ok" ? 0 : 1) - (b.status === "ok" ? 0 : 1));
        const td = h("td", { class: "num" });
        if (!here.length) add(td, h("span", { class: "muted", title: "no record at this level for this config", text: "no run" }));
        else here.forEach((r, i) => add(td, i ? h("br") : null, h("a", { href: "#trace/" + r.id, style: "color:inherit;text-decoration:none", title: "open trace" }, valueCell(r)), here.length > 1 ? h("span", { class: "band", text: r.backend }) : null));
        add(tr, td);
      });
      add(tr, h("td", null, f ? badge("v-" + f.verdict, f.verdict, (VERDICT_HELP[f.verdict] || "") + (f.detail ? " - " + f.detail : "")) : badge("v-one-sided", "not checked", "no cross-check finding for this row")));
      add(tb, tr);
    });
    const counts = {}; rows.forEach((x) => { const t = x.f ? x.f.verdict : "not checked"; counts[t] = (counts[t] || 0) + 1; });
    add(v, h("div", { class: "muted", text: rows.length + " rows: " + (Object.entries(counts).map(([a, b]) => b + " " + a).join(", ") || "none") }));
    add(v, h("div", { class: "tablewrap" }, h("table", null, h("thead", null, h("tr", null,
      h("th", { text: "Config" }), h("th", { text: "Workload" }), lv.map((l) => h("th", { class: "num" }, levelBadge(l))), h("th", { text: "Levels agree?" }))), tb)));
    add(v, h("dl", { class: "terms", style: "margin-top:10px" }, Object.entries(VERDICT_HELP).map(([a, b]) => [h("dt", null, badge("v-" + a, a)), h("dd", { class: "muted", text: b })])));
  }

  // ---------------------------------------------------------------- trace
  async function fetchRecord(id) {
    if (STATIC) { const m = S.records.filter((r) => r.id.startsWith(id)); if (m.length !== 1) throw new Error(m.length ? "ambiguous id prefix" : "no record with that id"); return m[0]; }
    const r = await fetch("/api/trace/" + encodeURIComponent(id));
    const j = await r.json(); if (!r.ok) throw new Error(j.error || r.status); return j;
  }
  function copyBtn(text) {
    const b = h("button", { text: "Copy" });
    b.addEventListener("click", () => {
      const done = () => { b.textContent = "Copied"; setTimeout(() => (b.textContent = "Copy"), 1200); };
      if (navigator.clipboard && navigator.clipboard.writeText) navigator.clipboard.writeText(text).then(done, () => fallbackCopy(text, done));
      else fallbackCopy(text, done);
    });
    return b;
  }
  function fallbackCopy(text, done) {
    const t = h("textarea", { style: "position:fixed;opacity:0" }); t.value = text; document.body.appendChild(t); t.select();
    try { document.execCommand("copy"); done(); } catch (e) { /* ignore */ } t.remove();
  }
  async function renderTrace(id) {
    const v = $("view-trace"); v.textContent = "";
    add(v, h("p", { class: "intro" }, "Where one number came from: the value, what configuration produced it, what tool and commit, and the command that reproduces it."));
    const form = h("div", { class: "bar" }, h("label", null, "Record id (or its first characters)", h("input", { id: "traceid", type: "text", value: id || "", class: "mono", style: "width:200px" })),
      h("button", { text: "Open", onclick: () => go("trace/" + $("traceid").value.trim()) }));
    add(v, form);
    if (!id) return add(v, h("p", { class: "muted" }, "Click any row in Results or Compare to open its trace."));
    let r;
    try { r = await fetchRecord(id); } catch (e) { return add(v, h("div", { class: "note-box", text: e.message })); }
    const p = r.provenance || {};
    const d = r.config_diff || {};
    const kb = Object.fromEntries(S.knobs.map((k) => [k.id, k]));
    add(v, h("div", { class: "bigcard" },
      h("div", { class: "bigval" }, valueCell(r)),
      h("div", null, h("b", { text: r.metric }), " on ", h("b", { text: r.workload }), Object.keys(r.workload_params || {}).length ? h("span", { class: "muted", text: " (" + Object.entries(r.workload_params).map(([a, b]) => a + "=" + b).join(", ") + ")" }) : null, "  ",
        levelBadge(r.level), " ", methodBadge(r.method), " ", badge("s-" + r.status.replace("/", ""), r.status)),
      r.note ? h("div", { class: "muted", text: "Note: " + r.note }) : null,
      h("div", { class: "muted mono", text: "record " + r.id })));
    add(v, h("div", { class: "two" },
      h("div", null, h("h2", null, "Configuration, compared with the MVP"),
        Object.keys(d).length ? h("table", null, h("thead", null, h("tr", null, ["Knob", "MVP", "This record"].map((t) => h("th", { text: t })))),
          h("tbody", null, Object.entries(d).map(([k, val]) => h("tr", null, h("td", { text: kb[k] ? kb[k].label + " (" + k + ")" : k }), h("td", { text: String(kb[k] ? kb[k].default : "?") }), h("td", null, h("b", { text: String(val) })))))) :
          h("p", null, "This is the MVP: every knob at its default."),
        h("div", { class: "muted mono", text: "config " + r.config_hash })),
      h("div", null, h("h2", null, "Provenance"),
        h("dl", { class: "kv" }, ["backend", "level", "method"].map((k) => [h("dt", { text: k }), h("dd", { text: r[k] })]),
          Object.entries(p).filter(([k]) => k !== "command").map(([k, val]) => [h("dt", { text: k }), h("dd", { class: "mono", text: typeof val === "object" ? JSON.stringify(val) : String(val) })])))));
    add(v, h("h2", null, "Reproduce"));
    if (p.command) add(v, h("pre", { class: "cmd", text: p.command }), copyBtn(p.command));
    else add(v, h("p", { class: "muted" }, "No command was recorded for this record" + (p.source ? "; it was read from " + p.source + "." : ".")));
    const others = S.records.filter((x) => gkey(x) === gkey(r) && x.id !== r.id);
    add(v, h("h2", null, "Same quantity elsewhere"));
    if (!others.length) add(v, h("p", { class: "muted" }, "No other record for this config, workload and metric."));
    else add(v, h("table", null, h("tbody", null, others.map((x) => h("tr", { class: "click", onclick: () => go("trace/" + x.id) }, h("td", null, levelBadge(x.level)), h("td", null, methodBadge(x.method)), h("td", { class: "num" }, valueCell(x)))))));
    const f = S.findIdx && S.findIdx[gkey(r)];
    if (f) add(v, h("p", null, "Cross-check: ", badge("v-" + f.verdict, f.verdict, VERDICT_HELP[f.verdict] || ""), f.detail ? " " + f.detail : ""));
  }

  // ---------------------------------------------------------------- help
  function renderHelp() {
    const v = $("view-help"); v.textContent = ""; v.className = "help";
    const task = (title, steps) => h("section", { class: "task" }, h("h2", { text: title }), h("ol", null, steps.map((s) => h("li", null, s))));
    add(v, h("p", { class: "intro" }, "The bench runs one architecture (a configuration of knobs, the MVP by default) on one workload through several simulators (levels) and keeps every number in one store. Pick your task."));
    if (!STATIC) add(v, task("Sweep FIFO depth for the bridge", [
      ["Open ", h("a", { href: "#configure", text: "Configure & sweep" }), "."],
      "In the FIFO depth row, type the depths to try in the Sweep column, for example 2, 4, 8, 16. Turn on Engine-to-engine route in Value if the bridge needs it.",
      "Tick the backends you want in the column headers (cells marked MVP only will refuse non-default depths) and tick the bridge workloads below.",
      ["Press Run. Watch Jobs; refused and skipped runs are listed with their reasons. Then open ", h("a", { href: "#compare", text: "Compare" }), " and choose a metric to see depths as rows."]]));
    add(v, task("Check a number's source", [
      ["Find it in ", h("a", { href: "#results", text: "Results" }), " (filter by workload, metric or level) and click the row."],
      "Trace shows the config difference from the MVP, the tool, commit and time, and the command that reproduces it. Use Copy and run it in the repo.",
      "Level and method tell you how much to trust it: measured is a tool output, estimate is a judgement, imported was read from an old file (its source is in the provenance)."]));
    add(v, task("See where two simulators disagree", [
      ["Open ", h("a", { href: "#compare", text: "Compare" }), ", pick a metric, and set Levels agree? to off to list only the disagreements."],
      "Each row is a configuration on a workload with a column per level. Rows are sorted by configuration (MVP first), then workload; the filter keeps only disagreements.",
      "Click a value to open its trace on each side and compare how they were produced. A disagreement is a result to explain, not a number to tune."]));
    add(v, h("h2", { text: "Words used here" }));
    add(v, h("h3", { text: "Levels" }), h("dl", { class: "terms" }, Object.entries(LEVEL_HELP).map(([a, b]) => [h("dt", null, levelBadge(a)), h("dd", { text: b })])));
    add(v, h("h3", { text: "Methods" }), h("dl", { class: "terms" }, Object.entries(METHOD_HELP).map(([a, b]) => [h("dt", null, methodBadge(a)), h("dd", { text: b })])));
    add(v, h("h3", { text: "Status" }), h("dl", { class: "terms" }, [["ok", "has a value"], ["n/a", "nobody observed it; the reason is in the note. Never zero"], ["error", "the run broke; the note has the end of the error"], ["fail", "ran, but a check inside the run failed"]].map(([a, b]) => [h("dt", null, badge("s-" + a.replace("/", ""), a)), h("dd", { text: b })])));
  }

  // ---------------------------------------------------------------- routing
  const VIEWS = ["configure", "results", "compare", "trace", "help"].filter((n) => !(STATIC && n === "configure"));
  const currentView = () => { const m = (location.hash || "#results").slice(1).split("/")[0]; return VIEWS.includes(m) ? m : "results"; };
  function go(r) { location.hash = "#" + r; }
  function route() {
    const parts = (location.hash || "#results").slice(1).split("/");
    const view = VIEWS.includes(parts[0]) ? parts[0] : "results";
    VIEWS.forEach((n) => { $("view-" + n).hidden = n !== view; });
    if (STATIC) { const t = document.querySelector('#tabs a[data-view="configure"]'); if (t) t.hidden = true; }
    document.querySelectorAll("#tabs a").forEach((a) => a.classList.toggle("on", a.dataset.view === view));
    const arg = decodeURIComponent(parts.slice(1).join("/"));
    if (view === "configure") { renderConfigure(); pollJobs(); }
    else if (view === "results") renderResults();
    else if (view === "compare") renderCompare(arg);
    else if (view === "trace") renderTrace(arg);
    else renderHelp();
  }
  window.addEventListener("hashchange", route);
  window.addEventListener("error", (e) => { const m = document.querySelector("main"); if (m) m.prepend(h("div", { class: "note-box", text: "Page error: " + e.message })); });
  loadAll().then(() => { S.backends.forEach((b) => S.sel.backends.add(b)); route(); }).catch((e) => {
    document.querySelector("main").prepend(h("div", { class: "note-box", text: "Could not load data: " + e.message }));
  });
  window.__benchState = S;
})();
