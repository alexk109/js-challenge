/* Explore tab: the trade-off views.
 *   1. Per-knob impact  (ArchSim.knobImpact)      which protocols flip and which way, chip consequence, area / clock deltas
 *   2. What would it take (ArchSim.whatWouldItTake) for the focused protocol, live; "solve every row of the tag" summary
 *   3. Sweep, scoped to the focused protocol (one knob, chip-fail x ranges shaded)
 *   4. Pareto over the presets, pinned points and the live point (area % against served on the tag)
 * Evidence rule: a candidate with estimate:true is labelled "estimate" with its judged inputs and ranks after
 * the evidence-backed ones (the model already orders them; the view only labels). A candidate that breaks the chip check says
 * "breaks the chip check: <driver> <margin>" and shows its `restore` list.
 *
 * Cost: everything here is computed only when this tab renders (the core renders the active tab only) and memoised by
 * point + tag (+ wl / sweep config), small LRUs. No other tab calls knobImpact or whatWouldItTake.
 * State (state.view.explore, all optional): impactAll (bool: all rows, not only the tag), impactSort ('flips'|'order'),
 *   solveAll (bool), sweep {knob, values[], metric}.
 * Side kinds registered here: impact {knob, value, col, all}, wwit {wl, which, idx, col}, sweep_point {knob, x, metric, col}.
 * Test hooks: .ex-alt[data-knob][data-value][data-ups][data-downs][data-chip]; .ex-cand[data-which][data-idx][data-estimate]
 *   [data-breaks-chip]; tr.ex-solve-row[data-wl][data-kind]; sweep points .pt[data-x][data-y-nom][data-chip]; rect[data-chip-fail];
 *   [data-binding-change]; Pareto .pp[data-id][data-kind][data-x][data-y][data-pareto][data-chip].
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum;

  // ------------------------------------------------------------ small helpers
  function isNum(x) { return typeof x === 'number' && isFinite(x); }
  function vs(state) { return (state.view && state.view.explore) || {}; }
  function knobDef(id) { return A.KNOBS.filter(function (k) { return k.id === id; })[0]; }
  function signed(v, d) { return (v > 0 ? '+' : '') + fmtNum(v, d); }
  function valText(v) { return Array.isArray(v) ? '[' + v.join(', ') + ']' : String(v); }
  function lru(n) {
    var keys = [], map = {};
    var f = function (key, fn) {
      if (map.hasOwnProperty(key)) { keys.splice(keys.indexOf(key), 1); keys.push(key); return map[key]; }
      var v = fn();
      map[key] = v; keys.push(key);
      while (keys.length > n) delete map[keys.shift()];
      return v;
    };
    f.has = function (key) { return map.hasOwnProperty(key); };
    return f;
  }
  // While the user is still dragging a knob (U.settling()), do not start the heavy solvers: a result already memoised is used,
  // anything else shows "updating..." until the edit has been quiet for a moment (the core repaints then).
  function wait(state, key, memo, force) { return !force && U.settling() && !memo.has(key); }
  function updating(what) { return '<p class="muted ex-updating" role="status" data-updating>updating ' + esc(what) + '...</p>'; }
  var memoImpact = lru(8), memoWwit = lru(96), memoSweep = lru(8), memoPresets = lru(4);
  function archKey(arch) { return JSON.stringify(arch); }

  // ------------------------------------------------------------ model calls (memoised)
  function impactOf(state, col, all, force) {
    var p = state.points[col] || state.point, tag = all ? 'all' : state.tag;
    if (wait(state, tag + '|' + archKey(p.arch), memoImpact, force)) return null;
    return memoImpact(tag + '|' + archKey(p.arch), function () { return A.knobImpact(p.arch, state.data, { tag: tag }); });
  }
  function wwitOf(state, col, wl, force) {
    var p = state.points[col] || state.point;
    if (wait(state, state.tag + '|' + wl + '|' + archKey(p.arch), memoWwit, force)) return null;
    return memoWwit(state.tag + '|' + wl + '|' + archKey(p.arch), function () {
      return A.whatWouldItTake(p.arch, wl, state.data, { tag: state.tag });
    });
  }
  function tagWorkloads(state) {
    return state.data.workloads.filter(function (w) { return state.tag === 'all' || (w.tags || []).indexOf(state.tag) >= 0; });
  }

  // ------------------------------------------------------------ shared pieces
  function chipNote(chip, driver, margin, unit) {
    // chip badge, with the driver and margin when it is not a clean pass
    var h = U.chipBadge(chip);
    if (chip !== 'pass' && driver) h += ' <span class="muted small">' + esc(driver) + (margin && isNum(margin.nom) ? ' ' + fmtNum(margin.nom, 0) + (unit ? ' ' + esc(unit) : '') : '') + '</span>';
    return h;
  }
  function judgedList(list) {
    return (list || []).map(function (j) {
      var p = String(j).split(':');
      if (p[0] === 'param' && p[1]) return '<button type="button" class="linkish mono"' + U.sideAttr({ kind: 'param', id: p[1] }) + '>' + esc(j) + '</button>';
      return '<span class="mono">' + esc(j) + '</span>';
    }).join(', ');
  }
  function estimateBadge(c) {
    if (!c.estimate) return '';
    return U.statusBadge('estimate') + ' <span class="small muted">judged: ' + judgedList(c.judged_inputs) + '</span>';
  }
  function changesText(c) {
    return c.changes.map(function (ch) {
      return '<span class="mono">' + esc(ch.knob) + '</span> ' + esc(valText(ch.from)) + ' &#8594; <b>' + esc(valText(ch.to)) + '</b>';
    }).join(' + ');
  }
  function restoreText(c) {
    if (!c.restore || !c.restore.length) return '';
    return '<div class="small ex-restore"><span class="muted">restore:</span> ' + c.restore.slice(0, 6).map(function (r) {
      return '<span class="mono">' + esc(r.knob) + '</span> ' + esc(valText(r.from)) + ' &#8594; ' + esc(valText(r.to));
    }).join('; or ') + '</div>';
  }
  function dArea(v) { return v ? signed(Math.round(v)) : '0'; }

  // ------------------------------------------------------------ 1. per-knob impact
  function flipNames(state, alt) {
    var out = [];
    alt.flips.forEach(function (f, i) {
      if (i < 3) out.push('<span class="ex-flip ' + esc(f.dir) + '">' + (f.dir === 'up' ? '&#9650; ' : '&#9660; ') + esc(U.wlName(state.data, f.wl)) + '</span>');
    });
    if (alt.flips.length > 3) out.push('<span class="muted">+' + (alt.flips.length - 3) + ' more</span>');
    return out.join(' ');
  }
  // Knobs of group ISA: their per-protocol effects are judgement estimates, so the flips they cause are estimates too.
  function isaKnob(id) {
    return (A.KNOBS || []).some(function (k) { return k.id === id && k.group === 'ISA'; });
  }
  function altChip(state, knob, alt, col, all) {
    var attrs = ' data-knob="' + esc(knob.knob) + '" data-value="' + esc(valText(alt.value)) + '" data-ups="' + alt.ups + '" data-downs="' + alt.downs + '"';
    if (alt.invalid) {
      return '<span class="ex-alt invalid"' + attrs + ' data-chip="invalid" title="' + esc(alt.invalid) + '"><b>' + esc(valText(alt.value)) + '</b> <span class="muted small">invalid</span></span>';
    }
    var flips = alt.ups || alt.downs ? '<span class="num">' + (alt.ups ? '+' + alt.ups : '') + (alt.ups && alt.downs ? ' / ' : '') + (alt.downs ? '-' + alt.downs : '') + '</span>' : '<span class="muted">no flip</span>';
    var chipLine = alt.chip.verdict === 'pass' ? '<span class="muted">chip ok</span>' : chipNote(alt.chip.verdict, alt.chip.driver, alt.chip.margin, alt.chip.unit);
    var deltas = '<span class="muted small num">area ' + dArea(alt.d_area_nom) + ' um2; clock margin ' + (alt.d_core_clock_nom ? signed(alt.d_core_clock_nom, 1) : '0') + ' MHz</span>';
    var cls = 'ex-alt' + (alt.downs ? ' has-down' : '') + (alt.ups ? ' has-up' : '') + (alt.chip.verdict !== 'pass' ? ' chip-' + alt.chip.verdict : '');
    return '<button type="button" class="' + cls + '"' + attrs + ' data-chip="' + esc(alt.chip.verdict) + '"' +
      U.sideAttr({ kind: 'impact', knob: knob.knob, value: alt.value, col: col, all: !!all, sticky: true }) + '>' +
      '<span class="ex-alt-top"><b>' + esc(valText(alt.value)) + '</b> ' + flips + (isaKnob(knob.knob) ? ' ' + U.estimateMark('ISA knob: its flips rest on judged effects') : '') + '</span>' +
      (alt.flips.length ? '<span class="ex-alt-flips">' + flipNames(state, alt) + '</span>' : '') +
      '<span class="ex-alt-chip">' + chipLine + '</span><span class="ex-alt-d">' + deltas + '</span></button>';
  }
  function impactSection(state) {
    var v = vs(state), all = !!v.impactAll, col = state.point.col;
    var R = impactOf(state, col, all);
    if (!R) return '<section class="ex-sec" data-section="impact"><h2>Per-knob impact</h2>' + updating('per-knob impact') + '</section>';
    var withFlips = function (k) { return k.alternatives.some(function (a) { return a.ups || a.downs; }); };
    var knobs = R.knobs.filter(function (k) { return !k.no_effect; });
    var idle = R.knobs.filter(function (k) { return k.no_effect; });
    if (v.impactSort !== 'order') {
      knobs = knobs.map(function (k, i) { return { k: k, i: i }; }).sort(function (a, b) {
        return (withFlips(b.k) ? 1 : 0) - (withFlips(a.k) ? 1 : 0) || a.i - b.i;
      }).map(function (o) { return o.k; });
    }
    var rows = knobs.map(function (k) {
      return '<tr class="ex-knob" data-knob-row="' + esc(k.knob) + '" data-flips="' + (withFlips(k) ? 1 : 0) + '"><th scope="row">' + esc(k.label) +
        '<div class="small muted mono">' + esc(k.knob) + ' = ' + esc(valText(state.point.arch[k.knob])) + '</div></th><td><div class="ex-alts">' +
        k.alternatives.map(function (a) { return altChip(state, k, a, col, all); }).join('') + '</div></td></tr>';
    }).join('');
    var b = R.base;
    var ctl = '<div class="ctl-row"><label><input type="checkbox" data-ex="impactAll"' + (all ? ' checked' : '') + '> all rows, not only tag ' + esc(state.tag) + '</label> ' +
      '<label><input type="checkbox" data-ex="impactSort"' + (v.impactSort === 'order' ? ' checked' : '') + '> keep the knob order (default: knobs with a flip first)</label></div>';
    return '<section class="ex-sec" data-section="impact"><h2>Per-knob impact</h2>' +
      '<p class="muted">Each knob moved to its neighbouring values, one at a time, at the ' + esc(state.point.label) + ' point: protocols that flip (up = better verdict), ' +
      'the chip check afterwards, and the change in placed area and in core-clock margin. Base: <span class="num">' + b.counts.served + '</span> served, <span class="num">' + b.counts.marginal +
      '</span> marginal, <span class="num">' + b.counts.impossible + '</span> impossible; chip ' + U.chipBadge(b.chip.verdict) + '. Click a value for the detail.</p>' + ctl +
      '<div class="tbl"><table class="ex-impact"><thead><tr><th>knob</th><th>alternatives: flips (+ up, - down), chip, deltas</th></tr></thead><tbody>' + rows + '</tbody></table></div>' +
      (idle.length ? '<p class="small muted">No effect at this point: ' + idle.map(function (k) { return esc(k.label) + (k.reason ? ' (' + esc(k.reason) + ')' : ''); }).join('; ') + '.</p>' : '') + '</section>';
  }

  // ------------------------------------------------------------ 2. what would it take
  function candRow(state, R, c, which, idx) {
    var losses = c.losses === null ? '<span class="muted">not computed</span>' : (c.losses.length ? '<span class="ex-loss">-' + c.losses.length + '</span> <span class="small muted">' +
      esc(c.losses.map(function (w) { return U.wlName(state.data, w); }).join(', ')) + '</span>' : '<span class="muted">none</span>');
    var chip = c.chip === 'pass' ? U.chipBadge('pass') : U.chipBadge(c.chip) + ' <span class="ex-break">' + esc(c.breaks_chip || '') + '</span>';
    return '<tr class="ex-cand' + (c.estimate ? ' estimate' : '') + (c.breaks_chip ? ' breaks' : '') + '" data-which="' + which + '" data-idx="' + idx + '" data-estimate="' + (c.estimate ? 1 : 0) +
      '" data-breaks-chip="' + (c.breaks_chip ? 1 : 0) + '"' + U.sideAttr({ kind: 'wwit', wl: R.wl, which: which, idx: idx, col: state.point.col, sticky: true }) + '>' +
      '<td class="num">' + (idx + 1) + '</td><td>' + changesText(c) + (c.estimate ? '<div>' + estimateBadge(c) + '</div>' : '') + restoreText(c) + '</td>' +
      '<td>' + U.verdictBadge(c.verdict) + '</td><td>' + chip + '</td><td class="r num">' + dArea(c.d_area_nom) + '</td><td>' + losses + '</td></tr>';
  }
  function candTable(state, R, list, which) {
    var shown = list.slice(0, 8), rest = list.slice(8);
    function rows(l, off) { return l.map(function (c, i) { return candRow(state, R, c, which, off + i); }).join(''); }
    var head = '<thead><tr><th>#</th><th>change</th><th>' + esc(U.wlName(state.data, R.wl)) + '</th><th>chip check</th><th class="r">&#916; area (um2)</th><th>collateral (tag ' + esc(state.tag) + ')</th></tr></thead>';
    return '<div class="tbl"><table class="ex-cands">' + head + '<tbody>' + rows(shown, 0) + '</tbody></table>' +
      (rest.length ? '<details><summary>' + rest.length + ' more</summary><table class="ex-cands">' + head + '<tbody>' + rows(rest, 8) + '</tbody></table></details>' : '') + '</div>';
  }
  function blockingText(R) {
    return (R.blocking || []).map(function (b) {
      return '<span class="ex-block"><b>' + esc(b.short) + '</b> <span class="num">' + esc(fmtNum(b.nom)) + ' ' + esc(b.unit === 'flag' ? '' : b.unit || '') + '</span></span>';
    }).join(', ');
  }
  function hintsText(R) {
    return (R.hints || []).map(function (h) {
      return '<li><b>' + esc(h.short) + '</b> improves most with <span class="mono">' + esc(h.knob) + '</span> ' + esc(valText(h.from)) + ' &#8594; ' + esc(valText(h.to)) +
        ': <span class="num">' + esc(fmtNum(h.nom_before)) + ' &#8594; ' + esc(fmtNum(h.nom_after)) + ' ' + (h.unit === 'flag' ? '' : esc(h.unit || '')) + '</span></li>';
    }).join('');
  }
  function wwitSection(state) {
    var wl = state.focus.wl, R = wwitOf(state, state.point.col, wl);
    if (!R) return '<section class="ex-sec" data-section="wwit" data-wl="' + esc(wl) + '"><h2>What would it take: ' + esc(U.wlName(state.data, wl)) + '</h2>' + updating('candidates') + '</section>';
    var h = '<section class="ex-sec" data-section="wwit" data-wl="' + esc(wl) + '"><h2>What would it take: ' + esc(U.wlName(state.data, wl)) + '</h2>' +
      '<p class="muted">The smallest change that makes the focused protocol served (' + esc(state.point.label) + ' point). Ranked by: chip check clean first; evidence-backed before ' +
      '<em>estimate</em> (needs an input whose status is judgement); fewer knobs; smaller placed area; fewer collateral losses; fewer steps.</p>';
    if (R.already) h += '<p class="ex-already" data-result="already">Already ' + U.verdictBadge(R.verdict) + ' at this point: nothing to change.</p>';
    else if (R.single.length) {
      h += '<p data-result="single">Currently ' + U.verdictBadge(R.verdict) + '. <b>' + R.single.length + '</b> single-knob change' + (R.single.length > 1 ? 's' : '') + ' serve it.</p>' + candTable(state, R, R.single, 'single');
    } else if (R.pairs.length) {
      h += '<p data-result="pairs">Currently ' + U.verdictBadge(R.verdict) + '. No single knob serves it; <b>' + R.pairs.length + '</b> pair' + (R.pairs.length > 1 ? 's' : '') +
        ' of knobs do' + (R.blocking.length ? ' (blocked by ' + blockingText(R) + ')' : '') + '.</p>' + candTable(state, R, R.pairs, 'pairs');
    } else {
      h += '<p class="ex-oor" data-result="out_of_reach"><b>Out of reach</b> with one or two knob changes. Blocked by: ' + (blockingText(R) || 'n/a') + '.</p>' +
        (R.hints && R.hints.length ? '<ul class="ex-hints">' + hintsText(R) + '</ul>' : '<p class="muted">No single knob improves a blocking limit.</p>');
    }
    return h + solveAllSection(state) + '</section>';
  }

  function solveSummary(state, R) {
    if (R.already) return ['already', '<span class="muted">already served</span>'];
    var list = R.single.length ? R.single : R.pairs, kind = R.single.length ? 'single' : 'pairs';
    if (!list.length) {
      return ['out_of_reach', '<b>out of reach</b> <span class="small muted">blocked by ' + (R.blocking || []).map(function (b) { return esc(b.short); }).join(', ') + '</span>'];
    }
    var c = list[0];
    return [kind, changesText(c) + ' ' + (c.estimate ? U.statusBadge('estimate') + ' ' : '') + (c.chip !== 'pass' ? U.chipBadge(c.chip) + ' <span class="small muted">' + esc(c.breaks_chip || '') + '</span> ' : '') +
      '<span class="small muted">(' + list.length + ' option' + (list.length > 1 ? 's' : '') + (kind === 'pairs' ? ', needs two knobs' : '') + ')</span>'];
  }
  function solveAllSection(state) {
    var v = vs(state), wls = tagWorkloads(state);
    var h = '<div class="ex-solve"><h3>Every row of tag ' + esc(state.tag) + '</h3>';
    if (!v.solveAll) {
      return h + '<p class="muted">Solves ' + wls.length + ' protocols (a few seconds in a slow engine, memoised afterwards).</p>' +
        '<button type="button" data-ex-act="solve-all">Solve every row of the tag</button></div>';
    }
    var busy = wls.some(function (w) { return !wwitOf(state, state.point.col, w.id); });
    if (busy) return h + updating('every row') + '</div>';
    var rows = wls.map(function (w) {
      var R = wwitOf(state, state.point.col, w.id), s = solveSummary(state, R);
      return '<tr class="ex-solve-row" data-wl="' + esc(w.id) + '" data-kind="' + s[0] + '"><td><button type="button" class="linkish"' + U.focusAttr(w.id) + '>' + esc(w.name || w.id) + '</button></td><td>' +
        U.verdictBadge(R.verdict) + '</td><td>' + s[1] + '</td></tr>';
    }).join('');
    return h + '<button type="button" data-ex-act="hide-solve-all">Hide</button>' +
      '<div class="tbl"><table class="ex-solve-table"><thead><tr><th>protocol</th><th>now</th><th>smallest change (first candidate)</th></tr></thead><tbody>' + rows + '</tbody></table></div>' +
      '<p class="small muted">Click a protocol to focus it; its full candidate list is in the table above.</p></div>';
  }

  // ------------------------------------------------------------ 3. sweep (scoped to the focused protocol)
  function defaultValues(k, data) {
    var pd = data && data.sweep_defaults && data.sweep_defaults[k.id];
    if (pd) {
      var g = {}, n = Math.round((pd.max - pd.min) / pd.step);
      for (var i = 0; i <= n; i++) g[+(pd.min + i * pd.step).toFixed(6)] = true;
      (pd.extra || []).forEach(function (e) { g[e] = true; });
      return Object.keys(g).map(Number).sort(function (a, b) { return a - b; });
    }
    if (k.type === 'enum') return k.options.slice();
    if (k.type === 'bool') return [false, true];
    var cnt = Math.round((k.max - k.min) / k.step) + 1, stride = Math.max(1, Math.ceil(cnt / 24)), out = [];
    for (var j = 0; j < cnt; j += stride) out.push(+(k.min + j * k.step).toFixed(6));
    return out;
  }
  function parseValues(k, text) {
    var out = [];
    String(text).split(/[\s,;]+/).filter(function (s) { return s !== ''; }).forEach(function (s) {
      if (k.type === 'number') { var n = parseFloat(s); if (isNum(n) && n >= k.min && n <= k.max) out.push(n); }
      else if (k.type === 'bool') { if (s === 'true' || s === 'false') out.push(s === 'true'); }
      else { var o = k.options.filter(function (q) { return String(q) === s; })[0]; if (o !== undefined) out.push(o); }
    });
    return out.slice(0, 40);
  }
  function sweepKnobs() { return A.KNOBS.filter(function (k) { return k.type !== 'multi'; }); }
  // Metric choices are scoped to the focused protocol (never a workload list).
  function metricGroups(state) {
    var wl = state.focus.wl, w = U.workloadOf(state.data, wl), g = [], prot = [];
    if (w && !w.rate_elastic) prot.push(['max_rate:' + wl, 'max rate (Hz)', 'Hz']);
    (state.cell.limits || []).forEach(function (l) {
      if (l.value && isNum(l.value.nom)) prot.push(['margin:' + wl + ':' + l.id, 'margin: ' + (l.short || l.id), l.unit || '']);
    });
    g.push({ label: 'Focused protocol: ' + U.wlName(state.data, wl), items: prot });
    g.push({ label: 'Counts on tag ' + state.tag, items: [['served_count:' + state.tag, 'served', ''], ['marginal_count:' + state.tag, 'marginal', '']] });
    g.push({ label: 'Chip check margins', items: state.chip.items.map(function (it) { return ['chip_margin:' + it.id, (it.short || it.id), it.unit || '']; }) });
    g.push({ label: 'Architecture', items: [['util_pct', 'area (% of core)', '%'], ['link_write_MBps', 'host link write (MB/s)', 'MB/s'], ['link_read_MBps', 'host link read (MB/s)', 'MB/s']] });
    return g;
  }
  function metricInfo(state, m) {
    var found = null;
    metricGroups(state).forEach(function (g) { g.items.forEach(function (it) { if (it[0] === m) found = it; }); });
    return found;
  }
  function sweepConfig(state) {
    var sw = vs(state).sweep || {}, d = state.data;
    var k = (sw.knob && knobDef(sw.knob) && knobDef(sw.knob).type !== 'multi') ? knobDef(sw.knob) : (knobDef('f_core_mhz') || sweepKnobs()[0]);
    var vals = (sw.values && sw.values.length && sw.knob === k.id) ? sw.values : defaultValues(k, d);
    var w = U.workloadOf(d, state.focus.wl);
    var metric = sw.metric && metricInfo(state, sw.metric) ? sw.metric : (w && !w.rate_elastic ? 'max_rate:' + state.focus.wl : 'served_count:' + state.tag);
    return { def: k, knob: k.id, values: vals, metric: metric, info: metricInfo(state, metric) };
  }
  function runSweep(state, cfg) {
    var arch = state.point.arch, key = [state.tag, archKey(arch), cfg.knob, JSON.stringify(cfg.values), cfg.metric, state.focus.wl].join('|');
    return memoSweep(key, function () {
      var pts = A.sweep(arch, cfg.knob, cfg.values, cfg.metric, state.data);
      var p = cfg.metric.split(':'), wl = (p[0] === 'max_rate' || p[0] === 'margin') ? p[1] : null, changes = [], prev = null;
      if (wl) pts.forEach(function (pt, i) {
        var a = Object.assign({}, arch); a[cfg.knob] = pt.x;
        pt.binding = A.evaluateCell(a, wl, state.data, { lite: true }).binding;
        if (prev !== null && pt.binding !== prev) changes.push({ x: pt.x, i: i, from: prev, to: pt.binding });
        prev = pt.binding;
      });
      return { pts: pts, changes: changes, wl: wl };
    });
  }
  var W = 680, H = 300, M = { l: 62, r: 16, t: 18, b: 44 };
  function r1(v) { return Math.round(v * 10) / 10; }
  function sweepChart(state, cfg, S) {
    var cont = cfg.def.type === 'number', xs = cfg.values, all = [];
    S.pts.forEach(function (p) { ['nom', 'pess', 'opt'].forEach(function (v) { if (isNum(p.y[v])) all.push(p.y[v]); }); });
    if (!all.length) return '<p class="muted">No finite values for this metric.</p>';
    var ymin = Math.min.apply(null, all.concat([0])), ymax = Math.max.apply(null, all.concat([0]));
    if (ymax === ymin) ymax = ymin + 1;
    var pad = (ymax - ymin) * 0.05; ymax += pad; if (ymin < 0) ymin -= pad;
    var xlo = cont ? Math.min.apply(null, xs) : 0, xhi = cont ? Math.max.apply(null, xs) : Math.max(xs.length - 1, 1);
    if (xhi === xlo) xhi = xlo + 1;
    function px(x, i) { var v = cont ? x : i; return r1(M.l + (v - xlo) / (xhi - xlo) * (W - M.l - M.r)); }
    function py(y) { return r1(H - M.b - (y - ymin) / (ymax - ymin) * (H - M.t - M.b)); }
    var g = '', firstFail = null;
    S.pts.forEach(function (p, i) {   // chip-fail shading first (under the data)
      if (p.chip !== 'fail') return;
      var x = px(p.x, i), prevX = i > 0 ? px(xs[i - 1], i - 1) : null, nextX = i < xs.length - 1 ? px(xs[i + 1], i + 1) : null;
      var l = prevX !== null ? (prevX + x) / 2 : x - (nextX !== null ? (nextX - x) / 2 : 10);
      var r = nextX !== null ? (x + nextX) / 2 : x + (prevX !== null ? (x - prevX) / 2 : 10);
      l = Math.max(l, M.l); r = Math.min(r, W - M.r);
      if (!firstFail) firstFail = { x: l, driver: p.chip_driver };
      g += '<rect class="chipfail" data-chip-fail="' + esc(p.x) + '" data-driver="' + esc(p.chip_driver) + '" x="' + r1(l) + '" y="' + M.t + '" width="' + r1(Math.max(1, r - l)) + '" height="' + (H - M.t - M.b) + '"/>';
    });
    if (firstFail) g += '<text x="' + r1(firstFail.x + 4) + '" y="' + (H - M.b - 6) + '">chip fails (' + esc(firstFail.driver) + ')</text>';
    U.niceTicks(ymin, ymax, 5).forEach(function (t) {
      g += '<line class="gridline" x1="' + M.l + '" x2="' + (W - M.r) + '" y1="' + py(t) + '" y2="' + py(t) + '"/><text x="' + (M.l - 6) + '" y="' + (py(t) + 4) + '" text-anchor="end">' + esc(U.eng(t)) + '</text>';
    });
    g += '<line class="axis" x1="' + M.l + '" x2="' + (W - M.r) + '" y1="' + (H - M.b) + '" y2="' + (H - M.b) + '"/><line class="axis" x1="' + M.l + '" x2="' + M.l + '" y1="' + M.t + '" y2="' + (H - M.b) + '"/>';
    var every = Math.max(1, Math.ceil(xs.length / 10));
    xs.forEach(function (x, i) {
      if (i % every === 0 || i === xs.length - 1) g += '<text x="' + px(x, i) + '" y="' + (H - M.b + 16) + '" text-anchor="middle">' + esc(cont ? fmtNum(x) : String(x)) + '</text>';
    });
    g += '<text x="' + (M.l + (W - M.l - M.r) / 2) + '" y="' + (H - 6) + '" text-anchor="middle">' + esc(cfg.def.label + (cfg.def.unit ? ' (' + cfg.def.unit + ')' : '')) + '</text>';
    var cur = state.point.arch[cfg.knob], ci = xs.indexOf(cur);
    if (cont ? (cur >= xlo && cur <= xhi) : ci >= 0) {
      var cx = px(cur, ci);
      g += '<line class="curule" x1="' + cx + '" x2="' + cx + '" y1="' + M.t + '" y2="' + (H - M.b) + '"/><text x="' + (cx + 3) + '" y="' + (M.t + 8) + '">current</text>';
    }
    S.changes.forEach(function (c, n) {
      var x = px(c.x, c.i);
      g += '<g data-binding-change="' + esc(c.to) + '" data-from="' + esc(c.from) + '" data-x="' + esc(c.x) + '"><line class="bindrule" x1="' + x + '" x2="' + x + '" y1="' + M.t + '" y2="' + (H - M.b) + '"/>' +
        '<text x="' + (x + 3) + '" y="' + (M.t + 20 + 11 * (n % 4)) + '">binding: ' + esc(c.from) + ' &#8594; ' + esc(c.to) + '</text></g>';
    });
    var segs = [], run = [], up = [], lo = [];
    S.pts.forEach(function (p, i) {
      if (isNum(p.y.nom)) run.push(px(p.x, i) + ',' + py(p.y.nom)); else { if (run.length) segs.push(run); run = []; }
      if (isNum(p.y.pess) && isNum(p.y.opt)) {
        up.push(px(p.x, i) + ',' + py(Math.max(p.y.pess, p.y.opt)));
        lo.unshift(px(p.x, i) + ',' + py(Math.min(p.y.pess, p.y.opt)));
      }
    });
    if (run.length) segs.push(run);
    if (cont && up.length > 1) g += '<polygon class="band" points="' + up.concat(lo).join(' ') + '"/>';
    if (cont) segs.forEach(function (sg) { if (sg.length > 1) g += '<polyline class="nomline" points="' + sg.join(' ') + '"/>'; });
    S.pts.forEach(function (p, i) {
      if (!isNum(p.y.nom)) return;
      var x = px(p.x, i), y = py(p.y.nom);
      if (!cont) g += '<line class="whisker" x1="' + x + '" x2="' + x + '" y1="' + py(isNum(p.y.pess) ? p.y.pess : p.y.nom) + '" y2="' + py(isNum(p.y.opt) ? p.y.opt : p.y.nom) + '"/>';
      g += '<circle class="pt' + (p.chip !== 'pass' ? ' chip-' + p.chip : '') + '" cx="' + x + '" cy="' + y + '" r="4" tabindex="0" data-x="' + esc(p.x) + '" data-i="' + i + '" data-y-nom="' + p.y.nom +
        '" data-y-pess="' + (isNum(p.y.pess) ? p.y.pess : '') + '" data-y-opt="' + (isNum(p.y.opt) ? p.y.opt : '') + '" data-chip="' + esc(p.chip) + '"' + (p.binding ? ' data-binding="' + esc(p.binding) + '"' : '') +
        U.sideAttr({ kind: 'sweep_point', knob: cfg.knob, x: p.x, metric: cfg.metric, col: state.point.col, sticky: true }) + '><title>' + esc(cfg.def.label + ' = ' + p.x + ': ' + fmtNum(p.y.nom) + ' (chip ' + p.chip + ')') + '</title></circle>';
    });
    return '<div class="chart ex-chart">' + U.svgWrap(W, H, g, 'Sweep of ' + (cfg.info ? cfg.info[1] : cfg.metric) + ' against ' + cfg.def.label) + '</div>';
  }
  function selectHtml(attr, opts, val) {
    return '<select data-ex="' + attr + '">' + opts.map(function (o) {
      return '<option value="' + esc(o[0]) + '"' + (String(o[0]) === String(val) ? ' selected' : '') + '>' + esc(o[1]) + '</option>';
    }).join('') + '</select>';
  }
  function sweepSection(state) {
    var cfg = sweepConfig(state), S = runSweep(state, cfg), nfail = S.pts.filter(function (p) { return p.chip === 'fail'; }).length;
    var metricSel = '<select data-ex="metric">' + metricGroups(state).map(function (g) {
      return '<optgroup label="' + esc(g.label) + '">' + g.items.map(function (o) {
        return '<option value="' + esc(o[0]) + '"' + (o[0] === cfg.metric ? ' selected' : '') + '>' + esc(o[1]) + '</option>';
      }).join('') + '</optgroup>';
    }).join('') + '</select>';
    var ctl = '<div class="ctl-row"><label>Knob ' + selectHtml('sweep-knob', sweepKnobs().map(function (k) { return [k.id, k.label]; }), cfg.knob) + '</label> ' +
      '<label>Values <input type="text" data-ex="sweep-values" value="' + esc(cfg.values.join(', ')) + '" size="30" aria-label="Sweep values"></label> ' +
      '<label>Metric ' + metricSel + '</label></div>';
    var bc = S.changes.length ? '<p class="muted small">The binding limit changes at: ' + S.changes.map(function (c) {
      return '<span class="num">' + esc(cfg.knob + ' = ' + c.x) + '</span> (' + esc(c.from) + ' &#8594; ' + esc(c.to) + ')';
    }).join('; ') + '.</p>' : '';
    return '<section class="ex-sec" data-section="sweep" data-sweep-knob="' + esc(cfg.knob) + '" data-sweep-metric="' + esc(cfg.metric) + '" data-npoints="' + cfg.values.length + '">' +
      '<h2>Sweep: ' + esc(cfg.info ? cfg.info[1] : cfg.metric) + ' against ' + esc(cfg.def.label) + '</h2>' + ctl + sweepChart(state, cfg, S) +
      '<p class="legend muted small">Line: nominal; band or bar: pessimistic to optimistic; dashed rule: binding limit changes; shaded: the chip check fails there' +
      (nfail ? ' (' + nfail + ' of ' + S.pts.length + ' points)' : '') + '. Click a point for its detail.</p>' + bc + '</section>';
  }

  // ------------------------------------------------------------ 4. Pareto
  function presetPoints(state) {
    return memoPresets(state.tag, function () {
      return (state.data.presets || []).map(function (p) {
        var a = A.applyPreset(p.id, state.data), ev = A.evaluateAll(a, state.data, state.tag);
        return { id: p.id, label: p.label || p.id, kind: 'preset', x: ev.arch.area.util_pct.nom, y: ev.counts.served, chip: ev.chip.verdict };
      });
    });
  }
  function paretoPoints(state) {
    var pts = presetPoints(state).map(function (p) { return Object.assign({}, p); });
    state.points.slice(1).forEach(function (p) { pts.push({ id: 'pin' + p.col, label: p.label, kind: 'pinned', x: p.derived.area.util_pct.nom, y: p.all.counts.served, chip: p.chip.verdict }); });
    var live = state.points[0];
    pts.push({ id: 'live', label: 'live', kind: 'live', x: live.derived.area.util_pct.nom, y: live.all.counts.served, chip: live.chip.verdict });
    var keep = A.pareto(pts.map(function (p) { return { area: p.x, served: p.y, chip: p.chip }; }));
    pts.forEach(function (p, i) { p.front = keep.indexOf(i) >= 0; });
    return pts;
  }
  var CLOSE = { circle: '</circle>', square: '</rect>', diamond: '</polygon>', triangle: '</polygon>' };
  function markT(shape, x, y, r, attrs, title) {   // U.mark with a <title> child (tooltip, accessible name)
    return U.mark(shape, x, y, r, attrs).replace(/\/>$/, '>' + title + CLOSE[shape]);
  }
  function paretoSection(state) {
    var pts = paretoPoints(state), PM = { l: 62, r: 16, t: 18, b: 44 };
    var xs = pts.map(function (p) { return p.x; }), ys = pts.map(function (p) { return p.y; });
    var xlo = Math.min.apply(null, xs), xhi = Math.max.apply(null, xs), ylo = Math.min.apply(null, ys.concat([0])), yhi = Math.max.apply(null, ys);
    var ex = (xhi - xlo) * 0.08 || 1, ey = (yhi - ylo) * 0.12 || 1;
    xlo -= ex; xhi += ex; yhi += ey;
    function px(x) { return r1(PM.l + (x - xlo) / (xhi - xlo) * (W - PM.l - PM.r)); }
    function py(y) { return r1(H - PM.b - (y - ylo) / (yhi - ylo) * (H - PM.t - PM.b)); }
    var g = '';
    U.niceTicks(ylo, yhi, 5).forEach(function (t) {
      g += '<line class="gridline" x1="' + PM.l + '" x2="' + (W - PM.r) + '" y1="' + py(t) + '" y2="' + py(t) + '"/><text x="' + (PM.l - 6) + '" y="' + (py(t) + 4) + '" text-anchor="end">' + esc(U.eng(t)) + '</text>';
    });
    U.niceTicks(xlo, xhi, 6).forEach(function (t) {
      g += '<line class="gridline" x1="' + px(t) + '" x2="' + px(t) + '" y1="' + PM.t + '" y2="' + (H - PM.b) + '"/><text x="' + px(t) + '" y="' + (H - PM.b + 16) + '" text-anchor="middle">' + esc(U.eng(t)) + '</text>';
    });
    g += '<line class="axis" x1="' + PM.l + '" x2="' + (W - PM.r) + '" y1="' + (H - PM.b) + '" y2="' + (H - PM.b) + '"/><line class="axis" x1="' + PM.l + '" x2="' + PM.l + '" y1="' + PM.t + '" y2="' + (H - PM.b) + '"/>';
    g += '<text x="' + (PM.l + (W - PM.l - PM.r) / 2) + '" y="' + (H - 6) + '" text-anchor="middle">area, % of core (lower is better)</text>' +
      '<text transform="translate(12 ' + (PM.t + (H - PM.t - PM.b) / 2) + ') rotate(-90)" text-anchor="middle">served on tag ' + esc(state.tag) + ' (higher is better)</text>';
    var front = pts.filter(function (p) { return p.front; }).sort(function (a, b) { return a.x - b.x || b.y - a.y; });
    if (front.length) {
      var dd = 'M' + px(front[0].x) + ',' + py(front[0].y);
      for (var i = 1; i < front.length; i++) dd += 'H' + px(front[i].x) + 'V' + py(front[i].y);
      g += '<path class="frontline" d="' + dd + 'H' + (W - PM.r) + '"/>';
    }
    var VAR = { preset: '--s1', pinned: '--s3', live: '--s2' };
    pts.forEach(function (p) {
      var col = 'var(' + VAR[p.kind] + ')', x = px(p.x), y = py(p.y), shape = p.kind === 'live' ? 'diamond' : 'circle';
      var status = p.chip === 'fail' ? 'chip-fail' : (p.front ? 'front' : 'dominated');
      var a = 'class="pp ' + status + '" tabindex="0" data-id="' + esc(p.id) + '" data-kind="' + p.kind + '" data-x="' + p.x + '" data-y="' + p.y + '" data-pareto="' + status + '" data-chip="' + esc(p.chip) + '"';
      var title = '<title>' + esc(p.label + ': area ' + fmtNum(p.x, 2) + ' %, served ' + p.y + ', chip ' + p.chip + (p.front ? ', non-dominated' : '')) + '</title>';
      if (p.chip === 'fail') g += markT(shape, x, y, 5, a + ' fill="none" stroke="' + col + '" stroke-width="2" stroke-dasharray="3 2"', title);
      else if (p.front) g += markT(shape, x, y, 6, a + ' fill="' + col + '" stroke="var(--ink)" stroke-width="2"', title);
      else g += markT(shape, x, y, 5, a + ' fill="none" stroke="' + col + '" stroke-width="2"', title);
      if (p.chip === 'marginal') g += '<text class="flag" x="' + (x + 7) + '" y="' + (y + 4) + '">~</text>';
    });
    var seen = {}, order = [];
    pts.forEach(function (p) {
      var key = p.x + '|' + p.y;
      if (!seen[key]) { seen[key] = { p: p, ids: [], front: false, flag: '' }; order.push(key); }
      seen[key].ids.push(p.id); seen[key].front = seen[key].front || p.front;
      if (p.chip !== 'pass') seen[key].flag = ' [chip ' + p.chip + ']';
    });
    order.forEach(function (k, n) {
      var s = seen[k];
      g += '<text class="plabel' + (s.front ? ' front' : '') + '" x="' + (px(s.p.x) + 10) + '" y="' + (py(s.p.y) - 7 - 10 * (n % 2)) + '">' + esc(s.ids.join(', ') + s.flag) + '</text>';
    });
    var frontIds = pts.filter(function (p) { return p.front && p.kind === 'preset'; }).map(function (p) { return p.id; });
    return '<section class="ex-sec" data-section="pareto" data-front="' + esc(frontIds.join(',')) + '" data-npoints="' + pts.length + '"><h2>Pareto: presets, pinned points and the live point</h2>' +
      '<div class="chart ex-chart">' + U.svgWrap(W, H, g, 'Pareto scatter of area against served protocols') + '</div>' +
      '<p class="legend muted small">Filled with heavy ring: non-dominated (dashed step line). Hollow: dominated. Hollow dashed: chip check fails, excluded from the front. ' +
      '"~" and [chip marginal]: the chip check is marginal (still on the front if nothing beats it). Blue: preset; green: pinned; orange diamond: live.</p></section>';
  }

  // ------------------------------------------------------------ view
  U.renderExplore = function (state) {
    return '<div class="v-explore"><p class="muted ex-lead">Trade-off views for the ' + esc(state.point.label) + ' point (tag ' + esc(state.tag) +
      '). Verdicts and the chip check per protocol are in <button type="button" class="linkish" data-tab="capability">Capability</button>.</p>' +
      impactSection(state) + wwitSection(state) + sweepSection(state) + paretoSection(state) + '</div>';
  };

  // ------------------------------------------------------------ side panel kinds
  function applyBtn(changes) {
    return '<p><button type="button" data-ex-apply="' + esc(JSON.stringify(changes)) + '">Apply to the live point</button></p>';
  }
  function wlLink(state, id) { return '<button type="button" class="linkish"' + U.focusAttr(id) + '>' + esc(U.wlName(state.data, id)) + '</button>'; }
  U.registerSide('impact', function (state, ref) {
    var R = impactOf(state, ref.col || 0, ref.all, true);
    var k = R.knobs.filter(function (q) { return q.knob === ref.knob; })[0];
    var alt = k && k.alternatives.filter(function (a) { return valText(a.value) === valText(ref.value); })[0];
    if (!alt) return '<p class="muted">This alternative is not in the current impact table.</p>';
    var h = '<p><span class="mono">' + esc(k.knob) + '</span>: ' + esc(valText(alt.from)) + ' &#8594; <b>' + esc(valText(alt.value)) + '</b> (' + alt.steps + ' step' + (alt.steps > 1 ? 's' : '') + ')</p>';
    if (alt.invalid) return h + '<p class="muted">Not buildable: ' + esc(alt.invalid) + '</p>';
    h += '<h3>Protocols that flip (tag ' + esc(R.tag) + ')</h3>';
    h += alt.flips.length ? '<ul class="ex-flips">' + alt.flips.map(function (f) {
      return '<li>' + (f.dir === 'up' ? '&#9650; up' : '&#9660; down') + ': ' + wlLink(state, f.wl) + ' ' + esc(f.from) + ' &#8594; ' + esc(f.to) + '</li>';
    }).join('') + '</ul>' : '<p class="muted">None: no protocol changes verdict.</p>';
    h += '<h3>Chip check after the change</h3><p>' + chipNote(alt.chip.verdict, alt.chip.driver, alt.chip.margin, alt.chip.unit) + ' <span class="small muted">before: ' + esc(R.base.chip.verdict) + '</span></p>';
    h += '<h3>Deltas</h3><p class="num">placed area ' + dArea(alt.d_area_nom) + ' um2<br>core-clock margin ' + (alt.d_core_clock_nom ? signed(alt.d_core_clock_nom, 2) : '0') + ' MHz</p>';
    return { title: 'Impact: ' + k.label + ' = ' + valText(alt.value), html: h + applyBtn([{ knob: k.knob, to: alt.value }]) };
  }, function (body) { bindApply(body); });
  U.registerSide('wwit', function (state, ref) {
    var R = wwitOf(state, ref.col || 0, ref.wl, true), list = (ref.which === 'pairs' ? R.pairs : R.single) || [], c = list[ref.idx];
    if (!c) return '<p class="muted">This candidate is not in the current list.</p>';
    var h = '<p>' + changesText(c) + '</p><p>' + U.verdictBadge(c.verdict) + ' for ' + wlLink(state, R.wl) + '</p>';
    if (c.estimate) h += '<p>' + estimateBadge(c) + '</p><p class="small muted">This candidate rests on an input whose status is judgement, so it ranks after the evidence-backed ones.</p>';
    h += '<h3>Chip check</h3><p>' + (c.chip === 'pass' ? U.chipBadge('pass') + ' <span class="small muted">driver ' + esc(c.chip_driver) + '</span>' : U.chipBadge(c.chip) + ' ' + esc(c.breaks_chip || '')) + '</p>' + restoreText(c);
    h += '<h3>Cost</h3><p class="num">placed area ' + dArea(c.d_area_nom) + ' um2; ' + c.steps + ' grid step' + (c.steps === 1 ? '' : 's') + '</p>';
    h += '<h3>Collateral (tag ' + esc(state.tag) + ')</h3>' + (c.losses === null ? '<p class="muted">Not computed beyond the 12th candidate.</p>' :
      (c.losses.length ? '<ul>' + c.losses.map(function (w) { return '<li>' + wlLink(state, w) + ' gets worse</li>'; }).join('') + '</ul>' : '<p class="muted">No protocol gets worse.</p>'));
    return { title: 'What would it take: ' + U.wlName(state.data, R.wl), html: h + applyBtn(c.changes.map(function (ch) { return { knob: ch.knob, to: ch.to }; })) };
  }, function (body) { bindApply(body); });
  U.registerSide('sweep_point', function (state, ref) {
    var p = (state.points[ref.col] || state.point), k = knobDef(ref.knob);
    var r = A.sweep(p.arch, ref.knob, [ref.x], ref.metric, state.data)[0], info = metricInfo(state, ref.metric);
    var parts = ref.metric.split(':'), wl = (parts[0] === 'max_rate' || parts[0] === 'margin') ? parts[1] : null;
    var h = '<p><span class="mono">' + esc(ref.knob) + '</span> = <b>' + esc(ref.x) + '</b> (now ' + esc(valText(p.arch[ref.knob])) + ')</p>' +
      '<p>' + esc(info ? info[1] : ref.metric) + ': <span class="num">' + U.fmtBandHtml(r.y, info ? info[2] : '') + '</span></p>' +
      '<p>chip check ' + chipNote(r.chip, r.chip_driver) + '</p>';
    if (wl) {
      var a = Object.assign({}, p.arch); a[ref.knob] = ref.x;
      var cell = A.evaluateCell(a, wl, state.data, { lite: true });
      h += '<p>' + wlLink(state, wl) + ' ' + U.verdictBadge(cell.verdict) + ' <span class="small muted">binding ' + esc(cell.binding) + '</span></p>';
    }
    return { title: 'Sweep point: ' + (k ? k.label : ref.knob) + ' = ' + ref.x, html: h + applyBtn([{ knob: ref.knob, to: ref.x }]) };
  }, function (body) { bindApply(body); });

  // ------------------------------------------------------------ DOM (browser only)
  function hasDom(el) { return typeof document !== 'undefined' && el && el.addEventListener; }
  function bindApply(body) {
    if (!hasDom(body)) return;
    var bs = body.querySelectorAll('[data-ex-apply]');
    for (var i = 0; i < bs.length; i++) bs[i].addEventListener('click', function (e) {
      var ch; try { ch = JSON.parse(e.currentTarget.getAttribute('data-ex-apply')); } catch (err) { return; }
      var patch = {}; ch.forEach(function (c) { patch[c.knob] = c.to; });
      U.setState({ arch: patch, preset: '' }, true);
    });
  }
  var LAST = null;
  function mountExplore(el, state) {
    LAST = state;
    if (el.__exBound) return;
    el.__exBound = true;
    el.addEventListener('change', function (e) {
      var t = e.target, key = t && t.getAttribute ? t.getAttribute('data-ex') : null;
      if (!key || !LAST) return;
      var sw = Object.assign({}, vs(LAST).sweep || {});
      if (key === 'impactAll') U.setView('explore', { impactAll: t.checked });
      else if (key === 'impactSort') U.setView('explore', { impactSort: t.checked ? 'order' : 'flips' });
      else if (key === 'sweep-knob') U.setView('explore', { sweep: { knob: t.value, metric: sw.metric } });
      else if (key === 'metric') { sw.metric = t.value; U.setView('explore', { sweep: sw }); }
      else if (key === 'sweep-values') {
        var cfg = sweepConfig(LAST), vals = parseValues(cfg.def, t.value);
        sw.knob = cfg.knob; sw.values = vals.length ? vals : undefined; U.setView('explore', { sweep: sw });
      }
    });
    el.addEventListener('click', function (e) {
      var b = e.target && e.target.closest ? e.target.closest('[data-ex-act]') : null;
      if (!b) return;
      var act = b.getAttribute('data-ex-act');
      if (act === 'solve-all') U.setView('explore', { solveAll: true });
      else if (act === 'hide-solve-all') U.setView('explore', { solveAll: false });
    });
  }

  U.register('explore', { title: 'Explore', render: function (s) { return U.renderExplore(s); }, mount: function (el, s) { if (hasDom(el)) mountExplore(el, s); } });
})(ArchSimUI);
