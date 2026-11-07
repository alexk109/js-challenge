/* Trust tab: how far to believe the model.
 *   Calibration  K1-K20 model against measured (classes from data.vocab.classes), error dot plot, perf-a against perf-b,
 *                legacy 19-row rows that DIFFER from archsim (agreeing rows' verdicts are Capability's).
 *   Assumptions  every parameter editable lo/nom/hi (-> arch.param_overrides), method/status badges, the "what we are
 *                guessing" filter, the full Not modelled list (the only full list; cells show the relevant items only).
 *   ISA estimates  data.isa: what M0 already has, encoding room, levers, every effect row with rationale and cites.
 *                All of it is an estimate: no ISA variant has been built or measured.
 *   Data         blocks, links, workloads, ISS constants, provenance (sha256).
 * Side kinds: 'calibration' {id}, 'isa_effect' {wl, lever, value?}. Per-view state: state.view.trust = {pstatus}.
 * Pure render functions (QuickJS); DOM only in mount(), guarded.
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum, fmtBandHtml = U.fmtBandHtml;
  var W = 860, CHART_M = { t: 12 };

  function isNum(x) { return typeof x === 'number' && isFinite(x); }
  function r1(v) { return Math.round(v * 10) / 10; }
  function tv(state) { return (state.view && state.view.trust) || {}; }
  function mono(s) { return '<span class="mono small">' + esc(s) + '</span>'; }
  function citeItems(list) {
    return (list || []).map(function (c) { return '<li>' + mono(c.path) + (c.quote ? ' <q>' + esc(c.quote) + '</q>' : '') + '</li>'; }).join('');
  }
  function citeBlock(list) { return list && list.length ? '<ul class="cites">' + citeItems(list) + '</ul>' : '<p class="muted small">no citation</p>'; }
  function classMeaning(state, id) {
    var l = (state.data.vocab && state.data.vocab.classes) || [];
    for (var i = 0; i < l.length; i++) if (l[i].id === id) return l[i].meaning;
    return '';
  }

  // ------------------------------------------------------------ Calibration
  var CLASS_GLYPH = { match: '=', close: '~', off: '!=', definitional: 'def', not_modelled: 'n/m', identity: 'id', brackets: '[ ]', info: 'i' };
  function classBadge(state, c) {
    return '<span class="badge cls cls-' + esc(c) + '" data-class-badge="' + esc(c) + '" title="' + esc(classMeaning(state, c)) + '"><span aria-hidden="true">' +
      esc(CLASS_GLYPH[c] || '?') + '</span> ' + esc(String(c).replace('_', ' ')) + '</span>';
  }
  function modelCell(r) {
    var m = r.model, h = '<span class="num">' + esc(r.model_text !== undefined && r.model_text !== null ? r.model_text : (isNum(m) ? fmtNum(m) : '')) + '</span>';
    if (m && typeof m === 'object' && isNum(m.nom) && m.pess !== m.opt) {
      var mlo = fmtNum(Math.min(m.pess, m.opt)), mhi = fmtNum(Math.max(m.pess, m.opt));
      if (mlo !== mhi) h += ' <span class="lo num">' + esc(mlo) + '..' + esc(mhi) + '</span>';
    }
    return h;
  }
  function itemsHtml(state, r, open) {
    if (!r.items || r.items.length < 2) return '';
    return '<details class="items"' + (open ? ' open' : '') + ' data-key="items-' + esc(r.id) + '"><summary>' + r.items.length + ' items</summary>' +
      '<div class="tbl">' + U.table(['item', 'model', 'measured', 'unit', 'err %', 'class', 'note'], r.items.map(function (it) {
        return ['<span class="mono">' + esc(it.label) + '</span>', fmtBandHtml(it.model, ''),
          '<span class="num">' + esc(isNum(it.measured) ? fmtNum(it.measured) : (it.measured === null || it.measured === undefined ? '' : it.measured)) + '</span>',
          esc(it.unit || ''), '<span class="num">' + esc(isNum(it.error_pct) ? fmtNum(it.error_pct, 2) : '') + '</span>',
          classBadge(state, it.class) + (it.scored === false ? ' <span class="muted">not scored</span>' : ''), esc(it.note || '')];
      })) + '</div></details>';
  }
  function calErrorChart(state, rows) {
    var pts = [];
    rows.forEach(function (r) {
      var scored = (r.items || []).filter(function (it) { return it.scored === true && isNum(it.error_pct); });
      if (r.items && r.items.length > 1 && scored.length) {
        scored.forEach(function (it) { pts.push({ id: r.id + ': ' + it.label, e: it.error_pct, c: it.class }); });
      } else if (isNum(r.error_pct) && (r.class === 'match' || r.class === 'close' || r.class === 'off')) pts.push({ id: r.id, e: r.error_pct, c: r.class });
    });
    if (!pts.length) return '';
    var rowH = 16, h = pts.length * rowH + CHART_M.t + 30, L = 190, R = 16;
    var emax = Math.max.apply(null, pts.map(function (p) { return Math.abs(p.e); }).concat([1]));
    function px(e) { return r1(L + (e + emax) / (2 * emax) * (W - L - R)); }
    var g = '';
    U.niceTicks(-emax, emax, 6).forEach(function (t) {
      g += '<line class="gridline" x1="' + px(t) + '" x2="' + px(t) + '" y1="' + (CHART_M.t - 6) + '" y2="' + (h - 28) + '"/><text x="' + px(t) + '" y="' + (h - 14) + '" text-anchor="middle">' + esc(U.tickFmt(t)) + '</text>';
    });
    g += '<line class="axis" x1="' + px(0) + '" x2="' + px(0) + '" y1="' + (CHART_M.t - 6) + '" y2="' + (h - 28) + '" style="stroke:var(--ink)"/>';
    g += '<text x="' + (L + (W - L - R) / 2) + '" y="' + (h - 1) + '" text-anchor="middle">model error against measured, % (0 = agrees)</text>';
    var SH = { match: 'circle', close: 'square', off: 'triangle' }, CV = { match: '--good', close: '--warn', off: '--bad' };
    pts.forEach(function (p, i) {
      var y = CHART_M.t + i * rowH + 4;
      g += '<text x="' + (L - 8) + '" y="' + (y + 4) + '" text-anchor="end">' + esc(p.id) + '</text>' +
        '<line x1="' + px(0) + '" x2="' + px(p.e) + '" y1="' + y + '" y2="' + y + '" stroke="var(--axis)" stroke-width="1.5"/>' +
        U.mark(SH[p.c], px(p.e), y, 4, 'fill="var(' + CV[p.c] + ')" stroke="var(--ink)" stroke-width="0.5" data-cal-pt="' + esc(p.id) + '" data-class="' + p.c + '"') +
        '<text x="' + (px(p.e) + (p.e >= 0 ? 8 : -8)) + '" y="' + (y + 4) + '" text-anchor="' + (p.e >= 0 ? 'start' : 'end') + '">' + esc(fmtNum(p.e, 1)) + '% ' + esc(p.c) + '</text>';
    });
    return '<h3>Model error per scored point</h3><div class="chart cal-chart">' + U.svgWrap(W, h, g, 'Model error percent per scored calibration point') + '</div>';
  }
  function legacySection(state) {
    var d = state.data, live = state.points[0], leg = A.legacy19(d, live.arch), cmp = (leg.compare || []).filter(function (c) { return c.disagree; });
    var rows = cmp.map(function (c) {
      var cell = live.all.cells[c.wl], lim = cell && cell.limits.filter(function (l) { return l.id === c.binding; })[0];
      var why;
      if (c.archsim && c.archsim !== 'served') {
        why = 'archsim: ' + esc(c.archsim) + ', bound by <b>' + esc(lim ? lim.short || lim.label || lim.id : c.binding) + '</b>' +
          (c.blocked_by && c.blocked_by.length ? ' (blocked by ' + esc(c.blocked_by.join(', ')) + ')' : '') +
          '; the legacy tier model has no such limit, so it called the row ' + (c.legacy_t0 ? 'feasible' : 'infeasible') + '.';
      } else {
        why = 'archsim serves it although legacy T0 calls it infeasible' + (c.legacy_blockers && c.legacy_blockers.length ? ' (legacy blockers: ' + esc(c.legacy_blockers.join(', ')) + ')' : '') +
          ': the legacy blockers are the baseline capacities of the plan, not the current architecture.';
      }
      return ['<a href="javascript:void(0)" class="linkish"' + U.focusAttr(c.wl, 0) + ' data-tab="capability">' + esc(c.name) + '</a>',
        c.legacy_t0 ? 'feasible' : 'infeasible', c.archsim ? U.verdictBadge(c.archsim) : '', why];
    });
    return '<h3>Legacy 19-row model against archsim (rows that differ)</h3><p class="muted">Legacy tier counts: <span class="num">' + esc(leg.counts.join(', ')) +
      '</span>' + (leg.matches_expected ? ' (match the expected output of protocols.py)' : ' (differ from the expected output)') + '. ' +
      'Rows where both models agree are not listed; their archsim verdicts are in Capability.</p>' +
      (rows.length ? '<div class="tbl" data-legacy-diff="' + rows.length + '">' + U.table(['legacy row', 'legacy T0', 'archsim', 'why they differ'], rows) + '</div>'
        : '<p class="muted" data-legacy-diff="0">No row differs on the live point.</p>');
  }
  function renderCalibration(state) {
    var rows = A.calibration(state.data), d = state.data, counts = {};
    rows.forEach(function (r) { counts[r.class] = (counts[r.class] || 0) + 1; });
    var summary = '<p class="cal-summary">' + Object.keys(counts).map(function (c) { return classBadge(state, c) + ' <span class="num">' + counts[c] + '</span>'; }).join(' &nbsp; ') + '</p>';
    var body = rows.map(function (r) {
      return '<tr data-cal="' + esc(r.id) + '" data-class="' + esc(r.class) + '"><td><button type="button" class="linkish mono"' + U.sideAttr({ kind: 'calibration', id: r.id }) + '>' + esc(r.id) + '</button></td>' +
        '<td>' + esc(r.quantity) + itemsHtml(state, r) + '</td><td>' + modelCell(r) + '</td>' +
        '<td><span class="num">' + esc(r.measured_text !== undefined && r.measured_text !== null ? r.measured_text : (r.measured === null ? '' : r.measured)) + '</span></td>' +
        '<td>' + esc(r.unit || '') + '</td><td>' + esc(r.env || '') + '</td>' +
        '<td class="r"><span class="num">' + esc(isNum(r.error_pct) ? fmtNum(r.error_pct, 2) : '') + '</span></td>' +
        '<td>' + classBadge(state, r.class) + '</td><td class="mono small">' + esc(r.source || '') + '</td><td class="small">' + esc(r.note || '') + '</td></tr>';
    }).join('');
    var tbl = '<div class="tbl"><table class="cal-table"><thead><tr><th>#</th><th>quantity</th><th>model</th><th>measured</th><th>unit</th><th>env</th><th class="r">err %</th><th>class</th><th>source</th><th>note</th></tr></thead><tbody>' + body + '</tbody></table></div>';
    var legend = '<p class="muted" data-class-legend>Classes: ' + ((d.vocab && d.vocab.classes) || []).map(function (c) { return esc(c.id.replace('_', ' ')) + ' (' + esc(c.meaning) + ')'; }).join('; ') + '.</p>';
    var ab = (d.perf_ab || []).map(function (r) {
      return ['<span class="mono">' + esc(r.id) + '</span>', esc(r.protocol || ''), esc(r.variant || ''), '<span class="num">' + esc(fmtNum(r.a)) + '</span>',
        '<span class="num">' + esc(fmtNum(r.b)) + '</span>', '<span class="num">' + esc(fmtNum(r.ratio, 3)) + '</span>', esc(r.class || ''), esc(r.note || '')];
    });
    return '<h2>Calibration: model against measured (' + rows.length + ' points)</h2>' + summary + tbl + legend + calErrorChart(state, rows) +
      '<h3>perf-a against perf-b</h3><div class="tbl">' + U.table(['id', 'protocol', 'variant', 'a', 'b', 'a/b', 'class', 'note'], ab) + '</div>' + legacySection(state);
  }

  // ------------------------------------------------------------ Assumptions
  var GUESS = ['judgement', 'unverified', 'disputed', 'estimate'];
  var ISA_PARAM = /^d_isa_/;
  // param id -> [limit ids / "chip:<item>"] over the cells and the chip records of the live point
  function usedBy(state) {
    var out = {};
    function walk(terms, who) {
      (terms || []).forEach(function (t) {
        if (t.ref && t.ref.kind === 'param') { var a = out[t.ref.id] = out[t.ref.id] || {}; a[who] = true; }
        if (t.terms) walk(t.terms, who);
      });
    }
    var all = state.points[0].all;
    Object.keys(all.cells).forEach(function (w) { all.cells[w].limits.forEach(function (l) { walk(l.terms, l.id); }); });
    var recs = state.points[0].chip.records || {};
    Object.keys(recs).forEach(function (k) { walk(recs[k].terms, 'chip:' + k); });
    var r = {};
    Object.keys(out).forEach(function (k) { r[k] = Object.keys(out[k]); });
    return r;
  }
  function pnum(v) { return isNum(v) ? String(v) : ''; }
  function paramRow(state, p, ub) {
    var ov = (state.points[0].arch.param_overrides || {})[p.id], forced = p.forced_by_env, isa = ISA_PARAM.test(p.id);
    var inp = function (f) {
      return '<input type="number" step="any" class="param-input" data-param="' + esc(p.id) + '" data-field="' + f + '" value="' + pnum(p[f]) + '" aria-label="' + esc(p.id + ' ' + f) + '"' + (forced ? ' disabled' : '') + '>';
    };
    var det = '<details data-key="p-' + esc(p.id) + '"><summary>citation, sources</summary>' + citeBlock(p.cite) +
      ((p.sources || []).length ? '<p class="small">sources: ' + mono(p.sources.join('; ')) + '</p>' : '') +
      ((p.assumptions || []).length ? '<p class="small">assumptions: ' + esc(p.assumptions.join(' | ')) + '</p>' : '') + '</details>';
    return '<tr data-param="' + esc(p.id) + '" data-status="' + esc(p.status || '') + '"' + (isa ? ' data-isa-param="1"' : '') + (ov ? ' data-edited="1" class="edited"' : '') + '>' +
      '<td><button type="button" class="linkish mono"' + U.sideAttr({ kind: 'param', id: p.id }) + '>' + esc(p.id) + '</button>' + (ov ? ' <span class="badge st-edited">edited</span>' : '') +
      (isa ? ' <span class="badge st-isa" title="ISA lever fmax estimate: no ISA variant has been built">ISA estimate</span>' : '') + '<div class="small">' + esc(p.label || '') + '</div></td>' +
      '<td>' + inp('lo') + '</td><td>' + inp('nom') + '</td><td>' + inp('hi') + '</td><td>' + esc(p.unit || '') +
      (forced ? ' <span class="muted small">forced to 0 by env=' + esc(forced) + '</span>' : '') + '<div class="err small" role="alert"></div></td>' +
      '<td>' + U.methodBadge(p.method) + ' ' + U.statusBadge(p.status) + '</td><td>' + det + '</td>' +
      '<td class="small mono">' + esc((ub[p.id] || []).join(', ')) + '</td>' +
      '<td><button type="button" data-reset="' + esc(p.id) + '"' + (ov ? '' : ' disabled') + '>reset</button></td></tr>';
  }
  function notModelled(state) {
    var items = state.data.not_modelled.items;
    return '<div class="notmodelled" role="note"><h3>Not modelled (' + items.length + ')</h3><p class="muted small">The full list. A cell or chip derivation shows only the items relevant to it.</p><ul>' +
      items.map(function (n) { return '<li data-nm="' + esc(n.id) + '">' + esc(n.text) + '</li>'; }).join('') + '</ul></div>';
  }
  function renderAssumptions(state) {
    var d = state.data, R = A.resolveParams(state.points[0].arch, d), ub = usedBy(state), flt = tv(state).pstatus || '';
    var ids = Object.keys(d.params), statuses = {}, ov = state.points[0].arch.param_overrides || {};
    ids.forEach(function (id) { statuses[d.params[id].status] = true; });
    var shown = ids.filter(function (id) {
      var s = d.params[id].status;
      if (!flt) return true;
      if (flt === 'guess') return GUESS.indexOf(s) >= 0;
      if (flt === 'isa') return ISA_PARAM.test(id);
      if (flt === 'edited') return !!ov[id];
      return s === flt;
    });
    var nEdit = Object.keys(ov).length;
    function opt(v, label) { return '<option value="' + esc(v) + '"' + (flt === v ? ' selected' : '') + '>' + esc(label) + '</option>'; }
    var fsel = '<label>Show <select data-pfilter aria-label="parameter filter">' + opt('', 'all parameters') +
      opt('guess', 'what we are guessing (' + GUESS.join(', ') + ')') + opt('isa', 'ISA estimates (d_isa_*)') + opt('edited', 'edited by you') +
      Object.keys(statuses).map(function (s) { return opt(s, 'status: ' + s); }).join('') + '</select></label>';
    var rows = shown.map(function (id) { return paramRow(state, R[id], ub); }).join('');
    var note = state.focus.col > 0 ? '<p class="muted">Edits apply to the live point (not the pinned point being shown).</p>' : '';
    return '<h2>Assumptions: parameters (' + shown.length + ' of ' + ids.length + ')</h2><div class="ctl-row">' + fsel +
      ' <button type="button" data-reset-all' + (nEdit ? '' : ' disabled') + '>reset all (' + nEdit + ' edited)</button></div>' +
      '<p class="muted">Edit lo / nom / hi; every view recomputes. A row must satisfy lo &lt;= nom &lt;= hi; it is applied when you press Enter or leave the row, in any edit order. Edits are kept in the page state (arch.param_overrides). ' +
      'Rows with status judgement, unverified, disputed or estimate are our guesses; the filter lists them.</p>' + note +
      '<div class="tbl"><table class="inputs-table"><thead><tr><th>parameter</th><th>lo</th><th>nom</th><th>hi</th><th>unit</th><th>method / status</th><th>citation</th><th>used by limits</th><th></th></tr></thead><tbody>' + rows + '</tbody></table></div>' +
      notModelled(state);
  }

  // ------------------------------------------------------------ ISA estimates
  var EFFECT_FIELDS = [['d_words', 'IMEM words'], ['d_ipu', 'instructions per unit'], ['d_reflex_sm', 'reflex, SM cycles'], ['d_decide_sm', 'decision, SM cycles']];
  function effKey(e) { return e.lever + '@' + e.wl + (e.value !== undefined ? '@' + e.value : ''); }
  function effRef(e) { var r = { kind: 'isa_effect', wl: e.wl, lever: e.lever }; if (e.value !== undefined) r.value = e.value; return r; }
  function effectActive(state, e) {
    var a = state.points[0].arch[e.lever];
    return e.value !== undefined ? a === e.value : !!a;
  }
  function bandCell(b) { return b ? fmtBandHtml({ nom: b.nom, lo: b.lo, hi: b.hi }, '') : ''; }
  function renderIsa(state) {
    var I = state.data.isa || {}, effects = I.effects || [], levers = I.levers || {}, enc = I.encoding || {};
    var head = '<h2>ISA estimates</h2><p class="isa-warning" role="note"><b>These are estimates.</b> ' + esc(I.src || '') +
      ' Rows marked <i>judgement</i> are scaled from a neighbouring count; rows marked <i>estimate</i> come from a hand rewrite of a shipped program. The model uses them only when the lever is on.</p>';
    var m0 = '<h3>Already in M0 (not levers)</h3><ul class="isa-m0">' + (I.m0_present || []).map(function (m) {
      return '<li data-m0="1"><b>' + esc(m.feature) + '</b>' + citeBlock(m.cite) + '</li>'; }).join('') + '</ul>';
    var encT = U.table(['instruction width', 'free major opcodes', 'free minor slots', 'basis'], Object.keys(enc).map(function (b) {
      var e = enc[b];
      return ['<span class="num">' + esc(b) + ' bit</span>', '<span class="num">' + esc(e.major_free) + '</span>', '<span class="num">' + esc(e.minor_free) + '</span>',
        '<span class="small">' + esc(e.note || '') + '</span><details data-key="enc-' + esc(b) + '"><summary>citations</summary>' + citeBlock(e.cite) + '</details>'];
    }));
    var levT = U.table(['lever', 'what it is', 'slots used (major / minor) by width', 'area blocks', 'fmax parameter', 'why other rows are unaffected'], Object.keys(levers).map(function (id) {
      var l = levers[id], sl = Object.keys(l.slots || {}).map(function (b) { return b + ': ' + l.slots[b].major + ' / ' + l.slots[b].minor; }).join('; ');
      return ['<span class="mono">' + esc(id) + '</span>', esc(l.label || '') + '<details data-key="lev-' + esc(id) + '"><summary>citations</summary>' + citeBlock(l.cite) + '</details>',
        '<span class="num small">' + esc(sl) + '</span>', mono((l.blocks || []).join(', ')),
        l.fmax_param ? '<button type="button" class="linkish mono"' + U.sideAttr({ kind: 'param', id: l.fmax_param }) + '>' + esc(l.fmax_param) + '</button>' : '',
        '<span class="small">' + esc(l.default_rationale || '') + '</span>'];
    }));
    var effRows = effects.map(function (e) {
      var wlLabel = U.wlName(state.data, e.wl);
      return '<tr data-isa-effect="' + esc(effKey(e)) + '"' + (effectActive(state, e) ? ' data-active="1"' : '') + '>' +
        '<td><button type="button" class="linkish mono"' + U.sideAttr(effRef(e)) + '>' + esc(e.lever) + (e.value !== undefined ? ' = ' + esc(e.value) : '') + '</button>' +
        (effectActive(state, e) ? ' <span class="badge">on in live point</span>' : '') + '</td>' +
        '<td>' + esc(wlLabel) + '<div class="small mono">' + esc(e.wl) + '</div></td>' +
        EFFECT_FIELDS.map(function (f) { return '<td class="r">' + bandCell(e[f[0]]) + '</td>'; }).join('') +
        '<td class="small mono">' + esc((e.enables || []).join(', ')) + '</td>' +
        '<td>' + U.methodBadge(e.method) + ' ' + U.statusBadge(e.status) + '</td>' +
        '<td><details data-key="eff-' + esc(effKey(e)) + '"><summary>rationale, cites</summary><p class="small">' + esc(e.rationale) + '</p>' +
        citeBlock(e.cite) + '<p class="small">basis: ' + mono(e.src || '') + (e.program ? ' ; program ' + mono(e.program) : '') + '</p></details></td></tr>';
    }).join('');
    var effT = '<h3>Effect rows (' + effects.length + ')</h3><p class="muted small">Delta per workload when the lever is on: lo / nom / hi. ' +
      EFFECT_FIELDS.map(function (f) { return f[0] + ' = ' + f[1]; }).join('; ') + '.</p>' +
      '<div class="tbl"><table class="isa-table"><thead><tr><th>lever</th><th>workload</th>' + EFFECT_FIELDS.map(function (f) { return '<th class="r" title="' + esc(f[1]) + '">' + f[0] + '</th>'; }).join('') +
      '<th>enables</th><th>method / status</th><th>rationale</th></tr></thead><tbody>' + effRows + '</tbody></table></div>';
    return head + m0 + '<h3>Encoding room</h3><div class="tbl">' + encT + '</div><h3>Levers</h3><div class="tbl">' + levT + '</div>' + effT;
  }

  // ------------------------------------------------------------ Data
  function renderData(state) {
    var d = state.data;
    var blocks = U.table(['block', 'area lo / nom / hi (um2)', 'per', 'basis', 'method / status', 'sources'], Object.keys(d.blocks).map(function (id) {
      var b = d.blocks[id];
      return ['<span class="mono">' + esc(id) + '</span><div class="small">' + esc(b.label || '') + '</div>', fmtBandHtml(b.area, ''), esc(b.per || ''), esc(b.basis || ''),
        U.methodBadge(b.method) + ' ' + U.statusBadge(b.status), '<span class="small mono">' + esc((b.sources || []).join('; ')) + '</span>'];
    }));
    var links = U.table(['link', 'kind', 'pins in / out', 'width (bits)', 'method / status', 'sources'], Object.keys(d.links).map(function (id) {
      var l = d.links[id];
      return ['<span class="mono">' + esc(id) + '</span><div class="small">' + esc(l.label || '') + '</div>', esc(l.kind || ''), esc(l.pins_in + ' / ' + l.pins_out), esc(l.width_bits),
        U.methodBadge(l.method) + ' ' + U.statusBadge(l.status), '<span class="small mono">' + esc((l.sources || []).join('; ')) + '</span>'];
    }));
    var wls = U.table(['workload', 'name', 'tags', 'role', 'rate (Hz)', 'engines', 'pins', 'fields with a source'], d.workloads.map(function (w) {
      return ['<span class="mono">' + esc(w.id) + '</span>', esc(w.name || ''), esc((w.tags || []).join(', ')), esc(w.role || ''),
        '<span class="num">' + esc(isNum(w.rate_hz) ? fmtNum(w.rate_hz) : '') + '</span>', esc(w.engines), esc(w.pins),
        '<details data-key="w-' + esc(w.id) + '"><summary>' + Object.keys(w.src || {}).length + ' fields</summary><ul class="cites">' + Object.keys(w.src || {}).map(function (k) {
          return '<li><span class="mono">' + esc(k) + '</span> = <span class="num">' + esc(JSON.stringify(w[k])) + '</span> <span class="small muted">' + esc(w.src[k]) + '</span></li>'; }).join('') + '</ul></details>'];
    }));
    var iss = Object.keys(d.iss).filter(function (k) { return k !== 'src' && k !== 'pins' && k !== 'sources'; }).map(function (k) { return [k, d.iss[k]]; });
    var issT = U.table(['ISS constant', 'value'], iss.concat([['pins', JSON.stringify(d.iss.pins)]]).map(function (r) {
      return ['<span class="mono">' + esc(r[0]) + '</span>', '<span class="num">' + esc(r[1]) + '</span>']; })) +
      '<p class="small muted">source ' + esc(d.iss.src || '') + '</p><ul class="cites small">' + (d.iss.sources || []).map(function (x) { return '<li class="mono">' + esc(x) + '</li>'; }).join('') + '</ul>';
    var srcs = Object.keys(d.sources || {}), st = d.stamp || {};
    var srcT = '<details data-key="srcs"><summary>Input files (' + srcs.length + ') with sha256</summary><div class="tbl">' + U.table(['file', 'sha256'], srcs.map(function (p) {
      return ['<span class="mono small">' + esc(p) + '</span>', '<span class="mono small">' + esc(d.sources[p]) + '</span>']; })) + '</div></details>' +
      '<p class="small muted">build ' + esc(st.git_describe_at_build || '') + ' ; sources sha256 ' + esc(st.sources_sha256 || '') + '</p>';
    function sec(t, h) { return '<details data-key="data-' + esc(t) + '"><summary><b>' + esc(t) + '</b></summary><div class="tbl">' + h + '</div></details>'; }
    return '<h2>Data</h2>' + sec('Blocks (area)', blocks) + sec('Host links', links) + sec('Workloads', wls) + sec('ISS constants', issT) + '<h3>Provenance</h3>' + srcT;
  }

  // ------------------------------------------------------------ view
  U.renderTrust = function (state) {
    function sec(key, open, inner) { return '<details class="trust-sec" data-trust-sec="' + key + '" data-key="trust-' + key + '"' + (open ? ' open' : '') + '><summary>' + SECT[key] + '</summary>' + inner + '</details>'; }
    var SECT = { calibration: 'Calibration', assumptions: 'Assumptions', isa: 'ISA estimates', data: 'Data' };
    return '<div class="v-trust"><p class="muted">How far to believe the model: measured against model, the numbers we are guessing, and the sources.</p>' +
      sec('calibration', true, renderCalibration(state)) + sec('assumptions', true, renderAssumptions(state)) +
      sec('isa', false, renderIsa(state)) + sec('data', false, renderData(state)) + '</div>';
  };

  // ------------------------------------------------------------ side panel kinds
  U.registerSide('calibration', function (state, ref) {
    var r = A.calibration(state.data).filter(function (x) { return x.id === ref.id; })[0];
    if (!r) return { title: ref.id, html: '<p class="muted">No calibration row <code>' + esc(ref.id) + '</code>.</p>' };
    return { title: r.id + ': ' + r.quantity, html: '<div class="v-trust"><p>' + classBadge(state, r.class) + ' <span class="muted">' + esc(classMeaning(state, r.class)) + '</span></p>' +
      '<p class="num">model ' + modelCell(r) + ' ; measured ' + esc(r.measured_text !== undefined && r.measured_text !== null ? r.measured_text : r.measured) + ' ' + esc(r.unit || '') +
      ' ; error ' + esc(isNum(r.error_pct) ? fmtNum(r.error_pct, 2) + ' %' : 'n/a') + '</p>' +
      (r.env ? '<p class="small">environment: ' + esc(r.env) + '</p>' : '') +
      '<p class="mono small">' + esc(r.source || '') + '</p>' + (r.note ? '<p class="muted">' + esc(r.note) + '</p>' : '') + itemsHtml(state, r, true) + '</div>' };
  });
  U.registerSide('isa_effect', function (state, ref) {
    var e = ((state.data.isa && state.data.isa.effects) || []).filter(function (x) {
      return x.wl === ref.wl && x.lever === ref.lever && (ref.value === undefined || x.value === ref.value);
    })[0];
    if (!e) return { title: ref.lever + ' @ ' + ref.wl, html: '<p class="muted">No ISA effect row for this lever and workload.</p>' };
    var lev = (state.data.isa.levers || {})[e.lever] || {};
    var h = '<div class="v-trust"><p class="isa-warning"><b>An estimate.</b> ' + esc(state.data.isa.src || '') + '</p>' +
      '<p>' + U.methodBadge(e.method) + ' ' + U.statusBadge(e.status) + (effectActive(state, e) ? ' <span class="badge">on in the live point</span>' : ' <span class="badge">off in the live point</span>') + '</p>' +
      '<p><b>' + esc(lev.label || e.lever) + '</b> on <b>' + esc(U.wlName(state.data, e.wl)) + '</b>' + (e.value !== undefined ? ' (width ' + esc(e.value) + ' bit)' : '') + '</p>' +
      '<table><thead><tr><th>effect</th><th class="r">lo / nom / hi</th></tr></thead><tbody>' + EFFECT_FIELDS.map(function (f) {
        return '<tr><td><code>' + f[0] + '</code> <span class="muted small">' + esc(f[1]) + '</span></td><td class="r">' + bandCell(e[f[0]]) + '</td></tr>'; }).join('') + '</tbody></table>' +
      ((e.enables || []).length ? '<p class="small">enables limits: ' + mono(e.enables.join(', ')) + '</p>' : '') +
      '<h4>Rationale</h4><p>' + esc(e.rationale) + '</p><h4>Citations</h4>' + citeBlock(e.cite) +
      '<p class="small">basis: ' + mono(e.src || '') + (e.program ? ' ; program ' + mono(e.program) : '') + '</p>' +
      (lev.fmax_param ? '<p class="small">cycle-time cost of the lever: <button type="button" class="linkish mono"' + U.sideAttr({ kind: 'param', id: lev.fmax_param }) + '>' + esc(lev.fmax_param) + '</button></p>' : '') +
      '</div>';
    return { title: e.lever + ' @ ' + e.wl, html: h };
  });

  // ------------------------------------------------------------ DOM hooks (browser only)
  function hasDom(el) { return !!el && typeof el.querySelectorAll === 'function'; }
  function on(el, selector, ev, fn) {
    var els = el.querySelectorAll(selector);
    for (var i = 0; i < els.length; i++) els[i].addEventListener(ev, fn);
  }
  function mountTrust(el, state) {
    on(el, '[data-pfilter]', 'change', function (e) { U.setView('trust', { pstatus: e.target.value }); });
    // A row is committed as a triple when focus leaves it or on Enter (not on every field change), so any edit order works;
    // lo <= nom <= hi is checked on the committed triple only.
    function commit(tr) {
      var id = tr.getAttribute('data-param'), v = {}, ok = true;
      ['lo', 'nom', 'hi'].forEach(function (f) {
        var x = parseFloat(tr.querySelector('input[data-field="' + f + '"]').value);
        if (!isNum(x)) ok = false;
        v[f] = x;
      });
      var err = tr.querySelector('.err');
      if (!ok || !(v.lo <= v.nom && v.nom <= v.hi)) { err.textContent = 'needs lo <= nom <= hi'; tr.classList.add('invalid'); return; }
      err.textContent = ''; tr.classList.remove('invalid');
      var ov = Object.assign({}, state.points[0].arch.param_overrides || {}), b = state.data.params[id];
      if (v.lo === b.lo && v.nom === b.nom && v.hi === b.hi) delete ov[id]; else ov[id] = v;
      if (JSON.stringify(ov) === JSON.stringify(state.points[0].arch.param_overrides || {})) return;   // nothing to commit
      U.setState({ arch: { param_overrides: ov } });
    }
    on(el, 'input.param-input', 'focusout', function (e) {
      var tr = e.target.closest('tr');
      if (tr && !(e.relatedTarget && tr.contains(e.relatedTarget))) commit(tr);
    });
    on(el, 'input.param-input', 'keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); commit(e.target.closest('tr')); }
    });
    on(el, '[data-reset]', 'click', function (e) {
      var ov = Object.assign({}, state.points[0].arch.param_overrides || {});
      delete ov[e.currentTarget.getAttribute('data-reset')];
      U.setState({ arch: { param_overrides: ov } });
    });
    on(el, '[data-reset-all]', 'click', function () { U.setState({ arch: { param_overrides: {} } }); });
  }

  U.register('trust', { title: 'Trust', render: function (s) { return U.renderTrust(s); }, mount: function (el, s) { if (hasDom(el)) mountTrust(el, s); } });
})(ArchSimUI);
