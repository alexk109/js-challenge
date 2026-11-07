/* Capability tab: the protocol matrix (protocols x architecture points; single protocols, protocol-level limits
 * only), the Scenario sub-view (protocols running at the same time, ArchSim.mix) and the side-panel kind `cell`
 * (PLAN-v2 1.2 rows 8, 11, 12, 15-22; 2.1 "how a failing chip check is shown"; 2.5).
 *
 * Matrix: rows grouped by verdict and by binding reason (view option `group`), columns = live point + pinned points. A cell
 * shows verdict, binding limit with margin and the pess..opt bar, plus `max <rate>` on scalable rows. Chip check: the column
 * header carries a chip badge (verdict + driver, click opens the `chip` side item); a failing column is hatched (`chip-fail`)
 * with exactly one caption under the header. No cell ever changes because of the chip.
 * Per-view state (state.view.capability): sub 'single'|'scenario', group 'verdict'|'binding'|'none', diff (only rows that
 * differ across points), all (phone: show every point column).
 * Pure render functions (QuickJS); DOM only in mount().
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum, fmtBand = U.fmtBand, fmtBandHtml = U.fmtBandHtml, workloadOf = U.workloadOf;
  var VERDICTS = ['served', 'marginal', 'impossible'];
  var GROUPS = [['verdict', 'verdict, then binding limit'], ['binding', 'binding limit'], ['none', 'protocol order']];

  function opts(state) {
    var v = (state.view && state.view.capability) || {};
    return {
      sub: v.sub === 'scenario' ? 'scenario' : 'single',
      group: GROUPS.some(function (g) { return g[0] === v.group; }) ? v.group : 'verdict',
      diff: !!v.diff, all: !!v.all
    };
  }

  // ------------------------------------------------------------ cells
  var rateCache = {};
  function maxRates(arch, wl, data) {
    var key = JSON.stringify(arch) + '|' + wl;
    if (!rateCache[key]) {
      rateCache[key] = {
        nom: A.maxRate(arch, wl, data, 'nom').rate_hz, pess: A.maxRate(arch, wl, data, 'pess').rate_hz,
        opt: A.maxRate(arch, wl, data, 'opt').rate_hz
      };
    }
    return rateCache[key];
  }
  function fmtRate(hz) {
    var a = Math.abs(hz);
    if (a >= 1e6) return fmtNum(hz / 1e6) + ' MHz';
    if (a >= 1e3) return fmtNum(hz / 1e3) + ' kHz';
    return fmtNum(hz) + ' Hz';
  }
  function fmtRateBand(r) {
    var lo = fmtRate(Math.min(r.pess, r.opt)), hi = fmtRate(Math.max(r.pess, r.opt));
    return fmtRate(r.nom) + (lo !== hi ? ' (' + lo + '..' + hi + ')' : '');
  }
  function limRec(cell, id) {
    for (var i = 0; i < cell.limits.length; i++) if (cell.limits[i].id === id) return cell.limits[i];
    return null;
  }

  function cellHtml(state, cell, wl, pt) {
    var h = cell.headline, rec = limRec(cell, h.limit), scale = rec ? rec.scale : 0;
    var band = { nom: h.nom, pess: h.pess, opt: h.opt };
    var selected = state.focus.wl === wl.id && state.focus.col === pt.col;
    var est = U.cellIsa(cell) ? ' ' + U.estimateMark('this verdict rests on ISA lever estimates (judgement)') : '';
    var s = U.verdictBadge(cell.verdict) + est + '<span class="mx-head"><b>' + esc(h.short) + '</b> ' + fmtBandHtml(band, h.unit) + '</span>' +
      U.pessOptBar(h.pess, h.nom, h.opt, scale, h.short + ' margin ' + fmtBand(band, h.unit));
    if (wl.scalable) s += '<span class="mx-max">max ' + esc(fmtRateBand(maxRates(pt.arch, wl.id, state.data))) + '</span>';
    return '<td class="mx-cell v-' + esc(cell.verdict) + (selected ? ' selected' : '') + (pt.chip.verdict === 'fail' ? ' chip-fail' : '') +
      ' mx-col' + (pt.col === state.focus.col ? ' mx-focus-col' : '') + '">' +
      '<button type="button" class="mx-cellbtn" data-cell="' + esc(wl.id) + '" data-col="' + pt.col + '"' +
      U.focusAttr(wl.id, pt.col) + U.sideAttr({ kind: 'cell', wl: wl.id, col: pt.col }) + ' title="click for the derivation">' + s + '</button></td>';
  }

  function chipItemShort(pt, id) {
    var it = pt.chip.items.filter(function (q) { return q.id === id; })[0];
    return it ? it.short : id;
  }
  function headerCell(state, pt) {
    var c = pt.all.counts, cv = pt.chip.verdict;
    var suffix = cv === 'fail' ? ' (chip fails)' : cv === 'marginal' ? ' (chip marginal)' : '';
    return '<th class="mx-colhead mx-col' + (pt.col === state.focus.col ? ' mx-focus-col' : '') + '" scope="col">' +
      '<button type="button" class="mx-colname"' + U.focusAttr(null, pt.col) + ' title="show this point in the other tabs">' +
      esc(pt.col === 0 ? 'live: ' + pt.label : pt.label) + '</button>' +
      (U.isaActive(pt.arch).length ? '<span class="mx-isa">ISA levers on ' + U.estimateMark('ISA levers are on at this point: cells marked estimate rest on judged effects') + '</span>' : '') +
      '<button type="button" class="mx-chipbadge"' + U.sideAttr({ kind: 'chip', item: pt.chip.driver, col: pt.col }) +
      ' title="chip check of this point; click for the driver\'s derivation">chip ' + U.chipBadge(cv) + ' <span class="muted">' +
      esc(chipItemShort(pt, pt.chip.driver)) + '</span></button>' +
      '<span class="mx-colsub num">&#10003; ' + c.served + ' served, ~ ' + c.marginal + ' marginal, &#10007; ' + c.impossible +
      ' impossible' + esc(suffix) + '</span></th>';
  }

  function differs(points, id) {
    var v = {}, b = {};
    points.forEach(function (p) { var c = p.all.cells[id]; if (c) { v[c.verdict] = 1; b[c.binding] = 1; } });
    return { verdict: Object.keys(v).length > 1, binding: Object.keys(b).length > 1 };
  }

  // Rows of the matrix, grouped by the focused point's verdict / binding reason.
  function groupRows(state, ids, mode) {
    var cells = state.point.all.cells;
    function bindShort(id) { return cells[id].headline.short; }
    if (mode === 'none') return [{ key: 'all', title: '', ids: ids }];
    var out = [];
    if (mode === 'binding') {
      var by = {};
      ids.forEach(function (id) { var b = bindShort(id); (by[b] = by[b] || []).push(id); });
      Object.keys(by).sort(function (a, b) { return by[b].length - by[a].length || (a < b ? -1 : 1); }).forEach(function (b) {
        var g = by[b].slice().sort(function (x, y) { return VERDICTS.indexOf(cells[x].verdict) - VERDICTS.indexOf(cells[y].verdict); });
        out.push({ key: 'b:' + b, title: 'binding: ' + b, ids: g });
      });
      return out;
    }
    VERDICTS.forEach(function (v) {
      var inV = ids.filter(function (id) { return cells[id].verdict === v; });
      if (!inV.length) return;
      out.push({ key: 'v:' + v, title: v, head: true, verdict: v, ids: [] });
      var by2 = {}, order = [];
      inV.forEach(function (id) { var b = bindShort(id); if (!by2[b]) { by2[b] = []; order.push(b); } by2[b].push(id); });
      order.sort(function (a, b) { return by2[b].length - by2[a].length || (a < b ? -1 : 1); });
      out[out.length - 1].count = inV.length;
      order.forEach(function (b) { out.push({ key: 'v:' + v + '|b:' + b, title: 'binding: ' + b, sub: true, ids: by2[b] }); });
    });
    return out;
  }

  function toolbar(state, o) {
    var data = state.data, tags = U.tagsOf(data), tagText = {};
    ((data.workload_tags && data.workload_tags.items) || []).forEach(function (t) { tagText[t.id] = t.text; });
    var h = '<div class="cap-sub" role="tablist" aria-label="capability view">' +
      '<button type="button" role="tab" data-cap-sub="single" aria-selected="' + (o.sub === 'single') + '">Single protocols</button>' +
      '<button type="button" role="tab" data-cap-sub="scenario" aria-selected="' + (o.sub === 'scenario') + '">Scenario (at the same time)</button></div>';
    if (o.sub !== 'single') return h;
    h += '<div class="mx-controls"><span class="mx-chips" role="group" aria-label="filter by tag">' + tags.map(function (t) {
      return '<button type="button" class="mx-chip" data-cap-tag="' + esc(t) + '" aria-pressed="' + (t === state.tag) + '"' +
        (tagText[t] ? ' title="' + esc(tagText[t]) + '"' : '') + '>' + esc(t) + '</button>';
    }).join('') + '</span>' +
      '<label class="mx-search">search <input type="search" data-mx-search placeholder="protocol"></label>' +
      '<label class="mx-sel">group rows by <select data-cap-group aria-label="group rows by">' + GROUPS.map(function (g) {
        return '<option value="' + g[0] + '"' + (g[0] === o.group ? ' selected' : '') + '>' + esc(g[1]) + '</option>';
      }).join('') + '</select></label>' +
      (state.points.length > 1 ? '<label class="mx-chk"><input type="checkbox" data-cap-diff' + (o.diff ? ' checked' : '') +
        '> only rows that differ across points</label>' : '') +
      (state.points.length > 1 ? '<button type="button" class="mx-allpts" data-cap-all aria-pressed="' + o.all + '">all points</button>' : '') +
      '</div>';
    var cur = tagText[state.tag];
    h += '<p class="muted mx-legend">' + (cur ? '<b>' + esc(state.tag) + '</b>: ' + esc(cur) : (state.tag === 'all' ? '<b>all</b>: every protocol row in the data.' : '')) + '</p>';
    return h;
  }

  function howToRead() {
    return '<details class="mx-how" data-key="how to read"><summary>How to read this matrix</summary>' +
      '<p><b>Cell</b>: the protocol verdict on that architecture point, the <b>binding</b> limit with its margin (nominal, then pessimistic..optimistic: ' +
      'a range from parameter bounds or judgement, not a confidence interval) and a bar of that range around zero. Click a cell for the ranked ' +
      'limits and the derivation in the side panel.</p>' +
      '<p><b>binding</b> = the smallest-margin limit among those that block the cell (negative even at the optimistic end); if none blocks, the overall ' +
      'smallest-margin limit. The closest limit overall is the first row of the ranked table in the side panel.</p>' +
      '<p><b>Chip check</b> is judged once per point, in the Configure card and in the column header badge. ' +
      'It never changes a cell: a hatched column says "the verdicts below assume the chip could be built".</p>' +
      '<p><b>Rows</b> are single protocols: the chip loaded with that protocol\'s firmware. Protocols running at the same time are in the Scenario view.</p></details>';
  }

  function renderSingle(state, o) {
    var data = state.data, pts = state.points, ids = Object.keys(state.point.all.cells), h = '';
    var diffIds = pts.length > 1 ? ids.filter(function (id) { var d = differs(pts, id); return d.verdict || d.binding; }) : [];
    var shown = o.diff && pts.length > 1 ? diffIds : ids;
    if (pts.length === 1) {
      h += '<p class="muted mx-pinhint" data-pin-hint>To compare architectures side by side, pin a preset (select it in Configure and press <b>pin</b>) ' +
        'or the current point (<b>Pin current</b>): each pinned point becomes a column here.</p>';
    }
    if (pts.length > 1) {
      h += '<p class="muted mx-diffnote">' + diffIds.length + ' of ' + ids.length + ' protocols differ in verdict or binding limit across the ' +
        pts.length + ' points.</p>';
    }
    var ncols = pts.length + 1;
    h += '<div class="tbl mx-wrap"><table class="mx-table"><thead><tr><th scope="col" class="mx-wlcol">protocol</th>' +
      pts.map(function (p) { return headerCell(state, p); }).join('') + '</tr>';
    if (pts.some(function (p) { return p.chip.verdict === 'fail'; })) {
      h += '<tr class="mx-chipcap"><th scope="row" class="mx-wlcol"><span class="muted">chip check</span></th>' + pts.map(function (p) {
        return '<td class="mx-col' + (p.col === state.focus.col ? ' mx-focus-col' : '') + (p.chip.verdict === 'fail' ? ' chip-fail' : '') + '">' +
          (p.chip.verdict === 'fail' ? '<span class="mx-capt">chip check fails (' + esc(chipItemShort(p, p.chip.driver)) +
            '): the verdicts below assume the chip could be built</span>' : '') + '</td>';
      }).join('') + '</tr>';
    }
    h += '</thead><tbody>';
    if (!shown.length) h += '<tr><td colspan="' + ncols + '" class="muted">No protocol differs across the points in this tag.</td></tr>';
    groupRows(state, shown, o.group).forEach(function (g) {
      if (g.title) {
        h += '<tr class="mx-group' + (g.sub ? ' sub' : '') + '" data-gh="' + esc(g.key) + '"><th colspan="' + ncols + '" scope="colgroup">' +
          (g.head ? U.verdictBadge(g.verdict) + ' <span class="num">(' + g.count + ')</span> <span class="muted">on ' + esc(state.point.label) + '</span>'
            : esc(g.title) + ' <span class="num">(' + g.ids.length + ')</span>') + '</th></tr>';
      }
      g.ids.forEach(function (id) {
        var wl = workloadOf(data, id), d = pts.length > 1 ? differs(pts, id) : { verdict: false, binding: false };
        h += '<tr data-row="' + esc(id) + '" data-g="' + esc(g.key) + '" data-search="' + esc((id + ' ' + wl.name).toLowerCase()) + '">' +
          '<th scope="row" class="mx-wl mx-wlcol"><button type="button" class="mx-wlbtn"' + U.focusAttr(id, null) + '>' + esc(wl.name) + '</button>' +
          '<span class="muted mono">' + esc(id) + '</span>' +
          (d.verdict ? ' <span class="mx-diff" title="the verdict differs across the points">verdict differs</span>' :
            d.binding ? ' <span class="mx-diff soft" title="same verdict, different binding limit">binding differs</span>' : '') +
          '<label class="mx-scn"><input type="checkbox" data-mix="' + esc(id) + '"' + (state.scenario.indexOf(id) >= 0 ? ' checked' : '') +
          ' aria-label="add ' + esc(id) + ' to the scenario"> scenario</label></th>';
        pts.forEach(function (p) {
          var cell = p.all.cells[id];
          h += cell ? cellHtml(state, cell, wl, p) : '<td></td>';
        });
        h += '</tr>';
      });
    });
    return h + '</tbody></table></div>';
  }

  // ------------------------------------------------------------ scenario
  function mixVerdict(v) {
    return '<span class="mx-mixv ' + esc(v) + '"><span class="ic" aria-hidden="true">' + (v === 'served' ? '&#10003;' : v === 'marginal' ? '~' : '&#10007;') +
      '</span> ' + (v === 'served' ? 'pass' : v === 'marginal' ? 'marginal' : 'fail') + '</span>';
  }
  function needText(r) {
    if (r.need && typeof r.need === 'object') return fmtBandHtml(r.need, r.unit);
    return '<span class="num">' + esc(fmtNum(r.need)) + ' ' + esc(r.unit) + '</span>';
  }
  // The link figures live in Throughput (single home): the scenario table shows need and margin, "available" links there.
  function availText(r) {
    if (r.id.indexOf('bw_') === 0) return '<button type="button" class="mx-link" data-tab="throughput">see Throughput</button>';
    return fmtBandHtml(r.avail, r.unit);
  }
  function scenarioPicker(state) {
    var pool = Object.keys(state.point.all.cells);
    state.scenario.forEach(function (id) { if (pool.indexOf(id) < 0) pool.push(id); });
    var sameSet = function (m) {
      return m.members.length === state.scenario.length && m.members.every(function (id) { return state.scenario.indexOf(id) >= 0; });
    };
    var chips = (state.data.mixes || []).length ? '<p class="mx-mixchips" role="group" aria-label="reference mixes">Reference mixes: ' +
      state.data.mixes.map(function (m) {
        return '<button type="button" class="mx-chip" data-cap-mix="' + esc(m.id) + '" aria-pressed="' + sameSet(m) + '" title="' + esc(m.description + ' (' + m.members.join(', ') + ')') + '">' + esc(m.label) + '</button>';
      }).join(' ') + '</p>' : '';
    return '<details class="mx-pick" data-key="scenario picker"' + (state.scenario.length ? '' : ' open') + '><summary>Choose protocols (' +
      state.scenario.length + ' selected)</summary>' + chips + '<div class="mx-pickgrid">' + pool.map(function (id) {
      return '<label><input type="checkbox" data-mix="' + esc(id) + '"' + (state.scenario.indexOf(id) >= 0 ? ' checked' : '') + '> ' +
        esc(U.wlName(state.data, id)) + ' <span class="muted mono">' + esc(id) + '</span></label>';
    }).join('') + '</div><p><button type="button" data-cap-scn-focus>add the focused protocol</button> ' +
      '<button type="button" data-cap-scn-clear>clear</button></p></details>';
  }
  // ------------------------------------------------------------ reference mixes, pairs, max concurrent (ArchSim.mix / pairs / maxConcurrent)
  var pairCache = {}, pairCacheN = 0;
  function pairsFor(state, pt) {
    var key = JSON.stringify(pt.arch) + '|' + state.tag;
    if (!pairCache[key]) {
      if (pairCacheN > 24) { pairCache = {}; pairCacheN = 0; }
      pairCache[key] = A.pairs(pt.arch, state.data, { tag: state.tag }); pairCacheN++;
    }
    return pairCache[key];
  }
  var MIX_GLYPH = { served: '&#10003;', marginal: '~', impossible: '&#10007;' };
  function rowShort(rows) {
    return rows.filter(function (r) { return r.verdict !== 'served'; }).map(function (r) { return r.label; });
  }
  // The verdict of one reference mix on one point: the shared-resource check plus members that cannot run even alone.
  function refMixResult(state, mx, pt) {
    var m = A.mix(pt.arch, mx.members, state.data, { derived: pt.derived });
    var alone = m.members.filter(function (b) { return b.verdict === 'impossible'; }).map(function (b) { return b.wl; });
    var bad = rowShort(m.rows), verdict = alone.length ? 'impossible' : m.verdict;
    var mc = A.maxConcurrent(pt.arch, state.data, mx.members);
    return { verdict: verdict, shared: m.verdict, bad: bad, alone: alone, max: mc.n, size: mx.members.length };
  }
  function referenceMixes(state) {
    var mixes = state.data.mixes || [], pts = state.points;
    if (!mixes.length) return '';
    var h = '<h3>Reference mixes on each point</h3><p class="muted">Typical products that run several protocols together. A mix passes when the ' +
      'shared-resource check passes and every member is at least possible alone. Click a result to tick that mix and look at its point.</p>';
    h += '<div class="tbl"><table class="mx-table mx-refmix"><thead><tr><th scope="col" class="mx-wlcol">reference mix</th>' + pts.map(function (p) {
      return '<th scope="col" class="mx-colhead mx-col"><span class="mx-colname-plain">' + esc(p.col === 0 ? 'live: ' + p.label : p.label) + '</span></th>';
    }).join('') + '</tr></thead><tbody>';
    mixes.forEach(function (mx) {
      h += '<tr data-mix-row="' + esc(mx.id) + '"><th scope="row" class="mx-wl mx-wlcol"><b>' + esc(mx.label) + '</b><span class="muted mono">' + esc(mx.members.join(' + ')) + '</span></th>';
      pts.forEach(function (p) {
        var r = refMixResult(state, mx, p);
        var why = r.alone.length ? 'impossible alone: ' + r.alone.join(', ') + (r.bad.length ? '; ' : '') : '';
        why += r.bad.join(', ');
        h += '<td class="mx-cell v-' + esc(r.verdict) + ' mx-col"><button type="button" class="mx-cellbtn" data-cap-mix="' + esc(mx.id) + '"' + U.focusAttr(null, p.col) +
          ' data-verdict="' + esc(r.verdict) + '" title="' + esc(mx.label + ' on ' + p.label + ': ' + (why || 'all shared resources pass')) + '">' + mixVerdict(r.verdict) +
          '<span class="mx-head">' + (why ? esc(why) : 'every shared resource fits') + '</span>' +
          '<span class="mx-max">' + r.max + ' of ' + r.size + ' can run together</span></button></td>';
      });
      h += '</tr>';
    });
    return h + '</tbody></table></div>';
  }

  function pairMatrix(state) {
    var pt = state.point, r = pairsFor(state, pt), ids = r.ids, t = r.tally;
    var h = '<details class="mx-pairs" data-key="pair matrix" open><summary>Pairwise compatibility on <b>' + esc(pt.label) + '</b></summary>';
    if (!ids.length) return h + '<p class="muted">No protocol of tag <b>' + esc(state.tag) + '</b> is served alone on this point.</p></details>';
    var idx = {};
    r.pairs.forEach(function (p) { idx[p.a + '|' + p.b] = p; });
    h += '<p><b data-pair-tally>' + t.served + ' of ' + t.total + ' pairs run together</b> <span class="muted">(' + t.marginal + ' marginal, ' + t.impossible +
      ' fail; ' + ids.length + ' protocols of tag ' + esc(state.tag) + ' that are served alone, each also with a second copy of itself; change the tag in Single protocols)</span></p>';
    h += '<div class="tbl"><table class="mx-pairtable"><thead><tr><td></td>' + ids.map(function (id) {
      return '<th scope="col" class="mx-pairhead"><span title="' + esc(U.wlName(state.data, id)) + '">' + esc(id) + '</span></th>';
    }).join('') + '</tr></thead><tbody>';
    ids.forEach(function (a, i) {
      h += '<tr><th scope="row" class="mx-pairrow" title="' + esc(U.wlName(state.data, a)) + '">' + esc(a) + '</th>';
      ids.forEach(function (b, j) {
        if (j < i) { h += '<td class="mx-pairblank"></td>'; return; }
        var p = idx[a + '|' + b], why = p.failing.length ? p.rows.filter(function (x) { return x.verdict !== 'served'; }).map(function (x) { return x.label; }).join(', ') : 'every shared resource fits';
        var bind = p.binding ? ' Binding: ' + (p.rows.filter(function (x) { return x.id === p.binding; })[0] || {}).label + '.' : '';
        h += '<td class="mx-pair v-' + esc(p.verdict) + '"><button type="button" class="mx-pairbtn" data-pair="' + esc(a + '|' + b) + '" data-verdict="' + esc(p.verdict) + '"' +
          U.sideAttr({ kind: 'pair', a: a, b: b, col: pt.col }) + ' aria-label="' + esc(a + ' with ' + b + ': ' + p.verdict) + '" title="' + esc(a + ' + ' + b + ' (' + p.verdict + '): ' + why + '.' + bind) + '">' +
          MIX_GLYPH[p.verdict] + '</button></td>';
      });
      h += '</tr>';
    });
    h += '</tbody></table></div>';
    if (r.skipped.length) {
      h += '<p class="muted">Left out (not served alone on this point): ' + r.skipped.map(function (s) { return '<code>' + esc(s.wl) + '</code>'; }).join(', ') + '.</p>';
    }
    return h + '<p class="muted">Cell = the shared-resource check for the two together (&#10003; pass, ~ marginal, &#10007; fail). Hover for the failing rows, click for the side panel. ' +
      'It checks engines, pins, program memory, link and host loop only; what it leaves out is in Trust, "Not modelled".</p></details>';
  }

  function maxConcurrentLine(state) {
    var ids = state.scenario;
    if (!ids.length) return '';
    var a = A.maxConcurrent(state.point.arch, state.data, ids), b = A.maxConcurrent(state.point.arch, state.data, ids, { marginal: true });
    var note = (a.exact && b.exact) ? '' : ' <span class="muted">(search capped at ' + a.cap + ' checks: a lower bound)</span>';
    return '<p data-max-concurrent><b>At most ' + a.n + ' of the ' + ids.length + ' ticked protocols can run together</b> on this point' +
      (b.n !== a.n ? ' (' + b.n + ' if marginal results count)' : '') + '.' + note +
      (a.alone_excluded.length ? ' <span class="muted">Not counted (not served alone): ' + a.alone_excluded.map(function (i) { return '<code>' + esc(i) + '</code>'; }).join(', ') + '.</span>' : '') + '</p>';
  }

  function renderScenario(state) {
    var h = '<p class="muted">Which protocols can run <b>at the same time</b> on the ' + (state.focus.col === 0 ? 'live' : 'pinned') + ' point <b>' +
      esc(state.point.label) + '</b>: shared engines, pins, tri-state pins, IMEM, host-link bandwidth and time share. Each member\'s own ' +
      'single-protocol verdict is listed separately from the shared-resource check.</p>';
    h += referenceMixes(state) + scenarioPicker(state);
    if (!state.scenario.length) return h + '<p class="muted">Tick two or more protocols above (or the scenario box on a matrix row) to check them together.</p>' + pairMatrix(state);
    var m = A.mix(state.point.arch, state.scenario, state.data), alone = [];
    h += '<h3>Members</h3><div class="tbl">' + U.table(['protocol', 'alone on this point', 'binding limit', 'program fits one engine', 'needs the host loop', ''],
      m.members.map(function (b) {
        if (b.verdict === 'impossible') alone.push(b.wl);
        var c = state.point.all.cells[b.wl];
        return [esc(U.wlName(state.data, b.wl)) + ' <code>' + esc(b.wl) + '</code>', U.verdictBadge(b.verdict),
          esc(c ? c.headline.short : b.binding), b.program_fits ? 'yes' : '<b>no</b>', b.host_loop ? 'yes' : 'no',
          '<button type="button"' + U.focusAttr(b.wl, null) + U.sideAttr({ kind: 'cell', wl: b.wl, col: state.focus.col }) + '>derivation</button>'];
      }), 'mx-members') + '</div>';
    h += '<h3>Shared resources</h3><div class="tbl">' + U.table(['resource', 'need', 'available', 'margin', 'result'], m.rows.map(function (r) {
      return [esc(r.label), needText(r), availText(r), fmtBandHtml(r.margin, r.unit), mixVerdict(r.verdict)];
    }), 'mx-mixtable') + '</div>';
    var bad = m.rows.filter(function (r) { return r.verdict !== 'served'; }).map(function (r) { return r.label; });
    h += '<p>Shared-resource check: ' + mixVerdict(m.verdict) + (bad.length ? ' <span class="muted">(' + esc(bad.join(', ')) + ')</span>' : '') +
      (m.exempt.length ? ' <span class="muted">(rate-elastic protocols exempt from bandwidth: ' + esc(m.exempt.join(', ')) + ')</span>' : '') + '</p>';
    if (alone.length) {
      h += '<p class="muted">Members that are impossible even alone on this point (a single-protocol failure, independent of sharing): ' +
        alone.map(function (i) { return '<code>' + esc(i) + '</code>'; }).join(', ') + '.</p>';
    }
    h += maxConcurrentLine(state);
    return h + '<p class="muted">Chip check of this point (reference only, not folded into the result): ' + U.chipBadge(m.chip.verdict) + ' ' +
      esc(chipItemShort(state.point, m.chip.driver)) + '.</p>' + pairMatrix(state);
  }

  U.renderCapability = function (state) {
    var o = opts(state);
    return '<div class="v-capability' + (o.all ? ' show-all' : '') + '">' + toolbar(state, o) +
      (o.sub === 'scenario' ? renderScenario(state) : renderSingle(state, o) + howToRead()) + '</div>';
  };

  // ------------------------------------------------------------ side panel: kind `cell`
  function notModelledFor(state, cell, wl) {
    var ids = cell.limits.map(function (l) { return l.id; });
    return state.data.not_modelled.items.filter(function (n) {
      return !!n.always || (n.limits || []).some(function (id) { return ids.indexOf(id) >= 0; }) ||
        (n.tristate && wl.pins_tristate > 0) || (n.multi_engine && wl.engines > 1) || (n.needs_xor && wl.needs_xor) ||
        (n.workload_match && wl.id.indexOf(n.workload_match) >= 0);
    }).map(function (n) { return n.text; });
  }
  function flagOr(r, f) {
    return '<span class="num">' + esc(r.unit === 'flag' ? fmtBand(r[f], 'flag') : fmtNum(r[f])) + '</span>';
  }
  function rankedTable(cell, blockedBy) {
    var rows = cell.ranked.map(function (r, i) {
      var rec = limRec(cell, r.id), scale = rec ? rec.scale : 0;
      return ['<span class="num">' + (i + 1) + '</span>',
        esc(r.short) + ' <code>' + esc(r.id) + '</code>' + (i === 0 ? ' <span class="mx-tag">closest</span>' : '') +
        (r.id === cell.binding ? ' <span class="mx-tag">binding</span>' : '') +
        (blockedBy.indexOf(r.id) >= 0 ? ' <span class="mx-blocks">blocks</span>' : ''),
        flagOr(r, 'nom'), flagOr(r, 'pess'), flagOr(r, 'opt'), esc(r.unit),
        '<span class="num">' + esc(fmtNum(r.rel, 2)) + '</span> ' + U.pessOptBar(r.pess, r.nom, r.opt, scale, r.short + ' range')];
    });
    return '<div class="tbl">' + U.table(['#', 'limit', 'nom', 'pess', 'opt', 'unit', 'relative margin'], rows, 'mx-ranked') + '</div>';
  }
  function limitTree(lim, rank, ctx, open, blocked) {
    var rel = typeof lim.rel === 'number' ? ' <span class="muted num">rel ' + esc(fmtNum(lim.rel, 2)) + '</span>' : '';
    ctx.seen = [lim.id]; ctx.srcSeen = {};
    return '<details class="mx-lim"' + (open ? ' open' : '') + ' data-key="limit ' + esc(lim.id) + '"><summary><span class="num">#' + rank + '</span> <b>' +
      esc(lim.label || lim.id) + '</b> <code>' + esc(lim.id) + '</code> ' + fmtBandHtml(lim.value, lim.unit) + rel +
      (blocked ? ' <span class="mx-blocks">blocks</span>' : '') + '</summary>' + U.derivTree(lim, ctx, 0) + '</details>';
  }
  function sideCell(state, ref) {
    var col = ref.col || 0, pt = state.points[col] || state.points[0], wl = workloadOf(state.data, ref.wl);
    if (!wl) return { title: String(ref.wl), html: '<p class="muted">No protocol <code>' + esc(ref.wl) + '</code> in the data.</p>' };
    var cell = pt.all.cells[wl.id] || A.evaluateCell(pt.arch, wl.id, state.data, { derived: pt.derived });
    var blockedBy = cell.blocked_by || [], band = { nom: cell.headline.nom, pess: cell.headline.pess, opt: cell.headline.opt };
    var h = '<div class="cap-side"><p><span class="muted mono">' + esc(wl.id) + '</span> on ' + esc(pt.label) + '</p>';
    h += '<p>' + U.verdictBadge(cell.verdict) + (U.cellIsa(cell) ? ' ' + U.estimateMark('this verdict rests on ISA lever estimates (judgement)') : '') + ' binding limit <b>' + esc(cell.headline.short) + '</b> <code>' + esc(cell.binding) + '</code>, margin ' +
      fmtBandHtml(band, cell.headline.unit) + '</p>';
    if (pt.chip.verdict !== 'pass') {
      h += '<p class="muted">The chip check of this point is <button type="button" class="mx-link"' + U.sideAttr({ kind: 'chip', item: pt.chip.driver, col: col }) + '>' +
        esc(pt.chip.verdict) + ' (' + esc(chipItemShort(pt, pt.chip.driver)) + ')</button>; this verdict does not include it.</p>';
    }
    h += '<p class="muted">Bands are ranges (bounds or judgement), not confidence intervals: pess and opt are the vertex values of the parameter bands.</p>';
    if (blockedBy.length) {
      h += '<p><b>Blocked by</b> (negative even at the optimistic end): ' + blockedBy.map(function (id) {
        var r = limRec(cell, id);
        return '<code>' + esc(id) + '</code>' + (r ? ' (' + esc(r.short) + ')' : '');
      }).join(', ') + '</p>';
    } else h += '<p class="muted">Blocked by: nothing (no limit fails at its optimistic end).</p>';
    h += '<h3>Distance to every limit (smallest relative margin first)</h3>' + rankedTable(cell, blockedBy);
    h += '<h3>Not applicable to this cell</h3>';
    if (cell.na && cell.na.length) {
      h += '<div class="tbl">' + U.table(['limit', 'why not'], cell.na.map(function (n) {
        return [esc(n.short || n.id) + ' <code>' + esc(n.id) + '</code>', esc(n.reason)];
      }), 'mx-na') + '</div>';
    } else h += '<p class="muted">none</p>';
    var ctx = U.derivCtx(state, col, cell);
    h += '<h3>Derivation tree (each number down to its parameter, method and source)</h3><div class="mx-tree">';
    cell.ranked.forEach(function (r, i) {
      var lim = limRec(cell, r.id);
      if (lim) h += limitTree(lim, i + 1, ctx, i === 0, blockedBy.indexOf(r.id) >= 0);
    });
    h += '</div><div class="mx-notmodelled"><h3>Not modelled (caveats for this cell)</h3><ul>' + notModelledFor(state, cell, wl).map(function (t) {
      return '<li>' + esc(t) + '</li>';
    }).join('') + '</ul><p class="muted">The full list is in <button type="button" class="mx-link" data-tab="trust">Trust</button>.</p></div></div>';
    return { title: wl.name || wl.id, html: h };
  }
  U.registerSide('cell', sideCell);

  // ------------------------------------------------------------ side panel: kind `pair`
  function sidePair(state, ref) {
    var col = ref.col || 0, pt = state.points[col] || state.points[0], d = state.data;
    if (!workloadOf(d, ref.a) || !workloadOf(d, ref.b)) return { title: 'pair', html: '<p class="muted">Unknown protocol in this pair.</p>' };
    var m = A.mix(pt.arch, [ref.a, ref.b], d, { derived: pt.derived });
    var h = '<div class="cap-side"><p><code>' + esc(ref.a) + '</code> + <code>' + esc(ref.b) + '</code>' + (ref.a === ref.b ? ' (two instances)' : '') + ' on ' + esc(pt.label) +
      ': shared-resource check ' + mixVerdict(m.verdict) + '</p>';
    var bad = m.rows.filter(function (r) { return r.verdict !== 'served'; });
    if (bad.length) {
      h += '<div class="tbl">' + U.table(['failing row', 'need', 'margin', 'result'], bad.map(function (r) {
        return [esc(r.label) + ' <code>' + esc(r.id) + '</code>', needText(r), fmtBandHtml(r.margin, r.unit), mixVerdict(r.verdict)];
      }), 'mx-pairrows') + '</div>';
    } else h += '<p class="muted">Every shared resource fits.</p>';
    h += '<p class="muted">To look at the two together with their members table, tick both in the scenario picker.</p>';
    h += '<p class="muted">Not checked here: host MCU CPU time, host-port arbitration latency between engines, engines needing different core clocks, ' +
      'timing interference on shared pins (see Trust, "Not modelled").</p></div>';
    return { title: ref.a + ' + ' + ref.b, html: h };
  }
  U.registerSide('pair', sidePair);

  // ------------------------------------------------------------ DOM (browser only)
  function applyFilter(el, q) {
    var rows = el.querySelectorAll('[data-row]'), s = String(q || '').toLowerCase(), vis = {}, i;
    for (i = 0; i < rows.length; i++) {
      var hide = s !== '' && rows[i].getAttribute('data-search').indexOf(s) < 0;
      rows[i].hidden = hide;
      if (!hide) vis[rows[i].getAttribute('data-g')] = true;
    }
    var gh = el.querySelectorAll('[data-gh]');
    for (i = 0; i < gh.length; i++) {
      var k = gh[i].getAttribute('data-gh');
      gh[i].hidden = s !== '' && !Object.keys(vis).some(function (g) { return g === k || g.indexOf(k + '|') === 0; });
    }
  }
  function mount(el, state) {
    if (!el || typeof el.addEventListener !== 'function') return;
    el.__capState = state;
    var q = el.__capQuery || '', box = el.querySelector ? el.querySelector('[data-mx-search]') : null;
    if (box) { box.value = q; applyFilter(el, q); }
    if (el.__capWired) return;
    el.__capWired = true;
    function inCap(t) { return !!(t.closest && t.closest('.v-capability')); }
    el.addEventListener('click', function (e) {
      var t = e.target, b;
      if (!t.closest || !inCap(t)) return;
      var s = el.__capState, o = opts(s);
      if ((b = t.closest('[data-cap-tag]'))) U.setState({ tag: b.getAttribute('data-cap-tag') });
      else if ((b = t.closest('[data-cap-sub]'))) U.setView('capability', { sub: b.getAttribute('data-cap-sub') });
      else if (t.closest('[data-cap-all]')) U.setView('capability', { all: !o.all });
      else if (t.closest('[data-cap-scn-focus]')) {
        if (s.scenario.indexOf(s.focus.wl) < 0) U.setScenario(s.scenario.concat([s.focus.wl]));
      } else if (t.closest('[data-cap-scn-clear]')) U.setScenario([]);
      else if ((b = t.closest('[data-cap-mix]'))) {
        var mx = ((s.data.mixes || []).filter(function (m) { return m.id === b.getAttribute('data-cap-mix'); }))[0];
        if (mx) U.setScenario(mx.members.filter(function (id, i, a) { return a.indexOf(id) === i; }));
      }
    });
    el.addEventListener('change', function (e) {
      var t = e.target;
      if (!t.hasAttribute || !inCap(t)) return;
      var s = el.__capState;
      if (t.hasAttribute('data-mix')) {
        var list = s.scenario.slice(), id = t.getAttribute('data-mix'), i = list.indexOf(id);
        if (t.checked && i < 0) list.push(id); else if (!t.checked && i >= 0) list.splice(i, 1);
        U.setScenario(list);
      } else if (t.hasAttribute('data-cap-group')) U.setView('capability', { group: t.value });
      else if (t.hasAttribute('data-cap-diff')) U.setView('capability', { diff: t.checked });
    });
    el.addEventListener('input', function (e) {
      if (e.target.hasAttribute && e.target.hasAttribute('data-mx-search')) { el.__capQuery = e.target.value; applyFilter(el, e.target.value); }
    });
  }

  U.register('capability', { title: 'Capability', render: function (s) { return U.renderCapability(s); }, mount: mount });
})(ArchSimUI);
