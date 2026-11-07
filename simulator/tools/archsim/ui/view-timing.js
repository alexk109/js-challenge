/* Timing tab (v2): everything about time for the focused protocol on the focused point, in one place.
 *   0. Cross-point strip (hook data-timing-strip): the focused protocol's reflex/response margin and binding timing limit per point.
 *   1. Reflex / response budget: ONE chart (v1 latency waterfall merged with the v1 cycle-grid timeline): segments on a time
 *      axis with core-clock ticks, the deadline, pess/opt whiskers and, when the lane serves, a second row (lane path).
 *      Test hooks: data-seg-ns (only on this chart's sampled-path segments), data-lane-seg-ns, data-deadline, data-total-nom,
 *      data-tick.
 *   2. Edge placement on the TX grid (ArchSim.edgeGrid).      hooks: data-edge-ideal / data-edge-placed / data-edge-err.
 *   3. Receive sampling (ArchSim.rxSampling).                 hooks: data-rx-part, data-rx-margin.
 *   4. Core clock vs fmax: the parts of the critical path D against the period T (ArchSim.clockBudget). Shows NO margin
 *      number: verdict and margin belong to the chip card (link to side kind 'chip', item core_clock). hooks: data-clk-ns.
 *   5. Measured example (golden-ISS waveform of an mvp_fw row).
 * Sections that do not apply to the focused protocol show one line saying why. Side kind 'segment' (this file) shows the
 * derivation behind a chart segment, the deadline or a clock part. Pure render functions; no DOM work is needed (the core
 * handles data-side), so there is no mount.
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum, fmtBand = U.fmtBand, fmtBandHtml = U.fmtBandHtml, methodBadge = U.methodBadge;

  function n1(x) { return (Math.round(x * 1000) / 1000).toString(); }          // numeric attribute value
  function kindClass(k) { return 'k-' + esc(k || 'pad'); }
  function niceStep(span, target) {
    var raw = span / target, p = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10)), m = raw / p;
    return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
  }
  function axisTicks(x0, x1, target) {
    if (!(x1 > x0)) return [x0];
    var st = niceStep(x1 - x0, target || 6), out = [];
    for (var v = Math.ceil(x0 / st) * st; v <= x1 + st * 1e-9; v += st) out.push(Math.round(v / st) * st);
    return out;
  }
  function paramOf(state, ref) { return ref && ref.kind === 'param' && state.data.params ? (state.data.params[ref.id] || null) : null; }
  function na(title, reason, sec) {
    return '<section class="tm-sec tm-na" data-sec="' + sec + '"><h2>' + esc(title) + '</h2><p class="muted">' + esc(reason) + '</p></section>';
  }
  function head(title, sub) { return '<h2>' + esc(title) + (sub ? ' <span class="muted">' + sub + '</span>' : '') + '</h2>'; }

  // ------------------------------------------------------------ 0. cross-point strip: the focused protocol's timing on every point
  // The single home of the cross-point timing summary. It shows only margins, the binding timing limit and its verdict, per
  // point; the segments behind them are in the budget below (focused point only).
  function stripRow(state, pt) {
    var wl = state.focus.wl, cell = pt.all.cells[wl] || A.evaluateCell(pt.arch, wl, state.data, { derived: pt.derived });
    var tl = cell.limits.filter(function (l) { return l.timing; });
    var dl = tl.filter(function (l) { return l.id === 'reflex'; })[0] || tl.filter(function (l) { return l.id === 'response'; })[0];
    var blocking = tl.filter(function (l) { return l.verdict === 'impossible'; }), pool = blocking.length ? blocking : tl;
    var bind = pool.slice().sort(function (a, b) { return a.rel - b.rel; })[0];
    var cur = pt.col === state.focus.col;
    var name = '<button type="button" class="tm-pt"' + U.focusAttr(null, pt.col) + ' data-strip-point="' + pt.col + '"' + (pt.id ? ' data-preset-id="' + esc(pt.id) + '"' : '') +
      (cur ? ' aria-current="true"' : '') + ' title="show this point in the tabs below">' + esc(pt.col === 0 ? 'live: ' + pt.label : pt.label) + '</button>';
    if (!bind) return [name, '<span class="muted">no timing limit applies</span>', '', ''];
    return [name,
      dl ? esc(dl.short) + ' ' + fmtBandHtml(dl.value, dl.unit, 1) : '<span class="muted">no reflex or response deadline</span>',
      esc(bind.short) + ' ' + fmtBandHtml(bind.value, bind.unit, 1),
      U.verdictBadge(bind.verdict)];
  }
  function stripSection(state) {
    var name = U.wlName(state.data, state.focus.wl);
    return '<section class="tm-sec tm-strip" data-sec="strip" data-timing-strip>' + head('Timing across points', 'for ' + esc(name)) +
      '<div class="tbl">' + U.table(['point', 'reflex / response margin', 'binding timing limit', 'verdict'], state.points.map(function (p) { return stripRow(state, p); }), 'tm-striptbl') + '</div>' +
      '<p class="muted small">Margins are nominal (pessimistic..optimistic). Pick a point to see its segments below.</p></section>';
  }

  // ------------------------------------------------------------ 1. reflex / response budget (one merged chart)
  function chartRows(tl) {
    var rows = [{ name: tl.lane ? 'sampled path' : tl.kind + ' path', segs: tl.segments, total: tl.total, attr: 'lat', path: 'lat' }];
    if (tl.lane) rows.push({ name: 'lane path (serves)', segs: tl.lane.segments, total: tl.lane.total, attr: 'lane', path: 'lane' });
    return rows;
  }
  function budgetChart(tl, col) {
    var rows = chartRows(tl), W = 760, L = 120, R = 24, xmax = tl.deadline;
    rows.forEach(function (rw) {
      xmax = Math.max(xmax, rw.total.pess, rw.total.opt, rw.total.nom, rw.segs[rw.segs.length - 1].t1);
    });
    xmax *= 1.06;
    var sx = function (t) { return L + (W - L - R) * t / xmax; };
    var top = 56, rowH = 62, H = top + rows.length * rowH + 36, s = '', T = tl.T;
    // core-clock grid: tick lines through every row and a square wave on top
    if (tl.ticks.length <= 300) {
      tl.ticks.forEach(function (t) {
        s += '<line class="clk" x1="' + sx(t) + '" x2="' + sx(t) + '" y1="34" y2="' + (H - 34) + '" data-tick="' + n1(t) + '"/>';
      });
    }
    axisTicks(0, xmax, 7).forEach(function (t) {
      s += '<text x="' + sx(t) + '" y="' + (H - 18) + '" text-anchor="middle">' + fmtNum(t, 0) + '</text>';
    });
    s += '<line class="axis" x1="' + L + '" x2="' + (W - R) + '" y1="' + (H - 32) + '" y2="' + (H - 32) + '"/>';
    s += '<text x="' + sx(xmax / 2) + '" y="' + (H - 4) + '" text-anchor="middle">time since the input edge (ns); thin lines = core clock ticks, T = ' + fmtNum(T, 2) + ' ns</text>';
    s += '<text x="' + (L - 8) + '" y="46" text-anchor="end">core clock</text>';
    var wave = 'M' + sx(0) + ' 48', hi = false;
    if (xmax / T < 600) {
      for (var t = 0; t <= xmax; t += T / 2) { wave += ' H' + sx(t) + ' V' + (hi ? 48 : 38); hi = !hi; }
      s += '<path class="wave" d="' + wave + '"/>';
    }
    rows.forEach(function (rw, ri) {
      var y = top + ri * rowH;
      s += '<text x="' + (L - 8) + '" y="' + (y + 17) + '" text-anchor="end">' + esc(rw.name) + '</text>';
      rw.segs.forEach(function (g, i) {
        var x0 = sx(g.t0), w = Math.max(1, sx(g.t1) - sx(g.t0));
        var lab = w > 78 ? g.label : (w > 30 ? fmtNum(g.dur, 2).replace(/\.?0+$/, '') : '');
        s += '<g class="seg ' + kindClass(g.kind) + '" tabindex="0" role="button"' +
          U.sideAttr({ kind: 'segment', path: rw.path, i: i, col: col }) + ' ' +
          (rw.attr === 'lat' ? 'data-seg-ns="' : 'data-lane-seg-ns="') + n1(g.dur) + '" data-seg="' + esc(g.label) + '">' +
          '<title>' + esc(g.label) + ': ' + fmtNum(g.dur, 2) + ' ns (t = ' + fmtNum(g.t0, 2) + ' to ' + fmtNum(g.t1, 2) + ')</title>' +
          '<rect x="' + x0 + '" y="' + y + '" width="' + w + '" height="26"/>' +
          (lab ? '<text class="in" x="' + (x0 + w / 2) + '" y="' + (y + 17) + '" text-anchor="middle">' + esc(lab.length > 14 && w < 110 ? lab.slice(0, 13) + '.' : lab) + '</text>' : '') + '</g>';
      });
      var wy = y + 38, lo = Math.min(rw.total.opt, rw.total.pess), hi2 = Math.max(rw.total.opt, rw.total.pess);
      s += '<g class="whisk" data-total-nom="' + n1(rw.total.nom) + '" data-total-pess="' + n1(rw.total.pess) + '" data-total-opt="' + n1(rw.total.opt) + '">' +
        '<line x1="' + sx(lo) + '" x2="' + sx(hi2) + '" y1="' + wy + '" y2="' + wy + '"/>' +
        '<line x1="' + sx(lo) + '" x2="' + sx(lo) + '" y1="' + (wy - 4) + '" y2="' + (wy + 4) + '"/>' +
        '<line x1="' + sx(hi2) + '" x2="' + sx(hi2) + '" y1="' + (wy - 4) + '" y2="' + (wy + 4) + '"/>' +
        '<line class="nomtick" x1="' + sx(rw.total.nom) + '" x2="' + sx(rw.total.nom) + '" y1="' + (wy - 6) + '" y2="' + (wy + 6) + '"/>' +
        '<text x="' + L + '" y="' + (wy + 18) + '">total ' + fmtNum(rw.total.nom, 1) + ' ns (opt ' + fmtNum(rw.total.opt, 1) + ', pess ' + fmtNum(rw.total.pess, 1) + ')</text></g>';
    });
    s += '<g class="deadline" data-deadline="' + n1(tl.deadline) + '"><line x1="' + sx(tl.deadline) + '" x2="' + sx(tl.deadline) + '" y1="26" y2="' + (H - 32) + '"/>' +
      '<text x="' + sx(tl.deadline) + '" y="20" text-anchor="middle">deadline ' + fmtNum(tl.deadline, 1) + ' ns</text></g>';
    return U.svgWrap(W, H, s, 'Latency budget against the deadline on the core clock grid');
  }
  function budgetSection(state) {
    var wl = state.focus.wl, col = state.focus.col, tl = A.timeline(state.point.arch, wl, state.data), TITLE = 'Reflex / response budget';
    if (tl.kind === 'none') return na(TITLE, tl.reason, 'budget');
    var kinds = {}, legend = '';
    tl.segments.concat(tl.lane ? tl.lane.segments : []).forEach(function (g) {
      if (!kinds[g.kind]) { kinds[g.kind] = 1; legend += '<span class="lg ' + kindClass(g.kind) + '"><i></i>' + esc(g.kind) + '</span>'; }
    });
    var act = tl.lane ? tl.lane : tl;
    var segRow = function (path) {
      return function (g, i) {
        var p = paramOf(state, g.ref);
        return ['<button type="button" class="lnk"' + U.sideAttr({ kind: 'segment', path: path, i: i, col: col }) + '>' + esc(g.label) + '</button>',
          '<span class="num">' + fmtNum(g.t0, 2) + '</span>', '<span class="num">' + fmtNum(g.dur, 3) + '</span>',
          methodBadge(p ? p.method : g.ref.kind), '<span class="mono">' + esc(g.ref.id || g.ref.field || '') + '</span>'];
      };
    };
    var HDR = ['segment', 't0 (ns)', 'dur (ns)', 'method', 'source'];
    var tbl = tl.segments.map(segRow('lat'));
    tbl.push(['<b>total (nominal)</b>', '', '<span class="num">' + fmtNum(tl.total.nom, 3) + '</span>', '', fmtBandHtml(tl.total, 'ns', 1)]);
    var laneTbl = '';
    if (tl.lane) {
      var lrows = tl.lane.segments.map(segRow('lane'));
      lrows.push(['<b>lane total (nominal)</b>', '', '<span class="num">' + fmtNum(tl.lane.total.nom, 3) + '</span>', '', fmtBandHtml(tl.lane.total, 'ns', 1)]);
      laneTbl = '<h3>Lane path</h3>' + U.table(HDR, lrows);
    }
    var note = tl.kind === 'response' && tl.on_chip === false
      ? '<p class="muted">The response buffer does not fit on chip, so the host decision loop is part of the path.</p>' : '';
    return '<section class="tm-sec" data-sec="budget" data-latency data-lat-kind="' + esc(tl.kind) + '" data-active="' + esc(tl.active) + '">' +
      head(TITLE, '(' + esc(tl.kind) + ')') + '<div class="tm-chart">' + budgetChart(tl, col) + '</div>' +
      '<div class="legend">' + legend + '</div>' +
      '<p>margin to the deadline: ' + fmtBandHtml(act.margin, 'ns', 1) + ' ' + methodBadge('formula') +
      (tl.lane ? ' <span class="muted">(the lane path serves; sampled-path margin ' + esc(fmtBand(tl.margin, 'ns', 1)) + ')</span>' : '') +
      ' <button type="button" class="lnk"' + U.sideAttr({ kind: 'segment', path: 'deadline', col: col }) + '>how the deadline is derived</button></p>' + note +
      '<details class="tm-tbl" data-key="timing-seg-table"><summary>Segment table</summary><div class="tbl">' +
      U.table(HDR, tbl) + laneTbl + '</div></details></section>';
  }

  // ------------------------------------------------------------ 2. edge placement
  function edgeSection(state) {
    var TITLE = 'Edge placement on the TX grid', eg = A.edgeGrid(state.point.arch, state.focus.wl, state.data, 8);
    if (!eg.applicable) return na(TITLE, eg.reason, 'edge');
    var W = 760, L = 80, R = 20, tmax = eg.ideal[eg.ideal.length - 1] + eg.tick_ns * 1.2, H = 150;
    var sx = function (t) { return L + (W - L - R) * t / tmax; }, s = '';
    eg.ticks.forEach(function (t) { s += '<line class="clk" x1="' + sx(t) + '" x2="' + sx(t) + '" y1="30" y2="100"/>'; });
    s += '<text x="' + (L - 8) + '" y="44" text-anchor="end">ideal</text><text x="' + (L - 8) + '" y="84" text-anchor="end">placed</text>';
    eg.ideal.forEach(function (t, k) {
      var p = eg.placed[k], e = eg.error[k];
      s += '<g class="edge" data-edge="' + k + '" data-edge-ideal="' + n1(t) + '" data-edge-placed="' + n1(p) + '" data-edge-err="' + n1(e) + '">' +
        '<title>edge ' + k + ': ideal ' + fmtNum(t, 2) + ' ns, placed ' + fmtNum(p, 2) + ' ns, error ' + fmtNum(e, 2) + ' ns (tolerance +-' + fmtNum(eg.tol_ns, 2) + ')</title>' +
        '<rect class="tol" x="' + sx(t - eg.tol_ns) + '" y="30" width="' + (sx(t + eg.tol_ns) - sx(t - eg.tol_ns)) + '" height="64"/>' +
        '<line class="ideal" x1="' + sx(t) + '" x2="' + sx(t) + '" y1="34" y2="54"/><circle class="ideal" cx="' + sx(t) + '" cy="44" r="4"/>' +
        '<rect class="placed" x="' + (sx(p) - 4) + '" y="76" width="8" height="8"/>' +
        '<line class="err' + (Math.abs(e) > eg.tol_ns ? ' bad' : '') + '" x1="' + sx(t) + '" x2="' + sx(p) + '" y1="64" y2="64"/>' +
        '<text x="' + sx(t) + '" y="114" text-anchor="middle">' + (e === 0 ? '0' : (e > 0 ? '+' : '') + fmtNum(e, 1)) + '</text></g>';
    });
    s += '<text x="' + (L - 8) + '" y="114" text-anchor="end">error ns</text>';
    axisTicks(0, tmax, 8).forEach(function (q) { s += '<text x="' + sx(q) + '" y="' + (H - 18) + '" text-anchor="middle">' + fmtNum(q, 0) + '</text>'; });
    s += '<text x="' + sx(tmax / 2) + '" y="' + (H - 4) + '" text-anchor="middle">ns (grid = TX tick ' + fmtNum(eg.tick_ns, 2) + ' ns; band = +-' + fmtNum(eg.tol_ns, 2) + ' ns tolerance)</text>';
    return '<section class="tm-sec" data-sec="edge" data-edge-sec data-integer-grid="' + (eg.integer_grid ? 1 : 0) + '" data-edge-maxerr="' + n1(eg.max_error_ns) + '">' +
      head(TITLE) + '<div class="tm-chart">' + U.svgWrap(W, H, s, 'Ideal against placed edges') + '</div><p>edge ' + fmtNum(eg.edge_ns, 2) + ' ns on a ' +
      fmtNum(eg.tick_ns, 2) + ' ns grid: ' + (eg.integer_grid ? 'integer multiple, every placed edge equals its ideal' : 'not an integer multiple, max error ' + fmtNum(eg.max_error_ns, 2) + ' ns') +
      (eg.limit ? '; tx_edge margin ' + fmtBandHtml(eg.limit, 'ns', 1) : '') + '</p></section>';
  }

  // ------------------------------------------------------------ 3. receive sampling
  function rxSection(state) {
    var TITLE = 'Receive sampling', arch = state.point.arch, r = A.rxSampling(arch, state.focus.wl, state.data);
    if (!r.applicable) return na(TITLE, r.reason, 'rx');
    var uLabel = arch.fine_edge === 'dtc' ? 'resolution u (delay-line tap)' : (r.capture === 'dpll' ? 'resolution u (T_rx / 2, DPLL)' : 'resolution u (T_rx)');
    var parts = [['u', uLabel, r.u.nom], ['drift', 'rate drift over the run', r.drift.nom], ['jitter', 'input jitter', r.jitter]];
    var total = parts.reduce(function (t, p) { return t + p[2]; }, 0), W = 760, L = 120, R = 30, xmax = Math.max(r.edge_half, total) * 1.12;
    var sx = function (t) { return L + (W - L - R) * t / xmax; }, s = '', x = 0, H = 120;
    axisTicks(0, xmax, 7).forEach(function (t) { s += '<line class="gridline" x1="' + sx(t) + '" x2="' + sx(t) + '" y1="22" y2="76"/><text x="' + sx(t) + '" y="92" text-anchor="middle">' + fmtNum(t, 0) + '</text>'; });
    s += '<text x="' + sx(xmax / 2) + '" y="112" text-anchor="middle">ns of the half-edge interval that the sampler can lose</text>';
    s += '<text x="' + (L - 8) + '" y="49" text-anchor="end">consumed</text>';
    parts.forEach(function (p) {
      var w = Math.max(p[2] > 0 ? 1 : 0, sx(x + p[2]) - sx(x));
      s += '<g class="seg rx-' + p[0] + '" data-rx-part="' + p[0] + '" data-rx-ns="' + n1(p[2]) + '"><title>' + esc(p[1]) + ': ' + fmtNum(p[2], 3) + ' ns</title>' +
        '<rect x="' + sx(x) + '" y="32" width="' + w + '" height="26"/>' +
        (w > 40 ? '<text class="in" x="' + (sx(x) + w / 2) + '" y="49" text-anchor="middle">' + esc(p[0]) + ' ' + fmtNum(p[2], 2) + '</text>' : '') + '</g>';
      x += p[2];
    });
    s += '<g class="deadline" data-rx-edge-half="' + n1(r.edge_half) + '"><line x1="' + sx(r.edge_half) + '" x2="' + sx(r.edge_half) + '" y1="22" y2="76"/>' +
      '<text x="' + sx(r.edge_half) + '" y="16" text-anchor="middle">edge / 2 = ' + fmtNum(r.edge_half, 2) + ' ns</text></g>';
    var legend = parts.map(function (p) { return '<span class="lg rx-' + p[0] + '"><i></i>' + esc(p[1]) + ' ' + fmtNum(p[2], 3) + ' ns</span>'; }).join('');
    var osr = r.osr && r.osr.need != null && r.osr.have
      ? '<p data-rx-osr>oversampling: ' + fmtBandHtml(r.osr.have, 'samples/bit', 2) + ' against ' + fmtNum(r.osr.need, 2) + ' needed (edge / T_rx).</p>'
      : '<p class="muted">Oversampling: no requirement, or the bus-clocked lane serves this protocol.</p>';
    return '<section class="tm-sec" data-sec="rx" data-rx-margin="' + n1(r.margin.nom) + '">' + head(TITLE, '(capture: ' + esc(r.capture) + ')') +
      '<div class="tm-chart">' + U.svgWrap(W, H, s, 'Receive eye budget: resolution, drift and jitter against half an edge interval') + '</div><div class="legend">' + legend + '</div>' +
      '<p>eye margin = edge / 2 - u - drift - jitter = ' + fmtBandHtml(r.margin, 'ns', 2) + ' ' + methodBadge('formula') + '</p>' + osr + '</section>';
  }

  // ------------------------------------------------------------ 4. core clock vs fmax
  function clockSection(state) {
    var TITLE = 'Core clock vs fmax', cb = A.clockBudget(state.point.arch, state.data), col = state.focus.col;
    var W = 760, L = 120, H = 120, R = 30, T = cb.T, xmax = Math.max(T, cb.D.pess, cb.D.nom) * 1.12;
    var sx = function (t) { return L + (W - L - R) * t / xmax; }, s = '', x = 0;
    axisTicks(0, xmax, 7).forEach(function (t) { s += '<line class="gridline" x1="' + sx(t) + '" x2="' + sx(t) + '" y1="22" y2="76"/><text x="' + sx(t) + '" y="92" text-anchor="middle">' + fmtNum(t, 0) + '</text>'; });
    s += '<text x="' + sx(xmax / 2) + '" y="112" text-anchor="middle">ns</text><text x="' + (L - 8) + '" y="49" text-anchor="end">critical path D</text>';
    cb.parts.forEach(function (p, i) {
      var w = sx(x + p.ns.nom) - sx(x);
      if (p.ns.nom > 0) {
        s += '<g class="seg clk-' + esc(p.id) + '" data-clk-part="' + esc(p.id) + '" data-clk-ns="' + n1(p.ns.nom) + '" tabindex="0" role="button"' +
          U.sideAttr({ kind: 'segment', path: 'clock', i: i, col: col }) + '><title>' + esc(p.label) + ': ' + fmtNum(p.ns.nom, 3) + ' ns</title>' +
          '<rect x="' + sx(x) + '" y="32" width="' + Math.max(1, w) + '" height="26"/>' +
          (w > 60 ? '<text class="in" x="' + (sx(x) + w / 2) + '" y="49" text-anchor="middle">' + esc(p.id) + ' ' + fmtNum(p.ns.nom, 2) + '</text>' : '') + '</g>';
      }
      x += p.ns.nom;
    });
    var lo = Math.min(cb.D.pess, cb.D.opt), hi = Math.max(cb.D.pess, cb.D.opt);
    if (hi > lo) s += '<g class="whisk"><line x1="' + sx(lo) + '" x2="' + sx(hi) + '" y1="68" y2="68"/><line x1="' + sx(lo) + '" x2="' + sx(lo) + '" y1="64" y2="72"/><line x1="' + sx(hi) + '" x2="' + sx(hi) + '" y1="64" y2="72"/></g>';
    s += '<g class="deadline" data-clk-T="' + n1(T) + '"><line x1="' + sx(T) + '" x2="' + sx(T) + '" y1="22" y2="76"/><text x="' + sx(T) + '" y="16" text-anchor="middle">period T = ' + fmtNum(T, 2) + ' ns (' + fmtNum(cb.f_core_mhz, 1) + ' MHz)</text></g>';
    var rows = cb.parts.map(function (p, i) {
      return [p.ref ? '<button type="button" class="lnk"' + U.sideAttr({ kind: 'segment', path: 'clock', i: i, col: col }) + '>' + esc(p.label) + '</button>' : esc(p.label),
        fmtBandHtml(p.ns, 'ns', 3), p.ref ? '<span class="mono">' + esc(p.ref.id) + '</span>' : '<span class="muted">none on this point</span>'];
    });
    rows.push(['<b>critical path D</b>', fmtBandHtml(cb.D, 'ns', 3), '']);
    return '<section class="tm-sec" data-sec="clock" data-clk-D="' + n1(cb.D.nom) + '">' + head(TITLE, '(' + esc(cb.corner) + ' corner)') +
      '<div class="tm-chart">' + U.svgWrap(W, H, s, 'Critical path parts against the core clock period') + '</div>' +
      '<p>The core clock period T must hold the critical path D. D is ' + fmtBandHtml(cb.D, 'ns', 2) + ' at the sign-off corner. ' +
      'The verdict and margin of this check are the chip card item <button type="button" class="lnk"' + U.sideAttr({ kind: 'chip', item: 'core_clock', col: col }) +
      '>core_clock</button>.</p><div class="tbl">' + U.table(['part', 'ns', 'source'], rows) + '</div></section>';
  }

  // ------------------------------------------------------------ 5. measured example
  function figureSection(state) {
    var TITLE = 'Measured example', wl = U.workloadOf(state.data, state.focus.wl);
    if (!wl || (wl.tags || []).indexOf('mvp_fw') < 0 || !state.data.figures) return na(TITLE, 'no measured ISS waveform for this protocol (only the mvp_fw rows have one)', 'figure');
    var key = wl.id.replace(/^fw_/, ''), figs = state.data.figures;
    while (key && !figs[key]) { var j = key.lastIndexOf('_'); key = j > 0 ? key.slice(0, j) : ''; }
    if (!key) return na(TITLE, 'no measured ISS waveform for this protocol', 'figure');
    var svg = String(figs[key]).replace(/^<\?xml[^>]*\?>\s*/, '');
    return '<section class="tm-sec" data-sec="figure">' + head(TITLE) + '<div class="fig" data-figure="' + esc(key) + '">' + svg + '</div>' +
      '<p class="muted">measured on the golden ISS (tools/waves)</p></section>';
  }

  // ------------------------------------------------------------ side kind 'segment'
  function sideSegment(state, ref) {
    var col = ref.col || 0, p = state.points[col] || state.points[0], wl = ref.wl || state.focus.wl, data = state.data;
    var cell = A.evaluateCell(p.arch, wl, data, { derived: p.derived }), ctx = U.derivCtx(state, col, cell);
    var tl = A.timeline(p.arch, wl, data);
    if (ref.path === 'deadline') {
      var rec = cell.records.reflex_deadline_ns || cell.records.response_deadline_ns;
      return { title: 'Deadline', html: rec ? '<div class="mx-tree">' + U.derivTree(rec, ctx, 0) + '</div>' : '<p class="muted">This protocol has no deadline.</p>' };
    }
    if (ref.path === 'clock') {
      var cb = A.clockBudget(p.arch, data), part = cb.parts[ref.i];
      if (!part) return { title: 'Clock part', html: '' };
      var hc = '<p>' + esc(part.label) + ': ' + fmtBandHtml(part.ns, 'ns', 3) + ' at the ' + esc(cb.corner) + ' corner.</p>';
      if (part.ref) hc += '<p><button type="button" class="lnk"' + U.sideAttr({ kind: 'param', id: part.ref.id }) + '>parameter <span class="mono">' + esc(part.ref.id) + '</span></button></p>';
      hc += '<p class="muted">Critical path D = base + IMEM + die terms; the verdict and margin against T are the chip card item core_clock.</p>' +
        '<p><button type="button" class="lnk"' + U.sideAttr({ kind: 'chip', item: 'core_clock', col: col }) + '>open chip item core_clock</button></p>';
      return { title: part.label, html: hc };
    }
    var segs = ref.path === 'lane' ? (tl.lane ? tl.lane.segments : []) : tl.segments, g = segs[ref.i];
    if (!g) return { title: 'Segment', html: '<p class="muted">No such segment on this point and protocol.</p>' };
    var prm = paramOf(state, g.ref), h = '<p>' + fmtNum(g.dur, 3) + ' ns, from ' + fmtNum(g.t0, 3) + ' to ' + fmtNum(g.t1, 3) + ' ns after the input edge. ' +
      methodBadge(prm ? prm.method : g.ref.kind) + '</p><p class="mono">source: ' + esc(g.ref.kind) + ' ' + esc(g.ref.id || g.ref.field || '') + '</p>';
    if (prm) {
      h += '<p>' + esc(prm.label) + ': ' + fmtBandHtml({ lo: prm.lo, nom: prm.nom, hi: prm.hi }, prm.unit) + ' ' + U.statusBadge(prm.status) +
        ' <button type="button" class="lnk"' + U.sideAttr({ kind: 'param', id: g.ref.id }) + '>sources and assumptions</button></p>';
    }
    var drec = p.derived.records[g.ref.id];
    if (drec) h += '<div class="mx-tree">' + U.derivTree(drec, ctx, 0) + '</div>';
    if (g.ref.kind === 'iss' && data.iss && data.iss[g.ref.id] !== undefined) {
      h += '<p>' + esc(g.ref.id) + ' = ' + esc(data.iss[g.ref.id]) + ' core cycles (golden ISS constant) of T = ' + fmtNum(tl.T, 3) + ' ns.</p>';
    }
    if (g.ref.kind === 'workload_field') {
      var w = U.workloadOf(data, wl);
      h += '<p>' + esc(g.ref.field) + ' = ' + esc(w && w[g.ref.field]) + ' core cycles of T = ' + fmtNum(tl.T, 3) + ' ns.</p>';
    }
    h += '<p class="muted">Core clock period T = ' + fmtNum(tl.T, 3) + ' ns; segments are in causal order.</p>';
    return { title: g.label, html: h };
  }
  U.registerSide('segment', sideSegment);

  // ------------------------------------------------------------ the view
  U.renderTiming = function (state) {
    var h = '<div class="v-timing" data-wl="' + esc(state.focus.wl) + '"><p class="muted tm-intro">Timing of <b>' + esc(U.wlName(state.data, state.focus.wl)) + '</b> on ' +
      esc(state.point.label) + '. The core clock against fmax is a chip-level check: its verdict is in the chip card.</p>';
    h += stripSection(state) + budgetSection(state) + edgeSection(state) + rxSection(state) + clockSection(state) + figureSection(state);
    return h + '</div>';
  };
  U.register('timing', { title: 'Timing', render: function (s) { return U.renderTiming(s); } });
})(ArchSimUI);
