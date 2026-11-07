/* Area & pins tab: die-area waterfall against the usable-area band and the pin budget strip, for the FOCUSED point
 * (state.point; a pinned point when focus.col > 0). Pure render functions (HTML/SVG strings, QuickJS-safe).
 * Owns: the waterfall (one row per area line + total, pess..opt whisker, util_max*core band, core line), the pin strip
 * (24 cells, allocation, demand of the focused protocol or of the scenario), side kind `area_line`.
 * Does NOT own: the chip verdict / die margin (Configure chip card; linked via the `chip` side kind), block boxes (`block`).
 * Test hooks: data-area-total/-pess/-opt, data-area (per line), data-area-limit, data-core-area, data-pin-use, data-tristate,
 * data-pin-bank, data-need-pins, data-need-tristate, data-free-pins.
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum, methodBadge = U.methodBadge;

  function n1(x) { return (Math.round(x * 1000) / 1000).toString(); }
  function niceStep(span, target) {
    var raw = span / target, p = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10)), m = raw / p;
    return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
  }
  function axisTicks(x0, x1, target) {
    var st = niceStep(x1 - x0, target || 6), out = [];
    for (var v = Math.ceil(x0 / st) * st; v <= x1 + st * 1e-9; v += st) out.push(Math.round(v / st) * st);
    return out;
  }
  function areaOf(l) { return l.area_placed ? { v: l.area_placed, placed: true } : { v: l.area_cell || l.area, placed: false }; }
  function kcls(n) { return fmtNum(n / 1000, 0) + 'k'; }
  function bandText(v, d) {
    var lo = fmtNum(Math.min(v.pess, v.opt), d), hi = fmtNum(Math.max(v.pess, v.opt), d);
    return lo === hi ? '' : lo + '..' + hi;
  }

  // ------------------------------------------------------------ die-area waterfall
  function areaSection(state) {
    var pt = state.point, bd = A.blockDiagram(pt.arch, state.data), col = pt.col, lines = bd.area_lines, W = 760, L = 230, R = 90;
    var total = bd.area_total, core = bd.core_area, lim = bd.area_limit;
    var xmax = Math.max(core, total.pess, lim.pess, lim.opt) * 1.03;
    var sx = function (v) { return L + (W - L - R) * v / xmax; };
    var top = 22, rowH = 20, H = top + (lines.length + 1) * rowH + 40, s = '', cum = 0, anyPre = false;
    axisTicks(0, xmax, 6).forEach(function (t) {
      s += '<line class="gridline" x1="' + sx(t) + '" x2="' + sx(t) + '" y1="' + (top - 4) + '" y2="' + (H - 32) + '"/>' +
        '<text x="' + sx(t) + '" y="' + (H - 18) + '" text-anchor="middle">' + kcls(t) + '</text>';
    });
    s += '<text x="' + sx(xmax / 2) + '" y="' + (H - 4) + '" text-anchor="middle">area (um2)</text>';
    var bl = Math.min(lim.pess, lim.opt), bh = Math.max(lim.pess, lim.opt);
    s += '<rect class="band" x="' + sx(bl) + '" y="' + (top - 4) + '" width="' + (sx(bh) - sx(bl)) + '" height="' + (H - 32 - top + 4) + '"/>';
    lines.forEach(function (l, i) {
      var a = areaOf(l), y = top + i * rowH;
      if (!a.placed) anyPre = true;
      s += '<g class="arow" tabindex="0"' + U.sideAttr({ kind: 'area_line', id: l.id, col: col }) + ' data-area-id="' + esc(l.id) + '" data-area="' + n1(a.v.nom) + '">' +
        '<title>' + esc(l.label) + ': ' + fmtNum(a.v.nom, 0) + ' um2 (' + fmtNum(l.pct_core, 2) + ' % of core), basis ' + esc(l.basis) + ' x ' + esc(l.factor) + (a.placed ? '' : ', pre-P&R') + '</title>' +
        '<text x="' + (L - 6) + '" y="' + (y + 13) + '" text-anchor="end">' + esc(l.label.length > 34 ? l.label.slice(0, 33) + '..' : l.label) + (l.count > 1 ? ' x' + l.count : '') + '</text>' +
        '<rect class="' + (l.basis === 'macro' ? 'macro' : 'synth') + '" x="' + sx(cum) + '" y="' + (y + 2) + '" width="' + Math.max(1, sx(cum + a.v.nom) - sx(cum)) + '" height="14"/>' +
        '<text x="' + (sx(cum + a.v.nom) + 4) + '" y="' + (y + 13) + '">' + fmtNum(a.v.nom, 0) + '</text></g>';
      cum += a.v.nom;
    });
    var ty = top + lines.length * rowH;
    s += '<g class="arow total" tabindex="0"' + U.sideAttr({ kind: 'record', id: 'area_total', col: col }) + '><text x="' + (L - 6) + '" y="' + (ty + 13) + '" text-anchor="end" font-weight="700">total</text>' +
      '<rect class="tot" x="' + sx(0) + '" y="' + (ty + 2) + '" width="' + (sx(total.nom) - sx(0)) + '" height="14"/>' +
      '<line class="whisk" x1="' + sx(total.opt) + '" x2="' + sx(total.pess) + '" y1="' + (ty + 9) + '" y2="' + (ty + 9) + '"/>' +
      '<text x="' + (sx(Math.max(total.pess, total.nom)) + 4) + '" y="' + (ty + 13) + '">' + fmtNum(total.nom, 0) + '</text></g>';
    s += '<g class="limline" data-area-limit="' + n1(lim.nom) + '"><line x1="' + sx(lim.nom) + '" x2="' + sx(lim.nom) + '" y1="' + (top - 6) + '" y2="' + (H - 32) + '"/>' +
      '<text x="' + sx(lim.nom) + '" y="' + (top - 8) + '" text-anchor="middle">usable ' + kcls(lim.nom) + '</text></g>';
    s += '<g class="coreline" data-core-area="' + n1(core) + '"><line x1="' + sx(core) + '" x2="' + sx(core) + '" y1="' + (top - 6) + '" y2="' + (H - 32) + '"/>' +
      '<text x="' + (sx(core) - 2) + '" y="' + (H - 21) + '" text-anchor="end">core ' + kcls(core) + '</text></g>';
    var isaOn = U.isaActive(pt.arch).length > 0, isaB = U.isaBlockIds(state.data, pt.arch), isaLines = lines.filter(function (l) { return isaB[l.id]; });
    var band = bandText(total, 0), ubands = bandText(bd.util_pct, 2);
    return '<section class="ap-sec" data-area-sec data-area-total="' + n1(total.nom) + '" data-area-pess="' + n1(total.pess) + '" data-area-opt="' + n1(total.opt) + '">' +
      '<h2>Die area</h2>' + U.svgWrap(W, H, s, 'Die area waterfall against core area and usable area') +
      '<p class="ap-sum"><span class="num">' + fmtNum(total.nom, 0) + '</span> um2 ' + methodBadge('synth') + (band ? ' <span class="lo num">' + esc(band) + '</span>' : '') +
      ' = <span class="num">' + fmtNum(bd.util_pct.nom, 2) + ' %</span>' + (ubands ? ' <span class="lo num">' + esc(ubands) + '</span>' : '') +
      ' of the <span class="num">' + fmtNum(core, 0) + '</span> um2 core; usable (util_max x core) <span class="num">' + fmtNum(lim.nom, 0) + '</span> um2' +
      (bandText(lim, 0) ? ' <span class="lo num">' + esc(bandText(lim, 0)) + '</span>' : '') + '. ' +
      '<button type="button" class="lnk"' + U.sideAttr({ kind: 'chip', item: 'die_area', col: col }) + '>die_area check (margin and verdict)</button></p>' +
      (isaOn ? '<p class="ap-isa" data-area-isa>ISA levers on' + (isaLines.length ? ' (' + isaLines.map(function (l) { return esc(l.label); }).join(', ') + ' in the lines above)' : '') + ': their area and fmax effects are ' + U.estimateMark('ISA lever blocks are judgement: no RTL exists') + '</p>' : '') +
      '<p class="legend"><span class="lg k-synth"><i></i>synth x P&amp;R factor</span> <span class="lg k-macro"><i></i>macro x halo</span> ' +
      '<span class="lg k-band"><i></i>pess..opt usable area</span>' + (anyPre ? ' <span class="muted">some lines pre-P&amp;R</span>' : '') + '</p></section>';
  }

  // ------------------------------------------------------------ pin strip
  var PIN_LABEL = { host: 'H', reserved: 'R', lane_clk: 'L', protocol: 'P', free: '-' };
  var PIN_TEXT = { host: 'host link', reserved: 'reserved', lane_clk: 'lane clock', protocol: 'protocol demand', free: 'unused' };
  // demand of the focus: the scenario (summed through ArchSim.mix) when one is set, else the focused protocol.
  function demandOf(state) {
    if (state.scenario && state.scenario.length) {
      var m = A.mix(state.point.arch, state.scenario, state.data);
      return { what: 'scenario of ' + state.scenario.length, pins: m.need.pins, tri: m.need.pins_tristate };
    }
    var wl = U.workloadOf(state.data, state.focus.wl);
    return wl ? { what: wl.name || wl.id, pins: wl.pins || 0, tri: wl.pins_tristate || 0, wl: wl.id } : null;
  }
  function pinSection(state) {
    var pt = state.point, bd = A.blockDiagram(pt.arch, state.data), ps = bd.pins_summary, dem = demandOf(state);
    var pins = bd.pins.map(function (p) { return Object.assign({}, p); }), needT = dem ? dem.tri : 0, left = dem ? dem.pins : 0, i;
    // illustrative assignment: tri-state needs first on free uio pins, then free pins in bank order
    for (i = 0; i < pins.length && needT > 0; i++) if (pins[i].use === 'free' && pins[i].tristate) { pins[i].use = 'protocol'; needT--; left--; }
    for (i = 0; i < pins.length && left > 0; i++) if (pins[i].use === 'free') { pins[i].use = 'protocol'; left--; }
    var unplaced = Math.max(0, left) + Math.max(0, needT), cnt = {};
    pins.forEach(function (p) { cnt[p.use] = (cnt[p.use] || 0) + 1; });
    var h = '<section class="ap-sec" data-pin-sec data-free-pins="' + ps.total_avail + '"' + (dem ? ' data-need-pins="' + dem.pins + '" data-need-tristate="' + dem.tri + '"' : '') + '>' +
      '<h2>Pins</h2><div class="pins">';
    ['ui_in', 'uo_out', 'uio'].forEach(function (b) {
      h += '<div class="pbank"><span class="pname mono">' + b + '</span>';
      pins.filter(function (p) { return p.bank === b; }).forEach(function (p) {
        h += '<span class="pin u-' + esc(p.use) + (p.tristate ? ' tri' : '') + '" tabindex="0" data-pin="' + p.pin + '" data-pin-bank="' + b +
          '" data-pin-use="' + esc(p.use) + '" data-tristate="' + (p.tristate ? 1 : 0) + '" title="pin ' + p.pin + ' ' + b + '[' + p.index + '] ' + esc(PIN_TEXT[p.use] || p.use) + (p.tristate ? ', tri-state' : '') + '">' +
          '<b>' + (PIN_LABEL[p.use] || '?') + '</b><small>' + p.index + (p.tristate ? ' T' : '') + '</small></span>';
      });
      h += '</div>';
    });
    h += '</div><div class="legend">' + ['host', 'reserved', 'lane_clk', 'protocol', 'free'].map(function (u) {
      return '<span class="pin u-' + u + ' sm"><b>' + PIN_LABEL[u] + '</b></span> ' + esc(PIN_TEXT[u]) + ' (' + (cnt[u] || 0) + ')';
    }).join(' &nbsp; ') + ' &nbsp; <span class="pin tri sm"><small>T</small></span> tri-state (uio)</div>';
    h += '<p>' + (dem ? esc(dem.what) + ' needs <span class="num">' + dem.pins + '</span> pins (<span class="num">' + dem.tri + '</span> tri-state); ' : '') +
      'free <span class="num">' + ps.total_avail + '</span> (<span class="num">' + ps.tristate_avail + '</span> tri-state) ' + methodBadge('formula') +
      (unplaced ? ' <span class="ap-short" data-unplaced="' + unplaced + '">' + unplaced + ' not placed</span>' : '') +
      ' <button type="button" class="lnk"' + U.sideAttr({ kind: 'chip', item: 'pins_budget', col: pt.col }) + '>pins_budget check</button>' +
      (dem && dem.wl ? ' <button type="button" class="lnk"' + U.focusAttr(dem.wl, pt.col) + ' data-tab="capability">cell verdict</button>' : '') + '</p>' +
      '<p class="muted">Host link pins ' + ps.link_in + ' in + ' + ps.link_out + ' out, reserved ' + ps.reserved +
      '. The protocol assignment is illustrative: the model counts pins, it does not route them.</p></section>';
    return h;
  }

  // ------------------------------------------------------------ side kind area_line
  function sideAreaLine(state, ref) {
    var col = ref.col || 0, pt = state.points[col] || state.points[0], bd = A.blockDiagram(pt.arch, state.data);
    var l = bd.area_lines.filter(function (x) { return x.id === ref.id; })[0];
    if (!l) return { title: ref.id, html: '<p class="muted">No area line <code>' + esc(ref.id) + '</code> on this point.</p>' };
    var a = areaOf(l), r = U.resolveRef({ ref: { kind: 'block', id: l.id } }, U.derivCtx(state, col));
    var h = '<div class="mx-rec"><div><b>' + esc(l.label) + '</b> ' + methodBadge(l.basis) + (r.status ? ' ' + U.statusBadge(r.status) : '') + '</div>' +
      '<div>' + (a.placed ? 'placed' : 'pre-P&amp;R') + ' area ' + U.fmtBandHtml(a.v, 'um2', 0) + ' (' + fmtNum(l.pct_core, 2) + ' % of core)</div>' +
      '<div class="mono mx-formula">area = count x raw x ' + esc(l.factor) + '</div>' +
      '<div>raw per line ' + U.fmtBandHtml(l.raw, 'um2', 0) + ', count ' + l.count + '</div>' +
      (l.note ? '<div class="muted">' + esc(l.note) + '</div>' : '') +
      (r.method ? '<div class="muted">block <code>' + esc(l.id) + '</code></div>' : '') +
      (r.sources && r.sources.length ? '<ul class="mx-src">' + r.sources.map(function (x) { return '<li><code>' + esc(x) + '</code></li>'; }).join('') + '</ul>' : '') +
      (r.assumptions && r.assumptions.length ? '<ul class="mx-assume">' + r.assumptions.map(function (x) { return '<li>' + esc(x) + '</li>'; }).join('') + '</ul>' : '') +
      '<div><button type="button" class="lnk"' + U.sideAttr({ kind: 'param', id: l.factor }) + '>' + esc(l.factor) + '</button> ' +
      '<button type="button" class="lnk"' + U.sideAttr({ kind: 'record', id: 'area_total', col: col }) + '>total area derivation</button></div></div>';
    return { title: l.label, html: h };
  }
  U.registerSide('area_line', sideAreaLine);

  U.renderArea = function (state) {
    return '<div class="v-area">' + areaSection(state) + pinSection(state) + '</div>';
  };
  U.register('area', { title: 'Area & pins', render: function (s) { return U.renderArea(s); } });
})(ArchSimUI);
