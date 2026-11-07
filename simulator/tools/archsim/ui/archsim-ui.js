/* archsim UI core (v2): layout, shared selection, view registry, derivation side panel, shared helpers, DOM glue.
 *
 * FILES. build_html.py concatenates UI_FILES in a fixed order (tests/conftest.py loads the same list):
 *   archsim-ui.js (this), view-configure.js, view-capability.js, view-timing.js, view-area.js, view-throughput.js,
 *   view-explore.js, view-trust.js, boot.js.
 * CSS: archsim.css (tokens, layout areas, shared components) then ui/css/{configure,capability,timing,area,throughput,
 * explore,trust}.css. Colours only through tokens (var(--s1), var(--good), ...), never a hex/rgb literal in js or css
 * outside a token definition; status is never colour alone (icon + word). Scope your selectors under your own root class
 * (e.g. `.v-timing ...`) so views cannot collide.
 *
 * LAYOUT (template.html). #topbar, then #layout = #left (#chip + #rail) | #main (#focus, #tabs, #view) | #side.
 * #left is `display: contents` below 900 px so #chip becomes a sticky strip and #rail a full-screen sheet.
 *  - #rail   <- views.configure.render(state, {open}) -> HTML; mount(railEl, state) afterwards.
 *  - #chip   <- views.configure.renderChip(state) -> HTML. Convention: root `.chip-card` with `.chip-head` (one line:
 *               verdict + driver; give it data-open-rail so a tap opens the rail on phone) and `.chip-body` (the rows;
 *               the core CSS hides it below 900 px). ONE element, never a second copy for phone.
 *  - #focus  <- renderFocus(state) (core): protocol picker, point picker, the focused cell's verdict badge.
 *  - #tabs   <- renderTabs(state) (core);   #view <- views[state.tab].render(state), then mount(viewEl, state).
 *  - #side   <- renderSide(state) (core), dispatching on state.side.kind (see SIDE PANEL).
 *
 * REGISTRY. ArchSimUI.register(id, {title, render(state) -> HTML string, mount(rootEl, state) optional}).
 *  ids: the six tabs of TAB_ORDER = capability, timing, area, throughput, explore, trust, and 'configure' (the rail:
 *  render(state, ctx) with ctx = {open: {knob group -> bool}}, renderChip(state), mount). A view file also exposes
 *  ArchSimUI.renderCapability / renderTiming / renderArea / renderThroughput / renderExplore / renderTrust (= its render
 *  function, so tests call ArchSimUI.renderX(ArchSimUI.makeState({}, DATA))) and, for Configure, renderKnobPanel,
 *  renderChip. A view may add update(state): the core calls it after EVERY state change (Configure refreshes its live
 *  sections with it, since #rail is repainted only on rebuild). Register with a late-bound wrapper (render: function (s) { return U.renderX(s); }) so tests may override.
 *
 * PURE vs DOM. Render functions are PURE: state in, HTML string out (inline SVG allowed). No DOM, no globals except
 * ArchSim, ArchSimUI and (read-only) the DATA you get through state.data, so they run under QuickJS. DOM work only in
 * mount(), guarded by `typeof document !== 'undefined'`. The core wraps every paint in try/catch with a visible error
 * box, so one broken view never blanks the page. Every number you display comes from ArchSim or state.data.
 *
 * STATE (read-only for views). makeState(raw, data) returns
 *   raw fields:  arch (the LIVE point), preset, tag ('cap19' default), pinned [{arch, label}] (max 5),
 *                focus {wl, col} (col 0 = live, k = pinned[k-1]), scenario [wl ids], side null|{kind, ...},
 *                tab, view {<viewId>: {...free-form per-view state...}}
 *   computed:    points [{label, col, live, arch, derived, all, chip}]   (all = evaluateAll(point, tag) incl. .chip;
 *                derived = ArchSim.derive result; chip = ArchSim.chipCheck result), point = points[focus.col],
 *                cell = the focused cell (evaluated outside the tag when needed), data,
 *                and for convenience derived/all/chip of the LIVE point (= points[0]).
 *  Tabs show the FOCUSED point (state.point); Capability shows all points. The focus bar says "showing pinned point"
 *  when focus.col > 0. Knobs always edit the live point. Never recompute what points[k] already holds.
 *
 * CHANGING STATE (DOM only: mount() or declarative attributes). Declarative, handled by the core for any element anywhere:
 *   data-focus-wl="<wl>" / data-focus-col="<k>"    set the focus (both may be on one element)
 *   data-tab="<tab id>"                             switch tab
 *   data-side='<json>'                              open the side panel (use ArchSimUI.sideAttr(ref) to build it)
 *   data-side-close                                 close it
 *   data-action="reset|pin|load-preset|pin-preset|load-point|remove-point" (+ data-value="<preset id | point col>";
 *       for the preset actions the value falls back to the selected <option> of a [data-preset] select in the rail)
 *   data-quick="<knob id>" data-value="<n>"        set a numeric knob; data-open-rail / data-close-rail   phone sheet
 *   Enter/Space activate any of these on a non-native element; the core gives them tabindex=0 (and role=button) after every paint.
 *   [data-knob], [data-knob-range], [data-multi] controls inside #rail are wired by the core (slider input is rAF-coalesced).
 *   a <select data-preset data-autoload> loads its preset on change. A <select data-focus-pick="wl|col"> sets the focus.
 * Imperative (mount code): ArchSimUI.setState(patch[, rebuildRail]) with patch keys arch (merged), preset, tag, pinned,
 *   focus, scenario, side, tab, view; plus helpers setFocus(wl, col), setTab(t), setView(id, patch) (shallow merge),
 *   setScenario(list), openSide(ref), closeSide(), setArch(patch) (rAF-coalesced; use for sliders and text inputs),
 *   pinCurrent(), pinPreset(id), loadPreset(id), loadPoint(col), removePoint(col), resetToMvp().
 * Anything that changes the focus closes the side panel unless its ref has sticky:true (record, param, chip are sticky
 * because they carry their own point). The panel otherwise survives re-renders and tab switches.
 *
 * SIDE PANEL. ArchSimUI.registerSide(kind, render(state, ref) -> html | {title, html}, mount(bodyEl, state, ref)?).
 * ArchSimUI.renderSide(state) draws header (title, point label, close button) + the body. Core kinds:
 *   {kind:'record', id, col}   a derive() record of a point (e.g. 'T', 'link_write_MBps')
 *   {kind:'param', id}         an inputs.json parameter with sources and assumptions
 *   {kind:'chip', item, col}   a chip-check item's record with its full derivation tree
 * View-owned kinds: cell (capability), segment (timing), block (configure), area_line (area), calibration, isa_effect (trust),
 * impact, wwit, sweep_point (explore). Inside your body use derivTree(rec, derivCtx(state, col, cell)) so every derivation
 * looks the same. open from a click with `<tr ... data-side='...'>` or `ArchSimUI.stat(label, value, sub, ref)`.
 * Persistence: state.side, tab, focus, scenario, pinned, view are in location.hash (v2 encoding; v1 hashes still decode)
 * and in localStorage `archsim.v2` (try/catch, convenience only). Keep your own per-view state in state.view[id] via
 * setView so it survives re-renders and reloads; details open/closed state is preserved across repaints automatically
 * (give a <details> a data-key if its summary text is not unique).
 *
 * SHARED HELPERS (pure; all on ArchSimUI): esc, fmtNum, unitStr, fmtBand (text) / fmtBandHtml (markup) with degenerate bands
 * collapsed, methodBadge, statusBadge (title = vocabulary meaning; `estimate` and `judgement` get the warning border),
 * verdictBadge (served/marginal/impossible), chipBadge (pass/marginal/fail), stat, recordHtml, derivTree, derivCtx,
 * resolveRef, pessOptBar, table, svgWrap, niceTicks, tickFmt, eng, mark, workloadOf, tagsOf, sideAttr, focusAttr, wlName.
 * Wide tables go inside <div class="tbl"> (own horizontal scroll); never let content force page-level scroll.
 */
var ArchSimUI = (function () {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var VOCAB = null;   // data.vocab of the last makeState: statusBadge titles read the meaning from it

  // ------------------------------------------------------------ shared helpers (pure)
  function esc(s) {
    return String(s === undefined || s === null ? '' : s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }
  function group3(intStr) { return intStr.replace(/\B(?=(\d{3})+(?!\d))/g, ','); }
  // number -> string with `digits` decimals (default: 3 significant digits for |x|<100, else integer), thousands commas
  function fmtNum(x, digits) {
    if (typeof x === 'string') { if (x === 'Infinity') return 'inf'; if (x === '-Infinity') return '-inf'; return x; }
    if (typeof x !== 'number' || x !== x) return 'n/a';
    if (x === Infinity) return 'inf';
    if (x === -Infinity) return '-inf';
    var d = digits;
    if (d === undefined) { var a = Math.abs(x); d = a >= 100 || a === 0 ? 0 : a >= 10 ? 1 : a >= 1 ? 2 : 3; }
    var s = x.toFixed(d), neg = s.charAt(0) === '-';
    if (neg) s = s.slice(1);
    var p = s.split('.');
    return (neg ? '-' : '') + group3(p[0]) + (p[1] ? '.' + p[1] : '');
  }
  function unitStr(u) { return u ? ' ' + u : ''; }
  // Interval {nom,pess,opt}, parameter {lo,nom,hi}, or plain number -> text "nom unit (lo..hi)". Plain text, escaped by caller
  // via fmtBandHtml when markup is wanted. Band ends are sorted so the smaller comes first.
  function bandEnds(v) {
    if (v && typeof v === 'object') {
      if ('pess' in v) return [v.nom, v.pess, v.opt];
      if ('lo' in v) return [v.nom, v.lo, v.hi];
    }
    return null;
  }
  // flag-type (boolean) limits: the sign is the answer, so show words instead of +-1.
  function flagWord(x) { return typeof x === 'number' && x < 0 ? 'missing' : 'present'; }
  // [nom text, range text or ''] with degenerate bands (the ends print the same) collapsed to the single value.
  function bandParts(v, unit, digits) {
    var b = bandEnds(v);
    if (unit === 'flag') return [flagWord(b ? b[0] : v), ''];
    if (!b) return [fmtNum(v, digits), ''];
    var lo = fmtNum(Math.min(b[1], b[2]), digits), hi = fmtNum(Math.max(b[1], b[2]), digits);
    return [fmtNum(b[0], digits), lo === hi ? '' : lo + '..' + hi];
  }
  function fmtBand(v, unit, digits) {
    var p = bandParts(v, unit, digits);
    return p[0] + (unit === 'flag' ? '' : unitStr(unit)) + (p[1] ? ' (' + p[1] + ')' : '');
  }
  function fmtBandHtml(v, unit, digits) {
    var p = bandParts(v, unit, digits);
    var s = '<span class="num">' + esc(p[0] + (unit === 'flag' ? '' : unitStr(unit))) + '</span>';
    if (p[1]) s += ' <span class="lo num">' + esc(p[1]) + '</span>';
    return s;
  }
  function methodBadge(m) { return m ? '<span class="badge m-' + esc(m) + '" title="' + esc(methodTitle(m)) + '">' + esc(m) + '</span>' : ''; }
  function vocabMeaning(kind, id) {
    var l = VOCAB && VOCAB[kind] || [];
    for (var i = 0; i < l.length; i++) if (l[i].id === id) return l[i].meaning;
    return '';
  }
  function methodTitle(m) { return 'method' + (vocabMeaning('methods', m) ? ': ' + vocabMeaning('methods', m) : ''); }
  // The title carries the vocabulary meaning (data.vocab.statuses); estimate/judgement are drawn with the warning border in CSS.
  function statusBadge(s) { return s ? '<span class="badge st-' + esc(s) + '" title="' + esc('status' + (vocabMeaning('statuses', s) ? ': ' + vocabMeaning('statuses', s) : '')) + '">' + esc(s) + '</span>' : ''; }
  // ISA levers are estimates (judgement rows): wherever a shown result depends on one, it carries this mark.
  function isaActive(arch) { return A && arch ? A.isaActiveLevers(arch) : []; }
  function estimateMark(why) {
    return '<span class="est-mark" data-estimate title="' + esc(why || 'depends on ISA lever estimates (judgement)') + '">' + statusBadge('estimate') + '</span>';
  }
  // Does a cell's verdict rest on an ISA effect row? (any limit with an isa_effect term)
  function cellIsa(cell) {
    return !!cell && (cell.limits || []).some(function (l) { return (l.terms || []).some(function (t) { return t.ref && t.ref.kind === 'isa_effect'; }); });
  }
  // Area-line / block ids that exist only because an ISA lever is on (from inputs.json isa.levers).
  function isaBlockIds(data, arch) {
    var lev = (data.isa || {}).levers || {}, out = {};
    isaActive(arch).forEach(function (l) { ((lev[l] || {}).blocks || []).forEach(function (b) { out[b] = true; }); });
    return out;
  }
  var VERDICT_ICON = { served: '✓', marginal: '~', impossible: '✗' };
  function verdictBadge(v) {
    return '<span class="verdict ' + esc(v) + '" data-verdict="' + esc(v) + '"><span class="ic" aria-hidden="true">' +
      (VERDICT_ICON[v] || '?') + '</span> ' + esc(v) + '</span>';
  }
  // Small stat block; ref (a side-panel ref object, or a record id string = {kind:'record', id, col:0}) makes it a button that
  // opens that derivation in the side panel.
  function stat(label, value, sub, ref) {
    var inner = '<span class="k">' + esc(label) + '</span><span class="v num">' + value + '</span>' +
      (sub ? '<span class="s num">' + sub + '</span>' : '');
    return ref ? '<button type="button" class="stat"' + sideAttr(typeof ref === 'string' ? { kind: 'record', id: ref, col: 0 } : ref) + '>' + inner + '</button>'
      : '<div class="stat">' + inner + '</div>';
  }
  // Minimal derivation record view (label, value, formula, terms). Task 8 provides the full tree.
  function recordHtml(r) {
    if (!r) return '';
    var h = '<h3>' + esc(r.label || r.id) + ' ' + methodBadge(r.method) + '</h3><p class="num">' +
      fmtBandHtml(r.value, r.unit) + '</p>';
    if (r.formula) h += '<p class="mono">' + esc(r.formula) + '</p>';
    if (r.terms && r.terms.length) {
      h += '<table><tr><th>term</th><th class="r">value</th></tr>' + r.terms.map(function (t) {
        return '<tr><td class="mono">' + esc(t.sym) + '</td><td class="r">' + fmtBandHtml(t.value, t.unit) + '</td></tr>';
      }).join('') + '</table>';
    }
    if (r.note) h += '<p class="muted">' + esc(r.note) + '</p>';
    return h;
  }
  // rows: array of arrays of ready HTML strings; header: array of strings (escaped here).
  function table(header, rows, cls) {
    return '<table' + (cls ? ' class="' + esc(cls) + '"' : '') + '><thead><tr>' + header.map(function (h) {
      return '<th>' + esc(h) + '</th>';
    }).join('') + '</tr></thead><tbody>' + rows.map(function (r) {
      return '<tr>' + r.map(function (c) { return '<td>' + c + '</td>'; }).join('') + '</tr>';
    }).join('') + '</tbody></table>';
  }
  function svgWrap(w, h, inner, label) {
    return '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ' + w + ' ' + h + '" width="100%" role="img" aria-label="' +
      esc(label || '') + '" preserveAspectRatio="xMinYMin meet">' + inner + '</svg>';
  }

  // ------------------------------------------------------------ shared: data lookups, derivation tree, margin bar, chart ticks
  function workloadOf(data, id) {
    for (var i = 0; i < data.workloads.length; i++) if (data.workloads[i].id === id) return data.workloads[i];
    return null;
  }
  // Thin bar: the pess..opt range of a margin relative to its scale, around zero (centre tick).
  function barSvg(pess, nom, opt, scale, label) {
    var W = 84, H = 10, mid = W / 2;
    if (!(scale > 0)) return '';
    function px(v) { var r = v / scale; r = Math.max(-1, Math.min(1, r)); return mid + r * (mid - 2); }
    var a = px(Math.min(pess, opt)), b = px(Math.max(pess, opt)), n = px(nom);
    var inner = '<line class="mx-track" x1="1" y1="5" x2="' + (W - 1) + '" y2="5"/>' +
      '<rect class="mx-range" x="' + a.toFixed(1) + '" y="2" width="' + Math.max(2, b - a).toFixed(1) + '" height="6"/>' +
      '<line class="mx-nom" x1="' + n.toFixed(1) + '" y1="0" x2="' + n.toFixed(1) + '" y2="10"/>' +
      '<line class="mx-zero" x1="' + mid + '" y1="0" x2="' + mid + '" y2="10"/>';
    return '<svg class="mx-bar" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ' + W + ' ' + H + '" width="' + W + '" height="' + H +
      '" role="img" aria-label="' + esc(label) + '">' + inner + '</svg>';
  }

  // Derivation tree (PLAN 3.1): a record's terms down to parameter / block / link / ISS constant with method, status and sources.
  function srcList(list) {
    if (!list || !list.length) return '';
    return '<ul class="mx-src">' + list.map(function (s) { return '<li><code>' + esc(s) + '</code></li>'; }).join('') + '</ul>';
  }
  function assumptionList(list) {
    if (!list || !list.length) return '';
    return '<ul class="mx-assume">' + list.map(function (s) { return '<li>' + esc(s) + '</li>'; }).join('') + '</ul>';
  }
  function citeList(list) {
    return (list || []).map(function (c) { return (c.path || '') + (c.quote ? ': ' + c.quote : ''); });
  }
  // Resolve a term's ref against inputs.json / the cell records.
  function resolveRef(t, ctx) {
    var ref = t.ref || {}, data = ctx.data, r = { kind: ref.kind || '', id: ref.id || ref.field || '', sources: [], assumptions: [] };
    if (ref.kind === 'param') {
      var p = data.params[ref.id];
      if (p) {
        r.label = p.label; r.method = p.method; r.status = p.status;
        r.sources = (p.sources || []).concat(citeList(p.cite)); r.assumptions = p.assumptions || [];
      }
    } else if (ref.kind === 'iss') {
      r.label = 'golden ISS constant'; r.method = 'spec'; r.status = 'measured';
      var iss = data.iss;
      r.sources = (iss.sources || [iss.src]).filter(Boolean);
      if (ref.id === 'pins' && iss.pins && iss.pins.banks_src) r.sources.push(iss.pins.banks_src);
    } else if (ref.kind === 'workload_field') {
      var wl = workloadOf(data, ref.wl);
      r.id = (ref.wl || '') + '.' + (ref.field || ''); r.label = 'workload field'; r.method = ref.default ? 'derived' : 'spec';
      if (wl) {
        var s = wl.src && wl.src[ref.field];
        if (s) r.sources = [s]; else r.assumptions = ['model default for this field (PLAN 2.4)'];
      }
    } else if (ref.kind === 'knob') {
      var k = (A.KNOBS || []).filter(function (q) { return q.id === ref.id; })[0];
      r.label = k ? 'architecture knob: ' + k.label : 'architecture knob'; r.method = 'derived';
    } else if (ref.kind === 'block') {
      var b = data.blocks[ref.id];
      if (b) { r.label = b.label; r.method = b.method; r.status = b.status; r.sources = b.sources || []; r.assumptions = b.assumptions || []; }
    } else if (ref.kind === 'link') {
      var l = data.links[ref.id];
      if (l) { r.label = l.label; r.method = l.method; r.status = l.status; r.sources = l.sources || []; r.assumptions = l.assumptions || []; }
    } else if (ref.kind === 'sram_macro') {
      r.label = 'SRAM macro'; r.method = 'flow';
      (data.sram_macros || []).forEach(function (m) {
        if (m && (m.id === ref.id || m.name === ref.id)) r.sources = m.sources || (m.src ? [m.src] : []);
      });
    } else if (ref.kind === 'isa_effect') {
      var row = ((data.isa && data.isa.effects) || []).filter(function (e) {
        return e.wl === ref.wl && e.lever === ref.lever && (ref.value === undefined || e.value === ref.value);
      })[0];
      r.id = ref.lever + '@' + ref.wl; r.label = 'ISA effect estimate';
      if (row) {
        r.method = row.method; r.status = row.status;
        r.sources = [row.src, row.program].filter(Boolean); r.assumptions = row.rationale ? [row.rationale] : [];
      }
      r.side = { kind: 'isa_effect', wl: ref.wl, lever: ref.lever };
      if (ref.value !== undefined) r.side.value = ref.value;
    } else if (ref.kind === 'isa_encoding') {
      var enc = ((data.isa || {}).encoding || {})[ref.bits];
      r.id = String(ref.bits) + '-bit'; r.label = 'ISA opcode room'; r.method = (enc && enc.method) || 'spec'; r.status = (enc && enc.status) || undefined;
      if (enc) r.sources = citeList(enc.cite);
      r.side = { kind: 'chip', item: 'isa_encoding', col: ctx.col || 0 };
    } else if (ref.kind === 'derived') {
      r.rec = ctx.records[ref.id] || (ctx.derivedRecords && ctx.derivedRecords[ref.id]) || null;
      r.label = r.rec ? r.rec.label : ''; r.method = r.rec ? r.rec.method : 'derived';
    }
    return r;
  }
  function termHtml(t, ctx, depth) {
    var r = resolveRef(t, ctx);
    var h = '<li class="mx-term"><div class="mx-termline"><code>' + esc(t.sym) + '</code> = ' + fmtBandHtml(t.value, t.unit) + ' ' +
      methodBadge(r.method) + ' ' + statusBadge(r.status) +
      ' <span class="muted">' + esc(r.kind) + ' <code>' + esc(r.id) + '</code>' + (r.label ? ' (' + esc(r.label) + ')' : '') + '</span>' +
      (r.side ? ' <button type="button" class="linklike"' + sideAttr(r.side) + '>open</button>' : '') + '</div>';
    if (r.rec && depth < 8 && ctx.seen.indexOf(r.rec.id) < 0) {
      ctx.seen.push(r.rec.id);
      h += '<details class="mx-sub"><summary>derivation of <code>' + esc(r.rec.id) + '</code></summary>' + recTree(r.rec, ctx, depth + 1) + '</details>';
      ctx.seen.pop();
    } else if (r.rec) {
      h += '<div class="muted">(derivation of <code>' + esc(r.rec.id) + '</code> shown above)</div>';
    }
    // a source list already printed in this tree is not repeated (ctx.srcSeen is reset per tree / per limit)
    var key = (r.sources || []).join('\n'), src = srcList(r.sources);
    if (key && ctx.srcSeen) {
      if (ctx.srcSeen[key]) src = '<div class="muted small">sources as above</div>'; else ctx.srcSeen[key] = true;
    }
    return h + src + assumptionList(r.assumptions) + '</li>';
  }
  function recTree(rec, ctx, depth) {
    depth = depth || 0;
    var h = '<div class="mx-rec">';
    h += '<div><b>' + esc(rec.label || rec.id) + '</b> ' + methodBadge(rec.method) + ' = ' + fmtBandHtml(rec.value, rec.unit) + '</div>';
    if (rec.formula) h += '<div class="mono mx-formula">' + esc(rec.formula) + '</div>';
    if (rec.terms && rec.terms.length) h += '<ul class="mx-terms">' + rec.terms.map(function (t) { return termHtml(t, ctx, depth); }).join('') + '</ul>';
    var pd = rec.pess_direction && Object.keys(rec.pess_direction);
    if (pd && pd.length) {
      h += '<div class="muted">pessimistic end: ' + pd.map(function (k) { return '<code>' + esc(k) + '</code> at ' + esc(rec.pess_direction[k]); }).join(', ') + '</div>';
    }
    if (rec.note) h += '<div class="muted">' + esc(rec.note) + '</div>';
    return h + '</div>';
  }
  function niceTicks(lo, hi, n) {
    if (!(hi > lo)) { hi = lo + 1; }
    var raw = (hi - lo) / n, mag = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10)), f = raw / mag;
    var step = (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * mag, t = [], v = Math.ceil(lo / step) * step;
    for (; v <= hi + step * 1e-9; v += step) t.push(+v.toPrecision(12));
    return t;
  }
  function tickFmt(v) { return fmtNum(v, Math.abs(v) >= 100 || v === Math.round(v) ? 0 : undefined); }
  function eng(v) {   // compact axis number: 1.5M, 250k
    var a = Math.abs(v);
    if (a >= 1e9) return +(v / 1e9).toPrecision(3) + 'G';
    if (a >= 1e6) return +(v / 1e6).toPrecision(3) + 'M';
    if (a >= 1e4) return +(v / 1e3).toPrecision(3) + 'k';
    return tickFmt(v);
  }

  function mark(shape, x, y, r, attrs) {
    var a = attrs || '';
    if (shape === 'square') return '<rect x="' + (x - r) + '" y="' + (y - r) + '" width="' + 2 * r + '" height="' + 2 * r + '" ' + a + '/>';
    if (shape === 'diamond') return '<polygon points="' + x + ',' + (y - r - 1) + ' ' + (x + r + 1) + ',' + y + ' ' + x + ',' + (y + r + 1) + ' ' + (x - r - 1) + ',' + y + '" ' + a + '/>';
    if (shape === 'triangle') return '<polygon points="' + x + ',' + (y - r - 1) + ' ' + (x + r + 1) + ',' + (y + r) + ' ' + (x - r - 1) + ',' + (y + r) + '" ' + a + '/>';
    return '<circle cx="' + x + '" cy="' + y + '" r="' + r + '" ' + a + '/>';
  }

  // ------------------------------------------------------------ shared: attributes, badges, lookups added in v2
  function sideAttr(ref) { return ' data-side="' + esc(JSON.stringify(ref)) + '"'; }
  function focusAttr(wl, col) {
    return (wl !== undefined && wl !== null ? ' data-focus-wl="' + esc(wl) + '"' : '') +
      (col !== undefined && col !== null ? ' data-focus-col="' + esc(col) + '"' : '');
  }
  var CHIP_ICON = { pass: '✓', marginal: '~', fail: '✗' };
  // Chip-check verdict: pass/marginal/fail (never served/impossible, so it cannot be mistaken for a protocol verdict).
  function chipBadge(v) {
    return '<span class="verdict chip-' + esc(v) + '" data-chip-verdict="' + esc(v) + '"><span class="ic" aria-hidden="true">' +
      (CHIP_ICON[v] || '?') + '</span> ' + esc(v) + '</span>';
  }
  function wlName(data, id) { var w = workloadOf(data, id); return w ? (w.name || w.id) : String(id); }
  var TAG_PREF = ['cap19', 'stretch', 'device', 'mvp_fw', 'frames'];
  function tagsOf(data) {
    var seen = [];
    data.workloads.forEach(function (w) { (w.tags || []).forEach(function (t) { if (seen.indexOf(t) < 0) seen.push(t); }); });
    var out = TAG_PREF.filter(function (t) { return seen.indexOf(t) >= 0; });
    seen.forEach(function (t) { if (out.indexOf(t) < 0) out.push(t); });
    return out.concat(['all']);
  }

  // ------------------------------------------------------------ view registry
  var TAB_ORDER = ['capability', 'timing', 'area', 'throughput', 'explore', 'trust'];
  var TAB_TITLES = { capability: 'Capability', timing: 'Timing', area: 'Area & pins', throughput: 'Throughput', explore: 'Explore', trust: 'Trust' };
  var views = {};
  function register(id, v) { if (!v.title && TAB_TITLES[id]) v.title = TAB_TITLES[id]; views[id] = v; return v; }
  var sideKinds = {};
  function registerSide(kind, render, mount) { sideKinds[kind] = { render: render, mount: mount }; }
  var self = { views: views, TAB_ORDER: TAB_ORDER, TAB_TITLES: TAB_TITLES, register: register, registerSide: registerSide, sideKinds: sideKinds };

  // ------------------------------------------------------------ state (pure)
  var DEFAULT_TAG = 'cap19', DEFAULT_WL = 'spi_target_10m', MAX_PINNED = 5;   // UI defaults (ids), not model numbers
  function defaultsOf() { return A.defaultArch(); }
  var MEMO = {}, MEMO_N = 0, MEMO_DATA = null;     // evaluateAll per (tag, arch): pinned points do not recompute on every knob move
  function evalPoint(arch, data, tag) {
    if (MEMO_DATA !== data) { MEMO = {}; MEMO_N = 0; MEMO_DATA = data; }
    var key = tag + '|' + JSON.stringify(arch);
    if (MEMO[key]) return MEMO[key];
    if (MEMO_N >= 32) { MEMO = {}; MEMO_N = 0; }
    MEMO_N++;
    return (MEMO[key] = A.evaluateAll(arch, data, tag));
  }
  function wlExists(data, id) { return !!workloadOf(data, id); }
  function presetLabelOf(data, id) {
    var p = ((data && data.presets) || []).filter(function (q) { return q.id === id; })[0];
    return p ? (p.label || p.id) : (id || '');
  }
  // raw: {arch (full or partial), preset, tag, pinned, focus, scenario, side, tab, view}; returns a computed state.
  function makeState(raw, data) {
    raw = raw || {};
    VOCAB = data && data.vocab || VOCAB;
    var arch = Object.assign(A.defaultArch(), raw.arch || {});
    var tag = raw.tag || DEFAULT_TAG;
    var pinned = (raw.pinned || []).slice(0, MAX_PINNED).map(function (p, i) {
      return { arch: Object.assign(A.defaultArch(), p.arch || {}), label: p.label || ('point ' + (i + 1)) };
    });
    var f = raw.focus || {};
    var wl = wlExists(data, f.wl) ? f.wl : (wlExists(data, DEFAULT_WL) ? DEFAULT_WL : data.workloads[0].id);
    var col = Math.max(0, Math.min(pinned.length, parseInt(f.col, 10) || 0));
    var side = raw.side && typeof raw.side === 'object' && raw.side.kind ? raw.side : null;
    if (side && side.col !== undefined && !(side.col >= 0 && side.col <= pinned.length)) side = null;   // its point is gone
    var s = {
      arch: arch, preset: raw.preset || '', tag: tag, pinned: pinned, focus: { wl: wl, col: col },
      scenario: (raw.scenario || []).filter(function (id) { return wlExists(data, id); }), side: side,
      tab: TAB_ORDER.indexOf(raw.tab) >= 0 ? raw.tab : TAB_ORDER[0], view: raw.view || {}, data: data
    };
    s.points = [{ label: presetLabelOf(data, s.preset) || 'live', id: s.preset || '', arch: arch }].concat(pinned).map(function (p, i) {
      var all = evalPoint(p.arch, data, tag);
      return { label: p.label, id: p.id || '', col: i, live: i === 0, arch: p.arch, derived: all.arch, all: all, chip: all.chip };
    });
    s.point = s.points[col];
    s.derived = s.points[0].derived; s.all = s.points[0].all; s.chip = s.points[0].chip;
    s.cell = s.point.all.cells[wl] || A.evaluateCell(s.point.arch, wl, data, { derived: s.point.derived });
    return s;
  }
  var TAB_V1 = { matrix: 'capability', budgets: 'timing', architecture: 'timing', sweep: 'explore', calibration: 'trust', inputs: 'trust' };
  // Compact serialisation: arch as the difference from the MVP defaults. Used for location.hash.
  function archDiff(arch) {
    var d = defaultsOf(), diff = {};
    Object.keys(arch).forEach(function (k) { if (JSON.stringify(arch[k]) !== JSON.stringify(d[k])) diff[k] = arch[k]; });
    return diff;
  }
  function encodeState(s) {
    var o = { v: 2, a: archDiff(s.arch) };
    if (s.preset) o.p = s.preset;
    if (s.tab !== TAB_ORDER[0]) o.t = s.tab;
    if (s.tag !== DEFAULT_TAG) o.g = s.tag;
    if (s.pinned && s.pinned.length) o.n = s.pinned.map(function (p) { return { a: archDiff(p.arch), l: p.label }; });
    if (s.focus && (s.focus.wl !== DEFAULT_WL || s.focus.col)) o.f = s.focus;
    if (s.scenario && s.scenario.length) o.x = s.scenario;
    if (s.side) o.s = s.side;
    if (s.view && Object.keys(s.view).length) o.w = s.view;
    return JSON.stringify(o);
  }
  // Accepts v2 and v1 encodings (v1: no `v`, s = selection {wl, col, mix}, tabs matrix|budgets|architecture|sweep|calibration|inputs).
  function decodeState(str) {
    var o;
    try { o = JSON.parse(str); } catch (e) { return {}; }
    if (!o || typeof o !== 'object') return {};
    var r = { arch: o.a || {}, preset: o.p, tag: o.g };
    if (o.v === 2) {
      r.tab = o.t; r.focus = o.f; r.scenario = o.x; r.side = o.s; r.view = o.w;
      r.pinned = (o.n || []).map(function (p) { return { arch: p.a || p.arch || {}, label: p.l || p.label }; });
    } else {
      r.tab = TAB_V1[o.t] || o.t;
      var sel = o.s || {};
      if (sel.wl || sel.col) r.focus = { wl: sel.wl, col: sel.col };
      if (sel.mix) r.scenario = sel.mix;
      r.pinned = o.n || [];
    }
    return r;
  }

  // ------------------------------------------------------------ focus bar (pure)
  function workloadOptions(state) {
    var data = state.data, used = {}, h = '';
    TAG_PREF.concat(tagsOf(data)).forEach(function (t) {
      if (t === 'all' || used[t]) return;
      used[t] = true;
      var shown = data.workloads.filter(function (w) { return (w.tags || []).indexOf(t) >= 0 && !used['@' + w.id]; });
      if (!shown.length) return;
      h += '<optgroup label="' + esc(t) + '">' + shown.map(function (w) {
        used['@' + w.id] = true;
        return '<option value="' + esc(w.id) + '"' + (w.id === state.focus.wl ? ' selected' : '') + '>' + esc(w.name || w.id) + '</option>';
      }).join('') + '</optgroup>';
    });
    var rest = data.workloads.filter(function (w) { return !used['@' + w.id]; });
    if (rest.length) h += '<optgroup label="other">' + rest.map(function (w) {
      return '<option value="' + esc(w.id) + '"' + (w.id === state.focus.wl ? ' selected' : '') + '>' + esc(w.name || w.id) + '</option>';
    }).join('') + '</optgroup>';
    return h;
  }
  function renderFocus(state) {
    var pts = state.points;
    var h = '<div class="focus-row"><label>protocol <select data-focus-pick="wl" aria-label="focused protocol">' + workloadOptions(state) + '</select></label>' +
      '<label>on point <select data-focus-pick="col" aria-label="focused point">' + pts.map(function (p, i) {
        return '<option value="' + i + '"' + (i === state.focus.col ? ' selected' : '') + '>' + esc(i === 0 ? 'live: ' + p.label : p.label) + '</option>';
      }).join('') + '</select></label>' + verdictBadge(state.cell.verdict) + '</div>';
    if (state.focus.col > 0) {
      h += '<div class="focus-note" role="status">Showing pinned point <b>' + esc(state.point.label) + '</b>; knobs edit the live point. ' +
        '<button type="button" data-action="load-point" data-value="' + state.focus.col + '">load into live</button></div>';
    }
    return h;
  }
  function renderTabs(state) {
    return TAB_ORDER.map(function (id) {
      return '<button type="button" role="tab" data-tab="' + id + '" aria-selected="' + (id === state.tab) + '">' +
        esc((views[id] || {}).title || TAB_TITLES[id] || id) + '</button>';
    }).join('');
  }
  function renderView(state) {
    var v = views[state.tab];
    return v ? v.render(state) : '';
  }
  function renderRail(state, open) {
    var v = views.configure;
    return v && v.render ? v.render(state, { open: open || {} }) : '';
  }
  function renderChip(state) {
    var v = views.configure;
    return v && v.renderChip ? v.renderChip(state) : '';
  }

  // ------------------------------------------------------------ derivation side panel (pure)
  // ctx for derivTree: records of the point (derive records, chip records, optionally one cell's records).
  function derivCtx(state, col, cell) {
    var p = state.points[col || 0] || state.points[0], recs = {};
    Object.keys(p.derived.records || {}).forEach(function (k) { recs[k] = p.derived.records[k]; });
    Object.keys(p.chip.records || {}).forEach(function (k) { recs[k] = p.chip.records[k]; });
    Object.keys((cell && cell.records) || {}).forEach(function (k) { recs[k] = cell.records[k]; });
    return { data: state.data, records: recs, derivedRecords: p.derived.records, seen: [], srcSeen: {}, col: col || 0 };
  }
  function pointLabel(state, col) { var p = state.points[col]; return p ? (col === 0 ? 'live point' : 'pinned point ' + p.label) : ''; }
  function srcBlock(list, assume) { return srcList(list) + assumptionList(assume); }
  function sideRecord(state, ref) {
    var col = ref.col || 0, p = state.points[col] || state.points[0];
    var rec = p.derived.records[ref.id] || p.chip.records[ref.id];
    if (!rec) return { title: ref.id, html: '<p class="muted">No derivation record <code>' + esc(ref.id) + '</code> on this point.</p>' };
    return { title: rec.label || rec.id, html: '<div class="mx-tree">' + recTree(rec, derivCtx(state, col), 0) + '</div>' };
  }
  function sideParam(state, ref) {
    var p = state.data.params[ref.id];
    if (!p) return { title: ref.id, html: '<p class="muted">No parameter <code>' + esc(ref.id) + '</code> in inputs.json.</p>' };
    var ov = (state.arch.param_overrides || {})[ref.id];
    var h = '<div class="mx-rec"><div><b>' + esc(p.label || p.id) + '</b> ' + methodBadge(p.method) + ' ' + statusBadge(p.status) + '</div>' +
      '<div class="num">' + fmtBandHtml(p, p.unit) + '</div>' +
      (ov !== undefined ? '<div class="muted">overridden on the live point: <span class="num">' + esc(JSON.stringify(ov)) + '</span></div>' : '') +
      srcBlock((p.sources || []).concat(citeList(p.cite)), p.assumptions) + '</div>';
    return { title: p.label || p.id, html: h };
  }
  function sideChip(state, ref) {
    var col = ref.col || 0, p = state.points[col] || state.points[0];
    var it = p.chip.items.filter(function (q) { return q.id === ref.item; })[0], rec = p.chip.records[ref.item];
    if (!it || !rec) return { title: ref.item, html: '<p class="muted">No chip-check item <code>' + esc(ref.item) + '</code> on this point.</p>' };
    var h = '<div>' + chipBadge(it.verdict) + (p.chip.driver === it.id ? ' <span class="badge">driver</span>' : '') +
      ' <span class="muted">chip check of the ' + esc(pointLabel(state, col)) + ': ' + chipBadge(p.chip.verdict) + '</span></div>' +
      '<div class="mx-tree">' + recTree(rec, derivCtx(state, col), 0) + '</div>';
    return { title: it.label || it.short, html: h };
  }
  registerSide('record', sideRecord);
  registerSide('param', sideParam);
  registerSide('chip', sideChip);
  var STICKY_KINDS = ['record', 'param', 'chip'];
  function isSticky(side) { return !!side && (side.sticky || STICKY_KINDS.indexOf(side.kind) >= 0); }

  // Header + body of the side panel for state.side; '' when closed.
  // refs without a point: the data they describe is not per point
  var SIDE_NOCOL = { calibration: 'data (all points)', isa_effect: 'data (all points)', param: 'data (all points)' };
  function renderSide(state) {
    var ref = state.side;
    if (!ref) return '';
    var k = sideKinds[ref.kind], title = ref.kind, body = '';
    try {
      if (!k) body = '<p class="muted">No renderer registered for side kind <code>' + esc(ref.kind) + '</code>.</p>';
      else {
        var out = k.render(state, ref);
        if (out && typeof out === 'object') { title = out.title || title; body = out.html || ''; } else body = out || '';
      }
    } catch (e) { body = errorBox('side panel (' + ref.kind + ')', e); }
    var where = ref.col !== undefined ? pointLabel(state, ref.col) : (SIDE_NOCOL[ref.kind] || '');
    return '<div class="side-head"><div><b>' + esc(title) + '</b>' + (where ? ' <span class="muted">' + esc(where) + '</span>' : '') +
      '</div><button type="button" data-side-close aria-label="close the derivation panel">close</button></div>' +
      '<div class="side-body" data-side-kind="' + esc(ref.kind) + '">' + body + '</div>';
  }
  function openSide(ref) { return setState({ side: ref }); }
  function closeSide() { return setState({ side: null }); }

  var RESET_HINT = 'Try resetting the state: clear the URL hash (remove everything after #) and the page\'s localStorage (keys archsim.v2, archsim.v1), then reload.';
  function errorBox(what, e) {
    if (typeof console !== 'undefined' && console.error) console.error('archsim: ' + what + ' failed', e);
    return '<div class="render-error" role="alert"><b>' + esc(what) + ' failed:</b> <code>' + esc(String((e && e.message) || e)) + '</code>' +
      '<p>' + esc(RESET_HINT) + '</p></div>';
  }

  // ------------------------------------------------------------ DOM glue (browser only)
  var S = null, DATA_ = null, OPEN_GROUPS = {}, OPEN_MAP = {}, pendingArch = null, LS_KEY = 'archsim.v2', LS_V1 = 'archsim.v1';
  function $(id) { return document.getElementById(id); }
  function lsGet(k) { try { return window.localStorage.getItem(k); } catch (e) { return null; } }
  function lsSet(k, v) { try { window.localStorage.setItem(k, v); } catch (e) { /* convenience only */ } }

  function persist() {
    var enc = encodeState(S);
    try { window.history.replaceState(null, '', '#' + encodeURIComponent(enc)); } catch (e) { /* file:// quirks */ }
    lsSet(LS_KEY, enc);
  }
  function rawOf(s) {
    return { arch: s.arch, preset: s.preset, tag: s.tag, pinned: s.pinned, focus: s.focus, scenario: s.scenario, side: s.side, tab: s.tab, view: s.view };
  }
  // <details> open/closed survives repaints: keyed by data-key or the summary text (plus a counter for repeats), per container and tab.
  function detailKey(d, seen) {
    var sm = d.querySelector ? d.querySelector('summary') : null;
    var base = d.getAttribute('data-key') || (sm ? sm.textContent.trim() : '');
    seen[base] = (seen[base] || 0) + 1;
    return base + '#' + seen[base];
  }
  function captureOpen(el, mapKey) {
    var m = OPEN_MAP[mapKey] || (OPEN_MAP[mapKey] = {}), seen = {}, ds = el.querySelectorAll('details');
    for (var i = 0; i < ds.length; i++) m[detailKey(ds[i], seen)] = !!ds[i].open;
  }
  function restoreOpen(el, mapKey) {
    var m = OPEN_MAP[mapKey];
    if (!m) return;
    var seen = {}, ds = el.querySelectorAll('details');
    for (var i = 0; i < ds.length; i++) { var k = detailKey(ds[i], seen); if (k in m) ds[i].open = m[k]; }
  }
  // Keyboard focus survives a repaint: remember the focused element inside `el` (id, else tag + data-* attributes + its index
  // among equals) and focus the equivalent element afterwards.
  function focusSig(el) {
    var a = typeof document !== 'undefined' ? document.activeElement : null;
    if (!a || a === document.body || !el.contains(a)) return null;
    var attrs = {}, n = a.attributes, i;
    for (i = 0; i < n.length; i++) if (n[i].name.indexOf('data-') === 0 || n[i].name === 'aria-label') attrs[n[i].name] = n[i].value;
    var same = [].slice.call(el.querySelectorAll(a.tagName)).filter(function (q) { return sameSig(q, a.id, attrs); });
    return { id: a.id, tag: a.tagName, attrs: attrs, nth: Math.max(0, same.indexOf(a)) };
  }
  function sameSig(q, id, attrs) {
    if (id) return q.id === id;
    var keys = Object.keys(attrs);
    return keys.length > 0 && keys.every(function (k) { return q.getAttribute(k) === attrs[k]; });
  }
  function restoreFocus(el, sig) {
    if (!sig || (!sig.id && !Object.keys(sig.attrs).length)) return;
    var c = [].slice.call(el.querySelectorAll(sig.tag)).filter(function (q) { return sameSig(q, sig.id, sig.attrs); });
    var t = c[sig.nth] || c[0];
    if (t && t.focus) { try { t.focus({ preventScroll: true }); } catch (e) { t.focus(); } }
  }
  function guarded(el, what, fn, mapKey) {
    if (!el) return;
    if (mapKey) captureOpen(el, mapKey);
    var sig = focusSig(el);
    try { el.innerHTML = fn(); } catch (e) { el.innerHTML = errorBox(what, e); }
    if (mapKey) restoreOpen(el, mapKey);
    restoreFocus(el, sig);
  }
  function mountSafe(el, what, fn) {
    try { fn(); } catch (e) { el.insertAdjacentHTML('beforeend', errorBox(what, e)); }
  }
  function paintRail() {
    guarded($('rail'), 'configure rail', function () { return renderRail(S, OPEN_GROUPS); });
    var v = views.configure;
    if (v && v.mount) mountSafe($('rail'), 'configure mount', function () { v.mount($('rail'), S); });
  }
  function paintChip() {
    guarded($('chip'), 'chip check', function () { return renderChip(S); }, 'chip');
  }
  function paintView() {
    guarded($('view'), 'view', function () { return renderView(S); }, 'view:' + S.tab);
    var v = views[S.tab];
    if (v && v.mount) mountSafe($('view'), 'view mount', function () { v.mount($('view'), S); });
  }
  function paintSide() {
    guarded($('side'), 'side panel', function () { return renderSide(S); }, 'side');
    var k = S.side && sideKinds[S.side.kind];
    if (k && k.mount) mountSafe($('side'), 'side mount', function () { k.mount($('side').querySelector('.side-body'), S, S.side); });
    document.body.classList.toggle('has-side', !!S.side);
  }
  function paintAll(rebuildRail) {
    if (rebuildRail) paintRail();
    paintChip();
    guarded($('focus'), 'focus bar', function () { return renderFocus(S); });
    guarded($('tabs'), 'tab bar', function () { return renderTabs(S); });
    paintView();
    paintSide();
    var cv = views.configure;
    if (cv && cv.update) { try { cv.update(S); } catch (e) { /* the live section keeps its last paint */ } }
    decorate();
    persist();
  }
  // patch keys: arch (merged into the live arch), preset, tag, pinned, focus, scenario, side, tab, view.
  // rebuildRail true when knob values change from outside the rail (preset load, reset, quick value, load point).
  function setState(patch, rebuildRail) {
    var raw = rawOf(S);
    Object.keys(patch || {}).forEach(function (k) {
      if (k === 'arch') raw.arch = Object.assign({}, raw.arch, patch.arch);
      else raw[k] = patch[k];
    });
    if (patch && 'focus' in patch && !('side' in patch) && JSON.stringify(raw.focus) !== JSON.stringify(S.focus) && !isSticky(S.side)) raw.side = null;
    S = makeState(raw, DATA_);
    paintAll(!!rebuildRail);
    return S;
  }
  function getState() { return S; }
  function setFocus(wl, col) { return setState({ focus: { wl: wl === undefined || wl === null ? S.focus.wl : wl, col: col === undefined || col === null ? S.focus.col : col } }); }
  function setTab(t) { return setState({ tab: t }); }
  function setView(id, patch) {
    var v = Object.assign({}, S.view); v[id] = Object.assign({}, v[id] || {}, patch || {});
    return setState({ view: v });
  }
  function setScenario(list) { return setState({ scenario: list }); }
  // Settling: a knob edit just happened (slider drag, typing); heavy views may defer their solvers until SETTLE_MS of quiet.
  var SETTLE_MS = 250, settleTimer = null;      // UI debounce interval, not a model number
  function settling() { return settleTimer !== null; }
  function beginSettle() {
    if (typeof setTimeout !== 'function' || !S || S.tab !== 'explore') return;
    if (settleTimer !== null) clearTimeout(settleTimer);
    settleTimer = setTimeout(function () { settleTimer = null; if (S && S.tab === 'explore') { paintView(); paintSide(); decorate(); } }, SETTLE_MS);
  }
  // Coalesced arch edit: many calls inside one animation frame become one recompute (sliders, text inputs).
  function setArch(patch) {
    var first = !pendingArch;
    pendingArch = Object.assign(pendingArch || {}, patch);
    if (!first) return;
    var flush = function () { var q = pendingArch; pendingArch = null; if (q) { beginSettle(); setState({ arch: q, preset: '' }); } };
    if (typeof window !== 'undefined' && typeof window.requestAnimationFrame === 'function') window.requestAnimationFrame(flush); else flush();
  }
  function presetLabel(id) {
    var p = (DATA_.presets || []).filter(function (q) { return q.id === id; })[0];
    return p ? (p.label || p.id) : id;
  }
  function pinCurrent() {
    if (S.pinned.length >= MAX_PINNED) return S;
    return setState({ pinned: S.pinned.concat([{ arch: S.arch, label: S.preset ? presetLabel(S.preset) : 'point ' + (S.pinned.length + 1) }]) });
  }
  function pinPreset(id) {
    if (S.pinned.length >= MAX_PINNED || !id) return S;
    return setState({ pinned: S.pinned.concat([{ arch: A.applyPreset(id, DATA_), label: presetLabel(id) }]) });
  }
  function loadPreset(id) {
    if (!id) return setState({ preset: '' });
    return setState({ arch: A.applyPreset(id, DATA_), preset: id }, true);
  }
  function loadPoint(col) {
    var p = S.pinned[col - 1];
    if (!p) return S;
    var patch = { arch: p.arch, preset: '' };
    if (S.focus.col === col) patch.focus = { wl: S.focus.wl, col: 0 };
    return setState(patch, true);
  }
  function removePoint(col) {
    if (!S.pinned[col - 1]) return S;
    var pins = S.pinned.slice(); pins.splice(col - 1, 1);
    var patch = { pinned: pins }, fc = S.focus.col;
    if (fc === col) patch.focus = { wl: S.focus.wl, col: 0 };
    else if (fc > col) patch.focus = { wl: S.focus.wl, col: fc - 1 };
    if (S.side && S.side.col !== undefined) {
      if (S.side.col === col) patch.side = null;
      else if (S.side.col > col) patch.side = Object.assign({}, S.side, { col: S.side.col - 1 });
    }
    return setState(patch);
  }
  function resetToMvp() { return setState({ arch: A.defaultArch(), preset: 'mvp' }, true); }

  function readKnob(el) {
    var id = el.getAttribute('data-knob') || el.getAttribute('data-knob-range');
    var k = A.KNOBS.filter(function (q) { return q.id === id; })[0];
    if (!k) return null;
    var v;
    if (k.type === 'number') { v = parseFloat(el.value); if (!(v === v)) return null; }
    else if (k.type === 'bool') v = el.checked;
    else if (k.type === 'enum') { v = k.options.filter(function (o) { return String(o) === el.value; })[0]; }
    else return null;
    return { id: id, value: v, knob: k };
  }
  function syncNumber(id, value) {
    var els = document.querySelectorAll('[data-knob="' + id + '"],[data-knob-range="' + id + '"]');
    for (var i = 0; i < els.length; i++) if (els[i].value !== String(value)) els[i].value = value;
  }
  function wireRail() {
    var rail = $('rail');
    // 'toggle' does not bubble: capture it to remember which knob groups the user opened or closed.
    rail.addEventListener('toggle', function (e) {
      var t = e.target;
      if (t && t.getAttribute && t.getAttribute('data-group') !== null) OPEN_GROUPS[t.getAttribute('data-group')] = !!t.open;
    }, true);
    rail.addEventListener('input', function (e) {
      var t = e.target;
      if (t.getAttribute('data-knob-range') !== null || (t.getAttribute('data-knob') && t.type === 'number')) {
        var r = readKnob(t);
        if (!r) return;
        syncNumber(r.id, r.value);
        if (r.value >= r.knob.min && r.value <= r.knob.max) { var p = {}; p[r.id] = r.value; setArch(p); }
      }
    });
    rail.addEventListener('change', function (e) {
      var t = e.target, p = {};
      if (t.hasAttribute('data-preset')) { if (t.hasAttribute('data-autoload')) loadPreset(t.value); return; }
      if (t.getAttribute('data-multi')) {
        var id = t.getAttribute('data-multi'), boxes = rail.querySelectorAll('[data-multi="' + id + '"]'), vals = [];
        for (var i = 0; i < boxes.length; i++) if (boxes[i].checked) vals.push(boxes[i].value);
        p[id] = vals;
        setState({ arch: p, preset: '' });
        return;
      }
      if (t.getAttribute('data-knob') && t.type !== 'number') {
        var r = readKnob(t);
        if (r) { p[r.id] = r.value; setState({ arch: p, preset: '' }); }
      }
    });
  }
  function actionValue(t) {
    var v = t.getAttribute('data-value');
    if (v !== null) return v;
    var sel = document.querySelector('#rail select[data-preset]');
    return sel ? sel.value : '';
  }
  var ACT_SEL = '[data-action],[data-quick],[data-focus-wl],[data-focus-col],[data-tab],[data-side],[data-side-close],[data-open-rail],[data-close-rail]';
  var NATIVE = { BUTTON: 1, A: 1, INPUT: 1, SELECT: 1, TEXTAREA: 1, SUMMARY: 1, OPTION: 1 };
  // Make every declarative-action element that is not natively interactive focusable and announced as a button.
  function decorate() {
    var els = document.querySelectorAll(ACT_SEL);
    for (var i = 0; i < els.length; i++) {
      var el = els[i];
      if (NATIVE[el.tagName] || el.tagName === 'SELECT') continue;
      if (!el.hasAttribute('tabindex')) el.setAttribute('tabindex', '0');
      if (!el.hasAttribute('role') && !/^(TR|TD|TH|LI)$/.test(el.tagName)) el.setAttribute('role', 'button');
    }
  }
  function onClick(e) {
    var t = e.target && e.target.closest ? e.target.closest(ACT_SEL) : null;
    if (t) activate(t);
  }
  function onKey(e) {
    if (e.key !== 'Enter' && e.key !== ' ') return;
    var t = e.target && e.target.closest ? e.target.closest(ACT_SEL) : null;
    if (!t || NATIVE[e.target.tagName] || NATIVE[t.tagName]) return;
    e.preventDefault();
    activate(t);
  }
  function activate(t) {
    var act = t.getAttribute('data-action');
    if (act === 'reset') return void resetToMvp();
    if (act === 'pin') return void pinCurrent();
    if (act === 'load-preset') return void loadPreset(actionValue(t));
    if (act === 'pin-preset') return void pinPreset(actionValue(t));
    if (act === 'load-point') return void loadPoint(parseInt(t.getAttribute('data-value'), 10));
    if (act === 'remove-point') return void removePoint(parseInt(t.getAttribute('data-value'), 10));
    if (t.getAttribute('data-quick')) {
      var q = {}; q[t.getAttribute('data-quick')] = parseFloat(t.getAttribute('data-value'));
      return void setState({ arch: q, preset: '' }, true);
    }
    if (t.hasAttribute('data-open-rail')) return void document.body.classList.add('panel-open');
    if (t.hasAttribute('data-close-rail')) return void document.body.classList.remove('panel-open');
    var patch = {};
    var fw = t.getAttribute('data-focus-wl'), fc = t.getAttribute('data-focus-col');
    if (fw !== null || fc !== null) patch.focus = { wl: fw !== null ? fw : S.focus.wl, col: fc !== null ? parseInt(fc, 10) : S.focus.col };
    if (t.getAttribute('data-tab')) { patch.tab = t.getAttribute('data-tab'); document.body.classList.remove('panel-open'); }
    if (t.hasAttribute('data-side-close')) patch.side = null;
    else if (t.getAttribute('data-side')) { try { patch.side = JSON.parse(t.getAttribute('data-side')); } catch (err) { /* ignore a malformed ref */ } }
    if (Object.keys(patch).length) setState(patch);
  }
  function onChange(e) {
    var t = e.target, pick = t && t.getAttribute ? t.getAttribute('data-focus-pick') : null;
    if (!pick) return;
    if (pick === 'wl') setFocus(t.value, null); else setFocus(null, parseInt(t.value, 10) || 0);
  }
  function wireMain() {
    document.addEventListener('click', onClick);
    document.addEventListener('keydown', onKey);
    document.addEventListener('change', onChange);
    document.addEventListener('keydown', function (e) {
      if (e.key !== 'Escape') return;
      if (S && S.side) closeSide(); else document.body.classList.remove('panel-open');
    });
    $('theme-toggle').addEventListener('click', function () {
      var cur = document.documentElement.getAttribute('data-theme');
      if (!cur) cur = (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) ? 'dark' : 'light';
      var next = cur === 'dark' ? 'light' : 'dark';
      document.documentElement.setAttribute('data-theme', next);
      lsSet(LS_KEY + '.theme', next);
    });
    $('knobs-toggle').addEventListener('click', function () { document.body.classList.toggle('panel-open'); });
    window.addEventListener('hashchange', function () {
      var h = window.location.hash.replace(/^#/, '');
      if (!h) return;
      var enc;
      try { enc = decodeURIComponent(h); } catch (e) { return; }
      if (enc !== encodeState(S)) { S = makeState(decodeState(enc), DATA_); paintAll(true); }
    });
  }

  // Browser entry point (called by boot.js after all view files have registered).
  function init() {
    DATA_ = JSON.parse($('archsim-data').textContent);
    var th = lsGet(LS_KEY + '.theme') || lsGet(LS_V1 + '.theme');
    if (th === 'light' || th === 'dark') document.documentElement.setAttribute('data-theme', th);
    var h = window.location.hash.replace(/^#/, ''), raw = null;
    try { if (h) raw = decodeState(decodeURIComponent(h)); } catch (e) { raw = null; }
    if (!raw || !Object.keys(raw).length) { var ls = lsGet(LS_KEY) || lsGet(LS_V1); raw = ls ? decodeState(ls) : {}; }
    try { S = makeState(raw, DATA_); } catch (e) { raw = {}; S = makeState({}, DATA_); }
    if (!S.preset && !Object.keys(raw.arch || {}).length) S.preset = 'mvp';
    $('stamp').textContent = (DATA_.stamp && DATA_.stamp.sources_sha256) ? 'data ' + DATA_.stamp.sources_sha256.slice(0, 12) : '';
    wireRail();
    wireMain();
    paintAll(true);
  }

  Object.assign(self, {
    esc: esc, fmtNum: fmtNum, fmtBand: fmtBand, fmtBandHtml: fmtBandHtml, unitStr: unitStr, methodBadge: methodBadge,
    statusBadge: statusBadge, verdictBadge: verdictBadge, chipBadge: chipBadge, stat: stat, recordHtml: recordHtml, table: table,
    svgWrap: svgWrap, workloadOf: workloadOf, pessOptBar: barSvg, derivTree: recTree, derivCtx: derivCtx, resolveRef: resolveRef,
    settling: settling, isaActive: isaActive, estimateMark: estimateMark, cellIsa: cellIsa, isaBlockIds: isaBlockIds,
    niceTicks: niceTicks, tickFmt: tickFmt, eng: eng, mark: mark, tagsOf: tagsOf, wlName: wlName, sideAttr: sideAttr, focusAttr: focusAttr,
    errorBox: errorBox, makeState: makeState, encodeState: encodeState, decodeState: decodeState,
    renderFocus: renderFocus, renderTabs: renderTabs, renderView: renderView, renderRail: renderRail, renderChip: renderChip,
    renderSide: renderSide, openSide: openSide, closeSide: closeSide,
    setState: setState, getState: getState, setFocus: setFocus, setTab: setTab, setView: setView, setScenario: setScenario, setArch: setArch,
    pinCurrent: pinCurrent, pinPreset: pinPreset, loadPreset: loadPreset, loadPoint: loadPoint, removePoint: removePoint, resetToMvp: resetToMvp,
    init: init
  });
  return self;
})();
