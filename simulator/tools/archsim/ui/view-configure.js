/* Configure: the rail (preset row, points list, compact live block diagram, knob groups) and
 * the chip check card (#chip). The architecture is a choice: this is where you set and see the chip, and the chip check
 * (every ArchSim.chipCheck item) is stated here once. Everything is generated from ArchSim.KNOBS / GROUPS / chipCheck /
 * blockDiagram, so new knob groups (ISA) and chip items (isa_encoding) appear without code changes.
 * The core repaints #rail only when knob values change from outside, so the sections that depend on the live point
 * (preset buttons, points list, diagram) are refreshed by the registry hook update(state), called on every state change.
 * Side kind registered here: 'block' {kind:'block', id, col} (a diagram box).
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum;

  // Knobs that are evaluation settings rather than architecture choices (PLAN-v2 1.2 #3).
  var EVAL_KNOBS = ['env', 'signoff_corner'], EVAL_GROUP = 'Evaluation settings', ISA_GROUP = 'ISA';
  // Result tab that explains the composition of a chip item (PLAN-v2 1.1 (3)); board_clock and isa_encoding: derivation only.
  var BREAKDOWN_TAB = { core_clock: 'timing', die_area: 'area', pins_budget: 'area' };
  var BREAKDOWN_TEXT = { timing: 'clock budget', area: 'area & pins' };

  // ------------------------------------------------------------ knob panel (pure)
  function knobControl(k, v) {
    var id = esc(k.id), isa = k.group === ISA_GROUP;
    var h = '<div class="knob" data-knob-wrap="' + id + '"><label for="knob-' + id + '">' + esc(k.label) +
      (k.unit ? ' <span class="u">(' + esc(k.unit) + ')</span>' : '') + (isa ? ' ' + U.statusBadge('estimate') : '') + '</label><div class="ctl">';
    if (k.type === 'number') {
      h += '<input type="number" id="knob-' + id + '" data-knob="' + id + '" min="' + k.min + '" max="' + k.max +
        '" step="' + k.step + '" value="' + esc(v) + '">' +
        '<input type="range" data-knob-range="' + id + '" min="' + k.min + '" max="' + k.max + '" step="' + k.step +
        '" value="' + esc(v) + '" aria-label="' + esc(k.label) + ' slider">';
      (k.quick || []).forEach(function (q) { h += '<button type="button" data-quick="' + id + '" data-value="' + q + '">' + q + '</button>'; });
    } else if (k.type === 'enum') {
      h += '<select id="knob-' + id + '" data-knob="' + id + '">' + k.options.map(function (o) {
        return '<option value="' + esc(o) + '"' + (String(o) === String(v) ? ' selected' : '') + '>' + esc(o) + '</option>';
      }).join('') + '</select>';
    } else if (k.type === 'bool') {
      h += '<input type="checkbox" id="knob-' + id + '" data-knob="' + id + '"' + (v ? ' checked' : '') + '>';
    } else if (k.type === 'multi') {
      h += '<fieldset id="knob-' + id + '" data-knob="' + id + '"><legend class="u">choose any</legend>' + k.options.map(function (o) {
        return '<label><input type="checkbox" data-multi="' + id + '" value="' + esc(o) + '"' +
          ((v || []).indexOf(o) >= 0 ? ' checked' : '') + '> ' + esc(o) + '</label>';
      }).join('') + '</fieldset>';
    }
    h += '</div>' + (k.help ? '<div class="help">' + esc(k.help) + '</div>' : '') + '</div>';
    return h;
  }
  // Knob groups: GROUPS in order (minus the evaluation knobs), then "Evaluation settings". open: {group: bool} from the core.
  function renderGroups(state, open) {
    var groups = A.GROUPS.map(function (g) {
      return { name: g, knobs: A.KNOBS.filter(function (k) { return k.group === g && EVAL_KNOBS.indexOf(k.id) < 0; }) };
    });
    groups.push({ name: EVAL_GROUP, knobs: A.KNOBS.filter(function (k) { return EVAL_KNOBS.indexOf(k.id) >= 0; }) });
    groups = groups.filter(function (g) { return g.knobs.length; });
    return '<div class="cfg-knobs">' + groups.map(function (g, gi) {
      var isOpen = open && open[g.name] !== undefined ? open[g.name] : gi < 2;
      var isa = g.name === ISA_GROUP;
      return '<details' + (isOpen ? ' open' : '') + ' data-group="' + esc(g.name) + '"><summary>' + esc(g.name) +
        (isa ? ' ' + U.statusBadge('estimate') : '') + '</summary>' +
        (isa ? '<p class="help cfg-note">Instruction-set levers: their per-protocol effects are estimates (hand-counted or judged), not measurements. ' +
          '<button type="button" class="linklike" data-tab="trust">see Trust: ISA estimates</button></p>' : '') +
        g.knobs.map(function (k) { return knobControl(k, state.arch[k.id]); }).join('') + '</details>';
    }).join('') + '</div>';
  }

  // ------------------------------------------------------------ preset row and points list (pure)
  function renderPresets(state) {
    var presets = (state.data && state.data.presets) || [], full = state.pinned.length >= 5;
    return '<div class="cfg-presets"><div class="row"><label for="preset">Preset</label><select id="preset" data-preset>' +
      '<option value=""' + (state.preset ? '' : ' selected') + '>(custom)</option>' + presets.map(function (p) {
        return '<option value="' + esc(p.id) + '"' + (p.id === state.preset ? ' selected' : '') + '>' + esc(p.label || p.id) + '</option>';
      }).join('') + '</select>' +
      '<button type="button" data-action="load-preset" title="load the selected preset into the live point">load</button>' +
      '<button type="button" data-action="pin-preset"' + (full ? ' disabled' : '') + ' title="add the selected preset as a pinned point">pin</button></div>' +
      '<div class="row"><button type="button" data-action="pin"' + (full ? ' disabled' : '') + '>Pin current</button>' +
      '<button type="button" data-action="reset">Reset to MVP</button></div></div>';
  }
  function renderPoints(state) {
    var rows = state.points.map(function (p, i) {
      var cur = i === state.focus.col;
      return '<li class="pt' + (cur ? ' cur' : '') + '" data-point="' + i + '"><button type="button" class="pt-name"' + U.focusAttr(null, i) +
        (cur ? ' aria-current="true"' : '') + ' title="show this point in the result tabs">' + esc(i === 0 ? 'live: ' + p.label : p.label) + '</button> ' +
        '<span class="pt-chip" title="chip check of this point">' + U.chipBadge(p.chip.verdict) + '</span>' +
        (i > 0 ? ' <button type="button" data-action="load-point" data-value="' + i + '">load into live</button>' +
          ' <button type="button" data-action="remove-point" data-value="' + i + '" aria-label="remove ' + esc(p.label) + '">remove</button>' : '') + '</li>';
    }).join('');
    return '<div class="cfg-points"><h2>Points</h2><ul class="pts">' + rows + '</ul></div>';
  }

  // ------------------------------------------------------------ chip check card (pure)
  // Chip items whose value an active ISA lever moves: the fmax (core clock band) and area blocks named in inputs.json isa.levers,
  // and the opcode room itself.
  function isaMovedItems(state) {
    var on = U.isaActive(state.arch), lev = (state.data.isa || {}).levers || {}, out = {};
    if (!on.length) return out;
    out.isa_encoding = true;
    on.forEach(function (l) {
      if ((lev[l] || {}).fmax_param) out.core_clock = true;
      if (((lev[l] || {}).blocks || []).length) out.die_area = true;
    });
    return out;
  }
  function renderChip(state) {
    var c = state.chip, d = c.items.filter(function (i) { return i.id === c.driver; })[0];
    var head = '<div class="chip-head" data-open-rail>chip check: ' + U.chipBadge(c.verdict) + ' <span class="muted">' +
      (c.verdict === 'pass' ? 'every item fits; tightest: ' : 'driven by ') + esc(d ? d.short : '') + '</span></div>';
    var movedByIsa = isaMovedItems(state);
    var rows = c.items.map(function (it) {
      var tab = BREAKDOWN_TAB[it.id];
      return '<div class="chip-row ' + esc(it.verdict) + '" data-chip-item="' + esc(it.id) + '">' +
        '<button type="button" class="chip-item"' + U.sideAttr({ kind: 'chip', item: it.id, col: 0 }) + ' title="' + esc(it.label) + ': open the derivation">' +
        U.chipBadge(it.verdict) + ' <span class="nm">' + esc(it.short) + '</span> ' +
        '<span class="mg num">' + U.fmtBandHtml(it.value, it.unit) + '</span>' +
        (movedByIsa[it.id] ? ' ' + U.estimateMark('moved by ISA lever estimates (judgement)') : '') +
        (c.driver === it.id ? ' <span class="badge driver" title="the item that explains the verdict">driver</span>' : '') + '</button>' +
        (tab ? '<button type="button" class="linklike" data-tab="' + tab + '" title="where this is composed">breakdown: ' + BREAKDOWN_TEXT[tab] + '</button>' : '') +
        '</div>';
    }).join('');
    return '<div class="chip-card">' + head + '<div class="chip-body"><div class="chip-sub muted">margin = limit minus use, nominal (pessimistic..optimistic) ' +
      'of the live point</div>' + rows + '</div></div>';
  }

  // ------------------------------------------------------------ compact live block diagram (pure; ported from v1 view-budgets)
  var DIAGRAM_ROWS = [
    ['lane', 'ddr_io', 'dtc'],
    ['pinmux'],
    ['@engines', 'crc', 'dpll', 'adpll'],
    ['deep_fifo', 'regfile', '@extras'],
    ['link', 'host_mcu']
  ];
  var DW = 300, BOX_H = 44, ROW_GAP = 14, GAP = 6, W_MIN = 64, W_VAR = 70;   // drawing geometry in SVG units, not model numbers
  function areaOf(b) { return b.area_placed || b.area_cell || b.area; }
  function boxLines(b) {
    var lines = [b.label];
    if (b.kind === 'engine') {
      lines.push(b.imem_words + ' w ' + b.imem_tech + (b.imem_banks > 1 ? ' x' + b.imem_banks : ''));
      lines.push('FIFO ' + b.fifo_depth + (b.crc ? ', CRC' : '') + (b.dpll ? ', DPLL' : ''));
    } else if (b.kind === 'link' && b.link) lines = [b.link.id];
    else if (b.note) lines.push(b.note);
    return lines;
  }
  function liveCell(state) {
    var wl = state.focus.wl;
    return state.all.cells[wl] || A.evaluateCell(state.arch, wl, state.data, { derived: state.derived });
  }
  function renderDiagram(state) {
    var bd = A.blockDiagram(state.arch, state.data), cell = liveCell(state), driver = state.chip.driver, byId = {}, s = '', y = 6;
    bd.blocks.forEach(function (b) { byId[b.id] = b; });
    var engineIds = bd.engines.map(function (e) { return e.id; });
    var extraIds = bd.blocks.filter(function (b) { return b.kind === 'extra'; }).map(function (b) { return b.id; });
    var maxA = 1;
    bd.blocks.forEach(function (b) { var a = areaOf(b); if (a && a.nom > maxA) maxA = a.nom; });
    var wOf = function (b) { var a = areaOf(b); return (!b.enabled || !a) ? W_MIN : W_MIN + W_VAR * Math.sqrt(a.nom / maxA); };
    var prev = null, boxes = '', wires = '';
    DIAGRAM_ROWS.forEach(function (row) {
      var ids = [];
      row.forEach(function (r) { if (r === '@engines') ids = ids.concat(engineIds); else if (r === '@extras') ids = ids.concat(extraIds); else ids.push(r); });
      var items = ids.map(function (id) { return byId[id]; }).filter(Boolean);
      if (!items.length) return;
      var subs = [[]], used = 0;
      items.forEach(function (b) {
        var w = wOf(b) + GAP;
        if (used + w > DW - 8 && subs[subs.length - 1].length) { subs.push([]); used = 0; }
        subs[subs.length - 1].push(b); used += w;
      });
      subs.forEach(function (sub) {
        var total = sub.reduce(function (t, b) { return t + wOf(b) + GAP; }, -GAP), x = Math.max(4, (DW - total) / 2), centers = [];
        sub.forEach(function (b) {
          var w = wOf(b), on = b.enabled, bind = !!(on && cell && cell.binding && (b.limits || []).indexOf(cell.binding) >= 0);
          var chipDrv = !!(on && driver && (b.chip_items || []).indexOf(driver) >= 0), tx = boxLines(b);
          var tip = tx.join('; ') + (on ? '' : ' (off)') + (bind ? '; holds the binding limit (' + cell.binding + ') of the focused protocol' : '') +
            (chipDrv ? '; holds the chip-check driver (' + driver + ')' : '');
          boxes += '<g class="blk b-' + esc(b.kind) + (on ? '' : ' off') + (bind ? ' binding' : '') + (chipDrv ? ' chip-driver' : '') + '" tabindex="0" role="button"' +
            U.sideAttr({ kind: 'block', id: b.id, col: 0 }) + ' data-block="' + esc(b.id) + '" data-enabled="' + (on ? 1 : 0) + '"><title>' + esc(tip) + '</title>' +
            '<rect x="' + x + '" y="' + y + '" width="' + w + '" height="' + BOX_H + '" rx="3"/>';
          tx.forEach(function (t, ti) {
            if (ti > 2) return;
            var maxc = Math.floor(w / 5.4);
            boxes += '<text x="' + (x + 4) + '" y="' + (y + 11 + ti * 11) + '"' + (ti === 0 ? ' font-weight="700"' : '') + '>' + esc(t.length > maxc ? t.slice(0, maxc - 1) + '..' : t) + '</text>';
          });
          var tags = (on ? '' : 'off ') + (bind ? 'binding ' : '') + (chipDrv ? 'chip' : '');
          if (tags.trim()) boxes += '<text class="tag" x="' + (x + w - 3) + '" y="' + (y + BOX_H - 4) + '" text-anchor="end">' + esc(tags.trim()) + '</text>';
          boxes += '</g>';
          centers.push(x + w / 2); x += w + GAP;
        });
        if (prev) {
          var from = prev[Math.floor(prev.length / 2)], to = centers[Math.floor(centers.length / 2)];
          wires += '<path class="wire" d="M' + from + ' ' + (y - ROW_GAP) + ' V' + (y - ROW_GAP / 2) + ' H' + to + ' V' + y + '"/>';
        }
        prev = centers; y += BOX_H + ROW_GAP;
      });
    });
    var H = y + 8;
    s += wires + boxes + '<text class="foot" x="' + (DW - 4) + '" y="' + (H - 2) + '" text-anchor="end">width ~ sqrt(area), not a floorplan</text>';
    return '<div class="cfg-diagram"><h2>Live block diagram</h2><div class="dgm">' +
      U.svgWrap(DW, H + 4, s, 'Block diagram of the live architecture, box width proportional to the square root of area') + '</div>' +
      '<p class="muted cfg-note">Boxes follow the knobs; click one for its detail. Dashed: off. Outlined: holds the binding limit of ' +
      esc(U.wlName(state.data, state.focus.wl)) + ' (<b>binding</b>) or the chip-check driver (<b>chip</b>).</p></div>';
  }

  // Preset row + knob groups (v1 name kept: test_html and the PLAN-v2 stub contract).
  function renderKnobPanel(state, open) { return renderPresets(state) + renderGroups(state, open); }

  // The sections that depend on the live point (refreshed by mount()).
  function renderLive(state) { return renderPoints(state) + renderDiagram(state); }

  function renderRail(state, ctx) {
    return '<div class="v-configure">' + renderPresets(state) + '<div class="cfg-live" data-cfg-live>' + renderLive(state) + '</div>' +
      '<h2>Knobs</h2>' + renderGroups(state, ctx && ctx.open) + '</div>';
  }

  // ------------------------------------------------------------ side kind: block
  function sideBlock(state, ref) {
    var col = ref.col || 0, p = state.points[col] || state.points[0];
    var bd = A.blockDiagram(p.arch, state.data), b = bd.blocks.filter(function (x) { return x.id === ref.id; })[0];
    if (!b) return { title: ref.id, html: '<p class="muted">No block <code>' + esc(ref.id) + '</code> on this point.</p>' };
    var cell = col === state.focus.col ? state.cell : (p.all.cells[state.focus.wl] || A.evaluateCell(p.arch, state.focus.wl, state.data, { derived: p.derived }));
    var h = '<div class="mx-rec"><div><b>' + esc(b.label) + '</b> ' + (b.enabled ? '' : '<span class="badge">off</span>') + '</div>';
    var cfg = boxLines(b).slice(1);
    if (b.kind === 'link' && b.link) cfg = [b.link.kind || '', 'write ' + fmtNum(b.link.write_MBps.nom, 3) + ' / read ' + fmtNum(b.link.read_MBps.nom, 3) + ' MB/s'];
    if (cfg.length) h += '<div class="muted">' + cfg.map(esc).join('; ') + '</div>';
    var a = areaOf(b);
    if (a) h += '<p>placed area ' + U.fmtBandHtml(a, 'um2', 0) + ', ' + fmtNum(b.pct_core, 2) + ' % of core. ' +
      '<button type="button" class="linklike" data-tab="area">area &amp; pins</button></p>';
    h += '<h3>limits this block takes part in (for ' + esc(U.wlName(state.data, state.focus.wl)) + ')</h3>';
    var lims = (b.limits || []);
    h += lims.length ? '<ul>' + lims.map(function (lid) {
      var lim = cell && cell.limits.filter(function (x) { return x.id === lid; })[0];
      return '<li><code>' + esc(lid) + '</code>' + (lim ? ' ' + esc(U.fmtBand(lim.value, lim.unit)) : '') + (cell && cell.binding === lid ? ' <b>(binding)</b>' : '') + '</li>';
    }).join('') + '</ul>' : '<p class="muted">none (a passive or external block).</p>';
    var cis = (b.chip_items || []);
    if (cis.length) h += '<h3>chip-check items</h3><ul>' + cis.map(function (id) {
      var it = p.chip.items.filter(function (q) { return q.id === id; })[0];
      return '<li><button type="button" class="linklike"' + U.sideAttr({ kind: 'chip', item: id, col: col }) + '>' + esc(it ? it.short : id) + '</button> ' + (it ? U.chipBadge(it.verdict) : '') + '</li>';
    }).join('') + '</ul>';
    return { title: b.label, html: h + '</div>' };
  }
  U.registerSide('block', sideBlock);

  // ------------------------------------------------------------ DOM glue (browser only)
  var LAST_PRESET = null;
  // Registry update(state) hook: the core calls it after every state change (the rail itself is repainted only on rebuild).
  function update(st) {
    if (typeof document === 'undefined') return;
    var root = document.getElementById('rail'), box = root && root.querySelector('[data-cfg-live]');
    if (!st || !box) return;
    try {
      box.innerHTML = renderLive(st);
      var full = st.pinned.length >= 5;
      ['pin', 'pin-preset'].forEach(function (a) { var b = root.querySelector('[data-action="' + a + '"]'); if (b) b.disabled = full; });
      var sel = root.querySelector('select[data-preset]');
      if (sel && LAST_PRESET !== st.preset) { sel.value = st.preset || ''; LAST_PRESET = st.preset; }
    } catch (e) { box.innerHTML = U.errorBox('configure live section', e); }
  }
  function mount(rail, st) {
    if (typeof document === 'undefined') return;
    LAST_PRESET = st ? st.preset : null;
  }

  U.renderKnobPanel = renderKnobPanel;
  U.renderPresetSelector = renderPresets;
  U.renderPoints = renderPoints;
  U.renderDiagram = renderDiagram;
  U.renderChip = renderChip;
  U.renderConfigure = renderRail;
  U.register('configure', {
    title: 'Configure',
    render: function (s, ctx) { return U.renderConfigure(s, ctx); },
    renderChip: function (s) { return U.renderChip(s); },
    mount: mount,
    update: update
  });
})(ArchSimUI);
