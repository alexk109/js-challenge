// archsim model (Tasks 4-6: core, limits, solvers and calibration). Dependency-free, deterministic, no DOM, no clock, no randomness.
// Formulas and structure live here; every number a data file can carry comes from `data` (inputs.json).
// Units: time ns, clocks MHz, area um2, bandwidth MB/s (= bytes/us), sizes bytes or words. PLAN Part 2.
var ArchSim = (function () {
  'use strict';
  var VERSION = '2';

  // ---------------------------------------------------------------- JSON helper (PLAN 4.4)
  // Maps +-Infinity to strings so JSON.stringify round-trips.
  function toJSON(x) {
    if (typeof x === 'number') {
      if (x === Infinity) return 'Infinity';
      if (x === -Infinity) return '-Infinity';
      return x;
    }
    if (Array.isArray(x)) return x.map(toJSON);
    if (x && typeof x === 'object') {
      var o = {};
      Object.keys(x).forEach(function (k) { o[k] = toJSON(x[k]); });
      return o;
    }
    return x;
  }

  // ---------------------------------------------------------------- knobs (PLAN 2.1)
  // Appendix D lists the same defaults for the `mvp` preset; defaultArch() is built from KNOBS.
  var GROUPS = ['Clock & timing', 'Engines & memory', 'I/O & host', 'Die', 'Environment', 'ISA'];
  // Identifiers of the area-only blocks of Appendix B.5 (the areas themselves are in data.blocks).
  var EXTRA_IDS = ['echo_comparator', 'pin_shadow_witness24', 'replay_module_18', 'fc_executor',
    'sched_shared_datapath', 'fc_table_sram_512B'];

  function num(id, label, g, def, min, max, step, unit, help, quick) {
    var k = { id: id, label: label, type: 'number', min: min, max: max, step: step, default: def, unit: unit, group: g, help: help };
    if (quick) k.quick = quick;
    return k;
  }
  function en(id, label, g, def, options, unit, help) {
    return { id: id, label: label, type: 'enum', options: options, default: def, unit: unit, group: g, help: help };
  }
  function bool(id, label, g, def, help) {
    return { id: id, label: label, type: 'bool', default: def, unit: '', group: g, help: help };
  }
  var KNOBS = [
    num('f_core_mhz', 'Core clock', GROUPS[0], 50, 10, 80, 0.5, 'MHz', 'Supplied by the board MCU (exact at 48/50/60).', [48, 50, 60]),
    en('timing_lever', 'Timing levers applied', GROUPS[0], 'none', ['none', 'tool', 'rtl', 'both'], '', 'Selects the fmax band of Appendix B.2.'),
    en('signoff_corner', 'Sign-off corner for the core-clock limit', GROUPS[0], 'typ', ['typ', 'slow'], '', 'typ is the working sign-off; the slow corner is always shown.'),
    bool('ddr_io', 'Both-edge I/O (DDR)', GROUPS[0], false, 'Halves the TX and RX grid.'),
    en('fine_edge', 'Delay-line edge placement (DTC/TDC)', GROUPS[0], 'none', ['none', 'dtc'], '', 'Analog-gated (kill audit K30).'),
    num('n_engines', 'Engines', GROUPS[1], 2, 1, 6, 1, '', 'Number of PIO engines.'),
    en('imem_words', 'IMEM words per engine (per bank)', GROUPS[1], 32, [32, 64, 128, 256, 512], '16-bit words', ''),
    en('imem_tech', 'IMEM technology', GROUPS[1], 'flop', ['flop', 'latch', 'sram2p'], '', 'sram2p is a two-port macro (a 1RW pair is behaviourally wrong).'),
    en('imem_paging', 'IMEM paging', GROUPS[1], 'none', ['none', 'bank_switch', 'host_reload'], '', ''),
    num('imem_banks', 'IMEM banks (bank_switch)', GROUPS[1], 1, 1, 8, 1, '', 'Physical words = imem_words * imem_banks.'),
    num('fifo_depth', 'Flop FIFO depth (each TX and RX FIFO)', GROUPS[1], 4, 2, 16, 1, '32-bit words', 'Per FIFO, every engine.'),
    en('deep_fifo_words', 'Deep SRAM-backed FIFO', GROUPS[1], 0, [0, 64, 256, 512, 1024, 2048, 4096], '32-bit words', 'One extra SRAM FIFO behind the flop FIFO.'),
    en('deep_fifo_dir', 'Deep FIFO direction', GROUPS[1], 'tx', ['tx', 'rx', 'both'], '', 'both = two macros.'),
    en('streaming', 'Instruction streaming via OUT EXEC', GROUPS[1], 'off', ['off', 'out_exec'], '', ''),
    bool('crc_unit', 'CRC/LFSR unit per engine', GROUPS[1], false, ''),
    en('host_link', 'Host link', GROUPS[2], 'spi', ['spi', 'spi_fast', 'qspi', 'nibble', 'byte', 'byte_load_only'], '', 'Table B.3.'),
    num('pins_reserved', 'Pins reserved (debug, IRQ, spare)', GROUPS[2], 0, 0, 6, 1, 'pins', ''),
    bool('lane', 'Bus-clocked lane (R1)', GROUPS[2], false, 'Serves bus_clock_in targets with the lane reflex.'),
    en('capture', 'RX capture assist', GROUPS[2], 'none', ['none', 'dpll', 'adpll'], '', ''),
    en('adpll_mult', 'ADPLL multiplier N', GROUPS[2], 4, [4, 6, 8], '', 'Used when capture = adpll.'),
    en('die', 'Tiles', GROUPS[3], '6x4', ['6x4', '8x4'], '', '8x4 unconfirmed for the competition.'),
    { id: 'extras', label: 'Area-only blocks', type: 'multi', options: EXTRA_IDS, default: [], unit: '', group: GROUPS[3], help: 'Table B.5.' },
    en('env', 'Environment', GROUPS[4], 'silicon', ['silicon', 'iss'], '', 'iss zeroes pad, skew, setup, phase and jitter.'),
    // ISA levers (PLAN-v2 2.4). Defaults are the M0 (MVP) instruction set; every effect is a judgement from inputs.json.isa.
    bool('isa_irq', 'IRQ flags and irq / wait irq', GROUPS[5], false, 'Restores the RP2040 IRQ instruction (major opcode 110) that the MVP removed. Estimate: inputs.json isa.'),
    bool('isa_xor', 'XOR / CRC-step operation', GROUPS[5], false, 'MOV op 11 (reserved in M0): dst ^ src in one SM cycle. Satisfies the CRC need at a cycle cost. Estimate.'),
    en('isa_word_bits', 'Instruction width', GROUPS[5], 16, [16, 24, 32], 'bits', '24 = dedicated side-set and wider delay; 32 = also 16-bit immediates. IMEM area scales with the width. Estimate.'),
    bool('isa_cond', 'More JMP conditions / compare', GROUPS[5], false, 'x == imm, x < y, pin edge, OSR count. A new major opcode at 16 bits. Estimate.'),
    bool('isa_mov_src', 'Wider MOV sources', GROUPS[5], false, 'FIFO levels, masked IN window (MOV source 100, reserved in M0). Estimate.')
  ];
  var ISA_BOOL_LEVERS = ['isa_irq', 'isa_xor', 'isa_cond', 'isa_mov_src'];
  // Not a UI knob, but part of the arch object (PLAN 2.2).
  var NON_KNOB_DEFAULTS = { param_overrides: {} };

  function clone(x) { return JSON.parse(JSON.stringify(x)); }

  function defaultArch() {
    var a = {};
    KNOBS.forEach(function (k) { a[k.id] = clone(k.default); });
    Object.keys(NON_KNOB_DEFAULTS).forEach(function (k) { a[k] = clone(NON_KNOB_DEFAULTS[k]); });
    return a;
  }

  function knobProblems(arch) {
    var bad = [];
    KNOBS.forEach(function (k) {
      var v = arch[k.id];
      if (k.type === 'number') {
        if (typeof v !== 'number' || !(v >= k.min && v <= k.max)) bad.push(k.id + '=' + v + ' outside ' + k.min + '..' + k.max);
      } else if (k.type === 'enum') {
        if (k.options.indexOf(v) < 0) bad.push(k.id + '=' + v + ' not in ' + k.options.join('|'));
      } else if (k.type === 'bool') {
        if (typeof v !== 'boolean') bad.push(k.id + ' must be boolean');
      } else if (k.type === 'multi') {
        if (!Array.isArray(v) || v.some(function (e) { return k.options.indexOf(e) < 0; })) bad.push(k.id + ' has unknown ids');
      }
    });
    return bad;
  }

  function normArch(arch) {
    var a = Object.assign(defaultArch(), arch || {});
    var bad = knobProblems(a);
    if (bad.length) throw new Error('invalid arch: ' + bad.join('; '));
    return a;
  }

  function applyPreset(presetId, data) {
    var p = (data.presets || []).filter(function (q) { return q.id === presetId; })[0];
    if (!p) throw new Error('unknown preset ' + presetId);
    return Object.assign(defaultArch(), clone(p.arch));
  }

  // ---------------------------------------------------------------- parameters (PLAN 2.2)
  // Parameters forced to 0 in the `iss` environment.
  var ISS_ZERO = ['t_pad_rt_ns', 't_skew_ns', 't_setup_ext_ns', 'sync_phase_cyc', 't_clk_jitter_ns'];

  // Resolved parameters {id: {lo, nom, hi, ...meta}}: data, then param_overrides, then env (forced).
  function resolveParams(arch, data) {
    var a = Object.assign(defaultArch(), arch || {});
    var out = {};
    Object.keys(data.params).forEach(function (id) { out[id] = clone(data.params[id]); });
    var ov = a.param_overrides || {};
    Object.keys(ov).forEach(function (id) {
      if (!out[id]) throw new Error('override of unknown parameter ' + id);
      var p = out[id], o = ov[id];
      ['lo', 'nom', 'hi'].forEach(function (f) { if (typeof o[f] === 'number') p[f] = o[f]; });
      if (typeof o.nom === 'number' && typeof o.lo !== 'number') p.lo = Math.min(p.lo, p.nom);
      if (typeof o.nom === 'number' && typeof o.hi !== 'number') p.hi = Math.max(p.hi, p.nom);
      if (!(p.lo <= p.nom && p.nom <= p.hi)) throw new Error('override of ' + id + ' violates lo <= nom <= hi');
      p.overridden = true;
    });
    if (a.env === 'iss') {
      ISS_ZERO.forEach(function (id) {
        if (out[id]) { out[id].lo = 0; out[id].nom = 0; out[id].hi = 0; out[id].forced_by_env = 'iss'; }
      });
    }
    return out;
  }

  // ---------------------------------------------------------------- vertex intervals (PLAN 3.2)
  // inputs: {key: {lo, nom, hi}}; f(values) -> number; better: 'high' | 'low' (which direction is good).
  // Returns {nom, pess, opt, pess_direction: {key: 'lo'|'hi'|'none'}}; 2|U|+3 evaluations.
  function vertex(inputs, f, better) {
    var sgn = better === 'low' ? -1 : 1;
    var keys = Object.keys(inputs);
    var nomV = {}, pessV = {}, optV = {}, dir = {};
    keys.forEach(function (k) { nomV[k] = inputs[k].nom; });
    keys.forEach(function (k) {
      var u = inputs[k];
      var dirn = 'none';
      if (u.lo !== u.hi) {
        var vl = Object.assign({}, nomV), vh = Object.assign({}, nomV);
        vl[k] = u.lo; vh[k] = u.hi;
        var ml = sgn * f(vl), mh = sgn * f(vh);
        if (ml < mh) dirn = 'lo'; else if (mh < ml) dirn = 'hi';
      }
      dir[k] = dirn;
      pessV[k] = dirn === 'none' ? u.nom : u[dirn];
      optV[k] = dirn === 'none' ? u.nom : (dirn === 'lo' ? u.hi : u.lo);
    });
    return { nom: f(nomV), pess: f(pessV), opt: f(optV), pess_direction: dir };
  }

  function ivl(v) { return (v !== null && typeof v === 'object') ? { nom: v.nom, pess: v.pess, opt: v.opt } : v; }
  function nomOf(v) { return (v !== null && typeof v === 'object') ? v.nom : v; }
  function pick(params, ids) { var o = {}; ids.forEach(function (i) { o[i] = params[i]; }); return o; }
  function ivlFn(inputs, f, better) {  // plain number if nothing uncertain, else vertex interval
    return Object.keys(inputs).length ? vertex(inputs, f, better) : f({});
  }
  function ivlAlways(inputs, f, better) {
    var v = ivlFn(inputs, f, better);
    return typeof v === 'number' ? { nom: v, pess: v, opt: v, pess_direction: {} } : v;
  }

  // ---------------------------------------------------------------- derivation records (PLAN 3.1)
  function rec(id, label, v, unit, formula, terms, note) {
    var value = (v !== null && typeof v === 'object' && 'pess' in v) ? ivl(v) : v;
    return { id: id, label: label, value: value, unit: unit, formula: formula, method: 'formula', terms: terms,
      pess_direction: (v && v.pess_direction) || {}, note: note || '' };
  }
  function term(sym, value, unit, ref) { return { sym: sym, value: value, unit: unit, ref: ref }; }
  function pT(sym, params, id, dirs) {  // parameter term with its pessimistic / optimistic ends
    var p = params[id], d = (dirs || {})[id];
    var val = p.lo === p.hi ? p.nom : { nom: p.nom, pess: d === 'none' || !d ? p.nom : p[d], opt: d === 'none' || !d ? p.nom : (d === 'lo' ? p.hi : p.lo) };
    return term(sym, val, p.unit, { kind: 'param', id: id });
  }
  function kT(sym, arch, id, unit) { return term(sym, arch[id], unit || '', { kind: 'knob', id: id }); }
  function dT(sym, rid, value, unit) { return term(sym, value, unit, { kind: 'derived', id: rid }); }

  // ---------------------------------------------------------------- ISA levers (PLAN-v2 2.4)
  function knobDefault(id) { return KNOBS.filter(function (k) { return k.id === id; })[0].default; }
  // The lever ids switched on in `arch` (bool levers that are true, isa_word_bits when wider than the M0 width).
  function isaActiveLevers(arch) {
    var out = ISA_BOOL_LEVERS.filter(function (id) { return arch[id]; });
    if (arch.isa_word_bits > knobDefault('isa_word_bits')) out.push('isa_word_bits');
    return out;
  }
  var ISA_FIELDS = ['ipu', 'words', 'reflex_sm', 'decide_sm'];
  // The effect rows of `wlId` under `arch`, summed per field: {rows, d_ipu, d_words, d_reflex_sm, d_decide_sm, enables, levers}.
  // isa_word_bits: the row with the largest value not above the chosen width (32 uses the 32 row, else the 24 row).
  // An isa_xor row that enables the CRC need is not paid when crc_unit does the CRC (the unit is assumed to keep up).
  function isaOf(arch, wlId, data) {
    var levers = isaActiveLevers(arch), all = ((data.isa || {}).effects || []), rows = [];
    var bestBits = {};
    all.forEach(function (e) {
      if (e.wl === wlId && e.lever === 'isa_word_bits' && levers.indexOf(e.lever) >= 0 && e.value <= arch.isa_word_bits && !(bestBits[e.wl] >= e.value)) bestBits[e.wl] = e.value;
    });
    all.forEach(function (e) {
      if (e.wl !== wlId || levers.indexOf(e.lever) < 0) return;
      if (e.lever === 'isa_word_bits' && e.value !== bestBits[e.wl]) return;
      if (e.lever === 'isa_xor' && arch.crc_unit && (e.enables || []).indexOf('isa_xor') >= 0) return;
      rows.push(e);
    });
    var o = { levers: levers, rows: rows, enables: [] };
    ISA_FIELDS.forEach(function (f) {
      var b = { lo: 0, nom: 0, hi: 0 };
      rows.forEach(function (e) { ['lo', 'nom', 'hi'].forEach(function (k) { b[k] += e['d_' + f][k]; }); });
      o['d_' + f] = b;
    });
    rows.forEach(function (e) { (e.enables || []).forEach(function (x) { if (o.enables.indexOf(x) < 0) o.enables.push(x); }); });
    return o;
  }
  // Pseudo-parameter for the band of an ISA delta (only when the band is not a point); isaD reads it back at a vertex.
  function isaIn(c, f) {
    var b = c.isa['d_' + f], o = {};
    if (b.lo !== b.hi) o['isa_d_' + f] = { lo: b.lo, nom: b.nom, hi: b.hi };
    return o;
  }
  function isaD(c, f, v) { var k = 'isa_d_' + f; return (v && k in v) ? v[k] : c.isa['d_' + f].nom; }
  // One term per effect row that moves field f, each with ref isa_effect (the evidence rule and the UI follow it).
  function isaT(c, f, V, unit) {
    var key = 'isa_d_' + f, dir = V && V.pess_direction ? V.pess_direction[key] : null;
    return c.isa.rows.filter(function (e) { var b = e['d_' + f]; return b.lo !== 0 || b.nom !== 0 || b.hi !== 0; }).map(function (e) {
      var b = e['d_' + f];
      var val = b.lo === b.hi ? b.nom : { nom: b.nom, pess: dir === 'lo' ? b.lo : (dir === 'hi' ? b.hi : b.nom), opt: dir === 'lo' ? b.hi : (dir === 'hi' ? b.lo : b.nom) };
      var ref = { kind: 'isa_effect', wl: e.wl, lever: e.lever };
      if (e.value !== undefined && e.value !== null) ref.value = e.value;
      return term('d_' + f + ' (' + e.lever + ')', val, unit, ref);
    });
  }

  function fieldIn(o, path) {  // 'framing.cmd_bits' resolves inside o
    return path.split('.').every(function (k) { var ok = o !== null && typeof o === 'object' && k in o; if (ok) o = o[k]; return ok; });
  }

  // Does a term/limit reference resolve? (used by tests; PLAN 3.1)
  function refResolves(ref, data, derivedIds) {
    switch (ref.kind) {
      case 'param': return !!data.params[ref.id];
      case 'block': return !!data.blocks[ref.id] && (!ref.field || ref.field in data.blocks[ref.id]);
      case 'link': return !!data.links[ref.id] && (!ref.field || fieldIn(data.links[ref.id], ref.field));
      case 'sram_macro': return data.sram_macros.some(function (m) { return m.name === ref.id; });
      case 'iss': return ref.id in data.iss && (!ref.field || fieldIn(data.iss[ref.id], ref.field));
      case 'knob': return KNOBS.some(function (k) { return k.id === ref.id; });
      case 'workload_field': return data.workloads.some(function (w) { return w.id === ref.wl && (ref.field in (w.src || {}) || (ref.default === true && ref.field in w)); });
      case 'derived': return !derivedIds || derivedIds.indexOf(ref.id) >= 0;
      case 'isa_effect': return ((data.isa || {}).effects || []).some(function (e) { return e.wl === ref.wl && e.lever === ref.lever && (ref.value === undefined || e.value === ref.value); });
      case 'isa_encoding': return !!((data.isa || {}).encoding || {})[ref.bits];
      default: return false;
    }
  }

  // ---------------------------------------------------------------- host link (PLAN 2.3, B.3)
  // Wire framing of the SPI family and the parallel loop cycles are data: links.<id>.framing (extract/links.py, with sources).
  // Serial: cmd_bits command bits, wr_bits per word written, rd_bits (turnaround + data) per word read; cs_cycles core
  // cycles each for CSn fall and rise, gap_cycles for the CSn-high gap, access_cycles in the bus-access cycle of the last
  // word (k/2 of the SCK bits are still to come). Parallel: loop_pio_cycles.
  function fr(link, k) {
    if (!link.framing || !(k in link.framing)) throw new Error('link ' + link.id + ' has no framing.' + k);
    return link.framing[k];
  }
  function fT(link, sym, k, unit) { return term(sym, fr(link, k), unit || 'cycles', { kind: 'link', id: link.id, field: 'framing.' + k }); }

  function isSerial(link) { return link.kind === 'spi' || link.kind === 'qspi'; }
  function writeWindow(link, n) {
    return fr(link, 'cs_cycles') + link.k_w * (fr(link, 'cmd_bits') + fr(link, 'wr_bits') * n) / link.width_bits + fr(link, 'cs_cycles');
  }
  function readWindow(link, n) {
    return fr(link, 'cs_cycles') + link.k_r * (fr(link, 'cmd_bits') + fr(link, 'rd_bits') * n) / link.width_bits + fr(link, 'cs_cycles');
  }
  // Bus access cycle of the last of n words written (321 at n = 1 for SPI k = 8).
  function hWrite(link, n) {
    return link.k_w * (fr(link, 'cmd_bits') + fr(link, 'wr_bits') * n) / link.width_bits - link.k_w / 2 + fr(link, 'access_cycles');
  }

  // ---------------------------------------------------------------- SRAM macro choice (PLAN 2.3)
  // The block of memory family `family` ('imem_flop' | 'imem_latch' | 'fifo') with role 'anchor' (the measured point the
  // area scales from) or 'slope' (um2 per extra word); words and bits are fields of the block (extract/blocks.py).
  function familyBlock(data, family, role) {
    var hit = Object.keys(data.blocks).filter(function (id) { var b = data.blocks[id]; return b.family === family && b.role === role; });
    if (hit.length !== 1) throw new Error('expected one ' + family + ' ' + role + ' block, found ' + hit.length);
    return data.blocks[hit[0]];
  }

  // Smallest total area among macros with `ports` ports and words >= minWords, ceil(widthBits / bits) copies.
  function pickMacro(data, ports, minWords, widthBits) {
    var best = null;
    data.sram_macros.forEach(function (m) {
      if (m.ports !== ports || m.words < minWords) return;
      var copies = Math.ceil(widthBits / m.bits), area = copies * m.area_um2;
      if (best === null || area < best.area) best = { name: m.name, copies: copies, area: area, words: m.words, bits: m.bits };
    });
    if (!best) throw new Error('no ' + ports + '-port SRAM macro with ' + minWords + ' words');
    return best;
  }

  // ---------------------------------------------------------------- derive (PLAN 2.3)
  function derive(archIn, data) {
    var arch = normArch(archIn);
    var P = resolveParams(arch, data);
    var R = {};  // records by id
    function add(r) { R[r.id] = r; return r.value; }
    var link = data.links[arch.host_link];
    if (!link) throw new Error('unknown link ' + arch.host_link);
    var E = arch.n_engines;

    // ---- clock grid
    var T = 1000 / arch.f_core_mhz;
    add(rec('T', 'Core clock period', T, 'ns', '1000 / f_core_mhz', [kT('f_core_mhz', arch, 'f_core_mhz', 'MHz')]));
    var gridIds = arch.fine_edge === 'dtc' ? ['tap_ns'] : [];
    var txFn = function (v) {
      if (arch.fine_edge === 'dtc') return v.tap_ns;
      return arch.ddr_io ? T / 2 : T;
    };
    var rxFn = function (v) {
      if (arch.fine_edge === 'dtc') return v.tap_ns;
      var base = arch.ddr_io ? T / 2 : T;
      return arch.capture === 'adpll' ? base / arch.adpll_mult : base;
    };
    var txV = ivlFn(pick(P, gridIds), txFn, 'low');
    var T_tx = add(rec('T_tx', 'TX edge grid', txV, 'ns', 'dtc ? tap_ns : (ddr_io ? T / 2 : T)',
      [kT('ddr_io', arch, 'ddr_io'), kT('fine_edge', arch, 'fine_edge'), dT('T', 'T', T, 'ns')].concat(arch.fine_edge === 'dtc' ? [pT('tap_ns', P, 'tap_ns', txV.pess_direction)] : [])));
    var rxV = ivlFn(pick(P, gridIds), rxFn, 'low');
    var T_rx = add(rec('T_rx', 'RX capture grid', rxV, 'ns', 'dtc ? tap_ns : T_tx / (capture = adpll ? N : 1)',
      [kT('capture', arch, 'capture'), kT('adpll_mult', arch, 'adpll_mult'), dT('T_tx', 'T_tx', nomOf(T_tx), 'ns')].concat(arch.fine_edge === 'dtc' ? [pT('tap_ns', P, 'tap_ns', rxV.pess_direction)] : [])));

    // ---- buffers
    var fifo_bytes = 4 * arch.fifo_depth;
    var deepTx = arch.deep_fifo_dir === 'tx' || arch.deep_fifo_dir === 'both';
    var deepRx = arch.deep_fifo_dir === 'rx' || arch.deep_fifo_dir === 'both';
    var buf_tx = fifo_bytes + (deepTx ? 4 * arch.deep_fifo_words : 0);
    var buf_rx = fifo_bytes + (deepRx ? 4 * arch.deep_fifo_words : 0);
    add(rec('fifo_bytes', 'Flop FIFO bytes (each)', fifo_bytes, 'B', '4 * fifo_depth', [kT('fifo_depth', arch, 'fifo_depth', 'words')]));
    add(rec('buf_tx_bytes', 'TX buffer bytes', buf_tx, 'B', 'fifo_bytes + (deep_fifo_dir in {tx, both} ? 4 * deep_fifo_words : 0)',
      [dT('fifo_bytes', 'fifo_bytes', fifo_bytes, 'B'), kT('deep_fifo_words', arch, 'deep_fifo_words', 'words'), kT('deep_fifo_dir', arch, 'deep_fifo_dir')]));
    add(rec('buf_rx_bytes', 'RX buffer bytes', buf_rx, 'B', 'fifo_bytes + (deep_fifo_dir in {rx, both} ? 4 * deep_fifo_words : 0)',
      [dT('fifo_bytes', 'fifo_bytes', fifo_bytes, 'B'), kT('deep_fifo_words', arch, 'deep_fifo_words', 'words'), kT('deep_fifo_dir', arch, 'deep_fifo_dir')]));

    // ---- host link
    var Bw = arch.fifo_depth + (deepTx ? arch.deep_fifo_words : 0);
    var Br = arch.fifo_depth + (deepRx ? arch.deep_fifo_words : 0);
    var L = { id: link.id, kind: link.kind, width_bits: link.width_bits, k_w: link.k_w, k_r: link.k_r, B_w: Bw, B_r: Br };
    var lref = { kind: 'link', id: link.id };
    var wMB, rMB, loop, load;
    var lf = { w: null, r: null, loop: null, load: null, wIds: [], rIds: [], loopIds: [], loadIds: [] };  // at-vertex closures for the limits
    var serialTerms = function (dirs, extra) {
      return [term('k', link.k_w, 'cycles/SCK bit', lref), kT('fifo_depth', arch, 'fifo_depth', 'words'),
        dT('T', 'T', T, 'ns')].concat(extra);
    };
    if (isSerial(link)) {
      var txnIds = ['t_mcu_txn_ns'], swIds = ['t_mcu_sw_ns'], gapC = fr(link, 'gap_cycles');
      L.write_window_1 = writeWindow(link, 1); L.write_window_B = writeWindow(link, Bw);
      L.read_window_1 = readWindow(link, 1); L.read_window_B = readWindow(link, Br);
      L.h_write_1 = hWrite(link, 1);
      lf.w = function (v) { return 4 * Bw / ((L.write_window_B + gapC) * T + v.t_mcu_txn_ns) * 1000; };
      lf.r = function (v) { return 4 * Br / ((L.read_window_B + gapC) * T + v.t_mcu_txn_ns) * 1000; };
      lf.wIds = lf.rIds = txnIds; lf.loopIds = lf.loadIds = swIds;
      wMB = vertex(pick(P, txnIds), lf.w, 'high');
      rMB = vertex(pick(P, txnIds), lf.r, 'high');
      add(rec('link_write_MBps', 'Host link write bandwidth', wMB, 'MB/s',
        '4 B / ((write_window(B) + 2) * T + t_mcu_txn) * 1000; write_window(n) = 2 + k_w * (8 + 32 n) / width + 2',
        serialTerms(wMB.pess_direction, [pT('t_mcu_txn', P, 't_mcu_txn_ns', wMB.pess_direction), fT(link, 'cmd_bits', 'cmd_bits', 'bits'), fT(link, 'wr_bits', 'wr_bits', 'bits'),
          fT(link, 'cs_cycles', 'cs_cycles'), fT(link, 'gap_cycles', 'gap_cycles'), term('width', link.width_bits, 'bits', lref)]), 'burst B = flop FIFO (+ deep FIFO if TX)'));
      add(rec('link_read_MBps', 'Host link read bandwidth', rMB, 'MB/s',
        '4 B / ((read_window(B) + 2) * T + t_mcu_txn) * 1000; read_window(n) = 2 + k_r * (8 + 40 n) / width + 2',
        serialTerms(rMB.pess_direction, [pT('t_mcu_txn', P, 't_mcu_txn_ns', rMB.pess_direction), fT(link, 'cmd_bits', 'cmd_bits', 'bits'), fT(link, 'rd_bits', 'rd_bits', 'bits'),
          fT(link, 'cs_cycles', 'cs_cycles'), fT(link, 'gap_cycles', 'gap_cycles'), term('width', link.width_bits, 'bits', lref)]), 'burst B = flop FIFO (+ deep FIFO if RX)'));
      lf.loop = function (v) { return v.t_mcu_sw_ns + (L.read_window_1 + L.write_window_1) * T; };
      lf.load = function (w, v) { return hWrite(link, w) * T + v.t_mcu_sw_ns; };
      loop = vertex(pick(P, swIds), lf.loop, 'low');
      add(rec('t_host_loop', 'Host decision loop', loop, 'ns', 't_mcu_sw + (read_window(1) + write_window(1)) * T',
        [pT('t_mcu_sw', P, 't_mcu_sw_ns', loop.pess_direction), dT('T', 'T', T, 'ns'), term('k', link.k_w, 'cycles/SCK bit', lref), fT(link, 'cmd_bits', 'cmd_bits', 'bits'),
          fT(link, 'wr_bits', 'wr_bits', 'bits'), fT(link, 'rd_bits', 'rd_bits', 'bits'), fT(link, 'cs_cycles', 'cs_cycles')]));
      load = vertex(pick(P, swIds), function (v) { return lf.load(data.iss.IMEM_SIZE, v); }, 'low');
      add(rec('t_imem_load_32', 'IMEM page load (IMEM_SIZE words)', load, 'ns', 'h_write(w) * T + t_mcu_sw',
        [term('w', data.iss.IMEM_SIZE, 'words', { kind: 'iss', id: 'IMEM_SIZE' }), dT('T', 'T', T, 'ns'), pT('t_mcu_sw', P, 't_mcu_sw_ns', load.pess_direction),
          term('k', link.k_w, 'cycles/SCK bit', lref), fT(link, 'cmd_bits', 'cmd_bits', 'bits'), fT(link, 'wr_bits', 'wr_bits', 'bits'), fT(link, 'access_cycles', 'access_cycles')]));
    } else if (link.kind === 'parallel') {
      lf.w = lf.r = function (v) { return arch.f_core_mhz * link.width_bits / 8 * v.eta_par; };
      lf.wIds = lf.rIds = ['eta_par'];
      lf.loop = function (v) { return v.t_mcu_pio_ns + fr(link, 'loop_pio_cycles') * T; };
      lf.load = function (w, v) { return w * 2 / (arch.f_core_mhz * link.width_bits / 8 * v.eta_par) * 1000 + v.t_mcu_pio_ns; };
      lf.loopIds = ['t_mcu_pio_ns']; lf.loadIds = ['t_mcu_pio_ns', 'eta_par'];
      wMB = vertex(pick(P, ['eta_par']), lf.w, 'high'); rMB = vertex(pick(P, ['eta_par']), lf.r, 'high');
      var pf = 'f_core_mhz * width / 8 * eta_par';
      var pTerms = function (d) { return [kT('f_core_mhz', arch, 'f_core_mhz', 'MHz'), term('width', link.width_bits, 'bits', lref), pT('eta_par', P, 'eta_par', d)]; };
      add(rec('link_write_MBps', 'Host link write bandwidth', wMB, 'MB/s', pf, pTerms(wMB.pess_direction)));
      add(rec('link_read_MBps', 'Host link read bandwidth', rMB, 'MB/s', pf, pTerms(rMB.pess_direction)));
      loop = vertex(pick(P, ['t_mcu_pio_ns']), lf.loop, 'low');
      add(rec('t_host_loop', 'Host decision loop', loop, 'ns', 't_mcu_pio + loop_pio_cycles * T', [pT('t_mcu_pio', P, 't_mcu_pio_ns', loop.pess_direction), fT(link, 'loop_pio_cycles', 'loop_pio_cycles'), dT('T', 'T', T, 'ns')]));
      load = vertex(pick(P, ['t_mcu_pio_ns', 'eta_par']), function (v) { return lf.load(data.iss.IMEM_SIZE, v); }, 'low');
      add(rec('t_imem_load_32', 'IMEM page load (IMEM_SIZE words)', load, 'ns', 'w * 2 / write_MBps * 1000 + t_mcu_pio',
        [term('w', data.iss.IMEM_SIZE, 'words', { kind: 'iss', id: 'IMEM_SIZE' }), dT('write_MBps', 'link_write_MBps', nomOf(wMB), 'MB/s'), pT('t_mcu_pio', P, 't_mcu_pio_ns', load.pess_direction)]));
    } else {  // load_only: pins are handed to protocols in run mode
      lf.w = lf.r = function () { return 0; };
      lf.loop = function () { return Infinity; };
      lf.load = function () { return Infinity; };
      wMB = rMB = { nom: 0, pess: 0, opt: 0, pess_direction: {} };
      loop = { nom: Infinity, pess: Infinity, opt: Infinity, pess_direction: {} };
      load = null;
      add(rec('link_write_MBps', 'Host link write bandwidth', 0, 'MB/s', '0 (pins handed to protocols in run mode)', [term('link', link.id, '', lref)]));
      add(rec('link_read_MBps', 'Host link read bandwidth', 0, 'MB/s', '0 (pins handed to protocols in run mode)', [term('link', link.id, '', lref)]));
      add(rec('t_host_loop', 'Host decision loop', Infinity, 'ns', 'Infinity (no run-mode host)', [term('link', link.id, '', lref)]));
    }
    var bothV = (wMB.nom > 0 && rMB.nom > 0)
      ? { nom: 1 / (1 / wMB.nom + 1 / rMB.nom), pess: 1 / (1 / wMB.pess + 1 / rMB.pess), opt: 1 / (1 / wMB.opt + 1 / rMB.opt), pess_direction: {} }
      : { nom: 0, pess: 0, opt: 0, pess_direction: {} };
    add(rec('link_both_MBps', 'Host link bandwidth, half duplex', bothV, 'MB/s', '1 / (1 / write_MBps + 1 / read_MBps)',
      [dT('write_MBps', 'link_write_MBps', ivl(wMB), 'MB/s'), dT('read_MBps', 'link_read_MBps', ivl(rMB), 'MB/s')],
      'vertices combine like with like (pessimistic with pessimistic)'));
    L.write_MBps = ivl(wMB); L.read_MBps = ivl(rMB); L.both_MBps = ivl(bothV);
    L.t_host_loop = ivl(loop); L.t_imem_load_32 = load ? ivl(load) : null;

    // ---- pins. Tiny Tapeout user pins: 8 ui_in (input only), 8 uo_out (output only), 8 uio (tri-state).
    var uio = data.iss.pins.uio, uiIn = data.iss.pins.ui_in, uoOut = data.iss.pins.uo_out;  // bank sizes: Tiny Tapeout pinout (iss.pins.banks_src)
    var pinsTotal = uiIn + uoOut + uio - link.pins_in - link.pins_out - arch.pins_reserved;
    add(rec('pins_total_avail', 'Pins free for protocols', pinsTotal, 'pins', 'ui_in + uo_out + uio - link.pins_in - link.pins_out - pins_reserved',
      [term('ui_in', uiIn, 'pins', { kind: 'iss', id: 'pins', field: 'ui_in' }), term('uo_out', uoOut, 'pins', { kind: 'iss', id: 'pins', field: 'uo_out' }), term('uio', uio, 'pins', { kind: 'iss', id: 'pins', field: 'uio' }), term('link.pins_in', link.pins_in, 'pins', lref),
        term('link.pins_out', link.pins_out, 'pins', lref), kT('pins_reserved', arch, 'pins_reserved', 'pins')]));
    add(rec('pins_tristate_avail', 'Tri-state pins (uio)', uio, 'pins', 'uio', [term('uio', uio, 'pins', { kind: 'iss', id: 'pins', field: 'uio' })]));
    var pins = { link_in: link.pins_in, link_out: link.pins_out, reserved: arch.pins_reserved, total_avail: pinsTotal, tristate_avail: uio };

    // ---- fmax (PLAN 2.3)
    var lever = 'fmax_slow_' + arch.timing_lever + '_mhz';
    var imemPhys = arch.imem_words * arch.imem_banks;
    var imemBase = data.iss.IMEM_SIZE;
    var sram = arch.imem_tech === 'sram2p';
    var dieDelay = arch.die === '8x4';
    var fIds = [lever, 'r_typ_slow'];
    if (sram) fIds.push('d_sram_slow_ns', 'd_sram_typ_ns'); else fIds.push('imem_dbl_ns');
    if (dieDelay) fIds.push('d_die_8x4_ns');
    // ISA levers lengthen the critical path by their slow-corner fmax parameter (judgement, lo 0); the typ corner divides by r_typ_slow
    var isaLevers = isaActiveLevers(arch), isaLev = (data.isa || {}).levers || {};
    var isaPar = isaLevers.map(function (l) { return (isaLev[l] || {}).fmax_param; }).filter(Boolean);
    isaPar.forEach(function (id) { fIds.push(id); });
    var dIsa = function (v) { return isaPar.reduce(function (s, id) { return s + v[id]; }, 0); };
    var dImemSlow = function (v) { return sram ? v.d_sram_slow_ns : v.imem_dbl_ns * Math.log2(imemPhys / imemBase); };
    var dImemTyp = function (v) { return sram ? v.d_sram_typ_ns : dImemSlow(v) / v.r_typ_slow; };
    var dDie = function (v) { return dieDelay ? v.d_die_8x4_ns : 0; };
    // the parts of the critical path D at one corner (clockBudget shows them; D is their sum, in this order)
    var Dparts = function (corner, v) {
      var o = corner === 'slow' ? { base: 1000 / v[lever], imem: dImemSlow(v), die: dDie(v) }
        : { base: 1000 / (v[lever] * v.r_typ_slow), imem: dImemTyp(v), die: dDie(v) / v.r_typ_slow };
      if (isaPar.length) o.isa = corner === 'slow' ? dIsa(v) : dIsa(v) / v.r_typ_slow;
      return o;
    };
    var DpartIds = function (corner) {
      var typ = corner !== 'slow';
      return { base: typ ? [lever, 'r_typ_slow'] : [lever], imem: (sram ? (typ ? ['d_sram_typ_ns'] : ['d_sram_slow_ns']) : ['imem_dbl_ns'].concat(typ ? ['r_typ_slow'] : [])),
        die: dieDelay ? ['d_die_8x4_ns'].concat(typ ? ['r_typ_slow'] : []) : [],
        isa: isaPar.length ? isaPar.concat(typ ? ['r_typ_slow'] : []) : [] };
    };
    var Dsum = function (corner, v) { var p = Dparts(corner, v); return p.base + p.imem + p.die + (p.isa || 0); };
    var Dslow = function (v) { return Dsum('slow', v); };
    var Dtyp = function (v) { return Dsum('typ', v); };
    var fIn = pick(P, fIds);
    var fSlow = vertex(fIn, function (v) { return 1000 / Dslow(v); }, 'high');
    var fTyp = vertex(fIn, function (v) { return 1000 / Dtyp(v); }, 'high');
    var dTermsS = fIds.map(function (id, i) { return pT(id, P, id, fSlow.pess_direction); });
    var dTermsT = fIds.map(function (id) { return pT(id, P, id, fTyp.pess_direction); });
    var dimemS = sram ? 'd_sram_slow_ns' : 'imem_dbl_ns * log2(imem_words * imem_banks / IMEM_SIZE)';
    var dimemT = sram ? 'd_sram_typ_ns' : 'dImem_slow / r_typ_slow';
    var kt = [kT('imem_tech', arch, 'imem_tech'), kT('imem_words', arch, 'imem_words'), kT('imem_banks', arch, 'imem_banks'),
      kT('timing_lever', arch, 'timing_lever'), kT('die', arch, 'die')].concat(isaLevers.map(function (l) { return kT(l, arch, l); }));
    var isaNote = isaPar.length ? '; ISA levers add their d_isa_* delays (judgement, no flow run of any ISA variant exists)' : '';
    add(rec('fmax_slow', 'Core-clock limit, slow corner', fSlow, 'MHz', '1000 / D_slow; D_slow = 1000 / F_s + dImem_slow + dDie' + (isaPar.length ? ' + dIsa' : '') + '; dImem_slow = ' + dimemS,
      kt.concat(dTermsS), 'F_s = ' + lever + '; pessimistic end per parameter in pess_direction' + isaNote));
    add(rec('fmax_typ', 'Core-clock limit, typ corner', fTyp, 'MHz', '1000 / D_typ; D_typ = 1000 / (F_s * r_typ_slow) + dImem_typ + dDie / r_typ_slow' + (isaPar.length ? ' + dIsa / r_typ_slow' : '') + '; dImem_typ = ' + dimemT,
      kt.concat(dTermsT), 'F_s = ' + lever + '; pessimistic end per parameter in pess_direction' + isaNote));
    var fmaxAt = function (corner, v) { return 1000 / (corner === 'slow' ? Dslow(v) : Dtyp(v)); };
    var fmax = { slow: ivl(fSlow), typ: ivl(fTyp), lever: lever };
    fmax.sel = fmax[arch.signoff_corner];
    fmax.corner = arch.signoff_corner;

    // ---- area (PLAN 2.3, B.4, B.5). Lines are {id,label,basis,count,uses,unit(v)}; v = block values.
    var lines = [];
    function block(id) { if (!data.blocks[id]) throw new Error('missing block ' + id); return data.blocks[id]; }
    function line(id, label, basis, count, uses, unit, note, extra) {  // extra: terms for the slope / anchor blocks
      lines.push({ id: id, label: label, basis: basis, count: count, uses: uses, unit: unit, note: note || '', extra: extra || [] });
    }
    function simple(bid, count, note) {
      var b = block(bid);
      line(bid, b.label, b.basis, count, [bid], function (v) { return v[bid]; }, note);
    }
    simple('engine_base', E, 'per engine');
    var imemAnchor = null;
    var wbits = arch.isa_word_bits;   // instruction width: IMEM area scales by wbits / anchor bits (flop, latch) or picks the macro of that width
    if (arch.imem_tech === 'flop') {
      imemAnchor = familyBlock(data, 'imem_flop', 'anchor');
      var imemSlope = familyBlock(data, 'imem_flop', 'slope');
      line('imem', 'IMEM, flop', 'synth', E, [imemAnchor.id, imemSlope.id],
        function (v) { return (v[imemAnchor.id] + (imemPhys - imemAnchor.words) * v[imemSlope.id]) * (wbits / imemAnchor.bits); },
        'anchor area + (w - anchor words) * slope per engine' + (wbits !== imemAnchor.bits ? ', x ' + wbits + ' / ' + imemAnchor.bits + ' for the instruction width (judgement: imem-ff-32x24 measures 1.495x of 32x16)' : ''),
        [term('w', imemPhys, 'words', { kind: 'knob', id: 'imem_words' }), term('w0', imemAnchor.words, 'words', { kind: 'block', id: imemAnchor.id, field: 'words' }),
          term('slope', imemSlope.area.nom, 'um2/word', { kind: 'block', id: imemSlope.id }),
          kT('bits', arch, 'isa_word_bits', 'bits'), term('bits0', imemAnchor.bits, 'bits', { kind: 'block', id: imemAnchor.id, field: 'bits' })]);
    } else if (arch.imem_tech === 'latch') {
      var latch = familyBlock(data, 'imem_latch', 'anchor');
      line('imem', 'IMEM, latch', 'synth', E, [latch.id], function (v) { return v[latch.id] * imemPhys / latch.words * (wbits / latch.bits); },
        'scales linearly with words' + (wbits !== latch.bits ? ' and, as a judgement, with the instruction width' : ''),
        [term('w', imemPhys, 'words', { kind: 'knob', id: 'imem_words' }), term('w0', latch.words, 'words', { kind: 'block', id: latch.id, field: 'words' }),
          kT('bits', arch, 'isa_word_bits', 'bits'), term('bits0', latch.bits, 'bits', { kind: 'block', id: latch.id, field: 'bits' })]);
    } else {
      imemAnchor = familyBlock(data, 'imem_flop', 'anchor');
      var im = pickMacro(data, 2, imemPhys, wbits);
      line('imem_macro', 'IMEM macro ' + im.copies + ' x ' + im.name, 'macro', E, [], function () { return im.area; }, im.name);
      simple('sram_wrapper', E, 'per engine');
    }
    var fifoAnchor = familyBlock(data, 'fifo', 'anchor'), fifoSlope = familyBlock(data, 'fifo', 'slope');
    line('fifo', 'FIFOs (TX and RX per engine)', 'synth', 2 * E, [fifoAnchor.id, fifoSlope.id],
      function (v) { return v[fifoAnchor.id] + (arch.fifo_depth - fifoAnchor.words) * v[fifoSlope.id]; }, 'depth ' + arch.fifo_depth,
      [kT('depth', arch, 'fifo_depth', 'words'), term('d0', fifoAnchor.words, 'words', { kind: 'block', id: fifoAnchor.id, field: 'words' }),
        term('slope', fifoSlope.area.nom, 'um2/word', { kind: 'block', id: fifoSlope.id })]);
    var pinmuxId = 'pinmux_' + Math.min(E, 4);
    if (E <= 4) simple(pinmuxId, 1, 'pin mux for ' + E + ' engines');
    else line('pinmux', 'Pin mux, ' + E + ' engines', 'synth', 1, ['pinmux_4', 'pinmux_extra_engine'],
      function (v) { return v.pinmux_4 + (E - 4) * v.pinmux_extra_engine; }, 'pinmux_4 + 15,000 per engine beyond 4');
    simple(link.area_block, 1, 'host port of link ' + link.id);
    if (arch.lane) simple('lane', 1);
    if (arch.crc_unit) simple('crc', E, 'per engine');
    if (arch.capture === 'dpll') simple('dpll', E, 'per engine');
    if (arch.capture === 'adpll') { simple('adpll_ctl', 1); simple('adpll_dco_n' + arch.adpll_mult, 1, 'N = ' + arch.adpll_mult); }
    if (arch.ddr_io) simple('ddr_io', 1);
    if (arch.fine_edge === 'dtc') simple('dtc', 1);
    if (arch.deep_fifo_words > 0) {
      var nDirs = arch.deep_fifo_dir === 'both' ? 2 : 1;
      simple('deep_fifo_ctl', nDirs, nDirs + ' direction(s)');
      var dm = pickMacro(data, 1, arch.deep_fifo_words, fifoAnchor.bits);
      line('deep_macro', 'Deep FIFO macro ' + dm.copies + ' x ' + dm.name, 'macro', nDirs, [], function () { return dm.area; }, dm.name);
    }
    arch.extras.forEach(function (id) { simple(id, 1, 'area-only extra'); });
    // ISA lever blocks (judgement): per engine unless the block says per chip; the decode block of the chosen width
    var isaBlocks = [];
    isaLevers.forEach(function (l) {
      ((isaLev[l] || {}).blocks || []).forEach(function (bid) {
        if (l === 'isa_word_bits' && !(bid.length > String(wbits).length && bid.slice(-String(wbits).length - 1) === '_' + wbits)) return;
        var b = block(bid), n = b.per === 'chip' ? 1 : E;
        simple(bid, n, (b.per === 'chip' ? 'per chip' : 'per engine') + ', ISA lever ' + l + ' (judgement)');
        isaBlocks.push(bid);
      });
    });

    var useIds = [];
    lines.forEach(function (ln) { ln.uses.forEach(function (u) { if (useIds.indexOf(u) < 0) useIds.push(u); }); });
    var areaIn = {};
    useIds.forEach(function (id) { areaIn[id] = block(id).area; });
    areaIn.pnr_factor = P.pnr_factor; areaIn.macro_halo = P.macro_halo;
    function areaOf(v) {
      var S = 0, M = 0;
      lines.forEach(function (ln) {
        var a = ln.count * ln.unit(v);
        if (ln.basis === 'macro') M += a; else S += a;
      });
      return { S: S, M: M, total: v.pnr_factor * S + v.macro_halo * M };
    }
    var aTot = vertex(areaIn, function (v) { return areaOf(v).total; }, 'low');
    var aS = vertex(areaIn, function (v) { return areaOf(v).S; }, 'low');
    var aM = vertex(areaIn, function (v) { return areaOf(v).M; }, 'low');
    var areaTerms = lines.map(function (ln) {
      var ref = data.blocks[ln.id] ? { kind: 'block', id: ln.id } : (ln.uses.length ? { kind: 'block', id: ln.uses[0] } : { kind: 'sram_macro', id: ln.note });
      return term(ln.id + ' x' + ln.count, null, 'um2', ref);
    });
    var blocksOut = lines.map(function (ln) {
      var inp = {};
      ln.uses.forEach(function (u) { inp[u] = areaIn[u]; });
      var one = ivlAlways(inp, function (v) { return ln.count * ln.unit(v); }, 'low');
      // area_cell = count x cell area before the P&R factor / macro halo; area_placed = area_cell x that factor (nominal factor; blockDiagram carries the vertices)
      var cell = ivl(one), fac = P[ln.basis === 'macro' ? 'macro_halo' : 'pnr_factor'];
      return { id: ln.id, label: ln.label, basis: ln.basis, count: ln.count, note: ln.note, area_cell: cell,
        area_placed: { nom: cell.nom * fac.nom, pess: cell.pess * fac.hi, opt: cell.opt * fac.lo } };
    });
    areaTerms.forEach(function (t, i) { t.value = blocksOut[i].area_cell; });
    lines.forEach(function (ln) { ln.extra.forEach(function (t) { areaTerms.push(t); }); });
    areaTerms.push(pT('pnr_factor', P, 'pnr_factor', aTot.pess_direction), pT('macro_halo', P, 'macro_halo', aTot.pess_direction));
    var core = data.core_area_um2[arch.die];
    var limV = vertex(pick(P, ['util_max']), function (v) { return v.util_max * core; }, 'high');
    var utilV = { nom: aTot.nom / core * 100, pess: aTot.pess / core * 100, opt: aTot.opt / core * 100, pess_direction: aTot.pess_direction };
    add(rec('area_synth', 'Synth-basis area sum S', aS, 'um2', 'sum over blocks of count * area (synth basis)', areaTerms));
    add(rec('area_macro', 'Macro-basis area sum M', aM, 'um2', 'sum over blocks of count * area (macro basis)', areaTerms));
    add(rec('area_total', 'Total area', aTot, 'um2', 'pnr_factor * S + macro_halo * M', [dT('S', 'area_synth', ivl(aS), 'um2'), dT('M', 'area_macro', ivl(aM), 'um2'),
      pT('pnr_factor', P, 'pnr_factor', aTot.pess_direction), pT('macro_halo', P, 'macro_halo', aTot.pess_direction)],
      'block bands (docs/mvp.md judgement ranges) are vertex inputs too' + (isaBlocks.length ? '; ISA lever blocks are judgement (no RTL exists)' : '')));
    add(rec('core_area', 'Core area of the die', core, 'um2', 'core_area_um2[die]', [kT('die', arch, 'die')]));
    add(rec('area_limit', 'Usable area', limV, 'um2', 'util_max * core_area', [pT('util_max', P, 'util_max', limV.pess_direction), dT('core_area', 'core_area', core, 'um2')]));
    add(rec('util_pct', 'Utilisation', utilV, '%', '100 * area_total / core_area', [dT('area_total', 'area_total', ivl(aTot), 'um2'), dT('core_area', 'core_area', core, 'um2')]));
    var area = { total: ivl(aTot), synth: ivl(aS), macro: ivl(aM), blocks: blocksOut, core: core, limit: ivl(limV), util_pct: ivl(utilV) };

    // At-vertex closures for the limits of 2.5 (functions: dropped by JSON; they take a values object {id: number}).
    var fn = {
      Ttx: txFn, Trx: rxFn, gridIds: gridIds,
      linkW: lf.w, linkR: lf.r, linkWIds: lf.wIds, linkRIds: lf.rIds,
      linkBoth: function (v) { var w = lf.w(v), r = lf.r(v); return (w > 0 && r > 0) ? 1 / (1 / w + 1 / r) : 0; },
      hostLoop: lf.loop, hostLoopIds: lf.loopIds, imemLoad: lf.load, imemLoadIds: lf.loadIds,
      fmax: fmaxAt, fmaxIds: fIds, Dparts: Dparts, DpartIds: DpartIds,
      area: function (v) { return areaOf(v).total; }, areaIn: areaIn
    };
    return {
      arch: arch, params: P, T: T, T_tx: T_tx, T_rx: T_rx, link: L, pins: pins, fmax: fmax, area: area,
      buffers: { fifo_bytes: fifo_bytes, buf_tx_bytes: buf_tx, buf_rx_bytes: buf_rx },
      records: R, fn: fn
    };
  }

  // ---------------------------------------------------------------- limits (PLAN 2.5)
  // A limit is {id, short, label, unit, applies, formula, check(c)}. check returns {na: reason} or
  // {inputs: {id: {lo,nom,hi}}, f(values) -> margin, scale, unit?, formula?, terms(V) -> [term], note?}.
  // Margins are "available minus required"; >= 0 is met. The vertex rule of 3.2 turns f into {nom, pess, opt}.
  // Float slack for "margin >= 0" (a margin of exactly 0 is met): REL_TOL times the limit's own scale (the same |scale| that
  // makes `rel` dimensionless), so a miss smaller than a part per million of the quantity is rounding, not a verdict.
  // No input carries a smaller uncertainty than that.
  var REL_TOL = 1e-6;
  function slack(scale) { return scale ? REL_TOL * Math.abs(scale) : 0; }
  var TICK_EPS = 1e-9;  // ns slack when stepping the timeline ticks
  var ROLES_OUT = ['tx', 'txrx', 'controller', 'target'];
  var ROLES_IN = ['rx', 'txrx', 'controller', 'target'];
  var ROLES_TX = ['tx', 'txrx', 'controller'];
  var ROLES_RX = ['rx', 'txrx'];

  function na(reason) { return { na: reason }; }
  function wf(wl, sym, field, unit) {  // workload field term; fields not in `src` are the defaults of PLAN 2.4
    return term(sym, wl[field], unit, { kind: 'workload_field', wl: wl.id, field: field, default: !(wl.src && field in wl.src) });
  }
  function pt(c, sym, id, V) { return pT(sym, c.P, id, V.pess_direction); }
  function isNum(x) { return typeof x === 'number'; }
  function isSnap(c, edge, Tg) { var r = edge / Tg; return Math.abs(r - Math.round(r)) < c.snap_tol; }

  // The RX eye budget at one input vector v: half an edge interval minus resolution u, drift and jitter (rx_eye body and
  // rxSampling share it).
  function rxEyeParts(c, v) {
    var wl = c.wl, a = c.arch;
    var run = wl.resync === 'packet' ? wl.packet_bits : wl.run_bits;
    var sides = c.opts.mismatch_single_sided ? 1 : 2;
    var Tr = c.d.fn.Trx(v), u = a.fine_edge === 'dtc' ? v.tap_ns : (a.capture === 'dpll' ? Tr / 2 : Tr);
    var drift = sides * wl.rate_tol_pct / 100 * run * c.edge;
    if (a.capture === 'dpll' && wl.rate_tol_pct <= v.dpll_track_pct) drift = 0;
    return { edge_half: c.edge / 2, u: u, drift: drift, jitter: wl.rx_jitter_ns };
  }

  // Host-link demand of a workload (host_stream body and hostDemand share it).
  function hostDemandOf(c) {
    var wl = c.wl, d = c.d, dir = wl.bw_dir;
    var stream = c.stream ? 2 * c.ipu * wl.rate_hz / 1e6 : 0;
    var buf = dir === 'write' ? d.buffers.buf_tx_bytes : (dir === 'read' ? d.buffers.buf_rx_bytes : Math.min(d.buffers.buf_tx_bytes, d.buffers.buf_rx_bytes));
    return { payload: wl.bw_MBps, stream: stream, bwEff: wl.bw_MBps + stream, dir: dir, buf: buf };
  }

  var LIMITS = [
    { id: 'pad_toggle_out', short: 'pad out', label: 'Output pad toggle rate', unit: 'ns',
      applies: 'role in {tx, txrx, controller, target}', formula: '2 * edge - 1000 / f_toggle_out',
      check: function (c) {
        if (ROLES_OUT.indexOf(c.wl.role) < 0) return na('role ' + c.wl.role + ' drives no pad');
        return { inputs: pick(c.P, ['f_toggle_out_mhz']), scale: 2 * c.edge,
          f: function (v) { return 2 * c.edge - 1000 / v.f_toggle_out_mhz; },
          terms: function (V) { return [dT('edge', 'edge_ns', c.edge, 'ns'), pt(c, 'f_toggle_out', 'f_toggle_out_mhz', V)]; } };
      } },
    { id: 'pad_toggle_in', short: 'pad in', label: 'Input pad toggle rate', unit: 'ns',
      applies: 'role in {rx, txrx, controller, target}', formula: '2 * edge - 1000 / f_toggle_in',
      check: function (c) {
        if (ROLES_IN.indexOf(c.wl.role) < 0) return na('role ' + c.wl.role + ' reads no pad');
        return { inputs: pick(c.P, ['f_toggle_in_mhz']), scale: 2 * c.edge,
          f: function (v) { return 2 * c.edge - 1000 / v.f_toggle_in_mhz; },
          terms: function (V) { return [dT('edge', 'edge_ns', c.edge, 'ns'), pt(c, 'f_toggle_in', 'f_toggle_in_mhz', V)]; } };
      } },
    { id: 'tx_edge', short: 'TX edge', label: 'TX edge placement error vs tolerance', unit: 'ns',
      applies: 'tx_tol_ns given, clocking = own, role in {tx, txrx, controller}',
      formula: 'tx_tol - (q + t_clk_jitter + (pair ? t_skew : 0)); q = 0 if edge / T_tx is an integer, else T_tx / 2 (dtc: tap / 2)',
      check: function (c) {
        var wl = c.wl, a = c.arch;
        if (wl.tx_tol_ns == null) return na('no TX tolerance');
        if (wl.clocking !== 'own') return na('clocking ' + wl.clocking + ': no edge grid of ours');
        if (ROLES_TX.indexOf(wl.role) < 0) return na('role ' + wl.role + ' places no edges');
        var ids = ['t_clk_jitter_ns'];
        if (wl.pair) ids.push('t_skew_ns');
        if (a.fine_edge === 'dtc') ids.push('tap_ns');
        return { inputs: pick(c.P, ids), scale: wl.tx_tol_ns,
          f: function (v) {
            var Tg = c.d.fn.Ttx(v), q;
            if (a.fine_edge === 'dtc') q = v.tap_ns / 2;
            else q = (!c.opts.force_snap && isSnap(c, c.edge, Tg)) ? 0 : Tg / 2;
            return wl.tx_tol_ns - (q + v.t_clk_jitter_ns + (wl.pair ? v.t_skew_ns : 0));
          },
          terms: function (V) {
            var t = [wf(wl, 'tx_tol', 'tx_tol_ns', 'ns'), dT('edge', 'edge_ns', c.edge, 'ns'), dT('T_tx', 'T_tx', ivl(c.d.T_tx), 'ns'),
              pt(c, 't_clk_jitter', 't_clk_jitter_ns', V)];
            if (wl.pair) t.push(pt(c, 't_skew', 't_skew_ns', V));
            return t;
          },
          note: a.fine_edge === 'dtc' ? 'delay-line placement: error is half a tap' : (c.opts.force_snap ? 'force_snap: the +-T/2 snap is applied even on an integer grid (calibration)' : 'no snap error when edge is an integer number of grid steps') };
      } },
    { id: 'rate_quant', short: 'rate quant', label: 'Divider quantisation error vs rate tolerance', unit: '%',
      applies: 'rate_tol_pct given, role in {tx, txrx, controller}, divider D >= 1',
      formula: 'rate_tol - 100 * |round(256 D) / 256 - D| / D; D = unit / (T * ceil(ipu))',
      check: function (c) {
        var wl = c.wl;
        if (wl.rate_tol_pct == null) return na('no rate tolerance');
        if (ROLES_TX.indexOf(wl.role) < 0) return na('role ' + wl.role + ' generates no clock');
        var D = c.unit / (c.T * Math.ceil(c.ipu));
        if (!(D >= 1)) return na('divider ' + D.toFixed(3) + ' < 1 (cycles limit binds)');
        return { inputs: {}, scale: wl.rate_tol_pct,
          f: function () { return wl.rate_tol_pct - 100 * Math.abs(Math.round(256 * D) / 256 - D) / D; },
          terms: function () { return [wf(wl, 'rate_tol', 'rate_tol_pct', '%'), dT('unit', 'unit_ns', c.unit, 'ns'), dT('T', 'T', c.T, 'ns'), wf(wl, 'ipu', 'ipu', 'cycles')].concat(isaT(c, 'ipu', null, 'cycles')); },
          note: 'D = ' + D.toFixed(4) };
      } },
    { id: 'rx_osr', short: 'RX OSR', label: 'RX oversampling', unit: 'samples',
      applies: 'osr given, role in {rx, txrx, controller, target}, not served by the lane', formula: 'edge / T_rx - osr',
      check: function (c) {
        var wl = c.wl;
        if (wl.osr == null) return na('no oversampling requirement');
        if (ROLES_IN.indexOf(wl.role) < 0) return na('role ' + wl.role + ' samples nothing');
        if (c.lane) return na('served by the bus-clocked lane');
        return { inputs: pick(c.P, c.d.fn.gridIds), scale: wl.osr,
          f: function (v) { return c.edge / c.d.fn.Trx(v) - wl.osr; },
          terms: function (V) { return [dT('edge', 'edge_ns', c.edge, 'ns'), dT('T_rx', 'T_rx', ivl(c.d.T_rx), 'ns'), wf(wl, 'osr', 'osr', 'samples')]; } };
      } },
    { id: 'rx_eye', short: 'RX eye', label: 'RX eye: resolution, drift and jitter vs half an edge interval', unit: 'ns',
      applies: 'rate_tol_pct and run_bits given, role in {rx, txrx}',
      formula: 'edge / 2 - u - drift - rx_jitter; u = T_rx (dpll: T_rx / 2, dtc: tap); drift = 2 * rate_tol * run * edge (0 with dpll tracking)',
      check: function (c) {
        var wl = c.wl, a = c.arch;
        if (wl.rate_tol_pct == null || wl.run_bits == null) return na('no rate tolerance or run length');
        if (ROLES_RX.indexOf(wl.role) < 0) return na('role ' + wl.role + ' receives no stream');
        var run = wl.resync === 'packet' ? wl.packet_bits : wl.run_bits;
        var ids = c.d.fn.gridIds.slice();
        if (a.capture === 'dpll') ids.push('dpll_track_pct');
        return { inputs: pick(c.P, ids), scale: c.edge / 2,
          f: function (v) { var q = rxEyeParts(c, v); return q.edge_half - q.u - q.drift - q.jitter; },
          terms: function (V) {
            var t = [dT('edge', 'edge_ns', c.edge, 'ns'), dT('T_rx', 'T_rx', ivl(c.d.T_rx), 'ns'), wf(wl, 'rate_tol', 'rate_tol_pct', '%'),
              wf(wl, 'run', wl.resync === 'packet' ? 'packet_bits' : 'run_bits', 'bits'), wf(wl, 'rx_jitter', 'rx_jitter_ns', 'ns'), kT('capture', a, 'capture')];
            if (a.capture === 'dpll') t.push(pt(c, 'dpll_track', 'dpll_track_pct', V));
            if (a.fine_edge === 'dtc') t.push(pt(c, 'tap', 'tap_ns', V));
            return t;
          },
          note: c.opts.mismatch_single_sided ? 'mismatch_single_sided: only our end is off (K18)' : 'both ends off by rate_tol (worst case)' };
      } },
    { id: 'rx_timestamp', short: 'timestamp', label: 'Timestamp resolution', unit: 'ns',
      applies: 'rx_ts_tol_ns given', formula: 'rx_ts_tol - T_rx / 2 (dtc: tap / 2)',
      check: function (c) {
        var wl = c.wl, a = c.arch;
        if (wl.rx_ts_tol_ns == null) return na('no timestamp requirement');
        return { inputs: pick(c.P, c.d.fn.gridIds), scale: wl.rx_ts_tol_ns,
          f: function (v) { return wl.rx_ts_tol_ns - (a.fine_edge === 'dtc' ? v.tap_ns : c.d.fn.Trx(v)) / 2; },
          terms: function (V) { return [wf(wl, 'rx_ts_tol', 'rx_ts_tol_ns', 'ns'), dT('T_rx', 'T_rx', ivl(c.d.T_rx), 'ns')]; } };
      } },
    { id: 'cycles', short: 'cycles', label: 'Core cycles per unit', unit: 'cycles',
      applies: 'not served by the lane', formula: 'unit / T - ipu * (streaming_active ? 2 : 1)',
      check: function (c) {
        var wl = c.wl;
        if (c.lane) return na('served by the bus-clocked lane');
        var inb = isaIn(c, 'ipu');
        return { inputs: inb, scale: wl.ipu, f: function (v) { return c.unit / c.T - (wl.ipu + isaD(c, 'ipu', v)) * (c.stream ? 2 : 1); },
          terms: function (V) { return [dT('unit', 'unit_ns', c.unit, 'ns'), dT('T', 'T', c.T, 'ns'), wf(wl, 'ipu', 'ipu', 'cycles'), kT('streaming', c.arch, 'streaming')].concat(isaT(c, 'ipu', V, 'cycles')); },
          note: (c.stream ? 'streaming: every instruction word also crosses the host link, doubling the cycle cost. ' : '') +
            (Object.keys(inb).length ? 'The only uncertainty is the ISA effect band (judgement): a miss at its pessimistic end only is marginal, a miss at its optimistic end too is impossible.'
              : 'No uncertainty inputs: pess = nom = opt, so any miss gives opt < 0 and the verdict impossible.') };
      } },
    { id: 'rate_ceiling', short: 'rate ceiling', label: 'Protocol rate ceiling', unit: 'Hz',
      applies: 'rate_ceiling_hz given', formula: 'rate_ceiling_hz - rate_hz',
      check: function (c) {
        var wl = c.wl;
        if (wl.rate_ceiling_hz == null) return na('no protocol ceiling');
        return { inputs: {}, scale: wl.rate_ceiling_hz, f: function () { return wl.rate_ceiling_hz - wl.rate_hz; },
          terms: function () { return [wf(wl, 'rate_ceiling', 'rate_ceiling_hz', 'Hz'), wf(wl, 'rate', 'rate_hz', 'Hz')]; },
          note: 'No uncertainty inputs: pess = nom = opt, so any miss gives opt < 0 and the verdict impossible.' };
      } },
    { id: 'fixed_clock', short: 'fixed clock', label: 'Workload clock must equal the core clock', unit: 'MHz',
      applies: 'requires_f_core_mhz given', formula: '-|f_core - requires_f_core|',
      check: function (c) {
        var wl = c.wl;
        if (wl.requires_f_core_mhz == null) return na('no fixed core clock required');
        return { inputs: {}, scale: wl.requires_f_core_mhz, f: function () { return -Math.abs(c.arch.f_core_mhz - wl.requires_f_core_mhz); },
          terms: function () { return [kT('f_core_mhz', c.arch, 'f_core_mhz', 'MHz'), wf(wl, 'requires_f_core', 'requires_f_core_mhz', 'MHz')]; } };
      } },
    { id: 'response', short: 'response', label: 'Autonomous response deadline', unit: 'ns',
      applies: 'resp_ns or resp_bits given',
      formula: 'deadline - react; react = (T_IN + T_OUT + decide_sm) * T + sync_phase * T + t_pad_rt (+ t_host_loop if the answer is not on chip)',
      check: function (c) {
        var wl = c.wl;
        if (wl.resp_ns == null && wl.resp_bits == null) return na('no response deadline');
        var onChip = c.d.buffers.buf_tx_bytes >= wl.resp_buf_B;
        var ids = ['sync_phase_cyc', 't_pad_rt_ns'].concat(onChip ? [] : c.d.fn.hostLoopIds);
        var deadline = c.records.response_deadline_ns.value;
        return { inputs: Object.assign(pick(c.P, ids), isaIn(c, 'decide_sm')), scale: deadline,
          f: function (v) {
            var react = (c.TIN + c.TOUT + wl.decide_sm + isaD(c, 'decide_sm', v)) * c.T + v.sync_phase_cyc * c.T + v.t_pad_rt_ns;
            return onChip ? deadline - react : deadline - (react + c.d.fn.hostLoop(v));
          },
          terms: function (V) {
            var t = [dT('deadline', 'response_deadline_ns', deadline, 'ns'), term('T_IN', c.TIN, 'cycles', { kind: 'iss', id: 'T_IN' }),
              term('T_OUT', c.TOUT, 'cycles', { kind: 'iss', id: 'T_OUT' }), wf(wl, 'decide_sm', 'decide_sm', 'cycles'), dT('T', 'T', c.T, 'ns'),
              pt(c, 'sync_phase', 'sync_phase_cyc', V), pt(c, 't_pad_rt', 't_pad_rt_ns', V), dT('buf_tx_bytes', 'buf_tx_bytes', c.d.buffers.buf_tx_bytes, 'B'), wf(wl, 'resp_buf_B', 'resp_buf_B', 'B')];
            if (!onChip) t.push(dT('t_host_loop', 't_host_loop', ivl(c.d.link.t_host_loop), 'ns'));
            return t.concat(isaT(c, 'decide_sm', V, 'cycles'));
          },
          note: onChip ? 'the answer (resp_buf_B) fits the TX buffer: no host round trip' : 'the answer does not fit the TX buffer: one host decision loop is added' };
      } },
    { id: 'reflex', short: 'reflex', label: 'Target reflex vs deadline', unit: 'ns',
      applies: 'reflex_ns or reflex_bits given',
      formula: 'deadline - reflex; sampled: (T_IN + T_OUT + reflex_sm) * T + sync_phase * T + t_pad_rt + t_skew + t_setup_ext; lane: t_pad_rt + t_skew + t_setup_ext + t_int',
      check: function (c) {
        var wl = c.wl;
        if (wl.reflex_ns == null && wl.reflex_bits == null) return na('no reflex deadline');
        var deadline = c.records.reflex_deadline_ns.value;
        var ids = c.lane ? ['t_pad_rt_ns', 't_skew_ns', 't_setup_ext_ns', 't_int_ns'] : ['sync_phase_cyc', 't_pad_rt_ns', 't_skew_ns', 't_setup_ext_ns'];
        return { inputs: Object.assign(pick(c.P, ids), c.lane ? {} : isaIn(c, 'reflex_sm')), scale: deadline,
          f: function (v) {
            var ext = v.t_pad_rt_ns + v.t_skew_ns + v.t_setup_ext_ns;
            return deadline - (c.lane ? ext + v.t_int_ns : (c.TIN + c.TOUT + wl.reflex_sm + isaD(c, 'reflex_sm', v)) * c.T + v.sync_phase_cyc * c.T + ext);
          },
          terms: function (V) {
            var t = [dT('deadline', 'reflex_deadline_ns', deadline, 'ns')];
            if (!c.lane) t.push(term('T_IN', c.TIN, 'cycles', { kind: 'iss', id: 'T_IN' }), term('T_OUT', c.TOUT, 'cycles', { kind: 'iss', id: 'T_OUT' }),
              wf(wl, 'reflex_sm', 'reflex_sm', 'cycles'), dT('T', 'T', c.T, 'ns'), pt(c, 'sync_phase', 'sync_phase_cyc', V));
            if (!c.lane) isaT(c, 'reflex_sm', V, 'cycles').forEach(function (x) { t.push(x); });
            t.push(pt(c, 't_pad_rt', 't_pad_rt_ns', V), pt(c, 't_skew', 't_skew_ns', V), pt(c, 't_setup_ext', 't_setup_ext_ns', V));
            if (c.lane) t.push(pt(c, 't_int', 't_int_ns', V), kT('lane', c.arch, 'lane'));
            return t;
          },
          note: c.lane ? 'bus-clocked lane: no core clock in the path' : 'sampled reflex through the core clock' };
      } },
    { id: 'round_trip', short: 'round trip', label: 'Pad round trip incl. external device vs unit interval', unit: 'ns',
      applies: 'rt_ext_ns given', formula: 'unit - (t_pad_rt + rt_ext)',
      check: function (c) {
        var wl = c.wl;
        if (wl.rt_ext_ns == null) return na('no external device delay');
        return { inputs: pick(c.P, ['t_pad_rt_ns']), scale: c.unit, f: function (v) { return c.unit - (v.t_pad_rt_ns + wl.rt_ext_ns); },
          terms: function (V) { return [dT('unit', 'unit_ns', c.unit, 'ns'), pt(c, 't_pad_rt', 't_pad_rt_ns', V), wf(wl, 'rt_ext', 'rt_ext_ns', 'ns')]; } };
      } },
    { id: 'host_stream', short: 'host stream', label: 'Host link streaming', unit: 'MB/s or B',
      applies: 'bandwidth need > 0 and the host cannot slow the clock',
      formula: 'L >= bw_eff: +inf; continuous: L - bw_eff (MB/s); framed: buf / (1 - L / bw_eff) - frame_B (bytes)',
      check: function (c) {
        var wl = c.wl, d = c.d;
        var hd = hostDemandOf(c), bwEff = hd.bwEff;
        if (!(bwEff > 0)) return na('no sustained bandwidth need');
        if (wl.rate_elastic) return na('the host may slow the clock (rate_elastic)');
        var dir = hd.dir, ov = c.opts.link_override_MBps, buf = hd.buf;
        var ids = ov != null ? [] : (dir === 'write' ? d.fn.linkWIds : (dir === 'read' ? d.fn.linkRIds : d.fn.linkWIds.concat(d.fn.linkRIds)));
        var framed = wl.frame_B != null;
        // load-only link: the host cannot refill during a transfer (L = 0), so the framed formula buf - frame_B holds for a
        // single isolated frame only. It is used when the workload has idle gaps between frames (gap_ns); a continuous
        // workload is judged by the continuous form L - bw_eff = -bw_eff (not met).
        var loadOnly = ov == null && d.link.kind === 'load_only';
        var framedEff = framed && (!loadOnly || wl.gap_ns != null);
        var Lat = function (v) { return ov != null ? ov : (dir === 'write' ? d.fn.linkW(v) : (dir === 'read' ? d.fn.linkR(v) : d.fn.linkBoth(v))); };
        return { inputs: pick(c.P, ids.filter(function (x, i) { return ids.indexOf(x) === i; })), scale: framedEff ? wl.frame_B || bwEff : bwEff,
          unit: framedEff ? 'B' : 'MB/s',
          f: function (v) {
            var L = Lat(v);
            if (L >= bwEff) return Infinity;
            return framedEff ? buf / (1 - L / bwEff) - wl.frame_B : L - bwEff;
          },
          terms: function (V) {
            var lid = dir === 'write' ? 'link_write_MBps' : (dir === 'read' ? 'link_read_MBps' : 'link_both_MBps');
            var Lv = dir === 'write' ? d.link.write_MBps : (dir === 'read' ? d.link.read_MBps : d.link.both_MBps);
            var t = [ov != null ? term('L', ov, 'MB/s', { kind: 'knob', id: 'host_link' }) : dT('L', lid, ivl(Lv), 'MB/s'),
              wf(wl, 'bw', 'bw_MBps', 'MB/s'), dT('buf', dir === 'read' ? 'buf_rx_bytes' : 'buf_tx_bytes', buf, 'B')];
            if (framed) t.push(wf(wl, 'frame', 'frame_B', 'B'));
            if (c.stream) t.push(wf(wl, 'ipu', 'ipu', 'cycles'), wf(wl, 'rate', 'rate_hz', 'Hz'));
            if (c.stream) isaT(c, 'ipu', null, 'cycles').forEach(function (x) { t.push(x); });
            return t;
          },
          note: (ov != null ? 'link_override_MBps replaces the modelled link (calibration). ' : '') + (c.stream ? 'streaming adds 2 * ipu * rate bytes/us of instruction words. ' : '') +
            (loadOnly ? (framedEff ? 'single frame only: the host cannot refill during the transfer on a load-only link' : 'load-only link: the host cannot refill during the run, so a continuous workload is not met (link bandwidth 0 < need)')
              : (framed ? 'a frame longer than the buffer must be refilled while it drains; L_max = buf / (1 - L / bw_eff)' : 'continuous stream: sustained link bandwidth vs need')) };
      } },
    { id: 'program_words', short: 'program', label: 'Program fits the IMEM', unit: 'words or ns',
      applies: 'not served by the lane', formula: 'cap - words; streaming: +inf; host_reload: gap - t_imem_load(imem_words) (or -1 without a gap)',
      check: function (c) {
        var wl = c.wl, a = c.arch;
        if (c.lane) return na('served by the bus-clocked lane');
        var cap = a.imem_words * a.imem_banks;
        if (c.stream) return { inputs: {}, scale: wl.words, unit: 'words', f: function () { return Infinity; },
          terms: function () { return [wf(wl, 'words', 'words', 'words'), kT('streaming', a, 'streaming'), kT('imem_words', a, 'imem_words', 'words')]; },
          note: 'served by streaming; the cost appears in cycles and host_stream' };
        if (a.imem_paging === 'host_reload' && c.words > cap) {
          var timed = wl.gap_ns != null;
          return { inputs: pick(c.P, timed ? c.d.fn.imemLoadIds : []), scale: wl.words, unit: timed ? 'ns' : 'flag',
            f: function (v) { return timed ? wl.gap_ns - c.d.fn.imemLoad(a.imem_words, v) : -1; },
            terms: function (V) {
              var t = [wf(wl, 'words', 'words', 'words'), kT('imem_words', a, 'imem_words', 'words'), kT('imem_paging', a, 'imem_paging')];
              if (timed) t.push(wf(wl, 'gap', 'gap_ns', 'ns'), dT('t_imem_load_32', 't_imem_load_32', ivl(c.d.link.t_imem_load_32), 'ns'));
              return t;
            },
            note: timed ? 'page reload must fit the idle gap between frames' : 'no idle gap to reload in: counts as failing (-1)' };
        }
        return { inputs: isaIn(c, 'words'), scale: wl.words, unit: 'words', f: function (v) { return cap - (wl.words + isaD(c, 'words', v)); },
          terms: function (V) { return [kT('imem_words', a, 'imem_words', 'words'), kT('imem_banks', a, 'imem_banks'), wf(wl, 'words', 'words', 'words')].concat(isaT(c, 'words', V, 'words')); } };
      } },
    { id: 'pins', short: 'pins', label: 'Pins', unit: 'pins',
      applies: 'always', formula: 'min(pins_total_avail - pins, tristate_avail - pins_tristate)',
      check: function (c) {
        var wl = c.wl;
        return { inputs: {}, scale: wl.pins, f: function () { return Math.min(c.d.pins.total_avail - wl.pins, c.d.pins.tristate_avail - wl.pins_tristate); },
          terms: function () { return [dT('pins_total_avail', 'pins_total_avail', c.d.pins.total_avail, 'pins'), dT('tristate_avail', 'pins_tristate_avail', c.d.pins.tristate_avail, 'pins'),
            wf(wl, 'pins', 'pins', 'pins'), wf(wl, 'pins_tristate', 'pins_tristate', 'pins')]; } };
      } },
    { id: 'engines', short: 'engines', label: 'Engines', unit: 'engines',
      applies: 'always', formula: 'n_engines - engines',
      check: function (c) {
        return { inputs: {}, scale: c.wl.engines, f: function () { return c.arch.n_engines - c.wl.engines; },
          terms: function () { return [kT('n_engines', c.arch, 'n_engines'), wf(c.wl, 'engines', 'engines', 'engines')]; } };
      } },
    { id: 'isa_xor', short: 'CRC unit', label: 'On-chip CRC for an autonomous reply', unit: 'flag',
      applies: 'needs_xor', formula: '(crc_unit or (isa_xor and the isa_xor effect row enables isa_xor)) ? +1 : -1',
      check: function (c) {
        if (!c.wl.needs_xor) return na('no on-chip CRC needed');
        var viaIsa = !!c.arch.isa_xor && c.isa.enables.indexOf('isa_xor') >= 0, ok = !!c.arch.crc_unit || viaIsa;
        return { inputs: {}, scale: 1, f: function () { return ok ? 1 : -1; },
          terms: function () { return [wf(c.wl, 'needs_xor', 'needs_xor', ''), kT('crc_unit', c.arch, 'crc_unit'), kT('isa_xor', c.arch, 'isa_xor')].concat(isaT(c, 'ipu', null, 'cycles')); },
          note: c.arch.crc_unit ? 'the CRC unit does the CRC (assumed to keep up)' : (viaIsa ? 'the XOR instruction does the CRC step: its cycles are paid in cycles and its words in program_words (ISA estimate rows)' : '') };
      } }
  ];

  // Chip-level items (PLAN-v2 2.1): evaluated once per architecture point by chipCheck, never inside a cell.
  var CHIP_ITEMS = [
    { id: 'core_clock', short: 'core clock', label: 'Core clock vs fmax', unit: 'MHz',
      applies: 'always', formula: 'fmax_<signoff_corner> - f_core_mhz',
      check: function (c) {
        var a = c.arch, d = c.d;
        return { inputs: pick(c.P, d.fn.fmaxIds), scale: a.f_core_mhz, f: function (v) { return d.fn.fmax(a.signoff_corner, v) - a.f_core_mhz; },
          terms: function () { return [dT('fmax', 'fmax_' + a.signoff_corner, ivl(d.fmax[a.signoff_corner]), 'MHz'), kT('f_core_mhz', a, 'f_core_mhz', 'MHz'), kT('signoff_corner', a, 'signoff_corner')]; } };
      } },
    { id: 'die_area', short: 'die area', label: 'Area vs usable die', unit: 'um2',
      applies: 'always', formula: 'util_max * core_area - area_total',
      check: function (c) {
        var d = c.d;
        var inputs = Object.assign({}, d.fn.areaIn, pick(c.P, ['util_max']));
        return { inputs: inputs, scale: c.P.util_max.nom * d.area.core, f: function (v) { return v.util_max * d.area.core - d.fn.area(v); },
          terms: function (V) { return [pt(c, 'util_max', 'util_max', V), dT('core_area', 'core_area', d.area.core, 'um2'), dT('area_total', 'area_total', ivl(d.area.total), 'um2')]; },
          note: 'every block band, pnr_factor and macro_halo are vertex inputs' };
      } },
    { id: 'board_clock', short: 'board clock', label: 'Core clock vs board clock limit', unit: 'MHz',
      applies: 'always', formula: 'f_ext_clk_max - f_core_mhz',
      check: function (c) {
        return { inputs: pick(c.P, ['f_ext_clk_max_mhz']), scale: c.P.f_ext_clk_max_mhz.nom, f: function (v) { return v.f_ext_clk_max_mhz - c.arch.f_core_mhz; },
          terms: function (V) { return [pt(c, 'f_ext_clk_max', 'f_ext_clk_max_mhz', V), kT('f_core_mhz', c.arch, 'f_core_mhz', 'MHz')]; } };
      } },
    { id: 'isa_encoding', short: 'ISA encoding', label: 'Opcode room for the ISA levers', unit: 'slots',
      applies: 'an ISA lever is on', formula: 'min(major_free - sum major slots, minor_free - sum minor slots) at isa_word_bits',
      check: function (c) {
        var a = c.arch, levers = isaActiveLevers(a);
        if (!levers.length) return na('no ISA lever is on');
        var isa = c.data.isa, enc = isa.encoding[a.isa_word_bits], used = { major: 0, minor: 0 }, terms = [kT('isa_word_bits', a, 'isa_word_bits', 'bits')];
        levers.forEach(function (l) {
          var sl = isa.levers[l].slots[a.isa_word_bits];
          used.major += sl.major; used.minor += sl.minor;
          terms.push(term(l + ' slots', sl.major + ' major + ' + sl.minor + ' minor', 'slots', { kind: 'knob', id: l }));
        });
        terms.push(term('major_free', enc.major_free, 'slots', { kind: 'isa_encoding', bits: a.isa_word_bits }), term('minor_free', enc.minor_free, 'slots', { kind: 'isa_encoding', bits: a.isa_word_bits }));
        return { inputs: {}, scale: 1, f: function () { return Math.min(enc.major_free - used.major, enc.minor_free - used.minor); },
          terms: function () { return terms; },
          note: 'major ' + (enc.major_free - used.major) + ', minor ' + (enc.minor_free - used.minor) + ' slots left. Judgement: the 24- and 32-bit formats are paper designs (inputs.json isa.encoding notes)' };
      } },
    { id: 'pins_budget', short: 'pins', label: 'Project pins left after the host link and reserved pins', unit: 'pins',
      applies: 'always', formula: 'ui_in + uo_out + uio - link.pins_in - link.pins_out - pins_reserved',
      check: function (c) {
        var bk = c.data.iss.pins, pn = bk.ui_in + bk.uo_out + bk.uio;
        return { inputs: {}, scale: pn, f: function () { return c.d.pins.total_avail; },
          terms: function () { return [dT('pins_total_avail', 'pins_total_avail', c.d.pins.total_avail, 'pins')]; },
          note: 'scale = sum of the three bank sizes (iss.pins)' };
      } }
  ];

  function limits() {
    return LIMITS.map(function (L) { return { id: L.id, short: L.short, label: L.label, unit: L.unit, applies: L.applies, formula: L.formula }; });
  }
  function chipItems() {
    return CHIP_ITEMS.map(function (L) { return { id: L.id, short: L.short, label: L.label, unit: L.unit, formula: L.formula }; });
  }

  function findWl(wl, data) {
    if (typeof wl !== 'string') return wl;
    var w = data.workloads.filter(function (x) { return x.id === wl; })[0];
    if (!w) throw new Error('unknown workload ' + wl);
    return w;
  }

  // Everything a limit needs about (arch, workload): derived quantities, unit/edge, flags, cell-level records.
  function cellContext(archIn, wlIn, data, opts) {
    opts = opts || {};
    var wl = findWl(wlIn, data);
    var d = opts.derived || derive(archIn, data);
    var arch = d.arch;
    var c = { arch: arch, d: d, wl: wl, P: d.params, opts: opts, T: d.T, data: data, records: {},
      TIN: data.iss.T_IN, TOUT: data.iss.T_OUT, snap_tol: 1e-6 };
    c.unit = 1e9 / wl.rate_hz;
    c.edge = wl.edge_ns != null ? wl.edge_ns : c.unit;
    c.lane = !!arch.lane && wl.clocking === 'bus_clock_in' && wl.role === 'target';
    // ISA levers: the active levers' effect rows for this workload (judgement); nominal effective counts drive the discrete
    // decisions (streaming, divider, host-link stream), the bands enter the limits as pseudo-parameters isa_d_<field>
    c.isa = isaOf(arch, wl.id, data);
    c.ipu = wl.ipu + c.isa.d_ipu.nom; c.words = wl.words + c.isa.d_words.nom;
    c.reflex_sm = wl.reflex_sm + c.isa.d_reflex_sm.nom; c.decide_sm = wl.decide_sm + c.isa.d_decide_sm.nom;
    c.stream = arch.streaming === 'out_exec' && c.words > arch.imem_words * arch.imem_banks;
    var R = c.records;
    R.unit_ns = rec('unit_ns', 'Unit interval', c.unit, 'ns', '1e9 / rate_hz', [wf(wl, 'rate_hz', 'rate_hz', 'Hz')]);
    R.edge_ns = rec('edge_ns', 'Shortest edge interval', c.edge, 'ns', 'edge_ns ?? unit_ns', [wf(wl, 'edge_ns', 'edge_ns', 'ns'), dT('unit', 'unit_ns', c.unit, 'ns')]);
    if (wl.resp_ns != null || wl.resp_bits != null) {
      var rd = wl.resp_ns != null ? wl.resp_ns : wl.resp_bits * c.unit;
      R.response_deadline_ns = rec('response_deadline_ns', 'Response deadline', rd, 'ns', 'resp_ns ?? resp_bits * unit_ns',
        [wf(wl, 'resp_ns', 'resp_ns', 'ns'), wf(wl, 'resp_bits', 'resp_bits', 'bits'), dT('unit', 'unit_ns', c.unit, 'ns')]);
    }
    if (wl.reflex_ns != null || wl.reflex_bits != null) {
      var fd = wl.reflex_ns != null ? wl.reflex_ns : wl.reflex_bits * c.unit;
      R.reflex_deadline_ns = rec('reflex_deadline_ns', 'Reflex deadline', fd, 'ns', 'reflex_ns ?? reflex_bits * unit_ns',
        [wf(wl, 'reflex_ns', 'reflex_ns', 'ns'), wf(wl, 'reflex_bits', 'reflex_bits', 'bits'), dT('unit', 'unit_ns', c.unit, 'ns')]);
    }
    return c;
  }

  // Limits that are about time (shown on the Timing tab and in its cross-point strip); flagged on every limit record.
  var TIMING_IDS = ['reflex', 'response', 'tx_edge', 'round_trip', 'rate_quant', 'rx_osr', 'rx_eye', 'rx_timestamp', 'pad_toggle_out', 'pad_toggle_in'];
  function evalOne(L, c) {
    var s = L.check(c);
    if (s.na) return { na: c.opts.lite ? true : { id: L.id, label: L.label, short: L.short, reason: s.na } };
    var V;
    if (Object.keys(s.inputs).length) V = vertex(s.inputs, s.f, 'high');
    else { var n = s.f({}); V = { nom: n, pess: n, opt: n, pess_direction: {} }; }
    var r = c.opts.lite ? { id: L.id, label: L.label, value: { nom: V.nom, pess: V.pess, opt: V.opt }, unit: s.unit || L.unit }
      : rec(L.id, L.label, V, s.unit || L.unit, s.formula || L.formula, s.terms(V), s.note);
    r.short = L.short; r.applies = true; r.scale = s.scale; r.timing = TIMING_IDS.indexOf(L.id) >= 0;
    r.rel = V.nom === Infinity ? Infinity : (V.nom === -Infinity ? -Infinity : (s.scale ? V.nom / Math.abs(s.scale) : (V.nom >= 0 ? Infinity : -Infinity)));
    r.rel_pess = V.pess === Infinity ? Infinity : (V.pess === -Infinity ? -Infinity : (s.scale ? V.pess / Math.abs(s.scale) : 0));
    return { rec: r };
  }

  // evaluateCell(arch, wl, data, opts): 2.6. `wl` is a workload object or id. opts: derived (cache), force_snap,
  // link_override_MBps, mismatch_single_sided (calibration only).
  function evaluateCell(archIn, wlIn, data, opts) {
    var c = cellContext(archIn, wlIn, data, opts);
    var lim = [], nas = [];
    LIMITS.forEach(function (L) {
      var o = evalOne(L, c);
      if (o.na) { if (o.na !== true) nas.push(o.na); } else lim.push(o.rec);
    });
    var verdict = 'served';
    lim.forEach(function (r) {   // per-limit verdict: the same test as the cell's, applied to this limit alone
      r.verdict = r.value.opt < -slack(r.scale) ? 'impossible' : (r.value.pess < -slack(r.scale) ? 'marginal' : 'served');
      if (r.value.opt < -slack(r.scale)) verdict = 'impossible';
      else if (r.value.pess < -slack(r.scale) && verdict !== 'impossible') verdict = 'marginal';
    });
    var order = lim.map(function (r, i) { return i; });
    order.sort(function (a, b) { return (lim[a].rel < lim[b].rel ? -1 : (lim[a].rel > lim[b].rel ? 1 : a - b)); });  // ties: table order
    var ranked = order.map(function (i) {
      var r = lim[i];
      return { id: r.id, short: r.short, nom: r.value.nom, pess: r.value.pess, opt: r.value.opt, rel: r.rel, unit: r.unit };
    });
    var scaleOf = {};
    lim.forEach(function (r) { scaleOf[r.id] = r.scale; });
    var blocked = verdict === 'impossible' ? ranked.filter(function (r) { return r.opt < -slack(scaleOf[r.id]); }).map(function (r) { return r.id; }) : [];
    // binding = the limit that explains the verdict: for an impossible cell the smallest-rel limit among blocked_by; otherwise
    // the overall smallest-rel limit. `closest` is always the overall smallest-rel limit.
    var closest = lim[order[0]];
    var bind = blocked.length ? lim[order.filter(function (i) { return lim[i].id === blocked[0]; })[0]] : closest;
    return {
      wl: c.wl.id, verdict: verdict, binding: bind.id, closest: closest.id,
      headline: { limit: bind.id, short: bind.short, nom: bind.value.nom, pess: bind.value.pess, opt: bind.value.opt, unit: bind.unit, rel: bind.rel },
      limits: lim, ranked: ranked, blocked_by: blocked, na: nas, records: c.opts.lite ? {} : c.records, flags: { lane_serves: c.lane, streaming_active: c.stream }
    };
  }

  // chipCheck (PLAN-v2 2.1): chip-level items evaluated once per architecture point.
  function chipCheck(archIn, data, opts) {
    opts = opts || {};
    var d = opts.derived || derive(archIn, data);
    var c = { arch: d.arch, d: d, P: d.params, data: data, opts: opts };
    var items = [], records = {};
    CHIP_ITEMS.forEach(function (L) {
      var o = evalOne(L, c);
      if (o.na) return;
      var r = o.rec, e = slack(r.scale);
      var verdict = r.value.opt < -e ? 'fail' : (r.value.pess < -e ? 'marginal' : 'pass');
      var it = { id: L.id, short: L.short, label: L.label, unit: r.unit, scale: r.scale, value: r.value, rel: r.rel, rel_pess: r.rel_pess, verdict: verdict };
      if (!opts.lite) { it.formula = r.formula; it.terms = r.terms; it.method = r.method; it.note = r.note; it.pess_direction = r.pess_direction; records[L.id] = r; }
      items.push(it);
    });
    var verdict = 'pass';
    items.forEach(function (it) { if (it.verdict === 'fail') verdict = 'fail'; else if (it.verdict === 'marginal' && verdict === 'pass') verdict = 'marginal'; });
    var pool = items.filter(function (it) { return verdict === 'fail' ? it.verdict === 'fail' : (verdict === 'marginal' ? it.verdict !== 'pass' : true); });
    var key = function (it) { return verdict === 'marginal' ? it.rel_pess : it.rel; };
    var best = pool.reduce(function (b, it) { return key(it) < key(b) ? it : b; }, pool[0]);
    return { verdict: verdict, driver: best.id, items: items, records: records };
  }

  function evaluateAll(archIn, data, filterTag, opts) {
    opts = opts || {};
    var d = derive(archIn, data);
    var cells = {}, counts = { served: 0, marginal: 0, impossible: 0 };
    var by = { served: [], marginal: [], impossible: [] };
    data.workloads.forEach(function (w) {
      if (filterTag && filterTag !== 'all' && (w.tags || []).indexOf(filterTag) < 0) return;
      var cell = evaluateCell(d.arch, w, data, { derived: d, lite: !!opts.lite });
      cells[w.id] = cell; counts[cell.verdict]++; by[cell.verdict].push(w.id);
    });
    return { cells: cells, counts: counts, by_verdict: by, arch: d, chip: chipCheck(d.arch, data, { derived: d, lite: !!opts.lite }) };
  }

  // For the monotonicity test (3.2): each applicable limit's margin at n evenly spaced values of every used
  // input between lo and hi, the others at nominal.
  function sensitivity(archIn, wlIn, data, n) {
    var c = cellContext(archIn, wlIn, data, {}), out = [];
    LIMITS.concat(CHIP_ITEMS).forEach(function (L) {
      var s = L.check(c);
      if (s.na) return;
      Object.keys(s.inputs).forEach(function (k) {
        var u = s.inputs[k];
        if (u.lo === u.hi) return;
        var xs = [], ms = [];
        for (var i = 0; i < n; i++) {
          var v = {};
          Object.keys(s.inputs).forEach(function (q) { v[q] = s.inputs[q].nom; });
          v[k] = u.lo + (u.hi - u.lo) * i / (n - 1);
          xs.push(v[k]); ms.push(s.f(v));
        }
        out.push({ limit: L.id, input: k, x: xs, margin: ms });
      });
    });
    return out;
  }

  // ================================================================ Task 6: solvers, calibration, diagrams
  var MAXRATE_LO = 1e3, MAXRATE_HI = 1e9, MAXRATE_ITER = 60;  // PLAN 2.7
  var VERTICES = ['nom', 'pess', 'opt'];

  function vertexTriple(fn) { return { nom: fn('nom'), pess: fn('pess'), opt: fn('opt') }; }

  // {id, scale, nom, pess, opt} of one limit in a cell context; null if it does not apply.
  function limitTriple(L, c) {
    var s = L.check(c);
    if (s.na) return null;
    var V;
    if (Object.keys(s.inputs).length) V = vertex(s.inputs, s.f, 'high');
    else { var n = s.f({}); V = { nom: n, pess: n, opt: n }; }
    return { id: L.id, scale: s.scale, nom: V.nom, pess: V.pess, opt: V.opt };
  }

  // The limits failing (< 0 at `vx`) for workload `wl`, sorted by margin / |scale| (smallest first).
  function failingAt(d, wl, data, vx, opts, skip, only) {
    var c = cellContext(d.arch, wl, data, Object.assign({}, opts, { derived: d }));
    var fails = [];
    LIMITS.forEach(function (L) {
      if (skip.indexOf(L.id) >= 0 || (only && only.indexOf(L.id) < 0)) return;
      var t = limitTriple(L, c);
      if (!t || !(t[vx] < -slack(t.scale))) return;
      fails.push({ id: L.id, rel: t.scale ? t[vx] / Math.abs(t.scale) : -Infinity });
    });
    fails.sort(function (a, b) { return a.rel - b.rel; });
    return fails;
  }

  // maxRate (PLAN 2.7): largest rate_hz at which every applicable limit is >= 0 at `vertexName` (nom | pess | opt).
  // opts: skip (limit ids, default ['fixed_clock']), only (limit ids; calibration), plus the evaluateCell calibration
  // options. The lower bound is MAXRATE_LO, or a thousandth of the row's own rate when that is lower.
  function maxRate(archIn, wlIn, data, vertexName, opts) {
    opts = opts || {};
    var vx = vertexName || 'nom';
    if (VERTICES.indexOf(vx) < 0) throw new Error('vertex must be nom, pess or opt');
    var wl0 = findWl(wlIn, data), d = derive(archIn, data);
    var skip = opts.skip || ['fixed_clock'], only = opts.only || null;
    var at = function (r) { return failingAt(d, Object.assign({}, wl0, { rate_hz: r }), data, vx, opts, skip, only); };
    var lo = Math.min(MAXRATE_LO, wl0.rate_hz / 1e3), hi = MAXRATE_HI;
    var f = at(lo);
    if (f.length) return { rate_hz: 0, binding: f[0].id, vertex: vx };
    if (!at(hi).length) return { rate_hz: hi, binding: null, vertex: vx };
    for (var i = 0; i < MAXRATE_ITER; i++) {
      var mid = Math.exp((Math.log(lo) + Math.log(hi)) / 2);
      if (at(mid).length) hi = mid; else lo = mid;
    }
    return { rate_hz: lo, binding: at(hi)[0].id, vertex: vx };
  }

  // Largest single-sided rate mismatch (%) for which rx_eye stays >= 0 at `vertexName` (K18).
  function rxTolerance(archIn, wlIn, data, vertexName) {
    var vx = vertexName || 'nom', wl0 = findWl(wlIn, data), d = derive(archIn, data);
    var L = LIMITS.filter(function (x) { return x.id === 'rx_eye'; })[0];
    var ok = function (tol) {
      var c = cellContext(d.arch, Object.assign({}, wl0, { rate_tol_pct: tol }), data, { derived: d, mismatch_single_sided: true });
      var t = limitTriple(L, c);
      return t === null ? null : t[vx] >= -slack(t.scale);
    };
    var first = ok(0);
    if (first === null) return null;
    if (!first) return 0;
    var lo = 0, hi = 100;
    if (ok(hi)) return Infinity;
    for (var i = 0; i < MAXRATE_ITER; i++) {
      var mid = (lo + hi) / 2;
      if (ok(mid)) lo = mid; else hi = mid;
    }
    return lo;
  }

  // Receive jitter (ns) at which rx_eye reaches 0 (nominal vertex): the eye budget left once spec jitter is removed (K19).
  function rxJitterBreakEven(archIn, wlIn, data) {
    var wl0 = findWl(wlIn, data), d = derive(archIn, data);
    var L = LIMITS.filter(function (x) { return x.id === 'rx_eye'; })[0];
    var ok = function (j) {
      var t = limitTriple(L, cellContext(d.arch, Object.assign({}, wl0, { rx_jitter_ns: j }), data, { derived: d }));
      return t.nom >= -slack(t.scale);
    };
    if (!ok(0)) return 0;
    var lo = 0, hi = wl0.edge_ns != null ? wl0.edge_ns : 1e9 / wl0.rate_hz;
    for (var i = 0; i < MAXRATE_ITER; i++) {
      var mid = (lo + hi) / 2;
      if (ok(mid)) lo = mid; else hi = mid;
    }
    return lo;
  }

  // ---------------------------------------------------------------- sweep, pareto (PLAN 2.7)
  function splitTag(s) { var i = s.indexOf(':'); return i < 0 ? [s, null] : [s.slice(0, i), s.slice(i + 1)]; }

  // Cells counted per vertex: a cell counts at `nom` / `pess` / `opt` when every limit is >= 0 at that vertex.
  function countsByVertex(arch, data, tag) {
    var ev = evaluateAll(arch, data, tag || 'all');
    var out = { served: { nom: 0, pess: 0, opt: 0 }, marginal: 0, impossible: 0, total: 0 };
    Object.keys(ev.cells).forEach(function (id) {
      var cell = ev.cells[id];
      out.total++;
      VERTICES.forEach(function (v) {
        if (cell.limits.every(function (r) { return r.value[v] >= -slack(r.scale); })) out.served[v]++;
      });
      if (cell.verdict === 'marginal') out.marginal++;
      else if (cell.verdict === 'impossible') out.impossible++;
    });
    return out;
  }

  function metricAt(arch, metric, data) {
    var parts = splitTag(metric), kind = parts[0], rest = parts[1];
    if (kind === 'served_count') return countsByVertex(arch, data, rest).served;
    if (kind === 'marginal_count') { var m = countsByVertex(arch, data, rest).marginal; return { nom: m, pess: m, opt: m }; }
    var d = derive(arch, data);
    if (kind === 'chip_margin') {
      var ci = chipCheck(d.arch, data, { derived: d, lite: true }).items.filter(function (x) { return x.id === rest; })[0];
      if (!ci) throw new Error('unknown chip item ' + rest);
      return ivl(ci.value);
    }
    switch (kind) {
      case 'area_um2': return ivl(d.area.total);
      case 'util_pct': return ivl(d.area.util_pct);
      case 'fmax_typ': return ivl(d.fmax.typ);
      case 'fmax_slow': return ivl(d.fmax.slow);
      case 'link_write_MBps': return ivl(d.link.write_MBps);
      case 'link_read_MBps': return ivl(d.link.read_MBps);
      case 'max_rate':
        return vertexTriple(function (v) { return maxRate(d.arch, rest, data, v).rate_hz; });
      case 'margin': {
        var q = splitTag(rest);
        if (CHIP_ITEMS.some(function (L) { return L.id === q[1]; })) throw new Error('chip item, not a protocol limit: use chip_margin:' + q[1]);
        var cell = evaluateCell(d.arch, q[0], data, { derived: d });
        var r = cell.limits.filter(function (x) { return x.id === q[1]; })[0];
        return r ? ivl(r.value) : { nom: null, pess: null, opt: null };
      }
      default: throw new Error('unknown metric ' + metric);
    }
  }

  // sweep(arch, knobId, values, metric, data) -> [{x, y: {nom, pess, opt}}]
  function sweep(archIn, knobId, values, metric, data) {
    var base = normArch(archIn);
    if (!KNOBS.some(function (k) { return k.id === knobId; })) throw new Error('unknown knob ' + knobId);
    return values.map(function (x) {
      var a = Object.assign({}, base); a[knobId] = x;
      var ch = chipCheck(a, data, { lite: true });
      return { x: x, y: metricAt(a, metric, data), chip: ch.verdict, chip_driver: ch.driver };
    });
  }

  // pareto(points): indices of the non-dominated points for (min area, max served). A point is {area, served}
  // (also accepted: {x, y}). A point is dominated if another has area <= and served >= with one strict.
  function pareto(points) {
    var P = points.map(function (p) { return { a: 'area' in p ? p.area : p.x, s: 'served' in p ? p.served : p.y, fail: p.chip === 'fail' }; });
    var keep = [];
    P.forEach(function (p, i) {
      if (p.fail) return;
      var dom = P.some(function (q, j) { return !q.fail && j !== i && q.a <= p.a && q.s >= p.s && (q.a < p.a || q.s > p.s); });
      if (!dom) keep.push(i);
    });
    return keep;
  }

  // ---------------------------------------------------------------- mix (PLAN 2.7)
  function verdictOf(m, scale) { var e = slack(scale); return m.opt < -e ? 'impossible' : (m.pess < -e ? 'marginal' : 'served'); }

  function mix(archIn, wlIds, data, mopts) {
    var d = (mopts && mopts.derived) || derive(archIn, data), L = d.link, cap = d.arch.imem_words * d.arch.imem_banks;
    var need = { engines: 0, pins: 0, pins_tristate: 0, write: 0, read: 0, both: 0 }, exempt = [];
    var members = [], noFit = [], hostLoopMembers = [];
    var reload = { nom: 0, pess: 0, opt: 0 };               // host_reload members: page load time per frame gap (link time share)
    var loopShare = { nom: 0, pess: 0, opt: 0 };            // members answering through the host loop: loop time per deadline
    wlIds.forEach(function (id) {
      var c = cellContext(d.arch, id, data, { derived: d }), wl = c.wl;
      var cell = evaluateCell(d.arch, wl, data, { derived: d, lite: true });
      var pw = limitById(cell.limits, 'program_words'), fits = !pw || verdictOf(pw.value, pw.scale) === 'served';
      var viaHost = (wl.resp_ns != null || wl.resp_bits != null) && wl.resp_buf_B > d.buffers.buf_tx_bytes;
      members.push({ wl: wl.id, verdict: cell.verdict, binding: cell.binding, program_fits: fits, host_loop: viaHost });
      if (!fits) noFit.push(wl.id);
      need.engines += wl.engines; need.pins += wl.pins; need.pins_tristate += wl.pins_tristate;
      if (viaHost) {
        hostLoopMembers.push(wl.id);
        var dl = c.records.response_deadline_ns.value;
        VERTICES.forEach(function (v) { loopShare[v] += L.t_host_loop[v] / dl; });
      }
      if (d.arch.imem_paging === 'host_reload' && !c.lane && !c.stream && c.words > cap && wl.gap_ns != null) {
        var tl = vertex(pick(d.params, d.fn.imemLoadIds), function (v) { return d.fn.imemLoad(d.arch.imem_words, v); }, 'low');
        VERTICES.forEach(function (v) { reload[v] += tl[v] / wl.gap_ns; });
      }
      if (wl.rate_elastic) { exempt.push(wl.id); return; }
      if (c.stream) need.write += 2 * c.ipu * wl.rate_hz / 1e6;
      need[wl.bw_dir] += wl.bw_MBps;
    });
    var rows = [];
    function row(id, label, unit, needV, availV, marginV, scale) {
      var m = { nom: marginV.nom, pess: marginV.pess, opt: marginV.opt };
      rows.push({ id: id, label: label, unit: unit, need: needV, avail: availV, margin: m, verdict: verdictOf(m, scale === undefined ? needV : scale) });
    }
    var flat = function (n, a) { return { nom: a - n, pess: a - n, opt: a - n }; };
    row('engines', 'Engines', 'engines', need.engines, d.arch.n_engines, flat(need.engines, d.arch.n_engines));
    row('pins', 'Pins', 'pins', need.pins, d.pins.total_avail, flat(need.pins, d.pins.total_avail));
    row('pins_tristate', 'Tri-state pins', 'pins', need.pins_tristate, d.pins.tristate_avail, flat(need.pins_tristate, d.pins.tristate_avail));
    row('imem', 'Program fits one engine\'s IMEM', 'members', noFit.length, 0, flat(noFit.length, 0), 1);
    [['write', L.write_MBps], ['read', L.read_MBps], ['both', L.both_MBps]].forEach(function (e) {
      var n = need[e[0]], a = e[1];
      row('bw_' + e[0], 'Link ' + e[0] + ' bandwidth', 'MB/s', n, ivl(a),
        vertexTriple(function (v) { return a[v] - n; }));
    });
    var share = vertexTriple(function (v) {
      var s = reload[v];
      [['write', L.write_MBps], ['read', L.read_MBps], ['both', L.both_MBps]].forEach(function (e) {
        if (need[e[0]] > 0) s += e[1][v] > 0 ? need[e[0]] / e[1][v] : Infinity;
      });
      return s;
    });
    row('link_share', 'Link time share (half duplex)', 'fraction', share, 1, vertexTriple(function (v) { return 1 - share[v]; }), 1);
    row('host_loop', 'Host decision loop time share', 'fraction', loopShare, 1, vertexTriple(function (v) { return 1 - loopShare[v]; }), 1);
    var worst = 'served';
    rows.forEach(function (r) { if (r.verdict === 'impossible') worst = 'impossible'; else if (r.verdict === 'marginal' && worst === 'served') worst = 'marginal'; });
    var chip = chipCheck(d.arch, data, { derived: d, lite: true });
    return { workloads: wlIds.slice(), rows: rows, verdict: worst, pass: worst === 'served', exempt: exempt, need: need,
      members: members, imem_no_fit: noFit, host_loop_members: hostLoopMembers, chip: { verdict: chip.verdict, driver: chip.driver } };
  }

  // pairs(arch, data, opts): every pair (a <= b, a == b means two instances of one protocol) of the workloads that are
  // `served` alone at this arch (opts.tag filters the workloads, default 'all'; opts.ids replaces the pool), checked with
  // mix(). Returns {ids, pairs: [{a, b, verdict, failing: [mix row ids not served], binding: id of the worst row, rows}],
  // tally: {served, marginal, impossible, total}, skipped: [workloads not served alone]}. `binding` is the failing row with
  // the smallest optimistic margin relative to its need (null when the pair passes). Pure; cost = one mix() per pair.
  function pairs(archIn, data, popts) {
    popts = popts || {};
    var d = derive(archIn, data), pool = popts.ids, skipped = [];
    if (!pool) {
      pool = [];
      data.workloads.forEach(function (w) {
        if (popts.tag && popts.tag !== 'all' && (w.tags || []).indexOf(popts.tag) < 0) return;
        pool.push(w.id);
      });
    }
    var ids = [];
    pool.forEach(function (id) {
      var cell = evaluateCell(d.arch, id, data, { derived: d, lite: true });
      if (cell.verdict === 'served') ids.push(id); else skipped.push({ wl: id, verdict: cell.verdict });
    });
    var out = [], tally = { served: 0, marginal: 0, impossible: 0, total: 0 };
    for (var i = 0; i < ids.length; i++) {
      for (var j = i; j < ids.length; j++) {
        var m = mix(d.arch, [ids[i], ids[j]], data, { derived: d });
        var bad = m.rows.filter(function (r) { return r.verdict !== 'served'; });
        var worst = null;
        bad.forEach(function (r) {
          var key = r.margin.opt / (Math.abs(typeof r.need === 'object' ? r.need.nom : r.need) || 1);
          if (!worst || key < worst.key) worst = { key: key, id: r.id };
        });
        out.push({ a: ids[i], b: ids[j], verdict: m.verdict, failing: bad.map(function (r) { return r.id; }),
          binding: worst ? worst.id : null, rows: bad.map(function (r) { return { id: r.id, label: r.label, unit: r.unit, need: r.need, avail: r.avail, margin: r.margin, verdict: r.verdict }; }) });
        tally[m.verdict]++; tally.total++;
      }
    }
    return { ids: ids, pairs: out, tally: tally, skipped: skipped };
  }

  // maxConcurrent(arch, data, ids, opts): the largest subset of `ids` (distinct protocols) that mix() passes (verdict
  // `served`; with opts.marginal also `marginal`). Members that are not served alone are left out first. mix needs only add, so a failing set fails every
  // superset: depth-first search with that pruning and a size bound, capped at opts.cap mix evaluations (default 3000).
  // Returns {n, members (one best set), candidates, exact (false if the cap was hit), evaluations, cap, upper_bound, marginal_ok, alone_excluded}.
  function maxConcurrent(archIn, data, ids, mopts) {
    mopts = mopts || {};
    var cap = mopts.cap || 3000, d = derive(archIn, data), evals = 0, capped = false;
    var cand = [], excluded = [];
    ids.forEach(function (id) {
      if (cand.indexOf(id) >= 0) return;
      var cell = evaluateCell(d.arch, id, data, { derived: d, lite: true });
      if (cell.verdict === 'served' || (mopts.marginal && cell.verdict === 'marginal')) cand.push(id); else excluded.push(id);
    });
    // upper bound on the size: per additive resource (engines, pins, tri-state pins) the most members whose smallest needs fit
    var ub = cand.length;
    [['engines', d.arch.n_engines], ['pins', d.pins.total_avail], ['pins_tristate', d.pins.tristate_avail]].forEach(function (r) {
      var needs = cand.map(function (id) { return findWl(id, data)[r[0]]; }).sort(function (a, b) { return a - b; }), used = 0, n = 0;
      for (var q = 0; q < needs.length && used + needs[q] <= r[1]; q++) { used += needs[q]; n++; }
      ub = Math.min(ub, n);
    });
    var best = [];
    function ok(set) {
      if (evals >= cap) { capped = true; return false; }
      evals++;
      var v = mix(d.arch, set, data, { derived: d }).verdict;
      return v === 'served' || (mopts.marginal && v === 'marginal');
    }
    function dfs(i, cur) {
      if (cur.length > best.length) best = cur.slice();
      if (capped || best.length >= ub || cur.length + (cand.length - i) <= best.length) return;
      for (var k = i; k < cand.length; k++) {
        if (best.length >= ub || cur.length + (cand.length - k) <= best.length) return;
        cur.push(cand[k]);
        if (ok(cur)) dfs(k + 1, cur);
        cur.pop();
        if (capped) return;
      }
    }
    dfs(0, []);
    return { n: best.length, members: best, candidates: cand, alone_excluded: excluded, exact: !capped, evaluations: evals, cap: cap, upper_bound: ub, marginal_ok: !!mopts.marginal };
  }

  // ---------------------------------------------------------------- explore solvers (PLAN-v2 2.2, 2.3)
  var VERDICT_RANK = { impossible: 0, marginal: 1, served: 2 };
  var CHIP_RANK = { pass: 0, marginal: 1, fail: 2 };
  var EXPLORE_TAG = 'cap19';          // default workload tag of knobImpact / whatWouldItTake (the UI default)
  var PAIR_TOP_K = 8, LOSS_EVAL_MAX = 12, RESTORE_MAX = 3;
  var SKIPPED_KNOBS = ['env', 'signoff_corner'];   // evaluation settings, not architecture

  function fullArch(a) { return Object.assign(defaultArch(), a || {}); }
  function knobIndex(id) { for (var i = 0; i < KNOBS.length; i++) if (KNOBS[i].id === id) return i; return -1; }
  function r6(x) { return Math.round(x * 1e6) / 1e6; }

  // Why a knob changes nothing at this point (the UI lists these as "no effect at this point"), or null.
  function noEffectReason(arch, id) {
    if (id === 'adpll_mult' && arch.capture !== 'adpll') return 'capture is not adpll';
    if (id === 'imem_banks' && arch.imem_paging !== 'bank_switch') return 'imem_paging is not bank_switch';
    if (id === 'deep_fifo_dir' && !(arch.deep_fifo_words > 0)) return 'there is no deep FIFO';
    return null;
  }

  // Sorted grid of a number knob: KNOBS.quick, plus inputs.json sweep_defaults[knob] (min..max step, extra); null if none.
  function numberGrid(k, data) {
    var g = (k.quick || []).slice(), sd = data && data.sweep_defaults && data.sweep_defaults[k.id];
    if (!g.length && !sd) return null;
    if (sd) {
      for (var x = sd.min, i = 0; x <= sd.max + 1e-9; i++, x = sd.min + i * sd.step) g.push(r6(x));
      (sd.extra || []).forEach(function (e) { g.push(e); });
    }
    g.sort(function (a, b) { return a - b; });
    return g.filter(function (x, i) { return i === 0 || x !== g[i - 1]; });
  }

  // alternatives(arch, mode, data): mode 'neighbour' (2.2) or 'full' (2.3). Every item {knob, value, from, steps, invalid?}.
  function alternatives(archIn, mode, data) {
    var arch = fullArch(archIn), out = [];
    KNOBS.forEach(function (k) {
      if (SKIPPED_KNOBS.indexOf(k.id) >= 0 || noEffectReason(arch, k.id)) return;
      var cur = arch[k.id], vals = [];     // vals: [value, steps]
      if (k.type === 'bool') vals.push([!cur, 1]);
      else if (k.type === 'enum') {
        var ci = k.options.indexOf(cur);
        k.options.forEach(function (o, i) { if (o !== cur) vals.push([o, Math.abs(i - ci)]); });
      } else if (k.type === 'multi') {
        k.options.forEach(function (o) {
          var has = cur.indexOf(o) >= 0;
          vals.push([has ? cur.filter(function (e) { return e !== o; }) : cur.concat([o]), 1]);
        });
      } else {
        var grid = numberGrid(k, data), lo, hi;
        if (!grid) { grid = []; for (var j = 0, x = k.min; x <= k.max + 1e-9; j++, x = k.min + j * k.step) grid.push(r6(x)); }
        var merged = grid.indexOf(cur) >= 0 ? grid : grid.concat([cur]).sort(function (a, b) { return a - b; });
        var mi = merged.indexOf(cur);
        merged.forEach(function (x, i) {
          if (x === cur) return;
          if (mode === 'neighbour' && Math.abs(i - mi) !== 1) return;
          vals.push([x, Math.abs(i - mi)]);
        });
      }
      vals.forEach(function (v) {
        var a = { knob: k.id, value: v[0], from: clone(cur), steps: v[1] };
        var t = clone(arch); t[k.id] = clone(v[0]);
        var bad = knobProblems(t);
        if (bad.length) a.invalid = bad.join('; ');
        out.push(a);
      });
    });
    return out;
  }

  function applyAlts(arch, alts) {
    var t = clone(arch);
    alts.forEach(function (a) { t[a.knob] = clone(a.value); });
    return t;
  }

  // The workloads of a tag evaluated at one point (lite), plus the chip check. wlOnly restricts to one workload.
  function evalPoint(arch, data, tag, wlOnly) {
    if (!wlOnly) {
      var ev = evaluateAll(arch, data, tag, { lite: true }), v = {};
      Object.keys(ev.cells).forEach(function (id) { v[id] = ev.cells[id].verdict; });
      return { verdicts: v, counts: ev.counts, chip: ev.chip, d: ev.arch };
    }
    var d = derive(arch, data), cell = evaluateCell(d.arch, wlOnly, data, { derived: d, lite: true }), vv = {};
    vv[cell.wl] = cell.verdict;
    var counts = { served: 0, marginal: 0, impossible: 0 }; counts[cell.verdict]++;
    return { verdicts: vv, counts: counts, chip: chipCheck(d.arch, data, { derived: d, lite: true }), d: d };
  }
  function chipItemOf(chip, id) { return chip.items.filter(function (i) { return i.id === id; })[0]; }
  function nomArea(d) { return nomOf(d.area.total); }

  // knobImpact(arch, data, opts): 2.2. opts: tag (default cap19), wl (restrict to one protocol).
  function knobImpact(archIn, data, opts) {
    opts = opts || {};
    var arch = fullArch(archIn), tag = opts.tag || EXPLORE_TAG;
    var base = evalPoint(arch, data, tag, opts.wl);
    var baseArea = nomArea(base.d), baseClk = chipItemOf(base.chip, 'core_clock').value.nom;
    var alts = alternatives(arch, 'neighbour', data), byKnob = {};
    alts.forEach(function (a) {
      var o = { value: a.value, from: a.from, steps: a.steps, flips: [], ups: 0, downs: 0, chip: null, d_area_nom: 0, d_core_clock_nom: 0 };
      if (a.invalid) o.invalid = a.invalid;
      else {
        try {
          var ev = evalPoint(applyAlts(arch, [a]), data, tag, opts.wl);
          Object.keys(ev.verdicts).forEach(function (id) {
            var f = base.verdicts[id], t = ev.verdicts[id];
            if (f === t) return;
            var up = VERDICT_RANK[t] > VERDICT_RANK[f];
            o.flips.push({ wl: id, from: f, to: t, dir: up ? 'up' : 'down' });
            if (up) o.ups++; else o.downs++;
          });
          var drv = chipItemOf(ev.chip, ev.chip.driver);
          o.chip = { verdict: ev.chip.verdict, driver: ev.chip.driver, margin: { nom: drv.value.nom, pess: drv.value.pess, opt: drv.value.opt }, unit: drv.unit };
          o.d_area_nom = nomArea(ev.d) - baseArea;
          o.d_core_clock_nom = chipItemOf(ev.chip, 'core_clock').value.nom - baseClk;
        } catch (e) { o.invalid = String(e.message || e); }
      }
      (byKnob[a.knob] = byKnob[a.knob] || []).push(o);
    });
    var knobs = [];
    KNOBS.forEach(function (k) {
      if (SKIPPED_KNOBS.indexOf(k.id) >= 0) return;
      var why = noEffectReason(arch, k.id);
      knobs.push({ knob: k.id, label: k.label, no_effect: !!why, reason: why, alternatives: byKnob[k.id] || [] });
    });
    return { tag: tag, wl: opts.wl || null, base: { counts: base.counts, chip: { verdict: base.chip.verdict, driver: base.chip.driver } }, knobs: knobs };
  }

  // ---- what would it take
  // Status of the input a term reference points at: 'judgement' for a parameter / ISA effect row of that status.
  function refStatus(ref, data) {
    if (ref.kind === 'param') { var p = data.params[ref.id]; return p ? p.status : null; }
    if (ref.kind === 'block') { var bk = data.blocks[ref.id]; return bk ? bk.status : null; }
    if (ref.kind === 'isa_effect') {
      var rows = ((data.isa || {}).effects || []).filter(function (e) { return e.wl === ref.wl && e.lever === ref.lever; });
      return rows.length ? rows[0].status : null;
    }
    return null;
  }
  // The judgement-status inputs a cell's verdict rests on: those reachable (through derived records) from the limits that
  // blocked the base point, plus every ISA effect row used anywhere in the cell. Returns {id: true}. A candidate is an
  // estimate when it needs such an input that the base point's own blocking limits did not already use.
  function judgedInputs(cell, d, data, blockIds) {
    var seen = {}, out = {};
    function walk(terms, any) {
      (terms || []).forEach(function (t) {
        var r = t.ref;
        if (!r) return;
        if (r.kind === 'derived') {
          var rr = cell.records[r.id] || d.records[r.id];
          if (rr && !seen[r.id]) { seen[r.id] = true; walk(rr.terms, any); }
        } else if (refStatus(r, data) === 'judgement' && (any || r.kind === 'param' || r.kind === 'isa_effect')) {
          out[r.kind + ':' + (r.id || r.lever) + (r.wl ? ':' + r.wl : '')] = true;
        }
      });
    }
    cell.limits.forEach(function (L) {
      walk(L.terms.filter(function (t) { return t.ref && t.ref.kind === 'isa_effect'; }), true);
      if (blockIds.indexOf(L.id) >= 0) walk(L.terms, false);
    });
    return out;
  }

  // ISA inputs behind the chip-level numbers of a point (area blocks, fmax parameters, opcode room): all judgement. Data-driven
  // from inputs.json isa.levers (their blocks and fmax_param), so only an active ISA lever contributes.
  function judgedChipInputs(d, data) {
    var out = {}, lev = (data.isa || {}).levers || {}, bl = {}, pr = {}, seen = {};
    Object.keys(lev).forEach(function (l) {
      (lev[l].blocks || []).forEach(function (b) { bl[b] = true; });
      if (lev[l].fmax_param) pr[lev[l].fmax_param] = true;
    });
    function walk(terms) {
      (terms || []).forEach(function (t) {
        var r = t.ref;
        if (!r) return;
        if (r.kind === 'derived') { if (!seen[r.id] && d.records[r.id]) { seen[r.id] = true; walk(d.records[r.id].terms); } }
        else if ((r.kind === 'block' && bl[r.id]) || (r.kind === 'param' && pr[r.id])) out[r.kind + ':' + r.id] = true;
      });
    }
    ['area_total', 'fmax_' + d.arch.signoff_corner].forEach(function (id) { if (d.records[id]) walk(d.records[id].terms); });
    if (isaActiveLevers(d.arch).length) out['isa_encoding:' + d.arch.isa_word_bits] = true;
    return out;
  }

  // The limits that keep a cell from being served: blocked_by for an impossible cell, the pess-negative ones for a marginal one.
  function failingIds(cell) {
    if (cell.verdict === 'impossible') return cell.blocked_by.slice();
    return cell.limits.filter(function (r) { return r.value.pess < -slack(r.scale); }).map(function (r) { return r.id; });
  }
  function relClamp(x) { return Math.max(-1e6, Math.min(1e6, x)); }
  function limRel(cell, id) {   // a limit that no longer applies counts as met with room to spare
    var r = cell.limits.filter(function (x) { return x.id === id; })[0];
    return r ? relClamp(r.rel) : 1e6;
  }
  function minPessRel(cell) { return cell.limits.reduce(function (m, r) { return Math.min(m, relClamp(r.rel_pess)); }, 1e6); }

  function whatWouldItTake(archIn, wlIn, data, opts) {
    opts = opts || {};
    var arch = fullArch(archIn), tag = opts.tag || EXPLORE_TAG, K = opts.K || PAIR_TOP_K;
    var wl = findWl(wlIn, data);
    var d0 = derive(arch, data), cell0 = evaluateCell(d0.arch, wl, data, { derived: d0, lite: true });
    var out = { wl: wl.id, tag: tag, already: cell0.verdict === 'served', verdict: cell0.verdict, single: [], pairs: [], out_of_reach: false, blocking: [], hints: [] };
    if (out.already) return out;
    var fail0 = failingIds(cell0), area0 = nomArea(d0);
    var judged0 = Object.assign(judgedInputs(evaluateCell(d0.arch, wl, data, { derived: d0 }), d0, data, fail0), judgedChipInputs(d0, data));
    out.blocking = cell0.limits.filter(function (r) { return fail0.indexOf(r.id) >= 0; }).map(function (r) {
      return { id: r.id, short: r.short, nom: r.value.nom, pess: r.value.pess, opt: r.value.opt, unit: r.unit, rel: r.rel };
    });
    var baseAll = null;
    function baseVerdicts() { return baseAll || (baseAll = evalPoint(arch, data, tag)); }

    // step 1: every full-grid alternative, one lite cell each
    var tried = [];
    alternatives(arch, 'full', data).forEach(function (a, idx) {
      if (a.invalid) return;
      var t = applyAlts(arch, [a]), d;
      try { d = derive(t, data); } catch (e) { return; }
      tried.push({ alts: [a], arch: t, d: d, cell: evaluateCell(d.arch, wl, data, { derived: d, lite: true }), order: idx });
    });

    function candOf(x) {
      var chip = chipCheck(x.d.arch, data, { derived: x.d, lite: true });
      var drv = chipItemOf(chip, chip.driver);
      var full = evaluateCell(x.d.arch, wl, data, { derived: x.d });
      var judged = Object.keys(Object.assign(judgedInputs(full, x.d, data, fail0), judgedChipInputs(x.d, data))).filter(function (k) { return !judged0[k]; });
      var c = {
        changes: x.alts.map(function (a) { return { knob: a.knob, from: a.from, to: a.value }; }),
        verdict: x.cell.verdict, chip: chip.verdict, chip_driver: chip.driver,
        chip_margin: { nom: drv.value.nom, pess: drv.value.pess, opt: drv.value.opt, unit: drv.unit },
        d_area_nom: nomArea(x.d) - area0, losses: null, steps: x.alts.reduce(function (s, a) { return s + a.steps; }, 0),
        estimate: judged.length > 0, judged_inputs: judged, breaks_chip: null
      };
      if (chip.verdict !== 'pass') {
        var dv = drv.value, u = drv.unit ? ' ' + drv.unit : '';
        c.breaks_chip = chip.verdict === 'fail'
          ? 'breaks the chip check: ' + chip.driver + ' fails, margin ' + fmtK(dv.nom) + u + ' nominal (' + fmtK(dv.pess) + u + ' at the pessimistic end)'
          : 'leaves the chip check marginal: ' + chip.driver + ' fails only at the pessimistic end, ' + fmtK(dv.pess) + u + ' (nominal ' + fmtK(dv.nom) + u + ')';
      }
      c.order = x.alts.map(function (a) { return knobIndex(a.knob); }).concat([x.order]);
      c.arch = x.arch;
      return c;
    }
    function lossesOf(c) {
      var ev = evalPoint(c.arch, data, tag), bv = baseVerdicts().verdicts;
      return Object.keys(bv).filter(function (id) { return VERDICT_RANK[ev.verdicts[id]] < VERDICT_RANK[bv[id]]; });
    }
    function cmpKeys(a, b) {
      for (var i = 0; i < a.length; i++) if (a[i] !== b[i]) return a[i] < b[i] ? -1 : 1;
      return 0;
    }
    function key(c, withLoss) {
      return [CHIP_RANK[c.chip], c.estimate ? 1 : 0, c.changes.length, r6(c.d_area_nom), withLoss ? (c.losses ? c.losses.length : 1e9) : 0, c.steps].concat(c.order);
    }
    function rank(cands) {
      cands.sort(function (a, b) { return cmpKeys(key(a, false), key(b, false)); });   // everything but the losses
      cands.slice(0, LOSS_EVAL_MAX).forEach(function (c) { c.losses = lossesOf(c); });
      cands.sort(function (a, b) { return cmpKeys(key(a, true), key(b, true)); });
      return cands;
    }
    function finish(cands) {
      cands.forEach(function (c) { delete c.arch; delete c.order; });
      return cands;
    }

    var singles = tried.filter(function (x) { return x.cell.verdict === 'served'; }).map(candOf);
    var found = singles;
    if (singles.length) {
      out.single = rank(singles);
    } else {
      // step 3: bounded pairs
      var useful = [];
      tried.forEach(function (x) {
        var f1 = failingIds(x.cell), removed = fail0.filter(function (id) { return f1.indexOf(id) < 0; }).length;
        var added = f1.filter(function (id) { return fail0.indexOf(id) < 0; }).length;
        if (added) return;
        if (!removed && !(cell0.verdict === 'marginal' && minPessRel(x.cell) > minPessRel(cell0))) return;
        var met = fail0.reduce(function (sum, id) { return sum + (f1.indexOf(id) < 0 ? limRel(x.cell, id) : 0); }, 0);   // tie-break: how comfortably the removed limits are met
        useful.push({ x: x, removed: removed, gain: limRel(x.cell, cell0.binding) - limRel(cell0, cell0.binding), met: met });
      });
      useful.sort(function (a, b) { return b.removed - a.removed || b.gain - a.gain || b.met - a.met || a.x.order - b.x.order; });
      var top = useful.slice(0, K).map(function (u) { return u.x; }), pairs = [];
      for (var i = 0; i < top.length; i++) for (var j = i + 1; j < top.length; j++) {
        if (top[i].alts[0].knob === top[j].alts[0].knob) continue;
        var pa = top[i].order < top[j].order ? [top[i], top[j]] : [top[j], top[i]];
        var alts = [pa[0].alts[0], pa[1].alts[0]], t = applyAlts(arch, alts), d;
        try { d = derive(t, data); } catch (e) { continue; }
        var cell = evaluateCell(d.arch, wl, data, { derived: d, lite: true });
        if (cell.verdict === 'served') pairs.push(candOf({ alts: alts, arch: t, d: d, cell: cell, order: pa[0].order * 1000 + pa[1].order }));
      }
      out.pairs = rank(pairs);
      found = out.pairs;
    }
    // step 4: nothing found -> the best single alternative per blocking limit
    if (!found.length) {
      out.out_of_reach = true;
      out.blocking.forEach(function (b) {
        var best = null;
        tried.forEach(function (x) {
          var g = limRel(x.cell, b.id) - relClamp(b.rel);
          if (g > 0 && (!best || g > best.g)) best = { g: g, x: x };
        });
        if (!best) return;
        var r = best.x.cell.limits.filter(function (q) { return q.id === b.id; })[0];
        out.hints.push({ limit: b.id, short: b.short, knob: best.x.alts[0].knob, from: best.x.alts[0].from, to: best.x.alts[0].value,
          nom_before: b.nom, nom_after: r ? r.value.nom : null, unit: b.unit, rel_before: b.rel, rel_after: r ? r.rel : null });
      });
    }
    // step 5: a candidate that breaks the chip check names the single knob that restores it
    var shown = 0;
    out.single.concat(out.pairs).forEach(function (c) {
      c.restore = [];
      if (c.chip === 'pass' || shown >= RESTORE_MAX) return;
      shown++;
      alternatives(c.arch, 'full', data).forEach(function (a) {
        if (a.invalid || c.changes.some(function (q) { return q.knob === a.knob; })) return;
        var t = applyAlts(c.arch, [a]);
        try {
          var d = derive(t, data);
          if (chipCheck(d.arch, data, { derived: d, lite: true }).verdict !== 'pass') return;
          if (evaluateCell(d.arch, wl, data, { derived: d, lite: true }).verdict !== 'served') return;
          c.restore.push({ knob: a.knob, from: a.from, to: a.value });
        } catch (e) { /* an alternative that cannot be built restores nothing */ }
      });
    });
    finish(out.single); finish(out.pairs);
    return out;
  }

  // ---------------------------------------------------------------- Timing and Throughput helpers (PLAN-v2 2.6)
  // Input vectors {id: value} at the nominal, pessimistic and optimistic vertex of a limit's inputs.
  function vertexValues(inputs, dirs) {
    var o = { nom: {}, pess: {}, opt: {} };
    Object.keys(inputs).forEach(function (k) {
      var u = inputs[k], dn = (dirs || {})[k] || 'none';
      o.nom[k] = u.nom;
      o.pess[k] = dn === 'none' ? u.nom : u[dn];
      o.opt[k] = dn === 'none' ? u.nom : (dn === 'lo' ? u.hi : u.lo);
    });
    return o;
  }
  function limitById(list, id) { return list.filter(function (L) { return L.id === id; })[0]; }

  // rxSampling(arch, wl, data): the RX eye budget of the rx_eye limit and the oversampling of rx_osr, from their own code.
  function rxSampling(archIn, wlIn, data) {
    var d = derive(archIn, data), c = cellContext(d.arch, wlIn, data, { derived: d });
    var eye = limitById(LIMITS, 'rx_eye').check(c);
    if (eye.na) return { applicable: false, reason: eye.na };
    var V = vertex(eye.inputs, eye.f, 'high'), vv = vertexValues(eye.inputs, V.pess_direction);
    var at = function (f) { return { nom: f(vv.nom), pess: f(vv.pess), opt: f(vv.opt) }; };
    var osr = limitById(LIMITS, 'rx_osr').check(c), have = null;
    if (!osr.na) {
      var ov = vertex(osr.inputs, function (v) { return osr.f(v) + c.wl.osr; }, 'high'), ow = vertexValues(osr.inputs, ov.pess_direction);
      have = { nom: osr.f(ow.nom) + c.wl.osr, pess: osr.f(ow.pess) + c.wl.osr, opt: osr.f(ow.opt) + c.wl.osr };
    }
    return {
      applicable: true, wl: c.wl.id, edge_half: c.edge / 2,
      u: at(function (v) { return rxEyeParts(c, v).u; }), drift: at(function (v) { return rxEyeParts(c, v).drift; }),
      jitter: c.wl.rx_jitter_ns, margin: ivl(V), scale: eye.scale, capture: d.arch.capture,
      osr: { have: have, need: osr.na ? null : c.wl.osr }
    };
  }

  // clockBudget(arch, data): the critical-path composition behind chip item core_clock, at the sign-off corner.
  function clockBudget(archIn, data) {
    var d = derive(archIn, data), corner = d.arch.signoff_corner, ids = d.fn.DpartIds(corner), all = [];
    var LABEL = { base: 'base path (1000 / F_s' + (corner === 'typ' ? ' / r_typ' : '') + ')', imem: 'IMEM access', die: 'die wiring', isa: 'ISA levers (judgement)' };
    var keys = ['base', 'imem', 'die'].concat(ids.isa.length ? ['isa'] : []);
    var parts = keys.map(function (k) {
      var inp = pick(d.params, ids[k]);
      ids[k].forEach(function (i) { if (all.indexOf(i) < 0) all.push(i); });
      var v = ids[k].length ? vertex(inp, function (x) { return d.fn.Dparts(corner, x)[k]; }, 'low') : { nom: 0, pess: 0, opt: 0 };
      return { id: k, label: LABEL[k], ns: ivl(v), ref: ids[k].length ? { kind: 'param', id: ids[k][0] } : null };
    });
    var Dsum = function (x) { var q = d.fn.Dparts(corner, x); return q.base + q.imem + q.die + (q.isa || 0); };
    var D = vertex(pick(d.params, all), Dsum, 'low');
    return { corner: corner, T: d.T, f_core_mhz: d.arch.f_core_mhz, D: ivl(D), fmax: ivl(d.fmax[corner]), parts: parts };
  }

  // hostDemand(arch, wl, data): the host_stream inputs for Throughput.
  function hostDemand(archIn, wlIn, data) {
    var d = derive(archIn, data), c = cellContext(d.arch, wlIn, data, { derived: d }), h = hostDemandOf(c);
    var L = h.dir === 'write' ? d.link.write_MBps : (h.dir === 'read' ? d.link.read_MBps : d.link.both_MBps);
    return { wl: c.wl.id, payload_MBps: h.payload, stream_MBps: h.stream, bw_eff: h.bwEff, dir: h.dir, link: ivl(L), buf_B: h.buf,
      frame_B: c.wl.frame_B, rate_elastic: !!c.wl.rate_elastic };
  }

  // ---------------------------------------------------------------- legacy 19-row model (PLAN 2.9)
  // Port of research/models/protocols.py check(), BASE, TIERS; data in data.legacy19 (imported by the extractor).
  function fmtK(x) {   // 6 significant digits with thousands separators
    var t = String(Number(x.toPrecision(6))).split('.'), neg = t[0].charAt(0) === '-';
    var i = neg ? t[0].slice(1) : t[0];
    return (neg ? '-' : '') + i.replace(/\B(?=(\d{3})+(?!\d))/g, ',') + (t[1] ? '.' + t[1] : '');
  }
  function fmtG(x) { return String(Number(x.toPrecision(6))); }   // Python's %g for the values in this table

  function legacyCheck(req, cap, padToggle) {
    var bit = req[0], tol = req[1], osr = req[2], respNs = req[3], respBuf = req[4], reflexNs = req[5],
      bw = req[6], frameB = req[7], pins = req[8], bidir = req[9];
    var why = [];
    if (2 * bit < padToggle) why.push('pad toggle');
    if (cap.res_ns / 2 > tol) why.push('edge grid ' + fmtG(cap.res_ns) + ' ns');
    if (bit / cap.rx_ns < osr) why.push((bit / cap.rx_ns).toFixed(1) + ' samples/bit');
    if (respNs !== null) {
      var autonomous = cap.react_ns <= respNs && cap.buf_b >= respBuf;
      var viaHost = cap.host_lat_ns + cap.react_ns <= respNs;
      if (!(autonomous || viaHost)) why.push('turnaround');
    }
    if (reflexNs !== null && cap.reflex_ns > reflexNs) why.push('reflex ' + cap.reflex_ns.toFixed(0) + ' ns');
    if (bw > cap.link_mbs && frameB > cap.buf_b) why.push('link ' + fmtG(cap.link_mbs) + ' MB/s');
    if (pins > cap.pins || bidir > cap.bidir) why.push('pins');
    return why;
  }

  // legacy19(data[, arch]): per tier the feasible count and per-row blockers. With `arch`, every row also carries
  // the archsim verdict and binding limit of the cap19 workload of the same name, and `disagree` (legacy T0 feasible
  // vs archsim served) for the Calibration view.
  function legacy19(data, archIn) {
    var Ld = data.legacy19, names = Object.keys(Ld.P);
    var cap = Object.assign({}, Ld.BASE), tiers = [];
    Ld.TIERS.forEach(function (t) {
      cap = Object.assign({}, cap, t[1]);
      var rows = names.map(function (n) { var why = legacyCheck(Ld.P[n], cap, Ld.PAD_TOGGLE_NS); return { name: n, feasible: why.length === 0, blockers: why }; });
      tiers.push({ name: t[0], cap: Object.assign({}, cap), feasible: rows.filter(function (r) { return r.feasible; }).length, rows: rows });
    });
    var count = function (c) { return names.filter(function (n) { return legacyCheck(Ld.P[n], c, Ld.PAD_TOGGLE_NS).length === 0; }).length; };
    var t0 = count(Ld.BASE);
    var levers = Ld.TIERS.slice(1).map(function (t) {
      var n = count(Object.assign({}, Ld.BASE, t[1]));
      return { name: t[0], count: n, delta: n - t0 };
    });
    var out = { counts: tiers.map(function (t) { return t.feasible; }), lever_deltas: levers.map(function (l) { return l.delta; }),
      t0_count: t0, tiers: tiers, levers: levers, expected: Ld.expected };
    out.matches_expected = JSON.stringify(out.counts) === JSON.stringify(Ld.expected.counts) &&
      JSON.stringify(out.lever_deltas) === JSON.stringify(Ld.expected.lever_deltas);
    if (archIn) {
      var ev = evaluateAll(archIn, data, 'cap19');
      out.compare = names.map(function (n, i) {
        var w = data.workloads.filter(function (x) { return x.name === n && (x.tags || []).indexOf('cap19') >= 0; })[0];
        var cell = w ? ev.cells[w.id] : null;
        var legacyOk = tiers[0].rows[i].feasible;
        return { name: n, wl: w ? w.id : null, legacy_t0: legacyOk, legacy_blockers: tiers[0].rows[i].blockers,
          archsim: cell ? cell.verdict : null, binding: cell ? cell.binding : null, blocked_by: cell ? cell.blocked_by : null,
          disagree: cell ? (legacyOk !== (cell.verdict === 'served')) : null };
      });
    }
    return out;
  }

  // ---------------------------------------------------------------- diagram data (PLAN 5.4)
  var PATH_BY_KIND = { pad: 'pad', sync: 'sync', sm: 'sm', out: 'out', ext: 'ext', host: 'host', lane: 'lane' };

  // Segments of a latency path: parts = [[label, kind, duration_ns, ref]]; consecutive from t = 0.
  function segmentsOf(parts) {
    var t = 0;
    return parts.map(function (p) {
      var s = { label: p[0], kind: PATH_BY_KIND[p[1]], t0: t, t1: t + p[2], dur: p[2], ref: p[3] };
      t += p[2];
      return s;
    });
  }

  // Total of a reflex / response path at each vertex: deadline minus the limit's margin.
  function pathTotal(arch, wl, data, lid, deadline) {
    var cell = evaluateCell(arch, wl, data, {});
    var r = cell.limits.filter(function (x) { return x.id === lid; })[0];
    return { total: vertexTriple(function (v) { return deadline - r.value[v]; }), margin: ivl(r.value), flags: cell.flags };
  }

  // timeline(arch, wl, data): {kind: 'reflex' | 'response' | 'none', deadline, segments, total, margin, lane, active, T, ticks}
  function timeline(archIn, wlIn, data) {
    var c = cellContext(archIn, wlIn, data, {}), wl = c.wl, d = c.d, P = c.P, T = c.T;
    var kind = (wl.reflex_ns != null || wl.reflex_bits != null) ? 'reflex' : ((wl.resp_ns != null || wl.resp_bits != null) ? 'response' : 'none');
    var out = { wl: wl.id, kind: kind, deadline: null, segments: [], total: null, margin: null, lane: null, active: 'sampled', T: T, ticks: [] };
    if (kind === 'none') { out.reason = 'the workload has no reflex or response deadline'; return out; }
    var half = P.t_pad_rt_ns.nom / 2;
    var sm = kind === 'reflex' ? c.reflex_sm : c.decide_sm;
    var parts = [
      ['pad in', 'pad', half, { kind: 'param', id: 't_pad_rt_ns' }],
      ['sync phase', 'sync', P.sync_phase_cyc.nom * T, { kind: 'param', id: 'sync_phase_cyc' }],
      ['synchronizer', 'sync', c.TIN * T, { kind: 'iss', id: 'T_IN' }],
      ['decision', 'sm', sm * T, { kind: 'workload_field', wl: wl.id, field: kind === 'reflex' ? 'reflex_sm' : 'decide_sm' }],
      ['output register', 'out', c.TOUT * T, { kind: 'iss', id: 'T_OUT' }],
      ['pad out', 'pad', half, { kind: 'param', id: 't_pad_rt_ns' }]];
    if (kind === 'reflex') parts.push(['skew + external setup', 'ext', P.t_skew_ns.nom + P.t_setup_ext_ns.nom, { kind: 'param', id: 't_skew_ns' }]);
    var onChip = true;
    if (kind === 'response') {
      onChip = d.buffers.buf_tx_bytes >= wl.resp_buf_B;
      if (!onChip) parts.push(['host decision loop', 'host', nomOf(d.link.t_host_loop), { kind: 'derived', id: 't_host_loop' }]);
    }
    out.deadline = kind === 'reflex' ? c.records.reflex_deadline_ns.value : c.records.response_deadline_ns.value;
    out.limit = kind;
    out.segments = segmentsOf(parts);
    var laneCapable = wl.clocking === 'bus_clock_in' && wl.role === 'target' && kind === 'reflex';
    var sampledArch = Object.assign({}, d.arch, { lane: false });
    var pt = pathTotal(sampledArch, wl, data, kind, out.deadline);
    out.total = pt.total; out.margin = pt.margin; out.on_chip = onChip;
    if (laneCapable && d.arch.lane) {
      var lp = segmentsOf([
        ['pad round trip', 'pad', P.t_pad_rt_ns.nom, { kind: 'param', id: 't_pad_rt_ns' }],
        ['lane internal', 'lane', P.t_int_ns.nom, { kind: 'param', id: 't_int_ns' }],
        ['skew + external setup', 'ext', P.t_skew_ns.nom + P.t_setup_ext_ns.nom, { kind: 'param', id: 't_skew_ns' }]]);
      var lt = pathTotal(Object.assign({}, d.arch, { lane: true }), wl, data, 'reflex', out.deadline);
      out.lane = { segments: lp, total: lt.total, margin: lt.margin };
      out.active = 'lane';
    }
    // core clock ticks: the first tick closes the sync-phase wait; then one per period over the path
    var tEnd = out.segments[out.segments.length - 1].t1, t0 = out.segments[1].t1;
    for (var t = t0; t <= Math.max(tEnd, out.deadline) + TICK_EPS; t += T) out.ticks.push(t);
    return out;
  }

  // edgeGrid(arch, wl, data, nEdges): the shortest-edge ladder k * edge placed on the TX grid.
  function edgeGrid(archIn, wlIn, data, nEdges) {
    var c = cellContext(archIn, wlIn, data, {}), wl = c.wl, d = c.d, n = nEdges || 8;
    var out = { wl: wl.id, applicable: false, ideal: [], placed: [], error: [], tick_ns: nomOf(d.T_tx), tol_ns: wl.tx_tol_ns, edge_ns: c.edge,
      integer_grid: null, max_error_ns: null, limit: null, ticks: [] };
    if (wl.clocking !== 'own' || ROLES_TX.indexOf(wl.role) < 0) {
      out.reason = 'clocking ' + wl.clocking + ', role ' + wl.role + ': no TX edge grid of ours';
      return out;
    }
    var tick = out.tick_ns;
    out.applicable = true;
    out.integer_grid = isSnap(c, c.edge, tick);
    for (var k = 0; k < n; k++) {
      var ideal = k * c.edge, placed = Math.round(ideal / tick) * tick;
      out.ideal.push(ideal); out.placed.push(placed); out.error.push(placed - ideal);
    }
    out.max_error_ns = Math.max.apply(null, out.error.map(Math.abs));
    var last = out.ideal[n - 1] + tick;
    if (last / tick <= 400) for (var g = 0; g * tick <= last; g++) out.ticks.push(g * tick);
    var L = LIMITS.filter(function (x) { return x.id === 'tx_edge'; })[0], t = limitTriple(L, c);
    out.limit = t ? { nom: t.nom, pess: t.pess, opt: t.opt } : null;
    return out;
  }

  // streamTrace(arch, wl, data, nPoints): FIFO fill vs time over one frame (or four buffer drains for a continuous stream).
  function streamTrace(archIn, wlIn, data, nPoints) {
    var c = cellContext(archIn, wlIn, data, {}), wl = c.wl, d = c.d, n = nPoints || 50;
    var bwEff = wl.bw_MBps + (c.stream ? 2 * c.ipu * wl.rate_hz / 1e6 : 0);
    var out = { wl: wl.id, kind: 'none', t_us: [], fill_B: [], fill_B_pess: [], fill_B_opt: [] };
    if (!(bwEff > 0)) { out.reason = 'no sustained bandwidth need'; return out; }
    var dir = wl.bw_dir, rx = dir === 'read';
    var buf = dir === 'write' ? d.buffers.buf_tx_bytes : (rx ? d.buffers.buf_rx_bytes : Math.min(d.buffers.buf_tx_bytes, d.buffers.buf_rx_bytes));
    var link = ivl(dir === 'write' ? d.link.write_MBps : (rx ? d.link.read_MBps : d.link.both_MBps));
    var frameUs = wl.frame_B != null ? wl.frame_B / bwEff : null;
    var horizon = frameUs != null ? frameUs : 4 * buf / bwEff;
    out.kind = rx ? 'rx' : 'tx'; out.dir = dir; out.bw_eff_MBps = bwEff; out.link_MBps = link; out.buf_B = buf;
    out.frame_B = wl.frame_B; out.frame_us = frameUs; out.horizon_us = horizon; out.elastic = !!wl.rate_elastic;
    // fill(t): tx starts full and drains at (bw - L) until empty (underrun); rx starts empty and fills at (bw - L) until full (overflow)
    var fillAt = function (L, t) {
      var net = bwEff - L;
      if (net <= 0) return rx ? 0 : buf;
      return rx ? Math.min(buf, net * t) : Math.max(0, buf - net * t);
    };
    for (var i = 0; i < n; i++) {
      var t = n > 1 ? horizon * i / (n - 1) : 0;
      out.t_us.push(t);
      out.fill_B.push(fillAt(link.nom, t)); out.fill_B_pess.push(fillAt(link.pess, t)); out.fill_B_opt.push(fillAt(link.opt, t));
    }
    var evAt = function (L) { return bwEff - L > 0 ? buf / (bwEff - L) : null; };
    var tev = evAt(link.nom);
    out.event = { kind: tev === null ? null : (rx ? 'overflow' : 'underrun'), t_us: tev, bytes_streamed: tev === null ? null : bwEff * tev,
      t_us_pess: evAt(link.pess), t_us_opt: evAt(link.opt) };
    out.l_max_B = tev === null ? Infinity : bwEff * tev;
    out.ok = out.elastic || tev === null || frameUs === null ? true : tev >= frameUs;
    return out;
  }

  // Which limits each block of the diagram is responsible for (for outlining the binding block).
  var BLOCK_LIMITS = {
    link: ['host_stream', 'response'], deep_fifo: ['host_stream', 'response', 'program_words'],
    engines: ['cycles', 'program_words', 'engines', 'isa_xor', 'rx_osr', 'rate_quant'], lane: ['reflex'],
    ddr_io: ['tx_edge', 'rx_osr', 'pad_toggle_out', 'pad_toggle_in'], dtc: ['tx_edge', 'rx_eye', 'rx_timestamp'],
    dpll: ['rx_eye'], adpll: ['rx_osr', 'rx_eye', 'rx_timestamp'], crc: ['isa_xor'], pinmux: ['pins'],
    pads: ['pad_toggle_out', 'pad_toggle_in', 'round_trip', 'reflex', 'response']
  };
  function chipItemsOf(id, hasArea) { return (id === 'pinmux' ? ['pins_budget'] : []).concat(hasArea ? ['die_area'] : []); }
  var ENGINE_LINE_IDS = ['engine_base', 'imem', 'imem_macro', 'sram_wrapper', 'fifo', 'crc', 'dpll', 'isa_xor_alu', 'isa_cond_logic', 'isa_mov_src_mux', 'isa_decode_24', 'isa_decode_32'];

  // blockDiagram(arch, data): boxes with placed area (raw synth area x P&R factor, macro area x halo) and enabled flags,
  // the per-line area waterfall (sums to area_total) and the 24-pin allocation.
  function blockDiagram(archIn, data) {
    var d = derive(archIn, data), a = d.arch, P = d.params, core = d.area.core;
    var fac = function (basis) { return P[basis === 'macro' ? 'macro_halo' : 'pnr_factor']; };
    var lines = d.area.blocks.map(function (b) {
      var area = b.area_placed;
      return { id: b.id, label: b.label, basis: b.basis, count: b.count, note: b.note, factor: b.basis === 'macro' ? 'macro_halo' : 'pnr_factor',
        raw: ivl(b.area_cell), area_cell: ivl(b.area_cell), area_placed: area, area: area, pct_core: area.nom / core * 100 };
    });
    var sum = function (ids) {
      var hit = lines.filter(function (l) { return ids.indexOf(l.id) >= 0; });
      return hit.length ? vertexTriple(function (v) { return hit.reduce(function (s, l) { return s + l.area[v]; }, 0); }) : null;
    };
    var box = function (id, label, kind, enabled, area, extra) {
      var b = { id: id, label: label, kind: kind, enabled: enabled, area: enabled ? area : null, pct_core: enabled && area ? area.nom / core * 100 : null,
        limits: BLOCK_LIMITS[id] || [], chip_items: chipItemsOf(id, enabled && !!area) };
      return Object.assign(b, extra || {});
    };
    var link = data.links[a.host_link], E = a.n_engines;
    var blocks = [
      box('host_mcu', 'Host MCU', 'external', true, null),
      box('link', link.label, 'link', true, sum([link.area_block]),
        { link: { id: link.id, kind: link.kind, write_MBps: d.link.write_MBps, read_MBps: d.link.read_MBps, t_host_loop: d.link.t_host_loop } }),
      box('regfile', 'Register file', 'regfile', true, null, { note: 'inside the engine control logic area' })];
    var engArea = sum(ENGINE_LINE_IDS);
    var engines = [];
    for (var i = 0; i < E; i++) {
      var per = engArea ? vertexTriple(function (v) { return engArea[v] / E; }) : null;
      var e = { id: 'engine_' + i, index: i, imem_words: a.imem_words, imem_banks: a.imem_banks, imem_tech: a.imem_tech, fifo_depth: a.fifo_depth,
        crc: a.crc_unit, dpll: a.capture === 'dpll', area: per, pct_core: per ? per.nom / core * 100 : null, limits: BLOCK_LIMITS.engines, chip_items: chipItemsOf('engine', !!per) };
      engines.push(e);
      blocks.push(Object.assign({ kind: 'engine', enabled: true, label: 'Engine ' + i }, e));
    }
    blocks.push(box('pinmux', 'Pin mux', 'pinmux', true, sum(['pinmux_1', 'pinmux_2', 'pinmux_3', 'pinmux_4', 'pinmux'])));
    blocks.push(box('deep_fifo', 'Deep FIFO (' + a.deep_fifo_words + ' words, ' + a.deep_fifo_dir + ')', 'memory', a.deep_fifo_words > 0, sum(['deep_fifo_ctl', 'deep_macro'])));
    blocks.push(box('adpll', 'ADPLL x' + a.adpll_mult, 'clock', a.capture === 'adpll', sum(['adpll_ctl', 'adpll_dco_n' + a.adpll_mult])));
    blocks.push(box('dpll', 'DPLL capture', 'clock', a.capture === 'dpll', sum(['dpll'])));
    blocks.push(box('lane', 'Bus-clocked lane', 'lane', !!a.lane, sum(['lane'])));
    blocks.push(box('ddr_io', 'Both-edge I/O', 'pad', !!a.ddr_io, sum(['ddr_io'])));
    blocks.push(box('dtc', 'Delay-line placement (DTC)', 'pad', a.fine_edge === 'dtc', sum(['dtc'])));
    blocks.push(box('crc', 'CRC/LFSR units', 'engine_part', !!a.crc_unit, sum(['crc'])));
    blocks.push(box('isa_irq', 'IRQ flags and wait logic (ISA)', 'engine_part', !!a.isa_irq, sum(['isa_irq_logic'])));
    EXTRA_IDS.forEach(function (id) {
      blocks.push(box(id, data.blocks[id].label, 'extra', a.extras.indexOf(id) >= 0, sum([id])));
    });
    // 24 project pins: ui_in 0-7 (input only), uo_out 0-7 (output only), uio 0-7 (tri-state)
    var bankSize = { ui_in: data.iss.pins.ui_in, uo_out: data.iss.pins.uo_out, uio: data.iss.pins.uio }, use = {}, key = function (bank, i) { return bank + ':' + i; };
    for (var h = 0; h < link.pins_in; h++) use[key('ui_in', h)] = 'host';
    for (h = 0; h < link.pins_out; h++) use[key('uo_out', h)] = 'host';
    var reserve = [['uo_out', bankSize.uo_out - 1], ['ui_in', bankSize.ui_in - 1], ['uio', bankSize.uio - 1]], left = a.pins_reserved;
    reserve.forEach(function (b) {
      for (var j = b[1]; j >= 0 && left > 0; j--) if (!use[key(b[0], j)]) { use[key(b[0], j)] = 'reserved'; left--; }
    });
    if (a.lane) {
      for (var q = 0; q < bankSize.ui_in; q++) if (!use[key('ui_in', q)]) { use[key('ui_in', q)] = 'lane_clk'; break; }
    }
    var pins = [], pinBase = 0;
    ['ui_in', 'uo_out', 'uio'].forEach(function (bank) {
      for (var j = 0; j < bankSize[bank]; j++) pins.push({ pin: pinBase + j, bank: bank, index: j, use: use[key(bank, j)] || 'free', tristate: bank === 'uio' });
      pinBase += bankSize[bank];
    });
    return { arch: a, blocks: blocks, engines: engines, area_lines: lines, area_total: ivl(d.area.total), core_area: core,
      area_limit: ivl(d.area.limit), util_pct: ivl(d.area.util_pct), pins: pins, pins_summary: d.pins, link: d.link,
      fmax: { typ: ivl(d.fmax.typ), slow: ivl(d.fmax.slow) } };
  }

  // ---------------------------------------------------------------- calibration (PLAN 3.5, Appendix C.5)
  function relErr(model, measured) {
    if (measured === model) return 0;
    if (measured === 0 || !isFinite(model) || !isFinite(measured)) return null;
    return (model - measured) / Math.abs(measured) * 100;
  }
  var CLASS_RANK = { match: 0, close: 1, off: 2 };
  function classOfError(err, absDiff, decimals) {
    if (decimals != null && absDiff <= 0.5 * Math.pow(10, -decimals) * (1 + 1e-6)) return 'match';  // equal at the printed precision
    if (err === null) return 'off';
    var a = Math.abs(err);
    return a <= 1 ? 'match' : (a <= 10 ? 'close' : 'off');
  }
  function num4(x) { return x === null || x === undefined ? '-' : (isFinite(x) ? String(Number(x.toPrecision(7))) : (x > 0 ? 'inf' : '-inf')); }

  // One compared quantity: model is a number or {nom, pess, opt}; measured a number. Optional: decimals (printed
  // precision of `measured`), scored (default true), cls (forced class), note.
  function cmp(label, model, measured, unit, o) {
    o = o || {};
    var m = typeof model === 'number' ? { nom: model, pess: model, opt: model } : model;
    var err = relErr(m.nom, measured);
    var it = { label: label, model: m, measured: measured, unit: unit, error_pct: err, scored: o.scored !== false, note: o.note || '' };
    it.class = o.cls || classOfError(err, Math.abs(m.nom - measured), o.decimals);
    return it;
  }
  function boolItem(label, modelYes, measuredYes, note) {
    return { label: label, model: modelYes ? 'yes' : 'no', measured: measuredYes ? 'yes' : 'no', unit: '', error_pct: null, scored: true,
      class: modelYes === measuredYes ? 'match' : 'off', note: note || '' };
  }
  function itemText(it, side) {
    var v = it[side];
    return typeof v === 'object' && v !== null ? num4(v.nom) : (typeof v === 'number' ? num4(v) : String(v));
  }
  function calRow(id, quantity, env, unit, items, source, o) {
    o = o || {};
    var scored = items.filter(function (i) { return i.scored && CLASS_RANK[i.class] !== undefined; });
    var cls = o.cls;
    if (!cls) {
      cls = 'match';
      scored.forEach(function (i) { if (CLASS_RANK[i.class] > CLASS_RANK[cls]) cls = i.class; });
    }
    var worst = null;
    scored.forEach(function (i) { if (i.error_pct !== null && (worst === null || Math.abs(i.error_pct) > Math.abs(worst))) worst = i.error_pct; });
    var single = items.length === 1;
    return { id: id, quantity: quantity, env: env, unit: unit, class: cls, items: items, source: source, note: o.note || '',
      model: o.model || (single && typeof items[0].model === 'object' ? items[0].model : { nom: null, pess: null, opt: null }),
      measured: single ? items[0].measured : null,
      error_pct: o.error_pct !== undefined ? o.error_pct : (single ? items[0].error_pct : worst),
      model_text: items.map(function (i) { return itemText(i, 'model'); }).join(' / '),
      measured_text: items.map(function (i) { return itemText(i, 'measured'); }).join(' / ') };
  }

  function wlById(data, id) {
    var w = data.workloads.filter(function (x) { return x.id === id; })[0];
    if (!w) throw new Error('unknown workload ' + id);
    return w;
  }
  function limitOf(cell, lid) {
    var r = cell.limits.filter(function (x) { return x.id === lid; })[0];
    if (!r) throw new Error(lid + ' does not apply to ' + cell.wl);
    return r.value;
  }
  function fixedParams(data, ids, vals) {   // param_overrides pinning each id to one value
    var o = {};
    ids.forEach(function (id, i) { var v = vals ? vals[i] : data.params[id].nom; o[id] = { lo: v, nom: v, hi: v }; });
    return o;
  }

  function calibration(data) {
    var CI = data.calibration_inputs, rows = [];
    var A0 = defaultArch(), AI = Object.assign(defaultArch(), { env: 'iss' });
    var d0 = derive(A0, data);
    var hp = function (id) { return data.hostport.filter(function (x) { return x.id === id; })[0]; };
    var MHz = function (hz) { return hz / 1e6; };

    // K1-K4: SPI host-port framing (iss; windows do not depend on the environment)
    var Ld = d0.link, PERF_B = 'research/perf/b/perf.csv (spi_host_port), tools/archsim/notes/perf-a-vs-b.md section 3';
    rows.push(calRow('K1', 'SPI write CSn window, 1 word', 'iss', 'cycles', [cmp('write_window(1)', Ld.write_window_1, hp('write_1w').b, 'cycles')], PERF_B));
    rows.push(calRow('K2', 'TXF0 4-word burst CSn window', 'iss', 'cycles', [cmp('write_window(4)', Ld.write_window_B, hp('txf0_burst_4').b, 'cycles')], PERF_B));
    rows.push(calRow('K3', 'h of a 1-word write / of a 28-word IMEM burst', 'iss', 'cycles',
      [cmp('h_write(1)', Ld.h_write_1, hp('write_1w').a, 'cycles'), cmp('h_write(28)', hWrite(data.links[A0.host_link], 28), hp('imem_burst_28w').a, 'cycles')],
      'research/perf/a/perf.csv (spi_reg_write_core_cycles, spi_imem_burst_28_words_core_cycles)'));
    rows.push(calRow('K4', 'SPI read CSn window, 1 word', 'iss', 'cycles', [cmp('read_window(1)', Ld.read_window_1, hp('read_1w').b, 'cycles',
      { note: hp('read_1w').note })], PERF_B, { note: 'harness defect in perf-b: ' + hp('read_1w').note }));

    // K5, K6: device-mode maximum rates
    var wT = wlById(data, CI.k5.workload);
    rows.push(calRow('K5', 'SPI target max SCK', 'iss', 'MHz',
      [cmp('max SCK', vertexTriple(function (v) { return MHz(maxRate(AI, wT, data, v).rate_hz); }), MHz(wT.measured.iss_max_rate_hz), 'MHz')], wT.measured.src));
    var wI = wlById(data, CI.k6.workload), cellI = evaluateCell(A0, wI, data, {});
    rows.push(calRow('K6', 'I2C target max SCL', 'iss', 'kHz',
      [{ label: 'max SCL', model: 'none', measured: wI.measured.iss_max_rate_hz / 1e3, unit: 'kHz', error_pct: null, scored: false, class: 'not_modelled',
        note: wI.measured.note }], 'fw/i2c_target/README.md', { cls: 'not_modelled',
        note: 'the model has no limit below ' + MHz(wI.rate_hz) + ' MHz (verdict ' + cellI.verdict + ' at that rate): the measured 600 kHz is a host-feed and master-side limit',
        model: { nom: null, pess: null, opt: null } }));

    // K7: lane f_std, pad fixed at its nominal (the C1 projection's 20 ns)
    var padFix = fixedParams(data, ['t_pad_rt_ns']);
    var typA = Object.assign(defaultArch(), { lane: true, param_overrides: padFix });
    var slowA = Object.assign(defaultArch(), { lane: true, param_overrides: Object.assign({}, padFix, fixedParams(data, ['t_int_ns'], [data.params.t_int_ns.hi])) });
    var ci7 = CI.k7;
    rows.push(calRow('K7', 'Lane f_std, typ / slow', 'silicon, pad fixed', 'MHz',
      [cmp('typ (t_int nom)', vertexTriple(function (v) { return MHz(maxRate(typA, wT, data, v).rate_hz); }), ci7.typ_MHz, 'MHz', { decimals: ci7.decimals }),
        cmp('slow (t_int hi)', vertexTriple(function (v) { return MHz(maxRate(slowA, wT, data, v).rate_hz); }), ci7.slow_MHz, 'MHz', { decimals: ci7.decimals })],
      ci7.src, { note: 'same formula as the C1 projection; t_int is the measured input' }));

    // K8: perf14 maximum rates vs perf-b (iss; the rate-setting limits only: cycles and the protocol ceiling)
    var k8 = [];
    data.workloads.forEach(function (w) {
      if ((w.tags || []).indexOf('mvp_fw') < 0 || !w.measured) return;
      var model = vertexTriple(function (v) { return maxRate(AI, w, data, v, { only: ['cycles', 'rate_ceiling'] }).rate_hz; });
      var b = w.measured.b_max_bit_rate, nm = w.id.replace(/^fw_/, '');
      if (b === null || b === undefined) {
        k8.push({ label: nm, model: model, measured: null, unit: 'bit/s', error_pct: null, scored: false, class: 'definitional', note: 'perf-b has no row for this program' });
      } else if (w.measured.class === 'DEF') {
        k8.push(cmp(nm, model, b, 'bit/s', { scored: false, cls: 'definitional', note: 'perf-a/perf-b class DEF: the two sides count different quantities' }));
      } else {
        k8.push(cmp(nm, model, b, 'bit/s', { note: 'perf-a/perf-b class ' + w.measured.class }));
      }
    });
    rows.push(calRow('K8', 'perf14 max bit rate: min(f_core / ipu, protocol ceiling)', 'iss', 'bit/s', k8,
      'research/perf/b/perf.csv max_bit_rate; tools/archsim/notes/perf-a-vs-b.md section 1',
      { note: 'row class = worst scored row; per-row classes in items. DEF rows are not scored.' }));

    // K9: perf-a tx_fifo_sustainable_over_spi: can one SPI write of one word per FIFO word keep the TX FIFO fed at the fastest divider?
    var k9 = CI.k9.rows.map(function (r) {
      var w = wlById(data, r.workload);
      var fast = d0.arch.f_core_mhz * 1e6 / w.ipu;                   // bit rate at the fastest divider
      var need = w.bw_MBps * fast / w.rate_hz;                       // MB/s the wire then drains
      var link1 = 4 / (d0.link.h_write_1 * d0.T) * 1000;             // MB/s of back-to-back 1-word writes (h to h)
      return boolItem(r.workload.replace(/^fw_/, '') + (r.variant.indexOf('mode 3') >= 0 ? ' (mode 3 row, same program)' : ''), link1 >= need, r.a_sustainable,
        'link ' + num4(link1) + ' MB/s vs wire ' + num4(need) + ' MB/s at ' + num4(MHz(fast)) + ' Mbit/s');
    });
    rows.push(calRow('K9', 'TX FIFO sustainable over SPI (perf-a yes/no)', 'iss', '', k9, CI.k9.src,
      { note: k9.filter(function (i) { return i.class === 'match'; }).length + '/' + k9.length + ' agree' }));

    // K10-K13: area and fmax against the flow
    var pnr = data.params.pnr_factor, I10 = CI.k10, I11 = CI.k11, I12 = CI.k12, I13 = CI.k13;
    rows.push(calRow('K10', 'MVP area', 'silicon', 'um2', [cmp('area_total', ivl(d0.area.total), I10.area_um2, 'um2', { cls: 'identity' })], I10.src,
      { cls: 'identity', note: 'the measured point that fits pnr_factor: agreement is by construction' }));
    var synthTimes = function (synth) { return vertex({ pnr_factor: pnr }, function (v) { return synth * v.pnr_factor; }, 'low'); };
    var m11 = synthTimes(I11.synth_um2), m12 = synthTimes(I12.synth_um2);
    rows.push(calRow('K11', 'IMEM probe 32x24 flop block area', 'silicon', 'um2', [cmp('synth x pnr_factor', ivl(m11), I11.flow_um2, 'um2')],
      I11.flow_src, { note: 'synth ' + I11.synth_um2 + ' um2 x ' + num4(pnr.nom) + '; the flow figure uses default hold margins' }));
    rows.push(calRow('K12', 'Pin-mux 4-SM block area', 'silicon', 'um2', [cmp('synth x pnr_factor', ivl(m12), I12.flow_um2, 'um2')],
      I12.flow_src, { note: 'synth ' + I12.synth_um2 + ' um2 x ' + num4(pnr.nom) }));
    rows.push(calRow('K13', 'MVP fmax typ / slow', 'silicon', 'MHz',
      [cmp('typ', ivl(d0.fmax.typ), I13.fmax_typ_mhz, 'MHz', { cls: 'identity' }), cmp('slow', ivl(d0.fmax.slow), I13.fmax_slow_mhz, 'MHz', { cls: 'identity' })],
      I13.src, { cls: 'identity', note: 'the band-none slow fmax and r_typ_slow are these two measurements' }));

    // K14: legacy 19-row tier counts
    var lg = legacy19(data), ex = data.legacy19.expected;
    var k14 = lg.counts.map(function (n, i) { return cmp('tier T' + i, n, ex.counts[i], 'rows'); })
      .concat(lg.lever_deltas.map(function (n, i) { return cmp('lever T' + (i + 1) + ' alone', n, ex.lever_deltas[i], 'rows'); }));
    rows.push(calRow('K14', '19-row legacy tier counts', 'n/a', 'rows', k14, data.legacy19.src));

    // K15-K17: the stretch brief's tables under the brief's own assumptions
    var zero = fixedParams(data, ['t_clk_jitter_ns', 't_skew_ns'], [0, 0]);
    var AB = Object.assign(defaultArch(), { param_overrides: zero });
    var I15 = CI.k15, I16 = CI.k16, I17 = CI.k17;
    var k15 = I15.rows.map(function (r) {
      var cell = evaluateCell(AB, r.workload, data, { force_snap: true });
      return cmp(r.label, ivl(limitOf(cell, 'tx_edge')), r.measured, 'ns', { decimals: I15.decimals, note: 'workload ' + r.workload });
    });
    rows.push(calRow('K15', 'Stretch TX margins (budget.py table 1), snap always applied, no jitter, no pair skew', 'silicon, overrides', 'ns', k15, I15.src,
      { note: 'overrides: t_clk_jitter_ns = 0, t_skew_ns = 0, force_snap' }));
    var k16 = I16.rows.map(function (r) {
      var arch = r.label.indexOf('ideal DPLL') >= 0 ? Object.assign(defaultArch(), { capture: 'dpll' }) : A0;
      var w = wlById(data, r.workload);
      if (r.rate_hz) w = Object.assign({}, w, { rate_hz: r.rate_hz });
      return cmp(r.label, ivl(limitOf(evaluateCell(arch, w, data, {}), 'rx_eye')), r.measured, 'ns', { decimals: I16.decimals, note: 'workload ' + r.workload + (r.rate_hz ? ' at ' + MHz(r.rate_hz) + ' Mbit/s' : '') });
    });
    rows.push(calRow('K16', 'Stretch RX eye (budget.py table 2)', 'silicon', 'ns', k16, I16.src, { note: 'the brief prints one decimal; equal at that precision counts as match' }));
    var k17 = [];
    I17.rows.forEach(function (r) {
      var w = wlById(data, r.workload);
      I17.link_MBps.forEach(function (L, j) {
        var margin = limitOf(evaluateCell(A0, w, data, { link_override_MBps: L }), 'host_stream');
        var lmax = margin.nom === Infinity ? Infinity : margin.nom + w.frame_B;
        k17.push(cmp(r.label + ' @' + num4(L) + ' MB/s', lmax, r.measured[j], 'B', { decimals: I17.decimals, note: 'workload ' + r.workload }));
      });
    });
    rows.push(calRow('K17', 'Stretch L_max (budget.py table 4) at the brief\'s links', 'silicon, overrides', 'B', k17, I17.src,
      { note: 'link_override_MBps ' + I17.link_MBps.map(num4).join(' and ') + '; the brief prints whole bytes' }));

    // K18: USB-LS host RX lossless device-clock window
    var I18 = CI.k18, tol = rxTolerance(AI, I18.workload, data);
    rows.push(calRow('K18', 'USB-LS host RX: lossless device-clock error, lock once per packet', 'iss', '%',
      [cmp('low side', -tol, I18.lo_pct, '%'),
        cmp('high side', tol, I18.hi_pct, '%', { scored: false, note: I18.note })],
      I18.src, { note: 'model is symmetric (+-' + num4(tol) + ' %); ' + I18.note + ', so only the low side is scored' }));

    // K19: 10BASE-T RX vs Monte Carlo: the model's zero-margin jitter with and without capture brackets the MC failure onset
    var I19 = CI.k19, jNo = rxJitterBreakEven(A0, I19.workload, data), jDpll = rxJitterBreakEven(Object.assign(defaultArch(), { capture: 'dpll' }), I19.workload, data);
    var brackets = jNo <= I19.measured_J_ns && I19.measured_J_ns <= jDpll;
    rows.push(calRow('K19', '10BASE-T RX vs Monte Carlo: jitter at which the eye closes', 'silicon', 'ns',
      [{ label: 'no capture', model: jNo, measured: I19.measured_J_ns, unit: 'ns', error_pct: null, scored: false, class: 'info', note: '' },
        { label: 'ideal DPLL', model: jDpll, measured: I19.measured_J_ns, unit: 'ns', error_pct: null, scored: false, class: 'info', note: '' }],
      I19.src, { cls: brackets ? 'brackets' : 'off', model: { nom: null, pess: jNo, opt: jDpll }, error_pct: null,
        note: I19.note + '; the model bounds the Monte Carlo between no capture and an ideal DPLL' }));

    // K20: core clock vs typ fmax
    var I20 = CI.k20, A20 = Object.assign(defaultArch(), { f_core_mhz: I20.f_core_mhz });
    var cc = chipCheck(A20, data).items.filter(function (x) { return x.id === 'core_clock'; })[0];
    rows.push(calRow('K20', 'Core clock at ' + I20.f_core_mhz + ' MHz vs typ fmax (margin)', 'silicon', 'MHz',
      [cmp('core_clock margin', ivl(cc.value), I20.margin_mhz, 'MHz', { decimals: I20.decimals })], I20.src));
    return rows;
  }

  return {
    VERSION: VERSION, toJSON: toJSON, KNOBS: KNOBS, GROUPS: GROUPS, EXTRA_IDS: EXTRA_IDS,
    defaultArch: defaultArch, isaActiveLevers: isaActiveLevers, applyPreset: applyPreset, resolveParams: resolveParams, checkArch: knobProblems,
    vertex: vertex, derive: derive, refResolves: refResolves,
    limits: limits, chipItems: chipItems, chipCheck: chipCheck, evaluateCell: evaluateCell, evaluateAll: evaluateAll, sensitivity: sensitivity,
    writeWindow: writeWindow, readWindow: readWindow, hWrite: hWrite, pickMacro: pickMacro,
    maxRate: maxRate, sweep: sweep, pareto: pareto, mix: mix, pairs: pairs, maxConcurrent: maxConcurrent, rxTolerance: rxTolerance, rxJitterBreakEven: rxJitterBreakEven,
    alternatives: alternatives, knobImpact: knobImpact, whatWouldItTake: whatWouldItTake,
    rxSampling: rxSampling, clockBudget: clockBudget, hostDemand: hostDemand,
    calibration: calibration, legacy19: legacy19, timeline: timeline, edgeGrid: edgeGrid, streamTrace: streamTrace, blockDiagram: blockDiagram
  };
})();
if (typeof module !== 'undefined') { module.exports = ArchSim; }
