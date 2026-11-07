/* Throughput tab: host link card, the focused protocol's host demand (payload + streamed instruction words),
 * and the host-stream FIFO fill trace with its pess/opt envelope. Every number comes from ArchSim (derive().link,
 * hostDemand, streamTrace); nothing is re-derived here except unit scaling and a presentation comparison of the link band
 * against the need. Derivations open through the core `record` side kind.
 */
(function (U) {
  'use strict';
  var A = (typeof ArchSim !== 'undefined') ? ArchSim : null;
  var esc = U.esc, fmtNum = U.fmtNum, fmtBandHtml = U.fmtBandHtml;

  function scaled(b, k) { return b ? { nom: b.nom / k, pess: b.pess / k, opt: b.opt / k } : null; }
  function recRef(id, col) { return { kind: 'record', id: id, col: col || 0 }; }

  // ------------------------------------------------------------ link card
  function linkCard(state) {
    var col = state.focus.col, lk = state.point.derived.link, none = !(lk.write_MBps.nom > 0);
    var h = '<section class="tp-sec tp-link" data-link-card><h3>Host link <span class="muted">' + esc(lk.id) + ', ' + esc(lk.kind) +
      (lk.width_bits ? ', ' + esc(lk.width_bits) + ' bit' : '') + '</span></h3>';
    if (none) {
      return h + '<p class="muted">This link only loads the program: its pins are handed to protocols in run mode, so there is no run-mode host bandwidth.</p></section>';
    }
    h += '<div class="stats">' +
      U.stat('Write', fmtBandHtml(lk.write_MBps, 'MB/s', 4), '', recRef('link_write_MBps', col)) +
      U.stat('Read', fmtBandHtml(lk.read_MBps, 'MB/s', 4), '', recRef('link_read_MBps', col)) +
      U.stat('Half duplex', fmtBandHtml(lk.both_MBps, 'MB/s', 4), 'read and write share the pins', recRef('link_both_MBps', col)) +
      U.stat('Host decision loop', fmtBandHtml(scaled(lk.t_host_loop, 1000), 'us', 2), '', recRef('t_host_loop', col)) +
      (lk.t_imem_load_32 ? U.stat('IMEM page load', fmtBandHtml(scaled(lk.t_imem_load_32, 1000), 'us', 1), 'one full IMEM', recRef('t_imem_load_32', col)) : '') +
      '</div>';
    if (lk.B_w != null) {
      h += '<p class="muted">Burst size: ' + esc(lk.B_w) + ' words per write window, ' + esc(lk.B_r) + ' per read window (flop FIFO, plus the deep FIFO on its direction). ' +
        'Bands run from pessimistic to optimistic host timing.</p>';
    }
    return h + '</section>';
  }

  // ------------------------------------------------------------ demand of the focused protocol
  function demandCard(state, hd) {
    var wl = state.cell.wl, name = U.wlName(state.data, wl);
    var h = '<section class="tp-sec tp-demand" data-demand><h3>Demand of <span' + U.focusAttr(wl) + '>' + esc(name) + '</span></h3>';
    if (!(hd.bw_eff > 0)) return h + '<p class="muted">This protocol has no sustained host bandwidth need (payload ' + fmtNum(hd.payload_MBps, 3) + ' MB/s).</p></section>';
    var ratioPess = hd.link.pess / hd.bw_eff, ratioNom = hd.link.nom / hd.bw_eff, ratioOpt = hd.link.opt / hd.bw_eff;
    var state_ = ratioPess >= 1 ? ['ok', '✓', 'link keeps up (even pessimistic)'] : ratioOpt >= 1 ? ['mid', '~', 'link keeps up only on part of the band'] :
      ['bad', '✗', 'link is slower than the need'];
    h += '<div class="stats">' +
      U.stat('Payload', '<span class="num">' + fmtNum(hd.payload_MBps, 3) + ' MB/s</span>', 'direction ' + esc(hd.dir)) +
      U.stat('Streamed instructions', '<span class="num">' + fmtNum(hd.stream_MBps, 3) + ' MB/s</span>', hd.stream_MBps > 0 ? 'OUT EXEC, 2 * ipu * rate' : 'streaming not active') +
      U.stat('Total need', '<span class="num">' + fmtNum(hd.bw_eff, 3) + ' MB/s</span>') +
      U.stat('Link (' + esc(hd.dir === 'both' ? 'half duplex' : hd.dir) + ')', fmtBandHtml(hd.link, 'MB/s', 4), '', recRef(hd.dir === 'write' ? 'link_write_MBps' : hd.dir === 'read' ? 'link_read_MBps' : 'link_both_MBps', state.focus.col)) +
      '</div>';
    h += '<p class="tp-cmp ' + state_[0] + '" data-link-vs-need="' + state_[0] + '"><span class="ic" aria-hidden="true">' + state_[1] + '</span> ' + state_[2] +
      ': link / need = ' + fmtNum(ratioNom, 2) + 'x (' + fmtNum(ratioPess, 2) + '..' + fmtNum(ratioOpt, 2) + 'x). Buffer ' + fmtNum(hd.buf_B, 0) + ' B' +
      (hd.frame_B != null ? ', frame ' + fmtNum(hd.frame_B, 0) + ' B' : ', continuous stream') + (hd.rate_elastic ? ', rate elastic' : '') + '.</p>';
    h += streamCost(state, hd);
    return h + '</section>';
  }
  // OUT EXEC cost: what the knob adds for this protocol (a second hostDemand call with the knob flipped; the model owns the numbers).
  function streamCost(state, hd) {
    var arch = state.point.arch, on = arch.streaming === 'out_exec';
    var alt = A.hostDemand(Object.assign({}, arch, { streaming: on ? 'off' : 'out_exec' }), hd.wl, state.data);
    var withS = on ? hd : alt, without = on ? alt : hd;
    var h = '<p class="tp-stream" data-stream-cost>';
    if (!(withS.stream_MBps > 0)) {
      return h + 'Instruction streaming (OUT EXEC) is ' + (on ? 'on' : 'off') + ' and costs nothing for this protocol: its program fits the instruction memory.</p>';
    }
    h += 'OUT EXEC streaming ' + (on ? 'is on' : 'would add') + ': every instruction word also crosses the link, ' + fmtNum(withS.stream_MBps, 3) +
      ' MB/s on top of ' + fmtNum(without.bw_eff, 3) + ' MB/s of payload, ' + fmtNum(withS.bw_eff, 3) + ' MB/s in total against a ' + esc(withS.dir === 'both' ? 'half-duplex' : withS.dir) +
      ' link of ' + fmtNum(withS.link.nom, 3) + ' MB/s (' + fmtNum(withS.link.nom / withS.bw_eff, 2) + 'x).';
    return h + '</p>';
  }

  // ------------------------------------------------------------ FIFO fill trace (port of v1 stream diagram)
  function streamDiagram(state) {
    var st = A.streamTrace(state.point.arch, state.cell.wl, state.data, 60);
    var h = '<section class="tp-sec tp-trace"><h3>Host stream: FIFO fill' + (st.kind !== 'none' ? ' (' + esc(st.kind) + ')' : '') + '</h3>';
    if (st.kind === 'none') return h + '<p class="muted">' + esc(st.reason) + '</p></section>';
    var W = 760, L = 70, R = 20, T0 = 18, H = 220, plotH = H - T0 - 40, tmax = st.horizon_us, ymax = Math.max(st.buf_B, 1) * 1.08;
    var sx = function (t) { return L + (W - L - R) * t / tmax; }, sy = function (b) { return T0 + plotH * (1 - b / ymax); }, s = '';
    U.niceTicks(0, ymax, 5).forEach(function (b) {
      s += '<line class="gridline" x1="' + L + '" x2="' + (W - R) + '" y1="' + sy(b) + '" y2="' + sy(b) + '"/><text x="' + (L - 6) + '" y="' + (sy(b) + 4) + '" text-anchor="end">' + U.tickFmt(b) + '</text>';
    });
    U.niceTicks(0, tmax, 7).forEach(function (t) {
      s += '<text x="' + sx(t) + '" y="' + (T0 + plotH + 14) + '" text-anchor="middle">' + U.tickFmt(t) + '</text>';
    });
    s += '<text x="' + sx(tmax / 2) + '" y="' + (H - 6) + '" text-anchor="middle">time (us)</text><text x="12" y="' + (T0 + plotH / 2) + '" transform="rotate(-90 12 ' + (T0 + plotH / 2) + ')" text-anchor="middle">FIFO fill (B)</text>';
    var path = function (a) { return a.map(function (b, i) { return (i ? 'L' : 'M') + sx(st.t_us[i]).toFixed(1) + ' ' + sy(b).toFixed(1); }).join(' '); };
    var band = path(st.fill_B_pess) + ' ' + st.fill_B_opt.map(function (b, i, a) { var j = a.length - 1 - i; return 'L' + sx(st.t_us[j]).toFixed(1) + ' ' + sy(a[j]).toFixed(1); }).join(' ') + ' Z';
    s += '<path class="envelope" d="' + band + '"/>';
    s += '<line class="buf" x1="' + L + '" x2="' + (W - R) + '" y1="' + sy(st.buf_B) + '" y2="' + sy(st.buf_B) + '"/><text x="' + (W - R) + '" y="' + (sy(st.buf_B) - 4) + '" text-anchor="end">buffer ' + fmtNum(st.buf_B, 0) + ' B</text>';
    s += '<path class="fill" d="' + path(st.fill_B) + '"/>';
    if (st.frame_us != null) {
      var fx = sx(Math.min(st.frame_us, tmax));
      s += '<g class="deadline"><line x1="' + fx + '" x2="' + fx + '" y1="' + T0 + '" y2="' + (T0 + plotH) + '"/><text x="' + fx + '" y="' + (T0 - 4) + '" text-anchor="end">frame end ' + fmtNum(st.frame_us, 2) + ' us</text></g>';
    }
    if (st.event && st.event.kind && st.event.t_us <= tmax) {
      s += '<g class="event"><path d="M' + sx(st.event.t_us) + ' ' + (T0 + plotH - 2) + ' l-6 10 h12 z"/><text x="' + sx(st.event.t_us) + '" y="' + (T0 + plotH - 8) + '" text-anchor="middle">' + esc(st.event.kind) + ' ' + fmtNum(st.event.t_us, 2) + ' us</text></g>';
    }
    return h.replace('<section class="tp-sec tp-trace">', '<section class="tp-sec tp-trace" data-stream-sec data-stream-ok="' + (st.ok ? 1 : 0) + '">') +
      U.svgWrap(W, H, s, 'FIFO fill over time with pess/opt envelope') +
      '<p>Link ' + fmtBandHtml(st.link_MBps, 'MB/s', 2) + ' against need ' + fmtNum(st.bw_eff_MBps, 2) + ' MB/s; ' +
      (st.event && st.event.kind ? esc(st.event.kind) + ' after ' + fmtNum(st.event.bytes_streamed, 0) + ' B (' + fmtNum(st.event.t_us, 2) + ' us)' + (st.ok ? ', beyond the frame' : ', inside the frame') : 'the link keeps up, no overflow or underrun') +
      '. Shaded: pess..opt link rate.</p></section>';
  }

  U.renderThroughput = function (state) {
    var hd = A.hostDemand(state.point.arch, state.cell.wl, state.data);
    return '<div class="v-throughput">' + linkCard(state) + demandCard(state, hd) + streamDiagram(state) + '</div>';
  };
  U.register('throughput', { title: 'Throughput', render: function (s) { return U.renderThroughput(s); } });
})(ArchSimUI);
