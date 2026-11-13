/* 交互式思维导图渲染（浏览器端） */
(function () {
  'use strict';

  var FONT_STACK = '"Inter","Segoe UI",system-ui,-apple-system,"Noto Sans SC","Microsoft YaHei",sans-serif';
  var PALETTE = ['#4cc9f0', '#f72585', '#ffd166', '#7bf1a8', '#b388ff', '#ff9e6d', '#5ad0c4', '#9fb3c8'];

  var canvasEl = document.getElementById('canvas');
  var svg = document.getElementById('svg');
  var scene = document.getElementById('scene');
  var panel = document.getElementById('panel');
  var tableView = document.getElementById('tableview');
  var toast = document.getElementById('toast');

  var mctx = document.createElement('canvas').getContext('2d');
  function measure(text, size, weight) {
    mctx.font = (weight || 400) + ' ' + size + 'px ' + FONT_STACK;
    return mctx.measureText(text).width;
  }

  var state = {
    collapsed: new Set(),
    k: 1,
    tx: 40,
    ty: 0,
    sel: null,
    query: '',
    view: 'map',
    lay: null,
  };

  function styleOf(depth) {
    if (depth === 0) return { size: 19, weight: 700 };
    if (depth === 1) return { size: 15, weight: 640 };
    if (depth === 2) return { size: 13, weight: 500 };
    return { size: 12.5, weight: 400 };
  }

  function truncateToWidth(text, maxW, size, weight) {
    if (measure(text, size, weight) <= maxW) return text;
    var lo = 1, hi = text.length;
    while (lo < hi) {
      var mid = Math.ceil((lo + hi) / 2);
      if (measure(text.slice(0, mid) + '…', size, weight) <= maxW) lo = mid;
      else hi = mid - 1;
    }
    return text.slice(0, Math.max(1, lo)) + '…';
  }

  // 依据文字实宽计算每层列坐标
  function computeCols(nodes) {
    var maxByDepth = {};
    var maxDepth = 0;
    nodes.forEach(function (n) {
      var s = styleOf(n.depth);
      var w = measure(n.label, s.size, s.weight);
      if (n.paper) w += measure(n.tag, 10.5, 400) + 12;
      w += 26; // dot + 间距
      maxByDepth[n.depth] = Math.max(maxByDepth[n.depth] || 0, w);
      maxDepth = Math.max(maxDepth, n.depth);
    });
    var cols = [];
    var x = 0;
    for (var d = 0; d <= maxDepth; d++) {
      cols.push(x);
      var w = Math.min(maxByDepth[d] || 180, 620);
      x += Math.max(w, 160) + 34;
    }
    return { cols: cols, total: x, maxDepth: maxDepth };
  }

  function build() {
    var full = layout(DATA, new Set(), PAPERS); // 用于测量全部文字（含折叠部分）
    var measured = layout(DATA, state.collapsed, PAPERS);
    var info = computeCols(full.nodes);
    var lay = layout(DATA, state.collapsed, PAPERS, info.cols);
    state.lay = lay;

    // 每条主线颜色 + 祖先链
    var nodes = lay.nodes;
    nodes.forEach(function (n, i) {
      var cur = n;
      var guard = 0;
      while (cur && cur.depth > 1 && guard++ < 10) cur = nodes[cur.parent];
      n.branch = (cur && cur.depth === 1) ? nodes.indexOf(cur) % PALETTE.length : 0;
      n.chain = [];
      var p = n.parent, g = 0;
      while (p >= 0 && g++ < 12) {
        n.chain.push(p);
        p = nodes[p].parent;
      }
    });

    var out = [];
    // 预测量文字宽度（连线要从父节点文字末尾出发，避免压住父标签）
    nodes.forEach(function (n) {
      var st = styleOf(n.depth);
      n.labelW = measure(n.label, st.size, st.weight);
      n.tagW = n.paper ? measure(n.tag, 10.5, 400) + 12 : 0;
    });

    // 连线
    out.push('<g class="links">');
    lay.links.forEach(function (l) {
      var a = nodes[l.from], b = nodes[l.to];
      var x1 = a.depth === 0 ? a.labelW + 46 : a.x + a.labelW + a.tagW + 32;
      var y1 = a.y;
      var x2 = b.x - 9;
      var y2 = b.y;
      var dx = Math.max(26, (x2 - x1) * 0.42);
      var color = PALETTE[b.depth === 1 ? nodes.indexOf(b) % PALETTE.length : b.branch];
      out.push('<path class="link d' + Math.min(3, b.depth) + '" data-a="' + l.from + '" data-b="' + l.to +
        '" d="M' + r(x1) + ',' + r(y1) + ' C' + r(x1 + dx) + ',' + r(y1) + ' ' + r(x2 - dx) + ',' + r(y2) + ' ' + r(x2) + ',' + r(y2) +
        '" stroke="' + color + '"/>');
    });
    out.push('</g>');

    // 节点
    out.push('<g class="nodes">');
    nodes.forEach(function (n, i) {
      var s = styleOf(n.depth);
      var color = PALETTE[n.depth === 1 ? i % PALETTE.length : n.branch];
      var cls = 'node d' + Math.min(3, n.depth) + (n.depth === 0 ? ' root' : '');
      if (n.paper) cls += ' paper';
      var label = n.label;
      var labelW;
      if (n.depth === 0) {
        labelW = measure(label, 19, 700);
        out.push('<g class="' + cls + '" data-i="' + i + '">');
        out.push('<rect class="pillbg" x="0" y="-21" width="' + r(labelW + 40) + '" height="42" rx="21"/>');
        out.push('<circle class="dot" cx="20" cy="0" r="5" fill="' + color + '"/>');
        out.push('<text class="label" x="34" y="1" dominant-baseline="central" fill="#eaf6ff">' + esc(label) + '</text>');
        out.push('</g>');
        return;
      }
      labelW = measure(label, s.size, s.weight);
      var tagW = n.paper ? measure(n.tag, 10.5, 400) + 12 : 0;
      out.push('<g class="' + cls + '" data-i="' + i + '">');
      out.push('<rect class="hit" x="' + r(n.x - 20) + '" y="' + r(n.y - 11) + '" width="' + r(labelW + tagW + 44) + '" height="22" rx="6"/>');
      out.push('<circle class="dot" cx="' + r(n.x) + '" cy="' + r(n.y) + '" r="' + (n.hasChildren ? 5 : 4) + '" fill="' +
        (n.hasChildren ? '#0b1521' : color) + '" stroke="' + color + '" stroke-width="' + (n.hasChildren ? 2.2 : 1.4) + '"/>');
      out.push('<text class="label" x="' + r(n.x + 15) + '" y="' + r(n.y + 1) + '" dominant-baseline="central" fill="' +
        (n.depth === 1 ? '#e6f1fb' : '#c2d3e4') + '" font-weight="' + s.weight + '" font-size="' + s.size + '">' + esc(label) + '</text>');
      if (n.paper) {
        out.push('<text class="tag" x="' + r(n.x + 15 + labelW + 10) + '" y="' + r(n.y + 1) + '" dominant-baseline="central">' + esc(n.tag) + '</text>');
      }
      if (n.hasChildren) {
        out.push('<circle class="plus" cx="' + r(n.x - 13) + '" cy="' + r(n.y) + '" r="7"/>');
        out.push('<text class="plusTxt" x="' + r(n.x - 13) + '" y="' + r(n.y + 0.5) + '" dominant-baseline="central">' + (n.collapsed ? '+' : '−') + '</text>');
      }
      out.push('</g>');
    });
    out.push('</g>');

    scene.innerHTML = out.join('');
    applyTransform();
    if (state.query) applyQuery();
    applySelection();
    return lay;
  }

  function r(v) { return Math.round(v * 100) / 100; }
  function esc(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function applyTransform() {
    scene.setAttribute('transform', 'translate(' + r(state.tx) + ',' + r(state.ty) + ') scale(' + r(state.k) + ')');
  }

  function viewSize() {
    return { w: canvasEl.clientWidth, h: canvasEl.clientHeight };
  }

  function layW() {
    var lay = state.lay;
    var max = 0;
    lay.nodes.forEach(function (n) {
      var s = styleOf(n.depth);
      var w = n.x + measure(n.label, s.size, s.weight) + 40;
      if (n.paper) w += measure(n.tag, 10.5, 400) + 14;
      max = Math.max(max, w);
    });
    return max;
  }

  function fitWidth() {
    var v = viewSize();
    var w = layW();
    var k = Math.max(0.3, Math.min(1.05, (v.w - 90) / w));
    state.k = k;
    state.tx = 46;
    state.ty = v.h / 2;
    applyTransform();
  }

  function fitAll() {
    var v = viewSize();
    var lay = state.lay;
    var w = layW();
    var h = lay.height;
    var k = Math.min((v.w - 70) / w, (v.h - 70) / h, 1.1);
    state.k = Math.max(0.16, k);
    state.tx = (v.w - w * state.k) / 2;
    state.ty = v.h / 2;
    applyTransform();
  }

  // ---------- 交互 ----------
  var drag = { on: false, x: 0, y: 0, moved: 0, pid: null };

  canvasEl.addEventListener('pointerdown', function (e) {
    if (e.target.closest('#panel')) return;
    drag.on = true;
    drag.x = e.clientX;
    drag.y = e.clientY;
    drag.moved = 0;
    drag.pid = e.pointerId;
    canvasEl.classList.add('dragging');
  });

  window.addEventListener('pointermove', function (e) {
    if (!drag.on) return;
    var dx = e.clientX - drag.x;
    var dy = e.clientY - drag.y;
    drag.moved += Math.abs(dx) + Math.abs(dy);
    state.tx += dx;
    state.ty += dy;
    drag.x = e.clientX;
    drag.y = e.clientY;
    applyTransform();
  });

  window.addEventListener('pointerup', function () {
    drag.on = false;
    canvasEl.classList.remove('dragging');
  });

  canvasEl.addEventListener('wheel', function (e) {
    if (e.target.closest('#panel')) return;
    e.preventDefault();
    var rect = canvasEl.getBoundingClientRect();
    var px = e.clientX - rect.left;
    var py = e.clientY - rect.top;
    var factor = Math.pow(1.0016, -e.deltaY);
    var k2 = Math.max(0.15, Math.min(3.2, state.k * factor));
    state.tx = px - (px - state.tx) * (k2 / state.k);
    state.ty = py - (py - state.ty) * (k2 / state.k);
    state.k = k2;
    applyTransform();
  }, { passive: false });

  scene.addEventListener('click', function (e) {
    if (drag.moved > 6) return;
    var g = e.target.closest('.node');
    if (!g) return;
    var i = Number(g.getAttribute('data-i'));
    var n = state.lay.nodes[i];
    if (!n) return;
    if (n.paper) {
      select(i);
    } else if (n.hasChildren) {
      if (state.collapsed.has(n.id)) state.collapsed.delete(n.id);
      else state.collapsed.add(n.id);
      build(); // 保留当前缩放/平移，避免每次展开都跳视野
    } else {
      select(i);
    }
  });

  function select(i) {
    state.sel = i;
    applySelection();
    renderPanel(state.lay.nodes[i]);
    panel.classList.add('open');
  }

  function applySelection() {
    var sel = scene.querySelectorAll('.node.sel');
    for (var i = 0; i < sel.length; i++) sel[i].classList.remove('sel');
    if (state.sel == null || !state.lay) return;
    var g = scene.querySelector('.node[data-i="' + state.sel + '"]');
    if (g) g.classList.add('sel');
  }

  function applyQuery() {
    var q = state.query.trim();
    var all = scene.querySelectorAll('.node');
    var paths = scene.querySelectorAll('.link');
    if (!q) {
      for (var i = 0; i < all.length; i++) all[i].classList.remove('dim', 'hit-match');
      for (var j = 0; j < paths.length; j++) paths[j].classList.remove('dim', 'hl');
      return;
    }
    var hits = new Set();
    state.lay.nodes.forEach(function (n, idx) {
      var p = n.paper;
      var hay = (n.label + ' ' + (n.note || '')).toLowerCase();
      if (p) hay += ' ' + (p.title + ' ' + p.zh + ' ' + p.venue + ' ' + p.team + ' ' + p.paradigm + ' ' + p.mech.join(' ')).toLowerCase();
      if (hay.indexOf(q.toLowerCase()) >= 0) hits.add(idx);
    });
    state.lay.nodes.forEach(function (n, idx) {
      var g = scene.querySelector('.node[data-i="' + idx + '"]');
      if (!g) return;
      g.classList.toggle('dim', !hits.has(idx));
      g.classList.toggle('hit-match', hits.has(idx));
    });
    for (var k = 0; k < paths.length; k++) {
      var a = Number(paths[k].getAttribute('data-a'));
      var b = Number(paths[k].getAttribute('data-b'));
      var hot = hits.has(a) && hits.has(b);
      paths[k].classList.toggle('hl', hot);
      paths[k].classList.toggle('dim', !hot);
    }
  }

  // ---------- 右侧面板 ----------
  function renderPanel(n) {
    if (!n) return;
    var html = [];
    if (n.paper) {
      var p = n.paper;
      html.push('<div class="p-top"><div><div class="p-venue">' + esc(p.venue) + '</div>');
      html.push('<h2>' + esc(p.zh) + '</h2>');
      html.push('<div class="en">' + esc(p.title) + '</div></div>');
      html.push('<button class="p-close" onclick="document.getElementById(\'panel\').classList.remove(\'open\')">✕</button></div>');
      html.push('<div class="meta">');
      html.push('<div class="k">团队</div><div class="v">' + esc(p.team) + '</div>');
      html.push('<div class="k">流派</div><div class="v">' + esc(p.paradigm) + '</div>');
      html.push('<div class="k">DOI</div><div class="v"><a href="https://doi.org/' + esc(p.doi) + '" target="_blank" rel="noopener">' + esc(p.doi) + '</a></div>');
      if (p.arxiv) html.push('<div class="k">预印本</div><div class="v">' + esc(p.arxiv) + '</div>');
      html.push('</div>');
      html.push('<div class="p-sec"><h3>方案机制</h3><ul>' + p.mech.map(function (m) { return '<li>' + esc(m) + '</li>'; }).join('') + '</ul></div>');
      html.push('<div class="p-sec innov"><h3>关键创新</h3><ul>' + p.innov.map(function (m) { return '<li>' + esc(m) + '</li>'; }).join('') + '</ul></div>');
      html.push('<div class="p-sec limit"><h3>局限 / 空白</h3><ul>' + p.limit.map(function (m) { return '<li>' + esc(m) + '</li>'; }).join('') + '</ul></div>');
    } else {
      html.push('<div class="p-top"><div><div class="p-venue">思维导图节点</div>');
      html.push('<h2>' + esc(n.label) + '</h2></div>');
      html.push('<button class="p-close" onclick="document.getElementById(\'panel\').classList.remove(\'open\')">✕</button></div>');
      if (n.note) html.push('<p style="font-size:12.5px;line-height:1.6;color:#b9cbdd">' + esc(n.note) + '</p>');
      if (n.hasChildren) {
        var kids = state.lay.nodes.filter(function (x) { return x.parent === state.lay.nodes.indexOf(n); });
        html.push('<div class="p-sec"><h3>下级节点（' + kids.length + '）</h3><ul>' +
          kids.map(function (k) { return '<li>' + esc(k.label) + '</li>'; }).join('') + '</ul></div>');
      }
      html.push('<p style="font-size:12px;color:#6f869c;line-height:1.6">点击圆点旁节点可展开/收起该分支。</p>');
    }
    panel.innerHTML = html.join('');
  }

  // ---------- 表格视图 ----------
  function renderTable() {
    var order = ['miki', 'he2025', 'hoeller', 'lee2024', 'kim2025', 'kang2026', 'grandia', 'zhang2026', 'lin2026', 'bjelonic'];
    var rows = order.map(function (k, i) {
      var p = PAPERS[k];
      return '<tr data-k="' + k + '"><td class="idx">' + (i + 1) + '</td>' +
        '<td class="zh">' + esc(p.zh) + '<div class="pv">' + esc(p.paradigm) + '</div></td>' +
        '<td class="en">' + esc(p.title) + '</td>' +
        '<td>' + esc(p.venue) + '</td>' +
        '<td class="doi">' + esc(p.doi) + '</td></tr>';
    }).join('');
    tableView.innerHTML = '<div class="tv-head"><h2>10 篇顶刊速查表</h2><span>点击任意一行回到导图并定位</span></div>' +
      '<table><thead><tr><th>#</th><th>方案</th><th>英文原题</th><th>Venue</th><th>DOI</th></tr></thead><tbody>' + rows + '</tbody></table>';
    tableView.querySelectorAll('tbody tr').forEach(function (tr) {
      tr.addEventListener('click', function () {
        var k = tr.getAttribute('data-k');
        switchView('map');
        var idx = -1;
        state.lay.nodes.forEach(function (n, i) { if (n.paperKey === k) idx = i; });
        if (idx >= 0) {
          // 确保祖先展开
          var chain = state.lay.nodes[idx].chain;
          chain.forEach(function (pi) { state.collapsed.delete(state.lay.nodes[pi].id); });
          build();
          fitWidth();
          var idx2 = -1;
          state.lay.nodes.forEach(function (n, i) { if (n.paperKey === k) idx2 = i; });
          select(idx2);
        }
      });
    });
  }

  // ---------- 视图切换 ----------
  function switchView(v) {
    state.view = v;
    var btnMap = document.getElementById('btn-map');
    var btnTable = document.getElementById('btn-table');
    if (v === 'map') {
      tableView.classList.remove('show');
      btnMap.classList.add('on');
      btnTable.classList.remove('on');
    } else {
      tableView.classList.add('show');
      btnTable.classList.add('on');
      btnMap.classList.remove('on');
      panel.classList.remove('open');
    }
  }

  // ---------- 导出 ----------
  function buildExportSVG() {
    var lay = state.lay;
    var w = layW() + 80;
    var h = lay.height + 80;
    var clone = svg.cloneNode(true);
    clone.setAttribute('width', w);
    clone.setAttribute('height', h);
    clone.setAttribute('viewBox', '-40 ' + (-h / 2) + ' ' + w + ' ' + h);
    var s = clone.querySelector('#scene');
    s.setAttribute('transform', 'translate(0,0) scale(1)');
    clone.querySelectorAll('.dim').forEach(function (el) { el.classList.remove('dim'); });
    clone.querySelectorAll('.hit-match,.sel').forEach(function (el) { el.classList.remove('hit-match', 'sel'); });
    var bg = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    bg.setAttribute('x', '-40');
    bg.setAttribute('y', -h / 2);
    bg.setAttribute('width', w);
    bg.setAttribute('height', h);
    bg.setAttribute('fill', '#0a1119');
    clone.insertBefore(bg, clone.firstChild);
    var str = new XMLSerializer().serializeToString(clone);
    if (str.indexOf('xmlns=') < 0) str = str.replace('<svg', '<svg xmlns="http://www.w3.org/2000/svg"');
    return { str: str, w: w, h: h };
  }

  function download(name, href) {
    var a = document.createElement('a');
    a.href = href;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  }

  function showToast(msg) {
    toast.textContent = msg;
    toast.classList.add('show');
    clearTimeout(showToast._t);
    showToast._t = setTimeout(function () { toast.classList.remove('show'); }, 1900);
  }

  document.getElementById('btn-svg').addEventListener('click', function () {
    var out = buildExportSVG();
    var blob = new Blob([out.str], { type: 'image/svg+xml;charset=utf-8' });
    var url = URL.createObjectURL(blob);
    download('四足机器狗导航_顶刊方案思维导图.svg', url);
    setTimeout(function () { URL.revokeObjectURL(url); }, 4000);
    showToast('已导出 SVG（矢量，可拖进 PPT / Word）');
  });

  document.getElementById('btn-png').addEventListener('click', function () {
    var out = buildExportSVG();
    var blob = new Blob([out.str], { type: 'image/svg+xml;charset=utf-8' });
    var url = URL.createObjectURL(blob);
    var img = new Image();
    img.onload = function () {
      var scale = Math.min(2, 8000 / Math.max(out.w, out.h));
      var cv = document.createElement('canvas');
      cv.width = Math.round(out.w * scale);
      cv.height = Math.round(out.h * scale);
      var ctx = cv.getContext('2d');
      ctx.fillStyle = '#0a1119';
      ctx.fillRect(0, 0, cv.width, cv.height);
      ctx.drawImage(img, 0, 0, cv.width, cv.height);
      cv.toBlob(function (b) {
        var u = URL.createObjectURL(b);
        download('四足机器狗导航_顶刊方案思维导图.png', u);
        setTimeout(function () { URL.revokeObjectURL(u); }, 4000);
        showToast('已导出 PNG（长图，适合贴文档）');
      }, 'image/png');
      URL.revokeObjectURL(url);
    };
    img.onerror = function () { showToast('PNG 导出失败，请改用 SVG'); };
    img.src = url;
  });

  // ---------- 顶部按钮 ----------
  document.getElementById('btn-expand').addEventListener('click', function () {
    state.collapsed.clear();
    build();
    fitWidth();
  });
  document.getElementById('btn-l1').addEventListener('click', function () {
    state.collapsed.clear();
    state.lay.nodes.forEach(function (n) { if (n.depth === 1) state.collapsed.add(n.id); });
    build();
    fitWidth();
  });
  document.getElementById('btn-fit').addEventListener('click', fitAll);
  document.getElementById('btn-width').addEventListener('click', fitWidth);
  document.getElementById('btn-map').addEventListener('click', function () { switchView('map'); });
  document.getElementById('btn-table').addEventListener('click', function () { switchView('table'); });

  var searchInput = document.getElementById('q');
  var searchTimer = null;
  searchInput.addEventListener('input', function () {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(function () {
      state.query = searchInput.value;
      applyQuery();
    }, 120);
  });

  document.addEventListener('keydown', function (e) {
    if (e.target && e.target.tagName === 'INPUT') {
      if (e.key === 'Escape') { searchInput.value = ''; state.query = ''; applyQuery(); searchInput.blur(); }
      return;
    }
    if (e.key === 'Escape') panel.classList.remove('open');
    else if (e.key === '0') fitAll();
    else if (e.key === '1') fitWidth();
    else if (e.key === '+' || e.key === '=') { state.k = Math.min(3.2, state.k * 1.15); applyTransform(); }
    else if (e.key === '-') { state.k = Math.max(0.15, state.k / 1.15); applyTransform(); }
  });

  window.addEventListener('resize', function () { fitWidth(); });

  // ---------- 启动 ----------
  build();
  renderTable();
  fitWidth();
  // 桌面端首屏略微放大
  if (window.innerWidth > 1500) {
    state.k = Math.min(1.05, state.k * 1.08);
    applyTransform();
  }
})();