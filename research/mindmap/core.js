// 布局与文本生成核心（纯函数，可在 Node 中直接测试，不依赖 DOM）

const COL_X = [0, 400, 700, 970, 1200, 1400];
const LEAF_GAP = 34;

function colXOf(depth, cols) {
  if (depth < cols.length) return cols[depth];
  return cols[cols.length - 1] + (depth - cols.length + 1) * 250;
}

function hasKids(n) {
  return !!(n.children && n.children.length);
}

// 把树展开成扁平节点数组，同时按「可见性 + 叶子顺序」算出坐标
// cols: 可选的每层列 x 坐标（由前端按文字实测量宽后传入）
function layout(root, collapsed, papers, cols) {
  const collapsedSet = collapsed || new Set();
  const colsX = cols && cols.length ? cols : COL_X;
  const nodes = [];
  const links = [];
  let cursor = 0;

  function idOf(path) {
    return path.join('.');
  }

  function labelOf(node) {
    if (node.paper && papers && papers[node.paper]) return papers[node.paper].zh;
    return node.label || '';
  }

  function visit(node, depth, path, parentIdx) {
    const id = idOf(path);
    const kids = hasKids(node);
    const isCollapsed = collapsedSet.has(id);
    const idx = nodes.push({
      id,
      node,
      depth,
      label: labelOf(node),
      paperKey: node.paper || null,
      paper: node.paper && papers ? papers[node.paper] : null,
      tag: node.paper && papers && papers[node.paper] ? papers[node.paper].venueShort || shortVenue(papers[node.paper]) : '',
      note: node.note || '',
      hasChildren: kids,
      collapsed: isCollapsed,
      x: colXOf(depth, colsX),
      y: 0,
      parent: parentIdx,
      children: [],
      leafCount: 0,
    }) - 1;
    if (parentIdx >= 0) {
      nodes[parentIdx].children.push(idx);
      links.push({ from: parentIdx, to: idx });
    }

    if (kids && !isCollapsed) {
      const start = nodes.length;
      node.children.forEach((c, i) => visit(c, depth + 1, path.concat(i + 1), idx));
      const end = nodes.length;
      if (end === start) {
        nodes[idx].y = cursor;
        cursor += LEAF_GAP;
      } else {
        const firstY = nodes[start].y;
        const lastY = nodes[end - 1].y;
        nodes[idx].y = (firstY + lastY) / 2;
        nodes[idx].leafCount = end - start;
      }
    } else {
      nodes[idx].y = cursor;
      cursor += LEAF_GAP;
      nodes[idx].leafCount = 1;
    }
    return idx;
  }

  visit(root, 0, [0], -1);

  // 以根为中心纵向平移，视觉更平衡
  const ys = nodes.map((n) => n.y);
  const minY = Math.min.apply(null, ys);
  const maxY = Math.max.apply(null, ys);
  const shift = -(minY + maxY) / 2;
  nodes.forEach((n) => {
    n.y += shift;
  });

  const maxX = Math.max.apply(null, nodes.map((n) => n.x));
  const height = maxY - minY + 120;
  const width = maxX + 460;

  return { nodes, links, width, height, minY: -height / 2, maxY: height / 2 };
}

function shortVenue(p) {
  if (!p || !p.venue) return '';
  var m = String(p.venue).match(/^([^,]+),\s*(\d{4})/);
  if (m) return m[1] + ' ' + m[2];
  return p.venue;
}

// 找出所有含 text 的节点及其祖先（用于搜索时自动展开）
function searchMatches(nodes, q) {
  const hit = new Set();
  if (!q) return hit;
  const needle = q.toLowerCase();
  nodes.forEach((n, i) => {
    const p = n.paper;
    let hay = (n.label + ' ' + n.note).toLowerCase();
    if (p) hay += (' ' + p.title + ' ' + p.zh + ' ' + p.venue + ' ' + p.team + ' ' + p.mechanism).toLowerCase();
    if (hay.indexOf(needle) >= 0) {
      hit.add(i);
      let cur = n.parent;
      while (cur >= 0) {
        hit.add(cur);
        cur = nodes[cur].parent;
      }
    }
  });
  return hit;
}

// 从树生成 Markdown（可直接导入 XMind / markmap / 语雀）
function toMarkdown(root, papers) {
  const out = [];
  out.push('# ' + root.label);
  out.push('');
  out.push('> 10 篇顶刊（Science Robotics / IEEE T-RO / IJRR）2022–2026 · 方案整合思维导图');
  out.push('> 生成日期：2026-11-12 · 全部 DOI 经 Crossref 核实');
  out.push('');

  function walk(node, depth) {
    const pad = '  '.repeat(depth);
    if (node.paper && papers[node.paper]) {
      const p = papers[node.paper];
      out.push(pad + '- **' + p.zh + '** · ' + p.short + ' · ' + p.venue);
      out.push(pad + '  - 原题：' + p.title);
      out.push(pad + '  - DOI：' + p.doi + ' · ' + p.arxiv);
      out.push(pad + '  - 团队：' + p.team);
      out.push(pad + '  - 流派：' + p.paradigm);
      out.push(pad + '  - 机制：');
      p.mech.forEach((m) => out.push(pad + '    - ' + m));
      out.push(pad + '  - 创新：');
      p.innov.forEach((m) => out.push(pad + '    - ' + m));
      out.push(pad + '  - 局限：');
      p.limit.forEach((m) => out.push(pad + '    - ' + m));
      return;
    }
    out.push(pad + '- ' + (node.label || ''));
    (node.children || []).forEach((c) => walk(c, depth + 1));
  }

  (root.children || []).forEach((c) => walk(c, 0));
  out.push('');
  out.push('---');
  out.push('');
  out.push('## 附：10 篇顶刊速查表');
  out.push('');
  out.push('| # | 方案 | 论文（原题） | Venue | DOI | 流派 |');
  out.push('|---|---|---|---|---|---|');
  const order = ['miki', 'he2025', 'hoeller', 'lee2024', 'kim2025', 'kang2026', 'grandia', 'zhang2026', 'lin2026', 'bjelonic'];
  order.forEach((k, i) => {
    const p = papers[k];
    out.push('| ' + (i + 1) + ' | ' + p.zh + ' | ' + p.title + ' | ' + p.venue + ' | ' + p.doi + ' | ' + p.paradigm + ' |');
  });
  out.push('');
  return out.join('\n');
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { layout, toMarkdown, searchMatches, shortVenue, COL_X, LEAF_GAP };
}