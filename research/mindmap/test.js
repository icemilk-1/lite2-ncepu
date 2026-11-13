// 自测：布局 + Markdown 生成（不需要浏览器）
const { DATA, PAPERS } = require('./data.js');
const { layout, toMarkdown, searchMatches } = require('./core.js');

const res = layout(DATA, new Set(), PAPERS);
console.log('nodes:', res.nodes.length, 'links:', res.links.length);
console.log('width:', Math.round(res.width), 'height:', Math.round(res.height));

let bad = 0;
for (const n of res.nodes) {
  if (!Number.isFinite(n.x) || !Number.isFinite(n.y)) {
    bad++;
    console.log('BAD COORD', n.id, n.label);
  }
}
console.log('bad coords:', bad);

// 重叠检测：同一列内 y 间距是否足够
const byDepth = {};
res.nodes.forEach((n) => {
  (byDepth[n.depth] = byDepth[n.depth] || []).push(n);
});
Object.keys(byDepth).forEach((d) => {
  const arr = byDepth[d].sort((a, b) => a.y - b.y);
  let minGap = Infinity;
  for (let i = 1; i < arr.length; i++) minGap = Math.min(minGap, arr[i].y - arr[i - 1].y);
  console.log(`depth ${d}: ${arr.length} nodes, minGap=${minGap}`);
});

// 折叠测试
const collapsed = new Set([res.nodes.find((n) => n.depth === 1).id]);
const res2 = layout(DATA, collapsed, PAPERS);
console.log('collapsed nodes:', res2.nodes.length);

// 搜索测试
const hit = searchMatches(res.nodes, 'parkour');
console.log('search "parkour" hits:', hit.size);

// 论文计数
const paperNodes = res.nodes.filter((n) => n.paper);
console.log('paper leaves:', paperNodes.length, paperNodes.map((p) => p.paper.short).join(' / '));

const md = toMarkdown(DATA, PAPERS);
console.log('markdown chars:', md.length, 'lines:', md.split('\n').length);
require('fs').writeFileSync(__dirname + '/_preview.md', md);