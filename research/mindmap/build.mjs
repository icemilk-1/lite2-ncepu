// 构建：把 data/core/app/style 打包成单文件 HTML，并顺带生成 Markdown 大纲
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '..', '..');

const read = (p) => fs.readFileSync(path.join(dir, p), 'utf8');

const { DATA, PAPERS } = require('./data.js');
const { toMarkdown } = require('./core.js');

const css = read('style.css');
const dataJs = read('data.js')
  .replace(/if \(typeof module[\s\S]*?module\.exports = \{ DATA, PAPERS \};\s*\}/, '')
  .trim();
const coreJs = read('core.js')
  .replace(/if \(typeof module[\s\S]*?module\.exports = \{[\s\S]*?\};\s*\}/, '')
  .trim();
const appJs = read('app.js');

const paperCount = Object.keys(PAPERS).length;
const nodeCount = (() => {
  let n = 0;
  const walk = (x) => { n++; (x.children || []).forEach(walk); };
  walk(DATA);
  return n;
})();

const html = `<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>四足机器狗导航 · 10 篇顶刊方案思维导图</title>
<style>
${css}
</style>
</head>
<body>
<div class="app">
  <header>
    <div class="brand">
      <h1>四足机器狗导航 · 10 篇顶刊方案整合</h1>
      <div class="sub">
        <span><b>Science Robotics ×6</b> · <b>IEEE T-RO ×3</b> · <b>IJRR ×1</b></span>
        <span>2022 – 2026</span>
        <span>${nodeCount} 个节点 / ${paperCount} 篇论文</span>
        <span>DOI、卷期、年份均经 Crossref 核实</span>
      </div>
    </div>
    <div class="toolbar">
      <div class="search">
        <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><circle cx="11" cy="11" r="7"/><path d="M20 20l-4.3-4.3"/></svg>
        <input id="q" type="search" placeholder="搜索论文 / 方案 / 关键词" autocomplete="off" spellcheck="false">
      </div>
      <button class="tb on" id="btn-map">思维导图</button>
      <button class="tb" id="btn-table">速查表</button>
      <button class="tb" id="btn-expand">展开全部</button>
      <button class="tb" id="btn-l1">仅一级</button>
      <button class="tb" id="btn-width">适配宽度</button>
      <button class="tb" id="btn-fit">全览</button>
      <button class="tb" id="btn-svg">导出 SVG</button>
      <button class="tb" id="btn-png">导出 PNG</button>
    </div>
  </header>
  <main>
    <div id="canvas">
      <svg id="svg" xmlns="http://www.w3.org/2000/svg">
        <defs>
          <linearGradient id="rootGrad" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stop-color="#123449"/>
            <stop offset="100%" stop-color="#0b1a27"/>
          </linearGradient>
        </defs>
        <g id="scene"></g>
      </svg>
    </div>
    <div id="tableview"></div>
    <aside id="panel"></aside>
    <div class="hint">
      <span>拖拽平移</span><span>滚轮缩放</span><span>点击节点展开 / 收起</span><span>点击论文看详情</span><span><kbd>0</kbd> 全览</span><span><kbd>1</kbd> 适配宽度</span>
    </div>
  </main>
</div>
<div id="toast"></div>
<script>
${dataJs}

${coreJs}

${appJs}
</script>
</body>
</html>
`;

fs.writeFileSync(path.join(root, '四足机器狗导航_顶刊方案思维导图.html'), html, 'utf8');

const md = toMarkdown(DATA, PAPERS)
  .replace('# 四足机器狗导航 · 10 篇顶刊方案整合', '# 四足机器狗导航 · 10 篇顶刊方案整合（思维导图大纲）')
  .replace(
    '> 10 篇顶刊',
    '> 用途：可直接粘贴进 XMind / markmap / 语雀 / Obsidian 生成思维导图。\n> 10 篇顶刊'
  );
fs.writeFileSync(path.join(root, '四足机器狗导航_顶刊方案思维导图.md'), md, 'utf8');

console.log('HTML bytes:', Buffer.byteLength(html, 'utf8'));
console.log('MD bytes:', Buffer.byteLength(md, 'utf8'));
console.log('nodes:', nodeCount, 'papers:', paperCount);