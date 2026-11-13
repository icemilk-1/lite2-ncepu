// 无浏览器 DOM 冒烟测试：用最小 DOM 垫片跑一遍 app.js，验证渲染逻辑不抛错、输出结构正确
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function approxWidth(text, size) {
  let w = 0;
  for (const ch of String(text)) {
    const c = ch.codePointAt(0);
    if (c > 0x2e80) w += size * 1.0; // CJK 全角
    else if (ch === ' ') w += size * 0.3;
    else w += size * 0.58;
  }
  return w;
}

class ClassList {
  constructor() { this.set = new Set(); }
  add(...c) { c.forEach((x) => this.set.add(x)); }
  remove(...c) { c.forEach((x) => this.set.delete(x)); }
  toggle(c, force) {
    const on = force === undefined ? !this.set.has(c) : !!force;
    if (on) this.set.add(c); else this.set.delete(c);
  }
  contains(c) { return this.set.has(c); }
  get value() { return [...this.set].join(' '); }
}

class El {
  constructor(tag) {
    this.tagName = String(tag).toUpperCase();
    this.attrs = {};
    this._html = '';
    this.textContent = '';
    this.classList = new ClassList();
    this.style = {};
    this.children = [];
    this.listeners = 0;
  }
  setAttribute(k, v) { this.attrs[k] = String(v); }
  getAttribute(k) { return this.attrs[k] === undefined ? null : this.attrs[k]; }
  addEventListener() { this.listeners++; }
  set innerHTML(v) { this._html = String(v); }
  get innerHTML() { return this._html; }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  appendChild(c) { this.children.push(c); return c; }
  removeChild(c) { return c; }
  insertBefore(c) { this.children.unshift(c); return c; }
  cloneNode() { const e = new El(this.tagName); e.attrs = { ...this.attrs }; e._html = this._html; return e; }
  getBoundingClientRect() { return { left: 0, top: 0, width: 1600, height: 1000 }; }
  get clientWidth() { return 1600; }
  get clientHeight() { return 1000; }
}

const registry = {};
const IDS = ['canvas', 'svg', 'scene', 'panel', 'tableview', 'toast', 'q', 'btn-map', 'btn-table',
  'btn-expand', 'btn-l1', 'btn-fit', 'btn-width', 'btn-svg', 'btn-png'];
IDS.forEach((id) => { registry[id] = new El(id === 'q' ? 'input' : 'div'); });

const documentShim = {
  getElementById: (id) => registry[id] || (registry[id] = new El('div')),
  createElement: (tag) => {
    if (tag === 'canvas') {
      return {
        getContext: () => ({
          font: '',
          measureText: (t) => ({ width: approxWidth(t, parseFloat((String(this && this.font) || '').replace(/\D+/g, '')) || 13) }),
        }),
      };
    }
    return new El(tag);
  },
  createElementNS: (ns, tag) => new El(tag),
  addEventListener() {},
  body: new El('body'),
};

// 让 canvas 的 measureText 用正确的字号
const origCreate = documentShim.createElement;
documentShim.createElement = function (tag) {
  if (tag === 'canvas') {
    const ctx = {
      font: '13px sans',
      measureText(t) {
        const m = /(\d+(?:\.\d+)?)px/.exec(this.font);
        const size = m ? parseFloat(m[1]) : 13;
        return { width: approxWidth(t, size) };
      },
    };
    return { getContext: () => ctx };
  }
  return new El(tag);
};

const windowShim = {
  addEventListener() {},
  innerWidth: 1600,
  innerHeight: 1000,
  clearTimeout,
  setTimeout,
  URL: { createObjectURL: () => 'blob:x', revokeObjectURL() {} },
  Image: class {},
  XMLSerializer: class { serializeToString() { return '<svg/>'; } },
  DOMParser: class {},
};

const sandbox = {
  document: documentShim,
  window: windowShim,
  setTimeout,
  clearTimeout,
  console,
  Math,
  Number,
  String,
  Array,
  Set,
  Map,
  Object,
  JSON,
  parseFloat,
  parseInt,
  isNaN,
  Blob: class {},
  URL: windowShim.URL,
  Image: windowShim.Image,
  XMLSerializer: windowShim.XMLSerializer,
  navigator: { userAgent: 'node-shim' },
};

// window 属性也要能当全局用（app.js 里直接写了 window.xxx，另有一些全局函数）
sandbox.globalThis = sandbox;

const dir = __dirname;
const src =
  fs.readFileSync(path.join(dir, 'data.js'), 'utf8').replace(/if \(typeof module[\s\S]*?module\.exports = \{ DATA, PAPERS \};\s*\}/, '') +
  '\n' +
  fs.readFileSync(path.join(dir, 'core.js'), 'utf8').replace(/if \(typeof module[\s\S]*?module\.exports = \{[\s\S]*?\};\s*\}/, '') +
  '\n' +
  fs.readFileSync(path.join(dir, 'app.js'), 'utf8');

let error = null;
try {
  vm.createContext(sandbox);
  vm.runInContext(src, sandbox, { filename: 'mindmap.bundle.js' });
} catch (e) {
  error = e;
}

if (error) {
  console.log('FAIL: 运行期异常');
  console.log(error && error.stack ? error.stack.split('\n').slice(0, 8).join('\n') : error);
  process.exit(1);
}

const scene = registry.scene;
const html = scene.innerHTML;
const nodeCount = (html.match(/class="node /g) || []).length + (html.match(/class="node root"/g) || []).length;
const linkCount = (html.match(/<path class="link/g) || []).length;
const labelCount = (html.match(/class="label"/g) || []).length;
const paperTagCount = (html.match(/class="tag"/g) || []).length;

console.log('scene svg bytes   :', html.length);
console.log('nodes rendered    :', nodeCount);
console.log('links rendered    :', linkCount);
console.log('labels rendered   :', labelCount);
console.log('paper tags        :', paperTagCount);
console.log('transform         :', scene.getAttribute('transform'));
console.log('table html bytes  :', registry.tableview.innerHTML.length);
console.log('table rows        :', (registry.tableview.innerHTML.match(/<tr data-k=/g) || []).length);

const checks = [
  ['节点数 = 130', nodeCount === 130],
  ['连线数 = 129', linkCount === 129],
  ['论文标签 = 10', paperTagCount === 10],
  ['根节点有 pill', html.includes('class="pillbg"')],
  ['transform 已设置', /translate\(.+\) scale\(.+\)/.test(scene.getAttribute('transform') || '')],
  ['表格 10 行', (registry.tableview.innerHTML.match(/<tr data-k=/g) || []).length === 10],
  ['表格含 DOI', registry.tableview.innerHTML.includes('10.1126/scirobotics.abk2822')],
  ['无 NaN 坐标', !/NaN/.test(html)],
  ['无 undefined 文本', !html.includes('>undefined<')],
];

// 几何检查：标签热区矩形两两不重叠（防止文字压字）
const boxes = [];
const re = /<rect class="hit" x="(-?[\d.]+)" y="(-?[\d.]+)" width="([\d.]+)" height="22"/g;
let m;
while ((m = re.exec(html))) {
  const x = parseFloat(m[1]);
  const y = parseFloat(m[2]);
  const w = parseFloat(m[3]);
  boxes.push({ x, y, w, h: 22 });
}
let overlaps = 0;
for (let i = 0; i < boxes.length; i++) {
  for (let j = i + 1; j < boxes.length; j++) {
    const a = boxes[i], b = boxes[j];
    const ox = Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x);
    const oy = Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y);
    if (ox > 1 && oy > 1) overlaps++;
  }
}
console.log('label boxes       :', boxes.length, '| overlapped pairs:', overlaps);
checks.push(['标签热区无重叠', overlaps === 0]);

// 连线起点必须落在父节点标签热区之外（不能被文字压住）
const scenes = html.split('<g class="links">')[1] || '';
const linkStarts = [...scenes.matchAll(/d="M([-\d.]+),([-\d.]+) C/g)].map((x) => ({ x: +x[1], y: +x[2] }));
let inside = 0;
linkStarts.forEach((p) => {
  const hit = boxes.some((b) => p.x > b.x + 1 && p.x < b.x + b.w - 1 && p.y > b.y + 1 && p.y < b.y + b.h - 1);
  if (hit) inside++;
});
console.log('link starts checked:', linkStarts.length, '| inside a label box:', inside);
checks.push(['连线起点不压文字', inside === 0]);

let ok = true;
checks.forEach(([name, pass]) => {
  console.log((pass ? 'PASS ' : 'FAIL ') + name);
  if (!pass) ok = false;
});
process.exit(ok ? 0 : 1);