// Real-parser validation of every mermaid block in the master document.
// Uses mermaid's own parser with a jsdom shim, so this catches anything the
// regex validator cannot (bad node ids, illegal unicode, broken arrows, ...).
import fs from 'node:fs';
import path from 'node:path';
import { JSDOM } from 'jsdom';

const PLAN_DIR = 'F:/airace/plan';
const CANDIDATES = fs.readdirSync(PLAN_DIR).filter((f) => f.endsWith('.md'));
const SRC = path.join(
  PLAN_DIR,
  CANDIDATES.find((f) => f.includes('\u6d41\u7a0b\u56fe')) || CANDIDATES[0]
);

const md = fs.readFileSync(SRC, 'utf8').replace(/\r\n/g, '\n');

const dom = new JSDOM('<!DOCTYPE html><html><body></body></html>', {
  pretendToBeVisual: true,
  url: 'http://localhost/',
});
globalThis.window = dom.window;
globalThis.document = dom.window.document;
try {
  Object.defineProperty(globalThis, 'navigator', {
    value: dom.window.navigator,
    configurable: true,
  });
} catch {
  /* node >=21 exposes a read-only navigator; fine to skip */
}
globalThis.DOMParser = dom.window.DOMParser;
globalThis.Element = dom.window.Element;
globalThis.HTMLElement = dom.window.HTMLElement;
globalThis.SVGElement = dom.window.SVGElement;
globalThis.Node = dom.window.Node;
globalThis.getComputedStyle = dom.window.getComputedStyle.bind(dom.window);
globalThis.requestAnimationFrame = (cb) => setTimeout(() => cb(Date.now()), 0);

const mermaid = (await import('mermaid')).default;
mermaid.initialize({
  startOnLoad: false,
  securityLevel: 'loose',
  flowchart: { htmlLabels: true },
});

const blocks = [...md.matchAll(/```mermaid[ \t]*\n([\s\S]*?)```/g)].map((m, i) => {
  const line = md.slice(0, m.index).split('\n').length;
  return { i: i + 1, line, code: m[1].replace(/\s+$/, '') };
});

let ok = 0;
const fails = [];
for (const b of blocks) {
  try {
    await mermaid.parse(b.code, { suppressErrors: false });
    ok++;
  } catch (e) {
    const raw = e && (e.str || e.message) ? e.str || e.message : String(e);
    const msg = raw.split('\n').slice(0, 4).join(' | ');
    fails.push({ ...b, msg });
  }
}

const lines = [`file=${SRC}`, `blocks=${blocks.length} ok=${ok} fail=${fails.length}`];
for (const f of fails) {
  lines.push(`\n[FAIL] block#${f.i} @L${f.line}: ${f.msg}`);
  lines.push(f.code.split('\n').slice(0, 6).join('\n'));
}
fs.writeFileSync('F:/airace/tmp/mermaid_parse_report.txt', lines.join('\n'), 'utf8');
console.log(`file=${path.basename(SRC)} blocks=${blocks.length} ok=${ok} fail=${fails.length}`);
process.exit(fails.length ? 1 : 0);