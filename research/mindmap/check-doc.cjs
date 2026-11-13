// 校验 总体技术方案.md 的 Markdown 结构：标题序列 + 表格列数一致性
const fs = require('fs');
const p = 'F:/airace/总体技术方案.md';
const lines = fs.readFileSync(p, 'utf8').split(/\r?\n/);

// 1) 标题
const heads = [];
lines.forEach((l, i) => {
  const m = /^(#{1,4})\s+(.*)$/.exec(l);
  if (m) heads.push({ level: m[1].length, text: m[2].trim(), line: i + 1 });
});
console.log('=== 标题（1-3 级）===');
heads.filter((h) => h.level <= 3).forEach((h) => console.log(`${'  '.repeat(h.level - 1)}L${h.level} ${h.text}  @${h.line}`));

// 2) 表格列数
console.log('\n=== 表格一致性 ===');
const cells = (l) => l.replace(/^\|/, '').replace(/\|$/, '').split('|').length;
let block = [];
let problems = 0;
const flush = () => {
  if (block.length === 0) return;
  const counts = block.map((b) => cells(b.text));
  const uniq = [...new Set(counts)];
  const startLine = block[0].line;
  if (uniq.length > 1) {
    problems++;
    console.log(`  列数不一致 @${startLine}: ${counts.join(' / ')}`);
  }
  block = [];
};
lines.forEach((l, i) => {
  if (/^\s*\|.*\|\s*$/.test(l)) block.push({ text: l.trim(), line: i + 1 });
  else flush();
});
flush();
if (!problems) console.log('  所有表格列数一致 ✓');

// 3) 关键小节存在性
console.log('\n=== 关键小节检查 ===');
const must = ['### 3.1 导航层次划分与范式选型', '### 3.2 三项落地改造', '### 3.3 阶段安排与明确不做的部分'];
must.forEach((m) => console.log((heads.some((h) => h.text === m.slice(4)) ? 'PASS ' : 'FAIL ') + m));

// 4) 交叉引用目标存在性
console.log('\n=== 交叉引用检查 ===');
const refs = [
  ['第 3.1 节', () => heads.some((h) => h.text.startsWith('3.1 '))],
  ['第 3.2、3.3 节', () => heads.some((h) => h.text.startsWith('3.2 ')) && heads.some((h) => h.text.startsWith('3.3 '))],
  ['第 4.1 节', () => heads.some((h) => h.text.startsWith('4.1 '))],
  ['第 4.3 节', () => heads.some((h) => h.text.startsWith('4.3 '))],
  ['第 2.1 节', () => heads.some((h) => h.text.startsWith('2.1 '))],
  ['第 9.1 节', () => heads.some((h) => h.text.startsWith('9.1 '))],
];
refs.forEach(([name, fn]) => console.log((fn() ? 'PASS ' : 'FAIL ') + name + ' 有对应小节'));

// 5) 链接目标文件存在
console.log('\n=== 链接文件存在性 ===');
const links = [...fs.readFileSync(p, 'utf8').matchAll(/\]\((F:[^)]+)\)/g)].map((m) => m[1]);
const uniq = [...new Set(links)];
uniq.filter((u) => /四足机器狗|research\//.test(u)).forEach((u) => {
  const f = u.replace(/\//g, '\\');
  console.log((fs.existsSync(f) ? 'PASS ' : 'FAIL ') + u);
});

console.log('\n总行数:', lines.length);