"""Build a standalone HTML viewer for the literature-review flowcharts.

Design note: mermaid definitions are handed to mermaid.render() as JS strings
(JSON-embedded), NOT via DOM text, so HTML entity escaping can never corrupt a
label. If the CDN is unreachable the raw source stays visible as a fallback.
"""
from __future__ import annotations

import html
import json
import re
import sys
from pathlib import Path

import markdown

SRC = Path(r"F:\airace\plan\四足机器人导航避障文献逐篇流程图讲解.md")
DST = Path(r"F:\airace\plan\四足机器人导航避障文献逐篇流程图讲解.html")

MERMAID_BLOCK = re.compile(
    r'<pre><code class="language-mermaid">(.*?)</code></pre>', re.S
)


def slugify(value: str, name: str) -> str:
    s = re.sub(r"\s+", "-", value.strip())
    s = re.sub(r"[^\w\u4e00-\u9fff\-.]", "", s)
    return s or "sec"


def build() -> int:
    text = SRC.read_text(encoding="utf-8")
    n_diagrams = text.count("```mermaid")

    md = markdown.Markdown(
        extensions=["tables", "fenced_code", "toc", "sane_lists", "attr_list"],
        extension_configs={
            "toc": {"slugify": slugify, "permalink": False, "toc_depth": "2-3"}
        },
    )
    body = md.convert(text)
    toc_html = md.toc

    defs: list[dict[str, str]] = []
    counter = {"n": 0}

    def repl(m: re.Match) -> str:
        counter["n"] += 1
        did = f"d{counter['n']}"
        code = html.unescape(m.group(1)).strip()
        defs.append({"id": did, "code": code})
        return (
            f'<figure class="diagram" id="{did}">'
            f'<div class="render" data-for="{did}"></div>'
            f'<details class="srcbox"><summary>查看该图 Mermaid 源码</summary>'
            f'<pre class="src"><code>{html.escape(code)}</code></pre></details>'
            f"</figure>"
        )

    body, n_sub = MERMAID_BLOCK.subn(repl, body)
    defs_json = json.dumps(defs, ensure_ascii=False).replace("</", "<\\/")

    page = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>四足机器狗导航避障 · 逐篇论文流程图讲解</title>
<style>
  :root {{
    --bg:#f6f8fa; --fg:#1b2027; --muted:#5b6470; --line:#dde2e8;
    --card:#ffffff; --accent:#1f6feb;
  }}
  * {{ box-sizing:border-box; }}
  body {{ margin:0; background:var(--bg); color:var(--fg);
    font:16px/1.78 -apple-system,"Segoe UI","Microsoft YaHei","PingFang SC",sans-serif; }}
  #wrap {{ display:flex; align-items:flex-start; max-width:1560px; margin:0 auto; }}
  nav {{ position:sticky; top:0; flex:0 0 330px; max-height:100vh; overflow-y:auto;
    padding:18px 14px 60px; background:var(--card); border-right:1px solid var(--line);
    font-size:13px; line-height:1.55; }}
  nav .toc-title {{ font-size:13px; letter-spacing:.08em; color:var(--muted); margin:0 0 10px; }}
  nav ul {{ list-style:none; margin:0; padding-left:12px; }}
  nav > ul {{ padding-left:0; }}
  nav li {{ margin:2px 0; }}
  nav a {{ color:#39424c; text-decoration:none; display:block; padding:2px 6px; border-radius:4px; }}
  nav a:hover {{ background:#eef2f7; color:var(--accent); }}
  main {{ flex:1 1 auto; min-width:0; padding:34px 48px 140px; background:var(--card); }}
  h1 {{ font-size:27px; line-height:1.35; margin:0 0 4px; }}
  h2 {{ font-size:21px; margin:46px 0 14px; padding-bottom:8px; border-bottom:2px solid var(--line); }}
  h3 {{ font-size:17px; margin:30px 0 10px; color:#12202e; }}
  p {{ margin:10px 0; }}
  hr {{ border:none; border-top:1px solid var(--line); margin:32px 0; }}
  blockquote {{ margin:14px 0; padding:12px 16px; background:#f2f6fc;
    border-left:4px solid var(--accent); border-radius:0 6px 6px 0; color:#2a3441; }}
  blockquote p {{ margin:4px 0; }}
  code {{ background:#eef1f5; padding:.12em .38em; border-radius:4px;
    font:13.5px/1.5 "Cascadia Code",Consolas,monospace; }}
  pre.src {{ background:#1d2430; color:#e6edf3; padding:12px 14px; border-radius:8px;
    overflow-x:auto; margin:8px 0 0; }}
  pre.src code {{ background:none; color:inherit; padding:0; font-size:13px; }}
  table {{ border-collapse:collapse; width:100%; margin:16px 0; font-size:14.5px;
    display:block; overflow-x:auto; }}
  th,td {{ border:1px solid var(--line); padding:8px 11px; text-align:left; vertical-align:top; }}
  th {{ background:#f1f4f8; font-weight:600; }}
  tbody tr:nth-child(even) {{ background:#fafbfd; }}
  figure.diagram {{ margin:18px 0 22px; padding:14px 12px 10px; background:#fcfdff;
    border:1px solid var(--line); border-radius:12px; }}
  figure.diagram .render {{ overflow-x:auto; text-align:center; min-height:40px; }}
  figure.diagram .render svg {{ max-width:100%; height:auto; }}
  figure.diagram .render.err {{ text-align:left; color:#b42318; font-size:14px; }}
  details.srcbox {{ margin-top:8px; font-size:13px; color:var(--muted); }}
  details.srcbox summary {{ cursor:pointer; user-select:none; padding:2px 0; }}
  #banner {{ display:none; margin:0 0 18px; padding:10px 14px; border-radius:8px;
    background:#fff8e1; color:#8a6100; font-size:14px; }}
  .meta {{ color:var(--muted); font-size:14px; margin:0 0 16px; }}
  @media print {{ nav,nav+main {{ display:block; }} nav {{ display:none; }}
    main {{ padding:0; }} body {{ background:#fff; }} figure.diagram {{ break-inside:avoid; }} }}
  @media (max-width:980px) {{ nav {{ display:none; }} main {{ padding:20px 16px; }} }}
</style>
</head>
<body>
<div id="wrap">
  <nav><p class="toc-title">目录</p>{toc_html}</nav>
  <main>
    <div id="banner"></div>
    <p class="meta">共 {n_diagrams} 张流程图 · 54 篇文献 · 2026-11-12</p>
    {body}
  </main>
</div>
<script type="module">
const DEFS = {defs_json};
const banner = document.getElementById('banner');
let mermaid = null;
try {{
  const mod = await import('https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs');
  mermaid = mod.default;
  mermaid.initialize({{
    startOnLoad: false,
    theme: 'base',
    securityLevel: 'loose',
    fontFamily: '-apple-system,"Segoe UI","Microsoft YaHei",sans-serif',
    themeVariables: {{
      fontSize: '14px',
      primaryColor: '#eef4ff',
      primaryBorderColor: '#4b7bd6',
      primaryTextColor: '#16202c',
      lineColor: '#5b6470',
      clusterBkg: '#f7f9fc',
      clusterBorder: '#c8d2de'
    }},
    flowchart: {{ htmlLabels: true, curve: 'basis', nodeSpacing: 36, rankSpacing: 44 }}
  }});
}} catch (e) {{
  banner.style.display = 'block';
  banner.textContent = '无法从 CDN 加载 mermaid（可能处于离网环境）。每张图下方仍可展开查看 Mermaid 源码；也可用 Typora / VS Code / Obsidian 打开同名 .md 文件查看渲染结果。';
}}

if (mermaid) {{
  let ok = 0, fail = 0;
  for (const d of DEFS) {{
    const box = document.querySelector('.render[data-for="' + d.id + '"]');
    if (!box) continue;
    try {{
      const {{ svg }} = await mermaid.render('svg-' + d.id, d.code);
      box.innerHTML = svg;
      ok++;
    }} catch (err) {{
      box.classList.add('err');
      box.textContent = '该图渲染失败（' + (err && err.message ? err.message : err) + '），请展开下方源码查看。';
      fail++;
    }}
  }}
  const badge = document.createElement('p');
  badge.className = 'meta';
  badge.textContent = '已渲染 ' + ok + ' 张流程图' + (fail ? '，失败 ' + fail + ' 张' : '') + '。';
  document.querySelector('main').prepend(badge);
}}
</script>
</body>
</html>
"""
    DST.write_text(page, encoding="utf-8")
    print(f"html written: {DST} ({len(page)} chars, {n_sub}/{n_diagrams} blocks, {len(defs)} defs)")
    return 0 if n_sub == n_diagrams else 1


if __name__ == "__main__":
    sys.exit(build())