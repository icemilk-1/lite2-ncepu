"""Validate Mermaid flowcharts inside Markdown files.

Checks (all render-breaking in recent mermaid):
  1. fence non-empty, first line `flowchart TD|TB|LR|RL|BT`
  2. subgraph/end balance
  3. subgraph titles quoted
  4. every node label wrapped in double quotes
  5. no `;` `#` `%` `|` inside a node label
  6. edge labels `-->|...|` quoted
  7. no style / classDef / class / click / linkStyle
  8. no `[1]`-style citation inside labels
  9. odd number of double quotes on a line

Usage:  python validate_mermaid.py <file.md> [...]
Writes full report to <workspace>/tmp/mermaid_report.txt (utf-8),
prints ASCII-only summary to stdout. Exit 1 when errors found.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

FENCE_RE = re.compile(r"```mermaid[ \t]*\n(.*?)```", re.S)
SUBGRAPH_RE = re.compile(r"^subgraph\b\s*(.*)$")
END_RE = re.compile(r"^end$")
FORBIDDEN_KW = re.compile(r"^(style|classDef|class|click|linkStyle)\b")
BAD_IN_LABEL = [";", "#", "%", "|"]
EDGE_LABEL_RE = re.compile(r"\|([^|\n]*)\|")
TWO_CH = {"([": "])", "[(": ")]", "((": "))", "[[": "]]", "{{": "}}"}
ONE_CH = {"[": "]", "(": ")", "{": "}"}


def extract_labels(line: str):
    """Return list of (position, raw label text) for node-label constructs."""
    out = []
    n = len(line)
    i = 0
    while i < n:
        two = line[i : i + 2]
        if two in TWO_CH:
            close = TWO_CH[two]
            j = line.find(close, i + 2)
            if j != -1:
                out.append((i, line[i + 2 : j]))
                i = j + 2
                continue
            i += 1
            continue
        ch = line[i]
        if ch in ONE_CH:
            close = ONE_CH[ch]
            j = line.find(close, i + 1)
            if j != -1:
                out.append((i, line[i + 1 : j]))
                i = j + 1
                continue
        i += 1
    return out


def id_before(line: str, pos: int) -> str | None:
    m = re.search(r"([A-Za-z_][\w-]*)\s*$", line[:pos])
    return m.group(1) if m else None


def check_fence(body: str, start_line: int) -> tuple[list[str], int]:
    errs: list[str] = []
    lines = body.splitlines()
    if not lines:
        return [f"L{start_line}: empty mermaid block"], 0

    head = lines[0].strip()
    if not re.match(r"^(flowchart|graph)\s+(TD|TB|LR|RL|BT)\b", head):
        errs.append(f"L{start_line}: first line must be 'flowchart TD|LR|...', got {head!r}")

    depth = 0
    node_ids: set[str] = set()

    for offset, raw in enumerate(lines[1:], start=1):
        ln = start_line + offset
        line = raw.strip()
        if not line or line.startswith("%%"):
            continue
        if FORBIDDEN_KW.match(line):
            errs.append(f"L{ln}: forbidden mermaid keyword {line.split()[0]!r}")
            continue

        m = SUBGRAPH_RE.match(line)
        if m:
            depth += 1
            title = m.group(1).strip()
            if title:
                ok = False
                inner = ""
                if title.startswith('"') and title.endswith('"'):
                    ok = True
                    inner = title[1:-1]
                else:
                    bm = re.match(r"^(?P<id>[A-Za-z_][\w-]*)\s*\[(?P<inner>.*)\]$", title)
                    if bm:
                        inner = bm.group("inner").strip()
                        ok = inner.startswith('"') and inner.endswith('"')
                    elif title.startswith("[") and title.endswith("]"):
                        inner = title[1:-1].strip()
                        ok = inner.startswith('"') and inner.endswith('"')
                if ok and '"' in inner:
                    pass
                if not ok:
                    errs.append(f'L{ln}: subgraph title must be quoted: subgraph S1["name"] -> {line}')
            continue

        if END_RE.match(line):
            depth -= 1
            if depth < 0:
                errs.append(f"L{ln}: stray 'end' with no open subgraph")
                depth = 0
            continue

        if line.count('"') % 2 != 0:
            errs.append(f"L{ln}: odd number of double quotes")

        # strip edge labels before scanning node labels
        scan = EDGE_LABEL_RE.sub("|EL|", line)
        for pos, content in extract_labels(scan):
            nid = id_before(scan, pos)
            if nid:
                node_ids.add(nid)
            s = content.strip()
            if not s:
                continue
            if s == "EL":
                continue
            if not (s.startswith('"') and s.endswith('"')):
                errs.append(f"L{ln}: unquoted node label {s!r}")
                continue
            inner = s[1:-1]
            for ch in BAD_IN_LABEL:
                if ch in inner:
                    errs.append(f"L{ln}: char {ch!r} inside label {inner!r}")
            if re.search(r"\[\d+\]", inner):
                errs.append(f"L{ln}: citation-like text inside label {inner!r}")

        for em in EDGE_LABEL_RE.finditer(line):
            lab = em.group(1).strip()
            if lab and not (lab.startswith('"') and lab.endswith('"')):
                errs.append(f"L{ln}: edge label must be quoted -> |{lab}|")

    if depth != 0:
        errs.append(f"L{start_line}: unbalanced subgraph/end (depth={depth})")
    if len(node_ids) < 3:
        errs.append(f"L{start_line}: only {len(node_ids)} node id(s) found")
    return errs, len(node_ids)


def main(argv: list[str]) -> int:
    if not argv:
        print("usage: python validate_mermaid.py <file.md> [...]")
        return 2

    report: list[str] = []
    total_d = total_e = 0
    for name in argv:
        path = Path(name)
        if not path.exists():
            report.append(f"MISSING {path}")
            total_e += 1
            print(f"MISSING {path}")
            continue
        text = path.read_text(encoding="utf-8")
        fences = list(FENCE_RE.finditer(text))
        report.append(f"\n=== {path.name} :: {len(fences)} block(s) ===")
        for fm in fences:
            total_d += 1
            start_line = text[: fm.start()].count("\n") + 1
            errs, nnodes = check_fence(fm.group(1), start_line)
            if errs:
                total_e += len(errs)
                report.append(f"  [FAIL] block@{start_line} nodes={nnodes}")
                report.extend(f"      - {e}" for e in errs)
            else:
                report.append(f"  [ok]   block@{start_line} nodes={nnodes}")

    report.append(f"\n--- {total_d} diagram(s), {total_e} error(s) ---")
    out = Path(r"F:\airace\tmp\mermaid_report.txt")
    out.write_text("\n".join(report), encoding="utf-8")
    print(f"diagrams={total_d} errors={total_e} report={out}")
    return 1 if total_e else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))