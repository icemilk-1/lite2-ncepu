"""Cross-check citation tokens in the shard files against the source review.

Any arXiv id / DOI that appears in a shard but NOT in the review document is a
potential fabrication -> must be verified or removed.

Prints ASCII summary; full report to tmp/citation_report.txt (utf-8).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REVIEW = Path(r"F:\airace\plan\四足机器人导航避障文献综述与选型建议.md")
SHARDS = Path(r"F:\airace\tmp\flownotes")

ARXIV_RE = re.compile(r"arXiv:\s*(\d{4}\.\d{4,5})", re.I)
DOI_RE = re.compile(r"\b10\.\d{4,6}/[A-Za-z0-9._;()/:\-]+")


def norm_doi(d: str) -> str:
    return d.rstrip(".,;)]`").lower()


def norm_arxiv(a: str) -> str:
    return a.lower()


def main() -> int:
    review = REVIEW.read_text(encoding="utf-8")
    rev_arxiv = {norm_arxiv(m) for m in ARXIV_RE.findall(review)}
    rev_doi = {norm_doi(m) for m in DOI_RE.findall(review)}

    report = [f"review: {len(rev_arxiv)} arxiv ids, {len(rev_doi)} dois"]

    bad = 0
    for shard in sorted(SHARDS.glob("*.md")):
        text = shard.read_text(encoding="utf-8")
        s_arxiv = {norm_arxiv(m) for m in ARXIV_RE.findall(text)}
        s_doi = {norm_doi(m) for m in DOI_RE.findall(text)}
        miss_a = sorted(s_arxiv - rev_arxiv)
        miss_d = sorted(s_doi - rev_doi)
        report.append(
            f"\n=== {shard.name}: arxiv={len(s_arxiv)} doi={len(s_doi)} "
            f"| new_arxiv={len(miss_a)} new_doi={len(miss_d)}"
        )
        for a in miss_a:
            report.append(f"   NEW ARXIV  {a}")
            bad += 1
        for d in miss_d:
            report.append(f"   NEW DOI    {d}")
            bad += 1

    report.append(f"\n--- {bad} token(s) not present in the review ---")
    out = Path(r"F:\airace\tmp\citation_report.txt")
    out.write_text("\n".join(report), encoding="utf-8")
    print(f"new_tokens={bad} report={out}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())