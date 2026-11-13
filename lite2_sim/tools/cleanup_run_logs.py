"""Prune bulky inputs from completed runs, preserving summaries and recent runs."""

import argparse
import json
from pathlib import Path


def cleanup(runs, keep=10, apply=False):
    if keep < 1:
        raise ValueError("keep must be at least 1")
    root = Path(runs).resolve(strict=True)
    completed = []
    for directory in root.iterdir():
        if directory.is_symlink() or not directory.is_dir():
            continue
        summary = directory / "navigation_summary.json"
        if summary.is_file():
            completed.append((summary.stat().st_mtime_ns, directory))
    completed.sort(reverse=True)
    count, size, pruned = 0, 0, 0
    for _, directory in completed[keep:]:
        files = [path for pattern in ("sensor_observations.jsonl*", "navigation_inputs.jsonl*", "lidar_*.npz")
                 for path in directory.glob(pattern)
                 if path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root)]
        if not files:
            continue
        if apply:
            marker = {"reason": "bulk_input_retention", "full_replay_available": False,
                      "removed_files": [path.name for path in files]}
            (directory / "log_cleanup.json").write_text(json.dumps(marker, indent=2), encoding="utf-8")
        for path in files:
            length = path.stat().st_size
            if apply:
                path.unlink()
            size += length
            count += 1
        pruned += 1
    return {"applied": apply, "completed_runs": len(completed), "keep": keep,
            "pruned_runs": pruned, "files": count, "bytes": size}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, default=Path(__file__).resolve().parents[1] / "runs")
    parser.add_argument("--keep", type=int, default=10)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    print(json.dumps(cleanup(args.runs, args.keep, args.apply)))
