"""Per-stage isolation evidence: run the remaining stages from their own start pose."""
import concurrent.futures
import json
import subprocess
import sys
from pathlib import Path

PROJECT = Path(r"F:\airace\lite2_sim")
PY = PROJECT / ".venv-mujoco" / "Scripts" / "python.exe"
STAGES = ["dead_end_exit", "south_rubble_route", "rubble_return", "round_pipe",
          "narrow_passage_rejection", "square_culvert", "finish"]


def run_stage(stage_id, attempt=1):
    command = [str(PY), "run_mujoco.py", "--scene", "ruins_reference", "--course",
               "--course-stage", stage_id, "--headless", "--seed", "42"]
    completed = subprocess.run(command, cwd=str(PROJECT), capture_output=True, text=True, timeout=330)
    directory = None
    for line in completed.stdout.splitlines():
        if line.startswith("Run output: "):
            directory = Path(line.split("Run output: ", 1)[1].strip())
    verification = summary = None
    if directory:
        for name in ("course_verification.json", "navigation_summary.json"):
            path = directory / name
            if path.is_file():
                data = json.loads(path.read_text(encoding="utf-8"))
                if name.startswith("course"):
                    verification = data
                else:
                    summary = data
    error = None
    if directory and (directory / "error.log").is_file():
        text = (directory / "error.log").read_text(encoding="utf-8")
        error = text.strip().splitlines()[-1] if text.strip() else None
    record = {
        "stage": stage_id, "seed": 42, "attempt": attempt,
        "passed": bool((verification or {}).get("passed")),
        "result": (summary or {}).get("result"),
        "stages": [(entry.get("id"), entry.get("passed")) for entry in ((verification or {}).get("stages") or [])],
        "sim_s": (summary or {}).get("elapsed_simulation_s"),
        "directory": str(directory) if directory else None,
        "recoveries": (verification or {}).get("recovery_count"),
        "error": error,
    }
    if not record["passed"] and error and "Fixture drift" in error and attempt < 2:
        return run_stage(stage_id, attempt + 1)
    return record


if __name__ == "__main__":
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = [future.result() for future in [pool.submit(run_stage, stage_id) for stage_id in STAGES]]
    for result in results:
        print(json.dumps(result))
    print("STAGE_SUMMARY passed=%d/%d" % (sum(r["passed"] for r in results), len(results)))