"""Diagnostic: run each course stage in isolation (--course-stage) and summarize."""
import concurrent.futures
import json
import subprocess
import sys
from pathlib import Path

PROJECT = Path(r"F:\airace\lite2_sim")
PY = PROJECT / ".venv-mujoco" / "Scripts" / "python.exe"
config = json.loads((PROJECT / "config" / "ruins_reference.json").read_text(encoding="utf-8"))
STAGES = [stage["id"] for stage in config["course"]["stages"]]


def run_stage(stage_id, seed):
    command = [str(PY), "run_mujoco.py", "--scene", "ruins_reference", "--course",
               "--course-stage", stage_id, "--headless", "--seed", str(seed)]
    completed = subprocess.run(command, cwd=str(PROJECT), capture_output=True, text=True, timeout=400)
    directory = None
    for line in completed.stdout.splitlines():
        if line.startswith("Run output: "):
            directory = Path(line.split("Run output: ", 1)[1].strip())
    verification, summary = None, None
    if directory:
        for name, slot in (("course_verification.json", "course"), ("navigation_summary.json", "summary")):
            path = directory / name
            if path.is_file():
                data = json.loads(path.read_text(encoding="utf-8"))
                if slot == "course":
                    verification = data
                else:
                    summary = data
    return {
        "stage": stage_id, "seed": seed,
        "passed": bool((verification or {}).get("passed")),
        "result": (summary or {}).get("result"),
        "stage_pass": [(entry.get("id"), entry.get("passed")) for entry in ((verification or {}).get("stages") or [])],
        "sim_s": (summary or {}).get("elapsed_simulation_s"),
        "directory": str(directory) if directory else None,
        "exit": completed.returncode,
    }


if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 42
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
        futures = [pool.submit(run_stage, stage_id, seed) for stage_id in STAGES]
        results = [future.result() for future in futures]
    for result in results:
        print(json.dumps(result))
    print("STAGE_SUMMARY passed=%d/%d seed=%d" % (sum(r["passed"] for r in results), len(results), seed))