"""Concurrency probe: run N cheap `--check` jobs in parallel and report failures."""
import concurrent.futures
import json
import subprocess
import sys
from pathlib import Path

PROJECT = Path(r"F:\airace\lite2_sim")
PY = PROJECT / ".venv-mujoco" / "Scripts" / "python.exe"


def check(index):
    command = [str(PY), "run_mujoco.py", "--scene", "ruins_reference", "--check", "--headless"]
    completed = subprocess.run(command, cwd=str(PROJECT), capture_output=True, text=True, timeout=600)
    directory = None
    for line in completed.stdout.splitlines():
        if line.startswith("Run output: "):
            directory = Path(line.split("Run output: ", 1)[1].strip())
    verification = None
    if directory and (directory / "verification.json").is_file():
        verification = json.loads((directory / "verification.json").read_text(encoding="utf-8"))
    return {"index": index, "exit": completed.returncode,
            "result": (verification or {}).get("result"),
            "error": (verification or {}).get("error"),
            "drift": (verification or {}).get("fixture_drift_m")}


if __name__ == "__main__":
    jobs = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        results = [future.result() for future in [pool.submit(check, i) for i in range(jobs)]]
    for result in results:
        print(json.dumps(result))
    print("CONCURRENCY_SUMMARY jobs=%d ok=%d" % (jobs, sum(1 for r in results if r["exit"] == 0)))