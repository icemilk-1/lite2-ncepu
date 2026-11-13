"""Regenerate ``mujoco/navigation_acceptance.json`` from live runs.

Every fact in the acceptance record is produced by actually running the stack:
online-mapping navigation runs on both scenes, the sensor-loss / cancellation /
timeout / dynamic-obstacle scenarios, recorded-input replay equivalence, the
regression suite, and (optionally) full-course and randomized Monte Carlo
evidence produced by their own tools.

Usage::

    python tools/build_navigation_acceptance.py
    python tools/build_navigation_acceptance.py --runs ruins_reference:42 --scenarios none
    python tools/build_navigation_acceptance.py --course-run runs/mujoco_... --monte-carlo-report runs/monte_carlo_.../report.json
    python tools/build_navigation_acceptance.py --video

The tool never fabricates a result: a missing run, scenario, replay or test
suite is recorded as ``null``/``false`` and flips the overall ``passed`` flag.
"""

import argparse
import concurrent.futures
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys
import time


PROJECT = Path(__file__).resolve().parents[1]
RUNS_DIR = PROJECT / "runs"
DEFAULT_RUNS = ("ruins_reference:42", "ruins_reference:43", "ruins_reference:44", "sensor_lab:42")
SCENARIOS = ("cancel", "timeout", "lidar_loss", "localization_loss", "dynamic_obstacle")
EXPECTED_SCENARIO_RESULTS = {"cancel": "cancelled", "timeout": "timeout", "lidar_loss": "timeout",
                             "localization_loss": "localization_lost",
                             "dynamic_obstacle": "goal_reached"}
NAVIGATION_TIMEOUT_S = 1800.0
SCENARIO_TIMEOUT_S = 900.0


def python_executable():
    candidate = PROJECT / ".venv-mujoco" / "Scripts" / "python.exe"
    return candidate if candidate.is_file() else Path(sys.executable)


def _run(command, timeout):
    started = time.time()
    try:
        completed = subprocess.run(command, cwd=str(PROJECT), capture_output=True, text=True,
                                   timeout=timeout)
        return completed, time.time() - started, None
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout.decode("utf-8", "replace") if isinstance(error.stdout, bytes) else error.stdout
        stderr = error.stderr.decode("utf-8", "replace") if isinstance(error.stderr, bytes) else error.stderr
        return (subprocess.CompletedProcess(command, 124, stdout or "", stderr or ""),
                time.time() - started, "timeout after %.0f s" % timeout)


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _last_json_object(text):
    for line in reversed((text or "").splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return None


def _navigation_run(scene, seed, deadline_s):
    command = [str(python_executable()), "run_mujoco.py", "--scene", scene, "--navigate",
               "--headless", "--seed", str(seed)]
    completed, duration, failure = _run(command, deadline_s)
    run_directory = None
    for line in (completed.stdout or "").splitlines():
        if line.startswith("Run output: "):
            run_directory = Path(line.split("Run output: ", 1)[1].strip())
    summary = _read_json(run_directory / "navigation_summary.json") if run_directory else None
    verification = _read_json(run_directory / "navigation_verification.json") if run_directory else None
    checks = (summary or {}).get("checks") or {}
    return {
        "directory": str(run_directory) if run_directory else None,
        "relative_directory": _relative(run_directory) if run_directory else None,
        "scene": scene,
        "seed": seed,
        "passed": bool((summary or {}).get("passed") and (verification or {}).get("passed")),
        "result": (summary or {}).get("result") or (verification or {}).get("result"),
        "goal_error_m": ((summary or {}).get("evaluation_only") or {}).get("goal_error_m"),
        "collision_stop_events": len((_read_json(run_directory / "collision_events.json") or [])
                                     if run_directory else []),
        "collision_free": checks.get("collision_free"),
        "stopped_for_one_second": checks.get("stopped_for_one_second"),
        "simulation_duration_s": (summary or {}).get("elapsed_simulation_s"),
        "wall_duration_s": round(duration, 2),
        "source_hashes": _read_json(run_directory / "source_hashes.json") if run_directory else None,
        "error": failure or (None if completed.returncode == 0 else
                             "exit code %d: %s" % (completed.returncode,
                                                   ((completed.stderr or "").strip().splitlines() or [""])[-1])),
    }


def _scenario_run(name, deadline_s):
    command = [str(python_executable()), "tools/check_navigation_scenarios.py", "--scenario", name]
    completed, duration, failure = _run(command, deadline_s)
    verification = _last_json_object(completed.stdout)
    expected = EXPECTED_SCENARIO_RESULTS[name]
    actual = (verification or {}).get("actual_result")
    return {
        "scenario": name,
        "expected_result": expected,
        "actual_result": actual,
        "passed": bool(actual == expected and not failure),
        "events": (verification or {}).get("events", []),
        "output": (verification or {}).get("output"),
        "wall_duration_s": round(duration, 2),
        "error": failure or (None if actual else "no scenario verification printed"),
    }


def _replay_run(run_directory, deadline_s):
    command = [str(python_executable()), "tools/replay_navigation.py", str(run_directory)]
    completed, _, failure = _run(command, deadline_s)
    result = _last_json_object(completed.stdout)
    if not result:
        return {"passed": False, "error": failure or "replay produced no verification record"}
    report = _read_json(Path(run_directory) / "replay_verification.json")
    return {
        "passed": bool(result.get("passed")),
        "frames": result.get("frames"),
        "mismatched_frames": result.get("mismatched_frames"),
        "result": result.get("result"),
        "control_output": "recorded_values_only",
        "report": _relative(Path(run_directory) / "replay_verification.json") if report else None,
        "error": failure,
    }


def _video_run(run_directory, deadline_s):
    command = [str(python_executable()), "tools/render_navigation.py", str(run_directory)]
    completed, _, failure = _run(command, deadline_s)
    video = Path(run_directory) / "navigation_replay.mp4"
    return {"path": _relative(video) if video.is_file() else None,
            "rendered": video.is_file(), "error": failure}


def _tests_run(deadline_s):
    command = [str(python_executable()), "tools/run_tests.py"]
    completed, duration, failure = _run(command, deadline_s)
    output = (completed.stdout or "") + (completed.stderr or "")
    match = re.search(r"Ran (\d+) tests", output)
    passed = "OK" in output and completed.returncode == 0 and not failure
    return {"count": int(match.group(1)) if match else None, "passed": passed,
            "wall_duration_s": round(duration, 2), "error": failure,
            "summary_line": next((line.strip() for line in reversed(output.splitlines())
                                  if line.strip().startswith(("OK", "FAILED"))), None)}


def _course_evidence(directories):
    evidence = []
    for directory in directories or ():
        path = Path(directory)
        verification = _read_json(path / "course_verification.json")
        summary = _read_json(path / "navigation_summary.json")
        evidence.append({
            "directory": str(path.resolve()),
            "relative_directory": _relative(path),
            "passed": bool((verification or {}).get("passed")),
            "stages": (verification or {}).get("stages"),
            "all_stages_passed": (verification or {}).get("all_stages_passed"),
            "stage_count": len((verification or {}).get("stages") or []),
            "simulation_duration_s": (summary or {}).get("elapsed_simulation_s"),
            "randomization": (summary or {}).get("randomization"),
        })
    return evidence


def _exploration_evidence(directories):
    evidence = []
    for directory in directories or ():
        path = Path(directory)
        summary = _read_json(path / "navigation_summary.json")
        evidence.append({
            "directory": str(path.resolve()),
            "relative_directory": _relative(path),
            "passed": bool((summary or {}).get("passed")),
            "result": (summary or {}).get("result"),
            "exploration": (summary or {}).get("exploration"),
        })
    return evidence


def _monte_carlo_evidence(path):
    report = _read_json(path) if path else None
    if not report:
        return None
    aggregate = report.get("aggregate") or {}
    return {
        "report": _relative(Path(path)),
        "scene": report.get("scene"), "mode": report.get("mode"), "profile": report.get("profile"),
        "seeds": report.get("seeds"),
        "runs": aggregate.get("runs"), "passed": aggregate.get("passed"),
        "success_rate": aggregate.get("success_rate"),
        "truth_error_m": aggregate.get("truth_error_m"),
        "collisions_total": aggregate.get("collisions_total"),
        "stage_pass_rates": aggregate.get("stage_pass_rates"),
        "failure_reasons": aggregate.get("failure_reasons"),
        "randomization_profiles": aggregate.get("randomization_profiles"),
    }


def _relative(path):
    try:
        return Path(path).resolve().relative_to(PROJECT).as_posix()
    except (ValueError, OSError):
        return None


def _parse_runs(values):
    parsed = []
    entries = []
    for value in values or DEFAULT_RUNS:
        entries.extend(part for part in value.replace(",", " ").split() if part)
    for item in entries:
        scene, _, seed = item.partition(":")
        if not seed:
            raise SystemExit("Expected --runs entries formatted as scene:seed, got %r" % (item,))
        parsed.append((scene.strip(), int(seed)))
    return parsed


def _hash_equal(runs):
    hashes = [run["source_hashes"] for run in runs if run["source_hashes"]]
    return bool(hashes) and all(value == hashes[0] for value in hashes)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="*", default=None,
                        help="scene:seed entries, default: %s" % (", ".join(DEFAULT_RUNS),))
    parser.add_argument("--scenarios", default="all", choices=["all", "none"])
    parser.add_argument("--no-replay", action="store_true", help="Skip the recorded-input replay check.")
    parser.add_argument("--no-tests", action="store_true", help="Skip the regression suite.")
    parser.add_argument("--video", action="store_true", help="Render the acceptance replay MP4.")
    parser.add_argument("--course-run", action="append", default=[],
                        help="Run directory containing a full-course verification (repeatable).")
    parser.add_argument("--explore-run", action="append", default=[],
                        help="Run directory containing an exploration-mode summary (repeatable).")
    parser.add_argument("--monte-carlo-report", type=Path, default=None,
                        help="report.json from tools/run_monte_carlo.py.")
    parser.add_argument("--gap", action="append", default=[],
                        help="Known capability gap to record verbatim (repeatable).")
    parser.add_argument("--jobs", type=int, default=3, help="Concurrent runs (default: 3).")
    parser.add_argument("--out", type=Path, default=PROJECT / "mujoco" / "navigation_acceptance.json")
    parser.add_argument("--deadline-s", type=float, default=NAVIGATION_TIMEOUT_S)
    args = parser.parse_args(argv)
    jobs = max(1, args.jobs)

    planned_runs = _parse_runs(args.runs)
    print("Navigation runs: %s" % ", ".join("%s@%d" % pair for pair in planned_runs))
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(_navigation_run, scene, seed, args.deadline_s) for scene, seed in planned_runs]
        navigation_runs = [future.result() for future in futures]

    scenarios = []
    if args.scenarios == "all":
        print("Scenarios: %s" % ", ".join(SCENARIOS))
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(jobs, len(SCENARIOS))) as pool:
            futures = [pool.submit(_scenario_run, name, SCENARIO_TIMEOUT_S) for name in SCENARIOS]
            scenarios = [future.result() for future in futures]

    replay = None
    replay_directory = next((run["directory"] for run in navigation_runs
                             if run["passed"] and run["scene"] == "ruins_reference"), None)
    if replay_directory is None:
        replay_directory = next((run["directory"] for run in navigation_runs if run["passed"]), None)
    if replay_directory and not args.no_replay:
        print("Replay: %s" % replay_directory)
        replay = _replay_run(replay_directory, args.deadline_s)
    video = _video_run(replay_directory, args.deadline_s) if (args.video and replay_directory) else None
    tests = None if args.no_tests else _tests_run(SCENARIO_TIMEOUT_S)

    course = _course_evidence(args.course_run)
    exploration = _exploration_evidence(args.explore_run)
    monte_carlo = _monte_carlo_evidence(args.monte_carlo_report)
    config = _read_json(PROJECT / "config" / "ruins_reference.json") or {}
    identical_hashes = _hash_equal(navigation_runs)
    run_passed = all(run["passed"] for run in navigation_runs)
    scenario_passed = all(scenario["passed"] for scenario in scenarios) if scenarios else None
    course_passed = all(entry["passed"] for entry in course) if course else None
    exploration_passed = all(entry["passed"] for entry in exploration) if exploration else None
    overall = bool(run_passed and identical_hashes
                   and (scenario_passed is not False) and (tests is None or tests["passed"])
                   and (replay is None or replay["passed"])
                   and (course_passed is not False) and (exploration_passed is not False))

    acceptance = {
        "passed": overall,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "scope": "mujoco_velocity_level_online_mapping_navigation",
        "runs": navigation_runs,
        "identical_source_hashes": identical_hashes,
        "regression_tests": tests,
        "scenarios": scenarios,
        "replay": replay,
        "video": video,
        "exploration": exploration,
        "course": course,
        "monte_carlo": monte_carlo,
        "known_gaps": list(args.gap),
        "assumptions": {
            "lidar": "config lidar + synthetic range noise/dropout/beam-block/latency per active profile",
            "geometry": "annotated reference dimensions, ramp exit adjustment authorized by user",
            "locomotion": "velocity-level mocap fixture with held joint pose; no gait or contact dynamics",
            "loop_closure": False,
            "hardware_validation": "pending sensor installation, extrinsic calibration and manufacturer interface verification",
            "randomization": "sampled ranges are simulation assumptions or vendor references until measured on hardware",
        },
        "applies_to_current_scene": True,
        "current_scene_revision": config.get("scene_revision"),
        "geometry_status": config.get("geometry_status"),
        "review_status": config.get("review_status"),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(acceptance, indent=2, allow_nan=False), encoding="utf-8")
    summary = {"passed": overall, "out": str(args.out), "runs": len(navigation_runs),
               "runs_passed": sum(run["passed"] for run in navigation_runs),
               "scenarios_passed": sum(scenario["passed"] for scenario in scenarios),
               "scenarios": len(scenarios), "identical_source_hashes": identical_hashes,
               "tests_passed": None if tests is None else tests["passed"],
               "replay_passed": None if replay is None else replay["passed"],
               "course_passed": course_passed, "exploration_passed": exploration_passed}
    print("ACCEPTANCE_FINISHED " + json.dumps(summary))
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())