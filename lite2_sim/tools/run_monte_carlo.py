"""Batch Monte Carlo evaluation of randomized navigation runs.

Each seed is executed as its own ``run_mujoco.py`` subprocess.  Per-run result
artifacts (``navigation_summary.json``, ``course_verification.json`` and
``randomization.json``) are collected into a ``report.json`` plus a human
readable ``report.md``.  A failing run is recorded with an ``error`` field and
never aborts the batch.  ``--min-pass-rate`` turns the pass rate into a gate:
the tool exits non-zero when the rate is below the threshold.

Usage:
    python tools/run_monte_carlo.py --scene ruins_reference --course \
        --profile sim2real_v1 --seeds 42-51 --jobs 3 [--out DIR] [--min-pass-rate 0.8]
"""

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import time
import uuid


PROJECT = Path(__file__).resolve().parents[1]
RUNS_DIRECTORY = PROJECT / "runs"
DEFAULT_PYTHON = PROJECT / ".venv-mujoco" / "Scripts" / "python.exe"
RUN_DIRECTORY_PATTERN = re.compile(r"^Run output:\s*(.+?)\s*$", re.MULTILINE)
RESULT_ARTIFACTS = ("navigation_summary.json", "navigation_verification.json",
                    "course_verification.json", "randomization.json")
MODES = ("course", "explore", "navigate")
RANDOMIZATION_HELPER_KEYS = ("lidar.noise_m", "lidar.range_noise_m",
                             "odometry.linear_scale_error", "odometry.yaw_scale_error",
                             "actuation.command_latency_s",
                             "initial_pose.dx_m", "initial_pose.dy_m")


def parse_seeds(text):
    """Parse ``"42-51"``, ``"42,45,50"`` or a mixed list into ordered unique seeds."""
    if not isinstance(text, str) or not text.strip():
        raise ValueError("--seeds must be a non-empty string such as '42-51' or '42,45,50'.")
    seeds = []
    for chunk in text.replace(" ", "").split(","):
        if not chunk:
            continue
        if "-" in chunk:
            parts = chunk.split("-")
            if len(parts) != 2 or not all(part.isdigit() for part in parts):
                raise ValueError("Invalid seed range %r; use 'lo-hi' with non-negative integers." % (chunk,))
            low, high = int(parts[0]), int(parts[1])
            if low > high:
                raise ValueError("Invalid seed range %r; the low bound exceeds the high bound." % (chunk,))
            candidates = range(low, high + 1)
        else:
            if not chunk.isdigit():
                raise ValueError("Invalid seed %r; seeds must be non-negative integers." % (chunk,))
            candidates = (int(chunk),)
        for seed in candidates:
            if seed not in seeds:
                seeds.append(seed)
    if not seeds:
        raise ValueError("--seeds did not contain any seed.")
    return seeds


def build_command(seed, scene="ruins_reference", mode="course", profile="sim2real_v1",
                  config=None, headless=True, python=None):
    """Build the ``run_mujoco.py`` argv list for one seed."""
    if mode not in MODES:
        raise ValueError("Unsupported navigation mode %r (expected one of %s)."
                         % (mode, ", ".join(MODES)))
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("Seed must be a non-negative integer, got %r." % (seed,))
    executable = str(python or DEFAULT_PYTHON)
    command = [executable, str(PROJECT / "run_mujoco.py"), "--scene", scene, "--" + mode]
    if headless:
        command.append("--headless")
    command += ["--seed", str(seed)]
    if profile:
        command += ["--randomize", str(profile)]
    if config:
        command += ["--config", str(config)]
    return command


def extract_run_directory(stdout):
    """Return the run output directory printed by ``run_mujoco.py`` when available."""
    if not stdout:
        return None
    match = RUN_DIRECTORY_PATTERN.search(stdout)
    if not match:
        return None
    candidate = match.group(1).strip().strip('"').strip("'")
    return Path(candidate) if candidate else None


def collect_run(run_dir, seed=None, exit_code=0, duration_s=None, stderr_tail=None):
    """Build one per-run report record from a run directory.

    Missing or unparsable artifacts are reported through the ``error`` field
    instead of raising, so a broken run can never abort the batch.
    """
    record = {
        "seed": seed,
        "run_dir": None,
        "result": None,
        "passed": False,
        "checks": {},
        "stages": [],
        "stage_pass": {},
        "truth_error_m": None,
        "collisions": 0,
        "recoveries": 0,
        "elapsed_simulation_s": None,
        "duration_s": None if duration_s is None else round(float(duration_s), 3),
        "randomization": None,
        "exit_code": exit_code,
        "error": None,
        "error_log": stderr_tail[:4000] if stderr_tail else None,
    }
    if run_dir is None:
        record["error"] = "run_directory_not_found"
        return record
    path = Path(run_dir)
    record["run_dir"] = _display_path(path)
    if not path.is_dir():
        record["error"] = "run_directory_not_found"
        return record

    summary = _read_json(path / "navigation_summary.json") or _read_json(
        path / "navigation_verification.json")
    course = _read_json(path / "course_verification.json")
    if summary is None and course is None:
        record["error"] = "missing_result_artifacts"
        error_log = _read_text(path / "error.log")
        if error_log:
            record["error_log"] = error_log[-4000:]
        return record

    result_source = course or summary
    record["result"] = result_source.get("result", summary.get("result") if summary else None)
    record["passed"] = bool(result_source.get("passed", False))
    checks = result_source.get("checks")
    record["checks"] = dict(checks) if isinstance(checks, dict) else {}

    if course is not None:
        stages = course.get("stages")
        if isinstance(stages, list):
            record["stages"] = [{"id": stage.get("id"), "passed": bool(stage.get("passed"))}
                                for stage in stages if isinstance(stage, dict)]
            record["stage_pass"] = {stage["id"]: stage["passed"] for stage in record["stages"]}
        recoveries = course.get("recovery_count")
        if _is_number(recoveries):
            record["recoveries"] = int(recoveries)

    evaluation = result_source.get("evaluation_only")
    if not isinstance(evaluation, dict) and summary is not None:
        evaluation = summary.get("evaluation_only")
    if isinstance(evaluation, dict):
        if _is_number(evaluation.get("goal_error_m")):
            record["truth_error_m"] = float(evaluation["goal_error_m"])
        if record["collisions"] == 0 and _is_number(evaluation.get("collision_stop_events")):
            record["collisions"] = int(evaluation["collision_stop_events"])

    collision_events = _read_json(path / "collision_events.json")
    if isinstance(collision_events, list):
        record["collisions"] = len(collision_events)

    for key in ("elapsed_simulation_s", "elapsed_simulation_time_s"):
        source = result_source if result_source.get(key) is not None else summary
        if source is not None and _is_number(source.get(key)):
            record["elapsed_simulation_s"] = float(source[key])
            break

    randomization = _read_json(path / "randomization.json")
    if not isinstance(randomization, dict) and isinstance(summary, dict):
        # Older runs only embed the sampled block in the summary.
        embedded = summary.get("randomization")
        if isinstance(embedded, dict):
            randomization = embedded
    if isinstance(randomization, dict):
        record["randomization"] = randomization

    error = result_source.get("error")
    if error:
        record["error"] = str(error)
    if record["result"] == "runtime_error" and not record["error"]:
        record["error"] = "runtime_error"
    if record["error"] and not record["error_log"]:
        error_log = _read_text(path / "error.log")
        if error_log:
            record["error_log"] = error_log[-4000:]
    return record


def aggregate(runs, min_pass_rate=None):
    """Aggregate per-run records into the report summary block."""
    total = len(runs)
    passed = [run for run in runs if run.get("passed")]
    errors = [run for run in runs if run.get("error")]
    success_rate = (len(passed) / total) if total else 0.0

    truth_errors = [float(run["truth_error_m"]) for run in runs
                    if _is_number(run.get("truth_error_m"))]
    error_stats = {"count": len(truth_errors)}
    if truth_errors:
        error_stats.update(mean=statistics.fmean(truth_errors),
                           std=statistics.pstdev(truth_errors) if len(truth_errors) > 1 else 0.0,
                           min=min(truth_errors), max=max(truth_errors))
    else:
        error_stats.update(mean=None, std=None, min=None, max=None)

    stage_order, stage_totals, stage_passed = [], {}, {}
    for run in runs:
        for stage in run.get("stages") or []:
            stage_id = stage.get("id")
            if stage_id is None:
                continue
            if stage_id not in stage_totals:
                stage_order.append(stage_id)
                stage_totals[stage_id] = 0
                stage_passed[stage_id] = 0
            stage_totals[stage_id] += 1
            if stage.get("passed"):
                stage_passed[stage_id] += 1
    stage_pass_rates = {stage_id: {"passed": stage_passed[stage_id],
                                   "total": stage_totals[stage_id],
                                   "rate": stage_passed[stage_id] / stage_totals[stage_id]}
                        for stage_id in stage_order}

    failure_reasons = Counter()
    for run in runs:
        if run.get("passed"):
            continue
        reason = "run_error" if run.get("error") else (run.get("result") or "unknown")
        failure_reasons[str(reason)] += 1

    elapsed = [float(run["elapsed_simulation_s"]) for run in runs
               if _is_number(run.get("elapsed_simulation_s"))]
    durations = [float(run["duration_s"]) for run in runs if _is_number(run.get("duration_s"))]
    profiles = sorted({str(run["randomization"].get("profile_id"))
                       for run in runs
                       if isinstance(run.get("randomization"), dict)
                       and run["randomization"].get("profile_id")})

    if min_pass_rate is None:
        threshold_met = True
    else:
        # An empty batch is no evidence of success even for a zero threshold.
        threshold_met = bool(total) and success_rate >= float(min_pass_rate)
    return {
        "runs": total,
        "passed": len(passed),
        "failed": total - len(passed),
        "error_runs": len(errors),
        "success_rate": success_rate,
        "truth_error_m": error_stats,
        "stage_order": stage_order,
        "stage_pass_rates": stage_pass_rates,
        "failure_reasons": dict(sorted(failure_reasons.items(),
                                       key=lambda item: (-item[1], item[0]))),
        "collisions_total": sum(int(run.get("collisions") or 0) for run in runs),
        "recoveries_total": sum(int(run.get("recoveries") or 0) for run in runs),
        "mean_elapsed_simulation_s": statistics.fmean(elapsed) if elapsed else None,
        "mean_wall_duration_s": statistics.fmean(durations) if durations else None,
        "randomization_profiles": profiles,
        "min_pass_rate": min_pass_rate,
        "threshold_met": threshold_met,
    }


def build_report(runs, metadata):
    """Assemble the full ``report.json`` payload."""
    ordered = sorted(runs, key=lambda run: (run.get("seed") is None, run.get("seed")))
    report = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "scene": metadata.get("scene"),
        "mode": metadata.get("mode"),
        "profile": metadata.get("profile"),
        "seeds": metadata.get("seeds", [run.get("seed") for run in ordered]),
        "jobs": metadata.get("jobs"),
        "python": metadata.get("python"),
        "command_template": metadata.get("command_template"),
        "randomization_enabled": bool(metadata.get("profile")),
        "runs": ordered,
    }
    report["aggregate"] = aggregate(ordered, metadata.get("min_pass_rate"))
    return report


def render_markdown(report):
    """Render the human-readable ``report.md`` table view."""
    aggregate_block = report["aggregate"]
    lines = ["# Monte Carlo navigation report", ""]
    lines += ["- generated_at_utc: `%s`" % report.get("generated_at_utc"),
              "- scene: `%s`" % report.get("scene"),
              "- mode: `%s`" % report.get("mode"),
              "- profile: `%s`" % (report.get("profile") or "none"),
              "- seeds: `%s`" % _format_seeds(report.get("seeds") or []),
              "- jobs: `%s`" % report.get("jobs"),
              "- min_pass_rate: `%s`" % (report.get("aggregate", {}).get("min_pass_rate")
                                          if report.get("aggregate", {}).get("min_pass_rate") is not None
                                          else "not gated"),
              ""]
    template = report.get("command_template")
    if template:
        lines += ["Command template:", "", "```text", " ".join(str(part) for part in template), "```", ""]

    lines += ["## Summary", "", "| metric | value |", "| --- | --- |"]
    summary_rows = [
        ("runs", aggregate_block["runs"]),
        ("passed", aggregate_block["passed"]),
        ("failed", aggregate_block["failed"]),
        ("success_rate", "%.1f%%" % (100.0 * aggregate_block["success_rate"])),
        ("error_runs", aggregate_block["error_runs"]),
        ("truth_error_m mean", aggregate_block["truth_error_m"]["mean"]),
        ("truth_error_m std", aggregate_block["truth_error_m"]["std"]),
        ("truth_error_m min", aggregate_block["truth_error_m"]["min"]),
        ("truth_error_m max", aggregate_block["truth_error_m"]["max"]),
        ("collisions_total", aggregate_block["collisions_total"]),
        ("recoveries_total", aggregate_block["recoveries_total"]),
        ("mean_elapsed_simulation_s", aggregate_block["mean_elapsed_simulation_s"]),
        ("mean_wall_duration_s", aggregate_block["mean_wall_duration_s"]),
        ("randomization_profiles", ", ".join(aggregate_block["randomization_profiles"]) or "none"),
        ("threshold_met", aggregate_block["threshold_met"]),
    ]
    lines += ["| %s | %s |" % (name, _cell(value)) for name, value in summary_rows]
    lines.append("")

    lines += ["## Per-run results", "",
              "| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |",
              "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for run in report["runs"]:
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            _cell(run.get("seed")), _cell(run.get("passed")), _cell(run.get("result")),
            _cell(run.get("truth_error_m")), _cell(run.get("collisions")),
            _cell(run.get("recoveries")), _cell(run.get("elapsed_simulation_s")),
            _cell(run.get("run_dir"))))
    lines.append("")

    stage_rates = aggregate_block["stage_pass_rates"]
    if stage_rates:
        lines += ["## Stage pass rates", "", "| stage | passed | total | rate |", "| --- | --- | --- | --- |"]
        for stage_id in aggregate_block["stage_order"]:
            stats = stage_rates[stage_id]
            lines.append("| %s | %d | %d | %.1f%% |" % (
                stage_id, stats["passed"], stats["total"], 100.0 * stats["rate"]))
        lines.append("")

    lines += ["## Failure reasons", "", "| reason | count |", "| --- | --- |"]
    if aggregate_block["failure_reasons"]:
        lines += ["| %s | %d |" % (reason, count)
                  for reason, count in aggregate_block["failure_reasons"].items()]
    else:
        lines.append("| none | 0 |")
    lines.append("")

    randomized = [run for run in report["runs"] if isinstance(run.get("randomization"), dict)]
    if randomized:
        lines += ["## Sampled randomization", "",
                  "| seed | profile | lidar noise m | linear scale | yaw scale | command latency s | dx m | dy m |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- |"]
        for run in randomized:
            values = run["randomization"].get("values") or {}
            lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
                _cell(run.get("seed")), _cell(run["randomization"].get("profile_id")),
                _cell(_first_value(values, "lidar.noise_m", "lidar.range_noise_m")),
                _cell(values.get("odometry.linear_scale_error")),
                _cell(values.get("odometry.yaw_scale_error")),
                _cell(values.get("actuation.command_latency_s")),
                _cell(values.get("initial_pose.dx_m")),
                _cell(values.get("initial_pose.dy_m"))))
        lines.append("")

    error_runs = [run for run in report["runs"] if run.get("error")]
    if error_runs:
        lines += ["## Errors", "", "| seed | error | stderr tail |", "| --- | --- | --- |"]
        for run in error_runs:
            tail = run.get("error_log") or ""
            lines.append("| %s | %s | %s |" % (
                _cell(run.get("seed")), _cell(run.get("error")),
                _cell(tail.strip().splitlines()[-1] if tail.strip() else None)))
        lines.append("")
    return "\n".join(lines) + "\n"


def write_report(report, out_dir):
    """Write ``report.json`` and ``report.md`` under ``out_dir``."""
    directory = Path(out_dir)
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / "report.json"
    markdown_path = directory / "report.md"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return json_path, markdown_path


def execute_run(seed, scene="ruins_reference", mode="course", profile="sim2real_v1",
                python=None, timeout_s=None, config=None, headless=True):
    """Run one seed as a subprocess and return its collected report record."""
    command = build_command(seed, scene=scene, mode=mode, profile=profile,
                            config=config, headless=headless, python=python)
    environment = dict(os.environ)
    environment.setdefault("PYTHONIOENCODING", "utf-8")
    environment["PYTHONUNBUFFERED"] = "1"
    started = time.perf_counter()
    stdout, stderr, exit_code, timed_out = "", "", 0, False
    try:
        completed = subprocess.run(command, cwd=str(PROJECT), capture_output=True, text=True,
                                   encoding="utf-8", errors="replace", timeout=timeout_s,
                                   env=environment)
        stdout = completed.stdout or ""
        stderr = completed.stderr or ""
        exit_code = completed.returncode
    except subprocess.TimeoutExpired as error:
        timed_out = True
        stdout = _decode_timeout_output(error.stdout)
        stderr = _decode_timeout_output(error.stderr)
        exit_code = -1
    except OSError as error:
        duration = time.perf_counter() - started
        record = collect_run(None, seed=seed, exit_code=-1, duration_s=duration,
                             stderr_tail=str(error))
        record["command"] = command
        record["error"] = "run_mujoco_launch_failed"
        return record
    duration = time.perf_counter() - started
    record = collect_run(extract_run_directory(stdout), seed=seed, exit_code=exit_code,
                         duration_s=duration, stderr_tail=stderr)
    record["command"] = command
    record["stdout_tail"] = stdout[-2000:] if stdout else None
    if timed_out:
        record["error"] = "timeout_after_%s_s" % (timeout_s,)
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", default="ruins_reference")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--course", action="store_true", help="Score the ordered course (default).")
    mode.add_argument("--explore", action="store_true", help="Run pure exploration mapping.")
    mode.add_argument("--navigate", action="store_true", help="Run single-goal navigation.")
    parser.add_argument("--profile", default="sim2real_v1",
                        help="Randomization profile id from config/randomization (default: sim2real_v1).")
    parser.add_argument("--no-randomize", action="store_true",
                        help="Run the ideal baseline without --randomize.")
    parser.add_argument("--seeds", default="42-51", help="Seed list such as '42-51' or '42,45,50'.")
    parser.add_argument("--jobs", type=int, default=1, help="Number of concurrent runs (default: 1).")
    parser.add_argument("--out", type=Path, help="Report directory (default: runs/monte_carlo_<stamp>).")
    parser.add_argument("--min-pass-rate", type=float, default=None,
                        help="Exit non-zero when the pass rate is below this value (0..1).")
    parser.add_argument("--timeout-s", type=float, default=3600.0,
                        help="Per-run wall clock timeout in seconds (default: 3600).")
    parser.add_argument("--python", type=Path, default=None,
                        help="Python executable for run_mujoco.py (default: .venv-mujoco).")
    parser.add_argument("--config", type=Path, default=None, help="Pass-through run_mujoco.py --config.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the planned per-seed commands and exit without running.")
    args = parser.parse_args(argv)

    try:
        seeds = parse_seeds(args.seeds)
    except ValueError as error:
        parser.error(str(error))
    if args.jobs < 1:
        parser.error("--jobs must be at least 1.")
    if args.min_pass_rate is not None and not 0.0 <= args.min_pass_rate <= 1.0:
        parser.error("--min-pass-rate must be within [0, 1].")
    if args.explore:
        mode_name = "explore"
    elif args.navigate:
        mode_name = "navigate"
    else:
        mode_name = "course"
    profile = None if args.no_randomize else args.profile
    python = args.python or DEFAULT_PYTHON

    out_dir = args.out or (RUNS_DIRECTORY / ("monte_carlo_" + datetime.now().strftime("%Y%m%d_%H%M%S")
                                             + "_" + uuid.uuid4().hex[:6]))
    metadata = {
        "scene": args.scene, "mode": mode_name, "profile": profile, "seeds": seeds,
        "jobs": args.jobs, "python": str(python), "min_pass_rate": args.min_pass_rate,
        "command_template": build_command(seeds[0], scene=args.scene, mode=mode_name,
                                          profile=profile, config=args.config, python=python),
    }
    if args.dry_run:
        for seed in seeds:
            print(" ".join(build_command(seed, scene=args.scene, mode=mode_name, profile=profile,
                                         config=args.config, python=python)))
        print("Report directory would be:", out_dir)
        return 0
    if not Path(python).is_file():
        print("ERROR: interpreter not found: %s" % (python,), file=sys.stderr)
        return 2

    print("Monte Carlo: %d seed(s), %d job(s), mode=%s, profile=%s" % (
        len(seeds), args.jobs, mode_name, profile or "none"), flush=True)
    print("Report directory:", out_dir, flush=True)
    runs = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(execute_run, seed, scene=args.scene, mode=mode_name,
                               profile=profile, python=python, timeout_s=args.timeout_s,
                               config=args.config): seed
                   for seed in seeds}
        for future in as_completed(futures):
            seed = futures[future]
            try:
                record = future.result()
            except Exception as error:  # noqa: BLE001 - a single run must never abort the batch
                record = collect_run(None, seed=seed, exit_code=-1, stderr_tail=repr(error))
                record["error"] = "run_execution_exception"
            runs.append(record)
            print("seed %s: passed=%s result=%s error=%s" % (
                seed, record.get("passed"), record.get("result"), record.get("error")), flush=True)

    report = build_report(runs, metadata)
    json_path, markdown_path = write_report(report, out_dir)
    aggregate_block = report["aggregate"]
    print("MONTE_CARLO_FINISHED " + json.dumps({
        "report_dir": str(out_dir), "report_json": str(json_path),
        "report_markdown": str(markdown_path),
        "runs": aggregate_block["runs"], "passed": aggregate_block["passed"],
        "success_rate": round(aggregate_block["success_rate"], 6),
        "threshold_met": aggregate_block["threshold_met"]}), flush=True)
    if aggregate_block["error_runs"] == aggregate_block["runs"] and aggregate_block["runs"]:
        print("ERROR: every run failed to produce usable artifacts; see report.md.", file=sys.stderr)
        return 1
    if args.min_pass_rate is not None and not aggregate_block["threshold_met"]:
        print("ERROR: success rate %.3f is below --min-pass-rate %.3f." % (
            aggregate_block["success_rate"], args.min_pass_rate), file=sys.stderr)
        return 1
    return 0


def _read_json(path):
    path = Path(path)
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def _read_text(path):
    path = Path(path)
    if not path.is_file():
        return None
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _display_path(path):
    try:
        return Path(path).resolve().relative_to(PROJECT).as_posix()
    except ValueError:
        return str(Path(path).resolve()).replace("\\", "/")


def _format_seeds(seeds):
    numbers = [seed for seed in seeds if _is_number(seed)]
    if not numbers:
        return ""
    if len(numbers) == 1:
        return str(numbers[0])
    if numbers == list(range(numbers[0], numbers[0] + len(numbers))):
        return "%d-%d" % (numbers[0], numbers[-1])
    return ",".join(str(seed) for seed in numbers)


def _cell(value):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return "%.4f" % value
    return str(value).replace("|", "\\|").replace("\n", " ").strip()


def _first_value(values, *keys):
    for key in keys:
        if key in values:
            return values[key]
    return None


def _decode_timeout_output(output):
    if output is None:
        return ""
    if isinstance(output, bytes):
        return output.decode("utf-8", errors="replace")
    return str(output)


if __name__ == "__main__":
    sys.exit(main())