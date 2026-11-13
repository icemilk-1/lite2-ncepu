"""Regression checks for the Monte Carlo aggregation and gating tool (no MuJoCo)."""

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from tools import run_monte_carlo as monte_carlo


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def make_course_run(root, seed, passed, result, error_m, stages, collisions=0,
                    recoveries=0, randomization_values=None, elapsed=42.0):
    run_dir = root / ("mujoco_seed_%d" % seed)
    run_dir.mkdir(parents=True)
    checks = {"goal_reached": passed, "collision_free": collisions == 0,
              "all_stages_passed": passed, "stopped_for_one_second": True}
    summary = {"passed": passed, "result": result, "checks": checks, "random_seed": seed,
               "evaluation_only": {"goal_error_m": error_m,
                                   "collision_stop_events": collisions},
               "elapsed_simulation_s": elapsed}
    write_json(run_dir / "navigation_summary.json", summary)
    write_json(run_dir / "navigation_verification.json", summary)
    write_json(run_dir / "course_verification.json", {
        "passed": passed, "result": result, "checks": checks,
        "stages": [{"id": stage_id, "passed": stage_passed} for stage_id, stage_passed in stages],
        "recovery_count": recoveries, "elapsed_simulation_s": elapsed,
        "evaluation_only": {"goal_error_m": error_m}})
    write_json(run_dir / "collision_events.json", [{"index": index} for index in range(collisions)])
    if randomization_values is not None:
        write_json(run_dir / "randomization.json", {
            "schema_version": 1, "profile_id": "sim2real_v1", "seed": seed,
            "values": randomization_values,
            "provenance": {key: "sim_assumed" for key in randomization_values}})
    return run_dir


class MonteCarloTestCase(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary.name)
        self.passed_dir = make_course_run(
            self.root, 42, True, "goal_reached", 0.10,
            [("ramp", True), ("finish", True)], collisions=0, recoveries=1,
            randomization_values={"lidar.range_noise_m": 0.01,
                                  "odometry.linear_scale_error": 1.02,
                                  "actuation.command_latency_s": 0.04,
                                  "initial_pose.dx_m": -0.01})
        self.failed_dir = make_course_run(
            self.root, 43, False, "start_blocked", 0.90,
            [("ramp", True), ("finish", False)], collisions=2)
        self.error_dir = self.root / "mujoco_seed_999"
        self.error_dir.mkdir(parents=True)
        (self.error_dir / "error.log").write_text("boom: simulator died", encoding="utf-8")
        self.runs = [
            monte_carlo.collect_run(self.passed_dir, seed=42),
            monte_carlo.collect_run(self.failed_dir, seed=43, exit_code=1),
            monte_carlo.collect_run(self.error_dir, seed=999),
        ]

    def tearDown(self):
        self._temporary.cleanup()


class SeedParsingTests(unittest.TestCase):
    def test_range_notation(self):
        self.assertEqual(monte_carlo.parse_seeds("42-45"), [42, 43, 44, 45])
        self.assertEqual(monte_carlo.parse_seeds("42-42"), [42])

    def test_mixed_notation_and_whitespace(self):
        self.assertEqual(monte_carlo.parse_seeds(" 42-43, 47 ,49-50 "),
                         [42, 43, 47, 49, 50])

    def test_duplicates_are_removed_but_order_is_preserved(self):
        self.assertEqual(monte_carlo.parse_seeds("45,42,45-46"), [45, 42, 46])

    def test_invalid_seed_lists_are_rejected(self):
        for text in ("", "abc", "42-", "-42", "51-42", "42,,x", "42.5"):
            with self.assertRaises(ValueError, msg=repr(text)):
                monte_carlo.parse_seeds(text)


class CommandBuildingTests(unittest.TestCase):
    def test_command_includes_randomization_headless_and_seed(self):
        command = monte_carlo.build_command(42, profile="sim2real_v1")
        self.assertIn("run_mujoco.py", command[1])
        self.assertEqual(command[command.index("--scene") + 1], "ruins_reference")
        self.assertIn("--course", command)
        self.assertIn("--headless", command)
        self.assertEqual(command[command.index("--seed") + 1], "42")
        self.assertEqual(command[command.index("--randomize") + 1], "sim2real_v1")

    def test_command_omits_randomization_when_disabled(self):
        command = monte_carlo.build_command(43, profile=None, mode="explore")
        self.assertNotIn("--randomize", command)
        self.assertIn("--explore", command)

    def test_invalid_commands_are_rejected(self):
        with self.assertRaises(ValueError):
            monte_carlo.build_command(42, mode="teleport")
        with self.assertRaises(ValueError):
            monte_carlo.build_command(-1)

    def test_run_directory_is_parsed_from_stdout(self):
        stdout = "Run output: F:\\airace\\lite2_sim\\runs\\mujoco_20261112_101010_abcdef\nbody"
        self.assertEqual(monte_carlo.extract_run_directory(stdout),
                         Path("F:\\airace\\lite2_sim\\runs\\mujoco_20261112_101010_abcdef"))
        self.assertEqual(monte_carlo.extract_run_directory(
            'Run output: "C:\\runs\\mujoco_1"\n'), Path("C:\\runs\\mujoco_1"))
        self.assertIsNone(monte_carlo.extract_run_directory("no marker here"))
        self.assertIsNone(monte_carlo.extract_run_directory(None))

    def test_dry_run_prints_the_planned_commands(self):
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = monte_carlo.main(["--dry-run", "--seeds", "42-43", "--no-randomize"])
        self.assertEqual(code, 0)
        printed = buffer.getvalue()
        self.assertIn("--seed 42", printed)
        self.assertIn("--seed 43", printed)
        self.assertNotIn("--randomize", printed)


class CollectionTests(MonteCarloTestCase):
    def test_collect_run_reads_course_and_randomization_artifacts(self):
        record = self.runs[0]
        self.assertTrue(record["passed"])
        self.assertEqual(record["result"], "goal_reached")
        self.assertEqual(record["stages"], [{"id": "ramp", "passed": True},
                                            {"id": "finish", "passed": True}])
        self.assertEqual(record["stage_pass"], {"ramp": True, "finish": True})
        self.assertAlmostEqual(record["truth_error_m"], 0.10)
        self.assertEqual(record["collisions"], 0)
        self.assertEqual(record["recoveries"], 1)
        self.assertAlmostEqual(record["elapsed_simulation_s"], 42.0)
        self.assertEqual(record["randomization"]["profile_id"], "sim2real_v1")
        self.assertEqual(record["randomization"]["seed"], 42)
        self.assertIsNone(record["error"])

    def test_failed_navigation_run_is_not_an_infrastructure_error(self):
        record = self.runs[1]
        self.assertFalse(record["passed"])
        self.assertEqual(record["result"], "start_blocked")
        self.assertEqual(record["collisions"], 2)
        self.assertIsNone(record["error"])
        self.assertEqual(record["exit_code"], 1)

    def test_missing_artifacts_are_recorded_as_a_run_error(self):
        record = self.runs[2]
        self.assertEqual(record["error"], "missing_result_artifacts")
        self.assertIn("boom", record["error_log"])
        self.assertFalse(record["passed"])

    def test_missing_directory_and_unparsed_stdout_never_raise(self):
        record = monte_carlo.collect_run(self.root / "absent", seed=7)
        self.assertEqual(record["error"], "run_directory_not_found")
        record = monte_carlo.collect_run(None, seed=8, stderr_tail="unrecognized arguments")
        self.assertEqual(record["error"], "run_directory_not_found")
        self.assertIn("unrecognized", record["error_log"])

    def test_navigate_run_without_course_artifacts_is_supported(self):
        run_dir = self.root / "mujoco_navigate_50"
        run_dir.mkdir()
        write_json(run_dir / "navigation_summary.json", {
            "passed": True, "result": "goal_reached",
            "checks": {"goal_reached": True},
            "evaluation_only": {"goal_error_m": 0.15}})
        record = monte_carlo.collect_run(run_dir, seed=50)
        self.assertTrue(record["passed"])
        self.assertEqual(record["stages"], [])
        self.assertEqual(record["stage_pass"], {})
        self.assertAlmostEqual(record["truth_error_m"], 0.15)
        self.assertIsNone(record["randomization"])

    def test_sampled_block_embedded_in_the_summary_is_used_as_a_fallback(self):
        run_dir = self.root / "mujoco_embedded_51"
        run_dir.mkdir()
        write_json(run_dir / "navigation_summary.json", {
            "passed": True, "result": "goal_reached",
            "randomization": {"profile_id": "sim2real_v1", "seed": 51,
                              "values": {"lidar.range_noise_m": 0.01},
                              "provenance": {"lidar.range_noise_m": "vendor_reference"}}})
        record = monte_carlo.collect_run(run_dir, seed=51)
        self.assertEqual(record["randomization"]["profile_id"], "sim2real_v1")
        self.assertEqual(record["randomization"]["seed"], 51)


class AggregationTests(MonteCarloTestCase):
    def test_counts_and_error_statistics(self):
        aggregate = monte_carlo.aggregate(self.runs)
        self.assertEqual(aggregate["runs"], 3)
        self.assertEqual(aggregate["passed"], 1)
        self.assertEqual(aggregate["failed"], 2)
        self.assertEqual(aggregate["error_runs"], 1)
        self.assertAlmostEqual(aggregate["success_rate"], 1.0 / 3.0)
        self.assertEqual(aggregate["truth_error_m"]["count"], 2)
        self.assertAlmostEqual(aggregate["truth_error_m"]["mean"], 0.5)
        self.assertAlmostEqual(aggregate["truth_error_m"]["std"], 0.4)
        self.assertAlmostEqual(aggregate["truth_error_m"]["min"], 0.1)
        self.assertAlmostEqual(aggregate["truth_error_m"]["max"], 0.9)
        self.assertEqual(aggregate["collisions_total"], 2)
        self.assertEqual(aggregate["recoveries_total"], 1)
        self.assertEqual(aggregate["randomization_profiles"], ["sim2real_v1"])

    def test_stage_pass_rates_keep_the_course_order(self):
        aggregate = monte_carlo.aggregate(self.runs)
        self.assertEqual(aggregate["stage_order"], ["ramp", "finish"])
        self.assertEqual(aggregate["stage_pass_rates"]["ramp"],
                         {"passed": 2, "total": 2, "rate": 1.0})
        self.assertEqual(aggregate["stage_pass_rates"]["finish"],
                         {"passed": 1, "total": 2, "rate": 0.5})

    def test_failure_histogram_separates_navigation_and_infrastructure_failures(self):
        aggregate = monte_carlo.aggregate(self.runs)
        self.assertEqual(aggregate["failure_reasons"], {"run_error": 1, "start_blocked": 1})

    def test_threshold_gate(self):
        self.assertTrue(monte_carlo.aggregate(self.runs)["threshold_met"])
        self.assertTrue(monte_carlo.aggregate(self.runs, min_pass_rate=0.3)["threshold_met"])
        self.assertFalse(monte_carlo.aggregate(self.runs, min_pass_rate=0.5)["threshold_met"])
        self.assertEqual(monte_carlo.aggregate(self.runs, min_pass_rate=0.5)["min_pass_rate"], 0.5)

    def test_empty_run_list_is_safe(self):
        aggregate = monte_carlo.aggregate([])
        self.assertEqual(aggregate["runs"], 0)
        self.assertEqual(aggregate["success_rate"], 0.0)
        self.assertIsNone(aggregate["truth_error_m"]["mean"])
        self.assertFalse(monte_carlo.aggregate([], min_pass_rate=0.0)["threshold_met"])


class ReportTests(MonteCarloTestCase):
    def metadata(self):
        return {"scene": "ruins_reference", "mode": "course", "profile": "sim2real_v1",
                "seeds": [42, 43, 999], "jobs": 3, "python": "python.exe",
                "min_pass_rate": 0.5,
                "command_template": ["python.exe", "run_mujoco.py", "--course"]}

    def test_report_orders_runs_by_seed_and_stays_json_serializable(self):
        report = monte_carlo.build_report([self.runs[1], self.runs[2], self.runs[0]],
                                          self.metadata())
        self.assertEqual([run["seed"] for run in report["runs"]], [42, 43, 999])
        self.assertEqual(report["profile"], "sim2real_v1")
        self.assertEqual(report["aggregate"]["min_pass_rate"], 0.5)
        self.assertFalse(report["aggregate"]["threshold_met"])
        self.assertIn("generated_at_utc", report)
        json.dumps(report, allow_nan=False)

    def test_write_report_creates_json_and_markdown(self):
        report = monte_carlo.build_report(self.runs, self.metadata())
        json_path, markdown_path = monte_carlo.write_report(report, self.root / "report")
        self.assertTrue(json_path.is_file())
        self.assertTrue(markdown_path.is_file())
        loaded = json.loads(json_path.read_text(encoding="utf-8"))
        self.assertEqual(loaded["aggregate"]["runs"], 3)
        text = markdown_path.read_text(encoding="utf-8")
        for heading in ("# Monte Carlo navigation report", "## Summary",
                        "## Per-run results", "## Stage pass rates",
                        "## Failure reasons", "## Sampled randomization", "## Errors"):
            self.assertIn(heading, text)
        self.assertIn("| 42 |", text)
        self.assertIn("start_blocked", text)
        self.assertIn("run_error", text)
        self.assertIn("sim2real_v1", text)

    def test_markdown_renders_without_course_stages_or_randomization(self):
        report = monte_carlo.build_report([self.runs[2]], {
            "scene": "ruins_reference", "mode": "navigate", "profile": None,
            "seeds": [999], "jobs": 1, "python": "python.exe"})
        text = monte_carlo.render_markdown(report)
        self.assertNotIn("## Stage pass rates", text)
        self.assertNotIn("## Sampled randomization", text)
        self.assertIn("not gated", text)

    def test_markdown_escapes_pipe_characters(self):
        run = dict(self.runs[0])
        run["result"] = "goal|reached"
        report = monte_carlo.build_report([run], self.metadata())
        text = monte_carlo.render_markdown(report)
        self.assertIn("goal\\|reached", text)


if __name__ == "__main__":
    unittest.main()