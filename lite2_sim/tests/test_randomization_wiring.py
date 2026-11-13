"""Wiring tests for randomization profiles and exploration-mode summaries.

These cover the lead-owned integration points only: profile sampling that feeds
``config["lidar"]``, the audit record, goal-less map export and the exploration
traversability audit. Behaviour of the navigation core itself is covered by the
other test modules.
"""

import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))

from mujoco_navigation import build_randomization, traversability_audit
from navigation_core import artifacts, evaluation
from navigation_core.grid import OccupancyGrid


class RandomizationWiringTests(unittest.TestCase):
    def _config(self, profile=None):
        config = json.loads((PROJECT / "config/sensor_lab.json").read_text(encoding="utf-8"))
        config["navigation"] = {"random_seed": 42, "goal_xy_m": [2.0, -3.0]}
        config["lidar"] = {
            "min_range_m": 0.08, "max_range_m": 8.0, "frequency_hz": 10.0,
            "mount_xy_m": [0.0, 0.0], "mount_xyz_m": [0.0, 0.0, 0.1],
            "model_status": "test", "noise_m": 0.0, "dropout_probability": 0.0,
        }
        if profile is not None:
            config["randomization_profile"] = profile
        return config

    def test_ideal_profile_preserves_the_sensor_baseline(self):
        config = self._config("ideal")
        with tempfile.TemporaryDirectory() as output:
            sampled = build_randomization(config, output)
            record = json.loads((Path(output) / "randomization.json").read_text(encoding="utf-8"))
        self.assertEqual(sampled["profile_id"], "ideal")
        self.assertEqual(config["lidar"]["noise_m"], 0.0)
        self.assertEqual(config["lidar"]["range_bias_m"], 0.0)
        self.assertEqual(config["lidar"]["dropout_probability"], 0.0)
        self.assertEqual(config["lidar"]["beam_block_probability"], 0.0)
        self.assertEqual(config["lidar"]["scan_latency_s"], 0.0)
        self.assertEqual(record["values"]["initial_pose.dx_m"], 0.0)
        self.assertEqual(record["provenance_summary"].get("measured_pending", 0), 0)
        self.assertEqual(record["provenance_summary"]["sim_assumed"], len(record["values"]))
        self.assertEqual(config["randomization_sampled"]["seed"], 42)

    def test_profile_sampling_is_reproducible_and_bounded(self):
        first, second = self._config("sim2real_v1"), self._config("sim2real_v1")
        with tempfile.TemporaryDirectory() as output:
            sampled_first = build_randomization(first, output)
        with tempfile.TemporaryDirectory() as output:
            sampled_second = build_randomization(second, output)
        self.assertEqual(sampled_first["values"], sampled_second["values"])
        self.assertEqual(first["lidar"], second["lidar"])
        for key in ("noise_m", "range_bias_m", "dropout_probability",
                    "beam_block_probability", "scan_latency_s"):
            self.assertIn(key, first["lidar"])
        self.assertEqual(len(sampled_first["provenance"]), len(sampled_first["values"]))
        self.assertTrue(0.0 <= first["lidar"]["dropout_probability"] < 1.0)
        self.assertLessEqual(first["lidar"]["scan_latency_s"], 0.12)
        self.assertLessEqual(first["lidar"]["noise_m"], 0.03)

    def test_disabled_profile_leaves_sensor_config_untouched(self):
        config = self._config()
        before = dict(config["lidar"])
        with tempfile.TemporaryDirectory() as output:
            self.assertIsNone(build_randomization(config, output))
            self.assertFalse((Path(output) / "randomization.json").exists())
        self.assertEqual(config["lidar"], before)
        self.assertIsNone(config["randomization_sampled"])


class GoalLessSummaryTests(unittest.TestCase):
    def test_save_map_accepts_no_goal(self):
        grid = OccupancyGrid([4.0, 4.0], 0.1)
        with tempfile.TemporaryDirectory() as output:
            metadata = artifacts.save_map(grid, output, [], (-1.0, -1.0), None, 0.28, 0.7)
        self.assertEqual(metadata["width_cells"], 40)
        self.assertEqual(metadata["integrated_scans"], 0)

    def test_evaluation_reports_null_goal_metrics(self):
        evaluator = evaluation.NavigationEvaluator([0.0, 0.0, 0.0])
        metrics = evaluator.summary(None, 3, 0.25)
        self.assertIsNone(metrics["goal_error_m"])
        self.assertIsNone(metrics["truth_goal_tolerance_met"])
        self.assertEqual(metrics["scan_match_corrections"], 3)


class TraversabilityAuditTests(unittest.TestCase):
    def test_audit_finds_occupied_cells_under_the_body(self):
        grid = OccupancyGrid([4.0, 4.0], 0.1)
        grid.log_odds[:, :] = -1.0
        grid.log_odds[20:22, 20:22] = 2.0
        trace = [(0.0, 0.0, 0.0, 0.0)]
        audit = traversability_audit(grid, trace, {"occupied_log_odds_threshold": 0.7}, 0.7)
        self.assertGreater(audit["traversed_body_cells"], 0)
        self.assertGreater(audit["traversed_occupied_cells"], 0)
        self.assertGreater(audit["traversability_contradiction_ratio"], 0.0)
        self.assertGreaterEqual(audit["map_completeness_ratio"], 0.0)

    def test_audit_reports_unknown_under_the_body(self):
        grid = OccupancyGrid([4.0, 4.0], 0.1)
        trace = [(0.0, 0.0, 0.0, 0.0)]
        audit = traversability_audit(grid, trace, {"occupied_log_odds_threshold": 0.7}, 0.7)
        self.assertEqual(audit["traversed_occupied_cells"], 0)
        self.assertGreater(audit["traversed_unknown_cells"], 0)
        self.assertAlmostEqual(audit["traversed_unknown_ratio"], 1.0)


if __name__ == "__main__":
    unittest.main()