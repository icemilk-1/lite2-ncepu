import json
from pathlib import Path
import sys
import unittest


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))

from navigation_core.course import CourseEvaluator, validate_course


class CourseNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((PROJECT / "config/ruins_reference.json").read_text(encoding="utf-8"))

    def test_configured_course_is_ordered_and_ends_at_finish(self):
        course = self.config["course"]
        validate_course(course)
        identifiers = [stage["id"] for stage in course["stages"]]
        self.assertEqual(identifiers[-1], "finish")
        self.assertIn("dead_end_exit", identifiers)
        self.assertIn("narrow_passage_rejection", identifiers)
        self.assertNotIn("round_pipe", identifiers)
        self.assertNotIn("rubble_return", identifiers)
        bypass = next(stage for stage in course["stages"] if stage["id"] == "obstacle_bypass")
        self.assertEqual(len(bypass["checkpoints"]), 1)
        self.assertEqual(course["excluded_stages"][0]["id"], "round_pipe")

    def test_truth_evaluator_rejects_early_finish_and_missing_events(self):
        course = {
            "stages": [
                {"id": "leg", "kind": "traverse", "timeout_s": 10,
                 "checkpoints": [{"xy_m": [1, 0], "radius_m": .1}]},
                {"id": "finish", "kind": "finish", "timeout_s": 10,
                 "checkpoints": [{"xy_m": [2, 0], "radius_m": .1}]},
            ]
        }
        evaluator = CourseEvaluator(course)
        evaluator.record(1, [2, 0, 0], .1)
        result = evaluator.summary([], {"collision_free": True}, "goal_reached")
        self.assertFalse(result["passed"])
        self.assertFalse(result["stages"][0]["passed"])

    def test_truth_evaluator_requires_reverse_motion(self):
        course = {
            "stages": [
                {"id": "exit", "kind": "dead_end", "timeout_s": 10,
                 "probe": {"center_xy_m": [0, 0], "yaw_rad": 0,
                           "depth_m": .2, "half_width_m": .2},
                 "checkpoints": [{"xy_m": [0, 0], "radius_m": .1},
                                 {"xy_m": [1, 0], "radius_m": .1, "reverse": True}]},
                {"id": "finish", "kind": "finish", "timeout_s": 10,
                 "checkpoints": [{"xy_m": [2, 0], "radius_m": .1}]},
            ]
        }
        evaluator = CourseEvaluator(course)
        evaluator.record(1, [0, 0, 0], .1)
        evaluator.record(2, [1, 0, 0], .1)
        self.assertEqual(len(evaluator.results[0]["checkpoints"]), 1)
        evaluator.record(3, [1, 0, 0], -.1)
        events = [{"stage_id": "exit", "event": "branch_closed"},
                  {"stage_id": "exit", "event": "stage_completed"},
                  {"stage_id": "finish", "event": "stage_completed"}]
        evaluator.record(4, [2, 0, 0], .1)
        result = evaluator.summary(events, {"collision_free": True}, "goal_reached")
        self.assertTrue(result["passed"])


if __name__ == "__main__":
    unittest.main()
