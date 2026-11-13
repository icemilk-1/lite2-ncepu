"""Simulation reference metrics, kept outside navigation decision code."""

import math


class NavigationEvaluator(object):
    def __init__(self, initial_truth_pose):
        self.final_truth_pose = list(initial_truth_pose)
        self.distance_travelled_m = 0.0
        self.collision_stop_events = 0

    def record_execution(self, truth_pose, distance_m, collision_stopped):
        self.final_truth_pose = list(truth_pose)
        self.distance_travelled_m += float(distance_m)
        if collision_stopped:
            self.collision_stop_events += 1

    def summary(self, goal_xy, scan_match_corrections, goal_tolerance_m):
        """Goal metrics are unavailable for exploration runs, which have no target."""
        if goal_xy is None:
            return {
                "final_truth_xy_m": self.final_truth_pose[:2],
                "goal_error_m": None,
                "goal_tolerance_m": None,
                "truth_goal_tolerance_met": None,
                "distance_travelled_m": self.distance_travelled_m,
                "scan_match_corrections": int(scan_match_corrections),
                "collision_stop_events": self.collision_stop_events,
            }
        goal_error = math.hypot(goal_xy[0] - self.final_truth_pose[0],
                                goal_xy[1] - self.final_truth_pose[1])
        return {
            "final_truth_xy_m": self.final_truth_pose[:2],
            "goal_error_m": goal_error,
            "goal_tolerance_m": float(goal_tolerance_m),
            "truth_goal_tolerance_met": goal_error <= float(goal_tolerance_m),
            "distance_travelled_m": self.distance_travelled_m,
            "scan_match_corrections": int(scan_match_corrections),
            "collision_stop_events": self.collision_stop_events,
        }
