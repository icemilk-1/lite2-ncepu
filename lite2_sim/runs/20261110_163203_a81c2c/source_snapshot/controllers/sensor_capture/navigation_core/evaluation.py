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

    def summary(self, goal_xy, scan_match_corrections):
        return {
            "final_truth_xy_m": self.final_truth_pose[:2],
            "goal_error_m": math.hypot(goal_xy[0] - self.final_truth_pose[0],
                                        goal_xy[1] - self.final_truth_pose[1]),
            "distance_travelled_m": self.distance_travelled_m,
            "scan_match_corrections": int(scan_match_corrections),
            "collision_stop_events": self.collision_stop_events,
        }
