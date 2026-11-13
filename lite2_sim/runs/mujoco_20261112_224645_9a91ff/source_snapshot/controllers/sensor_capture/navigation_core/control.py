"""Estimated-state task control and velocity-level command generation."""

import math

from navigation_core.contracts import VelocityCommand
from navigation_core.grid import wrap_angle


class NavigationController(object):
    def __init__(self, settings):
        self.settings = settings
        self.cancelled = False
        self._progress_pose = None
        self._last_progress_time = None

    def cancel(self):
        self.cancelled = True

    def update(self, now_s, pose, plan, observation, mission_elapsed_s, progress_active=True):
        now_s = float(now_s)
        pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        timeout = float(self.settings.get("command_timeout_s", 0.30))
        if self.cancelled:
            return self._decision("cancelled", 0.0, 0.0, now_s, timeout, plan.target_xy)
        if mission_elapsed_s >= float(self.settings["navigation_timeout_s"]):
            return self._decision("timeout", 0.0, 0.0, now_s, timeout, plan.target_xy)
        if (observation is None or not observation.valid or observation.timestamp_s > now_s + 1e-6
                or now_s - observation.timestamp_s > float(self.settings.get("observation_timeout_s", 0.36))):
            return self._decision("stale_observation", 0.0, 0.0, now_s, timeout, plan.target_xy)
        if plan.status in ("start_blocked", "goal_blocked", "map_disconnected", "insufficient_observations",
                           "start_out_of_map", "goal_out_of_map"):
            return self._decision(plan.status, 0.0, 0.0, now_s, timeout, plan.target_xy)
        stop_tolerance = float(self.settings.get("estimated_stop_tolerance_m", self.settings["goal_tolerance_m"]))
        if (plan.status == "path_found" and plan.goal_xy is not None
                and math.hypot(plan.goal_xy[0] - pose[0], plan.goal_xy[1] - pose[1]) <= stop_tolerance):
            return self._decision("goal_reached", 0.0, 0.0, now_s, timeout, plan.target_xy)
        if not plan.path:
            return self._decision("insufficient_observations", 0.0, 0.0, now_s, timeout, plan.target_xy)

        target = self._lookahead_target(pose, plan.path, float(self.settings.get("lookahead_m", 0.25)))
        cross_track_gain = float(self.settings.get("cross_track_gain", 0.0))
        if cross_track_gain > 0.0 and len(plan.path) >= 2:
            # Stanley-style lateral correction for narrow corridors: pull the
            # look-ahead point back onto the planned centre line so a 1 m pipe
            # entrance cannot be cut diagonally by accumulated lateral error.
            target = self._cross_track_target(pose, plan.path, target, cross_track_gain,
                                             bool(self.settings.get("reverse_only", False)))
        if plan.status == "exploring" and math.hypot(plan.path[-1][0] - pose[0], plan.path[-1][1] - pose[1]) <= float(self.settings.get("frontier_tolerance_m", 0.16)):
            self._progress_pose = pose[:2]
            self._last_progress_time = now_s
            return self._decision("frontier_reached", 0.0, 0.0, now_s, timeout, plan.target_xy)

        heading_error = wrap_angle(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2])
        obstacle_ranges = [math.hypot(px, py) for px, py, hit in observation.points
                           if hit and px > 0 and abs(math.atan2(py, px)) < float(self.settings["front_stop_angle_rad"])
                           and math.hypot(px, py) > float(self.settings.get("min_range_m", 0.08))]
        obstacle_ahead = bool(obstacle_ranges) and min(obstacle_ranges) < float(self.settings["front_stop_distance_m"])
        linear = float(self.settings["max_linear_speed_m_s"]) * max(0.0, math.cos(heading_error))
        if abs(heading_error) > float(self.settings.get("alignment_threshold_rad", 1.15)) or obstacle_ahead:
            linear = 0.0
        angular = max(-float(self.settings["max_yaw_speed_rad_s"]),
                      min(float(self.settings["max_yaw_speed_rad_s"]), 2.0 * heading_error))

        if progress_active:
            if self._progress_pose is None:
                self._progress_pose = pose[:2]
                self._last_progress_time = now_s
            elif math.hypot(pose[0] - self._progress_pose[0], pose[1] - self._progress_pose[1]) >= float(self.settings.get("progress_distance_m", 0.08)):
                self._progress_pose = pose[:2]
                self._last_progress_time = now_s
            elif self._last_progress_time is not None and now_s - self._last_progress_time > float(self.settings["stuck_timeout_s"]):
                return self._decision("stalled", 0.0, 0.0, now_s, timeout, plan.target_xy)

        status = "exploring" if plan.status == "exploring" else "navigating"
        return self._decision(status, linear, angular, now_s, timeout, plan.target_xy)

    @staticmethod
    def _lookahead_target(pose, path, lookahead_m):
        if len(path) < 2:
            return path[0]
        nearest = min(range(len(path)), key=lambda idx: math.hypot(path[idx][0] - pose[0], path[idx][1] - pose[1]))
        remaining = lookahead_m
        start = pose[:2]
        for index in range(min(nearest + 1, len(path) - 1), len(path)):
            end = path[index]
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            if distance >= remaining:
                fraction = remaining / max(distance, 1e-9)
                return (start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1]))
            remaining -= distance
            start = end
        return path[-1]

    @staticmethod
    def _nearest_on_path(pose, path):
        """Closest point on the polyline and its unit tangent."""
        best = None
        for index in range(len(path) - 1):
            ax, ay = path[index]
            bx, by = path[index + 1]
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                fraction = 0.0
            else:
                fraction = max(0.0, min(1.0, ((pose[0] - ax) * dx + (pose[1] - ay) * dy) / length_sq))
            point = (ax + fraction * dx, ay + fraction * dy)
            distance_sq = (pose[0] - point[0]) ** 2 + (pose[1] - point[1]) ** 2
            if best is None or distance_sq < best[0]:
                best = (distance_sq, point, dx, dy, length_sq)
        if best is None:
            return (path[0], (1.0, 0.0))
        _, point, dx, dy, length_sq = best
        length = math.sqrt(length_sq) if length_sq > 1e-12 else 1.0
        return point, (dx / length, dy / length)

    @staticmethod
    def _cross_track_target(pose, path, target, gain, reverse=False):
        """Shift a look-ahead point towards the path by the signed lateral error."""
        point, tangent = NavigationController._nearest_on_path(pose, path)
        tx, ty = tangent
        if reverse:
            tx, ty = -tx, -ty
        lateral = tx * (pose[1] - point[1]) - ty * (pose[0] - point[0])
        shift = max(-0.30, min(0.30, -gain * lateral))
        return (target[0] - ty * shift, target[1] + tx * shift)

    @staticmethod
    def _decision(status, linear, angular, now_s, timeout, target):
        command = VelocityCommand(linear, angular, now_s, now_s + timeout)
        return NavigationDecision(status, command, target)


class NavigationDecision(object):
    __slots__ = ("status", "command", "target_xy")

    def __init__(self, status, command, target_xy=None):
        self.status = str(status)
        self.command = command
        self.target_xy = tuple(target_xy) if target_xy is not None else None
