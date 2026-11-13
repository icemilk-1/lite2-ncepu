"""Simulator-independent mapping, exploration and navigation lifecycle."""

import math

import numpy as np

from navigation_core.contracts import VelocityCommand
from navigation_core.control import NavigationController
from navigation_core.grid import OccupancyGrid, wrap_angle
from navigation_core.planner import PlanResult


TERMINAL_STATES = frozenset((
    "goal_reached", "timeout", "stalled", "cancelled", "start_blocked",
    "goal_blocked", "map_disconnected", "insufficient_observations",
    "start_out_of_map", "goal_out_of_map", "localization_lost", "collision"))


class NavigationSession(object):
    """Consumes timestamped observations and measured motion, emits expiring commands."""

    def __init__(self, config, initial_pose, grid=None):
        self.settings = dict(config["navigation"])
        self.lidar = config["lidar"]
        self.settings["min_range_m"] = self.lidar["min_range_m"]
        self.grid = grid or OccupancyGrid(config["field_size_m"], self.settings["grid_resolution_m"])
        self.grid.frontier_visits = np.zeros_like(self.grid.log_odds)
        self.pose = list(initial_pose)
        self.odom_pose = list(initial_pose)
        self.goal = tuple(self.settings["goal_xy_m"])
        self.navigator = NavigationController(self.settings)
        self.plan = PlanResult("insufficient_observations", goal_xy=self.goal)
        self.status = "mapping"
        self.started_at = None
        self.last_replan = -math.inf
        self.last_observation = None
        self.history = []
        self.corrections = 0
        self.stopped_since = None
        self.last_scan_pose = None
        self.last_valid_motion = None

    @property
    def finished(self):
        return self.status in TERMINAL_STATES and self.status != "goal_reached" or (
            self.status == "goal_reached" and self.stopped_since is not None)

    def cancel(self):
        self.navigator.cancel()

    def propagate(self, motion, dt):
        if not motion.valid:
            return
        self.last_valid_motion = motion.timestamp_s
        v, w = motion.linear_x_m_s, motion.angular_z_rad_s
        for pose in (self.pose, self.odom_pose):
            mid = pose[2] + w * dt / 2
            pose[0] += v * math.cos(mid) * dt
            pose[1] += v * math.sin(mid) * dt
            pose[2] = wrap_angle(pose[2] + w * dt)

    def observe(self, observation):
        self.last_observation = observation
        if not observation.valid:
            return
        extrinsic = self.lidar["mount_xy_m"]
        # Re-matching a stationary scan causes map quantization to accumulate as drift.
        moved = self.last_scan_pose is not None and (
            math.hypot(self.pose[0] - self.last_scan_pose[0], self.pose[1] - self.last_scan_pose[1]) > 0.015
            or abs(wrap_angle(self.pose[2] - self.last_scan_pose[2])) > 0.02)
        if self.grid.scan_count and moved:
            prior = tuple(self.pose)
            self.pose = list(self.grid.match_pose(
                prior, observation.points, extrinsic, self.lidar["max_range_m"],
                self.settings["scan_match_search_xy_m"], self.settings["scan_match_search_yaw_rad"]))
            if math.hypot(self.pose[0] - prior[0], self.pose[1] - prior[1]) > 0.005:
                self.corrections += 1
        self.last_scan_pose = list(self.pose)
        self.grid.integrate(observation.points, self.pose, extrinsic,
                            self.lidar["max_range_m"], self.lidar["min_range_m"],
                            self.settings.get("scan_stride", 1))
        if hasattr(self.grid, "integrate_terrain"):
            self.grid.integrate_terrain(observation, self.pose, extrinsic)

    def _replan(self, now):
        self.plan = self.grid.plan_result(
            self.pose, self.goal, self.settings["robot_radius_m"],
            self.settings["occupied_log_odds_threshold"], self.settings["unknown_cell_cost"],
            self.settings.get("planning_clearance_m", 0.0))
        self.last_replan = now
        self.history.append({"sim_time_s": now, "status": self.plan.status,
                             "reason": self.plan.reason, "from_xy_m": self.pose[:2],
                             "goal_xy_m": self.goal, "target_xy_m": self.plan.target_xy,
                             "poses_xy_m": self.plan.path})

    def tick(self, now, motion=None, cancelled=False):
        if self.started_at is None:
            self.started_at = now
        ttl = self.settings["command_timeout_s"]
        zero = VelocityCommand(0, 0, now, now + ttl)
        if cancelled:
            self.cancel()
        if self.navigator.cancelled:
            self.status = "cancelled"
            return zero
        elapsed = now - self.started_at
        if elapsed >= self.settings["navigation_timeout_s"]:
            self.status = "timeout"
            return zero
        if self.last_valid_motion is not None and now - self.last_valid_motion > self.settings["observation_timeout_s"]:
            self.status = "localization_lost"
            return zero
        if self.status == "goal_reached":
            if motion is not None and abs(motion.linear_x_m_s) < 0.002 and abs(motion.angular_z_rad_s) < 0.005:
                if self.stopped_since is None:
                    self.stopped_since = now
            else:
                self.stopped_since = None
            return zero
        if self.status in TERMINAL_STATES:
            return zero
        if elapsed < self.settings["initial_mapping_duration_s"]:
            return zero
        if self.grid.scan_count < 3:
            if elapsed > 5:
                self.status = "insufficient_observations"
            return zero
        if now - self.last_replan >= self.settings["replan_period_s"]:
            self._replan(now)
        decision = self.navigator.update(now, self.pose, self.plan, self.last_observation, elapsed)
        self.status = decision.status
        if self.status == "frontier_reached":
            cell = self.grid.world_to_cell(*self.plan.target_xy)
            if cell is not None:
                column, row = cell
                radius = max(1, int(0.4 / self.grid.resolution))
                self.grid.frontier_visits[max(0, row-radius):row+radius+1,
                                          max(0, column-radius):column+radius+1] += 1
            self.last_replan = -math.inf
            # Observe new directions at an exhausted frontier before choosing another.
            return VelocityCommand(0, self.settings["max_yaw_speed_rad_s"], now, now + ttl)
        command = decision.command
        if command.linear_x_m_s:
            blocked = self.grid.inflated(self.settings["robot_radius_m"],
                                         self.settings["occupied_log_odds_threshold"])
            blocked |= self.grid.log_odds > -0.18
            stopping_distance = command.linear_x_m_s ** 2 / (2 * self.settings.get("deceleration_m_s2", 0.6))
            horizon = max(0.12, stopping_distance + command.linear_x_m_s * 0.3)
            end = (self.pose[0] + horizon * math.cos(self.pose[2]),
                   self.pose[1] + horizon * math.sin(self.pose[2]))
            if self.grid.segment_blocked(self.pose[:2], end, blocked):
                return VelocityCommand(0, command.angular_z_rad_s, now, now + ttl)
        return command
