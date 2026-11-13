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
        self.grid.scan_match_prior_weight = self.settings.get("scan_match_prior_weight", 0.08)
        self.grid.scan_match_yaw_prior_weight = self.settings.get("scan_match_yaw_prior_weight", self.grid.scan_match_prior_weight)
        self.grid.scan_match_xy_step_m = self.settings.get("scan_match_xy_step_m", self.grid.resolution * 0.5)
        self.grid.scan_match_yaw_step_rad = self.settings.get("scan_match_yaw_step_rad", 0.04)
        self.grid.scan_match_observability_prior = self.settings.get("scan_match_observability_prior", False)
        self.grid.frontier_radius_m = self.settings["robot_radius_m"] + 0.15
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
        self.oriented_footprint = self.settings.get("oriented_footprint", False)
        xs = np.linspace(-0.32, 0.32, 15)
        ys = np.linspace(-0.24, 0.24, 11)
        xx, yy = np.meshgrid(xs, ys)
        self.footprint_points = np.column_stack((xx.ravel(), yy.ravel()))
        self.body_points = self.footprint_points * np.array([0.30/0.32, 0.22/0.24])
        self.blocked_frontier_since = None
        self.motion = None
        self.mission_progress_pose = list(initial_pose[:2])
        self.mission_progress_time = None

    @property
    def finished(self):
        return self.status in TERMINAL_STATES

    def cancel(self):
        self.navigator.cancel()

    def propagate(self, motion, dt):
        if not motion.valid or dt <= 0 or not math.isfinite(dt):
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
        if hasattr(self.grid, "sensor_height_m"):
            self.grid.sensor_height_m = observation.metadata.get("sensor_height_m", self.grid.sensor_height_m)
        self.grid.integrate(observation.points, self.pose, extrinsic,
                            self.lidar["max_range_m"], self.lidar["min_range_m"],
                            self.settings.get("scan_stride", 1))
        if hasattr(self.grid, "integrate_terrain"):
            self.grid.integrate_terrain(observation, self.pose, extrinsic)

    def _replan(self, now):
        previous = self.plan
        self.plan = self.grid.plan_result(
            self.pose, self.goal, self.settings["robot_radius_m"],
            self.settings["occupied_log_odds_threshold"], self.settings["unknown_cell_cost"],
            self.settings.get("planning_clearance_m", 0.0))
        # Keep a reachable exploration subgoal until it is inspected. Re-selecting
        # every scan makes changing visibility reverse the route in narrow corridors.
        if previous.status == "exploring" and self.plan.status == "exploring" and previous.target_xy is not None:
            distance = math.hypot(previous.target_xy[0] - self.pose[0], previous.target_xy[1] - self.pose[1])
            cell = self.grid.world_to_cell(*previous.target_xy)
            unvisited = cell is not None and self.grid.frontier_visits[cell[1], cell[0]] == 0
            if unvisited and distance > self.settings["frontier_tolerance_m"]:
                continuation = self.grid.plan_result(self.pose, previous.target_xy,
                    self.settings["robot_radius_m"], self.settings["occupied_log_odds_threshold"],
                    self.settings["unknown_cell_cost"], self.settings.get("planning_clearance_m", 0.0))
                if continuation.status == "path_found":
                    self.plan = PlanResult("exploring", continuation.path, self.goal, previous.target_xy,
                                           "continuing_reachable_frontier")
        self.last_replan = now
        self.history.append({"sim_time_s": now, "status": self.plan.status,
                             "reason": self.plan.reason, "from_xy_m": self.pose[:2],
                             "goal_xy_m": self.goal, "target_xy_m": self.plan.target_xy,
                             "poses_xy_m": self.plan.path})

    def tick(self, now, motion=None, cancelled=False):
        self.motion = motion
        if self.started_at is None:
            self.started_at = now
            self.mission_progress_time = now
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
        last_motion_time = self.started_at if self.last_valid_motion is None else self.last_valid_motion
        if (not all(math.isfinite(value) for value in self.pose)
                or (motion is not None and (not motion.valid or motion.timestamp_s > now + 1e-6))
                or now - last_motion_time > self.settings["observation_timeout_s"]):
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
        if math.hypot(self.pose[0]-self.mission_progress_pose[0], self.pose[1]-self.mission_progress_pose[1]) >= self.settings.get("progress_distance_m", 0.06):
            self.mission_progress_pose = self.pose[:2]
            self.mission_progress_time = now
        elif now - self.mission_progress_time > self.settings["stuck_timeout_s"] + self.settings["initial_mapping_duration_s"]:
            self.status = "stalled"
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
        if self.oriented_footprint and self.status in ("navigating", "exploring"):
            return self._local_rollout(command, now)
        if command.linear_x_m_s:
            blocked = self.grid.inflated(self.settings["robot_radius_m"],
                                         self.settings["occupied_log_odds_threshold"])
            blocked |= self.grid.log_odds > -0.18
            stopping_distance = command.linear_x_m_s ** 2 / (2 * self.settings.get("deceleration_m_s2", 0.6))
            horizon = max(0.12, stopping_distance + command.linear_x_m_s * 0.3)
            end = (self.pose[0] + horizon * math.cos(self.pose[2]),
                   self.pose[1] + horizon * math.sin(self.pose[2]))
            if self.grid.segment_blocked(self.pose[:2], end, blocked):
                # Follow the first safe short segment when a long lookahead cuts
                # a tight corner. Small steps retain the observed clearance.
                for waypoint in self.plan.path[1:]:
                    distance = math.hypot(waypoint[0] - self.pose[0], waypoint[1] - self.pose[1])
                    if distance < 0.025 or self.grid.segment_blocked(self.pose[:2], waypoint, blocked):
                        continue
                    error = wrap_angle(math.atan2(waypoint[1] - self.pose[1], waypoint[0] - self.pose[0]) - self.pose[2])
                    angular = max(-self.settings["max_yaw_speed_rad_s"], min(self.settings["max_yaw_speed_rad_s"], 2*error))
                    linear = min(0.03, distance) if abs(error) < 0.20 else 0.0
                    return VelocityCommand(linear, angular, now, now + ttl)
                return VelocityCommand(0, command.angular_z_rad_s, now, now + ttl)
        return command

    def _local_rollout(self, preferred, now):
        target = self.navigator._lookahead_target(self.pose, self.plan.path, 0.45)
        maximum = self.settings["max_linear_speed_m_s"]
        angular_max = self.settings["max_yaw_speed_rad_s"]
        velocities = sorted(set((-0.08, -0.03, 0.0, 0.03, 0.08, maximum, preferred.linear_x_m_s)))
        turns = sorted(set((-angular_max, -angular_max/2, 0., angular_max/2, angular_max, preferred.angular_z_rad_s)))
        occupied = self.grid.log_odds >= self.settings["occupied_log_odds_threshold"]
        support_hazard = getattr(self.grid, "terrain_hazard", np.zeros_like(occupied))
        known_free = self.grid.log_odds <= -0.18
        best = None
        for linear in velocities:
            for angular in turns:
                pose = list(self.pose)
                executed_v = self.motion.linear_x_m_s if self.motion is not None else 0.0
                executed_w = self.motion.angular_z_rad_s if self.motion is not None else 0.0
                clear = True
                margin_cost = 0.0
                for _ in range(5):
                    acceleration = self.settings.get("acceleration_m_s2", 0.35) if abs(linear) > abs(executed_v) else self.settings.get("deceleration_m_s2", 0.6)
                    executed_v += max(-acceleration*0.1, min(acceleration*0.1, linear-executed_v))
                    angular_step = self.settings.get("angular_acceleration_rad_s2", 1.2)*0.1
                    executed_w += max(-angular_step, min(angular_step, angular-executed_w))
                    mid = pose[2] + executed_w*0.05
                    pose[0] += executed_v*math.cos(mid)*0.1
                    pose[1] += executed_v*math.sin(mid)*0.1
                    pose[2] = wrap_angle(pose[2] + executed_w*0.1)
                    center = self.grid.world_to_cell(*pose[:2])
                    if center is None or support_hazard[center[1], center[0]]:
                        clear = False
                        break
                    c, s = math.cos(pose[2]), math.sin(pose[2])
                    x = pose[0] + c*self.body_points[:, 0] - s*self.body_points[:, 1]
                    y = pose[1] + s*self.body_points[:, 0] + c*self.body_points[:, 1]
                    col = np.floor((x-self.grid.x_min)/self.grid.resolution).astype(int)
                    row = np.floor((y-self.grid.y_min)/self.grid.resolution).astype(int)
                    inside = (col >= 0) & (col < self.grid.width) & (row >= 0) & (row < self.grid.height)
                    if not inside.all() or occupied[row, col].any() or float(np.mean(~known_free[row, col])) > 0.15:
                        clear = False
                        break
                    x = pose[0] + c*self.footprint_points[:, 0] - s*self.footprint_points[:, 1]
                    y = pose[1] + s*self.footprint_points[:, 0] + c*self.footprint_points[:, 1]
                    guard_col = np.clip(np.floor((x-self.grid.x_min)/self.grid.resolution).astype(int), 0, self.grid.width-1)
                    guard_row = np.clip(np.floor((y-self.grid.y_min)/self.grid.resolution).astype(int), 0, self.grid.height-1)
                    margin_cost += float(np.mean(occupied[guard_row, guard_col]))
                if not clear:
                    continue
                heading = wrap_angle(math.atan2(target[1]-pose[1], target[0]-pose[0])-pose[2])
                score = -2.0*math.hypot(target[0]-pose[0], target[1]-pose[1]) - 0.25*abs(heading)
                score += 0.25*linear - 0.03*abs(angular-preferred.angular_z_rad_s)
                score -= 2.0*margin_cost
                if best is None or score > best[0]:
                    best = (score, linear, angular)
        linear, angular = (best[1], best[2]) if best is not None else (0., 0.)
        if self.status == "exploring" and abs(linear) < 0.001 and abs(angular) < 0.02:
            if self.blocked_frontier_since is None:
                self.blocked_frontier_since = now
            elif now - self.blocked_frontier_since > 3:
                cell = self.grid.world_to_cell(*self.plan.target_xy) if self.plan.target_xy is not None else None
                if cell is not None:
                    col, row = cell
                    radius = max(1, int(0.35 / self.grid.resolution))
                    self.grid.frontier_visits[max(0,row-radius):row+radius+1, max(0,col-radius):col+radius+1] += 2
                self.last_replan = -math.inf
                self.navigator._last_progress_time = now
                self.blocked_frontier_since = None
        else:
            self.blocked_frontier_since = None
        return VelocityCommand(linear, angular, now, now+self.settings["command_timeout_s"])
