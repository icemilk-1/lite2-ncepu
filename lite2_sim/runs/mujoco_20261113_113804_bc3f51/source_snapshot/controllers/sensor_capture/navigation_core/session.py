"""Simulator-independent mapping, exploration and navigation lifecycle."""

import math

import numpy as np

from navigation_core.contracts import VelocityCommand
from navigation_core.control import NavigationController
from navigation_core.grid import OccupancyGrid, wrap_angle
from navigation_core.planner import PlanResult


TERMINAL_STATES = frozenset((
    "goal_reached", "timeout", "stalled", "cancelled", "start_blocked",
    "goal_blocked", "map_disconnected",
    "start_out_of_map", "goal_out_of_map", "localization_lost", "collision",
    "map_complete"))

# A blocked or unobserved start/goal is usually a single noisy or stale scan.
# These states are recoverable: the session re-observes, backs off and replans a
# bounded number of times before it accepts a terminal failure.
RECOVERABLE_STATES = frozenset((
    "start_blocked", "goal_blocked", "map_disconnected", "insufficient_observations",
    "stalled"))

RECOVERED_STATES = frozenset(("navigating", "exploring", "frontier_reached", "goal_reached"))


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
        self.grid.frontier_visits_threshold = 0.0
        self.pose = list(initial_pose)
        self.odom_pose = list(initial_pose)
        self.exploration_mode = bool(self.settings.get("exploration_mode", False))
        self.goal = None if self.exploration_mode else tuple(self.settings["goal_xy_m"])
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
        # The executor's envelope is a 0.60 m long x 0.44 m wide box, so the body
        # is longer than it is wide; its corner radius is 0.372 m.
        xs = np.linspace(-0.32, 0.32, 15)
        ys = np.linspace(-0.24, 0.24, 11)
        xx, yy = np.meshgrid(xs, ys)
        self.footprint_points = np.column_stack((xx.ravel(), yy.ravel()))
        self.body_points = self.footprint_points * np.array([0.30/0.32, 0.22/0.24])
        self.body_corner_radius_m = math.hypot(0.30, 0.22)
        self.blocked_frontier_since = None
        self.motion = None
        self.mission_progress_pose = list(initial_pose[:2])
        self.mission_progress_time = None
        self.recovery = None
        self.recovery_events = []
        self.recoveries = 0
        self.recovery_exhausted = False
        self.frontiers_visited = 0
        self.distance_m = 0.0
        self.last_tick_pose = list(initial_pose[:2])
        self.exploration_idle_since = None
        # Narrow-corridor mode is switched by _update_corridor_mode(); keep the
        # configured open-space speed so it can be restored when leaving a pipe.
        self.base_max_linear_speed_m_s = float(self.settings["max_linear_speed_m_s"])
        self.narrow_corridor = False

    @property
    def finished(self):
        # A blocked start/goal is only final once the bounded recovery budget is
        # spent; until then the session keeps trying to re-observe the world.
        return self.status in TERMINAL_STATES or self.recovery_exhausted

    def cancel(self):
        self.navigator.cancel()

    def set_goal(self, goal_xy, now):
        """Start another leg while retaining accumulated observations and pose."""
        if self.navigator.cancelled:
            raise ValueError("A cancelled navigation session cannot accept another goal.")
        self.goal = None if self.exploration_mode else tuple(goal_xy)
        self.settings["goal_xy_m"] = list(goal_xy)
        self.navigator = NavigationController(self.settings)
        self.plan = PlanResult("insufficient_observations", goal_xy=self.goal)
        self.status = "mapping"
        self.started_at = now
        self.last_replan = -math.inf
        self.stopped_since = None
        self.blocked_frontier_since = None
        self.recovery = None
        self.exploration_idle_since = None
        self.mission_progress_pose = self.pose[:2]
        self.mission_progress_time = now

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
        self._mark_pose_cell_free()

    def _mark_pose_cell_free(self):
        """The robot physically occupies its estimated cell, so it is traversable.

        This keeps a single noisy return under the body from turning the robot's
        own start cell into an unknown or blocked cell.
        """
        cells = []
        radius = max(1, int(self.settings["robot_radius_m"] * 0.7 / self.grid.resolution))
        for pose in (self.pose, self.odom_pose):
            centre = self.grid.world_to_cell(*pose[:2])
            if centre is None:
                continue
            column, row = centre
            rr0, rr1 = max(0, row - radius), min(self.grid.height, row + radius + 1)
            cc0, cc1 = max(0, column - radius), min(self.grid.width, column + radius + 1)
            cells.append((slice(rr0, rr1), slice(cc0, cc1)))
        for rows, columns in cells:
            self.grid.log_odds[rows, columns] = np.minimum(self.grid.log_odds[rows, columns], -0.7)
            if hasattr(self.grid, "terrain_hazard"):
                self.grid.terrain_hazard[rows, columns] = False

    def _guided_corridor_plan(self):
        """Follow the tunnel axis instead of a diagonal pose-to-goal line.

        A 1 m pipe is only 0.56 m wider than the body; a straight line from an
        off-axis pose to the exit cuts the entrance diagonally. Planning an
        axis-aligned entry segment keeps the cross-track controller centred.
        """
        goal = self.goal
        axis_y = float(self.settings.get("corridor_axis_y_m", goal[1]))
        path = [tuple(self.pose[:2])]
        if abs(goal[1] - axis_y) <= 0.02 and abs(self.pose[1] - axis_y) > 0.02:
            path.append((self.pose[0], axis_y))
        if abs(goal[0] - path[-1][0]) > 0.02:
            path.append((goal[0], axis_y))
        if path[-1] != (goal[0], goal[1]):
            path.append(tuple(goal))
        return PlanResult("path_found", path, goal, goal,
                          reason="guided_corridor_axis_tracking")

    def _update_corridor_mode(self):
        """Limit speed and enable lateral centring inside a narrow corridor."""
        narrow = bool(self.settings.get("guided_corridor"))
        threshold = float(self.settings.get("narrow_corridor_clearance_m", 0.0))
        if not narrow and threshold > 0.0 and hasattr(self.grid, "clearance"):
            cell = self.grid.world_to_cell(*self.pose[:2])
            if cell is not None:
                field = self.grid.clearance(self.settings["occupied_log_odds_threshold"])
                value = float(field[cell[1], cell[0]])
                narrow = math.isfinite(value) and value < threshold
        self.narrow_corridor = narrow
        if narrow:
            limit = float(self.settings.get("narrow_speed_limit_m_s", self.base_max_linear_speed_m_s))
            self.settings["max_linear_speed_m_s"] = min(self.base_max_linear_speed_m_s, limit)
            self.settings["cross_track_gain"] = float(self.settings.get("cross_track_gain_m", 0.0))
            self.settings["alignment_threshold_rad"] = float(
                self.settings.get("narrow_alignment_threshold_rad", 1.15))
        else:
            self.settings["max_linear_speed_m_s"] = self.base_max_linear_speed_m_s
            self.settings["cross_track_gain"] = 0.0
            self.settings["alignment_threshold_rad"] = 1.15

    def _replan(self, now):
        previous = self.plan
        planning_radius = float(self.settings.get("planning_radius_m", self.settings["robot_radius_m"]))
        self.grid.frontier_heading_rad = self.pose[2]
        self.grid.frontier_rearward_penalty = float(self.settings.get("frontier_rearward_penalty_m", 2.0))
        clearance_preference = float(self.settings.get("clearance_preference_m", 0.0))
        if clearance_preference <= 0.0:
            clearance_preference = planning_radius + float(self.settings.get("planning_clearance_m", 0.0))
        self.plan = self.grid.plan_result(
            self.pose, self.goal, planning_radius,
            self.settings["occupied_log_odds_threshold"], self.settings["unknown_cell_cost"],
            self.settings.get("planning_clearance_m", 0.0),
            clearance_preference,
            float(self.settings.get("path_clearance_weight", 0.0)),
            float(self.settings.get("wall_log_odds_threshold", 0.2)))
        # Once a configured tunnel portal has been reached, the tunnel axis is a
        # semantic course constraint.  A 2-D occupancy projection can make the
        # annular entrance look disconnected even though the observed 3-D body
        # envelope is clear.  Follow the axis while the local rollout continues
        # enforcing observed occupancy, terrain and the oriented footprint.
        if (self.settings.get("guided_corridor") and self.goal is not None
                and self.plan.status in ("exploring", "goal_blocked", "map_disconnected")):
            self.plan = self._guided_corridor_plan()
        if self.oriented_footprint and self.plan.status == "start_blocked" and self._body_clear():
            planning_radius = min(planning_radius, self.settings.get("minimum_planning_radius_m", 0.24))
            self.plan = self.grid.plan_result(self.pose, self.goal, planning_radius,
                self.settings["occupied_log_odds_threshold"], self.settings["unknown_cell_cost"],
                self.settings.get("planning_clearance_m", 0.0))
            self.plan.reason = "oriented_start_clearance_recovery_" + self.plan.reason
        # Keep a reachable exploration subgoal until it is inspected. Re-selecting
        # every scan makes changing visibility reverse the route in narrow corridors.
        if previous.status == "exploring" and self.plan.status == "exploring" and previous.target_xy is not None:
            distance = math.hypot(previous.target_xy[0] - self.pose[0], previous.target_xy[1] - self.pose[1])
            cell = self.grid.world_to_cell(*previous.target_xy)
            unvisited = cell is not None and self.grid.frontier_visits[cell[1], cell[0]] == 0
            if unvisited and distance > self.settings["frontier_tolerance_m"]:
                continuation = self.grid.plan_result(self.pose, previous.target_xy,
                    planning_radius, self.settings["occupied_log_odds_threshold"],
                    self.settings["unknown_cell_cost"], self.settings.get("planning_clearance_m", 0.0))
                if continuation.status == "path_found":
                    self.plan = PlanResult("exploring", continuation.path, self.goal, previous.target_xy,
                                           "continuing_reachable_frontier")
        self.last_replan = now
        self.history.append({"sim_time_s": now, "status": self.plan.status,
                             "reason": self.plan.reason, "from_xy_m": self.pose[:2],
                             "planning_radius_m": planning_radius,
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
        if self.status in TERMINAL_STATES or self.recovery_exhausted:
            return zero
        if self.recovery is not None:
            # A recovery owns the command for its whole rotate -> reverse -> settle
            # window.  Clearing it on the next tick (because the navigator reports
            # "navigating" again) turned every recovery into a single 50 ms pulse
            # and reset the attempt counter, so the robot could stall forever.
            window = (float(self.settings.get("recovery_rotate_s", 1.2))
                      + float(self.settings.get("recovery_reverse_s", 0.8))
                      + float(self.settings.get("recovery_settle_s", 0.5)))
            travelled = math.hypot(self.pose[0] - self.recovery["pose"][0],
                                   self.pose[1] - self.recovery["pose"][1])
            if travelled >= float(self.settings.get("progress_distance_m", 0.06)):
                self._recovery_event(now, "recovery_succeeded", status=self.recovery["reason"],
                                     attempt=self.recovery["attempts"])
                self.recovery = None
            else:
                return self._recover(now, zero)
        moved = math.hypot(self.pose[0] - self.last_tick_pose[0], self.pose[1] - self.last_tick_pose[1])
        if math.isfinite(moved) and moved < 1.0:
            self.distance_m += moved
        self.last_tick_pose = list(self.pose[:2])
        if math.hypot(self.pose[0]-self.mission_progress_pose[0], self.pose[1]-self.mission_progress_pose[1]) >= self.settings.get("progress_distance_m", 0.06):
            self.mission_progress_pose = self.pose[:2]
            self.mission_progress_time = now
        elif now - self.mission_progress_time > self.settings["stuck_timeout_s"] + self.settings["initial_mapping_duration_s"]:
            self.status = "stalled"
            # A stall is usually a bad local minimum (a corner, a tight entry),
            # not a reason to end the mission: back off and replan first.
            return self._recover(now, zero)
        if elapsed < self.settings["initial_mapping_duration_s"]:
            return zero
        if self.grid.scan_count < 3:
            if elapsed > 5:
                self.status = "insufficient_observations"
            return zero
        if now - self.last_replan >= self.settings["replan_period_s"]:
            self._replan(now)
        self._update_corridor_mode()
        decision = self.navigator.update(now, self.pose, self.plan, self.last_observation, elapsed)
        self.status = decision.status
        if self.recovery is not None and self.status not in RECOVERABLE_STATES:
            self._recovery_event(now, "recovery_succeeded", status=self.recovery["reason"],
                                 attempt=self.recovery["attempts"])
            self.recovery = None
        # "No frontier left" is the successful end of pure exploration, not a
        # blocked start, so it must not enter the recovery loop.
        exhausted = self.exploration_mode and self.plan.reason == "no_reachable_known_free_frontier"
        if exhausted and self.recovery is not None:
            self._recovery_event(now, "recovery_succeeded", status=self.recovery["reason"],
                                 attempt=self.recovery["attempts"])
            self.recovery = None
        if self.status in RECOVERABLE_STATES and not exhausted:
            return self._recover(now, zero)
        if self.exploration_mode:
            self._update_exploration_state(now)
            if self.status == "map_complete":
                return zero
        if self.status == "frontier_reached":
            self._mark_frontier_visited(self.plan.target_xy, 0.4, 1)
            self.frontiers_visited += 1
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

    def _recovery_event(self, now, event, **details):
        record = {"timestamp_s": now, "event": event}
        record.update(details)
        record["estimated_pose_m_rad"] = list(self.pose)
        self.recovery_events.append(record)

    def _mark_frontier_visited(self, target_xy, radius_m, weight):
        if target_xy is None:
            return
        cell = self.grid.world_to_cell(*target_xy)
        if cell is None:
            return
        column, row = cell
        radius = max(1, int(radius_m / self.grid.resolution))
        self.grid.frontier_visits[max(0, row - radius):row + radius + 1,
                                  max(0, column - radius):column + radius + 1] += weight

    def _recover(self, now, zero):
        """Bounded recovery for a blocked or unobserved start/goal.

        One noisy scan must not end a mission: rotate in place to re-observe,
        then back off a little and replan. A run of failed attempts (or a single
        attempt window that outlives the limit) becomes a terminal failure, so a
        genuinely unreachable goal is still reported instead of retried forever.
        """
        reason = self.plan.status if self.plan.status in RECOVERABLE_STATES else self.status
        rotate_s = float(self.settings.get("recovery_rotate_s", 1.2))
        reverse_s = float(self.settings.get("recovery_reverse_s", 0.8))
        settle_s = float(self.settings.get("recovery_settle_s", 0.5))
        limit = int(self.settings.get("recovery_attempt_limit", 3))
        ttl = self.settings["command_timeout_s"]
        self.mission_progress_time = now
        self.mission_progress_pose = self.pose[:2]
        if self.recovery is None or self.recovery["reason"] != reason:
            attempts = 1 if self.recovery is None else self.recovery["attempts"] + 1
            self.recovery = {"reason": reason, "since": now, "attempts": attempts,
                             "pose": list(self.pose[:2])}
            self.recoveries += 1
            self._recovery_event(now, "recovery_started", status=reason, attempt=attempts)
            self.last_replan = -math.inf
        recovery = self.recovery
        elapsed = now - recovery["since"]
        self.status = "recovering"
        # The failed frontier must not be selected again while other options exist.
        self._mark_frontier_visited(self.plan.target_xy, 0.35, 2)
        # Wedged inside the map (the body footprint already overlaps occupied
        # cells) is a geometry problem, not a heading problem: drive out along the
        # reverse of the heading the robot came in with (hybrid A* reverse
        # primitive) before turning.  A pivot would swing the body into the wall it
        # is already touching, which is exactly the ramp-inner-edge collision.
        boxed = False
        if not self._body_clear():
            # Standing ON an occupied cell (start_blocked) is fixed by re-observing,
            # so only a body that is clear at its own cell but wedged against an
            # adjacent wall counts as boxed.
            cell = self.grid.world_to_cell(*self.pose[:2])
            boxed = bool(cell is not None and self.grid.log_odds[cell[1], cell[0]] <
                         self.settings["occupied_log_odds_threshold"])
        if boxed:
            reverse_s = max(reverse_s, float(self.settings.get("recovery_boxed_reverse_s", 3.0)))
        window = rotate_s + reverse_s + settle_s
        if elapsed >= window:
            if self.exploration_mode:
                # Mapping never gives up on a blocked start; it keeps exploring
                # until the mission timeout decides otherwise.
                self.recovery = {"reason": reason, "since": now, "attempts": recovery["attempts"],
                                 "pose": list(self.pose[:2])}
                self.last_replan = -math.inf
                self.navigator._last_progress_time = now
                return zero
            if recovery["attempts"] >= limit:
                self.recovery = None
                self.navigator._last_progress_time = now
                self.status = reason
                self.recovery_exhausted = True
                self._recovery_event(now, "recovery_exhausted", status=reason,
                                     attempt=recovery["attempts"])
                return zero
            recovery["attempts"] += 1
            recovery["since"] = now
            recovery["pose"] = list(self.pose[:2])
            self._recovery_event(now, "recovery_retry", status=reason, attempt=recovery["attempts"])
            self.last_replan = -math.inf
            return zero
        self.navigator._last_progress_time = now
        if boxed and not self.settings.get("reverse_only_disabled", False):
            speed = min(0.6 * self.settings["max_linear_speed_m_s"], 0.08)
            return VelocityCommand(-speed, 0, now, now + ttl)
        if elapsed < rotate_s:
            # Rotate towards the planned route instead of a fixed left turn: after
            # a hairpin the route is behind the robot, so the escape rotation must
            # follow its sign or the retries cancel each other out.
            direction = 1.0
            waypoint = self.plan.target_xy if self.plan.target_xy is not None else (
                self.plan.path[-1] if self.plan.path else None)
            if waypoint is not None:
                bearing = math.atan2(waypoint[1] - self.pose[1], waypoint[0] - self.pose[0])
                direction = 1.0 if wrap_angle(bearing - self.pose[2]) >= 0.0 else -1.0
            # A pivot swings the corners out to the 0.372 m body radius.  Next to
            # a wall-like cell (log-odds ~0.35, never occupied) the recovery's own
            # rotation used to clip the ramp inner edge; back out of the pinch
            # first instead of rotating into it (hybrid A* reverse-then-turn).
            # Clearance below the body half-width means the robot stands ON the
            # blocked cell (start-blocked recovery), where re-observing by
            # rotation is the intended action and reversing is not safer.
            gap = math.inf
            if hasattr(self.grid, "clearance"):
                cell = self.grid.world_to_cell(*self.pose[:2])
                if cell is not None:
                    gap = float(self.grid.clearance(
                        float(self.settings.get("wall_log_odds_threshold", 0.2)))[cell[1], cell[0]])
            body_half_width = float(self.body_points[:, 1].max())
            if math.isfinite(gap) and body_half_width <= gap < self.body_corner_radius_m:
                self.last_replan = -math.inf
                speed = min(0.6 * self.settings["max_linear_speed_m_s"], 0.08)
                return VelocityCommand(-speed, 0, now, now + ttl)
            return VelocityCommand(0, direction * self.settings["max_yaw_speed_rad_s"] * 0.6,
                                   now, now + ttl)
        if elapsed < rotate_s + reverse_s and not self.settings.get("reverse_only_disabled", False):
            speed = min(0.6 * self.settings["max_linear_speed_m_s"], 0.08)
            return VelocityCommand(-speed, 0, now, now + ttl)
        self.last_replan = -math.inf
        return zero

    def _update_exploration_state(self, now):
        """Finish pure exploration once no reachable frontier remains."""
        settle_s = float(self.settings.get("exploration_settle_s", 6.0))
        if self.plan.status == "exploring":
            self.exploration_idle_since = None
            return
        if self.plan.reason != "no_reachable_known_free_frontier":
            self.exploration_idle_since = None
            return
        if self.exploration_idle_since is None:
            self.exploration_idle_since = now
        elif now - self.exploration_idle_since >= settle_s:
            self.status = "map_complete"

    def exploration_summary(self):
        """Mapping statistics for a pure autonomous exploration run."""
        values = self.grid.log_odds
        occupied = values >= self.settings["occupied_log_odds_threshold"]
        known_free = values <= -0.18
        return {
            "known_free_cells": int(known_free.sum()),
            "occupied_cells": int(occupied.sum()),
            "unknown_cells": int((~known_free & ~occupied).sum()),
            "frontiers_visited": int(self.frontiers_visited),
            "distance_m": round(float(self.distance_m), 3),
            "scan_count": int(getattr(self.grid, "scan_count", 0)),
            "recoveries": int(self.recoveries),
        }

    def _body_clear(self):
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        x = self.pose[0] + c*self.body_points[:, 0] - s*self.body_points[:, 1]
        y = self.pose[1] + s*self.body_points[:, 0] + c*self.body_points[:, 1]
        col = np.floor((x-self.grid.x_min)/self.grid.resolution).astype(int)
        row = np.floor((y-self.grid.y_min)/self.grid.resolution).astype(int)
        inside = (col >= 0) & (col < self.grid.width) & (row >= 0) & (row < self.grid.height)
        if not inside.all():
            return False
        values = self.grid.log_odds[row, col]
        return not (values >= self.settings["occupied_log_odds_threshold"]).any()

    def _local_rollout(self, preferred, now):
        # Score against a longer look-ahead than the controller uses: the short
        # 0.25 m target sits at the near corner of a ramp/pipe mouth and aiming at
        # it hugs the inner edge.  The tie-break below uses the heading of THIS
        # target, so the two layers can never disagree about the turn direction
        # (mixing a 0.25 m and a 0.45 m bearing across +/-pi cancelled the
        # dead-end exit into a permanent standstill).
        lookahead = float(self.settings.get("rollout_lookahead_m", 0.45))
        target = self.navigator._lookahead_target(self.pose, self.plan.path, lookahead)
        if self.settings.get("cross_track_gain"):
            target = self.navigator._cross_track_target(
                self.pose, self.plan.path, target,
                float(self.settings["cross_track_gain"]),
                bool(self.settings.get("reverse_only", False)))
        maximum = self.settings["max_linear_speed_m_s"]
        angular_max = self.settings["max_yaw_speed_rad_s"]
        desired_heading = wrap_angle(
            math.atan2(target[1] - self.pose[1], target[0] - self.pose[0]) - self.pose[2])
        # Tie-break towards the same turn the scoring target asks for, not towards
        # the controller's short-look-ahead command.
        preferred_angular = max(-angular_max, min(angular_max, 2.0 * desired_heading))
        velocities = sorted(set((-0.08, -0.03, 0.0, 0.03, 0.08, maximum, preferred.linear_x_m_s)))
        if self.settings.get("reverse_only"):
            velocities = [-0.08, -0.03, 0.0]
        elif self.settings.get("forward_only"):
            velocities = [velocity for velocity in velocities if velocity >= 0.0]
        elif self.motion is not None and abs(self.motion.ground_height_m) > self.settings.get("max_step_m", 0.08):
            velocities = [velocity for velocity in velocities if velocity >= 0.0]
        turns = sorted(set((-angular_max, -angular_max/2, 0., angular_max/2, angular_max, preferred.angular_z_rad_s)))
        if self.settings.get("guided_corridor"):
            correction = max(-angular_max / 2, min(angular_max / 2,
                                                    preferred.angular_z_rad_s))
            turns = sorted(set((0.0, correction)))
        occupied = self.grid.log_odds >= self.settings["occupied_log_odds_threshold"]
        if hasattr(self.grid, "traversable_step_mask"):
            occupied &= ~self.grid.traversable_step_mask()
        support_hazard = getattr(self.grid, "terrain_hazard", np.zeros_like(occupied))
        known_free = self.grid.log_odds <= -0.18
        center_blocked = occupied | support_hazard | ~known_free
        # Wall-like cells (terrain height jumps) sit below the occupied threshold
        # at log-odds ~0.35, so they never stop the rollout by themselves.  A
        # pivot sweeps the 0.372 m body corners sideways, so in-place rotation is
        # only admissible where the observed clearance covers the corner radius.
        wall_clearance = None
        if hasattr(self.grid, "clearance"):
            wall_clearance = self.grid.clearance(
                float(self.settings.get("wall_log_odds_threshold", 0.2)))
        if self.grid.segment_blocked(self.pose[:2], target, center_blocked):
            # Keep local targets on the visible side of a corner or terrain edge.
            # A long lookahead may span several short A* correction segments.
            for waypoint in self.plan.path[1:]:
                distance = math.hypot(waypoint[0]-self.pose[0], waypoint[1]-self.pose[1])
                if 0.025 < distance <= 0.50 and not self.grid.segment_blocked(self.pose[:2], waypoint, center_blocked):
                    target = waypoint
        best = None
        best_forward = None
        samples = []
        rotations = []
        standstill_escapes = 0
        body_points = self.body_points
        local_half_width = self.settings.get("local_body_half_width_m")
        if local_half_width is not None:
            body_points = body_points[np.abs(body_points[:, 1]) <= float(local_half_width) + 1e-9]
        rejected = {"terrain": 0, "support": 0, "map_edge": 0,
                    "occupied": 0, "unknown": 0, "turn_unknown": 0,
                    "rotation_clearance": 0}
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
                        rejected["terrain"] += 1
                        clear = False
                        break
                    if not self._support_continuous(pose, center):
                        rejected["support"] += 1
                        clear = False
                        break
                    if (wall_clearance is not None and abs(linear) < 0.02
                            and abs(angular) > 0.02):
                        gap = float(wall_clearance[center[1], center[0]])
                        if math.isfinite(gap) and gap < self.body_corner_radius_m:
                            rejected["rotation_clearance"] += 1
                            clear = False
                            break
                    c, s = math.cos(pose[2]), math.sin(pose[2])
                    x = pose[0] + c*body_points[:, 0] - s*body_points[:, 1]
                    y = pose[1] + s*body_points[:, 0] + c*body_points[:, 1]
                    col = np.floor((x-self.grid.x_min)/self.grid.resolution).astype(int)
                    row = np.floor((y-self.grid.y_min)/self.grid.resolution).astype(int)
                    inside = (col >= 0) & (col < self.grid.width) & (row >= 0) & (row < self.grid.height)
                    if not inside.all():
                        rejected["map_edge"] += 1
                        clear = False
                        break
                    unobserved = ~known_free[row, col]
                    hazard_fraction = float(np.mean(support_hazard[row, col]))
                    # The lidar cannot observe the cells directly below the torso.
                    # Keep occupied and terrain-hazard cells as hard stops, while
                    # allowing a bounded unknown patch inside an otherwise observed
                    # footprint so narrow corridors remain traversable.
                    max_unknown = float(self.settings.get("local_unknown_fraction_limit", 0.35))
                    if occupied[row, col].any():
                        rejected["occupied"] += 1
                        clear = False
                        break
                    if float(np.mean(unobserved)) > max_unknown:
                        rejected["unknown"] += 1
                        clear = False
                        break
                    # Turning in place sweeps the body corners sideways; in a
                    # corridor that is only as wide as the inflated planning
                    # band, any unobserved cell under the body makes the turn a
                    # body-clearance violation for the executor.
                    if (abs(linear) < 0.02 and abs(angular) > 0.02
                            and float(np.mean(unobserved)) > max_unknown):
                        rejected["turn_unknown"] += 1
                        clear = False
                        break
                    x = pose[0] + c*self.footprint_points[:, 0] - s*self.footprint_points[:, 1]
                    y = pose[1] + s*self.footprint_points[:, 0] + c*self.footprint_points[:, 1]
                    guard_col = np.clip(np.floor((x-self.grid.x_min)/self.grid.resolution).astype(int), 0, self.grid.width-1)
                    guard_row = np.clip(np.floor((y-self.grid.y_min)/self.grid.resolution).astype(int), 0, self.grid.height-1)
                    margin_cost += float(np.mean(occupied[guard_row, guard_col]))
                    margin_cost += 2.0 * hazard_fraction
                if not clear:
                    continue
                travel_heading = pose[2] + (math.pi if linear < 0 else 0.0)
                heading = wrap_angle(math.atan2(target[1]-pose[1], target[0]-pose[0])-travel_heading)
                score = -2.0*math.hypot(target[0]-pose[0], target[1]-pose[1]) - 0.25*abs(heading)
                score += 0.25*linear - 0.03*abs(angular-preferred_angular)
                score -= 2.0*margin_cost
                if best is None or score > best[0]:
                    best = (score, linear, angular)
                if linear > 0.02 and (best_forward is None or score > best_forward[0]):
                    best_forward = (score, linear, angular)
                samples.append((score, linear, angular))
                if abs(linear) < 0.02 and abs(angular) > 0.02:
                    rotations.append((abs(heading), score, angular))
        selected = best
        # Clearance is a soft preference after every footprint/terrain check has
        # passed.  It must not make stopping forever cheaper than progressing
        # along an aligned path in a narrow corridor.
        desired_heading = wrap_angle(
            math.atan2(target[1] - self.pose[1], target[0] - self.pose[0]) - self.pose[2])
        if (selected is not None and abs(selected[1]) < 0.02 and best_forward is not None
                and abs(desired_heading) < 0.75):
            selected = best_forward
        # Local minima after a hairpin: the standstill candidate keeps the best
        # score because any translation increases the distance to a target that
        # lies behind, and any rotation sweeps the body corners past nearby
        # obstacle cells (margin cost).  Rotating in place costs no support
        # change, so it is an explicit primitive (DWA rotate-in-place, hybrid A*
        # in-place primitives): whenever the chosen motion is barely creeping and
        # the path leaves in a very different direction, turn towards it.
        standstill_threshold = float(self.settings.get("standstill_heading_threshold_rad", 0.6))
        creep_limit = float(self.settings.get("standstill_creep_m_s", 0.02))
        if (selected is not None and abs(selected[1]) < creep_limit and rotations
                and abs(desired_heading) > standstill_threshold):
            best_rotation = min(rotations, key=lambda item: item[0])
            if best_rotation[0] < abs(desired_heading) - 0.02:
                selected = (best_rotation[1], 0.0, best_rotation[2])
                standstill_escapes = 1
        linear, angular = (selected[1], selected[2]) if selected is not None else (0., 0.)
        self.local_rollout_debug = {"candidate_count": len(velocities) * len(turns),
                                    "valid_count": 0 if best is None else 1,
                                    "rejected": rejected,
                                    "target_xy_m": [float(target[0]), float(target[1])],
                                    "preferred": [float(preferred.linear_x_m_s),
                                                  float(preferred.angular_z_rad_s)],
                                    "reverse_only": bool(self.settings.get("reverse_only", False)),
                                    "forward_only": bool(self.settings.get("forward_only", False)),
                                    "velocities": [float(value) for value in velocities],
                                    "turns": [float(value) for value in turns],
                                    "desired_heading_rad": float(desired_heading),
                                    "standstill_escape": int(standstill_escapes),
                                    "best": None if best is None else [float(best[0]), float(best[1]), float(best[2])],
                                    "top_scores": [[round(float(s), 3), round(float(v), 3), round(float(w), 3)]
                                                   for s, v, w in sorted(samples, reverse=True)[:6]],
                                    "scores": [[round(float(s), 3), round(float(v), 3), round(float(w), 3)]
                                               for s, v, w in samples]}
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

    def _support_continuous(self, pose, cell):
        if self.motion is None or abs(self.motion.ground_height_m) <= self.settings.get("max_step_m", 0.08):
            return True
        if not hasattr(self.grid, "height_m"):
            return True
        col, row = cell
        height = self.grid.height_m[row, col]
        if not math.isfinite(float(height)):
            neighborhood = self.grid.height_m[max(0,row-2):row+3, max(0,col-2):col+3]
            finite = neighborhood[np.isfinite(neighborhood)]
            if not len(finite):
                return False
            height = float(np.median(finite))
        distance = math.hypot(pose[0]-self.pose[0], pose[1]-self.pose[1])
        allowed = self.settings["max_step_m"] + distance*math.tan(math.radians(self.settings["max_slope_deg"]))
        return abs(float(height)-self.motion.ground_height_m) <= allowed
