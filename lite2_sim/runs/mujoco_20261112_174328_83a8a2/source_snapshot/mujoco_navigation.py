"""MuJoCo velocity-level navigation, sensor scheduling and run artifacts."""

import csv
import gzip
import json
import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np

from navigation_core.artifacts import save_map
from navigation_core.contracts import MotionObservation, VelocityCommand
from navigation_core.evaluation import NavigationEvaluator
from navigation_core.course import CourseSession, CourseEvaluator
from navigation_core.grid import wrap_angle
from navigation_core.mujoco_adapter import MujocoLidarObservationAdapter
from navigation_core.session import NavigationSession, TERMINAL_STATES
from navigation_core.terrain import TerrainGrid, project_cloud


GROUPS = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)


def configure_navigation(config, scene_id, goal=None, seed=None):
    """Defaults apply only to navigation; sensor-baseline configs keep their meaning."""
    defaults = dict(goal_xy_m=[2.0, -3.0], grid_resolution_m=0.05,
                    robot_radius_m=0.28, planning_clearance_m=0.16,
                    minimum_planning_radius_m=0.24,
                    occupied_log_odds_threshold=0.7, unknown_cell_cost=2.8,
                    initial_mapping_duration_s=2.0, replan_period_s=0.5,
                    observation_timeout_s=0.36, command_timeout_s=0.30,
                    frontier_tolerance_m=0.20, progress_distance_m=0.06,
                    scan_match_search_xy_m=0.025, scan_match_search_yaw_rad=0.02,
                    scan_match_prior_weight=0.25,
                    scan_match_yaw_prior_weight=0.15,
                    scan_match_xy_step_m=0.01, scan_match_yaw_step_rad=0.005,
                    scan_match_observability_prior=True,
                    max_linear_speed_m_s=0.15, max_yaw_speed_rad_s=0.4,
                    lookahead_m=0.25, front_stop_angle_rad=0.35,
                    front_stop_distance_m=0.38, goal_tolerance_m=0.25,
                    estimated_stop_tolerance_m=0.12, navigation_timeout_s=1200.0,
                    stuck_timeout_s=30.0, odom_linear_noise_fraction=0.02,
                    odom_yaw_noise_rad_s=0.008, random_seed=42,
                    acceleration_m_s2=0.35, deceleration_m_s2=0.6,
                    angular_acceleration_rad_s2=1.2, stop_delay_s=0.05,
                    max_slope_deg=15.0, max_step_m=0.08,
                    body_clearance_height_m=0.60, obstacle_memory_s=5.0,
                    control_period_s=0.05, settling_stop_s=1.0,
                    exploration_mode=False, exploration_settle_s=6.0)
    defaults["oriented_footprint"] = True
    original = config.get("navigation", {})
    # Navigation has its own reviewed defaults, separate from the old Webots tuning.
    defaults["goal_xy_m"] = original.get("goal_xy_m", defaults["goal_xy_m"])
    defaults.update(config.get("mujoco_navigation", {}))
    if goal is not None:
        defaults["goal_xy_m"] = list(goal)
    if seed is not None:
        defaults["random_seed"] = seed
    config["navigation"] = defaults
    config["navigation_execution"] = "mocap_velocity_response_with_observed_navigation"
    if "lidar" not in config:
        reference = json.loads((Path(__file__).parent / "config/ruins_reference.json").read_text(encoding="utf-8"))
        config["lidar"] = reference["lidar"]
    config["lidar"].update({key: defaults[key] for key in
                            ("max_slope_deg", "max_step_m", "body_clearance_height_m")})
    navigation_lidar = config.get("mujoco_navigation_lidar", {
        "number_of_layers": 32, "vertical_field_of_view_rad": math.radians(120)})
    config["lidar"].update(navigation_lidar)
    config["lidar"]["random_seed"] = defaults["random_seed"] + 1000
    config["lidar"]["model_status"] = "ideal_navigation_multilayer_lidar_placeholder_pending_hardware"
    config["lidar"]["coverage_source"] = "navigation_near_ground_visibility_assumption_pending_hardware_validation"
    config["scene_id"] = scene_id
    return config


def build_randomization(config, output=None):
    """Sample the configured randomization profile and apply sensor-side values.

    The audit record is written before navigation starts so a run can be
    reproduced from its seed and profile id alone.
    """
    config["randomization_sampled"] = None
    profile = config.get("randomization_profile")
    if not profile:
        return None
    from navigation_core.randomization import apply_to_lidar, load_profile, record, sample

    seed = int(config["navigation"]["random_seed"])
    profile_data = load_profile(profile, base_dir=Path(__file__).resolve().parent)
    sampled = sample(profile_data, seed)
    values = dict(sampled.get("values") or {})
    lidar_overrides = dict(apply_to_lidar(values, config.get("lidar", {})) or {})
    allowed = {"noise_m", "range_bias_m", "dropout_probability",
               "beam_block_probability", "scan_latency_s"}
    unknown = set(lidar_overrides) - allowed
    if unknown:
        raise ValueError("Randomization returned unsupported LiDAR keys: " + str(sorted(unknown)))
    config.setdefault("lidar", {}).update(lidar_overrides)
    config["randomization_sampled"] = sampled
    if output is not None:
        (Path(output) / "randomization.json").write_text(
            json.dumps(record(sampled), indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    return sampled


class MujocoVelocityExecutor:
    """Private simulation state, geometric contact checks and synthetic motion sensors."""

    def __init__(self, model, data, config):
        self.model, self.data = model, data
        self.settings = config["navigation"]
        self.randomization = dict((config.get("randomization_sampled") or {}).get("values") or {})
        self.base_height = float(config["robot_position_m"][2])
        self.pose = [*config["robot_position_m"][:2], 0.0]
        self.ground = 0.0
        self.roll, self.pitch = 0.0, 0.0
        self.height_odometry = 0.0
        self.velocity = np.zeros(2)
        self.random = np.random.RandomState(self.settings["random_seed"])
        # Command link buffering: a real radio link delivers velocity commands late.
        self.pending_commands = []
        self.delivered_command = None
        self.probe = model.geom("navigation_envelope").id
        self.fixture = model.body("base_fixture").mocapid[0]
        self.root = model.joint("root").qposadr[0]
        self.check_data = mujoco.MjData(model)
        self.environment = np.flatnonzero(model.geom_group == 3)
        self.distance_m = 0.0
        self.collisions = 0
        self.last_moving_command_at = -math.inf
        self.collision_events = []
        self.last_collision_geometry = None

    def initial_estimate_pose(self):
        """Prior pose handed to the estimator; simulation truth is left untouched."""
        values = self.randomization
        dx = float(values.get("initial_pose.dx_m", 0.0) or 0.0)
        dy = float(values.get("initial_pose.dy_m", 0.0) or 0.0)
        dyaw = float(values.get("initial_pose.yaw_offset_rad", 0.0) or 0.0)
        if dx == 0.0 and dy == 0.0 and dyaw == 0.0:
            return list(self.pose)
        return [self.pose[0] + dx, self.pose[1] + dy, wrap_angle(self.pose[2] + dyaw)]

    def support(self, x, y):
        normal = np.zeros(3)
        origin = np.array([x, y, self.ground + self.settings["max_step_m"] + 0.01])
        distance = mujoco.mj_ray(self.model, self.data, origin, np.array([0., 0., -1.]),
                                 GROUPS, True, -1, None, normal)
        if distance < 0:
            return None
        height = origin[2] - distance
        slope = math.degrees(math.acos(np.clip(normal[2], -1, 1)))
        if abs(height - self.ground) > self.settings["max_step_m"] or slope > self.settings["max_slope_deg"]:
            return None
        return height, normal

    def set_pose(self, data, pose, ground, roll, pitch):
        yaw = pose[2]
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        quat = np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                         cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])
        pos = [pose[0], pose[1], ground + self.base_height]
        data.mocap_pos[self.fixture] = pos
        data.mocap_quat[self.fixture] = quat
        data.qpos[self.root:self.root+3] = pos
        data.qpos[self.root+3:self.root+7] = quat
        mujoco.mj_forward(self.model, data)

    def intersects(self, pose, ground, roll, pitch):
        self.last_collision_geometry = None
        self.set_pose(self.check_data, pose, ground, roll, pitch)
        center = self.check_data.geom_xpos[self.probe]
        positions = self.check_data.geom_xpos[self.environment]
        distances = np.linalg.norm(positions - center, axis=1)
        candidates = self.environment[distances < self.model.geom_rbound[self.environment] + 0.5]
        for index in candidates:
            if mujoco.mj_geomDistance(self.model, self.check_data, self.probe, int(index), 0.02, None) < 0.005:
                self.last_collision_geometry = self.model.geom(int(index)).name
                return True
        return False

    def step(self, command, now, dt):
        values = self.randomization
        requested = np.array([command.linear_x_m_s, command.angular_z_rad_s])
        if command.expired(now) or command.created_at_s > now:
            requested[:] = 0
        requested = np.clip(requested, [-self.settings["max_linear_speed_m_s"], -self.settings["max_yaw_speed_rad_s"]],
                            [self.settings["max_linear_speed_m_s"], self.settings["max_yaw_speed_rad_s"]])
        latency = max(0.0, float(values.get("actuation.command_latency_s", 0.0) or 0.0))
        if latency > 1e-9:
            # Radio link model: commands arrive late and the robot holds the newest
            # delivered command, exactly like a UDP velocity bridge on hardware.
            self.pending_commands.append((now + latency, requested.copy()))
            arrived = [entry for entry in self.pending_commands if entry[0] <= now + 1e-9]
            if arrived:
                self.pending_commands = [entry for entry in self.pending_commands if entry[0] > now + 1e-9]
                self.delivered_command = arrived[-1][1]
            requested = (self.delivered_command.copy() if self.delivered_command is not None
                         else np.zeros(2))
        if requested.any():
            self.last_moving_command_at = now
        elif now - self.last_moving_command_at < self.settings["stop_delay_s"]:
            requested = self.velocity.copy()
        acceleration_scale = max(0.05, float(values.get("actuation.acceleration_scale", 1.0) or 1.0))
        acceleration = ((self.settings["acceleration_m_s2"] if abs(requested[0]) > abs(self.velocity[0])
                         else self.settings["deceleration_m_s2"]) * acceleration_scale)
        limits = np.array([acceleration, self.settings["angular_acceleration_rad_s2"]]) * dt
        self.velocity += np.clip(requested - self.velocity, -limits, limits)
        v, w = self.velocity
        v *= float(values.get("actuation.linear_gain_error", 1.0) or 1.0)
        mid = self.pose[2] + w * dt / 2
        candidate = [self.pose[0] + v * math.cos(mid) * dt,
                     self.pose[1] + v * math.sin(mid) * dt, wrap_angle(self.pose[2] + w * dt)]
        support = self.support(*candidate[:2])
        collision = support is None
        ground, normal = support if support is not None else (self.ground, np.array([0., 0., 1.]))
        c, s = math.cos(candidate[2]), math.sin(candidate[2])
        local_normal = np.array([c*normal[0]+s*normal[1], -s*normal[0]+c*normal[1], normal[2]])
        pitch = math.atan2(local_normal[0], local_normal[2])
        roll = -math.atan2(local_normal[1], local_normal[2])
        collision |= self.intersects(candidate, ground, roll, pitch)
        if collision:
            self.collisions += 1
            self.collision_events.append({"timestamp_s": now, "proposed_pose": candidate,
                "retained_pose": list(self.pose), "ground_height_m": self.ground,
                "reason": "unsupported_step_or_slope" if support is None else "body_geometry_clearance",
                "geometry": self.last_collision_geometry})
            self.velocity[:] = 0
            v, w = 0., 0.
        else:
            self.pose = candidate
            self.ground, self.roll, self.pitch = ground, roll, pitch
        self.distance_m += abs(v * dt)
        self.set_pose(self.data, self.pose, self.ground, self.roll, self.pitch)
        # Physics remains at the scene's 2 ms timestep. Fixture tracking is synthetic.
        mujoco.mj_step(self.model, self.data, nstep=round(dt / self.model.opt.timestep))
        self.set_pose(self.data, self.pose, self.ground, self.roll, self.pitch)
        measured_v = v * float(values.get("odometry.linear_scale_error", 1.0) or 1.0) * (
            1 + self.random.normal(0, float(values.get("odometry.linear_noise_fraction",
                                                       self.settings["odom_linear_noise_fraction"]) or 0.0)))
        slip_probability = max(0.0, float(values.get("odometry.slip_probability", 0.0) or 0.0))
        if slip_probability > 0.0 and self.random.random() < slip_probability:
            # Foot slip: the body moves further than the leg odometry reports.
            measured_v *= 1.0 - min(1.0, max(0.0, float(values.get("odometry.slip_ratio", 0.0) or 0.0)))
        measured_w = (w * float(values.get("odometry.yaw_scale_error", 1.0) or 1.0)
                      + float(values.get("odometry.yaw_bias_rad_s", 0.0) or 0.0))
        if abs(v) + abs(w) > 0:
            measured_w += self.random.normal(0, float(values.get("odometry.yaw_noise_rad_s",
                                                                  self.settings["odom_yaw_noise_rad_s"]) or 0.0))
        imu_noise = float(values.get("imu.roll_pitch_noise_rad", 0.0) or 0.0)
        imu_roll = self.roll + (self.random.normal(0, imu_noise) if imu_noise > 0.0 else 0.0)
        imu_pitch = self.pitch + (self.random.normal(0, imu_noise) if imu_noise > 0.0 else 0.0)
        self.height_odometry += -measured_v * math.sin(self.pitch) * dt
        return MotionObservation(now + dt, measured_v, measured_w, imu_roll, imu_pitch,
                                 self.height_odometry, source="simulated_velocity_odometry_and_imu"), collision


def traversability_audit(grid, trace, settings, occupied_threshold):
    """Evaluation-only check that the map never contradicts the robot's own path.

    Every pose the robot actually visited is free space by construction, so a
    cell below the body footprint that the map marks occupied is a mapping error.
    The audit is computed from the recorded trace and never feeds decisions.
    """
    import numpy as _np

    known_free_threshold = -0.18
    footprint_x = _np.linspace(-0.30, 0.30, 7)
    footprint_y = _np.linspace(-0.22, 0.22, 5)
    offset_x, offset_y = _np.meshgrid(footprint_x, footprint_y)
    local = _np.column_stack((offset_x.ravel(), offset_y.ravel()))
    cells = {}
    step = max(1, len(trace) // 400)
    for row in trace[::step]:
        x, y, yaw = float(row[1]), float(row[2]), float(row[3])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        for local_x, local_y in local:
            cell = grid.world_to_cell(x + cos_yaw * local_x - sin_yaw * local_y,
                                      y + sin_yaw * local_x + cos_yaw * local_y)
            if cell is not None and cell not in cells:
                cells[cell] = grid.log_odds[cell[1], cell[0]]
    total = len(cells)
    occupied = sum(1 for value in cells.values() if value >= occupied_threshold)
    free = sum(1 for value in cells.values() if value <= known_free_threshold)
    unknown = total - occupied - free
    occupied_cells = int((grid.log_odds >= occupied_threshold).sum())
    free_cells = int((grid.log_odds <= known_free_threshold).sum())
    return {
        "traversed_body_cells": total,
        "traversed_occupied_cells": occupied,
        "traversed_unknown_cells": unknown,
        "traversability_contradiction_ratio": (occupied / total) if total else None,
        "traversed_unknown_ratio": (unknown / total) if total else None,
        "map_free_cells": free_cells,
        "map_occupied_cells": occupied_cells,
        "map_completeness_ratio": ((free_cells + occupied_cells) /
                                    float(grid.width * grid.height)),
        "map_cells": int(grid.width * grid.height),
    }


def json_ready(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def run_navigation(model, data, config, output, headless=False, scenario=None):
    output = Path(output)
    settings = config["navigation"]
    build_randomization(config, output)
    values = dict((config.get("randomization_sampled") or {}).get("values") or {})
    exploration_mode = bool(settings.get("exploration_mode", False))
    executor = MujocoVelocityExecutor(model, data, config)
    grid = TerrainGrid(config["field_size_m"], settings["grid_resolution_m"], settings)
    course_mode = config.get("course_enabled", False) and not exploration_mode
    initial_pose = executor.initial_estimate_pose()
    session = (CourseSession if course_mode else NavigationSession)(config, initial_pose, grid)
    course_evaluator = CourseEvaluator(config["course"]) if course_mode else None
    lidar = MujocoLidarObservationAdapter(model, data, config["lidar"], executor.base_height + config["lidar"]["mount_xyz_m"][2])
    evaluator = NavigationEvaluator(executor.pose)
    motion = MotionObservation(0, 0, 0)
    scan_latency = max(0.0, float(config["lidar"].get("scan_latency_s", 0.0) or 0.0))
    missing_scan_probability = min(1.0, max(0.0, float(values.get("lidar.missing_scan_probability", 0.0) or 0.0)))
    scan_random = np.random.RandomState(int(settings["random_seed"]) + 7000)
    pending_observations = []
    now, next_scan, next_save = 0., 0., 0.
    dt = settings["control_period_s"]
    viewer = None
    if not headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.opt.geomgroup[5] = False
        viewer.cam.lookat[:] = [0, 0, 0]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 16, 110, -65
    cancel_file = Path(os.environ.get("LITE2_CANCEL_FILE", str(output / "cancel_navigation")))
    trace, commands = [], []
    sensor_stream = gzip.open(output / "sensor_observations.jsonl.gz", "wt", encoding="utf-8", compresslevel=3)
    input_stream = gzip.open(output / "navigation_inputs.jsonl.gz", "wt", encoding="utf-8", compresslevel=3)
    sensor_index = 0
    last_report = -5.
    raw_batches = []
    failure_stopped_since = None
    native_state_finite = True
    try:
        while now <= settings["navigation_timeout_s"] + 2:
            wall_start = time.perf_counter()
            observation = None
            observation_index = None
            propagation_motion = vars(motion).copy()
            # Deliver queued scans first so a delayed sensor cannot overwrite a
            # newer observation inside the same control tick.
            while pending_observations and pending_observations[0][0] <= now + 1e-9:
                _, ready_observation, ready_index = pending_observations.pop(0)
                session.observe(ready_observation)
                observation, observation_index = ready_observation, ready_index
            if scenario is not None:
                scenario(now, model, data, session, motion)
            if now + 1e-8 >= next_scan:
                if not (scenario is not None and getattr(scenario, "drop_lidar", False)):
                    dropped_scan = (missing_scan_probability > 0.0
                                    and scan_random.random() < missing_scan_probability)
                    if not dropped_scan:
                        lidar.read(now)
                        observation = project_cloud(lidar.raw, config["lidar"], motion, executor.base_height + motion.ground_height_m)
                        raw_batches.append({"time": now, **{key: value.copy() for key, value in lidar.raw.items()}})
                        sensor_record = {"timestamp_s": now, "frame_id": observation.frame_id,
                                               "source": observation.source, "valid": observation.valid,
                                               "points": observation.points, "motion": vars(motion),
                                               "metadata": observation.metadata}
                        sensor_record["imu"] = {"frame_id": "base_link", "timestamp_s": now,
                            "orientation_wxyz": data.sensor("body_orientation").data.copy(),
                            "angular_velocity_rad_s": data.sensor("body_gyro").data.copy(),
                            "specific_force_m_s2": data.sensor("body_accelerometer").data.copy(),
                            "source": "mujoco_native_fixture_tracking_sensors"}
                        sensor_stream.write(json.dumps(sensor_record, default=json_ready, allow_nan=False) + "\n")
                        if scan_latency > 0.0:
                            pending_observations.append((now + scan_latency, observation, sensor_index))
                        else:
                            session.observe(observation)
                            observation_index = sensor_index
                        sensor_index += 1
                next_scan += 1 / config["lidar"]["frequency_hz"]
            command = session.tick(now, motion, cancelled=cancel_file.exists() or (viewer is not None and not viewer.is_running()))
            input_stream.write(json.dumps({"timestamp_s": now, "dt_s": dt, "motion": vars(motion),
                                          "propagation_motion": propagation_motion,
                                          "observation_index": observation_index,
                                          "cancelled": session.navigator.cancelled,
                                          "command": [command.linear_x_m_s, command.angular_z_rad_s],
                                          "course_stage": session.stage["id"] if course_mode else None,
                                          "course_state": session.state if course_mode else None,
                                          "status": session.status}, default=json_ready, allow_nan=False) + "\n")
            commands.append((now, command.linear_x_m_s, command.angular_z_rad_s, command.expires_at_s, session.status))
            motion, collision = executor.step(command, now, dt)
            native_state_finite &= bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
                                        and np.isfinite(data.sensordata).all())
            if scenario is not None and getattr(scenario, "drop_motion", False):
                motion.valid = False
            session.propagate(motion, dt)
            evaluator.record_execution(executor.pose, abs(executor.velocity[0] * dt), collision)
            now = round(now + dt, 8)
            if course_evaluator is not None:
                course_evaluator.record(now, executor.pose, executor.velocity[0])
            trace.append((now, *executor.pose, *session.odom_pose, *session.pose,
                          executor.ground, motion.roll_rad, motion.pitch_rad,
                          executor.velocity[0], executor.velocity[1]))
            if collision:
                session.status = "collision"
            if now >= next_save:
                save_map(grid, output, session.plan.path, config["robot_position_m"][:2], session.goal,
                         settings["robot_radius_m"], settings["occupied_log_odds_threshold"], [row[7:9] for row in trace])
                next_save += 10
                (output / "current_path.json").write_text(json.dumps(session.history[-1] if session.history else {}, indent=2), encoding="utf-8")
                # Chunk full clouds, keeping memory bounded even for long runs.
                if raw_batches:
                    np.savez_compressed(output / ("lidar_%07d.npz" % round(now * 1000)),
                                        timestamps_s=np.array([batch["time"] for batch in raw_batches]),
                                        **{key: np.stack([batch[key] for batch in raw_batches]) for key in lidar.raw})
                    raw_batches.clear()
            if now - last_report >= 10:
                print("NAV_PROGRESS t=%.1f status=%s pose=(%.2f,%.2f) scans=%d" %
                      (now, session.status, *session.pose[:2], grid.scan_count), flush=True)
                (output / "live_status.json").write_text(json.dumps({"time_s": now, "status": session.status,
                    "estimated_pose": session.pose, "truth_eval": executor.pose, "corrections": session.corrections,
                    "ground_eval": executor.ground, "odometry_height": motion.ground_height_m,
                    "command": [command.linear_x_m_s, command.angular_z_rad_s], "target": session.plan.target_xy,
                    "ground_returns": observation.metadata["ground_return_count"] if observation is not None else None}, indent=2), encoding="utf-8")
                last_report = now
            terminal = session.finished if course_mode else session.status in TERMINAL_STATES
            if terminal:
                if np.max(np.abs(executor.velocity)) < 0.002:
                    if failure_stopped_since is None:
                        failure_stopped_since = now
                else:
                    failure_stopped_since = None
                if failure_stopped_since is not None and now - failure_stopped_since >= settings["settling_stop_s"]:
                    break
            if viewer is not None and viewer.is_running():
                viewer.sync()
                time.sleep(max(0, dt - (time.perf_counter() - wall_start)))
    finally:
        if viewer is not None:
            viewer.close()
        if raw_batches:
            np.savez_compressed(output / "lidar_final.npz",
                                timestamps_s=np.array([batch["time"] for batch in raw_batches]),
                                **{key: np.stack([batch[key] for batch in raw_batches]) for key in lidar.raw})
        _write_csv(output / "slam_trajectory.csv", ["sim_time_s", "truth_x_m_eval", "truth_y_m_eval", "truth_yaw_rad_eval",
                   "odom_x_m", "odom_y_m", "odom_yaw_rad", "slam_x_m", "slam_y_m", "slam_yaw_rad",
                   "ground_height_m_eval", "imu_roll_rad", "imu_pitch_rad", "executed_v_m_s", "executed_w_rad_s"], trace)
        _write_csv(output / "velocity_commands.csv", ["sim_time_s", "linear_x_m_s", "angular_z_rad_s", "expires_at_s", "status"], commands)
        sensor_stream.close()
        input_stream.close()
        if course_mode:
            with (output / "course_events.jsonl").open("w", encoding="utf-8") as stream:
                for event in session.events:
                    stream.write(json.dumps(event, allow_nan=False) + "\n")
        (output / "replan_history.json").write_text(json.dumps(session.history, indent=2), encoding="utf-8")
        (output / "collision_events.json").write_text(json.dumps(executor.collision_events, indent=2), encoding="utf-8")
        (output / "planned_path.json").write_text(json.dumps(session.history[0] if session.history else {}, indent=2), encoding="utf-8")
        (output / "current_path.json").write_text(json.dumps(session.history[-1] if session.history else {}, indent=2), encoding="utf-8")
        map_info = save_map(grid, output, session.plan.path, config["robot_position_m"][:2], session.goal,
                            settings["robot_radius_m"], settings["occupied_log_odds_threshold"], [row[7:9] for row in trace])
        np.savez_compressed(output / "terrain_map.npz", height_m=grid.height_m, slope_deg=grid.slope_deg,
                            clearance_m=grid.clearance_m, support_hazard=grid.terrain_hazard, resolution_m=grid.resolution,
                            origin_xy_m=[grid.x_min, grid.y_min])
    metrics = evaluator.summary(session.goal, session.corrections, settings["goal_tolerance_m"])
    stopped = failure_stopped_since is not None and now - failure_stopped_since >= settings["settling_stop_s"]
    if exploration_mode:
        task_reached = session.status == "map_complete"
    elif course_mode:
        task_reached = bool(session.completed)
    else:
        task_reached = session.status == "goal_reached"
    checks = {"collision_free": executor.collisions == 0, "stopped_for_one_second": stopped,
              "finite_state": native_state_finite and bool(np.isfinite(np.asarray(trace, dtype=float)).all())}
    if exploration_mode:
        checks["map_complete"] = task_reached
    else:
        checks["goal_reached"] = task_reached
        checks["position_tolerance"] = metrics["truth_goal_tolerance_met"]
    summary = {"passed": all(checks.values()), "result": session.status, "checks": checks,
               "backend": "mujoco", "simulation_model": config["navigation_execution"],
               "physical_gait_validated": False, "loop_closure": False,
               "scene_id": config["scene_id"], "geometry_status": config["geometry_status"],
               "layout_review_status": config.get("review_status"), "random_seed": settings["random_seed"],
               "elapsed_simulation_s": now, "evaluation_only": metrics, "map": map_info,
               "estimated_pose_m_rad": session.pose, "goal_xy_m": session.goal,
               "last_motion_observation": vars(motion),
               "mission_clock_simulation_offset_s": float(data.time - now),
               "goal_coordinate_frame": "scene_map_with_known_start_prior",
               "planning_replans": len(session.history), "settings": settings,
               "mode": "exploration" if exploration_mode else ("course" if course_mode else "navigation"),
               "sensor_source": "ideal_lidar_with_synthetic_velocity_odometry_and_imu"}
    if exploration_mode:
        exploration = {}
        if hasattr(session, "exploration_summary"):
            exploration.update(session.exploration_summary() or {})
        exploration.update(traversability_audit(grid, trace, settings,
                                               settings["occupied_log_odds_threshold"]))
        exploration["scans_integrated"] = int(grid.scan_count)
        summary["exploration"] = exploration
    if config.get("randomization_sampled"):
        sampled = config["randomization_sampled"]
        summary["randomization"] = {"profile_id": sampled.get("profile_id"), "seed": sampled.get("seed"),
                                    "values": sampled.get("values"), "provenance": sampled.get("provenance")}
    if course_mode:
        course_summary = course_evaluator.summary(session.events, checks, session.status)
        course_summary.update(elapsed_simulation_s=now, evaluation_only=metrics,
                              active_stage=session.stage["id"], state=session.state,
                              random_seed=settings["random_seed"],
                              failure_reason=None if course_summary["passed"] else session.status)
        (output / "course_verification.json").write_text(json.dumps(course_summary, indent=2), encoding="utf-8")
        summary["passed"] = course_summary["passed"]
        summary["course_verification_file"] = "course_verification.json"
    for filename in ("navigation_verification.json", "navigation_summary.json"):
        (output / filename).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("NAVIGATION_FINISHED " + json.dumps({"passed": summary["passed"], "result": session.status,
          "goal_error_m": metrics["goal_error_m"], "collisions": executor.collisions,
          "output": str(output)}), flush=True)
    return summary


def _write_csv(path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)
