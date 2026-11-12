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
                    control_period_s=0.05, settling_stop_s=1.0)
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
    config["lidar"]["model_status"] = "ideal_navigation_multilayer_lidar_placeholder_pending_hardware"
    config["lidar"]["coverage_source"] = "navigation_near_ground_visibility_assumption_pending_hardware_validation"
    config["scene_id"] = scene_id
    return config


class MujocoVelocityExecutor:
    """Private simulation state, geometric contact checks and synthetic motion sensors."""

    def __init__(self, model, data, config):
        self.model, self.data = model, data
        self.settings = config["navigation"]
        self.base_height = float(config["robot_position_m"][2])
        self.pose = [*config["robot_position_m"][:2], 0.0]
        self.ground = 0.0
        self.roll, self.pitch = 0.0, 0.0
        self.height_odometry = 0.0
        self.velocity = np.zeros(2)
        self.random = np.random.RandomState(self.settings["random_seed"])
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
        requested = np.array([command.linear_x_m_s, command.angular_z_rad_s])
        if command.expired(now) or command.created_at_s > now:
            requested[:] = 0
        requested = np.clip(requested, [-self.settings["max_linear_speed_m_s"], -self.settings["max_yaw_speed_rad_s"]],
                            [self.settings["max_linear_speed_m_s"], self.settings["max_yaw_speed_rad_s"]])
        if requested.any():
            self.last_moving_command_at = now
        elif now - self.last_moving_command_at < self.settings["stop_delay_s"]:
            requested = self.velocity.copy()
        acceleration = self.settings["acceleration_m_s2"] if abs(requested[0]) > abs(self.velocity[0]) else self.settings["deceleration_m_s2"]
        limits = np.array([acceleration, self.settings["angular_acceleration_rad_s2"]]) * dt
        self.velocity += np.clip(requested - self.velocity, -limits, limits)
        v, w = self.velocity
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
        measured_v = v * (1 + self.random.normal(0, self.settings["odom_linear_noise_fraction"]))
        measured_w = w + (self.random.normal(0, self.settings["odom_yaw_noise_rad_s"]) if abs(v)+abs(w) > 0 else 0)
        self.height_odometry += -measured_v * math.sin(self.pitch) * dt
        return MotionObservation(now + dt, measured_v, measured_w, self.roll, self.pitch,
                                 self.height_odometry, source="simulated_velocity_odometry_and_imu"), collision


def json_ready(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def run_navigation(model, data, config, output, headless=False, scenario=None):
    output = Path(output)
    settings = config["navigation"]
    executor = MujocoVelocityExecutor(model, data, config)
    grid = TerrainGrid(config["field_size_m"], settings["grid_resolution_m"], settings)
    session = NavigationSession(config, executor.pose, grid)
    lidar = MujocoLidarObservationAdapter(model, data, config["lidar"], executor.base_height + config["lidar"]["mount_xyz_m"][2])
    evaluator = NavigationEvaluator(executor.pose)
    motion = MotionObservation(0, 0, 0)
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
            if scenario is not None:
                scenario(now, model, data, session, motion)
            if now + 1e-8 >= next_scan:
                if not (scenario is not None and getattr(scenario, "drop_lidar", False)):
                    lidar.read(now)
                    observation = project_cloud(lidar.raw, config["lidar"], motion, executor.base_height + motion.ground_height_m)
                    session.observe(observation)
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
                    observation_index = sensor_index
                    sensor_index += 1
                next_scan += 1 / config["lidar"]["frequency_hz"]
            command = session.tick(now, motion, cancelled=cancel_file.exists() or (viewer is not None and not viewer.is_running()))
            input_stream.write(json.dumps({"timestamp_s": now, "dt_s": dt, "motion": vars(motion),
                                          "propagation_motion": propagation_motion,
                                          "observation_index": observation_index,
                                          "cancelled": session.navigator.cancelled,
                                          "command": [command.linear_x_m_s, command.angular_z_rad_s],
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
            if session.status in TERMINAL_STATES:
                if session.status != "goal_reached" and np.max(np.abs(executor.velocity)) < 0.002:
                    if failure_stopped_since is None:
                        failure_stopped_since = now
                if ((session.status == "goal_reached" and session.stopped_since is not None and now - session.stopped_since >= settings["settling_stop_s"])
                        or (failure_stopped_since is not None and now - failure_stopped_since >= settings["settling_stop_s"])):
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
    stop_started = session.stopped_since if session.status == "goal_reached" else failure_stopped_since
    stopped = stop_started is not None and now - stop_started >= settings["settling_stop_s"]
    checks = {"goal_reached": session.status == "goal_reached", "position_tolerance": metrics["truth_goal_tolerance_met"],
              "collision_free": executor.collisions == 0, "stopped_for_one_second": stopped,
              "finite_state": native_state_finite and bool(np.isfinite(np.asarray(trace, dtype=float)).all())}
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
               "sensor_source": "ideal_lidar_with_synthetic_velocity_odometry_and_imu"}
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
