"""Webots sensor and execution adapters around the shared navigation session."""

import csv
import json
import os
from pathlib import Path

import numpy as np

from navigation_core.artifacts import save_map
from navigation_core.contracts import MotionObservation
from navigation_core.evaluation import NavigationEvaluator
from navigation_core.session import NavigationSession, TERMINAL_STATES
from navigation_core.webots_adapter import WebotsKinematicExecutor, WebotsLidarObservationAdapter


def run_navigation(robot, config, output, timestep):
    settings, lidar_settings = config["navigation"], config["lidar"]
    output = Path(output)
    lidar = robot.getDevice("front_lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()
    lidar.setFrequency(lidar_settings["frequency_hz"])
    fixture = robot.getFromDef("ROBOT_FIXTURE")
    if fixture is None:
        raise RuntimeError("Webots world is missing DEF ROBOT_FIXTURE.")
    initial = [*config["robot_position_m"][:2], 0.0]
    truth_pose = list(initial)
    session = NavigationSession(config, initial)
    evaluator = NavigationEvaluator(initial)
    executor = WebotsKinematicExecutor(fixture, config["robot_position_m"][2], session.grid)
    adapter = WebotsLidarObservationAdapter(lidar, lidar_settings,
        config["robot_position_m"][2] + lidar_settings["mount_xyz_m"][2])
    dt = timestep / 1000.0
    random = np.random.RandomState(settings["random_seed"])
    cancel_file = Path(os.environ.get("LITE2_CANCEL_FILE", str(output / "cancel_navigation")))
    next_scan, next_save = robot.getTime(), robot.getTime()
    motion = MotionObservation(robot.getTime(), 0, 0)
    trace = []
    stop_confirmed = False
    while robot.step(timestep) != -1:
        now = robot.getTime()
        if now >= next_scan:
            observation = adapter.read(now, output / "lidar_raw_first.npz" if session.grid.scan_count == 0 else None)
            session.observe(observation)
            if session.grid.scan_count == 1:
                (output / "lidar_scan.json").write_text(json.dumps(observation.metadata, indent=2), encoding="utf-8")
            next_scan = now + settings["scan_mapping_period_s"]
        command = session.tick(now, motion, cancel_file.is_file())
        blocked = session.grid.inflated(settings["robot_radius_m"], settings["occupied_log_odds_threshold"])
        executed, distance, collision = executor.step(truth_pose, command, now, dt, blocked)
        evaluator.record_execution(truth_pose, distance, collision)
        v, w = executed
        measured_v = v * (1 + random.normal(0, settings["odom_linear_noise_fraction"]))
        measured_w = w + (random.normal(0, settings["odom_yaw_noise_rad_s"]) if abs(v)+abs(w) > 0 else 0)
        motion = MotionObservation(now, measured_v, measured_w, source="webots_synthetic_odometry")
        session.propagate(motion, dt)
        trace.append((now, *truth_pose, *session.odom_pose, *session.pose,
                      command.linear_x_m_s, command.angular_z_rad_s))
        if now >= next_save:
            save_map(session.grid, output, session.plan.path, initial[:2], session.goal,
                     settings["robot_radius_m"], settings["occupied_log_odds_threshold"], [row[7:9] for row in trace])
            next_save = now + 5
        if session.status in TERMINAL_STATES:
            stop_confirmed = session.stopped_since is not None and now - session.stopped_since >= 1
            if session.status != "goal_reached" or stop_confirmed:
                break

    map_info = save_map(session.grid, output, session.plan.path, initial[:2], session.goal,
                        settings["robot_radius_m"], settings["occupied_log_odds_threshold"], [row[7:9] for row in trace])
    with (output / "slam_trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_time_s", "truth_x_m_eval", "truth_y_m_eval", "truth_yaw_rad_eval",
                         "odom_x_m", "odom_y_m", "odom_yaw_rad", "slam_x_m", "slam_y_m", "slam_yaw_rad", "cmd_v_m_s", "cmd_w_rad_s"])
        writer.writerows(trace)
    history = session.history
    for filename, value in (("planned_path.json", history[0] if history else {}),
                            ("current_path.json", {"frame_id": "map", "current_replan": history[-1] if history else {}, "history": history}),
                            ("replan_history.json", history)):
        (output / filename).write_text(json.dumps(value, indent=2), encoding="utf-8")
    summary = {
        "result": session.status, "scene_id": config.get("scene_id", "ruins_reference"),
        "backend": "webots", "mapping": "odometry with local scan-to-map correction", "loop_closure": False,
        "simulation_model": "velocity-level planar fixture with held joint pose",
        "task": {"original_goal_xy_m": session.goal, "decision_pose_xy_m": session.pose[:2],
                 "decision_pose_source": "scan_matched_odometry_estimate", "terminal_reason": session.status},
        "sensor": {"model": lidar_settings["model_status"], "vertical_layers": int(lidar.getNumberOfLayers()),
                   "horizontal_resolution": lidar_settings["horizontal_resolution"],
                   "last_observation_valid": session.last_observation.valid if session.last_observation else False},
        "planning": {"status": session.plan.status, "reason": session.plan.reason,
                     "goal_xy_m": session.goal, "selected_target_xy_m": session.plan.target_xy,
                     "replans": max(0, len(history)-1)},
        "evaluation_only": evaluator.summary(session.goal, session.corrections, settings["goal_tolerance_m"]),
        "stop_confirmed": stop_confirmed, "map": map_info}
    (output / "navigation_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("NAVIGATION_FINISHED " + json.dumps({"result": session.status, "distance_m": evaluator.distance_travelled_m,
                                               "scans": session.grid.scan_count}), flush=True)
    return summary
