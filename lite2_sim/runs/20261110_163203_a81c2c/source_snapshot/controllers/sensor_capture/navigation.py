"""Webots navigation runner assembled from portable core and run-side adapters."""

import csv
import json
import math
import os
from pathlib import Path

import numpy as np

from navigation_core.artifacts import save_map
from navigation_core.control import NavigationController
from navigation_core.contracts import VelocityCommand
from navigation_core.evaluation import NavigationEvaluator
from navigation_core.grid import OccupancyGrid, wrap_angle
from navigation_core.planner import PlanResult
from navigation_core.webots_adapter import WebotsKinematicExecutor, WebotsLidarObservationAdapter


def _save_path(plan, output, start_pose):
    (Path(output) / "planned_path.json").write_text(
        json.dumps({
            "frame_id": "map",
            "planner": "8-connected A* with known-free frontier exploration and line-of-sight smoothing",
            "planning_snapshot": "initial accumulated SLAM map",
            "status": plan.status,
            "reason": plan.reason,
            "start_xy_m": list(start_pose),
            "goal_xy_m": list(plan.goal_xy),
            "target_xy_m": list(plan.target_xy) if plan.target_xy is not None else None,
            "poses_xy_m": plan.path,
        }, indent=2),
        encoding="utf-8",
    )


def _save_current_path(plan, output, pose, sim_time_s, history):
    path = plan.path
    record = {
        "sim_time_s": sim_time_s,
        "status": plan.status,
        "reason": plan.reason,
        "from_xy_m": list(pose[:2]),
        "goal_xy_m": list(plan.goal_xy) if plan.goal_xy is not None else None,
        "target_xy_m": list(plan.target_xy) if plan.target_xy is not None else None,
        "poses_xy_m": path,
        "path_length_m": sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])),
    }
    history.append(record)
    (Path(output) / "current_path.json").write_text(
        json.dumps({
            "frame_id": "map", "planner": "8-connected A*", "current_replan": record, "history": history,
        }, indent=2),
        encoding="utf-8",
    )


def run_navigation(robot, config, output, timestep):
    settings = config["navigation"]
    lidar_settings = config["lidar"]
    output = Path(output)
    lidar = robot.getDevice("front_lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()
    lidar.setFrequency(lidar_settings["frequency_hz"])
    fixture = robot.getFromDef("ROBOT_FIXTURE")
    if fixture is None:
        raise RuntimeError("Webots world is missing DEF ROBOT_FIXTURE.")

    grid = OccupancyGrid(config["field_size_m"], settings["grid_resolution_m"])
    initial_pose = [config["robot_position_m"][0], config["robot_position_m"][1], 0.0]
    truth_pose = initial_pose.copy()
    odom_pose = initial_pose.copy()
    slam_pose = initial_pose.copy()
    evaluator = NavigationEvaluator(truth_pose)
    executor = WebotsKinematicExecutor(fixture, config["robot_position_m"][2], grid)
    observation_adapter = WebotsLidarObservationAdapter(
        lidar, lidar_settings, config["robot_position_m"][2] + lidar_settings["mount_xyz_m"][2],
    )
    goal = tuple(settings["goal_xy_m"])
    extrinsic = lidar_settings["mount_xy_m"]
    dt = timestep / 1000.0
    random = np.random.RandomState(settings["random_seed"])
    controller_settings = dict(settings)
    controller_settings["min_range_m"] = lidar_settings["min_range_m"]
    navigator = NavigationController(controller_settings)
    cancellation_file = Path(os.environ.get("LITE2_CANCEL_FILE", str(output / "cancel_navigation")))

    scan_period = settings["scan_mapping_period_s"]
    next_scan_time = robot.getTime()
    last_replan_time = -math.inf
    mapping_start = robot.getTime()
    navigation_start = None
    initial_mapping_done = False
    status = "mapping"
    current_command = VelocityCommand(0.0, 0.0, mapping_start,
                                      mapping_start + settings.get("command_timeout_s", 0.30))
    last_observation = None
    last_scan_points = []
    scan_metadata = None
    last_status_report = mapping_start
    path = []
    plan = PlanResult("insufficient_observations", goal_xy=goal)
    trace = []
    initial_path = []
    replan_history = []
    collision_map = None
    correction_count = 0

    while robot.step(timestep) != -1:
        now = robot.getTime()
        if cancellation_file.is_file():
            navigator.cancel()

        if now >= next_scan_time:
            capture_path = output / "lidar_raw_first.npz" if grid.scan_count == 0 else None
            observation = observation_adapter.read(now, capture_path)
            last_observation = observation
            scan_metadata = observation.metadata
            last_scan_points = list(observation.points)
            if grid.scan_count == 0 and observation.valid:
                (output / "lidar_scan.json").write_text(
                    json.dumps(scan_metadata, indent=2, allow_nan=False), encoding="utf-8",
                )
            if observation.valid:
                if grid.scan_count:
                    prior = tuple(slam_pose)
                    matched = grid.match_pose(
                        prior, observation.points, extrinsic, lidar_settings["max_range_m"],
                        settings["scan_match_search_xy_m"], settings["scan_match_search_yaw_rad"],
                    )
                    if math.hypot(matched[0] - prior[0], matched[1] - prior[1]) > 0.005 or abs(wrap_angle(matched[2] - prior[2])) > 0.01:
                        correction_count += 1
                    slam_pose = list(matched)
                else:
                    slam_pose = odom_pose.copy()
                grid.integrate(
                    observation.points, slam_pose, extrinsic,
                    lidar_settings["max_range_m"], lidar_settings["min_range_m"], settings["scan_stride"],
                )
                collision_map = grid.inflated(
                    settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
                )
                trace.append((now, *truth_pose, *odom_pose, *slam_pose,
                              current_command.linear_x_m_s, current_command.angular_z_rad_s))
            next_scan_time = now + scan_period

        if not initial_mapping_done and grid.scan_count >= 3 and now - mapping_start >= settings["initial_mapping_duration_s"]:
            navigation_start = now
            initial_mapping_done = True
            plan = grid.plan_result(
                slam_pose, goal, settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
                settings["unknown_cell_cost"], settings.get("planning_clearance_m", 0.0),
            )
            path = plan.path
            last_replan_time = now
            initial_path = path[:]
            _save_path(plan, output, slam_pose[:2])
            _save_current_path(plan, output, slam_pose, now, replan_history)
            print(
                "SLAM_MAP_READY scans=%d cells=%dx%d path_points=%d plan_status=%s" %
                (grid.scan_count, grid.width, grid.height, len(path), plan.status), flush=True,
            )
        elif not initial_mapping_done and now - mapping_start > max(5.0, settings["initial_mapping_duration_s"] + 3.0):
            initial_mapping_done = True
            navigation_start = now
            plan = PlanResult("insufficient_observations", goal_xy=goal,
                              reason="fewer_than_three_valid_scans_during_initial_mapping")
            status = plan.status

        if initial_mapping_done and status not in ("goal_reached", "timeout", "stalled", "cancelled",
                                                   "start_blocked", "goal_blocked", "map_disconnected",
                                                   "insufficient_observations", "start_out_of_map", "goal_out_of_map"):
            if last_observation is not None and last_observation.valid and (
                    now - last_replan_time >= settings["replan_period_s"] or not path):
                plan = grid.plan_result(
                    slam_pose, goal, settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
                    settings["unknown_cell_cost"], settings.get("planning_clearance_m", 0.0),
                )
                path = plan.path
                last_replan_time = now
                _save_current_path(plan, output, slam_pose, now, replan_history)

            decision = navigator.update(
                now, slam_pose, plan, last_observation,
                max(0.0, now - navigation_start), progress_active=True,
            )
            status = decision.status
            current_command = decision.command
        elif initial_mapping_done and status == "insufficient_observations":
            current_command = VelocityCommand(0.0, 0.0, now, now + settings.get("command_timeout_s", 0.30))
        else:
            current_command = VelocityCommand(0.0, 0.0, now, now + settings.get("command_timeout_s", 0.30))

        executed_command, traveled_step, collision = executor.step(
            truth_pose, current_command, now, dt, collision_map,
        )
        evaluator.record_execution(truth_pose, traveled_step, collision)
        if executed_command[0] or executed_command[1]:
            old_odom = odom_pose[:]
            odom_v = executed_command[0] * (1.0 + random.normal(0.0, settings["odom_linear_noise_fraction"]))
            odom_w = executed_command[1] + random.normal(0.0, settings["odom_yaw_noise_rad_s"])
            odom_mid_yaw = odom_pose[2] + odom_w * dt * 0.5
            odom_pose[0] += odom_v * math.cos(odom_mid_yaw) * dt
            odom_pose[1] += odom_v * math.sin(odom_mid_yaw) * dt
            odom_pose[2] = wrap_angle(odom_pose[2] + odom_w * dt)
            # Propagate scan-matched pose with odometry deltas; future planning
            # never reads truth_pose.
            slam_pose[0] += odom_pose[0] - old_odom[0]
            slam_pose[1] += odom_pose[1] - old_odom[1]
            slam_pose[2] = wrap_angle(slam_pose[2] + wrap_angle(odom_pose[2] - old_odom[2]))

        if status in ("goal_reached", "timeout", "stalled", "cancelled", "start_blocked", "goal_blocked",
                      "map_disconnected", "insufficient_observations", "start_out_of_map", "goal_out_of_map"):
            break
        if now - last_status_report >= 5.0:
            estimated_trajectory = [(record[7], record[8]) for record in trace]
            save_map(
                grid, output, initial_path, initial_pose[:2], goal,
                settings["robot_radius_m"], settings["occupied_log_odds_threshold"], estimated_trajectory,
            )
            print(
                "NAV_PROGRESS t=%.1fs status=%s pose=(%.2f,%.2f) scans=%d" %
                (now - mapping_start, status, slam_pose[0], slam_pose[1], grid.scan_count), flush=True,
            )
            last_status_report = now

    map_metadata = save_map(
        grid, output, initial_path, initial_pose[:2], goal,
        settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
        [(record[7], record[8]) for record in trace],
    )
    _save_current_path(plan, output, slam_pose, robot.getTime(), replan_history)
    (output / "replan_history.json").write_text(json.dumps(replan_history, indent=2), encoding="utf-8")
    with (output / "slam_trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "sim_time_s", "truth_x_m_eval", "truth_y_m_eval", "truth_yaw_rad_eval",
            "odom_x_m", "odom_y_m", "odom_yaw_rad", "slam_x_m", "slam_y_m", "slam_yaw_rad",
            "cmd_v_m_s", "cmd_w_rad_s",
        ])
        writer.writerows(trace)
    initial_route_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(initial_path, initial_path[1:]))
    remaining_route_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    summary = {
        "result": status,
        "scene_id": config.get("scene_id", "ruins_reference"),
        "mapping": "2-D occupancy grid with odometry-propagated local scan-to-map pose correction",
        "loop_closure": False,
        "sensor": {
            "model": lidar_settings["model_status"],
            "horizontal_resolution": lidar_settings["horizontal_resolution"],
            "vertical_layers": int(lidar.getNumberOfLayers()),
            "field_of_view_rad": lidar_settings["field_of_view_rad"],
            "max_range_m": lidar_settings["max_range_m"],
            "self_filter_status": lidar_settings.get("self_filter_status"),
            "last_observation_valid": bool(last_observation.valid) if last_observation is not None else False,
            "last_observation_age_s": max(0.0, robot.getTime() - last_observation.timestamp_s) if last_observation is not None else None,
        },
        "task": {"original_goal_xy_m": list(goal), "terminal_reason": plan.reason if status == plan.status else status},
        "planning": {
            "planner": "8-connected A* on observed free cells with reachable-frontier exploration",
            "status": plan.status, "reason": plan.reason,
            "robot_radius_m": settings["robot_radius_m"],
            "planning_clearance_m": settings.get("planning_clearance_m", 0.0),
            "goal_xy_m": list(goal),
            "selected_target_xy_m": list(plan.target_xy) if plan.target_xy is not None else None,
            "initial_path_points": len(initial_path),
            "initial_planned_path_length_m": initial_route_length,
            "final_remaining_path_length_m": remaining_route_length,
            "replans": max(0, len(replan_history) - 1),
        },
        "simulation_model": "kinematic planar base; manufacturer joint pose held fixed; no gait or terrain contact simulation",
        "evaluation_only": evaluator.summary(goal, correction_count),
        "map": map_metadata,
    }
    (output / "navigation_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(
        "NAVIGATION_FINISHED " + json.dumps({
            "result": status,
            "distance_m": round(evaluator.distance_travelled_m, 2),
            "scans": grid.scan_count,
            "map": str(output / "slam_map.png"),
        }, ensure_ascii=False), flush=True,
    )
    return summary
