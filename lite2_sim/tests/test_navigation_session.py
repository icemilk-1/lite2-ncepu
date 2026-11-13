"""Regressions for shared lifecycle, terrain projection and execution contracts."""

import ast
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers/sensor_capture"))

from mujoco_navigation import configure_navigation, MujocoVelocityExecutor, run_navigation
from navigation_core.contracts import MotionObservation, NavigationObservation, VelocityCommand
from navigation_core.control import NavigationController
from navigation_core.grid import OccupancyGrid
from navigation_core.session import NavigationSession
from navigation_core.planner import PlanResult
from navigation_core.terrain import project_cloud, TerrainGrid, tilt_rotation
from tools.prepare_mujoco import build_scene


def configuration(scene="sensor_lab"):
    config = json.loads((PROJECT / "config" / (scene + ".json")).read_text(encoding="utf-8"))
    return configure_navigation(config, scene)


class NavigationSessionTests(unittest.TestCase):
    def test_portable_modules_parse_as_python36(self):
        for name in ("contracts.py", "session.py", "terrain.py", "control.py", "grid.py", "planner.py"):
            path = PROJECT / "controllers/sensor_capture/navigation_core" / name
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))

    def test_stationary_scan_matching_has_exact_zero_candidate(self):
        grid = OccupancyGrid((8, 8), .05)
        points = [(2 * math.cos(a), 2 * math.sin(a), True) for a in np.linspace(-math.pi, math.pi, 120)]
        grid.integrate(points, (0, 0, 0), (0, 0), 8, .08)
        pose = grid.match_pose((0, 0, 0), points, (0, 0), 8, .025, .02)
        self.assertTrue(np.allclose(pose, (0, 0, 0)))

    def test_lookahead_interpolates_short_corner_and_long_segment(self):
        target = NavigationController._lookahead_target((0, 0, 0), [(0, 0), (0, .1), (3, .1)], .25)
        self.assertTrue(np.allclose(target, (.15, .1)))

    def test_cancel_timeout_and_invalid_motion_emit_zero(self):
        for mode in ("cancel", "timeout", "motion"):
            config = configuration()
            session = NavigationSession(config, (-4, -3, 0))
            session.tick(0, MotionObservation(0, 0, 0))
            now = config["navigation"]["navigation_timeout_s"] if mode == "timeout" else 1.0
            command = session.tick(now, MotionObservation(now, 0, 0, valid=False), cancelled=mode == "cancel")
            self.assertTrue(command.is_zero)
            self.assertEqual(session.status, {"cancel": "cancelled", "timeout": "timeout", "motion": "localization_lost"}[mode])

    def test_seen_goal_can_explore_around_unobserved_connection(self):
        grid = OccupancyGrid((6, 6), .1)
        grid.log_odds[20:40, 5:25] = -1
        grid.log_odds[20:40, 35:55] = -1
        result = grid.plan_result((-1, 0), (1, 0), .1, .7)
        self.assertEqual(result.status, "exploring")
        self.assertEqual(result.goal_xy, (1, 0))

    def test_projection_ground_next_to_missing_elevation_layer(self):
        # A continuous ramp remains ground when the next layer misses entirely.
        layers, beams = 16, 360
        horizontal = np.linspace(-math.pi, math.pi, beams, endpoint=False)
        vertical = np.linspace(-.3, .3, layers)
        directions = np.stack(np.broadcast_arrays(np.cos(vertical[:, None])*np.cos(horizontal),
            np.cos(vertical[:, None])*np.sin(horizontal), np.sin(vertical[:, None])*np.ones(beams)), axis=-1)
        normal = np.array([0., -.1, 1.])
        denominator = directions @ normal
        distance = np.divide(-.43, denominator, out=np.full((layers, beams), np.inf), where=denominator < -.001)
        hits = (distance > 0) & (distance < 8)
        xyz = directions * np.where(hits, distance, 8)[:, :, None]
        raw = {"xyz_m": xyz, "hit": hits, "valid": np.ones_like(hits)}
        settings = configuration()["lidar"]
        observation = project_cloud(raw, settings, MotionObservation(0, 0, 0), .33)
        north = [(x, y, hit) for x, y, hit in observation.points if y > 0 and abs(x) < .05]
        self.assertTrue(north)
        self.assertFalse(any(hit for x, y, hit in north))

    def test_height_interpolation_does_not_propagate_unobserved_cells(self):
        settings = configuration()["navigation"]
        grid = TerrainGrid((4, 4), .05, settings)
        metadata = {"terrain_points_sensor_level_m": [[0, 0, 0]], "terrain_slopes_deg": [0],
                    "terrain_normals_sensor_level": [[0, 0, 1]]}
        observation = NavigationObservation(0, "lidar", [], True, "test", metadata)
        for _ in range(20):
            grid.integrate_terrain(observation, (0, 0, 0), (0, 0))
        self.assertLessEqual(int(np.isfinite(grid.height_m).sum()), 5)

    def test_oriented_rollout_can_pass_a_narrow_straight_corridor(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        left, _ = grid.world_to_cell(-.325, 0)
        right, _ = grid.world_to_cell(.325, 0)
        grid.log_odds[:, left] = 3
        grid.log_odds[:, right] = 3
        session = NavigationSession(config, (0, 0, -math.pi/2), grid)
        session.status = "navigating"
        session.plan = PlanResult("path_found", [(0, 0), (0, -1.5)], (0, -1.5), (0, -1.5))
        command = session._local_rollout(VelocityCommand(.15, 0, 0, .3), 0)
        self.assertGreater(command.linear_x_m_s, 0)
        session.pose[2] = 0
        command = session._local_rollout(VelocityCommand(.15, 0, 0, .3), 0)
        self.assertTrue(command.is_zero)

    def test_velocity_expiry_acceleration_and_ramp_support(self):
        config = configuration("ruins_reference")
        with tempfile.TemporaryDirectory() as output:
            model, data, _ = build_scene(config, "ruins_reference", output)
            executor = MujocoVelocityExecutor(model, data, config)
            motion, collision = executor.step(VelocityCommand(.15, .4, 0, .3), 0, .05)
            self.assertFalse(collision)
            self.assertLessEqual(executor.velocity[0], .35*.05 + 1e-9)
            for tick in range(30):
                executor.step(VelocityCommand(.15, .4, 0, .3), 2 + tick*.05, .05)
            self.assertTrue(np.allclose(executor.velocity, 0))
            executor.ground = .17
            support = executor.support(-5.1, 2)
            self.assertIsNotNone(support)
            approach = config["upper_left_ramp"]["sections"][0]
            _, _, lower_y, upper_y = approach["bounds_m"]
            heights = approach["corner_heights_m"]
            expected_height = heights[0] + (2 - lower_y) / (upper_y - lower_y) * (heights[3] - heights[0])
            self.assertAlmostEqual(support[0], expected_height, places=5)

    def test_rollout_can_leave_a_conflicting_soft_margin(self):
        config = configuration()
        grid = TerrainGrid((6, 6), .05, config["navigation"])
        grid.log_odds.fill(-1)
        col, row = grid.world_to_cell(-1.225, 1.575)
        grid.log_odds[row, col] = 4
        session = NavigationSession(config, (-.8376423, 1.4954091, 2.2703985), grid)
        session.status = "exploring"
        session.plan = PlanResult("exploring", [(-.825, 1.475), (-.775, 1.525), (-.725, 2.025)], (2, 2), (-.725, 2.025))
        command = session._local_rollout(VelocityCommand(.1, -.4, 0, .3), 0)
        self.assertFalse(command.is_zero)

    def test_frontiers_ignore_unknown_behind_an_impassable_wall_gap(self):
        grid = OccupancyGrid((8, 8), .05)
        grid.log_odds[:, :80] = -1
        grid.log_odds[:, 80] = 3
        grid.log_odds[80, 80] = 0
        grid.frontier_radius_m = .43
        result = grid.plan_result((-1, 0), (2, 0), .28, .7)
        self.assertEqual(result.status, "insufficient_observations")

    def test_inflation_uses_metric_radius_without_rounding_up(self):
        grid = OccupancyGrid((4, 4), .05)
        grid.log_odds[40, 40] = 3
        blocked = grid.inflated(.28, .7)
        self.assertTrue(blocked[40, 45])
        self.assertFalse(blocked[40, 46])

    def test_projection_accounts_for_the_tilted_lidar_mount(self):
        settings = configuration()["lidar"]
        pitch = -.10
        rotation = tilt_rotation(0, pitch)
        mount = rotation @ np.asarray(settings["mount_xyz_m"])
        y, z = np.meshgrid(np.linspace(-.5, .5, 12), np.linspace(.05, .55, 6))
        level = np.stack((np.full_like(y, 2.), y, z), axis=-1)
        level -= np.array([mount[0], mount[1], .33+mount[2]])
        raw = {"xyz_m": level @ rotation, "hit": np.ones(y.shape, dtype=bool), "valid": np.ones(y.shape, dtype=bool)}
        observation = project_cloud(raw, settings, MotionObservation(0, 0, 0, pitch_rad=pitch), .33)
        self.assertEqual(observation.frame_id, "front_lidar_level")
        self.assertTrue(observation.valid)
        self.assertTrue(all(abs(x-2.) < 1e-6 for x, y, hit in observation.points if hit))

    def test_oriented_start_recovers_an_overconservative_circle(self):
        config = configuration()
        config["navigation"]["goal_xy_m"] = [1, -1]
        grid = TerrainGrid((6, 6), .05, config["navigation"])
        grid.log_odds.fill(-1)
        col, row = grid.world_to_cell(-1.075, .275)
        grid.log_odds[row, col] = 3
        pose = (-.898, .442, -.877)
        self.assertEqual(grid.plan_result(pose, (1, -1), .28, .7).status, "start_blocked")
        session = NavigationSession(config, pose, grid)
        session._replan(0)
        self.assertEqual(session.plan.status, "path_found")
        self.assertEqual(session.history[-1]["planning_radius_m"], .24)

    def test_closed_viewer_cancels_and_confirms_stop(self):
        config = configuration()
        viewer = SimpleNamespace(opt=SimpleNamespace(geomgroup=np.ones(6, dtype=bool)),
                                 cam=SimpleNamespace(lookat=np.zeros(3)),
                                 is_running=lambda: False, close=lambda: None)
        with tempfile.TemporaryDirectory() as output:
            model, data, _ = build_scene(config, "sensor_lab", output)
            with patch("mujoco.viewer.launch_passive", return_value=viewer), contextlib.redirect_stdout(io.StringIO()):
                result = run_navigation(model, data, config, output, headless=False)
            self.assertEqual(result["result"], "cancelled")
            self.assertTrue(result["checks"]["stopped_for_one_second"])

    def test_raised_support_blocks_a_drop_at_the_ramp_edge(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        grid.height_m[:, :40] = .3
        grid.height_m[:, 40:] = 0
        grid.height_updated_s.fill(0)
        session = NavigationSession(config, (-.02, 0, 0), grid)
        session.motion = MotionObservation(0, 0, 0, ground_height_m=.3)
        session.status = "exploring"
        session.plan = PlanResult("exploring", [(-.02, 0), (1, 0)], (1, 0), (1, 0))
        command = session._local_rollout(VelocityCommand(.15, 0, 0, .3), 0)
        self.assertLessEqual(command.linear_x_m_s, .03)

    def test_local_rollout_stops_when_body_overlaps_an_entry_hazard(self):
        config = configuration()
        grid = TerrainGrid((12, 10), .05, config["navigation"])
        grid.log_odds.fill(-1)
        grid.terrain_hazard[101:107, 218:224] = True
        session = NavigationSession(config, (5.0893, .0396, .8573), grid)
        session.status = "navigating"
        path = [(5.075, .025), (5.075, -.025), (5.225, -.025), (5.325, 2.975)]
        session.plan = PlanResult("path_found", path, (4.9339, 3.8074), (4.9339, 3.8074))
        command = session._local_rollout(VelocityCommand(.15, .4, 0, .3), 0)
        self.assertTrue(command.is_zero)
        self.assertGreater(session.local_rollout_debug["rejected"]["terrain"], 0)

    def test_reverse_tracking_uses_the_direction_of_travel(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "navigating"
        # Reverse is an explicit primitive now: only a reverse checkpoint (the
        # dead-end exit) drives tail-first.
        session.settings["reverse_only"] = True
        session.plan = PlanResult("path_found", [(0, 0), (-1, 0)], (-1, 0), (-1, 0))
        command = session._local_rollout(VelocityCommand(0, .4, 0, .3), 0)
        self.assertLess(command.linear_x_m_s, 0)

    def test_overhead_plane_is_not_a_ground_surface(self):
        settings = configuration()["lidar"]
        x, y = np.meshgrid(np.linspace(1, 2, 6), np.linspace(-.5, .5, 12), indexing="ij")
        cloud = np.stack((x, y, np.full_like(x, .33)), axis=-1)
        observation = project_cloud({"xyz_m": cloud, "hit": np.ones(x.shape, bool),
                                     "valid": np.ones(x.shape, bool)}, settings, MotionObservation(0, 0, 0), .33)
        self.assertEqual(len(observation.metadata["terrain_points_sensor_level_m"]), 0)
        self.assertGreater(len(observation.metadata["surface_points_sensor_level_m"]), 0)

    def test_low_riser_is_an_obstacle(self):
        config = configuration("ruins_reference")
        settings = config["lidar"]
        z, y = np.meshgrid(np.linspace(.04, .10, 6), np.linspace(-.2, .2, 12), indexing="ij")
        cloud = np.stack((np.full_like(z, .7), y,
                          z - float(settings["mount_xyz_m"][2]) - .33), axis=-1)
        observation = project_cloud({"xyz_m": cloud, "hit": np.ones(z.shape, bool),
                                     "valid": np.ones(z.shape, bool)}, settings,
                                    MotionObservation(0, 0, 0), .33)
        self.assertGreater(len(observation.metadata["obstacle_points_sensor_level_m"]), 0)
        self.assertEqual(len(observation.metadata["terrain_points_sensor_level_m"]), 0)

    def test_low_obstacle_is_inflated_and_not_cleared_as_a_step(self):
        config = configuration("ruins_reference")
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        col, row = grid.world_to_cell(.5, 0)
        grid.log_odds[row, col] = 3
        grid.obstacle_top_m[row, col] = .10
        grid.last_hit_s[row, col] = 0
        self.assertFalse(grid.traversable_step_mask().any())
        self.assertTrue(grid.inflated(.3, .7)[row, col - 4])
        grid.height_m.fill(0)
        grid.height_m[row, col] = .10
        grid.height_updated_s.fill(0)
        observation = NavigationObservation(0, "lidar", [], True, "test", {})
        grid.integrate_terrain(observation, (0, 0, 0), (0, 0))
        self.assertTrue(grid.terrain_hazard[row, col])
        self.assertFalse(grid.terrain_hazard[row, col - 1])
        self.assertGreaterEqual(grid.log_odds[row, col], .7)

    def test_avoidance_audit_rejects_footprint_overlap_with_a_low_stone(self):
        from mujoco_navigation import obstacle_avoidance_audit
        config = configuration("ruins_reference")
        with tempfile.TemporaryDirectory() as output:
            model, data, _ = build_scene(config, "ruins_reference", output)
            probe = model.geom("navigation_envelope").id
            half_height = float(model.geom_size[probe, 2])
            trace = [(0, -.09835622475594855, -3.3079311188552056, -.02306246612090046)]
            result = obstacle_avoidance_audit(model, config, trace)
            self.assertFalse(result["passed"])
            self.assertGreater(result["monitored_rubble_geometries"], 0)
            self.assertEqual(float(model.geom_size[probe, 2]), half_height)
            self.assertTrue(obstacle_avoidance_audit(model, config, [(0, -5, -4, 0)])["passed"])

    def test_no_hazard_evidence_preserves_observed_free_space(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        observation = NavigationObservation(0, "lidar", [], True, "test", {})
        grid.integrate_terrain(observation, (0, 0, 0), (0, 0))
        self.assertTrue(np.all(grid.log_odds == -1))

    def test_continuous_support_can_clear_a_false_footprint_hazard_but_not_a_rock_top(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        x = grid.x_min + (np.arange(grid.width)+.5)*grid.resolution
        grid.height_m[:] = .1*x[None, :]
        grid.terrain_hazard[40, 44] = True
        grid.log_odds.fill(-1)
        grid._clear_continuous_surface((0, 0, 0), 0)
        self.assertFalse(grid.terrain_hazard[40, 44])
        grid.height_m.fill(0)
        grid.height_m[40, 44] = .1
        grid.obstacle_top_m[40, 44] = .1
        grid.log_odds[40, 44] = 3
        grid.terrain_hazard[40, 44] = True
        grid._clear_continuous_surface((0, 0, 0), 0)
        self.assertTrue(grid.terrain_hazard[40, 44])
        self.assertGreaterEqual(grid.log_odds[40, 44], .7)

    def test_mapping_clearance_covers_the_actual_body_envelope_height(self):
        config = configuration("ruins_reference")
        required = config["robot_position_m"][2] + .23 + .02
        self.assertGreaterEqual(config["navigation"]["body_clearance_height_m"] + 1e-9, required)

    def test_distant_unobserved_inflated_goal_explores_its_approach(self):
        grid = OccupancyGrid((6, 6), .05)
        grid.log_odds[40:80, 20:60] = -1
        col, row = grid.world_to_cell(1.25, 0)
        grid.log_odds[row, col] = 3
        plan = grid.plan_result((-1, 0), (1, 0), .34, .7)
        self.assertEqual(plan.status, "exploring")
        self.assertEqual(plan.goal_xy, (1, 0))

    def test_recovery_reverse_obeys_the_full_observed_body_envelope(self):
        config = configuration()
        config["navigation"]["recovery_rotate_s"] = 0
        grid = OccupancyGrid((4, 4), .05)
        grid.log_odds.fill(-1)
        col, row = grid.world_to_cell(-.325, 0)
        grid.log_odds[row, col] = 3
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "stalled"
        command = session._recover(0, VelocityCommand(0, 0, 0, .3))
        self.assertTrue(command.is_zero)
        self.assertTrue(any(event["event"] == "recovery_motion_blocked" for event in session.recovery_events))

    def test_measured_support_seeds_the_lidar_blind_spot_without_overwriting_heights(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        session = NavigationSession(config, (0, 0, 0), grid)
        session.motion = MotionObservation(0, 0, 0)
        session._mark_pose_cell_free()
        cell = grid.world_to_cell(.02, 0)
        self.assertTrue(session._support_continuous((.02, 0, 0), cell))
        center = grid.world_to_cell(0, 0)
        grid.height_m[center[1], center[0]] = .07
        session._mark_pose_cell_free()
        self.assertAlmostEqual(float(grid.height_m[center[1], center[0]]), .07)

    def test_planner_descends_by_the_continuous_side_of_a_support_edge(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        x = grid.x_min + (np.arange(grid.width) + .5) * grid.resolution
        grid.height_m.fill(0)
        grid.height_m[40:, :] = np.clip(.08 - .2 * np.maximum(0, x), 0, .08)
        grid.height_updated_s.fill(0)
        observation = NavigationObservation(0, "lidar", [], True, "test", {"measured_support_height_m": .08})
        grid.integrate_terrain(observation, (-.5, .5, 0), (0, 0))
        self.assertTrue(grid.height_discontinuities.any())
        self.assertFalse(grid.terrain_hazard.any())
        plan = grid.plan_result((-.5, .5), (-.5, -.5), .1, .7)
        self.assertEqual(plan.status, "path_found")
        self.assertGreater(max(point[0] for point in plan.path), .2)

    def test_stale_heights_cannot_create_a_new_ground_discontinuity(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.height_m.fill(0)
        grid.height_m[40, 40] = .06
        grid.height_updated_s.fill(-10)
        grid.height_updated_s[40, 40] = 0
        observation = NavigationObservation(0, "lidar", [], True, "test", {})
        grid.integrate_terrain(observation, (0, 0, 0), (0, 0))
        self.assertFalse(grid.terrain_hazard.any())

    def test_fresh_continuous_floor_clears_a_stale_height_hazard(self):
        config = configuration()
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        grid.height_m.fill(0)
        grid.terrain_hazard[40, 40] = True
        observation = NavigationObservation(0, "lidar", [], True, "test", {})
        grid.integrate_terrain(observation, (0, 0, 0), (0, 0))
        self.assertFalse(grid.terrain_hazard.any())

    def test_dead_end_hairpin_never_commands_a_standstill(self):
        """A ~180 degree turn must not freeze the robot.

        Regression for the dead_end_exit stall: the planned route left along the
        S-bend corridor behind the robot, and the linearised score preferred the
        zero-velocity candidate over every rotation, so the run ended in
        "stalled" after 30 s of (0, 0) commands.
        """
        config = configuration("ruins_reference")
        grid = TerrainGrid((12, 10), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (-.255, .655, -.635), grid)
        session.status = "navigating"
        session.settings["forward_only"] = True
        # Approach leg of the dead-end stage: the route first runs north then west.
        path = [(-.275, .675), (-.275, .725), (-.325, .825), (-.525, .825),
                (-.725, .775), (-.875, .675), (-1.075, -.075), (-1.125, -1.225)]
        session.plan = PlanResult("path_found", path, (-1.11, -1.25), (-1.11, -1.25))
        command = session._local_rollout(VelocityCommand(0, .4, 0, .3), 0)
        debug = session.local_rollout_debug
        self.assertFalse(command.is_zero, debug)
        # Either an in-place rotation or a reverse-and-steer escape is acceptable;
        # what must never happen is the (0, 0) standstill.
        self.assertGreater(abs(command.linear_x_m_s) + abs(command.angular_z_rad_s), 0)
        self.assertEqual(len(debug["scores"]), debug["candidate_count"])

    def test_rollout_rejects_rotation_inside_wall_clearance(self):
        """In-place rotation must not swing the 0.372 m corners into a wall.

        Ramp inner walls live at log-odds ~0.35, below the occupied threshold 0.7,
        so they never stop the footprint by themselves; the rotation clearance
        guard is what keeps a pivot away from them.
        """
        config = configuration("ruins_reference")
        grid = TerrainGrid((12, 10), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "navigating"
        # A wall-like cell 0.25 m from the body centre: outside the 0.22 m
        # half-width footprint but inside the 0.372 m corner radius.
        cell = grid.world_to_cell(.28, 0.03)
        grid.log_odds[cell[1], cell[0]] = 0.35
        # Route leaves behind the robot, so scoring would love to rotate in place.
        session.plan = PlanResult("path_found", [(0, 0), (-1, 0)], (-1, 0), (-1, 0))
        command = session._local_rollout(VelocityCommand(0, .4, 0, .3), 0)
        debug = session.local_rollout_debug
        self.assertGreater(debug["rejected"]["rotation_clearance"], 0)
        self.assertEqual(debug["standstill_escape"], 0)
        self.assertEqual(abs(command.angular_z_rad_s), 0.0)
        # ... and it must not back out either: the robot drives head-first only.
        self.assertGreaterEqual(command.linear_x_m_s, 0.0)

    def test_recovery_backs_out_instead_of_rotating_without_clearance(self):
        """A recovery pivot next to a wall-like cell must reverse, not clip it.

        Regression for the ramp regression: the recovery's own rotation, pointed
        at the planned route, hit `ramp_inner_edge_collision_0` because the body
        corners sweep out to 0.372 m while the wall stood only ~0.29 m away.
        """
        config = configuration("ruins_reference")
        grid = TerrainGrid((12, 10), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "recovering"
        cell = grid.world_to_cell(.28, 0.03)
        grid.log_odds[cell[1], cell[0]] = 0.35
        # Route leaves behind the robot: an unguarded recovery would rotate right
        # into the wall-like cell.
        session.plan = PlanResult("path_found", [(0, 0), (-1, 0)], (-1, 0), (-1, 0))
        zero = VelocityCommand(0, 0, 0, .3)
        command = session._recover(0.0, zero)
        self.assertIsNotNone(session.recovery)
        self.assertLess(command.linear_x_m_s, 0,
                        "expected reverse instead of rotation: %s" % command)
        self.assertEqual(command.angular_z_rad_s, 0.0)

    def test_rollout_drives_head_first_unless_a_checkpoint_asks_for_reverse(self):
        """Regression: 56% of sampled motion used to be tail-first.

        With negative velocities in the default candidate set the rollout backed
        up whenever the target sat behind the body.  Reverse must be an explicit
        primitive: a checkpoint with `reverse` or an escape when forward is
        infeasible.
        """
        config = configuration("ruins_reference")
        grid = TerrainGrid((6, 6), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "navigating"
        session.settings["reverse_only"] = False
        session.settings["forward_only"] = False
        # Target straight behind the robot: the tempting answer is to back up.
        session.plan = PlanResult("path_found", [(0, 0), (-1.2, 0)], (-1.2, 0), (-1.2, 0))
        command = session._local_rollout(VelocityCommand(0, .4, 0, .3), 0)
        self.assertGreaterEqual(command.linear_x_m_s, 0.0, session.local_rollout_debug)
        self.assertFalse(session.local_rollout_debug["reverse_used"])
        # A reverse checkpoint still drives backwards.
        session.settings["reverse_only"] = True
        session.plan = PlanResult("path_found", [(0, 0), (-1.2, 0)], (-1.2, 0), (-1.2, 0))
        command = session._local_rollout(VelocityCommand(0, .4, 0, .3), 0)
        self.assertLess(command.linear_x_m_s, 0.0, session.local_rollout_debug)

    def test_forward_only_checkpoint_filters_reverse_commands(self):
        config = configuration("ruins_reference")
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "navigating"
        session.settings["forward_only"] = True
        session.plan = PlanResult("path_found", [(0, 0), (-1.5, 0)], (-1.5, 0), (-1.5, 0))
        session._local_rollout(VelocityCommand(.15, .4, 0, .3), 0)
        self.assertNotIn(-.08, session.local_rollout_debug["velocities"])
        session.plan = PlanResult("path_found", [(0, 0), (-.2, 0)], (-.2, 0), (-.2, 0))
        session.goal = (-.2, 0)
        session._local_rollout(VelocityCommand(.15, .4, 0, .3), 0)
        self.assertNotIn(-.08, session.local_rollout_debug["velocities"])

    def test_rollout_uses_a_configured_lookahead(self):
        config = configuration("ruins_reference")
        grid = TerrainGrid((4, 4), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "navigating"
        session.plan = PlanResult("path_found", [(0, 0), (0, 2)], (0, 2), (0, 2))
        session._local_rollout(VelocityCommand(.15, 0, 0, .3), 0)
        target = session.local_rollout_debug["target_xy_m"]
        lookahead = float(config["navigation"].get("rollout_lookahead_m", .45))
        self.assertAlmostEqual(session.plan.path[0][1] + target[1], min(2.0, lookahead), places=6)
        self.assertGreater(lookahead, float(config["navigation"].get("lookahead_m", .25)))

    def test_recovery_window_is_not_cancelled_by_the_next_tick(self):
        config = configuration("ruins_reference")
        grid = TerrainGrid((6, 6), .05, config["navigation"])
        grid.log_odds.fill(-1)
        session = NavigationSession(config, (0, 0, 0), grid)
        session.status = "recovering"
        session.plan = PlanResult("path_found", [(0, 0), (1, 0)], (1, 0), (1, 0))
        motion = MotionObservation(0, 0, 0)
        zero = VelocityCommand(0, 0, 0, .3)
        session._recover(0.0, zero)
        self.assertIsNotNone(session.recovery)
        for step in range(1, 12):
            now = step * .05
            session.propagate(MotionObservation(now, 0, 0), .05)
            command = session.tick(now, MotionObservation(now, 0, 0))
            self.assertIsNotNone(session.recovery)
            self.assertFalse(command.is_zero, "recovery pulse died at step %d" % step)


    def test_rotation_and_replanning_preserve_progress_deadline(self):
        for alternating in (False, True):
            controller = NavigationController(configuration()["navigation"])
            controller.settings["stuck_timeout_s"] = 1.0
            for step in range(11):
                yaw = (-1 if alternating and step % 2 else 1) * step * .2
                # Replanned route changes arc origin on every observation.
                path = [(-step * .01, 0), (1, 0)]
                stalled = controller.check_progress(step * .1, (0, 0, yaw), path)
                self.assertEqual(stalled, step == 10)
            self.assertAlmostEqual(controller.progress_debug["no_progress_s"], 1)
            self.assertEqual(controller.progress_debug["net_displacement_m"], 0)

    def test_replay_applies_external_collision_stop_on_the_following_tick(self):
        from tools.replay_navigation import replay
        config = configuration()
        with tempfile.TemporaryDirectory() as output:
            directory = Path(output)
            (directory / "config_snapshot.json").write_text(json.dumps(config))
            (directory / "sensor_observations.jsonl").write_text("")
            (directory / "collision_events.json").write_text(json.dumps([{"timestamp_s": 0}]))
            records = [{"timestamp_s": now, "dt_s": .05, "motion": vars(MotionObservation(now, 0, 0)),
                        "cancelled": False, "command": [0, 0], "status": status}
                       for now, status in ((0, "mapping"), (.05, "collision"))]
            (directory / "navigation_inputs.jsonl").write_text(
                "\n".join(json.dumps(record) for record in records))
            result, session = replay(directory)
            self.assertTrue(result["passed"])
            self.assertEqual(session.status, "collision")

    def test_measured_displacement_and_path_arc_refresh_progress(self):
        controller = NavigationController(configuration()["navigation"])
        path = [(0, 0), (.04, 0), (.04, .04)]
        controller.check_progress(0, (0, 0, 0), path)
        controller.check_progress(.5, (.04, .04, 0), path)
        # Arc travel exceeds .06 m while net displacement remains below it.
        self.assertEqual(controller.progress_debug["reset_reason"], "measured_progress")
        self.assertEqual(controller._last_progress_time, .5)
        controller.check_progress(1, (.12, .04, 0), path)
        self.assertEqual(controller._last_progress_time, 1)

    def test_rotating_frontier_times_out_and_stays_excluded_after_recovery(self):
        config = configuration()
        config["navigation"].update(exploration_mode=True, initial_mapping_duration_s=0, stuck_timeout_s=.5,
                                    recovery_rotate_s=.1, recovery_reverse_s=.1,
                                    recovery_settle_s=.1, recovery_attempt_limit=2)
        grid = OccupancyGrid((6, 6), .05)
        grid.log_odds[40:80, 40:80] = -1
        grid.scan_count = 3
        session = NavigationSession(config, (0, 0, 0), grid)
        session._replan(0)
        target = session.plan.target_xy
        self.assertEqual(session.plan.status, "exploring")
        for step in range(6):
            now = step * .1
            session.pose[2] = (-1) ** step * .4
            session.last_valid_motion = now
            session.last_observation = NavigationObservation(now, "lidar", [], True, "test")
            session.tick(now, MotionObservation(now, 0, .4))
        self.assertEqual(session.status, "recovering")
        self.assertIsNone(session.escape_hold)
        col, row = grid.world_to_cell(*target)
        self.assertGreater(grid.frontier_visits[row, col], 0)
        event = next(e for e in session.recovery_events if e["event"] == "frontier_invalidated")
        self.assertEqual(event["invalidated_target_xy_m"], target)
        self.assertGreaterEqual(event["no_progress_s"], .5)
        session.last_valid_motion = .81
        session.tick(.81, MotionObservation(.81, 0, 0))
        self.assertIsNone(session.recovery)
        self.assertEqual(session.navigator.progress_debug["reset_reason"], "recovery_window_completed")
        session._replan(.82)
        self.assertNotEqual(session.plan.target_xy, target)
        # Exhausting alternatives must never fall back to a failed frontier.
        grid.frontier_visits.fill(2)
        session._replan(.83)
        self.assertIsNone(session.plan.target_xy)
        self.assertEqual(session.plan.reason, "no_reachable_known_free_frontier")

    def test_recovery_rotation_keeps_counting_and_attempts_are_bounded(self):
        config = configuration()
        config["navigation"].update(initial_mapping_duration_s=0, stuck_timeout_s=.2,
                                    recovery_rotate_s=.3, recovery_reverse_s=0,
                                    recovery_settle_s=.1, recovery_attempt_limit=2)
        session = NavigationSession(config, (0, 0, 0))
        session.plan = PlanResult("start_blocked")
        session.status = "start_blocked"
        zero = VelocityCommand(0, 0, 0, .3)
        session._recover(0, zero)
        for step in range(1, 4):
            now = step * .1
            session.last_valid_motion = now
            session.tick(now, MotionObservation(now, 0, .4))
        self.assertGreaterEqual(session.navigator.progress_debug["no_progress_s"], .2)
        self.assertTrue(any(e.get("timeout_reason") == "no_position_or_path_progress"
                            for e in session.recovery_events))
        session._recover(.41, zero)
        session.status = "start_blocked"
        session._recover(.42, zero)
        session._recover(.83, zero)
        self.assertTrue(session.recovery_exhausted)
        self.assertEqual(session.recoveries, 2)


if __name__ == "__main__":
    unittest.main()
