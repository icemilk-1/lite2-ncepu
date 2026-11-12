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
            self.assertAlmostEqual(support[0], (2-.25)/3.5*.34, places=5)

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
        session = NavigationSession(config, (-.02, 0, 0), grid)
        session.motion = MotionObservation(0, 0, 0, ground_height_m=.3)
        session.status = "exploring"
        session.plan = PlanResult("exploring", [(-.02, 0), (1, 0)], (1, 0), (1, 0))
        command = session._local_rollout(VelocityCommand(.15, 0, 0, .3), 0)
        self.assertLessEqual(command.linear_x_m_s, .03)


if __name__ == "__main__":
    unittest.main()
