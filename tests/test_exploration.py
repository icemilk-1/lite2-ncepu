"""Regressions for bounded recovery and pure exploration mode."""

import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers/sensor_capture"))

from mujoco_navigation import configure_navigation
from navigation_core.contracts import MotionObservation, NavigationObservation
from navigation_core.grid import OccupancyGrid
from navigation_core.planner import plan_grid
from navigation_core.session import (NavigationSession, RECOVERABLE_STATES,
                                     TERMINAL_STATES)
from navigation_core.terrain import TerrainGrid


def configuration(scene="ruins_reference"):
    config = json.loads((PROJECT / "config" / (scene + ".json")).read_text(encoding="utf-8"))
    return configure_navigation(config, scene)


def open_grid(config, size=(12, 10)):
    grid = TerrainGrid(size, config["navigation"]["grid_resolution_m"], config["navigation"])
    grid.log_odds.fill(-1.0)
    grid.surface_z.fill(np.nan)
    grid.scan_count = 6
    return grid


def observation(now):
    return NavigationObservation(now, "front_lidar_level", [], True, "test_projection", {})


def advance(session, now, dt=.1):
    motion = MotionObservation(now, 0.0, 0.0)
    session.propagate(motion, dt)
    session.last_observation = observation(now)
    return session.tick(now, motion)


def start(session, now=0.0):
    session.started_at = now
    session.navigator.reset_progress(now, session.pose, session.plan.path, "test_setup")
    return session


class ExplorationPlannerTests(unittest.TestCase):
    def test_goal_none_selects_a_frontier_and_keeps_the_goal_unset(self):
        grid = OccupancyGrid((6, 6), .1)
        grid.log_odds[:, :] = -1.0
        grid.log_odds[:, 40:] = 0.0
        grid.frontier_radius_m = .2
        result = plan_grid(grid, (-1.0, 0.0), None, .1, .7)
        self.assertEqual(result.status, "exploring")
        self.assertIsNone(result.goal_xy)
        self.assertIsNotNone(result.target_xy)
        self.assertGreater(len(result.path), 1)

    def test_a_fully_known_map_has_no_frontier(self):
        grid = OccupancyGrid((4, 4), .1)
        grid.log_odds[:, :30] = -1.0
        grid.log_odds[:, 30:] = 3.0
        grid.frontier_radius_m = .2
        result = plan_grid(grid, (-1.0, 0.0), None, .1, .7)
        self.assertEqual(result.status, "insufficient_observations")
        self.assertEqual(result.reason, "no_reachable_known_free_frontier")
        self.assertIsNone(result.goal_xy)

    def test_frontier_does_not_prefer_the_lidar_blind_spot(self):
        grid = OccupancyGrid((5, 5), .1)
        grid.log_odds[:, :] = -1.0
        col, row = grid.world_to_cell(-1.0, 0.0)
        # Unobserved lidar blind spot just ahead of the body, plus the robot cell.
        grid.log_odds[row:row + 2, col:col + 2] = 0.0
        grid.log_odds[row, col] = -1.0
        grid.log_odds[:, 45:] = 0.0
        grid.frontier_radius_m = .2
        result = plan_grid(grid, (-1.0, 0.0), None, .1, .7)
        self.assertEqual(result.status, "exploring")
        self.assertGreater(math.hypot(result.target_xy[0] + 1.0, result.target_xy[1]), .3)

    def test_rearward_frontier_loses_while_forward_space_remains(self):
        grid = OccupancyGrid((5, 5), .1)
        grid.log_odds[:, :] = -1.0
        grid.log_odds[:, :4] = 0.0
        grid.log_odds[:, 42:] = 0.0
        grid.frontier_radius_m = .2
        grid.frontier_heading_rad = 0.0
        grid.frontier_rearward_penalty = 3.0
        result = plan_grid(grid, (-1.0, 0.0), None, .1, .7)
        self.assertGreater(result.target_xy[0], -1.0)


class ExplorationSessionTests(unittest.TestCase):
    def test_exploration_mode_has_no_goal_and_reports_map_statistics(self):
        config = configuration()
        config["navigation"]["exploration_mode"] = True
        session = NavigationSession(config, (-4.0, -3.0, 0.0))
        self.assertIsNone(session.goal)
        summary = session.exploration_summary()
        self.assertEqual(set(summary), {"known_free_cells", "occupied_cells", "unknown_cells",
                                        "frontiers_visited", "distance_m", "scan_count",
                                        "recoveries"})
        self.assertEqual(summary["frontiers_visited"], 0)
        self.assertEqual(summary["distance_m"], 0.0)
        self.assertEqual(summary["scan_count"], 0)

    def test_exploration_finishes_only_when_no_frontier_remains(self):
        config = configuration()
        config["navigation"]["exploration_mode"] = True
        config["navigation"]["exploration_settle_s"] = 1.0
        grid = open_grid(config)
        col, row = grid.world_to_cell(0.0, 0.0)
        grid.log_odds[:, :] = 3.0
        grid.log_odds[row - 12:row + 12, col - 12:col + 12] = -1.0
        grid.frontier_radius_m = .2
        session = start(NavigationSession(config, (0.0, 0.0, 0.0), grid))
        command = None
        for step in range(60):
            command = advance(session, step * .1)
            if session.status == "map_complete":
                break
        self.assertEqual(session.status, "map_complete")
        self.assertTrue(session.finished)
        self.assertTrue(command.is_zero)
        self.assertIn("map_complete", TERMINAL_STATES)

    def test_recovery_waits_for_a_clear_body_before_rotating(self):
        config = configuration()
        config["navigation"]["recovery_rotate_s"] = .3
        config["navigation"]["recovery_reverse_s"] = .2
        config["navigation"]["recovery_settle_s"] = .1
        config["navigation"]["recovery_attempt_limit"] = 2
        grid = open_grid(config)
        pose = (-1.0, 0.0, 0.0)
        col, row = grid.world_to_cell(*pose[:2])
        grid.log_odds[row, col] = 4.0
        session = start(NavigationSession(config, pose, grid))
        commands = [advance(session, 2.1 + step * .1) for step in range(6)]
        self.assertEqual(session.status, "recovering")
        self.assertTrue(all(command.is_zero for command in commands))
        self.assertTrue(any(event["event"] == "recovery_motion_blocked" for event in session.recovery_events))
        self.assertTrue(any(event["event"] == "recovery_started" for event in session.recovery_events))

    def test_a_blocked_start_is_terminal_after_the_attempt_limit(self):
        config = configuration()
        config["navigation"]["recovery_rotate_s"] = .1
        config["navigation"]["recovery_reverse_s"] = .1
        config["navigation"]["recovery_settle_s"] = .1
        config["navigation"]["recovery_attempt_limit"] = 2
        grid = open_grid(config)
        pose = (-1.0, 0.0, 0.0)
        col, row = grid.world_to_cell(*pose[:2])
        grid.log_odds[row, col] = 4.0
        session = NavigationSession(config, pose, grid)
        for step in range(120):
            advance(session, step * .1)
            if session.finished:
                break
        self.assertEqual(session.status, "start_blocked")
        self.assertTrue(session.finished)
        events = [event["event"] for event in session.recovery_events]
        self.assertIn("recovery_started", events)
        self.assertIn("recovery_exhausted", events)
        self.assertLessEqual(session.recoveries, 2)

    def test_recovery_never_drives_forward(self):
        config = configuration()
        config["navigation"]["recovery_rotate_s"] = .3
        config["navigation"]["recovery_reverse_s"] = .5
        config["navigation"]["recovery_settle_s"] = .1
        grid = open_grid(config)
        pose = (-1.0, 0.0, 0.0)
        col, row = grid.world_to_cell(*pose[:2])
        grid.log_odds[row, col] = 4.0
        session = NavigationSession(config, pose, grid)
        for step in range(10):
            command = advance(session, step * .1)
            if session.status == "recovering":
                self.assertLessEqual(command.linear_x_m_s, 0.0)
        self.assertIn("start_blocked", RECOVERABLE_STATES)

    def test_exploration_mode_never_terminates_on_a_blocked_start(self):
        config = configuration()
        config["navigation"]["exploration_mode"] = True
        config["navigation"]["recovery_attempt_limit"] = 1
        config["navigation"]["exploration_settle_s"] = 100.0
        grid = open_grid(config)
        pose = (-1.0, 0.0, 0.0)
        col, row = grid.world_to_cell(*pose[:2])
        grid.log_odds[row, col] = 4.0
        session = NavigationSession(config, pose, grid)
        for step in range(60):
            advance(session, step * .1)
            self.assertNotEqual(session.status, "start_blocked")
            self.assertFalse(session.finished)

    def test_exploration_mode_counts_distance(self):
        config = configuration()
        config["navigation"]["exploration_mode"] = True
        grid = open_grid(config)
        session = NavigationSession(config, (0.0, 0.0, 0.0), grid)
        session.pose = [0.5, 0.0, 0.0]
        session.last_tick_pose = [0.0, 0.0]
        session.distance_m = 0.0
        advance(session, 1.0)
        self.assertGreater(session.distance_m, 0.4)


if __name__ == "__main__":
    unittest.main()