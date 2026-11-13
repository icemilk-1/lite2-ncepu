from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "sensor_capture"))

from navigation_core.contracts import NavigationObservation, VelocityCommand
from navigation_core.control import NavigationController
from navigation_core.evaluation import NavigationEvaluator
from navigation_core.execution_adapters.ros1_twist import Ros1TwistExecutionAdapter
from navigation_core.grid import OccupancyGrid


def make_grid(field_size=(8.0, 8.0), resolution=0.1, value=-1.0):
    grid = OccupancyGrid(field_size, resolution)
    grid.log_odds.fill(value)
    return grid


def plan(grid, start, goal, radius=0.0, preferred=0.0):
    return grid.plan_result(start, goal, radius, 0.7, 2.8, preferred)


class NavigationCoreTests(unittest.TestCase):
    def test_scan_updates_each_cell_once_and_preserves_hit_endpoints(self):
        grid = OccupancyGrid((10.0, 10.0), 0.1)
        grid.integrate(
            [(2.0, 0.0, True), (2.0, 0.01, True), (2.0, 0.02, False)],
            (0.0, 0.0, 0.0), (0.0, 0.0), 8.0, 0.08,
        )
        endpoint = grid.world_to_cell(2.0, 0.0)
        traversed = grid.world_to_cell(1.0, 0.0)
        self.assertAlmostEqual(grid.log_odds[endpoint[1], endpoint[0]], 0.9)
        self.assertAlmostEqual(grid.log_odds[traversed[1], traversed[0]], -0.35)

    def test_fully_unknown_map_reports_insufficient_observations(self):
        grid = make_grid(value=0.0)
        result = plan(grid, (-2.0, -2.0), (2.0, 2.0))
        self.assertEqual(result.status, "insufficient_observations")
        self.assertEqual(result.goal_xy, (2.0, 2.0))
        self.assertEqual(result.path, [])

    def test_unknown_goal_routes_only_to_reachable_known_frontier(self):
        grid = OccupancyGrid((8.0, 8.0), 0.1)
        grid.log_odds[38:42, 15:51] = -1.0
        result = plan(grid, (-2.0, 0.0), (2.0, 0.0))
        self.assertEqual(result.status, "exploring")
        self.assertEqual(result.goal_xy, (2.0, 0.0))
        self.assertTrue(result.path)
        self.assertIsNotNone(result.target_xy)
        for x, y in result.path:
            cell = grid.world_to_cell(x, y)
            self.assertLessEqual(grid.log_odds[cell[1], cell[0]], -0.18)
        for start, end in zip(result.path, result.path[1:]):
            a = grid.world_to_cell(*start)
            b = grid.world_to_cell(*end)
            for column, row in grid.line_cells(a, b):
                self.assertLessEqual(grid.log_odds[row, column], -0.18)

    def test_blocked_goal_is_not_silently_moved(self):
        grid = make_grid()
        gx, gy = grid.world_to_cell(0.05, 0.05)
        grid.log_odds[gy - 3:gy + 4, gx - 3:gx + 4] = 2.0
        result = plan(grid, (-2.0, 0.05), (0.05, 0.05), radius=0.1)
        self.assertEqual(result.status, "goal_blocked")
        self.assertEqual(result.goal_xy, (0.05, 0.05))
        self.assertIsNone(result.target_xy)

    def test_blocked_start_is_reported_separately(self):
        grid = make_grid()
        sx, sy = grid.world_to_cell(-2.0, 0.0)
        grid.log_odds[sy, sx] = 2.0
        result = plan(grid, (-2.0, 0.0), (2.0, 0.0))
        self.assertEqual(result.status, "start_blocked")

    def test_known_free_but_separated_goal_reports_disconnected(self):
        grid = make_grid()
        wall_x, _ = grid.world_to_cell(0.0, 0.0)
        grid.log_odds[:, wall_x] = 2.0
        result = plan(grid, (-2.0, 0.0), (2.0, 0.0), radius=0.1)
        self.assertEqual(result.status, "map_disconnected")

    def test_diagonal_corner_is_not_smoothed_through(self):
        grid = make_grid()
        blocked = grid.inflated(0.0, 0.7)
        a = grid.world_to_cell(-0.05, -0.05)
        b = grid.world_to_cell(0.05, 0.05)
        blocked[a[1], b[0]] = True
        self.assertTrue(grid.segment_blocked(grid.cell_to_world(*a), grid.cell_to_world(*b), blocked))
        corners = grid.line_cells(a, b)
        self.assertIn((b[0], a[1]), corners)

    def test_boundary_and_corridor_clearance(self):
        grid = OccupancyGrid((4.0, 4.0), 0.1)
        grid.log_odds[18:22, 2:38] = -1.0
        result = plan(grid, (-1.5, 0.0), (1.85, 0.0), radius=0.1)
        self.assertEqual(result.status, "exploring")
        edge = grid.world_to_cell(-1.95, 0.0)
        self.assertTrue(grid.inflated(0.1, 0.7)[edge[1], edge[0]])

    def test_wall_like_corridor_keeps_the_path_on_the_centre_line(self):
        """Sub-threshold wall cells must still centre the path.

        Terrain height jumps sit at log-odds ~0.35, below the 0.7 occupied
        threshold. The distance transform, the clearance cost and the shortcut
        band all had to learn about them; otherwise the smoothed path hugs a
        wall with only ~0.08 m of body clearance instead of running down the
        middle of the corridor.
        """
        grid = OccupancyGrid((8.0, 6.0), 0.05)
        grid.log_odds.fill(-0.7)
        for y in np.arange(-3.0, 3.0, 0.05):
            row = int((y + 3.0) / 0.05)
            if abs(y) >= 0.6:
                grid.log_odds[row, :] = 0.35
        result = grid.plan_result((-3.0, -0.35), (3.0, -0.35), 0.34, 0.7, 2.8,
                                  0.26, 0.60, 0.8, 0.2)
        self.assertEqual(result.status, "path_found")
        middle = [point for point in result.path if -2.0 <= point[0] <= 2.0]
        self.assertTrue(middle, result.path)
        self.assertLess(max(abs(point[1]) for point in middle), 0.15, result.path)

    def test_command_expiry_cancellation_and_estimated_pose_control(self):
        settings = {
            "command_timeout_s": 0.3, "observation_timeout_s": 0.36,
            "navigation_timeout_s": 30.0, "goal_tolerance_m": 0.1,
            "front_stop_angle_rad": 0.4, "front_stop_distance_m": 0.4,
            "max_linear_speed_m_s": 0.2, "max_yaw_speed_rad_s": 0.6,
            "stuck_timeout_s": 10.0, "lookahead_m": 0.2,
        }
        observation = NavigationObservation(0.0, "front_lidar", [], True, "test")
        controller = NavigationController(settings)
        result = plan(make_grid(), (-2.0, 0.0), (2.0, 0.0))
        # A controller accepts the estimated pose as its only pose input.
        decision = controller.update(0.0, (-2.0, 0.0, 0.0), result, observation, 0.0)
        self.assertEqual(decision.status, "navigating")
        self.assertGreater(decision.command.linear_x_m_s, 0.0)
        self.assertTrue(decision.command.expired(0.31))
        stale = controller.update(0.5, (-2.0, 0.0, 0.0), result, observation, 0.5)
        self.assertEqual(stale.status, "stale_observation")
        controller.cancel()
        cancelled = controller.update(0.6, (-2.0, 0.0, 0.0), result,
                                      NavigationObservation(0.6, "front_lidar", [], True, "test"), 0.6)
        self.assertEqual(cancelled.status, "cancelled")
        self.assertTrue(cancelled.command.is_zero)

    def test_stall_is_measured_from_estimated_pose(self):
        settings = {
            "command_timeout_s": 0.3, "observation_timeout_s": 0.36,
            "navigation_timeout_s": 30.0, "goal_tolerance_m": 0.1,
            "front_stop_angle_rad": 0.4, "front_stop_distance_m": 0.4,
            "max_linear_speed_m_s": 0.2, "max_yaw_speed_rad_s": 0.6,
            "stuck_timeout_s": 2.0, "lookahead_m": 0.2,
        }
        controller = NavigationController(settings)
        observation = NavigationObservation(0.0, "front_lidar", [], True, "test")
        result = plan(make_grid(), (-2.0, 0.0), (2.0, 0.0))
        controller.update(0.0, (-2.0, 0.0, 0.0), result, observation, 0.0)
        stalled = controller.update(2.1, (-2.0, 0.0, 0.0), result,
                                    NavigationObservation(2.1, "front_lidar", [], True, "test"), 2.1)
        self.assertEqual(stalled.status, "stalled")
        self.assertTrue(stalled.command.is_zero)

    def test_truth_evaluation_is_separate_from_navigation_estimate(self):
        evaluator = NavigationEvaluator((0.0, 0.0, 0.0))
        evaluator.record_execution((0.8, 0.0, 0.0), 0.8, False)
        report = evaluator.summary((1.0, 0.0), 0, 0.25)
        self.assertAlmostEqual(report["goal_error_m"], 0.2)
        self.assertTrue(report["truth_goal_tolerance_met"])

    def test_ros1_adapter_requires_ownership_and_mode_and_enforces_watchdog(self):
        messages = []

        class Publisher(object):
            def publish(self, message):
                messages.append(message)

        factory = lambda: SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )
        with self.assertRaises(RuntimeError):
            Ros1TwistExecutionAdapter(Publisher(), factory, "/cmd_vel", 0.15, 0.4, 0.3)
        adapter = Ros1TwistExecutionAdapter(
            Publisher(), factory, "/cmd_vel", 0.15, 0.4, 0.15,
            single_writer_verified=True, mode_ready=lambda: True,
        )
        command = VelocityCommand(0.5, -1.0, 0.0, 0.3)
        self.assertTrue(adapter.publish(command, 0.0))
        self.assertEqual(messages[-1].linear.x, 0.15)
        self.assertEqual(messages[-1].linear.y, 0.0)
        self.assertEqual(messages[-1].angular.z, -0.4)
        self.assertTrue(adapter.watchdog(0.15))
        self.assertEqual(messages[-1].linear.x, 0.0)
        adapter.cancel(0.2)
        self.assertFalse(adapter.publish(command, 0.21))
        self.assertEqual(messages[-1].angular.z, 0.0)
        not_ready = Ros1TwistExecutionAdapter(
            Publisher(), factory, "/cmd_vel", 0.15, 0.4, 0.15,
            single_writer_verified=True, mode_ready=lambda: False,
        )
        self.assertFalse(not_ready.publish(command, 0.22))
        self.assertEqual(messages[-1].linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
