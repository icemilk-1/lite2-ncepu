"""导航几何、路径跟踪与地图更新的离线回归。"""
import math
import random

import pytest

from host_ui.app.nav import (
    PointNavigator, _build_blocked, _line_of_sight, _squared_distances, plan_path,
)
from host_ui.app.types import GridMap


def open_grid(width=100, height=100, resolution=0.1, origin=(-5.0, -5.0, 0.0)):
    return GridMap(width, height, resolution, origin=origin,
                   data=bytes(width * height))


def navigator(grid, start, goal, **config):
    nav = PointNavigator({"inflate_m": 0.0, **config}, {})
    ok, message = nav.set_goal(*goal, None, grid, start, now=0.0)
    assert ok, message
    return nav


def test_squared_distance_transform_matches_brute_force():
    randomizer = random.Random(42)
    for _ in range(30):
        values = [randomizer.choice([math.inf, 0.0, 1.0, 4.0]) for _ in range(20)]
        expected = [min(value + (index - source) ** 2
                        for source, value in enumerate(values))
                    for index in range(len(values))]
        assert _squared_distances(values) == expected


def test_euclidean_inflation_includes_diagonal_within_radius():
    grid = open_grid(11, 11, origin=(0.0, 0.0, 0.0))
    grid.set(5, 5, 100)
    blocked, width, _ = _build_blocked(grid, 0.30, False)
    assert blocked[7 * width + 7]  # sqrt(0.2² + 0.2²) < 0.30
    assert not blocked[8 * width + 6]  # sqrt(0.3² + 0.1²) > 0.30


def test_inflation_matches_nearest_obstacle_distance():
    grid = open_grid(13, 11, origin=(0.0, 0.0, 0.0))
    obstacles = [(1, 1), (8, 5), (12, 9)]
    for x, y in obstacles:
        grid.set(x, y, 100)
    blocked, width, height = _build_blocked(grid, 0.30, False)
    for y in range(height):
        for x in range(width):
            expected = min((x - ox) ** 2 + (y - oy) ** 2
                           for ox, oy in obstacles) <= 9
            assert bool(blocked[y * width + x]) == expected


def test_unknown_inflation_follows_allow_unknown():
    grid = open_grid(11, 11, origin=(0.0, 0.0, 0.0))
    grid.set(5, 5, -1)
    assert _build_blocked(grid, 0.2, False)[0][5 * 11 + 4]
    assert not any(_build_blocked(grid, 0.2, True)[0])


@pytest.mark.parametrize("origin", [(-3.0, -2.0, 0.0), (-3.0, -2.0, 37.0)])
def test_simplification_uses_translated_and_rotated_map(origin):
    grid = open_grid(30, 30, origin=origin)
    start = grid.cell_center(2, 2)
    goal = grid.cell_center(25, 25)
    result = plan_path(grid, start, goal, inflate_m=0.0)
    assert result.ok
    assert result.path == [start, goal]
    grid.set(14, 14, 100)
    result = plan_path(grid, start, goal, inflate_m=0.0)
    assert result.ok
    blocked, _, _ = _build_blocked(grid, 0.0, False)
    assert len(result.path) > 2
    assert all(_line_of_sight(grid, blocked, a, b)
               for a, b in zip(result.path, result.path[1:]))


@pytest.mark.parametrize("endpoint", ["start", "goal"])
def test_blocked_endpoint_is_rejected(endpoint):
    grid = open_grid()
    start, goal = (-1.0, 0.0), (1.0, 0.0)
    position = start if endpoint == "start" else goal
    grid.set(*grid.world_to_index(*position), 100)
    assert not plan_path(grid, start, goal, inflate_m=0.0).ok


@pytest.mark.parametrize("point", [(-6.0, 0.0), (math.nan, 0.0), (math.inf, 0.0)])
def test_out_of_map_and_invalid_endpoint_is_rejected(point):
    assert not plan_path(open_grid(), (0.0, 0.0), point, inflate_m=0.0).ok


def test_line_of_sight_checks_endpoints_and_corner_cells():
    grid = open_grid(5, 5, resolution=1.0, origin=(0.0, 0.0, 0.0))
    grid.set(1, 0, 100)
    blocked, _, _ = _build_blocked(grid, 0.0, False)
    assert not _line_of_sight(grid, blocked, (0.5, 0.5), (1.5, 1.5))
    assert not _line_of_sight(grid, blocked, (0.5, 0.5), (1.5, 0.5))


def test_tracking_advances_after_first_corner():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 0.0), (2.0, 2.0))
    nav._path = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (2.0, 2.0)]
    linear, angular = nav.update(1.0, (1.0, 1.0, 90.0), grid)
    assert linear > 0
    assert angular == pytest.approx(0.0)
    assert nav._path[0] == (1.0, 1.0)


def test_lookahead_is_interpolated_on_remaining_segment():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 0.0), (3.0, 0.0))
    linear, angular = nav.update(1.0, (1.5, 0.0, 0.0), grid)
    assert linear > 0
    assert angular == pytest.approx(0.0)
    assert nav.snapshot().progress == pytest.approx(0.5)


def test_watchdog_accepts_progress_away_from_goal_along_detour():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 180.0), (2.0, 2.0))
    nav._path = [(0.0, 0.0), (-1.0, 0.0), (-1.0, 2.0), (2.0, 2.0)]
    nav._last_progress_distance = 6.0
    nav.update(0.0, (0.0, 0.0, 180.0), grid)
    nav.update(7.0, (-0.6, 0.0, 180.0), grid)
    nav.update(14.0, (-1.0, 0.6, 90.0), grid)
    assert nav.state == "following"


def test_watchdog_stops_without_motion():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 0.0), (2.0, 0.0))
    nav.update(0.0, (0.0, 0.0, 0.0), grid)
    assert nav.update(9.0, (0.0, 0.0, 0.0), grid) == (0.0, 0.0)
    assert nav.state == "stuck"


def test_slow_turning_counts_as_progress():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 180.0), (2.0, 0.0))
    nav.update(0.0, (0.0, 0.0, 180.0), grid)
    for second in range(1, 20):
        nav.update(float(second), (0.0, 0.0, 180.0 - second * 5.0), grid)
    assert nav.state == "following"


def test_map_mutation_stops_and_replans_around_new_obstacle():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 0.0), (2.0, 0.0))
    nav.update(0.0, (0.0, 0.0, 0.0), grid)
    grid.set(*grid.world_to_index(1.0, 0.0), 100)
    assert nav.update(0.04, (0.0, 0.0, 0.0), grid) == (0.0, 0.0)
    assert nav.state == "following"
    assert len(nav._path) > 2
    blocked, _, _ = _build_blocked(grid, 0.0, False)
    assert all(_line_of_sight(grid, blocked, a, b)
               for a, b in zip(nav._path, nav._path[1:]))


def test_disconnected_map_cancels_navigation_with_zero():
    grid = open_grid()
    nav = navigator(grid, (-1.0, 0.0, 0.0), (1.0, 0.0))
    for y in range(grid.height):
        grid.set(50, y, 100)
    assert nav.update(0.04, (-1.0, 0.0, 0.0), grid) == (0.0, 0.0)
    assert nav.state == "blocked"
    assert not nav.snapshot().active


def test_short_horizon_checks_actual_heading():
    grid = open_grid()
    nav = navigator(grid, (0.0, 0.0, 0.0), (2.0, 0.0))
    grid.set(*grid.world_to_index(0.0, 0.1), 100)
    blocked, _, _ = _build_blocked(grid, 0.0, False)
    assert nav._safe_command(grid, blocked, (0.0, 0.0, 90.0), 0.18, 0.0) == (0.0, 0.0)


def test_same_cell_arrival_and_yaw_alignment():
    grid = open_grid()
    nav = PointNavigator({"inflate_m": 0.0}, {})
    assert nav.set_goal(0.01, 0.01, 90.0, grid, (0.0, 0.0, 0.0), now=0.0)[0]
    linear, angular = nav.update(0.04, (0.0, 0.0, 0.0), grid)
    assert linear == 0 and angular > 0
    assert nav.update(1.0, (0.0, 0.0, 90.0), grid) == (0.0, 0.0)
    assert nav.state == "arrived"
    assert nav.snapshot().progress == 1.0


def test_complete_detour_with_kinematic_feedback():
    grid = open_grid(40, 40, origin=(-2.0, -2.0, 0.0))
    for y in range(28):
        grid.set(20, y, 100)
    pose = (-1.0, 0.0, 0.0)
    nav = navigator(grid, pose, (1.0, 0.0), inflate_m=0.15, max_speed_m_s=0.18)
    blocked, _, _ = _build_blocked(grid, 0.15, False)
    for step in range(2400):
        command = nav.update(step * 0.05, pose, grid)
        if nav.state == "arrived":
            break
        assert nav.snapshot().active, nav.snapshot().message
        linear, angular = command
        yaw = math.radians(pose[2])
        pose = (pose[0] + linear * 0.05 * math.cos(yaw + angular * 0.025),
                pose[1] + linear * 0.05 * math.sin(yaw + angular * 0.025),
                pose[2] + math.degrees(angular * 0.05))
        gx, gy = grid.world_to_index(*pose[:2])
        assert not blocked[gy * grid.width + gx]
    assert nav.state == "arrived", nav.snapshot().as_dict()
