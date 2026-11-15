"""Exercise actual frontier choices across arrival and observation updates."""
import math

from host_ui.app.explore import ExplorationPlanner
from host_ui.app.types import GridMap


def room_grid():
    grid = GridMap(80, 60, 0.05)
    for y in range(10, 50):
        for x in range(10, 70):
            grid.set(x, y, 0)
    return grid


def test_arrival_excludes_same_target_until_map_content_changes():
    grid = room_grid()
    explorer = ExplorationPlanner({})
    start = grid.cell_center(39, 29)
    goal = explorer.next_goal(grid, start, 0)
    assert goal is not None
    assert explorer.mark_visited(*goal, grid)

    # At arrival the old planner selected this exact same point indefinitely.
    assert explorer.next_goal(grid, goal, 1) is None
    assert explorer.stats()["decision_reason"] == "waiting_for_map"
    assert explorer.stats()["visited"] == 1

    # ROS replaces the GridMap object on every message; duplicates are not progress.
    duplicate = GridMap(grid.width, grid.height, grid.resolution,
                        origin=grid.origin, data=grid.data, name=grid.name)
    assert explorer.next_goal(duplicate, goal, 2) is None
    assert explorer.stats()["decision_reason"] == "waiting_for_map"

    # Newly observed cells permit another evaluation even when mutated in place.
    grid.set(39, 9, 0)
    assert explorer.next_goal(grid, goal, 3) is not None
    assert explorer.stats()["visited"] == 0


def test_visited_candidate_does_not_hide_another_observation_target():
    grid = GridMap(100, 60, 0.05)
    # Two open doorways on one connected known room.
    for y in range(10, 50):
        for x in range(10, 90):
            grid.set(x, y, 0)
    for x in range(9, 91):
        grid.set(x, 9, 100)
        grid.set(x, 50, 100)
    for y in range(9, 51):
        grid.set(9, y, 100)
        grid.set(90, y, 100)
    for x in (*range(20, 35), *range(65, 80)):
        grid.set(x, 50, 0)
    explorer = ExplorationPlanner({})
    start = grid.cell_center(49, 29)
    first = explorer.next_goal(grid, start, 0)
    assert first is not None
    assert explorer.mark_visited(*first, grid)
    second = explorer.next_goal(grid, start, 1)
    assert second is not None
    assert math.dist(first, second) > explorer.cfg["blacklist_radius_m"]
    assert explorer.stats()["decision_reason"] == "goal_selected"


def test_no_candidates_is_distinct_from_no_frontiers():
    grid = room_grid()
    explorer = ExplorationPlanner({"max_backoff_m": 0.05})
    assert explorer.next_goal(grid, grid.cell_center(39, 29), 0) is None
    assert explorer.stats()["raw_frontiers"] == 1
    assert explorer.stats()["decision_reason"] == "no_candidates"

    known = GridMap(40, 40, 0.05, data=bytes(40 * 40))
    assert explorer.next_goal(known, (1, 1), 1) is None
    assert explorer.stats()["raw_frontiers"] == 0
    assert explorer.stats()["decision_reason"] == "no_frontiers"


def test_small_frontier_filtered_out_is_not_reported_as_absent():
    grid = GridMap(40, 40, 0.05)
    grid.set(10, 10, 0)
    explorer = ExplorationPlanner({})
    assert explorer.next_goal(grid, (0.525, 0.525), 0) is None
    assert explorer.stats()["frontiers"] == 0
    assert explorer.stats()["raw_frontiers"] == 1
    assert explorer.stats()["decision_reason"] == "no_candidates"


def test_reset_clears_visited_points_and_rejects_invalid_arrivals():
    grid = room_grid()
    explorer = ExplorationPlanner({})
    start = grid.cell_center(39, 29)
    goal = explorer.next_goal(grid, start, 0)
    assert goal is not None
    assert not explorer.mark_visited(float("nan"), goal[1], grid)
    assert not explorer.mark_visited("bad", goal[1], grid)
    assert explorer.stats()["visited"] == 0
    assert explorer.mark_visited(*goal, grid)
    explorer.reset()
    assert explorer.stats()["visited"] == 0
    assert explorer.next_goal(grid, goal, 1) is not None
