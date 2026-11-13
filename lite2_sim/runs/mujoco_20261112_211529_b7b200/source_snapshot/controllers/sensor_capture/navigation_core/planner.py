"""Known-free A* planning with reachable-frontier exploration."""

import heapq
import math

import numpy as np


class PlanResult(object):
    __slots__ = ("status", "path", "goal_xy", "target_xy", "reason")

    def __init__(self, status, path=None, goal_xy=None, target_xy=None, reason=None):
        self.status = str(status)
        self.path = list(path or [])
        self.goal_xy = tuple(goal_xy) if goal_xy is not None else None
        self.target_xy = tuple(target_xy) if target_xy is not None else None
        self.reason = reason or self.status

    @property
    def has_path(self):
        return bool(self.path)


def plan_grid(grid, start, goal, radius_m, occupied_threshold, unknown_cost=2.8, preferred_clearance_m=0.0):
    """Plan to a known goal, or to a reachable frontier without changing the goal.

    ``goal`` may be ``None`` for pure mapping, in which case only the
    reachable-frontier branch runs and every result keeps ``goal_xy`` unset.
    """
    start_xy = (float(start[0]), float(start[1]))
    goal_xy = None if goal is None else (float(goal[0]), float(goal[1]))
    start_cell = grid.world_to_cell(*start_xy)
    if start_cell is None:
        return PlanResult("start_out_of_map", goal_xy=goal_xy)
    goal_cell = None
    if goal_xy is not None:
        goal_cell = grid.world_to_cell(*goal_xy)
        if goal_cell is None:
            return PlanResult("goal_out_of_map", goal_xy=goal_xy)

    values = grid.log_odds
    known_free = values <= -0.18
    occupied = values >= occupied_threshold
    unknown = ~(known_free | occupied)
    blocked = grid.inflated(radius_m, occupied_threshold)
    preferred_blocked = grid.inflated(radius_m + preferred_clearance_m, occupied_threshold)
    sx, sy = start_cell

    if not known_free[sy, sx] and not occupied[sy, sx]:
        return PlanResult("insufficient_observations", goal_xy=goal_xy,
                          reason="start_pose_not_observed_free")
    if occupied[sy, sx] or blocked[sy, sx]:
        return PlanResult("start_blocked", goal_xy=goal_xy, reason="start_not_known_traversable")

    traversable = known_free & ~blocked
    smoothing_blocked = preferred_blocked | ~known_free
    if goal_xy is not None:
        gx, gy = goal_cell
        if occupied[gy, gx] or blocked[gy, gx]:
            return PlanResult("goal_blocked", goal_xy=goal_xy, reason="goal_occupied_or_clearance_blocked")
        if traversable[gy, gx]:
            path_cells, _, _ = _a_star(grid, start_cell, goal_cell, traversable, preferred_blocked)
            if path_cells is not None:
                path = _cells_to_world(grid, grid.smooth(path_cells, smoothing_blocked))
                if not path:
                    path = [start_xy]
                return PlanResult("path_found", path, goal_xy, goal_xy)

    # The exact target remains unchanged. Explore only free cells reachable
    # from the robot and select one bordering unobserved space.
    distances, parents = _reachable_costs(grid, start_cell, traversable, preferred_blocked)
    height, width = values.shape
    explorable_unknown = unknown & ~blocked
    frontier_band = explorable_unknown.copy()
    frontier_radius = getattr(grid, "frontier_radius_m", 0.0)
    if frontier_radius > 0:
        for _ in range(int(math.ceil(frontier_radius / grid.resolution))):
            expanded = frontier_band.copy()
            expanded[1:, :] |= frontier_band[:-1, :]
            expanded[:-1, :] |= frontier_band[1:, :]
            expanded[:, 1:] |= frontier_band[:, :-1]
            expanded[:, :-1] |= frontier_band[:, 1:]
            frontier_band |= expanded & ~blocked
    visited = getattr(grid, "frontier_visits", None)
    rearward_penalty = float(getattr(grid, "frontier_rearward_penalty", 0.0))
    heading = getattr(grid, "frontier_heading_rad", None)
    frontier_cells = []
    minimum_distance = float(getattr(grid, "frontier_min_distance_m", 0.55))
    for (column, row), cost in distances.items():
        if not (frontier_band[row, column] if frontier_radius > 0 else _touches_unknown(explorable_unknown, column, row, width, height)):
            continue
        frontier_xy = grid.cell_to_world(column, row)
        score = cost * grid.resolution
        if goal_xy is not None:
            remaining = math.hypot(goal_xy[0] - frontier_xy[0], goal_xy[1] - frontier_xy[1])
            remaining_weight = max(1.0, min(2.5, float(unknown_cost) / 1.75))
            score += remaining_weight * remaining
        if visited is not None:
            score += 3.0 * min(float(visited[row, column]), 3.0)
        if heading is not None and rearward_penalty > 0:
            facing = math.cos(math.atan2(frontier_xy[1] - start_xy[1],
                                         frontier_xy[0] - start_xy[0]) - heading)
            if facing < 0.0:
                # Keep exploring forward while forward options exist, but allow a
                # rearward frontier once the front is exhausted.
                score += rearward_penalty * (-facing)
        frontier_cells.append((score, cost, row, column))
    if not frontier_cells:
        if goal_xy is not None and traversable[goal_cell[1], goal_cell[0]]:
            return PlanResult("map_disconnected", goal_xy=goal_xy, reason="known_goal_not_reachable")
        return PlanResult("insufficient_observations", goal_xy=goal_xy,
                          reason="no_reachable_known_free_frontier")
    # The lidar blind spot under the robot is permanently unobserved, so a
    # frontier right next to the body would be re-selected forever and turn the
    # robot into a spinner. Prefer frontiers that need real travel.
    distant = [cell for cell in frontier_cells if cell[1] * grid.resolution >= minimum_distance]
    _, _, row, column = min(distant or frontier_cells)
    frontier = (column, row)
    path_cells = _reconstruct(parents, start_cell, frontier)
    path = _cells_to_world(grid, grid.smooth(path_cells, smoothing_blocked))
    if not path:
        path = [start_xy]
    target_xy = grid.cell_to_world(column, row)
    return PlanResult("exploring", path, goal_xy, target_xy,
                      reason="goal_not_observed_reachable_frontier_selected")


def _neighbors(cell, width, height, traversable):
    x, y = cell
    for dx, dy, step in ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                         (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
                         (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))):
        nx, ny = x + dx, y + dy
        if not (0 <= nx < width and 0 <= ny < height) or not traversable[ny, nx]:
            continue
        if dx and dy and (not traversable[y, nx] or not traversable[ny, x]):
            continue
        yield (nx, ny), step


def _a_star(grid, start, goal, traversable, preferred_blocked):
    height, width = traversable.shape

    def heuristic(cell):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    frontier = [(heuristic(start), 0.0, start)]
    parents = {}
    costs = {start: 0.0}
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if current == goal:
            return _reconstruct(parents, start, goal), parents, costs
        if cost > costs.get(current, math.inf):
            continue
        for nxt, step in _neighbors(current, width, height, traversable):
            extra = 2.0 if preferred_blocked[nxt[1], nxt[0]] else 0.0
            next_cost = cost + step * (1.0 + extra)
            if next_cost < costs.get(nxt, math.inf):
                costs[nxt] = next_cost
                parents[nxt] = current
                heapq.heappush(frontier, (next_cost + heuristic(nxt), next_cost, nxt))
    return None, parents, costs


def _reachable_costs(grid, start, traversable, preferred_blocked):
    height, width = traversable.shape
    frontier = [(0.0, start)]
    parents = {}
    costs = {start: 0.0}
    while frontier:
        cost, current = heapq.heappop(frontier)
        if cost > costs.get(current, math.inf):
            continue
        for nxt, step in _neighbors(current, width, height, traversable):
            extra = 2.0 if preferred_blocked[nxt[1], nxt[0]] else 0.0
            next_cost = cost + step * (1.0 + extra)
            if next_cost < costs.get(nxt, math.inf):
                costs[nxt] = next_cost
                parents[nxt] = current
                heapq.heappush(frontier, (next_cost, nxt))
    return costs, parents


def _touches_unknown(unknown, column, row, width, height):
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        x, y = column + dx, row + dy
        if 0 <= x < width and 0 <= y < height and unknown[y, x]:
            return True
    return False


def _reconstruct(parents, start, goal):
    cells = [goal]
    while cells[-1] != start:
        parent = parents.get(cells[-1])
        if parent is None:
            return []
        cells.append(parent)
    cells.reverse()
    return cells


def _cells_to_world(grid, cells):
    return [grid.cell_to_world(column, row) for column, row in cells]