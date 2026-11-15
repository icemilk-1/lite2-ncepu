"""基于占据栅格的点到点导航。

定位/避障完全依赖外部（仿真或实机 SLAM/定位），本模块只做：

1. 在占据栅格上做带障碍膨胀的 A*（8 邻域、防切角），再用视线检查简化路径；
2. 纯追踪（pure pursuit）跟踪路径，输出 ``(linear, angular)``；
3. 到位判定与终点朝向对齐。

地图更新后重新检查剩余路径，控制输出经过短时轨迹碰撞检查。
地图与定位需要外部模块提供，速度由 SafetyCore 统一限幅与急停拦截。
"""
from __future__ import annotations

import heapq
import math
import threading
from dataclasses import dataclass, field
from .types import GridMap, clamp, wrap_deg


# --------------------------------------------------------------------------- 规划器
@dataclass
class PlanResult:
    ok: bool
    path: list[tuple[float, float]] = field(default_factory=list)
    reason: str = ""
    expanded: int = 0

    def as_dict(self) -> dict:
        return {
            "ok": self.ok,
            "reason": self.reason,
            "expanded": self.expanded,
            "points": len(self.path),
        }


def _squared_distances(values: list[float]) -> list[float]:
    """一维平方欧氏距离变换，计算 min_i(values[i] + (x-i)^2)。"""
    sites: list[int] = []
    boundaries: list[float] = []
    for index, value in enumerate(values):
        if math.isinf(value):
            continue
        boundary = -math.inf
        while sites:
            previous = sites[-1]
            boundary = ((value + index * index - values[previous] - previous * previous)
                        / (2 * (index - previous)))
            if boundary > boundaries[-1]:
                break
            sites.pop()
            boundaries.pop()
        if not sites:
            boundary = -math.inf
        sites.append(index)
        boundaries.append(boundary)
    if not sites:
        return [math.inf] * len(values)
    result = []
    site = 0
    for index in range(len(values)):
        while site + 1 < len(sites) and boundaries[site + 1] < index:
            site += 1
        source = sites[site]
        result.append(values[source] + (index - source) ** 2)
    return result


def _build_blocked(grid: GridMap, inflate_m: float,
                   allow_unknown: bool) -> tuple[bytearray, int, int]:
    """按欧氏距离膨胀占用栅格；未知区域按配置参与膨胀。"""
    width, height = grid.width, grid.height
    blocked = bytearray(width * height)
    for index, raw in enumerate(grid.data):
        value = raw - 256 if raw > 127 else raw
        blocked[index] = value >= 50 or (value < 0 and not allow_unknown)
    radius = max(0.0, float(inflate_m)) / grid.resolution
    if radius > 0:
        horizontal = []
        for row in range(height):
            horizontal.extend(_squared_distances([
                0.0 if cell else math.inf for cell in blocked[row * width:(row + 1) * width]
            ]))
        for column in range(width):
            distances = _squared_distances([
                horizontal[row * width + column] for row in range(height)
            ])
            for row, distance in enumerate(distances):
                blocked[row * width + column] = distance <= radius * radius + 1e-9
    return blocked, width, height


def _line_of_sight(grid: GridMap, blocked: bytearray,
                   a: tuple[float, float], b: tuple[float, float]) -> bool:
    """遍历线段穿过的栅格，包括端点和角点两侧，使用地图坐标变换。"""
    ax, ay = grid.world_to_cell(*a)
    bx, by = grid.world_to_cell(*b)
    gx, gy = math.floor(ax), math.floor(ay)
    end_x, end_y = math.floor(bx), math.floor(by)

    def open_cell(x, y):
        return grid.contains_cell(x, y) and not blocked[y * grid.width + x]

    if not open_cell(gx, gy) or not open_cell(end_x, end_y):
        return False
    dx, dy = bx - ax, by - ay
    step_x = 1 if dx > 0 else -1
    step_y = 1 if dy > 0 else -1
    delta_x = abs(1 / dx) if dx else math.inf
    delta_y = abs(1 / dy) if dy else math.inf
    next_x = ((gx + 1 - ax) if dx > 0 else (ax - gx)) * delta_x if dx else math.inf
    next_y = ((gy + 1 - ay) if dy > 0 else (ay - gy)) * delta_y if dy else math.inf
    while (gx, gy) != (end_x, end_y):
        if abs(next_x - next_y) < 1e-12:
            if not open_cell(gx + step_x, gy) or not open_cell(gx, gy + step_y):
                return False
            gx += step_x
            gy += step_y
            next_x += delta_x
            next_y += delta_y
        elif next_x < next_y:
            gx += step_x
            next_x += delta_x
        else:
            gy += step_y
            next_y += delta_y
        if not open_cell(gx, gy):
            return False
    return True


def plan_path(grid: GridMap | None, start_xy: tuple[float, float],
              goal_xy: tuple[float, float], inflate_m: float = 0.30,
              allow_unknown: bool = False, max_expand: int = 300000) -> PlanResult:
    """在栅格地图上规划路径，返回世界坐标点列（已平滑）。"""
    if grid is None:
        return PlanResult(False, reason="没有加载地图，无法规划")
    if not all(math.isfinite(value) for value in (*start_xy, *goal_xy)):
        return PlanResult(False, reason="起点或目标坐标无效")
    blocked, width, height = _build_blocked(grid, inflate_m, allow_unknown)
    sgx, sgy = grid.world_to_index(*start_xy)
    ggx, ggy = grid.world_to_index(*goal_xy)
    for label, gx, gy in (("起点", sgx, sgy), ("目标点", ggx, ggy)):
        if not grid.contains_cell(gx, gy):
            return PlanResult(False, reason=f"{label}超出地图范围")
        if blocked[gy * width + gx]:
            return PlanResult(False, reason=f"{label}位于障碍、未知区域或膨胀范围内")
    start = (sgx, sgy)
    goal = (ggx, ggy)

    start_index = start[1] * width + start[0]
    goal_index = goal[1] * width + goal[0]
    if start_index == goal_index:
        return PlanResult(True, [(float(start_xy[0]), float(start_xy[1])),
                                 (float(goal_xy[0]), float(goal_xy[1]))],
                          reason="起点与目标点重合", expanded=1)

    size = width * height
    g_score = [float("inf")] * size
    came_from = [-1] * size
    closed = bytearray(size)
    g_score[start_index] = 0.0

    def heuristic(index: int) -> float:
        x, y = index % width, index // width
        dx, dy = abs(x - goal[0]), abs(y - goal[1])
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    heap: list[tuple[float, int]] = [(heuristic(start_index), start_index)]
    steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
             (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
    expanded = 0
    found = False
    while heap:
        _, current = heapq.heappop(heap)
        if closed[current]:
            continue
        closed[current] = 1
        expanded += 1
        if current == goal_index:
            found = True
            break
        if expanded > max_expand:
            return PlanResult(False, reason="规划搜索超限，请检查地图或减小膨胀半径",
                              expanded=expanded)
        cx, cy = current % width, current // width
        for dx, dy, cost in steps:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            neighbour = ny * width + nx
            if closed[neighbour] or blocked[neighbour]:
                continue
            # 防切角：斜向移动要求两个正交邻居都可通行
            if dx and dy and (blocked[cy * width + nx] or blocked[ny * width + cx]):
                continue
            tentative = g_score[current] + cost
            if tentative < g_score[neighbour]:
                g_score[neighbour] = tentative
                came_from[neighbour] = current
                heapq.heappush(heap, (tentative + heuristic(neighbour), neighbour))
    if not found:
        return PlanResult(False, reason="A* 未找到可行路径", expanded=expanded)

    # 回溯
    cells: list[int] = [goal_index]
    cursor = goal_index
    while cursor != start_index:
        cursor = came_from[cursor]
        if cursor < 0:
            return PlanResult(False, reason="路径回溯失败", expanded=expanded)
        cells.append(cursor)
    cells.reverse()

    raw = [(grid.cell_center(cell % width, cell // width)) for cell in cells]
    # 保留端点所在栅格的中心，保证真实端点与栅格路径的连接也经过视线校验。
    raw.insert(0, (float(start_xy[0]), float(start_xy[1])))
    raw.append((float(goal_xy[0]), float(goal_xy[1])))

    # 视线简化
    simplified: list[tuple[float, float]] = [raw[0]]
    index = 0
    while index < len(raw) - 1:
        nxt = index + 1
        candidate = nxt
        while candidate + 1 < len(raw) and _line_of_sight(
                grid, blocked, raw[index], raw[candidate + 1]):
            candidate += 1
        simplified.append(raw[candidate])
        index = candidate

    return PlanResult(True, simplified, reason="路径规划成功", expanded=expanded)


def _path_length(path: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def _remaining_path(path: list[tuple[float, float]],
                    position: tuple[float, float]) -> list[tuple[float, float]]:
    """投影到最近的路径线段并裁剪已走部分。"""
    best_distance = math.inf
    best_index = 0
    projection = path[0]
    for index, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_squared = dx * dx + dy * dy
        fraction = 0.0
        if length_squared:
            fraction = clamp(((position[0] - a[0]) * dx + (position[1] - a[1]) * dy)
                             / length_squared, 0.0, 1.0)
        point = (a[0] + dx * fraction, a[1] + dy * fraction)
        distance = math.dist(position, point)
        if distance <= best_distance:
            best_distance = distance
            best_index = index
            projection = point
    return [projection, *path[best_index + 1:]]


def _point_along_path(path: list[tuple[float, float]], distance: float) -> tuple[float, float]:
    for a, b in zip(path, path[1:]):
        length = math.dist(a, b)
        if length >= distance and length > 0:
            fraction = distance / length
            return a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction
        distance -= length
    return path[-1]


# --------------------------------------------------------------------------- 跟踪器
@dataclass
class NavSnapshot:
    active: bool = False
    state: str = "idle"
    goal: list[float] | None = None
    path: list[list[float]] = field(default_factory=list)
    distance_m: float | None = None
    message: str = ""
    progress: float = 0.0
    planned_at: float = 0.0

    def as_dict(self) -> dict:
        return {
            "active": self.active,
            "state": self.state,
            "goal": self.goal,
            "path": self.path,
            "distance_m": self.distance_m,
            "message": self.message,
            "progress": self.progress,
            "planned_at": self.planned_at,
        }


class PointNavigator:
    """点到点路径跟踪（纯追踪 + 终点朝向对齐）。"""

    def __init__(self, nav_cfg: dict, limits: dict) -> None:
        self._lock = threading.RLock()
        self.nav = dict(nav_cfg)
        self.limits = dict(limits)
        self._state = "idle"
        self._goal: tuple[float, float, float | None] | None = None
        self._path: list[tuple[float, float]] = []
        self._distance: float | None = None
        self._message = ""
        self._planned_at = 0.0
        self._start_distance = 0.0
        self._last_progress_distance = float("inf")
        self._last_progress_time = 0.0
        self._last_heading_error = math.inf
        self._remaining_distance = 0.0
        self._grid: GridMap | None = None
        self._grid_signature = None
        self._blocked = bytearray()

    # ------------------------------------------------------------------ 参数
    def update_config(self, nav_cfg: dict, limits: dict) -> None:
        with self._lock:
            self.nav = dict(nav_cfg)
            self.limits = dict(limits)

    # ------------------------------------------------------------------ 指令
    def set_goal(self, x: float, y: float, yaw_deg: float | None,
                 grid: GridMap | None, pose: tuple[float, float, float],
                 now: float) -> tuple[bool, str]:
        nav = self.nav
        if not nav.get("enabled", True):
            return False, "导航功能已在配置中禁用"
        result = plan_path(grid, (pose[0], pose[1]), (float(x), float(y)),
                           inflate_m=float(nav.get("inflate_m", 0.30)),
                           allow_unknown=bool(nav.get("allow_unknown", False)))
        if not result.ok:
            return False, result.reason
        with self._lock:
            self._goal = (float(x), float(y), yaw_deg)
            self._path = result.path
            self._grid = grid
            self._state = "following"
            self._message = result.reason
            self._planned_at = now
            self._start_distance = _path_length(result.path)
            self._remaining_distance = self._start_distance
            self._distance = math.hypot(x - pose[0], y - pose[1])
            self._last_progress_distance = self._start_distance
            self._last_progress_time = now
            self._last_heading_error = math.inf
        return True, f"已规划 {len(result.path)} 个路径点"

    def cancel(self, message: str = "导航已取消") -> None:
        with self._lock:
            self._state = "cancelled" if self._state != "idle" else "idle"
            self._goal = None
            self._path = []
            self._distance = None
            self._message = message

    def finish(self, state: str, message: str) -> None:
        with self._lock:
            self._state = state
            self._message = message
            self._distance = 0.0 if state == "arrived" else self._distance
            self._goal = None
            self._path = []

    # ------------------------------------------------------------------ 跟踪
    def _blocked_for(self, grid: GridMap) -> bytearray:
        signature = (grid.width, grid.height, grid.resolution, grid.origin,
                     bytes(grid.data), float(self.nav.get("inflate_m", 0.30)),
                     bool(self.nav.get("allow_unknown", False)))
        if signature != self._grid_signature:
            self._blocked, _, _ = _build_blocked(grid, signature[-2], signature[-1])
            self._grid_signature = signature
        return self._blocked

    def _replan(self, now: float, pose: tuple[float, float, float], grid: GridMap) -> None:
        result = plan_path(grid, pose[:2], self._goal[:2],
                           inflate_m=float(self.nav.get("inflate_m", 0.30)),
                           allow_unknown=bool(self.nav.get("allow_unknown", False)))
        if not result.ok:
            self.finish("blocked", "地图路径受阻，已停车：" + result.reason)
            return
        self._path = result.path
        self._planned_at = now
        self._last_progress_distance = _path_length(result.path)
        self._last_progress_time = now
        self._last_heading_error = math.inf
        self._message = "地图路径变化，已停车并重新规划"

    def _safe_command(self, grid: GridMap, blocked: bytearray,
                      pose: tuple[float, float, float], linear: float,
                      angular: float) -> tuple[float, float]:
        horizon = max(0.5, float(self.nav.get("collision_horizon_s", 1.0)))
        steps = max(1, math.ceil(abs(linear) * horizon / (grid.resolution * 0.5)),
                    math.ceil(abs(angular) * horizon / 0.1))
        x, y, yaw_deg = pose
        yaw = math.radians(yaw_deg)
        previous = (x, y)
        for step in range(1, steps + 1):
            elapsed = horizon * step / steps
            if abs(angular) > 1e-6:
                point = (x + linear / angular * (math.sin(yaw + angular * elapsed) - math.sin(yaw)),
                         y - linear / angular * (math.cos(yaw + angular * elapsed) - math.cos(yaw)))
            else:
                point = (x + linear * elapsed * math.cos(yaw),
                         y + linear * elapsed * math.sin(yaw))
            if not _line_of_sight(grid, blocked, previous, point):
                self._message = "预测轨迹接近地图障碍，停车调整朝向"
                return 0.0, angular
            previous = point
        return linear, angular

    def update(self, now: float, pose: tuple[float, float, float],
               grid: GridMap | None = None) -> tuple[float, float] | None:
        """返回本周期期望速度 ``(linear, angular)``；None 表示无活动导航。"""
        with self._lock:
            if self._state not in ("following", "aligning"):
                return None
            if self._goal is None or len(self._path) < 2:
                self.cancel("路径为空")
                return None
            nav = self.nav
            x, y, yaw_deg = pose
            if not all(math.isfinite(value) for value in pose):
                self.finish("blocked", "定位坐标无效，已停车")
                return 0.0, 0.0
            grid = grid or self._grid
            if grid is None:
                self.finish("blocked", "导航地图不可用，已停车")
                return 0.0, 0.0
            self._grid = grid
            blocked = self._blocked_for(grid)
            self._path = _remaining_path(self._path, (x, y))
            check_path = [(x, y), *self._path]
            if not all(_line_of_sight(grid, blocked, a, b)
                       for a, b in zip(check_path, check_path[1:])):
                self._replan(now, pose, grid)
                return 0.0, 0.0
            self._remaining_distance = _path_length(check_path)
            goal_x, goal_y, goal_yaw = self._goal
            distance = math.hypot(goal_x - x, goal_y - y)
            self._distance = distance
            max_speed = min(float(nav.get("max_speed_m_s", 0.18)),
                            float(self.limits.get("max_forward_m_s", 0.3)))
            max_yaw = min(float(nav.get("max_yaw_rad_s", 0.6)),
                          float(self.limits.get("max_yaw_rad_s", 0.8)))
            tolerance = float(nav.get("tolerance_m", 0.15))
            yaw_tolerance = math.radians(float(nav.get("yaw_tolerance_deg", 15.0)))

            if distance <= tolerance:
                # 终点朝向对齐
                if goal_yaw is not None:
                    error = wrap_deg(goal_yaw - yaw_deg)
                    if abs(error) > math.degrees(yaw_tolerance):
                        self._state = "aligning"
                        self._message = "已到点，正在对齐朝向"
                        if not self._record_progress(now, abs(math.radians(error))):
                            return 0.0, 0.0
                        angular = clamp(math.radians(error) * 1.6, -max_yaw * 0.7,
                                        max_yaw * 0.7)
                        return 0.0, angular
                self.finish("arrived", "已到达目标点")
                return 0.0, 0.0

            # 从当前位置在路径上的投影选前视点，障碍拐角处缩短前视距离。
            lookahead = max(0.15, float(nav.get("lookahead_m", 0.45)))
            target = _point_along_path(self._path, lookahead)
            while not _line_of_sight(grid, blocked, (x, y), target):
                lookahead *= 0.5
                if lookahead < grid.resolution * 0.25:
                    self._replan(now, pose, grid)
                    return 0.0, 0.0
                target = _point_along_path(self._path, lookahead)

            heading = math.atan2(target[1] - y, target[0] - x)
            error = math.radians(wrap_deg(math.degrees(heading) - yaw_deg))
            self._state = "following"
            if not self._record_progress(now, abs(error)):
                return 0.0, 0.0
            if abs(error) > 0.9:
                # 偏差太大先原地转向，避免画大弧
                angular = clamp(error * 1.5, -max_yaw, max_yaw)
                self._message = "原地转向目标方向"
                return 0.0, angular
            speed = max_speed * max(0.25, math.cos(error))
            if distance < max(tolerance * 2.0, 0.30):
                speed = min(speed, max(0.04, max_speed * 0.4))
            angular = clamp(error * 2.2, -max_yaw, max_yaw)
            self._message = "跟踪路径中"
            return self._safe_command(grid, blocked, pose, speed, angular)

    def _record_progress(self, now: float, heading_error: float) -> bool:
        # 绕障时允许暂时远离终点。前进进度按剩余路径长度判断，转向按角误差判断。
        if self._remaining_distance < self._last_progress_distance - 0.03:
            self._last_progress_distance = self._remaining_distance
            self._last_progress_time = now
        if heading_error > self._last_heading_error + 0.2:
            self._last_heading_error = heading_error
        if heading_error < self._last_heading_error - 0.05:
            self._last_heading_error = heading_error
            self._last_progress_time = now
        timeout = max(1.0, float(self.nav.get("stuck_timeout_s", 8.0)))
        if now - self._last_progress_time > timeout:
            self.finish("stuck", f"超过 {timeout:g} 秒没有路径或转向进展，导航中止")
            return False
        return True

    # ------------------------------------------------------------------ 快照
    def snapshot(self) -> NavSnapshot:
        with self._lock:
            goal = list(self._goal) if self._goal else None
            path = [[round(px, 3), round(py, 3)] for px, py in self._path[:600]]
            return NavSnapshot(
                active=self._state in ("following", "aligning"),
                state=self._state,
                goal=goal,
                path=path,
                distance_m=None if self._distance is None else round(self._distance, 3),
                message=self._message,
                progress=self._progress(),
                planned_at=self._planned_at,
            )

    def _progress(self) -> float:
        if self._state == "arrived":
            return 1.0
        if self._start_distance <= 1e-6 or self._distance is None:
            return 0.0
        return clamp(1.0 - self._remaining_distance / self._start_distance, 0.0, 1.0)

    @property
    def state(self) -> str:
        with self._lock:
            return self._state
