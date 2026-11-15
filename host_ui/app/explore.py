"""前沿（frontier）探索：候选点提取、聚簇与目标选择。

前沿格定义
----------
**自由格**（栅格值 ``0 <= v < 50``）且其 **8 邻域**内存在**未知格**（``v < 0``）。
前沿簇 = 前沿格在 8 邻域下的连通分量。每个簇给出一个可导航的代表点，供上层
``node.py`` 的探索状态机作为下一个 A* 目标。

目标点与「回退」（契约 §2 + 2026-11-14 修订）
--------------------------------------------
前沿质心按定义必然紧贴未知格，而导航以 ``allow_unknown=False`` 规划（未知格
视作障碍并做 ``inflate_m`` 膨胀），所以质心通常在 A* 的受阻区内。因此
``ExplorationPlanner.next_goal`` 会把质心沿指向机器人 ``pose`` 的方向回退
（最多 ``max_backoff_m``，默认 ``inflate_m + 0.30``），返回第一个既自由、又
不在「占用格 / 未知格」膨胀区内的点；退不出则丢弃该候选（保守，不返回受阻点）。
``frontiers_preview`` 仍返回质心，供 UI 展示「前沿在哪」。

上层在到达后调用 ``mark_visited(x, y, grid)``；相同地图内容下不再选择该目标
邻域，内容改变后允许重新评价。``stats().decision_reason`` 区分没有前沿、
前沿存在但筛选后无候选、已访问目标等待地图变化三种情况，避免筛空即宣告完成。

设计边界（契约 §2 / §7）
------------------------
本模块是**纯算法**，只读取 ``types.GridMap`` 与标准库：

* 不产生速度、不接触 ROS、不读写地图文件、没有线程与后台循环；
* 候选目标的实际运动由上层 ``nav.PointNavigator`` 规划跟踪，速度必须经
  ``SafetyCore`` 与机载 execution_guard 拦截，**本模块不得成为任何旁路**。

性能
----
``find_frontiers`` 先用 ``bytes.translate`` 把整图映射成 0/1 掩膜，再用大整数
位运算一次性求出「8 邻域存在未知格」的掩膜（O(N)，循环发生在 C 层），最后用
显式栈（非递归，避免大图栈溢出）做 8 邻域聚簇。1000x1000 的典型 SLAM 地图
（自由区与未知区相接）单次调用十几毫秒，低于契约要求的 ~50 ms。
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass

from .types import GridMap

__all__ = ["Frontier", "find_frontiers", "ExplorationPlanner"]

# ``GridMap.data`` 是 bytearray：未知格 ``-1`` 存成 ``255``，有符号值读法为
# ``v - 256 if v > 127 else v``。下面两张表把原始字节一次翻译成 0/1 掩膜。
_FREE_TABLE = bytes(
    1 if 0 <= (raw - 256 if raw > 127 else raw) < 50 else 0 for raw in range(256))
_UNKNOWN_TABLE = bytes(
    1 if (raw - 256 if raw > 127 else raw) < 0 else 0 for raw in range(256))

# cfg 默认值（契约 §2 及 2026-11-14 修订：新增 max_backoff_m；
# config.py 的 explore 段负责再收敛一次范围）
_DEFAULT_CFG = {
    "min_frontier_cells": 8,
    "max_goal_distance_m": 15.0,
    "gain_weight": 0.5,
    "blacklist_radius_m": 0.30,
    "inflate_m": 0.30,
}


# --------------------------------------------------------------------------- 工具
def _coerce_float(value, default):
    """容错解析浮点配置：非法/非有限值回落到默认值。"""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return float(default)
    return parsed if math.isfinite(parsed) else float(default)


def _coerce_int(value, default):
    """容错解析整数配置：非法值回落到默认值。"""
    try:
        parsed = int(float(value))
    except (TypeError, ValueError, OverflowError):
        return int(default)
    return parsed


def _normalize_cfg(cfg):
    """把外部配置收敛成本模块使用的六元组，非法值一律回默认。"""
    source = cfg if isinstance(cfg, dict) else {}
    defaults = _DEFAULT_CFG
    inflate_m = max(0.0, _coerce_float(source.get("inflate_m"),
                                      defaults["inflate_m"]))
    # 契约修订：``max_backoff_m`` 默认 = inflate + 0.30（= 0.60 m @ inflate=0.30），
    # 保证从「未知格膨胀区内」的前沿质心向 pose 一侧退到 nav 可规划区域；
    # 先算默认再 clamp 到 [0, 2] m。
    max_backoff_m = _coerce_float(source.get("max_backoff_m"),
                                  inflate_m + 0.30)
    max_backoff_m = min(2.0, max(0.0, max_backoff_m))
    return {
        "min_frontier_cells": max(
            1, _coerce_int(source.get("min_frontier_cells"),
                           defaults["min_frontier_cells"])),
        "max_goal_distance_m": max(
            0.0, _coerce_float(source.get("max_goal_distance_m"),
                               defaults["max_goal_distance_m"])),
        "gain_weight": _coerce_float(source.get("gain_weight"),
                                     defaults["gain_weight"]),
        "blacklist_radius_m": max(
            0.0, _coerce_float(source.get("blacklist_radius_m"),
                               defaults["blacklist_radius_m"])),
        "inflate_m": inflate_m,
        "max_backoff_m": max_backoff_m,
    }


def _has_unknown_neighbour(unknown_flags, width, height, x, y):
    """逐格检查 8 邻域是否有未知格（仅用于地图边界列的修正，调用量很小）。"""
    x0 = x - 1 if x > 0 else 0
    x1 = x + 1 if x + 1 < width else width - 1
    y0 = y - 1 if y > 0 else 0
    y1 = y + 1 if y + 1 < height else height - 1
    for yy in range(y0, y1 + 1):
        base = yy * width
        for xx in range(x0, x1 + 1):
            if xx == x and yy == y:
                continue
            if unknown_flags[base + xx]:
                return True
    return False


def _frontier_cell_flags(grid):
    """返回 (前沿格掩膜, 未知格掩膜)，两者都是每格 1 字节的 0/1 序列。

    快路径：把整图当成一个 8N bit 的大整数，按字节（=按格）位移求出 8 邻域并集，
    再与自由格掩膜相与。位移量都是 8 的整数倍，因此每个字节只会是 0/1。
    """
    data = grid.data
    free_flags = data.translate(_FREE_TABLE)
    unknown_flags = data.translate(_UNKNOWN_TABLE)
    width, height = grid.width, grid.height
    size = width * height

    if width < 2:
        # 单列地图：水平位移会跨行，直接用逐格实现（极小地图，无性能压力）。
        flags = bytearray(size)
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if free_flags[index] and _has_unknown_neighbour(
                        unknown_flags, width, height, x, y):
                    flags[index] = 1
        return flags, unknown_flags

    unknown_int = int.from_bytes(unknown_flags, "little")
    row_bits = 8 * width
    neighbour_unknown = (
        unknown_int
        | (unknown_int << 8) | (unknown_int >> 8)              # 左右
        | (unknown_int << row_bits) | (unknown_int >> row_bits)  # 上下
        | (unknown_int << (row_bits + 8))                        # 左下/右上
        | (unknown_int >> (row_bits + 8))
        | (unknown_int << (row_bits - 8))                        # 右下/左上
        | (unknown_int >> (row_bits - 8))
    )
    free_int = int.from_bytes(free_flags, "little")
    flags = bytearray((neighbour_unknown & free_int).to_bytes(size, "little"))

    # 左右位移（±8 bit）会把上一行末列 / 下一行首列误当成同行的邻居，
    # 只影响第 0 列与最后一列；这两列逐格重算，其余列结果精确。
    for x in (0, width - 1):
        for y in range(height):
            index = y * width + x
            if not free_flags[index]:
                continue
            flags[index] = 1 if _has_unknown_neighbour(
                unknown_flags, width, height, x, y) else 0
    return flags, unknown_flags


# --------------------------------------------------------------------------- 数据结构
@dataclass
class Frontier:
    """一个前沿簇（8 邻域连通的前沿格集合）。"""

    cells: int                       # 簇内自由前沿格数量
    gain: int                        # 与簇内任一前沿格 8 相邻的未知格数（去重）
    centroid: tuple[float, float]    # = cell_center(anchor_cell)，保证落在自由前沿格中心
    anchor_cell: tuple[int, int]     # 离簇质心最近的代表格 (gx, gy)


def find_frontiers(grid: GridMap, min_cells: int = 1) -> list[Frontier]:
    """提取并聚簇地图上的前沿，按 ``gain`` 降序返回（同 gain 时簇大者优先）。

    前沿格 = 自由格（``0 <= v < 50``）且 8 邻域存在未知格（``v < 0``）；
    簇 = 前沿格在 8 邻域下的连通分量。一次全图扫描 + 显式栈 DFS，无递归。

    ``min_cells`` 过滤小簇（``find_frontiers`` 侧按 ``>= max(1, min_cells)`` 判定）。
    ``centroid`` 取「距簇内格中心均值最近的簇内前沿格」的 ``cell_center()``，
    因此一定落在自由前沿格中心，不会落到未知/占用格；上层若再叠加膨胀检查
    （``ExplorationPlanner.next_goal``），被膨胀区命中的候选会被整体丢弃。
    """
    if grid is None:
        return []
    try:
        minimum = max(1, int(min_cells))
    except (TypeError, ValueError):
        minimum = 1

    flags, unknown_flags = _frontier_cell_flags(grid)
    width, height = grid.width, grid.height
    padded_width = width + 2
    padded_size = (height + 2) * padded_width

    # 加一圈 0 边界，聚簇时无需逐邻居做越界判断。
    padded_flags = bytearray(padded_size)
    padded_unknown = bytearray(padded_size)
    for y in range(height):
        source = y * width
        target = (y + 1) * padded_width + 1
        padded_flags[target:target + width] = flags[source:source + width]
        padded_unknown[target:target + width] = unknown_flags[source:source + width]

    # 收集全部前沿格（C 层的 find 循环；密度大时仍是本函数的主要开销）。
    roots = []
    find = padded_flags.find
    position = find(1)
    while position != -1:
        roots.append(position)
        position = find(1, position + 1)

    offsets = (-padded_width - 1, -padded_width, -padded_width + 1,
               -1, 1,
               padded_width - 1, padded_width, padded_width + 1)
    visited = bytearray(padded_size)
    frontiers = []
    for root in roots:
        if visited[root]:
            continue
        visited[root] = 1
        stack = [root]
        cells = []
        sum_x = 0
        sum_y = 0
        while stack:
            index = stack.pop()
            cells.append(index)
            sum_x += index % padded_width
            sum_y += index // padded_width
            for offset in offsets:
                neighbour = index + offset
                if padded_flags[neighbour] and not visited[neighbour]:
                    visited[neighbour] = 1
                    stack.append(neighbour)
        count = len(cells)
        if count < minimum:
            continue

        # 簇质心 = 簇内格中心均值（世界坐标是等距变换，可在格坐标里比较距离）。
        mean_x = sum_x / count
        mean_y = sum_y / count
        anchor_cell = (0, 0)
        best_key = None
        for index in cells:
            cx = index % padded_width
            cy = index // padded_width
            key = ((cx - mean_x) ** 2 + (cy - mean_y) ** 2, cy, cx)
            if best_key is None or key < best_key:
                best_key = key
                anchor_cell = (cx - 1, cy - 1)

        # 信息增益：簇内所有前沿格相邻未知格的并集大小（去重）。
        gain_cells = set()
        for index in cells:
            for offset in offsets:
                neighbour = index + offset
                if padded_unknown[neighbour]:
                    gain_cells.add(neighbour)

        frontiers.append(Frontier(
            cells=count,
            gain=len(gain_cells),
            centroid=grid.cell_center(*anchor_cell),
            anchor_cell=anchor_cell,
        ))

    frontiers.sort(key=lambda frontier: (-frontier.gain, -frontier.cells,
                                         frontier.anchor_cell))
    return frontiers


# --------------------------------------------------------------------------- 规划器
class ExplorationPlanner:
    """从 ``find_frontiers`` 的候选里挑选下一个探索目标（只选择，不执行运动）。

    选择规则（契约 §2 第 4 步 + 2026-11-14 修订）：

    1. ``find_frontiers(grid, cfg['min_frontier_cells'])``；
    2. 用**质心**做粗筛：距 ``pose`` 平面距离超过 ``cfg['max_goal_distance_m']``、
       或落在任一黑名单点 ``cfg['blacklist_radius_m']`` 内的候选丢弃；
    3. **目标回退（backoff）**：前沿格的定义决定质心必然紧贴未知格，而导航
       ``plan_path`` 以 ``allow_unknown=False`` 规划（未知格视作障碍并做
       ``inflate_m`` 膨胀），因此质心几乎总在 A* 的受阻区内。从质心沿指向
       ``pose`` 的方向按栅格分辨率回退（最多 ``cfg['max_backoff_m']``，默认
       ``inflate_m + 0.30``），取第一个「自由格且不在占用/未知膨胀区内」的点；
       回退超限仍无可用点则丢弃该候选（保守，绝不返回膨胀区内的点）；
    4. 评分 ``score = dist(pose, 回退点) - cfg['gain_weight'] * sqrt(gain)``，取最小；
    5. 记录 ``_last_goal``（**回退后的点**，不是质心）/ ``_last_score`` /
       ``_frontiers``；无候选返回 ``None``。

    ``frontiers_preview`` 仍然返回质心（UI 上「前沿在哪」与「开去哪」是两件事）。

    本类不做时间/线程逻辑，``now`` 只为与上层状态机签名对齐而保留。
    """

    def __init__(self, cfg: dict) -> None:
        self.cfg = _normalize_cfg(cfg)
        self._blacklist: list[tuple[float, float, str]] = []
        self._visited: list[tuple[float, float]] = []
        self._visited_map_signature = None
        self._last_goal: tuple[float, float] | None = None
        self._last_score: float | None = None
        self._frontiers = 0
        self._raw_frontiers = 0
        self._decision_reason = "idle"

    # ------------------------------------------------------------------ 状态
    def reset(self) -> None:
        """清空黑名单与上一轮选择统计（重新开始探索时调用）。"""
        self._blacklist.clear()
        self._visited.clear()
        self._visited_map_signature = None
        self._last_goal = None
        self._last_score = None
        self._frontiers = 0
        self._raw_frontiers = 0
        self._decision_reason = "idle"

    def _sync_visited_map(self, grid: GridMap) -> None:
        """地图内容变更后允许重评已到达点；重复发布相同地图不算观测进展。"""
        signature = (grid.width, grid.height, grid.resolution, grid.origin,
                     grid.name, hashlib.sha256(grid.data).digest())
        if signature != self._visited_map_signature:
            self._visited.clear()
            self._visited_map_signature = signature

    def mark_visited(self, x: float, y: float, grid: GridMap) -> bool:
        """记录真实到达的目标，在地图内容改变前不再选择其邻域。

        上层应在导航报告 arrived 后调用，再等待新观测。这里不把访问过的点
        永久拉黑：新地图可揭示门洞或新的可达路径，届时这些目标可重新评价。
        """
        try:
            point = (float(x), float(y))
        except (TypeError, ValueError):
            return False
        if grid is None or not all(math.isfinite(value) for value in point):
            return False
        self._sync_visited_map(grid)
        radius = self.cfg["blacklist_radius_m"]
        if not any(math.dist(point, visited) <= radius for visited in self._visited):
            self._visited.append(point)
        return True

    def blacklist(self, x: float, y: float, reason: str = "") -> None:
        """记录一个「不可达/卡死」位置；半径内的前沿候选会被过滤掉。"""
        try:
            black_x, black_y = float(x), float(y)
        except (TypeError, ValueError):
            return
        if not (math.isfinite(black_x) and math.isfinite(black_y)):
            return
        self._blacklist.append((black_x, black_y, str(reason)))

    def _is_blacklisted(self, x: float, y: float) -> bool:
        radius = self.cfg["blacklist_radius_m"]
        for black_x, black_y, _reason in self._blacklist:
            if math.hypot(x - black_x, y - black_y) <= radius:
                return True
        return False

    def _nav_blocked(self, grid, x: float, y: float) -> bool:
        """按导航（``allow_unknown=False`` + 欧氏膨胀）的语义判断该点能否作为目标。

        与 ``nav._build_blocked`` 保持一致：未知格与占用格按栅格中心的欧氏距离
        参与膨胀。地图外的目标不可用，地图内目标的膨胀只考虑地图内的格子。
        """
        gx, gy = grid.world_to_index(x, y)
        if not grid.contains_cell(gx, gy):
            return True
        inflate_m = self.cfg["inflate_m"]
        radius = inflate_m / grid.resolution
        pad = int(math.ceil(radius))
        for gy_cell in range(max(0, gy - pad), min(grid.height, gy + pad + 1)):
            for gx_cell in range(max(0, gx - pad), min(grid.width, gx + pad + 1)):
                if (gx_cell - gx) ** 2 + (gy_cell - gy) ** 2 > radius * radius + 1e-9:
                    continue
                value = grid.at(gx_cell, gy_cell)
                if value < 0 or value >= 50:  # 未知格或占用格
                    return True
        return False

    def _backoff_goal(self, grid, centroid: tuple[float, float],
                      position: tuple[float, float]) -> tuple[float, float] | None:
        """沿「质心 → pose」方向回退到第一个导航可规划的自由点。

        质心本身可用则直接返回它（回退 0 m）。找不到可用回退点时返回 ``None``，
        绝不返回未知/占用/膨胀区内的点。
        """
        if not self._nav_blocked(grid, centroid[0], centroid[1]):
            return centroid
        dx = position[0] - centroid[0]
        dy = position[1] - centroid[1]
        distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            return None
        unit_x, unit_y = dx / distance, dy / distance
        step = max(float(grid.resolution), 1e-6)
        max_backoff = self.cfg["max_backoff_m"]
        index = 1
        while index * step <= max_backoff + 1e-9:
            travel = index * step
            candidate = (centroid[0] + unit_x * travel,
                         centroid[1] + unit_y * travel)
            if not self._nav_blocked(grid, candidate[0], candidate[1]):
                return candidate
            index += 1
        return None

    # ------------------------------------------------------------------ 选择
    def next_goal(self, grid, pose, now) -> tuple[float, float] | None:
        """选择下一个探索目标，返回世界坐标 ``(x, y)``；无候选返回 ``None``。

        返回点保证是导航可规划的自由点（占用/未知膨胀区之外）；它可能从簇质心
        沿指向 ``pose`` 的方向回退若干栅格，见类文档第 3 步。
        """
        # ``now`` 仅为接口对齐保留：本模块不依赖时钟，也不做超时判断。
        del now
        if grid is not None:
            self._sync_visited_map(grid)
        raw_frontiers = find_frontiers(grid, 1) if grid is not None else []
        self._raw_frontiers = len(raw_frontiers)
        frontiers = [frontier for frontier in raw_frontiers
                     if frontier.cells >= self.cfg["min_frontier_cells"]]
        self._frontiers = len(frontiers)

        position = None
        try:
            position = (float(pose[0]), float(pose[1]))
        except (TypeError, ValueError, IndexError):
            position = None
        if position is None or not (math.isfinite(position[0])
                                    and math.isfinite(position[1])):
            self._last_goal = None
            self._last_score = None
            self._decision_reason = "invalid_pose"
            return None

        pose_x, pose_y = position
        max_distance = self.cfg["max_goal_distance_m"]
        gain_weight = self.cfg["gain_weight"]
        best_goal = None
        best_score = math.inf
        visited_candidate = False
        for frontier in frontiers:
            centroid_x, centroid_y = frontier.centroid
            # 用质心做粗筛；真正的目标点随后由 backoff 给出。
            if math.hypot(centroid_x - pose_x,
                          centroid_y - pose_y) > max_distance:
                continue
            if self._is_blacklisted(centroid_x, centroid_y):
                continue
            goal = self._backoff_goal(grid, frontier.centroid, position)
            if goal is None:
                continue
            goal_x, goal_y = goal
            # 黑名单记录的是实际开过去的目标点（node 在黑名单里存的是 last_goal），
            # 因此回退点本身也要做一次黑名单检查，避免原地点失败后反复重选同一点。
            if self._is_blacklisted(goal_x, goal_y):
                continue
            if any(math.dist(goal, visited) <= self.cfg["blacklist_radius_m"]
                   for visited in self._visited):
                visited_candidate = True
                continue
            distance = math.hypot(goal_x - pose_x, goal_y - pose_y)
            score = distance - gain_weight * math.sqrt(frontier.gain)
            if score < best_score:
                best_score = score
                best_goal = goal

        if best_goal is None:
            self._last_goal = None
            self._last_score = None
            self._decision_reason = ("no_frontiers" if not raw_frontiers
                                     else "waiting_for_map" if visited_candidate
                                     else "no_candidates")
            return None
        self._last_goal = best_goal
        self._last_score = float(best_score)
        self._decision_reason = "goal_selected"
        return best_goal

    def frontiers_preview(self, grid, limit: int = 200) -> list[list[float]]:
        """当前前沿簇质心（世界坐标，round 3 位）供 UI 画点，最多 ``limit`` 个。

        与 ``next_goal``/``stats()`` 使用同一个 ``min_frontier_cells``，保证
        「前沿簇数」与预览点数量一致；本调用不改变统计状态。
        """
        if grid is None:
            return []
        try:
            limit_value = int(limit)
        except (TypeError, ValueError):
            limit_value = 200
        if limit_value <= 0:
            return []
        frontiers = find_frontiers(grid, self.cfg["min_frontier_cells"])
        return [[round(frontier.centroid[0], 3), round(frontier.centroid[1], 3)]
                for frontier in frontiers[:limit_value]]

    # ------------------------------------------------------------------ 快照
    def stats(self) -> dict:
        """最近一次选择统计，包含原始前沿数、访问暂避数与无目标原因。"""
        goal = None
        if self._last_goal is not None:
            goal = [float(self._last_goal[0]), float(self._last_goal[1])]
        score = None if self._last_score is None else float(self._last_score)
        return {
            "frontiers": int(self._frontiers),
            "raw_frontiers": int(self._raw_frontiers),
            "blacklisted": len(self._blacklist),
            "visited": len(self._visited),
            "decision_reason": self._decision_reason,
            "last_goal": goal,
            "last_score": score,
        }
