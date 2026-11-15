"""前沿探索算法（``explore.py``）的离线回归：聚簇、过滤、评分、统计与性能。

全部用例使用手工构造的合成 ``GridMap``（默认全未知，未知格按 int8 ``-1`` 存成
``255``），不依赖任何地图文件、不启动 ROS、不产生速度。
"""
import math
import random
import time

import pytest

from host_ui.app.explore import ExplorationPlanner, find_frontiers
from host_ui.app.nav import plan_path
from host_ui.app.types import GridMap

UNKNOWN_BYTE = 255


# --------------------------------------------------------------------------- 构造
def make_grid(width, height, free_cells=(), occupied_cells=(),
              resolution=0.05, origin=(0.0, 0.0, 0.0), name="map"):
    """构造合成栅格：默认全未知，先写自由格(0)再写占用格(100)。"""
    data = bytearray([UNKNOWN_BYTE]) * (width * height)
    for x, y in free_cells:
        data[y * width + x] = 0
    for x, y in occupied_cells:
        data[y * width + x] = 100
    return GridMap(width, height, resolution, origin=origin, data=data, name=name)


def rectangle(x0, y0, x1, y1):
    """闭区间矩形内的全部栅格坐标。"""
    return {(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)}


def planner(**overrides):
    cfg = {
        "min_frontier_cells": 1,
        "max_goal_distance_m": 100.0,
        "gain_weight": 0.5,
        "blacklist_radius_m": 0.30,
        # 过滤/评分逻辑用例默认关闭膨胀（不需要 backoff）；
        # 回退专项用例显式设 inflate_m=0.30 + max_backoff_m 默认 0.60。
        "inflate_m": 0.0,
        "max_backoff_m": 0.0,
    }
    cfg.update(overrides)
    return ExplorationPlanner(cfg)


def only_frontier(grid, min_cells=1):
    frontiers = find_frontiers(grid, min_cells)
    assert len(frontiers) == 1, frontiers
    return frontiers[0]


def nav_blocked(grid, x, y, inflate_m):
    """按导航（allow_unknown=False + 欧氏膨胀）语义判断目标是否受阻（断言用）。

    与 ``nav._build_blocked`` 一致：占用格**和未知格**都参与膨胀。
    """
    gx, gy = grid.world_to_index(x, y)
    if not grid.contains_cell(gx, gy):
        return True
    pad = int(math.ceil(inflate_m / grid.resolution)) if inflate_m > 0 else 0
    for yy in range(gy - pad, gy + pad + 1):
        for xx in range(gx - pad, gx + pad + 1):
            if not grid.contains_cell(xx, yy):
                return True
            value = grid.at(xx, yy)
            if value < 0 or value >= 50:
                return True
    return False


# --------------------------------------------------------------------------- 提取与聚簇
def test_room_with_doorway_frontier_sits_on_known_unknown_boundary():
    """已知房间 + 门洞外未知：前沿只有一个门洞格，质心落在自由前沿格中心。"""
    free = rectangle(6, 6, 34, 34)
    occupied = set()
    for i in range(5, 36):
        occupied.update({(5, i), (35, i), (i, 5), (i, 35)})
    # 右侧墙开门：门洞格本身是自由格，外面是未知区域
    occupied.discard((35, 20))
    free.add((35, 20))
    grid = make_grid(50, 50, free, occupied)

    frontier = only_frontier(grid)
    assert frontier.cells == 1
    assert frontier.anchor_cell == (35, 20)
    assert frontier.gain == 3  # 门外 (36,19)/(36,20)/(36,21) 三个未知格
    assert grid.at(*frontier.anchor_cell) == 0

    # 质心 = 代表格中心，且代表格 8 邻域确实有未知格（已知/未知边界）。
    assert frontier.centroid == pytest.approx(grid.cell_center(35, 20))
    gx, gy = frontier.anchor_cell
    assert any(grid.at(gx + dx, gy + dy) < 0
               for dx in (-1, 0, 1) for dy in (-1, 0, 1)
               if (dx, dy) != (0, 0))


@pytest.mark.parametrize("value", [0, 100])
def test_fully_known_map_has_no_frontiers(value):
    """全自由或全占用的已建图区域没有前沿。"""
    grid = make_grid(25, 20)
    for y in range(20):
        for x in range(25):
            grid.set(x, y, value)
    assert find_frontiers(grid) == []


def test_fully_unknown_map_has_no_frontiers():
    """全未知地图没有自由格，因此没有前沿。"""
    assert find_frontiers(make_grid(30, 30)) == []


def test_min_cells_filters_small_clusters():
    """min_cells 过滤小簇：一条 19 格的自由带 + 一个孤立自由格。"""
    free = {(x, 5) for x in range(2, 21)} | {(25, 15)}
    grid = make_grid(40, 20, free)

    unfiltered = find_frontiers(grid, min_cells=1)
    assert len(unfiltered) == 2
    assert sorted(frontier.cells for frontier in unfiltered) == [1, 19]

    filtered = find_frontiers(grid, min_cells=2)
    assert len(filtered) == 1
    assert filtered[0].cells == 19
    assert find_frontiers(grid, min_cells=100) == []


def test_eight_neighbour_clusters_split_by_wall():
    """两段自由带被未知隔开（8 邻域不连通）→ 计为两个簇。"""
    free = ({(x, 5) for x in range(2, 19)}
            | {(x, 5) for x in range(22, 39)})
    grid = make_grid(50, 20, free)

    frontiers = find_frontiers(grid, min_cells=1)
    assert len(frontiers) == 2
    assert sorted(frontier.cells for frontier in frontiers) == [17, 17]
    # 按 gain 降序返回；两簇同 gain 同大小，仅要求 gain 有序即可。
    assert frontiers[0].gain >= frontiers[1].gain


def test_gain_dedupes_unknown_cells_shared_by_cluster_cells():
    """gain 是「相邻未知格去重」数：3x3 自由块周围是 5x5-3x3=16 个未知格。

    3x3 块的中心格四周都是自由格，不是前沿格，因此簇内只有 8 个前沿格。
    """
    grid = make_grid(20, 20, rectangle(3, 3, 5, 5))
    frontier = only_frontier(grid)
    assert frontier.cells == 8
    assert frontier.gain == 16


# --------------------------------------------------------------------------- 过滤
def test_max_goal_distance_filters_far_frontier():
    """唯一前沿在 max_goal_distance_m 之外 → next_goal 返回 None。"""
    grid = make_grid(80, 80, rectangle(3, 3, 5, 5))
    frontier = only_frontier(grid)
    pose = (0.05, 0.05, 0.0)

    near = planner(max_goal_distance_m=2.0)
    assert near.next_goal(grid, pose, 0.0) == pytest.approx(frontier.centroid)
    assert near.stats()["frontiers"] == 1

    far = planner(max_goal_distance_m=0.1)
    assert far.next_goal(grid, pose, 0.0) is None
    # 前沿本身仍被统计，只是被距离过滤掉。
    assert far.stats()["frontiers"] == 1
    assert far.stats()["last_goal"] is None


def test_blacklist_radius_filters_frontier():
    """黑名单半径 0.30 m：质心 0.29 m 内被过滤，0.31 m 外不受影响。"""
    grid = make_grid(80, 80, rectangle(3, 3, 5, 5))
    frontier = only_frontier(grid)
    cx, cy = frontier.centroid
    pose = (cx, cy, 0.0)

    blocked = planner()
    blocked.blacklist(cx + 0.29, cy, "目标不可达")
    assert blocked.next_goal(grid, pose, 0.0) is None
    assert blocked.stats()["blacklisted"] == 1

    unaffected = planner()
    unaffected.blacklist(cx + 0.31, cy, "别处卡死")
    assert unaffected.next_goal(grid, pose, 0.0) == pytest.approx(frontier.centroid)


def test_reset_clears_blacklist_and_stats():
    """reset() 清空黑名单与统计，同一前沿重新可选。"""
    grid = make_grid(80, 80, rectangle(3, 3, 5, 5))
    frontier = only_frontier(grid)
    explorer = planner()
    explorer.blacklist(*frontier.centroid, "卡死")
    assert explorer.next_goal(grid, (0.05, 0.05, 0.0), 0.0) is None

    explorer.reset()
    assert explorer.stats() == {
        "frontiers": 0, "raw_frontiers": 0, "blacklisted": 0, "visited": 0,
        "decision_reason": "idle", "last_goal": None, "last_score": None,
    }
    assert explorer.next_goal(grid, (0.05, 0.05, 0.0), 0.0) == pytest.approx(
        frontier.centroid)


# --------------------------------------------------------------------------- 评分
def test_score_prefers_higher_gain_at_equal_distance():
    """等距两簇：gain 大的（4x4 块）胜出，且 score 符合公式。"""
    grid = make_grid(60, 40, rectangle(5, 20, 7, 22) | rectangle(30, 20, 33, 23))
    frontiers = find_frontiers(grid, min_cells=1)
    assert len(frontiers) == 2
    # 3x3 块的 8 个外围格、4x4 块的 12 个外围格是前沿格。
    small = next(f for f in frontiers if f.cells == 8)
    large = next(f for f in frontiers if f.cells == 12)
    assert small.gain == 16
    assert large.gain == 20

    pose = ((small.centroid[0] + large.centroid[0]) / 2.0,
            (small.centroid[1] + large.centroid[1]) / 2.0,
            0.0)
    explorer = planner()
    goal = explorer.next_goal(grid, pose, 0.0)
    assert goal == pytest.approx(large.centroid)
    distance = math.dist(pose[:2], large.centroid)
    assert explorer.stats()["last_score"] == pytest.approx(
        distance - 0.5 * math.sqrt(20))


def test_score_prefers_closer_frontier_at_equal_gain():
    """相同 gain 的两个 3x3 块：站左边时选左边（距离分量胜出）。"""
    grid = make_grid(60, 40, rectangle(5, 20, 7, 22) | rectangle(30, 20, 32, 22))
    frontiers = find_frontiers(grid, min_cells=1)
    assert len(frontiers) == 2
    assert sorted(frontier.gain for frontier in frontiers) == [16, 16]
    left = min(frontiers, key=lambda frontier: frontier.centroid[0])

    explorer = planner()
    goal = explorer.next_goal(grid, (left.centroid[0], left.centroid[1], 0.0), 0.0)
    assert goal == pytest.approx(left.centroid)
    # 距离 0 时 score = -gain_weight * sqrt(gain)
    assert explorer.stats()["last_score"] == pytest.approx(-0.5 * 4.0)


# --------------------------------------------------------------------------- 质心与目标合法性
def test_centroid_is_free_cell_centre_and_goal_is_nav_reachable():
    """质心落在自由前沿格中心；返回的目标必须能被 nav 以 allow_unknown=False 规划。"""
    # 房间 20..59 x 10..49，四周墙体，顶部留 x=30..49 的宽开口对外未知。
    free = rectangle(20, 10, 59, 49)
    occupied = set()
    for y in range(9, 51):
        occupied.update({(19, y), (60, y)})
    occupied |= {(x, 9) for x in range(19, 61)}
    occupied |= {(x, 50) for x in list(range(20, 30)) + list(range(50, 60))}
    grid = make_grid(80, 60, free, occupied)

    frontier = only_frontier(grid)
    assert frontier.cells == 22  # 开口下方一排前沿格（含两角斜邻）
    gx, gy = frontier.anchor_cell
    assert grid.at(gx, gy) == 0
    assert frontier.centroid == pytest.approx(grid.cell_center(gx, gy))

    # 默认配置（min_frontier_cells=8、inflate_m=0.30）下必须给出可规划目标。
    center = grid.cell_center(39, 29)  # 房间内部、离墙足够远的起点
    pose = (center[0], center[1], 0.0)
    explorer = ExplorationPlanner({})
    goal = explorer.next_goal(grid, pose, 0.0)
    assert goal is not None
    # 质心贴未知格，落在 nav 的未知格膨胀区里：目标必须是从质心回退后的自由点。
    assert goal != pytest.approx(frontier.centroid)
    goal_cell = grid.world_to_index(*goal)
    assert grid.is_free(*goal_cell)
    assert not grid.is_occupied_world(goal[0], goal[1], 0.30)
    # 真正验收标准：nav 规划入口接受该目标。
    result = plan_path(grid, pose[:2], goal, inflate_m=0.30, allow_unknown=False)
    assert result.ok, result.reason


def test_next_goal_backs_off_frontier_centroid_inside_inflated_zone():
    """质心在 0.3 m 膨胀区内时，返回朝 pose 一侧回退、且自由/非膨胀区的目标。"""
    # 已知大房间 10..69 x 10..49，四周全是未知 → 前沿是房间内圈一圈前沿格。
    grid = make_grid(80, 60, rectangle(10, 10, 69, 49))
    frontier = only_frontier(grid)
    pose = grid.cell_center(39, 29)  # 房间内部
    pose = (pose[0], pose[1], 0.0)
    assert grid.is_free(*grid.world_to_index(*frontier.centroid[:2]))
    assert nav_blocked(grid, frontier.centroid[0], frontier.centroid[1], 0.30)  # 质心在膨胀区内

    explorer = ExplorationPlanner({"inflate_m": 0.30})  # max_backoff_m 默认 0.60
    goal = explorer.next_goal(grid, pose, 0.0)
    assert goal is not None
    assert goal != pytest.approx(frontier.centroid)
    # 方向朝 pose 一侧：目标到 pose 比质心到 pose 更近。
    assert math.dist(goal, pose[:2]) < math.dist(frontier.centroid, pose[:2])
    assert grid.is_free(*grid.world_to_index(*goal))
    assert not grid.is_occupied_world(goal[0], goal[1], 0.30)
    assert plan_path(grid, pose[:2], goal, inflate_m=0.30,
                     allow_unknown=False).ok
    # 关掉膨胀时不需要回退，质心直接可用（回退 0 m）。
    no_inflate = ExplorationPlanner({"inflate_m": 0.0})
    assert no_inflate.next_goal(grid, pose, 0.0) == pytest.approx(
        frontier.centroid)


def test_next_goal_returns_none_when_no_safe_backoff_point_exists():
    """四周都是未知、回退路径上没有可通行点时返回 None（不返回膨胀区内点）。"""
    grid = make_grid(40, 40, {(10, 10)})  # 唯一的自由格，四周全未知
    pose = (0.1, 0.1, 0.0)  # 远处，回退方向全部是未知格
    explorer = ExplorationPlanner({"inflate_m": 0.30})

    assert explorer.next_goal(grid, pose, 0.0) is None
    assert explorer.stats()["last_goal"] is None
    assert explorer.stats()["last_score"] is None


def test_next_goal_drops_candidate_when_max_backoff_is_too_small():
    """max_backoff_m 太小（0.05 m）时退不出膨胀区 → 候选被丢弃、返回 None。"""
    grid = make_grid(80, 60, rectangle(10, 10, 69, 49))
    frontier = only_frontier(grid)
    center = grid.cell_center(39, 29)
    pose = (center[0], center[1], 0.0)

    explorer = ExplorationPlanner({"inflate_m": 0.30, "max_backoff_m": 0.05})
    assert explorer.next_goal(grid, pose, 0.0) is None


def test_observation_point_uses_navigation_euclidean_clearance():
    """对角线外侧已知自由点可规划，不能被方形膨胀额外拒绝。"""
    from host_ui.app.nav import plan_path

    grid = make_grid(30, 30, rectangle(0, 0, 29, 29),
                     origin=(-2.0, -1.0, 30.0))
    grid.set(6, 6, -1)
    explorer = ExplorationPlanner({"inflate_m": 0.30})
    safe = grid.cell_center(11, 11)
    unsafe = grid.cell_center(10, 10)
    start = grid.cell_center(20, 20)
    assert not explorer._nav_blocked(grid, *safe)
    assert explorer._nav_blocked(grid, *unsafe)
    assert plan_path(grid, start, safe, inflate_m=0.30).ok
    assert explorer.stats()["last_goal"] is None


# --------------------------------------------------------------------------- 预览与统计
def test_frontiers_preview_is_limited_and_rounded():
    """36 个孤立前沿簇：preview 受 limit 约束、坐标 round 3 位、数量与统计一致。"""
    free = {(3 + 5 * i, 3 + 5 * j) for i in range(6) for j in range(6)}
    grid = make_grid(100, 100, free)
    explorer = planner()
    pose = (0.125, 0.125, 0.0)

    assert explorer.next_goal(grid, pose, 0.0) is not None
    assert explorer.stats()["frontiers"] == 36

    preview = explorer.frontiers_preview(grid, limit=10)
    assert len(preview) == 10
    for point in preview:
        assert isinstance(point, list) and len(point) == 2
        assert point[0] == round(point[0], 3)
        assert point[1] == round(point[1], 3)
    assert len(explorer.frontiers_preview(grid, limit=200)) == 36
    assert explorer.frontiers_preview(grid, limit=0) == []

    # 默认 min_frontier_cells=8 会过滤掉全部单格簇。
    assert ExplorationPlanner({}).next_goal(grid, pose, 0.0) is None


def test_stats_fields_and_types():
    """stats() 字段名/类型/内容正确，且 last_goal 与 next_goal 返回值一致。"""
    grid = make_grid(100, 100, {(3 + 5 * i, 3 + 5 * j)
                                for i in range(6) for j in range(6)})
    explorer = planner()
    assert explorer.stats() == {
        "frontiers": 0, "raw_frontiers": 0, "blacklisted": 0, "visited": 0,
        "decision_reason": "idle", "last_goal": None, "last_score": None,
    }

    goal = explorer.next_goal(grid, (0.125, 0.125, 0.0), 123.0)
    assert goal is not None
    stats = explorer.stats()
    assert set(stats) == {"frontiers", "raw_frontiers", "blacklisted", "visited",
                          "decision_reason", "last_goal", "last_score"}
    assert isinstance(stats["frontiers"], int) and stats["frontiers"] == 36
    assert isinstance(stats["blacklisted"], int) and stats["blacklisted"] == 0
    assert isinstance(stats["last_goal"], list)
    assert stats["last_goal"] == pytest.approx(list(goal))
    assert isinstance(stats["last_score"], float)

    explorer.blacklist(0.0, 0.0, "起始区域不可达")
    assert explorer.stats()["blacklisted"] == 1


# --------------------------------------------------------------------------- 配置与性能
def test_config_defaults_and_tolerant_parsing():
    """cfg 缺键/非法值回默认，数值 clamp 与 max_backoff_m 默认值（inflate+0.30）。"""
    defaults = ExplorationPlanner({}).cfg
    assert defaults == {
        "min_frontier_cells": 8,
        "max_goal_distance_m": 15.0,
        "gain_weight": 0.5,
        "blacklist_radius_m": 0.30,
        "inflate_m": 0.30,
        "max_backoff_m": 0.60,  # inflate_m(0.30) + 0.30
    }
    assert ExplorationPlanner(None).cfg == defaults
    broken = ExplorationPlanner({
        "min_frontier_cells": "坏值",
        "max_goal_distance_m": None,
        "gain_weight": float("nan"),
        "blacklist_radius_m": "x",
        "inflate_m": float("inf"),
        "max_backoff_m": float("nan"),
    })
    assert broken.cfg == defaults
    clamped = ExplorationPlanner({"min_frontier_cells": 0, "inflate_m": -1.0})
    assert clamped.cfg["min_frontier_cells"] == 1
    assert clamped.cfg["inflate_m"] == 0.0
    # 未显式配置时 max_backoff_m 跟随 inflate_m：0 + 0.30
    assert clamped.cfg["max_backoff_m"] == pytest.approx(0.30)
    # 显式配置的 max_backoff_m clamp 到 [0, 2]
    assert ExplorationPlanner({"max_backoff_m": 9.0}).cfg["max_backoff_m"] == 2.0
    assert ExplorationPlanner({"max_backoff_m": -3.0}).cfg["max_backoff_m"] == 0.0
    # 自定义 inflate 会同步抬高默认回退距离：0.5 + 0.30
    assert ExplorationPlanner({"inflate_m": 0.5}).cfg["max_backoff_m"] == pytest.approx(
        0.80)


def test_find_frontiers_performance_on_synthetic_300x300_map():
    """性能 sanity：300x300 混合栅格单次调用远低于 200 ms。"""
    randomizer = random.Random(7)
    data = bytearray(randomizer.choices([0, UNKNOWN_BYTE, 100],
                                        weights=(0.3, 0.5, 0.2), k=300 * 300))
    grid = GridMap(300, 300, 0.05, data=data)

    started = time.perf_counter()
    frontiers = find_frontiers(grid, min_cells=1)
    elapsed = time.perf_counter() - started
    assert elapsed < 0.2, f"find_frontiers 过慢: {elapsed * 1000:.1f} ms"
    assert frontiers  # 地图确实有前沿，避免空跑
    assert all(frontier.gain > 0 for frontier in frontiers)
