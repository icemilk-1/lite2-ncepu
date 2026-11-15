# 自主导航 + 建图 + 前沿探索 实施契约（2026-11-14 晚 · v1.4.0 批次）

> 本文件是 Lead 与 `link-dev` / `nav-dev` / `frontend-dev` 的共同接口约定，**以本文件为准**；需要变更先找 Lead。
> 前置阅读：`host_ui/README.md`、`host_ui/AUDIT_20261114.md`、`host_ui/IMPLEMENTATION_CONTRACT_20261114.md`（批次 1 契约，仍有效）。

## 0. 目标与现状

用户已确认：**机器人现在可达且现场有人监护**。本轮交付三件事并在实机验收：

1. **建图**：UI 一键启停机载 SLAM（hdl_graph_slam + octomap），`/projected_map` 与定位（`/odom` + TF map→odom）实时上图。
2. **自主导航**：点选目标 → A* + 纯追踪 → 25Hz 速度流（已有，需实机验证）；**新增自主探索建图**（frontier 前沿探索，自动走遍未知区域）。
3. **前进后退**：D-pad/键盘点动链路已有（机载门禁 ±0.05 m/s 支持负值），本轮只做实机低速验证，不改交互。

已探明的关键隐患（必须修）：

- `lite2_sim/hardware/navigation_mapping.launch` 目前 include 机载 `hdl_graph_slam/mapping_rslidar_indoor.launch`，而该文件内部会**再启动一次雷达驱动** `rs_lidar_16.launch`（见 `tmp/mapping_rslidar_indoor.launch.snapshot`）。机器人上 `/rslidar_points` 已有驱动在发布 → 一点「开始建图」就双驱动冲突。**launch 必须改为自包含 + 驱动条件启动**。
- `ssh_ros_stream.py` 的 `odometry_received` 每帧 `/odom` 都 emit 一条 localization JSON，无限频 → 需节流 ≤10 Hz。

## 1. 文件所有权（本批次）

| 角色 | 只允许改这些文件 |
|---|---|
| Lead | `host_ui/app/{config,node,server,state,types}.py`、`host_ui/run.py`、`host_ui/tests/test_{node,safety,control_features,navigation}.py`、`host_ui/*.md`、`host_ui/app/nav.py`（如需） |
| nav-dev | **新建** `host_ui/app/explore.py`、**新建** `host_ui/tests/test_exploration.py` |
| link-dev | `lite2_sim/hardware/navigation_mapping.launch`、`host_ui/app/transports/{ssh_ros_stream,ssh_ros,mock}.py`、`host_ui/tests/test_{ssh_ros,mock_transport,deployment_sources}.py` |
| frontend-dev | `host_ui/web/{index.html,app.js,workspace.js,map.js,style.css,workspace.css}` |

约束：
- `ssh_ros_stream.py` 必须保持 **Python 2.7 兼容**（无 f-string、无 `math.isfinite`、无类型注解；monotonic 时钟规则见该文件 docstring，禁止墙钟退化）。
- 任何新速度输出只能走既有链路：`nav → SafetyCore.tick → transport.send_velocity`。**探索不得绕过 SafetyCore / 操作者心跳 / 急停**。
- 版本号：Lead 统一把 `HOST_UI_VERSION` 升到 `"1.4.0"`，其他人不改版本。
- 自测不要占用 8710 端口；起独立实例用 `--port 8721+ --link mock --config tmp/<name>_config.json`。

## 2. nav-dev 交付：`host_ui/app/explore.py`

纯算法模块，**只依赖 `.types.GridMap` 与标准库**（可 import `nav.py` 的公开函数但不要改它）。无线程、无 IO、无 ROS。

```python
@dataclass
class Frontier:
    cells: int                      # 簇内自由前沿格数量
    gain: int                       # 相邻未知格数量（去重，信息增益）
    centroid: tuple[float, float]   # 簇质心世界坐标（snap 到该簇某个前沿格的 cell_center）
    anchor_cell: tuple[int, int]    # 代表格 (gx, gy)

def find_frontiers(grid: GridMap, min_cells: int = 1) -> list[Frontier]:
    """前沿格 = 自由格(0<=v<50) 且 8 邻域存在未知格(v<0)；连通域(8邻域)聚簇。
    min_cells 过滤小簇。返回按 gain 降序。"""

class ExplorationPlanner:
    def __init__(self, cfg: dict) -> None: ...
    def reset(self) -> None: ...
    def blacklist(self, x: float, y: float, reason: str = "") -> None:
        """把半径 cfg['blacklist_radius_m'] 内的前沿拉黑（目标不可达/卡死时用）。"""
    def next_goal(self, grid, pose, now) -> tuple[float, float] | None:
        """选下一个探索目标：
        1) find_frontiers(grid, min_frontier_cells)
        2) 过滤：距 pose 平面距离 > max_goal_distance_m 的丢弃；
           质心落在黑名单半径内的丢弃；
           grid.is_occupied_world(cx, cy, cfg['inflate_m']) 为 True 的丢弃
        3) 评分 score = dist - gain_weight * sqrt(gain)，取最小
        4) 记录 last_goal / last_score / 当前前沿数；无候选返回 None"""
    def frontiers_preview(self, grid, limit: int = 200) -> list[list[float]]:
        """当前全部前沿簇质心（世界坐标，四舍五入 3 位），供 UI 画点，最多 limit 个。"""
    def stats(self) -> dict:
        """{'frontiers': int, 'blacklisted': int, 'last_goal': [x,y]|None, 'last_score': float|None}"""
```

cfg 键（由 Lead 在 `config.py` 的 `explore` 段提供并收敛）：
`min_frontier_cells`(默认 8)、`max_goal_distance_m`(15.0)、`gain_weight`(0.5)、`blacklist_radius_m`(0.30)、`inflate_m`(0.30，与 nav 一致)、`max_backoff_m`(0.60)。

### 2.1 修订（2026-11-14 20:5x，mock E2E 实测后）

**原规则「质心 `is_occupied_world(…, inflate)` 为 True 即丢弃候选 + 直接把质心当目标」不可用**：
前沿格按定义必然紧邻未知格，而导航规划用 `allow_unknown=False` 把未知格当障碍再做 0.30 m（6 格 @0.05 m）膨胀，
因此前沿质心**永远落在膨胀受阻区内** → `next_goal` 恒返回 None、`plan_path` 恒报「目标点位于障碍、未知区域或膨胀范围内」。
实测证据：mock 掩膜地图上唯一前沿簇 6669 格 / gain 9155 / 质心 (0.925, -0.025)；`plan_path` 判目标格 (198,119) blocked，起点 (179,120) 正常。

**新规则（目标回退 backoff）**：对每个候选簇

1. 用**质心**做距离/黑名单粗筛（不满足则丢弃）；
2. 从质心沿**指向当前 pose 的方向**按 `grid.resolution` 步进（最多 `max_backoff_m`，默认 `inflate_m + 0.30 = 0.60` m），
   取第一个同时满足 `grid.is_free(cx, cy)`（值 0..49）且 `not grid.is_occupied_world(cx, cy, inflate_m)` 的点作为**目标**；
3. 回退超限仍无可用点 → 丢弃该候选（保守：绝不把目标放进未知/障碍/膨胀区）；
4. `dist`/`score = dist - gain_weight*sqrt(gain)` 用**回退后的目标**计算；`_last_goal` 记录回退后的目标；
5. `frontiers_preview` **不变**（继续画质心——"前沿在哪"与"开去哪"是两件事）。

对应测试：① 质心在膨胀区内仍能返回「不在膨胀区且 is_free」的 backoff 目标，且目标比质心更靠近 pose；② 两侧皆障碍、无可用回退点时返回 None；③ `max_backoff_m` 很小时候选被丢弃、返回 None。

性能：单次 `find_frontiers` 对 1000×1000 栅格应在 ~50 ms 内（bytearray + 显式栈/队列，避免递归）。

测试 `host_ui/tests/test_exploration.py`（≥10 个用例）：
- 合成 GridMap（手工构造 data bytes）：已知房间 + 门洞外未知 → 前沿出现在边界；全已知图 → 空；
- `min_frontier_cells` 过滤；`max_goal_distance_m` 过滤；黑名单半径过滤；
- 评分偏好：等距下 gain 大的簇胜出；近的小簇 vs 远的大簇按公式断言；
- 质心 snap：centroid 一定落在自由前沿格中心且 `is_occupied_world(…, inflate)` 为 False 才被选中；
- `stats()`/`frontiers_preview()` 内容正确、limit 生效；`reset()` 清空黑名单。

## 3. Lead 交付：config / node / server 集成

### 3.1 `config.py` 新增 `explore` 段（normalize 收敛）

```jsonc
"explore": {
  "enabled": true,
  "min_frontier_cells": 8,        // clamp 1..100
  "max_goal_distance_m": 15.0,    // clamp 0.5..50
  "gain_weight": 0.5,             // clamp 0..5
  "blacklist_radius_m": 0.30,     // clamp 0.05..2
  "max_consecutive_failures": 3   // clamp 1..10
}
```

### 3.2 `node.py` 探索状态机

状态：`idle | active | paused | completed | cancelled | failed`。快照字段（WS `state.exploration`，前端按此渲染）：

```jsonc
"exploration": {
  "state": "idle",
  "message": "",
  "since": 123.0,               // 进入当前状态的 monotonic
  "frontiers": 12,              // planner.stats().frontiers（最近一次选择时）
  "goals_visited": 3,
  "blacklisted": 2,
  "consecutive_failures": 0,
  "last_goal": [x, y],
  "frontiers_preview": [[x, y], ...]   // ≤200，active/paused 时 1 Hz 刷新
}
```

行为：
- `start_exploration()`：前置检查（explore.enabled、无急停、无姿态切换、链路连接、有地图、`map_pose()` 有效；ssh_ros 链路额外要求 `status.mapping.state == "running"` 或 localization 有效，否则返回“请先启动建图”）。通过则 `explorer.reset()`、state=active、事件 `exploration_start`。paused→active 为“继续探索”（不清黑名单）。
- `stop_exploration(reason)`：state=cancelled、`nav.cancel`、`safety.clear_nav()`、事件。
- `tick()` 内（在导航更新之前）：
  - active 且（急停 / 姿态 / 操作者心跳超时 / 链路断 / pose 丢失 / ssh_ros 建图停止）→ cancel（各自中文 message）；
  - active 且 nav 状态 arrived → `goals_visited += 1`，继续选下一个；
  - active 且 nav 状态 blocked/stuck → `explorer.blacklist(当前探索目标)`、`consecutive_failures += 1`；≥ max_consecutive_failures → state=failed；
  - active 且 nav idle（含上一目标完成后）→ `next_goal()`：None → state=completed（事件 `exploration_complete`）；有目标 → 内部调用与 `set_goal` 相同的规划入口（规划失败 → blacklist 该目标并计一次失败，下 tick 重试）；成功 → `consecutive_failures = 0`，记录 `last_goal`；
  - active/paused 时按 1 Hz 刷新 `frontiers_preview`。
- 手动接管：`set_teleop` 有实际输入 → exploration 转 paused（“手动接管，探索暂停”）；用户点选新目标 `set_goal` 成功 → exploration 转 paused（“切换手动目标，探索暂停”）。手动导航到达/取消后探索**不自动恢复**，需再次点「开始探索」。
- 录制行 `telemetry.jsonl` 增加 `exploration` 字段（state）。

### 3.3 `server.py`

- WS 消息：`exploration_start` / `exploration_stop`（需要操控权，与 `goal` 同级处理）。
- REST：`POST /api/explore/start`、`POST /api/explore/stop`（鉴权同其他控制端点）。
- `snapshot()` 已含 link.mapping / localization；新增 `exploration`（3.2 的 payload）。

## 4. frontend-dev 交付：探索 UI（导航操控工作区）

1. **控制按钮**（地图工具条，紧邻「开始建图/停止建图/保存地图」）：`开始探索`（id `exploration-start`）/ `停止探索`（id `exploration-stop`）。
   - 置灰逻辑与现有 mapping 按钮一致：无操控权/只读链路/急停/姿态切换中禁用；`exploration-start` 在无地图或（ssh_ros 且建图未运行且定位无效）时禁用并给 title 提示“请先启动建图”；active 时 start 禁用、stop 可用；paused 时 start 显示为「继续探索」可用。
2. **状态徽标** `#exploration-state`：`探索待命/探索中/探索暂停/探索完成/已停止/探索失败`，颜色沿用 badge ok/warn/soft/err 语义；title 显示 `message`。
3. **统计 chips**：前沿簇数、已访问目标数、拉黑数（来自 `snapshot.exploration`）。
4. **地图覆盖层**（`map.js`）：`frontiers_preview` 画小青色圆点（半径 3px，半透明），仅在 exploration active/paused 时绘制；当前 `last_goal` 复用现有 goal 渲染（导航 goal 本就由 nav 快照给出）。
5. **WS**：发送 `exploration_start` / `exploration_stop`；处理 ack/事件 toast；事件码 `exploration_start/exploration_complete/exploration_cancel/exploration_goal/exploration_failed` 走现有事件时间线渲染（中文 message 由后端给出）。
6. 手动接管提示：teleop 触发 paused 时 toast 一次“手动接管，探索暂停（可点继续探索恢复）”。
7. 自测：`node --check` 通过；起 8721 mock 实例，配合 Lead 的 mock 渐进建图（§5.3）截图四状态（待命/探索中/暂停/完成）到 `tmp/frontend_exploration_*.png`；控制台无 error。

## 5. link-dev 交付

### 5.1 `lite2_sim/hardware/navigation_mapping.launch` v2（自包含）

以 `tmp/mapping_rslidar_indoor.launch.snapshot`（机载真实文件）为参数基准重写，**不再 include 机载 hdl launch**：

- `<arg name="start_lidar_driver" default="false"/>`；仅当 true 时 `<include file="$(find rslidar_pointcloud)/launch/rs_lidar_16.launch"/>`。
- 复制 snapshot 中的：static TF `base_link→rslidar 0.243 0 0.058 0 0 0`、nodelet manager、PrefilteringNodelet（distance 0.4–100、VOXELGRID 0.1、RADIUS 0.5/2）、ScanMatchingOdometryNodelet（FAST_VGICP、odom frame、keyframe 0.25/1.0）、HdlGraphSlamNodelet（map/odom frame、回环与配准参数**逐项照抄 snapshot**）、`map2odom_publisher.py`。floor detection 不启用（保持 v1 的 `enable_floor_detection=false` 语义：直接不放该 nodelet）。
- octomap_server：`frame_id=map`、`resolution=0.05`、`latch=true`、`sensor_model/max_range=12.0`、`cloud_in→/filtered_points`、`pointcloud_min_z=0.15`、`pointcloud_max_z=1.5`、`occupancy_min_z/max_z` 同（沿用 v1 数值）。
- XML 注释注明：参数来源 `tmp/mapping_rslidar_indoor.launch.snapshot`（2026-11-14 机载抓取），改动需重新实机验证。

### 5.2 `ssh_ros_stream.py`（Python 2.7 兼容！）

1. `mapping_worker("mapping_start")`：启动前用 `rospy.get_master().getSystemState()` 检查 `/rslidar_points` 是否已有发布者；有 → `roslaunch <launch>`（默认不带驱动）；无 → `roslaunch <launch> start_lidar_driver:=true`。把该决策写进回执 message（如“建图已启动（复用机载雷达驱动）”）。
2. 失败诊断：启动失败/进程早退时读取 `/tmp/airace_mapping.log` 末尾 ~800 字节，拼进 control 回执 message（截断、去换行）。
3. `odometry_received`：emit 节流 ≤10 Hz（TF 查询可留在回调内，emit 前判断 `time.time() - last_localization_emit >= 0.1`）。
4. 其余（姿态/急停/深度心跳/地图/state）**不许动语义**；`test_deployment_sources.py` 里已有的源码特征锁不能破。

### 5.3 `mock.py`：渐进建图仿真（离线 E2E 的关键）

- `capabilities` 增加 `"mapping": True`。
- `set_mapping(running) -> (ok, msg)`：
  - True：初始化与 demo 地图同尺寸的**全未知**掩膜 GridMap（name="map"）；立即做一次局部披露；置 `_mapping="running"`（status().mapping 反映 `{"state","message"}`，含 starting/running/stopping/idle 过渡，模拟 0.3 s 启动延迟可简化为立即）。
  - False：停止掩膜更新，最后再 emit 一次掩膜地图；state=idle。
- `_step` 中当 mapping running：≤2 Hz 从当前位姿向 360° 打 ~120 条射线（复用 `_raycast` DDA，range 4.0 m）：沿途格写 FREE(0)、命中格写 OCCUPIED(100)；≤1 Hz 通过 `callbacks.on_map` 推送掩膜 GridMap（拷贝，避免外部改内部缓冲）。
- `status()` 增加：`"mapping": {...}` 与 `"localization": {"valid": running, "x","y","yaw_deg","age_s":0.0,"source":"mock","frame":"map"}`（值 = 当前仿真位姿；node 对 mock 的 pose 走 frame 分支，这里仅供 UI 徽标一致性）。
- 真实网格 `self._map`（碰撞/射线用）**保持不变**，掩膜是独立对象。
- 注意：`start()` 时仍推送完整 demo 地图（保留旧行为）；`set_mapping(True)` 后掩膜地图覆盖它。

### 5.4 测试

- `test_deployment_sources.py`：新增源码特征锁——launch 含 `start_lidar_driver` 条件 include 且**不含**对 `mapping_rslidar_indoor.launch` 的 include；stream 含 `/rslidar_points` 预检与 localization 节流特征；保持 py2.7 兼容特征检查。
- `test_mock_transport.py`：mapping 能力、set_mapping 启停、掩膜地图从全未知到局部披露（on_map 收到 name="map" 且未知格减少）、status().mapping/localization 字段。
- `test_ssh_ros.py`：如 `set_mapping` 行为有变则更新（现有 mapping_start/stop 命令流不变，应无需大改）。

## 6. 验收

1. `python -m pytest host_ui/tests -q` 全绿（在 168 基线上新增 ≥20 用例）。
2. **离线 E2E**（Lead 出脚本 `tmp/exploration_e2e.py`，mock 链路）：start mapping → 掩膜地图出现 → exploration_start → 机器人自主移动、地图渐进披露 → 访问 ≥3 个前沿目标 →（可选）completed；全程无 console error、事件时间线完整。截图/JSON 存 `tmp/`。
3. **实机验收**（Lead 主持，用户监护，机器人可达时执行；任何运动前口头确认）：
   a. 只读体检 → 站立 → 使能；
   b. **前进后退**：W 前进 2 s / S 后退 2 s（≤0.05 m/s），核对位移方向、松手零速、停车距离（用 telemetry + 外部参照）；
   c. **建图**：UI「开始建图」→ 确认 roslaunch 成功（无双驱动冲突）、`/projected_map` 上图、定位徽标“定位有效”；手动慢速走一段 → 地图增长 → 「保存地图」；
   d. **导航**：点选 2–3 m 内目标 → 自主前往 → 到位停车；期间测一次急停；
   e. **探索**：「开始探索」→ 观察自动选前沿、移动、地图披露；「停止探索」立即停；
   f. 每步证据存 `tmp/live_nav_mapping_*.json`，更新 `REAL_ROBOT_CHECKLIST.md`。
4. 文档：`host_ui/README.md`（§3.3 探索、§4 话题表）、`AUDIT` 批次记录、根 README 如需要。

## 7. 安全红线（所有人）

- 探索/导航的一切速度输出必须经过 SafetyCore（急停、姿态、状态超时、遥控接管、deadman、限幅、加速度）与机载 execution_guard 双层拦截；**不得新增旁路**。
- 操作者心跳超时（`nav_operator_timeout_s`）必须同时取消导航与探索。
- 实机运动测试只能在用户现场监护、明确同意后进行；脚本默认只读。
- `ssh_ros_stream.py` 的单调时钟/深度心跳/姿态单次 toggle 语义是 1.3.0 实机验证过的行为，**不许回退**。
