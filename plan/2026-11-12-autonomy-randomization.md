# 自主建图导航 + 随机过程 工作计划（2026-11-12）

目标：机器狗在 MuJoCo 废墟场中完成“在线建图 → 自动规划 → 全程通行 → 到点停车”的完整链路，
并加入随机过程（传感器噪声、里程计偏差、执行延迟、初始位姿误差等），为实机部署做迁移准备。

## 1. 已确认的当前状态

| 能力 | 证据 | 状态 |
| --- | --- | --- |
| 单目标导航（在线建图 + A* + 前沿探索 + 局部速度采样） | `runs/mujoco_20261112_155416_a0fe83` 等：seed 42 到达，真值误差 0.017 m | 通过 |
| 完整赛段链路 `--course`（9 段） | `runs/mujoco_20261112_162602_977fde`：stage `ramp` 探索至 (-5.19, 1.25) 后 `start_blocked` 终止 | 未通过（仅 1/9 段从未完成） |
| 随机过程 | 仅 `odom_linear_noise_fraction`、`odom_yaw_noise_rad_s`；`lidar.noise_m=0`，无 dropout/延迟/初始位姿误差 | 不足 |
| 回归测试 | 41 项；其中 7 项在本机沙盒下因 `tempfile` 0700 目录不可写而报错（环境问题，非代码缺陷） | 需修正运行方式 |

`start_blocked` 的机理（可复现）：前沿区域探索到左廊道后，上一前沿被标记访问，新选前沿位于机器人身后；
同时估计位姿贴近墙体，`plan_grid` 的起点落在 0.28 m 膨胀区 → 直接进入终态 `start_blocked`，整段任务终止。
这说明当前缺少“受阻恢复”语义，而赛题要求“异常时停止或恢复”。

## 2. 交付物

1. **完整链路**：`run_mujoco.py --scene ruins_reference --course --headless --seed S` 在种子 42/43/44/45
   全段通过（`course_verification.json` 的 `all_stages_passed=true`、`collision_free=true`、停车 1 s）。
2. **自动建图模式**：`--explore` 从空地图出发，纯前沿探索直到无可行前沿，输出覆盖率与地图统计；
   与真值对照给出建图误差/覆盖指标（`navigation_summary.json` 内 `exploration` 块）。
3. **随机过程**：`--randomize <profile>` 按 profile 采样噪声/偏差/延迟/初始位姿误差，
   写入 `randomization.json`（含参数来源标签），同种子完全可复现；`tools/run_monte_carlo.py`
   批量多种子统计成功率、误差分布、碰撞与恢复次数，输出 `report.json`/`report.md`。
4. **验收与文档**：回归测试全绿（含新增用例）、回放与 5 类故障场景仍通过、
   `mujoco/navigation_acceptance.json` 按当前场景修订重新生成、README 与总体技术方案更新。

内部验收门槛（沿用总体技术方案 9.1）：代表性任务连续 10 次至少 8 次完成，且逐次记录碰撞/接管/失败原因。

## 3. 接口冻结（改动前确认，写入后不得单方面变更）

### 3.1 命令行

```text
run_mujoco.py --scene ruins_reference --course --headless [--seed 42]        # 完整链路
run_mujoco.py --scene ruins_reference --explore --headless [--seed 42]       # 纯自动建图
run_mujoco.py --scene ruins_reference --course --headless --randomize sim2real_v1 --seed 42
```

### 3.2 随机化配置块（`config/randomization/<profile>.json` → 采样后写 `config["randomization_sampled"]`）

```json
{
  "schema_version": 1,
  "profile_id": "sim2real_v1",
  "lidar": {"range_noise_m": [0.0, 0.03], "range_bias_m": [-0.02, 0.02],
            "dropout_probability": [0.0, 0.08], "beam_block_probability": [0.0, 0.05],
            "missing_scan_probability": [0.0, 0.05], "scan_latency_s": [0.0, 0.12]},
  "odometry": {"linear_scale_error": [0.90, 1.10], "yaw_scale_error": [0.92, 1.08],
               "linear_noise_fraction": [0.01, 0.05], "yaw_noise_rad_s": [0.004, 0.02],
               "yaw_bias_rad_s": [-0.01, 0.01], "slip_probability": [0.0, 0.03],
               "slip_ratio": [0.2, 0.8]},
  "imu": {"roll_pitch_noise_rad": [0.0, 0.01]},
  "actuation": {"command_latency_s": [0.0, 0.15], "acceleration_scale": [0.7, 1.3],
                "linear_gain_error": [0.9, 1.10]},
  "initial_pose": {"dx_m": [-0.08, 0.08], "dy_m": [-0.08, 0.08], "yaw_offset_rad": [-0.03, 0.03]}
}
```

`apply_to_lidar(values, lidar_settings)` 必须返回仅含以下键的更新字典（供 `config["lidar"]` 合并）：
`{"noise_m", "range_bias_m", "dropout_probability", "beam_block_probability", "scan_latency_s"}`（缺失键表示不覆盖）。
`executor_parameters(values)` 返回扁平键字典，键名即 `values` 中出现过的 `odometry.*`/`actuation.*`/`initial_pose.*`。

AI 智能体注意：区间 `[lo, hi]` 为均匀分布；`null` 表示该参数取基线值（由 profile 显式给出或省略）。
`randomization_sampled` 的结构为 `{"profile_id", "seed", "values": {扁平键: 数值}, "provenance": {扁平键: 来源标签}}`。
扁平键形如 `lidar.range_noise_m`。来源标签取值：`sim_assumed`（仿真假设）、`vendor_reference`（厂商资料）、
`measured_pending`（待实测）。默认不启用随机化时 `randomization_sampled` 为 `null`。

### 3.3 执行器/传感器扩展点（由 lead 实现，B 只写 `randomization.py` 与工具）

- `mujoco_navigation.MujocoVelocityExecutor`：读取 `config["randomization_sampled"]["values"]` 的
  `odometry.*`、`actuation.*`、`initial_pose.*`；初始位姿偏移同时作用于执行器真值位姿与会话先验（保持一致）。
- `mujoco_adapter.MujocoLidarObservationAdapter`：读取 `lidar.range_noise_m/range_bias_m/dropout_probability/
  beam_block_probability/scan_latency_s`。
- `mujoco_navigation.run_navigation`：读取 `lidar.missing_scan_probability` 决定该帧是否丢弃；记录 `randomization.json`。

### 3.4 自动建图契约（A 在 `navigation_core/` 内实现，lead 接线）

- `session.settings["exploration_mode"] = true` 时：`session.goal is None`；`plan_grid` 不做目标 A*，
  只选可达前沿（`PlanResult("exploring", ...)`，`goal_xy=None`）；本地控制沿用现有跟踪与受阻恢复。
- 无可行前沿且连续 `exploration_settle_s`（建议 6 s，参数名固定，缺省 6.0）内地图不再出现新前沿时，
  `status = "map_complete"` 且 `finished = True`；`TERMINAL_STATES` 增加 `"map_complete"`。
- `session.exploration_summary()` 返回（缺失字段用 0 兜底）：
  `{"known_free_cells", "occupied_cells", "unknown_cells", "frontiers_visited", "distance_m", "scan_count"}`。
- `map_complete` 由 lead 在 `run_navigation` 中计为成功（`checks` 增加 `map_complete`，`passed` 以它替代 `goal_reached`）；
  其余终态语义不变。探索模式不得触发 `start_blocked`/`goal_blocked` 终态（无目标），只能超时/定位失效/取消。
- `artifacts.save_map` 与 `Evaluation.summary` 由 lead 支持 `goal=None`（不画目标点、误差字段为 null）。

## 4. 文件归属（写作用域）

| 归属 | 文件 |
| --- | --- |
| A 自主导航 | `lite2_sim/controllers/sensor_capture/navigation_core/{session,control,planner,grid,course,terrain}.py`、新增 `exploration` 相关实现（必须落在上述文件或 `navigation_core/exploration.py`）、`config/ruins_reference.json` 的 `course`/`navigation` 调参、`tests/test_course_navigation.py`、`tests/test_exploration.py` |
| B 随机过程 | 新增 `lite2_sim/controllers/sensor_capture/navigation_core/randomization.py`、`lite2_sim/config/randomization/*.json`、`lite2_sim/tools/run_monte_carlo.py`、`tests/test_randomization.py`、`tests/test_monte_carlo.py` |
| lead | `lite2_sim/run_mujoco.py`、`lite2_sim/mujoco_navigation.py`、`navigation_core/mujoco_adapter.py`、`lite2_sim/tools/run_tests.py`、`README.md`、`总体技术方案.md`、`lite2_sim/mujoco/navigation_acceptance.json` |

禁止：A/B 不得修改场景几何（`wall_paths`/`rubble_piles`/`helipads`/`upper_left_ramp`/`partition_culvert` 等）、
不得修改 `tools/prepare_mujoco.py`? no：不得修改 `tools/prepare_mujoco.py` 与 `tools/scene_geometry.py`，不得直接编辑 `mujoco/models/*`。

## 5. 验证命令

```powershell
$py = "F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe"
& $py F:\airace\lite2_sim\tools\run_tests.py                     # 全量回归（沙盒安全）
& $py F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42
& $py F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --explore --headless --seed 42
& $py F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --randomize sim2real_v1 --seed 42
& $py F:\airace\lite2_sim\tools\run_monte_carlo.py --scene ruins_reference --course --profile sim2real_v1 --seeds 42-51 --jobs 3
```

每次运行写入独立 `runs/mujoco_*`；判定以 `course_verification.json`/`navigation_summary.json` 为准。

## 6. 风险与边界

- 场地尺寸仍为图像估算，`--course` 检查点/半径允许按实际可达性微调，但必须在 README 记录改动与理由。
- 运动执行仍是速度级 mocap 简化，不含真实足端接触与越障动力学；结论只覆盖上层自主软件链路。
- `start_blocked`/`goal_blocked` 的恢复必须有限次并留下事件记录，禁止无限重试掩盖真实不可达。