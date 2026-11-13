# DeepSeek 干的活：机器狗仿真自主建图 / 完整链路 / 随机化 —— 阶段总结

> 生成时间：2026-11-12 傍晚 ｜ 工作区：`F:\airace` ｜ 仿真工程：`F:\airace\lite2_sim`
> 本文只写有证据的结论；没跑通的地方照实记为缺口，不修饰、不虚构。

---

## 0. 一句话结论

- **自动规划建图：部分达成**。单目标在线建图导航在标定场（`sensor_lab`）通过；开环探索能建出与真值高度一致的地图，但未收敛到「建图完成」。
- **完整 9 段链路：未跑通**。卡在第一段 `ramp` 的坡道内侧墙「角点净空」碰撞；已定位根因、给出下一步方案，但本轮窗口内没做出来。
- **随机过程 / 为实机部署准备：基础设施达成，鲁棒性不达标**。采样可复现、审计链完整、105 项测试全绿；`sim2real_v1` 随机化下导航成功率 4/10（内部门槛 8/10），失败模式已统计。

---

## 1. 环境与项目结构（先回答「读取我的项目结构」）

| 项 | 内容 |
| --- | --- |
| 仿真引擎 | MuJoCo 3.17.0，Python 3.12.13，独立虚拟环境 `lite2_sim\.venv-mujoco`（避开本机 Anaconda 运行库冲突） |
| 机器人 | DEEP Robotics Lite2，本地厂商 URDF；速度级 mocap 夹具执行器，四肢保持配置姿态（**无步态/接触动力学**） |
| 场地 | 12×10 m 标注尺寸场地：坡道 12°/高 0.45 m/净宽 1.2 m → S 弯 → 死胡同出口 → 乱石 → 圆管（内径 1 m）→ 窄口拒绝 → 方涵 → 终点 |
| 导航核心 | `lite2_sim/controllers/sensor_capture/navigation_core/`：占据栅格 log-odds、扫描匹配、A*（膨胀 + 优选净空）、可达前沿探索、局部速度采样、地形危险判据（坡度 >15°、台阶 >0.08 m、顶空 <0.60 m→后续 0.45 m） |
| 课程编排 | `course.py` 的 `CourseSession` / `CourseEvaluator`，9 段、检查点、`course_verification.json` |
| 评测 | 估计与真值分离：`evaluation.py` 用独立真值算终点误差、碰撞、恢复次数 |
| 随机化 | `navigation_core/randomization.py` + `config/randomization/*.json` + `tools/run_monte_carlo.py` |
| 文档 | `总体技术方案.md`（总体方案）、`plan/2026-11-12-autonomy-randomization.md`（本轮协调文档）、`lite2_sim/README.md`（运行手册） |

---

## 2. 本轮谁改了什么

### 2.1 Lead（我）

- `lite2_sim/run_mujoco.py`
  - 接随机化：`--randomize <profile>`、`--explore`，二者互斥；随机化 profile 校验失败时**不创建 run 目录**、直接退出码 2；
  - 把 `randomization_profile` 写入 `config_snapshot.json`，profile 文件进入 `source_snapshot/` 与 `source_hashes.json`；
  - 修复 `--course-stage` 起点位姿问题：段起点在 0.45 m 平台顶/管道口时，按局部支撑面高度生成机体（否则焊缝与平台对抗，fixture drift 可达 0.026 m）；
  - 段模式校验语义：`pose_dependent_checks=False`（跳过与标准起点绑定的单条西墙量程、焊缝容差 0.005→0.02 m），实际漂移仍写入 `verification.json`。
- `lite2_sim/mujoco_navigation.py`：随机化构建、执行器噪声/延迟/滑移、初始位姿误差、探索模式、`traversability_audit`。
- `lite2_sim/mujoco_capture.py`：`verify()` 增加 `pose_dependent_checks` 参数与容差记录。
- `lite2_sim/tools/build_navigation_acceptance.py`：一条命令重跑「双场景导航 + 5 类故障场景 + 回放 + 回归」，可附加课程/探索/蒙特卡洛证据与 `--gap` 缺口清单。
- `lite2_sim/tools/run_tests.py`：受限环境下可用的测试入口（`tempfile` 目录改到可访问位置）。
- `lite2_sim/tests/test_randomization_wiring.py`：7 项接线测试。
- 文档：`lite2_sim/README.md`、`总体技术方案.md` §9.1 实测状态段。

### 2.2 nav-autonomy（队友，任务 1）

`navigation_core/{terrain,session,planner,control,grid,course}.py` + `tests/test_exploration.py`：

- 坡道入口「假障碍/起步即退出」：地面连通洪水填充 + 高度场 4 轴坡度（取代点法向坡度），12° 坡不再误判 15° 陡坡；墙体顶面不再被误清为自由。
- 原地自旋死循环：前沿目标避开自车盲区（≥0.55 m）、已访问前沿计权、`frontier_reached` 计入 visited。
- 有界恢复：`start_blocked/goal_blocked/map_disconnected/insufficient_observations/stalled` → 旋转 1.2 s → 倒车 ≤0.08 m/s 0.8 s（**永不前进**）→ 稳定 0.5 s，上限 3 次；事件 `recovery_started/succeeded/retry/exhausted`。
- 近场占用 4 帧确认（单帧毛刺不抬高占用）。
- 探索模式：`goal=None`、`TERMINAL_STATES += map_complete`、`exploration_summary()`。
- `config/ruins_reference.json` 的 `mujoco_navigation` 覆盖块：`robot_radius_m 0.34`、`planning_clearance_m 0.26`、`body_clearance_height_m 0.45`、恢复/探索参数。**course 段字段一个没动、场景几何没动。**

### 2.3 sim-random（队友，任务 2）

`navigation_core/randomization.py`、`config/randomization/{ideal,sim2real_v1,sim2real_aggressive}.json`、`tools/run_monte_carlo.py`：

- 每 profile 20 个参数，区间或定值；独立 `numpy` Generator 按 `(profile, seed)` 采样，同种子逐字节一致、不污染全局随机；
- 参数来源标记 `sim_assumed` / `vendor_reference` / `measured_pending`（实机测量后逐项替换）；
- 覆盖：雷达测距噪声/偏置/丢点/波束遮挡/整帧丢包/扫描延迟，里程计比例误差/噪声/偏航零偏/打滑，IMU 倾角噪声，指令延迟/加速度比例/线速度增益，初始位姿误差；
- MC 并发多 seed，输出 `report.json` / `report.md`：成功率、真值误差分布、逐段通过率、失败原因直方图、碰撞与恢复、逐 seed 采样参数、`--min-pass-rate` 门槛。

---

## 3. 能力矩阵（真实状态）

| 能力 | 结果 | 证据 |
| --- | --- | --- |
| `sensor_lab` 单目标导航（理想传感器） | ✅ 通过：终点真值误差 0.093 m、45.1 s 停车、0 碰撞 | `lite2_sim/runs/mujoco_20261112_164940_b469e1`；队友复跑 `..._173400_81f296`（45.8 s，passed） |
| `ruins_reference` 单目标导航 | ❌ 未到点：出起点走廊必须过坡道，卡在同一瓶颈 | `lite2_sim/runs/mujoco_20261112_173450_e0678a` |
| 9 段连续课程 | ❌ 0/9：ramp 首段即终态 | 见 §4 |
| `--explore` 自动建图（sensor_lab） | ⚠️ 部分：751.9 s / 80.75 m / 35 前沿 / 7519 帧；地图误标占据比 1.4e-4；但以 `stalled` 结束，未到 `map_complete` | `lite2_sim/runs/mujoco_20261112_173237_2095be`（含 `exploration` 块与 `traversability_audit`） |
| 随机化可复现 + 审计链 | ✅ 通过：同种子逐字节一致；`randomization.json` + `config_snapshot` + `source_snapshot` 闭环 | `lite2_sim/tests/test_randomization_wiring.py`、各 run 目录 |
| `sim2real_v1` 鲁棒性（sensor_lab 导航，10 seeds） | ❌ 4/10（目标 8/10）：`goal_blocked`×5、`stalled`×1，0 碰撞，真值误差均值 1.95 m | `lite2_sim/runs/monte_carlo_sensor_lab_v1/report.md` |
| 回归测试 | ✅ 105 项全绿 | `python tools/run_tests.py` |

---

## 4. 完整链路缺口：ramp 内侧墙「角点净空」

### 4.1 事实

- 最好的一次：`lite2_sim/runs/mujoco_20261112_170820_4d9fe0`
  从 (-5.02,-4.24) **连续爬完 12° 坡 7.38 m** 到 (-5.02,2.80)，地面高度估计 0.296–0.30 m（坡度识别正确），522 帧扫描；
  t=74.4 s 在 y≈2.80 m 发生一次 `body_geometry_clearance` 碰撞，几何名 `ramp_inner_edge_collision_0`，机身距内侧墙 0.27–0.29 m。课程规则下**单次碰撞即终态**。
- 规划半径改 0.37（≈角点半径）后 0 碰撞，但 1.2 m 走廊可通行带只剩 0.456 m，多次复跑 120 s 出不了走廊（`..._172147_4cb978`、`..._172912_a92dc4`、`..._173237_2095be`）。
- 当前折中 `robot_radius_m=0.34`：坡道横向余量 0.52 m，仍不足以把机身稳定压在中心线。

### 4.2 根因（已定量）

1. 坡道净宽 1.2 m；机身箱体 0.60×0.44 m（`navigation_envelope` box，半宽 0.30、半长 0.22），**角点半径 √(0.30²+0.22²)=0.372 m**。
2. 全局规划按 0.28 m 圆膨胀（后调 0.34），A* 认为「离墙 0.28 m 即安全」，但箱体角点会侵入 0.09 m。
3. 坡道内墙栅格长期停在 log-odds 0.35：只被地形高度跳变标记为 hazard，**既不进占据、也不进膨胀**，软净空代价（0.16→0.26）盖不住它。
4. 局部控制倾向「朝目标点直线追」，在 1.2 m 宽 + 12° 坡上产生 0.27–0.29 m 侧偏，正好落在碰撞带上。

### 4.3 下一步方案（未在本轮完成，估 1.5–2 h）

- 贴墙**硬约束**：距墙 <0.37 m 的栅格给高代价或直接禁行（不再只靠软净空）；
- 真正的**路径跟随**（跟踪 A* 路径的最近点 + 横向误差修正），而不是朝目标点直线追；
- 原地转向前的**角点扫描**（把 0.60×0.44 箱体按当前朝向做角点检查）；
- 可选：坡道段局部把 `robot_radius_m` 提到 0.36–0.37 并配合路径跟随（单独提半径已验证会振荡，必须两件事一起）。

### 4.4 为什么不能拿「逐段通过」凑数

`--course-stage` 除 ramp 段（与场景默认起点重合）外，其余段起点分别在 0.45 m 平台顶或管道口内，自车在起点即「未观测可通行」或与管壁接触（`start_blocked` / 首帧 `collision`）。这是**段起点位姿问题**，不是通过能力，不能作为逐段证据。段模式只跳过与标准起点绑定的单条西墙量程检查，其余模型/传感器校验照跑（详见 `verification.json` 的 `pose_dependent_checks` / `fixture_drift_m`）。

---

## 5. 自动建图（探索）实测细节

`lite2_sim/runs/mujoco_20261112_173237_2095be`（sensor_lab，`--explore --seed 42`）：

| 指标 | 数值 |
| --- | --- |
| 仿真时长 | 751.9 s（以 `stalled` 终止，非 `map_complete`） |
| 行驶距离 | 80.75 m |
| 访问前沿 | 35 |
| 整合扫描 | 7519 帧 |
| 已知空闲 / 占据 / 未知 | 8850 / 752 / 38398 栅格 |
| 机身实际经过栅格 | 7071 个，其中被地图误标占据 **1 个**（`traversability_contradiction_ratio` = 1.4e-4） |
| 恢复次数 | 4 |
| 地图矩形完整度 | 20.0%（`map_completeness_ratio`，分母含场地外矩形，仅作参考） |

结论：**建图本身质量很好**（经过区域几乎无误标），缺的是「何时算建完」的收敛判定——前沿耗尽 + 恢复用尽后没有走到 `map_complete`，需要继续调。

---

## 6. 随机化与实机准备

- 三个 profile：`ideal`（全零基线）/ `sim2real_v1`（迁移前热身）/ `sim2real_aggressive`（压力测试）。
- 采样与审计：`randomization.json`（profile_id、seed、逐参数值、provenance）+ `config_snapshot.json.randomization_profile` + `navigation_summary.json.randomization` + `source_snapshot/` 中的 profile 文件；`ideal` 全零时不消耗额外 RNG，**理想基线行为与随机化前逐位一致**。
- Monte Carlo（`lite2_sim/tools/run_monte_carlo.py --scene sensor_lab --navigate --profile sim2real_v1 --seeds 42-51 --jobs 3`）：
  - 10 次 **4 次通过**（0.4），失败：`goal_blocked`×5、`stalled`×1；0 碰撞；真值误差均值 1.95 m、标准差 1.92 m、最好 0.055 m、最差 5.36 m。
  - 说明：当前规划/控制对传感器噪声与初始位姿误差偏敏感，恢复策略覆盖不足，是迁移前最该补的项。
- 训练/评测分离：随机化只作用于传感器与执行器模型，评分始终用独立真值。
- 已知限制：`navigate` 模式未把恢复次数写进 MC 汇总（记 0），恢复事件仅在课程模式透传——这是**指标接线缺口**，不是行为缺失。

---

## 7. 复现命令（照抄可用）

```powershell
Set-Location F:\airace\lite2_sim

# 1) 单目标导航（理想基线，标定场）
.\.venv-mujoco\Scripts\python.exe .\run_mujoco.py --scene sensor_lab --navigate --headless --seed 42

# 2) 废墟场单目标导航（当前会卡坡道走廊）
.\.venv-mujoco\Scripts\python.exe .\run_mujoco.py --scene ruins_reference --navigate --headless --seed 42

# 3) 连续课程（当前未通）
.\.venv-mujoco\Scripts\python.exe .\run_mujoco.py --scene ruins_reference --course --headless --seed 42

# 4) 随机化课程 / 随机化导航
.\.venv-mujoco\Scripts\python.exe .\run_mujoco.py --scene ruins_reference --course --headless --seed 42 --randomize sim2real_v1

# 5) 自动建图（无固定目标）
.\.venv-mujoco\Scripts\python.exe .\run_mujoco.py --scene sensor_lab --explore --headless --seed 42

# 6) 蒙特卡洛 10 seeds
.\.venv-mujoco\Scripts\python.exe .\tools\run_monte_carlo.py --scene sensor_lab --navigate --profile sim2real_v1 --seeds 42-51 --jobs 3 --out runs\monte_carlo_sensor_lab_v1

# 7) 回归测试（不要用 python -m unittest）
$env:LITE2_TEST_TMPDIR='F:\airace\tmp\work\pytest-temp'
.\.venv-mujoco\Scripts\python.exe .\tools\run_tests.py

# 8) 重新生成导航验收记录
.\.venv-mujoco\Scripts\python.exe .\tools\build_navigation_acceptance.py `
  --runs "ruins_reference:42 ruins_reference:43 sensor_lab:42" --jobs 3 `
  --explore-run runs\mujoco_20261112_173237_2095be `
  --monte-carlo-report runs\monte_carlo_sensor_lab_v1\report.json
```

---

## 8. 验收记录状态

- 目标文件：`lite2_sim/mujoco/navigation_acceptance.json`（包含 runs、回归、故障场景、回放、探索、课程、蒙特卡洛、`known_gaps`）。
- 旧文件（15:49 版）已过期；本文写作时正在用当前源码复跑（3 个导航 run + 5 类故障场景 + 回放 + 105 项回归 + 探索/蒙特卡洛证据 + 6 条 known gaps），跑完直接覆盖。
- 故障场景检查（cancel / timeout / lidar_loss / localization_loss / dynamic_obstacle）以 `scenario_verification.json` 的预期结果为准；如本轮语义有变化（例如恢复策略把 `timeout` 变成 `stalled`），以**实际结果 + 说明**记录，不改预期来凑通过。

---

## 9. 已知缺口清单（与验收文件 `known_gaps` 一致）

1. 连续 9 段链路未跑通：ramp 首段坡道内侧墙 `body_geometry_clearance` 碰撞，单次即终态；根因 1.2 m 净宽 vs 0.60×0.44 m 机身角点半径 0.372 m 下 A* 可行、局部侧偏 0.27–0.29 m。
2. `ruins_reference` 单目标导航未到点：出走廊必须过坡道，受同一瓶颈阻断。
3. 探索模式不收敛：751.9 s / 80.75 m / 35 前沿 / 7519 帧后仍 `stalled`，未到 `map_complete`；但地图与真值一致性良好（误标占据比 1.4e-4）。
4. `sim2real_v1` 随机化导航成功率 4/10（目标 8/10），失败 `goal_blocked`×5、`stalled`×1。
5. 无步态/接触动力学（速度级 mocap 执行器，四肢保持配置姿态）；无回环检测；硬件验证待传感器安装、外参标定与厂商接口核对。
6. `--course-stage` 逐段取证受段起点位姿限制（除 ramp 外，其余段起点即碰撞/未观测可通行），不能当逐段能力证据。

---

## 10. 诚实声明

本文所有「通过」结论都有对应的 run 目录与 JSON 记录；所有「未通过」都是真实复跑结果，没有为了好看调整判据、放宽碰撞阈值或删改失败记录。唯一放宽的是段模式的启动校验容差（焊缝 20 mm）并跳过与标准起点绑定的单条西墙量程检查——它已写进 `verification.json`，且不影响导航结果判定；非段模式（课程全跑、单目标导航）仍用严格 5 mm 容差与全部量程检查。

---

# 附：22:36 复读更新（本节取代前文 §0/§4 的进度结论）

> 前文写于 17:52。20:58–22:00 期间**另一条工作线**（workbuddy / 其他会话，非本 Team）继续改了导航代码并跑了 60+ 次实验，课程链路已有实质进展。以下是复读当前工作区后的事实。

## 更新后的结论

- **连续 9 段课程：从 0/9 推进到 4/9**。最佳证据 `lite2_sim/runs/mujoco_20261112_211915_fdd40c`（21:19，`--course`，timeout 已放宽到 1200 s）：
  - `ramp` ✅（t=68.25 s 过检查点）→ `s_bend` ✅（3 个检查点）→ `dead_end_exit` ✅（2 个检查点，含 `reverse=True` 倒车检查点）→ `south_rubble_route` ✅（3 个检查点）→ `rubble_return` 第 0 个检查点后 `goal_blocked`（1 次恢复），其余 4 段未开始。
- **单段可独立通过**：`--course-stage` 2 段模式复跑证据：`ramp` 通过（`..._210714_39b284`）、`dead_end_exit` 通过（`..._211755_242114`）、`rubble_return` 通过（`..._212723_92a9fb`）。
- **当前瓶颈：`round_pipe`（圆管）**。21:42–21:59 连续 6+ 次失败：撞 `concrete_pipe_65` / `concrete_pipe_82`（管壁）、`culvert_cover_slab`（盖板），或进管后在 x≈3.9–4.0、y≈0.98 处 `stalled`。几何矛盾与 ramp 同类：内径 1.0 m ＋ 机身 0.60×0.44 m（角点半径 0.372 m）＋ 规划半径 0.34 m，可通行带只剩 0.32 m。
- **22:00 之后没有新 run，也没有运行中的仿真进程**：工作停在「圆管段调试中断」状态，代码最后一次改动是 21:55–21:58（`session.py`、`course.py`、`config/ruins_reference.json`）。

## 这轮改了什么（与 17:40 快照逐文件 diff）

| 文件 | 变化 | 新机制 |
| --- | --- | --- |
| `navigation_core/session.py` | +62 / −6 行 | 2.5D 高度场支撑连续性 `_support_continuous()`（按 `max_step_m`＋距离×坡度放宽判据）、受阻前沿访问加权、恢复细节 |
| `navigation_core/terrain.py` | +32 / −23 行 | `traversable_step_mask()`、连续支撑面清除 `_clear_continuous_surface()`（不再整块判死坡面） |
| `navigation_core/course.py` | +12 行 | `_observed_barrier()`：用雷达点实测确认「死胡同封闭支路」与「窄口宽度不足」，窄口判据 = 机身宽 0.44 m ＋ 2×0.08 m 余量；检查点支持 `reverse` / `forward_only` |
| `mujoco_navigation.py` | +28 / −1 行 | 支撑/台阶/速度模型参数接线 |
| `run_mujoco.py` | +27 / −1 行 | `support_height()` 段起点生成高度（我 17:45 加的）＋后续参数 |
| `config/ruins_reference.json` | 21:58 更新 | `mujoco_navigation` 新增 `max_step_m 0.12`、`velocity_model_underbody_clearance_m 0.18`、`velocity_support_lookahead_m 0.32`、`local_unknown_fraction_limit 0.35`；`course.timeout_s 1200`、`probe_timeout_s 4`、`body_width_m 0.44`、`clearance_margin_m 0.08`；限速 0.15 m/s、`robot_radius_m 0.34`、`planning_clearance_m 0.26` |

## 当前健康度

- 测试：`tools/run_tests.py` **106 项全绿**（比 17:52 多 1 项）。
- 文档/验收**滞后于代码**：`lite2_sim/README.md` 仍是 17:49 版（写的是 0/9、ramp 碰撞）；`mujoco/navigation_acceptance.json` 仍是 15:49 旧版；`总体技术方案.md` 22:33 版新增了 §3.1–3.3「分层范式（L1 拓扑 / L2 通行代价 2.5D / L3 恢复）」与 §9.1 当前缺口，但 §9.1 的实测段仍写着「9 段未跑通；ramp 碰撞」，需要按 4/9 更新。
- 工作区另有独立产出：`plan/四足机器人导航避障文献综述与选型建议.md`（284 行）、`plan/四足机器人导航避障文献逐篇流程图讲解.md/html`、`research/` 思维导图应用、`四足机器狗导航_顶刊方案思维导图.md/html`；`plan/.workbuddy/memory/2026-11-12.md` 的「当前工程阻塞」记的还是 17:52 的 `ramp → start_blocked`，已过期。

## 下一步（按优先级）

1. **圆管段**：中心线引导 / 进入前对准（yaw≈0，管轴）+ 按段覆盖规划半径（例如管段 0.30、其他段 0.34）+ 角点扫掠检查；参考 ramp 经验，别只调半径。
2. **rubble_return 收尾**：它已能单段通过，连续链路在首检查点后 `goal_blocked`，属 L3 恢复/重规划问题（与 §3.3 文档建议一致）。
3. 打通后立刻重跑 `--course --seed 42/43/45` + `sim2real_v1`，再更新 `mujoco/navigation_acceptance.json` 与 README/§9.1 的实测数字。