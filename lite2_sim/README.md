# Lite2 仿真

## Lite2 MuJoCo 在线建图导航

在工作区根目录运行下列命令。入口自动切换到 `lite2_sim/.venv-mujoco` 的独立 CPython 3.12 环境。首次配置见[工作区说明](../README.md)。

```powershell
python .\lite2_sim\run_mujoco.py --scene sensor_lab --check
python .\lite2_sim\run_mujoco.py --scene ruins_reference --check
python .\lite2_sim\run_mujoco.py --scene sensor_lab --navigate --headless
python .\lite2_sim\run_mujoco.py --scene ruins_reference --navigate
python .\lite2_sim\run_mujoco.py --scene ruins_reference --navigate --goal-x 4.9339 --goal-y 3.8074 --headless --seed 42
python .\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42
python .\lite2_sim\run_mujoco.py --scene ruins_reference --explore --headless --seed 42
python .\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42 --randomize sim2real_v1
```

默认场景为 `sensor_lab`。废墟场默认目标为右上停机坪，标定场默认目标为 `[2.0, -3.0]`。目标坐标使用场景 `map` 坐标系，起点位姿为已知先验。`--goal-x` 和 `--goal-y` 成对提供。成功到点并停车1秒返回0，失败返回非零退出码。

`--course` 按 `config/ruins_reference.json` 的 `course` 段依次执行坡道、S弯、死胡同退出、乱石往返、圆管、过窄入口拒绝、方涵和终点共 9 个检查点，`course_verification.json` 记录逐段真值判定、受阻证据和总分。`--explore` 不追固定目标，只探索可达前沿，地图稳定后以 `map_complete` 结束并输出 `exploration` 统计。`--randomize <profile>` 采样 `config/randomization` 下的随机化剖面，详见下节。

场景 JSON 的 `mujoco_navigation` 块可覆盖导航参数，`navigation.goal_xy_m` 提供默认目标，命令行目标与种子优先。关闭导航窗口或在本次结果目录创建 `cancel_navigation` 文件可取消任务。

### 运行模型与参数

共享导航会话处理带噪里程计、IMU 倾角、局部扫描匹配、在线占据地图、A*、可达前沿与局部速度采样。导航决策使用估计位姿。真实位姿用于仿真执行及独立评分。

- 物理步长2 ms，导航控制20 Hz，雷达10 Hz，全局重规划2 Hz。
- 速度级 mocap 执行器模拟加减速、转向响应、停车延迟及地面支撑，关节保持厂商模型配置姿态。
- 默认前进速度上限0.15 m/s，转向上限0.4 rad/s，指令有效期0.30 s。
- 栅格0.05 m，全局规划半径0.28 m，优选额外净空0.16 m。有向包络确认安全的起点可用0.24 m半径恢复规划。
- 局部检查0.60×0.44 m机身，以0.64×0.48 m软余量评价净空。边缘最多15%的栅格可保持未知，地图保留未知标记。
- 坡度上限15°，高度突变上限0.08 m。已升高机身的前向预测还核对观测支撑高度，防止驶离坡道边缘。
- 导航默认采用32层、每层360束、垂直视场120°的理想雷达。`mujoco_navigation_lidar` 可覆盖参数，近场覆盖依赖实机验证。固定机身检查沿用原场景传感器配置。

场地尺寸来自图片估算。2026年11月12日，用户授权将左上坡道出口改为沿蛇形通道斜向落地。执行包络、传感器外参和运动响应均为仿真假设。真实四足接触、回环优化与实机运动接口仍待验证。

### 记录与回放

每次运行写入独立的 `runs/mujoco_*` 目录。

- `navigation_verification.json` 保存任务结果、真实终点误差、碰撞拦截、停车和数值检查。
- `slam_map.pgm/.yaml/.png`、`terrain_map.npz` 保存地图、地形与估计轨迹。
- `planned_path.json`、`current_path.json`、`replan_history.json` 保存路径及探索历史。
- `slam_trajectory.csv`、`velocity_commands.csv`、`collision_events.json` 保存轨迹、速度与碰撞事件。
- `sensor_observations.jsonl.gz`、`navigation_inputs.jsonl.gz` 保存导航回放输入。决策记录按序引用传感器记录。默认关闭额外的原始雷达分块存档，需要时在 `navigation` 中设置 `record_raw_lidar: true`，生成 `lidar_*.npz`。
- 运行 `python tools/cleanup_run_logs.py` 预览日志清理，追加 `--apply` 执行。默认保留最近 10 次已完成运行的完整输入，可用 `--keep` 调整。更早运行保留结果、地图、轨迹及配置，删除大体积传感器与雷达记录，并写入 `log_cleanup.json` 标记。这些运行随后无法完整回放。未完成运行保留原数据。
- `config_snapshot.json`、`versions.json`、`source_hashes.json`、`source_snapshot/` 保存复现依据；启用随机化时 `source_snapshot` 还包含实际使用的 profile 文件，采样值写在 `randomization.json`。
- `navigation_summary.json` 汇总结果、检查项、真值误差、地图统计，以及 `randomization`/`exploration` 块；`course_verification.json` 汇总 9 个赛段的逐段判定与证据。
- 根目录的 RGB-D、IMU、关节及雷达快照对应启动检查，结束数据保存在 `final_capture/`，终点预览为 `scene_final.png`。

原始点云采用 `front_lidar`，重力对齐后的导航投影采用 `front_lidar_level`。相机光学坐标为 X 向右、Y 向下、Z 向前，机身坐标为 X 向前、Y 向左、Z 向上。深度单位为米，并保存有效像素掩码。

```powershell
.\lite2_sim\.venv-mujoco\Scripts\python.exe .\lite2_sim\tools\replay_navigation.py <运行结果目录>
.\lite2_sim\.venv-mujoco\Scripts\python.exe .\lite2_sim\tools\check_navigation_scenarios.py --scenario dynamic_obstacle
.\lite2_sim\.venv-mujoco\Scripts\python.exe .\lite2_sim\tools\run_tests.py
.\lite2_sim\.venv-mujoco\Scripts\python.exe .\lite2_sim\tools\build_navigation_acceptance.py
```

`run_tests.py` 与 `python -m unittest discover -s tests` 等价，并在受限环境下把临时目录改为可访问位置。`build_navigation_acceptance.py` 实际重跑双场景导航、5 类故障场景、回放与回归，重新生成 `mujoco/navigation_acceptance.json`，可附加 `--course-run`、`--explore-run` 和 `--monte-carlo-report` 把完整链路与随机化证据写入同一份验收记录。

回放逐帧比较状态与速度指令，同时支持早期未压缩日志。故障检查还支持 `cancel`、`timeout`、`lidar_loss` 和 `localization_loss`。预期故障的检查结果保存在 `scenario_verification.json`。

### 环境与实机迁移

```powershell
Set-Location F:\airace\lite2_sim
uv python install 3.12
uv venv --managed-python --python 3.12 --allow-existing .venv-mujoco
uv pip install --python .\.venv-mujoco\Scripts\python.exe -r requirements-mujoco.txt
```

锁定依赖见 `requirements-mujoco.txt`。独立解释器解决本机 Anaconda 运行库与 MuJoCo 的加载冲突。

实机迁移依次核对雷达、IMU、外参、时钟和里程计，回放实机数据，再核对厂商速度通道、运动模式、单位和停车行为，实测响应后开展低速到点验证。已有 `/cmd_vel` 配置及 UDP 整数指令示例保留待核对状态。

### 随机化与蒙特卡洛

`config/randomization/` 下每个 profile 只声明参数区间或定值，采样与审计由 `navigation_core/randomization.py` 完成：

- `ideal.json` 全零/全 1，等价于原有理想传感器基线，用于回归对照。
- `sim2real_v1.json` 覆盖雷达测距噪声与偏置、丢点、波束遮挡、整帧丢包、扫描延迟，轮式/腿部里程计比例误差与噪声、偏航零偏、打滑，IMU 倾角噪声，指令延迟、加速度比例与线速度增益，以及初始位姿误差。
- `sim2real_aggressive.json` 用于迁移压力测试。

采样使用独立 `numpy` Generator，同一 `(profile, seed)` 完全可复现且不污染全局随机状态；参数来源标记为 `sim_assumed`、`vendor_reference` 或 `measured_pending`，实机测量后逐项替换。随机化只作用于传感器与执行器模型，评分仍使用独立真值。

```powershell
python .\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42 --randomize sim2real_v1
python .\lite2_sim\.venv-mujoco\Scripts\python.exe .\lite2_sim\tools\run_monte_carlo.py --scene ruins_reference --course --profile sim2real_v1 --seeds 42-51 --jobs 3
```

不传 `--randomize` 时保持理想基线。每次随机化运行都会写出 `randomization.json`，并把 `randomization_profile` 写入 `config_snapshot.json`、把采样值写入 `navigation_summary.json` 的 `randomization` 块。`tools/run_monte_carlo.py` 并发执行多个种子，汇总 `report.json`/`report.md`：成功率、真值误差统计、逐段通过率、失败原因直方图、碰撞与恢复计数、每个种子的采样参数；`--min-pass-rate` 使成功率不达标时返回非零退出码。

### 自动建图（无固定目标）

`--explore` 用于检验“只靠传感器能建出多少地图”：不设置固定目标，沿可达前沿推进，无可达前沿且地图稳定后结束为 `map_complete`。`navigation_summary.json` 的 `exploration` 块给出已知空闲/占据/未知栅格数、访问前沿数、行驶距离与扫描帧数，并附带 `traversability_audit`：机器人真实经过的机身栅格中，被地图误标为占据或仍未知的比例，以及地图矩形完整度 `map_completeness_ratio`。该审计只用于评分，不参与导航决策。

探索模式当前**不能保证收敛**：本机 sensor_lab 复跑 751.9 s、行驶 80.75 m、访问 35 个前沿、整合 7519 帧后仍以 `stalled` 终止（未到 `map_complete`），但地图与真值一致性很好（`traversability_contradiction_ratio` = 1.4e-4，即 7071 个机身经过栅格中仅 1 个被误标占据）。终止原因与前沿耗尽判定和恢复用尽有关，见“课程链路当前状态”。

机器人模型采用本地厂商 Lite2 URDF。参考资源包括 [官方模型仓库](https://github.com/DeepRoboticsLab/deep_robotics_model)、[官方 Lite3 导航组件](https://github.com/DeepRoboticsLab/Lite3_Navigation) 和 [Lite2 控制示例](https://github.com/OpenCVChina/DeepRobotDog)。

### 课程链路当前状态（2026-11-13，方案 B）

用户确认赛题没有强制倒车要求，默认配置已删除 `dead_end_exit` 第 2 个检查点的 `reverse` 属性。退出位置检查点与 `branch_closed` 观测证据继续参与评测。正常入口 `run_mujoco.py --scene ruins_reference --course --headless --seed 42` 直接使用该配置。

种子 42、43、45 完整复跑均通过 7/7 段，0 碰撞、0 倒车指令、0 recovery 事件，仿真耗时分别为 333.5、328.75、347.75 秒。130 项回归全部通过。验收详见 [方案 B 课程验收](mujoco/course_acceptance.json)，视频见 [完整课程视频](../plan/videos/nav_full_course_headfirst_20261113.mp4)。该验收覆盖速度级仿真，实机步态验证待完成。

### 课程链路历史记录（2026-11-12 复跑）

连续 9 段 `--course` 链路**尚未跑通**，这是当前已知缺口，如实记录：

- 首段 `ramp` 最好的一次从 (-5.02,-4.24) 连续爬完 12° 坡到 (-5.02,2.80)，地面高度估计 0.296–0.30 m（坡度识别正确）、累计 7.38 m、522 帧扫描，随后在 y≈2.80 m 发生一次 `body_geometry_clearance` 碰撞（几何名 `ramp_inner_edge_collision_0`，机身距内侧墙 0.27–0.29 m），课程规则下单次碰撞即终态。
- 根因是几何与控制叠加的矛盾：坡道净宽 1.2 m，机身箱体 0.60×0.44 m（角点半径 0.372 m），而全局规划按 0.28 m 圆膨胀，A* 判为可通行；坡道内墙栅格长期停在 log-odds 0.35（只被地形高度跳变标记，既不进占据也不进膨胀），软净空代价覆盖不到它，局部控制因此把机身侧偏到 0.27–0.29 m。
- 规划半径取 0.37（≈角点半径）时 0 碰撞但 1.2 m 走廊可通行带只剩 0.456 m，多次复跑在 120 s 内出不了走廊；当前折中取 `robot_radius_m=0.34`。
- `ruins_reference` 单目标导航同样卡在坡道走廊：出走廊必须经过坡道，因此该场景导航未到点。
- 本轮已修复并有证据：坡道入口假障碍与起步 `insufficient_observations`（地面连通洪水填充 + 高度场坡度判据）、原地自旋死锁（前沿目标避开自车盲区、已访问前沿计权）、有界恢复（旋转 1.2 s → 倒车 ≤0.08 m/s 0.8 s → 稳定 0.5 s，上限 3 次）、近场占用 4 帧确认、探索模式 `map_complete` 与 `exploration_summary()`；105 项回归全绿。
- 未完成、留待下一轮：贴墙硬约束（距墙 <0.37 m 高代价或禁行）+ 真正的路径跟随（而非朝目标直线追）+ 原地转向前的角点扫描；估需 1.5–2 h 迭代。
- `--course-stage` 不能当作逐段能力证据：除 ramp 段与场景默认起点重合外，其余段起点分别位于 0.45 m 平台顶或管道口内，自车在起点即“未观测可通行”或与管壁接触（`start_blocked` / 首帧 `collision`），属段起点位姿问题，不是通过能力。段模式只跳过与标准起点绑定的单条西墙量程检查，并把焊缝沉降容差放宽到 20 mm（`verification.json` 的 `pose_dependent_checks=false` 与 `fixture_drift_m` 会记录下来），其余模型/传感器校验照常执行。

### 导航验收与视频

2026年11月12日，废墟场种子42、43、44均到点停车，终点真实误差分别为0.191、0.101、0.150 m，碰撞拦截均为0。同一版源码的标定场检查通过。37项回归及5类故障、障碍场景检查通过，完整导航输入回放3413帧，状态与速度指令逐帧一致。记录见 [导航验收汇总](mujoco/navigation_acceptance.json)。

可将已保存的运行轨迹渲染为带路径及机载相机视角的视频。

```powershell
uv pip install --python .\lite2_sim\.venv-mujoco\Scripts\python.exe -r .\lite2_sim\requirements-render.txt
.\lite2_sim\.venv-mujoco\Scripts\python.exe .\lite2_sim\tools\render_navigation.py <运行结果目录> --speed 8
```

视频是记录轨迹回放，四肢保持配置姿态。`video_manifest.json` 记录来源轨迹哈希、倍速和编码版本，视频保存为 `navigation_replay.mp4`。

### 标注尺寸场地更新

当前场地按2026年11月12日提供的标注图更新。坡道12°、高0.45 m、净宽1.2 m，S弯净宽1.2 m，中心线圆弧半径暂取1.1 m，死胡同净空1.2×2 m。圆管内径1 m、长1.9 m，方涵净宽1.2 m、净高0.6 m、通行长1.5 m，五组乱石凸起4–12 cm。起点1.5×1.5 m，终点2×2 m。隧道1的0.2 m暂按净宽解释，场地整体12×10 m仍来自图片估算。参数集中在config/ruins_reference.json，后续按官方数据调整。

新版模型与传感器检查通过，尺寸与假设记录在mujoco/reference/dimensions_review.json。导航验收已在标注尺寸场地上按当前源码重跑，结论见“导航验收与视频”和“课程链路当前状态”。

左上坡道墙体已沿坡面边缘调整，坡道相关墙顶距地面暂设1.05 m，在0.45 m高的平台上方保留0.60 m墙高。坡道内侧墙在下降段边缘接入S弯。增加两侧边缘射线检查。

北侧乱石堆移至S弯入口墙外侧，坡道、平台及下坡面保持无碎石。已检查所有碎石旋转后的几何包围盒与坡面范围无重叠。当前回归共105项，`tools/run_tests.py` 全绿。
