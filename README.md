# Lite2 机器狗导航工作区

## 目录导航

| 目录 | 内容 |
| --- | --- |
| [lite2_sim](lite2_sim/README.md) | MuJoCo / Webots 仿真、导航代码、场景、模型、测试与运行记录 |
| [host_ui](host_ui/README.md) | 实机调试上位机（浏览器版）：导航地图、实际坐标、手动操控、急停、传感器、使能与姿态、会话录制 |
| [plan](plan/) | 总体方案、备赛计划、进度、流程图、文献综述与演示视频 |
| [research](research/) | 专题调研、论文分析与检索脚本 |
| [resources](resources/) | 官方赛题资料、厂商资料、原始资料压缩包与软件安装包 |
| [tmp](tmp/) | 调研工具、依赖、排查记录、过程数据和历史临时产物 |

## 代码与资料边界

```text
host_ui/                 实机上位机：app/ 后端、web/ 前端、tests/ 测试、maps/ 地图
lite2_sim/               仿真与导航：controllers/、hardware/、rl/、tools/、tests/
  assets/ worlds/ protos/ 仿真模型、场景与机器人描述
  config/                 仿真、随机化与机器人参数
  mujoco/                 MuJoCo 场景及验收材料
  runs/                   仿真运行输出（本地数据）
plan/                    方案、进度、实施记录、历史记录与演示材料
research/                专题调研与论文资料
resources/               厂商/赛事原始资料、压缩包及安装包
tmp/                     临时脚本、排障截图/日志和中间结果
```

新增的正式代码放入对应项目的 `app/`、`controllers/`、`hardware/` 或 `tools/`，测试放入同项目的 `tests/`；正式说明和可复用研究结论放入 `plan/` 或 `research/`。一次性探针、截图、临时配置和中间结果放入 `tmp/`，不要和源码、正式配置混放。需要长期保留的仿真输出放入 `lite2_sim/runs/`，大文件通过 Git LFS 管理。

`lite2_sim/lite2_rl_*` 是当前工作区里的模型生成目录，`lite2_sim/.venv*` 是本地 Python 环境；不要把它们当作源码目录。旧提交历史中仍可能含有环境文件和运行数据。当前版本已停止跟踪这些目录，历史体积需要另行处理。

## 常用材料

- [总体技术方案](plan/总体技术方案.md)
- [备赛执行计划](plan/备赛执行计划.md)
- [2026-11-13 进度](plan/进度_20261113.md)
- [导航 RL 改进计划](plan/导航RL改进计划_20261113.md)
- [导航策略流程图](plan/导航策略流程图.md)
- [顶刊方案思维导图](plan/四足机器狗导航_顶刊方案思维导图.html)
- [完整课程视频](plan/videos/nav_full_course_headfirst_20261113.mp4)
- [实机对接说明](lite2_sim/hardware/README.md)
- [厂商手册开发要点](plan/绝影Lite2厂商手册开发要点.md)
- [上位机使用说明](host_ui/README.md)
- [上位机架构审计与差距分析](host_ui/AUDIT_20261114.md)

## 获取工作区

此仓库使用 Git LFS 保存大文件。先安装 Git 和 Git LFS，再执行。

```powershell
git lfs install
git clone https://github.com/icemilk-1/lite2-sim.git
Set-Location lite2-sim
```

仓库保留协作所需的源码与资料。Python 环境和运行数据请在各自电脑上生成。使用 Python 3.12 在本机重建仿真环境。

```powershell
uv python install 3.12
uv venv --managed-python --python 3.12 --allow-existing lite2_sim/.venv-mujoco
uv pip install --python ./lite2_sim/.venv-mujoco/Scripts/python.exe -r ./lite2_sim/requirements-mujoco.txt
python ./lite2_sim/run_mujoco.py --scene sensor_lab --check
python ./lite2_sim/run_mujoco.py --scene ruins_reference --course --headless --seed 42
```

更多场景、回放、依赖与已知问题见[仿真说明](lite2_sim/README.md)。

## 资料位置

- `resources/官方资料/` 保存官方赛题文件。
- `resources/绝影机器人资料/` 保存展开后的厂商资料与 SDK。
- `resources/archives/` 保存原始资料压缩包。
- `resources/installers/` 保存 Webots 安装包。

## 过程记录

- `lite2_sim/runs/` 保存仿真运行结果，旧记录里的绝对路径对应生成时的本机位置。
- `plan/history/` 保存历史工作记录。
- `tmp/work/root_logs/` 保存原根目录的课程运行日志。
- `tmp/work/simulation_debug/` 保存历史导航调试脚本、随机化运行日志和进程记录。
- `tmp/pip/` 保存安装过程的临时目录。
- 安装残留统一归入 `tmp/pip/`。Python 缓存、进程日志与安装临时目录已加入 Git 忽略规则。
- `tmp/` 中的部分历史脚本使用 `F:\airace` 绝对路径，重用时需按本机位置调整。

2026-11-13 整理目录，Git 仓库根目录设为整个工作区。
