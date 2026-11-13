# Lite2 机器狗导航工作区

## 目录导航

| 目录 | 内容 |
| --- | --- |
| [lite2_sim](lite2_sim/README.md) | MuJoCo / Webots 仿真、导航代码、场景、模型、测试与运行记录 |
| [plan](plan/) | 总体方案、备赛计划、进度、流程图、文献综述与演示视频 |
| [research](research/) | 专题调研、论文分析与检索脚本 |
| [resources](resources/) | 官方赛题资料、厂商资料、原始资料压缩包与软件安装包 |
| [tmp](tmp/) | 调研工具、依赖、排查记录、过程数据和历史临时产物 |

## 常用材料

- [总体技术方案](plan/总体技术方案.md)
- [备赛执行计划](plan/备赛执行计划.md)
- [2026-11-13 进度](plan/进度_20261113.md)
- [导航策略流程图](plan/导航策略流程图.md)
- [顶刊方案思维导图](plan/四足机器狗导航_顶刊方案思维导图.html)
- [完整课程视频](plan/videos/nav_full_course_headfirst_20261113.mp4)
- [实机对接说明](lite2_sim/hardware/README.md)

## 获取工作区

此仓库使用 Git LFS 保存大文件。先安装 Git 和 Git LFS，再执行。

```powershell
git lfs install
git clone https://github.com/icemilk-1/lite2-sim.git
Set-Location lite2-sim
```

首次下载包含本地环境和完整历史运行数据，体积较大。使用 Python 3.12 在本机重建仿真环境。

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
- `tmp/` 中的部分历史脚本使用 `F:\airace` 绝对路径，重用时需按本机位置调整。

2026-11-13 整理目录，Git 仓库根目录设为整个工作区。
