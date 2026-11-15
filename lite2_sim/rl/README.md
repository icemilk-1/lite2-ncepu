# 局部导航 PPO 实验

局部策略使用 `VelocityCommand(vx, ω)`，通过 `NavigationSession` 的正常导航入口执行。训练环境复用 MuJoCo 速度执行器、雷达投影、地图、状态估计和全局路径。候选动作使用与现有局部规划器相同的机身包络和地形检查。

首轮绕障实验未达到晋级条件。下面的命令用于复现实验流程，模型效果与失败记录见执行报告。默认课程入口使用方案 B 的现有规划器，`--local-policy` 显式启用实验策略。

## 环境准备

在 `F:/airace/lite2_sim` 目录运行以下命令。CPU PyTorch 使用官方 CPU 软件源。

```powershell
uv venv --python 3.12 .venv-rl
uv pip install --python .venv-rl/Scripts/python.exe torch==2.9.1 --index-url https://download.pytorch.org/whl/cpu
uv pip install --python .venv-rl/Scripts/python.exe -r requirements-rl.txt
```

当前已验证环境的完整版本见 `rl/requirements-lock.txt`。重新安装锁定版本时需提供 PyTorch CPU 软件源。

## 检查与吞吐

```powershell
.venv-rl/Scripts/python.exe tools/navigation_rl.py check --task tracking --out runs/rl_check
.venv-rl/Scripts/python.exe tools/navigation_rl.py benchmark --task tracking --steps 512 --out runs/rl_benchmark
```

检查包括 Gymnasium / SB3 接口、重复 reset、独立模型状态与现有规划器单 episode。基准测量 1、2、4 个环境的采样速度，记录 reset、传感器、控制、执行和编码耗时。

## 训练与评测

```powershell
.venv-rl/Scripts/python.exe tools/navigation_rl.py train --task tracking --steps 8192 --jobs 2 --seed 42 --out runs/rl_tracking_42
.venv-rl/Scripts/python.exe tools/navigation_rl.py train --task obstacle --steps 16384 --jobs 2 --seed 42 --model runs/rl_tracking_42/policy.zip --out runs/rl_obstacle_42
.venv-rl/Scripts/python.exe tools/navigation_rl.py evaluate --task obstacle --jobs 4 --episodes 30 --seed 10000 --out runs/rl_eval_baseline
.venv-rl/Scripts/python.exe tools/navigation_rl.py evaluate --task obstacle --jobs 4 --episodes 30 --seed 10000 --model runs/rl_obstacle_42/policy.zip --out runs/rl_eval_policy
```

`tracking` 为平地短路径，`obstacle` 为单障碍绕行，`dynamic` 为正弦横移障碍，`obstacle_bypass` 使用废墟课程的对应赛段起终点。固定测试种子与训练、验证种子分别设置。CLI 默认测试从 10000 开始，训练时显式指定种子。

训练保存 checkpoint、最终模型、源文件哈希、依赖版本、动作映射及真值使用说明。观测采用固定归一化，模型推理复用同一编码器。策略异常或安全拒绝会产生停车命令和诊断信息。`policy_fallback` 默认关闭，启用后记录基线接管。

评估每完成一个回合便原子写入结果。中断后使用相同任务、模型和种子范围添加 `--resume`，从已保存的回合继续。此次运行出现内存压力后，最终评估采用 `--jobs 1` 顺序完成。首次创建的旧版中间报告缺少模型哈希，续跑时需结合训练清单核对模型来源。

奖励使用仿真真值计算同一条参考路径上的单步进度，并独立核对到达与碰撞。策略输入使用估计位姿、地图、在线路径及传感器信息。奖励分项通过环境 `info` 返回。

## 正常课程入口

```powershell
.venv-rl/Scripts/python.exe run_mujoco.py --scene ruins_reference --course --headless --seed 42 --local-policy runs/rl_obstacle_42/policy.zip --policy-stage obstacle_bypass
```

模型复制到运行目录并记录 SHA-256。`local_policy_decisions.jsonl` 记录策略原始动作、拦截原因及实际发出的命令，实际速度见 `slam_trajectory.csv`。可用 `.venv-rl` 运行 `tools/replay_navigation.py` 回放含策略的记录。

省略 `--policy-stage` 会在全部正常导航赛段启用该策略。选择启用范围前应完成对应任务的独立评测。

## 结果边界

本工程验证速度级导航，四肢保持配置姿态。实机传感器、控制话题、单位、运动模式、速度响应和停车看门狗需要现场核验。实验进度与最终数据见 `F:/airace/plan/导航RL执行记录_20261113.md`。
