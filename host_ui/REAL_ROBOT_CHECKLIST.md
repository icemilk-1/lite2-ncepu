# 实机验收清单（Lite2 · host_ui 1.3.0）

## 最新实机复验（2026-11-14 19:39）

电脑上位机入口已通过站立、持续控制心跳、使能、零速停车、解除、趴下验收。
`tmp/real_control_verify.json` 的 `ok=true`，最终 `basic=1`、未使能。
会话记录位于 `host_ui/sessions/session_20261114_193948_7efc2e/`。
此次验收覆盖姿态与零速控制，行走及运动中的急停仍需后续验证。

本次定位与处理如下。

- 深度话题有订阅者但无发布者。启动机载已有 RealSense 驱动后，
  桥接 `depth_fresh=true`，复验起立时深度帧龄为 0.018 s。
- 当前运动固件按控制终端源 IP 的登记顺序接收 `0x2...` 指令。
  失败时首位为旧地址 `.100`，机载 `.103` 排第二位。源端口改为 `43893`
  及使用厂商 MQTT 转发均未恢复起立。
- 经用户确认、机器人趴下且现场有人监管时，执行厂商提供的
  `sudo systemctl restart jy_exe.service`，清除旧终端记录；随后由 `.103`
  建立会话。实机状态经过 `1 → 17 → 4 → 5 → 6 → 7 → 1`。
- 姿态服务每次请求仅发送一次 toggle；预热计入总超时，发送前复核状态新鲜度
  与会话状态；主机额外保留 2 s 接收机载回执。
- 现场用户确认机器人没有实体按键，当前未连接遥控器。

相机恢复命令使用现场识别到的 D435i 序列号。重新上电后可先检查话题发布者，
缺失时在机载终端执行。

```bash
source /opt/ros/melodic/setup.bash
source /home/ysc/realsense-driver-jetson/devel/setup.bash
roslaunch realsense2_camera rs_camera_424_240.launch serial_no:=215322072016
```

## 以下为此前排查状态及通用验收项目

> **历史状态（2026-11-14 17:20）：机器人网络再次不可达**
> 1.3.0 已完成：机载桥接统一控制心跳、`/airace/stand|lie|estop` 服务、深度门禁同源时钟（py2.7 ctypes CLOCK_MONOTONIC）、诊断字段与 UI 分层。
> 待网络恢复后执行：`tmp/ssh_push.py` 同步桥接（自动备份）→ 重启桥接 → 重启上位机 → `tmp/real_control_verify.py`（只验站立/趴下/使能/解除/停车，不行走）。
> 任何一步无响应/心跳异常/保护状态：立即停止，保存 `/tmp/airace_bridge.log`、`/api/status`、事件日志后再询问用户。

> 机器人离线时以下为**回到现场后的逐项验收步骤**。每步记录输出，完成后把结果存到 `tmp/host_ui_real_*.json`。
> 前置：物理急停/遥控器在监护人手中；场地清空；车速限幅从 0.05 m/s 起。
> 本机网络已变为 WLAN `10.110.118.170`（另有 FlClash 代理），与机器人不在同一网段；SSH/验收无法执行。
> 恢复步骤：① 把 Windows 接回机器人网段（有线/WiFi），确认 `ping 192.168.1.103` 通；
> ② 设置 `$env:HOST_UI_SSH_PASSWORD`（不写入仓库）、`link.options.source_ip` 留空自动选路；
> ③ 重启 8710 并先跑只读体检，再按本清单做有人监护的运动验收。
> 在此之前，代码/测试/UI/验收工具均已就绪（155 项测试全绿，v1.2.0）。

> 机器人离线时以下为**回到现场后的逐项验收步骤**。每步记录输出，完成后把结果存到 `tmp/host_ui_real_*.json`。
> 前置：物理急停/遥控器在监护人手中；场地清空；车速限幅从 0.05 m/s 起。

## 0. 准备

```powershell
cd F:\airace
$env:HOST_UI_SSH_PASSWORD = "<机载 ysc 用户密码>"   # 不写入仓库
python host_ui/run.py --link ssh_ros --config host_ui/config.json --no-open
```

先跑只读体检（不会发运动指令）：

```powershell
python host_ui/tools/real_robot_acceptance.py --out tmp/host_ui_real_acceptance_readonly.json
```

1.3.0 起再跑控制专项验收（站立/趴下/使能/解除/停车，不行走）：

```powershell
python tmp/real_control_verify.py
```

它检查：控制会话建立、站立后 3 s 内心跳 `heartbeat_sent` 持续增加且 `heartbeat_age_s<0.5`、
`depth_fresh=true`、使能 `armed/mode_confirmed`、停车零速、解除、趴下；失败会保存
`tmp/real_control_verify.json` 与最后状态。

体检通过后再做下面的运动验收（有人监护、物理急停在手）：

```powershell
python host_ui/tools/real_robot_acceptance.py --allow-motion --confirm "现场有人监护" --speed 0.05
```

- 机载桥接进程在跑（提供 `/airace/state`、`/airace/execution`、`/airace/prepare`、`/airace/disarm`）。
- 相机/深度/雷达/定位驱动已启动；NTP 校时（当前机器人时钟约 2018 年）。
- 网络：Windows 与机器人同网段；`ping 192.168.1.103` 通；SSH 可登录。
- 主机密钥：建议把机载主机指纹写入 `known_hosts` 并在配置里填 `link.options.known_hosts`，避免中间人风险；留空时事件日志会持续提示。

## 1. 链路与深度心跳（最关键）

| 检查 | 期望 | 记录 |
|---|---|---|
| 页面链路状态 | `机载 ROS · SSH · 已连接`，帧龄 < 100 ms | |
| `health.bridge_depth_age_s` / `depth_fresh` | `< 0.5 s` 且 `depth_fresh=true`（桥接实际判定） | |
| `health.camera_depth_age_s` | 相机帧龄单独显示，只作参考 | |
| `link.control.control_session` | 站立/使能后 `active=true`，站立待命 3 s 内 `heartbeat_sent` 持续增加 | |
| `/airace/execution` 的 `reason` | 不出现 `stale_depth`；姿态/使能请求后有明确回执 | |
| 断网测试 | 拔网/close SSH → 横幅"自动重连中（第 N 次）"；恢复后自动重连 | |
| 时钟 | 流用 `CLOCK_MONOTONIC`（py2.7 走 ctypes）；若启动报"无法获得单调时钟"必须查机载环境，**禁止**退回墙钟 | |
| 关节温度 | 正常显示"未接入（仅过温上报）"；过温时 `0x11050400` 上报后 UI 显示关节最高温（≥70 °C 预警，阈值待标定） | |
| 机载信息 | 健康卡显示主机名/ROS 发行版（来自机载流 `system` 消息） | |

若 `depth_age_s` 一直 `null` 或 `stale_depth`：先查 `ssh_ros_stream.py` 是否收到深度帧、`/airace/depth_received` 是否发布、guard 与流是否同一时钟源。

## 2. 站立 / 趴下

1. 初始 `basic=1`（趴下）。
2. 点击「站立」→ 确认 basic 变为 **6**，再点「趴下」→ 回到 **1**。
3. 观察：姿态请求期间速度输出为 0、按钮置灰；取消按钮可中止；失败原因中文显示（含实际 basic）。
4. **控制会话检查**：站立后健康卡「机载控制会话」应为"运行中（心跳 N 次）"，「控制心跳帧龄」<0.3 s；
   站立待命 3 s 内心跳计数必须持续增加——这是 1.3.0 的关键修复点。
5. **注意**：`0x21010202` 是轮换指令，实现只发一次；若在遥控器/App 上同时操作姿态会互相干扰（厂商只认单一终端）。

## 3. 使能 / 解除

1. basic=6 时点击「使能」→ `health.armed=true`、`mode_confirmed=true`，就绪清单 7 项全绿（深度心跳必须绿）。
2. 点击「解除」→ `armed=false`，恢复手动模式。
3. 失败时看 `execution.reason`：
   - `mode_unconfirmed`：自动模式 ACK 未收到（机载 `prepare` 里等 `0x21010C03` 回包）。
   - `stale_depth`：见 §1。
   - `motion_mode_not_ready`：未站立。
   - `remote_takeover`：遥控器摇杆未回中。

## 4. 手动运动（低速）

1. 保持使能，手动线速度上限 0.05–0.10 m/s。
2. 按住 `W` 前进 2 s → 松手；页面下发速度归零、实际速度归零。
3. 观察 `execution.reason` 是否 `enabled`；若 `stale_command_feedback`/`command_timeout` 检查 25Hz 指令与反馈。
4. **停车确认**（历史曾出现零速后漂移 0.052 m）：用录制 JSONL 或外部视频，测量命令归零到静止的距离/时间；按 0.015 m 跨度判据验收，不达标需降低限速或改控制。

## 5. 急停（软件 + 厂商）

1. 运动中以 0.10 m/s 前进，点击「急停」：
   - 页面 1.5 s 内显示 `急停锁存` + 红色横幅；
   - 机器人应停止；机载代发 `0x21010C0E`（SSH 流 `control` 回执可见）。
2. **确认物理急停/遥控器可用**（软件全挂时的唯一兜底）。
3. 长按「复位急停」≥1.2 s（后端校验 `hold_s`），然后必须重新按住方向键才会运动。
4. 若页面提示"急停指令未确认，请立即使用物理急停/遥控器"→ 立即物理急停并抓包排查。

## 6. 进程兜底（必做）

1. 以 0.05 m/s 前进时，`taskkill /F /PID <上位机进程>`。
2. 记录：机器人是否在 0.2–0.5 s 内停稳；停车距离。
3. 若不达标：在机载侧增加超时停车（bridge 已有 0.25 s 指令超时 + 0.3 s 零速看门狗，需验证），或明确要求现场始终使用物理急停。

## 7. 录制与回放

1. 点击「开始录制」，跑一段手动 + 导航，停止录制，下载 zip。
2. 检查 `telemetry.jsonl` 含 pose/velocity/command/battery/armed/posture；`events.jsonl` 含接管/急停/姿态/使能/断线事件。

## 8. 事件证据（比赛要求）

确认以下事件都出现在「日志诊断」并带时间戳：
- 操控权申请/接管/释放、急停触发/复位、使能/解除、姿态站/趴、链路断开/重连、操作者心跳超时（导航取消）。

## 9. 待现场确认的未知点

- [ ] `0x21040001` 心跳由谁维持（bridge_server / 上层 / 都不发）——抓包 `tmp/robot_control_sources.py` 思路。
- [ ] 厂商终端 IP 白名单：机载 .103 代发姿态/急停是否稳定；App/遥控器并发时行为。
- [ ] 雷达是否实际安装、型号、`/rslidar_points` 频率；`/projected_map` 尺寸/更新率。
- [ ] 相机/深度 launch 参数与真实分辨率帧率；深度断流恢复策略。
- [ ] `battery_level` 实机 0–100 的满电/低电阈值标定（当前暂用 20% 告警）。
- [ ] `error_state` 确认恒 0（Lite 无效）；关节温度 0x11050400 过温上报是否可解析。
- [ ] 进程 kill 停车时间、停车距离。

## 10. 验收记录模板

```json
{
  "date": "2026-11-__",
  "link": {"kind": "ssh_ros", "frames": 0, "age_s": 0.0},
  "depth_age_s": 0.0,
  "stand": {"ok": true, "basic": 6},
  "arm": {"ok": true, "armed": true, "mode_confirmed": true},
  "move": {"speed_m_s": 0.05, "duration_s": 2.0},
  "stop_distance_m": 0.0,
  "estop": {"software": true, "vendor_ack": true, "stop_time_s": 0.0},
  "kill_test": {"stop_time_s": 0.0, "stop_distance_m": 0.0},
  "notes": ""
}
```
