# Lite2 机器狗实机调试上位机（host_ui）

浏览器版实机调试上位机：**导航地图 + 实际坐标 + 手动操控 + 急停 + 传感器 + 使能/姿态 + 健康诊断 + 会话录制**。

面向绝影 Lite2（Jueying Lite2）机器狗实机部署与比赛调试；支持三种链路：

| 链路 | 说明 | 适用场景 |
|---|---|---|
| `mock` | 内置实验室地图 + 50Hz 运动学仿真 | 无机器人时开发/演示/回归 |
| `ssh_ros` | Windows 通过 SSH 连机载 ROS（192.168.1.103），远端跑 `ssh_ros_stream.py` | **实机首选**（状态、传感器、使能、姿态都走这条） |
| `ros` | 本机 ROS1 环境订阅/发布 | 机载电脑本机运行 |
| `udp` | 厂商 UDP 直连（状态需中继） | 演示/抓包验证；Windows 不能直接收到状态包 |

> 实机安全链由三层组成：上位机 SafetyCore（25Hz 输出 + 急停/看门狗/限幅）→ 传输层零速看门狗 → 机载 `execution_guard`（armed/mode_confirmed/深度心跳/basic/摇杆/倾角）。**任何一层失效都不应依赖另两层兜底，现场必须另配物理急停/遥控器。**

---

## 1. 快速开始（无需机器人）

```powershell
cd F:\airace
python -m pip install -r host_ui/requirements-host-ui.txt
python host_ui/run.py --link mock --no-open
```

浏览器打开 <http://127.0.0.1:8710/>。

默认离线演示支持：地图点选目标 → A* 规划 → 自动前往；D-pad/键盘（WASD/方向键）手动操控；急停/复位；使能/站立/趴下（模拟）；断线重连模拟；四个工作区（传感器 / 导航操控 / 设备设置 / 日志诊断）。

键盘：按住 `W/A/S/D` 或方向键点动，松开立即零速；`Space` 立即停车；`Esc` 急停锁存。

---

## 2. 实机部署（Windows → 机载 ROS）

### 2.1 网络与地址

| 设备 | 地址 | 说明 |
|---|---|---|
| 运动主机（厂商 jy_exe） | `192.168.1.120` | 速度/姿态/模式 UDP 指令 → `43893` |
| 感知主机（Jetson NX，ROS1 Melodic） | `192.168.1.103` | 状态回传 `43897`；SSH 目标 |
| Windows 上位机 | WiFi `192.168.1.101` / 有线 `192.168.1.102` | 需与机器人同网段 |

```powershell
python host_ui/run.py ^
  --link ssh_ros ^
  --config host_ui/config.json ^
  --no-open
```

`host_ui/config.json` 中链路示例（`source_ip` 可留空让系统自动选路；`known_hosts` 填机载主机指纹文件可启用固定校验，留空则不做主机密钥校验并在事件日志里持续提示）：

```json
{
  "link": {
    "kind": "ssh_ros",
    "options": {"host": "192.168.1.103", "username": "ysc",
                "source_ip": "", "known_hosts": ""},
    "reconnect": {"enabled": true, "min_delay_s": 1.0, "max_delay_s": 10.0}
  },
  "web": {"host": "127.0.0.1", "port": 8710, "token": ""}
}
```

> 现场提示：本机 WiFi/有线 IP 在 2026-11-14 曾从 `192.168.1.101` 变为 `.200/.100`，机器人 SSH 连接随之中断；若 `ssh` 报连接重置，先确认本机路由/源地址与机器人是否同网段，再检查机器人防火墙/白名单。

SSH 密码通过环境变量提供（不要写进仓库）：

```powershell
$env:HOST_UI_SSH_PASSWORD = "..."
```

### 2.2 机载前置（实机必做）

1. 控制模式连接会按需恢复状态桥、深度驱动和雷达驱动；雷达在待命心跳建立后独立检查，失败会记录诊断事件。已有点云发布者时复用，不重复起驱动。只读模式仅接收已有数据。定位与 SLAM 需按具体任务另行启动，项目建图入口见 `lite2_sim/hardware/navigation_mapping.launch`；话题见 §4。
2. 确认 `/airace/state`、`/airace/execution`、`/projected_map`、`/camera/*`、`/rslidar_points` 正常。
3. 上位机会在 SSH 流里按 ≥2Hz 发布 `/airace/depth_received`（`time.monotonic()`）；**这是机载执行门禁允许运动的前置条件**，深度断流会立即变成 `stale_depth` 并禁止运动。
4. 机载 `lite2_live.py` bridge 需要在运行（提供 `/airace/state`、`/airace/execution`、`/airace/prepare`、`/airace/disarm`）。

### 2.3 标准操作顺序（UI 上）

```text
上电（趴下 basic=1）
  → 站立（basic=6，力控站立）
  → 使能（/airace/prepare：移动模式 + 自动模式 ACK）
  → 手动/导航运动（≤0.05 m/s、≤0.1 rad/s 由机载硬限幅；上位机再加一层）
  → 立即停车（Space 或按钮，零速连续下发）
  → 解除（/airace/disarm，回到手动站立模式）
  → 趴下（basic=1）
```

急停：界面红色「急停」/`Esc` → 上位机 SafetyCore 锁存零速，同时请求机载代发厂商急停 `0x21010C0E`；**确认安全后长按 ≥1.2s 复位**。复位后必须重新按住方向键才会运动。

### 2.4 现场安全规范（必须）

- **物理急停/遥控器必须有人持有**；上位机只是辅助。
- 首次运动前：清空场地、有人监护、速度限幅从小值开始（默认手动 0.10 m/s）。
- 停车确认：命令归零 ≠ 实际停稳。按实速 + 端到端延迟 + 实测停车距离验收（历史试跑曾出现零速后仍漂移 0.052 m）。
- **进程兜底实测**：用 `taskkill /F` 杀掉上位机进程，记录机器人停稳时间/距离；不达标必须在机载侧增加超时停车或改用物理急停。
- 禁止无观测倒车；姿态切换期间禁止任何运动指令；姿态/急停必须在机器人可见范围内操作。
- 机器人时钟实测为 2018 年（相机 stamp≈1517160615），上线前请做 NTP 校时，否则录包时间轴不可用。

---

## 3. 安全与访问控制

- 默认只监听 `127.0.0.1`；绑定 `0.0.0.0`/局域网地址时**必须**配置令牌，否则服务拒绝启动。
- `--token` 或配置 `web.token`；访问 `http://<ip>:8710/?token=...`。WS 会校验 Origin 白名单与令牌，REST 需要 `X-Auth-Token`。
- `/api/link` 的链路参数走白名单：只允许各传输层默认参数表里的键，`host/robot_ip/source_ip` 必须是合法 IP/主机名，端口 0–65535，避免被任意网页/脚本改造成"向任意地址发 UDP"或"SSH 到任意主机"。
- **操控权独占**：仅保留一个浏览器连接，新连接替换旧连接并释放控制权。当前页面申请操控权后可操作，操作会写事件日志。
- 所有运动/使能/姿态/导航/限幅指令都要求操控权；当前连接申请操控权后才可发速度。急停仍可直接触发。

---

## 3.1 只读接收模式（机器狗只读）

在链路参数里设置 `read_only: true`（例如 `tmp/host_ui_live_config.json`）：

```json
{"link": {"kind": "ssh_ros", "options": {
  "host": "192.168.1.103", "username": "ysc", "source_ip": "", "read_only": true}}}
```

行为：

- 机载流以 `--read-only` 启动：**不发布** `/airace/cmd_vel`、`/airace/depth_received`，**不发送**任何厂商 UDP 指令（含姿态/急停），只做订阅与回传；
- 上位机 `capabilities.arm/posture/vendor_estop=false`，UI 顶栏显示「只读接收」、状态栏显示「只读 · basic 状态」，手动操控/使能/姿态/导航/急停/停车全部置灰并提示使用物理遥控器/急停；
- 就绪清单切换为监控语义（不要求站立、不发布深度心跳、自动模式不可用），避免把"未使能"误报成故障；
- 仍可正常显示：位姿/帧龄/电量/机载温度（`/temperature`）、相机/深度/雷达、事件日志、录制；
- 地图与执行状态取决于机载 `/projected_map`、`/airace/execution` 是否有发布者（建图/机载桥接未启动时 UI 会明确提示"未加载/只读模式不接收"）。

2026-11-14 实机只读验证（8710）：`ssh_ros` 9.2 Hz、相机/深度 424×240、雷达 1679 点、电量 70%、机载温度 60.3 °C、`basic=1`、无 console error；四工作区截图在 `tmp/real_ui_*.png`，报告 `tmp/real_ui_readonly_report.json`。机载 `/airace/execution` 与 `/projected_map` 当时无发布者，UI 已按只读语义显示为不可用。

## 3.2 机载控制会话与深度门禁（1.3.0）

**单一控制会话（机载桥接持有）**

- `lite2_live.py bridge` 负责 5 Hz 持续发送厂商心跳 `0x21040001`，并暴露：
  - `/airace/prepare`、`/airace/disarm`：自动/手动模式与使能；
  - `/airace/stand`、`/airace/lie`：站立/趴下（心跳 + **单次** `0x21010202` + `basic` 轮询确认）；
  - `/airace/estop`：厂商急停 `0x21010C0E` ×3。
- 控制会话在 prepare/stand/lie 时建立，**站立待命期间心跳不中断**；disarm、急停、遥控接管、桥接退出时停止。
- 上位机不再从 Windows 直发模式指令；SSH 流只调用上述服务，避免多终端争抢。
- 状态接口：`/api/status` 的 `link.control.control_session`：
  `{active, heartbeat_sent, heartbeat_age_s, started_at, paused_reason}`。
- 部署/更新：`python tmp/ssh_push.py <本地文件> <机载路径>` 会先备份为 `*.bak_20261114_standfix`，再重启机载桥接（Python 3.6）。

**深度心跳时钟（同源要求）**

- 机载流用 **`CLOCK_MONOTONIC`** 发布 `/airace/depth_received`：py3 用 `time.monotonic`；
  机载 Python 2.7 用 `ctypes` 调 `clock_gettime(CLOCK_MONOTONIC)`，与桥接（Python 3）同一内核时钟。
- 时钟不可用时**显式报错并拒绝启动**，禁止静默退化为墙钟（否则桥接 `fresh()` 永远 `stale_depth`）。
- 桥接诊断新增：`depth_age_s`、`depth_fresh`（实际门禁判定）、`control_session`。
- UI 健康卡分别显示：**深度门禁（桥接判定）**、**相机帧龄**、**机载控制会话**、**控制心跳帧龄**、**执行就绪**；
  就绪清单的"深度心跳"以桥接判定为准，不再用相机帧龄代替。

**UI 状态分层**

- 浏览器操控权（顶栏 pill）≠ 机载控制会话（健康卡）≠ 机器人姿态（模式 pill/basic）≠ 执行就绪（臂章 armed/mode）。
- `basic=8/9/11`（失控保护/自主爬起/翻倒）时模式 pill 变红显示保护原因，并禁用普通姿态/使能按钮。
- 站立/趴下超时：结束等待，显示实际 `basic` 与服务返回的失败原因（含"机载执行桥接未运行"提示）。

## 4. 机载接口（SSH 流）

| 话题/服务 | 类型 | 说明 |
|---|---|---|
| `/airace/state` | `std_msgs/String`(JSON) | 50Hz 厂商状态（上位机流控 10Hz）：basic/gait/motion/rpy/position/velocity/joystick/battery |
| `/airace/execution` | `std_msgs/String`(JSON) | 执行门禁诊断：reason/armed/mode_confirmed/depth_time/automatic_ack |
| `/airace/cmd_vel` | `geometry_msgs/Twist` | 上位机发布，机载 bridge 限时 0.25s |
| `/airace/depth_received` | `std_msgs/Float64` | 深度心跳（monotonic，≥2Hz） |
| `/airace/prepare`、`/airace/disarm` | `std_srvs/Trigger` | 使能/解除 |
| `/airace/stand`、`/airace/lie`、`/airace/estop` | `std_srvs/Trigger` | 机载单一控制会话执行姿态/急停 |
| `/projected_map` | `nav_msgs/OccupancyGrid` | 导航地图（octomap 投影） |
| `/camera/color/image_raw`、`/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 424×240 JPEG 预览（≤2Hz） |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 点云（≤2Hz，≤1800 点） |

厂商 UDP（V1.2.3）：速度 `320/321/325`（double）；模式指令 `0x21010C02/03`（手动/自动）、`0x21010D05/06`（站立/移动）、`0x21010202`（**起立/趴下轮换，禁连发**）、`0x21010C0E`（急停）、`0x21040001`（心跳 >1Hz）；196B 状态包偏移见 `tmp/lite2_interface_v123.txt`。

---

## 5. 配置说明

`host_ui/config.json`（由 `Config` 加载并自动收敛到安全范围）：

| 段 | 关键项 |
|---|---|
| `link` | `kind`、`options`（ssh host/user/source_ip、话题名等）、`reconnect` |
| `limits` | 硬限幅前进/后退/偏航/加速度、手动默认速度 |
| `safety` | 指令超时 0.35s、状态超时 0.8s、可运动 basic=6、摇杆接管阈值、低电阈值、导航操作者心跳超时、复位长按 |
| `posture` | 姿态确认超时、切换后禁运动余量 |
| `nav` | 最大速度/角速度、到达容差、膨胀半径、是否允许未知区域 |
| `map` | 启动地图文件、地图对齐 `align{x,y,yaw_deg}` |
| `web` | host/port/token/allowed_origins/max_clients |
| `record` | 是否自动录制、目录、采样率 |

限幅、地图对齐、配置保存会写回配置文件（若指定了 `--config` 或存在 `host_ui/config.json`）。

---

## 6. 四个工作区

1. **传感器**：前视相机、深度彩色图、雷达俯视点云（2Hz 预览）。
2. **导航操控**：导航地图（栅格/轨迹/路径/目标/雷达点）、实际坐标、手动操控、使能/就绪面板、姿态、健康。
3. **设备设置**：链路连接、速度限幅、地图对齐、会话录制、访问令牌。
4. **日志诊断**：事件时间线（接管/急停/姿态/使能/断线重连）、速度遥测曲线。

会话录制：`host_ui/sessions/<时间戳>/`，`telemetry.jsonl`（10Hz 位姿/指令/安全/电量/姿态）+ `events.jsonl`；页面可下载 zip。

---

## 7. 测试与验收工具

```powershell
python -m pytest host_ui/tests -q
```

覆盖：厂商协议编解码（含关节温度/角度复杂包）、UDP 回环、SSH 流解析/使能命令、地图管线、ROS 适配、SafetyCore 状态矩阵、HostNode 使能/姿态/急停、操控权、鉴权、链路参数白名单。当前基线：**152 passed**（以实际输出为准）。

实机验收一条命令（默认只读，不发运动指令）：

```powershell
# 只读体检：链路/帧率/深度心跳/电量/basic/急停/版本
python host_ui/tools/real_robot_acceptance.py --out tmp/host_ui_real_acceptance.json

# 运动验收（必须现场有人监护、持物理急停；默认 0.05 m/s）
python host_ui/tools/real_robot_acceptance.py --allow-motion --confirm "现场有人监护" --speed 0.05
```

工具会依次检查：链路与帧增长 → 深度心跳 ≤0.5 s → basic 可读 → 站/趴 → 使能（armed+mode_confirmed）→ 低速位移 → 停车零速/停车距离 → 软件急停锁存 → 长按复位 → 解除；失败项会单独列出，记录 JSON 可直接归档进交付材料。

UI 无头验收脚本参考 `tmp/ui_audit.py`、`tmp/ui_probe.py`、`tmp/integration_check.py`（Chrome CDP）。

---

## 8. 故障排查

| 现象 | 排查 |
|---|---|
| 界面显示"未连接/断线横幅" | 检查网线/WiFi、机器人是否上电、`ping 192.168.1.103`、SSH 口令环境变量；上位机自动重连并显示次数 |
| 姿态/使能一直失败 | 看 `execution.reason`：`stale_depth→深度驱动/心跳`、`mode_unconfirmed→自动模式 ACK 未收到`、`remote_takeover→遥控器摇杆未回中`、`motion_mode_not_ready→basic≠6` |
| 相机/深度黑屏 | 机载 realsense 驱动是否启动、话题名是否匹配、SSH 带宽；UI 会显示数据超时 |
| 雷达点云无信号 | 检查 `/rslidar_points` 是否有发布者及新鲜非空数据；控制连接会独立恢复缺失的雷达驱动，自动恢复日志在 `/tmp/airace_lidar.log`。有发布者但无数据时排查雷达供电、网线及 UDP 配置，避免重复启动驱动 |
| 地图不显示/很小 | 用「适配视图」；地图加载在隐藏工作区时会自动重新 fit；检查 `/projected_map` 是否发布 |
| 无法操作（按钮灰） | 未取得操控权（点顶栏申请）；观察者只能急停/停车 |
| 速度发不出去 | 是否使能、basic=6、摇杆回中、倾角<18°、急停是否锁存、限幅是否为 0 |
| 急停复位无效 | 必须长按 ≥1.2s；后端也会校验 `hold_s` |
| 关节温度显示"未接入" | 正常：厂商仅在关节过温时上报 `0x11050400`，平时无数据；UI 会展示机载上报的原始温度 |
| `/api/link` 报"链路参数被拒绝" | 只允许各链路默认参数表里的键；`host/robot_ip/source_ip` 必须是合法 IP/主机名，端口 0–65535 |

---

## 9. 已知限制

- 厂商 `error_state@172` 在 Lite2 文档标注 *Lite 无效*、实测恒 0，UI 只记录不告警；关节温度仅过温时上报，暂未接入。
- Windows 不能直收厂商状态包（固定发往 `.103:43897`）；`udp` 链路需在 `.103` 上做中继。实机请用 `ssh_ros`。
- `0x2*******` 模式指令只认单一终端（机载白名单），因此姿态/急停必须由机载进程代发，Windows 直发可能被丢弃。
- 实时性：地图在加载时预编码（base64+JSON 缓存），广播时不再对整图做编码；单客户端发送超时 1 s，发送失败后关闭连接并释放控制权；相机/深度/雷达只在「传感器」工作区且页面可见时轮询。
- 安全与访问控制：默认只监听 `127.0.0.1`；绑定非回环必须配 token；WS 校验 Origin 与 token；`/api/link` 参数走白名单（拒绝任意 IP/端口注入）；SSH 可配 `known_hosts` 固定主机密钥。
- 控制会话/深度门禁：见 §3.2。机载桥接必须运行（`lite2_live.py bridge`），姿态/使能才可用；深度心跳必须与桥接同源单调时钟。
- 当前实机验证进度：只读 UI 已跑通；1.3.0 的站立/趴下/使能/解除/停车修复**尚未在实机完成验收**（2026-11-14 17:20 机器人网络再次不可达），恢复后按 `REAL_ROBOT_CHECKLIST.md` 执行。
- 设备健康：电量%（低电告警）、basic/gait/motion 中文、关节过温（`0x11050400` 上报时）、机载主机名/ROS 版本、帧龄/重连次数。
- 版本：当前 1.2.0（2026-11-14 批次 1 完成）。
- 姿态/深度心跳/使能为 2026-11-14 新增，尚未在实机逐项复验（机器人离线），验收清单见 `host_ui/REAL_ROBOT_CHECKLIST.md` 与 `host_ui/AUDIT_20261114.md`。
- ROS1 本地链路（`ros`）单元测试用假 rospy，未在真实 ROS 环境验证。
- 相机/雷达存在性与型号仍需现场确认（方案文档记"未配雷达"，但 11-14 实测收到 `/rslidar_points`）。

---

## 10. 目录

```
host_ui/
  run.py                     # 启动入口
  config.json                # 本地配置（可选，含令牌/链路）
  requirements-host-ui.txt
  start.bat
  app/                       # FastAPI 后端
    server.py node.py state.py nav.py config.py recorder.py mapstore.py protocol.py types.py
    transports/              # mock / vendor_udp / ros1 / ssh_ros / ssh_ros_stream
  web/                       # 浏览器 UI（无第三方前端依赖）
  maps/                      # 地图文件（ROS yaml/pgm/json 与验证记录）
  sessions/                  # 会话录制
  tests/                     # pytest
  AUDIT_20261114.md          # 架构审计与差距分析
  IMPLEMENTATION_CONTRACT_20261114.md
```

---

_版本 1.3.0 · 2026-11-14_

## 单客户端通信

浏览器连接固定使用客户端编号 1。刷新或打开新页面会关闭旧 WebSocket，并停车、释放旧操控权，新页面需重新申请操控权。旧页面收到关闭码 4001 后停止自动重连。服务端仅向当前连接广播数据，旧连接的命令与延迟清理均通过 socket 身份校验。机器人链路的连接和断开通过同一把锁串行执行。


### 通信故障恢复

浏览器建立连接超过 10 秒、连续 5 秒收不到状态，或发送队列超过 64 KiB 时，会清空本地操作输入并重建连接。新连接需要重新申请操控权。REST 写请求通过 hello 返回的会话令牌绑定当前页面，页面被替换后旧令牌失效。停车和触发急停仍可从失联页面提交。

服务端为状态、初始化消息和回执统一串行发送。单批发送超时为 1 秒，发送失败立即撤销会话、停车并释放操控权。等待机载确认的使能、解除、建图和急停操作在后台执行，心跳与停车仍可处理。每个连接同时最多等待一个普通控制请求和一个急停请求。

SSH 停止时关闭正在建立的连接和已建立的通道，保留尚未退出的线程引用，旧线程退出前禁止重启同一个传输对象。断线后清空控制回执和旧状态，新的急停请求重新等待回执。UDP 接收端口独占绑定。上位机按连接实例校验数据回调，已停止链路的迟到数据会被丢弃。

离线回归可在工作区根目录运行 `python -m pytest host_ui/tests -q`。通信专项测试位于 `tests/test_communication_stability.py` 和 `tests/test_communication_frontend.py`，覆盖假 SSH 通道、浏览器连接看门狗和本机 UDP 端口占用。
