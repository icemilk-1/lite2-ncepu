# host_ui 改造实施契约（2026-11-14 · 批次 1）

> **2026-11-14 修订（read hardware-reader 后，优先级最高）**
> 1. `/airace/depth_received` 必须用**机载 `time.monotonic()`** 发布（`execution_guard.py:5-7,21` 用 monotonic 比较），≥2Hz，深度断流停发；**不能用 wall clock / `rospy.Time.now()`**。
> 2. 姿态/急停/模式指令必须**由机载 .103 进程代发**（厂商 `0x2*******` 只认单一终端、有 IP 白名单），Window 直发会丢包。`ssh_ros_stream.py` 负责执行，`ssh_ros.py` 只发命令等回执。
> 3. `battery_level@180` 实机是 0–100（文档写 0.00–1.00），按 ≤1.5 才 ×100 兼容；`error_state@172` 文档标注 *Lite 无效* 且恒 0，**不做故障门禁/红色告警**，只展示；`is_charging` 同样无效。
> 4. 姿态切换失败的真实机制是 Windows 直发被厂商丢弃 + 主机锁竞争；新实现必须走机载，单次 toggle + basic 轮询确认。

本文件是 Lead 与 `link-dev` / `frontend-dev` 的共同接口约定。**以本文件为准**；需要变更先找 Lead。

---

## 0. 文件所有权（避免并发冲突）

| 角色 | 只允许改这些文件 |
|---|---|
| Lead | `host_ui/app/{types,state,node,server,config,recorder,mapstore}.py`、`host_ui/run.py`、`host_ui/tests/test_{safety,operator,auth,node}.py`、`host_ui/*.md` |
| link-dev | `host_ui/app/protocol.py`、`host_ui/app/transports/{vendor_udp,ros1,ssh_ros,ssh_ros_stream,mock}.py`、`host_ui/tests/test_{vendor_protocol,vendor_udp,ssh_ros,ros1_adapter,mock_transport}.py` |
| frontend-dev | `host_ui/web/{index.html,app.js,workspace.js,style.css,workspace.css}` |

`host_ui/app/transports/base.py` 已由 Lead 更新，**link-dev 不要改**（有新增默认方法）。
`host_ui/app/config.py` 已由 Lead 更新，**README 中说明的新配置项以 config.py 为准**。

---

## 1. 传输层新增接口（base.py 已提供默认实现）

```python
capabilities = {"arm": False, "posture": False, "vendor_estop": False,
                "sensors": False, "reconnect": False}
def arm(self, timeout_s=8.0) -> tuple[bool, str]
def disarm(self, timeout_s=8.0) -> tuple[bool, str]
def set_posture(self, action: str, cancel=None, timeout_s=10.0) -> tuple[bool, str]
def emergency_stop(self) -> tuple[bool, str]
```

`status()` 必须额外包含（可为 None/0）：

```jsonc
{
  "control": {
    "armed": false,
    "mode_confirmed": false,
    "pending": null,            // null | "prepare" | "disarm" | "posture"
    "last_error": "",
    "depth_age_s": null,        // /airace/depth_received 心跳年龄
    "automatic_ack_age_s": null
  },
  "reconnect": {"enabled": true, "attempts": 0, "next_retry_in_s": null},
  "execution": { ...原样透传 /airace/execution... }
}
```

`Frame.raw` 约定新增（有就填，没有就不填）：

- `battery_percent`：0–100 的百分数（厂商包是 0.00–1.00 的 double，需 ×100；实测固件会直接给 59/67/87，>1.5 视为已经是百分数）
- `battery_raw`：原始值
- `error_state`：整型（厂商文档注明 *Lite 无效，只展示不门禁）
- `is_charging`、`zero_position_flag`、`motion_state`

---

## 2. 厂商协议常量（来自《绝影Lite2 软件接口文档 V1.2.3》）

简单指令：`struct.pack("<3i", code, value, 0)`，12 字节；复杂指令 `type=1`，状态包 196B。

| 名称 | code | 说明 |
|---|---|---|
| 心跳 | `0x21040001` | 发送频率 >1Hz |
| **急停** | `0x21010C0E` | 厂商硬件急停 |
| 起立/趴下 | `0x21010202` | **轮换**：发一次切换一次，禁止连发 |
| 移动模式 | `0x21010D06` | |
| 站立模式 | `0x21010D05` | |
| 手动模式 | `0x21010C02` | |
| 自动模式 | `0x21010C03` | |
| 持续运动 | `0x21010C06` | value -1 启动 / 2 取消 |
| 回零 | `0x21010C05` | |
| 保存数据 | `0x21010C01` | |
| 前后/左右/转向/上下 | `0x21010130/31/35/02` | 摇杆轴 ±32768，死区见文档 |
| 导航速度 | 320 / 321 / 325 | double，前进 / 旋转 / 平移（现有实现） |

196B 状态包绝对偏移（12B 头 + RobotStateUpload）：

```
12 basic int | 16 gait int | 20 rpy[3] | 32 rpy_vel[3] | 56 xyz_acc[3]
80 pos_world[3] | 104 vel_world[3] | 128 vel_body[3]
164 touch uint32 | 168 is_charging bool | 172 error_state uint32
176 motion int | 180 battery_level double(0~1) | 188 task_state int
192 is_robot_need_move bool | 193 zero_position_flag bool
```

basic：1 趴 / 4 准备起立 / 5 起立中 / 6 力控站立(可运动) / 7 坐下 / 8 失控保护 / 9 自主爬起 / 11 翻倒 / 17 回零 / 18 后空翻 / 19 打招呼
gait：0 行走 / 1 跑跳 / 2 楼梯 / 3 遛步 / 4 跑步 / 6 斜坡
motion：0 基本运动 / 1 步态 / 2 跳舞

---

## 3. link-dev 交付

### 3.1 `protocol.py`
- `decode_state` 补齐上述字段，并保证旧字段兼容；battery 值做 >1.5 的百分数归一；`error_state` 透传。
- 新增：`HEARTBEAT`/`ESTOP`/`POSTURE_TOGGLE`/`MOVE_MODE`/`STAND_MODE`/`MANUAL_MODE`/`AUTO_MODE`/`CONTINUOUS_MOTION`/`ZERO_POSITION`/`SAVE_DATA`/`AXIS_*` 常量与 `simple_packet(code, value=0)`。
- 新增 `decode_any(packet)` 可选：区分 196 状态 / 60 遥控反馈 / 12 简单指令 ACK（返回 `{"code","value","type"}`）。
- `velocity_packets` 保持不变（逐字节兼容）。

### 3.2 `ssh_ros_stream.py`（机载 Python 2.7）
- 订阅 `/camera/aligned_depth_to_color/image_raw`（或当前深度话题）后，在收到新鲜帧时以 ≥2Hz 发布 `/airace/depth_received`（`std_msgs/Float64`，值 = **`time.monotonic()`**，不是墙钟）；深度断流时停止发布。
- stdin JSON 命令通道（一行一条），至少支持：
  - `{"action":"prepare"}` → `rospy.ServiceProxy("/airace/prepare", Trigger)()`，结果 `emit("control", {"action":"prepare","success":...,"message":...})`
  - `{"action":"disarm"}` → `/airace/disarm`
  - `{"action":"posture","target":"stand"|"lie"}` → **机载进程**发心跳 `0x21040001`（≥5Hz）+ **单次** `0x21010202`（轮换指令，禁连发），轮询 `/airace/state` basic 到目标（stand→6，lie→1）或超时，结果 `emit("control", ...)`
  - `{"action":"estop"}` → 机载进程发 `0x21010C0E` 三次，`emit("control", ...)`
  - `{"action":"ping"}` → `emit("heartbeat", {"stamp": time.time()})`
- 服务/姿态调用放子线程执行（prepare 最多 3 s、姿态最多 10 s），主循环继续收 stdin/publish 零速。
- 不要破坏现有 state/execution/map/sensor 输出与 Python 2.7 兼容性。

### 3.3 `ssh_ros.py`
- `capabilities = {"arm": True, "posture": True, "vendor_estop": True, "sensors": True, "reconnect": True}`
- `arm()/disarm()`：通过 stdin 发命令，等待 `control` 消息或 execution 的 armed 变化，超时返回中文原因；期间 `send_velocity` 强制 0（posture/control pending 锁）。
- **断线自动重连**：SSH/通道异常后按 `link.reconnect` 配置指数退避重连；重连期间 `send_velocity` 不报错、`status().connected=false`、`reconnect.attempts` 递增；重连成功发事件 `ssh_ros_reconnected`；持续失败发限频警告。
- **健壮解析**：单行 JSON 失败不拆链路（计数+限频事件）；`pending` 缓冲上限 8MB；消息缺 `type/data` 时忽略；`recv_stderr` 限频。
- `set_posture(action, cancel, timeout_s)`：通过 stdin 发 `{"action":"posture","target":...}`，等 `control` 回执（机载 .103 代发，单次 toggle + basic 确认）；`cancel` 置位立刻补零并返回取消；**禁止在同一动作里重复发 toggle**。
- `emergency_stop()`：通过 stdin 发 `{"action":"estop"}` 由机载代发 `0x21010C0E`；等回执，失败也返回原因（上层仍会零速+锁存急停）。
- `status()` 按契约第 1 节输出，`connected` 仍需在 stale 时按现有语义处理。
- 保留现有 `sensor_snapshot()` 供 `/api/sensors`。

### 3.4 `vendor_udp.py`
- `capabilities = {"posture": True, "vendor_estop": True, "reconnect": False}`
- 解码 battery/error 等字段进 `Frame.raw`（battery 按 ≤1.5 才 ×100；error_state 只记录）。
- `emergency_stop()`：尽力发 `0x21010C0E`（Windows 直发可能被厂商终端白名单丢弃，失败要返回原因）；`set_posture` 同理，只作演示/中继链路尽力而为。
- 修复遥控反馈过期：60B 反馈超过 0.5s 未更新时把 joystick 清零并限频告警。
- `status().control.depth_age_s = None`、`armed` 仅当收到状态包且 basic==6 时为 True（用于前端展示"机载执行层未连接"）。
- 监听端口保持 43898 默认，但 `default_options` 注释说明厂商实际上报目标是 `192.168.1.103:43897`，Windows 直连需要中继；文档里写清。

### 3.5 `ros1.py`
- `capabilities = {"arm": True}`；`arm()/disarm()` 用 `rospy.ServiceProxy("/airace/prepare"|"/airace/disarm", Trigger)`（懒加载），失败返回中文。
- `frame_from_state_payload` 把 `battery_level_raw` 归一为 `battery_percent`、透传 `error_state`/`is_charging` 到 `Frame.raw`。
- `status().connected` 目前恒 True 的问题：未 `start()` 或已 `stop()` 时返回 False。

### 3.6 `mock.py`
- `capabilities = {"arm": True, "posture": True, "vendor_estop": True}`，用于前端离线联调：
  - `arm()/disarm()`：模拟 0.8s 延迟后切换 armed，并更新 `status().control`。
  - `set_posture(action)`：basic 1↔6 切换，0.6s 后生效；`cancel` 可中止。
  - `emergency_stop()`：直接进入 estop 状态（basic 保持）。
- 保留现有 `simulate(joystick, link)`。

### 3.7 测试（必须通过）
- `test_vendor_protocol.py`：用 `struct.pack` 造 196B 包验证 battery(0.67/67)→67%、error_state、offset 对齐；`simple_packet(0x21010C0E)` 12B 字节序；`velocity_packets` 不变。
- `test_ssh_ros.py`：用假 channel 验证 `arm` 发出的 JSON 行、`control` 消息回填 status、坏行不拆链、pending 超限、reconnect 次数递增（可用短退避）。
- `test_mock_transport.py`：arm/posture/estop 状态迁移。

---

## 4. frontend-dev 交付

### 4.1 新增 UI（保持现有四工作区结构）
1. **操作权**：顶栏加 `#control-pill`（"操控权：我 / 他人 / 无"）。点击弹层：申请/接管/释放。观察者模式：手动操控、使能/解除、姿态、导航按钮全部置灰并显示"只读"；**急停按钮始终可用**（任何人可急停）。
2. **使能/就绪卡片**（导航操控页右栏，替代现有"机身姿态"卡）：
   - 状态：未使能/已使能/请求中；`使能`、`解除` 按钮（二次确认），`站立`、`趴下`、`取消姿态`。
   - 就绪检查清单（逐项 ✓/✗）：链路、状态帧龄、basic==6、深度心跳、自动模式确认、电量、故障码。
   - 中文原因映射：`disarmed→未使能`、`stale_depth→深度心跳失效（检查相机/深度驱动）`、`mode_unconfirmed→自动模式未确认`、`remote_takeover→遥控器接管`、`motion_mode_not_ready→未站立`、`stale_state/stale_command_feedback→状态/反馈超时`、`vendor_error→厂商错误`、`body_tilt→机身倾角过大`、`disabled→未使能`。
3. **链路断线横幅**：`link.connected=false` 时顶部红色横幅"机器人链路断开，自动重连中（第 N 次）"，并给"立即重连"按钮；恢复后自动消失。数据过期时位姿数字变暗 + "数据过期"水印。
4. **健康卡片**（右栏或设备设置页）：电量（% + 低电红色）、`error_state`、`robot_motion_state`、主机/机载版本、`depth_age_s`、重连次数。
5. **急停闭环**：点击急停后检查 `wsSend` 返回值；必须在 1.5s 内从 `state.safety.estop` 确认，否则红色 toast"急停指令未确认，请立即使用物理急停/遥控器"；复位必须长按 ≥ 配置值并携带 `hold_s` 提交。
6. **地图修复**：切换到导航操控工作区时 `mapView.fit()`；地图元数据分辨率显示 3 位小数；点选目标超出地图范围/障碍区时拒绝并提示；轨迹/路径/机器人按现有渲染保留。
7. **令牌**：从 URL `?token=` 或 `localStorage.host_ui_token` 读取；REST 加 `X-Auth-Token`，WS 加 `?token=`；设备设置页提供令牌输入框（连接 0.0.0.0/局域网时用）。

### 4.2 协议对接
- `hello` 新增 `client_id`；`state.operator = {id, since, clients}`；按 `operator.id === client_id` 判断自己是否持有操控权。
- WS 发送：`claim_control` / `release_control` / `takeover_control` / `arm` / `disarm` / `posture {action}` / `posture_cancel` / `heartbeat`（每 2s）。
- 监听 `ack`（`{ok,message}`）、`event`、`state`；断线重连退避保留。
- 只在操作权持有者时发送 teleop/goal/arm/posture；观察者可发 estop/stop。

### 4.3 自测要求
- 起两个独立实例，不要碰 8710 端口：`python host_ui/run.py --port 8720 --no-open --link mock --config tmp/frontend_dev_config.json`。
- 用 Chrome CDP（参考 `tmp/ui_audit.py`、`tmp/cdp_eval.py`）做只读+交互验收：四工作区布局无横向溢出、地图 fit 后 `mapView.view.scale` 合理（>20 px/m）、操作权切换、使能清单、断线横幅、急停 ack、控制台无 error。
- 交付前跑 `node --check host_ui/web/app.js host_ui/web/workspace.js`。
- 把截图放到 `tmp/frontend_*.png` 并在报告中列出。

---

## 5. Lead 交付（并行）

- `state.py`：`SafetyCore` 增加 posture 锁存、`flags` 快照、nav 操作者心跳取消、estop 硬件急停联动、复位门槛。
- `node.py`：`arm()/disarm()/posture()/cancel_posture()/emergency_stop()`、导航操作者租约、health/battery/operator/posture 快照字段、录制补字段。
- `server.py`：Origin/令牌校验、操作权仲裁、`/api/arm|disarm|posture|posture/cancel|operator|health`、复位 hold、配置持久化、地图上传上限。
- `run.py`：`--link ssh_ros` 可选、`--token`、默认配置 `host_ui/config.json`、启动打印访问 URL（带 token）。
- 测试：`test_safety.py`（状态矩阵/姿态/急停/nav 心跳）、`test_operator.py`（WS 操作权）、`test_auth.py`（Origin/令牌）。
- 文档：`host_ui/README.md`、`host_ui/requirements-host-ui.txt`、`host_ui/start.bat`、部署检查单（含物理急停、`taskkill /F` 停车实测、相机/深度驱动启动、NTP 校时）。

---

## 6. 验收（批次 1 完成标准）

1. `python -m pytest host_ui/tests -q` 全绿（新增测试 ≥ 25 个用例）。
2. mock 链路演示：连接→站立→使能→手动前进→急停→复位→趴下→断开重连，全流程无控制台错误。
3. 页面：操作权、就绪清单、断线横幅、电量/健康、地图 fit 全部可见且状态正确。
4. 文档：新人按 README 能在 10 分钟内启动 mock 演示，并知道实机接线/急停/驱动启动步骤。
5. 实机验收（机器人可达后）：`depth` 心跳有效→站立→使能→0.1 m/s 前进→软件急停→厂商急停 0x21010C0E→disarm→趴下，全程记录到 `host_ui/maps/...` 或 `tmp/` 验收 json。