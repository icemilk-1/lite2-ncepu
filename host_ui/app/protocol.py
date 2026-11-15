"""Lite2 厂商 UDP 协议 V1.2.3 编解码（纯标准库）。

线格式（小端 little-endian，四字节对齐）
----------------------------------------
* **状态包 196 字节**

  ====== =============================================================
  偏移   内容
  ====== =============================================================
  0..12  帧头 ``<3i`` = (2305, 184, 1)
  12      ``<i`` basic：6 表示已站立并进入力控/可运动模式
  16      ``<i`` gait：步态编号
  20      ``<18d``：rpy_deg[3]、angular[3]、保留[3]、position[3]、
          velocity_world[3]、velocity_body[3]
  164     ``<I`` touch：触地/楼梯（*Lite 无效）
  168     bool is_charging（*Lite 无效）
  172     ``<I`` error_state（*Lite 无效，实测恒 0，只展示不门禁）
  176     ``<i`` motion：运动状态
  180     ``<d`` battery_level：文档为 0.00~1.00，实测部分固件直接给百分数
  188     ``<i`` task_state（*Lite 无效）
  192     bool is_robot_need_move
  193     bool zero_position_flag
  ====== =============================================================

* **遥控反馈包 60 字节**：``<3i`` 帧头 (2309, 48, 1) + ``<6d``；
  前 3 个 double 是遥控器摇杆 [x, y, z]，后 3 个是遥控器下发的目标速度
  [vx, vy, wz]。

* **速度包 20 字节**：``<3id`` = (code, 8, 1, value)，其中
  code 320 为线速度 m/s，325 为保留字段（固定 0），321 为偏航角速度 rad/s。
  三个包必须成组发送。

* **简单指令 12 字节**：``<3i`` = (code, value, 0)，见 :func:`simple_packet`。
  厂商 *Lite2 同一时刻只接受一个终端下发 ``0x2*******`` 控制指令，因此
  ``HEARTBEAT`` / ``ESTOP`` / ``POSTURE_TOGGLE`` 等只允许由注册终端（机载
  .103）代发；Windows 直发仅用于演示/中继，见 ``docs``。

线程模型与健壮性
----------------
本模块无全局状态、不做任何 I/O，纯函数，可被接收线程、看门狗线程和测试
同时调用。``decode_state`` / ``decode_command`` 对任何非法输入（长度不符、
帧头不符、字段非有限值、物理量越界）一律返回 ``None``，**不抛异常**，
由调用方负责计数与限频告警；``velocity_packets`` 对非有限速度抛
``ValueError``，避免 NaN 被下发给执行器。

与 ``lite2_sim/hardware/vendor_protocol.py`` 的 ``decode_state``、
``velocity_packets``、``decode_command`` 逐字节兼容（同一固件接口），
并在其上补齐 V1.2.3 状态包后半段字段（touch/is_charging/error_state/
battery/task_state/need_move/zero_position）与简单指令编解码。
"""
from __future__ import annotations

import math
import struct

#: 状态包长度（字节）
STATE_LENGTH = 196
#: 遥控反馈包长度（字节）
COMMAND_LENGTH = 60
#: 单个速度包长度（字节）
VELOCITY_LENGTH = 20
#: 简单指令长度（字节）
SIMPLE_LENGTH = 12
#: 状态包帧头
STATE_HEADER = (2305, 184, 1)
#: 遥控反馈包帧头
COMMAND_HEADER = (2309, 48, 1)
#: 速度包 code：线速度 / 保留 / 角速度
VELOCITY_CODES = (320, 325, 321)
#: 速度包载荷固定字段 (8, 1)
VELOCITY_MARK = (8, 1)

# ---------------------------------------------------------------------------
# 厂商 V1.2.3 简单指令码（1.2 节「手柄下发指令集」 / 1.3 节「感知主机下发」）
# ---------------------------------------------------------------------------
#: 心跳：发送频率必须 > 1 Hz
HEARTBEAT = 0x21040001
#: 急停（厂商硬件急停）
ESTOP = 0x21010C0E
#: 起立 / 趴下（**轮换**：每发一次切换一次，禁止连发）
POSTURE_TOGGLE = 0x21010202
#: 切换到移动模式
MOVE_MODE = 0x21010D06
#: 切换到站立模式
STAND_MODE = 0x21010D05
#: 切换到手动模式
MANUAL_MODE = 0x21010C02
#: 切换到自动模式
AUTO_MODE = 0x21010C03
#: 启动 / 取消持续运动：value -1 启动，2 取消
CONTINUOUS_MOTION = 0x21010C06
#: 回零
ZERO_POSITION = 0x21010C05
#: 保存数据
SAVE_DATA = 0x21010C01
#: 摇杆轴：前后平移（±6553 死区）
AXIS_FORWARD = 0x21010130
#: 摇杆轴：左右平移（±12553 死区）
AXIS_LATERAL = 0x21010131
#: 摇杆轴：左右转向（±9553 死区）
AXIS_TURN = 0x21010135
#: 摇杆轴：上下移动（±20000 死区）
AXIS_VERTICAL = 0x21010102

#: 简单指令值：持续运动启动 / 取消
CONTINUOUS_MOTION_START = -1
CONTINUOUS_MOTION_STOP = 2

# ---------------------------------------------------------------------------
# 状态包 basic 字段（附录 3.1 机器人基本运动状态）
# ---------------------------------------------------------------------------
BASIC_LIE = 1              # 正常趴着
BASIC_PREPARE = 4          # 准备起立
BASIC_STAND_UP = 5         # 起立中
BASIC_FORCE_CONTROL = 6    # 力控站立 / 步态中（可运动）
BASIC_STAND_DOWN = 7       # 坐下
BASIC_LOSE_CONTROL = 8     # 失控保护
BASIC_RECOVERY = 9         # 自主爬起
BASIC_TURNOVER = 11        # 自主翻倒
BASIC_ZERO_POSITION = 17   # 回零
BASIC_BACKFLIP = 18        # 后空翻
BASIC_GREETING = 19        # 打招呼

#: 起立/趴下动作 -> 目标 basic
POSTURE_TARGETS = {"stand": BASIC_FORCE_CONTROL, "lie": BASIC_LIE}
#: 允许发起轮换切换的当前 basic（只在稳定趴下/站立时切换）
POSTURE_SOURCE_STATES = (BASIC_LIE, BASIC_FORCE_CONTROL)
#: 电量原始值超过该阈值时视为固件已经直接给百分数
BATTERY_PERCENT_THRESHOLD = 1.5

# 状态包后半段字段偏移（绝对偏移，含 12 字节帧头）
STATE_TOUCH_OFFSET = 164
STATE_CHARGING_OFFSET = 168
STATE_ERROR_OFFSET = 172
STATE_MOTION_OFFSET = 176
STATE_BATTERY_OFFSET = 180
STATE_TASK_OFFSET = 188
STATE_NEED_MOVE_OFFSET = 192
STATE_ZERO_POSITION_OFFSET = 193

# 位置与姿态允许的最大绝对值（与参考实现一致）
_MAX_RPY_DEG = 180.0
#: 世界系/本体系速度的最大绝对值（与参考实现一致）
_MAX_VELOCITY = 5.0

_HEADER = struct.Struct("<3i")
_STATE_BASIC_GAIT = struct.Struct("<2i")
_STATE_VALUES = struct.Struct("<18d")
_STATE_MOTION = struct.Struct("<i")
_STATE_TOUCH = struct.Struct("<I")
_STATE_ERROR = struct.Struct("<I")
_STATE_BATTERY = struct.Struct("<d")
_STATE_TASK = struct.Struct("<i")
_SIMPLE = struct.Struct("<3i")
_COMMAND_VALUES = struct.Struct("<6d")
_VELOCITY = struct.Struct("<3id")


def simple_packet(code: int, value: int = 0) -> bytes:
    """打包 12 字节简单指令：``<3i`` = (code, value, 0)。

    ``type=0`` 表示不带复杂数据。厂商 *Lite2 的控制类指令（``0x2*******``）
    只接受一个终端下发，且多数是电平/轮换语义，调用方不得循环连发
    （``POSTURE_TOGGLE`` 循环发送会让机器狗反复起立/趴下）。
    """
    return _SIMPLE.pack(int(code) & 0xFFFFFFFF, int(value), 0)


def heartbeat_packet() -> bytes:
    """心跳包（``HEARTBEAT``，指令值无意义）。"""
    return simple_packet(HEARTBEAT)


def estop_packet() -> bytes:
    """厂商硬件急停包（``ESTOP``）。"""
    return simple_packet(ESTOP)


def posture_packet() -> bytes:
    """起立/趴下轮换包（``POSTURE_TOGGLE``，单次有效）。"""
    return simple_packet(POSTURE_TOGGLE)


def normalize_battery_level(raw) -> float | None:
    """把厂商电量原始值归一为 0~100 的百分数；非法值返回 ``None``。

    V1.2.3 文档写 ``battery_level`` 范围 0.00~1.00，但实测固件会直接给
    59/67/87 这样的百分数，因此 **只有 ≤1.5 时才 ×100**（契约第 2 节）。
    """
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value < 0:
        return None
    percent = value * 100.0 if value <= BATTERY_PERCENT_THRESHOLD else value
    return max(0.0, min(100.0, percent))


def posture_target(action: str) -> int | None:
    """姿态动作 -> 目标 basic；非法动作返回 ``None``。"""
    return POSTURE_TARGETS.get(str(action))


def decode_state(packet: bytes | bytearray | memoryview) -> dict | None:
    """解码 196 字节状态包；非法包返回 ``None``。

    返回结构与参考实现兼容（列表字段便于直接 JSON 化）::

        {"basic": int, "gait": int, "motion": int,
         "rpy_deg": [roll, pitch, yaw],
         "angular": [wx, wy, wz],
         "position": [x, y, z],
         "velocity_world": [vx, vy, vz],
         "velocity_body": [vx, vy, vz],
         "battery_level_raw": float|None, "battery_percent": float|None,
         "error_state": int, "is_charging": bool,
         "touch_down_and_stair_trot": int, "task_state": int,
         "is_robot_need_move": bool, "zero_position_flag": bool}

    校验规则（与厂商 V1.2.3 参考实现一致）：

    * 长度必须为 196 字节且帧头为 (2305, 184, 1)；
    * 18 个 double 必须全部有限；
    * 姿态角绝对值不超过 180°，世界系/本体系速度绝对值不超过 5。

    ``battery_level`` / ``is_charging`` / ``error_state`` 在文档里标注
    *Lite 无效*：电量做有限性与归一处理，故障码原样透传但**不作为门禁**
    （``error_state`` 实测恒 0，只用于界面展示）。
    """
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        return None
    if len(packet) != STATE_LENGTH:
        return None
    try:
        if _HEADER.unpack_from(packet, 0) != STATE_HEADER:
            return None
        basic, gait = _STATE_BASIC_GAIT.unpack_from(packet, 12)
        values = _STATE_VALUES.unpack_from(packet, 20)
        motion = _STATE_MOTION.unpack_from(packet, STATE_MOTION_OFFSET)[0]
        touch = _STATE_TOUCH.unpack_from(packet, STATE_TOUCH_OFFSET)[0]
        error_state = _STATE_ERROR.unpack_from(packet, STATE_ERROR_OFFSET)[0]
        battery_raw = _STATE_BATTERY.unpack_from(packet, STATE_BATTERY_OFFSET)[0]
        task_state = _STATE_TASK.unpack_from(packet, STATE_TASK_OFFSET)[0]
        charging = bool(packet[STATE_CHARGING_OFFSET])
        need_move = bool(packet[STATE_NEED_MOVE_OFFSET])
        zero_position = bool(packet[STATE_ZERO_POSITION_OFFSET])
    except (struct.error, IndexError):  # 理论上前面的长度检查已覆盖，防御性兜底
        return None
    if (not all(math.isfinite(value) for value in values)
            or max(abs(value) for value in values[:3]) > _MAX_RPY_DEG
            or max(abs(value) for value in values[12:18]) > _MAX_VELOCITY):
        return None
    battery_level = normalize_battery_level(battery_raw)
    return {
        "basic": int(basic),
        "gait": int(gait),
        "motion": int(motion),
        "rpy_deg": list(values[0:3]),
        "angular": list(values[3:6]),
        "position": list(values[9:12]),
        "velocity_world": list(values[12:15]),
        "velocity_body": list(values[15:18]),
        "battery_level_raw": (float(battery_raw)
                              if math.isfinite(battery_raw) else None),
        "battery_percent": battery_level,
        "error_state": int(error_state),
        "is_charging": charging,
        "touch_down_and_stair_trot": int(touch),
        "task_state": int(task_state),
        "is_robot_need_move": need_move,
        "zero_position_flag": zero_position,
    }


def velocity_packets(linear: float, angular: float) -> list[bytes]:
    """生成一组（3 个）速度指令包，与参考实现逐字节兼容。

    参数
    ----
    linear:
        前进速度，单位 m/s，对应 code 320。
    angular:
        偏航角速度，单位 rad/s，逆时针为正，对应 code 321；code 325 固定 0。

    异常
    ----
    ValueError
        速度不是有限数值（NaN/Inf 绝不能下发到执行器）。
    """
    linear = float(linear)
    angular = float(angular)
    if not (math.isfinite(linear) and math.isfinite(angular)):
        raise ValueError("速度指令必须是有限数值")
    return [
        _VELOCITY.pack(code, VELOCITY_MARK[0], VELOCITY_MARK[1], value)
        for code, value in ((320, linear), (325, 0.0), (321, angular))
    ]


def safety_packets() -> list[bytes]:
    """零速安全包（等价于 ``velocity_packets(0.0, 0.0)``）。"""
    return velocity_packets(0.0, 0.0)


def decode_command(packet: bytes | bytearray | memoryview) -> dict | None:
    """解码 60 字节遥控反馈包；非法包返回 ``None``。

    返回 ``{"joystick": [x, y, z], "goal_velocity": [vx, vy, wz]}``，
    其中 ``goal_velocity`` 是遥控器当前给出的目标速度，供上位机判断
    是否发生人工接管（见 ``Frame.joystick_active``）。
    """
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        return None
    if len(packet) != COMMAND_LENGTH:
        return None
    try:
        if _HEADER.unpack_from(packet, 0) != COMMAND_HEADER:
            return None
        values = _COMMAND_VALUES.unpack_from(packet, 12)
    except struct.error:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return {"joystick": list(values[:3]), "goal_velocity": list(values[3:])}


def decode_any(packet: bytes | bytearray | memoryview) -> dict | None:
    """按长度/帧头区分 196 状态包 / 60 遥控反馈 / 12 简单指令 ACK。

    返回统一带 ``kind`` / ``type`` / ``code`` / ``value`` 的字典：

    * ``kind="state"``：额外带 :func:`decode_state` 的全部字段；
    * ``kind="command"``：额外带 ``joystick`` / ``goal_velocity``；
    * ``kind="simple"``：``code`` / ``value`` 为 12 字节简单指令内容。

    非法包返回 ``None``（不抛异常）。主链路仍用 :func:`decode_state` /
    :func:`decode_command`，本函数便于调试工具、抓包解析与单测。
    """
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        return None
    length = len(packet)
    if length == SIMPLE_LENGTH:
        try:
            code, value, kind = _SIMPLE.unpack(packet)
        except struct.error:
            return None
        return {"kind": "simple", "type": int(kind),
                "code": int(code), "value": int(value)}
    if length == COMMAND_LENGTH:
        report = decode_command(packet)
        if report is None:
            return None
        return {"kind": "command", "type": 1, "code": COMMAND_HEADER[0],
                "value": VELOCITY_MARK[1],
                "joystick": report["joystick"],
                "goal_velocity": report["goal_velocity"]}
    if length == STATE_LENGTH:
        report = decode_state(packet)
        if report is None:
            return None
        result = {"kind": "state", "type": 1, "code": STATE_HEADER[0], "value": 0}
        result.update(report)
        return result
    return None


# ---------------------------------------------------------------------------
# 复杂指令（机载信息上报，type=1）
# ---------------------------------------------------------------------------
JOINT_TEMPERATURE = 0x11050400   # 关节温度 int[12]（仅过温时上报）
JOINT_ANGLES = 0x0902            # 关节角度 double[12]
JOINT_VELOCITIES = 0x0903        # 关节角速度 double[12]

_COMPLEX_JOINT_TEMP = struct.Struct("<12i")
_COMPLEX_JOINT_VALUE = struct.Struct("<12d")


def decode_complex(packet: bytes | bytearray | memoryview) -> dict | None:
    """解码 type=1 的复杂上报包（关节温度/角度/角速度）。

    返回 ``{"kind": "joint_temperature"|"joint_angles"|"joint_velocities"
    |"complex", ...}``；结构不符返回 ``None``。这些数据只在厂商主动上报时
    出现（温度仅过温上报），因此 UI 必须允许"未接入/无数据"状态。
    """
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        return None
    if len(packet) < 12:
        return None
    try:
        code, size, kind = _HEADER.unpack_from(packet, 0)
    except struct.error:
        return None
    if kind != 1 or size <= 0 or size % 4 or len(packet) < 12 + size:
        return None
    payload = bytes(packet[12:12 + size])
    if code == JOINT_TEMPERATURE and size == 48:
        values = list(_COMPLEX_JOINT_TEMP.unpack(payload))
        return {"kind": "joint_temperature", "type": 1, "code": int(code),
                "temperature_c": values, "max_c": max(values)}
    if code in (JOINT_ANGLES, JOINT_VELOCITIES) and size == 96:
        values = list(_COMPLEX_JOINT_VALUE.unpack(payload))
        if not all(math.isfinite(value) for value in values):
            return None
        name = "joint_angles" if code == JOINT_ANGLES else "joint_velocities"
        return {"kind": name, "type": 1, "code": int(code), name: values}
    return {"kind": "complex", "type": 1, "code": int(code), "size": int(size)}


def decode_raw_hex(text: str | bytes) -> dict | None:
    """把 ``/airace/raw_packet`` 的十六进制字符串解成结构化字典。"""
    if isinstance(text, bytes):
        text = text.decode("ascii", "ignore")
    if not isinstance(text, str):
        return None
    text = text.strip()
    if not text or len(text) % 2 or len(text) > 2048:
        return None
    try:
        packet = bytes.fromhex(text)
    except ValueError:
        return None
    if len(packet) >= 12:
        try:
            _code, size, kind = _HEADER.unpack_from(packet, 0)
        except struct.error:
            size, kind = 0, 0
        if kind == 1 and size > 0:
            report = decode_complex(packet)
            if report is not None:
                return report
    return decode_any(packet)