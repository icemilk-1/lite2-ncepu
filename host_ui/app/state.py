"""安全控制与遥测缓存。

安全链路的设计原则（按优先级从高到低）：

1. **急停锁存**：一旦置位，任何来源的速度指令都被拦截为 0，必须显式复位。
2. **状态超时**：超过 ``state_timeout_s`` 没有新的状态反馈 -> 零速。
3. **链路断开**：传输层报错/停止 -> 零速。
4. **模式未就绪**：厂商状态 ``basic != ready_basic``（未站立/非力控）-> 零速。
5. **厂商遥控接管**：检测到随机遥控摇杆动作 -> 零速并保持一段时间。
6. **操作心跳超时**：浏览器超过 ``command_timeout_s`` 没发指令 -> 零速。
7. **限幅与加速度限制**：最终下发前再夹一次速度与角速度。

所有方法都可由不同线程调用，内部用锁保护。
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

from .types import Frame, clamp


# --------------------------------------------------------------------------- 地图对齐
def transform_pose(x: float, y: float, yaw_deg: float,
                   align: dict | None) -> tuple[float, float, float]:
    """里程计坐标 -> 地图坐标（align 描述地图相对里程计的平移/旋转）。"""
    if not align:
        return float(x), float(y), float(yaw_deg)
    ax = float(align.get("x", 0.0))
    ay = float(align.get("y", 0.0))
    ayaw = math.radians(float(align.get("yaw_deg", 0.0)))
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * x - s * y,
            ay + s * x + c * y,
            yaw_deg + math.degrees(ayaw))


def inverse_transform(x: float, y: float, align: dict | None) -> tuple[float, float]:
    """地图坐标 -> 里程计坐标。"""
    if not align:
        return float(x), float(y)
    ax = float(align.get("x", 0.0))
    ay = float(align.get("y", 0.0))
    ayaw = math.radians(float(align.get("yaw_deg", 0.0)))
    c, s = math.cos(ayaw), math.sin(ayaw)
    dx, dy = x - ax, y - ay
    return c * dx + s * dy, -s * dx + c * dy


# --------------------------------------------------------------------------- 数据类
@dataclass
class Decision:
    """控制循环一次输出。"""

    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    source: str = "idle"
    zero_reason: str = ""
    flags: dict[str, Any] = field(default_factory=dict)
    stamp: float = 0.0

    @property
    def moving(self) -> bool:
        return abs(self.linear_m_s) > 1e-6 or abs(self.angular_rad_s) > 1e-6


class SafetyCore:
    """速度指令的安全闸门。"""

    #: 会真正拦截运动的原因（前端据此显示红色告警）
    FAULTS = ("estop", "state_timeout", "link_down", "mode_not_ready",
              "joystick_takeover")

    def __init__(self, limits: dict, safety: dict) -> None:
        self._lock = threading.RLock()
        self.limits = dict(limits)
        self.safety = dict(safety)
        self._estop = False
        self._estop_reason = ""
        self._estop_stamp = 0.0
        self._estop_count = 0
        self._teleop_linear = 0.0
        self._teleop_angular = 0.0
        self._teleop_deadman = False
        self._teleop_stamp = 0.0
        self._nav_linear = 0.0
        self._nav_angular = 0.0
        self._nav_active = False
        self._takeover_until = 0.0
        self._posture_action: str | None = None
        self._posture_since = 0.0
        self._last_output = (0.0, 0.0)
        self._last_output_stamp = 0.0
        self.last_decision = Decision()
        self.counters = {"ticks": 0, "zero_outputs": 0, "clamped": 0}

    # ------------------------------------------------------------------ 参数
    def update_config(self, limits: dict, safety: dict) -> None:
        with self._lock:
            self.limits = dict(limits)
            self.safety = dict(safety)

    # ------------------------------------------------------------------ 急停
    def engage_estop(self, reason: str = "手动急停") -> None:
        with self._lock:
            if not self._estop:
                self._estop_count += 1
            self._estop = True
            self._estop_reason = str(reason)
            self._estop_stamp = time.monotonic()
            self._teleop_linear = 0.0
            self._teleop_angular = 0.0
            self._teleop_deadman = False
            self._nav_active = False
            self._nav_linear = self._nav_angular = 0.0
            self._last_output = (0.0, 0.0)

    def reset_estop(self) -> tuple[bool, str]:
        with self._lock:
            if not self._estop:
                return True, "急停未处于锁存状态"
            self._estop = False
            self._estop_reason = ""
            self._estop_stamp = 0.0
            self._teleop_linear = self._teleop_angular = 0.0
            self._teleop_deadman = False
            self._last_output = (0.0, 0.0)
            return True, "急停已复位，需重新按住方向键才可运动"

    @property
    def estop(self) -> bool:
        with self._lock:
            return self._estop

    @property
    def estop_reason(self) -> str:
        with self._lock:
            return self._estop_reason

    # ------------------------------------------------------------------ 指令输入
    def set_teleop(self, linear: float, angular: float, deadman: bool,
                   now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            if not (math.isfinite(linear) and math.isfinite(angular)):
                return
            self._teleop_linear = float(linear)
            self._teleop_angular = float(angular)
            self._teleop_deadman = bool(deadman)
            self._teleop_stamp = float(now)

    def release_teleop(self) -> None:
        with self._lock:
            self._teleop_linear = 0.0
            self._teleop_angular = 0.0
            self._teleop_deadman = False

    def set_nav_command(self, linear: float, angular: float, active: bool) -> None:
        with self._lock:
            self._nav_linear = float(linear) if math.isfinite(linear) else 0.0
            self._nav_angular = float(angular) if math.isfinite(angular) else 0.0
            self._nav_active = bool(active)

    def clear_nav(self) -> None:
        with self._lock:
            self._nav_linear = self._nav_angular = 0.0
            self._nav_active = False

    # ------------------------------------------------------------------ 姿态切换锁存
    def begin_posture(self, action: str, now: float | None = None) -> None:
        """站/趴切换期间禁止一切速度输出，直到 :meth:`end_posture`。"""
        with self._lock:
            self._posture_action = str(action)
            self._posture_since = time.monotonic() if now is None else now
            self._teleop_linear = self._teleop_angular = 0.0
            self._teleop_deadman = False
            self._nav_active = False
            self._nav_linear = self._nav_angular = 0.0
            self._last_output = (0.0, 0.0)

    def end_posture(self) -> None:
        with self._lock:
            self._posture_action = None
            self._posture_since = 0.0

    @property
    def posture_active(self) -> bool:
        with self._lock:
            return self._posture_action is not None

    @property
    def posture_action(self) -> str | None:
        with self._lock:
            return self._posture_action

    # ------------------------------------------------------------------ 核心 tick
    def tick(self, now: float, frame: Frame | None, link_up: bool) -> Decision:
        """计算本周期最终下发速度。``now`` 用 ``time.monotonic()``。"""
        with self._lock:
            self.counters["ticks"] += 1
            safety = self.safety
            limits = self.limits
            command_timeout = float(safety.get("command_timeout_s", 0.35))
            state_timeout = float(safety.get("state_timeout_s", 0.8))
            ready_basic = int(safety.get("ready_basic", 6))
            joystick_threshold = float(safety.get("joystick_threshold", 0.03))
            takeover_hold = float(safety.get("takeover_hold_s", 0.5))

            flags: dict[str, Any] = {}
            reason = ""
            source = "idle"

            # 1) 急停锁存优先级最高
            if self._estop:
                reason = "estop"
                flags["estop"] = True
            # 1.5) 姿态切换期间禁止运动（站/趴是有风险的机械动作）
            elif self._posture_action is not None:
                reason = "posture"
                flags["posture"] = self._posture_action
                flags["posture_age_s"] = round(now - self._posture_since, 2)
            # 2) 状态超时 / 链路断开
            elif frame is None or (now - frame.stamp) > state_timeout:
                reason = "state_timeout"
                flags["state_age_s"] = None if frame is None else now - frame.stamp
            elif not link_up:
                reason = "link_down"
            # 3) 未站立 / 模式不对
            elif int(frame.basic) != ready_basic:
                reason = "mode_not_ready"
                flags["basic"] = frame.basic
                flags["ready_basic"] = ready_basic
            else:
                # 4) 厂商遥控接管（带保持时间）
                if frame.joystick_active(joystick_threshold):
                    self._takeover_until = now + takeover_hold
                if now < self._takeover_until:
                    reason = "joystick_takeover"
                    flags["joystick"] = list(frame.joystick)

            if not reason:
                # 5) 选择指令来源：手动优先于导航
                teleop_fresh = (now - self._teleop_stamp) <= command_timeout
                if teleop_fresh and self._teleop_deadman:
                    linear, angular = self._teleop_linear, self._teleop_angular
                    source = "teleop"
                elif teleop_fresh and (self._teleop_linear or self._teleop_angular):
                    # 有输入但未按死按钮：按安全要求直接停车
                    linear = angular = 0.0
                    source = "idle"
                    reason = "deadman"
                elif self._nav_active:
                    linear, angular = self._nav_linear, self._nav_angular
                    source = "nav"
                else:
                    linear = angular = 0.0
                    source = "idle"
                    reason = "no_input"

                # 6) 硬限幅
                max_forward = float(limits.get("max_forward_m_s", 0.3))
                max_reverse = float(limits.get("max_reverse_m_s", 0.08))
                max_yaw = float(limits.get("max_yaw_rad_s", 0.8))
                clamped_linear = clamp(linear, -max_reverse, max_forward)
                clamped_angular = clamp(angular, -max_yaw, max_yaw)
                if (abs(clamped_linear - linear) > 1e-9
                        or abs(clamped_angular - angular) > 1e-9):
                    flags["clamped"] = True
                    self.counters["clamped"] += 1
                linear, angular = clamped_linear, clamped_angular

                # 7) 加速度/角加速度限制（按时间步长）
                if self._last_output_stamp > 0:
                    dt = clamp(now - self._last_output_stamp, 0.0, 0.30)
                else:
                    dt = 0.04
                max_accel = float(limits.get("max_accel_m_s2", 0.8))
                max_yaw_accel = float(limits.get("max_yaw_accel_rad_s2", 3.0))
                prev_linear, prev_angular = self._last_output
                linear = prev_linear + clamp(linear - prev_linear, -max_accel * dt,
                                             max_accel * dt)
                angular = prev_angular + clamp(angular - prev_angular,
                                               -max_yaw_accel * dt, max_yaw_accel * dt)

                self._last_output = (linear, angular)
                self._last_output_stamp = now

            if reason:
                linear = angular = 0.0
                self._last_output = (0.0, 0.0)
                self._last_output_stamp = now
                if reason in self.FAULTS:
                    self.counters["zero_outputs"] += 1

            decision = Decision(
                linear_m_s=float(linear),
                angular_rad_s=float(angular),
                source=source,
                zero_reason=reason,
                flags=flags,
                stamp=now,
            )
            self.last_decision = decision
            return decision

    # ------------------------------------------------------------------ 快照
    def snapshot(self) -> dict:
        with self._lock:
            decision = self.last_decision
            return {
                "estop": self._estop,
                "estop_reason": self._estop_reason,
                "estop_engages": self._estop_count,
                "teleop": {
                    "linear": self._teleop_linear,
                    "angular": self._teleop_angular,
                    "deadman": self._teleop_deadman,
                    "age_s": (time.monotonic() - self._teleop_stamp)
                    if self._teleop_stamp else None,
                },
                "nav_active": self._nav_active,
                "posture": {"active": self._posture_action is not None,
                            "action": self._posture_action,
                            "age_s": round(time.monotonic() - self._posture_since, 2)
                            if self._posture_since else None},
                "output": {"linear": decision.linear_m_s,
                           "angular": decision.angular_rad_s,
                           "source": decision.source,
                           "zero_reason": decision.zero_reason,
                           "flags": dict(decision.flags)},
                "counters": dict(self.counters),
            }


# --------------------------------------------------------------------------- 遥测辅助
class RateMeter:
    """滑动窗口频率统计。"""

    def __init__(self, window: int = 60) -> None:
        self._stamps: deque[float] = deque(maxlen=window)

    def tick(self, now: float | None = None) -> float:
        self._stamps.append(time.monotonic() if now is None else now)
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 1e-6 else 0.0

    @property
    def hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 1e-6 else 0.0


class TrailTracker:
    """轨迹点缓存 + 里程统计。"""

    def __init__(self, max_points: int = 6000, min_spacing_m: float = 0.03) -> None:
        self.points: deque[tuple[float, float]] = deque(maxlen=max_points)
        self.min_spacing_m = float(min_spacing_m)
        self._odometer_m = 0.0
        self._pending: list[tuple[float, float]] = []
        self._lock = threading.Lock()

    @property
    def odometer_m(self) -> float:
        with self._lock:
            return self._odometer_m

    def update(self, x: float, y: float) -> bool:
        """追加一个位置，返回是否新增了轨迹点。"""
        with self._lock:
            if self.points:
                last_x, last_y = self.points[-1]
                distance = math.hypot(x - last_x, y - last_y)
                if distance < 1e-6:
                    return False
                # 只有真正追加轨迹点时才累计里程，避免逐帧噪声把里程计放大
                if distance < self.min_spacing_m:
                    return False
                self._odometer_m += distance
            self.points.append((float(x), float(y)))
            self._pending.append((float(x), float(y)))
            return True

    def pending(self) -> list[tuple[float, float]]:
        """取出尚未推送给前端的增量点。"""
        with self._lock:
            points, self._pending = self._pending, []
            return points

    def all_points(self) -> list[tuple[float, float]]:
        with self._lock:
            return list(self.points)

    def reset(self) -> None:
        with self._lock:
            self.points.clear()
            self._pending.clear()
            self._odometer_m = 0.0


class EventLog:
    """环形事件日志，支持增量拉取。"""

    def __init__(self, max_events: int = 400) -> None:
        self._lock = threading.Lock()
        self.events: deque[dict] = deque(maxlen=max_events)
        self._seq = 0
        self._delivered = 0

    def add(self, level: str, code: str, message: str, **detail: Any) -> dict:
        with self._lock:
            self._seq += 1
            now = time.time()
            event = {
                "seq": self._seq,
                "ts": now,
                "time": datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3],
                "level": str(level),
                "code": str(code),
                "message": str(message),
            }
            if detail:
                event["detail"] = detail
            self.events.append(event)
            return event

    def drain(self) -> list[dict]:
        """返回上次之后新产生的事件。"""
        with self._lock:
            new = [event for event in self.events if event["seq"] > self._delivered]
            if new:
                self._delivered = new[-1]["seq"]
            return new

    def recent(self, count: int = 80) -> list[dict]:
        with self._lock:
            return list(self.events)[-int(count):]

    def reset_delivery(self) -> None:
        with self._lock:
            self._delivered = 0