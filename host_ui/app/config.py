"""上位机配置：默认值、校验与持久化。

配置分四层：链路（link）、安全限幅（limits/safety）、地图（map）、导航（nav）。
所有运行期可调参数都经过 :func:`normalize` 收敛到安全范围，防止界面填错值
直接把机器狗速度上限放大。
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any


DEFAULTS: dict[str, Any] = {
    "schema_version": 1,
    "link": {
        "kind": "mock",
        "options": {},
        "reconnect": {
            "enabled": True,
            "min_delay_s": 1.0,
            "max_delay_s": 10.0,
        },
    },
    "limits": {
        # 硬件/安全层硬限幅，界面滑块不能超过它
        "max_forward_m_s": 1.00,
        "max_reverse_m_s": 1.00,
        "max_yaw_rad_s": 1.00,
        "max_accel_m_s2": 0.80,
        "max_yaw_accel_rad_s2": 3.00,
        # 默认手动点动速度
        "teleop_linear_m_s": 0.50,
        "teleop_angular_rad_s": 0.50,
    },
    "safety": {
        "command_timeout_s": 0.35,   # 浏览器心跳超时 -> 零速
        "state_timeout_s": 0.80,     # 状态反馈超时 -> 零速
        "ready_basic": 6,            # basic==6 视为站立/力控可运动
        "joystick_threshold": 0.03,  # 厂商遥控摇杆接管阈值
        "takeover_hold_s": 0.50,     # 摇杆回中后继续屏蔽的保持时间
        "estop_reset_hold_s": 1.20,  # 界面长按复位时间（仅 UI 参考）
        "low_battery_percent": 20.0, # 低电量告警阈值
        "nav_operator_timeout_s": 2.50,  # 无操作者心跳时导航/探索自动取消（前端 1 s 心跳 + 1.5 s 容差）
        "require_operator": True,    # 是否强制"操作权"独占
    },
    "posture": {
        "timeout_s": 20.0,           # 站/趴切换确认超时（包含首次起立准备时间）
        "settle_margin_s": 1.0,      # 姿态切换期间额外禁运动时间
    },
    "nav": {
        "enabled": True,
        "max_speed_m_s": 0.50,
        "max_yaw_rad_s": 0.50,
        "tolerance_m": 0.15,
        "yaw_tolerance_deg": 15.0,
        "lookahead_m": 0.45,
        "inflate_m": 0.30,
        "allow_unknown": False,
    },
    "explore": {
        # 前沿探索建图：只产生"下一个目标点"，速度仍由 nav + SafetyCore 下发
        "enabled": True,
        "min_frontier_cells": 8,        # 小于该规模的前沿簇忽略
        "max_goal_distance_m": 15.0,    # 单次探索目标的最远直线距离
        "gain_weight": 0.5,             # 信息增益权重：score = dist - weight*sqrt(gain)
        "blacklist_radius_m": 0.30,     # 卡死/不可达目标的拉黑半径
        "max_backoff_m": 0.60,          # 前沿目标向机器人一侧回退（避开障碍膨胀区）的最大距离
        "max_consecutive_failures": 3,  # 连续失败多少次判定探索失败
        "observation_wait_s": 3.0,      # 到达前沿后停车观测，避免原地重复选点
        "observation_timeout_s": 15.0,  # 无新增候选时有界等待，超时暂停
    },
    "map": {
        "file": None,              # 启动时加载的地图路径；None 则按 demo 生成
        "demo": True,
        "align": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
    },
    "web": {
        "host": "127.0.0.1",
        "port": 8710,
        "token": "",                 # 访问令牌；留空时仅允许回环地址且校验 Origin
        "allowed_origins": [],       # 额外允许的 Origin，如 http://192.168.1.101:8710
        "max_clients": 6,
        "max_message_bytes": 65536,
    },
    "record": {
        "enabled": False,
        "dir": "host_ui/sessions",
        "hz": 10.0,
    },
}


def deep_merge(base: dict, override: dict) -> dict:
    """递归合并字典，override 优先；返回新字典。"""
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _number(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def normalize(config: dict) -> tuple[dict, list[str]]:
    """把配置收敛到安全范围，返回 (新配置, 警告列表)。"""
    data = deep_merge(DEFAULTS, config or {})
    warnings: list[str] = []

    limits = data["limits"]
    original = dict(limits)
    limits["max_forward_m_s"] = _clamp(
        _number(limits.get("max_forward_m_s"), 1.00), 0.01, 1.00)
    limits["max_reverse_m_s"] = _clamp(
        _number(limits.get("max_reverse_m_s"), 1.00), 0.0, 1.00)
    limits["max_yaw_rad_s"] = _clamp(
        _number(limits.get("max_yaw_rad_s"), 1.00), 0.05, 2.00)
    limits["max_accel_m_s2"] = _clamp(
        _number(limits.get("max_accel_m_s2"), 0.80), 0.05, 5.00)
    limits["max_yaw_accel_rad_s2"] = _clamp(
        _number(limits.get("max_yaw_accel_rad_s2"), 3.00), 0.10, 20.0)
    limits["teleop_linear_m_s"] = _clamp(
        _number(limits.get("teleop_linear_m_s"), 0.50), 0.01, limits["max_forward_m_s"])
    limits["teleop_angular_rad_s"] = _clamp(
        _number(limits.get("teleop_angular_rad_s"), 0.50), 0.01, limits["max_yaw_rad_s"])
    if limits != original:
        warnings.append("速度限幅已收敛到安全范围")

    safety = data["safety"]
    safety["command_timeout_s"] = _clamp(
        _number(safety.get("command_timeout_s"), 0.35), 0.10, 2.00)
    safety["state_timeout_s"] = _clamp(
        _number(safety.get("state_timeout_s"), 0.80), 0.15, 5.00)
    safety["ready_basic"] = int(_clamp(_number(safety.get("ready_basic"), 6), 0, 100))
    safety["joystick_threshold"] = _clamp(
        _number(safety.get("joystick_threshold"), 0.03), 0.01, 0.50)
    safety["takeover_hold_s"] = _clamp(
        _number(safety.get("takeover_hold_s"), 0.50), 0.0, 5.00)
    safety["estop_reset_hold_s"] = _clamp(
        _number(safety.get("estop_reset_hold_s"), 1.20), 0.30, 5.00)
    safety["low_battery_percent"] = _clamp(
        _number(safety.get("low_battery_percent"), 20.0), 0.0, 100.0)
    safety["nav_operator_timeout_s"] = _clamp(
        _number(safety.get("nav_operator_timeout_s"), 2.50), 0.30, 10.0)
    safety["require_operator"] = bool(safety.get("require_operator", True))

    posture = data["posture"]
    posture["timeout_s"] = _clamp(_number(posture.get("timeout_s"), 20.0), 2.0, 30.0)
    posture["settle_margin_s"] = _clamp(
        _number(posture.get("settle_margin_s"), 1.0), 0.0, 5.0)

    reconnect = data["link"].setdefault("reconnect", {})
    reconnect["enabled"] = bool(reconnect.get("enabled", True))
    reconnect["min_delay_s"] = _clamp(
        _number(reconnect.get("min_delay_s"), 1.0), 0.2, 30.0)
    reconnect["max_delay_s"] = _clamp(
        _number(reconnect.get("max_delay_s"), 10.0),
        reconnect["min_delay_s"], 120.0)

    nav = data["nav"]
    nav["max_speed_m_s"] = _clamp(
        _number(nav.get("max_speed_m_s"), 0.50), 0.01, limits["max_forward_m_s"])
    nav["max_yaw_rad_s"] = _clamp(
        _number(nav.get("max_yaw_rad_s"), 0.50), 0.05, limits["max_yaw_rad_s"])
    nav["tolerance_m"] = _clamp(_number(nav.get("tolerance_m"), 0.15), 0.05, 1.00)
    nav["yaw_tolerance_deg"] = _clamp(
        _number(nav.get("yaw_tolerance_deg"), 15.0), 2.0, 90.0)
    nav["lookahead_m"] = _clamp(_number(nav.get("lookahead_m"), 0.45), 0.15, 2.00)
    nav["inflate_m"] = _clamp(_number(nav.get("inflate_m"), 0.30), 0.0, 1.00)
    nav["enabled"] = bool(nav.get("enabled", True))
    nav["allow_unknown"] = bool(nav.get("allow_unknown", False))

    explore = data["explore"]
    explore["enabled"] = bool(explore.get("enabled", True))
    explore["min_frontier_cells"] = int(_clamp(
        _number(explore.get("min_frontier_cells"), 8), 1, 100))
    explore["max_goal_distance_m"] = _clamp(
        _number(explore.get("max_goal_distance_m"), 15.0), 0.5, 50.0)
    explore["gain_weight"] = _clamp(
        _number(explore.get("gain_weight"), 0.5), 0.0, 5.0)
    explore["blacklist_radius_m"] = _clamp(
        _number(explore.get("blacklist_radius_m"), 0.30), 0.05, 2.0)
    explore["max_backoff_m"] = _clamp(
        _number(explore.get("max_backoff_m"), 0.60), 0.0, 2.0)
    explore["max_consecutive_failures"] = int(_clamp(
        _number(explore.get("max_consecutive_failures"), 3), 1, 10))
    explore["observation_wait_s"] = _clamp(
        _number(explore.get("observation_wait_s"), 3.0), 0.1, 15.0)
    explore["observation_timeout_s"] = _clamp(
        _number(explore.get("observation_timeout_s"), 15.0),
        explore["observation_wait_s"], 120.0)

    align = data["map"].setdefault("align", {})
    align["x"] = _number(align.get("x"), 0.0)
    align["y"] = _number(align.get("y"), 0.0)
    align["yaw_deg"] = _number(align.get("yaw_deg"), 0.0)

    data["web"]["host"] = str(data["web"].get("host") or "127.0.0.1")
    data["web"]["port"] = int(_clamp(_number(data["web"].get("port"), 8710), 1, 65535))
    data["web"]["token"] = str(data["web"].get("token") or "")
    origins = data["web"].get("allowed_origins") or []
    if isinstance(origins, str):
        origins = [item.strip() for item in origins.split(",") if item.strip()]
    data["web"]["allowed_origins"] = [str(item) for item in origins
                                      if str(item).strip()]
    data["web"]["max_clients"] = int(_clamp(
        _number(data["web"].get("max_clients"), 6), 1, 32))
    data["web"]["max_message_bytes"] = int(_clamp(
        _number(data["web"].get("max_message_bytes"), 65536), 1024, 1024 * 1024))
    data["record"]["hz"] = _clamp(_number(data["record"].get("hz"), 10.0), 1.0, 50.0)
    data["record"]["enabled"] = bool(data["record"].get("enabled", False))
    data["record"]["dir"] = str(data["record"].get("dir") or "host_ui/sessions")

    kind = str(data["link"].get("kind") or "mock").strip().lower()
    if kind == "vendor":
        kind = "udp"
    elif kind == "ros1":
        kind = "ros"
    data["link"]["kind"] = kind
    if not isinstance(data["link"].get("options"), dict):
        data["link"]["options"] = {}
    return data, warnings


class Config:
    """配置容器，支持点号读写与文件持久化。"""

    def __init__(self, data: dict | None = None, path: Path | None = None) -> None:
        self.path = Path(path) if path else None
        self.warnings: list[str] = []
        self.data, self.warnings = normalize(data or {})

    # ------------------------------------------------------------------ 读写
    def get(self, dotted: str, default: Any = None) -> Any:
        node: Any = self.data
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def set(self, dotted: str, value: Any) -> None:
        parts = dotted.split(".")
        node = self.data
        for part in parts[:-1]:
            if not isinstance(node.get(part), dict):
                node[part] = {}
            node = node[part]
        node[parts[-1]] = value

    def update(self, patch: dict) -> list[str]:
        """局部更新并重新收敛；返回警告。"""
        self.data, warnings = normalize(deep_merge(self.data, patch))
        self.warnings = warnings
        return warnings

    def replace(self, data: dict) -> list[str]:
        self.data, warnings = normalize(data)
        self.warnings = warnings
        return warnings

    def to_dict(self) -> dict:
        return copy.deepcopy(self.data)

    def public(self) -> dict:
        """可下发给前端的配置（当前无敏感字段，保留接口）。"""
        return self.to_dict()

    # ------------------------------------------------------------------ 持久化
    @classmethod
    def load(cls, path: str | Path | None) -> "Config":
        if not path:
            return cls()
        file_path = Path(path)
        if not file_path.exists():
            return cls(path=file_path)
        raw = json.loads(file_path.read_text(encoding="utf-8"))
        return cls(raw, path=file_path)

    def save(self, path: str | Path | None = None) -> Path:
        target = Path(path) if path else self.path
        if target is None:
            raise ValueError("未指定配置文件路径")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")
        self.path = target
        return target
