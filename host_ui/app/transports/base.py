"""传输层接口约定。

后端控制循环每 40 ms 调用一次 ``send_velocity``（即使停车也调用，写入 0），
传输实现内部必须再有一层看门狗：超过 ``zero_timeout_s`` 没有收到新指令时
自己补发零速，防止上层进程卡死导致机器狗继续前进。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from ..types import Frame, GridMap

#: 事件级别
LEVELS = ("info", "warn", "error")


@dataclass
class TransportCallbacks:
    """传输层把收到的东西通过回调交给后端（后端负责加锁）。"""

    on_frame: Callable[[Frame], None]
    #: (level, code, message)，code 为稳定的英文短标识，message 面向操作者
    on_event: Callable[[str, str, str], None]
    on_map: Optional[Callable[[GridMap], None]] = None
    #: 激光/点云在里程计坐标系下的二维点集 [(x, y), ...]
    on_scan: Optional[Callable[[list[tuple[float, float]]], None]] = None


class TransportError(RuntimeError):
    """传输层无法建立连接或协议校验失败。"""


class Transport:
    """所有实机/仿真链路的统一接口。"""

    kind = "base"
    label = "传输基类"
    #: 前端连接面板展示的默认参数
    default_options: dict = {}
    #: 本链路支持的高级能力（前端据此置灰按钮）
    capabilities: dict = {
        "arm": False,          # 使能/解除（机载 execution guard）
        "posture": False,      # 站立/趴下
        "vendor_estop": False, # 厂商硬件急停指令 0x21010C0E
        "sensors": False,      # 相机/深度/雷达回传
        "reconnect": False,    # 断线自动重连
    }

    def __init__(self, options: dict | None, callbacks: TransportCallbacks) -> None:
        self.options = dict(options or {})
        self.callbacks = callbacks

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> None:
        """建立连接并启动后台线程；失败抛 TransportError。"""
        raise NotImplementedError

    def stop(self) -> None:
        """停线程、关套接字，并尽可能补发零速。可重复调用。"""

    # ------------------------------------------------------------------ 数据面
    def send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        """下发速度指令（已由安全层限幅）。必须是非阻塞的。"""
        raise NotImplementedError

    # ------------------------------------------------------------------ 能力面（默认不支持，子类按需覆盖）
    def arm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        """请求机载执行层进入自动模式（armed + mode_confirmed）。"""
        return False, "当前链路不支持使能"

    def disarm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        """退出自动模式并请求零速。"""
        return False, "当前链路不支持解除"

    def set_posture(self, action: str, cancel=None,
                    timeout_s: float = 10.0) -> tuple[bool, str]:
        """站立/趴下。``action`` ∈ {"stand","lie"}；``cancel`` 为 threading.Event。"""
        return False, "当前链路不支持姿态切换"

    def emergency_stop(self) -> tuple[bool, str]:
        """发送厂商硬件急停指令（0x21010C0E）或链路等价手段。"""
        return False, "当前链路不支持硬件急停"

    def status(self) -> dict:
        """连接状态快照，供 /api/status 与前端状态灯使用。"""
        return {"kind": self.kind, "label": self.label, "connected": False}

    def options_schema(self) -> dict:
        """可用参数及说明（前端连接面板据此渲染）。"""
        return dict(self.default_options)


class NullTransport(Transport):
    """无链路时的占位实现，保证控制循环与监控逻辑照常运行。"""

    kind = "offline"
    label = "未连接"

    def start(self) -> None:
        self.callbacks.on_event("info", "link_offline", "未连接实机，当前无状态输入")

    def send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        return

    def status(self) -> dict:
        return {"kind": self.kind, "label": self.label, "connected": False,
                "detail": "未连接"}