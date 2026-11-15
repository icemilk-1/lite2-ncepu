"""上位机共享数据类型：机器人状态帧与二维占据栅格地图。

约定
----
* 长度单位：米（m）；角度单位：度（deg），航向角 yaw 逆时针为正。
* ``Frame.position`` / ``Frame.rpy_deg`` 位于里程计（odom）坐标系，
  界面上显示的"地图坐标"会再应用一层地图对齐偏移（见 config.map_align）。
* ``GridMap`` 的行序与 ROS ``nav_msgs/OccupancyGrid`` 一致：第 0 行是地图最下方，
  第 0 列是最左方，单元格 (gx, gy) 的中心位于
  ``origin + R(origin_yaw) * ((gx + 0.5) * res, (gy + 0.5) * res)``。
* 栅格取值：``UNKNOWN = -1``、``FREE = 0``、``OCCUPIED = 100``，中间值为灰度占据概率。
"""
from __future__ import annotations

import base64
import math
from dataclasses import dataclass, field
from typing import Any

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

#: 地图数据编码版本，前后端共用。
MAP_SCHEMA = "lite2-grid-v1"


@dataclass(frozen=True)
class Frame:
    """一帧机器人状态反馈（由传输层把各协议统一成该结构）。"""

    source: str
    stamp: float
    position: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    velocity_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_world: tuple[float, float, float] = (0.0, 0.0, 0.0)
    basic: int = 0
    gait: int = 0
    motion: int = 0
    joystick: tuple[float, float, float] = (0.0, 0.0, 0.0)
    goal_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def yaw_deg(self) -> float:
        return float(self.rpy_deg[2])

    def mode_ready(self, ready_basic: int = 6) -> bool:
        """厂商状态 basic==6 表示已站立并进入力控/可运动模式。"""
        return int(self.basic) == int(ready_basic)

    def joystick_active(self, threshold: float = 0.03) -> bool:
        return max(abs(float(v)) for v in self.joystick) >= float(threshold)


class GridMap:
    """二维占据栅格地图（行主序、第 0 行在最下方）。"""

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        data: bytes | bytearray | None = None,
        name: str = "map",
    ) -> None:
        width, height = int(width), int(height)
        if width <= 0 or height <= 0:
            raise ValueError("地图宽高必须为正")
        resolution = float(resolution)
        if not (resolution > 0):
            raise ValueError("地图分辨率必须为正")
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2]))
        self.name = str(name)
        if data is None:
            self.data = bytearray([UNKNOWN & 0xFF]) * (width * height)
        else:
            raw = bytes(data)
            if len(raw) != width * height:
                raise ValueError(
                    f"地图数据长度 {len(raw)} 与 {width}x{height} 不匹配")
            self.data = bytearray(raw)

    # ------------------------------------------------------------------ 基本访问
    @property
    def cells(self) -> int:
        return self.width * self.height

    def at(self, gx: int, gy: int) -> int:
        """返回单元格原始值；越界返回 ``UNKNOWN``。"""
        gx, gy = int(gx), int(gy)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            value = self.data[gy * self.width + gx]
            return value - 256 if value > 127 else value
        return UNKNOWN

    def set(self, gx: int, gy: int, value: int) -> None:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.data[gy * self.width + gx] = value & 0xFF

    def is_occupied(self, gx: int, gy: int, threshold: int = 50) -> bool:
        return self.at(gx, gy) >= threshold

    def is_free(self, gx: int, gy: int, threshold: int = 50) -> bool:
        return 0 <= self.at(gx, gy) < threshold

    # ------------------------------------------------------------------ 坐标变换
    def _rotate(self, x: float, y: float, yaw_deg: float) -> tuple[float, float]:
        yaw = math.radians(yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        return c * x - s * y, s * x + c * y

    def cell_center(self, gx: int, gy: int) -> tuple[float, float]:
        """返回单元格中心在世界坐标系中的位置。"""
        ox, oy, oyaw = self.origin
        lx = (float(gx) + 0.5) * self.resolution
        ly = (float(gy) + 0.5) * self.resolution
        dx, dy = self._rotate(lx, ly, oyaw)
        return ox + dx, oy + dy

    def world_to_cell(self, x: float, y: float) -> tuple[float, float]:
        """返回世界坐标对应的浮点栅格坐标（未取整，可能越界）。"""
        ox, oy, oyaw = self.origin
        dx, dy = float(x) - ox, float(y) - oy
        lx, ly = self._rotate(dx, dy, -oyaw)
        return lx / self.resolution, ly / self.resolution

    def world_to_index(self, x: float, y: float) -> tuple[int, int]:
        gx, gy = self.world_to_cell(x, y)
        return int(math.floor(gx)), int(math.floor(gy))

    def contains_cell(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied_world(self, x: float, y: float, radius: float = 0.0) -> bool:
        """世界坐标是否压到障碍（可选膨胀半径，单位 m）。"""
        gx, gy = self.world_to_index(x, y)
        if not self.contains_cell(gx, gy):
            return True
        pad = int(math.ceil(max(0.0, radius) / self.resolution))
        for yy in range(gy - pad, gy + pad + 1):
            for xx in range(gx - pad, gx + pad + 1):
                if self.is_occupied(xx, yy):
                    return True
        return False

    # ------------------------------------------------------------------ 序列化
    def to_payload(self) -> dict[str, Any]:
        """转成前端/存档使用的 JSON 结构（int8 栅格数据 base64 编码）。"""
        signed = bytes((v & 0xFF) for v in self.data)
        return {
            "schema": MAP_SCHEMA,
            "name": self.name,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": {"x": self.origin[0], "y": self.origin[1], "yaw_deg": self.origin[2]},
            "row_order": "bottom",
            "data": base64.b64encode(signed).decode("ascii"),
        }

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "GridMap":
        if not isinstance(payload, dict):
            raise ValueError("地图数据格式错误")
        schema = payload.get("schema", MAP_SCHEMA)
        if schema != MAP_SCHEMA:
            raise ValueError(f"不支持的地图版本: {schema}")
        raw = base64.b64decode(payload.get("data", ""))
        origin = payload.get("origin") or {}
        return cls(
            width=int(payload["width"]),
            height=int(payload["height"]),
            resolution=float(payload["resolution"]),
            origin=(float(origin.get("x", 0.0)), float(origin.get("y", 0.0)),
                    float(origin.get("yaw_deg", 0.0))),
            data=raw,
            name=str(payload.get("name", "map")),
        )

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": {"x": self.origin[0], "y": self.origin[1], "yaw_deg": self.origin[2]},
        }


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def wrap_deg(angle: float) -> float:
    """把角度归一到 (-180, 180]。"""
    angle = math.fmod(float(angle) + 180.0, 360.0)
    if angle <= 0:
        angle += 360.0
    return angle - 180.0