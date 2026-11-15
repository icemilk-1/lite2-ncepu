"""传输适配器注册表：按链路类型创建实机/仿真连接。"""
from __future__ import annotations

from typing import Iterable

from .base import LEVELS, NullTransport, Transport, TransportCallbacks, TransportError

#: 前端可选的链路类型 -> 展示名
KINDS: dict[str, str] = {
    "mock": "离线演示",
    "udp": "厂商 UDP",
    "ros": "ROS1 话题",
    "ssh_ros": "机载 ROS · SSH",
}


def _load(kind: str):
    if kind == "ssh_ros":
        from .ssh_ros import SshRosTransport

        return SshRosTransport
    if kind == "mock":
        from .mock import MockTransport

        return MockTransport
    if kind in ("udp", "vendor", "vendor_udp"):
        from .vendor_udp import VendorUdpTransport

        return VendorUdpTransport
    if kind in ("ros", "ros1"):
        from .ros1 import Ros1Transport

        return Ros1Transport
    raise ValueError(f"未知链路类型: {kind!r}，可选: {', '.join(KINDS)}")


def create_transport(kind: str, options: dict, callbacks: TransportCallbacks) -> Transport:
    """按类型创建传输对象（不启动）。"""
    klass = _load(str(kind).strip().lower())
    transport = klass(options, callbacks)
    transport.default_options = dict(getattr(klass, "default_options", {}))
    return transport


def available_kinds() -> Iterable[dict]:
    for kind, label in KINDS.items():
        try:
            klass = _load(kind)
            options = dict(getattr(klass, "default_options", {}))
            available = getattr(klass, "available", None)
            yield {
                "kind": kind,
                "label": label,
                "options": options,
                "available": bool(available()) if callable(available) else True,
            }
        except Exception as error:  # pragma: no cover - 仅在依赖缺失时触发
            yield {"kind": kind, "label": label, "options": {}, "available": False,
                   "reason": str(error)}


__all__ = [
    "KINDS",
    "LEVELS",
    "NullTransport",
    "Transport",
    "TransportCallbacks",
    "TransportError",
    "available_kinds",
    "create_transport",
]
