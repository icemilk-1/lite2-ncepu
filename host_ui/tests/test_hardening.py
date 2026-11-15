"""地图预编码缓存、SSH 主机密钥策略等加固项测试。"""
from __future__ import annotations

from host_ui.app.config import Config
from host_ui.app.mapstore import demo_map
from host_ui.app.node import HostNode
from host_ui.app.transports.ssh_ros import host_key_mode


def make_node() -> HostNode:
    config = Config({"record": {"enabled": False},
                     "map": {"demo": True}})
    return HostNode(config)


def test_requested_speed_limits_survive_normalization_and_reload(tmp_path) -> None:
    config = Config({"limits": {"max_forward_m_s": 1.0, "max_reverse_m_s": 1.0,
                                 "max_yaw_rad_s": 1.0, "teleop_linear_m_s": 0.5,
                                 "teleop_angular_rad_s": 0.5},
                     "nav": {"max_speed_m_s": 0.5}})
    config.path = tmp_path / "limits.json"
    config.save()
    reloaded = Config.load(config.path)
    assert reloaded.get("limits.max_forward_m_s") == 1.0
    assert reloaded.get("limits.max_reverse_m_s") == 1.0
    assert reloaded.get("limits.max_yaw_rad_s") == 1.0
    assert reloaded.get("limits.teleop_linear_m_s") == 0.5
    assert reloaded.get("limits.teleop_angular_rad_s") == 0.5
    assert reloaded.get("nav.max_speed_m_s") == 0.5


def test_map_payload_is_cached_until_reload() -> None:
    node = make_node()
    first = node.map_payload()
    assert first is not None
    assert node.map_payload() is first  # 预编码缓存，广播环不再重复 base64
    message = node.map_message_json()
    assert message.startswith('{"type":"map","data":')
    assert message.endswith("}")

    replaced = demo_map()
    replaced.name = "demo_reload"
    node.load_map(replaced, source="测试重载")
    second = node.map_payload()
    assert second is not first
    assert node.map_message_json() != message
    assert node.map_message_json().startswith('{"type":"map","data":')


def test_ssh_host_key_mode() -> None:
    assert host_key_mode("") == "unverified"
    assert host_key_mode("   ") == "unverified"
    assert host_key_mode("C:/Users/x/.ssh/known_hosts") == "strict"


def test_ssh_default_options_include_security_keys() -> None:
    from host_ui.app.transports.ssh_ros import SshRosTransport

    options = SshRosTransport.default_options
    assert "known_hosts" in options
    assert "source_ip" in options
    assert "ssh_port" in options


def test_bind_source_falls_back_when_ip_stale() -> None:
    import socket

    from host_ui.app.transports.ssh_ros import bind_source

    sock = socket.socket()
    try:
        assert bind_source(sock, "") is True
    finally:
        sock.close()
    sock = socket.socket()
    try:
        # 本机不存在的地址（模拟换网后配置里的旧 IP）
        assert bind_source(sock, "192.0.2.123") is False
    finally:
        sock.close()
