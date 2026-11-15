"""机载健康数据（关节温度/机载信息）解码与链路参数白名单测试。"""
from __future__ import annotations

import struct

import pytest
from fastapi.testclient import TestClient

from host_ui.app.config import Config
from host_ui.app.protocol import (JOINT_ANGLES, JOINT_TEMPERATURE,
                                  decode_complex, decode_raw_hex)
from host_ui.app.server import create_app, sanitize_link_options


def complex_packet(code: int, payload: bytes) -> bytes:
    return struct.pack("<3i", code, len(payload), 1) + payload


def test_decode_joint_temperature() -> None:
    temperatures = [-3, 38, 41, 44, 39, 42, 40, 37, 45, 43, 36, 35]
    packet = complex_packet(JOINT_TEMPERATURE, struct.pack("<12i", *temperatures))
    report = decode_complex(packet)
    assert report["kind"] == "joint_temperature"
    assert report["temperature_c"] == temperatures
    assert report["max_c"] == 45


def test_decode_joint_angles() -> None:
    angles = [0.1 * index for index in range(12)]
    packet = complex_packet(JOINT_ANGLES, struct.pack("<12d", *angles))
    report = decode_complex(packet)
    assert report["kind"] == "joint_angles"
    assert report["joint_angles"] == pytest.approx(angles)


def test_decode_complex_rejects_bad_frames() -> None:
    assert decode_complex(b"") is None
    assert decode_complex(struct.pack("<3i", JOINT_TEMPERATURE, 48, 0)) is None
    # 声明长度 48 但实际正文只有 40 字节
    truncated = struct.pack("<3i", JOINT_TEMPERATURE, 48, 1) + b"\x00" * 40
    assert decode_complex(truncated) is None
    # NaN 关节角整帧丢弃
    nan_angles = struct.pack("<12d", *([float("nan")] * 12))
    assert decode_complex(complex_packet(JOINT_ANGLES, nan_angles)) is None


def test_decode_raw_hex() -> None:
    temperatures = [20] * 12
    packet = complex_packet(JOINT_TEMPERATURE, struct.pack("<12i", *temperatures))
    report = decode_raw_hex(packet.hex())
    assert report["kind"] == "joint_temperature"
    assert decode_raw_hex("zz") is None
    assert decode_raw_hex("") is None


def test_link_options_whitelist() -> None:
    options, error = sanitize_link_options("mock", {"evil": 1})
    assert options is None and "不支持" in error
    options, error = sanitize_link_options("ssh_ros", {"host": "192.168.1.103",
                                                       "username": "ysc",
                                                       "source_ip": ""})
    assert error == ""
    assert options["host"] == "192.168.1.103"
    options, error = sanitize_link_options("ssh_ros", {"host": "1.2.3.4; rm -rf /"})
    assert options is None
    options, error = sanitize_link_options("udp", {"bind_port": 70000})
    assert options is None and "端口" in error
    options, error = sanitize_link_options("udp", {"robot_ip": "256.1.1.1"})
    assert options is None
    options, error = sanitize_link_options("udp", {"bind_port": 43898,
                                                   "robot_ip": "192.168.1.120"})
    assert error == "" and options["bind_port"] == 43898
    options, error = sanitize_link_options("mock", {"reconnect": {"min_delay_s": 2}})
    assert error == "" and options["reconnect"]["min_delay_s"] == 2.0
    options, error = sanitize_link_options("mock", {"reconnect": {"evil": 1}})
    assert options is None
    options, error = sanitize_link_options("unknown", {})
    assert options is None


def test_api_rejects_unknown_link_option() -> None:
    config = Config({"link": {"kind": "mock"}, "web": {"host": "127.0.0.1"}})
    app = create_app(config, auto_connect=False)
    with TestClient(app) as client:
        response = client.post("/api/link",
                               json={"kind": "mock", "options": {"evil": 1}}).json()
        assert response["ok"] is False
        assert "拒绝" in response["message"]
        # 合法参数仍然可以连接 mock
        response = client.post("/api/link",
                               json={"kind": "mock", "options": {}}).json()
        assert response["ok"] is True