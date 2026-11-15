"""SafetyCore 状态矩阵与姿态锁存的回归测试（无网络、无实机）。"""
from __future__ import annotations

import time

import pytest

from host_ui.app.state import SafetyCore, transform_pose
from host_ui.app.types import Frame


LIMITS = {
    "max_forward_m_s": 0.30,
    "max_reverse_m_s": 0.08,
    "max_yaw_rad_s": 0.80,
    "max_accel_m_s2": 10.0,
    "max_yaw_accel_rad_s2": 20.0,
    "teleop_linear_m_s": 0.10,
    "teleop_angular_rad_s": 0.35,
}
SAFETY = {
    "command_timeout_s": 0.35,
    "state_timeout_s": 0.80,
    "ready_basic": 6,
    "joystick_threshold": 0.03,
    "takeover_hold_s": 0.50,
    "estop_reset_hold_s": 1.20,
}


def frame(basic: int = 6, joystick=(0.0, 0.0, 0.0), stamp: float | None = None) -> Frame:
    return Frame(source="test", stamp=stamp if stamp is not None else time.monotonic(),
                 position=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
                 velocity_body=(0.0, 0.0, 0.0), basic=basic, gait=0, motion=0,
                 joystick=joystick)


def make_core() -> SafetyCore:
    core = SafetyCore(LIMITS, SAFETY)
    now = time.monotonic()
    core.set_teleop(0.10, 0.20, True, now=now)
    return core


def test_teleop_passes_when_ready() -> None:
    core = make_core()
    decision = core.tick(time.monotonic(), frame(), link_up=True)
    assert decision.source == "teleop"
    assert decision.zero_reason == ""
    assert decision.linear_m_s == pytest.approx(0.10, abs=0.02)
    assert decision.angular_rad_s == pytest.approx(0.20, abs=0.03)


def test_estop_blocks_all_motion() -> None:
    core = make_core()
    core.engage_estop("测试")
    decision = core.tick(time.monotonic(), frame(), link_up=True)
    assert (decision.linear_m_s, decision.angular_rad_s) == (0.0, 0.0)
    assert decision.zero_reason == "estop"
    assert decision.flags.get("estop") is True


def test_deadman_required() -> None:
    core = SafetyCore(LIMITS, SAFETY)
    core.set_teleop(0.10, 0.0, False)
    decision = core.tick(time.monotonic(), frame(), link_up=True)
    assert decision.linear_m_s == 0.0
    assert decision.zero_reason == "deadman"


def test_state_timeout_blocks() -> None:
    core = make_core()
    old = frame(stamp=time.monotonic() - 2.0)
    decision = core.tick(time.monotonic(), old, link_up=True)
    assert decision.zero_reason == "state_timeout"
    assert decision.linear_m_s == 0.0


def test_link_down_blocks() -> None:
    core = make_core()
    decision = core.tick(time.monotonic(), frame(), link_up=False)
    assert decision.zero_reason == "link_down"


def test_mode_not_ready_blocks() -> None:
    core = make_core()
    decision = core.tick(time.monotonic(), frame(basic=1), link_up=True)
    assert decision.zero_reason == "mode_not_ready"
    assert decision.flags.get("basic") == 1


def test_joystick_takeover_with_hold() -> None:
    core = make_core()
    now = time.monotonic()
    decision = core.tick(now, frame(joystick=(0.8, 0.0, 0.0)), link_up=True)
    assert decision.zero_reason == "joystick_takeover"
    # 摇杆回中后仍保持 takeover_hold_s
    decision = core.tick(now + 0.2, frame(joystick=(0.0, 0.0, 0.0)), link_up=True)
    assert decision.zero_reason == "joystick_takeover"
    decision = core.tick(now + 0.8, frame(joystick=(0.0, 0.0, 0.0)), link_up=True)
    assert decision.zero_reason != "joystick_takeover"


def test_clamp_and_flags() -> None:
    core = make_core()
    core.set_teleop(5.0, 9.0, True)
    decision = core.tick(time.monotonic(), frame(), link_up=True)
    assert decision.linear_m_s <= LIMITS["max_forward_m_s"] + 1e-6
    assert abs(decision.angular_rad_s) <= LIMITS["max_yaw_rad_s"] + 1e-6
    assert decision.flags.get("clamped") is True


def test_acceleration_limit() -> None:
    core = make_core()
    now = time.monotonic()
    first = core.tick(now, frame(), link_up=True)
    second = core.tick(now + 0.04, frame(), link_up=True)
    assert second.linear_m_s < first.linear_m_s + 0.05


def test_command_timeout_blocks() -> None:
    core = make_core()
    now = time.monotonic()
    core.tick(now, frame(), link_up=True)
    stale = now + 2.0
    decision = core.tick(stale, frame(stamp=stale), link_up=True)
    assert decision.zero_reason == "no_input"
    assert decision.linear_m_s == 0.0


def test_posture_lock_blocks_motion() -> None:
    core = make_core()
    core.begin_posture("stand")
    decision = core.tick(time.monotonic(), frame(), link_up=True)
    assert decision.zero_reason == "posture"
    assert decision.flags.get("posture") == "stand"
    assert decision.linear_m_s == 0.0
    core.end_posture()
    decision = core.tick(time.monotonic(), frame(), link_up=True)
    assert decision.zero_reason != "posture"


def test_snapshot_contains_posture_and_flags() -> None:
    core = make_core()
    core.begin_posture("lie")
    core.tick(time.monotonic(), frame(), link_up=True)
    snapshot = core.snapshot()
    assert snapshot["posture"]["active"] is True
    assert snapshot["posture"]["action"] == "lie"
    assert "flags" in snapshot["output"]


def test_transform_pose_alignment() -> None:
    x, y, yaw = transform_pose(1.0, 0.0, 0.0, {"x": 0.5, "y": -0.5, "yaw_deg": 90.0})
    assert x == pytest.approx(0.5, abs=1e-6)
    assert y == pytest.approx(0.5, abs=1e-6)
    assert yaw == pytest.approx(90.0, abs=1e-6)


def test_trail_odometer_ignores_small_jitter() -> None:
    from host_ui.app.state import TrailTracker

    tracker = TrailTracker(min_spacing_m=0.03)
    for index in range(200):
        # 原地 ±5 mm 抖动（模拟状态噪声），不应累计成里程
        tracker.update((0.005 if index % 2 else -0.005), 0.0)
    assert tracker.odometer_m == pytest.approx(0.0, abs=1e-9)
    tracker.update(0.1, 0.0)
    assert tracker.odometer_m == pytest.approx(0.1, abs=0.02)