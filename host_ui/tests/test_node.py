"""HostNode 的使能/姿态/急停/操作权状态机测试（假链路，无网络）。"""
from __future__ import annotations

import time
import threading
from dataclasses import replace

import pytest

from host_ui.app.config import Config
from host_ui.app.explore import ExplorationPlanner
from host_ui.app.node import HostNode
from host_ui.app.transports.base import Transport
from host_ui.app.types import Frame, GridMap


class FakeTransport(Transport):
    kind = "fake"
    label = "假链路"
    default_options: dict = {}
    capabilities = {"arm": True, "posture": True, "vendor_estop": True,
                    "sensors": False, "reconnect": True}

    def __init__(self) -> None:
        super().__init__({}, None)  # type: ignore[arg-type]
        self.connected = True
        self.armed = False
        self.last_command = (0.0, 0.0)
        self.posture_calls: list[str] = []
        self.estop_calls = 0
        self.arm_calls = 0

    def start(self) -> None:
        self.connected = True

    def send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.last_command = (linear_m_s, angular_rad_s)

    def status(self) -> dict:
        return {
            "kind": self.kind, "label": self.label, "connected": self.connected,
            "control": {"armed": self.armed, "mode_confirmed": self.armed,
                        "pending": None, "depth_age_s": 0.1, "last_error": ""},
            "execution": {"reason": "enabled" if self.armed else "disarmed"},
            "reconnect": {"enabled": True, "attempts": 0, "next_retry_in_s": None},
        }

    def arm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        self.arm_calls += 1
        self.armed = True
        return True, "已进入自动模式"

    def disarm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        self.armed = False
        return True, "已退出自动模式"

    def set_posture(self, action: str, cancel=None,
                    timeout_s: float = 10.0) -> tuple[bool, str]:
        self.posture_calls.append(action)
        for _ in range(10):
            if cancel is not None and cancel.is_set():
                return False, "姿态切换已取消"
            time.sleep(0.01)
        return True, "已确认" + ("站立" if action == "stand" else "趴下")

    def emergency_stop(self) -> tuple[bool, str]:
        self.estop_calls += 1
        return True, "厂商急停 0x21010C0E 已发送"


def make_node() -> tuple[HostNode, FakeTransport]:
    config = Config({"record": {"enabled": False}, "map": {"demo": True}})
    node = HostNode(config)
    transport = FakeTransport()
    node.transport = transport
    node._on_frame(Frame(source="fake", stamp=time.monotonic(),
                         position=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
                         velocity_body=(0.0, 0.0, 0.0), basic=6))
    node.tick()
    return node, transport


def test_arm_requires_ready_mode() -> None:
    node, transport = make_node()
    ok, message = node.arm()
    assert ok is True and transport.armed is True, message
    assert transport.arm_calls == 1


def test_arm_blocked_when_not_standing() -> None:
    node, transport = make_node()
    node._on_frame(Frame(source="fake", stamp=time.monotonic(),
                         position=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
                         velocity_body=(0.0, 0.0, 0.0), basic=1))
    ok, message = node.arm()
    assert ok is False
    assert "basic=1" in message
    assert transport.arm_calls == 0


def test_posture_locks_motion_and_completes() -> None:
    node, transport = make_node()
    ok, message = node.posture("lie")
    assert ok is True, message
    # 姿态进行中：速度必须为零
    decision = node.tick()
    assert decision["decision"].zero_reason == "posture"
    assert transport.last_command == (0.0, 0.0)
    if node._posture_thread is not None:
        node._posture_thread.join(timeout=3)
    assert node.posture_state["state"] == "ok"
    assert transport.posture_calls == ["lie"]


@pytest.mark.parametrize("posture_config", [{}, {"timeout_s": "bad"}, {"timeout_s": None}])
def test_first_posture_has_budget_for_an_eighteen_second_confirmation(posture_config):
    node, transport = make_node()
    node.config = Config({"posture": posture_config, "record": {"enabled": False}})
    received_timeouts = []

    def slow_first_action(action, cancel=None, timeout_s=0):
        # A deterministic transport models a first action taking eighteen seconds.
        # HostNode must preserve its posture lock until that confirmation returns.
        received_timeouts.append(timeout_s)
        assert node.safety.posture_active
        node.tick()
        assert transport.last_command == (0.0, 0.0)
        if timeout_s < 18:
            return False, "first posture still preparing when old timeout expired"
        return True, "first posture confirmed at 18 s"

    transport.set_posture = slow_first_action
    assert node.posture("lie")[0]
    node._posture_thread.join(timeout=2)
    assert not node._posture_thread.is_alive()
    assert node.posture_state["state"] == "ok", node.posture_state
    assert received_timeouts == [20.0]
    assert not node.safety.posture_active


def test_posture_cancel() -> None:
    node, transport = make_node()
    ok, _ = node.posture("lie")
    assert ok is True
    ok, message = node.cancel_posture()
    assert ok is True
    if node._posture_thread is not None:
        node._posture_thread.join(timeout=3)
    assert node.posture_state["state"] == "cancelled"


def test_posture_requires_correct_current_mode() -> None:
    node, _ = make_node()
    ok, message = node.posture("stand")  # 当前 basic=6，不能起立
    assert ok is False
    assert "basic=6" in message


def test_posture_rejects_repeated_click_while_pending() -> None:
    node, transport = make_node()
    ok, _ = node.posture("lie")
    assert ok is True
    ok, message = node.posture("lie")
    assert ok is False
    assert "正在进行" in message
    if node._posture_thread is not None:
        node._posture_thread.join(timeout=3)
    assert transport.posture_calls == ["lie"]


def test_estop_triggers_vendor_estop_and_blocks() -> None:
    node, transport = make_node()
    ok, message = node.engage_estop("测试急停")
    assert ok is True
    assert node.safety.estop is True
    assert transport.estop_calls == 1
    assert "0x21010C0E" in message
    decision = node.tick()
    assert decision["decision"].zero_reason == "estop"


def test_estop_reset_requires_hold() -> None:
    node, _ = make_node()
    node.engage_estop("测试")
    ok, message = node.reset_estop(0.2)
    assert ok is False and "1.2" in message
    ok, message = node.reset_estop(1.3)
    assert ok is True
    assert node.safety.estop is False


@pytest.mark.parametrize("cancel_via_estop", [False, True])
def test_cancel_keeps_motion_locked_until_posture_worker_finishes(cancel_via_estop):
    node, transport = make_node()
    entered, release = threading.Event(), threading.Event()

    def delayed_posture(action, cancel=None, timeout_s=10):
        entered.set()
        assert release.wait(2)
        return False, "机载姿态请求结束"

    transport.set_posture = delayed_posture
    try:
        assert node.posture("lie")[0]
        assert entered.wait(1)
        if cancel_via_estop:
            node.engage_estop()
            node.reset_estop(2)
        else:
            node.cancel_posture()
        assert node.safety.posture_active
        node.set_teleop(.05, 0, True)
        decision = node.tick()["decision"]
        assert decision.linear_m_s == 0
        assert decision.zero_reason == "posture"
    finally:
        release.set()
        node._posture_thread.join(2)
    assert not node.safety.posture_active
    assert node.posture_state["state"] == "cancelled"


def test_operator_claim_release_and_stale() -> None:
    node, _ = make_node()
    node.set_operator(7)
    assert node.operator_snapshot()["id"] == 7
    assert node._operator_stale(time.monotonic()) is False
    node.last_operator_activity = time.monotonic() - 10.0
    assert node._operator_stale(time.monotonic()) is True
    ok, _ = node.release_operator(8)
    assert ok is False
    ok, _ = node.release_operator(7)
    assert ok is True
    assert node.operator_snapshot()["id"] is None


def test_battery_percent_normalization() -> None:
    node, _ = make_node()
    node._on_frame(Frame(source="fake", stamp=time.monotonic(),
                         position=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
                         velocity_body=(0.0, 0.0, 0.0), basic=6,
                         raw={"battery_level_raw": 0.67}))
    assert node.snapshot()["battery"]["percent"] == pytest.approx(67.0, abs=0.1)
    node._on_frame(Frame(source="fake", stamp=time.monotonic(),
                         position=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
                         velocity_body=(0.0, 0.0, 0.0), basic=6,
                         raw={"battery_level_raw": 59.0}))
    assert node.snapshot()["battery"]["percent"] == pytest.approx(59.0, abs=0.1)


# --------------------------------------------------------------------------- 自主探索
def make_exploring_node() -> tuple[HostNode, FakeTransport]:
    """使能后、可开始探索的节点（FakeTransport 不声明 mapping 能力）。"""
    node, transport = make_node()
    transport.armed = True
    node.set_operator(7)
    node.tick()
    return node, transport


def exploration_step(node, transport, now, pose=None):
    """Advance only the exploration state machine with an explicit operator clock."""
    node.last_operator_activity = now
    node._exploration_tick(now, node.map_pose() if pose is None else pose,
                           transport.connected, transport.status())


def test_exploration_requires_arming() -> None:
    node, transport = make_node()
    transport.armed = False
    ok, message = node.start_exploration()
    assert ok is False
    assert "使能" in message
    assert node.exploration_state["state"] == "idle"


def test_exploration_requires_map_and_link() -> None:
    node, transport = make_exploring_node()
    transport.connected = False
    ok, message = node.start_exploration()
    assert ok is False and "链路" in message
    transport.connected = True
    node.map = None
    ok, message = node.start_exploration()
    assert ok is False and "地图" in message


def test_exploration_blocked_by_estop() -> None:
    node, _ = make_exploring_node()
    node.engage_estop("测试")
    ok, message = node.start_exploration()
    assert ok is False and "急停" in message


def test_exploration_picks_goal_then_waits_for_observation(monkeypatch) -> None:
    node, transport = make_exploring_node()
    submitted: list[tuple[float, float]] = []

    def fake_next_goal(grid, pose, now):
        return None if submitted else (1.0, 2.0)

    def fake_submit(x, y, yaw, pose=None):
        submitted.append((x, y))
        return True, "已规划 3 个路径点"

    monkeypatch.setattr(node.explorer, "next_goal", fake_next_goal)
    monkeypatch.setattr(node, "_submit_goal", fake_submit)
    ok, _ = node.start_exploration()
    assert ok is True
    now = time.monotonic()
    exploration_step(node, transport, now)
    assert submitted == [(1.0, 2.0)]
    assert node.exploration_state["state"] == "active"
    assert node.exploration_state["last_goal"] == [1.0, 2.0]
    # 真正移到目标之后先停稳观测，不能在同一拍重复选点或宣告完成。
    node._on_frame(replace(node.frame, position=(1.0, 2.0, 0.0)))
    node.nav.finish("arrived", "已到达目标点")
    exploration_step(node, transport, now + 1, (1.0, 2.0, 0.0))
    assert node.exploration_state["goals_visited"] == 1
    assert node.exploration_state["state"] == "active"
    assert submitted == [(1.0, 2.0)]
    exploration_step(node, transport, now + 2, (1.0, 2.0, 0.0))
    assert submitted == [(1.0, 2.0)]


def test_exploration_blacklists_failed_goal(monkeypatch) -> None:
    node, _ = make_exploring_node()
    submitted: list[tuple[float, float]] = []

    def fake_submit(x, y, yaw, pose=None):
        submitted.append((float(x), float(y)))
        return True, "ok"

    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: (3.0, 4.0))
    monkeypatch.setattr(node, "_submit_goal", fake_submit)
    node.start_exploration()
    node.tick()
    assert len(submitted) == 1
    node.nav.finish("stuck", "卡死")
    node._exploration_retry_at = 0
    node.tick()
    # 失败目标被拉黑；下次选点仍保留连续失败计数。
    assert node.explorer.stats()["blacklisted"] == 1
    assert len(submitted) == 2
    assert node.exploration_state["consecutive_failures"] == 1
    assert node.exploration_state["state"] == "active"


def test_exploration_pauses_when_failed_target_leaves_no_candidate(monkeypatch) -> None:
    node, _ = make_exploring_node()
    targets = [(3.0, 4.0), None]

    def fake_next(grid, pose, now):
        return targets.pop(0) if targets else None

    monkeypatch.setattr(node.explorer, "next_goal", fake_next)
    stats = node.explorer.stats
    monkeypatch.setattr(node.explorer, "stats", lambda: dict(
        stats(), decision_reason="no_candidates", raw_frontiers=1, frontiers=1))
    monkeypatch.setattr(node, "_submit_goal", lambda x, y, yaw, pose=None: (True, "ok"))
    node.start_exploration()
    node.tick()
    node.nav.finish("blocked", "受阻")
    node._exploration_retry_at = 0
    node.tick()
    # 拉黑后仍有未知边界：暂停，不把不可达解释成探索完成。
    assert node.explorer.stats()["blacklisted"] == 1
    assert node.exploration_state["state"] == "paused"
    assert node.exploration_state["consecutive_failures"] == 1


def test_exploration_fails_after_consecutive_failures(monkeypatch) -> None:
    node, transport = make_exploring_node()
    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: (3.0, 4.0))
    monkeypatch.setattr(node, "_submit_goal", lambda x, y, yaw, pose=None: (False, "A* 未找到"))
    node.start_exploration()
    limit = int(node.config.get("explore.max_consecutive_failures", 3))
    now = time.monotonic()
    for _ in range(limit + 2):
        exploration_step(node, transport, now)
        now += 1.01
    assert node.exploration_state["state"] == "failed"
    assert "失败" in node.exploration_state["message"]


def test_exploration_pauses_on_manual_teleop_and_goal(monkeypatch) -> None:
    node, _ = make_exploring_node()
    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: (1.0, 1.0))
    monkeypatch.setattr(node, "_submit_goal", lambda x, y, yaw, pose=None: (True, "ok"))
    node.start_exploration()
    node.tick()
    assert node.exploration_state["state"] == "active"
    node.set_teleop(0.05, 0.0, True)
    assert node.exploration_state["state"] == "paused"
    assert "手动接管" in node.exploration_state["message"]


def test_exploration_paused_by_manual_goal(monkeypatch) -> None:
    node, _ = make_exploring_node()
    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: (1.0, 1.0))
    monkeypatch.setattr(node, "_submit_goal", lambda x, y, yaw, pose=None: (True, "ok"))
    node.start_exploration()
    node.tick()
    node.set_goal(0.5, 0.5)
    assert node.exploration_state["state"] == "paused"
    assert "手动目标" in node.exploration_state["message"]


def test_exploration_paused_resume_and_stop(monkeypatch) -> None:
    node, _ = make_exploring_node()
    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: (1.0, 1.0))
    monkeypatch.setattr(node, "_submit_goal", lambda x, y, yaw, pose=None: (True, "ok"))
    node.start_exploration()
    node.set_teleop(0.05, 0.0, True)
    assert node.exploration_state["state"] == "paused"
    node.set_teleop(0.0, 0.0, False)
    ok, message = node.start_exploration()
    assert ok is True and "继续" in message
    assert node.exploration_state["state"] == "active"
    ok, _ = node.stop_exploration("测试停止")
    assert ok is True
    assert node.exploration_state["state"] == "cancelled"
    assert node.nav.state not in ("following", "aligning")


def test_exploration_cancelled_by_estop_and_stop_all(monkeypatch) -> None:
    node, _ = make_exploring_node()
    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: (1.0, 1.0))
    monkeypatch.setattr(node, "_submit_goal", lambda x, y, yaw, pose=None: (True, "ok"))
    node.start_exploration()
    node.engage_estop("探索中急停")
    node.tick()
    assert node.exploration_state["state"] == "cancelled"
    assert "急停" in node.exploration_state["message"] or "停车" in node.exploration_state["message"]


def test_exploration_snapshot_fields() -> None:
    node, _ = make_exploring_node()
    payload = node.snapshot()["exploration"]
    for key in ("state", "message", "since", "frontiers", "goals_visited",
                "blacklisted", "consecutive_failures", "last_goal",
                "frontiers_preview"):
        assert key in payload, key
    assert payload["state"] == "idle"
    assert isinstance(payload["frontiers_preview"], list)


def test_exploration_disabled_by_config() -> None:
    config = Config({"record": {"enabled": False}, "map": {"demo": True},
                     "explore": {"enabled": False}})
    node = HostNode(config)
    transport = FakeTransport()
    transport.armed = True
    node.transport = transport
    node._on_frame(Frame(source="fake", stamp=time.monotonic(),
                         position=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
                         velocity_body=(0.0, 0.0, 0.0), basic=6))
    ok, message = node.start_exploration()
    assert ok is False and "禁用" in message


def test_exploration_requires_mapping_running_for_mapping_link(monkeypatch) -> None:
    node, transport = make_exploring_node()
    transport.capabilities = dict(transport.capabilities, mapping=True)
    original_status = transport.status

    def status_with_mapping():
        payload = original_status()
        payload["mapping"] = {"state": "idle"}
        payload["localization"] = {"valid": False}
        return payload

    monkeypatch.setattr(transport, "status", status_with_mapping)
    ok, message = node.start_exploration()
    assert ok is False and "建图" in message
    # 建图运行后允许开始探索
    def status_running():
        payload = original_status()
        payload["mapping"] = {"state": "running"}
        payload["localization"] = {"valid": True, "frame": "map"}
        return payload

    monkeypatch.setattr(transport, "status", status_running)
    monkeypatch.setattr(node.explorer, "next_goal", lambda grid, pose, now: None)
    ok, message = node.start_exploration()
    assert ok is True, message


def exploration_patch():
    """A known free patch with one real unknown boundary."""
    grid = GridMap(80, 60, 0.05)
    for y in range(10, 50):
        for x in range(10, 70):
            grid.set(x, y, 0)
    return grid


def test_physical_joystick_takeover_requires_explicit_exploration_resume():
    node, transport = make_exploring_node()
    node.map = exploration_patch()
    center = node.map.cell_center(39, 29)
    node._on_frame(replace(node.frame, position=(*center, 0.0)))
    assert node.start_exploration()[0]
    node.tick()
    assert node.nav.state == "following"

    node._on_frame(replace(node.frame, stamp=time.monotonic(), joystick=(0.5, 0, 0)))
    node.tick()
    assert node.exploration_state["state"] == "paused"
    assert node.nav.state not in ("following", "aligning")
    assert transport.last_command == (0.0, 0.0)

    # Even after joystick release and the safety hold, no autonomous task resumes.
    later = time.monotonic() + float(node.config.get("safety.takeover_hold_s")) + 0.1
    node.last_operator_activity = later
    node._on_frame(replace(node.frame, stamp=later, joystick=(0, 0, 0)))
    node.tick(now=later)
    assert node.exploration_state["state"] == "paused"
    assert node.nav.state not in ("following", "aligning")
    assert transport.last_command == (0.0, 0.0)
    assert node.start_exploration()[0]
    assert node.exploration_state["state"] == "active"


def test_existing_frontier_without_safe_candidate_pauses_instead_of_completing():
    node, transport = make_exploring_node()
    node.map = exploration_patch()
    node.explorer = ExplorationPlanner({"max_backoff_m": 0.05})
    center = node.map.cell_center(39, 29)
    node._on_frame(replace(node.frame, position=(*center, 0.0)))
    assert node.start_exploration()[0]
    exploration_step(node, transport, time.monotonic(), (*center, 0.0))
    assert node.explorer.stats()["raw_frontiers"] > 0
    assert node.explorer.stats()["decision_reason"] == "no_candidates"
    assert node.exploration_state["state"] == "paused"
    assert node.nav.state not in ("following", "aligning")


def test_no_frontiers_completes_only_after_stable_map_observation():
    node, transport = make_exploring_node()
    node.map = GridMap(60, 60, 0.05, data=bytes(60 * 60))
    assert node.start_exploration()[0]
    now = time.monotonic()
    exploration_step(node, transport, now)
    assert node.exploration_state["state"] == "active"

    # A new observation restarts stability timing even when it adds no frontier.
    node.map.set(50, 50, 100)
    exploration_step(node, transport, now + 1.01)
    wait = float(node.config.get("explore.observation_wait_s"))
    exploration_step(node, transport, now + wait + 0.1)
    assert node.exploration_state["state"] == "active"
    exploration_step(node, transport, now + wait + 1.12)
    assert node.exploration_state["state"] == "completed"
    assert node.exploration_state["goals_visited"] == 0


def test_unchanged_map_does_not_repeat_arrival_and_eventually_pauses():
    node, transport = make_exploring_node()
    node.map = exploration_patch()
    center = node.map.cell_center(39, 29)
    node._on_frame(replace(node.frame, position=(*center, 0.0)))
    assert node.start_exploration()[0]
    now = time.monotonic()
    exploration_step(node, transport, now, (*center, 0.0))
    goal = tuple(node.exploration_state["last_goal"])
    pose = (*goal, 0.0)
    node.nav.update(now + 0.1, pose, node.map)
    assert node.nav.state == "arrived"
    exploration_step(node, transport, now + 0.2, pose)
    assert node.exploration_state["goals_visited"] == 1

    timeout = float(node.config.get("explore.observation_timeout_s"))
    for step in range(1, int(timeout) + 7):
        exploration_step(node, transport, now + step * 1.01, pose)
    assert node.exploration_state["state"] == "paused"
    assert node.exploration_state["goals_visited"] == 1
    assert node.nav.state not in ("following", "aligning")


def test_map_noise_cannot_count_repeated_stationary_arrivals_as_progress():
    node, transport = make_exploring_node()
    node.map = exploration_patch()
    center = node.map.cell_center(39, 29)
    node._on_frame(replace(node.frame, position=(*center, 0.0)))
    assert node.start_exploration()[0]
    now = time.monotonic()
    exploration_step(node, transport, now, (*center, 0.0))
    goal = tuple(node.exploration_state["last_goal"])
    pose = (*goal, 0.0)
    for index in range(3):
        node.nav.update(now + 0.1, pose, node.map)
        assert node.nav.state == "arrived"
        exploration_step(node, transport, now + 0.2, pose)
        assert node.exploration_state["goals_visited"] == 1
        if index == 2:
            break
        # Remote cell noise changes the content signature without revealing space
        # around this frontier. Re-evaluation must not count stationary success.
        node.map.set(15, 15, 100 if index == 0 else 0)
        now += float(node.config.get("explore.observation_wait_s")) + 0.3
        exploration_step(node, transport, now, pose)
        assert node.nav.state == "following"
    assert node.exploration_state["state"] == "paused"
    assert node.nav.state not in ("following", "aligning")
