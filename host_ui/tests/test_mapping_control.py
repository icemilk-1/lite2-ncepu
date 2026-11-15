"""地图定位接入、定位失效停车及探索失败预算的离线集成回归。"""
import threading
import time

import pytest

import host_ui.app.node as node_module
from host_ui.app.config import Config
from host_ui.app.mapstore import load_map_file
from host_ui.app.node import HostNode
from host_ui.app.transports.ssh_ros import SshRosTransport


@pytest.fixture
def navigation_clock(monkeypatch):
    """Advance elapsed time without sleeps while preserving normal thread waits."""
    real_monotonic = time.monotonic
    offset = [0.0]
    monkeypatch.setattr(time, "monotonic", lambda: real_monotonic() + offset[0])

    def advance(seconds):
        offset[0] += seconds

    return advance


def refresh_robot(transport):
    """State, odometry and execution can remain alive independently of octomap."""
    transport._receive({"type": "localization", "data": {
        "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "age_s": 0.01,
        "frame": "map", "source": "hdl_graph_slam", "valid": True,
    }})
    transport._receive({"type": "state", "data": {
        "position": [1.0, 2.0, 0.3], "rpy_deg": [0.0, 0.0, 45.0], "basic": 6,
    }})
    transport._receive({"type": "execution", "data": {
        "reason": "enabled", "control": {"armed": True, "mode_confirmed": True},
    }})


@pytest.fixture
def localized_node(navigation_clock):
    node = HostNode(Config({
        "link": {"kind": "ssh_ros"},
        "map": {"demo": False},
        "safety": {"require_operator": False},
    }))
    transport = SshRosTransport({}, node._callbacks)
    node.transport = transport
    # 只注入实际消息解析入口，整个测试不启动 SSH 或 IO 线程。
    transport._started = True
    transport._link_ok = True
    transport._receive({"type": "map", "data": {
        "width": 60, "height": 60, "resolution": 0.1,
        "origin": [-3.0, -3.0, 0.0], "data": [0] * 3600, "name": "map",
        "stamp": 1234.0, "age_s": 0.0,
    }})
    refresh_robot(transport)
    return node, transport


def test_map_pose_uses_map_localization_instead_of_vendor_odometry(localized_node):
    node, _ = localized_node
    assert node.frame.position[:2] == (1.0, 2.0)
    assert node.map_pose() == (0.0, 0.0, 0.0)
    assert node.snapshot()["map_pose"] == {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}


def test_localization_expiry_cancels_active_navigation_with_zero(localized_node):
    node, transport = localized_node
    assert node.set_goal(1.0, 0.0)[0]
    node.tick()
    assert transport.command[0] > 0.0
    with transport.lock:
        transport.localization["received"] = time.monotonic() - 1.0
    assert transport.status()["connected"] is True
    assert transport.status()["localization"]["valid"] is False
    assert node.map_pose() is None
    node.tick()
    assert node.nav.state == "cancelled"
    assert "定位" in node.nav.snapshot().message
    assert transport.command[:2] == (0.0, 0.0)
    assert node.safety.snapshot()["nav_active"] is False


def test_localization_requires_connected_link_and_matching_map_frame(localized_node):
    node, transport = localized_node
    transport._link_ok = False
    assert node.map_pose() is None
    transport._link_ok = True
    transport.localization["frame"] = "another_map"
    assert node.map_pose() is None
    assert not node.set_goal(1.0, 0.0)[0]
    transport.localization["frame"] = "/map"
    assert node.map_pose() == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("field,value", [
    ("x", float("nan")), ("y", float("inf")), ("yaw_deg", "bad"),
    ("age_s", -0.1), ("age_s", None),
])
def test_invalid_localization_is_rejected_without_breaking_status(localized_node, field, value):
    node, transport = localized_node
    payload = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "age_s": 0.0,
               "frame": "map", "valid": True}
    payload[field] = value
    transport._receive({"type": "localization", "data": payload})
    assert transport.status()["localization"]["valid"] is False
    assert node.map_pose() is None


def test_save_map_roundtrip_preserves_cells_and_uses_unique_paths(localized_node, monkeypatch, tmp_path):
    node, _ = localized_node
    monkeypatch.setattr(node_module, "REPO_ROOT", tmp_path)
    node.map.set(12, 23, 100)
    node.map.set(22, 11, -1)
    ok, first = node.save_map()
    assert ok
    ok, second = node.save_map()
    assert ok and first != second
    restored = load_map_file(first)
    assert restored.data == node.map.data
    assert restored.origin == node.map.origin
    assert restored.resolution == node.map.resolution


def test_mapping_request_returns_while_transport_waits(localized_node):
    node, transport = localized_node
    entered = threading.Event()
    release = threading.Event()

    def wait_for_transport(running):
        assert running is True
        entered.set()
        assert release.wait(2.0)
        return True, "mapping started"

    transport.set_mapping = wait_for_transport
    try:
        assert node.set_mapping(True)[0]
        assert entered.wait(1.0)
        assert node._mapping_request["state"] == "starting"
        assert not node.set_mapping(True)[0]
    finally:
        release.set()
        if node._mapping_thread is not None:
            node._mapping_thread.join(2.0)
    assert node._mapping_request == {}


def test_exploration_stops_after_consecutive_motion_failures(
        localized_node, monkeypatch, navigation_clock):
    node, transport = localized_node
    targets = iter([(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)])
    monkeypatch.setattr(node.explorer, "next_goal", lambda *args: next(targets))
    assert node.start_exploration()[0]
    node.tick()
    for failure in range(1, 4):
        node.nav.finish("stuck", "no motion progress")
        navigation_clock(1.1)
        refresh_robot(transport)
        node.tick()
        assert node.exploration_state["consecutive_failures"] == failure
    assert node.exploration_state["state"] == "failed"
    assert node.exploration_state["blacklisted"] == 3


def test_arrival_resets_exploration_failure_budget(
        localized_node, monkeypatch, navigation_clock):
    node, transport = localized_node
    targets = iter([(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)])
    monkeypatch.setattr(node.explorer, "next_goal", lambda *args: next(targets))
    assert node.start_exploration()[0]
    node.tick()
    node.nav.finish("blocked", "blocked route")
    navigation_clock(1.1)
    refresh_robot(transport)
    node.tick()
    assert node.exploration_state["consecutive_failures"] == 1
    node.nav.finish("arrived", "reached next goal")
    node.tick()
    assert node.exploration_state["consecutive_failures"] == 0
    assert node.exploration_state["goals_visited"] == 1


def test_map_expiry_cancels_navigation_even_with_fresh_odometry(
        localized_node, navigation_clock):
    node, transport = localized_node
    assert node.set_goal(1.0, 0.0)[0]
    node.tick()
    assert transport.command[0] > 0
    navigation_clock(8.1)
    refresh_robot(transport)
    status = transport.status()
    assert status["connected"] and status["localization"]["valid"]
    assert not status["map_health"]["valid"]
    assert node.map_pose() is not None
    node.tick()
    assert node.nav.state == "cancelled"
    assert "地图过期" in node.nav.snapshot().message
    assert transport.command[:2] == (0.0, 0.0)
    assert not node.safety.snapshot()["nav_active"]
    assert not node.set_goal(1.0, 0.0)[0]


def test_exploration_requires_fresh_map_even_with_valid_localization(
        localized_node, navigation_clock):
    node, transport = localized_node
    navigation_clock(8.1)
    refresh_robot(transport)
    ok, message = node.start_exploration()
    assert not ok and "地图" in message
    assert node.exploration_state["state"] == "idle"
    assert transport.command[:2] == (0.0, 0.0)


def test_map_expiry_cancels_active_exploration_and_stops_motion(
        localized_node, monkeypatch, navigation_clock):
    node, transport = localized_node
    monkeypatch.setattr(node.explorer, "next_goal", lambda *args: (1.0, 0.0))
    assert node.start_exploration()[0]
    node.tick()
    assert node.exploration_state["state"] == "active"
    assert transport.command[0] > 0
    navigation_clock(8.1)
    refresh_robot(transport)
    node.tick()
    assert node.exploration_state["state"] == "cancelled"
    assert "地图过期" in node.exploration_state["message"]
    assert node.nav.state == "cancelled"
    assert transport.command[:2] == (0.0, 0.0)


def test_arrival_waits_for_new_observations_before_selecting_another_goal(
        localized_node, monkeypatch, navigation_clock):
    node, transport = localized_node
    calls = []

    def select_goal(*args):
        calls.append(args)
        return (0.5 + len(calls) * 0.5, 0.0)

    monkeypatch.setattr(node.explorer, "next_goal", select_goal)
    assert node.start_exploration()[0]
    node.tick()
    node.nav.finish("arrived", "observed first frontier")
    node.tick()
    assert len(calls) == 1
    assert node.exploration_state["goals_visited"] == 1
    assert transport.command[:2] == (0.0, 0.0)
    navigation_clock(2.9)
    refresh_robot(transport)
    node.tick()
    assert len(calls) == 1
    assert transport.command[:2] == (0.0, 0.0)
    navigation_clock(0.2)
    refresh_robot(transport)
    node.tick()
    assert len(calls) == 2
