"""只读模式的功能放行、地图列表/加载、探索配置与接口测试。"""
from __future__ import annotations

from types import SimpleNamespace

from fastapi.testclient import TestClient

from host_ui.app.config import Config
from host_ui.app.server import _operator_gate, create_app


class ReadOnlyTransport:
    def status(self):
        return {"kind": "ssh_ros", "connected": True, "read_only": True}


def fake_node(read_only: bool = True, operator_id=None):
    return SimpleNamespace(
        config=Config({"safety": {"require_operator": True}}),
        transport=ReadOnlyTransport() if read_only else SimpleNamespace(
            status=lambda: {"kind": "ssh_ros", "connected": True}),
        operator_id=operator_id,
    )


def test_read_only_allows_observation_actions_without_operator() -> None:
    node = fake_node()
    for action in ("set_limits", "map_align", "record", "clear_trail"):
        ok, why = _operator_gate(node, None, action=action)
        assert ok is True, (action, why)
    for action in ("teleop", "arm", "posture", "goal"):
        ok, why = _operator_gate(node, None, action=action)
        assert ok is False
        assert "操控权" in why


def test_non_readonly_still_requires_operator() -> None:
    node = fake_node(read_only=False)
    ok, _ = _operator_gate(node, None, action="set_limits")
    assert ok is False
    node.operator_id = 7
    ok, _ = _operator_gate(node, None, action="set_limits")
    assert ok is True


def make_app():
    config = Config({"link": {"kind": "mock"}, "map": {"demo": False},
                     "record": {"enabled": False},
                     "web": {"host": "127.0.0.1", "port": 8710}})
    return TestClient(create_app(config, auto_connect=False))


def test_maps_listing_and_safe_load() -> None:
    client = make_app()
    with client:
        listing = client.get("/api/maps").json()
        assert listing["maps"], "host_ui/maps 下应有真实地图"
        first = listing["maps"][0]
        response = client.post("/api/map/load", json={"path": first["path"]}).json()
        assert response["ok"] is True, response
        status = client.get("/api/status").json()
        assert status["map"]["loaded"] is True


def test_map_load_rejects_path_outside_maps_dir() -> None:
    client = make_app()
    with client:
        for path in ("C:/Windows/win.ini", "../../README.md",
                     "host_ui/maps/../../README.md"):
            response = client.post("/api/map/load", json={"path": path}).json()
            assert response["ok"] is False, path
            assert "maps" in response["message"] or "不存在" in response["message"]


# --------------------------------------------------------------------------- 探索配置
def test_explore_config_defaults_and_clamp() -> None:
    config = Config({})
    explore = config.get("explore")
    assert explore["enabled"] is True
    assert explore["min_frontier_cells"] == 8
    assert explore["max_goal_distance_m"] == 15.0
    assert explore["gain_weight"] == 0.5
    assert explore["blacklist_radius_m"] == 0.30
    assert explore["max_backoff_m"] == 0.60
    assert explore["max_consecutive_failures"] == 3


def test_explore_config_normalizes_out_of_range() -> None:
    config = Config({"explore": {"min_frontier_cells": 0,
                                 "max_goal_distance_m": 999.0,
                                 "gain_weight": -3.0,
                                 "blacklist_radius_m": 0.0,
                                 "max_backoff_m": 999.0,
                                 "max_consecutive_failures": 99,
                                 "enabled": "yes"}})
    explore = config.get("explore")
    assert explore["min_frontier_cells"] == 1
    assert explore["max_goal_distance_m"] == 50.0
    assert explore["gain_weight"] == 0.0
    assert explore["blacklist_radius_m"] == 0.05
    assert explore["max_backoff_m"] == 2.0
    assert explore["max_consecutive_failures"] == 10


def test_explore_config_ignores_garbage() -> None:
    config = Config({"explore": {"min_frontier_cells": "abc",
                                 "max_goal_distance_m": None,
                                 "gain_weight": float("nan")}})
    explore = config.get("explore")
    assert explore["min_frontier_cells"] == 8
    assert explore["max_goal_distance_m"] == 15.0
    assert explore["gain_weight"] == 0.5


# --------------------------------------------------------------------------- 探索接口
def test_explore_routes_registered() -> None:
    paths = {route.path for route in create_app(
        Config({"link": {"kind": "mock"}, "map": {"demo": False},
                "record": {"enabled": False}}), auto_connect=False).routes}
    assert "/api/explore/start" in paths
    assert "/api/explore/stop" in paths


def test_explore_gate_requires_operator() -> None:
    node = fake_node(read_only=False)
    for action in ("exploration_start", "exploration_stop"):
        ok, why = _operator_gate(node, None, action=action)
        assert ok is False
        assert "操控权" in why