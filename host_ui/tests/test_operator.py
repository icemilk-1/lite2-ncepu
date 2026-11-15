"""WebSocket 操控权独占与观察者权限测试（TestClient，无实机）。"""
from __future__ import annotations

import time

from fastapi.testclient import TestClient

from host_ui.app.config import Config
from host_ui.app.server import create_app


def test_observer_heartbeat_cannot_renew_operator_lease():
    from host_ui.app.node import HostNode
    from host_ui.app.server import dispatch_client_message

    node = HostNode(Config({"record": {"enabled": False}}))
    node.set_operator(1)
    node.last_operator_activity = 123.0
    dispatch_client_message(node, {"type": "heartbeat", "client_id": 1}, client_id=2)
    assert node.last_operator_activity == 123.0
    dispatch_client_message(node, {"type": "heartbeat"}, client_id=1)
    assert node.last_operator_activity > 123.0


def test_operator_release_stops_automatic_and_manual_commands():
    from host_ui.app.node import HostNode

    node = HostNode(Config({"record": {"enabled": False}}))
    node.set_operator(1)
    assert node.nav.set_goal(1.0, 0.0, None, node.map, (0.0, 0.0, 0.0), time.monotonic())[0]
    node.exploration_state["state"] = "active"
    node.safety.set_nav_command(0.05, 0.0, True)
    node.safety.set_teleop(0.05, 0.0, True)
    assert node.release_operator(1)[0]
    assert node.nav.state == "cancelled"
    assert node.exploration_state["state"] == "cancelled"
    assert not node.safety.snapshot()["nav_active"]
    assert not node.safety.snapshot()["teleop"]["deadman"]


def make_client() -> TestClient:
    config = Config({"link": {"kind": "mock"}, "record": {"enabled": False}})
    app = create_app(config, auto_connect=False)
    return TestClient(app)


def recv_ack(ws, timeout: float = 3.0) -> dict:
    end = time.time() + timeout
    while time.time() < end:
        message = ws.receive_json()
        if message.get("type") == "ack":
            return message
    raise AssertionError("没有收到 ack")


def recv_hello(ws) -> dict:
    while True:
        message = ws.receive_json()
        if message.get("type") == "hello":
            return message["data"]


def test_first_client_claims_control() -> None:
    client = make_client()
    with client:
        with client.websocket_connect("/ws") as ws:
            hello = recv_hello(ws)
            assert hello["client_id"] >= 1
            ws.send_json({"type": "claim_control"})
            ack = recv_ack(ws)
            assert ack["ok"] is True
            status = client.get("/api/status").json()
            assert status["operator"]["id"] == hello["client_id"]


def test_second_client_replaces_old_and_requires_claim() -> None:
    with make_client() as client:
        with client.websocket_connect("/ws") as old:
            recv_hello(old)
            old.send_json({"type": "claim_control"})
            assert recv_ack(old)["ok"]
            with client.websocket_connect("/ws") as new:
                recv_hello(new)
                new.send_json({"type": "teleop", "linear": 0.1,
                               "angular": 0, "deadman": True})
                assert not recv_ack(new)["ok"]
                new.send_json({"type": "claim_control"})
                assert recv_ack(new)["ok"]
                assert old.receive()["code"] == 4001
                assert client.get("/api/status").json()["operator"]["id"] == 1


def test_takeover_on_replacement_connection() -> None:
    with make_client() as client:
        with client.websocket_connect("/ws") as old:
            recv_hello(old)
            old.send_json({"type": "claim_control"})
            assert recv_ack(old)["ok"]
            with client.websocket_connect("/ws") as new:
                recv_hello(new)
                new.send_json({"type": "takeover_control"})
                assert recv_ack(new)["ok"]
                assert old.receive()["code"] == 4001
                assert client.get("/api/status").json()["operator"]["id"] == 1


def test_operator_released_on_disconnect() -> None:
    client = make_client()
    with client:
        with client.websocket_connect("/ws") as ws:
            hello = recv_hello(ws)
            ws.send_json({"type": "claim_control"})
            assert recv_ack(ws)["ok"] is True
            assert client.get("/api/status").json()["operator"]["id"] == hello["client_id"]
        # 退出 with 后连接关闭，操控权应释放
        time.sleep(0.2)
        assert client.get("/api/status").json()["operator"]["id"] is None
