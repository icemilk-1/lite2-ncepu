"""通信故障回归，使用假链路与本机回环端口。"""
import asyncio
import threading
import time

import pytest
from fastapi.testclient import TestClient

from host_ui.app.config import Config
from host_ui.app.mapstore import demo_map
from host_ui.app.node import HostNode
from host_ui.app.server import WebSocketHub, create_app
from host_ui.app.transports.base import Transport, TransportError
from host_ui.app.transports.vendor_udp import VendorUdpTransport
from host_ui.app.types import Frame
from host_ui.tests.test_single_client import Socket, receive
from host_ui.tests.test_ssh_ros import connected_transport, make_transport, FakeChannel


class SampleTransport(Transport):
    kind = "sample"
    label = "sample"

    def start(self):
        self.callbacks.on_frame(Frame("sample", time.monotonic(), (1, 2, 0), (0, 0, 0)))
        self.callbacks.on_scan([(1, 2)])
        if self.options.get("fail"):
            raise TransportError("start failed")

    def stop(self):
        self.callbacks.on_frame(Frame("retired", time.monotonic(), (9, 9, 0), (0, 0, 0)))

    def send_velocity(self, linear, angular):
        pass


def test_old_callbacks_cannot_replace_current_samples(monkeypatch):
    transports = []

    def factory(kind, options, callbacks):
        transport = SampleTransport(options, callbacks)
        transports.append(transport)
        return transport

    monkeypatch.setattr("host_ui.app.node.create_transport", factory)
    node = HostNode(Config({"record": {"enabled": False}}))
    assert node.connect("sample")[0]
    assert node.frame.source == "sample"
    assert node.scan == [(1, 2)]
    old = transports[0].callbacks
    assert node.connect("sample")[0]
    current_frame, current_map, revision = node.frame, node.map, node.map_revision
    old.on_frame(Frame("late", time.monotonic(), (8, 8, 0), (0, 0, 0)))
    old.on_map(demo_map())
    old.on_scan([(8, 8)])
    assert node.frame is current_frame
    assert node.map is current_map and node.map_revision == revision
    assert node.scan == [(1, 2)]
    node.disconnect()
    assert node.frame is None and node.scan == [] and node.scan_stamp == 0
    old.on_frame(current_frame)
    assert node.frame is None
    assert not node.connect("sample", {"fail": True})[0]
    assert node.frame is None and node.scan == []
    transports[-1].callbacks.on_frame(current_frame)
    assert node.frame is None
    node.shutdown()
    assert not node.connect("sample")[0]


def test_stopping_during_reconnect_closes_late_channel():
    transport = make_transport({"reconnect": {"min_delay_s": 0.01}})
    transport.STOP_JOIN_S = 0.01
    entered, finish = threading.Event(), threading.Event()
    late = FakeChannel()
    calls = []

    def connect():
        calls.append(True)
        if len(calls) == 1:
            raise TransportError("initial failure")
        entered.set()
        assert finish.wait(3)
        transport.channel = late

    transport._connect = connect
    try:
        transport.start()
        assert entered.wait(2)
        worker = transport.worker
        transport.stop()
        assert transport.worker is worker and worker.is_alive()
        with pytest.raises(TransportError, match="旧 SSH"):
            transport.start()
        finish.set()
        worker.join(2)
        assert not worker.is_alive()
        assert late.closed and transport.channel is None
    finally:
        finish.set()
        transport.stop()


def test_disconnect_clears_motion_even_without_an_operator():
    node = HostNode(Config({"record": {"enabled": False},
                            "safety": {"require_operator": False}}))
    assert node.operator_id is None
    assert node.nav.set_goal(1, 0, None, node.map, (0, 0, 0), time.monotonic())[0]
    node.safety.set_teleop(0.1, 0, True)
    node.safety.set_nav_command(0.1, 0, True)
    node.disconnect()
    assert node.nav.state == "cancelled"
    assert not node.safety.snapshot()["nav_active"]
    assert not node.safety.snapshot()["teleop"]["deadman"]
    node.shutdown()


def test_estop_requires_new_ack():
    transport = connected_transport()
    transport.ESTOP_ACK_TIMEOUT_S = 0.01
    transport._control_results["estop"] = {"success": True, "message": "old ack"}
    try:
        ok, message = transport.emergency_stop()
        assert not ok and "old ack" not in message
    finally:
        transport.stop()


def test_stop_closes_resources_from_in_progress_ssh_connect():
    transport = make_transport()
    resource = FakeChannel()
    transport._track_connecting(resource)
    transport.stop()
    assert resource.closed
    rejected = FakeChannel()
    with pytest.raises(TransportError, match="取消"):
        transport._track_connecting(rejected)
    assert rejected.closed


def test_udp_receiver_port_is_exclusive():
    callbacks = make_transport().callbacks
    options = {"robot_ip": "127.0.0.1", "robot_port": 9,
               "bind_host": "127.0.0.1", "bind_port": 0}
    first = VendorUdpTransport(options, callbacks)
    second = None
    try:
        first.start()
        second = VendorUdpTransport(dict(options, bind_port=first._bound[1]), callbacks)
        with pytest.raises(TransportError, match="绑定"):
            second.start()
    finally:
        first.stop()
        if second is not None:
            second.stop()


def test_slow_broadcast_does_not_block_identity_or_leave_operator():
    async def scenario():
        entered = asyncio.Event()

        class SlowSocket(Socket):
            async def send_text(self, text):
                entered.set()
                await asyncio.Event().wait()

            async def close(self, **kwargs):
                raise OSError("closed handshake unavailable")

        cleanup = []
        hub = WebSocketHub(on_disconnect=lambda: cleanup.append(True))
        hub.SEND_TIMEOUT_S = 0.05
        socket = SlowSocket()
        await hub.add(socket)
        broadcast = asyncio.create_task(hub.broadcast(["state"]))
        await entered.wait()
        assert await asyncio.wait_for(hub.is_current(1, socket), 0.02)
        await broadcast
        assert await hub.count() == 0 and cleanup == [True]

    asyncio.run(scenario())


def test_invalid_message_keeps_connection_and_rest_sessions_are_fenced():
    app = create_app(Config({"record": {"enabled": False}}), auto_connect=False)
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as old:
            old_session = receive(old, "hello")["data"]["client_session"]
            for invalid in (1, None, [], {"type": []}):
                old.send_json(invalid)
                assert receive(old, "ack")["ok"] is False
            old.send_json({"type": "claim_control"})
            assert receive(old, "ack")["ok"]
            with client.websocket_connect("/ws") as new:
                session = receive(new, "hello")["data"]["client_session"]
                assert session != old_session
                assert client.post("/api/operator", json={"action": "release"},
                                   headers={"X-Control-Session": old_session}).status_code == 409
                assert client.post("/api/operator", json={"action": "release"}).status_code == 409
                new.send_json({"type": "claim_control"})
                assert receive(new, "ack")["ok"]
                assert client.post("/api/operator", json={"action": "release"},
                                   headers={"X-Control-Session": session}).json()["ok"]
                assert client.post("/api/stop").json()["ok"]


def test_failure_of_retired_broadcast_preserves_new_session():
    async def scenario():
        entered, fail = asyncio.Event(), asyncio.Event()

        class RetiredSocket(Socket):
            async def send_text(self, text):
                entered.set()
                await fail.wait()
                raise OSError("retired socket failed")

        cleanup = []
        hub = WebSocketHub(on_disconnect=lambda: cleanup.append(True))
        old, new = RetiredSocket(), Socket()
        await hub.add(old)
        broadcast = asyncio.create_task(hub.broadcast(["old state"]))
        await entered.wait()
        await hub.add(new)
        token = hub.session_token
        fail.set()
        await broadcast
        assert await hub.is_current(1, new)
        assert hub.session_token == token
        assert cleanup == []
        await hub.broadcast(["new state"])
        assert new.sent == ["new state"]

    asyncio.run(scenario())


def test_waiting_for_control_ack_keeps_heartbeat_and_stop_responsive(monkeypatch):
    app = create_app(Config({"record": {"enabled": False}}), auto_connect=False)
    entered, finish = threading.Event(), threading.Event()

    def slow_arm():
        entered.set()
        assert finish.wait(3)
        return True, "confirmed"

    with TestClient(app) as client:
        monkeypatch.setattr(app.state.node, "arm", slow_arm)
        with client.websocket_connect("/ws") as ws:
            receive(ws, "hello")
            ws.send_json({"type": "claim_control"})
            assert receive(ws, "ack")["ok"]
            try:
                ws.send_json({"type": "arm", "id": "slow"})
                assert entered.wait(1)
                ws.send_json({"type": "stop", "id": "stop"})
                assert receive(ws, "ack")["id"] == "stop"
                ws.send_json({"type": "heartbeat"})
                ws.send_json({"type": "ping", "t": 123})
                assert receive(ws, "pong")["t"] == 123
            finally:
                finish.set()
            assert receive(ws, "ack")["id"] == "slow"
