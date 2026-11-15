import asyncio

from fastapi.testclient import TestClient

from host_ui.app.config import Config
from host_ui.app.server import WebSocketHub, create_app


class Socket:
    def __init__(self):
        self.closed = None
        self.sent = []

    async def close(self, code=1000, reason=""):
        self.closed = code

    async def send_text(self, text):
        self.sent.append(text)


def test_replacement_fences_old_commands_cleanup_and_broadcast():
    async def scenario():
        hub = WebSocketHub()
        old, new = Socket(), Socket()
        assert await hub.add(old) == 1
        replacements = []
        assert await hub.add(new, lambda: replacements.append(True)) == 1
        assert old.closed == 4001
        assert replacements == [True]
        assert await hub.count() == 1
        assert not await hub.is_current(1, old)
        assert await hub.is_current(1, new)
        assert not await hub.remove(1, old, lambda: replacements.append(False))
        await hub.broadcast(["state"])
        assert old.sent == []
        assert new.sent == ["state"]
        assert replacements == [True]
        assert await hub.remove(1, new)
        assert await hub.count() == 0

    asyncio.run(scenario())


def receive(ws, kind):
    for _ in range(100):
        message = ws.receive_json()
        if message.get("type") == kind:
            return message
    raise AssertionError(f"Missing {kind}")


def test_refresh_reuses_id_and_old_disconnect_preserves_new_operator():
    app = create_app(Config({"record": {"enabled": False}}), auto_connect=False)
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as old:
            assert receive(old, "hello")["data"]["client_id"] == 1
            old.send_json({"type": "claim_control"})
            assert receive(old, "ack")["ok"]
            with client.websocket_connect("/ws") as new:
                assert receive(new, "hello")["data"]["client_id"] == 1
                assert app.state.node.operator_id is None
                assert app.state.node.clients == 1
                new.send_json({"type": "claim_control"})
                assert receive(new, "ack")["ok"]
                assert app.state.node.operator_id == 1
                assert old.receive()["code"] == 4001
        with client.websocket_connect("/ws") as refreshed:
            assert receive(refreshed, "hello")["data"]["client_id"] == 1
            assert app.state.node.clients == 1
