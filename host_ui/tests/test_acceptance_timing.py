"""Run acceptance timing with synthetic replies and a virtual clock, without HTTP."""
import asyncio
import json
from types import SimpleNamespace

import host_ui.tools.real_robot_acceptance as acceptance_module


def test_acceptance_waits_for_first_stand_beyond_action_and_ack_budget(monkeypatch):
    clock = [100.0]
    sent = []

    class FakeWebSocket:
        def __init__(self):
            self.inbox = [{"type": "hello", "data": {"client_id": 1}}]

        async def send(self, raw):
            message = json.loads(raw)
            sent.append(message)
            if "id" in message:
                self.inbox.append({"type": "ack", "id": message["id"],
                                   "ok": True, "message": "request accepted"})

        async def recv(self):
            return json.dumps(self.inbox.pop(0))

        async def __aenter__(self):
            return self

        async def __aexit__(self, *args):
            return False

    tool = acceptance_module.Acceptance("http://offline.invalid")

    def status():
        standing = clock[0] >= 126.0
        return {"frame": {"basic": 6 if standing else 1},
                "safety": {"estop": False}, "health": {"armed": False},
                "posture": {"state": "ok" if standing else "pending"}}

    async def advance_sleep(duration):
        clock[0] += duration

    monkeypatch.setattr(tool, "status", status)
    monkeypatch.setattr(acceptance_module, "time",
                        SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0]))
    monkeypatch.setattr(acceptance_module.asyncio, "sleep", advance_sleep)
    monkeypatch.setattr(acceptance_module.websockets, "connect",
                        lambda *args, **kwargs: FakeWebSocket())
    result = asyncio.run(tool.motion(0.05))
    stand_checks = [check for check in tool.checks if check["name"] == "起立确认 basic=6"]
    assert len(stand_checks) == 1
    assert stand_checks[0]["ok"]
    assert result["posture_stand"]["state"] == "ok"
    assert not any(message["type"] == "teleop" for message in sent)
