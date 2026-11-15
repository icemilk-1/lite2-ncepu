"""Exercise supervised posture-only acceptance with a synthetic WebSocket."""
import asyncio
import copy
import json
from types import SimpleNamespace

import pytest

import host_ui.tools.verify_posture_cycle as cycle_module


def base_state():
    return {"link": {"connected": True, "read_only": False},
            "frame": {"basic": 1, "age_s": 0.01},
            "safety_config": {"state_timeout_s": 0.8},
            "safety": {"estop": False}, "execution": {"armed": False},
            "posture": {"state": "idle"}}


@pytest.mark.parametrize("frame_age,received_age,expected", [
    (0.1, 0.2, True), (0.1, 0.71, False), (0.81, 0, False),
    (float("nan"), 0, False), (0.1, float("inf"), False), (-0.1, 0, False),
])
def test_freshness_includes_local_message_age(frame_age, received_age, expected):
    state = base_state()
    state["frame"]["age_s"] = frame_age
    assert cycle_module.fresh_robot_state(state, received_age) is expected


def run_synthetic_cycle(monkeypatch, tmp_path, *, frame_age=0.01,
                        stale_during_stand=False, pending_at_start=False):
    clock = [100.0]
    sent = []
    real_sleep = asyncio.sleep

    class FakeWebSocket:
        def __init__(self):
            self.inbox = []
            self.action = None
            self.action_started = None

        async def __aenter__(self):
            return self

        async def __aexit__(self, *args):
            return False

        def __aiter__(self):
            return self

        async def __anext__(self):
            await real_sleep(0)
            if self.inbox:
                return json.dumps(self.inbox.pop(0))
            state = base_state()
            state["frame"]["age_s"] = frame_age
            if pending_at_start:
                state["posture"]["state"] = "pending"
            if self.action is not None:
                elapsed = clock[0] - self.action_started
                # First stand takes 26 s, exceeding the old 22 s script deadline.
                ready = elapsed >= (26 if self.action == "stand" else 3)
                state["frame"]["basic"] = (6 if ready else 4) if self.action == "stand" else (1 if ready else 7)
                state["posture"] = {"action": self.action, "state": "ok" if ready else "pending"}
                if stale_during_stand and self.action == "stand":
                    state["frame"]["age_s"] = 2.0
            return json.dumps({"type": "state", "data": state})

        async def send(self, raw):
            message = json.loads(raw)
            sent.append(copy.deepcopy(message))
            if message["type"] == "posture":
                self.action = message["action"]
                self.action_started = clock[0]
            if "id" in message:
                self.inbox.append({"type": "ack", "id": message["id"],
                                   "ok": True, "message": "accepted"})

    async def advance_sleep(duration):
        # The polling coroutine owns this virtual clock; concurrent heartbeat
        # sleeps must not add their duration on top of elapsed wall time.
        if duration != 0.5:
            clock[0] += duration
        await real_sleep(0)

    monkeypatch.setattr(cycle_module, "time", SimpleNamespace(
        monotonic=lambda: clock[0], strftime=lambda _: "virtual time"))
    monkeypatch.setattr(cycle_module.asyncio, "sleep", advance_sleep)
    monkeypatch.setattr(cycle_module.websockets, "connect",
                        lambda *args, **kwargs: FakeWebSocket())
    path = tmp_path / "posture_cycle.json"
    ok = asyncio.run(cycle_module.verify("ws://offline.invalid", path))
    return ok, json.loads(path.read_text(encoding="utf-8")), sent


def test_verify_posture_cycle_waits_for_26_second_first_stand(monkeypatch, tmp_path):
    ok, report, sent = run_synthetic_cycle(monkeypatch, tmp_path)
    assert ok, report.get("error")
    assert [step["confirmed"] for step in report["steps"] if "confirmed" in step] == ["stand", "lie"]
    assert [message["action"] for message in sent if message["type"] == "posture"] == ["stand", "lie"]
    assert not any(message["type"] in ("arm", "teleop", "goal", "exploration_start") for message in sent)


def test_verify_posture_cycle_never_sends_posture_on_stale_initial_feedback(monkeypatch, tmp_path):
    ok, report, sent = run_synthetic_cycle(monkeypatch, tmp_path, frame_age=2.0)
    assert not ok
    assert not any(message["type"] == "posture" for message in sent)


def test_verify_posture_cycle_stops_after_stale_feedback_during_stand(monkeypatch, tmp_path):
    ok, report, sent = run_synthetic_cycle(monkeypatch, tmp_path, stale_during_stand=True)
    assert not ok
    assert "stale" in report["error"]
    assert [message["action"] for message in sent if message["type"] == "posture"] == ["stand"]
    assert any(message["type"] == "stop" for message in sent)


def test_verify_posture_cycle_rejects_existing_pending_posture(monkeypatch, tmp_path):
    ok, report, sent = run_synthetic_cycle(monkeypatch, tmp_path, pending_at_start=True)
    assert not ok
    assert "pending" in report["error"]
    assert not any(message["type"] == "posture" for message in sent)
