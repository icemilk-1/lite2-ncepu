"""Supervised stand/lie acceptance through the UI WebSocket; sends no walking or arm command.

Run only while the robot is lying, the area is clear and an operator is present.
The mobile App must not compete for the vendor's single control-terminal slot.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
from pathlib import Path
import time

import websockets

POSTURE_CONFIRM_TIMEOUT_S = 32.0  # 20 s action + 5 s reply grace + scheduling margin


def fresh_robot_state(snapshot: dict, received_age_s: float = 0.0) -> bool:
    """Require fresh robot feedback, including time spent in this client's buffer."""
    if snapshot.get("link", {}).get("connected") is not True:
        return False
    try:
        frame_age = float(snapshot["frame"]["age_s"])
        timeout = float(snapshot.get("safety_config", {}).get("state_timeout_s", 0.8))
        received_age = float(received_age_s)
    except (KeyError, TypeError, ValueError):
        return False
    return (all(math.isfinite(value) for value in (frame_age, timeout, received_age))
            and frame_age >= 0 and received_age >= 0 and timeout > 0
            and frame_age + received_age <= timeout)


async def verify(url: str, output: Path) -> bool:
    state = {}
    state_received = None
    pending = {}
    seq = 0
    owned = False
    report = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "ok": False,
              "steps": [], "transitions": [], "samples": []}
    last_basic = None
    started = time.monotonic()
    async with websockets.connect(url, max_size=32 * 1024 * 1024) as ws:
        async def receive():
            nonlocal state, last_basic, state_received
            async for raw in ws:
                message = json.loads(raw)
                if message.get("type") == "state":
                    state = message["data"]
                    state_received = time.monotonic()
                    basic = (state.get("frame") or {}).get("basic")
                    if basic is not None and basic != last_basic:
                        last_basic = basic
                        row = {"basic": basic, "elapsed_s": round(time.monotonic() - started, 3)}
                        report["transitions"].append(row)
                        print(json.dumps(row), flush=True)
                if message.get("type") == "ack":
                    future = pending.get(message.get("id"))
                    if future is not None and not future.done():
                        future.set_result(message)

        async def heartbeat():
            while True:
                if owned:
                    await ws.send(json.dumps({"type": "heartbeat"}))
                await asyncio.sleep(0.5)

        async def request(kind, **fields):
            nonlocal seq
            seq += 1
            key = seq
            future = asyncio.get_running_loop().create_future()
            pending[key] = future
            await ws.send(json.dumps(dict(fields, type=kind, id=key)))
            try:
                ack = await asyncio.wait_for(future, 6)
                report["steps"].append({"type": kind, **fields, "ack": ack})
                print(json.dumps({"type": kind, **fields, "ok": ack.get("ok"),
                                  "message": ack.get("message")}, ensure_ascii=False), flush=True)
                if not ack.get("ok"):
                    raise RuntimeError(ack.get("message", "Request rejected"))
            finally:
                pending.pop(key, None)

        async def wait_for(predicate, timeout, check_posture=False):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if check_posture and not state_is_fresh():
                    raise RuntimeError("Robot feedback stale or disconnected; no further posture requests")
                if predicate(state):
                    return
                if (state.get("frame") or {}).get("basic") in (8, 9, 11):
                    raise RuntimeError("Robot entered protection state; no further posture requests")
                if check_posture and state.get("posture", {}).get("state") in ("failed", "cancelled"):
                    raise RuntimeError(state["posture"].get("message", "Posture failed"))
                report["samples"].append({"elapsed_s": round(time.monotonic() - started, 3),
                                          "frame": state.get("frame"), "posture": state.get("posture"),
                                          "execution": state.get("execution")})
                await asyncio.sleep(0.15)
            raise RuntimeError("Timed out waiting for confirmed posture")

        def state_is_fresh():
            age = math.inf if state_received is None else time.monotonic() - state_received
            return fresh_robot_state(state, age)

        reader = asyncio.create_task(receive())
        beater = asyncio.create_task(heartbeat())
        try:
            await wait_for(lambda s: state_is_fresh(), 12)
            if state["link"].get("read_only") or state.get("safety", {}).get("estop"):
                raise RuntimeError("Read-only or emergency stop; cannot run acceptance")
            if (state.get("frame") or {}).get("basic") != 1:
                raise RuntimeError("Acceptance must start from normal lying basic=1")
            if state.get("execution", {}).get("armed") is not False:
                raise RuntimeError("Acceptance requires disarmed robot")
            if state.get("posture", {}).get("state") in ("pending", "running"):
                raise RuntimeError("An existing posture request is still pending")
            await request("takeover_control")
            owned = True
            await request("record", action="start")
            for action, target in (("stand", 6), ("lie", 1)):
                if not state_is_fresh():
                    raise RuntimeError("Robot feedback stale or disconnected; no posture request sent")
                if state.get("safety", {}).get("estop") or state.get("execution", {}).get("armed") is not False:
                    raise RuntimeError("Posture acceptance requires disarmed robot without emergency stop")
                expected_basic = 1 if action == "stand" else 6
                if (state.get("frame") or {}).get("basic") != expected_basic:
                    raise RuntimeError("Robot is not in the expected state before %s" % action)
                await request("posture", action=action)
                # Allow the next state broadcast to replace the previous request's result.
                await asyncio.sleep(0.2)
                await wait_for(lambda s: s.get("posture", {}).get("state") == "ok"
                               and s.get("posture", {}).get("action") == action
                               and (s.get("frame") or {}).get("basic") == target,
                               POSTURE_CONFIRM_TIMEOUT_S, check_posture=True)
                report["steps"].append({"confirmed": action, "basic": target,
                                        "elapsed_s": round(time.monotonic() - started, 3)})
                await asyncio.sleep(2)
            report["ok"] = True
        except Exception as error:
            report["error"] = str(error)
            print("FAILED: " + str(error), flush=True)
        finally:
            if owned:
                for kind in ("stop", "release_control"):
                    try:
                        await request(kind)
                    except Exception as error:
                        report.setdefault("cleanup_errors", []).append(str(error))
                await asyncio.sleep(0.2)
            report["final_state"] = state
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            beater.cancel()
            reader.cancel()
            await asyncio.gather(reader, beater, return_exceptions=True)
    print(json.dumps({"ok": report["ok"], "report": str(output)}, ensure_ascii=False), flush=True)
    return report["ok"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:8710/ws")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(verify(args.url, args.out)) else 1)
