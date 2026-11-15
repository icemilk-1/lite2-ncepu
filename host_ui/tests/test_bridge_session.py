"""Exercise production onboard service closures without ROS, sockets or hardware."""
import ast
import json
import struct
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

from lite2_sim.hardware.execution_guard import fresh, guarded_velocity
from lite2_sim.hardware.vendor_protocol import velocity_packets


BRIDGE = Path(__file__).resolve().parents[2] / "lite2_sim/hardware/lite2_live.py"


def bridge_session(on_sleep=None, on_packet=None):
    """Run actual nested bridge functions with deterministic time and fake UDP."""
    tree = ast.parse(BRIDGE.read_text(encoding="utf-8"))
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "bridge")
    definitions = [node for node in function.body if isinstance(node, ast.FunctionDef)]
    constants = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id in (
                    "POSTURE_TIMEOUT_S", "POSTURE_SETTLE_S", "CONTROL_SESSION_SETTLE_S",
                    "CONTROL_HEARTBEAT_PERIOD_S"):
                constants[target.id] = ast.literal_eval(node.value)
    clock = SimpleNamespace(now=100.0, sleeps=[])
    world = SimpleNamespace(
        clock=clock, packets=[], events=[], shutdown=False, diagnostics=[],
        state={"basic": 1, "received": clock.now},
        feedback={"joystick": [0.0, 0.0, 0.0], "command_received": clock.now},
        control=dict(armed=False, mode_confirmed=False, automatic_ack=0.0,
                     requested_at=0.0, depth_time=clock.now),
        session=dict(active=False, heartbeat_sent=0, started_at=0.0,
                     last_heartbeat=0.0, paused_reason="", generation=0),
        command=[0.0, 0.0, 0.0])

    def sleep(seconds):
        clock.now += seconds
        clock.sleeps.append(seconds)
        world.state["received"] = clock.now
        world.feedback["command_received"] = clock.now
        if on_sleep:
            on_sleep(world, seconds)

    def sendto(packet, address):
        code = struct.unpack_from("<i", packet)[0]
        world.packets.append((code, packet, address))
        if on_packet:
            on_packet(world, code)

    def publish(message):
        world.diagnostics.append(json.loads(message.data))

    scope = dict(constants, time=SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep),
                 lock=threading.RLock(), operation_lock=threading.Lock(),
                 command=world.command, state=world.state, feedback=world.feedback,
                 session=world.session, control=world.control,
                 sender=SimpleNamespace(sendto=sendto, getsockname=lambda: ("", 43893)),
                 rospy=SimpleNamespace(is_shutdown=lambda: world.shutdown,
                                       signal_shutdown=lambda reason: setattr(world, "shutdown", True)),
                 emit=lambda event, **values: world.events.append((event, values)),
                 TriggerResponse=lambda **values: SimpleNamespace(**values),
                 String=lambda **values: SimpleNamespace(**values),
                 struct=struct, json=json, fresh=fresh, guarded_velocity=guarded_velocity,
                 velocity_packets=velocity_packets,
                 diagnostic_pub=SimpleNamespace(publish=publish),
                 MAX_FORWARD_M_S=0.05, MAX_REVERSE_M_S=0.05, MAX_YAW_RAD_S=0.1)
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(BRIDGE), "exec"), scope)
    world.scope = scope
    world.codes = lambda: [item[0] for item in world.packets]
    world.service = lambda name: scope[name](None)
    world.stand = scope["posture_service"](6, "standing")
    world.lie = scope["posture_service"](1, "lying")
    return world


def test_connection_acquires_heartbeat_without_arming_or_posture():
    world = bridge_session()
    response = world.service("acquire_service")
    assert response.success
    assert world.session["active"]
    assert world.codes().count(0x21040001) == 1
    assert 0x21010202 not in world.codes()
    assert not world.control["armed"] and not world.control["mode_confirmed"]
    assert all(struct.unpack_from("<d", packet, 12)[0] == 0.0
               for code, packet, _ in world.packets if code in (320, 321, 325))


def test_active_acquire_disarms_previous_stream_without_resetting_heartbeat():
    world = bridge_session()
    world.service("acquire_service")
    world.control.update(armed=True, mode_confirmed=True, requested_at=99.0,
                         automatic_ack=99.5)
    world.command[:] = [0.05, 0.1, world.clock.now]
    previous = dict(world.session)
    assert world.service("acquire_service").success
    assert not world.control["armed"] and not world.control["mode_confirmed"]
    assert world.control["requested_at"] == world.control["automatic_ack"] == 0.0
    assert world.command == [0.0, 0.0, 0.0]
    assert world.session["generation"] > previous["generation"]
    for field in ("active", "heartbeat_sent", "started_at", "last_heartbeat"):
        assert world.session[field] == previous[field]
    assert world.codes().count(0x21040001) == 1


def test_standby_heartbeat_continues_for_long_initialization():
    def stop_after_65_seconds(world, seconds):
        if world.clock.now >= 165.0:
            world.shutdown = True

    world = bridge_session(on_sleep=stop_after_65_seconds)
    world.service("acquire_service")
    world.scope["heartbeat_loop"]()
    assert world.session["heartbeat_sent"] >= 325
    assert set(world.clock.sleeps) == {0.2}
    assert not world.control["armed"]
    assert 0x21010202 not in world.codes()


def test_disarm_invalidates_workers_but_preserves_heartbeat():
    world = bridge_session()
    world.service("acquire_service")
    world.control.update(armed=True, mode_confirmed=True)
    generation = world.session["generation"]
    assert world.service("disarm").success
    assert world.session["active"]
    assert world.session["generation"] > generation
    assert not world.control["armed"]
    assert 0x21010202 not in world.codes()


def test_release_stops_heartbeat_and_allows_explicit_connection_again():
    world = bridge_session()
    world.service("acquire_service")
    assert world.service("release_service").success
    assert not world.session["active"]
    assert not world.control["armed"]
    assert world.service("acquire_service").success
    assert world.session["active"]


@pytest.mark.parametrize("reason", ["estop", "remote_takeover"])
def test_automatic_acquire_cannot_erase_safety_handoff(reason):
    world = bridge_session()
    world.service("acquire_service")
    world.scope["stop_session"](reason)
    world.service("release_service")  # Normal stream exit must retain the latch.
    before = world.codes().count(0x21040001)
    assert not world.service("acquire_service").success
    assert world.codes().count(0x21040001) == before
    assert world.session["paused_reason"] == reason


def test_stand_timeout_keeps_heartbeat_and_sends_one_toggle():
    world = bridge_session()
    response = world.stand(None)
    assert not response.success
    assert world.codes().count(0x21010202) == 1
    assert 20.0 <= world.clock.now - 100.0 < 20.1
    assert world.session["active"]


def test_slow_first_initialization_has_twenty_second_budget():
    def initialization(world, seconds):
        if 0x21010202 in world.codes():
            elapsed = world.clock.now - 100.0
            world.state["basic"] = 17 if elapsed < 12 else 4 if elapsed < 14 else 5 if elapsed < 18 else 6

    world = bridge_session(on_sleep=initialization)
    assert world.stand(None).success
    assert world.clock.now >= 118.0
    assert world.codes().count(0x21010202) == 1


@pytest.mark.parametrize("action", ["disarm", "release_service", "estop_service"])
def test_cancel_during_warmup_prevents_late_toggle(action):
    fired = []

    def cancel(world, seconds):
        if not fired:
            fired.append(True)
            world.service(action)

    world = bridge_session(on_sleep=cancel)
    response = world.stand(None)
    assert not response.success and "cancelled" in response.message
    assert 0x21010202 not in world.codes()


def test_old_posture_worker_cannot_continue_after_release_and_reacquire():
    def reconnect(world, seconds):
        world.service("release_service")
        world.service("acquire_service")

    world = bridge_session(on_sleep=reconnect)
    assert not world.stand(None).success
    assert world.session["active"]
    assert 0x21010202 not in world.codes()


def test_new_stream_acquire_cancels_old_posture_without_dropping_heartbeat():
    world = bridge_session(on_sleep=lambda world, seconds: world.service("acquire_service"))
    world.service("acquire_service")
    assert not world.stand(None).success
    assert world.session["active"]
    assert world.codes().count(0x21040001) == 1
    assert 0x21010202 not in world.codes()


def test_prepare_ignores_old_ack_and_keeps_standby_on_failure():
    world = bridge_session()
    world.control["automatic_ack"] = 99.0
    response = world.service("prepare")
    assert not response.success
    assert 0x21010C03 in world.codes()
    assert world.session["active"] and not world.control["armed"]


@pytest.mark.parametrize("action", ["disarm", "release_service", "estop_service"])
def test_cancel_prepare_before_mode_request_cannot_enable(action):
    fired = []

    def cancel(world, seconds):
        if not fired:
            fired.append(True)
            world.service(action)
            world.control["automatic_ack"] = world.clock.now  # A late ACK changes nothing.

    world = bridge_session(on_sleep=cancel)
    assert not world.service("prepare").success
    assert 0x21010C03 not in world.codes()
    assert not world.control["armed"]


def test_late_ack_after_release_cannot_arm_old_prepare():
    def release_after_request(world, code):
        if code == 0x21010C03:
            world.service("release_service")
            world.control["automatic_ack"] = world.clock.now

    world = bridge_session(on_packet=release_after_request)
    assert not world.service("prepare").success
    assert not world.control["armed"] and not world.session["active"]


def test_new_ack_for_current_prepare_can_enable():
    def ack(world, code):
        if code == 0x21010C03:
            world.control["automatic_ack"] = world.clock.now

    world = bridge_session(on_packet=ack)
    assert world.service("prepare").success
    assert world.control["armed"] and world.control["mode_confirmed"]


def test_remote_takeover_cancels_standby_even_when_disarmed():
    def stop(world, seconds):
        world.shutdown = True

    world = bridge_session(on_sleep=stop)
    world.service("acquire_service")
    world.feedback["joystick"] = [0.2, 0.0, 0.0]
    world.scope["watchdog"]()
    assert not world.session["active"]
    assert world.session["paused_reason"] == "remote_takeover"
