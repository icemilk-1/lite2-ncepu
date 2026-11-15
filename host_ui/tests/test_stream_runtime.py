"""Run onboard startup and localization callbacks without ROS or subprocesses."""
import ast
import builtins
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


STREAM = Path(__file__).resolve().parents[1] / "app/transports/ssh_ros_stream.py"


def run_function(name, scope):
    tree = ast.parse(STREAM.read_text(encoding="utf-8"))
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(STREAM), "exec"), scope)
    return scope[name]


def test_velocity_receipt_uses_monotonic_clock():
    command = SimpleNamespace(linear=SimpleNamespace(x=0), angular=SimpleNamespace(z=0))
    scope = {'READ_ONLY': False, 'math': math, 'command': command,
             'last_command': [0.0], 'monotonic': lambda: 100.0}
    handle = run_function('handle_command', scope)
    handle({'linear': .5, 'angular': .2})
    assert scope['last_command'] == [100.0]
    assert command.linear.x == .5
    assert command.angular.z == .2


@pytest.mark.parametrize('now,received,expired', [
    (100.0, 99.8, False), (100.0, 99.0, True),
    (100.0, 101.0, True), (100.0, float('nan'), True),
])
def test_velocity_watchdog_expires_invalid_or_old_receipts(now, received, expired):
    tree = ast.parse(STREAM.read_text(encoding='utf8'))
    condition = next(node for node in ast.walk(tree)
                     if isinstance(node, ast.If)
                     and 'last_command[0]' in ast.unparse(node.test))
    previous = object()
    stopped = object()
    scope = {'monotonic': lambda: now, 'last_command': [received],
             'command': previous, 'Twist': lambda: stopped}
    exec(compile(ast.Module(body=[condition], type_ignores=[]), str(STREAM), 'exec'), scope)
    assert (scope['command'] is stopped) == expired


def runtime_fixture(tmp_path, monkeypatch, bridge=False, camera=False, process_exit=None):
    running = {"bridge": bridge, "camera": camera}
    calls = []
    events = []
    locks = []
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setitem(sys.modules, "fcntl", SimpleNamespace(
        LOCK_EX=2, flock=lambda handle, mode: locks.append(mode)))

    def master_state():
        publishers = []
        if running["bridge"]:
            publishers.append(("/airace/state", ["/airace_vendor_bridge"]))
        if running["camera"]:
            publishers.append(("/camera/depth/image_rect_raw", ["/camera"]))
        return [1, "system state", [publishers, [], []]]

    def start_process(arguments, **options):
        calls.append((arguments, options))
        running["bridge" if arguments[0] == "python3" else "camera"] = True
        return SimpleNamespace(pid=100 + len(calls), poll=lambda: process_exit)

    def local_open(path, mode):
        return builtins.open(tmp_path / Path(path).name, mode)

    def sleep(seconds):
        clock.now += seconds

    scope = {
        "open": local_open,
        "rospy": SimpleNamespace(get_master=lambda: SimpleNamespace(getSystemState=master_state)),
        "os": SimpleNamespace(path=os.path, devnull=os.devnull, setsid=lambda: None),
        "subprocess": SimpleNamespace(Popen=start_process),
        "MAPPING_LAUNCH": "/home/ysc/airace_deployment_20261114/hardware/navigation_mapping.launch",
        "RUNTIME_LOCK": "/tmp/airace_runtime.lock",
        "time": SimpleNamespace(sleep=sleep),
        "monotonic": lambda: clock.now,
        "emit": lambda kind, data: events.append((kind, data)),
    }
    return run_function("ensure_runtime", scope), calls, events, locks


def test_control_connection_starts_missing_bridge_and_camera(tmp_path, monkeypatch):
    ensure, calls, events, locks = runtime_fixture(tmp_path, monkeypatch)
    ensure()
    assert len(calls) == 2
    assert calls[0][0] == ["python3", "-u", "lite2_live.py", "bridge"]
    assert calls[0][1]["cwd"].endswith("/hardware")
    assert "rs_camera_424_240.launch" in calls[1][0][-1]
    assert {data["component"] for _, data in events} == {"执行桥接", "深度相机"}
    assert locks == [2]
    ensure()
    assert len(calls) == 2, "再次连接已有发布者时应复用进程"


@pytest.mark.parametrize("bridge,camera,expected", [(True, True, 0), (True, False, 1),
                                                    (False, True, 1)])
def test_control_connection_reuses_existing_publishers(tmp_path, monkeypatch, bridge, camera, expected):
    ensure, calls, _, _ = runtime_fixture(tmp_path, monkeypatch, bridge=bridge, camera=camera)
    ensure()
    assert len(calls) == expected


def test_failed_component_start_reports_failure(tmp_path, monkeypatch):
    ensure, calls, _, _ = runtime_fixture(tmp_path, monkeypatch, process_exit=1)
    with pytest.raises(RuntimeError, match="启动失败"):
        ensure()
    assert len(calls) == 1


# Control-mode startup and read-only side effects are exercised by the actual
# connection lifecycle in test_control_connection.py, including lock ownership
# and the standby-heartbeat acquisition service.


def localization_fixture():
    class TransformError(Exception):
        pass

    clock = SimpleNamespace(monotonic=10.0, ros=100.0, transform=99.95)
    lookups = []
    events = []

    def stamp(value):
        return SimpleNamespace(to_sec=lambda: value)

    def lookup(target, source, when):
        lookups.append((target, source, when))
        return [1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]

    point = SimpleNamespace(x=0.5, y=0.25, z=0.0)
    rotation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    message = SimpleNamespace(header=SimpleNamespace(frame_id="odom", stamp=stamp(99.9)),
                              pose=SimpleNamespace(pose=SimpleNamespace(position=point,
                                                                       orientation=rotation)))
    scope = {
        "np": np,
        "math": __import__("math"),
        "monotonic": lambda: clock.monotonic,
        "last_localization_emit": [0.0],
        "LOCALIZATION_EMIT_PERIOD_S": 0.1,
        "rospy": SimpleNamespace(Time=SimpleNamespace(now=lambda: stamp(clock.ros))),
        "transforms": SimpleNamespace(getLatestCommonTime=lambda *args: stamp(clock.transform),
                                      lookupTransform=lookup),
        "tf": SimpleNamespace(LookupException=TransformError, ConnectivityException=TransformError,
                              ExtrapolationException=TransformError,
                              transformations=SimpleNamespace(quaternion_matrix=lambda q: np.eye(4),
                                  quaternion_multiply=lambda left, right: right,
                                  euler_from_quaternion=lambda q: (0.0, 0.0, 0.0))),
        "emit": lambda kind, data: events.append((kind, data)),
    }
    return run_function("odometry_received", scope), scope, message, clock, lookups, events


def test_localization_uses_odometry_stamp_and_transforms_map_coordinates():
    receive, _, message, _, lookups, events = localization_fixture()
    receive(message)
    assert lookups == [("map", "odom", message.header.stamp)]
    pose = events[-1][1]
    assert pose["x"] == pytest.approx(1.5)
    assert pose["y"] == pytest.approx(2.25)
    assert pose["age_s"] == pytest.approx(0.1)
    assert pose["valid"] is True


@pytest.mark.parametrize("ros,transform", [(100.0, 97.0), (100.0, 101.0), (101.0, 100.95)])
def test_localization_rejects_stale_or_future_transform_and_stale_odometry(ros, transform):
    receive, _, message, clock, _, events = localization_fixture()
    clock.ros, clock.transform = ros, transform
    receive(message)
    assert events[-1][1]["valid"] is False


def test_localization_throttle_uses_monotonic_clock_during_ros_time_changes():
    receive, _, message, clock, lookups, events = localization_fixture()
    receive(message)
    clock.monotonic += 0.05
    clock.ros -= 10.0
    receive(message)
    assert len(lookups) == len(events) == 1
    clock.monotonic += 0.06
    receive(message)
    assert len(lookups) == len(events) == 2


def test_localization_missing_transform_emits_no_valid_pose():
    receive, scope, message, _, _, events = localization_fixture()

    def unavailable(*args):
        raise scope["tf"].LookupException("No map transform")

    scope["transforms"].lookupTransform = unavailable
    receive(message)
    assert events == []
