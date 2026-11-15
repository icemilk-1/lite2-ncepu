"""Onboard lidar recovery: no hardware, subprocesses, ROS or motion commands."""
import ast
import builtins
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "app/transports/ssh_ros_stream.py"


def load_functions(scope, *names):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in names]
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), "exec"), scope)


def runtime(tmp_path, monkeypatch, present=False):
    config = SimpleNamespace(present=present, known_pid=None, registration_delay=0.0,
                             started_at=None, process_exit=None, master_error=False,
                             points=[(1.0, 2.0, 0.5)], stamp=99.98, empty=False,
                             no_message=False, read_only=False)
    clock = [10.0]
    processes, events, locks = [], [], []
    monkeypatch.setitem(sys.modules, "fcntl", SimpleNamespace(
        LOCK_EX=2, flock=lambda handle, flags: locks.append(handle.name)))

    def open_local(path, mode):
        return builtins.open(tmp_path / Path(path).name, mode)

    def publisher_present():
        if config.master_error:
            return None
        return config.present or (config.started_at is not None and
                                  clock[0] - config.started_at >= config.registration_delay)

    def start(arguments, **kwargs):
        processes.append(arguments)
        config.known_pid = 123
        config.started_at = clock[0]
        return SimpleNamespace(pid=123, poll=lambda: config.process_exit)

    def wait_for_message(topic, msg_type, timeout):
        clock[0] += min(0.05, timeout)
        if config.no_message:
            raise RuntimeError("No messages")
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            to_sec=lambda: config.stamp)), width=0 if config.empty else 28800,
            height=1, data=b"points")

    scope = {"READ_ONLY": False, "open": open_local, "RUNTIME_LOCK": "/tmp/airace_runtime.lock",
             "LIDAR_PID": "/tmp/airace_lidar.pid", "LIDAR_LOG": "/tmp/airace_lidar.log",
             "LIDAR_TOPIC": "/rslidar_points", "LIDAR_START_TIMEOUT_S": 0.3,
             "LIDAR_MAX_AGE_S": 1.0, "PointCloud2": object,
             "lidar_driver_running": publisher_present,
             "lidar_runtime_pid": lambda: config.known_pid,
             "subprocess": SimpleNamespace(Popen=start),
             "os": SimpleNamespace(devnull=os.devnull, setsid=lambda: None),
             "monotonic": lambda: clock[0],
             "time": SimpleNamespace(sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds)),
             "rospy": SimpleNamespace(is_shutdown=lambda: False, wait_for_message=wait_for_message,
                                      Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 100.0))),
             "point_cloud2": SimpleNamespace(read_points=lambda *a, **k: iter(config.points)),
             "_finite": math.isfinite,
             "emit": lambda kind, data: events.append((kind, data))}
    load_functions(scope, "lidar_cloud_age", "ensure_lidar_runtime", "lidar_runtime_worker",
                   "start_lidar_recovery")
    return scope, config, clock, processes, events, locks


def test_missing_lidar_starts_sensor_without_slam_and_reconnect_reuses_it(tmp_path, monkeypatch):
    scope, _, _, processes, _, locks = runtime(tmp_path, monkeypatch)
    result = scope["ensure_lidar_runtime"]()
    assert result["started"] and result["points"] == 28800
    assert result["age_s"] == pytest.approx(0.02)
    assert len(processes) == 1
    assert "rslidar_pointcloud rs_lidar_16.launch" in processes[0][-1]
    assert "navigation_mapping" not in processes[0][-1]
    assert (tmp_path / "airace_lidar.pid").read_text() == "123"
    assert not scope["ensure_lidar_runtime"]()["started"]
    assert len(processes) == 1 and len(locks) == 2
    assert all(Path(path).name == "airace_runtime.lock" for path in locks)


def test_registered_publisher_is_reused_only_after_actual_points_arrive(tmp_path, monkeypatch):
    scope, _, _, processes, _, _ = runtime(tmp_path, monkeypatch, present=True)
    result = scope["ensure_lidar_runtime"]()
    assert not result["started"] and result["points"] > 0
    assert processes == []


def test_delayed_registration_does_not_start_driver_twice(tmp_path, monkeypatch):
    scope, config, _, processes, _, _ = runtime(tmp_path, monkeypatch)
    config.registration_delay = 0.1
    assert scope["ensure_lidar_runtime"]()["started"]
    assert not scope["ensure_lidar_runtime"]()["started"]
    assert len(processes) == 1


def test_live_pid_suppresses_duplicate_launch_even_after_registration_timeout(tmp_path, monkeypatch):
    scope, config, _, processes, _, _ = runtime(tmp_path, monkeypatch)
    config.registration_delay = 10
    with pytest.raises(RuntimeError, match="未就绪"):
        scope["ensure_lidar_runtime"]()
    config.present = True
    assert not scope["ensure_lidar_runtime"]()["started"]
    assert len(processes) == 1


@pytest.mark.parametrize("problem", ["no_message", "empty", "old", "future", "invalid_points"])
def test_publisher_without_fresh_nonempty_cloud_never_starts_a_second_driver(
        tmp_path, monkeypatch, problem):
    scope, config, _, processes, _, _ = runtime(tmp_path, monkeypatch, present=True)
    if problem == "no_message":
        config.no_message = True
    elif problem == "empty":
        config.empty = True
    elif problem == "old":
        config.stamp = 90
    elif problem == "future":
        config.stamp = 101
    else:
        config.points = [(float("nan"), 1, 1), (float("inf"), 1, 1)]
    with pytest.raises(RuntimeError, match="未就绪"):
        scope["ensure_lidar_runtime"]()
    assert processes == []


def test_master_query_error_does_not_launch_a_driver(tmp_path, monkeypatch):
    scope, config, _, processes, _, _ = runtime(tmp_path, monkeypatch)
    config.master_error = True
    with pytest.raises(RuntimeError, match="检测失败"):
        scope["ensure_lidar_runtime"]()
    assert processes == []


def test_driver_early_exit_reports_log_path(tmp_path, monkeypatch):
    scope, config, _, processes, _, _ = runtime(tmp_path, monkeypatch)
    config.process_exit = 1
    with pytest.raises(RuntimeError, match="airace_lidar.log"):
        scope["ensure_lidar_runtime"]()
    assert len(processes) == 1


def test_background_failure_reports_diagnostic_without_raising_or_releasing_control(tmp_path, monkeypatch):
    scope, config, _, _, events, _ = runtime(tmp_path, monkeypatch)
    config.master_error = True
    scope["lidar_runtime_worker"]()
    assert events[-1][0] == "runtime"
    assert events[-1][1]["state"] == "failed"
    assert "检测失败" in events[-1][1]["message"]


def test_background_starts_only_for_control_connections_after_subscribers():
    source = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    call_line = next(node.lineno for node in tree.body if isinstance(node, ast.Expr)
                     and isinstance(node.value, ast.Call)
                     and getattr(node.value.func, "id", "") == "start_lidar_recovery")
    acquire_line = next(node.lineno for node in tree.body if isinstance(node, ast.Assign)
                        and isinstance(node.value, ast.Call)
                        and getattr(node.value.func, "id", "") == "begin_control_connection")
    subscribers = [node.lineno for node in tree.body if isinstance(node, ast.Expr)
                   and isinstance(node.value, ast.Call)
                   and isinstance(node.value.func, ast.Attribute)
                   and node.value.func.attr == "Subscriber"]
    assert acquire_line < max(subscribers) < call_line
    workers = []
    scope = {"READ_ONLY": True, "_start_worker": workers.append, "lidar_runtime_worker": object()}
    load_functions(scope, "start_lidar_recovery")
    scope["start_lidar_recovery"]()
    assert workers == []
    scope["READ_ONLY"] = False
    scope["start_lidar_recovery"]()
    assert workers == [scope["lidar_runtime_worker"]]


def test_mapping_refuses_to_start_without_verified_lidar(tmp_path):
    import threading
    launches, replies = [], []

    def no_lidar():
        raise RuntimeError("雷达未就绪")

    scope = {"mapping_lock": threading.Lock(), "mapping_pid": lambda: None,
             "ensure_lidar_runtime": no_lidar, "mapping_log_tail": lambda: "",
             "emit": lambda kind, data: replies.append(data),
             "subprocess": SimpleNamespace(Popen=lambda *a, **k: launches.append(a))}
    load_functions(scope, "mapping_worker")
    scope["mapping_worker"]("mapping_start")
    assert not replies[-1]["success"]
    assert "雷达未就绪" in replies[-1]["message"] and launches == []
