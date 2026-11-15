"""Exercise the deployed stream lifecycle without ROS or physical commands."""
import ast
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "app/transports/ssh_ros_stream.py"


def lifecycle(monkeypatch, read_only=False, busy=False, accepted=True):
    calls = []
    handles = []

    class Guard:
        closed = False

        def close(self):
            calls.append("close")
            self.closed = True

    def open_guard(*args):
        handles.append(Guard())
        return handles[-1]

    def flock(guard, flags):
        calls.append("lock")
        if busy:
            raise RuntimeError("已有上位机控制连接")

    def proxy(service, kind):
        def invoke():
            assert not handles[-1].closed, "release must precede unlocking"
            calls.append(service)
            return SimpleNamespace(success=accepted, message="result")
        return invoke

    monkeypatch.setitem(sys.modules, "fcntl", SimpleNamespace(
        flock=flock, LOCK_EX=1, LOCK_NB=2))
    scope = {"READ_ONLY": read_only, "open": open_guard,
             "acquire_control_stream_lock": lambda guard: flock(guard, 3),
             "ensure_runtime": lambda: calls.append("runtime"),
             "rospy": SimpleNamespace(wait_for_service=lambda *a, **k: None,
                                      ServiceProxy=proxy),
             "Trigger": object, "threading": threading,
             "_control_cleanup_started": [False],
             "emit": lambda *args: calls.append("emit")}
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    functions = [item for item in tree.body if isinstance(item, ast.FunctionDef)
                 and item.name in ("begin_control_connection", "end_control_connection")]
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope, calls, handles


def test_read_only_connect_and_exit_have_no_control_side_effect(monkeypatch):
    scope, calls, handles = lifecycle(monkeypatch, read_only=True)
    guard = scope["begin_control_connection"]()
    scope["end_control_connection"](guard)
    assert guard is None and calls == [] and handles == []


def test_connection_acquires_standby_before_ready_and_releases_before_unlock(monkeypatch):
    scope, calls, handles = lifecycle(monkeypatch)
    guard = scope["begin_control_connection"]()
    assert calls == ["lock", "runtime", "/airace/acquire", "emit"]
    assert guard is handles[0] and not guard.closed
    scope["end_control_connection"](guard)
    assert calls[-2:] == ["/airace/release", "close"]
    scope["end_control_connection"](guard)
    assert calls.count("/airace/release") == 1


def test_second_control_stream_cannot_touch_existing_session(monkeypatch):
    scope, calls, handles = lifecycle(monkeypatch, busy=True)
    with pytest.raises(RuntimeError, match="已有上位机控制连接"):
        scope["begin_control_connection"]()
    assert calls == ["lock", "close"] and handles[0].closed


def test_refused_acquire_does_not_release_another_or_protected_session(monkeypatch):
    scope, calls, handles = lifecycle(monkeypatch, accepted=False)
    with pytest.raises(RuntimeError, match="待命心跳建立失败"):
        scope["begin_control_connection"]()
    assert handles[0].closed
    assert "/airace/release" not in calls


def test_old_stream_is_terminated_before_new_lock_is_acquired(monkeypatch):
    calls = []
    attempts = []

    def flock(guard, flags):
        attempts.append(flags)
        if len(attempts) == 1:
            raise OSError('busy')

    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(
        flock=flock, LOCK_EX=1, LOCK_NB=2))
    scope = {'monotonic': lambda: 1.0,
             'control_stream_owner': lambda guard: 123,
             'os': SimpleNamespace(kill=lambda pid, sig: calls.append((pid, sig))),
             'signal': SimpleNamespace(SIGTERM=15),
             'time': SimpleNamespace(sleep=lambda seconds: None),
             'emit': lambda *args: None}
    tree = ast.parse(SOURCE.read_text(encoding='utf8'))
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                    and item.name == 'acquire_control_stream_lock')
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'), scope)
    scope['acquire_control_stream_lock'](object())
    assert calls == [(123, 15)]
    assert len(attempts) == 2


def test_unidentified_lock_owner_is_preserved(monkeypatch):
    def flock(guard, flags):
        raise OSError('busy')

    def owner(guard):
        raise RuntimeError('unidentified process')

    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(
        flock=flock, LOCK_EX=1, LOCK_NB=2))
    scope = {'monotonic': lambda: 1.0, 'control_stream_owner': owner}
    tree = ast.parse(SOURCE.read_text(encoding='utf8'))
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                    and item.name == 'acquire_control_stream_lock')
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'), scope)
    with pytest.raises(RuntimeError, match='unidentified'):
        scope['acquire_control_stream_lock'](object())
