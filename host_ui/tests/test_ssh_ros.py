"""机载 ROS · SSH 链路单测：完全离线，用假 channel 验证 JSON 命令/回执/重连。

覆盖契约第 3.3 / 3.2 节的关键行为：
* arm/disarm 通过 stdin 发 ``{"action": "prepare"|"disarm"}`` 并等待 control 回执
  （或 execution.armed 变化），超时中文原因，pending 期间速度强制 0；
* 坏 JSON 行不拆链、缺 type/data 忽略、pending 缓冲超限丢弃；
* 断线指数退避重连，``reconnect.attempts`` 递增、恢复发事件；
* set_posture/emergency_stop 只发机载命令并等回执（Windows 不再直发 UDP），
  cancel 立刻中止并补零；
* ``status().control`` / ``reconnect`` / ``execution`` 字段；
* 机载 stream 源码保持 Python 2.7 语法兼容（无 f-string/注解/pathlib）。
"""
import ast
import json
import re
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

import host_ui.app.transports.ssh_ros as ssh_module
from host_ui.app.transports.base import TransportCallbacks, TransportError
from host_ui.app.transports.ssh_ros import SshRosTransport


def wait_until(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class Recorder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.frames = []
        self.events = []
        self.maps = []

    def on_frame(self, frame) -> None:
        with self._lock:
            self.frames.append(frame)

    def on_event(self, level, code, message) -> None:
        with self._lock:
            self.events.append((level, code, message))

    def on_map(self, grid) -> None:
        with self._lock:
            self.maps.append(grid)

    def callbacks(self) -> TransportCallbacks:
        return TransportCallbacks(on_frame=self.on_frame, on_event=self.on_event,
                                  on_map=self.on_map)

    def codes(self) -> list[str]:
        with self._lock:
            return [code for _level, code, _message in self.events]


class FakeChannel:
    """最小 paramiko Channel 替身：记录上行的 JSON 行，注入下行消息。"""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.sent: list[dict] = []
        self.inbox: list[bytes] = []
        self.stderr = b""
        self.exit_ready = False
        self.closed = False
        self.on_send = None

    # paramiko Channel 接口
    def exit_status_ready(self) -> bool:
        return self.exit_ready

    def sendall(self, data) -> None:
        with self.lock:
            for line in data.splitlines():
                if line.strip():
                    self.sent.append(json.loads(line.decode("utf-8")))
            hook = self.on_send
        if hook is not None:
            hook(self)

    def recv_stderr_ready(self) -> bool:
        return bool(self.stderr)

    def recv_stderr(self, _size: int) -> bytes:
        data, self.stderr = self.stderr, b""
        return data

    def recv_ready(self) -> bool:
        return bool(self.inbox)

    def recv(self, _size: int) -> bytes:
        return self.inbox.pop(0) if self.inbox else b""

    def close(self) -> None:
        self.closed = True

    # 测试辅助
    def actions(self) -> list[str]:
        with self.lock:
            return [m.get("action") for m in self.sent if m.get("action")]

    def action_messages(self, action: str) -> list[dict]:
        with self.lock:
            return [m for m in self.sent if m.get("action") == action]


def make_transport(options=None) -> SshRosTransport:
    return SshRosTransport(options or {}, Recorder().callbacks())


def connected_transport(options=None) -> SshRosTransport:
    """构造一个“已建立连接”的 transport（绕过 paramiko）。"""
    transport = make_transport(options)
    transport.channel = FakeChannel()
    transport._started = True
    transport._link_ok = True
    transport.last_frame = time.monotonic()
    transport.last_basic = 6
    return transport


def test_remote_stream_exit_preserves_error_details():
    transport = connected_transport()
    transport.channel.exit_ready = True
    transport.channel.stderr = b'RuntimeError: previous stream still holds control lock'
    with pytest.raises(TransportError, match='previous stream still holds control lock'):
        transport._io_loop()


def test_successful_posture_clears_previous_control_error():
    transport = connected_transport()
    transport._last_error = 'Previous protection state basic=8'
    transport._posture_request = {'request_id': 'current', 'target': 'stand'}
    transport._on_control({'action': 'posture', 'request_id': 'current',
                           'target': 'stand', 'success': True, 'message': 'standing confirmed'})
    assert transport.status()['control']['last_error'] == ''


STATE_PAYLOAD = {"position": [-0.49, 0.16, 0.30],
                 "rpy_deg": [0.2, -0.6, -24.2],
                 "basic": 6, "battery_level_raw": 69.0}


# ---------------------------------------------------------------------------
# 基础状态映射
# ---------------------------------------------------------------------------
def test_real_state_mapping_and_staleness():
    frames = []
    transport = SshRosTransport({}, TransportCallbacks(frames.append,
                                                       lambda *args: None))
    assert not transport.status()["connected"]
    transport._receive({"type": "state", "data": {
        "position": [-0.49, 0.16, 0.30], "rpy_deg": [0.2, -0.6, -24.2],
        "basic": 6, "battery_level_raw": 69.0, "error_state": 0}}) 
    assert frames[0].source == "ssh_ros"
    assert frames[0].raw["battery_level_raw"] == 69.0
    assert frames[0].raw["battery_percent"] == 69.0
    assert transport.status()["connected"]
    transport.last_frame = time.monotonic() - 2
    assert transport.status()["stale"]


def test_status_control_execution_and_reconnect_shape():
    transport = make_transport({"reconnect": {"enabled": True,
                                              "min_delay_s": 0.2,
                                              "max_delay_s": 0.4}})
    status = transport.status()
    assert status["connected"] is False
    control = status["control"]
    for key, value in {"armed": None, "mode_confirmed": None,
                       "pending": None, "last_error": "",
                       "depth_age_s": None, "automatic_ack_age_s": None,
                       "bridge_depth_age_s": None,
                       "camera_depth_age_s": None,
                       "depth_fresh": None}.items():
        assert control[key] == value
    assert control["control_session"] == {}
    assert status["reconnect"]["enabled"] is True
    assert status["reconnect"]["attempts"] == 0
    assert status["reconnect"]["next_retry_in_s"] is None
    assert status["execution"] == {}

    transport._receive({"type": "execution", "data": {
        "reason": "enabled", "host_ui_depth_age_s": 0.05,
        "control": {"armed": True, "mode_confirmed": True}}})
    transport._receive({"type": "state", "data": STATE_PAYLOAD})
    status = transport.status()
    assert status["execution"]["reason"] == "enabled"
    assert status["control"]["armed"] is True
    assert status["control"]["mode_confirmed"] is True
    assert status["control"]["depth_age_s"] == pytest.approx(0.05)
    assert status["control"]["camera_depth_age_s"] == pytest.approx(0.05)
    assert status["control"]["automatic_ack_age_s"] is not None


def test_control_session_and_bridge_depth_diagnostics():
    transport = make_transport({})
    transport._receive({"type": "execution", "data": {
        "reason": "enabled",
        "host_ui_depth_age_s": 0.35,
        "depth_age_s": 0.08,
        "depth_fresh": True,
        "control_session": {"active": True, "heartbeat_sent": 42,
                            "heartbeat_age_s": 0.12},
        "control": {"armed": True, "mode_confirmed": True}}})
    control = transport.status()["control"]
    assert control["bridge_depth_age_s"] == pytest.approx(0.08, abs=0.02)
    assert control["camera_depth_age_s"] == pytest.approx(0.35, abs=0.02)
    assert control["depth_fresh"] is True
    assert control["control_session"]["active"] is True
    assert control["control_session"]["heartbeat_age_s"] == pytest.approx(0.12)
    # 深度门禁以桥接判定为准
    assert control["depth_age_s"] == pytest.approx(0.08)


def test_remote_map_and_execution_feedback():
    maps = []
    transport = SshRosTransport({}, TransportCallbacks(
        lambda frame: None, lambda *args: None, on_map=maps.append))
    transport._receive({"type": "map", "data": {
        "width": 2, "height": 1, "resolution": 0.05, "origin": [0, 0, 0],
        "data": [0, 100], "name": "map"}})
    assert maps[0].at(1, 0) == 100
    transport._receive({"type": "execution", "data": {"reason": "disarmed"}})
    assert transport.status()["execution"]["reason"] == "disarmed"


def test_sensor_snapshot_uses_local_age():
    transport = SshRosTransport({}, TransportCallbacks(lambda frame: None,
                                                       lambda *args: None))
    transport._receive({"type": "sensor", "data": {
        "name": "depth", "stamp": 1517160588.0, "jpeg": "test",
        "valid_percent": 84.4}})
    data = transport.sensor_snapshot()["depth"]
    assert data["jpeg"] == "test"
    assert 0 <= data["age_s"] < 1
    assert data["stamp"] == 1517160588.0


@pytest.mark.parametrize("state,level", [("ready", "info"), ("failed", "warn")])
def test_lidar_runtime_diagnostics_reach_events_without_disrupting_control(state, level):
    recorder = Recorder()
    transport = SshRosTransport({}, recorder.callbacks())
    transport._link_ok = True
    transport._control_session = {"active": True, "heartbeat_sent": 50}
    transport._receive({"type": "runtime", "data": {
        "component": "激光雷达", "state": state, "message": "雷达诊断信息"}})
    assert recorder.events == [(level, "ssh_ros_runtime", "激光雷达：雷达诊断信息")]
    assert transport._link_ok
    assert transport._control_session == {"active": True, "heartbeat_sent": 50}


# ---------------------------------------------------------------------------
# IO 循环 / 健壮解析
# ---------------------------------------------------------------------------
def test_io_loop_sends_zero_for_expired_commands():
    transport = make_transport()
    channel = FakeChannel()
    transport.channel = channel
    transport.command = (0.1, 0.2, time.monotonic() - 1)
    channel.on_send = lambda _channel: transport.closed.set()
    transport._io_loop()
    assert channel.sent == [{"linear": 0.0, "angular": 0.0}]


def test_bad_json_line_does_not_break_link():
    recorder = Recorder()
    transport = SshRosTransport({}, recorder.callbacks())
    transport._dispatch_line(b"{this is not json")
    transport._dispatch_line(b'["not", "dict"]')
    transport._dispatch_line(b'{"no_type": 1}')
    assert transport.status()["bad_lines"] == 3
    transport._dispatch_line((json.dumps(
        {"type": "state", "data": STATE_PAYLOAD}) + "\n").encode())
    assert len(recorder.frames) == 1  # 坏行之后链路仍然工作
    assert "ssh_ros_bad_line" in recorder.codes()


def test_io_loop_pending_overflow_is_dropped(monkeypatch):
    monkeypatch.setattr(ssh_module, "PENDING_LIMIT", 64)

    class OverflowChannel(FakeChannel):
        def __init__(self):
            super().__init__()
            self.chunks = [b"x" * 200]

        def recv_ready(self):
            return True

        def recv(self, _size):
            return self.chunks.pop(0) if self.chunks else b""

    transport = make_transport()
    transport.channel = OverflowChannel()
    with pytest.raises(TransportError):
        transport._io_loop()
    assert transport.status()["pending_overflow"] == 1


def test_stop_sends_zero_and_closes_channel():
    transport = connected_transport()
    channel = transport.channel
    transport.stop()
    assert transport.status()["connected"] is False
    assert channel.closed is True
    assert channel.sent[-1] == {"linear": 0.0, "angular": 0.0}


# ---------------------------------------------------------------------------
# 使能 / 解除
# ---------------------------------------------------------------------------
def test_arm_sends_prepare_and_waits_control_ack():
    transport = connected_transport()
    channel = transport.channel
    result = {}

    def run():
        result["value"] = transport.arm(timeout_s=2.0)

    worker = threading.Thread(target=run)
    worker.start()
    assert wait_until(lambda: channel.action_messages("prepare"))
    # pending 期间速度强制 0
    transport.send_velocity(0.4, 0.5)
    assert transport.command[:2] == (0.0, 0.0)
    assert transport.status()["control"]["pending"] == "prepare"
    transport._on_control({"action": "prepare", "success": True,
                           "message": "自动模式确认"})
    worker.join(2.0)
    assert result["value"] == (True, "自动模式确认")
    assert transport.status()["control"]["pending"] is None
    assert transport.status()["control"]["armed"] is True


def test_arm_succeeds_on_execution_armed_change():
    transport = connected_transport()
    channel = transport.channel
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(value=transport.arm(timeout_s=2.0)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("prepare"))
    transport._receive({"type": "execution", "data": {
        "reason": "enabled", "host_ui_depth_age_s": 0.02,
        "control": {"armed": True, "mode_confirmed": True}}})
    worker.join(2.0)
    ok, message = result["value"]
    assert ok is True
    assert "自动模式" in message
    assert transport.status()["control"]["armed"] is True


def test_arm_timeout_returns_chinese_error():
    transport = connected_transport()
    ok, message = transport.arm(timeout_s=0.6)
    assert ok is False
    assert "超时" in message
    assert transport.status()["control"]["pending"] is None


def test_arm_and_posture_require_link():
    transport = make_transport()
    ok, message = transport.arm()
    assert ok is False and "未连接" in message
    ok, message = transport.disarm()
    assert ok is False and "未连接" in message
    ok, message = transport.set_posture("stand")
    assert ok is False and "未连接" in message
    ok, message = transport.emergency_stop()
    assert ok is False and "物理急停" in message


def test_disarm_success_updates_status():
    transport = connected_transport()
    channel = transport.channel
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(value=transport.disarm(timeout_s=2.0)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("disarm"))
    transport._on_control({"action": "disarm", "success": True,
                           "message": "已退出自动模式"})
    worker.join(2.0)
    assert result["value"] == (True, "已退出自动模式")
    assert transport.status()["control"]["armed"] is False


# ---------------------------------------------------------------------------
# 姿态 / 急停：只发机载命令，等 control 回执
# ---------------------------------------------------------------------------
def test_posture_sends_single_command_and_waits_ack():
    transport = connected_transport()
    channel = transport.channel
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(value=transport.set_posture(
            "stand", timeout_s=2.0)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("posture"))
    request = channel.action_messages("posture")[-1]
    transport._on_control({"action": "posture", "target": "stand",
                           "request_id": request["request_id"],
                           "success": True, "message": "机载端已确认姿态切换完成"})
    worker.join(2.0)
    assert result["value"][0] is True
    assert len(channel.action_messages("posture")) == 1  # 只发一次，不连发
    assert transport.status()["control"]["pending"] is None


@pytest.mark.parametrize("options", [{}, {"timeout_s": "invalid"}])
def test_first_posture_waits_for_ack_after_old_timeout_budget(monkeypatch, options):
    """An 18 s first action must outlast the former 10 s + 5 s reply budget."""
    transport = connected_transport()
    clock = [100.0]
    monkeypatch.setattr(ssh_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    def advance_wait(duration):
        clock[0] += duration
        if clock[0] >= 118.0:
            request = transport.channel.action_messages("posture")[0]
            transport._on_control(dict(request, success=True,
                                       message="first stand confirmed after 18 s"))

    monkeypatch.setattr(transport._control_lock, "wait", advance_wait)
    ok, message = transport.set_posture("stand", **options)
    assert ok, message
    assert clock[0] >= 118.0
    assert len(transport.channel.action_messages("posture")) == 1
    assert transport._pending is None
    assert not transport.posture_lock.locked()


def test_posture_cancel_waits_for_matching_onboard_result_and_sends_zero():
    transport = connected_transport()
    channel = transport.channel
    cancel = threading.Event()
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(value=transport.set_posture(
            "lie", cancel=cancel, timeout_s=5.0)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("posture"))
    request = channel.action_messages("posture")[-1]
    cancel.set()
    time.sleep(0.12)
    assert worker.is_alive()
    assert transport.status()["control"]["pending"] == "posture"
    assert transport.posture_lock.locked()
    transport.send_velocity(0.1, 0.1)
    assert transport.command[:2] == (0.0, 0.0)
    transport._on_control(dict(request, success=True, message="lying confirmed"))
    worker.join(2.0)
    ok, message = result["value"]
    assert ok is False
    assert "取消" in message
    assert channel.sent[-1] == {"linear": 0.0, "angular": 0.0}


def test_late_or_wrong_target_posture_reply_cannot_complete_new_request():
    transport = connected_transport()
    channel = transport.channel
    first = {}
    cancel = threading.Event()
    worker = threading.Thread(target=lambda: first.update(value=transport.set_posture(
        "stand", cancel=cancel, timeout_s=1)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("posture"))
    old_request = channel.action_messages("posture")[-1]
    cancel.set()
    transport._on_control(dict(old_request, success=True, message="stand done"))
    worker.join(2)
    assert first["value"][0] is False
    second = {}
    worker = threading.Thread(target=lambda: second.update(value=transport.set_posture(
        "lie", timeout_s=1)))
    worker.start()
    assert wait_until(lambda: len(channel.action_messages("posture")) == 2)
    new_request = channel.action_messages("posture")[-1]
    assert old_request["request_id"] != new_request["request_id"]
    transport._on_control(dict(old_request, success=True, message="old stand done"))
    transport._on_control(dict(new_request, target="stand", success=True,
                              message="wrong target"))
    time.sleep(0.05)
    assert worker.is_alive() and not second
    transport._on_control(dict(new_request, success=True, message="lie done"))
    worker.join(2)
    assert second["value"] == (True, "lie done")


def test_cancelled_posture_has_a_bounded_wait_without_an_onboard_reply():
    transport = connected_transport()
    transport.POSTURE_ACK_GRACE_S = 0.0
    cancel = threading.Event()
    result = {}
    worker = threading.Thread(target=lambda: result.update(value=transport.set_posture(
        "lie", cancel=cancel, timeout_s=1)))
    worker.start()
    assert wait_until(lambda: transport.channel.action_messages("posture"))
    cancel.set()
    worker.join(2)
    assert not worker.is_alive()
    assert not result["value"][0] and "超时" in result["value"][1]
    assert transport.status()["control"]["pending"] is None
    assert not transport.posture_lock.locked()


@pytest.mark.parametrize("outcome", ["success", "service_error", "invalid_target"])
def test_onboard_posture_reply_always_echoes_request_identity(outcome):
    path = Path(__file__).resolve().parents[1] / "app/transports/ssh_ros_stream.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(item for item in tree.body
                    if isinstance(item, ast.FunctionDef) and item.name == "_posture_worker")
    replies = []

    def wait_for_service(*args, **kwargs):
        if outcome == "service_error":
            raise RuntimeError("service unavailable")

    scope = {"POSTURE_TARGETS": {"stand": 6, "lie": 1}, "SERVICE_WAIT_S": 2,
             "latest_basic": [6], "monotonic": lambda: 100,
             "emit": lambda kind, data: replies.append(data),
             "Trigger": object,
             "rospy": SimpleNamespace(wait_for_service=wait_for_service,
                                      ServiceProxy=lambda *args: lambda: SimpleNamespace(
                                          success=True, message="standing"))}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), scope)
    target = "bad" if outcome == "invalid_target" else "stand"
    scope["_posture_worker"](target, "unique-request")
    assert replies[0]["request_id"] == "unique-request"
    assert replies[0]["target"] == target
    assert replies[0]["success"] == (outcome == "success")


def test_onboard_busy_reply_echoes_request_identity():
    path = Path(__file__).resolve().parents[1] / "app/transports/ssh_ros_stream.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(item for item in tree.body
                    if isinstance(item, ast.FunctionDef) and item.name == "handle_command")
    replies = []
    scope = {"READ_ONLY": False, "_begin_command": lambda name: False,
             "emit": lambda kind, data: replies.append(data)}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), scope)
    scope["handle_command"]({"action": "posture", "target": "lie", "request_id": "busy-request"})
    assert replies[0]["request_id"] == "busy-request"
    assert replies[0]["target"] == "lie" and not replies[0]["success"]


def test_posture_rejects_invalid_action():
    transport = connected_transport()
    ok, message = transport.set_posture("fly")
    assert ok is False
    assert "无效" in message


def test_emergency_stop_relays_to_onboard_and_waits_ack():
    transport = connected_transport()
    channel = transport.channel
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(value=transport.emergency_stop()))
    worker.start()
    assert wait_until(lambda: channel.action_messages("estop"))
    transport._on_control({"action": "estop", "success": True,
                           "message": "机载端已发送厂商急停 0x21010C0E 三次"})
    worker.join(2.0)
    ok, message = result["value"]
    assert ok is True
    assert "0x21010C0E" in message


def test_emergency_stop_ack_timeout_warns_physical_stop():
    transport = connected_transport()
    transport.ESTOP_ACK_TIMEOUT_S = 0.2
    ok, message = transport.emergency_stop()
    assert ok is False
    assert "物理急停" in message


# ---------------------------------------------------------------------------
# 断线重连
# ---------------------------------------------------------------------------
def test_reconnect_attempts_increase_and_recovers(monkeypatch):
    recorder = Recorder()
    transport = SshRosTransport({"reconnect": {"enabled": True,
                                               "min_delay_s": 0.02,
                                               "max_delay_s": 0.04}},
                                recorder.callbacks())
    transport._started = True
    calls = []

    def fake_connect():
        calls.append(time.monotonic())
        if len(calls) <= 2:
            raise TransportError("模拟连接失败")
        transport.channel = FakeChannel()

    monkeypatch.setattr(transport, "_connect", fake_connect)
    worker = threading.Thread(target=transport._supervise, daemon=True)
    worker.start()
    try:
        assert wait_until(
            lambda: transport.status()["reconnect"]["attempts"] >= 2, 2.0)
        assert wait_until(lambda: "ssh_ros_reconnected" in recorder.codes(), 2.0)
        assert wait_until(
            lambda: transport.status()["reconnect"]["attempts"] >= 3, 2.0)
    finally:
        transport.closed.set()
        worker.join(2.0)
    assert len(calls) >= 3


def test_reconnect_disabled_reports_down():
    recorder = Recorder()
    transport = SshRosTransport({"reconnect": {"enabled": False}},
                                recorder.callbacks())
    transport._started = True
    transport._supervise()
    assert "ssh_ros_down" in recorder.codes()
    assert transport.status()["reconnect"]["enabled"] is False


# ---------------------------------------------------------------------------
# 机载 stream 源码的 Python 2.7 兼容性
# ---------------------------------------------------------------------------
def test_initial_connect_failure_retries_and_recovers(monkeypatch):
    transport = make_transport({"reconnect_min_delay_s": 0.01})
    attempts = []

    def connect():
        attempts.append(1)
        if len(attempts) == 1:
            raise TransportError("initial failure")
        transport.channel = FakeChannel()

    monkeypatch.setattr(transport, "_connect", connect)
    monkeypatch.setattr(transport, "_io_loop", lambda: transport.closed.set())
    transport.start()
    try:
        assert wait_until(lambda: len(attempts) == 2)
        assert transport.error == ""
    finally:
        transport.stop()


def test_initial_failure_without_reconnect_raises(monkeypatch):
    transport = make_transport({"reconnect_enabled": False})

    def fail():
        raise TransportError("initial failure")

    monkeypatch.setattr(transport, "_connect", fail)
    with pytest.raises(TransportError):
        transport.start()
    assert transport.worker is None


def test_read_only_transport_emits_no_velocity_or_actions():
    transport = make_transport({"read_only": True})
    transport.channel = FakeChannel()
    transport._send_line({"linear": 0.1, "angular": 0.0})
    transport._send_zero_now()
    assert transport.channel.sent == []
    with pytest.raises(TransportError, match="只读"):
        transport._send_line({"action": "posture", "target": "stand"})
    assert transport.channel.sent == []
    assert transport.status()["read_only"]
    assert not transport.capabilities["arm"]
    assert not transport.capabilities["posture"]

def test_stream_script_stays_python27_compatible():
    source = Path(ssh_module.__file__).with_name(
        "ssh_ros_stream.py").read_text(encoding="utf-8")
    assert "pathlib" not in source
    assert not re.search(r"(?<![\w.])f['\"]", source), "不允许 f-string"
    assert not re.search(r"def \w+\([^)]*\)\s*->", source), "不允许类型注解"
    # 海象运算符 := 是 Python 3.8 语法；字符串里的 ROS 参数语法
    # start_lidar_driver:=true 不是运算符，所以只检查真正的 OP token。
    import io
    import tokenize

    walrus = [token for token in
              tokenize.generate_tokens(io.StringIO(source).readline)
              if token.type == tokenize.OP and token.string == ":="]
    assert not walrus, "不允许海象运算符（Python 2.7 不支持）"
    # 深度心跳必须用单调时钟，而不是 rospy 墙钟
    assert "monotonic" in source
    assert "Float64(data=float(now_mono))" in source
    assert not re.search(r"Float64\(data=float\(rospy\.Time", source)


# ---------------------------------------------------------------------------
# 建图：命令流保持 mapping_start / mapping_stop（驱动决策在机载 stream 内完成）
# ---------------------------------------------------------------------------
def test_set_mapping_sends_mapping_action_and_waits_ack():
    transport = connected_transport()
    channel = transport.channel
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(value=transport.set_mapping(True)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("mapping_start"))
    transport._on_control({"action": "mapping_start", "success": True,
                           "message": "建图已启动（复用机载雷达驱动）"})
    worker.join(2.0)
    ok, message = result["value"]
    assert ok is True
    assert "复用机载雷达驱动" in message

    worker = threading.Thread(
        target=lambda: result.update(value=transport.set_mapping(False)))
    worker.start()
    assert wait_until(lambda: channel.action_messages("mapping_stop"))
    transport._on_control({"action": "mapping_stop", "success": True,
                           "message": "建图已停止"})
    worker.join(2.0)
    assert result["value"] == (True, "建图已停止")


def test_set_mapping_rejected_on_read_only_link():
    transport = make_transport({"read_only": True})
    ok, message = transport.set_mapping(True)
    assert ok is False
    assert "只读" in message


# ---------------------------------------------------------------------------
# /api/posture 异步语义 + 急停复位门槛（配合 Lead 的 server/node 契约）
# ---------------------------------------------------------------------------
def test_posture_endpoint_async_and_estop_reset_gate():
    from fastapi.testclient import TestClient
    from host_ui.app.server import create_app
    from host_ui.app.types import Frame

    app = create_app(auto_connect=False)
    calls = []

    class FakeTransport:
        kind = "fake"
        label = "假链路"
        # 不声明 vendor_estop，避免急停路径干扰本用例
        capabilities = {"arm": True, "posture": True}

        def status(self):
            return {"connected": True, "execution": {}, "control": {},
                    "reconnect": {}}

        def send_velocity(self, linear, angular):
            pass

        def set_posture(self, action, cancel=None, timeout_s=10.0):
            calls.append(action)
            return True, "fake 已确认 " + action

        def arm(self, timeout_s=8.0):
            return True, "fake 使能"

        def disarm(self, timeout_s=8.0):
            return True, "fake 解除"

        def stop(self):
            pass

    with TestClient(app) as client:
        node = app.state.node
        node.transport = FakeTransport()
        node.set_operator(1)
        node.frame = Frame(source="fake", stamp=time.monotonic(),
                           position=(0.0, 0.0, 0.0),
                           rpy_deg=(0.0, 0.0, 0.0), basic=6)
        assert not client.post("/api/posture",
                               json={"action": "bad"}).json()["ok"]
        client.post("/api/estop", json={"engaged": True})
        assert not client.post("/api/posture",
                               json={"action": "lie"}).json()["ok"]
        assert calls == []
        # 急停复位必须携带 hold_s（>= 配置的 1.2 s），否则拒绝
        assert client.post("/api/estop/reset").json()["ok"] is False
        assert client.post("/api/estop/reset",
                           json={"hold_s": 2.0}).json()["ok"] is True
        response = client.post("/api/posture", json={"action": "lie"}).json()
        assert response["ok"] is True
        assert "正在" in response["message"]  # 异步：立即返回“正在趴下…”
        assert wait_until(
            lambda: node.posture_state.get("state") == "ok", 2.0)
        assert calls == ["lie"]
