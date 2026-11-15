"""离线仿真传输测试：真实跑数秒，验证帧、地图、运动、断链、接管与碰撞。"""
import threading
import time

import pytest

from host_ui.app.transports.base import TransportCallbacks
from host_ui.app.transports.mock import DEFAULT_OPTIONS, MockTransport


def wait_until(predicate, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class Recorder:
    """线程安全地收集传输层回调。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.frames = []
        self.events = []
        self.maps = []
        self.scans = []

    def on_frame(self, frame) -> None:
        with self._lock:
            self.frames.append(frame)

    def on_event(self, level, code, message) -> None:
        with self._lock:
            self.events.append((level, code, message))

    def on_map(self, grid) -> None:
        with self._lock:
            self.maps.append(grid)

    def on_scan(self, points) -> None:
        with self._lock:
            self.scans.append(list(points))

    def callbacks(self) -> TransportCallbacks:
        return TransportCallbacks(on_frame=self.on_frame, on_event=self.on_event,
                                  on_map=self.on_map, on_scan=self.on_scan)

    def frame_count(self) -> int:
        with self._lock:
            return len(self.frames)

    def scan_count(self) -> int:
        with self._lock:
            return len(self.scans)

    def codes(self) -> list[str]:
        with self._lock:
            return [code for _level, code, _message in self.events]

    def last_frame(self):
        with self._lock:
            return self.frames[-1] if self.frames else None


def make_transport(recorder: Recorder, options: dict | None = None) -> MockTransport:
    return MockTransport(options or {}, recorder.callbacks())


def test_mock_default_options_exposed():
    assert set(DEFAULT_OPTIONS) == {
        "zero_timeout_s", "max_linear_m_s", "max_angular_rad_s", "fps",
        "collision_radius_m", "scan_enabled", "scan_max_hz"}
    transport = MockTransport({}, Recorder().callbacks())
    assert transport.kind == "mock"
    assert transport.label == "离线演示"


def test_mock_runs_two_seconds_with_map_and_motion():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        assert transport.status()["connected"] is True
        assert wait_until(lambda: len(recorder.maps) >= 1, 1.0)
        grid = recorder.maps[0]
        assert (grid.width, grid.height) == (360, 240)
        assert grid.resolution == pytest.approx(0.05)

        time.sleep(0.4)
        start_pose = transport.status()["pose"]
        transport.send_velocity(0.3, 0.25)
        time.sleep(1.6)

        assert recorder.frame_count() >= 50
        status = transport.status()
        assert status["pose"][0] > start_pose[0] + 0.05
        assert abs(status["pose"][2] - start_pose[2]) > 1.0
        # 50 Hz 状态帧的 source 与模式字段
        frame = recorder.last_frame()
        assert frame.source == "mock"
        assert (frame.basic, frame.gait, frame.motion) == (6, 2, 0)
        # 仿真激光：≤10 Hz 且点数受限
        assert recorder.scan_count() >= 5
        assert all(len(scan) <= 1500 for scan in recorder.scans)
    finally:
        transport.stop()
    assert transport.status()["connected"] is False


def test_mock_watchdog_stops_when_commands_stop():
    recorder = Recorder()
    transport = make_transport(recorder, {"zero_timeout_s": 0.15})
    transport.start()
    try:
        time.sleep(0.2)
        transport.send_velocity(0.4, 0.0)
        time.sleep(0.1)
        moving = transport.status()["velocity"][0]
        assert moving > 0.05
        # 不再下发指令，看门狗应把目标速度归零
        assert wait_until(lambda: transport.status()["velocity"][0] < 0.02, 1.5)
    finally:
        transport.stop()


def test_mock_link_disconnect_pauses_frames():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        assert wait_until(lambda: recorder.frame_count() >= 5, 1.0)
        transport.simulate(link=False)
        time.sleep(0.2)
        count = recorder.frame_count()
        time.sleep(0.3)
        assert recorder.frame_count() == count  # 断链期间不产生新帧
        assert transport.status()["connected"] is False
        assert "link_down" in recorder.codes()

        transport.simulate(link=True)
        time.sleep(0.3)
        assert recorder.frame_count() > count
        assert recorder.codes().count("link_up") >= 2
        assert transport.status()["connected"] is True
    finally:
        transport.stop()


def test_mock_joystick_takeover_cancels_autonomous_command():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        time.sleep(0.2)
        transport.send_velocity(0.4, 0.0)
        time.sleep(0.4)
        assert transport.status()["velocity"][0] > 0.1
        transport.simulate(joystick=[0.0, 0.0, 1.0])
        time.sleep(0.5)
        frame = recorder.last_frame()
        assert frame.joystick == pytest.approx((0.0, 0.0, 1.0))
        assert abs(frame.velocity_body[0]) < 0.15
        assert "joystick_takeover" in recorder.codes()
    finally:
        transport.stop()


def test_mock_collision_emits_event_and_stops():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        transport.reset(1.4, 2.5, 0.0)  # 柱子 (2.0, 2.5) 西侧，正对柱子
        time.sleep(0.1)
        def drive_until_collision():
            transport.send_velocity(0.6, 0.0)
            return "collision" in recorder.codes()

        assert wait_until(drive_until_collision, 2.0)
        status = transport.status()
        assert status["collisions"] >= 1
        assert status["pose"][0] < 1.6  # 碰撞后没有穿过柱子
    finally:
        transport.stop()


def test_mock_stop_is_idempotent_and_simulate_helpers():
    recorder = Recorder()
    transport = make_transport(recorder)
    # 未启动时 stop() 也必须安全
    transport.stop()
    transport.start()
    transport.start()  # 重复 start 不应起第二个线程
    try:
        time.sleep(0.2)
        snapshot = transport.simulate(link=None, joystick=None)
        assert snapshot["link"] is True
        snapshot = transport.reset(3.0, 1.0, 45.0)
        assert snapshot["pose"] == pytest.approx((3.0, 1.0, 45.0))
    finally:
        transport.stop()
        transport.stop()
    status = transport.status()
    assert status["connected"] is False


# ---------------------------------------------------------------------------
# 契约第 3.6 节：离线联调能力（arm / posture / emergency_stop）
# ---------------------------------------------------------------------------
def test_mock_capabilities_are_declared():
    transport = make_transport(Recorder())
    assert transport.capabilities == {"mapping": True, "arm": True, "posture": True,
                                      "vendor_estop": True, "sensors": False,
                                      "reconnect": False}
    status = transport.status()
    assert status["control"]["armed"] is False
    assert status["control"]["pending"] is None
    assert status["reconnect"]["enabled"] is False


def test_mock_arm_disarm_state_transition():
    transport = make_transport(Recorder(), {"arm_delay_s": 0.05,
                                            "disarm_delay_s": 0.01})
    transport.start()
    try:
        assert transport.status()["control"]["armed"] is False
        ok, message = transport.arm(timeout_s=1.0)
        assert ok is True and "使能" in message
        status = transport.status()
        assert status["control"]["armed"] is True
        assert status["control"]["mode_confirmed"] is True
        assert status["control"]["pending"] is None
        assert status["control"]["automatic_ack_age_s"] is not None

        ok, message = transport.disarm()
        assert ok is True and "解除" in message
        status = transport.status()
        assert status["control"]["armed"] is False
        assert status["control"]["mode_confirmed"] is False
    finally:
        transport.stop()


def test_mock_posture_transition_and_cancel():
    transport = make_transport(Recorder(), {"posture_delay_s": 0.6})
    transport.start()
    try:
        assert transport.status()["basic"] == 6
        # 站立 -> 趴下：0.6 s 后 basic 变 1，帧里的 basic 也要同步
        ok, message = transport.set_posture("lie", timeout_s=3.0)
        assert ok is True and "趴下" in message
        assert transport.status()["basic"] == 1
        assert wait_until(lambda: transport.status()["basic"] == 1, 1.0)
        frame = None
        # 等一帧确认 Frame.basic 跟随
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            frame = transport.status()
            if frame["basic"] == 1:
                break
            time.sleep(0.02)
        # cancel：起立等待中被中止，basic 保持 1
        cancel = threading.Event()
        result = {}
        worker = threading.Thread(
            target=lambda: result.update(value=transport.set_posture(
                "stand", cancel=cancel, timeout_s=3.0)))
        worker.start()
        time.sleep(0.1)
        cancel.set()
        worker.join(2.0)
        ok, message = result["value"]
        assert ok is False
        assert "取消" in message
        assert transport.status()["basic"] == 1
        # 再次正常起立
        assert transport.set_posture("stand", timeout_s=3.0)[0] is True
        assert transport.status()["basic"] == 6
    finally:
        transport.stop()


def test_mock_emergency_stop_enters_estop_without_changing_basic():
    recorder = Recorder()
    transport = make_transport(recorder, {"posture_delay_s": 0.05})
    transport.start()
    try:
        time.sleep(0.1)
        transport.send_velocity(0.3, 0.0)
        time.sleep(0.2)
        ok, message = transport.emergency_stop()
        assert ok is True
        assert "急停" in message
        status = transport.status()
        assert status["control"]["estop"] is True
        assert status["control"]["armed"] is False
        assert status["basic"] == 6  # basic 保持
        assert status["velocity"][0] == 0.0
        assert "vendor_estop" in recorder.codes()
        # 清除仿真急停标记后可继续联调
        transport.reset_estop()
        assert transport.status()["control"]["estop"] is False
    finally:
        transport.stop()


def test_mock_frames_expose_battery_and_error_fields():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        assert wait_until(lambda: recorder.frame_count() >= 2, 1.0)
        raw = recorder.last_frame().raw
        assert raw["battery_percent"] == pytest.approx(87.0)
        assert raw["battery_level_raw"] == pytest.approx(0.87)
        assert raw["error_state"] == 0
        assert raw["is_charging"] is False
    finally:
        transport.stop()


# ---------------------------------------------------------------------------
# 契约第 5.3 节：mock 渐进建图仿真（离线 E2E 依赖）
# ---------------------------------------------------------------------------
def _unknown_cells(grid) -> int:
    """GridMap 中 UNKNOWN(-1) 格的个数（内部按无符号字节存储为 255）。"""
    return sum(1 for value in grid.data if value > 127)


def _mapping_maps(recorder: Recorder) -> list:
    with recorder._lock:
        return [grid for grid in recorder.maps if grid.name == "map"]


def test_mock_mapping_capability_and_idle_status():
    transport = make_transport(Recorder())
    assert transport.capabilities["mapping"] is True
    status = transport.status()
    assert status["mapping"]["state"] == "idle"
    assert status["localization"]["valid"] is False
    # 未启动时不能启动建图
    ok, message = transport.set_mapping(True)
    assert ok is False and "未启动" in message


def test_mock_set_mapping_publishes_partial_unknown_mask():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        assert wait_until(lambda: len(recorder.maps) >= 1, 1.0)
        demo = recorder.maps[0]
        assert demo.name == "demo_lab"
        total = demo.width * demo.height

        ok, message = transport.set_mapping(True)
        assert ok is True and "建图" in message
        assert wait_until(lambda: len(_mapping_maps(recorder)) >= 1, 1.0)
        mask = _mapping_maps(recorder)[0]
        # 与 demo 地图同尺寸/同分辨率/同原点，但数据是独立的全未知掩膜
        assert (mask.width, mask.height) == (demo.width, demo.height)
        assert mask.resolution == pytest.approx(demo.resolution)
        assert mask.origin == pytest.approx(demo.origin)
        unknown = _unknown_cells(mask)
        assert 0 < unknown < total, "启动时应立即做一次局部披露"
        gx, gy = mask.world_to_index(0.0, 0.0)
        assert mask.is_free(gx, gy), "机器人起点周围应已被披露为 FREE"

        status = transport.status()
        assert status["mapping"]["state"] == "running"
        assert "建图" in status["mapping"]["message"]
        localization = status["localization"]
        assert localization["valid"] is True
        assert localization["source"] == "mock"
        assert localization["frame"] == "map"
        assert localization["age_s"] == 0.0
        assert localization["x"] == pytest.approx(status["pose"][0])
        assert localization["y"] == pytest.approx(status["pose"][1])
        assert localization["yaw_deg"] == pytest.approx(status["pose"][2])
        assert "mapping_started" in recorder.codes()
    finally:
        transport.stop()


def test_mock_mapping_mask_grows_and_pushes_copies():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        assert wait_until(lambda: len(recorder.maps) >= 1, 1.0)
        assert transport.set_mapping(True)[0] is True
        assert wait_until(lambda: len(_mapping_maps(recorder)) >= 1, 1.0)
        first = _mapping_maps(recorder)[0]
        first_unknown = _unknown_cells(first)
        first_data = bytes(first.data)

        # 机器人移动后掩膜应继续披露，未知格减少
        transport.send_velocity(0.5, 0.35)
        assert wait_until(lambda: len(_mapping_maps(recorder)) >= 3, 4.0)
        latest = _mapping_maps(recorder)[-1]
        assert latest is not first
        assert _unknown_cells(latest) < first_unknown
        # 已推送的掩膜是拷贝：内部继续更新不会写穿外部缓冲
        assert bytes(first.data) == first_data
        # 真实 demo 地图（碰撞/射线用）未被掩膜污染
        assert transport.status()["map"]["name"] == "demo_lab"
    finally:
        transport.stop()


def test_mock_localization_valid_tracks_mapping_state():
    """契约 §5.3：localization.valid == 建图是否运行（离线 E2E 的探索前置条件）。"""
    transport = make_transport(Recorder())
    assert transport.status()["localization"]["valid"] is False  # 未启动
    transport.start()
    try:
        status = transport.status()
        assert status["mapping"]["state"] == "idle"
        assert status["localization"]["valid"] is False

        assert transport.set_mapping(True)[0] is True
        status = transport.status()
        assert status["mapping"]["state"] == "running"
        assert status["localization"]["valid"] is True

        assert transport.set_mapping(False)[0] is True
        status = transport.status()
        assert status["mapping"]["state"] == "idle"
        assert status["localization"]["valid"] is False
        # 停止建图后仍提供当前仿真位姿（UI 徽标用），只是不再“有效”
        assert status["localization"]["x"] == pytest.approx(status["pose"][0])
        assert status["localization"]["y"] == pytest.approx(status["pose"][1])
        assert status["localization"]["yaw_deg"] == pytest.approx(status["pose"][2])
    finally:
        transport.stop()


def test_mock_set_mapping_stop_pushes_final_mask_and_resets_status():
    recorder = Recorder()
    transport = make_transport(recorder)
    transport.start()
    try:
        assert wait_until(lambda: len(recorder.maps) >= 1, 1.0)
        assert transport.set_mapping(True)[0] is True
        assert wait_until(lambda: len(_mapping_maps(recorder)) >= 1, 1.0)
        before = len(_mapping_maps(recorder))

        ok, message = transport.set_mapping(False)
        assert ok is True and "停止" in message
        assert len(_mapping_maps(recorder)) > before, "停止时应最后再推送一次掩膜"
        status = transport.status()
        assert status["mapping"]["state"] == "idle"
        assert status["localization"]["valid"] is False
        assert "mapping_stopped" in recorder.codes()
        # 重复停止幂等
        ok, _message = transport.set_mapping(False)
        assert ok is True
    finally:
        transport.stop()
    # 仿真停止后定位徽标必须置为无效
    assert transport.status()["localization"]["valid"] is False
