"""ROS1 适配层测试：纯函数 + 假 rospy 注入，本机没有 ROS 也能完整跑。

覆盖点：
* JSON 状态 -> Frame 字段映射与非法值丢弃；
* Twist 字段映射；
* OccupancyGrid -> GridMap；
* 四元数/里程计位姿换算；
* LaserScan -> 里程计坐标点集（过滤、限点）；
* 用假 rospy 模块走一遍 start()/订阅回调/发布/看门狗/stop()；
* 没有状态话题时退化为 odom 位姿帧；
* available_kinds() 在无 ROS 环境下的降级表现。
"""
import json
import math
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest

from host_ui.app.transports import ros1
from host_ui.app.transports.base import TransportCallbacks, TransportError
from host_ui.app.transports.ros1 import (
    Ros1Transport,
    apply_twist_fields,
    frame_from_state_payload,
    laser_scan_points,
    pose_from_odometry,
    rpy_deg_from_quaternion,
    twist_fields,
    yaw_deg_from_quaternion,
)


# ---------------------------------------------------------------------------
# 通用
# ---------------------------------------------------------------------------
def wait_until(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class Recorder:
    def __init__(self) -> None:
        self.frames = []
        self.events = []
        self.maps = []
        self.scans = []

    def on_frame(self, frame) -> None:
        self.frames.append(frame)

    def on_event(self, level, code, message) -> None:
        self.events.append((level, code, message))

    def on_map(self, grid) -> None:
        self.maps.append(grid)

    def on_scan(self, points) -> None:
        self.scans.append(list(points))

    def callbacks(self) -> TransportCallbacks:
        return TransportCallbacks(on_frame=self.on_frame, on_event=self.on_event,
                                  on_map=self.on_map, on_scan=self.on_scan)

    def codes(self) -> list[str]:
        return [code for _level, code, _message in self.events]


STATE_PAYLOAD = {
    "basic": 6,
    "gait": 2,
    "motion": 1,
    "rpy_deg": [1.0, -2.0, 30.0],
    "position": [1.0, 2.0, 0.3],
    "velocity_body": [0.1, 0.0, 0.0],
    "velocity_world": [0.08, 0.06, 0.0],
    "joystick": [0.0, 0.0, 0.0],
    "goal_velocity": [0.1, 0.0, 0.0],
}


# ---------------------------------------------------------------------------
# 纯函数：四元数 / 位姿 / Twist
# ---------------------------------------------------------------------------
def test_rpy_from_quaternion_identity():
    assert rpy_deg_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(
        (0.0, 0.0, 0.0))


def test_rpy_from_quaternion_yaw_90():
    half = math.radians(45.0)
    rpy = rpy_deg_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert rpy[2] == pytest.approx(90.0, abs=1e-9)
    assert yaw_deg_from_quaternion(0.0, 0.0, math.sin(half),
                                   math.cos(half)) == pytest.approx(90.0, abs=1e-9)
    # 零四元数不能除零，退化为 0
    assert rpy_deg_from_quaternion(0.0, 0.0, 0.0, 0.0) == pytest.approx(
        (0.0, 0.0, 0.0))


def test_pose_from_odometry():
    position = SimpleNamespace(x=1.5, y=-2.0, z=0.25)
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    pose, rpy = pose_from_odometry(position, orientation)
    assert pose == pytest.approx((1.5, -2.0, 0.25))
    assert rpy == pytest.approx((0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        pose_from_odometry(SimpleNamespace(x=1.0), orientation)
    with pytest.raises(ValueError):
        pose_from_odometry(SimpleNamespace(x=math.nan, y=0.0, z=0.0), orientation)


def test_twist_fields_and_apply():
    fields = twist_fields(0.2, -0.3)
    assert fields == {
        "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": -0.3},
    }
    message = SimpleNamespace(
        linear=SimpleNamespace(x=9.0, y=9.0, z=9.0),
        angular=SimpleNamespace(x=9.0, y=9.0, z=9.0),
    )
    apply_twist_fields(message, 0.4, 0.5)
    assert message.linear.x == pytest.approx(0.4)
    assert message.linear.y == 0.0
    assert message.linear.z == 0.0
    assert message.angular.x == 0.0
    assert message.angular.y == 0.0
    assert message.angular.z == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# 纯函数：状态 JSON -> Frame
# ---------------------------------------------------------------------------
def test_frame_from_state_payload():
    frame = frame_from_state_payload(STATE_PAYLOAD, 123.5)
    assert frame is not None
    assert frame.source == "ros"
    assert frame.stamp == pytest.approx(123.5)
    assert frame.position == pytest.approx((1.0, 2.0, 0.3))
    assert frame.rpy_deg == pytest.approx((1.0, -2.0, 30.0))
    assert frame.velocity_body == pytest.approx((0.1, 0.0, 0.0))
    assert frame.velocity_world == pytest.approx((0.08, 0.06, 0.0))
    assert frame.joystick == pytest.approx((0.0, 0.0, 0.0))
    assert frame.goal_velocity == pytest.approx((0.1, 0.0, 0.0))
    assert (frame.basic, frame.gait, frame.motion) == (6, 2, 1)


def test_frame_from_state_payload_defaults_and_wrap():
    minimal = {"position": [1.0, 2.0, 0.0], "rpy_deg": [0.0, 0.0, 350.0]}
    frame = frame_from_state_payload(minimal, 0.0)
    assert frame is not None
    assert frame.velocity_body == (0.0, 0.0, 0.0)
    assert frame.velocity_world == (0.0, 0.0, 0.0)
    assert frame.joystick == (0.0, 0.0, 0.0)
    assert frame.goal_velocity == (0.0, 0.0, 0.0)
    assert (frame.basic, frame.gait, frame.motion) == (0, 0, 0)
    assert frame.rpy_deg[2] == pytest.approx(-10.0)


def test_frame_from_state_payload_rejects_bad_input():
    assert frame_from_state_payload(None, 0.0) is None
    assert frame_from_state_payload(["not", "a", "dict"], 0.0) is None
    assert frame_from_state_payload({"position": [1.0, 2.0]}, 0.0) is None
    assert frame_from_state_payload(
        {"position": [1.0, 2.0, 0.0], "rpy_deg": [0.0, 0.0, math.nan]}, 0.0) is None
    assert frame_from_state_payload(
        {"position": [1.0, 2.0, math.inf], "rpy_deg": [0.0, 0.0, 0.0]}, 0.0) is None


# ---------------------------------------------------------------------------
# 纯函数：LaserScan
# ---------------------------------------------------------------------------
def test_laser_scan_points_identity_pose():
    points = laser_scan_points([1.0, 2.0, 3.0], 0.0, math.pi / 2.0, (1.0, 2.0, 0.0))
    assert points[0] == pytest.approx((2.0, 2.0))
    assert points[1] == pytest.approx((1.0, 4.0))
    assert points[2] == pytest.approx((-2.0, 2.0))


def test_laser_scan_points_filters_and_caps():
    ranges = [math.nan, math.inf, 0.0, 0.01, 0.05, 10.0, 10.1]
    points = laser_scan_points(ranges, 0.0, 0.1, (0.0, 0.0, 0.0),
                               max_range=10.0)
    assert len(points) == 2  # 只保留 0.05 与 10.0（边界包含）
    capped = laser_scan_points([1.0] * 2000, 0.0, 0.01, (0.0, 0.0, 0.0),
                               max_points=1500)
    assert len(capped) == 1500


# ---------------------------------------------------------------------------
# 无 ROS 环境：available() 与 start() 的降级行为
# ---------------------------------------------------------------------------
def test_ros_available_false_without_ros():
    if Ros1Transport.available():
        pytest.skip("本机已安装 ROS，跳过无 ROS 场景")
    with pytest.raises(TransportError) as error:
        Ros1Transport({}, Recorder().callbacks()).start()
    assert "ROS" in str(error.value)
    assert "source /opt/ros" in str(error.value)


def test_available_kinds_exposes_mock_udp_ros():
    from host_ui.app.transports import available_kinds

    entries = {item["kind"]: item for item in available_kinds()}
    assert {"mock", "udp", "ros"} <= set(entries)
    assert entries["mock"]["available"] is True
    assert entries["udp"]["available"] is True
    assert entries["mock"]["options"]
    if not Ros1Transport.available():
        assert entries["ros"]["available"] is False


# ---------------------------------------------------------------------------
# 假 rospy 注入
# ---------------------------------------------------------------------------
def _build_fake_ros():
    """构造最小可用的假 rospy 以及 std_msgs/geometry_msgs/nav_msgs/sensor_msgs。"""
    state = SimpleNamespace(init_calls=0, subscribers=[], publishers=[],
                            wait_calls=[], service_calls=[],
                            service_responses={})

    class FakeTwist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class FakeString:
        def __init__(self, data=""):
            self.data = data

    class FakeOccupancyGrid:
        pass

    class FakeOdometry:
        pass

    class FakeLaserScan:
        pass

    class FakeTrigger:
        pass

    class FakeServiceProxy:
        def __init__(self, service, service_type):
            self.service = service
            self.service_type = service_type

        def __call__(self):
            state.service_calls.append(self.service)
            success, message = state.service_responses.get(
                self.service, (True, "OK"))
            return SimpleNamespace(success=success, message=message)

    class FakePublisher:
        def __init__(self, topic, message_type, queue_size=10, **kwargs):
            self.topic = topic
            self.message_type = message_type
            self.messages = []
            self.unregistered = False
            state.publishers.append(self)

        def publish(self, message):
            self.messages.append(message)

        def unregister(self):
            self.unregistered = True

    class FakeSubscriber:
        def __init__(self, topic, message_type, callback, queue_size=10, **kwargs):
            self.topic = topic
            self.message_type = message_type
            self.callback = callback
            self.unregistered = False
            state.subscribers.append(self)

        def unregister(self):
            self.unregistered = True

    def fake_init_node(*args, **kwargs):
        state.init_calls += 1

    rospy_module = ModuleType("rospy")
    rospy_module.init_node = fake_init_node
    rospy_module.Publisher = FakePublisher
    rospy_module.Subscriber = FakeSubscriber
    rospy_module.is_shutdown = lambda: False

    def fake_wait_for_service(service, timeout=None):
        state.wait_calls.append(service)

    rospy_module.wait_for_service = fake_wait_for_service
    rospy_module.ServiceProxy = FakeServiceProxy

    packages = {
        "std_msgs": {"String": FakeString},
        "geometry_msgs": {"Twist": FakeTwist},
        "nav_msgs": {"OccupancyGrid": FakeOccupancyGrid, "Odometry": FakeOdometry},
        "sensor_msgs": {"LaserScan": FakeLaserScan},
    }
    modules = {"rospy": rospy_module}
    for package_name, attributes in packages.items():
        parent = ModuleType(package_name)
        child = ModuleType(f"{package_name}.msg")
        for attribute_name, value in attributes.items():
            setattr(child, attribute_name, value)
        parent.msg = child
        modules[package_name] = parent
        modules[f"{package_name}.msg"] = child
    # std_srvs.srv.Trigger（arm/disarm 用）
    std_srvs = ModuleType("std_srvs")
    srv = ModuleType("std_srvs.srv")
    srv.Trigger = FakeTrigger
    std_srvs.srv = srv
    modules["std_srvs"] = std_srvs
    modules["std_srvs.srv"] = srv
    return state, modules


@pytest.fixture
def fake_ros(monkeypatch):
    state, modules = _build_fake_ros()
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return state


def _find_subscriber(state, topic):
    return next(item for item in state.subscribers if item.topic == topic)


def _odometry_message(x, y, yaw_deg):
    half = math.radians(yaw_deg) / 2.0
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half),
                                    w=math.cos(half)))))


def _occupancy_grid_message():
    return SimpleNamespace(
        info=SimpleNamespace(
            width=2, height=1, resolution=0.5,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[-1, 100])


def test_transport_with_fake_rospy(fake_ros):
    recorder = Recorder()
    transport = Ros1Transport(
        {"zero_timeout_s": 0.05, "state_timeout_s": 0.2}, recorder.callbacks())
    transport.start()
    try:
        assert fake_ros.init_calls >= 1
        assert transport.status()["connected"] is True
        assert transport.status()["topics"]["state"] == "/airace/state"
        state_sub = _find_subscriber(fake_ros, "/airace/state")
        publisher = fake_ros.publishers[0]
        assert publisher.topic == "/airace/cmd_vel"

        # JSON 状态 -> Frame
        state_sub.callback(SimpleNamespace(data=json.dumps(STATE_PAYLOAD)))
        assert len(recorder.frames) == 1
        frame = recorder.frames[-1]
        assert frame.source == "ros"
        assert frame.position == pytest.approx((1.0, 2.0, 0.3))
        assert "state_up" in recorder.codes()

        # 速度 -> Twist 字段
        transport.send_velocity(0.2, -0.3)
        message = publisher.messages[-1]
        assert message.linear.x == pytest.approx(0.2)
        assert message.linear.y == 0.0
        assert message.angular.z == pytest.approx(-0.3)

        # 零速看门狗
        def zero_after_nonzero():
            indices = [index for index, item in enumerate(publisher.messages)
                       if item.linear.x != 0.0]
            return bool(indices) and any(
                item.linear.x == 0.0 and item.angular.z == 0.0
                for item in publisher.messages[indices[-1] + 1:])

        assert wait_until(zero_after_nonzero, 1.5)

        # OccupancyGrid -> GridMap
        _find_subscriber(fake_ros, "/map").callback(_occupancy_grid_message())
        assert recorder.maps
        grid = recorder.maps[-1]
        assert (grid.width, grid.height) == (2, 1)
        assert grid.resolution == pytest.approx(0.5)
        assert grid.origin == pytest.approx((1.0, 2.0, 0.0))
        assert grid.at(0, 0) == -1
        assert grid.at(1, 0) == 100
        assert "map_up" in recorder.codes()

        # odom 位姿 + LaserScan -> 里程计坐标点
        _find_subscriber(fake_ros, "/odom").callback(
            _odometry_message(1.0, 2.0, 90.0))
        _find_subscriber(fake_ros, "/scan").callback(SimpleNamespace(
            ranges=[1.0, float("nan"), 2.0], angle_min=0.0,
            angle_increment=math.pi / 2.0, range_min=0.05, range_max=10.0))
        assert recorder.scans
        points = recorder.scans[-1]
        assert len(points) == 2
        assert points[0] == pytest.approx((1.0, 3.0))
        assert points[1] == pytest.approx((1.0, 0.0))
    finally:
        transport.stop()
    assert transport.status()["connected"] is False
    assert publisher.unregistered is True
    assert all(item.unregistered for item in fake_ros.subscribers)


def test_odom_fallback_frames_without_state(fake_ros):
    recorder = Recorder()
    transport = Ros1Transport(
        {"zero_timeout_s": 0.5, "state_timeout_s": 0.3}, recorder.callbacks())
    transport.start()
    try:
        odom_sub = _find_subscriber(fake_ros, "/odom")
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            odom_sub.callback(_odometry_message(1.0, 2.0, 90.0))
            time.sleep(0.05)
        fallback_frames = [frame for frame in recorder.frames
                           if frame.raw.get("fallback") == "odom"]
        assert fallback_frames
        frame = fallback_frames[-1]
        assert frame.source == "ros"
        assert frame.position == pytest.approx((1.0, 2.0, 0.0))
        assert frame.rpy_deg[2] == pytest.approx(90.0)
        assert "odom_fallback" in recorder.codes()
    finally:
        transport.stop()


# ---------------------------------------------------------------------------
# 契约第 3.5 节：电量归一、status.connected、capabilities.arm、Trigger 使能
# ---------------------------------------------------------------------------
def test_frame_from_state_payload_battery_normalized_and_error_passthrough():
    percent_payload = dict(STATE_PAYLOAD, battery_level_raw=0.67,
                           error_state=3, is_charging=True,
                           zero_position_flag=True, motion_state=1)
    frame = frame_from_state_payload(percent_payload, 0.0)
    assert frame.raw["battery_level_raw"] == pytest.approx(0.67)
    assert frame.raw["battery_percent"] == pytest.approx(67.0)
    assert frame.raw["error_state"] == 3
    assert frame.raw["is_charging"] is True
    assert frame.raw["zero_position_flag"] is True
    assert frame.raw["motion_state"] == 1

    # 实测固件直接给百分数（>1.5 视为已经是百分数）
    direct = frame_from_state_payload(
        dict(STATE_PAYLOAD, battery_level_raw=59.0), 0.0)
    assert direct.raw["battery_percent"] == pytest.approx(59.0)

    # 旧字段名 battery_level 也兼容
    legacy = frame_from_state_payload(
        dict(STATE_PAYLOAD, battery_level=0.5), 0.0)
    assert legacy.raw["battery_percent"] == pytest.approx(50.0)


def test_ros_capabilities_and_connected_semantics(fake_ros):
    recorder = Recorder()
    transport = Ros1Transport({}, recorder.callbacks())
    assert transport.capabilities["arm"] is True
    assert transport.capabilities["posture"] is False
    assert transport.status()["connected"] is False  # 未 start()
    transport.start()
    try:
        status = transport.status()
        assert status["connected"] is True
        assert status["control"]["armed"] is None
        assert status["reconnect"] == {"enabled": False, "attempts": 0,
                                       "next_retry_in_s": None}
    finally:
        transport.stop()
    assert transport.status()["connected"] is False  # 已 stop()


def test_ros_arm_disarm_via_trigger_service(fake_ros):
    recorder = Recorder()
    transport = Ros1Transport({}, recorder.callbacks())
    transport.start()
    try:
        fake_ros.service_responses["/airace/prepare"] = (True, "自动模式确认")
        ok, message = transport.arm(timeout_s=2.0)
        assert ok is True
        assert message == "自动模式确认"
        assert fake_ros.service_calls == ["/airace/prepare"]
        assert transport.status()["control"]["armed"] is True
        assert transport.status()["control"]["automatic_ack_age_s"] is not None

        fake_ros.service_responses["/airace/disarm"] = (True, "已退出自动模式")
        ok, message = transport.disarm(timeout_s=2.0)
        assert ok is True
        assert transport.status()["control"]["armed"] is False
    finally:
        transport.stop()


def test_ros_arm_service_failure_returns_chinese(fake_ros):
    recorder = Recorder()
    transport = Ros1Transport({}, recorder.callbacks())
    transport.start()
    try:
        fake_ros.service_responses["/airace/prepare"] = (
            False, "No automatic-mode acknowledgement")
        ok, message = transport.arm(timeout_s=2.0)
        assert ok is False
        assert "No automatic-mode acknowledgement" in message
        assert transport.status()["control"]["armed"] is None
        assert "No automatic-mode acknowledgement" in transport.status()[
            "control"]["last_error"]
    finally:
        transport.stop()