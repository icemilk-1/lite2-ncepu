"""厂商 UDP 传输回环测试：本机 127.0.0.1 自发自收，不依赖真实机器狗。

覆盖点：
* start() 在无实机环境绑定成功；
* 状态过期时 status() 仍 connected=True 但 stale=True，并产生限频告警；
* 196 字节状态包 -> Frame（source="udp"）；
* 60 字节遥控反馈包合并进后续帧的 joystick/goal_velocity；
* send_velocity 发出的 3 个速度包与协议逐字节一致；
* stop() 补发零速且可重复调用。
"""
import socket
import struct
import threading
import time

import pytest

from host_ui.app import protocol
from host_ui.app.transports.base import TransportCallbacks, TransportError
from host_ui.app.transports.vendor_udp import DEFAULT_OPTIONS, VendorUdpTransport


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

    def on_frame(self, frame) -> None:
        with self._lock:
            self.frames.append(frame)

    def on_event(self, level, code, message) -> None:
        with self._lock:
            self.events.append((level, code, message))

    def callbacks(self) -> TransportCallbacks:
        return TransportCallbacks(on_frame=self.on_frame, on_event=self.on_event)

    def frame_count(self) -> int:
        with self._lock:
            return len(self.frames)

    def codes(self) -> list[str]:
        with self._lock:
            return [code for _level, code, _message in self.events]


def state_packet(*, basic=6, gait=2, motion=1,
                 position=(1.5, -2.5, 0.1),
                 rpy=(1.0, 2.0, 30.0), battery=0.9,
                 error_state=0, is_charging=False) -> bytes:
    packet = bytearray(protocol.STATE_LENGTH)
    struct.pack_into("<3i", packet, 0, *protocol.STATE_HEADER)
    struct.pack_into("<2i", packet, 12, basic, gait)
    struct.pack_into("<18d", packet, 20,
                     *rpy, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0,
                     *position, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0)
    struct.pack_into("<I", packet, protocol.STATE_ERROR_OFFSET, error_state)
    struct.pack_into("<i", packet, protocol.STATE_MOTION_OFFSET, motion)
    struct.pack_into("<d", packet, protocol.STATE_BATTERY_OFFSET, battery)
    packet[protocol.STATE_CHARGING_OFFSET] = 1 if is_charging else 0
    return bytes(packet)


def command_packet(joystick=(0.1, -0.2, 0.0),
                   goal_velocity=(0.05, 0.0, -0.1)) -> bytes:
    packet = bytearray(protocol.COMMAND_LENGTH)
    struct.pack_into("<3i", packet, 0, *protocol.COMMAND_HEADER)
    struct.pack_into("<6d", packet, 12, *joystick, *goal_velocity)
    return bytes(packet)


class LoopbackRobot:
    """监听本机 UDP 端口，模拟机器狗接收速度包的一侧。"""

    def __init__(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(1.0)
        self.port = self.socket.getsockname()[1]

    def receive(self, count: int) -> list[bytes]:
        return [self.socket.recvfrom(2048)[0] for _ in range(count)]

    def drain(self, timeout: float = 0.3) -> list[bytes]:
        packets: list[bytes] = []
        self.socket.settimeout(timeout)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                packet, _ = self.socket.recvfrom(2048)
            except socket.timeout:
                break
            packets.append(packet)
        return packets

    def close(self) -> None:
        self.socket.close()


def make_transport(recorder: Recorder, robot: LoopbackRobot,
                   options: dict | None = None) -> VendorUdpTransport:
    merged = {
        "bind_host": "127.0.0.1",
        "bind_port": 0,
        "robot_ip": "127.0.0.1",
        "robot_port": robot.port,
        "zero_timeout_s": 5.0,
        "state_timeout_s": 0.2,
    }
    merged.update(options or {})
    return VendorUdpTransport(merged, recorder.callbacks())


def test_udp_loopback_state_and_velocity():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot)
    transport.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.settimeout(1.0)
    try:
        status = transport.status()
        assert status["connected"] is True
        assert status["bound_port"] > 0
        assert status["robot"] == f"127.0.0.1:{robot.port}"

        # 没有状态包时：connected 仍为 True，但 stale 且 detail 说明
        time.sleep(0.4)
        stale_status = transport.status()
        assert stale_status["connected"] is True
        assert stale_status["stale"] is True
        assert stale_status["state_age_s"] is None
        assert "尚未收到状态包" in stale_status["detail"]
        assert "state_timeout" in recorder.codes()

        # 196 字节状态包 -> Frame
        target = ("127.0.0.1", status["bound_port"])
        sender.sendto(state_packet(), target)
        assert wait_until(lambda: recorder.frame_count() >= 1)
        frame = recorder.frames[0]
        assert frame.source == "udp"
        assert frame.position == pytest.approx((1.5, -2.5, 0.1))
        assert frame.rpy_deg == pytest.approx((1.0, 2.0, 30.0))
        assert frame.velocity_body == pytest.approx((0.2, 0.0, 0.0))
        assert (frame.basic, frame.gait, frame.motion) == (6, 2, 1)
        assert frame.joystick == (0.0, 0.0, 0.0)
        assert transport.status()["stale"] is False

        # 60 字节遥控反馈包合并进后续帧
        sender.sendto(command_packet(), target)
        time.sleep(0.05)
        sender.sendto(state_packet(), target)
        assert wait_until(lambda: recorder.frame_count() >= 2)
        merged = recorder.frames[-1]
        assert merged.joystick == pytest.approx((0.1, -0.2, 0.0))
        assert merged.goal_velocity == pytest.approx((0.05, 0.0, -0.1))

        # 速度包与协议逐字节一致
        transport.send_velocity(0.25, -0.5)
        packets = robot.receive(3)
        assert packets == protocol.velocity_packets(0.25, -0.5)

        # 无法解析的数据包被丢弃并计数
        sender.sendto(b"\x00" * 10, target)
        assert wait_until(lambda: transport.status()["ignored_packets"] >= 1)
    finally:
        sender.close()
        transport.stop()
        robot.close()
    assert transport.status()["connected"] is False
    transport.stop()  # 幂等


def test_udp_stop_sends_zero_and_is_idempotent():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot)
    transport.start()
    try:
        transport.send_velocity(0.3, 0.2)
        packets = robot.receive(3)
        values = [struct.unpack("<3id", packet) for packet in packets]
        assert [item[0] for item in values] == [320, 325, 321]
        assert values[0][3] == pytest.approx(0.3)
        assert values[2][3] == pytest.approx(0.2)
        transport.stop()
        # stop 会补发零速，且不应再有非零速度包
        tail = robot.drain(0.2)
        assert tail, "stop() 应该补发零速包"
        for packet in tail:
            code, mark, kind, value = struct.unpack("<3id", packet)
            assert mark == 8 and kind == 1
            assert value == 0.0
    finally:
        transport.stop()  # 重复 stop 安全
        robot.close()
    assert transport.status()["connected"] is False


def test_udp_option_validation():
    recorder = Recorder()
    with pytest.raises(TransportError):
        VendorUdpTransport({"robot_port": 70000}, recorder.callbacks())
    with pytest.raises(TransportError):
        VendorUdpTransport({"bind_port": -1}, recorder.callbacks())
    assert set(DEFAULT_OPTIONS) == {
        "robot_ip", "robot_port", "bind_host", "bind_port",
        "ready_basic", "joystick_threshold", "zero_timeout_s", "state_timeout_s"}


# ---------------------------------------------------------------------------
# 契约第 3.4 节：电量/故障码、急停、姿态单次 toggle、反馈过期、status.control
# ---------------------------------------------------------------------------
def test_udp_frame_raw_battery_and_error_fields():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot)
    transport.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        target = ("127.0.0.1", transport.status()["bound_port"])
        sender.sendto(state_packet(battery=67.0, error_state=5,
                                   is_charging=True), target)
        assert wait_until(lambda: recorder.frame_count() >= 1)
        raw = recorder.frames[0].raw
        assert raw["battery_level_raw"] == pytest.approx(67.0)
        assert raw["battery_percent"] == pytest.approx(67.0)
        assert raw["error_state"] == 5
        assert raw["is_charging"] is True
        assert raw["motion_state"] == 1
        assert raw["zero_position_flag"] is False
    finally:
        sender.close()
        transport.stop()
        robot.close()


def test_udp_status_control_armed_follows_basic():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot, {"state_timeout_s": 2.0})
    transport.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        target = ("127.0.0.1", transport.status()["bound_port"])
        status = transport.status()
        assert status["control"]["armed"] is False
        assert status["control"]["depth_age_s"] is None
        assert status["reconnect"] == {"enabled": False, "attempts": 0,
                                       "next_retry_in_s": None}
        sender.sendto(state_packet(basic=6), target)
        assert wait_until(lambda: transport.status()["control"]["armed"] is True)
        sender.sendto(state_packet(basic=1), target)
        assert wait_until(lambda: transport.status()["control"]["armed"] is False)
    finally:
        sender.close()
        transport.stop()
        robot.close()


def test_udp_emergency_stop_sends_vendor_estop_three_times():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot)
    transport.start()
    try:
        ok, message = transport.emergency_stop()
        assert ok is True
        assert "0x21010C0E" in message
        packets = robot.drain(0.3)
        codes = [struct.unpack("<3i", packet)[0] for packet in packets]
        assert codes.count(protocol.ESTOP) == 3
    finally:
        transport.stop()
        robot.close()


def test_udp_posture_sends_single_toggle_and_polls_basic():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot,
                               {"state_timeout_s": 2.0, "zero_timeout_s": 5.0})
    transport.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        target = ("127.0.0.1", transport.status()["bound_port"])
        sender.sendto(state_packet(basic=1), target)
        assert wait_until(lambda: transport.status()["basic"] == 1)
        result = {}
        worker = threading.Thread(
            target=lambda: result.update(
                value=transport.set_posture("stand", timeout_s=3.0)))
        worker.start()
        toggles = 0
        deadline = time.monotonic() + 3.0
        robot.socket.settimeout(0.05)
        while worker.is_alive() and time.monotonic() < deadline:
            try:
                packet, _ = robot.socket.recvfrom(2048)
            except socket.timeout:
                continue
            if len(packet) != protocol.SIMPLE_LENGTH:
                continue  # 20 字节速度包（看门狗零速）不在本用例关注范围
            code = struct.unpack("<3i", packet)[0]
            if code == protocol.POSTURE_TOGGLE:
                toggles += 1
            # 模拟实机：收到 toggle 后切到站立
            sender.sendto(state_packet(basic=6), target)
        worker.join(2.0)
        ok, message = result["value"]
        assert ok is True
        assert "站立" in message
        assert toggles == 1  # 轮换指令只发一次，绝不连发
    finally:
        sender.close()
        transport.stop()
        robot.close()


def test_udp_posture_cancel_returns_immediately():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot,
                               {"state_timeout_s": 2.0, "zero_timeout_s": 5.0})
    transport.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        target = ("127.0.0.1", transport.status()["bound_port"])
        sender.sendto(state_packet(basic=1), target)
        assert wait_until(lambda: transport.status()["basic"] == 1)
        cancel = threading.Event()
        cancel.set()
        ok, message = transport.set_posture("stand", cancel=cancel,
                                            timeout_s=3.0)
        assert ok is False
        assert "取消" in message
    finally:
        sender.close()
        transport.stop()
        robot.close()


def test_udp_stale_joystick_feedback_is_zeroed():
    recorder = Recorder()
    robot = LoopbackRobot()
    transport = make_transport(recorder, robot,
                               {"feedback_timeout_s": 0.15,
                                "state_timeout_s": 2.0})
    transport.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        target = ("127.0.0.1", transport.status()["bound_port"])
        sender.sendto(command_packet(joystick=(0.5, -0.4, 0.0)), target)
        time.sleep(0.03)
        sender.sendto(state_packet(), target)
        assert wait_until(lambda: recorder.frame_count() >= 1)
        assert recorder.frames[0].joystick == pytest.approx((0.5, -0.4, 0.0))
        time.sleep(0.25)  # 超过 feedback_timeout_s
        sender.sendto(state_packet(), target)
        assert wait_until(lambda: recorder.frame_count() >= 2)
        assert recorder.frames[-1].joystick == pytest.approx((0.0, 0.0, 0.0))
        assert "joystick_stale" in recorder.codes()
    finally:
        sender.close()
        transport.stop()
        robot.close()