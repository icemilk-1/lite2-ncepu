"""厂商 UDP 协议 V1.2.3 编解码回归测试（纯离线，无网络）。"""
import math
import struct

import pytest

from host_ui.app import protocol


# ---------------------------------------------------------------------------
# 测试用封包构造（与 lite2_sim/hardware/vendor_protocol.py 的布局一致）
# ---------------------------------------------------------------------------
def state_packet(*, basic: int = 6, gait: int = 2, motion: int = 1,
                 rpy=(1.0, 2.0, 3.0), angular=(0.1, 0.2, 0.3),
                 position=(1.5, -2.5, 0.1),
                 velocity_world=(0.2, 0.0, 0.0),
                 velocity_body=(0.2, 0.0, 0.0),
                 battery: float = 0.67, error_state: int = 0,
                 is_charging: bool = False, zero_position: bool = False,
                 touch: int = 0, task_state: int = 0,
                 need_move: bool = False) -> bytes:
    packet = bytearray(protocol.STATE_LENGTH)
    struct.pack_into("<3i", packet, 0, *protocol.STATE_HEADER)
    struct.pack_into("<2i", packet, 12, int(basic), int(gait))
    values = (*rpy, *angular, 0.0, 0.0, 0.0, *position,
              *velocity_world, *velocity_body)
    assert len(values) == 18
    struct.pack_into("<18d", packet, 20, *values)
    struct.pack_into("<I", packet, protocol.STATE_TOUCH_OFFSET, int(touch))
    packet[protocol.STATE_CHARGING_OFFSET] = 1 if is_charging else 0
    struct.pack_into("<I", packet, protocol.STATE_ERROR_OFFSET, int(error_state))
    struct.pack_into("<i", packet, protocol.STATE_MOTION_OFFSET, int(motion))
    struct.pack_into("<d", packet, protocol.STATE_BATTERY_OFFSET, float(battery))
    struct.pack_into("<i", packet, protocol.STATE_TASK_OFFSET, int(task_state))
    packet[protocol.STATE_NEED_MOVE_OFFSET] = 1 if need_move else 0
    packet[protocol.STATE_ZERO_POSITION_OFFSET] = 1 if zero_position else 0
    return bytes(packet)


def command_packet(values=(0.1, -0.2, 0.0, 0.05, 0.0, -0.1)) -> bytes:
    packet = bytearray(protocol.COMMAND_LENGTH)
    struct.pack_into("<3i", packet, 0, *protocol.COMMAND_HEADER)
    struct.pack_into("<6d", packet, 12, *values)
    return bytes(packet)


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
def test_wire_constants():
    assert protocol.STATE_LENGTH == 196
    assert protocol.COMMAND_LENGTH == 60
    assert protocol.VELOCITY_LENGTH == 20
    assert protocol.STATE_HEADER == (2305, 184, 1)
    assert protocol.COMMAND_HEADER == (2309, 48, 1)
    assert protocol.VELOCITY_CODES == (320, 325, 321)


# ---------------------------------------------------------------------------
# 状态包
# ---------------------------------------------------------------------------
def test_decode_state_roundtrip():
    packet = state_packet(basic=6, gait=2, motion=3,
                          rpy=(1.5, -2.0, 170.0), angular=(0.1, 0.2, 0.3),
                          position=(1.5, -2.5, 0.1),
                          velocity_world=(0.11, 0.22, 0.33),
                          velocity_body=(0.44, 0.55, 0.66))
    report = protocol.decode_state(packet)
    assert report is not None
    assert report["basic"] == 6
    assert report["gait"] == 2
    assert report["motion"] == 3
    assert report["rpy_deg"] == pytest.approx([1.5, -2.0, 170.0])
    assert report["angular"] == pytest.approx([0.1, 0.2, 0.3])
    assert report["position"] == pytest.approx([1.5, -2.5, 0.1])
    assert report["velocity_world"] == pytest.approx([0.11, 0.22, 0.33])
    assert report["velocity_body"] == pytest.approx([0.44, 0.55, 0.66])
    assert isinstance(report["position"], list)


def test_decode_state_accepts_memoryview():
    packet = state_packet()
    assert protocol.decode_state(memoryview(packet)) is not None


@pytest.mark.parametrize("bad_length", [0, 12, 195, 197, 400])
def test_decode_state_rejects_wrong_length(bad_length):
    packet = bytearray(b"\x00" * bad_length)
    assert protocol.decode_state(bytes(packet)) is None


def test_decode_state_rejects_wrong_header():
    packet = bytearray(state_packet())
    struct.pack_into("<3i", packet, 0, 2305, 184, 2)
    assert protocol.decode_state(bytes(packet)) is None
    packet = bytearray(state_packet())
    struct.pack_into("<3i", packet, 0, 2306, 184, 1)
    assert protocol.decode_state(bytes(packet)) is None


def test_decode_state_rejects_non_finite_values():
    assert protocol.decode_state(state_packet(rpy=(math.nan, 0.0, 0.0))) is None
    assert protocol.decode_state(state_packet(position=(math.inf, 0.0, 0.0))) is None
    assert protocol.decode_state(
        state_packet(velocity_body=(0.0, 0.0, math.nan))) is None


def test_decode_state_rejects_out_of_range_values():
    # 姿态角绝对值超过 180 度
    assert protocol.decode_state(state_packet(rpy=(0.0, 0.0, 181.0))) is None
    assert protocol.decode_state(state_packet(rpy=(-181.0, 0.0, 0.0))) is None
    # 世界系/本体系速度超过 5 m/s
    assert protocol.decode_state(
        state_packet(velocity_world=(5.1, 0.0, 0.0))) is None
    assert protocol.decode_state(
        state_packet(velocity_body=(0.0, -5.5, 0.0))) is None
    # 边界值本身合法
    assert protocol.decode_state(
        state_packet(rpy=(180.0, 0.0, 0.0))) is not None
    assert protocol.decode_state(
        state_packet(velocity_world=(5.0, 0.0, 0.0))) is not None


def test_decode_state_rejects_non_bytes():
    assert protocol.decode_state("not a packet") is None
    assert protocol.decode_state(None) is None


# ---------------------------------------------------------------------------
# 速度包
# ---------------------------------------------------------------------------
def test_velocity_packets_exact_bytes():
    packets = protocol.velocity_packets(0.25, -0.5)
    assert packets == [
        struct.pack("<3id", 320, 8, 1, 0.25),
        struct.pack("<3id", 325, 8, 1, 0.0),
        struct.pack("<3id", 321, 8, 1, -0.5),
    ]
    for packet in packets:
        assert len(packet) == protocol.VELOCITY_LENGTH
    assert [struct.unpack("<3id", packet)[0] for packet in packets] == [320, 325, 321]


def test_velocity_packets_reject_non_finite():
    with pytest.raises(ValueError):
        protocol.velocity_packets(math.nan, 0.0)
    with pytest.raises(ValueError):
        protocol.velocity_packets(0.0, math.inf)


def test_safety_packets_is_zero_velocity():
    assert protocol.safety_packets() == protocol.velocity_packets(0.0, 0.0)
    for packet in protocol.safety_packets():
        code, mark, kind, value = struct.unpack("<3id", packet)
        assert mark == 8 and kind == 1
        assert value == 0.0
        assert code in (320, 325, 321)


# ---------------------------------------------------------------------------
# 遥控反馈包
# ---------------------------------------------------------------------------
def test_decode_command_roundtrip():
    report = protocol.decode_command(
        command_packet((0.1, -0.2, 0.3, 0.05, 0.0, -0.1)))
    assert report is not None
    assert report["joystick"] == pytest.approx([0.1, -0.2, 0.3])
    assert report["goal_velocity"] == pytest.approx([0.05, 0.0, -0.1])


@pytest.mark.parametrize("bad_length", [0, 20, 59, 61, 196])
def test_decode_command_rejects_wrong_length(bad_length):
    assert protocol.decode_command(b"\x00" * bad_length) is None


def test_decode_command_rejects_wrong_header():
    packet = bytearray(command_packet())
    struct.pack_into("<3i", packet, 0, 2309, 48, 2)
    assert protocol.decode_command(bytes(packet)) is None
    packet = bytearray(command_packet())
    struct.pack_into("<i", packet, 0, 2308)
    assert protocol.decode_command(bytes(packet)) is None


def test_decode_command_rejects_non_finite():
    assert protocol.decode_command(
        command_packet((math.nan, 0.0, 0.0, 0.0, 0.0, 0.0))) is None
    assert protocol.decode_command(
        command_packet((0.0, 0.0, 0.0, 0.0, math.inf, 0.0))) is None
    assert protocol.decode_command("bad") is None


# ---------------------------------------------------------------------------
# 厂商指令常量与简单指令（契约第 2 / 3.1 节）
# ---------------------------------------------------------------------------
def test_vendor_command_constants():
    assert protocol.HEARTBEAT == 0x21040001
    assert protocol.ESTOP == 0x21010C0E
    assert protocol.POSTURE_TOGGLE == 0x21010202
    assert protocol.MOVE_MODE == 0x21010D06
    assert protocol.STAND_MODE == 0x21010D05
    assert protocol.MANUAL_MODE == 0x21010C02
    assert protocol.AUTO_MODE == 0x21010C03
    assert protocol.CONTINUOUS_MOTION == 0x21010C06
    assert protocol.ZERO_POSITION == 0x21010C05
    assert protocol.SAVE_DATA == 0x21010C01
    assert protocol.AXIS_FORWARD == 0x21010130
    assert protocol.AXIS_LATERAL == 0x21010131
    assert protocol.AXIS_TURN == 0x21010135
    assert protocol.AXIS_VERTICAL == 0x21010102
    # 附录状态值
    assert (protocol.BASIC_LIE, protocol.BASIC_FORCE_CONTROL) == (1, 6)


def test_simple_packet_layout_and_estop_bytes():
    packet = protocol.simple_packet(0x21010C0E, 0)
    assert packet == struct.pack("<3i", 0x21010C0E, 0, 0)
    assert len(packet) == protocol.SIMPLE_LENGTH == 12
    assert protocol.simple_packet(0x21010C06, -1) == struct.pack(
        "<3i", 0x21010C06, -1, 0)
    assert protocol.estop_packet() == protocol.simple_packet(protocol.ESTOP)
    assert protocol.heartbeat_packet() == protocol.simple_packet(
        protocol.HEARTBEAT)
    assert protocol.posture_packet() == protocol.simple_packet(
        protocol.POSTURE_TOGGLE)


# ---------------------------------------------------------------------------
# 196B 状态包后半段字段（契约第 2 节绝对偏移）
# ---------------------------------------------------------------------------
def test_decode_state_battery_and_error_fields():
    report = protocol.decode_state(state_packet(
        battery=0.67, error_state=7, is_charging=True,
        zero_position=True, touch=42, task_state=3, need_move=True))
    assert report is not None
    assert report["battery_level_raw"] == pytest.approx(0.67)
    assert report["battery_percent"] == pytest.approx(67.0)
    assert report["error_state"] == 7
    assert report["is_charging"] is True
    assert report["zero_position_flag"] is True
    assert report["touch_down_and_stair_trot"] == 42
    assert report["task_state"] == 3
    assert report["is_robot_need_move"] is True


@pytest.mark.parametrize("raw,expected", [(0.0, 0.0), (0.59, 59.0),
                                          (0.87, 87.0), (67.0, 67.0),
                                          (59.0, 59.0), (100.0, 100.0)])
def test_decode_state_battery_normalization(raw, expected):
    report = protocol.decode_state(state_packet(battery=raw))
    assert report["battery_level_raw"] == pytest.approx(raw)
    assert report["battery_percent"] == pytest.approx(expected)


def test_normalize_battery_level_rejects_bad_values():
    assert protocol.normalize_battery_level(None) is None
    assert protocol.normalize_battery_level("bad") is None
    assert protocol.normalize_battery_level(-1.0) is None
    assert protocol.normalize_battery_level(math.nan) is None
    # 超过 100 的百分数被收敛到 100
    assert protocol.normalize_battery_level(120.0) == pytest.approx(100.0)
    # 1.5 以内视为 0~1 比例（文档语义）
    assert protocol.normalize_battery_level(1.0) == pytest.approx(100.0)


def test_posture_targets():
    assert protocol.posture_target("stand") == 6
    assert protocol.posture_target("lie") == 1
    assert protocol.posture_target("fly") is None


def test_decode_state_offset_alignment_raw():
    """用绝对偏移写字段，确认 164/168/172/176/180/188/192/193 对齐。"""
    packet = bytearray(protocol.STATE_LENGTH)
    struct.pack_into("<3i", packet, 0, *protocol.STATE_HEADER)
    struct.pack_into("<2i", packet, 12, 1, 0)
    struct.pack_into("<18d", packet, 20, *([0.0] * 18))
    struct.pack_into("<I", packet, 164, 0xDEADBEEF & 0xFFFFFFFF)
    packet[168] = 1
    struct.pack_into("<I", packet, 172, 99)
    struct.pack_into("<i", packet, 176, 2)
    struct.pack_into("<d", packet, 180, 0.42)
    struct.pack_into("<i", packet, 188, 5)
    packet[192] = 1
    packet[193] = 1
    report = protocol.decode_state(bytes(packet))
    assert report["touch_down_and_stair_trot"] == 0xDEADBEEF
    assert report["is_charging"] is True
    assert report["error_state"] == 99
    assert report["motion"] == 2
    assert report["battery_percent"] == pytest.approx(42.0)
    assert report["task_state"] == 5
    assert report["is_robot_need_move"] is True
    assert report["zero_position_flag"] is True


# ---------------------------------------------------------------------------
# decode_any：区分 196 状态 / 60 遥控反馈 / 12 简单指令 ACK
# ---------------------------------------------------------------------------
def test_decode_any_distinguishes_packet_types():
    state = protocol.decode_any(state_packet())
    assert state["kind"] == "state"
    assert state["type"] == 1
    assert state["code"] == 2305
    assert state["basic"] == 6
    assert state["battery_percent"] == pytest.approx(67.0)

    command = protocol.decode_any(command_packet())
    assert command["kind"] == "command"
    assert command["joystick"] == pytest.approx([0.1, -0.2, 0.0])
    assert command["goal_velocity"] == pytest.approx([0.05, 0.0, -0.1])

    simple = protocol.decode_any(protocol.estop_packet())
    assert simple == {"kind": "simple", "type": 0,
                      "code": protocol.ESTOP, "value": 0}

    assert protocol.decode_any(b"\x00" * 7) is None
    assert protocol.decode_any("not bytes") is None


def test_velocity_packets_unchanged_by_new_fields():
    """新增状态字段不能改变速度包逐字节内容（旧固件兼容）。"""
    assert protocol.velocity_packets(0.25, -0.5) == [
        struct.pack("<3id", 320, 8, 1, 0.25),
        struct.pack("<3id", 325, 8, 1, 0.0),
        struct.pack("<3id", 321, 8, 1, -0.5),
    ]