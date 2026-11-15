"""部署源码级回归：机载时钟源、控制会话/姿态服务、诊断字段不能回退。

这些检查不启动 ROS，只锁定关键实现特征，防止后续修改把"必须同源的单调时钟"
或"单一控制会话"改回去。
"""
from __future__ import annotations

from pathlib import Path
import re

from host_ui.tests.test_bridge_session import bridge_session

ROOT = Path(__file__).resolve().parents[2]
STREAM = ROOT / "host_ui" / "app" / "transports" / "ssh_ros_stream.py"
BRIDGE = ROOT / "lite2_sim" / "hardware" / "lite2_live.py"
LAUNCH = ROOT / "lite2_sim" / "hardware" / "navigation_mapping.launch"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_stream_uses_kernel_monotonic_and_forbids_wall_clock() -> None:
    source = read(STREAM)
    assert "clock_gettime(1" in source, "机载 py2.7 必须用 CLOCK_MONOTONIC"
    assert "raise RuntimeError(" in source
    assert "拒绝启动" in source, "时钟不可用必须显式报错，不能静默退化"
    # 旧实现里的墙钟兜底不允许再出现
    assert "monotonic = time.time" not in source


def test_stream_python27_compatible_helpers() -> None:
    source = read(STREAM)
    # Python 2.7 没有 math.isfinite；必须用自定义 _finite
    assert "def _finite(value)" in source
    assert "math.isfinite(" not in source.replace(
        '"""Python 2.7 没有 math.isfinite（3.2+ 才加入）。"""', "")
    # 姿态/急停必须走机载服务，不再从 Windows 直发模式包
    assert '"/airace/stand"' in source
    assert '"/airace/lie"' in source
    assert '"/airace/estop"' in source


def test_bridge_owns_single_control_session_and_services() -> None:
    source = read(BRIDGE)
    assert 'rospy.Service("/airace/stand"' in source
    assert 'rospy.Service("/airace/lie"' in source
    assert 'rospy.Service("/airace/estop"' in source
    assert 'rospy.Service("/airace/acquire"' in source
    assert "CONTROL_HEARTBEAT_PERIOD_S = 0.2" in source
    assert "def heartbeat_loop" in source
    assert "control_session" in source
    assert "depth_fresh" in source
    assert "def stop_session" in source


def posture_service_fixture(on_sleep=None):
    """运行生产姿态服务闭包，替换 ROS 和时钟以复现延迟及断流。"""
    def sleep(world, seconds):
        if on_sleep:
            on_sleep(world.clock, world.state, world.session)

    return bridge_session(on_sleep=sleep)


def test_posture_timeout_sends_toggle_once_and_includes_warmup():
    world = posture_service_fixture()
    response = world.stand(None)
    assert not response.success
    assert world.codes().count(0x21010202) == 1
    assert 20.0 <= world.clock.now - 100.0 < 20.1
    assert "控制权" in response.message
    assert world.session["active"]


def test_posture_rechecks_state_after_warmup():
    def stale(clock, state, session):
        state["received"] = clock.now - 1.0

    world = posture_service_fixture(stale)
    assert not world.stand(None).success
    assert 0x21010202 not in world.codes()


def test_posture_cancelled_session_does_not_send_toggle():
    def release(clock, state, session):
        session["active"] = False

    world = posture_service_fixture(release)
    assert not world.stand(None).success
    assert 0x21010202 not in world.codes()


def test_posture_transition_waits_without_retry_or_busy_loop():
    def transition(clock, state, session):
        state["basic"] = 5 if clock.now < 104.0 else 6

    # 第一次预热后仍处于趴下，发送单次 toggle 后再模拟起立过程。
    def after_warmup(clock, state, session):
        if clock.now > 103.0:
            transition(clock, state, session)

    world = posture_service_fixture(after_warmup)
    assert world.stand(None).success
    assert world.codes().count(0x21010202) == 1
    assert len(world.clock.sleeps) > 2


# ---------------------------------------------------------------------------
# 契约第 5.1 节：建图 launch v2 自包含 + 雷达驱动条件启动
# ---------------------------------------------------------------------------
def test_mapping_launch_is_self_contained_v2() -> None:
    import xml.etree.ElementTree as ET

    source = read(LAUNCH)
    root = ET.parse(str(LAUNCH)).getroot()  # XML 必须合法（$(find ...) 是纯字符串）
    assert root.tag == "launch"
    # 参数来源必须写进文件头注释，提醒改动需重新实机验证
    assert "tmp/mapping_rslidar_indoor.launch.snapshot" in source
    assert "2026-11-14" in source
    # 绝不能再 include 机载 hdl_graph_slam 的 launch（否则会二次启动雷达驱动）。
    # 为什么不写裸字符串否定（如 assert "mapping_rslidar_indoor.launch" not in source）：
    # 契约 §5.1 同时要求文件头注释写明参数来源 tmp/mapping_rslidar_indoor.launch.snapshot，
    # 该路径本身包含这个名字；裸否定会与“保留来源注释”的硬要求互斥（已与 Lead 确认）。
    # 真正要防的是“把旧机载 hdl launch 重新 include 回来”，而 include 是结构化信息，
    # 因此这里做 XML include 结构断言 + 剥离注释后的可执行内容断言。
    # 请不要再改回裸字符串否定，也不要为此删掉文件头的 snapshot 来源注释。
    includes = [element.attrib for element in root.iter("include")]
    assert includes, "launch 至少应含雷达驱动的条件 include"
    assert all("mapping_rslidar_indoor.launch" not in attrs.get("file", "")
               for attrs in includes)
    assert not re.search(r'<include[^>]*file="[^"]*mapping_rslidar_indoor\.launch',
                         source)
    # 剥离 XML 注释后，可执行内容里不得再出现旧机载 launch 字符串
    executable = re.sub(r"<!--.*?-->", "", source, flags=re.S)
    assert "mapping_rslidar_indoor.launch" not in executable
    # 雷达驱动改为条件 include，默认 false（复用机载常驻驱动）
    args = {element.get("name"): element.get("default")
            for element in root.iter("arg")}
    assert args.get("start_lidar_driver") == "false"
    driver = [attrs for attrs in includes
              if "rs_lidar_16.launch" in attrs.get("file", "")]
    assert driver, "必须包含 rslidar_pointcloud/rs_lidar_16.launch"
    assert driver[0].get("if") == "$(arg start_lidar_driver)"
    # 自包含的 SLAM 节点与关键场景参数（逐项照抄机载快照）
    for token in ("tf2_ros", "static_transform_publisher", "base_link rslidar",
                  "PrefilteringNodelet", "ScanMatchingOdometryNodelet",
                  "HdlGraphSlamNodelet", "map2odom_publisher.py",
                  "FAST_VGICP",
                  'name="distance_near_thresh" value="0.4"',
                  'name="distance_far_thresh" value="100.0"',
                  'name="downsample_resolution" value="0.1"',
                  'name="keyframe_delta_trans" value="0.25"',
                  'name="keyframe_delta_angle" value="1.0"',
                  'name="distance_thresh" value="1.0"',
                  'name="accum_distance_thresh" value="3.0"',
                  'name="fitness_score_thresh" value="0.5"',
                  'name="graph_update_interval" value="1.5"',
                  'name="map_cloud_update_interval" value="3.0"',
                  'name="map_cloud_resolution" value="0.01"'):
        assert token in source, token
    # floor detection 保持旧版 enable_floor_detection=false 语义：不加载该 nodelet
    assert "load hdl_graph_slam/FloorDetectionNodelet" not in source
    assert not [node for node in root.iter("node")
                if node.get("name") == "floor_detection_nodelet"]


def test_mapping_launch_octomap_settings_unchanged() -> None:
    source = read(LAUNCH)
    for token in ('pkg="octomap_server"',
                  'name="frame_id" value="map"',
                  'name="resolution" value="0.05"',
                  'name="latch" value="true"',
                  'name="sensor_model/max_range" value="12.0"',
                  'name="pointcloud_min_z" value="0.15"',
                  'name="pointcloud_max_z" value="1.5"',
                  'name="occupancy_min_z" value="0.15"',
                  'name="occupancy_max_z" value="1.5"',
                  'from="cloud_in" to="/filtered_points"'):
        assert token in source, token


# ---------------------------------------------------------------------------
# 契约第 5.2 节：stream 的雷达预检、失败日志尾部与 localization 节流
# ---------------------------------------------------------------------------
def test_stream_mapping_preflight_and_log_tail() -> None:
    source = read(STREAM)
    # 独立恢复雷达并核对真实点云，SLAM 不再重复启动驱动。
    assert '"/rslidar_points"' in source or "LIDAR_TOPIC" in source
    assert "/rslidar_points" in source
    assert "getSystemState" in source
    assert "start_lidar_driver:=true" not in source
    assert "def ensure_lidar_runtime()" in source
    assert "rs_lidar_16.launch" in source
    assert "wait_for_message" in source
    assert "lidar_cloud_age(message)" in source
    assert "复用机载雷达驱动" in source
    assert "已独立启动驱动" in source
    assert "检测失败" in source  # getSystemState 异常时的兜底回执
    # 启动失败/进程早退：回传机载日志末尾（去换行、截断）
    assert "/tmp/airace_mapping.log" in source
    assert "MAPPING_LOG_TAIL_BYTES = 800" in source
    assert "mapping_log_tail()" in source
    assert "日志尾部" in source


def test_stream_localization_throttle_keeps_core_semantics() -> None:
    source = read(STREAM)
    # ≤10 Hz 节流：常量 + 模块级状态 + 回调内提前 return
    assert "LOCALIZATION_EMIT_PERIOD_S = 0.1" in source
    assert "last_localization_emit" in source
    assert "if now - last_localization_emit[0] < LOCALIZATION_EMIT_PERIOD_S:" in source
    assert "odometry_received" in source
    # 以下语义是 1.3.0 实机验证过的行为，禁止回退
    assert "CLOCK_MONOTONIC" in source
    assert "/airace/depth_received" in source
    assert "Float64(data=float(now_mono))" in source
    assert "POSTURE_TOGGLE_CODE" in source
    assert "def _finite(value)" in source
