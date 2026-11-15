"""ROS1 话题传输：``/airace/state`` JSON 状态 -> Frame，Twist 速度发布。

设计原则
--------
* **rospy 懒加载**：模块顶层不 import 任何 ROS 包，因此没有 ROS 的机器上也能
  ``import host_ui.app.transports.ros1``、调用纯适配函数和跑单测。
  ``Ros1Transport.available()`` 只探测 ``rospy`` 能否导入；
  ``start()`` 缺少 ROS 环境时抛 :class:`TransportError` 并给出中文安装提示。
* 所有消息字段 <-> 内部结构的转换都写成**纯函数**
  （``frame_from_state_payload`` / ``twist_fields`` / ``pose_from_odometry`` /
  ``laser_scan_points``），便于在没有 ROS 的环境做回归测试。

话题约定（与 ``lite2_sim/hardware/lite2_live.py`` 的 bridge 保持一致）
--------------------------------------------------------------------
* ``state_topic``（默认 ``/airace/state``，``std_msgs/String`` JSON）：
  字段 ``basic/gait/motion/rpy_deg/position/velocity_body/velocity_world/
  joystick/goal_velocity`` -> :class:`Frame`（``source="ros"``）。
* ``cmd_topic``（默认 ``/airace/cmd_vel``，``geometry_msgs/Twist``）：
  ``linear.x`` 为线速度 m/s，``angular.z`` 为偏航角速度 rad/s。
* ``map_topic``（默认 ``/map``，``nav_msgs/OccupancyGrid``）-> ``on_map``。
* ``scan_topic``（默认 ``/scan``，``sensor_msgs/LaserScan``）：用最近一次
  ``/odom`` 位姿转到里程计坐标，≤5 Hz、≤1500 点 -> ``on_scan``；
  假设雷达坐标系与 base_link 同向（无安装旋转），有外参时应在驱动/TF 层修正。
* ``odom_topic``（默认 ``/odom``，``nav_msgs/Odometry``）：提供位姿；若
  ``state_topic`` 连续 ``state_timeout_s`` 没有数据，则退化为用 odom 位姿
  以 10 Hz 产生帧（其余字段留默认值，``basic=0``），保证界面不全黑。
  任一订阅话题设为空字符串即可关闭该订阅。

线程模型
--------
* 订阅回调由 rospy 的 spinner 线程调用，内部状态用 ``self._lock`` 保护；
  回调在锁外触发，且吞掉异常，避免界面代码异常打断 rospy 回调线程。
* 零速看门狗用独立 ``threading.Thread``（不用 ``rospy.Timer``，减少对 ROS
  时钟/节点生命周期的依赖）：超过 ``zero_timeout_s`` 没有新指令就 5 Hz 发
  零速 Twist；``stop()`` 幂等，先补发零速再注销订阅/发布者。
* 不在 ``stop()`` 里调用 ``rospy.signal_shutdown``：上位机可能只是切换链路，
  ROS 节点生命周期由宿主进程决定。
"""
from __future__ import annotations

import json
import math
import threading
import time
from types import SimpleNamespace
from typing import Any, Iterable, Mapping, Sequence

from ..mapstore import map_from_occupancy_grid
from ..protocol import normalize_battery_level
from ..types import Frame, wrap_deg
from .base import Transport, TransportCallbacks, TransportError

#: 前端连接面板的默认参数（键名不可变）
DEFAULT_OPTIONS: dict[str, Any] = {
    "state_topic": "/airace/state",
    "cmd_topic": "/airace/cmd_vel",
    "map_topic": "/map",
    "scan_topic": "/scan",
    "odom_topic": "/odom",
    "ready_basic": 6,
    "joystick_threshold": 0.03,
    "zero_timeout_s": 0.3,
    "state_timeout_s": 1.0,
}

#: 状态话题持续无数据多久后启用 odom 退化帧
ODOM_FALLBACK_DELAY_S = 1.0
#: odom 退化帧推送周期（10 Hz）
ODOM_FRAME_INTERVAL_S = 0.1


def _as_float(value: Any, fallback: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def _as_int(value: Any, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


# ---------------------------------------------------------------------------
# 纯适配函数（无 ROS 依赖，可直接单测）
# ---------------------------------------------------------------------------
def rpy_deg_from_quaternion(x: float, y: float, z: float, w: float
                            ) -> tuple[float, float, float]:
    """四元数 -> (roll, pitch, yaw)，单位度。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        return (0.0, 0.0, 0.0)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def yaw_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> 航向角（度，逆时针为正）。"""
    return rpy_deg_from_quaternion(x, y, z, w)[2]


def pose_from_odometry(position: Any, orientation: Any
                       ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """``geometry_msgs/Vector3`` + ``Quaternion`` -> (position, rpy_deg)。

    ``position`` / ``orientation`` 只需具备 ``x/y/z``（四元数再加 ``w``）属性，
    因此本函数可以在没有 ROS 的环境里用简单对象测试。
    """
    try:
        px, py, pz = float(position.x), float(position.y), float(position.z)
        qx, qy = float(orientation.x), float(orientation.y)
        qz, qw = float(orientation.z), float(orientation.w)
    except AttributeError as error:
        raise ValueError(f"里程计位姿字段缺失: {error}") from None
    values = (px, py, pz, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("里程计位姿含 NaN/Inf")
    return (px, py, pz), rpy_deg_from_quaternion(qx, qy, qz, qw)


def _finite_triple(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in result):
        return None
    return result


def frame_from_state_payload(payload: Any, stamp: float,
                             source: str = "ros") -> Frame | None:
    """把 ``/airace/state`` 的 JSON 字典转成 :class:`Frame`；非法输入返回 None。

    缺省字段用 0 补齐（bridge 可能只发布了部分字段），但数组字段一旦存在
    就必须是 3 个有限数值，否则整帧丢弃，避免把 NaN 展示到界面。

    电量归一（契约第 1/3.5 节）：``battery_level_raw`` 为厂商 0.00~1.00 的
    double，也兼容实测直接给 59/67/87 的固件；两种都归一到
    ``battery_percent``（0~100）。``error_state`` / ``is_charging`` /
    ``zero_position_flag`` / ``motion_state`` 原样透传到 ``Frame.raw``；
    其中 ``error_state`` 文档标注 *Lite 无效*，只展示、不参与门禁。
    """
    if not isinstance(payload, Mapping):
        return None
    rpy = _finite_triple(payload.get("rpy_deg"))
    position = _finite_triple(payload.get("position"))
    if rpy is None or position is None:
        return None
    velocity_body = _finite_triple(payload.get("velocity_body")) or (0.0, 0.0, 0.0)
    velocity_world = _finite_triple(payload.get("velocity_world")) or (0.0, 0.0, 0.0)
    joystick = _finite_triple(payload.get("joystick")) or (0.0, 0.0, 0.0)
    goal_velocity = _finite_triple(payload.get("goal_velocity")) or (0.0, 0.0, 0.0)
    battery_raw = payload.get("battery_level_raw")
    if battery_raw is None:
        battery_raw = payload.get("battery_level")
    if battery_raw is None:
        battery_raw = payload.get("battery_raw")
    battery_percent = payload.get("battery_percent")
    if battery_percent is not None:
        try:
            number = float(battery_percent)
        except (TypeError, ValueError):
            number = None
        battery_percent = (max(0.0, min(100.0, number))
                           if number is not None and math.isfinite(number)
                           and number >= 0 else None)
    if battery_percent is None:
        battery_percent = normalize_battery_level(battery_raw)
    motion_state = payload.get("motion_state")
    if motion_state is None:
        motion_state = payload.get("motion")
    return Frame(
        source=source,
        stamp=float(stamp),
        position=position,
        rpy_deg=(rpy[0], rpy[1], wrap_deg(rpy[2])),
        velocity_body=velocity_body,
        velocity_world=velocity_world,
        basic=_as_int(payload.get("basic"), 0),
        gait=_as_int(payload.get("gait"), 0),
        motion=_as_int(payload.get("motion"), 0),
        joystick=joystick,
        goal_velocity=goal_velocity,
        raw={"transport": "ros", "state_topic": True,
             "battery_level_raw": battery_raw,
             "battery_raw": battery_raw,
             "battery_percent": battery_percent,
             "error_state": payload.get("error_state"),
             "is_charging": payload.get("is_charging"),
             "zero_position_flag": payload.get("zero_position_flag"),
             "motion_state": motion_state},
    )


def twist_fields(linear_m_s: float, angular_rad_s: float) -> dict[str, dict[str, float]]:
    """速度 -> Twist 字段字典（纯函数，便于验证单位/轴映射）。"""
    return {
        "linear": {"x": float(linear_m_s), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(angular_rad_s)},
    }


def apply_twist_fields(message: Any, linear_m_s: float,
                       angular_rad_s: float) -> Any:
    """把速度写进 ``geometry_msgs/Twist``（或测试用假对象）。"""
    fields = twist_fields(linear_m_s, angular_rad_s)
    message.linear.x = fields["linear"]["x"]
    message.linear.y = fields["linear"]["y"]
    message.linear.z = fields["linear"]["z"]
    message.angular.x = fields["angular"]["x"]
    message.angular.y = fields["angular"]["y"]
    message.angular.z = fields["angular"]["z"]
    return message


def laser_scan_points(ranges: Sequence[Any], angle_min: float,
                      angle_increment: float,
                      pose_xy_yaw_deg: tuple[float, float, float],
                      max_points: int = 1500, min_range: float = 0.05,
                      max_range: float = 30.0) -> list[tuple[float, float]]:
    """LaserScan 极坐标 -> 里程计坐标二维点集（纯函数，已按距离过滤/限点）。

    假设雷达坐标系与 base_link 同向（无安装旋转/平移）。``ranges`` 中的
    NaN、Inf、越界值与 0 值会被丢弃，最多输出 ``max_points`` 个点。
    """
    x, y, yaw_deg = (float(pose_xy_yaw_deg[0]), float(pose_xy_yaw_deg[1]),
                     float(pose_xy_yaw_deg[2]))
    yaw = math.radians(yaw_deg)
    limit = max(0, int(max_points))
    points: list[tuple[float, float]] = []
    for index, raw_distance in enumerate(ranges):
        if len(points) >= limit:
            break
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError):
            continue
        if (not math.isfinite(distance) or distance < min_range
                or distance > max_range):
            continue
        angle = yaw + float(angle_min) + index * float(angle_increment)
        points.append((x + distance * math.cos(angle),
                       y + distance * math.sin(angle)))
    return points


def _import_ros() -> SimpleNamespace:
    """懒加载 rospy 与消息包；缺失时抛带中文提示的 TransportError。"""
    try:
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except Exception as error:  # ImportError / 环境损坏
        raise TransportError(
            "未检测到可用的 ROS1 Python 环境（rospy 或消息包导入失败）。"
            "请先在终端执行 source /opt/ros/<distro>/setup.bash，"
            f"并确认当前 Python 能 import rospy；原始错误：{error}") from error
    return SimpleNamespace(rospy=rospy, Twist=Twist,
                           OccupancyGrid=OccupancyGrid, Odometry=Odometry,
                           LaserScan=LaserScan, String=String)


class Ros1Transport(Transport):
    """ROS1 话题链路。"""

    kind = "ros"
    label = "ROS1 话题"
    default_options = dict(DEFAULT_OPTIONS)
    #: 机载 ROS1 可以直接调用 /airace/prepare、/airace/disarm（Trigger）
    capabilities = {"arm": True, "posture": False, "vendor_estop": False,
                    "sensors": True, "reconnect": False}

    ZERO_RESEND_PERIOD_S = 0.2
    WARN_INTERVAL_S = 2.0
    SCAN_MAX_HZ = 5.0
    SCAN_MAX_POINTS = 1500

    @staticmethod
    def available() -> bool:
        """只探测 ``rospy`` 是否可导入（不初始化节点）。"""
        try:
            import rospy  # noqa: F401
            return True
        except Exception:
            return False

    def __init__(self, options: dict | None, callbacks: TransportCallbacks) -> None:
        super().__init__(options, callbacks)
        opts = self.options
        self.state_topic = str(opts.get("state_topic", DEFAULT_OPTIONS["state_topic"]))
        self.cmd_topic = str(opts.get("cmd_topic", DEFAULT_OPTIONS["cmd_topic"]))
        self.map_topic = str(opts.get("map_topic", DEFAULT_OPTIONS["map_topic"]))
        self.scan_topic = str(opts.get("scan_topic", DEFAULT_OPTIONS["scan_topic"]))
        self.odom_topic = str(opts.get("odom_topic", DEFAULT_OPTIONS["odom_topic"]))
        self.ready_basic = _as_int(opts.get("ready_basic"), 6)
        self.joystick_threshold = _as_float(opts.get("joystick_threshold"), 0.03)
        self.zero_timeout_s = max(0.05, _as_float(opts.get("zero_timeout_s"), 0.3))
        self.state_timeout_s = max(0.1, _as_float(opts.get("state_timeout_s"), 1.0))

        self._lifecycle_lock = threading.Lock()
        self._lock = threading.Lock()
        self._event_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False

        self._ros: SimpleNamespace | None = None
        self._publisher: Any = None
        self._subscribers: list[Any] = []

        self._last_cmd_time = 0.0
        self._last_zero_send = 0.0
        self._zero_active = True
        self._state_seen = False
        self._last_state_time = 0.0
        self._last_odom_time = 0.0
        self._odom_position = (0.0, 0.0, 0.0)
        self._odom_rpy = (0.0, 0.0, 0.0)
        self._last_odom_frame_time = 0.0
        self._last_scan_time = 0.0
        self._fallback_reported = False
        self._map_reported = False
        self._state_up_reported = False
        # 使能状态（Trigger 服务回执推断，仅用于 status.control 展示）
        self._control_armed: bool | None = None
        self._control_auto_ack = 0.0
        self._control_last_error = ""

        self._event_times: dict[str, float] = {}
        self._counters = {
            "state_frames": 0,
            "odom_frames": 0,
            "odom_fallback_frames": 0,
            "map_frames": 0,
            "scans": 0,
            "send_errors": 0,
        }

    # ------------------------------------------------------------------ 工具
    def _emit_event(self, level: str, code: str, message: str,
                    min_interval: float = 0.0) -> None:
        now = time.monotonic()
        if min_interval > 0:
            with self._event_lock:
                last = self._event_times.get(code)
                if last is not None and now - last < min_interval:
                    return
                self._event_times[code] = now
        try:
            self.callbacks.on_event(level, code, message)
        except Exception:  # 界面回调异常不能打断 rospy 回调线程
            pass

    def _safe_callback(self, callback, *args) -> None:
        if callback is None:
            return
        try:
            callback(*args)
        except Exception:  # pragma: no cover - 防御性
            pass

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> None:
        """初始化（或复用）ROS 节点并订阅话题；缺 ROS 环境抛 TransportError。"""
        with self._lifecycle_lock:
            if self._running:
                return
            ros = _import_ros()
            rospy = ros.rospy
            try:
                # 宿主可能已经初始化过节点；重复初始化失败不影响后续订阅
                rospy.init_node("lite2_host_ui_transport", anonymous=True,
                                disable_signals=True)
            except Exception:
                pass
            try:
                publisher = rospy.Publisher(self.cmd_topic, ros.Twist, queue_size=1)
                subscribers: list[Any] = [
                    rospy.Subscriber(self.state_topic, ros.String,
                                     self._on_state, queue_size=2),
                ]
                if self.map_topic:
                    subscribers.append(rospy.Subscriber(
                        self.map_topic, ros.OccupancyGrid, self._on_map,
                        queue_size=1))
                if self.scan_topic:
                    subscribers.append(rospy.Subscriber(
                        self.scan_topic, ros.LaserScan, self._on_scan,
                        queue_size=1))
                if self.odom_topic:
                    subscribers.append(rospy.Subscriber(
                        self.odom_topic, ros.Odometry, self._on_odom,
                        queue_size=1))
            except Exception as error:
                raise TransportError(f"ROS1 话题订阅/发布初始化失败：{error}") from error
            self._ros = ros
            self._publisher = publisher
            self._subscribers = subscribers
            self._stop_event.clear()
            self._last_cmd_time = time.monotonic()
            self._last_zero_send = 0.0
            self._zero_active = True
            self._state_seen = False
            self._last_state_time = 0.0
            self._last_odom_time = 0.0
            self._fallback_reported = False
            self._map_reported = False
            self._state_up_reported = False
            self._running = True
            self._thread = threading.Thread(
                target=self._watchdog_loop, name="lite2-ros-watchdog", daemon=True)
            self._thread.start()
        self._emit_event(
            "info", "link_up",
            f"ROS1 已订阅 {self.state_topic}，速度发布到 {self.cmd_topic}")

    def stop(self) -> None:
        """幂等停止：补发零速、注销订阅/发布者、停看门狗线程。"""
        with self._lifecycle_lock:
            if not self._running:
                return
            self._running = False
            self._stop_event.set()
            for _ in range(3):
                self._publish_twist(0.0, 0.0)
                time.sleep(0.02)
            thread = self._thread
            self._thread = None
            subscribers = self._subscribers
            self._subscribers = []
            publisher = self._publisher
            self._publisher = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        for subscriber in subscribers:
            try:
                subscriber.unregister()
            except Exception:
                pass
        if publisher is not None:
            try:
                publisher.unregister()
            except Exception:
                pass
        self._emit_event("info", "link_down", "ROS1 链路已关闭，已补发零速")

    # ------------------------------------------------------------------ 数据面
    def _publish_twist(self, linear_m_s: float, angular_rad_s: float) -> bool:
        publisher = self._publisher
        ros = self._ros
        if publisher is None or ros is None:
            return False
        try:
            message = ros.Twist()
            apply_twist_fields(message, linear_m_s, angular_rad_s)
            publisher.publish(message)
        except Exception as error:
            with self._lock:
                self._counters["send_errors"] += 1
            self._emit_event("warn", "send_failed",
                             f"发布 {self.cmd_topic} 失败：{error}",
                             self.WARN_INTERVAL_S)
            return False
        return True

    def send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        """非阻塞发布 Twist；内部看门狗据此判断是否补零。"""
        try:
            linear = float(linear_m_s)
            angular = float(angular_rad_s)
        except (TypeError, ValueError):
            self._emit_event("warn", "invalid_command",
                             "速度指令不是数值，已忽略", self.WARN_INTERVAL_S)
            return
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self._emit_event("warn", "invalid_command",
                             "速度指令含 NaN/Inf，已忽略", self.WARN_INTERVAL_S)
            return
        if not self._running:
            return
        with self._lock:
            self._last_cmd_time = time.monotonic()
        self._zero_active = False
        self._publish_twist(linear, angular)

    # ------------------------------------------------------------------ 能力面（使能/解除）
    def _trigger(self, action: str, timeout_s: float = 8.0) -> tuple[bool, str]:
        """懒加载调用 ``/airace/prepare`` | ``/airace/disarm``（std_srvs/Trigger）。"""
        service = "/airace/prepare" if action == "prepare" else "/airace/disarm"
        label = "使能" if action == "prepare" else "解除"
        if not self._running:
            return False, "ROS1 链路未启动，无法%s" % label
        try:
            import rospy
            from std_srvs.srv import Trigger
        except Exception as error:
            return False, ("ROS1 环境不可用（rospy/std_srvs 导入失败）：%s"
                           % error)
        timeout = max(1.0, _as_float(timeout_s, 8.0))
        try:
            rospy.wait_for_service(service, timeout=min(5.0, timeout))
            response = rospy.ServiceProxy(service, Trigger)()
            success = bool(getattr(response, "success", False))
            message = str(getattr(response, "message", "") or "")
        except Exception as error:
            message = "调用 %s 失败：%s" % (service, error)
            with self._lock:
                self._control_last_error = message
            self._emit_event("warn", "ros_service_failed",
                             "%s%s失败：%s" % (label, service, error),
                             self.WARN_INTERVAL_S)
            return False, message
        with self._lock:
            if success:
                self._control_armed = (action == "prepare")
                if action == "prepare":
                    self._control_auto_ack = time.monotonic()
                self._control_last_error = ""
            else:
                self._control_last_error = message or ("%s被拒绝" % label)
        self._emit_event("info" if success else "warn", "ros_control",
                         "%s：%s" % (label, message or service))
        if success:
            return True, message or ("机载执行层%s成功" % label)
        return False, message or ("机载执行层拒绝%s请求" % label)

    def arm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        return self._trigger("prepare", timeout_s)

    def disarm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        return self._trigger("disarm", timeout_s)

    # ------------------------------------------------------------------ 订阅回调
    def _on_state(self, message: Any) -> None:
        received = time.monotonic()
        data = getattr(message, "data", None)
        try:
            payload = json.loads(data) if isinstance(data, (str, bytes, bytearray)) else None
        except (TypeError, ValueError):
            payload = None
        if payload is None:
            self._emit_event("warn", "bad_state",
                             f"{self.state_topic} 不是合法 JSON，已丢弃",
                             self.WARN_INTERVAL_S)
            return
        frame = frame_from_state_payload(payload, received)
        if frame is None:
            self._emit_event("warn", "bad_state",
                             f"{self.state_topic} 字段非法（缺 position/rpy 或含 NaN），已丢弃",
                             self.WARN_INTERVAL_S)
            return
        with self._lock:
            self._state_seen = True
            self._last_state_time = received
            self._counters["state_frames"] += 1
            first = not self._state_up_reported
            if first:
                self._state_up_reported = True
        if first:
            self._emit_event(
                "info", "state_up",
                f"已收到 {self.state_topic} 状态（basic={frame.basic}）")
        self._safe_callback(self.callbacks.on_frame, frame)

    def _on_map(self, message: Any) -> None:
        try:
            info = message.info
            origin = info.origin
            yaw_deg = yaw_deg_from_quaternion(
                origin.orientation.x, origin.orientation.y,
                origin.orientation.z, origin.orientation.w)
            grid = map_from_occupancy_grid(
                info.width, info.height, info.resolution,
                (origin.position.x, origin.position.y, yaw_deg),
                message.data, name="ros_map")
        except (AttributeError, TypeError, ValueError) as error:
            self._emit_event("warn", "bad_map",
                             f"{self.map_topic} 地图消息转换失败：{error}",
                             self.WARN_INTERVAL_S)
            return
        with self._lock:
            self._counters["map_frames"] += 1
            first = not self._map_reported
            if first:
                self._map_reported = True
        if first:
            self._emit_event(
                "info", "map_up",
                f"已收到 {self.map_topic} 地图：{grid.width}x{grid.height} 格，"
                f"{grid.resolution:.3f} m/格")
        self._safe_callback(self.callbacks.on_map, grid)

    def _on_scan(self, message: Any) -> None:
        received = time.monotonic()
        with self._lock:
            if received - self._last_scan_time < 1.0 / self.SCAN_MAX_HZ:
                return
            pose = (self._odom_position[0], self._odom_position[1],
                    self._odom_rpy[2])
            has_odom = self._last_odom_time > 0.0
        if not has_odom:
            self._emit_event("warn", "scan_without_odom",
                             f"{self.scan_topic} 已收到，但还没有 {self.odom_topic} "
                             f"位姿，无法把激光转到里程计坐标，已跳过",
                             self.WARN_INTERVAL_S)
            return
        try:
            points = laser_scan_points(
                message.ranges, message.angle_min, message.angle_increment, pose,
                max_points=self.SCAN_MAX_POINTS,
                min_range=float(getattr(message, "range_min", 0.05)),
                max_range=float(getattr(message, "range_max", 30.0)))
        except (AttributeError, TypeError, ValueError) as error:
            self._emit_event("warn", "bad_scan",
                             f"{self.scan_topic} 激光消息转换失败：{error}",
                             self.WARN_INTERVAL_S)
            return
        with self._lock:
            self._last_scan_time = received
            self._counters["scans"] += 1
        if points:
            self._safe_callback(self.callbacks.on_scan, points)

    def _on_odom(self, message: Any) -> None:
        received = time.monotonic()
        try:
            position, rpy = pose_from_odometry(message.pose.pose.position,
                                               message.pose.pose.orientation)
        except (AttributeError, TypeError, ValueError) as error:
            self._emit_event("warn", "bad_odom",
                             f"{self.odom_topic} 里程计消息转换失败：{error}",
                             self.WARN_INTERVAL_S)
            return
        with self._lock:
            self._odom_position = position
            self._odom_rpy = rpy
            self._last_odom_time = received
            self._counters["odom_frames"] += 1

    # ------------------------------------------------------------------ 看门狗
    def _watchdog_loop(self) -> None:
        while not self._stop_event.wait(0.05):
            now = time.monotonic()
            with self._lock:
                last_cmd = self._last_cmd_time
                state_seen = self._state_seen
                last_state = self._last_state_time
                last_odom = self._last_odom_time
                odom_position = self._odom_position
                odom_rpy = self._odom_rpy
                last_odom_frame = self._last_odom_frame_time
            # 零速看门狗
            if now - last_cmd > self.zero_timeout_s:
                if now - self._last_zero_send >= self.ZERO_RESEND_PERIOD_S:
                    self._last_zero_send = now
                    self._publish_twist(0.0, 0.0)
                    if not self._zero_active:
                        self._zero_active = True
                        self._emit_event(
                            "warn", "watchdog_zero",
                            f"{self.zero_timeout_s:.2f}s 没有新速度指令，"
                            f"看门狗已开始周期补发零速",
                            self.WARN_INTERVAL_S)
            else:
                self._zero_active = False
            # 状态过期告警（限频）
            state_age = now - last_state if state_seen else math.inf
            if state_age > self.state_timeout_s:
                if state_seen:
                    message = (f"{self.state_topic} 已 {state_age:.1f}s 未更新"
                               f"（阈值 {self.state_timeout_s:.1f}s）")
                else:
                    message = f"尚未收到 {self.state_topic}，请检查 bridge 是否运行"
                self._emit_event("warn", "state_timeout", message,
                                 self.WARN_INTERVAL_S)
            # 状态话题长期缺失时用 odom 位姿退化出帧，避免界面上什么都没
            odom_age = now - last_odom if last_odom > 0.0 else math.inf
            if (state_age > ODOM_FALLBACK_DELAY_S and odom_age <= 1.0
                    and now - last_odom_frame >= ODOM_FRAME_INTERVAL_S):
                frame = Frame(
                    source="ros",
                    stamp=now,
                    position=(float(odom_position[0]), float(odom_position[1]),
                              float(odom_position[2])),
                    rpy_deg=(float(odom_rpy[0]), float(odom_rpy[1]),
                             wrap_deg(float(odom_rpy[2]))),
                    raw={"transport": "ros", "fallback": "odom"},
                )
                with self._lock:
                    self._last_odom_frame_time = now
                    self._counters["odom_fallback_frames"] += 1
                    first = not self._fallback_reported
                    if first:
                        self._fallback_reported = True
                if first:
                    self._emit_event(
                        "warn", "odom_fallback",
                        f"{self.state_topic} 无数据，已退化为使用 {self.odom_topic} "
                        f"位姿产生状态帧（basic 等字段未知）",
                        self.WARN_INTERVAL_S)
                self._safe_callback(self.callbacks.on_frame, frame)

    # ------------------------------------------------------------------ 状态
    def status(self) -> dict:
        now = time.monotonic()
        with self._lock:
            counters = dict(self._counters)
            state_seen = self._state_seen
            last_state = self._last_state_time
            last_odom = self._last_odom_time
            running = self._running
            control_armed = self._control_armed
            auto_ack = self._control_auto_ack
            control_error = self._control_last_error
        state_age = (now - last_state) if state_seen else None
        stale = (state_age is None) or (state_age > self.state_timeout_s)
        odom_age = (now - last_odom) if last_odom > 0.0 else None
        fallback = bool(state_age is None or state_age > ODOM_FALLBACK_DELAY_S) \
            and odom_age is not None and odom_age <= 1.0
        if not running:
            detail = "未启动"
        elif state_age is None:
            detail = f"已订阅 {self.state_topic}，尚未收到状态；等待 bridge"
        elif stale:
            detail = (f"{self.state_topic} 已 {state_age:.2f}s 未更新"
                      f"（阈值 {self.state_timeout_s:.2f}s）")
        else:
            detail = f"状态 {state_age * 1000:.0f} ms 前更新"
        return {
            "kind": self.kind,
            "label": self.label,
            "connected": bool(running),
            "detail": detail,
            "stale": bool(stale),
            "state_age_s": state_age,
            "odom_age_s": odom_age,
            "fallback_odom": bool(fallback),
            "topics": {
                "state": self.state_topic,
                "cmd": self.cmd_topic,
                "map": self.map_topic,
                "scan": self.scan_topic,
                "odom": self.odom_topic,
            },
            "zero_timeout_s": self.zero_timeout_s,
            "state_timeout_s": self.state_timeout_s,
            "control": {
                "armed": (None if control_armed is None
                          else bool(control_armed)),
                "mode_confirmed": None,
                "pending": None,
                "last_error": control_error,
                "depth_age_s": None,
                "automatic_ack_age_s": (round(time.monotonic() - auto_ack, 2)
                                        if auto_ack > 0.0 else None),
            },
            "reconnect": {"enabled": False, "attempts": 0,
                          "next_retry_in_s": None},
            "execution": {},
            **counters,
        }


__all__ = [
    "DEFAULT_OPTIONS",
    "ODOM_FALLBACK_DELAY_S",
    "ODOM_FRAME_INTERVAL_S",
    "Ros1Transport",
    "apply_twist_fields",
    "frame_from_state_payload",
    "laser_scan_points",
    "pose_from_odometry",
    "rpy_deg_from_quaternion",
    "twist_fields",
    "yaw_deg_from_quaternion",
]
