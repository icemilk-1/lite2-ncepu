"""ROS1-side JSON stream, compatible with the robot's Python 2.7 runtime.

本文件不参与上位机进程，而是由 ``ssh_ros.py`` 通过 SSH 把源码 base64 后
在机载感知主机（192.168.1.103，已注册终端）上用 ``python -u -c`` 执行。

数据面（机载 -> 上位机，一行一条 JSON）
---------------------------------------
* ``{"type":"state","data":{...}}``     对照 ``/airace/state``（≤10 Hz）
* ``{"type":"execution","data":{...}}`` ``/airace/execution`` 原样转发，并附加
  ``host_ui_depth_age_s``（机载侧深度帧龄，供上位机就绪清单展示）
* ``{"type":"map","data":{...}}``       ``/projected_map``
* ``{"type":"sensor","data":{...}}``    相机 JPEG / 激光点云预览
* ``{"type":"control","data":{...}}``   prepare/disarm/posture/estop 回执
* ``{"type":"heartbeat","data":{...}}`` ping 回执
* ``{"type":"ready","data":{}}``        初始化完成

命令面（上位机 -> 机载 stdin，一行一条 JSON）
---------------------------------------------
* ``{"linear":0.0,"angular":0.0}``  速度指令（0.3 s 无新指令自动补零）
* ``{"action":"prepare"}``          ``/airace/prepare``（Trigger 服务，最多阻塞 3 s）
* ``{"action":"disarm"}``           ``/airace/disarm``（Trigger 服务）
* ``{"action":"posture","target":"stand"|"lie"}``
  由**机载进程**发心跳 ``0x21040001``（≥5 Hz）+ **单次** ``0x21010202``，
  轮询 ``/airace/state.basic`` 到目标（stand→6，lie→1）或超时；结果回执。
* ``{"action":"estop"}``            由机载进程发 ``0x21010C0E`` 三次
* ``{"action":"ping"}``             回 ``heartbeat``

为什么要由机载代发厂商指令
--------------------------
厂商 Lite2 的 ``0x2*******`` 指令同一时刻只接受一个终端下发。当前实机
``jy_exe`` 按已登记源 IP 的顺序仲裁，只有首位终端的此类指令被处理。因此
``POSTURE_TOGGLE`` / ``ESTOP`` / ``HEARTBEAT`` 全部由 192.168.1.103 的机载
进程发送；Windows 侧只发高层 JSON 命令。

深度心跳时钟源
--------------
机载 execution guard（``lite2_sim/hardware/execution_guard.py`` 的 ``fresh``）
用 ``time.monotonic()`` 比较 ``/airace/depth_received`` 的 Float64 值：
``0 ≤ now - value ≤ 0.5``。所以这里发布的是 **monotonic 时钟**，不能用
``rospy.Time.now().to_sec()``（墙钟 ~1.79e9 会让差值为负，永远 stale_depth）。
Python 2.7 通过 ctypes 调用 ``clock_gettime(CLOCK_MONOTONIC)``，失败时尝试
``monotonic`` 包；时钟不可用则拒绝启动。
深度断流超过 0.5 s 立即停发，恢复收到新帧后重新发布（≥2 Hz）。
"""
import base64
import atexit
import binascii
import json
import math
import os
import platform
import signal
import subprocess
import select
import socket
import struct
import sys
READ_ONLY = "--read-only" in sys.argv
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
from std_msgs.msg import Float32, Float64, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 厂商指令（V1.2.3）
# ---------------------------------------------------------------------------
HEARTBEAT_CODE = 0x21040001
ESTOP_CODE = 0x21010C0E
POSTURE_TOGGLE_CODE = 0x21010202
VENDOR_ADDRESS = ("192.168.1.120", 43893)
POSTURE_TARGETS = {"stand": 6, "lie": 1}

#: 深度帧龄阈值（与机载 execution_guard 一致）
DEPTH_FRESH_S = 0.5
#: 发布者仅用于避免重复启动；就绪还必须验证实际新鲜点云。
LIDAR_TOPIC = "/rslidar_points"
RUNTIME_LOCK = "/tmp/airace_runtime.lock"
LIDAR_PID = "/tmp/airace_lidar.pid"
LIDAR_LOG = "/tmp/airace_lidar.log"
LIDAR_START_TIMEOUT_S = 8.0
LIDAR_MAX_AGE_S = 1.0
#: roslaunch 的 stdout/stderr 追加日志（启动失败时从尾部取诊断信息）
MAPPING_LOG = "/tmp/airace_mapping.log"
#: 失败回执里附带的日志尾部字节数
MAPPING_LOG_TAIL_BYTES = 800
#: /odom 回调里 localization 回执的节流周期（≤10 Hz，避免 SSH 单流被刷满）
LOCALIZATION_EMIT_PERIOD_S = 0.1


def _finite(value):
    """Python 2.7 没有 math.isfinite（3.2+ 才加入）。"""
    return not (math.isnan(value) or math.isinf(value))


#: /airace/depth_received 发布周期：10 Hz（要求 ≥2 Hz）
DEPTH_PUBLISH_PERIOD_S = 0.1
#: 姿态窗口内心跳周期：5 Hz（厂商要求 >1 Hz，这里留足余量）
POSTURE_HEARTBEAT_PERIOD_S = 0.2
#: 姿态确认超时（秒）；实际等待由机载 bridge 执行。
POSTURE_TIMEOUT_S = 20.0
#: 服务调用等待 ROS master 的超时（秒）
SERVICE_WAIT_S = 2.0

#: 单调时钟：必须与机载 execution_guard（Python 3 time.monotonic）同源。
#: py3 直接可用；机载 Python 2.7 没有 time.monotonic，用 ctypes 调
#: clock_gettime(CLOCK_MONOTONIC)，它与 py3 的 time.monotonic 是同一内核时钟。
#: 禁止静默退化为墙钟（否则桥接的 fresh() 永远判定 stale_depth）。
def _monotonic_from_ctypes():
    import ctypes
    import ctypes.util

    libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6",
                       use_errno=True)

    class Timespec(ctypes.Structure):
        _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

    def read_clock():
        value = Timespec()
        if libc.clock_gettime(1, ctypes.byref(value)) != 0:
            raise RuntimeError("clock_gettime(CLOCK_MONOTONIC) failed")
        return float(value.tv_sec) + float(value.tv_nsec) / 1e9

    return read_clock


try:
    monotonic = time.monotonic
except AttributeError:  # pragma: no cover - 机载 Python 2.7 路径
    try:
        monotonic = _monotonic_from_ctypes()
    except Exception:  # pragma: no cover
        try:
            from monotonic import monotonic  # type: ignore
        except ImportError:
            raise RuntimeError(
                "无法获得单调时钟（time.monotonic / clock_gettime / monotonic 均不可用）；"
                "深度心跳将与机载 execution_guard 不同源，拒绝启动以避免错误的运动门禁")
        if monotonic is time.time:  # pragma: no cover - 禁止墙钟退化
            raise RuntimeError("monotonic 包退化为墙钟，拒绝启动")


write_lock = threading.Lock()
last_state = [0.0]
#: 上一次 localization 回执时间（“≤10 Hz”节流的模块级状态）
last_localization_emit = [0.0]
sensor_times = {}
#: 最近一次新鲜深度帧的 monotonic 时间戳（0 表示尚未收到）
depth_time = [0.0]
#: 最近一次 /airace/state 的 basic（姿态轮换确认依据）
latest_basic = [None]
mapping_lock = threading.Lock()
MAPPING_LAUNCH = "/home/ysc/airace_deployment_20261114/hardware/navigation_mapping.launch"
MAPPING_PID = "/tmp/airace_mapping.pid"


def mapping_pid():
    try:
        with open(MAPPING_PID) as stream:
            pid = int(stream.read().strip())
        with open("/proc/%s/cmdline" % pid, "rb") as stream:
            args = stream.read().decode("utf-8").split("\x00")
        if MAPPING_LAUNCH in args:
            return pid
    except (IOError, OSError, ValueError):
        pass
    return None


def lidar_driver_running():
    """探测 /rslidar_points 是否已有发布者。

    返回 True（已有发布者，不应再起第二驱动）、False（无发布者）
    或 None（rosgraph 检测失败，此时不得推断驱动不存在或传感器已就绪）。
    注意 getSystemState 是远端调用，可能抛异常（master 重载/网络抖动），必须兜底。
    """
    try:
        code, message, state = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("ROS master getSystemState failed: %s" % message)
        for topic, nodes in state[0]:
            if str(topic) == LIDAR_TOPIC and nodes:
                return True
        return False
    except Exception:
        return None


def lidar_runtime_pid():
    """Recognize only our still-running standalone launch, not a recycled PID."""
    try:
        with open(LIDAR_PID) as stream:
            pid = int(stream.read().strip())
        if pid <= 0:
            return None
        with open("/proc/%s/cmdline" % pid, "rb") as stream:
            arguments = stream.read().decode("utf-8").split("\x00")
        if "rs_lidar_16.launch" in arguments:
            return pid
    except (IOError, OSError, ValueError):
        pass
    return None


def lidar_cloud_age(message):
    """Return source age only for nonempty, finite, fresh PointCloud2 data."""
    try:
        stamp = float(message.header.stamp.to_sec())
        age = float(rospy.Time.now().to_sec()) - stamp
        if (not _finite(stamp) or stamp <= 0 or not _finite(age)
                or age < 0 or age > LIDAR_MAX_AGE_S
                or message.width * message.height <= 0 or not message.data):
            return None
        for point in point_cloud2.read_points(message, field_names=("x", "y", "z"),
                                              skip_nans=True):
            if all(_finite(float(value)) for value in point):
                return age
    except Exception:
        pass
    return None


def ensure_lidar_runtime():
    """Reuse/start the sensor alone under the runtime lock, then verify real data.

    The PID record also prevents duplicate launches while ROS registration is
    delayed. A registered publisher without fresh points is a sensor failure,
    never a reason to start another driver.
    """
    if READ_ONLY:
        raise RuntimeError("只读连接不启动雷达驱动")
    import fcntl

    with open(RUNTIME_LOCK, "a") as guard:
        fcntl.flock(guard, fcntl.LOCK_EX)
        present = lidar_driver_running()
        if present is None:
            raise RuntimeError("雷达发布者检测失败，未启动额外驱动；请检查 ROS master")
        pid = lidar_runtime_pid()
        started = False
        process = None
        if not present and pid is None:
            arguments = ["bash", "-c",
                         ". /home/ysc/jueying_mapping_localization_ws/devel/setup.bash && "
                         "exec roslaunch rslidar_pointcloud rs_lidar_16.launch"]
            with open(LIDAR_LOG, "ab") as log:
                with open(os.devnull, "rb") as stdin:
                    process = subprocess.Popen(arguments, stdin=stdin, stdout=log,
                                               stderr=log, close_fds=True,
                                               preexec_fn=os.setsid)
            pid = process.pid
            with open(LIDAR_PID, "w") as stream:
                stream.write(str(pid))
            started = True
        deadline = monotonic() + LIDAR_START_TIMEOUT_S
        while monotonic() < deadline and not rospy.is_shutdown():
            if process is not None and process.poll() is not None:
                raise RuntimeError("雷达驱动启动失败，请检查 %s" % LIDAR_LOG)
            present = lidar_driver_running()
            if present is None:
                raise RuntimeError("雷达发布者检测失败，保留已有驱动；请检查 ROS master")
            if present:
                try:
                    message = rospy.wait_for_message(
                        LIDAR_TOPIC, PointCloud2, timeout=min(0.5, max(
                            0.01, deadline - monotonic())))
                except Exception:
                    message = None
                if message is not None:
                    age = lidar_cloud_age(message)
                    if age is not None:
                        return {"started": started, "pid": pid,
                                "points": int(message.width * message.height), "age_s": age}
            time.sleep(0.05)
        raise RuntimeError("雷达未就绪：未收到非空新鲜点云；未重复启动驱动，请检查供电、网络及 %s"
                           % LIDAR_LOG)


def lidar_runtime_worker():
    """Optional sensor recovery must never interrupt the control heartbeat."""
    try:
        detail = ensure_lidar_runtime()
        detail.update(component="激光雷达", state="ready",
                      message="雷达点云已就绪（%s）" % (
                          "已独立启动驱动" if detail["started"] else "复用机载雷达驱动"))
        emit("runtime", detail)
    except Exception as error:
        emit("runtime", {"component": "激光雷达", "state": "failed", "message": str(error)})


def start_lidar_recovery():
    if not READ_ONLY:
        _start_worker(lidar_runtime_worker)


def mapping_log_tail():
    """读取建图日志末尾约 800 字节，去换行并截断（现场排障用）。"""
    try:
        size = os.path.getsize(MAPPING_LOG)
        with open(MAPPING_LOG, "rb") as stream:
            stream.seek(max(0, size - MAPPING_LOG_TAIL_BYTES), os.SEEK_SET)
            raw = stream.read(MAPPING_LOG_TAIL_BYTES)
    except (IOError, OSError):
        return ""
    text = raw.decode("utf-8", "replace")
    return " ".join(text.split())[:MAPPING_LOG_TAIL_BYTES]


def mapping_worker(action):
    if not mapping_lock.acquire(False):
        emit("control", {"action": action, "success": False,
                         "message": "建图操作正在执行"})
        return
    message = "建图已停止"
    try:
        pid = mapping_pid()
        if action == "mapping_start" and pid is None:
            # 与后台恢复共用同一进程锁和独立驱动；SLAM launch 不再启动第二驱动。
            ensure_lidar_runtime()
            arguments = ["roslaunch", MAPPING_LAUNCH]
            message = "建图进程已启动（雷达点云已验证，地图及定位仍需等待）"
            with open(MAPPING_LOG, "ab") as log:
                with open(os.devnull, "rb") as stdin:
                    process = subprocess.Popen(arguments,
                                               stdin=stdin, stdout=log, stderr=log,
                                               close_fds=True, preexec_fn=os.setsid)
            with open(MAPPING_PID, "w") as stream:
                stream.write(str(process.pid))
            time.sleep(1.0)
            if process.poll() is not None:
                raise RuntimeError("建图进程退出，请检查 /tmp/airace_mapping.log")
        elif action == "mapping_stop" and pid is not None:
            os.kill(pid, signal.SIGINT)
            deadline = monotonic() + 12.0
            while mapping_pid() is not None and monotonic() < deadline:
                time.sleep(0.1)
            if mapping_pid() is not None:
                raise RuntimeError("建图进程尚未退出，请检查机载日志")
        elif action == "mapping_start":
            message = "建图已在运行"
        emit("control", {"action": action, "success": True, "message": message})
    except Exception as error:
        message = str(error)
        if action == "mapping_start":
            # 启动失败/进程早退：把机载日志尾部拼进回执，方便现场直接排障
            tail = mapping_log_tail()
            if tail:
                message = "%s；机载日志尾部：%s" % (message, tail)
        emit("control", {"action": action, "success": False, "message": message})
    finally:
        mapping_lock.release()

_vendor_socket = [None]
_vendor_lock = threading.Lock()
#: 正在执行的姿态/急停任务数，避免同一窗口重复下发轮换指令
_command_busy = [False]
_command_busy_lock = threading.Lock()
_control_cleanup_started = [False]


def emit(kind, data):
    with write_lock:
        sys.stdout.write(json.dumps({"type": kind, "data": data}) + "\n")
        sys.stdout.flush()


def ensure_runtime():
    """控制链路连接时恢复重启后缺失的执行桥接和深度驱动。"""
    import fcntl

    with open(RUNTIME_LOCK, "a") as guard:
        fcntl.flock(guard, fcntl.LOCK_EX)
        code, message, system = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("ROS master unavailable: %s" % message)
        publishers = dict(system[0])
        jobs = []
        if not publishers.get("/airace/state"):
            jobs.append(("执行桥接", ["python3", "-u", "lite2_live.py", "bridge"],
                         os.path.dirname(MAPPING_LAUNCH), "/tmp/airace_bridge.log"))
        if not (publishers.get("/camera/aligned_depth_to_color/image_raw")
                or publishers.get("/camera/depth/image_rect_raw")):
            jobs.append(("深度相机", ["bash", "-c",
                         ". /home/ysc/realsense-driver-jetson/devel/setup.bash && "
                         "exec roslaunch realsense2_camera rs_camera_424_240.launch"],
                         None, "/tmp/airace_camera_recovery.log"))
        for name, arguments, directory, log_path in jobs:
            with open(log_path, "ab") as log:
                with open(os.devnull, "rb") as stdin:
                    process = subprocess.Popen(arguments, cwd=directory, stdin=stdin,
                                               stdout=log, stderr=log, close_fds=True,
                                               preexec_fn=os.setsid)
            time.sleep(0.5)
            if process.poll() is not None:
                raise RuntimeError("%s启动失败，请检查 %s" % (name, log_path))
            emit("runtime", {"component": name, "pid": process.pid, "message": "已启动"})
        # 防止另一个连接在 ROS 发布者尚未注册时再次启动同一组件。
        if jobs:
            deadline = monotonic() + 8.0
            while monotonic() < deadline:
                code, message, system = rospy.get_master().getSystemState()
                if code == 1:
                    publishers = dict(system[0])
                    if publishers.get("/airace/state") and (
                            publishers.get("/camera/depth/image_rect_raw")
                            or publishers.get("/camera/aligned_depth_to_color/image_raw")):
                        return
                time.sleep(0.1)
            raise RuntimeError("执行桥接或深度发布者未就绪，请检查机载启动日志")


def _send_vendor(code, value=0):
    """由机载端（.103）向运动主机发 12 字节简单指令。"""
    if READ_ONLY:
        raise RuntimeError("Read-only receiver")
    packet = struct.pack("<3i", int(code), int(value), 0)
    with _vendor_lock:
        if _vendor_socket[0] is None:
            _vendor_socket[0] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _vendor_socket[0].sendto(packet, VENDOR_ADDRESS)


def _begin_command(name):
    """标记命令窗口开始；同一时刻只允许一个姿态/急停窗口。"""
    with _command_busy_lock:
        if _command_busy[0]:
            return False
        _command_busy[0] = True
        return True


def _end_command():
    with _command_busy_lock:
        _command_busy[0] = False


# ---------------------------------------------------------------------------
# 传感器回调
# ---------------------------------------------------------------------------
def image_received(message, name):
    now = time.time()
    if now - sensor_times.get(name, 0) < 0.5:
        return
    sensor_times[name] = now
    if message.encoding == "16UC1":
        data = np.ndarray((message.height, message.width),
                          dtype=">u2" if message.is_bigendian else "<u2",
                          buffer=message.data,
                          strides=(message.step, 2)).copy()
        valid = data[data > 0]
        preview = cv2.applyColorMap(
            np.uint8(np.clip(data / 5000.0, 0, 1) * 255), cv2.COLORMAP_JET)
        preview[data == 0] = 0
        extra = {"valid_percent": round(float(valid.size) / data.size * 100, 1),
                 "median_m": round(float(np.median(valid)) / 1000, 3)
                 if valid.size else None}
    else:
        preview = np.ndarray((message.height, message.width, 3),
                             dtype=np.uint8, buffer=message.data,
                             strides=(message.step, 3, 1)).copy()
        if message.encoding == "rgb8":
            preview = cv2.cvtColor(preview, cv2.COLOR_RGB2BGR)
        extra = {}
    ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if ok:
        extra.update({"name": name, "width": message.width,
                      "height": message.height,
                      "stamp": message.header.stamp.to_sec(),
                      "jpeg": base64.b64encode(encoded.tobytes()).decode("ascii")})
        emit("sensor", extra)


def depth_image_received(message):
    """任意深度话题的新鲜帧都刷新深度心跳（预览共用 "depth" 通道）。"""
    depth_time[0] = monotonic()
    image_received(message, "depth")


def raw_packet_received(message):
    """解析 /airace/raw_packet：只提取厂商复杂上报（关节温度/角度/角速度）。

    这些包由厂商按需主动上报（温度仅过温时），因此 UI 必须允许"无数据"。
    频控 5 Hz，避免 SSH 单流被 50 Hz 原始包占满。
    """
    now = time.time()
    if now - sensor_times.get("raw_packet", 0.0) < 0.2:
        return
    sensor_times["raw_packet"] = now
    try:
        packet = binascii.unhexlify(str(message.data).strip())
    except (TypeError, ValueError, binascii.Error):
        return
    if len(packet) < 12:
        return
    try:
        code, size, kind = struct.unpack_from("<3i", packet, 0)
    except struct.error:
        return
    if kind != 1 or size <= 0 or size % 4 or len(packet) < 12 + size:
        return
    if code == 0x11050400 and size == 48:
        values = list(struct.unpack_from("<12i", packet, 12))
        emit("health", {"joint_temperature_c": values, "max_c": max(values),
                        "source": "0x11050400", "stamp": now})
    elif code == 0x0902 and size == 96:
        values = list(struct.unpack_from("<12d", packet, 12))
        if all(_finite(value) for value in values):
            emit("health", {"joint_angles_rad": values, "source": "0x0902",
                            "stamp": now})
    elif code == 0x0903 and size == 96:
        values = list(struct.unpack_from("<12d", packet, 12))
        if all(_finite(value) for value in values):
            emit("health", {"joint_velocities_rad_s": values, "source": "0x0903",
                            "stamp": now})


def temperature_received(message):
    """板载/雷达温度（/temperature，Float32，cloud_node 发布）。

    只读模式下也允许采集；用于健康卡展示，不参与任何运动判断。
    """
    now = time.time()
    if now - sensor_times.get("temperature", 0.0) < 1.0:
        return
    sensor_times["temperature"] = now
    try:
        value = float(message.data)
    except (TypeError, ValueError):
        return
    if _finite(value):
        emit("health", {"temperature_c": round(value, 1),
                        "temperature_source": "/temperature", "stamp": now})


def cloud_received(message):
    now = time.time()
    if now - sensor_times.get("lidar", 0) < 0.5:
        return
    sensor_times["lidar"] = now
    total = message.width * message.height
    stride = max(1, total // 1800)
    points = []
    valid = 0
    for point in point_cloud2.read_points(message,
                                          field_names=("x", "y", "z"),
                                          skip_nans=True):
        valid += 1
        if valid % stride == 0:
            points.append([round(float(v), 3) for v in point])
    emit("sensor", {"name": "lidar", "points": points, "total": total,
                    "valid": valid, "frame": message.header.frame_id,
                    "stamp": message.header.stamp.to_sec()})


def odometry_received(message):
    # /odom 常常 20~50 Hz；localization 回执节流到 ≤10 Hz，避免占满 SSH 单流
    # （TF 查询仍在回调内保留，只是不再每帧都做）。
    now = monotonic()
    if now - last_localization_emit[0] < LOCALIZATION_EMIT_PERIOD_S:
        return
    try:
        transform_time = transforms.getLatestCommonTime("map", message.header.frame_id)
        transform_age = rospy.Time.now().to_sec() - transform_time.to_sec()
        translation, rotation = transforms.lookupTransform("map", message.header.frame_id,
                                                           message.header.stamp)
        matrix = tf.transformations.quaternion_matrix(rotation)
        point = message.pose.pose.position
        mapped = np.dot(matrix[:3, :3], [point.x, point.y, point.z]) + translation
        q = message.pose.pose.orientation
        orientation = tf.transformations.quaternion_multiply(rotation, [q.x, q.y, q.z, q.w])
        yaw = math.degrees(tf.transformations.euler_from_quaternion(orientation)[2])
        age = rospy.Time.now().to_sec() - message.header.stamp.to_sec()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    last_localization_emit[0] = now
    emit("localization", {"x": float(mapped[0]), "y": float(mapped[1]),
                          "yaw_deg": yaw, "age_s": age, "frame": "map",
                          "transform_age_s": transform_age,
                          "valid": 0.0 <= age < 0.8 and 0.0 <= transform_age < 2.0,
                          "source": "hdl_graph_slam"})


# ---------------------------------------------------------------------------
# 状态 / 执行 / 地图回调
# ---------------------------------------------------------------------------
def state_received(message):
    now = time.time()
    try:
        payload = json.loads(message.data)
    except (TypeError, ValueError):
        return
    latest_basic[0] = payload.get("basic")
    if now - last_state[0] >= 0.1:
        last_state[0] = now
        emit("state", payload)


def execution_received(message):
    try:
        report = json.loads(message.data)
    except (TypeError, ValueError):
        return
    # 机载侧深度帧龄（monotonic），上位机直接展示，避免跨主机时钟漂移
    if depth_time[0] > 0.0:
        report["host_ui_depth_age_s"] = round(monotonic() - depth_time[0], 3)
    else:
        report["host_ui_depth_age_s"] = None
    emit("execution", report)


def map_received(message):
    info = message.info
    q = info.origin.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))
    stamp = message.header.stamp.to_sec()
    age = rospy.Time.now().to_sec() - stamp
    emit("map", {"width": info.width, "height": info.height,
                 "resolution": info.resolution,
                 "origin": [info.origin.position.x, info.origin.position.y, yaw],
                 "data": list(message.data), "name": message.header.frame_id,
                 "stamp": stamp, "age_s": age})


# ---------------------------------------------------------------------------
# 控制命令（子线程执行，不阻塞主循环的 stdin/publish）
# ---------------------------------------------------------------------------
def _service_worker(action):
    service = "/airace/prepare" if action == "prepare" else "/airace/disarm"
    success = False
    message = ""
    try:
        rospy.wait_for_service(service, timeout=SERVICE_WAIT_S)
        proxy = rospy.ServiceProxy(service, Trigger)
        response = proxy()
        success = bool(response.success)
        message = str(response.message)
    except Exception as error:  # 服务缺失/超时/异常都要回执，避免上位机干等
        success = False
        message = "%s 服务调用失败: %s" % (service, error)
    emit("control", {"action": action, "success": success, "message": message})


def _posture_worker(target, request_id):
    """站立/趴下：交给机载执行桥接服务（单一控制会话 + 持续心跳）。"""
    service = "/airace/stand" if target == "stand" else "/airace/lie"
    target_basic = POSTURE_TARGETS.get(target)
    if target_basic is None:
        emit("control", {"action": "posture", "target": target, "request_id": request_id,
                         "success": False,
                         "message": "姿态目标无效，必须是 stand 或 lie"})
        return
    try:
        rospy.wait_for_service(service, timeout=SERVICE_WAIT_S)
        response = rospy.ServiceProxy(service, Trigger)()
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        if success:
            deadline = monotonic() + 3.0
            while monotonic() < deadline:
                if latest_basic[0] == target_basic:
                    break
                time.sleep(0.05)
            basic = latest_basic[0]
            if basic != target_basic:
                success = False
                message = "机载服务返回成功但 basic 未确认（当前 %s）" % basic
            else:
                message = message or "姿态已确认（basic=%s）" % basic
        emit("control", {"action": "posture", "target": target, "request_id": request_id,
                         "success": success, "basic": latest_basic[0],
                         "message": message})
    except Exception as error:
        emit("control", {"action": "posture", "target": target, "request_id": request_id,
                         "success": False, "basic": latest_basic[0],
                         "message": ("机载执行桥接未运行或姿态服务不可用：%s；"
                                     "先启动 lite2_live.py bridge" % error)})


def _estop_worker():
    """急停优先走机载桥接服务；服务不可用时直接发厂商急停兜底。"""
    try:
        rospy.wait_for_service("/airace/estop", timeout=1.0)
        response = rospy.ServiceProxy("/airace/estop", Trigger)()
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
    except Exception:
        try:
            for _ in range(3):
                _send_vendor(ESTOP_CODE)
                time.sleep(0.05)
            success = True
            message = "机载 estop 服务不可用，已直接发送厂商急停 0x21010C0E x3"
        except Exception as error:
            success = False
            message = "急停发送失败：%s；请立即使用物理急停/遥控器" % error
    emit("control", {"action": "estop", "success": success,
                     "message": message})


def _start_worker(target, *args):
    worker = threading.Thread(target=target, args=args)
    worker.daemon = True
    worker.start()


def handle_command(data):
    """处理一条 stdin JSON 命令；非法输入静默忽略（不拆链）。"""
    if not isinstance(data, dict):
        return
    action = data.get("action")
    if READ_ONLY and action != "ping":
        return
    if action:
        if action in ("mapping_start", "mapping_stop"):
            _start_worker(mapping_worker, action)
        elif action in ("prepare", "disarm"):
            _start_worker(_service_worker, action)
        elif action == "posture":
            target = str(data.get("target", ""))
            request_id = str(data.get("request_id", ""))
            if _begin_command("posture"):
                _start_worker(_run_posture_guarded, target, request_id)
            else:
                emit("control", {"action": "posture", "target": target,
                                 "request_id": request_id,
                                 "success": False,
                                 "message": "机载端已有姿态/急停命令在执行"})
        elif action == "estop":
            if _begin_command("estop"):
                _start_worker(_run_estop_guarded)
            else:
                # 急停永远要尝试，不因窗口占用而丢弃（另起任务仍发一次）
                _start_worker(_estop_worker)
        elif action == "ping":
            emit("heartbeat", {"stamp": time.time()})
        return
    try:
        linear = float(data["linear"])
        angular = float(data["angular"])
    except (KeyError, TypeError, ValueError):
        return
    if math.isnan(linear) or math.isinf(linear) or math.isnan(angular) or math.isinf(angular):
        return
    command.linear.x = linear
    command.angular.z = angular
    last_command[0] = monotonic()


def _run_posture_guarded(target, request_id):
    try:
        _posture_worker(target, request_id)
    finally:
        _end_command()


def _run_estop_guarded():
    try:
        _estop_worker()
    finally:
        _end_command()


def control_stream_owner(guard):
    """Resolve the actual flock owner and verify the project process identity."""
    inode = os.fstat(guard.fileno()).st_ino
    with open('/proc/locks') as locks:
        for line in locks:
            fields = line.split()
            if len(fields) < 6 or fields[1] != 'FLOCK':
                continue
            if int(fields[5].split(':')[-1]) != inode:
                continue
            pid = int(fields[4])
            directory = '/proc/%d' % pid
            if pid == os.getpid() or os.stat(directory).st_uid != os.getuid():
                raise RuntimeError('Control stream lock owner identity mismatch')
            with open(directory + '/cmdline', 'rb') as command:
                arguments = command.read()
            if b'host_ui_stream' not in arguments or b'--read-only' in arguments:
                raise RuntimeError('Control lock is held by an unidentified process')
            return pid
    return None


def acquire_control_stream_lock(guard):
    """Replace the previous project stream, waiting for its process to exit."""
    import fcntl

    deadline = monotonic() + 6.0
    terminated = None
    while True:
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return
        except (IOError, OSError):
            owner = control_stream_owner(guard)
            if owner is not None and owner != terminated:
                try:
                    os.kill(owner, signal.SIGTERM)
                except OSError:
                    if control_stream_owner(guard) is not None:
                        raise
                terminated = owner
                emit('runtime', {'component': '控制会话',
                                 'message': '已结束旧数据进程 PID %d' % owner})
            if monotonic() >= deadline:
                raise RuntimeError('旧数据进程尚未退出，请检查机载进程状态')
            time.sleep(0.1)


def begin_control_connection():
    """独占控制 stream，在连接阶段建立待命心跳；只读连接无控制副作用。"""
    if READ_ONLY:
        return None
    guard = open("/tmp/airace_control_stream.lock", "a")
    try:
        acquire_control_stream_lock(guard)
    except Exception:
        guard.close()
        raise
    acquired = False
    try:
        ensure_runtime()
        rospy.wait_for_service("/airace/acquire", timeout=5.0)
        result = rospy.ServiceProxy("/airace/acquire", Trigger)()
        if not result.success:
            raise RuntimeError("待命心跳建立失败：%s" % result.message)
        acquired = True
        emit("runtime", {"component": "控制会话", "message": str(result.message)})
        return guard
    except Exception:
        if acquired:
            end_control_connection(guard)
        else:
            guard.close()
        raise


def end_control_connection(guard):
    """退出时先交还会话再释放独占锁；失败必须留下诊断。"""
    if READ_ONLY or guard is None or guard.closed or _control_cleanup_started[0]:
        return
    _control_cleanup_started[0] = True
    result = []

    def release():
        try:
            rospy.wait_for_service("/airace/release", timeout=1.0)
            response = rospy.ServiceProxy("/airace/release", Trigger)()
            result.append((bool(response.success), str(response.message)))
        except Exception as error:
            result.append((False, str(error)))
        finally:
            guard.close()

    worker = threading.Thread(target=release)
    worker.daemon = True
    worker.start()
    worker.join(3.0)
    if not result or not result[0][0]:
        emit("runtime", {"component": "控制会话", "message":
             "断开时未确认会话释放：%s" % (result[0][1] if result else "服务响应超时")})


# ---------------------------------------------------------------------------
# ROS 节点 / 主循环
# ---------------------------------------------------------------------------
rospy.init_node("airace_host_ui", anonymous=True, disable_signals=True)
control_guard = begin_control_connection()
# 初始化中途失败也应释放；正常退出后 guard.closed 使该回调成为空操作。
atexit.register(end_control_connection, control_guard)
transforms = tf.TransformListener()
publisher = None if READ_ONLY else rospy.Publisher("/airace/cmd_vel", Twist, queue_size=1)
depth_publisher = None if READ_ONLY else rospy.Publisher("/airace/depth_received", Float64, queue_size=1)

rospy.Subscriber("/airace/state", String, state_received, queue_size=1)
rospy.Subscriber("/airace/execution", String, execution_received, queue_size=1)
rospy.Subscriber("/projected_map", OccupancyGrid, map_received, queue_size=1)
rospy.Subscriber("/odom", Odometry, odometry_received, queue_size=1)
rospy.Subscriber("/camera/color/image_raw", Image,
                 lambda message: image_received(message, "color"), queue_size=1)
# 对齐深度（lite2_live 使用）与旧版 rect 深度都接受，只要有一个新鲜即可
rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,
                 depth_image_received, queue_size=1)
rospy.Subscriber("/camera/depth/image_rect_raw", Image,
                 depth_image_received, queue_size=1)
rospy.Subscriber("/rslidar_points", PointCloud2, cloud_received, queue_size=1)
#: 板载/雷达温度（Float32），只用于健康展示
rospy.Subscriber("/temperature", Float32, temperature_received, queue_size=1)
#: 厂商原始上报（关节温度/角度/角速度），由 bridge 转成 hex 字符串
rospy.Subscriber("/airace/raw_packet", String, raw_packet_received, queue_size=20)

# 控制会话已 acquire、订阅已建立；雷达恢复在后台进行，失败不阻断待命心跳。
start_lidar_recovery()

try:
    emit("system", {"hostname": socket.gethostname(),
                    "python": platform.python_version(),
                    "rosdistro": os.environ.get("ROS_DISTRO", ""),
                    "ros_version": rospy.get_param("/rosversion", "")})
except Exception:
    pass
emit("ready", {})

command = Twist()
last_command = [monotonic()]
last_depth_publish = [0.0]
last_mapping_report = 0.0
try:
    while not rospy.is_shutdown():
        # 深度心跳：只有收到 0.5 s 内的新鲜深度帧才发布，断流立即停发
        now_mono = monotonic()
        if now_mono - last_mapping_report > 1.0:
            last_mapping_report = now_mono
            running = mapping_pid() is not None
            emit("mapping", {"state": "running" if running else "idle",
                             "message": "实时雷达建图" if running else "建图未运行"})
        if (not READ_ONLY and depth_time[0] > 0.0
                and now_mono - depth_time[0] <= DEPTH_FRESH_S
                and now_mono - last_depth_publish[0] >= DEPTH_PUBLISH_PERIOD_S):
            last_depth_publish[0] = now_mono
            depth_publisher.publish(Float64(data=float(now_mono)))
        readable, _, _ = select.select([sys.stdin], [], [], 0.04)
        if readable:
            line = sys.stdin.readline()
            if not line:
                break
            try:
                data = json.loads(line)
            except ValueError:
                continue
            handle_command(data)
        if not 0.0 <= monotonic() - last_command[0] <= 0.3:
            command = Twist()
        if not READ_ONLY:
            publisher.publish(command)
finally:
    if not READ_ONLY:
        publisher.publish(Twist())
    end_control_connection(control_guard)
    if _vendor_socket[0] is not None:
        try:
            _vendor_socket[0].close()
        except Exception:
            pass
    rospy.signal_shutdown("host UI disconnected")
