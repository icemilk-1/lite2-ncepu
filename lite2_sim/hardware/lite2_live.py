"""ROS1 RGB-D and vendor-state deployment, with a bounded straight-line trial.

Run bridge first, inspect preflight, then run mission on the perception host.
Vendor packet layouts follow Lite2 software interface V1.2.3.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "sensor_capture"))
sys.path.append("/opt/ros/melodic/lib/python2.7/dist-packages")
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, TriggerResponse
from navigation_core.contracts import NavigationObservation
from navigation_core.control import NavigationController
from navigation_core.planner import PlanResult
from vendor_protocol import decode_state, decode_command, velocity_packets
from execution_guard import (fresh, guarded_velocity, MAX_FORWARD_M_S,
                             MAX_REVERSE_M_S, MAX_YAW_RAD_S)

TRIAL_SETTINGS = dict(command_timeout_s=0.2, navigation_timeout_s=20.0,
                      observation_timeout_s=0.5, goal_tolerance_m=0.025,
                      front_stop_angle_rad=0.8, front_stop_distance_m=0.65,
                      max_linear_speed_m_s=0.05, max_yaw_speed_rad_s=0.1,
                      stuck_timeout_s=4.0, progress_distance_m=0.025,
                      lookahead_m=0.15)

#: 机载控制会话心跳周期（厂商要求 >1 Hz，这里 5 Hz）
CONTROL_HEARTBEAT_PERIOD_S = 0.2
#: 姿态确认超时
POSTURE_TIMEOUT_S = 20.0
#: 进入控制会话后等待心跳生效的时间（让运动主机完成终端登记）
CONTROL_SESSION_SETTLE_S = 0.3
#: 姿态请求的心跳预热时间，计入整个姿态操作的超时预算。
POSTURE_SETTLE_S = 3.0


def emit(event, **values):
    values.update(event=event)
    print(json.dumps(values, allow_nan=False), flush=True)


def bridge():
    # Own the receive socket before registering the fixed ROS node name.
    # A duplicate startup must fail before ROS can shut down the existing node.
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.bind(("127.0.0.1", 43898))
        receiver.settimeout(0.05)
    except Exception:
        receiver.close()
        raise
    rospy.init_node("airace_vendor_bridge")
    velocity_limits = rospy.get_param("~velocity_limits", dict(
        max_forward_m_s=MAX_FORWARD_M_S, max_reverse_m_s=MAX_REVERSE_M_S,
        max_yaw_rad_s=MAX_YAW_RAD_S))
    state_pub = rospy.Publisher("/airace/state", String, queue_size=2)
    raw_pub = rospy.Publisher("/airace/raw_packet", String, queue_size=128)
    diagnostic_pub = rospy.Publisher("/airace/execution", String, queue_size=10)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 保留当前部署的发送端口。厂商 bridge_server 的 sport 是目标端口，
    # 实际发送 socket 绑定 rport=43897；控制终端仲裁按源 IP 判断。
    vendor_source_port = 43893
    try:
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sender.bind(("0.0.0.0", vendor_source_port))
        emit("vendor_source_port", port=vendor_source_port)
    except OSError as error:
        emit("vendor_source_port", port=0, error=str(error))
    lock = threading.RLock()
    # ROS services run on separate threads. Safety cancellation must not wait for
    # a posture/prepare request, while ordinary control actions remain exclusive.
    operation_lock = threading.Lock()
    command = [0.0, 0.0, 0.0]
    state = {"received": 0.0, "basic": 0}
    feedback = {"joystick": [0.0, 0.0, 0.0], "goal_velocity": [0.0, 0.0, 0.0],
                "command_received": 0.0}
    control = dict(armed=False, mode_confirmed=False, depth_time=0.0,
                   automatic_ack=0.0, requested_at=0.0)
    #: 待命/未使能期间也持续心跳；仅明确释放、急停、遥控接管或退出停止。
    session = dict(active=False, heartbeat_sent=0, started_at=0.0,
                   last_heartbeat=0.0, paused_reason="", generation=0)

    def velocity(msg):
        with lock:
            command[:] = [msg.linear.x, msg.angular.z, time.monotonic()]

    rospy.Subscriber("/airace/cmd_vel", Twist, velocity, queue_size=1)

    def depth_heartbeat(message):
        with lock:
            control["depth_time"] = message.data

    rospy.Subscriber("/airace/depth_received", Float64, depth_heartbeat, queue_size=1)

    def mode_packet(code):
        sender.sendto(struct.pack("<3i", code, 0, 0), ("192.168.1.120", 43893))

    def start_session(reason=""):
        """建立/恢复机载控制会话（单一心跳源）。"""
        with lock:
            resumed = not session["active"]
            if resumed:
                # Send immediately: a newly connected controller must not wait
                # for the heartbeat thread's next scheduling slot.
                mode_packet(0x21040001)
                session.update(active=True, heartbeat_sent=0,
                               started_at=time.monotonic(),
                               last_heartbeat=time.monotonic(), paused_reason="",
                               generation=session["generation"] + 1)
                session["heartbeat_sent"] = 1
            generation = session["generation"]
        if resumed:
            emit("control_session", active=True, reason=reason)
        return generation

    def cancel_operations():
        """Invalidate sleeping workers without releasing the standby heartbeat."""
        with lock:
            session["generation"] += 1
            control.update(armed=False, mode_confirmed=False,
                           automatic_ack=0.0, requested_at=0.0)
            command[:] = [0.0, 0.0, 0.0]

    def operation_current(generation):
        return (session["active"] and session["generation"] == generation
                and not rospy.is_shutdown())

    def stop_session(reason=""):
        with lock:
            cancel_operations()
            was_active = session["active"]
            # A stream's normal cleanup must not erase a takeover/estop latch
            # and let its automatic reconnect silently acquire control again.
            if session["paused_reason"] in ("estop", "remote_takeover"):
                reason = session["paused_reason"]
            session.update(active=False, paused_reason=reason)
        if was_active:
            emit("control_session", active=False, reason=reason)

    def heartbeat_loop():
        """持续发送 0x21040001；姿态/站立待命期间不中断。"""
        while not rospy.is_shutdown():
            try:
                with lock:
                    if session["active"]:
                        mode_packet(0x21040001)
                        session["heartbeat_sent"] += 1
                        session["last_heartbeat"] = time.monotonic()
            except Exception as error:
                emit("heartbeat_error", error=str(error))
            time.sleep(CONTROL_HEARTBEAT_PERIOD_S)

    def acquire_service(_request):
        """Acquire standby heartbeats only; never arm or issue a posture toggle."""
        with lock:
            if session["paused_reason"] in ("estop", "remote_takeover"):
                return TriggerResponse(success=False,
                                       message="Control acquisition refused: %s; explicit operator recovery required"
                                               % session["paused_reason"])
            # A replacement stream can arrive after its predecessor was killed
            # without cleanup. Never inherit its armed state or pending action.
            cancel_operations()
            send(0.0, 0.0)
            if session["active"]:
                return TriggerResponse(success=True,
                                       message="Standby heartbeat continues; previous motion disarmed")
            start_session("control_connection")
        return TriggerResponse(success=True,
                               message="Standby heartbeat started; motion remains disarmed")

    def posture_service(target_basic, label):
        """站立/趴下服务：由单一控制会话发心跳 + 单次 0x21010202，轮询 basic。"""

        def handler(_request):
            if not operation_lock.acquire(False):
                return TriggerResponse(success=False, message="Another control action is in progress")
            try:
                return run_posture()
            finally:
                operation_lock.release()

        def run_posture():
            deadline = time.monotonic() + POSTURE_TIMEOUT_S
            with lock:
                current = state.get("basic")
                state_time = state.get("received")
                if not fresh(state_time, time.monotonic(), 0.3):
                    return TriggerResponse(success=False, message="Posture refused: stale robot state")
                if current not in (1, 6):
                    return TriggerResponse(
                        success=False,
                        message="Robot basic=%s, refuse posture change to %s" % (current, label))
                cancel_operations()
                send(0.0, 0.0)
                generation = start_session("posture_%s" % label)
                if current == target_basic:
                    return TriggerResponse(
                        success=True, message="Robot already %s (basic=%s)" % (label, current))
            time.sleep(POSTURE_SETTLE_S)
            with lock:
                current = state.get("basic")
                state_time = state.get("received")
                if not operation_current(generation) or not fresh(state_time, time.monotonic(), 0.3):
                    return TriggerResponse(success=False,
                                           message="Posture cancelled: session changed or state stale")
                if current == target_basic:
                    return TriggerResponse(success=True,
                                           message="%s confirmed (basic=%s)" % (label, current))
                if current not in (1, 6):
                    return TriggerResponse(success=False,
                                           message="Posture refused: basic changed to %s" % current)
                # Generation check and toggle are atomic with safety cancellation.
                # Every request sends this toggle at most once.
                mode_packet(0x21010202)
            while time.monotonic() < deadline and not rospy.is_shutdown():
                with lock:
                    current = state.get("basic")
                    state_time = state.get("received")
                    if not operation_current(generation) or not fresh(state_time, time.monotonic(), 0.3):
                        return TriggerResponse(success=False,
                                               message="Posture cancelled: session changed or state stale")
                    if current == target_basic:
                        return TriggerResponse(
                            success=True,
                            message="%s confirmed (basic=%s), heartbeat continues" % (label, current))
                if current in (8, 9, 11):
                    return TriggerResponse(
                        success=False,
                        message="Robot entered protection state basic=%s "
                                "during %s; stop and recover manually"
                                % (current, label))
                time.sleep(0.05)
            with lock:
                current = state.get("basic")
            if target_basic == 6 and current == 1:
                return TriggerResponse(
                    success=False,
                    message="站立指令未被执行，机器人仍正常趴下(basic=1)。"
                            "可能被手机或旧终端占用控制权；请退出手机App并检查控制终端登记。"
                            "待命心跳继续，未重复发送站趴指令。")
            return TriggerResponse(
                success=False,
                message="Posture timeout: expected basic=%s, current basic=%s"
                        % (target_basic, current))

        return handler

    def estop_service(_request):
        with lock:
            stop_session("estop")
            send(0.0, 0.0)
            mode_packet(0x21010C0E)
        for _ in range(2):
            time.sleep(0.02)
            with lock:
                mode_packet(0x21010C0E)
        return TriggerResponse(success=True,
                               message="Vendor estop 0x21010C0E sent x3")

    def release_service(_request):
        """安全释放控制会话：零速 + 手动模式，不发送站立/姿态指令。"""
        with lock:
            stop_session("release")
            send(0.0, 0.0)
            mode_packet(0x21010C02)  # 手动模式：交还控制权
        return TriggerResponse(
            success=True,
            message="Manual mode requested; control session released (no posture command)")

    def prepare(_):
        if not operation_lock.acquire(False):
            return TriggerResponse(success=False, message="Another control action is in progress")
        try:
            return run_prepare()
        finally:
            operation_lock.release()

    def run_prepare():
        with lock:
            cancel_operations()
            generation = start_session("prepare")
        time.sleep(CONTROL_SESSION_SETTLE_S)
        with lock:
            if not operation_current(generation):
                return TriggerResponse(success=False, message="Prepare cancelled: control session changed")
            now = time.monotonic()
            # Only a newly received ACK for this request can enable motion.
            control.update(automatic_ack=0.0, requested_at=now)
            send(0.0, 0.0)
            mode_packet(0x21010D06)
            mode_packet(0x21010C03)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if not operation_current(generation):
                    return TriggerResponse(success=False, message="Prepare cancelled: control session changed")
                confirmed = (control["automatic_ack"] > 0.0
                             and control["automatic_ack"] >= control["requested_at"])
                if confirmed:
                    control.update(armed=True, mode_confirmed=True)
                    return TriggerResponse(success=True, message="Automatic-mode acknowledgement received")
            time.sleep(0.02)
        with lock:
            if operation_current(generation):
                control.update(armed=False, mode_confirmed=False)
                send(0.0, 0.0)
                mode_packet(0x21010C02)
        return TriggerResponse(success=False, message="No automatic-mode acknowledgement in 3 seconds")

    def disarm(_):
        with lock:
            cancel_operations()
            send(0.0, 0.0)
            mode_packet(0x21010C02)
            mode_packet(0x21010D05)
        return TriggerResponse(success=True,
                               message="Zero velocity sent and manual standing requested; standby heartbeat unchanged")

    def send(v, w):
        for packet in velocity_packets(v, w):
            sender.sendto(packet, ("192.168.1.120", 43893))

    rospy.Service("/airace/acquire", Trigger, acquire_service)
    rospy.Service("/airace/prepare", Trigger, prepare)
    rospy.Service("/airace/disarm", Trigger, disarm)
    rospy.Service("/airace/stand", Trigger, posture_service(6, "standing"))
    rospy.Service("/airace/lie", Trigger, posture_service(1, "lying"))
    rospy.Service("/airace/estop", Trigger, estop_service)
    rospy.Service("/airace/release", Trigger, release_service)

    def watchdog():
        last_reason = None
        try:
            while not rospy.is_shutdown():
                now = time.monotonic()
                with lock:
                    request = list(command)
                    current = dict(state)
                    current.update(feedback)
                    takeover = (fresh(feedback.get("command_received"), now, 0.3)
                                and max(abs(v) for v in feedback.get("joystick", [0.0])) >= 0.03)
                    if takeover and session["active"]:
                        stop_session("remote_takeover")
                    v, w, reason = guarded_velocity(request, now, current,
                                                    control["armed"], control["mode_confirmed"],
                                                    control["depth_time"], velocity_limits)
                    # Do not let a velocity computed before release/estop be
                    # sent after that cancellation acquired the same lock.
                    send(v, w)
                depth_age = (now - control["depth_time"]
                             if control["depth_time"] > 0 else None)
                with lock:
                    session_snapshot = dict(session)
                session_snapshot["heartbeat_age_s"] = (
                    round(now - session_snapshot["last_heartbeat"], 3)
                    if session_snapshot["active"]
                    and session_snapshot["last_heartbeat"] > 0 else None)
                diagnostic = dict(received_monotonic=now, requested=request,
                                  limits=velocity_limits,
                                  sent=[v, w], reason=reason, state=current,
                                  depth_age_s=(None if depth_age is None
                                               else round(depth_age, 3)),
                                  depth_fresh=(depth_age is not None
                                               and 0.0 <= depth_age <= 0.5),
                                  control_session=session_snapshot,
                                  source_port=sender.getsockname()[1],
                                  control=dict(control),
                                  packets_hex=[p.hex() for p in velocity_packets(v, w)])
                diagnostic_pub.publish(String(data=json.dumps(diagnostic)))
                if reason != last_reason:
                    emit("execution_gate", **diagnostic)
                    last_reason = reason
                time.sleep(0.04)
        except Exception as failure:
            emit("bridge_failure", error=str(failure))
            rospy.signal_shutdown("Execution watchdog failed")
        finally:
            for _ in range(5):
                send(0.0, 0.0)
                time.sleep(0.02)

    thread = threading.Thread(target=watchdog)
    thread.start()
    heartbeat_thread = threading.Thread(target=heartbeat_loop)
    heartbeat_thread.daemon = True
    heartbeat_thread.start()
    emit("bridge_started", command_topic="/airace/cmd_vel")
    try:
        while not rospy.is_shutdown():
            try:
                packet, _ = receiver.recvfrom(2048)
            except socket.timeout:
                continue
            received = time.monotonic()
            raw_pub.publish(String(data=json.dumps(dict(received_monotonic=received,
                                                       packet_hex=packet.hex()))))
            if len(packet) == 12:
                code, value, kind = struct.unpack("<3i", packet)
                if code == 0x21010C03 and kind == 0:
                    with lock:
                        control["automatic_ack"] = received
                elif code == 0x21010C02 and kind == 0:
                    with lock:
                        control.update(armed=False, mode_confirmed=False, automatic_ack=0.0)
            command_report = decode_command(packet)
            if command_report is not None:
                with lock:
                    feedback.update(command_report, command_received=time.monotonic())
                continue
            report = decode_state(packet)
            if report is None:
                continue
            report["received"] = time.monotonic()
            with lock:
                report.update(feedback)
                report["automatic_ack_monotonic"] = control["automatic_ack"]
                state.update(report)
            state_pub.publish(String(data=json.dumps(report)))
    finally:
        rospy.signal_shutdown("bridge exit")
        stop_session("bridge_exit")
        thread.join(2)
        release_service(None)
        receiver.close()
        sender.close()


class Inputs(object):
    def __init__(self):
        self.state = None
        self.state_time = 0.0
        self.depth = None
        self.depth_time = 0.0
        self.depth_stamp_s = None
        self.last_depth_stamp_s = None
        self.lock = threading.Lock()
        self.camera = None
        self.color = None
        rospy.Subscriber("/airace/state", String, self.on_state, queue_size=1)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,
                         self.on_depth, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo,
                         self.on_camera, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.on_color, queue_size=1)

    def on_state(self, message):
        self.state = json.loads(message.data)
        self.state_time = self.state.get("received", 0.0)

    def on_depth(self, message):
        received = time.monotonic()
        stamp_s = message.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp_s
        if (not fresh(stamp_s, rospy.Time.now().to_sec(), 0.5)
                or self.last_depth_stamp_s is not None and stamp_s <= self.last_depth_stamp_s):
            return
        if message.encoding != "16UC1":
            return
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        data = np.ndarray((message.height, message.width), dtype=dtype,
                          buffer=message.data, strides=(message.step, 2))
        with self.lock:
            self.depth = data.astype(np.float64) * 0.001
            self.depth_time = received - age
            self.depth_stamp_s = stamp_s
            self.last_depth_stamp_s = stamp_s

    def on_camera(self, message):
        self.camera = list(message.K)

    def on_color(self, message):
        self.color = message

    def ready(self):
        now = time.monotonic()
        return (self.state is not None and self.depth is not None and self.camera is not None
                and fresh(self.state_time, now, 0.3)
                and fresh(self.state.get("command_received"), now, 0.3)
                and fresh(self.depth_time, now, 0.5))

    def wait(self):
        deadline = time.monotonic() + 15
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.ready():
                return
            time.sleep(0.05)
        raise RuntimeError("Fresh vendor state, aligned depth and camera intrinsics required")

    def cloud(self):
        with self.lock:
            full_depth = self.depth
            depth_time = self.depth_time
            stamp_s = self.depth_stamp_s
        depth = full_depth[::4, ::4]
        yy, xx = np.mgrid[0:full_depth.shape[0]:4, 0:full_depth.shape[1]:4]
        valid = (depth > 0.15) & (depth < 3.0)
        z = depth[valid]
        k = self.camera
        points = np.column_stack(((xx[valid] - k[2]) * z / k[0],
                                  (yy[valid] - k[5]) * z / k[4], z))
        return points, depth_time, stamp_s, float(np.mean(valid))

    def snapshot(self, output):
        import cv2
        with self.lock:
            depth = self.depth
            metadata = dict(camera=self.camera, state=self.state,
                            depth_timestamp_s=self.depth_time,
                            depth_device_timestamp_s=self.depth_stamp_s,
                            captured_monotonic=time.monotonic())
        if self.color is not None:
            msg = self.color
            data = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                              buffer=msg.data, strides=(msg.step, 3, 1)).copy()
            if msg.encoding == "rgb8":
                data = data[:, :, ::-1]
            cv2.imwrite(str(output / "camera.png"), data)
        np.save(str(output / "depth.npy"), depth)
        (output / "snapshot.json").write_text(json.dumps(metadata, indent=2))


def floor_plane(points):
    """Estimate a visible floor plane for this supervised flat-ground trial."""
    candidates = points[points[:, 1] > 0.0]
    if len(candidates) < 100:
        raise RuntimeError("Insufficient floor depth samples")
    random = np.random.RandomState(42)
    best = None
    best_count = 0
    for _ in range(150):
        a, b, c = candidates[random.choice(len(candidates), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-6:
            continue
        normal /= length
        if normal[1] > 0:
            normal *= -1
        offset = -float(np.dot(normal, a))
        if normal[1] > -0.5 or not 0.08 < offset < 0.9:
            continue
        count = int(np.sum(np.abs(candidates.dot(normal) + offset) < 0.025))
        if count > best_count:
            best, best_count = (normal, offset), count
    if best is None or best_count < len(candidates) * 0.4:
        raise RuntimeError("No reliable floor plane")
    return best


def observation(inputs, plane):
    points, received, stamp_s, valid_fraction = inputs.cloud()
    normal, offset = plane
    height = points.dot(normal) + offset
    obstacles = points[(height > 0.06) & (height < 0.9)
                       & (np.abs(points[:, 0]) < 0.4)]
    rays = [(float(p[2]), float(-p[0]), True) for p in obstacles]
    return NavigationObservation(received, "camera_forward", rays,
                                 valid_fraction > 0.15, "realsense_rgbd",
                                 dict(device_timestamp_s=stamp_s)), valid_fraction


def pulse(inputs, publisher, depth_pub, output, plane, duration):
    started = time.monotonic()
    initial = list(inputs.state["position"][:2])
    records = []
    error = None
    try:
        while time.monotonic() - started < duration:
            obs, _ = observation(inputs, plane)
            if time.monotonic() - started >= duration:
                break
            if (not inputs.ready() or not obs.valid or inputs.state["error_state"]
                    or max(abs(v) for v in inputs.state["joystick"]) >= 0.03
                    or max(abs(v) for v in inputs.state["rpy_deg"][:2]) > 18
                    or np.linalg.norm(inputs.state["velocity_body"][:2]) > 0.18):
                raise RuntimeError("Calibration pulse interrupted by input or mode state")
            if any(p[0] < 0.65 for p in obs.points):
                raise RuntimeError("Forward stopping region is obstructed")
            message = Twist()
            message.linear.x = 0.02
            depth_pub.publish(Float64(data=obs.timestamp_s))
            publisher.publish(message)
            records.append(dict(elapsed_s=time.monotonic()-started,
                                state=dict(inputs.state), command_m_s=0.02))
            time.sleep(0.02)
    except Exception as failure:
        error = str(failure)
    finally:
        zero_started = time.monotonic()
        for _ in range(100):
            publisher.publish(Twist())
            records.append(dict(elapsed_s=time.monotonic()-started,
                                state=dict(inputs.state), command_m_s=0.0))
            time.sleep(0.02)
        final = list(inputs.state["position"][:2])
        last_positions = np.array([r["state"]["position"][:2] for r in records[-25:]])
        stop_span = float(np.linalg.norm(np.ptp(last_positions, axis=0)))
        feedback_seen = any(0.005 <= r["state"]["goal_velocity"][0] <= 0.04
                            for r in records)
        feedback_zero = max(abs(v) for v in inputs.state["goal_velocity"]) < 0.005
        result = dict(error=error, feedback_seen=feedback_seen, feedback_zero=feedback_zero,
                      initial_xy=initial, final_xy=final,
                      displacement_m=float(np.linalg.norm(np.array(final)-initial)),
                      stopping_position_span_m=stop_span, records=records,
                      maximum_duration_s=duration,
                      zero_started_monotonic=zero_started)
        (output / "pulse.json").write_text(json.dumps(result, indent=2))
        emit("pulse_finished", error=error, feedback_seen=feedback_seen,
             feedback_zero=feedback_zero, stopping_position_span_m=stop_span)
    if error or not feedback_seen or not feedback_zero or stop_span >= 0.015:
        raise RuntimeError("Calibration pulse acceptance failed")


def trial(args):
    rospy.init_node("airace_trial", anonymous=True)
    inputs = Inputs()
    publisher = rospy.Publisher("/airace/cmd_vel", Twist, queue_size=1)
    depth_pub = rospy.Publisher("/airace/depth_received", Float64, queue_size=1)
    inputs.wait()
    args.output.mkdir(parents=True, exist_ok=True)
    inputs.snapshot(args.output)
    emit("preflight", state=inputs.state, depth_shape=list(inputs.depth.shape),
         valid_fraction=float(np.mean(inputs.depth > 0)), camera=inputs.camera)
    if args.role == "preflight":
        return
    if inputs.state["basic"] != 6:
        raise RuntimeError("Robot must be standing in force-control mode before the trial")
    publishers, _, _ = rospy.get_master().getSystemState()[2]
    owners = next((nodes for topic, nodes in publishers if topic == "/airace/cmd_vel"), [])
    if owners != [rospy.get_name()]:
        raise RuntimeError("The trial must be the sole /airace/cmd_vel publisher")
    positions = []
    for _ in range(20):
        if (not inputs.ready() or
                inputs.state.get("error_state", 0)
                or max(abs(v) for v in inputs.state["joystick"]) >= 0.03):
            raise RuntimeError("Fresh feedback and centered remote joysticks required")
        positions.append(inputs.state["position"][:2])
        time.sleep(0.1)
    if np.linalg.norm(np.ptp(np.array(positions), axis=0)) > 0.025:
        raise RuntimeError("Robot must be stationary before starting the trial")
    plane = floor_plane(inputs.cloud()[0])
    emit("floor", normal=plane[0].tolist(), offset_m=plane[1])
    if args.role == "pulse":
        pulse(inputs, publisher, depth_pub, args.output, plane, args.pulse_duration)
        return
    settings = dict(TRIAL_SETTINGS)
    controller = NavigationController(settings)
    plan = PlanResult("path_found", [(0.0, 0.0), (0.5, 0.0)], (0.5, 0.0), (0.5, 0.0))
    origin = inputs.state["position"][:2]
    yaw0 = math.radians(inputs.state["rpy_deg"][2])
    started = time.monotonic()
    status = "starting"
    records = []
    events = (args.output / "events.jsonl").open("x", buffering=1)
    events.write(json.dumps(dict(event="configuration", settings=settings,
                                goal=[0.5, 0.0], origin_xy=origin, yaw0_rad=yaw0)) + "\n")
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            if not inputs.ready() or inputs.state["basic"] != 6:
                raise RuntimeError("Lost fresh sensors or standing mode")
            if max(abs(v) for v in inputs.state["joystick"]) >= 0.03:
                raise RuntimeError("Remote joystick takeover detected")
            if np.linalg.norm(inputs.state["velocity_body"][:2]) > 0.18:
                raise RuntimeError("Measured speed exceeded trial limit")
            if max(abs(v) for v in inputs.state["rpy_deg"][:2]) > 18:
                raise RuntimeError("Body tilt exceeded trial limit")
            dx, dy = np.array(inputs.state["position"][:2]) - origin
            x = float(dx * math.cos(yaw0) + dy * math.sin(yaw0))
            y = float(-dx * math.sin(yaw0) + dy * math.cos(yaw0))
            yaw = math.radians(inputs.state["rpy_deg"][2]) - yaw0
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            if abs(y) > 0.12 or x < -0.08 or x > 0.6 or abs(yaw) > 0.3:
                raise RuntimeError("Measured motion exceeded the 0.5 m trial envelope")
            obs, fraction = observation(inputs, plane)
            now = time.monotonic()
            decision = controller.update(now, (x, y, yaw), plan, obs, now - started)
            status = decision.status
            command = decision.command
            if status == "navigating" and command.linear_x_m_s == 0:
                raise RuntimeError("Depth obstacle in the forward stopping region")
            msg = Twist()
            msg.linear.x = min(command.linear_x_m_s, max(0.015, (0.5 - x) * 0.5))
            msg.angular.z = command.angular_z_rad_s
            depth_pub.publish(Float64(data=obs.timestamp_s))
            publisher.publish(msg)
            row = dict(elapsed_s=now-started, x_m=x, y_m=y, yaw_rad=yaw,
                       status=status, command_m_s=msg.linear.x, valid_fraction=fraction,
                       measured_m_s=inputs.state["velocity_body"][0])
            row["goal_velocity"] = inputs.state["goal_velocity"]
            events.write(json.dumps(dict(event="control_tick", now_s=now,
                                         elapsed_s=now-started, pose=[x, y, yaw],
                                         observation=dict(timestamp_s=obs.timestamp_s,
                                                          frame_id=obs.frame_id, points=obs.points,
                                                          valid=obs.valid, source=obs.source,
                                                          metadata=obs.metadata),
                                         status=status,
                                         core_command=[command.linear_x_m_s, command.angular_z_rad_s],
                                         sent_command=[msg.linear.x, msg.angular.z],
                                         state=inputs.state)) + "\n")
            records.append(row)
            if len(records) % 10 == 0:
                emit("progress", **row)
            if status != "navigating":
                break
            time.sleep(0.05)
    except Exception as error:
        status = "failed"
        emit("trial_error", error=str(error))
        raise
    finally:
        stop_positions = []
        for _ in range(20):
            publisher.publish(Twist())
            stop_positions.append(list(inputs.state["position"][:2]))
            time.sleep(0.05)
        stop_span = float(np.linalg.norm(np.ptp(np.array(stop_positions[5:]), axis=0)))
        stopped = (inputs.ready() and stop_span < 0.015
                   and max(abs(v) for v in inputs.state["goal_velocity"]) < 0.005
                   and np.linalg.norm(inputs.state["velocity_body"][:2]) < 0.06)
        result = dict(status=status, stopped=bool(stopped), records=records,
                      final_state=inputs.state, origin_xy=origin,
                      goal_distance_m=0.5, floor_normal=plane[0].tolist(),
                      floor_offset_m=plane[1], stopping_position_span_m=stop_span)
        result["现场确认"] = "pending"
        result["外部距离复核"] = "pending"
        final_dx, final_dy = np.array(inputs.state["position"][:2]) - origin
        result["final_forward_m"] = float(final_dx * math.cos(yaw0) + final_dy * math.sin(yaw0))
        result["final_lateral_m"] = float(-final_dx * math.sin(yaw0) + final_dy * math.cos(yaw0))
        result["distance_within_tolerance"] = 0.45 <= result["final_forward_m"] <= 0.55
        (args.output / "trial.json").write_text(json.dumps(result, indent=2))
        events.close()
        emit("trial_finished", status=status, stopped=bool(stopped), samples=len(records))
    if status != "goal_reached" or not stopped or not result["distance_within_tolerance"]:
        raise RuntimeError("Trial did not reach the goal and confirm a stop")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("bridge", "preflight", "pulse", "mission"))
    parser.add_argument("--output", type=Path, default=ROOT / "trial")
    parser.add_argument("--pulse-duration", type=float, choices=(0.2, 2.0), default=0.2)
    args = parser.parse_args()
    if args.role == "bridge":
        bridge()
    else:
        trial(args)


if __name__ == "__main__":
    main()
