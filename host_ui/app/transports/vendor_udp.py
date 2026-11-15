"""厂商 UDP 传输：Lite2 V1.2.3 状态接收 + 速度指令下发（实机主链路）。

协议与线程模型
--------------
* **接收**：绑定 ``bind_host:bind_port``（默认 ``0.0.0.0:43898``，与
  ``lite2_sim/hardware/lite2_live.py`` 的 bridge 一致），独立线程用
  ``select`` + ``recvfrom`` 收包；196 字节状态包解码成 :class:`Frame`，
  60 字节遥控反馈包解码后合并进后续帧的 ``joystick`` / ``goal_velocity``。
* **发送**：向 ``robot_ip:robot_port``（默认 ``192.168.1.120:43893``）成组
  发送 3 个 20 字节速度包。``send_velocity`` 只写入内存并把包交给 socket，
  **不做任何阻塞等待**，控制循环每 40 ms 调用一次也不会被拖慢。
* **安全看门狗**：独立线程每 50 ms 检查最后一次指令时间；超过
  ``zero_timeout_s``（默认 0.3 s）没有新指令，就以约 5 Hz 反复补发零速，
  即使上层进程卡死也能让机器狗停下（与 bridge 的 ``command_timeout`` 语义一致）。
* **状态过期降级**：超过 ``state_timeout_s``（默认 1.0 s）没有状态包时，
  ``status()`` 仍返回 ``connected=True``（链路本身没断），但 ``stale=True``
  并在 ``detail`` 中说明；``state_timeout`` 告警限频 1 次/2 s。
* **回调**：``on_frame`` 在接收线程触发，``on_event`` 可能在接收/看门狗/
  调用者线程触发，后端负责加锁；本模块不依赖回调重入行为，并吞掉回调异常
  以免一个界面异常杀死链路线程。
* **生命周期**：``start()`` 失败（端口占用、地址非法）抛
  :class:`TransportError`；``stop()`` 幂等，会先补发多次零速再关线程/套接字。

没有实机时也可以 ``start()``（UDP 绑定成功即可），因此离线联调、回环自测
和单元测试都不需要机器狗在线。

厂商上报地址与 Windows 直连
---------------------------
厂商文档（V1.2.3 1 节）写明运动主机把状态上报到**感知主机**
``192.168.1.103:43897``，本链路默认监听 ``43898`` 是为了匹配
``lite2_sim/hardware/lite2_live.py`` bridge 把原始包转发到本机 43898 的
约定。**Windows 直连 .120 收不到状态包**（也不在运动主机终端白名单里），
需要在 .103 上跑 bridge/中继；本链路本身只用于演示、排障和回环自测。

能力与演示限制
--------------
* ``capabilities`` 只声明 ``posture`` / ``vendor_estop`` / ``reconnect=False``：
  使能（自动模式确认）必须由机载 execution guard 完成，见 ``ssh_ros``。
* ``set_posture`` 按厂商文档只发**一次** ``0x21010202``（轮换指令，连发会
  反复起立/趴下），再轮询 basic 确认；Windows 直发只作演示/中继。
* ``emergency_stop`` 用本链路 UDP socket 发 3 次 ``0x21010C0E``。
"""
from __future__ import annotations

import math
import select
import socket
import threading
import time
from typing import Any

from ..protocol import (
    COMMAND_LENGTH,
    ESTOP,
    HEARTBEAT,
    POSTURE_SOURCE_STATES,
    POSTURE_TOGGLE,
    STATE_LENGTH,
    decode_command,
    decode_state,
    posture_target,
    safety_packets,
    simple_packet,
    velocity_packets,
)
from ..types import Frame
from .base import Transport, TransportCallbacks, TransportError

#: 前端连接面板的默认参数（键名不可变）
DEFAULT_OPTIONS: dict[str, Any] = {
    "robot_ip": "192.168.1.120",
    "robot_port": 43893,
    "bind_host": "0.0.0.0",
    "bind_port": 43898,
    "ready_basic": 6,
    "joystick_threshold": 0.03,
    "zero_timeout_s": 0.3,
    "state_timeout_s": 1.0,
}


def _as_int(value: Any, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def _as_float(value: Any, fallback: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


class VendorUdpTransport(Transport):
    """Lite2 厂商 UDP 链路。"""

    kind = "udp"
    label = "厂商 UDP"
    default_options = dict(DEFAULT_OPTIONS)
    #: 自动模式/使能需要机载 execution guard，纯 UDP 链路不声明
    capabilities = {"arm": False, "posture": True, "vendor_estop": True,
                    "sensors": False, "reconnect": False}

    #: 零速补发周期（约 5 Hz）
    ZERO_RESEND_PERIOD_S = 0.2
    #: 姿态窗口内心跳周期（厂商要求 >1 Hz，这里 5 Hz）
    POSTURE_HEARTBEAT_PERIOD_S = 0.2
    #: 接收线程 select 轮询周期（决定 stop() 最坏等待）
    RX_POLL_S = 0.1
    #: 状态告警限频
    WARN_INTERVAL_S = 2.0

    @staticmethod
    def available() -> bool:
        """UDP 是标准库能力，没有额外的第三方依赖，始终可用。"""
        return True

    def __init__(self, options: dict | None, callbacks: TransportCallbacks) -> None:
        super().__init__(options, callbacks)
        opts = self.options
        self.robot_ip = str(opts.get("robot_ip", DEFAULT_OPTIONS["robot_ip"]))
        self.robot_port = _as_int(opts.get("robot_port"), DEFAULT_OPTIONS["robot_port"])
        self.bind_host = str(opts.get("bind_host", DEFAULT_OPTIONS["bind_host"]))
        self.bind_port = _as_int(opts.get("bind_port"), DEFAULT_OPTIONS["bind_port"])
        self.ready_basic = _as_int(opts.get("ready_basic"), 6)
        self.joystick_threshold = _as_float(
            opts.get("joystick_threshold"), 0.03)
        self.zero_timeout_s = max(0.05, _as_float(
            opts.get("zero_timeout_s"), 0.3))
        self.state_timeout_s = max(0.1, _as_float(
            opts.get("state_timeout_s"), 1.0))
        # 遥控反馈（60 B）超过该时长未更新即清零（契约第 3.4 节）
        self.feedback_timeout_s = max(0.1, _as_float(
            opts.get("feedback_timeout_s"), 0.5))
        # 姿态切换窗口（同步执行，锁用于禁运动与并发保护）
        self.posture_lock = threading.Lock()
        self._posture_action = None
        self._last_error = ""
        if not 0 < self.robot_port <= 65535:
            raise TransportError(f"厂商机器人端口非法: {self.robot_port}")
        if not 0 <= self.bind_port <= 65535:
            raise TransportError(f"本地绑定端口非法: {self.bind_port}")

        self._lifecycle_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._event_lock = threading.Lock()

        self._rx_socket: socket.socket | None = None
        self._tx_socket: socket.socket | None = None
        self._rx_thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False

        self._bound = (self.bind_host, self.bind_port)
        self._last_state: dict | None = None
        self._last_state_time = 0.0
        self._feedback: dict | None = None
        self._feedback_time = 0.0
        self._last_cmd_time = 0.0
        self._last_zero_send = 0.0
        self._zero_active = True
        self._state_up_reported = False

        self._event_times: dict[str, float] = {}
        self._counters = {
            "rx_packets": 0,
            "state_frames": 0,
            "command_reports": 0,
            "ignored_packets": 0,
            "send_errors": 0,
        }

    # ------------------------------------------------------------------ 工具
    def _emit_event(self, level: str, code: str, message: str,
                    min_interval: float = 0.0) -> None:
        """线程安全地触发事件回调；``min_interval`` 用于限频。"""
        now = time.monotonic()
        if min_interval > 0:
            with self._event_lock:
                last = self._event_times.get(code)
                if last is not None and now - last < min_interval:
                    return
                self._event_times[code] = now
        try:
            self.callbacks.on_event(level, code, message)
        except Exception:  # 界面回调异常不能杀死链路线程
            pass

    def _safe_callback(self, callback, *args) -> None:
        if callback is None:
            return
        try:
            callback(*args)
        except Exception:  # pragma: no cover - 防御性
            pass

    @property
    def _target(self) -> tuple[str, int]:
        return self.robot_ip, self.robot_port

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> None:
        """绑定本地端口并启动接收线程与零速看门狗；失败抛 TransportError。"""
        with self._lifecycle_lock:
            if self._running:
                return
            rx_socket: socket.socket | None = None
            tx_socket: socket.socket | None = None
            try:
                rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                    rx_socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                rx_socket.bind((self.bind_host, self.bind_port))
                bound = rx_socket.getsockname()
                self._bound = (str(bound[0]), int(bound[1]))
                tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except OSError as error:
                for item in (rx_socket, tx_socket):
                    if item is not None:
                        try:
                            item.close()
                        except OSError:
                            pass
                raise TransportError(
                    f"UDP 绑定 {self.bind_host}:{self.bind_port} 失败：{error}；"
                    f"请检查端口是否被占用或换用其它 bind_port") from error
            self._rx_socket = rx_socket
            self._tx_socket = tx_socket
            self._stop_event.clear()
            self._last_cmd_time = time.monotonic()
            self._last_zero_send = 0.0
            self._zero_active = True
            self._state_up_reported = False
            self._running = True
            self._rx_thread = threading.Thread(
                target=self._rx_loop, name="lite2-udp-rx", daemon=True)
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_loop, name="lite2-udp-watchdog", daemon=True)
            self._rx_thread.start()
            self._watchdog_thread.start()
        self._emit_event(
            "info", "link_up",
            f"UDP 已监听 {self._bound[0]}:{self._bound[1]}，"
            f"速度指令发往 {self.robot_ip}:{self.robot_port}")

    def stop(self) -> None:
        """幂等停止：补发零速、停线程、关套接字。"""
        with self._lifecycle_lock:
            if not self._running:
                self._close_sockets()
                return
            self._running = False
            self._stop_event.set()
            # 先补发零速（多次，容忍单次丢包），再断链路
            for _ in range(3):
                self._send_packets(safety_packets())
                time.sleep(0.02)
            self._close_sockets()
            for thread in (self._watchdog_thread, self._rx_thread):
                if thread is not None and thread.is_alive():
                    thread.join(timeout=1.0)
            self._watchdog_thread = None
            self._rx_thread = None
        self._emit_event("info", "link_down", "厂商 UDP 链路已关闭，已补发零速")

    def _close_sockets(self) -> None:
        for name in ("_rx_socket", "_tx_socket"):
            sock = getattr(self, name)
            setattr(self, name, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    # ------------------------------------------------------------------ 数据面
    def send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        """非阻塞下发速度（内部会记录时间戳，看门狗据此判断是否补零）。"""
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
        if self.posture_lock.locked():
            linear, angular = 0.0, 0.0
        with self._state_lock:
            self._last_cmd_time = time.monotonic()
        self._zero_active = False
        self._send_packets(velocity_packets(linear, angular))

    def _send_packets(self, packets: list[bytes]) -> bool:
        tx_socket = self._tx_socket
        if tx_socket is None:
            return False
        try:
            with self._tx_lock:
                for packet in packets:
                    tx_socket.sendto(packet, self._target)
        except OSError as error:
            with self._state_lock:
                self._counters["send_errors"] += 1
            self._emit_event(
                "warn", "send_failed",
                f"向 {self.robot_ip}:{self.robot_port} 发送速度包失败：{error}",
                self.WARN_INTERVAL_S)
            return False
        return True

    # ------------------------------------------------------------------ 接收线程
    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            rx_socket = self._rx_socket
            if rx_socket is None:
                break
            try:
                ready, _, _ = select.select([rx_socket], [], [], self.RX_POLL_S)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                packet, peer = rx_socket.recvfrom(2048)
            except (socket.timeout, BlockingIOError):
                continue
            except OSError:
                break
            received = time.monotonic()
            with self._state_lock:
                self._counters["rx_packets"] += 1
            self._handle_packet(packet, peer, received)

    def _handle_packet(self, packet: bytes, peer: tuple[str, int],
                       received: float) -> None:
        # 60 字节遥控反馈：只更新摇杆/目标速度，不单独产生状态帧
        if len(packet) == COMMAND_LENGTH:
            report = decode_command(packet)
            if report is None:
                self._count_ignored()
                return
            with self._state_lock:
                self._feedback = report
                self._feedback_time = received
                self._counters["command_reports"] += 1
            return
        if len(packet) != STATE_LENGTH:
            self._count_ignored()
            return
        report = decode_state(packet)
        if report is None:
            self._count_ignored()
            return
        with self._state_lock:
            self._last_state = report
            self._last_state_time = received
            self._counters["state_frames"] += 1
            feedback = dict(self._feedback) if self._feedback else None
            feedback_time = self._feedback_time
        if (feedback is not None and feedback_time > 0.0
                and received - feedback_time > self.feedback_timeout_s):
            # 遥控反馈超过 0.5 s 未更新：摇杆/目标速度清零，避免"幽灵接管"
            feedback = None
            self._emit_event(
                "warn", "joystick_stale",
                f"遥控反馈已超过 {self.feedback_timeout_s:.1f}s 未更新，"
                f"摇杆与目标速度已清零", self.WARN_INTERVAL_S)
        if feedback is not None:
            joystick = tuple(float(value) for value in feedback["joystick"])
            goal_velocity = tuple(float(value) for value in feedback["goal_velocity"])
        else:
            joystick = (0.0, 0.0, 0.0)
            goal_velocity = (0.0, 0.0, 0.0)
        frame = Frame(
            source="udp",
            stamp=received,
            position=tuple(float(value) for value in report["position"]),
            rpy_deg=tuple(float(value) for value in report["rpy_deg"]),
            velocity_body=tuple(float(value) for value in report["velocity_body"]),
            velocity_world=tuple(float(value) for value in report["velocity_world"]),
            basic=int(report["basic"]),
            gait=int(report["gait"]),
            motion=int(report["motion"]),
            joystick=joystick,
            goal_velocity=goal_velocity,
            raw={"angular": tuple(float(value) for value in report["angular"]),
                 "peer": peer[0], "received": received,
                 "battery_percent": report.get("battery_percent"),
                 "battery_level_raw": report.get("battery_level_raw"),
                 "battery_raw": report.get("battery_level_raw"),
                 "error_state": report.get("error_state"),
                 "is_charging": report.get("is_charging"),
                 "motion_state": report.get("motion"),
                 "touch": report.get("touch_down_and_stair_trot"),
                 "task_state": report.get("task_state"),
                 "is_robot_need_move": report.get("is_robot_need_move"),
                 "zero_position_flag": report.get("zero_position_flag")},
        )
        if not self._state_up_reported:
            self._state_up_reported = True
            self._emit_event(
                "info", "state_up",
                f"已收到厂商状态包（basic={frame.basic}, gait={frame.gait}）")
        self._safe_callback(self.callbacks.on_frame, frame)

    def _count_ignored(self) -> None:
        with self._state_lock:
            self._counters["ignored_packets"] += 1
        self._emit_event(
            "warn", "bad_packet",
            "收到无法解析的 UDP 数据包（长度/帧头/数值校验失败），已丢弃",
            self.WARN_INTERVAL_S)

    # ------------------------------------------------------------------ 看门狗
    def _watchdog_loop(self) -> None:
        while not self._stop_event.wait(0.05):
            now = time.monotonic()
            with self._state_lock:
                last_cmd = self._last_cmd_time
                last_state = self._last_state_time
                state_seen = self._last_state is not None
            # 状态过期降级告警（限频）
            state_age = now - last_state if state_seen else math.inf
            if state_age > self.state_timeout_s:
                if state_seen:
                    message = (f"状态包已 {state_age:.1f}s 未更新"
                               f"（阈值 {self.state_timeout_s:.1f}s），"
                               f"链路仍在但状态可能过期")
                else:
                    message = "尚未收到厂商状态包，请检查网络与转发端口"
                self._emit_event("warn", "state_timeout", message,
                                 self.WARN_INTERVAL_S)
            # 零速看门狗：超时后以约 5 Hz 补发零速
            if now - last_cmd > self.zero_timeout_s:
                if now - self._last_zero_send >= self.ZERO_RESEND_PERIOD_S:
                    self._last_zero_send = now
                    self._send_packets(safety_packets())
                    if not self._zero_active:
                        self._zero_active = True
                        self._emit_event(
                            "warn", "watchdog_zero",
                            f"{self.zero_timeout_s:.2f}s 没有新速度指令，"
                            f"看门狗已开始周期补发零速",
                            self.WARN_INTERVAL_S)
            else:
                self._zero_active = False
            # 遥控反馈过期：0.5 s 未更新即清零并限频告警
            with self._state_lock:
                feedback = self._feedback
                feedback_time = self._feedback_time
            if (feedback is not None and feedback_time > 0.0
                    and now - feedback_time > self.feedback_timeout_s):
                with self._state_lock:
                    self._feedback = None
                    self._feedback_time = 0.0
                self._emit_event(
                    "warn", "joystick_stale",
                    f"遥控反馈已超过 {self.feedback_timeout_s:.1f}s 未更新，"
                    f"摇杆与目标速度已清零", self.WARN_INTERVAL_S)

    # ------------------------------------------------------------------ 能力面
    def set_posture(self, action, cancel=None, timeout_s=10.0):
        """起立/趴下：**单次** 0x21010202 + 轮询 basic 确认。

        轮换指令发一次即切换一次，这里绝不在同一动作内重复发送；等待期间
        只补发心跳（>1 Hz）并在 ``cancel`` 置位时立即补零返回。Windows 直发
        只用于演示/中继，实机应走机载 ``ssh_ros`` 代发（终端白名单）。
        """
        target = posture_target(action)
        if target is None:
            return False, "姿态指令无效：%s（必须是 stand 或 lie）" % action
        if not self._running:
            return False, "厂商 UDP 链路未启动，无法切换姿态"
        if not self.posture_lock.acquire(blocking=False):
            return False, "已有姿态切换正在执行"
        timeout_s = max(1.0, _as_float(timeout_s, 10.0))
        label = "站立" if action == "stand" else "趴下"
        try:
            with self._state_lock:
                last = dict(self._last_state) if self._last_state else None
                last_time = self._last_state_time
            if last is None or time.monotonic() - last_time > self.state_timeout_s:
                return False, "没有新鲜状态反馈，无法切换姿态"
            basic = int(last["basic"])
            if basic == target:
                return True, "实机已处于%s状态（basic=%d）" % (label, target)
            if basic not in POSTURE_SOURCE_STATES:
                return False, ("实机正在切换姿态或处于保护状态（basic=%d），"
                               "请等待稳定后再试" % basic)
            self._posture_action = action
            self._last_error = ""
            # 心跳 + 单次轮换（绝不循环发 toggle）
            self._send_packets([simple_packet(HEARTBEAT)])
            self._send_packets([simple_packet(POSTURE_TOGGLE)])
            deadline = time.monotonic() + timeout_s
            last_heartbeat = time.monotonic()
            current = basic
            while time.monotonic() < deadline:
                if cancel is not None and cancel.is_set():
                    self._send_packets(safety_packets())
                    return False, "姿态切换已取消"
                now = time.monotonic()
                if now - last_heartbeat >= self.POSTURE_HEARTBEAT_PERIOD_S:
                    self._send_packets([simple_packet(HEARTBEAT)])
                    last_heartbeat = now
                with self._state_lock:
                    last = dict(self._last_state) if self._last_state else None
                    last_time = self._last_state_time
                if last is not None:
                    current = int(last["basic"])
                    if current == target:
                        self._send_packets(safety_packets())
                        return True, "实机已确认%s（basic=%d）" % (label, target)
                if now - last_time > self.state_timeout_s:
                    self._last_error = "姿态切换期间状态反馈中断"
                    return False, self._last_error
                time.sleep(0.05)
            self._last_error = "姿态确认超时（当前 basic=%s，目标 %d）" % (current, target)
            return False, self._last_error
        except Exception as error:
            self._last_error = str(error)
            return False, "姿态切换异常：%s" % error
        finally:
            self._posture_action = None
            self.posture_lock.release()

    def emergency_stop(self):
        """UDP 直发 3 次厂商急停 0x21010C0E（演示/中继用）。"""
        if self._tx_socket is None:
            return False, "厂商 UDP 发送套接字不可用，未能发送急停 0x21010C0E"
        sent = 0
        for _ in range(3):
            if self._send_packets([simple_packet(ESTOP)]):
                sent += 1
            time.sleep(0.02)
        if sent:
            return True, "已发送厂商急停 0x21010C0E（%d/3 次）" % sent
        self._last_error = "厂商急停发送失败"
        return False, "厂商急停发送失败：UDP 套接字不可用"

    # ------------------------------------------------------------------ 状态
    def status(self) -> dict:
        now = time.monotonic()
        with self._state_lock:
            counters = dict(self._counters)
            last_state = self._last_state
            last_state_time = self._last_state_time
            last_cmd_time = self._last_cmd_time
        state_seen = last_state is not None
        state_age = (now - last_state_time) if state_seen else None
        stale = (state_age is None) or (state_age > self.state_timeout_s)
        if not self._running:
            detail = "未启动" if state_age is None else "链路已停止"
        elif state_age is None:
            detail = (f"UDP 已监听 {self._bound[0]}:{self._bound[1]}，"
                      f"尚未收到状态包")
        elif stale:
            detail = (f"状态包已 {state_age:.2f}s 未更新"
                      f"（阈值 {self.state_timeout_s:.2f}s），请检查机器人")
        else:
            detail = (f"状态 {state_age * 1000:.0f} ms 前更新，"
                      f"basic={last_state['basic'] if last_state else '?'}")
        armed = bool(state_seen and not stale and last_state
                     and int(last_state["basic"]) == self.ready_basic)
        control = {
            # UDP 链路只能从 basic==6 推测已站立可运动，自动模式无法确认
            "armed": armed,
            "mode_confirmed": None,
            "pending": self._posture_action,
            "last_error": self._last_error,
            "depth_age_s": None,
            "automatic_ack_age_s": None,
        }
        return {
            "kind": self.kind,
            "label": self.label,
            "connected": bool(self._running),
            "detail": detail,
            "stale": bool(stale),
            "state_age_s": state_age,
            "basic": int(last_state["basic"]) if last_state else None,
            "listen": f"{self._bound[0]}:{self._bound[1]}",
            "bound_port": self._bound[1],
            "robot": f"{self.robot_ip}:{self.robot_port}",
            "zero_timeout_s": self.zero_timeout_s,
            "state_timeout_s": self.state_timeout_s,
            "last_command_age_s": (now - last_cmd_time) if self._running else None,
            "control": control,
            "reconnect": {"enabled": False, "attempts": 0,
                          "next_retry_in_s": None},
            "execution": {},
            **counters,
        }
