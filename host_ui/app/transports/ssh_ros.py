"""Access the onboard ROS1 bridge from a Windows host over SSH.

链路拓扑
--------
上位机（Windows）--SSH--> 机载感知主机 192.168.1.103 --UDP--> 运动主机
192.168.1.120:43893。厂商 ``0x2*******`` 指令只接受一个终端下发，运动主机
有终端 IP 白名单，所以姿态/急停/心跳**一律由机载 ``ssh_ros_stream.py`` 代发**，
本模块只通过 SSH stdin 发高层 JSON 命令并等待 ``control`` 回执（详见
``ssh_ros_stream.py`` 的模块 docstring）。

命令通道（stdin，一行一条 JSON）
-------------------------------
* ``{"linear": m/s, "angular": rad/s}``               速度（0.3 s 超时补零）
* ``{"action": "prepare"|"disarm"}``                  机载 Trigger 服务
* ``{"action": "posture", "target": "stand"|"lie"}``  机载发心跳 + 单次 toggle
* ``{"action": "estop"}``                             机载发 3 次 0x21010C0E

断线自动重连
------------
SSH/通道异常后按 ``options["reconnect"]``（也可用 ``reconnect_enabled`` 等
扁平键）做指数退避重连：``delay = min(max_delay, min_delay * 2**(n-1))``。
重连期间 ``status().connected=False``、``reconnect.attempts`` 递增；成功后发
``ssh_ros_reconnected`` 事件；持续失败按 5 s 限频告警。链路正常情况下
``send_velocity`` 不报错（写成命令并等 IO 线程发）。

健壮解析
--------
机载输出按行解析：坏 JSON 行只计数并限频告警，**不拆链路**；``pending``
缓冲上限 8 MB，超限丢弃旧数据；消息缺 ``type`` / ``data`` 直接忽略；
``recv_stderr`` 截断到 1500 字符并限频告警，避免 stderr 刷屏拖死 IO 线程。
"""
from __future__ import annotations

import base64
import json
import math
import os
import socket
import threading
import time
import uuid

from .base import Transport, TransportError
from .ros1 import frame_from_state_payload
from ..mapstore import map_from_occupancy_grid
from ..paths import resource_root

#: 机载 JSON 行缓冲上限（8 MB）
PENDING_LIMIT = 8 * 1024 * 1024
#: ``recv_stderr`` 单次事件保留的最大字符数
STDERR_KEEP = 1500
#: 断线重连默认参数（可被 options["reconnect"] 覆盖）
DEFAULT_RECONNECT = {"enabled": True, "min_delay_s": 1.0, "max_delay_s": 10.0}


def _as_float(value, fallback):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if result == result and result not in (float("inf"), float("-inf")) else fallback


def _as_bool(value, fallback):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    if value is None:
        return fallback
    return bool(value)


def host_key_mode(known_hosts: str) -> str:
    """``strict``：配置了 known_hosts 才做固定校验；否则 ``unverified``。"""
    return "strict" if str(known_hosts or "").strip() else "unverified"


def bind_source(sock, source_ip: str) -> bool:
    """尝试把 socket 绑定到指定源地址；无效/已失效的 IP 返回 False。

    现场 Windows 的 WiFi/有线地址会变化，配置里的 ``source_ip`` 可能过期；
    调用方应在返回 False 后改用系统自动选路，而不是直接失败。
    """
    if not str(source_ip or "").strip():
        return True
    try:
        sock.bind((str(source_ip), 0))
        return True
    except OSError:
        return False


class SshRosTransport(Transport):
    kind = "ssh_ros"
    label = "机载 ROS · SSH"
    default_options = {"host": "192.168.1.103", "username": "ysc",
                       "source_ip": "", "known_hosts": "", "ssh_port": 22,
                       "map_timeout_s": 8.0}
    capabilities = {"mapping": True, "arm": True, "posture": True, "vendor_estop": True,
                    "sensors": True, "reconnect": True}

    #: 状态帧超过该时长未更新视为不可用（与上位机 safety.state_timeout 对齐）
    STATE_TIMEOUT_S = 1.0
    #: 速度指令过期时间：超过后 IO 线程自动补零
    COMMAND_TIMEOUT_S = 0.3
    #: IO 循环轮询周期
    IO_POLL_S = 0.04
    #: 控制回执等待上限（姿态/急停额外限制）
    CONTROL_ACK_TIMEOUT_S = 8.0
    #: 急停回执等待上限（超时上层仍会零速+锁存急停）
    ESTOP_ACK_TIMEOUT_S = 2.0
    #: 服务发现（2 s）与状态确认（3 s）的额外回执预算。
    POSTURE_ACK_GRACE_S = 5.0
    STOP_JOIN_S = 2.0

    def __init__(self, options, callbacks):
        super().__init__(options, callbacks)
        opts = self.options
        self.host = str(opts.get("host", self.default_options["host"]))
        self.username = str(opts.get("username", self.default_options["username"]))
        self.source_ip = str(opts.get("source_ip", self.default_options["source_ip"]))
        self.known_hosts = str(opts.get("known_hosts",
                                       self.default_options["known_hosts"])).strip()
        self.ssh_port = int(_as_float(opts.get("ssh_port"), 22))
        self.read_only = _as_bool(opts.get("read_only"), False)
        self.capabilities = dict(type(self).capabilities)
        if self.read_only:
            self.capabilities.update(arm=False, posture=False, vendor_estop=False, mapping=False)
        self.state_timeout_s = max(0.2, _as_float(
            opts.get("state_timeout_s"), self.STATE_TIMEOUT_S))
        # SLAM 的 map_cloud 更新周期为 3 s；地图源过期时禁止自主运动。
        self.map_timeout_s = min(30.0, max(1.0, _as_float(
            opts.get("map_timeout_s"), self.default_options["map_timeout_s"])))

        reconnect = opts.get("reconnect")
        if not isinstance(reconnect, dict):
            reconnect = {}
        self._reconnect_enabled = _as_bool(
            reconnect.get("enabled", opts.get("reconnect_enabled")),
            DEFAULT_RECONNECT["enabled"])
        self._reconnect_min_s = max(0.01, _as_float(
            reconnect.get("min_delay_s", opts.get("reconnect_min_delay_s")),
            DEFAULT_RECONNECT["min_delay_s"]))
        self._reconnect_max_s = max(self._reconnect_min_s, _as_float(
            reconnect.get("max_delay_s", opts.get("reconnect_max_delay_s")),
            DEFAULT_RECONNECT["max_delay_s"]))

        # 数据面
        self.lock = threading.Lock()
        self.posture_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._event_lock = threading.Lock()
        self._lifecycle_lock = threading.RLock()
        self._connection_lock = threading.Lock()
        self._connecting_resources = []
        self._control_lock = threading.Condition()
        self._control_results = {}
        self._posture_request = None
        self._onboard_control = {}
        self._pending = None
        self._last_error = ""
        self._depth_age_s = None
        self._bridge_depth_age_s = None
        self._bridge_depth_fresh = None
        self._control_session = {}
        self._auto_ack_time = 0.0

        self.command = (0.0, 0.0, 0.0)
        self.last_frame = 0.0
        self.last_basic = None
        self.frames = 0
        self.execution = {}
        self.execution_received = 0.0
        self._control_state_received = 0.0
        self.mapping = {"state": "idle", "message": "等待机载建图状态"}
        self.mapping_received = 0.0
        self._map_received = 0.0
        self._map_source_stamp = None
        self._map_source_age = None
        self._map_time_error = "尚未收到实时地图"
        self.localization = {}
        self.sensors = {}
        self.health = {}
        self.system = {}
        self.error = ""
        self._bad_lines = 0
        self._pending_overflow = 0
        self._reconnect_attempts = 0
        self._next_retry_at = None
        self._event_times = {}

        self.client = None
        self.channel = None
        self._sock = None
        self.worker = None
        self.closed = threading.Event()
        self._wake = threading.Event()
        self._started = False
        self._link_ok = False

    @staticmethod
    def available():
        import importlib.util
        return importlib.util.find_spec("paramiko") is not None

    # ------------------------------------------------------------------ 事件
    def _emit_event(self, level, code, message, min_interval=0.0):
        if min_interval > 0:
            now = time.monotonic()
            with self._event_lock:
                last = self._event_times.get(code)
                if last is not None and now - last < min_interval:
                    return
                self._event_times[code] = now
        try:
            self.callbacks.on_event(level, code, message)
        except Exception:  # 界面回调异常不能杀死链路线程
            pass

    # ------------------------------------------------------------------ 生命周期
    def start(self):
        """建立连接；启用自动重连时首次失败也交由后台重试。"""
        with self._lifecycle_lock:
            return self._start()

    def _start(self):
        if self._started:
            return
        if self.worker is not None and self.worker.is_alive():
            raise TransportError("旧 SSH 工作线程尚未退出，请等待断开完成")
        self.closed.clear()
        self._wake.clear()
        try:
            self._connect()
        except Exception as error:
            self._close_connection()
            if not self._reconnect_enabled:
                raise TransportError(str(error)) from error
            self.error = str(error)
            self._emit_event("warn", "ssh_ros_initial_failure",
                             "首次连接失败，将自动重试：%s" % error)
        else:
            self.error = ""
        self._started = True
        self._link_ok = self.channel is not None
        self.worker = threading.Thread(target=self._supervise,
                                       name="ssh-ros-io", daemon=True)
        self.worker.start()

    def _track_connecting(self, resource):
        with self._connection_lock:
            if self.closed.is_set():
                resource.close()
                raise TransportError("SSH 连接已取消")
            self._connecting_resources.append(resource)
        return resource

    def _connect(self):
        """建立 SSH 会话并启动机载 stream；失败抛 TransportError。"""
        import paramiko
        sock = None
        client = None
        channel = None
        try:
            sock = self._track_connecting(socket.socket())
            sock.settimeout(5)
            if not bind_source(sock, self.source_ip):
                self._emit_event(
                    "warn", "ssh_source_ip_invalid",
                    "源地址 %s 在本机不可用（换网后会变），已改为系统自动选路"
                    % (self.source_ip or "-"), 60.0)
                try:
                    sock.close()
                except OSError:
                    pass
                sock = self._track_connecting(socket.socket())
                sock.settimeout(5)
            sock.connect((self.host, self.ssh_port))
            client = self._track_connecting(paramiko.SSHClient())
            client.load_system_host_keys()
            if self.known_hosts:
                known_path = os.path.expanduser(self.known_hosts)
                if not os.path.exists(known_path):
                    raise TransportError("known_hosts 文件不存在：%s" % known_path)
                client.load_host_keys(known_path)
                client.set_missing_host_key_policy(paramiko.RejectPolicy())
            else:
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                self._emit_event("warn", "ssh_host_key_unverified",
                                 "未配置 known_hosts，SSH 主机密钥未固定校验（存在中间人风险）",
                                 300.0)
            client.connect(self.host, port=self.ssh_port, username=self.username,
                           password=os.environ.get("HOST_UI_SSH_PASSWORD"),
                           sock=sock, timeout=6, auth_timeout=6,
                           look_for_keys=False, allow_agent=False)
            if self.closed.is_set():
                raise TransportError("SSH 连接已取消")
            script = (resource_root() / "host_ui/app/transports/ssh_ros_stream.py").read_bytes()
            if not self.read_only:
                sftp = client.open_sftp()
                try:
                    sftp.get_channel().settimeout(5.0)
                    launch = resource_root() / "lite2_sim/hardware/navigation_mapping.launch"
                    sftp.put(str(launch), "/home/ysc/airace_deployment_20261114/hardware/navigation_mapping.launch")
                finally:
                    sftp.close()
            encoded = base64.b64encode(script).decode("ascii")
            remote = ('source /opt/ros/melodic/setup.bash && '
                      'if [ -f /home/ysc/jueying_mapping_localization_ws/devel/setup.bash ]; then '
                      'source /home/ysc/jueying_mapping_localization_ws/devel/setup.bash; fi; python -u -c '
                      '"import base64; exec(compile(base64.b64decode(\'' + encoded +
                      '\'), \'host_ui_stream\', \'exec\'))"')
            if self.read_only:
                remote += " --read-only"
            channel = self._track_connecting(
                client.get_transport().open_session(timeout=6.0))
            channel.settimeout(1)
            if self.closed.is_set():
                raise TransportError("SSH 连接已取消")
            channel.exec_command(remote)
            with self._connection_lock:
                if self.closed.is_set():
                    raise TransportError("SSH 连接已取消")
                self._sock, self.client, self.channel = sock, client, channel
                self._connecting_resources = []
        except Exception as error:
            for item in (channel, client, sock):
                if item is not None:
                    try:
                        item.close()
                    except Exception:
                        pass
            with self._connection_lock:
                self._connecting_resources = []
            raise TransportError(
                "SSH 连接机载 ROS 失败（%s@%s）：%s" % (self.username, self.host, error)
            ) from error

    def _close_connection(self):
        with self._connection_lock:
            resources = [self.channel, self.client, self._sock]
            resources.extend(self._connecting_resources)
            self.channel = self.client = self._sock = None
            self._connecting_resources = []
        with self.lock:
            self.command = (0.0, 0.0, 0.0)
            self.last_frame = 0.0
            self.last_basic = None
            self._map_received = 0.0
            self._map_source_stamp = None
            self._map_source_age = None
            self._map_time_error = "链路已断开，等待新鲜地图"
        with self._control_lock:
            self._control_results.clear()
            self._onboard_control.clear()
            self._auto_ack_time = 0.0
            self._control_state_received = 0.0
            self._control_lock.notify_all()
        for item in resources:
            if item is not None:
                try:
                    item.close()
                except Exception:
                    pass

    def _supervise(self):
        """IO 线程：正常时跑 _io_loop；异常后指数退避重连。"""
        try:
            while not self.closed.is_set():
                if self.channel is None:
                    if not self._reconnect_enabled:
                        self._link_ok = False
                        self._emit_event("error", "ssh_ros_down",
                                         "机载 ROS 连接已断开（自动重连已关闭）：%s"
                                         % (self.error or "未知原因"))
                        return
                    self._reconnect_attempts += 1
                    delay = self._backoff_delay(self._reconnect_attempts)
                    self._next_retry_at = time.monotonic() + delay
                    self._emit_event(
                        "warn", "ssh_ros_reconnecting",
                        "机载 ROS 连接断开，%.1f s 后进行第 %d 次重连（%s）"
                        % (delay, self._reconnect_attempts, self.error or "通道中断"),
                        5.0)
                    self._wake.wait(delay)
                    self._wake.clear()
                    self._next_retry_at = None
                    if self.closed.is_set():
                        return
                    try:
                        self._connect()
                    except Exception as error:
                        self.error = str(error)
                        continue
                    if self.closed.is_set():
                        return
                    self.error = ""
                    self._link_ok = True
                    self._emit_event(
                        "info", "ssh_ros_reconnected",
                        "机载 ROS 连接已恢复（第 %d 次重连成功）"
                        % self._reconnect_attempts)
                    self._emit_event(
                        "info", "link_up",
                        "SSH 已连接 %s，机载 JSON 数据流已恢复" % self.host)
                    continue
                try:
                    self._link_ok = True
                    self._io_loop()
                except Exception as error:
                    self.error = str(error)
                    self._link_ok = False
                    self._close_connection()
                    if self.closed.is_set():
                        return
                    self._emit_event("warn", "ssh_ros_down",
                                     "机载 ROS 连接中断：%s" % error, 2.0)
        finally:
            self._link_ok = False
            self._close_connection()

    def _backoff_delay(self, attempt):
        exponent = max(0, int(attempt) - 1)
        delay = self._reconnect_min_s * (2 ** min(exponent, 12))
        return float(min(self._reconnect_max_s, delay))

    def _io_loop(self):
        """一个连接周期内的收发循环；任何异常都向上抛，由 _supervise 重连。"""
        channel = self.channel
        if channel is None:
            raise TransportError("SSH 通道未建立")
        pending = b""
        while not self.closed.is_set():
            if channel.exit_status_ready():
                detail = ""
                if channel.recv_stderr_ready():
                    detail = channel.recv_stderr(65536).decode("utf-8", "replace")[-STDERR_KEEP:].strip()
                raise TransportError("机载 ROS 数据流已退出" + ("：" + detail if detail else ""))
            with self.lock:
                linear, angular, stamp = self.command
            if time.monotonic() - stamp > self.COMMAND_TIMEOUT_S:
                linear, angular = 0.0, 0.0
            self._send_line({"linear": float(linear), "angular": float(angular)})
            if channel.recv_stderr_ready():
                try:
                    detail = channel.recv_stderr(65536).decode(
                        "utf-8", "replace")[-STDERR_KEEP:]
                except Exception:
                    detail = ""
                if detail.strip():
                    self._emit_event("warn", "ssh_ros_stderr", detail, 5.0)
            while channel.recv_ready():
                chunk = channel.recv(65536)
                if not chunk:
                    raise TransportError("SSH 数据流已关闭")
                pending += chunk
                if len(pending) > PENDING_LIMIT:
                    pending = b""
                    self._pending_overflow += 1
                    self._emit_event(
                        "warn", "ssh_ros_pending_overflow",
                        "机载 JSON 缓冲超过 %d MB，已丢弃旧数据"
                        % (PENDING_LIMIT // (1024 * 1024)), 5.0)
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    self._dispatch_line(line)
            self.closed.wait(self.IO_POLL_S)

    # ------------------------------------------------------------------ 发送
    def _send_line(self, payload):
        if self.read_only:
            if payload.get("action") not in (None, "ping"):
                raise TransportError("当前为只读接收模式")
            if payload.get("action") is None:
                return
        channel = self.channel
        if channel is None:
            raise TransportError("SSH 通道未建立")
        data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        with self._tx_lock:
            channel.sendall(data)

    def _send_command(self, payload):
        self._send_line(payload)

    def _send_zero_now(self):
        with self.lock:
            self.command = (0.0, 0.0, time.monotonic())
        try:
            self._send_line({"linear": 0.0, "angular": 0.0})
        except Exception:
            pass

    # ------------------------------------------------------------------ 接收
    def _dispatch_line(self, line):
        """解析一行机载 JSON；坏行只计数不拆链。"""
        text = line.strip()
        if not text:
            return
        try:
            message = json.loads(text.decode("utf-8", "replace"))
        except (ValueError, AttributeError):
            self._bad_lines += 1
            self._emit_event(
                "warn", "ssh_ros_bad_line",
                "机载 JSON 行解析失败（累计 %d 行），已忽略" % self._bad_lines, 2.0)
            return
        if not isinstance(message, dict):
            self._bad_lines += 1
            self._emit_event("warn", "ssh_ros_bad_line",
                             "机载消息不是 JSON 对象，已忽略", 2.0)
            return
        if "type" not in message or "data" not in message:
            self._bad_lines += 1
            self._emit_event("warn", "ssh_ros_bad_line",
                             "机载消息缺少 type/data，已忽略", 2.0)
            return
        try:
            self._receive(message)
        except Exception as error:  # 单条消息异常不能拖死 IO 线程
            self._bad_lines += 1
            self._emit_event("warn", "ssh_ros_bad_message",
                             "机载消息处理失败（已忽略）：%s" % error, 2.0)

    def _receive(self, message):
        kind = message.get("type")
        data = message.get("data")
        if kind == "state":
            now = time.monotonic()
            frame = frame_from_state_payload(data, now, source="ssh_ros")
            if frame:
                with self.lock:
                    self.last_frame = now
                    self.last_basic = frame.basic
                    self.frames += 1
                self.callbacks.on_frame(frame)
        elif kind == "execution":
            if isinstance(data, dict):
                with self.lock:
                    self.execution = data
                    self.execution_received = time.monotonic()
                    self._control_state_received = self.execution_received
                depth_age = data.get("host_ui_depth_age_s")
                bridge_depth_age = data.get("depth_age_s")
                bridge_fresh = data.get("depth_fresh")
                session = data.get("control_session")
                with self._control_lock:
                    if isinstance(depth_age, (int, float)):
                        self._depth_age_s = float(depth_age)
                    else:
                        self._depth_age_s = None
                    self._bridge_depth_age_s = (float(bridge_depth_age)
                                                if isinstance(bridge_depth_age,
                                                              (int, float))
                                                else None)
                    self._bridge_depth_fresh = (bool(bridge_fresh)
                                                if bridge_fresh is not None
                                                else None)
                    self._control_session = (dict(session)
                                             if isinstance(session, dict)
                                             else {})
                    self._note_execution_control(data)
                    self._control_lock.notify_all()
        elif kind == "sensor":
            if isinstance(data, dict) and data.get("name"):
                with self.lock:
                    self.sensors[data["name"]] = dict(
                        data, received=time.monotonic())
        elif kind == "health":
            if isinstance(data, dict):
                with self.lock:
                    merged = dict(self.health)
                    merged.update(data)
                    merged["received"] = time.monotonic()
                    self.health = merged
        elif kind == "mapping" and isinstance(data, dict):
            with self.lock:
                self.mapping = dict(data)
                self.mapping_received = time.monotonic()
        elif kind == "localization" and isinstance(data, dict):
            data = dict(data)
            try:
                for key in ("x", "y", "yaw_deg", "age_s"):
                    data[key] = float(data[key])
                    if not math.isfinite(data[key]):
                        raise ValueError("Nonfinite localization")
                data["valid"] = bool(data.get("valid")) and data["age_s"] >= 0.0
            except (KeyError, TypeError, ValueError):
                data = {"valid": False, "age_s": 0.0, "source": "invalid_localization"}
            with self.lock:
                self.localization = dict(data, received=time.monotonic())
        elif kind == "system":
            if isinstance(data, dict):
                with self.lock:
                    self.system = dict(data)
        elif kind == "runtime" and isinstance(data, dict):
            component = str(data.get("component") or "机载组件")
            detail = str(data.get("message") or data.get("state") or "状态更新")
            self._emit_event("warn" if data.get("state") == "failed" else "info",
                             "ssh_ros_runtime", "%s：%s" % (component, detail))
        elif kind == "map" and isinstance(data, dict):
            # 先验证地图内容，再推进健康状态；坏地图不能使旧图恢复可用。
            payload = dict(data)
            stamp = _as_float(payload.pop("stamp", None), None)
            source_age = _as_float(payload.pop("age_s", None), None)
            grid = map_from_occupancy_grid(**payload)
            now = time.monotonic()
            with self.lock:
                error = ""
                regressed = False
                previous_age = (self._map_source_age + max(0.0, now - self._map_received)
                                if self._map_source_age is not None and self._map_received
                                else None)
                if stamp is None or stamp <= 0 or source_age is None:
                    error = "地图缺少有效源时间戳"
                    source_age = previous_age
                elif source_age < 0:
                    error = "地图时间戳超前，请核对机载时钟"
                    source_age = previous_age
                elif self._map_source_stamp is not None and stamp <= self._map_source_stamp:
                    # 同一源帧重发不能刷新年龄；倒退的帧也不能覆盖新地图。
                    if previous_age is not None:
                        source_age = max(source_age, previous_age)
                    if stamp < self._map_source_stamp:
                        regressed = True
                        error = "地图源时间戳倒退，等待新鲜地图"
                self._map_received = now
                self._map_source_age = source_age
                self._map_time_error = error
                if not error and stamp is not None:
                    self._map_source_stamp = max(stamp, self._map_source_stamp or stamp)
            if not regressed and self.callbacks.on_map:
                # 无时间戳/时间异常的地图仍可显示，但健康门禁为不可用。
                self.callbacks.on_map(grid)
        elif kind == "control":
            self._on_control(data)
        elif kind == "ready":
            self._emit_event("info", "ssh_ros_ready",
                             "机载 ROS 数据流已就绪", 1.0)

    def _note_execution_control(self, data):
        """从 /airace/execution 的 control 字典同步 armed/mode_confirmed。

        调用方需持有 ``self._control_lock``。
        """
        inner = data.get("control")
        if not isinstance(inner, dict):
            return
        for key in ("armed", "mode_confirmed"):
            if key in inner:
                self._onboard_control[key] = bool(inner[key])
        if (self._onboard_control.get("armed")
                and self._onboard_control.get("mode_confirmed")):
            if not self._auto_ack_time:
                self._auto_ack_time = time.monotonic()

    def _on_control(self, data):
        if not isinstance(data, dict):
            return
        action = str(data.get("action") or "")
        if not action:
            return
        success = bool(data.get("success"))
        message = str(data.get("message") or "")
        if success and action in ("prepare", "disarm"):
            self._control_state_received = time.monotonic()
        with self._control_lock:
            if action == "posture":
                request = self._posture_request
                if (request is None or data.get("request_id") != request["request_id"]
                        or data.get("target") != request["target"]):
                    self._emit_event("warn", "ssh_ros_stale_posture_reply",
                                     "已忽略不属于当前请求的姿态回执", 1.0)
                    return
            self._control_results[action] = {"success": success,
                                             "message": message,
                                             "at": time.monotonic()}
            if action == "prepare" and success:
                self._onboard_control["armed"] = True
                self._onboard_control["mode_confirmed"] = True
                self._auto_ack_time = time.monotonic()
            elif action == "disarm" and success:
                self._onboard_control["armed"] = False
                self._onboard_control["mode_confirmed"] = False
            if not success:
                self._last_error = message
            elif action in ("posture", "prepare"):
                self._last_error = ""
            self._control_lock.notify_all()
        self._emit_event("info" if success else "warn", "ssh_ros_control",
                         "机载控制回执 %s：%s" % (action, message))

    # ------------------------------------------------------------------ 数据面
    def send_velocity(self, linear_m_s, angular_rad_s):
        """写命令给 IO 线程（非阻塞）；使能/姿态 pending 期间强制 0。"""
        try:
            linear = float(linear_m_s)
            angular = float(angular_rad_s)
        except (TypeError, ValueError):
            self._emit_event("warn", "invalid_command",
                             "速度指令不是数值，已忽略", 2.0)
            return
        if not (linear == linear and angular == angular
                and abs(linear) < float("inf") and abs(angular) < float("inf")):
            self._emit_event("warn", "invalid_command",
                             "速度指令含 NaN/Inf，已忽略", 2.0)
            return
        with self._control_lock:
            pending = self._pending is not None
        if pending or self.posture_lock.locked():
            linear, angular = 0.0, 0.0
        with self.lock:
            self.command = (linear, angular, time.monotonic())

    def sensor_snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {name: dict(data, age_s=round(now - data["received"], 3))
                    for name, data in self.sensors.items()}

    # ------------------------------------------------------------------ 使能 / 解除
    def arm(self, timeout_s=8.0):
        return self._run_control("prepare", timeout_s)

    def disarm(self, timeout_s=8.0):
        return self._run_control("disarm", timeout_s)

    def _run_control(self, action, timeout_s):
        label = {"prepare": "使能", "disarm": "解除",
                 "mapping_start": "开始建图", "mapping_stop": "停止建图"}.get(action, action)
        if not self._link_ok:
            return False, "机载 ROS 链路未连接，无法%s" % label
        timeout_s = max(0.5, _as_float(timeout_s, self.CONTROL_ACK_TIMEOUT_S))
        with self._control_lock:
            self._pending = action
            self._control_results.pop(action, None)
        try:
            self._send_zero_now()
            try:
                self._send_command({"action": action})
            except Exception as error:
                self._last_error = "发送%s命令失败：%s" % (label, error)
                return False, "%s命令下发失败：%s" % (label, error)
            deadline = time.monotonic() + timeout_s
            with self._control_lock:
                while True:
                    if self.closed.is_set() or not self._link_ok:
                        return False, "%s期间机载链路中断" % label
                    result = self._control_results.get(action)
                    if result is not None:
                        if result["success"]:
                            return True, result["message"] or ("%s成功" % label)
                        return False, result["message"] or ("%s失败" % label)
                    if action == "prepare":
                        if (self._onboard_control.get("armed")
                                and self._onboard_control.get("mode_confirmed")):
                            return True, "机载执行层已进入自动模式（armed + mode_confirmed）"
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._control_lock.wait(min(remaining, 0.1))
            message = ("%s超时：%.1f s 内未收到机载确认，请检查 /airace/%s 服务与自动模式"
                       % (label, timeout_s, action))
            self._last_error = message
            return False, message
        finally:
            with self._control_lock:
                self._pending = None
            self._send_zero_now()

    def set_mapping(self, running):
        if self.read_only:
            return False, "只读链路不能控制建图"
        return self._run_control("mapping_start" if running else "mapping_stop", 18.0)

    # ------------------------------------------------------------------ 姿态
    def set_posture(self, action, cancel=None, timeout_s=20.0):
        """经机载代发心跳 + 单次 0x21010202，轮询 basic 确认。返回 (ok, message)。"""
        if action not in ("stand", "lie"):
            return False, "姿态指令无效：%s（必须是 stand 或 lie）" % action
        if not self._link_ok:
            return False, "机载 ROS 链路未连接，无法切换姿态"
        if not self.posture_lock.acquire(blocking=False):
            return False, "已有姿态切换正在执行"
        timeout_s = max(1.0, _as_float(timeout_s, 20.0))
        request_id = uuid.uuid4().hex
        with self._control_lock:
            self._pending = "posture"
            self._posture_request = {"request_id": request_id, "target": action}
            self._control_results.pop("posture", None)
        cancelled = False
        try:
            self._send_zero_now()
            try:
                self._send_command({"action": "posture", "target": action,
                                    "request_id": request_id})
            except Exception as error:
                self._last_error = "发送姿态命令失败：%s" % error
                return False, "姿态命令下发失败：%s" % error
            # 留出机载服务发现及回执传输的时间，避免姿态服务在确认边界
            # 返回成功时，主机已经提前判定超时。
            deadline = time.monotonic() + timeout_s + self.POSTURE_ACK_GRACE_S
            with self._control_lock:
                while True:
                    if cancel is not None and cancel.is_set() and not cancelled:
                        cancelled = True
                        self._send_zero_now()
                        # 取消本地请求不等于中止 ROS 服务或物理动作。
                        # 保持 pending/姿态锁，直到机载对应回执或有界超时。
                    result = self._control_results.get("posture")
                    if result is not None:
                        if cancelled:
                            return False, "已取消姿态请求；机载已返回：%s" % result["message"]
                        if result["success"]:
                            return True, result["message"] or "姿态已确认"
                        return False, result["message"] or "姿态切换失败"
                    if not self._link_ok:
                        return False, "姿态切换期间机载链路中断"
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._control_lock.wait(min(remaining, 0.1))
            self._send_zero_now()
            message = ("姿态确认超时（%.1f s），机载动作状态未确认，请使用遥控器确认"
                       % (timeout_s + self.POSTURE_ACK_GRACE_S))
            self._last_error = message
            return False, message
        finally:
            with self._control_lock:
                self._pending = None
                self._posture_request = None
            self.posture_lock.release()

    # ------------------------------------------------------------------ 急停
    def emergency_stop(self):
        """请求机载进程代发 3 次 0x21010C0E 并等待回执。"""
        self._send_zero_now()
        if not self._link_ok:
            message = ("机载 ROS 链路断开，无法由机载端代发厂商急停；"
                       "请立即使用物理急停/遥控器")
            self._last_error = message
            return False, message
        with self._control_lock:
            self._control_results.pop("estop", None)
        try:
            self._send_command({"action": "estop"})
        except Exception as error:
            message = ("下发机载急停命令失败：%s；请立即使用物理急停/遥控器" % error)
            self._last_error = message
            return False, message
        deadline = time.monotonic() + self.ESTOP_ACK_TIMEOUT_S
        with self._control_lock:
            while True:
                if self.closed.is_set() or not self._link_ok:
                    return False, "等待急停确认期间机载链路中断，请使用物理急停/遥控器"
                result = self._control_results.get("estop")
                if result is not None:
                    if result["success"]:
                        return True, result["message"] or "机载端已发送厂商急停"
                    self._last_error = result["message"] or "机载端急停失败"
                    return False, self._last_error
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._control_lock.wait(min(remaining, 0.05))
        message = ("机载急停回执超时（%.1f s），请立即使用物理急停/遥控器"
                   % self.ESTOP_ACK_TIMEOUT_S)
        self._last_error = message
        return False, message

    # ------------------------------------------------------------------ 状态
    def status(self):
        now = time.monotonic()
        with self.lock:
            last_frame = self.last_frame
            frames = self.frames
            execution = dict(self.execution)
            execution_received = self.execution_received
            error = self.error
            health = dict(self.health)
            system = dict(self.system)
            mapping = dict(self.mapping)
            mapping_received = self.mapping_received
            localization = dict(self.localization)
            map_received = self._map_received
            map_source_stamp = self._map_source_stamp
            map_source_age = self._map_source_age
            map_time_error = self._map_time_error
        if localization:
            localization["age_s"] = max(0.0, now - localization.pop("received")) + float(localization.get("age_s", 0.0))
            localization["valid"] = (bool(localization.get("valid"))
                                     and 0.0 <= localization["age_s"] < 0.8)
        age = (now - last_frame) if last_frame else None
        with self._control_lock:
            onboard = dict(self._onboard_control)
            pending = self._pending
            last_error = self._last_error
            depth_age = self._depth_age_s
            bridge_depth_age = self._bridge_depth_age_s
            bridge_fresh = self._bridge_depth_fresh
            control_session = dict(self._control_session)
            auto_ack = self._auto_ack_time
            attempts = self._reconnect_attempts
            next_retry = self._next_retry_at
            bad_lines = self._bad_lines
            overflow = self._pending_overflow
        if not self._started:
            # 未启动（或单测直接注入消息）：沿用旧语义，仅看状态帧龄
            connected = age is not None and age < self.state_timeout_s
        else:
            connected = (self._link_ok and age is not None
                         and age < self.state_timeout_s)
        execution_age = now - execution_received if execution_received else None
        control_age = now - self._control_state_received if self._control_state_received else None
        if ((control_age is not None and control_age > self.state_timeout_s)
                or (self._started and not connected)):
            onboard = {"armed": None, "mode_confirmed": None}
        if ((execution_age is not None and execution_age > self.state_timeout_s)
                or (self._started and not connected)):
            bridge_fresh = False
        if execution_age is not None:
            depth_age = None if depth_age is None else depth_age + execution_age
            bridge_depth_age = None if bridge_depth_age is None else bridge_depth_age + execution_age
            bridge_fresh = bool(bridge_fresh) and bridge_depth_age is not None and bridge_depth_age <= 0.5
        if mapping_received and (now - mapping_received > 3.0 or not connected):
            mapping = {"state": "failed", "message": "建图状态过期，请检查机载链路"}
        if localization:
            localization["valid"] = bool(localization["valid"]) and connected
        map_received_age = max(0.0, now - map_received) if map_received else None
        map_age = (map_source_age + map_received_age
                   if map_source_age is not None and map_received_age is not None else None)
        map_valid = (connected and not map_time_error and map_age is not None
                     and 0.0 <= map_age <= self.map_timeout_s)
        map_reason = ("链路未连接" if not connected else map_time_error
                      or ("" if map_valid else "地图数据已过期，等待建图更新"))
        armed = onboard.get("armed")
        mode_confirmed = onboard.get("mode_confirmed")
        control = {
            "armed": None if armed is None else bool(armed),
            "mode_confirmed": (None if mode_confirmed is None
                               else bool(mode_confirmed)),
            "pending": pending,
            "last_error": last_error if last_error else "",
            # 深度门禁以机载执行桥接实际判定为准；相机帧龄单独保留
            "depth_age_s": (bridge_depth_age if bridge_depth_age is not None
                            else depth_age),
            "bridge_depth_age_s": bridge_depth_age,
            "camera_depth_age_s": depth_age,
            "depth_fresh": bridge_fresh,
            "control_session": control_session,
            "automatic_ack_age_s": (round(now - auto_ack, 2)
                                    if auto_ack > 0.0 else None),
        }
        return {
            "kind": self.kind,
            "label": self.label,
            "connected": bool(connected),
            "stale": not connected,
            "frames": frames,
            "last_frame_age_s": age,
            "basic": self.last_basic,
            "execution": execution,
            "execution_age_s": execution_age,
            "mapping": mapping,
            "map_health": {"valid": bool(map_valid), "age_s": map_age,
                           "received_age_s": map_received_age,
                           "source_stamp": map_source_stamp,
                           "timeout_s": self.map_timeout_s, "reason": map_reason},
            "localization": localization,
            "control": control,
            "reconnect": {
                "enabled": bool(self._reconnect_enabled),
                "attempts": int(attempts),
                "next_retry_in_s": (round(max(0.0, next_retry - now), 2)
                                    if next_retry is not None else None),
            },
            "bad_lines": bad_lines,
            "pending_overflow": overflow,
            "health": health,
            "system": system,
            "host_key": host_key_mode(self.known_hosts),
            "read_only": self.read_only,
            "ssh_connected": self._link_ok,
            "detail": (error if error else
                       (("机载 ROS 只读接收" if self.read_only else "机载 ROS 实时状态") if connected
                        else ("SSH 已连接，等待机载状态发布" if self._link_ok
                              else "等待机载连接"))),
        }

    # ------------------------------------------------------------------ 停止
    def stop(self):
        with self._lifecycle_lock:
            return self._stop()

    def _stop(self):
        if not self._started:
            self.closed.set()
            self._close_connection()
            return
        self.closed.set()
        self._send_zero_now()
        self._wake.set()
        with self._control_lock:
            self._control_lock.notify_all()
        worker = self.worker
        self._link_ok = False
        self._close_connection()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=self.STOP_JOIN_S)
        if worker is None or not worker.is_alive():
            self.worker = None
        self._started = False
        self._link_ok = False
        self.last_frame = 0.0
        self.last_basic = None
        self._emit_event("info", "link_down", "只读接收链路已关闭" if self.read_only
                         else "机载 ROS SSH 链路已关闭，已补发零速")
