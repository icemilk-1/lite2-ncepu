"""上位机节点：把传输层、安全闸门、导航、地图与录制串起来。

``HostNode`` 不依赖 FastAPI，可被单元测试直接驱动（手动调用 ``tick``）。
Web 服务在 asyncio 里以 25 Hz 调 :meth:`HostNode.tick`，以 20 Hz 调
:meth:`HostNode.snapshot` 广播；传输层回调从自己的线程进入，内部加锁。
"""
from __future__ import annotations

import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from .config import Config
from .explore import ExplorationPlanner
from .nav import PointNavigator
from .paths import data_root
from .recorder import Recorder
from .state import EventLog, RateMeter, SafetyCore, TrailTracker, transform_pose
from .transports import (KINDS, NullTransport, Transport, TransportCallbacks,
                         available_kinds, create_transport)
from .transports.base import TransportError
from .types import Frame, GridMap

REPO_ROOT = data_root()
HOST_UI_VERSION = "1.4.0"


def _resolve_path(value: str | Path | None) -> Path | None:
    """相对路径按仓库根目录解析，避免受启动目录影响。"""
    if not value:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


class HostNode:
    """一个上位机实例（单机单机器人）。"""

    def __init__(self, config: Config) -> None:
        self.config = config
        self._lock = threading.RLock()
        self._connection_lock = threading.RLock()
        self.started_at = time.monotonic()
        self.safety = SafetyCore(config.get("limits"), config.get("safety"))
        self.nav = PointNavigator(config.get("nav"), config.get("limits"))
        self.events = EventLog()
        self.recorder = Recorder(_resolve_path(config.get("record.dir")))
        self.control_rate = RateMeter()
        self.broadcast_rate = RateMeter()
        self.frame_rate = RateMeter()

        self.frame: Frame | None = None
        self.frame_count = 0
        self.last_frame_wall = 0.0
        self.map: GridMap | None = None
        self.map_revision = 0
        #: 地图 payload 与 WS 消息在加载时预编码，避免广播环里对大图做 base64/JSON
        self._map_payload_cache: dict | None = None
        self._map_message_json: str | None = None
        self._map_cache_revision = -1
        self.scan: list[tuple[float, float]] = []
        self.scan_stamp = 0.0
        self.trail = TrailTracker()
        self.link_kind = str(config.get("link.kind", "mock"))
        self.link_options: dict = dict(config.get("link.options") or {})
        self.link_started_at = 0.0
        self.link_error = ""
        self.clients = 0
        self._last_nav_state = "idle"
        self._nav_message_time = 0.0
        self._last_mode: int | None = None
        self._takeover_reported = False
        self._state_missing_reported = False
        self._last_row_time = 0.0
        self._closed = False
        self._boot_message = ""
        # 操作权 / 操作者心跳（导航与使能都依赖它）
        self.operator_id: int | None = None
        self.operator_since: float | None = None
        self.last_operator_activity = 0.0
        # 姿态切换状态机
        self.posture_state: dict[str, Any] = {"state": "idle", "action": None,
                                              "message": "", "since": 0.0}
        self._posture_cancel = threading.Event()
        self._posture_thread: threading.Thread | None = None
        self._posture_lock = threading.Lock()
        self.version = HOST_UI_VERSION
        self._mapping_thread: threading.Thread | None = None
        self._mapping_request: dict = {}
        self._saved_map = None
        self.robot_version = ""
        # 自主探索（前沿探索建图）：只负责选目标，速度仍由 nav + SafetyCore 下发
        explore_cfg = dict(config.get("explore") or {})
        # 膨胀半径与导航保持一致（配置只维护 nav.inflate_m 一份）
        explore_cfg.setdefault("inflate_m", (config.get("nav") or {}).get("inflate_m", 0.30))
        self.explorer = ExplorationPlanner(explore_cfg)
        self.exploration_state: dict[str, Any] = {
            "state": "idle", "message": "", "since": time.monotonic(),
            "frontiers": 0, "goals_visited": 0, "blacklisted": 0,
            "consecutive_failures": 0, "last_goal": None,
        }
        self._exploration_preview: list[list[float]] = []
        self._exploration_preview_time = 0.0
        self._exploration_goal: tuple[float, float] | None = None
        self._exploration_failures = 0
        self._exploration_retry_at = 0.0
        self._exploration_observe_until = 0.0
        self._exploration_wait_started: float | None = None
        self._exploration_wait_signature = None
        self._exploration_last_arrival_pose = None
        self._exploration_no_progress_arrivals = 0

        callbacks = TransportCallbacks(
            on_frame=self._on_frame,
            on_event=self._on_event,
            on_map=self._on_map,
            on_scan=self._on_scan,
        )
        self._transport_token = None
        self._callbacks = callbacks
        self.transport: Transport = NullTransport(None, callbacks)

        self.add_event("info", "host_start",
                       f"上位机启动，链路={self.link_kind}，配置={config.path or '默认'}")
        if config.warnings:
            for warning in config.warnings:
                self.add_event("warn", "config_normalized", warning)
        self._load_initial_map()
        if config.get("record.enabled"):
            self.start_recording(prefix="auto")

    # ------------------------------------------------------------------ 事件与回调
    def add_event(self, level: str, code: str, message: str, **detail: Any) -> None:
        event = self.events.add(level, code, message, **detail)
        if self.recorder.active:
            self.recorder.write_event(event)

    def _on_event(self, level: str, code: str, message: str) -> None:
        self.add_event(level, code, message)

    def _on_frame(self, frame: Frame) -> None:
        now = time.monotonic()
        with self._lock:
            self.frame = frame
            self.frame_count += 1
            self.last_frame_wall = time.time()
            self._state_missing_reported = False
            localized = self.map_pose()
            if localized is not None:
                self.trail.update(localized[0], localized[1])
            if self._last_mode is not None and frame.basic != self._last_mode:
                self.add_event("info", "mode_change",
                               f"机器人模式 basic: {self._last_mode} -> {frame.basic}",
                               basic=frame.basic)
            self._last_mode = frame.basic
            # 厂商遥控接管只报一次
            threshold = float(self.config.get("safety.joystick_threshold", 0.03))
            if frame.joystick_active(threshold):
                if not self._takeover_reported:
                    self._takeover_reported = True
                    self.add_event("warn", "joystick_takeover",
                                   "检测到厂商遥控器接管，运动指令已被拦截",
                                   joystick=list(frame.joystick))
            else:
                self._takeover_reported = False
        self.frame_rate.tick(now)

    def _on_map(self, grid: GridMap) -> None:
        with self._lock:
            self.map = grid
            self.map_revision += 1
        self._refresh_map_cache()
        self.add_event("info", "map_loaded",
                       f"地图已加载：{grid.name} {grid.width}x{grid.height} "
                       f"@{grid.resolution}m/格")

    def _on_scan(self, points: list[tuple[float, float]]) -> None:
        with self._lock:
            self.scan = points[:2000]
            self.scan_stamp = time.monotonic()

    def _connection_callbacks(self, token):
        def guarded(callback):
            def deliver(*args):
                with self._lock:
                    if self._transport_token is token:
                        callback(*args)
            return deliver

        return TransportCallbacks(
            on_frame=guarded(self._on_frame),
            on_event=guarded(self._on_event),
            on_map=guarded(self._on_map),
            on_scan=guarded(self._on_scan),
        )

    # ------------------------------------------------------------------ 地图
    def _load_initial_map(self) -> None:
        path = self.config.get("map.file")
        if path:
            self.load_map_file(str(path))
            return
        if self.config.get("map.demo", True):
            try:
                from .mapstore import demo_map

                grid = demo_map()
                with self._lock:
                    self.map = grid
                    self.map_revision += 1
                self._refresh_map_cache()
                self.add_event("info", "map_demo",
                               f"已加载示例地图 {grid.width}x{grid.height}", )
            except Exception as error:  # 地图模块缺失时不影响启动
                self.add_event("warn", "map_unavailable", f"示例地图加载失败：{error}")

    def load_map(self, grid: GridMap, source: str = "手动导入") -> None:
        with self._lock:
            self.map = grid
            self.map_revision += 1
            self.trail.reset()
        self._refresh_map_cache()
        self.add_event("info", "map_loaded",
                       f"{source}：{grid.name} {grid.width}x{grid.height} "
                       f"@{grid.resolution}m/格")

    def load_map_file(self, path: str) -> tuple[bool, str]:
        target = _resolve_path(path)
        try:
            from .mapstore import load_map_file

            grid = load_map_file(str(target))
        except Exception as error:
            self.add_event("error", "map_error", f"地图加载失败：{error}")
            return False, str(error)
        self.load_map(grid, source=f"地图文件 {Path(target).name}")
        return True, f"已加载地图 {grid.name}"

    def set_map_align(self, align: dict) -> None:
        self.config.update({"map": {"align": align}})
        with self._lock:
            self.trail.reset()
        self.add_event("info", "map_align",
                       f"地图对齐已更新：x={align.get('x', 0):.2f} y={align.get('y', 0):.2f} "
                       f"yaw={align.get('yaw_deg', 0):.1f}°")

    def _refresh_map_cache(self) -> None:
        """在加载线程里预编码地图（base64 + JSON），广播环只做发送。"""
        with self._lock:
            grid = self.map
            revision = self.map_revision
        payload = None
        message = None
        if grid is not None:
            payload = grid.to_payload()
            message = ('{"type":"map","data":'
                       + json.dumps(payload, ensure_ascii=False,
                                    separators=(",", ":")) + "}")
        with self._lock:
            if revision == self.map_revision:
                self._map_payload_cache = payload
                self._map_message_json = message
                self._map_cache_revision = revision

    def map_payload(self) -> dict | None:
        with self._lock:
            if (self._map_cache_revision == self.map_revision
                    and self._map_payload_cache is not None):
                return self._map_payload_cache
            return self.map.to_payload() if self.map is not None else None

    def map_message_json(self) -> str | None:
        """返回预编码的 ``{"type":"map","data":...}``，无地图时返回 None。"""
        with self._lock:
            if self._map_cache_revision == self.map_revision:
                return self._map_message_json
        self._refresh_map_cache()
        with self._lock:
            return self._map_message_json

    def map_describe(self) -> dict | None:
        with self._lock:
            return self.map.describe() if self.map is not None else None

    # ------------------------------------------------------------------ 链路管理
    def connect(self, kind: str | None = None, options: dict | None = None) -> tuple[bool, str]:
        with self._connection_lock:
            return self._connect(kind, options)

    def _connect(self, kind: str | None, options: dict | None) -> tuple[bool, str]:
        kind = str(kind or self.config.get("link.kind", "mock")).strip().lower()
        if kind == "vendor":
            kind = "udp"
        elif kind == "ros1":
            kind = "ros"
        merged_options = dict(self.config.get(f"link.options") or {})
        merged_options.update(options or {})
        # 重连参数由配置统一注入（传输层不认识 config 对象）
        reconnect = self.config.get("link.reconnect")
        if isinstance(reconnect, dict):
            merged_options.setdefault("reconnect", dict(reconnect))
        if self._closed:
            return False, "上位机已关闭"
        token = object()
        try:
            transport = create_transport(kind, merged_options, self._connection_callbacks(token))
        except Exception as error:
            self.link_error = str(error)
            self.add_event("error", "link_error", f"创建链路失败：{error}")
            return False, f"创建链路失败：{error}"

        self.disconnect(reason="切换链路", silent=True)
        with self._lock:
            self._transport_token = token
        try:
            transport.start()
        except TransportError as error:
            with self._lock:
                self._transport_token = None
                self.frame = None
                self.scan = []
                self.scan_stamp = 0.0
            self.link_error = str(error)
            self.add_event("error", "link_error", f"{kind} 连接失败：{error}")
            try:
                transport.stop()
            except Exception:
                pass
            return False, f"连接失败：{error}"
        except Exception as error:
            with self._lock:
                self._transport_token = None
                self.frame = None
                self.scan = []
                self.scan_stamp = 0.0
            self.link_error = str(error)
            self.add_event("error", "link_error", f"{kind} 连接异常：{error}")
            try:
                transport.stop()
            except Exception:
                pass
            return False, f"连接异常：{error}"

        with self._lock:
            self.transport = transport
            self.link_kind = kind
            self.link_options = merged_options
            self.link_started_at = time.monotonic()
            self.link_error = ""
            self.trail.reset()
            if kind != "mock" and self.map is not None and self.map.name == "demo_lab":
                self.map = None
                self.map_revision += 1
        self.config.update({"link": {"kind": kind, "options": merged_options}})
        self.add_event("info", "link_up", f"已连接链路：{transport.label}")
        return True, f"已连接 {transport.label}"

    def disconnect(self, reason: str = "手动断开", silent: bool = False) -> None:
        with self._connection_lock:
            self._disconnect(reason, silent)

    def _disconnect(self, reason: str, silent: bool) -> None:
        with self._lock:
            self._transport_token = None
            transport = self.transport
            self.transport = NullTransport(None, self._callbacks)
            self.frame = None
            self.scan = []
            self.scan_stamp = 0.0
        self._posture_cancel.set()
        self.safety.end_posture()
        self.release_operator()
        self.stop_all(f"链路断开：{reason}")
        try:
            transport.stop()
        except Exception as error:
            self.add_event("error", "link_stop_failed", f"停止旧链路失败：{error}")
        if transport.kind != "offline":
            if not silent:
                self.add_event("info", "link_down", f"链路已断开：{reason}")

    def link_up(self) -> bool:
        try:
            status = self.transport.status()
        except Exception:
            return False
        return bool(status.get("connected"))

    # ------------------------------------------------------------------ 操作权 / 操作者心跳
    def set_operator(self, client_id: int | None) -> None:
        with self._lock:
            self.operator_id = client_id
            self.operator_since = time.monotonic() if client_id is not None else None
        if client_id is not None:
            self.last_operator_activity = time.monotonic()
            self.add_event("info", "operator_claim",
                           f"客户端 {client_id} 取得操控权（接管需在界面确认）")

    def release_operator(self, client_id: int | None = None) -> tuple[bool, str]:
        with self._lock:
            if self.operator_id is None:
                return True, "当前没有操控者"
            if client_id is not None and self.operator_id != client_id:
                return False, "不是当前操控者"
            previous = self.operator_id
            self.operator_id = None
            self.operator_since = None
        self.stop_all("操控者已离开，自动任务已停止")
        self.add_event("info", "operator_release", f"客户端 {previous} 释放操控权")
        return True, "操控权已释放"

    def operator_snapshot(self) -> dict:
        with self._lock:
            lease_age = (time.monotonic() - self.last_operator_activity
                         if self.last_operator_activity else None)
            return {"id": self.operator_id,
                    "since": self.operator_since,
                    "clients": self.clients,
                    "lease_age_s": round(lease_age, 2) if lease_age is not None else None}

    def note_operator_activity(self) -> None:
        self.last_operator_activity = time.monotonic()

    def _capability(self, name: str) -> bool:
        try:
            return bool((getattr(self.transport, "capabilities", {}) or {}).get(name))
        except Exception:
            return False

    # ------------------------------------------------------------------ 使能 / 解除
    def _link_status(self) -> dict:
        try:
            return self.transport.status()
        except Exception:
            return {"connected": False}

    def arm(self) -> tuple[bool, str]:
        """请求机载执行层进入自动模式（armed + mode_confirmed）。"""
        if self.safety.estop:
            return False, "急停锁存中，先复位急停"
        status = self._link_status()
        if not status.get("connected"):
            return False, "链路未连接，无法使能"
        if not self._capability("arm"):
            return False, "当前链路不支持使能；请使用「机载 ROS · SSH」或机载 ROS1"
        if self.safety.posture_active:
            return False, "姿态切换进行中，等待完成后再使能"
        with self._lock:
            frame = self.frame
        ready_basic = int(self.config.get("safety.ready_basic", 6))
        state_timeout = float(self.config.get("safety.state_timeout_s", 0.8))
        if frame is None or (time.monotonic() - frame.stamp) > state_timeout:
            return False, "没有新鲜状态反馈，无法使能"
        if int(frame.basic) != ready_basic:
            return False, f"机器人当前 basic={frame.basic}，请先站立到力控站立({ready_basic})"
        self.stop_all("使能前停车")
        self.note_operator_activity()
        self.add_event("info", "arm_request", "请求使能：切换到自动模式")
        timeout = float(self.config.get("posture.timeout_s", 20.0))
        try:
            ok, message = self.transport.arm(timeout_s=timeout)
        except Exception as error:
            ok, message = False, f"使能异常：{error}"
        self.add_event("info" if ok else "error", "arm", message)
        return ok, message

    def disarm(self) -> tuple[bool, str]:
        """退出自动模式并请求零速（任何时候都允许）。"""
        if not self._capability("arm"):
            return True, "当前链路无自动模式，无需解除"
        self.stop_all("解除前停车")
        try:
            ok, message = self.transport.disarm()
        except Exception as error:
            ok, message = False, f"解除异常：{error}"
        self.add_event("info" if ok else "warn", "disarm", message)
        return ok, message

    # ------------------------------------------------------------------ 姿态切换（纳入安全核心）
    def posture(self, action: str) -> tuple[bool, str]:
        if action not in ("stand", "lie"):
            return False, "姿态必须是 stand 或 lie"
        if self.safety.estop:
            return False, "急停锁存中，禁止姿态切换"
        if not self._capability("posture"):
            return False, "当前链路不支持姿态切换"
        if not self._link_status().get("connected"):
            return False, "链路未连接，无法切换姿态"
        with self._lock:
            frame = self.frame
        if frame is None or (time.monotonic() - frame.stamp) > \
                float(self.config.get("safety.state_timeout_s", 0.8)):
            return False, "没有新鲜状态反馈，无法切换姿态"
        if action == "stand" and int(frame.basic) != 1:
            return False, f"只有趴下状态(basic=1)才能起立，当前 basic={frame.basic}"
        if action == "lie" and int(frame.basic) != 6:
            return False, f"只有站立状态(basic=6)才能趴下，当前 basic={frame.basic}"
        with self._posture_lock:
            if self.posture_state.get("state") == "pending":
                return False, "已有姿态切换正在进行"
            self.stop_all("姿态切换前停车")
            self.nav.cancel("姿态切换，导航取消")
            self.safety.begin_posture(action)
            self._posture_cancel.clear()
            self.posture_state = {"state": "pending", "action": action,
                                  "message": "正在切换，请勿操作", "since": time.monotonic()}
            self.note_operator_activity()
            self._posture_thread = threading.Thread(
                target=self._run_posture, args=(action,), daemon=True)
            self._posture_thread.start()
        self.add_event("info", "posture_request",
                       f"请求{'起立' if action == 'stand' else '趴下'}，运动已锁定")
        return True, f"正在{'起立' if action == 'stand' else '趴下'}…"

    def _run_posture(self, action: str) -> None:
        timeout = float(self.config.get("posture.timeout_s", 20.0))
        try:
            ok, message = self.transport.set_posture(
                action, cancel=self._posture_cancel, timeout_s=timeout)
        except Exception as error:
            ok, message = False, f"姿态切换异常：{error}"
        cancelled = self._posture_cancel.is_set()
        self.safety.end_posture()
        with self._posture_lock:
            if cancelled:
                self.posture_state = {"state": "cancelled", "action": action,
                                      "message": "姿态切换已取消", "since": time.monotonic()}
            elif ok:
                self.posture_state = {"state": "ok", "action": action,
                                      "message": message, "since": time.monotonic()}
            else:
                self.posture_state = {"state": "failed", "action": action,
                                      "message": message, "since": time.monotonic()}
        level = "info" if ok and not cancelled else ("warn" if cancelled else "error")
        self.add_event(level, "posture", message)

    def cancel_posture(self, reason: str = "操作者取消姿态切换") -> tuple[bool, str]:
        with self._posture_lock:
            if self.posture_state.get("state") != "pending":
                return True, "没有进行中的姿态切换"
            self._posture_cancel.set()
            self.posture_state["message"] = "已请求取消，等待机载姿态动作结束；运动保持锁定"
        self.add_event("warn", "posture_cancel", reason)
        return True, "已请求取消，等待机载动作结束"

    # ------------------------------------------------------------------ 急停
    def engage_estop(self, reason: str = "手动急停") -> tuple[bool, str]:
        self._posture_cancel.set()
        self.safety.engage_estop(reason)
        self.nav.cancel("急停触发，导航取消")
        self.stop_all("急停：零速")
        detail = ""
        if (self._capability("vendor_estop")
                and bool(self.config.get("safety.vendor_estop_on_estop", True))):
            try:
                ok, detail = self.transport.emergency_stop()
                if not ok:
                    detail = f"厂商急停未确认：{detail}"
            except Exception as error:
                detail = f"厂商急停发送异常：{error}"
        message = "急停已锁存，所有运动指令被拦截"
        if detail:
            message += f"；{detail}"
        self.add_event("error", "estop", f"急停已触发：{reason}｜{detail or '仅软件零速'}")
        return True, message

    def reset_estop(self, hold_s: float | None = None) -> tuple[bool, str]:
        required = float(self.config.get("safety.estop_reset_hold_s", 1.2))
        if hold_s is None or float(hold_s) < required:
            return False, f"急停复位需长按至少 {required:.1f} 秒"
        if not self.safety.estop:
            return True, "急停未锁存"
        self.safety.reset_estop()
        # 部分链路（如 mock / 厂商 UDP）有自己的急停标记，需要同步清除
        reset_transport = getattr(self.transport, "reset_estop", None)
        if callable(reset_transport):
            try:
                reset_transport()
            except Exception:
                pass
        self.add_event("info", "estop_reset", "急停已复位，需重新按住方向键才可运动")
        return True, "急停已复位，需重新按住方向键才可运动"

    def set_teleop(self, linear: float, angular: float, deadman: bool) -> tuple[bool, str]:
        if self.safety.estop:
            return False, "急停锁存中，先复位急停"
        if self.safety.posture_active:
            return False, "姿态切换进行中，禁止手动操控"
        self.note_operator_activity()
        self.safety.set_teleop(linear, angular, deadman)
        if deadman and (abs(linear) > 1e-6 or abs(angular) > 1e-6):
            if self.nav.state in ("following", "aligning"):
                self.nav.cancel("手动接管，导航取消")
                self.add_event("info", "nav_cancel", "手动接管，导航已取消")
            if self.exploration_state.get("state") == "active":
                self._exploration_goal = None
                self._set_exploration("paused", "手动接管，探索暂停", time.monotonic())
                self.add_event("warn", "exploration_paused",
                               "手动接管，自主探索已暂停（可点「继续探索」恢复）")
        return True, ""

    def stop_all(self, reason: str = "操作者立即停车") -> tuple[bool, str]:
        self.safety.release_teleop()
        self.safety.clear_nav()
        if self.nav.state in ("following", "aligning"):
            self.nav.cancel("操作者立即停车")
        if self.exploration_state.get("state") in ("active", "paused"):
            self._set_exploration("cancelled", "操作者立即停车，探索已取消", time.monotonic())
            self._exploration_preview = []
            self.add_event("warn", "exploration_cancel", "操作者立即停车，探索已取消")
        self.add_event("warn", "stop", reason)
        return True, "已发送零速指令"

    def clear_trail(self) -> None:
        with self._lock:
            self.trail.reset()
        self.add_event("info", "trail_clear", "轨迹已清除")

    def set_limits(self, patch: dict) -> tuple[bool, str]:
        if not isinstance(patch, dict):
            return False, "参数格式错误"
        allowed = {"max_forward_m_s", "max_reverse_m_s", "max_yaw_rad_s",
                   "teleop_linear_m_s", "teleop_angular_rad_s"}
        limits_patch = {key: value for key, value in patch.items() if key in allowed}
        if not limits_patch:
            return False, "没有可更新的限幅字段"
        warnings = self.config.update({"limits": limits_patch})
        self.safety.update_config(self.config.get("limits"), self.config.get("safety"))
        self.nav.update_config(self.config.get("nav"), self.config.get("limits"))
        message = "限幅已更新"
        if warnings:
            message += "（" + "；".join(warnings) + "）"
        self.add_event("info", "limits", message + f"：{limits_patch}")
        return True, message

    # ------------------------------------------------------------------ 导航
    def _apply_execution_limits(self, status: dict) -> None:
        reported = (status.get("execution") or {}).get("limits") or {}
        current = self.config.get("limits")
        patch = {}
        for key in ("max_forward_m_s", "max_reverse_m_s", "max_yaw_rad_s"):
            value = reported.get(key)
            if isinstance(value, (int, float)) and math.isfinite(value) and value > 0:
                if value < current[key]:
                    patch[key] = value
        if patch:
            self.config.update({"limits": patch})
            self.safety.update_config(self.config.get("limits"), self.config.get("safety"))
            self.nav.update_config(self.config.get("nav"), self.config.get("limits"))
            self.add_event("info", "execution_limits", "速度已按机载执行限幅同步", limits=patch)

    def set_mapping(self, running: bool) -> tuple[bool, str]:
        if not self._capability("mapping"):
            return False, "当前链路不支持机载建图控制"
        with self._lock:
            if self._mapping_thread is not None and self._mapping_thread.is_alive():
                return False, "建图操作正在执行"
            self.stop_all()
            self._mapping_request = {"state": "starting" if running else "stopping",
                                     "message": "正在启动建图" if running else "正在停止建图"}
            transport = self.transport

            def run():
                try:
                    ok, message = transport.set_mapping(running)
                except Exception as error:
                    ok, message = False, str(error)
                with self._lock:
                    self._mapping_request = {} if ok else {"state": "failed", "message": message}
                self.add_event("info" if ok else "error", "mapping", message)

            self._mapping_thread = threading.Thread(target=run, name="mapping-control", daemon=True)
            self._mapping_thread.start()
        return True, "建图请求已提交"

    def save_map(self) -> tuple[bool, str]:
        with self._lock:
            if self.map is None:
                return False, "没有地图可以保存"
            payload = self.map.to_payload()
        directory = REPO_ROOT / "host_ui/maps" / (time.strftime("mapped_%Y%m%d_%H%M%S")
                                                 + "_" + uuid.uuid4().hex[:6])
        directory.mkdir(parents=True, exist_ok=True)
        target = directory / "map.json"
        temporary = directory / "map.json.tmp"
        temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        temporary.replace(target)
        self._saved_map = str(target)
        self.add_event("info", "map_saved", f"地图已保存：{target}")
        return True, str(target)

    def map_pose(self) -> tuple[float, float, float] | None:
        if self.link_kind == "ssh_ros":
            status = self.transport.status()
            localization = status.get("localization") or {}
            if not status.get("connected") or not localization.get("valid"):
                return None
            with self._lock:
                if self.map is None or str(localization.get("frame", "")).lstrip("/") != self.map.name.lstrip("/"):
                    return None
            try:
                pose = tuple(float(localization[key]) for key in ("x", "y", "yaw_deg"))
                return pose if all(math.isfinite(value) for value in pose) else None
            except (KeyError, TypeError, ValueError):
                return None
        with self._lock:
            if self.frame is None:
                return None
            align = self.config.get("map.align")
            return transform_pose(self.frame.position[0], self.frame.position[1],
                                  self.frame.yaw_deg, align)

    def set_goal(self, x: float, y: float, yaw_deg: float | None = None) -> tuple[bool, str]:
        if self.safety.estop:
            return False, "急停锁存中，无法启动导航"
        if not self._navigation_map_ready():
            return False, "实时地图缺失或过期，请等待建图更新"
        pose = self.map_pose()
        if pose is None:
            return False, "地图定位不可用或过期，请先启动建图并等待定位"
        # 手动点选目标：探索转为暂停，避免和操作者抢目标
        if self.exploration_state.get("state") == "active":
            self._exploration_goal = None
            self._set_exploration("paused", "切换手动目标，探索暂停", time.monotonic())
            self.add_event("warn", "exploration_paused", "操作者点选新目标，自主探索已暂停")
        ok, message = self._submit_goal(float(x), float(y), yaw_deg, pose)
        if ok:
            self.note_operator_activity()
            self.add_event("info", "nav_start",
                           f"导航目标 ({x:.2f}, {y:.2f})：{message}")
        else:
            self.add_event("warn", "nav_error", f"导航规划失败：{message}")
        return ok, message

    def _submit_goal(self, x: float, y: float, yaw_deg: float | None,
                     pose: tuple[float, float, float] | None = None,
                     ) -> tuple[bool, str]:
        """规划并启动导航（不写事件），供手动导航与自主探索共用。"""
        if not self._navigation_map_ready():
            return False, "实时地图缺失或过期，请等待建图更新"
        if pose is None:
            pose = self.map_pose()
            if pose is None:
                return False, "地图定位不可用或过期，请先启动建图并等待定位"
        with self._lock:
            grid = self.map
        return self.nav.set_goal(float(x), float(y), yaw_deg, grid, pose,
                                 time.monotonic())

    def cancel_nav(self, message: str = "导航已取消") -> tuple[bool, str]:
        self.nav.cancel(message)
        self.safety.clear_nav()
        self.add_event("info", "nav_cancel", message)
        return True, message

    def plan_preview(self) -> list[list[float]]:
        """当前导航路径（给前端画线）。"""
        return [[round(px, 3), round(py, 3)] for px, py in self.nav.snapshot().path]

    # ------------------------------------------------------------------ 自主探索
    def _set_exploration(self, state: str, message: str, now: float | None = None) -> None:
        with self._lock:
            self.exploration_state.update(
                state=state, message=str(message),
                since=time.monotonic() if now is None else float(now))
            if state in ("idle", "completed", "failed", "cancelled"):
                self._exploration_goal = None

    def _mapping_unavailable(self, status: dict | None = None) -> bool:
        """建图/定位是否不可用（只对声明了 mapping 能力的链路生效）。"""
        if not self._capability("mapping"):
            return False
        if status is None:
            try:
                status = self.transport.status()
            except Exception:
                return False
        mapping = status.get("mapping") or {}
        if str(mapping.get("state") or "") == "running":
            return False
        localization = status.get("localization") or {}
        if localization.get("valid"):
            return False
        return True

    def _navigation_map_ready(self, status: dict | None = None) -> bool:
        """实机导航必须使用新鲜的机载地图；导入的旧图仅用于查看。"""
        if self.link_kind != "ssh_ros":
            return True
        status = self._link_status() if status is None else status
        return bool((status.get("map_health") or {}).get("valid"))

    def exploration_snapshot(self) -> dict:
        with self._lock:
            payload = dict(self.exploration_state)
            payload["frontiers_preview"] = [list(point) for point in self._exploration_preview]
        payload["since"] = round(payload.get("since") or 0.0, 3)
        if payload.get("last_goal"):
            payload["last_goal"] = [round(float(v), 3) for v in payload["last_goal"]]
        return payload

    def start_exploration(self) -> tuple[bool, str]:
        """开始/继续自主探索：由前沿算法选目标，导航链路自动前往。"""
        if not bool(self.config.get("explore.enabled", True)):
            return False, "配置已禁用自主探索"
        if self.safety.estop:
            return False, "急停锁存中，无法开始探索"
        if self.safety.posture_active or self.posture_state.get("state") == "pending":
            return False, "姿态切换进行中，等待完成后再探索"
        if not self.link_up():
            return False, "链路未连接，无法开始探索"
        with self._lock:
            grid = self.map
        if grid is None:
            return False, "没有地图，无法探索；请先启动建图"
        status = self._link_status()
        if self._mapping_unavailable(status):
            return False, "建图未运行且定位无效，请先启动建图"
        if not self._navigation_map_ready(status):
            return False, "实时地图缺失或过期，请等待建图更新"
        pose = self.map_pose()
        if pose is None:
            return False, "地图定位不可用或过期，请先启动建图并等待定位"
        execution = self._execution_summary(status)
        if execution.get("armed") is False:
            return False, "未使能（自动模式），请先站立并点击「使能」"
        state = self.exploration_state.get("state")
        resume = state == "paused"
        if not resume:
            self.explorer.reset()
            self._exploration_failures = 0
            with self._lock:
                self.exploration_state.update(
                    goals_visited=0, blacklisted=0, consecutive_failures=0,
                    last_goal=None, frontiers=0)
            self._exploration_preview = []
            self._exploration_last_arrival_pose = None
            self._exploration_no_progress_arrivals = 0
        self._exploration_goal = None
        self._exploration_retry_at = 0.0
        self._exploration_observe_until = 0.0
        self._exploration_wait_started = None
        self._exploration_wait_signature = None
        self._set_exploration("active", "继续探索中" if resume else "探索中")
        self.note_operator_activity()
        self.add_event("info", "exploration_start",
                       "继续自主探索" if resume else "开始自主探索建图")
        return True, "继续探索" if resume else "已开始自主探索"

    def stop_exploration(self, reason: str = "操作者停止探索") -> tuple[bool, str]:
        state = self.exploration_state.get("state")
        if state not in ("active", "paused"):
            return True, "当前没有进行中的探索"
        self.nav.cancel(reason)
        self.safety.clear_nav()
        self._set_exploration("cancelled", reason)
        self._exploration_preview = []
        self.add_event("warn", "exploration_cancel", reason)
        return True, reason

    def _emit_exploration_error(self, message: str) -> None:
        """前沿计算异常限频上报（1 次 / 5 s）。"""
        now = time.monotonic()
        if now - getattr(self, "_exploration_error_time", 0.0) < 5.0:
            return
        self._exploration_error_time = now
        self.add_event("warn", "exploration_error", message)

    def _refresh_exploration_preview(self, now: float, force: bool = False) -> None:
        """前沿预览 1 Hz 刷新（避免每次 tick 都做全图扫描）。"""
        if not force and now - self._exploration_preview_time < 1.0:
            return
        self._exploration_preview_time = now
        with self._lock:
            grid = self.map
        if grid is None:
            self._exploration_preview = []
            return
        try:
            self._exploration_preview = self.explorer.frontiers_preview(grid, 200)
            stats = self.explorer.stats()
        except Exception as error:  # 算法异常不能拖死控制环
            self._emit_exploration_error(f"前沿计算失败：{error}")
            return
        with self._lock:
            self.exploration_state["frontiers"] = int(stats.get("frontiers") or 0)
            self.exploration_state["blacklisted"] = int(stats.get("blacklisted") or 0)

    def _exploration_tick(self, now: float, pose: tuple[float, float, float] | None,
                          link_up: bool, status: dict) -> None:
        """探索状态机：安全检查 -> 目标结果处理 -> 选下一个前沿目标。"""
        state = self.exploration_state.get("state")
        if state not in ("active", "paused"):
            return
        reason = ""
        if self.safety.estop:
            reason = "急停锁存，探索已取消"
        elif self.safety.posture_active:
            reason = "姿态切换，探索已取消"
        elif not link_up:
            reason = "链路断开，探索已取消"
        elif pose is None:
            reason = "地图定位失效，探索已取消"
        elif self._operator_stale(now):
            reason = "操作者心跳超时，探索已取消"
        elif self._mapping_unavailable(status):
            reason = "建图已停止且定位无效，探索已取消"
        elif not self._navigation_map_ready(status):
            reason = "实时地图过期，探索已取消"
        if reason:
            self.nav.cancel(reason)
            self.safety.clear_nav()
            self._set_exploration("cancelled", reason, now)
            self._exploration_preview = []
            self.add_event("warn", "exploration_cancel", reason)
            return

        if state != "active":
            self._refresh_exploration_preview(now)
            return

        threshold = float(self.config.get("safety.joystick_threshold", 0.03))
        if self.frame is not None and self.frame.joystick_active(threshold):
            self.nav.cancel("物理遥控接管，探索暂停")
            self.safety.clear_nav()
            self._set_exploration("paused", "物理遥控接管，探索暂停；需手动继续", now)
            self.add_event("warn", "exploration_paused", "物理遥控接管，探索暂停")
            return

        pose_xy = (float(pose[0]), float(pose[1]))
        nav_state = self.nav.state
        goal = self._exploration_goal
        max_failures = int(self.config.get("explore.max_consecutive_failures", 3))

        if goal is not None and nav_state == "arrived":
            with self._lock:
                reached_map = self.map
            if reached_map is not None:
                self.explorer.mark_visited(goal[0], goal[1], reached_map)
            previous = self._exploration_last_arrival_pose
            min_progress = max(0.1, float(self.config.get("nav.tolerance_m", 0.15)))
            progressed = previous is None or math.dist(previous, pose_xy) >= min_progress
            with self._lock:
                if progressed:
                    self.exploration_state["goals_visited"] += 1
                visited = self.exploration_state["goals_visited"]
            if progressed:
                self._exploration_last_arrival_pose = pose_xy
                self._exploration_no_progress_arrivals = 0
            else:
                self._exploration_no_progress_arrivals += 1
            self._exploration_goal = None
            self._exploration_failures = 0
            self._exploration_observe_until = now + float(
                self.config.get("explore.observation_wait_s", 3.0))
            self._exploration_wait_started = None
            with self._lock:
                self.exploration_state["consecutive_failures"] = 0
            self.add_event("info", "exploration_goal",
                           f"探索目标已到达（第 {visited} 个）")
            if self._exploration_no_progress_arrivals >= 2:
                self.nav.cancel("连续前沿目标未产生实际位移")
                self.safety.clear_nav()
                self._set_exploration("paused", "连续前沿目标未产生实际位移，探索暂停；请检查观测与地图", now)
                self.add_event("warn", "exploration_paused", self.exploration_state["message"])
                return
        elif goal is not None and nav_state in ("blocked", "stuck"):
            self.explorer.blacklist(goal[0], goal[1], nav_state)
            self._exploration_goal = None
            self._exploration_failures += 1
            with self._lock:
                self.exploration_state["consecutive_failures"] = self._exploration_failures
                self.exploration_state["blacklisted"] = int(
                    self.explorer.stats().get("blacklisted") or 0)
            self.add_event("warn", "exploration_goal",
                           f"探索目标失败（{nav_state}），已拉黑该区域")
            if self._exploration_failures >= max_failures:
                message = (f"连续 {self._exploration_failures} 次探索目标失败，已停止探索")
                self._set_exploration("failed", message, now)
                self.add_event("error", "exploration_failed", message)
                return
        elif goal is not None and nav_state in ("cancelled", "idle"):
            # 目标被手动/安全原因取消，作废本次目标后重新选择
            self._exploration_goal = None

        if self._exploration_goal is None and self.nav.state not in ("following", "aligning"):
            if now < max(self._exploration_observe_until, self._exploration_retry_at):
                self.exploration_state["message"] = "已到达前沿，停车等待新观测"
                self._refresh_exploration_preview(now)
                return
            self._exploration_retry_at = now + 1.0
            with self._lock:
                grid = self.map
            if grid is None:
                self._set_exploration("failed", "地图不可用，探索中止", now)
                self.add_event("error", "exploration_failed", "地图不可用，探索中止")
                return
            try:
                target = self.explorer.next_goal(grid, pose_xy, now)
            except Exception as error:  # 算法异常不能拖死控制环
                self._set_exploration("failed", f"前沿计算失败：{error}", now)
                self.add_event("error", "exploration_failed", f"前沿计算失败：{error}")
                return
            if target is None:
                stats = self.explorer.stats()
                why = stats.get("decision_reason", "no_candidates")
                if why == "no_candidates":
                    self._set_exploration("paused", "仍有未知边界，但没有安全候选目标；请检查地图或调整探索范围", now)
                    self.add_event("warn", "exploration_paused", self.exploration_state["message"])
                    return
                signature = (grid.width, grid.height, grid.resolution, grid.origin,
                             hash(bytes(grid.data)), why)
                if self._exploration_wait_started is None or signature != self._exploration_wait_signature:
                    self._exploration_wait_started = now
                    self._exploration_wait_signature = signature
                elapsed = now - self._exploration_wait_started
                if why == "no_frontiers" and elapsed >= float(self.config.get("explore.observation_wait_s", 3.0)):
                    message = "当前地图已无未知边界，探索完成"
                    self._set_exploration("completed", message, now)
                    self._exploration_preview = []
                    self.add_event("info", "exploration_complete", message)
                elif why != "no_frontiers" and elapsed >= float(self.config.get("explore.observation_timeout_s", 15.0)):
                    message = "已访问当前候选区域，但没有新增观测；探索暂停"
                    self._set_exploration("paused", message, now)
                    self.add_event("warn", "exploration_paused", message)
                else:
                    self.exploration_state["message"] = "停车等待建图更新，确认剩余未知边界"
                return
            self._exploration_wait_started = None
            self._exploration_wait_signature = None
            ok, message = self._submit_goal(target[0], target[1], None, pose)
            if ok:
                self._exploration_goal = (float(target[0]), float(target[1]))
                with self._lock:
                    self.exploration_state["last_goal"] = [round(target[0], 3),
                                                           round(target[1], 3)]
                self.add_event("info", "exploration_goal",
                               f"探索目标 ({target[0]:.2f}, {target[1]:.2f})：{message}")
            else:
                self.explorer.blacklist(target[0], target[1], "plan_failed")
                self._exploration_failures += 1
                with self._lock:
                    self.exploration_state["consecutive_failures"] = self._exploration_failures
                    self.exploration_state["blacklisted"] = int(
                        self.explorer.stats().get("blacklisted") or 0)
                if self._exploration_failures >= max_failures:
                    text = (f"连续 {self._exploration_failures} 次目标规划失败，已停止探索："
                            f"{message}")
                    self._set_exploration("failed", text, now)
                    self.add_event("error", "exploration_failed", text)
                    return
        self._refresh_exploration_preview(now)

    # ------------------------------------------------------------------ 录制
    def start_recording(self, prefix: str = "session") -> tuple[bool, str]:
        if self.recorder.active:
            return True, "录制已在进行"
        metadata = {
            "link": self.link_kind,
            "limits": self.config.get("limits"),
            "safety": self.config.get("safety"),
            "map": self.map_describe(),
            "map_align": self.config.get("map.align"),
        }
        try:
            directory = self.recorder.start(metadata, prefix=prefix)
        except Exception as error:
            self.add_event("error", "record_error", f"录制启动失败：{error}")
            return False, str(error)
        self.add_event("info", "record_start", f"开始录制：{directory.name}")
        return True, f"开始录制 {directory}"

    def stop_recording(self) -> tuple[bool, str]:
        if not self.recorder.active:
            return True, "当前没有录制任务"
        directory = self.recorder.stop()
        self.add_event("info", "record_stop", f"录制结束：{directory}")
        return True, f"录制已停止：{directory}"

    # ------------------------------------------------------------------ 仿真调试
    def simulate(self, joystick: Any = None, link: Any = None) -> tuple[bool, str]:
        simulate = getattr(self.transport, "simulate", None)
        if not callable(simulate):
            return False, "当前链路不支持仿真调试"
        try:
            simulate(joystick=joystick, link=link)
        except Exception as error:
            return False, f"仿真注入失败：{error}"
        detail = []
        if joystick is not None:
            detail.append(f"遥控摇杆={joystick}")
        if link is not None:
            detail.append(f"链路={'恢复' if link else '断开'}")
        message = "仿真注入：" + "，".join(detail)
        self.add_event("warn", "simulate", message)
        return True, message

    def _operator_stale(self, now: float) -> bool:
        """操作者心跳超时（导航/自动运动的安全前提）。"""
        if not bool(self.config.get("safety.require_operator", True)):
            return False
        timeout = float(self.config.get("safety.nav_operator_timeout_s", 2.5))
        if self.last_operator_activity <= 0:
            return True
        return (now - self.last_operator_activity) > timeout

    # ------------------------------------------------------------------ 控制 tick
    def tick(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        self.control_rate.tick(now)
        frame = self.frame
        status = {}
        try:
            status = self.transport.status()
        except Exception:
            status = {"connected": False}
        link_up = bool(status.get("connected"))
        self._apply_execution_limits(status)

        # 导航更新（必须在安全闸门之前，让本周期指令进入 safety）
        nav_active = self.nav.state in ("following", "aligning")
        if nav_active and self.safety.posture_active:
            self.nav.cancel("姿态切换，导航取消")
            nav_active = False
        elif nav_active and not self.safety.estop and self._operator_stale(now):
            self.nav.cancel("操作者心跳超时，导航取消")
            nav_active = False
            self.add_event("warn", "nav_operator_timeout",
                           "超过操作者心跳时限，自主导航已取消并零速")
        pose = self.map_pose()
        if nav_active and pose is None:
            self.nav.cancel("地图定位失效，导航取消")
            self.safety.clear_nav()
            nav_active = False
        if nav_active and not self._navigation_map_ready(status):
            self.nav.cancel("实时地图过期，导航取消")
            self.safety.clear_nav()
            nav_active = False
        # 自主探索状态机：先处理上一个目标的结果，再选下一个前沿目标
        self._exploration_tick(now, pose, link_up, status)
        nav_active = self.nav.state in ("following", "aligning")
        if nav_active and not self.safety.estop and pose is not None:
            with self._lock:
                grid = self.map
            command = self.nav.update(now, pose, grid)
            if command is not None:
                self.safety.set_nav_command(command[0], command[1], True)
            else:
                self.safety.set_nav_command(0.0, 0.0, False)
        elif not nav_active:
            self.safety.set_nav_command(0.0, 0.0, False)

        decision = self.safety.tick(now, frame, link_up)

        # 厂商遥控接管或急停时，导航任务一并取消，避免恢复后突然继续跑
        if decision.zero_reason in ("joystick_takeover", "estop") and \
                self.nav.state in ("following", "aligning"):
            self.nav.cancel("安全拦截，导航取消")
            self.safety.clear_nav()

        # 导航状态变化 -> 事件
        nav_state = self.nav.state
        if nav_state != self._last_nav_state:
            if nav_state == "arrived":
                self.add_event("info", "nav_arrived", "导航到达目标点")
            elif nav_state == "stuck":
                self.add_event("warn", "nav_stuck", "导航卡死，已中止")
            self._last_nav_state = nav_state

        # 持续输出（含零速），传输层内部还有一层零速看门狗
        try:
            self.transport.send_velocity(decision.linear_m_s, decision.angular_rad_s)
        except Exception as error:
            self.add_event("error", "link_error", f"速度指令下发失败：{error}")

        self._record_tick(now, decision)
        return {"decision": decision, "nav": self.nav.snapshot().as_dict(),
                "link": status}

    def _record_tick(self, now: float, decision) -> None:
        if not self.recorder.active:
            return
        hz = float(self.config.get("record.hz", 10.0))
        if now - self._last_row_time < 1.0 / max(hz, 1.0):
            return
        self._last_row_time = now
        frame = self.frame
        pose = self.map_pose()
        execution = self._execution_summary(self._link_status())
        row = {
            "t": round(now - self.started_at, 3),
            "ts": round(time.time(), 3),
            "pose": None if pose is None else [round(pose[0], 4), round(pose[1], 4),
                                               round(pose[2], 2)],
            "velocity_body": None if frame is None
            else [round(v, 4) for v in frame.velocity_body],
            "command": [round(decision.linear_m_s, 4), round(decision.angular_rad_s, 4)],
            "source": decision.source,
            "zero_reason": decision.zero_reason,
            "basic": None if frame is None else frame.basic,
            "estop": self.safety.estop,
            "nav": self.nav.state,
            "link": self.link_kind,
            "battery_percent": self._battery_percent(frame),
            "armed": execution["armed"],
            "execution_reason": execution["reason"],
            "posture": self.posture_state.get("state"),
            "exploration": self.exploration_state.get("state"),
        }
        self.recorder.write_row(row)

    # ------------------------------------------------------------------ 快照
    @staticmethod
    def _battery_percent(frame: Frame | None) -> float | None:
        """兼容厂商 0.00~1.00 与实测直接给百分数两种固件行为。"""
        if frame is None:
            return None
        raw = frame.raw or {}
        value = raw.get("battery_percent")
        if value is None:
            value = raw.get("battery_level_raw")
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number) or number < 0:
            return None
        return round(number if number > 1.5 else number * 100.0, 1)

    @staticmethod
    def _execution_summary(link: dict) -> dict:
        execution = (link or {}).get("execution") or {}
        inner = execution.get("control") or {}
        control = (link or {}).get("control") or {}
        armed = control.get("armed")
        if armed is None:
            armed = inner.get("armed")
        mode_confirmed = control.get("mode_confirmed")
        if mode_confirmed is None:
            mode_confirmed = inner.get("mode_confirmed")
        return {
            "armed": None if armed is None else bool(armed),
            "mode_confirmed": None if mode_confirmed is None else bool(mode_confirmed),
            "reason": execution.get("reason") or control.get("reason") or "",
            "depth_age_s": control.get("depth_age_s"),
            "pending": control.get("pending"),
            "last_error": control.get("last_error") or "",
        }

    def snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        with self._lock:
            frame = self.frame
            scan = list(self.scan)
            grid = self.map
            frame_count = self.frame_count
        safety_core = self.safety.snapshot()
        safety_payload = {
            "estop": safety_core["estop"],
            "estop_reason": safety_core["estop_reason"],
            "estop_engages": safety_core["estop_engages"],
            "output": safety_core["output"],
            "teleop": safety_core["teleop"],
            "nav_active": safety_core["nav_active"],
            "counters": safety_core["counters"],
        }
        nav = self.nav.snapshot().as_dict()
        align = self.config.get("map.align") or {}
        pose = None
        frame_payload = None
        map_pose = None
        if frame is not None:
            age = max(0.0, now - frame.stamp)
            frame_payload = {
                "source": frame.source,
                "battery_level_raw": frame.raw.get("battery_level_raw"),
                "age_s": round(age, 3),
                "position": [round(v, 4) for v in frame.position],
                "rpy_deg": [round(v, 3) for v in frame.rpy_deg],
                "velocity_body": [round(v, 4) for v in frame.velocity_body],
                "velocity_world": [round(v, 4) for v in frame.velocity_world],
                "basic": frame.basic,
                "gait": frame.gait,
                "motion": frame.motion,
                "joystick": [round(v, 4) for v in frame.joystick],
                "goal_velocity": [round(v, 4) for v in frame.goal_velocity],
                "mode_ready": frame.mode_ready(int(self.config.get("safety.ready_basic", 6))),
                "takeover": frame.joystick_active(
                    float(self.config.get("safety.joystick_threshold", 0.03))),
                "battery_percent": self._battery_percent(frame),
                "error_state": frame.raw.get("error_state"),
                "is_charging": frame.raw.get("is_charging"),
                "motion_state": frame.raw.get("motion_state"),
                "zero_position_flag": frame.raw.get("zero_position_flag"),
            }
            map_x, map_y, map_yaw = transform_pose(
                frame.position[0], frame.position[1], frame.yaw_deg, align)
            map_pose = {"x": round(map_x, 4), "y": round(map_y, 4),
                        "yaw_deg": round(map_yaw, 3)}
            pose = [round(map_x, 4), round(map_y, 4), round(map_yaw, 3)]
        try:
            link = self.transport.status()
        except Exception:
            link = {"connected": False, "kind": self.link_kind}
        link.setdefault("kind", self.link_kind)
        localized = self.map_pose()
        if localized is not None:
            pose = [round(value, 4) for value in localized]
            map_pose = dict(zip(("x", "y", "yaw_deg"), pose))
        elif self.link_kind == "ssh_ros":
            pose = None
            map_pose = None
        link.setdefault("connected", False)
        link["frames"] = frame_count
        link["last_frame_age_s"] = (
            round(now - frame.stamp, 3) if frame is not None else None)

        execution = self._execution_summary(link)
        battery_percent = self._battery_percent(frame)
        low_battery = (battery_percent is not None and
                       battery_percent <= float(self.config.get(
                           "safety.low_battery_percent", 20.0)))
        state_timeout = float(self.config.get("safety.state_timeout_s", 0.8))
        frame_age = None if frame is None else max(0.0, now - frame.stamp)
        with self._posture_lock:
            posture = dict(self.posture_state)
        if posture.get("since"):
            posture["age_s"] = round(now - posture["since"], 1)
        capabilities = dict(getattr(self.transport, "capabilities", {}) or {})
        link_health = link.get("health") or {}
        link_system = link.get("system") or {}
        link_control = link.get("control") or {}
        link_execution = link.get("execution") or {}
        control_session = (link_control.get("control_session")
                           or link_execution.get("control_session") or {})
        temperature_c = link_health.get("max_c")
        temperature_source = "joint" if temperature_c is not None else None
        if temperature_c is None:
            temperature_c = link_health.get("temperature_c")
            temperature_source = (link_health.get("temperature_source")
                                  or "board") if temperature_c is not None else None
        joint_temperature_c = link_health.get("joint_temperature_c")
        robot_version = self.robot_version or ""
        if not robot_version and link_system:
            robot_version = "@".join(
                str(part) for part in (link_system.get("rosdistro"),
                                       link_system.get("hostname")) if part)
        health = {
            "execution_reason": execution["reason"],
            "armed": execution["armed"],
            "mode_confirmed": execution["mode_confirmed"],
            "depth_age_s": execution["depth_age_s"],
            "bridge_depth_age_s": link_control.get("bridge_depth_age_s"),
            "camera_depth_age_s": link_control.get("camera_depth_age_s"),
            "depth_fresh": link_control.get("depth_fresh"),
            "control_session": control_session,
            "error_state": None if frame is None else frame.raw.get("error_state"),
            "battery_percent": battery_percent,
            "low_battery": low_battery,
            "temperature_c": temperature_c,
            "temperature_source": temperature_source,
            "joint_temperature_c": joint_temperature_c,
            "reconnect_attempts": (link.get("reconnect") or {}).get("attempts", 0),
            "bridge_ok": bool(link.get("connected")) and frame_age is not None
            and frame_age <= state_timeout,
            "versions": {"host_ui": self.version,
                         "robot": robot_version or None},
            "system": link_system,
        }

        return {
            "server_time": round(time.time(), 3),
            "uptime_s": round(now - self.started_at, 1),
            "link": link,
            "mapping": dict(link.get("mapping") or {}, **self._mapping_request,
                            map_saved=self._saved_map),
            "localization": link.get("localization") or {},
            "map_health": link.get("map_health") or {},
            "frame": frame_payload,
            "pose": pose,
            "map_pose": map_pose,
            "map": {
                "loaded": grid is not None,
                "describe": grid.describe() if grid is not None else None,
                "align": align,
            },
            "safety": safety_payload,
            "battery": {
                "percent": battery_percent,
                "low": low_battery,
                "charging": None if frame is None else frame.raw.get("is_charging"),
            },
            "health": health,
            "execution": execution,
            "posture": posture,
            "exploration": self.exploration_snapshot(),
            "operator": self.operator_snapshot(),
            "capabilities": capabilities,
            "version": self.version,
            "nav": nav,
            "limits": self.config.get("limits"),
            "nav_config": self.config.get("nav"),
            "safety_config": self.config.get("safety"),
            "counters": {
                "frames": frame_count,
                "frame_hz": round(self.frame_rate.hz, 1),
                "control_hz": round(self.control_rate.hz, 1),
                "broadcast_hz": round(self.broadcast_rate.hz, 1),
                "odometer_m": round(self.trail.odometer_m, 2),
                "trail_points": len(self.trail.points),
                "scan_points": len(scan),
                "clients": self.clients,
            },
            "recording": self.recorder.status(),
            "simulation": getattr(self.transport, "kind", "") == "mock",
            "events": self.events.recent(1),
        }

    def hello(self) -> dict:
        """WebSocket 建连时的完整初始化数据。"""
        return {
            "snapshot": self.snapshot(),
            "map": self.map_payload(),
            "trail": [[round(x, 3), round(y, 3)] for x, y in self.trail.all_points()[-4000:]],
            "events": self.events.recent(120),
            "config": self.config.public(),
            "kinds": list(available_kinds()),
            "kind_labels": dict(KINDS),
            "nav_path": self.plan_preview(),
        }

    # ------------------------------------------------------------------ 关闭
    def shutdown(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self.safety.release_teleop()
        self.disconnect(reason="服务退出", silent=True)
        if self.recorder.active:
            self.recorder.stop()
