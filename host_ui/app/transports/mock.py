"""离线演示传输：50 Hz 单轨运动学仿真 + 演示地图 + 仿真激光。

用途
----
没有机器狗、没有 ROS 时，前端仍然需要一个"活"的数据源：状态帧持续刷新、
地图能显示、速度指令能改变位姿、断链/遥控接管/撞墙等异常能在界面上复现。
本模块就是这条链路（``kind="mock"``）。

仿真模型与线程
--------------
* 单轨（unicycle）模型：``yaw += w*dt``，``x += v*cos(yaw)*dt``，
  ``y += v*sin(yaw)*dt``；线速度/角速度都受加速度限制，逼近目标值而不是跳变。
* 50 Hz 独立线程推进仿真；``send_velocity`` 只写内存（非阻塞），超过
  ``zero_timeout_s`` 没有新指令时目标速度自动归零（与实机看门狗语义一致）。
* 用 :func:`host_ui.app.mapstore.demo_map` 做碰撞检测：机器人包络按
  ``collision_radius_m`` 膨胀，压到障碍即回退本步位移、``v=0`` 并触发
  ``collision`` 事件（限频 1 次/秒）。
* 地图在 ``start()`` 时通过 ``callbacks.on_map`` 推送一次；状态帧
  ``source="mock"``，``basic=6``、``gait=2``、``motion=0``。
* 可选仿真激光：栅格 DDA 打 90 条射线（270° 视场、步进式精确格点），
  命中点（≤90 个）以里程计坐标通过 ``callbacks.on_scan`` 推送，≤10 Hz。
* 渐进建图仿真（``set_mapping(True)``）：创建与 demo 地图同尺寸/同分辨率/
  同原点的**全未知**掩膜 ``GridMap(name="map")``，``_step`` 里 ≤2 Hz 从当前
  位姿打 120 条 4 m 射线把沿途写成 FREE、命中写成 OCCUPIED，≤1 Hz 通过
  ``callbacks.on_map`` 推送掩膜拷贝；``set_mapping(False)`` 停止并最后推送一次。
  真实碰撞地图 ``self._map`` 始终保持不变（掩膜是独立对象）。``status().mapping``
  反映 running/idle，``status().localization.valid`` 严格跟随“建图是否运行”
  （契约 §5.3，离线 E2E 的探索前置条件：停建图后不再放行探索）。

调试入口
--------
* ``simulate(joystick=[x, y, z], link=bool)``：模拟遥控器接管 / 链路断开，
  供前端"模拟遥控接管/断链"按钮使用；``link=False`` 时线程保留但不产生帧。
* ``reset(x, y, yaw_deg)``：把机器人放到指定位姿，便于演示某个房间或碰撞场景。

线程安全：所有可变状态由 ``self._lock`` 保护；回调一律在锁外触发并吞掉异常，
避免界面代码异常杀死仿真线程；``stop()`` 幂等。
"""
from __future__ import annotations

import math
import random
import threading
import time
from typing import Any, Iterable

from ..mapstore import demo_map
from ..types import FREE, OCCUPIED, Frame, GridMap, wrap_deg
from .base import Transport, TransportCallbacks

#: 前端连接面板的默认参数
DEFAULT_OPTIONS: dict[str, Any] = {
    "zero_timeout_s": 0.3,
    "max_linear_m_s": 0.6,
    "max_angular_rad_s": 0.8,
    "fps": 50.0,
    "collision_radius_m": 0.26,
    "scan_enabled": True,
    "scan_max_hz": 10.0,
}

#: 线加速度上限（m/s²）
LINEAR_ACCEL = 1.2
#: 角加速度上限（rad/s²）
ANGULAR_ACCEL = 3.0
#: 仿真激光参数
SCAN_RAYS = 90
SCAN_FOV_RAD = math.radians(270.0)
SCAN_MAX_RANGE_M = 5.0
#: 渐进建图掩膜参数（契约 §5.3）：360° 约 120 条射线、量程 4 m
MAPPING_RAYS = 120
MAPPING_MAX_RANGE_M = 4.0
#: 掩膜射线更新频率（≤2 Hz）与地图推送频率（≤1 Hz）
MAPPING_RAYCAST_HZ = 2.0
MAPPING_PUBLISH_HZ = 1.0


def _as_float(value: Any, fallback: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def _as_bool(value: Any, fallback: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    if value is None:
        return fallback
    return bool(value)


def _approach(current: float, target: float, max_delta: float) -> float:
    """按最大变化量逼近目标值（用于加减速限制）。"""
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


class MockTransport(Transport):
    """离线演示链路。"""

    kind = "mock"
    label = "离线演示"
    default_options = dict(DEFAULT_OPTIONS)
    #: 离线联调能力：使能/姿态/厂商急停都模拟；建图用掩膜地图渐进披露
    capabilities = {"mapping": True, "arm": True, "posture": True,
                    "vendor_estop": True, "sensors": False, "reconnect": False}

    @staticmethod
    def available() -> bool:
        return True

    def __init__(self, options: dict | None, callbacks: TransportCallbacks) -> None:
        super().__init__(options, callbacks)
        opts = self.options
        self.zero_timeout_s = max(0.05, _as_float(
            opts.get("zero_timeout_s"), 0.3))
        self.max_linear_m_s = max(0.05, _as_float(
            opts.get("max_linear_m_s"), 0.6))
        self.max_angular_rad_s = max(0.05, _as_float(
            opts.get("max_angular_rad_s"), 0.8))
        self.fps = min(200.0, max(10.0, _as_float(opts.get("fps"), 50.0)))
        self.collision_radius_m = max(0.0, _as_float(
            opts.get("collision_radius_m"), 0.26))
        self.scan_enabled = _as_bool(opts.get("scan_enabled"), True)
        self.scan_max_hz = min(10.0, max(1.0, _as_float(
            opts.get("scan_max_hz"), 10.0)))

        self._lifecycle_lock = threading.Lock()
        self._lock = threading.Lock()
        self._event_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False
        self._map: GridMap | None = None
        # 渐进建图掩膜：独立于 self._map 的 GridMap，所有读写都在 self._lock 内
        self._mapping_mask: GridMap | None = None
        self._mapping_state = "idle"
        self._mapping_message = "建图未运行"
        self._last_mapping_ray_time = 0.0
        self._last_mapping_publish_time = 0.0

        self._x = 0.0
        self._y = 0.0
        self._yaw_deg = 0.0
        self._v = 0.0
        self._w = 0.0
        self._command = (0.0, 0.0)
        self._command_time = 0.0
        self._link = True
        self._joystick = (0.0, 0.0, 0.0)

        self._rng = random.Random(20261114)
        self._started_at = time.monotonic()
        self._last_scan_time = 0.0
        self._frames = 0
        self._scans = 0
        self._collisions = 0
        self._event_times: dict[str, float] = {}

        # ------------------------------------------------------------------
        # 使能 / 姿态 / 急停模拟（供前端离线联调，契约第 3.6 节）
        # ------------------------------------------------------------------
        self.posture_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._armed = False
        self._mode_confirmed = False
        self._estop = False
        self._pending: str | None = None
        self._last_error = ""
        self._auto_ack_time = 0.0
        self._basic = 6
        self._arm_delay_s = max(0.0, _as_float(opts.get("arm_delay_s"), 0.8))
        self._posture_delay_s = max(0.0, _as_float(
            opts.get("posture_delay_s"), 0.6))
        self._disarm_delay_s = max(0.0, _as_float(
            opts.get("disarm_delay_s"), 0.2))

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
        except Exception:  # 界面回调异常不能杀死仿真线程
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
        """构建演示地图、推送 on_map 并启动 50 Hz 仿真线程。"""
        grid: GridMap | None = None
        with self._lifecycle_lock:
            if self._running:
                return
            grid = demo_map()
            self._map = grid
            self._stop_event.clear()
            self._running = True
            self._started_at = time.monotonic()
            self._last_scan_time = 0.0
            with self._lock:
                self._frames = 0
                self._scans = 0
                self._collisions = 0
                self._v = 0.0
                self._w = 0.0
                self._command = (0.0, 0.0)
                self._command_time = 0.0
                self._link = True
                self._joystick = (0.0, 0.0, 0.0)
                self._basic = 6
                self._mapping_mask = None
                self._mapping_state = "idle"
                self._mapping_message = "建图未运行"
                self._last_mapping_ray_time = 0.0
                self._last_mapping_publish_time = 0.0
            with self._control_lock:
                self._armed = False
                self._mode_confirmed = False
                self._estop = False
                self._pending = None
                self._last_error = ""
                self._auto_ack_time = 0.0
            self._thread = threading.Thread(
                target=self._sim_loop, name="lite2-mock-sim", daemon=True)
            self._thread.start()
        # 地图回调放在锁外，允许前端在回调里立刻调用 send_velocity 等方法
        self._safe_callback(self.callbacks.on_map, grid)
        self._emit_event(
            "info", "link_up",
            f"离线仿真已启动：{grid.width}x{grid.height} 格，"
            f"{grid.resolution:.2f} m/格，{self.fps:.0f} Hz")

    def stop(self) -> None:
        """幂等停止仿真线程（内存链路，无需补发网络零速，指令会归零）。"""
        with self._lifecycle_lock:
            if not self._running:
                with self._lock:
                    self._command = (0.0, 0.0)
                    self._v = 0.0
                    self._w = 0.0
                return
            self._running = False
            self._stop_event.set()
            with self._lock:
                self._command = (0.0, 0.0)
                self._v = 0.0
                self._w = 0.0
                self._mapping_mask = None
                self._mapping_state = "idle"
                self._mapping_message = "建图未运行"
            thread = self._thread
            self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._emit_event("info", "link_down", "离线仿真已停止")

    # ------------------------------------------------------------------ 数据面
    def send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        """写入目标速度（非阻塞）；超出仿真上限的部分会被限幅。"""
        try:
            linear = float(linear_m_s)
            angular = float(angular_rad_s)
        except (TypeError, ValueError):
            self._emit_event("warn", "invalid_command",
                             "速度指令不是数值，已忽略", 2.0)
            return
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self._emit_event("warn", "invalid_command",
                             "速度指令含 NaN/Inf，已忽略", 2.0)
            return
        linear = max(-self.max_linear_m_s, min(self.max_linear_m_s, linear))
        angular = max(-self.max_angular_rad_s, min(self.max_angular_rad_s, angular))
        if self.posture_lock.locked():
            linear, angular = 0.0, 0.0
        with self._lock:
            if not self._link:
                return
            self._command = (linear, angular)
            self._command_time = time.monotonic()

    # ------------------------------------------------------------------ 能力面
    def _park(self) -> None:
        with self._lock:
            self._command = (0.0, 0.0)
            self._command_time = 0.0
            self._v = 0.0
            self._w = 0.0

    @staticmethod
    def _sleep_with_cancel(seconds: float, cancel=None) -> bool:
        """分段睡眠；``cancel`` 置位时返回 False（用于姿态取消）。"""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if cancel is not None and cancel.is_set():
                return False
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return not (cancel is not None and cancel.is_set())

    def arm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        """模拟机载使能：0.8 s 后 armed=True（可配置 ``arm_delay_s``）。"""
        if not self._running:
            return False, "离线仿真未启动，无法使能"
        with self._control_lock:
            if self._armed:
                return True, "离线仿真：已处于使能状态"
            self._pending = "prepare"
            self._last_error = ""
        try:
            delay = min(self._arm_delay_s, max(0.0, _as_float(timeout_s, 8.0)))
            self._sleep_with_cancel(delay)
            with self._control_lock:
                self._armed = True
                self._mode_confirmed = True
                self._auto_ack_time = time.monotonic()
                self._pending = None
            self._emit_event("info", "arm", "离线仿真：已使能（模拟自动模式确认）")
            return True, "离线仿真：已使能（模拟自动模式确认）"
        finally:
            with self._control_lock:
                if self._pending == "prepare":
                    self._pending = None

    def disarm(self, timeout_s: float = 8.0) -> tuple[bool, str]:
        """模拟退出自动模式并请求零速。"""
        if not self._running:
            return False, "离线仿真未启动，无法解除"
        with self._control_lock:
            self._pending = "disarm"
            self._last_error = ""
        try:
            self._park()
            self._sleep_with_cancel(min(self._disarm_delay_s,
                                        max(0.0, _as_float(timeout_s, 8.0))))
            with self._control_lock:
                self._armed = False
                self._mode_confirmed = False
                self._pending = None
            self._emit_event("info", "disarm", "离线仿真：已解除使能（零速）")
            return True, "离线仿真：已解除使能（已补发零速）"
        finally:
            with self._control_lock:
                if self._pending == "disarm":
                    self._pending = None

    def set_posture(self, action: str, cancel=None,
                    timeout_s: float = 10.0) -> tuple[bool, str]:
        """模拟起立/趴下：0.6 s 后 basic 1↔6 切换，``cancel`` 可中止。"""
        if action not in ("stand", "lie"):
            return False, "姿态指令无效：%s（必须是 stand 或 lie）" % action
        target = 6 if action == "stand" else 1
        label = "站立" if action == "stand" else "趴下"
        if not self._running:
            return False, "离线仿真未启动，无法切换姿态"
        if not self.posture_lock.acquire(blocking=False):
            return False, "已有姿态切换正在执行"
        with self._control_lock:
            self._pending = "posture"
            self._last_error = ""
        try:
            with self._lock:
                current = self._basic
            if current == target:
                return True, "离线仿真：机器人已处于%s状态（basic=%d）" % (label, target)
            if current not in (1, 6):
                return False, ("离线仿真：当前 basic=%d 正在切换或保护中，"
                               "请等待稳定后再试" % current)
            delay = min(self._posture_delay_s,
                        max(0.1, _as_float(timeout_s, 10.0)))
            if not self._sleep_with_cancel(delay, cancel):
                self._park()
                self._emit_event("warn", "posture_cancel", "离线仿真：姿态切换已取消")
                return False, "姿态切换已取消"
            with self._lock:
                self._basic = target
            self._emit_event("info", "posture",
                             "离线仿真：已%s（basic=%d）" % (label, target))
            return True, "离线仿真：已确认%s（basic=%d）" % (label, target)
        finally:
            with self._control_lock:
                if self._pending == "posture":
                    self._pending = None
            self.posture_lock.release()

    def emergency_stop(self) -> tuple[bool, str]:
        """模拟厂商硬件急停：立即停车并进入 estop 状态（basic 保持）。"""
        self._park()
        with self._control_lock:
            self._estop = True
            self._armed = False
            self._mode_confirmed = False
            self._last_error = "离线仿真：厂商急停已触发"
        self._emit_event("error", "vendor_estop",
                         "离线仿真：厂商急停 0x21010C0E 已触发（basic 保持）")
        return True, "离线仿真：厂商急停已触发（basic 保持，需物理/软件复位）"

    def reset_estop(self) -> tuple[bool, str]:
        """清除离线仿真的急停标记（真实链路需物理复位）。"""
        with self._control_lock:
            self._estop = False
            if self._last_error == "离线仿真：厂商急停已触发":
                self._last_error = ""
        return True, "离线仿真：急停标记已清除"

    # ------------------------------------------------------------------ 建图仿真
    def set_mapping(self, running: bool) -> tuple[bool, str]:
        """模拟机载建图启停：维护一张独立的“全未知”掩膜地图并渐进披露。

        * 启动：创建与 :func:`demo_map` 同尺寸/同分辨率/同原点的全未知
          ``GridMap``（``name="map"``），立即做一次局部披露并推送；
        * 运行：``_step`` 里 ≤2 Hz 打 120 条 4 m 射线写掩膜，≤1 Hz 推送拷贝；
        * 停止：停止更新，最后再推送一次掩膜。
        真实的 ``self._map``（碰撞/射线所用 demo 地图）始终保持不变。
        """
        running = bool(running)
        mask: GridMap | None = None
        with self._lock:
            if not self._running:
                return False, "离线仿真未启动，无法控制建图"
            if running:
                if self._mapping_state == "running":
                    return True, "离线仿真：建图已在运行"
                base = self._map
                if base is None:  # pragma: no cover - start() 已保证有地图
                    base = demo_map()
                    self._map = base
                mask = GridMap(base.width, base.height, base.resolution,
                               base.origin, name="map")
                self._mapping_mask = mask
                self._mapping_state = "running"
                self._mapping_message = "离线仿真：建图运行中（雷达渐进披露）"
                self._last_mapping_ray_time = 0.0
                self._last_mapping_publish_time = 0.0
                # 立即局部披露一次，UI 不必等下一个 2 Hz 周期
                now = time.monotonic()
                self._update_mapping_mask_locked()
                self._last_mapping_ray_time = now
                self._last_mapping_publish_time = now
            else:
                if self._mapping_state != "running":
                    self._mapping_state = "idle"
                    self._mapping_message = "建图未运行"
                    return True, "离线仿真：建图未运行"
                mask = self._mapping_mask
                self._mapping_state = "idle"
                self._mapping_message = "离线仿真：建图已停止"
                self._mapping_mask = None
            message = self._mapping_message
        # 回调必须在锁外触发，且推送拷贝：外部改不到内部掩膜缓冲
        if mask is not None:
            self._safe_callback(self.callbacks.on_map, self._copy_mask(mask))
        self._emit_event("info",
                         "mapping_started" if running else "mapping_stopped",
                         message)
        return True, message

    def _update_mapping_mask_locked(self) -> None:
        """当前位姿 360° 打 120 条射线，把沿途 FREE / 命中 OCCUPIED 写入掩膜。

        调用方必须持有 ``self._lock``；命中判定复用 :meth:`_raycast`（作用于真实
        demo 地图，因此碰撞与仿真激光语义不变），掩膜只记录披露结果。
        """
        mask = self._mapping_mask
        grid = self._map
        if mask is None or grid is None:
            return
        yaw_rad = math.radians(self._yaw_deg)
        step = grid.resolution * 0.5
        for index in range(MAPPING_RAYS):
            angle = yaw_rad + 2.0 * math.pi * index / MAPPING_RAYS
            distance = self._raycast(grid, self._x, self._y, angle,
                                     MAPPING_MAX_RANGE_M)
            limit = MAPPING_MAX_RANGE_M if distance is None else distance
            travelled = step
            while travelled < limit:
                wx = self._x + travelled * math.cos(angle)
                wy = self._y + travelled * math.sin(angle)
                gx, gy = mask.world_to_index(wx, wy)
                if not mask.contains_cell(gx, gy):
                    break
                mask.set(gx, gy, FREE)
                travelled += step
            if distance is not None:
                hx = self._x + distance * math.cos(angle)
                hy = self._y + distance * math.sin(angle)
                gx, gy = mask.world_to_index(hx, hy)
                if mask.contains_cell(gx, gy):
                    mask.set(gx, gy, OCCUPIED)

    @staticmethod
    def _copy_mask(mask: GridMap) -> GridMap:
        """深拷贝掩膜（新建 GridMap，避免外部修改内部缓冲）。"""
        return GridMap(mask.width, mask.height, mask.resolution, mask.origin,
                       data=bytes(mask.data), name=mask.name)

    # ------------------------------------------------------------------ 调试入口
    def simulate(self, joystick: Iterable[float] | None = None,
                 link: bool | None = None) -> dict:
        """模拟遥控接管（``joystick``）与链路断开/恢复（``link``）。

        参数为 ``None`` 表示保持当前值。返回更新后的仿真状态快照。
        """
        link_changed: bool | None = None
        joystick_changed = False
        with self._lock:
            if joystick is not None:
                values = [float(value) for value in list(joystick)[:3]]
                while len(values) < 3:
                    values.append(0.0)
                self._joystick = tuple(values)
                joystick_changed = True
                # 遥控接管时立即取消自主指令，避免"接管后还在动"
                if max(abs(value) for value in self._joystick) >= 0.03:
                    self._command = (0.0, 0.0)
                    self._command_time = 0.0
            if link is not None:
                new_link = bool(link)
                if new_link != self._link:
                    self._link = new_link
                    link_changed = new_link
                    if not new_link:
                        self._v = 0.0
                        self._w = 0.0
                        self._command = (0.0, 0.0)
                        self._command_time = 0.0
            snapshot = self._snapshot_locked()
        if link_changed is True:
            self._emit_event("info", "link_up", "链路已恢复，仿真继续产生状态帧")
        elif link_changed is False:
            self._emit_event("warn", "link_down",
                             "已模拟断链：仿真暂停，不再产生状态帧")
        if joystick_changed and max(abs(v) for v in snapshot["joystick"]) >= 0.03:
            self._emit_event(
                "warn", "joystick_takeover",
                f"检测到模拟遥控接管（摇杆 {snapshot['joystick']}），自主指令已取消")
        return snapshot

    def reset(self, x: float = 0.0, y: float = 0.0, yaw_deg: float = 0.0) -> dict:
        """把机器人瞬移到指定位姿（用于演示指定房间/碰撞场景）。"""
        with self._lock:
            self._x = float(x)
            self._y = float(y)
            self._yaw_deg = float(wrap_deg(yaw_deg))
            self._v = 0.0
            self._w = 0.0
            self._command = (0.0, 0.0)
            self._command_time = 0.0
            snapshot = self._snapshot_locked()
        self._emit_event(
            "info", "reset",
            f"仿真位姿已重置为 ({snapshot['pose'][0]:.2f}, "
            f"{snapshot['pose'][1]:.2f}, {snapshot['pose'][2]:.1f}°)")
        return snapshot

    def _snapshot_locked(self) -> dict:
        return {
            "pose": (self._x, self._y, self._yaw_deg),
            "velocity": (self._v, self._w),
            "joystick": self._joystick,
            "link": self._link,
        }

    # ------------------------------------------------------------------ 仿真线程
    def _sim_loop(self) -> None:
        dt = 1.0 / self.fps
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            self._step(now, dt)
            next_tick += dt
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                # 线程被长时间挂起后不要补算，直接对齐当前时间
                next_tick = time.monotonic()

    def _step(self, now: float, dt: float) -> None:
        grid = self._map
        collision = False
        scan_due = False
        mapping_map: GridMap | None = None
        with self._lock:
            if not self._link:
                return
            joystick_active = max(abs(value) for value in self._joystick) >= 0.03
            if joystick_active:
                target_v, target_w = 0.0, 0.0
            elif now - self._command_time <= self.zero_timeout_s:
                target_v, target_w = self._command
            else:
                target_v, target_w = 0.0, 0.0
            # 加减速限制
            v = _approach(self._v, target_v, LINEAR_ACCEL * dt)
            w = _approach(self._w, target_w, ANGULAR_ACCEL * dt)
            # 测量噪声：让界面曲线更像实机（位置仍由运动学决定）
            v_measured = v + self._rng.gauss(0.0, 0.004)
            yaw_rad = math.radians(self._yaw_deg)
            nx = self._x + v_measured * math.cos(yaw_rad) * dt
            ny = self._y + v_measured * math.sin(yaw_rad) * dt
            if grid is not None and grid.is_occupied_world(
                    nx, ny, self.collision_radius_m):
                nx, ny = self._x, self._y
                v = 0.0
                v_measured = 0.0
                collision = True
            self._x, self._y = nx, ny
            self._yaw_deg = wrap_deg(self._yaw_deg + math.degrees(w * dt))
            self._v, self._w = v, w
            self._frames += 1
            frames = self._frames
            if collision:
                self._collisions += 1
            if (self.scan_enabled and self._map is not None
                    and now - self._last_scan_time >= 1.0 / self.scan_max_hz):
                self._last_scan_time = now
                scan_due = True
            # 渐进建图：≤2 Hz 更新掩膜，≤1 Hz 推送拷贝（回调在锁外触发）
            if self._mapping_state == "running" and self._mapping_mask is not None:
                if now - self._last_mapping_ray_time >= 1.0 / MAPPING_RAYCAST_HZ:
                    self._last_mapping_ray_time = now
                    self._update_mapping_mask_locked()
                if now - self._last_mapping_publish_time >= 1.0 / MAPPING_PUBLISH_HZ:
                    self._last_mapping_publish_time = now
                    mapping_map = self._copy_mask(self._mapping_mask)
            pose = (self._x, self._y, self._yaw_deg)
            elapsed = now - self._started_at
            basic = self._basic
        if collision:
            self._emit_event(
                "warn", "collision",
                f"仿真机器人撞到障碍，已回退并停车（位置 "
                f"({pose[0]:.2f}, {pose[1]:.2f})）", 1.0)
        # 姿态小幅晃动，便于界面观察 rpy 曲线
        roll = 0.6 * math.sin(elapsed * 1.7)
        pitch = 0.5 * math.sin(elapsed * 1.3 + 0.7)
        frame = Frame(
            source="mock",
            stamp=now,
            position=tuple(float(value) for value in pose),
            rpy_deg=(float(roll), float(pitch), float(pose[2])),
            velocity_body=(float(v_measured), 0.0, 0.0),
            velocity_world=(float(v_measured * math.cos(math.radians(pose[2]))),
                            float(v_measured * math.sin(math.radians(pose[2]))), 0.0),
            basic=basic,
            gait=2,
            motion=0,
            joystick=tuple(float(value) for value in self._joystick),
            goal_velocity=(float(target_v), 0.0, float(target_w)),
            raw={"sim": True, "link": True, "frames": frames,
                 "battery_level_raw": 0.87, "battery_percent": 87.0,
                 "error_state": 0, "is_charging": False,
                 "motion_state": 0, "zero_position_flag": False},
        )
        self._safe_callback(self.callbacks.on_frame, frame)
        if scan_due:
            points = self._raycast_scan(grid, pose[0], pose[1], pose[2])
            with self._lock:
                self._scans += 1
            self._safe_callback(self.callbacks.on_scan, points)
        if mapping_map is not None:
            self._safe_callback(self.callbacks.on_map, mapping_map)

    # ------------------------------------------------------------------ 仿真激光
    def _raycast_scan(self, grid: GridMap, x: float, y: float,
                      yaw_deg: float) -> list[tuple[float, float]]:
        """从位姿向 270° 视场打 90 条射线，返回命中的里程计坐标点。"""
        points: list[tuple[float, float]] = []
        yaw_rad = math.radians(yaw_deg)
        for index in range(SCAN_RAYS):
            angle = yaw_rad - SCAN_FOV_RAD / 2.0 + SCAN_FOV_RAD * index / (SCAN_RAYS - 1)
            distance = self._raycast(grid, x, y, angle, SCAN_MAX_RANGE_M)
            if distance is None:
                continue
            jitter = self._rng.uniform(-0.01, 0.01)
            hit = max(0.02, distance + jitter)
            points.append((x + hit * math.cos(angle), y + hit * math.sin(angle)))
        return points

    @staticmethod
    def _raycast(grid: GridMap, x: float, y: float, angle: float,
                 max_range: float) -> float | None:
        """栅格 DDA（Amanatides & Woo）：返回首个占据格的距离，未命中返回 None。"""
        resolution = grid.resolution
        gx_f, gy_f = grid.world_to_cell(x, y)
        gx, gy = int(math.floor(gx_f)), int(math.floor(gy_f))
        dx, dy = math.cos(angle), math.sin(angle)
        if dx == 0.0 and dy == 0.0:
            return None
        step_x = 1 if dx > 0 else -1
        step_y = 1 if dy > 0 else -1
        if dx != 0.0:
            next_x = gx + (1 if dx > 0 else 0)
            t_max_x = (next_x - gx_f) / dx
            t_delta_x = 1.0 / abs(dx)
        else:
            t_max_x = math.inf
            t_delta_x = math.inf
        if dy != 0.0:
            next_y = gy + (1 if dy > 0 else 0)
            t_max_y = (next_y - gy_f) / dy
            t_delta_y = 1.0 / abs(dy)
        else:
            t_max_y = math.inf
            t_delta_y = math.inf
        max_t = max_range / resolution
        travelled = 0.0
        while travelled <= max_t:
            if t_max_x < t_max_y:
                gx += step_x
                travelled = t_max_x
                t_max_x += t_delta_x
            else:
                gy += step_y
                travelled = t_max_y
                t_max_y += t_delta_y
            if travelled > max_t:
                return None
            if not grid.contains_cell(gx, gy):
                return None
            if grid.is_occupied(gx, gy):
                return travelled * resolution
        return None

    # ------------------------------------------------------------------ 状态
    def status(self) -> dict:
        with self._lock:
            snapshot = self._snapshot_locked()
            frames = self._frames
            scans = self._scans
            collisions = self._collisions
            running = self._running
            grid = self._map
            mapping_state = self._mapping_state
            mapping_message = self._mapping_message
        connected = bool(running and snapshot["link"])
        with self._control_lock:
            armed = self._armed
            mode_confirmed = self._mode_confirmed
            estop = self._estop
            pending = self._pending
            last_error = self._last_error
            auto_ack = self._auto_ack_time
        control = {
            "armed": bool(armed),
            "mode_confirmed": bool(mode_confirmed),
            "pending": pending,
            "last_error": last_error,
            "depth_age_s": None,
            "automatic_ack_age_s": (round(time.monotonic() - auto_ack, 2)
                                    if auto_ack > 0.0 else None),
            "estop": bool(estop),
        }
        mapping = {"state": mapping_state, "message": mapping_message}
        # 定位徽标一致性：mock 的 x/y/yaw_deg 直接来自运动学仿真（map 坐标系），
        # valid 必须与契约 §5.3 一致地跟随“建图是否运行”——否则停止建图后
        # node.start_exploration() 仍会认为定位有效而放行探索（与真机语义不符）。
        localization = {
            "valid": bool(mapping_state == "running"),
            "x": round(snapshot["pose"][0], 4),
            "y": round(snapshot["pose"][1], 4),
            "yaw_deg": round(snapshot["pose"][2], 3),
            "age_s": 0.0,
            "source": "mock",
            "frame": "map",
        }
        if not running:
            detail = "未启动"
        elif not snapshot["link"]:
            detail = "已模拟断链：不产生状态帧（点击恢复链路继续）"
        else:
            detail = (f"仿真运行中，位姿 ({snapshot['pose'][0]:.2f}, "
                      f"{snapshot['pose'][1]:.2f}, {snapshot['pose'][2]:.1f}°)，"
                      f"已推送 {frames} 帧")
        return {
            "kind": self.kind,
            "label": self.label,
            "connected": connected,
            "detail": detail,
            "link": bool(snapshot["link"]),
            "pose": [round(snapshot["pose"][0], 4), round(snapshot["pose"][1], 4),
                     round(snapshot["pose"][2], 3)],
            "velocity": [round(snapshot["velocity"][0], 4),
                         round(snapshot["velocity"][1], 4)],
            "joystick": list(snapshot["joystick"]),
            "frames": frames,
            "scans": scans,
            "collisions": collisions,
            "map": grid.describe() if grid is not None else None,
            "mapping": mapping,
            "localization": localization,
            "zero_timeout_s": self.zero_timeout_s,
            "basic": int(self._basic),
            "control": control,
            "reconnect": {"enabled": False, "attempts": 0,
                          "next_retry_in_s": None},
            "execution": {},
        }