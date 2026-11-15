"""FastAPI 服务：REST 接口 + WebSocket 实时总线 + 静态页面。

线程/协程模型
------------
* ``control_loop`` 以 25 Hz 调 :meth:`HostNode.tick`，最终速度在这里下发；
* ``broadcast_loop`` 以 20 Hz 广播状态快照、轨迹增量、扫描点、事件；
* 前端指令通过 WebSocket 文本消息进入，也可以走 REST（便于 curl/回归测试）；
* 传输层线程通过回调写 ``HostNode``，内部加锁，不直接碰事件循环。

安全边界
--------
1. 默认只监听 ``127.0.0.1``；一旦绑定非回环地址，必须配置 ``web.token``，
   否则启动直接拒绝（见 :func:`create_app`）。
2. WebSocket 校验 Origin 白名单与令牌；REST 校验 ``X-Auth-Token``。
3. 操控权（operator）独占：除急停/停车外，运动、使能、姿态、导航、限幅
   都只能由当前操控者发起。
"""
from __future__ import annotations

import asyncio
import hmac
import ipaddress
import json
import logging
import math
import re
import shutil
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import (Body, FastAPI, File, HTTPException, Request, UploadFile,
                     WebSocket, WebSocketDisconnect)
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .config import Config
from .node import HostNode
from .paths import data_root, resource_root
from .transports import KINDS, available_kinds

LOGGER = logging.getLogger("host_ui")
REPO_ROOT = resource_root()
WEB_DIR = REPO_ROOT / "host_ui" / "web"
MAPS_DIR = data_root() / "host_ui" / "maps"
MAX_MAP_BYTES = 25 * 1024 * 1024

CONTROL_PERIOD_S = 0.04     # 25 Hz
BROADCAST_PERIOD_S = 0.05   # 20 Hz


def _json(payload: Any) -> str:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def _ok(message: str = "", **extra: Any) -> dict:
    return {"ok": True, "message": message, **extra}


def _fail(message: str, **extra: Any) -> dict:
    return {"ok": False, "message": message, **extra}


# --------------------------------------------------------------------------- 鉴权
def _is_loopback(host: str | None) -> bool:
    if not host:
        return False
    host = str(host).strip("[]")
    return host in ("127.0.0.1", "::1", "localhost", "testclient")


def _allowed_origins(config: Config) -> set[str]:
    host = str(config.get("web.host", "127.0.0.1"))
    port = int(config.get("web.port", 8710))
    origins = {
        f"http://127.0.0.1:{port}", f"http://localhost:{port}",
        f"http://[::1]:{port}",
    }
    if host not in ("127.0.0.1", "0.0.0.0", "::"):
        origins.add(f"http://{host}:{port}")
    for item in (config.get("web.allowed_origins") or []):
        origins.add(str(item).rstrip("/"))
    return origins


def _token_matches(given: str | None, expected: str) -> bool:
    if not expected:
        return True
    return bool(given) and hmac.compare_digest(str(given), str(expected))


def _rest_authorized(request: Request, config: Config) -> bool:
    token = str(config.get("web.token") or "")
    header = request.headers.get("x-auth-token") or request.headers.get(
        "authorization", "").removeprefix("Bearer ").strip()
    if token:
        return _token_matches(header, token)
    # 未配置令牌时只允许本机访问
    return _is_loopback(request.client.host if request.client else None)


def _ws_authorized(websocket: WebSocket, config: Config) -> tuple[bool, str]:
    token = str(config.get("web.token") or "")
    origin = websocket.headers.get("origin")
    if origin is not None:
        if origin.rstrip("/") not in _allowed_origins(config):
            return False, f"Origin 不被允许：{origin}"
    given = (websocket.query_params.get("token")
             or websocket.headers.get("x-auth-token") or "")
    if token:
        if not _token_matches(given, token):
            return False, "访问令牌缺失或错误"
        return True, ""
    host = websocket.client.host if websocket.client else None
    if not _is_loopback(host):
        return False, "未配置访问令牌，只允许本机连接"
    return True, ""


# --------------------------------------------------------------------------- 工具
class WebSocketHub:
    """单客户端总线，注册操作与有界发送分别加锁。"""

    SEND_TIMEOUT_S = 1.0

    def __init__(self, max_clients: int = 1, on_disconnect=None) -> None:
        self._clients: dict[int, WebSocket] = {}
        self._lock = asyncio.Lock()
        self._send_lock = asyncio.Lock()
        self.max_clients = 1
        self.on_disconnect = on_disconnect
        self.session_token = None

    async def _close(self, websocket, code=1000, reason=""):
        try:
            await asyncio.wait_for(websocket.close(code=code, reason=reason),
                                   timeout=self.SEND_TIMEOUT_S)
        except Exception:
            LOGGER.debug("关闭客户端失败", exc_info=True)

    async def add(self, websocket: WebSocket, on_replace=None) -> int:
        async with self._lock:
            previous = list(self._clients.values())
            if previous and on_replace is not None:
                on_replace()
            self._clients = {1: websocket}
            self.session_token = uuid.uuid4().hex
        for old in previous:
            await self._close(old, code=4001, reason="新的客户端已连接")
        return 1

    async def is_current(self, client_id: int, websocket: WebSocket) -> bool:
        # 注册表只在事件循环中修改，判断身份时不等待网络发送锁。
        return self._clients.get(client_id) is websocket

    async def remove(self, client_id: int, websocket: WebSocket, on_remove=None) -> bool:
        async with self._lock:
            if self._clients.get(client_id) is not websocket:
                return False
            self._clients.pop(client_id)
            self.session_token = None
            callback = on_remove or self.on_disconnect
            if callback is not None:
                callback()
            return True

    async def count(self) -> int:
        return len(self._clients)

    async def send(self, client_id, websocket, payloads) -> bool:
        async with self._send_lock:
            if not await self.is_current(client_id, websocket):
                return False

            async def send_batch():
                for payload in payloads:
                    if not await self.is_current(client_id, websocket):
                        return
                    text = payload if isinstance(payload, str) else _json(payload)
                    await websocket.send_text(text)

            try:
                await asyncio.wait_for(send_batch(), timeout=self.SEND_TIMEOUT_S)
                return await self.is_current(client_id, websocket)
            except Exception:
                # 先撤销会话并停车，关闭握手即使失败也不保留控制权。
                await self.remove(client_id, websocket)
        await self._close(websocket)
        return False

    async def broadcast(self, payloads: list[dict | str]) -> None:
        if not payloads:
            return
        for client_id, websocket in list(self._clients.items()):
            await self.send(client_id, websocket, payloads)


def _transform_scan(points, align: dict | None) -> list[list[float]]:
    """里程计坐标点集 -> 地图坐标（只做刚体变换）。"""
    if not align:
        return [[round(x, 3), round(y, 3)] for x, y in points]
    ax = float(align.get("x", 0.0))
    ay = float(align.get("y", 0.0))
    yaw = math.radians(float(align.get("yaw_deg", 0.0)))
    c, s = math.cos(yaw), math.sin(yaw)
    return [[round(ax + c * x - s * y, 3), round(ay + s * x + c * y, 3)]
            for x, y in points]


def _operator_gate(node: HostNode, client_id: int | None,
                   action: str | None = None) -> tuple[bool, str]:
    """除急停/停车外的一切运动相关操作都要通过操控权检查。

    只读接收模式下，观测类/配置类动作（限幅、地图对齐、录制）仍然允许，
    因为它们不会向机器人发送运动指令。
    """
    if not bool(node.config.get("safety.require_operator", True)):
        return True, ""
    if action in _READ_ONLY_SAFE_ACTIONS:
        try:
            if bool(node.transport.status().get("read_only")):
                return True, ""
        except Exception:
            pass
    if node.operator_id is None:
        return False, "尚未取得操控权：请先在页面点击「申请操控权」"
    if client_id is not None and node.operator_id != client_id:
        return False, f"操控权在客户端 {node.operator_id}，请先接管"
    return True, ""


#: 只读接收模式下仍可执行的"非运动"动作
_READ_ONLY_SAFE_ACTIONS = {"set_limits", "map_align", "record", "clear_trail"}


# --------------------------------------------------------------------------- 链路参数白名单
_HOST_PATTERN = re.compile(r"^[A-Za-z0-9._:\-]{1,64}$")
_PORT_KEYS = {"robot_port", "bind_port", "port", "ssh_port"}
_IP_KEYS = {"robot_ip", "bind_host", "source_ip", "host"}
_RECONNECT_KEYS = {"enabled", "min_delay_s", "max_delay_s"}


def _link_option_keys() -> dict[str, set[str]]:
    keys: dict[str, set[str]] = {}
    for entry in available_kinds():
        keys[str(entry.get("kind"))] = set((entry.get("options") or {}).keys())
    return keys


def sanitize_link_options(kind: str, options: dict | None) -> tuple[dict | None, str]:
    """校验并清洗 ``/api/link`` 的 options，拒绝未知参数与非法地址。

    目的：避免任意客户端让上位机向任意 IP/端口发 UDP，或把任意主机/用户名
    传给 SSH 客户端；只允许传输层默认参数表里的键。
    """
    kind = str(kind or "").strip().lower()
    if kind in ("vendor", "vendor_udp"):
        kind = "udp"
    elif kind == "ros1":
        kind = "ros"
    allowed = _link_option_keys().get(kind)
    if allowed is None:
        return None, f"未知链路类型：{kind}"
    if options is None:
        return {}, ""
    if not isinstance(options, dict):
        return None, "链路参数必须是对象"
    cleaned: dict = {}
    for key, value in options.items():
        key = str(key)
        if key == "reconnect":
            if not isinstance(value, dict):
                return None, "reconnect 必须是对象"
            reconnect = {}
            for inner_key, inner_value in value.items():
                inner_key = str(inner_key)
                if inner_key not in _RECONNECT_KEYS:
                    return None, f"reconnect 不支持参数：{inner_key}"
                if inner_key == "enabled":
                    reconnect[inner_key] = bool(inner_value)
                elif (isinstance(inner_value, (int, float))
                      and math.isfinite(float(inner_value))
                      and 0.1 <= float(inner_value) <= 120.0):
                    reconnect[inner_key] = float(inner_value)
                else:
                    return None, f"reconnect.{inner_key} 数值非法"
            cleaned[key] = reconnect
            continue
        if key not in allowed:
            return None, f"不支持的链路参数：{key}"
        if isinstance(value, bool):
            cleaned[key] = value
            continue
        if isinstance(value, (int, float)):
            number = float(value)
            if not math.isfinite(number) or abs(number) > 1e6:
                return None, f"参数 {key} 数值非法"
            if key in _PORT_KEYS and not 0 <= int(number) <= 65535:
                return None, f"参数 {key} 端口越界"
            cleaned[key] = int(number) if key in _PORT_KEYS else number
            continue
        if isinstance(value, str):
            text = value.strip()
            if len(text) > 256 or "\n" in text or "\r" in text or "\x00" in text:
                return None, f"参数 {key} 字符串非法"
            if key in _IP_KEYS and text:
                try:
                    ipaddress.ip_address(text)
                except ValueError:
                    # 形如 IPv4 但非法（如 256.1.1.1）直接拒绝，不当作主机名
                    if re.fullmatch(r"\d+(?:\.\d+){3}", text):
                        return None, f"参数 {key} 地址格式非法"
                    if not _HOST_PATTERN.fullmatch(text):
                        return None, f"参数 {key} 地址格式非法"
            cleaned[key] = text
            continue
        return None, f"参数 {key} 类型不支持"
    return cleaned, ""


def dispatch_client_message(node: HostNode, message: dict,
                            client_id: int | None = None) -> dict | None:
    """处理前端一条指令消息；返回需要回给该客户端的 ack（可选）。"""
    if not isinstance(message, dict):
        return _fail("消息格式错误")
    kind = str(message.get("type", "")).strip()
    try:
        # 控制权
        if kind == "claim_control":
            if node.operator_id is None:
                node.set_operator(client_id)
                return _ok("已取得操控权")
            if node.operator_id == client_id:
                return _ok("你已持有操控权")
            return _fail(f"操控权已被客户端 {node.operator_id} 持有，可申请接管")
        if kind == "takeover_control":
            previous = node.operator_id
            node.set_operator(client_id)
            node.add_event("warn", "operator_takeover",
                           f"操控权被客户端 {client_id} 接管（原为 {previous}）")
            return _ok("已接管操控权")
        if kind == "release_control":
            ok, text = node.release_operator(client_id)
            return _ok(text) if ok else _fail(text)
        if kind == "heartbeat":
            if node.operator_id == client_id:
                node.note_operator_activity()
            return None

        # 急停类：任何客户端都可以触发，但复位需要操控权与长按
        if kind == "estop":
            engaged = bool(message.get("engaged", True))
            if engaged:
                ok, text = node.engage_estop(str(message.get("reason") or "界面急停"))
            else:
                gate, why = _operator_gate(node, client_id)
                if not gate:
                    return _fail(why)
                ok, text = node.reset_estop(message.get("hold_s"))
            return _ok(text) if ok else _fail(text)
        if kind == "stop":
            ok, text = node.stop_all("操作者立即停车")
            return _ok(text)
        if kind == "estop_reset":
            gate, why = _operator_gate(node, client_id)
            if not gate:
                return _fail(why)
            ok, text = node.reset_estop(message.get("hold_s"))
            return _ok(text) if ok else _fail(text)

        # 运动/使能/姿态/导航/限幅：需要操控权（只读模式下限幅/录制等配置动作放行）
        if kind in ("teleop", "arm", "disarm", "posture", "posture_cancel",
                    "goal", "nav_cancel", "set_limits", "simulate", "record",
                    "exploration_start", "exploration_stop",
                    "mapping_start", "mapping_stop", "map_save"):
            gate, why = _operator_gate(node, client_id, action=kind)
            if not gate:
                return _fail(why)

        if kind in ("mapping_start", "mapping_stop"):
            ok, text = node.set_mapping(kind == "mapping_start")
            return _ok(text) if ok else _fail(text)
        if kind == "map_save":
            ok, text = node.save_map()
            return _ok("地图已保存", map_saved=text) if ok else _fail(text)
        if kind == "teleop":
            ok, text = node.set_teleop(
                float(message.get("linear", 0.0)),
                float(message.get("angular", 0.0)),
                bool(message.get("deadman", False)))
            return None if ok else _fail(text)
        if kind == "arm":
            ok, text = node.arm()
            return _ok(text) if ok else _fail(text)
        if kind == "disarm":
            ok, text = node.disarm()
            return _ok(text) if ok else _fail(text)
        if kind == "posture":
            ok, text = node.posture(str(message.get("action", "")))
            return _ok(text) if ok else _fail(text)
        if kind == "posture_cancel":
            ok, text = node.cancel_posture()
            return _ok(text) if ok else _fail(text)
        if kind == "set_limits":
            ok, text = node.set_limits(message)
            return _ok(text) if ok else _fail(text)
        if kind == "goal":
            yaw = message.get("yaw_deg")
            ok, text = node.set_goal(float(message.get("x", 0.0)),
                                     float(message.get("y", 0.0)),
                                     None if yaw is None else float(yaw))
            return _ok(text) if ok else _fail(text)
        if kind == "nav_cancel":
            ok, text = node.cancel_nav()
            return _ok(text)
        if kind == "exploration_start":
            ok, text = node.start_exploration()
            return _ok(text) if ok else _fail(text)
        if kind == "exploration_stop":
            ok, text = node.stop_exploration("操作者停止探索")
            return _ok(text) if ok else _fail(text)
        if kind == "clear_trail":
            node.clear_trail()
            return _ok("轨迹已清除")
        if kind == "map_align":
            node.set_map_align({
                "x": float(message.get("x", 0.0)),
                "y": float(message.get("y", 0.0)),
                "yaw_deg": float(message.get("yaw_deg", 0.0)),
            })
            node.config.save()
            return _ok("地图对齐已更新")
        if kind == "simulate":
            joystick = message.get("joystick")
            link = message.get("link")
            ok, text = node.simulate(joystick=joystick, link=link)
            return _ok(text) if ok else _fail(text)
        if kind == "record":
            action = str(message.get("action", ""))
            if action == "start":
                ok, text = node.start_recording()
            else:
                ok, text = node.stop_recording()
            return _ok(text) if ok else _fail(text)
        if kind == "ping":
            return {"type": "pong", "t": message.get("t"), "server_time": time.time()}
    except (TypeError, ValueError) as error:
        return _fail(f"参数错误：{error}")
    return _fail(f"未知消息类型：{kind}")


def create_app(config: Config | None = None, auto_connect: bool = True,
               web_dir: Path | None = None) -> FastAPI:
    config = config or Config()
    web_root = Path(web_dir) if web_dir else WEB_DIR

    host = str(config.get("web.host", "127.0.0.1"))
    token = str(config.get("web.token") or "")
    if host not in ("127.0.0.1", "::1", "localhost") and not token:
        raise RuntimeError(
            "绑定非回环地址时必须配置 web.token（或用 --token 启动），"
            "否则任何人都能通过网络驱动机器狗")

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        node = HostNode(config)
        app.state.node = node
        def client_disconnected():
            node.clients = 0
            node.release_operator()
            node.stop_all("客户端断开，零速")

        app.state.hub = WebSocketHub(on_disconnect=client_disconnected)
        app.state.tasks = []
        app.state.command_tasks = set()
        app.state.stop_event = asyncio.Event()

        if auto_connect:
            ok, message = node.connect()
            if not ok:
                LOGGER.warning("自动连接失败: %s", message)

        async def control_loop() -> None:
            next_run = time.monotonic()
            while not app.state.stop_event.is_set():
                try:
                    node.tick()
                except Exception:
                    LOGGER.exception("控制循环异常")
                next_run += CONTROL_PERIOD_S
                delay = next_run - time.monotonic()
                if delay > 0:
                    await asyncio.sleep(delay)
                else:
                    next_run = time.monotonic()
                    await asyncio.sleep(0)

        async def broadcast_loop() -> None:
            last_scan_sent = 0.0
            last_nav_sent = 0.0
            last_nav_key = ""
            last_map_revision = -1
            while not app.state.stop_event.is_set():
                try:
                    node.broadcast_rate.tick()
                    node.clients = await app.state.hub.count()
                    payloads = [{"type": "state", "data": node.snapshot()}]

                    if node.map_revision != last_map_revision:
                        last_map_revision = node.map_revision
                        message = node.map_message_json()
                        if message is not None:
                            payloads.append(message)

                    trail = node.trail.pending()
                    if trail:
                        payloads.append({"type": "trail",
                                         "data": {"pts": [[round(x, 3), round(y, 3)]
                                                          for x, y in trail]}})

                    for event in node.events.drain():
                        payloads.append({"type": "event", "data": event})

                    now = time.monotonic()
                    with node._lock:
                        scan = list(node.scan)
                        scan_stamp = node.scan_stamp
                    if (scan and scan_stamp > last_scan_sent
                            and now - scan_stamp < 2.0 and now - last_scan_sent > 0.15):
                        payloads.append({"type": "scan", "data": {
                            "pts": _transform_scan(scan, node.config.get("map.align"))}})
                        last_scan_sent = now

                    nav = node.nav.snapshot().as_dict()
                    nav_key = f"{nav['state']}:{json.dumps(nav['goal'])}:{len(nav['path'])}"
                    if nav_key != last_nav_key or (
                            nav["active"] and now - last_nav_sent > 0.5):
                        payloads.append({"type": "nav", "data": nav})
                        last_nav_key = nav_key
                        last_nav_sent = now

                    await app.state.hub.broadcast(payloads)
                except Exception:
                    LOGGER.exception("广播循环异常")
                await asyncio.sleep(BROADCAST_PERIOD_S)

        app.state.tasks = [asyncio.create_task(control_loop()),
                           asyncio.create_task(broadcast_loop())]
        try:
            yield
        finally:
            app.state.stop_event.set()
            for task in app.state.tasks + list(app.state.command_tasks):
                task.cancel()
            await asyncio.gather(*app.state.tasks, *app.state.command_tasks,
                                 return_exceptions=True)
            node.shutdown()

    app = FastAPI(title="Lite2 机器狗上位机", version="1.4.0", lifespan=lifespan)

    @app.middleware("http")
    async def auth_middleware(request: Request, call_next):
        if request.url.path.startswith("/api/") and not _rest_authorized(request, config):
            return JSONResponse(status_code=403, content={
                "ok": False,
                "message": "未授权：请携带 X-Auth-Token，或从本机访问",
            })
        if request.method == "POST" and request.url.path.startswith("/api/"):
            # 急停和停车可由失联页面触发，其余写请求绑定当前浏览器会话。
            emergency = request.url.path == "/api/stop"
            if request.url.path == "/api/estop":
                try:
                    emergency = bool((await request.json()).get("engaged", True))
                except (ValueError, AttributeError):
                    emergency = False
            if not emergency:
                current = app.state.hub.session_token
                supplied = request.headers.get("x-control-session")
                if (current or supplied) and (not current or not supplied
                                              or not hmac.compare_digest(current, supplied)):
                    return JSONResponse(status_code=409, content={
                        "ok": False, "message": "客户端会话已失效，请使用当前连接页面操作"})
        return await call_next(request)

    # ------------------------------------------------------------------ 页面与静态资源
    @app.get("/")
    def index() -> FileResponse:
        index_path = web_root / "index.html"
        if not index_path.exists():
            raise HTTPException(status_code=500, detail="前端资源缺失：web/index.html")
        return FileResponse(index_path)

    if web_root.exists():
        app.mount("/static", StaticFiles(directory=str(web_root)), name="static")

    # ------------------------------------------------------------------ 状态与配置
    @app.get("/api/status")
    def api_status():
        return app.state.node.snapshot()

    @app.get("/api/health")
    def api_health():
        node: HostNode = app.state.node
        snapshot = node.snapshot()
        return {
            "version": snapshot["version"],
            "uptime_s": snapshot["uptime_s"],
            "link": {"kind": snapshot["link"].get("kind"),
                     "connected": snapshot["link"].get("connected"),
                     "detail": snapshot["link"].get("detail"),
                     "reconnect": snapshot["link"].get("reconnect")},
            "health": snapshot["health"],
            "battery": snapshot["battery"],
            "execution": snapshot["execution"],
            "operator": snapshot["operator"],
            "capabilities": snapshot["capabilities"],
            "recording": snapshot["recording"],
        }

    @app.get("/api/sensors")
    def api_sensors():
        transport = app.state.node.transport
        snapshot = getattr(transport, "sensor_snapshot", None)
        return snapshot() if snapshot else {}

    @app.get("/api/config")
    def api_config():
        node: HostNode = app.state.node
        return {
            "config": node.config.public(),
            "kinds": list(available_kinds()),
            "map": node.map_describe(),
            "recording": node.recorder.status(),
            "capabilities": dict(getattr(node.transport, "capabilities", {}) or {}),
        }

    @app.post("/api/config")
    def api_save_config(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        if not isinstance(payload, dict):
            return _fail("配置格式错误")
        warnings = node.config.update(payload)
        node.safety.update_config(node.config.get("limits"), node.config.get("safety"))
        node.nav.update_config(node.config.get("nav"), node.config.get("limits"))
        if node.config.path:
            try:
                node.config.save()
            except Exception as error:
                return _fail(f"配置保存失败：{error}")
        node.add_event("info", "config", "配置已更新")
        return _ok("配置已更新", warnings=warnings)

    @app.get("/api/kinds")
    def api_kinds():
        return list(available_kinds())

    # ------------------------------------------------------------------ 链路
    @app.post("/api/link")
    def api_link(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        kind = str(payload.get("kind") or node.config.get("link.kind", "mock"))
        options, error = sanitize_link_options(kind, payload.get("options") or {})
        if options is None:
            return _fail(f"链路参数被拒绝：{error}")
        ok, message = node.connect(kind, options)
        return _ok(message) if ok else _fail(message)

    @app.post("/api/link/disconnect")
    def api_link_disconnect():
        node: HostNode = app.state.node
        node.disconnect(reason="界面断开")
        return _ok("链路已断开")

    # ------------------------------------------------------------------ 操作权
    @app.get("/api/operator")
    def api_operator():
        node: HostNode = app.state.node
        return node.operator_snapshot()

    @app.post("/api/operator")
    def api_operator_claim(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        action = str(payload.get("action") or "claim")
        if action in ("claim", "takeover"):
            node.set_operator(None)
            return _ok("已释放旧操控权；请在页面重新申请（操控权绑定浏览器连接）")
        if action == "release":
            ok, message = node.release_operator()
            return _ok(message) if ok else _fail(message)
        return _fail("动作必须是 claim / release / takeover")

    # ------------------------------------------------------------------ 安全
    @app.post("/api/posture")
    def api_posture(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        ok, message = node.posture(str(payload.get("action", "")))
        return _ok(message) if ok else _fail(message)

    @app.post("/api/posture/cancel")
    def api_posture_cancel():
        node: HostNode = app.state.node
        ok, message = node.cancel_posture()
        return _ok(message) if ok else _fail(message)

    @app.post("/api/arm")
    def api_arm():
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        ok, message = node.arm()
        return _ok(message) if ok else _fail(message)

    @app.post("/api/disarm")
    def api_disarm():
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        ok, message = node.disarm()
        return _ok(message) if ok else _fail(message)

    @app.post("/api/estop")
    def api_estop(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        engaged = bool(payload.get("engaged", True))
        if engaged:
            ok, message = node.engage_estop(str(payload.get("reason") or "界面急停"))
        else:
            ok, message = node.reset_estop(payload.get("hold_s"))
        return _ok(message) if ok else _fail(message)

    @app.post("/api/estop/reset")
    def api_estop_reset(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        ok, message = node.reset_estop(payload.get("hold_s"))
        return _ok(message) if ok else _fail(message)

    @app.post("/api/teleop")
    def api_teleop(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        ok, message = node.set_teleop(float(payload.get("linear", 0.0)),
                                      float(payload.get("angular", 0.0)),
                                      bool(payload.get("deadman", False)))
        return _ok(message) if ok else _fail(message)

    @app.post("/api/stop")
    def api_stop():
        node: HostNode = app.state.node
        ok, message = node.stop_all()
        return _ok(message)

    @app.post("/api/limits")
    def api_limits(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None, action="set_limits")
        if not gate:
            return _fail(why)
        ok, message = node.set_limits(payload)
        if ok:
            try:
                node.config.save()
            except Exception:
                pass
        return _ok(message) if ok else _fail(message)

    # ------------------------------------------------------------------ 导航
    @app.post("/api/goal")
    def api_goal(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        yaw = payload.get("yaw_deg")
        ok, message = node.set_goal(float(payload.get("x", 0.0)),
                                    float(payload.get("y", 0.0)),
                                    None if yaw is None else float(yaw))
        return _ok(message) if ok else _fail(message)

    @app.post("/api/nav/cancel")
    def api_nav_cancel():
        node: HostNode = app.state.node
        ok, message = node.cancel_nav()
        return _ok(message) if ok else _fail(message)

    @app.get("/api/nav/path")
    def api_nav_path():
        return {"path": app.state.node.plan_preview()}

    # ------------------------------------------------------------------ 自主探索
    @app.post("/api/explore/start")
    def api_explore_start():
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        ok, message = node.start_exploration()
        return _ok(message) if ok else _fail(message)

    @app.post("/api/explore/stop")
    def api_explore_stop():
        node: HostNode = app.state.node
        gate, why = _operator_gate(node, None)
        if not gate:
            return _fail(why)
        ok, message = node.stop_exploration("操作者停止探索")
        return _ok(message) if ok else _fail(message)

    # ------------------------------------------------------------------ 地图
    @app.post("/api/map/upload")
    async def api_map_upload(file: UploadFile = File(...)):
        node: HostNode = app.state.node
        name = Path(file.filename or "map.json").name
        suffix = Path(name).suffix.lower()
        if suffix not in (".json", ".yaml", ".yml", ".pgm"):
            return _fail("只支持 .json / .yaml / .yml / .pgm 地图文件")
        MAPS_DIR.mkdir(parents=True, exist_ok=True)
        target = MAPS_DIR / f"{uuid.uuid4().hex[:12]}_{name}"
        total = 0
        try:
            with target.open("wb") as handle:
                while True:
                    chunk = await file.read(1024 * 1024)
                    if not chunk:
                        break
                    total += len(chunk)
                    if total > MAX_MAP_BYTES:
                        handle.close()
                        target.unlink(missing_ok=True)
                        return _fail(f"地图文件超过 {MAX_MAP_BYTES // (1024 * 1024)} MB 上限")
                    handle.write(chunk)
        finally:
            await file.close()
        ok, message = node.load_map_file(str(target))
        return _ok(message, path=str(target)) if ok else _fail(message)

    @app.post("/api/map/demo")
    def api_map_demo():
        node: HostNode = app.state.node
        try:
            from .mapstore import demo_map

            grid = demo_map()
        except Exception as error:
            return _fail(f"示例地图不可用：{error}")
        node.load_map(grid, source="示例地图")
        return _ok("已切换到示例地图")

    @app.get("/api/maps")
    def api_maps():
        """列出 host_ui/maps 下可直接加载的本机/机载地图。"""
        items: list[dict] = []
        if MAPS_DIR.exists():
            for directory in sorted(MAPS_DIR.iterdir()):
                if not directory.is_dir() or directory.name.startswith("."):
                    continue
                yaml_file = directory / "map.yaml"
                if yaml_file.exists():
                    items.append({"name": directory.name, "path": str(yaml_file),
                                  "kind": "ros"})
                for json_file in sorted(directory.glob("*.json")):
                    if json_file.name.startswith("verification"):
                        continue
                    items.append({"name": f"{directory.name}/{json_file.name}",
                                  "path": str(json_file), "kind": "json"})
        return {"root": str(MAPS_DIR), "maps": items}

    @app.post("/api/map/load")
    def api_map_load(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        raw = str(payload.get("path") or "").strip()
        if not raw:
            return _fail("缺少地图路径")
        root = MAPS_DIR.resolve()
        candidate = Path(raw)
        target = (candidate if candidate.is_absolute() else MAPS_DIR / candidate)
        try:
            target = target.resolve()
        except OSError as error:
            return _fail(f"地图路径无效：{error}")
        if target != root and root not in target.parents:
            return _fail("只允许加载 host_ui/maps 目录下的地图")
        if not target.is_file():
            return _fail(f"地图文件不存在：{target.name}")
        ok, message = node.load_map_file(str(target))
        return _ok(message, path=str(target)) if ok else _fail(message)

    @app.post("/api/map/align")
    def api_map_align(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        node.set_map_align({
            "x": float(payload.get("x", 0.0)),
            "y": float(payload.get("y", 0.0)),
            "yaw_deg": float(payload.get("yaw_deg", 0.0)),
        })
        try:
            node.config.save()
        except Exception:
            pass
        return _ok("地图对齐已更新")

    # ------------------------------------------------------------------ 录制
    @app.post("/api/record/start")
    def api_record_start():
        node: HostNode = app.state.node
        ok, message = node.start_recording()
        return _ok(message) if ok else _fail(message)

    @app.post("/api/record/stop")
    def api_record_stop():
        node: HostNode = app.state.node
        ok, message = node.stop_recording()
        return _ok(message) if ok else _fail(message)

    @app.get("/api/record/download")
    def api_record_download():
        node: HostNode = app.state.node
        try:
            archive = node.recorder.archive_latest()
        except Exception as error:
            return _fail(f"打包失败：{error}")
        if archive is None or not archive.exists():
            return _fail("还没有录制数据")
        return FileResponse(archive, filename=archive.name)

    # ------------------------------------------------------------------ 仿真调试
    @app.post("/api/simulate")
    def api_simulate(payload: dict = Body(default={})):
        node: HostNode = app.state.node
        ok, message = node.simulate(joystick=payload.get("joystick"),
                                    link=payload.get("link"))
        return _ok(message) if ok else _fail(message)

    # ------------------------------------------------------------------ WebSocket
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        node: HostNode = app.state.node
        allowed, reason = _ws_authorized(websocket, config)
        if not allowed:
            await websocket.close(code=1008, reason=reason)
            return
        await websocket.accept()

        def stop_previous_session():
            node.release_operator()
            node.stop_all("客户端切换，零速")

        try:
            client_id = await app.state.hub.add(websocket, stop_previous_session)
        except RuntimeError as error:
            await websocket.close(code=1013, reason=str(error))
            return
        node.clients = await app.state.hub.count()
        session = app.state.hub.session_token
        pending_commands = {}
        blocking_commands = {"arm", "disarm", "estop", "estop_reset",
                             "mapping_start", "mapping_stop", "map_save"}

        async def reply_to(message, reply):
            if reply is not None:
                reply.setdefault("type", "ack")
                if "id" in message:
                    reply["id"] = message["id"]
                return await app.state.hub.send(client_id, websocket, [reply])
            return True

        async def run_command(message, lane):
            def execute():
                if app.state.hub.session_token != session:
                    return _fail("客户端会话已失效")
                return dispatch_client_message(node, message, client_id)

            try:
                reply = await asyncio.to_thread(execute)
                await reply_to(message, reply)
            except Exception:
                LOGGER.exception("客户端指令执行异常")
            finally:
                pending_commands.pop(lane, None)

        try:
            hello = node.hello()
            hello["client_id"] = client_id
            hello["client_session"] = app.state.hub.session_token
            hello["version"] = node.version
            hello["capabilities"] = dict(getattr(node.transport, "capabilities", {}) or {})
            if not await app.state.hub.send(client_id, websocket,
                                            [{"type": "hello", "data": hello}]):
                return
            while True:
                text = await websocket.receive_text()
                if len(text) > int(config.get("web.max_message_bytes", 65536)):
                    await reply_to({}, _fail("消息过长，已忽略"))
                    continue
                try:
                    message = json.loads(text)
                except json.JSONDecodeError:
                    continue
                if not await app.state.hub.is_current(client_id, websocket):
                    break
                if not isinstance(message, dict):
                    await reply_to({}, _fail("消息格式错误"))
                    continue
                kind = str(message.get("type", ""))
                if kind in blocking_commands:
                    lane = "estop" if kind == "estop" else "control"
                    if lane in pending_commands:
                        await reply_to(message, _fail("已有指令等待机载确认，请稍后重试"))
                        continue
                    task = asyncio.create_task(run_command(message, lane))
                    pending_commands[lane] = task
                    app.state.command_tasks.add(task)
                    task.add_done_callback(app.state.command_tasks.discard)
                    continue
                reply = dispatch_client_message(node, message, client_id)
                if not await reply_to(message, reply):
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            LOGGER.debug("WebSocket 连接异常", exc_info=True)
        finally:
            def cleanup_session():
                node.clients = 0
                if node.operator_id == client_id:
                    node.release_operator(client_id)
                    node.add_event("warn", "operator_lost",
                                   f"操控者客户端 {client_id} 断开，操控权已释放")
                node.stop_all("所有客户端断开，零速")

            await app.state.hub.remove(client_id, websocket, cleanup_session)

    @app.exception_handler(Exception)
    async def unhandled(request: Request, exc: Exception):  # pragma: no cover - 兜底
        LOGGER.exception("未处理异常")
        return JSONResponse(status_code=500,
                            content={"ok": False, "message": "服务内部错误"})

    return app

