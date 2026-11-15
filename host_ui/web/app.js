/* ==========================================================================
   Lite2 实机调试上位机 · 前端主逻辑
   数据流：WebSocket 20Hz 状态 -> DOM 面板 / 地图 / 曲线
           键盘与方向键 -> 20Hz 心跳速度指令（按住才发，松开立即零速）
   ========================================================================== */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const HOLD_RESET_MS = 1200;
  const ESTOP_CONFIRM_MS = 1500;
  const TOKEN_KEY = "host_ui_token";
  const WS_CONNECT_TIMEOUT_MS = 10000;
  const WS_STATE_TIMEOUT_MS = 5000;
  const WS_MAX_BUFFERED_BYTES = 64 * 1024;

  // 契约 4.1.2：execution.reason 中文映射（后端字段缺失时显示原始值）
  const REASON_TEXT = {
    disarmed: "未使能",
    stale_depth: "深度心跳失效（检查相机/深度驱动）",
    mode_unconfirmed: "自动模式未确认",
    remote_takeover: "遥控器接管",
    motion_mode_not_ready: "未站立",
    stale_state: "状态反馈超时",
    stale_command_feedback: "指令反馈超时",
    vendor_error: "厂商错误",
    body_tilt: "机身倾角过大",
    disabled: "未使能",
    no_operator: "无操控权",
    posture_busy: "姿态切换中",
    estop: "急停锁存",
  };
  const ARM_STATE_TEXT = { armed: "已使能", disarmed: "未使能", pending: "请求中…", unknown: "未知" };
  const MOTION_TEXT = { 0: "基本运动", 1: "步态", 2: "跳舞" };
  const BASIC_TEXT = {
    1: "趴下", 4: "准备起立", 5: "起立中", 6: "力控站立(可运动)", 7: "坐下中",
    8: "失控保护", 9: "自主爬起", 11: "翻倒", 17: "回零", 18: "后空翻", 19: "打招呼",
  };
  const BASIC_FAULT = new Set([8, 9, 11]);
  const EVENT_CATEGORY = {
    estop: "safety", estop_reset: "safety", estop_confirm: "safety", vendor_estop: "safety",
    operator: "operator", takeover: "operator", claim: "operator", release: "operator",
    operator_claim: "operator", operator_takeover: "operator", operator_release: "operator",
    arm: "control", disarm: "control", posture: "control", posture_cancel: "control",
    link_down: "link", link_up: "link", ssh_ros_reconnected: "link",
    reconnect: "link", link_error: "link",
    exploration_start: "control", exploration_paused: "control",
    exploration_goal: "control", exploration_complete: "control",
    exploration_cancel: "control", exploration_failed: "control",
    exploration_error: "control",
  };
  const EVENT_EVIDENCE = new Set([
    "estop", "estop_reset", "vendor_estop", "operator", "takeover", "claim", "release",
    "operator_claim", "operator_takeover", "operator_release",
    "arm", "disarm", "posture", "posture_cancel", "link_up", "link_down",
    "ssh_ros_reconnected", "reconnect",
  ]);

  const SOURCE_TEXT = { teleop: "手动", nav: "导航", idle: "待命" };
  const ZERO_TEXT = {
    "": "正常输出",
    no_input: "待命",
    deadman: "未按住使能",
    estop: "急停锁存",
    state_timeout: "状态反馈超时",
    link_down: "链路断开",
    mode_not_ready: "模式未就绪",
    joystick_takeover: "厂商遥控接管",
  };
  const NAV_TEXT = {
    idle: "导航待命",
    following: "导航中",
    aligning: "终点对齐",
    arrived: "已到达",
    cancelled: "已取消",
    stuck: "导航卡死",
  };
  // 契约 §4：自主探索状态文案与配色（沿用既有 badge ok/warn/soft/danger，
  // 契约中的「err」语义在本项目即 .badge.danger）
  const EXPLORATION_TEXT = {
    idle: "探索待命",
    active: "探索中",
    paused: "探索暂停",
    completed: "探索完成",
    cancelled: "已停止",
    failed: "探索失败",
  };
  const EXPLORATION_BADGE = {
    idle: "soft",
    active: "ok",
    paused: "warn",
    completed: "ok",
    cancelled: "soft",
    failed: "danger",
  };
  const EXPLORATION_EVENT_TOASTS = new Set([
    "exploration_start", "exploration_paused", "exploration_complete",
    "exploration_cancel", "exploration_failed", "exploration_error",
  ]);
  const KIND_LABEL = { mock: "离线仿真", udp: "厂商 UDP", ros: "ROS1", ssh_ros: "机载 ROS" };
  const OPTION_LABEL = {
    host: "感知主机 IP",
    username: "SSH 用户",
    source_ip: "本机机器人网卡 IP",
    robot_ip: "机器人 IP",
    robot_port: "机器人端口",
    bind_host: "本机监听地址",
    bind_port: "本机接收端口",
    state_topic: "状态话题",
    cmd_topic: "速度话题",
    map_topic: "地图话题",
    scan_topic: "雷达话题",
    odom_topic: "里程计话题",
    ready_basic: "可运动模式码",
    joystick_threshold: "摇杆接管阈值",
    zero_timeout_s: "零速看门狗 s",
    state_timeout_s: "状态超时 s",
    start_x: "起点 X",
    start_y: "起点 Y",
    start_yaw_deg: "起点朝向 °",
    noise_m: "噪声 m",
  };

  const state = {
    ws: null,
    connected: false,
    reconnectTimer: null,
    reconnectDelay: 800,
    hello: null,
    snapshot: null,
    nav: null,
    kinds: [],
    limits: null,
    config: null,
    lastPose: null,
    lastPoseTime: 0,
    yawRate: 0,
    keys: new Set(),
    pad: new Set(),
    movingSent: false,
    pendingGoal: null,
    goalPick: true,
    logCount: 0,
    chart: { cmdV: [], measV: [], yaw: [] },
    estop: false,
    simJoystick: false,
    simLink: false,
    resetInterval: null,
    resetStart: 0,
    limitTimer: null,
    pingTimer: null,
    rtt: null,
    lastStateAt: 0,
    socketStartedAt: 0,
    lastFundamentalMapKey: "",
    explorationLastPhase: "",
    // ---- 批次 1 新增 ----
    token: "",
    clientId: null,
    sessionToken: null,
    contractMode: false,      // 后端是否已提供 hello.client_id / state.operator
    operator: null,           // {id, since, clients}
    operatorTestOverride: null,
    wsCommandQueue: [],
    commandSequence: 0,
    wsUnsupported: new Set(), // 已确认后端不支持的消息类型
    noHeartbeat: false,
    estopPendingAt: 0,
    estopConfirmTimer: null,
    estopUnconfirmed: false,
    posturePending: false,
    armPending: false,
    confirmAction: null,
    mapCells: null,
    mapShape: null,
    logFilter: "all",
    lastContractMissingWarn: 0,
  };

  const mapView = new MapView($("map-canvas"));
  mapView.onPick = (world) => {
    if (!state.goalPick || !world) return;
    if (!canControl()) {
      toast("只读模式：没有操控权，无法设置导航目标", "warn");
      return;
    }
    const rejected = validateGoal(world);
    if (rejected) {
      toast(rejected, "error");
      return;
    }
    state.pendingGoal = { x: world.x, y: world.y };
    mapView.setGoal({ x: world.x, y: world.y, yaw_deg: null });
    updateGoalUi();
  };
  mapView.onCursor = (world) => {
    $("cursor-xy").textContent = world
      ? `${world.x.toFixed(2)} / ${world.y.toFixed(2)}`
      : "-- / --";
  };

  /* ------------------------------------------------------------------ 工具 */
  const fmt = (value, digits = 2) =>
    (value === null || value === undefined || Number.isNaN(Number(value)))
      ? "--" : Number(value).toFixed(digits);

  function setText(id, text) {
    const node = $(id);
    if (node && node.textContent !== String(text)) node.textContent = String(text);
  }

  function toggle(node, className, force) {
    if (!node) return;
    if (force === undefined) node.classList.toggle(className);
    else node.classList.toggle(className, !!force);
  }

  let toastTimer = null;
  // 模块级标志：exploration_paused 每个暂停回合只 toast 一次，避免事件刷屏
  let explorationPauseToastShown = false;
  function toast(message, level = "ok") {
    const node = $("toast");
    if (!node || !message) return;
    node.textContent = message;
    node.className = `toast ${level} show`;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => node.classList.remove("show"), 3600);
  }

  /* ------------------------------------------------------------------ 令牌 */
  function readTokenFromUrl() {
    try {
      const url = new URL(location.href);
      const token = url.searchParams.get("token");
      if (!token) return "";
      url.searchParams.delete("token");
      const search = url.searchParams.toString();
      history.replaceState(null, "", `${url.pathname}${search ? `?${search}` : ""}${url.hash}`);
      return token;
    } catch (error) {
      return "";
    }
  }

  function loadStoredToken() {
    try { return localStorage.getItem(TOKEN_KEY) || ""; } catch (error) { return ""; }
  }

  function persistToken(token) {
    state.token = token || "";
    try {
      if (state.token) localStorage.setItem(TOKEN_KEY, state.token);
      else localStorage.removeItem(TOKEN_KEY);
    } catch (error) { /* 隐私模式忽略 */ }
    renderTokenUi();
  }

  function renderTokenUi() {
    const input = $("token-input");
    if (input && document.activeElement !== input) input.value = state.token;
    const badge = $("token-badge");
    if (badge) {
      badge.textContent = state.token ? "已设置" : "未设置";
      badge.className = `badge ${state.token ? "ok" : "soft"}`;
    }
  }

  function authHeaders(extra) {
    const headers = Object.assign({}, extra || {});
    if (state.token) headers["X-Auth-Token"] = state.token;
    if (state.sessionToken) headers["X-Control-Session"] = state.sessionToken;
    if (state.clientId !== null && state.clientId !== undefined) {
      headers["X-Client-Id"] = String(state.clientId);
    }
    return headers;
  }

  async function fetchJson(path, payload) {
    try {
      const response = await fetch(path, {
        method: "POST",
        headers: authHeaders({ "Content-Type": "application/json" }),
        body: JSON.stringify(payload || {}),
      });
      let data;
      try { data = await response.json(); } catch (error) { data = null; }
      if (!data) {
        const text = response.status === 404
          ? `后端接口未就绪：${path}（等待后端联调）`
          : `请求失败（HTTP ${response.status}）`;
        toast(text, "warn");
        return { ok: false, message: text };
      }
      if (!data.ok) toast(data.message || "操作失败", "error");
      else if (data.message) toast(data.message, "ok");
      return data;
    } catch (error) {
      toast(`请求失败：${error}`, "error");
      return { ok: false, message: String(error) };
    }
  }

  function wsSend(message) {
    const socket = state.ws;
    if (!state.connected || !socket || socket.readyState !== WebSocket.OPEN) return false;
    if (socket.bufferedAmount > WS_MAX_BUFFERED_BYTES) {
      reconnectStalledSocket(socket, "发送队列拥塞，正在重连");
      return false;
    }
    try {
      socket.send(JSON.stringify(message));
      return true;
    } catch (error) {
      reconnectStalledSocket(socket, "发送失败，正在重连");
      return false;
    }
  }

  const REST_FALLBACK = {
    arm: { path: "/api/arm", body: {} },
    disarm: { path: "/api/disarm", body: {} },
    posture: { path: "/api/posture", body: { action: "stand" } },
    posture_cancel: { path: "/api/posture/cancel", body: {} },
    claim_control: { path: "/api/operator", body: { action: "claim" } },
    takeover_control: { path: "/api/operator", body: { action: "takeover" } },
    release_control: { path: "/api/operator", body: { action: "release" } },
    estop_reset: { path: "/api/estop/reset", body: {} },
  };

  function wsCommand(type, payload) {
    const id = `${Date.now()}-${++state.commandSequence}`;
    const message = Object.assign({ type, id }, payload || {});
    if (!wsSend(message)) {
      const rest = REST_FALLBACK[type];
      if (rest) {
        toast("WebSocket 未连接，改用 REST 提交…", "warn");
        restFallback(type, payload);
      } else {
        toast("与上位机服务断开，指令未发送", "error");
      }
      return false;
    }
    state.wsCommandQueue.push({ id, type, payload: payload || {}, at: Date.now() });
    if (type === "posture") state.postureCommandId = id;
    while (state.wsCommandQueue.length > 8) state.wsCommandQueue.shift();
    return true;
  }

  function restFallback(type, payload) {
    const rest = REST_FALLBACK[type];
    if (!rest) return;
    const body = Object.assign({}, rest.body, payload || {});
    if (type === "posture") body.action = (payload && payload.action) || "stand";
    if (type === "estop_reset" && payload && payload.hold_s) body.hold_s = payload.hold_s;
    return fetchJson(rest.path, body);
  }

  function onAck(message) {
    const text = String(message.message || "");
    const queuedIndex = message.id === undefined ? -1
      : state.wsCommandQueue.findIndex((entry) => entry.id === message.id);
    const queued = queuedIndex < 0 ? null : state.wsCommandQueue.splice(queuedIndex, 1)[0];
    if (message.ok === false && queued?.type === "posture"
        && queued.id === state.postureCommandId) {
      state.posturePending = false;
      state.postureIntent = null;
      setText("posture-result", text || "姿态请求未被接受");
    }
    if (message.ok !== false && message.map_saved) refreshMapList();
    if (message.ok === false && text.startsWith("未知消息类型")) {
      // 后端尚未实现契约消息：退化为 REST，或给出明确提示（不刷屏、不报错）
      const entry = queued || state.wsCommandQueue.shift();
      const type = entry ? entry.type : text.split("：").pop().trim();
      state.wsUnsupported.add(type);
      if (type === "heartbeat") {
        state.noHeartbeat = true;
        return;
      }
      if (entry && REST_FALLBACK[type]) {
        toast(`WS 指令 ${type} 未支持，改用 REST 提交`, "warn");
        restFallback(type, entry.payload);
      } else {
        toast(`后端接口未就绪：${type}（等待 Lead 完成后端联调）`, "warn");
      }
      return;
    }
    if (message.ok === false) toast(text || "操作失败", "error");
    else if (text) toast(text, "ok");
    if (state.lastCommandType && message.ok !== false) state.lastCommandType = null;
  }

  /* ------------------------------------------------------------------ WebSocket */
  function wsUrl() {
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    const query = state.token ? `?token=${encodeURIComponent(state.token)}` : "";
    return `${protocol}://${location.host}/ws${query}`;
  }

  function resetConnectionState() {
    state.connected = false;
    state.movingSent = false;
    state.keys.clear();
    state.pad.clear();
    state.clientId = null;
    state.sessionToken = null;
    state.operator = null;
    state.wsCommandQueue.length = 0;
    state.posturePending = false;
    state.armPending = false;
    document.querySelectorAll(".dpad-btn.active").forEach((node) => node.classList.remove("active"));
    stopPing();
    toggle($("conn-overlay"), "hidden", false);
  }

  function scheduleReconnect() {
    clearTimeout(state.reconnectTimer);
    state.reconnectTimer = setTimeout(connectWs, state.reconnectDelay);
    state.reconnectDelay = Math.min(5000, state.reconnectDelay * 1.6);
  }

  function reconnectStalledSocket(socket, reason) {
    if (state.ws !== socket) return;
    state.ws = null;
    resetConnectionState();
    setText("link-pill-text", reason);
    socket.onclose = null;
    try { socket.close(1000, reason); } catch (error) { /* 重试由定时器处理 */ }
    scheduleReconnect();
  }

  function connectWs() {
    clearTimeout(state.reconnectTimer);
    resetConnectionState();
    state.lastStateAt = 0;
    state.socketStartedAt = Date.now();
    if (state.ws && (state.ws.readyState === WebSocket.OPEN || state.ws.readyState === WebSocket.CONNECTING)) {
      try { state.ws.onclose = null; state.ws.close(); } catch (error) { /* ignore */ }
    }
    const socket = new WebSocket(wsUrl());
    state.ws = socket;

    socket.onopen = () => {
      if (state.ws !== socket) return;
      state.connected = true;
      state.socketStartedAt = Date.now();
      state.reconnectDelay = 800;
      toggle($("conn-overlay"), "hidden", true);
      setText("link-pill-text", "服务已连接");
      startPing();
    };

    socket.onclose = (event) => {
      if (state.ws !== socket) return;
      state.ws = null;
      resetConnectionState();
      if (event && event.code === 1008) {
        // 服务端策略拒绝：令牌错误 / Origin 不被允许
        toast("服务拒绝连接：访问令牌缺失或错误（或 Origin 不被允许）。请在「设备设置 → 访问令牌」中填写正确令牌。", "error");
        state.reconnectDelay = 5000;
      } else if (event && event.code === 1006) {
        // 握手失败时浏览器只能给 1006，用 REST 探测是否为令牌/Origin 拒绝
        probeAuthFailure();
      }
      if (event && event.reason && event.code !== 1008 && event.code !== 1006) {
        setText("link-pill-text", `服务已断开：${event.reason}`);
      }
      if (event && event.code === 4001) {
        setText("link-pill-text", "连接已由新的客户端接管");
        return;
      }
      scheduleReconnect();
    };

    socket.onerror = () => { /* onclose 会接手重连 */ };

    socket.onmessage = (event) => {
      if (state.ws !== socket) return;
      let message;
      try { message = JSON.parse(event.data); } catch (error) { return; }
      handleMessage(message);
    };
  }

  async function probeAuthFailure() {
    if (Date.now() - (state.lastAuthProbe || 0) < 8000) return;
    state.lastAuthProbe = Date.now();
    try {
      const response = await fetch("/api/status", { headers: authHeaders() });
      if (response.status === 401 || response.status === 403) {
        toast("访问被拒绝：请检查是否需要在「设备设置 → 访问令牌」填写服务端令牌（或检查 Origin 白名单）。", "error");
      }
    } catch (error) { /* 网络不可达时静默，交给自动重连 */ }
  }

  function startPing() {
    stopPing();
    // 1 s 一次：ping 用于 RTT；heartbeat 供后端 nav_operator_timeout_s(2.5 s) 判定操作者在线。
    // 原 2 s 心跳会周期撞上旧的 1.5 s 超时，导致导航/探索被误取消（Lead 2026-11-14 发现并修复）。
    state.pingTimer = setInterval(() => {
      wsSend({ type: "ping", t: performance.now() });
      // 后端未提供 client_id 时避免刷「未知消息类型」告警
      if (state.contractMode && !state.noHeartbeat) {
        wsSend({ type: "heartbeat", t: Date.now(), client_id: state.clientId });
      }
    }, 1000);
  }

  function stopPing() {
    if (state.pingTimer) clearInterval(state.pingTimer);
    state.pingTimer = null;
  }

  function handleMessage(message) {
    switch (message.type) {
      case "hello":
        onHello(message.data);
        break;
      case "state":
        updateAll(message.data);
        break;
      case "trail":
        if (message.data && message.data.pts) mapView.appendTrail(message.data.pts);
        break;
      case "scan":
        if (message.data && message.data.pts) mapView.setScan(message.data.pts);
        break;
      case "nav":
        onNav(message.data);
        break;
      case "map":
        applyMap(message.data);
        toast("地图已更新", "ok");
        break;
      case "event":
        if (message.data) {
          appendLog(message.data);
          onExplorationEvent(message.data);
        }
        break;
      case "ack":
        onAck(message);
        break;
      case "pong":
        if (typeof message.t === "number") {
          state.rtt = Math.max(0, performance.now() - message.t);
          setText("ld-rtt", `${state.rtt.toFixed(0)} ms`);
        }
        break;
      default:
        break;
    }
  }

  /* ------------------------------------------------------------------ 本机/机载地图 */
  async function refreshMapList() {
    try {
      const response = await fetch("/api/maps", { headers: authHeaders() });
      if (!response.ok) return;
      const data = await response.json();
      const select = $("map-select");
      if (!select) return;
      const previous = select.value;
      select.innerHTML = '<option value="">选择地图…</option>';
      for (const item of (data.maps || [])) {
        const option = document.createElement("option");
        option.value = item.path;
        option.textContent = item.name;
        select.appendChild(option);
      }
      if (previous) select.value = previous;
    } catch (error) { /* 地图列表非关键路径，失败静默 */ }
  }

  async function loadSelectedMap() {
    const select = $("map-select");
    const path = select && select.value;
    if (!path) {
      toast("请先在下拉框选择一张地图（host_ui/maps 下）", "warn");
      return;
    }
    await fetchJson("/api/map/load", { path });
  }

  function mappingControlReason() {
    const snapshot = state.snapshot || {};
    if (state.readOnly) return "只读接收模式";
    if (!canControl() || (state.contractMode && operatorMode() !== "mine")) {
      return "请先取得操控权";
    }
    if (!state.connected || !snapshot.link?.connected) return "机器人链路未连接";
    if (snapshot.capabilities?.mapping !== true) return "当前链路未接入机载建图";
    return "";
  }

  function requestMapping(type) {
    const reason = mappingControlReason();
    if (reason) {
      toast(reason, "warn");
      return;
    }
    wsCommand(type);
  }

  function renderMapping(snapshot) {
    const mapping = snapshot.mapping || {};
    const phase = mapping.state || "idle";
    const reason = mappingControlReason();
    const busy = phase === "starting" || phase === "stopping";
    const running = phase === "running";
    const start = $("mapping-start");
    const stop = $("mapping-stop");
    const save = $("map-save");
    start.disabled = !!reason || busy || running;
    stop.disabled = !!reason || !running;
    save.disabled = !!reason || busy || !snapshot.map?.loaded;
    start.title = reason || (running ? "机载建图已运行" : "启动机载雷达建图");
    stop.title = reason || "停止机载建图";
    save.title = reason || (snapshot.map?.loaded ? "保存当前地图" : "等待地图数据");
    const labels = {
      idle: "建图待命", starting: "建图启动中…", running: "建图中",
      stopping: "建图停止中…", failed: "建图失败",
    };
    const badge = $("mapping-state");
    badge.textContent = labels[phase] || phase;
    badge.className = `badge ${phase === "failed" ? "bad" : running ? "ok" : busy ? "warn" : "soft"}`;
    badge.title = mapping.message || reason || "";

    const localization = snapshot.localization;
    const poseBadge = $("map-localization");
    poseBadge.classList.toggle("hidden", !localization);
    if (localization) {
      const age = Number(localization.age_s);
      const ageText = localization.age_s !== null && Number.isFinite(age)
        ? ` · ${age.toFixed(2)} s` : "";
      poseBadge.textContent = `${localization.valid ? "定位有效" : "定位不可用"}${ageText}`;
      poseBadge.className = `badge ${localization.valid ? "ok" : "warn"}`;
      poseBadge.title = [localization.source, localization.frame].filter(Boolean).join(" · ");
    }
    if (snapshot.link?.kind === "ssh_ros" && snapshot.map_health?.valid === false) {
      poseBadge.classList.remove("hidden");
      poseBadge.textContent += " · 地图不可用于导航";
      poseBadge.className = "badge warn";
      poseBadge.title = snapshot.map_health.reason || "等待新鲜地图";
    }
    if (mapping.map_saved) {
      const saved = JSON.stringify(mapping.map_saved);
      if (saved !== state.lastSavedMap) {
        state.lastSavedMap = saved;
        refreshMapList();
      }
    }
  }

  /* ------------------------------------------------------------------ 自主探索 */
  function explorationStateOf(snapshot) {
    const payload = snapshot && snapshot.exploration;
    if (!payload || typeof payload !== "object") return null;
    const phase = String(payload.state || "idle");
    return { phase: EXPLORATION_TEXT[phase] ? phase : "idle", payload };
  }

  // 探索的通用操控门禁：只读 / 无操控权 / 链路断开
  function explorationControlReason(snapshot) {
    snapshot = snapshot || {};
    if (state.readOnly) return "只读接收模式：不能控制探索";
    if (!canControl() || (state.contractMode && operatorMode() !== "mine")) {
      return "请先取得操控权";
    }
    if (!state.connected || !snapshot.link?.connected) return "机器人链路未连接";
    return "";
  }

  function explorationStartReason(snapshot, current) {
    snapshot = snapshot || {};
    const reason = explorationControlReason(snapshot);
    if (reason) return reason;
    if (state.estop || snapshot.safety?.estop) return "急停锁存：先复位急停再开始探索";
    if (state.posturePending || state.armPending) return "姿态切换中，请稍候";
    if (!(snapshot.map?.loaded || snapshot.map?.describe)) return "没有地图，请先启动建图";
    const kind = snapshot.link?.kind || state.lastLinkKind || "";
    const mappingRunning = String(snapshot.mapping?.state || "") === "running";
    const localizationValid = !!(snapshot.localization && snapshot.localization.valid);
    if (kind === "ssh_ros" && !mappingRunning && !localizationValid) {
      return "请先启动建图";
    }
    if (kind === "ssh_ros" && !snapshot.map_health?.valid) {
      return snapshot.map_health?.reason || "请等待新鲜地图";
    }
    // 后端 start_exploration 的前置检查还要求「已使能（自动模式）」，提前给出提示
    const execution = snapshot.execution || snapshot.control
      || (snapshot.link && snapshot.link.execution) || {};
    const armSupported = snapshot.capabilities?.arm !== false;
    if (armSupported && execution.armed === false) return "请先站立并使能";
    if (!current) return "后端探索接口未就绪（等待联调）";
    return "";
  }

  function renderExploration(snapshot) {
    snapshot = snapshot || {};
    const current = explorationStateOf(snapshot);
    const payload = current ? current.payload : {};
    const phase = current ? current.phase : "idle";
    const start = $("exploration-start");
    const stop = $("exploration-stop");
    const badge = $("exploration-state");
    const stats = $("exploration-stats");

    // 前沿覆盖层：仅 active/paused 绘制（后端 frontiers_preview 最多 200 点）
    const preview = (phase === "active" || phase === "paused")
      ? payload.frontiers_preview : null;
    mapView.setFrontiers(Array.isArray(preview) ? preview : []);

    // 进入新一轮 active 时重置“暂停只提示一次”的标志（每个探索回合提示一次即可）；
    // 不能按“非 paused 就重置”，否则 active→paused 的同一轮里可能重复 toast。
    if (phase === "active" && state.explorationLastPhase !== "active") {
      explorationPauseToastShown = false;
    }
    state.explorationLastPhase = phase;

    if (badge) {
      badge.textContent = EXPLORATION_TEXT[phase];
      badge.className = `badge ${EXPLORATION_BADGE[phase]}`;
      badge.title = payload.message || EXPLORATION_TEXT[phase];
    }

    if (stats) {
      const frontiers = numOrNull(payload.frontiers);
      const visited = numOrNull(payload.goals_visited);
      const blacklisted = numOrNull(payload.blacklisted);
      const hasData = frontiers !== null || visited !== null || blacklisted !== null;
      stats.textContent = hasData
        ? `前沿 ${frontiers ?? "--"} · 已访问 ${visited ?? "--"} · 拉黑 ${blacklisted ?? "--"}`
        : "--";
      const goal = Array.isArray(payload.last_goal) && payload.last_goal.length >= 2
        ? payload.last_goal : null;
      stats.title = goal
        ? `最近探索目标 (${fmt(goal[0])}, ${fmt(goal[1])}) · 连续失败 `
          + `${payload.consecutive_failures ?? 0}`
        : "前沿簇数 · 已访问目标 · 已拉黑";
    }

    if (start) {
      const reason = explorationStartReason(snapshot, current);
      let label = "开始探索";
      let disabled = !!reason;
      let title = reason || "开始自主前沿探索";
      if (current && phase === "active") {
        disabled = true;
        title = "探索进行中";
      } else if (current && phase === "paused") {
        label = "继续探索";
        title = reason || "继续探索（保留已拉黑目标）";
      }
      start.textContent = label;
      start.disabled = disabled;
      start.title = title;
    }

    if (stop) {
      const reason = explorationControlReason(snapshot);
      const running = !!current && (phase === "active" || phase === "paused");
      stop.disabled = !!reason || !running;
      stop.title = reason || (running ? "立即停止自主探索" : "当前没有进行中的探索");
    }
  }

  function requestExploration(type) {
    const snapshot = state.snapshot || {};
    const current = explorationStateOf(snapshot);
    const reason = type === "exploration_start"
      ? explorationStartReason(snapshot, current)
      : explorationControlReason(snapshot);
    if (reason) {
      toast(reason, "warn");
      return;
    }
    if (!wsSend({ type })) {
      toast("与上位机服务断开，探索指令未发送", "error");
      return;
    }
    toast(type === "exploration_start" ? "已发送开始探索指令…" : "已发送停止探索指令…",
      "warn");
  }

  function onExplorationEvent(event) {
    const code = String(event && event.code || "");
    if (!EXPLORATION_EVENT_TOASTS.has(code)) return;
    const message = String(event.message || "").trim();
    if (code === "exploration_paused") {
      if (explorationPauseToastShown) return;
      explorationPauseToastShown = true;
      toast(message || "手动接管，探索暂停（可点「继续探索」恢复）", "warn");
      return;
    }
    if (!message) return;
    const level = (code === "exploration_failed" || code === "exploration_error")
      ? "error"
      : (code === "exploration_start" || code === "exploration_complete") ? "ok" : "warn";
    toast(message, level);
  }

  /* ------------------------------------------------------------------ 初始化 hello */
  function onHello(data) {
    state.hello = data;
    state.sessionToken = data.client_session || null;
    state.config = data.config || {};
    state.kinds = data.kinds || [];
    state.clientId = (data.client_id !== undefined && data.client_id !== null)
      ? data.client_id : null;
    state.contractMode = data.client_id !== undefined
      || !!(data.snapshot && data.snapshot.operator !== undefined)
      || !!data.operator_supported;
    state.operator = data.operator || data.snapshot?.operator || null;
    state.versions = data.versions || data.snapshot?.versions
      || data.snapshot?.health?.versions || null;
    state.capabilities = data.capabilities || data.snapshot?.capabilities || null;
    if (data.kind_labels) Object.assign(KIND_LABEL, data.kind_labels);
    renderKinds();
    renderTokenUi();
    renderOperator();
    refreshMapList();
    applyMap(data.map);
    mapView.setTrail(data.trail || []);
    mapView.setPath(data.nav_path || []);
    renderLogs(data.events || []);
    if (data.snapshot) {
      state.limits = data.snapshot.limits || null;
      applyLimitsToUi();
      updateAll(data.snapshot);
    }
    const align = data.snapshot?.map?.align || data.config?.map?.align || {};
    $("align-x").value = Number(align.x || 0).toFixed(2);
    $("align-y").value = Number(align.y || 0).toFixed(2);
    $("align-yaw").value = Number(align.yaw_deg || 0).toFixed(1);
    const linkKind = data.config?.link?.kind;
    if (linkKind) {
      $("link-kind").value = linkKind;
      renderLinkOptions(linkKind);
    }
    setText("link-pill-text", "与服务已连接");
    toggle($("conn-overlay"), "hidden", true);
  }

  /* ------------------------------------------------------------------ 链路面板 */
  function renderKinds() {
    const select = $("link-kind");
    if (!select) return;
    const current = select.value;
    select.innerHTML = "";
    for (const item of state.kinds) {
      const option = document.createElement("option");
      option.value = item.kind;
      option.textContent = `${item.label || KIND_LABEL[item.kind] || item.kind}`
        + (item.available === false ? "（依赖缺失）" : "");
      select.appendChild(option);
    }
    const preferred = current || state.config?.link?.kind || "mock";
    if ([...select.options].some((option) => option.value === preferred)) {
      select.value = preferred;
    }
    renderLinkOptions(select.value);
  }

  const OPTION_SKIP = new Set(["bind_host"]);
  //: 常用参数直接显示，其余收进「高级参数」
  const ESSENTIAL_OPTIONS = new Set([
    "robot_ip", "robot_port", "bind_port",
    "state_topic", "cmd_topic", "map_topic",
    "start_x", "start_y", "start_yaw_deg",
  ]);

  function buildOptionRow(key, defaultValue, saved) {
    const row = document.createElement("div");
    row.className = "opt-row";
    const label = document.createElement("label");
    label.textContent = OPTION_LABEL[key] || key;
    const input = document.createElement("input");
    input.dataset.key = key;
    const value = (saved[key] !== undefined && saved[key] !== null) ? saved[key] : defaultValue;
    input.type = (typeof defaultValue === "number") ? "number" : "text";
    if (typeof defaultValue === "number") input.step = "any";
    input.value = value;
    row.appendChild(label);
    row.appendChild(input);
    return row;
  }

  function renderLinkOptions(kind) {
    const container = $("link-options");
    const item = state.kinds.find((entry) => entry.kind === kind);
    container.innerHTML = "";
    if (!item) return;
    const saved = (state.config?.link?.kind === kind) ? (state.config.link.options || {}) : {};
    const options = item.options || {};
    const advanced = document.createElement("details");
    advanced.className = "advanced";
    const summary = document.createElement("summary");
    summary.textContent = "高级参数（看门狗 / 阈值 / 仿真细节）";
    advanced.appendChild(summary);
    const advancedBox = document.createElement("div");
    advancedBox.className = "link-options";
    let advancedCount = 0;
    for (const [key, defaultValue] of Object.entries(options)) {
      if (OPTION_SKIP.has(key)) continue;
      const row = buildOptionRow(key, defaultValue, saved);
      if (ESSENTIAL_OPTIONS.has(key)) container.appendChild(row);
      else {
        advancedBox.appendChild(row);
        advancedCount += 1;
      }
    }
    if (advancedCount > 0) {
      advanced.appendChild(advancedBox);
      container.appendChild(advanced);
    }

    const hints = {
      mock: "离线仿真：内置实验室地图与运动学模型，用于无实机时验证界面、急停与操控链路。",
      udp: "厂商 UDP：默认监听本机 43898 接收 2305 状态包，向机器人 192.168.1.120:43893 发送 320/325/321 速度包。",
      ros: "ROS1：订阅 /airace/state（JSON）并发布 /airace/cmd_vel（Twist），需要先 source ROS 环境。",
      ssh_ros: "通过 SSH 接入机载 ROS，接收真实状态与 /projected_map。SSH 密码由服务环境变量 HOST_UI_SSH_PASSWORD 提供。",
    };
    setText("link-hint", hints[kind] || "");
    const badge = $("link-badge");
    if (item.available === false) {
      badge.className = "badge danger";
      badge.textContent = "依赖缺失";
    } else if (!$("ld-state").dataset.connected) {
      badge.className = "badge";
      badge.textContent = "离线";
    }
  }

  async function doConnect() {
    const kind = $("link-kind").value;
    const options = {};
    document.querySelectorAll("#link-options input").forEach((input) => {
      const key = input.dataset.key;
      options[key] = input.type === "number" ? Number(input.value) : input.value;
    });
    const result = await fetchJson("/api/link", { kind, options });
    if (result.ok) {
      toast(result.message, "ok");
    }
  }

  async function doDisconnect() {
    const result = await fetchJson("/api/link/disconnect", {});
    if (result.ok) toast(result.message, "ok");
  }

  /* ------------------------------------------------------------------ 状态刷新 */
  function updateAll(snapshot) {
    state.snapshot = snapshot;
    const frame = snapshot.frame;
    const safety = snapshot.safety || {};
    const output = safety.output || {};
    const link = snapshot.link || {};
    const counters = snapshot.counters || {};
    const limits = snapshot.limits || state.limits || {};
    const safetyConfig = snapshot.safety_config || state.config?.safety || {};
    state.limits = limits;
    state.readOnly = !!(snapshot.link && snapshot.link.read_only);
    state.estop = !!safety.estop;
    if (state.estop) {
      state.estopUnconfirmed = false;
      state.estopPendingAt = 0;
      if (state.estopConfirmTimer) {
        clearTimeout(state.estopConfirmTimer);
        state.estopConfirmTimer = null;
      }
    }
    if (snapshot.operator !== undefined) {
      state.operator = snapshot.operator;
      state.contractMode = true;
    }
    const opMode = renderOperator();

    // 顶栏
    const linkPill = $("link-pill");
    let linkClass = "";
    let linkText = "未连接";
    if (link.connected) {
      const stale = link.stale || (link.last_frame_age_s !== null && link.last_frame_age_s > 1.5);
      linkClass = stale ? "warn" : "ok";
      linkText = `${link.label || KIND_LABEL[link.kind] || link.kind}${stale ? " · 无数据" : " · 已连接"}`;
    } else if (link.kind) {
      linkClass = "";
      const label = link.label || KIND_LABEL[link.kind] || link.kind;
      linkText = (link.kind === "offline" || label === "未连接") ? "未连接" : `${label} · 未连接`;
    }
    if (link.connected === true) {
      state.linkEverConnected = true;
      if (link.kind && link.kind !== "offline") state.lastLinkKind = link.kind;
    }
    linkPill.className = `pill ${linkClass}`;
    setText("link-pill-text", linkText);

    const modeReady = frame ? frame.mode_ready : false;
    const modePill = $("mode-pill");
    if (state.readOnly) {
      modePill.className = "pill";
      const basicText = frame && frame.basic !== null && frame.basic !== undefined
        ? (BASIC_TEXT[Number(frame.basic)] || `basic=${frame.basic}`) : "只读监控";
      setText("mode-pill-text", `只读 · ${basicText}`);
    } else if (frame && BASIC_FAULT.has(Number(frame.basic))) {
      modePill.className = "pill bad";
      setText("mode-pill-text",
        `保护 · ${BASIC_TEXT[Number(frame.basic)] || `basic=${frame.basic}`}`);
    } else {
      modePill.className = `pill ${modeReady ? "ok" : frame ? "warn" : ""}`;
      setText("mode-pill-text", modeReady ? "可运动" : frame ? "未就绪" : "--");
    }

    const estopPill = $("estop-pill");
    estopPill.className = `pill ${state.estop ? "bad" : "ok"}`;
    setText("estop-pill-text", state.estop ? "锁存" : "正常");
    setText("hz-pill", `${fmt(counters.frame_hz, 1)} / ${fmt(counters.control_hz, 1)} Hz`);

    // 急停（顶栏常驻）
    const readOnly = state.readOnly === true;
    $("estop-btn").classList.toggle("engaged", state.estop);
    $("estop-banner").classList.toggle("hidden", !state.estop);
    $("btn-estop-reset").disabled = !state.estop || readOnly;
    $("estop-btn").disabled = readOnly;
    $("btn-stop").disabled = readOnly;
    $("estop-btn").title = readOnly
      ? "只读接收模式：不发送急停指令，请使用物理急停/遥控器"
      : "急停（Esc）：立即锁存并拦截所有运动指令";
    $("btn-stop").title = readOnly
      ? "只读接收模式：不发送停车指令，请使用物理遥控器/急停"
      : "立即停车（Space）";
    const roBanner = $("readonly-banner");
    if (roBanner) roBanner.classList.toggle("hidden", !readOnly);
    const roPill = $("readonly-link-pill");
    if (roPill) roPill.classList.toggle("hidden", !readOnly);
    // 只读接收：禁止在地图上点选导航目标（地图缩放/平移/加载仍可用）
    const toolGoal = $("tool-goal");
    if (toolGoal) {
      toolGoal.disabled = readOnly;
      toolGoal.classList.toggle("active", !readOnly && state.goalPick);
      toolGoal.title = readOnly ? "只读接收模式：不能设置导航目标"
        : "在地图上点选导航目标";
    }
    if (readOnly) {
      mapView.setPickMode(false);
      $("goal-hint").classList.add("hidden");
      $("btn-go").disabled = true;
      $("btn-nav-cancel").disabled = true;
    } else if (state.goalPick) {
      mapView.setPickMode(true);
    }

    // 链路详情
    const linkBadge = $("link-badge");
    if (link.connected) {
      linkBadge.className = "badge ok";
      linkBadge.textContent = "已连接";
    } else if (link.kind && link.kind !== "offline") {
      linkBadge.className = "badge warn";
      linkBadge.textContent = "未连接";
    } else {
      linkBadge.className = "badge";
      linkBadge.textContent = "离线";
    }
    $("ld-state").dataset.connected = link.connected ? "1" : "";
    setText("ld-state", link.connected ? (link.detail || "已连接") : "未连接");
    setText("ld-frames", counters.frames ?? 0);
    setText("ld-age", link.last_frame_age_s !== null && link.last_frame_age_s !== undefined
      ? `${fmt(link.last_frame_age_s * 1000, 0)} ms` : "--");

    // 位姿
    const pose = snapshot.pose;
    setText("pose-frame", snapshot.map?.loaded ? "地图对齐坐标" : "机身里程计坐标");
    const execution = snapshot.execution || link.execution || {};
    const reason = execution.reason || "";
    setText("execution-state", reason ? reasonCn(reason) : "--");
    const battery = batteryPercent(snapshot);
    setText("battery-level", battery === null ? "--" : `${Math.round(battery)}%`);
    if (pose) {
      setText("pose-x", fmt(pose[0], 2));
      setText("pose-y", fmt(pose[1], 2));
      setText("pose-yaw", fmt(pose[2], 1));
      mapView.setPose({ x: pose[0], y: pose[1], yaw_deg: pose[2] });
      const now = performance.now();
      if (state.lastPose && now - state.lastPoseTime > 30) {
        let delta = pose[2] - state.lastPose[2];
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        const dt = (now - state.lastPoseTime) / 1000;
        if (dt > 0.02) state.yawRate = (delta * Math.PI / 180) / dt;
      }
      state.lastPose = pose;
      state.lastPoseTime = now;
    }

    if (frame) {
      const rpy = frame.rpy_deg || [0, 0, 0];
      setText("rpy-r", `${fmt(rpy[0], 1)}°`);
      setText("rpy-p", `${fmt(rpy[1], 1)}°`);
      const velocity = frame.velocity_body || [0, 0, 0];
      setText("vel-vx", `${fmt(velocity[0], 2)} m/s`);
      setText("vel-wz", `${fmt(state.yawRate, 2)} rad/s`);
      setText("frame-age", `${fmt(frame.age_s * 1000, 0)} ms`);
      setText("basic-text", frame.mode_ready ? "可运动" : "未就绪");
      setText("basic-code", `basic=${frame.basic} · 可运动码=6`);
      setText("gait-text", `gait=${frame.gait}`);
      setText("motion-text", `motion=${frame.motion}`);
      const joystick = frame.joystick || [0, 0, 0];
      const stickMax = Math.max(...joystick.map((value) => Math.abs(Number(value) || 0)));
      setText("joystick-text", `${stickMax.toFixed(2)} / ${joystick.map((v) => Number(v).toFixed(1)).join("·")}`);
      setText("source-badge", KIND_LABEL[frame.source] || frame.source || "--");
    } else {
      setText("vel-vx", "--");
      setText("vel-wz", "--");
      setText("frame-age", "--");
      setText("basic-text", "--");
      setText("basic-code", "basic --");
      setText("gait-text", "--");
      setText("motion-text", "--");
      setText("joystick-text", "--");
      setText("source-badge", "--");
    }
    setText("odometer", `${fmt(counters.odometer_m, 2)} m`);

    // 限幅
    setText("hard-forward", `${fmt(limits.max_forward_m_s, 2)} m/s`);
    setText("hard-reverse", `${fmt(limits.max_reverse_m_s, 2)} m/s`);
    setText("hard-yaw", `${fmt(limits.max_yaw_rad_s, 2)} rad/s`);
    setText("hard-accel", `${fmt(limits.max_accel_m_s2, 2)} m/s²`);
    applyLimitsToUi();

    // 输出与拦截
    const linear = Number(output.linear || 0);
    const angular = Number(output.angular || 0);
    setText("cmd-v", fmt(linear, 2));
    setText("cmd-w", fmt(angular, 2));
    setText("output-text", `${fmt(linear, 2)} m/s · ${fmt(angular, 2)} rad/s`);
    setText("output-source", SOURCE_TEXT[output.source] || output.source || "待命");
    const zeroReason = output.zero_reason || "";
    const showFault = zeroReason && zeroReason !== "no_input";
    $("fault-row").classList.toggle("hidden", !showFault);
    if (showFault) setText("fault-text", ZERO_TEXT[zeroReason] || zeroReason);

    // 安全标志位
    renderFlags(frame, safety, output);

    // 批次 1：操作权 / 使能就绪 / 设备健康 / 断线横幅 / 数据过期
    renderControlPanel(snapshot, opMode);
    renderHealth(snapshot);
    renderLinkBanner(snapshot);
    renderPoseStale(link, safetyConfig);

    // 死按钮状态
    const deadman = !!safety.teleop?.deadman && output.source === "teleop";
    const deadmanBadge = $("deadman-badge");
    deadmanBadge.classList.toggle("active", deadman);
    deadmanBadge.textContent = deadman ? "操控中" : "待命";

    // 安全看门狗
    setText("wd-mode", snapshot.simulation ? "离线仿真" : (KIND_LABEL[link.kind] || link.kind || "--"));
    setText("wd-clients", counters.clients ?? 0);
    setText("wd-cmd-timeout", safetyConfig.command_timeout_s
      ? `${Math.round(safetyConfig.command_timeout_s * 1000)} ms` : "--");
    setText("wd-state-timeout", safetyConfig.state_timeout_s
      ? `${Math.round(safetyConfig.state_timeout_s * 1000)} ms` : "--");
    setText("wd-ready-basic", safetyConfig.ready_basic ?? "--");
    setText("wd-estop-count", safety.estop_engages ?? 0);
    const watchdogBadge = $("watchdog-badge");
    const faultActive = !!zeroReason && zeroReason !== "no_input";
    watchdogBadge.className = `badge ${faultActive ? "warn" : "ok"}`;
    watchdogBadge.textContent = faultActive ? (state.estop ? "急停锁存" : "拦截中") : "正常";

    // 录制
    const recording = snapshot.recording || {};
    setText("rec-rows", recording.rows ?? 0);
    setText("rec-events", recording.events ?? 0);
    setText("rec-path", recording.active ? recording.path : (recording.path || "--"));
    const recordBadge = $("record-badge");
    recordBadge.className = `badge ${recording.active ? "warn" : "soft"}`;
    recordBadge.textContent = recording.active ? "录制中" : "未录制";
    $("btn-record").textContent = recording.active ? "停止录制" : "开始录制";
    $("btn-record").dataset.active = recording.active ? "1" : "";
    $("btn-download").disabled = !(recording.rows > 0) && !(recording.events > 0) && !recording.active;

    // 地图信息
    const mapInfo = snapshot.map || {};
    if (mapInfo.describe) {
      const describe = mapInfo.describe;
      const key = `${describe.name}:${describe.width}:${describe.height}:${describe.resolution}`;
      if (key !== state.lastFundamentalMapKey) {
        state.lastFundamentalMapKey = key;
        setText("map-meta", `${describe.name} · ${describe.width}×${describe.height} · `
          + `@${Number(describe.resolution).toFixed(3)} m/格 · `
          + `${(describe.width * describe.resolution).toFixed(1)}×`
          + `${(describe.height * describe.resolution).toFixed(1)} m`);
      }
      $("map-empty").classList.add("hidden");
    } else {
      setText("map-meta", "未加载地图");
      $("map-empty").classList.remove("hidden");
    }

    // 导航
    renderMapping(snapshot);
    renderExploration(snapshot);
    onNav(snapshot.nav || state.nav);
    updateStatusChips(snapshot);

    // 模拟调试按钮
    if (snapshot.simulation) $("sim-buttons").classList.remove("hidden");
    else $("sim-buttons").classList.add("hidden");

    // 曲线
    pushChartSample(output, frame);
    state.lastStateAt = Date.now();
  }

  function renderFlags(frame, safety, output) {
    const row = $("flag-row");
    const chips = [];
    const add = (text, cls) => chips.push([text, cls]);
    if (frame && frame.mode_ready) add("可运动", "on");
    if (state.estop) add("急停锁存", "bad");
    if (frame && frame.takeover) add("遥控接管", "warn");
    if (safety.output && safety.output.zero_reason === "state_timeout") add("状态超时", "bad");
    if (output.source === "nav") add("导航输出", "warn");
    if (output.flags && output.flags.clamped) add("已限幅", "warn");
    if (!frame) add("无状态", "warn");
    if (!chips.length) add("待命", "");
    row.innerHTML = "";
    for (const [text, cls] of chips) {
      const span = document.createElement("span");
      span.className = `flag ${cls}`;
      span.textContent = text;
      row.appendChild(span);
    }
  }

  function updateStatusChips(snapshot) {
    const nav = snapshot.nav || {};
    const counters = snapshot.counters || {};
    $("map-mode-chip").textContent = snapshot.link?.connected
      ? `${KIND_LABEL[snapshot.link.kind] || snapshot.link.kind} · ${counters.frames ?? 0} 帧`
      : "链路未连接";
    const navText = NAV_TEXT[nav.state] || nav.state || "导航待命";
    $("nav-chip").textContent = nav.distance_m !== null && nav.distance_m !== undefined
      ? `${navText} · 距目标 ${fmt(nav.distance_m, 2)} m` : navText;
    $("nav-chip").classList.toggle("subtle", !nav.active);
  }

  /* ------------------------------------------------------------------ 地图 */
  function applyMap(payload) {
    if (!payload) {
      mapView.setMap(null);
      state.mapCells = null;
      state.mapShape = null;
      $("map-empty").classList.remove("hidden");
      setText("map-meta", "未加载地图");
      return;
    }
    mapView.setMap(payload);
    // 保存栅格用于点选目标的越界/障碍校验（map.js 只暴露渲染所需元数据）
    try {
      const binary = atob(payload.data || "");
      const cells = new Int8Array(binary.length);
      for (let i = 0; i < binary.length; i += 1) {
        const value = binary.charCodeAt(i);
        cells[i] = value > 127 ? value - 256 : value;
      }
      state.mapCells = cells;
      state.mapShape = {
        width: payload.width,
        height: payload.height,
        resolution: payload.resolution,
        origin: payload.origin || { x: 0, y: 0, yaw_deg: 0 },
      };
    } catch (error) {
      state.mapCells = null;
      state.mapShape = null;
    }
    $("map-empty").classList.add("hidden");
    setText("map-meta", `${payload.name || "map"} · ${payload.width}×${payload.height} · `
      + `@${Number(payload.resolution).toFixed(3)} m/格 · `
      + `${(payload.width * payload.resolution).toFixed(1)}×`
      + `${(payload.height * payload.resolution).toFixed(1)} m`);
    if (state.snapshot?.pose) {
      mapView.setPose({
        x: state.snapshot.pose[0], y: state.snapshot.pose[1],
        yaw_deg: state.snapshot.pose[2],
      });
    }
    fitMapSoon();
  }

  function updateGoalUi() {
    const pending = state.pendingGoal;
    const nav = state.nav || {};
    const allowed = canControl();
    if (pending) {
      setText("goal-text", `(${pending.x.toFixed(2)}, ${pending.y.toFixed(2)})`);
      $("btn-go").disabled = !allowed;
    } else if (nav.goal) {
      setText("goal-text", `(${Number(nav.goal[0]).toFixed(2)}, ${Number(nav.goal[1]).toFixed(2)})`);
      $("btn-go").disabled = true;
    } else {
      setText("goal-text", "未设置");
      $("btn-go").disabled = true;
    }
    $("btn-nav-cancel").disabled = !nav.active || !allowed;
    $("goal-hint").classList.toggle("hidden", !(state.goalPick && !pending && !nav.active));
  }

  function onNav(nav) {
    if (!nav) return;
    state.nav = nav;
    mapView.setPath(nav.path || []);
    mapView.setNavActive(!!nav.active);
    if (nav.goal) {
      mapView.setGoal({ x: nav.goal[0], y: nav.goal[1], yaw_deg: nav.goal[2] });
      state.pendingGoal = null;
    } else if (!state.pendingGoal) {
      mapView.setGoal(null);
    }
    updateGoalUi();
  }

  async function goToGoal() {
    if (!state.pendingGoal) return;
    const goal = state.pendingGoal;
    const result = await fetchJson("/api/goal", { x: goal.x, y: goal.y });
    if (!result.ok) {
      toast(result.message || "导航启动失败", "error");
    }
  }

  async function cancelNav() {
    const result = await fetchJson("/api/nav/cancel", {});
    if (result.ok) {
      state.pendingGoal = null;
      mapView.setGoal(null);
      updateGoalUi();
    }
  }

  /* ------------------------------------------------------------------ 操作权 / 使能 / 健康 */
  function pickFirst(...values) {
    for (const value of values) {
      if (value !== null && value !== undefined) return value;
    }
    return undefined;
  }

  function numOrNull(value) {
    const number = Number(value);
    return value === null || value === undefined || !Number.isFinite(number) ? null : number;
  }

  function pickBool(...values) {
    for (const value of values) {
      if (value === true || value === false) return value;
    }
    return undefined;
  }

  function reasonCn(reason) {
    return REASON_TEXT[reason] || reason || "--";
  }

  function batteryPercent(snapshot) {
    const frame = snapshot.frame || {};
    const health = snapshot.health || {};
    const control = snapshot.control || snapshot.link?.control || {};
    let raw = pickFirst(snapshot.battery?.percent, snapshot.battery?.battery_percent,
      snapshot.battery_percent, frame.battery_percent, frame.raw?.battery_percent,
      health.battery_percent, control.battery_percent);
    if (raw === undefined) {
      raw = pickFirst(frame.battery_level_raw, health.battery_level_raw);
    }
    if (raw === undefined || raw === null) return null;
    let value = Number(raw);
    if (!Number.isFinite(value)) return null;
    // 厂商文档 0.00~1.00，实测固件直接给 0~100；>1.5 视为百分数
    if (value >= 0 && value <= 1.5) value *= 100;
    return Math.max(0, Math.min(100, value));
  }

  function effectiveOperator() {
    return state.operatorTestOverride !== null ? state.operatorTestOverride : state.operator;
  }

  function operatorMode() {
    if (!state.contractMode) return "unsupported";
    const operator = effectiveOperator();
    if (!operator || operator.id === null || operator.id === undefined) return "none";
    if (state.clientId !== null && String(operator.id) === String(state.clientId)) return "mine";
    return "other";
  }

  function canControl() {
    if (state.readOnly) return false;
    return operatorMode() !== "other";
  }

  function movementLocked() {
    return !canControl() || state.readOnly || state.posturePending || state.armPending;
  }

  function renderOperator() {
    const mode = operatorMode();
    if (state.readOnly) {
      const pill = $("control-pill");
      if (pill) {
        pill.className = "pill warn control-pill readonly";
        setText("control-pill-text", "只读");
        pill.title = "只读接收模式：不会向机器人发送任何运动/姿态/急停指令";
      }
      setText("op-holder", "只读接收");
      setText("op-since", "--");
      setText("op-clients", String(state.snapshot?.counters?.clients ?? "--"));
      setText("op-self", state.clientId !== null ? `#${state.clientId}` : "--");
      const hint = $("op-hint");
      if (hint) {
        hint.textContent = "只读接收模式：界面仅监控机载数据，不发送运动/姿态/急停指令；"
          + "需要操作请使用物理遥控器/急停，或切换到非只读链路。";
      }
      ["op-claim", "op-takeover", "op-release"].forEach((id) => {
        const node = $(id);
        if (node) node.disabled = true;
      });
      document.body.classList.add("observer");
      const readonly = $("readonly-badge");
      if (readonly) {
        readonly.classList.remove("hidden");
        readonly.textContent = "只读接收 · 不发指令";
      }
      document.querySelectorAll(".dpad-btn").forEach((button) => {
        button.disabled = true;
      });
      return "readonly";
    }
    const operator = effectiveOperator();
    const pill = $("control-pill");
    if (pill) {
      pill.className = `pill control-pill ${mode}`;
      setText("control-pill-text", { mine: "我", other: "他人", none: "无", unsupported: "未启用" }[mode]);
      pill.title = mode === "unsupported"
        ? "后端尚未提供操作权仲裁（hello.client_id / state.operator）"
        : mode === "mine" ? "你持有操控权：点击可释放"
          : mode === "other" ? "操控权被其他客户端持有：只读，仅可急停/停车"
            : "当前无人持有操控权：点击申请";
    }
    const holder = operator && operator.id !== null && operator.id !== undefined
      ? `#${operator.id}` : "无";
    setText("op-holder", mode === "unsupported" ? "未启用（后端未提供）"
      : holder === "无" ? "无" : (mode === "mine" ? `我（${holder}）` : `客户端 ${holder}`));
    let since = "--";
    if (operator && operator.since !== undefined && operator.since !== null) {
      const stamp = Number(operator.since);
      if (Number.isFinite(stamp) && stamp > 0) {
        const seconds = stamp > 1e12 ? (Date.now() - stamp) / 1000 : (Date.now() / 1000 - stamp);
        since = seconds >= 0 ? `${Math.round(seconds)} s` : "--";
      }
    }
    setText("op-since", since);
    setText("op-clients", operator && operator.clients !== undefined
      ? String(operator.clients) : String(state.snapshot?.counters?.clients ?? "--"));
    setText("op-self", state.clientId !== null ? `#${state.clientId}` : "--");
    const hint = $("op-hint");
    if (hint) {
      hint.textContent = mode === "unsupported"
        ? "后端未提供操作权仲裁字段（hello.client_id / state.operator），当前按单操作者兼容模式运行。"
        : mode === "other"
          ? "操控权由其他客户端持有：手动操控/使能/姿态/导航已置灰（只读），急停与立即停车始终可用。"
          : mode === "none" ? "当前无人持有操控权：申请或接管后可操控机器狗。"
            : "你持有操控权，可下发速度/使能/姿态/导航指令。";
    }
    const claim = $("op-claim");
    const takeover = $("op-takeover");
    const release = $("op-release");
    if (claim) claim.disabled = mode === "unsupported" || mode === "mine";
    if (takeover) takeover.disabled = mode === "unsupported";
    if (release) release.disabled = mode !== "mine";
    document.body.classList.toggle("observer", mode === "other");
    const readonly = $("readonly-badge");
    if (readonly) readonly.classList.toggle("hidden", mode !== "other");
    document.querySelectorAll(".dpad-btn").forEach((button) => {
      button.disabled = mode === "other";
    });
    return mode;
  }

  function requestOperator(action) {
    const type = { claim: "claim_control", takeover: "takeover_control", release: "release_control" }[action];
    if (!type) return;
    const pop = $("operator-pop");
    if (pop) pop.classList.add("hidden");
    if (!state.contractMode) {
      toast("后端尚未提供操作权仲裁，当前为兼容模式（无需申请）", "warn");
      return;
    }
    wsCommand(type, { client_id: state.clientId });
    toast({ claim: "已发送操控权申请，等待仲裁…", takeover: "已发送接管请求，等待仲裁…",
      release: "已发送释放请求…" }[action], "warn");
  }

  function renderReadyList(items) {
    const list = $("ready-list");
    if (!list) return;
    const icon = { ok: "✓", fail: "✗", unknown: "?" };
    const signature = items.map((item) => `${item.label}|${item.status}|${item.value}`).join("~");
    if (signature === state.readySig && list.children.length === items.length) return;
    state.readySig = signature;
    list.innerHTML = "";
    for (const item of items) {
      const row = document.createElement("div");
      row.className = `ready-item ${item.status}`;
      const mark = document.createElement("i");
      mark.textContent = icon[item.status] || "?";
      const label = document.createElement("span");
      label.textContent = item.label;
      const value = document.createElement("b");
      value.textContent = item.value;
      value.title = item.title || item.value;
      row.append(mark, label, value);
      list.appendChild(row);
    }
  }

  function renderControlPanel(snapshot, opMode) {
    const link = snapshot.link || {};
    const frame = snapshot.frame || null;
    const health = snapshot.health || {};
    const control = snapshot.control || link.control || {};
    const execution = snapshot.execution || link.execution || {};
    const posture = snapshot.posture || {};
    const capabilities = snapshot.capabilities || {};
    const backendPosturePending = posture.state === "pending" || posture.state === "running"
      || posture.active === true || posture.pending === true;
    const postureWaitMs = (Math.min(30, Math.max(2,
      Number(state.config?.posture?.timeout_s) || 20)) + 7) * 1000;
    const pending = control.pending || execution.pending || (backendPosturePending ? "posture" : null);
    const armed = pickBool(control.armed, execution.armed, health.armed,
      typeof execution.reason === "string" && execution.reason
        ? !["disarmed", "disabled"].includes(execution.reason) : undefined);
    state.armed = armed === undefined ? null : armed;
    const pendingText = pending === "prepare" ? "使能请求中…"
      : pending === "disarm" ? "解除中…" : pending === "posture" ? "姿态切换中…" : "";
    // 请求中的本地标记：状态到位或超时后自动清除，避免按钮长期置灰
    if (state.armPending) {
      const reached = state.armIntent === "arm" ? armed === true
        : state.armIntent === "disarm" ? armed === false : false;
      if (reached || Date.now() - (state.armPendingSince || 0) > 9000) {
        state.armPending = false;
        state.armIntent = null;
      }
    }
    if (state.posturePending && !backendPosturePending
        && frame && (frame.basic === 6 || frame.basic === 1)) {
      const target = state.postureIntent === "stand" ? 6 : state.postureIntent === "lie" ? 1 : null;
      const finished = ["ok", "failed", "cancelled"].includes(posture.state)
        && posture.since !== state.postureBackendSince;
      if (target === null || frame.basic === target
          || finished || Date.now() - (state.posturePendingSince || 0) > postureWaitMs) {
        state.posturePending = false;
        state.postureIntent = null;
      }
    } else if (state.posturePending && !backendPosturePending
        && (Date.now() - (state.posturePendingSince || 0) > postureWaitMs
          || (["failed", "cancelled"].includes(posture.state)
            && posture.since !== state.postureBackendSince))) {
      state.posturePending = false;
      state.postureIntent = null;
    }
    const armState = pendingText ? "pending"
      : armed === true ? "armed" : armed === false ? "disarmed" : "unknown";
    const armBadge = $("arm-state");
    if (armBadge) {
      armBadge.className = `badge ${state.readOnly ? "warn" : armState === "armed" ? "ok" : armState === "disarmed" ? "warn" : "soft"}`;
      armBadge.textContent = state.readOnly ? "只读接收" : (pendingText || ARM_STATE_TEXT[armState]);
      armBadge.title = state.readOnly
        ? "只读接收模式：不会发送使能/解除指令" : "";
    }
    const execNode = $("execution-state");
    if (execNode) {
      const reason = execution.reason || health.execution_reason || "";
      execNode.textContent = reason ? reasonCn(reason)
        : state.readOnly ? "只读模式（不接收执行状态）" : "--";
      execNode.title = reason ? `原始字段：${reason}` : "";
      execNode.classList.toggle("low", ["estop", "vendor_error", "body_tilt"].includes(reason));
    }

    const readyBasic = Number(snapshot.safety_config?.ready_basic
      ?? state.config?.safety?.ready_basic ?? 6);
    const stateTimeout = Number(snapshot.safety_config?.state_timeout_s ?? 0.8);
    const lowBattery = Number(snapshot.safety_config?.low_battery_percent ?? 20);
    const age = numOrNull(link.last_frame_age_s ?? frame?.age_s);
    const basic = frame && frame.basic !== null && frame.basic !== undefined
      ? Number(frame.basic) : null;
    const depthAge = numOrNull(pickFirst(control.depth_age_s, health.depth_age_s,
      frame?.depth_age_s));
    const bridgeDepth = numOrNull(pickFirst(health.bridge_depth_age_s,
      control.bridge_depth_age_s));
    const cameraDepth = numOrNull(pickFirst(health.camera_depth_age_s,
      control.camera_depth_age_s));
    const depthFresh = (health.depth_fresh !== undefined
      ? health.depth_fresh : control.depth_fresh);
    const modeConfirmed = pickBool(control.mode_confirmed, health.mode_confirmed);
    const battery = batteryPercent(snapshot);
    const protect = basic !== null && BASIC_FAULT.has(basic);
    const protectText = protect
      ? `保护状态：basic=${basic} ${BASIC_TEXT[basic] || ""}，普通姿态/使能操作已禁用`
      : "";
    const items = [
      {
        label: "链路连接",
        status: link.connected === true ? "ok" : link.connected === false ? "fail" : "unknown",
        value: link.connected !== true ? "未连接"
          : state.readOnly ? "已连接 · 只读接收" : "已连接",
      },
      {
        label: "状态帧龄",
        status: age === null ? "unknown" : age <= stateTimeout ? "ok" : "fail",
        value: age === null ? "无数据"
          : `${Math.round(age * 1000)} ms / ≤${Math.round(stateTimeout * 1000)} ms`,
      },
      {
        label: `basic == ${readyBasic}`,
        status: basic === null ? "unknown" : basic === readyBasic ? "ok" : "fail",
        value: basic === null ? "无状态" : `basic=${basic} ${BASIC_TEXT[basic] ? `(${BASIC_TEXT[basic]})` : ""}`,
      },
      {
        label: "深度心跳",
        status: depthFresh === true ? "ok" : depthFresh === false ? "fail"
          : (depthAge === null ? "unknown" : depthAge <= 0.5 ? "ok" : "fail"),
        value: depthFresh === true
          ? `桥接判定新鲜${bridgeDepth === null ? "" : ` · ${fmt(bridgeDepth, 2)} s`}`
          : depthFresh === false
            ? `桥接判定不新鲜${bridgeDepth === null ? "" : ` · ${fmt(bridgeDepth, 2)} s`}`
            : (depthAge === null ? "未提供（旧桥/只读）"
              : `${fmt(depthAge, 2)} s（旧桥回退）`),
        title: "以机载执行桥接实际判定为准；"
          + `相机帧龄=${cameraDepth === null ? "--" : `${fmt(cameraDepth, 2)} s`}`,
      },
      {
        label: "自动模式确认",
        status: modeConfirmed === undefined ? "unknown" : modeConfirmed ? "ok" : "fail",
        value: modeConfirmed === undefined ? "后端未提供" : modeConfirmed ? "已确认" : "未确认",
      },
      {
        label: "电量",
        status: battery === null ? "unknown" : battery >= lowBattery ? "ok" : "fail",
        value: battery === null ? "后端未提供"
          : `${Math.round(battery)}%（低电阈值 ${Math.round(lowBattery)}%）`,
      },
      {
        label: "basic 状态正常（非 8/9/11）",
        status: basic === null ? "unknown" : BASIC_FAULT.has(basic) ? "fail" : "ok",
        value: basic === null ? "无状态" : (BASIC_TEXT[basic] || `basic=${basic}`),
        title: "8 失控保护 / 9 自主爬起 / 11 翻倒 视为基本状态异常；error_state 在 Lite 上无效，仅记录",
      },
    ];
    if (state.readOnly) {
      // 只读接收模式：不参与使能/运动就绪判断，避免把"未站立/未使能"误显示成故障
      items[2].status = "unknown";
      items[2].label = "basic 状态（只读监控，不要求站立）";
      items[2].title = "只读接收模式不会使能或发运动指令，basic==6 不是必需条件";
      items[3].status = "unknown";
      items[3].value = "只读模式不发布（不参与运动门禁）";
      items[3].title = "只读模式下机载流不会发布 /airace/depth_received";
      items[4].status = "unknown";
      items[4].value = "只读模式不可用";
      items[4].title = "只读模式不接收/不执行自动模式切换";
    }
    renderReadyList(items);

    const observer = opMode === "other" || state.readOnly === true;
    const connected = link.connected === true;
    const pendingAny = !!pending || state.armPending || state.posturePending;
    const armSupported = capabilities.arm !== false;
    const postureSupported = capabilities.posture !== false;
    if (protect) {
      setText("posture-result", protectText);
    } else if (state.readOnly) {
      setText("posture-result", "只读接收模式：不发送姿态/使能指令");
    }
    const armBtn = $("btn-arm");
    const disarmBtn = $("btn-disarm");
    const standBtn = $("btn-stand");
    const lieBtn = $("btn-lie");
    const cancelBtn = $("btn-posture-cancel");
    if (armBtn) {
      armBtn.disabled = observer || !connected || pendingAny || armed === true
        || !armSupported || protect;
      armBtn.title = protect ? protectText
        : (armSupported ? "机载进入自动模式并允许运动"
          : "当前链路不支持使能（capabilities.arm=false）");
    }
    if (disarmBtn) {
      disarmBtn.disabled = observer || !connected || pendingAny || armed !== true
        || !armSupported;
      disarmBtn.title = armSupported ? "解除使能并回手动站立"
        : "当前链路不支持解除使能";
    }
    if (standBtn) standBtn.disabled = observer || !connected || pendingAny
      || !postureSupported || basic === null || protect || basic !== 1;
    if (standBtn && protect) standBtn.title = protectText;
    if (lieBtn) lieBtn.disabled = observer || !connected || pendingAny
      || !postureSupported || basic === null || protect || basic !== 6;
    if (lieBtn && protect) lieBtn.title = protectText;
    if (cancelBtn) cancelBtn.disabled = observer
      || !(pending === "posture" || state.posturePending
        || backendPosturePending || snapshot.safety?.posture?.active);
    const resultHint = $("posture-result");
    if (resultHint && !state.posturePending && !state.armPending) {
      const text = posture.message || execution.last_error || "";
      if (text && resultHint.textContent !== text) resultHint.textContent = text;
    }
  }

  function renderHealth(snapshot) {
    const frame = snapshot.frame || {};
    const health = snapshot.health || {};
    const link = snapshot.link || {};
    const versions = state.versions || {};
    const control = snapshot.control || link.control || {};
    const execution = snapshot.execution || link.execution || {};
    const lowBattery = Number(snapshot.safety_config?.low_battery_percent ?? 20);
    const battery = batteryPercent(snapshot);
    const batteryLow = health.low_battery === true || snapshot.battery?.low === true
      || (battery !== null && battery <= lowBattery);

    const batteryNode = $("health-battery");
    if (batteryNode) {
      batteryNode.textContent = battery === null ? "后端未提供"
        : `${Math.round(battery)}%${batteryLow ? " · 低电告警" : ""}`;
      batteryNode.classList.toggle("low", battery !== null && batteryLow);
    }
    const errorState = numOrNull(pickFirst(frame.error_state, frame.raw?.error_state,
      health.error_state));
    const errorNode = $("health-error");
    if (errorNode) {
      errorNode.textContent = errorState === null ? "后端未提供" : `${errorState} · Lite 无效`;
      errorNode.title = "厂商字段 error_state：文档标注 Lite 无效、实测恒 0，仅记录，不参与安全判断";
      errorNode.classList.remove("low", "warn");
    }
    const motion = pickFirst(frame.motion_state, frame.motion, health.motion_state);
    setText("health-motion", motion === undefined ? "后端未提供"
      : (MOTION_TEXT[Number(motion)] || `motion=${motion}`));
    const jointTemps = health.joint_temperature_c;
    let temperature = numOrNull(pickFirst(health.temperature_c, health.max_c));
    if (temperature === null && Array.isArray(jointTemps) && jointTemps.length) {
      const values = jointTemps.map(Number).filter(Number.isFinite);
      if (values.length) temperature = Math.max(...values);
    }
    const tempSource = String(health.temperature_source || "");
    const tempNode = $("health-temperature");
    if (tempNode) {
      tempNode.textContent = temperature === null ? "未接入（过温/温度话题未上报）"
        : `${Math.round(temperature)} °C${tempSource === "/temperature" ? "（机载）" : tempSource === "joint" ? "（关节）" : ""}`;
      tempNode.classList.toggle("low", temperature !== null && temperature >= 70);
      tempNode.title = temperature === null
        ? "厂商仅在关节过温时上报 0x11050400；机载 /temperature 话题未发布时无数据"
        : "机载 /temperature（cloud_node）或关节过温上报；过温阈值按 70 °C 预警（待实机标定）";
    }
    const depth = numOrNull(pickFirst(control.depth_age_s, health.depth_age_s,
      snapshot.execution?.depth_age_s));
    const bridgeDepth = numOrNull(pickFirst(health.bridge_depth_age_s,
      control.bridge_depth_age_s, depth));
    const cameraDepth = numOrNull(pickFirst(health.camera_depth_age_s,
      control.camera_depth_age_s));
    const depthFresh = (health.depth_fresh !== undefined)
      ? health.depth_fresh : control.depth_fresh;
    const depthNode = $("health-depth");
    if (depthNode) {
      if (depthFresh === true) {
        depthNode.textContent = `新鲜${bridgeDepth === null ? "" : ` · ${fmt(bridgeDepth, 2)} s`}`;
      } else if (depthFresh === false) {
        depthNode.textContent = `不新鲜${bridgeDepth === null ? "" : ` · ${fmt(bridgeDepth, 2)} s`}`;
      } else {
        depthNode.textContent = "未提供（旧桥/只读）";
      }
      depthNode.classList.toggle("low", depthFresh === false);
      depthNode.title = "机载 execution_guard 实际用于运动门禁的深度心跳判定";
    }
    setText("health-depth-camera", cameraDepth === null ? "--" : `${fmt(cameraDepth, 2)} s`);
    const session = health.control_session || control.control_session || {};
    const sessionNode = $("health-session");
    if (sessionNode) {
      const active = session.active === true;
      sessionNode.textContent = active
        ? `运行中（心跳 ${session.heartbeat_sent ?? "--"} 次）`
        : (session.paused_reason ? `已暂停（${session.paused_reason}）` : "未建立");
      sessionNode.classList.toggle("low", !active);
      sessionNode.title = active
        ? "机载桥接正在以 5 Hz 发送 0x21040001，站立待命期间持续有效"
        : "调用使能或姿态服务后由机载桥接建立控制会话";
    }
    setText("health-heartbeat", session.heartbeat_age_s === undefined ||
      session.heartbeat_age_s === null ? "--" : `${fmt(session.heartbeat_age_s, 2)} s`);
    const executionReady = $("health-execution");
    if (executionReady) {
      const armed = pickBool(control.armed, health.armed, execution.armed);
      const confirmed = pickBool(control.mode_confirmed, health.mode_confirmed,
        execution.mode_confirmed);
      executionReady.textContent = armed === true && confirmed === true
        ? "已使能"
        : (armed === false ? "未使能" : "未提供");
      executionReady.classList.toggle("low", armed === false);
    }
    setText("health-host-version", String(pickFirst(versions.host, versions.host_version,
      versions.host_ui, health.host_version, snapshot.version) ?? "--"));
    setText("health-onboard-version", String(pickFirst(versions.onboard, versions.onboard_version,
      versions.robot, health.onboard_version, link.version) ?? "--"));
    const reconnect = pickFirst(link.reconnect?.attempts, snapshot.reconnect?.attempts,
      health.reconnect_attempts);
    setText("health-reconnect", reconnect === undefined ? "--" : String(reconnect));
    const age = numOrNull(link.last_frame_age_s ?? frame.age_s);
    setText("health-frame-age", age === null ? "--" : `${Math.round(age * 1000)} ms`);

    const badge = $("health-badge");
    if (badge) {
      const overTemp = temperature !== null && temperature >= 70;
      const alarm = batteryLow || overTemp;
      const offline = link.connected === false;
      badge.className = `badge ${alarm ? "danger" : offline ? "warn" : battery === null ? "soft" : "ok"}`;
      badge.textContent = overTemp ? "关节过温" : batteryLow ? "低电告警"
        : offline ? "链路断开"
          : battery === null ? "部分字段待后端" : "正常";
    }
  }

  function renderLinkBanner(snapshot) {
    const link = snapshot.link || {};
    const banner = $("link-banner");
    if (!banner) return;
    // 断线横幅：链路曾经连接过即视为“断线”（含手动断开后 kind=offline）
    const everConnected = state.linkEverConnected === true;
    const disconnected = link.connected === false
      && (!!link.kind && link.kind !== "" && (link.kind !== "offline" || everConnected));
    banner.classList.toggle("hidden", !disconnected);
    if (!disconnected) return;
    const reconnect = link.reconnect || snapshot.reconnect || {};
    const attempts = Number(reconnect.attempts ?? 0);
    const next = reconnect.next_retry_in_s;
    const kindLabel = KIND_LABEL[link.kind] || KIND_LABEL[state.lastLinkKind]
      || link.label || "链路";
    setText("link-banner-title", `机器人链路断开（${kindLabel}）`);
    let text = attempts > 0 ? `自动重连中（第 ${attempts} 次）` : "自动重连中…";
    if (next !== null && next !== undefined) text += `，${Number(next).toFixed(1)} s 后重试`;
    if (link.detail && link.detail !== "未连接") text += ` · ${link.detail}`;
    setText("link-banner-text", text);
  }

  function renderPoseStale(link, safetyConfig) {
    const card = $("pose-card");
    const mark = $("pose-stale");
    if (!card) return;
    const timeout = Number(safetyConfig?.state_timeout_s ?? 0.8);
    const age = numOrNull(link.last_frame_age_s);
    const stale = link.connected === false || (age !== null && age > timeout);
    card.classList.toggle("stale", !!stale);
    if (mark) mark.classList.toggle("hidden", !stale);
  }

  /* ------------------------------------------------------------------ 地图 fit / 目标校验 */
  function fitMapSoon() {
    const run = () => {
      try {
        mapView._resize();
        mapView.fit();
      } catch (error) { /* 画布尚未布局完成时忽略 */ }
    };
    requestAnimationFrame(run);
    setTimeout(run, 140);
  }

  function worldToCell(world) {
    const shape = state.mapShape;
    if (!shape) return null;
    const origin = shape.origin || {};
    const yaw = Number(origin.yaw_deg) || 0;
    const cos = Math.cos(-yaw * Math.PI / 180);
    const sin = Math.sin(-yaw * Math.PI / 180);
    const dx = world.x - Number(origin.x || 0);
    const dy = world.y - Number(origin.y || 0);
    const lx = dx * cos - dy * sin;
    const ly = dx * sin + dy * cos;
    const resolution = Number(shape.resolution) || 0.05;
    return {
      col: Math.floor(lx / resolution),
      row: Math.floor(ly / resolution),
      width: shape.width,
      height: shape.height,
    };
  }

  function validateGoal(world) {
    if (!state.mapCells || !state.mapShape) return null; // 无地图时交给后端 A* 校验
    const cell = worldToCell(world);
    if (!cell) return null;
    if (cell.col < 0 || cell.row < 0 || cell.col >= cell.width || cell.row >= cell.height) {
      return "目标点在地图范围外，请重新选择";
    }
    const value = state.mapCells[cell.row * cell.width + cell.col];
    const allowUnknown = !!state.snapshot?.nav_config?.allow_unknown;
    if (value < 0 && !allowUnknown) return "目标点位于未知区域（未建图），请重新选择";
    if (value >= 50) return "目标点位于障碍物/膨胀区内，请重新选择";
    return null;
  }

  /* ------------------------------------------------------------------ 确认弹层 */
  function confirmDialog(title, text, onOk) {
    const layer = $("confirm-layer");
    if (!layer) { onOk(); return; }
    state.confirmAction = onOk;
    setText("confirm-title", title);
    setText("confirm-text", text);
    layer.classList.remove("hidden");
  }

  function closeConfirm(ok) {
    const layer = $("confirm-layer");
    if (layer) layer.classList.add("hidden");
    const action = state.confirmAction;
    state.confirmAction = null;
    if (ok && action) action();
  }

  /* ------------------------------------------------------------------ 使能 / 姿态指令 */
  function doArm() {
    if (!canControl()) { toast("只读模式：没有操控权，无法使能", "warn"); return; }
    state.armPending = true;
    state.armIntent = "arm";
    state.armPendingSince = Date.now();
    setText("posture-result", "正在请求使能（机载进入自动模式并确认）…");
    if (!wsCommand("arm")) state.armPending = false;
    setTimeout(() => { if (state.armIntent === "arm") state.armPending = false; }, 9000);
  }

  function doDisarm() {
    if (!canControl()) { toast("只读模式：没有操控权，无法解除", "warn"); return; }
    confirmDialog("确认解除使能？",
      "解除后机载回到手动站立，所有速度指令被拦截。请确认机器狗周围安全、已有人扶稳。",
      () => {
        state.armPending = true;
        state.armIntent = "disarm";
        state.armPendingSince = Date.now();
        setText("posture-result", "正在请求解除使能（机载回手动站立）…");
        if (!wsCommand("disarm")) state.armPending = false;
        setTimeout(() => { if (state.armIntent === "disarm") state.armPending = false; }, 9000);
      });
  }

  function sendPosture(action) {
    if (!canControl()) { toast("只读模式：没有操控权，无法切换姿态", "warn"); return; }
    if (state.posturePending || ["pending", "running"].includes(state.snapshot?.posture?.state)) {
      toast("已有姿态切换正在执行，请等待机载确认", "warn");
      return;
    }
    state.posturePending = true;
    state.postureIntent = action;
    state.posturePendingSince = Date.now();
    state.postureBackendSince = state.snapshot?.posture?.since;
    setText("posture-result", action === "stand" ? "正在等待机载确认站立…" : "正在等待机载确认趴下…");
    if (!wsCommand("posture", { action })) state.posturePending = false;
  }

  function doPosture(action) {
    if (action === "stand") {
      confirmDialog("确认站立？",
        "机器人将起身站立（basic 目标 6）。请确认周围 1 m 内无人、无障碍并扶稳遥控器。",
        () => sendPosture("stand"));
    } else {
      sendPosture("lie");
    }
  }

  function cancelPosture() {
    if (!canControl()) { toast("只读模式：没有操控权", "warn"); return; }
    wsCommand("posture_cancel");
    setText("posture-result", "已请求取消；保持停车，等待机载动作结束确认");
  }
  function eventCategory(event) {
    const code = String(event.code || "");
    if (EVENT_CATEGORY[code]) return EVENT_CATEGORY[code];
    if (/estop/i.test(code)) return "safety";
    if (/operator|takeover|claim|release/i.test(code)) return "operator";
    if (/arm|disarm|posture/i.test(code)) return "control";
    if (/link|reconnect|ssh/i.test(code)) return "link";
    return "other";
  }

  function applyLogFilter() {
    const list = $("log-list");
    if (!list) return;
    let visible = 0;
    list.querySelectorAll(".log-row").forEach((row) => {
      const show = state.logFilter === "all" || row.dataset.cat === state.logFilter;
      row.classList.toggle("hidden-row", !show);
      if (show) visible += 1;
    });
    setText("log-count", state.logFilter === "all"
      ? `${state.logCount} 条` : `${visible} / ${state.logCount} 条`);
  }

  function appendLog(event) {
    renderLogRow(event, true);
  }

  function renderLogs(events) {
    const list = $("log-list");
    list.innerHTML = "";
    state.logCount = 0;
    for (const event of events.slice().reverse()) renderLogRow(event, false);
    applyLogFilter();
  }

  function renderLogRow(event, prepend) {
    const list = $("log-list");
    if (!list) return;
    const category = eventCategory(event);
    const evidence = EVENT_EVIDENCE.has(String(event.code || ""));
    const row = document.createElement("div");
    row.className = `log-row ${event.level || "info"}${evidence ? " evidence" : ""}`;
    row.dataset.cat = category;
    const time = document.createElement("span");
    time.className = "time";
    time.textContent = event.time || "";
    const level = document.createElement("span");
    level.className = "lvl";
    level.textContent = { info: "INFO", warn: "WARN", error: "ERR" }[event.level] || "INFO";
    const message = document.createElement("span");
    message.className = "msg";
    message.textContent = event.message || "";
    message.title = `${event.time || ""} ${event.message || ""}`.trim();
    row.append(time, level, message);
    if (prepend) list.prepend(row);
    else list.appendChild(row);
    state.logCount += 1;
    while (list.children.length > 400) list.removeChild(list.lastChild);
    applyLogFilter();
  }

  /* ------------------------------------------------------------------ 曲线 */
  function pushChartSample(output, frame) {
    const chart = state.chart;
    chart.cmdV.push(Number(output.linear || 0));
    chart.measV.push(frame ? Number((frame.velocity_body || [0])[0] || 0) : 0);
    chart.yaw.push(Number(output.angular || 0));
    const maxSamples = 700;
    if (chart.cmdV.length > maxSamples) {
      chart.cmdV.splice(0, chart.cmdV.length - maxSamples);
      chart.measV.splice(0, chart.measV.length - maxSamples);
      chart.yaw.splice(0, chart.yaw.length - maxSamples);
    }
  }

  function drawChart() {
    const canvas = $("chart-canvas");
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 10 || rect.height < 10) return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const w = rect.width;
    const h = rect.height;
    ctx.clearRect(0, 0, w, h);

    const chart = state.chart;
    const samples = chart.cmdV.length;
    const padLeft = 34;
    const padRight = 10;
    const padTop = 10;
    const padBottom = 18;
    const plotW = w - padLeft - padRight;
    const plotH = h - padTop - padBottom;

    // 速度范围（至少 0.1 以便观察小速度）
    let maxV = 0.1;
    for (const value of chart.cmdV.concat(chart.measV)) maxV = Math.max(maxV, Math.abs(value));
    maxV = Math.ceil(maxV * 10) / 10;
    let maxW = 0.4;
    for (const value of chart.yaw) maxW = Math.max(maxW, Math.abs(value));
    maxW = Math.ceil(maxW * 10) / 10;

    const yZero = padTop + plotH / 2;

    // 网格
    ctx.strokeStyle = "rgba(150,190,210,0.10)";
    ctx.fillStyle = "rgba(150,185,205,0.6)";
    ctx.font = "9px 'JetBrains Mono', Consolas, monospace";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i += 1) {
      const y = padTop + (plotH * i) / 4;
      ctx.beginPath();
      ctx.moveTo(padLeft, Math.round(y) + 0.5);
      ctx.lineTo(w - padRight, Math.round(y) + 0.5);
      ctx.stroke();
    }
    // 零线
    ctx.strokeStyle = "rgba(200,225,235,0.25)";
    ctx.beginPath();
    ctx.moveTo(padLeft, Math.round(yZero) + 0.5);
    ctx.lineTo(w - padRight, Math.round(yZero) + 0.5);
    ctx.stroke();

    ctx.fillStyle = "rgba(150,185,205,0.65)";
    ctx.fillText(`${maxV.toFixed(1)} m/s`, 2, padTop + 4);
    ctx.fillText("0", 2, yZero + 3);
    ctx.fillText(`-${maxV.toFixed(1)}`, 2, h - padBottom + 2);
    ctx.fillText("30s", w - padRight - 18, h - 4);

    const line = (data, scale, color, width) => {
      if (data.length < 2) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      for (let i = 0; i < data.length; i += 1) {
        const x = padLeft + (plotW * i) / Math.max(1, samples - 1);
        const y = yZero - (data[i] / scale) * (plotH / 2);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    };

    line(chart.yaw.map((value) => value / maxW * maxV * 0.7), maxV, "rgba(255,184,77,0.55)", 1.2);
    line(chart.cmdV, maxV, "rgba(55,219,229,0.9)", 1.6);
    line(chart.measV, maxV, "rgba(53,224,161,0.85)", 1.2);
  }

  /* ------------------------------------------------------------------ 遥测指令 */
  function activeDirection() {
    const has = (name) => state.keys.has(name) || state.pad.has(name);
    return {
      forward: has("forward"),
      back: has("back"),
      left: has("left"),
      right: has("right"),
      stop: has("stop"),
    };
  }

  function teleopTick() {
    if (!state.connected) return;
    if (!canControl() || state.posturePending) {
      if (state.movingSent) {
        wsSend({ type: "teleop", linear: 0, angular: 0, deadman: false });
        state.movingSent = false;
      }
      state.keys.clear();
      state.pad.clear();
      return;
    }
    if (state.estop) {
      if (state.movingSent) {
        wsSend({ type: "teleop", linear: 0, angular: 0, deadman: false });
        state.movingSent = false;
      }
      return;
    }
    const direction = activeDirection();
    if (direction.stop) {
      sendStop("操作者立即停车");
      return;
    }
    const anyMove = direction.forward || direction.back || direction.left || direction.right;
    if (!anyMove) {
      if (state.movingSent) {
        wsSend({ type: "teleop", linear: 0, angular: 0, deadman: false });
        state.movingSent = false;
      }
      return;
    }
    const limits = state.limits || {};
    const maxLinear = Number(limits.teleop_linear_m_s ?? 0.10);
    const maxAngular = Number(limits.teleop_angular_rad_s ?? 0.35);
    const maxReverse = Math.min(Number(limits.max_reverse_m_s ?? 0.08), maxLinear);
    let linear = 0;
    if (direction.forward && !direction.back) linear = maxLinear;
    else if (direction.back && !direction.forward) linear = -maxReverse;
    let angular = 0;
    if (direction.left && !direction.right) angular = maxAngular;
    else if (direction.right && !direction.left) angular = -maxAngular;
    wsSend({ type: "teleop", linear, angular, deadman: true });
    state.movingSent = Boolean(linear || angular);
  }

  function sendStop(reason) {
    if (state.readOnly) {
      toast("只读接收模式：不发送停车指令，请使用物理遥控器/急停", "warn");
      return;
    }
    state.keys.clear();
    state.pad.clear();
    document.querySelectorAll(".dpad-btn.active").forEach((node) => node.classList.remove("active"));
    wsSend({ type: "stop" });
    wsSend({ type: "teleop", linear: 0, angular: 0, deadman: false });
    state.movingSent = false;
    if (reason) toast(reason, "warn");
  }

  function resetHoldMs() {
    const configured = Number(state.snapshot?.safety_config?.estop_reset_hold_s
      ?? state.config?.safety?.estop_reset_hold_s);
    return Number.isFinite(configured) && configured > 0
      ? Math.max(300, configured * 1000) : HOLD_RESET_MS;
  }

  function engageEstop() {
    if (state.readOnly) {
      toast("只读接收模式：不发送急停指令，请立即使用物理急停/遥控器", "error");
      return;
    }
    if (state.estop) {
      toast("急停已锁存，请长按「复位急停」", "warn");
      return;
    }
    state.estopUnconfirmed = false;
    state.estopPendingAt = performance.now();
    setText("estop-pill-text", "确认中…");
    const sent = wsSend({ type: "estop", engaged: true, reason: "界面急停按钮" });
    if (!sent) {
      toast("急停指令发送失败（与上位机服务断开），请立即使用物理急停/遥控器！", "error");
      // REST 兜底：不用 fetchJson，避免“成功”toast 顶掉红色报警
      fetch("/api/estop", {
        method: "POST",
        headers: authHeaders({ "Content-Type": "application/json" }),
        body: JSON.stringify({ engaged: true, reason: "界面急停按钮" }),
      }).catch(() => { /* 网络也不可用时保留红色报警 */ });
    } else {
      toast("急停指令已发送，等待 1.5 s 内状态确认…", "warn");
    }
    if (state.estopConfirmTimer) clearTimeout(state.estopConfirmTimer);
    state.estopConfirmTimer = setTimeout(() => {
      state.estopConfirmTimer = null;
      if (!state.estop) {
        state.estopUnconfirmed = true;
        setText("estop-pill-text", "未确认");
        toast("急停指令未在 1.5 s 内确认，请立即使用物理急停/遥控器！", "error");
      }
    }, ESTOP_CONFIRM_MS);
  }

  function startResetHold(event) {
    if (event) event.preventDefault();
    if (!state.estop || state.resetInterval) return;
    const holdMs = resetHoldMs();
    state.resetStart = performance.now();
    const fill = $("hold-fill");
    state.resetInterval = setInterval(() => {
      const progress = Math.min(1, (performance.now() - state.resetStart) / holdMs);
      if (fill) fill.style.width = `${progress * 100}%`;
      if (progress >= 1) {
        stopResetHold();
        const hold_s = Number(((performance.now() - state.resetStart) / 1000).toFixed(2));
        wsCommand("estop_reset", { hold_s });
        toast(`已提交复位（长按 ${hold_s.toFixed(1)} s），需重新按住方向键运动`, "ok");
      }
    }, 30);
  }

  function stopResetHold() {
    if (state.resetInterval) clearInterval(state.resetInterval);
    state.resetInterval = null;
    const fill = $("hold-fill");
    if (fill) fill.style.width = "0%";
  }

  /* ------------------------------------------------------------------ 限幅与对齐 */
  function applyLimitsToUi() {
    const limits = state.limits;
    if (!limits) return;
    const linearSlider = $("limit-linear");
    const angularSlider = $("limit-angular");
    linearSlider.max = String(Number(limits.max_forward_m_s || 0.30));
    angularSlider.max = String(Number(limits.max_yaw_rad_s || 0.80));
    if (document.activeElement !== linearSlider) {
      linearSlider.value = String(Number(limits.teleop_linear_m_s ?? 0.10));
    }
    if (document.activeElement !== angularSlider) {
      angularSlider.value = String(Number(limits.teleop_angular_rad_s ?? 0.35));
    }
    setText("limit-linear-out", `${Number(linearSlider.value).toFixed(2)} m/s`);
    setText("limit-angular-out", `${Number(angularSlider.value).toFixed(2)} rad/s`);
  }

  function onLimitInput() {
    setText("limit-linear-out", `${Number($("limit-linear").value).toFixed(2)} m/s`);
    setText("limit-angular-out", `${Number($("limit-angular").value).toFixed(2)} rad/s`);
    clearTimeout(state.limitTimer);
    state.limitTimer = setTimeout(() => {
      fetchJson("/api/limits", {
        teleop_linear_m_s: Number($("limit-linear").value),
        teleop_angular_rad_s: Number($("limit-angular").value),
      });
    }, 220);
  }

  /* ------------------------------------------------------------------ 界面绑定 */
  function bind() {
    $("btn-connect").addEventListener("click", doConnect);
    $("btn-disconnect").addEventListener("click", doDisconnect);
    $("link-kind").addEventListener("change", (event) => renderLinkOptions(event.target.value));

    $("estop-btn").addEventListener("click", engageEstop);
    $("btn-stop").addEventListener("click", () => sendStop("已发送零速指令"));
    $("btn-estop-reset").addEventListener("pointerdown", startResetHold);
    $("btn-estop-reset").addEventListener("pointerup", stopResetHold);
    $("btn-estop-reset").addEventListener("pointerleave", stopResetHold);
    $("btn-estop-reset").addEventListener("pointercancel", stopResetHold);

    // ---- 操作权 ----
    $("control-pill").addEventListener("click", () => {
      const pop = $("operator-pop");
      if (!pop) return;
      pop.classList.toggle("hidden");
    });
    $("operator-close").addEventListener("click", () => $("operator-pop").classList.add("hidden"));
    $("op-claim").addEventListener("click", () => requestOperator("claim"));
    $("op-takeover").addEventListener("click", () => requestOperator("takeover"));
    $("op-release").addEventListener("click", () => requestOperator("release"));
    document.addEventListener("pointerdown", (event) => {
      const pop = $("operator-pop");
      const pill = $("control-pill");
      if (!pop || pop.classList.contains("hidden")) return;
      if (pop.contains(event.target) || (pill && pill.contains(event.target))) return;
      pop.classList.add("hidden");
    });

    // ---- 使能 / 姿态 / 确认弹层 ----
    $("btn-arm").addEventListener("click", doArm);
    $("btn-disarm").addEventListener("click", doDisarm);
    $("btn-stand").addEventListener("click", () => doPosture("stand"));
    $("btn-lie").addEventListener("click", () => doPosture("lie"));
    $("btn-posture-cancel").addEventListener("click", cancelPosture);
    $("confirm-ok").addEventListener("click", () => closeConfirm(true));
    $("confirm-cancel").addEventListener("click", () => closeConfirm(false));
    $("confirm-layer").addEventListener("click", (event) => {
      if (event.target === $("confirm-layer")) closeConfirm(false);
    });

    // ---- 断线重连 ----
    $("btn-link-reconnect").addEventListener("click", async () => {
      toast("正在请求重新连接链路…", "warn");
      await doConnect();
    });

    // ---- 令牌 ----
    $("btn-token-save").addEventListener("click", () => {
      const value = ($("token-input").value || "").trim();
      persistToken(value);
      toast(value ? "令牌已保存，正在重新连接服务…" : "令牌已清空，正在重新连接服务…", "ok");
      connectWs();
    });
    $("btn-token-clear").addEventListener("click", () => {
      persistToken("");
      toast("令牌已清除，正在重新连接服务…", "ok");
      connectWs();
    });

    // ---- 日志筛选 ----
    document.querySelectorAll("#log-filters button").forEach((button) => {
      button.addEventListener("click", () => {
        state.logFilter = button.dataset.filter || "all";
        document.querySelectorAll("#log-filters button").forEach((node) => {
          node.classList.toggle("active", node === button);
        });
        applyLogFilter();
      });
    });

    $("limit-linear").addEventListener("input", onLimitInput);
    $("limit-angular").addEventListener("input", onLimitInput);

    $("btn-align").addEventListener("click", () => fetchJson("/api/map/align", {
      x: Number($("align-x").value || 0),
      y: Number($("align-y").value || 0),
      yaw_deg: Number($("align-yaw").value || 0),
    }));

    $("btn-record").addEventListener("click", () => {
      const active = $("btn-record").dataset.active === "1";
      fetchJson(active ? "/api/record/stop" : "/api/record/start", {});
    });
    $("btn-download").addEventListener("click", () => {
      window.location.href = "/api/record/download";
    });

    $("tool-goal").addEventListener("click", () => {
      state.goalPick = !state.goalPick;
      mapView.setPickMode(state.goalPick);
      $("tool-goal").classList.toggle("active", state.goalPick);
      updateGoalUi();
      toast(state.goalPick ? "点选目标已开启：左键点击地图设置目标" : "点选目标已关闭", "ok");
    });
    $("tool-clear").addEventListener("click", () => {
      wsSend({ type: "clear_trail" });
      mapView.setTrail([]);
    });
    $("tool-fit").addEventListener("click", () => mapView.fit());
    $("tool-in").addEventListener("click", () => mapView.zoomBy(1.35));
    $("tool-out").addEventListener("click", () => mapView.zoomBy(1 / 1.35));
    $("tool-upload").addEventListener("click", () => $("map-file").click());
    $("tool-demo").addEventListener("click", () => fetchJson("/api/map/demo", {}));
    $("map-load").addEventListener("click", loadSelectedMap);
    $("mapping-start").addEventListener("click", () => requestMapping("mapping_start"));
    $("mapping-stop").addEventListener("click", () => requestMapping("mapping_stop"));
    $("map-save").addEventListener("click", () => requestMapping("map_save"));
    $("exploration-start").addEventListener("click", () => requestExploration("exploration_start"));
    $("exploration-stop").addEventListener("click", () => requestExploration("exploration_stop"));
    $("map-select").addEventListener("change", () => {
      const option = $("map-select").selectedOptions[0];
      if (option && option.value) toast(`已选择 ${option.textContent}，点「加载」生效`, "ok");
    });
    $("map-file").addEventListener("change", async (event) => {
      const file = event.target.files && event.target.files[0];
      if (!file) return;
      const form = new FormData();
      form.append("file", file);
      try {
        const response = await fetch("/api/map/upload", {
          method: "POST", headers: authHeaders(), body: form,
        });
        const data = await response.json().catch(() => ({ ok: false, message: "地图上传失败" }));
        if (!data.ok) toast(data.message || "地图上传失败", "error");
        else toast(data.message || "地图已加载", "ok");
      } catch (error) {
        toast(`地图上传失败：${error}`, "error");
      }
      event.target.value = "";
    });
    $("opt-trail").addEventListener("change", (event) => mapView.setOptions({ trail: event.target.checked }));
    $("opt-scan").addEventListener("change", (event) => mapView.setOptions({ scan: event.target.checked }));
    $("opt-grid").addEventListener("change", (event) => mapView.setOptions({ grid: event.target.checked }));

    $("btn-go").addEventListener("click", goToGoal);
    $("btn-nav-cancel").addEventListener("click", cancelNav);

    $("btn-sim-joystick").addEventListener("click", () => {
      state.simJoystick = !state.simJoystick;
      wsSend({
        type: "simulate",
        joystick: state.simJoystick ? [0.8, 0.1, 0] : [0, 0, 0],
      });
      $("btn-sim-joystick").classList.toggle("warn", state.simJoystick);
      toast(state.simJoystick ? "已模拟厂商遥控接管" : "已释放模拟遥控", "warn");
    });
    $("btn-sim-link").addEventListener("click", () => {
      state.simLink = !state.simLink;
      wsSend({ type: "simulate", link: !state.simLink });
      $("btn-sim-link").classList.toggle("warn", state.simLink);
      toast(state.simLink ? "已模拟断链" : "已恢复链路", "warn");
    });

    // 方向键按住操控
    document.querySelectorAll(".dpad-btn").forEach((button) => {
      const move = button.dataset.move;
      const press = (event) => {
        event.preventDefault();
        button.classList.add("active");
        if (move === "stop") {
          sendStop("已发送零速指令");
          return;
        }
        state.pad.add(move);
      };
      const release = () => {
        button.classList.remove("active");
        state.pad.delete(move);
      };
      button.addEventListener("pointerdown", press);
      button.addEventListener("pointerup", release);
      button.addEventListener("pointerleave", release);
      button.addEventListener("pointercancel", release);
      button.addEventListener("contextmenu", (event) => event.preventDefault());
    });

    // 键盘
    const keyMap = {
      KeyW: "forward", ArrowUp: "forward",
      KeyS: "back", ArrowDown: "back",
      KeyA: "left", ArrowLeft: "left",
      KeyD: "right", ArrowRight: "right",
    };
    window.addEventListener("keydown", (event) => {
      const tag = (event.target.tagName || "").toUpperCase();
      if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
      if (event.code === "Space") {
        event.preventDefault();
        sendStop("已发送零速指令");
        return;
      }
      if (event.code === "Escape") {
        event.preventDefault();
        engageEstop();
        return;
      }
      const direction = keyMap[event.code];
      if (direction) {
        event.preventDefault();
        if (!event.repeat) state.keys.add(direction);
        document.querySelector(`.dpad-btn[data-move="${direction}"]`)
          ?.classList.add("active");
      }
    });
    window.addEventListener("keyup", (event) => {
      const direction = keyMap[event.code];
      if (direction) {
        state.keys.delete(direction);
        if (!state.pad.has(direction)) {
          document.querySelector(`.dpad-btn[data-move="${direction}"]`)
            ?.classList.remove("active");
        }
      }
    });
    window.addEventListener("blur", () => {
      state.keys.clear();
      state.pad.clear();
      document.querySelectorAll(".dpad-btn.active").forEach((node) => node.classList.remove("active"));
      if (state.movingSent) {
        wsSend({ type: "teleop", linear: 0, angular: 0, deadman: false });
        state.movingSent = false;
      }
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) {
        state.keys.clear();
        state.pad.clear();
        wsSend({ type: "teleop", linear: 0, angular: 0, deadman: false });
        state.movingSent = false;
      }
    });
  }

  /* ------------------------------------------------------------------ 周期任务 */
  function tickClock() {
    const now = new Date();
    setText("clock-text", now.toLocaleTimeString("zh-CN", { hour12: false }));
    // 比例尺
    const scale = mapView.view.scale;
    const options = [0.5, 1, 2, 5, 10, 20];
    let meters = options[options.length - 1];
    for (const candidate of options) {
      if (candidate * scale >= 70) { meters = candidate; break; }
    }
    const widthPx = Math.round(meters * scale);
    const line = $("scale-line");
    if (line) line.style.width = `${Math.max(24, Math.min(160, widthPx))}px`;
    setText("scale-label", `${meters} m`);
  }

  function tickConnectionHealth() {
    const socket = state.ws;
    if (!socket) return;
    const now = Date.now();
    if (socket.readyState === WebSocket.CONNECTING &&
        now - state.socketStartedAt > WS_CONNECT_TIMEOUT_MS) {
      reconnectStalledSocket(socket, "连接超时，正在重连");
      return;
    }
    if (state.connected) {
      const stateAge = now - (state.lastStateAt || state.socketStartedAt);
      if (stateAge > WS_STATE_TIMEOUT_MS) {
        reconnectStalledSocket(socket, "状态反馈超时，正在重连");
        return;
      }
      toggle($("conn-overlay"), "hidden", stateAge <= 3000);
    }
  }

  // eslint-disable-next-line no-unused-vars
  function onWorkspaceChange(name) {
    if (name === "drive") {
      fitMapSoon();
      updateGoalUi();
    } else {
      state.keys.clear();
      state.pad.clear();
      state.movingSent = false;
    }
  }

  /* ------------------------------------------------------------------ 启动 */
  function boot() {
    window.__ui = {
      state,
      mapView,
      authHeaders,
      canControl,
      movementLocked,
      fitMapSoon,
      onWorkspaceChange,
      renderExploration,
      getToken: () => state.token,
      // 只读验收钩子：模拟/恢复操作权字段，便于 CDP 验证“他人/我”两种状态
      setOperatorForTest: (operator) => {
        if (state.operatorTestPrevContract === undefined) {
          state.operatorTestPrevContract = state.contractMode;
        }
        state.contractMode = true;
        state.operatorTestOverride = operator === undefined ? null : operator;
        renderOperator();
      },
      resetOperatorTest: () => {
        state.operatorTestOverride = null;
        if (state.operatorTestPrevContract !== undefined) {
          state.contractMode = state.operatorTestPrevContract;
          state.operatorTestPrevContract = undefined;
        }
        renderOperator();
      },
    };
    // 令牌：URL ?token= 优先，其次 localStorage.host_ui_token
    state.token = readTokenFromUrl() || loadStoredToken();
    persistToken(state.token);
    bind();
    mapView.setPickMode(true);
    connectWs();
    setInterval(teleopTick, 50);
    setInterval(drawChart, 100);
    setInterval(tickClock, 500);
    setInterval(tickConnectionHealth, 1000);
    setInterval(() => {
      if (state.connected && state.movingSent === false && state.ws?.readyState === WebSocket.OPEN) {
        // 心跳保活由服务端看门狗保证，这里不发多余数据
      }
    }, 1000);
    window.addEventListener("beforeunload", () => {
      clearTimeout(state.reconnectTimer);
      stopPing();
      if (state.ws) {
        try {
          state.ws.send(JSON.stringify({ type: "teleop", linear: 0, angular: 0, deadman: false }));
          state.ws.send(JSON.stringify({ type: "stop" }));
          state.ws.onclose = null;
          state.ws.close(1000, "页面关闭");
        } catch (error) { /* ignore */ }
      }
    });
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", boot);
  } else {
    boot();
  }
})();
