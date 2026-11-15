(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  let sensors = {};
  let receivedAt = 0;
  const captions = {
    sensors: "彩色相机 · 深度相机 · 雷达点云",
    drive: "地图导航 · 手动操控",
    settings: "通信连接 · 速度限幅 · 地图对齐 · 令牌",
    diagnostics: "事件记录（操控权/急停/使能/姿态/重连） · 速度遥测",
  };

  function authHeaders() {
    try {
      return window.__ui && window.__ui.authHeaders ? window.__ui.authHeaders() : {};
    } catch (error) {
      return {};
    }
  }

  /* ------------------------------------------------------------ 工作区切换 */
  function switchWorkspace(name) {
    const ui = window.__ui;
    if (ui && ui.state) {
      ui.state.keys.clear();
      ui.state.pad.clear();
      if (ui.state.ws && ui.state.ws.readyState === WebSocket.OPEN) {
        ui.state.ws.send(JSON.stringify({ type: "teleop", linear: 0, angular: 0, deadman: false }));
      }
    }
    document.body.dataset.workspace = name;
    document.querySelectorAll(".workspace-tabs button").forEach((tab) => {
      tab.setAttribute("aria-selected", String(tab.dataset.workspace === name));
    });
    const context = $("workspace-context");
    if (context) context.textContent = captions[name] || "";
    // 地图在隐藏的工作区里 fit 会被钳到最小缩放，切换后必须重新适配
    if (ui && typeof ui.onWorkspaceChange === "function") ui.onWorkspaceChange(name);
    if (name === "sensors") setTimeout(poll, 0);
    window.dispatchEvent(new Event("resize"));
    drawLidar();
  }

  document.querySelectorAll(".workspace-tabs button").forEach((button) => {
    button.addEventListener("click", () => switchWorkspace(button.dataset.workspace));
  });

  // 只有“导航操控”工作区才响应移动键；姿态/使能进行中或只读时同样屏蔽
  window.addEventListener("keydown", (event) => {
    const key = String(event.key || "").toLowerCase();
    const movementKeys = ["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"];
    if (!movementKeys.includes(key)) return;
    const ui = window.__ui;
    const locked = ui && typeof ui.movementLocked === "function" ? ui.movementLocked() : false;
    if (locked || document.body.dataset.workspace !== "drive") {
      event.stopImmediatePropagation();
    }
  }, true);

  $("camera-fullscreen").addEventListener("click", () => {
    const panel = document.querySelector(".camera-card");
    const action = document.fullscreenElement ? document.exitFullscreen() : panel.requestFullscreen();
    action.catch(() => { $("color-detail").textContent = "浏览器未允许全屏"; });
  });

  /* ------------------------------------------------------------ 传感器渲染 */
  function updateSensor(name) {
    const data = sensors[name];
    const age = data ? data.age_s + (performance.now() - receivedAt) / 1000 : Infinity;
    const fresh = age < 2;
    const badge = $(name + "-state");
    badge.textContent = fresh ? `实时 · ${Math.round(age * 1000)} ms` : data ? "数据超时" : "等待数据";
    badge.className = "badge " + (fresh ? "ok" : "warn");
    const empty = $(name + "-empty");
    empty.hidden = fresh;
    empty.textContent = data ? "数据超时，请检查机载驱动与连接" : name === "lidar" ? "等待 /rslidar_points" : "等待机载图像";
    if (name === "lidar") {
      $("lidar-detail").textContent = data ? `${data.valid.toLocaleString()} 有效点 / 帧` : "--";
      return;
    }
    const img = $(name + "-image");
    img.classList.toggle("stale", !fresh);
    if (data?.jpeg) {
      const source = "data:image/jpeg;base64," + data.jpeg;
      if (img.getAttribute("src") !== source) img.src = source;
      img.hidden = false;
      $(name + "-detail").textContent = name === "depth"
        ? `有效 ${data.valid_percent}% · 中值 ${data.median_m ?? "--"} m`
        : `${data.width} × ${data.height} · 预览 ≤ 2 fps`;
    }
  }

  function drawLidar() {
    const canvas = $("lidar-canvas");
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width || !height) return;
    const dpr = Math.min(devicePixelRatio || 1, 2);
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);
    const radius = Number($("lidar-range").value);
    const scale = Math.min(width, height) / (2 * radius + 2);
    ctx.strokeStyle = "#20313c";
    for (let distance = radius / 4; distance <= radius; distance += radius / 4) {
      ctx.beginPath(); ctx.arc(width / 2, height / 2, distance * scale, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.fillStyle = "#728a98"; ctx.font = "11px monospace";
    ctx.fillText(`+X ↑    +Y ←    ${radius} m`, 12, 20);
    const data = sensors.lidar;
    if (data && data.age_s + (performance.now() - receivedAt) / 1000 < 2) {
      for (const [x, y, z] of data.points) {
        ctx.fillStyle = z > .5 ? "#ffbf68" : "#42d6cf";
        ctx.fillRect(width / 2 - y * scale, height / 2 - x * scale, 2, 2);
      }
    }
    ctx.fillStyle = "#ffffff"; ctx.fillRect(width / 2 - 3, height / 2 - 3, 6, 6);
  }
  $("lidar-range").addEventListener("change", drawLidar);
  new ResizeObserver(drawLidar).observe($("lidar-canvas").parentElement);

  async function poll() {
    const active = document.body.dataset.workspace === "sensors" && !document.hidden;
    if (active) {
      try {
        const response = await fetch("/api/sensors", { cache: "no-store", headers: authHeaders() });
        if (!response.ok) throw new Error("sensor request failed");
        sensors = await response.json();
        receivedAt = performance.now();
      } catch (error) { /* 保留旧帧，超时后自动变灰 */ }
    }
    ["color", "depth", "lidar"].forEach(updateSensor);
    drawLidar();
    // 只在“传感器”工作区且页面可见时拉流，避免后台/其他工作区白耗 SSH 带宽
    setTimeout(poll, active ? 500 : 1500);
  }
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden && document.body.dataset.workspace === "sensors") {
      setTimeout(poll, 0);
    }
  });
  poll();
})();