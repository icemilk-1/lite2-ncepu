/* ==========================================================================
   地图视图：占据栅格渲染 + 机器人位姿 + 轨迹 / 路径 / 目标 / 扫描点
   纯 Canvas 2D，无第三方依赖。
   ========================================================================== */
(function (global) {
  "use strict";

  const BODY_LENGTH_M = 0.62;
  const BODY_WIDTH_M = 0.45;

  function normalizeAngle(deg) {
    let d = deg % 360;
    if (d > 180) d -= 360;
    if (d <= -180) d += 360;
    return d;
  }

  function decodeCells(base64) {
    const binary = atob(base64 || "");
    const cells = new Int8Array(binary.length);
    for (let i = 0; i < binary.length; i += 1) {
      const value = binary.charCodeAt(i);
      cells[i] = value > 127 ? value - 256 : value;
    }
    return cells;
  }

  class MapView {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.dpr = Math.min(2, global.devicePixelRatio || 1);
      this.map = null;
      this.view = { x: 0, y: 0, scale: 24 };
      this.pose = null;
      this.targetPose = null;
      this.trail = [];
      this.scan = [];
      this.path = [];
      this.frontiers = [];
      this.goal = null;
      this.navActive = false;
      this.options = { trail: true, scan: true, grid: true };
      this.cursor = null;
      this.pickMode = true;
      this.drag = null;
      this.width = 0;
      this.height = 0;
      this.onPick = null;
      this.onCursor = null;
      this._time = 0;
      this._raf = null;
      this._bind();
      this._resize();
      if (typeof ResizeObserver !== "undefined") {
        this._observer = new ResizeObserver(() => this._resize());
        this._observer.observe(canvas.parentElement || canvas);
      } else {
        global.addEventListener("resize", () => this._resize());
      }
      this._loop();
    }

    /* ------------------------------------------------------------ 事件绑定 */
    _bind() {
      const canvas = this.canvas;
      canvas.addEventListener("pointerdown", (event) => {
        if (event.button !== 0 && event.button !== 1) return;
        canvas.setPointerCapture(event.pointerId);
        this.drag = {
          id: event.pointerId,
          startX: event.offsetX,
          startY: event.offsetY,
          x: event.offsetX,
          y: event.offsetY,
          moved: false,
          button: event.button,
        };
        canvas.classList.add("dragging");
      });

      canvas.addEventListener("pointermove", (event) => {
        this.cursor = { x: event.offsetX, y: event.offsetY, world: this.screenToWorld(event.offsetX, event.offsetY) };
        if (this.onCursor) this.onCursor(this.cursor.world);
        if (!this.drag || this.drag.id !== event.pointerId) return;
        const dx = event.offsetX - this.drag.x;
        const dy = event.offsetY - this.drag.y;
        if (Math.abs(event.offsetX - this.drag.startX) > 3 ||
            Math.abs(event.offsetY - this.drag.startY) > 3) {
          this.drag.moved = true;
        }
        if (this.drag.moved) {
          this.view.x -= dx / this.view.scale;
          this.view.y += dy / this.view.scale;
        }
        this.drag.x = event.offsetX;
        this.drag.y = event.offsetY;
      });

      const endDrag = (event) => {
        if (!this.drag || this.drag.id !== event.pointerId) return;
        const wasMoved = this.drag.moved;
        const button = this.drag.button;
        this.drag = null;
        canvas.classList.remove("dragging");
        if (!wasMoved && button === 0 && this.pickMode && this.onPick) {
          const world = this.screenToWorld(event.offsetX, event.offsetY);
          this.onPick(world);
        }
      };
      canvas.addEventListener("pointerup", endDrag);
      canvas.addEventListener("pointercancel", () => {
        this.drag = null;
        canvas.classList.remove("dragging");
      });
      canvas.addEventListener("pointerleave", () => {
        this.cursor = null;
        if (this.onCursor) this.onCursor(null);
      });

      canvas.addEventListener("wheel", (event) => {
        event.preventDefault();
        const factor = Math.exp(-event.deltaY * 0.0014);
        this.zoomAt(event.offsetX, event.offsetY, factor);
      }, { passive: false });

      canvas.addEventListener("dblclick", () => this.fit());
      canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    }

    /* ------------------------------------------------------------ 尺寸 */
    _resize() {
      const rect = this.canvas.getBoundingClientRect();
      this.width = Math.max(1, Math.round(rect.width));
      this.height = Math.max(1, Math.round(rect.height));
      this.canvas.width = Math.round(this.width * this.dpr);
      this.canvas.height = Math.round(this.height * this.dpr);
    }

    /* ------------------------------------------------------------ 坐标变换 */
    worldToScreen(x, y) {
      return {
        x: this.width / 2 + (x - this.view.x) * this.view.scale,
        y: this.height / 2 - (y - this.view.y) * this.view.scale,
      };
    }

    screenToWorld(sx, sy) {
      return {
        x: this.view.x + (sx - this.width / 2) / this.view.scale,
        y: this.view.y - (sy - this.height / 2) / this.view.scale,
      };
    }

    zoomAt(sx, sy, factor) {
      const before = this.screenToWorld(sx, sy);
      this.view.scale = Math.min(400, Math.max(2.5, this.view.scale * factor));
      const after = this.screenToWorld(sx, sy);
      this.view.x += before.x - after.x;
      this.view.y += before.y - after.y;
    }

    zoomBy(factor) {
      this.zoomAt(this.width / 2, this.height / 2, factor);
    }

    /* ------------------------------------------------------------ 数据设置 */
    setMap(payload) {
      if (!payload || !payload.width) {
        this.map = null;
        return;
      }
      const width = payload.width;
      const height = payload.height;
      const resolution = payload.resolution;
      const cells = decodeCells(payload.data);
      const offscreen = document.createElement("canvas");
      offscreen.width = width;
      offscreen.height = height;
      const octx = offscreen.getContext("2d");
      const image = octx.createImageData(width, height);
      const pixels = image.data;

      for (let row = 0; row < height; row += 1) {
        const imageRow = height - 1 - row;
        for (let col = 0; col < width; col += 1) {
          const value = cells[row * width + col];
          const index = (imageRow * width + col) * 4;
          let r;
          let g;
          let b;
          if (value < 0) {
            r = 10; g = 16; b = 22;
          } else if (value < 50) {
            // 自由区域：随占据概率略微提亮
            const t = value / 50;
            r = 22 + 12 * t; g = 34 + 16 * t; b = 44 + 18 * t;
          } else {
            const t = Math.min(1, (value - 50) / 50);
            r = 72 + 26 * t; g = 94 + 26 * t; b = 108 + 24 * t;
          }
          // 轻微噪点，避免大片纯色
          const noise = ((col * 7 + row * 13) % 5) - 2;
          pixels[index] = Math.max(0, Math.min(255, r + noise));
          pixels[index + 1] = Math.max(0, Math.min(255, g + noise));
          pixels[index + 2] = Math.max(0, Math.min(255, b + noise));
          pixels[index + 3] = 255;
        }
      }
      octx.putImageData(image, 0, 0);
      const origin = payload.origin || { x: 0, y: 0, yaw_deg: 0 };
      this.map = {
        width,
        height,
        resolution,
        origin: {
          x: Number(origin.x || 0),
          y: Number(origin.y || 0),
          yaw: Number(origin.yaw_deg || 0) * Math.PI / 180,
        },
        name: payload.name || "map",
        image: offscreen,
      };
      this.fit();
    }

    hasMap() {
      return !!this.map;
    }

    fit() {
      if (!this.map) {
        this.view = { x: 0, y: 0, scale: 24 };
        return;
      }
      const { width, height, resolution, origin } = this.map;
      const halfW = width * resolution / 2;
      const halfH = height * resolution / 2;
      const cos = Math.cos(origin.yaw);
      const sin = Math.sin(origin.yaw);
      let minX = Infinity;
      let maxX = -Infinity;
      let minY = Infinity;
      let maxY = -Infinity;
      for (const [lx, ly] of [[0, 0], [width * resolution, 0],
        [0, height * resolution], [width * resolution, height * resolution]]) {
        const wx = origin.x + cos * lx - sin * ly;
        const wy = origin.y + sin * lx + cos * ly;
        minX = Math.min(minX, wx); maxX = Math.max(maxX, wx);
        minY = Math.min(minY, wy); maxY = Math.max(maxY, wy);
      }
      const spanX = Math.max(maxX - minX, 0.5);
      const spanY = Math.max(maxY - minY, 0.5);
      const scale = Math.min(this.width / spanX, this.height / spanY) * 0.9;
      this.view.scale = Math.min(400, Math.max(2.5, scale));
      this.view.x = (minX + maxX) / 2;
      this.view.y = (minY + maxY) / 2;
    }

    setPose(pose) {
      if (!pose) return;
      this.targetPose = { x: pose.x, y: pose.y, yaw: pose.yaw_deg };
      if (!this.pose) this.pose = { ...this.targetPose };
    }

    setTrail(points) {
      this.trail = Array.isArray(points) ? points.map((p) => [p[0], p[1]]) : [];
    }

    appendTrail(points) {
      if (!Array.isArray(points)) return;
      for (const point of points) this.trail.push([point[0], point[1]]);
      if (this.trail.length > 8000) this.trail.splice(0, this.trail.length - 8000);
    }

    setScan(points) {
      this.scan = Array.isArray(points) ? points : [];
    }

    setPath(path) {
      this.path = Array.isArray(path) ? path : [];
    }

    /* 自主探索前沿点（世界坐标 [[x,y],...]），只在 active/paused 时由 app.js 传入 */
    setFrontiers(points) {
      if (!Array.isArray(points)) {
        this.frontiers = [];
        return;
      }
      const cleaned = [];
      for (const point of points) {
        if (!Array.isArray(point) || point.length < 2) continue;
        const x = Number(point[0]);
        const y = Number(point[1]);
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        cleaned.push([x, y]);
        if (cleaned.length >= 200) break;
      }
      this.frontiers = cleaned;
    }

    setGoal(goal) {
      this.goal = goal ? { x: goal.x, y: goal.y, yaw: goal.yaw_deg } : null;
    }

    setNavActive(active) {
      this.navActive = !!active;
    }

    setPickMode(enabled) {
      this.pickMode = !!enabled;
      this.canvas.classList.toggle("picking", this.pickMode);
    }

    setOptions(options) {
      Object.assign(this.options, options || {});
    }

    /* ------------------------------------------------------------ 渲染 */
    _loop() {
      const render = (timestamp) => {
        this._time = timestamp || 0;
        this._smoothPose();
        this._render();
        this._raf = global.requestAnimationFrame(render);
      };
      this._raf = global.requestAnimationFrame(render);
    }

    _smoothPose() {
      if (!this.targetPose) return;
      if (!this.pose) {
        this.pose = { ...this.targetPose };
        return;
      }
      const k = 0.22;
      this.pose.x += (this.targetPose.x - this.pose.x) * k;
      this.pose.y += (this.targetPose.y - this.pose.y) * k;
      const dyaw = normalizeAngle(this.targetPose.yaw - this.pose.yaw);
      this.pose.yaw += dyaw * k;
    }

    _render() {
      const ctx = this.ctx;
      const w = this.width;
      const h = this.height;
      ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#070d12";
      ctx.fillRect(0, 0, w, h);

      if (!this.map) {
        this._drawNoMap();
        return;
      }
      this._drawMapImage();
      if (this.options.grid) this._drawGrid();
      if (this.options.scan && this.scan.length) this._drawScan();
      if (this.options.trail && this.trail.length > 1) this._drawTrail();
      if (this.path.length > 1) this._drawPath();
      if (this.frontiers.length) this._drawFrontiers();
      if (this.goal) this._drawGoal();
      if (this.pose) this._drawRobot();
      this._drawCursor();
    }

    _drawNoMap() {
      const ctx = this.ctx;
      const w = this.width;
      const h = this.height;
      ctx.save();
      ctx.strokeStyle = "rgba(55,219,229,0.06)";
      ctx.lineWidth = 1;
      const step = 48;
      for (let x = 0; x < w; x += step) {
        ctx.beginPath(); ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, h); ctx.stroke();
      }
      for (let y = 0; y < h; y += step) {
        ctx.beginPath(); ctx.moveTo(0, y + 0.5); ctx.lineTo(w, y + 0.5); ctx.stroke();
      }
      ctx.restore();
    }

    _drawMapImage() {
      const ctx = this.ctx;
      const { width, height, resolution, origin, image } = this.map;
      const scale = this.view.scale;
      const cos = Math.cos(origin.yaw);
      const sin = Math.sin(origin.yaw);
      const res = resolution * scale;
      const a = res * cos;
      const b = -res * sin;
      const c = res * sin;
      const d = res * cos;
      const e = this.width / 2 + scale * (origin.x - this.view.x)
        + res * (cos * 0.5 - sin * (height - 0.5));
      const f = this.height / 2 - scale * (origin.y - this.view.y)
        - res * (sin * 0.5 + cos * (height - 0.5));
      ctx.save();
      ctx.setTransform(this.dpr * a, this.dpr * b, this.dpr * c, this.dpr * d,
        this.dpr * e, this.dpr * f);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(image, 0, 0);
      ctx.restore();
      ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);

      // 地图边界
      const corners = [[0, 0], [width * resolution, 0],
        [width * resolution, height * resolution], [0, height * resolution]]
        .map(([lx, ly]) => this.worldToScreen(
          origin.x + cos * lx - sin * ly, origin.y + sin * lx + cos * ly));
      ctx.save();
      ctx.beginPath();
      corners.forEach((p, index) => (index ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      ctx.closePath();
      ctx.strokeStyle = "rgba(120,160,180,0.35)";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.restore();
    }

    _drawGrid() {
      const ctx = this.ctx;
      const scale = this.view.scale;
      const topLeft = this.screenToWorld(0, 0);
      const bottomRight = this.screenToWorld(this.width, this.height);
      const minX = Math.floor(topLeft.x);
      const maxX = Math.ceil(bottomRight.x);
      const minY = Math.floor(bottomRight.y);
      const maxY = Math.ceil(topLeft.y);
      if ((maxX - minX) > 400 || (maxY - minY) > 400) return;
      ctx.save();
      ctx.lineWidth = 1;
      for (let x = minX; x <= maxX; x += 1) {
        const major = x % 5 === 0;
        ctx.strokeStyle = major ? "rgba(150,190,210,0.10)" : "rgba(150,190,210,0.045)";
        const sx = Math.round(this.worldToScreen(x, 0).x) + 0.5;
        ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, this.height); ctx.stroke();
      }
      for (let y = minY; y <= maxY; y += 1) {
        const major = y % 5 === 0;
        ctx.strokeStyle = major ? "rgba(150,190,210,0.10)" : "rgba(150,190,210,0.045)";
        const sy = Math.round(this.worldToScreen(0, y).y) + 0.5;
        ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(this.width, sy); ctx.stroke();
      }
      // 坐标刻度
      if (scale > 12) {
        ctx.fillStyle = "rgba(150,185,205,0.5)";
        ctx.font = "9px 'JetBrains Mono', Consolas, monospace";
        for (let x = Math.ceil(minX / 5) * 5; x <= maxX; x += 5) {
          const p = this.worldToScreen(x, 0);
          ctx.fillText(`${x}`, p.x + 2, this.height - 6);
        }
        for (let y = Math.ceil(minY / 5) * 5; y <= maxY; y += 5) {
          const p = this.worldToScreen(0, y);
          ctx.fillText(`${y}`, 4, p.y - 3);
        }
      }
      ctx.restore();
    }

    _drawScan() {
      const ctx = this.ctx;
      ctx.save();
      ctx.fillStyle = "rgba(255,209,102,0.55)";
      const size = Math.max(1.4, Math.min(2.6, this.view.scale * 0.06));
      for (const [x, y] of this.scan) {
        const p = this.worldToScreen(x, y);
        ctx.fillRect(p.x - size / 2, p.y - size / 2, size, size);
      }
      ctx.restore();
    }

    _drawTrail() {
      const ctx = this.ctx;
      const points = this.trail;
      const start = Math.max(0, points.length - 6000);
      ctx.save();
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.strokeStyle = "rgba(46,163,175,0.75)";
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      let started = false;
      for (let i = start; i < points.length; i += 1) {
        const p = this.worldToScreen(points[i][0], points[i][1]);
        if (!started) { ctx.moveTo(p.x, p.y); started = true; }
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      // 末端高亮
      if (points.length > 1) {
        const last = this.worldToScreen(points[points.length - 1][0],
          points[points.length - 1][1]);
        ctx.fillStyle = "rgba(120,235,245,0.9)";
        ctx.beginPath();
        ctx.arc(last.x, last.y, 2.6, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    }

    _drawPath() {
      const ctx = this.ctx;
      const points = this.path;
      ctx.save();
      ctx.strokeStyle = "rgba(255,184,77,0.85)";
      ctx.lineWidth = 2;
      ctx.setLineDash([7, 5]);
      ctx.lineDashOffset = -(this._time * 0.02) % 12;
      ctx.beginPath();
      points.forEach((point, index) => {
        const p = this.worldToScreen(point[0], point[1]);
        if (index === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(255,184,77,0.5)";
      for (let i = 1; i < points.length - 1; i += 1) {
        const p = this.worldToScreen(points[i][0], points[i][1]);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 1.6, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    }

    _drawFrontiers() {
      const ctx = this.ctx;
      const radius = 3; // 屏幕像素半径（与缩放无关，世界坐标经 worldToScreen 投影）
      ctx.save();
      ctx.fillStyle = "rgba(64,220,210,0.75)";
      ctx.strokeStyle = "rgba(64,220,210,0.30)";
      ctx.lineWidth = 1;
      for (const [x, y] of this.frontiers) {
        const p = this.worldToScreen(x, y);
        if (p.x < -radius || p.y < -radius
            || p.x > this.width + radius || p.y > this.height + radius) continue;
        ctx.beginPath();
        ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
    }

    _drawGoal() {
      const ctx = this.ctx;
      const p = this.worldToScreen(this.goal.x, this.goal.y);
      const pulse = 1 + 0.12 * Math.sin(this._time * 0.004);
      ctx.save();
      ctx.strokeStyle = "rgba(255,184,77,0.95)";
      ctx.fillStyle = "rgba(255,184,77,0.12)";
      ctx.lineWidth = 1.5;
      const r = 9 * pulse;
      ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.arc(p.x, p.y, 2.2, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(255,184,77,1)"; ctx.fill();
      ctx.beginPath();
      ctx.moveTo(p.x - r - 6, p.y); ctx.lineTo(p.x - r + 2, p.y);
      ctx.moveTo(p.x + r - 2, p.y); ctx.lineTo(p.x + r + 6, p.y);
      ctx.moveTo(p.x, p.y - r - 6); ctx.lineTo(p.x, p.y - r + 2);
      ctx.moveTo(p.x, p.y + r - 2); ctx.lineTo(p.x, p.y + r + 6);
      ctx.strokeStyle = "rgba(255,184,77,0.55)";
      ctx.stroke();
      if (this.goal.yaw !== null && this.goal.yaw !== undefined) {
        const yaw = this.goal.yaw * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + Math.cos(yaw) * 22, p.y - Math.sin(yaw) * 22);
        ctx.strokeStyle = "rgba(255,184,77,0.6)";
        ctx.stroke();
      }
      ctx.restore();
    }

    _drawRobot() {
      const ctx = this.ctx;
      const { x, y, yaw } = this.pose;
      const p = this.worldToScreen(x, y);
      const scale = this.view.scale;
      const length = Math.max(10, BODY_LENGTH_M * scale);
      const width = Math.max(8, BODY_WIDTH_M * scale);
      const rad = yaw * Math.PI / 180;

      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-rad);

      // 足迹
      ctx.beginPath();
      ctx.rect(-length / 2, -width / 2, length, width);
      ctx.fillStyle = "rgba(55,219,229,0.13)";
      ctx.fill();
      ctx.strokeStyle = "rgba(55,219,229,0.55)";
      ctx.lineWidth = 1;
      ctx.stroke();

      // 四足示意点
      ctx.fillStyle = "rgba(55,219,229,0.55)";
      const legX = length * 0.32;
      const legY = width * 0.46;
      for (const [lx, ly] of [[legX, legY], [legX, -legY], [-legX, legY], [-legX, -legY]]) {
        ctx.beginPath();
        ctx.arc(lx, ly, Math.max(1.4, scale * 0.035), 0, Math.PI * 2);
        ctx.fill();
      }

      // 朝向箭头
      ctx.beginPath();
      ctx.moveTo(length / 2 + Math.max(6, scale * 0.18), 0);
      ctx.lineTo(length / 2 - 2, -width * 0.42);
      ctx.lineTo(length / 2 - 2, width * 0.42);
      ctx.closePath();
      ctx.fillStyle = "rgba(55,219,229,0.95)";
      ctx.fill();
      ctx.strokeStyle = "rgba(220,255,255,0.85)";
      ctx.lineWidth = 0.8;
      ctx.stroke();
      ctx.restore();

      // 中心点与光晕
      const glow = ctx.createRadialGradient(p.x, p.y, 1, p.x, p.y, Math.max(10, scale * 0.3));
      glow.addColorStop(0, "rgba(55,219,229,0.32)");
      glow.addColorStop(1, "rgba(55,219,229,0)");
      ctx.save();
      ctx.fillStyle = glow;
      ctx.beginPath(); ctx.arc(p.x, p.y, Math.max(10, scale * 0.3), 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#d9feff";
      ctx.beginPath(); ctx.arc(p.x, p.y, 2.2, 0, Math.PI * 2); ctx.fill();
      ctx.restore();
    }

    _drawCursor() {
      if (!this.cursor) return;
      const ctx = this.ctx;
      const { x, y } = this.cursor;
      ctx.save();
      ctx.strokeStyle = this.pickMode ? "rgba(255,184,77,0.55)" : "rgba(150,200,220,0.22)";
      ctx.setLineDash([3, 4]);
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, this.height);
      ctx.moveTo(0, y + 0.5); ctx.lineTo(this.width, y + 0.5);
      ctx.stroke();
      ctx.restore();
    }

    destroy() {
      if (this._raf) global.cancelAnimationFrame(this._raf);
      if (this._observer) this._observer.disconnect();
    }
  }

  global.MapView = MapView;
})(window);