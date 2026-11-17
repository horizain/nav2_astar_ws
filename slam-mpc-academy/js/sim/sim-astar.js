/* ============================================================
   sim-astar.js —— 第 10 章：A* 交互规划 + MPC 跟踪
   · 拖拽起点/终点，点击格子涂墙（再点可擦除）
   · "运行 A*"动画展示扩展过程 → 视线拉直 → MPC 跟踪到终点
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  class AstarSim extends S.SimBase {
    constructor(canvasId) {
      super(canvasId);
      if (!this.canvas) return;
      this.fixedDt = 0.05;
      this.world = S.makeWorld();
      this.res = 0.25;
      this.customWalls = new Set();       // 用户手涂的墙（"r,c"）
      this.resetState();
      this.fitView([0.15, 0.15, 19.85, 11.85]);
      this.setWorldMap();
      this.onReset = () => this.resetState();
      // 交互
      let drag = null;
      this.canvas.addEventListener("pointerdown", e => {
        const [wx, wy] = this.px2world(e.offsetX, e.offsetY);
        if (Math.hypot(wx - this.start[0], wy - this.start[1]) < 0.6) {
          drag = "start"; return;
        }
        if (Math.hypot(wx - this.goal[0], wy - this.goal[1]) < 0.6) {
          drag = "goal"; return;
        }
        // 涂墙 / 擦墙
        const c = S.clamp((wx / this.res) | 0, 0, this.g.W - 1);
        const r = S.clamp((wy / this.res) | 0, 0, this.g.H - 1);
        const key = r + "," + c;
        if (this.customWalls.has(key)) this.customWalls.delete(key);
        else this.customWalls.add(key);
        this.rebuild();
        this.draw();
        this.canvas.setPointerCapture(e.pointerId);
      });
      this.canvas.addEventListener("pointermove", e => {
        if (!drag) return;
        const [wx, wy] = this.px2world(e.offsetX, e.offsetY);
        const cell = this.freeCell(wx, wy);
        if (drag === "start") this.start = cell;
        else this.goal = cell;
        this.rebuild();
        this.draw();
      });
      this.canvas.addEventListener("pointerup", () => { drag = null; });
    }
    freeCell(wx, wy) {
      const c = S.clamp((wx / this.res) | 0, 0, this.g.W - 1);
      const r = S.clamp((wy / this.res) | 0, 0, this.g.H - 1);
      // 落在障碍里就找最近自由格
      if (!this.blocked[r * this.g.W + c])
        return [(c + 0.5) * this.res, (r + 0.5) * this.res];
      for (let rad = 1; rad < 10; rad++) {
        for (let dr = -rad; dr <= rad; dr++) for (let dc = -rad; dc <= rad; dc++) {
          const rr = r + dr, cc = c + dc;
          if (rr < 0 || cc < 0 || rr >= this.g.H || cc >= this.g.W) continue;
          if (!this.blocked[rr * this.g.W + cc])
            return [(cc + 0.5) * this.res, (rr + 0.5) * this.res];
        }
      }
      return [(c + 0.5) * this.res, (r + 0.5) * this.res];
    }
    resetState() {
      this.start = [2.5, 2.5];
      this.goal = [16.0, 10.5];
      this.customWalls.clear();
      this.rebuild();
      this.robot = null; this.mpcc = null;
      this.expanded = []; this.expIdx = -1; this.pathPts = null; this.pulled = null;
      this.phase = "edit";              // edit | search | ready | track | done
      this.doneMsg = "";
      this.stats = null;
      this.robotPoseTrail = null;
    }
    rebuild() {
      this.g = S.buildGrid(this.world, this.res, 0.35);
      // 用户手涂的墙也参与膨胀（简单地再挡一圈邻域）
      const W = this.g.W, H = this.g.H;
      for (const key of this.customWalls) {
        const [r, c] = key.split(",").map(Number);
        for (let dr = -2; dr <= 2; dr++) for (let dc = -2; dc <= 2; dc++) {
          const rr = r + dr, cc = c + dc;
          if (rr >= 0 && cc >= 0 && rr < H && cc < W) this.g.blocked[rr * W + cc] = 1;
        }
      }
      this.phase = this.phase === "track" ? "track" : "edit";
      if (this.phase !== "track") { this.pathPts = null; this.pulled = null;
        this.expanded = []; this.expIdx = -1; }
    }
    runSearch() {
      this.rebuild();
      // start/goel 存的是 [x, y]；A* 需要 [row=y, col=x]
      const sc = S.clamp((this.start[0] / this.res) | 0, 0, this.g.W - 1);
      const srR = S.clamp((this.start[1] / this.res) | 0, 0, this.g.H - 1);
      const gc = S.clamp((this.goal[0] / this.res) | 0, 0, this.g.W - 1);
      const gr = S.clamp((this.goal[1] / this.res) | 0, 0, this.g.H - 1);
      const res = S.astar(this.g, [srR, sc], [gr, gc]);
      if (!res) { this.doneMsg = "❌ 无可达路径（被墙挡死了）"; this.phase = "edit";
        this.draw(); return; }
      this.expanded = res.expanded;
      this.expIdx = 0;
      this.pathIdx = res.path;
      this.phase = "search";
      this.doneMsg = "";
      // 扩展动画（约 12 格/帧）
      const tick = () => {
        if (this.phase !== "search") return;
        this.expIdx = Math.min(this.expanded.length, this.expIdx + 14);
        this.draw();
        if (this.expIdx < this.expanded.length) requestAnimationFrame(tick);
        else {
          this.pulled = S.stringPull(this.g, this.pathIdx);
          this.pathPts = this.pathIdx.map(i =>
            [(((i % this.g.W) + 0.5) * this.res), ((((i / this.g.W) | 0)) + 0.5) * this.res]);
          this.stats = { expanded: this.expanded.length,
            cells: this.pathIdx.length, wps: this.pulled.length };
          this.phase = "ready";
          this.draw(); this.emit("state");
        }
      };
      tick();
    }
    startTracking() {
      if (!this.pulled) return;
      // 参考路径：拉直后的折线按 4cm 重采样 + 简单航向
      const ref = [];
      for (let i = 0; i < this.pulled.length - 1; i++) {
        const a = this.pulled[i], b = this.pulled[i + 1];
        const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
        const steps = Math.max(1, (L / 0.04) | 0);
        for (let k = 0; k < steps; k++) {
          const t = k / steps;
          ref.push([a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t,
                    Math.atan2(b[1]-a[1], b[0]-a[0]), 0.5, 0]);
        }
      }
      const lastPt = this.pulled[this.pulled.length - 1];
      ref.push([lastPt[0], lastPt[1], ref[ref.length-1][2], 0.4, 0]);
      this.track = { pts: ref };
      this.rand = S.rng(3);
      this.robot = new S.Robot(this.world, this.start.slice().concat([0]), 0.05, this.rand);
      this.robot.pose[2] = ref[0][2];
      this.mpcc = S.makeMPC(20, 0.05, { vMax: 0.7 });
      this.ghost = [];
      this.robotPoseTrail = [this.robot.pose.slice()];
      this.wPrev = 0;
      this.seg = 0;
      this.phase = "track";
      this.doneMsg = "";
    }
    step(dt) {
      if (this.phase !== "track" || !this.robot) return;
      // 当前目标点：路径上最近的前方点
      const pts = this.track.pts;
      let bi = this.seg, bd = Infinity;
      for (let i = this.seg; i < Math.min(pts.length, this.seg + 60); i++) {
        const d = Math.hypot(pts[i][0] - this.robot.pose[0],
                             pts[i][1] - this.robot.pose[1]);
        if (d < bd) { bd = d; bi = i; }
      }
      this.seg = bi;
      const N = this.mpcc.N;
      const idx = [];
      for (let j = 0; j <= N; j++)
        idx.push(Math.min(pts.length - 1, bi + j * 3));
      const ref = idx.map(i => pts[i]);
      const { v, w } = this.mpcc.solve(this.robot.pose, ref, this.wPrev);
      this.wPrev = w;
      this.robot.command(v, w);
      const g = [this.robot.pose.slice(0, 2)];
      let gs = this.robot.pose.slice();
      for (let j = 0; j < N; j++) {
        gs = S.integrate(gs, ref[j][3], w, dt);
        g.push(gs.slice(0, 2));
      }
      this.ghost = g;
      const dGoal = Math.hypot(this.goal[0] - this.robot.pose[0],
                               this.goal[1] - this.robot.pose[1]);
      if (dGoal < 0.45) {
        this.phase = "done";
        this.doneMsg = "🎉 到达终点！规划 + 控制全链路打通";
        this.pause();
        this.emit("state");
      }
    }
    draw() {
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
      const g = this.g;
      // 栅格底
      const s = this.view.s;
      for (let r = 0; r < g.H; r++) for (let c = 0; c < g.W; c++) {
        const blocked = g.blocked[r * g.W + c];
        const hardOcc = g.occ[r * g.W + c];
        if (!blocked && !hardOcc) continue;
        ctx.fillStyle = hardOcc ? "#1e2c4f"
                      : blocked ? "rgba(96,165,250,0.16)" : "transparent";
        if (hardOcc) ctx.fillStyle = "#18244a";
        ctx.fillRect(this.w2x(c * g.res), this.w2y((r + 1) * g.res),
                     g.res * s + 0.5, g.res * s + 0.5);
      }
      // 世界线框
      S.drawWorld(ctx, this, this.world, { color: "#42598f", lw: 1.4 });
      // A* 扩展动画
      if (this.expIdx > 0 && this.expanded.length) {
        ctx.fillStyle = "rgba(96,165,250,0.35)";
        const upto = this.phase === "search" ? this.expIdx : this.expanded.length;
        for (let i = 0; i < upto; i++) {
          const idx = this.expanded[i];
          const c = idx % g.W, r = (idx / g.W) | 0;
          ctx.fillRect(this.w2x(c * g.res), this.w2y((r + 1) * g.res),
                       g.res * s + 0.5, g.res * s + 0.5);
        }
      }
      // 栅格路径 & 拉直路径
      if (this.pathPts && this.phase !== "search") {
        S.drawPolyline(ctx, this, this.pathPts, "rgba(96,165,250,0.8)", 2.4);
      }
      if (this.pulled) {
        S.drawPolyline(ctx, this, this.pulled, "#f59e0b", 2.8, [8, 6]);
      }
      // 跟踪阶段
      if (this.robot) {
        if (this.ghost && this.ghost.length)
          S.drawPolyline(ctx, this, this.ghost, "rgba(147,197,253,0.9)", 2);
        S.drawPolyline(ctx, this, this.robotPoseTrail || [], "#2563eb", 2.2);
        S.drawRobot(ctx, this, this.robot.pose, "#60a5fa", 7);
        this.robotPoseTrail.push(this.robot.pose.slice());
        if (this.robotPoseTrail.length > 4000) this.robotPoseTrail.shift();
      }
      // 起终点
      const drawPin = (p, color, glyph) => {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(this.w2x(p[0]), this.w2y(p[1]), 9, 0, S.TAU);
        ctx.fill();
        ctx.fillStyle = "#0b1220"; ctx.font = "700 11px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(glyph, this.w2x(p[0]), this.w2y(p[1]) + 4);
        ctx.textAlign = "left";
      };
      drawPin(this.start, "#10b981", "起");
      drawPin(this.goal, "#ef4444", "终");
      // 状态
      ctx.font = "600 13px 'JetBrains Mono', monospace";
      ctx.fillStyle = "#eaf1ff";
      let msg = this.doneMsg;
      if (this.phase === "search") msg = `A* 搜索中… 已扩展 ${this.expIdx} 格`;
      else if (this.phase === "ready" && this.stats)
        msg = `扩展 ${this.stats.expanded} 格 · 栅格路径 ${this.stats.cells} 格 · 拉直后 ${this.stats.wps} 个航点 → 点「MPC 跟踪」`;
      else if (this.phase === "track") msg = "MPC 跟踪中…";
      else if (this.phase === "edit" && !msg)
        msg = "拖拽起/终点 · 点击画墙（点击墙可擦除）→ 点「运行 A*」";
      ctx.fillStyle = "#9db3dd";
      ctx.fillText(msg, 14, H - 14);
    }
  }

  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("sim-astar");
    if (!el || el.dataset.mounted) return;
    el.dataset.mounted = "1";
    const sim = new AstarSim(el);
    window.__simAstar = sim;
    const bind = (id, fn) => { const e = document.getElementById(id); if (e) fn(e); };
    bind("sim-astar-run", b => b.onclick = () => sim.runSearch());
    bind("sim-astar-track", b => b.onclick = () => {
      sim.pause();
      sim.startTracking();
      sim.play();
    });
    bind("sim-astar-reset", b => b.onclick = () => sim.reset());
    bind("sim-astar-clear", b => b.onclick = () => {
      sim.customWalls.clear(); sim.rebuild(); sim.draw();
    });
    sim.play();   // 空转渲染
  });

  window.__AstarSim = AstarSim;
})();
