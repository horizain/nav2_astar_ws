/* ============================================================
   sim-nav.js —— 终章演示：完整的"建图 → 规划 → 控制"导航栈
   阶段①：机器人巡逻两圈，实时 mini-SLAM 建图（含回环 + 位姿图优化）
   阶段②：在"建出来的地图"上 A* 规划到目标点（可点击自定义目标）
   阶段③：MPC 跟踪路径，抵达终点
   复用 sim-slam.js 的完整 SLAM 机器（SlamSim 子类）。
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  /* BFS 膨胀（把障碍按机器人半径"涂厚"） */
  function inflate(blocked, W, H, cells) {
    const dist = new Int16Array(W * H).fill(-1);
    const q = [];
    for (let i = 0; i < W * H; i++)
      if (blocked[i]) { dist[i] = 0; q.push(i); }
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      const r = (cur / W) | 0, c = cur % W;
      if (dist[cur] >= cells) continue;
      for (const [dr, dc] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= H || nc >= W) continue;
        const ni = nr * W + nc;
        if (dist[ni] < 0) { dist[ni] = dist[cur] + 1; q.push(ni); }
      }
    }
    const out = new Uint8Array(W * H);
    for (let i = 0; i < W * H; i++)
      out[i] = (dist[i] >= 0 && dist[i] <= cells) ? 1 : 0;
    return out;
  }

  class NavSim extends window.__SlamSim {
    constructor(canvasId) {
      super(canvasId);
      if (!this.canvas) return;
      this.stage = "map";              // map | ready | plan | track | done
      this.goal = [16.2, 10.6];
      this.navPath = null; this.navPulled = null;
      this.navBlocked = null;
      this.onReset = () => { this.resetState(); this.resetNav(); };
      // 点击设定目标（建图完成后可用）
      this.canvas.addEventListener("pointerdown", e => {
        if (this.stage === "map") return;
        const [wx, wy] = this.px2world(e.offsetX, e.offsetY);
        if (wx < 0.5 || wy < 0.5 || wx > 19.5 || wy > 11.5) return;
        this.goal = [wx, wy];
        if (this.stage === "ready") this.plan();
        this.draw();
      });
      this.resetNav();
    }
    resetNav() {
      this.stage = "map";
      this.goal = [16.2, 10.6];
      this.navPath = null; this.navPulled = null;
      this.ghost = []; this.trail = null;
      this.doneMsg = "";
    }
    step(dt) {
      if (this.stage === "map") {
        super.step(dt);
        if (this.routeDone) { this.stage = "ready"; this.emit("state"); }
      } else if (this.stage === "track") {
        this.trackStep(dt);
      }
    }
    plan() {
      // 1) 用"机器人自己建出来的地图"二值化 + 膨胀
      const g = this.grid;
      const blocked0 = g.binarize(1.2);
      const blocked = inflate(blocked0, g.w, g.h, 6);   // 6 格 ≈ 0.36 m
      this.navBlocked = blocked;
      // 2) 起点 = 机器人当前位置
      const px = this.robot.pose[0], py = this.robot.pose[1];
      const sc = S.clamp((px / g.res) | 0, 0, g.w - 1);
      const sr = S.clamp((py / g.res) | 0, 0, g.h - 1);
      const gc = S.clamp((this.goal[0] / g.res) | 0, 0, g.w - 1);
      const gr = S.clamp((this.goal[1] / g.res) | 0, 0, g.h - 1);
      // 起点若在膨胀区，就地"破例"（机器人本体所在处）
      blocked[sr * g.w + sc] = 0;
      const astarGrid = { grid: blocked, W: g.w, H: g.h, res: g.res };
      const res = S.astar(astarGrid, [sr, sc], [gr, gc]);
      if (!res) {
        this.doneMsg = "❌ 目标不可达（点地图换个目标试试）";
        this.stage = "ready";
        this.draw(); this.emit("state");
        return;
      }
      this.navPath = res.path.map(i =>
        [((i % g.w) + 0.5) * g.res, (((i / g.w) | 0) + 0.5) * g.res]);
      this.navPulled = S.stringPull(
        { W: g.w, H: g.h, res: g.res, blocked }, res.path);
      this.expandedCount = res.expanded.length;
      // 3) 组装 MPC 参考轨迹
      const ref = [];
      for (let i = 0; i < this.navPulled.length - 1; i++) {
        const a = this.navPulled[i], b = this.navPulled[i + 1];
        const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
        const steps = Math.max(1, (L / 0.05) | 0);
        for (let k = 0; k < steps; k++) {
          const t = k / steps;
          ref.push([a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t,
                    Math.atan2(b[1]-a[1], b[0]-a[0]), 0.5, 0]);
        }
      }
      const lastPt = this.navPulled[this.navPulled.length - 1];
      ref.push([lastPt[0], lastPt[1], ref[ref.length-1][2], 0.35, 0]);
      this.trackRef = ref;
      this.rand = S.rng(3);
      this.robot = new S.Robot(this.world,
        [px, py, this.robot.pose[2]], 0.05, this.rand);
      this.mpcc = S.makeMPC(20, 0.05, { vMax: 0.65 });
      this.wPrev = 0; this.seg = 0;
      this.trail = [this.robot.pose.slice()];
      this.stage = "track";
      this.doneMsg = "";
      this.emit("state");
    }
    trackStep(dt) {
      const pts = this.trackRef;
      let bi = this.seg, bd = Infinity;
      for (let i = this.seg; i < Math.min(pts.length, this.seg + 80); i++) {
        const d = Math.hypot(pts[i][0] - this.robot.pose[0],
                             pts[i][1] - this.robot.pose[1]);
        if (d < bd) { bd = d; bi = i; }
      }
      this.seg = bi;
      const N = this.mpcc.N;
      const idx = [];
      for (let j = 0; j <= N; j++)
        idx.push(Math.min(pts.length - 1, bi + j * 4));
      const ref = idx.map(i => pts[i]);
      const { v, w } = this.mpcc.solve(this.robot.pose, ref, this.wPrev);
      this.wPrev = w;
      this.robot.command(v, w);
      const gArr = [this.robot.pose.slice(0, 2)];
      let gs = this.robot.pose.slice();
      for (let j = 0; j < N; j++) {
        gs = S.integrate(gs, ref[j][3], w, dt);
        gArr.push(gs.slice(0, 2));
      }
      this.ghost = gArr;
      this.trail.push(this.robot.pose.slice());
      const dGoal = Math.hypot(this.goal[0] - this.robot.pose[0],
                               this.goal[1] - this.robot.pose[1]);
      if (dGoal < 0.5) {
        this.stage = "done";
        this.doneMsg = "🎉 抵达目标！SLAM 建的地图 → A* 规划 → MPC 跟踪，全链路打通";
        this.pause();
        this.emit("state");
      }
    }
    /* 覆盖父类绘制：单大图 + 阶段徽章 */
    draw() {
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
      this.layout();
      const p = this.panels[1];
      p.x = this.panels[0].x; p.w = this.panels[2].x + this.panels[2].w - p.x;
      this.setPanelView(p, 16);
      // 机器人自建的地图
      this.grid.toCanvas(this.gridOff);
      S.drawGridCanvas(ctx, this, this.gridOff, 0, 0,
        this.world.size[0], this.world.size[1], 1);
      // 规划可视化
      if (this.navBlocked && this.stage !== "map") {
        const g = this.grid, s = this.view.s;
        ctx.fillStyle = "rgba(96,165,250,0.14)";
        for (let r = 0; r < g.h; r++) for (let c = 0; c < g.w; c++) {
          if (this.navBlocked[r * g.w + c])
            ctx.fillRect(this.w2x(c * g.res), this.w2y((r + 1) * g.res),
                         g.res * s + 0.4, g.res * s + 0.4);
        }
      }
      if (this.navPath) {
        S.drawPolyline(ctx, this, this.navPath, "rgba(96,165,250,0.55)", 1.6);
        S.drawPolyline(ctx, this, this.navPulled, "#f59e0b", 2.6, [9, 6]);
      }
      // SLAM 轨迹
      S.drawPolyline(ctx, this, this.slamPath, "rgba(96,165,250,0.85)", 2.0);
      // 跟踪
      if (this.robot) {
        S.drawPolyline(ctx, this, this.trail || [], "#2563eb", 2.6);
        if (this.ghost.length)
          S.drawPolyline(ctx, this, this.ghost, "rgba(147,197,253,0.95)", 2.2);
        S.drawRobot(ctx, this, this.robot.pose, "#60a5fa", 7);
      }
      S.drawWorld(ctx, this, this.world, {
        color: "rgba(220,38,38,0.7)", lw: 1.2,
        fillRects: "rgba(220,38,38,0.20)" });
      // 目标
      ctx.fillStyle = "#ef4444";
      ctx.beginPath();
      ctx.arc(this.w2x(this.goal[0]), this.w2y(this.goal[1]), 8, 0, S.TAU);
      ctx.fill();
      ctx.fillStyle = "#0b1220"; ctx.font = "700 10px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("目标", this.w2x(this.goal[0]), this.w2y(this.goal[1]) + 3.5);
      ctx.textAlign = "left";
      // 阶段徽章
      const stages = ["① 巡逻建图", "② A* 规划", "③ MPC 跟踪"];
      const stageIdx = { map: 0, ready: 0, plan: 1, track: 2, done: 2 };
      ctx.font = "700 12px sans-serif";
      stages.forEach((t, i) => {
        const x = 14 + i * 108, y = 24;
        const active = i === stageIdx[this.stage];
        const complete = i < stageIdx[this.stage];
        ctx.fillStyle = complete ? "rgba(16,185,129,0.2)"
                      : active ? "rgba(96,165,250,0.22)" : "rgba(255,255,255,0.06)";
        ctx.beginPath();
        ctx.roundRect(x - 8, y - 15, 96, 24, 8);
        ctx.fill();
        ctx.fillStyle = complete ? "#34d399" : active ? "#93c5fd" : "#5b6c8f";
        ctx.fillText((complete ? "✓ " : "") + t, x, y);
      });
      // 状态行
      ctx.font = "600 12.5px 'JetBrains Mono', monospace";
      ctx.fillStyle = "#9db3dd";
      let msg = this.doneMsg ||
        `SLAM: 关键帧 ${this.nodes.length} · 回环边 ${this.loopEvents.length} · ATE ${this.ateSlam.toFixed(2)} m`;
      if (this.stage === "ready")
        msg = "✅ 地图建好了！点「规划 → MPC」出发，或直接点地图选目标";
      if (this.stage === "track")
        msg = `A* 扩展 ${this.expandedCount ?? 0} 格 · MPC 跟踪中…`;
      ctx.fillText(msg, 14, H - 14);
    }
  }

  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("sim-nav");
    if (!el || el.dataset.mounted || !window.__SlamSim) return;
    el.dataset.mounted = "1";
    const sim = new NavSim(el);
    window.__simNav = sim;
    const bind = (id, fn) => { const e = document.getElementById(id); if (e) fn(e); };
    bind("sim-nav-play", b => b.onclick = () => sim.toggle());
    bind("sim-nav-reset", b => b.onclick = () => sim.reset());
    bind("sim-nav-plan", b => b.onclick = () => {
      if (sim.stage === "map") return;
      sim.plan(); sim.play();
    });
    bind("sim-nav-speed", r => r.oninput = e => {
      sim.speed = +e.target.value;
      document.getElementById("sim-nav-speed-val").textContent = e.target.value + "×";
    });
    const st = document.getElementById("sim-nav-status");
    sim.on("state", () => {
      if (!st) return;
      const names = { map: "① 巡逻建图中…", ready: "② 地图就绪，等待规划",
        track: "③ MPC 跟踪中…", done: "✅ 导航完成" };
      st.textContent = names[sim.stage] || "";
    });
    sim.play();
  });

  window.__NavSim = NavSim;
})();
