/* ============================================================
   sim-lidar.js —— 激光雷达扫描动画（首页 Hero 背景版 + 第 3 章交互版）
   机器人沿小圈巡航，360° 激光逐帧扫出带余辉的"雷达轨迹"。
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  class LidarSim extends S.SimBase {
    constructor(canvasId, opts = {}) {
      super(canvasId, opts);
      if (!this.canvas) return;
      this.fixedDt = 0.04;
      this.heroMode = !!opts.hero;             // Hero：暗色余辉、无 UI 数值
      this.world = S.makeWorld();
      this.rand = S.rng(opts.seed ?? 42);
      this.beams = opts.beams ?? 180;
      this.maxRange = opts.maxRange ?? 6.0;
      this.resetState();
      this.fitView([1.2, 1.0, 9.6, 7.6]);
      this.setWorldMap();
      this.onReset = () => this.resetState();
    }
    resetState() {
      const start = this.heroMode ? [5.4, 4.2, 0] : [3.0, 2.6, 0];
      this.robotPose = start.slice();
      // 小圈航点（避开障碍）
      this.wps = [[7.2, 2.4], [7.6, 4.0], [6.4, 5.6], [4.4, 5.8],
                  [3.0, 4.6], [3.2, 2.8]];
      this.wpIdx = 0;
      this.angles = [];
      for (let i = 0; i < this.beams; i++) this.angles.push((i / this.beams) * S.TAU);
      this.hits = [];          // 最近一帧命中点（世界系）
      this.trail = [];         // 机器人轨迹余辉
      this.sweep = 0;
    }
    step(dt) {
      // 朝当前航点巡航
      const t = this.wps[this.wpIdx % this.wps.length];
      const dx = t[0] - this.robotPose[0], dy = t[1] - this.robotPose[1];
      if (Math.hypot(dx, dy) < 0.25) this.wpIdx++;
      let err = S.wrap(Math.atan2(dy, dx) - this.robotPose[2]);
      const v = Math.abs(err) < 0.8 ? 0.55 : 0.3;
      const w = S.clamp(2.0 * err, -1.4, 1.4);
      this.robotPose = S.integrate(this.robotPose, v, w, dt);
      // 扫描
      const raw = this.world.raycast(this.robotPose, this.angles, this.maxRange);
      this.hits = [];
      for (let i = 0; i < raw.length; i++) {
        if (!isFinite(raw[i])) continue;
        const g = this.angles[i] + this.robotPose[2];
        this.hits.push([this.robotPose[0] + raw[i] * Math.cos(g),
                        this.robotPose[1] + raw[i] * Math.sin(g)]);
      }
      if (!this.heroMode) {
        this.trail.push(this.robotPose.slice());
        if (this.trail.length > 900) this.trail.shift();
      }
    }
    draw() {
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      if (this.heroMode) {
        // 暗色余辉：每帧盖一层半透明底色
        ctx.fillStyle = "rgba(11, 18, 32, 0.22)";
        ctx.fillRect(0, 0, W, H);
      } else {
        ctx.clearRect(0, 0, W, H);
        ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
        // 轨迹
        S.drawPolyline(ctx, this, this.trail, "#1d4ed8", 2);
      }
      // 光束
      ctx.lineWidth = 0.8;
      for (const h of this.hits) {
        const g = ctx.createLinearGradient(
          this.w2x(this.robotPose[0]), this.w2y(this.robotPose[1]),
          this.w2x(h[0]), this.w2y(h[1]));
        const a = this.heroMode ? 0.34 : 0.5;
        g.addColorStop(0, `rgba(96,165,250,${a * 0.25})`);
        g.addColorStop(1, `rgba(96,165,250,${a})`);
        ctx.strokeStyle = g;
        ctx.beginPath();
        ctx.moveTo(this.w2x(this.robotPose[0]), this.w2y(this.robotPose[1]));
        ctx.lineTo(this.w2x(h[0]), this.w2y(h[1]));
        ctx.stroke();
      }
      // 命中点
      ctx.fillStyle = this.heroMode ? "rgba(245,158,11,0.85)" : "#f59e0b";
      for (const h of this.hits) {
        ctx.beginPath();
        ctx.arc(this.w2x(h[0]), this.w2y(h[1]), this.heroMode ? 1.6 : 2.2, 0, S.TAU);
        ctx.fill();
      }
      // 世界
      S.drawWorld(ctx, this, this.world, {
        color: this.heroMode ? "rgba(120,150,210,0.5)" : "#31446e",
        lw: 2,
        fillRects: this.heroMode ? "rgba(35,54,100,0.5)" : "rgba(43,63,114,0.65)",
      });
      // 机器人
      S.drawRobot(ctx, this, this.robotPose, "#f59e0b", this.heroMode ? 5 : 6.5);
      if (!this.heroMode) {
        ctx.fillStyle = "#8fa3c8"; ctx.font = "12px var(--mono)";
        ctx.fillText(`激光点 ${this.hits.length} 个 · 最大量程 ${this.maxRange} m`, 14, H - 14);
      }
    }
  }

  // ---- 自动挂载 ----
  document.addEventListener("DOMContentLoaded", () => {
    const heroEl = document.getElementById("hero-canvas");
    if (heroEl) {
      const hero = new LidarSim(heroEl, { hero: true, beams: 150, seed: 11 });
      hero.fitView([1.2, 1.0, 10.5, 8.2]);
      hero.setWorldMap();
      hero.play();
      window.__heroSim = hero;
    }
    const el = document.getElementById("sim-lidar");
    if (el && !el.dataset.mounted) {
      el.dataset.mounted = "1";
      const sim = new LidarSim(el, { beams: 180 });
      window.__simLidar = sim;
      const btn = document.getElementById("sim-lidar-play");
      if (btn) btn.onclick = () => sim.toggle();
      const beams = document.getElementById("sim-lidar-beams");
      if (beams) beams.oninput = e => {
        sim.beams = +e.target.value;
        sim.angles = [];
        for (let i = 0; i < sim.beams; i++) sim.angles.push((i / sim.beams) * S.TAU);
        document.getElementById("sim-lidar-beams-val").textContent = sim.beams;
      };
      const range = document.getElementById("sim-lidar-range");
      if (range) range.oninput = e => {
        sim.maxRange = +e.target.value;
        document.getElementById("sim-lidar-range-val").textContent =
          sim.maxRange + " m";
      };
      sim.play();
    }
  });

  window.__LidarSim = LidarSim;
})();
