/* ============================================================
   sim-mpc.js —— 第 9 章：MPC 轨迹跟踪交互演示
   · 灰虚线 = 参考轨迹；蓝色 = 机器人；浅蓝 = 未来 N 步预测滚动
   · 滑杆实时调整预测时域 N / 权重 Q / R / 限速，立即看到行为变化
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  class MpcSim extends S.SimBase {
    constructor(canvasId) {
      super(canvasId);
      if (!this.canvas) return;
      this.fixedDt = 0.05;
      this.world = S.makeWorld();
      this.track = S.makeRefTrack(2, 0.03, 0.55, 1.6);
      this.opts = { N: 25, qx: 3, qy: 3, qth: 0.8, rv: 0.2, rw: 0.2,
                    vMax: 1.0, vMin: 0.0, wMax: 1.6 };
      this.showGhost = true;
      this.resetState();
      this.fitView([0.3, 0.3, 19.7, 11.7]);
      this.setWorldMap();
      this.onReset = () => this.resetState();
    }
    buildMPC() { this.mpc = S.makeMPC(this.opts.N, this.fixedDt, this.opts); }
    resetState() {
      this.buildMPC();
      this.rand = S.rng(3);
      const t = this.track;
      this.robot = new S.Robot(this.world, [t.xs[0], t.ys[0], t.th[0]],
        this.fixedDt, this.rand);
      this.sPos = 0;               // 参考弧长游标（格）
      this.path = [this.robot.pose.slice()];
      this.ghost = [];
      this.errHist = [];
      this.wPrev = 0;
      this.done = false;
    }
    step(dt) {
      if (this.done) return;
      const t = this.track, n = t.xs.length;
      const i0 = (this.sPos | 0) % n;
      const idx = [];
      for (let j = 0; j < this.opts.N + 1; j++) idx.push((i0 + j) % n);
      const ref = idx.map(i => [t.xs[i], t.ys[i], t.th[i], t.vr[i], t.wr[i]]);
      const { v, w } = this.mpc.solve(this.robot.pose, ref, this.wPrev);
      this.wPrev = w;
      this.robot.command(v, w);
      this.lastV = v; this.lastW = w;
      this.sPos += ref[0][3] * dt / t.ds;
      this.path.push(this.robot.pose.slice());
      // 预测滚动（非线性前向，画"水晶球"）
      if (this.showGhost) {
        const g = [this.robot.pose.slice(0, 2)];
        let gs = this.robot.pose.slice();
        for (let j = 0; j < this.opts.N; j++) {
          gs = S.integrate(gs, ref[j][3], w, dt);
          g.push(gs.slice(0, 2));
        }
        this.ghost = g;
      } else this.ghost = [];
      // 横向误差（到参考线最近距离）
      let bd = Infinity;
      for (let i = 0; i < n; i += 2) {
        const d = Math.hypot(t.xs[i] - this.robot.pose[0],
                             t.ys[i] - this.robot.pose[1]);
        if (d < bd) bd = d;
      }
      this.errHist.push(bd);
      if (this.errHist.length > 260) this.errHist.shift();
      this.curErr = bd;
      // 跑完两圈自动停
      if (this.sPos >= n * 1.02) { this.done = true; this.pause(); }
    }
    draw() {
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
      const t = this.track;
      // 参考轨迹
      ctx.save();
      ctx.setLineDash([9, 7]);
      ctx.strokeStyle = "#5b6c8f"; ctx.lineWidth = 2.6;
      ctx.beginPath();
      ctx.moveTo(this.w2x(t.xs[0]), this.w2y(t.ys[0]));
      for (let i = 1; i < t.xs.length; i++)
        ctx.lineTo(this.w2x(t.xs[i]), this.w2y(t.ys[i]));
      ctx.stroke();
      ctx.restore();
      // 实际轨迹
      S.drawPolyline(ctx, this, this.path, "#2563eb", 2.4);
      // 预测滚动
      if (this.ghost.length) {
        S.drawPolyline(ctx, this, this.ghost, "rgba(147,197,253,0.95)", 2.2);
        ctx.fillStyle = "rgba(147,197,253,0.5)";
        for (const p of this.ghost) {
          ctx.beginPath();
          ctx.arc(this.w2x(p[0]), this.w2y(p[1]), 2.2, 0, S.TAU); ctx.fill();
        }
      }
      // 误差连线
      if (this.curErr != null) {
        const n = t.xs.length;
        let bi = 0, bd = Infinity;
        for (let i = 0; i < n; i++) {
          const d = Math.hypot(t.xs[i] - this.robot.pose[0],
                               t.ys[i] - this.robot.pose[1]);
          if (d < bd) { bd = d; bi = i; }
        }
        ctx.setLineDash([3, 4]); ctx.strokeStyle = "rgba(239,68,68,0.9)";
        ctx.lineWidth = 1.3;
        ctx.beginPath();
        ctx.moveTo(this.w2x(this.robot.pose[0]), this.w2y(this.robot.pose[1]));
        ctx.lineTo(this.w2x(t.xs[bi]), this.w2y(t.ys[bi]));
        ctx.stroke(); ctx.setLineDash([]);
      }
      S.drawRobot(ctx, this, this.robot.pose, "#60a5fa", 7);
      // 迷你误差曲线
      ctx.fillStyle = "rgba(13,21,38,0.85)";
      ctx.fillRect(W - 236, 12, 224, 64);
      ctx.strokeStyle = "rgba(147,197,253,0.35)";
      ctx.strokeRect(W - 236, 12, 224, 64);
      if (this.errHist.length > 1) {
        const mx = Math.max(0.06, ...this.errHist);
        ctx.strokeStyle = "#f59e0b"; ctx.lineWidth = 1.3;
        ctx.beginPath();
        this.errHist.forEach((e, i) => {
          const x = W - 232 + (i / 259) * 216;
          const y = 70 - (e / mx) * 52;
          i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.fillStyle = "#8fa3c8"; ctx.font = "10.5px sans-serif";
        ctx.fillText("横向误差（近 13 s）", W - 228, 26);
      }
      // 读数
      ctx.font = "600 13px 'JetBrains Mono', monospace";
      ctx.fillStyle = "#eaf1ff";
      ctx.fillText(`t = ${(this.path.length * 0.05).toFixed(1)} s`, 14, 22);
      ctx.fillStyle = "#34d399";
      ctx.fillText(`横向误差 = ${(this.curErr ?? 0).toFixed(3)} m`, 14, 42);
      ctx.fillStyle = "#9db3dd";
      ctx.fillText(`v = ${(this.lastV ?? 0).toFixed(2)} m/s   ω = ${(this.lastW ?? 0).toFixed(2)} rad/s   N = ${this.opts.N}`,
        14, 62);
      if (this.done) {
        ctx.fillStyle = "#fbbf24"; ctx.font = "700 14px sans-serif";
        ctx.fillText("✅ 两圈跟踪完成，点“重置”再来一次", 14, H - 16);
      }
    }
  }

  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("sim-mpc");
    if (!el || el.dataset.mounted) return;
    el.dataset.mounted = "1";
    const sim = new MpcSim(el);
    window.__simMpc = sim;
    const bind = (id, fn) => { const e = document.getElementById(id); if (e) fn(e); };
    bind("sim-mpc-play", b => b.onclick = () => sim.toggle());
    bind("sim-mpc-reset", b => b.onclick = () => sim.reset());
    bind("sim-mpc-ghost", b => b.onchange = e => sim.showGhost = e.target.checked);
    bind("sim-mpc-N", r => r.oninput = e => {
      sim.opts.N = +e.target.value;
      sim.buildMPC();
      document.getElementById("sim-mpc-N-val").textContent = e.target.value;
    });
    bind("sim-mpc-q", r => r.oninput = e => {
      sim.opts.qx = sim.opts.qy = +e.target.value;
      sim.buildMPC();
      document.getElementById("sim-mpc-q-val").textContent = e.target.value;
    });
    bind("sim-mpc-r", r => r.oninput = e => {
      sim.opts.rv = sim.opts.rw = +e.target.value;
      sim.buildMPC();
      document.getElementById("sim-mpc-r-val").textContent = e.target.value;
    });
    bind("sim-mpc-vmax", r => r.oninput = e => {
      sim.opts.vMax = +e.target.value;
      sim.buildMPC();
      document.getElementById("sim-mpc-vmax-val").textContent = e.target.value;
    });
    sim.play();
  });

  window.__MpcSim = MpcSim;
})();
