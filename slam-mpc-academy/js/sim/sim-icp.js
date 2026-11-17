/* ============================================================
   sim-icp.js —— 第 5 章：ICP 交互演示
   · 灰点 = 参考帧扫描 A，彩点 = 扫描 B（可拖拽 / 滑杆改变初值）
   · 点"运行 ICP"逐步动画：红线显示对应关系，看它一步步对齐
   · 故意拖太远 → ICP 掉进局部极小（这就是"初值很重要"的铁证）
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  class IcpSim extends S.SimBase {
    constructor(canvasId) {
      super(canvasId);
      if (!this.canvas) return;
      this.world = S.makeWorld();
      // 采集一对相邻扫描（0.8 m 间隔，与课程流水线一致）
      const rand = S.rng(7);
      const rA = new S.Robot(this.world, [3.0, 2.4, 0.35], 0.05, rand);
      const rB = new S.Robot(this.world, [3.8, 2.45, 0.42], 0.05, rand);
      this.scanA = rA.scanToLocal(rA.scan());
      this.scanB = rB.scanToLocal(rB.scan());
      // 里程计给出的"标准初值"
      this.T0 = S.se2toT6([0.8, 0.06, 0.09]);
      this.resetT();
      this.animTimer = null;
      this.stats = { iter: 0, rmse: null };
      // 视图：跟随扫描质心
      const cx = 1.6, cy = 0.6;
      this.fitView([-6.5 + cx, -4.5 + cy, 7.5 + cx, 5.5 + cy]);
      this.setWorldMap();
      // 拖拽
      let dragging = false;
      this.canvas.addEventListener("pointerdown", e => {
        const [wx, wy] = this.px2world(e.offsetX, e.offsetY);
        const c = this.centroid();
        if (Math.hypot(wx - c[0], wy - c[1]) < 1.6) {
          dragging = true;
          this._dragOff = [wx - c[0], wy - c[1]];
          this.canvas.setPointerCapture(e.pointerId);
        }
      });
      this.canvas.addEventListener("pointermove", e => {
        if (!dragging) return;
        const [wx, wy] = this.px2world(e.offsetX, e.offsetY);
        const p = S.se2fromT6(this.T);
        const nx = wx - this._dragOff[0], ny = wy - this._dragOff[1];
        // 绕 B 质心平移
        const c = this.centroidOf(this.scanB, this.T);
        const dx = nx - c[0], dy = ny - c[1];
        this.T = S.se2toT6([p[0] + dx, p[1] + dy, p[2]]);
        this.stats = { iter: 0, rmse: null };
        this.draw();
      });
      this.canvas.addEventListener("pointerup", () => { dragging = false; });
    }
    centroid() { return this.centroidOf(this.scanB, this.T); }
    centroidOf(pts, T) {
      const t = S.applyT6(T, pts);
      let x = 0, y = 0;
      for (const p of t) { x += p[0]; y += p[1]; }
      return [x / t.length, y / t.length];
    }
    resetT() { this.T = this.T0.slice(); this.stats = { iter: 0, rmse: null }; }
    joggle() {   // 给初值加扰动
      const rand = S.rng((Math.random() * 1e9) | 0);
      const p = S.se2fromT6(this.T0);
      this.T = S.se2toT6([p[0] + (rand() - 0.5) * 2 * this.jolt,
                          p[1] + (rand() - 0.5) * 2 * this.jolt,
                          p[2] + (rand() - 0.5) * this.jolt * 1.2]);
      this.stats = { iter: 0, rmse: null };
      this.draw();
    }
    runICP() {
      this.stopAnim();
      let n = 0;
      const tick = () => {
        const st = S.icpStep(this.scanA, this.scanB, this.T, 0.8);
        const prev = this.T; this.T = st.T;
        this.stats = { iter: ++n, rmse: st.rmse, matches: st.matches };
        this.draw();
        this.emit("icp");
        if (n > 30 || Math.abs(st.rmse - (this._lastRmse ?? Infinity)) < 1e-5) {
          this.stopAnim(); this._lastRmse = null; return;
        }
        this._lastRmse = st.rmse;
        this.animTimer = setTimeout(tick, 140);
      };
      this._lastRmse = null;
      tick();
    }
    stopAnim() { if (this.animTimer) { clearTimeout(this.animTimer); this.animTimer = null; } }
    draw() {
      if (!this.view) return;               // 视图未就绪（首帧 resize 前）
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
      const b = S.applyT6(this.T, this.scanB);
      // 对应关系线
      if (this.stats.matches) {
        ctx.strokeStyle = "rgba(239,68,68,0.4)"; ctx.lineWidth = 0.7;
        ctx.beginPath();
        for (const m of this.stats.matches) {
          ctx.moveTo(this.w2x(m[0][0]), this.w2y(m[0][1]));
          ctx.lineTo(this.w2x(m[1][0]), this.w2y(m[1][1]));
        }
        ctx.stroke();
      }
      // 点云
      ctx.fillStyle = "#94a3b8";
      for (const p of this.scanA) {
        ctx.beginPath(); ctx.arc(this.w2x(p[0]), this.w2y(p[1]), 2.4, 0, S.TAU); ctx.fill();
      }
      ctx.fillStyle = "#f59e0b";
      for (const p of b) {
        ctx.beginPath(); ctx.arc(this.w2x(p[0]), this.w2y(p[1]), 2.4, 0, S.TAU); ctx.fill();
      }
      // 质心 + 拖拽提示
      const c = this.centroid();
      ctx.strokeStyle = "#fbbf24"; ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.arc(this.w2x(c[0]), this.w2y(c[1]), 10, 0, S.TAU); ctx.stroke();
      // 读数
      ctx.font = "600 13px 'JetBrains Mono', monospace";
      ctx.fillStyle = "#eaf1ff";
      ctx.fillText(`迭代 ${this.stats.iter}`, 14, 22);
      ctx.fillStyle = this.stats.rmse < 0.12 ? "#34d399" : "#f59e0b";
      ctx.fillText(`RMSE = ${this.stats.rmse == null ? "--" : this.stats.rmse.toFixed(3)} m`,
        14, 42);
      ctx.font = "12px sans-serif"; ctx.fillStyle = "#8fa3c8";
      ctx.fillText("灰=A（固定）  橙=B（可拖拽橙色质心圈）  红=对应关系", 14, H - 12);
    }
  }

  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("sim-icp");
    if (!el || el.dataset.mounted) return;
    el.dataset.mounted = "1";
    const sim = new IcpSim(el);
    window.__simIcp = sim;
    const bind = (id, fn) => { const e = document.getElementById(id); if (e) fn(e); };
    bind("sim-icp-run", b => b.onclick = () => sim.runICP());
    bind("sim-icp-reset", b => b.onclick = () => { sim.stopAnim(); sim.resetT(); sim.draw(); });
    bind("sim-icp-joggle", b => b.onclick = () => sim.joggle());
    bind("sim-icp-jolt", r => r.oninput = e => {
      sim.jolt = +e.target.value;
      document.getElementById("sim-icp-jolt-val").textContent = e.target.value + " m";
    });
    sim.jolt = 0.6;
    sim.on("icp", s => {
      const t = document.getElementById("sim-icp-status");
      if (t) t.textContent = `迭代 ${s.stats.iter} · RMSE ${s.stats.rmse.toFixed(3)} m`;
    });
    sim.draw();
  });

  window.__IcpSim = IcpSim;
})();
