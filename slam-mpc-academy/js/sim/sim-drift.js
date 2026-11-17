/* ============================================================
   sim-drift.js —— 第 4 章：航迹推算漂移演示
   左：真值路径（绿） vs 里程计路径（橙），右下角实时漂移读数。
   滑杆调节里程计噪声倍率，按钮复位。
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  class DriftSim extends S.SimBase {
    constructor(canvasId) {
      super(canvasId);
      if (!this.canvas) return;
      this.fixedDt = 0.05;
      this.world = S.makeWorld();
      this.resetState();
      this.fitView([0.4, 0.4, 19.6, 11.6]);
      this.setWorldMap();
      this.onReset = () => this.resetState();
      // 拖拽相机平移
      this.canvas.addEventListener("pointerdown", e => {
        this._drag = { x: e.offsetX, y: e.offsetY };
      });
    }
    resetState() {
      this.rand = S.rng(7);
      this.robot = new S.Robot(this.world, [2.0, 2.0, 0.0], 0.05, this.rand);
      this.noiseScale = 1.0;
      this.robot.stdV = 0.045 * this.noiseScale;
      this.robot.stdW = 0.055 * this.noiseScale;
      this.stepFn = S.makeFollower(this.robot, S.ROUTE.map(p => p.slice()), 0.55);
      this.lapsDone = 0;
      this.gtPath = [this.robot.pose.slice()];
      this.odomPath = [this.robot.odom.slice()];
      this.drift = 0;
    }
    setNoise(k) {
      this.noiseScale = k;
      this.robot.stdV = 0.045 * k;
      this.robot.stdW = 0.055 * k;
    }
    step(dt) {
      const [v, w, done] = this.stepFn();
      this.robot.command(v, w);
      this.gtPath.push(this.robot.pose.slice());
      this.odomPath.push(this.robot.odom.slice());
      this.drift = Math.hypot(this.robot.pose[0] - this.robot.odom[0],
                              this.robot.pose[1] - this.robot.odom[1]);
      if (done) { // 一圈结束 → 再来一圈，漂移越滚越大
        this.lapsDone++;
        if (this.lapsDone < 3) {
          this.stepFn = S.makeFollower(this.robot,
            S.ROUTE.map(p => p.slice()), 0.55);
        } else { this.pause(); }
      }
    }
    draw() {
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
      S.drawWorld(ctx, this, this.world, {
        color: "#31446e", lw: 1.8, fillRects: "rgba(43,63,114,0.55)" });
      S.drawPolyline(ctx, this, this.gtPath, "#10b981", 2.2, [7, 6]);
      S.drawPolyline(ctx, this, this.odomPath, "#f59e0b", 2.0);
      const gtEnd = this.gtPath[this.gtPath.length - 1];
      const odEnd = this.odomPath[this.odomPath.length - 1];
      // 漂移连线
      ctx.setLineDash([4, 4]); ctx.strokeStyle = "#ef4444"; ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(this.w2x(gtEnd[0]), this.w2y(gtEnd[1]));
      ctx.lineTo(this.w2x(odEnd[0]), this.w2y(odEnd[1]));
      ctx.stroke(); ctx.setLineDash([]);
      S.drawRobot(ctx, this, gtEnd, "#10b981");
      S.drawRobot(ctx, this, odEnd, "#f59e0b");
      // 读数
      ctx.fillStyle = "#eaf1ff";
      ctx.font = "600 13px 'JetBrains Mono', monospace";
      ctx.fillText(`t = ${(this.gtPath.length * 0.05).toFixed(1)} s`, 14, 22);
      ctx.fillStyle = "#f59e0b";
      ctx.fillText(`当前漂移 = ${this.drift.toFixed(2)} m`, 14, 42);
      ctx.font = "12px sans-serif";
      ctx.fillStyle = "#8fa3c8";
      ctx.fillText("绿=真值  橙=里程计认为的位置  红=二者差距", 14, H - 12);
    }
  }

  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("sim-drift");
    if (!el || el.dataset.mounted) return;
    el.dataset.mounted = "1";
    const sim = new DriftSim(el);
    window.__simDrift = sim;
    const bind = (id, fn) => { const e = document.getElementById(id); if (e) fn(e); };
    bind("sim-drift-play", b => b.onclick = () => sim.toggle());
    bind("sim-drift-reset", b => b.onclick = () => sim.reset());
    bind("sim-drift-noise", r => r.oninput = e => {
      sim.setNoise(+e.target.value);
      document.getElementById("sim-drift-noise-val").textContent =
        e.target.value + "×";
    });
    bind("sim-drift-speed", r => r.oninput = e => {
      sim.speed = +e.target.value;
      document.getElementById("sim-drift-speed-val").textContent =
        e.target.value + "×";
    });
    sim.play();
  });

  window.__DriftSim = DriftSim;
})();
