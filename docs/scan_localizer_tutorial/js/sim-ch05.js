// Ch05 interactive lab: Long corridor Hessian degeneracy & anisotropic covariance ellipse vs damped/scalar failure
(function () {
  "use strict";
  function init() {
    const canvas = document.getElementById("ch05-canvas");
    if (!canvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    const corrGrid = S.createCorridorMap(0.12);
    const corrField = new S.MapField(corrGrid, 0.22);
    const whGrid = S.createWarehouseMap(0.12);
    const whField = new S.MapField(whGrid, 0.22);

    let mode = "corridor"; // corridor | warehouse
    let slipPct = 2.0;
    let fakeDamp = false;

    function computeScene() {
      const f = mode === "corridor" ? corrField : whField;
      const g = mode === "corridor" ? corrGrid : whGrid;
      const gt = mode === "corridor" ? { x: 15.0, y: 2.5, theta: 0.0 } : { x: 5.5, y: 8.0, theta: 0.25 };
      const scan = S.makeScan(g, gt, { beams: 140, maxRange: 10.0 });
      const res = S.fieldGnMatch(f, { x: gt.x + 0.05, y: gt.y + 0.04, theta: gt.theta + 0.01 }, scan);
      const H = res.hessian || [1, 0, 0, 0, 1, 0, 0, 0, 1];
      let cov = res.covariance;
      if (fakeDamp) {
        // Simulate common mistake: adding huge damping lambda*I or scalar inflation into covariance
        const Hd = H.slice();
        Hd[0] += 50.0; Hd[4] += 50.0; Hd[8] += 50.0;
        cov = S.covFromHessian(Hd, 0.002, 1e-4, 1e-5, 1.0);
      }
      return { f, g, gt, res, cov };
    }

    function render() {
      const W = 740, H = 360;
      const ctx = window.SlocApp.setupCanvas(canvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const { g, gt, cov } = computeScene();
      const x0 = mode === "corridor" ? 3.0 : 1.0;
      const x1 = mode === "corridor" ? 27.0 : 12.0;
      const y0 = mode === "corridor" ? -0.5 : 4.0;
      const y1 = mode === "corridor" ? 5.5 : 12.0;
      const sx = W / (x1 - x0), sy = H / (y1 - y0);
      const toX = (wx) => (wx - x0) * sx;
      const toY = (wy) => H - (wy - y0) * sy;

      for (let y = 0; y < g.height; y++) {
        for (let x = 0; x < g.width; x++) {
          if (!g.occupied(x, y)) continue;
          const w = g.gridToWorld(x, y);
          if (w.x < x0 || w.x > x1 || w.y < y0 || w.y > y1) continue;
          ctx.fillStyle = pal.wall;
          ctx.fillRect(toX(w.x) - 3, toY(w.y) - 3, 6, 6);
        }
      }

      // Draw 3-sigma covariance ellipse from 2x2 XY block
      const c00 = cov[0], c01 = cov[1], c11 = cov[4];
      const tr = c00 + c11, det = c00 * c11 - c01 * c01;
      const disc = Math.sqrt(Math.max(0, 0.25 * tr * tr - det));
      const l1 = Math.sqrt(Math.max(1e-6, 0.5 * tr + disc));
      const l2 = Math.sqrt(Math.max(1e-6, 0.5 * tr - disc));
      const ang = 0.5 * Math.atan2(2 * c01, c00 - c11);

      const rx = toX(gt.x), ry = toY(gt.y);
      ctx.save();
      ctx.translate(rx, ry);
      ctx.rotate(-ang);
      ctx.fillStyle = fakeDamp ? "rgba(255,59,48,0.18)" : "rgba(0,113,227,0.16)";
      ctx.strokeStyle = fakeDamp ? pal.red : pal.blue;
      ctx.lineWidth = 2.5;
      // Amplify visually so thin axis is visible while preserving huge aspect ratio
      const rxPx = Math.min(320, Math.max(8, l1 * sx * 0.45));
      const ryPx = Math.max(6, l2 * sy * 3.0);
      ctx.beginPath();
      ctx.ellipse(0, 0, rxPx, ryPx, 0, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.restore();

      // Robot pose & slip vector over 4m walk
      const odomDriftX = 4.0 * (slipPct / 100.0);
      ctx.fillStyle = pal.green;
      ctx.beginPath();
      ctx.arc(rx, ry, 7, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = pal.text;
      ctx.font = '600 12px -apple-system, sans-serif';
      ctx.fillText(
        mode === "corridor"
          ? `30m×3m 长走廊：激光只能看见上下平行墙，沿 X 轴无端墙约束 (${fakeDamp ? "⚠ 阻尼污染协方差" : "真实各向异性协方差"})`
          : "仓库转角与货架区：X/Y 均有充足约束，协方差近似各向同性小圆",
        12, 22
      );

      const ratio = c00 / Math.max(c11, 1e-9);
      const setT = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
      setT("ch05-c00", `${c00.toFixed(4)} m²`);
      setT("ch05-c11", `${c11.toFixed(5)} m²`);
      setT("ch05-ratio", `${ratio.toFixed(0)}×`);
      setT("ch05-ekf", mode === "corridor"
        ? (fakeDamp ? "⚠ 过自信误拉偏！" : `横向 0mm / 沿轴保里程计 (${(odomDriftX * 100).toFixed(0)}mm)`)
        : "X/Y 全方向高精度收敛 (~2cm)");
    }

    document.getElementById("ch05-scene")?.addEventListener("click", (e) => {
      mode = mode === "corridor" ? "warehouse" : "corridor";
      e.target.textContent = mode === "corridor" ? "切到仓库对比" : "切回 30m 长走廊";
      render();
    });
    document.getElementById("ch05-damp")?.addEventListener("click", (e) => {
      fakeDamp = !fakeDamp;
      e.target.textContent = fakeDamp ? "恢复未阻尼 Hessian 协方差" : "错误注入 LM 阻尼到协方差";
      render();
    });
    const slipEl = document.getElementById("ch05-slip");
    if (slipEl) slipEl.addEventListener("input", () => {
      slipPct = parseFloat(slipEl.value);
      document.getElementById("ch05-slip-v").textContent = `${slipPct.toFixed(1)}%`;
      render();
    });

    window.addEventListener("sloc:theme", render);
    render();
  }
  window.addEventListener("sloc:ready", init);
})();
