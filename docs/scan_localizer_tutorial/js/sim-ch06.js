// Ch06 interactive lab: Global Correlative B&B pruning vs 500-particle Particle Filter race
(function () {
  "use strict";
  function init() {
    const bnbCanvas = document.getElementById("ch06-bnb-canvas");
    const pfCanvas = document.getElementById("ch06-pf-canvas");
    if (!bnbCanvas || !pfCanvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    const grid = S.createWarehouseMap(0.2);
    const field = new S.MapField(grid, 0.25);
    const gt = { x: 16.5, y: 10.0, theta: 0.45 };
    let scan = S.makeScan(grid, gt, { beams: 108, maxRange: 10 });

    let bnbRes = S.globalCorrelativeBnB(field, scan, { beamsSubsample: 2, angStep: 0.25, nHypotheses: 3 });
    let bnbStep = bnbRes.trace.length;

    let seed = 101;
    const rng = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    const pf = new S.ParticleFilter(400, rng);
    pf.initGlobal(grid);
    let pfIter = 0;

    function drawBase(ctx, W, H, pal) {
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);
      const sx = W / 24, sy = H / 16;
      for (let y = 0; y < grid.height; y++) {
        for (let x = 0; x < grid.width; x++) {
          if (!grid.occupied(x, y)) continue;
          ctx.fillStyle = pal.wall;
          ctx.fillRect(x * grid.resolution * sx, H - (y + 1) * grid.resolution * sy, grid.resolution * sx + 0.5, grid.resolution * sy + 0.5);
        }
      }
      return { sx, sy, toX: (x) => x * sx, toY: (y) => H - y * sy };
    }

    function renderBnB() {
      const W = 360, H = 260;
      const ctx = window.SlocApp.setupCanvas(bnbCanvas, W, H);
      const pal = window.SlocApp.palette();
      const { sx, sy, toX, toY } = drawBase(ctx, W, H, pal);

      const nDraw = Math.min(bnbStep, bnbRes.trace.length);
      for (let i = 0; i < nDraw; i++) {
        const nd = bnbRes.trace[i];
        const px = nd.bx * grid.resolution * sx;
        const py = H - (nd.by + nd.span) * grid.resolution * sy;
        const pw = nd.span * grid.resolution * sx;
        const ph = nd.span * grid.resolution * sy;
        ctx.strokeStyle = nd.level <= 1 ? "rgba(0,113,227,0.45)" : "rgba(88,86,214,0.22)";
        ctx.strokeRect(px, py, pw, ph);
      }

      // GT
      ctx.strokeStyle = pal.green;
      ctx.lineWidth = 2.4;
      ctx.beginPath();
      ctx.arc(toX(gt.x), toY(gt.y), 8, 0, Math.PI * 2);
      ctx.stroke();

      // Top-K Hypotheses
      bnbRes.poses.forEach((p, idx) => {
        ctx.fillStyle = idx === 0 ? pal.blue : pal.orange;
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), idx === 0 ? 6 : 4.5, 0, Math.PI * 2);
        ctx.fill();
      });

      const bestErr = bnbRes.pose ? Math.hypot(bnbRes.pose.x - gt.x, bnbRes.pose.y - gt.y) : Infinity;
      document.getElementById("ch06-bnb-stat").textContent = `${bestErr < 99 ? (bestErr * 100).toFixed(1) + " cm" : "失败"} (展开 ${bnbRes.expanded} / 剪枝 ${bnbRes.pruned})`;
    }

    function renderPF() {
      const W = 360, H = 260;
      const ctx = window.SlocApp.setupCanvas(pfCanvas, W, H);
      const pal = window.SlocApp.palette();
      const { toX, toY } = drawBase(ctx, W, H, pal);

      ctx.fillStyle = pal.indigo;
      ctx.globalAlpha = 0.7;
      for (const p of pf.particles) {
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 2.1, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.globalAlpha = 1.0;

      ctx.strokeStyle = pal.green;
      ctx.lineWidth = 2.4;
      ctx.beginPath();
      ctx.arc(toX(gt.x), toY(gt.y), 8, 0, Math.PI * 2);
      ctx.stroke();

      const est = pf.estimate();
      ctx.fillStyle = pal.red;
      ctx.beginPath();
      ctx.arc(toX(est.x), toY(est.y), 5.5, 0, Math.PI * 2);
      ctx.fill();

      const err = Math.hypot(est.x - gt.x, est.y - gt.y);
      document.getElementById("ch06-pf-stat").textContent = `迭代 #${pfIter} · 均值误差 ${(err * 100).toFixed(0)} cm`;
    }

    document.getElementById("ch06-kidnap-btn")?.addEventListener("click", () => {
      const candidates = [
        { x: 5.0, y: 8.0, theta: 0.2 },
        { x: 16.5, y: 10.0, theta: 0.45 },
        { x: 11.2, y: 5.5, theta: -1.1 },
        { x: 21.0, y: 11.5, theta: 2.3 },
      ];
      const next = candidates[Math.floor(Math.random() * candidates.length)];
      gt.x = next.x; gt.y = next.y; gt.theta = next.theta;
      scan = S.makeScan(grid, gt, { beams: 108, maxRange: 10 });
      bnbRes = S.globalCorrelativeBnB(field, scan, { beamsSubsample: 2, angStep: 0.25, nHypotheses: 3 });
      bnbStep = bnbRes.trace.length;
      pf.initGlobal(grid);
      pfIter = 0;
      renderBnB();
      renderPF();
    });

    document.getElementById("ch06-pf-step")?.addEventListener("click", () => {
      const delta = { x: 0.12, y: 0, theta: 0.05 };
      gt.x += Math.cos(gt.theta) * delta.x;
      gt.y += Math.sin(gt.theta) * delta.x;
      gt.theta = S.wrapAngle(gt.theta + delta.theta);
      scan = S.makeScan(grid, gt, { beams: 108, maxRange: 10 });
      pf.predict(delta);
      pf.update(field, scan, 3);
      pfIter++;
      renderPF();
    });

    window.addEventListener("sloc:theme", () => { renderBnB(); renderPF(); });
    renderBnB();
    renderPF();
  }
  window.addEventListener("sloc:ready", init);
})();
