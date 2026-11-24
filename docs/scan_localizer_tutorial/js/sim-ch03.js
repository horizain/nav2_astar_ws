// Ch03 interactive lab: Correlative heatmap vs ICP-P2P local minimum vs 2D NDT voxel ellipsoids
(function () {
  "use strict";
  function init() {
    const cLeft = document.getElementById("ch03-corr-canvas");
    const cRight = document.getElementById("ch03-icp-canvas");
    if (!cLeft || !cRight || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    const grid = S.createWarehouseMap(0.15);
    const field = new S.MapField(grid, 0.25);
    const ndt = S.ndtBuild(grid, 0.9);
    const gt = { x: 5.5, y: 8.2, theta: 0.25 };
    const scan = S.makeScan(grid, gt, { beams: 140, maxRange: 10 });

    let errXY = 0.35;
    let errDeg = 22;
    let showNdt = false;
    let icpPose = { x: gt.x + errXY, y: gt.y - 0.15, theta: gt.theta + (errDeg * Math.PI) / 180 };
    let icpPairs = [];
    let icpIter = 0;

    function resetIcp() {
      icpPose = { x: gt.x + errXY, y: gt.y - errXY * 0.5, theta: gt.theta + (errDeg * Math.PI) / 180 };
      icpPairs = [];
      icpIter = 0;
    }

    function drawHeatmap() {
      const W = 360, H = 300;
      const ctx = window.SlocApp.setupCanvas(cLeft, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const prior = { x: gt.x + errXY, y: gt.y - errXY * 0.5, theta: gt.theta + (errDeg * Math.PI) / 180 };
      const res = S.correlativeMatch(field, prior, scan, { windowLin: 0.65, windowAng: 0.45, stepLin: 0.10, stepAng: 0.08 });
      const cellW = W / res.nx, cellH = H / res.ny;
      let maxS = 0.01;
      for (const h of res.heatmap) if (h.score > maxS) maxS = h.score;

      for (const h of res.heatmap) {
        const t = h.score / maxS;
        const r = Math.round(240 - t * 210);
        const g = Math.round(244 - t * 120);
        const b = 255;
        ctx.fillStyle = `rgb(${r},${g},${b})`;
        ctx.fillRect(h.xi * cellW, H - (h.yi + 1) * cellH, cellW + 0.5, cellH + 0.5);
      }

      ctx.fillStyle = pal.text;
      ctx.font = '600 12px -apple-system, sans-serif';
      ctx.fillText(`Olson 滑窗热力图 (±0.65m) · best score=${res.score.toFixed(2)}`, 10, 20);
      const corrErr = Math.hypot(res.pose.x - gt.x, res.pose.y - gt.y);
      const el = document.getElementById("ch03-corr-err");
      if (el) el.textContent = `${(corrErr * 100).toFixed(1)} cm / ${(Math.abs(S.wrapAngle(res.pose.theta - gt.theta)) * 57.3).toFixed(1)}°`;
    }

    function drawRight() {
      const W = 360, H = 300;
      const ctx = window.SlocApp.setupCanvas(cRight, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      // View window around GT [1.5..10.5] x [4.2..12.2]
      const x0 = 1.5, x1 = 10.5, y0 = 4.2, y1 = 12.2;
      const toX = (wx) => ((wx - x0) / (x1 - x0)) * W;
      const toY = (wy) => H - ((wy - y0) / (y1 - y0)) * H;

      // Walls or NDT ellipsoids
      if (showNdt) {
        for (const g of ndt.gauss) {
          if (g.mx < x0 || g.mx > x1 || g.my < y0 || g.my > y1) continue;
          const tr = g.c00 + g.c11;
          const det = g.c00 * g.c11 - g.c01 * g.c01;
          const disc = Math.sqrt(Math.max(0, 0.25 * tr * tr - det));
          const l1 = Math.sqrt(Math.max(1e-4, 0.5 * tr + disc));
          const l2 = Math.sqrt(Math.max(1e-4, 0.5 * tr - disc));
          const ang = 0.5 * Math.atan2(2 * g.c01, g.c00 - g.c11);
          ctx.save();
          ctx.translate(toX(g.mx), toY(g.my));
          ctx.rotate(-ang);
          ctx.strokeStyle = pal.indigo;
          ctx.fillStyle = "rgba(88,86,214,0.18)";
          ctx.beginPath();
          ctx.ellipse(0, 0, Math.min(34, l1 * (W / (x1 - x0)) * 2), Math.min(34, l2 * (H / (y1 - y0)) * 2), 0, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
          ctx.restore();
        }
      } else {
        for (let y = 0; y < grid.height; y++) {
          for (let x = 0; x < grid.width; x++) {
            if (!grid.occupied(x, y)) continue;
            const w = grid.gridToWorld(x, y);
            if (w.x < x0 || w.x > x1 || w.y < y0 || w.y > y1) continue;
            ctx.fillStyle = pal.wall;
            ctx.fillRect(toX(w.x) - 2.5, toY(w.y) - 2.5, 5, 5);
          }
        }
      }

      // Draw ICP pairs
      ctx.strokeStyle = pal.orange;
      ctx.lineWidth = 1;
      for (const pr of icpPairs) {
        ctx.beginPath();
        ctx.moveTo(toX(pr.wx), toY(pr.wy));
        ctx.lineTo(toX(pr.qx), toY(pr.qy));
        ctx.stroke();
      }

      // Transformed scan under icpPose
      const wpts = S.transformScan(icpPose, scan);
      ctx.fillStyle = pal.red;
      for (const p of wpts) {
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 2.1, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.fillStyle = pal.text;
      ctx.font = '600 12px -apple-system, sans-serif';
      ctx.fillText(showNdt ? `2D NDT 体素高斯场 (voxel=${ndt.voxelM}m)` : `ICP-P2P 迭代 #${icpIter}`, 10, 20);
      const icpErr = Math.hypot(icpPose.x - gt.x, icpPose.y - gt.y);
      const el = document.getElementById("ch03-icp-err");
      if (el) el.textContent = `${(icpErr * 100).toFixed(1)} cm / ${(Math.abs(S.wrapAngle(icpPose.theta - gt.theta)) * 57.3).toFixed(1)}°`;
    }

    const bind = (id, fn) => { const el = document.getElementById(id); if (el) el.addEventListener("input", fn); };
    bind("ch03-err-xy", (e) => {
      errXY = parseFloat(e.target.value);
      document.getElementById("ch03-err-xy-v").textContent = `${errXY.toFixed(2)} m`;
      resetIcp(); drawHeatmap(); drawRight();
    });
    bind("ch03-err-th", (e) => {
      errDeg = parseFloat(e.target.value);
      document.getElementById("ch03-err-th-v").textContent = `${errDeg.toFixed(0)}°`;
      resetIcp(); drawHeatmap(); drawRight();
    });
    const stepBtn = document.getElementById("ch03-icp-step");
    if (stepBtn) stepBtn.addEventListener("click", () => {
      const st = S.icpP2pStep(field, icpPose, scan);
      if (st.ok) { icpPose = st.nextPose; icpPairs = st.pairs || []; icpIter++; }
      drawRight();
    });
    const runBtn = document.getElementById("ch03-icp-run");
    if (runBtn) runBtn.addEventListener("click", () => {
      for (let i = 0; i < 12; i++) {
        const st = S.icpP2pStep(field, icpPose, scan);
        if (!st.ok) break;
        icpPose = st.nextPose; icpPairs = st.pairs || []; icpIter++;
      }
      drawRight();
    });
    const ndtBtn = document.getElementById("ch03-ndt-toggle");
    if (ndtBtn) ndtBtn.addEventListener("click", () => { showNdt = !showNdt; ndtBtn.textContent = showNdt ? "切回 ICP 网格视图" : "叠加 NDT 体素椭圆"; drawRight(); });

    window.addEventListener("sloc:theme", () => { drawHeatmap(); drawRight(); });
    drawHeatmap();
    drawRight();
  }
  window.addEventListener("sloc:ready", init);
})();
