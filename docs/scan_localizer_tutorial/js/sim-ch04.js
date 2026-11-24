// Ch04 interactive lab: Field-GN step-by-step gradient forces + Pedestrian dynamic obstacle L2 vs Huber
(function () {
  "use strict";
  function init() {
    const canvas = document.getElementById("ch04-canvas");
    if (!canvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    const grid = S.createWarehouseMap(0.12);
    const field = new S.MapField(grid, 0.22);
    const gt = { x: 11.2, y: 5.8, theta: 0.12 };

    let hasPedestrian = false;
    let useHuber = true;
    let iter = 0;
    let curPose = { x: gt.x + 0.32, y: gt.y - 0.25, theta: gt.theta + 0.16 };
    let forces = [];
    let scanPts = S.makeScan(grid, gt, { beams: 160, maxRange: 10 });

    function rebuildScan() {
      const dyn = hasPedestrian
        ? [{ cx: 12.6, cy: 6.1, hw: 0.45, hh: 0.45 }, { cx: 10.4, cy: 4.5, hw: 0.42, hh: 0.42 }]
        : [];
      scanPts = S.makeScan(grid, gt, { beams: 160, maxRange: 10, dynBoxes: dyn });
    }

    function reset() {
      curPose = { x: gt.x + 0.32, y: gt.y - 0.25, theta: gt.theta + 0.16 };
      iter = 0;
      forces = [];
      rebuildScan();
      render();
    }

    function stepOnce() {
      const st = S.fieldGnStep(field, curPose, scanPts, { useHuber, huberDelta: 0.08, trimDist: hasPedestrian ? 0.35 : 0.65 });
      if (st.ok) {
        curPose = st.nextPose;
        forces = st.forces;
        iter++;
      }
      render();
    }

    function render() {
      const W = 740, H = 380;
      const ctx = window.SlocApp.setupCanvas(canvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const x0 = 6.5, x1 = 19.5, y0 = 1.5, y1 = 10.5;
      const toX = (wx) => ((wx - x0) / (x1 - x0)) * W;
      const toY = (wy) => H - ((wy - y0) / (y1 - y0)) * H;

      for (let y = 0; y < grid.height; y++) {
        for (let x = 0; x < grid.width; x++) {
          if (!grid.occupied(x, y)) continue;
          const w = grid.gridToWorld(x, y);
          if (w.x < x0 || w.x > x1 || w.y < y0 || w.y > y1) continue;
          ctx.fillStyle = pal.wall;
          ctx.fillRect(toX(w.x) - 3, toY(w.y) - 3, 6, 6);
        }
      }

      if (hasPedestrian) {
        const dyn = [{ cx: 12.6, cy: 6.1, hw: 0.45, hh: 0.45 }, { cx: 10.4, cy: 4.5, hw: 0.42, hh: 0.42 }];
        ctx.fillStyle = "rgba(255,59,48,0.18)";
        ctx.strokeStyle = pal.red;
        ctx.setLineDash([4, 4]);
        for (const b of dyn) {
          const rx = toX(b.cx - b.hw), ry = toY(b.cy + b.hh);
          const rw = toX(b.cx + b.hw) - rx, rh = toY(b.cy - b.hh) - ry;
          ctx.fillRect(rx, ry, rw, rh);
          ctx.strokeRect(rx, ry, rw, rh);
        }
        ctx.setLineDash([]);
      }

      // Draw gradient forces on points
      for (const f of forces) {
        const px = toX(f.wx), py = toY(f.wy);
        ctx.strokeStyle = f.w < 0.8 ? pal.orange : pal.green;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px - f.r * f.gx * 55, py + f.r * f.gy * 55);
        ctx.stroke();
      }

      const wpts = S.transformScan(curPose, scanPts);
      ctx.fillStyle = pal.blue;
      for (const p of wpts) {
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 2.3, 0, Math.PI * 2);
        ctx.fill();
      }

      // GT marker & current marker
      ctx.strokeStyle = pal.green;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(toX(gt.x), toY(gt.y), 8, 0, Math.PI * 2);
      ctx.stroke();

      ctx.fillStyle = pal.blue;
      ctx.beginPath();
      ctx.arc(toX(curPose.x), toY(curPose.y), 6, 0, Math.PI * 2);
      ctx.fill();

      const errCm = Math.hypot(curPose.x - gt.x, curPose.y - gt.y) * 100;
      const angErr = Math.abs(S.wrapAngle(curPose.theta - gt.theta)) * 57.3;
      const setT = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
      setT("ch04-iter-v", `#${iter}`);
      setT("ch04-pos-err", `${errCm.toFixed(1)} cm`);
      setT("ch04-ang-err", `${angErr.toFixed(2)}°`);
      setT("ch04-mode-v", `${useHuber ? "Huber+Trim" : "纯 L2 平方"} / ${hasPedestrian ? "含2动态障碍" : "干净环境"}`);
    }

    document.getElementById("ch04-step")?.addEventListener("click", stepOnce);
    document.getElementById("ch04-converge")?.addEventListener("click", () => {
      for (let k = 0; k < 15; k++) stepOnce();
    });
    document.getElementById("ch04-reset")?.addEventListener("click", reset);
    document.getElementById("ch04-ped")?.addEventListener("click", (e) => {
      hasPedestrian = !hasPedestrian;
      e.target.textContent = hasPedestrian ? "移除动态行人" : "投放 2 个动态行人";
      reset();
    });
    document.getElementById("ch04-loss")?.addEventListener("click", (e) => {
      useHuber = !useHuber;
      e.target.textContent = useHuber ? "切换为纯 L2 对比" : "恢复 Huber + 截断";
      reset();
    });

    window.addEventListener("sloc:theme", render);
    reset();
  }
  window.addEventListener("sloc:ready", init);
})();
