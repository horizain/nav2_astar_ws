// Ch01 interactive lab: 2D LiDAR polar->Cartesian + SE(2) map->odom->base->laser TF chain + PGM flip toggle
(function () {
  "use strict";
  function init() {
    const canvas = document.getElementById("ch01-canvas");
    if (!canvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;
    const grid = S.createWarehouseMap(0.2);

    const state = {
      driftX: 0.45,
      driftY: -0.30,
      driftThDeg: 10,
      laserX: 0.20,
      inverted: false,
      pgmFlipView: false,
    };

    const gtPose = { x: 5.2, y: 7.8, theta: 0.25 };

    function render() {
      const W = 740, H = 400;
      const ctx = window.SlocApp.setupCanvas(canvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const sx = W / 24, sy = H / 16;
      const toX = (wx) => wx * sx;
      const toY = (wy) => state.pgmFlipView ? wy * sy : (H - wy * sy);

      // Draw occupancy cells
      for (let y = 0; y < grid.height; y++) {
        for (let x = 0; x < grid.width; x++) {
          if (!grid.occupied(x, y)) continue;
          const px = x * grid.resolution * sx;
          const py = state.pgmFlipView ? y * grid.resolution * sy : H - (y + 1) * grid.resolution * sy;
          ctx.fillStyle = pal.wall;
          ctx.fillRect(px, py, grid.resolution * sx + 0.6, grid.resolution * sy + 0.6);
        }
      }

      // Compute sensor pose in world and simulated odom chain
      const odomDeltaDrift = { x: state.driftX, y: state.driftY, theta: (state.driftThDeg * Math.PI) / 180 };
      const odomToBase = S.poseMul(gtPose, S.poseInv(odomDeltaDrift));
      const mapToOdom = S.mapToOdom(gtPose, odomToBase);
      const baseToLaser = { x: state.laserX, y: 0, theta: 0 };
      const worldLaser = S.poseMul(gtPose, baseToLaser);

      // Simulate laser points in laser frame, then optionally simulate inverted roll=pi mirror if uncompensated
      const rawScan = S.makeScan(grid, worldLaser, { beams: 110, maxRange: 9.5 });
      const basePts = rawScan.map((pt) => {
        const yVal = state.inverted ? -pt.y : pt.y; // uncompensated vs compensated illustration
        return { x: pt.x + baseToLaser.x, y: yVal };
      });
      const odomWorldPts = S.transformScan(odomToBase, basePts);
      const correctedPts = S.transformScan(gtPose, basePts);

      // Ghost uncorrected odom->base scan (shows drift without map->odom!)
      ctx.fillStyle = pal.red;
      ctx.globalAlpha = 0.45;
      for (const p of odomWorldPts) {
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 2.3, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.globalAlpha = 1.0;

      // Corrected map->odom->base->laser scan
      ctx.fillStyle = state.inverted ? pal.orange : pal.blue;
      for (const p of correctedPts) {
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 2.6, 0, Math.PI * 2);
        ctx.fill();
      }

      // Draw odomToBase (orange dashed ghost robot)
      const ox = toX(odomToBase.x), oy = toY(odomToBase.y);
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = pal.red;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(ox, oy, 10, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();

      // Draw true base pose
      const rx = toX(gtPose.x), ry = toY(gtPose.y);
      ctx.fillStyle = pal.bg;
      ctx.strokeStyle = pal.green;
      ctx.lineWidth = 2.6;
      ctx.beginPath();
      ctx.arc(rx, ry, 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Laser mount
      const lx = toX(worldLaser.x), ly = toY(worldLaser.y);
      ctx.fillStyle = pal.blue;
      ctx.beginPath();
      ctx.arc(lx, ly, 4, 0, Math.PI * 2);
      ctx.fill();

      // Legend
      ctx.fillStyle = pal.text;
      ctx.font = '600 12px -apple-system, BlinkMacSystemFont, sans-serif';
      ctx.fillText("● 蓝点：加入 map→odom 补偿后的激光落点   ● 红点/虚线圆：纯里程计漂移 (odom→base)   " + (state.pgmFlipView ? "⚠ PGM 未翻转状态" : ""), 14, 22);

      // Update telemetry
      const setT = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
      setT("ch01-mto", `(${mapToOdom.x.toFixed(2)}, ${mapToOdom.y.toFixed(2)}, ${(mapToOdom.theta * 57.3).toFixed(1)}°)`);
      setT("ch01-otb", `(${odomToBase.x.toFixed(2)}, ${odomToBase.y.toFixed(2)}, ${(odomToBase.theta * 57.3).toFixed(1)}°)`);
      setT("ch01-ext", state.inverted ? "roll = π (未补偿 y 取反!)" : `x = +${state.laserX.toFixed(2)}m, roll = 0`);
      setT("ch01-pgm", state.pgmFlipView ? "错误: 地图上下倒置" : "正确: row = h - 1 - iy");
    }

    const bind = (id, key, valId, fmt) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.addEventListener("input", () => {
        state[key] = parseFloat(el.value);
        const v = document.getElementById(valId);
        if (v) v.textContent = fmt(state[key]);
        render();
      });
    };
    bind("ch01-dx", "driftX", "ch01-dx-v", (v) => `${v >= 0 ? "+" : ""}${v.toFixed(2)} m`);
    bind("ch01-dy", "driftY", "ch01-dy-v", (v) => `${v >= 0 ? "+" : ""}${v.toFixed(2)} m`);
    bind("ch01-dth", "driftThDeg", "ch01-dth-v", (v) => `${v >= 0 ? "+" : ""}${v.toFixed(0)}°`);
    bind("ch01-lx", "laserX", "ch01-lx-v", (v) => `+${v.toFixed(2)} m`);

    const invBtn = document.getElementById("ch01-inv-btn");
    if (invBtn) invBtn.addEventListener("click", () => { state.inverted = !state.inverted; invBtn.textContent = state.inverted ? "恢复正装雷达" : "模拟未补偿倒装 (roll=π)"; render(); });
    const flipBtn = document.getElementById("ch01-flip-btn");
    if (flipBtn) flipBtn.addEventListener("click", () => { state.pgmFlipView = !state.pgmFlipView; flipBtn.textContent = state.pgmFlipView ? "恢复 Y 轴正确翻转" : "模拟漏掉 PGM Y 轴翻转"; render(); });

    window.addEventListener("sloc:theme", render);
    render();
  }
  window.addEventListener("sloc:ready", init);
})();
