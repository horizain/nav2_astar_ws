// Hero canvas demo: live moving robot inside the warehouse SDF contour map with
// real-time raycast laser points and animated localization estimate.
(function () {
  "use strict";
  function init() {
    const canvas = document.getElementById("hero-canvas");
    if (!canvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;
    const grid = S.createWarehouseMap(0.2);
    const field = new S.MapField(grid, 0.30);
    const W = 920, H = 380;

    // Pre-render clean Apple-styled SDF/map backdrop offscreen for current theme
    let cachedTheme = null;
    let backdrop = null;
    function buildBackdrop() {
      const dark = window.SlocApp.isDark();
      const pal = window.SlocApp.palette();
      cachedTheme = dark;
      backdrop = document.createElement("canvas");
      backdrop.width = W;
      backdrop.height = H;
      const bctx = backdrop.getContext("2d");
      bctx.fillStyle = pal.bg;
      bctx.fillRect(0, 0, W, H);
      const sx = W / (grid.width * grid.resolution);
      const sy = H / (grid.height * grid.resolution);
      const cellW = grid.resolution * sx + 0.6;
      const cellH = grid.resolution * sy + 0.6;
      for (let y = 0; y < grid.height; y++) {
        for (let x = 0; x < grid.width; x++) {
          const i = y * grid.width + x;
          const px = x * grid.resolution * sx;
          const py = H - (y + 1) * grid.resolution * sy;
          if (grid.occupied(x, y)) {
            bctx.fillStyle = dark ? "#3a3a3c" : "#1d1d1f";
            bctx.fillRect(px, py, cellW, cellH);
          } else {
            const sdf = field.sdfCells[i];
            if (sdf < 1.2) {
              const alpha = Math.max(0, 1 - sdf / 1.2) * (dark ? 0.14 : 0.08);
              bctx.fillStyle = `rgba(0,113,227,${alpha.toFixed(3)})`;
              bctx.fillRect(px, py, cellW, cellH);
            }
          }
        }
      }
    }

    const waypoints = [
      { x: 2.0, y: 2.2 }, { x: 6.5, y: 2.2 }, { x: 6.5, y: 8.0 },
      { x: 10.5, y: 8.0 }, { x: 10.5, y: 5.5 }, { x: 18.8, y: 5.5 },
      { x: 18.8, y: 10.0 }, { x: 10.5, y: 10.0 }, { x: 10.5, y: 8.0 },
      { x: 6.5, y: 8.0 }, { x: 6.5, y: 13.8 }, { x: 2.0, y: 13.8 },
    ];
    let t = 0;
    const trail = [];

    function samplePath(u) {
      const n = waypoints.length;
      const seg = Math.floor(u) % n;
      const f = u - Math.floor(u);
      const a = waypoints[seg], b = waypoints[(seg + 1) % n];
      const x = a.x + (b.x - a.x) * f;
      const y = a.y + (b.y - a.y) * f;
      const theta = Math.atan2(b.y - a.y, b.x - a.x);
      return { x, y, theta };
    }

    function draw() {
      if (cachedTheme !== window.SlocApp.isDark() || !backdrop) buildBackdrop();
      const ctx = window.SlocApp.setupCanvas(canvas, W, H);
      const pal = window.SlocApp.palette();
      const sx = W / 24, sy = H / 16;
      const toX = (wx) => wx * sx;
      const toY = (wy) => H - wy * sy;

      ctx.drawImage(backdrop, 0, 0);

      t += 0.008;
      const pose = samplePath(t);
      trail.push({ x: pose.x, y: pose.y });
      if (trail.length > 180) trail.shift();

      const scan = S.makeScan(grid, pose, { beams: 96, maxRange: 10.0 });
      const worldPts = S.transformScan(pose, scan);

      // Rays
      ctx.strokeStyle = window.SlocApp.isDark() ? "rgba(41,151,255,0.16)" : "rgba(0,113,227,0.13)";
      ctx.lineWidth = 1;
      for (const p of worldPts) {
        ctx.beginPath();
        ctx.moveTo(toX(pose.x), toY(pose.y));
        ctx.lineTo(toX(p.x), toY(p.y));
        ctx.stroke();
      }

      // Trail
      if (trail.length > 1) {
        ctx.strokeStyle = pal.green;
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        ctx.moveTo(toX(trail[0].x), toY(trail[0].y));
        for (let i = 1; i < trail.length; i++) ctx.lineTo(toX(trail[i].x), toY(trail[i].y));
        ctx.stroke();
      }

      // Laser hit dots
      ctx.fillStyle = pal.blue;
      for (const p of worldPts) {
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 2.5, 0, Math.PI * 2);
        ctx.fill();
      }

      // Robot body
      const rx = toX(pose.x), ry = toY(pose.y);
      ctx.fillStyle = pal.bg;
      ctx.strokeStyle = pal.text;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(rx, ry, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.strokeStyle = pal.red;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(rx + Math.cos(pose.theta) * 14, ry - Math.sin(pose.theta) * 14);
      ctx.stroke();

      // Subtle floating HUD badge
      ctx.fillStyle = window.SlocApp.isDark() ? "rgba(29,29,31,0.86)" : "rgba(255,255,255,0.88)";
      ctx.strokeStyle = pal.axis;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(16, 14, 286, 34, 10);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = pal.green;
      ctx.beginPath();
      ctx.arc(32, 31, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = pal.text;
      ctx.font = '600 12px "SF Mono", Menlo, monospace';
      ctx.fillText(`TRACKING  x=${pose.x.toFixed(2)}m  y=${pose.y.toFixed(2)}m  θ=${(pose.theta * 57.3).toFixed(0)}°`, 44, 35);

      requestAnimationFrame(draw);
    }

    requestAnimationFrame(draw);
  }
  window.addEventListener("sloc:ready", init);
})();
