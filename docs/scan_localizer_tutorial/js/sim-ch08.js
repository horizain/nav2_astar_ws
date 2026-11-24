// Ch08 interactive lab: Full end-to-end warehouse cockpit with loop motion, crowd occlusion, kidnap & global B&B recovery
(function () {
  "use strict";
  function init() {
    const canvas = document.getElementById("ch08-canvas");
    if (!canvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    const grid = S.createWarehouseMap(0.16);
    const field = new S.MapField(grid, 0.25);
    const core = new S.LocalizerCore4D(field, { autoGlobalRecover: true, anomalyWindow: 6, anomalyThreshold: 4 });

    const loop = [
      { x: 2.2, y: 2.2 }, { x: 6.4, y: 2.2 }, { x: 6.4, y: 8.0 },
      { x: 10.6, y: 8.0 }, { x: 10.6, y: 5.5 }, { x: 18.8, y: 5.5 },
      { x: 18.8, y: 10.0 }, { x: 10.6, y: 10.0 }, { x: 6.4, y: 8.0 },
      { x: 6.4, y: 13.8 }, { x: 2.2, y: 13.8 },
    ];
    let u = 0;
    let gt = { x: 2.2, y: 2.2, theta: 0.0 };
    core.reset(gt);

    let running = true;
    let crowd = false;
    const gtTrail = [];
    const estTrail = [];
    let lastTick = 0;

    function sampleGt(param) {
      const n = loop.length;
      const seg = Math.floor(param) % n;
      const f = param - Math.floor(param);
      const a = loop[seg], b = loop[(seg + 1) % n];
      return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, theta: Math.atan2(b.y - a.y, b.x - a.x) };
    }

    function stepCockpit() {
      const nextGt = sampleGt(u + 0.035);
      u += 0.035;
      // Compute noisy odom delta (2% scale error)
      const rel = S.poseMul(S.poseInv(gt), nextGt);
      const noisyDelta = { x: rel.x * 1.02, y: rel.y * 1.02, theta: rel.theta + 0.002 };
      gt = nextGt;
      const dyn = crowd ? [{ cx: gt.x + Math.cos(gt.theta) * 1.6, cy: gt.y + Math.sin(gt.theta) * 1.6, hw: 0.5, hh: 0.5 }] : [];
      const scan = S.makeScan(grid, gt, { beams: 110, maxRange: 10.0, dynBoxes: dyn });
      const out = core.step(noisyDelta, scan);

      gtTrail.push({ x: gt.x, y: gt.y });
      estTrail.push({ x: out.pose.x, y: out.pose.y });
      if (gtTrail.length > 140) gtTrail.shift();
      if (estTrail.length > 140) estTrail.shift();

      const errCm = Math.hypot(out.pose.x - gt.x, out.pose.y - gt.y) * 100;
      const setT = (id, v, cls) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.textContent = v;
        if (cls) el.className = "tel-v " + cls;
      };
      setT("ch08-st", out.state, out.state === "TRACKING" ? "ok" : "warn");
      setT("ch08-err", `${errCm.toFixed(1)} cm`, errCm < 8 ? "ok" : "warn");
      setT("ch08-sc", out.score.toFixed(2));
      setT("ch08-inl", `${(out.inlierRatio * 100).toFixed(0)}%`);
    }

    function draw() {
      const W = 740, H = 380;
      const ctx = window.SlocApp.setupCanvas(canvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const sx = W / 24, sy = H / 16;
      const toX = (x) => x * sx, toY = (y) => H - y * sy;

      for (let y = 0; y < grid.height; y++) {
        for (let x = 0; x < grid.width; x++) {
          if (!grid.occupied(x, y)) continue;
          ctx.fillStyle = pal.wall;
          ctx.fillRect(x * grid.resolution * sx, H - (y + 1) * grid.resolution * sy, grid.resolution * sx + 0.5, grid.resolution * sy + 0.5);
        }
      }

      if (gtTrail.length > 1) {
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = pal.green;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(toX(gtTrail[0].x), toY(gtTrail[0].y));
        for (const p of gtTrail) ctx.lineTo(toX(p.x), toY(p.y));
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (estTrail.length > 1) {
        ctx.strokeStyle = pal.blue;
        ctx.lineWidth = 2.4;
        ctx.beginPath();
        ctx.moveTo(toX(estTrail[0].x), toY(estTrail[0].y));
        for (const p of estTrail) ctx.lineTo(toX(p.x), toY(p.y));
        ctx.stroke();
      }

      // Draw estimated pose & GT pose
      ctx.strokeStyle = pal.green;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(toX(gt.x), toY(gt.y), 9, 0, Math.PI * 2);
      ctx.stroke();

      ctx.fillStyle = pal.blue;
      ctx.beginPath();
      ctx.arc(toX(core.pose.x), toY(core.pose.y), 6, 0, Math.PI * 2);
      ctx.fill();

      // Frosted pill backdrop for legend so it never collides with top warehouse wall
      ctx.fillStyle = window.SlocApp.isDark() ? "rgba(29,29,31,0.90)" : "rgba(255,255,255,0.92)";
      ctx.strokeStyle = pal.axis;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(14, 10, 356, 26, 8);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = pal.text;
      ctx.font = '600 12px -apple-system, sans-serif';
      ctx.fillText("绿虚线：真实轨迹 (GT)   ●   蓝实线：LocalizerCore 4D 估计轨迹", 24, 27);
    }

    function loopRaf(ts) {
      if (running && ts - lastTick > 90) {
        stepCockpit();
        lastTick = ts;
      }
      draw();
      requestAnimationFrame(loopRaf);
    }

    document.getElementById("ch08-pause")?.addEventListener("click", (e) => {
      running = !running;
      e.target.textContent = running ? "暂停驾驶舱" : "继续运行";
    });
    document.getElementById("ch08-crowd")?.addEventListener("click", (e) => {
      crowd = !crowd;
      e.target.textContent = crowd ? "移除动态行人" : "开启前方动态行人遮挡";
    });
    document.getElementById("ch08-kidnap")?.addEventListener("click", () => {
      // Simulate kidnapping core estimate far away while robot keeps moving
      core.pose = { x: 20.0, y: 13.0, theta: -2.0 };
      core.state = "GLOBAL";
      estTrail.length = 0;
    });

    requestAnimationFrame(loopRaf);
  }
  window.addEventListener("sloc:ready", init);
})();
