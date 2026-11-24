// Ch02 interactive lab: Drawable 4-layer MapField explorer + 1D EDT lower-envelope stepper
(function () {
  "use strict";
  function init() {
    const mapCanvas = document.getElementById("ch02-map-canvas");
    const edtCanvas = document.getElementById("ch02-edt-canvas");
    if (!mapCanvas || !edtCanvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    // 32x24 coarse interactive grid (0.25m resolution -> 8m x 6m) so redraws are instant
    const GW = 32, GH = 24, RES = 0.25;
    const grid = new S.OccupancyGrid(GW, GH, RES, 0, 0);
    function resetGrid() {
      grid.cells.fill(0);
      grid.addBox(0, 0, 8, 0.26);
      grid.addBox(0, 5.74, 8, 6.0);
      grid.addBox(0, 0, 0.26, 6.0);
      grid.addBox(7.74, 0, 8, 6.0);
      grid.addBox(2.2, 1.5, 2.7, 3.8);
      grid.addBox(4.8, 2.2, 6.2, 2.7);
    }
    resetGrid();

    let sigma = 0.35;
    let layer = "sdf"; // occ | sdf | like | swmax
    let swLevel = 2;
    let field = new S.MapField(grid, sigma, 65, 25, 6);
    function rebuild() { field = new S.MapField(grid, sigma, 65, 25, 6); }

    function renderMap() {
      const W = 740, H = 400;
      const ctx = window.SlocApp.setupCanvas(mapCanvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);
      const cw = W / GW, ch = H / GH;

      for (let y = 0; y < GH; y++) {
        for (let x = 0; x < GW; x++) {
          const i = y * GW + x;
          const px = x * cw, py = H - (y + 1) * ch;
          const occ = grid.occupied(x, y);
          if (layer === "occ") {
            ctx.fillStyle = occ ? pal.wall : pal.bg;
          } else if (layer === "sdf") {
            const d = field.sdfCells[i];
            if (occ) ctx.fillStyle = pal.wall;
            else {
              const t = Math.min(1, Math.max(0, d / 2.2));
              ctx.fillStyle = `rgb(${Math.round(0 + t * 235)}, ${Math.round(113 + t * 130)}, ${Math.round(227 + t * 28)})`;
            }
          } else if (layer === "like") {
            const lk = field.like[i];
            const b = Math.round(255 * (1 - lk * 0.88));
            ctx.fillStyle = occ ? pal.wall : `rgb(${b}, ${Math.round(255 - lk * 110)}, 255)`;
          } else {
            const lk = field.poolScore(Math.min(swLevel, field.poolLevels() - 1), x, y);
            ctx.fillStyle = `rgb(${Math.round(255 - lk * 160)}, ${Math.round(255 - lk * 75)}, 255)`;
          }
          ctx.fillRect(px, py, cw + 0.5, ch + 0.5);
          ctx.strokeStyle = pal.grid;
          ctx.strokeRect(px, py, cw, ch);
        }
      }

      // Draw analytic SDF gradient arrows in SDF mode
      if (layer === "sdf") {
        const grad = { x: 0, y: 0 };
        ctx.strokeStyle = "rgba(255,59,48,0.85)";
        ctx.lineWidth = 1.5;
        for (let y = 2; y < GH - 1; y += 2) {
          for (let x = 2; x < GW - 1; x += 2) {
            if (grid.occupied(x, y)) continue;
            field.sdf(x, y, grad);
            const len = Math.hypot(grad.x, grad.y);
            if (len < 0.1) continue;
            const cx = (x + 0.5) * cw, cy = H - (y + 0.5) * ch;
            const ux = (grad.x / len) * 9, uy = -(grad.y / len) * 9;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + ux, cy + uy);
            ctx.stroke();
          }
        }
      }

      const setT = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
      setT("ch02-layer-v", { occ: "占据栅格", sdf: "SDF + 解析梯度", like: "高斯似然场", swmax: `swmax_[${swLevel}] (2^${swLevel}=${1 << swLevel})` }[layer]);
      setT("ch02-sig-v", `${sigma.toFixed(2)} m`);
    }

    // Paint / erase walls interactively
    let painting = false, paintVal = 100;
    const cellFromEv = (e) => {
      const rect = mapCanvas.getBoundingClientRect();
      const nx = (e.clientX - rect.left) / rect.width;
      const ny = (e.clientY - rect.top) / rect.height;
      const ix = Math.floor(nx * GW);
      const iy = GH - 1 - Math.floor(ny * GH);
      return { ix, iy };
    };
    mapCanvas.addEventListener("mousedown", (e) => {
      const { ix, iy } = cellFromEv(e);
      if (!grid.inside(ix, iy)) return;
      painting = true;
      paintVal = grid.occupied(ix, iy) ? 0 : 100;
      grid.cells[iy * GW + ix] = paintVal;
      rebuild();
      renderMap();
    });
    window.addEventListener("mousemove", (e) => {
      if (!painting) return;
      const { ix, iy } = cellFromEv(e);
      if (!grid.inside(ix, iy)) return;
      if (grid.cells[iy * GW + ix] !== paintVal) {
        grid.cells[iy * GW + ix] = paintVal;
        rebuild();
        renderMap();
      }
    });
    window.addEventListener("mouseup", () => { painting = false; });

    document.querySelectorAll("#ch02-layer-tabs button").forEach((b) => {
      b.addEventListener("click", () => {
        document.querySelectorAll("#ch02-layer-tabs button").forEach((x) => x.classList.remove("active"));
        b.classList.add("active");
        layer = b.dataset.layer;
        renderMap();
      });
    });
    const sigEl = document.getElementById("ch02-sig");
    if (sigEl) sigEl.addEventListener("input", () => { sigma = parseFloat(sigEl.value); rebuild(); renderMap(); });
    const lvlEl = document.getElementById("ch02-lvl");
    if (lvlEl) lvlEl.addEventListener("input", () => { swLevel = parseInt(lvlEl.value, 10); renderMap(); });
    const rstEl = document.getElementById("ch02-reset-btn");
    if (rstEl) rstEl.addEventListener("click", () => { resetGrid(); rebuild(); renderMap(); });

    // 1D EDT Parabola Lower Envelope Demo ({9, 100, 4, 25, 1, 16, 9, 49, 0})
    const fArr = [9, 100, 4, 25, 1, 16, 9, 49, 0];
    const trace = S.edt1dWithTrace(fArr);
    let stepIdx = trace.steps.length - 1;

    function renderEdt1d() {
      const W = 740, H = 260;
      const ctx = window.SlocApp.setupCanvas(edtCanvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const st = trace.steps[stepIdx];
      const maxY = 38;
      const padL = 48, padR = 24, padT = 24, padB = 34;
      const plotW = W - padL - padR, plotH = H - padT - padB;
      const toX = (q) => padL + (q / (fArr.length - 1)) * plotW;
      const toY = (v) => padT + plotH - (Math.min(v, maxY) / maxY) * plotH;

      // Grid & axes
      ctx.strokeStyle = pal.axis;
      ctx.beginPath();
      ctx.moveTo(padL, padT + plotH);
      ctx.lineTo(W - padR, padT + plotH);
      ctx.stroke();

      // Parabolas in active stack up to st.q
      const colors = [pal.blue, pal.green, pal.orange, pal.indigo, pal.teal, pal.red];
      st.v.forEach((p, idx) => {
        ctx.strokeStyle = colors[idx % colors.length];
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let u = 0; u <= fArr.length - 1; u += 0.08) {
          const val = (u - p) * (u - p) + fArr[p];
          const px = toX(u), py = toY(val);
          if (u === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.stroke();
      });

      // Vertical intersection dividers z[1..k]
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = pal.muted;
      for (let i = 1; i < st.z.length - 1; i++) {
        const zv = st.z[i];
        if (zv >= -0.5 && zv <= fArr.length - 0.5) {
          ctx.beginPath();
          ctx.moveTo(toX(zv), padT);
          ctx.lineTo(toX(zv), padT + plotH);
          ctx.stroke();
        }
      }
      ctx.setLineDash([]);

      // Base points f[q]
      for (let q = 0; q < fArr.length; q++) {
        const inStack = st.v.includes(q);
        ctx.fillStyle = inStack ? pal.blue : pal.muted;
        ctx.beginPath();
        ctx.arc(toX(q), toY(fArr[q]), inStack ? 5 : 3.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = pal.text;
        ctx.font = '11px "SF Mono", monospace';
        ctx.fillText(`q=${q}`, toX(q) - 10, H - 10);
      }

      const infoEl = document.getElementById("ch02-edt-info");
      if (infoEl) {
        infoEl.textContent = `当前扫描 q=${st.q} | 下包络顶点栈 v=[${st.v.join(", ")}] | 弹出被遮蔽抛物线: [${st.popped.join(", ") || "无"}] | edt1d([9,100,4])=[${S.edt1d([9, 100, 4]).join(", ")}]`;
      }
    }

    const edtPrev = document.getElementById("ch02-edt-prev");
    const edtNext = document.getElementById("ch02-edt-next");
    if (edtPrev) edtPrev.addEventListener("click", () => { stepIdx = Math.max(0, stepIdx - 1); renderEdt1d(); });
    if (edtNext) edtNext.addEventListener("click", () => { stepIdx = Math.min(trace.steps.length - 1, stepIdx + 1); renderEdt1d(); });

    window.addEventListener("sloc:theme", () => { renderMap(); renderEdt1d(); });
    renderMap();
    renderEdt1d();
  }
  window.addEventListener("sloc:ready", init);
})();
