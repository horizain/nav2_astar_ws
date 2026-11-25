// Ch07 interactive lab: 4D EKF gyro bias b_g convergence + 4-gate / 5-state simulator
(function () {
  "use strict";
  function init() {
    const biasCanvas = document.getElementById("ch07-bias-canvas");
    if (!biasCanvas || !window.SlocSim || !window.SlocApp) return;
    const S = window.SlocSim;

    const grid = S.createWarehouseMap(0.12);
    const field = new S.MapField(grid, 0.22);
    let core = new S.LocalizerCore4D(field, { autoGlobalRecover: false });

    let trueBias = 0.04;
    let occlude = false;
    let stationaryLock = false;
    let stepNum = 0;
    let gt = { x: 4.5, y: 8.0, theta: 0.0 };
    const biasHist = [];

    function resetSim() {
      gt = { x: 4.5, y: 8.0, theta: 0.0 };
      core = new S.LocalizerCore4D(field, { degradedAfter: 3, lostAfter: 12, anomalyWindow: 6, anomalyThreshold: 5, autoGlobalRecover: false });
      core.reset(gt);
      stepNum = 0;
      biasHist.length = 0;
      biasHist.push({ k: 0, est: 0, gt: trueBias, varB: core.P4[3][3] });
      render();
      window.dispatchEvent(new CustomEvent("ch07:reset"));
    }

    function advance(nSteps = 1) {
      for (let i = 0; i < nSteps; i++) {
        const dt = 0.2;
        const dtrans = stationaryLock ? 0.0 : 0.14;
        const dth = stationaryLock ? 0.0 : 0.03;
        const trueWz = dth / dt;
        const rawWz = trueWz + trueBias;
        gt = S.poseMul(gt, { x: dtrans, y: 0, theta: dth });
        let scan;
        if (occlude) {
          scan = Array.from({ length: 24 }, (_, j) => ({ x: 40 + j, y: 40 }));
        } else if (stationaryLock) {
          // Simulate small prior jump while stationary so mahal fails once then self-relaxes
          scan = S.makeScan(grid, S.poseMul(gt, { x: 0.15, y: 0.08, theta: 0.04 }), { beams: 140 });
        } else {
          scan = S.makeScan(grid, gt, { beams: 140 });
        }
        const poseBefore = { ...core.pose };
        const Pbefore = core.P4.map((row) => row.slice());
        const stateBefore = core.state;
        const out = core.step({ x: dtrans, y: 0, theta: dth }, scan, { wz: rawWz, dt });
        stepNum++;
        biasHist.push({ k: stepNum, est: out.gyroBias, gt: trueBias, varB: core.P4[3][3] });
        if (biasHist.length > 80) biasHist.shift();
        updateGatesUI(out);
        window.dispatchEvent(new CustomEvent("ch07:step", {
          detail: {
            iteration: stepNum,
            out,
            stateBefore,
            poseBefore,
            Pbefore,
            poseAfter: { ...core.pose },
            Pafter: core.P4.map((row) => row.slice()),
          },
        }));
      }
      render();
    }

    function updateGatesUI(out) {
      const setGate = (id, pass, text) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.textContent = text;
        el.className = "tel-v " + (pass ? "ok" : "bad");
      };
      setGate("ch07-g-score", out.gates.score, `${out.score.toFixed(2)} ${out.gates.score ? "PASS" : "FAIL"}`);
      setGate("ch07-g-inl", out.gates.inlier, `${(out.inlierRatio * 100).toFixed(0)}% ${out.gates.inlier ? "PASS" : "FAIL"}`);
      setGate("ch07-g-mahal", out.gates.mahal, `${Number.isFinite(out.mahal) ? out.mahal.toFixed(2) : "∞"} ${out.gates.mahal ? "PASS" : "FAIL"}`);
      setGate("ch07-g-eig", out.gates.cov, `${out.minEig.toFixed(4)} ${out.gates.cov ? "PASS" : "FAIL"}`);
      const stEl = document.getElementById("ch07-state-v");
      if (stEl) {
        stEl.textContent = out.state;
        stEl.className = "tel-v " + (out.state === "TRACKING" ? "ok" : (out.state === "DEGRADED" ? "warn" : "bad"));
      }
      document.querySelectorAll(".fsm-pill").forEach((p) => {
        p.style.borderColor = p.dataset.st === out.state ? "var(--accent)" : "var(--hairline)";
        p.style.background = p.dataset.st === out.state ? "var(--accent-soft)" : "var(--bg-card)";
        p.style.color = p.dataset.st === out.state ? "var(--accent)" : "var(--ink-2)";
      });
    }

    function render() {
      const W = 740, H = 260;
      const ctx = window.SlocApp.setupCanvas(biasCanvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const padL = 56, padR = 24, padT = 28, padB = 34;
      const plotW = W - padL - padR, plotH = H - padT - padB;
      const minY = -0.02, maxY = 0.10;
      const toX = (i) => padL + (i / Math.max(1, biasHist.length - 1)) * plotW;
      const toY = (v) => padT + plotH - ((v - minY) / (maxY - minY)) * plotH;

      // Grid lines
      ctx.strokeStyle = pal.grid;
      for (const yv of [0, 0.02, 0.04, 0.06, 0.08]) {
        const y = toY(yv);
        ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
        ctx.fillStyle = pal.muted;
        ctx.font = '11px "SF Mono", monospace';
        ctx.fillText(yv.toFixed(2), 10, y + 4);
      }

      // Ground truth line
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = pal.green;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(padL, toY(trueBias));
      ctx.lineTo(W - padR, toY(trueBias));
      ctx.stroke();
      ctx.setLineDash([]);

      // Estimated b_g curve + 1-sigma band
      if (biasHist.length > 1) {
        ctx.fillStyle = "rgba(0,113,227,0.12)";
        ctx.beginPath();
        biasHist.forEach((p, idx) => {
          const sd = Math.sqrt(Math.max(0, p.varB));
          if (idx === 0) ctx.moveTo(toX(idx), toY(p.est + sd));
          else ctx.lineTo(toX(idx), toY(p.est + sd));
        });
        for (let idx = biasHist.length - 1; idx >= 0; idx--) {
          const p = biasHist[idx];
          const sd = Math.sqrt(Math.max(0, p.varB));
          ctx.lineTo(toX(idx), toY(p.est - sd));
        }
        ctx.closePath();
        ctx.fill();

        ctx.strokeStyle = pal.blue;
        ctx.lineWidth = 2.6;
        ctx.beginPath();
        biasHist.forEach((p, idx) => {
          if (idx === 0) ctx.moveTo(toX(idx), toY(p.est));
          else ctx.lineTo(toX(idx), toY(p.est));
        });
        ctx.stroke();
      }

      ctx.fillStyle = pal.text;
      ctx.font = '600 12px -apple-system, sans-serif';
      ctx.fillText(`4D EKF 在线陀螺零偏估计 (虚线真值 b_g = ${trueBias.toFixed(3)} rad/s, 蓝线当前 = ${core.gyroBias.toFixed(4)} rad/s)`, padL, 18);
    }

    document.getElementById("ch07-step-1")?.addEventListener("click", () => advance(1));
    document.getElementById("ch07-step-10")?.addEventListener("click", () => advance(15));
    document.getElementById("ch07-reset")?.addEventListener("click", () => { occlude = false; stationaryLock = false; resetSim(); });
    document.getElementById("ch07-occ-btn")?.addEventListener("click", (e) => {
      occlude = !occlude;
      stationaryLock = false;
      e.target.textContent = occlude ? "解除严重遮挡" : "注入全遮挡异常";
    });
    document.getElementById("ch07-lock-btn")?.addEventListener("click", (e) => {
      stationaryLock = !stationaryLock;
      occlude = false;
      e.target.textContent = stationaryLock ? "停止静止扰动" : "模拟静止马氏锁死测试";
    });
    document.getElementById("ch07-bias-sl")?.addEventListener("input", (e) => {
      trueBias = parseFloat(e.target.value);
      document.getElementById("ch07-bias-v").textContent = `${trueBias.toFixed(3)} rad/s`;
      render();
    });

    window.addEventListener("sloc:theme", render);
    resetSim();
  }
  window.addEventListener("sloc:ready", init);
})();
