/* ============================================================
   sim-slam.js —— 第 8 章：完整 mini-SLAM 实时演示
   三个联动视图：真值 | 纯里程计 | SLAM 建图
   内含完整前端（关键帧 + ICP）与后端（回环 + 位姿图 GN 优化），
   逻辑与 code/mini_slam/run_slam.py 一致。
   ============================================================ */
(function () {
  "use strict";
  const S = window.SIM;

  /* ---------- SE(2) 指数/对数（与 pose_graph.py 相同公式） ---------- */
  function se2Exp(d) {          // 指数坐标 [dx,dy,dθ] → 位姿 [x,y,θ]
    const [dx, dy, th] = d;
    const c = Math.cos(th), s = Math.sin(th);
    let vx = dx, vy = dy;
    if (Math.abs(th) > 1e-9) {
      // v = A⁻¹ [dx,dy]，A = (1/θ)[[s, -(1-c)],[1-c, s]]
      const k = th / (s * s + (1 - c) * (1 - c));
      vx = k * (s * dx + (1 - c) * dy);
      vy = k * (-(1 - c) * dx + s * dy);
    }
    return [vx, vy, th];
  }
  function se2Log(p) {          // 位姿 → 指数坐标
    const [x, y, th] = p;
    const c = Math.cos(th), s = Math.sin(th);
    if (Math.abs(th) < 1e-9) return [x, y, th];
    const k = th / (s * s + (1 - c) * (1 - c));
    return [k * (s * x + (1 - c) * y), k * (-(1 - c) * x + s * y), th];
  }
  const compose = (p, q) => [
    p[0] + Math.cos(p[2]) * q[0] - Math.sin(p[2]) * q[1],
    p[1] + Math.sin(p[2]) * q[0] + Math.cos(p[2]) * q[1],
    S.wrap(p[2] + q[2])];
  const inverse = p => {
    const c = Math.cos(p[2]), s = Math.sin(p[2]);
    return [-(c * p[0] + s * p[1]), s * p[0] - c * p[1], S.wrap(-p[2])];
  };
  /* 残差 e = Log(Z⁻¹ · T_i⁻¹ · T_j) */
  function edgeResidual(pi, pj, z) {
    const e = se2Log(compose(inverse(z), compose(inverse(pi), pj)));
    e[2] = S.wrap(e[2]);
    return e;
  }

  /* ---------- 位姿图 Gauss-Newton（数值雅可比，与 pose_graph.py 相同） ---------- */
  function optimizePoseGraph(nodes, edges, onIter) {
    const n = nodes.length;
    for (let it = 0; it < 8; it++) {
      const N3 = 3 * n;
      const H = new Float64Array(N3 * N3);
      const b = new Float64Array(N3);
      let cost = 0;
      const resAt = () => edgeResidual(nodes[i], nodes[j], z);
      for (const [i, j, z, info] of edges) {
        const e0 = edgeResidual(nodes[i], nodes[j], z);
        const W = [info[0], info[4], info[8]];   // 对角信息矩阵
        cost += W[0] * e0[0] ** 2 + W[1] * e0[1] ** 2 + W[2] * e0[2] ** 2;
        const eps = 1e-6;
        const Ji = new Float64Array(9), Jj = new Float64Array(9);
        for (const [node, J] of [[i, Ji], [j, Jj]]) {
          for (let d = 0; d < 3; d++) {
            // 与更新规则一致：右乘 Exp 扰动（T ← T·Exp(εe_d)），
            // 若直接扰动分量会与 Exp 更新不自洽 → 大回环时 GN 发散
            const old = nodes[node].slice();
            const d3 = [0, 0, 0]; d3[d] = eps;
            nodes[node] = compose(old, se2Exp(d3));
            const e1 = edgeResidual(nodes[i], nodes[j], z);
            nodes[node] = old;
            J[d * 3 + 0] = (e1[0] - e0[0]) / eps;
            J[d * 3 + 1] = (e1[1] - e0[1]) / eps;
            J[d * 3 + 2] = (e1[2] - e0[2]) / eps;
          }
        }
        for (let a = 0; a < 3; a++) {
          for (let c = 0; c < 3; c++) {
            H[(3*i+a)*N3 + (3*i+c)] += Ji[a*3+0]*W[0]*Ji[c*3+0]
              + Ji[a*3+1]*W[1]*Ji[c*3+1] + Ji[a*3+2]*W[2]*Ji[c*3+2];
            H[(3*j+a)*N3 + (3*j+c)] += Jj[a*3+0]*W[0]*Jj[c*3+0]
              + Jj[a*3+1]*W[1]*Jj[c*3+1] + Jj[a*3+2]*W[2]*Jj[c*3+2];
            H[(3*i+a)*N3 + (3*j+c)] += Ji[a*3+0]*W[0]*Jj[c*3+0]
              + Ji[a*3+1]*W[1]*Jj[c*3+1] + Ji[a*3+2]*W[2]*Jj[c*3+2];
            H[(3*j+a)*N3 + (3*i+c)] += Jj[a*3+0]*W[0]*Ji[c*3+0]
              + Jj[a*3+1]*W[1]*Ji[c*3+1] + Jj[a*3+2]*W[2]*Ji[c*3+2];
          }
          b[3*i+a] += Ji[a*3+0]*W[0]*e0[0]
            + Ji[a*3+1]*W[1]*e0[1] + Ji[a*3+2]*W[2]*e0[2];
          b[3*j+a] += Jj[a*3+0]*W[0]*e0[0]
            + Jj[a*3+1]*W[1]*e0[1] + Jj[a*3+2]*W[2]*e0[2];
        }
      }
      // 固定节点 0（锚）
      for (let d = 0; d < 3; d++) H[d * N3 + d] += 1e6;
      // 解 H Δ = -b（列主元高斯消元）
      const M = new Float64Array(N3 * (N3 + 1));
      for (let r = 0; r < N3; r++) {
        for (let c = 0; c < N3; c++) M[r * (N3 + 1) + c] = H[r * N3 + c];
        M[r * (N3 + 1) + N3] = -b[r];
      }
      for (let col = 0; col < N3; col++) {
        let piv = col;
        for (let r = col + 1; r < N3; r++)
          if (Math.abs(M[r*(N3+1)+col]) > Math.abs(M[piv*(N3+1)+col])) piv = r;
        if (piv !== col) for (let c = col; c <= N3; c++) {
          const t = M[col*(N3+1)+c]; M[col*(N3+1)+c] = M[piv*(N3+1)+c];
          M[piv*(N3+1)+c] = t;
        }
        const d = M[col*(N3+1)+col] || 1e-9;
        for (let r = col + 1; r < N3; r++) {
          const f = M[r*(N3+1)+col] / d;
          if (!f) continue;
          for (let c = col; c <= N3; c++) M[r*(N3+1)+c] -= f * M[col*(N3+1)+c];
        }
      }
      const delta = new Float64Array(N3);
      for (let r = N3 - 1; r >= 0; r--) {
        let sv = M[r*(N3+1)+N3];
        for (let c = r + 1; c < N3; c++) sv -= M[r*(N3+1)+c] * delta[c];
        delta[r] = sv / (M[r*(N3+1)+r] || 1e-9);
      }
      for (let k = 0; k < n; k++) {
        nodes[k] = compose(nodes[k], se2Exp([delta[3*k], delta[3*k+1], delta[3*k+2]]));
      }
      if (onIter) onIter(it + 1, cost);
      let mx = 0;
      for (const d of delta) mx = Math.max(mx, Math.abs(d));
      if (mx < 1e-4) break;
    }
    return nodes;
  }

  /* ---------- Umeyama 对齐（ATE 用） ---------- */
  function umeyamaErr(est, gt) {
    const n = est.length;
    let mx = 0, my = 0, gx = 0, gy = 0;
    for (const p of est) { mx += p[0]; my += p[1]; }
    for (const p of gt) { gx += p[0]; gy += p[1]; }
    mx /= n; my /= n; gx /= n; gy /= n;
    let Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
    for (let i = 0; i < n; i++) {
      const ax = est[i][0] - mx, ay = est[i][1] - my;
      const bx = gt[i][0] - gx, by = gt[i][1] - gy;
      Sxx += ax*bx; Sxy += ax*by; Syx += ay*bx; Syy += ay*by;
    }
    const th = Math.atan2(Sxy - Syx, Sxx + Syy);
    const c = Math.cos(th), s = Math.sin(th);
    let sum = 0;
    for (let i = 0; i < n; i++) {
      const px = c*est[i][0] - s*est[i][1] + (gx - (c*mx - s*my));
      const py = s*est[i][0] + c*est[i][1] + (gy - (s*mx + c*my));
      sum += Math.hypot(px - gt[i][0], py - gt[i][1]) ** 2;
    }
    return Math.sqrt(sum / n);
  }

  /* ---------- 主仿真 ---------- */
  class SlamSim extends S.SimBase {
    constructor(canvasId) {
      super(canvasId);
      if (!this.canvas) return;
      this.fixedDt = 0.05;
      this.world = S.makeWorld();
      this.onReset = () => this.resetState();
      this.resetState();
    }
    resetState() {
      this.rand = S.rng(7);
      this.robot = new S.Robot(this.world, [2.0, 2.0, 0.0], 0.05, this.rand);
      this.noiseScale = 1.0;
      this.applyNoise();
      this.stepFn = S.makeFollower(this.robot, S.ROUTE.map(p => p.slice()), 0.55);
      this.laps = 0;
      this.frameN = 0;
      // 三条轨迹
      this.gtPath = [this.robot.pose.slice()];
      this.odomPath = [this.robot.odom.slice()];
      this.slamPath = [this.robot.pose.slice()];   // 关键帧 SLAM 位姿连线
      // SLAM 状态
      this.kfIdx = [0];              // 关键帧对应的帧号
      this.kfScan = [this.robot.scan()];   // 每关键帧原始距离
      this.kfPts = [this.robot.scanToLocal(this.kfScan[0])];   // 局部点云
      this.kfOdomEst = [this.robot.odom.slice()];
      this.kfOdomPose = this.robot.odom.slice();
      this.nodes = [this.robot.odom.slice()];      // 位姿图节点（链式估计）
      this.edges = [];               // [i, j, z, info]
      this.chain = [this.robot.odom.slice()];
      this.lastKfOdom = this.robot.odom.slice();
      this.lastLoopA = -1;
      this.grid = new S.OccGrid(this.world.size, 0.06);
      this.gridOff = document.createElement("canvas");
      this.loopEvents = [];          // [{a, b}]
      this.pgoCount = 0;
      this.ateOdom = 0; this.ateSlam = 0;
      this.routeDone = false;
      // 初始建一小段
      this.grid.update(this.robot.pose, this.kfScan[0], this.robot.angles,
        this.robot.maxRange);
    }
    applyNoise() {
      this.robot.stdV = 0.045 * this.noiseScale;
      this.robot.stdW = 0.055 * this.noiseScale;
    }
    step(dt) {
      this.frameN++;
      const [v, w, done] = this.stepFn();
      this.robot.command(v, w);
      const ranges = this.robot.scan();
      this.gtPath.push(this.robot.pose.slice());
      this.odomPath.push(this.robot.odom.slice());

      // ---- 关键帧判定（里程计位移 ≥ 0.8 m 或转向 ≥ 45°） ----
      const lo = this.robot.odom;
      const d = Math.hypot(lo[0] - this.lastKfOdom[0], lo[1] - this.lastKfOdom[1]);
      const turn = Math.abs(S.wrap(lo[2] - this.lastKfOdom[2]));
      if (d >= 0.8 || turn > Math.PI / 4) {
        this.lastKfOdom = lo.slice();
        this.addKeyframe(ranges);
      }

      // ---- 实时指标 ----
      if (this.frameN % 20 === 0) this.updateMetrics();

      if (done) {
        this.laps++;
        if (this.laps < 2) {
          this.stepFn = S.makeFollower(this.robot,
            S.ROUTE.map(p => p.slice()), 0.55);
        } else {
          this.routeDone = true;
          this.pause();
          this.runPGO();           // 跑完两圈 → 自动优化
        }
      }
    }
    addKeyframe(ranges) {
      const a = this.nodes.length;
      // 先用"上一个关键帧"的里程计位姿算相对运动（顺序不能反！）
      const rel = compose(inverse(this.kfOdomPose), this.robot.odom);
      // 维护关键帧缓存（索引/扫描/局部点云/里程计估计）
      this.kfIdx.push(this.frameN);
      this.kfScan.push(ranges);
      this.kfPts.push(this.robot.scanToLocal(ranges));
      this.kfOdomEst.push(this.robot.odom.slice());
      this.kfOdomPose = this.robot.odom.slice();
      // ICP 运动边：初值 = 里程计相对位姿
      const refPts = this.kfPts[a - 1];
      const curPts = this.robot.scanToLocal(ranges);
      const init = [rel[0], rel[1], rel[2]];
      const icp = S.icp2d(refPts, curPts, init, 30, 0.8);
      const z = S.se2fromT6(icp.T);
      const info = icp.rmse > 0.25 ? [40,0,0, 0,40,0, 0,0,1000]
                                   : [400,0,0, 0,400,0, 0,0,10000];
      this.edges.push([a - 1, a, z, info]);
      this.chain.push(compose(this.chain[a - 1], se2Exp(z)));
      this.nodes[a] = this.chain[a].slice();
      // 地图更新（用当前链式估计）
      this.grid.update(this.nodes[a], ranges, this.robot.angles,
        this.robot.maxRange);
      this.slamPath.push(this.nodes[a].slice());
      // ---- 回环检测 ----
      if (a > 20 && (this.lastLoopA < 0 || a - this.lastLoopA > 6)) {
        for (let b = 0; b < a - 20; b++) {
          const dd = Math.hypot(this.nodes[a][0] - this.nodes[b][0],
                                this.nodes[a][1] - this.nodes[b][1]);
          if (dd > 1.5) continue;
          const refB = this.kfPts[b];
          const curA = this.robot.scanToLocal(ranges);
          const Tini = compose(inverse(this.nodes[b]), this.nodes[a]);
          const icpL = S.icp2d(refB, curA, [Tini[0], Tini[1], Tini[2]], 30, 0.8);
          if (icpL.rmse < 0.15) {
            const zL = S.se2fromT6(icpL.T);
            this.edges.push([b, a, zL, [300,0,0, 0,300,0, 0,0,5000]]);
            this.loopEvents.push({ a, b });
            this.lastLoopA = a;
            this.runPGO();       // 每次回环 → 立即优化（看地图"啪"地绷紧！）
            break;
          }
        }
      }
    }
    runPGO() {
      const t0 = performance.now();
      optimizePoseGraph(this.nodes, this.edges);
      this.pgoCount++;
      // 用优化后的位姿重建地图
      this.grid.clear();
      for (let k = 0; k < this.nodes.length; k++) {
        this.grid.update(this.nodes[k], this.kfScan[k], this.robot.angles,
          this.robot.maxRange);
      }
      this.slamPath = this.nodes.map(p => p.slice());
      this.updateMetrics();
      this.pgoMs = performance.now() - t0;
      this.emit("state");
      this.draw();
    }
    updateMetrics() {
      const gtKf = this.kfIdx.map(i => this.gtPath[Math.min(i, this.gtPath.length - 1)].slice(0, 2));
      this.ateOdom = umeyamaErr(
        this.kfOdomEst.map(p => p.slice(0, 2)), gtKf);
      this.ateSlam = umeyamaErr(
        this.nodes.map(p => p.slice(0, 2)), gtKf);
    }
    /* ---- 三面板视图 ---- */
    layout() {
      const W = this.cssW, H = this.cssH, gap = 6;
      const pw = (W - gap * 2) / 3;
      this.panels = [
        { x: 0, y: 0, w: pw, h: H },
        { x: pw + gap, y: 0, w: pw, h: H },
        { x: 2 * (pw + gap), y: 0, w: pw, h: H },
      ];
    }
    setPanelView(p, pad = 14) {
      const [x0, y0, x1, y1] = [0.3, 0.3, 19.7, 11.7];
      const s = Math.min((p.w - pad*2) / (x1 - x0), (p.h - pad*2 - 22) / (y1 - y0));
      this.view = { s, x0, y0,
        ox: p.x + (p.w - (x1 - x0) * s) / 2,
        oy: p.y + (p.h - (y1 - y0) * s) / 2 };
      this.w2x = x => this.view.ox + (x - this.view.x0) * this.view.s;
      this.w2y = y => this.cssH - this.view.oy - (y - this.view.y0) * this.view.s;
    }
    draw() {
      const ctx = this.ctx, W = this.cssW, H = this.cssH;
      this.layout();
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0d1830"; ctx.fillRect(0, 0, W, H);
      const titles = ["① 真值（上帝视角）", "② 里程计（会漂移）", "③ mini-SLAM（本课成果）"];
      const paths = [
        [this.gtPath, "#10b981", null],
        [this.odomPath, "#f59e0b", null],
        [this.slamPath, "#60a5fa", null],
      ];
      for (let k = 0; k < 3; k++) {
        const p = this.panels[k];
        ctx.save();
        ctx.beginPath();
        ctx.rect(p.x, p.y, p.w, p.h - 24);
        ctx.clip();
        // 面板底色
        ctx.fillStyle = k === 2 ? "#0e1b3c" : "#0d1830";
        ctx.fillRect(p.x, p.y, p.w, p.h - 24);
        this.setPanelView(p);
        // 地图（SLAM 面板用 log-odds 栅格）
        if (k === 2) {
          this.grid.toCanvas(this.gridOff);
          S.drawGridCanvas(ctx, this, this.gridOff, 0, 0,
            this.world.size[0], this.world.size[1], 1);
        }
        // 回环边
        if (k === 2 && this.loopEvents.length) {
          ctx.strokeStyle = "rgba(239,68,68,0.85)"; ctx.lineWidth = 1.2;
          ctx.setLineDash([3, 3]);
          for (const ev of this.loopEvents) {
            const a = this.nodes[ev.a], b = this.nodes[ev.b];
            ctx.beginPath();
            ctx.moveTo(this.w2x(a[0]), this.w2y(a[1]));
            ctx.lineTo(this.w2x(b[0]), this.w2y(b[1]));
            ctx.stroke();
          }
          ctx.setLineDash([]);
        }
        S.drawWorld(ctx, this, this.world,
          { color: k === 2 ? "rgba(220,38,38,0.75)" : "#4a5f95", lw: 1.4,
            fillRects: k === 2 ? "rgba(220,38,38,0.22)" : "rgba(64,90,152,0.55)" });
        // 轨迹
        S.drawPolyline(ctx, this, paths[k][0], paths[k][1], 2.0);
        if (k === 1) S.drawPolyline(ctx, this, this.gtPath, "#10b981", 1.2, [5, 5], 0.55);
        // 机器人
        const pose = k === 0 ? this.robot.pose
                   : k === 1 ? this.robot.odom
                   : (this.nodes[this.nodes.length - 1] || this.robot.odom);
        S.drawRobot(ctx, this, pose, paths[k][1], 6);
        ctx.restore();
        // 标题
        ctx.fillStyle = "#93a5c9"; ctx.font = "600 12.5px sans-serif";
        ctx.fillText(titles[k], p.x + 10, p.y + 18);
      }
      // 底部状态条
      const y = H - 7;
      ctx.fillStyle = "#8fa3c8"; ctx.font = "11.5px 'JetBrains Mono', monospace";
      ctx.fillText(`帧 ${this.frameN}`, 8, y);
      ctx.fillText(`关键帧 ${this.nodes.length}`, 90, y);
      ctx.fillText(`回环边 ${this.loopEvents.length}`, 200, y);
      ctx.fillStyle = "#f59e0b";
      ctx.fillText(`ATE里程计 ${this.ateOdom.toFixed(2)}m`, 300, y);
      ctx.fillStyle = "#34d399";
      ctx.fillText(`ATE·SLAM ${this.ateSlam.toFixed(2)}m`, 440, y);
      if (this.routeDone) {
        ctx.fillStyle = "#fbbf24"; ctx.font = "700 12px sans-serif";
        ctx.fillText("✅ 已跑完 2 圈并自动优化", 620, y);
      }
    }
  }

  /* ---- 挂载 ---- */
  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("sim-slam");
    if (!el || el.dataset.mounted) return;
    el.dataset.mounted = "1";
    const sim = new SlamSim(el);           // 关键帧缓存已在 addKeyframe 内维护
    window.__simSlam = sim;
    const bind = (id, fn) => { const e = document.getElementById(id); if (e) fn(e); };
    bind("sim-slam-play", b => b.onclick = () => sim.toggle());
    bind("sim-slam-reset", b => b.onclick = () => sim.reset());
    bind("sim-slam-pgo", b => b.onclick = () => sim.runPGO());
    bind("sim-slam-noise", r => r.oninput = e => {
      sim.noiseScale = +e.target.value;
      sim.applyNoise();
      document.getElementById("sim-slam-noise-val").textContent = e.target.value + "×";
    });
    bind("sim-slam-speed", r => r.oninput = e => {
      sim.speed = +e.target.value;
      document.getElementById("sim-slam-speed-val").textContent = e.target.value + "×";
    });
    const st = document.getElementById("sim-slam-status");
    sim.on("state", () => {
      if (st) st.textContent = sim.routeDone ?
        `已完成：关键帧 ${sim.nodes.length} · 回环 ${sim.loopEvents.length} · 优化 ${sim.pgoCount} 次` :
        `运行中：关键帧 ${sim.nodes.length} · 回环 ${sim.loopEvents.length}`;
    });
    sim.play();
  });

  window.__SlamSim = SlamSim;
})();
