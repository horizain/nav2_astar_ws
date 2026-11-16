/* ============================================================
   core.js —— 网页仿真共享引擎（与 code/mini_slam 中的 Python 逻辑一一对应）
   · world：线段世界 + 光线投射（解析求交，与 world.py 相同）
   · OccGrid：log-odds 占据栅格（与 grid_map.py 相同）
   · icp2d：点到点 ICP（2D 闭式旋转解）
   · Robot：差速运动学 + 里程计噪声（与 robot.py 相同）
   · astar：栅格 A* + 障碍膨胀 + 视线拉直（与 run_astar.py 相同）
   · MPC：凝聚 QP 的轻量实现（无约束求解 + 输入限幅；
     完整的 OSQP 约束版见 code/mini_slam/run_mpc.py）
   ============================================================ */
window.SIM = (function () {
  "use strict";

  /* ---------------- 小工具 ---------------- */
  const TAU = Math.PI * 2;
  const wrap = a => { a = (a + Math.PI) % TAU; if (a < 0) a += TAU; return a - Math.PI; };
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  function rng(seed) {               // mulberry32：可复现随机数
    let a = seed >>> 0;
    return function () {
      a |= 0; a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  const gauss = rand => {            // Box-Muller
    const u = Math.max(rand(), 1e-9), v = rand();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * v);
  };

  /* ---------------- 2D 世界（与 world.py 相同的几何） ---------------- */
  function makeWorld() {
    const segs = [];
    const rects = [];
    function rect(cx, cy, w, h, solid) {
      const x0 = cx - w / 2, y0 = cy - h / 2, x1 = cx + w / 2, y1 = cy + h / 2;
      const cs = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
      for (let i = 0; i < 4; i++) {
        const a = cs[i], b = cs[(i + 1) % 4];
        segs.push(a[0], a[1], b[0], b[1]);
      }
      if (solid) rects.push([cx, cy, w, h]);
    }
    const W = 20, H = 12;
    rect(W / 2, H / 2, W, H, false);      // 外墙（空壳）
    rect(5.0, 8.0, 1.2, 1.2, true);       // 方形立柱
    rect(10.0, 3.6, 2.6, 0.8, true);      // 长桌
    rect(14.0, 8.5, 1.5, 1.5, true);      // 大柱子
    rect(7.5, 5.5, 0.8, 0.8, true);       // 小柱子
    rect(16.0, 4.2, 1.0, 1.5, true);      // 靠墙柜子
    const size = [W, H];

    /* 光线投射：p = o + t·d 与线段解析求交（与 world.py 同公式）
       pose=[x,y,θ]，angles 机体系角度数组 → Float64Array 距离 */
    function raycast(pose, angles, maxRange) {
      const n = angles.length, out = new Float64Array(n).fill(Infinity);
      const ox = pose[0], oy = pose[1], th = pose[2];
      for (let i = 0; i < n; i++) {
        const g = angles[i] + th;
        const dx = Math.cos(g), dy = Math.sin(g);
        let best = Infinity;
        for (let s = 0; s < segs.length; s += 4) {
          const ax = segs[s], ay = segs[s + 1];
          const ex = segs[s + 2] - ax, ey = segs[s + 3] - ay;
          const den = dx * ey - dy * ex;
          if (Math.abs(den) < 1e-12) continue;
          const aox = ax - ox, aoy = ay - oy;
          const t = (aox * ey - aoy * ex) / den;
          const u = (aox * dy - aoy * dx) / den;
          if (t > 0.02 && t <= maxRange && u >= 0 && u <= 1 && t < best) best = t;
        }
        out[i] = best;
      }
      return out;
    }
    return { size, segs, rects, raycast };
  }

  /* ---------------- 运动学 ---------------- */
  function integrate(pose, v, w, dt) {
    return [pose[0] + v * Math.cos(pose[2]) * dt,
            pose[1] + v * Math.sin(pose[2]) * dt,
            wrap(pose[2] + w * dt)];
  }

  /* ---------------- 占据栅格（与 grid_map.py 相同） ---------------- */
  class OccGrid {
    constructor(size, res = 0.06) {
      this.res = res;
      this.w = Math.round(size[0] / res);
      this.h = Math.round(size[1] / res);
      this.logodds = new Float32Array(this.w * this.h);
    }
    update(pose, ranges, angles, maxRange, rand, sigma = 0) {
      const L_OCC = 0.85, L_FREE = -0.40, L_MAX = 4.0;
      const ox = pose[0], oy = pose[1], th = pose[2];
      const step = this.res / 2, nBeams = ranges.length;
      const maxSteps = Math.ceil(maxRange / step);
      const W = this.w, H = this.h, log = this.logodds;
      for (let b = 0; b < nBeams; b++) {
        const g = angles[b] + th;
        const dx = Math.cos(g), dy = Math.sin(g);
        const r = Math.min(ranges[b], maxRange);
        const nSteps = Math.max(1, Math.floor(r / step));
        for (let k = 0; k < nSteps; k++) {
          const d = k * step;
          if (d > r - 0.02) break;
          const x = ox + dx * d, y = oy + dy * d;
          const c = (x / this.res) | 0, rr = (y / this.res) | 0;
          if (c < 0 || c >= W || rr < 0 || rr >= H) continue;
          log[rr * W + c] = Math.max(-L_MAX, Math.min(L_MAX,
            log[rr * W + c] + L_FREE));
        }
        if (isFinite(ranges[b])) {         // 有回波：命中格 +l_occ
          const x = ox + dx * r, y = oy + dy * r;
          const c = (x / this.res) | 0, rr = (y / this.res) | 0;
          if (c >= 0 && c < W && rr >= 0 && rr < H) {
            log[rr * W + c] = Math.max(-L_MAX, Math.min(L_MAX,
              log[rr * W + c] + L_OCC));
          }
        }
      }
    }
    clear() { this.logodds.fill(0); }
    /* 二值化：log-odds 超过阈值视为占据（供 A* 规划使用） */
    binarize(th = 1.2) {
      const out = new Uint8Array(this.logodds.length);
      for (let i = 0; i < this.logodds.length; i++)
        out[i] = this.logodds[i] > th ? 1 : 0;
      return out;
    }
    /* 渲染到 offscreen canvas（暗色主题）：
       未知 → 深蓝（近乎融入面板底色），空闲 → 蓝灰，占据 → 亮白 */
    toCanvas(off) {
      const W = this.w, H = this.h;
      if (off.width !== W || off.height !== H) { off.width = W; off.height = H; }
      const img = off.getContext("2d").createImageData(W, H);
      for (let i = 0; i < W * H; i++) {
        const l = this.logodds[i];
        let r, g, b;
        if (l > 0.7) {            // 占据：亮
          r = 225; g = 232; b = 250;
        } else if (l < -0.7) {    // 空闲：蓝灰
          r = 66; g = 82; b = 126;
        } else if (l > 0.2) {     // 弱占据：过渡
          const t = (l - 0.2) / 0.5;
          r = 66 + (225 - 66) * t; g = 82 + (232 - 82) * t; b = 126 + (250 - 126) * t;
        } else if (l < -0.2) {    // 弱空闲：过渡
          const t = (-l - 0.2) / 0.5;
          r = 22 + (66 - 22) * t; g = 33 + (82 - 33) * t; b = 60 + (126 - 60) * t;
        } else {                  // 未知：融入面板
          r = 20; g = 31; b = 62;
        }
        img.data[i * 4] = r; img.data[i * 4 + 1] = g;
        img.data[i * 4 + 2] = b; img.data[i * 4 + 3] = 255;
      }
      off.getContext("2d").putImageData(img, 0, 0);
    }
  }

  /* ---------------- ICP（2D 闭式旋转，无 SVD 依赖） ----------------
     icpStep: 单步迭代（最近点匹配 + 闭式刚体拟合），供动画逐步演示 */
  function icpStep(ref, cur, T6, trim = 0.8) {
    const apply = (T, pts) => pts.map(p => [
      T[0] * p[0] + T[1] * p[1] + T[2],
      T[3] * p[0] + T[4] * p[1] + T[5]]);
    const transformed = apply(T6, cur);
    const matches = [];
    for (const p of transformed) {
      let bd = Infinity, bi = -1;
      for (let j = 0; j < ref.length; j++) {
        const q = ref[j], d = (p[0]-q[0])**2 + (p[1]-q[1])**2;
        if (d < bd) { bd = d; bi = j; }
      }
      matches.push([p, ref[bi], bd]);
    }
    matches.sort((a, b) => a[2] - b[2]);
    const keep = matches.slice(0, Math.max(4, (matches.length * trim) | 0));
    const rmse = Math.sqrt(keep.reduce((s, m) => s + m[2], 0) / keep.length);
    // 闭式刚体拟合（去质心 + 2D 最优旋转 atan2 公式）
    let sa = [0, 0], sb = [0, 0];
    for (const m of keep) { sa[0]+=m[0][0]; sa[1]+=m[0][1]; sb[0]+=m[1][0]; sb[1]+=m[1][1]; }
    sa = [sa[0]/keep.length, sa[1]/keep.length];
    sb = [sb[0]/keep.length, sb[1]/keep.length];
    let Sxx=0, Sxy=0, Syx=0, Syy=0;
    for (const m of keep) {
      const ax = m[0][0]-sa[0], ay = m[0][1]-sa[1];
      const bx = m[1][0]-sb[0], by = m[1][1]-sb[1];
      Sxx += ax*bx; Sxy += ax*by; Syx += ay*bx; Syy += ay*by;
    }
    const dth = Math.atan2(Sxy - Syx, Sxx + Syy);
    const c = Math.cos(dth), s = Math.sin(dth);
    const dT = [c, -s, sb[0] - (c*sa[0] - s*sa[1]),
                s,  c, sb[1] - (s*sa[0] + c*sa[1])];
    const T = [dT[0]*T6[0] + dT[1]*T6[3], dT[0]*T6[1] + dT[1]*T6[4],
               dT[0]*T6[2] + dT[1]*T6[5] + dT[2],
               dT[3]*T6[0] + dT[4]*T6[3], dT[3]*T6[1] + dT[4]*T6[4],
               dT[3]*T6[2] + dT[4]*T6[5] + dT[5]];
    return { T, rmse, matches, transformed };
  }

  function icp2d(ref, cur, initPose, maxIter = 20, trim = 0.8) {
    let T = [Math.cos(initPose[2]), -Math.sin(initPose[2]), initPose[0],
             Math.sin(initPose[2]),  Math.cos(initPose[2]), initPose[1]];
    let rmse = Infinity, iters = 0, last = null;
    for (let it = 0; it < maxIter; it++) {
      last = icpStep(ref, cur, T, trim);
      const prev = rmse;
      T = last.T; rmse = last.rmse; iters = it + 1;
      if (prev - rmse < 1e-5) break;
    }
    return { T, rmse, iters, matches: last ? last.matches : [] };
  }
  const se2fromT6 = T => [T[2], T[5], Math.atan2(T[3], T[0])];
  const se2toT6 = p => [Math.cos(p[2]), -Math.sin(p[2]), p[0],
                        Math.sin(p[2]),  Math.cos(p[2]), p[1]];
  function applyT6(T, pts) {
    return pts.map(p => [T[0]*p[0]+T[1]*p[1]+T[2], T[3]*p[0]+T[4]*p[1]+T[5]]);
  }

  /* ---------------- 机器人（与 robot.py 相同） ---------------- */
  class Robot {
    constructor(world, pose0, dt = 0.05, rand = rng(7)) {
      this.world = world;
      this.pose = pose0.slice();          // 真值
      this.odom = pose0.slice();          // 里程计
      this.dt = dt;
      this.rand = rand;
      this.beams = 120;
      this.fov = TAU;
      this.maxRange = 9.0;
      this.sigma = 0.02;
      this.angles = new Float64Array(this.beams);
      for (let i = 0; i < this.beams; i++) this.angles[i] = (i / this.beams) * this.fov;
      this.stdV = 0.045; this.stdW = 0.055;      // 里程计噪声（网站演示同款）
    }
    command(v, w) {
      const r = this.rand;
      this.pose = integrate(this.pose, v + gauss(r) * 0.008, w + gauss(r) * 0.010, this.dt);
      const vm = v + gauss(r) * this.stdV;
      const wm = w + gauss(r) * this.stdW;
      this.odom = integrate(this.odom, vm, wm, this.dt);
      return [vm, wm];
    }
    scan() {
      const raw = this.world.raycast(this.pose, this.angles, this.maxRange);
      for (let i = 0; i < raw.length; i++) {
        if (isFinite(raw[i])) raw[i] += gauss(this.rand) * this.sigma;
      }
      return raw;
    }
    /* 极坐标 → 机体系点云 */
    scanToLocal(ranges) {
      const pts = [];
      for (let i = 0; i < ranges.length; i++) {
        if (!isFinite(ranges[i])) continue;
        pts.push([ranges[i] * Math.cos(this.angles[i]),
                  ranges[i] * Math.sin(this.angles[i])]);
      }
      return pts;
    }
    apply(T6, pts) { return applyT6(T6, pts); }
  }

  /* ---------------- 航点跟随 ---------------- */
  function makeFollower(robot, wps, speed = 0.55, tol = 0.30) {
    let idx = 0;
    return function step() {
      if (idx >= wps.length) return [0, 0, true];
      const p = robot.pose, t = wps[idx];
      const dx = t[0] - p[0], dy = t[1] - p[1];
      if (Math.hypot(dx, dy) < tol) { idx++; return step(); }
      let err = wrap(Math.atan2(dy, dx) - p[2]);
      const v = Math.abs(err) < 0.7 ? speed : 0.35 * speed;
      const w = clamp(2.2 * err, -1.5, 1.5);
      return [v, w, false];
    };
  }
  const ROUTE = [[17.5, 2.0], [17.5, 9.8], [8.0, 9.8], [4.0, 9.5],
                 [2.0, 6.0], [2.0, 2.5]];

  /* ---------------- A*（与 run_astar.py 相同） ---------------- */
  function buildGrid(world, res = 0.2, robotR = 0.35) {
    const W = Math.round(world.size[0] / res), H = Math.round(world.size[1] / res);
    const occ = new Uint8Array(W * H);
    // 1) 线段栅格化
    for (let s = 0; s < world.segs.length; s += 4) {
      const [x1, y1, x2, y2] = world.segs.slice(s, s + 4);
      const L = Math.max(1, (Math.hypot(x2 - x1, y2 - y1) / (res / 2)) | 0);
      for (let k = 0; k <= L; k++) {
        const t = k / L;
        const c = clamp(((x1 + (x2-x1)*t) / res) | 0, 0, W - 1);
        const r = clamp(((y1 + (y2-y1)*t) / res) | 0, 0, H - 1);
        occ[r * W + c] = 1;
      }
    }
    // 2) 实心矩形内部填充
    for (const [cx, cy, w, h] of world.rects) {
      const c0 = clamp(((cx - w/2) / res) | 0, 0, W - 1), c1 = clamp(((cx + w/2) / res) | 0, 0, W - 1);
      const r0 = clamp(((cy - h/2) / res) | 0, 0, H - 1), r1 = clamp(((cy + h/2) / res) | 0, 0, H - 1);
      for (let r = r0; r <= r1; r++) for (let c = c0; c <= c1; c++) occ[r * W + c] = 1;
    }
    // 3) 膨胀：自由格到最近障碍（矩形/墙线）的距离 < robotR 则不可走
    const dist = (x, y) => {
      let d = Infinity;
      for (let s = 0; s < world.segs.length; s += 4) {
        const [ax, ay, bx, by] = world.segs.slice(s, s + 4);
        const ex = bx - ax, ey = by - ay;
        let t = ((x - ax) * ex + (y - ay) * ey) / (ex*ex + ey*ey);
        t = clamp(t, 0, 1);
        d = Math.min(d, Math.hypot(x - (ax + ex * t), y - (ay + ey * t)));
      }
      for (const [cx, cy, w, h] of world.rects) {
        const dx = Math.max(Math.abs(x - cx) - w/2, 0);
        const dy = Math.max(Math.abs(y - cy) - h/2, 0);
        d = Math.min(d, Math.hypot(dx, dy));       // 内部为 0
      }
      return d;
    };
    const grid = new Uint8Array(W * H);
    for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) {
      const x = (c + 0.5) * res, y = (r + 0.5) * res;
      if (occ[r * W + c] || dist(x, y) < robotR) grid[r * W + c] = 1;
    }
    return { grid, W, H, res, blocked: grid, occ, dist };
  }

  function astar(g, startRC, goalRC) {
    const { grid, W, H } = g;
    const start = startRC[0] * W + startRC[1], goal = goalRC[0] * W + goalRC[1];
    if (grid[start] || grid[goal]) return null;
    const N = W * H;
    const gcost = new Float64Array(N).fill(Infinity);
    const parent = new Int32Array(N).fill(-1);
    const closed = new Uint8Array(N);
    const SQ2 = Math.SQRT2;
    const gr = goalRC[0], gc = goalRC[1];
    const hFn = i => {
      const r = (i / W) | 0, c = i % W;
      const dr = Math.abs(r - gr), dc = Math.abs(c - gc);
      return dr + dc + (SQ2 - 2) * Math.min(dr, dc);
    };
    // 小顶堆（数组版，规模小足够快）
    const heap = [];
    const push = (f, i) => { heap.push([f, i]); let k = heap.length - 1;
      while (k > 0) { const p = (k-1)>>1; if (heap[p][0] <= heap[k][0]) break;
        [heap[p], heap[k]] = [heap[k], heap[p]]; k = p; } };
    const pop = () => { const top = heap[0], last = heap.pop();
      if (heap.length) { heap[0] = last; let k = 0;
        for (;;) { const l = 2*k+1, r = l+1; let m = k;
          if (l < heap.length && heap[l][0] < heap[m][0]) m = l;
          if (r < heap.length && heap[r][0] < heap[m][0]) m = r;
          if (m === k) break; [heap[m], heap[k]] = [heap[k], heap[m]]; k = m; } }
      return top; };
    gcost[start] = 0; push(hFn(start), start);
    const nb = [[-1,0,1],[1,0,1],[0,-1,1],[0,1,1],[-1,-1,SQ2],[-1,1,SQ2],[1,-1,SQ2],[1,1,SQ2]];
    const expanded = [];
    let found = false;
    while (heap.length) {
      const [f, cur] = pop();
      if (closed[cur]) continue;
      closed[cur] = 1; expanded.push(cur);
      if (cur === goal) { found = true; break; }
      const r = (cur / W) | 0, c = cur % W;
      for (const [dr, dc, w] of nb) {
        const nr = r + dr, nc = c + dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        const ni = nr * W + nc;
        if (grid[ni] || closed[ni]) continue;
        if (dr !== 0 && dc !== 0 && (grid[r * W + nc] || grid[nr * W + c])) continue; // 防穿角
        const ng = gcost[cur] + w;
        if (ng < gcost[ni]) { gcost[ni] = ng; parent[ni] = cur; push(ng + hFn(ni), ni); }
      }
    }
    if (!found) return null;
    const path = [];
    for (let i = goal; i !== -1; i = parent[i]) path.push(i);
    path.reverse();
    return { path, expanded };
  }

  function stringPull(g, pathIdx) {
    const { W, res, blocked } = g;
    const pts = pathIdx.map(i => [((i % W) + 0.5) * res, (((i / W) | 0) + 0.5) * res]);
    const los = (a, b) => {
      const n = Math.max(2, (Math.hypot(b[0]-a[0], b[1]-a[1]) / (res/2)) | 0);
      for (let k = 0; k <= n; k++) {
        const t = k / n;
        const c = clamp(((a[0] + (b[0]-a[0])*t) / res) | 0, 0, W - 1);
        const r = clamp(((a[1] + (b[1]-a[1])*t) / res) | 0, 0, g.H - 1);
        if (blocked[r * W + c]) return false;
      }
      return true;
    };
    const out = [pts[0]];
    let i = 0;
    while (i < pts.length - 1) {
      let j = pts.length - 1;
      while (j > i + 1 && !los(pts[i], pts[j])) j--;
      out.push(pts[j]); i = j;
    }
    return out;
  }

  /* ---------------- MPC 轻量版（无约束凝聚 QP + 限幅） ----------------
     完整 OSQP 约束版见 code/mini_slam/run_mpc.py。
     N: 预测步数, dt: 控制周期。决策变量 z = [Δv_0, Δw_0, Δv_1, ...] */
  function makeMPC(N = 25, dt = 0.05, opts = {}) {
    const V_MAX = opts.vMax ?? 1.0, V_MIN = opts.vMin ?? 0.0, W_MAX = opts.wMax ?? 1.6;
    const DW = opts.dwMax ?? 3.0;
    const Q = [opts.qx ?? 3.0, opts.qy ?? 3.0, opts.qth ?? 0.8];
    const R = [opts.rv ?? 0.2, opts.rw ?? 0.2];
    const QF = opts.qf ?? 8.0;
    const NZ = 2 * N;

    function solve(state, ref, wPrev) {
      // ref: (N+1)×5 [x,y,θ,v,ω]
      const e0 = [state[0]-ref[0][0], state[1]-ref[0][1],
                  wrap(state[2]-ref[0][2])];
      const mulM = (A, B) => {          // 3×3
        const C = new Array(9);
        for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
          C[i*3+j] = A[i*3]*B[j] + A[i*3+1]*B[3+j] + A[i*3+2]*B[6+j];
        return C;
      };
      let Phi = [1,0,0, 0,1,0, 0,0,1];
      let acc = null;                    // (3, NZ) 扁平：acc[i*NZ+a]
      const Cacc = [0, 0, 0];
      const Su = [], E = [];
      for (let k = 0; k < N; k++) {
        const [x, y, th, v, w] = ref[k];
        const [xn, yn, thn] = ref[k+1];
        const c = Math.cos(th), s = Math.sin(th);
        const A = [1, 0, -v*s*dt,  0, 1, v*c*dt,  0, 0, 1];
        const B = [c*dt, 0, s*dt, 0, 0, dt];
        Phi = mulM(A, Phi);
        // 注意：三项必须用"同一时刻"的 Cacc 计算（先存副本再回写）
        const c0 = A[0]*Cacc[0] + A[1]*Cacc[1] + A[2]*Cacc[2] + (x + v*c*dt - xn);
        const c1 = A[3]*Cacc[0] + A[4]*Cacc[1] + A[5]*Cacc[2] + (y + v*s*dt - yn);
        const c2 = A[6]*Cacc[0] + A[7]*Cacc[1] + A[8]*Cacc[2] + wrap(th + w*dt - thn);
        Cacc[0] = c0; Cacc[1] = c1; Cacc[2] = c2;
        const out = new Float64Array(NZ * 3);
        if (acc) for (let i = 0; i < 3; i++) {
          for (let a = 0; a < NZ; a++) {
            out[i*NZ+a] = A[i*3]*acc[a] + A[i*3+1]*acc[NZ+a] + A[i*3+2]*acc[2*NZ+a];
          }
        }
        out[0*NZ + 2*k]   += B[0];       // ∂e_x/∂Δv_k
        out[1*NZ + 2*k]   += B[2];       // ∂e_y/∂Δv_k
        out[2*NZ + 2*k+1] += B[5];       // ∂e_θ/∂Δw_k
        acc = out; Su.push(acc);
        E.push([ Phi[0]*e0[0]+Phi[1]*e0[1]+Phi[2]*e0[2] + Cacc[0],
                 Phi[3]*e0[0]+Phi[4]*e0[1]+Phi[5]*e0[2] + Cacc[1],
                 Phi[6]*e0[0]+Phi[7]*e0[1]+Phi[8]*e0[2] + Cacc[2] ]);
      }
      // H = Σ Suᵀ Qk Su + blkdiag(R)；q = Σ Suᵀ Qk E
      const H = new Float64Array(NZ * NZ);
      const q = new Float64Array(NZ);
      for (let k = 0; k < N; k++) {
        const S = Su[k], e = E[k];
        const f = (k === N - 1) ? QF : 1.0;
        const Qk = [Q[0]*f, Q[1]*f, Q[2]*f];
        for (let a = 0; a < NZ; a++) {
          let qa = 0;
          for (let i = 0; i < 3; i++) qa += S[i*NZ+a] * Qk[i] * e[i];
          q[a] += qa;
          for (let b = a; b < NZ; b++) {
            let v = 0;
            for (let i = 0; i < 3; i++) v += S[i*NZ+a] * Qk[i] * S[i*NZ+b];
            H[a*NZ+b] += v;
            if (a === b) H[a*NZ+b] += (a % 2 === 0 ? R[0] : R[1]);
          }
        }
      }
      for (let a = 0; a < NZ; a++) for (let b = 0; b < a; b++) H[a*NZ+b] = H[b*NZ+a];
      // 高斯消元解 H z = -q
      const M = new Float64Array(NZ * (NZ + 1));
      for (let i = 0; i < NZ; i++) {
        for (let j = 0; j < NZ; j++) M[i*(NZ+1)+j] = H[i*NZ+j];
        M[i*(NZ+1)+NZ] = -q[i];
      }
      for (let i = 0; i < NZ; i++) {
        let piv = i;
        for (let r = i + 1; r < NZ; r++)
          if (Math.abs(M[r*(NZ+1)+i]) > Math.abs(M[piv*(NZ+1)+i])) piv = r;
        if (piv !== i) for (let c = i; c <= NZ; c++) {
          const t = M[i*(NZ+1)+c]; M[i*(NZ+1)+c] = M[piv*(NZ+1)+c]; M[piv*(NZ+1)+c] = t;
        }
        const d = M[i*(NZ+1)+i] || 1e-9;
        for (let r = i + 1; r < NZ; r++) {
          const fac = M[r*(NZ+1)+i] / d;
          if (!fac) continue;
          for (let c = i; c <= NZ; c++) M[r*(NZ+1)+c] -= fac * M[i*(NZ+1)+c];
        }
      }
      const z = new Float64Array(NZ);
      for (let i = NZ - 1; i >= 0; i--) {
        let sv = M[i*(NZ+1)+NZ];
        for (let j = i + 1; j < NZ; j++) sv -= M[i*(NZ+1)+j] * z[j];
        z[i] = sv / (M[i*(NZ+1)+i] || 1e-9);
      }
      let v = clamp(ref[0][3] + z[0], V_MIN, V_MAX);
      let w = clamp(ref[0][4] + z[1], -W_MAX, W_MAX);
      w = clamp(w, wPrev - DW * dt, wPrev + DW * dt);     // 角加速度限幅
      return { v, w };
    }
    return { solve, N, dt };
  }

  /* ---------------- 参考轨迹（闭环 Catmull-Rom + 弧长重采样） ---------------- */
  function makeRefTrack(laps = 2, ds = 0.03, vNom = 0.55, wMax = 1.6) {
    const wp = ROUTE;
    const n = wp.length;
    const P = i => wp[((i % n) + n) % n];
    const raw = [];
    const SEG = 400;                       // 每段采样数
    for (let i = 0; i < n * laps; i++) {
      const p0 = P(i - 1), p1 = P(i), p2 = P(i + 1), p3 = P(i + 2);
      for (let j = 0; j < SEG; j++) {
        const t = j / SEG, t2 = t * t, t3 = t2 * t;
        // Catmull-Rom（0.5 系数）
        const x = 0.5 * ((2*p1[0]) + (-p0[0]+p2[0])*t +
                (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3);
        const y = 0.5 * ((2*p1[1]) + (-p0[1]+p2[1])*t +
                (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3);
        raw.push([x, y]);
      }
    }
    // 曲率（有限差分）→ 限速调度
    const M = raw.length;
    const d1 = [], d2 = [];
    for (let i = 0; i < M; i++) {
      const a = raw[(i - 1 + M) % M], b = raw[(i + 1) % M];
      d1.push([(b[0]-a[0]), (b[1]-a[1])]);
    }
    for (let i = 0; i < M; i++) {
      const a = d1[(i - 1 + M) % M], b = d1[(i + 1) % M];
      d2.push([b[0]-a[0], b[1]-a[1]]);
    }
    const kap = raw.map((_, i) => {
      const [dx, dy] = d1[i], [ddx, ddy] = d2[i];
      const den = Math.pow(dx*dx + dy*dy, 1.5);
      return den > 1e-9 ? Math.abs(dx*ddy - dy*ddx) / den : 0;
    });
    const vr = [], wr = [];
    for (let i = 0; i < M; i++) {
      const v = Math.min(vNom, 0.9 * wMax / Math.max(kap[i], 1e-6));
      vr.push(v); wr.push(kap[i] * v);
    }
    // 弧长均匀重采样
    const s = [0];
    for (let i = 1; i <= M; i++) {
      const a = raw[i - 1], b = raw[i % M];
      s.push(s[i-1] + Math.hypot(b[0]-a[0], b[1]-a[1]));
    }
    const total = s[M] / laps;             // 单圈弧长
    const nPer = Math.max(2, Math.round(total / ds));
    const out = { xs: [], ys: [], th: [], vr: [], wr: [], ds: 0 };
    const thRaw = [];
    for (let i = 0; i < M; i++)
      thRaw.push(Math.atan2(d1[i][1], d1[i][0]));
    // 单圈重采样（注意：target 只在*一圈*的弧长内取 —— 之前误用全长导致
    // 参考线以 2 倍速前进、机器人永远在追赶），再复制 laps 圈
    const oneLap = { xs: [], ys: [], th: [], vr: [], wr: [] };
    for (let k = 0; k < nPer; k++) {
      const target = (k / nPer) * total;
      let i = 0;
      while (i < M - 1 && s[i + 1] < target) i++;
      const f = (target - s[i]) / Math.max(s[i+1] - s[i], 1e-9);
      oneLap.xs.push(raw[i][0] + (raw[(i+1)%M][0]-raw[i][0]) * f);
      oneLap.ys.push(raw[i][1] + (raw[(i+1)%M][1]-raw[i][1]) * f);
      oneLap.th.push(thRaw[i]);
      oneLap.vr.push(vr[i]);
      oneLap.wr.push(wr[i]);
    }
    for (let L = 0; L < laps; L++) {
      out.xs.push(...oneLap.xs); out.ys.push(...oneLap.ys);
      out.th.push(...oneLap.th); out.vr.push(...oneLap.vr); out.wr.push(...oneLap.wr);
    }
    out.ds = total / nPer;
    return out;
  }

  /* ---------------- 仿真基类 ---------------- */
  class SimBase {
    constructor(canvasId, opts = {}) {
      this.canvas = typeof canvasId === "string" ?
        document.getElementById(canvasId) : canvasId;
      if (!this.canvas) return;
      this.ctx = this.canvas.getContext("2d");
      this.running = false;
      this.raf = null;
      this.lastT = 0;
      this.acc = 0;
      this.speed = opts.speed ?? 1;
      this._onResize = () => this.resize();
      window.addEventListener("resize", this._onResize);
      // 首次绘制延迟到子类构造完成后（状态就绪），避免"画到 undefined"
      queueMicrotask(() => this.resize());
    }
    resize() {
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
      if (!w || !h) return;
      this.canvas.width = Math.round(w * dpr);
      this.canvas.height = Math.round(h * dpr);
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.cssW = w; this.cssH = h;
      this._applyFit();
      if (this.onViewResize) this.onViewResize();
      if (!this._fit || (this.view && isFinite(this.view.s))) this.draw();
    }
    /* 世界坐标 → 画布像素（保持等比，含留白）。
       惰性：构造期间 cssW 尚未知 → 只存 bounds，resize 时再真正应用 */
    fitView(bounds, pad = 24) {
      this._fit = { bounds, pad };
      this._applyFit();
    }
    _applyFit() {
      if (!this._fit || !this.cssW) return;
      const [x0, y0, x1, y1] = this._fit.bounds;
      const pad = this._fit.pad;
      const s = Math.min((this.cssW - pad * 2) / (x1 - x0),
                         (this.cssH - pad * 2) / (y1 - y0));
      this.view = { s, x0, y0,
        ox: (this.cssW - (x1 - x0) * s) / 2,
        oy: (this.cssH - (y1 - y0) * s) / 2 };
      this.w2x = x => this.view.ox + (x - this.view.x0) * this.view.s;
      this.w2y = y => this.cssH - this.view.oy - (y - this.view.y0) * this.view.s;
    }
    setWorldMap(fn) {
      if (!this.w2x) {
        this.w2x = x => this.view.ox + (x - this.view.x0) * this.view.s;
        this.w2y = y => this.cssH - this.view.oy - (y - this.view.y0) * this.view.s;
      }
      if (fn) fn(this);
    }
    px2world(px, py) {
      return [(px - this.view.ox) / this.view.s + this.view.x0,
              (this.cssH - py - this.view.oy) / this.view.s + this.view.y0];
    }
    play() {
      if (this.running) return;
      this.running = true;
      this.lastT = performance.now();
      const loop = (t) => {
        if (!this.running) return;
        const dtReal = Math.min((t - this.lastT) / 1000, 0.05);
        this.lastT = t;
        this.acc += dtReal * this.speed;
        const h = this.fixedDt ?? 0.02;
        let n = 0;
        while (this.acc >= h && n < 8) { this.step(h); this.acc -= h; n++; }
        // 视图未就绪（构造未完成/画布 0 尺寸）时先不画
        if (!this._fit || (this.view && isFinite(this.view.s))) this.draw();
        this.raf = requestAnimationFrame(loop);
      };
      this.raf = requestAnimationFrame(loop);
      this.emit("state");
    }
    pause() { this.running = false;
      if (this.raf) cancelAnimationFrame(this.raf); this.emit("state"); }
    toggle() { this.running ? this.pause() : this.play(); }
    reset() { this.pause(); this.acc = 0; if (this.onReset) this.onReset(); this.draw(); this.emit("state"); }
    on(name, fn) { (this._h = this._h || {})[name] = fn; if (name === "state") fn(this); }
    emit(name) { if (this._h && this._h[name]) this._h[name](this); }
  }

  /* ---------------- 渲染助手 ---------------- */
  function drawWorld(ctx, sim, world, opt = {}) {
    const { color = "#42506b", lw = 1.6 } = opt;
    ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.lineCap = "round";
    ctx.beginPath();
    for (let s = 0; s < world.segs.length; s += 4) {
      ctx.moveTo(sim.w2x(world.segs[s]), sim.w2y(world.segs[s+1]));
      ctx.lineTo(sim.w2x(world.segs[s+2]), sim.w2y(world.segs[s+3]));
    }
    ctx.stroke();
    if (opt.fillRects) {
      ctx.fillStyle = opt.fillRects;
      for (const [cx, cy, w, h] of world.rects) {
        ctx.fillRect(sim.w2x(cx - w/2), sim.w2y(cy + h/2),
                     w * sim.view.s, h * sim.view.s);
      }
    }
  }
  function drawRobot(ctx, sim, pose, color = "#60a5fa", r = 5.5) {
    const x = sim.w2x(pose[0]), y = sim.w2y(pose[1]);
    ctx.save();
    ctx.translate(x, y); ctx.rotate(-pose[2]);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(r * 1.7, 0); ctx.lineTo(-r, r * 0.85); ctx.lineTo(-r * 0.5, 0);
    ctx.lineTo(-r, -r * 0.85); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  function drawPolyline(ctx, sim, pts, color, lw = 1.6, dash = null, alpha = 1) {
    if (!pts || pts.length < 2) return;
    ctx.save();
    if (dash) ctx.setLineDash(dash);
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color; ctx.lineWidth = lw;
    ctx.beginPath();
    ctx.moveTo(sim.w2x(pts[0][0]), sim.w2y(pts[0][1]));
    for (let i = 1; i < pts.length; i++)
      ctx.lineTo(sim.w2x(pts[i][0]), sim.w2y(pts[i][1]));
    ctx.stroke();
    ctx.restore();
  }
  function drawGridCanvas(ctx, sim, off, x0, y0, wM, hM, alpha = 1) {
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(off,
      sim.w2x(x0), sim.w2y(y0 + hM), wM * sim.view.s, hM * sim.view.s);
    ctx.restore();
  }

  return { TAU, wrap, clamp, rng, gauss, makeWorld, integrate,
           OccGrid, icp2d, icpStep, se2fromT6, se2toT6, applyT6,
           Robot, makeFollower, ROUTE, makeRefTrack,
           buildGrid, astar, stringPull, makeMPC,
           SimBase, drawWorld, drawRobot, drawPolyline, drawGridCanvas };
})();
