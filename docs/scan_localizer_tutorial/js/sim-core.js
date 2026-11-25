// Pure JavaScript port of scan_localizer core math & localization algorithms.
// Dual-mode UMD module: works cleanly in browser (window.SlocSim) and Node.js tests.
(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.SlocSim = factory();
  }
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  const TWO_PI = 2 * Math.PI;

  function wrapAngle(a) {
    if (!Number.isFinite(a)) return 0.0;
    let r = a - TWO_PI * Math.trunc(a / TWO_PI);
    if (r <= -Math.PI) r += TWO_PI;
    else if (r > Math.PI) r -= TWO_PI;
    if (r <= -Math.PI) r += TWO_PI;
    if (r > Math.PI) r -= TWO_PI;
    return r;
  }

  function poseMul(a, b) {
    const c = Math.cos(a.theta), s = Math.sin(a.theta);
    return {
      x: a.x + c * b.x - s * b.y,
      y: a.y + s * b.x + c * b.y,
      theta: wrapAngle(a.theta + b.theta),
    };
  }

  function poseInv(a) {
    const c = Math.cos(a.theta), s = Math.sin(a.theta);
    return {
      x: -(c * a.x + s * a.y),
      y: -(-s * a.x + c * a.y),
      theta: wrapAngle(-a.theta),
    };
  }

  function transformScan(pose, pts) {
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    const out = new Array(pts.length);
    for (let i = 0; i < pts.length; i++) {
      const px = pts[i].x, py = pts[i].y;
      out[i] = { x: pose.x + c * px - s * py, y: pose.y + s * px + c * py };
    }
    return out;
  }

  function mapToOdom(mapToBase, odomToBase) {
    return poseMul(mapToBase, poseInv(odomToBase));
  }

  // ---------- Small Linear Algebra Helpers (3x3 & 4x4) ----------
  function invert3x3(A) {
    const a00 = A[0], a01 = A[1], a02 = A[2];
    const a10 = A[3], a11 = A[4], a12 = A[5];
    const a20 = A[6], a21 = A[7], a22 = A[8];
    const c00 = a11 * a22 - a12 * a21;
    const c01 = -(a10 * a22 - a12 * a20);
    const c02 = a10 * a21 - a11 * a20;
    const det = a00 * c00 + a01 * c01 + a02 * c02;
    if (!Number.isFinite(det) || Math.abs(det) < 1e-18) return null;
    const invDet = 1.0 / det;
    return [
      c00 * invDet, (a02 * a21 - a01 * a22) * invDet, (a01 * a12 - a02 * a11) * invDet,
      c01 * invDet, (a00 * a22 - a02 * a20) * invDet, (a02 * a10 - a00 * a12) * invDet,
      c02 * invDet, (a01 * a20 - a00 * a21) * invDet, (a00 * a11 - a01 * a10) * invDet,
    ];
  }

  function solve3x3(A, b) {
    const inv = invert3x3(A);
    if (!inv) return null;
    return [
      inv[0] * b[0] + inv[1] * b[1] + inv[2] * b[2],
      inv[3] * b[0] + inv[4] * b[1] + inv[5] * b[2],
      inv[6] * b[0] + inv[7] * b[1] + inv[8] * b[2],
    ];
  }

  // Jacobi eigen-decomposition for 3x3 symmetric matrix
  function eigenSym3x3(M) {
    const A = [
      [M[0], 0.5 * (M[1] + M[3]), 0.5 * (M[2] + M[6])],
      [0.5 * (M[1] + M[3]), M[4], 0.5 * (M[5] + M[7])],
      [0.5 * (M[2] + M[6]), 0.5 * (M[5] + M[7]), M[8]],
    ];
    const V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let sweep = 0; sweep < 30; sweep++) {
      let p = 0, q = 1, maxVal = Math.abs(A[0][1]);
      if (Math.abs(A[0][2]) > maxVal) { p = 0; q = 2; maxVal = Math.abs(A[0][2]); }
      if (Math.abs(A[1][2]) > maxVal) { p = 1; q = 2; maxVal = Math.abs(A[1][2]); }
      if (maxVal < 1e-15) break;
      const app = A[p][p], aqq = A[q][q], apq = A[p][q];
      const tau = (aqq - app) / (2 * apq);
      const t = tau >= 0 ? 1 / (tau + Math.sqrt(1 + tau * tau)) : -1 / (-tau + Math.sqrt(1 + tau * tau));
      const c = 1 / Math.sqrt(1 + t * t), s = t * c;
      A[p][p] = app - t * apq;
      A[q][q] = aqq + t * apq;
      A[p][q] = A[q][p] = 0;
      for (let r = 0; r < 3; r++) {
        if (r !== p && r !== q) {
          const arp = A[r][p], arq = A[r][q];
          A[r][p] = A[p][r] = c * arp - s * arq;
          A[r][q] = A[q][r] = s * arp + c * arq;
        }
        const vrp = V[r][p], vrq = V[r][q];
        V[r][p] = c * vrp - s * vrq;
        V[r][q] = s * vrp + c * vrq;
      }
    }
    const items = [0, 1, 2].map((i) => ({ val: A[i][i], vec: [V[0][i], V[1][i], V[2][i]] }));
    items.sort((a, b) => a.val - b.val);
    return { values: items.map((x) => x.val), vectors: items.map((x) => x.vec) };
  }

  function covFromHessian(H, resVar, minVarXY = 1e-4, minVarTh = 1.2e-5, inflation = 2.0) {
    const eig = eigenSym3x3(H);
    const kMinEig = 1e-4;
    const invEigs = eig.values.map((v) => 1.0 / Math.max(v, kMinEig));
    const V = eig.vectors; // V[col][row]
    const Hinv = new Array(9).fill(0);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        let sum = 0;
        for (let k = 0; k < 3; k++) sum += V[k][r] * invEigs[k] * V[k][c];
        Hinv[r * 3 + c] = sum;
      }
    }
    const scale = Math.max(resVar, minVarXY) * inflation;
    const cov = Hinv.map((v) => v * scale);
    cov[0] = Math.max(cov[0], minVarXY);
    cov[4] = Math.max(cov[4], minVarXY);
    cov[8] = Math.max(cov[8], minVarTh);
    return cov;
  }

  // ---------- OccupancyGrid & Sim Maps ----------
  class OccupancyGrid {
    constructor(width, height, resolution = 0.1, originX = 0, originY = 0) {
      this.width = width;
      this.height = height;
      this.resolution = resolution;
      this.originX = originX;
      this.originY = originY;
      this.cells = new Int8Array(width * height); // 0 free by default
    }
    inside(ix, iy) { return ix >= 0 && iy >= 0 && ix < this.width && iy < this.height; }
    occupied(ix, iy, occThresh = 65) { return this.cells[iy * this.width + ix] > occThresh; }
    worldToGridIndex(wx, wy) {
      return {
        ix: Math.floor((wx - this.originX) / this.resolution),
        iy: Math.floor((wy - this.originY) / this.resolution),
      };
    }
    worldToGridF(wx, wy) {
      return {
        gx: (wx - this.originX) / this.resolution - 0.5,
        gy: (wy - this.originY) / this.resolution - 0.5,
      };
    }
    gridToWorld(gx, gy) {
      return {
        x: this.originX + (gx + 0.5) * this.resolution,
        y: this.originY + (gy + 0.5) * this.resolution,
      };
    }
    addBox(x0, y0, x1, y1, val = 100) {
      const ix0 = Math.max(0, Math.floor((x0 - this.originX) / this.resolution));
      const iy0 = Math.max(0, Math.floor((y0 - this.originY) / this.resolution));
      const ix1 = Math.min(this.width - 1, Math.floor((x1 - this.originX) / this.resolution));
      const iy1 = Math.min(this.height - 1, Math.floor((y1 - this.originY) / this.resolution));
      for (let y = iy0; y <= iy1; y++) {
        for (let x = ix0; x <= ix1; x++) this.cells[y * this.width + x] = val;
      }
    }
  }

  function createWarehouseMap(res = 0.1) {
    const w = Math.round(24 / res), h = Math.round(16 / res);
    const g = new OccupancyGrid(w, h, res, 0, 0);
    // Outer walls
    g.addBox(0, 0, 24, 0.35);
    g.addBox(0, 15.65, 24, 16);
    g.addBox(0, 0, 0.35, 16);
    g.addBox(23.65, 0, 24, 16);
    // Internal partition wall with doorway
    g.addBox(8.2, 0.35, 8.55, 6.2);
    g.addBox(8.2, 9.6, 8.55, 15.65);
    // Shelves & pillars (asymmetric so global pose is unique)
    g.addBox(3.0, 4.0, 5.6, 5.2);
    g.addBox(3.0, 10.2, 5.6, 11.4);
    g.addBox(12.0, 3.0, 17.5, 4.1);
    g.addBox(12.0, 7.2, 16.5, 8.4);
    g.addBox(12.5, 11.6, 18.5, 12.8);
    g.addBox(19.8, 5.2, 21.0, 7.2);
    return g;
  }

  function createCorridorMap(res = 0.1) {
    const w = Math.round(30 / res), h = Math.round(5 / res);
    const g = new OccupancyGrid(w, h, res, 0, 0);
    g.addBox(0, 0, 30, 0.4);
    g.addBox(0, 4.6, 30, 5.0);
    g.addBox(0, 0, 0.4, 5.0);
    g.addBox(29.6, 0, 30, 5.0);
    return g;
  }

  function raycastRange(grid, px, py, worldAngle, minR = 0.08, maxR = 14.0, dynBoxes = []) {
    const dx = Math.cos(worldAngle), dy = Math.sin(worldAngle);
    const step = grid.resolution * 0.45;
    for (let r = minR; r <= maxR; r += step) {
      const wx = px + r * dx, wy = py + r * dy;
      for (let k = 0; k < dynBoxes.length; k++) {
        const b = dynBoxes[k];
        if (Math.abs(wx - b.cx) <= b.hw && Math.abs(wy - b.cy) <= b.hh) return r;
      }
      const { ix, iy } = grid.worldToGridIndex(wx, wy);
      if (!grid.inside(ix, iy)) break;
      if (grid.occupied(ix, iy)) return r;
    }
    return -1;
  }

  function makeScan(grid, pose, opts = {}) {
    const beams = opts.beams || 180;
    const fov = opts.fov !== undefined ? opts.fov : TWO_PI;
    const minR = opts.minRange || 0.08;
    const maxR = opts.maxRange || 14.0;
    const noiseStd = opts.noiseStd || 0.0;
    const dynBoxes = opts.dynBoxes || [];
    const pts = [];
    const rng = opts.rng || Math.random;
    const gauss = () => {
      let u = 0, v = 0;
      while (u === 0) u = rng();
      while (v === 0) v = rng();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(TWO_PI * v);
    };
    const startA = -0.5 * fov;
    const dA = fov / beams;
    for (let i = 0; i < beams; i++) {
      const localA = startA + i * dA;
      let r = raycastRange(grid, pose.x, pose.y, pose.theta + localA, minR, maxR, dynBoxes);
      if (r < 0) continue;
      if (noiseStd > 0) r = Math.max(minR, r + noiseStd * gauss());
      pts.push({ x: r * Math.cos(localA), y: r * Math.sin(localA), r, a: localA });
    }
    return pts;
  }

  // ---------- Exact 1D & 2D EDT + MapField ----------
  function floorDiv(num, den) {
    let q = Math.trunc(num / den);
    const r = num % den;
    if (r !== 0 && ((r < 0) !== (den < 0))) q -= 1;
    return q;
  }

  function edt1dWithTrace(f, captureTrace = true) {
    const n = f.length;
    const out = new Array(n).fill(0);
    if (n === 0) return { out, v: [], z: [], steps: [] };
    const INF_Z = 1e12;
    const v = new Array(n).fill(0);
    const z = new Array(n + 1).fill(0);
    let k = 0;
    v[0] = 0;
    z[0] = -INF_Z;
    z[1] = INF_Z;
    const steps = captureTrace ? [{ q: 0, v: [0], z: [-INF_Z, INF_Z], popped: [] }] : [];
    for (let q = 1; q < n; q++) {
      const popped = [];
      let p = v[k];
      let s = floorDiv((f[q] + q * q) - (f[p] + p * p), 2 * (q - p));
      while (s <= z[k]) {
        popped.push(v[k]);
        k--;
        p = v[k];
        s = floorDiv((f[q] + q * q) - (f[p] + p * p), 2 * (q - p));
      }
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = INF_Z;
      if (captureTrace) steps.push({ q, v: v.slice(0, k + 1), z: z.slice(0, k + 2), popped });
    }
    let kk = 0;
    for (let q = 0; q < n; q++) {
      while (z[kk + 1] < q) kk++;
      const p = v[kk];
      const d = q - p;
      out[q] = d * d + f[p];
    }
    return { out, v: v.slice(0, k + 1), z: z.slice(0, k + 2), steps };
  }

  function edt1d(f) {
    return edt1dWithTrace(f, false).out;
  }

  function edt2d(width, height, seeds) {
    const n = width * height;
    const dist2 = new Float64Array(n);
    if (width <= 0 || height <= 0) return dist2;
    const INF = width + height + 2;
    const g = new Int32Array(n);
    for (let x = 0; x < width; x++) {
      for (let y = 0; y < height; y++) {
        const i = y * width + x;
        g[i] = seeds[i] ? 0 : (y > 0 ? Math.min(INF, g[i - width] + 1) : INF);
      }
      for (let y = height - 2; y >= 0; y--) {
        const i = y * width + x;
        if (g[i + width] + 1 < g[i]) g[i] = g[i + width] + 1;
      }
    }
    const f = new Array(width);
    for (let y = 0; y < height; y++) {
      const row = y * width;
      for (let x = 0; x < width; x++) {
        const gx = g[row + x];
        f[x] = gx * gx;
      }
      const rowOut = edt1d(f);
      for (let x = 0; x < width; x++) dist2[row + x] = rowOut[x];
    }
    return dist2;
  }

  class MapField {
    constructor(grid, sigmaM = 0.20, occThresh = 65, freeThresh = 25, maxPoolLevels = 8) {
      this.grid = grid;
      this.sigmaM = sigmaM;
      this.occThresh = occThresh;
      this.freeThresh = freeThresh;
      this.unknownScore = 0.10;
      const w = grid.width, h = grid.height, n = w * h;
      const occSeeds = new Uint8Array(n);
      const freeSeeds = new Uint8Array(n);
      let hasOcc = false, hasFree = false;
      for (let i = 0; i < n; i++) {
        const c = grid.cells[i];
        if (c > occThresh) { occSeeds[i] = 1; hasOcc = true; }
        else if (c >= 0 && c <= freeThresh) { freeSeeds[i] = 1; hasFree = true; }
      }
      const dOcc = hasOcc ? edt2d(w, h, occSeeds) : null;
      const dFree = hasFree ? edt2d(w, h, freeSeeds) : null;
      this.sdfCells = new Float32Array(n);
      this.like = new Float32Array(n);
      const res = grid.resolution;
      const s2 = 2 * sigmaM * sigmaM;
      const maxDistM = Math.hypot(w, h) * res;
      const unknownBound = Math.max(res, 3 * sigmaM);

      for (let i = 0; i < n; i++) {
        const c = grid.cells[i];
        let sdfM;
        if (c > occThresh) {
          sdfM = hasFree ? -(Math.sqrt(dFree[i]) - 1.0) * res : -maxDistM;
          this.sdfCells[i] = sdfM;
          this.like[i] = Math.exp(-(sdfM * sdfM) / s2);
        } else if (c >= 0 && c <= freeThresh) {
          sdfM = hasOcc ? Math.sqrt(dOcc[i]) * res : maxDistM;
          this.sdfCells[i] = sdfM;
          this.like[i] = Math.exp(-(sdfM * sdfM) / s2);
        } else {
          sdfM = hasFree ? -Math.min(Math.sqrt(dFree[i]) * res, unknownBound) : -unknownBound;
          this.sdfCells[i] = sdfM;
          this.like[i] = this.unknownScore;
        }
      }

      // Sliding window max pyramid (Cartographer PrecomputationGrid style)
      this.swmax = [this.like];
      const maxDim = Math.max(w, h);
      while (this.swmax.length < maxPoolLevels && (1 << this.swmax.length) < maxDim) {
        const src = this.swmax[this.swmax.length - 1];
        const span = 1 << (this.swmax.length - 1);
        const dst = new Float32Array(n);
        for (let y = 0; y < h; y++) {
          const yh = Math.min(y + span, h - 1);
          const ry = y * w, ryh = yh * w;
          for (let x = 0; x < w; x++) {
            const xh = Math.min(x + span, w - 1);
            let m = src[ry + x];
            if (src[ry + xh] > m) m = src[ry + xh];
            if (src[ryh + x] > m) m = src[ryh + x];
            if (src[ryh + xh] > m) m = src[ryh + xh];
            dst[ry + x] = m;
          }
        }
        this.swmax.push(dst);
      }
    }

    poolLevels() { return this.swmax.length; }

    poolScore(level, px, py) {
      const w = this.grid.width, h = this.grid.height;
      const cx = Math.min(Math.max(px, 0), w - 1);
      const cy = Math.min(Math.max(py, 0), h - 1);
      return this.swmax[level][cy * w + cx];
    }

    likelihood(ix, iy) {
      if (!this.grid.inside(ix, iy)) return 0.0;
      const idx = iy * this.grid.width + ix;
      const c = this.grid.cells[idx];
      if (c < 0 || (c > this.freeThresh && c <= this.occThresh)) return this.unknownScore;
      return this.like[idx];
    }

    likelihoodAtF(gx, gy) {
      return this.likelihood(Math.round(gx), Math.round(gy));
    }

    sdf(gx, gy, gradOut = null) {
      const w = this.grid.width, h = this.grid.height;
      const cx = Math.min(Math.max(gx, 0), w - 1);
      const cy = Math.min(Math.max(gy, 0), h - 1);
      const x0 = w > 1 ? Math.min(Math.floor(cx), w - 2) : 0;
      const y0 = h > 1 ? Math.min(Math.floor(cy), h - 2) : 0;
      const x1 = w > 1 ? x0 + 1 : 0;
      const y1 = h > 1 ? y0 + 1 : 0;
      const fx = w > 1 ? cx - x0 : 0;
      const fy = h > 1 ? cy - y0 : 0;
      const v00 = this.sdfCells[y0 * w + x0];
      const v10 = this.sdfCells[y0 * w + x1];
      const v01 = this.sdfCells[y1 * w + x0];
      const v11 = this.sdfCells[y1 * w + x1];
      if (gradOut) {
        const res = this.grid.resolution;
        gradOut.x = ((v10 - v00) * (1 - fy) + (v11 - v01) * fy) / res;
        gradOut.y = ((v01 - v00) * (1 - fx) + (v11 - v10) * fx) / res;
      }
      return v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy) + v01 * (1 - fx) * fy + v11 * fx * fy;
    }

    descendToObstacle(ix, iy) {
      const w = this.grid.width, h = this.grid.height;
      let cx = Math.min(Math.max(ix, 0), w - 1);
      let cy = Math.min(Math.max(iy, 0), h - 1);
      for (let step = 0; step < Math.max(w, h); step++) {
        const curVal = this.grid.cells[cy * w + cx];
        if (curVal > this.occThresh) return { ix: cx, iy: cy };
        let bestX = cx, bestY = cy;
        let bestSdf = this.sdfCells[cy * w + cx];
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            const nx = cx + dx, ny = cy + dy;
            if (!this.grid.inside(nx, ny)) continue;
            const s = this.sdfCells[ny * w + nx];
            if (s < bestSdf) { bestSdf = s; bestX = nx; bestY = ny; }
          }
        }
        if (bestX === cx && bestY === cy) break;
        cx = bestX; cy = bestY;
      }
      return { ix: cx, iy: cy };
    }
  }

  function evalStats(field, pose, pts, inlierGate = 0.12) {
    let s = 0, inl = 0, n = 0;
    const c = Math.cos(pose.theta), sn = Math.sin(pose.theta);
    for (let i = 0; i < pts.length; i++) {
      const wx = pose.x + c * pts[i].x - sn * pts[i].y;
      const wy = pose.y + sn * pts[i].x + c * pts[i].y;
      const g = field.grid.worldToGridF(wx, wy);
      s += field.likelihoodAtF(g.gx, g.gy);
      if (Math.abs(field.sdf(g.gx, g.gy)) < inlierGate) inl++;
      n++;
    }
    return { score: n > 0 ? s / n : 0, inlierRatio: n > 0 ? inl / n : 0 };
  }

  // ---------- Matchers ----------
  function correlativeMatch(field, prior, pts, opt = {}) {
    const windowLin = opt.windowLin !== undefined ? opt.windowLin : 0.30;
    const windowAng = opt.windowAng !== undefined ? opt.windowAng : 0.20;
    const stepLin = opt.stepLin || 0.06;
    const stepAng = opt.stepAng || 0.04;
    const nx = Math.max(1, Math.ceil((2 * windowLin) / stepLin)) + 1;
    const ny = nx;
    const na = Math.max(1, Math.ceil((2 * windowAng) / stepAng)) + 1;
    let bestS = -1, bestPose = { ...prior };
    const heatmap = [];
    for (let ai = 0; ai < na; ai++) {
      const th = wrapAngle(prior.theta + (ai - 0.5 * (na - 1)) * stepAng);
      const c = Math.cos(th), s = Math.sin(th);
      const rotCells = pts.map((p) => field.grid.worldToGridF(prior.x + c * p.x - s * p.y, prior.y + s * p.x + c * p.y));
      for (let yi = 0; yi < ny; yi++) {
        const dy = (yi - 0.5 * (ny - 1)) * stepLin;
        const offy = dy / field.grid.resolution;
        for (let xi = 0; xi < nx; xi++) {
          const dx = (xi - 0.5 * (nx - 1)) * stepLin;
          const offx = dx / field.grid.resolution;
          let sc = 0;
          for (let k = 0; k < rotCells.length; k++) {
            sc += field.likelihood(Math.round(rotCells[k].gx + offx), Math.round(rotCells[k].gy + offy));
          }
          sc /= pts.length;
          if (ai === Math.floor(na / 2)) heatmap.push({ xi, yi, dx, dy, score: sc });
          if (sc > bestS) {
            bestS = sc;
            bestPose = { x: prior.x + dx, y: prior.y + dy, theta: th };
          }
        }
      }
    }
    const stats = evalStats(field, bestPose, pts, opt.inlierGate || 0.12);
    return {
      pose: bestPose,
      score: stats.score,
      inlierRatio: stats.inlierRatio,
      heatmap,
      nx,
      ny,
      valid: bestS > 0,
    };
  }

  function fieldGnStep(field, pose, pts, opt = {}) {
    const huberDelta = opt.huberDelta !== undefined ? opt.huberDelta : 0.12;
    const trimDist = opt.trimDist !== undefined ? opt.trimDist : 0.65;
    const useHuber = opt.useHuber !== false;
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    const H = new Array(9).fill(0);
    const b = [0, 0, 0];
    const grad = { x: 0, y: 0 };
    let rsum = 0, rw = 0, used = 0;
    let samplePoint = null;
    const forces = [];
    for (let i = 0; i < pts.length; i++) {
      const wx = pose.x + c * pts[i].x - s * pts[i].y;
      const wy = pose.y + s * pts[i].x + c * pts[i].y;
      const g = field.grid.worldToGridF(wx, wy);
      const r = field.sdf(g.gx, g.gy, grad);
      if (Math.abs(r) > trimDist) continue;
      const ar = Math.abs(r);
      const w = useHuber ? (ar <= huberDelta ? 1.0 : huberDelta / Math.max(ar, 1e-9)) : 1.0;
      const jth = grad.x * (-(wy - pose.y)) + grad.y * (wx - pose.x);
      const J = [grad.x, grad.y, jth];
      if (!samplePoint) samplePoint = { wx, wy, r, w, J: J.slice() };
      for (let r0 = 0; r0 < 3; r0++) {
        b[r0] += w * J[r0] * r;
        for (let c0 = 0; c0 < 3; c0++) H[r0 * 3 + c0] += w * J[r0] * J[c0];
      }
      rsum += w * r * r;
      rw += w;
      used++;
      if (i % 4 === 0) forces.push({ wx, wy, r, w, gx: grad.x, gy: grad.y });
    }
    if (used < 5) return { ok: false, nextPose: pose, H, b, ravg: Infinity, used, forces, samplePoint };
    const ravg = rsum / Math.max(rw, 1e-9);
    const Hd = H.slice();
    Hd[0] += 1e-6 * Math.max(H[0], 1.0);
    Hd[4] += 1e-6 * Math.max(H[4], 1.0);
    Hd[8] += 1e-6 * Math.max(H[8], 1.0);
    const dp = solve3x3(Hd, [-b[0], -b[1], -b[2]]);
    if (!dp) return { ok: false, nextPose: pose, H, b, Hd, ravg, used, forces, samplePoint };
    const nl = Math.hypot(dp[0], dp[1]);
    const clip = Math.min(1.0, Math.min(0.2 / Math.max(nl, 1e-9), 0.1 / Math.max(Math.abs(dp[2]), 1e-9)));
    const nextPose = {
      x: pose.x + clip * dp[0],
      y: pose.y + clip * dp[1],
      theta: wrapAngle(pose.theta + clip * dp[2]),
    };
    return {
      ok: true,
      nextPose,
      H,
      b,
      Hd,
      dp,
      clip,
      ravg,
      rmse: Math.sqrt(ravg),
      used,
      stepNorm: nl * clip,
      stepAng: Math.abs(dp[2]) * clip,
      forces,
      samplePoint,
    };
  }

  function fieldGnMatch(field, prior, pts, opt = {}) {
    const maxIters = opt.maxIterations || 20;
    const convT = opt.convergT || 1e-4;
    const convR = opt.convergR || 1e-5;
    let p = { ...prior, theta: wrapAngle(prior.theta) };
    let bestRavg = Infinity, bestPose = p, bestH = null, bestVar = 1e-6;
    const history = [];
    for (let it = 0; it < maxIters; it++) {
      const st = fieldGnStep(field, p, pts, opt);
      if (!st.ok) break;
      history.push({ iter: it, pose: { ...p }, rmse: st.rmse, used: st.used });
      if (st.ravg < bestRavg) {
        bestRavg = st.ravg;
        bestPose = { ...p };
        bestH = st.H;
        bestVar = Math.max(st.ravg, 1e-6);
      }
      p = st.nextPose;
      if (st.stepNorm < convT && st.stepAng < convR) break;
    }
    if (!bestH) return { valid: false, pose: prior, covariance: [1,0,0,0,1,0,0,0,1], score: 0, inlierRatio: 0, history };
    const endSt = fieldGnStep(field, p, pts, opt);
    if (endSt.ok && endSt.ravg < bestRavg) {
      bestRavg = endSt.ravg;
      bestPose = p;
      bestH = endSt.H;
      bestVar = Math.max(endSt.ravg, 1e-6);
    }
    const cov = covFromHessian(bestH, bestVar);
    const stats = evalStats(field, bestPose, pts, opt.inlierGate || 0.12);
    return {
      valid: true,
      pose: bestPose,
      hessian: bestH,
      covariance: cov,
      score: stats.score,
      inlierRatio: stats.inlierRatio,
      rmse: Math.sqrt(bestRavg),
      history,
    };
  }

  // 可直接手算的 Field-GN 一轮迭代：竖直墙 + 5 个对称点。
  // 页面数值表与自动化测试共用同一实例，避免教学数字与仿真核漂移。
  function createFieldGnTeachingExample() {
    const grid = new OccupancyGrid(80, 60, 0.1, -4.0, -3.0);
    grid.cells.fill(0);
    for (let y = 0; y < grid.height; y++) grid.cells[y * grid.width + 40] = 100;
    const field = new MapField(grid, 0.2);
    const pose = { x: -0.15, y: 0, theta: 0 };
    const points = [-2, -1, 0, 1, 2].map((y) => ({ x: 0, y }));
    const options = { useHuber: true, huberDelta: 0.1, trimDist: 0.65 };
    return { grid, field, pose, points, options, step: fieldGnStep(field, pose, points, options) };
  }

  function icpP2pStep(field, pose, pts, opt = {}) {
    const huberDelta = opt.huberDelta || 0.12;
    const trimDist = opt.trimDist || 0.80;
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    const H = new Array(9).fill(0);
    const b = [0, 0, 0];
    let rsum = 0, rw = 0, used = 0;
    const pairs = [];
    for (let i = 0; i < pts.length; i++) {
      const wx = pose.x + c * pts[i].x - s * pts[i].y;
      const wy = pose.y + s * pts[i].x + c * pts[i].y;
      const g = field.grid.worldToGridF(wx, wy);
      const ob = field.descendToObstacle(Math.round(g.gx), Math.round(g.gy));
      const qw = field.grid.gridToWorld(ob.ix, ob.iy);
      const ex = wx - qw.x, ey = wy - qw.y;
      const e2 = ex * ex + ey * ey;
      if (e2 > trimDist * trimDist) continue;
      const e = Math.sqrt(e2);
      const w = e <= huberDelta ? 1.0 : huberDelta / Math.max(e, 1e-9);
      // 2x3 Jacobian [1, 0, -(wy-py); 0, 1, wx-px]
      const j02 = -(wy - pose.y), j12 = wx - pose.x;
      const J0 = [1, 0, j02], J1 = [0, 1, j12];
      for (let r0 = 0; r0 < 3; r0++) {
        b[r0] += w * (J0[r0] * ex + J1[r0] * ey);
        for (let c0 = 0; c0 < 3; c0++) H[r0 * 3 + c0] += w * (J0[r0] * J0[c0] + J1[r0] * J1[c0]);
      }
      rsum += w * e2;
      rw += w;
      used++;
      if (i % 5 === 0) pairs.push({ wx, wy, qx: qw.x, qy: qw.y });
    }
    if (used < 5) return { ok: false, nextPose: pose, pairs };
    const Hd = H.slice();
    Hd[0] += 1e-6 * Math.max(H[0], 1);
    Hd[4] += 1e-6 * Math.max(H[4], 1);
    Hd[8] += 1e-6 * Math.max(H[8], 1);
    const dp = solve3x3(Hd, [-b[0], -b[1], -b[2]]);
    if (!dp) return { ok: false, nextPose: pose, pairs };
    const nl = Math.hypot(dp[0], dp[1]);
    const clip = Math.min(1.0, Math.min(0.25 / Math.max(nl, 1e-9), 0.15 / Math.max(Math.abs(dp[2]), 1e-9)));
    return {
      ok: true,
      nextPose: { x: pose.x + clip * dp[0], y: pose.y + clip * dp[1], theta: wrapAngle(pose.theta + clip * dp[2]) },
      rmse: Math.sqrt(rsum / Math.max(rw, 1e-9)),
      pairs,
    };
  }

  function ndtBuild(grid, voxelM = 0.8) {
    const vw = Math.max(1, Math.ceil((grid.width * grid.resolution) / voxelM));
    const vh = Math.max(1, Math.ceil((grid.height * grid.resolution) / voxelM));
    const buckets = Array.from({ length: vw * vh }, () => []);
    for (let y = 0; y < grid.height; y++) {
      for (let x = 0; x < grid.width; x++) {
        if (!grid.occupied(x, y)) continue;
        const w = grid.gridToWorld(x, y);
        const vx = Math.min(Math.max(Math.floor((w.x - grid.originX) / voxelM), 0), vw - 1);
        const vy = Math.min(Math.max(Math.floor((w.y - grid.originY) / voxelM), 0), vh - 1);
        buckets[vy * vw + vx].push(w);
      }
    }
    const gauss = [];
    for (let i = 0; i < buckets.length; i++) {
      const b = buckets[i];
      if (b.length < 3) continue;
      let mx = 0, my = 0;
      for (const p of b) { mx += p.x; my += p.y; }
      mx /= b.length; my /= b.length;
      let c00 = 0, c01 = 0, c11 = 0;
      for (const p of b) {
        const dx = p.x - mx, dy = p.y - my;
        c00 += dx * dx; c01 += dx * dy; c11 += dy * dy;
      }
      const denom = b.length - 1;
      c00 = c00 / denom + 0.002 * voxelM * voxelM;
      c01 = c01 / denom;
      c11 = c11 / denom + 0.002 * voxelM * voxelM;
      gauss.push({ mx, my, c00, c01, c11 });
    }
    return { voxelM, vw, vh, gauss };
  }

  // ---------- Global Correlative Branch-and-Bound ----------
  function globalCorrelativeBnB(field, pts, opts = {}) {
    const step = Math.max(1, opts.beamsSubsample || 3);
    const angStep = opts.angStep || 0.25;
    const nHyp = opts.nHypotheses || 3;
    const sub = [];
    for (let i = 0; i < pts.length; i += step) sub.push(pts[i]);
    if (sub.length < 5) return { valid: false, poses: [], trace: [] };

    const grid = field.grid;
    const W = grid.width, H = grid.height;
    let fxmin = W, fxmax = -1, fymin = H, fymax = -1;
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const c = grid.cells[y * W + x];
        if (c >= 0 && c <= 65) {
          if (x < fxmin) fxmin = x;
          if (x > fxmax) fxmax = x;
          if (y < fymin) fymin = y;
          if (y > fymax) fymax = y;
        }
      }
    }
    const K = field.poolLevels() - 1;
    const nAng = Math.ceil(TWO_PI / angStep);
    const ang = [];
    for (let a = 0; a < nAng; a++) {
      const th = a * angStep;
      const cc = Math.cos(th), ss = Math.sin(th);
      const cells = sub.map((pt) => ({
        cx: Math.round((cc * pt.x - ss * pt.y) / grid.resolution),
        cy: Math.round((ss * pt.x + cc * pt.y) / grid.resolution),
      }));
      ang.push({ th, cells });
    }
    const evalBound = (level, bx, by, A) => {
      const span = 1 << level;
      let sc = 0;
      for (let i = 0; i < A.cells.length; i++) {
        const px = A.cells[i].cx + bx, py = A.cells[i].cy + by;
        if (px >= W || px + span <= 0 || py >= H || py + span <= 0) continue;
        sc += field.poolScore(level, Math.max(px, 0), Math.max(py, 0));
      }
      return sc / sub.length;
    };

    // Simple max-heap priority queue
    const pq = [];
    const pushPQ = (node) => {
      pq.push(node);
      let idx = pq.length - 1;
      while (idx > 0) {
        const par = (idx - 1) >> 1;
        if (pq[par].bound >= pq[idx].bound) break;
        const t = pq[par]; pq[par] = pq[idx]; pq[idx] = t;
        idx = par;
      }
    };
    const popPQ = () => {
      const top = pq[0];
      const last = pq.pop();
      if (pq.length > 0) {
        pq[0] = last;
        let idx = 0;
        for (;;) {
          let l = 2 * idx + 1, r = l + 1, lg = idx;
          if (l < pq.length && pq[l].bound > pq[lg].bound) lg = l;
          if (r < pq.length && pq[r].bound > pq[lg].bound) lg = r;
          if (lg === idx) break;
          const t = pq[idx]; pq[idx] = pq[lg]; pq[lg] = t;
          idx = lg;
        }
      }
      return top;
    };

    const G = 1 << K;
    const x0 = Math.floor(fxmin / G) * G, y0 = Math.floor(fymin / G) * G;
    for (let a = 0; a < nAng; a++) {
      for (let by = y0; by <= fymax; by += G) {
        for (let bx = x0; bx <= fxmax; bx += G) {
          const b = evalBound(K, bx, by, ang[a]);
          if (b > 0.05) pushPQ({ bound: b, level: K, bx, by, aidx: a });
        }
      }
    }

    const hyps = [];
    const trace = [];
    let worstKept = 0, expanded = 0, pruned = 0;
    const sameMode = (a, b) => {
      const dm = Math.hypot((a.dx - b.dx) * grid.resolution, (a.dy - b.dy) * grid.resolution);
      const da = Math.abs(wrapAngle((a.aidx - b.aidx) * angStep));
      return dm < 0.8 && da < 0.45;
    };
    while (pq.length > 0 && expanded < 150000) {
      const nd = popPQ();
      if (hyps.length >= nHyp && nd.bound <= worstKept) {
        pruned += pq.length + 1;
        break;
      }
      if (nd.level === 0) {
        if (nd.bx < fxmin || nd.bx > fxmax || nd.by < fymin || nd.by > fymax) continue;
        let sc = 0;
        const A = ang[nd.aidx];
        for (let i = 0; i < A.cells.length; i++) sc += field.likelihood(A.cells[i].cx + nd.bx, A.cells[i].cy + nd.by);
        sc /= A.cells.length;
        if (sc > 0.15) {
          const cand = { score: sc, dx: nd.bx, dy: nd.by, aidx: nd.aidx };
          let merged = false;
          for (let i = 0; i < hyps.length; i++) {
            if (sameMode(hyps[i], cand)) {
              if (cand.score > hyps[i].score) hyps[i] = cand;
              merged = true;
              break;
            }
          }
          if (!merged) {
            if (hyps.length < nHyp) hyps.push(cand);
            else {
              let wi = 0;
              for (let i = 1; i < hyps.length; i++) if (hyps[i].score < hyps[wi].score) wi = i;
              if (cand.score > hyps[wi].score) hyps[wi] = cand;
            }
          }
          worstKept = hyps.length < nHyp ? 0 : Math.min(...hyps.map((h) => h.score));
        }
        continue;
      }
      const h = 1 << (nd.level - 1);
      for (let dy = 0; dy < 2; dy++) {
        for (let dx = 0; dx < 2; dx++) {
          const bx = nd.bx + dx * h, by = nd.by + dy * h;
          if (bx > fxmax || by > fymax || bx + h - 1 < fxmin || by + h - 1 < fymin) continue;
          const b = evalBound(nd.level - 1, bx, by, ang[nd.aidx]);
          if (b > 0.05 && (hyps.length < nHyp || b > worstKept)) {
            pushPQ({ bound: b, level: nd.level - 1, bx, by, aidx: nd.aidx });
            if (trace.length < 360) trace.push({ level: nd.level - 1, bx, by, span: h, bound: b, aidx: nd.aidx });
          } else {
            pruned++;
          }
          expanded++;
        }
      }
    }
    hyps.sort((a, b) => b.score - a.score);
    const poses = hyps.map((h) => {
      const w = grid.gridToWorld(h.dx, h.dy);
      return { x: w.x, y: w.y, theta: wrapAngle(h.aidx * angStep), score: h.score };
    });
    return { valid: poses.length > 0, pose: poses[0] || null, poses, expanded, pruned, trace };
  }

  // ---------- Particle Filter (AMCL-lite) ----------
  class ParticleFilter {
    constructor(nParticles = 400, rng = Math.random) {
      this.nParticles = nParticles;
      this.rng = rng;
      this.particles = [];
    }
    gauss() {
      let u = 0, v = 0;
      while (u === 0) u = this.rng();
      while (v === 0) v = this.rng();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(TWO_PI * v);
    }
    initGlobal(grid) {
      const free = [];
      for (let y = 0; y < grid.height; y++) {
        for (let x = 0; x < grid.width; x++) {
          const c = grid.cells[y * grid.width + x];
          if (c >= 0 && c <= 25) free.push({ ix: x, iy: y });
        }
      }
      this.particles = [];
      const w0 = 1.0 / this.nParticles;
      for (let i = 0; i < this.nParticles; i++) {
        const c = free[Math.floor(this.rng() * free.length)];
        const w = grid.gridToWorld(c.ix, c.iy);
        this.particles.push({ x: w.x, y: w.y, th: -Math.PI + this.rng() * TWO_PI, w: w0 });
      }
    }
    predict(odomDelta) {
      const dx = odomDelta.x, dy = odomDelta.y, dth = wrapAngle(odomDelta.theta);
      const trans = Math.hypot(dx, dy);
      let rot1 = trans < 1e-6 ? 0.0 : wrapAngle(Math.atan2(dy, dx));
      let signedTrans = trans;
      if (Math.abs(rot1) > 0.5 * Math.PI) {
        rot1 = wrapAngle(rot1 + Math.PI);
        signedTrans = -trans;
      }
      const rot2 = wrapAngle(dth - rot1);
      const sdR1 = Math.sqrt(0.08 * rot1 * rot1 + 0.05 * trans * trans + 1e-6);
      const sdT = Math.sqrt(0.05 * trans * trans + 0.05 * (rot1 * rot1 + rot2 * rot2) + 1e-6);
      const sdR2 = Math.sqrt(0.08 * rot2 * rot2 + 0.05 * trans * trans + 1e-6);
      for (let i = 0; i < this.particles.length; i++) {
        const p = this.particles[i];
        const dh1 = rot1 - sdR1 * this.gauss();
        const dt = signedTrans - sdT * this.gauss();
        const dh2 = rot2 - sdR2 * this.gauss();
        const sx = dt * Math.cos(dh1), sy = dt * Math.sin(dh1);
        const c = Math.cos(p.th), s = Math.sin(p.th);
        p.x += c * sx - s * sy;
        p.y += s * sx + c * sy;
        p.th = wrapAngle(p.th + dh1 + dh2);
      }
    }
    update(field, pts, subsample = 3) {
      const sub = [];
      for (let i = 0; i < pts.length; i += subsample) sub.push(pts[i]);
      const beamScale = Math.min(1.0, 30.0 / Math.max(1, sub.length));
      const logW = new Float64Array(this.particles.length);
      let maxLw = -Infinity;
      for (let i = 0; i < this.particles.length; i++) {
        const p = this.particles[i];
        const c = Math.cos(p.th), s = Math.sin(p.th);
        let ll = 0;
        for (let k = 0; k < sub.length; k++) {
          const wx = p.x + c * sub[k].x - s * sub[k].y;
          const wy = p.y + s * sub[k].x + c * sub[k].y;
          const g = field.grid.worldToGridF(wx, wy);
          const lk = field.likelihoodAtF(g.gx, g.gy);
          ll += Math.log(0.9 * lk + 0.1);
        }
        const lw = Math.log(Math.max(p.w, 1e-300)) + beamScale * ll;
        logW[i] = lw;
        if (lw > maxLw) maxLw = lw;
      }
      let wsum = 0;
      for (let i = 0; i < this.particles.length; i++) {
        const w = Math.exp(logW[i] - maxLw);
        this.particles[i].w = w;
        wsum += w;
      }
      let essInv = 0;
      for (let i = 0; i < this.particles.length; i++) {
        this.particles[i].w /= wsum;
        essInv += this.particles[i].w * this.particles[i].w;
      }
      const essRatio = (1.0 / essInv) / this.particles.length;
      if (essRatio < 0.5) this.resample();
      return essRatio;
    }
    resample() {
      const n = this.particles.length;
      const out = new Array(n);
      const step = 1.0 / n;
      let r = this.rng() * step;
      let c = this.particles[0].w, idx = 0;
      for (let m = 0; m < n; m++) {
        const u = r + m * step;
        while (u > c && idx < n - 1) { idx++; c += this.particles[idx].w; }
        const src = this.particles[idx];
        out[m] = { x: src.x, y: src.y, th: src.th, w: step };
      }
      this.particles = out;
    }
    estimate() {
      let x = 0, y = 0, cs = 0, sn = 0;
      for (const p of this.particles) {
        x += p.w * p.x; y += p.w * p.y;
        cs += p.w * Math.cos(p.th); sn += p.w * Math.sin(p.th);
      }
      return { x, y, theta: Math.atan2(sn, cs) };
    }
  }

  // ---------- 4D Anisotropic EKF + 4 Gates + 5-State Machine ----------
  class LocalizerCore4D {
    constructor(field, opts = {}) {
      this.field = field;
      this.cfg = Object.assign({
        minScore: 0.35,
        minInlierRatio: 0.55,
        mahalGate: 9.0,
        maxCovEig: 1.0,
        odomTransVarPerM: 4e-4,
        odomRotVarPerM: 2e-3,
        odomRotVarPerRad: 1e-3,
        imuRotVarPerRad: 2e-5,
        gyroBiasInitVar: 1e-2,
        gyroBiasRwVarPerS: 1e-6,
        maxGyroBias: 0.5,
        degradedAfter: 4,
        lostAfter: 18,
        anomalyWindow: 8,
        anomalyThreshold: 6,
        anomalyLowScore: 0.25,
        jumpMinDist: 1.0,
        jumpScoreMargin: 0.15,
      }, opts);
      this.reset({ x: 0, y: 0, theta: 0 });
    }
    reset(pose, initCov3 = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.02]) {
      this.pose = { ...pose, theta: wrapAngle(pose.theta) };
      this.gyroBias = 0.0;
      this.P4 = [
        [initCov3[0], 0, 0, 0],
        [0, initCov3[4], 0, 0],
        [0, 0, initCov3[8], 0],
        [0, 0, 0, this.cfg.gyroBiasInitVar],
      ];
      this.state = "TRACKING";
      this.rejectStreak = 0;
      this.anomalyWin = [];
    }
    step(odomDelta, scanPts, imuInfo = null) {
      if (this.state === "GLOBAL" || this.state === "LOST" || this.state === "UNINIT") {
        // Mirror C++ LocalizerCore::on_scan top-level recovery path without overwriting GLOBAL->DEGRADED
        if (this.cfg.autoGlobalRecover) {
          const g = globalCorrelativeBnB(this.field, scanPts, { beamsSubsample: 3, angStep: 0.25, nHypotheses: 1 });
          if (g.valid && g.pose) {
            this.reset(g.pose);
            return { pose: { ...this.pose }, gyroBias: this.gyroBias, state: this.state, accepted: true, score: g.pose.score, inlierRatio: 1, mahal: 0, minEig: 0.01, gates: { score: true, inlier: true, cov: true, mahal: true }, rejectStreak: 0 };
          }
        }
        return { pose: { ...this.pose }, gyroBias: this.gyroBias, state: this.state, accepted: false, score: 0, inlierRatio: 0, mahal: Infinity, minEig: 1, gates: { score: false, inlier: false, cov: false, mahal: false }, rejectStreak: this.rejectStreak };
      }
      const c = Math.cos(this.pose.theta), s = Math.sin(this.pose.theta);
      let dth = odomDelta.theta;
      let imuUsed = false, dtImu = 0;
      if (imuInfo && imuInfo.dt > 0) {
        imuUsed = true;
        dtImu = imuInfo.dt;
        dth = wrapAngle((imuInfo.wz - this.gyroBias) * dtImu);
      }
      const pred = {
        x: this.pose.x + c * odomDelta.x - s * odomDelta.y,
        y: this.pose.y + s * odomDelta.x + c * odomDelta.y,
        theta: wrapAngle(this.pose.theta + dth),
      };
      // F4
      const F4 = [
        [1, 0, -(odomDelta.x * s + odomDelta.y * c), 0],
        [0, 1, odomDelta.x * c - odomDelta.y * s, 0],
        [0, 0, 1, imuUsed ? -dtImu : 0],
        [0, 0, 0, 1],
      ];
      const dtrans = Math.hypot(odomDelta.x, odomDelta.y);
      const qt = dtrans * this.cfg.odomTransVarPerM + 1e-6;
      const qr = dtrans * this.cfg.odomRotVarPerM +
        Math.abs(dth) * (imuUsed ? this.cfg.imuRotVarPerRad : this.cfg.odomRotVarPerRad) +
        (imuUsed ? dtImu * this.cfg.imuRotVarPerRad : 1e-6);
      const Q4 = [
        [c * c * qt + s * s * qt, 0, 0, 0],
        [0, s * s * qt + c * c * qt, 0, 0],
        [0, 0, qr, 0],
        [0, 0, 0, imuUsed ? this.cfg.gyroBiasRwVarPerS * dtImu : 0],
      ];
      // P_pred = F4 * P4 * F4^T + Q4
      const FP = Array.from({ length: 4 }, (_, i) =>
        Array.from({ length: 4 }, (_, j) =>
          F4[i][0] * this.P4[0][j] + F4[i][1] * this.P4[1][j] + F4[i][2] * this.P4[2][j] + F4[i][3] * this.P4[3][j]
        )
      );
      const Ppred = Array.from({ length: 4 }, (_, i) =>
        Array.from({ length: 4 }, (_, j) =>
          FP[i][0] * F4[j][0] + FP[i][1] * F4[j][1] + FP[i][2] * F4[j][2] + FP[i][3] * F4[j][3] + Q4[i][j]
        )
      );

      // Scan match around pred
      const mr = fieldGnMatch(this.field, pred, scanPts);
      const R = mr.covariance;
      const gateScore = mr.valid && mr.score >= this.cfg.minScore;
      const gateInlier = mr.valid && mr.inlierRatio >= this.cfg.minInlierRatio;
      const eigR = eigenSym3x3(R);
      const minEig = Math.min(...eigR.values);
      const gateCov = mr.valid && minEig > 0 && minEig < this.cfg.maxCovEig;

      const S = [
        Ppred[0][0] + R[0] + 1e-9, Ppred[0][1] + R[1], Ppred[0][2] + R[2],
        Ppred[1][0] + R[3], Ppred[1][1] + R[4] + 1e-9, Ppred[1][2] + R[5],
        Ppred[2][0] + R[6], Ppred[2][1] + R[7], Ppred[2][2] + R[8] + 1e-9,
      ];
      const Sinv = invert3x3(S);
      const innov = [mr.pose.x - pred.x, mr.pose.y - pred.y, wrapAngle(mr.pose.theta - pred.theta)];
      let mahal = Infinity, gateMahal = false;
      if (mr.valid && Sinv) {
        const SinvInn = [
          Sinv[0] * innov[0] + Sinv[1] * innov[1] + Sinv[2] * innov[2],
          Sinv[3] * innov[0] + Sinv[4] * innov[1] + Sinv[5] * innov[2],
          Sinv[6] * innov[0] + Sinv[7] * innov[1] + Sinv[8] * innov[2],
        ];
        mahal = innov[0] * SinvInn[0] + innov[1] * SinvInn[1] + innov[2] * SinvInn[2];
        gateMahal = Number.isFinite(mahal) && mahal >= 0 && mahal < this.cfg.mahalGate;
      }

      const accept = Boolean(mr.valid && gateScore && gateInlier && gateCov && gateMahal && Sinv);
      const isBad = !accept && (!mr.valid || mr.score < this.cfg.anomalyLowScore || (!gateInlier && !gateScore));
      this.anomalyWin.push(isBad ? 1 : 0);
      if (this.anomalyWin.length > this.cfg.anomalyWindow) this.anomalyWin.shift();

      let K4 = null;
      let dx = null;
      if (accept) {
        // K4 = Ppred[:,0:3] * Sinv (4x3)
        K4 = Array.from({ length: 4 }, (_, i) => [
          Ppred[i][0] * Sinv[0] + Ppred[i][1] * Sinv[3] + Ppred[i][2] * Sinv[6],
          Ppred[i][0] * Sinv[1] + Ppred[i][1] * Sinv[4] + Ppred[i][2] * Sinv[7],
          Ppred[i][0] * Sinv[2] + Ppred[i][1] * Sinv[5] + Ppred[i][2] * Sinv[8],
        ]);
        dx = K4.map((row) => row[0] * innov[0] + row[1] * innov[1] + row[2] * innov[2]);
        this.pose = { x: pred.x + dx[0], y: pred.y + dx[1], theta: wrapAngle(pred.theta + dx[2]) };
        this.gyroBias = Math.min(Math.max(this.gyroBias + dx[3], -this.cfg.maxGyroBias), this.cfg.maxGyroBias);

        // Joseph update
        const IKH = Array.from({ length: 4 }, (_, i) =>
          Array.from({ length: 4 }, (_, j) => (i === j ? 1 : 0) - (j < 3 ? K4[i][j] : 0))
        );
        const tmp = Array.from({ length: 4 }, (_, i) =>
          Array.from({ length: 4 }, (_, j) =>
            IKH[i][0] * Ppred[0][j] + IKH[i][1] * Ppred[1][j] + IKH[i][2] * Ppred[2][j] + IKH[i][3] * Ppred[3][j]
          )
        );
        for (let i = 0; i < 4; i++) {
          for (let j = 0; j < 4; j++) {
            let v = tmp[i][0] * IKH[j][0] + tmp[i][1] * IKH[j][1] + tmp[i][2] * IKH[j][2] + tmp[i][3] * IKH[j][3];
            for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) v += K4[i][a] * R[a * 3 + b] * K4[j][b];
            this.P4[i][j] = v;
          }
        }
        this.rejectStreak = 0;
        this.state = "TRACKING";
      } else {
        this.pose = pred;
        this.P4 = Ppred.map((row) => row.slice());
        // Stationary Mahalanobis lockout relaxation (98fcd28)
        if (mr.valid && gateInlier && gateScore && gateCov && !gateMahal) {
          for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) this.P4[i][j] *= 1.5;
          this.P4[0][0] = Math.min(this.P4[0][0] + 2e-4, 0.25);
          this.P4[1][1] = Math.min(this.P4[1][1] + 2e-4, 0.25);
          this.P4[2][2] = Math.min(this.P4[2][2] + 2e-5, 0.10);
        }
        this.rejectStreak++;
        if (this.rejectStreak >= this.cfg.lostAfter) { this.state = "LOST"; this.rejectStreak = 0; }
        else if (this.rejectStreak >= this.cfg.degradedAfter) { this.state = "DEGRADED"; }
        const sumBad = this.anomalyWin.reduce((a, b) => a + b, 0);
        if (sumBad >= this.cfg.anomalyThreshold) { this.state = "GLOBAL"; this.anomalyWin = []; }
      }
      const trace = {
        pred: { ...pred },
        F4: F4.map((row) => row.slice()),
        Q4: Q4.map((row) => row.slice()),
        Ppred: Ppred.map((row) => row.slice()),
        match: { pose: { ...mr.pose }, valid: mr.valid, score: mr.score, inlierRatio: mr.inlierRatio },
        R: R.slice(),
        innov: innov.slice(),
        S: S.slice(),
        K4: K4 ? K4.map((row) => row.slice()) : null,
        dx: dx ? dx.slice() : null,
        accepted: accept,
        Ppost: this.P4.map((row) => row.slice()),
      };
      return {
        pose: { ...this.pose },
        gyroBias: this.gyroBias,
        state: this.state,
        accepted: accept,
        score: mr.score,
        inlierRatio: mr.inlierRatio,
        mahal,
        minEig,
        gates: { score: gateScore, inlier: gateInlier, cov: gateCov, mahal: gateMahal },
        rejectStreak: this.rejectStreak,
        trace,
      };
    }
  }

  return {
    TWO_PI,
    wrapAngle,
    poseMul,
    poseInv,
    transformScan,
    mapToOdom,
    invert3x3,
    solve3x3,
    eigenSym3x3,
    covFromHessian,
    OccupancyGrid,
    createWarehouseMap,
    createCorridorMap,
    raycastRange,
    makeScan,
    floorDiv,
    edt1dWithTrace,
    edt1d,
    edt2d,
    MapField,
    evalStats,
    correlativeMatch,
    fieldGnStep,
    fieldGnMatch,
    createFieldGnTeachingExample,
    icpP2pStep,
    ndtBuild,
    globalCorrelativeBnB,
    ParticleFilter,
    LocalizerCore4D,
  };
});
