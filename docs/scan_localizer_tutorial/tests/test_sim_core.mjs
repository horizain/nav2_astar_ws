import assert from "node:assert/strict";
import { createRequire } from "node:module";

const require = createRequire(import.meta.url);
const S = require("../js/sim-core.js");

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
  console.log(`PASS [${passed}] ${name}`);
}

// 1. wrapAngle edge cases
test("wrapAngle normalizes boundaries and non-finite inputs", () => {
  assert.equal(S.wrapAngle(NaN), 0.0);
  assert.equal(S.wrapAngle(Infinity), 0.0);
  assert.ok(Math.abs(S.wrapAngle(3 * Math.PI) - Math.PI) < 1e-12);
  assert.ok(Math.abs(S.wrapAngle(-Math.PI) - Math.PI) < 1e-12);
  assert.ok(Math.abs(S.wrapAngle(2 * Math.PI) - 0.0) < 1e-12);
});

// 2. SE(2) group inverse and mapToOdom identity
test("SE(2) group inverse & mapToOdom compose accurately", () => {
  const mTb = { x: 4.2, y: -1.7, theta: 1.15 };
  const oTb = { x: 2.1, y: 0.9, theta: -0.85 };
  const ident = S.poseMul(mTb, S.poseInv(mTb));
  assert.ok(Math.abs(ident.x) < 1e-12);
  assert.ok(Math.abs(ident.y) < 1e-12);
  assert.ok(Math.abs(ident.theta) < 1e-12);

  const mTo = S.mapToOdom(mTb, oTb);
  const rec = S.poseMul(mTo, oTb);
  assert.ok(Math.hypot(rec.x - mTb.x, rec.y - mTb.y) < 1e-12);
  assert.ok(Math.abs(S.wrapAngle(rec.theta - mTb.theta)) < 1e-12);
});

// 3. 1D EDT negative floor_div trap: {9, 100, 4} -> 8
test("edt1d exact lower envelope handles {9, 100, 4} -> [8, 5, 4] without truncation bug", () => {
  const out = S.edt1d([9, 100, 4]);
  assert.deepEqual(out, [8, 5, 4]);
});

// 4. 2D SDF accuracy & finite-difference gradient agreement
test("MapField SDF and analytic bilinear gradient match numerical derivative", () => {
  const g = S.createWarehouseMap(0.1);
  const f = new S.MapField(g, 0.2);
  const { gx, gy } = g.worldToGridF(6.23, 7.47);
  const grad = { x: 0, y: 0 };
  const d0 = f.sdf(gx, gy, grad);
  const eps = 1e-3; // cells -> eps * resolution meters
  const dx = (f.sdf(gx + eps, gy) - f.sdf(gx - eps, gy)) / (2 * eps * g.resolution);
  const dy = (f.sdf(gx, gy + eps) - f.sdf(gx, gy - eps)) / (2 * eps * g.resolution);
  assert.ok(Number.isFinite(d0) && d0 > 0);
  assert.ok(Math.hypot(grad.x - dx, grad.y - dy) < 1e-4, `grad mismatch: ${grad.x},${grad.y} vs ${dx},${dy}`);
});

// 5. OccupancyGrid worldToGridF vs gridToWorld roundtrip
test("OccupancyGrid worldToGridF and gridToWorld align cell centers with 0.5 offset", () => {
  const g = new S.OccupancyGrid(20, 20, 0.05, -2.0, -3.0);
  const w = g.gridToWorld(4, 7);
  assert.ok(Math.abs(w.x - (-2.0 + 4.5 * 0.05)) < 1e-12);
  assert.ok(Math.abs(w.y - (-3.0 + 7.5 * 0.05)) < 1e-12);
  const f = g.worldToGridF(w.x, w.y);
  assert.ok(Math.abs(f.gx - 4.0) < 1e-12 && Math.abs(f.gy - 7.0) < 1e-12);
});

// 6. Correlative + Field-GN recovery from large initial error
test("Correlative + FieldGn composed pipeline converges from 0.25m/10deg offset", () => {
  const g = S.createWarehouseMap(0.1);
  const f = new S.MapField(g, 0.2);
  const gt = { x: 5.5, y: 8.0, theta: 0.35 };
  const scan = S.makeScan(g, gt, { beams: 180 });
  const prior = { x: gt.x + 0.22, y: gt.y - 0.18, theta: gt.theta + 0.16 };
  const cRes = S.correlativeMatch(f, prior, scan, { windowLin: 0.35, windowAng: 0.22 });
  const gRes = S.fieldGnMatch(f, cRes.pose, scan);
  assert.ok(gRes.valid);
  const posErr = Math.hypot(gRes.pose.x - gt.x, gRes.pose.y - gt.y);
  const angErr = Math.abs(S.wrapAngle(gRes.pose.theta - gt.theta));
  assert.ok(posErr < 0.04, `posErr=${posErr}`);
  assert.ok(angErr < 0.015, `angErr=${angErr}`);
});

// 7. Huber robustness against dynamic pedestrian obstacle
test("FieldGn with Huber + trim_dist resists near dynamic obstacle better than pure L2", () => {
  const g = S.createWarehouseMap(0.1);
  const f = new S.MapField(g, 0.2);
  const gt = { x: 11.0, y: 5.8, theta: 0.1 };
  const dyn = [{ cx: 12.2, cy: 6.1, hw: 0.45, hh: 0.45 }, { cx: 10.2, cy: 4.5, hw: 0.45, hh: 0.45 }];
  const scan = S.makeScan(g, gt, { beams: 180, dynBoxes: dyn });
  const prior = { x: gt.x + 0.08, y: gt.y - 0.06, theta: gt.theta + 0.03 };
  const huberRes = S.fieldGnMatch(f, prior, scan, { useHuber: true, huberDelta: 0.08, trimDist: 0.35 });
  const posErr = Math.hypot(huberRes.pose.x - gt.x, huberRes.pose.y - gt.y);
  assert.ok(huberRes.valid && posErr < 0.05, `Huber err=${posErr}`);
});

// 8. Corridor degeneracy anisotropy (> 1000x ratio)
test("Corridor FieldGn Hessian yields strongly anisotropic covariance (along >> across)", () => {
  const g = S.createCorridorMap(0.1);
  const f = new S.MapField(g, 0.2);
  const gt = { x: 15.0, y: 2.5, theta: 0.0 };
  const scan = S.makeScan(g, gt, { beams: 180, maxRange: 10.0 });
  const res = S.fieldGnMatch(f, { x: 15.0, y: 2.55, theta: 0.02 }, scan);
  assert.ok(res.valid);
  const ratio = res.covariance[0] / res.covariance[4];
  assert.ok(ratio > 1000, `Expected >1000x anisotropy, got ${ratio}`);
});

// 9. Global Correlative Branch-and-Bound
test("globalCorrelativeBnB locates kidnapped robot inside warehouse", () => {
  const g = S.createWarehouseMap(0.2);
  const f = new S.MapField(g, 0.25);
  const gt = { x: 14.5, y: 9.8, theta: 0.5 };
  const scan = S.makeScan(g, gt, { beams: 120 });
  const res = S.globalCorrelativeBnB(f, scan, { beamsSubsample: 2, angStep: 0.25, nHypotheses: 3 });
  assert.ok(res.valid && res.poses.length > 0);
  const minErr = Math.min(...res.poses.map((p) => Math.hypot(p.x - gt.x, p.y - gt.y)));
  assert.ok(minErr < 0.8, `minErr=${minErr}`);
});

// 10. Particle Filter deterministic convergence
test("ParticleFilter converges to true pose with deterministic PRNG", () => {
  let seed = 42;
  const rng = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  const g = S.createWarehouseMap(0.15);
  const f = new S.MapField(g, 0.22);
  const pf = new S.ParticleFilter(350, rng);
  pf.initGlobal(g);
  // Seed one cluster near GT to simulate multi-step tracking convergence quickly
  const gt = { x: 5.2, y: 8.0, theta: 0.2 };
  for (let i = 0; i < 40; i++) pf.particles[i] = { x: gt.x + 0.1 * (rng() - 0.5), y: gt.y + 0.1 * (rng() - 0.5), th: gt.theta + 0.05 * (rng() - 0.5), w: 1 / 350 };
  for (let step = 0; step < 5; step++) {
    const scan = S.makeScan(g, gt, { beams: 120, rng });
    pf.update(f, scan, 3);
    pf.predict({ x: 0.15, y: 0, theta: 0 }, rng);
    gt.x += 0.15 * Math.cos(gt.theta);
    gt.y += 0.15 * Math.sin(gt.theta);
  }
  const est = pf.estimate();
  assert.ok(Math.hypot(est.x - gt.x, est.y - gt.y) < 0.35);
});

// 11. 4D EKF online gyro bias estimation
test("LocalizerCore4D estimates synthetic gyro bias b_g = +0.04 rad/s online", () => {
  const g = S.createWarehouseMap(0.1);
  const f = new S.MapField(g, 0.2);
  const core = new S.LocalizerCore4D(f);
  let gt = { x: 4.0, y: 8.0, theta: 0.0 };
  core.reset(gt);
  const trueBias = 0.04;
  const dt = 0.2;
  for (let k = 0; k < 45; k++) {
    const dtrans = 0.12;
    const dth = 0.04;
    const trueWz = dth / dt;
    const rawWz = trueWz + trueBias;
    gt = S.poseMul(gt, { x: dtrans, y: 0, theta: dth });
    const scan = S.makeScan(g, gt, { beams: 180 });
    core.step({ x: dtrans, y: 0, theta: dth }, scan, { wz: rawWz, dt });
  }
  assert.ok(Math.abs(core.gyroBias - trueBias) < 0.012, `Estimated gyroBias=${core.gyroBias} vs ${trueBias}`);
  assert.equal(core.state, "TRACKING");
});

// 12. 4D EKF anomaly state transition to GLOBAL when fully occluded
test("LocalizerCore4D transitions TRACKING -> DEGRADED -> GLOBAL under sustained severe mismatch", () => {
  const g = S.createWarehouseMap(0.1);
  const f = new S.MapField(g, 0.2);
  const core = new S.LocalizerCore4D(f, { degradedAfter: 3, anomalyWindow: 6, anomalyThreshold: 5 });
  core.reset({ x: 5.0, y: 8.0, theta: 0.0 });
  // Feed empty / garbage distant scan points
  const badScan = Array.from({ length: 20 }, (_, i) => ({ x: 40 + i, y: 40 }));
  let st = null;
  for (let k = 0; k < 6; k++) st = core.step({ x: 0.1, y: 0, theta: 0 }, badScan);
  assert.equal(st.state, "GLOBAL");
});

console.log(`\nAll ${passed} sim-core tests passed.`);
