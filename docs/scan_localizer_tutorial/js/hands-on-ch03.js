// Ch03 hands-on layer: reusable Correlative window scan, ICP basin comparison,
// real matchers.cpp source stepper and three checked exercises.
(function () {
  "use strict";

  const SOURCE_FILE = "src/matchers.cpp";
  const GT = { x: 5.5, y: 8.2, theta: 0.25 };
  const CORR_OPTIONS = {
    windowLin: 0.65,
    windowAng: 0.45,
    stepLin: 0.10,
    stepAng: 0.08,
  };
  const ICP_COMPARISON_STEPS = 80;

  // Contiguous excerpts copied from the current src/matchers.cpp. Line ranges
  // are kept with each stage so the source drawer opens exactly what is shown.
  const SOURCE_STAGES = [
    {
      title: "0 · Correlative：输入契约、有效点与粗搜缓冲",
      lines: "111-140",
      explain: "先拒绝无效地图、位姿和参数，再过滤非有限点。n_x、n_y、n_a 由窗口与步长决定；cells 在角度循环外预留容量，避免每个候选都重新分配。",
      code: `  if (!field.valid() || !pose_is_finite(prior) || scan.points.empty()) {
    return res;
  }
  if (!std::isfinite(opt.step_lin) || opt.step_lin <= 0.0 ||
    !std::isfinite(opt.step_ang) || opt.step_ang <= 0.0 ||
    !std::isfinite(opt.window_lin) || opt.window_lin < 0.0 ||
    !std::isfinite(opt.window_ang) || opt.window_ang < 0.0)
  {
    return res;
  }
  const auto * grid = &field.grid();

  std::vector<Vec2> valid_pts;
  valid_pts.reserve(scan.points.size());
  for (const auto & pt : scan.points) {
    if (pt_is_finite(pt)) {valid_pts.push_back(pt);}
  }
  const int n = static_cast<int>(valid_pts.size());
  if (n < 5) {return res;}

    // 粗搜 + 细搜两轮（细轮在粗最优 ± 一个粗步长内以 1/3 步长重扫）
  double best_s = -1.0;
  double best_x = prior.x, best_y = prior.y, best_th = wrap_angle(prior.theta);
  std::vector<std::array<double, 4>> coarse_poses;    // (s, x, y, θ)
  std::vector<Eigen::Vector2d> cells;
  cells.reserve(valid_pts.size());
  const double inv_2sig2 =
    0.5 / std::max(field.sigma_m_ * field.sigma_m_, 1e-6);
  const double max_gx = static_cast<double>(grid->width - 1);
  const double max_gy = static_cast<double>(grid->height - 1);`,
    },
    {
      title: "1 · 外层角度只旋转一次，内层平移直接加偏移",
      lines: "142-176",
      explain: "粗轮使用整数似然查表，细轮改用连续 SDF 重评分。关键是 cells 只在 ai 循环里计算一次；随后 nx×ny 个平移候选共享这份旋转点云。",
      code: `  for (int round = 0; round < 2; ++round) {
    if (round == 1) {
      best_s = -1.0;  // 细轮用连续 SDF 似然重评分，消除栅格量化平顶
    }
    const double lin_step =
      round == 0 ? opt.step_lin : opt.step_lin / 3.0;
    const double ang_step =
      round == 0 ? opt.step_ang : opt.step_ang / 3.0;
    const double wlin = round == 0 ? opt.window_lin : opt.step_lin;
    const double wang = round == 0 ? opt.window_ang : opt.step_ang;
    const int n_x = static_cast<int>(
      std::max(1.0, std::ceil(2.0 * wlin / lin_step)) + 1);
    const int n_y = n_x;
    const int n_a = static_cast<int>(
      std::max(1.0, std::ceil(2.0 * wang / ang_step)) + 1);
    const double cx = round == 0 ? prior.x : best_x;
    const double cy = round == 0 ? prior.y : best_y;
    const double cth = round == 0 ? wrap_angle(prior.theta) : best_th;
    if (round == 0) {
      coarse_poses.reserve(
        static_cast<std::size_t>(n_a) *
        static_cast<std::size_t>(n_x) *
        static_cast<std::size_t>(n_y));
    }

    for (int ai = 0; ai < n_a; ++ai) {
      const double th =
        wrap_angle(cth - 0.5 * (n_a - 1) * ang_step + ai * ang_step);
      const double c = std::cos(th), s = std::sin(th);
      cells.clear();
      for (const auto & p : valid_pts) {
        const double wx = prior.x + c * p.x() - s * p.y();
        const double wy = prior.y + s * p.x() + c * p.y();
        cells.push_back(grid->worldToGridF(wx, wy));
      }`,
    },
    {
      title: "2 · 滑窗评分：整数查表与连续 SDF 两条分支",
      lines: "177-214",
      explain: "offx/offy 是候选相对 prior 的偏移，不是相对粗最优的偏移。细轮围绕 best_x/best_y 枚举，但仍以 prior 为坐标基准，从根上避免双倍偏移。",
      code: `      for (int yi = 0; yi < n_y; ++yi) {
        const double local_dy = (yi - 0.5 * (n_y - 1)) * lin_step;
        const double cand_y = cy + local_dy;
        const double dy = cand_y - prior.y;
        const double offy = dy / grid->resolution;
        for (int xi = 0; xi < n_x; ++xi) {
          const double local_dx = (xi - 0.5 * (n_x - 1)) * lin_step;
          const double cand_x = cx + local_dx;
          const double dx = cand_x - prior.x;
          const double offx = dx / grid->resolution;
          double sc = 0.0;
          if (round == 0) {
            for (const auto & cel : cells) {
              sc += field.likelihood(
                static_cast<int>(std::lround(cel.x() + offx)),
                static_cast<int>(std::lround(cel.y() + offy)));
            }
          } else {
            for (const auto & cel : cells) {
              const double gx = cel.x() + offx;
              const double gy = cel.y() + offy;
              if (gx < 0.0 || gx > max_gx || gy < 0.0 || gy > max_gy) {
                continue;
              }
              const double r = field.sdf(gx, gy);
              sc += std::exp(-r * r * inv_2sig2);
            }
          }
          sc /= n;
          if (round == 0) {
            coarse_poses.push_back({sc, cand_x, cand_y, th});
          }
          if (sc > best_s) {
            best_s = sc;
            best_x = cand_x;
            best_y = cand_y;
            best_th = th;
          }`,
    },
    {
      title: "3 · 响应面 Softmax 二阶矩：峰值周围也是信息",
      lines: "220-249",
      explain: "协方差不是只看最高格，而是给所有粗候选乘 exp(β(s-smax)) 后求位移二阶矩。β 随点数在 80~250 之间夹紧；后续 L250-268 再做 SPD 与物理下界保护。",
      code: `  res.pose = {best_x, best_y, wrap_angle(best_th)};
    // 响应面二阶矩后验协方差（基于完整窗口均匀采样，softmax 权重 + SPD 下界保护）
  Cov3 cov = Cov3::Zero();
  bool cov_computed = false;
  if (!coarse_poses.empty()) {
    double smax_c = -1.0;
    for (const auto & q : coarse_poses) {
      smax_c = std::max(smax_c, q[0]);
    }
    const double beta = std::clamp(static_cast<double>(n), 80.0, 250.0);
    double wsum = 0.0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
    for (const auto & q : coarse_poses) {
      const double w = std::exp((q[0] - smax_c) * beta);
      if (w < 1e-9) {continue;}
      const Eigen::Vector3d v(
        q[1] - best_x, q[2] - best_y, wrap_angle(q[3] - best_th));
      wsum += w;
      mean += w * v;
      m2 += w * v * v.transpose();
    }
    if (wsum > 1e-12) {
      mean /= wsum;
      m2 /= wsum;
      cov = m2 - mean * mean.transpose();
      cov = 0.5 * (cov + cov.transpose()).eval();
      cov_computed = true;
    }
  }`,
    },
    {
      title: "4 · ICP-P2P：先找最近障碍格，再累积 H 与 b",
      lines: "417-441",
      explain: "每个扫描点沿 SDF 下降找到离散障碍格 q，残差是二维向量。旋转力臂使用世界坐标相对机器人位姿的差，J 的第三列不能误删。",
      code: `    for (const auto & pt : scan.points) {
      if (!pt_is_finite(pt)) {continue;}
      const double wx = p.x + c * pt.x() - s * pt.y();
      const double wy = p.y + s * pt.x() + c * pt.y();
      const auto g = grid->worldToGridF(wx, wy);
      const auto ob = field.descendToObstacle(
        static_cast<int>(std::lround(g.x())),
        static_cast<int>(std::lround(g.y())));
      const auto qw = grid->gridToWorld(ob.x(), ob.y());
      const double ex = wx - qw.x(), ey = wy - qw.y();
      const double e2 = ex * ex + ey * ey;
      if (e2 > opt.trim_dist * opt.trim_dist) {continue;}
      const double e = std::sqrt(e2);
      const double w = e <= opt.huber_delta ?
        1.0 :
        opt.huber_delta / std::max(e, 1e-9);
        // 世界系相对机器人位姿的旋转力臂
      Eigen::Matrix<double, 2, 3> J;
      J << 1.0, 0.0, -(wy - p.y), 0.0, 1.0, (wx - p.x);
      H += w * J.transpose() * J;
      b += w * J.transpose() * Eigen::Vector2d(ex, ey);
      rsum += w * e2;
      rw += w;
      ++used;
    }`,
    },
    {
      title: "5 · ICP-P2P：阻尼求解、限幅与实际步长收敛",
      lines: "447-463",
      explain: "Hd 只用于解步，不能直接当作几何协方差。收敛判断看 clip 后的真实位移；若原 dp 很大但被护栏截断，不能误报已收敛。",
      code: `    Eigen::Matrix3d Hd = H;
    Hd.diagonal().array() += 1e-6 * H.diagonal().array().max(1.0);
    const Eigen::Vector3d dp = Hd.ldlt().solve(-b);
    if (!dp.allFinite()) {break;}
    const double nl = dp.head<2>().norm();
    const double s_clip =
      std::min(1.0, std::min(0.2 / std::max(nl, 1e-9),
        0.1 / std::max(std::abs(dp(2)), 1e-9)));
    p.x += s_clip * dp(0);
    p.y += s_clip * dp(1);
    p.theta = wrap_angle(p.theta + s_clip * dp(2));
    if (nl * s_clip < opt.converg_t &&
      std::abs(dp(2)) * s_clip < opt.converg_r)
    {
      break;
    }
  }`,
    },
    {
      title: "6 · NDT：3×3 体素邻域里挑马氏距离最小的高斯",
      lines: "610-637",
      explain: "NDT 不是在整个地图找最近点，而是先按体素定位，再检查 3×3 邻域。best_m2 初始为 9，等价于 3σ 截断；距离门限 cell_r 进一步减少远场误匹配。",
      code: `        // 完整遍历 3x3 邻域，挑选马氏距离最小的高斯体素
      int best_g = -1;
      double best_m2 = 9.0;  // 3σ 截断
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = vx + dx, ny = vy + dy;
          if (nx < 0 || ny < 0 || nx >= voxel_w_ || ny >= voxel_h_) {
            continue;
          }
          const int gi = voxel_gauss_idx_[static_cast<std::size_t>(ny) *
              static_cast<std::size_t>(voxel_w_) +
              static_cast<std::size_t>(nx)];
          if (gi < 0) {continue;}
          const Gauss & cand = gauss_[static_cast<std::size_t>(gi)];
          const Eigen::Vector2d d(wx - cand.mu.x(), wy - cand.mu.y());
          if (d.squaredNorm() > cell_r * cell_r) {continue;}
          const double m2 = d.dot(cand.cov_inv * d);
          if (m2 < best_m2) {
            best_m2 = m2;
            best_g = gi;
          }
        }
      }
      if (best_g < 0) {continue;}
      const Gauss & G = gauss_[static_cast<std::size_t>(best_g)];
      const Eigen::Vector2d d(wx - G.mu.x(), wy - G.mu.y());
      const Eigen::Vector2d cd = G.cov_inv * d;
      const double sc = std::exp(-0.5 * best_m2);`,
    },
    {
      title: "7 · NDT：Magnusson 近似 Hessian 与最大化步",
      lines: "638-657",
      explain: "Hessian 丢弃二阶项后得到 PSD 近似。源码的 grad/g3 实际累加的是 -∇F（尽管上方注释写成了 ∂s/∂p 的负号形式）；因此最大化 Σs 时用 dp=-Hs.ldlt().solve(g3)，等价于沿 +∇F 上升，与最小化残差的 GN 符号相反。",
      code: `        // ∂s/∂p = -s · Jᵀ C⁻¹ d （J = ∂d/∂p, 2x3）
      Eigen::Matrix<double, 2, 3> J;
      J << 1.0, 0.0, -(wy - p.y), 0.0, 1.0, (wx - p.x);
      const Eigen::Vector3d grad = sc * J.transpose() * cd;
        // GN 近似 Hessian（Magnusson：丢弃二阶项）
      H += sc * J.transpose() * G.cov_inv * J;
      g3 += grad;
      ++used;
    }
    if (used < 5) {break;}
    has_valid_iter = true;
    last_H = H;
    last_used = used;

      // 最大化 F=Σs 的近似牛顿步：∇F = -Σ s JᵀC⁻¹d，
      // -∇²F ≈ Σ s JᵀC⁻¹J = H（PSD，丢弃二阶项，Magnusson eq.17-19）
    Eigen::Matrix3d Hs = H;
    Hs.diagonal().array() += 1e-3 * H.diagonal().maxCoeff() + 1e-6;
    const Eigen::Vector3d dp = -Hs.ldlt().solve(g3);      // dp = H⁻¹∇F = -H⁻¹g3
    if (!dp.allFinite()) {break;}`,
    },
    {
      title: "8 · NDT：按有效点数归一化，避免 N 倍过自信",
      lines: "678-684",
      explain: "H 已含逆协方差，单位是 m⁻²。直接对 N 个点求和再求逆会让方差随点数错误缩小 N 倍；last_used 归一化保留各方向信息比。",
      code: `  res.pose = p;
    // NDT 的 H 已含 C⁻¹（单位 m⁻²）；按有效点数归一化后求逆，防止 N 倍过自信并保留退化比
  const Eigen::Matrix3d H_norm = last_H / static_cast<double>(std::max(1, last_used));
  res.covariance = cov_from_hessian(H_norm, 1.0);
  eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
    res.inlier_ratio);
  res.valid = true;`,
    },
    {
      title: "9 · 组合匹配：粗捕获后再精修，并允许退回粗解",
      lines: "691-705",
      explain: "在窗口覆盖真位姿时，Correlative 负责离开局部盆地，Field-GN 负责亚胞元精修。精修无效或得分比粗解低超过 0.05 时返回粗解，避免局部优化把好解修坏。",
      code: `MatchResult ComposedMatcher::match(
  const MapField & field, const Pose2 & prior,
  const Scan & scan,
  const MatchOptions & opt) const
{
  MatchResult res;
  if (!coarse_ && !fine_) {return res;}
  if (!coarse_) {return fine_->match(field, prior, scan, opt);}
  MatchResult coarse = coarse_->match(field, prior, scan, opt);
  if (!coarse.valid || !fine_) {return coarse;}
  MatchResult fine = fine_->match(field, coarse.pose, scan, opt);
    // 门限：精修需有效且似然不应显著低于粗匹配（防精修跑飞）
  if (!fine.valid || fine.score + 0.05 < coarse.score) {return coarse;}
  return fine;
}`,
    },
  ];

  function escapeHtml(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#39;");
  }

  function fmt(value, digits = 4) {
    if (value === null || value === undefined) return "—";
    if (typeof value === "boolean") return value ? "true" : "false";
    if (typeof value === "number") {
      if (!Number.isFinite(value)) return String(value);
      const av = Math.abs(value);
      if (av !== 0 && (av < 1e-3 || av >= 1e4)) return value.toExponential(2);
      const text = value.toFixed(digits);
      return digits > 0 ? text.replace(/\.?0+$/, "") : text;
    }
    return String(value);
  }

  function vec(values) {
    return Array.isArray(values) ? `[${values.map((x) => fmt(x)).join(", ")}]` : "—";
  }

  function pose(value) {
    if (!value) return "—";
    return `(${fmt(value.x)}, ${fmt(value.y)}, ${fmt(value.theta)})`;
  }

  function fillVars(host, rows) {
    if (!host) return;
    host.innerHTML = rows.map(([key, value]) =>
      `<div class="hands-on-var"><dt>${escapeHtml(key)}</dt><dd>${escapeHtml(value)}</dd></div>`).join("");
  }

  function highlight(code) {
    if (!code) return;
    code.className = "language-cpp";
    delete code.dataset.highlighted;
    if (window.hljs) {
      try { window.hljs.highlightElement(code); } catch (_) {}
    }
  }

  function posErrorCm(value, gt) {
    return Math.hypot(value.x - gt.x, value.y - gt.y) * 100;
  }

  function yawErrorDeg(value, gt, S) {
    return Math.abs(S.wrapAngle(value.theta - gt.theta)) * 180 / Math.PI;
  }

  function errorText(value, gt, S) {
    return `${fmt(posErrorCm(value, gt), 1)} cm / ${fmt(yawErrorDeg(value, gt, S), 1)}°`;
  }

  // Minimal single coarse-window implementation. It deliberately mirrors the
  // browser core's grid contract: rotate+worldToGrid once per angle, then add
  // integer translation offsets. Production's fine SDF round and covariance
  // are omitted here; every displayed number is cross-checked against S.*.
  function minimalCorrelativeWindow(S, field, prior, pts, opt) {
    const nx = Math.max(1, Math.ceil((2 * opt.windowLin) / opt.stepLin)) + 1;
    const ny = nx;
    const na = Math.max(1, Math.ceil((2 * opt.windowAng) / opt.stepAng)) + 1;
    let bestScore = -1;
    let bestPose = { ...prior };
    const heatmap = [];
    let pointTransforms = 0;
    let lookups = 0;

    for (let ai = 0; ai < na; ai++) {
      const theta = S.wrapAngle(prior.theta + (ai - 0.5 * (na - 1)) * opt.stepAng);
      const c = Math.cos(theta), s = Math.sin(theta);
      const cells = pts.map((p) => {
        pointTransforms++;
        return field.grid.worldToGridF(
          prior.x + c * p.x - s * p.y,
          prior.y + s * p.x + c * p.y);
      });
      for (let yi = 0; yi < ny; yi++) {
        const dy = (yi - 0.5 * (ny - 1)) * opt.stepLin;
        const offY = dy / field.grid.resolution;
        for (let xi = 0; xi < nx; xi++) {
          const dx = (xi - 0.5 * (nx - 1)) * opt.stepLin;
          const offX = dx / field.grid.resolution;
          let score = 0;
          for (const cell of cells) {
            score += field.likelihood(
              Math.round(cell.gx + offX),
              Math.round(cell.gy + offY));
            lookups++;
          }
          score /= pts.length;
          if (ai === Math.floor(na / 2)) heatmap.push({ xi, yi, dx, dy, score });
          if (score > bestScore) {
            bestScore = score;
            bestPose = { x: prior.x + dx, y: prior.y + dy, theta };
          }
        }
      }
    }

    const stats = S.evalStats(field, bestPose, pts, 0.12);
    const naivePointTransforms = pts.length * na * nx * ny;
    return {
      nx,
      ny,
      na,
      pose: bestPose,
      maxGridScore: bestScore,
      score: stats.score,
      heatmap,
      pointTransforms,
      naivePointTransforms,
      lookups,
      reductionFactor: naivePointTransforms / pointTransforms,
    };
  }

  function samePose(a, b, S) {
    return Math.abs(a.x - b.x) < 1e-9 &&
      Math.abs(a.y - b.y) < 1e-9 &&
      Math.abs(S.wrapAngle(a.theta - b.theta)) < 1e-9;
  }

  function buildScenario(S, field, scan, ndt, errXY, errDeg) {
    const prior = {
      x: GT.x + errXY,
      y: GT.y - errXY * 0.5,
      theta: GT.theta + errDeg * Math.PI / 180,
    };
    const corr = S.correlativeMatch(field, prior, scan, CORR_OPTIONS);
    const minimal = minimalCorrelativeWindow(S, field, prior, scan, CORR_OPTIONS);
    const coreHeatmap = new Map(corr.heatmap.map((cell) => [`${cell.xi},${cell.yi}`, cell]));
    const heatmapParity = minimal.heatmap.length === corr.heatmap.length &&
      minimal.heatmap.every((cell) => {
        const other = coreHeatmap.get(`${cell.xi},${cell.yi}`);
        return other && Math.abs(cell.score - other.score) < 1e-12 &&
          Math.abs(cell.dx - other.dx) < 1e-12 &&
          Math.abs(cell.dy - other.dy) < 1e-12;
      });
    const parity = samePose(minimal.pose, corr.pose, S) && heatmapParity &&
      Math.abs(minimal.score - corr.score) < 1e-12;

    const localGn = S.fieldGnMatch(field, prior, scan);
    const composedGn = S.fieldGnMatch(field, corr.pose, scan);
    const icpTrace = [{ iteration: 0, pose: { ...prior }, step: null }];
    let icpPose = { ...prior };
    for (let iteration = 1; iteration <= ICP_COMPARISON_STEPS; iteration++) {
      const step = S.icpP2pStep(field, icpPose, scan);
      if (!step.ok) {
        icpTrace.push({ iteration, pose: { ...icpPose }, step, failed: true });
        break;
      }
      icpPose = step.nextPose;
      icpTrace.push({ iteration, pose: { ...icpPose }, step });
    }
    const traceAt = (iteration) => icpTrace[Math.min(iteration, icpTrace.length - 1)];
    const initialStats = S.evalStats(field, prior, scan, 0.12);
    const centerAi = Math.floor(minimal.na / 2);
    const centerTheta = S.wrapAngle(
      prior.theta + (centerAi - 0.5 * (minimal.na - 1)) * CORR_OPTIONS.stepAng);
    const centerBest = corr.heatmap.reduce((best, cell) =>
      !best || cell.score > best.score ? cell : best, null);
    const centerPose = centerBest ? {
      x: prior.x + centerBest.dx,
      y: prior.y + centerBest.dy,
      theta: centerTheta,
    } : null;

    return {
      prior,
      corr,
      minimal,
      parity,
      localGn,
      composedGn,
      icpTrace,
      icp12: traceAt(12),
      icp80: traceAt(ICP_COMPARISON_STEPS),
      initialStats,
      centerBest,
      centerPose,
      ndt,
      errXY,
      errDeg,
    };
  }

  function heatmapPlot(W, H) {
    return { x: 54, y: 44, w: W - 78, h: H - 88 };
  }

  function drawHeatmap(canvas, scenario, hover, S) {
    if (!canvas || !scenario || !window.SlocApp) return;
    const W = 740, H = 320;
    const ctx = window.SlocApp.setupCanvas(canvas, W, H);
    const pal = window.SlocApp.palette();
    const plot = heatmapPlot(W, H);
    const { corr, minimal, prior } = scenario;
    const cellW = plot.w / minimal.nx;
    const cellH = plot.h / minimal.ny;
    const maxScore = Math.max(0.01, ...corr.heatmap.map((cell) => cell.score));
    const dark = window.SlocApp.isDark();

    ctx.fillStyle = pal.bg;
    ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = pal.text;
    ctx.font = '600 14px -apple-system, sans-serif';
    ctx.fillText("Correlative 中心角切片：实时窗口似然", 16, 24);
    ctx.fillStyle = pal.muted;
    ctx.font = '11px monospace';
    ctx.fillText(`dx → · dy ↑ · ${minimal.nx}×${minimal.ny} cells · θ=${fmt(centerAngle(scenario, S), 3)} rad`, 16, 40);

    for (const cell of corr.heatmap) {
      const t = Math.max(0, Math.min(1, cell.score / maxScore));
      const light = dark ? 16 + t * 52 : 95 - t * 43;
      const hue = 222 - t * 152;
      ctx.fillStyle = `hsl(${hue} 82% ${light}%)`;
      ctx.fillRect(plot.x + cell.xi * cellW, plot.y + plot.h - (cell.yi + 1) * cellH,
        cellW + 0.5, cellH + 0.5);
    }

    ctx.strokeStyle = pal.axis;
    ctx.lineWidth = 1;
    ctx.strokeRect(plot.x, plot.y, plot.w, plot.h);
    ctx.beginPath();
    ctx.moveTo(plot.x + plot.w / 2, plot.y);
    ctx.lineTo(plot.x + plot.w / 2, plot.y + plot.h);
    ctx.moveTo(plot.x, plot.y + plot.h / 2);
    ctx.lineTo(plot.x + plot.w, plot.y + plot.h / 2);
    ctx.stroke();

    const toX = (dx) => plot.x + ((dx + CORR_OPTIONS.windowLin) / (2 * CORR_OPTIONS.windowLin)) * plot.w;
    const toY = (dy) => plot.y + plot.h - ((dy + CORR_OPTIONS.windowLin) / (2 * CORR_OPTIONS.windowLin)) * plot.h;
    const marker = (dx, dy, color, label, dashed) => {
      const x = Math.max(plot.x + 5, Math.min(plot.x + plot.w - 5, toX(dx)));
      const y = Math.max(plot.y + 5, Math.min(plot.y + plot.h - 5, toY(dy)));
      ctx.save();
      ctx.strokeStyle = color;
      ctx.fillStyle = pal.bg;
      ctx.lineWidth = 2;
      ctx.setLineDash(dashed ? [4, 3] : []);
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = color;
      ctx.font = '600 11px -apple-system, sans-serif';
      ctx.fillText(label, Math.min(x + 8, plot.x + plot.w - 118), Math.max(y - 8, plot.y + 12));
      ctx.restore();
    };

    marker(0, 0, pal.text, "prior");
    if (scenario.centerBest) {
      marker(scenario.centerBest.dx, scenario.centerBest.dy, pal.orange, "θ0 slice best");
    }
    marker(corr.pose.x - prior.x, corr.pose.y - prior.y, pal.red, "window best", true);
    if (hover) marker(hover.dx, hover.dy, pal.teal, "hover", false);

    ctx.fillStyle = pal.muted;
    ctx.font = '11px monospace';
    ctx.textAlign = "center";
    [-0.6, 0, 0.6].forEach((value) => ctx.fillText(fmt(value, 1), toX(value), plot.y + plot.h + 18));
    ctx.textAlign = "right";
    [-0.6, 0, 0.6].forEach((value) => ctx.fillText(fmt(value, 1), plot.x - 8, toY(value) + 4));
    ctx.textAlign = "left";
    ctx.fillText("dx / m", plot.x + plot.w - 42, plot.y + plot.h + 34);
    ctx.fillText("dy / m", 8, plot.y - 8);
  }

  function centerAngle(scenario, S) {
    const na = scenario.minimal.na;
    return S.wrapAngle(scenario.prior.theta +
      (Math.floor(na / 2) - 0.5 * (na - 1)) * CORR_OPTIONS.stepAng);
  }

  function drawComparison(canvas, scenario, S) {
    if (!canvas || !scenario || !window.SlocApp) return;
    const W = 740, H = 310;
    const ctx = window.SlocApp.setupCanvas(canvas, W, H);
    const pal = window.SlocApp.palette();
    const plot = { x: 58, y: 52, w: W - 82, h: H - 96 };
    const icpPoints = scenario.icpTrace.map((entry) => ({
      x: entry.iteration,
      y: posErrorCm(entry.pose, GT),
    }));
    const gnPoints = (scenario.localGn.history || []).map((entry, index) => ({
      x: index,
      y: posErrorCm(entry.pose, GT),
    }));
    const globalPoints = [
      { x: 0, y: posErrorCm(scenario.corr.pose, GT) },
      { x: 0, y: posErrorCm(scenario.composedGn.pose, GT) },
    ];
    const yMax = Math.max(10, ...icpPoints.map((p) => p.y), ...gnPoints.map((p) => p.y),
      ...globalPoints.map((p) => p.y)) * 1.12;
    const xMax = Math.max(ICP_COMPARISON_STEPS, gnPoints.length - 1);
    const toX = (value) => plot.x + value / xMax * plot.w;
    const toY = (value) => plot.y + plot.h - value / yMax * plot.h;

    ctx.fillStyle = pal.bg;
    ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = pal.text;
    ctx.font = '600 14px -apple-system, sans-serif';
    ctx.fillText("同一初值的位置误差：局部迭代 vs 窗口捕获", 16, 24);

    const legend = [
      [pal.orange, "ICP-P2P"],
      [pal.blue, "Field-GN 局部"],
      [pal.red, "Correlative 窗口"],
      [pal.green, "Corr + Field-GN"],
    ];
    let legendX = 16;
    ctx.font = '11px -apple-system, sans-serif';
    for (const [color, label] of legend) {
      ctx.fillStyle = color;
      ctx.fillRect(legendX, 34, 16, 3);
      ctx.fillStyle = pal.muted;
      ctx.fillText(label, legendX + 21, 39);
      legendX += ctx.measureText(label).width + 48;
    }

    ctx.strokeStyle = pal.grid;
    ctx.fillStyle = pal.muted;
    ctx.font = '11px monospace';
    ctx.textAlign = "right";
    for (let i = 0; i <= 4; i++) {
      const value = yMax * i / 4;
      const y = toY(value);
      ctx.beginPath();
      ctx.moveTo(plot.x, y);
      ctx.lineTo(plot.x + plot.w, y);
      ctx.stroke();
      ctx.fillText(fmt(value, 0), plot.x - 8, y + 4);
    }
    ctx.textAlign = "center";
    [0, 12, 40, xMax].forEach((value) => ctx.fillText(String(value), toX(value), plot.y + plot.h + 18));
    ctx.textAlign = "left";
    ctx.fillText("iteration", plot.x + plot.w - 54, plot.y + plot.h + 34);
    ctx.save();
    ctx.translate(12, plot.y + plot.h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("position error / cm", 0, 0);
    ctx.restore();

    const line = (points, color, width) => {
      if (!points.length) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      points.forEach((point, index) => {
        if (index === 0) ctx.moveTo(toX(point.x), toY(point.y));
        else ctx.lineTo(toX(point.x), toY(point.y));
      });
      ctx.stroke();
    };
    line(icpPoints, pal.orange, 2);
    line(gnPoints, pal.blue, 2);

    ctx.strokeStyle = pal.axis;
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(toX(12), plot.y);
    ctx.lineTo(toX(12), plot.y + plot.h);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = pal.muted;
    ctx.fillText("#12", toX(12) + 5, plot.y + 12);

    const dot = (x, y, color, radius) => {
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(toX(x), toY(y), radius, 0, Math.PI * 2);
      ctx.fill();
    };
    dot(0, posErrorCm(scenario.corr.pose, GT), pal.red, 4);
    dot(0, posErrorCm(scenario.composedGn.pose, GT), pal.green, 5);
    dot(scenario.icp12.iteration, posErrorCm(scenario.icp12.pose, GT), pal.orange, 4);
    dot(scenario.icp80.iteration, posErrorCm(scenario.icp80.pose, GT), pal.orange, 4);
  }

  function init() {
    if (!window.SlocSim || window.PAGE_ID !== "ch03") return;
    const S = window.SlocSim;
    const grid = S.createWarehouseMap(0.15);
    const field = new S.MapField(grid, 0.25);
    const ndt = S.ndtBuild(grid, 0.9);
    const scan = S.makeScan(grid, GT, { beams: 140, maxRange: 10 });
    const xyInput = document.getElementById("ch03-err-xy");
    const thInput = document.getElementById("ch03-err-th");
    const heatCanvas = document.getElementById("ch03-live-heatmap");
    const compareCanvas = document.getElementById("ch03-compare-canvas");
    const compareBody = document.getElementById("ch03-compare-body");
    const stageTitle = document.getElementById("ch03-stage-title");
    const stageMeta = document.getElementById("ch03-stage-meta");
    const stageExplain = document.getElementById("ch03-stage-explain");
    const stageCode = document.getElementById("ch03-stage-code");
    const stageIndex = document.getElementById("ch03-stage-index");
    const stagePrev = document.getElementById("ch03-stage-prev");
    const stageNext = document.getElementById("ch03-stage-next");
    const sourceBtn = document.getElementById("ch03-stage-source");
    const copyBtn = document.getElementById("ch03-stage-copy");
    const exerciseProgress = document.getElementById("ch03-exercise-progress");

    let scenario = null;
    let sourceStage = 0;
    let hover = null;
    let showNdt = false;
    let frame = 0;
    let shadowIcp = {
      pose: null,
      iteration: 0,
      before: null,
      lastStep: null,
    };

    function readPrior() {
      const errXY = xyInput ? parseFloat(xyInput.value) : 0.35;
      const errDeg = thInput ? parseFloat(thInput.value) : 22;
      return {
        errXY,
        errDeg,
        pose: {
          x: GT.x + errXY,
          y: GT.y - errXY * 0.5,
          theta: GT.theta + errDeg * Math.PI / 180,
        },
      };
    }

    function resetShadowIcp() {
      const prior = readPrior().pose;
      shadowIcp = { pose: { ...prior }, iteration: 0, before: null, lastStep: null };
      renderAnimationVars();
      renderSource();
    }

    function renderAnimationVars() {
      const step = shadowIcp.lastStep;
      const pair = step && step.pairs && step.pairs[0];
      fillVars(document.getElementById("ch03-animation-vars"), [
        ["同步迭代", `#${shadowIcp.iteration}`],
        ["当前 pose", pose(shadowIcp.pose)],
        ["最近一步 before", pose(shadowIcp.before)],
        ["最近一步 RMSE", step ? `${fmt(step.rmse)} m` : "—"],
        ["pairs[0] 抽样", pair ? `w=(${fmt(pair.wx)}, ${fmt(pair.wy)}) → q=(${fmt(pair.qx)}, ${fmt(pair.qy)})` : "—"],
        ["同步说明", "与上方原 ICP 动画使用同一初值和 S.icpP2pStep；pairs 仅为教学核抽样。"],
      ]);
    }

    function renderBestVars() {
      if (!scenario) return;
      const { prior, corr, minimal, centerBest, centerPose } = scenario;
      const delta = [corr.pose.x - prior.x, corr.pose.y - prior.y, S.wrapAngle(corr.pose.theta - prior.theta)];
      fillVars(document.getElementById("ch03-best-vars"), [
        ["prior (x, y, θ)", pose(prior)],
        ["窗口 nx × ny × nθ", `${minimal.nx} × ${minimal.ny} × ${minimal.na}`],
        ["max_grid_score", fmt(minimal.maxGridScore, 6)],
        ["S.score（最佳位姿重评分）", fmt(corr.score, 6)],
        ["best_pose", pose(corr.pose)],
        ["best_delta [dx, dy, dθ]", vec(delta)],
        ["best_error", errorText(corr.pose, GT, S)],
        ["θ0 切片最佳", centerPose ? `${pose(centerPose)} · score=${fmt(centerBest.score, 6)}` : "—"],
        ["旋转点变换：复用 / 朴素", `${minimal.pointTransforms} / ${minimal.naivePointTransforms}`],
        ["似然查表次数", minimal.lookups],
        ["最小核 vs S.correlativeMatch", scenario.parity ? "PASS：位姿、中心热力图、score 一致" : "FAIL：请检查窗口契约"],
      ]);
      const parity = document.getElementById("ch03-parity");
      if (parity) {
        parity.textContent = scenario.parity
          ? `一致性 PASS：${minimal.pointTransforms} 次点变换替代 ${minimal.naivePointTransforms} 次，降幅 ${fmt(minimal.reductionFactor, 0)}×；查表仍为 ${minimal.lookups} 次。`
          : "一致性 FAIL：最小窗口核与共享 S.correlativeMatch 不一致。";
      }
    }

    function renderHoverVars() {
      if (!scenario || !hover) {
        fillVars(document.getElementById("ch03-hover-vars"), [["候选检查器", "在热力图上移动指针，查看任意网格候选。"]]);
        return;
      }
      const candidate = {
        x: scenario.prior.x + hover.dx,
        y: scenario.prior.y + hover.dy,
        theta: centerAngle(scenario, S),
      };
      fillVars(document.getElementById("ch03-hover-vars"), [
        ["candidate (x, y, θ0)", pose(candidate)],
        ["candidate_delta [dx, dy]", vec([hover.dx, hover.dy])],
        ["grid_score", fmt(hover.score, 6)],
        ["离窗口 best 的位置差", `${fmt(posErrorCm(candidate, scenario.corr.pose), 2)} cm`],
      ]);
    }

    function renderComparison() {
      if (!scenario) return;
      const localGnImproved = posErrorCm(scenario.localGn.pose, GT) < posErrorCm(scenario.prior, GT);
      const truthDelta = [
        GT.x - scenario.prior.x,
        GT.y - scenario.prior.y,
        S.wrapAngle(GT.theta - scenario.prior.theta),
      ];
      const truthInsideWindow = Math.abs(truthDelta[0]) <= CORR_OPTIONS.windowLin + 1e-9 &&
        Math.abs(truthDelta[1]) <= CORR_OPTIONS.windowLin + 1e-9 &&
        Math.abs(truthDelta[2]) <= CORR_OPTIONS.windowAng + 1e-9;
      const composedImproved = posErrorCm(scenario.composedGn.pose, GT) < posErrorCm(scenario.corr.pose, GT);
      const rows = [
        ["初始 prior", "搜索起点", errorText(scenario.prior, GT, S), `S.score=${fmt(scenario.initialStats.score, 4)}`, "局部优化共同初值"],
        ["ICP-P2P", "12 步（原动画）", errorText(scenario.icp12.pose, GT, S), `RMSE=${fmt(scenario.icp12.step && scenario.icp12.step.rmse)} m`, "离散对应关系可能换墙"],
        ["ICP-P2P", "80 步对照", errorText(scenario.icp80.pose, GT, S), `RMSE=${fmt(scenario.icp80.step && scenario.icp80.step.rmse)} m`, "不保证离开错误盆地"],
        ["Field-GN（局部）", "同一 prior", errorText(scenario.localGn.pose, GT, S), `RMSE=${fmt(scenario.localGn.rmse)} m`, localGnImproved ? "连续场降低真值误差，但无全局保证" : "连续场未降低真值误差"],
        ["Correlative（窗口）", `${scenario.minimal.na} 个角度`, errorText(scenario.corr.pose, GT, S), `S.score=${fmt(scenario.corr.score, 4)}`, truthInsideWindow ? "真位姿在窗口内，枚举候选捕获" : "真位姿超出窗口，不能承诺全局捕获"],
        ["Correlative + Field-GN", "捕获后精修", errorText(scenario.composedGn.pose, GT, S), `RMSE=${fmt(scenario.composedGn.rmse)} m`, truthInsideWindow ? (composedImproved ? "窗口捕获后精修降低误差" : "窗口捕获后精修未改善") : "窗口未覆盖真位姿，精修无法补回全局性"],
      ];
      if (compareBody) {
        compareBody.innerHTML = rows.map((row) =>
          `<tr>${row.map((cell) => `<td>${escapeHtml(cell)}</td>`).join("")}</tr>`).join("");
      }
      fillVars(document.getElementById("ch03-compare-vars"), [
        ["初始误差", errorText(scenario.prior, GT, S)],
        ["ICP #12", errorText(scenario.icp12.pose, GT, S)],
        ["ICP #80", errorText(scenario.icp80.pose, GT, S)],
        ["Field-GN 局部", errorText(scenario.localGn.pose, GT, S)],
        ["Correlative 捕获", errorText(scenario.corr.pose, GT, S)],
        ["组合精修", errorText(scenario.composedGn.pose, GT, S)],
      ]);
    }

    function sourceRows() {
      if (!scenario) return [["状态", "等待仿真核初始化"]];
      if (sourceStage <= 3) {
        const delta = [
          scenario.corr.pose.x - scenario.prior.x,
          scenario.corr.pose.y - scenario.prior.y,
          S.wrapAngle(scenario.corr.pose.theta - scenario.prior.theta),
        ];
        return [
          ["有效激光点 N", scan.length],
          ["nx × ny × nθ", `${scenario.minimal.nx} × ${scenario.minimal.ny} × ${scenario.minimal.na}`],
          ["点变换：复用 / 朴素", `${scenario.minimal.pointTransforms} / ${scenario.minimal.naivePointTransforms}`],
          ["max_grid_score", fmt(scenario.minimal.maxGridScore, 6)],
          ["best_delta", vec(delta)],
          ["最小核一致性", scenario.parity ? "PASS" : "FAIL"],
        ];
      }
      if (sourceStage <= 5) {
        const step = shadowIcp.lastStep;
        const pair = step && step.pairs && step.pairs[0];
        return [
          ["原动画同步迭代", `#${shadowIcp.iteration}`],
          ["p_before", pose(shadowIcp.before)],
          ["p_after", pose(shadowIcp.pose)],
          ["本步 RMSE", step ? `${fmt(step.rmse)} m` : "—"],
          ["对应点抽样", pair ? `w=(${fmt(pair.wx)}, ${fmt(pair.wy)}) → q=(${fmt(pair.qx)}, ${fmt(pair.qy)})` : "—"],
          ["变量边界", "S.icpP2pStep 暴露 nextPose/RMSE/pairs；H、b、dp 以左侧真实源码为准。"],
        ];
      }
      if (sourceStage <= 8) {
        return [
          ["NDT 体素尺寸", `${fmt(scenario.ndt.voxelM)} m`],
          ["有效高斯数", scenario.ndt.gauss.length],
          ["原动画椭圆叠加", showNdt ? "开启" : "关闭"],
          ["实现边界", "共享核提供 ndtBuild 椭圆；不在本页另造 NDT 牛顿步数值实现。"],
        ];
      }
      return [
        ["coarse_pose", pose(scenario.corr.pose)],
        ["coarse_score", fmt(scenario.corr.score, 6)],
        ["fine_pose", pose(scenario.composedGn.pose)],
        ["fine_rmse", `${fmt(scenario.composedGn.rmse)} m`],
        ["精修后误差", errorText(scenario.composedGn.pose, GT, S)],
      ];
    }

    function renderSource() {
      const stage = SOURCE_STAGES[sourceStage];
      if (stageTitle) stageTitle.textContent = stage.title;
      if (stageMeta) stageMeta.textContent = `${SOURCE_FILE}:L${stage.lines} · 真实仓库源码`;
      if (stageExplain) stageExplain.textContent = stage.explain;
      if (stageIndex) stageIndex.textContent = `${sourceStage + 1} / ${SOURCE_STAGES.length}`;
      if (stageCode) { stageCode.textContent = stage.code; highlight(stageCode); }
      if (stagePrev) stagePrev.disabled = sourceStage === 0;
      if (stageNext) stageNext.disabled = sourceStage === SOURCE_STAGES.length - 1;
      if (sourceBtn) {
        sourceBtn.setAttribute("data-file", SOURCE_FILE);
        sourceBtn.setAttribute("data-lines", stage.lines);
        sourceBtn.setAttribute("data-label", `${SOURCE_FILE} L${stage.lines}`);
      }
      fillVars(document.getElementById("ch03-source-vars"), sourceRows());
    }

    function resetExercise(form) {
      form.dataset.passed = "false";
      const input = form.querySelector(".exercise-input");
      const feedback = form.querySelector(".exercise-feedback");
      if (input) input.value = "";
      if (feedback) {
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
      }
    }

    function updateProgress() {
      const forms = Array.from(document.querySelectorAll(".hands-on-exercise"));
      const passed = forms.filter((form) => form.dataset.passed === "true").length;
      if (exerciseProgress) {
        exerciseProgress.textContent = passed === forms.length && forms.length > 0
          ? `验收完成：${passed} / ${forms.length}`
          : `验收进度：${passed} / ${forms.length}`;
      }
    }

    function updateExercises() {
      if (!scenario) return;
      const forms = {
        reuse: document.getElementById("ch03-ex1-form"),
        delta: document.getElementById("ch03-ex2-form"),
      };
      const answers = {
        reuse: String(scenario.minimal.reductionFactor),
        delta: (scenario.corr.pose.x - scenario.prior.x).toFixed(2),
      };
      Object.keys(forms).forEach((key) => {
        const form = forms[key];
        if (!form) return;
        if (form.dataset.answer && form.dataset.answer !== answers[key]) resetExercise(form);
        form.dataset.answer = answers[key];
      });
      updateProgress();
    }

    function renderAll() {
      renderBestVars();
      renderHoverVars();
      renderComparison();
      renderAnimationVars();
      renderSource();
      updateExercises();
      drawHeatmap(heatCanvas, scenario, hover, S);
      drawComparison(compareCanvas, scenario, S);
    }

    function scheduleUpdate() {
      if (frame) window.cancelAnimationFrame(frame);
      frame = window.requestAnimationFrame(() => {
        frame = 0;
        const { errXY, errDeg } = readPrior();
        scenario = buildScenario(S, field, scan, ndt, errXY, errDeg);
        hover = null;
        renderAll();
      });
    }

    if (xyInput) xyInput.addEventListener("input", () => { resetShadowIcp(); scheduleUpdate(); });
    if (thInput) thInput.addEventListener("input", () => { resetShadowIcp(); scheduleUpdate(); });

    const stepBtn = document.getElementById("ch03-icp-step");
    if (stepBtn) stepBtn.addEventListener("click", () => {
      if (!scenario) return;
      shadowIcp.before = { ...shadowIcp.pose };
      const step = S.icpP2pStep(field, shadowIcp.pose, scan);
      shadowIcp.lastStep = step;
      if (step.ok) {
        shadowIcp.pose = step.nextPose;
        shadowIcp.iteration++;
      }
      renderAnimationVars();
      renderSource();
    });

    const runBtn = document.getElementById("ch03-icp-run");
    if (runBtn) runBtn.addEventListener("click", () => {
      if (!scenario) return;
      for (let i = 0; i < 12; i++) {
        shadowIcp.before = { ...shadowIcp.pose };
        const step = S.icpP2pStep(field, shadowIcp.pose, scan);
        shadowIcp.lastStep = step;
        if (!step.ok) break;
        shadowIcp.pose = step.nextPose;
        shadowIcp.iteration++;
      }
      renderAnimationVars();
      renderSource();
    });

    const ndtBtn = document.getElementById("ch03-ndt-toggle");
    if (ndtBtn) ndtBtn.addEventListener("click", () => {
      showNdt = !showNdt;
      renderSource();
    });

    const resetBtn = document.getElementById("ch03-compare-reset");
    if (resetBtn && xyInput && thInput) resetBtn.addEventListener("click", () => {
      xyInput.value = "0.35";
      thInput.value = "22";
      xyInput.dispatchEvent(new Event("input", { bubbles: true }));
      thInput.dispatchEvent(new Event("input", { bubbles: true }));
    });

    if (stagePrev) stagePrev.addEventListener("click", () => {
      if (sourceStage > 0) { sourceStage--; renderSource(); }
    });
    if (stageNext) stageNext.addEventListener("click", () => {
      if (sourceStage < SOURCE_STAGES.length - 1) { sourceStage++; renderSource(); }
    });
    if (copyBtn) copyBtn.addEventListener("click", () => {
      if (navigator.clipboard && stageCode) {
        navigator.clipboard.writeText(stageCode.textContent).catch(() => {});
      }
      const old = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = old; }, 1200);
    });

    if (heatCanvas) {
      heatCanvas.addEventListener("pointermove", (event) => {
        if (!scenario) return;
        const rect = heatCanvas.getBoundingClientRect();
        const x = (event.clientX - rect.left) * 740 / rect.width;
        const y = (event.clientY - rect.top) * 320 / rect.height;
        const plot = heatmapPlot(740, 320);
        const xi = Math.floor((x - plot.x) / (plot.w / scenario.minimal.nx));
        const yi = Math.floor((plot.y + plot.h - y) / (plot.h / scenario.minimal.ny));
        hover = scenario.corr.heatmap.find((cell) => cell.xi === xi && cell.yi === yi) || null;
        renderHoverVars();
        drawHeatmap(heatCanvas, scenario, hover, S);
      });
      heatCanvas.addEventListener("pointerleave", () => {
        hover = null;
        renderHoverVars();
        drawHeatmap(heatCanvas, scenario, hover, S);
      });
    }

    document.querySelectorAll(".hands-on-exercise").forEach((form) => {
      const input = form.querySelector(".exercise-input");
      const feedback = form.querySelector(".exercise-feedback");
      if (!input || !feedback) return;
      form.dataset.passed = "false";
      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const raw = input.value.trim().replace(/\s+/g, "");
        const kind = form.dataset.kind || "number";
        let ok = false;
        if (kind === "number") {
          const value = Number(raw);
          const expected = Number(form.dataset.answer);
          const tolerance = Number(form.dataset.tolerance || 0);
          ok = raw !== "" && Number.isFinite(value) &&
            Math.abs(value - expected) <= tolerance;
        } else if (kind === "choice") {
          ok = raw.toLowerCase() === String(form.dataset.answer).replace(/\s+/g, "").toLowerCase();
        }
        form.dataset.passed = ok ? "true" : "false";
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续完成下一题。")
          : (form.dataset.hint || "还不对：回到实时变量面板，先写出公式再代入。");
        updateProgress();
      });
      input.addEventListener("input", () => {
        form.dataset.passed = "false";
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
        updateProgress();
      });
    });

    const exerciseReset = document.getElementById("ch03-exercise-reset");
    if (exerciseReset) exerciseReset.addEventListener("click", () => {
      document.querySelectorAll(".hands-on-exercise").forEach(resetExercise);
      updateProgress();
    });

    window.addEventListener("sloc:theme", () => {
      drawHeatmap(heatCanvas, scenario, hover, S);
      drawComparison(compareCanvas, scenario, S);
    });

    const initial = readPrior();
    scenario = buildScenario(S, field, scan, ndt, initial.errXY, initial.errDeg);
    shadowIcp.pose = { ...scenario.prior };
    renderAll();
  }

  window.addEventListener("sloc:ready", init);
})();
