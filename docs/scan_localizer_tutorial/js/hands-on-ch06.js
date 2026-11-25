// Ch06 hands-on layer: minimal B&B reconstruction, real candidates, PF comparison,
// controlled timing and production-source steppers. No second algorithm is implemented here.
(function () {
  "use strict";

  const BNB_OPTS = Object.freeze({ beamsSubsample: 2, angStep: 0.25, nHypotheses: 3 });
  const GT = Object.freeze({ x: 16.5, y: 10.0, theta: 0.45 });
  const DELTA = Object.freeze({ x: 0.12, y: 0.0, theta: 0.05 });
  const N_PARTICLES = 400;
  const PF_STEPS = 10;
  const BASE_SEED = 20261125;
  const WARMUPS = 2;
  const SAMPLES = 7;
  const ERROR_GATE = Object.freeze({ linearM: 0.5, angularRad: 0.35 });
  const MATCHERS_FILE = "src/matchers.cpp";
  const PF_FILE = "src/particle_filter.cpp";

  const SOURCE_STAGES = {
    [MATCHERS_FILE]: [
      {
        title: "1 · 输入契约、粗点集与自由空间搜索域",
        lines: "716-766",
        explain: "全局搜索先拒绝无效地图、空扫描和非法角步长，再收集有限点。B&B 使用降采样点集，搜索域来自自由单元外接矩形；末端精修才使用全点集。",
        code: `GlobalResult global_correlative(
  const MapField & field, const Scan & scan,
  int beams_subsample, double ang_step,
  int n_hypotheses)
{
  GlobalResult out;
  if (!field.valid() || scan.points.empty() ||
    !std::isfinite(ang_step) || ang_step <= 0.0)
  {
    return out;
  }
  const auto * grid = &field.grid();
  const int topk = std::max(1, n_hypotheses);
  const int W = grid->width, H = grid->height;

    // 收集有限点并降采样（B&B 用粗点集，末端重评分用全点集）
  std::vector<Vec2> valid_pts;
  valid_pts.reserve(scan.points.size());
  for (const auto & pt : scan.points) {
    if (pt_is_finite(pt)) {valid_pts.push_back(pt);}
  }
  if (valid_pts.size() < 5) {return out;}

  std::vector<Vec2> sub_pts;
  const std::size_t step =
    static_cast<std::size_t>(std::max(1, beams_subsample));
  for (std::size_t i = 0; i < valid_pts.size(); i += step) {
    sub_pts.push_back(valid_pts[i]);
  }
  const int n = static_cast<int>(sub_pts.size());
  if (n < 5) {return out;}
  const int K = field.poolLevels() - 1;
  if (K < 0) {return out;}

    // 计算地图自由空间边界框作为机器人位姿搜索域（允许部分光束越界记 0 分）
  int free_xmin = W, free_xmax = -1, free_ymin = H, free_ymax = -1;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int8_t c = grid->cells[static_cast<std::size_t>(y) *
          static_cast<std::size_t>(W) + static_cast<std::size_t>(x)];
      if (c >= 0 && c <= 65) {
        free_xmin = std::min(free_xmin, x);
        free_xmax = std::max(free_xmax, x);
        free_ymin = std::min(free_ymin, y);
        free_ymax = std::max(free_ymax, y);
      }
    }
  }
  if (free_xmin > free_xmax || free_ymin > free_ymax) {
    return out;
  }`,
      },
      {
        title: "2 · 块上界与按 bound 排序的最大堆",
        lines: "796-818",
        explain: "完全越界的端点不参与求和，部分越界仍可读取夹紧锚点的滑窗最大值，因此只会高估。Node 的 operator< 反向比较 bound，使 std::priority_queue 的栈顶成为最大上界。",
        code: `    // 块上界打分：整块完全越界的光束严格记 0（避免 poolScore 边界夹紧导致虚高上界）
  auto eval_block_bound = [&](int level, int bx, int by, const AngData & A)
    {
      const int span = 1 << level;
      double sc = 0.0;
      for (const auto & c : A.cells) {
        const int px = c.x() + bx;
        const int py = c.y() + by;
        if (px >= W || px + span <= 0 || py >= H || py + span <= 0) {
          continue;
        }
        sc += field.poolScore(level, std::max(px, 0), std::max(py, 0));
      }
      return sc / n;
    };

  struct Node
  {
    double bound;
    int level, bx, by, aidx;
    bool operator<(const Node & o) const {return bound < o.bound;}
  };
  std::priority_queue<Node> pq;`,
      },
      {
        title: "3 · 播种、NMS 与 worst_kept 的更新契约",
        lines: "820-868",
        explain: "顶层块覆盖自由空间外接矩形。生产 NMS 使用 0.50m/0.35rad；Top-K 未满时阈值保持 0，满 K 后必须取所有已保留峰的最低分。",
        code: `    // 播种：顶层块（步长 2^K）覆盖自由空间边界框
  {
    const int G = 1 << K;
    for (int a = 0; a < n_ang; ++a) {
      const auto & A = ang[static_cast<std::size_t>(a)];
      const int x0 = static_cast<int>(
        std::floor(A.dx_min / static_cast<double>(G))) * G;
      const int y0 = static_cast<int>(
        std::floor(A.dy_min / static_cast<double>(G))) * G;
      for (int by = y0; by <= A.dy_max; by += G) {
        for (int bx = x0; bx <= A.dx_max; bx += G) {
          const double sc = eval_block_bound(K, bx, by, A);
          if (sc > 0.05) {pq.push({sc, K, bx, by, a});}
        }
      }
    }
  }
  if (pq.empty()) {return out;}

    // NMS 判定阈值（用于区分多峰假设，防止同一峰相邻像素占满 top-K）
  constexpr double kNmsLinM = 0.50;
  constexpr double kNmsAngRad = 0.35;

  struct Hyp { double score; int dx, dy, aidx; };
  std::vector<Hyp> hyps;
  hyps.reserve(static_cast<std::size_t>(topk));
  double worst_kept = 0.0;

  auto same_mode = [&](const Hyp & a, const Hyp & b)
    {
      const double dist_m = std::hypot(
        (a.dx - b.dx) * grid->resolution,
        (a.dy - b.dy) * grid->resolution);
      const double dang = std::abs(wrap_angle((a.aidx - b.aidx) * ang_step));
      return dist_m < kNmsLinM && dang < kNmsAngRad;
    };

  auto update_worst_kept = [&]()
    {
      if (static_cast<int>(hyps.size()) < topk) {
        worst_kept = 0.0;
        return;
      }
      double w = hyps[0].score;
      for (std::size_t i = 1; i < hyps.size(); ++i) {
        w = std::min(w, hyps[i].score);
      }
      worst_kept = w;
    };`,
      },
      {
        title: "4 · 出队即剪枝；叶子精确评分并维护 Top-K",
        lines: "870-916",
        explain: "队列按上界降序，因此队首 bound <= worst_kept 时剩余节点可全部放弃。叶节点先做精确评分，再合并同模式、替换最低分或追加候选，最后重算阈值。",
        code: `    // B&B 主循环
  std::int64_t expanded = 0;
  while (!pq.empty()) {
    const Node nd = pq.top();
    pq.pop();
    if (static_cast<int>(hyps.size()) >= topk && nd.bound <= worst_kept) {
      break;        // 剪枝：所有剩余节点上界均不超过当前第 K 优假设
    }
    const auto & A = ang[static_cast<std::size_t>(nd.aidx)];
    if (nd.level == 0) {
      if (nd.bx < A.dx_min || nd.bx > A.dx_max ||
        nd.by < A.dy_min || nd.by > A.dy_max)
      {
        continue;
      }
        // 精确重评分（出界点 0 分，无滑窗膨胀）
      double sc = 0.0;
      for (const auto & c : A.cells) {
        sc += field.likelihood(c.x() + nd.bx, c.y() + nd.by);
      }
      sc /= n;
      if (sc > 0.10) {
        const Hyp cand{sc, nd.bx, nd.by, nd.aidx};
        bool merged = false;
        for (auto & h : hyps) {
          if (same_mode(h, cand)) {
            if (cand.score > h.score) {h = cand;}
            merged = true;
            break;
          }
        }
        if (!merged) {
          if (static_cast<int>(hyps.size()) < topk) {
            hyps.push_back(cand);
          } else {
            auto worst_it = std::min_element(
              hyps.begin(), hyps.end(),
              [](const Hyp & a, const Hyp & b) {return a.score < b.score;});
            if (cand.score > worst_it->score) {
              *worst_it = cand;
            }
          }
        }
        update_worst_kept();
      }
      continue;
    }`,
      },
      {
        title: "5 · 四分裂：只入队仍有希望超过第 K 优的子块",
        lines: "918-937",
        explain: "half 是子块跨度的一半，不是分辨率。子块先与搜索域求交，再计算上界；Top-K 未满或 bound > worst_kept 时才入队。expanded 同时作为防御性搜索上限。",
        code: `    const int h = 1 << (nd.level - 1);
    for (int dy = 0; dy < 2; ++dy) {
      for (int dx = 0; dx < 2; ++dx) {
        const int bx = nd.bx + dx * h;
        const int by = nd.by + dy * h;
        if (bx > A.dx_max || by > A.dy_max ||
          bx + h - 1 < A.dx_min || by + h - 1 < A.dy_min)
        {
          continue;
        }
        const double sc = eval_block_bound(nd.level - 1, bx, by, A);
        if (sc > 0.05 &&
          (static_cast<int>(hyps.size()) < topk || sc > worst_kept))
        {
          pq.push({sc, nd.level - 1, bx, by, nd.aidx});
        }
        ++expanded;
      }
    }
    if (expanded > 30000000LL) {break;}        // 防御性上限`,
      },
      {
        title: "6 · 小窗口精修：把离散候选变成连续位姿",
        lines: "941-980",
        explain: "每个粗候选在 ±2 格平移、±半角步长范围内用全点集重评分。教学 sim-core 目前没有这一段，所以候选面板只展示离散 B&B 结果。",
        code: `    // 每个假设：连续小窗口细修 + 全点集重评分（点云旋转提至 5x5 平移循环外）
  struct Refined { double score; Pose2 pose; };
  std::vector<Refined> refined;
  refined.reserve(hyps.size());
  std::vector<Eigen::Vector2d> rot_pts;
  rot_pts.reserve(valid_pts.size());
  for (const auto & hyp : hyps) {
    const double th0 = hyp.aidx * ang_step;
    const Eigen::Vector2d w0 = grid->gridToWorld(hyp.dx, hyp.dy);
    double best_sc = -1.0;
    double bx = w0.x(), by = w0.y(), bth = wrap_angle(th0);
    for (int dth = -4; dth <= 4; ++dth) {
      const double th = wrap_angle(th0 + dth * ang_step / 8.0);
      const double cc = std::cos(th), ss = std::sin(th);
      rot_pts.clear();
      for (const auto & pt : valid_pts) {
        rot_pts.emplace_back(
          cc * pt.x() - ss * pt.y(), ss * pt.x() + cc * pt.y());
      }
      for (int dy = -2; dy <= 2; ++dy) {
        const double py = w0.y() + dy * grid->resolution;
        for (int dx = -2; dx <= 2; ++dx) {
          const double px = w0.x() + dx * grid->resolution;
          double sc = 0.0;
          for (const auto & rpt : rot_pts) {
            const auto g = grid->worldToGridF(px + rpt.x(), py + rpt.y());
            sc += field.likelihoodAtF(g.x(), g.y());
          }
          sc /= static_cast<double>(rot_pts.size());
          if (sc > best_sc) {
            best_sc = sc;
            bx = px;
            by = py;
            bth = th;
          }
        }
      }
    }
    refined.push_back({best_sc, {bx, by, bth}});
  }`,
      },
      {
        title: "7 · 精修后再次 NMS，并输出 pose 与 alternates",
        lines: "986-1013",
        explain: "粗筛后的 NMS 不能替代最终 NMS：连续精修可能让原本分开的候选重新靠近。最终按分数排序，保留互不重复的峰，总数不超过 topk。",
        code: `    // 精修后最终 NMS 去重，保证 alternates 满足最小间距且总数 <= topk
  std::vector<Refined> dedup;
  dedup.reserve(static_cast<std::size_t>(topk));
  for (const auto & r : refined) {
    bool dup = false;
    for (const auto & kept : dedup) {
      const double dist_m = std::hypot(
        r.pose.x - kept.pose.x, r.pose.y - kept.pose.y);
      const double dang = std::abs(wrap_angle(r.pose.theta - kept.pose.theta));
      if (dist_m < kNmsLinM && dang < kNmsAngRad) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      dedup.push_back(r);
      if (static_cast<int>(dedup.size()) >= topk) {break;}
    }
  }
  if (dedup.empty()) {return out;}

  out.pose = dedup[0].pose;
  out.score = dedup[0].score;
  for (std::size_t i = 1; i < dedup.size(); ++i) {
    out.alternates.push_back(dedup[i].pose);
  }
  out.valid = out.score > 0.10;
  return out;`,
      },
    ],
    [PF_FILE]: [
      {
        title: "1 · Thrun 运动采样： rot1 → trans → rot2",
        lines: "99-118",
        explain: "sample_motion 只负责带噪声的机体增量。三个标准差分别由旋转/平移噪声系数构造，随后把带符号平移旋转到机器人机体坐标系。",
        code: `// Thrun 采样里程计运动模型（支持带符号平移以兼容倒车分解）
Pose2 sample_motion(
  double dth1, double dtrans, double dth2,
  const PfOptions & o, std::mt19937 & rng)
{
  std::normal_distribution<double> g(0.0, 1.0);
  const double sd_rot1 = std::sqrt(
    std::max(0.0, o.alpha_rot_rot * dth1 * dth1 +
    o.alpha_rot_trans * dtrans * dtrans));
  const double sd_trans = std::sqrt(
    std::max(0.0, o.alpha_trans_trans * dtrans * dtrans +
    o.alpha_trans_rot * (dth1 * dth1 + dth2 * dth2)));
  const double sd_rot2 = std::sqrt(
    std::max(0.0, o.alpha_rot_rot * dth2 * dth2 +
    o.alpha_rot_trans * dtrans * dtrans));
  const double dh1 = dth1 - sd_rot1 * g(rng);
  const double dt_ = dtrans - sd_trans * g(rng);
  const double dh2 = dth2 - sd_rot2 * g(rng);
  return {dt_ * std::cos(dh1), dt_ * std::sin(dh1), wrap_angle(dh1 + dh2)};
}`,
      },
      {
        title: "2 · 倒车反向分解，避免 ±π 噪声爆炸",
        lines: "121-148",
        explain: "当 odom 平移方向接近 ±π 时，rot1 加 π、signed_trans 取反，把“倒车”改写为等价的短旋转 + 负平移。rot2 再用分解后的 rot1 计算。",
        code: `void ParticleFilter::predict(const Pose2 & odom_delta, std::mt19937 & rng)
{
  if (particles_.empty()) {return;}
  const double dx = odom_delta.x, dy = odom_delta.y;
  const double dth = wrap_angle(odom_delta.theta);
  const double trans = std::hypot(dx, dy);
  if (trans < 1e-9 && std::abs(dth) < 1e-9) {return;}

    // 分解为 rot1-trans-rot2；倒车（|rot1| > π/2）时反向分解避免 ±π 噪声爆炸
  double rot1 = trans < 1e-6 ? 0.0 : wrap_angle(std::atan2(dy, dx));
  double signed_trans = trans;
  if (std::abs(rot1) > 0.5 * kPi) {
    rot1 = wrap_angle(rot1 + kPi);
    signed_trans = -trans;
  }
  const double rot2 = wrap_angle(dth - rot1);

  accum_d_ += trans;
  accum_a_ += std::abs(dth);

  for (auto & p : particles_) {
    const Pose2 step = sample_motion(rot1, signed_trans, rot2, opt_, rng);
    const double c = std::cos(p.th), s = std::sin(p.th);
    p.x += c * step.x - s * step.y;
    p.y += s * step.x + c * step.y;
    p.th = wrap_angle(p.th + step.theta);
  }
}`,
      },
      {
        title: "3 · 观测子采样、有效光束缩放与对数似然",
        lines: "170-211",
        explain: "obs_subsample 先限制点数和无效输入。beam_scale 限制强相关光束的总权重，log(0.9*lk+0.1) 避免单项为零时整个粒子对数权重变成 -∞。",
        code: `    // 观测子采样（复用缓冲区，防 obs_subsample <= 0 死循环）
  sub_pts_.clear();
  const std::size_t step =
    static_cast<std::size_t>(std::max(1, opt_.obs_subsample));
  for (std::size_t i = 0; i < scan.points.size(); i += step) {
    const auto & pt = scan.points[i];
    if (std::isfinite(pt.x()) && std::isfinite(pt.y())) {
      sub_pts_.push_back(pt);
    }
  }
  const int n = static_cast<int>(sub_pts_.size());
  if (n == 0) {return 0.0;}

  const double z_hit = std::clamp(opt_.z_hit, 0.0, 1.0);
  const double z_rand = std::max(opt_.z_rand, 1e-6);
    // 有效光束数上限（抑制相邻光束强相关导致的单帧粒子塌缩）
  const double beam_scale = std::min(1.0, 30.0 / static_cast<double>(n));

  log_w_buf_.resize(particles_.size());
  double ssum = 0.0;
  double max_lw = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    const auto & p = particles_[i];
    const double c = std::cos(p.th), s = std::sin(p.th);
    double log_like = 0.0;
    double sc = 0.0;
    for (const auto & pt : sub_pts_) {
      const double wx = p.x + c * pt.x() - s * pt.y();
      const double wy = p.y + s * pt.x() + c * pt.y();
      const auto g = field.grid().worldToGridF(wx, wy);
      const double lk = field.likelihoodAtF(g.x(), g.y());
      sc += lk;
      log_like += std::log(z_hit * lk + z_rand);
    }
    sc /= n;
    ssum += sc;
    const double lw = std::log(std::max(p.w, 1e-300)) + beam_scale * log_like;
    log_w_buf_[i] = lw;
    if (std::isfinite(lw) && lw > max_lw) {
      max_lw = lw;
    }
  }`,
      },
      {
        title: "4 · log-sum-exp 归一化与非有限权重回退",
        lines: "213-241",
        explain: "先减 max_lw 再 exp，避免大量粒子同时下溢。max_lw 非有限或权重和为 0 时恢复均匀权重；这是数值保护，不代表观测支持所有粒子。",
        code: `    // log-sum-exp 归一化（防下溢与 NaN）
  const double inv_n = 1.0 / static_cast<double>(particles_.size());
  if (!std::isfinite(max_lw)) {
    for (auto & p : particles_) {
      p.w = inv_n;
    }
  } else {
    double wsum = 0.0;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      const double w = std::isfinite(log_w_buf_[i]) ?
        std::exp(log_w_buf_[i] - max_lw) : 0.0;
      particles_[i].w = w;
      wsum += w;
    }
    if (wsum <= 0.0 || !std::isfinite(wsum)) {
      for (auto & p : particles_) {
        p.w = inv_n;
      }
    } else {
      for (auto & p : particles_) {
        p.w /= wsum;
      }
    }
  }

  mean_score_ = ssum / static_cast<double>(particles_.size());
  accum_d_ = 0.0;
  accum_a_ = 0.0;
  has_updated_ = true;`,
      },
      {
        title: "5 · ESS 门控：低于阈值才重采样",
        lines: "243-253",
        explain: "ESS = 1/Σw²。权重越集中，ESS 越小；ess_frac*N 是触发系统重采样的阈值。函数返回 ESS/N，便于监控退化程度。",
        code: `    // ESS 重采样
  double ess = 0.0;
  for (const auto & p : particles_) {
    ess += p.w * p.w;
  }
  ess = ess > 0.0 ? 1.0 / ess : 0.0;
  const double ess_needed = opt_.ess_frac * static_cast<double>(particles_.size());
  if (ess < ess_needed) {
    resample(rng, &field.grid());
  }
  return ess / static_cast<double>(particles_.size());`,
      },
      {
        title: "6 · 圆均值角度与完整 3×3 协方差",
        lines: "256-290",
        explain: "x/y 是普通加权均值，角度必须用 sin/cos 圆均值。协方差中的角残差同样先 wrap，避免 ±π 边界产生巨大伪方差。",
        code: `void ParticleFilter::estimate(Pose2 & mean, Cov3 & cov) const
{
  mean = {};
  cov = Cov3::Identity();
  if (particles_.empty()) {return;}
  double sx = 0.0, sy = 0.0, sth_sin = 0.0, sth_cos = 0.0;
  for (const auto & p : particles_) {
    sx += p.w * p.x;
    sy += p.w * p.y;
    sth_sin += p.w * std::sin(p.th);
    sth_cos += p.w * std::cos(p.th);
  }
  mean = {sx, sy, wrap_angle(std::atan2(sth_sin, sth_cos))};

  double cxx = 0.0, cyy = 0.0, ctt = 0.0;
  double cxy = 0.0, cxt = 0.0, cyt = 0.0;
  for (const auto & p : particles_) {
    const double dx = p.x - mean.x;
    const double dy = p.y - mean.y;
    const double dth = wrap_angle(p.th - mean.theta);
    cxx += p.w * dx * dx;
    cyy += p.w * dy * dy;
    ctt += p.w * dth * dth;
    cxy += p.w * dx * dy;
    cxt += p.w * dx * dth;
    cyt += p.w * dy * dth;
  }
  cov = Cov3::Zero();
  cov(0, 0) = cxx;
  cov(1, 1) = cyy;
  cov(2, 2) = ctt;
  cov(0, 1) = cov(1, 0) = cxy;
  cov(0, 2) = cov(2, 0) = cxt;
  cov(1, 2) = cov(2, 1) = cyt;
}`,
      },
      {
        title: "7 · 系统重采样：等间距累积权重指针",
        lines: "292-312",
        explain: "r 均匀取自 [0,1/N)，随后 u_m=r+m/N。指针只向右移动，每个输出粒子权重重置为 1/N；可选随机粒子注入位于此片段之后。",
        code: `void ParticleFilter::resample(std::mt19937 & rng, const OccupancyGrid * grid)
{
    // 低方差重采样（复用成员缓冲区避免每帧堆分配）
  const std::size_t n = particles_.size();
  if (n == 0) {return;}
  resample_buf_.clear();
  resample_buf_.reserve(n);
  const double inv_n = 1.0 / static_cast<double>(n);
  std::uniform_real_distribution<double> u(0.0, inv_n);
  const double r = u(rng);
  double c = particles_[0].w;
  std::size_t i = 0;
  for (std::size_t m = 0; m < n; ++m) {
    const double um = r + static_cast<double>(m) * inv_n;
    while (um > c && i + 1 < n) {
      ++i;
      c += particles_[i].w;
    }
    resample_buf_.push_back(particles_[i]);
    resample_buf_.back().w = inv_n;
  }`,
      },
    ],
  };

  let S = null;
  let grid = null;
  let field = null;
  let scan = null;
  let frames = [];
  let truths = [];
  let result = null;
  let traceStep = 0;
  let pfSnapshot = null;
  let activeSourceFile = MATCHERS_FILE;
  let sourceStage = 0;
  let benchmarkRunning = false;

  function fmt(value, digits = 4) {
    if (value === null || value === undefined) return "—";
    if (typeof value === "boolean") return value ? "true" : "false";
    if (typeof value === "number") {
      if (value === Infinity) return "∞";
      if (value === -Infinity) return "-∞";
      if (!Number.isFinite(value)) return String(value);
      const av = Math.abs(value);
      if (av !== 0 && (av < 1e-3 || av >= 1e4)) return value.toExponential(2);
      return value.toFixed(digits).replace(/\.?0+$/, "");
    }
    return String(value);
  }

  function poseText(pose) {
    if (!pose) return "—";
    return `(${fmt(pose.x)}, ${fmt(pose.y)}, ${fmt(pose.theta)})`;
  }

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function fillVars(id, rows) {
    const host = document.getElementById(id);
    if (!host) return;
    host.textContent = "";
    const fragment = document.createDocumentFragment();
    rows.forEach(([key, value]) => {
      const cell = document.createElement("div");
      cell.className = "hands-on-var";
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = key;
      dd.textContent = value;
      cell.appendChild(dt);
      cell.appendChild(dd);
      fragment.appendChild(cell);
    });
    host.appendChild(fragment);
  }

  function renderRows(id, rows, emptyCols, emptyText) {
    const body = document.getElementById(id);
    if (!body) return;
    body.textContent = "";
    if (!rows.length) {
      const tr = document.createElement("tr");
      const td = document.createElement("td");
      td.colSpan = emptyCols;
      td.textContent = emptyText;
      tr.appendChild(td);
      body.appendChild(tr);
      return;
    }
    const fragment = document.createDocumentFragment();
    rows.forEach((row) => {
      const tr = document.createElement("tr");
      row.forEach((value) => {
        const td = document.createElement("td");
        td.textContent = String(value);
        tr.appendChild(td);
      });
      fragment.appendChild(tr);
    });
    body.appendChild(fragment);
  }

  function highlight(code) {
    if (!code) return;
    code.className = "language-cpp";
    delete code.dataset.highlighted;
    if (window.hljs) {
      try { window.hljs.highlightElement(code); } catch (_) {}
    }
  }

  function wrap(value) {
    return S.wrapAngle(value);
  }

  function poseMetrics(pose, truth) {
    if (!pose || !truth) return { linear: Infinity, angular: Infinity, pass: false };
    const linear = Math.hypot(pose.x - truth.x, pose.y - truth.y);
    const angular = Math.abs(wrap(pose.theta - truth.theta));
    return {
      linear,
      angular,
      pass: linear < ERROR_GATE.linearM && angular < ERROR_GATE.angularRad,
    };
  }

  function advancePose(pose) {
    return {
      x: pose.x + Math.cos(pose.theta) * DELTA.x,
      y: pose.y + Math.sin(pose.theta) * DELTA.x,
      theta: wrap(pose.theta + DELTA.theta),
    };
  }

  function makeRng(seed) {
    let state = seed >>> 0;
    return function () {
      state = (state * 1664525 + 1013904223) >>> 0;
      return state / 4294967296;
    };
  }

  function buildScenario() {
    grid = S.createWarehouseMap(0.2);
    field = new S.MapField(grid, 0.25);
    scan = S.makeScan(grid, GT, { beams: 108, maxRange: 10 });
    frames = [];
    truths = [];
    let truth = { ...GT };
    for (let i = 0; i < PF_STEPS; i++) {
      truth = advancePose(truth);
      truths.push(truth);
      frames.push(S.makeScan(grid, truth, { beams: 108, maxRange: 10 }));
    }
    result = S.globalCorrelativeBnB(field, scan, BNB_OPTS);
  }

  function runBnb(targetScan) {
    return S.globalCorrelativeBnB(field, targetScan, BNB_OPTS);
  }

  function renderCandidates() {
    if (!result) return;
    const poses = Array.isArray(result.poses) ? result.poses : [];
    const best = result.pose || poses[0] || null;
    const bestMetrics = poseMetrics(best, GT);
    const rows = poses.map((pose, index) => {
      const metrics = poseMetrics(pose, GT);
      return [
        index + 1,
        fmt(pose.x),
        fmt(pose.y),
        fmt(pose.theta),
        fmt(pose.score),
        fmt(metrics.linear),
        fmt(metrics.angular),
      ];
    });
    renderRows("ch06-candidates-body", rows, 7, result.valid ? "函数未返回候选" : "globalCorrelativeBnB 返回 invalid");
    fillVars("ch06-result-vars", [
      ["调用", "S.globalCorrelativeBnB(field, scan, opts)"],
      ["valid", result.valid],
      ["返回候选数", poses.length],
      ["best pose", poseText(best)],
      ["best score", fmt(best ? best.score : null)],
      ["best 平移 / 角误差", `${fmt(bestMetrics.linear)} m / ${fmt(bestMetrics.angular)} rad`],
      ["expanded / pruned", `${fmt(result.expanded)} / ${fmt(result.pruned)}`],
      ["trace 已保存 / 上限", `${fmt(result.trace ? result.trace.length : 0)} / 360`],
      ["地图", `${grid.width}×${grid.height} @ ${fmt(grid.resolution)}m`],
      ["参数", `subsample=${BNB_OPTS.beamsSubsample}, angStep=${BNB_OPTS.angStep}, topK=${BNB_OPTS.nHypotheses}`],
      ["固定真值", poseText(GT)],
    ]);
    setText("ch06-result-status", result.valid
      ? `已返回 ${poses.length} 个真实候选；最佳平移误差 ${fmt(bestMetrics.linear)}m`
      : "返回 invalid；请检查扫描与地图输入");
    renderTrace();
    renderSource();
  }

  function renderTrace() {
    const nodes = result && Array.isArray(result.trace) ? result.trace : [];
    const prev = document.getElementById("ch06-trace-prev");
    const next = document.getElementById("ch06-trace-next");
    if (!nodes.length) {
      traceStep = 0;
      setText("ch06-trace-title", "没有可展示的 trace 节点");
      setText("ch06-trace-meta", "S.globalCorrelativeBnB(...).trace = []");
      setText("ch06-trace-index", "0 / 0");
      setText("ch06-trace-explain", "本次调用未保存子节点。trace 只记录前 360 个入队节点，为空不代表没有执行搜索。");
      fillVars("ch06-trace-vars", [["提示", "检查 result.valid、expanded 与 pruned"]]);
      if (prev) prev.disabled = true;
      if (next) next.disabled = true;
      return;
    }
    traceStep = Math.max(0, Math.min(traceStep, nodes.length - 1));
    const node = nodes[traceStep];
    const theta = wrap(node.aidx * BNB_OPTS.angStep);
    setText("ch06-trace-title", `节点 ${traceStep + 1}：level ${node.level}，bound ${fmt(node.bound)}`);
    setText("ch06-trace-meta", `result.trace[${traceStep}] · 入队子节点样本`);
    setText("ch06-trace-index", `${traceStep + 1} / ${nodes.length}`);
    setText("ch06-trace-explain", node.level === 0
      ? "这是 level-0 叶节点的入队记录；出队后会执行精确似然评分。trace 不保证它就是下一次 pop 的节点。"
      : `该节点跨度 ${1 << node.level} 格；若出队且未被剪枝，将分裂为四个 ${1 << Math.max(0, node.level - 1)} 格子块。`);
    fillVars("ch06-trace-vars", [
      ["bound", fmt(node.bound)],
      ["level", node.level],
      ["bx / by", `${node.bx} / ${node.by}`],
      ["span (格)", node.span],
      ["aidx", node.aidx],
      ["θ (rad)", fmt(theta)],
      ["trace 截断", nodes.length === 360 ? "已到 360 条上限" : "未到上限"],
    ]);
    if (prev) prev.disabled = traceStep === 0;
    if (next) next.disabled = traceStep === nodes.length - 1;
  }

  function executePf(steps, seed, collect) {
    const rng = makeRng(seed);
    const pf = new S.ParticleFilter(N_PARTICLES, rng);
    pf.initGlobal(grid);
    let essRatio = 1.0;
    const details = [];
    for (let i = 0; i < steps; i++) {
      pf.predict(DELTA);
      essRatio = pf.update(field, frames[i], 3);
      if (collect) {
        details.push({
          update: i + 1,
          essRatio,
          resampled: essRatio < 0.5,
          estimate: pf.estimate(),
          truth: truths[i],
        });
      }
    }
    return { pf, essRatio, details };
  }

  function renderPf() {
    if (!pfSnapshot) return;
    const execution = pfSnapshot.execution;
    const estimate = execution.pf.estimate();
    const metrics = poseMetrics(estimate, truths[PF_STEPS - 1]);
    const last = execution.details[execution.details.length - 1];
    pfSnapshot.estimate = estimate;
    pfSnapshot.metrics = metrics;
    pfSnapshot.lastEssRatio = execution.essRatio;
    fillVars("ch06-pf-vars", [
      ["随机种子", pfSnapshot.seed],
      ["粒子数", N_PARTICLES],
      ["预测 / 更新帧数", PF_STEPS],
      ["最终 PF 均值", poseText(estimate)],
      ["最终真值", poseText(truths[PF_STEPS - 1])],
      ["最终平移 / 角误差", `${fmt(metrics.linear)} m / ${fmt(metrics.angular)} rad`],
      ["最终 ESS / N", fmt(execution.essRatio)],
      ["最后一帧重采样", last && last.resampled ? "是" : "否"],
      ["同一验收门限", metrics.pass ? "通过" : "未通过"],
    ]);
    const rows = execution.details.map((detail) => {
      const m = poseMetrics(detail.estimate, detail.truth);
      return [
        detail.update,
        fmt(detail.essRatio),
        detail.resampled ? "是" : "否",
        fmt(m.linear),
        fmt(m.angular),
      ];
    });
    renderRows("ch06-pf-steps-body", rows, 5, "没有 PF 更新记录");
    renderSource();
  }

  function runPfUi() {
    const input = document.getElementById("ch06-pf-seed");
    const parsed = Number(input ? input.value : BASE_SEED);
    const seed = Number.isFinite(parsed) ? (parsed >>> 0) : BASE_SEED;
    if (input) input.value = String(seed);
    const execution = executePf(PF_STEPS, seed, true);
    pfSnapshot = { seed, execution };
    renderPf();
  }

  function measureBnb(scans, truth) {
    const start = performance.now();
    let last = null;
    for (let i = 0; i < scans.length; i++) last = runBnb(scans[i]);
    const ms = performance.now() - start;
    return { ms, ...poseMetrics(last ? last.pose : null, truth) };
  }

  function measurePf(steps, seed) {
    const start = performance.now();
    const execution = executePf(steps, seed, false);
    const ms = performance.now() - start;
    const estimate = execution.pf.estimate();
    return {
      ms,
      ...poseMetrics(estimate, truths[steps - 1]),
      essRatio: execution.essRatio,
    };
  }

  function median(values) {
    const sorted = values.slice().sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    if (sorted.length % 2) return sorted[mid];
    return (sorted[mid - 1] + sorted[mid]) / 2;
  }

  function summarize(samples, withEss) {
    const byTime = samples.slice().sort((a, b) => a.ms - b.ms);
    const representative = byTime[Math.floor(byTime.length / 2)];
    return {
      ms: median(samples.map((sample) => sample.ms)),
      linear: representative.linear,
      angular: representative.angular,
      passes: samples.filter((sample) => sample.pass).length,
      total: samples.length,
      essRatio: withEss ? representative.essRatio : null,
    };
  }

  function renderBenchmark(summary) {
    const specs = [
      ["B&B · 1 帧全局解", 1, summary.bnb1],
      ["PF · 初始化 + 1 次更新", 1, summary.pf1],
      ["B&B · 10 帧各解一次", 10, summary.bnb10],
      ["PF · 初始化 + 10 次更新", 10, summary.pf10],
    ];
    const rows = specs.map(([label, budget, item]) => [
      label,
      budget,
      fmt(item.ms, 3),
      fmt(item.linear),
      fmt(item.angular),
      `${item.passes} / ${item.total}`,
      item.essRatio === null ? "—" : fmt(item.essRatio),
    ]);
    renderRows("ch06-benchmark-body", rows, 7, "尚无计时样本");
    fillVars("ch06-benchmark-vars", [
      ["计时 API", "performance.now()"],
      ["预热 / 采样", `${WARMUPS} / ${SAMPLES}`],
      ["PF 种子序列", `${BASE_SEED} … ${BASE_SEED + SAMPLES - 1}`],
      ["统计量", "耗时为中位数；误差/ESS 取中位耗时对应试验"],
      ["通过门限", `误差 < ${ERROR_GATE.linearM}m 且 < ${ERROR_GATE.angularRad}rad`],
      ["计时区外", "建图、makeScan、真值推进、误差计算、DOM/画布"],
      ["结论边界", "JS 教学核；不计算跨任务加速比"],
    ]);
    setText("ch06-benchmark-status", `完成：每任务 ${WARMUPS} 次预热 + ${SAMPLES} 次采样`);
  }

  function runBenchmark() {
    if (benchmarkRunning) return;
    benchmarkRunning = true;
    const button = document.getElementById("ch06-benchmark-btn");
    if (button) button.disabled = true;
    setText("ch06-benchmark-status", "正在预热并采样，请稍候…");

    window.setTimeout(() => {
      try {
        for (let i = 0; i < WARMUPS; i++) {
          const seed = BASE_SEED + SAMPLES + i;
          runBnb(frames[0]);
          executePf(1, seed, false);
          for (let k = 0; k < PF_STEPS; k++) runBnb(frames[k]);
          executePf(PF_STEPS, seed, false);
        }

        const samples = { bnb1: [], pf1: [], bnb10: [], pf10: [] };
        for (let i = 0; i < SAMPLES; i++) {
          const seed = BASE_SEED + i;
          samples.bnb1.push(measureBnb([frames[0]], truths[0]));
          samples.pf1.push(measurePf(1, seed));
          samples.bnb10.push(measureBnb(frames, truths[PF_STEPS - 1]));
          samples.pf10.push(measurePf(PF_STEPS, seed));
        }

        renderBenchmark({
          bnb1: summarize(samples.bnb1, false),
          pf1: summarize(samples.pf1, true),
          bnb10: summarize(samples.bnb10, false),
          pf10: summarize(samples.pf10, true),
        });
      } catch (error) {
        setText("ch06-benchmark-status", `计时失败：${error && error.message ? error.message : String(error)}`);
      } finally {
        benchmarkRunning = false;
        if (button) button.disabled = false;
      }
    }, 20);
  }

  function sourceVars() {
    if (activeSourceFile === MATCHERS_FILE) {
      const poses = result && Array.isArray(result.poses) ? result.poses : [];
      const best = result && result.pose ? result.pose : poses[0];
      if (sourceStage === 0) return [
        ["扫描点数", fmt(scan.length)],
        ["粗点集点数", fmt(Math.ceil(scan.length / BNB_OPTS.beamsSubsample))],
        ["top-K", BNB_OPTS.nHypotheses],
        ["自由空间外接矩形", "由 grid.cells 实时统计"],
      ];
      if (sourceStage === 1) return [
        ["poolLevels()", fmt(field.poolLevels())],
        ["顶层 K", field.poolLevels() - 1],
        ["顶层 span (格)", 1 << (field.poolLevels() - 1)],
        ["可采纳性", "完全越界端点记 0；部分越界只可能高估"],
      ];
      if (sourceStage === 2) return [
        ["生产 NMS", "0.50m / 0.35rad"],
        ["教学核 NMS", "0.80m / 0.45rad"],
        ["Top-K 容量", BNB_OPTS.nHypotheses],
      ];
      if (sourceStage === 3 || sourceStage === 4) return [
        ["result.valid", fmt(result.valid)],
        ["expanded", fmt(result.expanded)],
        ["pruned", fmt(result.pruned)],
        ["trace 样本", fmt(result.trace ? result.trace.length : 0)],
      ];
      if (sourceStage === 5) return [
        ["教学核 best", poseText(best)],
        ["生产末端精修", "±2 格 / ±angStep/2；教学核未实现"],
      ];
      return [
        ["教学核返回峰数", poses.length],
        ["教学核 best score", fmt(best ? best.score : null)],
        ["生产 alternates", "来自精修后再次 NMS 的 dedup[1..]"],
      ];
    }

    if (!pfSnapshot) return [["PF 状态", "尚未运行固定种子对照"]];
    const last = pfSnapshot.execution.details[pfSnapshot.execution.details.length - 1];
    if (sourceStage === 0) return [
      ["粒子数", N_PARTICLES],
      ["odom delta", `(${DELTA.x}, ${DELTA.y}, ${DELTA.theta})`],
      ["trans / rot1", `${fmt(Math.hypot(DELTA.x, DELTA.y))} / 0`],
      ["rot2", fmt(DELTA.theta)],
    ];
    if (sourceStage === 1) return [
      ["本例 rot1", "0（不触发倒车反向分解）"],
      ["signed_trans", fmt(Math.hypot(DELTA.x, DELTA.y))],
      ["rot2", fmt(DELTA.theta)],
    ];
    const n = frames[0].length;
    const subCount = Math.ceil(n / 3);
    if (sourceStage === 2) return [
      ["首帧原始点数", n],
      ["subsample=3 后点数", subCount],
      ["beam_scale", fmt(Math.min(1, 30 / Math.max(1, subCount)))],
      ["混合似然", "log(0.9*lk + 0.1)"],
    ];
    if (sourceStage === 3) return [
      ["最后 ESS / N", fmt(pfSnapshot.lastEssRatio)],
      ["权重有限", pfSnapshot.execution.pf.particles.every((p) => Number.isFinite(p.w))],
      ["归一化保护", "减 max_lw；异常时均匀回退"],
    ];
    if (sourceStage === 4) return [
      ["最后 ESS / N", fmt(pfSnapshot.lastEssRatio)],
      ["是否触发", last && last.resampled ? "是" : "否"],
      ["教学核门限", "ESS/N < 0.5"],
    ];
    if (sourceStage === 5) return [
      ["PF 圆均值", poseText(pfSnapshot.estimate)],
      ["真值", poseText(truths[PF_STEPS - 1])],
      ["平移 / 角误差", pfSnapshot.metrics ? `${fmt(pfSnapshot.metrics.linear)} / ${fmt(pfSnapshot.metrics.angular)}` : "—"],
    ];
    return [
      ["固定种子", pfSnapshot.seed],
      ["重采样后权重", `1/${N_PARTICLES}`],
      ["生产可选项", "random_particle_ratio 可在绑架后注入随机粒子"],
    ];
  }

  function renderSource() {
    const stages = SOURCE_STAGES[activeSourceFile];
    if (!stages || !result && activeSourceFile === MATCHERS_FILE) return;
    sourceStage = Math.max(0, Math.min(sourceStage, stages.length - 1));
    const stage = stages[sourceStage];
    setText("ch06-stage-title", stage.title);
    setText("ch06-stage-meta", `${activeSourceFile}:L${stage.lines} · ${sourceStage + 1}/${stages.length}`);
    setText("ch06-stage-explain", stage.explain);
    setText("ch06-stage-index", `${sourceStage + 1} / ${stages.length}`);
    const code = document.getElementById("ch06-stage-code");
    if (code) {
      code.textContent = stage.code;
      highlight(code);
    }
    const sourceBtn = document.getElementById("ch06-stage-source");
    if (sourceBtn) {
      sourceBtn.setAttribute("data-file", activeSourceFile);
      sourceBtn.setAttribute("data-lines", stage.lines);
      sourceBtn.setAttribute("data-label", `${activeSourceFile} L${stage.lines}`);
    }
    const prev = document.getElementById("ch06-stage-prev");
    const next = document.getElementById("ch06-stage-next");
    if (prev) prev.disabled = sourceStage === 0;
    if (next) next.disabled = sourceStage === stages.length - 1;
    const bnbBtn = document.getElementById("ch06-source-bnb");
    const pfBtn = document.getElementById("ch06-source-pf");
    if (bnbBtn) bnbBtn.setAttribute("aria-pressed", String(activeSourceFile === MATCHERS_FILE));
    if (pfBtn) pfBtn.setAttribute("aria-pressed", String(activeSourceFile === PF_FILE));
    fillVars("ch06-source-vars", sourceVars());
  }

  function bindExercises() {
    const forms = Array.from(document.querySelectorAll(".hands-on-exercise"));
    const passed = new Set();
    const progress = document.getElementById("ch06-exercise-progress");
    function updateProgress() {
      if (progress) progress.textContent = `${passed.size} / ${forms.length} 通过`;
    }
    forms.forEach((form) => {
      const input = form.querySelector(".exercise-input");
      const feedback = form.querySelector(".exercise-feedback");
      if (!input || !feedback) return;
      const kind = form.dataset.kind || "number";
      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const raw = input.value.trim();
        let ok = false;
        if (kind === "choice") {
          ok = raw.replace(/\s+/g, "").toLowerCase() ===
            String(form.dataset.answer || "").replace(/\s+/g, "").toLowerCase();
        } else {
          const value = Number(raw);
          const expected = Number(form.dataset.answer);
          const tolerance = Number(form.dataset.tolerance || 0);
          ok = raw !== "" && Number.isFinite(value) && Math.abs(value - expected) <= tolerance;
        }
        if (ok) passed.add(form);
        else passed.delete(form);
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续尝试下一题。")
          : (form.dataset.hint || "还不对：先写出公式或分支，再代入数值。");
        updateProgress();
      });
      input.addEventListener("input", () => {
        passed.delete(form);
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
        updateProgress();
      });
    });
    updateProgress();
  }

  function bindControls() {
    document.getElementById("ch06-rerun-btn")?.addEventListener("click", () => {
      result = runBnb(scan);
      traceStep = 0;
      renderCandidates();
    });
    document.getElementById("ch06-trace-prev")?.addEventListener("click", () => {
      if (traceStep > 0) { traceStep--; renderTrace(); }
    });
    document.getElementById("ch06-trace-next")?.addEventListener("click", () => {
      const count = result && result.trace ? result.trace.length : 0;
      if (traceStep + 1 < count) { traceStep++; renderTrace(); }
    });
    document.getElementById("ch06-pf-run")?.addEventListener("click", runPfUi);
    document.getElementById("ch06-pf-seed")?.addEventListener("keydown", (event) => {
      if (event.key === "Enter") { event.preventDefault(); runPfUi(); }
    });
    document.getElementById("ch06-benchmark-btn")?.addEventListener("click", runBenchmark);
    document.getElementById("ch06-source-bnb")?.addEventListener("click", () => {
      activeSourceFile = MATCHERS_FILE;
      sourceStage = 0;
      renderSource();
    });
    document.getElementById("ch06-source-pf")?.addEventListener("click", () => {
      activeSourceFile = PF_FILE;
      sourceStage = 0;
      renderSource();
    });
    document.getElementById("ch06-stage-prev")?.addEventListener("click", () => {
      if (sourceStage > 0) { sourceStage--; renderSource(); }
    });
    document.getElementById("ch06-stage-next")?.addEventListener("click", () => {
      const stages = SOURCE_STAGES[activeSourceFile];
      if (sourceStage + 1 < stages.length) { sourceStage++; renderSource(); }
    });
    document.getElementById("ch06-stage-copy")?.addEventListener("click", () => {
      const code = document.getElementById("ch06-stage-code");
      if (code && navigator.clipboard) navigator.clipboard.writeText(code.textContent).catch(() => {});
      const button = document.getElementById("ch06-stage-copy");
      if (!button) return;
      const old = button.textContent;
      button.textContent = "已复制";
      window.setTimeout(() => { button.textContent = old; }, 1200);
    });
  }

  function init() {
    if (window.PAGE_ID !== "ch06" || !window.SlocSim) return;
    S = window.SlocSim;
    if (!document.getElementById("ch06-stage-code") || !document.getElementById("ch06-candidates-body")) return;
    buildScenario();
    bindControls();
    bindExercises();
    renderCandidates();
    runPfUi();
    renderSource();
  }

  window.addEventListener("sloc:ready", init);
})();
