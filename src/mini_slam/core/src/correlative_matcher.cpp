#include "mini_slam/correlative_matcher.hpp"

#include <cmath>
#include <vector>

namespace mini_slam {
namespace {

// 单个候选位姿的得分：点云按候选角旋转后平移查表求平均。
// rotated 已经按 (prior.theta + da) 旋转，平移在评分时再叠加，
// 这样同一个角度下所有平移候选共享一份旋转结果。
// 关键细节：分母只数"落在场内"的点 —— 查询扫描射程往往远超参考场的
// 覆盖范围，出界的点不构成任何证据，不该稀释 response（与 karto 一致）。
struct ScoreResult {
  double sum{0.0};
  int n_in{0};  // 在场内的点数
};

ScoreResult score_pose(const LikelihoodField& field, double px, double py,
                       std::span<const Vec2> rotated) {
  ScoreResult out;
  const Grid& g = field.grid();
  for (const Vec2& p : rotated) {
    int cx = 0, cy = 0;
    if (g.world_to_cell(Vec2{px + p.x, py + p.y}, cx, cy)) {
      out.sum += g.at(cx, cy);
      ++out.n_in;
    }
  }
  return out;
}

double normalized(const ScoreResult& s, bool in_bounds_only, size_t n_total) {
  if (in_bounds_only) {
    return s.n_in > 0 ? s.sum / static_cast<double>(s.n_in) : 0.0;
  }
  // 总点数归一：出界点贡献 0，天然惩罚"大部分视野不在候选区"的假匹配
  return static_cast<double>(n_total) > 0
             ? s.sum / static_cast<double>(n_total)
             : 0.0;
}

}  // namespace

MatchResult CorrelativeMatcher::match(const LikelihoodField& field,
                                      const Pose2& prior,
                                      std::span<const Vec2> points,
                                      const MatchWindow& w) {
  MatchResult out;
  if (points.empty()) return out;
  // 防御性校验：步长/窗口非法会让下面的 floor(W/step) 除零。
  if (!std::isfinite(w.lin) || !std::isfinite(w.ang) || w.lin < 0.0 ||
      w.ang < 0.0 || !std::isfinite(w.lin_step) ||
      !std::isfinite(w.ang_step) || w.lin_step <= 0.0 || w.ang_step <= 0.0) {
    return out;
  }

  // 窗口枚举：±W 内以 step 走，每侧步数 = floor(W/step)。
  // （这里曾经写成 floor(2W/step)，实际搜索范围变成 ±2W——假闭环的元凶）
  const int n_ang = static_cast<int>(std::floor(w.ang / w.ang_step));
  const int n_lin = static_cast<int>(std::floor(w.lin / w.lin_step));
  if (n_ang < 0 || n_lin < 0) return out;

  std::vector<Vec2> rotated(points.size());
  const size_t n_total = points.size();

  // ---------- 1) 粗搜索：全窗口 ----------
  double best_coarse = -1.0;
  double best_dx = 0.0, best_dy = 0.0, best_da = 0.0;
  struct CoarseCand {
    double dx, dy, da, score;
  };
  std::vector<CoarseCand> coarse;
  coarse.reserve(static_cast<size_t>(2 * n_ang + 1) * (2 * n_lin + 1) * 2);
  for (int ia = -n_ang; ia <= n_ang; ++ia) {
    const double da = ia * w.ang_step;
    const double th = prior.theta + da;
    const double c = std::cos(th), s = std::sin(th);
    for (size_t i = 0; i < points.size(); ++i) {
      rotated[i] = {c * points[i].x - s * points[i].y,
                    s * points[i].x + c * points[i].y};
    }
    const double py0 = prior.y;
    for (int ix = -n_lin; ix <= n_lin; ++ix) {
      const double px = prior.x + ix * w.lin_step;
      for (int iy = -n_lin; iy <= n_lin; ++iy) {
        const ScoreResult sc = score_pose(field, px, py0 + iy * w.lin_step, rotated);
        const double v = normalized(sc, w.in_bounds_only, n_total);
        coarse.push_back({ix * w.lin_step, iy * w.lin_step, da, v});
        if (v > best_coarse) {
          best_coarse = v;
          best_dx = ix * w.lin_step;
          best_dy = iy * w.lin_step;
          best_da = da;
        }
      }
    }
  }
  if (best_coarse < 0.0) return out;
  out.coarse_response = best_coarse;

  // 响应面位置方差 + 3×3 协方差：以 (响应/峰值)² 加权的候选分布二阶矩。
  // 单峰响应 → 方差小；平摊/多峰（滑移歧义）→ 方差大。
  // 方差用于 karto 式门控；协方差作为边信息矩阵的物理依据（阶段3）。
  {
    double wsum = 0.0;
    std::array<double, 3> m1{0.0, 0.0, 0.0};                 // Σ w·o
    std::array<double, 6> m2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Σ w·o·oᵀ (xx xy yy xt yt tt)
    for (const auto& cand : coarse) {
      const double wt = std::pow(cand.score / best_coarse, 2.0);
      if (wt < 1e-6) continue;
      const std::array<double, 3> o{cand.dx - best_dx, cand.dy - best_dy,
                                    cand.da - best_da};
      wsum += wt;
      for (int k = 0; k < 3; ++k) m1[k] += wt * o[k];
      m2[0] += wt * o[0] * o[0];
      m2[1] += wt * o[0] * o[1];
      m2[2] += wt * o[0] * o[2];
      m2[3] += wt * o[1] * o[1];
      m2[4] += wt * o[1] * o[2];
      m2[5] += wt * o[2] * o[2];
    }
    out.variance = wsum > 0 ? m2[0] / wsum + m2[3] / wsum : 0.0;
    if (wsum > 1e-9) {
      const std::array<double, 3> mean{m1[0] / wsum, m1[1] / wsum, m1[2] / wsum};
      const std::array<double, 6> cov{m2[0] / wsum - mean[0] * mean[0],
                                      m2[1] / wsum - mean[0] * mean[1],
                                      m2[2] / wsum - mean[0] * mean[2],
                                      m2[3] / wsum - mean[1] * mean[1],
                                      m2[4] / wsum - mean[1] * mean[2],
                                      m2[5] / wsum - mean[2] * mean[2]};
      out.covariance = {cov[0], cov[1], cov[2],
                        cov[1], cov[3], cov[4],
                        cov[2], cov[4], cov[5]};
    } else {
      out.covariance = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
    }
  }

  // ---------- 2) 细搜索：粗最优 ± 一个粗步长，平移步长 ÷4、角度步长 ÷8 ----------
  // 角度步长要够细：长走廊里航向观测本来就弱，0.7° 级的量化噪声
  // 每帧累积起来会把整条走廊画成弧
  const double fine_lin = w.lin_step * 0.25;
  const double fine_ang = w.ang_step * 0.125;
  const int nf_ang = std::max(1, static_cast<int>(std::lround(w.ang_step / fine_ang)));
  const int nf_lin = std::max(1, static_cast<int>(std::lround(w.lin_step / fine_lin)));

  double best_fine = -1.0;
  int fine_n_in = 0;
  double fdx = 0.0, fdy = 0.0, fda = 0.0;
  for (int ia = -nf_ang; ia <= nf_ang; ++ia) {
    const double da = best_da + ia * fine_ang;
    const double th = prior.theta + da;
    const double c = std::cos(th), s = std::sin(th);
    for (size_t i = 0; i < points.size(); ++i) {
      rotated[i] = {c * points[i].x - s * points[i].y,
                    s * points[i].x + c * points[i].y};
    }
    for (int ix = -nf_lin; ix <= nf_lin; ++ix) {
      const double px = prior.x + best_dx + ix * fine_lin;
      for (int iy = -nf_lin; iy <= nf_lin; ++iy) {
        const ScoreResult sc = score_pose(
            field, px, prior.y + best_dy + iy * fine_lin, rotated);
        if (const double v = normalized(sc, w.in_bounds_only, n_total); v > best_fine) {
          best_fine = v;
          fine_n_in = sc.n_in;
          fdx = best_dx + ix * fine_lin;
          fdy = best_dy + iy * fine_lin;
          fda = da;
        }
      }
    }
  }

  out.response = std::max(best_fine, best_coarse);
  out.pose = Pose2{prior.x + fdx, prior.y + fdy, prior.theta + fda};
  // 场内证据太少的候选不可信（比如扫描大部分落在参考场覆盖之外）
  out.valid = out.response > 0.0 && fine_n_in >= 0.3 * static_cast<int>(n_total);
  return out;
}

}  // namespace mini_slam
