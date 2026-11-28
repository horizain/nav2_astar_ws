// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// 匹配器实现
// ============================================================================
#include "scan_localizer/matchers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace sloc
{

// ---------- 公共工具 ----------
namespace
{

inline bool pose_is_finite(const Pose2 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}

inline bool pt_is_finite(const Vec2 & pt)
{
  return std::isfinite(pt.x()) && std::isfinite(pt.y());
}

// 平均似然 + 内点率
inline void eval_stats(
  const MapField & f, const Pose2 & p, const Scan & scan,
  double inlier_gate, double & score,
  double & inlier_ratio)
{
  if (!f.valid() || !pose_is_finite(p)) {
    score = 0.0;
    inlier_ratio = 0.0;
    return;
  }
  const auto w = transform_scan(p, scan);
  double s = 0.0, inl = 0.0;
  int n = 0;
  for (const auto & q : w) {
    if (!pt_is_finite(q)) {continue;}
    const auto g = f.grid().worldToGridF(q.x(), q.y());
    s += f.likelihoodAtF(g.x(), g.y());
    inl += (std::abs(f.sdf(g.x(), g.y())) < inlier_gate) ? 1.0 : 0.0;
    ++n;
  }
  score = n > 0 ? s / n : 0.0;
  inlier_ratio = n > 0 ? inl / n : 0.0;
}

// 计算每束激光点的弧长独立观测权重 ν_i ∈ [0.02, 1]、扫描局部轮廓法向与均方根旋转力臂 L_arm (m)
// 消除等角采样近墙密度偏置，并消除斜向走廊栅格阶梯锯齿伪梯度（任意旋转角下协方差严格各向异性）
void compute_scan_info_weights(
  const Scan & scan, double corr_len,
  std::vector<double> & weights, double & rms_arm,
  std::vector<Vec2> * contour_normals = nullptr)
{
  const std::size_t n = scan.points.size();
  weights.assign(n, 0.0);
  if (contour_normals) {
    contour_normals->assign(n, Vec2::Zero());
  }
  const double lc = std::max(0.10, corr_len);
  double r2_sum = 0.0;
  double w_sum = 0.0;
  int valid_cnt = 0;
  bool has_prev = false;
  Vec2 prev = Vec2::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    const auto & pt = scan.points[i];
    if (!pt_is_finite(pt)) {continue;}
    r2_sum += pt.squaredNorm();
    ++valid_cnt;
    const double ds = has_prev ? std::min((pt - prev).norm(), lc) : lc;
    const double w = std::clamp(ds / lc, 0.02, 1.0);
    weights[i] = w;
    w_sum += w;
    prev = pt;
    has_prev = true;
  }
  if (contour_normals && n >= 5) {
    for (std::size_t i = 0; i < n; ++i) {
      if (!pt_is_finite(scan.points[i])) {continue;}
      const double max_chord = std::max(0.60, 0.45 * scan.points[i].norm());
      // 在前后 ±3 束内寻找共线连续墙面弦向量（兼容远距掠射侧墙，平滑 5cm 栅格阶梯锯齿）
      int lo = static_cast<int>(i);
      int hi = static_cast<int>(i);
      for (int d = 1; d <= 3; ++d) {
        const int il = static_cast<int>(i) - d;
        if (il >= 0 && pt_is_finite(scan.points[static_cast<std::size_t>(il)]) &&
          (scan.points[static_cast<std::size_t>(il)] - scan.points[i]).norm() < max_chord)
        {
          lo = il;
        }
        const int ir = static_cast<int>(i) + d;
        if (ir < static_cast<int>(n) &&
          pt_is_finite(scan.points[static_cast<std::size_t>(ir)]) &&
          (scan.points[static_cast<std::size_t>(ir)] - scan.points[i]).norm() < max_chord)
        {
          hi = ir;
        }
      }
      if (hi > lo) {
        const Vec2 & plo = scan.points[static_cast<std::size_t>(lo)];
        const Vec2 & phi = scan.points[static_cast<std::size_t>(hi)];
        const Vec2 chord = phi - plo;
        const double cn = chord.norm();
        if (cn > 0.04) {
          const Vec2 rel = scan.points[i] - plo;
          const double line_dist =
            std::abs(rel.x() * chord.y() - rel.y() * chord.x()) / cn;
          if (lo == static_cast<int>(i) || hi == static_cast<int>(i) || line_dist < 0.08) {
            (*contour_normals)[i] = Vec2(-chord.y() / cn, chord.x() / cn);
          }
        }
      }
    }
  }
  rms_arm = (valid_cnt > 0) ?
    std::max(0.5, std::sqrt(r2_sum / static_cast<double>(valid_cnt))) :
    2.0;
  if (valid_cnt >= 5 && w_sum > 1e-9 && w_sum < 5.0) {
    const double boost = 5.0 / w_sum;
    for (double & w : weights) {
      w *= boost;
    }
  }
}

// 量纲一致的可观子空间海森求逆协方差（无需 cond > 6 经验补丁）：
// 通过均方根力臂 L_arm 将 [x, y, θ] 映射至同量纲米制空间，自动压制栅格锯齿伪刚度并膨胀退化轴方差
Cov3 cov_from_hessian(
  const Eigen::Matrix3d & H, double res_var,
  double rms_arm = 2.0,
  double min_var_xy = 1e-4,                       // (1cm)²
  double min_var_th = 1.2e-5,                     // (0.2°)²
  double inflation = 2.0)
{
  const Eigen::Matrix3d Hsym = 0.5 * (H + H.transpose());
  const double inv_arm = 1.0 / std::max(0.5, rms_arm);
  const Eigen::Vector3d d_vec(1.0, 1.0, inv_arm);
  const Eigen::Matrix3d Hm = d_vec.asDiagonal() * Hsym * d_vec.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(0.5 * (Hm + Hm.transpose()));
  if (es.info() != Eigen::Success) {
    Cov3 fallback = Cov3::Zero();
    fallback(0, 0) = 1.0;
    fallback(1, 1) = 1.0;
    fallback(2, 2) = 1.0;
    return fallback;
  }
  constexpr double kMinEig = 1e-4;
  constexpr double kObsSupport = 1.0;  // 至少约 1 个独立墙面段支撑才视为真实几何约束
  Eigen::Vector3d inv_eigs;
  for (int k = 0; k < 3; ++k) {
    const double lam = std::max(0.0, es.eigenvalues()(k));
    const double obs_gate = std::clamp(lam / kObsSupport, 0.0, 1.0);
    const double eff_lam = std::max(lam * obs_gate * obs_gate, kMinEig);
    inv_eigs(k) = 1.0 / eff_lam;
  }
  const Eigen::Matrix3d Hm_inv =
    es.eigenvectors() * inv_eigs.asDiagonal() * es.eigenvectors().transpose();
  const Eigen::Matrix3d H_inv =
    d_vec.asDiagonal() * Hm_inv * d_vec.asDiagonal();
  const double scale = std::max(res_var, min_var_xy) * inflation;
  Cov3 cov = scale * H_inv;
  cov = 0.5 * (cov + cov.transpose()).eval();

  cov(0, 0) = std::max(cov(0, 0), min_var_xy);
  cov(1, 1) = std::max(cov(1, 1), min_var_xy);
  cov(2, 2) = std::max(cov(2, 2), min_var_th);
  return cov;
}

}  // namespace

// ============================================================================
// CorrelativeMatcher —— Olson 窗口穷举（粗+细两级），协方差取响应面二阶矩
// ============================================================================
MatchResult CorrelativeMatcher::match(
  const MapField & field, const Pose2 & prior,
  const Scan & scan,
  const MatchOptions & opt) const
{
  MatchResult res;
  if (!field.valid() || !pose_is_finite(prior) || scan.points.empty()) {
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
  const double max_gy = static_cast<double>(grid->height - 1);

  const double prior_th = wrap_angle(prior.theta);
  const double prior_sig_lin2 = std::max(0.04, 4.0 * opt.window_lin * opt.window_lin);
  const double prior_sig_ang2 = std::max(0.01, 4.0 * opt.window_ang * opt.window_ang);

  const int max_rounds = opt.correlative_two_stage ? 2 : 1;
  for (int round = 0; round < max_rounds; ++round) {
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
    const double cth = round == 0 ? prior_th : best_th;
    if (round == 0) {
      coarse_poses.reserve(
        static_cast<std::size_t>(n_a) *
        static_cast<std::size_t>(n_x) *
        static_cast<std::size_t>(n_y));
    }

    for (int ai = 0; ai < n_a; ++ai) {
      const double th =
        wrap_angle(cth - 0.5 * (n_a - 1) * ang_step + ai * ang_step);
      const double dth = wrap_angle(th - prior_th);
      const double c = std::cos(th), s = std::sin(th);
      cells.clear();
      for (const auto & p : valid_pts) {
        const double wx = prior.x + c * p.x() - s * p.y();
        const double wy = prior.y + s * p.x() + c * p.y();
        cells.push_back(grid->worldToGridF(wx, wy));
      }
      for (int yi = 0; yi < n_y; ++yi) {
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
          // 弱运动先验破平局（MAP tie-breaker）：非退化峰差 >0.20 不受影响，走廊平底谷锚定在先验处
          const double map_sc = sc - 0.002 * (
            (dx * dx + dy * dy) / prior_sig_lin2 + (dth * dth) / prior_sig_ang2);
          if (map_sc > best_s) {
            best_s = map_sc;
            best_x = cand_x;
            best_y = cand_y;
            best_th = th;
          }
        }
      }
    }
  }

  res.pose = {best_x, best_y, wrap_angle(best_th)};
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
  }
  if (!cov_computed) {
    cov = Cov3::Zero();
  }
  // SPD 下界保护（同时避免退化时压垮奇异特征值比）
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  if (es.info() == Eigen::Success) {
    const Eigen::Vector3d eigs = es.eigenvalues().cwiseMax(1e-5);
    cov = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    cov = 0.5 * (cov + cov.transpose()).eval();
  }
  cov(0, 0) = std::max(cov(0, 0), 2.5e-5);
  cov(1, 1) = std::max(cov(1, 1), 2.5e-5);
  cov(2, 2) = std::max(cov(2, 2), 1e-5);
  res.covariance = cov;

  eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
    res.inlier_ratio);
  res.valid = best_s > 0.0;
  return res;
}

// ============================================================================
// FieldGnMatcher —— SDF 连续场 GN（点-线 ICP 等价，零数据关联）
// ============================================================================
MatchResult FieldGnMatcher::match(
  const MapField & field, const Pose2 & prior,
  const Scan & scan,
  const MatchOptions & opt) const
{
  MatchResult res;
  if (!field.valid() || !pose_is_finite(prior)) {return res;}
  const auto * grid = &field.grid();
  const int n = static_cast<int>(scan.points.size());
  if (n < 5) {return res;}

  const double corr_len = std::max(2.0 * grid->resolution, field.sigma_m_);
  std::vector<double> arc_w;
  std::vector<Vec2> contour_n;
  double rms_arm = 2.0;
  compute_scan_info_weights(scan, corr_len, arc_w, rms_arm, &contour_n);

  struct PtObs
  {
    Eigen::Vector3d J_sdf;
    Eigen::Vector3d J_geom;
    double r;
    double nu_w;
    int bin;
  };
  std::vector<PtObs> obs_buf;
  obs_buf.reserve(scan.points.size());

  auto eval_pose = [&](
    const Pose2 & pose, Eigen::Matrix3d & H, Eigen::Matrix3d & H_info,
    Eigen::Vector3d & b, double & ravg, int & used)
    {
      constexpr int kNormalBins = 8;
      constexpr double kMaxIndepPerAxis = 12.0;
      const double c = std::cos(pose.theta), s = std::sin(pose.theta);
      H.setZero();
      H_info.setZero();
      b.setZero();
      obs_buf.clear();
      std::array<double, kNormalBins> bin_sum{};
      used = 0;

      for (std::size_t i = 0; i < scan.points.size(); ++i) {
        const auto & pt = scan.points[i];
        if (!pt_is_finite(pt) || arc_w[i] <= 0.0) {continue;}
        const double lx = pt.x(), ly = pt.y();
        const double wx = pose.x + c * lx - s * ly;
        const double wy = pose.y + s * lx + c * ly;
        const auto g = grid->worldToGridF(wx, wy);
        Eigen::Vector2d grad;
        const double r = field.sdf(g.x(), g.y(), &grad);
        if (std::abs(r) > opt.trim_dist) {continue;}
        const double ar = std::abs(r);
        const double w_huber = ar <= opt.huber_delta ?
          1.0 :
          opt.huber_delta / std::max(ar, 1e-9);

        // 若存在连续扫描轮廓法向，旋转至世界系消除栅格阶梯锯齿；孤立角点则回退 SDF 梯度
        Eigen::Vector2d n_world = grad;
        const Vec2 & cn = contour_n[i];
        if (cn.squaredNorm() > 0.5) {
          Eigen::Vector2d wn(c * cn.x() - s * cn.y(), s * cn.x() + c * cn.y());
          if (wn.dot(grad) < 0.0) {wn = -wn;}
          n_world = wn;
        }
        const double nnorm = n_world.norm();
        int bin = 0;
        if (nnorm > 1e-6) {
          double ang = std::atan2(n_world.y(), n_world.x());
          if (ang < 0.0) {ang += kPi;}
          bin = std::clamp(
            static_cast<int>(ang * (static_cast<double>(kNormalBins) / kPi)),
            0, kNormalBins - 1);
        }
        const double nu_w = arc_w[i] * w_huber;
        bin_sum[static_cast<std::size_t>(bin)] += nu_w;
        const double jth_sdf = grad.x() * (-(wy - pose.y)) +
          grad.y() * (wx - pose.x);
        const double jth_geom = n_world.x() * (-(wy - pose.y)) +
          n_world.y() * (wx - pose.x);
        obs_buf.push_back({
          Eigen::Vector3d(grad.x(), grad.y(), jth_sdf),
          Eigen::Vector3d(n_world.x(), n_world.y(), jth_geom),
          r, nu_w, bin});
        ++used;
      }

      std::array<double, kNormalBins> alpha{};
      for (int bi = 0; bi < kNormalBins; ++bi) {
        const double bs = bin_sum[static_cast<std::size_t>(bi)];
        alpha[static_cast<std::size_t>(bi)] =
          (bs > kMaxIndepPerAxis) ? (kMaxIndepPerAxis / bs) : 1.0;
      }

      double rsum = 0.0, rw = 0.0;
      for (const auto & ob : obs_buf) {
        const double w = ob.nu_w * alpha[static_cast<std::size_t>(ob.bin)];
        H += w * ob.J_sdf * ob.J_sdf.transpose();
        H_info += w * ob.J_geom * ob.J_geom.transpose();
        b += w * ob.J_sdf * ob.r;
        rsum += w * ob.r * ob.r;
        rw += w;
      }
      ravg = rw > 0.0 ? rsum / rw : std::numeric_limits<double>::infinity();
    };

  Pose2 p(prior.x, prior.y, wrap_angle(prior.theta));
  bool has_valid_iter = false;
  double best_ravg = std::numeric_limits<double>::infinity();
  Pose2 best_pose = p;
  Eigen::Matrix3d best_H = Eigen::Matrix3d::Zero();
  double best_var = 1e-6;
  const double inv_arm = 1.0 / std::max(0.5, rms_arm);
  const Eigen::Vector3d d_vec(1.0, 1.0, inv_arm);

  for (int it = 0; it < opt.max_iterations; ++it) {
    Eigen::Matrix3d H;
    Eigen::Matrix3d H_info;
    Eigen::Vector3d b;
    double ravg = 0.0;
    int used = 0;
    eval_pose(p, H, H_info, b, ravg, used);
    if (used < 5) {break;}

      // 应用步长前，先用当前位姿 p 与其自身残差 ravg 更新历史最优
    if (!has_valid_iter || ravg < best_ravg) {
      has_valid_iter = true;
      best_ravg = ravg;
      best_pose = p;
      best_H = H_info;
      best_var = std::max(ravg, 1e-6);
    }

      // 量纲一致可观子空间解映射（由无锯齿轮廓信息矩阵 H_info 判定可观子空间，在 H 上解步）
    const Eigen::Matrix3d Hm_info = d_vec.asDiagonal() * H_info * d_vec.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_info(
      0.5 * (Hm_info + Hm_info.transpose()));
    if (es_info.info() != Eigen::Success) {break;}

    Eigen::Matrix3d Hd = H;
    Hd.diagonal().array() += 1e-5 * H.diagonal().array().max(1.0);
    const Eigen::Vector3d dp_raw = Hd.ldlt().solve(-b);
    if (!dp_raw.allFinite()) {break;}

    const Eigen::Vector3d dp_raw_m = d_vec.cwiseInverse().cwiseProduct(dp_raw);
    Eigen::Vector3d dp_m = Eigen::Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
      const double lam = std::max(0.0, es_info.eigenvalues()(k));
      const double eta = std::clamp((lam - 0.35) / 0.85, 0.0, 1.0);
      if (eta <= 0.0) {continue;}
      const Eigen::Vector3d vk = es_info.eigenvectors().col(k);
      dp_m += eta * vk.dot(dp_raw_m) * vk;
    }
    const Eigen::Vector3d dp = d_vec.cwiseProduct(dp_m);
    if (!dp.allFinite()) {break;}
      // 步长护栏：单次迭代 ≤0.2m / 0.1rad
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
  }

  if (!has_valid_iter) {
    res.valid = false;
    return res;
  }

    // 评估循环结束后的最终位姿 p，若残差更优则采纳
  {
    Eigen::Matrix3d H_end;
    Eigen::Matrix3d H_info_end;
    Eigen::Vector3d b_end;
    double ravg_end = 0.0;
    int used_end = 0;
    eval_pose(p, H_end, H_info_end, b_end, ravg_end, used_end);
    if (used_end >= 5 && ravg_end < best_ravg) {
      best_ravg = ravg_end;
      best_pose = p;
      best_H = H_info_end;
      best_var = std::max(ravg_end, 1e-6);
    }
  }

  res.pose = best_pose;
  // H 已由弧长独立权重与法向空间分桶归一化至真实独立观测尺度，叠加半胞元几何离散方差底
  const double quant_var = 0.25 * grid->resolution * grid->resolution;
  res.covariance = cov_from_hessian(
    best_H, std::max(best_var, quant_var), rms_arm);
  eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
    res.inlier_ratio);
  res.valid = true;
  return res;
}

// ============================================================================
// IcpP2pMatcher —— 经典点到点 ICP（SDF 下降找对应）
// ============================================================================
MatchResult IcpP2pMatcher::match(
  const MapField & field, const Pose2 & prior,
  const Scan & scan,
  const MatchOptions & opt) const
{
  MatchResult res;
  if (!field.valid() || !pose_is_finite(prior)) {return res;}
  const auto * grid = &field.grid();
  const int n = static_cast<int>(scan.points.size());
  if (n < 5) {return res;}

  Pose2 p(prior.x, prior.y, wrap_angle(prior.theta));
  bool has_valid_iter = false;
  Eigen::Matrix3d last_H = Eigen::Matrix3d::Zero();
  double last_var = 1e-6;

  for (int it = 0; it < opt.max_iterations; ++it) {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    int used = 0;
    double rsum = 0.0, rw = 0.0;
    for (const auto & pt : scan.points) {
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
    }
    if (used < 5) {break;}
    has_valid_iter = true;
    last_H = H;
    last_var = std::max(rsum / std::max(rw, 1e-9), 1e-6);

    Eigen::Matrix3d Hd = H;
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
  }

  if (!has_valid_iter) {
    res.valid = false;
    return res;
  }

    // 基于最终位姿处的法向均衡信息矩阵计算几何协方差（如实反映走廊等滑移退化）
  const double corr_len = std::max(2.0 * grid->resolution, field.sigma_m_);
  std::vector<double> arc_w;
  double rms_arm = 2.0;
  compute_scan_info_weights(scan, corr_len, arc_w, rms_arm);
  Eigen::Matrix3d H_geom = Eigen::Matrix3d::Zero();
  {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    int used_g = 0;
    for (std::size_t i = 0; i < scan.points.size(); ++i) {
      const auto & pt = scan.points[i];
      if (!pt_is_finite(pt) || arc_w[i] <= 0.0) {continue;}
      const double wx = p.x + c * pt.x() - s * pt.y();
      const double wy = p.y + s * pt.x() + c * pt.y();
      const auto g = grid->worldToGridF(wx, wy);
      Eigen::Vector2d grad;
      const double r = field.sdf(g.x(), g.y(), &grad);
      if (std::abs(r) > opt.trim_dist) {continue;}
      const double jth = grad.x() * (-(wy - p.y)) + grad.y() * (wx - p.x);
      const Eigen::Vector3d Jg(grad.x(), grad.y(), jth);
      H_geom += arc_w[i] * Jg * Jg.transpose();
      ++used_g;
    }
    if (used_g < 5) {
      H_geom = last_H;
    }
  }

  res.pose = p;
  const double quant_var = 0.25 * grid->resolution * grid->resolution;
  res.covariance = cov_from_hessian(H_geom, std::max(last_var, quant_var), rms_arm);
  eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
    res.inlier_ratio);
  res.valid = true;
  return res;
}

// ============================================================================
// NdtMatcher —— 2D NDT（体素高斯，D2P 分数最大化）
// ============================================================================
bool NdtMatcher::build(const OccupancyGrid & grid, double voxel_m)
{
  gauss_.clear();
  voxel_gauss_idx_.clear();
  voxel_w_ = 0;
  voxel_h_ = 0;
  if (grid.width <= 0 || grid.height <= 0 ||
    !std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
    !std::isfinite(voxel_m) || voxel_m <= 0.0 ||
    grid.cells.size() <
    static_cast<std::size_t>(grid.width) *
    static_cast<std::size_t>(grid.height))
  {
    return false;
  }

  voxel_m_ = voxel_m;
  origin_x_ = grid.origin_x;
  origin_y_ = grid.origin_y;
  const int vw = static_cast<int>(
    std::max(1.0, std::ceil(grid.width * grid.resolution / voxel_m)));
  const int vh = static_cast<int>(
    std::max(1.0, std::ceil(grid.height * grid.resolution / voxel_m)));
  voxel_w_ = vw;
  voxel_h_ = vh;

  std::vector<std::vector<Eigen::Vector2d>> buckets(
    static_cast<std::size_t>(vw) * static_cast<std::size_t>(vh));
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (!grid.occupied(x, y)) {continue;}
      const auto w = grid.gridToWorld(x, y);
      const int vx = std::clamp(
        static_cast<int>(std::floor((w.x() - origin_x_) / voxel_m_)),
        0, vw - 1);
      const int vy = std::clamp(
        static_cast<int>(std::floor((w.y() - origin_y_) / voxel_m_)),
        0, vh - 1);
      buckets[static_cast<std::size_t>(vy) * static_cast<std::size_t>(vw) +
        static_cast<std::size_t>(vx)].push_back(w);
    }
  }
  voxel_gauss_idx_.assign(buckets.size(), -1);
  for (std::size_t bi = 0; bi < buckets.size(); ++bi) {
    const auto & b = buckets[bi];
    if (b.size() < 3) {continue;}
    Gauss g;
    g.mu = Eigen::Vector2d::Zero();
    for (const auto & q : b) {
      g.mu += q;
    }
    g.mu /= static_cast<double>(b.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto & q : b) {
      const Eigen::Vector2d d = q - g.mu;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(b.size() - 1);
      // 正则化：保证正定（NDT 惯例 λ=0.001·voxel²）
    cov.diagonal().array() += 0.001 * voxel_m_ * voxel_m_;
    g.cov = cov;
    g.cov_inv = cov.inverse();
    voxel_gauss_idx_[bi] = static_cast<int>(gauss_.size());
    gauss_.push_back(g);
  }
  return !gauss_.empty();
}

MatchResult NdtMatcher::match(
  const MapField & field, const Pose2 & prior,
  const Scan & scan,
  const MatchOptions & opt) const
{
  MatchResult res;
  if (!field.valid() || !pose_is_finite(prior) || gauss_.empty() ||
    voxel_w_ <= 0 || voxel_h_ <= 0 || voxel_m_ <= 0.0 ||
    scan.points.size() < 5)
  {
    return res;
  }
  const double cell_r = 2.0 * voxel_m_;    // 对应搜索半径

  Pose2 p(prior.x, prior.y, wrap_angle(prior.theta));
  bool has_valid_iter = false;
  Eigen::Matrix3d last_H = Eigen::Matrix3d::Zero();
  int last_used = 0;

  for (int it = 0; it < opt.max_iterations; ++it) {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g3 = Eigen::Vector3d::Zero();
    int used = 0;
    for (const auto & pt : scan.points) {
      if (!pt_is_finite(pt)) {continue;}
      const double wx = p.x + c * pt.x() - s * pt.y();
      const double wy = p.y + s * pt.x() + c * pt.y();
      if (!std::isfinite(wx) || !std::isfinite(wy)) {continue;}
      const double fx = std::floor((wx - origin_x_) / voxel_m_);
      const double fy = std::floor((wy - origin_y_) / voxel_m_);
      if (fx < -1.0 || fx > static_cast<double>(voxel_w_) ||
        fy < -1.0 || fy > static_cast<double>(voxel_h_))
      {
        continue;
      }
      const int vx = static_cast<int>(fx);
      const int vy = static_cast<int>(fy);
        // 完整遍历 3x3 邻域，挑选马氏距离最小的高斯体素
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
      const double sc = std::exp(-0.5 * best_m2);
        // ∂s/∂p = -s · Jᵀ C⁻¹ d （J = ∂d/∂p, 2x3）
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
    if (!dp.allFinite()) {break;}
      // 步长护栏：单步不超过 0.5m / 0.2rad（防远场高斯误导时发散）
    const double nl = dp.head<2>().norm();
    const double s_clip =
      std::min(1.0, std::min(0.5 / std::max(nl, 1e-9),
        0.2 / std::max(std::abs(dp(2)), 1e-9)));
    p.x += s_clip * dp(0);
    p.y += s_clip * dp(1);
    p.theta = wrap_angle(p.theta + s_clip * dp(2));
    if (nl * s_clip < opt.converg_t &&
      std::abs(dp(2)) * s_clip < opt.converg_r)
    {
      break;
    }
  }

  if (!has_valid_iter) {
    res.valid = false;
    return res;
  }

  res.pose = p;
    // NDT 的 H 已含 C⁻¹（单位 m⁻²）；按有效点数归一化后求逆，防止 N 倍过自信并保留退化比
  const Eigen::Matrix3d H_norm = last_H / static_cast<double>(std::max(1, last_used));
  res.covariance = cov_from_hessian(H_norm, 1.0);
  eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
    res.inlier_ratio);
  res.valid = true;
  return res;
}

// ============================================================================
// ComposedMatcher —— 粗捕获 → 精修
// ============================================================================
MatchResult ComposedMatcher::match(
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

    // 可观子空间回投：若精修海森协方差表明某平移主轴严重退化（如走廊纵向），
    // 将粗搜在该不可观方向上的残余漂移回投至运动先验，保证走廊纵向平滑无乱跳
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es_xy(
    0.5 * (fine.covariance.topLeftCorner<2, 2>() +
    fine.covariance.topLeftCorner<2, 2>().transpose()));
  if (es_xy.info() == Eigen::Success) {
    Eigen::Vector2d dxy(fine.pose.x - prior.x, fine.pose.y - prior.y);
    bool remapped = false;
    for (int k = 0; k < 2; ++k) {
      const double eig_var = es_xy.eigenvalues()(k);
      if (eig_var > 0.20) {
        const double blend = std::clamp((eig_var - 0.20) / 0.80, 0.0, 1.0);
        const Eigen::Vector2d vk = es_xy.eigenvectors().col(k);
        dxy -= blend * dxy.dot(vk) * vk;
        remapped = true;
      }
    }
    if (remapped) {
      Pose2 proj_pose(prior.x + dxy.x(), prior.y + dxy.y(), fine.pose.theta);
      double proj_sc = 0.0, proj_inl = 0.0;
      eval_stats(field, proj_pose, scan, opt.inlier_gate, proj_sc, proj_inl);
      if (proj_sc + 0.02 >= fine.score) {
        fine.pose = proj_pose;
        fine.score = proj_sc;
        fine.inlier_ratio = proj_inl;
      }
    }
  }
  return fine;
}

// 工厂
std::shared_ptr<Matcher> make_matcher(
  const std::string & name,
  const OccupancyGrid * grid,
  double ndt_voxel)
{
  if (name == "correlative") {
    return std::make_shared<CorrelativeMatcher>();
  }
  if (name == "field_gn") {return std::make_shared<FieldGnMatcher>();}
  if (name == "icp_p2p") {return std::make_shared<IcpP2pMatcher>();}
  if (name == "ndt") {
    if (grid == nullptr) {return nullptr;}
    auto m = std::make_shared<NdtMatcher>();
    if (!m->build(*grid, ndt_voxel)) {return nullptr;}
    return m;
  }
  if (name == "composed") {
    return std::make_shared<ComposedMatcher>(
      std::make_shared<CorrelativeMatcher>(),
      std::make_shared<FieldGnMatcher>());
  }
  return nullptr;
}

}  // namespace sloc
