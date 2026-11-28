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
#include <chrono>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
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

// ============================================================================
// GlobalCorrelative —— 分支定界全局搜索（Cartographer MatchFullSubmap 思路）
//
// · 金字塔层存滑窗最大值 swmax_ → 对偏移块 [b, b+2^lv) 内任意离散位姿
//   提供紧致可采纳上界（越界光束按 0 分处理而非剔除整个位姿）；
// · 优先队列按上界出队，弹出 level-0 节点即精确得分；
// · 固定容量 top-K + 空间/角度 NMS 维护剪枝下界 worst_kept；
// · 末端小窗口细修（±2 格 / ±半角步长），产出交给跟踪匹配器。
// ============================================================================
// 内部全命中无分支 uint8 似然累加 + 部分和上界早停（统一 4 路标量展开：
// ARM64 Cortex-A72 实测 NEON 缺乏硬件 gather，手写 8 路打包反而慢 ~1.26%）
static inline std::uint32_t eval_u8_inbounds_early_exit(
  const std::uint8_t * base, const std::int32_t * offs,
  int n, std::uint32_t min_sum_u8)
{
  std::uint32_t sum = 0u;
  int i = 0;
  for (; i + 4 <= n; i += 4) {
    sum += static_cast<std::uint32_t>(base[offs[i]]) +
      static_cast<std::uint32_t>(base[offs[i + 1]]) +
      static_cast<std::uint32_t>(base[offs[i + 2]]) +
      static_cast<std::uint32_t>(base[offs[i + 3]]);
    if (((i + 4) & 15) == 0 && min_sum_u8 > 0u) {
      if (sum + static_cast<std::uint32_t>(n - (i + 4)) * 255u <= min_sum_u8) {
        return 0u;
      }
    }
  }
  for (; i < n; ++i) {
    sum += static_cast<std::uint32_t>(base[offs[i]]);
  }
  return sum;
}

GlobalResult global_correlative(
  const MapField & field, const Scan & scan,
  int beams_subsample, double ang_step,
  int n_hypotheses, double max_time_ms)
{
  const auto t_start = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
    };
  const bool use_time_budget = std::isfinite(max_time_ms) && max_time_ms > 0.0;

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

    // 计算地图自由空间边界框与可通行面积作为机器人位姿搜索域（允许部分光束越界记 0 分）
  int free_xmin = W, free_xmax = -1, free_ymin = H, free_ymax = -1;
  std::size_t free_cells = 0;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int8_t c = grid->cells[static_cast<std::size_t>(y) *
          static_cast<std::size_t>(W) + static_cast<std::size_t>(x)];
      if (c >= 0 && c <= 65) {
        ++free_cells;
        free_xmin = std::min(free_xmin, x);
        free_xmax = std::max(free_xmax, x);
        free_ymin = std::min(free_ymin, y);
        free_ymax = std::max(free_ymax, y);
      }
    }
  }
  if (free_xmin > free_xmax || free_ymin > free_ymax) {
    return out;
  }

  const int n_ang = static_cast<int>(std::ceil(kTwoPi / ang_step));
  if (n_ang <= 0) {return out;}

    // 每角度：整数胞元 c_i + 1D 扁平偏移 + 范围包围盒（供内部全命中无分支快路径使用）
  struct AngData
  {
    std::vector<Eigen::Vector2i> cells;
    std::vector<std::int32_t> flat_offs;
    int min_cx{0}, max_cx{0}, min_cy{0}, max_cy{0};
    int dx_min{0}, dx_max{0}, dy_min{0}, dy_max{0};
  };
  std::vector<AngData> ang(static_cast<std::size_t>(n_ang));
  for (int a = 0; a < n_ang; ++a) {
    const double th = a * ang_step;
    const double cc = std::cos(th), ss = std::sin(th);
    auto & A = ang[static_cast<std::size_t>(a)];
    A.cells.reserve(sub_pts.size());
    A.flat_offs.reserve(sub_pts.size());
    bool first_pt = true;
    for (const auto & pt : sub_pts) {
      const int cx = static_cast<int>(std::lround(
          (cc * pt.x() - ss * pt.y()) / grid->resolution));
      const int cy = static_cast<int>(std::lround(
          (ss * pt.x() + cc * pt.y()) / grid->resolution));
      A.cells.push_back({cx, cy});
      A.flat_offs.push_back(static_cast<std::int32_t>(cy * W + cx));
      if (first_pt) {
        A.min_cx = A.max_cx = cx;
        A.min_cy = A.max_cy = cy;
        first_pt = false;
      } else {
        A.min_cx = std::min(A.min_cx, cx);
        A.max_cx = std::max(A.max_cx, cx);
        A.min_cy = std::min(A.min_cy, cy);
        A.max_cy = std::max(A.max_cy, cy);
      }
    }
    A.dx_min = free_xmin;
    A.dx_max = free_xmax;
    A.dy_min = free_ymin;
    A.dy_max = free_ymax;
  }

  const double inv_n255 = 1.0 / (255.0 * static_cast<double>(n));

    // 块上界打分：优先走 uint8 量化金字塔 + 内部无分支快路径 + 部分和早停
  auto eval_block_bound = [&](int level, int bx, int by, const AngData & A, double min_bound)
    {
      const std::uint8_t * plane = field.poolDataU8(level);
      const std::uint32_t min_sum_u8 = (min_bound > 0.0) ?
        static_cast<std::uint32_t>(std::floor(min_bound * 255.0 * static_cast<double>(n))) : 0u;
      if (plane != nullptr) {
        if (bx + A.min_cx >= 0 && bx + A.max_cx < W &&
          by + A.min_cy >= 0 && by + A.max_cy < H)
        {
          const std::uint8_t * base =
            plane + static_cast<std::size_t>(by) * static_cast<std::size_t>(W) +
            static_cast<std::size_t>(bx);
          const std::uint32_t sum = eval_u8_inbounds_early_exit(
            base, A.flat_offs.data(), n, min_sum_u8);
          return static_cast<double>(sum) * inv_n255;
        }
        const int span = 1 << level;
        std::uint32_t sum = 0u;
        for (int i = 0; i < n; ++i) {
          const auto & c = A.cells[static_cast<std::size_t>(i)];
          const int px = c.x() + bx;
          const int py = c.y() + by;
          if (px >= W || px + span <= 0 || py >= H || py + span <= 0) {
            if (min_sum_u8 > 0u &&
              sum + static_cast<std::uint32_t>(n - 1 - i) * 255u <= min_sum_u8)
            {
              return 0.0;
            }
            continue;
          }
          const int cx = std::max(px, 0);
          const int cy = std::max(py, 0);
          sum += static_cast<std::uint32_t>(
            plane[static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) +
            static_cast<std::size_t>(cx)]);
        }
        return static_cast<double>(sum) * inv_n255;
      }

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
  std::priority_queue<Node> pq;

    // 播种：顶层块（步长 2^K）覆盖自由空间边界框
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
          const double sc = eval_block_bound(K, bx, by, A, 0.05);
          if (sc > 0.05) {pq.push({sc, K, bx, by, a});}
        }
      }
    }
  }
  if (pq.empty()) {return out;}

    // NMS 判定阈值（用于区分多峰假设，防止同一峰相邻像素占满 top-K）
  constexpr double kNmsLinM = 0.50;
  constexpr double kNmsAngRad = 0.35;
    // 空间+航向分桶配额：单桶硬封顶 2，桶尺度随可通行面积、地图跨度与射程 p90 自适应放大，
    // 防止 200m 长走廊/大厂房下局部高分区域吞掉 top-K 全局配额并过早抬高剪枝下界。
  double r_p90 = 1.5;
  {
    std::vector<double> ranges;
    ranges.reserve(valid_pts.size());
    for (const auto & pt : valid_pts) {
      const double r = pt.norm();
      if (r > 1e-3) {ranges.push_back(r);}
    }
    if (!ranges.empty()) {
      const std::size_t idx90 = (ranges.size() * 9) / 10;
      std::nth_element(
        ranges.begin(),
        ranges.begin() + static_cast<std::ptrdiff_t>(idx90),
        ranges.end());
      r_p90 = std::max(1.5, ranges[idx90]);
    }
  }
  const double free_area_m2 =
    static_cast<double>(free_cells) * grid->resolution * grid->resolution;
  const double free_diag_m = std::hypot(
    static_cast<double>(free_xmax - free_xmin + 1) * grid->resolution,
    static_cast<double>(free_ymax - free_ymin + 1) * grid->resolution);
  const double denom_slots = static_cast<double>(std::max(4, topk));
  const double area_bucket = 0.6 * std::sqrt(std::max(0.0, free_area_m2) / denom_slots);
  const double span_bucket = 0.75 * free_diag_m / denom_slots;
  const double range_bucket = 0.35 * r_p90;
  const double bucket_size_m = std::clamp(
    std::max({2.50, area_bucket, span_bucket, range_bucket}), 2.50, 20.0);
  constexpr int kAngBuckets = 4;
  const int max_per_bucket = std::min(topk, 2);

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

  auto same_bucket = [&](const Hyp & a, const Hyp & b)
    {
      const int axb = static_cast<int>(std::floor(a.dx * grid->resolution / bucket_size_m));
      const int ayb = static_cast<int>(std::floor(a.dy * grid->resolution / bucket_size_m));
      const int bxb = static_cast<int>(std::floor(b.dx * grid->resolution / bucket_size_m));
      const int byb = static_cast<int>(std::floor(b.dy * grid->resolution / bucket_size_m));
      if (axb != bxb || ayb != byb) {return false;}
      const double tha = wrap_angle(a.aidx * ang_step);
      const double thb = wrap_angle(b.aidx * ang_step);
      const int aab = std::clamp(
        static_cast<int>(std::floor((tha + kPi) / (kTwoPi / kAngBuckets))),
        0, kAngBuckets - 1);
      const int bab = std::clamp(
        static_cast<int>(std::floor((thb + kPi) / (kTwoPi / kAngBuckets))),
        0, kAngBuckets - 1);
      return aab == bab;
    };

  auto update_bounds = [&]()
    {
      double w = hyps.empty() ? 0.0 : hyps[0].score;
      double b = 0.0;
      for (const auto & h : hyps) {
        w = std::min(w, h.score);
        b = std::max(b, h.score);
      }
      const double rel_floor = (b > 0.35) ? (b - 0.25) : 0.0;
      if (static_cast<int>(hyps.size()) < topk) {
        worst_kept = rel_floor;
      } else {
        worst_kept = std::max(w, rel_floor);
      }
    };

    // B&B 主循环
  std::int64_t expanded = 0;
  std::int64_t next_time_check = 2048;
  while (!pq.empty()) {
    const Node nd = pq.top();
    pq.pop();
    if (nd.bound <= worst_kept) {
      break;        // 剪枝：所有剩余节点上界均不超过当前第 K 优下界或最优峰-裕度阈值
    }
    const auto & A = ang[static_cast<std::size_t>(nd.aidx)];
    if (nd.level == 0) {
      if (nd.bx < A.dx_min || nd.bx > A.dx_max ||
        nd.by < A.dy_min || nd.by > A.dy_max)
      {
        continue;
      }
      const int8_t origin_cell = grid->cells[
        static_cast<std::size_t>(nd.by) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(nd.bx)];
      if (origin_cell < 0 || origin_cell > 65) {
        continue;
      }
        // 精确重评分（出界点 0 分，优先走 uint8 内部快路径 + 早停）
      double sc = 0.0;
      const double min_need = std::max(0.10, worst_kept);
      const std::uint8_t * like_u8 = field.likeDataU8();
      if (like_u8 != nullptr &&
        nd.bx + A.min_cx >= 0 && nd.bx + A.max_cx < W &&
        nd.by + A.min_cy >= 0 && nd.by + A.max_cy < H)
      {
        const std::uint32_t min_sum_u8 = static_cast<std::uint32_t>(
          std::floor(min_need * 255.0 * static_cast<double>(n)));
        const std::uint8_t * base =
          like_u8 + static_cast<std::size_t>(nd.by) * static_cast<std::size_t>(W) +
          static_cast<std::size_t>(nd.bx);
        const std::uint32_t sum = eval_u8_inbounds_early_exit(
          base, A.flat_offs.data(), n, min_sum_u8);
        sc = static_cast<double>(sum) * inv_n255;
      } else {
        for (const auto & c : A.cells) {
          sc += field.likelihood(c.x() + nd.bx, c.y() + nd.by);
        }
        sc /= n;
      }
      if (sc > 0.10 && sc > worst_kept) {
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
          int bucket_cnt = 0;
          int worst_in_bucket = -1;
          for (std::size_t hi = 0; hi < hyps.size(); ++hi) {
            if (same_bucket(hyps[hi], cand)) {
              ++bucket_cnt;
              if (worst_in_bucket < 0 ||
                hyps[hi].score < hyps[static_cast<std::size_t>(worst_in_bucket)].score)
              {
                worst_in_bucket = static_cast<int>(hi);
              }
            }
          }
          if (bucket_cnt >= max_per_bucket && worst_in_bucket >= 0) {
            auto & wb = hyps[static_cast<std::size_t>(worst_in_bucket)];
            if (cand.score > wb.score) {
              wb = cand;
            }
          } else if (static_cast<int>(hyps.size()) < topk) {
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
        update_bounds();
      }
      continue;
    }

    const int h = 1 << (nd.level - 1);
    for (int dy = 0; dy < 2; ++dy) {
      for (int dx = 0; dx < 2; ++dx) {
        const int bx = nd.bx + dx * h;
        const int by = nd.by + dy * h;
        if (bx > A.dx_max || by > A.dy_max ||
          bx + h - 1 < A.dx_min || by + h - 1 < A.dy_min)
        {
          continue;
        }
        const double sc = eval_block_bound(
          nd.level - 1, bx, by, A, std::max(0.05, worst_kept));
        if (sc > 0.05 && sc > worst_kept) {
          pq.push({sc, nd.level - 1, bx, by, nd.aidx});
        }
        ++expanded;
      }
    }
    if (use_time_budget && expanded >= next_time_check) {
      next_time_check = expanded + 2048;
      const double el = elapsed_ms();
      if ((!hyps.empty() && el >= 0.75 * max_time_ms) ||
        el >= max_time_ms)
      {
        break;  // 墙钟熔断：有候选时保留 25% 给精修；无候选时到总预算硬止损
      }
    }
    if (expanded > 30000000LL) {break;}        // 防御性上限
  }
  if (hyps.empty()) {return out;}

    // 每个假设：连续小窗口细修 + GN 亚像素抛光 + 全点集重评分
  struct Refined { double score; Pose2 pose; };
  std::vector<Refined> refined;
  refined.reserve(hyps.size());
  std::vector<Eigen::Vector2d> rot_pts;
  rot_pts.reserve(valid_pts.size());
  // 按粗分降序排列，确保时间熔断触发时优先精修最高分候选
  std::sort(hyps.begin(), hyps.end(),
    [](const Hyp & a, const Hyp & b) {return a.score > b.score;});
  const std::size_t min_polish = std::min<std::size_t>(hyps.size(), 6u);
  FieldGnMatcher gn_refiner;
  MatchOptions gn_opt;
  gn_opt.max_iterations = 10;
  for (std::size_t hi = 0; hi < hyps.size(); ++hi) {
    if (use_time_budget && hi >= 1u && elapsed_ms() >= max_time_ms) {
      break;
    }
    if (use_time_budget && hi >= min_polish && elapsed_ms() >= 0.90 * max_time_ms) {
      break;
    }
    const auto & hyp = hyps[hi];
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
    const MatchResult gn_res = gn_refiner.match(field, {bx, by, bth}, scan, gn_opt);
    if (gn_res.valid && pose_is_finite(gn_res.pose) &&
      std::isfinite(gn_res.score) && gn_res.score + 0.01 >= best_sc)
    {
      bx = gn_res.pose.x;
      by = gn_res.pose.y;
      bth = wrap_angle(gn_res.pose.theta);
      best_sc = std::max(best_sc, gn_res.score);
    }
    refined.push_back({best_sc, {bx, by, bth}});
  }
  std::sort(refined.begin(), refined.end(),
    [](const Refined & a, const Refined & b) {
      return a.score > b.score;
    });

    // 精修后最终 NMS 去重，保证 alternates 满足最小间距且总数 <= topk
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
  return out;
}

double compute_adaptive_ang_step(const MapField & field, const Scan & scan)
{
  constexpr double kMinStep = 0.75 * kPi / 180.0;
  constexpr double kMaxStep = 3.0 * kPi / 180.0;
  constexpr double kFallback = 2.0 * kPi / 180.0;
  if (!field.valid() || scan.points.empty()) {
    return kFallback;
  }
  std::vector<double> ranges;
  ranges.reserve(scan.points.size());
  for (const auto & pt : scan.points) {
    if (pt_is_finite(pt)) {
      const double r = pt.norm();
      if (r > 1e-3) {ranges.push_back(r);}
    }
  }
  if (ranges.empty()) {return kFallback;}
  const std::size_t idx90 = (ranges.size() * 9) / 10;
  std::nth_element(
    ranges.begin(),
    ranges.begin() + static_cast<std::ptrdiff_t>(idx90),
    ranges.end());
  const double r_p90 = std::max(1.5, ranges[idx90]);
  const double sigma = std::max(field.grid().resolution, field.sigma_m_);
  return std::clamp(0.85 * sigma / r_p90, kMinStep, kMaxStep);
}

BeamPhysicsStats evaluate_beam_physics(
  const MapField & field, const Pose2 & pose, const Scan & scan,
  double inlier_gate, bool check_occupied)
{
  BeamPhysicsStats stats;
  if (!field.valid() || !pose_is_finite(pose) || scan.points.empty()) {
    return stats;
  }
  const auto & grid = field.grid();
  const Eigen::Vector2i og = grid.worldToGridIndex(pose.x, pose.y);
  if (!grid.inside(og.x(), og.y())) {
    stats.visibility = 0.05;
    stats.wall_visibility = 0.05;
    return stats;
  }
  const int8_t oc = grid.cells[
    static_cast<std::size_t>(og.y()) * static_cast<std::size_t>(grid.width) +
    static_cast<std::size_t>(og.x())];
  if (oc < 0 || (check_occupied && oc > 65)) {
    stats.visibility = 0.15;
    stats.wall_visibility = 0.15;
    return stats;
  }

  const double res = grid.resolution;
  const double sigma = std::max(res, field.sigma_m_);
  // 所有物理尺度均由地图分辨率与似然平滑尺度 σ 导出，并给 tau_wall 加 0.35m 上限防止粗地图漏判薄墙
  const double tau_wall = std::min(std::max(3.0 * res, 2.5 * sigma), 0.35);
  const double clear_m = std::max(2.0 * res, 1.5 * sigma);
  const double gate = std::isfinite(inlier_gate) ? std::max(res, inlier_gate) : sigma;

  const std::size_t n = scan.points.size();
  const std::size_t stride = std::max<std::size_t>(1, n / 256);
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);

  double total_len = 0.0;
  double wall_unk_energy = 0.0;
  double frontier_unk_energy = 0.0;
  double occ_bad_energy = 0.0;
  double max_wall_unk_depth = 0.0;
  int wall_unk_contradict = 0;
  int occ_contradict = 0;
  double static_like_sum = 0.0;
  int used = 0;
  int inlier_cnt = 0;

  for (std::size_t i = 0; i < n; i += stride) {
    const auto & pt = scan.points[i];
    if (!pt_is_finite(pt)) {continue;}
    const double r = std::hypot(pt.x(), pt.y());
    if (r <= 1e-4) {continue;}

    const double lx = c * pt.x() - s * pt.y();
    const double ly = s * pt.x() + c * pt.y();
    const double ux = lx / r;
    const double uy = ly / r;

    const double ex = pose.x + lx;
    const double ey = pose.y + ly;
    const Eigen::Vector2i end_idx = grid.worldToGridIndex(ex, ey);
    bool end_unknown = !grid.inside(end_idx.x(), end_idx.y());
    if (!end_unknown) {
      const int8_t ec = grid.cells[
        static_cast<std::size_t>(end_idx.y()) * static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(end_idx.x())];
      end_unknown = (ec < 0);
    }
    const Eigen::Vector2d end_gf = grid.worldToGridF(ex, ey);
    const double d_end = end_unknown ? -tau_wall : field.sdf(end_gf.x(), end_gf.y());
    const bool end_inlier = !end_unknown && std::isfinite(d_end) && (std::abs(d_end) <= gate);
    const bool end_on_wall = !end_unknown && std::isfinite(d_end) && (std::abs(d_end) <= tau_wall);
    const bool end_in_free = !end_unknown && std::isfinite(d_end) && (d_end > tau_wall);

    // 基于精确 SDF 的球面跳跃追踪（自由区按 0.75·SDF 大步跳跃，兼顾精度与低耗时）
    double unk_len = end_unknown ? (0.5 * sigma) : 0.0;
    double occ_len = 0.0;
    double first_occ_t = -1.0;
    if (r > 2.0 * clear_m) {
      double t = clear_m;
      const double t_limit = r - clear_m;
      while (t <= t_limit) {
        const Eigen::Vector2i idx = grid.worldToGridIndex(pose.x + t * ux, pose.y + t * uy);
        if (!grid.inside(idx.x(), idx.y())) {
          const double step = std::max(2.0 * res, sigma);
          unk_len += step;
          t += step;
          continue;
        }
        const int8_t cell = grid.cells[
          static_cast<std::size_t>(idx.y()) * static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(idx.x())];
        if (cell < 0) {
          const double step = std::max(2.0 * res, sigma);
          unk_len += step;
          t += step;
        } else if (check_occupied && cell > 65) {
          const double step = std::max(2.0 * res, 0.75 * sigma);
          occ_len += step;
          if (first_occ_t < 0.0) {first_occ_t = t;}
          t += step;
        } else {
          const double d_obs = static_cast<double>(field.cellSdf(idx.x(), idx.y()));
          // 临近薄墙（<=max(2.5格,1.5σ)）时把最小步长缩到单格，防大步跨越 1 格厚封口墙误判为前沿
          const double min_step = (d_obs <= std::max(2.5 * res, 1.5 * sigma)) ?
            res : std::max(2.0 * res, sigma);
          const double step = std::clamp(0.75 * d_obs, min_step, 0.60);
          t += step;
        }
      }
    }

    total_len += r;
    ++used;
    if (end_inlier) {
      ++inlier_cnt;
    }

    // 四态物理分类
    if (end_in_free && unk_len == 0.0 && occ_len == 0.0) {
      // 状态 2：自由空间前景动态遮挡 (z_short)
      ++stats.n_short;
      continue;
    }
    static_like_sum += field.likelihoodAtF(end_gf.x(), end_gf.y());
    if (end_on_wall && occ_len > 0.0 && unk_len <= tau_wall) {
      // 状态 3：穿过建图静态残影团块后准确命中远端真实墙面 (clutter pass-through)
      ++stats.n_clutter_pass;
      ++stats.n_static;
      if (occ_len > 4.0 * tau_wall) {
        occ_bad_energy += 0.4 * (occ_len - 4.0 * tau_wall);
        ++occ_contradict;
        ++stats.n_contradict;
      }
      continue;
    }

    const bool unk_spill = (end_unknown && unk_len > 5.0 * tau_wall) ||
      (unk_len > 8.0 * tau_wall);
    const bool void_pierce = !end_on_wall && (occ_len >= 1.1 * tau_wall) &&
      (first_occ_t >= 0.0) && ((r - first_occ_t) > 3.5 * tau_wall);
    if (unk_spill || void_pierce) {
      // 状态 4：物理拆分三类异常：
      // (a) wall_unk：先穿已知墙面再深穿未知灰区（180°翻转硬矛盾，极限强罚）；
      // (b) frontier_unk：零穿墙直接从自由区射出地图边界/侧向开口（合法前沿，仅温和覆盖折扣）；
      // (c) void_pierce：穿透映射区内旧障碍（门开/移走家具，中等容差）。
      if (unk_spill && occ_len > 0.0) {
        wall_unk_energy += std::min(r, unk_len);
        max_wall_unk_depth = std::max(max_wall_unk_depth, unk_len);
        ++wall_unk_contradict;
        ++stats.n_contradict;
      } else if (unk_spill) {
        frontier_unk_energy += std::min(r, unk_len);
        ++stats.n_frontier_unk;
      } else {
        occ_bad_energy += std::min(r, 0.5 * (r - first_occ_t));
        ++occ_contradict;
        ++stats.n_contradict;
      }
      continue;
    }

    // 状态 1：常规静态墙面观测
    ++stats.n_static;
  }

  if (used == 0 || total_len <= 1e-6) {
    return stats;
  }

  // 硬矛盾分项惩罚：穿墙入未知(wall_unk)保持极限判别，穿占旧障碍(occ)保留门开/家具容差
  const double wall_unk_ratio = wall_unk_energy / total_len;
  const double excess_wall_unk_ratio = std::max(0.0, wall_unk_ratio - 0.05);
  const double excess_wall_unk_frac = std::max(
    0.0, static_cast<double>(wall_unk_contradict) / static_cast<double>(used) - 0.035);
  const double unk_depth_term = (wall_unk_ratio > 0.06) ?
    std::min(0.80, 0.08 * std::max(0.0, max_wall_unk_depth - 5.0 * tau_wall)) : 0.0;
  const double wall_unk_penalty =
    6.5 * excess_wall_unk_ratio + 4.0 * excess_wall_unk_frac + unk_depth_term;

  const double occ_ratio = occ_bad_energy / total_len;
  const double excess_occ_ratio = std::max(0.0, occ_ratio - 0.10);
  const double excess_occ_frac = std::max(
    0.0, static_cast<double>(occ_contradict) / static_cast<double>(used) - 0.08);
  const double occ_penalty = 2.5 * excess_occ_ratio + 2.0 * excess_occ_frac;

  // 复合硬矛盾：主项 + 0.35 * 次项，避免“一半穿墙 + 一半穿墙入未知”被 std::max 低估
  const double hard_penalty =
    std::max(wall_unk_penalty, occ_penalty) +
    0.35 * std::min(wall_unk_penalty, occ_penalty);
  stats.wall_visibility = std::clamp(std::exp(-hard_penalty), 0.05, 1.0);

  // 零穿墙纯前沿外溢：在 GLOBAL 期给温和覆盖折扣（保底 0.62），防止未封口真实位姿被一刀砍死跳镜像
  const double frontier_ratio = frontier_unk_energy / total_len;
  const double excess_frontier_ratio = std::max(0.0, frontier_ratio - 0.08);
  const double excess_frontier_frac = std::max(
    0.0, static_cast<double>(stats.n_frontier_unk) / static_cast<double>(used) - 0.06);
  const double frontier_penalty =
    1.2 * excess_frontier_ratio + 0.8 * excess_frontier_frac;
  stats.frontier_visibility = std::clamp(std::exp(-frontier_penalty), 0.62, 1.0);
  stats.short_ratio = static_cast<double>(stats.n_short) / static_cast<double>(used);
  stats.frontier_ratio = static_cast<double>(stats.n_frontier_unk) / static_cast<double>(used);

  stats.visibility = std::clamp(
    stats.wall_visibility * stats.frontier_visibility, 0.05, 1.0);

  // 动态遮挡分母修正：最多扣除 55% 的 z_short 光束，既抗高密人群又防止空旷错位假高分/内点率
  const double min_explainable = 0.45 * static_cast<double>(used);
  const double eff_denom = std::max(
    min_explainable, static_cast<double>(used - stats.n_short));
  stats.static_score = std::clamp(
    static_like_sum / std::max(1.0, eff_denom), 0.0, 1.0);
  stats.static_inlier_ratio = std::clamp(
    static_cast<double>(inlier_cnt) / std::max(1.0, eff_denom), 0.0, 1.0);
  return stats;
}

double beam_visibility_consistency(
  const MapField & field, const Pose2 & pose, const Scan & scan,
  bool check_occupied)
{
  return evaluate_beam_physics(
    field, pose, scan, 2.0 * field.sigma_m_, check_occupied).visibility;
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
