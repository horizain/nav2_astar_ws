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
// LocalizerCore 实现：4 状态 [x, y, theta, b_g] 各向异性 EKF 预测、门控与更新
// （多假设序贯贝叶斯晋升与全局恢复已拆分至 src/hypothesis_tracker.cpp）
// ============================================================================
#include "scan_localizer/localizer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "localizer_core_internal.hpp"

namespace sloc
{

namespace
{

using detail::evaluate_observation;
using detail::is_finite_pose;
using detail::is_valid_spd_cov3;
using detail::ScanObservation;


}  // namespace

LocalizerCore::LocalizerCore(CoreConfig cfg)
: cfg_(cfg)
{
  P4_ = Cov4::Identity();
  P4_(3, 3) = std::max(1e-9, cfg_.gyro_bias_init_var);
}

void LocalizerCore::set_map(std::shared_ptr<const MapField> f)
{
  field_ = std::move(f);
  clear_hypotheses();
  anomaly_window_.clear();
}

void LocalizerCore::set_matcher(std::shared_ptr<const Matcher> m)
{
  matcher_ = std::move(m);
}

void LocalizerCore::set_options(const CoreConfig & cfg)
{
  cfg_ = cfg;
  if (cfg_.anomaly_window <= 0) {
    anomaly_window_.clear();
  } else {
    while (static_cast<int>(anomaly_window_.size()) > cfg_.anomaly_window) {
      anomaly_window_.pop_front();
    }
  }
}

void LocalizerCore::set_initial_pose(const Pose2 & p)
{
  set_initial_pose(p, Cov3::Identity() * 1e-4);
}

void LocalizerCore::set_initial_pose(const Pose2 & p, const Cov3 & cov)
{
  if (!is_finite_pose(p)) {
    return;
  }
  last_pose_ = Pose2(p.x, p.y, wrap_angle(p.theta));
  first_fix_ = true;
  manual_global_ = false;
  reject_streak_ = 0;
  stable_tracking_streak_ = 0;
  gyro_bias_ = 0.0;
  P4_ = Cov4::Zero();
  if (is_valid_spd_cov3(cov)) {
    P4_.topLeftCorner<3, 3>() = 0.5 * (cov + cov.transpose());
  } else {
    P4_.topLeftCorner<3, 3>() = Cov3::Identity() * 1e-4;
  }
  P4_(3, 3) = std::max(1e-9, cfg_.gyro_bias_init_var);
  clear_hypotheses();
  anomaly_window_.clear();

  if (has_odom_) {
    odom_at_last_scan_ = last_odom_;
    state_ = LocState::TRACKING;
  } else if (cfg_.allow_odomless) {
    state_ = LocState::TRACKING;
  } else {
    state_ = LocState::UNINIT;
  }
  if (has_imu_) {
    imu_yaw_at_last_scan_ = imu_yaw_;
  }
  imu_dt_accum_ = 0.0;
  imu_consumed_ = true;
}

bool LocalizerCore::request_global(bool manual)
{
  if (!field_ || !matcher_) {
    return false;
  }
  if (state_ == LocState::GLOBAL) {
    return false;
  }
  state_ = LocState::GLOBAL;
  manual_global_ = manual;
  reject_streak_ = 0;
  stable_tracking_streak_ = 0;
  anomaly_window_.clear();
  return true;
}

void LocalizerCore::set_global_fn(GlobalFn fn)
{
  global_fn_ = std::move(fn);
}

void LocalizerCore::set_global_multi_fn(GlobalMultiFn fn)
{
  global_multi_fn_ = std::move(fn);
}

void LocalizerCore::enable_builtin_global(
  bool enable, int beams_subsample, double ang_step, int n_hypotheses,
  double max_time_ms)
{
  builtin_global_enabled_ = enable;
  builtin_beams_sub_ = std::max(1, beams_subsample);
  builtin_ang_step_ = (std::isfinite(ang_step) && ang_step > 0.0) ? ang_step : 0.0;
  builtin_n_hyps_ = std::max(1, n_hypotheses);
  builtin_max_time_ms_ = (std::isfinite(max_time_ms) && max_time_ms > 0.0) ? max_time_ms : 0.0;
}

void LocalizerCore::on_odom(const Pose2 & odom_pose)
{
  if (!is_finite_pose(odom_pose)) {
    return;
  }
  const Pose2 clean_odom(odom_pose.x, odom_pose.y, wrap_angle(odom_pose.theta));
  if (!has_odom_) {
    odom_at_last_scan_ = clean_odom;
    if (state_ == LocState::UNINIT && first_fix_) {
      state_ = LocState::TRACKING;
    }
  }
  last_odom_ = clean_odom;
  has_odom_ = true;
}

void LocalizerCore::on_imu_yaw(double integrated_yaw)
{
  on_imu_yaw(integrated_yaw, 0.0);
}

void LocalizerCore::on_imu_yaw(double integrated_yaw, double dt)
{
  if (!cfg_.use_imu_yaw || !std::isfinite(integrated_yaw) || !std::isfinite(dt)) {
    return;
  }
  if (!has_imu_) {
    // 首帧双锚定，杜绝非零初始 yaw 造成的首帧航向跳变
    imu_yaw_ = integrated_yaw;
    imu_yaw_at_last_scan_ = integrated_yaw;
    imu_dt_accum_ = 0.0;
    imu_consumed_ = true;
    has_imu_ = true;
    return;
  }
  if (imu_consumed_) {
    imu_yaw_at_last_scan_ = imu_yaw_;
    imu_dt_accum_ = 0.0;
    imu_consumed_ = false;
  }
  imu_yaw_ = integrated_yaw;
  if (dt > 0.0) {
    imu_dt_accum_ += dt;
  }
}

void LocalizerCore::on_imu_gyro(double wz, double dt)
{
  if (!cfg_.use_imu_yaw || !std::isfinite(wz) || !std::isfinite(dt) || dt <= 0.0) {
    return;
  }
  if (!has_imu_) {
    imu_yaw_at_last_scan_ = 0.0;
    imu_yaw_ = wz * dt;
    imu_dt_accum_ = dt;
    imu_consumed_ = false;
    has_imu_ = true;
    return;
  }
  if (imu_consumed_) {
    imu_yaw_at_last_scan_ = imu_yaw_;
    imu_dt_accum_ = 0.0;
    imu_consumed_ = false;
  }
  imu_yaw_ += wz * dt;
  imu_dt_accum_ += dt;
}

Pose2 LocalizerCore::map_to_odom() const
{
  if (!has_odom_) {
    return last_pose_;
  }
  // T_map_odom = T_map_base(t_scan) * inv(T_odom_base(t_scan))
  return last_pose_ * odom_at_last_scan_.inverse();
}

void LocalizerCore::compute_motion_delta(
  Pose2 & delta, bool & imu_used, double & dt_imu)
{
  delta = Pose2{};
  imu_used = false;
  dt_imu = 0.0;

  if (has_odom_) {
    const Pose2 odom_now = last_odom_;
    if (first_fix_ || !hypotheses_.empty()) {
      delta = odom_at_last_scan_.inverse() * odom_now;
    }
    odom_at_last_scan_ = odom_now;
  }

  if (cfg_.use_imu_yaw && has_imu_ && !imu_consumed_) {
    const double dth_raw = wrap_angle(imu_yaw_ - imu_yaw_at_last_scan_);
    dt_imu = imu_dt_accum_;
    const double dth_corr = (dt_imu > 0.0) ?
      wrap_angle(dth_raw - gyro_bias_ * dt_imu) :
      dth_raw;
    delta.theta = dth_corr;
    imu_yaw_at_last_scan_ = imu_yaw_;
    imu_dt_accum_ = 0.0;
    imu_consumed_ = true;
    imu_used = true;
  }
}

CoreOutput LocalizerCore::on_scan(const Scan & scan)
{
  CoreOutput out;
  out.pose = last_pose_;
  out.covariance = last_cov();
  out.state = state_;
  if (!field_ || !matcher_) {
    return out;
  }
  if (!has_odom_ && !cfg_.allow_odomless) {
    return out;
  }

  // 防护：过滤扫描中的 NaN / ±inf 点（全有限时零拷贝快路径）
  bool all_finite = true;
  for (const auto & pt : scan.points) {
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y())) {
      all_finite = false;
      break;
    }
  }
  Scan filtered_scan;
  const Scan * active_scan = &scan;
  if (!all_finite) {
    filtered_scan.points.reserve(scan.points.size());
    for (const auto & pt : scan.points) {
      if (std::isfinite(pt.x()) && std::isfinite(pt.y())) {
        filtered_scan.points.push_back(pt);
      }
    }
    active_scan = &filtered_scan;
  }

  // ---------- UNINIT / LOST / GLOBAL：尝试多假设验证或全局搜索 ----------
  if (state_ == LocState::UNINIT || state_ == LocState::LOST ||
    state_ == LocState::GLOBAL)
  {
    handle_recovery_scan(*active_scan, out);
    return out;
  }

  // ---------- 预测：SE(2) + 陀螺零偏 4D EKF 预测 ----------
  Pose2 delta{};
  bool imu_used = false;
  double dt_imu = 0.0;
  compute_motion_delta(delta, imu_used, dt_imu);

  // /odom 瞬时跳变/断流突变防护：单周期增量超过物理合理上限时，
  // 保留里程计先验以兼容真实长步长/断流位移，但自动膨胀过程噪声、放宽搜窗，
  // 且当里程计先验匹配不佳时自动回退至无里程计静止先验 last_pose_ 兜底。
  const double raw_dtrans = std::hypot(delta.x, delta.y);
  const double raw_drot = std::abs(delta.theta);
  const bool odom_noise_enabled =
    cfg_.odom_trans_var_per_m > 0.0 || cfg_.odom_rot_var_per_rad > 0.0;
  const bool odom_trans_jump = has_odom_ && first_fix_ && odom_noise_enabled &&
    cfg_.odom_jump_max_trans > 0.0 && raw_dtrans > cfg_.odom_jump_max_trans;
  const bool odom_rot_jump = has_odom_ && first_fix_ && !imu_used && odom_noise_enabled &&
    cfg_.odom_jump_max_rot > 0.0 && raw_drot > cfg_.odom_jump_max_rot;
  const bool odom_jump = odom_trans_jump || odom_rot_jump;

  if (!hypotheses_.empty() && first_fix_) {
    propagate_hypotheses(delta);
  }

  const double th = last_pose_.theta;
  const double c = std::cos(th);
  const double s = std::sin(th);
  Pose2 pred = first_fix_ ?
    Pose2(
      last_pose_.x + c * delta.x - s * delta.y,
      last_pose_.y + s * delta.x + c * delta.y,
      wrap_angle(th + delta.theta)) :
    last_pose_;

  // 4×4 状态转移雅可比 F4（航向不确定性向 x,y 传播；IMU 零偏向航向传播）
  Eigen::Matrix4d F4 = Eigen::Matrix4d::Identity();
  if (first_fix_) {
    F4(0, 2) = -(delta.x * s + delta.y * c);
    F4(1, 2) = delta.x * c - delta.y * s;
    if (imu_used && dt_imu > 0.0) {
      F4(2, 3) = -dt_imu;
    }
  }

  // 过程噪声 Q4：本体系噪声经 G 旋转至 map 系 + 陀螺零偏随机游走
  double q_trans = 0.0;
  double q_rot = 0.0;
  if (has_odom_ && !odom_jump) {
    const double dtrans = std::hypot(delta.x, delta.y);
    const double drot = std::abs(delta.theta);
    q_trans = dtrans * cfg_.odom_trans_var_per_m + drot * cfg_.odom_trans_var_per_rad;
    q_rot = dtrans * cfg_.odom_rot_var_per_m +
      drot * (imu_used ? cfg_.imu_rot_var_per_rad : cfg_.odom_rot_var_per_rad);
    if (imu_used && dt_imu > 0.0) {
      q_rot += dt_imu * cfg_.imu_rot_var_per_rad;
    }
  } else if (odom_jump) {
    const double dtrans = std::hypot(delta.x, delta.y);
    const double drot = std::abs(delta.theta);
    q_trans = std::max(
      std::max(0.0, cfg_.odomless_trans_var),
      dtrans * cfg_.odom_trans_var_per_m + drot * cfg_.odom_trans_var_per_rad +
      0.25 * raw_dtrans * raw_dtrans);
    q_rot = imu_used ?
      (drot * cfg_.imu_rot_var_per_rad + dt_imu * cfg_.imu_rot_var_per_rad) :
      std::max(
        std::max(0.0, cfg_.odomless_rot_var),
        dtrans * cfg_.odom_rot_var_per_m + drot * cfg_.odom_rot_var_per_rad +
        0.25 * raw_drot * raw_drot);
  } else {
    q_trans = std::max(0.0, cfg_.odomless_trans_var);
    q_rot = imu_used ?
      (std::abs(delta.theta) * cfg_.imu_rot_var_per_rad + dt_imu * cfg_.imu_rot_var_per_rad) :
      std::max(0.0, cfg_.odomless_rot_var);
  }

  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  G(0, 0) = c;
  G(0, 1) = -s;
  G(1, 0) = s;
  G(1, 1) = c;
  Eigen::Matrix3d Q_body = Eigen::Matrix3d::Zero();
  Q_body(0, 0) = q_trans;
  Q_body(1, 1) = q_trans;
  Q_body(2, 2) = q_rot;
  // 速度耦合时序抖动噪声 Q_sync = (σ_t/Δt)² · Δu · Δuᵀ：静止时严格为 0，高速/急转时仅沿运动矢量放宽
  if (has_odom_ && cfg_.sync_jitter_ratio > 0.0 &&
    (cfg_.odom_trans_var_per_m > 0.0 || cfg_.odom_rot_var_per_rad > 0.0))
  {
    const double s2 = cfg_.sync_jitter_ratio * cfg_.sync_jitter_ratio;
    const Eigen::Vector3d du(delta.x, delta.y, delta.theta);
    Q_body += s2 * (du * du.transpose());
  }

  Eigen::Matrix4d Q4 = Eigen::Matrix4d::Zero();
  Q4.topLeftCorner<3, 3>() = G * Q_body * G.transpose();
  // MapPrior 退化区前瞻整形：进入走廊/单墙低可定位区时，
  // 仅沿退化主轴温和放宽过程不确定度，横向强轴保持紧锁
  if (cfg_.use_map_prior && has_odom_ && q_trans > 0.0) {
    const LocalMapPrior mp = field_->queryPrior(pred.x, pred.y);
    if (mp.localizability < 0.15) {
      const double deg_ratio = 1.0 - mp.localizability / 0.15;
      const double q_deg = 0.5 * deg_ratio * q_trans;
      Q4.topLeftCorner<2, 2>() +=
        q_deg * (mp.degenerate_axis * mp.degenerate_axis.transpose());
    }
  }
  if (imu_used && dt_imu > 0.0) {
    Q4(3, 3) = std::max(0.0, cfg_.gyro_bias_rw_var_per_s) * dt_imu;
  }

  Cov4 P4_pred = F4 * P4_ * F4.transpose() + Q4;
  if (cfg_.min_pred_trans_var > 0.0) {
    P4_pred(0, 0) = std::max(P4_pred(0, 0), cfg_.min_pred_trans_var);
    P4_pred(1, 1) = std::max(P4_pred(1, 1), cfg_.min_pred_trans_var);
  }
  if (cfg_.min_pred_rot_var > 0.0) {
    P4_pred(2, 2) = std::max(P4_pred(2, 2), cfg_.min_pred_rot_var);
  }
  P4_pred = 0.5 * (P4_pred + P4_pred.transpose());

  // ---------- 协方差驱动自适应搜窗 + 匹配 ----------
  MatchOptions active_opt = cfg_.match_opt;
  if (odom_jump) {
    const double base_wlin = cfg_.match_opt.window_lin;
    const double base_wang = cfg_.match_opt.window_ang;
    active_opt.window_lin = std::clamp(
      std::max(2.0 * base_wlin, raw_dtrans + 0.10), base_wlin, 0.60);
    active_opt.window_ang = std::clamp(
      std::max(2.0 * base_wang, raw_drot + 0.10), base_wang, 0.60);
  } else if (cfg_.adaptive_window && has_odom_) {
    const double base_wlin = cfg_.match_opt.window_lin;
    const double base_wang = cfg_.match_opt.window_ang;
    const double pos_std = std::sqrt(
      std::max(0.0, std::max(P4_pred(0, 0), P4_pred(1, 1))));
    const double ang_std = std::sqrt(std::max(0.0, P4_pred(2, 2)));
    if (state_ == LocState::TRACKING && reject_streak_ == 0) {
      active_opt.window_lin = std::clamp(
        3.5 * pos_std + 0.03, std::max(0.06, 0.55 * base_wlin), base_wlin);
      active_opt.window_ang = std::clamp(
        3.5 * ang_std + 0.02, std::max(0.04, 0.55 * base_wang), base_wang);
    } else {
      // 连续拒帧或 DEGRADED 时，除先验协方差外还按拒帧数渐进扩窗（最高 3x / 0.50m），
      // 避免默认低过程噪声下大转弯/滑移超出基础搜窗后 pos_std 几乎不增长导致失锁。
      const double streak_scale = 1.0 + 0.25 * std::min(reject_streak_, 8);
      const double max_wlin = std::min(0.50, std::max(base_wlin, 3.0 * base_wlin));
      const double max_wang = std::min(0.45, std::max(base_wang, 3.0 * base_wang));
      active_opt.window_lin = std::clamp(
        std::max(3.5 * pos_std + 0.05, streak_scale * base_wlin), base_wlin, max_wlin);
      active_opt.window_ang = std::clamp(
        std::max(3.5 * ang_std + 0.04, streak_scale * base_wang), base_wang, max_wang);
    }
  }
  MatchResult mr = matcher_->match(*field_, pred, *active_scan, active_opt);
  auto is_mr_finite = [](const MatchResult & r) {
      return r.valid &&
             is_finite_pose(r.pose) &&
             std::isfinite(r.score) &&
             std::isfinite(r.inlier_ratio) &&
             r.covariance.allFinite();
    };
  if (odom_jump && first_fix_) {
    const bool pred_ok = is_mr_finite(mr) &&
      mr.score >= cfg_.min_score &&
      mr.inlier_ratio >= cfg_.min_inlier_ratio &&
      std::hypot(mr.pose.x - pred.x, mr.pose.y - pred.y) <= active_opt.window_lin;
    if (!pred_ok) {
      const MatchResult fb = matcher_->match(*field_, last_pose_, *active_scan, active_opt);
      if (is_mr_finite(fb) &&
        (!is_mr_finite(mr) || fb.score > mr.score + 0.05 ||
        std::hypot(fb.pose.x - last_pose_.x, fb.pose.y - last_pose_.y) <
        std::hypot(mr.pose.x - pred.x, mr.pose.y - pred.y)))
      {
        mr = fb;
        pred = last_pose_;
      }
    }
  }
  const bool mr_finite = is_mr_finite(mr);
  // 正交观测评估：几何贴合、前景动态遮挡、硬穿墙矛盾、前沿覆盖度完全解耦
  ScanObservation obs;
  if (mr_finite) {
    obs = evaluate_observation(
      *field_, mr.pose, *active_scan, mr.score, mr.inlier_ratio,
      cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
  }

  // ---------- 五道正交门控（valid / wall_phys / inlier / score / cov / mahal） ----------
  // 硬物理门控独立判别穿墙/穿墙入未知矛盾；纯自由前沿外溢不再乘进几何分污染 min_score 门限
  const bool gate_wall = mr_finite && (obs.wall_visibility >= cfg_.min_wall_visibility);
  const double track_score = gate_wall ? obs.static_score : (obs.geom_score * obs.wall_visibility);
  const double track_inlier = gate_wall ? obs.static_inlier : obs.geom_inlier;

  out.score = std::isfinite(track_score) ? track_score : 0.0;
  out.inlier_ratio = std::isfinite(track_inlier) ? track_inlier : 0.0;
  out.pose = pred;

  const bool gate_inlier = mr_finite && (track_inlier >= cfg_.min_inlier_ratio);
  const bool gate_score = mr_finite && (track_score >= cfg_.min_score);

  Cov3 R = Cov3::Identity();
  bool gate_cov = false;
  double cov_cond = std::numeric_limits<double>::infinity();
  if (mr_finite) {
    R = 0.5 * (mr.covariance + mr.covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Cov3> es(R);
    if (es.info() == Eigen::Success && es.eigenvalues().size() == 3) {
      const double min_eig = es.eigenvalues().minCoeff();
      gate_cov = std::isfinite(min_eig) && (min_eig > 0.0) &&
        (min_eig < cfg_.max_cov_eig);
      if (gate_cov) {
        // 用 2x2 平移子块条件数（量纲统一为 m²）判断几何退化，避免 m² 与 rad² 混算
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es_xy(R.topLeftCorner<2, 2>());
        if (es_xy.info() == Eigen::Success && es_xy.eigenvalues().size() == 2) {
          const double xy_min = es_xy.eigenvalues().minCoeff();
          const double xy_max = es_xy.eigenvalues().maxCoeff();
          if (std::isfinite(xy_min) && std::isfinite(xy_max) && xy_min > 1e-12) {
            cov_cond = xy_max / xy_min;
          }
        }
      }
    }
  }

  Eigen::Vector3d innov = Eigen::Vector3d::Zero();
  Cov3 S = Cov3::Identity();
  Eigen::LDLT<Cov3> ldlt;
  bool gate_mahal = false;
  double mahal_val = 0.0;
  if (mr_finite) {
    innov = Eigen::Vector3d(
      mr.pose.x - pred.x,
      mr.pose.y - pred.y,
      wrap_angle(mr.pose.theta - pred.theta));
    const Cov3 S_raw = P4_pred.topLeftCorner<3, 3>() + R;
    S = 0.5 * (S_raw + S_raw.transpose()) + Cov3::Identity() * 1e-9;
    ldlt.compute(S);
    if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
      const Eigen::Vector3d S_inv_innov = ldlt.solve(innov);
      if (ldlt.info() == Eigen::Success && S_inv_innov.allFinite()) {
        const double mahal = innov.dot(S_inv_innov);
        if (std::isfinite(mahal) && mahal >= 0.0) {
          mahal_val = mahal;
          gate_mahal = mahal < cfg_.mahal_gate;
        }
      }
    }
  }

  // 高几何置信度且非退化下的 kMahal 软门控：
  // 仅当其余四门全通过、几何贴合极佳且匹配非退化（排除对称走廊沿墙滑移），
  // 在连续 >=1 帧确认同一局部吸引盆后软膨胀 P4_pred 放行，根治静止/高漂移误拒锁死
  const bool high_geom_conf = mr_finite && gate_wall && gate_inlier && gate_score &&
    gate_cov &&
    (track_score >= cfg_.mahal_soft_min_score) &&
    (track_inlier >= cfg_.mahal_soft_min_inlier) &&
    (obs.wall_visibility >= 0.99) &&
    (cov_cond <= cfg_.mahal_soft_max_cond);
  const double max_local_xy = high_geom_conf ?
    std::max(0.50, 3.0 * active_opt.window_lin) :
    std::max(0.20, 1.5 * active_opt.window_lin);
  const double max_local_th = high_geom_conf ?
    std::max(0.35, 3.0 * active_opt.window_ang) :
    std::max(0.15, 1.5 * active_opt.window_ang);

  if (high_geom_conf && !gate_mahal && reject_streak_ >= 1) {
    const double innov_xy = std::hypot(innov(0), innov(1));
    if (innov_xy <= max_local_xy && std::abs(innov(2)) <= max_local_th) {
      P4_pred.topLeftCorner<3, 3>() += innov * innov.transpose();
      P4_pred = 0.5 * (P4_pred + P4_pred.transpose());
      const Cov3 S_soft = P4_pred.topLeftCorner<3, 3>() + R;
      S = 0.5 * (S_soft + S_soft.transpose()) + Cov3::Identity() * 1e-9;
      ldlt.compute(S);
      if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
        const Eigen::Vector3d S_inv_innov = ldlt.solve(innov);
        if (ldlt.info() == Eigen::Success && S_inv_innov.allFinite()) {
          const double mahal_soft = innov.dot(S_inv_innov);
          if (std::isfinite(mahal_soft) && mahal_soft >= 0.0) {
            mahal_val = mahal_soft;
            gate_mahal = mahal_soft < cfg_.mahal_gate;
          }
        }
      }
    }
  }

  // 填充结构化诊断快照，任一门控拒绝均可通过位掩码直接定因
  std::uint32_t reject_flags = GateFlag::kNone;
  if (!mr_finite) {
    reject_flags |= GateFlag::kInvalid;
  } else {
    if (!gate_wall) {reject_flags |= GateFlag::kWallPhys;}
    if (!gate_inlier) {reject_flags |= GateFlag::kInlier;}
    if (!gate_score) {reject_flags |= GateFlag::kScore;}
    if (!gate_cov) {reject_flags |= GateFlag::kCov;}
    if (!gate_mahal) {reject_flags |= GateFlag::kMahal;}
  }

  out.diag.reject_flags = reject_flags;
  out.diag.raw_score = mr_finite ? obs.geom_score : 0.0;
  out.diag.raw_inlier = mr_finite ? obs.geom_inlier : 0.0;
  out.diag.wall_visibility = obs.wall_visibility;
  out.diag.frontier_visibility = obs.frontier_visibility;
  out.diag.short_ratio = obs.short_ratio;
  out.diag.frontier_ratio = obs.frontier_ratio;
  out.diag.mahal_dist = mahal_val;

  const bool accept = mr_finite && gate_wall && gate_inlier && gate_score &&
    gate_mahal && gate_cov;

  // 滑动窗口异常监测（绑架主动触发）：仅将匹配失效、极低分或得分与内点率均不达标
  // 视作绑架异常；高分高内点率但仅因马氏门控拒绝属于正常局部抖动/转弯，绝非绑架。
  const bool is_anomaly = !accept &&
    (!mr_finite || mr.score < cfg_.anomaly_low_score ||
    (!gate_inlier && !gate_score));
  record_anomaly(is_anomaly);

  if (accept) {
    // ---------- 各向异性 4D Joseph 形式 EKF 更新 ----------
    // H = [I_3 | 0], K4 = P4_pred * H^T * S^{-1} = (S^{-1} * H * P4_pred)^T
    const Eigen::Matrix<double, 3, 4> S_inv_HP = ldlt.solve(P4_pred.topRows<3>());
    if (ldlt.info() == Eigen::Success && S_inv_HP.allFinite()) {
      const Eigen::Matrix<double, 4, 3> K4 = S_inv_HP.transpose();
      Eigen::Vector4d dx = K4 * innov;

      if (cfg_.max_update_trans > 0.0) {
        const double tn = std::hypot(dx(0), dx(1));
        if (tn > cfg_.max_update_trans) {
          const double scale = cfg_.max_update_trans / tn;
          dx(0) *= scale;
          dx(1) *= scale;
        }
      }
      if (cfg_.max_update_rot > 0.0) {
        dx(2) = std::clamp(dx(2), -cfg_.max_update_rot, cfg_.max_update_rot);
      }

      last_pose_ = Pose2(
        pred.x + dx(0),
        pred.y + dx(1),
        wrap_angle(pred.theta + dx(2)));
      const double max_b = std::max(0.0, cfg_.max_gyro_bias);
      gyro_bias_ = std::clamp(gyro_bias_ + dx(3), -max_b, max_b);

      Eigen::Matrix<double, 3, 4> H = Eigen::Matrix<double, 3, 4>::Zero();
      H.leftCols<3>() = Eigen::Matrix3d::Identity();
      const Eigen::Matrix4d I_KH = Eigen::Matrix4d::Identity() - K4 * H;
      P4_ = I_KH * P4_pred * I_KH.transpose() + K4 * R * K4.transpose();
      P4_ = 0.5 * (P4_ + P4_.transpose());

      reject_streak_ = 0;
      state_ = LocState::TRACKING;
      ++stable_tracking_streak_;
      if (!hypotheses_.empty() &&
        stable_tracking_streak_ >= std::max(1, cfg_.hyp_confirm_frames))
      {
        clear_hypotheses();
      }
      out.pose = last_pose_;
      out.covariance = last_cov();
      out.state = state_;
      out.accepted = true;
      out.reject_streak = 0;
      first_fix_ = true;
      return out;
    }
  }

  // ---------- 拒绝分支：纯预测外推 + 状态降级 + 多假设/绑架检测 ----------
  last_pose_ = pred;
  P4_ = P4_pred;
  // 局部吸引盆秩 1 经验贝叶斯松弛：仅当高分匹配落在本地跟踪窗邻域内却因短时滞后触发马氏拒绝时，
  // 严格沿当前新息方向叠加有界秩 1 协方差；远距离对称假峰绝不触发任何协方差膨胀（替代旧 P*=2.5）
  if (mr_finite && gate_wall && gate_inlier && gate_score && gate_cov && !gate_mahal) {
    const double innov_xy = std::hypot(innov(0), innov(1));
    if (innov_xy <= max_local_xy && std::abs(innov(2)) <= max_local_th) {
      P4_.topLeftCorner<3, 3>() += 0.6 * (innov * innov.transpose());
      P4_.topLeftCorner<3, 3>() =
        0.5 * (P4_.topLeftCorner<3, 3>() + P4_.topLeftCorner<3, 3>().transpose());
    }
  }
  stable_tracking_streak_ = 0;
  ++reject_streak_;

  const int deg_thr = std::max(1, cfg_.degraded_after);
  const int lost_thr = std::max(deg_thr, cfg_.lost_after);
  if (reject_streak_ >= deg_thr) {
    state_ = LocState::DEGRADED;
  }
  if (reject_streak_ >= lost_thr) {
    state_ = LocState::LOST;
    manual_global_ = false;
    reject_streak_ = 0;
  }

  // DEGRADED 阶段若存在候选假设，尝试多帧验证晋升
  if ((state_ == LocState::DEGRADED || state_ == LocState::LOST) &&
    !hypotheses_.empty())
  {
    if (try_promote_hypotheses(*active_scan, delta, out)) {
      first_fix_ = true;
      return out;
    }
  }

  // 主动绑架检测：滑动窗口内连续/高比例极低分或匹配失效异常 → 加速触发 GLOBAL
  if (cfg_.anomaly_window > 0 && cfg_.anomaly_bad_threshold > 0 &&
    anomaly_count() >= cfg_.anomaly_bad_threshold)
  {
    state_ = LocState::GLOBAL;
    manual_global_ = false;
    reject_streak_ = 0;
    anomaly_window_.clear();
  }

  out.pose = last_pose_;
  out.covariance = last_cov();
  out.state = state_;
  out.accepted = false;
  out.reject_streak = reject_streak_;
  first_fix_ = true;
  return out;
}

}  // namespace sloc
