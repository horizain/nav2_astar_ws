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
// LocalizerCore 实现：4 状态 [x, y, theta, b_g] 各向异性 EKF + 多假设 + 绑架检测
// ============================================================================
#include "scan_localizer/localizer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sloc
{

namespace
{

bool is_finite_pose(const Pose2 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}

bool is_valid_spd_cov3(const Cov3 & c)
{
  if (!c.allFinite()) {
    return false;
  }
  const Cov3 sym = 0.5 * (c + c.transpose());
  Eigen::SelfAdjointEigenSolver<Cov3> es(sym);
  if (es.info() != Eigen::Success || es.eigenvalues().size() != 3) {
    return false;
  }
  return es.eigenvalues().minCoeff() > 0.0;
}

double compute_inlier_ratio(
  const MapField & field, const Pose2 & pose,
  const Scan & scan, double gate)
{
  if (!field.valid() || scan.points.empty() || !std::isfinite(gate)) {
    return 0.0;
  }
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  const auto & grid = field.grid();
  std::size_t inliers = 0;
  for (const auto & pt : scan.points) {
    const double wx = pose.x + c * pt.x() - s * pt.y();
    const double wy = pose.y + s * pt.x() + c * pt.y();
    const Eigen::Vector2d gf = grid.worldToGridF(wx, wy);
    const double d = field.sdf(gf.x(), gf.y());
    if (std::isfinite(d) && std::abs(d) <= gate) {
      ++inliers;
    }
  }
  return static_cast<double>(inliers) / static_cast<double>(scan.points.size());
}

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
  if (!is_finite_pose(p)) {
    return;
  }
  last_pose_ = Pose2(p.x, p.y, wrap_angle(p.theta));
  first_fix_ = true;
  reject_streak_ = 0;
  stable_tracking_streak_ = 0;
  gyro_bias_ = 0.0;
  P4_ = Cov4::Zero();
  P4_.topLeftCorner<3, 3>() = Cov3::Identity() * 1e-4;
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

bool LocalizerCore::request_global()
{
  if (!field_ || !matcher_) {
    return false;
  }
  if (state_ == LocState::GLOBAL) {
    return false;
  }
  state_ = LocState::GLOBAL;
  reject_streak_ = 0;
  stable_tracking_streak_ = 0;
  anomaly_window_.clear();
  return true;
}

void LocalizerCore::set_global_fn(GlobalFn fn)
{
  global_fn_ = std::move(fn);
}

void LocalizerCore::add_hypotheses(const std::vector<Pose2> & hyps)
{
  for (const auto & p : hyps) {
    if (!is_finite_pose(p)) {
      continue;
    }
    if (hypotheses_.size() >= cfg_.max_hypotheses) {
      break;
    }
    Hypothesis h;
    h.pose = Pose2(p.x, p.y, wrap_angle(p.theta));
    hypotheses_.push_back(h);
  }
  if (!hypotheses_.empty()) {
    stable_tracking_streak_ = 0;
  }
}

void LocalizerCore::clear_hypotheses()
{
  hypotheses_.clear();
}

std::vector<Pose2> LocalizerCore::hypothesis_poses() const
{
  std::vector<Pose2> out;
  out.reserve(hypotheses_.size());
  for (const auto & h : hypotheses_) {
    out.push_back(h.pose);
  }
  return out;
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
    if (first_fix_) {
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

void LocalizerCore::propagate_hypotheses(const Pose2 & delta)
{
  for (auto & h : hypotheses_) {
    h.pose = h.pose * delta;
  }
}

bool LocalizerCore::try_promote_hypotheses(const Scan & scan, CoreOutput & out)
{
  if (hypotheses_.empty() || !field_ || !matcher_) {
    return false;
  }
  const int req_frames = std::max(1, cfg_.hyp_confirm_frames);
  int best_idx = -1;
  double best_score = -1.0;
  MatchResult best_mr;

  for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
    auto & h = hypotheses_[i];
    const MatchResult mr = matcher_->match(*field_, h.pose, scan, cfg_.match_opt);
    const bool ok = mr.valid &&
      is_finite_pose(mr.pose) &&
      std::isfinite(mr.score) &&
      std::isfinite(mr.inlier_ratio) &&
      mr.score >= cfg_.hyp_min_score &&
      mr.inlier_ratio >= cfg_.hyp_min_inlier;
    if (ok) {
      h.pose = Pose2(mr.pose.x, mr.pose.y, wrap_angle(mr.pose.theta));
      h.last_score = mr.score;
      ++h.confirm_count;
      if (h.confirm_count >= req_frames && mr.score > best_score) {
        best_score = mr.score;
        best_idx = static_cast<int>(i);
        best_mr = mr;
      }
    } else {
      h.confirm_count = 0;
    }
  }

  if (best_idx < 0) {
    return false;
  }

  last_pose_ = Pose2(best_mr.pose.x, best_mr.pose.y, wrap_angle(best_mr.pose.theta));
  Cov3 pose_cov = Cov3::Identity() * 0.04;
  if (is_valid_spd_cov3(best_mr.covariance)) {
    pose_cov = 0.5 * (best_mr.covariance + best_mr.covariance.transpose());
  }
  const double bias_var = std::max(1e-9, P4_(3, 3));
  P4_ = Cov4::Zero();
  P4_.topLeftCorner<3, 3>() = pose_cov;
  P4_(3, 3) = bias_var;
  state_ = LocState::TRACKING;
  reject_streak_ = 0;
  stable_tracking_streak_ = 0;
  first_fix_ = true;
  clear_hypotheses();
  anomaly_window_.clear();

  out.pose = last_pose_;
  out.covariance = last_cov();
  out.state = state_;
  out.accepted = true;
  out.score = best_mr.score;
  out.inlier_ratio = best_mr.inlier_ratio;
  out.reject_streak = 0;
  return true;
}

void LocalizerCore::record_anomaly(bool is_bad)
{
  if (cfg_.anomaly_window <= 0) {
    return;
  }
  anomaly_window_.push_back(is_bad);
  while (static_cast<int>(anomaly_window_.size()) > cfg_.anomaly_window) {
    anomaly_window_.pop_front();
  }
}

int LocalizerCore::anomaly_count() const
{
  int cnt = 0;
  for (bool b : anomaly_window_) {
    if (b) {
      ++cnt;
    }
  }
  return cnt;
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
    Pose2 delta{};
    bool imu_used = false;
    double dt_imu = 0.0;
    compute_motion_delta(delta, imu_used, dt_imu);

    if (!hypotheses_.empty()) {
      if (first_fix_) {
        propagate_hypotheses(delta);
      }
      if (try_promote_hypotheses(*active_scan, out)) {
        return out;
      }
    }

    if (global_fn_) {
      Pose2 g{};
      double sc = 0.0;
      if (global_fn_(*active_scan, g, sc) && is_finite_pose(g) && std::isfinite(sc)) {
        last_pose_ = Pose2(g.x, g.y, wrap_angle(g.theta));
        P4_ = Cov4::Zero();
        P4_.topLeftCorner<3, 3>() = Cov3::Identity() * 0.04;
        P4_(3, 3) = std::max(1e-9, cfg_.gyro_bias_init_var);
        gyro_bias_ = 0.0;
        if (has_odom_) {
          odom_at_last_scan_ = last_odom_;
        }
        if (has_imu_) {
          imu_yaw_at_last_scan_ = imu_yaw_;
        }
        imu_dt_accum_ = 0.0;
        imu_consumed_ = true;
        state_ = LocState::TRACKING;
        reject_streak_ = 0;
        stable_tracking_streak_ = 0;
        first_fix_ = true;
        clear_hypotheses();
        anomaly_window_.clear();

        out.pose = last_pose_;
        out.covariance = last_cov();
        out.state = state_;
        out.accepted = true;
        out.score = sc;
        out.inlier_ratio = compute_inlier_ratio(
          *field_, last_pose_, *active_scan, cfg_.match_opt.inlier_gate);
        out.reject_streak = 0;
      }
    }
    return out;
  }

  // ---------- 预测：SE(2) + 陀螺零偏 4D EKF 预测 ----------
  Pose2 delta{};
  bool imu_used = false;
  double dt_imu = 0.0;
  compute_motion_delta(delta, imu_used, dt_imu);

  if (!hypotheses_.empty() && first_fix_) {
    propagate_hypotheses(delta);
  }

  const double th = last_pose_.theta;
  const double c = std::cos(th);
  const double s = std::sin(th);
  const Pose2 pred = first_fix_ ?
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
  if (has_odom_) {
    const double dtrans = std::hypot(delta.x, delta.y);
    q_trans = dtrans * cfg_.odom_trans_var_per_m;
    q_rot = dtrans * cfg_.odom_rot_var_per_m +
      std::abs(delta.theta) * (imu_used ? cfg_.imu_rot_var_per_rad : cfg_.odom_rot_var_per_rad);
    if (imu_used && dt_imu > 0.0) {
      q_rot += dt_imu * cfg_.imu_rot_var_per_rad;
    }
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

  Eigen::Matrix4d Q4 = Eigen::Matrix4d::Zero();
  Q4.topLeftCorner<3, 3>() = G * Q_body * G.transpose();
  if (imu_used && dt_imu > 0.0) {
    Q4(3, 3) = std::max(0.0, cfg_.gyro_bias_rw_var_per_s) * dt_imu;
  }

  Cov4 P4_pred = F4 * P4_ * F4.transpose() + Q4;
  P4_pred = 0.5 * (P4_pred + P4_pred.transpose());

  // ---------- 匹配 ----------
  const MatchResult mr = matcher_->match(*field_, pred, *active_scan, cfg_.match_opt);
  const bool mr_finite = mr.valid &&
    is_finite_pose(mr.pose) &&
    std::isfinite(mr.score) &&
    std::isfinite(mr.inlier_ratio) &&
    mr.covariance.allFinite();

  out.score = std::isfinite(mr.score) ? mr.score : 0.0;
  out.inlier_ratio = std::isfinite(mr.inlier_ratio) ? mr.inlier_ratio : 0.0;
  out.pose = pred;

  // ---------- 四道门控（valid / inlier / score / mahal + cov） ----------
  const bool gate_inlier = mr_finite && (mr.inlier_ratio >= cfg_.min_inlier_ratio);
  const bool gate_score = mr_finite && (mr.score >= cfg_.min_score);

  Cov3 R = Cov3::Identity();
  bool gate_cov = false;
  if (mr_finite) {
    R = 0.5 * (mr.covariance + mr.covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Cov3> es(R);
    if (es.info() == Eigen::Success && es.eigenvalues().size() == 3) {
      const double min_eig = es.eigenvalues().minCoeff();
      gate_cov = std::isfinite(min_eig) && (min_eig > 0.0) &&
        (min_eig < cfg_.max_cov_eig);
    }
  }

  Eigen::Vector3d innov = Eigen::Vector3d::Zero();
  Cov3 S = Cov3::Identity();
  Eigen::LDLT<Cov3> ldlt;
  bool gate_mahal = false;
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
        gate_mahal = std::isfinite(mahal) && mahal >= 0.0 &&
          mahal < cfg_.mahal_gate;
      }
    }
  }

  const bool accept = mr_finite && gate_inlier && gate_score &&
    gate_mahal && gate_cov;

  // 滑动窗口异常监测（绑架主动触发）
  const bool is_anomaly = !accept &&
    (!mr_finite || mr.score < cfg_.anomaly_low_score || !gate_mahal);
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
  stable_tracking_streak_ = 0;
  ++reject_streak_;

  const int deg_thr = std::max(1, cfg_.degraded_after);
  const int lost_thr = std::max(deg_thr, cfg_.lost_after);
  if (reject_streak_ >= deg_thr) {
    state_ = LocState::DEGRADED;
  }
  if (reject_streak_ >= lost_thr) {
    state_ = LocState::LOST;
    reject_streak_ = 0;
  }

  // DEGRADED 阶段若存在候选假设，尝试多帧验证晋升
  if ((state_ == LocState::DEGRADED || state_ == LocState::LOST) &&
    !hypotheses_.empty())
  {
    if (try_promote_hypotheses(*active_scan, out)) {
      first_fix_ = true;
      return out;
    }
  }

  // 主动绑架检测：滑动窗口内连续/高比例极低分或大马氏距离异常 → 加速触发 GLOBAL
  if (cfg_.anomaly_window > 0 && cfg_.anomaly_bad_threshold > 0 &&
    anomaly_count() >= cfg_.anomaly_bad_threshold)
  {
    state_ = LocState::GLOBAL;
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
