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

double effective_match_score(
  const MapField & field, const Pose2 & pose, const Scan & scan,
  double raw_score, bool check_vis)
{
  if (!check_vis || scan.points.size() < 16) {
    return raw_score;
  }
  const double vis = beam_visibility_consistency(field, pose, scan, true);
  return raw_score * vis;
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
  bool enable, int beams_subsample, double ang_step, int n_hypotheses)
{
  builtin_global_enabled_ = enable;
  builtin_beams_sub_ = std::max(1, beams_subsample);
  builtin_ang_step_ = (std::isfinite(ang_step) && ang_step > 0.0) ?
    ang_step : (2.0 * kPi / 180.0);
  builtin_n_hyps_ = std::max(1, n_hypotheses);
}

void LocalizerCore::add_hypotheses(const std::vector<Pose2> & hyps)
{
  const bool was_empty = hypotheses_.empty();
  for (const auto & p : hyps) {
    if (!is_finite_pose(p)) {
      continue;
    }
    const Pose2 clean_p(p.x, p.y, wrap_angle(p.theta));
    bool duplicate = false;
    for (const auto & existing : hypotheses_) {
      if (std::hypot(existing.pose.x - clean_p.x, existing.pose.y - clean_p.y) < 0.35 &&
        std::abs(wrap_angle(existing.pose.theta - clean_p.theta)) < 0.25)
      {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }
    Hypothesis h;
    h.pose = clean_p;
    if (hypotheses_.size() < cfg_.max_hypotheses) {
      hypotheses_.push_back(h);
    } else {
      // 僵尸淘汰替换：优先覆盖未确认且历史 EMA 最低的槽位，防止死假设永久占满队列
      int evict_idx = -1;
      double worst_ema = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
        if (hypotheses_[i].confirm_count == 0 && hypotheses_[i].ema_score < worst_ema) {
          worst_ema = hypotheses_[i].ema_score;
          evict_idx = static_cast<int>(i);
        }
      }
      if (evict_idx >= 0) {
        hypotheses_[static_cast<std::size_t>(evict_idx)] = h;
      }
    }
  }
  if (!hypotheses_.empty()) {
    stable_tracking_streak_ = 0;
    if (was_empty && !first_fix_ && has_odom_) {
      odom_at_last_scan_ = last_odom_;
    }
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
  const int ambig_frames = std::max(req_frames, cfg_.hyp_ambiguous_confirm_frames);

  struct EvalEntry
  {
    bool valid{false};
    bool ok{false};
    MatchResult mr{};
  };
  std::vector<EvalEntry> evals(hypotheses_.size());

  int best_idx = -1;
  double best_score = -1.0;
  double frame_best_score = -1.0;

  for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
    auto & h = hypotheses_[i];
    MatchResult mr = matcher_->match(*field_, h.pose, scan, cfg_.match_opt);
    const bool valid_match = mr.valid &&
      is_finite_pose(mr.pose) &&
      std::isfinite(mr.score) &&
      std::isfinite(mr.inlier_ratio);
    if (valid_match) {
      mr.score = effective_match_score(
        *field_, mr.pose, scan, mr.score, cfg_.use_visibility_check);
    }
    evals[i].valid = valid_match;
    evals[i].mr = mr;
    if (valid_match) {
      h.ema_score = (h.eval_count == 0) ? mr.score : (0.7 * h.ema_score + 0.3 * mr.score);
      ++h.eval_count;
      if (mr.inlier_ratio >= cfg_.min_inlier_ratio) {
        frame_best_score = std::max(frame_best_score, mr.score);
      }
    } else {
      h.ema_score = (h.eval_count == 0) ? 0.0 : (0.7 * h.ema_score);
      ++h.eval_count;
    }
    const bool ok = valid_match &&
      mr.score >= cfg_.hyp_min_score &&
      mr.inlier_ratio >= cfg_.hyp_min_inlier;
    evals[i].ok = ok;
    if (ok) {
      h.pose = Pose2(mr.pose.x, mr.pose.y, wrap_angle(mr.pose.theta));
      h.last_score = mr.score;
      ++h.confirm_count;
      if (h.confirm_count >= req_frames && mr.score > best_score) {
        best_score = mr.score;
        best_idx = static_cast<int>(i);
      }
    } else {
      if (valid_match && mr.score >= cfg_.min_score &&
        mr.inlier_ratio >= cfg_.min_inlier_ratio)
      {
        h.pose = Pose2(mr.pose.x, mr.pose.y, wrap_angle(mr.pose.theta));
      }
      // 滞回退火：单帧偶发遮挡不直接清零，防低分次优对称峰趁虚抢先晋升
      h.confirm_count = std::max(0, h.confirm_count - 1);
    }
  }

  auto prune_dead_hypotheses = [&]() {
      hypotheses_.erase(
        std::remove_if(
          hypotheses_.begin(), hypotheses_.end(),
          [&](const Hypothesis & h) {
            return h.eval_count >= 4 && h.confirm_count == 0 &&
                   h.ema_score < cfg_.min_score;
          }),
        hypotheses_.end());
    };

  // 多峰竞争保护：若当前帧存在显著更高分的另一竞争假设（差值 >= 0.08），
  // 则暂缓低分假设抢跑晋升，继续观察确认。
  if (best_idx < 0 ||
    (hypotheses_.size() > 1 && frame_best_score >= best_score + 0.08))
  {
    prune_dead_hypotheses();
    return false;
  }

  const auto is_distinct = [](const Pose2 & a, const Pose2 & b) {
      return std::hypot(a.x - b.x, a.y - b.y) >= 0.8 ||
             std::abs(wrap_angle(a.theta - b.theta)) >= 0.35;
    };
  const auto is_far_from_anchor = [&](const Pose2 & p) {
      return std::hypot(p.x - last_pose_.x, p.y - last_pose_.y) > cfg_.global_jump_min_dist ||
             std::abs(wrap_angle(p.theta - last_pose_.theta)) > 0.8;
    };

  bool local_reanchored = false;
  MatchResult best_mr = evals[best_idx].mr;

  // 自动恢复连续性保护（!manual_global_ && first_fix_）：
  // 拟晋升为远距离/反朝向假设时，优先核验锚点附近是否存在健康局部匹配；若存在且远端分差
  // 未超过跳变裕度，则抑制跨区瞬移，改由局部连续假设/重锚定接管。
  if (!manual_global_ && first_fix_ && is_far_from_anchor(best_mr.pose)) {
    int local_idx = -1;
    double local_best_score = -1.0;
    for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
      if (!evals[i].valid || is_far_from_anchor(evals[i].mr.pose)) {
        continue;
      }
      const auto & m = evals[i].mr;
      if (m.score >= cfg_.min_score && m.inlier_ratio >= cfg_.min_inlier_ratio &&
        m.score > local_best_score)
      {
        local_best_score = m.score;
        local_idx = static_cast<int>(i);
      }
    }
    if (local_idx >= 0 && best_score < local_best_score + cfg_.global_jump_score_margin) {
      if (evals[local_idx].ok && hypotheses_[local_idx].confirm_count >= req_frames) {
        best_idx = local_idx;
        best_score = local_best_score;
        best_mr = evals[local_idx].mr;
        local_reanchored = true;
      } else {
        prune_dead_hypotheses();
        return false;
      }
    } else {
      MatchResult local_mr = matcher_->match(
        *field_, last_pose_, scan, cfg_.match_opt);
      const bool local_valid = local_mr.valid &&
        is_finite_pose(local_mr.pose) &&
        std::isfinite(local_mr.score) &&
        std::isfinite(local_mr.inlier_ratio);
      if (local_valid) {
        local_mr.score = effective_match_score(
          *field_, local_mr.pose, scan, local_mr.score, cfg_.use_visibility_check);
      }
      const bool local_healthy = local_valid &&
        local_mr.score >= cfg_.min_score &&
        local_mr.inlier_ratio >= cfg_.min_inlier_ratio;
      if (local_healthy && best_score < local_mr.score + cfg_.global_jump_score_margin) {
        best_mr = local_mr;
        local_reanchored = true;
      }
    }
  }

  // 对称多峰歧义保护：若非本地连续重锚且存在空间分离的近似平手峰（分差 < hyp_ambiguity_margin），
  // 禁止 2 帧快进晋升，至少观测 hyp_ambiguous_confirm_frames 帧等待几何对称打破。
  if (!local_reanchored && hypotheses_.size() > 1) {
    double competing_score = -1.0;
    double competing_ema = -1.0;
    for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
      if (static_cast<int>(i) == best_idx || !evals[i].ok) {
        continue;
      }
      if (is_distinct(evals[i].mr.pose, best_mr.pose)) {
        competing_score = std::max(competing_score, evals[i].mr.score);
        competing_ema = std::max(competing_ema, hypotheses_[i].ema_score);
      }
    }
    const bool ambiguous = competing_score >= 0.0 &&
      (best_score - competing_score) < cfg_.hyp_ambiguity_margin;
    if (ambiguous) {
      const int c = hypotheses_[best_idx].confirm_count;
      if (c < ambig_frames) {
        prune_dead_hypotheses();
        return false;
      }
      if (!manual_global_ && first_fix_ && is_far_from_anchor(best_mr.pose) &&
        hypotheses_[best_idx].ema_score < competing_ema + 0.02 &&
        c < ambig_frames * 2)
      {
        prune_dead_hypotheses();
        return false;
      }
    }
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
  manual_global_ = false;
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

bool LocalizerCore::handle_recovery_scan(const Scan & active_scan, CoreOutput & out)
{
  Pose2 delta{};
  bool imu_used = false;
  double dt_imu = 0.0;
  compute_motion_delta(delta, imu_used, dt_imu);
  if (first_fix_) {
    last_pose_ = last_pose_ * delta;
  }
  if (!hypotheses_.empty()) {
    propagate_hypotheses(delta);
  }

  if (!hypotheses_.empty()) {
    if (try_promote_hypotheses(active_scan, out)) {
      return true;
    }
    // 仅当存在本轮匹配健康（last_score >= min_score 且 confirm_count > 0）的活跃竞争假设时，
    // 才暂缓重跑全局搜索；若旧队列全为失效残留假设，立即清空并回退全局搜索。
    bool has_live_competing = false;
    for (const auto & h : hypotheses_) {
      if (h.confirm_count > 0 || h.last_score >= cfg_.min_score) {
        has_live_competing = true;
        break;
      }
    }
    if (has_live_competing) {
      return true;
    }
    if (global_multi_fn_ || builtin_global_enabled_ || global_fn_) {
      clear_hypotheses();
    }
  }

  if (global_multi_fn_ || builtin_global_enabled_) {
    const GlobalResult gres = global_multi_fn_ ?
      global_multi_fn_(active_scan) :
      global_correlative(
        *field_, active_scan,
        builtin_beams_sub_, builtin_ang_step_, builtin_n_hyps_);
    if (gres.valid && is_finite_pose(gres.pose)) {
      std::vector<Pose2> raw_cands;
      raw_cands.reserve(1 + gres.alternates.size());
      raw_cands.push_back(gres.pose);
      for (const auto & alt : gres.alternates) {
        if (is_finite_pose(alt)) {
          raw_cands.push_back(alt);
        }
      }

      MatchOptions cand_opt = cfg_.match_opt;
      cand_opt.window_lin = std::max(cand_opt.window_lin, 0.20);
      cand_opt.window_ang = std::max(cand_opt.window_ang, 0.15);

      struct EvalCand
      {
        Pose2 pose;
        double eff_score{0.0};
        double inlier{0.0};
        Cov3 cov{Cov3::Identity() * 0.04};
        bool has_cov{false};
      };
      std::vector<EvalCand> cands;
      cands.reserve(raw_cands.size());

      for (std::size_t i = 0; i < raw_cands.size(); ++i) {
        const Pose2 & p0 = raw_cands[i];
        const MatchResult mr = matcher_->match(*field_, p0, active_scan, cand_opt);
        EvalCand ec;
        if (mr.valid && is_finite_pose(mr.pose) &&
          std::isfinite(mr.score) && std::isfinite(mr.inlier_ratio))
        {
          ec.pose = Pose2(mr.pose.x, mr.pose.y, wrap_angle(mr.pose.theta));
          ec.eff_score = effective_match_score(
            *field_, ec.pose, active_scan, mr.score, cfg_.use_visibility_check);
          ec.inlier = mr.inlier_ratio;
          if (is_valid_spd_cov3(mr.covariance)) {
            ec.cov = 0.5 * (mr.covariance + mr.covariance.transpose());
            ec.has_cov = true;
          }
        } else if (i == 0 && std::isfinite(gres.score)) {
          ec.pose = Pose2(p0.x, p0.y, wrap_angle(p0.theta));
          ec.eff_score = effective_match_score(
            *field_, ec.pose, active_scan, gres.score, cfg_.use_visibility_check);
          ec.inlier = compute_inlier_ratio(
            *field_, ec.pose, active_scan, cfg_.match_opt.inlier_gate);
        } else {
          continue;
        }
        if (ec.eff_score > 0.10) {
          cands.push_back(ec);
        }
      }

      std::sort(cands.begin(), cands.end(),
        [](const EvalCand & a, const EvalCand & b) {
          return a.eff_score > b.eff_score;
        });

      std::vector<EvalCand> dedup;
      dedup.reserve(cands.size());
      for (const auto & c : cands) {
        bool same = false;
        for (const auto & k : dedup) {
          if (std::hypot(c.pose.x - k.pose.x, c.pose.y - k.pose.y) < 0.50 &&
            std::abs(wrap_angle(c.pose.theta - k.pose.theta)) < 0.35)
          {
            same = true;
            break;
          }
        }
        if (!same) {
          dedup.push_back(c);
        }
      }

      if (!dedup.empty()) {
        const auto & best = dedup[0];

        if (first_fix_ && !manual_global_) {
          const double jump_dist = std::hypot(
            best.pose.x - last_pose_.x, best.pose.y - last_pose_.y);
          const double jump_ang = std::abs(
            wrap_angle(best.pose.theta - last_pose_.theta));
          if (jump_dist > cfg_.global_jump_min_dist || jump_ang > 0.8) {
            MatchResult local_mr = matcher_->match(
              *field_, last_pose_, active_scan, cfg_.match_opt);
            const bool local_valid = local_mr.valid &&
              is_finite_pose(local_mr.pose) &&
              std::isfinite(local_mr.score) &&
              std::isfinite(local_mr.inlier_ratio);
            if (local_valid) {
              local_mr.score = effective_match_score(
                *field_, local_mr.pose, active_scan,
                local_mr.score, cfg_.use_visibility_check);
            }
            const bool local_healthy = local_valid &&
              local_mr.score >= cfg_.min_score &&
              local_mr.inlier_ratio >= cfg_.min_inlier_ratio;
            if (local_healthy &&
              best.eff_score < local_mr.score + cfg_.global_jump_score_margin)
            {
              last_pose_ = Pose2(
                local_mr.pose.x, local_mr.pose.y, wrap_angle(local_mr.pose.theta));
              state_ = LocState::TRACKING;
              reject_streak_ = 0;
              stable_tracking_streak_ = 0;
              anomaly_window_.clear();
              std::vector<Pose2> far_hyps;
              for (const auto & d : dedup) {
                far_hyps.push_back(d.pose);
              }
              add_hypotheses(far_hyps);

              out.pose = last_pose_;
              out.covariance = last_cov();
              out.state = state_;
              out.accepted = true;
              out.score = local_mr.score;
              out.inlier_ratio = local_mr.inlier_ratio;
              out.reject_streak = 0;
              return true;
            }
          }
        }

        double competing_eff = -1.0;
        for (std::size_t i = 1; i < dedup.size(); ++i) {
          const bool distinct =
            std::hypot(dedup[i].pose.x - best.pose.x, dedup[i].pose.y - best.pose.y) >= 0.8 ||
            std::abs(wrap_angle(dedup[i].pose.theta - best.pose.theta)) >= 0.35;
          if (distinct && dedup[i].eff_score >= cfg_.min_score &&
            dedup[i].inlier >= cfg_.min_inlier_ratio * 0.85)
          {
            competing_eff = std::max(competing_eff, dedup[i].eff_score);
          }
        }
        const bool ambiguous = competing_eff >= 0.0 &&
          (best.eff_score - competing_eff) < cfg_.hyp_ambiguity_margin;

        if (ambiguous) {
          clear_hypotheses();
          const double keep_floor = std::max(cfg_.min_score, best.eff_score - 0.20);
          for (const auto & d : dedup) {
            if (d.eff_score < keep_floor || hypotheses_.size() >= cfg_.max_hypotheses) {
              continue;
            }
            Hypothesis h;
            h.pose = d.pose;
            h.confirm_count = (d.eff_score >= cfg_.hyp_min_score &&
              d.inlier >= cfg_.hyp_min_inlier) ? 1 : 0;
            h.last_score = d.eff_score;
            h.ema_score = d.eff_score;
            h.eval_count = 1;
            hypotheses_.push_back(h);
          }
          if (has_odom_ && !first_fix_) {
            odom_at_last_scan_ = last_odom_;
          }
          state_ = LocState::GLOBAL;
          out.pose = last_pose_;
          out.covariance = last_cov();
          out.state = state_;
          out.accepted = false;
          out.score = best.eff_score;
          out.inlier_ratio = best.inlier;
          out.reject_streak = reject_streak_;
          return true;
        }

        last_pose_ = best.pose;
        P4_ = Cov4::Zero();
        P4_.topLeftCorner<3, 3>() = best.has_cov ? best.cov : (Cov3::Identity() * 0.04);
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
        manual_global_ = false;
        reject_streak_ = 0;
        stable_tracking_streak_ = 0;
        first_fix_ = true;
        clear_hypotheses();
        anomaly_window_.clear();

        out.pose = last_pose_;
        out.covariance = last_cov();
        out.state = state_;
        out.accepted = true;
        out.score = best.eff_score;
        out.inlier_ratio = best.inlier;
        out.reject_streak = 0;
        return true;
      }
    }
    return true;
  }

  if (global_fn_) {
    Pose2 g{};
    double sc = 0.0;
    if (global_fn_(active_scan, g, sc) && is_finite_pose(g) && std::isfinite(sc)) {
      if (first_fix_ && !manual_global_) {
        const double jump_dist = std::hypot(g.x - last_pose_.x, g.y - last_pose_.y);
        if (jump_dist > cfg_.global_jump_min_dist) {
          const MatchResult local_mr = matcher_->match(
            *field_, last_pose_, active_scan, cfg_.match_opt);
          const bool local_healthy = local_mr.valid &&
            is_finite_pose(local_mr.pose) &&
            std::isfinite(local_mr.score) &&
            std::isfinite(local_mr.inlier_ratio) &&
            local_mr.score >= cfg_.min_score &&
            local_mr.inlier_ratio >= cfg_.min_inlier_ratio;
          if (local_healthy && sc < local_mr.score + cfg_.global_jump_score_margin) {
            last_pose_ = Pose2(
              local_mr.pose.x, local_mr.pose.y, wrap_angle(local_mr.pose.theta));
            state_ = LocState::TRACKING;
            reject_streak_ = 0;
            stable_tracking_streak_ = 0;
            anomaly_window_.clear();
            add_hypotheses({g});

            out.pose = last_pose_;
            out.covariance = last_cov();
            out.state = state_;
            out.accepted = true;
            out.score = local_mr.score;
            out.inlier_ratio = local_mr.inlier_ratio;
            out.reject_streak = 0;
            return true;
          }
        }
      }

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
      manual_global_ = false;
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
        *field_, last_pose_, active_scan, cfg_.match_opt.inlier_gate);
      out.reject_streak = 0;
    }
  }
  return true;
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
    const double drot = std::abs(delta.theta);
    q_trans = dtrans * cfg_.odom_trans_var_per_m + drot * cfg_.odom_trans_var_per_rad;
    q_rot = dtrans * cfg_.odom_rot_var_per_m +
      drot * (imu_used ? cfg_.imu_rot_var_per_rad : cfg_.odom_rot_var_per_rad);
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
  // 若匹配本身高分高内点率且协方差有效，仅因马氏门控过紧被拒（典型场景：静止/原地旋转时
  // dtrans≈0 导致 P4_pred 过小），适度松弛先验协方差以打破马氏锁死。
  if (mr_finite && gate_inlier && gate_score && gate_cov && !gate_mahal) {
    P4_.topLeftCorner<3, 3>() *= 2.5;
    P4_(0, 0) = std::min(P4_(0, 0) + 4e-3, 0.25);
    P4_(1, 1) = std::min(P4_(1, 1) + 4e-3, 0.25);
    P4_(2, 2) = std::min(P4_(2, 2) + 5e-4, 0.10);
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
    if (try_promote_hypotheses(*active_scan, out)) {
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
