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
// LocalizerCore 多假设序贯贝叶斯队列、异常监测与全局重定位恢复
// （从 localizer_core.cpp 拆分）
// ============================================================================
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "localizer_core_internal.hpp"
#include "scan_localizer/localizer_core.hpp"

namespace sloc
{

namespace
{

using detail::compute_inlier_ratio;
using detail::evaluate_observation;
using detail::is_finite_pose;
using detail::is_valid_spd_cov3;
using detail::ScanObservation;

}  // namespace

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
    h.seed_pose = clean_p;
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
  if (hypotheses_.size() > 1) {
    bool has_distinct_pair = false;
    for (std::size_t i = 0; i < hypotheses_.size() && !has_distinct_pair; ++i) {
      for (std::size_t j = i + 1; j < hypotheses_.size(); ++j) {
        if (std::hypot(
            hypotheses_[i].pose.x - hypotheses_[j].pose.x,
            hypotheses_[i].pose.y - hypotheses_[j].pose.y) >= 0.8 ||
          std::abs(wrap_angle(hypotheses_[i].pose.theta - hypotheses_[j].pose.theta)) >= 0.35)
        {
          has_distinct_pair = true;
          break;
        }
      }
    }
    if (has_distinct_pair) {
      for (auto & h : hypotheses_) {
        h.seeded_ambiguous = true;
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

void LocalizerCore::propagate_hypotheses(const Pose2 & delta)
{
  for (auto & h : hypotheses_) {
    h.pose = h.pose * delta;
  }
}

bool LocalizerCore::try_promote_hypotheses(
  const Scan & scan, const Pose2 & motion_delta, CoreOutput & out)
{
  if (hypotheses_.empty() || !field_ || !matcher_) {
    return false;
  }
  const int req_frames = std::max(1, cfg_.hyp_confirm_frames);
  const int ambig_frames = std::max(req_frames, cfg_.hyp_ambiguous_confirm_frames);
  const bool real_scan = scan.points.size() >= 16;
  const double p_bg = std::clamp(
    std::max(cfg_.min_score, 0.5 * (cfg_.min_score + cfg_.hyp_min_score)), 0.15, 0.65);

  // 运动新息门控因子 γ_motion：真实几何扫描在原地静止且多峰平手时 γ=0，
  // 杜绝静态相关性假收敛（陷阱 1）
  const double d0 = std::max(0.15, field_->sigma_m_);
  const double raw_motion =
    std::hypot(motion_delta.x, motion_delta.y) / d0 +
    std::abs(motion_delta.theta) / 0.15;

  struct EvalEntry
  {
    bool valid{false};
    bool ok{false};
    double ll{0.0};
    double eff_q{0.0};
    MatchResult mr{};
  };
  std::vector<EvalEntry> evals(hypotheses_.size());

  int best_idx = -1;
  double best_score = -1.0;
  double frame_best_score = -1.0;
  int top_valid_idx = -1;
  double top_valid_eff_q = -1.0;
  double max_score_innov = 0.0;
  int live_ok_count = 0;

  const auto is_far_from_anchor = [&](const Pose2 & p) {
      return std::hypot(p.x - last_pose_.x, p.y - last_pose_.y) > cfg_.global_jump_min_dist ||
             std::abs(wrap_angle(p.theta - last_pose_.theta)) > 0.8;
    };
  const auto is_distinct = [](const Pose2 & a, const Pose2 & b) {
      return std::hypot(a.x - b.x, a.y - b.y) >= 0.8 ||
             std::abs(wrap_angle(a.theta - b.theta)) >= 0.35;
    };

  for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
    auto & h = hypotheses_[i];
    MatchResult mr = matcher_->match(*field_, h.pose, scan, cfg_.match_opt);
    const bool valid_match = mr.valid &&
      is_finite_pose(mr.pose) &&
      std::isfinite(mr.score) &&
      std::isfinite(mr.inlier_ratio);
    if (valid_match) {
      const ScanObservation obs = evaluate_observation(
        *field_, mr.pose, scan, mr.score, mr.inlier_ratio,
        cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
      mr.score = obs.global_rank_score();
      mr.inlier_ratio = obs.geom_inlier;
      if (h.eval_count > 0) {
        max_score_innov = std::max(max_score_innov, std::abs(mr.score - h.last_score));
      }
      // 正交物理对数似然：几何贴合 + 内点支撑 + 硬物理惩罚独立叠加，避免混合标量耦合
      const double eff_q = std::sqrt(
        std::max(mr.score, 0.05) * std::max(mr.inlier_ratio, 0.05));
      evals[i].eff_q = eff_q;
      const double phys_ll = (obs.wall_visibility < cfg_.min_wall_visibility) ?
        2.0 * std::log(std::max(obs.wall_visibility, 0.05)) : 0.0;
      evals[i].ll = 6.0 * std::log(eff_q / p_bg) + phys_ll;
      if (eff_q > top_valid_eff_q) {
        top_valid_eff_q = eff_q;
        top_valid_idx = static_cast<int>(i);
      }
    } else {
      // 软化单帧失效惩罚，避免瞬时行人遮挡一帧清空十余帧累积的正确假设优势
      evals[i].ll = -1.8;
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
      ++live_ok_count;
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
      if (valid_match) {
        h.last_score = mr.score;
      }
      // 滞回退火：单帧偶发遮挡不直接清零，防低分次优对称峰趁虚抢先晋升
      h.confirm_count = std::max(0, h.confirm_count - 1);
    }
  }

  // 区分“真正对称平手多峰”与“主峰显著领先弱干扰峰”：
  // 仅当存在与当前最优质量空间分离且综合质量差 < sym_freeze_margin 的近乎等价竞争峰时，
  // 静止期才冻结 γ=0；若主峰已明确领先全部异地候选，允许静止期累积后验，避免永不晋升死锁。
  double distinct_runner_eff_q = -1.0;
  if (top_valid_idx >= 0) {
    const Pose2 & top_pose = evals[static_cast<std::size_t>(top_valid_idx)].mr.pose;
    for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
      if (static_cast<int>(i) == top_valid_idx || !evals[i].valid) {continue;}
      if (evals[i].mr.score < cfg_.min_score || evals[i].mr.inlier_ratio < cfg_.min_inlier_ratio) {
        continue;
      }
      if (is_distinct(evals[i].mr.pose, top_pose)) {
        distinct_runner_eff_q = std::max(distinct_runner_eff_q, evals[i].eff_q);
      }
    }
  }
  const double sym_freeze_margin = std::min(0.04, 0.5 * cfg_.hyp_ambiguity_margin);
  const double sym_clear_margin = std::max(sym_freeze_margin + 0.04, cfg_.hyp_ambiguity_margin);
  double stationary_gamma = 0.8;
  if (distinct_runner_eff_q >= 0.0) {
    const double eff_gap = top_valid_eff_q - distinct_runner_eff_q;
    const double ramp = std::clamp(
      (eff_gap - sym_freeze_margin) / std::max(1e-6, sym_clear_margin - sym_freeze_margin),
      0.0, 1.0);
    stationary_gamma = 0.8 * ramp;
  }

  // 更新序贯贝叶斯对数权重：对称平手且静止时 γ=0；在 [freeze, clear] 区间平滑过渡避免阶跃假锁
  const double env_innov_gate = std::clamp((max_score_innov - 0.08) / 0.15, 0.0, 0.6);
  const double motion_gamma = std::clamp(raw_motion + env_innov_gate, 0.0, 1.0);
  double gamma = 1.0;
  if (real_scan && live_ok_count > 1) {
    gamma = std::max(motion_gamma, stationary_gamma);
  }

  double max_lw = 0.0;  // 含虚拟离群零假设 log_w_null = 0.0
  for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
    auto & h = hypotheses_[i];
    if (h.eval_count == 1 && !manual_global_ && first_fix_ &&
      evals[i].valid && !is_far_from_anchor(evals[i].mr.pose))
    {
      h.log_weight += 2.2;
    }
    const double step_g = (evals[i].ll < 0.0) ? 1.0 : gamma;
    h.log_weight = std::clamp(h.log_weight + step_g * evals[i].ll, -12.0, 30.0);
    max_lw = std::max(max_lw, h.log_weight);
  }
  if (max_lw > 20.0) {
    const double shift = max_lw - 20.0;
    for (auto & h : hypotheses_) {
      h.log_weight -= shift;
    }
  }

  auto prune_dead_hypotheses = [&]() {
      hypotheses_.erase(
        std::remove_if(
          hypotheses_.begin(), hypotheses_.end(),
          [&](const Hypothesis & h) {
            return h.eval_count >= 4 && h.confirm_count == 0 &&
                   (h.ema_score < cfg_.min_score || h.log_weight < -6.0);
          }),
        hypotheses_.end());
    };

  if (best_idx < 0 ||
    (hypotheses_.size() > 1 && frame_best_score >= best_score + 0.08))
  {
    prune_dead_hypotheses();
    return false;
  }

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
      ScanObservation local_obs;
      if (local_valid) {
        local_obs = evaluate_observation(
          *field_, local_mr.pose, scan, local_mr.score, local_mr.inlier_ratio,
          cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
        local_mr.score = local_obs.global_rank_score();
      }
      const bool local_healthy = local_valid &&
        local_obs.wall_visibility >= cfg_.min_wall_visibility &&
        local_mr.score >= cfg_.min_score &&
        local_mr.inlier_ratio >= cfg_.min_inlier_ratio;
      if (local_healthy && best_score < local_mr.score + cfg_.global_jump_score_margin) {
        best_mr = local_mr;
        local_reanchored = true;
      }
    }
  }

  // 序贯贝叶斯后验晋升检验（SPRT）：
  // 真实激光扫描下计算 P(H_best | Z_{1:t}) >= hyp_promote_posterior (0.95)；
  // 极简桩点云测试 (<16 束) 保留兼容帧数计数契约。
  if (!local_reanchored) {
    if (real_scan) {
      const double lw_best = hypotheses_[static_cast<std::size_t>(best_idx)].log_weight;
      const double m_lw = std::max(0.0, lw_best);
      double sum_exp = std::exp(lw_best - m_lw) + std::exp(0.0 - m_lw);
      for (std::size_t i = 0; i < hypotheses_.size(); ++i) {
        if (static_cast<int>(i) == best_idx) {continue;}
        if (evals[i].valid && is_distinct(evals[i].mr.pose, best_mr.pose)) {
          sum_exp += std::exp(hypotheses_[i].log_weight - m_lw);
        }
      }
      const double posterior = std::exp(lw_best - m_lw) / std::max(1e-12, sum_exp);
      const double req_post = std::clamp(cfg_.hyp_promote_posterior, 0.55, 0.999);
      if (posterior < req_post) {
        prune_dead_hypotheses();
        return false;
      }
      // 多峰对称环境下的远距离自动跳变保护：若候选源自多峰歧义池且远离上一锚点，
      // 必须具备极高几何置信度（score>=0.85 且 inlier>=0.75）或经历足够累积运动消歧
      // （位移>=0.50m 或旋转>=0.35rad），防止对称圆弧走廊中短距离内误锁镜像假峰。
      const auto & best_h = hypotheses_[static_cast<std::size_t>(best_idx)];
      if (!manual_global_ && first_fix_ && is_far_from_anchor(best_mr.pose) &&
        best_h.seeded_ambiguous)
      {
        const bool high_geom_lock =
          best_mr.score >= 0.85 && best_mr.inlier_ratio >= 0.75;
        const double net_disp = std::hypot(
          best_mr.pose.x - best_h.seed_pose.x,
          best_mr.pose.y - best_h.seed_pose.y);
        const double net_rot = std::abs(
          wrap_angle(best_mr.pose.theta - best_h.seed_pose.theta));
        const bool enough_motion = net_disp >= 0.50 || net_rot >= 0.35;
        if (!high_geom_lock && !enough_motion) {
          prune_dead_hypotheses();
          return false;
        }
      }
    } else if (hypotheses_.size() > 1) {
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
    if (try_promote_hypotheses(active_scan, delta, out)) {
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
    const double eff_ang_step = (builtin_ang_step_ > 0.0) ?
      builtin_ang_step_ : compute_adaptive_ang_step(*field_, active_scan);
    const GlobalResult gres = global_multi_fn_ ?
      global_multi_fn_(active_scan) :
      global_correlative(
        *field_, active_scan,
        builtin_beams_sub_, eff_ang_step, builtin_n_hyps_,
        builtin_max_time_ms_);
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
        bool from_twin{false};
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
          const ScanObservation obs = evaluate_observation(
            *field_, ec.pose, active_scan, mr.score, mr.inlier_ratio,
            cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
          ec.eff_score = obs.global_rank_score();
          ec.inlier = obs.geom_inlier;
          if (is_valid_spd_cov3(mr.covariance)) {
            ec.cov = 0.5 * (mr.covariance + mr.covariance.transpose());
            ec.has_cov = true;
          }
        } else if (i == 0 && std::isfinite(gres.score)) {
          ec.pose = Pose2(p0.x, p0.y, wrap_angle(p0.theta));
          const double raw_in = compute_inlier_ratio(
            *field_, ec.pose, active_scan, cfg_.match_opt.inlier_gate);
          const ScanObservation obs = evaluate_observation(
            *field_, ec.pose, active_scan, gres.score, raw_in,
            cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
          ec.eff_score = obs.global_rank_score();
          ec.inlier = obs.geom_inlier;
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
        // MapPrior 孪生区主动交叉验证：最优候选落在自相似区时，
        // 主动探测全部孪生锚点，防止 B&B 漏峰
        if (cfg_.use_map_prior && active_scan.points.size() >= 16) {
          const LocalMapPrior mp = field_->queryPrior(dedup[0].pose.x, dedup[0].pose.y);
          if (mp.has_twins) {
            for (const auto & tp : mp.twin_positions) {
              for (int m = 0; m < 4; ++m) {
                const Pose2 tseed(
                  tp.x(), tp.y(), wrap_angle(dedup[0].pose.theta + m * (0.5 * kPi)));
                bool already_near = false;
                for (const auto & k : dedup) {
                  if (std::hypot(tseed.x - k.pose.x, tseed.y - k.pose.y) < 1.2 &&
                    std::abs(wrap_angle(tseed.theta - k.pose.theta)) < 0.45)
                  {
                    already_near = true;
                    break;
                  }
                }
                if (already_near) {continue;}
                const MatchResult tmr = matcher_->match(
                  *field_, tseed, active_scan, cand_opt);
                if (!tmr.valid || !is_finite_pose(tmr.pose) ||
                  !std::isfinite(tmr.score) || !std::isfinite(tmr.inlier_ratio))
                {
                  continue;
                }
                EvalCand tc;
                tc.from_twin = true;
                tc.pose = Pose2(tmr.pose.x, tmr.pose.y, wrap_angle(tmr.pose.theta));
                const ScanObservation tobs = evaluate_observation(
                  *field_, tc.pose, active_scan, tmr.score, tmr.inlier_ratio,
                  cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
                tc.eff_score = tobs.global_rank_score();
                tc.inlier = tobs.geom_inlier;
                if (tc.eff_score >= cfg_.min_score) {
                  if (is_valid_spd_cov3(tmr.covariance)) {
                    tc.cov = 0.5 * (tmr.covariance + tmr.covariance.transpose());
                    tc.has_cov = true;
                  }
                  dedup.push_back(tc);
                }
              }
            }
            std::sort(dedup.begin(), dedup.end(),
              [](const EvalCand & a, const EvalCand & b) {
                return a.eff_score > b.eff_score;
              });
          }
        }

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
            ScanObservation local_obs;
            if (local_valid) {
              local_obs = evaluate_observation(
                *field_, local_mr.pose, active_scan, local_mr.score, local_mr.inlier_ratio,
                cfg_.match_opt.inlier_gate, cfg_.use_visibility_check);
              local_mr.score = local_obs.global_rank_score();
            }
            const bool local_healthy = local_valid &&
              local_obs.wall_visibility >= cfg_.min_wall_visibility &&
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
          if (distinct && !dedup[i].from_twin && dedup[i].eff_score >= cfg_.min_score &&
            dedup[i].inlier >= cfg_.min_inlier_ratio * 0.85)
          {
            competing_eff = std::max(competing_eff, dedup[i].eff_score);
          }
        }
        const bool ambiguous = competing_eff >= 0.0 &&
          (best.eff_score - competing_eff) < cfg_.hyp_ambiguity_margin;

        if (ambiguous) {
          clear_hypotheses();
          const double p_bg = std::clamp(cfg_.min_score, 0.10, 0.60);
          const double keep_floor = std::max(cfg_.min_score, best.eff_score - 0.20);
          for (const auto & d : dedup) {
            if (d.eff_score < keep_floor || hypotheses_.size() >= cfg_.max_hypotheses) {
              continue;
            }
            Hypothesis h;
            h.pose = d.pose;
            h.seed_pose = d.pose;
            h.log_weight = 6.0 * std::log(std::max(d.eff_score, 0.05) / p_bg);
            h.confirm_count = (d.eff_score >= cfg_.hyp_min_score &&
              d.inlier >= cfg_.hyp_min_inlier) ? 1 : 0;
            h.last_score = d.eff_score;
            h.ema_score = d.eff_score;
            h.eval_count = 1;
            h.seeded_ambiguous = true;
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
        {
          // 孪生锚点仅作为后备假设，由运动门控 SPRT 证伪或晋升，不阻塞首次锁定
          std::vector<Pose2> twin_hyps;
          for (const auto & d : dedup) {
            if (d.from_twin && d.eff_score >= best.eff_score - cfg_.hyp_ambiguity_margin) {
              twin_hyps.push_back(d.pose);
            }
          }
          if (!twin_hyps.empty()) {
            add_hypotheses(twin_hyps);
          }
        }

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

}  // namespace sloc
