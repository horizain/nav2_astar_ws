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
// 四态物理波束评估与可见性一致性检验（从 matchers.cpp 拆分）
// ============================================================================
#include <algorithm>
#include <cmath>
#include <vector>

#include "scan_localizer/matchers.hpp"

namespace sloc
{

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

}  // namespace

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

}  // namespace sloc
