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

#pragma once
// ============================================================================
// LocalizerCore 内部共享观测与校验工具（供 localizer_core.cpp 与 hypothesis_tracker.cpp 共用）
// ============================================================================
#include <algorithm>
#include <cmath>

#include "scan_localizer/localizer_core.hpp"

namespace sloc
{
namespace detail
{

inline bool is_finite_pose(const Pose2 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}

inline bool is_valid_spd_cov3(const Cov3 & c)
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

inline double compute_inlier_ratio(
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

// 纯无状态正交观测评估：物理层不感知 TRACKING/GLOBAL 状态，也不在原地改写几何分
struct ScanObservation
{
  double geom_score{0.0};           // 原始几何贴合度
  double geom_inlier{0.0};          // 原始几何内点率
  double static_score{0.0};         // 扣除前景动态遮挡 (z_short) 后的静态贴合度
  double static_inlier{0.0};        // 扣除前景动态遮挡 (z_short) 后的静态内点率
  double wall_visibility{1.0};      // 硬物理一致性（穿墙/穿墙入未知）∈ [0.05, 1.0]
  double frontier_visibility{1.0};  // 零穿墙地图前沿覆盖度 ∈ [0.62, 1.0]
  double short_ratio{0.0};
  double frontier_ratio{0.0};

  // 全局候选排序质量：禁用 z_short 提分防错位洗白，按硬一致性与覆盖度综合排序
  double global_rank_score() const
  {
    return geom_score * wall_visibility * frontier_visibility;
  }
};

inline ScanObservation evaluate_observation(
  const MapField & field, const Pose2 & pose, const Scan & scan,
  double raw_score, double raw_inlier, double inlier_gate, bool check_vis)
{
  ScanObservation obs;
  obs.geom_score = raw_score;
  obs.geom_inlier = raw_inlier;
  obs.static_score = raw_score;
  obs.static_inlier = raw_inlier;
  if (!check_vis || scan.points.size() < 16) {
    return obs;
  }
  const BeamPhysicsStats bp = evaluate_beam_physics(
    field, pose, scan, inlier_gate, true);
  obs.wall_visibility = bp.wall_visibility;
  obs.frontier_visibility = bp.frontier_visibility;
  obs.static_score = std::max(raw_score, bp.static_score);
  obs.static_inlier = std::max(raw_inlier, bp.static_inlier_ratio);
  obs.short_ratio = bp.short_ratio;
  obs.frontier_ratio = bp.frontier_ratio;
  return obs;
}

}  // namespace detail
}  // namespace sloc
