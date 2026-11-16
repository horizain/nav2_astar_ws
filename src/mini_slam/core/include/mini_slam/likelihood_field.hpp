#pragma once
// ============================================================================
// Likelihood Field：占据栅格 → 模糊得分场。
// 这是 correlative scan matching 的"底图"：把参考扫描的端点涂进栅格，
// 两遍分离 box blur 得到平滑得分面，再归一化峰值 = 1，
// 于是任意位姿下"逐点查表求平均"天然落在 [0,1]，作为匹配 response。
// 对应 libkarto 的 CorrelationGrid / ScanMatcher::ComputeCorrelation。
// ============================================================================
#include <expected>
#include <span>
#include <string>
#include <vector>

#include "mini_slam/occupancy_grid.hpp"

namespace mini_slam {

// 参考帧中的一帧扫描（位姿 + 传感器系点云视图）
struct FramedScan {
  Pose2 pose;  // 在场坐标系中的位姿
  std::span<const Vec2> points;
};

class LikelihoodField {
 public:
  // 由若干参考扫描构建：paint → blur×2 → 归一化
  static std::expected<LikelihoodField, std::string> build(
      std::span<const FramedScan> scans, double resolution, double margin,
      int blur_radius);

  // 一帧查询点云（传感器系）按位姿投影后的平均得分 ∈ [0,1]，出界查 0
  [[nodiscard]] double score(const Pose2& pose,
                             std::span<const Vec2> points) const;

  const Grid& grid() const { return grid_; }

 private:
  Grid grid_;
};

}  // namespace mini_slam
