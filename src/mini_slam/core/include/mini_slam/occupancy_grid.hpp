#pragma once
// ============================================================================
// 浮点栅格：likelihood field 与 log-odds 地图共用的存储与坐标变换。
// ============================================================================
#include <cmath>
#include <cstddef>
#include <vector>

#include "mini_slam/types.hpp"

namespace mini_slam {

class Grid {
 public:
  Grid() = default;
  Grid(double resolution, Vec2 origin, int width, int height, float init = 0.0f)
      : resolution_(resolution),
        origin_(origin),
        width_(width),
        height_(height),
        cells_(static_cast<size_t>(width) * static_cast<size_t>(height), init) {}

  bool empty() const { return cells_.empty(); }
  double resolution() const { return resolution_; }
  const Vec2& origin() const { return origin_; }
  int width() const { return width_; }
  int height() const { return height_; }
  const std::vector<float>& cells() const { return cells_; }
  std::vector<float>& cells() { return cells_; }

  Vec2 cell_center(int cx, int cy) const {
    return {origin_.x + (cx + 0.5) * resolution_,
            origin_.y + (cy + 0.5) * resolution_};
  }

  // 世界坐标 → 栅格下标；出界返回 false。
  // 约定：格子 cx 覆盖 [origin + cx·res, origin + (cx+1)·res)，中心是
  // origin + (cx+0.5)·res（与 cell_center 严格一致）。
  bool world_to_cell(Vec2 p, int& cx, int& cy) const {
    const double fx = (p.x - origin_.x) / resolution_;
    const double fy = (p.y - origin_.y) / resolution_;
    cx = static_cast<int>(std::floor(fx));
    cy = static_cast<int>(std::floor(fy));
    return cx >= 0 && cy >= 0 && cx < width_ && cy < height_;
  }

  float at(int cx, int cy) const {
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) return 0.0f;
    return cells_[static_cast<size_t>(cy) * width_ + cx];
  }

  void add(int cx, int cy, float v) {
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) return;
    cells_[static_cast<size_t>(cy) * width_ + cx] += v;
  }

  // 二值标记：占用过即 1（不做累加）。
  // 得分场必须用它：近处墙点密、远处墙点稀，累加会让峰值被近处密度
  // 垄断，远处稀疏墙的得分被系统性压低（karto CorrelationGrid 同为二值）。
  void mark(int cx, int cy) {
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) return;
    cells_[static_cast<size_t>(cy) * width_ + cx] = 1.0f;
  }

 private:
  double resolution_{0.05};
  Vec2 origin_;
  int width_{0}, height_{0};
  std::vector<float> cells_;
};

}  // namespace mini_slam
