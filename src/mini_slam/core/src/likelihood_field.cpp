#include "mini_slam/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mini_slam {
namespace {

// 一维 box blur（running-sum，O(len)）。
// stride = 1 为横向行内遍历；stride = w 为纵向列内遍历（同一函数复用）。
void box_blur_1d(const float* src, float* dst, int len, int stride, int r) {
  double sum = 0.0;
  for (int i = 0; i <= std::min(r, len - 1); ++i) {
    sum += src[static_cast<size_t>(i) * stride];
  }
  for (int x = 0; x < len; ++x) {
    const int lo = std::max(x - r, 0);
    const int hi = std::min(x + r, len - 1);
    dst[static_cast<size_t>(x) * stride] =
        static_cast<float>(sum / static_cast<double>(hi - lo + 1));
    // 滑动窗口：加入 x+r+1，移除 x-r
    if (x + r + 1 < len) sum += src[static_cast<size_t>(x + r + 1) * stride];
    if (x - r >= 0) sum -= src[static_cast<size_t>(x - r) * stride];
  }
}

}  // namespace

std::expected<LikelihoodField, std::string> LikelihoodField::build(
    std::span<const FramedScan> scans, double resolution, double margin,
    int blur_radius) {
  if (scans.empty()) {
    return std::unexpected("LikelihoodField::build: 没有参考扫描");
  }
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(margin) || margin < 0.0) {
    return std::unexpected(
        "LikelihoodField::build: resolution 必须 > 0，margin 必须 >= 0");
  }

  // 1) 所有端点投到场坐标系，求包围盒
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  bool any_valid = false;
  for (const auto& fs : scans) {
    for (const Vec2& p : fs.points) {
      const Vec2 w = transform(fs.pose, p);
      if (!std::isfinite(w.x) || !std::isfinite(w.y)) continue;
      any_valid = true;
      min_x = std::min(min_x, w.x);
      min_y = std::min(min_y, w.y);
      max_x = std::max(max_x, w.x);
      max_y = std::max(max_y, w.y);
    }
  }
  if (!any_valid) {
    return std::unexpected("LikelihoodField::build: 参考扫描没有有效点");
  }

  // 2) 分配栅格（包围盒 + margin）
  const int pad = std::max(1, static_cast<int>(std::ceil(margin / resolution)));
  const int width = std::max(
      1, static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 2 * pad + 1);
  const int height = std::max(
      1, static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 2 * pad + 1);
  Grid grid(resolution, Vec2{min_x - margin, min_y - margin}, width, height);

  // 3) paint：二值标记（占用过即 1），消除点密度偏差
  for (const auto& fs : scans) {
    for (const Vec2& p : fs.points) {
      const Vec2 w = transform(fs.pose, p);
      int cx = 0, cy = 0;
      if (grid.world_to_cell(w, cx, cy)) grid.mark(cx, cy);
    }
  }

  // 4) 两遍分离 box blur（H,V,H,V）≈ 高斯模糊，σ ≈ r·0.8·resolution
  const int r = std::max(0, blur_radius);
  if (r > 0) {
    std::vector<float> tmp(grid.cells().size());
    for (int pass = 0; pass < 2; ++pass) {
      for (int y = 0; y < height; ++y) {
        box_blur_1d(&grid.cells()[static_cast<size_t>(y) * width],
                    &tmp[static_cast<size_t>(y) * width], width, 1, r);
      }
      for (int x = 0; x < width; ++x) {
        box_blur_1d(&tmp[x], &grid.cells()[x], height, width, r);
      }
    }
  }

  // 5) 归一化峰值 = 1 → response ∈ [0,1]
  float peak = 0.0f;
  for (const float v : grid.cells()) peak = std::max(peak, v);
  if (peak <= 0.0f) {
    return std::unexpected("LikelihoodField::build: 模糊后得分为 0");
  }
  for (float& v : grid.cells()) v /= peak;

  LikelihoodField field;
  field.grid_ = std::move(grid);
  return field;
}

double LikelihoodField::score(const Pose2& pose,
                              std::span<const Vec2> points) const {
  if (points.empty()) return 0.0;
  double sum = 0.0;
  int n = 0;
  for (const Vec2& p : points) {
    const Vec2 w = transform(pose, p);
    int cx = 0, cy = 0;
    if (grid_.world_to_cell(w, cx, cy)) {
      sum += grid_.at(cx, cy);
      ++n;
    }
  }
  return n > 0 ? sum / static_cast<double>(n) : 0.0;
}

}  // namespace mini_slam
