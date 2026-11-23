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
// MapField 实现 —— Meijster 精确 EDT（双向）→ SDF → 似然 → max-pool 金字塔
// ============================================================================
#include "scan_localizer/map_field.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace sloc
{

namespace
{
// Meijster 精确欧氏距离变换：seeds[i] 为 true 的单元距离为 0（平方距离，单元系）
void edt2d(
  int width, int height, const std::vector<uint8_t> & seeds,
  std::vector<float> & dist2_out)
{
    // 距离值可能达 width+height，平方后超 int32 → 用 int64 防溢出
  const std::int64_t INF = static_cast<std::int64_t>(width) + height + 2;
  std::vector<std::int64_t> g(static_cast<std::size_t>(width) *
    static_cast<std::size_t>(height));
    // Pass 1：逐列扫描
  for (int x = 0; x < width; ++x) {
        // 正向
    for (int y = 0; y < height; ++y) {
      const std::size_t i = static_cast<std::size_t>(y) * width + x;
      g[i] = seeds[i] ? 0 :
        (y > 0 ? std::min(INF, g[i - width] + 1) : INF);
    }
        // 反向
    for (int y = height - 2; y >= 0; --y) {
      const std::size_t i = static_cast<std::size_t>(y) * width + x;
      g[i] = std::min(g[i], g[i + width] + 1);
    }
  }
    // Pass 2：逐行下包络
    // 注意：哨兵必须是“真无穷”（±2^40），不能用距离上界 INF 充当
  const std::int64_t Z_NEG = -(std::int64_t{1} << 40);
  const std::int64_t Z_POS = +(std::int64_t{1} << 40);
  dist2_out.assign(static_cast<std::size_t>(width) *
                     static_cast<std::size_t>(height), 0.0f);
  std::vector<std::int64_t> f(width), v(width), z(width + 1);
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width; ++x) {
      f[x] = g[row + x];
      f[x] = f[x] * f[x];        // 平方
    }
    std::int64_t k = 0;
    v[0] = 0;
    z[0] = Z_NEG;
    z[1] = Z_POS;
    for (int q = 1; q < width; ++q) {
      std::int64_t p = v[k];
      std::int64_t s = ((f[q] + q * q) - (f[p] + p * p)) / (2 * (q - p));
      while (s <= z[k]) {
        --k;
        p = v[k];
        s = ((f[q] + q * q) - (f[p] + p * p)) / (2 * (q - p));
      }
      ++k;
      v[k] = q;
      z[k] = s;
      z[k + 1] = Z_POS;
    }
    k = 0;
    for (int q = 0; q < width; ++q) {
      while (z[k + 1] < q) {++k;}
      const std::int64_t p = v[k];
      const std::int64_t df = q - p;
      dist2_out[row + q] = static_cast<float>(df * df + f[p]);
    }
  }
}
}  // namespace

void MapField::build(const OccupancyGrid & grid, double sigma_m)
{
  grid_ = &grid;
  sigma_m_ = sigma_m;
  const std::size_t n = static_cast<std::size_t>(grid.width) * grid.height;

    // 1) 双向 EDT
  std::vector<uint8_t> occ_seeds(n), free_seeds(n);
  for (std::size_t i = 0; i < n; ++i) {
    const int8_t c = grid.cells[i];
    occ_seeds[i] = (c > 65) ? 1 : 0;
    free_seeds[i] = (c >= 0 && c <= 65) ? 1 : 0;
  }
  std::vector<float> d_occ, d_free;
  edt2d(grid.width, grid.height, occ_seeds, d_occ);
  edt2d(grid.width, grid.height, free_seeds, d_free);

    // 2) SDF：自由 >0（距墙），墙内 <0（自由空间方向深度）
    //    未知区域当墙处理惩罚；似然给 unknown_score_ 平底
  sdf_cells_.resize(n);
  like_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const int8_t c = grid.cells[i];
    const float d_o = std::sqrt(d_occ[i]);
    const float d_f = std::sqrt(d_free[i]);
    const double sdf_m = (d_o - d_f) * grid.resolution;
    sdf_cells_[i] = static_cast<float>(sdf_m);
    if (c < 0) {
      like_[i] = static_cast<float>(unknown_score_);
    } else {
      const double s2 = 2.0 * sigma_m * sigma_m;
      like_[i] = static_cast<float>(std::exp(-sdf_m * sdf_m / s2));
    }
  }

    // 3) 滑动窗口最大值栈：[0]=like，[k]=2^k 窗口最大（锚点为胞元自身）
    //    递推：窗口 2^k = 四个 2^(k-1) 子窗口之并（锚点平移 h=2^(k-1)）
  swmax_.clear();
  SWGrid base;
  base.width = grid.width;
  base.height = grid.height;
  base.data = like_;
  swmax_.push_back(std::move(base));
  while ((1 << swmax_.size()) <
    std::max(grid.width, grid.height))
  {
    const SWGrid & src = swmax_.back();
    const int h = 1 << (swmax_.size() - 1);
    SWGrid dst;
    dst.width = grid.width;
    dst.height = grid.height;
    dst.data.assign(n, 0.f);
    for (int y = 0; y < grid.height; ++y) {
      const int yh = std::min(y + h, grid.height - 1);
      for (int x = 0; x < grid.width; ++x) {
        const int xh = std::min(x + h, grid.width - 1);
        float m = src.data[static_cast<std::size_t>(y) *
            grid.width + x];
        m = std::max(m, src.data[static_cast<std::size_t>(y) *
            grid.width + xh]);
        m = std::max(m, src.data[static_cast<std::size_t>(yh) *
            grid.width + x]);
        m = std::max(m, src.data[static_cast<std::size_t>(yh) *
            grid.width + xh]);
        dst.data[static_cast<std::size_t>(y) * grid.width + x] = m;
      }
    }
    swmax_.push_back(std::move(dst));
  }
}

double MapField::sdf(double gx, double gy, Eigen::Vector2d * grad) const
{
  const int w = grid_->width, h = grid_->height;
    // 非有限输入护栏（上游位姿发散时不能让查询反噬进程）
  if (!std::isfinite(gx) || !std::isfinite(gy)) {
    if (grad) {grad->setZero();}
    return std::numeric_limits<double>::max() / 4.0;      // 视作远离墙 → 被 trim
  }
    // clamp 到 [0, w-1]×[0, h-1]，基胞取 min(floor, dim-2)：
    // 保证最后半胞的双线性插值仍能看到边界行/列
  gx = std::min(std::max(gx, 0.0), static_cast<double>(w - 1));
  gy = std::min(std::max(gy, 0.0), static_cast<double>(h - 1));
  const int x0 = std::min(static_cast<int>(gx), w - 2);
  const int y0 = std::min(static_cast<int>(gy), h - 2);
  const double fx = gx - x0, fy = gy - y0;
  const std::size_t i00 = static_cast<std::size_t>(y0) * w + x0;
  const std::size_t i10 = i00 + 1;
  const std::size_t i01 = i00 + w;
  const std::size_t i11 = i01 + 1;
  const float v00 = sdf_cells_[i00], v10 = sdf_cells_[i10];
  const float v01 = sdf_cells_[i01], v11 = sdf_cells_[i11];
  if (grad) {
    const double res = grid_->resolution;
        // 解析双线性梯度（/m）
    const double gxv = ((v10 - v00) * (1 - fy) + (v11 - v01) * fy) / res;
    const double gyv = ((v01 - v00) * (1 - fx) + (v11 - v10) * fx) / res;
    grad->x() = gxv;
    grad->y() = gyv;
  }
  return  v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy) +
         v01 * (1 - fx) * fy + v11 * fx * fy;
}

Eigen::Vector2i MapField::descendToObstacle(int ix, int iy) const
{
  const int w = grid_->width, h = grid_->height;
  ix = std::min(std::max(ix, 0), w - 1);
  iy = std::min(std::max(iy, 0), h - 1);
  float best = sdf_cells_[static_cast<std::size_t>(iy) * w + ix];
  if (best <= 0.f) {    // 已在障碍/边界
    return {ix, iy};
  }
    // 8 邻域下降（EDT 是 1-Lipschitz 的，必收敛到最近种子）
  for (int step = 0; step < 2 * (w + h); ++step) {
    int bx = ix, by = iy;
    float bv = best;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {continue;}
        const int nx = ix + dx, ny = iy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
        const float v =
          sdf_cells_[static_cast<std::size_t>(ny) * w + nx];
        if (v < bv) {bv = v; bx = nx; by = ny;}
      }
    }
    if (bv >= best || bv <= 0.f) {
      return bv <= 0.f ? Eigen::Vector2i(bx, by) :
             Eigen::Vector2i(ix, iy);
    }
    ix = bx;
    iy = by;
    best = bv;
  }
  return {ix, iy};
}

void rasterize_segment(
  OccupancyGrid & grid, double x0, double y0,
  double x1, double y1)
{
  auto g0 = grid.worldToGridF(x0, y0);
  auto g1 = grid.worldToGridF(x1, y1);
  const int steps = static_cast<int>(
    std::max(std::abs(g1.x() - g0.x()), std::abs(g1.y() - g0.y())) + 1);
  for (int i = 0; i <= steps; ++i) {
    const double t = steps == 0 ? 0.0 : static_cast<double>(i) / steps;
    const int ix = static_cast<int>(std::lround(g0.x() + t * (g1.x() - g0.x())));
    const int iy = static_cast<int>(std::lround(g0.y() + t * (g1.y() - g0.y())));
    if (grid.inside(ix, iy)) {
      grid.cells[static_cast<std::size_t>(iy) * grid.width + ix] = 100;
    }
  }
}

}  // namespace sloc
