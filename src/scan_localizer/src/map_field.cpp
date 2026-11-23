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
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace sloc
{

namespace
{

// 精确向下取整除法（负分子绝不向零截断）
inline std::int64_t floor_div(std::int64_t num, std::int64_t den)
{
  std::int64_t q = num / den;
  const std::int64_t r = num % den;
  if (r != 0 && ((r < 0) != (den < 0))) {
    --q;
  }
  return q;
}

// Meijster 精确欧氏距离变换：seeds[i] 为 true 的单元距离为 0（平方距离，单元系）
void edt2d(
  int width, int height, const std::vector<uint8_t> & seeds,
  std::vector<double> & dist2_out)
{
  const std::size_t w = static_cast<std::size_t>(width);
  const std::size_t h = static_cast<std::size_t>(height);
  dist2_out.assign(w * h, 0.0);
  if (width <= 0 || height <= 0) {return;}

  // 距离值可能达 width+height，平方后超 int32 → 全程 int64 防溢出
  const std::int64_t INF = static_cast<std::int64_t>(width) +
    static_cast<std::int64_t>(height) + 2;
  std::vector<std::int64_t> g(w * h, INF);

  // Pass 1：逐列扫描
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const std::size_t i = static_cast<std::size_t>(y) * w +
        static_cast<std::size_t>(x);
      g[i] = seeds[i] ? 0 :
        (y > 0 ? std::min(INF, g[i - w] + 1) : INF);
    }
    for (int y = height - 2; y >= 0; --y) {
      const std::size_t i = static_cast<std::size_t>(y) * w +
        static_cast<std::size_t>(x);
      g[i] = std::min(g[i], g[i + w] + 1);
    }
  }

  // Pass 2：逐行下包络
  std::vector<std::int64_t> f(w), row_out(w);
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * w;
    for (int x = 0; x < width; ++x) {
      const std::int64_t gx = g[row + static_cast<std::size_t>(x)];
      f[static_cast<std::size_t>(x)] = gx * gx;
    }
    edt1d(f, row_out);
    for (int x = 0; x < width; ++x) {
      dist2_out[row + static_cast<std::size_t>(x)] =
        static_cast<double>(row_out[static_cast<std::size_t>(x)]);
    }
  }
}

}  // namespace

void edt1d(
  const std::vector<std::int64_t> & f,
  std::vector<std::int64_t> & out)
{
  const std::size_t n = f.size();
  out.assign(n, 0);
  if (n == 0) {return;}

  constexpr std::int64_t kZNeg = -(std::int64_t{1} << 60);
  constexpr std::int64_t kZPos = +(std::int64_t{1} << 60);

  std::vector<std::int64_t> v(n, 0);
  std::vector<std::int64_t> z(n + 1, 0);
  std::int64_t k = 0;
  v[0] = 0;
  z[0] = kZNeg;
  z[1] = kZPos;

  for (std::size_t qi = 1; qi < n; ++qi) {
    const std::int64_t q = static_cast<std::int64_t>(qi);
    std::int64_t p = v[static_cast<std::size_t>(k)];
    std::int64_t s = floor_div(
      (f[qi] + q * q) - (f[static_cast<std::size_t>(p)] + p * p),
      2 * (q - p));
    while (s <= z[static_cast<std::size_t>(k)]) {
      --k;
      p = v[static_cast<std::size_t>(k)];
      s = floor_div(
        (f[qi] + q * q) - (f[static_cast<std::size_t>(p)] + p * p),
        2 * (q - p));
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = kZPos;
  }

  k = 0;
  for (std::size_t qi = 0; qi < n; ++qi) {
    const std::int64_t q = static_cast<std::int64_t>(qi);
    while (z[static_cast<std::size_t>(k + 1)] < q) {++k;}
    const std::int64_t p = v[static_cast<std::size_t>(k)];
    const std::int64_t df = q - p;
    out[qi] = df * df + f[static_cast<std::size_t>(p)];
  }
}

bool MapField::build(
  const OccupancyGrid & grid, double sigma_m,
  int occupied_thresh, int free_thresh, int max_pool_levels)
{
  return build(
    std::make_shared<OccupancyGrid>(grid), sigma_m,
    occupied_thresh, free_thresh, max_pool_levels);
}

bool MapField::build(
  std::shared_ptr<const OccupancyGrid> grid, double sigma_m,
  int occupied_thresh, int free_thresh, int max_pool_levels)
{
  valid_ = false;
  grid_.reset();
  sdf_cells_.clear();
  like_.clear();
  swmax_.clear();

  if (!grid || grid->width <= 0 || grid->height <= 0) {return false;}
  if (!std::isfinite(grid->resolution) || grid->resolution <= 0.0) {
    return false;
  }
  if (!std::isfinite(sigma_m) || sigma_m <= 0.0) {return false;}
  if (occupied_thresh < 0 || occupied_thresh > 100 ||
    free_thresh < 0 || free_thresh > occupied_thresh)
  {
    return false;
  }
  if (max_pool_levels < 1) {return false;}

  const std::size_t n =
    static_cast<std::size_t>(grid->width) *
    static_cast<std::size_t>(grid->height);
  if (grid->cells.size() != n) {return false;}

  grid_ = std::move(grid);
  sigma_m_ = sigma_m;
  occupied_thresh_ = occupied_thresh;
  free_thresh_ = free_thresh;

  // 1) 双向 EDT（自由格与占据格分别做种子，未知格不计为自由空间）
  std::vector<uint8_t> occ_seeds(n, 0), free_seeds(n, 0);
  bool has_occ = false;
  bool has_free = false;
  for (std::size_t i = 0; i < n; ++i) {
    const int8_t c = grid_->cells[i];
    if (c > occupied_thresh_) {
      occ_seeds[i] = 1;
      has_occ = true;
    } else if (c >= 0 && c <= free_thresh_) {
      free_seeds[i] = 1;
      has_free = true;
    }
  }
  std::vector<double> d_occ, d_free;
  if (has_occ) {
    edt2d(grid_->width, grid_->height, occ_seeds, d_occ);
  }
  if (has_free) {
    edt2d(grid_->width, grid_->height, free_seeds, d_free);
  }

  // 2) SDF：自由 >0（距墙），障碍表面 =0，障碍内部 <0，未知区给有界负值（非 0）
  sdf_cells_.resize(n);
  like_.resize(n);
  const double res = grid_->resolution;
  const double s2 = 2.0 * sigma_m_ * sigma_m_;
  const double max_dist_m = std::hypot(
    static_cast<double>(grid_->width),
    static_cast<double>(grid_->height)) * res;
  const double unknown_sdf_bound = std::max(res, 3.0 * sigma_m_);

  for (std::size_t i = 0; i < n; ++i) {
    const int8_t c = grid_->cells[i];
    if (c > occupied_thresh_) {
      // 紧邻自由空间的障碍格 d_free==1 → sdf==0（表面），厚墙内部 d_free>1 → sdf<0
      const double sdf_m = has_free ?
        -(std::sqrt(d_free[i]) - 1.0) * res :
        -max_dist_m;
      sdf_cells_[i] = static_cast<float>(sdf_m);
      like_[i] = static_cast<float>(std::exp(-sdf_m * sdf_m / s2));
    } else if (c >= 0 && c <= free_thresh_) {
      const double sdf_m = has_occ ?
        (std::sqrt(d_occ[i]) * res) :
        max_dist_m;
      sdf_cells_[i] = static_cast<float>(sdf_m);
      like_[i] = static_cast<float>(std::exp(-sdf_m * sdf_m / s2));
    } else {
      // 未知区：绝不产生正 SDF，全未知图也不为 0；似然给低背景值
      const double sdf_m = has_free ?
        -std::min(std::sqrt(d_free[i]) * res, unknown_sdf_bound) :
        -unknown_sdf_bound;
      sdf_cells_[i] = static_cast<float>(sdf_m);
      like_[i] = static_cast<float>(unknown_score_);
    }
  }

  // 3) 滑动窗口最大值栈：[0]=like，[k]=2^k 窗口最大（锚点为胞元自身）
  SWGrid base;
  base.width = grid_->width;
  base.height = grid_->height;
  base.data = like_;
  swmax_.push_back(std::move(base));
  const std::size_t max_dim =
    static_cast<std::size_t>(std::max(grid_->width, grid_->height));
  while (static_cast<int>(swmax_.size()) < max_pool_levels &&
    swmax_.size() < 62 &&
    (std::size_t{1} << swmax_.size()) < max_dim)
  {
    const SWGrid & src = swmax_.back();
    const std::size_t h_sz = std::size_t{1} << (swmax_.size() - 1);
    const int h = static_cast<int>(std::min(h_sz, max_dim));
    SWGrid dst;
    dst.width = grid_->width;
    dst.height = grid_->height;
    dst.data.assign(n, 0.f);
    for (int y = 0; y < grid_->height; ++y) {
      const int yh = std::min(y + h, grid_->height - 1);
      const std::size_t row_y =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(grid_->width);
      const std::size_t row_yh =
        static_cast<std::size_t>(yh) * static_cast<std::size_t>(grid_->width);
      for (int x = 0; x < grid_->width; ++x) {
        const int xh = std::min(x + h, grid_->width - 1);
        const std::size_t ux = static_cast<std::size_t>(x);
        const std::size_t uxh = static_cast<std::size_t>(xh);
        float m = src.data[row_y + ux];
        m = std::max(m, src.data[row_y + uxh]);
        m = std::max(m, src.data[row_yh + ux]);
        m = std::max(m, src.data[row_yh + uxh]);
        dst.data[row_y + ux] = m;
      }
    }
    swmax_.push_back(std::move(dst));
  }

  valid_ = true;
  return true;
}

double MapField::sdf(double gx, double gy, Eigen::Vector2d * grad) const
{
  if (!valid_ || !std::isfinite(gx) || !std::isfinite(gy)) {
    if (grad) {grad->setZero();}
    return std::numeric_limits<double>::max() / 4.0;
  }
  const int w = grid_->width, h = grid_->height;
  gx = std::min(std::max(gx, 0.0), static_cast<double>(w - 1));
  gy = std::min(std::max(gy, 0.0), static_cast<double>(h - 1));
  const int x0 = (w > 1) ? std::min(static_cast<int>(gx), w - 2) : 0;
  const int y0 = (h > 1) ? std::min(static_cast<int>(gy), h - 2) : 0;
  const int x1 = (w > 1) ? (x0 + 1) : 0;
  const int y1 = (h > 1) ? (y0 + 1) : 0;
  const double fx = (w > 1) ? (gx - x0) : 0.0;
  const double fy = (h > 1) ? (gy - y0) : 0.0;

  const std::size_t uw = static_cast<std::size_t>(w);
  const float v00 = sdf_cells_[static_cast<std::size_t>(y0) * uw +
      static_cast<std::size_t>(x0)];
  const float v10 = sdf_cells_[static_cast<std::size_t>(y0) * uw +
      static_cast<std::size_t>(x1)];
  const float v01 = sdf_cells_[static_cast<std::size_t>(y1) * uw +
      static_cast<std::size_t>(x0)];
  const float v11 = sdf_cells_[static_cast<std::size_t>(y1) * uw +
      static_cast<std::size_t>(x1)];
  if (grad) {
    const double res = grid_->resolution;
    const double gxv = ((v10 - v00) * (1.0 - fy) + (v11 - v01) * fy) / res;
    const double gyv = ((v01 - v00) * (1.0 - fx) + (v11 - v10) * fx) / res;
    grad->x() = gxv;
    grad->y() = gyv;
  }
  return v00 * (1.0 - fx) * (1.0 - fy) + v10 * fx * (1.0 - fy) +
         v01 * (1.0 - fx) * fy + v11 * fx * fy;
}

Eigen::Vector2i MapField::descendToObstacle(int ix, int iy) const
{
  if (!valid_) {return {0, 0};}
  const int w = grid_->width, h = grid_->height;
  const std::size_t uw = static_cast<std::size_t>(w);
  ix = std::min(std::max(ix, 0), w - 1);
  iy = std::min(std::max(iy, 0), h - 1);
  const int max_steps = 2 * (w + h);

  auto sdf_at = [this, uw](int x, int y) -> float {
      return sdf_cells_[static_cast<std::size_t>(y) * uw +
               static_cast<std::size_t>(x)];
    };

  float cur = sdf_at(ix, iy);
  if (cur == 0.f && grid_->occupied(ix, iy, occupied_thresh_)) {
    return {ix, iy};
  }

  // 起点在障碍内部或未知区（sdf < 0）：沿正 SDF 梯度单调爬升至障碍表面
  if (cur < 0.f) {
    for (int step = 0; step < max_steps; ++step) {
      int bx = ix, by = iy;
      float bv = cur;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {continue;}
          const int nx = ix + dx, ny = iy + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
          const float v = sdf_at(nx, ny);
          if (v > bv) {
            bv = v;
            bx = nx;
            by = ny;
          }
        }
      }
      // 严格递增防振荡：无更高邻域则停在当前局部极大值
      if (bv <= cur) {
        return {ix, iy};
      }
      if (bv >= 0.f) {
        if (grid_->occupied(bx, by, occupied_thresh_)) {
          return {bx, by};
        }
        if (grid_->occupied(ix, iy, occupied_thresh_)) {
          return {ix, iy};
        }
        // 从未知区进入自由空间：继续下方自由区下降以寻找最近障碍
        ix = bx;
        iy = by;
        cur = bv;
        break;
      }
      ix = bx;
      iy = by;
      cur = bv;
    }
    if (cur <= 0.f) {
      return {ix, iy};
    }
  }

  // 起点在自由区（sdf > 0）：8 邻域严格下降至障碍表面（sdf <= 0）
  for (int step = 0; step < max_steps; ++step) {
    int bx = ix, by = iy;
    float bv = cur;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {continue;}
        const int nx = ix + dx, ny = iy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
        const float v = sdf_at(nx, ny);
        if (v < bv) {
          bv = v;
          bx = nx;
          by = ny;
        }
      }
    }
    if (bv >= cur || bv <= 0.f) {
      return bv <= 0.f ? Eigen::Vector2i(bx, by) : Eigen::Vector2i(ix, iy);
    }
    ix = bx;
    iy = by;
    cur = bv;
  }
  return {ix, iy};
}

void rasterize_segment(
  OccupancyGrid & grid, double x0, double y0,
  double x1, double y1)
{
  const auto g0 = grid.worldToGridF(x0, y0);
  const auto g1 = grid.worldToGridF(x1, y1);
  const int steps = static_cast<int>(
    std::max(std::abs(g1.x() - g0.x()), std::abs(g1.y() - g0.y())) + 1);
  for (int i = 0; i <= steps; ++i) {
    const double t = steps == 0 ? 0.0 : static_cast<double>(i) / steps;
    const int ix = static_cast<int>(std::lround(g0.x() + t * (g1.x() - g0.x())));
    const int iy = static_cast<int>(std::lround(g0.y() + t * (g1.y() - g0.y())));
    if (grid.inside(ix, iy)) {
      grid.cells[static_cast<std::size_t>(iy) *
        static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(ix)] = 100;
    }
  }
}

}  // namespace sloc
