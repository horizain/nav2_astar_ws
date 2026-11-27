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
#include <array>
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
  int occupied_thresh, int free_thresh, int max_pool_levels,
  bool compute_free_edt)
{
  return build(
    std::make_shared<OccupancyGrid>(grid), sigma_m,
    occupied_thresh, free_thresh, max_pool_levels, compute_free_edt);
}

bool MapField::build(
  std::shared_ptr<const OccupancyGrid> grid, double sigma_m,
  int occupied_thresh, int free_thresh, int max_pool_levels,
  bool compute_free_edt)
{
  valid_ = false;
  grid_.reset();
  sdf_cells_.clear();
  like_.clear();
  swmax_.clear();
  swmax_u8_.clear();
  prior_grid_.clear();
  prior_w_ = 0;
  prior_h_ = 0;

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
  if (compute_free_edt && has_free) {
    edt2d(grid_->width, grid_->height, free_seeds, d_free);
  }
  const bool use_free_edt = compute_free_edt && has_free;

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
      const double sdf_m = use_free_edt ?
        -(std::sqrt(d_free[i]) - 1.0) * res :
        (compute_free_edt ? -max_dist_m : 0.0);
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
      const double sdf_m = use_free_edt ?
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

  // 构建 uint8 量化上界金字塔（使用 ceil 严格保持 B&B 上界可采纳性，体积缩至 1/4）
  swmax_u8_.resize(swmax_.size());
  for (std::size_t lv = 0; lv < swmax_.size(); ++lv) {
    const auto & src = swmax_[lv].data;
    auto & dst = swmax_u8_[lv];
    dst.resize(src.size());
    for (std::size_t i = 0; i < src.size(); ++i) {
      const int q = static_cast<int>(std::ceil(src[i] * 255.0f));
      dst[i] = static_cast<std::uint8_t>(std::clamp(q, 0, 255));
    }
  }

  valid_ = true;
  buildMapPrior();
  return true;
}

void MapField::buildMapPrior()
{
  if (!valid_ || !grid_ || grid_->width <= 0 || grid_->height <= 0) {
    return;
  }
  const int w = grid_->width;
  const int h = grid_->height;
  const double res = grid_->resolution;
  prior_block_m_ = 1.0;
  prior_w_ = std::max(1, static_cast<int>(std::ceil(w * res / prior_block_m_)));
  prior_h_ = std::max(1, static_cast<int>(std::ceil(h * res / prior_block_m_)));
  const std::size_t n_coarse =
    static_cast<std::size_t>(prior_w_) * static_cast<std::size_t>(prior_h_);
  prior_grid_.assign(n_coarse, PriorCell{});

  // 1) 障碍近表面 SDF 梯度结构张量 M = Σ n·nᵀ（粗网格 1m 分箱 + ±5m 积分滑窗，耗时 ~1ms）
  struct TensorAcc { double sxx{0.0}, sxy{0.0}, syy{0.0}, cnt{0.0}; };
  std::vector<TensorAcc> raw_t(n_coarse);
  const float surf_band = static_cast<float>(std::max(0.25, 4.0 * res));
  const std::size_t uw = static_cast<std::size_t>(w);

  if (w >= 3 && h >= 3) {
    for (int y = 1; y < h - 1; ++y) {
      const int by = std::clamp(
        static_cast<int>((y * res) / prior_block_m_), 0, prior_h_ - 1);
      const std::size_t row = static_cast<std::size_t>(y) * uw;
      for (int x = 1; x < w - 1; ++x) {
        const float d = sdf_cells_[row + static_cast<std::size_t>(x)];
        if (d < 0.01f || d > surf_band) {continue;}
        const double gx = static_cast<double>(
          sdf_cells_[row + static_cast<std::size_t>(x + 1)] -
          sdf_cells_[row + static_cast<std::size_t>(x - 1)]);
        const double gy = static_cast<double>(
          sdf_cells_[static_cast<std::size_t>(y + 1) * uw + static_cast<std::size_t>(x)] -
          sdf_cells_[static_cast<std::size_t>(y - 1) * uw + static_cast<std::size_t>(x)]);
        const double gn = std::hypot(gx, gy);
        if (gn < 1e-4) {continue;}
        const double nx = gx / gn;
        const double ny = gy / gn;
        const int bx = std::clamp(
          static_cast<int>((x * res) / prior_block_m_), 0, prior_w_ - 1);
        auto & acc = raw_t[static_cast<std::size_t>(by) *
            static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(bx)];
        acc.sxx += nx * nx;
        acc.sxy += nx * ny;
        acc.syy += ny * ny;
        acc.cnt += 1.0;
      }
    }
  }

  constexpr int kVisRad = 5;  // ±5m 局部可见邻域
  std::vector<TensorAcc> horiz_t(n_coarse);
  for (int by = 0; by < prior_h_; ++by) {
    for (int bx = 0; bx < prior_w_; ++bx) {
      TensorAcc sum;
      const int x0 = std::max(0, bx - kVisRad);
      const int x1 = std::min(prior_w_ - 1, bx + kVisRad);
      for (int xx = x0; xx <= x1; ++xx) {
        const auto & c = raw_t[static_cast<std::size_t>(by) *
            static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(xx)];
        sum.sxx += c.sxx;
        sum.sxy += c.sxy;
        sum.syy += c.syy;
        sum.cnt += c.cnt;
      }
      horiz_t[static_cast<std::size_t>(by) *
        static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(bx)] = sum;
    }
  }

  for (int by = 0; by < prior_h_; ++by) {
    for (int bx = 0; bx < prior_w_; ++bx) {
      TensorAcc sum;
      const int y0 = std::max(0, by - kVisRad);
      const int y1 = std::min(prior_h_ - 1, by + kVisRad);
      for (int yy = y0; yy <= y1; ++yy) {
        const auto & c = horiz_t[static_cast<std::size_t>(yy) *
            static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(bx)];
        sum.sxx += c.sxx;
        sum.sxy += c.sxy;
        sum.syy += c.syy;
        sum.cnt += c.cnt;
      }
      auto & pc = prior_grid_[static_cast<std::size_t>(by) *
          static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(bx)];
      if (sum.cnt < 3.0) {
        pc.kappa = 1.0f;
        pc.deg_x = 1.0f;
        pc.deg_y = 0.0f;
        continue;
      }
      const double tr = sum.sxx + sum.syy;
      const double det_disc = std::hypot(sum.sxx - sum.syy, 2.0 * sum.sxy);
      const double lmax = 0.5 * (tr + det_disc);
      const double lmin = std::max(0.0, 0.5 * (tr - det_disc));
      pc.kappa = static_cast<float>(std::clamp(lmin / std::max(lmax, 1e-6), 0.0, 1.0));
      double vx = sum.sxy;
      double vy = lmin - sum.sxx;
      double vn = std::hypot(vx, vy);
      if (vn < 1e-8) {
        if (sum.sxx <= sum.syy) {
          vx = 1.0;
          vy = 0.0;
        } else {
          vx = 0.0;
          vy = 1.0;
        }
      } else {
        vx /= vn;
        vy /= vn;
      }
      pc.deg_x = static_cast<float>(vx);
      pc.deg_y = static_cast<float>(vy);
    }
  }

  // 2) 稀疏自由空间节点 36 束球面追踪旋转不变傅里叶轮廓描述符（自动识别孪生区，耗时 <5ms）
  constexpr int kNumRays = 36;
  constexpr double kMaxDescRange = 12.0;
  std::array<double, kNumRays> ray_cos{};
  std::array<double, kNumRays> ray_sin{};
  for (int m = 0; m < kNumRays; ++m) {
    const double a = m * (kTwoPi / static_cast<double>(kNumRays));
    ray_cos[static_cast<std::size_t>(m)] = std::cos(a);
    ray_sin[static_cast<std::size_t>(m)] = std::sin(a);
  }

  struct AnchorDesc
  {
    int bx{0}, by{0};
    float wx{0.0f}, wy{0.0f};
    std::array<double, 8> feat{};
    double norm{1.0};
  };
  std::vector<AnchorDesc> anchors;
  const int stride_b = std::max(
    2, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n_coarse) / 360.0))));

  for (int by = 0; by < prior_h_; by += stride_b) {
    for (int bx = 0; bx < prior_w_; bx += stride_b) {
      const double cx_w = grid_->origin_x + (bx + 0.5 * stride_b) * prior_block_m_;
      const double cy_w = grid_->origin_y + (by + 0.5 * stride_b) * prior_block_m_;
      const Eigen::Vector2i gi = grid_->worldToGridIndex(cx_w, cy_w);
      int best_ix = -1, best_iy = -1;
      float best_clearance = 0.30f;
      const int search_r = std::max(1, static_cast<int>(std::ceil(0.8 / res)));
      for (int dy = -search_r; dy <= search_r; ++dy) {
        for (int dx = -search_r; dx <= search_r; ++dx) {
          const int ix = gi.x() + dx;
          const int iy = gi.y() + dy;
          if (!grid_->inside(ix, iy)) {continue;}
          const std::size_t idx = static_cast<std::size_t>(iy) * uw +
            static_cast<std::size_t>(ix);
          const int8_t c = grid_->cells[idx];
          if (c >= 0 && c <= free_thresh_ && sdf_cells_[idx] > best_clearance) {
            best_clearance = sdf_cells_[idx];
            best_ix = ix;
            best_iy = iy;
          }
        }
      }
      if (best_ix < 0) {continue;}

      const Eigen::Vector2d aw = grid_->gridToWorld(best_ix, best_iy);
      std::array<double, kNumRays> ranges{};
      double r_sum = 0.0;
      double r_min = kMaxDescRange;
      for (int m = 0; m < kNumRays; ++m) {
        const double ux = ray_cos[static_cast<std::size_t>(m)];
        const double uy = ray_sin[static_cast<std::size_t>(m)];
        double t = res;
        while (t < kMaxDescRange) {
          const Eigen::Vector2i ridx = grid_->worldToGridIndex(
            aw.x() + t * ux, aw.y() + t * uy);
          if (!grid_->inside(ridx.x(), ridx.y())) {break;}
          const std::size_t cidx = static_cast<std::size_t>(ridx.y()) * uw +
            static_cast<std::size_t>(ridx.x());
          const int8_t cell = grid_->cells[cidx];
          if (cell > occupied_thresh_ || cell < 0) {break;}
          const double d_obs = static_cast<double>(sdf_cells_[cidx]);
          t += std::clamp(0.8 * d_obs, std::max(res, 0.10), 0.80);
        }
        const double rm = std::min(t, kMaxDescRange);
        ranges[static_cast<std::size_t>(m)] = rm;
        r_sum += rm;
        r_min = std::min(r_min, rm);
      }

      AnchorDesc ad;
      ad.bx = std::clamp(
        static_cast<int>((aw.x() - grid_->origin_x) / prior_block_m_), 0, prior_w_ - 1);
      ad.by = std::clamp(
        static_cast<int>((aw.y() - grid_->origin_y) / prior_block_m_), 0, prior_h_ - 1);
      ad.wx = static_cast<float>(aw.x());
      ad.wy = static_cast<float>(aw.y());

      const double r_mean = r_sum / static_cast<double>(kNumRays);
      double var_sum = 0.0;
      for (double rm : ranges) {
        var_sum += (rm - r_mean) * (rm - r_mean);
      }
      std::array<double, kNumRays> sorted_r = ranges;
      std::nth_element(sorted_r.begin(), sorted_r.begin() + kNumRays / 2, sorted_r.end());
      ad.feat[0] = r_mean;
      ad.feat[1] = std::sqrt(var_sum / static_cast<double>(kNumRays));
      ad.feat[2] = r_min;
      ad.feat[3] = sorted_r[kNumRays / 2];

      // 前 4 阶旋转不变傅里叶幅值 |F_1|..|F_4|
      for (int k = 1; k <= 4; ++k) {
        double re = 0.0, im = 0.0;
        for (int m = 0; m < kNumRays; ++m) {
          const double ang = (kTwoPi * k * m) / static_cast<double>(kNumRays);
          re += ranges[static_cast<std::size_t>(m)] * std::cos(ang);
          im += ranges[static_cast<std::size_t>(m)] * std::sin(ang);
        }
        ad.feat[static_cast<std::size_t>(3 + k)] =
          std::hypot(re, im) / static_cast<double>(kNumRays);
      }
      double fn2 = 0.0;
      for (double v : ad.feat) {
        fn2 += v * v;
      }
      ad.norm = std::max(1.0, std::sqrt(fn2));
      anchors.push_back(ad);
    }
  }

  // 两两检索远距（>=3.0m）高相似孪生对并写入邻域先验格
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    struct TwinHit { double dist_f; float wx, wy; };
    std::vector<TwinHit> hits;
    for (std::size_t j = 0; j < anchors.size(); ++j) {
      if (i == j) {continue;}
      const double spatial_d = std::hypot(
        anchors[i].wx - anchors[j].wx, anchors[i].wy - anchors[j].wy);
      if (spatial_d < 3.0) {continue;}
      double diff2 = 0.0;
      for (std::size_t f = 0; f < 8; ++f) {
        const double df = anchors[i].feat[f] - anchors[j].feat[f];
        diff2 += df * df;
      }
      const double rel_diff =
        std::sqrt(diff2) / (0.5 * (anchors[i].norm + anchors[j].norm));
      if (rel_diff < 0.065) {
        hits.push_back({rel_diff, anchors[j].wx, anchors[j].wy});
      }
    }
    if (hits.empty()) {continue;}
    std::sort(hits.begin(), hits.end(),
      [](const TwinHit & a, const TwinHit & b) {return a.dist_f < b.dist_f;});

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nbx = anchors[i].bx + dx;
        const int nby = anchors[i].by + dy;
        if (nbx < 0 || nby < 0 || nbx >= prior_w_ || nby >= prior_h_) {continue;}
        auto & pc = prior_grid_[static_cast<std::size_t>(nby) *
            static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(nbx)];
        for (const auto & th : hits) {
          if (pc.twins.size() >= 4) {break;}
          bool dup = false;
          for (const auto & ex : pc.twins) {
            if (std::hypot(ex.x() - th.wx, ex.y() - th.wy) < 2.0f) {
              dup = true;
              break;
            }
          }
          if (!dup) {
            pc.twins.emplace_back(th.wx, th.wy);
          }
        }
      }
    }
  }
}

LocalMapPrior MapField::queryPrior(double wx, double wy) const
{
  LocalMapPrior out;
  if (!valid_ || !grid_ || prior_w_ <= 0 || prior_h_ <= 0 ||
    prior_grid_.empty() || !std::isfinite(wx) || !std::isfinite(wy))
  {
    return out;
  }
  const int bx = std::clamp(
    static_cast<int>(std::floor((wx - grid_->origin_x) / prior_block_m_)),
    0, prior_w_ - 1);
  const int by = std::clamp(
    static_cast<int>(std::floor((wy - grid_->origin_y) / prior_block_m_)),
    0, prior_h_ - 1);
  const auto & pc = prior_grid_[static_cast<std::size_t>(by) *
      static_cast<std::size_t>(prior_w_) + static_cast<std::size_t>(bx)];
  out.localizability = static_cast<double>(pc.kappa);
  out.degenerate_axis = Eigen::Vector2d(pc.deg_x, pc.deg_y);
  out.has_twins = !pc.twins.empty();
  out.twin_positions.reserve(pc.twins.size());
  for (const auto & t : pc.twins) {
    out.twin_positions.emplace_back(t.x(), t.y());
  }
  return out;
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

  // 起点在障碍内部或未知区（sdf < 0）：沿正 SDF 最陡梯度单调爬升至障碍表面
  if (cur < 0.f) {
    for (int step = 0; step < max_steps; ++step) {
      int bx = ix, by = iy;
      float bv = cur;
      float best_slope = 0.f;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {continue;}
          const int nx = ix + dx, ny = iy + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
          const float v = sdf_at(nx, ny);
          if (v > cur) {
            const float step_len = (dx != 0 && dy != 0) ? 1.41421356f : 1.0f;
            const float slope = (v - cur) / step_len;
            if (slope > best_slope) {
              best_slope = slope;
              bv = v;
              bx = nx;
              by = ny;
            }
          }
        }
      }
      // 严格递增防振荡：无更高邻域则停在当前局部极大值
      if (best_slope <= 0.f) {
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

  // 起点在自由区（sdf > 0）：8 邻域最陡梯度严格下降至障碍表面（sdf <= 0）
  for (int step = 0; step < max_steps; ++step) {
    int bx = ix, by = iy;
    float bv = cur;
    float best_slope = 0.f;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {continue;}
        const int nx = ix + dx, ny = iy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
        const float v = sdf_at(nx, ny);
        if (v < cur) {
          const float step_len = (dx != 0 && dy != 0) ? 1.41421356f : 1.0f;
          const float slope = (cur - v) / step_len;
          if (slope > best_slope) {
            best_slope = slope;
            bv = v;
            bx = nx;
            by = ny;
          }
        }
      }
    }
    if (best_slope <= 0.f || bv <= 0.f) {
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
