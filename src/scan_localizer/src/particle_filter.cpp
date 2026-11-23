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
// 粒子滤波实现
// ============================================================================
#include "scan_localizer/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sloc
{

ParticleFilter::ParticleFilter(PfOptions opt)
: opt_(opt) {}

bool ParticleFilter::init_global(
  const OccupancyGrid & grid,
  std::mt19937 & rng)
{
  particles_.clear();
  mean_score_ = 0.0;
  accum_d_ = 0.0;
  accum_a_ = 0.0;
  has_updated_ = false;
  if (opt_.n_particles <= 0 || grid.width <= 0 || grid.height <= 0 ||
    grid.cells.size() <
    static_cast<std::size_t>(grid.width) *
    static_cast<std::size_t>(grid.height))
  {
    return false;
  }

    // 在自由单元上均匀撒点（可配置自由阈值）
  std::vector<Eigen::Vector2i> free_cells;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const int8_t c = grid.cells[static_cast<std::size_t>(y) *
          static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(x)];
      if (c >= 0 && c <= opt_.free_thresh) {
        free_cells.push_back({x, y});
      }
    }
  }
  if (free_cells.empty()) {return false;}

  std::uniform_int_distribution<std::size_t> cell_dist(
    0, free_cells.size() - 1);
  std::uniform_real_distribution<double> ang_dist(-kPi, kPi);
  particles_.reserve(static_cast<std::size_t>(opt_.n_particles));
  const double inv_n = 1.0 / static_cast<double>(opt_.n_particles);
  for (int i = 0; i < opt_.n_particles; ++i) {
    const auto & c = free_cells[cell_dist(rng)];
    const auto w = grid.gridToWorld(c.x(), c.y());
    particles_.push_back({w.x(), w.y(), ang_dist(rng), inv_n});
  }
  return true;
}

void ParticleFilter::init_gaussian(
  const Pose2 & mean, double std_xy,
  double std_th, std::mt19937 & rng)
{
  particles_.clear();
  mean_score_ = 0.0;
  accum_d_ = 0.0;
  accum_a_ = 0.0;
  has_updated_ = false;
  if (opt_.n_particles <= 0) {return;}

  const double sxy = std::max(0.0, std_xy);
  const double sth = std::max(0.0, std_th);
  std::normal_distribution<double> nx(mean.x, sxy);
  std::normal_distribution<double> ny(mean.y, sxy);
  std::normal_distribution<double> nt(mean.theta, sth);
  particles_.reserve(static_cast<std::size_t>(opt_.n_particles));
  const double inv_n = 1.0 / static_cast<double>(opt_.n_particles);
  for (int i = 0; i < opt_.n_particles; ++i) {
    particles_.push_back({nx(rng), ny(rng), wrap_angle(nt(rng)), inv_n});
  }
}

namespace
{
// Thrun 采样里程计运动模型（支持带符号平移以兼容倒车分解）
Pose2 sample_motion(
  double dth1, double dtrans, double dth2,
  const PfOptions & o, std::mt19937 & rng)
{
  std::normal_distribution<double> g(0.0, 1.0);
  const double sd_rot1 = std::sqrt(
    std::max(0.0, o.alpha_rot_rot * dth1 * dth1 +
    o.alpha_rot_trans * dtrans * dtrans));
  const double sd_trans = std::sqrt(
    std::max(0.0, o.alpha_trans_trans * dtrans * dtrans +
    o.alpha_trans_rot * (dth1 * dth1 + dth2 * dth2)));
  const double sd_rot2 = std::sqrt(
    std::max(0.0, o.alpha_rot_rot * dth2 * dth2 +
    o.alpha_rot_trans * dtrans * dtrans));
  const double dh1 = dth1 - sd_rot1 * g(rng);
  const double dt_ = dtrans - sd_trans * g(rng);
  const double dh2 = dth2 - sd_rot2 * g(rng);
  return {dt_ * std::cos(dh1), dt_ * std::sin(dh1), wrap_angle(dh1 + dh2)};
}
}  // namespace

void ParticleFilter::predict(const Pose2 & odom_delta, std::mt19937 & rng)
{
  if (particles_.empty()) {return;}
  const double dx = odom_delta.x, dy = odom_delta.y;
  const double dth = wrap_angle(odom_delta.theta);
  const double trans = std::hypot(dx, dy);
  if (trans < 1e-9 && std::abs(dth) < 1e-9) {return;}

    // 分解为 rot1-trans-rot2；倒车（|rot1| > π/2）时反向分解避免 ±π 噪声爆炸
  double rot1 = trans < 1e-6 ? 0.0 : wrap_angle(std::atan2(dy, dx));
  double signed_trans = trans;
  if (std::abs(rot1) > 0.5 * kPi) {
    rot1 = wrap_angle(rot1 + kPi);
    signed_trans = -trans;
  }
  const double rot2 = wrap_angle(dth - rot1);

  accum_d_ += trans;
  accum_a_ += std::abs(dth);

  for (auto & p : particles_) {
    const Pose2 step = sample_motion(rot1, signed_trans, rot2, opt_, rng);
    const double c = std::cos(p.th), s = std::sin(p.th);
    p.x += c * step.x - s * step.y;
    p.y += s * step.x + c * step.y;
    p.th = wrap_angle(p.th + step.theta);
  }
}

double ParticleFilter::update(
  const MapField & field, const Scan & scan,
  std::mt19937 & rng)
{
  if (!field.valid() || particles_.empty() || scan.points.empty()) {
    return 0.0;
  }

    // 可选运动门控：静止或微动时跳过观测更新
  if ((opt_.update_min_d > 0.0 || opt_.update_min_a > 0.0) && has_updated_ &&
    accum_d_ < opt_.update_min_d && accum_a_ < opt_.update_min_a)
  {
    double ess = 0.0;
    for (const auto & p : particles_) {
      ess += p.w * p.w;
    }
    return ess > 0.0 ? (1.0 / ess) / static_cast<double>(particles_.size()) :
           0.0;
  }

    // 观测子采样（复用缓冲区，防 obs_subsample <= 0 死循环）
  sub_pts_.clear();
  const std::size_t step =
    static_cast<std::size_t>(std::max(1, opt_.obs_subsample));
  for (std::size_t i = 0; i < scan.points.size(); i += step) {
    const auto & pt = scan.points[i];
    if (std::isfinite(pt.x()) && std::isfinite(pt.y())) {
      sub_pts_.push_back(pt);
    }
  }
  const int n = static_cast<int>(sub_pts_.size());
  if (n == 0) {return 0.0;}

  const double z_hit = std::clamp(opt_.z_hit, 0.0, 1.0);
  const double z_rand = std::max(opt_.z_rand, 1e-6);
    // 有效光束数上限（抑制相邻光束强相关导致的单帧粒子塌缩）
  const double beam_scale = std::min(1.0, 30.0 / static_cast<double>(n));

  log_w_buf_.resize(particles_.size());
  double ssum = 0.0;
  double max_lw = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    const auto & p = particles_[i];
    const double c = std::cos(p.th), s = std::sin(p.th);
    double log_like = 0.0;
    double sc = 0.0;
    for (const auto & pt : sub_pts_) {
      const double wx = p.x + c * pt.x() - s * pt.y();
      const double wy = p.y + s * pt.x() + c * pt.y();
      const auto g = field.grid().worldToGridF(wx, wy);
      const double lk = field.likelihoodAtF(g.x(), g.y());
      sc += lk;
      log_like += std::log(z_hit * lk + z_rand);
    }
    sc /= n;
    ssum += sc;
    const double lw = std::log(std::max(p.w, 1e-300)) + beam_scale * log_like;
    log_w_buf_[i] = lw;
    if (std::isfinite(lw) && lw > max_lw) {
      max_lw = lw;
    }
  }

    // log-sum-exp 归一化（防下溢与 NaN）
  const double inv_n = 1.0 / static_cast<double>(particles_.size());
  if (!std::isfinite(max_lw)) {
    for (auto & p : particles_) {
      p.w = inv_n;
    }
  } else {
    double wsum = 0.0;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      const double w = std::isfinite(log_w_buf_[i]) ?
        std::exp(log_w_buf_[i] - max_lw) : 0.0;
      particles_[i].w = w;
      wsum += w;
    }
    if (wsum <= 0.0 || !std::isfinite(wsum)) {
      for (auto & p : particles_) {
        p.w = inv_n;
      }
    } else {
      for (auto & p : particles_) {
        p.w /= wsum;
      }
    }
  }

  mean_score_ = ssum / static_cast<double>(particles_.size());
  accum_d_ = 0.0;
  accum_a_ = 0.0;
  has_updated_ = true;

    // ESS 重采样
  double ess = 0.0;
  for (const auto & p : particles_) {
    ess += p.w * p.w;
  }
  ess = ess > 0.0 ? 1.0 / ess : 0.0;
  const double ess_needed = opt_.ess_frac * static_cast<double>(particles_.size());
  if (ess < ess_needed) {
    resample(rng, &field.grid());
  }
  return ess / static_cast<double>(particles_.size());
}

void ParticleFilter::estimate(Pose2 & mean, Cov3 & cov) const
{
  mean = {};
  cov = Cov3::Identity();
  if (particles_.empty()) {return;}
  double sx = 0.0, sy = 0.0, sth_sin = 0.0, sth_cos = 0.0;
  for (const auto & p : particles_) {
    sx += p.w * p.x;
    sy += p.w * p.y;
    sth_sin += p.w * std::sin(p.th);
    sth_cos += p.w * std::cos(p.th);
  }
  mean = {sx, sy, wrap_angle(std::atan2(sth_sin, sth_cos))};

  double cxx = 0.0, cyy = 0.0, ctt = 0.0;
  double cxy = 0.0, cxt = 0.0, cyt = 0.0;
  for (const auto & p : particles_) {
    const double dx = p.x - mean.x;
    const double dy = p.y - mean.y;
    const double dth = wrap_angle(p.th - mean.theta);
    cxx += p.w * dx * dx;
    cyy += p.w * dy * dy;
    ctt += p.w * dth * dth;
    cxy += p.w * dx * dy;
    cxt += p.w * dx * dth;
    cyt += p.w * dy * dth;
  }
  cov = Cov3::Zero();
  cov(0, 0) = cxx;
  cov(1, 1) = cyy;
  cov(2, 2) = ctt;
  cov(0, 1) = cov(1, 0) = cxy;
  cov(0, 2) = cov(2, 0) = cxt;
  cov(1, 2) = cov(2, 1) = cyt;
}

void ParticleFilter::resample(std::mt19937 & rng, const OccupancyGrid * grid)
{
    // 低方差重采样（复用成员缓冲区避免每帧堆分配）
  const std::size_t n = particles_.size();
  if (n == 0) {return;}
  resample_buf_.clear();
  resample_buf_.reserve(n);
  const double inv_n = 1.0 / static_cast<double>(n);
  std::uniform_real_distribution<double> u(0.0, inv_n);
  const double r = u(rng);
  double c = particles_[0].w;
  std::size_t i = 0;
  for (std::size_t m = 0; m < n; ++m) {
    const double um = r + static_cast<double>(m) * inv_n;
    while (um > c && i + 1 < n) {
      ++i;
      c += particles_[i].w;
    }
    resample_buf_.push_back(particles_[i]);
    resample_buf_.back().w = inv_n;
  }

    // 可选随机粒子注入（应对绑架恢复）
  const double rand_ratio = std::clamp(opt_.random_particle_ratio, 0.0, 1.0);
  const std::size_t n_rand = static_cast<std::size_t>(
    std::llround(rand_ratio * static_cast<double>(n)));
  if (n_rand > 0 && grid != nullptr && grid->width > 0 && grid->height > 0 &&
    grid->cells.size() >=
    static_cast<std::size_t>(grid->width) *
    static_cast<std::size_t>(grid->height))
  {
    std::vector<Eigen::Vector2i> free_cells;
    for (int y = 0; y < grid->height; ++y) {
      for (int x = 0; x < grid->width; ++x) {
        const int8_t cell = grid->cells[static_cast<std::size_t>(y) *
            static_cast<std::size_t>(grid->width) +
            static_cast<std::size_t>(x)];
        if (cell >= 0 && cell <= opt_.free_thresh) {
          free_cells.push_back({x, y});
        }
      }
    }
    if (!free_cells.empty()) {
      std::uniform_int_distribution<std::size_t> cell_dist(
        0, free_cells.size() - 1);
      std::uniform_real_distribution<double> ang_dist(-kPi, kPi);
      const std::size_t count = std::min(n_rand, n);
      for (std::size_t k = 0; k < count; ++k) {
        const auto & fc = free_cells[cell_dist(rng)];
        const auto w = grid->gridToWorld(fc.x(), fc.y());
        resample_buf_[n - 1 - k] = {w.x(), w.y(), ang_dist(rng), inv_n};
      }
    }
  }

  particles_.swap(resample_buf_);
}

}  // namespace sloc
