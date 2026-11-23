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

namespace sloc
{

ParticleFilter::ParticleFilter(PfOptions opt)
: opt_(opt) {}

void ParticleFilter::init_global(
  const OccupancyGrid & grid,
  std::mt19937 & rng)
{
    // 在自由单元上均匀撒点
  std::vector<Eigen::Vector2i> free_cells;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (grid.cells[static_cast<std::size_t>(y) * grid.width + x] == 0) {
        free_cells.push_back({x, y});
      }
    }
  }
  particles_.clear();
  if (free_cells.empty()) {return;}
  std::uniform_real_distribution<double> u(0.0, 1.0);
  particles_.reserve(static_cast<std::size_t>(opt_.n_particles));
  const double inv_n = 1.0 / opt_.n_particles;
  for (int i = 0; i < opt_.n_particles; ++i) {
    const auto & c =
      free_cells[static_cast<std::size_t>(u(rng) * free_cells.size())];
    const auto w = grid.gridToWorld(c.x(), c.y());
    particles_.push_back({w.x(), w.y(), u(rng) * 2.0 * M_PI - M_PI,
        inv_n});
  }
  mean_score_ = 0.0;
}

void ParticleFilter::init_gaussian(
  const Pose2 & mean, double std_xy,
  double std_th, std::mt19937 & rng)
{
  std::normal_distribution<double> nx(mean.x, std_xy);
  std::normal_distribution<double> ny(mean.y, std_xy);
  std::normal_distribution<double> nt(mean.theta, std_th);
  particles_.clear();
  particles_.reserve(static_cast<std::size_t>(opt_.n_particles));
  const double inv_n = 1.0 / opt_.n_particles;
  for (int i = 0; i < opt_.n_particles; ++i) {
    particles_.push_back({nx(rng), ny(rng), nt(rng), inv_n});
  }
  mean_score_ = 0.0;
}

namespace
{
// Thrun 采样里程计运动模型
Pose2 sample_motion(
  double dth1, double dtrans, double dth2,
  const PfOptions & o, std::mt19937 & rng)
{
  std::normal_distribution<double> g(0.0, 1.0);
  const double sd_rot1 =
    std::sqrt(o.alpha_rot_rot * dth1 * dth1 +
                  o.alpha_rot_trans * dtrans * dtrans);
  const double sd_trans = std::sqrt(
        o.alpha_trans_trans * dtrans * dtrans +
        o.alpha_trans_rot * (dth1 * dth1 + dth2 * dth2));
  const double sd_rot2 =
    std::sqrt(o.alpha_rot_rot * dth2 * dth2 +
                  o.alpha_rot_trans * dtrans * dtrans);
  const double dh1 = dth1 - sd_rot1 * g(rng);
  const double dt_ = dtrans - sd_trans * g(rng);
  const double dh2 = dth2 - sd_rot2 * g(rng);
  return {dt_ * std::cos(dh1), dt_ * std::sin(dh1), dh1 + dh2};
}
}  // namespace

void ParticleFilter::predict(const Pose2 & odom_delta, std::mt19937 & rng)
{
    // 分解为 rot1-trans-rot2
  const double dx = odom_delta.x, dy = odom_delta.y, dth = odom_delta.theta;
  double trans = std::hypot(dx, dy);
  double rot1 = trans < 1e-6 ?
    0.0 :
    std::atan2(dy, dx) - 0.0;                      // 相对起始角
  const double rot2 = dth - rot1;
  if (trans < 1e-9 && std::abs(dth) < 1e-9) {return;}
  for (auto & p : particles_) {
        // 世界系复合
    const Pose2 step = sample_motion(rot1, trans, rot2, opt_, rng);
    const double c = std::cos(p.th), s = std::sin(p.th);
    p.x += c * step.x - s * step.y;
    p.y += s * step.x + c * step.y;
    p.th += step.theta;
  }
}

double ParticleFilter::update(
  const MapField & field, const Scan & scan,
  std::mt19937 & rng)
{
  if (!field.valid() || particles_.empty()) {return 0.0;}
    // 观测子采样
  Scan sub;
  for (std::size_t i = 0; i < scan.points.size();
    i += static_cast<std::size_t>(opt_.obs_subsample))
  {
    sub.points.push_back(scan.points[i]);
  }
  const int n = static_cast<int>(sub.points.size());
  if (n == 0) {return 0.0;}

  double wsum = 0.0;
  double ssum = 0.0;
  for (auto & p : particles_) {
    const double c = std::cos(p.th), s = std::sin(p.th);
    double sc = 0.0;
    for (const auto & pt : sub.points) {
      const double wx = p.x + c * pt.x() - s * pt.y();
      const double wy = p.y + s * pt.x() + c * pt.y();
      const auto g = field.grid().worldToGridF(wx, wy);
      sc += field.likelihoodAtF(g.x(), g.y());
    }
    sc /= n;
        // AMCL 语义：证据跨帧累积（w *= p(z|x)），重采样后重置 1/n。
        // 逐帧重置会让真值小簇在大而平的云里永远无法触发重采样。
    p.w *= std::max(sc, 1e-6);
    wsum += p.w;
    ssum += sc;
  }
  if (wsum < 1e-300) {return 0.0;}
  for (auto & p : particles_) {
    p.w /= wsum;
  }
  mean_score_ = ssum / particles_.size();

    // ESS 重采样
  double ess = 0.0;
  for (const auto & p : particles_) {
    ess += p.w * p.w;
  }
  ess = ess > 0 ? 1.0 / ess : 0.0;
  const double ess_needed = opt_.ess_frac * particles_.size();
  if (ess < ess_needed) {resample(rng);}
  return ess / particles_.size();
}

void ParticleFilter::estimate(Pose2 & mean, Cov3 & cov) const
{
  mean = {};
  cov = Cov3::Identity();
  if (particles_.empty()) {return;}
  double sx = 0, sy = 0, sth_sin = 0, sth_cos = 0;
  for (const auto & p : particles_) {
    sx += p.w * p.x;
    sy += p.w * p.y;
    sth_sin += p.w * std::sin(p.th);
    sth_cos += p.w * std::cos(p.th);
  }
  mean = {sx, sy, std::atan2(sth_sin, sth_cos)};
  double cxx = 0, cyy = 0, ctt = 0, cxy = 0;
  for (const auto & p : particles_) {
    const double dx = p.x - mean.x, dy = p.y - mean.y;
    double dth = p.th - mean.theta;
    while (dth > M_PI) {dth -= 2 * M_PI;}
    while (dth < -M_PI) {dth += 2 * M_PI;}
    cxx += p.w * dx * dx;
    cyy += p.w * dy * dy;
    ctt += p.w * dth * dth;
    cxy += p.w * dx * dy;
  }
  cov(0, 0) = cxx;
  cov(1, 1) = cyy;
  cov(2, 2) = ctt;
  cov(0, 1) = cov(1, 0) = cxy;
}

void ParticleFilter::resample(std::mt19937 & rng)
{
    // 低方差重采样
  const std::size_t n = particles_.size();
  std::vector<Particle> out;
  out.reserve(n);
  std::uniform_real_distribution<double> u(0.0, 1.0 / n);
  double r = u(rng);
  double c = particles_[0].w;
  std::size_t i = 0;
  const double inv_n = 1.0 / n;
  for (std::size_t m = 0; m < n; ++m) {
    const double um = r + m * inv_n;
    while (um > c && i + 1 < n) {
      ++i;
      c += particles_[i].w;
    }
    out.push_back(particles_[i]);
    out.back().w = inv_n;
  }
  particles_ = std::move(out);
}

}  // namespace sloc
