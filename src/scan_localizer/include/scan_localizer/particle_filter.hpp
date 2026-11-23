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
// 轻量粒子滤波定位（AMCL-lite）：里程计运动模型 + 似然场观测模型
// 作为"PF 架构 vs 扫描匹配架构"的对照组，也充当全局重定位的候选方案
// ============================================================================
#include <random>
#include <vector>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc
{

struct PfOptions
{
  int n_particles{500};
    // 里程计噪声（Thrun sample model）
  double alpha_rot_rot{0.08};
  double alpha_rot_trans{0.05};
  double alpha_trans_trans{0.05};
  double alpha_trans_rot{0.05};
    // 观测（混合似然场模型）
  int obs_subsample{3};
  double z_hit{0.9};
  double z_rand{0.1};
  int free_thresh{25};
    // 重采样与可选运动门控/随机粒子注入
  double ess_frac{0.5};
  double update_min_d{0.0};
  double update_min_a{0.0};
  double random_particle_ratio{0.0};
};

class ParticleFilter {
public:
  struct Particle { double x, y, th, w; };

  explicit ParticleFilter(PfOptions opt = {});

  bool init_global(const OccupancyGrid & grid, std::mt19937 & rng);
  void init_gaussian(
    const Pose2 & mean, double std_xy, double std_th,
    std::mt19937 & rng);

    // 里程计增量（odom 系，base 位姿差）
  void predict(const Pose2 & odom_delta, std::mt19937 & rng);

    // 似然场观测更新 + ESS 重采样；返回有效粒子占比
  double update(const MapField & field, const Scan & scan, std::mt19937 & rng);

    // 加权均值位姿 + 完整 3x3 协方差（角度用 sin/cos 均值）
  void estimate(Pose2 & mean, Cov3 & cov) const;

  double mean_score() const {return mean_score_;}
  bool initialized() const {return !particles_.empty();}
  std::size_t size() const {return particles_.size();}
  const std::vector<Particle> & particles() const {return particles_;}

private:
  std::vector<Particle> particles_;
  std::vector<Particle> resample_buf_;
  std::vector<Vec2> sub_pts_;
  std::vector<double> log_w_buf_;
  PfOptions opt_;
  double mean_score_{0.0};
  double accum_d_{0.0};
  double accum_a_{0.0};
  bool has_updated_{false};
  void resample(std::mt19937 & rng, const OccupancyGrid * grid = nullptr);
};

}  // namespace sloc
