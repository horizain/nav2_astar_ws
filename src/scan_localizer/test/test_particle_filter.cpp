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

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/particle_filter.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

TEST(ParticleFilterTest, BackwardMotionHasBoundedDispersion)
{
  sloc::PfOptions opt;
  opt.n_particles = 1000;
  opt.alpha_rot_rot = 0.08;
  opt.alpha_rot_trans = 0.05;
  opt.alpha_trans_trans = 0.05;
  opt.alpha_trans_rot = 0.05;

  std::mt19937 rng_fwd(42);
  sloc::ParticleFilter pf_fwd(opt);
  pf_fwd.init_gaussian(sloc::Pose2(0.0, 0.0, 0.0), 0.0, 0.0, rng_fwd);
  pf_fwd.predict(sloc::Pose2(0.2, 0.0, 0.0), rng_fwd);

  std::mt19937 rng_bwd(42);
  sloc::ParticleFilter pf_bwd(opt);
  pf_bwd.init_gaussian(sloc::Pose2(0.0, 0.0, 0.0), 0.0, 0.0, rng_bwd);
  pf_bwd.predict(sloc::Pose2(-0.2, 0.0, 0.0), rng_bwd);

  sloc::Pose2 mean_fwd, mean_bwd;
  sloc::Cov3 cov_fwd, cov_bwd;
  pf_fwd.estimate(mean_fwd, cov_fwd);
  pf_bwd.estimate(mean_bwd, cov_bwd);

  EXPECT_NEAR(mean_bwd.x, -0.2, 0.03);
  EXPECT_NEAR(mean_bwd.y, 0.0, 0.03);
  EXPECT_NEAR(mean_bwd.theta, 0.0, 0.05);

  // Backward motion dispersion must be comparable to forward motion (no ±pi blowup)
  EXPECT_LT(cov_bwd(0, 0), 0.02);
  EXPECT_LT(cov_bwd(1, 1), 0.02);
  EXPECT_LT(cov_bwd(2, 2), 0.02);
  EXPECT_NEAR(cov_bwd(0, 0), cov_fwd(0, 0), 0.005);
  EXPECT_NEAR(cov_bwd(2, 2), cov_fwd(2, 2), 0.005);
}

TEST(ParticleFilterTest, InitGlobalRejectsAllObstacleAndInvalidCounts)
{
  sloc::OccupancyGrid all_occ;
  all_occ.width = 20;
  all_occ.height = 20;
  all_occ.resolution = 0.05;
  all_occ.cells.assign(400, 100);

  std::mt19937 rng(1);
  sloc::PfOptions opt;
  opt.n_particles = 100;
  sloc::ParticleFilter pf(opt);
  EXPECT_FALSE(pf.init_global(all_occ, rng));
  EXPECT_FALSE(pf.initialized());
  EXPECT_EQ(pf.size(), 0u);

  sloc::OccupancyGrid free_grid;
  free_grid.width = 20;
  free_grid.height = 20;
  free_grid.resolution = 0.05;
  free_grid.cells.assign(400, 0);

  sloc::PfOptions bad_opt;
  bad_opt.n_particles = 0;
  sloc::ParticleFilter bad_pf(bad_opt);
  EXPECT_FALSE(bad_pf.init_global(free_grid, rng));
  EXPECT_FALSE(bad_pf.initialized());

  EXPECT_TRUE(pf.init_global(free_grid, rng));
  EXPECT_TRUE(pf.initialized());
  EXPECT_EQ(pf.particles().size(), 100u);
}

TEST(ParticleFilterTest, AllZeroLikelihoodProducesNoNaN)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(77);
  sloc::PfOptions opt;
  opt.n_particles = 200;
  opt.obs_subsample = 0;  // Must not infinite-loop
  sloc::ParticleFilter pf(opt);
  pf.init_gaussian(sloc::Pose2(10.0, 8.5, 0.0), 0.2, 0.1, rng);

  // Scan whose points all land far outside map (likelihood == 0 everywhere)
  sloc::Scan oob_scan;
  for (int i = 0; i < 60; ++i) {
    oob_scan.points.push_back(sloc::Vec2(500.0 + i, 500.0));
  }

  const double ess_frac = pf.update(field, oob_scan, rng);
  EXPECT_TRUE(std::isfinite(ess_frac));
  EXPECT_GT(ess_frac, 0.0);

  sloc::Pose2 mean;
  sloc::Cov3 cov;
  pf.estimate(mean, cov);
  EXPECT_TRUE(std::isfinite(mean.x));
  EXPECT_TRUE(std::isfinite(mean.y));
  EXPECT_TRUE(std::isfinite(mean.theta));
  EXPECT_TRUE(cov.allFinite());
  for (const auto & p : pf.particles()) {
    EXPECT_TRUE(std::isfinite(p.w));
  }
}

TEST(ParticleFilterTest, WrapAroundPiMeanAndFull3x3Covariance)
{
  // Construct correlated particles around theta = pi so (0,2) and (1,2) cross-terms are non-zero
  sloc::PfOptions opt;
  opt.n_particles = 2000;
  opt.alpha_rot_rot = 0.0;
  opt.alpha_rot_trans = 0.0;
  opt.alpha_trans_trans = 0.0;
  opt.alpha_trans_rot = 0.0;

  std::mt19937 rng(1234);
  sloc::ParticleFilter pf(opt);
  pf.init_gaussian(sloc::Pose2(5.0, -3.0, sloc::kPi), 0.1, 0.05, rng);

  // Apply a known forward-and-turn motion with noise to induce x/y/theta correlation
  sloc::PfOptions noisy = opt;
  noisy.alpha_rot_rot = 0.1;
  noisy.alpha_rot_trans = 0.1;
  noisy.alpha_trans_trans = 0.1;
  noisy.alpha_trans_rot = 0.1;
  sloc::ParticleFilter pf2(noisy);
  pf2.init_gaussian(sloc::Pose2(5.0, -3.0, sloc::kPi), 0.0, 0.05, rng);
  pf2.predict(sloc::Pose2(1.0, 0.2, 0.02), rng);

  sloc::Pose2 mean1, mean2;
  sloc::Cov3 cov1, cov2;
  pf.estimate(mean1, cov1);
  pf2.estimate(mean2, cov2);

  // Mean angle around ±pi must stay near pi (|wrap_angle(mean - pi)| < 0.02)
  EXPECT_NEAR(mean1.x, 5.0, 0.02);
  EXPECT_NEAR(mean1.y, -3.0, 0.02);
  EXPECT_LT(std::abs(sloc::wrap_angle(mean1.theta - sloc::kPi)), 0.02);
  EXPECT_NEAR(cov1(2, 2), 0.05 * 0.05, 0.001);

  // Full 3x3 covariance must be symmetric and capture non-zero (0,2)/(1,2) cross-terms
  EXPECT_NEAR(cov2(0, 1), cov2(1, 0), 1e-12);
  EXPECT_NEAR(cov2(0, 2), cov2(2, 0), 1e-12);
  EXPECT_NEAR(cov2(1, 2), cov2(2, 1), 1e-12);
  EXPECT_GT(std::abs(cov2(0, 2)) + std::abs(cov2(1, 2)), 1e-4);
}

TEST(ParticleFilterTest, TracksPoseWithLogLikelihoodMixture)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(999);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.01;

  const sloc::Pose2 gt(10.0, 8.5, 0.3);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::PfOptions opt;
  opt.n_particles = 500;
  sloc::ParticleFilter pf(opt);
  pf.init_gaussian(sloc::Pose2(gt.x + 0.15, gt.y - 0.15, gt.theta + 0.1),
    0.25, 0.15, rng);

  for (int step = 0; step < 3; ++step) {
    pf.update(field, scan, rng);
  }
  sloc::Pose2 mean;
  sloc::Cov3 cov;
  pf.estimate(mean, cov);
  EXPECT_LT(std::hypot(mean.x - gt.x, mean.y - gt.y), 0.15);
  EXPECT_LT(std::abs(sloc::wrap_angle(mean.theta - gt.theta)), 0.10);
}

}  // namespace
