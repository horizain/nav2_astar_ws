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
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/matchers.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

constexpr double kDeg = sloc::kPi / 180.0;

void expect_valid_non_identity_spd_cov(const sloc::Cov3 & cov)
{
  ASSERT_TRUE(cov.allFinite());
  EXPECT_FALSE(cov.isApprox(sloc::Cov3::Identity(), 1e-6));
  EXPECT_NEAR(cov(0, 1), cov(1, 0), 1e-12);
  EXPECT_NEAR(cov(0, 2), cov(2, 0), 1e-12);
  EXPECT_NEAR(cov(1, 2), cov(2, 1), 1e-12);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  ASSERT_EQ(es.info(), Eigen::Success);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

class AlwaysFailMatcher : public sloc::Matcher {
public:
  sloc::MatchResult match(
    const sloc::MapField &, const sloc::Pose2 &,
    const sloc::Scan &, const sloc::MatchOptions &) const override
  {
    sloc::MatchResult res;
    res.valid = false;
    return res;
  }
};

sloc::OccupancyGrid make_symmetric_rooms(double resolution = 0.05)
{
  sloc::OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(16.0 / resolution);
  g.height = static_cast<int>(8.0 / resolution);
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), 0);
  // Two identical 8x8 square rooms side by side (180-deg + translation symmetry)
  sloc::rasterize_segment(g, 0.1, 0.1, 15.9, 0.1);
  sloc::rasterize_segment(g, 0.1, 7.9, 15.9, 7.9);
  sloc::rasterize_segment(g, 0.1, 0.1, 0.1, 7.9);
  sloc::rasterize_segment(g, 8.0, 0.1, 8.0, 7.9);
  sloc::rasterize_segment(g, 15.9, 0.1, 15.9, 7.9);
  // Identical L-shaped pillar in each room
  sloc::rasterize_segment(g, 2.5, 2.5, 4.0, 2.5);
  sloc::rasterize_segment(g, 2.5, 2.5, 2.5, 4.0);
  sloc::rasterize_segment(g, 10.5, 2.5, 12.0, 2.5);
  sloc::rasterize_segment(g, 10.5, 2.5, 10.5, 4.0);
  return g;
}

TEST(MatchersTest, CorrelativeConvergesWithoutDoubleOffset)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(42);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(10.0, 8.5, 0.4);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::CorrelativeMatcher matcher;
  sloc::MatchOptions opt;

  const std::vector<sloc::Pose2> offsets = {
    {0.08, 0.08, 3.0 * kDeg},
    {-0.08, 0.08, -3.0 * kDeg},
    {0.08, -0.08, 3.0 * kDeg},
    {-0.08, -0.08, -3.0 * kDeg},
  };
  for (const auto & d : offsets) {
    const sloc::Pose2 prior(gt.x + d.x, gt.y + d.y, gt.theta + d.theta);
    const auto res = matcher.match(field, prior, scan, opt);
    ASSERT_TRUE(res.valid);
    const double err_xy = std::hypot(res.pose.x - gt.x, res.pose.y - gt.y);
    const double err_th = std::abs(sloc::wrap_angle(res.pose.theta - gt.theta));
    EXPECT_LT(err_xy, 0.02);
    EXPECT_LT(err_th, 1.0 * kDeg);
    expect_valid_non_identity_spd_cov(res.covariance);
  }

  sloc::MatchOptions coarse_only = opt;
  coarse_only.correlative_two_stage = false;
  const sloc::Pose2 prior(gt.x + 0.12, gt.y - 0.10, gt.theta + 4.0 * kDeg);
  const auto coarse_res = matcher.match(field, prior, scan, coarse_only);
  ASSERT_TRUE(coarse_res.valid);
  EXPECT_LT(std::hypot(coarse_res.pose.x - gt.x, coarse_res.pose.y - gt.y), 0.08);
  EXPECT_LT(std::abs(sloc::wrap_angle(coarse_res.pose.theta - gt.theta)), 3.0 * kDeg);
}

TEST(MatchersTest, IcpP2pRotationalConvergenceAndEarlyExitCov)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(123);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(10.0, 8.5, 0.35);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::IcpP2pMatcher matcher;
  sloc::MatchOptions opt;
  opt.max_iterations = 60;

  // Standard offset (+0.10m, -0.10m, +8 deg)
  const sloc::Pose2 prior(gt.x + 0.10, gt.y - 0.10, gt.theta + 8.0 * kDeg);
  const auto res = matcher.match(field, prior, scan, opt);
  ASSERT_TRUE(res.valid);
  EXPECT_LT(std::hypot(res.pose.x - gt.x, res.pose.y - gt.y), 0.02);
  EXPECT_LT(std::abs(sloc::wrap_angle(res.pose.theta - gt.theta)), 0.5 * kDeg);
  expect_valid_non_identity_spd_cov(res.covariance);

  // Early-exit convergence from exact ground truth must still output non-identity SPD cov
  const auto early = matcher.match(field, gt, scan, opt);
  ASSERT_TRUE(early.valid);
  expect_valid_non_identity_spd_cov(early.covariance);
}

TEST(MatchersTest, FieldGnConsistencyAndRollback)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(7);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(10.0, 8.5, 0.35);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::FieldGnMatcher matcher;
  sloc::MatchOptions opt;
  const sloc::Pose2 prior(gt.x + 0.08, gt.y - 0.08, gt.theta + 5.0 * kDeg);
  const auto res = matcher.match(field, prior, scan, opt);
  ASSERT_TRUE(res.valid);
  EXPECT_LT(std::hypot(res.pose.x - gt.x, res.pose.y - gt.y), 0.02);
  EXPECT_LT(std::abs(sloc::wrap_angle(res.pose.theta - gt.theta)), 0.5 * kDeg);
  expect_valid_non_identity_spd_cov(res.covariance);

  // Verify res.score and res.inlier_ratio match res.pose strictly
  const auto world = sloc::transform_scan(res.pose, scan);
  double expected_score = 0.0;
  double expected_inliers = 0.0;
  for (const auto & q : world) {
    const auto g = field.grid().worldToGridF(q.x(), q.y());
    expected_score += field.likelihoodAtF(g.x(), g.y());
    expected_inliers +=
      (std::abs(field.sdf(g.x(), g.y())) < opt.inlier_gate) ? 1.0 : 0.0;
  }
  expected_score /= static_cast<double>(world.size());
  expected_inliers /= static_cast<double>(world.size());
  EXPECT_NEAR(res.score, expected_score, 1e-9);
  EXPECT_NEAR(res.inlier_ratio, expected_inliers, 1e-9);
}

TEST(MatchersTest, NdtSearchesFullNeighborhoodAndOutputsSpdCov)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  sloc::NdtMatcher ndt;
  ASSERT_TRUE(ndt.build(grid, 0.5));

  std::mt19937 rng(99);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(10.0, 8.5, 0.35);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::MatchOptions opt;
  const sloc::Pose2 prior(gt.x + 0.08, gt.y - 0.08, gt.theta + 4.0 * kDeg);
  const auto res = ndt.match(field, prior, scan, opt);
  ASSERT_TRUE(res.valid);
  EXPECT_LT(std::hypot(res.pose.x - gt.x, res.pose.y - gt.y), 0.05);
  EXPECT_LT(std::abs(sloc::wrap_angle(res.pose.theta - gt.theta)), 1.5 * kDeg);
  expect_valid_non_identity_spd_cov(res.covariance);

  // Invalid voxel_m <= 0 and empty grid must return false and never crash
  sloc::NdtMatcher bad_ndt;
  EXPECT_FALSE(bad_ndt.build(grid, 0.0));
  EXPECT_FALSE(bad_ndt.build(grid, -0.5));
  sloc::OccupancyGrid empty_grid;
  EXPECT_FALSE(bad_ndt.build(empty_grid, 0.5));
  const auto bad_res = bad_ndt.match(field, gt, scan, opt);
  EXPECT_FALSE(bad_res.valid);
}

TEST(MatchersTest, ComposedFallbackAndNullSafety)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(55);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(10.0, 8.5, 0.35);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);
  const sloc::Pose2 prior(gt.x + 0.08, gt.y - 0.08, gt.theta + 3.0 * kDeg);
  sloc::MatchOptions opt;

  // Fine matcher fails -> must fall back to coarse with valid non-identity SPD cov
  sloc::ComposedMatcher fallback(
    std::make_shared<sloc::CorrelativeMatcher>(),
    std::make_shared<AlwaysFailMatcher>());
  const auto fb_res = fallback.match(field, prior, scan, opt);
  ASSERT_TRUE(fb_res.valid);
  EXPECT_LT(std::hypot(fb_res.pose.x - gt.x, fb_res.pose.y - gt.y), 0.02);
  expect_valid_non_identity_spd_cov(fb_res.covariance);

  // Null coarse & fine must safely return invalid
  sloc::ComposedMatcher both_null(nullptr, nullptr);
  EXPECT_FALSE(both_null.match(field, prior, scan, opt).valid);
}

TEST(MatchersTest, AllFiveMatchersConvergeAndDetectCorridorDegeneracy)
{
  const auto wh_grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField wh_field;
  ASSERT_TRUE(wh_field.build(wh_grid, 0.20));

  const auto corr_grid = sloc::sim::make_corridor(0.05);
  sloc::MapField corr_field;
  ASSERT_TRUE(corr_field.build(corr_grid, 0.20));

  std::mt19937 rng(2026);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;

  const sloc::Pose2 wh_gt(10.0, 8.5, 0.25);
  const sloc::Pose2 wh_prior(wh_gt.x + 0.07, wh_gt.y - 0.07, wh_gt.theta + 3.0 * kDeg);
  const auto wh_scan = sloc::sim::make_scan(wh_grid, wh_gt, scfg, rng);

  // Corridor middle x = 15.0 in [13, 17] so end walls (>12m away) are invisible
  const sloc::Pose2 corr_gt(15.0, 1.5, 0.0);
  const auto corr_scan = sloc::sim::make_scan(corr_grid, corr_gt, scfg, rng);

  sloc::MatchOptions opt;
  const std::vector<std::string> names = {
    "correlative", "field_gn", "icp_p2p", "ndt", "composed"};

  for (const auto & name : names) {
    auto m_wh = sloc::make_matcher(name, &wh_grid, 0.5);
    ASSERT_NE(m_wh, nullptr) << name;
    const auto r_wh = m_wh->match(wh_field, wh_prior, wh_scan, opt);
    ASSERT_TRUE(r_wh.valid) << name;
    EXPECT_LT(std::hypot(r_wh.pose.x - wh_gt.x, r_wh.pose.y - wh_gt.y), 0.05)
      << name;
    EXPECT_LT(
      std::abs(sloc::wrap_angle(r_wh.pose.theta - wh_gt.theta)), 1.5 * kDeg)
      << name;
    expect_valid_non_identity_spd_cov(r_wh.covariance);

    auto m_corr = sloc::make_matcher(name, &corr_grid, 0.5);
    ASSERT_NE(m_corr, nullptr) << name;
    const auto r_corr = m_corr->match(corr_field, corr_gt, corr_scan, opt);
    ASSERT_TRUE(r_corr.valid) << name;
    expect_valid_non_identity_spd_cov(r_corr.covariance);
    // Along-corridor (x) variance must be >= 10x cross-corridor (y) variance
    EXPECT_GE(r_corr.covariance(0, 0), 10.0 * r_corr.covariance(1, 1))
      << "Matcher=" << name << " cov_xx=" << r_corr.covariance(0, 0)
      << " cov_yy=" << r_corr.covariance(1, 1);
  }
}

TEST(MatchersTest, DegenerateAndMalformedInputsDoNotCrash)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  EXPECT_EQ(sloc::make_matcher("nonexistent_matcher", &grid, 0.5), nullptr);
  EXPECT_EQ(sloc::make_matcher("ndt", nullptr, 0.5), nullptr);
  sloc::OccupancyGrid empty_grid;
  EXPECT_EQ(sloc::make_matcher("ndt", &empty_grid, 0.5), nullptr);

  sloc::Scan empty_scan;
  sloc::Scan one_pt_scan;
  one_pt_scan.points.push_back(sloc::Vec2(1.0, 0.0));

  sloc::Scan nan_inf_scan;
  const double qnan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 10; ++i) {
    nan_inf_scan.points.push_back(sloc::Vec2(qnan, inf));
  }

  const sloc::Pose2 oob_prior(500.0, -500.0, 0.0);
  const sloc::Pose2 ok_prior(10.0, 8.5, 0.0);
  sloc::MatchOptions opt;

  const std::vector<std::string> names = {
    "correlative", "field_gn", "icp_p2p", "ndt", "composed"};
  for (const auto & name : names) {
    auto m = sloc::make_matcher(name, &grid, 0.5);
    ASSERT_NE(m, nullptr) << name;
    EXPECT_FALSE(m->match(field, ok_prior, empty_scan, opt).valid) << name;
    EXPECT_FALSE(m->match(field, ok_prior, nan_inf_scan, opt).valid) << name;
    EXPECT_FALSE(m->match(field, oob_prior, one_pt_scan, opt).valid) << name;
    EXPECT_FALSE(m->match(field, ok_prior, one_pt_scan, opt).valid) << name;
  }
}

TEST(MatchersTest, GlobalCorrelativeNearWallOobAndSymmetricNms)
{
  // 1. Near-wall pose with OOB beams must still recall ground truth
  const auto grid = sloc::sim::make_warehouse(0.05);
  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(321);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(2.0, 2.0, 0.5);
  auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);
  // Inject some out-of-bounds beams pointing far outside map
  for (int i = 0; i < 20; ++i) {
    scan.points.push_back(sloc::Vec2(-25.0, -25.0 - i * 0.1));
  }

  const auto res = sloc::global_correlative(field, scan, 3, 5.0 * kDeg, 3);
  ASSERT_TRUE(res.valid);
  EXPECT_LT(std::hypot(res.pose.x - gt.x, res.pose.y - gt.y), 0.25);
  EXPECT_LT(std::abs(sloc::wrap_angle(res.pose.theta - gt.theta)), 10.0 * kDeg);

  // Invalid ang_step / empty scan safety
  EXPECT_FALSE(sloc::global_correlative(field, scan, 3, 0.0, 3).valid);
  EXPECT_FALSE(sloc::global_correlative(field, scan, 3, -0.1, 3).valid);
  sloc::Scan empty;
  EXPECT_FALSE(sloc::global_correlative(field, empty, 3, 0.1, 3).valid);

  // 2. Symmetric map: alternates must satisfy NMS spacing and count <= topk
  const auto sym_grid = make_symmetric_rooms(0.05);
  sloc::MapField sym_field;
  ASSERT_TRUE(sym_field.build(sym_grid, 0.20));
  const sloc::Pose2 sym_gt(5.5, 5.5, 0.2);
  const auto sym_scan = sloc::sim::make_scan(sym_grid, sym_gt, scfg, rng);

  const int topk = 4;
  const auto sym_res =
    sloc::global_correlative(sym_field, sym_scan, 3, 5.0 * kDeg, topk);
  ASSERT_TRUE(sym_res.valid);
  EXPECT_LE(static_cast<int>(sym_res.alternates.size()), topk - 1);
  EXPECT_GE(sym_res.alternates.size(), 1u);

  std::vector<sloc::Pose2> all_modes;
  all_modes.push_back(sym_res.pose);
  for (const auto & alt : sym_res.alternates) {
    all_modes.push_back(alt);
  }
  for (std::size_t i = 0; i < all_modes.size(); ++i) {
    for (std::size_t j = i + 1; j < all_modes.size(); ++j) {
      const double dxy = std::hypot(
        all_modes[i].x - all_modes[j].x, all_modes[i].y - all_modes[j].y);
      const double dth = std::abs(
        sloc::wrap_angle(all_modes[i].theta - all_modes[j].theta));
      EXPECT_TRUE(dxy >= 0.50 || dth >= 0.35)
        << "NMS violated between modes " << i << " and " << j
        << " dxy=" << dxy << " dth=" << dth;
    }
  }
}

TEST(MatchersTest, BeamVisibilityPenalizesOutOfMapAndWallPiercingCorridorFlip)
{
  // Open west corridor + east wall/unknown exterior (simulating single-ended corridor flip)
  sloc::OccupancyGrid grid;
  grid.resolution = 0.10;
  grid.width = 260;
  grid.height = 50;
  grid.origin_x = -20.0;
  grid.origin_y = -2.5;
  grid.cells.assign(static_cast<std::size_t>(grid.width) * grid.height, -1);

  // Carve free corridor x in [20..220] (wx in [-18..+2]), y in [15..34] (wy in [-1..+1])
  for (int y = 15; y <= 34; ++y) {
    for (int x = 20; x <= 220; ++x) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
  }
  // Solid walls around corridor
  for (int x = 19; x <= 221; ++x) {
    grid.cells[14 * grid.width + x] = 100;
    grid.cells[35 * grid.width + x] = 100;
  }
  for (int y = 14; y <= 35; ++y) {
    grid.cells[y * grid.width + 19] = 100;
    grid.cells[y * grid.width + 221] = 100;
  }
  // Asymmetric alcove near the east end so (true_pose) is uniquely identifiable.
  for (int y = 35; y <= 44; ++y) {
    for (int x = 192; x <= 202; ++x) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
    grid.cells[y * grid.width + 191] = 100;
    grid.cells[y * grid.width + 203] = 100;
  }
  for (int x = 191; x <= 203; ++x) {
    grid.cells[45 * grid.width + x] = 100;
  }
  for (int x = 192; x <= 202; ++x) {
    grid.cells[35 * grid.width + x] = 0;
  }

  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.20));

  std::mt19937 rng(42);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 180;
  scfg.range_max = 25.0;
  scfg.noise_std = 0.0;
  // True pose near east end facing WEST (-x, pi) down the 18m corridor
  const sloc::Pose2 true_pose(-1.5, 0.0, M_PI);
  const auto scan = sloc::sim::make_scan(grid, true_pose, scfg, rng);

  // Flipped pose facing EAST (+x, 0) shoots the 17m corridor beam straight through east wall
  // and 13m+ into gray unknown (-1) cells despite matching side walls!
  const sloc::Pose2 flipped_pose(-1.5, 0.0, 0.0);
  const double vis_true = sloc::beam_visibility_consistency(field, true_pose, scan, true);
  const double vis_flip = sloc::beam_visibility_consistency(field, flipped_pose, scan, true);
  EXPECT_GT(vis_true, 0.95);
  EXPECT_LT(vis_flip, 0.45);

  // A 0.5m moved pillar/cart blocking a few corridor rays must NOT break vis_true.
  sloc::OccupancyGrid dirty = grid;
  for (int y = 24; y <= 25; ++y) {
    for (int x = 120; x <= 125; ++x) {
      dirty.cells[static_cast<std::size_t>(y) * dirty.width + x] = 100;
    }
  }
  sloc::MapField dirty_field;
  ASSERT_TRUE(dirty_field.build(dirty, 0.20));
  EXPECT_GT(sloc::beam_visibility_consistency(dirty_field, true_pose, scan, true), 0.90);

  const auto gres = sloc::global_correlative(field, scan, 2, 5.0 * kDeg, 4);
  ASSERT_TRUE(gres.valid);
  EXPECT_LT(std::abs(sloc::wrap_angle(gres.pose.theta - true_pose.theta)), 0.30);

  // Minor real-map imperfections (~4% of beams shooting past an opened doorway into unknown space)
  // must NOT tank visibility below healthy threshold while 180-deg flip remains rejected.
  sloc::Scan noisy_door_scan = scan;
  for (int i = 0; i < 6; ++i) {
    const double ang = -0.10 + 0.04 * i;
    noisy_door_scan.points[static_cast<std::size_t>(i)] =
      sloc::Vec2(8.0 * std::cos(ang), 8.0 * std::sin(ang));
  }
  const auto bp_noisy = sloc::evaluate_beam_physics(field, true_pose, noisy_door_scan, 0.15, true);
  EXPECT_GT(bp_noisy.visibility, 0.85);
}

TEST(MatchersTest, GlobalCorrelativeAdaptiveBucketSpreadInLongCorridor)
{
  // 80m long repetitive corridor (0.20m coarse search grid) with unique far-end alcove:
  // with topk=24, adaptive spatial bucket scaling + hard cap max_per_bucket=2 must spread
  // candidates across the corridor and avoid packing 8 modes into a single 2.5m cluster.
  sloc::OccupancyGrid grid;
  grid.resolution = 0.20;
  grid.width = 410;
  grid.height = 24;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.cells.assign(static_cast<std::size_t>(grid.width) * grid.height, -1);

  for (int y = 6; y <= 17; ++y) {
    for (int x = 5; x <= 404; ++x) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
  }
  for (int x = 4; x <= 405; ++x) {
    grid.cells[5 * grid.width + x] = 100;
    grid.cells[18 * grid.width + x] = 100;
  }
  for (int y = 5; y <= 18; ++y) {
    grid.cells[y * grid.width + 4] = 100;
    grid.cells[y * grid.width + 405] = 100;
  }
  // Unique doorway alcove near far east end (x in [382..392] -> wx in [76.4..78.4])
  for (int y = 18; y <= 22; ++y) {
    for (int x = 383; x <= 391; ++x) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
    grid.cells[y * grid.width + 382] = 100;
    grid.cells[y * grid.width + 392] = 100;
  }
  for (int x = 382; x <= 392; ++x) {
    grid.cells[23 * grid.width + x] = 100;
  }

  sloc::MapField field;
  ASSERT_TRUE(field.build(grid, 0.25));

  std::mt19937 rng(2026);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 180;
  scfg.range_max = 20.0;
  scfg.noise_std = 0.0;
  const sloc::Pose2 gt(77.4, 2.3, 0.0);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  const auto res = sloc::global_correlative(field, scan, 3, 5.0 * kDeg, 24);
  ASSERT_TRUE(res.valid);

  std::vector<sloc::Pose2> modes;
  modes.push_back(res.pose);
  for (const auto & alt : res.alternates) {
    modes.push_back(alt);
  }
  ASSERT_GE(modes.size(), 6u);

  bool recalled_gt = false;
  double min_x = modes[0].x;
  double max_x = modes[0].x;
  for (const auto & m : modes) {
    min_x = std::min(min_x, m.x);
    max_x = std::max(max_x, m.x);
    if (std::hypot(m.x - gt.x, m.y - gt.y) < 0.60 &&
      std::abs(sloc::wrap_angle(m.theta - gt.theta)) < 0.35)
    {
      recalled_gt = true;
    }
  }
  EXPECT_TRUE(recalled_gt);
  // Candidates must span widely instead of crowding into one local hotspot.
  EXPECT_GT(max_x - min_x, 20.0);

  // Verify no 2.5m spatial window within the same 90-deg heading quadrant holds > 2 modes.
  for (std::size_t i = 0; i < modes.size(); ++i) {
    int same_local_cluster = 0;
    for (std::size_t j = 0; j < modes.size(); ++j) {
      if (std::abs(modes[i].x - modes[j].x) < 2.5 &&
        std::abs(modes[i].y - modes[j].y) < 2.5 &&
        std::abs(sloc::wrap_angle(modes[i].theta - modes[j].theta)) < 0.75)
      {
        ++same_local_cluster;
      }
    }
    EXPECT_LE(same_local_cluster, 3) << "mode i=" << i << " x=" << modes[i].x;
  }
}

}  // namespace
