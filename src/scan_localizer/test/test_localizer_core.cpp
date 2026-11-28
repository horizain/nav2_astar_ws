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

#include "scan_localizer/localizer_core.hpp"
#include "scan_localizer/map_field.hpp"
#include "scan_localizer/matchers.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

class ScriptedMatcher : public sloc::Matcher {
public:
  sloc::MatchResult next_result{};
  bool offset_from_prior{false};
  sloc::Pose2 prior_offset{};

  sloc::MatchResult match(
    const sloc::MapField &, const sloc::Pose2 & prior,
    const sloc::Scan &, const sloc::MatchOptions &) const override
  {
    sloc::MatchResult r = next_result;
    if (offset_from_prior) {
      r.pose = sloc::Pose2(
        prior.x + prior_offset.x,
        prior.y + prior_offset.y,
        sloc::wrap_angle(prior.theta + prior_offset.theta));
    }
    return r;
  }
};

class HypothesisAwareMatcher : public sloc::Matcher {
public:
  sloc::Pose2 true_pose{};

  sloc::MatchResult match(
    const sloc::MapField &, const sloc::Pose2 & prior,
    const sloc::Scan &, const sloc::MatchOptions &) const override
  {
    sloc::MatchResult r;
    const double d = std::hypot(prior.x - true_pose.x, prior.y - true_pose.y);
    const double da = std::abs(sloc::wrap_angle(prior.theta - true_pose.theta));
    if (d < 0.5 && da < 0.3) {
      r.valid = true;
      r.pose = true_pose;
      r.score = 0.92;
      r.inlier_ratio = 0.95;
      r.covariance = sloc::Cov3::Identity() * 1e-3;
    } else {
      r.valid = true;
      r.pose = prior;
      r.score = 0.15;
      r.inlier_ratio = 0.20;
      r.covariance = sloc::Cov3::Identity() * 0.5;
    }
    return r;
  }
};

struct WarehouseFixture
{
  sloc::OccupancyGrid grid;
  std::shared_ptr<sloc::MapField> field;

  WarehouseFixture()
  {
    grid = sloc::sim::make_warehouse(0.05);
    field = std::make_shared<sloc::MapField>();
    field->build(grid, 0.20);
  }
};

const WarehouseFixture & shared_warehouse()
{
  static const WarehouseFixture kFixture;
  return kFixture;
}

sloc::Scan make_dummy_scan()
{
  sloc::Scan s;
  s.points.emplace_back(1.0, 0.0);
  s.points.emplace_back(0.0, 1.0);
  s.points.emplace_back(-1.0, 0.0);
  return s;
}

TEST(LocalizerCoreTest, AllFiveMatchersTrackWithZeroFalseRejections)
{
  const auto & wf = shared_warehouse();
  const std::vector<std::string> names = {
    "correlative", "field_gn", "icp_p2p", "ndt", "composed"
  };

  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;

  for (const auto & name : names) {
    SCOPED_TRACE("matcher=" + name);
    auto matcher = sloc::make_matcher(name, &wf.grid, 0.5);
    ASSERT_NE(matcher, nullptr);

    sloc::CoreConfig cfg;
    cfg.match_opt.window_lin = 0.20;
    cfg.match_opt.window_ang = 0.12;
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(matcher);

    std::mt19937 rng(123);
    sloc::Pose2 gt(10.0, 8.5, 0.3);
    core.on_odom(gt);
    core.set_initial_pose(gt);
    ASSERT_EQ(core.state(), sloc::LocState::TRACKING);

    for (int step = 0; step < 5; ++step) {
      gt = gt * sloc::Pose2(0.15, 0.0, 0.02);
      core.on_odom(gt);
      const auto scan = sloc::sim::make_scan(wf.grid, gt, scfg, rng);
      const auto out = core.on_scan(scan);
      EXPECT_TRUE(out.accepted);
      EXPECT_EQ(out.state, sloc::LocState::TRACKING);
      EXPECT_EQ(out.reject_streak, 0);
      EXPECT_LT(std::hypot(out.pose.x - gt.x, out.pose.y - gt.y), 0.08);
      EXPECT_LT(std::abs(sloc::wrap_angle(out.pose.theta - gt.theta)), 0.05);
    }
  }
}

TEST(LocalizerCoreTest, NonZeroInitialImuYawHasNoFirstScanJump)
{
  const auto & wf = shared_warehouse();
  auto matcher = sloc::make_matcher("composed");
  ASSERT_NE(matcher, nullptr);

  sloc::CoreConfig cfg;
  cfg.use_imu_yaw = true;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(matcher);

  const sloc::Pose2 p0(10.0, 8.5, 0.4);
  core.on_odom(p0);
  core.set_initial_pose(p0);

  // First IMU sample starts at non-zero 1.5 rad
  core.on_imu_yaw(1.5, 0.02);

  std::mt19937 rng(7);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;
  const auto scan = sloc::sim::make_scan(wf.grid, p0, scfg, rng);
  const auto out = core.on_scan(scan);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_LT(std::abs(sloc::wrap_angle(out.pose.theta - p0.theta)), 0.01);

  // Next IMU sample increments by +0.02 rad (no jump from 0)
  const sloc::Pose2 p1(10.0, 8.5, 0.42);
  core.on_odom(p1);
  core.on_imu_yaw(1.52, 0.02);
  const auto scan1 = sloc::sim::make_scan(wf.grid, p1, scfg, rng);
  const auto out1 = core.on_scan(scan1);
  EXPECT_TRUE(out1.accepted);
  EXPECT_LT(std::abs(sloc::wrap_angle(out1.pose.theta - p1.theta)), 0.01);
}

TEST(LocalizerCoreTest, FourDimEkfGyroBiasConverges)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();
  sm->next_result.valid = true;
  sm->next_result.score = 0.95;
  sm->next_result.inlier_ratio = 0.98;
  sm->next_result.covariance = sloc::Cov3::Zero();
  sm->next_result.covariance(0, 0) = 1e-4;
  sm->next_result.covariance(1, 1) = 1e-4;
  sm->next_result.covariance(2, 2) = 1e-5;

  sloc::CoreConfig cfg;
  cfg.use_imu_yaw = true;
  cfg.gyro_bias_init_var = 0.05;
  cfg.gyro_bias_rw_var_per_s = 1e-6;
  cfg.max_gyro_bias = 0.5;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  const double true_bias = 0.05;  // rad/s
  const double dt = 0.1;          // 10 Hz scans
  const double true_wz = 0.10;    // rad/s
  const sloc::Scan dummy = make_dummy_scan();

  sloc::Pose2 gt(5.0, 5.0, 0.0);
  core.on_odom(gt);
  core.set_initial_pose(gt);
  const double init_bias_var = core.gyro_bias_var();

  double max_heading_err_after_warmup = 0.0;
  for (int k = 1; k <= 60; ++k) {
    gt = gt * sloc::Pose2(0.05, 0.0, true_wz * dt);
    core.on_odom(gt);
    // Inject raw gyro rate corrupted by constant bias
    core.on_imu_gyro(true_wz + true_bias, dt);
    sm->next_result.pose = gt;

    const auto out = core.on_scan(dummy);
    ASSERT_TRUE(out.accepted);
    if (k >= 20) {
      const double herr = std::abs(sloc::wrap_angle(out.pose.theta - gt.theta));
      max_heading_err_after_warmup = std::max(max_heading_err_after_warmup, herr);
    }
  }

  EXPECT_NEAR(core.gyro_bias(), true_bias, 0.005);
  EXPECT_LT(core.gyro_bias_var(), init_bias_var * 0.2);
  EXPECT_LT(max_heading_err_after_warmup, 0.01);
}

TEST(LocalizerCoreTest, MapToOdomInvariantBetweenScans)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();
  sm->next_result.valid = true;
  sm->next_result.pose = sloc::Pose2(10.0, 8.0, 0.5);
  sm->next_result.score = 0.9;
  sm->next_result.inlier_ratio = 0.9;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-3;

  sloc::LocalizerCore core;
  core.set_map(wf.field);
  core.set_matcher(sm);

  const sloc::Pose2 odom0(1.0, 2.0, 0.1);
  core.on_odom(odom0);
  core.set_initial_pose(sloc::Pose2(10.0, 8.0, 0.5));
  const auto out = core.on_scan(make_dummy_scan());
  ASSERT_TRUE(out.accepted);

  const sloc::Pose2 m2o_ref = core.map_to_odom();
  // Feed multiple odom updates without any scan
  for (int i = 1; i <= 10; ++i) {
    core.on_odom(sloc::Pose2(1.0 + 0.2 * i, 2.0 - 0.1 * i, 0.1 + 0.05 * i));
    const sloc::Pose2 m2o_now = core.map_to_odom();
    EXPECT_NEAR(m2o_now.x, m2o_ref.x, 1e-12);
    EXPECT_NEAR(m2o_now.y, m2o_ref.y, 1e-12);
    EXPECT_NEAR(sloc::wrap_angle(m2o_now.theta - m2o_ref.theta), 0.0, 1e-12);
  }
}

TEST(LocalizerCoreTest, OdomlessInitAndTracking)
{
  const auto & wf = shared_warehouse();
  auto matcher = sloc::make_matcher("composed");
  ASSERT_NE(matcher, nullptr);

  sloc::CoreConfig cfg;
  cfg.allow_odomless = true;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(matcher);

  // Never call on_odom()
  const sloc::Pose2 gt(10.0, 8.5, 0.3);
  core.set_initial_pose(gt);
  EXPECT_EQ(core.state(), sloc::LocState::TRACKING);

  std::mt19937 rng(99);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.0;

  sloc::Pose2 cur = gt;
  for (int i = 0; i < 4; ++i) {
    cur = cur * sloc::Pose2(0.05, 0.0, 0.01);
    const auto scan = sloc::sim::make_scan(wf.grid, cur, scfg, rng);
    const auto out = core.on_scan(scan);
    EXPECT_TRUE(out.accepted);
    EXPECT_EQ(out.state, sloc::LocState::TRACKING);
    EXPECT_LT(std::hypot(out.pose.x - cur.x, out.pose.y - cur.y), 0.05);
    const sloc::Pose2 m2o = core.map_to_odom();
    EXPECT_NEAR(m2o.x, out.pose.x, 1e-12);
    EXPECT_NEAR(m2o.y, out.pose.y, 1e-12);
  }

  // Odom-less global recovery
  ASSERT_TRUE(core.request_global());
  EXPECT_EQ(core.state(), sloc::LocState::GLOBAL);
  core.set_global_fn([&](const sloc::Scan &, sloc::Pose2 & out_p, double & sc) {
      out_p = cur;
      sc = 0.91;
      return true;
    });
  const auto scan_g = sloc::sim::make_scan(wf.grid, cur, scfg, rng);
  const auto out_g = core.on_scan(scan_g);
  EXPECT_TRUE(out_g.accepted);
  EXPECT_EQ(out_g.state, sloc::LocState::TRACKING);
  EXPECT_GT(out_g.inlier_ratio, 0.8);
  EXPECT_TRUE(out_g.covariance.allFinite());
  EXPECT_NEAR(out_g.covariance(0, 0), 0.04, 1e-12);
}

TEST(LocalizerCoreTest, FullStateMachineAndImmediateDegradedRecovery)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.degraded_after = 2;
  cfg.lost_after = 5;
  cfg.anomaly_window = 0;  // Disable anomaly shortcut so lost_after is tested directly
  sloc::LocalizerCore core(cfg);

  // request_global() contract before map/matcher are set
  EXPECT_FALSE(core.request_global());

  core.set_map(wf.field);
  core.set_matcher(sm);
  EXPECT_EQ(core.state(), sloc::LocState::UNINIT);

  const sloc::Pose2 p0(10.0, 8.5, 0.0);
  core.on_odom(p0);
  core.set_initial_pose(p0);
  EXPECT_EQ(core.state(), sloc::LocState::TRACKING);

  const sloc::Scan dummy = make_dummy_scan();

  // Trigger DEGRADED (2 rejections)
  sm->next_result.valid = false;
  auto out = core.on_scan(dummy);
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  out = core.on_scan(dummy);
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::DEGRADED);

  // Recover DEGRADED -> TRACKING: out.state MUST be TRACKING on the recovery frame!
  sm->next_result.valid = true;
  sm->next_result.pose = p0;
  sm->next_result.score = 0.9;
  sm->next_result.inlier_ratio = 0.9;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-3;
  out = core.on_scan(dummy);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_EQ(core.state(), sloc::LocState::TRACKING);

  // Drive TRACKING -> DEGRADED -> LOST (5 rejections)
  sm->next_result.valid = false;
  for (int i = 0; i < 4; ++i) {
    out = core.on_scan(dummy);
    EXPECT_FALSE(out.accepted);
  }
  EXPECT_EQ(out.state, sloc::LocState::DEGRADED);
  out = core.on_scan(dummy);
  EXPECT_EQ(out.state, sloc::LocState::LOST);

  // LOST -> GLOBAL via request_global()
  EXPECT_TRUE(core.request_global());
  EXPECT_EQ(core.state(), sloc::LocState::GLOBAL);
  // Already GLOBAL -> returns false
  EXPECT_FALSE(core.request_global());

  // GLOBAL -> TRACKING via global_fn_
  std::mt19937 rng(5);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 180;
  scfg.noise_std = 0.0;
  const auto real_scan = sloc::sim::make_scan(wf.grid, p0, scfg, rng);

  core.set_global_fn([&](const sloc::Scan &, sloc::Pose2 & g, double & sc) {
      g = p0;
      sc = 0.88;
      return true;
    });
  out = core.on_scan(real_scan);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_DOUBLE_EQ(out.score, 0.88);
  EXPECT_GT(out.inlier_ratio, 0.8);
  EXPECT_NEAR(out.covariance(0, 0), 0.04, 1e-12);
}

TEST(LocalizerCoreTest, AllFourGatesTriggeredIndividually)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();
  const sloc::Pose2 p0(5.0, 5.0, 0.0);
  const sloc::Scan dummy = make_dummy_scan();

  auto make_core = [&]() {
      sloc::CoreConfig cfg;
      cfg.min_inlier_ratio = 0.60;
      cfg.min_score = 0.35;
      cfg.mahal_gate = 9.0;
      cfg.max_cov_eig = 1.0;
      sloc::LocalizerCore core(cfg);
      core.set_map(wf.field);
      core.set_matcher(sm);
      core.on_odom(p0);
      core.set_initial_pose(p0);
      return core;
    };

  auto set_good = [&]() {
      sm->next_result.valid = true;
      sm->next_result.pose = p0;
      sm->next_result.score = 0.90;
      sm->next_result.inlier_ratio = 0.90;
      sm->next_result.covariance = sloc::Cov3::Identity() * 1e-3;
    };

  // Baseline passes all gates
  set_good();
  EXPECT_TRUE(make_core().on_scan(dummy).accepted);

  // 1) valid gate
  set_good();
  sm->next_result.valid = false;
  EXPECT_FALSE(make_core().on_scan(dummy).accepted);

  // 2) inlier_ratio gate
  set_good();
  sm->next_result.inlier_ratio = 0.40;
  EXPECT_FALSE(make_core().on_scan(dummy).accepted);

  // 3) score gate
  set_good();
  sm->next_result.score = 0.20;
  EXPECT_FALSE(make_core().on_scan(dummy).accepted);

  // 4) mahalanobis gate (huge pose jump with small covariance)
  set_good();
  sm->next_result.pose = sloc::Pose2(p0.x + 2.0, p0.y, p0.theta);
  EXPECT_FALSE(make_core().on_scan(dummy).accepted);

  // 5) max_cov_eig gate (min eigenvalue >= max_cov_eig)
  set_good();
  sm->next_result.covariance = sloc::Cov3::Identity() * 2.0;
  EXPECT_FALSE(make_core().on_scan(dummy).accepted);
}

TEST(LocalizerCoreTest, DegenerateCorridorAnisotropicEkf)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.odom_trans_var_per_m = 0.01;  // After 1m, prior var = 0.01 m^2
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(0.0, 0.0, 0.0));

  // Move 1m forward so prior covariance grows to ~0.01 m^2
  core.on_odom(sloc::Pose2(1.0, 0.0, 0.0));

  // Degenerate corridor along x: R_xx = 200 m^2, lateral R_yy = 0.001 m^2
  // Matcher drifts +0.50 m along x, and measures true lateral y = +0.05 m
  sm->next_result.valid = true;
  sm->next_result.score = 0.90;
  sm->next_result.inlier_ratio = 0.90;
  sm->next_result.pose = sloc::Pose2(1.50, 0.05, 0.0);
  sm->next_result.covariance = sloc::Cov3::Zero();
  sm->next_result.covariance(0, 0) = 200.0;
  sm->next_result.covariance(1, 1) = 0.001;
  sm->next_result.covariance(2, 2) = 0.001;

  const auto out = core.on_scan(make_dummy_scan());
  ASSERT_TRUE(out.accepted);

  // Along-corridor drift absorption must be < 5% of 0.50 m (= 0.025 m)
  const double along_absorbed = std::abs(out.pose.x - 1.0);
  EXPECT_LT(along_absorbed, 0.50 * 0.05);

  // Constrained lateral direction snaps to LiDAR (< 0.01 m error from 0.05 m)
  const double lateral_err = std::abs(out.pose.y - 0.05);
  EXPECT_LT(lateral_err, 0.01);
}

TEST(LocalizerCoreTest, Se2HeadingVariancePropagatesToPosition)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.odom_trans_var_per_m = 0.0;
  cfg.odom_rot_var_per_rad = 0.0;
  cfg.odom_rot_var_per_m = 0.0;
  cfg.degraded_after = 10;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  // Inject large heading variance via global recovery (0.04 rad^2)
  core.set_global_fn([](const sloc::Scan &, sloc::Pose2 & g, double & sc) {
      g = sloc::Pose2(0.0, 0.0, 0.0);
      sc = 0.9;
      return true;
    });
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  ASSERT_TRUE(core.on_scan(make_dummy_scan()).accepted);
  const double py_before = core.last_cov()(1, 1);
  const double pth_before = core.last_cov()(2, 2);
  ASSERT_GT(pth_before, 0.01);

  // Move forward 2.0 m along x=0, theta=0 with matcher rejected so we inspect pure pred cov
  // F(1, 2) = dx * cos(0) = 2.0 => P_yy' = P_yy + 4.0 * P_thth
  sm->next_result.valid = false;
  core.on_odom(sloc::Pose2(2.0, 0.0, 0.0));
  const auto out = core.on_scan(make_dummy_scan());
  EXPECT_FALSE(out.accepted);
  EXPECT_NEAR(out.covariance(1, 1), py_before + 4.0 * pth_before, 1e-6);
  EXPECT_GT(out.covariance(1, 2), 0.0);
}

TEST(LocalizerCoreTest, MultiHypothesisPropagationPromotionAndAutoClear)
{
  const auto & wf = shared_warehouse();
  auto hm = std::make_shared<HypothesisAwareMatcher>();
  const sloc::Scan dummy = make_dummy_scan();

  sloc::CoreConfig cfg;
  cfg.degraded_after = 1;
  cfg.lost_after = 20;
  cfg.hyp_confirm_frames = 2;
  cfg.hyp_min_score = 0.70;
  cfg.hyp_min_inlier = 0.65;
  cfg.anomaly_window = 0;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(hm);

  // Start at wrong pose (0,0,0), true robot is at (5,5,0)
  hm->true_pose = sloc::Pose2(5.0, 5.0, 0.0);
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(0.0, 0.0, 0.0));
  core.add_hypotheses({sloc::Pose2(5.0, 5.0, 0.0), sloc::Pose2(12.0, 12.0, 0.0)});
  ASSERT_EQ(core.hypothesis_count(), 2u);

  // Move +0.5 m: both primary and hypotheses propagate by +0.5 m
  hm->true_pose = sloc::Pose2(5.5, 5.0, 0.0);
  core.on_odom(sloc::Pose2(0.5, 0.0, 0.0));
  auto out = core.on_scan(dummy);
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::DEGRADED);
  // First confirmation frame -> not yet promoted (needs 2)
  ASSERT_EQ(core.hypothesis_count(), 2u);
  EXPECT_NEAR(core.hypothesis_poses()[0].x, 5.5, 1e-6);

  // Second frame (+0.5 m): second confirmation -> promoted to TRACKING & cleared!
  hm->true_pose = sloc::Pose2(6.0, 5.0, 0.0);
  core.on_odom(sloc::Pose2(1.0, 0.0, 0.0));
  out = core.on_scan(dummy);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_NEAR(out.pose.x, 6.0, 1e-6);
  EXPECT_EQ(core.hypothesis_count(), 0u);

  // Auto-clear on stable TRACKING
  core.add_hypotheses({sloc::Pose2(20.0, 20.0, 0.0)});
  EXPECT_EQ(core.hypothesis_count(), 1u);
  EXPECT_TRUE(core.on_scan(dummy).accepted);
  EXPECT_TRUE(core.on_scan(dummy).accepted);
  EXPECT_EQ(core.hypothesis_count(), 0u);

  // Auto-clear on set_initial_pose and set_map
  core.add_hypotheses({sloc::Pose2(20.0, 20.0, 0.0)});
  core.set_initial_pose(sloc::Pose2(6.0, 5.0, 0.0));
  EXPECT_EQ(core.hypothesis_count(), 0u);

  core.add_hypotheses({sloc::Pose2(20.0, 20.0, 0.0)});
  core.set_map(wf.field);
  EXPECT_EQ(core.hypothesis_count(), 0u);
}

TEST(LocalizerCoreTest, ActiveKidnappedDetectionTriggersGlobalEarly)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.degraded_after = 5;
  cfg.lost_after = 60;
  cfg.anomaly_window = 10;
  cfg.anomaly_bad_threshold = 8;
  cfg.anomaly_low_score = 0.25;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  core.on_odom(sloc::Pose2(2.0, 2.0, 0.0));
  core.set_initial_pose(sloc::Pose2(2.0, 2.0, 0.0));

  // Simulate kidnapped robot: very low score every scan
  sm->next_result.valid = true;
  sm->next_result.pose = sloc::Pose2(2.0, 2.0, 0.0);
  sm->next_result.score = 0.10;
  sm->next_result.inlier_ratio = 0.10;
  sm->next_result.covariance = sloc::Cov3::Identity() * 0.1;

  const sloc::Scan dummy = make_dummy_scan();
  sloc::CoreOutput out;
  int frames_to_global = -1;
  for (int i = 1; i <= 15; ++i) {
    out = core.on_scan(dummy);
    if (out.state == sloc::LocState::GLOBAL) {
      frames_to_global = i;
      break;
    }
  }
  EXPECT_EQ(frames_to_global, 8);
  EXPECT_EQ(core.state(), sloc::LocState::GLOBAL);
}

TEST(LocalizerCoreTest, HighScoreMahalRejectionDoesNotTriggerSpuriousGlobal)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.degraded_after = 5;
  cfg.lost_after = 60;
  cfg.anomaly_window = 10;
  cfg.anomaly_bad_threshold = 8;
  cfg.anomaly_low_score = 0.25;
  cfg.mahal_gate = 9.0;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  const sloc::Pose2 p0(5.0, 5.0, 0.0);
  core.on_odom(p0);
  core.set_initial_pose(p0, sloc::Cov3::Identity() * 1e-4);

  // Stationary / turning-in-place with high score & high inlier, but small 5cm offset
  // exceeding tight 3-sigma Mahal gate on frame 1 -> must NOT trigger GLOBAL and must
  // relax covariance to recover TRACKING smoothly.
  sm->next_result.valid = true;
  sm->next_result.pose = sloc::Pose2(5.05, 5.0, 0.01);
  sm->next_result.score = 0.96;
  sm->next_result.inlier_ratio = 0.98;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-4;

  const sloc::Scan dummy = make_dummy_scan();
  int global_count = 0;
  int accepted_count = 0;
  for (int i = 0; i < 15; ++i) {
    const auto out = core.on_scan(dummy);
    if (out.state == sloc::LocState::GLOBAL) {
      ++global_count;
    }
    if (out.accepted) {
      ++accepted_count;
    }
  }
  EXPECT_EQ(global_count, 0);
  EXPECT_GT(accepted_count, 5);
  EXPECT_EQ(core.state(), sloc::LocState::TRACKING);
}

TEST(LocalizerCoreTest, AutomaticGlobalJumpGuardProtectsHealthyTracking)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.anomaly_window = 5;
  cfg.anomaly_bad_threshold = 4;
  cfg.anomaly_low_score = 0.25;
  cfg.global_jump_min_dist = 1.0;
  cfg.global_jump_score_margin = 0.15;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  const sloc::Pose2 p0(5.0, 5.0, 0.0);
  core.on_odom(p0);
  core.set_initial_pose(p0);

  const sloc::Scan dummy = make_dummy_scan();

  // 1) Drive into automatic GLOBAL via 4 bad frames
  sm->next_result.valid = true;
  sm->next_result.pose = p0;
  sm->next_result.score = 0.10;
  sm->next_result.inlier_ratio = 0.10;
  sm->next_result.covariance = sloc::Cov3::Identity() * 0.1;
  for (int i = 0; i < 4; ++i) {
    core.on_scan(dummy);
  }
  ASSERT_EQ(core.state(), sloc::LocState::GLOBAL);

  // Global proposes a 20m distant symmetric corridor (score=0.88),
  // but local tracking has already recovered (score=0.92, inlier=0.95).
  sm->next_result.pose = p0;
  sm->next_result.score = 0.92;
  sm->next_result.inlier_ratio = 0.95;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-3;
  core.set_global_fn([](const sloc::Scan &, sloc::Pose2 & g, double & sc) {
      g = sloc::Pose2(25.0, 5.0, 0.0);
      sc = 0.88;
      return true;
    });
  auto out = core.on_scan(dummy);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_LT(std::hypot(out.pose.x - p0.x, out.pose.y - p0.y), 0.2);
  EXPECT_EQ(core.hypothesis_count(), 1u);

  // 2) Real kidnapped robot: local match stays bad (score=0.12, inlier=0.15),
  // global finds true room 20m away (score=0.90) -> MUST accept immediately!
  sm->next_result.score = 0.12;
  sm->next_result.inlier_ratio = 0.15;
  for (int i = 0; i < 4; ++i) {
    core.on_scan(dummy);
  }
  ASSERT_EQ(core.state(), sloc::LocState::GLOBAL);

  core.set_global_fn([](const sloc::Scan &, sloc::Pose2 & g, double & sc) {
      g = sloc::Pose2(25.0, 5.0, 0.0);
      sc = 0.90;
      return true;
    });
  out = core.on_scan(dummy);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_NEAR(out.pose.x, 25.0, 1e-6);
  EXPECT_NEAR(out.pose.y, 5.0, 1e-6);
}

TEST(LocalizerCoreTest, SetOptionsHotSwapAndNonFiniteInputSafety)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();
  const sloc::Pose2 p0(5.0, 5.0, 0.0);

  sloc::CoreConfig cfg;
  cfg.min_inlier_ratio = 0.60;
  cfg.use_imu_yaw = true;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);
  core.on_odom(p0);
  core.set_initial_pose(p0);

  sm->next_result.valid = true;
  sm->next_result.pose = p0;
  sm->next_result.score = 0.85;
  sm->next_result.inlier_ratio = 0.70;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-3;

  const sloc::Scan dummy = make_dummy_scan();
  EXPECT_TRUE(core.on_scan(dummy).accepted);

  // Hot-swap stricter min_inlier_ratio -> same match now rejected
  cfg.min_inlier_ratio = 0.80;
  core.set_options(cfg);
  EXPECT_FALSE(core.on_scan(dummy).accepted);

  // NaN / +-inf safety across all inputs
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  core.set_initial_pose(sloc::Pose2(nan, inf, -inf));
  core.on_odom(sloc::Pose2(nan, 0.0, inf));
  core.on_imu_yaw(nan, 0.02);
  core.on_imu_yaw(inf, nan);
  core.on_imu_gyro(nan, 0.02);
  core.on_imu_gyro(0.1, -inf);
  core.add_hypotheses({sloc::Pose2(nan, 1.0, 0.0), sloc::Pose2(1.0, inf, 0.0)});
  EXPECT_EQ(core.hypothesis_count(), 0u);

  sloc::Scan bad_scan;
  bad_scan.points.emplace_back(nan, 1.0);
  bad_scan.points.emplace_back(1.0, inf);
  bad_scan.points.emplace_back(1.0, 0.0);

  sm->next_result.pose = sloc::Pose2(nan, inf, -inf);
  sm->next_result.score = nan;
  sm->next_result.inlier_ratio = inf;
  sm->next_result.covariance = sloc::Cov3::Constant(nan);

  const auto out = core.on_scan(bad_scan);
  EXPECT_FALSE(out.accepted);
  EXPECT_TRUE(std::isfinite(out.pose.x));
  EXPECT_TRUE(std::isfinite(out.pose.y));
  EXPECT_TRUE(std::isfinite(out.pose.theta));
  EXPECT_TRUE(out.covariance.allFinite());
}

TEST(LocalizerCoreTest, InPlaceRotationWithLateralSkewSurvivesMahalGate)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.odom_trans_var_per_m = 1e-3;
  cfg.odom_trans_var_per_rad = 1.5e-2;
  cfg.odom_rot_var_per_rad = 1.5e-2;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.mahal_gate = 9.0;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  sloc::Pose2 odom(0.0, 0.0, 0.0);
  const sloc::Pose2 p0(5.0, 5.0, 0.0);
  core.on_odom(odom);
  core.set_initial_pose(p0, sloc::Cov3::Identity() * 1e-4);

  const sloc::Scan dummy = make_dummy_scan();
  for (int i = 0; i < 6; ++i) {
    odom = odom * sloc::Pose2(0.0, 0.0, -0.11);
    core.on_odom(odom);
    const sloc::Pose2 pred(p0.x, p0.y, sloc::wrap_angle(-0.11 * (i + 1)));
    sm->next_result.valid = true;
    // Simulate real-device 10cm lateral shift seen during ~60 deg/s in-place spin
    sm->next_result.pose = sloc::Pose2(pred.x, pred.y + 0.10, pred.theta - 0.01);
    sm->next_result.score = 0.75;
    sm->next_result.inlier_ratio = 0.75;
    sm->next_result.covariance = sloc::Cov3::Zero();
    sm->next_result.covariance(0, 0) = 6.25e-4;
    sm->next_result.covariance(1, 1) = 6.25e-4;
    sm->next_result.covariance(2, 2) = 4.0e-4;

    const auto out = core.on_scan(dummy);
    EXPECT_TRUE(out.accepted) << "step " << i;
    EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  }
}

TEST(LocalizerCoreTest, HypothesisPromotionHysteresisPreventsWeakCandidateSteal)
{
  class CompetingHypMatcher : public sloc::Matcher {
public:
    double leader_score{0.80};
    double leader_inlier{0.85};
    double runner_score{0.52};
    double runner_inlier{0.62};

    sloc::MatchResult match(
      const sloc::MapField &, const sloc::Pose2 & prior,
      const sloc::Scan &, const sloc::MatchOptions &) const override
    {
      sloc::MatchResult r;
      r.valid = true;
      r.pose = prior;
      r.covariance = sloc::Cov3::Identity() * 1e-3;
      if (prior.x < 8.0) {
        r.score = leader_score;
        r.inlier_ratio = leader_inlier;
      } else {
        r.score = runner_score;
        r.inlier_ratio = runner_inlier;
      }
      return r;
    }
  };

  const auto & wf = shared_warehouse();
  auto hm = std::make_shared<CompetingHypMatcher>();
  const sloc::Scan dummy = make_dummy_scan();

  sloc::CoreConfig cfg;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.hyp_confirm_frames = 2;
  cfg.hyp_min_score = 0.45;
  cfg.hyp_min_inlier = 0.50;
  cfg.anomaly_window = 0;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(hm);

  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(0.0, 0.0, 0.0));
  ASSERT_TRUE(core.request_global());
  const sloc::Pose2 leader(5.0, 5.0, 0.0);
  const sloc::Pose2 runner(12.0, 12.0, 0.0);
  core.add_hypotheses({leader, runner});

  // Frame 1: both pass (confirm=1,1) -> not yet promoted
  EXPECT_FALSE(core.on_scan(dummy).accepted);

  // Frame 2: leader still leads on score (0.65 >= 0.52 + 0.08) though slightly below
  // hyp_min_inlier, while runner hits confirm=2 at 0.52 -> runner must NOT steal!
  hm->leader_score = 0.65;
  hm->leader_inlier = 0.48;
  auto out = core.on_scan(dummy);
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(core.state(), sloc::LocState::GLOBAL);

  // Frame 3: leader recovers -> hysteresis decremented 1->0 then +1? Wait: if hysteresis kept >0 or
  // needs one more frame, either way leader (x ~ 5.0) must win, never runner (x ~ 12.0).
  hm->leader_score = 0.80;
  hm->leader_inlier = 0.85;
  for (int i = 0; i < 2 && !out.accepted; ++i) {
    out = core.on_scan(dummy);
  }
  EXPECT_TRUE(out.accepted);
  EXPECT_NEAR(out.pose.x, leader.x, 1e-6);
}

TEST(LocalizerCoreTest, AutoGlobalContinuityAndSymmetryAmbiguityGuard)
{
  class SymmetricCorridorMatcher : public sloc::Matcher {
public:
    double far_score{0.95};
    double local_score{0.93};
    bool local_valid{true};

    sloc::MatchResult match(
      const sloc::MapField &, const sloc::Pose2 & prior,
      const sloc::Scan &, const sloc::MatchOptions &) const override
    {
      sloc::MatchResult r;
      r.pose = prior;
      r.covariance = sloc::Cov3::Identity() * 1e-3;
      if (std::hypot(prior.x - 5.0, prior.y - 5.0) < 1.5) {
        r.valid = local_valid;
        r.score = local_valid ? local_score : 0.0;
        r.inlier_ratio = local_valid ? 0.94 : 0.0;
      } else if (std::hypot(prior.x - 24.0, prior.y - 5.0) < 1.5) {
        r.valid = true;
        r.score = far_score;
        r.inlier_ratio = 0.95;
      }
      return r;
    }
  };

  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<SymmetricCorridorMatcher>();
  const sloc::Scan dummy = make_dummy_scan();

  sloc::CoreConfig cfg;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.hyp_confirm_frames = 2;
  cfg.hyp_ambiguous_confirm_frames = 6;
  cfg.hyp_ambiguity_margin = 0.08;
  cfg.hyp_min_score = 0.45;
  cfg.hyp_min_inlier = 0.50;
  cfg.global_jump_min_dist = 1.0;
  cfg.global_jump_score_margin = 0.15;
  cfg.anomaly_window = 0;

  // 1) Auto-triggered global recovery must prefer continuous local candidate over slightly
  // higher (+0.02) 19m flipped corridor mirror.
  {
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(sm);
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0));
    ASSERT_TRUE(core.request_global(false));
    core.add_hypotheses({sloc::Pose2(24.0, 5.0, 3.14159), sloc::Pose2(5.05, 5.0, 0.01)});

    EXPECT_FALSE(core.on_scan(dummy).accepted);
    const auto out = core.on_scan(dummy);
    EXPECT_TRUE(out.accepted);
    EXPECT_EQ(out.state, sloc::LocState::TRACKING);
    EXPECT_LT(std::hypot(out.pose.x - 5.0, out.pose.y - 5.0), 0.5);
  }

  // 2) Manual/cold global localization with two nearly tied far peaks (diff=0.02 < 0.08)
  // must NOT promote prematurely at frame 2.
  {
    sm->local_valid = false;
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(sm);
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    ASSERT_TRUE(core.request_global(true));
    core.add_hypotheses({sloc::Pose2(24.0, 5.0, 3.14159), sloc::Pose2(14.0, 5.0, 0.0)});
    // Re-score both as high-scoring far peaks via distinct x coordinates:
    class TiedFarMatcher : public sloc::Matcher {
public:
      sloc::MatchResult match(
        const sloc::MapField &, const sloc::Pose2 & prior,
        const sloc::Scan &, const sloc::MatchOptions &) const override
      {
        sloc::MatchResult r;
        r.valid = true;
        r.pose = prior;
        r.inlier_ratio = 0.95;
        r.covariance = sloc::Cov3::Identity() * 1e-3;
        r.score = (prior.x > 20.0) ? 0.95 : 0.93;
        return r;
      }
    };
    core.set_matcher(std::make_shared<TiedFarMatcher>());

    for (int f = 0; f < 5; ++f) {
      const auto out = core.on_scan(dummy);
      EXPECT_FALSE(out.accepted) << "must hold promotion under ambiguity at frame " << f;
    }
    const auto out6 = core.on_scan(dummy);
    EXPECT_TRUE(out6.accepted);
  }
}

TEST(LocalizerCoreTest, CorridorLongitudinalLagRelaxesMahalGate)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.mahal_gate = 9.0;
  cfg.odom_trans_var_per_m = 1e-3;
  cfg.anomaly_window = 0;
  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);

  sloc::Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);
  core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-4);

  const sloc::Scan dummy = make_dummy_scan();
  sm->next_result.valid = true;
  sm->next_result.score = 0.96;
  sm->next_result.inlier_ratio = 0.99;
  sm->next_result.covariance = sloc::Cov3::Zero();
  sm->next_result.covariance(0, 0) = 6.25e-4;
  sm->next_result.covariance(1, 1) = 6.25e-4;
  sm->next_result.covariance(2, 2) = 4.0e-4;

  // Frame 1: 0.13m lag -> MAHAL_GATE rejects, inflates P4_
  odom = sloc::Pose2(0.08, 0.0, 0.0);
  core.on_odom(odom);
  sm->next_result.pose = sloc::Pose2(4.95, 5.0, 0.0);
  const auto out1 = core.on_scan(dummy);
  EXPECT_FALSE(out1.accepted);

  // Frame 2: 0.18m lag against relaxed covariance -> recovers locally without hitting GLOBAL
  odom = sloc::Pose2(0.16, 0.0, 0.0);
  core.on_odom(odom);
  sm->next_result.pose = sloc::Pose2(4.98, 5.0, 0.0);
  const auto out2 = core.on_scan(dummy);
  EXPECT_TRUE(out2.accepted);
  EXPECT_EQ(out2.state, sloc::LocState::TRACKING);
}

TEST(LocalizerCoreTest, StationaryClearWinnerPromotesWhileWeakLoneSurvivorWaits)
{
  class ScoredPoseMatcher : public sloc::Matcher {
public:
    double high_score{0.87};
    double high_inlier{0.80};
    double low_score{0.55};
    double low_inlier{0.56};

    sloc::MatchResult match(
      const sloc::MapField &, const sloc::Pose2 & prior,
      const sloc::Scan &, const sloc::MatchOptions &) const override
    {
      sloc::MatchResult r;
      r.valid = true;
      r.pose = prior;
      r.covariance = sloc::Cov3::Identity() * 1e-3;
      if (prior.x < 10.0) {
        r.score = high_score;
        r.inlier_ratio = high_inlier;
      } else {
        r.score = low_score;
        r.inlier_ratio = low_inlier;
      }
      return r;
    }
  };

  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScoredPoseMatcher>();

  // Real-size scan (>=16 beams) to exercise SPRT path without visibility distortion.
  sloc::Scan real_scan;
  for (int i = 0; i < 36; ++i) {
    const double ang = i * (2.0 * sloc::kPi / 36.0);
    real_scan.points.emplace_back(2.0 * std::cos(ang), 2.0 * std::sin(ang));
  }

  sloc::CoreConfig cfg;
  cfg.use_visibility_check = false;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.hyp_min_score = 0.45;
  cfg.hyp_min_inlier = 0.50;
  cfg.hyp_confirm_frames = 2;
  cfg.hyp_ambiguity_margin = 0.08;
  cfg.anomaly_window = 0;

  // 1) Stationary robot with a clear dominant winner (0.87 vs 0.55 distractor) must promote
  //    within a few frames instead of deadlocking with gamma=0.
  {
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(sm);
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    ASSERT_TRUE(core.request_global(true));
    core.add_hypotheses({sloc::Pose2(5.0, 5.0, 0.0), sloc::Pose2(15.0, 5.0, 0.0)});

    sloc::CoreOutput out;
    bool promoted = false;
    for (int f = 0; f < 6; ++f) {
      out = core.on_scan(real_scan);
      if (out.accepted) {
        promoted = true;
        break;
      }
    }
    EXPECT_TRUE(promoted);
    EXPECT_EQ(out.state, sloc::LocState::TRACKING);
    EXPECT_NEAR(out.pose.x, 5.0, 1e-6);
  }

  // 2) Marginal lone survivor (score=0.47, inlier=0.51) must NOT jump into TRACKING in 2 frames.
  {
    sm->low_score = 0.47;
    sm->low_inlier = 0.51;
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(sm);
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    ASSERT_TRUE(core.request_global(true));
    core.add_hypotheses({sloc::Pose2(15.0, 5.0, 0.0)});

    EXPECT_FALSE(core.on_scan(real_scan).accepted);
    EXPECT_FALSE(core.on_scan(real_scan).accepted);
  }
}


TEST(LocalizerCoreTest, TrackingSurvivesOpenMapFrontierWithoutLowScoreKick)
{
  // Open west boundary corridor (x=0..400 free, no west wall)
  sloc::OccupancyGrid grid;
  grid.resolution = 0.05;
  grid.width = 420;
  grid.height = 100;
  grid.origin_x = -20.0;
  grid.origin_y = -2.5;
  grid.cells.assign(static_cast<std::size_t>(grid.width) * grid.height, -1);
  for (int y = 30; y <= 69; ++y) {
    for (int x = 0; x <= 400; ++x) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
  }
  for (int x = 0; x <= 401; ++x) {
    grid.cells[29 * grid.width + x] = 100;
    grid.cells[70 * grid.width + x] = 100;
  }
  for (int y = 29; y <= 70; ++y) {
    grid.cells[y * grid.width + 401] = 100;
  }

  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.15));

  sloc::Scan frontier_scan;
  for (int i = 0; i < 180; ++i) {
    const double ang = -sloc::kPi + i * (sloc::kTwoPi / 180.0);
    if (std::abs(ang) < 0.40) {
      frontier_scan.points.emplace_back(12.0 * std::cos(ang), 12.0 * std::sin(ang));
    } else if (std::abs(std::sin(ang)) > 0.20) {
      const double r = std::min(15.0, 1.0 / std::abs(std::sin(ang)));
      frontier_scan.points.emplace_back(r * std::cos(ang), r * std::sin(ang));
    } else {
      frontier_scan.points.emplace_back(12.0 * std::cos(ang), 12.0 * std::sin(ang));
    }
  }

  sloc::CoreConfig cfg;
  cfg.use_visibility_check = true;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  sloc::LocalizerCore core(cfg);
  core.set_map(field);
  core.set_matcher(sloc::make_matcher("field_gn"));
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(-18.5, 0.0, sloc::kPi));

  // Consecutive frames facing unsealed west map boundary must stay TRACKING with zero kicks.
  for (int f = 0; f < 6; ++f) {
    const auto out = core.on_scan(frontier_scan);
    EXPECT_TRUE(out.accepted) << "frame=" << f << " score=" << out.score;
    EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  }
}


TEST(LocalizerCoreTest, OrthogonalGatesExposeExactDiagnosticsForWallFlipVsOpenFrontier)
{
  // Open west boundary + walled east end with unknown exterior behind east wall
  sloc::OccupancyGrid grid;
  grid.resolution = 0.05;
  grid.width = 420;
  grid.height = 100;
  grid.origin_x = -20.0;
  grid.origin_y = -2.5;
  grid.cells.assign(static_cast<std::size_t>(grid.width) * grid.height, -1);
  for (int y = 30; y <= 69; ++y) {
    for (int x = 0; x <= 400; ++x) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
  }
  for (int x = 0; x <= 401; ++x) {
    grid.cells[29 * grid.width + x] = 100;
    grid.cells[70 * grid.width + x] = 100;
  }
  for (int y = 29; y <= 70; ++y) {
    grid.cells[y * grid.width + 401] = 100;
  }

  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.15));

  sloc::Scan scan;
  for (int i = 0; i < 180; ++i) {
    const double ang = -sloc::kPi + i * (sloc::kTwoPi / 180.0);
    if (std::abs(ang) < 0.40) {
      scan.points.emplace_back(12.0 * std::cos(ang), 12.0 * std::sin(ang));
    } else if (std::abs(std::sin(ang)) > 0.20) {
      const double r = std::min(15.0, 1.0 / std::abs(std::sin(ang)));
      scan.points.emplace_back(r * std::cos(ang), r * std::sin(ang));
    } else {
      scan.points.emplace_back(12.0 * std::cos(ang), 12.0 * std::sin(ang));
    }
  }

  sloc::CoreConfig cfg;
  cfg.use_visibility_check = true;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.min_wall_visibility = 0.50;

  // 1) Open west frontier: zero wall contradiction -> accepted, raw_score intact, zero reject flags
  {
    sloc::LocalizerCore core(cfg);
    core.set_map(field);
    core.set_matcher(sloc::make_matcher("field_gn"));
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    core.set_initial_pose(sloc::Pose2(-18.5, 0.0, sloc::kPi));
    const auto out = core.on_scan(scan);
    EXPECT_TRUE(out.accepted);
    EXPECT_EQ(out.diag.reject_flags, sloc::GateFlag::kNone);
    EXPECT_GT(out.diag.wall_visibility, 0.95);
    EXPECT_GT(out.diag.frontier_ratio, 0.05);
    EXPECT_GE(out.score, out.diag.raw_score - 1e-6);
  }

  // 2) East wall-piercing flip: independently flagged by GateFlag::kWallPhys
  {
    sloc::LocalizerCore core(cfg);
    core.set_map(field);
    core.set_matcher(sloc::make_matcher("field_gn"));
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    core.set_initial_pose(sloc::Pose2(-1.5, 0.0, 0.0));
    const auto out = core.on_scan(scan);
    EXPECT_FALSE(out.accepted);
    EXPECT_NE(out.diag.reject_flags & sloc::GateFlag::kWallPhys, 0u);
    EXPECT_LT(out.diag.wall_visibility, cfg.min_wall_visibility);
  }
}

}  // namespace

TEST(LocalizerCoreTest, StationaryCovarianceFloorPreventsMahalLockup)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.mahal_gate = 9.0;
  cfg.min_pred_trans_var = 2.25e-4;  // (0.015 m)^2
  cfg.min_pred_rot_var = 2.25e-4;    // (0.015 rad)^2
  cfg.anomaly_window = 0;

  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-6);

  const sloc::Scan dummy = make_dummy_scan();
  sm->next_result.valid = true;
  sm->next_result.score = 0.96;
  sm->next_result.inlier_ratio = 0.95;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-5;
  // 0.03m stationary offset: without floor (P=1e-6, R=1e-5) mahal=81.8 > 9.0 -> rejected;
  // with 2.25e-4 floor, S=2.35e-4 -> mahal=3.83 < 9.0 -> accepted immediately.
  sm->next_result.pose = sloc::Pose2(5.03, 5.0, 0.0);

  const auto out = core.on_scan(dummy);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_LT(out.diag.mahal_dist, cfg.mahal_gate);
}

TEST(LocalizerCoreTest, HighConfidenceNonDegenerateMahalSoftRelaxesWhileDegenerateRejects)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.mahal_gate = 9.0;
  cfg.mahal_soft_min_score = 0.85;
  cfg.mahal_soft_min_inlier = 0.80;
  cfg.mahal_soft_max_cond = 25.0;
  cfg.anomaly_window = 0;

  const sloc::Scan dummy = make_dummy_scan();

  // 1) High-confidence non-degenerate match with 0.28m offset:
  //    Frame 1 rejects & relaxes; Frame 2 soft-accepts without hitting LOST.
  {
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(sm);
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-4);

    sm->next_result.valid = true;
    sm->next_result.score = 0.96;
    sm->next_result.inlier_ratio = 0.94;
    sm->next_result.covariance = sloc::Cov3::Identity() * 5e-4;
    sm->next_result.pose = sloc::Pose2(5.28, 5.0, 0.0);

    const auto out1 = core.on_scan(dummy);
    EXPECT_FALSE(out1.accepted);
    EXPECT_EQ(out1.diag.reject_flags, sloc::GateFlag::kMahal);

    const auto out2 = core.on_scan(dummy);
    EXPECT_TRUE(out2.accepted);
    EXPECT_EQ(out2.state, sloc::LocState::TRACKING);
    EXPECT_GT(out2.pose.x, 5.10);
  }

  // 2) Degenerate corridor match (cond = 100 > 25) with 0.28m offset:
  //    Must NOT soft-relax; stays rejected to protect against corridor slip.
  {
    sloc::LocalizerCore core(cfg);
    core.set_map(wf.field);
    core.set_matcher(sm);
    core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
    core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-4);

    sm->next_result.valid = true;
    sm->next_result.score = 0.96;
    sm->next_result.inlier_ratio = 0.94;
    sm->next_result.covariance = sloc::Cov3::Zero();
    sm->next_result.covariance(0, 0) = 5e-2;   // 100x longitudinal uncertainty
    sm->next_result.covariance(1, 1) = 5e-4;
    sm->next_result.covariance(2, 2) = 5e-4;
    sm->next_result.pose = sloc::Pose2(5.0, 5.28, 0.0);

    EXPECT_FALSE(core.on_scan(dummy).accepted);
    EXPECT_FALSE(core.on_scan(dummy).accepted);
  }
}

TEST(LocalizerCoreTest, OdomJumpDegradesToWideWindowWithoutLosingLock)
{
  const auto & wf = shared_warehouse();
  auto sm = std::make_shared<ScriptedMatcher>();

  sloc::CoreConfig cfg;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.mahal_gate = 9.0;
  cfg.odom_jump_max_trans = 0.30;
  cfg.odom_jump_max_rot = 0.45;
  cfg.anomaly_window = 0;

  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(sm);
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-4);

  const sloc::Scan dummy = make_dummy_scan();
  sm->next_result.valid = true;
  sm->next_result.score = 0.95;
  sm->next_result.inlier_ratio = 0.94;
  sm->next_result.covariance = sloc::Cov3::Identity() * 1e-3;
  sm->next_result.pose = sloc::Pose2(5.0, 5.0, 0.0);
  ASSERT_TRUE(core.on_scan(dummy).accepted);

  // Inject sudden 0.45m /odom jump while robot actually moved 0.05m
  core.on_odom(sloc::Pose2(0.45, 0.0, 0.0));
  sm->next_result.pose = sloc::Pose2(5.05, 5.0, 0.0);
  const auto out = core.on_scan(dummy);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.state, sloc::LocState::TRACKING);
  EXPECT_NEAR(out.pose.x, 5.05, 0.03);
}

TEST(LocalizerCoreTest, ProgressiveDegradedWindowExpandsWithRejectStreak)
{
  class WindowGatedMatcher : public sloc::Matcher {
public:
    mutable std::vector<double> seen_wlin;
    mutable std::vector<double> seen_wang;

    sloc::MatchResult match(
      const sloc::MapField &, const sloc::Pose2 & prior,
      const sloc::Scan &, const sloc::MatchOptions & opt) const override
    {
      seen_wlin.push_back(opt.window_lin);
      seen_wang.push_back(opt.window_ang);
      sloc::MatchResult r;
      r.covariance = sloc::Cov3::Identity() * 5e-4;
      if (opt.window_lin >= 0.30) {
        r.valid = true;
        r.pose = sloc::Pose2(prior.x + 0.28, prior.y, prior.theta);
        r.score = 0.94;
        r.inlier_ratio = 0.92;
      } else {
        r.valid = true;
        r.pose = prior;
        r.score = 0.55;
        r.inlier_ratio = 0.35;
      }
      return r;
    }
  };

  const auto & wf = shared_warehouse();
  auto wm = std::make_shared<WindowGatedMatcher>();
  sloc::CoreConfig cfg;
  cfg.adaptive_window = true;
  cfg.match_opt.window_lin = 0.15;
  cfg.match_opt.window_ang = 0.15;
  cfg.odom_trans_var_per_m = 4e-4;
  cfg.odom_rot_var_per_rad = 1e-3;
  cfg.degraded_after = 5;
  cfg.lost_after = 30;
  cfg.anomaly_window = 0;

  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(wm);
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-4);

  const sloc::Scan dummy = make_dummy_scan();
  bool recovered = false;
  for (int i = 0; i < 12; ++i) {
    const auto out = core.on_scan(dummy);
    if (out.accepted) {
      recovered = true;
      break;
    }
  }
  EXPECT_TRUE(recovered);
  EXPECT_EQ(core.state(), sloc::LocState::TRACKING);
  ASSERT_GE(wm->seen_wlin.size(), 5u);
  EXPECT_LE(wm->seen_wlin.front(), 0.15 + 1e-9);
  EXPECT_GE(wm->seen_wlin.back(), 0.30 - 1e-9);
  EXPECT_GE(wm->seen_wang.back(), 0.30 - 1e-9);
}

TEST(LocalizerCoreTest, AmbiguousFarHypothesisRequiresNetMapMotionOrHighConfidence)
{
  class PinnedMirrorMatcher : public sloc::Matcher {
public:
    bool allow_advance{false};

    sloc::MatchResult match(
      const sloc::MapField &, const sloc::Pose2 & prior,
      const sloc::Scan &, const sloc::MatchOptions &) const override
    {
      sloc::MatchResult r;
      r.covariance = sloc::Cov3::Identity() * 1e-3;
      if (prior.x < 10.0) {
        r.valid = false;
        return r;
      }
      if (prior.x < 18.0) {
        r.valid = true;
        r.pose = allow_advance ? prior : sloc::Pose2(15.05, 5.0, 0.0);
        r.score = 0.80;
        r.inlier_ratio = 0.68;
      } else {
        r.valid = true;
        r.pose = prior;
        r.score = 0.45;
        r.inlier_ratio = 0.40;
      }
      return r;
    }
  };

  const auto & wf = shared_warehouse();
  auto pm = std::make_shared<PinnedMirrorMatcher>();
  sloc::Scan real_like_scan;
  for (int i = 0; i < 32; ++i) {
    const double a = -1.5 + 0.1 * i;
    real_like_scan.points.emplace_back(2.0 * std::cos(a), 2.0 * std::sin(a));
  }

  sloc::CoreConfig cfg;
  cfg.use_visibility_check = false;
  cfg.min_score = 0.35;
  cfg.min_inlier_ratio = 0.45;
  cfg.hyp_confirm_frames = 2;
  cfg.hyp_min_score = 0.70;
  cfg.hyp_min_inlier = 0.65;
  cfg.hyp_promote_posterior = 0.90;
  cfg.global_jump_min_dist = 1.0;
  cfg.anomaly_window = 0;

  sloc::LocalizerCore core(cfg);
  core.set_map(wf.field);
  core.set_matcher(pm);
  core.on_odom(sloc::Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(sloc::Pose2(5.0, 5.0, 0.0));
  ASSERT_TRUE(core.request_global(false));
  core.add_hypotheses({sloc::Pose2(15.0, 5.0, 0.0), sloc::Pose2(22.0, 5.0, 3.14159)});

  // Phase 1: odom advances 0.60m, but matcher keeps pinning hyp[0] at x=15.05 (net_disp=0.05m).
  // Mediocre score/inlier (0.80/0.68) in an ambiguous far pool must NOT promote.
  for (int f = 1; f <= 6; ++f) {
    core.on_odom(sloc::Pose2(0.10 * f, 0.0, 0.0));
    const auto out = core.on_scan(real_like_scan);
    EXPECT_FALSE(out.accepted) << "pinned mirror hyp promoted prematurely at frame " << f;
  }

  // Phase 2: once hyp[0] advances >= 0.50m on the map in agreement with odom, it promotes cleanly.
  pm->allow_advance = true;
  bool promoted = false;
  for (int f = 7; f <= 14; ++f) {
    core.on_odom(sloc::Pose2(0.10 * f, 0.0, 0.0));
    const auto out = core.on_scan(real_like_scan);
    if (out.accepted) {
      promoted = true;
      EXPECT_EQ(out.state, sloc::LocState::TRACKING);
      EXPECT_GT(out.pose.x, 15.5);
      break;
    }
  }
  EXPECT_TRUE(promoted);
}
