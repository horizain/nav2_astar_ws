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
// 六场景泛化回归矩阵：强制使用同一套未修改的默认 CoreConfig{} 通过全部场景
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "scan_localizer/localizer_core.hpp"
#include "scan_localizer/map_field.hpp"
#include "scan_localizer/sim_world.hpp"

namespace sloc::sim
{
namespace
{

TEST(GeneralizationMatrixTest, S1_WarehouseColdStartSingleFrameAndSlipTracking)
{
  const OccupancyGrid grid = make_warehouse(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(42);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 12.0;

  Pose2 gt(10.0, 8.5, 0.0);
  Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);

  // 冷启动 UNINIT：仓库非对称区单帧直接锁定
  const auto scan0 = make_scan(grid, gt, scfg, rng);
  const auto out0 = core.on_scan(scan0);
  EXPECT_TRUE(out0.accepted);
  EXPECT_EQ(out0.state, LocState::TRACKING);
  EXPECT_LT(std::hypot(out0.pose.x - gt.x, out0.pose.y - gt.y), 0.10);

  for (int i = 0; i < 20; ++i) {
    gt.x -= 0.20;
    odom = odom * Pose2(-0.20 * 1.02, 0.0, 0.0);
    core.on_odom(odom);
    const auto scan = make_scan(grid, gt, scfg, rng);
    const auto out = core.on_scan(scan);
    EXPECT_TRUE(out.accepted);
    EXPECT_EQ(out.state, LocState::TRACKING);
  }
  const auto final_out = core.pose();
  EXPECT_LT(std::hypot(final_out.x - gt.x, final_out.y - gt.y), 0.08);
}

TEST(GeneralizationMatrixTest, S2_CorridorStrictDegeneracyBoundedDrift)
{
  const OccupancyGrid grid = make_corridor(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(1337);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 12.0;

  Pose2 gt(13.0, 1.5, 0.0);
  Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);
  core.set_initial_pose(gt);

  for (int i = 0; i < 16; ++i) {
    gt.x += 0.25;
    odom = odom * Pose2(0.25 * 1.02, 0.0, 0.0);
    core.on_odom(odom);
    const auto scan = make_scan(grid, gt, scfg, rng);
    const auto out = core.on_scan(scan);
    EXPECT_TRUE(out.accepted);
    EXPECT_NE(out.state, LocState::LOST);
    EXPECT_LT(std::abs(out.pose.y - 1.5), 0.04);
    EXPECT_LT(std::abs(out.pose.x - gt.x), 0.25);
  }
}

TEST(GeneralizationMatrixTest, S3_TwinRoomsHoldsSymmetryThenResolvesAtDoorway)
{
  const OccupancyGrid grid = make_twin_rooms(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(999);
  ScanSimConfig scfg;
  scfg.beams = 270;
  scfg.fov = sloc::kPi;  // 180° 前视雷达：朝南时左右孪生房几何完全一致
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 12.0;

  Pose2 gt(5.0, 4.5, -0.5 * sloc::kPi);
  Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);

  // 冷启动第 0 帧处于全对称视角：必须保持 GLOBAL 多假设，禁止单帧盲锁错房
  const auto scan0 = make_scan(grid, gt, scfg, rng);
  const auto out0 = core.on_scan(scan0);
  EXPECT_FALSE(out0.accepted);
  EXPECT_EQ(out0.state, LocState::GLOBAL);
  EXPECT_GE(core.hypothesis_count(), 2u);

  const std::vector<Pose2> waypoints = {
    {5.0, 4.5, -0.5 * sloc::kPi},
    {5.0, 4.5, 0.0},
    {5.0, 4.5, 0.5 * sloc::kPi},
    {5.0, 5.5, 0.5 * sloc::kPi},
    {5.0, 6.5, 0.5 * sloc::kPi},
    {5.5, 7.5, 0.3},
    {6.2, 8.0, 0.2},
    {6.8, 8.2, 0.1},
    {7.2, 8.2, 0.0},
    {7.6, 8.2, 0.0},
    {7.9, 8.2, 0.0},
  };

  Pose2 last_gt = waypoints[0];
  CoreOutput last_out = out0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const Pose2 delta = last_gt.inverse() * waypoints[i];
    odom = odom * delta;
    core.on_odom(odom);
    gt = waypoints[i];
    last_gt = gt;

    const auto scan = make_scan(grid, gt, scfg, rng);
    last_out = core.on_scan(scan);
  }
  EXPECT_EQ(last_out.state, LocState::TRACKING);
  EXPECT_LT(std::hypot(last_out.pose.x - gt.x, last_out.pose.y - gt.y), 0.15);
  EXPECT_LT(last_out.pose.x, 10.0);  // 锁定在左房走廊段，绝非右孪生房
}

TEST(GeneralizationMatrixTest, S4_OpenCorridorRejects180DegreeUnknownFlip)
{
  const OccupancyGrid grid = make_open_corridor(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(777);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 20.0;

  Pose2 gt(-1.6, 0.0, sloc::kPi);
  Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);

  bool reached_tracking = false;
  for (int step = 0; step < 4; ++step) {
    if (step > 0) {
      gt.x -= 0.25;
      odom = odom * Pose2(0.25, 0.0, 0.0);  // 朝向 pi 时前进 +0.25 即世界系 -x
      core.on_odom(odom);
    }
    const auto scan = make_scan(grid, gt, scfg, rng);
    const auto out = core.on_scan(scan);
    if (out.state == LocState::TRACKING) {
      reached_tracking = true;
      EXPECT_LT(std::abs(sloc::wrap_angle(out.pose.theta - gt.theta)), 0.15);
      EXPECT_LT(std::hypot(out.pose.x - gt.x, out.pose.y - gt.y), 0.20);
      break;
    }
  }
  EXPECT_TRUE(reached_tracking);
}

TEST(GeneralizationMatrixTest, S5_ClutteredCorridorStaticGhostsAndDynamicCrowd)
{
  const OccupancyGrid map_grid = make_cluttered_corridor(0.05, true);
  const OccupancyGrid real_grid = make_cluttered_corridor(0.05, false);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(map_grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(555);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 12.0;

  Pose2 gt(5.5, 2.0, 0.0);
  Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);
  core.set_initial_pose(gt);

  for (int step = 0; step < 36; ++step) {
    gt.x += 0.25;
    odom = odom * Pose2(0.25 * 1.02, 0.0, 0.0);
    core.on_odom(odom);

    const std::vector<DynBox> dyn = {
      {gt.x + 1.8, 1.4, 0.25, 0.25},
      {gt.x - 1.5, 2.6, 0.25, 0.25}
    };

    const auto scan = make_scan(real_grid, gt, scfg, rng, dyn);
    const auto out = core.on_scan(scan);
    EXPECT_NE(out.state, LocState::LOST);
    if (step == 35) {
      EXPECT_EQ(out.state, LocState::TRACKING);
      EXPECT_LT(std::hypot(out.pose.x - gt.x, out.pose.y - gt.y), 0.12);
    }
  }
}

TEST(GeneralizationMatrixTest, S6_OpenHallSparsePillarsAndMaxRangeMisses)
{
  const OccupancyGrid grid = make_open_hall(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(333);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 12.0;

  Pose2 gt(11.0, 11.0, 0.0);
  Pose2 odom(0.0, 0.0, 0.0);
  core.on_odom(odom);

  // 冷启动 UNINIT + 开阔大厅稀疏柱阵(>50% 光束超量程漏射)连续跟踪
  const auto scan0 = make_scan(grid, gt, scfg, rng);
  const auto out0 = core.on_scan(scan0);
  EXPECT_EQ(out0.state, LocState::TRACKING);
  EXPECT_LT(std::hypot(out0.pose.x - gt.x, out0.pose.y - gt.y), 0.15);

  for (int step = 0; step < 20; ++step) {
    gt.x += 0.25;
    gt.y += 0.10;
    odom = odom * Pose2(0.25 * 1.02, 0.10 * 1.02, 0.0);
    core.on_odom(odom);

    const auto scan = make_scan(grid, gt, scfg, rng);
    const auto out = core.on_scan(scan);
    EXPECT_TRUE(out.accepted);
    EXPECT_NE(out.state, LocState::LOST);
    if (step == 19) {
      EXPECT_EQ(out.state, LocState::TRACKING);
      EXPECT_LT(std::hypot(out.pose.x - gt.x, out.pose.y - gt.y), 0.10);
    }
  }
}

TEST(GeneralizationMatrixTest, S7_ZombieHypothesisEvictionDoesNotBlockFreshCandidate)
{
  const OccupancyGrid grid = make_warehouse(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  const sloc::CoreConfig kDefaultCfg{};
  auto matcher = sloc::make_matcher("composed");
  sloc::LocalizerCore core(kDefaultCfg);
  core.set_map(field);
  core.set_matcher(matcher);
  core.enable_builtin_global(true);

  std::mt19937 rng(111);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.subsample = 1;
  scfg.noise_std = 0.01;
  scfg.range_max = 12.0;

  std::vector<Pose2> bad_poses;
  for (int i = 0; i < 8; ++i) {
    bad_poses.emplace_back(1.5 + i * 2.0, 2.0, 0.0);
  }
  core.add_hypotheses(bad_poses);

  const Pose2 gt(10.0, 8.5, 0.3);
  core.on_odom(Pose2(0.0, 0.0, 0.0));
  const auto scan_true = make_scan(grid, gt, scfg, rng);

  // 僵尸假设首轮不达标后自动清空并触发内置全局重定位，绝不卡死
  const auto out = core.on_scan(scan_true);
  EXPECT_EQ(out.state, LocState::TRACKING);
  EXPECT_LT(std::hypot(out.pose.x - gt.x, out.pose.y - gt.y), 0.15);
}

TEST(GeneralizationMatrixTest, S8_TiltedStaircaseCorridorAndComposedZeroJump)
{
  // 1) ComposedMatcher 在严格退化走廊不得出现粗搜无约束 18cm 乱跳
  const OccupancyGrid cgrid = make_corridor(0.05);
  auto cfield = std::make_shared<MapField>();
  ASSERT_TRUE(cfield->build(cgrid, 0.20));
  auto comp = sloc::make_matcher("composed");

  std::mt19937 rng(808);
  ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.005;
  const Pose2 gt(15.0, 1.5, 0.0);
  const auto cscan = make_scan(cgrid, gt, scfg, rng);
  const Pose2 prior(15.03, 1.52, 0.01);
  const auto cres = comp->match(*cfield, prior, cscan, MatchOptions{});
  ASSERT_TRUE(cres.valid);
  EXPECT_LT(std::abs(cres.pose.x - prior.x), 0.03);
  EXPECT_LT(std::abs(cres.pose.y - gt.y), 0.02);

  // 2) 18° 斜交栅格锯齿长走廊（30m 长使两端墙超 12m 量程不可见，检验无 cond>6 补丁下的旋转不变性）
  OccupancyGrid tgrid;
  tgrid.resolution = 0.05;
  tgrid.width = 720;
  tgrid.height = 360;
  tgrid.origin_x = 0.0;
  tgrid.origin_y = 0.0;
  tgrid.cells.assign(static_cast<std::size_t>(tgrid.width) * tgrid.height, -1);
  const double th = 18.0 * sloc::kPi / 180.0;
  const double ct = std::cos(th);
  const double st = std::sin(th);
  for (int y = 0; y < tgrid.height; ++y) {
    for (int x = 0; x < tgrid.width; ++x) {
      const auto w = tgrid.gridToWorld(x, y);
      const double dx = w.x() - 2.0;
      const double dy = w.y() - 2.0;
      const double u = ct * dx + st * dy;
      const double v = -st * dx + ct * dy;
      if (u >= 0.0 && u <= 30.0 && std::abs(v) <= 1.25) {
        tgrid.cells[static_cast<std::size_t>(y) * tgrid.width + x] = 0;
      } else if (u >= -0.15 && u <= 30.15 && std::abs(v) <= 1.40) {
        tgrid.cells[static_cast<std::size_t>(y) * tgrid.width + x] = 100;
      }
    }
  }

  auto tfield = std::make_shared<MapField>();
  ASSERT_TRUE(tfield->build(tgrid, 0.20));

  const Pose2 tgt(2.0 + ct * 15.0, 2.0 + st * 15.0, th);
  const auto tscan = make_scan(tgrid, tgt, scfg, rng);
  const auto tres = comp->match(*tfield, tgt, tscan, MatchOptions{});
  ASSERT_TRUE(tres.valid);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(tres.covariance.topLeftCorner<2, 2>());
  ASSERT_EQ(es.info(), Eigen::Success);
  EXPECT_GT(es.eigenvalues()(1), 10.0 * es.eigenvalues()(0));
}

TEST(GeneralizationMatrixTest, S9_FarSymmetricJumpMahalRejectionDoesNotInflateCov)
{
  class FarJumpMatcher : public sloc::Matcher {
public:
    sloc::MatchResult match(
      const sloc::MapField &, const sloc::Pose2 & prior,
      const sloc::Scan &, const sloc::MatchOptions &) const override
    {
      sloc::MatchResult r;
      r.valid = true;
      r.pose = sloc::Pose2(prior.x + 1.8, prior.y, prior.theta);
      r.score = 0.92;
      r.inlier_ratio = 0.95;
      r.covariance = sloc::Cov3::Identity() * 1e-3;
      return r;
    }
  };

  const OccupancyGrid grid = make_warehouse(0.05);
  auto field = std::make_shared<MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  sloc::LocalizerCore core(sloc::CoreConfig{});
  core.set_map(field);
  core.set_matcher(std::make_shared<FarJumpMatcher>());
  core.on_odom(Pose2(0.0, 0.0, 0.0));
  core.set_initial_pose(Pose2(5.0, 5.0, 0.0), sloc::Cov3::Identity() * 1e-3);

  Scan dummy;
  for (int i = 0; i < 8; ++i) {
    dummy.points.emplace_back(1.0 + 0.1 * i, 0.0);
  }
  const double trace_before = core.last_cov().trace();
  const auto out = core.on_scan(dummy);
  EXPECT_FALSE(out.accepted);
  // 远距离跨峰马氏拒绝绝不能触发旧版 P*=2.5 协方差膨胀
  EXPECT_NEAR(core.last_cov().trace(), trace_before, 1e-9);
}

}  // namespace
}  // namespace sloc::sim
