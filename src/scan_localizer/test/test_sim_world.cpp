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
#include <random>
#include <vector>

#include "scan_localizer/sim_world.hpp"

namespace
{

TEST(SimWorldTest, SymmetricExteriorUnknownBorder)
{
  const auto wh = sloc::sim::make_warehouse(0.05);
  ASSERT_EQ(wh.width, 800);
  ASSERT_EQ(wh.height, 600);
  // 四边外缘第 0 行/列与末行/列均应为未知(-1)
  for (int x = 0; x < wh.width; ++x) {
    EXPECT_EQ(wh.cells[static_cast<std::size_t>(x)], -1);
    EXPECT_EQ(
      wh.cells[static_cast<std::size_t>(wh.height - 1) * wh.width + x], -1);
  }
  for (int y = 0; y < wh.height; ++y) {
    EXPECT_EQ(wh.cells[static_cast<std::size_t>(y) * wh.width], -1);
    EXPECT_EQ(
      wh.cells[static_cast<std::size_t>(y) * wh.width + (wh.width - 1)], -1);
  }
  // 紧邻内侧的第 1 圈应为外墙占据(100)
  EXPECT_EQ(wh.cells[static_cast<std::size_t>(1) * wh.width + 1], 100);
  EXPECT_EQ(
    wh.cells[static_cast<std::size_t>(wh.height - 2) * wh.width +
    (wh.width - 2)], 100);
}

TEST(SimWorldTest, RaycastAccuracyAndRangeClamp)
{
  const auto corr = sloc::sim::make_corridor(0.05);
  // 走廊宽 3m，墙体在 cell 1 ([0.05, 0.10]) 与 cell 58 ([2.90, 2.95])
  // 站在 (15.0, 1.5) 朝 +y (pi/2) 射线应打到 y=2.90（距离 1.40m）
  const double d_up = sloc::sim::raycast_range(
    corr, 15.0, 1.5, 0.5 * sloc::kPi, 0.10, 12.0);
  EXPECT_NEAR(d_up, 1.40, 0.01);

  // 朝 -y (-pi/2) 射线应打到 y=0.10（距离 1.40m）
  const double d_dn = sloc::sim::raycast_range(
    corr, 15.0, 1.5, -0.5 * sloc::kPi, 0.10, 12.0);
  EXPECT_NEAR(d_dn, 1.40, 0.01);

  // 朝 +x (0) 在中段距端墙 14.9m > range_max=12.0m，应未命中(-1)
  const double d_along = sloc::sim::raycast_range(
    corr, 15.0, 1.5, 0.0, 0.10, 12.0);
  EXPECT_LT(d_along, 0.0);

  // 大噪声下所有输出点均严格落在 [min_range, range_max] 内
  std::mt19937 rng(123);
  sloc::sim::ScanSimConfig cfg;
  cfg.beams = 360;
  cfg.noise_std = 0.5;
  cfg.min_range = 0.2;
  cfg.range_max = 1.6;
  const auto scan = sloc::sim::make_scan(corr, sloc::Pose2(15.0, 1.5, 0.0), cfg, rng);
  ASSERT_GT(scan.points.size(), 10u);
  for (const auto & pt : scan.points) {
    const double r = pt.norm();
    EXPECT_GE(r, cfg.min_range - 1e-9);
    EXPECT_LE(r, cfg.range_max + 1e-9);
  }
}

TEST(SimWorldTest, DynBoxOcclusionAndZeroNoise)
{
  const auto corr = sloc::sim::make_corridor(0.05);
  std::mt19937 rng(7);
  sloc::sim::ScanSimConfig cfg;
  cfg.beams = 360;
  cfg.noise_std = 0.0;  // 必须安全跳过 normal_distribution，无 UB
  cfg.range_max = 12.0;

  const sloc::Pose2 pose(15.0, 1.5, 0.0);
  const std::vector<sloc::sim::DynBox> dyn = {{17.0, 1.5, 0.3, 0.3}};

  // 沿 +x 无动态箱时未命中，放上动态箱 (cx=17.0, hw=0.3 -> 近面 x=16.7) 后距离 1.7m
  const double d_dyn = sloc::sim::raycast_range(
    corr, pose.x, pose.y, 0.0, 0.10, 12.0, dyn);
  EXPECT_NEAR(d_dyn, 1.70, 0.01);

  const auto scan_clean = sloc::sim::make_scan(corr, pose, cfg, rng);
  const auto scan_dyn = sloc::sim::make_scan(corr, pose, cfg, rng, dyn);
  // 注入近处动态箱后，原本沿 +x 未命中的光束被遮挡命中，总点数应增多
  EXPECT_GT(scan_dyn.points.size(), scan_clean.points.size());

  // make_laser_ranges 直接按 beam 角采样，beam 180 对应 angle = 0 rad
  const double angle_min = -sloc::kPi;
  const double angle_inc = sloc::kTwoPi / 360.0;
  const auto ranges = sloc::sim::make_laser_ranges(
    corr, pose, angle_min, angle_inc, 360, 0.10, 12.0, 0.0, rng, dyn);
  ASSERT_EQ(ranges.size(), 360u);
  EXPECT_NEAR(ranges[180], 1.70f, 0.01f);
  // beam 0 对应 -pi（朝 -x，距端墙 14.9m > 12m），应为 +inf
  EXPECT_TRUE(std::isinf(ranges[0]));
}

TEST(SimWorldTest, RandomFreePosesNonZeroOriginAndCorridorBoundedExit)
{
  std::mt19937 rng(42);
  // 1) 非零原点地图
  auto shifted = sloc::sim::make_warehouse(0.05);
  shifted.origin_x = -25.0;
  shifted.origin_y = -15.0;
  const auto poses = sloc::sim::random_free_poses(shifted, 20, rng, 1.0, 20000);
  ASSERT_EQ(poses.size(), 20u);
  for (const auto & p : poses) {
    EXPECT_GE(p.x, shifted.origin_x + 1.0);
    EXPECT_LE(p.x, shifted.origin_x + shifted.width * shifted.resolution - 1.0);
    EXPECT_GE(p.y, shifted.origin_y + 1.0);
    EXPECT_LE(p.y, shifted.origin_y + shifted.height * shifted.resolution - 1.0);
    const auto idx = shifted.worldToGridIndex(p.x, p.y);
    ASSERT_TRUE(shifted.inside(idx.x(), idx.y()));
    EXPECT_EQ(
      shifted.cells[static_cast<std::size_t>(idx.y()) * shifted.width + idx.x()],
      0);
  }

  // 2) make_corridor（总宽 3m）配默认 1.5m 净空必须有界快速退出（返回空而非死循环）
  const auto corr = sloc::sim::make_corridor(0.05);
  const auto impossible = sloc::sim::random_free_poses(corr, 10, rng, 1.5, 5000);
  EXPECT_TRUE(impossible.empty());

  // 3) 走廊配 0.8m 净空可正常采样
  const auto corr_poses = sloc::sim::random_free_poses(corr, 10, rng, 0.8, 5000);
  EXPECT_EQ(corr_poses.size(), 10u);
}

TEST(SimWorldTest, WarehouseTrajectoryLengthAndContinuity)
{
  const auto traj = sloc::sim::make_warehouse_trajectory(2, 0.25);
  ASSERT_EQ(traj.size(), 417u);  // 起点 + 2 圈 × 208 步 = 417 位姿
  double total_len = 0.0;
  for (std::size_t i = 1; i < traj.size(); ++i) {
    const double step = std::hypot(
      traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);
    EXPECT_NEAR(step, 0.25, 1e-6);
    total_len += step;
  }
  EXPECT_NEAR(total_len, 104.0, 1e-6);
}

}  // namespace
