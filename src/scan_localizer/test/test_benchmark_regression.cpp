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
// 轻量确定性基准回归测试（< 5s）：守卫 E1 / E4 / E5 / E8 核心定量不变量
// ============================================================================
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "scan_localizer/localizer_core.hpp"
#include "scan_localizer/matchers.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

struct WarehouseEnv
{
  sloc::OccupancyGrid grid;
  std::shared_ptr<sloc::MapField> field;

  WarehouseEnv()
  : grid(sloc::sim::make_warehouse(0.05)),
    field(std::make_shared<sloc::MapField>())
  {
    field->build(grid, 0.20);
  }
};

const WarehouseEnv & shared_warehouse()
{
  static const WarehouseEnv kEnv;
  return kEnv;
}

TEST(BenchmarkRegression, E1AllFiveMatchersConvergeIncludingIcpP2p)
{
  const auto & env = shared_warehouse();
  std::mt19937 rng(101);
  sloc::sim::ScanSimConfig scfg;
  const auto poses = sloc::sim::random_free_poses(env.grid, 12, rng);
  ASSERT_EQ(poses.size(), 12u);

  std::normal_distribution<double> n_xy(0.0, 0.02);
  std::normal_distribution<double> n_th(0.0, 0.5 * sloc::kPi / 180.0);
  std::vector<sloc::Scan> scans;
  std::vector<sloc::Pose2> priors;
  for (const auto & p : poses) {
    scans.push_back(sloc::sim::make_scan(env.grid, p, scfg, rng));
    priors.emplace_back(
      p.x + n_xy(rng), p.y + n_xy(rng), sloc::wrap_angle(p.theta + n_th(rng)));
  }

  sloc::MatchOptions opt;
  const std::vector<std::pair<std::string, double>> max_med_err = {
    {"correlative", 0.06},
    {"field_gn", 0.03},
    {"icp_p2p", 0.04},
    {"ndt", 0.05},
    {"composed", 0.03},
  };
  for (const auto & [name, limit_m] : max_med_err) {
    auto m = sloc::make_matcher(name, &env.grid, 0.5);
    ASSERT_NE(m, nullptr) << name;
    std::vector<double> errs;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const auto res = m->match(*env.field, priors[i], scans[i], opt);
      errs.push_back(std::hypot(
        res.pose.x - poses[i].x, res.pose.y - poses[i].y));
    }
    std::sort(errs.begin(), errs.end());
    const double med = errs[errs.size() / 2];
    EXPECT_LT(med, limit_m) << "matcher=" << name << " med=" << med;
  }
}

TEST(BenchmarkRegression, E4DynamicObstaclesInjectedAndRobust)
{
  const auto & env = shared_warehouse();
  std::mt19937 rng(202);
  sloc::sim::ScanSimConfig scfg;
  const auto poses = sloc::sim::random_free_poses(env.grid, 10, rng);
  ASSERT_EQ(poses.size(), 10u);

  auto comp = sloc::make_matcher("composed");
  sloc::MatchOptions opt;
  std::uniform_real_distribution<double> u_dir(-sloc::kPi, sloc::kPi);
  std::uniform_real_distribution<double> u_dist(1.2, 2.5);
  std::normal_distribution<double> n_xy(0.0, 0.02);
  std::normal_distribution<double> n_th(0.0, 0.5 * sloc::kPi / 180.0);

  int ok = 0;
  int total_boxes = 0;
  for (const auto & p : poses) {
    std::vector<sloc::sim::DynBox> dyn;
    for (int b = 0; b < 2; ++b) {
      for (int a = 0; a < 32; ++a) {
        const double dir = u_dir(rng);
        const double d = u_dist(rng);
        const double bx = p.x + d * std::cos(dir);
        const double by = p.y + d * std::sin(dir);
        if (bx > 1.0 && bx < 39.0 && by > 1.0 && by < 29.0) {
          dyn.push_back({bx, by, 0.35, 0.35});
          break;
        }
      }
    }
    ASSERT_GT(dyn.size(), 0u);
    total_boxes += static_cast<int>(dyn.size());
    const auto scan = sloc::sim::make_scan(env.grid, p, scfg, rng, dyn);
    const sloc::Pose2 prior(
      p.x + n_xy(rng), p.y + n_xy(rng), sloc::wrap_angle(p.theta + n_th(rng)));
    const auto res = comp->match(*env.field, prior, scan, opt);
    const double e = std::hypot(res.pose.x - p.x, res.pose.y - p.y);
    if (e < 0.05) {++ok;}
  }
  EXPECT_EQ(total_boxes, 20);
  EXPECT_GE(ok, 8);
}

TEST(BenchmarkRegression, E5CorridorDegeneracyAndBoundedEkfDrift)
{
  const auto cgrid = sloc::sim::make_corridor(0.05);
  auto cfield = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(cfield->build(cgrid, 0.20));

  auto fgn = sloc::make_matcher("field_gn");
  auto comp = sloc::make_matcher("composed");
  sloc::sim::ScanSimConfig scfg;
  sloc::MatchOptions opt;
  std::mt19937 rng(303);

  // 严格退化中段 x=15.0：沿轴方差必须远大于横向方差
  const sloc::Pose2 gt(15.0, 1.5, 0.0);
  const auto scan = sloc::sim::make_scan(cgrid, gt, scfg, rng);
  const auto res = fgn->match(*cfield, sloc::Pose2(15.02, 1.52, 0.01), scan, opt);
  EXPECT_TRUE(res.valid);
  EXPECT_GT(res.covariance(0, 0), 10.0 * res.covariance(1, 1));
  EXPECT_GT(res.covariance(0, 0), 1.0);
  EXPECT_LT(std::abs(res.pose.y - gt.y), 0.02);

  // 各向异性 EKF 沿走廊行走 4m（2% 轮滑）：横向误差 < 2cm，沿轴漂移有界 (< 0.25m)
  sloc::CoreConfig cc;
  sloc::LocalizerCore core(cc);
  core.set_map(cfield);
  core.set_matcher(comp);
  sloc::Pose2 cur(13.0, 1.5, 0.0);
  sloc::Pose2 odom = cur;
  core.on_odom(odom);
  core.set_initial_pose(cur);
  for (int s = 0; s < 16; ++s) {
    cur.x += 0.25;
    odom = odom * sloc::Pose2(0.25 * 1.02, 0.0, 0.0);
    core.on_odom(odom);
    const auto sc = sloc::sim::make_scan(cgrid, cur, scfg, rng);
    const auto outp = core.on_scan(sc);
    EXPECT_LT(std::abs(outp.pose.y - cur.y), 0.03);
    EXPECT_LT(std::abs(outp.pose.x - cur.x), 0.25);
  }
}

TEST(BenchmarkRegression, E8GlobalCorrelativeRecall)
{
  const auto & env = shared_warehouse();
  std::mt19937 rng(404);
  sloc::sim::ScanSimConfig scfg;
  const auto poses = sloc::sim::random_free_poses(env.grid, 8, rng);
  ASSERT_EQ(poses.size(), 8u);

  int ok = 0;
  for (const auto & p : poses) {
    const auto scan = sloc::sim::make_scan(env.grid, p, scfg, rng);
    const auto g = sloc::global_correlative(
      *env.field, scan, 3, 2.0 * sloc::kPi / 180.0, 5);
    if (g.valid) {
      const double e = std::hypot(g.pose.x - p.x, g.pose.y - p.y);
      const double a = std::abs(sloc::wrap_angle(g.pose.theta - p.theta));
      if (e < 0.3 && a < 3.0 * sloc::kPi / 180.0) {++ok;}
    }
  }
  EXPECT_GE(ok, 6);
}

}  // namespace
