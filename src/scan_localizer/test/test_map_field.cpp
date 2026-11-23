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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "scan_localizer/map_field.hpp"

namespace
{

sloc::OccupancyGrid make_grid(int w, int h, double res = 0.1, int8_t fill = 0)
{
  sloc::OccupancyGrid g;
  g.width = w;
  g.height = h;
  g.resolution = res;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), fill);
  return g;
}

TEST(MapFieldTest, Edt1dNegativeNumeratorCounterexample)
{
  // Classic counterexample where truncating division (-1 / 4 == 0) yields 9 instead of 8
  const std::vector<std::int64_t> f = {9, 100, 4};
  std::vector<std::int64_t> out;
  sloc::edt1d(f, out);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_EQ(out[0], 8);
  EXPECT_EQ(out[1], 5);
  EXPECT_EQ(out[2], 4);
}

TEST(MapFieldTest, Edt1dLargeInt64NoOverflow)
{
  // n = 60000: (59999)^2 = 3599880001 > INT32_MAX (2147483647)
  constexpr std::size_t kN = 60000;
  constexpr std::int64_t kInf = std::int64_t{1} << 50;
  std::vector<std::int64_t> f(kN, kInf);
  f[kN - 1] = 0;

  std::vector<std::int64_t> out;
  sloc::edt1d(f, out);
  ASSERT_EQ(out.size(), kN);
  const std::int64_t d0 = static_cast<std::int64_t>(kN - 1);
  EXPECT_EQ(out[0], d0 * d0);
  EXPECT_EQ(out[kN - 1], 0);
  EXPECT_EQ(out[kN - 2], 1);
}

TEST(MapFieldTest, UnifiedRosStandardOriginGeometry)
{
  sloc::OccupancyGrid g = make_grid(10, 10, 0.2);
  g.origin_x = -3.0;
  g.origin_y = 4.5;

  // Cell (ix, iy) center = origin + (i + 0.5) * res
  const auto center = g.gridToWorld(3, 5);
  EXPECT_DOUBLE_EQ(center.x(), -3.0 + 3.5 * 0.2);
  EXPECT_DOUBLE_EQ(center.y(), 4.5 + 5.5 * 0.2);

  // worldToGridF is exact inverse of gridToWorld
  const auto gf = g.worldToGridF(center.x(), center.y());
  EXPECT_NEAR(gf.x(), 3.0, 1e-12);
  EXPECT_NEAR(gf.y(), 5.0, 1e-12);

  // worldToGridIndex floors [origin + i*res, origin + (i+1)*res) -> i
  const auto idx_lo = g.worldToGridIndex(-3.0 + 3.01 * 0.2, 4.5 + 5.01 * 0.2);
  const auto idx_hi = g.worldToGridIndex(-3.0 + 3.99 * 0.2, 4.5 + 5.99 * 0.2);
  EXPECT_EQ(idx_lo.x(), 3);
  EXPECT_EQ(idx_lo.y(), 5);
  EXPECT_EQ(idx_hi.x(), 3);
  EXPECT_EQ(idx_hi.y(), 5);
}

TEST(MapFieldTest, SdfSignAndCentralDifferenceGradientAccuracy)
{
  // Vertical thick wall at x = 0..4, free space at x = 5..19
  sloc::OccupancyGrid g = make_grid(20, 15, 0.1, 0);
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x <= 4; ++x) {
      g.cells[static_cast<std::size_t>(y) * g.width + x] = 100;
    }
  }

  sloc::MapField field;
  ASSERT_TRUE(field.build(g, 0.20));
  ASSERT_TRUE(field.valid());

  // Surface (x=4) == 0, free (x=8) == +0.4, wall interior (x=1) == -0.3
  EXPECT_NEAR(field.sdf(4.0, 7.0), 0.0, 1e-5);
  EXPECT_NEAR(field.sdf(8.0, 7.0), 0.4, 1e-5);
  EXPECT_NEAR(field.sdf(1.0, 7.0), -0.3, 1e-5);

  // Compare analytic bilinear gradient against central difference (< 1e-3)
  const double eps_g = 1e-3;
  const double eps_m = eps_g * g.resolution;
  const std::vector<Eigen::Vector2d> pts = {
    {6.3, 7.4},
    {4.5, 7.2},
    {2.2, 7.6},
  };
  for (const auto & pt : pts) {
    Eigen::Vector2d grad;
    field.sdf(pt.x(), pt.y(), &grad);
    const double dsdx =
      (field.sdf(pt.x() + eps_g, pt.y()) - field.sdf(pt.x() - eps_g, pt.y())) /
      (2.0 * eps_m);
    const double dsdy =
      (field.sdf(pt.x(), pt.y() + eps_g) - field.sdf(pt.x(), pt.y() - eps_g)) /
      (2.0 * eps_m);
    EXPECT_NEAR(grad.x(), dsdx, 1e-3);
    EXPECT_NEAR(grad.y(), dsdy, 1e-3);
    EXPECT_NEAR(grad.x(), 1.0, 1e-3);
    EXPECT_NEAR(grad.y(), 0.0, 1e-3);
  }
}

TEST(MapFieldTest, ConfigurableThresholds)
{
  sloc::OccupancyGrid g = make_grid(5, 1, 0.1, 0);
  g.cells = {0, 20, 50, 80, 100};

  // With occupied_thresh=40, free_thresh=20: cell 2 (50) is occupied (surface sdf == 0)
  sloc::MapField f1;
  ASSERT_TRUE(f1.build(g, 0.2, 40, 20));
  EXPECT_NEAR(f1.sdf(2.0, 0.0), 0.0, 1e-5);
  EXPECT_GT(f1.sdf(1.0, 0.0), 0.0);

  // With occupied_thresh=85, free_thresh=25: cell 2 (50) & cell 3 (80) are unknown (< 0)
  sloc::MapField f2;
  ASSERT_TRUE(f2.build(g, 0.2, 85, 25));
  EXPECT_LT(f2.sdf(2.0, 0.0), 0.0);
  EXPECT_NEAR(f2.likelihood(2, 0), f2.unknown_score_, 1e-6);
}

TEST(MapFieldTest, UnknownCellsAndAllUnknownMap)
{
  // 1) All-unknown map: SDF must NOT be 0 and NOT be positive
  {
    sloc::OccupancyGrid g = make_grid(8, 8, 0.1, -1);
    sloc::MapField field;
    ASSERT_TRUE(field.build(g, 0.20));
    for (int y = 0; y < g.height; ++y) {
      for (int x = 0; x < g.width; ++x) {
        const double d = field.sdf(x, y);
        EXPECT_LT(d, 0.0);
        EXPECT_TRUE(std::isfinite(d));
        EXPECT_NEAR(field.likelihood(x, y), field.unknown_score_, 1e-6);
      }
    }
  }

  // 2) Mixed map with unknown region: unknown cells never have positive SDF
  {
    sloc::OccupancyGrid g = make_grid(12, 12, 0.1, 0);
    for (int y = 0; y < 12; ++y) {
      g.cells[static_cast<std::size_t>(y) * 12 + 5] = 100;
      for (int x = 6; x < 12; ++x) {
        g.cells[static_cast<std::size_t>(y) * 12 + x] = -1;
      }
    }
    sloc::MapField field;
    ASSERT_TRUE(field.build(g, 0.20));
    for (int y = 0; y < 12; ++y) {
      for (int x = 6; x < 12; ++x) {
        EXPECT_LT(field.sdf(x, y), 0.0);
        EXPECT_NEAR(field.likelihood(x, y), field.unknown_score_, 1e-6);
      }
    }
  }
}

TEST(MapFieldTest, DescendToObstacleFromFreeAndThickWallInterior)
{
  // Thick wall at x = 0..5, free space at x = 6..15
  sloc::OccupancyGrid g = make_grid(16, 10, 0.1, 0);
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x <= 5; ++x) {
      g.cells[static_cast<std::size_t>(y) * g.width + x] = 100;
    }
  }
  sloc::MapField field;
  ASSERT_TRUE(field.build(g, 0.20));

  // From free space (sdf > 0) -> lands on surface x = 5
  const auto from_free = field.descendToObstacle(12, 5);
  EXPECT_EQ(from_free.x(), 5);
  EXPECT_TRUE(field.grid().occupied(from_free.x(), from_free.y()));

  // From deep inside thick wall (sdf < 0) -> ascends to surface x = 5
  const auto from_inside = field.descendToObstacle(1, 5);
  EXPECT_EQ(from_inside.x(), 5);
  EXPECT_TRUE(field.grid().occupied(from_inside.x(), from_inside.y()));
  EXPECT_NEAR(field.sdf(from_inside.x(), from_inside.y()), 0.0, 1e-5);
}

TEST(MapFieldTest, SwmaxUpperBoundInvariantAndLevelCap)
{
  sloc::OccupancyGrid g = make_grid(17, 13, 0.1, 0);
  g.cells[3 * 17 + 4] = 100;
  g.cells[9 * 17 + 12] = 100;
  g.cells[1 * 17 + 15] = -1;

  sloc::MapField field;
  ASSERT_TRUE(field.build(g, 0.20));
  ASSERT_GT(field.poolLevels(), 1);

  for (int k = 0; k < field.poolLevels(); ++k) {
    const int win = field.poolFactor(k);
    for (int y = 0; y < g.height; ++y) {
      for (int x = 0; x < g.width; ++x) {
        double max_like = 0.0;
        const int x_end = std::min(x + win, g.width);
        const int y_end = std::min(y + win, g.height);
        for (int yy = y; yy < y_end; ++yy) {
          for (int xx = x; xx < x_end; ++xx) {
            max_like = std::max(max_like, field.likelihood(xx, yy));
          }
        }
        EXPECT_GE(field.poolScore(k, x, y) + 1e-6, max_like)
          << "level=" << k << " x=" << x << " y=" << y;
      }
    }
  }

  // Max pool levels cap
  sloc::MapField capped;
  ASSERT_TRUE(capped.build(g, 0.20, 65, 25, 2));
  EXPECT_EQ(capped.poolLevels(), 2);
}

TEST(MapFieldTest, DegenerateAndInvalidInputsGuarded)
{
  sloc::MapField field;
  EXPECT_FALSE(field.valid());

  // 0x0 grid
  sloc::OccupancyGrid empty;
  EXPECT_FALSE(field.build(empty, 0.20));
  EXPECT_FALSE(field.valid());

  // sigma <= 0
  sloc::OccupancyGrid g = make_grid(4, 4, 0.1, 0);
  EXPECT_FALSE(field.build(g, 0.0));
  EXPECT_FALSE(field.build(g, -0.1));
  EXPECT_FALSE(field.valid());

  // resolution <= 0
  g.resolution = 0.0;
  EXPECT_FALSE(field.build(g, 0.20));
  EXPECT_FALSE(field.valid());

  // cells size mismatch
  g = make_grid(4, 4, 0.1, 0);
  g.cells.pop_back();
  EXPECT_FALSE(field.build(g, 0.20));
  EXPECT_FALSE(field.valid());

  // 1x1 grid: sdf() must not read OOB (x0 = -1)
  sloc::OccupancyGrid g1 = make_grid(1, 1, 0.1, 100);
  ASSERT_TRUE(field.build(g1, 0.20));
  Eigen::Vector2d grad(1.0, 1.0);
  const double d1 = field.sdf(0.0, 0.0, &grad);
  EXPECT_TRUE(std::isfinite(d1));
  EXPECT_DOUBLE_EQ(grad.x(), 0.0);
  EXPECT_DOUBLE_EQ(grad.y(), 0.0);

  // NaN / Inf queries to sdf()
  const double d_nan = field.sdf(
    std::numeric_limits<double>::quiet_NaN(), 0.0, &grad);
  EXPECT_GT(d_nan, 1e100);
  EXPECT_DOUBLE_EQ(grad.x(), 0.0);
  EXPECT_DOUBLE_EQ(grad.y(), 0.0);
}

TEST(MapFieldTest, NoUseAfterFreeWhenCallerGridDestroyed)
{
  sloc::MapField field;
  {
    sloc::OccupancyGrid local_grid = make_grid(10, 10, 0.1, 0);
    local_grid.cells[5 * 10 + 5] = 100;
    ASSERT_TRUE(field.build(local_grid, 0.20));
    // Overwrite local_grid before destruction to catch any aliasing
    local_grid.width = 0;
    local_grid.height = 0;
    local_grid.cells.clear();
  }
  // local_grid is now destroyed; field must remain completely valid & safe
  ASSERT_TRUE(field.valid());
  EXPECT_EQ(field.grid().width, 10);
  EXPECT_EQ(field.grid().height, 10);
  EXPECT_NEAR(field.sdf(5.0, 5.0), 0.0, 1e-5);
  EXPECT_GT(field.sdf(8.0, 5.0), 0.25);
  EXPECT_NEAR(field.likelihood(5, 5), 1.0, 1e-5);
  const auto ob = field.descendToObstacle(8, 5);
  EXPECT_EQ(ob.x(), 5);
  EXPECT_EQ(ob.y(), 5);
  EXPECT_NEAR(field.poolScore(0, 5, 5), 1.0, 1e-5);
}

}  // namespace
