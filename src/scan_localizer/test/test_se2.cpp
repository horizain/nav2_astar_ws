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
#include <vector>

#include "scan_localizer/se2.hpp"

namespace
{

constexpr double kEps = 1e-12;

TEST(Se2Test, DefaultAndParameterizedPose)
{
  const sloc::Pose2 identity;
  EXPECT_DOUBLE_EQ(identity.x, 0.0);
  EXPECT_DOUBLE_EQ(identity.y, 0.0);
  EXPECT_DOUBLE_EQ(identity.theta, 0.0);
  EXPECT_DOUBLE_EQ(identity.trans().x(), 0.0);
  EXPECT_DOUBLE_EQ(identity.trans().y(), 0.0);

  const sloc::Pose2 p(1.5, -2.25, 0.5);
  EXPECT_DOUBLE_EQ(p.x, 1.5);
  EXPECT_DOUBLE_EQ(p.y, -2.25);
  EXPECT_DOUBLE_EQ(p.theta, 0.5);
  EXPECT_DOUBLE_EQ(p.trans().x(), 1.5);
  EXPECT_DOUBLE_EQ(p.trans().y(), -2.25);
}

TEST(Se2Test, CompositionAndAssociativity)
{
  const sloc::Pose2 p1(1.0, 2.0, M_PI / 2.0);
  const sloc::Pose2 p2(3.0, 0.0, M_PI / 4.0);
  const sloc::Pose2 p12 = p1 * p2;

  EXPECT_NEAR(p12.x, 1.0, kEps);
  EXPECT_NEAR(p12.y, 5.0, kEps);
  EXPECT_NEAR(p12.theta, 3.0 * M_PI / 4.0, kEps);

  const sloc::Pose2 p3(-0.75, 1.25, -0.6);
  const sloc::Pose2 lhs = (p1 * p2) * p3;
  const sloc::Pose2 rhs = p1 * (p2 * p3);
  EXPECT_NEAR(lhs.x, rhs.x, kEps);
  EXPECT_NEAR(lhs.y, rhs.y, kEps);
  EXPECT_NEAR(lhs.theta, rhs.theta, kEps);
}

TEST(Se2Test, InverseCancelsPose)
{
  const sloc::Pose2 p(2.4, -1.7, 1.1);
  const sloc::Pose2 p_inv = p.inverse();

  const sloc::Pose2 right = p * p_inv;
  EXPECT_NEAR(right.x, 0.0, kEps);
  EXPECT_NEAR(right.y, 0.0, kEps);
  EXPECT_NEAR(right.theta, 0.0, kEps);

  const sloc::Pose2 left = p_inv * p;
  EXPECT_NEAR(left.x, 0.0, kEps);
  EXPECT_NEAR(left.y, 0.0, kEps);
  EXPECT_NEAR(left.theta, 0.0, kEps);
}

TEST(Se2Test, TransformScan)
{
  const std::vector<sloc::Vec2> empty_pts;
  const auto empty_out = sloc::transform_scan(
    sloc::Pose2(1.0, 2.0, 0.5), empty_pts.begin(), empty_pts.end());
  EXPECT_TRUE(empty_out.empty());

  const std::vector<sloc::Vec2> pts = {
    sloc::Vec2(1.0, 0.0),
    sloc::Vec2(0.0, 2.0),
    sloc::Vec2(-1.0, -1.0),
  };
  const sloc::Pose2 pose(10.0, -5.0, M_PI / 2.0);
  const auto world = sloc::transform_scan(pose, pts.begin(), pts.end());

  ASSERT_EQ(world.size(), pts.size());
  EXPECT_NEAR(world[0].x(), 10.0, kEps);
  EXPECT_NEAR(world[0].y(), -4.0, kEps);
  EXPECT_NEAR(world[1].x(), 8.0, kEps);
  EXPECT_NEAR(world[1].y(), -5.0, kEps);
  EXPECT_NEAR(world[2].x(), 11.0, kEps);
  EXPECT_NEAR(world[2].y(), -6.0, kEps);
}

}  // namespace
