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

#include "scan_localizer/odom_sync_buffer.hpp"

namespace
{

TEST(OdomSyncBufferTest, InterpolatesPoseAndWrapsAngleAcrossPi)
{
  sloc::OdomSyncBuffer buf(10);
  buf.push(1.0, sloc::Pose2(0.0, 0.0, 3.0), 1.0, 0.0, 0.2);
  buf.push(2.0, sloc::Pose2(2.0, 1.0, -3.0), 1.0, 0.5, 0.2);

  sloc::Pose2 out{};
  bool stale = false;
  double age = 0.0;
  ASSERT_TRUE(buf.lookup(1.5, out, 1.0, stale, age));
  EXPECT_FALSE(stale);
  EXPECT_NEAR(out.x, 1.0, 1e-9);
  EXPECT_NEAR(out.y, 0.5, 1e-9);
  EXPECT_NEAR(std::abs(out.theta), sloc::kPi, 1e-6);
}

TEST(OdomSyncBufferTest, DetectsStaleAndResetsOnTimeRollback)
{
  sloc::OdomSyncBuffer buf(10);
  buf.push(10.0, sloc::Pose2(1.0, 2.0, 0.1));

  sloc::Pose2 out{};
  bool stale = false;
  double age = 0.0;
  ASSERT_TRUE(buf.lookup(12.5, out, 1.0, stale, age));
  EXPECT_TRUE(stale);
  EXPECT_NEAR(age, 2.5, 1e-9);

  // Time rollback clears old samples
  buf.push(5.0, sloc::Pose2(9.0, 8.0, 0.5));
  EXPECT_EQ(buf.size(), 1u);
  ASSERT_TRUE(buf.lookup(5.0, out, 1.0, stale, age));
  EXPECT_FALSE(stale);
  EXPECT_NEAR(out.x, 9.0, 1e-9);
}

TEST(OdomSyncBufferTest, ComputesDeltaAndExtrapolatesAsyncGlobalResult)
{
  sloc::OdomSyncBuffer buf(20);
  // Robot moves forward 0.30m along its heading (pi/2) during 0.3s global search
  buf.push(10.0, sloc::Pose2(1.0, 2.0, sloc::kPi / 2.0));
  buf.push(10.3, sloc::Pose2(1.0, 2.3, sloc::kPi / 2.0 + 0.1));

  sloc::Pose2 delta{};
  ASSERT_TRUE(buf.compute_delta(10.0, 10.3, delta));
  EXPECT_NEAR(delta.x, 0.30, 1e-6);
  EXPECT_NEAR(delta.y, 0.0, 1e-6);
  EXPECT_NEAR(delta.theta, 0.1, 1e-6);

  sloc::GlobalResult g;
  g.valid = true;
  g.pose = sloc::Pose2(5.0, 6.0, 0.0);
  g.alternates.emplace_back(10.0, 6.0, sloc::kPi);
  sloc::OdomSyncBuffer::extrapolate_global_result(g, delta);

  EXPECT_NEAR(g.pose.x, 5.30, 1e-6);
  EXPECT_NEAR(g.pose.y, 6.0, 1e-6);
  EXPECT_NEAR(g.pose.theta, 0.1, 1e-6);
  ASSERT_EQ(g.alternates.size(), 1u);
  EXPECT_NEAR(g.alternates[0].x, 9.70, 1e-6);
  EXPECT_NEAR(g.alternates[0].y, 6.0, 1e-6);
}

TEST(GlobalRetryGateTest, ThrottlesWhenStationaryAndUnblocksOnMotionOrCooldown)
{
  sloc::GlobalRetryGate gate;
  const sloc::Pose2 p0(0.0, 0.0, 0.0);

  // First submission allowed when idle, blocked when worker busy
  EXPECT_FALSE(gate.should_submit(1.0, true, p0, true, 1.5, 0.15));
  EXPECT_TRUE(gate.should_submit(1.0, true, p0, false, 1.5, 0.15));

  gate.mark_submitted(1.0, true, p0);

  // Stationary within cooldown -> throttled
  EXPECT_FALSE(gate.should_submit(1.5, true, sloc::Pose2(0.02, 0.0, 0.0), false, 1.5, 0.15));

  // Moved >= min_move_m -> unblocked immediately
  EXPECT_TRUE(gate.should_submit(1.5, true, sloc::Pose2(0.20, 0.0, 0.0), false, 1.5, 0.15));

  // Stationary after cooldown elapsed -> unblocked
  EXPECT_TRUE(gate.should_submit(2.6, true, p0, false, 1.5, 0.15));
}

}  // namespace
