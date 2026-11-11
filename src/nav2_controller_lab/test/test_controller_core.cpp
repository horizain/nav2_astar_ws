// Copyright (c) 2026
// 纯算法层单元测试：不依赖 ROS
//
// 运行：
//   colcon test --packages-select nav2_controller_lab
//   colcon test-result --verbose

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "nav2_controller_lab/controller_types.hpp"
#include "nav2_controller_lab/dwb_core.hpp"
#include "nav2_controller_lab/mpc_core.hpp"
#include "nav2_controller_lab/mppi_core.hpp"
#include "nav2_controller_lab/path_utils.hpp"
#include "nav2_controller_lab/pid_core.hpp"
#include "nav2_controller_lab/pure_pursuit_core.hpp"
#include "nav2_controller_lab/stanley_core.hpp"
#include "nav2_controller_lab/trajectory_utils.hpp"

namespace
{

using namespace nav2_controller_lab;  // NOLINT

std::vector<Pose2D> makeStraightPath(double length, double step = 0.1, double y = 0.0)
{
  std::vector<Pose2D> path;
  for (double x = 0.0; x <= length + 1e-9; x += step) {
    path.push_back(Pose2D{x, y, 0.0});
  }
  recomputePathYaw(path);
  return path;
}

}  // namespace

TEST(ControllerTypes, NormalizeAngle)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-9);
  EXPECT_NEAR(normalizeAngle(M_PI + 0.1), -M_PI + 0.1, 1e-6);
  EXPECT_NEAR(normalizeAngle(-M_PI - 0.1), M_PI - 0.1, 1e-6);
}

TEST(ControllerTypes, ClampControl)
{
  const Control out = clampControl(Control{10.0, -10.0}, 0.5, 1.9);
  EXPECT_DOUBLE_EQ(out.linear, 0.5);
  EXPECT_DOUBLE_EQ(out.angular, -1.9);
}

TEST(PathUtils, ProjectsOntoPath)
{
  const auto path = makeStraightPath(10.0, 0.5);
  const auto projection = projectOntoPath(Pose2D{5.0, 1.0, 0.0}, path);

  EXPECT_NEAR(projection.point.x, 5.0, 1e-9);
  EXPECT_NEAR(projection.point.y, 0.0, 1e-9);
  EXPECT_NEAR(projection.distance, 1.0, 1e-9);
  EXPECT_NEAR(projection.signed_lateral, 1.0, 1e-9);  // 路径左侧为正
  EXPECT_NEAR(projection.path_yaw, 0.0, 1e-9);
}

TEST(PathUtils, FindsLookaheadPoint)
{
  const auto path = makeStraightPath(10.0, 1.0);
  Pose2D target;
  ASSERT_TRUE(findLookaheadPoint(Pose2D{0.0, 0.0, 0.0}, path, 2.5, target));

  EXPECT_NEAR(target.x, 2.5, 1e-9);
  EXPECT_NEAR(target.y, 0.0, 1e-9);
}

TEST(PathUtils, LookaheadFallsBackToLastPoint)
{
  const auto path = makeStraightPath(2.0, 1.0);
  Pose2D target;
  ASSERT_TRUE(findLookaheadPoint(Pose2D{0.0, 0.0, 0.0}, path, 10.0, target));

  EXPECT_NEAR(target.x, 2.0, 1e-9);
  EXPECT_NEAR(target.y, 0.0, 1e-9);
}

TEST(PathUtils, RemainingDistance)
{
  const auto path = makeStraightPath(10.0, 0.5);
  EXPECT_NEAR(remainingDistance(Pose2D{4.0, 0.0, 0.0}, path), 6.0, 1e-9);
}

TEST(TrajectoryUtils, RolloutConstantControl)
{
  const auto trajectory = rolloutConstantControl(
    Pose2D{0.0, 0.0, 0.0}, 1.0, 0.0, 10, 0.1);

  ASSERT_EQ(trajectory.size(), 11u);
  EXPECT_NEAR(trajectory.back().x, 1.0, 1e-9);
  EXPECT_NEAR(trajectory.back().y, 0.0, 1e-9);
}

TEST(PidCore, StraightPathHasNoAngularCommand)
{
  PidCore core;
  const auto path = makeStraightPath(10.0);
  const Control command = core.compute(Pose2D{0.0, 0.0, 0.0}, path, 0.0, 0.1);

  EXPECT_GT(command.linear, 0.0);
  EXPECT_NEAR(command.angular, 0.0, 1e-9);
}

TEST(PidCore, OffsetProducesCorrectTurnDirection)
{
  PidCore core;
  const auto path = makeStraightPath(10.0);
  // 机器人在路径左侧，应该向右修正 -> 负角速度
  const Control command = core.compute(Pose2D{0.0, 1.0, 0.0}, path, 0.0, 0.1);

  EXPECT_LT(command.angular, 0.0);
}

TEST(PidCore, StopsAtGoal)
{
  PidCore core;
  const auto path = makeStraightPath(10.0);
  const Control command = core.compute(Pose2D{10.0, 0.0, 0.0}, path, 0.0, 0.1);

  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(PurePursuitCore, StraightPathHasNoAngularCommand)
{
  PurePursuitCore core;
  const auto path = makeStraightPath(10.0);
  const Control command = core.compute(Pose2D{0.0, 0.0, 0.0}, path, 0.0);

  EXPECT_GT(command.linear, 0.0);
  EXPECT_NEAR(command.angular, 0.0, 1e-9);
}

TEST(PurePursuitCore, OffsetProducesCorrectTurnDirection)
{
  PurePursuitCore core;
  const auto path = makeStraightPath(10.0);
  // 机器人在路径左侧，前视点在右前方 -> 负角速度
  const Control command = core.compute(Pose2D{0.0, 1.0, 0.0}, path, 0.0);

  EXPECT_LT(command.angular, 0.0);
}

TEST(PurePursuitCore, StopsAtGoal)
{
  PurePursuitCore core;
  const auto path = makeStraightPath(10.0);
  const Control command = core.compute(Pose2D{10.0, 0.0, 0.0}, path, 0.0);

  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(StanleyCore, StraightPathHasNoAngularCommand)
{
  StanleyCore core;
  const auto path = makeStraightPath(10.0);
  const Control command = core.compute(Pose2D{0.0, 0.0, 0.0}, path, 0.0);

  EXPECT_GT(command.linear, 0.0);
  EXPECT_NEAR(command.angular, 0.0, 1e-9);
}

TEST(StanleyCore, OffsetProducesCorrectTurnDirection)
{
  StanleyCore core;
  const auto path = makeStraightPath(10.0);
  // 机器人在路径左侧 -> 应该向右修正 -> 负角速度
  const Control command = core.compute(Pose2D{0.0, 1.0, 0.0}, path, 0.0);

  EXPECT_LT(command.angular, 0.0);
}

TEST(StanleyCore, StopsAtGoal)
{
  StanleyCore core;
  const auto path = makeStraightPath(10.0);
  const Control command = core.compute(Pose2D{10.0, 0.0, 0.0}, path, 0.0);

  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(DwbCore, ChoosesForwardOnClearStraightPath)
{
  DwbParams params;
  params.horizon = 1.0;
  params.dt = 0.1;
  DwbCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto no_obstacle = [](double, double) -> uint8_t {return 0;};

  const auto result = core.compute(Pose2D{0.0, 0.0, 0.0}, path, no_obstacle, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.linear, 0.0);
  EXPECT_LT(std::abs(result.control.angular), 0.5);
}

TEST(DwbCore, RejectsAllCollidingTrajectories)
{
  DwbParams params;
  params.horizon = 1.0;
  params.dt = 0.1;
  DwbCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto always_obstacle = [](double, double) -> uint8_t {return 254;};

  const auto result = core.compute(
    Pose2D{0.0, 0.0, 0.0}, path, always_obstacle, 0.0);

  EXPECT_FALSE(result.valid);
}

TEST(MppiCore, ChoosesForwardOnClearStraightPath)
{
  MppiParams params;
  params.time_steps = 10;
  params.batch_size = 40;
  params.dt = 0.1;
  MppiCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto no_obstacle = [](double, double) -> uint8_t {return 0;};

  const auto result = core.compute(Pose2D{0.0, 0.0, 0.0}, path, no_obstacle, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.linear, 0.0);
  EXPECT_LE(std::abs(result.control.angular), params.max_angular_velocity);
}

TEST(MppiCore, RejectsAllCollidingTrajectories)
{
  MppiParams params;
  params.time_steps = 10;
  params.batch_size = 20;
  params.dt = 0.1;
  MppiCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto always_obstacle = [](double, double) -> uint8_t {return 254;};

  const auto result = core.compute(
    Pose2D{0.0, 0.0, 0.0}, path, always_obstacle, 0.0);

  EXPECT_FALSE(result.valid);
}

TEST(MpcCore, ChoosesForwardOnClearStraightPath)
{
  MpcParams params;
  params.horizon_steps = 8;
  params.iterations = 6;
  params.dt = 0.1;
  MpcCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto no_obstacle = [](double, double) -> uint8_t {return 0;};

  const auto result = core.compute(Pose2D{0.0, 0.0, 0.0}, path, no_obstacle, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.linear, 0.0);
  EXPECT_LT(std::abs(result.control.angular), 0.5);
}

TEST(MpcCore, OffsetProducesCorrectTurnDirection)
{
  MpcParams params;
  params.horizon_steps = 8;
  params.iterations = 6;
  params.dt = 0.1;
  MpcCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto no_obstacle = [](double, double) -> uint8_t {return 0;};

  // 机器人在路径左侧 -> 应该向右修正 -> 负角速度
  const auto result = core.compute(Pose2D{0.0, 1.0, 0.0}, path, no_obstacle, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.control.angular, 0.0);
}

TEST(MpcCore, StopsAtGoal)
{
  MpcParams params;
  MpcCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto no_obstacle = [](double, double) -> uint8_t {return 0;};

  const auto result = core.compute(Pose2D{10.0, 0.0, 0.0}, path, no_obstacle, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.control.linear, 0.0);
  EXPECT_DOUBLE_EQ(result.control.angular, 0.0);
}

TEST(MpcCore, RejectsAllCollidingTrajectories)
{
  MpcParams params;
  params.horizon_steps = 8;
  params.iterations = 4;
  MpcCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto always_obstacle = [](double, double) -> uint8_t {return 254;};

  const auto result = core.compute(
    Pose2D{0.0, 0.0, 0.0}, path, always_obstacle, 0.0);

  EXPECT_FALSE(result.valid);
}
