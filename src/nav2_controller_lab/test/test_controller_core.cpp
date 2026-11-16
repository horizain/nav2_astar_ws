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
#include "nav2_controller_lab/cbf_core.hpp"
#include "nav2_controller_lab/dwb_core.hpp"
#include "nav2_controller_lab/mpc_core.hpp"
#include "nav2_controller_lab/mppi_core.hpp"
#include "nav2_controller_lab/path_utils.hpp"
#include "nav2_controller_lab/pid_core.hpp"
#include "nav2_controller_lab/pure_pursuit_core.hpp"
#include "nav2_controller_lab/stanley_core.hpp"
#include "nav2_controller_lab/trajectory_cost.hpp"
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

// ---- rotateInPlace：终点旋转（2026-11 端到端实验新增）----
// 教训 #2：不仅要测"有没有输出"，还要专门测"转哪边"。

TEST(PathUtils, RotateInPlaceZeroCommandWhenAligned)
{
  const Control command = rotateInPlace(Pose2D{1.0, 2.0, 0.5}, 0.5, 1.5);
  EXPECT_NEAR(command.linear, 0.0, 1e-9);
  EXPECT_NEAR(command.angular, 0.0, 1e-9);
}

TEST(PathUtils, RotateInPlaceSmallErrorProportional)
{
  // 误差 0.1 rad → w = 2.0 * 0.1 = 0.2，未触发限幅
  const Control command = rotateInPlace(Pose2D{0.0, 0.0, 0.0}, 0.1, 1.5);
  EXPECT_NEAR(command.linear, 0.0, 1e-9);
  EXPECT_NEAR(command.angular, 0.2, 1e-9);
}

TEST(PathUtils, RotateInPlaceClampsLargeError)
{
  // 误差 +90° → 未限幅 w = π ≈ 3.14，被 clamp 到 +1.5
  const Control command = rotateInPlace(Pose2D{0.0, 0.0, 0.0}, M_PI / 2.0, 1.5);
  EXPECT_NEAR(command.linear, 0.0, 1e-9);
  EXPECT_NEAR(command.angular, 1.5, 1e-9);
}

TEST(PathUtils, RotateInPlaceTurnsShortestDirection)
{
  // 机器人在 +170°，目标 -170°：直接差 -340°，最短方向是 +20°（逆时针）。
  // 验证 normalizeAngle 把误差卷绕到 [-π, π]，机器人不会傻转 340°。
  const Control command =
    rotateInPlace(Pose2D{0.0, 0.0, 170.0 * M_PI / 180.0}, -170.0 * M_PI / 180.0, 1.5);
  const double expected = 2.0 * (20.0 * M_PI / 180.0);
  EXPECT_NEAR(command.angular, expected, 1e-9);
}

TEST(PathUtils, RotateInPlaceNegativeErrorTurnsClockwise)
{
  // 目标在顺时针方向（负误差）→ w 必须为负（ROS 坐标系 z 轴朝上）
  const Control command = rotateInPlace(Pose2D{0.0, 0.0, 0.0}, -0.1, 1.5);
  EXPECT_NEAR(command.angular, -0.2, 1e-9);
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

// ---- 评分权重重构（2026-11 解冻实验，详见 docs/16）----
// 旧权重（obstacle=100/goal=2/threshold=254）在膨胀场里会冻结：
// 前进轨迹的 obstacle 均值代价压倒 goal 收益，v=0 原地不动总分最小。
// 以下测试锁定新语义：碰撞二值化 + 前进收益主导 + obstacle 只是平手裁决。

TEST(DwbCore, ProgressesThroughModerateInflation)
{
  // 解冻回归测试：中等代价（raw=100）的膨胀场里也必须继续前进。
  // 权重对齐 yaml DWB 块（即官方 DWB 比例在本评分器单位下的映射）。
  DwbParams params;
  params.horizon = 1.5;
  params.dt = 0.1;
  params.cost.path_weight = 8.0;
  params.cost.goal_weight = 24.0;
  params.cost.obstacle_weight = 2.0;
  params.cost.collision_threshold = 253;
  DwbCore core(params);

  const auto path = makeStraightPath(10.0);
  // 膨胀场：机器人脚下代价 0，前方一律 raw=100
  const auto inflation_field = [](double x, double) -> uint8_t {
    return x <= 0.05 ? static_cast<uint8_t>(0) : static_cast<uint8_t>(100);
  };

  const auto result = core.compute(Pose2D{0.0, 0.0, 0.0}, path, inflation_field, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.linear, 0.1);  // 选择前进，而不是原地不动
}

TEST(DwbCore, SlowsBeforeInscribedZoneInsteadOfSmashing)
{
  // 官方 isValidCost 语义：INSCRIBED(253) 也是必碰撞。
  // 前方 0.3m 外整条走廊都是 253 → 高速轨迹出界即整条否决，
  // 采样器只能选出低速/不动的轨迹——减速，而不是硬闯。
  DwbParams params;
  params.horizon = 1.5;
  params.dt = 0.1;
  params.cost.collision_threshold = 253;
  DwbCore core(params);
  const auto path = makeStraightPath(10.0);
  const auto inscribed_wall = [](double x, double) -> uint8_t {
    return x > 0.3 ? static_cast<uint8_t>(253) : static_cast<uint8_t>(0);
  };

  const auto result = core.compute(Pose2D{0.0, 0.0, 0.0}, path, inscribed_wall, 0.0);

  // 要么整批否决（v=0 兜底），要么只敢以低速蹭过去（v·horizon < 0.3m），
  // 绝不允许选出会撞进 253 区的高速轨迹。
  if (result.valid) {
    EXPECT_LT(result.control.linear, 0.2);
  } else {
    EXPECT_DOUBLE_EQ(result.control.linear, 0.0);
  }
}

TEST(TrajectoryCost, InscribedIsCollision)
{
  // 官方 isValidCost：253 与 254 同罪。阈值 253 时撞进内切区 = 一票否决。
  TrajectoryCostParams params;
  params.collision_threshold = 253;
  const auto path = makeStraightPath(10.0);
  const auto inscribed_band = [](double x, double) -> uint8_t {
    return (x > 0.4 && x < 0.6) ? static_cast<uint8_t>(253) : static_cast<uint8_t>(0);
  };
  const auto trajectory =
    rolloutConstantControl(Pose2D{0.0, 0.0, 0.0}, 0.5, 0.0, 10, 0.1);

  const double score = evaluateTrajectory(
    trajectory, path, inscribed_band, params, 0.5, 0.0);

  EXPECT_DOUBLE_EQ(score, params.collision_cost);
}

TEST(TrajectoryCost, ModerateCostIsNotCollision)
{
  // raw=100 的膨胀带不触发否决，只是普通代价项——这正是"穿过膨胀场"
  // 与"撞上障碍"的本质区别，也是解冻的关键语义。
  TrajectoryCostParams params;
  params.collision_threshold = 253;
  const auto path = makeStraightPath(10.0);
  const auto moderate_band = [](double x, double) -> uint8_t {
    return (x > 0.4 && x < 0.6) ? static_cast<uint8_t>(100) : static_cast<uint8_t>(0);
  };
  const auto trajectory =
    rolloutConstantControl(Pose2D{0.0, 0.0, 0.0}, 0.5, 0.0, 10, 0.1);

  const double score = evaluateTrajectory(
    trajectory, path, moderate_band, params, 0.5, 0.0);

  EXPECT_LT(score, params.collision_cost);
  EXPECT_GT(score, 0.0);
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

TEST(MpcCore, ExposesSelectedModelPredictionAndClearsInvalidHistory)
{
  MpcCore core;
  const Pose2D robot{0, 0, 0};
  const auto result = core.compute(robot, makeStraightPath(3.0),
    [](double, double) -> uint8_t {return 0;}, 0);
  ASSERT_TRUE(result.valid);
  const auto & predicted = core.predictedTrajectory();
  ASSERT_EQ(predicted.size(), static_cast<size_t>(core.params().horizon_steps + 1));
  EXPECT_DOUBLE_EQ(predicted.front().x, 0);
  EXPECT_NEAR(predicted[1].x, result.control.linear * core.params().dt, 1e-12);
  EXPECT_NEAR(predicted[1].yaw, result.control.angular * core.params().dt, 1e-12);
  EXPECT_FALSE(core.compute(robot, makeStraightPath(3.0),
    [](double, double) -> uint8_t {return 254;}, 0).valid);
  EXPECT_TRUE(core.predictedTrajectory().empty());
}

TEST(MpcCore, AlignsBeforeFollowingPathWhenFacingBackwards)
{
  MpcCore core;
  auto path = makeStraightPath(3.0);
  Pose2D pose{0.0, 0.0, 3.141592653589793};
  Control velocity{};
  const auto clear = [](double, double) -> uint8_t {return 0;};
  for (int i = 0; i < 400 && distance(pose, path.back()) > 0.3; ++i) {
    auto result = core.compute(pose, path, clear, velocity.linear, 0.05, velocity.angular);
    ASSERT_TRUE(result.valid);
    EXPECT_LE(std::abs(result.control.linear - velocity.linear), 0.050001);
    EXPECT_LE(std::abs(result.control.angular - velocity.angular), 0.160001);
    velocity = result.control;
    pose = rolloutConstantControl(pose, velocity.linear, velocity.angular, 1, 0.05).back();
  }
  EXPECT_LT(distance(pose, path.back()), 0.3);
}

TEST(MpcCore, RejectsInscribedObstacleRegion)
{
  MpcCore core;
  EXPECT_FALSE(core.compute({}, makeStraightPath(3.0),
    [](double, double) -> uint8_t {return 253;}, 0.0).valid);
}

TEST(MpcCore, RecoversAfterBlockedMapBecomesClear)
{
  MpcCore core;
  auto path = makeStraightPath(3.0);
  EXPECT_FALSE(core.compute({}, path, [](double, double) -> uint8_t {return 254;}, 0).valid);
  auto result = core.compute({}, path, [](double, double) -> uint8_t {return 0;}, 0);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.linear, 0.0);
}

TEST(MpcCore, ClosedLoopCornerWithQuantizedInflationAndAccelerationLimits)
{
  MpcParams params;
  params.cost.obstacle_weight = 2.0;
  params.goal_tolerance = 0.2;
  MpcCore core(params);
  auto path = makeStraightPath(1.5);
  for (double y = 0.1; y <= 1.5; y += 0.1) {path.push_back({1.5, y, 1.57079632679});}
  const auto cost = [](double x, double y) -> uint8_t {
      // Quantized inflation beside the route, with a lethal wall further away.
      if (y < -0.4 || x > 2.0) {return 254;}
      return y < -0.15 ? 180 : 0;
    };
  Pose2D pose{};
  Control velocity{};
  int moving = 0;
  for (int i = 0; i < 500 && distance(pose, path.back()) > 0.22; ++i) {
    const auto result = core.compute(pose, path, cost, velocity.linear, 0.05, velocity.angular);
    ASSERT_TRUE(result.valid) << "cycle " << i;
    EXPECT_LE(std::abs(result.control.linear - velocity.linear), 0.050001);
    EXPECT_LE(std::abs(result.control.angular - velocity.angular), 0.160001);
    if (result.control.linear > 0.02) {++moving;}
    velocity = result.control;
    pose = rolloutConstantControl(pose, velocity.linear, velocity.angular, 1, 0.05).back();
    ASSERT_LT(cost(pose.x, pose.y), 254);
  }
  EXPECT_LT(distance(pose, path.back()), 0.25);
  EXPECT_GT(moving, 40);
}

TEST(MpcCore, EscapesCollidingStraightSeedByTurning)
{
  MpcParams params;
  params.cost.obstacle_weight = 2.0;
  MpcCore core(params);
  auto path = makeStraightPath(2.0);
  auto cost = [](double x, double y) -> uint8_t {
      return x > 0.20 && std::abs(y) < 0.12 ? 254 : 0;
    };
  auto result = core.compute({}, path, cost, 0.0);
  ASSERT_TRUE(result.valid);
  auto prediction = rolloutConstantControl({}, result.control.linear, result.control.angular, 1, 0.05);
  EXPECT_LT(cost(prediction.back().x, prediction.back().y), 254);
}

// ---- CBF 安全过滤器（2026-11，详见 docs/17）----
// 每个用例都能用纸笔验证：约束 v·cos(ψ) ≤ α·h，h = range - d_safe

TEST(CbfFilter, PassthroughWhenClear)
{
  // 障碍在影响范围外 → 原样透传，过滤器"隐形"
  CbfParams params;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{0.0, 3.0}};
  const auto result = filter.filter(0.5, 0.3, scan);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.linear, 0.5);
  EXPECT_DOUBLE_EQ(result.angular, 0.3);
}

TEST(CbfFilter, ClampsHeadOnApproach)
{
  // 手算：正前方 0.5m，d_safe=0.3，α=2 → h=0.2 → 上界 = 2×0.2/1 = 0.4
  // 标称 0.5 被裁剪到 0.4，且记录束缚点索引
  CbfParams params;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{0.0, 0.5}};
  const auto result = filter.filter(0.5, 0.0, scan);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.linear, 0.4, 1e-9);
  EXPECT_EQ(result.binding_index, 0);
}

TEST(CbfFilter, TangentialObstacleDoesNotConstrain)
{
  // 纯侧向点（ψ=90°）：一阶下前进会增大到它的距离，不构成约束
  // 即使已在余量内，向前的运动也是"离开"方向
  CbfParams params;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{M_PI / 2, 0.25}};
  const auto result = filter.filter(0.5, 0.0, scan);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.linear, 0.5);
}

TEST(CbfFilter, ForceReversalWhenInsideMargin)
{
  // 已侵入余量：正前方 0.25m < d_safe=0.3 → h=-0.05
  // 上界 = 2×(-0.05)/1 = -0.1 → 标称 +0.3 被强制为倒车 -0.1（内置恢复行为）
  CbfParams params;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{0.0, 0.25}};
  const auto result = filter.filter(0.3, 0.0, scan);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.linear, -0.1, 1e-9);
  EXPECT_LT(result.min_h, 0.0);
}

TEST(CbfFilter, InfeasibleWhenSqueezed)
{
  // 前后夹击：前方 0.25m 逼倒车（上界 -0.1），后方 0.25m 逼前进（下界 +0.1）
  // 可行域为空 → 最安全的裁决是停车
  CbfParams params;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{0.0, 0.25}, {M_PI, 0.25}};
  const auto result = filter.filter(0.0, 0.0, scan);
  EXPECT_TRUE(result.infeasible);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.linear, 0.0);
}

TEST(CbfFilter, RearObstacleLimitsReverseOnly)
{
  // 后方有近障碍、前方畅通：前进不受限，只有倒车被裁剪
  // 后点 ψ=π, range=0.4, h=0.1, cos=-1 → 下界 = 2×0.1/(-1) = -0.2
  CbfParams params;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{M_PI, 0.4}};
  const auto result = filter.filter(-0.5, 0.0, scan);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.linear, -0.2, 1e-9);
}

TEST(CbfFilter, LidarOffsetCompensationPreventsFalseBarrier)
{
  // 实车案例（docs/17 §6）：TB3 waffle 雷达在中心后方 6.4cm。
  // 不补偿：侧后方束 range=0.222 被当成中心距离 → h=-0.008 < 0 → 误拦合法运动。
  // 补偿后：该点在中心系为 (0.002, -0.212)，几乎纯切向（|cosθ|≈0.008）→ 跳过。
  CbfParams params;
  params.d_safe = 0.22;
  params.lidar_offset_x = -0.064;  // TB3 waffle 实测外参
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{286.7 * M_PI / 180.0, 0.222}};
  const auto result = filter.filter(0.5, 0.0, scan);
  EXPECT_FALSE(result.active);  // 补偿前这里会把 v 裁到 ~0.013，机器人假死
  EXPECT_DOUBLE_EQ(result.linear, 0.5);
}

TEST(CbfFilter, OffsetMakesFrontalMarginHonest)
{
  // 外参的方向性：雷达在中心后方 6.4cm → 前方束的中心距离 = range - 0.064
  // （雷达比中心更远离前方障碍？不——雷达在后方，前方障碍到雷达更近：
  //   中心距离 = range + ... 本例用准确几何：range=0.30, ψ=0 → 中心系点
  //   (0.30-0.064, 0) = (0.236, 0) → 中心距离 0.236 < d_safe=0.30 → 必须减速）
  CbfParams params;
  params.lidar_offset_x = -0.064;
  CbfFilter filter(params);
  const std::vector<std::pair<double, double>> scan = {{0.0, 0.30}};
  const auto result = filter.filter(0.5, 0.0, scan);
  EXPECT_TRUE(result.active);
  // h = 0.236 - 0.30 = -0.064 → 上界 = 2×(-0.064)/1 = -0.128
  EXPECT_NEAR(result.linear, -0.128, 1e-9);
}
