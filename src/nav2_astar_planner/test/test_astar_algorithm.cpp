// Copyright (c) 2026
// A* 算法单元测试 —— 不依赖 ROS，只测纯算法层
//
// 运行：
//   colcon build --packages-select nav2_astar_planner
//   colcon test --packages-select nav2_astar_planner
//   colcon test-result --verbose

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "nav2_astar_planner/astar_algorithm.hpp"

namespace
{

using nav2_astar_planner::AStarAlgorithm;

std::vector<uint8_t> makeMap(unsigned int width, unsigned int height, uint8_t fill = 0)
{
  return std::vector<uint8_t>(static_cast<size_t>(width) * height, fill);
}

void setCell(
  std::vector<uint8_t> & map, unsigned int width,
  unsigned int x, unsigned int y, uint8_t value)
{
  map[static_cast<size_t>(y) * width + x] = value;
}

AStarAlgorithm makeAStar(
  const std::vector<uint8_t> & map, unsigned int width, unsigned int height,
  bool allow_diagonal = false, bool allow_unknown = true)
{
  AStarAlgorithm astar;
  astar.setCostmap(map, width, height);
  astar.setTraversabilityChecker(
    [allow_unknown](unsigned int, uint8_t cost) {
      if (cost == 255) {
        return allow_unknown;
      }
      return cost < 254;
    });
  astar.setOptionOverrides(allow_diagonal, allow_unknown, 0.0, 1.0, 1000000);
  return astar;
}

}  // namespace

TEST(AStarAlgorithm, FindsPathOnEmptyMap)
{
  const unsigned int W = 10, H = 10;
  auto map = makeMap(W, H);
  auto astar = makeAStar(map, W, H);

  const auto result = astar.plan({1, 1}, {8, 8});

  ASSERT_EQ(result, AStarAlgorithm::Result::SUCCESS);
  ASSERT_FALSE(astar.getPath().empty());
  EXPECT_EQ(astar.getPath().front().x, 1);
  EXPECT_EQ(astar.getPath().front().y, 1);
  EXPECT_EQ(astar.getPath().back().x, 8);
  EXPECT_EQ(astar.getPath().back().y, 8);
  EXPECT_DOUBLE_EQ(astar.getStats().path_cost, 14.0);  // 7 + 7 步
}

TEST(AStarAlgorithm, ReturnsNoPathWhenWallBlocks)
{
  const unsigned int W = 5, H = 5;
  auto map = makeMap(W, H);
  for (unsigned int y = 0; y < H; ++y) {
    setCell(map, W, 2, y, 254);  // 一整列墙
  }
  auto astar = makeAStar(map, W, H);

  EXPECT_EQ(astar.plan({1, 1}, {3, 3}), AStarAlgorithm::Result::NO_PATH);
}

TEST(AStarAlgorithm, AllowsStartOnInscribedObstacle)
{
  const unsigned int W = 5, H = 5;
  auto map = makeMap(W, H);
  setCell(map, W, 1, 1, 253);  // 起点是内切膨胀
  auto astar = makeAStar(map, W, H);

  EXPECT_EQ(astar.plan({1, 1}, {3, 3}), AStarAlgorithm::Result::SUCCESS);
}

TEST(AStarAlgorithm, RejectsGoalOnLethalObstacle)
{
  const unsigned int W = 5, H = 5;
  auto map = makeMap(W, H);
  setCell(map, W, 3, 3, 254);
  auto astar = makeAStar(map, W, H);

  EXPECT_EQ(astar.plan({1, 1}, {3, 3}), AStarAlgorithm::Result::GOAL_INVALID);
}

TEST(AStarAlgorithm, HandlesStartEqualsGoal)
{
  const unsigned int W = 5, H = 5;
  auto map = makeMap(W, H);
  auto astar = makeAStar(map, W, H);

  const auto result = astar.plan({1, 1}, {1, 1});

  ASSERT_EQ(result, AStarAlgorithm::Result::SUCCESS);
  ASSERT_EQ(astar.getPath().size(), 1u);
  EXPECT_EQ(astar.getPath().front().x, 1);
  EXPECT_EQ(astar.getPath().front().y, 1);
}

TEST(AStarAlgorithm, RespectsAllowUnknownFalse)
{
  const unsigned int W = 5, H = 5;
  auto map = makeMap(W, H, 255);  // 整张图未知
  setCell(map, W, 1, 1, 0);       // 起点
  setCell(map, W, 3, 3, 0);       // 终点
  auto astar = makeAStar(map, W, H, false, false);

  EXPECT_EQ(astar.plan({1, 1}, {3, 3}), AStarAlgorithm::Result::NO_PATH);
}

TEST(AStarAlgorithm, AllowsUnknownWhenConfigured)
{
  const unsigned int W = 5, H = 5;
  auto map = makeMap(W, H, 255);
  setCell(map, W, 1, 1, 0);
  setCell(map, W, 3, 3, 0);
  auto astar = makeAStar(map, W, H, false, true);

  EXPECT_EQ(astar.plan({1, 1}, {3, 3}), AStarAlgorithm::Result::SUCCESS);
}

TEST(AStarAlgorithm, DiagonalDoesNotCutCorners)
{
  // 3x3 地图，起点被墙围住，唯一“出口”是对角 (2,2)：
  //   ###
  //   #S#
  //   ##G
  // 如果不做切角检查，会从 (1,1) 斜穿到 (2,2)；
  // 正确实现应该返回 NO_PATH。
  const unsigned int W = 3, H = 3;
  auto map = makeMap(W, H, 254);
  setCell(map, W, 1, 1, 0);  // S
  setCell(map, W, 2, 2, 0);  // G
  auto astar = makeAStar(map, W, H, true, true);

  EXPECT_EQ(astar.plan({1, 1}, {2, 2}), AStarAlgorithm::Result::NO_PATH);
}

TEST(AStarAlgorithm, CancelsWhenRequested)
{
  const unsigned int W = 10, H = 10;
  auto map = makeMap(W, H);
  auto astar = makeAStar(map, W, H);

  const auto result = astar.plan({1, 1}, {8, 8}, []() {return true;});

  EXPECT_EQ(result, AStarAlgorithm::Result::CANCELLED);
}

TEST(AStarAlgorithm, StopsAtMaxExpansions)
{
  const unsigned int W = 20, H = 20;
  auto map = makeMap(W, H);

  AStarAlgorithm astar;
  astar.setCostmap(map, W, H);
  astar.setOptionOverrides(false, true, 0.0, 1.0, 1);  // 只允许扩展 1 次

  EXPECT_EQ(astar.plan({1, 1}, {18, 18}), AStarAlgorithm::Result::NO_PATH);
}
