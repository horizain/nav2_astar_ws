#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "nav2_astar_planner/astar_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class PlannerGoalContract : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("use_sim_time", false)});
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap_->configure();
    ASSERT_EQ(costmap_->get_current_state().label(), "inactive");
    auto * map = costmap_->getCostmap();
    map->resizeMap(40, 40, 0.05, 0.0, 0.0);
    std::fill(map->getCharMap(), map->getCharMap() + 40 * 40, 0);
    parent_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_contract_parent");
    tf_ = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
    planner_.configure(parent_, "GridBased", tf_, costmap_);
  }

  void TearDown() override
  {
    planner_.cleanup();
    costmap_->cleanup();
    costmap_.reset();
    parent_.reset();
    tf_.reset();
  }

  geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "map";
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.z = std::sin(yaw / 2);
    result.pose.orientation.w = std::cos(yaw / 2);
    return result;
  }

  nav2_astar_planner::AStarPlanner planner_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

TEST_F(PlannerGoalContract, PreservesContinuousStartGoalAndRequestedHeading)
{
  const auto start = pose(0.011, 0.013);
  const auto goal = pose(0.731, 0.687, 0.7);
  const auto path = planner_.createPlan(start, goal, []() {return false;});
  ASSERT_GT(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y, start.pose.position.y);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.z, goal.pose.orientation.z);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.w, goal.pose.orientation.w);
}

TEST_F(PlannerGoalContract, SameCellStillTargetsRequestedContinuousPose)
{
  const auto start = pose(0.011, 0.013);
  const auto goal = pose(0.041, 0.038, -0.4);
  const auto path = planner_.createPlan(start, goal, []() {return false;});
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.z, goal.pose.orientation.z);
}

TEST_F(PlannerGoalContract, InscribedGoalRemainsRejected)
{
  costmap_->getCostmap()->setCost(10, 8, 253);
  EXPECT_THROW(planner_.createPlan(pose(0.1, 0.1), pose(0.53, 0.42),
    []() {return false;}), nav2_core::GoalOccupied);
}


TEST_F(PlannerGoalContract, FinalApproachUsesActualLastSegment)
{
  parent_->set_parameter(rclcpp::Parameter("GridBased.use_final_approach_orientation", true));
  planner_.cleanup();
  planner_.configure(parent_, "GridBased", tf_, costmap_);
  const auto goal = pose(0.731, 0.687, -1.2);
  const auto path = planner_.createPlan(pose(0.011, 0.013), goal, []() {return false;});
  ASSERT_GT(path.poses.size(), 1u);
  const auto & last = path.poses.back().pose;
  const auto & previous = path.poses[path.poses.size()-2].pose;
  const double expected = std::atan2(last.position.y-previous.position.y,
    last.position.x-previous.position.x);
  EXPECT_NEAR(last.orientation.z, std::sin(expected/2), 1e-12);
  EXPECT_NEAR(last.orientation.w, std::cos(expected/2), 1e-12);
  EXPECT_DOUBLE_EQ(last.position.x, goal.pose.position.x);
}
