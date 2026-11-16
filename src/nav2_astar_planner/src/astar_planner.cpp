// Copyright (c) 2026
// A* 全局路径规划器 —— Nav2 插件封装层实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_astar_planner/astar_planner.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Failed to lock parent node in AStarPlanner::configure");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_->getGlobalFrameID();

  astar_ = std::make_unique<AStarAlgorithm>();

  // -------- 参数声明 --------
  // 注意：参数名必须带上插件名作为前缀（如 GridBased.allow_diagonal），
  // 这是 nav2 的多插件参数规范。
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + ".cost_weight", cost_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_expansions", rclcpp::ParameterValue(1000000));
  node->get_parameter(name_ + ".max_expansions", max_expansions_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);

  RCLCPP_INFO(
    logger_,
    "AStarPlanner(%s) 已配置: allow_diagonal=%d, allow_unknown=%d, "
    "cost_weight=%.2f, heuristic_weight=%.2f, interpolation_resolution=%.3f",
    name_.c_str(), static_cast<int>(allow_diagonal_), static_cast<int>(allow_unknown_),
    cost_weight_, heuristic_weight_, interpolation_resolution_);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "AStarPlanner(%s) 清理资源", name_.c_str());
  astar_.reset();
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "AStarPlanner(%s) 已激活", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "AStarPlanner(%s) 已去激活", name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;

  if (!astar_) {
    throw nav2_core::PlannerException("AStarPlanner 尚未 configure");
  }

  // 拿最新的代价地图
  costmap_ = costmap_ros_->getCostmap();
  if (!costmap_) {
    throw nav2_core::PlannerException("无法获取代价地图");
  }

  // 代价地图可能被其他线程更新，需要加锁
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // -------- 世界坐标 -> 格子坐标 --------
  unsigned int mx_start, my_start, mx_goal, my_goal;

  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    throw nav2_core::StartOutsideMapBounds(
            "起点 (" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") 超出地图范围");
  }

  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    throw nav2_core::GoalOutsideMapBounds(
            "终点 (" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") 超出地图范围");
  }

  if (costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE ||
    costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    throw nav2_core::GoalOccupied(
            "终点 (" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") 位于致命障碍上");
  }

  // -------- 起终点同一格的特殊情况 --------
  if (mx_start == mx_goal && my_start == my_goal) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose = goal.pose;
    if (use_final_approach_orientation_) {
      const double dx = goal.pose.position.x - start.pose.position.x;
      const double dy = goal.pose.position.y - start.pose.position.y;
      if (std::hypot(dx, dy) > 1e-9) {
        const double theta = std::atan2(dy, dx);
        pose.pose.orientation.x = pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(theta / 2.0);
        pose.pose.orientation.w = std::cos(theta / 2.0);
      } else {
        pose.pose.orientation = start.pose.orientation;
      }
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  // -------- 把代价地图交给 A* --------
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned char * char_map = costmap_->getCharMap();

  std::vector<uint8_t> costs(char_map, char_map + static_cast<size_t>(size_x) * size_y);
  astar_->setCostmap(costs, size_x, size_y);

  // 配置可通行判定
  //
  // 重要：nav2 的代价语义
  //   0        FREE_SPACE               空闲
  //   1-252    膨胀代价                越大越靠近障碍
  //   253      INSCRIBED_INFLATED_OBSTACLE 内切膨胀（机器人中心不能进）
  //   254      LETHAL_OBSTACLE         真实障碍
  //   255      NO_INFORMATION          未知区域
  //
  // 253 is inside the robot footprint exclusion region, not a traversable high cost.
  const bool allow_unknown = allow_unknown_;
  astar_->setTraversabilityChecker(
    [allow_unknown](unsigned int, uint8_t cost) -> bool {
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown;
      }
      return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    });

  // -------- 执行 A* --------
  GridCoord start_grid{static_cast<int>(mx_start), static_cast<int>(my_start)};
  GridCoord goal_grid{static_cast<int>(mx_goal), static_cast<int>(my_goal)};

  // 设置算法选项（通过重新设置 costmap 后的内部配置生效）
  // 注意：这里直接用参数在算法内部生效的路径是 setOptions，
  // 为保持接口简洁，我们在每次规划前重建选项。
  astar_->setOptionOverrides(
    allow_diagonal_, allow_unknown_, cost_weight_, heuristic_weight_, max_expansions_);

  RCLCPP_DEBUG(
    logger_,
    "A* 规划请求: 地图=%ux%u 起点=(%d,%d) 终点=(%d,%d)",
    size_x, size_y, start_grid.x, start_grid.y, goal_grid.x, goal_grid.y);

  const auto result = astar_->plan(start_grid, goal_grid, cancel_checker);

  switch (result) {
    case AStarAlgorithm::Result::SUCCESS:
      break;
    case AStarAlgorithm::Result::CANCELLED:
      throw nav2_core::PlannerCancelled("规划被取消");
    case AStarAlgorithm::Result::NO_PATH:
      throw nav2_core::NoValidPathCouldBeFound("A* 未能找到可行路径");
    case AStarAlgorithm::Result::START_INVALID:
      throw nav2_core::StartOutsideMapBounds("起点非法");
    case AStarAlgorithm::Result::GOAL_INVALID:
      throw nav2_core::GoalOccupied("终点非法或位于障碍上");
    default:
      throw nav2_core::PlannerException("A* 规划失败，未知原因");
  }

  const auto & grid_path = astar_->getPath();
  if (grid_path.empty()) {
    throw nav2_core::NoValidPathCouldBeFound("A* 返回了空路径");
  }

  const auto & stats = astar_->getStats();
  RCLCPP_INFO(
    logger_,
    "A* 规划完成: 扩展 %zu 节点, 打开 %zu 节点, 路径 %zu 格, 代价 %.2f, 耗时 %.2f ms",
    stats.nodes_expanded, stats.nodes_opened, stats.path_length,
    stats.path_cost, stats.planning_time_ms);

  // -------- 插值为世界坐标路径 --------
  global_path = interpolatePath(grid_path, costmap_, interpolation_resolution_);

  // Grid centers are search representatives, not the user's continuous endpoints.
  // Keep endpoints in their already-validated cells and recompute segment headings.
  if (!global_path.poses.empty()) {
    global_path.poses.front().pose.position.x = start.pose.position.x;
    global_path.poses.front().pose.position.y = start.pose.position.y;
    global_path.poses.back().pose.position.x = goal.pose.position.x;
    global_path.poses.back().pose.position.y = goal.pose.position.y;
    for (size_t i = 0; i + 1 < global_path.poses.size(); ++i) {
      const auto & next = global_path.poses[i + 1].pose.position;
      auto & current = global_path.poses[i].pose;
      const double dx = next.x - current.position.x;
      const double dy = next.y - current.position.y;
      if (std::hypot(dx, dy) > 1e-9) {
        const double theta = std::atan2(dy, dx);
        current.orientation.x = current.orientation.y = 0.0;
        current.orientation.z = std::sin(theta / 2.0);
        current.orientation.w = std::cos(theta / 2.0);
      }
    }
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  // 若要求「末端接近朝向」，用倒数第二点的方向计算最终朝向，避免原地转向
  if (use_final_approach_orientation_ && global_path.poses.size() >= 2) {
    const auto & last = global_path.poses.back().pose.position;
    const auto & prev = global_path.poses[global_path.poses.size() - 2].pose.position;
    const double theta = std::atan2(last.y - prev.y, last.x - prev.x);
    global_path.poses.back().pose.orientation.x = 0.0;
    global_path.poses.back().pose.orientation.y = 0.0;
    global_path.poses.back().pose.orientation.z = std::sin(theta / 2.0);
    global_path.poses.back().pose.orientation.w = std::cos(theta / 2.0);
  }

  return global_path;
}

nav_msgs::msg::Path AStarPlanner::interpolatePath(
  const std::vector<GridCoord> & grid_path,
  const nav2_costmap_2d::Costmap2D * costmap,
  double interpolation_resolution)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  const double resolution = costmap->getResolution();
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();

  if (interpolation_resolution <= 0.0) {
    interpolation_resolution = resolution;
  }

  // 把起点加入
  {
    double wx, wy;
    AStarAlgorithm::gridToWorld(
      grid_path.front().x, grid_path.front().y, origin_x, origin_y, resolution, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  // 在相邻格子之间线性插值
  for (size_t i = 1; i < grid_path.size(); ++i) {
    double wx_prev, wy_prev, wx_cur, wy_cur;
    AStarAlgorithm::gridToWorld(
      grid_path[i - 1].x, grid_path[i - 1].y, origin_x, origin_y, resolution, wx_prev, wy_prev);
    AStarAlgorithm::gridToWorld(
      grid_path[i].x, grid_path[i].y, origin_x, origin_y, resolution, wx_cur, wy_cur);

    const double dx = wx_cur - wx_prev;
    const double dy = wy_cur - wy_prev;
    const double segment_length = std::hypot(dx, dy);

    // 该段需要插几个点
    const int num_steps = std::max(1, static_cast<int>(segment_length / interpolation_resolution));

    for (int s = 1; s <= num_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(num_steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = wx_prev + t * dx;
      pose.pose.position.y = wy_prev + t * dy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }

  // 路径点朝向：沿路径切线方向，让控制器知道朝向
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const size_t next = std::min(i + 1, path.poses.size() - 1);
    const size_t prev = (i == 0) ? 0 : i - 1;
    const double dx = path.poses[next].pose.position.x - path.poses[prev].pose.position.x;
    const double dy = path.poses[next].pose.position.y - path.poses[prev].pose.position.y;
    if (std::hypot(dx, dy) > 1e-9) {
      const double yaw = std::atan2(dy, dx);
      path.poses[i].pose.orientation.x = 0.0;
      path.poses[i].pose.orientation.y = 0.0;
      path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    }
  }

  return path;
}

}  // namespace nav2_astar_planner

// 关键：导出插件类，让 pluginlib 能实例化它
PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
