// Copyright (c) 2026
// A* 全局路径规划器 —— Nav2 插件封装层
//
// 本文件负责：
//   1. 实现 nav2_core::GlobalPlanner 接口
//   2. 处理生命周期（configure / activate / deactivate / cleanup）
//   3. 从代价地图取数据、转换坐标系
//   4. 把 A* 输出的格子路径插值为稠密的世界坐标路径
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_astar_planner/astar_algorithm.hpp"

namespace nav2_astar_planner
{

/**
 * @brief 基于 A* 的 Nav2 全局路径规划器插件
 */
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  /**
   * @brief 插件初始化：读取参数、绑定代价地图
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 释放资源
   */
  void cleanup() override;

  /**
   * @brief 激活（可在此启动线程）
   */
  void activate() override;

  /**
   * @brief 去激活
   */
  void deactivate() override;

  /**
   * @brief 核心方法：生成从 start 到 goal 的全局路径
   *
   * @param start 起点位姿（已在 global_frame 下）
   * @param goal  终点位姿
   * @param cancel_checker 取消检查回调
   * @return nav_msgs::msg::Path 规划出的路径
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  /**
   * @brief 把格子路径插值为稠密的世界坐标路径
   *
   * 为什么要插值？
   *   A* 输出的路径是「格子序列」，相邻格子间距 = 分辨率（如 0.05m）。
   *   但局部控制器（controller_server）需要足够稠密的路径点来跟踪，
   *   且部分控制器默认会按固定间距采样。这里做线性插值保证点间距
   *   不超过 interpolation_resolution。
   *
   * @param grid_path A* 输出的格子路径
   * @param costmap 代价地图（提供 origin / resolution）
   * @param interpolation_resolution 插值间距（米）
   */
  nav_msgs::msg::Path interpolatePath(
    const std::vector<GridCoord> & grid_path,
    const nav2_costmap_2d::Costmap2D * costmap,
    double interpolation_resolution);

  // ---- 成员 ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // A* 算法实例
  std::unique_ptr<AStarAlgorithm> astar_;

  // 可配置参数
  bool allow_diagonal_{true};
  bool allow_unknown_{true};
  double cost_weight_{0.0};
  double heuristic_weight_{1.0};
  double interpolation_resolution_{0.05};
  int max_expansions_{1000000};
  bool use_final_approach_orientation_{false};
};

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
