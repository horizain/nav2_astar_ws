// Copyright (c) 2026
// 轨迹评分器 —— DWB / MPPI 共用的 critics
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__TRAJECTORY_COST_HPP_
#define NAV2_CONTROLLER_LAB__TRAJECTORY_COST_HPP_

#include <cstdint>
#include <functional>
#include <vector>

#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

/**
 * @brief 轨迹代价权重
 *
 * 与 Nav2 DWB/MPPI 的 critics 对应：
 *  - path_weight      ↔ PathAlign / PathFollow
 *  - goal_weight      ↔ GoalCritic
 *  - obstacle_weight  ↔ CostCritic
 *  - heading_weight   ↔ PathAngleCritic
 *  - velocity_weight  ↔ 速度偏好
 *  - control_weight   ↔ 平滑/能耗
 */
struct TrajectoryCostParams
{
  double path_weight{1.0};
  double goal_weight{2.0};
  double obstacle_weight{100.0};
  double heading_weight{0.5};
  double velocity_weight{0.2};
  double control_weight{0.05};

  double collision_cost{1.0e6};
  int collision_threshold{254};
  double desired_linear_velocity{0.4};
};

/**
 * @brief 给一条未来轨迹打分，越小越好
 *
 * @param trajectory 未来轨迹（从当前位姿开始）
 * @param path 全局路径
 * @param cost_at 世界坐标 -> 代价地图值（0~255），越界返回 255
 * @param params 权重
 * @param command_v 该轨迹使用的线速度
 * @param command_w 该轨迹使用的角速度
 */
double evaluateTrajectory(
  const std::vector<Pose2D> & trajectory,
  const std::vector<Pose2D> & path,
  const std::function<uint8_t(double, double)> & cost_at,
  const TrajectoryCostParams & params,
  double command_v,
  double command_w);

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__TRAJECTORY_COST_HPP_
