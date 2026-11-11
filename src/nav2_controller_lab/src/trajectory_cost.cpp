// Copyright (c) 2026
// 轨迹评分器实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/trajectory_cost.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

double evaluateTrajectory(
  const std::vector<Pose2D> & trajectory,
  const std::vector<Pose2D> & path,
  const std::function<uint8_t(double, double)> & cost_at,
  const TrajectoryCostParams & params,
  double command_v,
  double command_w)
{
  if (trajectory.empty() || path.empty()) {
    return params.collision_cost;
  }

  double path_cost = 0.0;
  double heading_cost = 0.0;
  double obstacle_cost = 0.0;

  for (const auto & pose : trajectory) {
    const PathProjection projection = projectOntoPath(pose, path);
    path_cost += projection.distance;
    heading_cost += std::abs(normalizeAngle(projection.path_yaw - pose.yaw));

    if (cost_at) {
      const uint8_t cell = cost_at(pose.x, pose.y);
      if (static_cast<int>(cell) >= params.collision_threshold) {
        return params.collision_cost;
      }
      obstacle_cost += static_cast<double>(cell) / 252.0;
    }
  }

  const double inv_n = 1.0 / static_cast<double>(trajectory.size());
  path_cost *= inv_n;
  heading_cost *= inv_n;
  obstacle_cost *= inv_n;

  const double goal_cost = distance(trajectory.back(), path.back());
  const double velocity_cost = std::abs(command_v - params.desired_linear_velocity);
  const double control_cost = std::abs(command_w);

  return params.path_weight * path_cost +
         params.goal_weight * goal_cost +
         params.obstacle_weight * obstacle_cost +
         params.heading_weight * heading_cost +
         params.velocity_weight * velocity_cost +
         params.control_weight * control_cost;
}

}  // namespace nav2_controller_lab
