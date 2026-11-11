// Copyright (c) 2026
// 轨迹 rollout 工具实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_controller_lab
{

std::vector<Pose2D> rolloutConstantControl(
  const Pose2D & start,
  double linear_velocity,
  double angular_velocity,
  int steps,
  double dt)
{
  std::vector<Control> controls(
    static_cast<size_t>(std::max(0, steps)),
    Control{linear_velocity, angular_velocity});
  return rolloutControlSequence(start, controls, dt);
}

std::vector<Pose2D> rolloutControlSequence(
  const Pose2D & start,
  const std::vector<Control> & controls,
  double dt)
{
  std::vector<Pose2D> trajectory;
  trajectory.reserve(controls.size() + 1);
  trajectory.push_back(start);

  Pose2D pose = start;
  for (const auto & control : controls) {
    pose.x += control.linear * std::cos(pose.yaw) * dt;
    pose.y += control.linear * std::sin(pose.yaw) * dt;
    pose.yaw = normalizeAngle(pose.yaw + control.angular * dt);
    trajectory.push_back(pose);
  }
  return trajectory;
}

}  // namespace nav2_controller_lab
