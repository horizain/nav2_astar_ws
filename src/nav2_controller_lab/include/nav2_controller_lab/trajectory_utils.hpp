// Copyright (c) 2026
// 轨迹 rollout 工具 —— 纯 C++
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__TRAJECTORY_UTILS_HPP_
#define NAV2_CONTROLLER_LAB__TRAJECTORY_UTILS_HPP_

#include <vector>

#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

/**
 * @brief 用恒定 (v, w) 向前模拟 N 步差速运动
 *
 *   x += v cos(theta) dt
 *   y += v sin(theta) dt
 *   theta += w dt
 */
std::vector<Pose2D> rolloutConstantControl(
  const Pose2D & start,
  double linear_velocity,
  double angular_velocity,
  int steps,
  double dt);

/**
 * @brief 用一串控制量向前模拟
 */
std::vector<Pose2D> rolloutControlSequence(
  const Pose2D & start,
  const std::vector<Control> & controls,
  double dt);

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__TRAJECTORY_UTILS_HPP_
