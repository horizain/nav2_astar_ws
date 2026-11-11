// Copyright (c) 2026
// PID 路径跟踪基线 —— 纯 C++，不依赖 ROS
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__PID_CORE_HPP_
#define NAV2_CONTROLLER_LAB__PID_CORE_HPP_

#include <vector>

#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

struct PidParams
{
  double kp_heading{0.8};
  double ki_heading{0.0};
  double kd_heading{0.05};
  double kp_cross{0.6};

  double kp_speed{1.0};
  double ki_speed{0.0};
  double kd_speed{0.0};

  double max_linear_velocity{0.5};
  double max_angular_velocity{1.9};
  double goal_tolerance{0.25};
  double slow_radius{0.6};
  double heading_slowdown{0.8};
  double integral_limit{1.0};
};

/**
 * @brief 用 PID 跟踪路径的“最近点 + 切线”
 *
 * 这是一个教学基线，不是生产级控制器。它会暴露 PID 在弯道、
 * 高速、障碍场景下的局限。
 */
class PidCore
{
public:
  explicit PidCore(const PidParams & params = PidParams());

  Control compute(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    double current_speed,
    double dt);

  void reset();

  const PidParams & params() const {return params_;}
  PidParams & params() {return params_;}

private:
  PidParams params_;
  double heading_integral_{0.0};
  double previous_heading_error_{0.0};
  bool heading_initialized_{false};
  double speed_integral_{0.0};
  double previous_speed_error_{0.0};
  bool speed_initialized_{false};
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__PID_CORE_HPP_
