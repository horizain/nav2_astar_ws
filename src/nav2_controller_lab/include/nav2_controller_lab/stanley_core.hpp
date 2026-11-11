// Copyright (c) 2026
// Stanley 控制器核心 —— 纯 C++，不依赖 ROS
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__STANLEY_CORE_HPP_
#define NAV2_CONTROLLER_LAB__STANLEY_CORE_HPP_

#include <vector>

#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

struct StanleyParams
{
  double k_cross{1.0};
  double k_heading{1.0};
  double wheelbase{0.5};
  double front_axle_offset{0.0};

  double max_linear_velocity{0.5};
  double min_linear_velocity{0.05};
  double max_angular_velocity{1.9};
  double max_steering_angle{0.7};
  double goal_tolerance{0.25};
  double slowdown_radius{0.6};
  double heading_slowdown{0.8};
};

/**
 * @brief Stanley 路径跟踪
 *
 *   delta = heading_error + atan(k_cross * cross_track / (v + eps))
 *   omega = v * tan(delta) / wheelbase
 *
 * front_axle_offset 是“前轴”相对机器人中心的前向偏移；差速车可以设 0。
 */
class StanleyCore
{
public:
  explicit StanleyCore(const StanleyParams & params = StanleyParams());

  Control compute(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    double current_speed);

  void reset() {}

  const StanleyParams & params() const {return params_;}
  StanleyParams & params() {return params_;}

private:
  StanleyParams params_;
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__STANLEY_CORE_HPP_
