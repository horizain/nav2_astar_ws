// Copyright (c) 2026
// Pure Pursuit 控制器核心 —— 纯 C++，不依赖 ROS
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__PURE_PURSUIT_CORE_HPP_
#define NAV2_CONTROLLER_LAB__PURE_PURSUIT_CORE_HPP_

#include <vector>

#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

struct PurePursuitParams
{
  double lookahead_distance{0.6};
  double min_lookahead_distance{0.3};
  double max_lookahead_distance{1.2};
  bool use_velocity_scaled_lookahead{true};
  double lookahead_time{1.0};

  double max_linear_velocity{0.5};
  double min_linear_velocity{0.05};
  double max_angular_velocity{1.9};
  double goal_tolerance{0.25};
  double slowdown_radius{0.6};
  double curvature_slowdown{0.7};
};

/**
 * @brief 纯几何 Pure Pursuit
 *
 * 前视点 P 在机器人坐标系下为 (x, y)，曲率
 *   kappa = 2y / (x^2 + y^2)
 * 角速度
 *   omega = v * kappa
 */
class PurePursuitCore
{
public:
  explicit PurePursuitCore(const PurePursuitParams & params = PurePursuitParams());

  Control compute(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    double current_speed);

  void reset() {}

  const PurePursuitParams & params() const {return params_;}
  PurePursuitParams & params() {return params_;}

private:
  PurePursuitParams params_;
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__PURE_PURSUIT_CORE_HPP_
