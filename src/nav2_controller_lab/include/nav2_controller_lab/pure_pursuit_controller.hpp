// Copyright (c) 2026
// Pure Pursuit 控制器插件
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_CONTROLLER_LAB__PURE_PURSUIT_CONTROLLER_HPP_

#include "nav2_controller_lab/controller_base.hpp"
#include "nav2_controller_lab/pure_pursuit_core.hpp"

namespace nav2_controller_lab
{

class PurePursuitController : public ControllerBase
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

protected:
  void configureParams(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) override;

  Control computeCommand(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    double current_speed,
    double dt) override;

  void resetController() override {core_.reset();}

  double maxLinearVelocity() const override {return params_.max_linear_velocity;}

private:
  PurePursuitCore core_;
  PurePursuitParams params_;
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__PURE_PURSUIT_CONTROLLER_HPP_
