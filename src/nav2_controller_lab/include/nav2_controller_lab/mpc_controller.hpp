// Copyright (c) 2026
// MPC-lite 控制器插件
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__MPC_CONTROLLER_HPP_
#define NAV2_CONTROLLER_LAB__MPC_CONTROLLER_HPP_

#include "nav2_controller_lab/controller_base.hpp"
#include "nav2_controller_lab/mpc_core.hpp"
#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

class MpcController : public ControllerBase
{
public:
  MpcController() = default;
  ~MpcController() override = default;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommandsImpl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override
  {
    current_angular_ = velocity.angular.z;
    const auto path = transformPlanToLocalFrame();
    const auto & q = pose.pose.orientation;
    const Pose2D robot{pose.pose.position.x, pose.pose.position.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    if (!path.empty()) {
      goal_error_m_ = distance(robot, path.back());
      goal_yaw_error_rad_ = std::abs(normalizeAngle(path_terminal_yaw_ - robot.yaw));
    }
    // Nav2 checks Euclidean goal distance, not remaining polyline arc length.
    if (!path.empty() && distance(robot, path.back()) <= goal_tolerance_) {
      control_stage_ = "terminal_rotation";
      core_.reset();
      publishPrediction();
      const auto command = rotateInPlace(robot, path_terminal_yaw_, terminal_angular_velocity_);
      geometry_msgs::msg::TwistStamped output;
      output.header.frame_id = base_frame_;
      output.header.stamp = clock_->now();
      output.twist.angular.z = command.angular;
      return output;
    }
    return ControllerBase::computeVelocityCommandsImpl(pose, velocity, checker);
  }

protected:
  void configureParams(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) override;

  Control computeCommand(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    double current_speed,
    double dt) override;

  // Replanning must not destroy the warm start; all seeds are rescored on the new path.
  void resetController() override {}

  bool needsCostmap() const override {return true;}

  double maxLinearVelocity() const override {return params_.max_linear_velocity;}

private:
  void publishPrediction();
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  double current_angular_{0.0};
  MpcCore core_;
  MpcParams params_;
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__MPC_CONTROLLER_HPP_
