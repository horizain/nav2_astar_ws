// Copyright (c) 2026
// Nav2 Controller 适配层公共基类
//
// 负责所有控制器共享的工作：
//   - 生命周期
//   - 参数前缀
//   - 全局路径 -> 局部代价地图坐标系变换
//   - 目标检查、速度限制、TwistStamped 封装
//
// 具体算法只需实现 configureParams() 和 computeCommand()。
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__CONTROLLER_BASE_HPP_
#define NAV2_CONTROLLER_LAB__CONTROLLER_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

class ControllerBase : public nav2_core::Controller
{
public:
  ~ControllerBase() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommandsImpl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker);
  void publishDiagnostics(
    const std::string & stage, const std::string & detail, double compute_ms,
    const geometry_msgs::msg::TwistStamped * command);
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::string control_stage_{"waiting"};
  double goal_error_m_{-1.0};
  double goal_yaw_error_rad_{-1.0};
  /**
   * @brief 读取本控制器自己的参数
   *
   * plugin_name_ 已经设置好，子类用 declareAndGet(node, "xxx", default, member)
   * 即可自动加上 "FollowPath.xxx" 前缀。
   */
  virtual void configureParams(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) = 0;

  /**
   * @brief 纯算法入口：输入局部坐标系下的位姿/路径，输出控制量
   */
  virtual Control computeCommand(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    double current_speed,
    double dt) = 0;

  /// 新路径到来时清空内部状态（PID 积分、MPPI 标称序列等）
  virtual void resetController() {}

  /// 只有 DWB / MPPI 需要读局部代价地图
  virtual bool needsCostmap() const {return false;}

  /// 用于百分比速度限制
  virtual double maxLinearVelocity() const = 0;

  template<typename T>
  void declareAndGet(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::string & suffix,
    const T & default_value,
    T & out)
  {
    const std::string parameter_name = plugin_name_ + "." + suffix;
    nav2_util::declare_parameter_if_not_declared(
      node, parameter_name, rclcpp::ParameterValue(default_value));
    node->get_parameter(parameter_name, out);
  }

  std::vector<Pose2D> transformPlanToLocalFrame();
  geometry_msgs::msg::TwistStamped zeroTwist() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  std::string plugin_name_;
  std::string global_frame_;
  std::string base_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_controller_lab")};
  rclcpp::Clock::SharedPtr clock_;

  double transform_tolerance_{0.1};
  double control_frequency_{20.0};
  double goal_tolerance_{0.25};
  double terminal_angular_velocity_{1.5};  ///< 终点旋转阶段的最大角速度 (rad/s)
  /// 终点旋转的目标朝向 = 原始路径末位姿朝向（经 TF 到局部坐标系）。
  /// 注意不能用 path.back().yaw —— recomputePathYaw 已把它覆盖成切线。
  double path_terminal_yaw_{0.0};

  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};
  bool speed_limit_active_{false};

  nav_msgs::msg::Path global_plan_;
  bool has_plan_{false};
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__CONTROLLER_BASE_HPP_
