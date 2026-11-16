// Copyright (c) 2026
// Nav2 Controller 适配层公共基类实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/controller_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

void ControllerBase::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("父节点已失效，无法 configure 控制器");
  }

  plugin_name_ = name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  if (!costmap_ros_) {
    throw nav2_core::ControllerException("局部代价地图为空");
  }
  costmap_ = costmap_ros_->getCostmap();
  if (!costmap_) {
    throw nav2_core::ControllerException("无法获取局部代价地图数据");
  }

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  declareAndGet(node, "transform_tolerance", 0.1, transform_tolerance_);
  declareAndGet(node, "goal_tolerance", 0.25, goal_tolerance_);
  declareAndGet(node, "terminal_angular_velocity", 1.5, terminal_angular_velocity_);

  nav2_util::declare_parameter_if_not_declared(
    node, "controller_frequency", rclcpp::ParameterValue(20.0));
  node->get_parameter("controller_frequency", control_frequency_);
  control_frequency_ = std::max(1.0, control_frequency_);

  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/controller_lab/controller_diagnostics", rclcpp::QoS(10));
  configureParams(node);

  RCLCPP_INFO(
    logger_,
    "ControllerLab(%s) 已配置: frame=%s -> %s, freq=%.1f Hz, goal_tol=%.2f m",
    plugin_name_.c_str(), global_frame_.c_str(), base_frame_.c_str(),
    control_frequency_, goal_tolerance_);
}

void ControllerBase::cleanup()
{
  RCLCPP_INFO(logger_, "ControllerLab(%s) 清理资源", plugin_name_.c_str());
  resetController();
  global_plan_.poses.clear();
  has_plan_ = false;
  diagnostics_pub_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void ControllerBase::activate()
{
  if (diagnostics_pub_) {diagnostics_pub_->on_activate();}
  RCLCPP_INFO(logger_, "ControllerLab(%s) 已激活", plugin_name_.c_str());
}

void ControllerBase::deactivate()
{
  if (diagnostics_pub_) {diagnostics_pub_->on_deactivate();}
  RCLCPP_INFO(logger_, "ControllerLab(%s) 已去激活", plugin_name_.c_str());
}

void ControllerBase::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("收到空路径");
  }

  global_plan_ = path;
  if (global_plan_.header.frame_id.empty()) {
    global_plan_.header.frame_id = global_frame_;
  }
  has_plan_ = true;
  resetController();

  RCLCPP_DEBUG(
    logger_, "ControllerLab(%s) 收到路径: %zu 个点, frame=%s",
    plugin_name_.c_str(), global_plan_.poses.size(),
    global_plan_.header.frame_id.c_str());
}

std::vector<Pose2D> ControllerBase::transformPlanToLocalFrame()
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("全局路径为空");
  }

  const std::string source_frame = global_plan_.header.frame_id.empty() ?
    global_frame_ : global_plan_.header.frame_id;

  std::vector<Pose2D> path;
  path.reserve(global_plan_.poses.size());
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> transforms;

  for (const auto & input : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped transformed = input;

    if (input.header.frame_id.empty()) {
      transformed.header.frame_id = source_frame;
    }

    if (transformed.header.frame_id != global_frame_) {
      if (!tf_) {
        throw nav2_core::ControllerTFError("没有 TF buffer，无法变换全局路径");
      }
      // 一次控制周期内，每个源坐标系使用同一份最新可用变换。
      // 按当前时间查询会因 SLAM TF 略有延迟而触发向未来外推。
      try {
        auto transform = transforms.find(transformed.header.frame_id);
        if (transform == transforms.end()) {
          transform = transforms.emplace(
            transformed.header.frame_id,
            tf_->lookupTransform(
              global_frame_, transformed.header.frame_id, tf2::TimePointZero,
              tf2::durationFromSec(transform_tolerance_))).first;
        }
        geometry_msgs::msg::PoseStamped output;
        tf2::doTransform(transformed, output, transform->second);
        transformed = output;
      } catch (const tf2::TransformException & ex) {
        throw nav2_core::ControllerTFError(
                std::string("路径变换到局部坐标系失败: ") + ex.what());
      }
    }

    Pose2D pose;
    pose.x = transformed.pose.position.x;
    pose.y = transformed.pose.position.y;
    pose.yaw = quaternionToYaw(transformed.pose.orientation);
    path.push_back(pose);
  }

  // 保存原始路径末位姿的朝向（AStarPlanner 已写入目标朝向），供终点旋转
  // 阶段使用。注意：recomputePathYaw 会把所有 yaw 覆盖成相邻点切线，
  // 所以必须在这之后、覆盖之前提取 —— 这正是"路径终点朝向"和
  // "路径切线"是两回事的地方。
  path_terminal_yaw_ = path.back().yaw;

  // 统一用相邻点切线，避免上游朝向缺失或不一致
  recomputePathYaw(path);
  return path;
}

geometry_msgs::msg::TwistStamped ControllerBase::zeroTwist() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = base_frame_;
  if (clock_) {
    command.header.stamp = clock_->now();
  }
  return command;
}

void ControllerBase::publishDiagnostics(
  const std::string & stage, const std::string & detail, double compute_ms,
  const geometry_msgs::msg::TwistStamped * command)
{
  if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {return;}
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "controller_lab/" + plugin_name_;
  status.hardware_id = plugin_name_;
  status.level = command ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = detail.empty() ? stage : detail;
  auto add = [&](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key; item.value = value; status.values.push_back(item);
    };
  add("controller", plugin_name_);
  add("stage", stage);
  add("compute_ms", std::to_string(compute_ms));
  add("goal_error_m", std::to_string(goal_error_m_));
  add("goal_yaw_error_rad", std::to_string(goal_yaw_error_rad_));
  add("frame_id", global_frame_);
  if (command) {
    add("linear_velocity", std::to_string(command->twist.linear.x));
    add("angular_velocity", std::to_string(command->twist.angular.z));
  }
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

geometry_msgs::msg::TwistStamped ControllerBase::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto begin = std::chrono::steady_clock::now();
  auto elapsed = [&]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    };
  goal_error_m_ = goal_yaw_error_rad_ = -1.0;
  control_stage_ = "tracking";
  try {
    auto output = computeVelocityCommandsImpl(pose, velocity, goal_checker);
    publishDiagnostics(control_stage_, "", elapsed(), &output);
    return output;
  } catch (const std::exception & ex) {
    publishDiagnostics("control_error", ex.what(), elapsed(), nullptr);
    throw;
  }
}

geometry_msgs::msg::TwistStamped ControllerBase::computeVelocityCommandsImpl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (!has_plan_ || global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("还没有收到全局路径");
  }
  if (!costmap_) {
    throw nav2_core::ControllerException("控制器尚未 configure 或代价地图已释放");
  }

  const std::vector<Pose2D> path = transformPlanToLocalFrame();

  Pose2D robot;
  robot.x = pose.pose.position.x;
  robot.y = pose.pose.position.y;
  robot.yaw = quaternionToYaw(pose.pose.orientation);
  if (!path.empty()) {
    goal_error_m_ = distance(robot, path.back());
    goal_yaw_error_rad_ = std::abs(normalizeAngle(path_terminal_yaw_ - robot.yaw));
  }

  const double dt = 1.0 / std::max(control_frequency_, 1.0);

  Control command;
  // ============================================================
  // 终点旋转阶段（2026-11 端到端实验新增）
  //
  // 现象：机器人位置已到达 goal_tolerance 内，导航却永不返回成功，
  // 控制器停在原地直到超时。
  //
  // 根因：六种控制器核心都只负责"跟路径"，路径走完就输出零速；
  // 而 nav2 的 SimpleGoalChecker 同时检查位置和朝向（对路径末位姿，
  // 即 A* 写入的目标朝向）。缺了"原地转到目标朝向"这一步就死锁。
  //
  // 官方控制器的对应实现：RPP 的 rotateToGoalHeading 阶段、
  // MPPI 的 GoalAngleCritic。这里在基类统一实现，六种控制器共享；
  // 纯算法在 path_utils::rotateInPlace（可单测）。
  //
  // 局限：本阶段不查局部代价地图（教学简化）。贴着障碍的目标
  // 旋转可能碰到障碍， footprint 检查是 TODO（见 doc 14 后续可做）。
  // ============================================================
  if (!path.empty() && remainingDistance(robot, path) <= goal_tolerance_) {
    control_stage_ = "terminal_rotation";
    command = rotateInPlace(robot, path_terminal_yaw_, terminal_angular_velocity_);
  } else if (needsCostmap()) {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));
    command = computeCommand(robot, path, velocity.linear.x, dt);
  } else {
    command = computeCommand(robot, path, velocity.linear.x, dt);
  }

  if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) {
    throw nav2_core::NoValidControl("控制器输出了非有限值");
  }

  if (speed_limit_active_) {
    double limit = speed_limit_;
    if (speed_limit_is_percentage_) {
      const double ratio = speed_limit_ > 1.0 ? speed_limit_ / 100.0 : speed_limit_;
      limit = maxLinearVelocity() * std::max(0.0, ratio);
    }
    command.linear = clamp(command.linear, -std::abs(limit), std::abs(limit));
  }

  geometry_msgs::msg::TwistStamped command_msg;
  command_msg.header.frame_id = base_frame_;
  if (clock_) {
    command_msg.header.stamp = clock_->now();
  }
  command_msg.twist.linear.x = command.linear;
  command_msg.twist.angular.z = command.angular;

  RCLCPP_DEBUG(
    logger_, "ControllerLab(%s): v=%.3f m/s, w=%.3f rad/s, path=%zu",
    plugin_name_.c_str(), command.linear, command.angular, path.size());

  return command_msg;
}

void ControllerBase::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  speed_limit_active_ = true;

  RCLCPP_INFO(
    logger_, "ControllerLab(%s) 速度限制: %.3f (%s)",
    plugin_name_.c_str(), speed_limit, percentage ? "百分比" : "绝对 m/s");
}

}  // namespace nav2_controller_lab
