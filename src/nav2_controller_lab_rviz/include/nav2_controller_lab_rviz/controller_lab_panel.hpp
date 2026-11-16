// Copyright (c) 2026
// Controller Lab RViz Panel
//
// 功能：
//   1. 一键切换 /controller_selector（Pure Pursuit / PID / Stanley / DWB / MPPI）
//   2. 显示当前控制器、/cmd_vel
//   3. 开关 /controller_lab/debug_enable（控制可视化节点的 MarkerArray）
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB_RVIZ__CONTROLLER_LAB_PANEL_HPP_
#define NAV2_CONTROLLER_LAB_RVIZ__CONTROLLER_LAB_PANEL_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <QtWidgets>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rviz_common/panel.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class QLabel;
class QPushButton;
class QCheckBox;
class QTimer;

namespace nav2_controller_lab_rviz
{

class ControllerLabPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ControllerLabPanel(QWidget * parent = nullptr);
  ~ControllerLabPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void selectPurePursuit();
  void selectPid();
  void selectStanley();
  void selectDwb();
  void selectMppi();
  void selectMpc();
  void onDebugToggled(bool checked);
  void refreshLabels();

private:
  void setupUi();
  void selectController(const std::string & controller_id);
  void onControllerSelected(const std_msgs::msg::String::SharedPtr msg);
  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selector_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr debug_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selector_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{true};

  std::mutex state_mutex_;
  std::string selected_controller_{"FollowPath"};
  std::atomic<double> linear_velocity_{0.0};
  std::atomic<double> angular_velocity_{0.0};

  struct VelocitySample {
    double v{0.0}, w{0.0};
    std::chrono::steady_clock::time_point received{};
  };
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> chain_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stats_client_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::atomic<bool> stats_pending_{false};
  int64_t stats_request_id_{0};
  std::chrono::steady_clock::time_point stats_requested_{};
  std::map<std::string, VelocitySample> chain_;
  std::map<std::string, std::string> diagnostic_values_;
  std::string diagnostic_detail_, mode_info_, action_info_{"未观测到导航目标"};
  std::chrono::steady_clock::time_point diagnostic_received_{}, mode_received_{}, action_received_{};
  QLabel * diagnostic_label_{nullptr};
  QLabel * chain_label_{nullptr};
  QLabel * mode_label_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * command_label_{nullptr};
  QLabel * hint_label_{nullptr};
  QCheckBox * debug_checkbox_{nullptr};
  QTimer * refresh_timer_{nullptr};
};

}  // namespace nav2_controller_lab_rviz

#endif  // NAV2_CONTROLLER_LAB_RVIZ__CONTROLLER_LAB_PANEL_HPP_
