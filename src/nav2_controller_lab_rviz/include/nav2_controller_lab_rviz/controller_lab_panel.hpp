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
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <QtWidgets>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rviz_common/panel.hpp"

#include "geometry_msgs/msg/twist.hpp"
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

  QLabel * status_label_{nullptr};
  QLabel * command_label_{nullptr};
  QLabel * hint_label_{nullptr};
  QCheckBox * debug_checkbox_{nullptr};
  QTimer * refresh_timer_{nullptr};
};

}  // namespace nav2_controller_lab_rviz

#endif  // NAV2_CONTROLLER_LAB_RVIZ__CONTROLLER_LAB_PANEL_HPP_
