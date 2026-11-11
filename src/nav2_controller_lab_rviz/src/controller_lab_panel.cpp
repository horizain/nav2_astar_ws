// Copyright (c) 2026
// Controller Lab RViz Panel 实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab_rviz/controller_lab_panel.hpp"

#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller_lab_rviz
{

ControllerLabPanel::ControllerLabPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  node_ = std::make_shared<rclcpp::Node>("controller_lab_rviz_panel");

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  selector_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "controller_selector", qos);
  debug_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "controller_lab/debug_enable", qos);

  selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "controller_selector", qos,
    std::bind(&ControllerLabPanel::onControllerSelected, this, std::placeholders::_1));

  cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&ControllerLabPanel::onCommand, this, std::placeholders::_1));

  setupUi();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
        if (executor_) {
          executor_->spin();
        }
      });

  // 默认打开可视化；transient_local 保证晚启动的 visualizer 也能收到。
  std_msgs::msg::Bool enabled;
  enabled.data = true;
  debug_pub_->publish(enabled);
}

ControllerLabPanel::~ControllerLabPanel()
{
  running_ = false;
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void ControllerLabPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  setMinimumWidth(340);
}

void ControllerLabPanel::setupUi()
{
  auto * main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(8, 8, 8, 8);
  main_layout->setSpacing(6);

  auto * title = new QLabel("<b>Controller Lab</b>");
  title->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(title);

  auto * grid = new QGridLayout();
  grid->setSpacing(4);

  auto add_button = [this, grid](const QString & text, int row, int col, auto slot) {
      auto * button = new QPushButton(text);
      button->setMinimumHeight(32);
      connect(button, &QPushButton::clicked, this, slot);
      grid->addWidget(button, row, col);
    };

  add_button("Pure Pursuit", 0, 0, &ControllerLabPanel::selectPurePursuit);
  add_button("PID", 0, 1, &ControllerLabPanel::selectPid);
  add_button("Stanley", 1, 0, &ControllerLabPanel::selectStanley);
  add_button("DWB-lite", 1, 1, &ControllerLabPanel::selectDwb);
  add_button("MPPI-lite", 2, 0, &ControllerLabPanel::selectMppi);
  add_button("MPC-lite", 2, 1, &ControllerLabPanel::selectMpc);

  main_layout->addLayout(grid);

  status_label_ = new QLabel();
  status_label_->setWordWrap(true);
  main_layout->addWidget(status_label_);

  command_label_ = new QLabel();
  main_layout->addWidget(command_label_);

  debug_checkbox_ = new QCheckBox("显示调试标记（前视点 / 预测轨迹）");
  debug_checkbox_->setChecked(true);
  connect(
    debug_checkbox_, &QCheckBox::toggled,
    this, &ControllerLabPanel::onDebugToggled);
  main_layout->addWidget(debug_checkbox_);

  hint_label_ = new QLabel(
    "MarkerArray: <code>/controller_lab/markers</code><br>"
    "切换实例名：FollowPath / PID / Stanley / DWB / MPPI / MPC");
  hint_label_->setWordWrap(true);
  hint_label_->setStyleSheet("color: #b0b0b0;");
  main_layout->addWidget(hint_label_);

  main_layout->addStretch();

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &ControllerLabPanel::refreshLabels);
  refresh_timer_->start(100);

  refreshLabels();
}

void ControllerLabPanel::selectController(const std::string & controller_id)
{
  if (!selector_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = controller_id;
  selector_pub_->publish(msg);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    selected_controller_ = controller_id;
  }

  RCLCPP_INFO(
    node_->get_logger(), "切换控制器 -> %s", controller_id.c_str());
  refreshLabels();
}

void ControllerLabPanel::selectPurePursuit()
{
  selectController("FollowPath");
}

void ControllerLabPanel::selectPid()
{
  selectController("PID");
}

void ControllerLabPanel::selectStanley()
{
  selectController("Stanley");
}

void ControllerLabPanel::selectDwb()
{
  selectController("DWB");
}

void ControllerLabPanel::selectMppi()
{
  selectController("MPPI");
}

void ControllerLabPanel::selectMpc()
{
  selectController("MPC");
}

void ControllerLabPanel::onDebugToggled(bool checked)
{
  if (!debug_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = checked;
  debug_pub_->publish(msg);
}

void ControllerLabPanel::onControllerSelected(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  selected_controller_ = msg->data;
}

void ControllerLabPanel::onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linear_velocity_ = msg->linear.x;
  angular_velocity_ = msg->angular.z;
}

void ControllerLabPanel::refreshLabels()
{
  std::string selected;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    selected = selected_controller_;
  }

  QString display = QString::fromStdString(selected);
  if (selected == "FollowPath") {
    display += " (Pure Pursuit)";
  }

  if (status_label_) {
    status_label_->setText(
      QString("<b>当前控制器：</b>%1").arg(display));
  }
  if (command_label_) {
    command_label_->setText(
      QString("<b>cmd_vel：</b>v = %1 m/s, ω = %2 rad/s")
      .arg(linear_velocity_.load(), 0, 'f', 3)
      .arg(angular_velocity_.load(), 0, 'f', 3));
  }
}

}  // namespace nav2_controller_lab_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_controller_lab_rviz::ControllerLabPanel, rviz_common::Panel)
