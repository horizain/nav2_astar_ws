// Copyright (c) 2026
// Controller Lab RViz Panel 实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab_rviz/controller_lab_panel.hpp"

#include <utility>
#include <algorithm>

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

  for (const auto & topic : {"cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel_cbf", "cmd_vel"}) {
    const std::string name(topic);
    chain_subs_.push_back(node_->create_subscription<geometry_msgs::msg::Twist>(
      name, 10, [this, name](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        chain_[name] = {msg->linear.x, msg->angular.z, std::chrono::steady_clock::now()};
      }));
  }
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      chain_["odom"] = {msg->twist.twist.linear.x, msg->twist.twist.angular.z,
        std::chrono::steady_clock::now()};
    });
  diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "controller_lab/controller_diagnostics", 10,
    [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (const auto & status : msg->status) {
        if (status.name.rfind("controller_lab/", 0) != 0) {continue;}
        diagnostic_values_.clear();
        for (const auto & item : status.values) {diagnostic_values_[item.key] = item.value;}
        diagnostic_detail_ = status.message;
        diagnostic_received_ = std::chrono::steady_clock::now();
      }
    });
  rclcpp::QoS action_qos(10);
  action_qos.transient_local().reliable();
  action_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status", action_qos,
    [this](action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      if (msg->status_list.empty()) {return;}
      const auto latest = std::max_element(msg->status_list.begin(), msg->status_list.end(),
        [](const auto & a, const auto & b) {
          return rclcpp::Time(a.goal_info.stamp) < rclcpp::Time(b.goal_info.stamp);
        });
      const std::vector<std::string> names{
        "未知", "已接受", "执行中", "取消中", "成功", "已取消", "失败"};
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto index = static_cast<size_t>(latest->status);
      action_info_ = index < names.size() ? names[index] : "未知状态";
      action_received_ = std::chrono::steady_clock::now();
    });
  stats_client_ = node_->create_client<std_srvs::srv::Trigger>("/get_stats");
  stats_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this]() {
      if (stats_pending_) {
        if (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stats_requested_).count() < 4.0) {return;}
        stats_client_->remove_pending_request(stats_request_id_);
        stats_pending_ = false;
      }
      if (!stats_client_->service_is_ready()) {return;}
      stats_pending_ = true;
      stats_requested_ = std::chrono::steady_clock::now();
      const auto pending = stats_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          stats_pending_ = false;
          try {
            const auto response = future.get();
            std::lock_guard<std::mutex> lock(state_mutex_);
            mode_info_ = response->success ? response->message : "mini_slam统计读取失败";
            mode_received_ = std::chrono::steady_clock::now();
          } catch (const std::exception &) {
            // A stale timestamp is shown explicitly; no mode is inferred from missing data.
          }
        });
      stats_request_id_ = pending.request_id;
    });

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
  auto * outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  auto * contents = new QWidget(scroll);
  scroll->setWidget(contents);
  outer_layout->addWidget(scroll);
  auto * main_layout = new QVBoxLayout(contents);
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

  mode_label_ = new QLabel();
  mode_label_->setWordWrap(true);
  main_layout->addWidget(mode_label_);
  diagnostic_label_ = new QLabel();
  diagnostic_label_->setWordWrap(true);
  main_layout->addWidget(diagnostic_label_);
  chain_label_ = new QLabel();
  chain_label_->setWordWrap(true);
  main_layout->addWidget(chain_label_);
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
    "选择实例：FollowPath / PID / Stanley / DWB / MPPI / MPC<br>蓝线：底盘指令匀速外推<br>青线：最近MPC选中预测序列，过期消失");
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
      QString("<b>选择请求：</b>%1").arg(display));
  }
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto fresh = [&](auto stamp, double age) {
      return stamp.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - stamp).count() < age;
    };
  if (mode_label_) {
    const auto text = fresh(mode_received_, 6.0) ? mode_info_ : "mini_slam统计暂无或已过期";
    mode_label_->setText(QString("<b>建图/定位：</b>%1<br><b>最近导航目标：</b>%2")
      .arg(QString::fromStdString(text).toHtmlEscaped())
      .arg(QString::fromStdString(action_info_).toHtmlEscaped()));
  }
  if (diagnostic_label_) {
    if (fresh(diagnostic_received_, 2.0)) {
      const auto controller = QString::fromStdString(diagnostic_values_["controller"]).toHtmlEscaped();
      const auto stage = QString::fromStdString(diagnostic_values_["stage"]).toHtmlEscaped();
      diagnostic_label_->setText(QString(
        "<b>最近实际计算：</b>%1 / %2<br>位置误差 %3 m，角差 %4 rad<br>"
        "本次计算 %5 ms（含TF/适配，不含服务器等待）<br>%6")
        .arg(controller).arg(stage)
        .arg(QString::fromStdString(diagnostic_values_["goal_error_m"]))
        .arg(QString::fromStdString(diagnostic_values_["goal_yaw_error_rad"]))
        .arg(QString::fromStdString(diagnostic_values_["compute_ms"]))
        .arg(QString::fromStdString(diagnostic_detail_).toHtmlEscaped()));
    } else {
      diagnostic_label_->setText("控制诊断暂无或已过期：空闲时正常，不能据此判断求解无解");
    }
  }
  if (chain_label_) {
    QString chain_text;
    for (const auto & name : {"cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel_cbf", "cmd_vel", "odom"}) {
      auto entry = chain_.find(name);
      chain_text += QString("%1: ").arg(name);
      if (entry != chain_.end() && fresh(entry->second.received, 1.0)) {
        chain_text += QString("v=%1, w=%2<br>")
          .arg(entry->second.v, 0, 'f', 3).arg(entry->second.w, 0, 'f', 3);
      } else {chain_text += "暂无/过期<br>";}
    }
    auto moving = [&](const char * name) {
        auto item = chain_.find(name);
        return item != chain_.end() && fresh(item->second.received, 1.0) &&
          (std::abs(item->second.v) > 0.03 || std::abs(item->second.w) > 0.06);
      };
    auto current = [&](const char * name) {
        auto item = chain_.find(name);
        return item != chain_.end() && fresh(item->second.received, 1.0);
      };
    if (moving("cmd_vel_smoothed") && current("cmd_vel_cbf") && !moving("cmd_vel_cbf")) {
      chain_text += "观测：CBF前有速度，CBF后接近零。";
    } else if (moving("cmd_vel_cbf") && current("cmd_vel") && !moving("cmd_vel")) {
      chain_text += "观测：安全监控前有速度，底盘指令接近零。";
    } else if (moving("cmd_vel") && current("odom") && !moving("odom")) {
      chain_text += "观测：已下发运动指令，里程计仍接近静止。";
    }
    chain_label_->setText(chain_text);
  }
  if (command_label_) {
    const auto last = chain_.find("cmd_vel");
    if (last == chain_.end() || !fresh(last->second.received, 1.0)) {
      command_label_->setText("底盘指令暂无或已过期");
      return;
    }
    command_label_->setText(
      QString("<b>cmd_vel：</b>v = %1 m/s, ω = %2 rad/s")
      .arg(linear_velocity_.load(), 0, 'f', 3)
      .arg(angular_velocity_.load(), 0, 'f', 3));
  }
}

}  // namespace nav2_controller_lab_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_controller_lab_rviz::ControllerLabPanel, rviz_common::Panel)
