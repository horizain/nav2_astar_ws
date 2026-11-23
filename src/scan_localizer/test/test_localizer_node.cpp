// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "scan_localizer/localizer_node.hpp"
#include "scan_localizer/map_io.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

using lifecycle_msgs::msg::State;

const sloc::OccupancyGrid & sharedWarehouse()
{
  static const sloc::OccupancyGrid kGrid = sloc::sim::make_warehouse(0.05);
  return kGrid;
}

nav_msgs::msg::OccupancyGrid toRosMap(
  const sloc::OccupancyGrid & g, const rclcpp::Time & stamp,
  const std::string & frame_id = "map")
{
  nav_msgs::msg::OccupancyGrid m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.info.resolution = static_cast<float>(g.resolution);
  m.info.width = static_cast<std::uint32_t>(g.width);
  m.info.height = static_cast<std::uint32_t>(g.height);
  m.info.origin.position.x = g.origin_x;
  m.info.origin.position.y = g.origin_y;
  m.info.origin.orientation.w = 1.0;
  m.data = g.cells;
  return m;
}

sensor_msgs::msg::LaserScan makeRosScan(
  const sloc::OccupancyGrid & g, const sloc::Pose2 & pose,
  const rclcpp::Time & stamp, const std::string & frame_id = "base_link",
  bool invert_roll_pi = false, bool inject_nan_inf = false)
{
  sloc::sim::ScanSimConfig cfg;
  cfg.beams = 360;
  cfg.range_max = 12.0;
  std::mt19937 rng(42);
  const sloc::Scan s = sloc::sim::make_scan(g, pose, cfg, rng);

  sensor_msgs::msg::LaserScan ls;
  ls.header.stamp = stamp;
  ls.header.frame_id = frame_id;
  ls.angle_min = -static_cast<float>(sloc::kPi);
  ls.angle_max = static_cast<float>(sloc::kPi);
  ls.angle_increment = static_cast<float>(sloc::kTwoPi / 360.0);
  ls.range_min = 0.10f;
  ls.range_max = 12.0f;
  ls.ranges.assign(360, std::numeric_limits<float>::infinity());

  for (const auto & p : s.points) {
    // 倒装雷达(roll=pi)：激光系 y 轴反向，物理束角 a_laser = -a_base
    const double py = invert_roll_pi ? -p.y() : p.y();
    double a = std::atan2(py, p.x());
    if (a < -sloc::kPi) {a += sloc::kTwoPi;}
    const int idx = static_cast<int>(
      std::lround((a + sloc::kPi) / static_cast<double>(ls.angle_increment)));
    if (idx >= 0 && idx < 360) {
      ls.ranges[static_cast<std::size_t>(idx)] =
        static_cast<float>(std::hypot(p.x(), py));
    }
  }

  if (inject_nan_inf) {
    ls.ranges[0] = std::numeric_limits<float>::quiet_NaN();
    ls.ranges[1] = std::numeric_limits<float>::infinity();
    ls.ranges[2] = -std::numeric_limits<float>::infinity();
    ls.ranges[3] = -1.5f;
  }
  return ls;
}

nav_msgs::msg::Odometry makeRosOdom(
  const sloc::Pose2 & p, const rclcpp::Time & stamp,
  const std::string & odom_frame = "odom",
  const std::string & base_frame = "base_link")
{
  nav_msgs::msg::Odometry om;
  om.header.stamp = stamp;
  om.header.frame_id = odom_frame;
  om.child_frame_id = base_frame;
  om.pose.pose.position.x = p.x;
  om.pose.pose.position.y = p.y;
  om.pose.pose.orientation.z = std::sin(0.5 * p.theta);
  om.pose.pose.orientation.w = std::cos(0.5 * p.theta);
  return om;
}

geometry_msgs::msg::PoseWithCovarianceStamped makeInitialPoseMsg(
  const sloc::Pose2 & p, const rclcpp::Time & stamp,
  const std::string & frame_id = "map",
  double var_xy = 0.04, double var_yaw = 0.02)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = p.x;
  msg.pose.pose.position.y = p.y;
  msg.pose.pose.orientation.z = std::sin(0.5 * p.theta);
  msg.pose.pose.orientation.w = std::cos(0.5 * p.theta);
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = var_xy;
  msg.pose.covariance[7] = var_xy;
  msg.pose.covariance[35] = var_yaw;
  return msg;
}

bool spinUntil(
  rclcpp::Executor & exec, const std::function<bool()> & pred,
  int timeout_ms = 3000)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(10));
    if (pred()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

std::string findKv(
  const diagnostic_msgs::msg::DiagnosticArray & da, const std::string & key)
{
  if (da.status.empty()) {return "";}
  for (const auto & kv : da.status[0].values) {
    if (kv.key == key) {return kv.value;}
  }
  return "";
}

class LocalizerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// 1) 完整生命周期转换 + 非法参数/map_yaml 返回 FAILURE + cleanup 后安全
TEST_F(LocalizerNodeTest, LifecycleTransitionsAndCleanupSafety)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/t_lifecycle"});
  opts.append_parameter_override("tf_publish_rate", 0.0);
  auto node = std::make_shared<sloc::LocalizerNode>(opts);
  auto helper = std::make_shared<rclcpp::Node>("helper_lifecycle", "/t_lifecycle");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // 非法 matcher 导致 configure() 失败并保持 UNCONFIGURED
  node->set_parameter(rclcpp::Parameter("matcher", "invalid_matcher_xyz"));
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // 恢复合法 matcher，但设置不存在的 map_yaml 仍应失败
  node->set_parameter(rclcpp::Parameter("matcher", "composed"));
  node->set_parameter(
    rclcpp::Parameter("map_yaml", "/nonexistent/path/to/map.yaml"));
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // 清空 map_yaml -> configure -> INACTIVE -> activate -> ACTIVE
  node->set_parameter(rclcpp::Parameter("map_yaml", ""));
  node->configure();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  // INACTIVE 下发地图和扫描不应产出位姿或 TF
  std::atomic<int> pose_count{0};
  std::atomic<int> tf_count{0};
  auto pose_sub =
    helper->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "localizer/pose", 10,
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
      ++pose_count;
    });
  auto tf_sub = helper->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 50,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        if (t.header.frame_id == "map" && t.child_frame_id == "odom") {
          ++tf_count;
        }
      }
    });
  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  auto init_pub =
    helper->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
  auto map_path_pub = helper->create_publisher<std_msgs::msg::String>(
    "localizer/map_path", 10);

  const sloc::Pose2 gt(2.0, 13.5, 0.0);
  map_pub->publish(toRosMap(sharedWarehouse(), helper->now()));
  init_pub->publish(makeInitialPoseMsg(gt, helper->now()));
  spinUntil(exec, []() {return false;}, 150);
  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, helper->now()));
  spinUntil(exec, []() {return false;}, 150);
  EXPECT_EQ(pose_count.load(), 0);
  EXPECT_EQ(tf_count.load(), 0);

  // activate -> ACTIVE -> 应正常输出位姿与 TF
  node->activate();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, helper->now()));
  ASSERT_TRUE(spinUntil(exec, [&]() {return pose_count.load() > 0;}));
  EXPECT_GT(tf_count.load(), 0);

  // deactivate -> INACTIVE -> 停止输出位姿与 TF
  node->deactivate();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  const int pose_before = pose_count.load();
  const int tf_before = tf_count.load();
  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, helper->now()));
  spinUntil(exec, []() {return false;}, 150);
  EXPECT_EQ(pose_count.load(), pose_before);
  EXPECT_EQ(tf_count.load(), tf_before);

  // cleanup -> UNCONFIGURED -> 发送 map_path 不崩溃
  node->cleanup();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  std_msgs::msg::String mp;
  mp.data = "/tmp/does_not_exist_after_cleanup.yaml";
  map_path_pub->publish(mp);
  spinUntil(exec, []() {return false;}, 100);

  // 重新 configure -> INACTIVE 成功
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  node->shutdown();
}

// 2) 6x6 协方差槽位 + NaN/Inf 扫描过滤 + 少点丢帧 + /initialpose + nomotion_update
TEST_F(LocalizerNodeTest, CovarianceSlotsNanFilteringAndNoMotionUpdate)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/t_cov"});
  opts.append_parameter_override("tf_publish_rate", 0.0);
  opts.append_parameter_override("map_frame", "map_cov");
  opts.append_parameter_override("odom_frame", "odom_cov");
  opts.append_parameter_override("base_frame", "base_cov");
  auto node = std::make_shared<sloc::LocalizerNode>(opts);
  auto helper = std::make_shared<rclcpp::Node>("helper_cov", "/t_cov");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  node->configure();
  node->activate();

  std::mutex mtx;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose;
  std::atomic<int> pose_count{0};
  std::atomic<int> hyp_pub_count{0};

  auto pose_sub =
    helper->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "localizer/pose", 10,
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      last_pose = *msg;
      ++pose_count;
    });
  auto hyp_pub_sub = helper->create_subscription<geometry_msgs::msg::PoseArray>(
    "localizer/hypotheses", 10,
    [&](const geometry_msgs::msg::PoseArray::SharedPtr) {
      ++hyp_pub_count;
    });

  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());
  auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  auto init_pub =
    helper->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
  auto nomotion_cli = helper->create_client<std_srvs::srv::Empty>(
    "localizer/nomotion_update");

  const sloc::Pose2 gt(2.0, 13.5, 0.0);
  const auto t0 = helper->now();
  map_pub->publish(toRosMap(sharedWarehouse(), t0, "map_cov"));
  odom_pub->publish(makeRosOdom(gt, t0, "odom_cov", "base_cov"));
  spinUntil(exec, []() {return false;}, 100);

  // 错误 frame_id 的 initialpose 应被忽略；正确 frame_id 才生效
  init_pub->publish(makeInitialPoseMsg({99.0, 99.0, 1.0}, t0, "wrong_frame"));
  init_pub->publish(makeInitialPoseMsg(gt, t0, "map_cov", 0.05, 0.03));
  spinUntil(exec, []() {return false;}, 100);

  // 含 NaN / ±Inf / 负值的扫描应正常过滤并收敛
  scan_pub->publish(
    makeRosScan(sharedWarehouse(), gt, t0, "base_cov", false, true));
  ASSERT_TRUE(spinUntil(exec, [&]() {return pose_count.load() >= 1;}));
  EXPECT_GT(hyp_pub_count.load(), 0);

  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_NEAR(last_pose.pose.pose.position.x, gt.x, 0.15);
    EXPECT_NEAR(last_pose.pose.pose.position.y, gt.y, 0.15);
    // 6x6 协方差槽位严格验证
    EXPECT_GT(last_pose.pose.covariance[0], 0.0);
    EXPECT_GT(last_pose.pose.covariance[7], 0.0);
    EXPECT_GT(last_pose.pose.covariance[35], 0.0);
    EXPECT_DOUBLE_EQ(last_pose.pose.covariance[14], 0.0);
    EXPECT_DOUBLE_EQ(
      last_pose.pose.covariance[5], last_pose.pose.covariance[30]);
    EXPECT_DOUBLE_EQ(
      last_pose.pose.covariance[11], last_pose.pose.covariance[31]);
  }

  // 点数不足 min_scan_points 的扫描应被安全丢弃
  const int before_sparse = pose_count.load();
  sensor_msgs::msg::LaserScan sparse;
  sparse.header.stamp = helper->now();
  sparse.header.frame_id = "base_cov";
  sparse.angle_min = -1.0f;
  sparse.angle_max = 1.0f;
  sparse.angle_increment = 0.5f;
  sparse.range_min = 0.1f;
  sparse.range_max = 10.0f;
  sparse.ranges = {1.0f, 1.0f, std::numeric_limits<float>::quiet_NaN()};
  scan_pub->publish(sparse);
  spinUntil(exec, []() {return false;}, 120);
  EXPECT_EQ(pose_count.load(), before_sparse);

  // nomotion_update 服务应触发一次新的位姿发布
  ASSERT_TRUE(nomotion_cli->wait_for_service(std::chrono::seconds(2)));
  const int before_nomotion = pose_count.load();
  auto fut = nomotion_cli->async_send_request(
    std::make_shared<std_srvs::srv::Empty::Request>());
  ASSERT_TRUE(spinUntil(exec, [&]() {
      return fut.wait_for(std::chrono::milliseconds(0)) ==
             std::future_status::ready &&
             pose_count.load() > before_nomotion;
  }));

  node->deactivate();
  node->cleanup();
}

// 3) Best-effort IMU 联通 + 负 dt 丢弃 + cleanup->reconfigure 无 imu_yaw 跳变
TEST_F(LocalizerNodeTest, ImuSensorDataQosNegativeDtAndCleanupReset)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/t_imu"});
  opts.append_parameter_override("use_imu", true);
  opts.append_parameter_override("tf_publish_rate", 0.0);
  opts.append_parameter_override("map_frame", "map_imu");
  opts.append_parameter_override("odom_frame", "odom_imu");
  opts.append_parameter_override("base_frame", "base_imu");
  auto node = std::make_shared<sloc::LocalizerNode>(opts);
  auto helper = std::make_shared<rclcpp::Node>("helper_imu", "/t_imu");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  node->configure();
  node->activate();

  auto imu_pub = helper->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS());
  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());
  auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  auto init_pub =
    helper->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);

  std::mutex mtx;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose;
  std::atomic<int> pose_count{0};
  auto pose_sub =
    helper->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "localizer/pose", 10,
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      last_pose = *msg;
      ++pose_count;
    });

  // 确认 best-effort IMU 发布者可与 LocalizerNode 匹配联通
  ASSERT_TRUE(spinUntil(exec, [&]() {
      return imu_pub->get_subscription_count() >= 1;
  }));

  const sloc::Pose2 gt(2.0, 13.5, 0.0);
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  map_pub->publish(toRosMap(sharedWarehouse(), t0, "map_imu"));
  odom_pub->publish(makeRosOdom(gt, t0, "odom_imu", "base_imu"));
  init_pub->publish(makeInitialPoseMsg(gt, t0, "map_imu"));
  spinUntil(exec, []() {return false;}, 100);

  // 发送正向 IMU 积累大偏航角，再发送负 dt（时间戳回退）应被丢弃而非夹紧为 +1e-4
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = rclcpp::Time(100, 0, RCL_ROS_TIME);
  imu_msg.angular_velocity.z = 1.0;
  imu_pub->publish(imu_msg);
  imu_msg.header.stamp = rclcpp::Time(100, 200000000, RCL_ROS_TIME);
  imu_msg.angular_velocity.z = 5.0;  // 积分 +1.0 rad
  imu_pub->publish(imu_msg);
  // 负 dt：时间戳倒退，带巨大角速度，必须丢弃
  imu_msg.header.stamp = rclcpp::Time(99, 500000000, RCL_ROS_TIME);
  imu_msg.angular_velocity.z = 1000.0;
  imu_pub->publish(imu_msg);
  spinUntil(exec, []() {return false;}, 100);

  // cleanup -> reconfigure -> activate，验证 imu_yaw_ 已被清零无航向跳变
  node->deactivate();
  node->cleanup();
  node->configure();
  node->activate();

  map_pub->publish(toRosMap(sharedWarehouse(), t0, "map_imu"));
  odom_pub->publish(makeRosOdom(gt, t0, "odom_imu", "base_imu"));
  init_pub->publish(makeInitialPoseMsg(gt, t0, "map_imu"));
  spinUntil(exec, []() {return false;}, 100);

  // 重启后发两帧零角速度 IMU + 一帧扫描：航向应稳定在 0.0 rad 附近，无历史残留跳变
  imu_msg.header.stamp = rclcpp::Time(200, 0, RCL_ROS_TIME);
  imu_msg.angular_velocity.z = 0.0;
  imu_pub->publish(imu_msg);
  imu_msg.header.stamp = rclcpp::Time(200, 100000000, RCL_ROS_TIME);
  imu_msg.angular_velocity.z = 0.0;
  imu_pub->publish(imu_msg);
  spinUntil(exec, []() {return false;}, 100);

  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, t0, "base_imu"));
  ASSERT_TRUE(spinUntil(exec, [&]() {return pose_count.load() >= 1;}));
  {
    std::lock_guard<std::mutex> lk(mtx);
    const auto & q = last_pose.pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    EXPECT_NEAR(yaw, 0.0, 0.15);
  }

  node->deactivate();
  node->cleanup();
}

// 4) 倒装雷达 TF（roll = pi）正确翻转激光束无镜像误差
TEST_F(LocalizerNodeTest, InvertedLidarTfRollPi)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/t_inv"});
  opts.append_parameter_override("tf_publish_rate", 0.0);
  opts.append_parameter_override("map_frame", "map_inv");
  opts.append_parameter_override("odom_frame", "odom_inv");
  opts.append_parameter_override("base_frame", "base_inv");
  auto node = std::make_shared<sloc::LocalizerNode>(opts);
  auto helper = std::make_shared<rclcpp::Node>("helper_inv", "/t_inv");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  node->configure();
  node->activate();

  // 发布静态 TF: base_inv -> laser_inv，绕 X 轴旋转 pi（roll=pi，倒装雷达）
  tf2_ros::StaticTransformBroadcaster static_tf(helper);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = helper->now();
  tf.header.frame_id = "base_inv";
  tf.child_frame_id = "laser_inv";
  tf.transform.rotation.x = 1.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 0.0;
  static_tf.sendTransform(tf);

  std::mutex mtx;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose;
  diagnostic_msgs::msg::DiagnosticArray last_diag;
  std::atomic<int> pose_count{0};
  auto pose_sub =
    helper->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "localizer/pose", 10,
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      last_pose = *msg;
      ++pose_count;
    });
  auto diag_sub =
    helper->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "localizer/status", 10,
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      last_diag = *msg;
    });

  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());
  auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  auto init_pub =
    helper->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);

  // 选取非对称位置，镜像错误会导致低分拒绝
  const sloc::Pose2 gt(2.0, 13.5, 0.3);
  const auto t0 = helper->now();
  map_pub->publish(toRosMap(sharedWarehouse(), t0, "map_inv"));
  odom_pub->publish(makeRosOdom(gt, t0, "odom_inv", "base_inv"));
  init_pub->publish(makeInitialPoseMsg(
      {gt.x + 0.05, gt.y - 0.04, gt.theta + 0.03}, t0, "map_inv"));
  spinUntil(exec, []() {return false;}, 200);

  // 发送倒装雷达扫描（frame_id="laser_inv"，y 反向）
  scan_pub->publish(
    makeRosScan(sharedWarehouse(), gt, t0, "laser_inv", true, false));
  ASSERT_TRUE(spinUntil(exec, [&]() {return pose_count.load() >= 1;}));

  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_NEAR(last_pose.pose.pose.position.x, gt.x, 0.12);
    EXPECT_NEAR(last_pose.pose.pose.position.y, gt.y, 0.12);
    EXPECT_EQ(findKv(last_diag, "accepted"), "1");
  }

  node->deactivate();
  node->cleanup();
}

// 5) TF 保活在扫描断流期间持续发布 + 动态参数校验
TEST_F(LocalizerNodeTest, TfKeepaliveAndDynamicParameters)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/t_tf"});
  opts.append_parameter_override("tf_publish_rate", 20.0);
  opts.append_parameter_override("map_frame", "map_keepalive");
  opts.append_parameter_override("odom_frame", "odom_keepalive");
  opts.append_parameter_override("base_frame", "base_keepalive");
  auto node = std::make_shared<sloc::LocalizerNode>(opts);
  auto helper = std::make_shared<rclcpp::Node>("helper_tf", "/t_tf");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  node->configure();
  node->activate();

  // 动态参数校验：合法接受，非法拒绝且给出原因
  auto ok_res = node->set_parameters({
      rclcpp::Parameter("min_score", 0.40),
      rclcpp::Parameter("odom_trans_var_per_m", 5.0e-4),
      rclcpp::Parameter("tf_broadcast", true),
  });
  ASSERT_EQ(ok_res.size(), 3u);
  for (const auto & r : ok_res) {
    EXPECT_TRUE(r.successful) << r.reason;
  }

  auto bad_score = node->set_parameter(rclcpp::Parameter("min_score", -0.2));
  EXPECT_FALSE(bad_score.successful);
  EXPECT_FALSE(bad_score.reason.empty());

  auto bad_matcher = node->set_parameter(rclcpp::Parameter("matcher", "nope"));
  EXPECT_FALSE(bad_matcher.successful);
  EXPECT_FALSE(bad_matcher.reason.empty());

  auto bad_gate = node->set_parameter(rclcpp::Parameter("mahal_gate", 0.0));
  EXPECT_FALSE(bad_gate.successful);
  EXPECT_FALSE(bad_gate.reason.empty());

  std::atomic<int> tf_count{0};
  auto tf_sub = helper->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 50,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        if (t.header.frame_id == "map_keepalive" &&
        t.child_frame_id == "odom_keepalive")
        {
          ++tf_count;
        }
      }
    });

  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());
  auto init_pub =
    helper->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);

  const sloc::Pose2 gt(2.0, 13.5, 0.0);
  const auto t0 = helper->now();
  map_pub->publish(toRosMap(sharedWarehouse(), t0, "map_keepalive"));
  init_pub->publish(makeInitialPoseMsg(gt, t0, "map_keepalive"));

  // 不发任何扫描，TF 保活定时器也应持续发布多次 map_keepalive -> odom_keepalive
  ASSERT_TRUE(spinUntil(exec, [&]() {return tf_count.load() >= 3;}, 2000));

  node->deactivate();
  node->cleanup();
}

// 6) set_fine_mode + global_relocalize + /map 动态重载 + map_path 热切 + global_hypotheses
TEST_F(LocalizerNodeTest, ServicesMapReloadMapPathAndHypotheses)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/t_srv"});
  opts.append_parameter_override("tf_publish_rate", 0.0);
  opts.append_parameter_override("map_frame", "map_srv");
  opts.append_parameter_override("odom_frame", "odom_srv");
  opts.append_parameter_override("base_frame", "base_srv");
  auto node = std::make_shared<sloc::LocalizerNode>(opts);
  auto helper = std::make_shared<rclcpp::Node>("helper_srv", "/t_srv");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  node->configure();
  node->activate();

  std::mutex mtx;
  diagnostic_msgs::msg::DiagnosticArray last_diag;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose;
  std::atomic<int> diag_count{0};

  auto diag_sub =
    helper->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "localizer/status", 10,
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      last_diag = *msg;
      ++diag_count;
    });
  auto pose_sub =
    helper->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "localizer/pose", 10,
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      last_pose = *msg;
    });

  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());
  auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  auto init_pub =
    helper->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
  auto hyp_pub = helper->create_publisher<geometry_msgs::msg::PoseArray>(
    "localizer/global_hypotheses", 10);
  auto map_path_pub = helper->create_publisher<std_msgs::msg::String>(
    "localizer/map_path", 10);

  auto fine_cli = helper->create_client<std_srvs::srv::SetBool>(
    "localizer/set_fine_mode");
  auto global_cli = helper->create_client<std_srvs::srv::Trigger>(
    "localizer/global_relocalize");
  ASSERT_TRUE(fine_cli->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(global_cli->wait_for_service(std::chrono::seconds(2)));

  // 无地图时 global_relocalize 应返回 false
  {
    auto fut = global_cli->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_TRUE(spinUntil(exec, [&]() {
        return fut.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::ready;
    }));
    EXPECT_FALSE(fut.get()->success);
  }

  // 发送首张地图 + 初始位姿 + 扫描
  const sloc::Pose2 gt(2.0, 13.5, 0.0);
  const auto t0 = helper->now();
  map_pub->publish(toRosMap(sharedWarehouse(), t0, "map_srv"));
  odom_pub->publish(makeRosOdom(gt, t0, "odom_srv", "base_srv"));
  init_pub->publish(makeInitialPoseMsg(gt, t0, "map_srv"));
  spinUntil(exec, []() {return false;}, 120);

  // 开启 fine_mode
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto fut = fine_cli->async_send_request(req);
    ASSERT_TRUE(spinUntil(exec, [&]() {
        return fut.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::ready;
    }));
    EXPECT_TRUE(fut.get()->success);
  }

  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, t0, "base_srv"));
  ASSERT_TRUE(spinUntil(exec, [&]() {return diag_count.load() >= 1;}));
  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_EQ(findKv(last_diag, "fine_mode"), "1");
    EXPECT_EQ(last_diag.status[0].level,
      diagnostic_msgs::msg::DiagnosticStatus::OK);
  }

  // 触发 global_relocalize（第一次成功，已在 GLOBAL 时第二次返回 false）
  {
    auto fut1 = global_cli->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_TRUE(spinUntil(exec, [&]() {
        return fut1.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::ready;
    }));
    EXPECT_TRUE(fut1.get()->success);

    auto fut2 = global_cli->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_TRUE(spinUntil(exec, [&]() {
        return fut2.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::ready;
    }));
    EXPECT_FALSE(fut2.get()->success);
  }

  // 注入外部假设（接近真值），下一帧激光应直接验证通过并收敛到真值
  geometry_msgs::msg::PoseArray hyps;
  hyps.header.stamp = helper->now();
  hyps.header.frame_id = "map_srv";
  geometry_msgs::msg::Pose hp;
  hp.position.x = gt.x + 0.08;
  hp.position.y = gt.y - 0.06;
  hp.orientation.w = 1.0;
  hyps.poses.push_back(hp);
  hyp_pub->publish(hyps);
  spinUntil(exec, []() {return false;}, 100);

  const int c_before_hyp = diag_count.load();
  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, helper->now(), "base_srv"));
  ASSERT_TRUE(spinUntil(exec, [&]() {return diag_count.load() > c_before_hyp;}));
  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_NEAR(last_pose.pose.pose.position.x, gt.x, 0.15);
    EXPECT_NEAR(last_pose.pose.pose.position.y, gt.y, 0.15);
  }

  // 动态 /map 二次发布重载（map_keep_pose=true -> 保持 TRACKING）
  map_pub->publish(toRosMap(sharedWarehouse(), helper->now(), "map_srv"));
  spinUntil(exec, []() {return false;}, 150);
  const int c_before_reload = diag_count.load();
  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, helper->now(), "base_srv"));
  ASSERT_TRUE(spinUntil(exec, [&]() {return diag_count.load() > c_before_reload;}));
  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_EQ(last_diag.status[0].message, "TRACKING");
  }

  // 导出临时地图并用 localizer/map_path 热切测试 map_keep_pose=true & false
  const auto tmp_base =
    (std::filesystem::temp_directory_path() / "sloc_test_node_map").string();
  std::string err;
  ASSERT_TRUE(sloc::save_map_yaml(sharedWarehouse(), tmp_base, err)) << err;
  const std::string yaml_file = tmp_base + ".yaml";

  // map_keep_pose=true 热切：保持 TRACKING
  node->set_parameter(rclcpp::Parameter("map_keep_pose", true));
  std_msgs::msg::String mp_msg;
  mp_msg.data = yaml_file;
  map_path_pub->publish(mp_msg);
  spinUntil(exec, []() {return false;}, 150);
  const int c_before_keep = diag_count.load();
  scan_pub->publish(makeRosScan(sharedWarehouse(), gt, helper->now(), "base_srv"));
  ASSERT_TRUE(spinUntil(exec, [&]() {return diag_count.load() > c_before_keep;}));
  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_EQ(last_diag.status[0].message, "TRACKING");
  }

  // map_keep_pose=false 热切：自动进入 GLOBAL，此时再调 global_relocalize 应返回 false
  node->set_parameter(rclcpp::Parameter("map_keep_pose", false));
  map_path_pub->publish(mp_msg);
  spinUntil(exec, []() {return false;}, 150);
  {
    auto fut = global_cli->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_TRUE(spinUntil(exec, [&]() {
        return fut.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::ready;
    }));
    EXPECT_FALSE(fut.get()->success);
  }

  std::error_code ec;
  std::filesystem::remove(yaml_file, ec);
  std::filesystem::remove(tmp_base + ".pgm", ec);

  node->deactivate();
  node->cleanup();
}

}  // namespace
