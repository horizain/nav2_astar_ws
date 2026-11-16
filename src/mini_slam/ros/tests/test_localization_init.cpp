// 定位初始化回归：
//  1) 启动时按 graph_path 自动加载图（stats 的“点云”>0）
//  2) /initialpose 在 localization 模式设置 map→odom TF
//  3) 不发布 scan，只验证自动加载 + initialpose + TF 链路
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include "mini_slam/serialization.hpp"

#include "../mini_slam_node.hpp"

using namespace mini_slam;

namespace {

using namespace std::chrono_literals;

LaserScan make_scan() {
  LaserScan scan;
  for (int i = 0; i < 40; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / 40.0;
    const double r = 2.5 + 0.8 * std::sin(3.0 * a);
    scan.points.push_back(Vec2{r * std::cos(a), r * std::sin(a)});
  }
  return scan;
}

builtin_interfaces::msg::Time fixed_stamp() {
  builtin_interfaces::msg::Time t;
  t.sec = 10;
  t.nanosec = 0;
  return t;
}

template <typename ClientT>
bool call_trigger(const typename ClientT::SharedPtr& client, std::string* message) {
  auto req = std::make_shared<typename ClientT::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(15s) != std::future_status::ready) return false;
  const auto res = future.get();
  if (message) *message = res->message;
  return res->success;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 临时目录带 PID：并行 ctest / 两个测试进程共用 /tmp 时不会互相删目录。
  const auto dir = std::filesystem::temp_directory_path() /
                   ("mini_slam_localization_init_" +
                    std::to_string(static_cast<long>(::getpid())));
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);  // 启动时清一次（防上次崩溃残留）
  std::filesystem::create_directories(dir, ec);
  const std::string graph_path = (dir / "map.mmpg").string();

  // ---------- core 先生成一张小图（5 帧，每帧 40 点）----------
  {
    Mapper mapper;
    const LaserScan scan = make_scan();
    for (int i = 0; i < 5; ++i) {
      const Pose2 pose{0.6 * i, 0.0, 0.0};
      mapper.process_scan(scan, pose);
    }
    std::string err;
    if (!mapper.save_graph(graph_path, &err)) {
      std::printf("[FAIL] 构图保存失败: %s\n", err.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  int rc = 0;
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("mode", "localization"),
        rclcpp::Parameter("graph_path", graph_path),
        rclcpp::Parameter("transform_publish_period", 0.02),
        rclcpp::Parameter("map_update_interval", 0.2),
    });
    auto node = std::make_shared<MiniSlamNode>(options);
    auto driver = std::make_shared<rclcpp::Node>("localization_init_driver");

    auto odom_pub = driver->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    auto init_pub =
        driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);
    auto stats_cli =
        driver->create_client<std_srvs::srv::Trigger>("get_stats");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(driver->get_clock());
    tf_buffer->setUsingDedicatedThread(true);  // 允许带 timeout 的 lookup
    tf2_ros::TransformListener tf_listener(*tf_buffer, driver,
                                           /*SpinThread=*/false);

    rclcpp::executors::MultiThreadedExecutor node_exec(
        rclcpp::ExecutorOptions(), 4);
    node_exec.add_node(node);
    rclcpp::executors::SingleThreadedExecutor driver_exec;
    driver_exec.add_node(driver);
    std::thread node_spin([&] { node_exec.spin(); });
    std::thread driver_spin([&] { driver_exec.spin(); });

    if (!stats_cli->wait_for_service(15s)) {
      std::puts("[FAIL] get_stats 服务未就绪");
      rc = 1;
    } else {
      // ---------- 自动加载：点云数 > 0 ----------
      std::string msg;
      if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(stats_cli,
                                                                &msg)) {
        std::puts("[FAIL] get_stats 失败");
        rc = 1;
      } else {
        const auto pos = msg.find("点云=");
        const long pts =
            pos == std::string::npos ? 0 : std::stol(msg.substr(pos + 7));
        std::printf("  自动加载点云数: %ld\n", pts);
        if (pts <= 0) {
          std::puts("[FAIL] graph_path 未被自动加载（点云为 0）");
          rc = 1;
        }
      }
    }

    if (rc == 0) {
      // ---------- 发布一帧 odom，再发 /initialpose ----------
      const Pose2 odom_pose{1.0, 2.0, 0.3};
      const Pose2 map_pose{2.0, 3.0, 0.5};
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = fixed_stamp();
      odom.header.frame_id = "odom";
      odom.pose.pose.position.x = odom_pose.x;
      odom.pose.pose.position.y = odom_pose.y;
      odom.pose.pose.orientation.z = std::sin(odom_pose.theta / 2.0);
      odom.pose.pose.orientation.w = std::cos(odom_pose.theta / 2.0);
      odom_pub->publish(odom);
      std::this_thread::sleep_for(200ms);

      geometry_msgs::msg::PoseWithCovarianceStamped init;
      init.header.stamp = fixed_stamp();
      init.header.frame_id = "map";
      init.pose.pose.position.x = map_pose.x;
      init.pose.pose.position.y = map_pose.y;
      init.pose.pose.orientation.z = std::sin(map_pose.theta / 2.0);
      init.pose.pose.orientation.w = std::cos(map_pose.theta / 2.0);
      init_pub->publish(init);

      // ---------- 等 map→odom TF ----------
      const Pose2 expected = compose(map_pose, inverse(odom_pose));
      bool got = false;
      Pose2 actual;
      for (int i = 0; i < 40 && !got; ++i) {
        try {
          const auto tf = tf_buffer->lookupTransform(
              "map", "odom", rclcpp::Time(0),
              rclcpp::Duration::from_seconds(0.1));
          actual = Pose2{tf.transform.translation.x,
                         tf.transform.translation.y,
                         std::atan2(2.0 * (tf.transform.rotation.w *
                                               tf.transform.rotation.z +
                                           tf.transform.rotation.x *
                                               tf.transform.rotation.y),
                                    1.0 - 2.0 * (tf.transform.rotation.y *
                                                     tf.transform.rotation.y +
                                                 tf.transform.rotation.z *
                                                     tf.transform.rotation.z))};
          got = true;
        } catch (const tf2::TransformException&) {
          std::this_thread::sleep_for(100ms);
        }
      }
      if (!got) {
        std::puts("[FAIL] 未收到 map→odom TF");
        rc = 1;
      } else {
        const double pos_err = distance_xy(actual, expected);
        const double ang_err = std::abs(shortest_angle_diff(
            actual.theta, expected.theta));
        std::printf("  initialpose TF 误差: pos=%.6f ang=%.6f\n", pos_err,
                    ang_err);
        if (pos_err > 1e-4 || ang_err > 1e-4) {
          std::puts("[FAIL] initialpose 后 map→odom TF 与期望不符");
          rc = 1;
        }
      }
    }

    node_exec.cancel();
    driver_exec.cancel();
    node_spin.join();
    driver_spin.join();
    node_exec.remove_node(node);
    driver_exec.remove_node(driver);
  }

  std::filesystem::remove_all(dir, ec);
  rclcpp::shutdown();
  if (rc == 0) std::puts("[PASS] test_localization_init");
  return rc;
}
