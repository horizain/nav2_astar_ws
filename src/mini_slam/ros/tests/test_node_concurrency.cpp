// ROS 节点并发回归测试（Phase 1）：
//   worker 灌帧（odom+scan）与 map/TF timer、save/load/stats 服务同时运行，
//   反复替换/序列化图。测试目标是"不崩溃、服务不超时、图操作可完成"。
//
// 覆盖边界：这不是 TSan 证明。没有 TSAN 时无法证明零数据竞争；本测试验证
//   锁把并发路径串行化后，混合负载下节点仍能稳定响应（可检测出未加锁时
//   常见的迭代器失效/半替换图崩溃）。
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <unistd.h>

#include "../mini_slam_node.hpp"

namespace {

using namespace std::chrono_literals;

builtin_interfaces::msg::Time make_stamp(int i) {
  builtin_interfaces::msg::Time t;
  t.sec = i / 100;
  t.nanosec = static_cast<std::uint32_t>((i % 100) * 10'000'000);
  return t;
}

nav_msgs::msg::Odometry make_odom(int i) {
  nav_msgs::msg::Odometry m;
  m.header.stamp = make_stamp(i);
  m.header.frame_id = "odom";
  m.pose.pose.position.x = 0.06 * i;
  m.pose.pose.position.y = 0.01 * std::sin(0.05 * i);
  const double yaw = 0.01 * i;
  m.pose.pose.orientation.z = std::sin(yaw / 2.0);
  m.pose.pose.orientation.w = std::cos(yaw / 2.0);
  return m;
}

sensor_msgs::msg::LaserScan make_scan(int i) {
  sensor_msgs::msg::LaserScan m;
  m.header.stamp = make_stamp(i);
  m.header.frame_id = "laser";
  m.angle_min = -3.14159265f;
  m.angle_max = 3.14159265f;
  m.angle_increment = 6.2831853f / 60.0f;
  m.range_min = 0.05f;
  m.range_max = 20.0f;
  m.ranges.resize(60);
  for (int k = 0; k < 60; ++k) {
    // 半径 2~5m 的环形场景：给匹配器足够的结构
    m.ranges[static_cast<size_t>(k)] =
        2.0f + 3.0f * static_cast<float>((k * 7 + i / 10) % 13) / 12.0f;
  }
  return m;
}

template <typename ClientT>
bool call_trigger(const typename ClientT::SharedPtr& client,
                  std::string* message) {
  auto req = std::make_shared<typename ClientT::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(15s) != std::future_status::ready) return false;
  const auto res = future.get();
  if (message) *message = res->message;
  return res->success;
}

// get_stats 的 message 形如 "模式=mapping 扫描=922 节点=116 ..."。
int parse_scan_count(const std::string& msg) {
  const auto pos = msg.find("扫描=");
  if (pos == std::string::npos) return -1;
  return std::stoi(msg.substr(pos + 7));
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 临时目录带 PID：并行 ctest / 两个测试进程共用 /tmp 时不会互相删目录。
  const auto dir = std::filesystem::temp_directory_path() /
                   ("mini_slam_concurrency_test_" +
                    std::to_string(static_cast<long>(::getpid())));
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);  // 启动时清一次（防上次崩溃残留）
  std::filesystem::create_directories(dir, ec);
  const std::string graph_path = (dir / "stress.mmpg").string();

  int rc = 0;
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("graph_path", graph_path),
        rclcpp::Parameter("minimum_travel_distance", 0.05),
        rclcpp::Parameter("minimum_travel_heading", 0.05),
        rclcpp::Parameter("map_update_interval", 0.1),
        rclcpp::Parameter("transform_publish_period", 0.02),
        rclcpp::Parameter("sparsify_enabled", true),
        rclcpp::Parameter("sparsify_every", 25),
    });
    auto node = std::make_shared<MiniSlamNode>(options);
    auto driver = std::make_shared<rclcpp::Node>("mini_slam_stress_driver");

    auto odom_pub = driver->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    auto scan_pub =
        driver->create_publisher<sensor_msgs::msg::LaserScan>("scan", 50);
    auto save_cli =
        driver->create_client<std_srvs::srv::Trigger>("save_graph");
    auto load_cli =
        driver->create_client<std_srvs::srv::Trigger>("load_graph");
    auto stats_cli =
        driver->create_client<std_srvs::srv::Trigger>("get_stats");

    // laser → base_footprint 外参：让节点真的走 TF lookup 路径。
    tf2_ros::StaticTransformBroadcaster static_tf(driver);
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = driver->now();
      t.header.frame_id = "base_footprint";
      t.child_frame_id = "laser";
      t.transform.translation.x = 0.10;
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.2;
      t.transform.rotation.z = std::sin(0.05 / 2.0);
      t.transform.rotation.w = std::cos(0.05 / 2.0);
      static_tf.sendTransform(t);
    }

    rclcpp::executors::MultiThreadedExecutor node_exec(
        rclcpp::ExecutorOptions(), 4);
    node_exec.add_node(node);
    rclcpp::executors::SingleThreadedExecutor driver_exec;
    driver_exec.add_node(driver);

    std::thread node_spin([&] { node_exec.spin(); });
    std::thread driver_spin([&] { driver_exec.spin(); });

    const bool services_up =
        save_cli->wait_for_service(15s) && load_cli->wait_for_service(15s) &&
        stats_cli->wait_for_service(15s);
    if (!services_up) {
      std::puts("[FAIL] 服务未在超时内就绪");
      rc = 1;
    } else {
      int failures = 0;
      // 等静态 TF 进入节点 listener，避免开头几帧被有意丢弃。
      std::this_thread::sleep_for(250ms);
      for (int i = 0; i < 80 && failures == 0; ++i) {
        odom_pub->publish(make_odom(i));
        scan_pub->publish(make_scan(i));

        if (i % 8 == 0) {
          std::string msg;
          if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(
                  stats_cli, &msg)) {
            std::printf("[FAIL] get_stats 失败: %s\n", msg.c_str());
            ++failures;
          }
        }
        if (i > 0 && i % 16 == 0) {
          std::string msg;
          if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(
                  save_cli, &msg)) {
            std::printf("[FAIL] save_graph 失败: %s\n", msg.c_str());
            ++failures;
          }
          if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(
                  load_cli, &msg)) {
            std::printf("[FAIL] load_graph 失败: %s\n", msg.c_str());
            ++failures;
          }
        }
        std::this_thread::sleep_for(3ms);
      }
      // 运行时参数校验：非法值必须被拒绝（successful=false），合法值可生效。
      if (failures == 0) {
        const auto bad_queue = node->set_parameter(
            rclcpp::Parameter("processing_queue_size", 0));
        const auto bad_sparse =
            node->set_parameter(rclcpp::Parameter("sparsify_every", 0));
        const auto bad_res =
            node->set_parameter(rclcpp::Parameter("resolution", 0.0));
        const auto bad_coarse = node->set_parameter(
            rclcpp::Parameter("loop_match_minimum_response_coarse", -1.0));
        const auto bad_fine = node->set_parameter(
            rclcpp::Parameter("loop_match_minimum_response_fine", 2.0));
        if (bad_queue.successful || bad_sparse.successful ||
            bad_res.successful || bad_coarse.successful ||
            bad_fine.successful) {
          std::puts("[FAIL] 非法运行时参数未被拒绝");
          ++failures;
        } else if (!node->set_parameter(
                        rclcpp::Parameter("processing_queue_size", 8))
                        .successful ||
                   !node->set_parameter(rclcpp::Parameter("resolution", 0.08))
                        .successful ||
                   !node->set_parameter(rclcpp::Parameter(
                                            "loop_match_minimum_response_coarse",
                                            0.3))
                        .successful ||
                   !node->set_parameter(rclcpp::Parameter(
                                            "loop_match_minimum_response_fine",
                                            0.6))
                        .successful) {
          std::puts("[FAIL] 合法运行时参数被误拒");
          ++failures;
        }
      }

      // 证明 laser→base_footprint 的 TF 路径下扫描确实进入了 core。
      if (failures == 0) {
        std::string msg;
        if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(stats_cli,
                                                                  &msg)) {
          std::puts("[FAIL] TF 路径验证前 get_stats 失败");
          ++failures;
        } else {
          const int scans = parse_scan_count(msg);
          std::printf("  TF 路径后扫描计数: %d\n", scans);
          if (scans <= 0) {
            std::puts("[FAIL] 扫描未进入处理（TF 可能全部 lookup 失败）");
            ++failures;
          }
        }
      }

      // 运行时 graph_path / timer / 拒绝项 / sparsify 应用。
      if (failures == 0) {
        const auto runtime_graph = (dir / "runtime.mmpg").string();
        std::string msg;
        if (!node->set_parameter(
                     rclcpp::Parameter("graph_path", runtime_graph))
                 .successful) {
          std::puts("[FAIL] 运行时 graph_path 设置失败");
          ++failures;
        } else if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(
                       save_cli, &msg) ||
                   !std::filesystem::exists(runtime_graph)) {
          std::puts("[FAIL] 运行时 graph_path 未影响 save_graph");
          ++failures;
        }
      }
      if (failures == 0) {
        if (!node->set_parameter(
                 rclcpp::Parameter("map_update_interval", 0.2))
                 .successful ||
            !node->set_parameter(
                 rclcpp::Parameter("transform_publish_period", 0.05))
                 .successful) {
          std::puts("[FAIL] timer 参数运行时设置失败");
          ++failures;
        } else {
          std::string msg;
          if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(
                  stats_cli, &msg)) {
            std::puts("[FAIL] timer 重建后 get_stats 失败");
            ++failures;
          }
        }
      }
      if (failures == 0) {
        const auto bad_topic =
            node->set_parameter(rclcpp::Parameter("scan_topic", "scan2"));
        const auto bad_map =
            node->set_parameter(rclcpp::Parameter("map_frame", "map2"));
        if (bad_topic.successful || bad_map.successful ||
            bad_topic.reason.find("运行时不支持修改") == std::string::npos ||
            bad_map.reason.find("运行时不支持修改") == std::string::npos) {
          std::puts("[FAIL] 运行时不支持修改的 frame/topic 未被明确拒绝");
          ++failures;
        }
      }
      if (failures == 0) {
        if (!node->set_parameter(
                 rclcpp::Parameter("sparsify_enabled", true))
                 .successful ||
            !node->set_parameter(rclcpp::Parameter("sparsify_every", 10))
                 .successful) {
          std::puts("[FAIL] sparsify 运行时设置失败");
          ++failures;
        }
      }

      // ---- Phase 2：TF 晚于扫描到达 —— 验证 TF 监听有独立线程 ----
      // 场景：scan 的时间戳比"当前已知的那帧 TF"晚 10ms，能覆盖该时刻的
      // 下一帧 TF（+20ms）要再过 5ms 才发布。
      //   - TF 回调与 scan_callback 共用一个执行器线程时：scan 内的 20ms
      //     lookupTransform 把那帧必需的 TF 堵在回调队列里 → 等待必然超时
      //     → 每帧都被丢弃（旧 spin_thread=false + 默认 callback group）；
      //   - TF 监听有专用线程时：5ms 后 TF 入库 → lookup 成功 → 扫描被处理。
      // 这不是"时序巧合"：等待窗口（5ms）远小于 lookup 超时（20ms），
      // 也远小于帧间隔（60ms）。
      std::this_thread::sleep_for(300ms);  // 先等 Phase 1 的队列排空
      if (failures == 0) {
        tf2_ros::TransformBroadcaster dyn_tf(driver);
        const auto stamp_ns = [](std::int64_t ns) {
          builtin_interfaces::msg::Time t;
          t.sec = static_cast<std::int32_t>(ns / 1'000'000'000LL);
          t.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000LL);
          return t;
        };
        const auto send_dyn_tf = [&](std::int64_t ns) {
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = stamp_ns(ns);
          t.header.frame_id = "base_footprint";
          t.child_frame_id = "laser_dyn";
          t.transform.translation.x = 0.10;
          t.transform.rotation.w = 1.0;
          dyn_tf.sendTransform(t);
        };

        auto dyn_scan = make_scan(0);
        dyn_scan.header.frame_id = "laser_dyn";  // 只走动态 TF，不蹭静态外参

        std::string msg;
        int before = -1;
        if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(stats_cli,
                                                                  &msg)) {
          std::puts("[FAIL] 动态 TF 阶段前 get_stats 失败");
          ++failures;
        } else {
          before = parse_scan_count(msg);
        }

        constexpr int kDynFrames = 20;
        constexpr std::int64_t kTfPeriodNs = 20'000'000LL;   // 20ms
        constexpr std::int64_t kFramePeriodNs = 100'000'000LL;  // 100ms
        const std::int64_t base_ns = 1'000'000'000LL;  // 1s：避开 0 时间戳
        for (int i = 0; failures == 0 && i < kDynFrames; ++i) {
          const std::int64_t s =
              base_ns + static_cast<std::int64_t>(i) * kFramePeriodNs;
          send_dyn_tf(s);  // 当前可用的 TF
          auto odom = make_odom(100 + i);
          odom.header.stamp = stamp_ns(s);
          odom_pub->publish(odom);
          auto scan = dyn_scan;
          scan.header.stamp = stamp_ns(s + 10'000'000LL);  // 比当前 TF 晚 10ms
          scan_pub->publish(scan);
          std::this_thread::sleep_for(5ms);
          // 能覆盖 scan 时刻的那帧 TF：只有它在 scan 的等待窗口内入库，
          // 这帧扫描才可能被处理。
          send_dyn_tf(s + kTfPeriodNs);
          std::this_thread::sleep_for(55ms);
        }

        // worker 处理是异步的：轮询 get_stats，最多等 4s 让计数稳定。
        int after = before;
        for (int wait = 0; failures == 0 && wait < 40; ++wait) {
          std::this_thread::sleep_for(100ms);
          std::string m;
          if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(stats_cli,
                                                                    &m)) {
            std::puts("[FAIL] 动态 TF 阶段 get_stats 失败");
            ++failures;
            break;
          }
          after = parse_scan_count(m);
          if (after - before >= kDynFrames) break;
        }
        if (failures == 0) {
          const int processed = after - before;
          std::printf(
              "  动态 TF 阶段：发布 %d 帧，扫描计数 %d → %d（处理 %d 帧）\n",
              kDynFrames, before, after, processed);
          if (processed < kDynFrames * 8 / 10) {
            std::printf(
                "[FAIL] TF 晚于扫描时大量丢帧（%d/%d）：TF 监听可能又被"
                "执行器线程串行化了\n",
                processed, kDynFrames);
            ++failures;
          }
        }
      }

      if (failures == 0 && !std::filesystem::exists(graph_path)) {
        std::puts("[FAIL] 压测结束后图文件不存在");
        ++failures;
      }
      if (failures == 0) {
        std::puts(
            "[PASS] test_node_concurrency（80 帧混合负载 + 动态 TF 20 帧）");
      }
      rc = failures == 0 ? 0 : 1;
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
  return rc;
}
