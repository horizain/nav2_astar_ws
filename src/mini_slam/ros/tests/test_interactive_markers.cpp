// 交互式 marker（Phase 4C）集成回归：
//   1) enable_interactive_markers=true → 节点在 /mini_slam/update 上广播
//      InteractiveMarker（名字 "vertex_<id>"），RViz 的 InteractiveMarkers
//      display 就是订阅这个主题；
//   2) 向 /mini_slam/feedback 发 MOUSE_UP 反馈 → marker_feedback →
//      Mapper::move_vertex → 固定该节点重跑 SPA；
//   3) 用 get_stats 轮询 SPA 计数增加，证明整条链路真的打通（而不是只
//      把消息收下了）。
//
// 话题名/服务名按 interactive_markers 库的约定：
//   <namespace>/update, <namespace>/feedback, <namespace>/get_interactive_markers
// namespace 这里固定是 "mini_slam"（节点命名空间为 "/"）→ 绝对名
// /mini_slam/update、/mini_slam/feedback。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unistd.h>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>

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

// get_stats 的 message 形如 "模式=mapping 扫描=0 节点=0 闭环=0 SPA=3 点云=200"
int parse_field(const std::string& msg, const std::string& key) {
  const auto pos = msg.find(key);
  if (pos == std::string::npos) return -1;
  return std::stoi(msg.substr(pos + key.size()));
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

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 临时目录带 PID：并行 ctest / 两个进程互删同一个目录会互相踩。
  const auto dir = std::filesystem::temp_directory_path() /
                   ("mini_slam_interactive_markers_test_" +
                    std::to_string(static_cast<long>(::getpid())));
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  const std::string graph_path = (dir / "map.mmpg").string();

  // ---------- core 先生成一张 5 节点小图（与 test_localization_init 同法）----------
  {
    Mapper mapper;
    const LaserScan scan = make_scan();
    for (int i = 0; i < 5; ++i) {
      mapper.process_scan(scan, Pose2{0.6 * i, 0.0, 0.0});
    }
    std::string err;
    if (!mapper.save_graph(graph_path, &err)) {
      std::printf("[FAIL] 构图保存失败: %s\n", err.c_str());
      rclcpp::shutdown();
      return 1;
    }
    std::printf("  测试图: %d 节点 → %s\n", mapper.graph().size(),
                graph_path.c_str());
  }

  int rc = 0;
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("mode", "mapping"),
        rclcpp::Parameter("graph_path", graph_path),
        rclcpp::Parameter("enable_interactive_markers", true),
        rclcpp::Parameter("interactive_marker_max_vertices", 10),
        rclcpp::Parameter("map_update_interval", 0.1),
    });
    auto node = std::make_shared<MiniSlamNode>(options);
    auto driver = std::make_shared<rclcpp::Node>("interactive_markers_driver");

    std::mutex markers_mutex;
    std::vector<std::string> marker_names;
    int updates_seen = 0;
    auto update_sub =
        driver->create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>(
            "/mini_slam/update", 10,
            [&](visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr
                    msg) {
              std::lock_guard<std::mutex> lock(markers_mutex);
              ++updates_seen;
              for (const auto& m : msg->markers) {
                marker_names.push_back(m.name);
              }
            });
    auto feedback_pub =
        driver->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>(
            "/mini_slam/feedback", 10);
    auto stats_cli =
        driver->create_client<std_srvs::srv::Trigger>("/get_stats");

    rclcpp::executors::MultiThreadedExecutor node_exec(
        rclcpp::ExecutorOptions(), 4);
    node_exec.add_node(node);
    rclcpp::executors::SingleThreadedExecutor driver_exec;
    driver_exec.add_node(driver);
    std::thread node_spin([&] { node_exec.spin(); });
    std::thread driver_spin([&] { driver_exec.spin(); });

    std::string stats_msg;
    if (!stats_cli->wait_for_service(15s)) {
      std::puts("[FAIL] get_stats 服务未就绪");
      rc = 1;
    } else {
      int spa_before = -1;
      if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(stats_cli,
                                                                &stats_msg)) {
        std::puts("[FAIL] get_stats 失败（初始）");
        rc = 1;
      } else {
        spa_before = parse_field(stats_msg, "SPA=");
        std::printf("  初始 stats: %s\n", stats_msg.c_str());
      }

      // ---------- 1) 等 /mini_slam/update 里的 marker ----------
      std::vector<std::string> names;
      int updates = 0;
      for (int i = 0; rc == 0 && i < 100; ++i) {  // 最多 10s
        {
          std::lock_guard<std::mutex> lock(markers_mutex);
          names = marker_names;
          updates = updates_seen;
        }
        if (!names.empty()) break;
        std::this_thread::sleep_for(100ms);
      }

      std::string target;
      for (const auto& n : names) {
        if (n.rfind("vertex_", 0) == 0) {
          target = n;
          break;
        }
      }
      if (target.empty()) {
        std::printf(
            "[FAIL] 10s 内未收到 vertex_* marker（update 消息 %d 条，name 数 "
            "%zu）\n",
            updates, names.size());
        for (const auto& n : names) std::printf("    marker name: %s\n", n.c_str());
        rc = 1;
      } else {
        std::printf("  收到 marker: %s（update 消息 %d 条，共 %zu 个名字）\n",
                    target.c_str(), updates, names.size());
        const int id = std::stoi(target.substr(std::string("vertex_").size()));
        // 图有 5 个节点 → 至少应有 5 个 marker（max_vertices=10 不抽样）
        if (names.size() < 5) {
          std::printf("[FAIL] marker 数偏少（%zu < 5）\n", names.size());
          rc = 1;
        }
        (void)id;
      }

      // ---------- 2) 发 MOUSE_UP 反馈 ----------
      if (rc == 0) {
        // 等 DDS 发现：server 的 /mini_slam/feedback 订阅必须已匹配，
        // 否则消息发出去就丢了（volatile QoS，无重传历史）。
        int matched = 0;
        for (int i = 0; i < 100 && matched == 0; ++i) {
          matched = static_cast<int>(feedback_pub->get_subscription_count());
          if (matched == 0) std::this_thread::sleep_for(100ms);
        }
        std::printf("  /mini_slam/feedback 匹配订阅数: %d\n", matched);
        if (matched == 0) {
          std::puts("[FAIL] /mini_slam/feedback 没有匹配订阅者");
          rc = 1;
        } else {
          visualization_msgs::msg::InteractiveMarkerFeedback fb;
          fb.header.frame_id = "map";
          fb.client_id = "mini_slam_interactive_markers_test";
          fb.marker_name = target;
          fb.control_name = "move_xy";
          fb.event_type =
              visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP;
          fb.pose.position.x = 1.5;
          fb.pose.position.y = 0.8;
          fb.pose.position.z = 0.0;
          fb.pose.orientation =
              mini_slam::ros_node::yaw_to_quat(0.4);
          feedback_pub->publish(fb);

          // ---------- 3) 轮询 stats：SPA 计数必须增加 ----------
          // 首包偶发丢包时重发一次（MOUSE_UP 是幂等的位置提交，不会改变语义）。
          int spa_after = spa_before;
          std::string last;
          int retries = 0;
          for (int i = 0; i < 100 && rc == 0; ++i) {  // 最多 10s
            std::this_thread::sleep_for(100ms);
            if (!call_trigger<rclcpp::Client<std_srvs::srv::Trigger>>(
                    stats_cli, &last)) {
              std::puts("[FAIL] get_stats 失败（反馈后）");
              rc = 1;
              break;
            }
            spa_after = parse_field(last, "SPA=");
            if (spa_after > spa_before) break;
            if (i == 20 && retries == 0) {  // 2s 无反应 → 重发一次
              ++retries;
              feedback_pub->publish(fb);
            }
          }
          std::printf("  反馈后 stats: %s\n", last.c_str());
          if (rc == 0 && (spa_before < 0 || spa_after <= spa_before)) {
            std::printf(
                "[FAIL] MOUSE_UP 反馈未触发 SPA（SPA %d → %d）\n"
                "        marker='%s'，说明 feedback → move_vertex 链路没打通\n",
                spa_before, spa_after, target.c_str());
            rc = 1;
          } else if (rc == 0) {
            std::printf("  SPA 次数 %d → %d（拖动节点后重优化完成）\n",
                        spa_before, spa_after);
          }
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
  if (rc == 0) std::puts("[PASS] test_interactive_markers");
  return rc;
}
