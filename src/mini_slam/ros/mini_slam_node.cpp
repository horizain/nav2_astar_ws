// ============================================================================
// mini_slam_node 入口：节点实现见 mini_slam_node.hpp（便于并发回归测试）。
// 生产入口用多线程执行器：scan_callback 里的 TF lookup 最长等 20ms，单线程
// rclcpp::spin 会把 odom/服务/参数/定时器回调一起堵住。
// ============================================================================
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mini_slam_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    // 构造失败（启动期参数非法等）时节点对象还没进执行器，add_node 不会被调用。
    auto node = std::make_shared<MiniSlamNode>();
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception& e) {
    // 启动期参数非法等致命错误：打印明确原因后退出，而不是静默运行。
    RCLCPP_FATAL(rclcpp::get_logger("mini_slam_node"), "启动失败: %s",
                 e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
