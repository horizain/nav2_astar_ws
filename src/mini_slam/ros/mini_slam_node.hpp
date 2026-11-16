#pragma once
// ============================================================================
// MiniSlamNode：slam_toolbox 行为对齐版 ROS 包装。
//   订阅 /scan + /odom → 异步处理队列 → core::Mapper → 发布 /map（latched）
//   + map→odom TF
//
// 线程模型（Phase 1 并发修复 + Phase 4A TF 专用线程）：
//   - main executor：生产入口是 MultiThreadedExecutor(4)。注意节点默认
//     callback group 是 MutuallyExclusive，所以 scan/odom/服务/timer 之间
//     仍然串行（rclcpp 语义）；真正的并发来自下面两条独立线程。
//   - TF listener：TransformListener 用 spin_thread=true —— tf2_ros 会为
//     /tf、/tf_static 建独立 callback group + 独立 SingleThreadedExecutor +
//     专用线程，并自行 buffer_.setUsingDedicatedThread(true)。因此
//     scan_callback 里的 20ms lookupTransform 等待期间，TF 依然在另一个
//     线程上入库，不会因为"等待者占着唯一执行器线程"而必然丢帧。
//   - worker：出队后持 mapper_mutex_ 处理扫描（独立 std::thread）；
//   - map timer：持锁复制图快照，释放锁后再渲染 + 发布；
//   - TF timer：持锁取 last_map_odom_/has_tf_ 的轻量快照，锁外发布；
//   - marker timer（Phase 4C，仅 enable_interactive_markers=true）：持锁取
//     "图摘要 + 选中节点位姿" 快照，释放锁后再清空/重建 InteractiveMarker；
//   - interactive marker feedback（RViz 拖拽）：持锁调 move_vertex，释放锁后
//     再 setPose/applyChanges；
//   - save/load/stats 服务与参数回调：同样走 mapper_mutex_。
//   mapper_mutex_ 是唯一保护 mapper_ 及其派生态的锁；不递归、不在锁内
//   再做长时间 ROS 发布。
// ============================================================================
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include "mini_slam/map_builder.hpp"
#include "mini_slam/map_output.hpp"
#include "mini_slam/mapper.hpp"
#include "mini_slam/odom_buffer.hpp"
#include "mini_slam/validation.hpp"

namespace mini_slam::ros_node {

// 四元数 ← yaw（只在平面内，手工构造省一个依赖）
inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

inline double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 时间戳统一转成纳秒整数再比较，避免 rclcpp::Time 的 clock type 差异。
inline std::int64_t stamp_ns(const builtin_interfaces::msg::Time& t) {
  return static_cast<std::int64_t>(t.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(t.nanosec);
}

inline bool is_zero_stamp(const builtin_interfaces::msg::Time& t) {
  return t.sec == 0 && t.nanosec == 0;
}

// 扫描时间戳在核心层以 double 秒保存；这里还原成 ROS 时间消息。
inline builtin_interfaces::msg::Time stamp_from_seconds(double seconds) {
  builtin_interfaces::msg::Time t;
  if (!std::isfinite(seconds) || seconds <= 0.0) return t;
  auto sec = static_cast<std::int64_t>(std::floor(seconds));
  auto nsec =
      static_cast<std::int64_t>(std::llround((seconds - static_cast<double>(sec)) *
                                             1'000'000'000.0));
  if (nsec >= 1'000'000'000LL) {
    ++sec;
    nsec -= 1'000'000'000LL;
  }
  t.sec = static_cast<std::int32_t>(sec);
  t.nanosec = static_cast<std::uint32_t>(nsec);
  return t;
}

}  // namespace mini_slam::ros_node

class MiniSlamNode : public rclcpp::Node {
 public:
  explicit MiniSlamNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("mini_slam_node", options) {
    // ---------- 参数（名字对齐 slam_toolbox）----------
    mini_slam::Params p;
    resolution_ = declare_parameter("resolution", p.resolution);
    p.resolution = resolution_;  // 匹配场与地图渲染使用同一分辨率
    p.min_travel_distance =
        declare_parameter("minimum_travel_distance", p.min_travel_distance);
    p.min_travel_heading =
        declare_parameter("minimum_travel_heading", p.min_travel_heading);
    p.scan_range_max =
        declare_parameter("scan_range_max", p.scan_range_max);
    p.loop_search_distance =
        declare_parameter("loop_search_maximum_distance", p.loop_search_distance);
    p.loop_min_chain_size =
        declare_parameter("loop_match_minimum_chain_size", p.loop_min_chain_size);
    p.loop_min_response_coarse = declare_parameter(
        "loop_match_minimum_response_coarse", p.loop_min_response_coarse);
    p.loop_min_response_fine =
        declare_parameter("loop_match_minimum_response_fine", p.loop_min_response_fine);
    map_update_interval_ =
        declare_parameter("map_update_interval", 5.0);  // 秒
    tf_publish_period_ =
        declare_parameter("transform_publish_period", 0.05);  // 秒
    odom_frame_ = declare_parameter("odom_frame", "odom");
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    scan_topic_ = declare_parameter("scan_topic", "scan");
    odom_topic_ = declare_parameter("odom_topic", "odom");
    map_file_ = declare_parameter("map_output_prefix", "");
    graph_path_ = declare_parameter("graph_path", "");
    // 运行模式（对应 slam_toolbox 的 mapping / localization 两种模式）
    const std::string mode_str = declare_parameter("mode", "mapping");
    queue_size_ = declare_parameter("processing_queue_size", 10);
    p.sparsify_enabled = declare_parameter("sparsify_enabled", p.sparsify_enabled);
    p.sparsify_keep_radius =
        declare_parameter("sparsify_keep_radius", p.sparsify_keep_radius);
    p.sparsify_min_keep_recent = declare_parameter(
        "sparsify_min_keep_recent", p.sparsify_min_keep_recent);
    p.sparsify_every = declare_parameter("sparsify_every", p.sparsify_every);
    p.localization_window =
        declare_parameter("localization_window", p.localization_window);
    // 交互式 marker（Phase 4C）：默认关，避免默认开销；启动参数打开。
    // enable_* 决定启动期是否创建 InteractiveMarkerServer，运行期不允许改。
    enable_interactive_markers_ =
        declare_parameter("enable_interactive_markers", false);
    interactive_marker_max_vertices_ = static_cast<int>(declare_parameter(
        "interactive_marker_max_vertices", 200));

    // ---------- 启动期集中校验（非法值直接抛出，带参数名）----------
    mini_slam::NodeParams np;
    np.processing_queue_size = queue_size_;
    np.map_update_interval = map_update_interval_;
    np.transform_publish_period = tf_publish_period_;
    np.scan_topic = scan_topic_;
    np.odom_topic = odom_topic_;
    np.map_frame = map_frame_;
    np.odom_frame = odom_frame_;
    np.base_frame = base_frame_;
    np.interactive_marker_max_vertices = interactive_marker_max_vertices_;
    if (const std::string err = mini_slam::validate_parameters(p, np);
        !err.empty()) {
      throw std::invalid_argument("mini_slam 启动参数非法：" + err);
    }
    if (mode_str != "mapping" && mode_str != "localization") {
      throw std::invalid_argument(
          "mini_slam 启动参数非法：参数 'mode'=" + mode_str +
          " 非法：只接受 mapping / localization");
    }
    apply_mode(mode_str, p);

    // ---------- 自动加载图（必须在订阅/timer/worker 之前）----------
    if (!graph_path_.empty()) {
      std::string err;
      if (mapper_->load_graph(graph_path_, &err)) {
        RCLCPP_INFO(get_logger(), "已自动加载图 '%s'（%d 个顶点）",
                    graph_path_.c_str(), mapper_->graph().size());
      } else if (mapper_->mode() == mini_slam::Mapper::Mode::Localization) {
        throw std::invalid_argument(
            "定位模式启动失败：无法加载 graph_path='" + graph_path_ +
            "'：" + err);
      } else {
        RCLCPP_WARN(get_logger(),
                    "graph_path='%s' 加载失败（%s），mapping 模式以空图继续",
                    graph_path_.c_str(), err.c_str());
      }
    } else if (mapper_->mode() == mini_slam::Mapper::Mode::Localization) {
      RCLCPP_WARN(get_logger(),
                  "定位模式未设置 graph_path：图加载前 localize 会退回里程计；"
                  "可稍后调用 load_graph 或发布 /initialpose 初始化");
    }

    // 运行时参数：先整体校验，再应用；非法返回 successful=false + reason。
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          for (const auto& param : params) {
            std::string err = validate_runtime_parameter(param);
            if (!err.empty()) {
              res.successful = false;
              res.reason = std::move(err);
              return res;
            }
          }

          // 全部合法后才生效，避免"前半部分已改、后半部分被拒"的半状态。
          std::optional<std::string> new_mode;
          double new_map_interval = 0.0;
          double new_tf_period = 0.0;
          int new_queue_size = 0;
          {
            std::lock_guard<std::mutex> lock(mapper_mutex_);
            mini_slam::Params q = mapper_->params();
            for (const auto& param : params) {
              const std::string& n = param.get_name();
              if (n == "mode") {
                new_mode = param.as_string();
              } else if (n == "resolution") {
                resolution_ = param.as_double();
                q.resolution = resolution_;
              } else if (n == "minimum_travel_distance") {
                q.min_travel_distance = param.as_double();
              } else if (n == "minimum_travel_heading") {
                q.min_travel_heading = param.as_double();
              } else if (n == "scan_range_max") {
                q.scan_range_max = param.as_double();
              } else if (n == "loop_search_maximum_distance") {
                q.loop_search_distance = param.as_double();
              } else if (n == "loop_match_minimum_chain_size") {
                q.loop_min_chain_size = static_cast<int>(param.as_int());
              } else if (n == "loop_match_minimum_response_coarse") {
                q.loop_min_response_coarse = param.as_double();
              } else if (n == "loop_match_minimum_response_fine") {
                q.loop_min_response_fine = param.as_double();
              } else if (n == "sparsify_enabled") {
                q.sparsify_enabled = param.as_bool();
              } else if (n == "sparsify_every") {
                q.sparsify_every = static_cast<int>(param.as_int());
              } else if (n == "sparsify_keep_radius") {
                q.sparsify_keep_radius = param.as_double();
              } else if (n == "sparsify_min_keep_recent") {
                q.sparsify_min_keep_recent = static_cast<int>(param.as_int());
              } else if (n == "localization_window") {
                q.localization_window = param.as_double();
              } else if (n == "graph_path") {
                graph_path_ = param.as_string();
              } else if (n == "map_output_prefix") {
                map_file_ = param.as_string();
              } else if (n == "map_update_interval") {
                new_map_interval = param.as_double();
              } else if (n == "transform_publish_period") {
                new_tf_period = param.as_double();
              } else if (n == "processing_queue_size") {
                new_queue_size = static_cast<int>(param.as_int());
              } else if (n == "interactive_marker_max_vertices") {
                // 受 mapper_mutex_ 保护：marker timer 在同一把锁下读它。
                interactive_marker_max_vertices_ =
                    static_cast<int>(param.as_int());
              }
            }
            mapper_->set_params(q);
            if (new_mode) apply_mode(*new_mode, q);
          }

          // 队列长度与裁剪走 proc_mutex_（worker 的队列锁），不混入 mapper 锁。
          if (new_queue_size > 0) {
            std::lock_guard<std::mutex> lock(proc_mutex_);
            queue_size_ = new_queue_size;
            while (static_cast<int>(proc_queue_.size()) > queue_size_) {
              proc_queue_.pop_front();
              ++dropped_;
            }
          }

          // timer 重建必须在释放 mapper_mutex_ 之后：timer 回调可能正在等锁，
          // 持锁重建会形成"回调等锁 / 重建等回调"的死锁。
          if (new_map_interval > 0.0 &&
              new_map_interval != map_update_interval_) {
            map_update_interval_ = new_map_interval;
            map_timer_ = create_wall_timer(
                std::chrono::duration<double>(map_update_interval_),
                [this] { publish_map(); });
          }
          if (new_tf_period > 0.0 && new_tf_period != tf_publish_period_) {
            tf_publish_period_ = new_tf_period;
            tf_timer_ = create_wall_timer(
                std::chrono::duration<double>(tf_publish_period_),
                [this] { publish_tf(); });
          }
          return res;
        });

    // ---------- 订阅 ----------
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          scan_callback(std::move(msg));
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odom_callback(std::move(msg));
        });
    initialpose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            [this](
                geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
                    msg) { initialpose_callback(std::move(msg)); });

    // ---------- 发布 / TF ----------
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local());  // latched，同 slam_toolbox
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // TF listener 走专用线程（spin_thread=true）：tf2_ros 内部自建
    // callback group + SingleThreadedExecutor + 线程，并自行调用
    // buffer_.setUsingDedicatedThread(true)，所以这里不再手工设置。
    // 好处：scan_callback 里带 timeout 的 lookupTransform 等待时，/tf 仍能
    // 并发入库；旧写法（spin_thread=false + 默认组）会让等待把 TF 回调堵死。
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this,
                                                     /*spin_thread=*/true);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    map_timer_ = create_wall_timer(
        std::chrono::duration<double>(map_update_interval_),
        [this] { publish_map(); });
    tf_timer_ = create_wall_timer(
        std::chrono::duration<double>(tf_publish_period_),
        [this] { publish_tf(); });

    // ---------- 交互式 marker（Phase 4C，可选）----------
    // 只有启动参数打开才创建 server：它自带 /mini_slam/update 发布器、
    // /mini_slam/feedback 订阅器与 /mini_slam/get_interactive_markers 服务，
    // 默认关掉可避免无谓的 DDS 资源与 CPU。构造函数接受 NodePtr，内部取
    // node interfaces（不需要 shared_from_this）。
    if (enable_interactive_markers_) {
      marker_server_ =
          std::make_unique<interactive_markers::InteractiveMarkerServer>(
              "mini_slam", this);
      marker_timer_ = create_wall_timer(
          std::chrono::milliseconds(500), [this] { refresh_markers(); });
      RCLCPP_INFO(get_logger(),
                  "交互式 marker 已启用：RViz 加 InteractiveMarkers display，"
                  "Update Topic=/mini_slam/update（最多显示 %d 个节点）",
                  interactive_marker_max_vertices_);
    }

    // ---------- 服务（对应 slam_toolbox 的 serialize/localization 接口）----------
    save_srv_ = create_service<std_srvs::srv::Trigger>(
        "save_graph",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::string err;
          {
            // save 期间持锁：worker 不能在序列化中途改图（等价于暂停消费）。
            std::lock_guard<std::mutex> lock(mapper_mutex_);
            const std::string path = graph_path_.empty()
                                         ? "mini_slam_graph.mmpg"
                                         : graph_path_;
            res->success = mapper_->save_graph(path, &err);
            res->message = res->success ? ("已保存 " + path) : err;
          }
        });
    load_srv_ = create_service<std_srvs::srv::Trigger>(
        "load_graph",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::string err;
          {
            // load 期间持锁：避免 worker 继续访问被替换的旧图。
            std::lock_guard<std::mutex> lock(mapper_mutex_);
            const std::string path = graph_path_.empty()
                                         ? "mini_slam_graph.mmpg"
                                         : graph_path_;
            res->success = mapper_->load_graph(path, &err);
            res->message = res->success ? ("已加载 " + path) : err;
            if (res->success) {
              // 旧图的 TF/定位跟踪状态不能残留到新图。
              last_map_odom_ = {};
              has_tf_ = false;
              last_tf_stamp_.sec = 0;
              last_tf_stamp_.nanosec = 0;
              has_tf_stamp_ = false;
              last_published_tf_stamp_.sec = 0;
              last_published_tf_stamp_.nanosec = 0;
              has_published_tf_stamp_ = false;
              loc_warn_count_ = 0;
              // 图被整体替换：地图渲染缓存必须失效（即使顶点/边数恰好相同）。
              ++map_generation_;
            }
          }
        });
    stats_srv_ = create_service<std_srvs::srv::Trigger>(
        "get_stats",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lock(mapper_mutex_);
          const auto& st = mapper_->stats();
          res->success = true;
          res->message = std::format(
              "模式={} 扫描={} 节点={} 闭环={} SPA={} 点云={}B",
              mapper_->mode() == mini_slam::Mapper::Mode::Mapping ? "mapping"
                                                                  : "localization",
              st.scans, st.nodes, st.loop_closures, st.solves,
              total_scan_points_locked());
        });

    // ---------- 异步处理队列（对应 slam_toolbox 的 async 模式）----------
    // 放在所有可能抛异常的初始化之后，避免构造失败时留下 joinable 线程。
    worker_ = std::thread([this] { worker_loop(); });

    RCLCPP_INFO(get_logger(),
                "mini_slam 就绪: scan='%s' odom='%s' resolution=%.3f queue=%d",
                scan_topic_.c_str(), odom_topic_.c_str(), resolution_,
                queue_size_);
  }

  ~MiniSlamNode() override {
    stopping_ = true;
    {
      std::lock_guard<std::mutex> lock(proc_mutex_);
    }
    proc_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  struct PendingScan {
    mini_slam::LaserScan scan;
    mini_slam::Pose2 odom;
  };

  // 地图渲染缓存键：只要其中任一项变化，就必须重新渲染。
  // stats.nodes/solves 覆盖"图内容变了但顶点/边数没变"的情况（例如闭环后
  // SPA 重摆位姿）；稀疏化只在成功新增节点后触发，因此由 nodes 覆盖。
  // 注意不能把 stats.scans 放进键：它每处理一帧扫描就 +1，而 map timer
  // 是秒级周期，放进来会让缓存每个周期必然 miss，失去缓存意义。
  struct MapRenderKey {
    mini_slam::Mapper::Mode mode{mini_slam::Mapper::Mode::Mapping};
    std::size_t vertices{0};
    std::size_t edges{0};
    int nodes{0};
    int solves{0};
    double resolution{0.0};
    double range_max{0.0};
    double margin{0.0};
    std::string prefix;
    std::uint64_t generation{0};
    bool operator==(const MapRenderKey&) const = default;
  };

  // 交互式 marker 重建缓存键：任一字段变化才重建 marker 集合。
  //   vertices/pose_revision 覆盖"节点数或位姿变了"；solves 覆盖"SPA 重摆
  //   但顶点数不变"；mode 覆盖 mapping↔localization 切换；max_vertices 覆盖
  //   运行时参数；nodes 覆盖稀疏化（点云释放）等 stats 变化；
  //   generation 覆盖 /load_graph 换图（map_generation_，与地图缓存同源）。
  struct MarkerKey {
    mini_slam::Mapper::Mode mode{mini_slam::Mapper::Mode::Mapping};
    std::size_t vertices{0};
    int nodes{0};
    int solves{0};
    std::uint64_t pose_revision{0};
    int max_vertices{0};
    std::string frame;
    std::uint64_t generation{0};
    bool operator==(const MarkerKey&) const = default;
  };

  // 控制朝向：绕 y 轴 -90° → 控制帧 x 轴 = 世界 +z。
  // （RViz 的 MOVE_PLANE 用控制帧 x 轴作平面法线，ROTATE_AXIS 用它作转轴。）
  static constexpr double kAxisControlY = -0.7071067811865476;  // sin(-45°)
  static constexpr double kAxisControlW = 0.7071067811865476;   // cos(45°)

  // ---------- 交互式 marker：构建 ----------
  // 一个节点 = 一个 InteractiveMarker：
  //   - MOVE_PLANE：在 map 的 XY 平面平移。RViz 的平面法线取"控制帧的
  //     x 轴"（见 rviz_default_plugins interactive_marker_control.cpp 的
  //     intersectSomeYzPlane / movePlane），所以控制朝向绕 y 转 -90°，
  //     把 x 轴转到世界 +z；
  //   - ROTATE_AXIS：绕控制帧 x 轴旋转，同一朝向 → 绕世界 z（yaw）。
  // 注意：控制帧被转过 90°，控制内的可视化 marker 需要反向补偿朝向，
  // 否则球/圆盘会立起来。
  static visualization_msgs::msg::InteractiveMarker make_marker(
      int id, const mini_slam::Pose2& pose, const std::string& frame) {
    using Control = visualization_msgs::msg::InteractiveMarkerControl;
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = frame;
    marker.name = std::format("vertex_{}", id);
    marker.description = std::format("mini_slam 节点 {}（拖动后松手 = 重新优化）", id);
    marker.scale = 1.0;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation = mini_slam::ros_node::yaw_to_quat(pose.theta);

    // 半透明球：XY 平面拖拽的手柄
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.24;
    sphere.color.r = 0.20f;
    sphere.color.g = 0.75f;
    sphere.color.b = 1.00f;
    sphere.color.a = 0.55f;
    sphere.pose.orientation.w = 1.0;

    Control move;
    move.name = "move_xy";
    move.description = "XY 平面拖动";
    move.interaction_mode = Control::MOVE_PLANE;
    move.orientation.y = kAxisControlY;
    move.orientation.w = kAxisControlW;
    move.always_visible = true;
    move.markers.push_back(sphere);
    marker.controls.push_back(move);

    // 半透明圆盘：绕 z 旋转的手柄（补偿控制帧的 90°，保持水平）
    visualization_msgs::msg::Marker disc;
    disc.type = visualization_msgs::msg::Marker::CYLINDER;
    disc.scale.x = 0.34;
    disc.scale.y = 0.34;
    disc.scale.z = 0.03;
    disc.color.r = 1.00f;
    disc.color.g = 0.70f;
    disc.color.b = 0.20f;
    disc.color.a = 0.45f;
    disc.pose.orientation.y = 0.7071067811865476;  // 控制帧内再绕 y +90°
    disc.pose.orientation.w = 0.7071067811865476;
    disc.pose.orientation.x = 0.0;
    disc.pose.orientation.z = 0.0;

    Control rotate;
    rotate.name = "rotate_z";
    rotate.description = "绕 z 轴旋转";
    rotate.interaction_mode = Control::ROTATE_AXIS;
    rotate.orientation.y = kAxisControlY;
    rotate.orientation.w = kAxisControlW;
    rotate.always_visible = true;
    rotate.markers.push_back(disc);
    marker.controls.push_back(rotate);

    return marker;
  }

  // ---------------- 交互式 marker：定时重建 ----------------
  // 只在 enable_interactive_markers=true 时被 timer 驱动（0.5s）。
  // 锁纪律：mapper_mutex_ 内只做快照（位姿 + 摘要），清空/插入/发布都在
  // 锁外——marker 重建会触发 ROS 发布，不能占着算法锁。
  void refresh_markers() {
    if (!marker_server_) return;

    MarkerKey key;
    std::vector<std::pair<int, mini_slam::Pose2>> selected;
    std::string frame;
    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      const auto& graph = mapper_->graph();
      key.mode = mapper_->mode();
      key.vertices = graph.vertices().size();
      key.nodes = mapper_->stats().nodes;
      key.solves = mapper_->stats().solves;
      key.pose_revision = graph.pose_revision();
      key.max_vertices = interactive_marker_max_vertices_;
      key.frame = map_frame_;
      key.generation = map_generation_;
      if (marker_key_ && *marker_key_ == key) return;  // 无变化：跳过

      if (key.mode == mini_slam::Mapper::Mode::Mapping && key.vertices > 0) {
        const int total = static_cast<int>(key.vertices);
        const int max_n = std::max(1, interactive_marker_max_vertices_);
        // 均匀抽样：stride = ceil(total / max)。不能"只显示最后 N 个"——
        // 老闭环节点恰恰是最需要手动纠正的，必须有机会被拖到。
        const int stride = std::max(1, (total + max_n - 1) / max_n);
        for (int id = 0; id < total; id += stride) {
          selected.emplace_back(
              id, graph.vertices()[static_cast<size_t>(id)].pose);
        }
        // 补上最新节点：抽样点可能跳过 total-1。
        if (selected.empty() || selected.back().first != total - 1) {
          selected.emplace_back(
              total - 1, graph.vertices()[static_cast<size_t>(total - 1)].pose);
        }
      }
      marker_key_ = key;  // 即使清空（localization/空图）也记账，避免每周期重发
      frame = key.frame;
    }

    // ---- 锁外：重建 marker 集合并广播 ----
    marker_server_->clear();
    for (const auto& [id, pose] : selected) {
      marker_server_->insert(
          make_marker(id, pose, frame),
          [this](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr
                     feedback) { marker_feedback(std::move(feedback)); });
    }
    marker_server_->applyChanges();
  }

  // ---------------- 交互式 marker：反馈（RViz 松手）----------------
  // 只在 MOUSE_UP 提交：拖动过程中的 POSE_UPDATE/ MOVE 事件不跑 SPA，
  // 否则一次拖动会触发上百次图优化。
  void marker_feedback(
      interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr
          feedback) {
    if (!marker_server_ || !feedback) return;
    if (feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      return;
    }

    // marker 名 → 节点 id：namespace 前缀必须是 "vertex_"
    constexpr std::string_view kPrefix = "vertex_";
    const std::string& name = feedback->marker_name;
    int id = -1;
    if (name.rfind(kPrefix, 0) == 0) {
      try {
        id = std::stoi(name.substr(kPrefix.size()));
      } catch (const std::exception&) {
        id = -1;
      }
    }
    if (id < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "交互式 marker 反馈的 name='%s' 无法解析为节点 id，忽略",
                           name.c_str());
      return;
    }

    // 反馈位姿所在帧：空（RViz 有时省略）或 map_frame_ 才可信；
    // 其它帧（如 odom）会带来坐标系错位，宁可拒绝。
    if (!feedback->header.frame_id.empty() &&
        feedback->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "交互式 marker 反馈帧 '%s' != '%s'，忽略",
                           feedback->header.frame_id.c_str(),
                           map_frame_.c_str());
      return;
    }

    const mini_slam::Pose2 pose{
        feedback->pose.position.x, feedback->pose.position.y,
        mini_slam::ros_node::yaw_from_quat(feedback->pose.orientation)};

    enum class Reject { None, NotMapping, BadVertex };
    Reject reject = Reject::None;
    {
      // 锁内只做图操作（move_vertex 内含一次 SPA）；发布放锁外。
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      if (mapper_->mode() != mini_slam::Mapper::Mode::Mapping) {
        reject = Reject::NotMapping;
      } else if (!mapper_->move_vertex(id, pose)) {
        reject = Reject::BadVertex;
      }
    }
    if (reject == Reject::NotMapping) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "交互式编辑只在 mapping 模式生效（当前 localization）");
      return;
    }
    if (reject == Reject::BadVertex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "交互式编辑被拒绝：节点 %d 不存在或位姿非有限 (%.3f, %.3f, %.3f)",
          id, pose.x, pose.y, pose.theta);
      return;
    }

    // 成功：把客户端 marker 摆到用户放下的位置（不必等下一次 timer）。
    // setPose/applyChanges 只用 marker server 内部递归锁，不碰 mapper_mutex_。
    marker_server_->setPose(name, feedback->pose);
    marker_server_->applyChanges();
    RCLCPP_INFO(get_logger(),
                "交互式编辑：节点 %d → (%.3f, %.3f, %.3f)，已重跑 SPA 并刷新地图",
                id, pose.x, pose.y, pose.theta);
  }

  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    const mini_slam::Pose2 pose{
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        mini_slam::ros_node::yaw_from_quat(msg->pose.pose.orientation)};
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push(stamp, pose);  // ~5s 余量（400 帧 @ 80Hz）
  }

  std::optional<mini_slam::Pose2> odom_at(double stamp) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return odom_buffer_.at(stamp);
  }

  void initialpose_callback(
      geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    mini_slam::Pose2 map_pose{
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        mini_slam::ros_node::yaw_from_quat(msg->pose.pose.orientation)};

    // 位姿所在帧不是 map 时，先变换到 map 系（同一时间戳）。
    const std::string& frame = msg->header.frame_id;
    if (!frame.empty() && frame != map_frame_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
            map_frame_, frame, rclcpp::Time(msg->header.stamp),
            rclcpp::Duration::from_seconds(0.02));
        const mini_slam::Pose2 frame_in_map{
            tf.transform.translation.x, tf.transform.translation.y,
            mini_slam::ros_node::yaw_from_quat(tf.transform.rotation)};
        map_pose = mini_slam::compose(frame_in_map, map_pose);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "initialpose TF 变换失败（target='%s' source='%s'）：%s",
            map_frame_.c_str(), frame.c_str(), ex.what());
        return;
      }
    }
    map_pose.theta = mini_slam::wrap_angle(map_pose.theta);

    const auto odom = odom_at(stamp);
    if (!odom) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "收到 initialpose 但还没有对应时间的 odom，忽略");
      return;
    }

    enum class Reject { None, NotLocalization, EmptyGraph };
    Reject reject = Reject::None;
    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      if (mapper_->mode() != mini_slam::Mapper::Mode::Localization) {
        reject = Reject::NotLocalization;
      } else if (mapper_->graph().size() == 0) {
        reject = Reject::EmptyGraph;
      } else {
        mapper_->initialize_localization(map_pose, *odom);
        // 立即更新 TF 快照，不等下一帧扫描。时间戳用 initialpose 自身的
        // header.stamp（0 时退回 now()），避免拿墙钟时间给仿真数据打戳。
        last_map_odom_ =
            mini_slam::compose(map_pose, mini_slam::inverse(*odom));
        last_map_odom_.theta =
            mini_slam::wrap_angle(last_map_odom_.theta);
        has_tf_ = true;
        last_tf_stamp_ = msg->header.stamp;
        if (mini_slam::ros_node::is_zero_stamp(last_tf_stamp_)) {
          last_tf_stamp_ = now();
        }
        has_tf_stamp_ = true;
        loc_warn_count_ = 0;
      }
    }
    if (reject == Reject::NotLocalization) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "initialpose 只在 localization 模式生效");
    } else if (reject == Reject::EmptyGraph) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "initialpose 被忽略：图为空，请先 load_graph");
    }
  }

  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    auto odom = odom_at(stamp);
    if (!odom) return;

    // 激光外参：把端点从扫描帧变到 base_frame。只取平面 x/y/yaw，
    // z/roll/pitch 投影掉（2D SLAM 不做三维补偿）。
    std::optional<mini_slam::Pose2> scan_to_base;
    const std::string& scan_frame = msg->header.frame_id;
    if (!scan_frame.empty() && scan_frame != base_frame_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
            base_frame_, scan_frame, rclcpp::Time(msg->header.stamp),
            rclcpp::Duration::from_seconds(0.02));
        scan_to_base = mini_slam::Pose2{
            tf.transform.translation.x, tf.transform.translation.y,
            mini_slam::ros_node::yaw_from_quat(tf.transform.rotation)};
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "扫描 TF 变换失败（target='%s' source='%s'）：%s；丢弃该帧",
            base_frame_.c_str(), scan_frame.c_str(), ex.what());
        return;  // fail-loud：缺 TF 不静默当恒等
      }
    }

    // ROS LaserScan → core LaserScan（点已变换到 base_frame）。
    // 先用原始 range 剔除无效回波（NaN/量程外），有效点再套外参，避免
    // 对无效测量做无意义的 TF 变换。
    mini_slam::LaserScan scan;
    scan.stamp = stamp;
    scan.points.reserve(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r <= static_cast<float>(msg->range_min) ||
          r > static_cast<float>(msg->range_max)) {
        continue;
      }
      const double ang =
          msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      mini_slam::Vec2 p{r * std::cos(ang), r * std::sin(ang)};
      if (scan_to_base) p = mini_slam::transform(*scan_to_base, p);
      scan.points.push_back(p);
    }

    // 异步队列：回调只入队（限长，溢出丢最旧），处理在工作线程
    bool dropped_warn = false;
    int dropped = 0;
    {
      std::lock_guard<std::mutex> lock(proc_mutex_);
      if (static_cast<int>(proc_queue_.size()) >= queue_size_) {
        proc_queue_.pop_front();
        ++dropped_;
        dropped = dropped_;
        dropped_warn = (dropped_ % 50 == 1);
      }
      proc_queue_.push_back(PendingScan{std::move(scan), *odom});
    }
    if (dropped_warn) {
      RCLCPP_WARN(get_logger(),
                  "处理队列溢出，累计丢弃 %d 帧（处理跟不上输入频率）", dropped);
    }
    proc_cv_.notify_one();
  }

  void worker_loop() {
    while (!stopping_) {
      PendingScan item;
      {
        std::unique_lock<std::mutex> lock(proc_mutex_);
        proc_cv_.wait(lock, [this] { return stopping_ || !proc_queue_.empty(); });
        if (stopping_) break;
        item = std::move(proc_queue_.front());
        proc_queue_.pop_front();
      }
      process_one(item.scan, item.odom);
    }
  }

  void process_one(const mini_slam::LaserScan& scan,
                   const mini_slam::Pose2& odom) {
    bool loop_closed = false;
    int loop_id = -1;
    int loops = 0;
    bool loc_warn = false;
    int loc_warn_count = 0;
    {
      // worker 的 mapper_/last_map_odom_/has_tf_/loc_warn_count_ 访问统一加锁。
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      if (mapper_->mode() == mini_slam::Mapper::Mode::Localization) {
        // 定位模式：图冻结，只做匹配跟踪，TF 从修正位姿直接构造
        const auto loc = mapper_->localize(scan, odom);
        if (!loc.valid) {
          ++loc_warn_count_;
          if ((loc_warn_count_ - 1) % 50 == 0) {
            loc_warn = true;
            loc_warn_count = loc_warn_count_;
          }
        }
        last_map_odom_ = mini_slam::compose(loc.pose, mini_slam::inverse(odom));
        last_map_odom_.theta = mini_slam::wrap_angle(last_map_odom_.theta);
      } else {
        const auto result = mapper_->process_scan(scan, odom);
        if (result.loop_closed) {
          loop_closed = true;
          loop_id = result.id;
          loops = mapper_->stats().loop_closures;
        }
        last_map_odom_ = mapper_->map_odom_tf();
      }
      // TF 打戳策略：latest processed scan timestamp。
      // 两种模式都在"这一帧确实处理完"后更新；publish_tf 只读这个时间，
      // 不再用 now()——否则在仿真/回放里 TF 会超前于实际数据。
      last_tf_stamp_ = mini_slam::ros_node::stamp_from_seconds(scan.stamp);
      if (mini_slam::ros_node::is_zero_stamp(last_tf_stamp_)) {
        // 扫描时间戳缺失/为 0 时不能用 0 发布 TF：tf2 会把它当作
        // 无效/极旧数据。与 initialpose_callback 一致，退回 now()。
        last_tf_stamp_ = now();
      }
      has_tf_stamp_ = true;
      has_tf_ = true;
    }
    // 锁外做日志，避免持锁时被日志后端阻塞。
    if (loc_warn) {
      RCLCPP_WARN(get_logger(), "定位匹配不可信（累计 %d 次），退回里程计",
                  loc_warn_count);
    }
    if (loop_closed) {
      RCLCPP_INFO(get_logger(), "闭环 @ 节点 %d（第 %d 次）", loop_id, loops);
    }
  }

  // 运行时切模式：图保留（localization ↔ mapping）。调用方必须持 mapper_mutex_。
  void apply_mode(const std::string& mode_str, const mini_slam::Params& p) {
    if (!mapper_) {
      mapper_ = std::make_unique<mini_slam::Mapper>(p);
    }
    loc_warn_count_ = 0;  // 切模式后定位告警计数不残留
    if (mode_str == "localization") {
      mapper_->set_mode(mini_slam::Mapper::Mode::Localization);
      RCLCPP_INFO(get_logger(), "模式：localization（图冻结，仅位姿跟踪）");
    } else {
      mapper_->set_mode(mini_slam::Mapper::Mode::Mapping);
      RCLCPP_INFO(get_logger(), "模式：mapping");
    }
  }

  // 调用方必须持 mapper_mutex_。
  long total_scan_points_locked() const {
    long s = 0;
    for (const auto& v : mapper_->graph().vertices()) {
      if (v.scan) s += static_cast<long>(v.scan->points.size());
    }
    return s;
  }

  static std::string validate_runtime_parameter(
      const rclcpp::Parameter& param) {
    const std::string& n = param.get_name();
    const auto bad = [&n](auto value, const char* why) {
      return std::format("参数 '{}'={} 非法：{}", n, value, why);
    };
    if (n == "mode") {
      const auto v = param.as_string();
      if (v != "mapping" && v != "localization")
        return bad(v, "只接受 mapping / localization");
    } else if (n == "resolution") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "processing_queue_size") {
      const auto v = param.as_int();
      if (v <= 0) return bad(v, "必须 > 0");
    } else if (n == "map_update_interval") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "transform_publish_period") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "minimum_travel_distance") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v < 0.0) return bad(v, "必须 >= 0");
    } else if (n == "minimum_travel_heading") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v < 0.0) return bad(v, "必须 >= 0");
    } else if (n == "scan_range_max") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "loop_search_maximum_distance") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "loop_match_minimum_chain_size") {
      const auto v = param.as_int();
      if (v < 1) return bad(v, "必须 >= 1");
    } else if (n == "loop_match_minimum_response_coarse" ||
               n == "loop_match_minimum_response_fine") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v < 0.0 || v > 1.0)
        return bad(v, "必须在 [0,1]");
    } else if (n == "sparsify_every") {
      const auto v = param.as_int();
      if (v < 1) return bad(v, "必须 >= 1（取模除零）");
    } else if (n == "sparsify_keep_radius") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "sparsify_min_keep_recent") {
      const auto v = param.as_int();
      if (v < 0) return bad(v, "必须 >= 0");
    } else if (n == "localization_window") {
      const double v = param.as_double();
      if (!std::isfinite(v) || v <= 0.0) return bad(v, "必须 > 0");
    } else if (n == "interactive_marker_max_vertices") {
      const auto v = param.as_int();
      if (v < 1) return bad(v, "必须 >= 1（抽样 stride 除零）");
    } else if (n == "scan_topic" || n == "odom_topic" || n == "map_frame" ||
               n == "odom_frame" || n == "base_frame" ||
               n == "enable_interactive_markers") {
      // 这些参数绑定在订阅/发布器/TF 树/InteractiveMarkerServer 上，运行期
      // 重建代价与风险都高；明确拒绝，而不是"接受但静默不生效"。
      return std::format(
          "参数 '{}' 运行时不支持修改，请通过 launch 参数重启节点", n);
    }
    return {};
  }

  void publish_map() {
    // 地图 timer 通常单线程；参数回调重建 timer 时可能短暂出现两个 timer
    // 实例，用独立缓存锁把它们串行化（不碰 mapper_mutex_，不阻塞 worker）。
    std::lock_guard<std::mutex> publish_lock(map_cache_mutex_);

    std::shared_ptr<mini_slam::OccupancyMap> cached_map;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> cached_grid;
    std::unique_ptr<mini_slam::PoseGraph> snapshot;
    MapRenderKey key;
    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      // 定位模式：地图来自 map_server（挂载的 yaml），本节点不发 /map
      if (mapper_->mode() == mini_slam::Mapper::Mode::Localization) return;

      key.mode = mapper_->mode();
      key.vertices = mapper_->graph().vertices().size();
      key.edges = mapper_->graph().edges().size();
      key.nodes = mapper_->stats().nodes;
      key.solves = mapper_->stats().solves;
      key.resolution = resolution_;
      // 地图渲染用 map_range_max（不是 scan_range_max：后者只管匹配/入图）。
      key.range_max = mapper_->params().map_range_max;
      key.margin = mapper_->params().map_margin;
      key.prefix = map_file_;  // 运行时可能改，snapshot 到 key 里
      key.generation = map_generation_;

      if (map_cache_key_ && *map_cache_key_ == key && map_cache_ &&
          map_grid_cache_) {
        // 命中：只拿 shared_ptr 快照，绝不复制整张图。
        cached_map = map_cache_;
        cached_grid = map_grid_cache_;
      } else {
        // 未命中：持锁只做"图快照"，渲染放到锁外。
        snapshot = std::make_unique<mini_slam::PoseGraph>(mapper_->graph());
      }
    }

    if (cached_grid) {
      // 地图文件只在未命中时写；但 yaml 被删/上次写失败时补写一次，
      // 保证 map_output_prefix 的产物最终存在。
      if (!key.prefix.empty()) {
        std::error_code ec;
        const bool yaml_missing =
            !std::filesystem::exists(key.prefix + ".yaml", ec);
        if (yaml_missing || ec) {
          std::string err;
          if (!mini_slam::map_output::write_map_files(key.prefix, *cached_map,
                                                      &err)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "地图文件写入失败: %s", err.c_str());
          }
        }
      }
      // header.stamp 仍是"当前发布时刻"：地图是定时快照，不是扫描时刻。
      cached_grid->header.stamp = now();
      map_pub_->publish(*cached_grid);
      return;
    }

    // 未命中：锁外渲染（可能耗时，不能占着 mapper_mutex_）。
    // Phase 4B：交给增量 builder —— 只有新增顶点时做增量射线更新；
    // generation 变化（/load_graph）或 SPA/稀疏化/参数变化时全量重建。
    const bool force_full = key.generation != map_builder_generation_;
    const auto& rendered = map_builder_.update(
        *snapshot, key.resolution, key.range_max, key.margin, force_full);
    map_builder_generation_ = key.generation;
    if (rendered.width == 0) return;
    auto map = std::make_shared<mini_slam::OccupancyMap>(rendered);

    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.stamp = now();
    grid->header.frame_id = map_frame_;
    grid->info.resolution = static_cast<float>(map->resolution);
    grid->info.width = static_cast<std::uint32_t>(map->width);
    grid->info.height = static_cast<std::uint32_t>(map->height);
    grid->info.origin.position.x = map->origin.x;
    grid->info.origin.position.y = map->origin.y;
    grid->info.origin.orientation.w = 1.0;
    grid->data.assign(map->data.begin(), map->data.end());
    map_pub_->publish(*grid);

    if (!key.prefix.empty()) {
      std::string err;
      if (!mini_slam::map_output::write_map_files(key.prefix, *map, &err)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "地图文件写入失败: %s", err.c_str());
      }
    }

    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      // 渲染期间 /load_graph 可能整体换了图：旧结果不能进缓存。
      if (map_generation_ == key.generation) {
        map_cache_key_ = std::move(key);
        map_cache_ = std::move(map);
        map_grid_cache_ = std::move(grid);
      }
    }
  }

  void publish_tf() {
    // TF 时间戳策略：latest processed scan timestamp。
    // 只发布"最近一帧真正处理完的扫描"的时间，避免 now() 在仿真/回放中
    // 领先数据时间导致 TF 外推。单调性保护：候选时间不得小于已发布值，
    // 否则沿用上次（防 TF_OLD_DATA）。
    mini_slam::Pose2 map_odom;
    builtin_interfaces::msg::Time stamp;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      valid = has_tf_ && has_tf_stamp_;
      if (valid) {
        map_odom = last_map_odom_;
        stamp = last_tf_stamp_;
        if (has_published_tf_stamp_ &&
            mini_slam::ros_node::stamp_ns(stamp) <
                mini_slam::ros_node::stamp_ns(last_published_tf_stamp_)) {
          stamp = last_published_tf_stamp_;
        }
        last_published_tf_stamp_ = stamp;
        has_published_tf_stamp_ = true;
      }
    }
    if (!valid) return;  // 还没有任何处理过的时间戳：不发 TF

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform.translation.x = map_odom.x;
    t.transform.translation.y = map_odom.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = mini_slam::ros_node::yaw_to_quat(map_odom.theta);
    tf_broadcaster_->sendTransform(t);
  }

  // ---------- 成员 ----------
  std::unique_ptr<mini_slam::Mapper> mapper_;
  // 保护 mapper_ 以及 last_map_odom_ / has_tf_ / loc_warn_count_。
  // 不递归：任何路径都不得在已持锁时再次加锁。
  std::mutex mapper_mutex_;

  double resolution_{0.05};
  double map_update_interval_{5.0};
  double tf_publish_period_{0.05};
  std::string odom_frame_, map_frame_, base_frame_, scan_topic_, odom_topic_;
  std::string map_file_;
  std::string graph_path_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  // 顺序必须 buffer 在 listener 之前：析构按声明逆序进行，listener 先释放
  // （cancel 自己的 executor 并 join 专用线程），buffer 后释放。
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr map_timer_, tf_timer_;

  std::mutex odom_mutex_;
  mini_slam::OdomBuffer odom_buffer_;

  // 异步处理队列（阶段5）：回调入队 → worker 线程处理
  std::deque<PendingScan> proc_queue_;
  std::mutex proc_mutex_;
  std::condition_variable proc_cv_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  int queue_size_{10};
  int dropped_{0};

  int loc_warn_count_{0};
  mini_slam::Pose2 last_map_odom_;
  bool has_tf_{false};

  // TF 时间戳：最近一帧处理完的扫描（或 /initialpose 的时刻）。
  // 均由 mapper_mutex_ 保护。
  builtin_interfaces::msg::Time last_tf_stamp_;
  bool has_tf_stamp_{false};
  builtin_interfaces::msg::Time last_published_tf_stamp_;
  bool has_published_tf_stamp_{false};

  // 地图渲染缓存（map_cache_mutex_ 保护）。/load_graph 成功时递增
  // map_generation_ 强制失效。
  std::mutex map_cache_mutex_;
  std::optional<MapRenderKey> map_cache_key_;
  std::shared_ptr<mini_slam::OccupancyMap> map_cache_;
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_grid_cache_;
  std::uint64_t map_generation_{0};
  // 增量地图渲染器（仅 map_cache_mutex_ 保护下访问）。generation 不一致时
  // 强制全量重建，避免 /load_graph 换图后沿用旧栅格。
  mini_slam::IncrementalMapBuilder map_builder_;
  std::uint64_t map_builder_generation_{0};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stats_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;

  // ---------- 交互式 marker（Phase 4C）----------
  // 声明顺序在 ROS 接口成员之后：析构逆序时 timer/server 先销毁，此时
  // node interfaces 仍然有效（InteractiveMarkerServer 内部持有它们的
  // shared_ptr，析构里还要 clear + applyChanges 走一遍发布器）。
  // marker_server_ 只在 enable_interactive_markers=true 时非空。
  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  // 重建缓存键（marker_timer_ 与参数回调都只经 mapper_mutex_ 访问）。
  std::optional<MarkerKey> marker_key_;
  bool enable_interactive_markers_{false};
  int interactive_marker_max_vertices_{200};
};
