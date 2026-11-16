#pragma once
// ============================================================================
// 启动期参数集中校验（纯 C++，便于单元测试）。
//
// 这些参数一旦非法会在运行期制造除零 / 空队列 pop / 负尺寸等 UB：
//   processing_queue_size <= 0  → 空 deque 上 pop_front
//   sparsify_every / loop_attempt_interval == 0 → 取模除零
//   resolution <= 0 → 栅格/得分场除零
//   匹配步长 <= 0 → 相关匹配除零
// 返回空字符串表示合法；否则返回带参数名与当前值的可读错误。
// ============================================================================
#include <cmath>
#include <format>
#include <string>
#include <utility>

#include "mini_slam/types.hpp"

namespace mini_slam {

// 节点层（非算法核）参数，和 Params 一起做整体校验。
struct NodeParams {
  int processing_queue_size{10};
  double map_update_interval{5.0};
  double transform_publish_period{0.05};
  std::string scan_topic{"scan"};
  std::string odom_topic{"odom"};
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  // 交互式 marker 最多显示的节点数（RViz 拖拽用）；抽样逻辑按它算 stride。
  int interactive_marker_max_vertices{200};
};

inline std::string validate_parameters(const Params& p, const NodeParams& n) {
  const auto bad = [](std::string name, auto value, const char* why) {
    return std::format("参数 '{}'={} 非法：{}", std::move(name), value, why);
  };
  const auto finite = [](double v) { return std::isfinite(v); };

  // ---- 节点参数 ----
  if (n.processing_queue_size <= 0)
    return bad("processing_queue_size", n.processing_queue_size,
               "必须 > 0（否则空队列 pop_front）");
  if (!finite(n.map_update_interval) || n.map_update_interval <= 0.0)
    return bad("map_update_interval", n.map_update_interval, "必须 > 0");
  if (!finite(n.transform_publish_period) || n.transform_publish_period <= 0.0)
    return bad("transform_publish_period", n.transform_publish_period,
               "必须 > 0");
  if (n.scan_topic.empty()) return "参数 'scan_topic' 不能为空";
  if (n.odom_topic.empty()) return "参数 'odom_topic' 不能为空";
  if (n.map_frame.empty()) return "参数 'map_frame' 不能为空";
  if (n.odom_frame.empty()) return "参数 'odom_frame' 不能为空";
  if (n.base_frame.empty()) return "参数 'base_frame' 不能为空";
  if (n.interactive_marker_max_vertices < 1)
    return bad("interactive_marker_max_vertices",
               n.interactive_marker_max_vertices,
               "必须 >= 1（抽样 stride 除零）");

  // ---- 栅格/匹配 ----
  if (!finite(p.resolution) || p.resolution <= 0.0)
    return bad("resolution", p.resolution, "必须 > 0（栅格除零）");
  if (!finite(p.scan_range_max) || p.scan_range_max <= 0.0)
    return bad("scan_range_max", p.scan_range_max, "必须 > 0");
  if (!finite(p.map_range_max) || p.map_range_max <= 0.0)
    return bad("map_range_max", p.map_range_max, "必须 > 0");
  if (!finite(p.map_margin) || p.map_margin < 0.0)
    return bad("map_margin", p.map_margin, "必须 >= 0");
  if (p.blur_radius < 0)
    return bad("blur_radius", p.blur_radius, "必须 >= 0");

  if (!finite(p.min_travel_distance) || p.min_travel_distance < 0.0)
    return bad("minimum_travel_distance", p.min_travel_distance,
               "必须 >= 0");
  if (!finite(p.min_travel_heading) || p.min_travel_heading < 0.0)
    return bad("minimum_travel_heading", p.min_travel_heading, "必须 >= 0");

  if (!finite(p.match_window_lin) || p.match_window_lin <= 0.0)
    return bad("match_window_lin", p.match_window_lin, "必须 > 0");
  if (!finite(p.match_window_ang) || p.match_window_ang <= 0.0)
    return bad("match_window_ang", p.match_window_ang, "必须 > 0");
  if (!finite(p.coarse_lin_step) || p.coarse_lin_step <= 0.0)
    return bad("coarse_lin_step", p.coarse_lin_step,
               "必须 > 0（匹配步长除零）");
  if (!finite(p.coarse_ang_step) || p.coarse_ang_step <= 0.0)
    return bad("coarse_ang_step", p.coarse_ang_step,
               "必须 > 0（匹配步长除零）");
  if (!finite(p.loop_search_distance) || p.loop_search_distance <= 0.0)
    return bad("loop_search_maximum_distance", p.loop_search_distance,
               "必须 > 0");
  if (!finite(p.loop_window_lin) || p.loop_window_lin <= 0.0)
    return bad("loop_window_lin", p.loop_window_lin, "必须 > 0");
  if (!finite(p.loop_window_ang) || p.loop_window_ang <= 0.0)
    return bad("loop_window_ang", p.loop_window_ang, "必须 > 0");
  if (p.loop_min_chain_size < 1)
    return bad("loop_match_minimum_chain_size", p.loop_min_chain_size,
               "必须 >= 1");
  if (!finite(p.loop_min_response_coarse) ||
      p.loop_min_response_coarse < 0.0 || p.loop_min_response_coarse > 1.0)
    return bad("loop_match_minimum_response_coarse",
               p.loop_min_response_coarse, "必须在 [0,1]");
  if (!finite(p.loop_min_response_fine) || p.loop_min_response_fine < 0.0 ||
      p.loop_min_response_fine > 1.0)
    return bad("loop_match_minimum_response_fine", p.loop_min_response_fine,
               "必须在 [0,1]");
  if (!finite(p.loop_max_variance) || p.loop_max_variance < 0.0)
    return bad("loop_max_variance", p.loop_max_variance, "必须 >= 0");
  if (!finite(p.match_min_response) || p.match_min_response < 0.0 ||
      p.match_min_response > 1.0)
    return bad("match_min_response", p.match_min_response, "必须在 [0,1]");
  if (!finite(p.running_scan_distance) || p.running_scan_distance < 0.0)
    return bad("running_scan_distance", p.running_scan_distance, "必须 >= 0");
  if (p.loop_max_edges < 1)
    return bad("loop_max_edges", p.loop_max_edges, "必须 >= 1");
  if (p.loop_min_node_gap < 0)
    return bad("loop_min_node_gap", p.loop_min_node_gap, "必须 >= 0");
  if (p.loop_attempt_interval < 1)
    return bad("loop_attempt_interval", p.loop_attempt_interval,
               "必须 >= 1（取模除零）");

  if (!finite(p.odom_sigma_xy) || p.odom_sigma_xy <= 0.0)
    return bad("odom_sigma_xy", p.odom_sigma_xy, "必须 > 0");
  if (!finite(p.odom_sigma_theta) || p.odom_sigma_theta <= 0.0)
    return bad("odom_sigma_theta", p.odom_sigma_theta, "必须 > 0");
  if (!finite(p.match_sigma_min) || p.match_sigma_min <= 0.0)
    return bad("match_sigma_min", p.match_sigma_min, "必须 > 0");
  if (!finite(p.match_sigma_max) || p.match_sigma_max < p.match_sigma_min)
    return bad("match_sigma_max", p.match_sigma_max,
               "必须 >= match_sigma_min");

  // ---- 定位 / 稀疏化 ----
  if (!finite(p.localization_window) || p.localization_window <= 0.0)
    return bad("localization_window", p.localization_window, "必须 > 0");
  if (!finite(p.loc_min_response) || p.loc_min_response < 0.0 ||
      p.loc_min_response > 1.0)
    return bad("loc_min_response", p.loc_min_response, "必须在 [0,1]");
  if (!finite(p.sparsify_keep_radius) || p.sparsify_keep_radius <= 0.0)
    return bad("sparsify_keep_radius", p.sparsify_keep_radius, "必须 > 0");
  if (p.sparsify_min_keep_recent < 0)
    return bad("sparsify_min_keep_recent", p.sparsify_min_keep_recent,
               "必须 >= 0");
  if (p.sparsify_every < 1)
    return bad("sparsify_every", p.sparsify_every,
               "必须 >= 1（取模除零）");
  if (p.running_scan_max_count < 1)
    return bad("running_scan_max_count", p.running_scan_max_count,
               "必须 >= 1");

  return {};
}

}  // namespace mini_slam
