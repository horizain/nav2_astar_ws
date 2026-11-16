#pragma once
// ============================================================================
// 里程计时间缓冲：按扫描时间戳在里程计轨迹上插值。
// 纯 C++（只依赖 Pose2），便于脱离 ROS 做端点/跨越 ±π 的单元测试。
//
// 约定（对应 ROS 节点 odom_at 的三种情况）：
//   stamp < 最早样本      → 返回最早位姿（外推没有意义，夹住）
//   stamp > 最新样本      → 返回最新位姿（不是最早！）
//   stamp 落在区间内      → 线性插值；角度按最短差插值
// 返回位姿的 theta 统一 wrap 到 (-π,π]。
// ============================================================================
#include <algorithm>
#include <cstddef>
#include <deque>
#include <optional>

#include "mini_slam/types.hpp"

namespace mini_slam {

struct OdomSample {
  double stamp{0.0};
  Pose2 pose;
};

class OdomBuffer {
 public:
  explicit OdomBuffer(std::size_t capacity = 400) : capacity_(capacity) {}

  // 追加样本；乱序时间戳按时间插入，保证 at() 的夹逼前提。
  void push(double stamp, const Pose2& pose) {
    if (!samples_.empty() && stamp < samples_.back().stamp) {
      auto it = std::lower_bound(
          samples_.begin(), samples_.end(), stamp,
          [](const OdomSample& s, double v) { return s.stamp < v; });
      samples_.insert(it, OdomSample{stamp, pose});
    } else {
      samples_.push_back(OdomSample{stamp, pose});
    }
    while (samples_.size() > capacity_) samples_.pop_front();
  }

  bool empty() const { return samples_.empty(); }
  std::size_t size() const { return samples_.size(); }
  void clear() { samples_.clear(); }

  std::optional<Pose2> at(double stamp) const {
    if (samples_.empty()) return std::nullopt;

    Pose2 p;
    if (stamp <= samples_.front().stamp) {
      p = samples_.front().pose;
    } else if (stamp >= samples_.back().stamp) {
      // 扫描晚于最新 odom：用最新位姿（回放时 scan 常常略晚于最后一帧 odom）
      p = samples_.back().pose;
    } else {
      const auto b = std::lower_bound(
          samples_.begin(), samples_.end(), stamp,
          [](const OdomSample& s, double v) { return s.stamp < v; });
      const auto a = b - 1;
      const double span = b->stamp - a->stamp;
      const double alpha = span > 0.0 ? (stamp - a->stamp) / span : 0.0;
      p.x = a->pose.x + alpha * (b->pose.x - a->pose.x);
      p.y = a->pose.y + alpha * (b->pose.y - a->pose.y);
      p.theta = wrap_angle(
          a->pose.theta + alpha * wrap_angle(b->pose.theta - a->pose.theta));
      return p;  // 插值结果已 wrap
    }
    p.theta = wrap_angle(p.theta);  // 端点也返回一致的角度表示
    return p;
  }

 private:
  std::size_t capacity_;
  std::deque<OdomSample> samples_;
};

}  // namespace mini_slam
