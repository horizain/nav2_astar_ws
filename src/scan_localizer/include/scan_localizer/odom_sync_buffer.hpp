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

#pragma once
// ============================================================================
// OdomSyncBuffer & GlobalRetryGate —— 纯 C++20 里程计时间同步、异步外推与冷却节流
//
// 从 LocalizerNode 剥离为无 ROS 依赖组件，支持独立 gtest 单元测试：
// 1) 环形缓冲插值：按任意扫描时间戳精确插值 SE(2) 位姿与速度（供激光去畸变）；
// 2) 异步重定位结果外推：计算 T_odom(t_submit)^-1 * T_odom(t_now)，将后台 B&B
//    搜索返回的主峰与全部 alternates 补偿至当前扫描时刻；
// 3) 静止重试冷却门控：防止 LOST/GLOBAL 原地静止时每帧重复触发全局搜索打满 CPU。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

#include "scan_localizer/matchers.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc
{

struct OdomSample
{
  double stamp_s{0.0};
  Pose2 pose{};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

class OdomSyncBuffer {
public:
  explicit OdomSyncBuffer(std::size_t max_size = 200)
  : max_size_(std::max<std::size_t>(2, max_size)) {}

  void clear()
  {
    buf_.clear();
  }

  bool empty() const {return buf_.empty();}
  std::size_t size() const {return buf_.size();}

  double latest_stamp() const
  {
    return buf_.empty() ? 0.0 : buf_.back().stamp_s;
  }

  bool latest_pose(Pose2 & out) const
  {
    if (buf_.empty()) {return false;}
    out = buf_.back().pose;
    return true;
  }

  void push(
    double stamp_s, const Pose2 & pose,
    double vx = 0.0, double vy = 0.0, double wz = 0.0)
  {
    if (!std::isfinite(stamp_s) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta))
    {
      return;
    }
    if (!buf_.empty() && stamp_s < buf_.back().stamp_s) {
      buf_.clear();
    }
    buf_.push_back(OdomSample{
        stamp_s,
        Pose2(pose.x, pose.y, wrap_angle(pose.theta)),
        std::isfinite(vx) ? vx : 0.0,
        std::isfinite(vy) ? vy : 0.0,
        std::isfinite(wz) ? wz : 0.0});
    while (buf_.size() > max_size_) {
      buf_.pop_front();
    }
  }

  // 查询指定时刻的 SE(2) 里程计位姿（线性位置 + 航向最短弧插值）
  bool lookup(
    double stamp_s, Pose2 & out,
    double timeout_s, bool & stale, double & stale_age_s) const
  {
    stale = false;
    stale_age_s = 0.0;
    if (buf_.empty()) {
      return false;
    }
    if (stamp_s > 0.0 && timeout_s > 0.0) {
      const double age = stamp_s - buf_.back().stamp_s;
      if (age > timeout_s) {
        stale = true;
        stale_age_s = age;
      }
    }
    if (buf_.size() == 1 || stamp_s <= buf_.front().stamp_s) {
      out = buf_.front().pose;
      return true;
    }
    if (stamp_s >= buf_.back().stamp_s) {
      out = buf_.back().pose;
      return true;
    }
    for (std::size_t i = 0; i + 1 < buf_.size(); ++i) {
      const auto & a = buf_[i];
      const auto & b = buf_[i + 1];
      if (stamp_s >= a.stamp_s && stamp_s <= b.stamp_s) {
        const double dt = b.stamp_s - a.stamp_s;
        const double alpha = (dt > 1e-9) ? std::clamp((stamp_s - a.stamp_s) / dt, 0.0, 1.0) : 0.0;
        const double dth = wrap_angle(b.pose.theta - a.pose.theta);
        out = Pose2(
          a.pose.x + alpha * (b.pose.x - a.pose.x),
          a.pose.y + alpha * (b.pose.y - a.pose.y),
          wrap_angle(a.pose.theta + alpha * dth));
        return true;
      }
    }
    out = buf_.back().pose;
    return true;
  }

  // 查询 stamp_s 处的本体系速度估计（优先采用里程计消息速度，缺失时用相邻位姿有限差分）
  bool estimate_twist(
    double stamp_s, double & vx, double & vy, double & wz) const
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
    if (buf_.empty()) {
      return false;
    }
    if (buf_.size() == 1) {
      vx = buf_.back().vx;
      vy = buf_.back().vy;
      wz = buf_.back().wz;
      return true;
    }
    const OdomSample * a_ptr = &buf_[buf_.size() - 2];
    const OdomSample * b_ptr = &buf_.back();
    if (stamp_s < buf_.back().stamp_s) {
      for (std::size_t i = 0; i + 1 < buf_.size(); ++i) {
        if (buf_[i].stamp_s <= stamp_s && stamp_s <= buf_[i + 1].stamp_s) {
          a_ptr = &buf_[i];
          b_ptr = &buf_[i + 1];
          break;
        }
      }
    }
    const double dt = b_ptr->stamp_s - a_ptr->stamp_s;
    if (dt > 1e-4 && dt <= 1.0) {
      if (std::hypot(b_ptr->vx, b_ptr->vy) > 1e-4 || std::abs(b_ptr->wz) > 1e-4) {
        vx = b_ptr->vx;
        vy = b_ptr->vy;
        wz = b_ptr->wz;
      } else {
        const double dx = b_ptr->pose.x - a_ptr->pose.x;
        const double dy = b_ptr->pose.y - a_ptr->pose.y;
        const double dth = wrap_angle(b_ptr->pose.theta - a_ptr->pose.theta);
        const double c = std::cos(a_ptr->pose.theta);
        const double s = std::sin(a_ptr->pose.theta);
        vx = (c * dx + s * dy) / dt;
        vy = (-s * dx + c * dy) / dt;
        wz = dth / dt;
      }
    } else {
      vx = buf_.back().vx;
      vy = buf_.back().vy;
      wz = buf_.back().wz;
    }
    return true;
  }

  // 计算两时刻之间的本体系相对位姿增量：T_odom(from)^-1 * T_odom(to)
  bool compute_delta(
    double from_stamp_s, double to_stamp_s, Pose2 & delta) const
  {
    delta = Pose2{};
    if (buf_.empty() || !std::isfinite(from_stamp_s) || !std::isfinite(to_stamp_s) ||
      to_stamp_s <= from_stamp_s || from_stamp_s < buf_.front().stamp_s - 0.05)
    {
      return false;
    }
    Pose2 p_from{};
    Pose2 p_to{};
    bool stale_dummy = false;
    double age_dummy = 0.0;
    if (!lookup(from_stamp_s, p_from, 0.0, stale_dummy, age_dummy) ||
      !lookup(to_stamp_s, p_to, 0.0, stale_dummy, age_dummy))
    {
      return false;
    }
    delta = p_from.inverse() * p_to;
    delta.theta = wrap_angle(delta.theta);
    return true;
  }

  // 将异步全局搜索返回的位姿与备选假设统一向前外推至当前时刻
  static void extrapolate_global_result(GlobalResult & g, const Pose2 & delta)
  {
    if (!g.valid) {return;}
    g.pose = g.pose * delta;
    g.pose.theta = wrap_angle(g.pose.theta);
    for (auto & alt : g.alternates) {
      alt = alt * delta;
      alt.theta = wrap_angle(alt.theta);
    }
  }

private:
  std::size_t max_size_{200};
  std::deque<OdomSample> buf_;
};

// 异步全局重定位重试冷却门控：
// 防止车辆原地静止处于 LOST/GLOBAL 时每帧无节流重复提交 B&B 搜索打满 CPU；
// 一旦机器人明显移动（>= min_move_m）或冷却到期（>= cooldown_s）即允许立即重试。
class GlobalRetryGate {
public:
  void reset()
  {
    has_last_submit_ = false;
    last_submit_s_ = 0.0;
    has_last_odom_ = false;
    last_odom_ = Pose2{};
  }

  bool should_submit(
    double now_s, bool has_odom, const Pose2 & cur_odom,
    bool worker_busy, double cooldown_s, double min_move_m) const
  {
    if (worker_busy) {
      return false;
    }
    if (!has_last_submit_ || cooldown_s <= 0.0) {
      return true;
    }
    if (now_s < last_submit_s_) {
      // 时间回拨（如重新播包），立即放行
      return true;
    }
    if ((now_s - last_submit_s_) >= cooldown_s) {
      return true;
    }
    if (has_odom && has_last_odom_ && min_move_m > 0.0) {
      const double moved = std::hypot(
        cur_odom.x - last_odom_.x, cur_odom.y - last_odom_.y);
      const double turned = std::abs(wrap_angle(cur_odom.theta - last_odom_.theta));
      if (moved >= min_move_m || turned >= 0.25) {
        return true;
      }
    }
    return false;
  }

  void mark_submitted(double now_s, bool has_odom, const Pose2 & cur_odom)
  {
    has_last_submit_ = true;
    last_submit_s_ = now_s;
    has_last_odom_ = has_odom;
    if (has_odom) {
      last_odom_ = cur_odom;
    }
  }

private:
  bool has_last_submit_{false};
  double last_submit_s_{0.0};
  bool has_last_odom_{false};
  Pose2 last_odom_{};
};

}  // namespace sloc
