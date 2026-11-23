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
// LocalizerCore —— 定位核心状态机（ROS 无关，可独立单测）
//
//   UNINIT ──initial_pose──► TRACKING ──连续拒绝──► DEGRADED ──超时──► LOST
//                              ▲  │                    │
//                              └──┼────────────────────┘ 匹配恢复
//                                 └─► GLOBAL（外部执行全局搜索后回填）
//
// 每帧扫描：odom 预测 → 匹配器 → 三道门（内点率 / 马氏距离 / 协方差上限）
// 拒绝则纯 odom 外推，绝不接受坏匹配；连续拒绝进入 DEGRADED。
// ============================================================================
#include <functional>
#include <memory>

#include "scan_localizer/matchers.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc
{

enum class LocState { UNINIT, TRACKING, DEGRADED, LOST, GLOBAL };

struct CoreConfig
{
    // 门限
  double min_inlier_ratio{0.60};
  double min_score{0.35};
  double mahal_gate{9.0};          // χ²(3) 3σ ≈ 9.35
  double max_cov_eig{1.0};        // 匹配协方差最小特征值上限 (m²)
    // 预测协方差（odom 滑移模型）：覆盖 ~2% 尺度滑移
  double odom_trans_var_per_m{4e-4};     // (0.02m/m)²
  double odom_rot_var_per_rad{1e-3};
  double odom_rot_var_per_m{2e-3};
    // IMU 融合（可选）：陀螺积分 Δyaw 替换轮式 Δθ（平移仍取轮式）
  bool use_imu_yaw{false};
  double imu_rot_var_per_rad{2e-5};     // MEMS 陀螺短时精度高两个数量级
    // 状态机
  int degraded_after{5};           // 连续拒绝 N 帧进 DEGRADED
  int lost_after{60};              // DEGRADED 持续 N 帧进 LOST
    // 修正平滑（防跳变）
  double slew_alpha{0.7};          // 每帧最多吸收 70% 新息
    // 匹配器选项（跟踪窗口）
  MatchOptions match_opt{};
};

struct CoreOutput
{
  Pose2 pose;                  // map 系 base 位姿
  Cov3 covariance{Cov3::Identity()};
  LocState state{LocState::UNINIT};
  bool accepted{false};        // 本帧匹配是否被采纳
  double score{0.0}, inlier_ratio{0.0};
  int reject_streak{0};
};

class LocalizerCore {
public:
  explicit LocalizerCore(CoreConfig cfg = {});

  void set_map(std::shared_ptr<const MapField> field);
  void set_matcher(std::shared_ptr<const Matcher> m);
  void set_initial_pose(const Pose2 & p);
  void set_options(const CoreConfig & cfg) {cfg_ = cfg;}     // 热更新（fine mode 等）
  void request_global()      // 手动触发：下一帧扫描走全局搜索
  {
    if (state_ == LocState::TRACKING || state_ == LocState::DEGRADED) {
      state_ = LocState::LOST;
      reject_streak_ = 0;
    }
  }

    // 全局搜索回调（UNINIT/LOST 时由外层驱动：PF 或 correlative 全局）
  using GlobalFn =
    std::function<bool(const Scan &, Pose2 & out, double & score)>;
  void set_global_fn(GlobalFn fn);

    // odom：odom 系 base 位姿（消息时刻）
  void on_odom(const Pose2 & odom_pose);

    // IMU：陀螺积分 yaw（任意固定系；帧间差分消除慢漂移）
    // 每条 IMU 消息调用一次；帧间增量替换轮式 Δθ
  void on_imu_yaw(double integrated_yaw);

    // 每帧扫描处理；scan 为传感器系（= base 系，安装偏移由外层处理）
  CoreOutput on_scan(const Scan & scan);

  LocState state() const {return state_;}
  Pose2 pose() const {return last_pose_;}
    // map→odom TF（供 ROS 层发布）
  Pose2 map_to_odom() const;

private:
  CoreConfig cfg_;
  std::shared_ptr<const MapField> field_;
  std::shared_ptr<const Matcher> matcher_;
  GlobalFn global_fn_;

  LocState state_{LocState::UNINIT};
  Pose2 last_pose_{};          // map 系
  Cov3 last_cov_{Cov3::Identity()};
  Pose2 last_odom_{};         // odom 系
  Pose2 odom_at_last_scan_{};
  bool has_odom_{false};
    // IMU 状态
  double imu_yaw_{0.0};
  double imu_yaw_at_last_scan_{0.0};
  bool has_imu_{false};
  bool imu_consumed_{true};     // true=上一帧已消费，新帧需重新积累
  int reject_streak_{0};
  bool first_fix_{false};
};

}  // namespace sloc
