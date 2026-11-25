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
//   UNINIT ──initial_pose──► TRACKING ──连续拒绝──► DEGRADED ──连续拒绝──► LOST
//                              ▲  │                    │                     │
//                              └──┼────────────────────┘ 匹配恢复            │
//                                 └─► GLOBAL ◄─────────┴─── request_global() ┘
//                                      (全局搜索 / 多假设验证恢复)
//
// 4 状态各向异性 EKF：[x, y, theta, b_g]^T（在线估计陀螺零偏 b_g），
// 完整 SE(2) 雅可比预测 + Joseph 形式协方差更新 + 4 道门控 +
// 多假设队列 + 滑动窗口主动绑架检测。
// ============================================================================
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "scan_localizer/matchers.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc
{

using Cov4 = Eigen::Matrix4d;

enum class LocState { UNINIT, TRACKING, DEGRADED, LOST, GLOBAL };

struct CoreConfig
{
  // 门限
  double min_inlier_ratio{0.60};
  double min_score{0.35};
  double mahal_gate{9.0};          // χ²(3) 3σ ≈ 9.35
  double max_cov_eig{1.0};         // 匹配协方差最小特征值上限 (m²)
  // 预测协方差（odom 滑移模型）：覆盖 ~2% 尺度滑移与原地旋转偏心/畸变平移不确定度
  double odom_trans_var_per_m{4e-4};     // (0.02m/m)²
  double odom_trans_var_per_rad{2e-3};   // 原地旋转引起的横向平移过程噪声 (m²/rad)
  double odom_rot_var_per_rad{1e-3};
  double odom_rot_var_per_m{2e-3};
  // IMU 融合（可选）：陀螺积分 Δyaw 替换轮式 Δθ（平移仍取轮式）
  bool use_imu_yaw{false};
  double imu_rot_var_per_rad{2e-5};      // MEMS 陀螺短时精度高两个数量级
  // 4D EKF 陀螺零偏估计（状态 [x, y, theta, b_g]^T）
  double gyro_bias_init_var{1e-2};       // 初始零偏方差 (rad/s)²
  double gyro_bias_rw_var_per_s{1e-6};   // 零偏随机游走过程噪声 (rad²/s³)
  double max_gyro_bias{0.5};             // 零偏物理限幅 (rad/s)
  // 无里程计模式（无 /odom 时以恒定平移 + 可选 IMU 航向预测）
  bool allow_odomless{true};
  double odomless_trans_var{0.04};       // 无 odom 每帧平移过程噪声 (m²)
  double odomless_rot_var{0.01};         // 无 odom 且无 IMU 每帧航向过程噪声 (rad²)
  // 状态机
  int degraded_after{5};           // 连续拒绝 N 帧进 DEGRADED
  int lost_after{60};              // 累计连续拒绝达到 N 帧进 LOST（应 >= degraded_after）
  // 修正平滑 / 可选步长限幅
  double slew_alpha{0.7};          // 保留兼容字段（已由各向异性 EKF 取代）
  double max_update_trans{0.0};    // 可选单帧最大平移修正 (m)，<=0 不限幅
  double max_update_rot{0.0};      // 可选单帧最大航向修正 (rad)，<=0 不限幅
  // 多假设队列
  int hyp_confirm_frames{2};       // 候选假设连续通过验证帧数后晋升
  double hyp_min_score{0.70};      // 候选假设得分门槛
  double hyp_min_inlier{0.65};     // 候选假设内点率门槛
  std::size_t max_hypotheses{8};   // 候选假设数量上限
  // 主动绑架检测（滑动窗口低分 / 匹配失效监测）
  int anomaly_window{10};          // 滑动窗口长度（<=0 关闭）
  int anomaly_bad_threshold{8};    // 窗口内异常帧数达到阈值即主动触发 GLOBAL
  double anomaly_low_score{0.25};  // 极低分异常阈值
  // 自动全局重定位远距离跳变安全保护（防对称走廊误跳）
  double global_jump_min_dist{1.0};       // 远距离跳变判定阈值 (m)
  double global_jump_score_margin{0.15};  // 远距离候选需显著优于当前局部匹配的得分裕度
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
  void set_initial_pose(const Pose2 & p, const Cov3 & cov);
  void set_options(const CoreConfig & cfg);     // 热更新（fine mode 等）

  // 手动触发全局重定位：
  // - 无 map/matcher 或已处于 GLOBAL 时返回 false
  // - UNINIT / TRACKING / DEGRADED / LOST 时切换至 GLOBAL 并返回 true
  bool request_global();

  // 全局搜索回调（UNINIT / LOST / GLOBAL 时由外层驱动：PF 或 correlative 全局）
  using GlobalFn =
    std::function<bool(const Scan &, Pose2 & out, double & score)>;
  void set_global_fn(GlobalFn fn);

  // 多假设候选队列（随 odom/IMU 传播，DEGRADED/LOST/GLOBAL 下多帧验证晋升）
  void add_hypotheses(const std::vector<Pose2> & hyps);
  void clear_hypotheses();
  std::size_t hypothesis_count() const {return hypotheses_.size();}
  std::vector<Pose2> hypothesis_poses() const;

  // odom：odom 系 base 位姿（消息时刻）
  void on_odom(const Pose2 & odom_pose);

  // IMU：陀螺积分 yaw（任意固定系；首帧双锚定杜绝跳变）
  void on_imu_yaw(double integrated_yaw);
  void on_imu_yaw(double integrated_yaw, double dt);
  // IMU：原始角速度 wz (rad/s) 与步长 dt (s)
  void on_imu_gyro(double wz, double dt);

  // 每帧扫描处理；scan 为传感器系（= base 系，安装偏移由外层处理）
  CoreOutput on_scan(const Scan & scan);

  LocState state() const {return state_;}
  Pose2 pose() const {return last_pose_;}
  Cov3 last_cov() const {return P4_.topLeftCorner<3, 3>();}
  double gyro_bias() const {return gyro_bias_;}
  double gyro_bias_var() const {return P4_(3, 3);}
  // map→odom TF（供 ROS 层发布；基于上一帧扫描时刻锚定的 odom，不受帧间 odom 抖动影响）
  Pose2 map_to_odom() const;

private:
  struct Hypothesis
  {
    Pose2 pose;
    int confirm_count{0};
    double last_score{0.0};
  };

  void compute_motion_delta(Pose2 & delta, bool & imu_used, double & dt_imu);
  bool try_promote_hypotheses(const Scan & scan, CoreOutput & out);
  void propagate_hypotheses(const Pose2 & delta);
  void record_anomaly(bool is_bad);
  int anomaly_count() const;

  CoreConfig cfg_;
  std::shared_ptr<const MapField> field_;
  std::shared_ptr<const Matcher> matcher_;
  GlobalFn global_fn_;

  LocState state_{LocState::UNINIT};
  Pose2 last_pose_{};          // map 系
  Cov4 P4_{Cov4::Identity()};  // 4×4 协方差 [x, y, theta, b_g]
  double gyro_bias_{0.0};      // 在线估计陀螺零偏 (rad/s)
  Pose2 last_odom_{};          // 最新 odom 位姿
  Pose2 odom_at_last_scan_{};  // 上一帧扫描时刻锚定的 odom 位姿
  bool has_odom_{false};
  // IMU 状态
  double imu_yaw_{0.0};
  double imu_yaw_at_last_scan_{0.0};
  double imu_dt_accum_{0.0};
  bool has_imu_{false};
  bool imu_consumed_{true};    // true=上一帧已消费，新帧需重新积累
  int reject_streak_{0};
  int stable_tracking_streak_{0};
  bool first_fix_{false};
  bool manual_global_{false};

  std::vector<Hypothesis> hypotheses_;
  std::deque<bool> anomaly_window_;
};

}  // namespace sloc
