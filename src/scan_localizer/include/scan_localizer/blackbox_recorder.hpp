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
// BlackboxRecorder —— 低内存环形缓冲黑匣子（ROS 无关，面向边缘端实车排障）
//
// 职责：
// - 启动时一次性预分配固定环形缓冲（60s@10Hz 激光 + 50Hz 里程计 < 3 MB），
//   主定位线程运行期零堆分配；
// - 接入 LocalizerCore::on_scan 输出的 CoreOutput::diag，在进入 GLOBAL/LOST、
//   触发 GateFlag::kWallPhys 或连续拒帧超阈值时自动截取前后窗口；
// - 后台低优先级线程异步落盘紧凑二进制 (.slocbb) + 可读摘要 (.json)，
//   并执行磁盘文件配额轮转，防止撑爆嵌入式存储。
// ============================================================================
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "scan_localizer/localizer_core.hpp"

namespace sloc
{

enum class BlackboxTrigger : std::uint8_t
{
  kNone = 0,
  kEnterGlobal = 1,
  kEnterLost = 2,
  kWallPhysReject = 3,
  kRejectStreak = 4,
  kExternalRequest = 5,
};

const char * blackbox_trigger_name(BlackboxTrigger reason);

struct BlackboxConfig
{
  std::string output_dir{"/var/log/slamware/blackbox"};
  double pre_seconds{30.0};
  double post_seconds{30.0};
  double scan_hz{10.0};
  double odom_hz{50.0};
  double cooldown_s{60.0};
  int reject_streak_trigger{5};
  std::size_t max_files{20};        // 目录内最多保留的事故包数量（环形淘汰最老文件）
  bool enabled{true};
};

// 固定尺寸扫描与诊断快照（单帧 ~4.2 KB，消除运行期 vector 动态分配）
struct PackedScanRecord
{
  static constexpr std::size_t kMaxPoints = 512;
  double stamp_s{0.0};
  Pose2 out_pose{};
  LocState state{LocState::UNINIT};
  bool accepted{false};
  double score{0.0};
  double inlier_ratio{0.0};
  int reject_streak{0};
  FrameDiagnostics diag{};
  std::uint16_t num_points{0};
  std::array<float, kMaxPoints * 2> xy{};  // 交错存 [x0, y0, x1, y1, ...]
};

struct PackedOdomRecord
{
  double stamp_s{0.0};
  Pose2 odom_pose{};
  double imu_yaw{0.0};
  bool has_imu{false};
};

// 落盘与离线回放使用的完整事故包
struct BlackboxBundle
{
  double trigger_stamp_s{0.0};
  BlackboxTrigger reason{BlackboxTrigger::kNone};
  std::vector<PackedOdomRecord> odoms;
  std::vector<PackedScanRecord> scans;
};

// 将 PackedScanRecord 还原为 LocalizerCore::on_scan 可直接回放的 Scan
Scan unpack_scan(const PackedScanRecord & rec);

// 紧凑二进制事故包 (.slocbb) 读写接口（含魔数、版本与边界校验）
bool save_blackbox_bundle(const std::string & path, const BlackboxBundle & bundle);
bool load_blackbox_bundle(const std::string & path, BlackboxBundle & out_bundle);

class BlackboxRecorder {
public:
  explicit BlackboxRecorder(BlackboxConfig cfg = {});
  ~BlackboxRecorder();

  BlackboxRecorder(const BlackboxRecorder &) = delete;
  BlackboxRecorder & operator=(const BlackboxRecorder &) = delete;

  // 实时线程调用：O(1) 写入高频里程计/IMU，零堆分配
  void record_odom(
    double stamp_s, const Pose2 & odom_pose,
    double imu_yaw = 0.0, bool has_imu = false);

  // 实时线程调用：写入激光与诊断快照，并自动监测触发/完成条件
  void record_scan(double stamp_s, const Scan & scan, const CoreOutput & out);

  // 供外部桥接层（如收到外部 TRIGGER_GLOBAL）主动触发
  void trigger(double stamp_s, BlackboxTrigger reason);

  // 等待后台落盘队列清空；force_finalize=true 时强制提交当前未满后窗的录制
  void flush_and_wait(bool force_finalize = false);

  std::size_t dump_count() const {return dump_count_.load(std::memory_order_acquire);}
  bool is_capturing_post() const {return capturing_post_;}

private:
  void start_capture(double stamp_s, BlackboxTrigger reason);
  void dispatch_snapshot();
  void writer_loop();
  bool write_bundle_to_disk(const BlackboxBundle & job);
  void enforce_disk_quota();

  BlackboxConfig cfg_;
  std::vector<PackedScanRecord> scan_ring_;
  std::vector<PackedOdomRecord> odom_ring_;
  std::size_t scan_head_{0};
  std::size_t scan_count_{0};
  std::size_t odom_head_{0};
  std::size_t odom_count_{0};

  // 触发与后窗状态机（仅主线程访问）
  bool capturing_post_{false};
  BlackboxTrigger active_reason_{BlackboxTrigger::kNone};
  double trigger_stamp_s_{0.0};
  double last_dump_stamp_s_{-1e18};
  LocState prev_state_{LocState::UNINIT};

  // 后台异步写盘线程
  std::mutex job_mtx_;
  std::condition_variable job_cv_;
  std::condition_variable idle_cv_;
  bool stop_writer_{false};
  bool job_pending_{false};
  bool writer_busy_{false};
  BlackboxBundle pending_job_;
  std::atomic<std::size_t> dump_count_{0};
  std::thread writer_thread_;
};

}  // namespace sloc
