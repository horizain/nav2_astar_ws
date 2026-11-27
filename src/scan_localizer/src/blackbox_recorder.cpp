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

#include "scan_localizer/blackbox_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace sloc
{
namespace
{

constexpr char kMagic[8] = {'S', 'L', 'O', 'C', 'B', 'B', '0', '1'};
constexpr std::uint32_t kFormatVersion = 1u;
constexpr std::uint32_t kMaxBundleOdoms = 200000u;
constexpr std::uint32_t kMaxBundleScans = 50000u;

template<typename T>
void write_pod(std::ostream & os, const T & val)
{
  os.write(reinterpret_cast<const char *>(&val), sizeof(T));
}

template<typename T>
bool read_pod(std::istream & is, T & val)
{
  is.read(reinterpret_cast<char *>(&val), sizeof(T));
  return is.good();
}

const char * loc_state_name(LocState st)
{
  switch (st) {
    case LocState::UNINIT: return "UNINIT";
    case LocState::TRACKING: return "TRACKING";
    case LocState::DEGRADED: return "DEGRADED";
    case LocState::LOST: return "LOST";
    case LocState::GLOBAL: return "GLOBAL";
  }
  return "UNKNOWN";
}

void pack_scan_points(const Scan & scan, PackedScanRecord & rec)
{
  rec.num_points = 0;
  const std::size_t n = scan.points.size();
  if (n == 0) {
    return;
  }
  const std::size_t stride = std::max<std::size_t>(
    1u, (n + PackedScanRecord::kMaxPoints - 1u) / PackedScanRecord::kMaxPoints);
  std::uint16_t out_n = 0;
  for (std::size_t i = 0; i < n && out_n < PackedScanRecord::kMaxPoints; i += stride) {
    const auto & pt = scan.points[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y())) {
      continue;
    }
    rec.xy[static_cast<std::size_t>(out_n) * 2u] = static_cast<float>(pt.x());
    rec.xy[static_cast<std::size_t>(out_n) * 2u + 1u] = static_cast<float>(pt.y());
    ++out_n;
  }
  rec.num_points = out_n;
}

}  // namespace

const char * blackbox_trigger_name(BlackboxTrigger reason)
{
  switch (reason) {
    case BlackboxTrigger::kNone: return "NONE";
    case BlackboxTrigger::kEnterGlobal: return "ENTER_GLOBAL";
    case BlackboxTrigger::kEnterLost: return "ENTER_LOST";
    case BlackboxTrigger::kWallPhysReject: return "WALL_PHYS_REJECT";
    case BlackboxTrigger::kRejectStreak: return "REJECT_STREAK";
    case BlackboxTrigger::kExternalRequest: return "EXTERNAL_REQUEST";
  }
  return "UNKNOWN";
}

Scan unpack_scan(const PackedScanRecord & rec)
{
  Scan scan;
  const std::size_t n = std::min<std::size_t>(rec.num_points, PackedScanRecord::kMaxPoints);
  scan.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    scan.points.emplace_back(
      static_cast<double>(rec.xy[i * 2u]),
      static_cast<double>(rec.xy[i * 2u + 1u]));
  }
  return scan;
}

bool save_blackbox_bundle(const std::string & path, const BlackboxBundle & bundle)
{
  std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
  if (!ofs.is_open()) {
    return false;
  }
  ofs.write(kMagic, sizeof(kMagic));
  write_pod(ofs, kFormatVersion);
  write_pod(ofs, bundle.trigger_stamp_s);
  const std::uint8_t reason_u8 = static_cast<std::uint8_t>(bundle.reason);
  write_pod(ofs, reason_u8);
  const std::uint32_t n_odoms = static_cast<std::uint32_t>(bundle.odoms.size());
  const std::uint32_t n_scans = static_cast<std::uint32_t>(bundle.scans.size());
  write_pod(ofs, n_odoms);
  write_pod(ofs, n_scans);

  for (const auto & od : bundle.odoms) {
    write_pod(ofs, od.stamp_s);
    write_pod(ofs, od.odom_pose.x);
    write_pod(ofs, od.odom_pose.y);
    write_pod(ofs, od.odom_pose.theta);
    write_pod(ofs, od.imu_yaw);
    const std::uint8_t has_imu_u8 = od.has_imu ? 1u : 0u;
    write_pod(ofs, has_imu_u8);
  }

  for (const auto & sc : bundle.scans) {
    write_pod(ofs, sc.stamp_s);
    write_pod(ofs, sc.out_pose.x);
    write_pod(ofs, sc.out_pose.y);
    write_pod(ofs, sc.out_pose.theta);
    const std::uint8_t st_u8 = static_cast<std::uint8_t>(sc.state);
    const std::uint8_t acc_u8 = sc.accepted ? 1u : 0u;
    write_pod(ofs, st_u8);
    write_pod(ofs, acc_u8);
    write_pod(ofs, sc.score);
    write_pod(ofs, sc.inlier_ratio);
    const std::int32_t streak_i32 = static_cast<std::int32_t>(sc.reject_streak);
    write_pod(ofs, streak_i32);
    write_pod(ofs, sc.diag.reject_flags);
    write_pod(ofs, sc.diag.raw_score);
    write_pod(ofs, sc.diag.raw_inlier);
    write_pod(ofs, sc.diag.wall_visibility);
    write_pod(ofs, sc.diag.frontier_visibility);
    write_pod(ofs, sc.diag.short_ratio);
    write_pod(ofs, sc.diag.frontier_ratio);
    write_pod(ofs, sc.diag.mahal_dist);
    const std::uint16_t npts = static_cast<std::uint16_t>(
      std::min<std::size_t>(sc.num_points, PackedScanRecord::kMaxPoints));
    write_pod(ofs, npts);
    if (npts > 0) {
      ofs.write(
        reinterpret_cast<const char *>(sc.xy.data()),
        static_cast<std::streamsize>(static_cast<std::size_t>(npts) * 2u * sizeof(float)));
    }
  }
  ofs.flush();
  return ofs.good();
}

bool load_blackbox_bundle(const std::string & path, BlackboxBundle & out_bundle)
{
  out_bundle = BlackboxBundle{};
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    return false;
  }
  char magic[8] = {};
  ifs.read(magic, sizeof(magic));
  if (!ifs.good() || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    return false;
  }
  std::uint32_t version = 0;
  if (!read_pod(ifs, version) || version != kFormatVersion) {
    return false;
  }
  std::uint8_t reason_u8 = 0;
  std::uint32_t n_odoms = 0;
  std::uint32_t n_scans = 0;
  if (!read_pod(ifs, out_bundle.trigger_stamp_s) ||
    !read_pod(ifs, reason_u8) ||
    !read_pod(ifs, n_odoms) ||
    !read_pod(ifs, n_scans))
  {
    return false;
  }
  if (n_odoms > kMaxBundleOdoms || n_scans > kMaxBundleScans) {
    return false;
  }
  out_bundle.reason = static_cast<BlackboxTrigger>(reason_u8);

  out_bundle.odoms.resize(n_odoms);
  for (std::uint32_t i = 0; i < n_odoms; ++i) {
    auto & od = out_bundle.odoms[i];
    std::uint8_t has_imu_u8 = 0;
    if (!read_pod(ifs, od.stamp_s) ||
      !read_pod(ifs, od.odom_pose.x) ||
      !read_pod(ifs, od.odom_pose.y) ||
      !read_pod(ifs, od.odom_pose.theta) ||
      !read_pod(ifs, od.imu_yaw) ||
      !read_pod(ifs, has_imu_u8))
    {
      return false;
    }
    od.has_imu = (has_imu_u8 != 0u);
  }

  out_bundle.scans.resize(n_scans);
  for (std::uint32_t i = 0; i < n_scans; ++i) {
    auto & sc = out_bundle.scans[i];
    std::uint8_t st_u8 = 0;
    std::uint8_t acc_u8 = 0;
    std::int32_t streak_i32 = 0;
    std::uint16_t npts = 0;
    if (!read_pod(ifs, sc.stamp_s) ||
      !read_pod(ifs, sc.out_pose.x) ||
      !read_pod(ifs, sc.out_pose.y) ||
      !read_pod(ifs, sc.out_pose.theta) ||
      !read_pod(ifs, st_u8) ||
      !read_pod(ifs, acc_u8) ||
      !read_pod(ifs, sc.score) ||
      !read_pod(ifs, sc.inlier_ratio) ||
      !read_pod(ifs, streak_i32) ||
      !read_pod(ifs, sc.diag.reject_flags) ||
      !read_pod(ifs, sc.diag.raw_score) ||
      !read_pod(ifs, sc.diag.raw_inlier) ||
      !read_pod(ifs, sc.diag.wall_visibility) ||
      !read_pod(ifs, sc.diag.frontier_visibility) ||
      !read_pod(ifs, sc.diag.short_ratio) ||
      !read_pod(ifs, sc.diag.frontier_ratio) ||
      !read_pod(ifs, sc.diag.mahal_dist) ||
      !read_pod(ifs, npts))
    {
      return false;
    }
    if (npts > PackedScanRecord::kMaxPoints) {
      return false;
    }
    sc.state = static_cast<LocState>(st_u8);
    sc.accepted = (acc_u8 != 0u);
    sc.reject_streak = static_cast<int>(streak_i32);
    sc.num_points = npts;
    if (npts > 0) {
      ifs.read(
        reinterpret_cast<char *>(sc.xy.data()),
        static_cast<std::streamsize>(static_cast<std::size_t>(npts) * 2u * sizeof(float)));
      if (!ifs.good()) {
        return false;
      }
    }
  }
  return true;
}

BlackboxRecorder::BlackboxRecorder(BlackboxConfig cfg)
: cfg_(std::move(cfg))
{
  const double total_s = std::max(1.0, cfg_.pre_seconds + cfg_.post_seconds);
  const std::size_t scan_cap = std::max<std::size_t>(
    16u, static_cast<std::size_t>(std::ceil(total_s * std::max(1.0, cfg_.scan_hz))) + 8u);
  const std::size_t odom_cap = std::max<std::size_t>(
    32u, static_cast<std::size_t>(std::ceil(total_s * std::max(1.0, cfg_.odom_hz))) + 16u);
  scan_ring_.resize(scan_cap);
  odom_ring_.resize(odom_cap);
  pending_job_.scans.reserve(scan_cap);
  pending_job_.odoms.reserve(odom_cap);

  if (cfg_.enabled) {
    writer_thread_ = std::thread(&BlackboxRecorder::writer_loop, this);
  }
}

BlackboxRecorder::~BlackboxRecorder()
{
  {
    std::lock_guard<std::mutex> lk(job_mtx_);
    stop_writer_ = true;
  }
  job_cv_.notify_all();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
}

void BlackboxRecorder::record_odom(
  double stamp_s, const Pose2 & odom_pose, double imu_yaw, bool has_imu)
{
  if (!cfg_.enabled || odom_ring_.empty()) {
    return;
  }
  auto & slot = odom_ring_[odom_head_];
  slot.stamp_s = stamp_s;
  slot.odom_pose = odom_pose;
  slot.imu_yaw = imu_yaw;
  slot.has_imu = has_imu;
  odom_head_ = (odom_head_ + 1u) % odom_ring_.size();
  if (odom_count_ < odom_ring_.size()) {
    ++odom_count_;
  }
}

void BlackboxRecorder::record_scan(
  double stamp_s, const Scan & scan, const CoreOutput & out)
{
  if (!cfg_.enabled || scan_ring_.empty()) {
    return;
  }
  auto & slot = scan_ring_[scan_head_];
  slot.stamp_s = stamp_s;
  slot.out_pose = out.pose;
  slot.state = out.state;
  slot.accepted = out.accepted;
  slot.score = out.score;
  slot.inlier_ratio = out.inlier_ratio;
  slot.reject_streak = out.reject_streak;
  slot.diag = out.diag;
  pack_scan_points(scan, slot);

  scan_head_ = (scan_head_ + 1u) % scan_ring_.size();
  if (scan_count_ < scan_ring_.size()) {
    ++scan_count_;
  }

  // 自动监测异常触发条件（仅在非后窗捕获且过冷却期时生效）
  if (!capturing_post_) {
    const bool streak_hit = cfg_.reject_streak_trigger > 0 &&
      out.reject_streak >= cfg_.reject_streak_trigger;
    const bool wall_phys_hit =
      (out.diag.reject_flags & GateFlag::kInvalid) == 0u &&
      (out.diag.reject_flags & GateFlag::kWallPhys) != 0u;
    const bool initialized = prev_state_ != LocState::UNINIT;
    BlackboxTrigger auto_reason = BlackboxTrigger::kNone;
    if (initialized && out.state == LocState::GLOBAL && prev_state_ != LocState::GLOBAL) {
      auto_reason = BlackboxTrigger::kEnterGlobal;
    } else if (initialized && out.state == LocState::LOST && prev_state_ != LocState::LOST) {
      auto_reason = BlackboxTrigger::kEnterLost;
    } else if (wall_phys_hit) {
      auto_reason = BlackboxTrigger::kWallPhysReject;
    } else if (streak_hit) {
      auto_reason = BlackboxTrigger::kRejectStreak;
    }
    if (auto_reason != BlackboxTrigger::kNone) {
      start_capture(stamp_s, auto_reason);
    }
  }
  prev_state_ = out.state;

  // 若正处于后窗捕获阶段，录满 post_seconds 后立刻异步切分落盘
  if (capturing_post_ && (stamp_s - trigger_stamp_s_) >= cfg_.post_seconds) {
    dispatch_snapshot();
  }
}

void BlackboxRecorder::trigger(double stamp_s, BlackboxTrigger reason)
{
  if (!cfg_.enabled || reason == BlackboxTrigger::kNone) {
    return;
  }
  start_capture(stamp_s, reason);
  if (capturing_post_ && cfg_.post_seconds <= 0.0) {
    dispatch_snapshot();
  }
}

void BlackboxRecorder::start_capture(double stamp_s, BlackboxTrigger reason)
{
  if (capturing_post_) {
    return;
  }
  if ((stamp_s - last_dump_stamp_s_) < cfg_.cooldown_s) {
    return;
  }
  capturing_post_ = true;
  active_reason_ = reason;
  trigger_stamp_s_ = stamp_s;
}

void BlackboxRecorder::dispatch_snapshot()
{
  const double win_start = trigger_stamp_s_ - cfg_.pre_seconds;
  const double win_end = trigger_stamp_s_ + cfg_.post_seconds;
  {
    std::lock_guard<std::mutex> lk(job_mtx_);
    if (job_pending_) {
      // 后台槽位尚未消费完，放弃本次避免阻塞主定位线程
      capturing_post_ = false;
      return;
    }
    pending_job_.trigger_stamp_s = trigger_stamp_s_;
    pending_job_.reason = active_reason_;
    pending_job_.scans.clear();
    pending_job_.odoms.clear();

    const std::size_t s_cap = scan_ring_.size();
    const std::size_t s_start = (scan_head_ + s_cap - scan_count_) % s_cap;
    for (std::size_t i = 0; i < scan_count_; ++i) {
      const auto & rec = scan_ring_[(s_start + i) % s_cap];
      if (rec.stamp_s >= win_start && rec.stamp_s <= win_end + 1e-6) {
        pending_job_.scans.push_back(rec);
      }
    }

    const std::size_t o_cap = odom_ring_.size();
    const std::size_t o_start = (odom_head_ + o_cap - odom_count_) % o_cap;
    for (std::size_t i = 0; i < odom_count_; ++i) {
      const auto & rec = odom_ring_[(o_start + i) % o_cap];
      if (rec.stamp_s >= win_start && rec.stamp_s <= win_end + 1e-6) {
        pending_job_.odoms.push_back(rec);
      }
    }

    job_pending_ = true;
    last_dump_stamp_s_ = trigger_stamp_s_;
    capturing_post_ = false;
  }
  job_cv_.notify_one();
}

void BlackboxRecorder::flush_and_wait(bool force_finalize)
{
  if (!cfg_.enabled) {
    return;
  }
  if (force_finalize && capturing_post_) {
    dispatch_snapshot();
  }
  std::unique_lock<std::mutex> lk(job_mtx_);
  idle_cv_.wait(lk, [&]() {return !job_pending_ && !writer_busy_;});
}

void BlackboxRecorder::writer_loop()
{
  BlackboxBundle local_job;
  while (true) {
    {
      std::unique_lock<std::mutex> lk(job_mtx_);
      job_cv_.wait(lk, [&]() {return stop_writer_ || job_pending_;});
      if (stop_writer_ && !job_pending_) {
        break;
      }
      std::swap(local_job, pending_job_);
      job_pending_ = false;
      writer_busy_ = true;
    }

    if (write_bundle_to_disk(local_job)) {
      dump_count_.fetch_add(1u, std::memory_order_release);
      enforce_disk_quota();
    }

    {
      std::lock_guard<std::mutex> lk(job_mtx_);
      writer_busy_ = false;
    }
    idle_cv_.notify_all();
  }
}

bool BlackboxRecorder::write_bundle_to_disk(const BlackboxBundle & job)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(cfg_.output_dir, ec);
  if (ec) {
    return false;
  }

  const std::int64_t stamp_ms = static_cast<std::int64_t>(std::llround(job.trigger_stamp_s * 1e3));
  std::ostringstream base;
  base << "bb_" << std::setw(13) << std::setfill('0') << std::max<std::int64_t>(0, stamp_ms)
       << "_" << blackbox_trigger_name(job.reason);
  const fs::path bin_path = fs::path(cfg_.output_dir) / (base.str() + ".slocbb");
  const fs::path json_path = fs::path(cfg_.output_dir) / (base.str() + ".json");

  if (!save_blackbox_bundle(bin_path.string(), job)) {
    return false;
  }

  // 写出人类可读 JSON 摘要，便于现场 1 秒定位触发原因与门控位掩码
  std::ofstream jfs(json_path, std::ios::trunc);
  if (jfs.is_open()) {
    jfs << std::fixed << std::setprecision(4);
    jfs << "{\n"
        << "  \"trigger_stamp_s\": " << job.trigger_stamp_s << ",\n"
        << "  \"reason\": \"" << blackbox_trigger_name(job.reason) << "\",\n"
        << "  \"num_scans\": " << job.scans.size() << ",\n"
        << "  \"num_odoms\": " << job.odoms.size() << ",\n"
        << "  \"binary_file\": \"" << bin_path.filename().string() << "\"";
    if (!job.scans.empty()) {
      const auto it = std::min_element(
        job.scans.begin(), job.scans.end(),
        [&](const PackedScanRecord & a, const PackedScanRecord & b) {
          return std::abs(a.stamp_s - job.trigger_stamp_s) <
                 std::abs(b.stamp_s - job.trigger_stamp_s);
        });
      jfs << ",\n  \"trigger_frame\": {\n"
          << "    \"stamp_s\": " << it->stamp_s << ",\n"
          << "    \"state\": \"" << loc_state_name(it->state) << "\",\n"
          << "    \"accepted\": " << (it->accepted ? "true" : "false") << ",\n"
          << "    \"score\": " << it->score << ",\n"
          << "    \"inlier_ratio\": " << it->inlier_ratio << ",\n"
          << "    \"reject_streak\": " << it->reject_streak << ",\n"
          << "    \"reject_flags\": " << it->diag.reject_flags << ",\n"
          << "    \"raw_score\": " << it->diag.raw_score << ",\n"
          << "    \"wall_visibility\": " << it->diag.wall_visibility << ",\n"
          << "    \"frontier_visibility\": " << it->diag.frontier_visibility << ",\n"
          << "    \"short_ratio\": " << it->diag.short_ratio << ",\n"
          << "    \"frontier_ratio\": " << it->diag.frontier_ratio << ",\n"
          << "    \"mahal_dist\": " << it->diag.mahal_dist << "\n"
          << "  }\n";
    } else {
      jfs << "\n";
    }
    jfs << "}\n";
  }
  return true;
}

void BlackboxRecorder::enforce_disk_quota()
{
  if (cfg_.max_files == 0) {
    return;
  }
  namespace fs = std::filesystem;
  std::error_code ec;
  std::vector<fs::path> bundles;
  for (const auto & entry : fs::directory_iterator(cfg_.output_dir, ec)) {
    if (ec) {
      return;
    }
    if (entry.is_regular_file() && entry.path().extension() == ".slocbb") {
      bundles.push_back(entry.path());
    }
  }
  if (bundles.size() <= cfg_.max_files) {
    return;
  }
  // 文件名以零填充毫秒时间戳前缀排序，字典序即时间先后顺序
  std::sort(bundles.begin(), bundles.end());
  const std::size_t to_remove = bundles.size() - cfg_.max_files;
  for (std::size_t i = 0; i < to_remove; ++i) {
    fs::remove(bundles[i], ec);
    fs::path companion_json = bundles[i];
    companion_json.replace_extension(".json");
    fs::remove(companion_json, ec);
  }
}

}  // namespace sloc
