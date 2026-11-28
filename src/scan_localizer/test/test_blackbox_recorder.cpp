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

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "scan_localizer/blackbox_recorder.hpp"

namespace
{

namespace fs = std::filesystem;

struct ScopedTempDir
{
  fs::path path;
  explicit ScopedTempDir(const std::string & name)
  : path(fs::temp_directory_path() / ("sloc_bb_test_" + name))
  {
    std::error_code ec;
    fs::remove_all(path, ec);
    fs::create_directories(path, ec);
  }
  ~ScopedTempDir()
  {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

sloc::Scan make_ring_scan(std::size_t n_pts, double radius)
{
  sloc::Scan s;
  s.points.reserve(n_pts);
  for (std::size_t i = 0; i < n_pts; ++i) {
    const double ang = static_cast<double>(i) * (sloc::kTwoPi / static_cast<double>(n_pts));
    s.points.emplace_back(radius * std::cos(ang), radius * std::sin(ang));
  }
  return s;
}

TEST(BlackboxRecorderTest, BinaryRoundTripAndCorruptRejection)
{
  ScopedTempDir tmp("roundtrip");
  const fs::path bin_path = tmp.path / "sample.slocbb";

  sloc::BlackboxBundle in;
  in.trigger_stamp_s = 123.456;
  in.reason = sloc::BlackboxTrigger::kWallPhysReject;

  sloc::PackedOdomRecord od;
  od.stamp_s = 123.40;
  od.odom_pose = sloc::Pose2(1.5, -2.25, 0.75);
  od.imu_yaw = 0.78;
  od.has_imu = true;
  in.odoms.push_back(od);

  sloc::PackedScanRecord sc;
  sc.stamp_s = 123.456;
  sc.out_pose = sloc::Pose2(1.6, -2.2, 0.76);
  sc.state = sloc::LocState::DEGRADED;
  sc.accepted = false;
  sc.score = 0.12;
  sc.inlier_ratio = 0.79;
  sc.reject_streak = 3;
  sc.diag.reject_flags = sloc::GateFlag::kWallPhys | sloc::GateFlag::kScore;
  sc.diag.raw_score = 0.81;
  sc.diag.raw_inlier = 0.79;
  sc.diag.wall_visibility = 0.15;
  sc.diag.frontier_visibility = 0.92;
  sc.diag.short_ratio = 0.08;
  sc.diag.frontier_ratio = 0.03;
  sc.diag.mahal_dist = 1.42;
  sc.num_points = 3;
  sc.xy[0] = 1.0f;
  sc.xy[1] = 2.0f;
  sc.xy[2] = -3.5f;
  sc.xy[3] = 4.25f;
  sc.xy[4] = 0.0f;
  sc.xy[5] = -1.25f;
  in.scans.push_back(sc);

  ASSERT_TRUE(sloc::save_blackbox_bundle(bin_path.string(), in));

  sloc::BlackboxBundle out;
  ASSERT_TRUE(sloc::load_blackbox_bundle(bin_path.string(), out));
  EXPECT_DOUBLE_EQ(out.trigger_stamp_s, in.trigger_stamp_s);
  EXPECT_EQ(out.reason, in.reason);
  ASSERT_EQ(out.odoms.size(), 1u);
  EXPECT_DOUBLE_EQ(out.odoms[0].odom_pose.x, 1.5);
  EXPECT_TRUE(out.odoms[0].has_imu);
  ASSERT_EQ(out.scans.size(), 1u);
  EXPECT_EQ(out.scans[0].state, sloc::LocState::DEGRADED);
  EXPECT_EQ(out.scans[0].diag.reject_flags, sc.diag.reject_flags);
  EXPECT_DOUBLE_EQ(out.scans[0].diag.wall_visibility, 0.15);
  EXPECT_DOUBLE_EQ(out.scans[0].diag.mahal_dist, 1.42);

  const sloc::Scan unpacked = sloc::unpack_scan(out.scans[0]);
  ASSERT_EQ(unpacked.points.size(), 3u);
  EXPECT_NEAR(unpacked.points[1].x(), -3.5, 1e-5);
  EXPECT_NEAR(unpacked.points[1].y(), 4.25, 1e-5);

  // Corrupt magic must fail cleanly
  const fs::path bad_path = tmp.path / "bad.slocbb";
  {
    std::ofstream bad(bad_path, std::ios::binary);
    bad << "NOTMAGIC12345678";
  }
  EXPECT_FALSE(sloc::load_blackbox_bundle(bad_path.string(), out));
}

TEST(BlackboxRecorderTest, CapturesPrePostWindowAndDownsamplesLargeScans)
{
  ScopedTempDir tmp("prepost");
  sloc::BlackboxConfig cfg;
  cfg.output_dir = tmp.path.string();
  cfg.pre_seconds = 1.0;
  cfg.post_seconds = 1.0;
  cfg.scan_hz = 10.0;
  cfg.odom_hz = 20.0;
  cfg.cooldown_s = 10.0;
  cfg.reject_streak_trigger = 5;

  sloc::BlackboxRecorder rec(cfg);
  // 720-beam scan exercises uniform downsampling to <= kMaxPoints (512)
  const sloc::Scan big_scan = make_ring_scan(720, 4.0);

  for (int i = 0; i <= 40; ++i) {
    const double t = 10.0 + 0.1 * i;
    rec.record_odom(t, sloc::Pose2(0.1 * i, 0.0, 0.0), 0.01 * i, true);
    sloc::CoreOutput out;
    out.pose = sloc::Pose2(0.1 * i, 0.0, 0.0);
    out.state = (i == 20) ? sloc::LocState::GLOBAL : sloc::LocState::TRACKING;
    out.accepted = (i != 20);
    out.score = (i == 20) ? 0.10 : 0.90;
    out.inlier_ratio = 0.88;
    out.diag.raw_score = 0.85;
    out.diag.wall_visibility = (i == 20) ? 0.12 : 1.0;
    out.diag.reject_flags = (i == 20) ? sloc::GateFlag::kWallPhys : sloc::GateFlag::kNone;
    rec.record_scan(t, big_scan, out);
  }
  rec.flush_and_wait();

  EXPECT_EQ(rec.dump_count(), 1u);

  std::vector<fs::path> bins;
  std::vector<fs::path> jsons;
  for (const auto & e : fs::directory_iterator(tmp.path)) {
    if (e.path().extension() == ".slocbb") {bins.push_back(e.path());}
    if (e.path().extension() == ".json") {jsons.push_back(e.path());}
  }
  ASSERT_EQ(bins.size(), 1u);
  ASSERT_EQ(jsons.size(), 1u);

  sloc::BlackboxBundle loaded;
  ASSERT_TRUE(sloc::load_blackbox_bundle(bins[0].string(), loaded));
  EXPECT_EQ(loaded.reason, sloc::BlackboxTrigger::kEnterGlobal);
  EXPECT_NEAR(loaded.trigger_stamp_s, 12.0, 1e-6);
  // Pre 1.0s [11.0..12.0] + Post 1.0s [12.0..13.0] at 10Hz => ~21 frames
  EXPECT_GE(loaded.scans.size(), 19u);
  EXPECT_LE(loaded.scans.size(), 22u);
  EXPECT_GE(loaded.odoms.size(), 19u);
  EXPECT_LE(loaded.scans[0].num_points, sloc::PackedScanRecord::kMaxPoints);
  EXPECT_GT(loaded.scans[0].num_points, 300u);
  // Chronological order check
  for (std::size_t k = 1; k < loaded.scans.size(); ++k) {
    EXPECT_GT(loaded.scans[k].stamp_s, loaded.scans[k - 1].stamp_s);
  }
}

TEST(BlackboxRecorderTest, CooldownSuppressesStormAndQuotaRotatesOldestFiles)
{
  ScopedTempDir tmp("quota");
  sloc::BlackboxConfig cfg;
  cfg.output_dir = tmp.path.string();
  cfg.pre_seconds = 0.5;
  cfg.post_seconds = 0.2;
  cfg.scan_hz = 10.0;
  cfg.odom_hz = 10.0;
  cfg.cooldown_s = 2.0;
  cfg.max_files = 3;

  sloc::BlackboxRecorder rec(cfg);
  const sloc::Scan scan = make_ring_scan(36, 3.0);
  sloc::CoreOutput ok_out;
  ok_out.state = sloc::LocState::TRACKING;
  ok_out.accepted = true;
  ok_out.score = 0.90;

  // Trigger 5 times spaced by 3.0s (> cooldown 2.0s), plus an ignored storm trigger +0.5s after #1
  for (int k = 0; k < 5; ++k) {
    const double t0 = 100.0 + k * 3.0;
    rec.record_scan(t0, scan, ok_out);
    rec.trigger(t0, sloc::BlackboxTrigger::kExternalRequest);
    rec.record_scan(t0 + 0.25, scan, ok_out);  // finishes post window
    rec.flush_and_wait();
    if (k == 0) {
      // Within cooldown (t0 + 0.5 < t0 + 2.0): must be suppressed
      rec.trigger(t0 + 0.5, sloc::BlackboxTrigger::kWallPhysReject);
      EXPECT_FALSE(rec.is_capturing_post());
    }
  }

  EXPECT_EQ(rec.dump_count(), 5u);

  std::size_t bin_cnt = 0;
  std::size_t json_cnt = 0;
  for (const auto & e : fs::directory_iterator(tmp.path)) {
    if (e.path().extension() == ".slocbb") {++bin_cnt;}
    if (e.path().extension() == ".json") {++json_cnt;}
  }
  EXPECT_EQ(bin_cnt, 3u);
  EXPECT_EQ(json_cnt, 3u);
}

TEST(BlackboxRecorderTest, DisabledRecorderWritesNothing)
{
  ScopedTempDir tmp("disabled");
  sloc::BlackboxConfig cfg;
  cfg.output_dir = tmp.path.string();
  cfg.enabled = false;

  sloc::BlackboxRecorder rec(cfg);
  sloc::CoreOutput out;
  out.state = sloc::LocState::LOST;
  out.diag.reject_flags = sloc::GateFlag::kWallPhys;
  rec.record_scan(1.0, make_ring_scan(36, 2.0), out);
  rec.trigger(1.0, sloc::BlackboxTrigger::kExternalRequest);
  rec.flush_and_wait(true);
  EXPECT_EQ(rec.dump_count(), 0u);
}

TEST(BlackboxRecorderTest, IgnoresColdStartGlobalAndInvalidMatchFlag)
{
  ScopedTempDir tmp("coldstart_invalid");
  sloc::BlackboxConfig cfg;
  cfg.output_dir = tmp.path.string();
  cfg.pre_seconds = 0.5;
  cfg.post_seconds = 0.2;
  cfg.cooldown_s = 1.0;

  sloc::BlackboxRecorder rec(cfg);
  const sloc::Scan scan = make_ring_scan(36, 3.0);

  // Cold start UNINIT -> GLOBAL must not trigger a false-positive incident dump
  sloc::CoreOutput cold_global;
  cold_global.state = sloc::LocState::GLOBAL;
  rec.record_scan(1.0, scan, cold_global);
  EXPECT_FALSE(rec.is_capturing_post());

  // Enter normal tracking
  sloc::CoreOutput tracking;
  tracking.state = sloc::LocState::TRACKING;
  tracking.accepted = true;
  rec.record_scan(1.1, scan, tracking);

  // Invalid match frame must not masquerade as kWallPhysReject
  sloc::CoreOutput invalid_out;
  invalid_out.state = sloc::LocState::TRACKING;
  invalid_out.diag.reject_flags = sloc::GateFlag::kInvalid | sloc::GateFlag::kWallPhys;
  rec.record_scan(1.2, scan, invalid_out);
  EXPECT_FALSE(rec.is_capturing_post());

  // Genuine wall-piercing rejection (without kInvalid) must still trigger
  sloc::CoreOutput wall_out;
  wall_out.state = sloc::LocState::TRACKING;
  wall_out.diag.reject_flags = sloc::GateFlag::kWallPhys;
  rec.record_scan(1.3, scan, wall_out);
  EXPECT_TRUE(rec.is_capturing_post());
  rec.record_scan(1.6, scan, tracking);
  rec.flush_and_wait();
  EXPECT_EQ(rec.dump_count(), 1u);
}

TEST(BlackboxRecorderTest, DestructorFlushesInFlightCaptureAndSanitizesJsonNan)
{
  ScopedTempDir tmp("dtor_nan");
  {
    sloc::BlackboxConfig cfg;
    cfg.output_dir = tmp.path.string();
    cfg.pre_seconds = 0.5;
    cfg.post_seconds = 30.0;  // Still in post window when destroyed
    cfg.cooldown_s = 1.0;

    sloc::BlackboxRecorder rec(cfg);
    const sloc::Scan scan = make_ring_scan(36, 3.0);
    sloc::CoreOutput out;
    out.state = sloc::LocState::TRACKING;
    out.score = std::numeric_limits<double>::quiet_NaN();
    out.diag.mahal_dist = std::numeric_limits<double>::infinity();
    out.diag.reject_flags = sloc::GateFlag::kWallPhys;
    rec.record_scan(5.0, scan, out);
    EXPECT_TRUE(rec.is_capturing_post());
  }

  std::vector<fs::path> bins;
  std::vector<fs::path> jsons;
  std::size_t tmp_files = 0;
  for (const auto & e : fs::directory_iterator(tmp.path)) {
    if (e.path().extension() == ".slocbb") {bins.push_back(e.path());}
    if (e.path().extension() == ".json") {jsons.push_back(e.path());}
    if (e.path().extension() == ".tmp") {++tmp_files;}
  }
  EXPECT_EQ(tmp_files, 0u);
  ASSERT_EQ(bins.size(), 1u);
  ASSERT_EQ(jsons.size(), 1u);

  std::ifstream ifs(jsons[0]);
  const std::string jtxt(
    (std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  EXPECT_EQ(jtxt.find("nan"), std::string::npos);
  EXPECT_EQ(jtxt.find("inf"), std::string::npos);
}

TEST(BlackboxRecorderTest, TimeRollbackResetsInFlightCaptureAndCooldown)
{
  ScopedTempDir tmp("rollback");
  sloc::BlackboxConfig cfg;
  cfg.output_dir = tmp.path.string();
  cfg.pre_seconds = 0.5;
  cfg.post_seconds = 2.0;
  cfg.cooldown_s = 30.0;

  sloc::BlackboxRecorder rec(cfg);
  const sloc::Scan scan = make_ring_scan(36, 3.0);
  sloc::CoreOutput ok_out;
  ok_out.state = sloc::LocState::TRACKING;
  ok_out.accepted = true;
  ok_out.score = 0.90;

  // Start a capture at t=100.0 (post window ends at 102.0)
  rec.record_scan(100.0, scan, ok_out);
  rec.trigger(100.0, sloc::BlackboxTrigger::kExternalRequest);
  EXPECT_TRUE(rec.is_capturing_post());

  // Clock rolls back to t=5.0 (e.g. rosbag loop): must unstick capture & cooldown
  rec.record_scan(5.0, scan, ok_out);
  EXPECT_FALSE(rec.is_capturing_post());

  rec.trigger(5.1, sloc::BlackboxTrigger::kExternalRequest);
  EXPECT_TRUE(rec.is_capturing_post());
  rec.record_scan(7.2, scan, ok_out);
  rec.flush_and_wait();
  EXPECT_EQ(rec.dump_count(), 1u);
}

}  // namespace
