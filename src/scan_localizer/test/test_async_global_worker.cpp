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

#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "scan_localizer/async_global_worker.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

TEST(AsyncGlobalWorkerTest, SolvesRelocalizationWithCpuPinningAndNicePriority)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  std::mt19937 rng(42);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.01;
  const sloc::Pose2 gt(6.0, 5.0, 0.7);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::GlobalWorkerConfig cfg;
  cfg.beams_subsample = 3;
  cfg.ang_step = 0.0;  // auto range-adaptive angular step
  cfg.n_hypotheses = 6;
  cfg.max_time_ms = 250.0;
  cfg.cpu_affinity = {0};
  cfg.nice_value = 10;

  sloc::AsyncGlobalWorker worker(cfg);
  const auto req_id = worker.submit(10.0, field, scan);
  EXPECT_EQ(req_id, 1u);

  sloc::GlobalWorkerResult out;
  ASSERT_TRUE(worker.wait_result(out, 5000));
  EXPECT_EQ(out.request_id, req_id);
  EXPECT_DOUBLE_EQ(out.stamp_s, 10.0);
  EXPECT_GT(out.elapsed_ms, 0.0);
  ASSERT_TRUE(out.result.valid);
  EXPECT_LT(std::hypot(out.result.pose.x - gt.x, out.result.pose.y - gt.y), 0.30);
  EXPECT_LT(std::abs(sloc::wrap_angle(out.result.pose.theta - gt.theta)), 0.20);
  EXPECT_EQ(worker.completed_count(), 1u);

  // Poll after wait must return false (result already consumed)
  sloc::GlobalWorkerResult empty_out;
  EXPECT_FALSE(worker.poll_result(empty_out));
}

TEST(AsyncGlobalWorkerTest, LatestOnlyQueueOverwritesStalePendingRequests)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  std::mt19937 rng(123);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.01;
  const auto scan = sloc::sim::make_scan(grid, sloc::Pose2(6.0, 5.0, 0.7), scfg, rng);

  sloc::GlobalWorkerConfig cfg;
  cfg.n_hypotheses = 4;
  cfg.max_time_ms = 200.0;
  sloc::AsyncGlobalWorker worker(cfg);

  // Rapid burst of 12 submissions must coalesce stale queued frames instead of building a backlog
  std::uint64_t last_id = 0;
  for (int i = 0; i < 12; ++i) {
    last_id = worker.submit(100.0 + i * 0.1, field, scan);
  }

  sloc::GlobalWorkerResult res;
  ASSERT_TRUE(worker.wait_result(res, 5000));
  // Drain until worker settles on the latest request
  while (res.request_id != last_id && worker.wait_result(res, 5000)) {}
  EXPECT_EQ(res.request_id, last_id);
  EXPECT_GT(worker.dropped_count(), 0u);
  EXPECT_LT(worker.completed_count(), 12u);
}

TEST(AsyncGlobalWorkerTest, DestructorDiscardsPendingJobAndExitsPromptly)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  std::mt19937 rng(7);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.01;
  const auto scan = sloc::sim::make_scan(grid, sloc::Pose2(6.0, 5.0, 0.7), scfg, rng);

  sloc::GlobalWorkerConfig cfg;
  cfg.n_hypotheses = 4;
  cfg.max_time_ms = 50.0;

  const auto t0 = std::chrono::steady_clock::now();
  {
    sloc::AsyncGlobalWorker worker(cfg);
    worker.submit(1.0, field, scan);
    worker.submit(2.0, field, scan);
    worker.submit(3.0, field, scan);
  }
  const double dt_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(dt_ms, 1000.0);
}

TEST(AsyncGlobalWorkerTest, UpdateConfigRefreshesIsolationAndBudget)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  std::mt19937 rng(9);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.01;
  const sloc::Pose2 gt(6.0, 5.0, 0.7);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::GlobalWorkerConfig cfg;
  cfg.max_time_ms = 250.0;
  sloc::AsyncGlobalWorker worker(cfg);

  cfg.cpu_affinity = {0};
  cfg.nice_value = 5;
  cfg.max_time_ms = 300.0;
  worker.update_config(cfg);

  worker.submit(20.0, field, scan);
  sloc::GlobalWorkerResult out;
  ASSERT_TRUE(worker.wait_result(out, 5000));
  ASSERT_TRUE(out.result.valid);
  EXPECT_LT(std::hypot(out.result.pose.x - gt.x, out.result.pose.y - gt.y), 0.30);
}

TEST(AsyncGlobalWorkerTest, InvalidateDiscardsStaleResultsAndTracksBusyState)
{
  const auto grid = sloc::sim::make_warehouse(0.05);
  auto field = std::make_shared<sloc::MapField>();
  ASSERT_TRUE(field->build(grid, 0.20));

  std::mt19937 rng(31);
  sloc::sim::ScanSimConfig scfg;
  scfg.beams = 360;
  scfg.noise_std = 0.01;
  const sloc::Pose2 gt(6.0, 5.0, 0.7);
  const auto scan = sloc::sim::make_scan(grid, gt, scfg, rng);

  sloc::GlobalWorkerConfig cfg;
  cfg.n_hypotheses = 4;
  cfg.max_time_ms = 250.0;
  sloc::AsyncGlobalWorker worker(cfg);

  const auto id1 = worker.submit(10.0, field, scan);
  EXPECT_TRUE(worker.is_busy());
  sloc::GlobalWorkerResult out;
  ASSERT_TRUE(worker.wait_result(out, 5000));
  EXPECT_EQ(out.request_id, id1);

  // Submit and invalidate immediately: stale result must not be delivered
  worker.submit(11.0, field, scan);
  worker.invalidate();
  EXPECT_FALSE(worker.wait_result(out, 150));

  // New submission after invalidate must succeed cleanly
  const auto id3 = worker.submit(12.0, field, scan);
  ASSERT_TRUE(worker.wait_result(out, 5000));
  EXPECT_EQ(out.request_id, id3);
  EXPECT_TRUE(out.result.valid);
}

}  // namespace
