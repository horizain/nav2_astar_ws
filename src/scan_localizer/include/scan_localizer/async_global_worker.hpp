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
// AsyncGlobalWorker —— 异步全局重定位隔离工作线程（ROS 无关）
//
// 面向 RK3399 等异构多核边缘端设计：
// - 常驻单工作线程，消除频繁创建/销毁线程的开销；
// - Latest-Only 单槽覆盖队列：重定位耗时期间积压的旧扫描直接丢弃，只处理最新帧；
// - 支持 Linux pthread_setaffinity_np 绑核与 nice 降优先级，避免抢占 TRACKING
//   与底盘控制回路；
// - 透传 max_time_ms 墙钟熔断预算至 global_correlative，锁死 P99 长尾耗时。
// ============================================================================
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "scan_localizer/matchers.hpp"

namespace sloc
{

struct GlobalWorkerConfig
{
  int beams_subsample{3};
  double ang_step{0.0};            // <=0 表示按有效射程 p90 与 σ 自动推导
  int n_hypotheses{8};
  double max_time_ms{220.0};       // 墙钟熔断预算 (ms)，<=0 表示不限
  std::vector<int> cpu_affinity;   // Linux 绑核列表（例如 RK3399 可设 {5} 或 {2, 3}）
  int nice_value{10};              // Linux 调度 nice 降权值（越大优先级越低，0 表示不调整）
};

struct GlobalWorkerResult
{
  std::uint64_t request_id{0};
  double stamp_s{0.0};
  double elapsed_ms{0.0};
  GlobalResult result{};
};

class AsyncGlobalWorker {
public:
  explicit AsyncGlobalWorker(GlobalWorkerConfig cfg = {});
  ~AsyncGlobalWorker();

  AsyncGlobalWorker(const AsyncGlobalWorker &) = delete;
  AsyncGlobalWorker & operator=(const AsyncGlobalWorker &) = delete;

  // 实时线程调用：提交全局重定位请求（若前一待处理槽尚未开始，直接覆盖防积压）
  std::uint64_t submit(
    double stamp_s, std::shared_ptr<const MapField> field, Scan scan);

  // 实时线程调用：非阻塞拉取最新完成的重定位结果
  bool poll_result(GlobalWorkerResult & out);

  // 同步等待下一份结果（供测试或阻塞式恢复调用）
  bool wait_result(GlobalWorkerResult & out, int timeout_ms = 5000);

  // 动态更新参数（下一次搜索生效）
  void update_config(const GlobalWorkerConfig & cfg);

  bool is_busy() const {return busy_.load(std::memory_order_acquire);}
  std::uint64_t dropped_count() const {return dropped_.load(std::memory_order_acquire);}
  std::uint64_t completed_count() const {return completed_.load(std::memory_order_acquire);}

private:
  struct Job
  {
    std::uint64_t request_id{0};
    double stamp_s{0.0};
    std::shared_ptr<const MapField> field;
    Scan scan;
  };

  void apply_thread_isolation();
  void worker_loop();

  std::mutex mtx_;
  std::condition_variable job_cv_;
  std::condition_variable res_cv_;
  GlobalWorkerConfig cfg_;
  bool stop_{false};
  bool has_pending_{false};
  Job pending_job_{};
  bool has_result_{false};
  GlobalWorkerResult latest_result_{};
  std::uint64_t next_id_{1};

  std::atomic<bool> busy_{false};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> completed_{0};
  std::thread worker_;
};

}  // namespace sloc
