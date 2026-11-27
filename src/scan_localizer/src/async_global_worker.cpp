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

#include "scan_localizer/async_global_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace sloc
{
AsyncGlobalWorker::AsyncGlobalWorker(GlobalWorkerConfig cfg)
: cfg_(std::move(cfg))
{
  worker_ = std::thread(&AsyncGlobalWorker::worker_loop, this);
}

AsyncGlobalWorker::~AsyncGlobalWorker()
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = true;
    has_pending_ = false;
    pending_job_ = Job{};
  }
  job_cv_.notify_all();
  res_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AsyncGlobalWorker::update_config(const GlobalWorkerConfig & cfg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  cfg_ = cfg;
}

std::uint64_t AsyncGlobalWorker::submit(
  double stamp_s, std::shared_ptr<const MapField> field, Scan scan)
{
  std::uint64_t id = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (has_pending_) {
      dropped_.fetch_add(1u, std::memory_order_relaxed);
    }
    id = next_id_++;
    pending_job_.request_id = id;
    pending_job_.stamp_s = stamp_s;
    pending_job_.field = std::move(field);
    pending_job_.scan = std::move(scan);
    has_pending_ = true;
  }
  job_cv_.notify_one();
  return id;
}

bool AsyncGlobalWorker::poll_result(GlobalWorkerResult & out)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!has_result_) {
    return false;
  }
  out = std::move(latest_result_);
  has_result_ = false;
  return true;
}

bool AsyncGlobalWorker::wait_result(GlobalWorkerResult & out, int timeout_ms)
{
  std::unique_lock<std::mutex> lk(mtx_);
  const auto ok = res_cv_.wait_for(
    lk, std::chrono::milliseconds(std::max(1, timeout_ms)),
    [&]() {return has_result_ || stop_;});
  if (!ok || !has_result_) {
    return false;
  }
  out = std::move(latest_result_);
  has_result_ = false;
  return true;
}

void AsyncGlobalWorker::apply_thread_isolation()
{
#if defined(__linux__)
  std::vector<int> cpus;
  int nice_val = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cpus = cfg_.cpu_affinity;
    nice_val = cfg_.nice_value;
  }
  if (!cpus.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    const int max_cpus = static_cast<int>(CPU_SETSIZE);
    bool any_valid = false;
    for (int cpu : cpus) {
      if (cpu >= 0 && cpu < max_cpus) {
        CPU_SET(cpu, &set);
        any_valid = true;
      }
    }
    if (any_valid) {
      // 绑核失败（如容器限制或核号超限）时静默降级，不影响定位功能
      (void)pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &set);
    }
  }
  if (nice_val != 0) {
    const pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
    (void)setpriority(PRIO_PROCESS, static_cast<id_t>(tid), std::clamp(nice_val, -20, 19));
  }
#endif
}

void AsyncGlobalWorker::worker_loop()
{
  apply_thread_isolation();
  while (true) {
    Job job;
    GlobalWorkerConfig local_cfg;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      job_cv_.wait(lk, [&]() {return stop_ || has_pending_;});
      if (stop_) {
        break;
      }
      job = std::move(pending_job_);
      has_pending_ = false;
      local_cfg = cfg_;
      busy_.store(true, std::memory_order_release);
    }

    const auto t0 = std::chrono::steady_clock::now();
    GlobalResult gres;
    if (job.field && job.field->valid() && !job.scan.points.empty()) {
      const double ang_step =
        (std::isfinite(local_cfg.ang_step) && local_cfg.ang_step > 0.0) ?
        local_cfg.ang_step :
        compute_adaptive_ang_step(*job.field, job.scan);
      gres = global_correlative(
        *job.field, job.scan,
        local_cfg.beams_subsample,
        ang_step,
        local_cfg.n_hypotheses,
        local_cfg.max_time_ms);
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_result_.request_id = job.request_id;
      latest_result_.stamp_s = job.stamp_s;
      latest_result_.elapsed_ms = elapsed_ms;
      latest_result_.result = std::move(gres);
      has_result_ = true;
      busy_.store(false, std::memory_order_release);
    }
    completed_.fetch_add(1u, std::memory_order_release);
    res_cv_.notify_all();
  }
}

}  // namespace sloc
