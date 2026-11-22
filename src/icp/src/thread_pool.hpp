#pragma once
// ============================================================================
// ForkJoinPool —— 轻量 fork-join 线程池（spin-then-park 策略）
//
// 为 ICP 的迭代式并行设计：每轮 Gauss-Newton 都要并行做一次对应点搜索，
// 两次并行之间只隔几十微秒。通用线程池（cv 唤醒 ~10-50μs）在这样高的
// 同步频率下唤醒开销会吃掉并行收益，因此：
//   • 任务到来时 worker 先自旋等待（~200μs 窗口）——ICP 迭代期间
//     worker 始终保持热态，轮间同步只剩几次原子操作；
//   • 长时间无任务才挂到 condition_variable 上（1ms 超时兜底防丢唤醒）。
// 用法：parallel_chunks(n, fn)，fn(chunk_id, begin, end) 处理 [begin,end)。
// 限制：同一时刻只允许一个线程调用（bench 中由主线程独占使用）。
// ============================================================================
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define ICP_PAUSE() _mm_pause()
#elif defined(__aarch64__) || defined(__arm__)
// ARMv8 yield 指令：给同核兄弟线程让流水线资源，功耗语义同 _mm_pause
#define ICP_PAUSE() ::__asm__ volatile("yield")
#else
#define ICP_PAUSE() std::this_thread::yield()
#endif

#include "affinity.hpp"

namespace icp {

class ForkJoinPool {
public:
    explicit ForkJoinPool(unsigned threads = std::thread::hardware_concurrency(),
                          int pin_base = -1)  // ≥0：worker 从该核开始轮转绑定
    {
        if (threads > 128) {
            threads = 128;  // nn2() 手动遍历栈的安全余量与调度开销权衡
        }
        for (unsigned i = 0; i < threads; ++i) {
            workers_.emplace_back([this, i, pin_base] { worker_loop(i, pin_base); });
        }
        // 等待所有 worker 完成初始化（读取 gen 并登记 ready）。
        // 没有这个闩锁，主线程第一轮任务可能与 worker 启动竞争：
        // worker 若在 gen 首次递增之后才读取初值，会以为该轮已处理，
        // done_ 永不递增 → 主线程死等（绑核到冷核时近乎必现）
        while (ready_.load(std::memory_order_acquire) != threads) {
            ICP_PAUSE();
        }
    }

    ~ForkJoinPool()
    {
        {
            std::lock_guard lk(mu_);
            stop_ = true;
        }
        cv_.notify_all();
        workers_.clear();  // jthread 析构自动 join
    }

    ForkJoinPool(const ForkJoinPool&) = delete;
    auto operator=(const ForkJoinPool&) -> ForkJoinPool& = delete;

    [[nodiscard]] unsigned num_workers() const
    {
        return static_cast<unsigned>(workers_.size());
    }

    // 把 [0, n) 分成若干 chunk 并行执行 fn(chunk_id, begin, end)。
    // 主线程也参与执行；返回时全部完成。返回实际使用的 chunk 数。
    // 小任务（n < kMinParallel）或无线程时直接内联执行。
    template <typename F>
    auto parallel_chunks(std::size_t n, F&& fn) -> std::size_t
    {
        if (n == 0) {
            return 0;
        }
        if (workers_.empty() || n < kMinParallel) {
            fn(std::size_t{0}, std::size_t{0}, n);
            return 1;
        }
        const std::size_t nchunks =
            std::min<std::size_t>(n, 4 * (workers_.size() + 1));
        chunk_ = (n + nchunks - 1) / nchunks;
        n_ = n;
        nchunks_ = nchunks;
        fn_ = [&fn, this](std::size_t ci) {
            const std::size_t b = ci * chunk_;
            const std::size_t e = std::min(b + chunk_, n_);
            fn(ci, b, e);
        };
        next_.store(0, std::memory_order_relaxed);
        done_.store(0, std::memory_order_relaxed);
        gen_.fetch_add(1, std::memory_order_acq_rel);  // release：任务数据就绪
        {
            std::lock_guard lk(mu_);
            cv_.notify_all();  // 唤醒可能挂起的 worker
        }
        run_chunks();  // 主线程也干活
        while (done_.load(std::memory_order_acquire) != workers_.size()) {
            ICP_PAUSE();
        }
        return nchunks;
    }

private:
    void worker_loop(unsigned id, int pin_base)
    {
        if (pin_base >= 0) {
            (void)ficp::affinity::pin_this_thread(static_cast<unsigned>(pin_base) + id);
        }
        std::uint64_t seen = gen_.load(std::memory_order_acquire);
        // 初始化登记：构造函数等所有 worker 到齐后才允许第一轮任务
        ready_.fetch_add(1, std::memory_order_release);
        for (;;) {
            if (stop_.load(std::memory_order_acquire)) {
                return;
            }
            if (const std::uint64_t g = gen_.load(std::memory_order_acquire);
                g != seen) {
                seen = g;
                run_chunks();
                done_.fetch_add(1, std::memory_order_acq_rel);
                continue;
            }
            // 自旋窗口：下一轮迭代马上就来，别睡
            bool woke = false;
            for (int i = 0; i < kSpinIters; ++i) {
                if (gen_.load(std::memory_order_acquire) != seen ||
                    stop_.load(std::memory_order_relaxed)) {
                    woke = true;
                    break;
                }
                ICP_PAUSE();
            }
            if (woke) {
                continue;
            }
            // 长闲：挂起等唤醒（无界等待——唤醒协议由 mutex+predicate 保证
            // 无丢失唤醒，超时轮询会让空闲 worker 每 2ms 自旋一轮烧 ~50% 核）
            std::unique_lock lk(mu_);
            cv_.wait(lk, [&] {
                return gen_.load(std::memory_order_acquire) != seen ||
                       stop_.load(std::memory_order_acquire);
            });
        }
    }

    void run_chunks()
    {
        for (;;) {
            const std::size_t ci = next_.fetch_add(1, std::memory_order_relaxed);
            if (ci >= nchunks_) {
                break;
            }
            fn_(ci);
        }
    }

    static constexpr std::size_t kMinParallel = 256;
    static constexpr int kSpinIters = 100000;  // ≈1ms @ _mm_pause：盖住 ICP 轮间空隙

    // 热同步数据：缓存行隔离，避免伪共享
    alignas(64) std::atomic<std::uint64_t> gen_{0};
    alignas(64) std::atomic<std::size_t> next_{0};
    alignas(64) std::atomic<std::size_t> done_{0};
    alignas(64) std::atomic<unsigned> ready_{0};  // worker 初始化闩锁

    // 任务描述（由 gen_ 的 release/acquire 定序保护）
    std::size_t chunk_{1};
    std::size_t n_{0};
    std::size_t nchunks_{1};
    std::function<void(std::size_t)> fn_;

    std::mutex mu_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
    std::vector<std::jthread> workers_;
};

} // namespace icp
