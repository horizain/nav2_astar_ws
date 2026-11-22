#pragma once
// ============================================================================
// affinity.hpp —— CPU 亲和性工具（Linux）
//
// 低频实时工作负载（如 10-30Hz 激光 SLAM）的绑定策略：
//   • 把 SLAM 处理线程钉在固定物理核 → 避免迁移导致的缓存重热与延迟抖动
//   • 避开 cpu0/1（内核中断、ROS 基础设施常驻）
//   • 用 SMT 兄弟去重后的物理核编号（见 ficp::detail 的物理核检测）
// ============================================================================
#include <pthread.h>
#include <sched.h>

#include <thread>

namespace ficp::affinity {

// 把当前线程绑定到指定 CPU
[[nodiscard]] inline auto pin_this_thread(unsigned cpu) -> bool
{
#if defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<unsigned>(cpu), &set);
    return ::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set) == 0;
#else
    (void)cpu;
    return false;
#endif
}

// 把指定线程绑定到指定 CPU
[[nodiscard]] inline auto pin_thread(std::thread& t, unsigned cpu) -> bool
{
#if defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<unsigned>(cpu), &set);
    return ::pthread_setaffinity_np(t.native_handle(), sizeof(set), &set) == 0;
#else
    (void)t;
    (void)cpu;
    return false;
#endif
}

} // namespace ficp::affinity
