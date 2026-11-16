#pragma once
// ============================================================================
// 特性探测输出头：统一 println/print 入口。
// GCC 14+ 提供 <print>（std::println）；GCC 13 只有 <format>，回退 vformat+cout。
// 这正是 C++ 生态兼容新特性的标准姿势：__has_include + 特性测试宏。
// ============================================================================
#include <format>
#include <iostream>
#include <utility>

#if defined(__cpp_lib_print)
  #include <print>
  #define MINI_SLAM_HAS_PRINT 1
#endif

namespace mini_slam::io {

#if defined(MINI_SLAM_HAS_PRINT)

template <typename... Args>
void println(std::format_string<Args...> fmt, Args&&... args) {
  std::println(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void print(std::format_string<Args...> fmt, Args&&... args) {
  std::print(fmt, std::forward<Args>(args)...);
}

#else  // GCC 13：std::format + std::cout

template <typename... Args>
void println(std::format_string<Args...> fmt, Args&&... args) {
  std::cout << std::vformat(fmt.get(), std::make_format_args(args...)) << '\n';
}

template <typename... Args>
void print(std::format_string<Args...> fmt, Args&&... args) {
  std::cout << std::vformat(fmt.get(), std::make_format_args(args...));
}

#endif

}  // namespace mini_slam::io
