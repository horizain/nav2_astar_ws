// ============================================================================
// 特性探针：写算法代码之前，先确认这套工具链能吃到哪些 C++23 / Ceres / Eigen 能力。
// 必需项失败 → 退出码 1，后续代码需要调整；可选项只汇报不阻塞。
// ============================================================================
#include "mini_slam/detail/format.hpp"

#include <cmath>
#include <expected>
#include <ranges>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace {

int g_passed = 0;
int g_failed = 0;

void check(bool ok, const char* name) {
  if (ok) {
    ++g_passed;
    mini_slam::io::println("  [PASS] {}", name);
  } else {
    ++g_failed;
    mini_slam::io::println("  [FAIL] {}", name);
  }
}

// ---- C++23 旗舰特性：deducing this（显式对象形参）----
// fluent 链式调用不再需要 CRTP：模板里用 `this Self&& self` 接住值类别。
#if defined(__cpp_deduction_this)
struct Fluent {
  double v{0.0};
  template <typename Self>
  constexpr auto&& add(this Self&& self, double x) {
    self.v += x;
    return std::forward<Self>(self);
  }
};
#endif

// ---- Ceres autodiff 探针代价函数 ----
struct ProbeEdge {
  template <typename T>
  bool operator()(const T* const a, const T* const b, T* r) const {
    r[0] = b[0] - a[0] - T(1.0);
    r[1] = b[1] - a[1] - T(0.5);
    r[2] = b[2] - a[2];
    return true;
  }
};

}  // namespace

int main() {
  mini_slam::io::println("== feature probe ==");

  // --- 语言/库基线（必需） ---
  check(__cpp_lib_format >= 202110L, "std::format（编译期格式串检查）");
  check(__cpp_lib_expected >= 202202L, "std::expected");
  check(__cpp_lib_span >= 202002L, "std::span");

  // --- deducing this（GCC 14+ 才有，本机 GCC 13.3 → 可选项） ---
#if defined(__cpp_deduction_this)
  static_assert(Fluent{}.add(1.0).add(2.0).v == 3.0);
  check(true, "deducing this（C++23）");
#else
  mini_slam::io::println("  [info] deducing this 需要 GCC 14+（本机为 GCC 13.3），代码用普通写法");
#endif

  // --- 可选特性：仅汇报 ---
#if defined(__cpp_lib_print)
  check(true, "std::print / std::println（GCC 14+）");
#else
  mini_slam::io::println("  [info] std::print 不可用（GCC 13），println 回退 format+cout");
#endif

#if defined(__cpp_lib_ranges_to)
  check(true, "std::ranges::to");
#else
  mini_slam::io::println("  [info] ranges::to 不可用（GCC 14 才有），用手动收集代替");
#endif

#if defined(__cpp_lib_zip)
  check(true, "views::zip");
#else
  mini_slam::io::println("  [info] views::zip 不可用");
#endif

  // --- span + ranges 实跑 ---
  {
    const std::vector<int> v{1, 2, 3, 4, 5, 6};
    const std::span s{v};
    int sum = 0;
    for (const int x : s | std::views::filter([](int x) { return x % 2 == 0; })) {
      sum += x;
    }
    check(sum == 12, "span + views::filter 实跑");
  }

  // --- std::expected 单子链式调用实跑 ---
  {
    std::expected<int, std::string> e = 7;
    e = e.and_then([](int x) { return std::expected<int, std::string>(x * 2); });
    check(e.value_or(-1) == 14, "std::expected and_then 链式");
  }

  // --- Eigen LLT：信息矩阵 → sqrt_info 的关键路径 ---
  {
    const Eigen::Matrix3d m = Eigen::Matrix3d::Identity() * 4.0;
    const Eigen::LLT<Eigen::Matrix3d> llt(m);
    check(llt.matrixU()(0, 0) == 2.0, "Eigen LLT（info → sqrt_info）");
  }

  // --- Ceres：autodiff + 稀疏求解器实跑（slam_toolbox 的 SPA 用同一套） ---
  {
    double a[3] = {0.1, -0.2, 0.3};
    double b[3] = {0.9, 0.7, 0.1};
    ceres::Problem problem;
    problem.AddParameterBlock(a, 3);
    problem.AddParameterBlock(b, 3);
    problem.SetParameterBlockConstant(a);  // 固定首节点消除规范自由度
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ProbeEdge, 3, 3, 3>(new ProbeEdge),
        nullptr, a, b);
    ceres::Solver::Options opt;
    opt.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opt.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(opt, &problem, &summary);
    check(summary.IsSolutionUsable() && std::abs(b[0] - a[0] - 1.0) < 1e-8,
          "ceres autodiff + SPARSE_NORMAL_CHOLESKY");
    mini_slam::io::println("  ceres: cost {:.3e} -> {:.3e}", summary.initial_cost,
                           summary.final_cost);
  }

  mini_slam::io::println("== probe done: {} passed, {} failed ==", g_passed, g_failed);
  return g_failed == 0 ? 0 : 1;
}
