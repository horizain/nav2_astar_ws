// ============================================================================
// bench —— FastICP vs mini_slam CorrelativeMatcher：速度与配准度对比
//
// 公平性约定：
//   • 同一份合成数据（射线求交仿真 270° 激光，带高斯点噪声）
//   • 同一初始位姿（真值 + 漂移，模拟里程计先验误差）
//   • 同一编译器与优化 flags（g++-14, -O3 -march=native）
//   • mini_slam 匹配窗口按场景初始误差设置，能罩住初值误差
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <print>
#include <random>
#include <span>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "fast_icp.hpp"
#include "icp.hpp"
#include "mini_slam/correlative_matcher.hpp"
#include "mini_slam/likelihood_field.hpp"
#include "scan_sim.hpp"

namespace {

// 计时：预热一次后重复，返回 {均值, 最小值}（单位 µs）
template <typename F>
auto measure_us(F&& f, int repeats) -> std::pair<double, double>
{
    using clock = std::chrono::steady_clock;
    f();  // warmup（顺带预热线程池 / cache）
    double total = 0.0;
    double best = 1e9;
    for (int i = 0; i < repeats; ++i) {
        const auto t0 = clock::now();
        f();
        const double us =
            std::chrono::duration<double, std::micro>(clock::now() - t0).count();
        total += us;
        best = std::min(best, us);
    }
    return {total / static_cast<double>(repeats), best};
}

struct Scenario {
    std::string_view name;
    double drift_x, drift_y, drift_theta;  // 初始位姿误差（模拟里程计漂移）
    double sigma;                          // 点噪声 (m)
    int beams;
    mini_slam::MatchWindow win;            // mini_slam 搜索窗口
};

struct Row {
    std::string_view algo;
    double build_us{};      // 参考结构构建（KD-tree / 似然场），0 = 每次重建
    double mean_us{};
    double min_us{};
    double trans_mm{};      // 平移误差
    double ang_mrad{};      // 角度误差
    int iters{};
    bool ok{};
};

auto angle_err(double a, double b) -> double
{
    return std::abs(std::remainder(a - b, 2.0 * std::numbers::pi));
}

} // namespace

auto main() -> int
{
    const auto segs = sim::make_room();
    // 真值位姿（世界系）：源传感器在房间右侧回望
    constexpr icp::Pose2 gt{8.0, 3.0, 2.8};
    // 参考扫描的传感器位姿：房间内部，270° 视野全覆盖
    constexpr icp::Pose2 ref_pose{2.5, 2.0, 0.9};

    std::println("════════ ICP vs mini_slam CorrelativeMatcher 基准测试 ════════");
    std::println("CPU: {} 线程 | g++-14 -O3 -march=native | C++23\n",
                 std::thread::hardware_concurrency());

    const std::vector<Scenario> scenarios{
        {.name = "A 小漂移+低噪声 (里程计正常)",
         .drift_x = 0.10,
         .drift_y = -0.06,
         .drift_theta = 0.08,
         .sigma = 0.01,
         .beams = 1080,
         .win = {.lin = 0.15, .ang = 0.2, .lin_step = 0.15, .ang_step = 0.05}},
        {.name = "B 大漂移 (接近窗口边界)",
         .drift_x = 0.35,
         .drift_y = 0.22,
         .drift_theta = 0.25,
         .sigma = 0.02,
         .beams = 1080,
         .win = {.lin = 0.4, .ang = 0.5, .lin_step = 0.1, .ang_step = 0.05}},
        {.name = "C 强噪声",
         .drift_x = 0.12,
         .drift_y = 0.08,
         .drift_theta = 0.10,
         .sigma = 0.05,
         .beams = 720,
         .win = {.lin = 0.2, .ang = 0.25, .lin_step = 0.1, .ang_step = 0.05}},
    };

    std::vector<Row> all_rows;  // 汇总表用

    for (const auto& sc : scenarios) {
        std::mt19937 rng{1234};
        const icp::Pose2 init{gt.x + sc.drift_x, gt.y + sc.drift_y,
                              gt.theta + sc.drift_theta};
        auto target = sim::make_scan(segs, ref_pose, sc.beams, sc.sigma, rng);
        sim::to_world(target, ref_pose);  // 参考点云统一到世界系（两种算法同帧对比）
        const auto source = sim::make_scan(segs, gt, sc.beams, sc.sigma, rng);

        // mini_slam 用的 Vec2 视图
        std::vector<mini_slam::Vec2> tv(target.pc.size());
        std::vector<mini_slam::Vec2> sv(source.pc.size());
        std::ranges::transform(target.pc, tv.begin(),
                               [](const icp::Point2& p) { return mini_slam::Vec2{p.x, p.y}; });
        std::ranges::transform(source.pc, sv.begin(),
                               [](const icp::Point2& p) { return mini_slam::Vec2{p.x, p.y}; });

        std::println("─────────────────────────────────────────────────────");
        std::println("场景 {} | {} 束 → 参考 {} 点 / 源 {} 点 | σ={:.0f}mm | 初值误差 {:.2f}m/{:.2f}rad",
                     sc.name, sc.beams, target.pc.size(), source.pc.size(),
                     sc.sigma * 1000.0, std::hypot(sc.drift_x, sc.drift_y),
                     std::abs(sc.drift_theta));
        std::println("{:<26}{:>10}{:>11}{:>11}{:>11}{:>11}{:>7}",
                     "算法", "构建µs", "均值µs", "最小µs", "平移mm", "角度mrad", "迭代");

        std::vector<Row> rows;

        // ── 1. 参考实现 icp::align（AoS double，每次调用内部重建 KD-tree）──
        {
            const icp::AlignParams rp{.max_correspondence_dist = 2.0};
            const auto [mean, mn] = measure_us(
                [&] { (void)icp::align(source.pc, target.pc, init, rp); }, 100);
            const auto res = icp::align(source.pc, target.pc, init, rp);
            const bool ok = res.has_value() && res->converged;
            rows.push_back({.algo = "icp::align ref (含建树)",
                            .mean_us = mean,
                            .min_us = mn,
                            .trans_mm = ok ? std::hypot(res->pose.x - gt.x, res->pose.y - gt.y) * 1000.0 : -1,
                            .ang_mrad = ok ? angle_err(res->pose.theta, gt.theta) * 1000.0 : -1,
                            .iters = ok ? res->iterations : 0,
                            .ok = ok});
        }

        // ── 2/3/4. FastICP 单线程 / 16 物理核 / 32 逻辑线程 ──
        for (const int threads : {1, 16, 32}) {
            ficp::Engine engine(ficp::Params{.threads = threads});
            engine.set_reference(target.fc);
            const auto [bmean, bmn] = measure_us([&] { engine.set_reference(target.fc); }, 20);
            const auto [mean, mn] =
                measure_us([&] { (void)engine.align(source.fc, init.x, init.y, init.theta); },
                           threads == 1 ? 200 : 200);
            const auto res = engine.align(source.fc, init.x, init.y, init.theta);
            rows.push_back({.algo = threads == 1 ? "FastICP 1线程"
                                : threads == 16 ? "FastICP 16线程(物理)"
                                                : "FastICP 32线程(逻辑)",
                            .build_us = bmean,
                            .mean_us = mean,
                            .min_us = mn,
                            .trans_mm = res.converged
                                            ? std::hypot(res.x - gt.x, res.y - gt.y) * 1000.0
                                            : -1,
                            .ang_mrad = res.converged
                                            ? angle_err(res.theta, gt.theta) * 1000.0
                                            : -1,
                            .iters = res.iterations,
                            .ok = res.converged});
            (void)bmn;
        }

        // ── 4. mini_slam CorrelativeMatcher（Olson 两级穷举 + 似然场）──
        {
            auto field = [&] {
                std::vector<mini_slam::FramedScan> refs{
                    {.pose = {0.0, 0.0, 0.0}, .points = tv}};
                return mini_slam::LikelihoodField::build(refs, 0.05, 1.0, 4).value();
            }();
            const auto [bmean, bmn] = measure_us(
                [&] {
                    std::vector<mini_slam::FramedScan> refs{
                        {.pose = {0.0, 0.0, 0.0}, .points = tv}};
                    (void)mini_slam::LikelihoodField::build(refs, 0.05, 1.0, 4);
                },
                20);            const mini_slam::Pose2 prior{init.x, init.y, init.theta};
            const auto [mean, mn] = measure_us(
                [&] { (void)mini_slam::CorrelativeMatcher::match(field, prior, sv, sc.win); },
                25);
            const auto res = mini_slam::CorrelativeMatcher::match(field, prior, sv, sc.win);
            rows.push_back({.algo = "mini_slam correlative",
                            .build_us = bmean,
                            .mean_us = mean,
                            .min_us = mn,
                            .trans_mm = res.valid
                                            ? std::hypot(res.pose.x - gt.x, res.pose.y - gt.y) * 1000.0
                                            : -1,
                            .ang_mrad = res.valid
                                            ? angle_err(res.pose.theta, gt.theta) * 1000.0
                                            : -1,
                            .iters = 2,  // 粗+细两级
                            .ok = res.valid});
            (void)bmn;
        }

        for (const auto& r : rows) {
            if (r.ok) {
                std::println("{:<26}{:>10.1f}{:>11.1f}{:>11.1f}{:>11.2f}{:>11.2f}{:>7}",
                             r.algo, r.build_us, r.mean_us, r.min_us, r.trans_mm,
                             r.ang_mrad, r.iters);
            } else {
                std::println("{:<26}  失败（未收敛/无有效匹配）", r.algo);
            }
        }
        std::println("");
        all_rows.insert(all_rows.end(), rows.begin(), rows.end());
    }

    std::println("═══════════════════ 汇总（场景 A 均值） ═══════════════════");
    return 0;
}
