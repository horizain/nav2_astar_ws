// dbg —— 诊断：在给定位姿上直接统计 ICP 的对应点质量
//   1) 点到点 NN 距离（数据/坐标系是否正确）
//   2) 点到线残差（line 估计是否正确）
// 分别在真值位姿与错误收敛位姿上对比。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <print>
#include <random>
#include <thread>
#include <vector>

#include "affinity.hpp"
#include "scan_sim.hpp"

// fast_icp.cpp 内部 BucketKDTree 的性能诊断钩子（声明与定义一致）
namespace ficp::detail {
struct TreeBench {
    double seconds;
    double sink;
};
auto bench_tree(const Cloud& ref, const Cloud& queries, int iters) -> TreeBench;
auto check_tree(const Cloud& ref, const Cloud& queries) -> int;
} // namespace ficp::detail

namespace {

struct Stats {
    double mean{};
    double p95{};
    double max{};
    int n{};
};

auto residual_stats(const icp::PointCloud2& source, const icp::PointCloud2& target,
                    const icp::Pose2& pose, bool use_line) -> Stats
{
    const icp::KDTree tree{target};
    std::vector<double> rs;
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    for (const auto& p : source) {
        const icp::Point2 pw{pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
        const auto nn = tree.find2(pw);
        if (nn.nearest == nullptr) {
            continue;
        }
        const double dx = pw.x - nn.nearest->x;
        const double dy = pw.y - nn.nearest->y;
        const double d2 = dx * dx + dy * dy;
        if (!use_line) {
            rs.push_back(std::sqrt(d2));
            continue;
        }
        double r{};
        if (nn.second != nullptr) {
            const double vx = nn.second->x - nn.nearest->x;
            const double vy = nn.second->y - nn.nearest->y;
            const double norm = std::hypot(vx, vy);
            if (norm > 1e-12) {
                r = (-vy * dx + vx * dy) / norm;
            } else {
                r = std::sqrt(d2);
            }
        } else {
            r = std::sqrt(d2);
        }
        rs.push_back(std::abs(r));
    }
    std::ranges::sort(rs);
    Stats st;
    st.n = static_cast<int>(rs.size());
    double sum{};
    for (const double v : rs) {
        sum += v;
    }
    st.mean = sum / st.n;
    st.p95 = rs[static_cast<std::size_t>(0.95 * (st.n - 1))];
    st.max = rs.back();
    return st;
}

} // namespace

auto main() -> int
{
    const auto segs = sim::make_room();
    constexpr icp::Pose2 gt{8.0, 3.0, 2.8};
    constexpr icp::Pose2 ref_pose{2.5, 2.0, 0.9};
    // 参考实现上次"错误收敛"到的位姿
    constexpr icp::Pose2 wrong{8.1646, 3.0806, 2.8333};

    std::mt19937 rng{1234};
    auto target = sim::make_scan(segs, ref_pose, 1080, 0.01, rng);
    sim::to_world(target, ref_pose);
    const auto source = sim::make_scan(segs, gt, 1080, 0.01, rng);

    std::println("参考 {} 点, 源 {} 点", target.pc.size(), source.pc.size());

    // ── 数据自检：点是否真的贴在墙上（距最近线段 ≤ 噪声水平）──
    const auto seg_dist = [&](double x, double y) {
        double best = 1e9;
        for (const auto& s : segs) {
            const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
            const double len2 = ex * ex + ey * ey;
            double u = ((x - s.x0) * ex + (y - s.y0) * ey) / len2;
            u = std::clamp(u, 0.0, 1.0);
            best = std::min(best, std::hypot(x - (s.x0 + u * ex), y - (s.y0 + u * ey)));
        }
        return best;
    };
    {
        double sum{}, mx{};
        for (const auto& p : target.pc) {
            const double d = seg_dist(p.x, p.y);
            sum += d;
            mx = std::max(mx, d);
        }
        std::println("自检 参考点云贴墙距离: mean={:.4f} max={:.4f}", sum / target.pc.size(), mx);
    }
    {
        const double c = std::cos(gt.theta), s = std::sin(gt.theta);
        double sum{}, mx{};
        for (const auto& p : source.pc) {
            const double d = seg_dist(gt.x + c * p.x - s * p.y, gt.y + s * p.x + c * p.y);
            sum += d;
            mx = std::max(mx, d);
        }
        std::println("自检 源点云@真值贴墙距离: mean={:.4f} max={:.4f}", sum / source.pc.size(), mx);
    }

    for (const auto& [name, pose] :
         std::vector<std::pair<const char*, icp::Pose2>>{{"真值位姿", gt}, {"错误收敛位姿", wrong}}) {
        const auto pp = residual_stats(source.pc, target.pc, pose, false);
        const auto pl = residual_stats(source.pc, target.pc, pose, true);
        std::println("{}: 点到点 NN  mean={:.4f} p95={:.4f} max={:.4f}", name, pp.mean,
                     pp.p95, pp.max);
        std::println("{}: 点到线残差 mean={:.4f} p95={:.4f} max={:.4f} (n={})", name,
                     pl.mean, pl.p95, pl.max, pl.n);
    }

    // ── 场景 A / C 实跑 align，看收敛行为 ──
    const auto run = [&](double sigma, int beams, icp::Pose2 drift, const char* tag) {
        std::mt19937 r2{1234};
        auto tgt = sim::make_scan(segs, ref_pose, beams, sigma, r2);
        sim::to_world(tgt, ref_pose);
        const auto src = sim::make_scan(segs, gt, beams, sigma, r2);
        const icp::Pose2 init{gt.x + drift.x, gt.y + drift.y, gt.theta + drift.theta};

        std::println("\n[{}] σ={:.0f}mm beams={} 初值误差 ({:.2f},{:.2f},{:.2f})", tag,
                     sigma * 1000.0, beams, drift.x, drift.y, drift.theta);
        const auto res = icp::align(src.pc, tgt.pc, init);
        if (res) {
            std::println(
                "[{}] ref  收敛={} it={} corr={} rms={:.4f} err=({:.3f}mm, {:.3f}mrad, θ={:.4f})",
                tag, res->converged, res->iterations, res->correspondences, res->rms_error,
                std::hypot(res->pose.x - gt.x, res->pose.y - gt.y) * 1000.0,
                std::abs(std::remainder(res->pose.theta - gt.theta, 6.283)) * 1000.0,
                res->pose.theta);
        } else {
            std::println("[{}] ref  失败: {}", tag, icp::to_string(res.error()));
        }
        ficp::Engine engine(ficp::Params{.threads = 1});
        engine.set_reference(tgt.fc);
        const auto fr = engine.align(src.fc, init.x, init.y, init.theta);
        std::println(
            "[{}] fast 收敛={} it={} corr={} rms={:.4f} err=({:.3f}mm, {:.3f}mrad, θ={:.4f})",
            tag, fr.converged, fr.iterations, fr.correspondences, fr.rms,
            std::hypot(fr.x - gt.x, fr.y - gt.y) * 1000.0,
            std::abs(std::remainder(fr.theta - gt.theta, 6.283)) * 1000.0, fr.theta);
    };

    run(0.01, 1080, {.x = 0.10, .y = -0.06, .theta = 0.08}, "A");
    run(0.0001, 1080, {.x = 0.10, .y = -0.06, .theta = 0.08}, "A0(无噪声)");
    run(0.05, 720, {.x = 0.12, .y = 0.08, .theta = 0.10}, "C");
    run(0.02, 1080, {.x = 0.35, .y = 0.22, .theta = 0.25}, "B");

    // ── 性能剖析：KD-tree 查询吞吐 & 每轮迭代成本 ──
    {
        using clock = std::chrono::steady_clock;
        std::mt19937 r4{1234};
        auto tgt = sim::make_scan(segs, ref_pose, 1080, 0.01, r4);
        sim::to_world(tgt, ref_pose);
        const auto src = sim::make_scan(segs, gt, 1080, 0.01, r4);

        // 查询点：源点在真值位姿下的世界坐标
        const double c = std::cos(gt.theta), s = std::sin(gt.theta);
        icp::PointCloud2 qw;
        ficp::Cloud qf;
        for (std::size_t i = 0; i < src.pc.size(); ++i) {
            const double x = gt.x + c * src.pc[i].x - s * src.pc[i].y;
            const double y = gt.y + s * src.pc[i].x + c * src.pc[i].y;
            qw.push_back({x, y});
            qf.push_back(x, y);
        }

        {   // 参考 KDTree（AoS double，单点节点，递归）
            const icp::KDTree tree{tgt.pc};
            double sink = 0.0;
            constexpr int iters = 2000;
            const auto t0 = clock::now();
            for (int it = 0; it < iters; ++it) {
                for (const auto& q : qw) {
                    const auto nn = tree.find2(q);
                    sink += nn.nearest ? nn.nearest->x : 0.0;
                }
            }
            const double ns = std::chrono::duration<double, std::nano>(clock::now() - t0)
                                  .count() / (double(iters) * double(qw.size()));
            std::println("KD查询: ref  {:6.1f} ns/点  (sink {:.0f})", ns, sink);
        }
        {   // FastICP KDTreeF（double，扁平节点，迭代遍历）
            const auto tb = ficp::detail::bench_tree(tgt.fc, qf, 2000);
            std::println("KD查询: fast {:6.1f} ns/点  (sink {:.0f})",
                         tb.seconds / (2000.0 * double(qf.size())) * 1e9, tb.sink);
        }
        std::println("一致性: 两树最近/次近不一致 {} / {} 点",
                     ficp::detail::check_tree(tgt.fc, qf), qf.size());

        // 每轮迭代成本：tolerance=0 + stall 禁用，强制跑满 K 轮，K=4/16 求斜率
        const auto time_align = [&](int K, bool fast) {
            constexpr int reps = 100;
            const auto t0 = clock::now();
            if (fast) {
                ficp::Engine engine(ficp::Params{.max_iterations = K,
                                                .tolerance = 0.0,
                                                .stall_iters = 0,
                                                .threads = 1});
                engine.set_reference(tgt.fc);
                for (int i = 0; i < reps; ++i) {
                    (void)engine.align(src.fc, 8.1, 2.94, 2.88);
                }
            } else {
                const icp::AlignParams rp{.max_iterations = K, .tolerance = 0.0,
                                          .stall_iters = 0};
                for (int i = 0; i < reps; ++i) {
                    (void)icp::align(src.pc, tgt.pc, {8.1, 2.94, 2.88}, rp);
                }
            }
            return std::chrono::duration<double, std::micro>(clock::now() - t0).count() /
                   reps;
        };
        const double r4us = time_align(4, false);
        const double r16us = time_align(16, false);
        const double f4us = time_align(4, true);
        const double f16us = time_align(16, true);
        std::println("每轮迭代: ref {:7.1f} µs  |  fast(1T) {:7.1f} µs",
                     (r16us - r4us) / 12.0, (f16us - f4us) / 12.0);

        // ── 强扩展性：单核 → 32 线程 ──
        const auto scaling = [&](const ficp::Cloud& ref, const char* label) {
            std::println("\n强扩展性 [{}] 参考点 {}，源点 {}：", label, ref.size(),
                         src.fc.size());
            std::println("{:>6}{:>12}{:>12}{:>10}{:>9}", "线程", "均值µs", "最小µs", "加速比",
                         "效率%");
            double base = 0.0;
            for (const int th : {1, 2, 4, 8, 12, 16, 24, 32}) {
                ficp::Engine engine(ficp::Params{.threads = th});
                engine.set_reference(ref);
                (void)engine.align(src.fc, 8.1, 2.94, 2.88);  // warmup
                constexpr int reps = 300;
                double total = 0.0;
                double best = 1e9;
                for (int i = 0; i < reps; ++i) {
                    const auto t0 = clock::now();
                    (void)engine.align(src.fc, 8.1, 2.94, 2.88);
                    const double us =
                        std::chrono::duration<double, std::micro>(clock::now() - t0).count();
                    total += us;
                    best = std::min(best, us);
                }
                const double mean = total / reps;
                if (th == 1) {
                    base = mean;
                }
                std::println("{:>6}{:>12.1f}{:>12.1f}{:>10.2f}{:>8.0f}", th, mean, best,
                             base / mean, 100.0 * base / mean / th);
            }
        };
        scaling(tgt.fc, "扫描vs扫描");

        // 参考地图：5 个视角拼接 ≈ 5400 点（模拟建好的局部地图）
        ficp::Cloud map_cloud;
        map_cloud.reserve(5400);
        for (const auto& rp : {icp::Pose2{2.5, 2.0, 0.9}, icp::Pose2{6.0, 1.5, 2.2},
                               icp::Pose2{9.0, 4.0, 3.5}, icp::Pose2{4.0, 6.0, 5.0},
                               icp::Pose2{7.5, 5.5, 0.4}}) {
            std::mt19937 rmap{77};
            auto part = sim::make_scan(segs, rp, 1080, 0.01, rmap);
            sim::to_world(part, rp);
            for (std::size_t i = 0; i < part.fc.size(); ++i) {
                map_cloud.push_back(part.fc.x[i], part.fc.y[i]);
            }
        }
        scaling(map_cloud, "扫描vs地图");

        // ── 杠杆4/1：大地图（~108k 点，真实 SLAM 规模）──
        ficp::Cloud big_map;
        big_map.reserve(108000);
        for (int k = 0; k < 100; ++k) {
            const icp::Pose2 rp{0.8 + (k % 10) * 1.15, 0.8 + (k / 10) * 0.7,
                                0.61 * static_cast<double>(k)};
            std::mt19937 rbig{77u + static_cast<unsigned>(k)};
            auto part = sim::make_scan(segs, rp, 1080, 0.01, rbig);
            sim::to_world(part, rp);
            for (std::size_t i = 0; i < part.fc.size(); ++i) {
                big_map.push_back(part.fc.x[i], part.fc.y[i]);
            }
        }
        std::println("\n大地图 {} 点，源 {} 点（cache 开/关 × 1/16 线程）：",
                     big_map.size(), src.fc.size());
        for (const bool cache : {false, true}) {
            for (const int th : {1, 16}) {
                ficp::Engine engine(
                    ficp::Params{.cache_correspondences = cache, .threads = th});
                engine.set_reference(big_map);
                (void)engine.align(src.fc, 8.1, 2.94, 2.88);  // warmup
                constexpr int reps = 100;
                double total = 0.0;
                double best = 1e9;
                for (int i = 0; i < reps; ++i) {
                    const auto t0 = clock::now();
                    (void)engine.align(src.fc, 8.1, 2.94, 2.88);
                    const double us =
                        std::chrono::duration<double, std::micro>(clock::now() - t0)
                            .count();
                    total += us;
                    best = std::min(best, us);
                }
                std::println("  cache={} {:>2}线程: 均值 {:8.1f} µs, 最小 {:8.1f} µs",
                             cache ? "开" : "关", th, total / reps, best);
            }
        }

        // ── 杠杆2 的天花板：完美初值能省多少轮 ──
        {
            ficp::Engine engine(ficp::Params{.threads = 16});
            engine.set_reference(tgt.fc);
            const auto r = engine.align(src.fc, gt.x, gt.y, gt.theta);
            constexpr int reps = 300;
            const auto t0 = clock::now();
            for (int i = 0; i < reps; ++i) {
                (void)engine.align(src.fc, gt.x, gt.y, gt.theta);
            }
            const double mean =
                std::chrono::duration<double, std::micro>(clock::now() - t0).count() /
                reps;
            std::println("\n完美初值: {} 轮收敛（漂移初值是 7 轮），16线程均值 {:.1f} µs",
                         r.iterations, mean);
        }

        // ── 帧率预算验证：30Hz 到帧间隔下的冷启动单帧延迟（含线程池唤醒）──
        {
            using namespace std::chrono_literals;
            // 模拟低频部署：SLAM 主线程钉 cpu3，唯一 worker 钉 cpu2
            (void)ficp::affinity::pin_this_thread(3);
            ficp::Engine engine(ficp::Params{.threads = 2, .pin_base_core = 2});
            engine.set_reference(tgt.fc);
            (void)engine.align(src.fc, 8.1, 2.94, 2.88);  // warmup
            constexpr int frames = 40;
            double total = 0.0;
            double worst = 0.0;
            for (int f = 0; f < frames; ++f) {
                std::this_thread::sleep_for(33ms);  // 30Hz 到帧间隔，worker 深度挂起
                const auto t0 = clock::now();
                (void)engine.align(src.fc, 8.1, 2.94, 2.88);
                const double us =
                    std::chrono::duration<double, std::micro>(clock::now() - t0).count();
                total += us;
                worst = std::max(worst, us);
            }
            std::println(
                "\n30Hz 帧率模式（2线程, 核绑定 cpu2/3）: 均值 {:.0f} µs, "
                "最差 {:.0f} µs — 占 33.3ms 帧预算 {:.2f}%",
                total / frames, worst, 100.0 * worst / 33300.0);
        }
    }
    return 0;
}
