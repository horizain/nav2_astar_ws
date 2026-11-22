// ============================================================================
// laser_prep_lab：CARMEN 日志 → 完整清洗链 → 统计 + 可视化。
//
// 全链（每帧）：
//   clean_frame ①-④ → deskew（odom 插值逐束校正）
//   → icp::align 对着融合局部地图求本帧世界位姿（odom 差给初值）
//   → cluster_by_break → judge_clusters（透视记忆判动态）
//   → record_frame 记透视 → ScanAccumulator 融合
//
// 可视化产物（--out 目录，浏览器打开 index.html 一次全看）：
//   index.html          带说明的图目录（其余 SVG 都在里面引用）
//   frame_best.svg      "删除最多的一帧"：灰=原始、蓝=保留、
//                       红=鬼点、橙=椒盐、紫=阴影、深灰=门控
//   ghost_zoom.svg      鬼点最密集处的局部放大（同一个语义，看得清）
//   deskew_best.svg     "跑得最快的一帧"：灰=不校正、蓝=逐束校正
//   frame_NNNN.svg      --plot-scan 指定的帧（同 frame_best 语义）
//   world_view.svg      全程总览：蓝=静态点、红=动态簇、绿=轨迹
//   report.txt          逐类统计
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <string>
#include <vector>

#include "accumulate.hpp"
#include "carmen.hpp"
#include "cluster.hpp"
#include "deskew.hpp"
#include "dynamic.hpp"
#include "filters.hpp"
#include "icp.hpp"
#include "polar.hpp"
#include "viz.hpp"

using namespace lpl;

namespace {

struct Args {
    std::string input;
    std::string out{"out"};
    double scan_duration{0.0};  // 0 = 按"相邻扫描间隔×0.25"估
    long max_scans{0};
    long plot_scan{200};
    bool show_help{false};
};

Args parse_args(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string s = argv[i];
        auto next = [&]() -> std::string {
            return (i + 1 < argc) ? argv[++i] : "";
        };
        if (s == "--input" || s == "-i") a.input = next();
        else if (s == "--out" || s == "-o") a.out = next();
        else if (s == "--scan-duration") {
            try { a.scan_duration = std::stod(next()); } catch (...) {}
        }
        else if (s == "--max-scans") a.max_scans = std::atol(next().c_str());
        else if (s == "--plot-scan") a.plot_scan = std::atol(next().c_str());
        else if (s == "--help" || s == "-h") a.show_help = true;
    }
    return a;
}

// 供可视化的一帧快照：只给被选中的帧用（内存拷贝很小）
struct FrameViz {
    bool used{false};
    long idx{};
    icp::Pose2 pose{};
    icp::PointCloud2 raw;      // 清洗前的原始端点（传感器系）
    icp::PointCloud2 kept;      // 清洗+deskew 后保留的点（世界系）
    icp::PointCloud2 removed[5];  // [1..4] = 按类别的被删点（世界系）
    icp::PointCloud2 dyn;      // 本帧判动态的点（世界系）
    long n_removed{};
};

// 把一帧的数据填进快照（在主循环里调用）
FrameViz snapshot_frame(long fi, const PolarScan& raw_scan,
                        const PolarScan& cleaned, const DeskewResult& d,
                        const std::vector<std::uint8_t>& why,
                        const icp::Pose2& pose,
                        const std::vector<Cluster>& world_clusters,
                        const std::vector<DynamicVerdict>& verdicts)
{
    FrameViz v;
    v.used = true;
    v.idx = fi;
    v.pose = pose;
    // 原始端点（含后来被删的——用未清洗的 range）
    for (int i = 0; i < raw_scan.size(); ++i) {
        if (!raw_scan.valid(i)) continue;
        const double a = raw_scan.angle(i);
        v.raw.push_back({raw_scan.range[static_cast<std::size_t>(i)] * std::cos(a),
                         raw_scan.range[static_cast<std::size_t>(i)] * std::sin(a)});
    }
    // 保留 / 被删：deskew 后的点按束序对齐（deskew 跳 NaN）
    std::size_t k = 0;
    long removed = 0;
    for (int i = 0; i < cleaned.size(); ++i) {
        if (!cleaned.valid(i)) {
            // 被删的束：用"当作没动"的原始端点画（保留在原位，直观）
            if (why[static_cast<std::size_t>(i)] > 0 &&
                raw_scan.valid(i)) {
                const double a = cleaned.angle(i);
                icp::Point2 p{
                    raw_scan.range[static_cast<std::size_t>(i)] * std::cos(a),
                    raw_scan.range[static_cast<std::size_t>(i)] * std::sin(a)};
                v.removed[why[static_cast<std::size_t>(i)]].push_back(
                    transform(pose, p));
                ++removed;
            }
            continue;
        }
        v.kept.push_back(transform(pose, d.points[k]));
        ++k;
    }
    for (std::size_t c = 0; c < world_clusters.size(); ++c) {
        if (verdicts[c].is_dynamic) {
            for (const auto& p : world_clusters[c].points) v.dyn.push_back(p);
        }
    }
    v.n_removed = removed;
    return v;
}

// 清洗对比图：灰底原始 + 蓝保留 + 分类色被删点
void write_clean_svg(const std::string& path, const FrameViz& v)
{
    viz::write_svg(path,
                   {{&v.raw, "#c8c8c8", 1.3},
                    {&v.kept, "#1a56db", 1.8},
                    {&v.removed[1], "#6e7781", 3.2},   // 门控
                    {&v.removed[2], "#f0883e", 3.2},   // 椒盐
                    {&v.removed[3], "#e02424", 3.2},   // 鬼点
                    {&v.removed[4], "#a256e8", 3.2},   // 阴影
                    {&v.dyn, "#e02424", 5.0}});        // 动态（大圈盖红）
}

// 鬼点特写：围绕鬼点（无鬼点则任一被删点）最密集处放大
void write_ghost_zoom(const std::string& path, const FrameViz& v)
{
    const icp::PointCloud2* focus = nullptr;
    for (const auto& r : v.removed) {
        if (!r.empty()) { focus = &r; if (&r == &v.removed[3] && r.size() >= 3) break; }
    }
    if (!focus || focus->empty()) {
        // 没有被删点：写个空 SVG 占位
        viz::write_svg(path, {{&v.kept, "#1a56db", 1.8}});
        return;
    }
    double cx = 0, cy = 0, spread = 0;
    for (const auto& p : *focus) { cx += p.x; cy += p.y; }
    cx /= focus->size();
    cy /= focus->size();
    for (const auto& p : *focus) {
        spread = std::max(spread, std::hypot(p.x - cx, p.y - cy));
    }
    const double window[3] = {cx, cy, std::max(2.0, spread + 1.5)};
    viz::write_svg(path,
                   {{&v.raw, "#c8c8c8", 1.3},
                    {&v.kept, "#1a56db", 1.8},
                    {&v.removed[1], "#6e7781", 4.5},
                    {&v.removed[2], "#f0883e", 4.5},
                    {&v.removed[3], "#e02424", 4.5},
                    {&v.removed[4], "#a256e8", 4.5}},
                   window);
}

}  // namespace

int main(int argc, char** argv)
{
    Args args = parse_args(argc, argv);
    if (args.show_help || args.input.empty()) {
        std::printf("用法: laser_prep_lab --input <carmen.log> [--out dir] "
                    "[--scan-duration 0.1] [--max-scans N] [--plot-scan N]\n");
        return args.input.empty() ? 1 : 0;
    }

    // ---------- 读日志 ----------
    auto data = carmen::read(args.input);
    if (!data) {
        std::printf("读取失败: %s\n", data.error().c_str());
        return 1;
    }
    const auto& scans = data->scans;
    std::printf("日志: scans=%zu odom=%zu skipped=%ld\n",
                scans.size(), data->odom.size(), data->skipped_lines);

    // ---------- 扫描时长（deskew 的束时序来源）----------
    // CARMEN 日志只有扫描级时间戳，没有扫描时长。按相邻扫描间隔的
    // 1/4 估计并夹在 [0.02, 0.4]：这是本包最大的诚实假设——
    // 真实束时序要么来自雷达协议（时间戳逐束），要么实测标定。
    double duration = args.scan_duration;
    std::filesystem::create_directories(args.out);
    if (duration <= 0 && scans.size() > 10) {
        std::vector<double> dts;
        for (std::size_t i = 1; i + 1 < scans.size(); ++i) {
            dts.push_back(scans[i + 1].stamp - scans[i].stamp);
        }
        std::sort(dts.begin(), dts.end());
        const double med = dts[dts.size() / 2];
        duration = std::clamp(0.25 * med, 0.02, 0.4);
    }
    std::printf("扫描时长假设: %.3fs（束间隔 = 时长/束数）\n", duration);

    // ---------- 全链状态 ----------
    FilterStats total;
    AccumulatorParams ap;
    ScanAccumulator acc(ap);
    FreeSpaceGrid grid(0.15, 25.0);
    const DynamicParams dp;

    icp::Pose2 last_pose{0, 0, 0};
    icp::Pose2 last_odom{0, 0, 0};
    bool has_last = false;

    // 可视化素材
    icp::PointCloud2 static_world, dynamic_world, traj;
    long dynamic_clusters_total = 0;
    long icp_failures = 0;

    // 自动选样：删除最多的一帧 + 跑得最快的一帧
    FrameViz viz_best;      // 删除最多
    FrameViz viz_requested; // --plot-scan 指定
    bool has_requested = false;
    struct {
        bool used{false};
        long idx{};
        double skew{0.0};
        double dtheta{0.0};     // 扫描期间的旋转量（rad）——远墙上放大成大错位
        icp::PointCloud2 naive;   // 不校正（当作机器人在扫描起点不动）
        icp::PointCloud2 fixed;   // 逐束校正后
    } deskew_viz;

    const long n_run = args.max_scans > 0
                           ? std::min<long>(args.max_scans,
                                            static_cast<long>(scans.size()))
                           : static_cast<long>(scans.size());

    for (long fi = 0; fi < n_run; ++fi) {
        PolarScan s = scans[static_cast<std::size_t>(fi)];
        const std::size_t n_beams = s.range.size();
        if (n_beams < 2) continue;
        s.beam_dt = duration / static_cast<double>(n_beams - 1);

        // ---------- ①-④ 清洗（why 记录逐束删除原因）----------
        std::vector<std::uint8_t> why;
        FilterStats st;
        clean_frame(s, st, &why);
        total.total_beams += st.total_beams;
        total.gated += st.gated;
        total.despiked += st.despiked;
        total.demixed += st.demixed;
        total.deshadowed += st.deshadowed;

        // ---------- deskew ----------
        const DeskewResult d = deskew(s, data->odom);

        // 束下标（清洗后仍有效的）
        std::vector<int> beams;
        beams.reserve(d.points.size());
        for (int i = 0; i < s.size(); ++i) {
            if (s.valid(i)) beams.push_back(i);
        }

        // ---------- ICP 位姿链：对融合局部地图配准 ----------
        const icp::Pose2 odom_now = data->scan_odom[static_cast<std::size_t>(fi)];
        icp::Pose2 pose = odom_now;
        if (has_last) {
            const icp::Pose2 guess = compose(
                last_pose, delta(last_odom, odom_now));
            const icp::PointCloud2 target = acc.cloud();
            if (target.size() > 100) {
                auto res = icp::align(d.points, target, guess);
                if (res) {
                    pose = res->pose;
                } else {
                    pose = guess;
                    ++icp_failures;
                }
            } else {
                pose = guess;
            }
        }
        has_last = true;
        last_pose = pose;
        last_odom = odom_now;
        traj.push_back({pose.x, pose.y});

        // ---------- 聚类（世界系）----------
        auto clusters = cluster_by_break(d.points, beams, s.angle_inc,
                                         ClusterParams{});
        for (auto& c : clusters) {
            for (auto& p : c.points) p = transform(pose, p);
        }

        // ---------- 动态判别（先判后记）----------
        const auto verdicts = judge_clusters(clusters, grid, dp);
        for (std::size_t k = 0; k < clusters.size(); ++k) {
            if (verdicts[k].is_dynamic) ++dynamic_clusters_total;
        }

        // ---------- 选样快照（只在候选帧上做，成本可忽略）----------
        const bool want_requested = (fi == args.plot_scan);
        const long n_removed = st.removed();
        if (want_requested) {
            viz_requested = snapshot_frame(fi, scans[static_cast<std::size_t>(fi)],
                                           s, d, why, pose, clusters, verdicts);
            has_requested = true;
        }
        if (!viz_best.used || n_removed > viz_best.n_removed) {
            viz_best = snapshot_frame(fi, scans[static_cast<std::size_t>(fi)],
                                      s, d, why, pose, clusters, verdicts);
        }
        // deskew 选样：只要"物理上可信"的帧。不设防的话会选中
        // 里程计跳变帧（0.022s 里"挪"0.74m = 33m/s——那是轮子打滑
        // 或重定位跳变，不是运动畸变，真实踩坑）。
        if (d.corrected && d.raw_skew_m > deskew_viz.skew &&
            d.raw_skew_m <= 1.5 * duration) {
            deskew_viz.used = true;
            deskew_viz.idx = fi;
            deskew_viz.skew = d.raw_skew_m;
            if (const auto p0 = data->odom.at(s.stamp);
                p0 && s.size() > 1) {
                const auto pl = data->odom.at(s.time(s.size() - 1));
                if (pl) {
                    double dth = pl->theta - p0->theta;
                    deskew_viz.dtheta =
                        std::atan2(std::sin(dth), std::cos(dth));
                }
            }
            deskew_viz.naive = s.endpoints();
            deskew_viz.fixed = d.points;
        }

        // ---------- 记透视 + 融合 ----------
        grid.decay(dp.decay);
        record_frame(pose, d.points, grid);
        acc.push(d.points, pose);

        // 全程采样（静态点）——每 40 帧采一次 + 每帧只取一半点，
        // 否则 SVG 有几十 MB，浏览器打不开（真实踩坑）。
        if (fi % 40 == 0) {
            std::size_t k = 0;
            for (const auto& p : d.points) {
                if ((k++ & 1) == 0) static_world.push_back(transform(pose, p));
            }
        }
        // 动态点采样收集（每 10 帧一次，量太大画不动）
        if (fi % 10 == 0) {
            for (std::size_t k = 0; k < clusters.size(); ++k) {
                if (verdicts[k].is_dynamic) {
                    for (const auto& p : clusters[k].points) {
                        dynamic_world.push_back(p);
                    }
                }
            }
        }

        if ((fi + 1) % 1000 == 0) {
            std::printf("  进度 %ld/%ld  删除束 %ld/%ld  动态簇 %ld\n",
                        fi + 1, n_run, total.removed(), total.total_beams,
                        dynamic_clusters_total);
            std::fflush(stdout);
        }
    }

    // ---------- 报告 ----------
    {
        std::ofstream f(args.out + "/report.txt");
        f << std::format("总束数: {}\n", total.total_beams);
        f << std::format("① 量程/无效: {} ({:.2f}%)\n", total.gated,
                        100.0 * total.gated / total.total_beams);
        f << std::format("② 椒盐孤点: {} ({:.3f}%)\n", total.despiked,
                        100.0 * total.despiked / total.total_beams);
        f << std::format("③ 混合像素: {} ({:.3f}%)\n", total.demixed,
                        100.0 * total.demixed / total.total_beams);
        f << std::format("④ 掠射阴影: {} ({:.3f}%)\n", total.deshadowed,
                        100.0 * total.deshadowed / total.total_beams);
        f << std::format("ICP 链失败回退: {} 次\n", icp_failures);
        f << std::format("动态簇判定: {}\n", dynamic_clusters_total);
        f << std::format("扫描时长假设: {:.3}s\n", duration);
    }
    std::printf("完成: 总束 %ld, 删除 %ld（门控 %ld 椒盐 %ld 鬼点 %ld 阴影 %ld）\n",
                total.total_beams, total.removed(), total.gated,
                total.despiked, total.demixed, total.deshadowed);
    std::printf("ICP 回退 %ld 次, 动态簇 %ld\n", icp_failures,
                dynamic_clusters_total);

    // ---------- 可视化 ----------
    const std::string base = args.out + "/";
    if (viz_best.used) {
        write_clean_svg(base + "frame_best.svg", viz_best);
        write_ghost_zoom(base + "ghost_zoom.svg", viz_best);
    }
    if (has_requested) {
        write_clean_svg(base + "frame_" + std::to_string(args.plot_scan) +
                            ".svg",
                        viz_requested);
    }
    if (deskew_viz.used) {
        // 找 naive/deskew 错位最大的一对点，围绕它放大——
        // 真实数据的畸变量级只有几厘米，全景图里根本看不见。
        double best_d = -1;
        double wx = 0, wy = 0;
        const std::size_t n_pair = std::min(deskew_viz.naive.size(),
                                            deskew_viz.fixed.size());
        for (std::size_t k = 0; k < n_pair; ++k) {
            const double dx = deskew_viz.naive[k].x - deskew_viz.fixed[k].x;
            const double dy = deskew_viz.naive[k].y - deskew_viz.fixed[k].y;
            if (std::hypot(dx, dy) > best_d) {
                best_d = std::hypot(dx, dy);
                wx = deskew_viz.fixed[k].x;
                wy = deskew_viz.fixed[k].y;
            }
        }
        // 微窗对照：真实数据的畸变只有厘米级，且最大错位点常在远端
        // 掠射墙上（邻束间距 1m+）——扩窗只会把比例尺压到看不见。
        // 干脆只框住错位对 ±max(0.2, 5×错位)：大点、高倍放大，
        // "同一束激光，校正前后落在差几厘米的两个位置"一眼可见。
        const double window[3] = {wx, wy, std::max(0.2, 5.0 * best_d)};
        viz::write_svg(base + "deskew_best.svg",
                       {{&deskew_viz.naive, "#c8c8c8", 5.0},
                        {&deskew_viz.fixed, "#1a56db", 5.0}},
                       window);
    }

    // 合成 deskew 演示：真实数据的扫描时长是估计值（几厘米级畸变），
    // 看不清原理——合成场景把畸变放大到肉眼可见的量级。
    {
        // 走廊 y=±2，机器人 1m/s 前进，扫描历时 0.2s（畸变 0.2m）
        icp::PointCloud2 naive, fixed;
        const double v = 1.0, T = 0.2;
        const int beams = 121;
        const double a0 = -75.0 * kPi / 180.0;
        const double da = 150.0 * kPi / 180.0 / (beams - 1);
        for (int i = 0; i < beams; ++i) {
            const double t = T * static_cast<double>(i) / (beams - 1);
            const double a = a0 + da * i;
            // 该束时刻机器人位置 (v·t, 0)，打到的真实墙点（世界系）
            const double s = std::sin(a);
            if (std::abs(s) < 0.2) continue;  // 平行墙的束不要
            const double r = 2.0 / std::abs(s);
            const icp::Point2 world{v * t + r * std::cos(a), r * s};
            // 雷达自己的报告（该束时刻的传感器系）
            const icp::Point2 sensor_report{world.x - v * t, world.y};
            // deskew：拉回扫描起始帧（机器人在原点时刻）
            fixed.push_back(world);
            // 不校正：整帧当成机器人在原点拍的
            naive.push_back(sensor_report);
        }
        viz::write_svg(base + "deskew_demo.svg",
                       {{&naive, "#c8c8c8", 2.2},
                        {&fixed, "#1a56db", 2.2}});
    }
    viz::write_svg(base + "world_view.svg",
                   {{&static_world, "#1a56db", 1.0},
                    {&dynamic_world, "#e02424", 2.0},
                    {&traj, "#2da44e", 1.0}});

    // ---------- index.html：一个文件看完全部 ----------
    {
        std::ofstream f(base + "index.html");
        f << "<!doctype html>\n<meta charset=\"utf-8\">\n";
        f << "<title>laser_prep_lab 可视化</title>\n";
        f << "<body style=\"max-width:1700px;margin:2em auto;"
             "font-family:sans-serif;background:#fafbfc\">\n";
        f << std::format("<h1>laser_prep_lab — 清洗链可视化</h1>"
                         "<p>日志: {}　总束 {}　删除 {} ({:.2f}%)　"
                         "ICP 回退 {} 次　动态簇 {}</p>\n",
                         args.input, total.total_beams, total.removed(),
                         100.0 * total.removed() / total.total_beams,
                         icp_failures, dynamic_clusters_total);

        f << "<h2>① 清洗对比 — 删除最多的一帧"
          << (viz_best.used ? std::format("（第 {} 帧，删 {} 束）",
                                          viz_best.idx, viz_best.n_removed)
                            : std::string("（无）"))
          << "</h2>\n";
        f << "<p>浅灰=原始束；<b style='color:#1a56db'>蓝</b>=保留；"
             "<b style='color:#e02424'>红</b>=鬼点(混合像素)；"
             "<b style='color:#f0883e'>橙</b>=椒盐；"
             "<b style='color:#a256e8'>紫</b>=掠射阴影；"
             "深灰=门控。</p>\n";
        if (viz_best.used) {
            f << "<img src=\"frame_best.svg\" style=\"width:100%;"
                 "background:white;border:1px solid #ddd\"><br>\n";
        }

        f << "<h2>② 鬼点特写（自动放大到鬼点最密集处）</h2>\n";
        f << "<img src=\"ghost_zoom.svg\" style=\"width:70%;"
             "background:white;border:1px solid #ddd\"><br>\n";

        f << "<h2>③ 运动畸变校正 — 真实数据里畸变最大的一帧"
          << (deskew_viz.used
                  ? std::format("（第 {} 帧，扫描期间平移 {:.3f}m、旋转 "
                                "{:.3f}rad——旋转在 10m 外的墙上放大成 "
                                "0.4m 级的点错位；高倍微窗对照）",
                                deskew_viz.idx, deskew_viz.skew,
                                deskew_viz.dtheta)
                  : std::string("（无）"))
          << "</h2>\n";
        f << "<p>浅灰=不校正；<b style='color:#1a56db'>蓝</b>=逐束按里程计"
             "校正。两色的错位就是运动畸变本身。</p>\n";
        if (deskew_viz.used) {
            f << "<img src=\"deskew_best.svg\" style=\"width:70%;"
                 "background:white;border:1px solid #ddd\"><br>\n";
        }

        f << "<h2>③b 运动畸变 — 合成演示（畸变放大到肉眼量级）</h2>\n";
        f << "<p>1m/s 的机器人、0.2s 扫描：浅灰=不校正（两墙被拖成"
             "平行四边形），<b style='color:#1a56db'>蓝</b>=校正后（墙"
             "是直的）。原理与上图完全相同，只是量级放大。</p>\n";
        f << "<img src=\"deskew_demo.svg\" style=\"width:70%;"
             "background:white;border:1px solid #ddd\"><br>\n";

        if (has_requested) {
            f << std::format("<h2>④ 指定帧（第 {} 帧）</h2>\n",
                             args.plot_scan);
            f << "<img src=\"frame_" << args.plot_scan
              << ".svg\" style=\"width:100%;"
                 "background:white;border:1px solid #ddd\"><br>\n";
        }

        f << "<h2>⑤ 全程总览</h2>\n";
        f << "<p><b style='color:#1a56db'>蓝</b>=静态点（每 20 帧采样）；"
             "<b style='color:#e02424'>红</b>=动态簇候选；"
             "<b style='color:#2da44e'>绿</b>=轨迹。红点密集处即位姿链"
             "漂移或掠射墙的系统性格点（诚实记录，见 README）。</p>\n";
        f << "<img src=\"world_view.svg\" style=\"width:100%;"
             "background:white;border:1px solid #ddd\">\n";
        f << "</body>\n";
    }
    std::printf("已写出: %sindex.html 及全部 SVG（浏览器打开即看）\n",
                base.c_str());
    return 0;
}
