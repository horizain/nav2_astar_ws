// ============================================================================
// scan_localizer 基准测试：算法组件 × 场景 的系统对比
//
// E1 跟踪精度（小扰动）      E2 捕获范围（大初始误差）
// E3 传感器噪声鲁棒性        E4 动态障碍鲁棒性
// E5 长走廊退化              E6 扫描密度
// E7 CPU 时间               E8 全局重定位（correlative vs PF）
// E9 轨迹级架构对比（扫描匹配 vs 粒子滤波，含人群遮挡段）
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "scan_localizer/localizer_core.hpp"
#include "scan_localizer/matchers.hpp"
#include "scan_localizer/particle_filter.hpp"
#include "scan_localizer/sim_world.hpp"

using namespace sloc;
using Clock = std::chrono::steady_clock;

namespace {

double wrap(double a) {
    while (a > M_PI) { a -= 2 * M_PI; }
    while (a < -M_PI) { a += 2 * M_PI; }
    return a;
}

struct Stat {
    double median{0}, p90{0}, mean{0};
};
Stat stats(std::vector<double> v) {
    Stat s;
    if (v.empty()) { return s; }
    std::sort(v.begin(), v.end());
    s.median = v[v.size() / 2];
    s.p90 = v[static_cast<std::size_t>(v.size() * 0.9)];
    s.mean = 0;
    for (double x : v) { s.mean += x; }
    s.mean /= v.size();
    return s;
}

double ms_of(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0)
        .count();
}

struct Row {
    std::string name;
    double pos_med{0}, pos_p90{0}, ang_med{0}, t_med{0}, succ{0};
};

void print_table(const std::string& title,
                 const std::vector<Row>& rows) {
    std::cout << "\n== " << title << " ==\n";
    std::cout << std::left << std::setw(22) << "matcher"
              << std::right << std::setw(12) << "pos_med"
              << std::setw(12) << "pos_p90"
              << std::setw(12) << "ang_med(deg)" << std::setw(12) << "ms_med"
              << std::setw(10) << "succ%" << "\n";
    for (const auto& r : rows) {
        std::cout << std::left << std::setw(22) << r.name << std::right
                  << std::setw(12) << std::fixed << std::setprecision(3)
                  << r.pos_med << std::setw(12) << r.pos_p90 << std::setw(12)
                  << r.ang_med * 180.0 / M_PI << std::setw(12) << r.t_med
                  << std::setw(10) << std::setprecision(1) << r.succ * 100.0
                  << "\n";
    }
}

// 试验一组 (gt, scan, prior)，返回每匹配器的行
Row run_trials(const std::string& name, const MapField& field,
               const Matcher& m, const MatchOptions& opt,
               const std::vector<std::pair<Pose2, Scan>>& data,
               const std::vector<Pose2>& priors, double succ_pos = 0.05,
               double succ_ang = 1.0 * M_PI / 180.0) {
    Row r;
    r.name = name;
    std::vector<double> pe, ae, tt;
    int ok = 0;
    for (std::size_t i = 0; i < data.size(); ++i) {
        const auto t0 = Clock::now();
        const MatchResult res = m.match(field, priors[i], data[i].second, opt);
        const double ms = ms_of(t0);
        const double e = std::hypot(res.pose.x - data[i].first.x,
                                    res.pose.y - data[i].first.y);
        const double a = std::abs(wrap(res.pose.theta - data[i].first.theta));
        pe.push_back(e);
        ae.push_back(a);
        tt.push_back(ms);
        if (e < succ_pos && a < succ_ang) { ++ok; }
    }
    const auto s1 = stats(pe), s2 = stats(ae), s3 = stats(tt);
    r.pos_med = s1.median;
    r.pos_p90 = s1.p90;
    r.ang_med = s2.median;
    r.t_med = s3.median;
    r.succ = data.empty() ? 0.0 : double(ok) / data.size();
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    const int quick = argc > 1 && std::string(argv[1]) == "--quick" ? 5 : 1;
    std::cout << "scan_localizer benchmark (quick x" << quick << ")\n";

    std::mt19937 rng(42);

    // ================= 地图与场 =================
    const auto t_build = Clock::now();
    const OccupancyGrid grid = sim::make_warehouse(0.05);
    auto field = std::make_shared<MapField>();
    field->build(grid, 0.20);
    std::cout << "warehouse map: " << grid.width << "x" << grid.height
              << " cells, MapField build: " << ms_of(t_build) / 1000.0
              << " s\n";

    // 匹配器
    auto corr = make_matcher("correlative");
    auto fgn = make_matcher("field_gn");
    auto icp = make_matcher("icp_p2p");
    auto ndt = make_matcher("ndt", &grid, 0.5);
    auto comp = make_matcher("composed");
    // PF 专用观测场：σ 更尖（0.1），否则粒子云无法回拉
    auto field10 = std::make_shared<MapField>();
    field10->build(grid, 0.10);
    MatchOptions opt;  // 跟踪默认窗口
    MatchOptions opt_wide = opt;
    opt_wide.window_lin = 1.6;
    opt_wide.window_ang = 0.55;
    opt_wide.step_lin = 0.08;
    opt_wide.step_ang = 0.03;
    MatchOptions opt_tight = opt;  // 动态障碍场景：紧截断
    opt_tight.trim_dist = 0.20;
    opt_tight.huber_delta = 0.05;

    // ================= 通用数据 =================
    const int N1 = 100 / quick;
    sim::ScanSimConfig sim_cfg;
    auto poses = sim::random_free_poses(grid, N1, rng);

    // ================= E1 跟踪精度 =================
    {
        std::vector<std::pair<Pose2, Scan>> data;
        std::vector<Pose2> priors;
        std::normal_distribution<double> n2(0, 0.02), a2(0, 0.5 * M_PI / 180);
        for (const auto& p : poses) {
            data.emplace_back(p, sim::make_scan(grid, p, sim_cfg, rng));
            priors.push_back({p.x + n2(rng), p.y + n2(rng),
                              p.theta + a2(rng)});
        }
        print_table("E1 tracking accuracy (prior err ~2cm/0.5deg, clean scan)",
                    {run_trials("correlative", *field, *corr, opt, data, priors),
                     run_trials("field_gn", *field, *fgn, opt, data, priors),
                     run_trials("icp_p2p", *field, *icp, opt, data, priors),
                     run_trials("ndt", *field, *ndt, opt, data, priors),
                     run_trials("composed", *field, *comp, opt, data, priors)});
    }

    // ================= E2 捕获范围 =================
    {
        struct Lvl { double pos, ang; };
        const Lvl lvls[] = {{0.05, 2}, {0.15, 5}, {0.40, 10},
                            {0.80, 18}, {1.50, 30}};
        const int N2 = 50 / quick;
        std::cout << "\n== E2 capture range (success% by initial error) ==\n";
        std::cout << std::left << std::setw(22) << "matcher";
        for (const auto& l : lvls) {
            std::cout << std::setw(10) << std::to_string(l.pos) + "m";
        }
        std::cout << std::setw(12) << "ms@large" << "\n";
        struct Cand { std::string n; Matcher* m; const MatchOptions* o; };
        std::vector<Cand> cands = {
            {"correlative", corr.get(), &opt},
            {"field_gn", fgn.get(), &opt},
            {"icp_p2p", icp.get(), &opt},
            {"ndt", ndt.get(), &opt},
            {"composed", comp.get(), &opt},
            {"composed_wide", comp.get(), &opt_wide}};
        for (const auto& c : cands) {
            std::cout << std::left << std::setw(22) << c.n;
            std::vector<double> ms_at_large;
            for (const auto& l : lvls) {
                int ok = 0;
                for (int i = 0; i < N2; ++i) {
                    const Pose2 gt = poses[static_cast<std::size_t>(i) %
                                           poses.size()];
                    const auto scan = sim::make_scan(grid, gt, sim_cfg, rng);
                    const double dir = rng() * 2 * M_PI;
                    const Pose2 prior(gt.x + l.pos * std::cos(dir),
                                      gt.y + l.pos * std::sin(dir),
                                      gt.theta + l.ang * M_PI / 180.0);
                    const auto t1 = Clock::now();
                    const auto res = c.m->match(*field, prior, scan, *c.o);
                    if (l.pos >= 0.8) { ms_at_large.push_back(ms_of(t1)); }
                    const double e = std::hypot(res.pose.x - gt.x,
                                                res.pose.y - gt.y);
                    const double a =
                        std::abs(wrap(res.pose.theta - gt.theta));
                    if (e < 0.05 && a < 1.0 * M_PI / 180.0) { ++ok; }
                }
                std::cout << std::setw(10) << std::fixed
                          << std::setprecision(1) << 100.0 * ok / N2;
            }
            std::cout << std::setw(12) << std::setprecision(1)
                      << stats(ms_at_large).median << "\n";
        }
    }

    // ================= E3 噪声鲁棒性 =================
    {
        std::cout << "\n== E3 range noise (median pos err, prior 2cm) ==\n";
        std::cout << std::left << std::setw(22) << "matcher";
        const double sigmas[] = {0.0, 0.01, 0.03, 0.05};
        for (double s : sigmas) {
            std::cout << std::setw(12) << ("sigma=" + std::to_string(s));
        }
        std::cout << "\n";
        sim::ScanSimConfig cfg = sim_cfg;
        std::normal_distribution<double> n2(0, 0.02), a2(0, 0.5 * M_PI / 180);
        struct Cand { std::string n; Matcher* m; };
        for (const auto& c : std::vector<Cand>{
                 {"correlative", corr.get()}, {"field_gn", fgn.get()},
                 {"icp_p2p", icp.get()}, {"ndt", ndt.get()},
                 {"composed", comp.get()}}) {
            std::cout << std::left << std::setw(22) << c.n;
            for (double s : sigmas) {
                cfg.noise_std = s;
                std::vector<double> pe;
                for (const auto& p : poses) {
                    const auto scan = sim::make_scan(grid, p, cfg, rng);
                    const Pose2 prior(p.x + n2(rng), p.y + n2(rng),
                                      p.theta + a2(rng));
                    const auto res = c.m->match(*field, prior, scan, opt);
                    pe.push_back(std::hypot(res.pose.x - p.x, res.pose.y - p.y));
                }
                std::cout << std::setw(12) << std::fixed
                          << std::setprecision(3) << stats(pe).median;
            }
            std::cout << "\n";
        }
    }

    // ================= E4 动态障碍 =================
    {
        std::cout << "\n== E4 dynamic obstacles (2 boxes 0.6m near robot) ==\n";
        std::vector<std::pair<Pose2, Scan>> data;
        std::vector<Pose2> priors;
        std::normal_distribution<double> n2(0, 0.02), a2(0, 0.5 * M_PI / 180);
        for (const auto& p : poses) {
            std::vector<sim::DynBox> dyn;
            for (int b = 0; b < 2; ++b) {
                const double dir = rng() * 2 * M_PI;
                const double d = 2.0 + rng() * 2.0;
                const double bx = p.x + d * std::cos(dir);
                const double by = p.y + d * std::sin(dir);
                if (bx > 1 && bx < 39 && by > 1 && by < 29) {
                    dyn.push_back({bx, by, 0.3, 0.3});
                }
            }
            data.emplace_back(p, sim::make_scan(grid, p, sim_cfg, rng, dyn));
            priors.push_back({p.x + n2(rng), p.y + n2(rng),
                              p.theta + a2(rng)});
        }
        print_table("E4 dynamic obstacles",
                    {run_trials("correlative", *field, *corr, opt, data, priors),
                     run_trials("field_gn", *field, *fgn, opt, data, priors),
                     run_trials("field_gn_tight", *field, *fgn, opt_tight,
                                data, priors),
                     run_trials("ndt", *field, *ndt, opt, data, priors),
                     run_trials("composed", *field, *comp, opt, data, priors),
                     run_trials("composed_tight", *field, *comp, opt_tight,
                                data, priors)});
    }

    // ================= E5 走廊退化 =================
    {
        const OccupancyGrid cgrid = sim::make_corridor(0.05);
        auto cfield = std::make_shared<MapField>();
        cfield->build(cgrid, 0.20);
        auto cndt = make_matcher("ndt", &cgrid, 0.5);
        const int N5 = 50 / quick;
        std::uniform_real_distribution<double> ux(2, 28), uy(1.2, 1.8),
            ut(-0.3, 0.3);
        std::vector<Pose2> cpos;
        for (int i = 0; i < N5; ++i) {
            cpos.push_back({ux(rng), uy(rng), ut(rng)});
        }
        std::vector<std::pair<Pose2, Scan>> data;
        std::vector<Pose2> priors;
        std::normal_distribution<double> n2(0, 0.02), a2(0, 0.5 * M_PI / 180);
        for (const auto& p : cpos) {
            data.emplace_back(p, sim::make_scan(cgrid, p, sim_cfg, rng));
            priors.push_back({p.x + n2(rng), p.y + n2(rng),
                              p.theta + a2(rng)});
        }
        std::cout << "\n== E5 corridor degeneracy (30x3m) ==\n";
        std::cout << std::left << std::setw(22) << "matcher" << std::right
                  << std::setw(14) << "err_along(m)" << std::setw(14)
                  << "err_across(m)" << std::setw(14) << "cov_along_med"
                  << "\n";
        struct Cand { std::string n; Matcher* m; };
        for (const auto& c : std::vector<Cand>{
                 {"correlative", corr.get()}, {"field_gn", fgn.get()},
                 {"icp_p2p", icp.get()}, {"ndt", cndt.get()},
                 {"composed", comp.get()}}) {
            std::vector<double> ea, ec, cv;
            for (std::size_t i = 0; i < data.size(); ++i) {
                const auto res = c.m->match(*cfield, priors[i], data[i].second,
                                            opt);
                ea.push_back(std::abs(res.pose.x - data[i].first.x));
                ec.push_back(std::abs(res.pose.y - data[i].first.y));
                cv.push_back(res.covariance(0, 0));  // 沿轴方差（退化探测器）
            }
            std::cout << std::left << std::setw(22) << c.n << std::right
                      << std::setw(14) << std::fixed << std::setprecision(3)
                      << stats(ea).median << std::setw(14)
                      << stats(ec).median << std::setw(14)
                      << stats(cv).median << "\n";
        }
    }

    // ================= E6 扫描密度 =================
    {
        std::cout << "\n== E6 scan density (720/360/180/90 beams) ==\n";
        std::cout << std::left << std::setw(22) << "matcher" << std::right
                  << std::setw(10) << "beams" << std::setw(14)
                  << "pos_med" << std::setw(12) << "ms_med" << "\n";
        struct Cand { std::string n; Matcher* m; };
        const int beams[] = {720, 360, 180, 90};
        for (const auto& c : std::vector<Cand>{{"correlative", corr.get()},
                                               {"field_gn", fgn.get()},
                                               {"composed", comp.get()}}) {
            for (int b : beams) {
                sim::ScanSimConfig cfg = sim_cfg;
                cfg.subsample = 720 / b;
                std::vector<double> pe, tt;
                for (const auto& p : poses) {
                    const auto scan = sim::make_scan(grid, p, cfg, rng);
                    const Pose2 prior(p.x, p.y, p.theta);
                    const auto t1 = Clock::now();
                    const auto res = c.m->match(*field, prior, scan, opt);
                    tt.push_back(ms_of(t1));
                    pe.push_back(std::hypot(res.pose.x - p.x, res.pose.y - p.y));
                }
                std::cout << std::left << std::setw(22) << c.n << std::right
                          << std::setw(10) << b << std::setw(14)
                          << std::fixed << std::setprecision(4)
                          << stats(pe).median << std::setw(12)
                          << stats(tt).median << "\n";
            }
        }
    }

    // ================= E7 全局定位 =================
    {
        const int N8 = 30 / quick;
        std::cout << "\n== E8 global localization (cold start, " << N8
                  << " trials) ==\n";
        // correlative global
        {
            int ok = 0;
            std::vector<double> tt;
            const int n_trials =
                std::min(N8, static_cast<int>(poses.size()));
            for (int t = 0; t < n_trials; ++t) {
                const auto& p = poses[static_cast<std::size_t>(t)];
                const auto scan = sim::make_scan(grid, p, sim_cfg, rng);
                const auto t1 = Clock::now();
                const auto g = global_correlative(*field, scan, 3,
                                                  2.0 * M_PI / 180.0, 5);
                tt.push_back(ms_of(t1));
                if (g.valid) {
                    const double e = std::hypot(g.pose.x - p.x, g.pose.y - p.y);
                    const double a = std::abs(wrap(g.pose.theta - p.theta));
                    if (e < 0.3 && a < 3.0 * M_PI / 180.0) { ++ok; }
                }
            }
            std::cout << "global_correlative : succ " << 100.0 * ok / n_trials
                      << "%  median " << stats(tt).median << " ms (n="
                      << n_trials << ")\n";
        }
        // PF global（需运动收敛：10 步 × 0.25m）
        {
            PfOptions po;
            po.n_particles = 20000;
            po.alpha_trans_trans = 0.02;
            po.alpha_rot_rot = 0.02;
            po.alpha_rot_trans = 0.02;
            po.alpha_trans_rot = 0.02;
            std::uniform_real_distribution<double> wob(-0.05, 0.05);
            int ok = 0;
            std::vector<double> tt;
            std::vector<double> final_err;  // 诊断
            const int n_trials =
                std::min(N8, static_cast<int>(poses.size()));
            for (int t = 0; t < n_trials; ++t) {
                const auto& p0 = poses[static_cast<std::size_t>(t)];
                ParticleFilter pf(po);
                const auto t1 = Clock::now();
                pf.init_global(grid, rng);
                Pose2 gt = p0, prev_gt = p0;
                for (int k = 0; k < 20; ++k) {
                    // 真实运动 0.25m + 航向扰动
                    gt.x += 0.25 * std::cos(gt.theta);
                    gt.y += 0.25 * std::sin(gt.theta);
                    gt.theta += wob(rng);
                    // 本体系真实增量
                    const double c = std::cos(prev_gt.theta);
                    const double s = std::sin(prev_gt.theta);
                    const double dx = gt.x - prev_gt.x, dy = gt.y - prev_gt.y;
                    const Pose2 d_true(c * dx + s * dy, -s * dx + c * dy,
                                       gt.theta - prev_gt.theta);
                    // 滑移 odom：2% 尺度 + 转角偏差
                    const Pose2 d_odom(d_true.x * 1.02, d_true.y * 1.02,
                                       d_true.theta * 1.02 + 0.002);
                    pf.predict(d_odom, rng);
                    const auto scan = sim::make_scan(grid, gt, sim_cfg, rng);
                    pf.update(*field10, scan, rng);
                    prev_gt = gt;
                }
                tt.push_back(ms_of(t1));
                Pose2 mean;
                Cov3 cov;
                pf.estimate(mean, cov);
                const double e = std::hypot(mean.x - gt.x, mean.y - gt.y);
                final_err.push_back(e);
                const double a = std::abs(wrap(mean.theta - gt.theta));
                if (e < 0.3 && a < 3.0 * M_PI / 180.0) { ++ok; }
            }
            std::cout << "pf_global(20000,20步) : succ " << 100.0 * ok / n_trials
                      << "%  median " << stats(tt).median << " ms"
                      << "  err_med " << stats(final_err).median << " m\n";
        }
    }

    // ================= E9 轨迹级架构对比 =================
    {
        std::cout << "\n== E9 trajectory tracking (100m loop, odom slip, "
                     "crowd segment) ==\n";
        // 轨迹：左上大自由区的矩形环路 ×2 圈（避墙/凸台/柱阵/货架）
        // (2,13.5)→(13.5,13.5)→(13.5,28)→(2,28)→(2,13.5)
        struct Node { double x, y; };
        const Node base_wp[] = {{2, 13.5}, {13.5, 13.5},
                                {13.5, 28}, {2, 28}, {2, 13.5}};
        std::vector<Node> wp;
        for (int lap = 0; lap < 2; ++lap) {
            for (const auto& w : base_wp) { wp.push_back(w); }
        }
        wp.push_back({2, 13.75});  // 收尾小段
        // 生成 GT 轨迹（0.25m 步长）
        std::vector<Pose2> traj;
        {
            Pose2 cur(2, 13.5, 0);
            for (std::size_t seg = 0; seg + 1 < wp.size(); ++seg) {
                const double dx = wp[seg + 1].x - wp[seg].x;
                const double dy = wp[seg + 1].y - wp[seg].y;
                const double len = std::hypot(dx, dy);
                const double n = len / 0.25;
                for (int i = 0; i < static_cast<int>(n); ++i) {
                    cur.x += dx / n;
                    cur.y += dy / n;
                    cur.theta = std::atan2(dy, dx);
                    traj.push_back(cur);
                }
            }
        }
        // 滑移 odom 增量
        std::normal_distribution<double> gn01(0, 0.01);
        auto odom_delta = [&](const Pose2& d) {
            const double s = 1.0 + 0.02;  // 2% 尺度误差
            return Pose2(d.x * s + gn01(rng) * 0.01, d.y * s + gn01(rng) * 0.01,
                         d.theta * (1.0 + 0.01) + gn01(rng) * 0.002);
        };

        const bool with_crowd = true;
        // 人群遮挡段：中段 40 帧注入动态箱
        auto crowd_at = [&](int k) -> std::vector<sim::DynBox> {
            if (!with_crowd || k < 150 || k > 190) { return {}; }
            const auto& p = traj[static_cast<std::size_t>(k)];
            return {{p.x + 1.5 * std::cos(p.theta),
                     p.y + 1.5 * std::sin(p.theta), 0.35, 0.35},
                    {p.x + 2.8 * std::cos(p.theta + 0.4),
                     p.y + 2.8 * std::sin(p.theta + 0.4), 0.35, 0.35}};
        };

        // ---- 架构 A：LocalizerCore + composed ----
        {
            CoreConfig cc;
            cc.match_opt.window_lin = 0.25;  // 打滑放宽一点
            LocalizerCore core(cc);
            core.set_map(field);
            core.set_matcher(comp);
            core.on_odom(traj.front());  // 先喂 t0 的 odom，再设初值（基线精确锚定）
            core.set_initial_pose(traj.front());
            Pose2 odom = traj.front();  // odom 系与 map 系同起点
            double err_sum = 0;
            int n_acc = 0, n_total = 0;
            double err_max = 0;
            std::vector<double> errs;
            for (std::size_t k = 1; k < traj.size(); ++k) {
                const Pose2 prev = traj[k - 1];
                const Pose2 gt = traj[k];
                // 本体系真实增量
                const double c = std::cos(prev.theta), s = std::sin(prev.theta);
                const double mx = gt.x - prev.x, my = gt.y - prev.y;
                const Pose2 d_local(c * mx + s * my, -s * mx + c * my,
                                    gt.theta - prev.theta);
                const Pose2 d_odom = odom_delta(d_local);
                odom = odom * d_odom;
                core.on_odom(odom);
                if (k % 4 == 0) {  // 每 1m 一帧扫描
                    const auto scan = sim::make_scan(
                        grid, gt, sim_cfg, rng, crowd_at(static_cast<int>(k)));
                    const auto outp = core.on_scan(scan);
                    if (outp.state == LocState::DEGRADED ||
                        outp.state == LocState::LOST) {
                        // 记录但不干预（产品中此处触发重定位）
                    }
                    const double e = std::hypot(outp.pose.x - gt.x,
                                                outp.pose.y - gt.y);
                    errs.push_back(e);
                    err_sum += e;
                    err_max = std::max(err_max, e);
                    ++n_total;
                    if (outp.accepted) { ++n_acc; }
                }
            }
            std::cout << "A core+composed   : mean_err "
                      << err_sum / n_total << " m, max_err " << err_max
                      << " m, accept_rate " << 100.0 * n_acc / n_total
                      << "%\n";
        }

        // ---- 架构 B：粒子滤波（公平调参：σ=0.1 场 + 帧间累积预测）----
        {
            PfOptions po;
            po.n_particles = 2000;
            po.alpha_trans_trans = 0.02;
            po.alpha_rot_rot = 0.02;
            po.alpha_rot_trans = 0.02;
            po.alpha_trans_rot = 0.02;
            ParticleFilter pf(po);
            pf.init_gaussian(traj.front(), 0.05, 0.02, rng);
            double err_sum = 0;
            int n_total = 0;
            double err_max = 0;
            Pose2 odom = traj.front();
            Pose2 accum_delta{};  // 两次观测之间的本体系增量累积
            for (std::size_t k = 1; k < traj.size(); ++k) {
                const Pose2 prev = traj[k - 1];
                const Pose2 gt = traj[k];
                const double c = std::cos(prev.theta), s = std::sin(prev.theta);
                const double mx = gt.x - prev.x, my = gt.y - prev.y;
                const Pose2 d_local(c * mx + s * my, -s * mx + c * my,
                                    gt.theta - prev.theta);
                const Pose2 d_odom = odom_delta(d_local);
                odom = odom * d_odom;
                // SE(2) 复合累积
                accum_delta = accum_delta * d_odom;
                if (k % 4 == 0) {
                    pf.predict(accum_delta, rng);
                    accum_delta = Pose2{};
                    const auto scan = sim::make_scan(
                        grid, gt, sim_cfg, rng, crowd_at(static_cast<int>(k)));
                    pf.update(*field10, scan, rng);
                    Pose2 mean;
                    Cov3 cov;
                    pf.estimate(mean, cov);
                    const double e = std::hypot(mean.x - gt.x, mean.y - gt.y);
                    err_sum += e;
                    err_max = std::max(err_max, e);
                    ++n_total;
                }
            }
            std::cout << "B particle_filter : mean_err " << err_sum / n_total
                      << " m, max_err " << err_max << " m ("
                      << po.n_particles << " particles)\n";
        }
    }


    // ================= E10 IMU 融合收益 =================
    {
        // 环路与 E9 相同；轮式平移恒带 2% 尺度误差，
        // 旋转尺度误差分档：3%（常规打滑）/ 10%（地毯重打滑）
        struct NodeW { double x, y; };
        const NodeW base_wp[] = {{2, 13.5}, {13.5, 13.5},
                                 {13.5, 28}, {2, 28}, {2, 13.5}};
        std::vector<NodeW> wp;
        for (int lap = 0; lap < 2; ++lap) {
            for (const auto& w : base_wp) { wp.push_back(w); }
        }
        wp.push_back({2, 13.75});
        std::vector<Pose2> traj;
        {
            Pose2 cur(2, 13.5, 0);
            for (std::size_t seg = 0; seg + 1 < wp.size(); ++seg) {
                const double dx = wp[seg + 1].x - wp[seg].x;
                const double dy = wp[seg + 1].y - wp[seg].y;
                const double n = std::hypot(dx, dy) / 0.25;
                for (int i = 0; i < static_cast<int>(n); ++i) {
                    cur.x += dx / n;
                    cur.y += dy / n;
                    cur.theta = std::atan2(dy, dx);
                    traj.push_back(cur);
                }
            }
        }
        std::normal_distribution<double> gn01(0, 0.01);
        std::normal_distribution<double> gyro_n(0, 0.0002);  // 消费级 MEMS
        // 长厚人群段（150-230 步，对应 ~20 帧重度遮挡扫描）
        auto crowd_long = [&](int k) -> std::vector<sim::DynBox> {
            if (k < 150 || k > 230) { return {}; }
            const auto& p = traj[static_cast<std::size_t>(k)];
            return {{p.x + 1.2 * std::cos(p.theta),
                     p.y + 1.2 * std::sin(p.theta), 0.4, 0.4},
                    {p.x + 2.2 * std::cos(p.theta + 0.5),
                     p.y + 2.2 * std::sin(p.theta + 0.5), 0.4, 0.4},
                    {p.x + 2.0 * std::cos(p.theta - 0.5),
                     p.y + 2.0 * std::sin(p.theta - 0.5), 0.4, 0.4}};
        };

        std::cout << "\n== E10 IMU yaw fusion (crowd 80 frames) ==\n";
        for (int slip_lvl = 0; slip_lvl < 2; ++slip_lvl) {
            const double rot_scale_err = slip_lvl == 0 ? 1.03 : 1.10;
            std::cout << "  [wheel rot slip "
                      << (slip_lvl == 0 ? "3% (realistic)" : "10% (carpet)")
                      << "]\n";
            for (int variant = 0; variant < 2; ++variant) {
                const bool use_imu = variant == 1;
                CoreConfig cc;
                cc.match_opt.window_lin = 0.25;
                cc.use_imu_yaw = use_imu;
                LocalizerCore core(cc);
                core.set_map(field);
                core.set_matcher(comp);
                core.on_odom(traj.front());
                core.set_initial_pose(traj.front());
                Pose2 odom = traj.front();
                double imu_yaw = 0.0;  // 陀螺积分（真值 + MEMS 噪声）
                double err_sum = 0, err_max = 0;
                int n_total = 0, n_acc = 0, n_reject = 0;
                for (std::size_t k = 1; k < traj.size(); ++k) {
                    const Pose2 prev = traj[k - 1];
                    const Pose2 gt = traj[k];
                    const double c = std::cos(prev.theta);
                    const double s = std::sin(prev.theta);
                    const double mx = gt.x - prev.x, my = gt.y - prev.y;
                    const Pose2 d_local(c * mx + s * my, -s * mx + c * my,
                                        gt.theta - prev.theta);
                    const Pose2 d_odom(d_local.x * 1.02 + gn01(rng) * 0.01,
                                       d_local.y * 1.02 + gn01(rng) * 0.01,
                                       d_local.theta * rot_scale_err +
                                           gn01(rng) * 0.005);
                    odom = odom * d_odom;
                    core.on_odom(odom);
                    if (use_imu) {
                        imu_yaw += d_local.theta + gyro_n(rng);
                        core.on_imu_yaw(imu_yaw);
                    }
                    if (k % 4 == 0) {
                        const auto scan = sim::make_scan(
                            grid, gt, sim_cfg, rng,
                            crowd_long(static_cast<int>(k)));
                        const auto o = core.on_scan(scan);
                        const double e = std::hypot(o.pose.x - gt.x,
                                                    o.pose.y - gt.y);
                        err_sum += e;
                        err_max = std::max(err_max, e);
                        ++n_total;
                        if (o.accepted) { ++n_acc; } else { ++n_reject; }
                    }
                }
                printf("    %s : mean_err %.4f m, max_err %.4f m, accept "
                       "%.1f%% (reject %d)\n",
                       use_imu ? "B imu_fused " : "A wheel_only",
                       err_sum / n_total, err_max, 100.0 * n_acc / n_total,
                       n_reject);
            }
        }
    }

    std::cout << "\nbenchmark done.\n";
    return 0;
}
