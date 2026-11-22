// ============================================================================
// laser_prep_eval —— 清洗链的效果评估（消融实验 + 病灶注入）。
//
// 评估哲学：前处理自己没有 ground truth，"删了多少点"不等于"删得对"。
// 三层评估，每层回答不同的问题：
//
//   ① 注入实验（真实数据 + 已知病灶）
//      往真实扫描里人工种椒盐/鬼点（位置已知），跑清洗链，
//      量"种下的病灶删掉多少"（召回）和"好束被冤枉多少"（误杀）。
//      比纯合成更硬：数据的一切统计特性（噪声、走廊、遮挡）都是真的，
//      只有病灶是已知副本。
//
//   ② 下游消融（最有说服力的一层）
//      同一份数据跑三条位姿链：
//        raw  : 不处理（只有解析期无回波 NaN）
//        clean: 只过清洗链 ①-④
//        full : 清洗 + deskew
//      比 ICP 配准质量（RMS 残差 / 对应点数 / 失败次数）和世界点云的
//      "墙锐利度"（局部厚度）。前处理的价值由下游证明，
//      不由删除率证明。
//
//   ③ 单帧结构指标
//      每帧聚类做总体最小二乘直线拟合（PCA），RMS 垂距残差——
//      椒盐/鬼点/运动拖影都不服从任何直线，全部在这里现形。
//
// 用法：laser_prep_eval --input <carmen.log> [--max-scans N] [--inject N]
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include "accumulate.hpp"
#include "carmen.hpp"
#include "cluster.hpp"
#include "deskew.hpp"
#include "filters.hpp"
#include "icp.hpp"
#include "polar.hpp"

using namespace lpl;

namespace {

struct Args {
    std::string input;
    long max_scans{0};
    long inject_scans{2000};
    // 0=按"相邻间隔×0.25"估计；负值 = 向后跨（时间戳是扫描结束时用）；
    // tiny 值（如 0.001）≈ 关 deskew 的对照
    double scan_duration{0.0};
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
        else if (s == "--max-scans") a.max_scans = std::atol(next().c_str());
        else if (s == "--inject") a.inject_scans = std::atol(next().c_str());
        else if (s == "--scan-duration") {
            try { a.scan_duration = std::stod(next()); } catch (...) {}
        }
        else if (s == "--help" || s == "-h") a.show_help = true;
    }
    return a;
}

double median(std::vector<double> v)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

double mean95(const std::vector<double>& v_in)
{
    if (v_in.empty()) return 0.0;
    std::vector<double> v(v_in);
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(v.size() * 0.95)];
}

// ----------------------------------------------------------------------------
// 结构残差：一帧点云聚成簇，每簇做总体最小二乘直线拟合（PCA 主轴），
// 残差 = sqrt(λ_min/n)——点到最佳拟合直线的 RMS 垂距。
// ----------------------------------------------------------------------------

// 一簇的 PCA 拟合 RMS 残差（nullopt：点太少退化）
std::optional<double> pca_line_residual(const icp::PointCloud2& pts)
{
    const std::size_t n = pts.size();
    if (n < 5) return std::nullopt;
    double sx = 0, sy = 0;
    for (const auto& p : pts) { sx += p.x; sy += p.y; }
    const double mx = sx / n, my = sy / n;
    double sxx = 0, syy = 0, sxy = 0;
    for (const auto& p : pts) {
        const double dx = p.x - mx, dy = p.y - my;
        sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
    }
    // 2×2 对称阵小特征值（垂向方差×n）
    const double tr = sxx + syy;
    const double det = sxx * syy - sxy * sxy;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
    const double lambda_min = tr / 2.0 - disc;
    return std::sqrt(std::max(0.0, lambda_min) / static_cast<double>(n));
}

// 一帧的加权平均结构残差（按簇点数加权；没有任何可拟合簇 → nullopt）
std::optional<double> scan_line_residual(const icp::PointCloud2& pts,
                                         double ang_step)
{
    if (pts.size() < 10) return std::nullopt;
    std::vector<int> beams(pts.size());
    std::iota(beams.begin(), beams.end(), 0);
    auto clusters = cluster_by_break(pts, beams, ang_step, ClusterParams{});
    double wsum = 0, rsum = 0;
    for (const auto& c : clusters) {
        if (auto r = pca_line_residual(c.points)) {
            const double w = static_cast<double>(c.points.size());
            rsum += *r * w;
            wsum += w;
        }
    }
    if (wsum == 0) return std::nullopt;
    return rsum / wsum;
}

// ----------------------------------------------------------------------------
// 世界点云的"墙锐利度"：0.05m 栅格，每格 3×3 邻域内的点做 PCA，
// 厚度 = sqrt(λ_min/n)。尖锐的墙 ≈ 点云噪声厚度；漂移/鬼点/拖影
// 会把同一面墙画厚。只统计 ≥8 点的格子（单帧孤区没统计意义）。
// ----------------------------------------------------------------------------

struct WorldSharpness {
    long cells{0};         // 有点的格子数（地图规模）
    long measured{0};      // 参与厚度统计的格子数
    double thickness{0.0}; // 平均墙厚 (m)
};

WorldSharpness world_sharpness(const icp::PointCloud2& world, double res = 0.05)
{
    std::map<std::pair<int, int>, std::vector<icp::Point2>> grid;
    for (const auto& p : world) {
        const int cx = static_cast<int>(std::floor(p.x / res));
        const int cy = static_cast<int>(std::floor(p.y / res));
        grid[{cx, cy}].push_back(p);
    }
    WorldSharpness out;
    out.cells = static_cast<long>(grid.size());
    double thick_sum = 0;
    for (const auto& [key, cell_pts] : grid) {
        // 3×3 邻域的点合起来做 PCA
        icp::PointCloud2 patch;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto it = grid.find({key.first + dx, key.second + dy});
                if (it != grid.end()) {
                    patch.insert(patch.end(), it->second.begin(),
                                  it->second.end());
                }
            }
        }
        if (patch.size() < 8) continue;
        if (auto r = pca_line_residual(patch)) {
            thick_sum += *r;
            ++out.measured;
        }
    }
    out.thickness = out.measured > 0 ? thick_sum / out.measured : 0.0;
    return out;
}

// ----------------------------------------------------------------------------
// 消融：一条位姿链
// ----------------------------------------------------------------------------

struct ChainConfig {
    const char* name;
    bool clean;
    bool deskew;
};

struct ChainMetrics {
    std::vector<double> rms;        // ICP 收敛 RMS（成功配准的帧）
    std::vector<double> iters;
    std::vector<double> corresps;
    long align_failures{0};
    long frames{0};
    std::vector<double> scan_res;  // 每帧结构残差
    icp::Pose2 end_pose{};
    icp::PointCloud2 world;        // 采样世界点云
};

ChainMetrics run_chain(const ChainConfig& cfg, const carmen::CarmenData& data,
                       double duration, long max_scans)
{
    ChainMetrics m;
    AccumulatorParams ap;
    ScanAccumulator acc(ap);
    icp::Pose2 last_pose{}, last_odom{};
    bool has_last = false;
    const long n = max_scans > 0
                       ? std::min<long>(max_scans,
                                        static_cast<long>(data.scans.size()))
                       : static_cast<long>(data.scans.size());
    for (long fi = 0; fi < n; ++fi) {
        PolarScan s = data.scans[static_cast<std::size_t>(fi)];
        if (s.range.size() < 2) continue;
        if (cfg.clean) {
            FilterStats st;
            clean_frame(s, st);
        }
        if (cfg.deskew) {
            s.beam_dt = duration / static_cast<double>(s.range.size() - 1);
        }
        const DeskewResult d = deskew(s, data.odom);  // 无时序时诚实回退
        const icp::Pose2 odom_now = data.scan_odom[static_cast<std::size_t>(fi)];
        icp::Pose2 pose = odom_now;
        if (has_last) {
            const icp::Pose2 guess = compose(last_pose,
                                             delta(last_odom, odom_now));
            const icp::PointCloud2 target = acc.cloud();
            if (target.size() > 100) {
                auto res = icp::align(d.points, target, guess);
                if (res) {
                    pose = res->pose;
                    m.rms.push_back(res->rms_error);
                    m.iters.push_back(static_cast<double>(res->iterations));
                    m.corresps.push_back(
                        static_cast<double>(res->correspondences));
                } else {
                    pose = guess;
                    ++m.align_failures;
                }
            } else {
                pose = guess;
            }
        }
        has_last = true;
        last_pose = pose;
        last_odom = odom_now;
        m.end_pose = pose;
        ++m.frames;
        // 目标地图要长出来：本帧配完就是下一帧的参照（忘了这行，
        // ICP 永远没有 target，全部指标归零——写评估工具真实踩的坑）
        acc.push(d.points, pose);

        if (auto r = scan_line_residual(d.points, s.angle_inc)) {
            m.scan_res.push_back(*r);
        }
        if (fi % 20 == 0) {
            for (const auto& p : d.points) m.world.push_back(transform(pose, p));
        }
    }
    return m;
}

// ----------------------------------------------------------------------------
// 病灶注入：真实扫描 + 已知位置的椒盐/鬼点副本
// ----------------------------------------------------------------------------

struct InjectionReport {
    long spikes_planted{0}, spikes_recalled{0};
    long ghosts_planted{0}, ghosts_recalled{0};
    long false_removals{0};   // 没种病灶、基线也没删、这轮却删了
    long beams_total{0};
};

// 简单确定性伪随机（可复现）
inline unsigned long rng_next(unsigned long& state)
{
    state = state * 6364136223846793005UL + 1442695040888963407UL;
    return (state >> 33);
}

InjectionReport inject_eval(const carmen::CarmenData& data, long n_scans)
{
    InjectionReport rep;
    unsigned long rng = 20261121UL;
    const long n = std::min<long>(n_scans,
                                  static_cast<long>(data.scans.size()));
    for (long fi = 0; fi < n; ++fi) {
        const PolarScan& orig = data.scans[static_cast<std::size_t>(fi)];
        const int nb = orig.size();
        if (nb < 30) continue;
        rep.beams_total += nb;

        // 基线：不种病灶时的删除集合
        PolarScan base = orig;
        std::vector<std::uint8_t> why0;
        FilterStats st_base;
        clean_frame(base, st_base, &why0);

        // 种椒盐：找"两侧邻居一致"的束（真表面的典型位置），
        // 把它挪 1.0m ——这是粉尘/雨滴的标准形态
        PolarScan inj = orig;
        std::vector<std::uint8_t> why1;
        std::vector<int> planted_spikes, planted_ghosts;
        int planted = 0;
        for (int attempt = 0; attempt < 12 && planted < 4; ++attempt) {
            const int i = static_cast<int>(rng_next(rng) % static_cast<unsigned long>(nb));
            if (!inj.valid(i) || i < 2 || i > nb - 3) continue;
            int li = -1, ri = -1;
            neighbors(inj, i, li, ri);
            if (li < 0 || ri < 0) continue;
            const double rl = inj.range[static_cast<std::size_t>(li)];
            const double rr = inj.range[static_cast<std::size_t>(ri)];
            if (std::abs(rl - rr) > 0.2) continue;  // 要贴着平表面
            inj.range[static_cast<std::size_t>(i)] += 1.0;
            planted_spikes.push_back(i);
            ++planted;
        }
        rep.spikes_planted += static_cast<long>(planted_spikes.size());

        // 种鬼点：找"两侧平坦的深度跳变对"，把远侧换成中点值
        planted = 0;
        for (int attempt = 0; attempt < 16 && planted < 2; ++attempt) {
            const int i = static_cast<int>(rng_next(rng) % static_cast<unsigned long>(nb - 4)) + 1;
            if (!inj.valid(i) || !inj.valid(i + 1) || i < 1 || i > nb - 3) continue;
            const double a = inj.range[static_cast<std::size_t>(i)];
            const double b = inj.range[static_cast<std::size_t>(i + 1)];
            if (std::abs(a - b) < 1.5) continue;  // 要真跳变
            int ll = -1, rr2 = -1;
            for (int k = i - 1; k >= 0 && ll < 0; --k) if (inj.valid(k)) ll = k;
            for (int k = i + 2; k < nb && rr2 < 0; ++k) if (inj.valid(k)) rr2 = k;
            if (ll < 0 || rr2 < 0) continue;
            const double rll = inj.range[static_cast<std::size_t>(ll)];
            const double rrr = inj.range[static_cast<std::size_t>(rr2)];
            if (std::abs(rll - a) > 0.3 || std::abs(rrr - b) > 0.3) continue;  // 两侧要平
            const double mid = (a + b) / 2.0;
            const bool far_is_b = b > a;
            const int j = far_is_b ? i + 1 : i;
            inj.range[static_cast<std::size_t>(j)] = mid;
            planted_ghosts.push_back(j);
            ++planted;
        }
        rep.ghosts_planted += static_cast<long>(planted_ghosts.size());

        // 清洗注入版
        FilterStats st_inj;
        clean_frame(inj, st_inj, &why1);

        // 计分
        for (const int i : planted_spikes) {
            if (why1[static_cast<std::size_t>(i)] != 0) ++rep.spikes_recalled;
        }
        for (const int i : planted_ghosts) {
            if (why1[static_cast<std::size_t>(i)] != 0) ++rep.ghosts_recalled;
        }
        for (int i = 0; i < nb; ++i) {
            const std::uint8_t w1 = why1[static_cast<std::size_t>(i)];
            const bool planted_here =
                std::find(planted_spikes.begin(), planted_spikes.end(), i) !=
                    planted_spikes.end() ||
                std::find(planted_ghosts.begin(), planted_ghosts.end(), i) !=
                    planted_ghosts.end();
            if (w1 != 0 && !planted_here &&
                why0[static_cast<std::size_t>(i)] == 0) {
                ++rep.false_removals;
            }
        }
    }
    return rep;
}

}  // namespace

int main(int argc, char** argv)
{
    const Args args = parse_args(argc, argv);
    if (args.show_help || args.input.empty()) {
        std::printf("用法: laser_prep_eval --input <carmen.log> "
                    "[--max-scans N] [--inject N]\n");
        return args.input.empty() ? 1 : 0;
    }
    auto data = carmen::read(args.input);
    if (!data) {
        std::printf("读取失败: %s\n", data.error().c_str());
        return 1;
    }
    std::printf("日志: scans=%zu\n", data->scans.size());

    // ================= ① 病灶注入 =================
    std::printf("\n== ① 病灶注入实验（真实扫描 + 已知病灶副本）==\n");
    const InjectionReport inj = inject_eval(*data, args.inject_scans);
    std::printf("椒盐: 种 %ld / 检出 %ld（召回 %.1f%%）\n", inj.spikes_planted,
                inj.spikes_recalled,
                inj.spikes_planted ? 100.0 * inj.spikes_recalled / inj.spikes_planted : 0.0);
    std::printf("鬼点: 种 %ld / 检出 %ld（召回 %.1f%%）\n", inj.ghosts_planted,
                inj.ghosts_recalled,
                inj.ghosts_planted ? 100.0 * inj.ghosts_recalled / inj.ghosts_planted : 0.0);
    std::printf("误杀: %ld 束（占 %.4f%%，基线本来就删的不计）\n",
                inj.false_removals,
                inj.beams_total
                    ? 100.0 * inj.false_removals / inj.beams_total
                    : 0.0);

    // ================= ② 下游消融 =================
    // 扫描时长估计（与 main.cpp 同款假设）
    double duration = 0.0;
    if (data->scans.size() > 10) {
        std::vector<double> dts;
        for (std::size_t i = 1; i + 1 < data->scans.size(); ++i) {
            dts.push_back(data->scans[i + 1].stamp - data->scans[i].stamp);
        }
        std::sort(dts.begin(), dts.end());
        duration = std::clamp(0.25 * dts[dts.size() / 2], 0.02, 0.4);
    }

    std::printf("\n== ② 下游消融：同一条 ICP 位姿链，不同前处理 ==\n");
    if (args.scan_duration != 0.0) {
        duration = args.scan_duration;
        std::printf("（--scan-duration %.4fs 覆盖自动估计）\n", duration);
    }
    std::printf("%-7s %10s %8s %8s %8s %6s | %8s %8s | %8s %8s\n",
                "配置", "RMS中位", "RMS95", "迭代", "对应点", "失败",
                "帧残差", "帧残差95", "墙厚(m)", "格数");
    for (const ChainConfig cfg : {ChainConfig{"raw", false, false},
                                   ChainConfig{"clean", true, false},
                                   ChainConfig{"full", true, true}}) {
        const ChainMetrics m = run_chain(cfg, *data, duration, args.max_scans);
        const WorldSharpness ws = world_sharpness(m.world);
        std::printf("%-7s %10.4f %8.4f %8.2f %8.1f %6ld | %8.4f %8.4f | %8.4f %8ld\n",
                    cfg.name, median(m.rms), mean95(m.rms),
                    median(m.iters), median(m.corresps), m.align_failures,
                    median(m.scan_res), mean95(m.scan_res),
                    ws.thickness, ws.cells);
    }
    std::printf("\n判读：RMS=配准残差（越低=配得越准）；帧残差=单帧结构直线度\n"
                "（低=椒盐/鬼点少）；墙厚=世界点云局部厚度（低=漂移/重影少）。\n"
                "下游指标不改善的前处理没有存在价值——这是评估的最终裁判。\n");
    return 0;
}
