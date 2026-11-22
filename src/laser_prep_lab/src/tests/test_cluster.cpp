// ============================================================================
// 聚类与线段提取：自适应断裂（远墙不碎、近物不粘）+ split-and-merge。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "cluster.hpp"

using namespace lpl;

static int failures = 0;
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::printf("FAIL %s (line %d)\n", msg, __LINE__);    \
            ++failures;                                           \
        }                                                         \
    } while (0)

// 从角度 a0 起按束序生成"半径 r 的弧"点列（0.5° 间隔）
static icp::PointCloud2 arc(double a0, int n, double r)
{
    icp::PointCloud2 pts;
    const double d = 0.5 * kPi / 180.0;
    for (int i = 0; i < n; ++i) {
        const double a = a0 + i * d;
        pts.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return pts;
}
static std::vector<int> iota(int n)
{
    std::vector<int> b(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) b[static_cast<std::size_t>(i)] = i;
    return b;
}

int main()
{
    const double ang_step = 0.5 * kPi / 180.0;
    const ClusterParams cp;  // base 0.06 / stretch 3 / min 3

    // ---------- 8m 远的整面墙：一簇 ----------
    {
        auto pts = arc(0.0, 181, 8.0);  // 束间距 8×0.0087 ≈ 7cm
        auto clusters = cluster_by_break(pts, iota(181), ang_step, cp);
        // 允许阈值 = 0.06 + 3×8×0.0087 ≈ 0.27m > 7cm → 不碎
        CHECK(clusters.size() == 1, "远墙保持 1 簇");
    }

    // ---------- 近场两个箱子，间隙 0.5m：两簇 ----------
    {
        // 箱A：-30°~-5° 半径 1.5m 的弧；箱B：+5°~+30° 半径 1.5m
        // 中间角度上点距 = 2×1.5×sin(10°) ≈ 0.52m > 阈值 → 断
        auto a = arc(-30.0 * kPi / 180.0, 51, 1.5);
        auto b = arc(5.0 * kPi / 180.0, 51, 1.5);
        icp::PointCloud2 pts = a;
        std::vector<int> beams = iota(51);
        pts.insert(pts.end(), b.begin(), b.end());
        for (int i = 0; i < 51; ++i) beams.push_back(51 + i);
        auto clusters = cluster_by_break(pts, beams, ang_step, cp);
        CHECK(clusters.size() == 2, "0.5m 间隙的两个箱子 → 2 簇");
        if (clusters.size() == 2) {
            CHECK(clusters[0].size() == 51 && clusters[1].size() == 51,
                  "每箱各 51 点");
        }
    }

    // ---------- 少于 min_points 的残渣不输出 ----------
    {
        auto pts = arc(0.0, 2, 5.0);  // 2 点的"簇"
        auto clusters = cluster_by_break(pts, iota(2), ang_step, cp);
        CHECK(clusters.empty(), "2 点残渣被 min_points 滤掉");
    }

    // ---------- 线段提取：L 形一簇 → 两段 ----------
    {
        icp::PointCloud2 pts;
        std::vector<int> beams;
        // 竖直段：沿 y 从 (2,0) 到 (2,5)，步长 0.1（模拟 0.5°@~11m 的密度）
        for (int i = 0; i < 51; ++i) {
            pts.push_back({2.0, 0.1 * i});
            beams.push_back(i);
        }
        // 水平段：沿 x 从 (2,5) 到 (7,5)
        for (int i = 0; i < 51; ++i) {
            pts.push_back({2.0 + 0.1 * i, 5.0});
            beams.push_back(51 + i);
        }
        auto clusters = cluster_by_break(pts, beams, ang_step, cp);
        CHECK(clusters.size() == 1, "连续折线是一簇");
        LineParams lp;
        lp.split_tol = 0.02;
        auto lines = extract_lines(clusters[0], lp);
        std::printf("  L 形 → %zu 段线\n", lines.size());
        CHECK(lines.size() == 2, "L 形劈成 2 段");
        if (lines.size() == 2) {
            // 每段残差都该很小
            CHECK(lines[0].residual < 0.02 && lines[1].residual < 0.02,
                  "两段拟合残差 < 2cm");
        }
    }

    // ---------- 直墙：一整段，残差近零 ----------
    {
        icp::PointCloud2 pts;
        std::vector<int> beams;
        for (int i = 0; i < 50; ++i) {
            pts.push_back({5.0, 0.15 * i});
            beams.push_back(i);
        }
        auto clusters = cluster_by_break(pts, beams, ang_step, cp);
        auto lines = extract_lines(clusters[0], LineParams{});
        CHECK(lines.size() == 1 && lines[0].residual < 1e-9,
              "纯直线一整段、零残差");
    }

    if (failures == 0) std::printf("test_cluster: OK\n");
    return failures == 0 ? 0 : 1;
}
