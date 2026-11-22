// ============================================================================
// 动态障碍判别："被透视过的位置上现在站着东西" → 动态。
// 场景：走廊里机器人静止观察数帧（建立透视记忆），然后一个人
// 走到走廊中间——人所在的位置此前被射线穿过成百上千次。
// 对照组：墙（从未被透视）与"从未观察过的新区域"都判静态。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "cluster.hpp"
#include "dynamic.hpp"

using namespace lpl;

static int failures = 0;
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::printf("FAIL %s (line %d)\n", msg, __LINE__);    \
            ++failures;                                           \
        }                                                         \
    } while (0)

// 走廊扫描：机器人在 (0,0) 朝 +x，墙在 y=±3，x ∈ [-6, 6]，
// 从 -80° 到 +80° 每度一束打到墙上。
static icp::PointCloud2 corridor_scan()
{
    icp::PointCloud2 pts;
    for (int deg = -80; deg <= 80; ++deg) {
        const double a = deg * kPi / 180.0;
        const double c = std::cos(a), s = std::sin(a);
        // 上下墙都各打一束（简化：取两侧交点）
        if (std::abs(s) > 1e-6) {
            const double t = 3.0 / std::abs(s);
            const double x = t * c;
            if (std::abs(x) <= 6.0) {
                pts.push_back({x, s > 0 ? 3.0 : -3.0});
            }
        }
        // 端墙 x=6
        if (c > 1e-6) {
            const double t = 6.0 / c;
            if (t * std::abs(s) <= 3.0) {
                pts.push_back({6.0, t * s});
            }
        }
    }
    return pts;
}

// 把一串点变成一个"簇"（借 Cluster 结构复用判别接口）
static Cluster as_cluster(const icp::PointCloud2& pts)
{
    Cluster c;
    c.points = pts;
    icp::Point2 cen{0, 0};
    for (const auto& p : pts) {
        cen.x += p.x;
        cen.y += p.y;
    }
    c.centroid = {cen.x / pts.size(), cen.y / pts.size()};
    return c;
}

int main()
{
    const DynamicParams dp;
    FreeSpaceGrid grid(dp.grid_res, dp.grid_half);

    // ---------- 建立透视记忆：同一位置观察 10 帧 ----------
    const icp::Pose2 pose{0, 0, 0};
    const icp::PointCloud2 scan = corridor_scan();
    for (int i = 0; i < 10; ++i) {
        record_frame(pose, scan, grid);
    }

    // 现在三样东西出现在场景里：
    //  A. 人：站在 (3, 0) 附近——走廊正中，被透视过无数次
    //  B. 墙的一部分：(0, 3) 附近——从未被透视
    //  C. 观测盲区里的新东西：(-10, 0)——从没看过（格子里无证据）
    const icp::PointCloud2 person{{2.9, -0.1}, {3.0, 0.0}, {3.1, 0.1},
                                  {3.05, -0.05}, {3.0, 0.1}};
    const icp::PointCloud2 wall_bit{{0.0, 3.0}, {0.05, 3.0}, {-0.05, 3.0},
                                    {0.1, 3.0}, {-0.1, 3.0}};
    const icp::PointCloud2 unseen{{-10.0, 0.0}, {-9.9, 0.05}, {-10.1, -0.05}};

    std::vector<Cluster> clusters{as_cluster(person), as_cluster(wall_bit),
                                  as_cluster(unseen)};
    const auto verdicts = judge_clusters(clusters, grid, dp);

    std::printf("  人(被透视区) ratio=%.2f  墙 ratio=%.2f  盲区 ratio=%.2f\n",
                verdicts[0].seen_through, verdicts[1].seen_through,
                verdicts[2].seen_through);
    CHECK(verdicts[0].is_dynamic, "站在透视记忆上的人 → 动态");
    CHECK(!verdicts[1].is_dynamic, "墙 → 静态");
    CHECK(!verdicts[2].is_dynamic, "从未观察过的区域 → 静态（未知≠动态）");

    // ---------- 机理检查：人的位置的格子确实积累过自由证据 ----------
    CHECK(grid.seen_through_ratio({{3.0, 0.0}}) > 0.5,
          "(3,0) 的格子确实被反复透视过");

    // ---------- 人挪了位置：仍然动态（透视记忆是空间性的） ----------
    const icp::PointCloud2 person_moved{{1.0, 0.5}, {1.05, 0.45}, {0.95, 0.55}};
    const auto v2 = judge_clusters({as_cluster(person_moved)}, grid, dp);
    CHECK(v2[0].is_dynamic, "挪了位置的人仍判动态");

    if (failures == 0) std::printf("test_dynamic: OK\n");
    return failures == 0 ? 0 : 1;
}
