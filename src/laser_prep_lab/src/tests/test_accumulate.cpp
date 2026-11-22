// ============================================================================
// 多帧融合：位姿补偿拼接 + 距离窗裁剪。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "accumulate.hpp"

using namespace lpl;

static int failures = 0;
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::printf("FAIL %s (line %d)\n", msg, __LINE__);    \
            ++failures;                                           \
        }                                                         \
    } while (0)

int main()
{
    // ---------- 两帧不同位姿看同一面墙 → 融合后都落在真实墙线上 ----------
    // 真墙：x = 10 的竖墙。帧1 在 (0,0,0) 看到 (10, y)；帧2 在 (4,2,0)
    // 看到 (10, y)（世界系）。融合云里两条束列应重合在 x=10。
    ScanAccumulator acc;
    icp::PointCloud2 s1, s2;
    for (int i = 0; i < 10; ++i) {
        s1.push_back({10.0 - 0.0, -1.0 + 0.2 * i});   // 帧1传感器系 = 世界系
        s2.push_back({10.0 - 4.0, -1.0 + 0.2 * i - 2.0});  // 帧2系: 世界−(4,2)
    }
    acc.push(s1, icp::Pose2{0, 0, 0});
    acc.push(s2, icp::Pose2{4, 2, 0});
    const auto fused = acc.cloud();
    CHECK(fused.size() == 20, "两帧全部融合");
    for (const auto& p : fused) {
        CHECK(std::abs(p.x - 10.0) < 1e-9, "融合点都落在真实墙线上");
        if (std::abs(p.x - 10.0) >= 1e-9) break;
    }

    // ---------- 环形缓冲：超过 max_scans 丢最老帧 ----------
    ScanAccumulator acc2;
    for (int i = 0; i < 20; ++i) {
        icp::PointCloud2 one{{0.0, 0.0}};
        acc2.push(one, icp::Pose2{static_cast<double>(i), 0, 0});
    }
    CHECK(acc2.size() == 16, "窗口封顶 16 帧");
    const auto c2 = acc2.cloud();
    // 帧位姿 x ∈ [4..19]（16 帧窗口），最新位姿 (19,0)，
    // range_window=15 → x=4 恰好在界内保留，x=3 应该被裁
    bool has_old = false;
    for (const auto& p : c2) {
        if (p.x < 4.0) has_old = true;
    }
    CHECK(!has_old, "离最新位姿超过 range_window 的点被裁掉");

    // ---------- 旋转位姿的补偿也没问题 ----------
    ScanAccumulator acc3;
    const double c = std::cos(kPi / 2), s = std::sin(kPi / 2);
    // 帧1 位姿 (0,0,90°)，它的传感器系点 (1,0) → 世界 (0,1)
    acc3.push({{1.0, 0.0}}, icp::Pose2{0, 0, kPi / 2});
    const auto c3 = acc3.cloud();
    CHECK(c3.size() == 1 && std::abs(c3[0].x - 0.0) < 1e-9 &&
              std::abs(c3[0].y - 1.0) < 1e-9,
          "旋转位姿下的补偿正确");
    (void)c;
    (void)s;

    if (failures == 0) std::printf("test_accumulate: OK\n");
    return failures == 0 ? 0 : 1;
}
