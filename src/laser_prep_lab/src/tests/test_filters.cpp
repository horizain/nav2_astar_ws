// ============================================================================
// 清洗链 ①-④ 的合成真值验证。每个用例都是"造一个已知病灶的扫描，
// 看清掉的是病灶、留下的是正常结构"。
//
// 几何构造纪律（踩坑总结）：
//  - 墙扫描别超过 ~160° 张角：端部束与墙的夹角小于 10° 掠射阈值，
//    deshadow 会合法地删掉它们——那是掠射，不是误删；
//  - "盲区"必须真的低于 min_range（0.05m）：0.2m 的孤立点不是盲区，
//    是近处假目标，它引发的是另一种合法级联（邻束掠射）。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "filters.hpp"

using namespace lpl;

static int failures = 0;
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::printf("FAIL %s (line %d)\n", msg, __LINE__);    \
            ++failures;                                           \
        }                                                         \
    } while (0)

// 平墙扫描：角度覆盖 20°..160°（sin > 0.34，端部入射余角 20°，
// 远离 10° 掠射阈值——构造本身不触发 deshadow）。
static PolarScan wall_scan(int beams, double wall_y)
{
    PolarScan s;
    s.angle_min = 20.0 * kPi / 180.0;
    s.angle_inc = 140.0 * kPi / 180.0 / (beams - 1);
    for (int i = 0; i < beams; ++i) {
        s.range.push_back(wall_y / std::sin(s.angle(i)));
    }
    return s;
}

int main()
{
    // ---------- ① 量程门控（单独验证，不掺其他清洗器）----------
    {
        PolarScan s;
        s.angle_min = 0;
        s.angle_inc = 0.01;
        s.range = {0.005, 4.0, 81.83, 12.0, -1.0, 5.0};
        range_gate(s, GateParams{});
        CHECK(!s.valid(0), "盲区(0.005m)被门控");
        CHECK(s.valid(1), "正常回波保留");
        CHECK(!s.valid(2), "占位值 81.83 被门控");
        CHECK(s.valid(3), "量程内保留");
        CHECK(!s.valid(4), "负值被门控");
        CHECK(s.valid(5), "正常回波保留");
    }

    // ---------- ② 椒盐孤点：墙上的单束噪声 ----------
    {
        PolarScan s = wall_scan(161, 2.0);
        const int mid = 80;  // a=90°，range=2.0
        s.range[static_cast<std::size_t>(mid)] = 3.2;  // 比两侧墙远 1.2m
        FilterStats st;
        clean_frame(s, st);
        CHECK(!s.valid(mid), "椒盐束被剔除");
        CHECK(st.despiked == 1, "椒盐计数 = 1");
        CHECK(st.removed() == 1, "直墙上只有这一个病灶");
    }

    // ---------- ② 反例：墙角台阶不是椒盐、也不是鬼点 ----------
    {
        // 左墙 3m、右墙 6m，台阶边缘一束 3.05m 贴左墙——
        // 真实边缘表面，谁都不许动它
        PolarScan s = wall_scan(161, 3.0);
        for (int i = 0; i < 100; ++i) {
            s.range[static_cast<std::size_t>(i)] = 3.0;
        }
        s.range[static_cast<std::size_t>(100)] = 3.05;  // 边缘束，贴左墙
        for (int i = 101; i < 161; ++i) {
            s.range[static_cast<std::size_t>(i)] = 6.0;
        }
        FilterStats st;
        clean_frame(s, st);
        CHECK(s.valid(100), "墙角贴边束保留（side_gap 保护）");
        CHECK(st.demixed == 0, "贴边中间值不判鬼点");
    }

    // ---------- ③ 混合像素：门口的桥接鬼点 ----------
    {
        // 左墙 2m、右墙 5m（中间是门/空隙），门边缘产生
        // 两束 3.2m、3.8m 的混合回波。
        // 注意 deshadow 的连带（合法）：门右侧墙的第一束（远侧边缘）
        // 与门边的连线几乎平行于它自己的光束——掠射不可靠，会被 ④
        // 删掉。这是门口场景的真实形态：远侧贴边束本来就不可信。
        PolarScan s = wall_scan(161, 3.0);
        for (int i = 0; i < 100; ++i) {
            s.range[static_cast<std::size_t>(i)] = 2.0;
        }
        s.range[static_cast<std::size_t>(100)] = 3.2;  // 混合像素
        s.range[static_cast<std::size_t>(101)] = 3.8;  // 混合像素
        for (int i = 102; i < 161; ++i) {
            s.range[static_cast<std::size_t>(i)] = 5.0;
        }
        FilterStats st;
        clean_frame(s, st);
        CHECK(!s.valid(100) && !s.valid(101), "两束混合像素被剔除");
        CHECK(st.demixed == 2, "鬼点计数 = 2");
        // 墙本体（离门口有余量的束）不受牵连
        CHECK(s.valid(95) && s.valid(107), "两侧真墙（留边距）保留");
        CHECK(!s.valid(102), "远侧贴边束被 deshadow 连带剔除（合法掠射）");
    }

    // ---------- ④ 掠射阴影：贴边擦过去的远回波 ----------
    {
        // 箱子正面 1m（束 0-59，角度 -30°..-0.5°），箱子右边缘的下一束
        // （a=0°）掠过边缘打到 4m 外——连线方向与该束光束几乎平行
        PolarScan s;
        s.angle_min = -30.0 * kPi / 180.0;
        s.angle_inc = 0.5 * kPi / 180.0;
        for (int i = 0; i < 60; ++i) s.range.push_back(1.0);
        s.range.push_back(4.0);  // 掠射束
        for (int i = 0; i < 60; ++i) s.range.push_back(4.2);
        FilterStats st;
        clean_frame(s, st);
        CHECK(!s.valid(60), "掠射远回波被剔除");
        CHECK(st.deshadowed >= 1, "阴影计数 ≥ 1");
        CHECK(s.valid(0), "遮挡物本体保留");
    }

    // ---------- 组合场景：病灶互不干扰，好束零误删 ----------
    {
        PolarScan s = wall_scan(161, 3.0);
        s.range[5] = 0.02;  // 真盲区（< min_range 0.05）
        s.range[80] = 9.0;  // 椒盐病灶（90° 方向，两侧墙 ~3.0）
        FilterStats st;
        clean_frame(s, st);
        CHECK(st.gated == 1 && st.despiked == 1, "各病灶分别计数");
        CHECK(st.removed() == 2, "总删除恰好 = 病灶数（零误删）");
        CHECK(!s.valid(5) && !s.valid(80), "删除的就是病灶本身");
        CHECK(s.valid(40) && s.valid(120), "墙上好束不受牵连");
    }

    if (failures == 0) std::printf("test_filters: OK\n");
    return failures == 0 ? 0 : 1;
}
