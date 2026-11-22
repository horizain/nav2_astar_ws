#pragma once
// ============================================================================
// 单帧清洗链（按推荐执行顺序）：
//   ① range_gate    量程/无效值门控——0 回波、NaN、超量程
//   ② despike       椒盐孤点剔除——单束突跳且两侧邻居一致
//   ③ demix         混合像素（鬼点）——深度跳变处的"桥接"假点
//   ④ deshadow       掠射阴影——光束贴着障碍物边缘擦过去的远回波
//
// 全部在 PolarScan 上原位工作（坏束写 NaN，不挪动数组——束序就是
// 拓扑，丢了顺序后面全废）。逐束分类计数进 FilterStats：
// "你到底删了我多少点、为什么"必须可交代。
// ============================================================================
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "polar.hpp"

namespace lpl {

struct GateParams {
    double min_range{0.05};  // 盲区（m）：比这近的回波是自碰/镜面内反射
    double max_range{30.0};  // 超量程（m）：CARMEN 占位值 81.83 在这挡掉
};

struct SpikeParams {
    double jump{0.35};  // 与两侧邻居都差超过此值 → 突跳 (m)
    // 诚实边界：单束细杆（电线、椅子腿）在这个判据下与噪声不可分——
    // 几何上它俩就是同一个东西。要救细杆靠多帧一致性（accumulate），
    // 或者有强度通道的雷达靠强度。别把 jump 调小来"救"——那会把
    // 噪声也救回来。
};

struct MixParams {
    double jump{0.5};          // 两侧构成深度跳变的绝对下限 (m)
    double side_gap{0.05};     // 绝对最小边距 (m)
    double mid_frac{0.15};     // 比例边距：离两侧都超过 15%·|rL−rR| 才算"居中"
    double slope_factor{3.0};  // 跳变须超过局部坡度的此倍数（渐变墙保护）
    double flat_k{4.0};        // "同表面"坡度容差系数：tol(r) = flat_k·r·Δθ
    // 混合像素机理：光束跨过深度不连续边界时一部分能量打在前表面、
    // 一部分打在后表面，雷达报一个"中间距离"。门/柱/箱边缘的经典重影。
    //
    // 判据四件套（每一条都对应一类真实踩坑，见 demix 实现内注释）：
    //   a. 跳变压过"较干净一侧"的局部坡度——"邻居差"在斜看走廊时
    //     只是坡度累积（1° 束距在 16m 处合法相邻差就有 1m+），初版
    //     没这条把 Intel 的走廊墙整段当鬼点删了（10% 的束）；
    //   b. 居中（比例边距）——贴边中间值是真实表面延续；
    //   c. 至少一侧是可靠表面（容差随距离缩放）。注意成对鬼点会
    //      互相污染坡度，所以 a/c 都取"任一侧满足"——干净的那一侧
    //      保证了"这里确实有两个不同深度的表面"。
    //
    // 诚实边界：跳变处的"过渡束"（斜走廊边缘）与真鬼点在 1° 分辨率
    // 下逐束不可分，删除它们是工程标准的"边界侵蚀"取舍：损失 1-2 束
    // 边缘信息，换地图不长重影。
};

struct ShadowParams {
    double grazing{10.0 * kPi / 180.0};  // 掠射角阈值 (rad)
    // 掠射机理：光束方向与相邻两 hit 点连线的夹角极小 = 光束贴着
    // 近处障碍的边缘擦过去打到了后面的东西。这个 hit 不可靠：
    // 稍微挪一毫米它就没了，地图上会画出一根"影子墙"。
};

struct FilterStats {
    long total_beams{0};
    long gated{0};       // ① 量程/无效
    long despiked{0};    // ② 椒盐
    long demixed{0};     // ③ 混合像素
    long deshadowed{0};  // ④ 掠射阴影
    [[nodiscard]] long removed() const
    {
        return gated + despiked + demixed + deshadowed;
    }
};

// ① 量程与无效值门控。最先执行：后面所有算法都假设 range 有限、
// 物理可达。注意只把"有限但超界"的改写为 NaN——解析期的 NaN
// （无回波）不动它，计数在 clean_frame 里统一做。
inline void range_gate(PolarScan& s, const GateParams& p)
{
    for (auto& r : s.range) {
        if (!std::isfinite(r)) continue;
        if (r < p.min_range || r > p.max_range) r = kNaN;
    }
}

// 左右最近的"当前有效"邻居（不越界，找不到 = -1）。
// 注意在清洗过程中调用：前面步骤删掉的束（NaN）会被跳过——
// 正因为如此各步骤必须按顺序跑，后面的算法看到的才是干净邻居。
inline void neighbors(const PolarScan& s, int i, int& li, int& ri)
{
    const int n = s.size();
    li = -1;
    ri = -1;
    for (int k = i - 1; k >= 0 && li < 0; --k) {
        if (s.valid(k)) li = k;
    }
    for (int k = i + 1; k < n && ri < 0; ++k) {
        if (s.valid(k)) ri = k;
    }
}

// ② 椒盐孤点：单束与两侧最近有效邻居都差 > jump，且邻居彼此一致。
// "邻居一致"是关键：墙角处 rL、rR 天差地别，那是真实台阶，不是椒盐。
inline void despike(PolarScan& s, const SpikeParams& p, FilterStats& st)
{
    for (int i = 0; i < s.size(); ++i) {
        if (!s.valid(i)) continue;
        int li = -1, ri = -1;
        neighbors(s, i, li, ri);
        if (li < 0 || ri < 0) continue;  // 边缘孤束：证据不足，不动
        const double r = s.range[static_cast<std::size_t>(i)];
        const double rl = s.range[static_cast<std::size_t>(li)];
        const double rr = s.range[static_cast<std::size_t>(ri)];
        const bool far_from_both =
            std::abs(r - rl) > p.jump && std::abs(r - rr) > p.jump;
        const bool neighbors_agree = std::abs(rl - rr) < p.jump;
        if (far_from_both && neighbors_agree) {
            s.range[static_cast<std::size_t>(i)] = kNaN;
            ++st.despiked;
        }
    }
}

// ③ 混合像素：夹在两个"可靠平面"之间、且落在跳变区间中部的假回波。
inline void demix(PolarScan& s, const MixParams& p, FilterStats& st)
{
    const int n = s.size();
    for (int i = 0; i < n; ++i) {
        if (!s.valid(i)) continue;
        int li = -1, ri = -1;
        neighbors(s, i, li, ri);
        // 边界防御：li ≥ 1 保证左侧二阶邻居存在；ri ∈ [0, n-2]
        // 保证右侧二阶邻居存在（ri = -1 是"右边没邻居"，必须显式排
        // 除——漏掉它就是 range[-1] 的越界读，UB）。
        if (li < 1 || ri < 0 || ri > n - 2) continue;
        // 二阶邻居（定义两侧表面的局部坡度）
        int ll = -1, rrr = -1;
        for (int k = li - 1; k >= 0 && ll < 0; --k) {
            if (s.valid(k)) ll = k;
        }
        for (int k = ri + 1; k < n && rrr < 0; ++k) {
            if (s.valid(k)) rrr = k;
        }
        if (ll < 0 || rrr < 0) continue;

        const double r = s.range[static_cast<std::size_t>(i)];
        const double rl = s.range[static_cast<std::size_t>(li)];
        const double rr = s.range[static_cast<std::size_t>(ri)];
        const double rll = s.range[static_cast<std::size_t>(ll)];
        const double rrrv = s.range[static_cast<std::size_t>(rrr)];

        // (a) 跳变（按邻居跨度归一）必须压过"较干净一侧"的局部坡度
        //     ——渐变墙的"邻居差"只是坡度×跨度（gating 挖出的洞会
        //     加宽跨度，让渐变墙看起来像跳变，然后级联吃掉整段墙，
        //     真实踩坑）。取 min：成对鬼点会互相污染自己一侧的坡度，
        //     干净的那一侧才是"这里有两个不同深度表面"的证人。
        //     span ≥ 2 恒成立（本束严格夹在 li、ri 之间）。
        const double slope_l = std::abs(rl - rll);
        const double slope_r = std::abs(rr - rrrv);
        const double jump = std::abs(rl - rr);
        const double span = static_cast<double>(ri - li);
        if (jump <=
            std::max(p.jump, p.slope_factor * span * std::min(slope_l, slope_r))) {
            continue;
        }
        // (b) 居中：离两侧都超过 max(绝对边距, 比例边距)。
        //     贴边 = 真实表面延续，居中 = 分裂假回波。
        const double margin = std::max(p.side_gap, p.mid_frac * jump);
        const double lo = std::min(rl, rr), hi = std::max(rl, rr);
        if (!(r > lo + margin && r < hi - margin)) continue;
        // (c) 至少一侧是可靠表面：其局部坡度在同表面容差内
        //     （容差随距离缩放：远处合法掠射墙的相邻差天然大）。
        const double tol_l = std::max(0.10, p.flat_k * rl * s.angle_inc);
        const double tol_r = std::max(0.10, p.flat_k * rr * s.angle_inc);
        if (slope_l < tol_l || slope_r < tol_r) {
            s.range[static_cast<std::size_t>(i)] = kNaN;
            ++st.demixed;
        }
    }
}

// ④ 掠射阴影：相邻两 hit 的连线与"远侧"光束方向夹角 < grazing
// → 远的那个不可靠（贴边擦过去的），删远侧。近侧是遮挡物本体。
// 链式掠射（连续多束都擦边）会在第一步删掉后自动断链：
// 被删束变 NaN，下一对的几何量全 NaN，不再触发删除——保守但安全。
inline void deshadow(PolarScan& s, const ShadowParams& p, FilterStats& st)
{
    std::vector<int> idx;  // 先抄一份有效束下标，避免迭代器失效类问题
    idx.reserve(s.range.size());
    for (int i = 0; i < s.size(); ++i) {
        if (s.valid(i)) idx.push_back(i);
    }
    for (std::size_t k = 0; k + 1 < idx.size(); ++k) {
        const int i = idx[k], j = idx[k + 1];
        const double ri = s.range[static_cast<std::size_t>(i)];
        const double rj = s.range[static_cast<std::size_t>(j)];
        if (!std::isfinite(ri) || !std::isfinite(rj)) continue;
        const double ai = s.angle(i), aj = s.angle(j);
        // 两 hit 点与连线（传感器系）
        const double cx = rj * std::cos(aj) - ri * std::cos(ai);
        const double cy = rj * std::sin(aj) - ri * std::sin(ai);
        const double chord = std::hypot(cx, cy);
        if (chord < 1e-9) continue;
        // 光束与连线的夹角（双侧）
        const double cosb_i = (std::cos(ai) * cx + std::sin(ai) * cy) / chord;
        const double cosb_j = (std::cos(aj) * cx + std::sin(aj) * cy) / chord;
        const double beta_i = std::acos(std::clamp(cosb_i, -1.0, 1.0));
        const double beta_j = std::acos(std::clamp(cosb_j, -1.0, 1.0));
        if (rj > ri && beta_j < p.grazing) {
            s.range[static_cast<std::size_t>(j)] = kNaN;
            ++st.deshadowed;
        } else if (ri > rj && beta_i < p.grazing) {
            s.range[static_cast<std::size_t>(i)] = kNaN;
            ++st.deshadowed;
        }
    }
}

// 一次跑完整条单帧清洗链。顺序有讲究，别乱调：
//   gating 最先——NaN/超量程会污染"邻居一致"比较；
//   despike 先于 demix——椒盐点本身会伪装成"桥接"；
//   deshadow 最后——它看的是两两相邻几何，输入越干净越准。
//
// why（可选）：逐束删除原因（0=保留，1=门控，2=椒盐，3=鬼点，
// 4=阴影）——可视化用："删了哪个点、为什么删"必须能画出来。
inline void clean_frame(PolarScan& s, FilterStats& st,
                        std::vector<std::uint8_t>* why = nullptr)
{
    if (why) why->assign(s.range.size(), 0);
    // 逐段标记"本段新出现的 NaN"归因于本段
    auto mark_stage = [&](std::uint8_t cat) {
        if (!why) return;
        for (int i = 0; i < s.size(); ++i) {
            if (!s.valid(i) && (*why)[static_cast<std::size_t>(i)] == 0) {
                (*why)[static_cast<std::size_t>(i)] = cat;
            }
        }
    };

    st.total_beams += s.size();
    long nan_before = 0;
    for (const auto& r : s.range) {
        if (!std::isfinite(r)) ++nan_before;
    }
    range_gate(s, GateParams{});
    mark_stage(1);  // 解析期 NaN 也归因于门控（无回波）
    long nan_after = 0;
    for (const auto& r : s.range) {
        if (!std::isfinite(r)) ++nan_after;
    }
    st.gated += nan_after - nan_before;
    despike(s, SpikeParams{}, st);
    mark_stage(2);
    demix(s, MixParams{}, st);
    mark_stage(3);
    deshadow(s, ShadowParams{}, st);
    mark_stage(4);
}

}  // namespace lpl
