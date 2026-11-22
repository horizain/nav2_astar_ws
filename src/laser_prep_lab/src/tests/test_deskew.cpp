// ============================================================================
// deskew 的定量验证：每束点与其"真值位置"（扫描起始帧下）的偏差。
// 平移与旋转两个病灶各造一例，未校正误差是可预测的运动量级，
// 校正后误差必须压到数值噪声级。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "deskew.hpp"

using namespace lpl;

static int failures = 0;
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::printf("FAIL %s (line %d)\n", msg, __LINE__);    \
            ++failures;                                           \
        }                                                         \
    } while (0)

// 圆形围栏扫描：机器人在 (0,0) 朝向 0，半径 R 的圆墙上取点。
// 用来同时喂平移/旋转病灶：真值点位置只依赖"机器人该束时刻的位姿"。
static PolarScan fence_scan(int beams, double R)
{
    PolarScan s;
    s.stamp = 100.0;
    s.beam_dt = 0.001;  // 100 束 → 扫描历时 0.1s
    s.angle_min = -kPi / 2.0;
    s.angle_inc = kPi / (beams - 1);
    for (int i = 0; i < beams; ++i) {
        s.range.push_back(R);
    }
    return s;
}

// 一帧扫描期间机器人恒速平移 (vx, vy) + 恒转速 ω 的里程计流。
// 位姿解析式（t 从 stamp 起）：p(t) = p0 ∘ (v·τ, ω·τ)
// 采样间隔 0.02s，覆盖扫描前后。
static OdomBuffer moving_odom(double stamp, icp::Pose2 p0, double vx,
                              double vy, double omega, double span)
{
    OdomBuffer ob;
    for (double t = stamp - span; t <= stamp + span + 1e-9; t += 0.02) {
        const double tau = t - stamp;
        ob.push(t, compose(p0, icp::Pose2{vx * tau, vy * tau, omega * tau}));
    }
    return ob;
}

// 未 deskew 的点（当成机器人在 p0 不动）：endpoints()
// deskew 后的点：deskew()
// 真值：世界点 = R(beam) 在圆墙上 → 拉回 p0 帧。
// 用"圆墙约束"检验：世界系里所有真值点距原点 = R（p0 在原点时）。
int main()
{
    const int beams = 100;
    const double R = 5.0;
    const icp::Pose2 p0{0, 0, 0};

    // ---------- 病灶 A：平移 1m/s ----------
    {
        PolarScan s = fence_scan(beams, R);
        OdomBuffer ob = moving_odom(s.stamp, p0, 1.0, 0.0, 0.0, 0.3);
        const DeskewResult d = deskew(s, ob);
        CHECK(d.corrected, "有 odom + beam_dt → 执行校正");
        CHECK(d.raw_skew_m > 0.09, "0.1s × 1m/s ≈ 0.1m 的原始畸变可观测");

        // 未校正：把每束当成 p0 时刻拍的 → 点在 p0 系里仍距原点 R？
        // 平移畸变不改变"点到原点距离"……但会改变点的实际位置。
        // 用真值对照：真值 p0 系坐标 = p(t_i)^{-1} ∘ p0 ∘ beam_t
        // 直接验证 deskew 输出 == 真值。
        double max_err = 0.0, max_raw_err = 0.0;
        const auto raw = s.endpoints();
        int k = 0;
        for (int i = 0; i < beams; ++i) {
            // 真值：beam 点在世界系 = p(t_i) ∘ (R·u(a_i))；
            // p0 系真值 = p(t_i)^{-1}... 直接算：
            const double tau = s.beam_dt * i;
            const icp::Pose2 pt = compose(p0, icp::Pose2{1.0 * tau, 0.0, 0.0});
            const double a = s.angle(i);
            const icp::Point2 beam{R * std::cos(a), R * std::sin(a)};
            // 世界真值
            const icp::Point2 world = transform(pt, beam);
            // p0 系真值
            const icp::Point2 truth = transform(inverse(p0), world);
            // deskew 输出的第 k 个有效点对应束 i（全部有效）
            const icp::Point2& got = d.points[static_cast<std::size_t>(k)];
            max_err = std::max(max_err, std::hypot(got.x - truth.x, got.y - truth.y));
            // 未校正（当作 p0 时刻拍的）
            const icp::Point2& raw_pt = raw[static_cast<std::size_t>(k)];
            max_raw_err = std::max(
                max_raw_err,
                std::hypot(raw_pt.x - truth.x, raw_pt.y - truth.y));
            ++k;
        }
        std::printf("  平移 1m/s: deskew 最大误差 %.5fm, 未校正最大误差 %.3fm\n",
                    max_err, max_raw_err);
        CHECK(max_err < 1e-6, "deskew 输出 = 解析真值（数值噪声级）");
        CHECK(max_raw_err > 0.05, "未校正误差可观测（>0.05m）");
    }

    // ---------- 病灶 B：原地旋转 1rad/s ----------
    {
        PolarScan s = fence_scan(beams, R);
        OdomBuffer ob = moving_odom(s.stamp, p0, 0.0, 0.0, 1.0, 0.3);
        const DeskewResult d = deskew(s, ob);
        double max_err = 0.0, max_raw_err = 0.0;
        const auto raw = s.endpoints();
        int k = 0;
        for (int i = 0; i < beams; ++i) {
            const double tau = s.beam_dt * i;
            const icp::Pose2 pt = compose(p0, icp::Pose2{0.0, 0.0, 1.0 * tau});
            const double a = s.angle(i);
            const icp::Point2 world = transform(pt, {R * std::cos(a), R * std::sin(a)});
            const icp::Point2 truth = transform(inverse(p0), world);
            const icp::Point2& got = d.points[static_cast<std::size_t>(k)];
            max_err = std::max(max_err, std::hypot(got.x - truth.x, got.y - truth.y));
            const icp::Point2& raw_pt = raw[static_cast<std::size_t>(k)];
            max_raw_err = std::max(
                max_raw_err,
                std::hypot(raw_pt.x - truth.x, raw_pt.y - truth.y));
            ++k;
        }
        std::printf("  旋转 1rad/s: deskew 最大误差 %.5fm, 未校正最大误差 %.3fm\n",
                    max_err, max_raw_err);
        // 旋转半径 5m、历时 0.1rad：未校正点弧向错位最大 ~5×0.1=0.5m
        CHECK(max_err < 1e-6, "旋转 deskew 输出 = 解析真值");
        CHECK(max_raw_err > 0.3, "旋转未校正误差 >0.3m");
    }

    // ---------- 诚实回退：无 odom / 无 beam_dt ----------
    {
        PolarScan s = fence_scan(beams, R);
        s.beam_dt = 0.0;  // 没有束时序 → 不许装没动，标记未校正
        const DeskewResult d = deskew(s, OdomBuffer{});
        CHECK(!d.corrected, "无时序 → corrected=false");
        CHECK(d.points.size() == static_cast<std::size_t>(beams),
              "回退输出 = 普通端点转换");
    }

    if (failures == 0) std::printf("test_deskew: OK\n");
    return failures == 0 ? 0 : 1;
}
