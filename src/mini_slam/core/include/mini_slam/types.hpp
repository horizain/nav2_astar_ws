#pragma once
// ============================================================================
// 基础类型：Vec2 / Pose2 (SE(2)) / LaserScan / Params
// 对齐 slam_toolbox（libkarto）的概念命名，方便对照阅读源码。
// ============================================================================
#include <cmath>
#include <span>
#include <vector>

namespace mini_slam {

constexpr double kPi = 3.14159265358979323846;

// ---------------- 二维向量 ----------------
struct Vec2 {
  double x{0.0}, y{0.0};
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(double s, Vec2 a) { return {s * a.x, s * a.y}; }
constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double norm(Vec2 a) { return std::hypot(a.x, a.y); }

// ---------------- SE(2) 位姿 (x, y, θ) ----------------
// 3 自由度位姿：图优化的节点、边观测、里程计都用它。
// 注：compose/inverse 只做刚体复合，不对 theta 做归一化——图优化内部需要
//     连续的角度表示，粗暴地逐位姿 wrap 会在 ±π 处制造伪跳变。
//     但"相对角"（两角之差）必须取最短差：用 shortest_angle_diff（对
//     double 与 ceres::Jet 都可用），不要直接写 theta_a - theta_b。
//     需要 (-π, π] 的对外表示（TF/显示）时用 wrap_angle。
struct Pose2 {
  double x{0.0}, y{0.0}, theta{0.0};
};

inline double wrap_angle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

// 最短角差：返回 (a - b) 归一化到 (-π, π]。
// 模板版本同时服务 double（SPA/建图/定位）与 ceres::Jet（自动求导残差），
// 用 atan2(sin, cos) 实现，避免 Jet 不支持 fmod 的问题。unqualified 调用
// 依赖 ADL 找到 ceres::sin/cos/atan2；double 时回落到 std::。
template <typename T>
T shortest_angle_diff(const T& a, const T& b) {
  using std::atan2;
  using std::cos;
  using std::sin;
  return atan2(sin(a - b), cos(a - b));
}

// 复合 a∘b：b 在 a 的坐标系下作用到世界
constexpr Pose2 compose(Pose2 a, Pose2 b) {
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.theta + b.theta};
}

// 逆变换 a⁻¹
constexpr Pose2 inverse(Pose2 a) {
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  return {-(c * a.x + s * a.y), s * a.x - c * a.y, -a.theta};
}

// 相对位姿 b 在 a 坐标系下的表示：a⁻¹ ∘ b。
// 与 compose 的区别：这里的结果是"增量/观测"，theta 必须取最短角差。
// 用于里程计增量、定位先验和建图位移阈值判定。
inline Pose2 delta_pose(Pose2 from, Pose2 to) {
  Pose2 d = compose(inverse(from), to);
  d.theta = wrap_angle(d.theta);
  return d;
}

constexpr Vec2 rotate(Pose2 a, Vec2 p) {
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  return {c * p.x - s * p.y, s * p.x + c * p.y};
}

// p（a 坐标系下）→ 世界系
constexpr Vec2 transform(Pose2 a, Vec2 p) {
  const Vec2 r = rotate(a, p);
  return {a.x + r.x, a.y + r.y};
}

inline double distance_xy(Pose2 a, Pose2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// ---------------- 一帧激光 ----------------
// 已过滤的端点（invalid / 超量程已剔除）。
// 坐标约定：点已变换到 base_frame，与里程计位姿属于同一刚体（base）；
// 不再假设激光与里程计原点重合。ROS 节点负责按 TF 做外参变换。
struct LaserScan {
  double stamp{0.0};
  std::vector<Vec2> points;
};

// ---------------- 全参数表 ----------------
// 默认值对齐 slam_toolbox 的默认配置（minimum_travel_* / loop_closure 块）。
struct Params {
  double resolution{0.05};  // 栅格分辨率 (m)

  // --- 节点插入（对应 minimum_travel_distance / minimum_travel_heading）---
  double min_travel_distance{0.5};  // 平移超过才入图 (m)
  double min_travel_heading{0.5};   // 转角超过才入图 (rad)

  // --- running scans：顺序匹配的参照链（karto 的 running scans 概念）---
  // 链越长，锚定结构越多：长走廊里航向弱可观测，短链（几帧）压不住
  // 里程计航向偏差，走廊会被画成弧。karto 同款做法是拉长参照链。
  double running_scan_distance{10.0};  // 参照链空间窗口 (m)
  int running_scan_max_count{120};     // 参照链最大节点数

  // --- correlative 匹配窗口（Olson 2009 两级搜索）---
  // 顺序匹配是"fine-tuning"：窗口只需罩住单帧里程计误差（karto 同思路）。
  // 窗口开大了，长直墙沿墙方向的滑移歧义会让节点位姿随机游走。
  double match_window_lin{0.15};  // 顺序匹配平移窗口 ±(m)
  double match_window_ang{0.2};   // 顺序匹配角度窗口 ±(rad)
  // 闭环窗口必须罩得住"重访时图与真实位置的全部偏差"——libkarto 的
  // LoopSearchSpaceDimensions 默认 7.0m（±3.5m）就是这个道理；
  // 防假闭环靠响应阈值（0.4/0.5 + 总点数归一），不靠窄窗。
  double loop_window_lin{3.5};    // 闭环匹配平移窗口 ±(m)
  double loop_window_ang{0.6};    // 闭环匹配角度窗口 ±(rad)
  double coarse_lin_step{0.15};   // 粗搜索平移步长 (m)——宽窗下控制成本
  double coarse_ang_step{0.05};   // 粗搜索角度步长 (rad)

  double match_min_response{0.10};  // 顺序匹配得分低于此值 → 退回里程计

  // --- 闭环检测（对应 loop_closure 参数块）---
  double loop_search_distance{3.0};       // 闭环候选空间半径 (m)（karto 默认）
  int loop_min_chain_size{10};            // 候选链最小帧数
  int loop_min_node_gap{20};              // 候选与新节点的时间间隔（防相邻自匹配）
  double loop_min_response_coarse{0.40};  // 粗匹配得分阈值（总点数归一）
  double loop_min_response_fine{0.50};    // 细匹配得分阈值——宁缺毋滥
  double loop_max_variance{10.0};         // 响应面方差上限 (m²)——karto
                                          // LoopMatchMaximumVarianceCoarse 的
                                          // 思路：峰不尖锐（滑移歧义）的匹配
                                          // 直接拒绝。数值按本实现的方差
                                          // 公式标定（对称场景真闭环 ~4）
  int loop_max_edges{5};                  // 闭环成功时连接的最近候选数
  int loop_attempt_interval{3};           // 每入图 N 个节点尝试一次闭环（1=每帧试）

  // --- 得分场 ---
  int blur_radius{4};  // box blur 半径（格），两遍 ≈ 高斯 σ≈0.16m

  // --- 边的信息矩阵（对角元 1/σ²）---
  // 刚柔配比是图 SLAM 调参的核心：里程计边太硬 → 闭环掰不动轨迹，地图
  // 基本等于原始里程计；闭环边太硬 → 一旦假闭环就毁图。
  // 经验：里程计按"每步 0.5m 的漂移"给 σ；闭环按匹配精度给 σ（还要乘
  // response² 自适应降权）。
  // 里程计边 σ：按"每步(0.5m)的里程计漂移"给。轮式里程计正常时 0.1 够用；
  // slam_toolbox 明确要求 reasonably good odometry——Intel PowerBot 那种
  // 漂 60m 的输入本身就超出单轨迹图 SLAM 的适用范围。
  double odom_sigma_xy{0.10}, odom_sigma_theta{0.10};
  // 匹配/闭环边的信息矩阵来自匹配协方差（阶段3）：
  // σ 夹紧区间——太小（过度自信）会把图焊死，太大（几乎无信息）失去约束
  double match_sigma_min{0.02}, match_sigma_max{0.50};

  // --- 扫描裁剪与地图渲染 ---
  double scan_range_max{12.0};  // 匹配/入图用的最远点（karto RangeThreshold 同角色）
  double map_range_max{10.0};
  double map_margin{1.0};

  // --- 定位模式（对应 slam_toolbox 的 localization 模式）---
  double localization_window{10.0};  // 定位参照场半径 (m)：该范围内的建图节点
                                     // 参与匹配场构建
  double loc_min_response{0.15};     // 定位匹配得分低于此值 → 退回里程计先验

  // --- 地图稀疏化（对应 slam_toolbox 的地图生命周期管理）---
  // 长时运行内存控制：图结构（位姿+边）永久保留，但"被新扫描覆盖"的旧节点
  // 剪枝释放点云——地图渲染与匹配场由存活扫描等价覆盖。
  bool sparsify_enabled{false};      // 默认关（行为不变），长时运行打开
  double sparsify_keep_radius{0.8};  // 0.8m 内有更新扫描 → 旧点云可释放
  int sparsify_min_keep_recent{50};  // 最近 N 个节点的扫描永不剪枝
  int sparsify_every{50};            // 每入图 N 个节点跑一次剪枝

  bool verbose{false};  // 打印每帧匹配/闭环的内部数值（调试用）
};

}  // namespace mini_slam
