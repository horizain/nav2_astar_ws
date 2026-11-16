#include "mini_slam/spa_solver.hpp"

#include <cmath>

#include <ceres/ceres.h>

#include "mini_slam/pose_graph.hpp"

namespace mini_slam {
namespace {

// SE(2) 位姿图边的残差，3 维：
//   r_ab = R(a.θ)ᵀ (t_b - t_a)            // b 在 a 系下的平移
//   e_xy = R(a.θ) · (r_ab - z_xy)         // 世界系平移误差
//   e_θ  = shortest(r_θ - z_θ)            // 相对角误差
// 平移之所以先减 z_xy、再按 a.θ 旋到世界系，是因为 Edge::info_x/info_y 是
// **世界（map）坐标系轴向**信息。注意不要先做 z⁻¹ 的旋转再按 a.θ 旋转：
// 那等价于按 a.θ + z.θ 旋转，z.θ ≠ 0 时信息加权方向就错了。
// 参数块是两个 3-double 数组（Pose2 平凡布局）。
// cos/sin 不加 std:: 前缀：T=ceres::Jet 时靠 ADL 找到 ceres::cos，
// T=double 时落到全局 ::cos —— 一份代码同时服务求值与求导。
struct Se2EdgeCost {
  double zx, zy, zt;          // 边观测
  double sx, sy, st;          // sqrt 信息（对角）
  bool loop{false};           // 仅作标记，便于调试

  template <typename T>
  bool operator()(const T* const a, const T* const b, T* r) const {
    // a⁻¹ ∘ b 的平移：(rx, ry) 在 from 节点坐标系下
    const T ca = cos(a[2]), sa = sin(a[2]);
    const T rx = ca * (b[0] - a[0]) + sa * (b[1] - a[1]);
    const T ry = -sa * (b[0] - a[0]) + ca * (b[1] - a[1]);
    const T rt = b[2] - a[2];
    // from 系下的原始平移误差（不预先做 z⁻¹ 旋转）
    const T ex = rx - zx;
    const T ey = ry - zy;
    // 相对角残差取最短差：ros yaw 跨 ±π 时 rt-zt 会差 ±2π，直接优化会把
    // 整段轨迹拉坏。atan2(sin,cos) 对 ceres::Jet 也可自动求导。
    const T et = shortest_angle_diff(rt, T(zt));
    // info_x/info_y 是世界系轴向信息，先用 from 当前角把平移误差旋转到
    // 世界系再加权。否则长墙的退化方向（世界轴）会被错误地套到节点局部轴上。
    const T wa = a[2];
    const T wx = cos(wa) * ex - sin(wa) * ey;
    const T wy = sin(wa) * ex + cos(wa) * ey;
    r[0] = T(sx) * wx;
    r[1] = T(sy) * wy;
    r[2] = T(st) * et;
    return true;
  }
};

}  // namespace

SpaSummary SpaSolver::solve(PoseGraph& graph, int fixed_id) const {
  SpaSummary out;
  if (graph.size() == 0) return out;

  ceres::Problem problem;

  // 参数块：每个节点一个 3-double 块（Pose2 平凡可复制，内存连续）
  for (int i = 0; i < graph.size(); ++i) {
    problem.AddParameterBlock(&graph.pose(i).x, 3);
  }
  // 固定首节点：位姿图有整体平移/旋转的规范自由度，不固定则解不定
  problem.SetParameterBlockConstant(&graph.pose(0).x);
  // 交互式拖拽：被拖节点也必须固定，否则优化会把它拉回观测一致的位置，
  // 用户在 RViz 里放下的位姿就白费了。固定后残差压力由邻居节点承担
  // （这正是"手动纠正 + 局部重优化"的语义）。
  if (fixed_id > 0 && fixed_id < graph.size()) {
    problem.SetParameterBlockConstant(&graph.pose(fixed_id).x);
  }

  // 边 → autodiff 残差块
  for (const auto& e : graph.edges()) {
    const double sx = std::sqrt(e.info_x);
    const double sy = std::sqrt(e.info_y);
    const double st = std::sqrt(e.info_theta);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Se2EdgeCost, 3, 3, 3>(
            new Se2EdgeCost{e.z.x, e.z.y, e.z.theta, sx, sy, st, e.loop}),
        nullptr,  // 不加鲁棒核（karto 同样默认不用）
        &graph.pose(e.from).x, &graph.pose(e.to).x);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 50;
  options.function_tolerance = 1e-9;   // 默认 1e-6 会提前收手，闭环折不到位
  options.gradient_tolerance = 1e-10;
  options.num_threads = 1;  // 保持结果确定性
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  out.initial_cost = summary.initial_cost;
  out.final_cost = summary.final_cost;
  out.iterations = static_cast<int>(summary.iterations.size());
  return out;
}

}  // namespace mini_slam
