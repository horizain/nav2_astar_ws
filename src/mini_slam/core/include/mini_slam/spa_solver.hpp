#pragma once
// ============================================================================
// SPA（Sparse Pose Adjustment，Konolige 等 IROS 2010）——slam_toolbox 的
// 后端（solvers/spa_solver.cpp）就是用 Ceres 实现的这个。
//
// 目标函数：min Σ ‖ z_ij⁻¹ ∘ (x_i⁻¹ ∘ x_j) ‖²_Λ_ij
//   x_i, x_j : 节点 SE(2) 位姿（3 自由度参数块）
//   z_ij     : 边观测的相对位姿
//   Λ_ij     : 信息矩阵（本实现用对角近似）
// 残差/雅可比交给 Ceres autodiff（Jet），图稀疏 → SPARSE_NORMAL_CHOLESKY。
// 首节点 SetParameterBlockConstant 消除规范自由度（整体平移/旋转不定解）。
// ============================================================================
namespace mini_slam {

class PoseGraph;

struct SpaSummary {
  double initial_cost{0.0};
  double final_cost{0.0};
  int iterations{0};
};

class SpaSolver {
 public:
  // fixed_id：额外固定一个节点（交互式拖拽时传被拖节点的 id）。
  // 节点 0 始终固定（消除整体平移/旋转的规范自由度）；fixed_id 非法
  // （<0 / >= size）或等于 0 时忽略——现有调用不传第二参数，行为不变。
  SpaSummary solve(PoseGraph& graph, int fixed_id = 0) const;
};

}  // namespace mini_slam
