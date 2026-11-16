#pragma once
// ============================================================================
// 位姿图（Pose Graph）：图优化 SLAM 的数据骨架。
//   节点 = 一帧激光的位姿；边 = 相对位姿观测 z（里程计 / 扫描匹配 / 闭环）。
// 附带一个空间桶哈希索引，闭环候选查询 vertices_within() 是 O(邻桶数)。
// ============================================================================
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "mini_slam/types.hpp"

namespace mini_slam {

struct Vertex {
  int id{-1};
  Pose2 pose;       // 当前估计（会被 SPA 更新）
  Pose2 odom_pose;  // 原始里程计位姿（不变），触发判定与 map→odom TF 用
  std::shared_ptr<const LaserScan> scan;
};

struct Edge {
  int from{-1}, to{-1};
  Pose2 z;        // 观测：T_from⁻¹ ∘ T_to
  // 信息矩阵对角元（1/σ²）。info_x/info_y 是**世界（map）坐标系轴向**的
  // 平移信息，不是 from 节点局部轴；SPA 的世界系平移误差为
  //   e_xy = R(θ_from) · (Δt_ab - z_xy)，Δt_ab = R(θ_from)ᵀ (t_to - t_from)
  // 即残差会先按 from 当前角把平移误差旋转到世界系，再用这里的轴向信息加权。
  double info_x{1.0};
  double info_y{1.0};
  double info_theta{1.0};
  bool loop{false};        // 闭环边标记
};

class PoseGraph {
 public:
  // 返回新节点 id（== 追加下标）
  int add_vertex(Pose2 odom_pose, Pose2 pose,
                 std::shared_ptr<const LaserScan> scan = nullptr);
  void add_edge(int from, int to, Pose2 z, double info_x, double info_y,
                double info_theta, bool loop);

  int size() const { return static_cast<int>(vertices_.size()); }
  const std::vector<Vertex>& vertices() const { return vertices_; }
  const std::vector<Edge>& edges() const { return edges_; }
  Pose2& pose(int id) { return vertices_[static_cast<size_t>(id)].pose; }
  // 稀疏化用：替换节点扫描（释放旧点云、保留位姿）
  // 扫描内容变化会改变地图渲染结果，因此自动递增 pose_revision()。
  void set_scan(int id, std::shared_ptr<const LaserScan> scan) {
    vertices_[static_cast<size_t>(id)].scan = std::move(scan);
    bump_pose_revision();
  }

  // 地图内容版本：增量渲染只用它判断"历史扫描/位姿是否被改动"。
  //   - add_vertex/add_edge 不递增（新增顶点由 size() 增量发现）；
  //   - set_scan 自动递增（内容变了）；
  //   - SPA 重摆位姿、手动改 pose、load_graph 换图后由调用方递增。
  std::uint64_t pose_revision() const { return pose_revision_; }
  void bump_pose_revision() { ++pose_revision_; }

  // 半径查询：返回中心 radius 内的节点 id（排除 exclude 列表）
  std::vector<int> vertices_within(Vec2 center, double radius,
                                   const std::vector<int>& exclude = {}) const;

  // 图连通链（karto 式闭环候选）：从 seed 沿边 BFS，收集拓扑连通的节点。
  //   max_nodes  链的最大节点数
  //   max_depth  BFS 最大跳数（闭环边也算一跳——它正是连通远处的关键）
  //   exclude    不进入链的节点 id（如刚走过的节点、当前节点）
  // 返回的链是"一段拓扑连贯的轨迹"，而非散乱的空间邻居集合。
  std::vector<int> connected_chain(int seed, int max_nodes, int max_depth,
                                   const std::vector<int>& exclude) const;

  // SPA 之后位姿变了，空间索引按最新位姿重建
  void rebuild_index();

 private:
  static constexpr double kBucket = 1.0;  // 空间桶边长 (m)
  static std::uint64_t bucket_key(int bx, int by) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bx)) << 32) |
           static_cast<std::uint32_t>(by);
  }
  void index_vertex(int id);

  std::vector<Vertex> vertices_;
  std::vector<Edge> edges_;
  std::uint64_t pose_revision_{0};
  std::unordered_map<std::uint64_t, std::vector<int>> buckets_;
  std::vector<std::vector<int>> adj_;   // 邻接表（双向），连通链 BFS 用
};

}  // namespace mini_slam
