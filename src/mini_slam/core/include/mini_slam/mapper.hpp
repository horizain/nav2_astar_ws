#pragma once
// ============================================================================
// Mapper：整个 SLAM 系统的编排器，对齐 slam_toolbox / libkarto 的
// MapperProcessScan 流程：
//
//   scan + odom → 位移阈值判定 → running scans 顺序匹配（修正里程计漂移）
//   → 节点/边入图 → 闭环候选搜索 → 宽窗两级阈值匹配 → 闭环边 → SPA 优化
// ============================================================================
#include <optional>
#include <vector>

#include "mini_slam/correlative_matcher.hpp"
#include "mini_slam/pose_graph.hpp"
#include "mini_slam/spa_solver.hpp"
#include "mini_slam/types.hpp"

namespace mini_slam {

class Mapper {
 public:
  // 运行模式：建图（默认）或定位（图冻结，只做位姿跟踪）
  // 对应 slam_toolbox 的 mapping / localization 两种模式
  enum class Mode { Mapping, Localization };

  struct NodeAdded {
    bool added{false};
    int id{-1};
    bool loop_closed{false};
  };

  // 定位模式单帧结果
  struct LocalizationResult {
    Pose2 pose;          // 图坐标系下的修正位姿（用于 map→odom TF）
    double response{0.0};
    bool valid{false};   // false = 匹配不可信，pose 已退回里程计先验
  };

  struct Stats {
    int scans{0};          // 收到的总帧数
    int nodes{0};          // 实际入图的节点数
    int loop_closures{0};
    int solves{0};
    SpaSummary last_solve;
  };

  explicit Mapper(Params params = {}) : params_(params) {}

  // 切换模式：实际变化时清空定位跟踪状态（避免 mapping↔localization 残留）。
  void set_mode(Mode m);
  Mode mode() const { return mode_; }

  // 运行时整体替换算法参数（调用方保证与 process_scan/localize 不并发；
  // ROS 节点在 mapper_mutex_ 下调用）。
  void set_params(const Params& p) { params_ = p; }

  // 运行时修改栅格分辨率（匹配场使用）。
  void set_resolution(double r) { params_.resolution = r; }

  // 定位初始化：给定 map 系位姿和对应 odom 位姿，后续先验从这里往前推。
  // 对应 /initialpose 或外部地图对齐。
  void initialize_localization(const Pose2& map_pose, const Pose2& odom_pose);

  // 建图模式：输入一帧扫描及其里程计位姿（激光与里程计原点重合已由上游保证）
  NodeAdded process_scan(const LaserScan& scan, const Pose2& odom_pose);

  // 定位模式：图冻结（不加节点/边、不触发 SPA），只做匹配跟踪。
  // 要求图非空（先建图或从序列化文件加载）。
  LocalizationResult localize(const LaserScan& scan, const Pose2& odom_pose);

  // 交互式编辑：把 id 节点移到 pose，固定该节点跑一次 SPA。
  // 成功返回 true；id 越界/非有限位姿返回 false。
  // 语义（对应 slam_toolbox 的 interactive SLAM 手动纠偏）：
  //   - 节点 0 与 id 同时固定：前者消规范自由度，后者保证用户放下的位置不被拉回；
  //   - stats_.solves 递增让地图渲染缓存键失效（地图重渲染）；
  //   - pose_revision 递增让 IncrementalMapBuilder 全量重建；
  //   - SPA 会重摆其它节点，因此必须 rebuild_index()。
  bool move_vertex(int id, Pose2 pose);

  // 位姿图持久化（对应 slam_toolbox 的 serialize/deserialize）
  bool save_graph(const std::string& path, std::string* err = nullptr) const;
  bool load_graph(const std::string& path, std::string* err = nullptr);

  const PoseGraph& graph() const { return graph_; }
  const Params& params() const { return params_; }
  const Stats& stats() const { return stats_; }

  // map→odom TF：T_map_odom = T_map(last) ∘ T_odom(last)⁻¹（最近入图节点，建图模式用）
  Pose2 map_odom_tf() const;

 private:
  // running scans：从最新节点往回收集，空间/数量双重窗口
  std::vector<int> running_ids(const Pose2& odom) const;

  // 用 ref_ids 各扫描（当前位姿）建得分场，再做 correlative 匹配
  MatchResult match_against(const std::vector<int>& ref_ids, const Pose2& prior,
                            std::span<const Vec2> points,
                            const MatchWindow& window) const;

  // 闭环：候选搜索 + 两级阈值匹配 + 加边；成功返回匹配结果
  std::optional<MatchResult> try_loop_close(int new_id);

  // 稀疏化（阶段6）：被新扫描覆盖的旧点云释放
  void sparsify();

  Params params_;
  PoseGraph graph_;
  SpaSolver solver_;
  Stats stats_;
  Mode mode_{Mode::Mapping};

  // 定位模式跟踪状态：最近一次修正位姿与其对应里程计位姿
  Pose2 last_loc_pose_{0.0, 0.0, 0.0};
  Pose2 last_loc_odom_{0.0, 0.0, 0.0};
  bool loc_initialized_{false};
};

}  // namespace mini_slam
