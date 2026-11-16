#include "mini_slam/mapper.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "mini_slam/detail/format.hpp"
#include "mini_slam/serialization.hpp"

namespace mini_slam {

void Mapper::set_mode(Mode m) {
  if (mode_ != m) loc_initialized_ = false;
  mode_ = m;
}

void Mapper::initialize_localization(const Pose2& map_pose,
                                     const Pose2& odom_pose) {
  last_loc_pose_ = map_pose;
  last_loc_odom_ = odom_pose;
  loc_initialized_ = true;
}

std::vector<int> Mapper::running_ids(const Pose2& odom) const {
  std::vector<int> ids;
  // 顺序匹配的参照集：时间上最近的 max_count 个节点，且位姿在空间窗口内。
  // 两个职责要分清：
  //   - 这个集合只服务于"顺序匹配的参照场"，老节点（上一圈经过的）混进来
  //     不仅无害，反而帮锚定（相当于对着局部地图做匹配）；
  //   - 闭环候选的排除靠 loop_min_node_gap（时间间隔），绝不能用这个集合
  //     去排除——否则绕圈场景下整段旧路径都会被划进 running，闭环永远
  //     凑不满候选链（血的教训）。
  for (int id = graph_.size() - 1; id >= 0; --id) {
    if (static_cast<int>(ids.size()) >= params_.running_scan_max_count) break;
    const Pose2& p = graph_.vertices()[static_cast<size_t>(id)].pose;
    if (std::hypot(p.x - odom.x, p.y - odom.y) <= params_.running_scan_distance) {
      ids.push_back(id);
    }
  }
  return ids;
}

MatchResult Mapper::match_against(const std::vector<int>& ref_ids,
                                  const Pose2& prior,
                                  std::span<const Vec2> points,
                                  const MatchWindow& window) const {
  if (ref_ids.empty()) return {};
  std::vector<FramedScan> frames;
  frames.reserve(ref_ids.size());
  for (const int id : ref_ids) {
    const Vertex& v = graph_.vertices()[static_cast<size_t>(id)];
    if (v.scan) frames.push_back(FramedScan{v.pose, v.scan->points});
  }
  if (frames.empty()) return {};

  // 场的 margin 需要覆盖匹配窗口 + 模糊半径的延伸
  const double margin = window.lin + 0.5;
  auto field = LikelihoodField::build(frames, params_.resolution, margin,
                                      params_.blur_radius);
  if (!field) return {};
  return CorrelativeMatcher::match(*field, prior, points, window);
}

std::optional<MatchResult> Mapper::try_loop_close(int new_id) {
  const Vertex& cur = graph_.vertices()[static_cast<size_t>(new_id)];

  // 1) 候选搜索（karto 式）：空间种子 → 最近种子的图连通链。
  //    链是"拓扑连贯的一段旧轨迹"，比空间邻居集合更真实；空间过滤 +
  //    时间间隔（防与刚走过的节点自匹配）在链上再做一遍。
  std::vector<int> seeds = graph_.vertices_within(
      Vec2{cur.pose.x, cur.pose.y}, params_.loop_search_distance, {new_id});
  std::erase_if(seeds, [&](int id) {
    return new_id - id < params_.loop_min_node_gap;
  });
  if (seeds.empty()) {
    if (params_.verbose) {
      io::println("  [loop] node={} 无空间种子", new_id);
    }
    return std::nullopt;
  }
  const int nearest = *std::min_element(
      seeds.begin(), seeds.end(), [&](int a, int b) {
        return distance_xy(graph_.vertices()[static_cast<size_t>(a)].pose,
                           cur.pose) <
               distance_xy(graph_.vertices()[static_cast<size_t>(b)].pose,
                           cur.pose);
      });
  std::vector<int> candidates = graph_.connected_chain(
      nearest, params_.loop_min_chain_size * 3, /*max_depth=*/6, {new_id});
  const double chain_radius = 2.0 * params_.loop_search_distance;
  std::erase_if(candidates, [&](int id) {
    if (new_id - id < params_.loop_min_node_gap) return true;
    const Pose2& p = graph_.vertices()[static_cast<size_t>(id)].pose;
    return distance_xy(p, cur.pose) > chain_radius;
  });
  if (static_cast<int>(candidates.size()) < params_.loop_min_chain_size) {
    if (params_.verbose) {
      io::println("  [loop] node={} 候选不足: {} < {}", new_id,
                  candidates.size(), params_.loop_min_chain_size);
    }
    return std::nullopt;
  }

  // 2) 宽窗匹配，两级阈值（对应 loop_match_minimum_response_coarse/fine）。
  //    归一化用总点数：稀释效应拦截假闭环（见 MatchWindow 注释）
  const MatchWindow window{params_.loop_window_lin, params_.loop_window_ang,
                           params_.coarse_lin_step, params_.coarse_ang_step,
                           /*in_bounds_only=*/false};
  const MatchResult m =
      match_against(candidates, cur.pose, cur.scan->points, window);
  if (params_.verbose) {
    io::println("  [loop] node={} cur=({:.2f},{:.2f},{:.2f}) 候选={} "
                "coarse={:.3f} fine={:.3f} var={:.2f} "
                "位移=({:+.2f},{:+.2f},{:+.3f})",
                new_id, cur.pose.x, cur.pose.y, cur.pose.theta,
                candidates.size(), m.coarse_response, m.response, m.variance,
                m.pose.x - cur.pose.x, m.pose.y - cur.pose.y,
                wrap_angle(m.pose.theta - cur.pose.theta));
    for (const int id : candidates) {
      const Pose2& p = graph_.vertices()[static_cast<size_t>(id)].pose;
      io::println("    cand id={} pose=({:.2f},{:.2f}) dist={:.2f}", id, p.x,
                  p.y, distance_xy(p, cur.pose));
    }
  }
  if (!m.valid || m.coarse_response < params_.loop_min_response_coarse ||
      m.response < params_.loop_min_response_fine ||
      m.variance > params_.loop_max_variance) {
    return std::nullopt;
  }

  // 3) 闭环成立：连到最近的 loop_max_edges 个候选，信息矩阵来自匹配协方差
  std::ranges::sort(candidates, std::less<double>{}, [&](int id) {
    const Pose2& p = graph_.vertices()[static_cast<size_t>(id)].pose;
    return distance_xy(p, m.pose);
  });
  const double sx = std::clamp(std::sqrt(std::max(m.covariance[0], 1e-6)),
                               params_.match_sigma_min, params_.match_sigma_max);
  const double sy = std::clamp(std::sqrt(std::max(m.covariance[4], 1e-6)),
                               params_.match_sigma_min, params_.match_sigma_max);
  double stt = std::sqrt(std::max(m.covariance[8], 1e-8));
  stt = std::clamp(stt, params_.match_sigma_min, params_.match_sigma_max);
  const int n_edges = std::min<int>(params_.loop_max_edges,
                                    static_cast<int>(candidates.size()));
  for (int i = 0; i < n_edges; ++i) {
    const Pose2& cand = graph_.vertices()[static_cast<size_t>(candidates[i])].pose;
    const Pose2 z = delta_pose(m.pose, cand);  // 相对观测 θ 取最短差
    graph_.add_edge(new_id, candidates[i], z,
                    1.0 / (sx * sx), 1.0 / (sy * sy), 1.0 / (stt * stt),
                    /*loop=*/true);
  }
  return m;
}

Mapper::NodeAdded Mapper::process_scan(const LaserScan& scan,
                                       const Pose2& odom_pose) {
  NodeAdded out;
  ++stats_.scans;

  // 裁剪超远点（glass/穿墙误读、超量程回波），匹配与建图都更稳
  LaserScan trimmed = scan;
  bool need_trim = false;
  for (const Vec2& p : scan.points) {
    if (norm(p) > params_.scan_range_max) {
      need_trim = true;
      break;
    }
  }
  if (need_trim) {
    std::erase_if(trimmed.points, [&](const Vec2& p) {
      return norm(p) > params_.scan_range_max;
    });
  }
  auto scan_ptr = std::make_shared<const LaserScan>(trimmed);

  // 首帧：无条件入图
  if (graph_.size() == 0) {
    out.added = true;
    out.id = graph_.add_vertex(odom_pose, odom_pose, scan_ptr);
    ++stats_.nodes;
    return out;
  }

  const int last_id = graph_.size() - 1;
  const Vertex& last = graph_.vertices()[static_cast<size_t>(last_id)];

  // ---------- 位移阈值判定（slam_toolbox 的 minimum_travel_*）----------
  // ROS odom 的 yaw 是 [-π,π]：3.10 → -3.10 的真实增量是 +0.083 而不是 -6.2。
  // delta_pose 只对"增量"取最短角差，图内位姿仍保持连续表示。
  const Pose2 delta_odom = delta_pose(last.odom_pose, odom_pose);
  if (norm(Vec2{delta_odom.x, delta_odom.y}) < params_.min_travel_distance &&
      std::abs(delta_odom.theta) < params_.min_travel_heading) {
    return out;  // 没走够：这帧不入图（与 slam_toolbox 一致）
  }

  // ---------- 顺序匹配：先验 = 最近修正位姿 ⊕ 里程计增量 ----------
  // 增量式先验让匹配窗口只需吸收"局部"误差；累积漂移交给闭环处理。
  // theta 保持与图内一致的连续表示（相对增量已在上面取最短差）。
  const Pose2 prior = compose(last.pose, delta_odom);
  const MatchWindow window{params_.match_window_lin, params_.match_window_ang,
                           params_.coarse_lin_step, params_.coarse_ang_step};
  const std::vector<int> running = running_ids(odom_pose);
  const MatchResult m = match_against(running, prior, trimmed.points, window);
  if (params_.verbose && !running.empty()) {
    io::println("  [match] frame refs={} resp={:.3f} coarse={:.3f} valid={}",
                running.size(), m.response, m.coarse_response, m.valid);
  }

  Pose2 corrected = prior;
  if (m.valid && m.response >= params_.match_min_response) {
    corrected = m.pose;
  }
  // 匹配不可信 → 退回纯里程计（karto 同样有这条 fallback 路径）

  // ---------- 节点与边入图 ----------
  out.added = true;
  out.id = graph_.add_vertex(odom_pose, corrected, scan_ptr);
  ++stats_.nodes;

  // 里程计边：观测 = 纯里程计增量
  graph_.add_edge(last_id, out.id, delta_odom,
                  1.0 / (params_.odom_sigma_xy * params_.odom_sigma_xy),
                  1.0 / (params_.odom_sigma_xy * params_.odom_sigma_xy),
                  1.0 / (params_.odom_sigma_theta * params_.odom_sigma_theta),
                  /*loop=*/false);

  // 链内边：与 running scans 的相对观测。
  // P3：信息矩阵来自匹配协方差（对角 + σ 夹紧），取代 response² 启发式——
  // 响应面平坦（滑移歧义）时 σ 自动变大、边自动变软。
  if (m.valid && m.response >= params_.match_min_response && !running.empty()) {
    const double sx = std::clamp(std::sqrt(std::max(m.covariance[0], 1e-6)),
                                 params_.match_sigma_min, params_.match_sigma_max);
    const double sy = std::clamp(std::sqrt(std::max(m.covariance[4], 1e-6)),
                                 params_.match_sigma_min, params_.match_sigma_max);
    double stt = std::sqrt(std::max(m.covariance[8], 1e-8));
    stt = std::clamp(stt, params_.match_sigma_min, params_.match_sigma_max);
    for (const int ref : running) {
      const Pose2 ref_pose = graph_.vertices()[static_cast<size_t>(ref)].pose;
      const Pose2 z = delta_pose(corrected, ref_pose);  // 相对观测 θ 取最短差
      graph_.add_edge(out.id, ref, z,
                      1.0 / (sx * sx), 1.0 / (sy * sy), 1.0 / (stt * stt),
                      /*loop=*/false);
    }
  }

  // ---------- 闭环 ----------
  // 宽窗匹配 + SPA 都不便宜，走廊重访类数据集会连续满足候选条件，
  // 按间隔节流（slam_toolbox 里对应 loop_closure.throttle_scans 的角色）
  if (params_.loop_attempt_interval > 0 &&
      stats_.nodes % params_.loop_attempt_interval == 0) {
    if (auto loop = try_loop_close(out.id)) {
      out.loop_closed = true;
      ++stats_.loop_closures;
      stats_.last_solve = solver_.solve(graph_);
      ++stats_.solves;
      // SPA 重摆了历史位姿：增量地图渲染必须全量重建。
      graph_.bump_pose_revision();
      graph_.rebuild_index();  // 位姿更新后空间索引过期，重建
    }
  }

  // ---------- 稀疏化（阶段6）：被新扫描覆盖的旧点云定期释放 ----------
  if (params_.sparsify_enabled && params_.sparsify_every > 0 &&
      stats_.nodes % params_.sparsify_every == 0) {
    sparsify();
  }

  return out;
}

void Mapper::sparsify() {
  const int size = graph_.size();
  const int recent_from =
      std::max(0, size - params_.sparsify_min_keep_recent);
  long freed = 0;
  // 贪心铺点：从最老到保底窗口，与"上一个保留点"距离 ≥ keep_radius 才保留
  // ——保留点永不释放，被覆盖的中间扫描释放后仍被保留点等价覆盖（可靠性）
  Pose2 last_kept{0.0, 0.0, 0.0};
  bool has_kept = false;
  for (int id = 0; id < recent_from; ++id) {
    const Vertex& v = graph_.vertices()[static_cast<size_t>(id)];
    const bool has_pts = v.scan && !v.scan->points.empty();
    const bool far = !has_kept ||
                     distance_xy(v.pose, last_kept) >=
                         params_.sparsify_keep_radius;
    if (has_pts && !far) {
      LaserScan shell;
      shell.stamp = v.scan->stamp;
      graph_.set_scan(id, std::make_shared<const LaserScan>(std::move(shell)));
      ++freed;
    } else if (has_pts && far) {
      last_kept = v.pose;
      has_kept = true;
    }
  }
  if (params_.verbose && freed > 0) {
    io::println("  [sparsify] 释放 {} 帧旧点云", freed);
  }
}

// ---------------------------------------------------------------------------
// 定位模式：图冻结的单帧位姿跟踪（对应 slam_toolbox 的 localization 模式）
// 流程：先验（上次修正位姿 ⊕ 里程计增量）→ 定位窗口内建场 → 宽窗匹配
//       → 不可信则退回先验。全程不改图。
// ---------------------------------------------------------------------------
Mapper::LocalizationResult Mapper::localize(const LaserScan& scan,
                                            const Pose2& odom_pose) {
  LocalizationResult out;
  ++stats_.scans;

  // 图为空：定位无从谈起（先建图或加载序列化图）
  if (graph_.size() == 0) {
    out.pose = odom_pose;
    last_loc_pose_ = odom_pose;
    last_loc_odom_ = odom_pose;
    loc_initialized_ = true;
    return out;
  }

  // ---------- 先验：上次修正位姿 ⊕ 里程计增量 ----------
  Pose2 prior;
  if (!loc_initialized_) {
    // 首帧定位：不知道图与当前里程计的对应关系，直接信任里程计
    // （调用方应保证机器人初始位置与建图起点一致，或用外界给出粗略初值）
    prior = odom_pose;
    loc_initialized_ = true;
  } else {
    // 里程计增量同样跨 ±π 取最短差，再与上次修正位姿复合。
    prior = compose(last_loc_pose_, delta_pose(last_loc_odom_, odom_pose));
    prior.theta = wrap_angle(prior.theta);  // 跟踪状态保持在 [-π,π)
  }

  // ---------- 定位窗口内建场并匹配 ----------
  std::vector<int> refs =
      graph_.vertices_within(Vec2{prior.x, prior.y},
                             params_.localization_window);
  MatchResult m;
  if (!refs.empty()) {
    // 定位窗口比顺序匹配宽（允许吸收重启后的位姿差），但仍远小于全图
    const MatchWindow window{params_.match_window_lin * 2.0,
                             params_.match_window_ang * 1.5,
                             params_.coarse_lin_step,
                             params_.coarse_ang_step};
    m = match_against(refs, prior, scan.points, window);
  }

  Pose2 corrected = prior;
  if (m.valid && m.response >= params_.loc_min_response) {
    corrected = m.pose;
    out.response = m.response;
    out.valid = true;
  }
  // 匹配不可信 → 退回先验（valid=false，由调用方决定是否报警）

  // ---------- 更新跟踪状态（图保持冻结） ----------
  last_loc_pose_ = corrected;
  last_loc_odom_ = odom_pose;
  out.pose = corrected;
  return out;
}

// ---------------------------------------------------------------------------
// 交互式编辑：手动摆放节点 + 以该节点为固定点重优化
// 对应 slam_toolbox 的 interactive SLAM（RViz 里拖动图节点后局部重建地图）。
// 与 process_scan 的闭环 SPA 的区别只在"谁被固定"：这里被拖节点是硬约束，
// 其它节点按边观测让步；因此用户放下的位置一定保留。
// ---------------------------------------------------------------------------
bool Mapper::move_vertex(int id, Pose2 pose) {
  if (id < 0 || id >= graph_.size()) return false;
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.theta)) {
    return false;
  }

  // 对外位姿统一成 (-π, π] 表示（与 TF/显示一致）；图内其它位姿不动。
  pose.theta = wrap_angle(pose.theta);
  graph_.pose(id) = pose;

  // 手动改位姿 = 一次重摆：增量渲染必须全量重建（与 SPA/稀疏化同路径）。
  graph_.bump_pose_revision();
  stats_.last_solve = solver_.solve(graph_, id);
  ++stats_.solves;  // MapRenderKey 含 solves：地图缓存随之失效
  graph_.rebuild_index();  // 位姿更新后空间索引过期，闭环/定位查询要重建
  return true;
}

bool Mapper::save_graph(const std::string& path, std::string* err) const {
  auto r = serialization::save_graph(graph_, path);
  if (!r) {
    if (err) *err = r.error();
    return false;
  }
  return true;
}

bool Mapper::load_graph(const std::string& path, std::string* err) {
  auto r = serialization::load_graph(path);
  if (!r) {
    if (err) *err = r.error();
    return false;
  }
  graph_ = std::move(r.value());
  // 换图强失效：即使顶点/边数恰好相同，也必须让增量渲染器全量重建。
  graph_.bump_pose_revision();
  graph_.rebuild_index();
  stats_ = {};              // 新会话从零计数
  loc_initialized_ = false; // 定位跟踪状态随之失效
  return true;
}

Pose2 Mapper::map_odom_tf() const {
  if (graph_.size() == 0) return {};
  const Vertex& last = graph_.vertices().back();
  Pose2 t = compose(last.pose, inverse(last.odom_pose));
  t.theta = wrap_angle(t.theta);  // TF 输出统一表示
  return t;
}

}  // namespace mini_slam
