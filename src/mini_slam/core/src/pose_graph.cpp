#include "mini_slam/pose_graph.hpp"

#include <algorithm>
#include <cmath>

namespace mini_slam {

void PoseGraph::index_vertex(int id) {
  const Pose2& p = vertices_[static_cast<size_t>(id)].pose;
  const int bx = static_cast<int>(std::floor(p.x / kBucket));
  const int by = static_cast<int>(std::floor(p.y / kBucket));
  buckets_[bucket_key(bx, by)].push_back(id);
}

int PoseGraph::add_vertex(Pose2 odom_pose, Pose2 pose,
                          std::shared_ptr<const LaserScan> scan) {
  Vertex v;
  v.id = size();
  v.odom_pose = odom_pose;
  v.pose = pose;
  v.scan = std::move(scan);
  vertices_.push_back(std::move(v));
  index_vertex(v.id);
  return v.id;
}

void PoseGraph::add_edge(int from, int to, Pose2 z, double info_x,
                         double info_y, double info_theta, bool loop) {
  edges_.push_back(Edge{from, to, z, info_x, info_y, info_theta, loop});
  // 邻接表同步维护（连通链 BFS 用）
  if (static_cast<int>(adj_.size()) < size()) adj_.resize(size());
  adj_[static_cast<size_t>(from)].push_back(to);
  adj_[static_cast<size_t>(to)].push_back(from);
}

std::vector<int> PoseGraph::connected_chain(int seed, int max_nodes,
                                            int max_depth,
                                            const std::vector<int>& exclude) const {
  std::vector<int> chain;
  if (seed < 0 || seed >= size()) return chain;
  std::vector<char> blocked(static_cast<size_t>(size()), 0);
  for (const int id : exclude) {
    if (id >= 0 && id < size()) blocked[static_cast<size_t>(id)] = 1;
  }
  std::vector<char> visited(static_cast<size_t>(size()), 0);
  std::vector<std::pair<int, int>> queue;  // (节点 id, 跳数)
  queue.emplace_back(seed, 0);
  visited[static_cast<size_t>(seed)] = 1;
  size_t head = 0;
  while (head < queue.size() && static_cast<int>(chain.size()) < max_nodes) {
    const auto [cur, depth] = queue[head++];
    chain.push_back(cur);
    if (depth >= max_depth) continue;
    for (const int nxt : adj_[static_cast<size_t>(cur)]) {
      if (visited[static_cast<size_t>(nxt)] || blocked[static_cast<size_t>(nxt)]) {
        continue;
      }
      visited[static_cast<size_t>(nxt)] = 1;
      queue.emplace_back(nxt, depth + 1);
    }
  }
  return chain;
}

void PoseGraph::rebuild_index() {
  buckets_.clear();
  for (int id = 0; id < size(); ++id) index_vertex(id);
}

std::vector<int> PoseGraph::vertices_within(Vec2 center, double radius,
                                            const std::vector<int>& exclude) const {
  std::vector<int> out;
  if (vertices_.empty()) return out;

  const int bx0 = static_cast<int>(std::floor((center.x - radius) / kBucket));
  const int bx1 = static_cast<int>(std::floor((center.x + radius) / kBucket));
  const int by0 = static_cast<int>(std::floor((center.y - radius) / kBucket));
  const int by1 = static_cast<int>(std::floor((center.y + radius) / kBucket));
  const double r2 = radius * radius;

  for (int bx = bx0; bx <= bx1; ++bx) {
    for (int by = by0; by <= by1; ++by) {
      const auto it = buckets_.find(bucket_key(bx, by));
      if (it == buckets_.end()) continue;
      for (const int id : it->second) {
        if (std::find(exclude.begin(), exclude.end(), id) != exclude.end()) {
          continue;
        }
        const Pose2& p = vertices_[static_cast<size_t>(id)].pose;
        const double dx = p.x - center.x;
        const double dy = p.y - center.y;
        if (dx * dx + dy * dy <= r2) out.push_back(id);
      }
    }
  }
  return out;
}

}  // namespace mini_slam
