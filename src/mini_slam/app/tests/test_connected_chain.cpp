// 图连通链测试：两条平行走廊的空间邻居 vs 拓扑连通链的区分能力
//
// 结构：走廊 A（节点 0-49，y=0，x=0..24.5，0.5m 间隔）
//       走廊 B（节点 50-99，y=1.0，x=0..24.5）——与 A 平行、仅 1m 之隔
// 场景意义：在走廊 B 中段做闭环匹配时，空间半径查询会把走廊 A 的节点
// 也抓进来（散乱的跨走廊集合）；而连通链必须只含 B 的连贯轨迹段。
#include "mini_slam/pose_graph.hpp"

#include "test_util.hpp"

#include <algorithm>
#include <cstdio>

using namespace mini_slam;

int main() {
  PoseGraph g;
  const int N = 50;
  for (int i = 0; i < N; ++i) {
    g.add_vertex(Pose2{i * 0.5, 0.0, 0.0}, Pose2{i * 0.5, 0.0, 0.0});
  }
  for (int i = 0; i < N; ++i) {
    g.add_vertex(Pose2{i * 0.5, 1.0, 0.0}, Pose2{i * 0.5, 1.0, 0.0});
  }
  // 各走廊内部顺序连边
  for (int i = 0; i + 1 < N; ++i) {
    g.add_edge(i, i + 1, Pose2{0.5, 0, 0}, 100, 100, 100, false);
    g.add_edge(N + i, N + i + 1, Pose2{0.5, 0, 0}, 100, 100, 100, false);
  }

  // 从走廊 B 中段（id=75）取连通链
  auto chain = g.connected_chain(75, 20, 10, {});
  CHECK(chain.size() >= 15);
  bool cross_A = false;
  for (const int id : chain) {
    if (id < N) cross_A = true;   // 混入了走廊 A → 连通链失效
  }
  CHECK(!cross_A);
  // 链应覆盖 B 的相邻段（75±若干）
  int lo = 100, hi = -1;
  for (const int id : chain) {
    lo = std::min(lo, id - N);
    hi = std::max(hi, id - N);
  }
  std::printf("  链范围: B[%d..%d]，共 %zu 节点\n", lo, hi, chain.size());
  CHECK(hi - lo >= 10);   // 链沿走廊有足够延展

  // 对比：同位置的空间半径查询（1.5m）会同时抓到两条走廊
  auto spatial = g.vertices_within(Vec2{12.5, 0.5}, 1.5);
  int from_A = 0, from_B = 0;
  for (const int id : spatial) (id < N ? from_A : from_B)++;
  std::printf("  空间查询 1.5m: A=%d B=%d（跨走廊污染对照）\n", from_A, from_B);
  CHECK(from_A > 0 && from_B > 0);   // 证明空间查询确实有跨走廊污染

  std::puts("[PASS] test_connected_chain");
  return 0;
}
