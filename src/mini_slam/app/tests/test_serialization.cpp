// 序列化测试：
//  1) 建图 → 保存 → 加载 → 结构逐位对账（顶点/位姿/扫描/边）
//  2) 加载后的图可直接做定位（扫描点云完整还原）
//  3) 原子保存：覆盖写、嵌套目录、.tmp 清理
//  4) V1 格式向后兼容
//  5) 坏文件 → 明确报错而不是崩溃
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <vector>

#include "mini_slam/mapper.hpp"
#include "mini_slam/serialization.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

template <typename T>
void write_pod(std::ostream& out, const T& v) {
  out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

void write_pose(std::ostream& out, const Pose2& p) {
  write_pod(out, p.x);
  write_pod(out, p.y);
  write_pod(out, p.theta);
}

std::filesystem::path make_root() {
  const auto root =
      std::filesystem::temp_directory_path() / "mini_slam_serialization_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  std::filesystem::create_directories(root, ec);
  return root;
}

}  // namespace

int main() {
  const auto root = make_root();
  const auto walls = test::box_world();
  std::mt19937 rng{23};

  // ---------- 建图：1.5m 方形环路 ----------
  Mapper mapper;
  std::vector<Pose2> gt;
  const std::vector<Vec2> corners = {Vec2{0, 0}, Vec2{1.5, 0},
                                     Vec2{1.5, 1.5}, Vec2{0, 1.5},
                                     Vec2{0, 0}};
  Pose2 odom = gt.emplace_back(corners.front().x, corners.front().y, 0.0);
  for (size_t c = 0; c + 1 < corners.size(); ++c) {
    const Vec2 d = corners[c + 1] - corners[c];
    const int steps = std::max(1, static_cast<int>(std::ceil(norm(d) / 0.25)));
    for (int i = 0; i < steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      const Pose2 cur{corners[c].x + t * d.x, corners[c].y + t * d.y,
                      std::atan2(d.y, d.x)};
      odom = compose(odom, compose(inverse(gt.back()), cur));
      const LaserScan scan =
          test::generate_scan(cur, walls, 361, 2.0 * kPi, 10.0, &rng);
      mapper.process_scan(scan, odom);
      gt.push_back(cur);
    }
  }
  CHECK(mapper.stats().nodes > 10);

  // ---------- 保存 → 加载 ----------
  const auto path = (root / "graph.mmpg").string();
  std::string err;
  CHECK(mapper.save_graph(path, &err));
  CHECK(!std::filesystem::exists(path + ".tmp"));
  auto loaded = serialization::load_graph(path);
  CHECK(loaded.has_value());

  // ---------- 逐位对账 ----------
  const auto& v0 = mapper.graph().vertices();
  const auto& v1 = loaded->vertices();
  CHECK(v0.size() == v1.size());
  for (size_t i = 0; i < v0.size(); ++i) {
    CHECK(distance_xy(v0[i].odom_pose, v1[i].odom_pose) < 1e-12);
    CHECK(distance_xy(v0[i].pose, v1[i].pose) < 1e-12);
    CHECK(static_cast<bool>(v0[i].scan) == static_cast<bool>(v1[i].scan));
    if (v0[i].scan && v1[i].scan) {
      CHECK(v0[i].scan->points.size() == v1[i].scan->points.size());
    }
  }
  const auto& e0 = mapper.graph().edges();
  const auto& e1 = loaded->edges();
  CHECK(e0.size() == e1.size());
  for (size_t i = 0; i < e0.size(); ++i) {
    CHECK(e0[i].from == e1[i].from && e0[i].to == e1[i].to);
    CHECK(e0[i].loop == e1[i].loop);
    CHECK(e0[i].info_x == e1[i].info_x);
    CHECK(e0[i].info_y == e1[i].info_y);
    CHECK(e0[i].info_theta == e1[i].info_theta);
  }

  // ---------- 覆盖写 + .tmp 清理 ----------
  CHECK(mapper.save_graph(path, &err));
  CHECK(!std::filesystem::exists(path + ".tmp"));
  auto loaded2 = serialization::load_graph(path);
  CHECK(loaded2.has_value());
  CHECK(loaded2->edges().size() == e0.size());

  // ---------- 嵌套不存在目录：自动创建 ----------
  const auto nested = root / "a" / "b" / "nested.mmpg";
  CHECK(mapper.save_graph(nested.string(), &err));
  CHECK(std::filesystem::exists(nested));
  CHECK(!std::filesystem::exists(nested.string() + ".tmp"));

  // ---------- V1 兼容：手工构造最小图 ----------
  {
    const auto v1_path = root / "legacy_v1.mmpg";
    std::ofstream f(v1_path, std::ios::binary);
    CHECK(static_cast<bool>(f));
    constexpr char magic[16] = "MINISLAMGRAPHV1";
    f.write(magic, 16);
    write_pod(f, std::uint32_t{1});  // 顶点数
    write_pod(f, std::uint32_t{0});  // id
    write_pose(f, Pose2{0.0, 0.0, 0.0});
    write_pose(f, Pose2{0.0, 0.0, 0.0});
    write_pod(f, false);             // has_scan
    write_pod(f, std::uint32_t{1});  // 边数
    write_pod(f, std::uint32_t{0});  // from
    write_pod(f, std::uint32_t{0});  // to
    write_pose(f, Pose2{0.0, 0.0, 0.0});
    write_pod(f, 4.0);   // info_xy
    write_pod(f, 9.0);   // info_theta
    write_pod(f, true);  // loop
    f.close();
    CHECK(static_cast<bool>(f));

    auto legacy = serialization::load_graph(v1_path.string());
    CHECK(legacy.has_value());
    CHECK(legacy->edges().size() == 1);
    CHECK(legacy->edges()[0].info_x == 4.0);
    CHECK(legacy->edges()[0].info_y == 4.0);
    CHECK(legacy->edges()[0].info_theta == 9.0);
  }

  // ---------- 加载后的图做定位 ----------
  Mapper loc_mapper;
  CHECK(loc_mapper.load_graph(path, &err));
  loc_mapper.set_mode(Mapper::Mode::Localization);
  std::mt19937 rng2{99};
  const LaserScan probe =
      test::generate_scan(gt[gt.size() / 2], walls, 361, 2.0 * kPi, 10.0, &rng2);
  const auto res = loc_mapper.localize(probe, gt[gt.size() / 2]);
  CHECK(res.valid);
  CHECK(distance_xy(res.pose, gt[gt.size() / 2]) < 0.15);
  CHECK(loc_mapper.stats().nodes == 0);   // 加载不产生新节点计数

  // ---------- 坏文件 → 明确报错 ----------
  {
    const auto bad_path = root / "bad.mmpg";
    std::ofstream f(bad_path, std::ios::binary);
    f << "GARBAGE NOT A GRAPH";
  }
  auto bad = serialization::load_graph((root / "bad.mmpg").string());
  CHECK(!bad.has_value());
  CHECK(!bad.error().empty());

  std::printf("[PASS] test_serialization  (nodes=%zu edges=%zu)\n",
              v0.size(), e0.size());

  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  return 0;
}
