// 验证成套快照的位姿图与栅格来自同一输入，并拒绝覆盖已有文件。
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include "mini_slam/map_builder.hpp"
#include "mini_slam/map_output.hpp"
#include "mini_slam/serialization.hpp"
#include "test_util.hpp"

using namespace mini_slam;

int main() {
  const auto root = std::filesystem::temp_directory_path() /
      ("mini_slam_snapshot_test_" + std::to_string(static_cast<long>(::getpid())));
  const std::string prefix = (root / "session").string();
  std::filesystem::create_directories(root);

  PoseGraph graph;
  auto scan = std::make_shared<LaserScan>();
  scan->points = {Vec2{1.0, 0.0}, Vec2{0.0, 1.0},
                  Vec2{-1.0, 0.0}, Vec2{0.0, -1.0}};
  graph.add_vertex(Pose2{0.0, 0.0, 0.0}, Pose2{0.4, 0.1, 0.2}, scan);
  const auto expected = MapBuilder::render(graph, 0.05, 12.0, 0.2);
  CHECK(expected.width > 0 && expected.height > 0);

  std::string err;
  CHECK(map_output::write_snapshot(prefix, graph, 0.05, 12.0, 0.2, &err));
  auto loaded = serialization::load_graph(prefix + ".mmpg");
  CHECK(loaded.has_value());
  const auto rendered = MapBuilder::render(*loaded, 0.05, 12.0, 0.2);
  CHECK(rendered.width == expected.width && rendered.height == expected.height);
  CHECK(rendered.data == expected.data);

  // 地图文件是 y 轴翻转的 PGM；逐像素核对图快照渲染的结果。
  std::ifstream pgm(prefix + ".pgm", std::ios::binary);
  std::string magic;
  int width = 0, height = 0, maximum = 0;
  pgm >> magic >> width >> height >> maximum;
  pgm.get();
  CHECK(magic == "P5" && width == expected.width && height == expected.height);
  CHECK(maximum == 255);
  std::vector<unsigned char> pixels(
      std::istreambuf_iterator<char>(pgm), std::istreambuf_iterator<char>{});
  CHECK(pixels.size() == expected.data.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto cell = expected.data[static_cast<size_t>(height - 1 - y) * width + x];
      const unsigned char want = cell == 100 ? 0 : cell == 0 ? 254 : 205;
      CHECK(pixels[static_cast<size_t>(y) * width + x] == want);
    }
  }

  std::ifstream yaml(prefix + ".yaml");
  const std::string yaml_before(std::istreambuf_iterator<char>(yaml), {});
  CHECK(yaml_before.find("image: session.pgm") != std::string::npos);
  err.clear();
  CHECK(!map_output::write_snapshot(prefix, graph, 0.05, 12.0, 0.2, &err));
  CHECK(err.find("已存在") != std::string::npos);
  std::ifstream yaml_again(prefix + ".yaml");
  const std::string yaml_after(std::istreambuf_iterator<char>(yaml_again), {});
  CHECK(yaml_before == yaml_after);

  // 即使只存在临时文件，也不能覆盖或清理它。
  const std::string collision = (root / "collision").string();
  { std::ofstream f(collision + ".pgm.tmp"); f << "preserve"; }
  err.clear();
  CHECK(!map_output::write_snapshot(collision, graph, 0.05, 12.0, 0.2, &err));
  std::ifstream marker(collision + ".pgm.tmp");
  const std::string contents(std::istreambuf_iterator<char>(marker), {});
  CHECK(contents == "preserve");
  CHECK(!std::filesystem::exists(collision + ".mmpg"));
  // 失效软链接也是现有路径，不能被 rename 覆盖。
  const std::string linked = (root / "linked").string();
  std::filesystem::create_symlink(root / "missing_target", linked + ".yaml");
  err.clear();
  CHECK(!map_output::write_snapshot(linked, graph, 0.05, 12.0, 0.2, &err));
  CHECK(std::filesystem::is_symlink(linked + ".yaml"));
  CHECK(!std::filesystem::exists(linked + ".mmpg"));
  std::printf("[PASS] test_snapshot (%s)\n", root.c_str());
  return 0;
}
