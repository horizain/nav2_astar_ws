// map_output_prefix 路径处理：
//   裸文件名（parent_path 为空）不能抛 filesystem_error；
//   带目录前缀时自动创建父目录并写出 png/meta/pgm/yaml；
//   标准 map_server YAML + 原子写（无 .tmp 残留）+ 失败路径。
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "mini_slam/map_output.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

OccupancyMap tiny_map() {
  OccupancyMap m;
  m.resolution = 0.05;
  m.origin = Vec2{-0.1, -0.1};
  m.width = 4;
  m.height = 4;
  m.data.assign(16, std::int8_t{-1});
  m.data[0] = 100;
  m.data[5] = 0;
  return m;
}

}  // namespace

int main() {
  const auto root =
      std::filesystem::temp_directory_path() / "mini_slam_map_output_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  std::filesystem::create_directories(root, ec);
  CHECK(!ec);

  const auto old_cwd = std::filesystem::current_path();
  std::filesystem::current_path(root, ec);
  CHECK(!ec);

  // 1) 裸文件名：不应抛异常，文件落在当前目录
  std::string err;
  const OccupancyMap map = tiny_map();
  CHECK(map_output::write_map_files("mini_map", map, &err));
  CHECK(err.empty());
  for (const char* ext : {".png", ".meta", ".pgm", ".yaml"}) {
    CHECK(std::filesystem::exists(root / (std::string("mini_map") + ext)));
  }
  {
    std::ifstream f(root / "mini_map.meta");
    std::string line;
    std::getline(f, line);
    CHECK(!line.empty());
  }
  {
    std::ifstream f(root / "mini_map.yaml");
    const std::string text((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
    CHECK(text.find("image: mini_map.pgm") != std::string::npos);
    CHECK(text.find("mode: trinary") != std::string::npos);
    CHECK(text.find("resolution:") != std::string::npos);
    CHECK(text.find("origin:") != std::string::npos);
  }
  // 原子写完成后不应残留任何 .tmp（含子目录）
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(root)) {
    CHECK(entry.path().extension() != ".tmp");
  }

  // 2) 带目录前缀：父目录不存在时自动创建
  err.clear();
  CHECK(map_output::write_map_files("out/sub/mini_map", map, &err));
  CHECK(err.empty());
  CHECK(std::filesystem::is_directory(root / "out" / "sub"));
  for (const char* ext : {".png", ".meta", ".pgm", ".yaml"}) {
    CHECK(std::filesystem::exists(root / "out" / "sub" /
                                 (std::string("mini_map") + ext)));
  }

  // 3) ensure_parent_directory 对空父路径直接成功
  CHECK(map_output::ensure_parent_directory("bare_name", &err));

  // 4) 失败路径：父目录位置是普通文件 → 返回 false，不抛异常
  {
    std::ofstream blocker(root / "blocker");
    blocker << "not a directory";
    blocker.close();
    err.clear();
    CHECK(!map_output::write_map_files("blocker/map", map, &err));
    CHECK(!err.empty());
  }

  std::filesystem::current_path(old_cwd, ec);
  std::filesystem::remove_all(root, ec);

  std::puts("[PASS] test_map_output");
  return 0;
}
