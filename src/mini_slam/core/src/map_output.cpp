#include "mini_slam/map_output.hpp"

#include <cstdint>
#include <expected>
#include <fstream>
#include <initializer_list>
#include <utility>
#include <vector>

#include "mini_slam/png_writer.hpp"
#include "mini_slam/serialization.hpp"

namespace mini_slam::map_output {

bool write_map_files(const std::string& prefix, const OccupancyMap& map,
                     std::string* err) {
  if (map.width <= 0 || map.height <= 0) {
    if (err) *err = "地图为空（width/height <= 0）";
    return false;
  }
  const std::filesystem::path path(prefix);
  if (!ensure_parent_directory(path, err)) return false;

  // OccupancyMap 的 data 行序是 y 从下到上；PNG/PGM 行序是 y 从上到下。
  std::vector<std::uint8_t> pix(
      static_cast<size_t>(map.width) * static_cast<size_t>(map.height));
  for (int y = 0; y < map.height; ++y) {
    const int src_y = map.height - 1 - y;
    for (int x = 0; x < map.width; ++x) {
      const std::int8_t v =
          map.data[static_cast<size_t>(src_y) * map.width + x];
      pix[static_cast<size_t>(y) * map.width + x] =
          static_cast<std::uint8_t>(v == 100 ? 0 : (v == 0 ? 254 : 205));
    }
  }

  const std::filesystem::path png_path = prefix + ".png";
  const std::filesystem::path meta_path = prefix + ".meta";
  const std::filesystem::path pgm_path = prefix + ".pgm";
  const std::filesystem::path yaml_path = prefix + ".yaml";
  const std::filesystem::path png_tmp = png_path.string() + ".tmp";
  const std::filesystem::path meta_tmp = meta_path.string() + ".tmp";
  const std::filesystem::path pgm_tmp = pgm_path.string() + ".tmp";
  const std::filesystem::path yaml_tmp = yaml_path.string() + ".tmp";

  const auto cleanup = [&] {
    std::error_code ec;
    std::filesystem::remove(png_tmp, ec);
    std::filesystem::remove(meta_tmp, ec);
    std::filesystem::remove(pgm_tmp, ec);
    std::filesystem::remove(yaml_tmp, ec);
  };
  const auto fail = [&](const std::string& msg) {
    cleanup();
    if (err) *err = msg;
    return false;
  };

  // 1) PNG（可视化）
  if (!png::write_grayscale(png_tmp.string(), map.width, map.height, pix)) {
    return fail("写 PNG 失败: " + png_tmp.string());
  }

  // 2) meta（compare_maps 仍在使用）
  {
    std::ofstream f(meta_tmp);
    if (!f) return fail("写 meta 失败: " + meta_tmp.string());
    f << map.resolution << " " << map.origin.x << " " << map.origin.y << " "
      << map.width << " " << map.height << "\n";
    f.flush();
    f.close();
    if (!f) return fail("写 meta 失败: " + meta_tmp.string());
  }

  // 3) PGM（map_server / compare_maps 通用）
  {
    std::ofstream f(pgm_tmp, std::ios::binary);
    if (!f) return fail("写 PGM 失败: " + pgm_tmp.string());
    f << "P5\n" << map.width << " " << map.height << "\n255\n";
    f.write(reinterpret_cast<const char*>(pix.data()),
            static_cast<std::streamsize>(pix.size()));
    f.flush();
    f.close();
    if (!f) return fail("写 PGM 失败: " + pgm_tmp.string());
  }

  // 4) 标准 map_server YAML：image 用最终 PGM 的 basename，便于整体搬移。
  {
    std::ofstream f(yaml_tmp);
    if (!f) return fail("写 YAML 失败: " + yaml_tmp.string());
    f << "image: " << pgm_path.filename().string() << "\n"
      << "mode: trinary\n"
      << "resolution: " << map.resolution << "\n"
      << "origin: [" << map.origin.x << ", " << map.origin.y << ", 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.65\n"
      << "free_thresh: 0.196\n";
    f.flush();
    f.close();
    if (!f) return fail("写 YAML 失败: " + yaml_tmp.string());
  }

  // 全部临时文件写成功后统一 rename；yaml 最后（它引用 pgm）。
  const auto commit =
      [](const std::filesystem::path& tmp,
         const std::filesystem::path& final_path)
      -> std::expected<void, std::string> {
    std::error_code ec;
    std::filesystem::rename(tmp, final_path, ec);
    if (ec) {
      return std::unexpected("重命名失败 '" + tmp.string() + "' -> '" +
                             final_path.string() + "': " + ec.message());
    }
    return {};
  };

  for (const auto& [tmp, dst] :
       std::initializer_list<std::pair<std::filesystem::path,
                                       std::filesystem::path>>{
           {pgm_tmp, pgm_path},
           {png_tmp, png_path},
           {meta_tmp, meta_path},
           {yaml_tmp, yaml_path}}) {
    if (const auto r = commit(tmp, dst); !r) return fail(r.error());
  }
  return true;
}

bool write_snapshot(const std::string& prefix, const PoseGraph& graph,
                    double resolution, double range_max, double margin,
                    std::string* err) {
  if (prefix.empty()) {
    if (err) *err = "地图前缀不能为空";
    return false;
  }

  // 预检成套目标和临时文件，避免覆盖已有地图或未完成的上次输出。
  for (const char* ext : {".mmpg", ".mmpg.tmp", ".png", ".png.tmp",
                          ".meta", ".meta.tmp", ".pgm", ".pgm.tmp",
                          ".yaml", ".yaml.tmp"}) {
    const std::filesystem::path path = prefix + ext;
    std::error_code ec;
    const auto status = std::filesystem::symlink_status(path, ec);
    // 不存在的路径可用；失效软链接仍是已有目录项，必须拒绝覆盖。
    if (ec && ec != std::errc::no_such_file_or_directory) {
      if (err) *err = "检查输出文件失败: " + path.string() + ": " + ec.message();
      return false;
    }
    if (status.type() != std::filesystem::file_type::not_found) {
      if (err) *err = "输出文件已存在: " + path.string();
      return false;
    }
  }

  const auto map = MapBuilder::render(graph, resolution, range_max, margin);
  if (map.width <= 0 || map.height <= 0) {
    if (err) *err = "位姿图没有可渲染的地图";
    return false;
  }

  const auto graph_result = serialization::save_graph(graph, prefix + ".mmpg");
  if (!graph_result) {
    if (err) *err = "保存位姿图失败: " + graph_result.error();
    return false;
  }
  if (!write_map_files(prefix, map, err)) {
    if (err) *err = "位姿图已保存，栅格地图保存失败: " + *err;
    return false;
  }
  return true;
}

}  // namespace mini_slam::map_output
