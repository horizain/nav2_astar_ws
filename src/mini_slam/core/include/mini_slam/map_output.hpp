#pragma once
// ============================================================================
// 地图文件输出：PNG（可视化）+ .meta（分辨率/原点/尺寸）+ PGM（对拍工具）。
// 从 ROS 节点抽出成纯 C++，便于测试"裸文件名 / 带目录前缀"两种路径。
//
// 关键约束：map_output_prefix 是裸文件名时 path::parent_path() 为空，
// 不能对空路径调用 create_directories（会抛 filesystem_error）。
// ============================================================================
#include <filesystem>
#include <string>

#include "mini_slam/map_builder.hpp"

namespace mini_slam::map_output {

// 保证 file 的父目录存在。父目录为空（裸文件名）时直接成功。
// 目录创建失败返回 false，err 带可读原因（不抛异常）。
inline bool ensure_parent_directory(const std::filesystem::path& file,
                                    std::string* err) {
  const std::filesystem::path parent = file.parent_path();
  if (parent.empty()) return true;  // 裸文件名：写到当前工作目录

  std::error_code ec;
  std::filesystem::create_directories(parent, ec);
  if (ec) {
    if (err) {
      *err = "无法创建地图目录 '" + parent.string() + "': " + ec.message();
    }
    return false;
  }
  return true;
}

// 写出 prefix.png / prefix.meta / prefix.pgm。
// 返回 false 时 err 说明失败的文件；不抛 filesystem_error。
bool write_map_files(const std::string& prefix, const OccupancyMap& map,
                     std::string* err);

}  // namespace mini_slam::map_output
