#pragma once
// ============================================================================
// 位姿图序列化/反序列化（对应 slam_toolbox 的 .posegraph 持久化能力）
//
// 当前格式 MINISLAMGRAPHV2（本机字节序，跨机器交换请另行转换）：
//   magic[16]  "MINISLAMGRAPHV2\0"
//   u32 顶点数 N
//   N × { u32 id, f64 odom(x,y,θ), f64 map(x,y,θ),
//         u8 has_scan, [f64 scan_stamp, u32 点数, 点数 × (f64 x, f64 y)] }
//   u32 边数 M
//   M × { u32 from, u32 to, f64 z(x,y,θ),
//         f64 info_x, f64 info_y, f64 info_theta, u8 loop }
//
// 兼容读取 MINISLAMGRAPHV1：V1 边字段是各向同性 info_xy + info_theta，
// 读入时展开为 info_x = info_y = info_xy。
//
// 教学要点：图 SLAM 的持久化本质是把"图结构 + 每帧传感器原始数据"一起存，
// 恢复后才能继续定位（定位需要扫描点云来建匹配场，光有位姿不够）。
// ============================================================================
#include <expected>
#include <string>

#include "mini_slam/pose_graph.hpp"

namespace mini_slam::serialization {

// 保存 PoseGraph（含全部扫描点云）到二进制文件
std::expected<void, std::string> save_graph(const PoseGraph& graph,
                                            const std::string& path);

// 从文件恢复 PoseGraph（结构、位姿、扫描全部还原）
std::expected<PoseGraph, std::string> load_graph(const std::string& path);

}  // namespace mini_slam::serialization
