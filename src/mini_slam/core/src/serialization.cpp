#include "mini_slam/serialization.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>

namespace mini_slam::serialization {
namespace {

// V2：边信息矩阵从单标量 info_xy 升级为世界系 info_x/info_y/info_theta。
constexpr char kMagicV2[16] = "MINISLAMGRAPHV2";
constexpr char kMagicV1[16] = "MINISLAMGRAPHV1";

// POD 块直写工具（同机字节序；跨机器交换是另一个话题，教学实现不展开）
template <typename T>
void write_pod(std::ostream& out, const T& v) {
  out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool read_pod(std::istream& in, T& v) {
  in.read(reinterpret_cast<char*>(&v), sizeof(T));
  return static_cast<bool>(in) || in.gcount() == static_cast<std::streamsize>(sizeof(T));
}

void write_pose(std::ostream& out, const Pose2& p) {
  write_pod(out, p.x);
  write_pod(out, p.y);
  write_pod(out, p.theta);
}

Pose2 read_pose(std::istream& in) {
  Pose2 p;
  in.read(reinterpret_cast<char*>(&p.x), sizeof(double));
  in.read(reinterpret_cast<char*>(&p.y), sizeof(double));
  in.read(reinterpret_cast<char*>(&p.theta), sizeof(double));
  return p;
}

}  // namespace

std::expected<void, std::string> save_graph(const PoseGraph& graph,
                                            const std::string& path) {
  const std::filesystem::path final_path(path);
  if (!final_path.parent_path().empty()) {
    std::error_code ec;
    std::filesystem::create_directories(final_path.parent_path(), ec);
    if (ec) {
      return std::unexpected("无法创建输出目录 '" +
                             final_path.parent_path().string() +
                             "': " + ec.message());
    }
  }

  // 原子写：先写 <path>.tmp，flush/close 全部成功后再 rename 覆盖目标。
  const std::string tmp = path + ".tmp";
  const auto cleanup = [&tmp] {
    std::error_code ec;
    std::filesystem::remove(tmp, ec);
  };

  std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
  if (!out) return std::unexpected("打不开输出文件: " + tmp);

  out.write(kMagicV2, 16);

  const auto& verts = graph.vertices();
  write_pod(out, static_cast<std::uint32_t>(verts.size()));
  for (const auto& v : verts) {
    write_pod(out, static_cast<std::uint32_t>(v.id));
    write_pose(out, v.odom_pose);
    write_pose(out, v.pose);
    const bool has_scan = static_cast<bool>(v.scan);
    write_pod(out, has_scan);
    if (has_scan) {
      write_pod(out, v.scan->stamp);
      write_pod(out, static_cast<std::uint32_t>(v.scan->points.size()));
      for (const Vec2& p : v.scan->points) {
        write_pod(out, p.x);
        write_pod(out, p.y);
      }
    }
  }

  const auto& edges = graph.edges();
  write_pod(out, static_cast<std::uint32_t>(edges.size()));
  for (const auto& e : edges) {
    write_pod(out, static_cast<std::uint32_t>(e.from));
    write_pod(out, static_cast<std::uint32_t>(e.to));
    write_pose(out, e.z);
    write_pod(out, e.info_x);
    write_pod(out, e.info_y);
    write_pod(out, e.info_theta);
    write_pod(out, e.loop);
  }

  out.flush();
  if (!out) {
    cleanup();
    return std::unexpected("写出失败: " + tmp);
  }
  out.close();
  if (!out) {
    cleanup();
    return std::unexpected("关闭输出文件失败: " + tmp);
  }

  std::error_code ec;
  std::filesystem::rename(tmp, final_path, ec);
  if (ec) {
    cleanup();
    return std::unexpected("重命名失败 '" + tmp + "' -> '" + path +
                           "': " + ec.message());
  }
  return {};
}

std::expected<PoseGraph, std::string> load_graph(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return std::unexpected("打不开文件: " + path);

  char magic[16];
  in.read(magic, 16);
  const bool v1 = in && std::memcmp(magic, kMagicV1, 16) == 0;
  const bool v2 = in && std::memcmp(magic, kMagicV2, 16) == 0;
  if (!v1 && !v2) {
    return std::unexpected("不是 MINISLAMGRAPHV1/V2 格式: " + path);
  }

  PoseGraph graph;
  std::uint32_t n_verts = 0;
  read_pod(in, n_verts);
  if (!in || n_verts > 1'000'000u) return std::unexpected("顶点数异常");
  for (std::uint32_t i = 0; i < n_verts; ++i) {
    std::uint32_t id = 0;
    read_pod(in, id);
    const Pose2 odom = read_pose(in);
    const Pose2 pose = read_pose(in);
    bool has_scan = false;
    read_pod(in, has_scan);
    std::shared_ptr<const LaserScan> scan;
    if (has_scan) {
      auto s = std::make_shared<LaserScan>();
      read_pod(in, s->stamp);
      std::uint32_t n_pts = 0;
      read_pod(in, n_pts);
      if (!in || n_pts > 100'000u) return std::unexpected("扫描点数异常");
      s->points.reserve(n_pts);
      for (std::uint32_t k = 0; k < n_pts; ++k) {
        Vec2 pt;
        in.read(reinterpret_cast<char*>(&pt.x), sizeof(double));
        in.read(reinterpret_cast<char*>(&pt.y), sizeof(double));
        s->points.push_back(pt);
      }
      scan = std::move(s);
    }
    if (!in) return std::unexpected("顶点数据截断");
    graph.add_vertex(odom, pose, std::move(scan));
  }

  std::uint32_t n_edges = 0;
  read_pod(in, n_edges);
  if (!in || n_edges > 5'000'000u) return std::unexpected("边数异常");
  for (std::uint32_t i = 0; i < n_edges; ++i) {
    std::uint32_t from = 0, to = 0;
    read_pod(in, from);
    read_pod(in, to);
    const Pose2 z = read_pose(in);
    double info_x = 0.0, info_y = 0.0, info_theta = 0.0;
    if (v2) {
      read_pod(in, info_x);
      read_pod(in, info_y);
      read_pod(in, info_theta);
    } else {
      // V1：只有各向同性 info_xy，补成 info_x = info_y。
      double info_xy = 0.0;
      read_pod(in, info_xy);
      read_pod(in, info_theta);
      info_x = info_xy;
      info_y = info_xy;
    }
    bool loop = false;
    read_pod(in, loop);
    if (!in) return std::unexpected("边数据截断");
    const auto n_vertices = static_cast<std::uint32_t>(graph.size());
    if (from >= n_vertices || to >= n_vertices) {
      return std::unexpected("边引用了不存在的节点");
    }
    graph.add_edge(static_cast<int>(from), static_cast<int>(to), z,
                   info_x, info_y, info_theta, loop);
  }

  graph.rebuild_index();
  return graph;
}

}  // namespace mini_slam::serialization
