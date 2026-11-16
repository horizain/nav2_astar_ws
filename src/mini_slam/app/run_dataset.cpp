// ============================================================================
// mini_slam_dataset：读 CARMEN 日志 → 跑完整 SLAM 流水线 → 输出
//   out/map.pgm / out/map.png / out/map.meta / out/trajectory.svg
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "carmen_reader.hpp"
#include "mini_slam/detail/format.hpp"
#include "mini_slam/map_builder.hpp"
#include "mini_slam/mapper.hpp"
#include "mini_slam/png_writer.hpp"

using namespace mini_slam;

namespace {

struct Args {
  std::string input;
  std::string out{"out"};
  double resolution{0.0};  // 0 = 用默认
  int skip{0};
  long max_scans{0};  // 0 = 不限（按扫描帧数计）
  bool verbose{false};
  bool flip{false};    // 把每帧点云旋转 180°（intel PowerBot 激光朝后装的坑）
  bool no_loop{false}; // 隔离实验：完全关掉闭环
  bool probe_offset{false};  // 诊断：探测激光安装角偏移
  bool show_help{false};
};

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    auto next = [&]() -> std::string {
      return (i + 1 < argc) ? argv[++i] : "";
    };
    if (s == "--input" || s == "-i") a.input = next();
    else if (s == "--out" || s == "-o") a.out = next();
    else if (s == "--resolution") { auto v = next(); try { a.resolution = std::stod(v); } catch (...) {} }
    else if (s == "--skip") a.skip = std::atoi(next().c_str());
    else if (s == "--max-scans") a.max_scans = std::atol(next().c_str());
    else if (s == "--verbose" || s == "-v") a.verbose = true;
    else if (s == "--flip") a.flip = true;
    else if (s == "--no-loop") a.no_loop = true;
    else if (s == "--probe-offset") a.probe_offset = true;
    else if (s == "--help" || s == "-h") a.show_help = true;
  }
  return a;
}

bool write_pgm(const std::string& path, const OccupancyMap& map) {
  // 图像行序 = 世界 y 向上翻转（PNG/SVG 与世界坐标一致）
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << "P5\n" << map.width << " " << map.height << "\n255\n";
  for (int y = map.height - 1; y >= 0; --y) {
    for (int x = 0; x < map.width; ++x) {
      const std::int8_t v = map.data[static_cast<size_t>(y) * map.width + x];
      f.put(v == 100 ? 0 : (v == 0 ? 254 : 205));
    }
  }
  return static_cast<bool>(f);
}

// RGB 地图 + 轨迹叠加：墙=黑，空闲=白，未知=浅灰；灰线=原始里程计轨迹，
// 蓝线=优化后轨迹，红圈=闭环节点。一张图看全 SLAM 的因果。
bool write_map_png(const std::string& path, const OccupancyMap& map,
                   const std::vector<std::pair<double, Pose2>>& odom,
                   const PoseGraph& graph, const std::vector<int>& loop_nodes) {
  std::vector<std::uint8_t> pix_rgba;
  auto to_px = [&](double wx, double wy) -> std::pair<int, int> {
    const int px = static_cast<int>((wx - map.origin.x) / map.resolution);
    const int py = static_cast<int>((wy - map.origin.y) / map.resolution);
    return {px, map.height - 1 - py};
  };
  auto put = [&](int px, int py, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    if (px < 0 || py < 0 || px >= map.width || py >= map.height) return;
    const size_t i = (static_cast<size_t>(py) * map.width + px) * 3;
    pix_rgba[i] = r;
    pix_rgba[i + 1] = g;
    pix_rgba[i + 2] = b;
  };
  auto line = [&](std::pair<int, int> a, std::pair<int, int> b,
                  std::uint8_t r, std::uint8_t g, std::uint8_t bl) {
    const int steps = std::max(std::abs(b.first - a.first),
                               std::abs(b.second - a.second));
    for (int i = 0; i <= steps; ++i) {
      const double t = steps > 0 ? static_cast<double>(i) / steps : 0.0;
      put(a.first + static_cast<int>(t * (b.first - a.first)),
          a.second + static_cast<int>(t * (b.second - a.second)), r, g, bl);
    }
  };

  // 底图
  pix_rgba.assign(static_cast<size_t>(map.width) * map.height * 3, 255);
  for (int y = 0; y < map.height; ++y) {
    const int src_y = map.height - 1 - y;
    for (int x = 0; x < map.width; ++x) {
      const std::int8_t v = map.data[static_cast<size_t>(src_y) * map.width + x];
      if (v == 100) {
        put(x, y, 0, 0, 0);
      } else if (v != 0) {
        put(x, y, 220, 220, 220);
      }
    }
  }
  // 里程计轨迹（灰，隔 5 点画一次别糊死）
  for (size_t i = 5; i < odom.size(); i += 5) {
    line(to_px(odom[i - 5].second.x, odom[i - 5].second.y),
         to_px(odom[i].second.x, odom[i].second.y), 180, 180, 200);
  }
  // 优化后轨迹（蓝）
  const auto& verts = graph.vertices();
  for (size_t i = 1; i < verts.size(); ++i) {
    line(to_px(verts[i - 1].pose.x, verts[i - 1].pose.y),
         to_px(verts[i].pose.x, verts[i].pose.y), 30, 70, 220);
  }
  // 闭环节点（红圈）
  for (const int id : loop_nodes) {
    const auto [px, py] = to_px(verts[id].pose.x, verts[id].pose.y);
    for (int dy = -3; dy <= 3; ++dy) {
      for (int dx = -3; dx <= 3; ++dx) {
        if (dx * dx + dy * dy <= 9) put(px + dx, py + dy, 220, 30, 30);
      }
    }
  }
  return png::write_rgb(path, map.width, map.height, pix_rgba);
}

bool write_meta(const std::string& path, const OccupancyMap& map) {
  std::ofstream f(path);
  if (!f) return false;
  f << map.resolution << " " << map.origin.x << " " << map.origin.y << " "
    << map.width << " " << map.height << "\n";
  return static_cast<bool>(f);
}

void write_trajectory_svg(const std::string& path,
                          const std::vector<std::pair<double, Pose2>>& odom,
                          const PoseGraph& graph,
                          const std::vector<int>& loop_nodes) {
  // 没有任何轨迹点：写一个合法的最小空 SVG，避免 w/h 为负、scale 为 inf。
  if (odom.empty() && graph.vertices().empty()) {
    std::ofstream f(path);
    if (!f) return;
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1\" "
         "height=\"1\" style=\"background:white\"></svg>\n";
    return;
  }

  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  auto grow = [&](double x, double y) {
    min_x = std::min(min_x, x); min_y = std::min(min_y, y);
    max_x = std::max(max_x, x); max_y = std::max(max_y, y);
  };
  for (const auto& [t, p] : odom) grow(p.x, p.y);
  for (const auto& v : graph.vertices()) grow(v.pose.x, v.pose.y);
  const double pad = 1.0;
  min_x -= pad; min_y -= pad; max_x += pad; max_y += pad;
  const double w = max_x - min_x, h = max_y - min_y;
  const int img_w = 1000;
  const double scale = static_cast<double>(img_w) / std::max(w, 1e-9);
  // 纵向至少 1px：全在同一水平线上的轨迹 h*scale 会取整成 0。
  const int img_h = std::max(1, static_cast<int>(h * scale));
  auto sx = [&](double x) { return (x - min_x) * scale; };
  auto sy = [&](double y) { return static_cast<double>(img_h) - (y - min_y) * scale; };

  auto polyline = [&](const auto& pts, const char* color) {
    std::string s = "<polyline fill=\"none\" stroke=\"" + std::string(color) +
                    "\" stroke-width=\"1.5\" points=\"";
    for (size_t i = 0; i < pts.size(); ++i) {
      const auto [px, py] = pts[i];
      s += std::format("{:.1f},{:.1f} ", sx(px), sy(py));
    }
    return s + "\"/>";
  };

  std::ofstream f(path);
  if (!f) return;
  f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << img_w
    << "\" height=\"" << img_h << "\" style=\"background:white\">\n";
  std::vector<std::pair<double, double>> odom_pts;
  for (const auto& [t, p] : odom) odom_pts.emplace_back(p.x, p.y);
  f << polyline(odom_pts, "#aaaaaa") << "\n";
  std::vector<std::pair<double, double>> map_pts;
  for (const auto& v : graph.vertices()) map_pts.emplace_back(v.pose.x, v.pose.y);
  f << polyline(map_pts, "#1a56db") << "\n";
  for (const int id : loop_nodes) {
    const auto& v = graph.vertices()[static_cast<size_t>(id)];
    f << std::format("<circle cx=\"{:.1f}\" cy=\"{:.1f}\" r=\"4\" fill=\"none\" "
                     "stroke=\"#e02424\" stroke-width=\"2\"/>\n",
                     sx(v.pose.x), sy(v.pose.y));
  }
  f << "</svg>\n";
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  if (args.show_help || args.input.empty()) {
    io::println("用法: mini_slam_dataset --input <carmen.log> [--out dir] "
                "[--resolution 0.05] [--skip N] [--max-scans N]");
    return args.input.empty() ? 1 : 0;
  }

  // ---------- 读日志 ----------
  const auto log = app::read_carmen_log(args.input);
  if (!log) {
    io::println("读取失败: {}", log.error());
    return 1;
  }
  io::println("日志: odom={} scans={} skipped_lines={}", log->odom.size(),
              log->scans.size(), log->skipped_lines);
  if (log->scans.empty()) {
    io::println("没有激光扫描，退出");
    return 1;
  }

  // ---------- 跑 SLAM ----------
  Params params;
  if (args.resolution > 0) params.resolution = args.resolution;
  params.verbose = args.verbose;
  if (args.no_loop) params.loop_attempt_interval = 1 << 30;
  Mapper mapper{params};

  // 诊断开关：若激光朝后安装，点云需整体旋转 180°（x,y 同时取反）
  auto scans = log->scans;  // log 是 const 的，拷一份出来改
  if (args.flip) {
    for (auto& scan : scans) {
      for (auto& pt : scan.points) {
        pt.x = -pt.x;
        pt.y = -pt.y;
      }
    }
    io::println("[flip] 已将全部扫描旋转 180°");
  }

  const auto t0 = std::chrono::steady_clock::now();
  long processed = 0;   // 实际喂给 SLAM 的扫描帧数
  long nodes_added = 0;
  std::vector<int> loop_nodes;

  // ---------- 诊断：激光安装角偏移探测 ----------
  // 取一帧扫描，用它前面的帧（按里程计位姿摆放）建得分场，然后把这一帧
  // 分别旋转不同的固定角 φ 去匹配。响应最高的 φ 就是激光坐标系相对
  // 里程计坐标系的固定安装偏角（0 说明没有偏移）。
  if (args.probe_offset) {
    // 参照场固定取探测帧之前的 30 帧；不足 31 帧时 probe_idx - 30 会下溢。
    if (scans.size() < 31) {
      io::println("--probe-offset 至少需要 31 帧扫描（当前只有 {} 帧）",
                  scans.size());
      return 1;
    }
    const size_t probe_idx = std::min<size_t>(1500, scans.size() - 1);
    std::vector<mini_slam::FramedScan> refs;
    for (size_t i = probe_idx - 30; i < probe_idx; ++i) {
      refs.push_back({log->scan_odom[i], scans[i].points});
    }
    auto field = mini_slam::LikelihoodField::build(refs, params.resolution,
                                                   1.0, params.blur_radius);
    if (field) {
      const mini_slam::Pose2 prior = log->scan_odom[probe_idx];
      io::println("探测 scan {} @ odom ({:.2f},{:.2f},{:.2f}):", probe_idx,
                  prior.x, prior.y, prior.theta);
      for (double deg = -180; deg <= 180; deg += 15) {
        const double phi = deg * kPi / 180.0;
        std::vector<mini_slam::Vec2> rotated;
        rotated.reserve(scans[probe_idx].points.size());
        for (const auto& pt : scans[probe_idx].points) {
          const double c = std::cos(phi), s = std::sin(phi);
          rotated.push_back({c * pt.x - s * pt.y, s * pt.x + c * pt.y});
        }
        const mini_slam::MatchWindow w{0.3, 0.3, 0.10, 0.05, true};
        const auto m = mini_slam::CorrelativeMatcher::match(*field, prior,
                                                            rotated, w);
        io::println("  φ={:+4.0f}°  resp={:.3f}  coarse={:.3f}", deg,
                    m.response, m.coarse_response);
      }
    }
    std::fflush(stdout);
    return 0;
  }
  for (size_t i = 0; i < log->scans.size(); ++i) {
    if (static_cast<int>(i) < args.skip) continue;
    if (args.max_scans > 0 && processed >= args.max_scans) break;
    const auto r = mapper.process_scan(scans[i], log->scan_odom[i]);
    ++processed;
    if (r.added) {
      ++nodes_added;
      if (r.loop_closed) {
        loop_nodes.push_back(r.id);
        io::println("  闭环 @ scan {} → 节点 {}（第 {} 次）", i, r.id,
                    mapper.stats().loop_closures);
      }
    }
    if (processed % 200 == 0 && processed > 0) {
      const auto& st = mapper.stats();
      io::println("  进度 {}/{}  nodes={} loops={} solves={}", i + 1,
                  log->scans.size(), st.nodes, st.loop_closures, st.solves);
      std::fflush(stdout);  // 重定向时也能看到进度
    }
  }
  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();

  const auto& st = mapper.stats();
  io::println("完成: 喂入 {}/{} 帧, 节点 {}, 闭环 {}, SPA {} 次 "
              "(cost {:.3e} → {:.3e}, iters {})",
              processed, log->scans.size(), st.nodes, st.loop_closures,
              st.solves, st.last_solve.initial_cost, st.last_solve.final_cost,
              st.last_solve.iterations);
  io::println("耗时 {:.2f}s ({:.1f} 帧/秒)", secs,
              secs > 0 ? static_cast<double>(processed) / secs : 0.0);

  // ---------- 输出 ----------
  std::filesystem::create_directories(args.out);
  const auto map = MapBuilder::render(mapper.graph(), params.resolution,
                                      params.map_range_max, params.map_margin);
  const std::string base = args.out + "/map";
  io::println("地图: {}x{} @ {:.3f}m, origin ({:.2f}, {:.2f})", map.width,
              map.height, map.resolution, map.origin.x, map.origin.y);
  if (!write_pgm(base + ".pgm", map) ||
      !write_map_png(base + ".png", map, log->odom, mapper.graph(),
                     loop_nodes) ||
      !write_meta(base + ".meta", map)) {
    io::println("地图写出失败");
    return 1;
  }
  write_trajectory_svg(args.out + "/trajectory.svg", log->odom,
                       mapper.graph(), loop_nodes);
  io::println("已写出: {}/map.pgm, map.png, map.meta, trajectory.svg",
              args.out);
  return 0;
}
