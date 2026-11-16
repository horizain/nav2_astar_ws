#include "carmen_reader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>

namespace mini_slam::app {
namespace {

std::vector<std::string> tokenize(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> out;
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}

std::expected<double, std::string> parse_double(const std::string& s) {
  try {
    const double v = std::stod(s);
    return v;
  } catch (...) {
    return std::unexpected("不是数字: " + s);
  }
}

// 经典 CARMEN FLASER 行（Intel Lab 数据集用的就是它）：
//   FLASER <n> <r1..rn> <x> <y> <theta> <tv> <rv> <accel> <ipc_ts> <host> <logger_ts>
// 位姿内嵌在行里（扫描时刻的里程计），连插值都省了。
std::expected<std::pair<LaserScan, Pose2>, std::string> parse_flaser(
    const std::vector<std::string>& t) {
  const auto n_v = parse_double(t[1]);
  if (!n_v || *n_v < 1 || *n_v > 5000) return std::unexpected("FLASER 点数异常");
  const int n = static_cast<int>(*n_v);
  const size_t base = 2 + static_cast<size_t>(n);
  if (t.size() < base + 8) return std::unexpected("FLASER 行太短");

  LaserScan scan;
  scan.stamp = parse_double(t[base + 6]).value_or(0.0);
  // SICK LMS：180° 视场角；无效读数是巨大的占位值（81.83）或非正数
  const double angle_min = -kPi / 2.0;
  const double inc = n > 1 ? kPi / static_cast<double>(n - 1) : 0.0;
  for (int i = 0; i < n; ++i) {
    const double r = parse_double(t[1 + static_cast<size_t>(i)]).value_or(0.0);
    if (r <= 0.01 || r >= 80.0) continue;
    const double ang = angle_min + inc * i;
    scan.points.push_back(Vec2{r * std::cos(ang), r * std::sin(ang)});
  }
  const auto x = parse_double(t[base]);
  const auto y = parse_double(t[base + 1]);
  const auto th = parse_double(t[base + 2]);
  if (!x || !y || !th) return std::unexpected("FLASER 位姿不是数字");
  return std::make_pair(std::move(scan), Pose2{*x, *y, *th});
}

// ROBOT LASER 行有两种常见布局，自动探测：
//   A: ROBOT LASER <n> <r1..rn> <fov> <maxrange> <res> <mode> [<remissions>] <ts>
//   B: ROBOT LASER <invert> <n> <r1..rn> <fov> <maxrange> <res> <mode> ... <ts>
std::expected<LaserScan, std::string> parse_laser(const std::vector<std::string>& t) {
  int n = -1;
  size_t idx = 2;  // n 所在的下标
  if (auto v = parse_double(t[2]); v && v.value() > 0 && v.value() < 5000 &&
                                  v.value() == std::floor(v.value())) {
    n = static_cast<int>(v.value());
  } else if (t.size() > 3) {
    if (auto v2 = parse_double(t[3]); v2 && v2.value() > 0 && v2.value() < 5000 &&
                                      v2.value() == std::floor(v2.value())) {
      n = static_cast<int>(v2.value());
      idx = 3;
    }
  }
  if (n <= 0) return std::unexpected("LASER 行读不出点数");

  const size_t fov_i = idx + 1 + static_cast<size_t>(n);
  if (t.size() < fov_i + 5) return std::unexpected("LASER 行太短");

  auto fov = parse_double(t[fov_i]);
  auto maxr = parse_double(t[fov_i + 1]);
  auto res = parse_double(t[fov_i + 2]);
  if (!fov || !maxr || !res) return std::unexpected("LASER 参数不是数字");
  if (*fov <= 0.1 || *fov > 2.0 * kPi + 0.2) return std::unexpected("LASER fov 异常");
  if (*maxr < 0.5 || *maxr > 100.0) return std::unexpected("LASER maxrange 异常");

  LaserScan scan;
  scan.stamp = parse_double(t.back()).value_or(0.0);
  const double angle_min = -*fov / 2.0;
  const double inc = n > 1 ? *fov / static_cast<double>(n - 1) : 0.0;
  for (int i = 0; i < n; ++i) {
    const double r = parse_double(t[idx + 1 + static_cast<size_t>(i)]).value_or(0.0);
    if (r <= 0.01 || r >= *maxr) continue;  // 无回波/超量程
    const double ang = angle_min + inc * i;
    scan.points.push_back(Vec2{r * std::cos(ang), r * std::sin(ang)});
  }
  return scan;
}

Pose2 lerp_pose(const Pose2& a, const Pose2& b, double alpha) {
  return {a.x + alpha * (b.x - a.x), a.y + alpha * (b.y - a.y),
          wrap_angle(a.theta + alpha * wrap_angle(b.theta - a.theta))};
}

}  // namespace

std::expected<CarmenLog, std::string> read_carmen_log(const std::string& path) {
  std::ifstream f(path);
  if (!f) return std::unexpected("打不开日志文件: " + path);

  CarmenLog log;
  // 与 scans 对齐：FLASER 直接带位姿，ROBOT LASER 需要事后插值补上
  std::vector<std::optional<Pose2>> odom_per_scan;
  std::string line;
  while (std::getline(f, line)) {
    const auto t = tokenize(line);
    if (t.empty()) continue;

    if (t[0] == "ODOM") {
      if (t.size() < 8) {
        ++log.skipped_lines;
        continue;
      }
      const auto x = parse_double(t[1]);
      const auto y = parse_double(t[2]);
      const auto th = parse_double(t[3]);
      // ODOM x y th tv rv accel ipc_ts host logger_ts —— 时间戳在第 7 列
      std::expected<double, std::string> ts = parse_double(t[7]);
      if (!ts) ts = parse_double(t.back());
      if (!x || !y || !th || !ts) {
        ++log.skipped_lines;
        continue;
      }
      log.odom.emplace_back(*ts, Pose2{*x, *y, *th});
    } else if (t[0] == "FLASER" && t.size() > 2) {
      auto parsed = parse_flaser(t);
      if (!parsed) {
        ++log.skipped_lines;
        continue;
      }
      auto& [scan, odom] = *parsed;
      log.scans.push_back(std::move(scan));
      odom_per_scan.emplace_back(odom);  // FLASER 内嵌位姿，直接用
    } else if (t[0] == "ROBOT" && t.size() > 2 && t[1] == "LASER") {
      auto scan = parse_laser(t);
      if (!scan) {
        ++log.skipped_lines;
        continue;
      }
      log.scans.push_back(std::move(*scan));
      odom_per_scan.emplace_back(std::nullopt);
    }
    // 其余行（FLATLAND / PARAM / COMMENT ...）静默忽略
  }

  // ---------- 装配：没有内嵌位姿的扫描，用相邻 ODOM 线性插值补齐 ----------
  if (log.odom.empty()) {
    if (std::any_of(odom_per_scan.begin(), odom_per_scan.end(),
                    [](const auto& o) { return !o.has_value(); })) {
      return std::unexpected("日志里有 ROBOT LASER 但没有 ODOM 记录");
    }
    log.scan_odom.reserve(log.scans.size());
    for (auto& o : odom_per_scan) log.scan_odom.push_back(*o);
    return log;
  }
  log.scan_odom.reserve(log.scans.size());
  for (size_t i = 0; i < log.scans.size(); ++i) {
    if (odom_per_scan[i]) {
      log.scan_odom.push_back(*odom_per_scan[i]);
      continue;
    }
    const double ts = log.scans[i].stamp;
    const auto it = std::lower_bound(
        log.odom.begin(), log.odom.end(), ts,
        [](const auto& o, double v) { return o.first < v; });
    if (it == log.odom.begin()) {
      log.scan_odom.push_back(log.odom.front().second);
    } else if (it == log.odom.end()) {
      log.scan_odom.push_back(log.odom.back().second);
    } else {
      const auto& [t0, p0] = *std::prev(it);
      const auto& [t1, p1] = *it;
      const double alpha =
          t1 > t0 ? std::clamp((ts - t0) / (t1 - t0), 0.0, 1.0) : 0.0;
      log.scan_odom.push_back(lerp_pose(p0, p1, alpha));
    }
  }
  return log;
}

}  // namespace mini_slam::app
