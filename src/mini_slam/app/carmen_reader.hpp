#pragma once
// ============================================================================
// CARMEN 日志读取器（Intel Lab 数据集的格式）。
// 每帧 ROBOT LASER 与时间上夹着它的两条 ODOM 做插值，得到扫描时刻的里程计位姿。
// ============================================================================
#include <expected>
#include <string>
#include <utility>
#include <vector>

#include "mini_slam/types.hpp"

namespace mini_slam::app {

struct CarmenLog {
  std::vector<std::pair<double, Pose2>> odom;  // (时间戳, 里程计位姿)
  std::vector<LaserScan> scans;                // 时间戳在 scan.stamp
  std::vector<Pose2> scan_odom;                // 每帧扫描时刻的插值里程计位姿
  int skipped_lines{0};                        // 解析失败的 ODOM/LASER 行数
};

std::expected<CarmenLog, std::string> read_carmen_log(const std::string& path);

}  // namespace mini_slam::app
