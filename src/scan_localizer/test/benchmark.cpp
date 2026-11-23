// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// scan_localizer 基准测试：算法组件 × 场景 的系统对比（E1–E10）
//
// E1 跟踪精度（小扰动）      E2 捕获范围（大初始误差，跨匹配器配对）
// E3 传感器噪声鲁棒性        E4 动态障碍鲁棒性（真实 DynBox 遮挡验证）
// E5 长走廊严格退化 + EKF    E6 扫描密度（含先验扰动与测距噪声）
// E7 CPU 单帧耗时汇总        E8 全局重定位（correlative vs PF，公平计时）
// E9 轨迹级架构对比（104m 环路 + 人群遮挡 + CPU 计时）
// E10 IMU 陀螺航向融合（含零偏与随机游走，验证 4D EKF 在线零偏估计）
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "scan_localizer/localizer_core.hpp"
#include "scan_localizer/matchers.hpp"
#include "scan_localizer/particle_filter.hpp"
#include "scan_localizer/sim_world.hpp"

using sloc::CoreConfig;
using sloc::Cov3;
using sloc::global_correlative;
using sloc::kPi;
using sloc::LocalizerCore;
using sloc::LocState;
using sloc::make_matcher;
using sloc::MapField;
using sloc::Matcher;
using sloc::MatchOptions;
using sloc::MatchResult;
using sloc::OccupancyGrid;
using sloc::ParticleFilter;
using sloc::PfOptions;
using sloc::Pose2;
using sloc::Scan;
using sloc::wrap_angle;
namespace sim = sloc::sim;
using Clock = std::chrono::steady_clock;

namespace
{

struct Stat
{
  double median{0.0};
  double p90{0.0};
  double mean{0.0};
};

Stat stats(std::vector<double> v)
{
  Stat s;
  if (v.empty()) {return s;}
  std::sort(v.begin(), v.end());
  s.median = v[v.size() / 2];
  const std::size_t idx90 = std::min(
    v.size() - 1, static_cast<std::size_t>(v.size() * 0.9));
  s.p90 = v[idx90];
  double sum = 0.0;
  for (double x : v) {
    sum += x;
  }
  s.mean = sum / static_cast<double>(v.size());
  return s;
}

double ms_of(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct Row
{
  std::string name;
  double pos_med{0.0};
  double pos_p90{0.0};
  double ang_med{0.0};
  double t_med{0.0};
  double t_p90{0.0};
  double t_mean{0.0};
  double succ{0.0};
};

void print_table(
  const std::string & title,
  const std::vector<Row> & rows)
{
  std::cout << "\n== " << title << " ==\n";
  std::cout << std::left << std::setw(20) << "matcher"
            << std::right << std::setw(11) << "pos_med"
            << std::setw(11) << "pos_p90"
            << std::setw(14) << "ang_med(deg)"
            << std::setw(11) << "ms_med"
            << std::setw(10) << "succ%" << "\n";
  for (const auto & r : rows) {
    std::cout << std::left << std::setw(20) << r.name << std::right
              << std::fixed << std::setprecision(3)
              << std::setw(11) << r.pos_med
              << std::setw(11) << r.pos_p90
              << std::setw(14) << r.ang_med * 180.0 / kPi
              << std::setw(11) << r.t_med
              << std::setprecision(1)
              << std::setw(10) << r.succ * 100.0 << "\n";
  }
}

Row run_trials(
  const std::string & name, const MapField & field,
  const Matcher & m, const MatchOptions & opt,
  const std::vector<std::pair<Pose2, Scan>> & data,
  const std::vector<Pose2> & priors,
  double succ_pos = 0.05,
  double succ_ang = 1.0 * kPi / 180.0)
{
  Row r;
  r.name = name;
  std::vector<double> pe, ae, tt;
  pe.reserve(data.size());
  ae.reserve(data.size());
  tt.reserve(data.size());
  int ok = 0;
  for (std::size_t i = 0; i < data.size(); ++i) {
    const auto t0 = Clock::now();
    const MatchResult res = m.match(field, priors[i], data[i].second, opt);
    const double ms = ms_of(t0);
    const double e = std::hypot(
      res.pose.x - data[i].first.x, res.pose.y - data[i].first.y);
    const double a = std::abs(wrap_angle(res.pose.theta - data[i].first.theta));
    pe.push_back(e);
    ae.push_back(a);
    tt.push_back(ms);
    if (e < succ_pos && a < succ_ang) {++ok;}
  }
  const auto s1 = stats(pe);
  const auto s2 = stats(ae);
  const auto s3 = stats(tt);
  r.pos_med = s1.median;
  r.pos_p90 = s1.p90;
  r.ang_med = s2.median;
  r.t_med = s3.median;
  r.t_p90 = s3.p90;
  r.t_mean = s3.mean;
  r.succ = data.empty() ? 0.0 :
    static_cast<double>(ok) / static_cast<double>(data.size());
  return r;
}

bool is_pose_clear(
  const OccupancyGrid & grid, double x, double y, double clearance_m)
{
  const auto g = grid.worldToGridIndex(x, y);
  const int r = static_cast<int>(std::ceil(clearance_m / grid.resolution));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r * r) {continue;}
      const int ix = g.x() + dx;
      const int iy = g.y() + dy;
      if (!grid.inside(ix, iy) ||
        grid.cells[static_cast<std::size_t>(iy) *
        static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(ix)] != 0)
      {
        return false;
      }
    }
  }
  return true;
}

void run_e9_trajectory(
  const OccupancyGrid & grid,
  const std::shared_ptr<MapField> & field,
  const std::shared_ptr<Matcher> & comp,
  const sim::ScanSimConfig & sim_cfg,
  std::mt19937 & rng)
{
  const auto traj = sim::make_warehouse_trajectory(2, 0.25);
  const double total_len = 0.25 * static_cast<double>(traj.size() - 1);

  // 人群遮挡段：k ∈ [150, 190] 且每 4 步一帧扫描 → 10 帧遮挡
  auto crowd_at = [&](std::size_t k) -> std::vector<sim::DynBox> {
      if (k < 150 || k > 190) {return {};}
      const auto & p = traj[k];
      return {
      {p.x + 1.5 * std::cos(p.theta),
        p.y + 1.5 * std::sin(p.theta), 0.35, 0.35},
      {p.x + 2.8 * std::cos(p.theta + 0.4),
        p.y + 2.8 * std::sin(p.theta + 0.4), 0.35, 0.35}};
    };

  // 预生成配对的滑移 odom 与扫描序列，确保架构 A / B 输入完全一致
  std::normal_distribution<double> gn_xy(0.0, 0.005);
  std::normal_distribution<double> gn_th(0.0, 0.002);
  std::vector<Pose2> odom_deltas;
  std::vector<Pose2> odom_poses;
  std::vector<Scan> scans;
  odom_deltas.reserve(traj.size());
  odom_poses.reserve(traj.size());
  scans.resize(traj.size());
  Pose2 odom_acc = traj.front();
  odom_deltas.push_back(Pose2{});
  odom_poses.push_back(odom_acc);
  int crowd_scan_frames = 0;
  for (std::size_t k = 1; k < traj.size(); ++k) {
    const Pose2 prev = traj[k - 1];
    const Pose2 gt = traj[k];
    const double c = std::cos(prev.theta);
    const double s = std::sin(prev.theta);
    const double mx = gt.x - prev.x;
    const double my = gt.y - prev.y;
    const Pose2 d_local(
      c * mx + s * my, -s * mx + c * my, wrap_angle(gt.theta - prev.theta));
    const Pose2 d_odom(
      d_local.x * 1.02 + gn_xy(rng),
      d_local.y * 1.02 + gn_xy(rng),
      d_local.theta * 1.01 + gn_th(rng));
    odom_acc = odom_acc * d_odom;
    odom_deltas.push_back(d_odom);
    odom_poses.push_back(odom_acc);
    if (k % 4 == 0) {
      const auto dyn = crowd_at(k);
      if (!dyn.empty()) {++crowd_scan_frames;}
      scans[k] = sim::make_scan(grid, gt, sim_cfg, rng, dyn);
    }
  }

  // 架构 A / B 共享相同的初始位姿扰动（公平对齐初值）
  std::normal_distribution<double> init_xy(0.0, 0.03);
  std::normal_distribution<double> init_th(0.0, 0.01);
  const Pose2 init_pose(
    traj.front().x + init_xy(rng),
    traj.front().y + init_xy(rng),
    wrap_angle(traj.front().theta + init_th(rng)));

  std::cout << "\n== E9 trajectory tracking (" << std::fixed
            << std::setprecision(0) << total_len
            << "m loop, 2% odom slip, " << crowd_scan_frames
            << " crowd scan frames) ==\n";

  // ---- 架构 A：LocalizerCore + composed（共用 MapField sigma=0.20）----
  {
    CoreConfig cc;
    cc.match_opt.window_lin = 0.25;
    LocalizerCore core(cc);
    core.set_map(field);
    core.set_matcher(comp);
    core.on_odom(odom_poses.front());
    core.set_initial_pose(init_pose);
    double err_sum = 0.0;
    double err_max = 0.0;
    double cpu_ms_sum = 0.0;
    int n_acc = 0;
    int n_total = 0;
    for (std::size_t k = 1; k < traj.size(); ++k) {
      const auto t0 = Clock::now();
      core.on_odom(odom_poses[k]);
      if (k % 4 == 0) {
        const auto outp = core.on_scan(scans[k]);
        cpu_ms_sum += ms_of(t0);
        const double e = std::hypot(
          outp.pose.x - traj[k].x, outp.pose.y - traj[k].y);
        err_sum += e;
        err_max = std::max(err_max, e);
        ++n_total;
        if (outp.accepted) {++n_acc;}
      } else {
        cpu_ms_sum += ms_of(t0);
      }
    }
    std::cout << std::fixed << std::setprecision(4)
              << "A core+composed   : mean_err " << err_sum / n_total
              << " m, max_err " << err_max << " m, accept "
              << std::setprecision(1) << 100.0 * n_acc / n_total
              << "%, cpu " << std::setprecision(2)
              << cpu_ms_sum / n_total << " ms/scan (total "
              << cpu_ms_sum << " ms)\n";
  }

  // ---- 架构 B：粒子滤波（同初值、同 MapField sigma=0.20）----
  {
    PfOptions po;
    po.n_particles = 2000;
    po.alpha_trans_trans = 0.02;
    po.alpha_rot_rot = 0.02;
    po.alpha_rot_trans = 0.02;
    po.alpha_trans_rot = 0.02;
    ParticleFilter pf(po);
    pf.init_gaussian(init_pose, 0.05, 0.02, rng);
    double err_sum = 0.0;
    double err_max = 0.0;
    double cpu_ms_sum = 0.0;
    int n_total = 0;
    Pose2 accum_delta{};
    for (std::size_t k = 1; k < traj.size(); ++k) {
      accum_delta = accum_delta * odom_deltas[k];
      if (k % 4 == 0) {
        const auto t0 = Clock::now();
        pf.predict(accum_delta, rng);
        accum_delta = Pose2{};
        pf.update(*field, scans[k], rng);
        Pose2 mean;
        Cov3 cov;
        pf.estimate(mean, cov);
        cpu_ms_sum += ms_of(t0);
        const double e = std::hypot(mean.x - traj[k].x, mean.y - traj[k].y);
        err_sum += e;
        err_max = std::max(err_max, e);
        ++n_total;
      }
    }
    std::cout << std::fixed << std::setprecision(4)
              << "B particle_filter : mean_err " << err_sum / n_total
              << " m, max_err " << err_max << " m (" << po.n_particles
              << " particles), cpu " << std::setprecision(2)
              << cpu_ms_sum / n_total << " ms/scan (total "
              << cpu_ms_sum << " ms)\n";
  }
}

void run_e10_imu(
  const OccupancyGrid & grid,
  const std::shared_ptr<MapField> & field,
  const std::shared_ptr<Matcher> & comp,
  const sim::ScanSimConfig & sim_cfg,
  std::mt19937 & rng)
{
  const auto traj = sim::make_warehouse_trajectory(2, 0.25);
  const double total_len = 0.25 * static_cast<double>(traj.size() - 1);
  constexpr double kDt = 0.25;
  constexpr double kInitGyroBias = 0.02;  // 0.02 rad/s (~1.15 deg/s) 消费级零偏

  // 长厚人群段（k ∈ [150, 230]，每 4 步一帧扫描 → 20 帧重度遮挡）
  auto crowd_long = [&](std::size_t k) -> std::vector<sim::DynBox> {
      if (k < 150 || k > 230) {return {};}
      const auto & p = traj[k];
      return {
      {p.x + 1.2 * std::cos(p.theta),
        p.y + 1.2 * std::sin(p.theta), 0.4, 0.4},
      {p.x + 2.2 * std::cos(p.theta + 0.5),
        p.y + 2.2 * std::sin(p.theta + 0.5), 0.4, 0.4},
      {p.x + 2.0 * std::cos(p.theta - 0.5),
        p.y + 2.0 * std::sin(p.theta - 0.5), 0.4, 0.4}};
    };

  int crowd_scan_frames = 0;
  std::vector<Scan> scans(traj.size());
  for (std::size_t k = 1; k < traj.size(); ++k) {
    if (k % 4 == 0) {
      const auto dyn = crowd_long(k);
      if (!dyn.empty()) {++crowd_scan_frames;}
      scans[k] = sim::make_scan(grid, traj[k], sim_cfg, rng, dyn);
    }
  }

  std::cout << "\n== E10 IMU yaw fusion (" << std::fixed
            << std::setprecision(0) << total_len << "m loop, "
            << crowd_scan_frames
            << " crowd scan frames, gyro bias=0.02 rad/s + RW) ==\n";

  std::normal_distribution<double> gn_xy(0.0, 0.005);
  std::normal_distribution<double> gn_th(0.0, 0.003);
  std::normal_distribution<double> gyro_white(0.0, 0.002);
  std::normal_distribution<double> gyro_rw(0.0, 1.0e-4 * std::sqrt(kDt));

  for (int slip_lvl = 0; slip_lvl < 2; ++slip_lvl) {
    const double rot_scale_err = slip_lvl == 0 ? 1.03 : 1.10;
    std::cout << "  [wheel rot slip "
              << (slip_lvl == 0 ? "3% (realistic)" : "10% (carpet)")
              << "]\n";

    // 为当前打滑档位生成配对的 odom 与 gyro 序列
    std::vector<Pose2> odom_poses;
    std::vector<double> gyro_wz;
    odom_poses.reserve(traj.size());
    gyro_wz.reserve(traj.size());
    Pose2 odom_acc = traj.front();
    odom_poses.push_back(odom_acc);
    gyro_wz.push_back(0.0);
    double true_bias = kInitGyroBias;
    for (std::size_t k = 1; k < traj.size(); ++k) {
      const Pose2 prev = traj[k - 1];
      const Pose2 gt = traj[k];
      const double c = std::cos(prev.theta);
      const double s = std::sin(prev.theta);
      const double mx = gt.x - prev.x;
      const double my = gt.y - prev.y;
      const Pose2 d_local(
        c * mx + s * my, -s * mx + c * my, wrap_angle(gt.theta - prev.theta));
      const Pose2 d_odom(
        d_local.x * 1.02 + gn_xy(rng),
        d_local.y * 1.02 + gn_xy(rng),
        d_local.theta * rot_scale_err + gn_th(rng));
      odom_acc = odom_acc * d_odom;
      odom_poses.push_back(odom_acc);
      true_bias += gyro_rw(rng);
      gyro_wz.push_back(d_local.theta / kDt + true_bias + gyro_white(rng));
    }

    for (int variant = 0; variant < 2; ++variant) {
      const bool use_imu = variant == 1;
      CoreConfig cc;
      cc.match_opt.window_lin = 0.25;
      cc.use_imu_yaw = use_imu;
      LocalizerCore core(cc);
      core.set_map(field);
      core.set_matcher(comp);
      core.on_odom(odom_poses.front());
      core.set_initial_pose(traj.front());
      double err_sum = 0.0;
      double err_max = 0.0;
      int n_total = 0;
      int n_acc = 0;
      int n_reject = 0;
      for (std::size_t k = 1; k < traj.size(); ++k) {
        core.on_odom(odom_poses[k]);
        if (use_imu) {
          core.on_imu_gyro(gyro_wz[k], kDt);
        }
        if (k % 4 == 0) {
          const auto o = core.on_scan(scans[k]);
          const double e = std::hypot(
            o.pose.x - traj[k].x, o.pose.y - traj[k].y);
          err_sum += e;
          err_max = std::max(err_max, e);
          ++n_total;
          if (o.accepted) {++n_acc;} else {++n_reject;}
        }
      }
      if (use_imu) {
        std::printf(
          "    B imu_fused  : mean_err %.4f m, max_err %.4f m, accept "
          "%.1f%% (reject %d), est_bg %.4f rad/s (true %.4f)\n",
          err_sum / n_total, err_max, 100.0 * n_acc / n_total,
          n_reject, core.gyro_bias(), true_bias);
      } else {
        std::printf(
          "    A wheel_only : mean_err %.4f m, max_err %.4f m, accept "
          "%.1f%% (reject %d)\n",
          err_sum / n_total, err_max, 100.0 * n_acc / n_total, n_reject);
      }
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const int quick = argc > 1 && std::string(argv[1]) == "--quick" ? 5 : 1;
  std::cout << "scan_localizer benchmark (quick x" << quick << ")\n";

  std::mt19937 rng(42);

  // ================= 地图与场 =================
  const auto t_build = Clock::now();
  const OccupancyGrid grid = sim::make_warehouse(0.05);
  auto field = std::make_shared<MapField>();
  field->build(grid, 0.20);
  std::cout << "warehouse map: " << grid.width << "x" << grid.height
            << " cells, MapField build: " << std::fixed << std::setprecision(4)
            << ms_of(t_build) / 1000.0 << " s\n";

  // 匹配器
  auto corr = make_matcher("correlative");
  auto fgn = make_matcher("field_gn");
  auto icp = make_matcher("icp_p2p");
  auto ndt = make_matcher("ndt", &grid, 0.5);
  auto comp = make_matcher("composed");
  // PF 全局定位专用较尖观测场
  auto field10 = std::make_shared<MapField>();
  field10->build(grid, 0.10);

  MatchOptions opt;
  MatchOptions opt_wide = opt;
  opt_wide.window_lin = 1.6;
  opt_wide.window_ang = 0.55;
  opt_wide.step_lin = 0.08;
  opt_wide.step_ang = 0.03;
  MatchOptions opt_tight = opt;
  opt_tight.trim_dist = 0.20;
  opt_tight.huber_delta = 0.05;

  // ================= 通用数据 =================
  const int N1 = 100 / quick;
  sim::ScanSimConfig sim_cfg;
  auto poses = sim::random_free_poses(grid, N1, rng);

  // ================= E1 跟踪精度 =================
  std::vector<Row> e1_rows;
  {
    std::vector<std::pair<Pose2, Scan>> data;
    std::vector<Pose2> priors;
    std::normal_distribution<double> n2(0.0, 0.02);
    std::normal_distribution<double> a2(0.0, 0.5 * kPi / 180.0);
    for (const auto & p : poses) {
      data.emplace_back(p, sim::make_scan(grid, p, sim_cfg, rng));
      priors.emplace_back(p.x + n2(rng), p.y + n2(rng),
        wrap_angle(p.theta + a2(rng)));
    }
    e1_rows = {
      run_trials("correlative", *field, *corr, opt, data, priors),
      run_trials("field_gn", *field, *fgn, opt, data, priors),
      run_trials("icp_p2p", *field, *icp, opt, data, priors),
      run_trials("ndt", *field, *ndt, opt, data, priors),
      run_trials("composed", *field, *comp, opt, data, priors)};
    print_table(
      "E1 tracking accuracy (prior err ~2cm/0.5deg, clean scan)", e1_rows);
  }

  // ================= E2 捕获范围（跨匹配器严格配对）=================
  {
    struct Lvl { double pos, ang; };
    const Lvl lvls[] = {
      {0.05, 2.0}, {0.15, 5.0}, {0.40, 10.0}, {0.80, 18.0}, {1.50, 30.0}};
    constexpr int kNumLvls = 5;
    const int N2 = 50 / quick;

    // 预生成每一档的配对试验（均匀随机方向 + 正负对称角度扰动）
    std::uniform_real_distribution<double> u_dir(-kPi, kPi);
    std::uniform_int_distribution<int> u_sign(0, 1);
    std::vector<std::vector<std::pair<Pose2, Scan>>> lvl_data(kNumLvls);
    std::vector<std::vector<Pose2>> lvl_priors(kNumLvls);
    for (int li = 0; li < kNumLvls; ++li) {
      lvl_data[li].reserve(static_cast<std::size_t>(N2));
      lvl_priors[li].reserve(static_cast<std::size_t>(N2));
      for (int i = 0; i < N2; ++i) {
        const Pose2 gt = poses[static_cast<std::size_t>(i) % poses.size()];
        lvl_data[li].emplace_back(gt, sim::make_scan(grid, gt, sim_cfg, rng));
        const double dir = u_dir(rng);
        const double sgn = u_sign(rng) == 0 ? -1.0 : 1.0;
        lvl_priors[li].emplace_back(
          gt.x + lvls[li].pos * std::cos(dir),
          gt.y + lvls[li].pos * std::sin(dir),
          wrap_angle(gt.theta + sgn * lvls[li].ang * kPi / 180.0));
      }
    }

    std::cout << "\n== E2 capture range (paired trials, success% by initial error) ==\n";
    std::cout << std::left << std::setw(20) << "matcher";
    for (const auto & l : lvls) {
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%.2fm", l.pos);
      std::cout << std::right << std::setw(10) << buf;
    }
    std::cout << std::right << std::setw(12) << "ms@large" << "\n";

    struct Cand { std::string n; Matcher * m; const MatchOptions * o; };
    std::vector<Cand> cands = {
      {"correlative", corr.get(), &opt},
      {"correlative_wide", corr.get(), &opt_wide},
      {"field_gn", fgn.get(), &opt},
      {"icp_p2p", icp.get(), &opt},
      {"ndt", ndt.get(), &opt},
      {"composed", comp.get(), &opt},
      {"composed_wide", comp.get(), &opt_wide}};
    for (const auto & c : cands) {
      std::cout << std::left << std::setw(20) << c.n;
      std::vector<double> ms_at_large;
      for (int li = 0; li < kNumLvls; ++li) {
        int ok = 0;
        for (int i = 0; i < N2; ++i) {
          const Pose2 & gt = lvl_data[li][static_cast<std::size_t>(i)].first;
          const Scan & scan = lvl_data[li][static_cast<std::size_t>(i)].second;
          const Pose2 & prior = lvl_priors[li][static_cast<std::size_t>(i)];
          const auto t1 = Clock::now();
          const auto res = c.m->match(*field, prior, scan, *c.o);
          if (lvls[li].pos >= 0.8) {ms_at_large.push_back(ms_of(t1));}
          const double e = std::hypot(res.pose.x - gt.x, res.pose.y - gt.y);
          const double a = std::abs(wrap_angle(res.pose.theta - gt.theta));
          if (e < 0.05 && a < 1.0 * kPi / 180.0) {++ok;}
        }
        std::cout << std::right << std::setw(10) << std::fixed
                  << std::setprecision(1) << 100.0 * ok / N2;
      }
      std::cout << std::right << std::setw(12) << std::setprecision(1)
                << stats(ms_at_large).median << "\n";
    }
  }

  // ================= E3 噪声鲁棒性（跨匹配器严格配对）=================
  {
    const double sigmas[] = {0.0, 0.01, 0.03, 0.05};
    constexpr int kNumSigmas = 4;
    std::normal_distribution<double> n2(0.0, 0.02);
    std::normal_distribution<double> a2(0.0, 0.5 * kPi / 180.0);

    std::vector<std::vector<std::pair<Pose2, Scan>>> s_data(kNumSigmas);
    std::vector<std::vector<Pose2>> s_priors(kNumSigmas);
    for (int si = 0; si < kNumSigmas; ++si) {
      sim::ScanSimConfig cfg = sim_cfg;
      cfg.noise_std = sigmas[si];
      for (const auto & p : poses) {
        s_data[si].emplace_back(p, sim::make_scan(grid, p, cfg, rng));
        s_priors[si].emplace_back(
          p.x + n2(rng), p.y + n2(rng), wrap_angle(p.theta + a2(rng)));
      }
    }

    std::cout << "\n== E3 range noise (paired trials, median pos err m, prior 2cm) ==\n";
    std::cout << std::left << std::setw(20) << "matcher";
    for (double s : sigmas) {
      char buf[16];
      std::snprintf(buf, sizeof(buf), "sigma=%.2f", s);
      std::cout << std::right << std::setw(12) << buf;
    }
    std::cout << "\n";

    struct Cand { std::string n; Matcher * m; };
    for (const auto & c : std::vector<Cand>{
      {"correlative", corr.get()}, {"field_gn", fgn.get()},
      {"icp_p2p", icp.get()}, {"ndt", ndt.get()},
      {"composed", comp.get()}})
    {
      std::cout << std::left << std::setw(20) << c.n;
      for (int si = 0; si < kNumSigmas; ++si) {
        std::vector<double> pe;
        pe.reserve(s_data[si].size());
        for (std::size_t i = 0; i < s_data[si].size(); ++i) {
          const auto res = c.m->match(
            *field, s_priors[si][i], s_data[si][i].second, opt);
          pe.push_back(std::hypot(
            res.pose.x - s_data[si][i].first.x,
            res.pose.y - s_data[si][i].first.y));
        }
        std::cout << std::right << std::setw(12) << std::fixed
                  << std::setprecision(3) << stats(pe).median;
      }
      std::cout << "\n";
    }
  }

  // ================= E4 动态障碍（真实注入校验）=================
  {
    std::vector<std::pair<Pose2, Scan>> data;
    std::vector<Pose2> priors;
    std::normal_distribution<double> n2(0.0, 0.02);
    std::normal_distribution<double> a2(0.0, 0.5 * kPi / 180.0);
    std::uniform_real_distribution<double> u_dir(-kPi, kPi);
    std::uniform_real_distribution<double> u_dist(1.2, 2.8);
    int total_boxes = 0;
    int min_boxes_per_trial = std::numeric_limits<int>::max();
    for (const auto & p : poses) {
      std::vector<sim::DynBox> dyn;
      for (int b = 0; b < 2; ++b) {
        for (int attempt = 0; attempt < 32; ++attempt) {
          const double dir = u_dir(rng);
          const double d = u_dist(rng);
          const double bx = p.x + d * std::cos(dir);
          const double by = p.y + d * std::sin(dir);
          if (bx > 1.0 && bx < 39.0 && by > 1.0 && by < 29.0) {
            dyn.push_back({bx, by, 0.35, 0.35});
            break;
          }
        }
      }
      const int nb = static_cast<int>(dyn.size());
      total_boxes += nb;
      min_boxes_per_trial = std::min(min_boxes_per_trial, nb);
      data.emplace_back(p, sim::make_scan(grid, p, sim_cfg, rng, dyn));
      priors.emplace_back(
        p.x + n2(rng), p.y + n2(rng), wrap_angle(p.theta + a2(rng)));
    }
    if (min_boxes_per_trial <= 0) {
      std::cerr << "E4 ERROR: failed to inject DynBox on at least one trial!\n";
      return 1;
    }
    std::cout << "\n== E4 dynamic obstacles (injected " << total_boxes
              << " boxes across " << poses.size()
              << " trials, min=" << min_boxes_per_trial << "/trial) ==\n";
    print_table(
      "E4 dynamic obstacles (2 boxes 0.7m near robot)",
      {run_trials("correlative", *field, *corr, opt, data, priors),
        run_trials("field_gn", *field, *fgn, opt, data, priors),
        run_trials("field_gn_tight", *field, *fgn, opt_tight, data, priors),
        run_trials("ndt", *field, *ndt, opt, data, priors),
        run_trials("composed", *field, *comp, opt, data, priors),
        run_trials("composed_tight", *field, *comp, opt_tight, data, priors)});
  }

  // ================= E5 走廊严格退化 + EKF 融合 =================
  {
    const OccupancyGrid cgrid = sim::make_corridor(0.05);
    auto cfield = std::make_shared<MapField>();
    cfield->build(cgrid, 0.20);
    auto cndt = make_matcher("ndt", &cgrid, 0.5);
    const int N5 = 50 / quick;
    // 严格中段 x ∈ [13.0, 17.0]，距两端墙均 >= 12.9m > range_max=12.0m
    std::uniform_real_distribution<double> ux(13.0, 17.0);
    std::uniform_real_distribution<double> uy(1.2, 1.8);
    std::uniform_real_distribution<double> ut(-0.3, 0.3);
    std::vector<Pose2> cpos;
    cpos.reserve(static_cast<std::size_t>(N5));
    for (int i = 0; i < N5; ++i) {
      cpos.emplace_back(ux(rng), uy(rng), ut(rng));
    }
    std::vector<std::pair<Pose2, Scan>> data;
    std::vector<Pose2> priors;
    std::normal_distribution<double> n2(0.0, 0.02);
    std::normal_distribution<double> a2(0.0, 0.5 * kPi / 180.0);
    for (const auto & p : cpos) {
      data.emplace_back(p, sim::make_scan(cgrid, p, sim_cfg, rng));
      priors.emplace_back(
        p.x + n2(rng), p.y + n2(rng), wrap_angle(p.theta + a2(rng)));
    }
    std::cout << "\n== E5 corridor degeneracy (30x3m, strictly x in [13,17]) ==\n";
    std::cout << std::left << std::setw(20) << "matcher" << std::right
              << std::setw(14) << "err_along(m)"
              << std::setw(14) << "err_across(m)"
              << std::setw(15) << "cov_along_med"
              << std::setw(15) << "cov_across_med" << "\n";
    struct Cand { std::string n; Matcher * m; };
    for (const auto & c : std::vector<Cand>{
      {"correlative", corr.get()}, {"field_gn", fgn.get()},
      {"icp_p2p", icp.get()}, {"ndt", cndt.get()},
      {"composed", comp.get()}})
    {
      std::vector<double> ea, ec, cv_a, cv_c;
      for (std::size_t i = 0; i < data.size(); ++i) {
        const auto res = c.m->match(*cfield, priors[i], data[i].second, opt);
        ea.push_back(std::abs(res.pose.x - data[i].first.x));
        ec.push_back(std::abs(res.pose.y - data[i].first.y));
        cv_a.push_back(res.covariance(0, 0));
        cv_c.push_back(res.covariance(1, 1));
      }
      std::cout << std::left << std::setw(20) << c.n << std::right
                << std::fixed << std::setprecision(4)
                << std::setw(14) << stats(ea).median
                << std::setw(14) << stats(ec).median
                << std::setprecision(3)
                << std::setw(15) << stats(cv_a).median
                << std::setprecision(5)
                << std::setw(15) << stats(cv_c).median << "\n";
    }

    // 各向异性 EKF 在退化走廊中行走 4m（x: 13->17, 2% 轮滑）
    CoreConfig cc;
    LocalizerCore core(cc);
    core.set_map(cfield);
    core.set_matcher(comp);
    Pose2 gt(13.0, 1.5, 0.0);
    Pose2 odom = gt;
    core.on_odom(odom);
    core.set_initial_pose(gt);
    double max_along = 0.0;
    double max_across = 0.0;
    int accepted = 0;
    constexpr int kSteps = 16;
    for (int s = 0; s < kSteps; ++s) {
      gt.x += 0.25;
      odom = odom * Pose2(0.25 * 1.02, 0.0, 0.0);
      core.on_odom(odom);
      const auto scan = sim::make_scan(cgrid, gt, sim_cfg, rng);
      const auto outp = core.on_scan(scan);
      if (outp.accepted) {++accepted;}
      max_along = std::max(max_along, std::abs(outp.pose.x - gt.x));
      max_across = std::max(max_across, std::abs(outp.pose.y - gt.y));
    }
    const Cov3 final_cov = core.last_cov();
    std::cout << std::fixed << std::setprecision(4)
              << "  EKF corridor 4m  : max_along " << max_along
              << " m, max_across " << max_across
              << " m, cov_along " << final_cov(0, 0)
              << ", cov_across " << final_cov(1, 1)
              << ", accept " << accepted << "/" << kSteps << "\n";
  }

  // ================= E6 扫描密度（含非零先验扰动与真实测距噪声）=================
  {
    std::cout << "\n== E6 scan density (720/360/180/90 beams, prior 2cm, noise 1cm) ==\n";
    std::cout << std::left << std::setw(20) << "matcher" << std::right
              << std::setw(10) << "beams"
              << std::setw(14) << "pos_med(m)"
              << std::setw(12) << "ms_med" << "\n";
    const int beams[] = {720, 360, 180, 90};
    constexpr int kNumBeams = 4;
    std::normal_distribution<double> n2(0.0, 0.02);
    std::normal_distribution<double> a2(0.0, 0.5 * kPi / 180.0);

    std::vector<std::vector<std::pair<Pose2, Scan>>> b_data(kNumBeams);
    std::vector<std::vector<Pose2>> b_priors(kNumBeams);
    for (int bi = 0; bi < kNumBeams; ++bi) {
      sim::ScanSimConfig cfg = sim_cfg;
      cfg.subsample = 720 / beams[bi];
      cfg.noise_std = 0.01;
      for (const auto & p : poses) {
        b_data[bi].emplace_back(p, sim::make_scan(grid, p, cfg, rng));
        b_priors[bi].emplace_back(
          p.x + n2(rng), p.y + n2(rng), wrap_angle(p.theta + a2(rng)));
      }
    }

    struct Cand { std::string n; Matcher * m; };
    for (const auto & c : std::vector<Cand>{
      {"correlative", corr.get()}, {"field_gn", fgn.get()},
      {"composed", comp.get()}})
    {
      for (int bi = 0; bi < kNumBeams; ++bi) {
        std::vector<double> pe, tt;
        for (std::size_t i = 0; i < b_data[bi].size(); ++i) {
          const auto t1 = Clock::now();
          const auto res = c.m->match(
            *field, b_priors[bi][i], b_data[bi][i].second, opt);
          tt.push_back(ms_of(t1));
          pe.push_back(std::hypot(
            res.pose.x - b_data[bi][i].first.x,
            res.pose.y - b_data[bi][i].first.y));
        }
        std::cout << std::left << std::setw(20) << c.n << std::right
                  << std::setw(10) << beams[bi]
                  << std::fixed << std::setprecision(4)
                  << std::setw(14) << stats(pe).median
                  << std::setprecision(3)
                  << std::setw(12) << stats(tt).median << "\n";
      }
    }
  }

  // ================= E7 单帧 CPU 耗时汇总 =================
  {
    std::cout << "\n== E7 per-matcher CPU latency summary (720 beams, E1 dataset) ==\n";
    std::cout << std::left << std::setw(20) << "matcher" << std::right
              << std::setw(12) << "ms_med"
              << std::setw(12) << "ms_p90"
              << std::setw(12) << "ms_mean" << "\n";
    for (const auto & r : e1_rows) {
      std::cout << std::left << std::setw(20) << r.name << std::right
                << std::fixed << std::setprecision(3)
                << std::setw(12) << r.t_med
                << std::setw(12) << r.t_p90
                << std::setw(12) << r.t_mean << "\n";
    }
  }

  // ================= E8 全局定位（公平计时，PF 无碰撞轨迹）=================
  {
    const int N8 = 30 / quick;
    const int n_trials = std::min(N8, static_cast<int>(poses.size()));
    std::cout << "\n== E8 global localization (cold start, " << n_trials
              << " trials) ==\n";

    // correlative global（射线生成在计时外）
    {
      int ok = 0;
      std::vector<double> tt;
      for (int t = 0; t < n_trials; ++t) {
        const auto & p = poses[static_cast<std::size_t>(t)];
        const auto scan = sim::make_scan(grid, p, sim_cfg, rng);
        const auto t1 = Clock::now();
        const auto g = global_correlative(
          *field, scan, 3, 2.0 * kPi / 180.0, 5);
        tt.push_back(ms_of(t1));
        if (g.valid) {
          const double e = std::hypot(g.pose.x - p.x, g.pose.y - p.y);
          const double a = std::abs(wrap_angle(g.pose.theta - p.theta));
          if (e < 0.3 && a < 3.0 * kPi / 180.0) {++ok;}
        }
      }
      std::cout << std::fixed << std::setprecision(1)
                << "global_correlative     : succ " << 100.0 * ok / n_trials
                << "%  median " << std::setprecision(2) << stats(tt).median
                << " ms (n=" << n_trials << ")\n";
    }

    // PF global（10 步 × 0.25m 无碰撞路径，扫描预生成在计时外）
    {
      constexpr int kPfSteps = 10;
      PfOptions po;
      po.n_particles = 20000;
      po.alpha_trans_trans = 0.02;
      po.alpha_rot_rot = 0.02;
      po.alpha_rot_trans = 0.02;
      po.alpha_trans_rot = 0.02;
      std::uniform_real_distribution<double> wob(-0.05, 0.05);
      int ok = 0;
      std::vector<double> tt;
      std::vector<double> final_err;
      for (int t = 0; t < n_trials; ++t) {
        const auto & p0 = poses[static_cast<std::size_t>(t)];
        // 预先生成 10 步无碰撞运动轨迹与扫描（排除在计时器外）
        std::vector<Pose2> step_odom;
        std::vector<Scan> step_scans;
        step_odom.reserve(kPfSteps);
        step_scans.reserve(kPfSteps);
        Pose2 gt = p0;
        Pose2 prev_gt = p0;
        for (int k = 0; k < kPfSteps; ++k) {
          double heading = wrap_angle(gt.theta + wob(rng));
          double nx = gt.x + 0.25 * std::cos(heading);
          double ny = gt.y + 0.25 * std::sin(heading);
          if (!is_pose_clear(grid, nx, ny, 0.35)) {
            for (int tries = 1; tries <= 16; ++tries) {
              const double cand_h = wrap_angle(
                gt.theta + static_cast<double>(tries) * (kPi / 8.0));
              const double cx = gt.x + 0.25 * std::cos(cand_h);
              const double cy = gt.y + 0.25 * std::sin(cand_h);
              if (is_pose_clear(grid, cx, cy, 0.35)) {
                heading = cand_h;
                nx = cx;
                ny = cy;
                break;
              }
            }
          }
          gt = Pose2(nx, ny, heading);
          const double c = std::cos(prev_gt.theta);
          const double s = std::sin(prev_gt.theta);
          const double dx = gt.x - prev_gt.x;
          const double dy = gt.y - prev_gt.y;
          const Pose2 d_true(
            c * dx + s * dy, -s * dx + c * dy,
            wrap_angle(gt.theta - prev_gt.theta));
          step_odom.emplace_back(
            d_true.x * 1.02, d_true.y * 1.02, d_true.theta * 1.02 + 0.002);
          step_scans.push_back(sim::make_scan(grid, gt, sim_cfg, rng));
          prev_gt = gt;
        }

        // 仅计时 PF 算法本身
        ParticleFilter pf(po);
        const auto t1 = Clock::now();
        pf.init_global(grid, rng);
        for (int k = 0; k < kPfSteps; ++k) {
          pf.predict(step_odom[static_cast<std::size_t>(k)], rng);
          pf.update(*field10, step_scans[static_cast<std::size_t>(k)], rng);
        }
        Pose2 mean;
        Cov3 cov;
        pf.estimate(mean, cov);
        tt.push_back(ms_of(t1));

        const double e = std::hypot(mean.x - gt.x, mean.y - gt.y);
        final_err.push_back(e);
        const double a = std::abs(wrap_angle(mean.theta - gt.theta));
        if (e < 0.3 && a < 3.0 * kPi / 180.0) {++ok;}
      }
      std::cout << std::fixed << std::setprecision(1)
                << "pf_global(20000,10步)  : succ " << 100.0 * ok / n_trials
                << "%  median " << std::setprecision(2) << stats(tt).median
                << " ms  err_med " << std::setprecision(3)
                << stats(final_err).median << " m\n";
    }
  }

  // ================= E9 轨迹级架构对比 =================
  run_e9_trajectory(grid, field, comp, sim_cfg, rng);

  // ================= E10 IMU 融合收益 =================
  run_e10_imu(grid, field, comp, sim_cfg, rng);

  std::cout << "\nbenchmark done.\n";
  return 0;
}
