// Ch09 hands-on layer: fair paired-experiment sandbox, raw-data recomputation,
// verbatim benchmark.cpp source stepping, and self-deception acceptance checks.
(function () {
  "use strict";

  const SOURCE_FILE = "test/benchmark.cpp";
  const SCHEMA = "ch09-synthetic-v1";
  const PROVENANCE = "synthetic-browser-not-cpp-or-intel";
  const STAGES = [
    {
      title: "1 · 入口：种子、共享地图与匹配器",
      lines: "470-504",
      explain: "main() 用 mt19937(42) 建立全局随机流，先构建仓库和 MapField，再创建候选。共享 field 是公平比较的基础；但单一全局 RNG 仍把输入随机性与代码执行顺序耦合在一起。",
      code: `int main(int argc, char ** argv)
{
  const int quick = argc > 1 && std::string(argv[1]) == "--quick" ? 5 : 1;
  std::cout << "scan_localizer benchmark (quick x" << quick << ")\\n";

  std::mt19937 rng(42);

  // ================= 地图与场 =================
  const auto t_build = Clock::now();
  const OccupancyGrid grid = sim::make_warehouse(0.05);
  auto field = std::make_shared<MapField>();
  field->build(grid, 0.20);
  std::cout << "warehouse map: " << grid.width << "x" << grid.height
            << " cells, MapField build: " << std::fixed << std::setprecision(4)
            << ms_of(t_build) / 1000.0 << " s\\n";

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
  opt_tight.huber_delta = 0.05;`,
      vars: [
        ["随机流", "一个全局 std::mt19937 rng(42)"],
        ["共享资源", "grid / field / sim_cfg / opt 在候选外创建"],
        ["审计问题", "若改变候选顺序，后续消费 RNG 的实验是否仍同输入？"],
      ],
    },
    {
      title: "2 · run_trials：误差、成功判据与耗时",
      lines: "130-169",
      explain: "每次调用都记录平移误差、角度误差和 steady_clock 延迟；成功要求平移与角度同时严格小于阈值。stats() 使用上中位数与固定索引 P90。当前 Row 只保存汇总，逐次 pe/ae/tt 没有落盘。",
      code: `Row run_trials(
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
}`,
      vars: [
        ["主指标", "e < 0.05 m 且 |a| < 1°；两侧阈值相同"],
        ["计时边界", "仅 m.match(...)，不含输入生成"],
        ["缺口", "pe / ae / tt 是局部 vector；当前输出没有逐调用原始行"],
      ],
    },
    {
      title: "3 · E1：先冻结数据，再逐个调用候选",
      lines: "506-531",
      explain: "所有扫描与先验在候选循环之前生成，随后五个 run_trials 复用同一 data/priors。这是配对试验的正确形状：样本生成不属于任何候选。",
      code: `  // ================= 通用数据 =================
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
  }`,
      vars: [
        ["样本单元", "data[i] + priors[i]，五个候选共用"],
        ["顺序", "data / priors 完全生成后才进入候选循环"],
        ["可复核性", "需要额外导出 data 摘要、参数和每次调用结果"],
      ],
    },
    {
      title: "4 · E2：跨档位预生成配对输入，再统一判成功",
      lines: "533-601",
      explain: "E2 在五个初始误差档位上分别预生成扫描和先验，候选只读取这些共享输入。成功阈值固定为 5cm/1°；large 档延迟单独收集。注意 wide 与默认配置代表不同资源/搜索预算，不能把差异全归因于算法名称。",
      code: `  // ================= E2 捕获范围（跨匹配器严格配对）=================
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

    std::cout << "\\n== E2 capture range (paired trials, success% by initial error) ==\\n";
    std::cout << std::left << std::setw(20) << "matcher";
    for (const auto & l : lvls) {
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%.2fm", l.pos);
      std::cout << std::right << std::setw(10) << buf;
    }
    std::cout << std::right << std::setw(12) << "ms@large" << "\\n";

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
                << stats(ms_at_large).median << "\\n";
    }
  }`,
      vars: [
        ["配对范围", "同一档、同一 i 的 scan / prior / GT 被所有候选读取"],
        ["成功判据", "平移 < 0.05 m 且角度 < 1°"],
        ["审计问题", "opt 与 opt_wide 的搜索预算不同；比较时必须声明资源条件"],
      ],
    },
    {
      title: "5 · E4：动态障碍必须证明真的注入",
      lines: "655-701",
      explain: "E4 为每个 trial 尝试放入两个 DynBox，累计 total_boxes 并维护 min_boxes_per_trial。只要出现零注入就返回错误；正常 N1=100 时预期累计 200 箱。这是输入有效性门，不是性能指标。",
      code: `  // ================= E4 动态障碍（真实注入校验）=================
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
      std::cerr << "E4 ERROR: failed to inject DynBox on at least one trial!\\n";
      return 1;
    }
    std::cout << "\\n== E4 dynamic obstacles (injected " << total_boxes
              << " boxes across " << poses.size()
              << " trials, min=" << min_boxes_per_trial << "/trial) ==\\n";
    print_table(
      "E4 dynamic obstacles (2 boxes 0.7m near robot)",
      {run_trials("correlative", *field, *corr, opt, data, priors),
        run_trials("field_gn", *field, *fgn, opt, data, priors),
        run_trials("field_gn_tight", *field, *fgn, opt_tight, data, priors),
        run_trials("ndt", *field, *ndt, opt, data, priors),
        run_trials("composed", *field, *comp, opt, data, priors),
        run_trials("composed_tight", *field, *comp, opt_tight, data, priors)});
  }`,
      vars: [
        ["输入门", "min_boxes_per_trial <= 0 时整个基准失败"],
        ["正常规模", "N1=100，每轮 2 箱 → 预期累计 200 箱"],
        ["教训", "鲁棒性实验先证明扰动存在，再比较算法"],
      ],
    },
    {
      title: "6 · E9：预生成配对 odom / scan，并共享初值",
      lines: "214-256",
      explain: "E9 先一次性生成轨迹、带滑移 odom、扫描和初值扰动，随后两个架构读取同一批数据。这里做对了“输入配对”；但下一步会看到两侧计时边界不同。",
      code: `  // 预生成配对的滑移 odom 与扫描序列，确保架构 A / B 输入完全一致
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
    wrap_angle(traj.front().theta + init_th(rng)));`,
      vars: [
        ["共享输入", "odom_poses / scans / init_pose 只生成一次"],
        ["配对变量", "人群遮挡扫描也预先写入 scans[k]"],
        ["边界", "输入公平不代表输出计时公平"],
      ],
    },
    {
      title: "7 · E9 架构 A：计时包含每个 on_odom",
      lines: "263-300",
      explain: "A 在每一步调用 on_odom，扫描帧再调用 on_scan；两步都把 ms_of(t0) 加入 cpu_ms_sum，但误差和 n_total 只在扫描帧累计。因此 cpu_ms_sum/n_total 是“所有步耗时 / 扫描帧数”，不是每扫描帧算法耗时。",
      code: `  // ---- 架构 A：LocalizerCore + composed（共用 MapField sigma=0.20）----
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
              << cpu_ms_sum << " ms)\\n";
  }`,
      vars: [
        ["计时包含", "每个 k 的 on_odom；扫描帧另含 on_scan"],
        ["误差分母", "仅扫描帧 n_total"],
        ["CPU 分母", "却仍是 n_total；分子包含非扫描帧"],
      ],
    },
    {
      title: "8 · E9 架构 B：只在扫描帧计时",
      lines: "302-340",
      explain: "B 每步先合成 accum_delta，但计时器到扫描帧才启动；predict/update/estimate 在计时内，初值估计在计时外。与上一步 A 的边界不同，当前 CPU 数字不能直接支持“谁更省”。",
      code: `  // ---- 架构 B：粒子滤波（同初值、同 MapField sigma=0.20）----
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
              << cpu_ms_sum << " ms)\\n";
  }`,
      vars: [
        ["计时包含", "扫描帧 predict / update / estimate"],
        ["计时排除", "每步 accum_delta 合成；非扫描帧不入 CPU 分子"],
        ["结论", "同输入 ≠ 同计时口径；先修口径再比较 CPU"],
      ],
    },
    {
      title: "9 · E10：先造当前档位的配对 odom / gyro",
      lines: "394-420",
      explain: "每个 slip_lvl 内先构造完全相同的 odom_poses 与 gyro_wz，再切换 use_imu_yaw。零偏随机游走与白噪声也只生成一次，因此 wheel-only 与 IMU 版本消费相同传感器序列。",
      code: `    // 为当前打滑档位生成配对的 odom 与 gyro 序列
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
    }`,
      vars: [
        ["档位内配对", "odom_poses / gyro_wz 在 variant 循环外生成"],
        ["唯一开关", "variant → use_imu_yaw"],
        ["复现细节", "两个 slip_lvl 仍顺序消费全局 RNG"],
      ],
    },
    {
      title: "10 · E10：只切 IMU 开关，保持其余路径一致",
      lines: "422-464",
      explain: "两个 variant 使用同一地图、匹配器、odom 和初值；IMU 版本才调用 on_imu_gyro。误差、拒收和零偏估计在同一扫描帧上计算。这是比 E9 更清晰的单因素消融形状。",
      code: `    for (int variant = 0; variant < 2; ++variant) {
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
          "%.1f%% (reject %d), est_bg %.4f rad/s (true %.4f)\\n",
          err_sum / n_total, err_max, 100.0 * n_acc / n_total,
          n_reject, core.gyro_bias(), true_bias);
      } else {
        std::printf(
          "    A wheel_only : mean_err %.4f m, max_err %.4f m, accept "
          "%.1f%% (reject %d)\\n",
          err_sum / n_total, err_max, 100.0 * n_acc / n_total, n_reject);
      }
    }`,
      vars: [
        ["控制变量", "地图、匹配器、odom、扫描、初值全部相同"],
        ["处理变量", "仅 cc.use_imu_yaw 与对应 on_imu_gyro 调用"],
        ["输出", "mean / max / accept / reject / estimated bias"],
      ],
    },
  ];

  const MODELS = {
    steady: { bias: 0.006, gain: 0.22, noise: 0.004, failureRate: 0.01, baseMs: 0.075, angBias: 0.0008, angGain: 0.18, angNoise: 0.0022 },
    coarse: { bias: 0.018, gain: 0.48, noise: 0.008, failureRate: 0.04, baseMs: 0.115, angBias: 0.0022, angGain: 0.35, angNoise: 0.0045 },
    stressed: { bias: 0.028, gain: 0.62, noise: 0.012, failureRate: 0.14, baseMs: 0.240, angBias: 0.0040, angGain: 0.48, angNoise: 0.0070 },
  };

  function byId(id) {
    return document.getElementById(id);
  }

  function clearNode(node) {
    while (node && node.firstChild) node.removeChild(node.firstChild);
  }

  function appendRow(body, values) {
    const tr = document.createElement("tr");
    values.forEach((value) => {
      const td = document.createElement("td");
      td.textContent = String(value);
      tr.appendChild(td);
    });
    body.appendChild(tr);
  }

  function fillVars(host, entries) {
    clearNode(host);
    const fragment = document.createDocumentFragment();
    entries.forEach(([key, value]) => {
      const cell = document.createElement("div");
      cell.className = "hands-on-var";
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = key;
      dd.textContent = String(value);
      cell.appendChild(dt);
      cell.appendChild(dd);
      fragment.appendChild(cell);
    });
    host.appendChild(fragment);
  }

  function fmt(value, digits = 4) {
    if (value === null || value === undefined) return "null";
    if (typeof value === "boolean") return value ? "true" : "false";
    if (typeof value === "number") {
      if (!Number.isFinite(value)) return String(value);
      if (value !== 0 && (Math.abs(value) < 0.001 || Math.abs(value) >= 100000)) {
        return value.toExponential(2);
      }
      return value.toFixed(digits).replace(/\.?0+$/, "");
    }
    return String(value);
  }

  function highlight(code) {
    code.className = "language-cpp";
    delete code.dataset.highlighted;
    if (window.hljs) {
      try { window.hljs.highlightElement(code); } catch (_) {}
    }
  }

  function initStepper() {
    const title = byId("ch09-stage-title");
    const meta = byId("ch09-stage-meta");
    const explain = byId("ch09-stage-explain");
    const code = byId("ch09-stage-code");
    const index = byId("ch09-stage-index");
    const prev = byId("ch09-stage-prev");
    const next = byId("ch09-stage-next");
    const source = byId("ch09-stage-source");
    const copy = byId("ch09-stage-copy");
    const vars = byId("ch09-stage-vars");
    if (!title || !meta || !explain || !code || !index || !prev || !next || !source || !copy || !vars) return;

    let stage = 0;
    function render() {
      const cur = STAGES[stage];
      title.textContent = cur.title;
      meta.textContent = `src/scan_localizer/${SOURCE_FILE}:L${cur.lines} · ${stage + 1}/${STAGES.length}`;
      index.textContent = `${stage + 1} / ${STAGES.length}`;
      explain.textContent = cur.explain;
      code.textContent = cur.code;
      highlight(code);
      source.setAttribute("data-file", SOURCE_FILE);
      source.setAttribute("data-lines", cur.lines);
      source.setAttribute("data-label", `benchmark.cpp:L${cur.lines} · ${cur.title}`);
      fillVars(vars, cur.vars);
      prev.disabled = stage === 0;
      next.disabled = stage === STAGES.length - 1;
    }

    prev.addEventListener("click", () => { if (stage > 0) { stage -= 1; render(); } });
    next.addEventListener("click", () => { if (stage < STAGES.length - 1) { stage += 1; render(); } });
    copy.addEventListener("click", () => {
      if (navigator.clipboard) navigator.clipboard.writeText(code.textContent).catch(() => {});
      const old = copy.textContent;
      copy.textContent = "已复制";
      setTimeout(() => { copy.textContent = old; }, 1200);
    });
    render();
  }

  function splitmix32(value) {
    let x = value >>> 0;
    x = (x + 0x9e3779b9) >>> 0;
    x = Math.imul(x ^ (x >>> 16), 0x21f0aaad) >>> 0;
    x = Math.imul(x ^ (x >>> 15), 0x735a2d97) >>> 0;
    return (x ^ (x >>> 15)) >>> 0;
  }

  function hash32(text) {
    let hash = 0x811c9dc5;
    const value = String(text);
    for (let i = 0; i < value.length; i += 1) {
      hash ^= value.charCodeAt(i);
      hash = Math.imul(hash, 0x01000193) >>> 0;
    }
    return hash >>> 0;
  }

  function makeRng(seed) {
    let state = seed >>> 0;
    return function random() {
      state = (state + 0x6d2b79f5) >>> 0;
      let t = state;
      t = Math.imul(t ^ (t >>> 15), t | 1) >>> 0;
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function roleRng(seed, stream, id) {
    return makeRng(splitmix32((seed ^ hash32(stream) ^ Math.imul(id + 1, 0x9e3779b9)) >>> 0));
  }

  function normal(random) {
    const u = Math.max(random(), 1e-12);
    const v = random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  function wrapAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }

  function stableDigest(value) {
    return hash32(JSON.stringify(value)).toString(16).padStart(8, "0");
  }

  function readConfig() {
    return {
      candidateA: byId("ch09-name-a").value.trim(),
      candidateB: byId("ch09-name-b").value.trim(),
      modelA: byId("ch09-model-a").value,
      modelB: byId("ch09-model-b").value,
      trials: Number(byId("ch09-trials").value),
      seedInput: Number(byId("ch09-seed").value),
      fixedSeed: byId("ch09-fixed-seed").checked,
      mapId: byId("ch09-map-id").value.trim(),
      beams: Number(byId("ch09-beams").value),
      noiseStdM: 0.01,
      budgetA: Number(byId("ch09-budget-a").value),
      budgetB: Number(byId("ch09-budget-b").value),
      successPosM: Number(byId("ch09-success-pos").value),
      successAngDeg: Number(byId("ch09-success-ang").value),
      hypothesis: byId("ch09-hypothesis").value,
      deltaThresholdM: Number(byId("ch09-delta").value),
      failurePolicy: byId("ch09-failure-policy").value,
      timing: byId("ch09-timing").value,
      order: byId("ch09-order").value,
      paired: byId("ch09-paired").checked,
      precommit: byId("ch09-precommit").checked,
    };
  }

  function fairnessChecks(cfg) {
    return [
      {
        key: "candidate",
        ok: cfg.candidateA !== "" && cfg.candidateB !== "" && cfg.candidateA !== cfg.candidateB &&
          cfg.mapId !== "" && MODELS[cfg.modelA] && MODELS[cfg.modelB] && [180, 360, 720].includes(cfg.beams),
        detail: cfg.candidateA && cfg.candidateB && cfg.candidateA !== cfg.candidateB && cfg.mapId
          ? `${cfg.candidateA} vs ${cfg.candidateB}；map=${cfg.mapId}；beams=${cfg.beams}` : "候选名称、地图标识、模型与束数必须有效",
      },
      {
        key: "paired",
        ok: cfg.paired,
        detail: cfg.paired ? "A/B 逐对读取同一 map/scan/prior/GT" : "当前会生成两套独立 case，不能计算配对差",
      },
      {
        key: "seed",
        ok: cfg.fixedSeed && Number.isInteger(cfg.seedInput) && cfg.seedInput >= 0 && cfg.seedInput <= 4294967295,
        detail: cfg.fixedSeed ? "基准种子固定；case 与两个 role 流分离" : "运行时将解析随机种子，只能诊断，不能按输入种子复现",
      },
      {
        key: "order",
        ok: cfg.order === "balanced",
        detail: cfg.order === "balanced" ? "逐对 AB/BA 交替" : "固定候选先后，可能混入缓存、温度或调度顺序效应",
      },
      {
        key: "budget",
        ok: Number.isFinite(cfg.budgetA) && Number.isFinite(cfg.budgetB) && cfg.budgetA > 0 && cfg.budgetB > 0 && Math.abs(cfg.budgetA - cfg.budgetB) < 1e-12,
        detail: `A=${fmt(cfg.budgetA)} ms，B=${fmt(cfg.budgetB)} ms；等资源假设要求相等`,
      },
      {
        key: "timing",
        ok: cfg.timing === "match" || cfg.timing === "pipeline",
        detail: cfg.timing === "match" ? "双方都只计教学 match" : cfg.timing === "pipeline" ? "双方都计完整 pipeline" : "A 只计 match，B 计完整 pipeline，口径不一致",
      },
      {
        key: "failure",
        ok: cfg.failurePolicy === "count",
        detail: cfg.failurePolicy === "count" ? "全部调用保留；无效/超预算/未达标均计失败" : "将删除未成功行并改变分母，属于自欺诊断",
      },
      {
        key: "hypothesis",
        ok: cfg.precommit && (cfg.hypothesis === "a" || cfg.hypothesis === "b") &&
          Number.isFinite(cfg.deltaThresholdM) && cfg.deltaThresholdM < 0,
        detail: cfg.precommit && (cfg.hypothesis === "a" || cfg.hypothesis === "b") &&
          Number.isFinite(cfg.deltaThresholdM) && cfg.deltaThresholdM < 0
          ? `方向性假设与负阈值 ${fmt(cfg.deltaThresholdM)} m 已预先声明` : "确认性假设必须预先声明，且配对差阈值必须为负（要求至少改善指定幅度）",
      },
      {
        key: "sample",
        ok: Number.isInteger(cfg.trials) && cfg.trials >= 20 && cfg.trials <= 200,
        detail: `${cfg.trials} 对；本章最小确认性样本为 20 对（仍不是显著性证明）`,
      },
      {
        key: "threshold",
        ok: cfg.successPosM > 0 && cfg.successAngDeg > 0 && Number.isFinite(cfg.successPosM) && Number.isFinite(cfg.successAngDeg),
        detail: `成功：e < ${fmt(cfg.successPosM)} m 且 |θ| < ${fmt(cfg.successAngDeg)}°`,
      },
    ];
  }

  function renderFairness() {
    const cfg = readConfig();
    const checks = fairnessChecks(cfg);
    const passed = checks.filter((check) => check.ok).length;
    const body = byId("ch09-fairness-body");
    clearNode(body);
    checks.forEach((check) => appendRow(body, [check.key, check.ok ? "PASS" : "FAIL", check.detail]));
    byId("ch09-fairness-score").textContent = `（${passed} / ${checks.length}）`;
    byId("ch09-fairness-status").textContent = passed === checks.length
      ? "协议检查通过：可以生成确认性合成试验；仍需检查完整配对覆盖率。"
      : "协议检查未通过：可以运行诊断样本，但不得作出确认性算法结论。";
    return { cfg, checks, allPass: passed === checks.length, passed };
  }

  function makeCase(inputSeed, id, cfg) {
    const random = roleRng(inputSeed, "case-input", 0);
    const gt = { x: 3 + random() * 4, y: 2 + random() * 3, theta: (random() - 0.5) * 1.2 };
    const magnitude = 0.02 + random() * 0.25;
    const direction = random() * Math.PI * 2;
    const prior = {
      x: gt.x + magnitude * Math.cos(direction),
      y: gt.y + magnitude * Math.sin(direction),
      theta: wrapAngle(gt.theta + (random() * 2 - 1) * 0.28),
    };
    const difficulty = 0.65 + random() * 0.7;
    const ranges = Array.from({ length: cfg.beams }, () => Math.max(0.05, 1.5 + random() * 0.9 + normal(random) * cfg.noiseStdM));
    const input = {
      map_id: cfg.mapId,
      gt,
      prior,
      beams: cfg.beams,
      noise_std_m: cfg.noiseStdM,
      ranges,
    };
    return {
      case_id: id + 1,
      case_seed: inputSeed,
      difficulty,
      input,
      input_digest: stableDigest(input),
    };
  }

  function runCandidate(testCase, cfg, arm, roleSeed, position, executionOrder) {
    const modelId = arm === "A" ? cfg.modelA : cfg.modelB;
    const model = MODELS[modelId];
    const candidateName = arm === "A" ? cfg.candidateA : cfg.candidateB;
    const budget = arm === "A" ? cfg.budgetA : cfg.budgetB;
    const random = roleRng(roleSeed, `role-${arm}`, testCase.case_id - 1);
    const input = testCase.input;
    const priorError = Math.hypot(input.prior.x - input.gt.x, input.prior.y - input.gt.y);
    const errorDirection = random() * Math.PI * 2;
    const plannedError = model.bias + priorError * model.gain +
      Math.abs(normal(random)) * model.noise + cfg.noiseStdM * 0.35 * testCase.difficulty;
    const priorAngle = wrapAngle(input.prior.theta - input.gt.theta);
    const plannedAngle = model.angBias + priorAngle * model.angGain + normal(random) * model.angNoise;
    const workMs = model.baseMs * (0.75 + random() * 0.5) * (0.7 + cfg.beams / 720) +
      priorError * 0.015 * testCase.difficulty;
    const pipelineOverhead = 0.10 + cfg.beams / 720 * 0.03;
    const fullPipeline = cfg.timing === "pipeline" || (cfg.timing === "asymmetric" && arm === "B");
    const latencyMs = workMs + (fullPipeline ? pipelineOverhead : 0);
    const stochasticFailure = random() < model.failureRate;
    const overBudget = latencyMs > budget;
    const valid = !stochasticFailure && !overBudget;
    let pose = null;
    let posError = null;
    let angErrorDeg = null;
    if (valid) {
      pose = {
        x: input.gt.x + plannedError * Math.cos(errorDirection),
        y: input.gt.y + plannedError * Math.sin(errorDirection),
        theta: wrapAngle(input.gt.theta + plannedAngle),
      };
      posError = Math.hypot(pose.x - input.gt.x, pose.y - input.gt.y);
      angErrorDeg = Math.abs(wrapAngle(pose.theta - input.gt.theta)) * 180 / Math.PI;
    }
    const success = valid && posError < cfg.successPosM && angErrorDeg < cfg.successAngDeg;
    return {
      schema: SCHEMA,
      provenance: PROVENANCE,
      case_id: testCase.case_id,
      case_seed: testCase.case_seed,
      arm,
      candidate_name: candidateName,
      model_id: modelId,
      role_seed: roleSeed,
      execution_order: executionOrder,
      position,
      input: testCase.input,
      input_digest: testCase.input_digest,
      valid,
      invalid_reason: stochasticFailure ? "synthetic_model_failure" : overBudget ? "budget_exceeded" : null,
      pose,
      pos_error_m: posError,
      ang_error_deg: angErrorDeg,
      latency_ms: latencyMs,
      work_ms: workMs,
      budget_ms: budget,
      timing_scope: fullPipeline ? "pipeline" : "match",
      success_pos_m: cfg.successPosM,
      success_ang_deg: cfg.successAngDeg,
      success,
    };
  }

  function median(values) {
    if (!values.length) return null;
    const sorted = values.slice().sort((a, b) => a - b);
    return sorted[Math.floor(sorted.length / 2)];
  }

  function p90(values) {
    if (!values.length) return null;
    const sorted = values.slice().sort((a, b) => a - b);
    return sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * 0.9))];
  }

  function armSummary(rows, arm) {
    const selected = rows.filter((row) => row.arm === arm);
    const valid = selected.filter((row) => row.valid === true && Number.isFinite(row.pos_error_m) && Number.isFinite(row.ang_error_deg));
    const errors = valid.map((row) => row.pos_error_m);
    const latencies = selected.map((row) => row.latency_ms).filter(Number.isFinite);
    const successCount = selected.filter((row) => row.success === true).length;
    return {
      calls: selected.length,
      valid: valid.length,
      invalid: selected.length - valid.length,
      successes: successCount,
      successRate: selected.length ? successCount / selected.length : null,
      medianErrorM: median(errors),
      p90ErrorM: p90(errors),
      medianLatencyMs: median(latencies),
      p90LatencyMs: p90(latencies),
    };
  }

  function pairedSummary(rows, cfg) {
    if (!cfg.paired) {
      return { eligiblePairs: 0, completePairs: 0, coverage: null, medianDeltaM: null, medianDeltaBMinusAM: null, aWinRate: null };
    }
    const byCase = new Map();
    rows.forEach((row) => {
      if (!byCase.has(row.case_id)) byCase.set(row.case_id, {});
      byCase.get(row.case_id)[row.arm] = row;
    });
    const deltas = [];
    const reverseDeltas = [];
    let winsA = 0;
    byCase.forEach((pair) => {
      const a = pair.A;
      const b = pair.B;
      if (!a || !b || !a.valid || !b.valid || !Number.isFinite(a.pos_error_m) || !Number.isFinite(b.pos_error_m)) return;
      const delta = a.pos_error_m - b.pos_error_m;
      deltas.push(delta);
      reverseDeltas.push(-delta);
      if (delta < 0) winsA += 1;
    });
    return {
      eligiblePairs: cfg.trials,
      completePairs: deltas.length,
      coverage: cfg.trials ? deltas.length / cfg.trials : null,
      medianDeltaM: median(deltas),
      medianDeltaBMinusAM: median(reverseDeltas),
      aWinRate: deltas.length ? winsA / deltas.length : null,
    };
  }

  function summaryCore(rows, cfg) {
    return {
      A: armSummary(rows, "A"),
      B: armSummary(rows, "B"),
      paired: pairedSummary(rows, cfg),
    };
  }

  function fingerprint(core) {
    return hash32(JSON.stringify(core)).toString(16).padStart(8, "0");
  }

  function decisionText(cfg, checks, summary) {
    if (!checks.allPass) return "不可判定：公平性协议未全部通过。当前数字只能用于诊断，不能支持确认性假设。";
    if (cfg.hypothesis === "explore") return "仅探索：未预先锁定方向性假设，不报告支持/不支持。";
    const paired = summary.core.paired;
    const minimumPairs = Math.ceil(cfg.trials * 0.8);
    if (paired.completePairs < minimumPairs) {
      return `不可判定：完整有效配对仅 ${paired.completePairs}/${cfg.trials}，低于 80% 覆盖要求。`;
    }
    const observed = cfg.hypothesis === "a" ? paired.medianDeltaM : paired.medianDeltaBMinusAM;
    if (observed === null) return "不可判定：没有可计算的配对差。";
    const supported = observed <= cfg.deltaThresholdM;
    return supported
      ? `合成沙盘：观察到的配对差 ${fmt(observed)} m ≤ 预先阈值 ${fmt(cfg.deltaThresholdM)} m，支持该 H1。这是描述性阈值判定，不是显著性检验，也不是真实算法结论。`
      : `合成沙盘：观察到的配对差 ${fmt(observed)} m > 预先阈值 ${fmt(cfg.deltaThresholdM)} m，不支持该 H1。这是描述性阈值判定，不是显著性检验，也不是真实算法结论。`;
  }

  function resolvedBaseSeed(cfg) {
    if (cfg.fixedSeed) return cfg.seedInput >>> 0;
    if (window.crypto && typeof window.crypto.getRandomValues === "function") {
      const buffer = new Uint32Array(1);
      window.crypto.getRandomValues(buffer);
      return buffer[0];
    }
    return (Date.now() ^ Math.floor(Math.random() * 4294967296)) >>> 0;
  }

  function runExperiment() {
    const audit = renderFairness();
    const cfg = audit.cfg;
    const baseSeed = resolvedBaseSeed(cfg);
    const caseSeed = splitmix32((baseSeed ^ 0xc003) >>> 0);
    const seedA = splitmix32((baseSeed ^ 0xa001) >>> 0);
    const seedB = splitmix32((baseSeed ^ 0xb002) >>> 0);
    const separateCaseSeedB = splitmix32((caseSeed ^ 0xd004) >>> 0);
    const allRows = [];

    for (let i = 0; i < cfg.trials; i += 1) {
      const inputSeedA = splitmix32((caseSeed ^ Math.imul(i + 1, 0x85ebca6b)) >>> 0);
      const inputSeedB = splitmix32((separateCaseSeedB ^ Math.imul(i + 1, 0x85ebca6b)) >>> 0);
      const caseA = makeCase(inputSeedA, i, cfg);
      const caseB = cfg.paired ? caseA : makeCase(inputSeedB, i, cfg);
      const aFirst = cfg.order === "a_first" || (cfg.order === "balanced" && i % 2 === 0);
      const bFirst = cfg.order === "b_first" || (cfg.order === "balanced" && i % 2 !== 0);
      const executionOrder = aFirst ? "AB" : "BA";
      if (aFirst) {
        allRows.push(runCandidate(caseA, cfg, "A", seedA, 1, executionOrder));
        allRows.push(runCandidate(caseB, cfg, "B", seedB, 2, executionOrder));
      } else if (bFirst) {
        allRows.push(runCandidate(caseB, cfg, "B", seedB, 1, executionOrder));
        allRows.push(runCandidate(caseA, cfg, "A", seedA, 2, executionOrder));
      }
    }

    const rows = cfg.failurePolicy === "drop" ? allRows.filter((row) => row.success === true) : allRows;
    const dropped = allRows.length - rows.length;
    const core = summaryCore(rows, cfg);
    const summary = {
      schema: SCHEMA,
      provenance: PROVENANCE,
      core,
      fingerprint: fingerprint(core),
    };
    const runConfig = {
      ...cfg,
      schema: SCHEMA,
      provenance: PROVENANCE,
      generator: "hands-on-ch09.js deterministic teaching models; no C++ matcher is invoked",
      modelDefinitions: MODELS,
      resolvedBaseSeed: baseSeed,
      caseSeed,
      separateCaseSeedB: cfg.paired ? null : separateCaseSeedB,
      roleSeedA: seedA,
      roleSeedB: seedB,
      randomAlgorithm: "splitmix32-seeded mulberry32; role/case streams; Box-Muller normal",
      digestAlgorithm: "FNV-1a 32-bit over stable JSON; non-cryptographic input fingerprint",
      medianConvention: "upper median sorted[floor(n/2)]",
      p90Convention: "sorted[min(n-1,floor(0.9*n))]",
      fairness: audit.checks,
      fairnessAllPass: audit.allPass,
      generatedRows: allRows.length,
      exportedRows: rows.length,
      droppedRows: dropped,
      staticIntelMetricsUsed: false,
    };

    currentRun = { rows, rawText: rows.map((row) => JSON.stringify(row)).join("\n") + (rows.length ? "\n" : ""), cfg: runConfig, summary };
    byId("ch09-results").hidden = false;
    byId("ch09-run-status").textContent = `已生成 ${allRows.length} 行；导出 ${rows.length} 行，删除 ${dropped} 行。解析种子 ${baseSeed}；case=${caseSeed}；role A=${seedA}；role B=${seedB}。`;
    byId("ch09-decision").textContent = decisionText(runConfig, audit, summary);
    byId("ch09-recompute-status").textContent = "尚未复算。";
    byId("ch09-raw-count").textContent = `原始记录 ${rows.length} 行；摘要指纹 ${summary.fingerprint}。上中位数/P90 索引规则与配置一同导出。`;

    const a = core.A;
    const b = core.B;
    fillVars(byId("ch09-summary-vars"), [
      ["证据类型", "浏览器合成教学样本；非 C++ / ROS 2 / Intel Lab"],
      ["A 成功", `${a.successes}/${a.calls} (${a.successRate === null ? "—" : (a.successRate * 100).toFixed(1)}%)`],
      ["A 有效 / 失败", `${a.valid} / ${a.invalid}`],
      ["A 误差 中位 / P90 (m)", `${fmt(a.medianErrorM)} / ${fmt(a.p90ErrorM)}`],
      ["A 延迟 中位 / P90 (ms)", `${fmt(a.medianLatencyMs)} / ${fmt(a.p90LatencyMs)}`],
      ["B 成功", `${b.successes}/${b.calls} (${b.successRate === null ? "—" : (b.successRate * 100).toFixed(1)}%)`],
      ["B 有效 / 失败", `${b.valid} / ${b.invalid}`],
      ["B 误差 中位 / P90 (m)", `${fmt(b.medianErrorM)} / ${fmt(b.p90ErrorM)}`],
      ["B 延迟 中位 / P90 (ms)", `${fmt(b.medianLatencyMs)} / ${fmt(b.p90LatencyMs)}`],
      ["完整有效配对", `${core.paired.completePairs}/${core.paired.eligiblePairs} (${core.paired.coverage === null ? "—" : (core.paired.coverage * 100).toFixed(1)}%)`],
      ["配对差 median(eA-eB) (m)", fmt(core.paired.medianDeltaM)],
      ["A 配对胜率", core.paired.aWinRate === null ? "—" : `${(core.paired.aWinRate * 100).toFixed(1)}%`],
      ["公平性", `${audit.passed}/${audit.checks.length} PASS`],
      ["失败策略", cfg.failurePolicy === "count" ? "保留全部调用" : `故意删除 ${dropped} 行；不可判定`],
    ]);

    const preview = byId("ch09-raw-preview");
    clearNode(preview);
    rows.slice(0, 8).forEach((row) => appendRow(preview, [
      row.case_id,
      row.arm,
      row.executionOrder,
      row.valid,
      row.success,
      fmt(row.pos_error_m),
      fmt(row.ang_error_deg),
      fmt(row.latency_ms),
      row.input_digest,
    ]));
    ["ch09-recompute", "ch09-download-jsonl", "ch09-download-csv", "ch09-download-config"].forEach((id) => { byId(id).disabled = false; });
  }

  function csvCell(value) {
    let text = value === null || value === undefined ? "null" : String(value);
    if (/^[=+\-@]/.test(text)) text = `'${text}`;
    return `"${text.replace(/"/g, '""')}"`;
  }

  function csvExport(rows) {
    const headers = [
      "schema", "provenance", "case_id", "case_seed", "arm", "candidate_name", "model_id", "role_seed",
      "execution_order", "position", "map_id", "input_digest", "gt_x", "gt_y", "gt_theta_rad",
      "prior_x", "prior_y", "prior_theta_rad", "beams", "noise_std_m", "valid", "invalid_reason",
      "pred_x", "pred_y", "pred_theta_rad", "pos_error_m", "ang_error_deg", "latency_ms", "work_ms",
      "budget_ms", "timing_scope", "success_pos_m", "success_ang_deg", "success",
    ];
    const lines = [headers.map(csvCell).join(",")];
    rows.forEach((row) => {
      const input = row.input;
      lines.push([
        row.schema, row.provenance, row.case_id, row.case_seed, row.arm, row.candidate_name, row.model_id, row.role_seed,
        row.execution_order, row.position, input.map_id, row.input_digest, input.gt.x, input.gt.y, input.gt.theta,
        input.prior.x, input.prior.y, input.prior.theta, input.beams, input.noise_std_m, row.valid, row.invalid_reason,
        row.pose ? row.pose.x : null, row.pose ? row.pose.y : null, row.pose ? row.pose.theta : null,
        row.pos_error_m, row.ang_error_deg, row.latency_ms, row.work_ms, row.budget_ms, row.timing_scope,
        row.success_pos_m, row.success_ang_deg, row.success,
      ].map(csvCell).join(","));
    });
    return lines.join("\n") + "\n";
  }

  function safeFilename(text) {
    return String(text).replace(/[^a-zA-Z0-9_-]+/g, "_").slice(0, 48) || "candidate";
  }

  function download(filename, text, mime) {
    const blob = new Blob([text], { type: mime });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = filename;
    document.body.appendChild(anchor);
    anchor.click();
    anchor.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  function recomputeRawRows(rows, cfg) {
    const issues = [];
    const recomputed = rows.map((row, index) => {
      const input = row.input;
      const gt = input && input.gt;
      const pose = row.pose;
      const finiteInput = gt && [gt.x, gt.y, gt.theta].every(Number.isFinite);
      const finitePose = pose && [pose.x, pose.y, pose.theta].every(Number.isFinite);
      if (!finiteInput || (row.valid && !finitePose)) {
        issues.push(`第 ${index + 1} 行缺少有限 GT 或有效位姿`);
      }
      if (!input || stableDigest(input) !== row.input_digest) {
        issues.push(`第 ${index + 1} 行 input_digest 与输入不符`);
      }
      if (!Number.isFinite(row.latency_ms)) issues.push(`第 ${index + 1} 行延迟不是有限数`);

      let posError = null;
      let angErrorDeg = null;
      if (row.valid === true && finiteInput && finitePose) {
        // 不复用行内已计算的误差：从原始 GT 与输出位姿重新推导。
        posError = Math.hypot(pose.x - gt.x, pose.y - gt.y);
        angErrorDeg = Math.abs(wrapAngle(pose.theta - gt.theta)) * 180 / Math.PI;
        if (!Number.isFinite(row.pos_error_m) || Math.abs(row.pos_error_m - posError) > 1e-12 ||
            !Number.isFinite(row.ang_error_deg) || Math.abs(row.ang_error_deg - angErrorDeg) > 1e-10) {
          issues.push(`第 ${index + 1} 行存储误差与 GT/pose 复算值不符`);
        }
      } else if (row.pos_error_m !== null || row.ang_error_deg !== null) {
        issues.push(`第 ${index + 1} 行无效输出却存储了非空误差`);
      }
      const success = row.valid === true && posError !== null && angErrorDeg !== null &&
        posError < cfg.successPosM && angErrorDeg < cfg.successAngDeg;
      if (row.success !== success) issues.push(`第 ${index + 1} 行 success 与配置阈值复算值不符`);
      return { ...row, pos_error_m: posError, ang_error_deg: angErrorDeg, success };
    });
    return { rows: recomputed, issues };
  }

  function recompute() {
    if (!currentRun) return;
    try {
      const rawText = currentRun.rawText;
      const rows = rawText.split("\n").filter((line) => line.trim() !== "").map((line) => JSON.parse(line));
      const audit = recomputeRawRows(rows, currentRun.cfg);
      const core = summaryCore(audit.rows, currentRun.cfg);
      const nextFingerprint = fingerprint(core);
      const same = audit.issues.length === 0 && nextFingerprint === currentRun.summary.fingerprint && rows.length === currentRun.rows.length;
      byId("ch09-recompute-status").textContent = same
        ? `复算一致：从 ${rows.length} 行 JSONL 的 GT/pose 重算误差与成功标志，摘要指纹 ${nextFingerprint} 与运行结果相同。`
        : `复算不一致：原始 ${rows.length} 行，新指纹 ${nextFingerprint}，原指纹 ${currentRun.summary.fingerprint}；${audit.issues.slice(0, 3).join("；")}`;
    } catch (error) {
      byId("ch09-recompute-status").textContent = `复算失败：${error.message}`;
    }
  }

  let currentRun = null;

  function markStale() {
    if (!currentRun) return;
    currentRun = null;
    byId("ch09-results").hidden = true;
    ["ch09-recompute", "ch09-download-jsonl", "ch09-download-csv", "ch09-download-config"].forEach((id) => { byId(id).disabled = true; });
  }

  function initLab() {
    const form = byId("ch09-lab-form");
    if (!form || !byId("ch09-fairness-body")) return;
    form.addEventListener("input", () => { renderFairness(); markStale(); });
    form.addEventListener("change", () => { renderFairness(); markStale(); });
    form.addEventListener("submit", (event) => {
      event.preventDefault();
      if (!form.reportValidity()) return;
      runExperiment();
    });
    byId("ch09-check-fairness").addEventListener("click", renderFairness);
    byId("ch09-recompute").addEventListener("click", recompute);
    byId("ch09-download-jsonl").addEventListener("click", () => {
      if (!currentRun) return;
      download(`ch09_synthetic_${currentRun.cfg.resolvedBaseSeed}.jsonl`, currentRun.rawText, "application/x-ndjson;charset=utf-8");
    });
    byId("ch09-download-csv").addEventListener("click", () => {
      if (!currentRun) return;
      download(`ch09_synthetic_${currentRun.cfg.resolvedBaseSeed}.csv`, csvExport(currentRun.rows), "text/csv;charset=utf-8");
    });
    byId("ch09-download-config").addEventListener("click", () => {
      if (!currentRun) return;
      download(`ch09_synthetic_${currentRun.cfg.resolvedBaseSeed}_config.json`, JSON.stringify(currentRun.cfg, null, 2) + "\n", "application/json;charset=utf-8");
    });
    ["ch09-recompute", "ch09-download-jsonl", "ch09-download-csv", "ch09-download-config"].forEach((id) => { byId(id).disabled = true; });
    renderFairness();
  }

  function updateExerciseScore() {
    const forms = Array.from(document.querySelectorAll(".hands-on-exercise[data-exercise]"));
    const passed = forms.filter((form) => form.dataset.passed === "true").length;
    byId("ch09-exercise-score").textContent = `（${passed} / ${forms.length}）`;
    byId("ch09-acceptance").textContent = passed === forms.length
      ? "练习验收通过：已识别独立样本伪装配对、计时口径不等和删除失败三类自欺。"
      : `练习验收尚未完成：还需通过 ${forms.length - passed} 题。`;
  }

  function initExercises() {
    const forms = Array.from(document.querySelectorAll(".hands-on-exercise[data-exercise]"));
    forms.forEach((form) => {
      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const input = form.querySelector(".exercise-input");
        const feedback = form.querySelector(".exercise-feedback");
        if (!input || !feedback) return;
        const raw = input.value.trim().replace(/\s+/g, "").toLowerCase();
        const ok = raw === String(form.dataset.answer || "").toLowerCase();
        form.dataset.passed = String(ok);
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok ? form.dataset.success : form.dataset.hint;
        updateExerciseScore();
      });
      form.querySelector(".exercise-input").addEventListener("input", () => {
        delete form.dataset.passed;
        const feedback = form.querySelector(".exercise-feedback");
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
        updateExerciseScore();
      });
    });
    byId("ch09-reset-exercises").addEventListener("click", () => {
      forms.forEach((form) => {
        delete form.dataset.passed;
        form.querySelector(".exercise-input").value = "";
        const feedback = form.querySelector(".exercise-feedback");
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
      });
      updateExerciseScore();
    });
    updateExerciseScore();
  }

  function init() {
    if (window.PAGE_ID !== "ch09") return;
    initStepper();
    initLab();
    initExercises();
  }

  window.addEventListener("sloc:ready", init);
})();
