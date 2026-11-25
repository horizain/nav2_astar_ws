// Ch07 hands-on layer: normal tracking stepper, live EKF trace and fill-in exercises.
(function () {
  "use strict";

  const TRACE_FIELDS = [
    "pred", "F4", "Q4", "Ppred", "match", "R", "innov", "S", "K4", "dx", "accepted",
  ];
  const RECOVERY_STATES = new Set(["UNINIT", "LOST", "GLOBAL"]);
  const STAGES = [
    {
      title: "1 · 合成运动增量，得到预测位姿",
      lines: "528-546",
      explain: "正常路径从 compute_motion_delta() 开始：平移取里程计相对增量，IMU 航向减去零偏积分。随后用上一帧航向把平动增量旋转到 map 系，得到 pred。compute_motion_delta() 的完整实现另见 L277-304。",
      code: `Pose2 delta{};
bool imu_used = false;
double dt_imu = 0.0;
compute_motion_delta(delta, imu_used, dt_imu);

if (!hypotheses_.empty() && first_fix_) {
  propagate_hypotheses(delta);
}

const double th = last_pose_.theta;
const double c = std::cos(th);
const double s = std::sin(th);
const Pose2 pred = first_fix_ ?
  Pose2(
    last_pose_.x + c * delta.x - s * delta.y,
    last_pose_.y + s * delta.x + c * delta.y,
    wrap_angle(th + delta.theta)) :
  last_pose_;`,
    },
    {
      title: "2 · 构造 F4：航向与零偏如何耦合",
      lines: "548-556",
      explain: "F4 的前三列描述 SE(2) 预测雅可比；使用 IMU 且 dt>0 时，F4(2,3)=-dt，把零偏误差传向航向。没有首次定位时 F4 保持单位阵，避免从无效先验传播误差。",
      code: `Eigen::Matrix4d F4 = Eigen::Matrix4d::Identity();
if (first_fix_) {
  F4(0, 2) = -(delta.x * s + delta.y * c);
  F4(1, 2) = delta.x * c - delta.y * s;
  if (imu_used && dt_imu > 0.0) {
    F4(2, 3) = -dt_imu;
  }
}`,
    },
    {
      title: "3 · 构造 Q4，预测并对称化 Ppred",
      lines: "558-593",
      explain: "平动噪声随行驶距离增长，转角噪声同时考虑距离与角位移；IMU 使用时写入对应的转角过程噪声和零偏随机游走。G 把体系平动噪声旋转到 map 系，最后 Ppred=F4*P4*F4ᵀ+Q4 并对称化。",
      code: `double q_trans = 0.0;
double q_rot = 0.0;
if (has_odom_) {
  const double dtrans = std::hypot(delta.x, delta.y);
  q_trans = dtrans * cfg_.odom_trans_var_per_m;
  q_rot = dtrans * cfg_.odom_rot_var_per_m +
    std::abs(delta.theta) *
      (imu_used ? cfg_.imu_rot_var_per_rad : cfg_.odom_rot_var_per_rad);
  if (imu_used && dt_imu > 0.0) {
    q_rot += dt_imu * cfg_.imu_rot_var_per_rad;
  }
} else {
  q_trans = std::max(0.0, cfg_.odomless_trans_var);
  q_rot = imu_used ?
    (std::abs(delta.theta) * cfg_.imu_rot_var_per_rad +
     dt_imu * cfg_.imu_rot_var_per_rad) :
    std::max(0.0, cfg_.odomless_rot_var);
}

Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
G(0, 0) = c; G(0, 1) = -s;
G(1, 0) = s; G(1, 1) = c;
Eigen::Matrix3d Q_body = Eigen::Matrix3d::Zero();
Q_body(0, 0) = q_trans;
Q_body(1, 1) = q_trans;
Q_body(2, 2) = q_rot;
Eigen::Matrix4d Q4 = Eigen::Matrix4d::Zero();
Q4.topLeftCorner<3, 3>() = G * Q_body * G.transpose();
if (imu_used && dt_imu > 0.0) {
  Q4(3, 3) = std::max(0.0, cfg_.gyro_bias_rw_var_per_s) * dt_imu;
}

Cov4 P4_pred = F4 * P4_ * F4.transpose() + Q4;
P4_pred = 0.5 * (P4_pred + P4_pred.transpose());`,
    },
    {
      title: "4 · 调用 matcher，先建立 mr_finite 总前提",
      lines: "595-605",
      explain: "匹配器只围绕 pred 工作，输出位姿、score、inlier_ratio 与 3×3 协方差。mr_finite 不是“第五门”，而是所有门控的前提：valid、位姿有限、统计量有限、整个协方差有限。输出先默认指向 pred。",
      code: `const MatchResult mr =
  matcher_->match(*field_, pred, *active_scan, cfg_.match_opt);
const bool mr_finite = mr.valid &&
  is_finite_pose(mr.pose) &&
  std::isfinite(mr.score) &&
  std::isfinite(mr.inlier_ratio) &&
  mr.covariance.allFinite();

out.score = std::isfinite(mr.score) ? mr.score : 0.0;
out.inlier_ratio =
  std::isfinite(mr.inlier_ratio) ? mr.inlier_ratio : 0.0;
out.pose = pred;`,
    },
    {
      title: "5 · Score / Inlier 与第四门 MinEig(R)",
      lines: "607-621",
      explain: "门 1、门 2 分别检查分数与内点率。第四门检查匹配协方差 R：对称化后求最小特征值，必须有限且严格处于 0 与 max_cov_eig 之间。源码先算它，是因为下一步构造创新协方差 S 需要 R。",
      code: `const bool gate_inlier =
  mr_finite && (mr.inlier_ratio >= cfg_.min_inlier_ratio);
const bool gate_score =
  mr_finite && (mr.score >= cfg_.min_score);

Cov3 R = Cov3::Identity();
bool gate_cov = false;
if (mr_finite) {
  R = 0.5 * (mr.covariance + mr.covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Cov3> es(R);
  if (es.info() == Eigen::Success && es.eigenvalues().size() == 3) {
    const double min_eig = es.eigenvalues().minCoeff();
    gate_cov = std::isfinite(min_eig) && (min_eig > 0.0) &&
      (min_eig < cfg_.max_cov_eig);
  }
}`,
    },
    {
      title: "6 · 第三门马氏距离与总 accept",
      lines: "623-653",
      explain: "innov 的角分量必须 wrap；S 由 Ppred 的位姿块、R 和 1e-9 对角正则项构成。LDLT 成功且正定后才计算 d²。默认 mahal_gate 是固定 9.0，接受条件是有限的 0≤d²<9.0，不是动态卡方分位数。",
      code: `Eigen::Vector3d innov = Eigen::Vector3d::Zero();
Cov3 S = Cov3::Identity();
Eigen::LDLT<Cov3> ldlt;
bool gate_mahal = false;
if (mr_finite) {
  innov = Eigen::Vector3d(
    mr.pose.x - pred.x,
    mr.pose.y - pred.y,
    wrap_angle(mr.pose.theta - pred.theta));
  const Cov3 S_raw =
    P4_pred.topLeftCorner<3, 3>() + R;
  S = 0.5 * (S_raw + S_raw.transpose()) +
      Cov3::Identity() * 1e-9;
  ldlt.compute(S);
  if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
    const Eigen::Vector3d S_inv_innov = ldlt.solve(innov);
    if (ldlt.info() == Eigen::Success &&
        S_inv_innov.allFinite()) {
      const double mahal = innov.dot(S_inv_innov);
      gate_mahal = std::isfinite(mahal) && mahal >= 0.0 &&
        mahal < cfg_.mahal_gate;
    }
  }
}

const bool accept = mr_finite && gate_inlier && gate_score &&
  gate_mahal && gate_cov;

// 高分高内点、仅马氏拒绝：不属于绑架异常。
const bool is_anomaly = !accept &&
  (!mr_finite || mr.score < cfg_.anomaly_low_score ||
   (!gate_inlier && !gate_score));
record_anomaly(is_anomaly);`,
    },
    {
      title: "7 · 全通过后才计算 K4 与 dx",
      lines: "655-680",
      explain: "K4 是 4×3 增益，dx 是四维状态修正。生产代码用 S_inv_HP 的转置得到 K4，避免显式求逆；可选限幅只缩放 dx 的平移/转角，零偏仍按 dx(3) 更新并做物理限幅。",
      code: `if (accept) {
  const Eigen::Matrix<double, 3, 4> S_inv_HP =
    ldlt.solve(P4_pred.topRows<3>());
  if (ldlt.info() == Eigen::Success &&
      S_inv_HP.allFinite()) {
    const Eigen::Matrix<double, 4, 3> K4 =
      S_inv_HP.transpose();
    Eigen::Vector4d dx = K4 * innov;

    if (cfg_.max_update_trans > 0.0) {
      const double tn = std::hypot(dx(0), dx(1));
      if (tn > cfg_.max_update_trans) {
        const double scale = cfg_.max_update_trans / tn;
        dx(0) *= scale;
        dx(1) *= scale;
      }
    }
    if (cfg_.max_update_rot > 0.0) {
      dx(2) = std::clamp(
        dx(2), -cfg_.max_update_rot, cfg_.max_update_rot);
    }

    last_pose_ = Pose2(
      pred.x + dx(0), pred.y + dx(1),
      wrap_angle(pred.theta + dx(2)));
    const double max_b = std::max(0.0, cfg_.max_gyro_bias);
    gyro_bias_ = std::clamp(
      gyro_bias_ + dx(3), -max_b, max_b);`,
    },
    {
      title: "8 · Joseph 更新、对称化并回到 TRACKING",
      lines: "682-704",
      explain: "H=[I3|0] 只观测位姿。Joseph 两项分别保留预测协方差与测量噪声的非负二次型结构，随后对称化。接受帧清零拒收计数、递增稳定跟踪计数，并在足够多帧后清理多假设队列。",
      code: `Eigen::Matrix<double, 3, 4> H =
  Eigen::Matrix<double, 3, 4>::Zero();
H.leftCols<3>() = Eigen::Matrix3d::Identity();
const Eigen::Matrix4d I_KH =
  Eigen::Matrix4d::Identity() - K4 * H;
P4_ = I_KH * P4_pred * I_KH.transpose() +
      K4 * R * K4.transpose();
P4_ = 0.5 * (P4_ + P4_.transpose());

reject_streak_ = 0;
state_ = LocState::TRACKING;
++stable_tracking_streak_;
if (!hypotheses_.empty() &&
    stable_tracking_streak_ >=
      std::max(1, cfg_.hyp_confirm_frames)) {
  clear_hypotheses();
}

out.pose = last_pose_;
out.covariance = last_cov();
out.state = state_;
out.accepted = true;
out.reject_streak = 0;
first_fix_ = true;
return out;
    }
  }`,
    },
    {
      title: "9 · 拒绝：纯预测、温和解锁与状态降级",
      lines: "706-757",
      explain: "拒绝帧不执行 K4/Joseph，只提交 pred 与 Ppred。高分、高内点、R 有效但马氏失败时，生产代码适度松弛 P 以打破静止锁死；这仍不是测量更新。随后累计拒收，按阈值进入 DEGRADED/LOST，或由异常窗口触发 GLOBAL。片段省略了中间多假设晋升细节。",
      code: `last_pose_ = pred;
P4_ = P4_pred;
if (mr_finite && gate_inlier && gate_score &&
    gate_cov && !gate_mahal) {
  P4_.topLeftCorner<3, 3>() *= 1.5;
  P4_(0, 0) = std::min(P4_(0, 0) + 2e-4, 0.25);
  P4_(1, 1) = std::min(P4_(1, 1) + 2e-4, 0.25);
  P4_(2, 2) = std::min(P4_(2, 2) + 2e-5, 0.10);
}

stable_tracking_streak_ = 0;
++reject_streak_;
const int deg_thr = std::max(1, cfg_.degraded_after);
const int lost_thr = std::max(deg_thr, cfg_.lost_after);
if (reject_streak_ >= deg_thr) {
  state_ = LocState::DEGRADED;
}
if (reject_streak_ >= lost_thr) {
  state_ = LocState::LOST;
  manual_global_ = false;
  reject_streak_ = 0;
}

// ... L731-749：多假设晋升 / 滑动窗口异常触发 GLOBAL ...

out.pose = last_pose_;
out.covariance = last_cov();
out.state = state_;
out.accepted = false;
out.reject_streak = reject_streak_;
first_fix_ = true;
return out;`,
    },
    {
      title: "10 · 旁路：UNINIT / LOST / GLOBAL 恢复",
      lines: "431-526",
      explain: "这三个状态在正常预测之前进入独立恢复分支：传播并验证多假设，或调用 global_fn_；自动远距离跳变还要比较局部健康匹配。恢复成功会重置 P4/gyro_bias 并提前返回，不能假设它拥有正常路径的 F4、K4 或 Joseph trace。",
      code: `if (state_ == LocState::UNINIT ||
    state_ == LocState::LOST ||
    state_ == LocState::GLOBAL)
{
  Pose2 delta{};
  bool imu_used = false;
  double dt_imu = 0.0;
  compute_motion_delta(delta, imu_used, dt_imu);

  if (!hypotheses_.empty()) {
    if (first_fix_) { propagate_hypotheses(delta); }
    if (try_promote_hypotheses(*active_scan, out)) {
      return out;
    }
  }

  if (global_fn_) {
    Pose2 g{};
    double sc = 0.0;
    if (global_fn_(*active_scan, g, sc) &&
        is_finite_pose(g) && std::isfinite(sc)) {
      // ... L453-492：自动远距离跳变安全保护 ...

      last_pose_ = Pose2(g.x, g.y, wrap_angle(g.theta));
      P4_ = Cov4::Zero();
      P4_.topLeftCorner<3, 3>() =
        Cov3::Identity() * 0.04;
      gyro_bias_ = 0.0;
      state_ = LocState::TRACKING;
      // ... 清计数 / 清假设 / 填输出 ...
    }
  }
  return out;
}`,
    },
  ];

  function fmt(value, digits = 4) {
    if (value === null || value === undefined) return "—";
    if (typeof value === "boolean") return value ? "true" : "false";
    if (typeof value === "number") {
      if (!Number.isFinite(value)) return String(value);
      const av = Math.abs(value);
      if (av !== 0 && (av < 1e-3 || av >= 1e4)) {
        return value.toExponential(2);
      }
      return value.toFixed(digits).replace(/\.?0+$/, "");
    }
    return String(value);
  }

  function flatten(value) {
    if (!Array.isArray(value)) return null;
    const out = [];
    (function walk(item) {
      if (Array.isArray(item)) item.forEach(walk);
      else out.push(item);
    })(value);
    return out;
  }

  function vector(value) {
    const flat = flatten(value);
    return flat ? `[${flat.map((x) => fmt(x)).join(", ")}]` : "—";
  }

  function matrix(value, rows = 4, cols = 4) {
    if (!Array.isArray(value) || value.length === 0) return "—";
    if (Array.isArray(value[0])) {
      return `[${value.map((row) => {
        const flat = flatten(row) || [];
        return `[${flat.map((x) => fmt(x)).join(", ")}]`;
      }).join("; ")}]`;
    }
    const flat = flatten(value);
    if (!flat || flat.length !== rows * cols) return vector(value);
    const out = [];
    for (let i = 0; i < rows; i++) {
      out.push(`[${flat.slice(i * cols, (i + 1) * cols)
        .map((x) => fmt(x)).join(", ")}]`);
    }
    return `[${out.join("; ")}]`;
  }

  function pose(value) {
    if (!value || typeof value !== "object") return "—";
    return `(${fmt(value.x)}, ${fmt(value.y)}, ${fmt(value.theta)})`;
  }

  function yesNo(value) {
    return typeof value === "boolean" ? (value ? "true / PASS" : "false / FAIL") : "—";
  }

  function gateSummary(out, keys = ["score", "inlier", "mahal", "cov"]) {
    const gates = out && out.gates;
    if (!gates || typeof gates !== "object") return "事件未提供 gates";
    return keys
      .map((key) => `${key}=${typeof gates[key] === "boolean" ? (gates[key] ? "PASS" : "FAIL") : "—"}`)
      .join(" · ");
  }

  function matchSummary(match) {
    if (!match || typeof match !== "object") return "—";
    const parts = [];
    if (match.pose) parts.push(pose(match.pose));
    ["valid", "score", "inlierRatio", "minEig"].forEach((key) => {
      if (Object.prototype.hasOwnProperty.call(match, key)) {
        parts.push(`${key}=${fmt(match[key])}`);
      }
    });
    if (Object.prototype.hasOwnProperty.call(match, "covariance")) {
      parts.push(`cov=${matrix(match.covariance, 3, 3)}`);
    }
    return parts.length ? parts.join(" · ") : "match 已返回，但字段未识别";
  }

  function fillVars(host, rows) {
    if (!host) return;
    host.textContent = "";
    const fragment = document.createDocumentFragment();
    rows.forEach(([key, value]) => {
      const cell = document.createElement("div");
      cell.className = "hands-on-var";
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = key;
      dd.textContent = value;
      cell.appendChild(dt);
      cell.appendChild(dd);
      fragment.appendChild(cell);
    });
    host.appendChild(fragment);
  }

  function init() {
    if (window.PAGE_ID !== "ch07") return;
    const stageTitle = document.getElementById("ch07-stage-title");
    const stageMeta = document.getElementById("ch07-stage-meta");
    const stageExplain = document.getElementById("ch07-stage-explain");
    const stageCode = document.getElementById("ch07-stage-code");
    const stageIndex = document.getElementById("ch07-stage-index");
    const prev = document.getElementById("ch07-stage-prev");
    const next = document.getElementById("ch07-stage-next");
    const sourceBtn = document.getElementById("ch07-stage-source");
    const copyBtn = document.getElementById("ch07-stage-copy");
    const liveIter = document.getElementById("ch07-live-iter");
    const liveVars = document.getElementById("ch07-live-vars");
    if (!stageTitle || !stageCode || !liveVars) return;

    let stage = 0;
    let snapshot = null;

    function commonRows() {
      if (!snapshot) {
        return [
          ["动画帧", "尚未收到 ch07:step"],
          ["state / accepted", "—"],
          ["P4", "—"],
          ["trace", "先点击上方“步进 1 帧”或“连跑 15 帧”"],
        ];
      }
      const out = snapshot.out && typeof snapshot.out === "object" ? snapshot.out : {};
      const trace = out.trace && typeof out.trace === "object" ? out.trace : null;
      const rows = [
        ["iteration", fmt(snapshot.iteration)],
        ["state_before", fmt(snapshot.stateBefore)],
        ["state_after", fmt(out.state)],
        ["accepted", typeof out.accepted === "boolean" ? fmt(out.accepted) : "—"],
        ["reject_streak", fmt(out.rejectStreak !== undefined ? out.rejectStreak : out.reject_streak)],
        ["gyroBias b_g", fmt(out.gyroBias !== undefined ? out.gyroBias : out.gyro_bias)],
        ["pose_after", pose(snapshot.poseAfter || out.pose)],
        ["P4_after", matrix(snapshot.Pafter, 4, 4)],
      ];
      if (trace) {
        const missing = TRACE_FIELDS.filter((key) =>
          !Object.prototype.hasOwnProperty.call(trace, key));
        rows.push(["trace 字段", missing.length
          ? `缺少 ${missing.join(", ")}；对应变量显示 —`
          : "完整（K4/dx 在拒绝帧可为 null）"]);
      }
      return rows;
    }

    function liveRows() {
      const common = commonRows();
      if (!snapshot) return common;
      const out = snapshot.out && typeof snapshot.out === "object" ? snapshot.out : {};
      const trace = out.trace && typeof out.trace === "object" ? out.trace : null;
      if (!trace) {
        const entryState = snapshot.stateBefore || out.state;
        const recovery = RECOVERY_STATES.has(entryState);
        return [
          ...common,
          ["P4_before", matrix(snapshot.Pbefore, 4, 4)],
          ["trace", recovery
            ? "入口为恢复态（UNINIT/LOST/GLOBAL）：本帧不走正常 EKF 更新内核；恢复成功后会回到 TRACKING"
            : "out.trace 不存在：可能来自恢复旁路或旧版模拟器；状态与 P4 快照仍可阅读"],
          ["源码", recovery ? "localizer_core.cpp:L431-526" : "正常路径约定：localizer_core.cpp:L528-757"],
        ];
      }

      if (stage === 0) return [
        ...common,
        ["P4_before", matrix(snapshot.Pbefore, 4, 4)],
        ["pred", pose(trace.pred)],
      ];
      if (stage === 1) return [...common, ["F4", matrix(trace.F4, 4, 4)]];
      if (stage === 2) return [...common, ["Q4", matrix(trace.Q4, 4, 4)]];
      if (stage === 3) return [
        ...common,
        ["Ppred", matrix(trace.Ppred, 4, 4)],
        ["match", matchSummary(trace.match)],
      ];
      if (stage === 4) return [
        ...common,
        ["R", matrix(trace.R, 3, 3)],
        ["λmin(R)", fmt(out.minEig)],
        ["门 1 / 门 2 / 门 4", gateSummary(out, ["score", "inlier", "cov"])],
      ];
      if (stage === 5) return [
        ...common,
        ["innov ν", vector(trace.innov)],
        ["S", matrix(trace.S, 3, 3)],
        ["d²", fmt(out.mahal)],
        ["门 3 / 门 4", gateSummary(out, ["mahal", "cov"])],
      ];
      if (stage === 6) return [
        ...common,
        ["K4 (4×3)", matrix(trace.K4, 4, 3)],
        ["dx", vector(trace.dx)],
      ];
      if (stage === 7) return [
        ...common,
        ["trace.accepted", fmt(trace.accepted)],
        ["更新形式", trace.accepted === true ? "Joseph + 对称化" : "本帧未接受，无 Joseph 更新"],
      ];
      if (stage === 8) return [
        ...common,
        ["Ppred", matrix(trace.Ppred, 4, 4)],
        ["四门", gateSummary(out)],
        ["K4 / dx", trace.K4 == null && trace.dx == null
          ? "拒绝帧未计算或未提供"
          : `${matrix(trace.K4, 4, 3)} / ${vector(trace.dx)}`],
        ["拒绝语义", "P4_after 为纯预测；仅马氏拒绝时可能含 L711-716 的协方差松弛"],
      ];
      return [
        ...common,
        ["说明", "若 out.trace 存在，先按正常路径读取；恢复态没有 trace 时显示提示，不补造矩阵"],
      ];
    }

    function render() {
      const cur = STAGES[stage];
      stageTitle.textContent = cur.title;
      stageMeta.textContent = `src/localizer_core.cpp:L${cur.lines} · ${stage + 1}/${STAGES.length}`;
      stageIndex.textContent = `${stage + 1} / ${STAGES.length}`;
      stageExplain.textContent = cur.explain;
      stageCode.textContent = cur.code;
      stageCode.className = "language-cpp";
      delete stageCode.dataset.highlighted;
      if (window.hljs) {
        try { window.hljs.highlightElement(stageCode); } catch (_) {}
      }
      sourceBtn.setAttribute("data-file", "src/localizer_core.cpp");
      sourceBtn.setAttribute("data-lines", cur.lines);
      sourceBtn.setAttribute("data-label", `LocalizerCore::on_scan L${cur.lines}`);
      prev.disabled = stage === 0;
      next.disabled = stage === STAGES.length - 1;
      liveIter.textContent = snapshot
        ? `动画帧 #${fmt(snapshot.iteration)} · ${snapshot.out && snapshot.out.state ? snapshot.out.state : "状态未提供"}`
        : "尚未收到动画帧；下方变量等待 ch07:step";
      fillVars(liveVars, liveRows());
    }

    prev.addEventListener("click", () => {
      if (stage > 0) { stage--; render(); }
    });
    next.addEventListener("click", () => {
      if (stage < STAGES.length - 1) { stage++; render(); }
    });
    copyBtn.addEventListener("click", () => {
      if (navigator.clipboard) {
        navigator.clipboard.writeText(stageCode.textContent).catch(() => {});
      }
      const old = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = old; }, 1200);
    });

    window.addEventListener("ch07:step", (event) => {
      const detail = event.detail && typeof event.detail === "object" ? event.detail : {};
      snapshot = { ...detail, out: detail.out && typeof detail.out === "object" ? detail.out : {} };
      render();
    });
    window.addEventListener("ch07:reset", () => {
      snapshot = null;
      render();
    });

    document.querySelectorAll(".hands-on-exercise").forEach((form) => {
      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const input = form.querySelector(".exercise-input");
        const feedback = form.querySelector(".exercise-feedback");
        if (!input || !feedback) return;
        const raw = input.value.trim().replace(/\s+/g, "");
        const kind = form.dataset.kind || "number";
        let ok = false;
        if (kind === "choice") {
          ok = raw.toLowerCase() === String(form.dataset.answer || "").toLowerCase();
        } else {
          const value = Number(raw);
          const tolerance = Number(form.dataset.tolerance || 1e-6);
          ok = raw !== "" && Number.isFinite(value) &&
            Math.abs(value - Number(form.dataset.answer)) <= tolerance;
        }
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续尝试下一题。")
          : (form.dataset.hint || "还不对：回到公式与决策表，先写出该项再代入。");
      });
    });

    render();
  }

  window.addEventListener("sloc:ready", init);
})();
