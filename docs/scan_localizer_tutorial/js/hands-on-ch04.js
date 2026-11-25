// Ch04 hands-on layer: source stepper, live variable inspector and fill-in exercises.
(function () {
  "use strict";

  const STAGES = [
    {
      title: "0 · 先守住输入契约",
      lines: "279-283",
      explain: "第一版不需要复杂优化，但必须先拒绝无效地图、非有限位姿和少于 5 点的扫描。否则后面的 LDLT 解算只是在放大垃圾输入。",
      code: `MatchResult res;
if (!field.valid() || !pose_is_finite(prior)) { return res; }
const auto * grid = &field.grid();
const int n = static_cast<int>(scan.points.size());
if (n < 5) { return res; }`,
    },
    {
      title: "1 · 评估当前位姿，累积 H、b 与加权残差",
      lines: "285-318",
      explain: "把“给定位姿打分”封装成 eval_pose。每个有效点贡献一个三维雅可比 J；H 记录信息量，b 记录当前残差造成的梯度，ravg 只用于比较位姿优劣。",
      code: `const double wx = pose.x + c * lx - s * ly;
const double wy = pose.y + s * lx + c * ly;
const auto g = grid->worldToGridF(wx, wy);
Eigen::Vector2d grad;
const double r = field.sdf(g.x(), g.y(), &grad);
if (std::abs(r) > opt.trim_dist) { continue; }
const double ar = std::abs(r);
const double w = ar <= opt.huber_delta ? 1.0
  : opt.huber_delta / std::max(ar, 1e-9);
const double jth = grad.x() * (-(wy - pose.y))
  + grad.y() * (wx - pose.x);
const Eigen::Vector3d J(grad.x(), grad.y(), jth);
H += w * J * J.transpose();
b += w * J * r;
rsum += w * r * r;
rw += w;`,
    },
    {
      title: "2 · 应用步长前，先记录历史最优",
      lines: "320-342",
      explain: "LM 阻尼可能让某一步越过谷底。因此每轮先保存当前 p 的成绩，再尝试更新。注意 best_H 必须与 best_pose 对应，不能最后一次求解完就随手覆盖。",
      code: `Pose2 p(prior.x, prior.y, wrap_angle(prior.theta));
bool has_valid_iter = false;
double best_ravg = std::numeric_limits<double>::infinity();
Pose2 best_pose = p;
Eigen::Matrix3d best_H = Eigen::Matrix3d::Zero();

// 每轮 eval_pose(p, ...) 之后、更新 p 之前：
if (!has_valid_iter || ravg < best_ravg) {
  has_valid_iter = true;
  best_ravg = ravg;
  best_pose = p;
  best_H = H;
  best_var = std::max(ravg, 1e-6);
}`,
    },
    {
      title: "3 · 阻尼只用于求解，不用于协方差",
      lines: "344-348",
      explain: "Hd = H + λI 让奇异问题可解；但 λI 会把不可观测方向人为抬高。若把 Hd 拿去算协方差，长走廊会假装自己沿走廊也很准。",
      code: `Eigen::Matrix3d Hd = H;
Hd.diagonal().array() +=
  1e-6 * H.diagonal().array().max(1.0);
const Eigen::Vector3d dp = Hd.ldlt().solve(-b);
if (!dp.allFinite()) { break; }`,
    },
    {
      title: "4 · 限幅后更新位姿，并判断收敛",
      lines: "349-361",
      explain: "先算未截断步长，再统一缩放平移和角度。收敛判断使用的是“实际执行后的步长”，不是原始 dp；否则被护栏截断的大步会被误判为已收敛。",
      code: `const double nl = dp.head<2>().norm();
const double s_clip = std::min(1.0,
  std::min(0.2 / std::max(nl, 1e-9),
           0.1 / std::max(std::abs(dp(2)), 1e-9)));
p.x += s_clip * dp(0);
p.y += s_clip * dp(1);
p.theta = wrap_angle(p.theta + s_clip * dp(2));
if (nl * s_clip < opt.converg_t &&
    std::abs(dp(2)) * s_clip < opt.converg_r) {
  break;
}`,
    },
    {
      title: "5 · 循环结束后补评估最后一跳",
      lines: "369-382",
      explain: "for 循环里的 best_pose 属于“更新前”的 p。最后一跳发生后就 break 时，新位姿还没有成绩。必须再调用一次 eval_pose(p)，否则会稳定丢掉最后一次改进。",
      code: `Eigen::Matrix3d H_end;
Eigen::Vector3d b_end;
double ravg_end = 0.0;
int used_end = 0;
eval_pose(p, H_end, b_end, ravg_end, used_end);
if (used_end >= 5 && ravg_end < best_ravg) {
  best_ravg = ravg_end;
  best_pose = p;
  best_H = H_end;
  best_var = std::max(ravg_end, 1e-6);
}`,
    },
    {
      title: "6 · 用最优位姿对应的未阻尼 H 输出协方差",
      lines: "384-389",
      explain: "位姿、协方差、score 必须来自同一个 best_pose。协方差用 best_H，而不是用于求解的 Hd；最后再独立统计 score 与 inlier_ratio。",
      code: `res.pose = best_pose;
res.covariance = cov_from_hessian(best_H, best_var);
eval_stats(field, res.pose, scan, opt.inlier_gate,
           res.score, res.inlier_ratio);
res.valid = true;
return res;`,
    },
  ];

  function fmt(v, digits = 4) {
    if (v === null || v === undefined) return "—";
    if (typeof v === "number" && !Number.isFinite(v)) return String(v);
    if (typeof v === "number") {
      const av = Math.abs(v);
      if (av !== 0 && (av < 1e-3 || av >= 1e4)) return v.toExponential(2);
      return v.toFixed(digits).replace(/\.?0+$/, "");
    }
    return String(v);
  }

  function vec(v) {
    return v ? `[${v.map((x) => fmt(x)).join(", ")}]` : "—";
  }

  function mat(m) {
    if (!m) return "—";
    return `[${m.slice(0, 3).map(fmt).join(", ")}; ${m.slice(3, 6).map(fmt).join(", ")}; ${m.slice(6, 9).map(fmt).join(", ")}]`;
  }

  function pose(p) {
    return p ? `(${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.theta)})` : "—";
  }

  function fillVars(host, rows) {
    if (!host) return;
    host.innerHTML = rows.map(([k, v]) =>
      `<div class="hands-on-var"><dt>${k}</dt><dd>${v}</dd></div>`).join("");
  }

  function init() {
    if (!window.SlocSim || window.PAGE_ID !== "ch04") return;
    const S = window.SlocSim;
    const example = S.createFieldGnTeachingExample();
    const stageTitle = document.getElementById("ch04-stage-title");
    const stageMeta = document.getElementById("ch04-stage-meta");
    const stageExplain = document.getElementById("ch04-stage-explain");
    const stageCode = document.getElementById("ch04-stage-code");
    const stageIndex = document.getElementById("ch04-stage-index");
    const prev = document.getElementById("ch04-stage-prev");
    const next = document.getElementById("ch04-stage-next");
    const sourceBtn = document.getElementById("ch04-stage-source");
    const copyBtn = document.getElementById("ch04-stage-copy");
    const liveIter = document.getElementById("ch04-live-iter");
    let stage = 0;
    let trace = null;

    const st = example.step;
    fillVars(document.getElementById("ch04-example-vars"), [
      ["p_initial", pose(example.pose)],
      ["r_i / grad", `${fmt(st.samplePoint.r)} m / ${vec([st.samplePoint.J[0], st.samplePoint.J[1]])}`],
      ["w_i", fmt(st.samplePoint.w)],
      ["H", mat(st.H)],
      ["b", vec(st.b)],
      ["dp", vec(st.dp)],
      ["p_next", pose(st.nextPose)],
      ["ravg", `${fmt(st.ravg)} m²`],
    ]);

    function liveRows() {
      const cur = trace && trace.step;
      if (!cur) return [["状态", "先点击上方“迭代 1 步”"], ["p", "—"], ["used / ravg", "—"]];
      const common = [["p_before", pose(trace.poseBefore || null)]];
      if (stage === 0) return [...common, ["used", cur.used], ["ok", cur.ok]];
      if (stage === 1) return [
        ...common,
        ["sample r / w", cur.samplePoint ? `${fmt(cur.samplePoint.r)} / ${fmt(cur.samplePoint.w)}` : "—"],
        ["sample J", cur.samplePoint ? vec(cur.samplePoint.J) : "—"],
        ["H", mat(cur.H)],
        ["ravg", fmt(cur.ravg)],
      ];
      if (stage === 2) return [
        ["best_ravg", fmt(trace.bestRavg)],
        ["best_pose", pose(trace.bestPose)],
        ["updated_best", trace.updatedBest ? "是" : "否"],
      ];
      if (stage === 3) return [["b", vec(cur.b)], ["Hd", mat(cur.Hd)], ["dp", vec(cur.dp)]];
      if (stage === 4) return [
        ["clip", fmt(cur.clip)],
        ["p_next", pose(cur.nextPose)],
        ["stepNorm", cur.ok ? `${fmt(cur.stepNorm)} m` : "—"],
        ["stepAng", cur.ok ? `${fmt(cur.stepAng)} rad` : "—"],
      ];
      if (stage === 5) return [
        ["best_ravg", fmt(trace.bestRavg)],
        ["best_pose", pose(trace.bestPose)],
        ["说明", "动画未模拟终态补评估；生产实现见本阶段源码"],
      ];
      return [
        ["best_H", mat(trace.bestH)],
        ["best_pose", pose(trace.bestPose)],
        ["说明", "协方差由 best_H 计算，不能用 Hd"],
      ];
    }

    function render() {
      const cur = STAGES[stage];
      stageTitle.textContent = cur.title;
      stageMeta.textContent = `src/matchers.cpp:L${cur.lines} · ${stage + 1}/${STAGES.length}`;
      stageIndex.textContent = `${stage + 1} / ${STAGES.length}`;
      stageExplain.textContent = cur.explain;
      stageCode.textContent = cur.code;
      stageCode.className = "language-cpp";
      delete stageCode.dataset.highlighted;
      if (window.hljs) {
        try { window.hljs.highlightElement(stageCode); } catch (_) {}
      }
      sourceBtn.setAttribute("data-file", "src/matchers.cpp");
      sourceBtn.setAttribute("data-lines", cur.lines);
      sourceBtn.setAttribute("data-label", `FieldGnMatcher::match L${cur.lines}`);
      prev.disabled = stage === 0;
      next.disabled = stage === STAGES.length - 1;
      liveIter.textContent = trace ? `当前动画迭代 #${trace.iteration}` : "尚未运行动画迭代";
      fillVars(document.getElementById("ch04-live-vars"), liveRows());
    }

    prev.addEventListener("click", () => { if (stage > 0) { stage--; render(); } });
    next.addEventListener("click", () => { if (stage < STAGES.length - 1) { stage++; render(); } });
    copyBtn.addEventListener("click", () => {
      navigator.clipboard?.writeText(stageCode.textContent).catch(() => {});
      const old = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = old; }, 1200);
    });

    window.addEventListener("ch04:step", (e) => {
      trace = { ...e.detail, poseBefore: trace && trace.pose ? trace.pose : example.pose };
      // sim-ch04 sends the post-step pose; the evaluated pose is recovered from dp.
      if (trace.step && trace.step.ok && trace.step.dp) {
        const d = trace.step.dp, c = trace.step.clip;
        trace.poseBefore = {
          x: trace.pose.x - c * d[0],
          y: trace.pose.y - c * d[1],
          theta: S.wrapAngle(trace.pose.theta - c * d[2]),
        };
      }
      render();
    });
    window.addEventListener("ch04:reset", () => { trace = null; render(); });

    document.querySelectorAll(".hands-on-exercise").forEach((form) => {
      form.addEventListener("submit", (e) => {
        e.preventDefault();
        const input = form.querySelector(".exercise-input");
        const feedback = form.querySelector(".exercise-feedback");
        const raw = input.value.trim().replace(/\s+/g, "");
        const kind = form.dataset.kind || "number";
        let ok = false;
        if (kind === "choice") {
          ok = raw.toLowerCase() === form.dataset.answer.toLowerCase();
        } else {
          const value = Number(raw);
          const tolerance = Number(form.dataset.tolerance || 1e-6);
          ok = raw !== "" && Number.isFinite(value) && Math.abs(value - Number(form.dataset.answer)) <= tolerance;
        }
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok ? "正确。继续尝试下一题。" : "还不对：回到上面的单步数值演算，先写出该项的公式再代入。";
      });
    });

    render();
  }

  window.addEventListener("sloc:ready", init);
})();
