// Ch01 hands-on layer: shared SE(2) numeric example, real source stepper and exercises.
(function () {
  "use strict";

  const SOURCE_STAGES = [
    {
      title: "0 · 角度归一化：先拒绝非有限输入",
      file: "include/scan_localizer/se2.hpp",
      lines: "30-46",
      explain: "生产版把 π 与 2π 定义为常量，并用 std::remainder 做 O(1) 归一化。NaN/inf 会先返回 0.0；边界 -π 再加 2π，因此最终区间是 (-π, π]，与 operator* 的角度契约一致。",
      code: `inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * kPi;

// O(1) 安全角度归一化至 (-kPi, kPi]；非有限输入（±inf / NaN）返回 0.0
inline double wrap_angle(double a)
{
  if (!std::isfinite(a)) {
    return 0.0;
  }
  double r = std::remainder(a, kTwoPi);
  if (r <= -kPi) {
    r += kTwoPi;
  } else if (r > kPi) {
    r -= kTwoPi;
  }
  return r;
}`,
    },
    {
      title: "1 · Pose2：复合与群逆都来自同一个 R(θ)",
      file: "include/scan_localizer/se2.hpp",
      lines: "49-69",
      explain: "operator* 用左位姿的 c/s 旋转右位姿平移，因此顺序不可交换。inverse 使用 -Rᵀt 与 -θ；trans() 只把平移导出给 Eigen 调用方，不改变位姿语义。",
      code: `struct Pose2
{
  double x{0.0}, y{0.0}, theta{0.0};

  Pose2() = default;
  Pose2(double x_, double y_, double th_)
  : x(x_), y(y_), theta(th_) {}

  Pose2 operator*(const Pose2 & o) const     // 本系位姿 o 复合到本位姿
  {
    const double c = std::cos(theta), s = std::sin(theta);
    return {x + c * o.x - s * o.y, y + s * o.x + c * o.y,
      wrap_angle(theta + o.theta)};
  }
  Pose2 inverse() const
  {
    const double c = std::cos(theta), s = std::sin(theta);
    return {-(c * x + s * y), -(-s * x + c * y), wrap_angle(-theta)};
  }
  Eigen::Vector2d trans() const {return {x, y};}
};`,
    },
    {
      title: "2 · transform_scan：旋转平移每个点，并复用输出缓冲",
      file: "include/scan_localizer/se2.hpp",
      lines: "71-93",
      explain: "Vec2 是 Eigen::Vector2d，Scan 只保存传感器系点。核心重载先 clear，再为随机访问迭代器 reserve，最后对每个点执行 R(pose.θ)p + t。clear 保留容量，适合每帧复用。",
      code: `using Vec2 = Eigen::Vector2d;

// ---------- 扫描 ----------
struct Scan
{
  std::vector<Vec2> points;    // 传感器系直角坐标点
};

// 传感器系点云 → 世界系（缓冲复用重载，避免热循环重复分配）
template<typename It>
inline void transform_scan(
  const Pose2 & pose, It begin, It end, std::vector<Vec2> & out)
{
  out.clear();
  if constexpr (std::random_access_iterator<It>) {
    out.reserve(static_cast<std::size_t>(std::distance(begin, end)));
  }
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  for (It it = begin; it != end; ++it) {
    const double px = it->x(), py = it->y();
    out.emplace_back(pose.x + c * px - s * py, pose.y + s * px + c * py);
  }
}`,
    },
    {
      title: "3 · 三个便利重载：统一委托给缓冲复用版",
      file: "include/scan_localizer/se2.hpp",
      lines: "95-114",
      explain: "返回向量的两个重载只负责创建局部 out 并转调核心实现；Scan 重载再委托给迭代器重载。这样热路径与便利 API 共用一份等式，不会出现两份逐渐漂移的实现。",
      code: `template<typename It>
inline std::vector<Vec2> transform_scan(const Pose2 & pose, It begin, It end)
{
  std::vector<Vec2> out;
  transform_scan(pose, begin, end, out);
  return out;
}

inline void transform_scan(
  const Pose2 & pose, const Scan & scan, std::vector<Vec2> & out)
{
  transform_scan(pose, scan.points.begin(), scan.points.end(), out);
}

inline std::vector<Vec2> transform_scan(const Pose2 & pose, const Scan & scan)
{
  std::vector<Vec2> out;
  transform_scan(pose, scan, out);
  return out;
}`,
    },
    {
      title: "4 · 生产落点：last_pose_ * odom_at_last_scan_.inverse()",
      file: "src/localizer_core.cpp",
      lines: "268-275",
      explain: "map_to_odom 不在 se2.hpp 中，而是 LocalizerCore 的使用点。last_pose_ 是扫描时刻的 T_map_base，odom_at_last_scan_ 是同刻 T_odom_base；没有里程计时返回 last_pose_，相当于修正量取单位位姿。",
      code: `Pose2 LocalizerCore::map_to_odom() const
{
  if (!has_odom_) {
    return last_pose_;
  }
  // T_map_odom = T_map_base(t_scan) * inv(T_odom_base(t_scan))
  return last_pose_ * odom_at_last_scan_.inverse();
}`,
    },
  ];

  function escapeHtml(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#39;");
  }

  function fmt(value) {
    if (value === null || value === undefined) return "—";
    if (typeof value !== "number") return String(value);
    if (!Number.isFinite(value)) return String(value);
    if (Object.is(value, -0)) return "0";
    const av = Math.abs(value);
    if (av !== 0 && av < 1e-4) return value.toExponential(3);
    return value.toFixed(6).replace(/\.?0+$/, "");
  }

  function pose(value) {
    return value ? `(${fmt(value.x)}, ${fmt(value.y)}, ${fmt(value.theta)})` : "—";
  }

  function point(value) {
    return value ? `(${fmt(value.x)}, ${fmt(value.y)})` : "—";
  }

  function fillVars(host, rows) {
    if (!host) return;
    host.innerHTML = rows.map(([key, value]) =>
      `<div class="hands-on-var"><dt>${escapeHtml(key)}</dt><dd>${escapeHtml(value)}</dd></div>`).join("");
  }

  function highlight(code) {
    if (!code) return;
    code.className = "language-cpp";
    delete code.dataset.highlighted;
    if (window.hljs) {
      try { window.hljs.highlightElement(code); } catch (_) {}
    }
  }

  function init() {
    if (!window.SlocSim || window.PAGE_ID !== "ch01") return;
    const S = window.SlocSim;

    // One example feeds the numeric inspector and the first two exercises.
    const A = { x: 1, y: 0, theta: Math.PI / 2 };
    const B = { x: 1, y: 0, theta: 0 };
    const mapToBase = { x: 2, y: 1, theta: Math.PI / 2 };
    const odomToBase = { x: 1, y: 0, theta: 0 };
    const basePoint = { x: 1, y: 0.5 };

    const composed = S.poseMul(A, B);
    const inverseMapToBase = S.poseInv(mapToBase);
    const mapToOdom = S.mapToOdom(mapToBase, odomToBase);
    const reconstructed = S.poseMul(mapToOdom, odomToBase);
    const rightIdentity = S.poseMul(mapToBase, inverseMapToBase);
    const leftIdentity = S.poseMul(inverseMapToBase, mapToBase);
    const odomPoint = S.transformScan(odomToBase, [basePoint])[0];
    const repairedPoint = S.transformScan(mapToOdom, [odomPoint])[0];
    const directMapPoint = S.transformScan(mapToBase, [basePoint])[0];
    const pointError = Math.hypot(
      repairedPoint.x - directMapPoint.x,
      repairedPoint.y - directMapPoint.y);
    const poseError = Math.max(
      Math.abs(reconstructed.x - mapToBase.x),
      Math.abs(reconstructed.y - mapToBase.y),
      Math.abs(reconstructed.theta - mapToBase.theta));

    fillVars(document.getElementById("ch01-numeric-vars"), [
      ["A = T₁", pose(A)],
      ["B = T₂", pose(B)],
      ["S.poseMul(A, B)", pose(composed)],
      ["M = S.poseInv 的输入", pose(mapToBase)],
      ["S.poseInv(M)", pose(inverseMapToBase)],
      ["O = T_odom_base", pose(odomToBase)],
      ["C = S.mapToOdom(M, O)", pose(mapToOdom)],
      ["S.poseMul(C, O)（回代 M）", pose(reconstructed)],
      ["M * M⁻¹ / M⁻¹ * M", `${pose(rightIdentity)} / ${pose(leftIdentity)}`],
      ["p_base", point(basePoint)],
      ["S.transformScan(O, [p_base])", point(odomPoint)],
      ["C · (O · p_base)", point(repairedPoint)],
      ["S.transformScan(M, [p_base])", point(directMapPoint)],
      ["点链 / 位姿回代最大误差", `${fmt(pointError)} / ${fmt(poseError)}`],
    ]);

    const ex1 = document.getElementById("ch01-ex1");
    const ex2 = document.getElementById("ch01-ex2");
    if (ex1) ex1.closest(".hands-on-exercise").dataset.answer = String(composed.y);
    if (ex2) ex2.closest(".hands-on-exercise").dataset.answer = String(inverseMapToBase.x);

    const stageTitle = document.getElementById("ch01-stage-title");
    const stageMeta = document.getElementById("ch01-stage-meta");
    const stageExplain = document.getElementById("ch01-stage-explain");
    const stageCode = document.getElementById("ch01-stage-code");
    const stageIndex = document.getElementById("ch01-stage-index");
    const stagePrev = document.getElementById("ch01-stage-prev");
    const stageNext = document.getElementById("ch01-stage-next");
    const sourceBtn = document.getElementById("ch01-stage-source");
    const copyBtn = document.getElementById("ch01-stage-copy");
    let sourceStage = 0;

    function renderSource() {
      const stage = SOURCE_STAGES[sourceStage];
      if (stageTitle) stageTitle.textContent = stage.title;
      if (stageMeta) stageMeta.textContent = `${stage.file}:L${stage.lines} · 真实仓库源码`;
      if (stageExplain) stageExplain.textContent = stage.explain;
      if (stageCode) { stageCode.textContent = stage.code; highlight(stageCode); }
      if (stageIndex) stageIndex.textContent = `${sourceStage + 1} / ${SOURCE_STAGES.length}`;
      if (stagePrev) stagePrev.disabled = sourceStage === 0;
      if (stageNext) stageNext.disabled = sourceStage === SOURCE_STAGES.length - 1;
      if (sourceBtn) {
        sourceBtn.setAttribute("data-file", stage.file);
        sourceBtn.setAttribute("data-lines", stage.lines);
        sourceBtn.setAttribute("data-label", `${stage.file} L${stage.lines}`);
      }
    }

    if (stagePrev) stagePrev.addEventListener("click", () => {
      if (sourceStage > 0) { sourceStage--; renderSource(); }
    });
    if (stageNext) stageNext.addEventListener("click", () => {
      if (sourceStage < SOURCE_STAGES.length - 1) { sourceStage++; renderSource(); }
    });
    if (copyBtn) copyBtn.addEventListener("click", () => {
      if (navigator.clipboard && stageCode) {
        navigator.clipboard.writeText(stageCode.textContent).catch(() => {});
      }
      const old = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = old; }, 1200);
    });

    document.querySelectorAll(".hands-on-exercise").forEach((form) => {
      const input = form.querySelector(".exercise-input");
      const feedback = form.querySelector(".exercise-feedback");
      if (!input || !feedback) return;
      const kind = form.dataset.kind || "number";

      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const raw = input.value.trim().replace(/\s+/g, "");
        let ok = false;
        if (kind === "number") {
          const value = Number(raw);
          const expected = Number(form.dataset.answer);
          const tolerance = Number(form.dataset.tolerance || 0);
          ok = raw !== "" && Number.isFinite(value) && Math.abs(value - expected) <= tolerance;
        } else if (kind === "choice") {
          ok = raw !== "" && raw.toLowerCase() ===
            String(form.dataset.answer).replace(/\s+/g, "").toLowerCase();
        }
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续尝试下一题。")
          : (form.dataset.hint || "还不对：回到数值例，先写出公式再代入。");
      });

      input.addEventListener("input", () => {
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
      });
    });

    renderSource();
  }

  window.addEventListener("sloc:ready", init);
})();
