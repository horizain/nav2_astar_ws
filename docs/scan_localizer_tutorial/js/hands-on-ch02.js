// Ch02 hands-on layer: live EDT trace, real source stepper and fill-in exercises.
(function () {
  "use strict";

  const INPUT = [9, 100, 4];
  const SOURCE_FILE = "src/map_field.cpp";
  const SOURCE_STAGES = [
    {
      title: "0 · 先把负数交点向下取整",
      lines: "35-43",
      explain: "C++ 的整数除法对负数向零截断，而抛物线分界需要数学意义上的 floor。这里同时检查余数与分母符号；生产代码和教学最小版共用同一条契约。",
      code: `inline std::int64_t floor_div(std::int64_t num, std::int64_t den)
{
  std::int64_t q = num / den;
  const std::int64_t r = num % den;
  if (r != 0 && ((r < 0) != (den < 0))) {
    --q;
  }
  return q;
}`,
    },
    {
      title: "1 · 2D 输入：种子为零，其余为列向代价",
      lines: "45-58",
      explain: "edt1d 只处理一维平方代价；edt2d 先用列向扫描得到每个格子到最近种子的曼哈顿步数 g，再把 g² 交给逐行 edt1d。距离可能超过 int32，全程使用 int64。",
      code: `// Meijster 精确欧氏距离变换：seeds[i] 为 true 的单元距离为 0（平方距离，单元系）
void edt2d(
  int width, int height, const std::vector<uint8_t> & seeds,
  std::vector<double> & dist2_out)
{
  const std::size_t w = static_cast<std::size_t>(width);
  const std::size_t h = static_cast<std::size_t>(height);
  dist2_out.assign(w * h, 0.0);
  if (width <= 0 || height <= 0) {return;}

  // 距离值可能达 width+height，平方后超 int32 → 全程 int64 防溢出
  const std::int64_t INF = static_cast<std::int64_t>(width) +
    static_cast<std::int64_t>(height) + 2;
  std::vector<std::int64_t> g(w * h, INF);`,
    },
    {
      title: "2 · 第一遍：每列正向 + 反向扫描",
      lines: "60-73",
      explain: "一维种子到格子的最短步数，正向转发后再反向转发即可得到。此时 g 仍是曼哈顿步数，不是最终欧氏距离；列扫描把二维问题降成每行独立的一维输入。",
      code: `  // Pass 1：逐列扫描
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const std::size_t i = static_cast<std::size_t>(y) * w +
        static_cast<std::size_t>(x);
      g[i] = seeds[i] ? 0 :
        (y > 0 ? std::min(INF, g[i - w] + 1) : INF);
    }
    for (int y = height - 2; y >= 0; --y) {
      const std::size_t i = static_cast<std::size_t>(y) * w +
        static_cast<std::size_t>(x);
      g[i] = std::min(g[i], g[i + w] + 1);
    }
  }`,
    },
    {
      title: "3 · 第二遍：逐行调用 edt1d",
      lines: "75-89",
      explain: "对固定行 y，令 f[x] = g[y,x]²，edt1d 求 minₓ((x-p)² + g[y,p]²)。横向与纵向平方项因此精确相加，输出仍是格子单位的平方距离。",
      code: `  // Pass 2：逐行下包络
  std::vector<std::int64_t> f(w), row_out(w);
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * w;
    for (int x = 0; x < width; ++x) {
      const std::int64_t gx = g[row + static_cast<std::size_t>(x)];
      f[static_cast<std::size_t>(x)] = gx * gx;
    }
    edt1d(f, row_out);
    for (int x = 0; x < width; ++x) {
      dist2_out[row + static_cast<std::size_t>(x)] =
        static_cast<double>(row_out[static_cast<std::size_t>(x)]);
    }
  }`,
    },
    {
      title: "4 · edt1d：初始化顶点栈与分界栈",
      lines: "93-110",
      explain: "v[k] 保存抛物线的顶点下标 p，不是 f[p] 或距离值；z[k] 保存 v[k-1] 与 v[k] 在整数轴上的分界。两端哨兵覆盖整个定义域。",
      code: `void edt1d(
  const std::vector<std::int64_t> & f,
  std::vector<std::int64_t> & out)
{
  const std::size_t n = f.size();
  out.assign(n, 0);
  if (n == 0) {return;}

  constexpr std::int64_t kZNeg = -(std::int64_t{1} << 60);
  constexpr std::int64_t kZPos = +(std::int64_t{1} << 60);

  std::vector<std::int64_t> v(n, 0);
  std::vector<std::int64_t> z(n + 1, 0);
  std::int64_t k = 0;
  v[0] = 0;
  z[0] = kZNeg;
  z[1] = kZPos;`,
    },
    {
      title: "5 · 构建下包络：算 s，弹栈，再入栈",
      lines: "111-128",
      explain: "每条新抛物线 q 先与栈顶 p 求 floor 交点 s。如果 s <= z[k]，栈顶在整数定义域内已没有独占格，立即弹出并重算；否则把 q 压入 v，把 s 写入新的 z[k]。",
      code: `  for (std::size_t qi = 1; qi < n; ++qi) {
    const std::int64_t q = static_cast<std::int64_t>(qi);
    std::int64_t p = v[static_cast<std::size_t>(k)];
    std::int64_t s = floor_div(
      (f[qi] + q * q) - (f[static_cast<std::size_t>(p)] + p * p),
      2 * (q - p));
    while (s <= z[static_cast<std::size_t>(k)]) {
      --k;
      p = v[static_cast<std::size_t>(k)];
      s = floor_div(
        (f[qi] + q * q) - (f[static_cast<std::size_t>(p)] + p * p),
        2 * (q - p));
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = kZPos;
  }`,
    },
    {
      title: "6 · 查询下包络：分界指针只向右走",
      lines: "130-138",
      explain: "查询时从 k=0 开始，只要下一个分界仍小于 q 就前进一步。选中的顶点 p 给出 out[q]=(q-p)²+f[p]。构建与查询合计都是线性扫描。",
      code: `  k = 0;
  for (std::size_t qi = 0; qi < n; ++qi) {
    const std::int64_t q = static_cast<std::int64_t>(qi);
    while (z[static_cast<std::size_t>(k + 1)] < q) {++k;}
    const std::int64_t p = v[static_cast<std::size_t>(k)];
    const std::int64_t df = q - p;
    out[qi] = df * df + f[static_cast<std::size_t>(p)];
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
    if (value === -1e12) return "-∞";
    if (value === 1e12) return "+∞";
    return String(value);
  }

  function vec(values) {
    return values ? `[${values.map(fmt).join(", ")}]` : "—";
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

  function parseVector(raw) {
    return raw.replace(/^\[/, "").replace(/\]$/, "").split(",").map((x) => Number(x.trim()));
  }

  function init() {
    if (!window.SlocSim || window.PAGE_ID !== "ch02") return;
    const S = window.SlocSim;
    const trace = S.edt1dWithTrace(INPUT);
    const directOut = S.edt1d(INPUT);
    const floorExercise = document.getElementById("ch02-ex1");
    if (floorExercise && floorExercise.closest) {
      // In this trace, the accepted q=2 / p=0 boundary is exactly floor(-1, 4).
      floorExercise.closest(".hands-on-exercise").dataset.answer = String(trace.steps[2].z[1]);
    }

    // The numeric panel is populated from the existing math core, not a second EDT.
    fillVars(document.getElementById("ch02-numeric-vars"), [
      ["f", vec(INPUT)],
      ["S.edt1d(f)", vec(directOut)],
      ["trace.out", vec(trace.out)],
      ["q=1 时的 s", trace.steps[1].z[1]],
      ["q=2 最终接受的 s", trace.steps[2].z[1]],
      ["q=2 的 popped", vec(trace.steps[2].popped)],
      ["最终 v[]", vec(trace.v)],
      ["最终 z[]", vec(trace.z)],
    ]);

    const traceTitle = document.getElementById("ch02-trace-title");
    const traceMeta = document.getElementById("ch02-trace-meta");
    const traceExplain = document.getElementById("ch02-trace-explain");
    const traceIndex = document.getElementById("ch02-trace-index");
    const tracePrev = document.getElementById("ch02-trace-prev");
    const traceNext = document.getElementById("ch02-trace-next");
    const traceReset = document.getElementById("ch02-trace-reset");
    let traceStep = 0;

    function renderTrace() {
      const step = trace.steps[traceStep];
      if (traceTitle) traceTitle.textContent = `扫描 q=${step.q}：${step.popped.length ? `弹出顶点 ${vec(step.popped)}` : "没有顶点被弹出"}`;
      if (traceMeta) traceMeta.textContent = `S.edt1dWithTrace([9, 100, 4]).steps[${traceStep}]`;
      if (traceIndex) traceIndex.textContent = `${traceStep + 1} / ${trace.steps.length}`;
      if (tracePrev) tracePrev.disabled = traceStep === 0;
      if (traceNext) traceNext.disabled = traceStep === trace.steps.length - 1;

      let explain;
      if (step.q === 0) {
        explain = "初始化：v[0]=0，z[0]=-∞，z[1]=+∞。第一条抛物线覆盖整个整数轴，因此 s 尚不存在。";
      } else if (step.q === 1) {
        explain = "q=1 与 p=0 的最终交点是 s=46。因为 46 ≤ z[0]=-∞ 不成立，顶点 0 保留，顶点 1 入栈。";
      } else {
        explain = "q=2 先弹出 trace 记录的 popped=[1]，再与 p=0 比较，最终接受 s=-1。因为 -1 ≤ z[0]=-∞ 不成立，顶点 0 保留，顶点 2 入栈。";
      }
      if (traceExplain) traceExplain.textContent = explain;

      fillVars(document.getElementById("ch02-trace-vars"), [
        ["q", step.q],
        ["f[q]", INPUT[step.q]],
        ["v[]", vec(step.v)],
        ["z[]", vec(step.z)],
        ["popped", vec(step.popped)],
        ["s", step.q === 0 ? "—（初始化）" : `${step.z[step.z.length - 2]}（最终接受值）`],
        ["栈顶 p = v[k]", step.v[step.v.length - 1]],
        ["out[q]（完整结果）", trace.out[step.q]],
      ]);
    }

    if (tracePrev) tracePrev.addEventListener("click", () => {
      if (traceStep > 0) { traceStep--; renderTrace(); }
    });
    if (traceNext) traceNext.addEventListener("click", () => {
      if (traceStep < trace.steps.length - 1) { traceStep++; renderTrace(); }
    });
    if (traceReset) traceReset.addEventListener("click", () => { traceStep = 0; renderTrace(); });

    const stageTitle = document.getElementById("ch02-stage-title");
    const stageMeta = document.getElementById("ch02-stage-meta");
    const stageExplain = document.getElementById("ch02-stage-explain");
    const stageCode = document.getElementById("ch02-stage-code");
    const stageIndex = document.getElementById("ch02-stage-index");
    const stagePrev = document.getElementById("ch02-stage-prev");
    const stageNext = document.getElementById("ch02-stage-next");
    const sourceBtn = document.getElementById("ch02-stage-source");
    const copyBtn = document.getElementById("ch02-stage-copy");
    let sourceStage = 0;

    function renderSource() {
      const stage = SOURCE_STAGES[sourceStage];
      if (stageTitle) stageTitle.textContent = stage.title;
      if (stageMeta) stageMeta.textContent = `${SOURCE_FILE}:L${stage.lines} · 真实仓库源码`;
      if (stageExplain) stageExplain.textContent = stage.explain;
      if (stageIndex) stageIndex.textContent = `${sourceStage + 1} / ${SOURCE_STAGES.length}`;
      if (stageCode) { stageCode.textContent = stage.code; highlight(stageCode); }
      if (stagePrev) stagePrev.disabled = sourceStage === 0;
      if (stageNext) stageNext.disabled = sourceStage === SOURCE_STAGES.length - 1;
      if (sourceBtn) {
        sourceBtn.setAttribute("data-file", SOURCE_FILE);
        sourceBtn.setAttribute("data-lines", stage.lines);
        sourceBtn.setAttribute("data-label", `map_field.cpp L${stage.lines}`);
      }
    }

    if (stagePrev) stagePrev.addEventListener("click", () => {
      if (sourceStage > 0) { sourceStage--; renderSource(); }
    });
    if (stageNext) stageNext.addEventListener("click", () => {
      if (sourceStage < SOURCE_STAGES.length - 1) { sourceStage++; renderSource(); }
    });
    if (copyBtn) copyBtn.addEventListener("click", () => {
      if (navigator.clipboard) navigator.clipboard.writeText(stageCode.textContent).catch(() => {});
      const old = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = old; }, 1200);
    });

    document.querySelectorAll(".hands-on-exercise").forEach((form) => {
      const input = form.querySelector(".exercise-input");
      const feedback = form.querySelector(".exercise-feedback");
      if (!input || !feedback) return;
      const kind = form.dataset.kind || "number";
      if (kind === "vector") form.dataset.answer = directOut.join(",");

      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const raw = input.value.trim().replace(/\s+/g, "");
        let ok = false;
        if (kind === "number") {
          const value = Number(raw);
          const expected = Number(form.dataset.answer);
          const tolerance = Number(form.dataset.tolerance || 0);
          ok = raw !== "" && Number.isFinite(value) && Math.abs(value - expected) <= tolerance;
        } else if (kind === "vector") {
          const expected = directOut;
          if (raw !== "") {
            const values = parseVector(raw);
            ok = values.length === expected.length && values.every((value, i) =>
              Number.isFinite(value) && value === expected[i]);
          }
        } else if (kind === "choice") {
          ok = raw.toLowerCase() === String(form.dataset.answer).replace(/\s+/g, "").toLowerCase();
        }
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续尝试下一题。")
          : (form.dataset.hint || "还不对：回到上面的单步数值演算，先写出公式再代入。");
      });

      input.addEventListener("input", () => {
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
      });
    });

    renderTrace();
    renderSource();
  }

  window.addEventListener("sloc:ready", init);
})();
