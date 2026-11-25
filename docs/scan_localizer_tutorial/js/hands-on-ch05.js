// Ch05 hands-on layer: minimal eigen-clamped covariance, live H/Hd numbers,
// real matchers.cpp source stepping and fill-in acceptance exercises.
(function () {
  "use strict";

  const SOURCE_FILE = "src/matchers.cpp";
  const K_MIN_EIG = 1e-4;
  const MIN_VAR_XY = 1e-4;
  const MIN_VAR_TH = 1.2e-5;
  const INFLATION = 2.0;

  const STAGES = [
    {
      title: "1 · 对称化 H，再交给自伴特征分解",
      lines: "70-78",
      explain: "协方差函数接收的是未阻尼 H。先消除浮点累加造成的微小非对称，再用 SelfAdjointEigenSolver 得到有序特征值与正交特征向量；Hd 不能在这里替换 H。",
      code: `// 基于特征分解的安全海森求逆协方差（退化方向夹紧至正下界保留大方差，含物理下限）
Cov3 cov_from_hessian(
  const Eigen::Matrix3d & H, double res_var,
  double min_var_xy = 1e-4,                       // (1cm)²
  double min_var_th = 1.2e-5,                     // (0.2°)²
  double inflation = 2.0)
{
  const Eigen::Matrix3d Hsym = 0.5 * (H + H.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Hsym);`,
    },
    {
      title: "2 · 分解失败时退回单位协方差",
      lines: "79-85",
      explain: "教学最小值也要有失败出口。若自伴分解没有成功，不继续传播 NaN 或半个结果，而是返回有限的单位协方差；它本身会通过有限性检查，是否拒绝仍由 score/inlier/MinEig/Mahal 等上层门控决定。",
      code: `  if (es.info() != Eigen::Success) {
    Cov3 fallback = Cov3::Zero();
    fallback(0, 0) = 1.0;
    fallback(1, 1) = 1.0;
    fallback(2, 2) = 1.0;
    return fallback;
  }`,
    },
    {
      title: "3 · 先夹紧特征值，再构造 H⁻¹",
      lines: "86-90",
      explain: "kMinEig=1e-4 是信息矩阵的特征值下界，不是协方差对角线的物理下限。cwiseMax 让零/负特征值先变成可逆的正数，再用 Q diag(1/λ) Qᵀ 重构逆矩阵。",
      code: `  constexpr double kMinEig = 1e-4;
  const Eigen::Vector3d eigs = es.eigenvalues().cwiseMax(kMinEig);
  const Eigen::Matrix3d H_inv =
    es.eigenvectors() * eigs.cwiseInverse().asDiagonal() *
    es.eigenvectors().transpose();`,
    },
    {
      title: "4 · 残差缩放、对称化与物理方差下限",
      lines: "91-97",
      explain: "scale 控制整体不确定度；随后再次对称化消除重构误差。最后的对角线下限是物理护栏：平移至少 1e-4 m²，转角至少 1.2e-5 rad²。它发生在特征值夹紧之后，不能反过来替代夹紧。",
      code: `  const double scale = std::max(res_var, min_var_xy) * inflation;
  Cov3 cov = scale * H_inv;
  cov = 0.5 * (cov + cov.transpose()).eval();
  cov(0, 0) = std::max(cov(0, 0), min_var_xy);
  cov(1, 1) = std::max(cov(1, 1), min_var_xy);
  cov(2, 2) = std::max(cov(2, 2), min_var_th);
  return cov;`,
    },
    {
      title: "5 · best_H 先保存；Hd 只服务步长求解",
      lines: "335-348",
      explain: "每轮先用当前位姿的 H 更新历史最优，随后才复制出 Hd 并加对角阻尼。dp 可以由 Hd 求出，但 best_H 必须保持未阻尼，才能如实描述最优位姿的观测信息。",
      code: `      // 应用步长前，先用当前位姿 p 与其自身残差 ravg 更新历史最优
    if (!has_valid_iter || ravg < best_ravg) {
      has_valid_iter = true;
      best_ravg = ravg;
      best_pose = p;
      best_H = H;
      best_var = std::max(ravg, 1e-6);
    }

      // LM 阻尼只用于解步；协方差用未阻尼 Hessian（退化方向如实膨胀）
    Eigen::Matrix3d Hd = H;
    Hd.diagonal().array() += 1e-6 * H.diagonal().array().max(1.0);
    const Eigen::Vector3d dp = Hd.ldlt().solve(-b);
    if (!dp.allFinite()) {break;}`,
    },
    {
      title: "6 · 最终输出必须传入 best_H",
      lines: "384-389",
      explain: "位姿、残差尺度与协方差必须来自同一个历史最优。最后一行调用是整章的数据流契约：cov_from_hessian(best_H, best_var)，绝不是 Hd。",
      code: `  res.pose = best_pose;
  res.covariance = cov_from_hessian(best_H, best_var);
  eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
    res.inlier_ratio);
  res.valid = true;
  return res;`,
    },
  ];

  function fmt(value, digits = 5) {
    if (value === null || value === undefined) return "—";
    if (typeof value === "number" && !Number.isFinite(value)) return String(value);
    if (typeof value === "number") {
      const av = Math.abs(value);
      if (av !== 0 && (av < 1e-3 || av >= 1e4)) return value.toExponential(2);
      return value.toFixed(digits).replace(/\.?0+$/, "");
    }
    return String(value);
  }

  function vector(values) {
    return Array.isArray(values) ? `[${values.map((v) => fmt(v)).join(", ")}]` : "—";
  }

  function matrix(values) {
    if (!Array.isArray(values) || values.length !== 9) return "—";
    return `[${[0, 1, 2].map((r) =>
      values.slice(r * 3, r * 3 + 3).map((v) => fmt(v)).join(", ")).join("; ")}]`;
  }

  function pose(value) {
    if (!value || typeof value !== "object") return "—";
    return `(${fmt(value.x)}, ${fmt(value.y)}, ${fmt(value.theta)})`;
  }

  function finiteMatrix(value) {
    return Array.isArray(value) && value.length === 9 && value.every(Number.isFinite);
  }

  function symmetrize3x3(H) {
    return [
      H[0], 0.5 * (H[1] + H[3]), 0.5 * (H[2] + H[6]),
      0.5 * (H[1] + H[3]), H[4], 0.5 * (H[5] + H[7]),
      0.5 * (H[2] + H[6]), 0.5 * (H[5] + H[7]), H[8],
    ];
  }

  function fallbackDetails(reason) {
    return {
      covariance: [1, 0, 0, 0, 1, 0, 0, 0, 1],
      Hsym: null,
      rawEigenvalues: [NaN, NaN, NaN],
      clampedEigenvalues: [NaN, NaN, NaN],
      inverseEigenvalues: [NaN, NaN, NaN],
      covarianceEigenvalues: [1, 1, 1],
      clampedCount: 0,
      scale: NaN,
      fallback: true,
      reason,
    };
  }

  // Minimal teaching implementation of src/matchers.cpp:L70-98.
  // The existing math core supplies only the symmetric eigensolver; all
  // symmetrization, eigen clamping, reconstruction and physical floors live here.
  function covarianceDetails(H, resVar, minVarXY = MIN_VAR_XY,
    minVarTH = MIN_VAR_TH, inflation = INFLATION) {
    if (!finiteMatrix(H) || !Number.isFinite(resVar) || !window.SlocSim) {
      return fallbackDetails("H 或 res_var 非有限，退回单位协方差");
    }
    const S = window.SlocSim;
    const Hsym = symmetrize3x3(H);
    const eig = S.eigenSym3x3(Hsym);
    if (!eig || !Array.isArray(eig.values) || eig.values.length !== 3 ||
      !eig.values.every(Number.isFinite) || !Array.isArray(eig.vectors)) {
      return fallbackDetails("对称特征分解失败，退回单位协方差");
    }

    const rawEigenvalues = eig.values.slice();
    const clampedEigenvalues = rawEigenvalues.map((v) => Math.max(v, K_MIN_EIG));
    const inverseEigenvalues = clampedEigenvalues.map((v) => 1 / v);
    const V = eig.vectors; // V[column][row]
    const Hinv = new Array(9).fill(0);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        let sum = 0;
        for (let k = 0; k < 3; k++) {
          sum += V[k][r] * inverseEigenvalues[k] * V[k][c];
        }
        Hinv[r * 3 + c] = sum;
      }
    }

    const scale = Math.max(resVar, minVarXY) * inflation;
    const covariance = Hinv.map((v) => scale * v);
    for (let i = 0; i < 3; i++) {
      for (let j = i + 1; j < 3; j++) {
        const a = covariance[i * 3 + j];
        const b = covariance[j * 3 + i];
        covariance[i * 3 + j] = covariance[j * 3 + i] = 0.5 * (a + b);
      }
    }
    covariance[0] = Math.max(covariance[0], minVarXY);
    covariance[4] = Math.max(covariance[4], minVarXY);
    covariance[8] = Math.max(covariance[8], minVarTH);

    const covEig = S.eigenSym3x3(covariance);
    return {
      covariance,
      Hsym,
      rawEigenvalues,
      clampedEigenvalues,
      inverseEigenvalues,
      covarianceEigenvalues: covEig ? covEig.values.slice() : [NaN, NaN, NaN],
      clampedCount: rawEigenvalues.filter((v) => v < K_MIN_EIG).length,
      scale,
      fallback: false,
      reason: "对称特征分解成功",
    };
  }

  function cov_from_hessian(H, resVar, minVarXY = MIN_VAR_XY,
    minVarTH = MIN_VAR_TH, inflation = INFLATION) {
    return covarianceDetails(H, resVar, minVarXY, minVarTH, inflation).covariance;
  }

  function makeDampedHessian(H) {
    if (!finiteMatrix(H)) return null;
    const Hd = H.slice();
    Hd[0] += 1e-6 * Math.max(H[0], 1.0);
    Hd[4] += 1e-6 * Math.max(H[4], 1.0);
    Hd[8] += 1e-6 * Math.max(H[8], 1.0);
    return Hd;
  }

  function maxAbsDelta(a, b) {
    if (!finiteMatrix(a) || !finiteMatrix(b)) return NaN;
    return Math.max(...a.map((v, i) => Math.abs(v - b[i])));
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

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function init() {
    if (!window.SlocSim || window.PAGE_ID !== "ch05") return;
    if (!document.getElementById("ch05-numeric-vars") ||
      !document.getElementById("ch05-stage-code")) return;

    const S = window.SlocSim;
    const scenes = {
      corridor: {
        label: "30m×5m JS 教学走廊",
        factory: "S.createCorridorMap(0.12)",
        grid: S.createCorridorMap(0.12),
        gt: { x: 15.0, y: 2.5, theta: 0.0 },
      },
      warehouse: {
        label: "仓库转角对照",
        factory: "S.createWarehouseMap(0.12)",
        grid: S.createWarehouseMap(0.12),
        gt: { x: 10.0, y: 8.5, theta: 0.25 },
      },
    };
    scenes.corridor.field = new S.MapField(scenes.corridor.grid, 0.22);
    scenes.warehouse.field = new S.MapField(scenes.warehouse.grid, 0.22);

    const stageTitle = document.getElementById("ch05-stage-title");
    const stageMeta = document.getElementById("ch05-stage-meta");
    const stageExplain = document.getElementById("ch05-stage-explain");
    const stageCode = document.getElementById("ch05-stage-code");
    const stageIndex = document.getElementById("ch05-stage-index");
    const stagePrev = document.getElementById("ch05-stage-prev");
    const stageNext = document.getElementById("ch05-stage-next");
    const sourceBtn = document.getElementById("ch05-stage-source");
    const copyBtn = document.getElementById("ch05-stage-copy");
    const liveIter = document.getElementById("ch05-live-iter");
    const stageVars = document.getElementById("ch05-stage-vars");
    const covToggle = document.getElementById("ch05-cov-toggle");
    const corridorBtn = document.getElementById("ch05-live-corridor");
    const warehouseBtn = document.getElementById("ch05-live-warehouse");
    const resetBtn = document.getElementById("ch05-live-reset");
    const ranges = {
      x: document.getElementById("ch05-live-x"),
      y: document.getElementById("ch05-live-y"),
      theta: document.getElementById("ch05-live-theta"),
    };

    let sceneKey = "corridor";
    let useHd = false;
    let stage = 0;
    let snapshot = null;
    let liveFrame = 0;

    function syncControlLabels() {
      if (ranges.x) setText("ch05-live-x-v", `${Number(ranges.x.value) >= 0 ? "+" : ""}${Number(ranges.x.value).toFixed(2)} m`);
      if (ranges.y) setText("ch05-live-y-v", `${Number(ranges.y.value) >= 0 ? "+" : ""}${Number(ranges.y.value).toFixed(2)} m`);
      if (ranges.theta) setText("ch05-live-theta-v", `${Number(ranges.theta.value) >= 0 ? "+" : ""}${Number(ranges.theta.value).toFixed(3)} rad`);
    }

    function syncSourceToggle() {
      if (covToggle) {
        covToggle.textContent = useHd ? "恢复：使用未阻尼 H" : "错误实验：改用 Hd";
        covToggle.setAttribute("aria-pressed", String(useHd));
      }
      setText("ch05-cov-input", useHd
        ? "当前协方差实参：Hd（错误实验）"
        : "当前协方差实参：H（正确）");
    }

    function syncSceneButtons() {
      if (corridorBtn) corridorBtn.setAttribute("aria-pressed", String(sceneKey === "corridor"));
      if (warehouseBtn) warehouseBtn.setAttribute("aria-pressed", String(sceneKey === "warehouse"));
    }

    function numericRows(current) {
      if (!current || !current.match || !current.match.valid || !current.H) {
        return [
          ["状态", "匹配失败；不伪造 H 或协方差"],
          ["场景", current ? current.sceneLabel : "—"],
          ["prior", current ? pose(current.prior) : "—"],
          ["下一步", "减小先验偏移或切回长走廊中段"],
        ];
      }
      return [
        ["数据来源", `${current.sceneFactory} → S.makeScan → S.fieldGnMatch`],
        ["prior → matched", `${pose(current.prior)} → ${pose(current.match.pose)}`],
        ["res.hessian = H", matrix(current.H)],
        ["Hd = H + 1e-6·max(diag(H),1)", matrix(current.Hd)],
        ["Δdiag(Hd−H)", vector([0, 4, 8].map((i) => current.Hd[i] - current.H[i]))],
        ["λ(H) raw", vector(current.correct.rawEigenvalues)],
        ["λ(H) clamped", vector(current.correct.clampedEigenvalues)],
        ["当前协方差实参", useHd ? "Hd（错误实验）" : "H（正确）"],
        ["Σ = cov_from_hessian(实参,res_var)", matrix(current.chosen.covariance)],
        ["λ(Σ)", vector(current.chosen.covarianceEigenvalues)],
        ["max|Σ(H)−res.covariance|", fmt(current.referenceDelta)],
        ["max|Σ(Hd)−Σ(H)|", fmt(current.dampedDelta)],
      ];
    }

    function stageRows(current) {
      if (!current || !current.match || !current.match.valid || !current.H) {
        return [
          ["实时状态", "尚未获得有效匹配"],
          ["提示", "调整 5.3 的先验偏移；矩阵为空时显示 —，不补造数值"],
        ];
      }
      const common = [
        ["场景", current.sceneLabel],
        ["prior", pose(current.prior)],
        ["matched pose", pose(current.match.pose)],
        ["rmse / res_var", `${fmt(current.match.rmse)} m / ${fmt(current.resVar)} m²`],
        ["协方差实参", useHd ? "Hd（错误实验）" : "H（正确）"],
      ];
      const chosen = current.chosen;
      if (stage === 0) return [
        ...common,
        ["输入矩阵", useHd ? "Hd" : "H"],
        ["Hsym", matrix(chosen.Hsym)],
        ["eig.status", chosen.fallback ? chosen.reason : "Eigen / Jacobi 成功"],
      ];
      if (stage === 1) return [
        ...common,
        ["fallback", chosen.fallback ? "是" : "否"],
        ["回退协方差", chosen.fallback ? matrix(chosen.covariance) : "本帧不需要"],
        ["有限性", finiteMatrix(chosen.covariance) ? "3×3 全有限" : "存在非有限值"],
      ];
      if (stage === 2) return [
        ...common,
        ["λ raw", vector(chosen.rawEigenvalues)],
        ["λ clamped", vector(chosen.clampedEigenvalues)],
        ["1/λ clamped", vector(chosen.inverseEigenvalues)],
        ["夹紧方向数", `${chosen.clampedCount} / 3`],
      ];
      if (stage === 3) return [
        ...common,
        ["scale", fmt(chosen.scale)],
        ["Σ", matrix(chosen.covariance)],
        ["λ(Σ)", vector(chosen.covarianceEigenvalues)],
        ["物理下限", "diag ≥ [1e-4, 1e-4, 1.2e-5]"],
      ];
      if (stage === 4) return [
        ...common,
        ["best_H（未阻尼）", matrix(current.H)],
        ["Hd（只用于 solve）", matrix(current.Hd)],
        ["max|Hd−H|", fmt(maxAbsDelta(current.Hd, current.H))],
        ["生产语义", "dp = solve(Hd,-b)；Σ = cov_from_hessian(best_H,best_var)"],
      ];
      return [
        ...common,
        ["cov_from_hessian(H)", matrix(current.correct.covariance)],
        ["当前显示 Σ", matrix(chosen.covariance)],
        ["res.covariance 参考", matrix(current.match.covariance)],
        ["max|Σ(H)−参考|", fmt(current.referenceDelta)],
        ["max|Σ(Hd)−Σ(H)|", fmt(current.dampedDelta)],
      ];
    }

    function renderStageVars() {
      fillVars(stageVars, stageRows(snapshot));
      setText("ch05-live-iter", snapshot && snapshot.match && snapshot.match.valid
        ? `实时匹配：${snapshot.sceneLabel} · rmse=${fmt(snapshot.match.rmse)} m · 协方差实参=${useHd ? "Hd（错误）" : "H（正确）"}`
        : "等待 5.3 实时面板的有效匹配结果。");
    }

    function renderStage() {
      const cur = STAGES[stage];
      stageTitle.textContent = cur.title;
      stageMeta.textContent = `${SOURCE_FILE}:L${cur.lines} · 真实仓库源码 · ${stage + 1}/${STAGES.length}`;
      stageIndex.textContent = `${stage + 1} / ${STAGES.length}`;
      stageExplain.textContent = cur.explain;
      stageCode.textContent = cur.code;
      stageCode.className = "language-cpp";
      delete stageCode.dataset.highlighted;
      if (window.hljs) {
        try { window.hljs.highlightElement(stageCode); } catch (_) {}
      }
      sourceBtn.setAttribute("data-file", SOURCE_FILE);
      sourceBtn.setAttribute("data-lines", cur.lines);
      sourceBtn.setAttribute("data-label", `matchers.cpp L${cur.lines}`);
      stagePrev.disabled = stage === 0;
      stageNext.disabled = stage === STAGES.length - 1;
      renderStageVars();
    }

    function computeLive() {
      const def = scenes[sceneKey];
      const prior = {
        x: def.gt.x + Number(ranges.x.value),
        y: def.gt.y + Number(ranges.y.value),
        theta: def.gt.theta + Number(ranges.theta.value),
      };
      const scan = S.makeScan(def.grid, def.gt, { beams: 140, maxRange: 10.0, noiseStd: 0.0 });
      const match = S.fieldGnMatch(def.field, prior, scan);
      const H = finiteMatrix(match.hessian) ? match.hessian : null;
      const Hd = makeDampedHessian(H);
      const resVar = match.valid && Number.isFinite(match.rmse)
        ? Math.max(match.rmse * match.rmse, 1e-6)
        : NaN;
      const correct = H
        ? covarianceDetails(H, resVar)
        : fallbackDetails("fieldGnMatch 未返回有限 Hessian");
      const damped = Hd
        ? covarianceDetails(Hd, resVar)
        : fallbackDetails("无法构造 Hd");
      const chosen = useHd ? damped : correct;
      const ratio = chosen.covariance[0] / Math.max(chosen.covariance[4], 1e-12);
      snapshot = {
        sceneKey,
        sceneLabel: def.label,
        sceneFactory: def.factory,
        prior,
        scanCount: scan.length,
        match,
        H,
        Hd,
        resVar,
        correct,
        damped,
        chosen,
        referenceDelta: H ? maxAbsDelta(correct.covariance, match.covariance) : NaN,
        dampedDelta: H && Hd ? maxAbsDelta(correct.covariance, damped.covariance) : NaN,
      };

      setText("ch05-eig-raw", vector(correct.rawEigenvalues));
      setText("ch05-eig-clamped", vector(correct.clampedEigenvalues));
      setText("ch05-cov-eigs", vector(chosen.covarianceEigenvalues));
      setText("ch05-live-ratio", `${fmt(ratio, 3)}×`);
      setText("ch05-clamp-count", `${correct.clampedCount} / 3`);
      setText("ch05-match-state", match.valid
        ? `${scan.length} 点 · rmse ${fmt(match.rmse)} m`
        : "匹配失败");
      if (!match.valid || !H) {
        setText("ch05-live-note", "本帧没有有效 H；面板不伪造矩阵。请减小先验偏移后重试。");
      } else {
        const clampNote = correct.clampedCount > 0
          ? `λmin(H)=${fmt(correct.rawEigenvalues[0])}，已把 ${correct.clampedCount} 个方向夹紧到 1e-4；Σ(H) 与数学核参考协方差最大差 ${fmt(snapshot.referenceDelta)}。`
          : `本帧 λmin(H)=${fmt(correct.rawEigenvalues[0])}，没有方向需要夹紧；Σ(H) 与数学核参考协方差最大差 ${fmt(snapshot.referenceDelta)}。`;
        setText("ch05-live-note", `${clampNote}${useHd
          ? ` 当前显示错误的 Σ(Hd)，与 Σ(H) 最大差 ${fmt(snapshot.dampedDelta)}；生产实现不会这样调用。`
          : " 当前显示正确的 Σ(H)。"}`);
      }
      fillVars(document.getElementById("ch05-numeric-vars"), numericRows(snapshot));
      renderStageVars();
    }

    function scheduleLive() {
      syncControlLabels();
      if (liveFrame) cancelAnimationFrame(liveFrame);
      liveFrame = requestAnimationFrame(() => {
        liveFrame = 0;
        computeLive();
      });
    }

    stagePrev.addEventListener("click", () => {
      if (stage > 0) { stage--; renderStage(); }
    });
    stageNext.addEventListener("click", () => {
      if (stage < STAGES.length - 1) { stage++; renderStage(); }
    });
    copyBtn.addEventListener("click", () => {
      if (navigator.clipboard) {
        navigator.clipboard.writeText(stageCode.textContent).catch(() => {});
      }
      const old = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = old; }, 1200);
    });

    corridorBtn.addEventListener("click", () => {
      sceneKey = "corridor";
      syncSceneButtons();
      scheduleLive();
    });
    warehouseBtn.addEventListener("click", () => {
      sceneKey = "warehouse";
      syncSceneButtons();
      scheduleLive();
    });
    resetBtn.addEventListener("click", () => {
      sceneKey = "corridor";
      useHd = false;
      ranges.x.value = "0.05";
      ranges.y.value = "0.04";
      ranges.theta.value = "0.01";
      syncSceneButtons();
      syncSourceToggle();
      syncControlLabels();
      computeLive();
    });
    covToggle.addEventListener("click", () => {
      useHd = !useHd;
      syncSourceToggle();
      computeLive();
    });
    Object.values(ranges).forEach((input) => {
      input.addEventListener("input", scheduleLive);
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
        if (kind === "choice") {
          ok = raw.toLowerCase() === String(form.dataset.answer || "").toLowerCase();
        } else {
          const value = Number(raw);
          const expected = Number(form.dataset.answer);
          const tolerance = Number(form.dataset.tolerance || 0);
          ok = raw !== "" && Number.isFinite(value) &&
            Math.abs(value - expected) <= tolerance;
        }
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续尝试下一题。")
          : (form.dataset.hint || "还不对：回到特征值夹紧与物理下限的两个阶段，再算一次。");
      });
      input.addEventListener("input", () => {
        feedback.className = "exercise-feedback";
        feedback.textContent = "";
      });
    });

    syncControlLabels();
    syncSourceToggle();
    syncSceneButtons();
    renderStage();
    computeLive();
  }

  window.addEventListener("sloc:ready", init);
})();
