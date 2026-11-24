// Ch09 interactive Apple-styled bar/scatter visualization of E1–E10 & Intel Lab GT 3-way metrics
(function () {
  "use strict";
  function init() {
    const canvas = document.getElementById("ch09-bench-canvas");
    if (!canvas || !window.SlocApp) return;

    const DATASETS = {
      gt3: {
        title: "Intel Lab 450 组精校真值三方对比 · 平移 RPE RMSE (cm，越低越好)",
        unit: "cm",
        items: [
          { name: "scan_localizer (Composed)", val: 3.06, note: "Median 1.57 cm · Loop 2.95 cm · CPU 12.1%", color: "blue" },
          { name: "nav2_amcl (KLD-PF)", val: 12.57, note: "Median 3.64 cm · Loop 22.60 cm · CPU 4.1%", color: "indigo" },
          { name: "slam_toolbox (Localization)", val: 24.56, note: "Median 2.28 cm · Loop 43.99 cm · CPU 40.7%", color: "orange" },
        ],
      },
      e7: {
        title: "E7 单帧耗时对比 (720 束激光 x86 Release 中位耗时 ms)",
        unit: "ms",
        items: [
          { name: "field_gn (SDF 连续场 GN)", val: 0.054, note: "P90 0.075 ms · 精度 20mm", color: "green" },
          { name: "ndt (2D 体素高斯)", val: 0.132, note: "P90 0.348 ms · 精度 35mm", color: "teal" },
          { name: "icp_p2p (点对点 ICP)", val: 0.297, note: "P90 0.448 ms · 捕获仅 0.15m", color: "indigo" },
          { name: "correlative (滑窗粗搜)", val: 6.362, note: "P90 8.008 ms · 捕获 0.40m", color: "orange" },
          { name: "composed (粗+精融合)", val: 6.393, note: "subsample=2 仅需 3.31ms", color: "blue" },
        ],
      },
      e2: {
        title: "E2 大初始误差捕获成功率 (% at 0.80m / ±18°)",
        unit: "%",
        items: [
          { name: "composed_wide (±1.6m)", val: 100.0, note: "全范围 100% 捕获 (突发恢复)", color: "green" },
          { name: "composed (±0.15m)", val: 86.0, note: "0.40m 内 100%，日常跟踪首选", color: "blue" },
          { name: "field_gn 单独", val: 48.0, note: "易陷局部极小", color: "indigo" },
          { name: "icp_p2p 单独", val: 10.0, note: "0.40m 即掉至 28%", color: "orange" },
          { name: "ndt 单独", val: 0.0, note: "仅靠体素半径局部收敛", color: "red" },
        ],
      },
    };

    let activeKey = "gt3";

    function render() {
      const W = 740, H = 280;
      const ctx = window.SlocApp.setupCanvas(canvas, W, H);
      const pal = window.SlocApp.palette();
      ctx.fillStyle = pal.bg;
      ctx.fillRect(0, 0, W, H);

      const ds = DATASETS[activeKey];
      ctx.fillStyle = pal.text;
      ctx.font = '600 14px -apple-system, sans-serif';
      ctx.fillText(ds.title, 20, 28);

      const maxVal = Math.max(...ds.items.map((x) => x.val), 1e-6);
      const barLeft = 296, barMaxW = 340;
      const rowH = Math.min(48, (H - 64) / ds.items.length);

      ds.items.forEach((it, idx) => {
        const y = 52 + idx * rowH;
        ctx.fillStyle = pal.text;
        ctx.font = '600 13px -apple-system, sans-serif';
        ctx.fillText(it.name, 20, y + 18);

        ctx.fillStyle = pal.muted;
        ctx.font = '11.5px -apple-system, sans-serif';
        ctx.fillText(it.note, 20, y + 34);

        // Background track
        ctx.fillStyle = pal.bgAlt;
        ctx.beginPath();
        ctx.roundRect(barLeft, y + 8, barMaxW, 18, 9);
        ctx.fill();

        // Value bar
        const bw = Math.max(8, (it.val / maxVal) * barMaxW);
        ctx.fillStyle = pal[it.color] || pal.blue;
        ctx.beginPath();
        ctx.roundRect(barLeft, y + 8, bw, 18, 9);
        ctx.fill();

        ctx.fillStyle = pal.text;
        ctx.font = '600 12.5px "SF Mono", monospace';
        ctx.fillText(`${it.val} ${ds.unit}`, barLeft + bw + 10, y + 21);
      });
    }

    document.querySelectorAll("#ch09-metric-tabs button").forEach((b) => {
      b.addEventListener("click", () => {
        document.querySelectorAll("#ch09-metric-tabs button").forEach((x) => x.classList.remove("active"));
        b.classList.add("active");
        activeKey = b.dataset.metric;
        render();
      });
    });

    window.addEventListener("sloc:theme", render);
    render();
  }
  window.addEventListener("sloc:ready", init);
})();
