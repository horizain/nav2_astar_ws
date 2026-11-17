/* ============================================================
   main.js —— 全站共享逻辑
   · 统一注入顶栏/页脚（各页面只需一个占位 <div>）
   · 章节注册表：目录下拉、翻页、当前章高亮
   · 阅读进度条 / 自动 TOC / 代码复制 / 滚动浮现
   · KaTeX 自动渲染 + highlight.js 高亮
   ============================================================ */

const CHAPTERS = [
  { no: 1,  file: "ch01.html", title: "课程总览与环境准备",   brief: "项目拆解 · 技术路线 · 10 分钟搭好环境", time: 20,  part: "第一部分 · 准备", partNo: 0 },
  { no: 2,  file: "ch02.html", title: "数学最小集",           brief: "只学用得上的：概率 · log-odds · 最小二乘 · SE(2)", time: 35,  part: "第一部分 · 准备", partNo: 0 },
  { no: 3,  file: "ch03.html", title: "传感器与仿真器",       brief: "激光雷达模型 · 运动学 · 写出第一个模拟器", time: 45,  part: "第二部分 · mini-SLAM", partNo: 1 },
  { no: 4,  file: "ch04.html", title: "里程计与航迹推算",     brief: "亲手制造漂移惨案，理解 SLAM 为什么存在", time: 30,  part: "第二部分 · mini-SLAM", partNo: 1 },
  { no: 5,  file: "ch05.html", title: "扫描匹配：ICP",        brief: "SVD 闭式解 · 迭代最近点 · 外点修剪", time: 50,  part: "第二部分 · mini-SLAM", partNo: 1 },
  { no: 6,  file: "ch06.html", title: "占据栅格地图",         brief: "log-odds 贝叶斯更新 · 光线追踪", time: 40,  part: "第二部分 · mini-SLAM", partNo: 1 },
  { no: 7,  file: "ch07.html", title: "位姿图优化与回环",     brief: "把 SLAM 变成图问题 · Gauss-Newton 拉直漂移", time: 55,  part: "第二部分 · mini-SLAM", partNo: 1 },
  { no: 8,  file: "ch08.html", title: "组装完整 mini-SLAM",   brief: "关键帧 · 回环检测 · ATE 评估 · 全流程实跑", time: 60,  part: "第二部分 · mini-SLAM", partNo: 1 },
  { no: 9,  file: "ch09.html", title: "MPC 控制器",           brief: "滚动时域 · 误差模型 · QP 约束求解（OSQP）", time: 60,  part: "第三部分 · 导航与控制", partNo: 2 },
  { no: 10, file: "ch10.html", title: "导航全栈整合",         brief: "A* 全局规划 + 路径平滑 + MPC 跟踪", time: 55,  part: "第三部分 · 导航与控制", partNo: 2 },
  { no: 11, file: "ch11.html", title: "进阶路线与真实机器人", brief: "ROS 2 · Cartographer · 硬件清单 · 调参秘籍", time: 30,  part: "第三部分 · 导航与控制", partNo: 2 },
];

const ROOT = (window.SITE_ROOT || "..");   // 章节页位于 chapters/ 下

/* ---------- 顶栏 ---------- */
function buildHeader() {
  const path = location.pathname;
  const isHome = document.getElementById("home-page") !== null;
  const here = CHAPTERS.find(c => path.includes("/" + c.file));
  const logoSvg = `<svg class="logo" viewBox="0 0 40 40" fill="none">
    <circle cx="20" cy="20" r="18" stroke="#2563eb" stroke-width="2.6"/>
    <circle cx="20" cy="20" r="4.5" fill="#f59e0b"/>
    <g stroke="#60a5fa" stroke-width="2" stroke-linecap="round">
      <line x1="20" y1="20" x2="34" y2="12"/><line x1="20" y1="20" x2="30" y2="31"/>
      <line x1="20" y1="20" x2="7" y2="26"/><line x1="20" y1="20" x2="8" y2="9"/>
    </g></svg>`;
  const chapMenu = CHAPTERS.map(c =>
    `<a href="${ROOT}/chapters/${c.file}"><span class="no">${String(c.no).padStart(2, "0")}</span>
     <span class="t">${c.title}</span><span class="m">${c.time} 分钟</span></a>`).join("");
  const el = document.getElementById("site-header");
  if (!el) return;
  el.innerHTML = `
  <div id="progress"></div>
  <nav class="topnav"><div class="topnav-inner">
    <a class="brand" href="${ROOT}/index.html">${logoSvg}
      <span>SLAM·MPC 导航课堂</span><small>MINI-SLAM ACADEMY</small></a>
    <div class="nav-links">
      <a href="${ROOT}/index.html" class="${isHome ? "active" : ""}">首页</a>
      <div class="nav-drop" id="nav-drop">
        <button id="nav-drop-btn">课程目录 <svg width="12" height="12" viewBox="0 0 12 12" fill="none">
          <path d="M2.5 4.5L6 8l3.5-3.5" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/></svg></button>
        <div class="nav-drop-menu">${chapMenu}</div>
      </div>
      <a href="${ROOT}/index.html#sim-nav-section">在线仿真</a>
      <a href="${ROOT}/code/mini_slam/README.md">源码仓库</a>
      <a class="nav-cta" href="${ROOT}/chapters/ch01.html">开始学习 →</a>
    </div>
  </div></nav>`;
  const drop = document.getElementById("nav-drop");
  document.getElementById("nav-drop-btn").addEventListener("click", e => {
    e.stopPropagation(); drop.classList.toggle("open");
  });
  document.addEventListener("click", () => drop.classList.remove("open"));
  if (here) {
    // 当前章节在目录里标蓝
    const links = drop.querySelectorAll(".nav-drop-menu a");
    links[here.no - 1].style.background = "var(--accent-soft)";
  }
}

/* ---------- 页脚 ---------- */
function buildFooter() {
  const el = document.getElementById("site-footer");
  if (!el) return;
  el.innerHTML = `
  <footer class="site"><div class="inner">
    <div>
      <b>SLAM·MPC 导航课堂</b>
      <div style="margin-top:6px">从一个会漂移的轮式机器人，到一套完整的激光 SLAM + MPC 导航栈。</div>
    </div>
    <div style="display:flex; gap:44px; flex-wrap:wrap">
      <div><b style="font-size:14px">课程</b><div style="margin-top:8px; display:flex; flex-direction:column; gap:6px">
        <a href="${ROOT}/chapters/ch01.html">课程总览</a>
        <a href="${ROOT}/chapters/ch08.html">mini-SLAM 实战</a>
        <a href="${ROOT}/chapters/ch09.html">MPC 控制器</a>
      </div></div>
      <div><b style="font-size:14px">代码</b><div style="margin-top:8px; display:flex; flex-direction:column; gap:6px">
        <a href="${ROOT}/code/mini_slam/README.md">mini_slam 仓库说明</a>
        <a href="${ROOT}/chapters/ch11.html">进阶路线</a>
      </div></div>
    </div>
  </div>
  <div class="fine" style="max-width:1180px; margin:26px auto 0; padding:0 24px">
    所有结果图均由本课程代码真实运行生成 · 交互仿真基于原生 Canvas，无需任何构建工具 · Powered by curiosity
  </div></footer>`;
}

/* ---------- 阅读进度条 + 导航阴影 ---------- */
function initScrollFx() {
  const bar = document.getElementById("progress");
  const nav = document.querySelector(".topnav");
  if (!bar) return;
  const onScroll = () => {
    const h = document.documentElement;
    const max = h.scrollHeight - h.clientHeight;
    bar.style.width = (max > 0 ? (h.scrollTop / max) * 100 : 0) + "%";
    if (nav) nav.classList.toggle("scrolled", h.scrollTop > 8);
  };
  document.addEventListener("scroll", onScroll, { passive: true });
  onScroll();
}

/* ---------- 自动 TOC ---------- */
function buildTOC() {
  const holder = document.getElementById("toc");
  if (!holder) return;
  const hs = [...document.querySelectorAll(".content h2[id]")];
  if (!hs.length) { holder.style.display = "none"; return; }
  holder.innerHTML = `<div class="toc-title">本章目录</div>` +
    hs.map(h => `<a href="#${h.id}" data-toc="${h.id}">${h.textContent}</a>`).join("");
  const links = [...holder.querySelectorAll("a")];
  const obs = new IntersectionObserver(entries => {
    entries.forEach(en => {
      if (en.isIntersecting) {
        links.forEach(l => l.classList.toggle("active",
          l.dataset.toc === en.target.id));
      }
    });
  }, { rootMargin: "-20% 0px -70% 0px" });
  hs.forEach(h => obs.observe(h));
}

/* ---------- 代码复制 ---------- */
function initCopy() {
  document.querySelectorAll(".codeblock").forEach(cb => {
    const btn = cb.querySelector(".copy");
    const pre = cb.querySelector("pre");
    if (!btn || !pre) return;
    btn.addEventListener("click", () => {
      navigator.clipboard.writeText(pre.innerText).then(() => {
        btn.textContent = "已复制 ✓";
        setTimeout(() => (btn.textContent = "复制"), 1600);
      });
    });
  });
}

/* ---------- 滚动浮现 ---------- */
function initReveal() {
  const els = document.querySelectorAll(".reveal");
  if (!("IntersectionObserver" in window) || !els.length) {
    els.forEach(e => e.classList.add("in")); return;
  }
  const obs = new IntersectionObserver(entries => {
    entries.forEach(en => {
      if (en.isIntersecting) { en.target.classList.add("in"); obs.unobserve(en.target); }
    });
  }, { threshold: 0.12 });
  els.forEach(e => obs.observe(e));
}

/* ---------- 翻页 ---------- */
function buildPager() {
  const holder = document.getElementById("pager");
  if (!holder) return;
  const here = CHAPTERS.find(c => location.pathname.includes("/" + c.file));
  if (!here) return;
  const prev = CHAPTERS[here.no - 2], next = CHAPTERS[here.no];
  let html = "";
  html += prev ? `<a class="prev" href="${prev.file}">
      <div class="dir">← 上一章</div><div class="pt">第${prev.no}章 · ${prev.title}</div></a>`
    : `<a class="prev" href="../index.html"><div class="dir">← 返回</div>
       <div class="pt">课程首页</div></a>`;
  html += next ? `<a class="next" href="${next.file}">
      <div class="dir">下一章 →</div><div class="pt">第${next.no}章 · ${next.title}</div></a>`
    : `<a class="next" href="../index.html"><div class="dir">🎉 已到终点</div>
       <div class="pt">回到首页 · 开始你的项目</div></a>`;
  holder.innerHTML = html;
}

/* ---------- 公式与高亮 ---------- */
function initMathAndCode() {
  if (window.hljs) window.hljs.highlightAll();
  if (window.renderMathInElement) {
    window.renderMathInElement(document.body, {
      delimiters: [
        { left: "$$", right: "$$", display: true },
        { left: "\\[", right: "\\]", display: true },
        { left: "\\(", right: "\\)", display: false },
        { left: "$", right: "$", display: false },
      ],
      ignoredTags: ["script", "noscript", "style", "textarea", "pre", "code"],
      throwOnError: false,
    });
  }
}

document.addEventListener("DOMContentLoaded", () => {
  buildHeader();
  buildFooter();
  initScrollFx();
  buildTOC();
  initCopy();
  initReveal();
  buildPager();
  initMathAndCode();
  window.dispatchEvent(new Event("site:ready"));   // 仿真脚本监听此事件
});
