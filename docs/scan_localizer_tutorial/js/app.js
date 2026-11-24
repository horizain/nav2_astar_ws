// Shared UI controller: Apple frosted navbar, sticky TOC, KaTeX, Highlight.js,
// live code drawer with line highlighting, Cmd+K global search, and theme toggle.
(function () {
  "use strict";

  const rootDepth = window.TUTORIAL_ROOT || "."; // "." for top-level, ".." for chapters/
  const pageId = window.PAGE_ID || "home";
  const REPO_ABS = "/home/horizain/ros2_jazzy/nav2_astar_ws/src/scan_localizer/";
  const VSCODE_MODE_KEY = "sloc_vscode_mode";
  let wslDistro = "Ubuntu-24.04";
  let hasCodeCli = false;

  async function detectEnv() {
    if (window.location.protocol.startsWith("http")) {
      try {
        const resp = await fetch("/api/env");
        if (resp.ok) {
          const info = await resp.json();
          if (info.wsl_distro) wslDistro = info.wsl_distro;
          hasCodeCli = Boolean(info.has_code_cli);
          syncDrawerVscodeModeUI();
        }
      } catch (_) {}
    }
  }

  function getVscodeMode() {
    return localStorage.getItem(VSCODE_MODE_KEY) || "wsl-remote";
  }

  function buildVscodeUrl(fileRel, startLine, modeOverride) {
    const line = Math.max(1, parseInt(startLine, 10) || 1);
    const cleanRel = String(fileRel || "").replace(/^\/+/, "");
    const absLinux = REPO_ABS + cleanRel;
    const mode = modeOverride || getVscodeMode();
    if (mode === "wsl-unc") {
      return `vscode://file///wsl.localhost/${wslDistro}${absLinux}:${line}:1`;
    }
    return `vscode://vscode-remote/wsl+${wslDistro}${absLinux}:${line}:1`;
  }

  async function openInVscode(fileRel, startLine) {
    const line = Math.max(1, parseInt(startLine, 10) || 1);
    const cleanRel = String(fileRel || "").replace(/^\/+/, "");
    const mode = getVscodeMode();
    if (mode === "wsl-cli" && window.location.protocol.startsWith("http")) {
      try {
        const resp = await fetch("/api/open-vscode", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ file: cleanRel, line }),
        });
        if (resp.ok) return true;
      } catch (_) {}
    }
    window.location.href = buildVscodeUrl(cleanRel, line, mode);
    return true;
  }

  const THEME_KEY = "sloc_tutorial_theme";
  function applyTheme(theme) {
    if (theme === "dark") {
      document.documentElement.setAttribute("data-theme", "dark");
    } else {
      document.documentElement.removeAttribute("data-theme");
    }
    const btn = document.getElementById("theme-toggle-btn");
    if (btn) btn.textContent = theme === "dark" ? "浅色外观" : "深色外观";
    window.dispatchEvent(new CustomEvent("sloc:theme", { detail: { theme } }));
  }

  function initTheme() {
    const saved = localStorage.getItem(THEME_KEY) || "light";
    applyTheme(saved);
  }

  function rUrl(rel) {
    return rootDepth === "." ? rel : rootDepth + "/" + rel;
  }

  function buildNav() {
    const host = document.getElementById("site-nav");
    if (!host) return;
    const chapters = (window.SLOC_MANIFEST && window.SLOC_MANIFEST.CHAPTERS) || [];
    const ddHtml = chapters
      .map((c) => {
        const cur = pageId === c.id ? " current" : "";
        return `<a class="dd-item${cur}" href="${rUrl(c.file)}">
          <span class="dd-num">${c.num}</span>
          <div><div class="dd-title">${c.title}</div><div class="dd-sub">${c.sub}</div></div>
        </a>`;
      })
      .join("");

    const nav = document.createElement("header");
    nav.className = "apple-nav";
    nav.innerHTML = `
      <div class="apple-nav-inner">
        <a class="nav-brand" href="${rUrl("index.html")}">
          <span class="nav-dot"></span>
          <span>scan_localizer 工程课</span>
        </a>
        <nav class="nav-links">
          <a class="nav-link${pageId === "home" ? " active" : ""}" href="${rUrl("index.html")}">概览</a>
          <div class="chapter-menu-wrap">
            <button type="button" class="nav-link${pageId.startsWith("ch") ? " active" : ""}" id="ch-menu-btn">课程目录 ▾</button>
            <div class="chapter-dropdown" id="ch-dropdown">${ddHtml}</div>
          </div>
          <a class="nav-link${pageId === "walkthrough" ? " active" : ""}" href="${rUrl("walkthrough.html")}">思路导读</a>
          <a class="nav-link${pageId === "code-index" ? " active" : ""}" href="${rUrl("code-index.html")}">代码索引</a>
          <a class="nav-link${pageId === "ch10" ? " active" : ""}" href="${rUrl("chapters/ch10.html")}">语法手册</a>
        </nav>
        <div class="nav-actions">
          <button type="button" class="icon-btn" id="open-search-btn" title="全局符号搜索">
            <span>搜索源码</span><span class="kbd">⌘K</span>
          </button>
          <button type="button" class="icon-btn" id="theme-toggle-btn">深色外观</button>
        </div>
      </div>
    `;
    host.replaceWith(nav);

    const chBtn = document.getElementById("ch-menu-btn");
    const chDd = document.getElementById("ch-dropdown");
    chBtn.addEventListener("click", (e) => {
      e.stopPropagation();
      chDd.classList.toggle("open");
    });
    document.addEventListener("click", () => chDd.classList.remove("open"));
    chDd.addEventListener("click", (e) => e.stopPropagation());

    document.getElementById("theme-toggle-btn").addEventListener("click", () => {
      const isDark = document.documentElement.getAttribute("data-theme") === "dark";
      const next = isDark ? "light" : "dark";
      localStorage.setItem(THEME_KEY, next);
      applyTheme(next);
    });
    document.getElementById("open-search-btn").addEventListener("click", openSearch);
  }

  function buildToc() {
    const tocList = document.getElementById("page-toc");
    const article = document.querySelector(".article-col");
    if (!tocList || !article) return;
    const headings = Array.from(article.querySelectorAll("h2[id]"));
    tocList.innerHTML = headings
      .map((h) => {
        const noEl = h.querySelector(".sec-no");
        const no = noEl ? noEl.textContent.trim() : "";
        const rest = noEl ? h.textContent.replace(noEl.textContent, "").trim() : h.textContent.trim();
        const label = no ? `${no} · ${rest}` : rest;
        return `<li><a href="#${h.id}">${escapeHtml(label)}</a></li>`;
      })
      .join("");
    const links = Array.from(tocList.querySelectorAll("a"));
    if (!links.length) return;

    const updateActive = () => {
      const top = window.scrollY + 110;
      let activeIdx = 0;
      for (let i = 0; i < headings.length; i++) {
        if (headings[i].offsetTop <= top) activeIdx = i;
      }
      links.forEach((a, i) => a.classList.toggle("active", i === activeIdx));
    };
    window.addEventListener("scroll", updateActive, { passive: true });
    updateActive();
  }

  function initCodeBlocks() {
    if (window.hljs) {
      document.querySelectorAll("pre code").forEach((el) => {
        try {
          window.hljs.highlightElement(el);
        } catch (_) {}
      });
    }
    document.querySelectorAll(".code-copy-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        const card = btn.closest(".code-card");
        const code = card ? card.querySelector("pre code") : null;
        if (!code) return;
        navigator.clipboard?.writeText(code.textContent).catch(() => {});
        const orig = btn.textContent;
        btn.textContent = "已复制";
        setTimeout(() => { btn.textContent = orig; }, 1200);
      });
    });
  }

  function initKatex() {
    if (typeof window.renderMathInElement === "function") {
      window.renderMathInElement(document.body, {
        delimiters: [
          { left: "$$", right: "$$", display: true },
          { left: "$", right: "$", display: false },
          { left: "\\(", right: "\\)", display: false },
          { left: "\\[", right: "\\]", display: true },
        ],
        ignoredTags: ["script", "noscript", "style", "textarea", "pre", "code"],
        throwOnError: false,
      });
    }
  }

  // ---------- Live Source Drawer ----------
  let drawerBackdrop = null;
  let drawerEl = null;
  let currentDrawerFile = "";
  let currentDrawerLine = 1;

  function syncDrawerVscodeModeUI() {
    const sel = document.getElementById("drawer-vscode-mode");
    const btn = document.getElementById("drawer-vscode-btn");
    if (sel) {
      sel.value = getVscodeMode();
      const cliOpt = sel.querySelector('option[value="wsl-cli"]');
      if (cliOpt) {
        cliOpt.disabled = !(hasCodeCli && window.location.protocol.startsWith("http"));
        cliOpt.textContent = cliOpt.disabled ? "CLI 直开 (需 serve.sh)" : "CLI 直开 (免弹窗)";
      }
    }
    if (btn && currentDrawerFile) {
      btn.href = buildVscodeUrl(currentDrawerFile, currentDrawerLine);
    }
  }

  function ensureDrawer() {
    if (drawerEl) return;
    drawerBackdrop = document.createElement("div");
    drawerBackdrop.className = "drawer-backdrop";
    drawerEl = document.createElement("aside");
    drawerEl.className = "code-drawer";
    drawerEl.id = "code-drawer";
    drawerEl.innerHTML = `
      <div class="drawer-top">
        <div>
          <div class="drawer-title" id="drawer-file-title"></div>
          <div class="drawer-sub" id="drawer-file-sub"></div>
        </div>
        <div class="code-actions">
          <select id="drawer-vscode-mode" class="code-select" title="选择 Windows 侧 VS Code 打开方式">
            <option value="wsl-remote">WSL Remote</option>
            <option value="wsl-unc">WSL UNC 路径</option>
            <option value="wsl-cli">CLI 直开 (免弹窗)</option>
          </select>
          <button type="button" class="code-btn" id="drawer-copy-btn">复制路径</button>
          <a class="code-btn" id="drawer-vscode-btn" href="#">VS Code 打开</a>
          <button type="button" class="code-btn" id="drawer-close-btn">关闭</button>
        </div>
      </div>
      <div class="drawer-body" id="drawer-body"></div>
    `;
    document.body.appendChild(drawerBackdrop);
    document.body.appendChild(drawerEl);

    const modeSel = document.getElementById("drawer-vscode-mode");
    modeSel.addEventListener("change", () => {
      localStorage.setItem(VSCODE_MODE_KEY, modeSel.value);
      syncDrawerVscodeModeUI();
    });
    const vscBtn = document.getElementById("drawer-vscode-btn");
    vscBtn.addEventListener("click", (e) => {
      if (getVscodeMode() === "wsl-cli" && currentDrawerFile) {
        e.preventDefault();
        openInVscode(currentDrawerFile, currentDrawerLine);
      }
    });
    syncDrawerVscodeModeUI();

    const close = () => {
      drawerBackdrop.classList.remove("open");
      drawerEl.classList.remove("open");
    };
    drawerBackdrop.addEventListener("click", close);
    document.getElementById("drawer-close-btn").addEventListener("click", close);
    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape") {
        close();
        closeSearch();
        closeLightbox();
      }
    });
  }

  function parseLineRange(str) {
    if (!str) return [1, 1];
    const parts = String(str).replace(/L/gi, "").split(/[-–:]/).map((x) => parseInt(x, 10));
    const s = Number.isFinite(parts[0]) ? parts[0] : 1;
    const e = Number.isFinite(parts[1]) ? Math.max(s, parts[1]) : s;
    return [s, e];
  }

  function escapeHtml(s) {
    return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  }

  async function openSourceDrawer(fileRel, linesRange, label) {
    ensureDrawer();
    const [startLine, endLine] = parseLineRange(linesRange);
    const absPath = REPO_ABS + fileRel;
    currentDrawerFile = fileRel;
    currentDrawerLine = startLine;
    document.getElementById("drawer-file-title").textContent = `src/scan_localizer/${fileRel} : L${startLine}-${endLine}`;
    document.getElementById("drawer-file-sub").textContent = label || "实时源码阅读抽屉";
    syncDrawerVscodeModeUI();
    const copyBtn = document.getElementById("drawer-copy-btn");
    copyBtn.onclick = () => {
      navigator.clipboard?.writeText(`src/scan_localizer/${fileRel}:${startLine}-${endLine}`).catch(() => {});
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = "复制路径"; }, 1200);
    };

    const body = document.getElementById("drawer-body");
    body.innerHTML = `<div class="drawer-msg">正在读取 src/scan_localizer/${escapeHtml(fileRel)} ...</div>`;
    drawerBackdrop.classList.add("open");
    drawerEl.classList.add("open");

    const fetchRel = (rootDepth === "." ? "../../src/scan_localizer/" : "../../../src/scan_localizer/") + fileRel;
    try {
      const resp = await fetch(fetchRel);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const text = await resp.text();
      const rawLines = text.split(/\r?\n/);
      const isPy = fileRel.endsWith(".py");
      const isYaml = fileRel.endsWith(".yaml") || fileRel.endsWith(".yml");
      const lang = isPy ? "python" : isYaml ? "yaml" : "cpp";

      // Highlight full snippet safely per-line fallback if large
      let htmlLines;
      if (window.hljs && text.length < 180000) {
        try {
          const highlighted = window.hljs.highlight(text, { language: lang }).value;
          htmlLines = highlighted.split(/\r?\n/);
        } catch (_) {
          htmlLines = rawLines.map(escapeHtml);
        }
      } else {
        htmlLines = rawLines.map(escapeHtml);
      }

      const chunks = [];
      for (let i = 0; i < rawLines.length; i++) {
        const ln = i + 1;
        const hl = ln >= startLine && ln <= endLine ? " hl" : "";
        const content = htmlLines[i] !== undefined ? htmlLines[i] : escapeHtml(rawLines[i]);
        chunks.push(`<div class="src-line${hl}" data-ln="${ln}"><span class="src-ln">${ln}</span><span class="src-tx">${content || " "}</span></div>`);
      }
      body.innerHTML = chunks.join("");
      const focusEl = body.querySelector(`.src-line[data-ln="${startLine}"]`);
      if (focusEl) {
        focusEl.scrollIntoView({ block: "center" });
      }
    } catch (err) {
      body.innerHTML = `
        <div class="drawer-msg">
          <p>当前浏览器处于离线 <code>file://</code> 安全限制，无法直接读取本地 C++ 文件（${escapeHtml(err.message)}）。</p>
          <p>在仓库根目录运行 <code>./docs/scan_localizer_tutorial/serve.sh</code> 并访问本地 HTTP 地址，即可实时加载高亮源码；或直接点击右上角 <b>VS Code 打开</b> 按钮跳转至第 ${startLine} 行。</p>
          <p>目标文件：<code>${escapeHtml(absPath)}:${startLine}-${endLine}</code></p>
        </div>`;
    }
  }

  // ---------- Global Cmd+K Symbol Search ----------
  let searchBackdrop = null;
  function ensureSearch() {
    if (searchBackdrop) return;
    searchBackdrop = document.createElement("div");
    searchBackdrop.className = "search-backdrop";
    searchBackdrop.innerHTML = `
      <div class="search-box" role="dialog" aria-label="源码符号检索">
        <div class="search-input-row">
          <input type="text" id="global-search-input" placeholder="搜索类名、函数、文件名或章节（如 FieldGnMatcher, 4D EKF, swmax）..." />
          <span class="kbd">ESC</span>
        </div>
        <div class="search-results" id="global-search-results"></div>
      </div>
    `;
    document.body.appendChild(searchBackdrop);
    searchBackdrop.addEventListener("click", (e) => {
      if (e.target === searchBackdrop) closeSearch();
    });
    const input = document.getElementById("global-search-input");
    input.addEventListener("input", () => renderSearchResults(input.value));
  }

  function renderSearchResults(q) {
    const resEl = document.getElementById("global-search-results");
    if (!resEl || !window.SLOC_MANIFEST) return;
    const term = q.trim().toLowerCase();
    const hits = [];
    for (const f of window.SLOC_MANIFEST.FILES) {
      if (!term || f.path.toLowerCase().includes(term) || f.role.toLowerCase().includes(term)) {
        hits.push({ kind: "file", title: f.path, desc: f.role, file: f.path, range: `1-${Math.min(45, f.lines)}`, chapter: f.chapter });
      }
      for (const s of f.symbols) {
        if (!term || s.name.toLowerCase().includes(term) || s.desc.toLowerCase().includes(term)) {
          hits.push({ kind: "sym", title: s.name, desc: `${f.path}:${s.range} — ${s.desc}`, file: f.path, range: s.range, chapter: f.chapter });
        }
      }
    }
    const top = hits.slice(0, 18);
    if (!top.length) {
      resEl.innerHTML = `<div class="drawer-msg" style="color:var(--ink-3)">未找到匹配的源码符号。</div>`;
      return;
    }
    resEl.innerHTML = top
      .map((h, idx) => `
        <div class="search-item" data-idx="${idx}">
          <div>
            <div class="search-sym">${escapeHtml(h.title)}</div>
            <div class="search-desc">${escapeHtml(h.desc)}</div>
          </div>
          <span class="pill">${h.chapter.toUpperCase()}</span>
        </div>
      `)
      .join("");
    Array.from(resEl.querySelectorAll(".search-item")).forEach((el, idx) => {
      el.addEventListener("click", () => {
        const item = top[idx];
        closeSearch();
        openSourceDrawer(item.file, item.range, item.title);
      });
    });
  }

  function openSearch() {
    ensureSearch();
    searchBackdrop.classList.add("open");
    const inp = document.getElementById("global-search-input");
    inp.value = "";
    renderSearchResults("");
    setTimeout(() => inp.focus(), 20);
  }

  function closeSearch() {
    if (searchBackdrop) searchBackdrop.classList.remove("open");
  }

  // ---------- Image Lightbox ----------
  let lbEl = null;
  function initLightbox() {
    document.querySelectorAll(".fig-card img").forEach((img) => {
      img.addEventListener("click", () => {
        if (!lbEl) {
          lbEl = document.createElement("div");
          lbEl.style.cssText = "position:fixed;inset:0;background:rgba(0,0,0,0.85);z-index:300;display:none;align-items:center;justify-content:center;padding:24px;cursor:zoom-out;";
          lbEl.innerHTML = `<img style="max-width:96vw;max-height:92vh;border-radius:12px;background:#fff;" />`;
          lbEl.addEventListener("click", closeLightbox);
          document.body.appendChild(lbEl);
        }
        lbEl.querySelector("img").src = img.src;
        lbEl.style.display = "flex";
      });
    });
  }
  function closeLightbox() {
    if (lbEl) lbEl.style.display = "none";
  }

  // Global source jump delegation
  document.addEventListener("click", (e) => {
    const btn = e.target.closest("[data-file][data-lines]");
    if (btn) {
      e.preventDefault();
      openSourceDrawer(btn.getAttribute("data-file"), btn.getAttribute("data-lines"), btn.getAttribute("data-label") || btn.textContent.trim());
    }
  });

  document.addEventListener("keydown", (e) => {
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === "k") {
      e.preventDefault();
      openSearch();
    }
  });

  document.addEventListener("DOMContentLoaded", () => {
    initTheme();
    buildNav();
    buildToc();
    initCodeBlocks();
    initKatex();
    initLightbox();
    detectEnv();
    window.dispatchEvent(new CustomEvent("sloc:ready"));
  });

  window.SlocApp = {
    openSourceDrawer,
    openSearch,
    buildVscodeUrl,
    openInVscode,
    rUrl,
    isDark: () => document.documentElement.getAttribute("data-theme") === "dark",
    palette: () => {
      const dark = document.documentElement.getAttribute("data-theme") === "dark";
      return {
        bg: dark ? "#111113" : "#ffffff",
        bgAlt: dark ? "#1d1d1f" : "#f5f5f7",
        grid: dark ? "rgba(255,255,255,0.06)" : "rgba(0,0,0,0.05)",
        axis: dark ? "rgba(255,255,255,0.28)" : "rgba(0,0,0,0.22)",
        wall: dark ? "#e5e5ea" : "#1d1d1f",
        text: dark ? "#f5f5f7" : "#1d1d1f",
        muted: dark ? "#98989d" : "#6e6e73",
        blue: dark ? "#2997ff" : "#0071e3",
        green: dark ? "#30d158" : "#248a3d",
        orange: dark ? "#ff9f0a" : "#e66a00",
        red: dark ? "#ff453a" : "#d70015",
        indigo: dark ? "#7d7aff" : "#5856d6",
        teal: dark ? "#64d2ff" : "#0088b9",
      };
    },
    setupCanvas: (canvas, logicalW, logicalH) => {
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.round(logicalW * dpr);
      canvas.height = Math.round(logicalH * dpr);
      const ctx = canvas.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return ctx;
    },
  };
})();
