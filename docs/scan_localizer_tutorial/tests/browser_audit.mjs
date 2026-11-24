import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

const CHROME_BIN = "/home/horizain/.cache/ms-playwright/chromium-1255/chrome-linux64/chrome";
const HTTP_PORT = 18091;
const CDP_PORT = 19223;
const SCREENSHOT_DIR = "/home/horizain/.gemini/antigravity-cli/brain/15f8b0b6-edf0-4935-a3dd-3bf525549445/scratch/tutorial_screenshots";

const PAGES = [
  { rel: "index.html", shot: "00_index.png" },
  { rel: "walkthrough.html", shot: "02_walkthrough.png", testWalkthrough: true },
  { rel: "code-index.html", shot: "01_code_index.png", testDrawer: true },
  { rel: "chapters/ch01.html" },
  { rel: "chapters/ch02.html", shot: "02_ch02.png" },
  { rel: "chapters/ch03.html" },
  { rel: "chapters/ch04.html", shot: "04_ch04.png", testDrawer: true },
  { rel: "chapters/ch05.html" },
  { rel: "chapters/ch06.html" },
  { rel: "chapters/ch07.html", shot: "07_ch07.png", testDrawer: true },
  { rel: "chapters/ch08.html", shot: "08_ch08.png" },
  { rel: "chapters/ch09.html", shot: "09_ch09.png" },
  { rel: "chapters/ch10.html" },
];

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function waitForHttp(url, timeoutMs = 8000) {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    try {
      const r = await fetch(url);
      if (r.ok) return r;
    } catch (_) {}
    await sleep(120);
  }
  throw new Error(`Timeout waiting for ${url}`);
}

class CDPClient {
  constructor(wsUrl) {
    this.ws = new WebSocket(wsUrl);
    this.id = 0;
    this.pending = new Map();
    this.events = [];
  }
  async connect() {
    await new Promise((resolve, reject) => {
      this.ws.addEventListener("open", resolve, { once: true });
      this.ws.addEventListener("error", reject, { once: true });
    });
    this.ws.addEventListener("message", (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.id && this.pending.has(msg.id)) {
        const { resolve, reject } = this.pending.get(msg.id);
        this.pending.delete(msg.id);
        if (msg.error) reject(new Error(JSON.stringify(msg.error)));
        else resolve(msg.result);
      } else if (msg.method) {
        this.events.push(msg);
      }
    });
  }
  send(method, params = {}) {
    const id = ++this.id;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(JSON.stringify({ id, method, params }));
    });
  }
  async evalJS(expression) {
    const res = await this.send("Runtime.evaluate", {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (res.exceptionDetails) {
      throw new Error(`Eval exception: ${JSON.stringify(res.exceptionDetails)}`);
    }
    return res.result ? res.result.value : undefined;
  }
  close() {
    this.ws.close();
  }
}

async function main() {
  fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });
  const profileDir = fs.mkdtempSync(path.join(os.tmpdir(), "sloc_cdp_"));

  const server = spawn("python3", ["docs/scan_localizer_tutorial/server.py", "--port", String(HTTP_PORT)], {
    stdio: "ignore",
  });
  const chrome = spawn(CHROME_BIN, [
    "--headless=new",
    "--no-sandbox",
    "--disable-gpu",
    "--disable-dev-shm-usage",
    `--remote-debugging-port=${CDP_PORT}`,
    `--user-data-dir=${profileDir}`,
    "--window-size=1440,960",
    "about:blank",
  ], { stdio: "ignore" });

  let cdp = null;
  try {
    await waitForHttp(`http://127.0.0.1:${HTTP_PORT}/docs/scan_localizer_tutorial/index.html`);
    const envResp = await waitForHttp(`http://127.0.0.1:${HTTP_PORT}/api/env`);
    const envJson = await envResp.json();
    assert.ok(envJson.wsl_distro && envJson.wsl_distro.length > 0, "Missing wsl_distro from /api/env");
    console.log(`ENV /api/env -> wsl_distro=${envJson.wsl_distro}, has_code_cli=${envJson.has_code_cli}`);
    const listResp = await waitForHttp(`http://127.0.0.1:${CDP_PORT}/json/list`);
    const targets = await listResp.json();
    const pageTarget = targets.find((t) => t.type === "page") || targets[0];
    assert.ok(pageTarget && pageTarget.webSocketDebuggerUrl, "No CDP page target found");

    cdp = new CDPClient(pageTarget.webSocketDebuggerUrl);
    await cdp.connect();
    await cdp.send("Page.enable");
    await cdp.send("Runtime.enable");
    await cdp.send("Network.enable");
    await cdp.send("Emulation.setDeviceMetricsOverride", { width: 1440, height: 960, deviceScaleFactor: 2, mobile: false });

    const failures = [];
    for (const pg of PAGES) {
      cdp.events.length = 0;
      const url = `http://127.0.0.1:${HTTP_PORT}/docs/scan_localizer_tutorial/${pg.rel}`;
      await cdp.send("Page.navigate", { url });
      // Wait for load & animations
      for (let i = 0; i < 40; i++) {
        const rs = await cdp.evalJS("document.readyState");
        if (rs === "complete") break;
        await sleep(100);
      }
      await sleep(750);

      // Inspect runtime console/page errors & HTTP >= 400 responses
      const consoleErrors = [];
      const httpErrors = [];
      for (const ev of cdp.events) {
        if (ev.method === "Runtime.exceptionThrown") {
          consoleErrors.push(JSON.stringify(ev.params.exceptionDetails));
        } else if (ev.method === "Runtime.consoleAPICalled" && ev.params.type === "error") {
          consoleErrors.push(ev.params.args.map((a) => a.value || a.description).join(" "));
        } else if (ev.method === "Network.responseReceived" && ev.params.response.status >= 400) {
          httpErrors.push(`${ev.params.response.status} ${ev.params.response.url}`);
        }
      }

      const domCheck = await cdp.evalJS(`(() => {
        const katexErr = document.querySelectorAll('.katex-error').length;
        const canvases = Array.from(document.querySelectorAll('canvas'));
        const canvasResults = canvases.map((c) => {
          if (c.width === 0 || c.height === 0) return { id: c.id, nonEmpty: false, reason: 'zero-dim' };
          const ctx = c.getContext('2d');
          const d = ctx.getImageData(0, 0, Math.min(c.width, 400), Math.min(c.height, 300)).data;
          let first = [d[0], d[1], d[2], d[3]], diff = 0;
          for (let i = 0; i < d.length; i += 16) {
            if (Math.abs(d[i] - first[0]) > 4 || Math.abs(d[i+1] - first[1]) > 4 || Math.abs(d[i+2] - first[2]) > 4) { diff++; break; }
          }
          return { id: c.id, nonEmpty: diff > 0 };
        });
        const brokenImgs = Array.from(document.querySelectorAll('img')).filter((img) => !img.complete || img.naturalWidth === 0).map((i) => i.src);
        return { katexErr, canvasResults, brokenImgs };
      })()`);

      let drawerOk = true;
      if (pg.testDrawer) {
        await cdp.evalJS(`(() => {
          const btn = document.querySelector('[data-file][data-lines]');
          if (btn) btn.click();
        })()`);
        await sleep(500);
        const drawerState = await cdp.evalJS(`(() => {
          const d = document.getElementById('code-drawer');
          const open = Boolean(d && d.classList.contains('open'));
          const lines = d ? d.querySelectorAll('.src-line').length : 0;
          const hl = d ? d.querySelectorAll('.src-line.hl').length : 0;
          const vscHref = document.getElementById('drawer-vscode-btn')?.getAttribute('href') || '';
          const close = document.getElementById('drawer-close-btn');
          if (close) close.click();
          return { open, lines, hl, vscHref };
        })()`);
        const validVsc = drawerState.vscHref.startsWith('vscode://vscode-remote/wsl+');
        drawerOk = drawerState.open && drawerState.lines > 10 && drawerState.hl >= 1 && validVsc;
        if (!drawerOk) failures.push(`${pg.rel}: drawer failed (${JSON.stringify(drawerState)})`);
      }

      if (pg.testWalkthrough) {
        const wtCheck = await cdp.evalJS(`(() => {
          const umlCards = document.querySelectorAll('#wt-uml-svg .uml-card').length;
          const flowNodes = document.querySelectorAll('#wt-flow-strip .wt-flow-node').length;
          const beforeTitle = document.querySelector('#wt-fn-detail h3')?.textContent || '';
          const secondFlow = document.querySelectorAll('#wt-flow-strip .wt-flow-node')[4];
          if (secondFlow) secondFlow.click();
          const afterFlowTitle = document.querySelector('#wt-fn-detail h3')?.textContent || '';
          const umlCard = document.querySelector('#wt-uml-svg .uml-card[data-cls="MapField"]');
          if (umlCard) { umlCard.dispatchEvent(new MouseEvent('click', { bubbles: true })); }
          const afterUmlTitle = document.querySelector('#wt-fn-detail h3')?.textContent || '';
          return { umlCards, flowNodes, beforeTitle, afterFlowTitle, afterUmlTitle };
        })()`);
        if (wtCheck.umlCards < 8 || wtCheck.flowNodes < 7 || wtCheck.beforeTitle === wtCheck.afterFlowTitle || wtCheck.afterFlowTitle === wtCheck.afterUmlTitle) {
          failures.push(`${pg.rel}: walkthrough interactive switch failed (${JSON.stringify(wtCheck)})`);
        }
      }

      if (pg.shot) {
        const cap = await cdp.send("Page.captureScreenshot", { format: "png" });
        fs.writeFileSync(path.join(SCREENSHOT_DIR, pg.shot), Buffer.from(cap.data, "base64"));
      }

      if (consoleErrors.length) failures.push(`${pg.rel} consoleErrors: ${consoleErrors.join("; ")}`);
      if (httpErrors.length) failures.push(`${pg.rel} httpErrors: ${httpErrors.join("; ")}`);
      if (domCheck.katexErr > 0) failures.push(`${pg.rel} katexErrors: ${domCheck.katexErr}`);
      if (domCheck.brokenImgs.length) failures.push(`${pg.rel} brokenImgs: ${domCheck.brokenImgs.join(", ")}`);
      for (const cr of domCheck.canvasResults) {
        if (!cr.nonEmpty) failures.push(`${pg.rel} blank canvas: ${cr.id}`);
      }

      console.log(`CHECK ${pg.rel} -> canvases=${domCheck.canvasResults.length} katexErr=${domCheck.katexErr} brokenImgs=${domCheck.brokenImgs.length} httpErr=${httpErrors.length} jsErr=${consoleErrors.length}${pg.testDrawer ? " drawer=" + drawerOk : ""}`);
    }

    assert.equal(failures.length, 0, "Browser audit failures:\n" + failures.join("\n"));
    console.log(`\nPASS: All ${PAGES.length} tutorial pages audited with zero errors!`);
  } finally {
    if (cdp) cdp.close();
    chrome.kill("SIGKILL");
    server.kill("SIGKILL");
    fs.rmSync(profileDir, { recursive: true, force: true });
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
