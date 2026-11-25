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
  { rel: "chapters/ch01.html", testSe2HandsOn: true },
  { rel: "chapters/ch02.html", shot: "02_ch02.png", testEdtHandsOn: true },
  { rel: "chapters/ch03.html", testCorrelativeHandsOn: true },
  { rel: "chapters/ch04.html", shot: "04_ch04.png", testDrawer: true, testHandsOn: true },
  { rel: "chapters/ch05.html", testCovarianceHandsOn: true },
  { rel: "chapters/ch06.html", testGlobalHandsOn: true },
  { rel: "chapters/ch07.html", shot: "07_ch07.png", testDrawer: true, testEfkHandsOn: true },
  { rel: "chapters/ch08.html", shot: "08_ch08.png", testNodeHandsOn: true },
  { rel: "chapters/ch09.html", shot: "09_ch09.png", testBenchmarkHandsOn: true },
  { rel: "chapters/ch10.html", testSyntaxHandsOn: true },
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

      if (pg.testSe2HandsOn) {
        const se2Check = await cdp.evalJS(`(() => {
          const before = document.getElementById('ch01-stage-title').textContent;
          document.getElementById('ch01-stage-next').click();
          const after = document.getElementById('ch01-stage-title').textContent;
          const ex1 = document.getElementById('ch01-ex1');
          ex1.value = '1'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch01-ex2');
          ex2.value = '-1'; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch01-ex3');
          ex3.value = 'map_to_base * odom_to_base.inverse()'; ex3.closest('form').requestSubmit();
          return {
            numericVars: document.querySelectorAll('#ch01-numeric-vars .hands-on-var').length,
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (se2Check.numericVars < 10 || !se2Check.stageChanged || !se2Check.ex1Ok || !se2Check.ex2Ok || !se2Check.ex3Ok) {
          failures.push(`${pg.rel}: SE2 hands-on interaction failed (${JSON.stringify(se2Check)})`);
        }
      }

      if (pg.testCorrelativeHandsOn) {
        const corrCheck = await cdp.evalJS(`(() => {
          document.getElementById('ch03-icp-step').click();
          const before = document.getElementById('ch03-stage-title').textContent;
          document.getElementById('ch03-stage-next').click();
          const after = document.getElementById('ch03-stage-title').textContent;
          const ex1 = document.getElementById('ch03-ex1');
          ex1.value = '196'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch03-ex2');
          ex2.value = ex2.closest('form').dataset.answer; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch03-ex3');
          ex3.value = 'correlative'; ex3.closest('form').requestSubmit();
          return {
            parity: document.getElementById('ch03-parity').textContent.includes('PASS'),
            icpSynced: document.getElementById('ch03-animation-vars').textContent.includes('#1'),
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (!corrCheck.parity || !corrCheck.icpSynced || !corrCheck.stageChanged || !corrCheck.ex1Ok || !corrCheck.ex2Ok || !corrCheck.ex3Ok) {
          failures.push(`${pg.rel}: Correlative hands-on interaction failed (${JSON.stringify(corrCheck)})`);
        }
      }

      if (pg.testCovarianceHandsOn) {
        const covCheck = await cdp.evalJS(`(() => {
          document.getElementById('ch05-cov-toggle').click();
          const damped = document.getElementById('ch05-cov-input').textContent.includes('Hd');
          const before = document.getElementById('ch05-stage-title').textContent;
          document.getElementById('ch05-stage-next').click();
          const after = document.getElementById('ch05-stage-title').textContent;
          const ex1 = document.getElementById('ch05-ex1');
          ex1.value = '40'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch05-ex2');
          ex2.value = '0.0001'; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch05-ex3');
          ex3.value = 'H'; ex3.closest('form').requestSubmit();
          return {
            numericVars: document.querySelectorAll('#ch05-numeric-vars .hands-on-var').length,
            damped,
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (covCheck.numericVars < 8 || !covCheck.damped || !covCheck.stageChanged || !covCheck.ex1Ok || !covCheck.ex2Ok || !covCheck.ex3Ok) {
          failures.push(`${pg.rel}: covariance hands-on interaction failed (${JSON.stringify(covCheck)})`);
        }
      }

      if (pg.testGlobalHandsOn) {
        const globalCheck = await cdp.evalJS(`(() => {
          document.getElementById('ch06-rerun-btn').click();
          document.getElementById('ch06-trace-next').click();
          document.getElementById('ch06-source-pf').click();
          const before = document.getElementById('ch06-stage-title').textContent;
          document.getElementById('ch06-stage-next').click();
          const after = document.getElementById('ch06-stage-title').textContent;
          const ex1 = document.getElementById('ch06-ex1');
          ex1.value = '0.73'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch06-ex2');
          ex2.value = 'prune'; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch06-ex3');
          ex3.value = '1.5151515151515151'; ex3.closest('form').requestSubmit();
          return {
            candidates: document.querySelectorAll('#ch06-candidates-body tr').length,
            traceAdvanced: !document.getElementById('ch06-trace-index').textContent.startsWith('0 /'),
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (globalCheck.candidates < 1 || !globalCheck.traceAdvanced || !globalCheck.stageChanged || !globalCheck.ex1Ok || !globalCheck.ex2Ok || !globalCheck.ex3Ok) {
          failures.push(`${pg.rel}: global hands-on interaction failed (${JSON.stringify(globalCheck)})`);
        }
      }

      if (pg.testNodeHandsOn) {
        const nodeCheck = await cdp.evalJS(`(() => {
          document.querySelector('[data-life-action="configure"]').click();
          document.querySelector('[data-life-action="activate"]').click();
          const active = document.getElementById('ch08-life-vars').textContent.includes('ACTIVE');
          document.getElementById('ch08-timeline-next').click();
          const before = document.getElementById('ch08-stage-title').textContent;
          document.getElementById('ch08-stage-next').click();
          const after = document.getElementById('ch08-stage-title').textContent;
          const ex1 = document.getElementById('ch08-ex1');
          ex1.value = 'UNCONFIGURED'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch08-ex2');
          ex2.value = 'old'; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch08-ex3');
          ex3.value = '1.1'; ex3.closest('form').requestSubmit();
          return {
            active,
            timelineAdvanced: document.getElementById('ch08-timeline-meta').textContent.includes('1 /'),
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (!nodeCheck.active || !nodeCheck.timelineAdvanced || !nodeCheck.stageChanged || !nodeCheck.ex1Ok || !nodeCheck.ex2Ok || !nodeCheck.ex3Ok) {
          failures.push(`${pg.rel}: node hands-on interaction failed (${JSON.stringify(nodeCheck)})`);
        }
      }

      if (pg.testBenchmarkHandsOn) {
        const benchCheck = await cdp.evalJS(`(() => {
          document.getElementById('ch09-lab-form').requestSubmit();
          const resultsVisible = !document.getElementById('ch09-results').hidden;
          const rawRows = document.querySelectorAll('#ch09-raw-preview tr').length;
          document.getElementById('ch09-recompute').click();
          const recomputed = document.getElementById('ch09-recompute-status').textContent.includes('复算一致');
          const before = document.getElementById('ch09-stage-title').textContent;
          document.getElementById('ch09-stage-next').click();
          const after = document.getElementById('ch09-stage-title').textContent;
          const ex1 = document.getElementById('ch09-ex1');
          ex1.value = 'not_paired'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch09-ex2');
          ex2.value = 'not_fair'; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch09-ex3');
          ex3.value = 'not_valid'; ex3.closest('form').requestSubmit();
          return {
            resultsVisible,
            rawRows,
            recomputed,
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (!benchCheck.resultsVisible || benchCheck.rawRows < 1 || !benchCheck.recomputed || !benchCheck.stageChanged || !benchCheck.ex1Ok || !benchCheck.ex2Ok || !benchCheck.ex3Ok) {
          failures.push(`${pg.rel}: benchmark hands-on interaction failed (${JSON.stringify(benchCheck)})`);
        }
      }

      if (pg.testSyntaxHandsOn) {
        const syntaxCheck = await cdp.evalJS(`(() => {
          const input = document.getElementById('ch10-rebuild-input');
          input.value = 'return 0.0;';
          input.dispatchEvent(new Event('input', { bubbles: true }));
          const firstOk = document.getElementById('ch10-rebuild-feedback').classList.contains('ok');
          const before = document.getElementById('ch10-rebuild-title').textContent;
          document.getElementById('ch10-rebuild-next').click();
          const after = document.getElementById('ch10-rebuild-title').textContent;
          const ex1 = document.getElementById('ch10-ex1');
          ex1.value = 'random_access'; ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch10-ex2');
          ex2.value = '1e-4,1e-4,1'; ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch10-ex3');
          ex3.value = 'B'; ex3.closest('form').requestSubmit();
          return {
            firstOk,
            stageChanged: before !== after,
            progress: document.getElementById('ch10-rebuild-progress').textContent,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (!syntaxCheck.firstOk || !syntaxCheck.stageChanged || !syntaxCheck.progress.includes('1/16') || !syntaxCheck.ex1Ok || !syntaxCheck.ex2Ok || !syntaxCheck.ex3Ok) {
          failures.push(`${pg.rel}: syntax hands-on interaction failed (${JSON.stringify(syntaxCheck)})`);
        }
      }

      if (pg.testEdtHandsOn) {
        const edtCheck = await cdp.evalJS(`(() => {
          document.getElementById('ch02-trace-next').click();
          document.getElementById('ch02-trace-next').click();
          const traceTitle = document.getElementById('ch02-trace-title').textContent;
          const before = document.getElementById('ch02-stage-title').textContent;
          document.getElementById('ch02-stage-next').click();
          const after = document.getElementById('ch02-stage-title').textContent;
          const ex1 = document.getElementById('ch02-ex1');
          ex1.value = '-1';
          ex1.closest('form').requestSubmit();
          const ex2 = document.getElementById('ch02-ex2');
          ex2.value = '8,5,4';
          ex2.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch02-ex3');
          ex3.value = 's<=z[k]';
          ex3.closest('form').requestSubmit();
          return {
            numericVars: document.querySelectorAll('#ch02-numeric-vars .hands-on-var').length,
            traceVars: document.querySelectorAll('#ch02-trace-vars .hands-on-var').length,
            traceAtQ2: traceTitle.includes('q=2'),
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex2Ok: ex2.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (edtCheck.numericVars < 8 || edtCheck.traceVars < 8 || !edtCheck.traceAtQ2 ||
            !edtCheck.stageChanged || !edtCheck.ex1Ok || !edtCheck.ex2Ok || !edtCheck.ex3Ok) {
          failures.push(`${pg.rel}: EDT hands-on interaction failed (${JSON.stringify(edtCheck)})`);
        }
      }

      if (pg.testEfkHandsOn) {
        await cdp.evalJS(`document.getElementById('ch07-step-1').click()`);
        await sleep(250);
        const efkCheck = await cdp.evalJS(`(() => {
          const before = document.getElementById('ch07-stage-title').textContent;
          document.getElementById('ch07-stage-next').click();
          const after = document.getElementById('ch07-stage-title').textContent;
          const ex1 = document.getElementById('ch07-ex1');
          ex1.value = '9';
          ex1.closest('form').requestSubmit();
          const ex3 = document.getElementById('ch07-ex3');
          ex3.value = 'prediction';
          ex3.closest('form').requestSubmit();
          document.getElementById('ch07-step-10').click();
          const iterAfter15 = document.getElementById('ch07-live-iter').textContent;
          document.getElementById('ch07-occ-btn').click();
          document.getElementById('ch07-step-1').click();
          const rejectionUi = document.getElementById('ch07-g-score').textContent.includes('FAIL');
          const rejectedTrace = document.getElementById('ch07-live-vars').textContent.includes('false');
          return {
            liveVars: document.querySelectorAll('#ch07-live-vars .hands-on-var').length,
            liveIter: document.getElementById('ch07-live-iter').textContent,
            stageChanged: before !== after,
            ex1Ok: ex1.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            ex3Ok: ex3.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
            iterAfter15,
            rejectionUi,
            rejectedTrace,
          };
        })()`);
        if (efkCheck.liveVars < 8 || !efkCheck.liveIter.includes('#1') ||
            !efkCheck.stageChanged || !efkCheck.ex1Ok || !efkCheck.ex3Ok ||
            !efkCheck.iterAfter15.includes('#16') || !efkCheck.rejectionUi || !efkCheck.rejectedTrace) {
          failures.push(`${pg.rel}: EKF hands-on interaction failed (${JSON.stringify(efkCheck)})`);
        }
      }

      if (pg.testHandsOn) {
        await cdp.evalJS(`document.getElementById('ch04-step').click()`);
        await sleep(250);
        const handsOnCheck = await cdp.evalJS(`(() => {
          const before = document.getElementById('ch04-stage-title').textContent;
          document.getElementById('ch04-stage-next').click();
          const after = document.getElementById('ch04-stage-title').textContent;
          const input = document.getElementById('ch04-ex1');
          input.value = '0.5';
          input.closest('form').requestSubmit();
          return {
            exampleVars: document.querySelectorAll('#ch04-example-vars .hands-on-var').length,
            liveVars: document.querySelectorAll('#ch04-live-vars .hands-on-var').length,
            liveIter: document.getElementById('ch04-live-iter').textContent,
            stageChanged: before !== after,
            exerciseOk: input.closest('form').querySelector('.exercise-feedback').classList.contains('ok'),
          };
        })()`);
        if (handsOnCheck.exampleVars < 8 || handsOnCheck.liveVars < 3 ||
            !handsOnCheck.liveIter.includes('#1') || !handsOnCheck.stageChanged || !handsOnCheck.exerciseOk) {
          failures.push(`${pg.rel}: hands-on interaction failed (${JSON.stringify(handsOnCheck)})`);
        }
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
