#!/usr/bin/env python3
"""SLAM 可视化学习站点生成器 —— 运行 `python3 build.py` 重新生成全部页面。

页面拆分原则（doc 系列同款）：一个主题一页 + 上一篇/下一篇跳转。
内容修改：直接编辑本文件里对应页面的 BODY 字符串，再运行本脚本。
"""
import os

OUT = os.path.dirname(os.path.abspath(__file__))

# ============================================================
# 共享样式
# ============================================================
CSS = """:root{
  --bg0:#06080f; --panel:rgba(18,24,40,.72); --panel2:rgba(26,34,56,.6);
  --line:rgba(120,140,190,.16); --text:#e6edf8; --dim:#94a3bd;
  --cy:#22d3ee; --vi:#a78bfa; --pk:#f472b6; --ok:#34d399; --wa:#fbbf24; --bad:#fb7185;
}
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{
  background:
    radial-gradient(1200px 600px at 15% -10%, rgba(34,211,238,.10), transparent 60%),
    radial-gradient(1000px 700px at 110% 10%, rgba(167,139,250,.10), transparent 55%),
    radial-gradient(900px 700px at 50% 120%, rgba(244,114,182,.08), transparent 60%),
    var(--bg0);
  color:var(--text);
  font-family:"Segoe UI","Microsoft YaHei","PingFang SC",system-ui,sans-serif;
  line-height:1.75; min-height:100vh;
}
::selection{background:rgba(34,211,238,.35)}
::-webkit-scrollbar{width:10px;height:10px}
::-webkit-scrollbar-thumb{background:rgba(120,140,190,.25);border-radius:8px}
.wrap{max-width:1180px;margin:0 auto;padding:0 22px}
.topnav{position:sticky;top:0;z-index:50;
  background:rgba(8,11,20,.72);backdrop-filter:blur(14px);-webkit-backdrop-filter:blur(14px);
  border-bottom:1px solid var(--line)}
.topnav .wrap{display:flex;align-items:center;gap:12px;padding:10px 22px;flex-wrap:wrap}
.brand{font-weight:800;font-size:16px;letter-spacing:.5px;text-decoration:none;
  background:linear-gradient(120deg,#22d3ee,#818cf8 55%,#f472b6);
  -webkit-background-clip:text;background-clip:text;color:transparent;white-space:nowrap}
.topnav a{color:var(--dim);text-decoration:none;font-size:13.5px;padding:6px 12px;border-radius:8px;transition:.15s;white-space:nowrap}
.topnav a:hover{color:var(--text);background:rgba(120,140,190,.10)}
.topnav a.on{color:#06101f;background:linear-gradient(120deg,#22d3ee,#67e8f9);font-weight:700}
.hero{position:relative;padding:54px 0 34px;overflow:hidden}
#heroBg{position:absolute;inset:0;opacity:.55;pointer-events:none}
.hero .wrap{position:relative}
.hero h1{font-size:40px;line-height:1.25;font-weight:900}
.hero h1 em{font-style:normal;background:linear-gradient(120deg,#22d3ee,#818cf8 55%,#f472b6);
  -webkit-background-clip:text;background-clip:text;color:transparent}
.hero p.lead{color:var(--dim);font-size:15.5px;max-width:780px;margin-top:12px}
.chips{margin-top:18px;display:flex;gap:10px;flex-wrap:wrap}
.chip{border:1px solid var(--line);background:var(--panel);border-radius:999px;
  padding:6px 16px;font-size:13px;color:var(--dim)}
.chip b{color:var(--cy)}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;
  padding:24px;margin-top:20px;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);
  transition:border-color .2s;position:relative;overflow:hidden}
.card:hover{border-color:rgba(34,211,238,.4)}
.card h2{font-size:21px;margin-bottom:4px}
.card h2 .no{color:var(--vi);font-family:Consolas,monospace;margin-right:8px}
.sub{color:var(--dim);font-size:13.5px;margin-bottom:12px}
h3{font-size:15.5px;margin:20px 0 8px;color:var(--cy)}
h4{font-size:14px;margin:14px 0 6px;color:var(--vi)}
p{margin:8px 0;font-size:14px}
.muted{color:var(--dim)}
.formula{background:rgba(8,11,20,.85);border:1px solid var(--line);border-radius:10px;
  padding:12px 16px;font-family:Consolas,"Cascadia Code",monospace;font-size:13.5px;
  color:#a5f3fc;margin:12px 0;overflow-x:auto;white-space:pre-wrap;line-height:1.7}
.note{background:rgba(34,211,238,.07);border-left:3px solid var(--cy);border-radius:8px;
  padding:11px 15px;font-size:13.5px;margin:12px 0}
.warnbox{background:rgba(251,191,36,.07);border-left:3px solid var(--wa);border-radius:8px;
  padding:11px 15px;font-size:13.5px;margin:12px 0}
canvas{background:rgba(8,11,20,.9);border:1px solid var(--line);border-radius:12px;
  display:block;margin:14px auto;max-width:100%;box-shadow:0 0 40px rgba(34,211,238,.06) inset}
.ctrls{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:14px 0}
.ctrls button{background:linear-gradient(120deg,#22d3ee,#60a5fa);color:#06101f;border:0;
  padding:9px 18px;border-radius:9px;cursor:pointer;font-size:13.5px;font-weight:700;transition:.15s}
.ctrls button:hover{filter:brightness(1.12);transform:translateY(-1px)}
.ctrls button.ghost{background:var(--panel2);color:var(--text);border:1px solid var(--line);font-weight:600}
.ctrls label{color:var(--dim);font-size:13px}
.ctrls input[type=range]{width:160px;accent-color:#22d3ee;vertical-align:middle}
.readout{background:rgba(8,11,20,.9);border:1px solid var(--line);border-radius:10px;
  padding:11px 15px;font-family:Consolas,monospace;font-size:12.5px;color:#86efac;
  margin-top:12px;white-space:pre-wrap;overflow-x:auto;line-height:1.6}
table{border-collapse:collapse;width:100%;margin:12px 0;font-size:13.5px}
th,td{border:1px solid var(--line);padding:7px 12px;text-align:left}
th{background:rgba(26,34,56,.7);color:var(--cy);font-weight:700}
tr:hover td{background:rgba(34,211,238,.04)}
.kbd{background:var(--panel2);border:1px solid var(--line);border-radius:6px;padding:1px 8px;
  font-family:Consolas,monospace;font-size:12.5px;color:#a5f3fc}
details{background:var(--panel2);border:1px solid var(--line);border-radius:10px;
  padding:10px 16px;margin:10px 0}
details summary{cursor:pointer;color:var(--vi);font-size:14px;font-weight:700}
details[open] summary{margin-bottom:8px}
footer{color:var(--dim);font-size:12.5px;padding:26px 0 40px;text-align:center}
.small{font-size:12.5px;color:var(--dim)}
b.cy{color:var(--cy)} b.vi{color:var(--vi)} b.pk{color:var(--pk)} b.ok{color:var(--ok)} b.wa{color:var(--wa)}
.pn{display:flex;justify-content:space-between;gap:14px;margin-top:26px}
.pn a{flex:1;background:var(--panel2);border:1px solid var(--line);border-radius:10px;
  padding:12px 16px;text-decoration:none;color:var(--text);transition:.15s}
.pn a:hover{border-color:var(--cy)}
.pn .dir{color:var(--dim);font-size:12px}
.pn .ttl{color:var(--cy);font-size:14px;font-weight:700}
.pn .next{text-align:right}
"""

# ============================================================
# 导航与页面骨架
# ============================================================
NAV = [
    ("index.html", "首页"),
    ("sim.html", "仿真实验室"),
    ("og.html", "① 栅格建图"),
    ("bf.html", "② 贝叶斯滤波"),
    ("ekf.html", "③ EKF"),
    ("pf.html", "④ 粒子滤波"),
    ("sm.html", "⑤ 扫描匹配"),
    ("go.html", "⑥ 图优化回环"),
    ("eng.html", "⑦ 工程实战"),
    ("res.html", "资源"),
]

def chrome(active, title, body, script=""):
    nav_html = ""
    for f, name in NAV:
        cls = ' class="on"' if f == active else ""
        nav_html += f'<a href="{f}"{cls}>{name}</a>'
    files = [f for f, _ in NAV]
    idx = files.index(active)
    pn = ""
    if idx > 0:
        pf_, pn_ = NAV[idx - 1]
        pn += f'<a href="{pf_}"><div class="dir">← 上一篇</div><div class="ttl">{pn_}</div></a>'
    if idx < len(NAV) - 1:
        nf_, nn_ = NAV[idx + 1]
        pn += f'<a class="next" href="{nf_}" style="flex:1;background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:12px 16px;text-decoration:none;color:var(--text)"><div class="dir">下一篇 →</div><div class="ttl">{nn_}</div></a>'
    if pn:
        pn = f'<div class="pn">{pn}</div>'
    return f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title} · SLAM 可视化实验室</title>
<link rel="stylesheet" href="style.css">
</head>
<body>
<nav class="topnav"><div class="wrap">
  <a class="brand" href="index.html">SLAM 可视化实验室</a>
  {nav_html}
</div></nav>
<div class="wrap">
{body}
{pn}
</div>
<footer>nav2 教学实验室 · SLAM 可视化系列 · 由 build.py 生成（编辑内容后重新运行）</footer>
{script}
</body>
</html>'''

# ============================================================
# 首页
# ============================================================
HOME_BODY = '''
<div class="hero">
  <canvas id="heroBg"></canvas>
  <div class="wrap">
    <h1>把 <em>SLAM</em> 拆开，在浏览器里亲手跑一遍</h1>
    <p class="lead">栅格建图 · 贝叶斯滤波 · EKF · 粒子滤波 · 扫描匹配 · 图优化回环 ——
       六个核心零件全部做成可交互仿真，外加一个<b style="color:var(--cy)">实时 SLAM 实验室</b>：
       看机器人自动探索、激光逐帧扫描、地图一格一格生长、扫描匹配把漂移拉回来。</p>
    <div class="chips">
      <span class="chip">📖 原理 <b>doc 21</b>（每个算法手算）</span>
      <span class="chip">🔧 管线实测 <b>doc 20</b>（slam_toolbox）</span>
      <span class="chip">⚙️ 工程实战 <b>doc 22</b>（参数/标定/抗干扰）</span>
      <span class="chip">💻 终端版 <b>slam_playground.py</b></span>
    </div>
  </div>
</div>
<div class="card">
  <h2>学习地图</h2>
  <div class="sub">推荐顺序：仿真实验室（先看现象）→ 六个模块（懂原理）→ 工程实战（会落地）。
  每页底部有 上一篇/下一篇 跳转。</div>
  <table>
    <tr><th>页面</th><th>回答的问题</th><th>交互玩法</th><th>文档</th></tr>
    <tr><td><a href="sim.html">🏭 仿真实验室</a></td><td>完整 SLAM 长什么样？</td><td>自动探索 + 激光扫描 + 地图生长 + 匹配纠偏，全程可视化</td><td>doc 20/21/22</td></tr>
    <tr><td><a href="og.html">① 占据栅格</a></td><td>已知位姿，怎么画地图？</td><td>发射激光束，看 log-odds 逐格投票与噪声对冲</td><td>doc 21 §3</td></tr>
    <tr><td><a href="bf.html">② 贝叶斯滤波</a></td><td>噪声世界怎么合并证据？</td><td>逐步"测门/右移"，看信念条打破对称</td><td>doc 21 §2</td></tr>
    <tr><td><a href="ekf.html">③ EKF-SLAM</a></td><td>高斯世界怎么同时估计？</td><td>拖噪声滑块，看协方差收缩与"关联"诞生</td><td>doc 21 §4</td></tr>
    <tr><td><a href="pf.html">④ 粒子滤波</a></td><td>多峰世界怎么办？</td><td>200 粒子从迷失到锁定的旗舰动画</td><td>doc 21 §5</td></tr>
    <tr><td><a href="sm.html">⑤ 扫描匹配</a></td><td>两帧激光怎么对齐？</td><td>拖偏移量看评分实时变化、里程计先验破歧义</td><td>doc 21 §6</td></tr>
    <tr><td><a href="go.html">⑥ 图优化回环</a></td><td>漂移怎么被全局拉回？</td><td>拖回环 σ 滑块，看节点被"弹簧"拉回</td><td>doc 21 §7</td></tr>
    <tr><td><a href="eng.html">⑦ 工程实战</a></td><td>参数/标定/抗干扰</td><td>slam_toolbox 参数逐条 + 标定手算 + 故障速查</td><td>doc 22</td></tr>
  </table>
  <h3>三个必须先建立的直觉</h3>
  <p><b class="cy">① 里程计必然漂移</b> —— 轮径磨损 3% → 走 5m 漂 15cm；轮距差 2% → 每转一圈偏 7°。
     误差是<b>积分器</b>，随时间累积。SLAM 的本质就是用外部观测（激光/相机）不断把漂移拉回来。</p>
  <p><b class="vi">② 信念是分布，不是一个数</b> —— 硬说"机器人在 (2.3, 1.1)"是假装不糊涂。
     正确表达是概率分布。SLAM 全文都在回答：分布怎么随「我动了」「我看到了」更新。</p>
  <p><b class="pk">③ 地图是概率场，不是照片</b> —— "墙"只是占据概率 &gt; 阈值的格子。
     噪声票会被后续观测对冲——这是动态物体不留永久幽灵墙的数学根基。</p>
  <div class="note">📌 学习路径建议：<b>先玩模块 → 读对应文档章节 → 拿纸笔重算 → 改参数看变化</b>。
     "看了就理解"来自这个循环，不来自任何单一材料。终端版六演示：
     <span class="kbd">python3 docs/tools/slam_playground.py --all</span></div>
</div>
<script>
(function(){
  const cv=document.getElementById('heroBg'), ctx=cv.getContext('2d');
  function resize(){ cv.width=cv.offsetWidth||1200; cv.height=cv.offsetHeight||380; }
  resize(); window.addEventListener('resize',resize);
  const pts=[]; for(let i=0;i<70;i++) pts.push({
    x:Math.random()*1600, y:Math.random()*400,
    vx:(Math.random()-.5)*.5, vy:(Math.random()-.5)*.5});
  const hov={x:800,y:200};
  cv.parentElement.addEventListener('mousemove',e=>{
    const r=cv.getBoundingClientRect(); hov.x=e.clientX-r.left; hov.y=e.clientY-r.top;});
  (function loop(){
    ctx.clearRect(0,0,cv.width,cv.height);
    for(const p of pts){
      p.x+=p.vx; p.y+=p.vy;
      if(p.x<0||p.x>cv.width)p.vx*=-1;
      if(p.y<0||p.y>cv.height)p.vy*=-1;
      ctx.fillStyle='rgba(34,211,238,.55)'; ctx.fillRect(p.x,p.y,2,2);
      const d=Math.hypot(p.x-hov.x,p.y-hov.y);
      if(d<140){ ctx.strokeStyle=`rgba(167,139,250,${.35*(1-d/140)})`;
        ctx.beginPath(); ctx.moveTo(p.x,p.y); ctx.lineTo(hov.x,hov.y); ctx.stroke(); }
    }
    for(let i=0;i<pts.length;i++) for(let j=i+1;j<pts.length;j++){
      const d=Math.hypot(pts[i].x-pts[j].x,pts[i].y-pts[j].y);
      if(d<110){ ctx.strokeStyle=`rgba(34,211,238,${.14*(1-d/110)})`;
        ctx.beginPath(); ctx.moveTo(pts[i].x,pts[i].y); ctx.lineTo(pts[j].x,pts[j].y); ctx.stroke(); }
    }
    requestAnimationFrame(loop);
  })();
})();
</script>
'''

PAGES = {
    'index.html': ('学习地图', HOME_BODY, ''),
}

if __name__ == '__main__':
    with open(os.path.join(OUT, 'style.css'), 'w') as f:
        f.write(CSS)
    print('生成 style.css')
    from pages_b import EXTRA_PAGES   # 其余页面在 pages_b.py（避免单文件过长）
    PAGES.update(EXTRA_PAGES)
    for fname, (title, body, script) in PAGES.items():
        html = chrome(fname, title, body, script)
        with open(os.path.join(OUT, fname), 'w') as f:
            f.write(html)
        print(f'生成 {fname} ({len(html)} chars)')
    print('全部完成')
