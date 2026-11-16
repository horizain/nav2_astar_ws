#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可视化课堂（滚动网页版）渲染框架。

内容文件用法（与放映版共用）：
  S('标题', body_html, sub='副标题', frags=[...], notes='讲稿提示', cls='cover')
  qa('问题', ['选项A','选项B','选项C'], 答案索引, '解析')   # 测验小节，答案折叠
  demo('og', '标题', watch=['看点1','看点2'])                # 嵌入交互实验

build.py 把每个 DECK 渲染成一个普通滚动页面：顶部导航 + 小节卡片 + 上一篇/下一篇。
S/qa/demo 返回 dict（内容结构），由 render_section() 渲染为网页小节。
"""

# ============================================================
# 样式（滚动网页版）
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
  line-height:1.8; min-height:100vh;
}
::selection{background:rgba(34,211,238,.35)}
::-webkit-scrollbar{width:10px;height:10px}
::-webkit-scrollbar-thumb{background:rgba(120,140,190,.25);border-radius:8px}
.wrap{max-width:1180px;margin:0 auto;padding:0 22px}
.topnav{position:sticky;top:0;z-index:50;
  background:rgba(8,11,20,.78);backdrop-filter:blur(14px);-webkit-backdrop-filter:blur(14px);
  border-bottom:1px solid var(--line)}
.topnav .wrap{display:flex;align-items:center;gap:10px;padding:10px 22px;flex-wrap:wrap}
.brand{font-weight:800;font-size:16px;letter-spacing:.5px;text-decoration:none;
  background:linear-gradient(120deg,#22d3ee,#818cf8 55%,#f472b6);
  -webkit-background-clip:text;background-clip:text;color:transparent;white-space:nowrap}
.topnav a{color:var(--dim);text-decoration:none;font-size:13.5px;padding:6px 12px;border-radius:8px;transition:.15s;white-space:nowrap}
.topnav a:hover{color:var(--text);background:rgba(120,140,190,.10)}
.topnav a.on{color:#06101f;background:linear-gradient(120deg,#22d3ee,#67e8f9);font-weight:700}
.hero{padding:46px 0 10px}
.hero .tag{display:inline-block;background:rgba(34,211,238,.12);border:1px solid rgba(34,211,238,.35);
  color:var(--cy);border-radius:999px;padding:3px 16px;font-size:14px;font-weight:700;margin-bottom:14px}
.hero h1{font-size:36px;line-height:1.3;font-weight:900}
.hero .lead{color:var(--dim);font-size:15.5px;max-width:860px;margin-top:12px}
.hero .lead p.big{color:var(--text)}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;
  padding:26px 30px;margin-top:22px;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);
  transition:border-color .2s;position:relative;overflow:hidden}
.card:hover{border-color:rgba(34,211,238,.4)}
.card h2{font-size:22px;margin-bottom:6px}
.card h2 .no{color:var(--vi);font-family:Consolas,monospace;margin-right:10px}
.sub{color:var(--dim);font-size:14px;margin-bottom:10px}
h3{font-size:16px;margin:20px 0 8px;color:var(--cy)}
h4{font-size:14.5px;margin:14px 0 6px;color:var(--vi)}
p{margin:9px 0;font-size:15px}
li{margin:6px 0;font-size:15px;margin-left:24px}
.muted{color:var(--dim)}
.formula{background:rgba(8,11,20,.85);border:1px solid var(--line);border-left:3px solid var(--vi);
  border-radius:10px;padding:13px 18px;font-family:Consolas,"Cascadia Code",monospace;font-size:14.5px;
  color:#a5f3fc;margin:12px 0;overflow-x:auto;white-space:pre;line-height:1.8}
.formula .hl{color:var(--wa)}
.note{background:rgba(34,211,238,.07);border-left:3px solid var(--cy);border-radius:8px;
  padding:11px 15px;font-size:14.5px;margin:12px 0}
.warnbox{background:rgba(251,191,36,.07);border-left:3px solid var(--wa);border-radius:8px;
  padding:11px 15px;font-size:14.5px;margin:12px 0}
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
table{border-collapse:collapse;width:100%;margin:12px 0;font-size:14px}
th,td{border:1px solid var(--line);padding:7px 12px;text-align:left}
th{background:rgba(26,34,56,.7);color:var(--cy);font-weight:700}
td.mono,span.mono{font-family:Consolas,monospace;color:#a5f3fc}
tr:hover td{background:rgba(34,211,238,.04)}
.kbd{background:var(--panel2);border:1px solid var(--line);border-radius:6px;padding:1px 8px;
  font-family:Consolas,monospace;font-size:12.5px;color:#a5f3fc}
details{background:var(--panel2);border:1px solid var(--line);border-radius:10px;
  padding:10px 16px;margin:12px 0}
details summary{cursor:pointer;color:var(--vi);font-size:14.5px;font-weight:700}
details[open] summary{margin-bottom:8px}
b.cy{color:var(--cy)} b.vi{color:var(--vi)} b.pk{color:var(--pk)} b.ok{color:var(--ok)} b.wa{color:var(--wa)} b.bad{color:var(--bad)}
.math{margin:16px 0;overflow-x:auto;overflow-y:hidden;text-align:center;padding:4px 0}
.math .katex-display{margin:10px 0}
.math .katex{font-size:1.14em;text-align:left}
.math .muted{font-size:14px;text-align:left}
.katex-display>.katex{white-space:normal}
.quiz-opt{list-style:none;margin:10px 0;padding:0}
.quiz-opt li{margin:8px 0;border:1px solid var(--line);border-radius:10px;padding:8px 16px;font-size:15px}
.quiz-opt li.good{background:rgba(52,211,153,.10);border-color:rgba(52,211,153,.4)}
.pn{display:flex;justify-content:space-between;gap:14px;margin-top:28px}
.pn a{flex:1;background:var(--panel2);border:1px solid var(--line);border-radius:10px;
  padding:12px 16px;text-decoration:none;color:var(--text);transition:.15s}
.pn a:hover{border-color:var(--cy)}
.pn .dir{color:var(--dim);font-size:12px}
.pn .ttl{color:var(--cy);font-size:14px;font-weight:700}
.pn .next{text-align:right}
footer{color:var(--dim);font-size:12.5px;padding:26px 0 40px;text-align:center}
"""

# ============================================================
# 内容构造器：返回 dict（与放映版共用内容模块）
# ============================================================
def S(title, body, sub=None, frags=None, notes=None, cls=''):
    return {'kind': 'slide', 'title': title, 'sub': sub, 'body': body,
            'frags': list(frags or []), 'notes': notes, 'cls': cls}

def qa(question, options, answer, explain, notes=None):
    return {'kind': 'qa', 'title': question[:30] + '…', 'question': question,
            'options': list(options), 'answer': answer, 'explain': explain, 'notes': notes}

def demo(name, title, sub=None, watch=None, shrink=None, notes=None):
    return {'kind': 'demo', 'name': name, 'title': title, 'sub': sub,
            'watch': list(watch or []), 'shrink': shrink, 'notes': notes}

def _demo_h(name):
    m = __import__('re').search(r'<canvas[^>]*height="(\d+)"', DEMO_HTML[name])
    return int(m.group(1)) if m else 400

# DEMO_HTML / 演示脚本由 build.py 注入（从旧版页面自动提取）
DEMO_HTML = {}

# ============================================================
# 网页渲染
# ============================================================
def render_section(item, no):
    """一个小节 = 一张卡片。返回 (html, used_demo_or_None)。"""
    if item['kind'] == 'qa':
        marks = ['A', 'B', 'C', 'D']
        opts = ''.join(
            f'<li class="{"good" if i == item["answer"] else ""}"><b class="{"ok" if i == item["answer"] else "muted"}">{marks[i]}.</b> {o}</li>'
            for i, o in enumerate(item['options']))
        body = (f'<p class="big" style="font-size:16.5px">{item["question"]}</p>'
                f'<ul class="quiz-opt" style="max-width:900px">{opts}</ul>'
                f'<details><summary>看答案与解析</summary>'
                f'<div class="note">✅ 正确答案：<b class="ok">{marks[item["answer"]]}. {item["options"][item["answer"]]}</b><br>{item["explain"]}</div></details>')
        return (f'<div class="card"><h2><span class="no">✎</span>测验</h2>{body}</div>', None)

    if item['kind'] == 'demo':
        watch = ''
        if item['watch']:
            lis = ''.join(f'<li style="margin-left:20px">{w}</li>' for w in item['watch'])
            watch = f'<div class="note"><b class="cy">👀 观察任务</b><ul style="margin-top:4px">{lis}</ul></div>'
        sub = f'<div class="sub">{item["sub"]}</div>' if item['sub'] else ''
        body = (f'<div class="demo" data-demo="{item["name"]}">{DEMO_HTML[item["name"]]}</div>'
                f'{watch}')
        return (f'<div class="card"><h2><span class="no">▶</span>{item["title"]}</h2>{sub}{body}</div>',
                item['name'])

    # 普通小节
    frags_html = ''.join(item['frags'])
    sub = f'<div class="sub">{item["sub"]}</div>' if item['sub'] else ''
    return (f'<div class="card"><h2><span class="no">{no}</span>{item["title"]}</h2>{sub}'
            f'{item["body"]}{frags_html}</div>', None)

NAV = [
    ('index.html', '首页'),
    ('sim.html', '仿真实验室'),
    ('og.html', '① 栅格建图'),
    ('bf.html', '② 贝叶斯滤波'),
    ('ekf.html', '③ EKF'),
    ('pf.html', '④ 粒子滤波'),
    ('sm.html', '⑤ 扫描匹配'),
    ('go.html', '⑥ 图优化回环'),
    ('eng.html', '⑦ 工程实战'),
    ('res.html', '资源'),
]

def page_chrome(active, hero_html, sections_html, extra_js='', prev_deck=None, next_deck=None):
    """滚动网页骨架：顶部导航 + 页头 + 小节卡片 + 上一篇/下一篇。"""
    nav_html = ''
    for f, name in NAV:
        if f == active:
            nav_html += f'<a href="{f}" class="on">{name}</a>'
        else:
            nav_html += f'<a href="{f}">{name}</a>'
    links = ''
    if prev_deck:
        links += (f'<a href="{prev_deck["file"]}"><div class="dir">← 上一篇</div>'
                  f'<div class="ttl">{prev_deck["title"]}</div></a>')
    if next_deck:
        links += (f'<a class="next" href="{next_deck["file"]}"><div class="dir">下一篇 →</div>'
                  f'<div class="ttl">{next_deck["title"]}</div></a>')
    if links:
        pn = f'<div class="pn">{links}</div>'
    return f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{active} · SLAM 可视化课堂</title>
<link rel="stylesheet" href="vendor/katex/katex.min.css">
<script defer src="vendor/katex/katex.min.js"></script>
<script defer src="vendor/katex/contrib/auto-render.min.js"
  onload="renderMathInElement(document.body,{{delimiters:[{{left:'$$',right:'$$',display:true}},{{left:'\\(',right:'\\)',display:false}}],throwOnError:false}});"></script>
<style>{CSS}</style>
</head>
<body>
<nav class="topnav"><div class="wrap">
  <a class="brand" href="index.html">SLAM 可视化课堂</a>
  {nav_html}
</div></nav>
<div class="wrap">
{hero_html}
{sections_html}
{pn}
</div>
<footer>nav2 教学实验室 · SLAM 可视化课堂 · 由 slides/build.py 生成（编辑 slides/c_*.py 后重新运行）</footer>
{extra_js}
</body>
</html>'''
