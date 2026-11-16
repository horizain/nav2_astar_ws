#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可视化课堂构建器（滚动网页版）—— 运行 `python3 slides/build.py` 重新生成全部页面。

内容模块 c_*.py 与放映版共用（S/qa/demo 返回 dict）。
每讲渲染为一个普通滚动页面：页头 + 小节卡片 + 交互实验 + 上一篇/下一篇。
交互实验 DOM 从旧版 pages_b.py 自动提取，脚本按页面实际用到的演示注入并在加载时执行。
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)          # docs/website/
sys.path.insert(0, HERE)

import framework as fw

# ============================================================
# 第 1 步：注入演示 DOM 与脚本（必须在导入内容模块之前）
# ============================================================
sys.path.insert(0, ROOT)
import pages_b  # noqa: E402  旧版页面源（bodies + *_SCRIPT）

DEMO_SRC = {  # 演示名 -> (文件名, 脚本变量名)
    'sim': ('sim.html', 'SIM_SCRIPT'),
    'og':  ('og.html',  'OG_SCRIPT'),
    'bf':  ('bf.html',  'BF_SCRIPT'),
    'ekf': ('ekf.html', 'EKF_SCRIPT'),
    'pf':  ('pf.html',  'PF_SCRIPT'),
    'sm':  ('sm.html',  'SM_SCRIPT'),
    'go':  ('go.html',  'GO_SCRIPT'),
}

DEMO_RE = re.compile(r'<canvas[\s\S]*?<div class="readout"[^>]*>[\s\S]*?</div>')
MOBJ = {'sim': 'SIM', 'og': 'OG', 'bf': 'BF', 'ekf': 'EKF', 'pf': 'PF', 'sm': 'SM', 'go': 'GO'}
bodies = {fname: body for fname, (_, body, _s) in pages_b.EXTRA_PAGES.items()}

demo_html, demo_js = {}, []
for name, (fname, script_var) in DEMO_SRC.items():
    body = bodies[fname]
    m = DEMO_RE.search(body)
    assert m, f'{fname}: 未找到演示块'
    demo_html[name] = m.group(0)
    script = getattr(pages_b, script_var)
    script = re.search(r'<script>([\s\S]*?)</script>', script).group(1)
    script = script.replace('</script', '<\\/script')  # 防提前闭合
    # 脚本被包进函数后，模块对象（const SIM/OG/...）成为局部变量；
    # 而 DOM 里的内联 onclick="OG.beam(0)" 需要全局可见 —— 暴露到 window
    obj = MOBJ[name]
    first_const = re.search(r'const (\w+)\s*=', script)
    assert first_const and first_const.group(1) == obj, \
        f'{name}: 期望模块对象 {obj}，实际 {first_const.group(1) if first_const else None}'
    demo_js.append(
        f"__demos['{name}'] = function(){{\n{script}\n"
        f"try {{ window.{obj} = {obj}; }} catch(e) {{}}\n}};")

fw.DEMO_HTML.update(demo_html)
ALL_DEMO_JS = '\n\n'.join(demo_js)

# ============================================================
# 第 2 步：导入内容模块
# ============================================================
import c_home, c_sim, c_og, c_bf, c_ekf, c_pf, c_sm, c_go, c_eng, c_res  # noqa: E402

DECKS = [c_home.DECK, c_sim.DECK, c_og.DECK, c_bf.DECK, c_ekf.DECK,
         c_pf.DECK, c_sm.DECK, c_go.DECK, c_eng.DECK, c_res.DECK]

# ============================================================
# 第 3 步：渲染为滚动页面
# ============================================================
def demo_js_block(used):
    """为页面生成演示脚本：定义 + 立即执行用到的演示。"""
    if not used:
        return ''
    calls = ''.join(f"try {{ __demos['{n}'](); }} catch(e) {{ console.error('demo {n} error:', e); }}\n"
                    for n in used)
    return (f'<script>window.__demos = {{}};\n{ALL_DEMO_JS}\n'
            f'// 启动本页用到的演示\n{calls}</script>')

if __name__ == '__main__':
    total = 0
    for i, deck in enumerate(DECKS):
        prev_deck = DECKS[i-1] if i > 0 else None
        next_deck = DECKS[i+1] if i < len(DECKS)-1 else None

        # 页头：取第一张 cover 小节做 hero
        first = deck['slides'][0]
        if first.get('cls') == 'cover':
            hero = (f'<div class="hero"><span class="tag">{deck["chapter"]}</span>'
                    f'<h1>{first["title"]}</h1>'
                    f'<div class="lead">{first["body"]}</div></div>')
            rest = deck['slides'][1:]
        else:
            hero = (f'<div class="hero"><span class="tag">{deck["chapter"]}</span>'
                    f'<h1>{deck["title"]}</h1></div>')
            rest = deck['slides']

        # 小节渲染（cover 也渲染成普通卡片，保留其内容）
        cards, used_demos = [], []
        no = 0
        for item in rest:
            if item.get('cls') == 'cover':
                # 总结页等 cover：去掉 hero 用途，正常渲染
                item = dict(item, cls='')
            if item.get('kind') == 'demo' and item.get('shrink'):
                item = dict(item, shrink=None)   # 滚动版无需缩放
            html, dn = fw.render_section(item, no if item['kind'] == 'slide' else '')
            if item['kind'] == 'slide':
                no += 1
            cards.append(html)
            if dn:
                used_demos.append(dn)

        html = fw.page_chrome(deck['file'], hero, '\n'.join(cards),
                              demo_js_block(used_demos),
                              prev_deck=prev_deck, next_deck=next_deck)
        out = os.path.join(ROOT, deck['file'])
        with open(out, 'w', encoding='utf-8') as f:
            f.write(html)
        total += len(deck['slides'])
        print(f"生成 {deck['file']:12} {deck['chapter']:18} {len(deck['slides']):3} 小节 "
              f"({len(html)//1024} KB, 演示: {','.join(used_demos) or '—'})")
    print(f'全部完成：{len(DECKS)} 页滚动网页，共 {total} 小节')
