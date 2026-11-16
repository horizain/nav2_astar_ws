#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 0 讲 · 学习地图"""
from framework import S

DECK = {
    'file': 'index.html',
    'title': '学习地图',
    'chapter': 'SLAM 可视化课堂',
    'slides': [
        S('把 SLAM 拆开，从上到下亲手学会',
          '''<p class="big">这门课把 SLAM 拆成 <b class="cy">7 讲</b>：一讲一个主题，从上往下顺序读即可。</p>
             <p>每个算法都走完整链条：<b class="vi">直觉类比 → 公式推导 → 逐行手算 → 交互实验 → 工程陷阱 → 测验 → 一页总结</b>。
             看懂 ≠ 学会；每讲的测验小节会逼你当场做决定（答案折叠着，先想再看）。</p>
             <div class="note">配套文档：原理课 <b>doc 21</b>（手算全表）· 管线实测 <b>doc 20</b>（slam_toolbox）· 工程实战 <b>doc 22</b>（标定/抗干扰/调参）。
             终端版练习：<span class="kbd">python3 docs/tools/slam_playground.py --all</span></div>''',
          cls='cover'),

        S('怎么使用这门课', '''
          <table>
            <tr><th>栏目</th><th>用法</th></tr>
            <tr><td>正文小节</td><td>从上往下顺序读；顶部导航可在十讲之间任意跳转，页面底部有 上一篇/下一篇</td></tr>
            <tr><td>手算小节</td><td>表格里数字齐全 —— 拿纸笔照着算一遍再看结论，这一步不能跳</td></tr>
            <tr><td>交互实验</td><td>页面里直接点按钮/拖滑块，所有演示离线可跑，不必装 ROS</td></tr>
            <tr><td>测验小节</td><td>先在心里选答案，再点"看答案与解析" —— 答案是折叠的，防偷看</td></tr>
            <tr><td>讲次切换</td><td>每讲末尾有 上一篇/下一篇 按钮；顶部导航条直达任意一讲</td></tr>
          </table>
          <div class="note">每个小节独立成卡片。如果你在某个小节停了 3 分钟以上 —— 正常，手算小节就该停。</div>'''),

        S('七讲的地图：从现象到落地', '''
          <table>
            <tr><th>讲</th><th>回答的问题</th><th>核心一招</th><th>交互实验</th></tr>
            <tr><td><b class="cy">① 栅格建图</b></td><td>已知位姿，怎么把地图画出来？</td><td>log-odds 逐格投票</td><td>激光投票机</td></tr>
            <tr><td><b class="cy">② 贝叶斯滤波</b></td><td>噪声世界怎么合并证据？</td><td>预测 → 观测 → 归一化</td><td>三格走廊定位</td></tr>
            <tr><td><b class="cy">③ EKF-SLAM</b></td><td>高斯世界怎么同时估计位姿+路标？</td><td>协方差膨胀/收缩 + 卡尔曼增益</td><td>协方差椭圆动画</td></tr>
            <tr><td><b class="cy">④ 粒子滤波</b></td><td>多峰世界怎么办？</td><td>N 个假设 + 加权重采样</td><td>200 粒子锁定动画</td></tr>
            <tr><td><b class="cy">⑤ 扫描匹配</b></td><td>两帧激光怎么对齐？</td><td>评分函数 + 里程计先验</td><td>拖偏移看评分</td></tr>
            <tr><td><b class="cy">⑥ 图优化回环</b></td><td>累积漂移怎么被全局拉回？</td><td>位姿图最小二乘</td><td>拖 σ 看弹簧回拉</td></tr>
            <tr><td><b class="cy">⑦ 工程实战</b></td><td>真实机器人怎么落地？</td><td>标定 / 参数 / 抗干扰 / 排障</td><td>—</td></tr>
          </table>
          <p>推荐顺序：<b class="ok">先看 仿真实验室（10 分钟了解全貌）→ 按序六讲 → 工程实战</b>。
          ①④⑥ 是主线（建图 / 定位 / 全局纠偏），②③⑤ 是它们的数学零件。</p>'''),

        S('先建立的直觉 ①：里程计必然漂移', '''
          <p class="big">轮式里程计 = 把每一步 <b class="wa">有误差</b> 的位移<b class="wa">连乘连加</b>。</p>
          <table>
            <tr><th>误差源</th><th>典型大小</th><th>后果</th></tr>
            <tr><td>轮径磨损 / 打滑</td><td>3% × 行驶距离</td><td>走 5 m 漂 15 cm</td></tr>
            <tr><td>轮距实际值 ≠ 标称值</td><td>2% × 转过的角度</td><td>原地转一圈偏 7°</td></tr>
            <tr><td>地面打滑（地毯/瓷砖）</td><td>随机、突发</td><td>方向突然错</td></tr>
          </table>
          <div class="frag"><p class="big" style="margin-top:22px">关键：误差是<b class="bad">积分器</b> —— 不修正就只会越滚越大，永远不会自己变小。</p></div>
          <div class="frag"><p>SLAM 的全部工作 = <b class="ok">用外部观测（激光/相机）不断把漂移拉回来</b>。后面五讲就是五种"拉回来"的数学方法。</p></div>''',
          notes='先问听众：为什么轮子转了 100 圈机器人却没走够 100 圈的距离？让误差积累的概念先落地。'),

        S('先建立的直觉 ②：信念是分布，不是一个数', '''
          <p class="big">硬说"机器人在 (2.3, 1.1)" 是<b class="bad">假装不糊涂</b>。</p>
          <p>正确的表达是概率分布 —— "最有可能是这里，但旁边一圈也说得通"。</p>
          <div class="formula">位置信念的三种表达：
  单点     (2.3, 1.1)                    ← 简单，但丢失了"多不确定"
  高斯     N(均值, 协方差)                ← 第 ③ 讲 EKF 的世界（单峰）
  粒子云   {(x₁,w₁), (x₂,w₂), …}         ← 第 ④ 讲 PF 的世界（可多峰）</div>
          <div class="frag"><p class="big">SLAM 全文的中心问题只有一个：<b class="cy">分布怎么随「我动了」「我看到了」更新？</b></p></div>
          <div class="frag"><p>「我动了」→ 分布变宽变糊（不确定性增大）；「我看到了」→ 分布收窄变尖（不确定性减小）。整门课就是这八个字。</p></div>''',
          notes='用走廊里两扇一模一样的门举例：单点表达在这里直接崩溃。'),

        S('先建立的直觉 ③：地图是概率场，不是照片', '''
          <p class="big">"墙"不是画上去的，是<b class="vi">投票投出来的</b>。</p>
          <p>每个格子存一个占据概率：激光打到 → 投"占据"票；激光穿过 → 投"自由"票。</p>
          <div class="formula">p &gt; 0.65  → 占据（画成黑）      ← 阈值可调
0.35 ≤ p ≤ 0.65 → 未知（灰）
p &lt; 0.35  → 自由（白）</div>
          <div class="frag"><p class="big" style="margin-top:18px">投票制的好处：<b class="ok">噪声票会被后续观测对冲</b>。</p></div>
          <div class="frag"><p>一只路过的猫投了一次"占据"票，随后 100 束激光穿过那个格子 → 票被对冲，幽灵消失。
          这就是动态物体不留永久鬼影的数学根基（第 ① 讲重点）。</p></div>'''),

        S('学习循环：怎么才算"学会了"', '''
          <p class="big">只看课程是<b class="bad">学不会</b>的。每讲请走完这个循环：</p>
          <p>🧠 <b class="cy">看懂直觉页</b> → ✍️ <b class="vi">手算页拿纸笔重算</b>（数字都给了，自己列一遍表）→ 🖱️ <b class="pk">交互页改参数观察</b> → ✅ <b class="ok">测验页做对</b> → 🔧 <b class="wa">工程页记住三个坑</b></p>
          <div class="frag"><div class="warnbox"><b>检验标准</b>：合上课程材料，能白板复述该算法的"输入 → 一步核心更新 → 输出"，并说出它什么时候会失败。做到才算过。</div></div>
          <div class="frag"><div class="note">文档与本课的分工：本课负责<b>建立直觉+手算</b>；doc 20/21/22 负责真实机器人落地（命令、参数、标定）。两边配合，不要只看一边。</div></div>''',
          cls='cover'),
    ],
}
