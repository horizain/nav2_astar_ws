#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 ① 讲 · 占据栅格建图（log-odds 投票）"""
from framework import S, demo, qa

DECK = {
    'file': 'og.html',
    'title': '① 占据栅格建图',
    'chapter': '第 ① 讲 · 占据栅格建图',
    'slides': [
        S('这一讲回答：已知机器人走到哪，地图怎么画？',
          '''<p class="big">输入：<b class="cy">位姿</b>（机器人在哪、朝哪）+ <b class="cy">激光</b>（各方向障碍物距离）
             <br>输出：<b class="vi">占据栅格地图</b>（一格一格的黑/白/灰）</p>
             <p>本讲链条：概率底子 → 贝叶斯更新 → log-odds 化简 → 手算三束激光 → 交互实验 → 工程参数与陷阱 → 测验。</p>''',
          cls='cover'),

        S('为什么机器人需要地图（一个死循环）', '''
          <p>导航需要地图（规划路径要知道哪里有墙）；但画地图又需要知道机器人位置 —— <b class="wa">鸡生蛋问题</b>：</p>
          <div class="formula">SLAM = Simultaneous Localization And Mapping
         同时定位与建图：地图和位姿<b>互相依赖</b>，只能一起估计</div>
          <p>本讲的策略是<b class="cy">先斩后奏</b>：假设位姿已知（或已由其他模块估好），
          把"画地图"这一半问题做干净 —— 这是整个 SLAM 的地基模块。
          位姿那一半怎么估，是第②~⑥讲的内容。</p>''',
          notes='一句话交代 SLAM 的"鸡生蛋"本质，防止听众后面问"位姿哪来的"。'),

        S('地图表示法：为什么选栅格？', '''
          <table>
            <tr><th>表示法</th><th>长什么样</th><th>优点</th><th>缺点</th></tr>
            <tr><td><b class="cy">占据栅格</b> ★</td><td>均匀小方格，每格存占据概率</td><td>查询 O(1)、激光直接更新、适合路径规划</td><td>内存大、分辨率固定</td></tr>
            <tr><td>点云地图</td><td>一堆 3D 点</td><td>精确、无信息损失</td><td>没法直接问"这里能不能走"</td></tr>
            <tr><td>拓扑地图</td><td>节点+边（走廊A—门—房间B）</td><td>极省内存、像人的心理地图</td><td>没有几何细节，不能规划</td></tr>
            <tr><td>特征地图</td><td>路标点/线/面列表</td><td>紧凑，EKF-SLAM 好用（第③讲）</td><td>依赖特征提取，弱纹理场景失效</td></tr>
          </table>
          <p>轮式机器人在室内跑导航，几乎都是栅格（<span class="mono">/map</span> 就是它）；
          自动驾驶常用多层混合（点云+栅格）。本课主战场：2D 栅格。</p>'''),

        S('分辨率怎么选：一格 0.05 m 意味着什么', '''
          <p>分辨率 = 一条边代表多长真实距离。常用 <b class="cy">0.05 m</b>（5 cm），机器人本体直径通常 ≥ 0.3 m。</p>
          <div class="formula">内存账：100 m × 100 m 场地，5 cm 格
  格子数 = (100/0.05)² = 2000 × 2000 = <span class="hl">400 万格</span>
  每格 float32（log-odds） = 16 MB      ← 完全可接受
  3D 版（10 cm 体素，10 层楼高）≈ 数 GB  ← 所以 3D 要八叉树压缩</div>
          <div class="frag"><p><b>选粗了</b>（20 cm）：门框 60 cm 只剩 3 格，规划器把门判成"半堵墙"。</p></div>
          <div class="frag"><p><b>选细了</b>（1 cm）：内存 ×25，每帧更新的格子数 ×25，CPU 顶不住；而且激光噪声 σ≈2 cm，
          1 cm 格里概率抖来抖去，墙反而画得毛糙 —— <b class="ok">分辨率应该和传感器噪声同量级</b>。</p></div>''',
          notes='核心结论：分辨率与传感器噪声同量级。太细是浪费+毛糙，太粗是撞门框。'),

        S('一个格子里存什么：占据概率 p(m)', '''
          <p>给世界引入随机变量 <b class="cy">m = 这个格子被占据</b>。每个格子维护：</p>
          <div class="formula">p(m)   ∈ [0, 1]    占据概率
p=0.5  完全未知      p=1 绝对是墙      p=0 绝对是空</div>
          <p>注意三个易混概念：</p>
          <li><b class="vi">未知</b>（p=0.5）：没扫到过，灰</li>
          <li><b class="vi">自由</b>（p&lt;0.35）：确定扫过、没有东西，白</li>
          <li><b class="vi">占据</b>（p&gt;0.65）：大概率有墙，黑</li>
          <div class="frag"><p class="big">麻烦来了：传感器是<b class="bad">带噪声的</b> —— 同一面墙，
          这次报 2.3 m，下次报 2.5 m。一票就定墙/空？<b class="bad">不行</b>。</p></div>''',
          notes='引出下一页：需要概率工具把多次带噪观测合并。'),

        S('概率底子速成：贝叶斯公式（3 分钟）', r'''
          <p>先验 → 看到证据 → 后验，全靠贝叶斯公式：</p>
          <div class="math">$$
p(\text{墙} \mid \text{证据}) \;=\; \frac{p(\text{证据} \mid \text{墙}) \cdot p(\text{先验})}{p(\text{证据})}
$$</div>
          <p class="muted">读法：后验 ∝ 似然 × 先验。
  似然 p(证据|墙)：如果那里真是墙，看到这个证据的可能性多大；
  先验 p(墙)：还没看证据时，你相信多少。</p>
          <div class="frag"><p class="big" style="margin-top:14px">应用到我们问题：证据 = "激光在这个格子处回波了"。</p></div>
          <div class="frag"><div class="math">$$
p(m \mid z) \;\propto\; p(z \mid m) \cdot p(m)
\qquad\Longrightarrow\qquad
\text{占据后验} \;\propto\; \text{传感器模型} \cdot \text{占据先验}
$$</div></div>
          <div class="frag"><p>每次新激光束来了，就把上一轮的<b>后验</b>当这一轮的<b>先验</b> —— 概率像接力棒一样传递。</p></div>''',
          notes='贝叶斯这一页必须慢。读法比公式重要：似然×先验→后验，后验变下一轮先验。'),

        S('传感器模型：束端投"占"票，束中投"空"票', '''
          <p>一束激光给路径上每类格子提供<b>两种证据</b>（逆传感器模型 inverse model）：</p>
          <div class="formula">束端格（回波停在那里）  p(z|m) = <span class="hl">0.7</span>   → 占据票
束中格（激光穿过去了）  p(z|¬m)= <span class="hl">0.3</span>   → 自由票
未扫到的格              不提供证据</div>
          <p>为什么束端只给 0.7 不给 0.95？—— 一次回波本来就不太确定（动态物体、噪声、玻璃）。
          确定性来自<b class="ok">多次观测的累积</b>，不来自单票的自信。</p>
          <div class="note"><b>对称性检查</b>：0.7 是"占据票"的强度，1−0.3=0.7 正好是"自由票"的对偶强度 —— 模型自洽。</div>''',
          notes='0.7/0.3 这对数值后面所有手算都用它们。'),

        S('直接乘概率的两个大麻烦', r'''
          <p>把贝叶斯接力直接写成乘法：</p>
          <div class="math">$$
p \;\leftarrow\; \frac{p(z\mid m)\cdot p}{\,p(z\mid m)\cdot p \;+\; p(z\mid \neg m)\cdot (1-p)\,}
\qquad\text{（每束一次）}
$$</div>
          <p>麻烦① <b class="bad">数值下溢</b>：0.7 连乘 100 次 ≈ 3×10⁻¹⁶ —— float64 还行，
          但连乘 1000 次（大场景）直接沉到 0，再也拉不起来。</p>
          <p>麻烦② <b class="bad">对冲难做</b>："10 票占据 + 8 票自由 ≈ 净 2 票占据" 这种<b>加减直觉</b>，
          在乘法世界要绕概率比才能表达。</p>
          <div class="frag"><p class="big">解法：换坐标系 —— 把概率搬到 <b class="cy">log-odds</b> 域，乘法变加法。</p></div>''',
          notes='先立问题再给药，这一页的"麻烦"就是下一页公式的动机。'),

        S('log-odds：定义与 p↔l 对照表', r'''
          <div class="math">$$
\text{定义：}\quad l \;=\; \ln\frac{p}{1-p}
\qquad\qquad
\text{逆映射：}\quad p \;=\; \frac{1}{1+e^{-l}}\;\;\text{（sigmoid）}
$$</div>
          <table>
            <tr><th>p</th><th>0.5</th><th>0.3</th><th>0.7</th><th>0.845</th><th>0.927</th><th>0.982</th></tr>
            <tr><td>l</td><td>0</td><td>−0.847</td><td>+0.847</td><td>1.694</td><td>2.541</td><td>4.0</td></tr>
            <tr><td>含义</td><td>未知</td><td>1 票自由</td><td>1 票占据</td><td>2 票占</td><td>3 票占</td><td>clamp 上限</td></tr>
          </table>
          <p>直觉：l 是<b class="vi">"证据净余额"</b> —— 正数偏墙，负数偏空，0 是未知。
          每张票固定 ±0.847，加加减减就是投票箱。</p>''',
          notes='这张表建议听众抄下来——后面手算全部查这张表。'),

        S('推导：贝叶斯更新在 log 域 = 加法（4 行代数）', r'''
          <div class="math">$$
\begin{aligned}
\text{① 后验 odds：}\quad \frac{p(m\mid z)}{p(\neg m\mid z)} &= \underbrace{\frac{p(z\mid m)}{p(z\mid \neg m)}}_{\text{似然比}} \cdot \underbrace{\frac{p(m)}{p(\neg m)}}_{\text{先验 odds}}\\[4pt]
\text{② 两边取对数：}\quad l(m\mid z) &= \ln\frac{p(z\mid m)}{p(z\mid \neg m)} + l(m)\\[4pt]
\text{③ 多次观测接力（上一轮后验当先验）：}\quad l_t(m) &= l_{t-1}(m) + \ln\frac{p(z_t\mid m)}{p(z_t\mid \neg m)} - \underbrace{\ln\frac{p(z_t)}{1-p(z_t)}}_{\text{归一化项}}
\end{aligned}
$$</div>
          <p>第③步末项是<b>归一化校正</b>；在我们的对称模型里它恒为 0，于是得到最终形式：</p>
          <div class="frag"><div class="math">$$
\begin{aligned}
\text{束端格：}\quad & \boxed{l \;\leftarrow\; l + l_{\text{hit}}}\quad (l_{\text{hit}} = +0.847)\\[3pt]
\text{束中格：}\quad & \boxed{l \;\leftarrow\; l + l_{\text{miss}}}\quad (l_{\text{miss}} = -0.847)
\end{aligned}
$$</div></div>
          <div class="frag"><p class="big">乘法下溢消失，证据对冲变成加减 —— 两个麻烦一次解决。</p></div>''',
          notes='推导只保留骨架 4 行；听众要能自己从①走到②。完整推导在 doc 21 §3。'),

        S('最终更新公式（整讲就这 2 行）', r'''
          <p>对一束激光路径上的每个格子：</p>
          <div class="math">$$
\begin{aligned}
\text{束端格：}\quad & l \leftarrow \mathrm{clamp}\big(l + l_{\text{occ}}\big), \qquad l_{\text{occ}} = +\ln\tfrac{0.7}{0.3} = +0.847\\[3pt]
\text{束中格：}\quad & l \leftarrow \mathrm{clamp}\big(l + l_{\text{free}}\big), \qquad l_{\text{free}} = -\ln\tfrac{0.7}{0.3} = -0.847
\end{aligned}
$$</div>
          <p class="muted">clamp：l ∈ [−4, +4]（防止绝对化，见工程页）</p>
          <p class="big">为什么这 2 行能撑起整个 2D SLAM 的建图？因为它同时做到了：</p>
          <li><b class="cy">增量</b>：来一帧算一帧，流式处理，不存历史</li>
          <li><b class="cy">对冲</b>：错误证据被后续观测抵消（下一页手算验证）</li>
          <li><b class="cy">可并行</b>：每格独立更新，GPU/多核友好</li>''',
          notes='这页是本讲的"一页总结"预习版；最后一页还会再收拢一次。'),

        S('手算例 ①：一束激光扫过 12 格世界', '''
          <p>1D 走廊 12 格，机器人在格 2 朝右打一束，回波在格 8（束长 6）。</p>
          <table>
            <tr><th>格子</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th><th>7</th><th>8</th><th>9~11</th></tr>
            <tr><td>角色</td><td>机器人</td><td colspan="5" style="text-align:center">束中（穿过）</td><td>束端</td><td>没扫到</td></tr>
            <tr><td>更新前 l</td><td>—</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr>
            <tr><td>票据</td><td>—</td><td>−0.847</td><td>−0.847</td><td>−0.847</td><td>−0.847</td><td>−0.847</td><td>+0.847</td><td>0</td></tr>
            <tr><td>更新后 l</td><td>—</td><td>−0.847</td><td>−0.847</td><td>−0.847</td><td>−0.847</td><td>−0.847</td><td><b class="ok">+0.847</b></td><td>0</td></tr>
            <tr><td>更新后 p</td><td>—</td><td>0.30</td><td>0.30</td><td>0.30</td><td>0.30</td><td>0.30</td><td><b class="ok">0.70</b></td><td>0.5</td></tr>
          </table>
          <p class="frag">一束之后：3~7 变"可能是空"（0.30），8 变"可能是墙"（0.70），9~11 仍未知 ——
          <b class="vi">和肉眼看激光点的感觉完全一致</b>。</p>''',
          notes='拿纸笔跟着抄一遍表格，数字要自己算：σ(0.847)=0.7。'),

        S('手算例 ②：同一面墙连测三次（确定性从哪来）', '''
          <p>机器人没动，对格 8 连续打三束（每束都命中格 8）。</p>
          <table>
            <tr><th>格子 8</th><th>第 1 束</th><th>第 2 束</th><th>第 3 束</th></tr>
            <tr><td>更新前 l</td><td>0</td><td>+0.847</td><td>+1.694</td></tr>
            <tr><td>+ l_occ</td><td>+0.847</td><td>+0.847</td><td>+0.847</td></tr>
            <tr><td>更新后 l</td><td>0.847</td><td>1.694</td><td><b class="ok">2.541</b></td></tr>
            <tr><td>p = σ(l)</td><td>0.700</td><td>0.845</td><td><b class="ok">0.927</b></td></tr>
          </table>
          <p class="frag"><p style="margin-top:8px">看得见的规律：<b class="vi">每票加 0.847，确定性单调上升但增速递减</b>
          （0.70 → 0.845 → 0.927 → …）—— sigmoid 天生的"越接近 1 越难推动"。
          这就是贝叶斯的谦虚性：有限证据永远到不了 100%。</p></p>''',
          notes='强调 sigmoid 增速递减：后面第 4 票只把 0.927→0.963。'),

        S('手算例 ③：噪声票被对冲（本讲最重要的一幕）', '''
          <p>第 4 束是坏数据 —— 假回波在格 5（束长 3），给格 5 投了一张占据票：</p>
          <table>
            <tr><th>格子 5</th><th>事件</th><th>l 变化</th><th>累计 l</th><th>p</th></tr>
            <tr><td rowspan="3">格 5</td><td>第 1 束（穿过）</td><td>−0.847</td><td>−0.847</td><td>0.30</td></tr>
            <tr><td>第 4 束（噪声回波）</td><td>+0.847</td><td><b class="wa">0</b></td><td><b class="wa">0.50</b></td></tr>
            <tr><td>第 5 束（又穿过）</td><td>−0.847</td><td><b class="ok">−0.847</b></td><td><b class="ok">0.30</b></td></tr>
          </table>
          <p class="big">一张噪声票只是一次加减法，会被下一次正常观测<b class="ok">完全对冲</b>。</p>
          <div class="frag"><p>推论：动态物体（走过的人）投的票会随时间被反复穿过对冲 ——
          <b class="vi">这就是栅格地图"人走过去了，墙没留下"的数学原因</b>。</p></div>''',
          notes='这一页是 log-odds 设计的价值证明：对冲在乘法域很难这么干净。'),

        demo('og', '交互实验：亲手投票', sub='点"打一束"逐次更新；先正常三束，再制造一束噪声',
             watch=['每点一次"打一束"，读出条里 l 与 p 的逐格变化 —— 对照刚才手算的两张表',
                    '造一束噪声（改束长）后，格 5 的票被后续自由票对冲回灰色',
                    '阈值线：p 越过 0.65 才变黑 —— 投票未达标时只是"嫌疑墙"'],
             notes='让听众点 5~8 次，跟读数。这就是终端版 slam_playground --demo 1 的图形版。'),

        S('工程实现：一条激光束怎么"走"过格子', '''
          <p>更新哪些格子？—— 沿光线路径逐格走一遍，走法是<b class="cy">Bresenham 直线光栅化</b>（图形学画线算法）：</p>
          <div class="formula">起点：机器人格子 (c0, r0)
终点：回波点格子 (ce, re)   （超出最大射程 → 只走 free，不投占据票）
while 未到终点：
    投 l_free 票给当前格
    按 Bresenham 判据步进（x 或 y 前进一格，选择更贴近直线者）
终点格：投 l_hit 票</div>
          <li><b class="cy">步数 = 距离/分辨率</b>：5 m 束在 5 cm 格下走 100 步 —— 每帧几百束 × 百步 = 每帧数万次格子更新，这就是建图的 CPU 主开销</li>
          <li><b class="bad">斜线锯齿</b>：Bresenham 只走"主轴步进"，对角穿越会漏掉两个相邻格（击穿细物体）；
          严格版叫 <b>supercover</b>，会多走相邻格</li>''',
          notes='提醒：回波超过最大射程时不能投占据票——那是空气的尽头，不是墙。'),

        S('工程参数：四个旋钮的后果表', '''
          <table>
            <tr><th>参数</th><th>典型值</th><th>调大了</th><th>调小了</th></tr>
            <tr><td>l_occ（占据票强度）</td><td>+0.85（p=0.7）</td><td>1 帧就成墙：噪声直接变鬼影</td><td>墙要扫很多遍才出现，动态场景更稳</td></tr>
            <tr><td>l_free（自由票强度）</td><td>−0.85（p=0.3）</td><td>空白区更干净但物体刚出现时难"长出"</td><td>自由区信心弱，规划器过度保守</td></tr>
            <tr><td>clamp 上限</td><td>±4.0（p=0.982）</td><td>接近绝对化：真动态物体永远擦不掉</td><td>墙太"软"，遮挡后易褪色</td></tr>
            <tr><td>最大射程</td><td>传感器标称 × 0.8</td><td>远处噪声回波投占据票 → 远处鬼影</td><td>建图范围变小（可接受）</td></tr>
          </table>
          <div class="note"><b>为什么 clamp 在 ±4</b>：p=0.982 意味着"至少 2% 的怀疑余地"——
          一旦真的错了（比如玻璃门被当墙），10 张自由票（−8.47）就能把墙彻底翻案。
          若 l 累到 +10，同样的翻案需要 100 张票 —— <b class="bad">绝对化 = 不可恢复</b>。</div>''',
          notes='clamp 是工程上最重要的一个旋钮：它买的是"可恢复性"。'),

        S('工程陷阱清单（建图翻车 80% 在这里）', '''
          <li>❌ <b class="bad">TF 时间戳错位</b>：激光帧用旧位姿变换 → 整面墙斜掉/重影。解法：message_filters 精确同步，TF 树里 map→odom 只由定位模块发布</li>
          <li>❌ <b class="bad">坐标系反转</b>：栅格 (行, 列) 与 (x, y) 搞混，或 PGM 图像 y 轴向下 → 地图上下颠倒。PGM 的第 0 行对应地图<b>最高 y</b></li>
          <li>❌ <b class="bad">忘记清除语义</b>：只投占据不投自由 → 自由空间永远灰色，规划器无路可走</li>
          <li>❌ <b class="bad">激光与里程计不同步</b>：转弯快时每帧激光跨越 5° 以上，需按时间在束内插值位姿（否则转弯处墙变"扇形"）</li>
          <li>❌ <b class="bad">原地旋转超速</b>：>90°/s 时相邻束错位超过 1 格，建出来全是斜纹</li>''',
          notes='每条都对应 doc 22 里的真实案例；听众踩过一次就懂了。'),

        qa('同一面墙连续测 10 次，l = 10 × 0.847 = 8.47（p ≈ 0.9998）。工程上为什么要 clamp 在 ±4？',
           ['纯粹为了省内存，float 存不下大数',
            '避免绝对化：保留可恢复性，错误证据（玻璃门、临时纸箱）仍能被后续观测翻案',
            '因为 sigmoid 在 l&gt;4 后数值不稳定会溢出',
            '为了兼容 ROS 地图消息的 int8 编码'],
           1,
           'p=0.982 vs 0.9998：后者意味着要 100 张反向票才能翻案，前者只要 10 张。绝对化的信念 = 永久鬼影。',
           notes='正确答案强调"可恢复性"这个工程价值观。'),

        qa('机器人朝玻璃窗打激光：一半束穿透（自由票），一半束镜面反射回别处（错位占据票）。长期后果是？',
           ['玻璃窗处的格子概率稳定在 0.5 附近（票据大致对冲）',
            '玻璃窗处会形成一道确定的墙（占据票永远赢）',
            '地图完全不能用了',
            '玻璃位置会变成自由区，永远不会有墙'],
           0,
           '穿透束持续投自由票，镜面反射票投到别处（那里的墙变厚一点）。玻璃处的票据长期对冲 → p 停在 0.5 灰色地带。这就是激光对玻璃"半透明"的建图表现。',
           notes='顺带讲工程对策：贴带纹理的胶带/防撞条，让玻璃可被雷达看到。'),

        S('一页总结：占据栅格建图',
          '''<div class="formula" style="font-size:16px">输入：位姿 + 激光束
  对每束激光沿 Bresenham 路径逐格：
    束中格  l += −0.847      （自由票）
    束端格  l += +0.847      （占据票）    clamp ±4
    p = σ(l)  →  p&gt;0.65 黑（墙） / p&lt;0.35 白（空） / 其余灰（未知）
输出：/map 占据栅格</div>
          <p class="big">记三个词就够了：<b class="cy">投票</b>（±0.847 一票）· <b class="vi">对冲</b>（噪声/动态物体可翻案）· <b class="pk">clamp</b>（保留可恢复性）</p>
          <div class="note">下一讲预告：本讲一直假设"位姿已知"—— 位姿本身怎么在噪声中估计？下一讲：贝叶斯滤波。</div>''',
          cls='cover'),
    ],
}
