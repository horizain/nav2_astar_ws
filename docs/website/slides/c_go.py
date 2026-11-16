#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 ⑥ 讲 · 图优化回环（全局拉直漂移）"""
from framework import S, demo, qa

DECK = {
    'file': 'go.html',
    'title': '⑥ 图优化回环',
    'chapter': '第 ⑥ 讲 · 图优化回环',
    'slides': [
        S('这一讲回答：累积漂移怎么被一次性全局修正？',
          '''<p class="big">前五讲都是<b class="cy">局部方法</b>：逐帧、逐格、逐粒子 —— 误差只减不消。
             里程计积分到 100 m 后差 2 m，扫描匹配再准也只能"修准每一步"，<b class="vi">救不了账面总差</b>。</p>
             <p>图优化的答案：把整条轨迹当<b class="pk">一张弹簧网</b>整体求解 —— 回环闭合的那一刻，全轨迹一起被拉直。</p>''',
          cls='cover'),

        S('局部方法的宿命：误差只增不减', '''
          <div class="formula">走一步差 1 cm（扫描匹配的正常精度）
走 100 步 ──> 最坏差 100 cm        ← 误差是随机游走：√100 = 10 cm（好情况）
绕场回到起点，账面上却差了 0.8 m   ← 地图上出现两堵"错位的墙"</div>
          <p>三个特征说明这是<b>系统性</b>问题，不是调参能解决的：</p>
          <li>误差与<b class="cy">轨迹长度</b>成正相关（随机游走）</li>
          <li>误差在<b class="cy">起点与终点重合处</b>暴露最狠（回环处错位最大）</li>
          <li>局部方法<b class="cy">原理上</b>无解：每一步的"局部最优"不含全局信息</li>''',
          notes='随机游走 √t 的直觉：误差增长比线性慢，但单调不减。'),

        S('大局观：位姿图 = 弹簧网', '''
          <div class="formula">节点 = 每个关键时刻的机器人位姿  x₁ ── x₂ ── x₃ ── … ── x_n
里程计边（软弹簧）：相邻节点间的相对位移观测
回环边（硬弹簧）：  "现在的我 ≈ 很久以前的我" 的相对位移观测

┌──里程计边──┐┌─里程计边─┐
x₁ ══════ x₂ ══════ x₃
└────── 回环边（虚线弹簧，拉力大）──────┘</div>
          <p>优化 = 调整每个节点的位置，让<b class="cy">所有弹簧同时最接近自然长度</b>：
          谁的弹簧被拉得越离谱，谁的节点被拉回得越多。</p>
          <div class="note">回环边是"以现在纠正过去"的唯一通道 —— 没有它，弹簧网是一串链条，怎么摆都一样松。</div>''',
          notes='弹簧网比喻贯穿全讲；下面全部数学都是"弹簧"的代数化身。'),

        S('数学化：节点、边、信息矩阵', r'''
          <div class="math">$$
\begin{aligned}
\text{节点：}\;\; & x_i = (x,\,y,\,\theta)^{\mathsf T} \qquad\text{位姿，未知，待优化}\\[3pt]
\text{边：}\;\; & z_{ij} = \text{两节点间的相对位移观测（里程计边 } i{=}j{+}1\text{；回环边 } i,j \text{ 相距很远）}\\[3pt]
\text{信息矩阵：}\;\; & \Omega_{ij} = \Sigma_{ij}^{-1} \qquad\text{← 权重 = 1/方差² 的矩阵版}
\end{aligned}
$$</div>
          <p class="muted">一条边贡献的代价（残差的马氏距离平方）：</p>
          <div class="math">$$
e_{ij}(x) = f(x_i, x_j) - z_{ij}
\qquad\Longrightarrow\qquad
\text{cost}_{ij} = e_{ij}^{\mathsf T}\,\Omega_{ij}\,e_{ij}
\qquad\text{（} \sigma \text{ 越小} \rightarrow \Omega \text{ 越大} \rightarrow \text{这条边越"硬"}\text{）}
$$</div>
          <p class="big">全图目标函数：<b class="cy">J(x) = Σ eᵀΩe —— 加权最小二乘</b>。
          "软弹簧拉太长很疼，硬弹簧拉长更疼" —— 疼 = J。</p>''',
          notes='信息矩阵=每条边的"硬度"。σ 是建图模块输出的边可信度。'),

        S('为什么权重是 1/σ²（信息矩阵直觉）', r'''
          <p>两个测量同一物理量：里程计链说 x₃=2.0（σ=0.14），回环说 x₃=1.8（σ=0.2）。听谁的？</p>
          <div class="math">$$
\hat x \;=\; \frac{x_1/\sigma_1^2 \,+\, x_2/\sigma_2^2}{1/\sigma_1^2 \,+\, 1/\sigma_2^2}
\qquad\text{← 谁的 } \sigma^2 \text{ 小（更自信），票重}
$$</div>
          <table>
            <tr><th>σ</th><th>1/σ²（信息量）</th><th>话语权</th></tr>
            <tr><td>0.1（里程计单步）</td><td>100</td><td>硬</td></tr>
            <tr><td>0.2（回环匹配）</td><td>25</td><td>较硬</td></tr>
            <tr><td>1.0（瞎猜）</td><td>1</td><td>软（存在但不指挥）</td></tr>
          </table>
          <div class="note">这就是"调参=调信任分配"的含义：调 σ 不是调精度，是调<b class="cy">这条边在全局仲裁里的话语权</b>。</div>''',
          notes='σ→权重是后面手算的核心；提前把融合公式亮出来。'),

        S('手算 ①：三节点一维小世界，列方程', r'''
          <p>x₁ 锚定在 0（原点已知）。两条里程计边 + 一条回环边：</p>
          <table>
            <tr><th>边</th><th>方程</th><th>观测值</th><th>σ</th><th>权重 1/σ²</th></tr>
            <tr><td>里程计 x₁→x₂</td><td>x₂ − x₁</td><td>1.0</td><td>0.1</td><td>100</td></tr>
            <tr><td>里程计 x₂→x₃</td><td>x₃ − x₂</td><td>1.0</td><td>0.1</td><td>100</td></tr>
            <tr><td><b class="pk">回环 x₁→x₃</b></td><td>x₃ − x₁</td><td><b class="pk">1.8</b></td><td>0.2</td><td>25</td></tr>
          </table>
          <p>里程计积分说 x₃ = 2.0，回环匹配说 x₃ = 1.8 —— 漂了 0.2。<b class="vi">问：x₂、x₃ 的全局最优在哪？</b></p>
          <div class="frag"><div class="math">$$
J \;=\; 100\,(x_2-1)^2 \;+\; 100\,(x_3-x_2-1)^2 \;+\; 25\,(x_3-1.8)^2
\qquad\text{← 三根弹簧的总"疼"}
$$</div></div>''',
          notes='让听众先猜：x₃ 会落在 1.8~2.0 之间的哪里？偏向谁？'),

        S('手算 ②：正规方程求解（两个未知数）', r'''
          <div class="math">$$
\begin{aligned}
\frac{\partial J}{\partial x_2} &= 200(x_2-1) - 200(x_3-x_2-1) = 0 \;\;\Longrightarrow\;\; 2x_2 = x_3\\[3pt]
\frac{\partial J}{\partial x_3} &= 200(x_3-x_2-1) + 50(x_3-1.8) = 0
\end{aligned}
$$</div>
          <p class="muted">代入：250x₃ − 200x₂ = 290；x₂ = x₃/2 ⟹ 150x₃ = 290</p>
          <div class="frag"><div class="math">$$
\boxed{\;x_2 = 0.9667 \qquad x_3 = 1.9333\;}
$$</div></div>
          <div class="frag"><p class="big">矩阵视角（图优化的真实计算形态）：</p>
            <div class="math">$$
H = A^{\mathsf T} W A = \begin{bmatrix}200 & -100\\-100 & 125\end{bmatrix}
\qquad
g = A^{\mathsf T} W b = \begin{bmatrix}0\\145\end{bmatrix}
$$</div>
            <div class="math">$$
H\,x = g \;\;\Longrightarrow\;\; x = H^{-1}g = \begin{bmatrix}0.9667\\1.9333\end{bmatrix}
\qquad\text{（线性最小二乘一步到位）}
$$</div></div>''',
          notes='两个方法对照：求导消元 vs 正规方程——同一答案。'),

        S('手算 ③：读结果 —— 回环如何"惠及全链"', r'''
          <table>
            <tr><th>量</th><th>优化前（纯里程计）</th><th>优化后</th><th>变化</th></tr>
            <tr><td>x₂</td><td>1.0</td><td><b class="ok">0.9667</b></td><td>被拉回 0.033</td></tr>
            <tr><td>x₃</td><td>2.0</td><td><b class="ok">1.9333</b></td><td>被拉回 0.067</td></tr>
          </table>
          <p class="big">精度加权互验（口算就能验）：</p>
          <div class="math">$$
\text{里程计链：}\; x_3 = 2.0,\;\; \text{精度 } \frac{1}{0.1^2+0.1^2} = 50
\qquad
\text{回环边：}\; x_3 = 1.8,\;\; \text{精度 } \frac{1}{0.2^2} = 25
$$</div>
          <div class="math">$$
x_3 = \frac{2.0 \times 50 + 1.8 \times 25}{50 + 25} = \mathbf{1.9333}
\;\;\checkmark\;\text{与正规方程解完全一致}
$$</div>
          <div class="frag"><p>两个结论：</p>
            <li><b class="cy">一条回环边改善整条链</b>：x₂ 明明没有直接连回环，也被拉动了 —— 弹簧网是整体变形</li>
            <li><b class="vi">x₃ 偏向"更自信"的一方</b>：偏向里程计（精度 50 vs 25），停在三比二的位置 —— 不是简单平均</li></div>''',
          notes='互验页是本讲"aha 时刻"：复杂求解能用口算验证。'),

        demo('go', '交互实验：拖动 σ，看节点被弹簧拉回', sub='调 σ_loop / σ_odo，实时重解位姿图',
             watch=['σ_loop 从 0.2 拉大 → 回环越来越"软"，x₃ 从 1.8 一侧滑向 2.0 一侧',
                    'σ_loop 调小（更自信的回环）→ 全链向回环观测靠拢',
                    'x₂ 永远被 x₃ 带动 —— 一条回环边改善整条链（手算 ③ 结论）'],
             notes='先让听众预测再动手：σ_loop=0.5 时 x₃ 大概在哪？'),

        S('真实世界是非线性的：迭代线性化（Gauss-Newton）', r'''
          <p>2D 里边模型 e(x) = f(x_i, x_j) − z 是<b class="vi">非线性的</b>（含 cos/sin）。处理方式：迭代</p>
          <div class="math">$$
\begin{aligned}
\text{① 线性化：}\quad & e(x^{k}+\Delta) \;\approx\; e(x^{k}) + J\,\Delta\\[3pt]
\text{② 解增量方程：}\quad & (J^{\mathsf T}\Omega J)\,\Delta = -J^{\mathsf T}\Omega\, e \qquad\text{← 就是手算例的正规方程！}\\[3pt]
\text{③ 更新：}\quad & x^{k+1} = x^{k} + \Delta
\end{aligned}
$$</div>
          <p class="muted">④ 每次迭代都在解一个"线性版小世界" —— 手算例就是其中一次迭代。</p>
          <p>非线性版的差别只有两点：A 矩阵换成了<b class="cy">雅可比 J</b>；需要<b class="cy">迭代 3~10 次</b>收敛。
          思想没有变 —— "局部直线 + 加权最小二乘"。</p>''',
          notes='手算例=一次迭代的完整解剖。Gauss-Newton 只是把"直线"每轮重画。'),

        S('稀疏性：H 矩阵长什么样（为什么算得动）', '''
          <div class="formula">n=6 个节点、边 (1,2)(2,3)(3,4)(4,5)(5,6)(1,6回环)：

        c1  c2  c3  c4  c5  c6
r1  [ ██   █              ]      每条边只在
r2  [  █  ██   █           ]      对应两节点
r3  [      █  ██   █       ]      的行列块上有值
r4  [         █  ██   █    ]      其余 = 0
r5  [            █  ██  █  ]
r6  [ █                ███ ]      ← 回环边打破三对角带</div>
          <p class="big">千节点问题里 99% 的元素是 0。用<b class="cy">稀疏 Cholesky</b> 求解，实际算量 ∝ 有效元素数 —— 这就是千级位姿图毫秒级求解的原因。</p>
          <div class="note">回环边会"填矩阵"（fill-in）：回环越多 H 越密 —— 但几百条回环仍远好于稠密。</div>''',
          notes='稀疏性是图优化能在线跑的根本。'),

        S('求解器一览：g2o / Ceres / g2s/gtsam', '''
          <table>
            <tr><th>求解器</th><th>风格</th><th>特点</th><th>谁在用</th></tr>
            <tr><td><b class="cy">g2o</b></td><td>图优化专用</td><td>节点/边抽象直接对应本讲概念</td><td>ORB-SLAM、老版 slam_toolbox</td></tr>
            <tr><td><b class="vi">Ceres</b></td><td>通用非线性最小二乘</td><td>自动求导，工业级稳健</td><td>cartographer、新版 slam_toolbox</td></tr>
            <tr><td><b class="pk">gtsam</b></td><td>因子图 + 增量(Smoothing)</td><td>iSAM2 增量求解：新节点不重解全图</td><td>LIO-SAM、Kimera</td></tr>
          </table>
          <p>对用户透明：你在参数里选 <span class="mono">solver</span>，数学都是这一讲的 J = ΣeᵀΩe + 稀疏线性代数。</p>
          <p class="big">gtsam 的<b class="cy">增量式</b>值得记住：只更新受新边影响的部分 —— 大图实时优化的关键技术。</p>''',
          notes='三种求解器=同一数学的三种工程封装。'),

        S('回环检测：怎么发现"我回来了"？', '''
          <p>图优化只是"修正执行器"，还得先有人<b class="vi">喊出"回来了"</b>。三条路线：</p>
          <table>
            <tr><th>路线</th><th>做法</th><th>代表</th></tr>
            <tr><td><b class="cy">扫描匹配对地图</b></td><td>当前帧与"更早时刻的局部地图"匹配成功且距离远 → 回环</td><td>slam_toolbox 默认</td></tr>
            <tr><td><b class="vi">描述子检索</b></td><td>每帧算外观指纹（如 Scan Context），在指纹库里查相似历史帧</td><td>FAB-MAP、Scan Context</td></tr>
            <tr><td><b class="pk">分支定位 Branch-and-Bound</b></td><td>不依赖里程计先验，全局搜索任何可能的匹配 —— 连"我在哪都不知道"都能找到回环</td><td>cartographer</td></tr>
          </table>
          <div class="note">slam_toolbox 的门槛：<span class="mono">loop_search_space</span>（在预测位置附近多大范围找回环）+
          <span class="mono">loop_match_minimum_chain_size</span>（候选历史链最短长度，防噪声触发）。</div>''',
          notes='三路线覆盖从"带先验"到"无先验"的谱系。'),

        S('假回环与鲁棒核：别让一条脏边毁全图', r'''
          <p>回环匹配偶尔会<b class="bad">错认</b>：两段相似的走廊（对称环境！）被配成一对 —— 一条错误约束，权重还可能很高。</p>
          <div class="math">$$
J_{\text{普通最小二乘}} = \sum_i e_i^{\mathsf T}\Omega_i\, e_i
\qquad\qquad
J_{\text{鲁棒核}} = \sum_i \rho\big(e_i^{\mathsf T}\Omega_i\, e_i\big)
$$</div>
          <p class="muted">普通最小二乘：误差平方 —— 大误差被平方放大 → 一条脏边统治全图；鲁棒核 ρ 压扁大误差的贡献。
  Huber：超过阈值后线性增长（不再平方）；DCS/Cauchy：超过阈值后增长趋零（"你说错了？行，但我不听"）。</p>
          <p class="big">工程三保险：</p>
          <li><b class="cy">几何校验</b>：接受回环前检查与现有图的一致性（匹配协方差、多帧验证）</li>
          <li><b class="vi">鲁棒核</b>：优化时压脏边（gtsam/g2o 都内置）</li>
          <li><b class="pk">多假设</b>：保留 top-k 回环候选，由优化结果反选</li>''',
          notes='对称环境（两条相同走廊）是假回环重灾区——呼应第②讲对称问题。'),

        S('slam_toolbox 回环参数对照', '''
          <table>
            <tr><th>参数</th><th>默认</th><th>含义</th></tr>
            <tr><td><span class="mono">loop_search_space</span></td><td>3.0 m</td><td>在预测位置附近多大范围找回环</td></tr>
            <tr><td><span class="mono">loop_search_maximum_difference</span></td><td>10.0 m</td><td>链搜索位姿差上限</td></tr>
            <tr><td><span class="mono">loop_match_minimum_chain_size</span></td><td>10</td><td>历史链至少几个节点（防噪声触发）</td></tr>
            <tr><td><span class="mono">loop_match_maximum_variance_coarse</span></td><td>3.0</td><td>粗匹配方差上限（太差直接拒绝）</td></tr>
            <tr><td><span class="mono">minimum_loop_scan_period / travel</span></td><td>30 s / 5 m</td><td>两次回环尝试的最小间隔（防连发误检）</td></tr>
            <tr><td><span class="mono">scan_buffer_size</span></td><td>10</td><td>局部地图缓冲几帧（匹配用）</td></tr>
          </table>
          <div class="note">调参心法：<b class="cy">防误检靠 chain_size 与间隔，防漏检靠 search_space</b> —— 两类参数别搞混方向。</div>''',
          notes='给听众一张"回环失灵先查这"的表。'),

        qa('手算例里 x₃ 的全局解 1.9333 更偏向里程计链（2.0）而不是回环（1.8）。原因是什么？',
           ['里程计边有两条，回环只有一条，少数服从多数',
            '里程计链精度 1/(0.1²+0.1²)=50 高于回环 1/0.2²=25，信息加权偏向自信者',
            '代码 bug，正确值应为 1.9',
            'x₂ 的存在拉偏了结果'],
           1,
           '融合权重=1/σ²。两条 0.1 的边合成一条精度 50 的"等效边"，压过精度 25 的回环边。不是数边数，是算信息量。',
           notes='追问：若回环 σ=0.1，x₃ 落哪？（三条精度 100 的边：x₃=(2.0·100+1.8·100)/200=1.9）'),

        qa('某次回环匹配错误地把两条对称走廊配成一对，且 σ=0.05（权重巨大）。如果没有鲁棒核，后果是？',
           ['地图没什么变化，只是多了一条边',
            '这条高权重脏边在优化中统治全图，把正确结构硬掰变形',
            '只会让本次回环附近的节点变形',
            '优化会自动拒绝矛盾约束'],
           1,
           '最小二乘对大误差平方放大：误差大→代价平方级大→优化器宁可扭曲全图也要减小它。Huber/Cauchy 核 + 几何校验是标配防线。',
           notes='呼应 PF 页"错误对齐往地图写脏数据"：脏边比脏格子更难擦。'),

        S('一页总结：图优化回环',
          '''<div class="formula" style="font-size:16px">位姿图：节点=位姿，边=相对位移观测，Ω=1/σ²
目标：  J = Σ eᵀΩe → 最小     （加权最小二乘，Gauss-Newton 迭代）
手算例：H·x = g（稀疏）→ x₂=0.9667, x₃=1.9333 —— 一条回环边惠及全链
防线：  鲁棒核 + 几何校验（假回环）</div>
          <p class="big">带走三句话：<b class="cy">弹簧网=位姿图</b> · <b class="vi">权重=1/σ²=话语权</b> · <b class="pk">回环是"以现在纠正过去"的唯一通道</b></p>
          <div class="note">七讲到此完成闭环：建图（①）+定位（②③④）+匹配（⑤）+全局纠偏（⑥）。下一讲：全部装到真实机器人上 —— 工程实战。</div>''',
          cls='cover'),
    ],
}
