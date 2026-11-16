#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 ③ 讲 · EKF-SLAM（高斯世界的同时估计）"""
from framework import S, demo, qa

DECK = {
    'file': 'ekf.html',
    'title': '③ EKF-SLAM',
    'chapter': '第 ③ 讲 · EKF-SLAM',
    'slides': [
        S('这一讲回答：把信念压成一个高斯，SLAM 怎么做？',
          '''<p class="big">贝叶斯滤波告诉我们"搬运+打折"两步循环。EKF 是它的<b class="cy">高斯坦白版</b>：
             信念只用两个数概括（均值 + 协方差），换来<b class="vi">解析公式、算得飞快</b>。</p>
             <p>本讲用最小的 1D 例子（机器人 + 1 个路标）把预测/更新/增益/协方差收缩<b class="ok">每一步手算出来</b>，
             再看它为什么能同时估计"我在哪"和"路标在哪"。</p>''',
          cls='cover'),

        S('高斯三分钟：一个均值，一个"胖瘦"', r'''
          <p>一维高斯：</p>
          <div class="math">$$
N(x;\,\mu,\sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}}\;\exp\!\Big(-\frac{(x-\mu)^2}{2\sigma^2}\Big)
$$</div>
          <p class="muted">μ = 最优猜测；σ² = 对猜测的不确定程度。
  二维 (x, y)：等高线是一圈一圈椭圆 —— μ 为椭圆中心，
  协方差矩阵 Σ = [[σx², σxy], [σxy, σy²]] 决定椭圆的形状和朝向。</p>
          <p class="big">贝叶斯滤波的两种操作在高斯世界里变成<b class="cy">纯代数</b>：</p>
          <li>预测：均值沿运动模型前进，协方差<b class="wa">膨胀</b>（+Q）</li>
          <li>更新：均值被拉向观测，协方差<b class="ok">收缩</b>（−K·H·Σ）</li>''',
          notes='让听众记住两种操作对应的协方差方向：预测变胖、更新变瘦。'),

        S('为什么偏偏选高斯？（工程账）', '''
          <table>
            <tr><th>性质</th><th>带来的好处</th></tr>
            <tr><td>高斯 ⊕ 高斯仍是高斯（卷积封闭）</td><td>预测步有<b>解析解</b>：μ+u，Σ+Q，不用积分</td></tr>
            <tr><td>两个高斯相乘仍是高斯</td><td>更新步有<b>解析解</b>：卡尔曼 1960 年的公式</td></tr>
            <tr><td>只需存 μ 和 Σ（n 维 = n+n² 个数）</td><td>内存与历史长度<b>无关</b>（递归性保住了）</td></tr>
            <tr><td>边缘分布/条件分布都是高斯</td><td>"知道 a 能推 b" 可以直接用公式（本讲灵魂）</td></tr>
          </table>
          <div class="warnbox">代价：高斯只有<b class="bad">一个峰</b>。走廊两扇门的例子（第②讲 0.444/0.444 双峰）——
          高斯只能把均值摆在两门中间、方差拉大，<b>两头不靠</b>。这是后面第④讲粒子滤波登场的理由。</div>''',
          notes='先讲清"封闭性=便宜"，再立刻自曝单峰缺陷，为 PF 埋钩。'),

        S('SLAM 的联合状态：把世界也装进高斯里', r'''
          <div class="math">$$
x = \begin{bmatrix}\text{机器人位姿} \\ m_1 \\ m_2 \\ \vdots \\ m_n\end{bmatrix}
\;=\;
\begin{bmatrix}x_r \\ m_1 \\ m_2 \\ \vdots \\ m_n\end{bmatrix}
\qquad\text{（2D：每项 3 维，此处先讲 1D）}
$$</div>
          <div class="math">$$
\Sigma = \begin{bmatrix}
\sigma_{rr} & \sigma_{r1} & \sigma_{r2} & \cdots \\
\sigma_{1r} & \sigma_{11} & \sigma_{12} & \cdots \\
\sigma_{2r} & \sigma_{21} & \sigma_{22} & \cdots \\
\vdots & \vdots & \vdots & \ddots
\end{bmatrix}
$$</div>
          <p>关键一步：路标位置不再是"地图上固定的真值"，而是<b class="cy">和机器人位姿一起被估计的随机变量</b>。
          SLAM 的"S"（同时）就体现在这一个联合向量里。</p>''',
          notes='这一页定义了"同时"：机器人和路标进同一个高斯。'),

        S('预测步：机器人动了，世界跟着"变胖"', r'''
          <p class="muted">运动模型 x′ = f(x, u)（u = 控制/里程计增量）；噪声 w ~ N(0, Q)（里程计不完美）。预测公式：</p>
          <div class="math">$$
\begin{aligned}
\text{均值前进：}\quad & \bar\mu = f(\mu, u)\\[3pt]
\text{协方差膨胀：}\quad & \bar\Sigma = F\,\Sigma\,F^{\mathsf T} + Q,
\qquad F = \left.\frac{\partial f}{\partial x}\right|_{\mu}\;\text{（雅可比：非线性模型的"局部直线化"）}
\end{aligned}
$$</div>
          <p>两个直觉：</p>
          <li><b class="cy">+Q</b>：动一步就多一份运动噪声 —— 不确定性只增不减</li>
          <li><b class="vi">F·Σ·Fᵀ</b>：如果运动模型会放大误差（如转弯），雅可比负责把这个放大算进去</li>''',
          notes='强调雅可比=局部线性化，EKF 的 E 就是它，也是误差之源。'),

        S('手算 · 预测：1D 机器人 + 1 个路标', r'''
          <p>设定：机器人从 1.0 出发（不确定 σ²=1.5），路标在 3.0（σ²=0.5），彼此独立。</p>
          <div class="math">$$
\mu = \begin{bmatrix}1.0\\3.0\end{bmatrix},\quad
\Sigma = \begin{bmatrix}1.5 & 0\\0 & 0.5\end{bmatrix},\quad
u = 1,\quad
Q = \begin{bmatrix}0.5 & 0\\0 & 0\end{bmatrix}\;\text{（只有机器人有运动噪声）}
$$</div>
          <div class="frag"><div class="math">$$
\bar\mu = \begin{bmatrix}1.0+1.0\\3.0\end{bmatrix} = \begin{bmatrix}\mathbf{2.0}\\3.0\end{bmatrix}
\qquad
\bar\Sigma = \Sigma + Q = \begin{bmatrix}1.5+0.5 & 0\\0 & 0.5\end{bmatrix} = \begin{bmatrix}\mathbf{2.0} & 0\\0 & 0.5\end{bmatrix}
$$</div></div>
          <div class="frag"><p class="big">检查两个方向：机器人方差 1.5 → 2.0（<b class="wa">变胖 ✓</b>）；
             路标方差 0.5 不变（它没动，<b class="ok">✓</b>）。
             注意交叉项还是 0 —— 还没看到彼此，互不相干。</p></div>''',
          notes='手算页节奏：先自己算，再对照下一个小节的对答案。'),

        S('更新步：观测到来，"信谁多一点"怎么算', r'''
          <p class="muted">观测模型 z = h(x) + v，v ~ N(0, R)。本例：z = 路标 − 机器人 = 1.2（测距，观测噪声 R = 0.25²），
  H = ∂h/∂x = [−1, +1]（对机器人取负号，对路标取正号）。更新公式：</p>
          <div class="math">$$
\begin{aligned}
\text{残差：}\quad & y = z - h(\bar\mu)\\[3pt]
\text{增益：}\quad & K = \frac{\bar\Sigma\,H^{\mathsf T}}{H\,\bar\Sigma\,H^{\mathsf T} + R}
\qquad\text{← 本讲灵魂，下一页拆解}\\[3pt]
\text{均值：}\quad & \mu = \bar\mu + K\,y\\[3pt]
\text{协方差：}\quad & \Sigma = (I - K H)\,\bar\Sigma \qquad\text{← 收缩}
\end{aligned}
$$</div>
          <p>K 在 0 与 1 之间：<b>K→1 全信观测，K→0 全信预测</b>。分母里 R 大（传感器烂）→ K 小。</p>''',
          notes='让听众先看 K 的结构再算数。'),

        S('手算 · 更新：四个数字一步一步', r'''
          <div class="math">$$
\begin{aligned}
\text{① 预测观测：}\;\; & h(\bar\mu) = 3.0 - 2.0 = 1.0, \qquad y = 1.2 - 1.0 = \mathbf{0.2}\\[2pt]
\text{② 观测方差：}\;\; & S = H\bar\Sigma H^{\mathsf T} + R = (2.0+0.5) + 0.25 = \mathbf{2.75}\\[2pt]
\text{③ 增益：}\;\; & K = \frac{\bar\Sigma H^{\mathsf T}}{S} = \frac{1}{2.75}\begin{bmatrix}-2.0\\0.5\end{bmatrix} = \begin{bmatrix}\mathbf{-0.727}\\+0.182\end{bmatrix}\\[2pt]
\text{④ 均值修正：}\;\; & \mu = \begin{bmatrix}2.0\\3.0\end{bmatrix} + 0.2\,K = \begin{bmatrix}\mathbf{1.855}\\\mathbf{3.036}\end{bmatrix}\\[2pt]
\text{⑤ 协方差：}\;\; & \Sigma = \bar\Sigma - KH\bar\Sigma = \begin{bmatrix}\mathbf{0.546} & \mathbf{0.364}\\0.364 & \mathbf{0.591}\end{bmatrix}
\end{aligned}
$$</div>
          <div class="frag"><p class="big">逐项解读 K = [−0.727, +0.182]：</p>
            <li>残差 0.2 m 说明"预测的间距比测的短"—— 主因是机器人位置不确定大（σ²=2.0），<b class="vi">所以主要修机器人</b>（|−0.727| 大）</li>
            <li>路标也被顺带修正 +0.036（K=0.182 × 0.2）—— 它的误差被"传染"了一部分</li>
            <li>机器人方差 2.0 → 0.546：<b class="ok">一次观测砍掉 73% 的不确定性</b></li></div>''',
          notes='K 的不对称来自两个 σ² 的悬殊（2.0 vs 0.5），这就是"信谁多一点"的数字版。'),

        demo('ekf', '交互实验：看协方差收缩与关联诞生', sub='拖动噪声滑块，观察 Σ 的数字与图形',
             watch=['预测后机器人方差膨胀（2.0），更新后收缩（0.546）—— 对照手算',
                    '交叉协方差从 0 变成 0.364 ≠ 0 —— 关联的诞生',
                    '调大观测噪声 R：K 变小，修正变弱 —— "传感器烂就少信它"'],
             notes='让听众把 R 拉到最大再看 K，验证"信谁"逻辑。'),

        S('交叉协方差 ≠ 0：SLAM 的灵魂在这一格', r'''
          <div class="math">$$
\bar\Sigma = \begin{bmatrix}2.0 & 0\\0 & 0.5\end{bmatrix}
\;\xrightarrow{\;\text{更新}\;}\;
\Sigma = \begin{bmatrix}0.546 & \mathbf{0.364}\\ \mathbf{0.364} & 0.591\end{bmatrix}
\qquad \sigma_{r1} = 0.364 \neq 0
$$</div>
          <p class="big">这个数字的意思：<b class="cy">"知道机器人在哪，就能推路标在哪；反之亦然。"</b></p>
          <li>机器人往左偏（估计偏小）→ 路标估计必然往右偏（间距守恒于观测）—— 误差不再独立，而是<b class="vi">锁死成对</b></li>
          <li>第 2、3…个路标被观测后，全体路标与机器人、路标与路标之间都会建立关联 ——
              <b>整个地图变成一张"不确定性关系网"</b></li>
          <li>回环之所以能拉直整条轨迹（第⑥讲），本质就是这张网把"起点误差"传给了"终点观测"，二者互证</li>''',
          notes='关联=SLAM 区别于"定位+建图各自做"的唯一本质。'),

        S('复杂度账单：关联网络的代价', '''
          <table>
            <tr><th>路标数 n</th><th>Σ 的元素数 (n+1)²</th><th>每步更新开销</th><th>可行吗</th></tr>
            <tr><td>10</td><td>121</td><td>微秒级</td><td>✓ 教学场景</td></tr>
            <tr><td>100</td><td>10 201</td><td>毫秒级</td><td>✓ 小仓库</td></tr>
            <tr><td>1 000</td><td>≈ 1 000 000</td><td>十毫秒级</td><td>⚠ 勉强</td></tr>
            <tr><td>10 000（激光特征）</td><td>≈ 1 亿</td><td>秒级</td><td>✗ 爆炸</td></tr>
          </table>
          <p class="big">EKF-SLAM 的死因：<b class="bad">每观测一个新特征，整张 Σ 都要动 —— O(n²) 不可扩展</b>。</p>
          <div class="note">所以工程史上 EKF-SLAM 逐渐退出建图主战场，转型去了两件事：
          <b class="vi">机器人状态融合</b>（robot_localization 把 IMU+轮速+视觉融合成 /odom，就是 EKF/UKF）和
          <b class="vi">小规模特征 SLAM</b>。第④讲粒子滤波 + 栅格地图接管了大规模建图。</div>''',
          notes='死因页要讲透：不是错了，是贵了。'),

        S('EKF 的第二个软肋：一致性（过度自信）', '''
          <p>EKF 把非线性函数在 μ 处<b>局部直线化</b>（雅可比 F、H）。问题：</p>
          <div class="formula">真实：  x′ = f(x, u)      （弯曲的运动轨迹）
EKF：   x′ ≈ F·x + …      （切线近似）
        → 切线只在一小块邻域内准；步子迈大（大转弯/远观测）就系统性偏差</div>
          <p>后果叫<b class="cy">不一致（inconsistency）</b>：滤波器报告的协方差比真实误差<b class="bad">小</b> ——
          它<b>自以为很准</b>，实际早就偏了。表现为：</p>
          <li>估计协方差椭圆越跑越小，但真值误差越跑越大</li>
          <li>一旦"过度自信"，后续观测再难纠正（K 已经很小了）</li>
          <div class="frag"><div class="warnbox">这就是 EKF-SLAM 一致性问题（Julier & Uhlmann 的著名批评）。
          对策：UKF（无迹变换采点代替切线）、First-Estimates Jacobian（FEJ）、可观测性约束 —— 都是在补这个洞。</div></div>''',
          notes='不一致=越自信越错的悖论，是比复杂度更隐蔽的死因。'),

        S('UKF 一瞥：不切切线，改用"探子"', '''
          <div class="formula">EKF：  在 μ 处取一条切线（雅可比）     → 线性化误差
UKF：  按确定性规则撒 2n+1 个 σ-点    → 每个点真的穿过非线性 f/h
       用穿过后的均值/协方差代替切线  → 精度到二阶泰勒</div>
          <table>
            <tr><th></th><th>EKF</th><th>UKF</th></tr>
            <tr><td>需要雅可比？</td><td>要（求导易错）</td><td><b class="ok">不要</b>（黑盒即可）</td></tr>
            <tr><td>强非线性</td><td>漂</td><td class="ok">稳</td></tr>
            <tr><td>计算量</td><td>低</td><td>中（2n+1 倍函数调用）</td></tr>
            <tr><td>工程出现地</td><td>robot_localization（默认 ekf 模式）</td><td>同包可选 ukf 模式</td></tr>
          </table>
          <p>ROS2 的 <span class="mono">robot_localization</span> 两种都实现了 —— 标定差/强机动场景值得切 ukf 试试。</p>''',
          notes='UKF 是 EKF 的直接升级，工程上"换模式"成本极低。'),

        S('EKF 在今天的真实岗位', '''
          <p>EKF-SLAM（建图）已退役，但<b class="vi">EKF 本身到处都在</b>：</p>
          <table>
            <tr><th>岗位</th><th>在做什么</th><th>包/项目</th></tr>
            <tr><td>里程计融合</td><td>轮速 + IMU 融合成平滑 /odom</td><td>robot_localization（nav2 标配）</td></tr>
            <tr><td>IMU 预积分</td><td>VIO/激光惯导里把 IMU 积分成约束</td><td>LIO-SAM、VINS</td></tr>
            <tr><td>飞机姿态</td><td>加速度计+陀螺仪融合姿态角</td><td>飞控标配</td></tr>
            <tr><td>目标跟踪</td><td>雷达点迹→航迹</td><td>自动驾驶感知</td></tr>
          </table>
          <div class="note">判断口诀：<b class="ok">状态少（≤20 维）、需要在线高频、可以接受高斯假设 → EKF/UKF 仍是性价比之王</b>。
          大规模建图交给第④⑥讲的粒子滤波+图优化。</div>''',
          notes='给 EKF 一个公道的定位：退役的是"EKF-SLAM"，不是 EKF。'),

        qa('手算例里 K = [−0.727, +0.182]。为什么修正机器人的力度是路标的 4 倍？',
           ['观测直接测的是路标，理应多信路标',
            '机器人预测方差 (2.0) 远大于路标 (0.5)：谁更不确定就多修谁',
            '残差为正，所以修机器人',
            '雅可比符号决定的'],
           1,
           'K ∝ Σ·Hᵀ：修正量分配与各自方差成正比。2.0 vs 0.5 → 4 倍。这就是"信谁多一点"的定量答案。',
           notes='追问：若把 R 调大 4 倍，K 变多少？（整体除以~4，比例关系不变）'),

        qa('EKF-SLAM 里第 1000 个路标入库时，计算量主要花在哪？',
           ['把第 1000 个路标加进 μ 向量',
            '扩展 Σ 矩阵并与已有 999 个路标全部建立交叉协方差（O(n²) 增广）',
            '重新计算全部观测',
            '机器人位姿的更新'],
           1,
           '新特征入库 = Σ 增广，要求与全体旧状态建立关联项 —— 这正是 O(n²) 死因的来源。',
           notes='接下一题的伏笔：PF 家族怎么绕开这个问题。'),

        S('一页总结：EKF-SLAM',
          r'''<p class="muted">状态：x = [位姿, 路标…]（全体协方差 Σ = 关联网络）</p>
          <div class="math">$$
\begin{aligned}
\text{预测（变胖）：}\quad & \bar\mu = f(\mu, u), \qquad \bar\Sigma = F\Sigma F^{\mathsf T} + Q\\[4pt]
\text{更新（变瘦）：}\quad & K = \frac{\bar\Sigma H^{\mathsf T}}{H\bar\Sigma H^{\mathsf T} + R}, \qquad
\mu = \bar\mu + K\big(z - h(\bar\mu)\big), \qquad
\Sigma = (I - KH)\,\bar\Sigma
\end{aligned}
$$</div>
          <p class="big">带走三句话：<b class="cy">高斯=便宜但单峰</b> · <b class="vi">交叉协方差=SLAM 的灵魂</b> · <b class="pk">O(n²)+不一致=退役原因</b></p>
          <div class="note">下一讲：放弃"解析解"，改用 N 个假设硬扛多峰 —— 粒子滤波。走廊两扇门终于能表达了。</div>''',
          cls='cover'),
    ],
}
