#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 ④ 讲 · 粒子滤波（多峰世界的硬核解法）"""
from framework import S, demo, qa

DECK = {
    'file': 'pf.html',
    'title': '④ 粒子滤波',
    'chapter': '第 ④ 讲 · 粒子滤波',
    'slides': [
        S('这一讲回答：双峰、非高斯的世界怎么定位？',
          '''<p class="big">第②讲走廊例子的遗留问题：信念是<b class="cy">双峰</b>的（两扇门都可能），高斯表达不了。
             第③讲 EKF 单峰破产。</p>
             <p>粒子滤波的答案朴实到惊人：<b class="vi">别算分布了，直接养 N 个"活样本"代表它</b>。</p>
             <p>这一讲跑通：加权 → 重采样 → 收敛的完整循环，最后看它怎么长成 Gmapping/AMCL 这两大工程支柱。</p>''',
          cls='cover'),

        S('核心思想：用 N 个假设代替一张概率表', '''
          <div class="formula">概率分布（解析）          粒子云（采样）
bel = {0.444, 0.111,     ≈   粒子1 @格1  权重 0.8
       0.444}                粒子2 @格3  权重 0.8
                             粒子3 @格2  权重 0.2
                             …（共 N 个）</div>
          <p>三个要点：</p>
          <li><b class="cy">粒子 = 一个完整的假设</b>："如果我真在位置 x，世界应该是这样" —— 粒子自带位姿（Gmapping 里还自带一张地图！）</li>
          <li><b class="vi">权重 = 这个假设的可信度</b>（来自观测的似然）</li>
          <li><b class="pk">粒子云的形状 = 信念分布的形状</b>：多峰？散开就是两个团。非高斯？随便什么形状都行</li>
          <div class="note">和直方图滤波比：格子法在所有格子上"全面开花"，粒子法只把计算花在<b>可能的地方</b> —— 高维时省得惊人。</div>''',
          notes='一句话钉死：粒子=带权重的假设；云的形状=信念的形状。'),

        S('算法循环：四步，永远循环', '''
          <div class="formula" style="font-size:16px">① <span class="hl">预测</span>   每个粒子按运动模型动一步（各自带噪声）
② <span class="hl">加权</span>   用观测给每个粒子打分 w_i = P(z | 粒子i的位置)
③ <span class="hl">重采样</span>  按权重抽签：高分粒子多生，低分粒子绝育（可选，按需触发）
④ <span class="hl">估计</span>   加权平均 = 当前位置估计 → 回到 ①</div>
          <table>
            <tr><th>对应贝叶斯滤波</th><th>粒子版实现</th></tr>
            <tr><td>预测（搬运）</td><td>每个粒子独立采样运动（蒙特卡洛积分）</td></tr>
            <tr><td>更新（打折）</td><td>乘似然作权重</td></tr>
            <tr><td>归一化</td><td>权重除以总和</td></tr>
          </table>
          <div class="note">重采样不是每帧必须：权重越悬殊越该抽。判断指标 Neff —— 后面专页讲。</div>''',
          notes='四步口诀：动、打分、抽签、读数。'),

        S('权重怎么算：高斯核打分', r'''
          <p>观测 z（如"距最近门 0.45 m"），粒子预测值 ẑᵢ（"按我的位置算应该是 0.4 m"）：</p>
          <div class="math">$$
w_i = \exp\!\Big(-\frac{(z - \hat z_i)^2}{2\sigma^2}\Big)
\qquad\text{（}\sigma = \text{传感器噪声标准差}\text{）}
$$</div>
          <div class="math">$$
\hat z_i = z \;\text{差}\; 0 \rightarrow w = 1\;\text{（完美吻合）}
\qquad \text{差}\;2\sigma \rightarrow w = 0.14\;\text{（边缘）}
\qquad \text{差}\;5\sigma \rightarrow w \approx 0\;\text{（死刑）}
$$</div>
          <p>本质：把"预测 − 实测"的距离换算成概率。σ 是唯一的旋钮 ——
          <b class="vi">σ 就是传感器的诚实度</b>：激光测距 σ≈0.02~0.05 m，所以粒子滤波里错 10 cm 的粒子直接判死。</p>
          <div class="warnbox">工程坑：σ 调太小 → 所有粒子权重同时≈0（全灭）；调太大 → 好坏粒子权重一样（失去分辨力）。要和真实噪声匹配。</div>''',
          notes='高斯核两个极限都讲：全灭与失明。'),

        S('重采样：轮盘赌怎么杀粒子', '''
          <div class="formula">权重 [0.5, 0.3, 0.15, 0.05] 归一化后做一条轮盘：

  |——— 粒子1 50% ———|— 粒子2 30% —|— 3 15% —|4|
  ↑ 转动 N 次指针，落在谁的扇区就复制谁</div>
          <p>规则：<b class="cy">概率 = 权重</b>。期望上，权重 0.5 的粒子被复制 N×0.5 次；权重 0.05 的粒子大概率一次都轮不到 —— 直接消失。</p>
          <li>复制体保持原位姿（可能加微小抖动 = 粗化）</li>
          <li>重采样后所有权重重置为 1/N（"新的一代重新起跑"）</li>
          <li><b class="wa">副作用</b>：低权重粒子里的"少数派正确答案"可能被一起杀掉 —— 多样性损失，本讲后段专治</li>''',
          notes='重采样=自然选择：繁殖成功率∝适应度。'),

        S('手算 · 5 粒子迷你版（走廊同世界）', '''
          <p>走廊 3 格，门在格 1、3（第②讲同款）。初始 5 个粒子：[1, 1, 2, 3, 3]，权重各 0.2。</p>
          <table>
            <tr><th>回合</th><th>粒子位置</th><th>权重（观测后）</th><th>重采样后</th></tr>
            <tr><td>观测"门"</td><td>[1,1,2,3,3]</td><td>[0.235, 0.235, <b class="bad">0.059</b>, 0.235, 0.235]<br><span class="mono">η=3.4</span></td><td>[1,1,3,3,1] <span class="mono">格2 绝育</span></td></tr>
            <tr><td>右移一格<br>(80%右/20%留)</td><td>[2,2,3,3,2]</td><td>—</td><td>—</td></tr>
            <tr><td>观测"门"</td><td>[2,2,3,3,2]</td><td>[<b class="bad">0.091</b>, 0.091, <b class="ok">0.364</b>, 0.364, 0.091]<br><span class="mono">η=2.2</span></td><td>[3,3,3,3,2] <span class="mono">格3 胜出</span></td></tr>
          </table>
          <p class="big">读法：格 2 的粒子每轮都是 0.2 的误报概率 → 权重最低 → 先绝育；
          格 1、3 双峰靠<b class="vi">运动+第二次观测</b>分胜负 —— 和第②讲的 0.745 收敛完全同源，只是这次用 5 个"活样本"表达。</p>''',
          notes='让听众自己算 η=3.4 和 2.2，以及格2 的 0.2/3.4=0.059。'),

        demo('pf', '交互实验：200 粒子从迷失到锁定', sub='逐步点"测距→移动→测距"，看粒子云形态',
             watch=['<b>初始</b>：粒子均匀撒满世界 —— "完全迷失"的样子',
                    '<b>第一次测距</b>：大部分粒子死亡，存活簇贴在真值附近',
                    '<b>移动+测距</b>：粒子云整体平移并继续收紧',
                    '<b>锁定后</b>：粒子标准差不再变小 —— 这是"收敛"的信号，继续跑不会更准'],
             notes='收敛后继续跑只是维持，不会更准——工程上"粒子云不再收缩"就是定位稳定标志。'),

        S('粒子数 N 怎么选：一张 trade-off 表', '''
          <table>
            <tr><th>N</th><th>CPU</th><th>表达力</th><th>典型用途</th></tr>
            <tr><td>几十</td><td>极低</td><td>只够单峰</td><td>收敛后的<b>跟踪</b>（AMCL 后期）</td></tr>
            <tr><td>几百~几千</td><td>低</td><td>2~3 个峰、中等噪声</td><td>室内 SLAM 常规（Gmapping 默认 30~50 粒子另算）</td></tr>
            <tr><td>上万</td><td>中</td><td>绑架恢复、大初始不确定</td><td>AMCL 初始撒点、重定位</td></tr>
            <tr><td>十万+</td><td>高</td><td>高维/超多峰</td><td>通常不值得 —— 该换方法了</td></tr>
          </table>
          <p class="big">工程共识：<b class="cy">前期多、后期少</b> —— AMCL 的 KLD 自适应采样就是干这个的（后页）。</p>
          <p>判断粒子够不够的土办法：把 N 减半再跑一遍，结果轨迹不变 → N 够了。</p>''',
          notes='土办法很实用：减半测试。'),

        S('粒子退化与 Neff：什么时候该重采样', r'''
          <div class="math">$$
N_{\text{eff}} = \frac{1}{\sum_i w_i^2}
$$</div>
          <p class="muted">权重均匀 [1/N × N] → N_eff = N（健康：别重采样，白抽）；
权重集中 [1, 0, 0, …] → N_eff = 1（退化：必须重采样）。</p>
          <p>退化是<b class="bad">必然趋势</b>：每轮观测都会拉开权重差距，几轮后权重全堆在少数粒子上。</p>
          <li>N_eff &lt; N/2 → 触发重采样（AMCL 默认策略）</li>
          <li>重采样过频 → 多样性损失（下一页）；过疏 → 拿一堆死粒子做平均 → 估计有偏</li>
          <div class="note">N_eff 就是"信念实际由几个粒子撑着"。它同时是输出质量的体检指标：
          锁定后 N_eff 低是正常的（单峰），但<b>探索期</b> N_eff 就很低 = 粒子太少的警报。</div>''',
          notes='Neff 一个公式两用：触发器+体检指标。'),

        S('重采样的三种抽法 + 多样性急救', '''
          <table>
            <tr><th>方法</th><th>做法</th><th>特点</th></tr>
            <tr><td>轮盘赌</td><td>独立抽 N 次</td><td>简单；方差最大（可能重复抽中同一个）</td></tr>
            <tr><td><b class="cy">系统采样</b> ★</td><td>一根指针 + N 个等距刻度，每刻度复制对应粒子</td><td>方差小、O(N)、工程默认</td></tr>
            <tr><td>残差采样</td><td>整数部分直接复制，小数部分轮盘</td><td>介于两者之间</td></tr>
          </table>
          <div class="frag"><p class="big" style="margin-top:10px">急救多样性两招：</p>
            <li><b class="vi">粗化 roughening</b>：重采样后给每个粒子加微小随机抖动（σ≈N^(−2/D)），把"挤成一团"重新撑开</li>
            <li><b class="vi">随机注入</b>：每轮随机撒少量均匀粒子（如 1%），权重极低但"保底"—— 绑架恢复的种子。AMCL 的 random limit 参数</li></div>''',
          notes='系统采样是无脑默认；两招急救对应 AMCL 两个参数。'),

        S('从粒子到 Gmapping：每个粒子背一张地图', '''
          <div class="formula">纯 PF：粒子 = 位姿假设
Gmapping：粒子 = 位姿假设 + 一张专属栅格地图    ← Rao-Blackwell 化

为什么能这样：给定"我确实走了这条轨迹"，地图的更新是第①讲的确定性计算
（log-odds 投票）—— 不需要再采样，直接算。</div>
          <p class="big">这就是 <b class="cy">FastSLAM 的分解智慧</b>：位置用粒子硬扛（多峰），
          地图用解析更新（便宜）。</p>
          <li>每个粒子维护自己的轨迹 + 自己的地图 → 回环时，正确轨迹的地图自洽、权重涨；错误轨迹的地图对不上、权重跌 → 自然选择</li>
          <li>代价：内存 × N 张地图。所以 Gmapping 默认只有 <b class="wa">30 个粒子</b>（不是几千）</li>''',
          notes='Rao-Blackwell 一词口头带过即可：条件可解析的部分解析化。'),

        S('Gmapping 参数对照（第⑦讲会实操）', '''
          <table>
            <tr><th>参数</th><th>默认</th><th>含义 / 对应本讲概念</th></tr>
            <tr><td><span class="mono">particles</span></td><td>30</td><td>N 个假设（每粒子一张地图，别开大）</td></tr>
            <tr><td><span class="mono">srr / srt</span></td><td>0.01</td><td>平移/旋转耦合的里程计噪声 —— 预测步的 Q</td></tr>
            <tr><td><span class="mono">stt</span></td><td>0.01</td><td>纯旋转噪声</td></tr>
            <tr><td><span class="mono">sigma_hit</span></td><td>0.147</td><td>似然打分的高斯 σ —— 权重页的 σ</td></tr>
            <tr><td><span class="mono">lstep / astep</span></td><td>10</td><td>粗到细搜索步 —— 第⑤讲的评分搜索</td></tr>
            <tr><td><span class="mono">map_update_interval</span></td><td>5 s</td><td>/map 发布频率（不是内部更新频率）</td></tr>
          </table>
          <div class="warnbox">新手通病：把 particles 调到 300 想"更准" → 内存与 CPU 爆炸，精度几乎不变。
          精度瓶颈通常在 <span class="mono">sigma_hit</span> 与里程计噪声模型，不在粒子数。</div>''',
          notes=' particles 恰恰不是精度旋钮——最常见的调参误区。'),

        S('粒子滤波什么时候会失败', '''
          <li>❌ <b class="bad">绑架问题（kidnapped robot）</b>：机器人被抱到完全陌生的位置 ——
          所有粒子都错、权重全 0、没有假设幸存。对策：随机注入（前页）+ 权重和过低时全局重撒</li>
          <li>❌ <b class="bad">粒子饥饿</b>：真值恰好在某粒子附近，但它在上一轮被绝育 —— 1000 个粒子也救不了。
          对策：粗化 + 注入；根本解是增加粒子或降低重采样频率</li>
          <li>❌ <b class="bad">高维状态</b>：状态 6 维（AMCL 只有 3 维）时，覆盖同样密度需要指数级粒子 —— 换 EKF/UKF 或图优化</li>
          <li>❌ <b class="bad">动态场景误锁</b>：一走廊的人把所有粒子的观测都对成"人墙" —— 需要鲁棒似然（截断高斯）</li>''',
          notes='绑架是面试高频题：全灭→全局重撒。'),

        S('AMCL：粒子滤波在定位岗位上的工程形态', '''
          <p>地图建好后，PF 换个岗位：<b class="cy">已知地图上定位</b>（蒙特卡洛定位 MCL）。粒子不再背地图，只背位姿 —— 轻多了。</p>
          <table>
            <tr><th>AMCL 的工程改进</th><th>解决什么</th></tr>
            <tr><td>KLD 自适应采样</td><td>信念散（迷失）时自动加粒子，收敛后自动减 —— CPU 花在刀刃上</td></tr>
            <tr><td>随机粒子注入 (random limit)</td><td>绑架恢复的种子</td></tr>
            <tr><td>odom alpha 5 参数</td><td>运动模型噪声（第②讲那个 20% 的精细化版）</td></tr>
            <tr><td>laser z_hit / z_rand</td><td>观测模型："真命中" 与 "随机噪声" 的混合比例</td></tr>
          </table>
          <div class="note">nav2 默认定位栈 = <span class="mono">map_server + amcl</span>。
          AMCL 崩掉的三大常见原因（初始位姿没给、odom 噪声 alpha 乱设、地图分辨率不匹配）—— 第⑦讲排障表见。</div>''',
          notes='AMCL=PF 的岗位化：地图已知、粒子只装位姿。'),

        qa('重采样后，某个"少数派但正确"的粒子簇权重被杀光。这属于粒子滤波的什么失败模式？',
           ['绑架问题',
            '粒子饥饿 / 多样性损失',
            '数值下溢',
            '过度自信'],
           1,
           '重采样是自然选择，会牺牲少数派。真值若恰在少数派中，后续观测再也救不回来（没有假设了）。粗化+注入是对策。',
           notes='与 bf 误报率 0.2 呼应：留余地就是留多样性。'),

        qa('Gmapping 默认只有 30 个粒子，为什么不敢开几百个？',
           ['粒子太多数值不稳定',
            '每个粒子携带一张完整栅格地图，内存与更新开销 × N',
            '粒子多会互相干扰',
            'ROS 消息大小限制'],
           1,
           'Gmapping 粒子=位姿+地图（Rao-Blackwell 化的代价）。AMCL 粒子只装位姿，所以敢开几千。',
           notes='对比记忆：Gmapping 30 个，AMCL 可 2000 个。'),

        S('一页总结：粒子滤波',
          '''<div class="formula" style="font-size:16px">N 个带权假设 = 信念分布的采样表达
循环：预测（各自动）→ 加权（似然）→ 重采样（按权重抽签）→ 估计（加权平均）
体检：N_eff = 1/Σwᵢ²      触发重采样 + 质量报警
衍生：Gmapping（粒子+地图 → 建图）、AMCL（粒子+已知地图 → 定位）</div>
          <p class="big">带走三句话：<b class="cy">粒子云=信念的形状</b> · <b class="vi">重采样=自然选择（会饿死少数派）</b> · <b class="pk">N 不是精度旋钮</b></p>
          <div class="note">下一讲：粒子之间毕竟还是"独立假设"。两帧激光怎么直接对齐出精确位姿？—— 扫描匹配，评分函数登场。</div>''',
          cls='cover'),
    ],
}
