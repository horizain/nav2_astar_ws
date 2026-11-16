#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 ⑦ 讲 · 工程实战（标定 / 参数 / 抗干扰 / 排障）"""
from framework import S, qa

DECK = {
    'file': 'eng.html',
    'title': '⑦ 工程实战',
    'chapter': '第 ⑦ 讲 · 工程实战',
    'slides': [
        S('这一讲回答：六讲算法怎么装到真实机器人上不出事？',
          '''<p class="big">仿真里"能用"和实车上"可靠"之间隔着四座山：
             <b class="cy">传感器标定 · 时间同步 · 参数适配 · 故障排查</b>。
             本讲全部是实操清单 —— 每一条都对应一个真实翻车现场。</p>''',
          cls='cover'),

        S('传感器选型账单（按场景挑）', '''
          <table>
            <tr><th>传感器</th><th>盲区/短板</th><th>单价档</th><th>适合</th></tr>
            <tr><td>2D 单线激光（如 rplidar S2）</td><td>看不到悬空/低矮障碍；玻璃/镜面不友好</td><td>¥千元级</td><td>室内平面 AGV、服务机器人</td></tr>
            <tr><td>3D 激光（如 livox mid360）</td><td>贵；点云处理算力要求高</td><td>¥万元级</td><td>室外、多层结构、无人机</td></tr>
            <tr><td>深度相机（如 D435i）</td><td>强光/透明/远距离（&gt;8 m）失效</td><td>¥千元级</td><td>室内近距离避障、视觉 SLAM 入门</td></tr>
            <tr><td>轮式里程计</td><td>打滑/磨损 → 必漂（直觉①）</td><td>≈0</td><td>永远作为预测初值，不单独定位</td></tr>
            <tr><td>IMU</td><td>陀螺零偏漂移 → 单独用必崩</td><td>¥百~千元</td><td>姿态估计、转弯增强（与轮速互补）</td></tr>
          </table>
          <div class="note">黄金组合（教学/室内）：轮速 + IMU + 2D 激光 —— 成本最低、噪声模型最简单、第②~⑥讲全部直接适用。</div>''',
          notes='选型先问环境几何：单层平面？多层？室外？'),

        S('标定 ①：轮式里程计（最常被跳过、最该做）', '''
          <p>原理：<b class="cy">真值 vs 读数</b>的线性回归。需要：卷尺 + 地面胶带 + 一个来回。</p>
          <div class="formula">平移标定：让机器人直线走 5 m，测真实距离 d_real
    比例 k_d = d_real / d_odom          （轮径误差：磨损/胎压）

旋转标定：原地转 10 圈，测真实朝向角 θ_real
    比例 k_θ = θ_real / θ_odom          （轮距误差）

修正：把 k 写进 URDF 的 wheel 参数或里程计源码（diff_drive 的 wheel_separation_multiplier）</div>
          <div class="frag"><p class="big">UM 协议的进阶做法（5 条直线各走两次：正走/交换位置反走）——
             能同时估计出<b>轮径、轮距、左右轮偏差</b> 4 个参数，工具：<span class="mono">turtlebot3_calibration</span>、<span class="mono">burgers_calibration</span> 思路。</p></div>
          <div class="warnbox">不做标定的代价：k_d 差 3% → 每 5 m 漂 15 cm —— 第①~⑥讲全在为这 15 cm 加班。</div>''',
          notes='两把尺子一次 afternoon 就能做完，ROI 最高的一项工程。'),

        S('标定 ②：激光雷达外参（安装位姿）', '''
          <div class="formula">雷达装的位置 (x,y,z) 与朝向 (roll,pitch,yaw) 必须写进 TF：
  base_link → laser 静态 TF
  高度不准 → 激光平面错过桌面/台阶边缘
  偏航不准 → 前进方向与激光 0° 不重合 → 地图整体歪</div>
          <p>手算校验法（10 分钟）：</p>
          <li>机器人正对一面平墙 2 m，雷达测距应 ≈ <b class="cy">2 − 安装偏移</b>；左右平移机器人 0.5 m，测距变化量应与移动量一致</li>
          <li>原地旋转 90°，若墙在地图里"扇形错开" → yaw 安装角或旋转标定（k_θ）有问题</li>
          <li>工具：<span class="mono">rqt_tf_tree</span> 检查 TF；rviz 里把激光点云与机器人体素叠着看</li>''',
          notes='雷达歪 1°，走廊尽头偏 1.7 m——外参要拿量角器认真装。'),

        S('标定 ③：时间同步（隐形的墙）', '''
          <div class="formula">问题：激光帧 t=10.000 到达，里程计位姿是 t=9.985 的 → 位姿差 1.5 cm（0.5 m/s 时）
转弯 90°/s 时位姿差 1.35° → 走廊尽头错 17 cm → 地图重影</div>
          <table>
            <tr><th>层级</th><th>手段</th></tr>
            <tr><td>系统层</td><td>chrony/NTP 对时；WSL2 里宿主机与容器必须同源</td></tr>
            <tr><td>消息层</td><td><span class="mono">message_filters::Synchronizer</span> 精确配对 scan+odom 时间戳</td></tr>
            <tr><td>算法层</td><td>束内插值：一帧激光跨越 25 ms，按时间在两个 odom 位姿间插值（slerp）</td></tr>
          </table>
          <div class="warnbox">WSL2 用户必看：Windows 休眠/唤醒后时钟漂移，slam_toolbox 突然开始"抽搐" —— 先查 <span class="mono">chronyc tracking</span>。</div>''',
          notes='时间同步是"地图重影"类问题的第一大来源。'),

        S('抗干扰 ①：动态环境（人、椅子、门）', '''
          <table>
            <tr><th>干扰</th><th>后果</th><th>工程对策</th></tr>
            <tr><td>行人穿越</td><td>鬼影墙 + 匹配抖动</td><td>log-odds 对冲自然消化（第①讲 clamp ±4）；cartographer 可开 occupancy 剪枝</td></tr>
            <tr><td>椅子被挪动</td><td>静态地图与现场不符 → 定位飘</td><td>定期重建地图；或开<b>持续建图模式</b>（slam_toolbox localization + map update）</td></tr>
            <tr><td>门开关</td><td>门区域两套几何来回刷</td><td>规划层处理（inflate 半径 &gt; 门宽变化）；SLAM 不管这事</td></tr>
            <tr><td>玻璃幕墙</td><td>半透/镜面回波错乱</td><td>物理解决：贴防撞条/纹理胶带（让雷达"看见"玻璃）</td></tr>
          </table>
          <p class="big">分界原则：<b class="cy">SLAM 的地图应包含"准静态"结构，动态障碍交给规划/感知层</b> —— 别把人画进墙。</p>''',
          notes='职责分界是架构问题：SLAM 管墙，感知管活物。'),

        S('抗干扰 ②：环境对称与几何退化', '''
          <p>三个高危环境 + 预案：</p>
          <li><b class="cy">对称走廊</b>：假回环重灾区 → 加贴地码/二维码/立牌，或改用视觉描述子辅助</li>
          <li><b class="vi">长直走廊</b>：纵向漂移不可观（第⑤讲）→ 提高里程计权重 λ、依赖回环约束、缩短回环间隔</li>
          <li><b class="pk">空旷大厅</b>：多方向不可观 → 加柱状参照物；运动时保持对墙有一定夹角（别正对）</li>
          <div class="formula">检测退化的土办法：跑一圈后看 rviz ——
  墙变厚（灰带加宽）     → 匹配不稳
  墙错位/断裂           → 漂移未被回环吸收
  直线变弧线            → 里程计旋转标定（k_θ）不准</div>''',
          notes='三个土办法征状 → 病因映射，排障时直接背。'),

        S('参数调优：先调哪个？（有顺序！）', '''
          <table>
            <tr><th>优先级</th><th>参数组</th><th>为什么先调它</th></tr>
            <tr><td>①</td><td>里程计标定（k_d/k_θ/alpha）</td><td>地基：它错 → 后面全错；标定一次终身受益</td></tr>
            <tr><td>②</td><td>传感器噪声（sigma_hit、R）</td><td>"信谁"的根基（第③④⑤讲反复出现）</td></tr>
            <tr><td>③</td><td>地图分辨率 / clamp</td><td>决定建图的"笔触"与可恢复性</td></tr>
            <tr><td>④</td><td>匹配窗口（coarse/fine）</td><td>处理大场景/快运动</td></tr>
            <tr><td>⑤</td><td>回环参数（search_space/chain_size）</td><td>最后微调全局纠偏行为</td></tr>
            <tr><td>⑥</td><td>粒子数 / 更新频率</td><td>性能旋钮，最后动</td></tr>
          </table>
          <div class="warnbox">反模式警告：一上来把 20 个参数全改一遍 → 失败了无法定位是哪个参数的锅。<b class="ok">一次一个参数 + 固定测试路线</b>。</div>''',
          notes='顺序方法论：从地基到屋顶。'),

        S('性能预算：CPU 都花在哪了', '''
          <table>
            <tr><th>模块</th><th>开销来源</th><th>典型占比</th><th>压开销的开关</th></tr>
            <tr><td>扫描匹配</td><td>窗口穷举 × 点数</td><td>40~60%</td><td>multi-res 粗到细、minimum_travel_head 拉大</td></tr>
            <tr><td>建图更新</td><td>Bresenham 逐格 × 束数</td><td>20~30%</td><td>降低更新频率（隔帧）、降分辨率</td></tr>
            <tr><td>粒子滤波</td><td>N 粒子 × 束数打分</td><td>10~20%</td><td>KLD 自适应（AMCL）、收敛后减粒子</td></tr>
            <tr><td>图优化</td><td>稀疏求解 × 迭代</td><td>5~10%</td><td>增量求解（gtsam iSAM2）</td></tr>
          </table>
          <p class="big">排位规律：<b class="cy">扫描匹配几乎总是第一名</b> —— 窗口面积 × 分辨率² × 点数。
          紧了先动它的窗口参数，别先杀粒子。</p>''',
          notes='给出"先动谁"的顺序，避免瞎优化。'),

        S('故障速查表（收藏页）', '''
          <table>
            <tr><th>症状</th><th>最可能原因</th><th>第一动作</th></tr>
            <tr><td>地图整体歪斜/镜像</td><td>TF 树错 / laser 外参 yaw 错</td><td><span class="mono">rqt_tf_tree</span>；重装雷达</td></tr>
            <tr><td>墙重影（双线）</td><td>时间不同步 / odom 标定差</td><td>chronyc tracking；做轮径标定</td></tr>
            <tr><td>转弯处墙变扇形</td><td>束内插值缺失 / 转速过快</td><td>检查插值；限速 60°/s</td></tr>
            <tr><td>走廊尽头漂移大</td><td>几何退化（正常！）</td><td>靠回环/贴参照物</td></tr>
            <tr><td>回环后地图"跳变"</td><td>回环正常工作（不是 bug）</td><td>确认跳变后几何对齐即可</td></tr>
            <tr><td>AMCL 抖动/失效</td><td>初始位姿未给 / alpha 乱 / 分辨率不匹配</td><td>先给 /initialpose；核对地图 yaml</td></tr>
            <tr><td>CPU 满</td><td>匹配窗口过大 / 粒子过多</td><td>按性能预算表从匹配开刀</td></tr>
            <tr><td>WSL2 建图突抽搐</td><td>宿主机时钟漂移</td><td><span class="mono">chronyc tracking</span> / 重启 WSL</td></tr>
          </table>''',
          notes='这张表建议截图保存——80% 的现场问题都在里面。'),

        qa('机器人直线走 5 m，里程计报 4.85 m。正确处理是？',
           ['在 SLAM 参数里把传感器噪声调大掩盖误差',
            '做轮径标定：k_d = 5.0/4.85 ≈ 1.031，写进驱动或 diff_drive 参数',
            '接受误差，靠扫描匹配每帧硬修',
            '把地图分辨率调细一点'],
           1,
           '系统性的轮径误差必须在源头标定。k_d=1.031 意味着轮径标称值偏了 3.1%（磨损/胎压）。让 SLAM 每帧硬修是让纠偏模块替标定上班——既费 CPU 又增加失败面。',
           notes='"在源头修，不在下游修"是工程第一原则。'),

        qa('建图过程中机器人回到起点，地图突然整体"跳"了一下并几何对齐。发生了什么？',
           ['算法 bug，需要重启',
            '回环检测+图优化正常工作：一次性把累积漂移全局拉直',
            'AMCL 恢复了定位',
            '激光掉帧后的自动恢复'],
           1,
           '回环跳变是全局优化的标志性行为（第⑥讲弹簧网一次变形）。跳变后墙线对齐=好事；如果跳变后反而错位，才需要查假回环。',
           notes='很多新手在回环跳变时慌了——这页就是给他们打的疫苗。'),

        S('上机 checklist（从零到跑通）', '''
          <div class="formula">□ 0. 传感器能出数据    <span class="mono">ros2 topic hz /scan</span>  ≈ 标称频率
□ 1. TF 树正确          <span class="mono">ros2 run rqt_tf_tree rqt_tf_tree</span>  base_link→laser 存在
□ 2. 里程计标定完成      5m 直线 + 10 圈旋转 误差 &lt; 3%
□ 3. 时钟同步           <span class="mono">chronyc tracking</span>  offset &lt; 10 ms
□ 4. 固定测试路线       贴胶带标记起点，保证每次实验可复现
□ 5. 建图启动           slam_toolbox online_async + rviz /map
□ 6. 慢速建图           ≤0.3 m/s、≤60°/s（新手期）
□ 7. 回环验证           绕圈回起点，看跳变后几何是否对齐
□ 8. 保存地图           <span class="mono">slam_toolbox serialize</span> 或 map_saver
□ 9. 定位验证           map_server + AMCL + /initialpose
□ 10. 导航联调          nav2 + 本课控制器文档 15/16/17</div>
          <p class="big">每一步都是<b class="vi">可独立验证</b>的 —— 哪步失败就只查那步，不要跳步。</p>''',
          notes='checklist 精神：小步验证，失败面最小化。'),

        S('一页总结：工程实战',
          '''<div class="formula" style="font-size:16px">四座山：
  标定（轮径/轮距/雷达外参）  —— 在源头消灭系统性误差
  时间同步（系统/消息/束内插值）—— 消灭重影
  抗干扰（动态/对称/退化）    —— 职责分界 + 物理预案
  排障（速查表 + checklist）  —— 小步验证</div>
          <p class="big">带走三句话：<b class="cy">在源头修误差，别让下游硬扛</b> · <b class="vi">一次一个参数+固定路线</b> · <b class="pk">回环跳变是好事</b></p>
          <div class="note">本课全部内容与 doc 20/21/22 互为镜像：本课给直觉与手算，文档给命令与参数全集。配合食用。</div>''',
          cls='cover'),
    ],
}
