#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""资源页：文档 / 书 / 课程 / 工具"""
from framework import S

DECK = {
    'file': 'res.html',
    'title': '资源',
    'chapter': '附页 · 资源与延伸',
    'slides': [
        S('本仓库配套文档（与本课互为镜像）',
          '''<table>
            <tr><th>文档</th><th>内容</th><th>配合哪几讲</th></tr>
            <tr><td><span class="mono">docs/20_定位与建图SLAM.md</span></td><td>管线实测：slam_toolbox 命令、跑通全链、地图保存</td><td>全讲 / ⑦</td></tr>
            <tr><td><span class="mono">docs/21_SLAM建图原理零基础课.md</span></td><td>原理详解：每个算法的手算与推导（本课的"底稿"）</td><td>①~⑥</td></tr>
            <tr><td><span class="mono">docs/22_SLAM工程实战.md</span></td><td>标定/参数/抗干扰完整版</td><td>⑦</td></tr>
            <tr><td><span class="mono">docs/tools/slam_playground.py</span></td><td>终端版六演示（--all 一键跑）</td><td>①~⑥</td></tr>
            <tr><td><span class="mono">docs/tools/save_slam_map.py / auto_explorer.py</span></td><td>自动探索与存图脚本</td><td>⑦</td></tr>
          </table>
          <p class="big">本课负责<b class="cy">懂</b>，文档负责<b class="vi">做</b> —— 复习时按小节回本课、按章节回文档。</p>''',
          cls='cover'),

        S('经典书与课程（进阶路线）', '''
          <table>
            <tr><th>资源</th><th>定位</th><th>读法建议</th></tr>
            <tr><td>《Probabilistic Robotics》(Thrun)</td><td>本课②~④讲的原著</td><td>看 Ch.2-7：贝叶斯滤波 → KF/EKF → MCL；公式与本课一一对应</td></tr>
            <tr><td>《视觉SLAM十四讲》(高翔)</td><td>视觉向进阶 + 数学底子</td><td>第 3/4 讲补李代数，第 10 讲位姿图与第⑥讲同源</td></tr>
            <tr><td>Cyrill Stachniss 的 YouTube 课程</td><td>免费视频课（SLAM/传感器建模）</td><td>适合当"有声版"复习；Grid Maps / EKF / GraphSLAM 系列全</td></tr>
            <tr><td>《State Estimation for Robotics》(Tim Barfoot)</td><td>状态估计大部头</td><td>想吃透 gtsam/因子图再啃</td></tr>
          </table>
          <div class="note">路线：本课 → 概率机器人 Ch.2-7 → 十四讲（视觉兴趣）/ Barfoot（深度兴趣）。</div>'''),

        S('工程工具箱（开源实现）', '''
          <table>
            <tr><th>工具</th><th>是第几讲的工业实现</th><th>何时选它</th></tr>
            <tr><td><b class="cy">slam_toolbox</b></td><td>①+⑤+⑥ 的组合（2D 栅格 SLAM）</td><td>室内 2D，ROS2 首选、教学友好</td></tr>
            <tr><td><b class="cy">cartographer</b></td><td>①+⑤+⑥（含分支定位回环）</td><td>2D 大场景/高鲁棒要求；配置复杂</td></tr>
            <tr><td><b class="vi">Gmapping</b></td><td>④ 粒子滤波建图</td><td>老牌、轻量；ROS2 里维护减弱</td></tr>
            <tr><td><b class="vi">AMCL</b></td><td>④ 定位版</td><td>nav2 标配定位</td></tr>
            <tr><td><b class="pk">LIO-SAM / FAST-LIO</b></td><td>③+⑥ 的 3D 惯导融合版</td><td>3D 激光 + IMU，室外</td></tr>
            <tr><td><b class="pk">RTAB-Map / ORB-SLAM3</b></td><td>视觉 SLAM</td><td>相机为主传感器时</td></tr>
          </table>
          <p>判断顺序：先问维度（2D/3D）→ 再问主传感器 → 再问算力预算。</p>'''),

        S('保持练习：三个自测项目',
          '''<p class="big">学完七讲后，用这三个项目验收：</p>
          <p><b class="cy">项目 A · 白板复述</b>：不看材料，画出"扫描匹配 → 建图 → 回环"管线图并标注每步输入输出与失败模式。</p>
          <p><b class="vi">项目 B · 手算重考</b>：重做第⑥讲正规方程（改 σ：里程计 0.2、回环 0.05，问 x₃ 落哪？）—— 答案自验：加权融合公式。</p>
          <p><b class="pk">项目 C · 实车对照</b>：跑通 doc 20 的 slam_toolbox 全链路，故意把时钟弄不同步，复现"重影"故障并用第⑦讲速查表修好。</p>
          <div class="note">B 的答案：里程计链精度 1/(0.2²+0.2²)=12.5 &lt; 回环 1/0.05²=400 → x₃ 紧贴回环的 1.8（≈1.802）。
          这就是"标定好回环就能救烂里程计"的定量版。</div>''',
          cls='cover'),
    ],
}
