# 30 · `scan_localizer` 交互式工程教程导览

> **定位（Localization）决定机器人知不知道自己在哪。**  
> [scan_localizer](../src/scan_localizer/README.md) 是本工作空间基于纯 C++20 + Eigen 核心与 ROS 2 Jazzy Lifecycle 节点的生产级 2D 激光定位系统。本课程以**全新 Apple 极简交互式网站**呈现，将数学推导、可交互 Canvas 动画、实时源码阅读抽屉与 Intel Research Lab 真值实测融为一体。

---

## 一键打开交互式课堂

```bash
cd ~/ros2_ws/src/nav2_astar_ws
./docs/scan_localizer_tutorial/serve.sh
```

在浏览器打开输出的本地地址（默认 `http://127.0.0.1:8090/docs/scan_localizer_tutorial/index.html`）：
- **实时源码抽屉**：点击正文中任意蓝色源码徽章（如 `localizer_core.cpp:L548-704`），右侧滑出仓库真实源文件并高亮焦点行，支持 `⌘K` 全局符号检索与深浅色一键切换；
- **也可直接离线打开** [教程首页](scan_localizer_tutorial/index.html) 或 [33 文件全景索引](scan_localizer_tutorial/code-index.html)（注：离线 `file://` 模式受浏览器安全策略限制，实时抽屉需通过 `./serve.sh` 启动或使用按钮跳至 VS Code）。

---

## 十章课程与全景索引目录

| 阶段 | 入口页面 | 核心主题与交互实验 |
|---|---|---|
| 总览 | [首页 · 架构与指标](scan_localizer_tutorial/index.html) | 实时仓库 SDF 扫描动画、Intel Lab 核心指标 Bento、五层架构图 |
| 导读 | [交互式代码思路导读](scan_localizer_tutorial/walkthrough.html) | 3 条执行流程图、UML 类图、可折叠调用链与 12 个函数“从零手写思路” |
| 索引 | [代码全景索引 (33 文件)](scan_localizer_tutorial/code-index.html) | 3 条阅读路线（3h 速通 / 算法核 / 全栈工程）、全符号搜索与 Windows/WSL VS Code 直开 |
| Ch01 | [定位与 SE(2) / TF 树](scan_localizer_tutorial/chapters/ch01.html) | 极坐标点云、`map→odom` 合成、PGM Y 轴翻转、倒装雷达演示 |
| Ch02 | [从零写出 MapField 与精确 EDT](scan_localizer_tutorial/chapters/ch02.html) | 最小 edt1d + `[9,100,4]` 栈状态手算 + 源码步进 + 填空验收 |
| Ch03 | [经典匹配器演进](scan_localizer_tutorial/chapters/ch03.html) | Correlative 热力图 vs ICP-P2P 局部极小掉坑 + 2D NDT 椭圆场 |
| Ch04 | [从零写出 Field-GN 与两级组合](scan_localizer_tutorial/chapters/ch04.html) | 最小可运行内核 + 一轮 H/b/dp 手算 + 7 段源码步进 + 填空练习与验收 |
| Ch05 | [长走廊退化与各向异性](scan_localizer_tutorial/chapters/ch05.html) | 未阻尼 Hessian 求逆、120,000× 协方差椭圆实验室 |
| Ch06 | [B&B 全局定位 vs 粒子滤波](scan_localizer_tutorial/chapters/ch06.html) | `swmax_` 分支定界剪枝 vs 400 粒子滤波，一键随机绑架 |
| Ch07 | [从零实现 4D EKF、四门控与状态机](scan_localizer_tutorial/chapters/ch07.html) | 最小 EKF 更新内核 + 预测/门控/Joseph 源码步进 + 实时 P4 状态面板 |
| Ch08 | [ROS 2 生产节点 LocalizerNode](scan_localizer_tutorial/chapters/ch08.html) | Lifecycle、锁外构场原子交换、6×6 `[35]` 坑与仓库驾驶舱 |
| Ch09 | [E1–E10 与真值三方评测](scan_localizer_tutorial/chapters/ch09.html) | 基准防自欺方法论与 Intel Lab 450 组真值三方实测解读 |
| Ch10 | [C++20 × Eigen × ROS 2 手册](scan_localizer_tutorial/chapters/ch10.html) | 按领域标签筛选的 10 组最佳实践 vs 常见陷阱代码卡 |

---

## 配套验证测试

每次修改教程 JS 数学核或调整源码行号后，可在仓库根目录运行以下验证：

```bash
node docs/scan_localizer_tutorial/tests/test_sim_core.mjs       # 16 项算法与数学一致单测
python3 docs/scan_localizer_tutorial/tests/check_code_refs.py   # 33 文件覆盖率与行号越界检查
python3 docs/tools/check_tutorial_links.py                      # 全仓 Markdown 本地链接检查
```
