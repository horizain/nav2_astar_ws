# 30 · `scan_localizer` 交互式工程教程导览

> **定位（Localization）决定机器人知不知道自己在哪。**  
> [scan_localizer](../src/scan_localizer/README.md) 是本工作空间基于纯 C++20 + Eigen 核心与 ROS 2 Jazzy Lifecycle 节点的生产级 2D 激光定位系统。本课程以**全新 Apple 极简交互式网站**呈现，将数学推导、可交互 Canvas 动画、实时源码阅读抽屉与 Intel Research Lab 历史真值结果解读（本轮未复现）融为一体。

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
| 导读 | [交互式代码思路导读](scan_localizer_tutorial/walkthrough.html) | 3 条执行流程图、UML 类图、可折叠调用链与 14 个函数“从零手写思路” |
| 索引 | [代码全景索引 (33 文件)](scan_localizer_tutorial/code-index.html) | 3 条阅读路线（3h 速通 / 算法核 / 全栈工程）、全符号搜索与 Windows/WSL VS Code 直开 |
| Ch01 | [从零实现 SE(2) 与 TF 树](scan_localizer_tutorial/chapters/ch01.html) | 最小 Pose2 代数 + 复合/逆数值演算 + `map→odom` 源码步进与练习 |
| Ch02 | [从零写出 MapField 与精确 EDT](scan_localizer_tutorial/chapters/ch02.html) | 最小 edt1d + `[9,100,4]` 栈状态手算 + 源码步进 + 填空验收 |
| Ch03 | [从零实现 Correlative 与经典匹配器](scan_localizer_tutorial/chapters/ch03.html) | 最小窗口评分 + 旋转复用 + ICP 局部极小对比 + 源码步进与练习 |
| Ch04 | [从零写出 Field-GN 与两级组合](scan_localizer_tutorial/chapters/ch04.html) | 最小可运行内核 + 一轮 H/b/dp 手算 + 7 段源码步进 + 填空练习与验收 |
| Ch05 | [从零实现退化安全的协方差](scan_localizer_tutorial/chapters/ch05.html) | 最小特征值夹紧 + 未阻尼 Hessian + 实时各向异性面板与练习 |
| Ch06 | [从零实现 B&B 全局重定位](scan_localizer_tutorial/chapters/ch06.html) | 上界剪枝 + Top-K NMS + 粒子滤波对照 + 源码步进与练习 |
| Ch07 | [从零实现 4D EKF、四门控与状态机](scan_localizer_tutorial/chapters/ch07.html) | 最小 EKF 更新内核 + 预测/门控/Joseph 源码步进 + 实时 P4 状态面板 |
| Ch08 | [从零实现 ROS 2 定位节点](scan_localizer_tutorial/chapters/ch08.html) | Lifecycle 骨架 + 锁外构场 + 时间对齐 + 快照发布与练习 |
| Ch09 | [从零设计不作弊的基准实验](scan_localizer_tutorial/chapters/ch09.html) | 配对试验 + 固定种子 + 指标复算 + 公平性检查与练习 |
| Ch10 | [C++20 × Eigen × ROS 2 代码重建](scan_localizer_tutorial/chapters/ch10.html) | 语法卡片 + 关键行填空 + 源码步进与验收 |

---

## 配套验证测试

每次修改教程 JS 数学核或调整源码行号后，可在仓库根目录运行以下验证：

```bash
node docs/scan_localizer_tutorial/tests/test_sim_core.mjs       # 16 项算法与数学一致单测
python3 docs/scan_localizer_tutorial/tests/check_code_refs.py   # 33 文件覆盖率与行号越界检查
python3 docs/tools/check_tutorial_links.py                      # 全仓 Markdown 本地链接检查
```
