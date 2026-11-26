# scan_localizer 泛化架构独立审核意见与四阶段演进路线

本文档对 `8784b96` 提交及 Opus 5.5 泛化评审报告进行了独立交叉核查，指出了原评审方案中的物理/数学/算力陷阱以及源码级根因缺陷，并给出了三层闭环泛化架构设计与四阶段工程落地路线。

---

## 背景与总体裁定

针对 `8784b96` 提交与 Opus 5.5 泛化评审报告的独立交叉核查结论——战略方向 80% 正确（去经验常数化、概率化、多峰与可见性下沉至 Core），但原评审方案存在 **3 个物理/数学/算力陷阱**，并遗漏了 **3 个源码级根因缺陷**。

---

## 一、逐项独立复核与关键修正（含三大陷阱）

### 1. ROS 节点路径多峰与可见性断路（确认并深化）
不仅 [matchers.hpp](../include/scan_localizer/matchers.hpp) 的 `beam_visibility_consistency` 在仓内零调用，更深层原因是 [localizer_core.hpp](../include/scan_localizer/localizer_core.hpp) 的 `GlobalFn` 仅支持单一位姿返回，且 [localizer_core.cpp](../src/localizer_core.cpp) 收到返回后直接切换 `TRACKING` 并调用 `clear_hypotheses()`，导致节点层无法挂载备选假设。必须将多峰召回、可见性校验与多假设竞争统一收拢进 `LocalizerCore`。

### 2. 陷阱 1——序贯贝叶斯（SPRT）在静止/对称走廊下的静态相关性坍缩
若按每帧无条件累加 `log_w_i += n_eff * l_i`，机器人在原地静止或全对称走廊匀速直行时，连续帧观测高度相关，栅格量化导致的 `0.01` 静态分差经 10 帧累加后后验即突破 `0.99`，引发原地假自信锁错峰。
- **修正方案**：第 0 帧按初始对数似然与锚点连续性先验计算后验（唯一特征区 1 帧瞬锁）；多峰歧义区后续证据累加必须乘以运动/视角新息门控因子 $\gamma_t = \min(1, \|\Delta p\|/d_0 + |\Delta\theta|/\theta_0)$，不动不虚增置信度，运动遇破缺特征即刻锁定。

### 3. 退化子空间更新与法向均衡加权（高性价比采纳并补充根因）
2D 激光等角采样使 1m 近侧墙线点密度为 10m 端墙/门框的 10 倍，且真实栅格墙面像素锯齿梯度 $g_x \neq 0$ 在 300+ 侧墙点上平方累加（$\sum g_x^2$）伪造纵向海森刚度。通过**法向空间分桶均衡/弧长加权**（导出有效独立观测数 $n_{\text{eff}}$）配合**可观子空间 EKF 更新**，从根源替代 `cond > 6` 经验补丁。

### 4. 扫描去畸变与时序噪声建模（采纳去畸变，否决在线时延估计与 Sage-Husa）
10Hz 雷达在 `1 rad/s` 转向时首尾光束畸变达 `5.7°`（10m 处偏移 1m），必须补齐运动去畸变；删掉易吞错峰的 `P *= 2.5`，但不引入易在退化/遮挡下发散的在线时延状态估计与 Sage-Husa 自适应 $Q$，改用物理闭合的速度耦合时序抖动项 $Q_{\text{sync}} = \sigma_{t,\text{sync}}^2 \mathbf{v}\mathbf{v}^T$ 与可观子空间 Huber 软加权更新。

### 5. 陷阱 2——可见性模型漏判“建图静态残影/搬走家具（穿占据格命中真墙）”
实核 [corridor_relocalization_notes.md](corridor_relocalization_notes.md) 与 [test_matchers.cpp](../test/test_matchers.cpp) 可知，实车痛点并非仅 `z_short` 短读数，而是**建图残留 0.5m 桌椅占据格（`100`）、运行时已搬走导致光束穿透占据格命中远端真墙**。逐格扣 `log(p_pass_occ)` 会立即复现真机掉线。
- **修正方案**：基于精确 SDF 球面跳跃追踪建立**四态物理波束模型**——① 正常表面命中；② 新增动态遮挡 `z_short`（自由空间提前截断，从内点分母剔除）；③ 残影/擦边穿透（穿过局部占据团块但终点落在后方真墙 `|SDF| < 2σ`，免除重罚）；④ 非物理穿墙外溢（穿墙射入 `-1` 未知灰区或终点悬空，重罚）。

### 6. 陷阱 3——RK3399 板端算力预算失控（修正自适应角步长与地图离线分析算子）
- **全局搜索角步长**若用 `acos(1 - res^2 / (2 r_max^2))` 在 5cm 地图、12~25m 量程下将产生 1500~3100 个角度切片，RK3399 耗时将从 `~350ms` 暴涨至 `3~6s`。因似然场具备 $\sigma$ 平滑吸引盆，粗搜角步长应按 $\Delta\theta \approx \sigma / r_{p90}$ 物理推导（约 `1.0°~1.5°`），并将末端 `9x5x5` 网格细修替换为 `FieldGnMatcher`。
- **地图离线孪生分析**若对自由格逐点采样跑 `global_correlative`，RK3399 单次载图需 30+ 分钟，破坏热切图。改为基于 `MapField` 障碍表面 SDF 梯度结构张量 $\nabla\text{SDF}\cdot\nabla\text{SDF}^T$ 粗网格统计（`<5ms` 输出可定位性 $\kappa$ 与退化主轴）+ 稀疏自由节点 36 束 SDF 球面追踪旋转不变指纹哈希（`<15ms` 索引自相似孪生区），总计 `<20ms`。

---

## 二、独立核查新发现的 3 个源码级隐患

1. `ComposedMatcher` 在严格退化走廊存在 `179.2mm` 纵向乱跳（见 [benchmark_results_x86.txt](benchmark_results_x86.txt) E5，`field_gn` 仅 `13.6mm`）：因 `CorrelativeMatcher` 无先验破平局（纯 ML），在走廊平底谷内跳至搜窗边缘并连累 `MAHAL_GATE` 整帧拒掉精准横向观测。
2. [localizer_node.cpp](../src/localizer_node.cpp) 的外部假设话题 `pending_hypotheses_` 在 `TRACKING/DEGRADED` 态下从不送入 `core_->add_hypotheses()`，3 帧后被静默清空。
3. [localizer_core.cpp](../src/localizer_core.cpp) 的 `hypotheses_` 队列缺乏低分僵尸假设淘汰机制，占满 `max_hypotheses` 后阻塞新候选。

---

## 三、三层闭环泛化架构设计

1. **`PlatformSpec`**（物理标定层：测距尺度、帧周期、时序抖动、里程计滑移率）
2. **`LocalizerCore`**（通用几何与概率定位核：去畸变 + 法向均衡 $n_{\text{eff}}$ + MAP 破平局匹配 + 四态波束模型 + 可观子空间 EKF + 运动门控 MHT）
3. **`MapPrior`**（`<20ms` 自动地图先验层：SDF 结构张量可定位性/退化轴 + 旋转不变孪生指纹 + 硬件 `RuntimeBudget` 算力预算）

---

## 四、四阶段工程落地路线（Step 1 ~ Step 4）

- **Step 1**：收拢 `LocalizerCore` 多峰/可见性/外部假设/淘汰闭环 + 空间分桶 NMS + 搭建 6 场景泛化回归矩阵（仓库、长走廊、对称孪生房、半建图外溢走廊、建图残影+高动态人群、开阔大厅），同一套默认参数全通过。
- **Step 2**：法向空间均衡加权 + `CorrelativeMatcher`/`FieldGnMatcher` 弱先验正则 + 可观子空间 EKF 更新与软门控（移除 `cond > 6` 与 `P *= 2.5`）+ 激光运动去畸变与 $Q_{\text{sync}}$。
- **Step 3**：SDF 球面追踪四态物理波束模型 + 运动门控序贯贝叶斯多假设晋升（移除帧数/分差经验常数）。
- **Step 4**：`<20ms` `MapPrior` 自动地图先验加速 + $\sigma/r_{p90}$ 自适应粗搜角步长 + 协方差驱动自适应跟踪搜窗。
