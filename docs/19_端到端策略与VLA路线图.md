# 19 · 端到端策略与 VLA 路线图——从行为克隆到 VLA 的完整地图（规划篇）

> **状态声明（先读这个）**：本篇是**路线图与设计文档**，不是实施记录。
> 文中"阶段 1-5"的代码**尚未编写**（⏳ 标记），但每一阶段的设计、公式、
> 数据格式、验收标准已经落到可以直接开工的粒度。等环境或时间到位，
> 按图施工即可——就像 doc 01 是 A* 的施工图一样，本篇是端到端策略的施工图。
>
> **为什么现在写**：doc 15-18 已经把"经典管线"（A* + 六控制器 + CBF）
> 跑通到厘米级可复现，学习方法的全部基础设施——专家、评估 harness、
> 安全层、插件机制——都已经就位。端到端不是另起炉灶，是在这套地基上
> 盖新楼层。
>
> **前置阅读**：[16 评分函数设计](16_局部轨迹评分函数设计.md)、
> [17 CBF 安全过滤器](17_CBF安全过滤器.md)、[18 工程优化](18_工程优化与实战技巧.md)。

---

## 0. 三分钟版

"用学习模型做规划与控制"是**三个不同档位**的工程，门槛差三个数量级：

| 档位 | 输入 → 输出 | 代表 | 门槛 | 本实验室 |
|---|---|---|---|---|
| **A. 端到端局部策略** | 激光 360 维 + 目标相对位姿 → (v, ω) | 行为克隆、NavDP | ★ 纯 numpy 可复刻 | **doc 19 阶段 1-4 ⏳** |
| **B. 端到端视觉导航** | 相机图像 + 目标图像/文本 → 动作 | ViNT、NoMaD | ★★ 需 GPU + 图像数据 | 远期 |
| **C. VLA 大模型** | 图像 + 语言指令 → 动作序列 | RT-2、OpenVLA、π0 | ★★★★ 从零需 GPU 集群 | **路线：微调开源 VLA** |

核心判断（本篇论证）：**先做 A**。A 把 VLA 的全部核心环节——数据采集、
训练、部署、评估、安全兜底——在能跑通的环境里走一遍；之后上 VLA
只是"换数据集和模型"，管线完全复用。

---

## 1. 为什么是现在：地基已经全部就位

端到端策略需要的每一样东西，前面 18 篇已经建好：

| 端到端需要什么 | 本仓库现成的 | 出处 |
|---|---|---|
| 专家控制器（采集示范用） | DWB（10s 到达）、MPPI（保守绕行） | doc 14-16 |
| 数据采集航线 | e2e_nav_test.py：一条命令"初始位姿→激活→发目标→判结果" | doc 15 |
| 部署机制 | pluginlib 四件套：第七种控制器插件的全部模板 | doc 06/11 |
| 评估 harness | 固定地图/起点/目标的自动回归 + 时长/拦截统计 | doc 15/18 |
| 安全兜底 | CBF 过滤器 + collision_monitor（学习策略无碰撞保证，必须外挂） | doc 17 |
| 调参方法论 | 离线复算 + 单次对比的陷阱意识 | doc 16/18 |

**唯一缺的是模型与训练循环**——而这恰好可以不依赖 torch：
本机 numpy 2.5.1 ✓、torch ✗、无 sudo（32 核 CPU / 31G 内存）。
小规模 BC（行为克隆）用纯 numpy 手写训练循环完全可行，
而且呼应 doc 04"从零手算 A*"的教学风格：**手动反向传播是理解
深度学习的 A\* 星搜索**。

---

## 2. VLA 构建的真实管线（档位 C，先看全局）

### 2.1 架构：三段式 + 两条动作解码路线（2025-2026 分水岭）

```text
图像 ─→ 视觉编码器（ViT / SigLIP / DINOv2）──┐
                                              ├─→ 融合主干 ─→ 动作解码头 ─→ 动作
语言指令 ─→ 语言主干（预训练 LLM）──────────┘
```

动作解码的两条路线（选型时最重要的一刀）：

| 路线 | 代表 | 动作形态 | 优劣 |
|---|---|---|---|
| **离散 token** | OpenVLA（7B） | 动作离散成 token，LLM 自回归输出 | 实现简单、复用 LLM 生态；控制频率低（3-10Hz）、量化误差 |
| **连续流** | π0（flow matching）、π0.5 | VLM 主干 + 独立 action expert 生成**连续动作块** | 高频平滑、适合精细控制；实现复杂 |

### 2.2 数据是护城河（这就是为什么个人从零不现实）

- OpenVLA：Open X-Embodiment 数据集 **97 万条真机演示**，29 个任务
- π0：同量级跨本体数据 + 内部遥操
- 结论：从零预训练需要 GPU 集群 + 数月数据工程，**个人/实验室的正确路线
  是"微调开源 VLA"**：OpenVLA 全文支持 LoRA，24G 显存单卡即可微调；
  π0 开源了权重与微调代码。

### 2.3 训练范式（三段）

```text
① 预训练：互联网图文对齐（VLM 骨干，开源模型已含）
② 机器人数据微调：LoRA 注入演示数据（含语言标注）
③ 动作头训练/校准：匹配目标机器人的动作空间与控制频率
```

### 2.4 部署的真相：低频意图 + 高频兜底

VLA 推理频率 3-10Hz，且**没有任何碰撞保证**。2025-2026 的标准架构
（也是本仓库已经具备的）：

```text
VLA（3-10Hz，语言→子目标/意图）
  └─→ 经典局部控制器或端到端策略（20Hz，跟踪执行）
        └─→ CBF 安全过滤器（100Hz 级，硬屏障）   ← doc 17
              └─→ collision_monitor（最后防线）    ← doc 17
```

**本仓库的 CBF 层在这里的意义**：它让"不可证明安全的神经网络输出"
变得可部署——这正是 doc 15 技术代际图里"学习+安全"合流的落点。

---

## 3. 端到端导航模型构建管线（档位 A/B，NavDP/NoMaD 路线）

NavDP（ICRA 2026，纯仿真训练零样本上车）与 NoMaD（CVPR 2025 基线）
验证过的五步管线：

```text
① 专家采集    经典控制器跑 N 条航线，记录 (观测, 动作) 对
② 观测编码    scan 向量化 + 目标转到机器人系（极坐标）
③ 行为克隆    监督学习 观测→专家动作（或扩散策略头）
④ 封装部署    包成 nav2_core::Controller 插件，运行时切换上场
⑤ 评估对比    同一 harness：成功率/时长/最小距离/拦截统计
```

三个真坑（每一条都有成熟对策，阶段设计里会预埋）：

| 坑 | 现象 | 对策（预埋在哪） |
|---|---|---|
| **分布偏移** | BC 只见过专家走对的路；自己走歪后没人教 → 误差滚雪球 | DAgger：让专家在**策略自己的轨迹**上重新标注（阶段 5） |
| **安全无保证** | 网络输出没有碰撞证明 | CBF 过滤器外挂（doc 17，已就位） |
| **sim-to-real gap** | 仿真噪声 vs 真实噪声分布不同 | 域随机化 + 传感器噪声注入训练（阶段 5） |
| **过拟合窄世界** | 只在 tb3_sandbox 训练 → 换世界就崩 | 多世界/随机圆柱阵采集（阶段 1 的采集脚本参数化世界） |

---

## 4. 本实验室落地蓝图（阶段 1-5 详细设计）

### 4.0 环境现状盘点（2026-11-14 实测）

```text
numpy 2.5.1  ✓（训练循环用纯 numpy 手写——呼应 doc 04 的"从零手算"风格）
torch / sklearn  ✗（未安装；无 sudo 装不了——不影响阶段 2 的最小版本）
32 核 CPU / 31G 内存   ✓（CPU 训练小 MLP 完全够）
Gazebo headless 5Hz 扫描  ✓（采集管线已验证）
```

### 4.1 阶段 1 ⏳：专家数据采集器

**改动点**：`e2e_nav_test.py` 增加 `--record <目录>` 模式——导航过程中
以 20Hz 记录对齐的 (观测, 动作) 对。

**数据 schema**（每行一条样本，npz/parquet 均可）：

| 字段 | 维度/类型 | 说明 |
|---|---|---|
| `scan` | 360 float32（米，inf→range_max） | 雷达（基准束对齐到 base_link 朝向） |
| `goal_polar` | 2 float32 | 目标在机器人系的 (Δd, Δψ)，弧度 |
| `action` | 2 float32 | 专家输出 (v, ω)——**取 CBF 过滤前的标称值**（不加安全层污染） |
| `meta` | 控制器名/航线号/时间戳 | 分组用（DAgger、按控制器评估） |

**采集矩阵（初始目标）**：3 个世界配置（默认沙盒 ×2 随机圆柱阵）×
2 个专家（DWB / MPPI）× 10 条随机航线 ≈ **6 万对**。
关键设计：**目标点随机化**（不是固定的 (1.5,0.5)）——防止网络学会
"背航线"（doc 16 证明过窄世界的评分器会过拟合）。

**验收**：单条航线采集时间 < 2 分钟；数据对齐误差（scan 与 cmd 的时间戳）< 50ms。

### 4.2 阶段 2 ⏳：纯 numpy 行为克隆（手动反向传播）

**网络**（刻意小，教学优先）：

```text
输入 362（scan 360 + goal_polar 2）
  → Linear(362, 128) + ReLU
  → Linear(128, 128) + ReLU
  → Linear(128, 2)        输出 (v, ω)；v 用 sigmoid×v_max 保证正值域
损失 MSE（预测动作 vs 专家动作），Adam 或手写动量 SGD
```

**训练循环伪代码**（纯 numpy，~200 行，这是阶段 2 的核心交付）：

```python
for epoch in range(E):
    for batch in dataloader:                 # minibatch 256
        h1 = relu(X @ W1 + b1)
        h2 = relu(h1 @ W2 + b2)
        pred = h2 @ W3 + b3
        loss = mse(pred, Y)
        # 手动反向传播（逐层链式法则——doc 04 手算 A* 的深度学习版）
        g = 2*(pred - Y)/batch_size          # dL/dpred
        gW3 = h2.T @ g; gb3 = g.sum(0)
        g2 = (g @ W3.T) * (h2 > 0)           # ReLU 导数
        gW2 = h1.T @ g2; gb2 = g2.sum(0)
        g1 = (g2 @ W2.T) * (h1 > 0)
        gW1 = X.T @ g1; gb1 = g1.sum(0)
        update(W1,b1,W2,b2,W3,b3)            # 动量 SGD
```

**验收**：
1. 验证集 MSE 收敛曲线（TensorBoard 没有就用 matplotlib 存图）
2. **预测 vs 专家的 (v,ω) 散点对角图**——偏离对角 = 学到的东西不对
3. 离线"影子模式"评估：网络在专家轨迹上重放，计算动作误差分布

**预估规模**：6 万样本 × 362 维 → 一个 epoch CPU 上 < 10 秒；
100 epoch 训练 < 20 分钟。**纯 CPU 完全够。**

### 4.3 阶段 3 ⏳：E2EController——第七种控制器插件

完全复用 doc 06/11 的 pluginlib 四件套，新增：

```text
include/nav2_controller_lab/e2e_core.hpp     权重加载 + 前向推理（一次矩阵乘）
include/nav2_controller_lab/e2e_controller.hpp
src/e2e_core.cpp / src/e2e_controller.cpp
controller_plugins.xml   注册 nav2_controller_lab::E2EController
params yaml              E2E: 块（model_path, scan_downsample, goal 处理）
```

设计要点：
- 推理 = 两次矩阵乘 + ReLU，**微秒级**，20Hz 毫无压力（doc 18 调优后余量充足）
- 权重文件从 install/share 读取（`colcon` install 到 share/e2e_model.npz）
- **runInflation 复用 ControllerBase**：终点旋转阶段、TF 变换、限幅全部继承
  （这就是把 E2EController 当"第七种"而非独立体系的好处）
- 上链后自动被 CBF + collision_monitor 双层兜底（doc 17 的架构红利）

### 4.4 阶段 4 ⏳：横评实验（本系列的收官之战）

同一 harness、同一航线矩阵，四种控制器对比：

| 指标 | DWB（基线） | MPPI（基线） | E2E-BC | E2E-BC + CBF |
|---|---|---|---|---|
| 成功率（10 条航线） | 已知 | 已知 | ⏳ | ⏳ |
| 时长中位数 | 10s | 157-162s | ⏳ | ⏳ |
| 最小障碍距离 | — | — | ⏳ | ⏳（应 ≥ d_safe） |
| 拦截/CBF 激活次数 | 16 | — | ⏳ | ⏳ |

**预期教学结论（写在这里，跑完对照）**：BC 策略在**训练分布内**接近专家，
在**分布外**（新起点/新障碍配置）劣化——这就是 doc 16"过拟合窄世界"
的学习版；CBF 打开后的成功率提升 = "学习 + 形式化安全"架构的实证。

### 4.5 阶段 5 ⏳：进阶三连（按价值排序）

1. **DAgger**：E2E 策略跑航线 → 专家在同一轨迹上重新标注 → 混合重训。
   预期：分布外成功率显著提升。这是 BC→RL 之间性价比最高的一步。
2. **扩散策略头（NavDP 缩小版）**：把回归头换成小扩散头（10 步去噪），
   输出多模态动作分布（直行/绕左/绕右），对比单模态 BC 的死板。
3. **语言条件化**：LLM 把自然语言（"去左边柱子后面"）解析成目标点/
   子目标序列，注入 goal_polar——**这就是 VLA 的语言接口在局部层的
   最小等价物**；做完这一步，接 OpenVLA 只剩"换骨干"。

---

## 5. VLA 微调路线（档位 C 的正确姿势，需要 GPU 环境）

当有 24G+ 显存的环境时，从零训练弃置，走微调：

```text
① 选型：OpenVLA（LoRA 微调，生态最全）或 π0（flow matching，高频控制）
② 数据：遥操/仿真演示 → 转 RLDS 格式（Open X-Embodiment 标准）
③ 微调：LoRA rank 32 起，动作 token 化对齐目标本体
④ 部署：量化 INT8（推理 ~10Hz）+ 本仓库 CBF 层兜底 + 局部控制器执行
```

**衔接点**：VLA 输出的不是 cmd_vel 而是"意图/子目标"——把它接到
本仓库的 NavigateToPose 目标点或 BT 的自定义节点上，
下面的六种控制器、CBF、评估 harness 全部原样复用。

---

## 6. 参考文献

- [OpenVLA: An Open-Source Vision-Language-Action Model](https://arxiv.org/abs/2406.09246)（7B、LoRA 微调、Open X-Embodiment 97 万演示）
- [π0: A Vision-Language-Action Flow Model](https://www.physicalintelligence.company/blog/pi0)（flow matching action expert）
- [VLA Models for Robots: RT-2 vs OpenVLA vs π0, Compared (2025-12)](https://robocloud-dashboard.vercel.app/learn/blog/vla-models-robotics-2025)
- [Vision-language-action model — Wikipedia](https://en.wikipedia.org/wiki/Vision%E2%80%93language%E2%80%93action_model)
- NavDP（ICRA 2026，arXiv 2505.08712）：仿真训练端到端导航扩散策略
- NoMaD（CVPR 2025 基线）：目标掩码 Transformer 扩散策略
- DAgger：Ross et al., "A Reduction of Imitation Learning to No-Regret Online Learning"（AISTATS 2011）

---

## 7. 附：启动清单（什么时候想做了，从这里开始）

```bash
# 阶段 1 的前三条命令（预计一个工作日内出数据）
# ① e2e_nav_test.py 加 --record 模式（设计见 §4.1）
# ② 写随机航线生成器（世界/目标点参数化）
# ③ 跑 6 条航线 × 2 专家，检查数据 schema 与对齐误差

# 环境补齐（有 GPU/sudo 时）
pip install torch --index-url https://download.pytorch.org/whl/cpu   # 或 CPU 版训练
sudo apt install ros-jazzy-rmw-cyclonedds-cpp                         # 可选：换 RMW
```

依赖关系：阶段 2 依赖 1；阶段 3 依赖 2；阶段 4 依赖 3；
阶段 5 各项独立。**每一阶段都有独立验收标准，做完即收，不必一口气。**
