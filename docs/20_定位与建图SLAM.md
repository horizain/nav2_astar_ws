# 20 · 定位与建图（SLAM）——技术演进与实车管线

> **本篇回答的问题**：机器人怎么知道"我在哪"（定位）和"世界长什么样"（建图）？
> 这两件事为什么是一体两面？技术二十年来走到了哪一步？——以及在本实验室
> 用 slam_toolbox **从零建出一张地图、再用它完成导航**的完整实测。
>
> **本篇实测成果**：
> - slam_toolbox 在线建图：探索器 9.6m / 150s → 自建地图 112×105 格（九圆柱
>   环全部成形，与官方地图结构一致）
> - 用**自建地图**替换官方地图后，AMCL 定位 + DWB 导航 **11.4s PASS**
> - 完整闭环：建图 → 存图 → 定位 → 导航，全部命令可复现
>
> **前置**：[15 链路实战](15_端到端链路实战.md)（速度链路）、
> [01 教程](01_教程_从零复现A星规划器.md)（AMCL 初始位姿的坑）。
> **配套**：[21 原理零基础课](21_SLAM建图原理零基础课.md)（本文管线的数学，
> 每个算法手算）、[22 工程实战](22_SLAM工程实战.md)（参数/标定/抗干扰）。

---

## 0. 三分钟版

```text
建图（SLAM）  = 边走边画地图     同时估计"我在哪"+"墙在哪"（鸡生蛋问题）
定位          = 拿着已知地图回答"我在哪"（本仓库一直用的 AMCL）
里程计        = 靠轮子积分推算位置（必然漂移，5% 级误差）——一切的下界

本篇闭环：slam_toolbox 在线建图（探索器 9.6m）→ 存图 112×105
        → map:= 自建图 → AMCL 定位 → DWB 导航 11.4s PASS
```

一句话记住：**SLAM 是"边画地图边在地图上找到自己"的鸡生蛋问题；
所有现代解法都归结为"图优化"——把每一帧观测当作图的一条边，
全局求解让所有边尽量自洽。**

---

## 1. 三个概念与一张合同

| 概念 | 输入 | 输出 | 本仓库对应 |
|---|---|---|---|
| **里程计** odometry | 轮速积分 | odom→base_link（连续但漂移） | gz DiffDrive 插件 |
| **建图** SLAM | 里程计 + 激光 | map→odom **和** 地图本身 | slam_toolbox（本篇） |
| **定位** localization | 里程计 + 激光 + **已知地图** | map→odom | AMCL（doc 01 起） |

关键认知：**地图是一份"坐标系的合同"**。所有规划、代价地图、目标点
都写在 map 系里；谁来发布 map→odom，谁就定义了"地图坐标系是什么"。
- 模块化栈里 AMCL 发布它（用官方地图）
- SLAM 模式里 slam_toolbox 发布它（边建边发）
- **两者绝不能同时跑**——两个 map→odom 发布者会让 TF 精神分裂
  （§4 第二课的实录）

---

## 2. 技术演进二十年（从滤波到图优化到学习）

### 2.1 滤波时代（2000s）——"用概率硬扛不确定性"

- **EKF-SLAM**（早期）：把路标和位姿全部塞进一个协方差矩阵。路标多了
  O(n²) 爆炸。
- **FastSLAM**（2003）：粒子滤波分解条件独立性，每个粒子带一张地图。
- **Gmapping**（2007）：RBPF + 改进提议分布，ROS1 时代的默认 SLAM。
  已停止维护，Jazzy 无官方包——技术演进第一批被淘汰的"曾经标准"。

### 2.2 图优化时代（2016-）——"全局求解让所有观测自洽"

把 SLAM 表述成**位姿图**：节点 = 关键帧位姿，边 = 扫描匹配/里程计约束，
用非线性最小二乘（Ceres/g2o）全局求解。两次突破塑造了今天的格局：

- **Cartographer**（Google 2016）：submap（子图）+ 分支定位回环检测，
  工程质量极高（本机 `src/cartographer` 有源码与编译产物，2D/3D 通吃，
  但配置复杂——lua + 子图参数一整套）
- **SLAM Toolbox**（Macenski 2019）：专为 ROS2 打造，**在线/离线/定位/
  lifelong（多会话续建）四种模式**，位姿图序列化可存可续。如今
  ROS2 社区的事实标准——**本篇实测用的就是它**（`/opt/ros/jazzy`
  自带，配置里 base_frame/scan_topic 默认值就是给 TB3 写的）

### 2.3 视觉与多传感器（2015-）

ORB-SLAM3（视觉惯性，2021）、VINS-Fusion、RTAB-Map（RGB-D + 回环，
nav2 生态里可当视觉 SLAM 用）。解决"激光看不见玻璃/长走廊"的问题。

### 2.4 学习时代（2023-2026）——"让网络学出几何"

- **DROID-SLAM**（2021）：可微 bundling，GPU 上全局优化
- 学习型里程计/深度估计（如 DROID、MASt3R 类两视图重建）
- 语义 SLAM：地图不再是格子，而是"这是桌子、那是门"的实体层——
  与本系列 doc 19 的 VLA 路线在"语义地图"处汇合
- 社区争论"SLAM 是否已被深度学习解决"——目前共识：**学习组件嵌入
  传统图优化框架**（前端学特征/深度，后端仍是优化）是落地主流

**给 2D 地面机器人的实用答案（2026）**：slam_toolbox（建图）+
AMCL 或 slam_toolbox localization 模式（定位）——本篇用的正是这对。
Cartographer 适合多传感器融合与高精度需求。

---

## 3. 本机管线实测

### 3.1 建图：一个参数切换（官方支持，零自定义 SLAM 配置）

nav2 bringup 自带 `slam:=True` 开关——slam_launch.py 用 slam_toolbox
的 **online_sync** 模式**替换** map_server + amcl（lifecycle 组也从
"定位组"换成"slam 组"），导航栈照常运行（用增长中的地图规划）：

```bash
ros2 launch nav2_controller_lab controller_lab_launch.py \
  slam:=True headless:=True use_rviz:=False
# RViz 玩法（强烈推荐，看地图"长"出来）：
#   去掉 headless 与 use_rviz:=False，RViz 里 Map 层会随探索扩张
```

lab launch 的 `slam` 透传与 `map` 透传（本篇新增）见
`src/nav2_controller_lab/launch/controller_lab_launch.py`。

### 3.2 探索器：接触触发转向的随机覆盖（`docs/tools/auto_explorer.py`）

无头建图的关键是"谁来开车"。用 Nav2 探索会先撞上"未知区不可规划"
的死结；用键盘没法自动化。方案是**扫地机器人式状态机**：

```text
DRIVE  前进 0.22 m/s；前方 ±35° 锥内障碍 < 0.35m → TURN
       转向方向自动避开更近的一侧（scan_side = 左近距 − 右近距）
TURN   原地旋转 ±90~170°；期间免疫卡住检测；转完 3s 免疫期
卡住   仅 DRIVE 态且免疫期外：4s 位移 < 5cm → 脱困旋转 180°
环扫   单段直行 > 2.5m → 原地慢转 360°（补全激光视野）
```

**实测**：150s、总里程 9.6m，覆盖整个六边形世界（含九圆柱间隙）。

**第一课（踩坑实录）：状态机的免疫设计**。初版把"旋转"也算进
"4 秒无位移"的卡住判定 → 卡住触发旋转 → 旋转中又"无位移" → 再触发
→ **原地转圈死锁**（100s 只走了 4.3m，位置钉在 (4.33, 0.00)）。
修复：TURN 态免疫卡住检测 + 转完给 3s 免疫期。教训：**任何"检测→
动作"的状态机，动作本身会改变检测条件，必须给免疫期**。

### 3.3 地图文件格式解剖（`docs/tools/save_slam_map.py`）

nav2 地图 = 一张灰度图 + 一个 yaml（坐标系的合同）：

```text
lab_built.pgm   P5 灰度图：254=自由  0=占据  205=未知
lab_built.yaml  resolution: 0.05        # 一格 5cm
                origin: [-0.96, -2.07, 0]  # 图左下角在 map 系的坐标
                occupied_thresh: 0.65   # 占据概率 ≥ 0.65 画成 0
```

两个易踩的坑（都在 `save_slam_map.py` 里处理了）：
1. **y 轴翻转**：OccupancyGrid 行 0 = y 最小端；PGM 行 0 = 图最上端
   （y 最大端）→ 保存时 `np.flipud`
2. **概率 → 灰度**：OccupancyGrid 是 0~100 占据概率（-1 未知），
   PGM 是 0~255 灰度——按 occupied_thresh/free_thresh 两段映射

**实测地图**：112×105 格 @ 0.05m，原点 (-0.96, -2.07)，自由格 7847、
障碍格 742（官方图 870——探索器没走到每一面墙，属正常）。
九个圆柱的环全部成形（预览：`docs/tools/logs/lab_built_map_preview.png`）。

> 顺带一提：`ros2 run nav2_map_server map_saver_cli` 是官方存图工具，
> 但本机实测它对订阅时序敏感（2s 内等不到 /map 就退出）。`save_slam_map.py`
> 用 rclpy 直接抓 /map 更稳，且把格式讲透——**工具失灵时，读懂格式
> 自己写一个只要 40 行**。

### 3.4 定位闭环：用自建图导航

```bash
# 会话 1（建图）：slam:=True + 探索器 → 存图（§3.2-3.3）
# 会话 2（定位+导航）：把自建图传给 bringup
ros2 launch nav2_controller_lab controller_lab_launch.py \
  headless:=True use_rviz:=False \
  map:=$HOME/ros2_jazzy/nav2_astar_ws/docs/tools/maps/lab_built.yaml
# e2e 导航（坐标系约定见下）
python3 docs/tools/e2e_nav_test.py --init 0 0 --goal 3.5 1.0 --controller DWB
```

**坐标系怎么换算**：slam_toolbox 的 map 帧 = 轨迹起点（里程计零点），
出生点 (-2,-0.5) 在里程计里是 (0,0) → **自建图里出生点 ≈ (0,0)**；
世界系目标 (1.5, 0.5) → 自建图 (3.5, 1.0)。

**实测结果**：map_server 收到 `112 X 105 map @ 0.050 m/pix`，AMCL
在自建图上定位，DWB 导航 **11.4s SUCCEEDED**（轨迹与官方图运行一致：
穿中线缝隙到目标）——**自建地图与官方地图在导航里完全等价**。
证据：`docs/tools/logs/e2e_builtmap_success.log`。

---

## 4. 踩坑实录（第二、三课）

### 第二课：两个 SLAM 发布者 = TF 精神分裂

一次会话中断后残留了一个 slam_toolbox 孤儿进程；重新 bringup 时
**两个 map→odom 发布者**互相打架 → `map` 帧时有时无 → 全局代价地图
激活超时 → bringup abort → 后续 `/map` 都对不上。

**修复**：清理清单加入 `[s]lam_toolbox` 模式（`/tmp/nav_cleanup.sh`）；
启动 SLAM 会话后**先验证 `/map` 只有一个发布者**再开探索。

> **教训**：TF 树里每个 frame 只能有一个人发布。凡是"帧时有时无"的
> 症状，第一件事 `ros2 topic info /map`（或 /tf）数发布者数量。

### 第三课：官方工具失灵时，读懂格式自己写

`map_saver_cli` 两次在 2 秒订阅超时后退出（对本机 DDS 时序敏感）。
`save_slam_map.py` 40 行解决——且格式推导本身就是教学价值
（§3.3 的两个坑）。**依赖工具不如理解格式**。

---

## 5. 复现命令全录

```bash
# ===== 会话 1：建图 =====
./清理脚本（doc 15 §7.2，已含 slam_toolbox 模式）
ros2 launch nav2_controller_lab controller_lab_launch.py \
  slam:=True headless:=True use_rviz:=False use_visualizer:=False
# 等 70s（slam_toolbox active + /map 有发布者）：
ros2 topic info /map                      # Publisher count 应为 1
python3 docs/tools/auto_explorer.py --duration 150
python3 docs/tools/save_slam_map.py --out docs/tools/maps/lab_built

# ===== 会话 2：用自建图定位导航 =====
清理 → 启动（map:= 自建图，见 §3.4）
python3 docs/tools/e2e_nav_test.py --init 0 0 --goal 3.5 1.0 --controller DWB
# 期望：[RESULT] ✅ SUCCEEDED，时长 ~11s
```

GUI 玩法（看着地图长大）：会话 1 去掉 `headless` 与 `use_rviz:=False`，
RViz 的 Map 层随探索实时扩张；`live_monitor.py` 的左图还能同时看
机器人轨迹与激光点（coverage 热区一目了然）。

---

## 6. 后续可做

- [ ] **slam_toolbox localization 模式**：把建好的图以 pbstream 形式
      载入（`localization:=True`），对比 AMCL 与 slam_toolbox 定位的
      精度/漂移（有 ground truth：gz odom）
- [ ] **Cartographer 对照实验**：src 里已编译，写 TB3 的 lua 配置跑同
      一航线，对比地图质量与 CPU
- [ ] **探索自动化**：接 explore_lite 或"frontier 检测"（未知-自由边界
      作为导航目标），替代随机游走——预计覆盖时间减半
- [ ] **里程计漂移量化**：长航线下 gz 真值 vs diff odom 误差曲线
      （doc 17 探针已有半成品）
- [ ] **学习型里程计/SLAM 调研落地**：doc 19 路线的感知侧延伸

---

## 附 A：证据与交付物索引

| 文件 | 内容 |
|---|---|
| `docs/tools/maps/lab_built.pgm/.yaml` | **slam_toolbox 自建地图**（112×105 @ 0.05m） |
| `docs/tools/logs/lab_built_map_preview.png` | 自建地图预览（九圆柱环全部成形） |
| `docs/tools/logs/e2e_builtmap_success.log` | 自建图导航 11.4s PASS |
| `docs/tools/auto_explorer.py` | 自动探索状态机（DRIVE/TURN/SWEEP + 免疫设计） |
| `docs/tools/save_slam_map.py` | /map → PGM+YAML（含格式推导） |
| `src/nav2_controller_lab/launch/controller_lab_launch.py` | 新增 slam:/map: 透传 |
| `docs/tools/e2e_nav_test.py` | 新增 --slam 模式（跳过定位组与初始位姿） |
