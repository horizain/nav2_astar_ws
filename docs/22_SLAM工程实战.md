# 22 · SLAM 工程实战——参数、标定与抗干扰

> **本篇回答的问题**：原理懂了（[doc 21](21_SLAM建图原理零基础课.md)），
> 但工程上：slam_toolbox 几十个参数各管什么？传感器装歪了/轮子磨损了
> 怎么标定？玻璃、镜子、黑海绵、走动的人这些"激光杀手"怎么对付？
> 建出来的地图重影/双墙/漂移怎么诊断？
>
> **本篇口径**：所有参数取自本机 `/opt/ros/jazzy/share/slam_toolbox/config/
> mapper_params_online_async.yaml`（doc 20 建图会话用的就是这套）；
> 标定与抗干扰的案例来自本系列实测（doc 17 的外参、doc 20 的故障）。
>
> **前置**：[doc 20 管线实测](20_定位与建图SLAM.md)、
> [doc 21 原理](21_SLAM建图原理零基础课.md)。

---

## 0. 三分钟版

```text
SLAM 工程三板斧：
① 参数     = 把 doc 21 的数学旋钮对准你的机器人（关键帧/搜索窗/回环阈值）
② 标定     = 让"传感器说的"和"真实几何"一致（雷达外参/里程计比例/时间同步）
③ 抗干扰   = 知道激光的物理弱点（玻璃/黑物/动态物），用架构兜住
```

一句话记住：**建图质量的上限由传感器与标定决定，下限由参数决定；
参数救不了标定错误。**

---

## 1. slam_toolbox 参数逐条（结合本机配置原文）

本机默认配置分五组讲。每条给：默认值 → 语义（对应 doc 21 哪个数学零件）
→ 动它会怎样。

### 1.1 关键帧组（哪些扫描进入位姿图）

```yaml
minimum_travel_distance: 0.5     # 每移动 0.5m 才取一个关键帧
minimum_travel_heading: 0.5      # 或每转 0.5rad（≈29°）
scan_buffer_size: 10             # 链式匹配时保留最近 10 帧
scan_buffer_maximum_scan_distance: 10.0
```

- **语义**：doc 21 §7 的"节点"不是每帧激光，而是关键帧——
  走 0.5m 或转 29° 才落一个节点。这是**内存与精度的总权衡**：
  调小 → 图密 → 精但重；调大 → 图疏 → 轻但快/急转弯可能跟丢。
- **本机建议**：TB3 尺寸 + 0.05 分辨率下 0.5m/0.5rad 合理；
  精细建图（建 API 测试地图）可降到 0.25。

### 1.2 扫描匹配组（前端，doc 21 §6 的搜索窗口）

```yaml
correlation_search_space_dimension: 0.5     # 搜索窗口 0.5m×0.5m
correlation_search_space_resolution: 0.01   # 窗口内步长 1cm
correlation_search_space_smear_deviation: 0.1  # 代价场高斯模糊 σ=0.1m
max_laser_range: 20.0                        # 建图用到的最大激光距离
min_laser_range: 0.0
```

- **语义**：搜索窗口 = "允许扫描匹配纠正里程计多大偏差"。
  0.5m 窗口 + 1cm 步长 = 50×50 个平移候选 × 角度候选
  （`coarse_search_angle_offset: 0.349` ≈ ±20°），每个候选做一次
  地图代价求和（doc 21 §6.2 手算的那个操作）。
- **动它会怎样**：窗口调大 → 能纠正更大漂移，但慢 + 更容易配错；
  步长调小 → 亚格精度更好，慢一倍。
- **`smear_deviation` 是个聪明设计**：把地图代价场做高斯模糊，
  让"差 2cm 的对齐"和"完美对齐"得分连续——否则代价场是阶梯的，
  梯度信息全无，粗搜无法逼近。
- **本机建议**：默认即可。里程计很差（打滑严重）才加大窗口。

### 1.3 回环组（doc 21 §7 的回环边从哪来）

```yaml
do_loop_closing: true
loop_search_maximum_distance: 3.0      # 只在 3m 内找回环候选
loop_match_minimum_chain_size: 10      # 候选子图至少 10 帧
loop_match_minimum_response_coarse: 0.35   # 粗匹配响应 ≥0.35 才算数
loop_match_minimum_response_fine: 0.45     # 精匹配 ≥0.45 才建回环边
loop_search_space_dimension: 8.0       # 回环搜索窗 8m（大！）
loop_search_space_resolution: 0.05
distance_variance_penalty: 0.5         # doc 21 §7.2 的 σ 来源之一
angle_variance_penalty: 1.0
```

- **语义**：回环两道门——第一道"位姿距离 <3m"（§7.3 朴素法），
  第二道"扫描匹配响应 ≥0.35/0.45"（长得像才算）。响应 = 匹配得分
  归一化值，0~1。
- **动它会怎样**：
  - `response_coarse/fine` 调低 → 更多回环边 → 纠偏更积极，
    但**误回环会把两张相似的地方焊在一起**（地图折叠，最恶性故障）；
  - 调高 → 回环更保守 → 漂移留得多（地图"开口"）。
- **本机建议**：相似结构多的环境（重复走廊）**调高**；特征丰富
  环境保持默认。建图时如果发现绕回起点后地图"没合上"，
  先查这两个值（也查 `loop_match_minimum_chain_size`——10 帧的
  子图在 0.5m 关键帧下 = 至少走过 5m 才有资格回环，短程绕圈
  会等不到）。

### 1.4 栅格输出组（你眼睛看到的那张图）

```yaml
resolution: 0.05              # 5cm/格
map_update_interval: 5.0      # 每 5s 重发布一次 /map
min_pass_through: 2           # 格子被穿过 ≥2 次才算"已知"
occupancy_threshold: 0.1      # log-odds 判据
```

- `map_update_interval` 就是 doc 20 里"等 /map 5 秒"的出处；
  调小 → RViz 图更流畅、slam_toolbox 更忙。
- `min_pass_through: 2`：一个格子至少被激光穿过 2 次才标记为自由
  ——抗单次噪声（doc 21 §3.3 的"证据对冲"在输出端的体现）。

### 1.5 系统组

```yaml
mode: mapping                    # mapping / localization / lifelong ...
transform_publish_period: 0.02   # map→odom 发布周期 50Hz
transform_timeout: 0.2
tf_buffer_duration: 30.0
stack_size_to_use: 40000000      # 40MB 栈（序列化大图用）
```

- `mode: localization` = 载入已有地图只定位不扩建（对应 AMCL 的角色，
  但基于扫描匹配——doc 20 后续可做里"slam_toolbox localization vs
  AMCL 对比"就用它）。
- `transform_publish_period: 0.02` = TF 50Hz——比 scan 5Hz 高得多，
  因为 TF 消费者（nav2）跑 20Hz，TF 必须比它们密。

---

## 2. 传感器标定三课（"传感器说的"必须等于"真实几何"）

### 2.1 雷达外参标定（本机踩过：doc 17 的 −0.064m）

**问题**：CBF/SLAM 都假设激光器在机器人中心；TB3 waffle 的雷达
实际在中心后方 6.4cm（`gz_waffle.sdf.xacro` 的 lidar_joint：
`-0.064 0 0.121`）。不补偿的后果（doc 17 实录）：侧向束被误判
侵入屏障，CBF 假死。

**工程标定法（无 CAD 图纸时）**：

```text
转圈拟合法：
① 机器人原地匀速转 360°，对某根固定障碍柱记录 36 个 (ψ_i, range_i)
② 若雷达在中心，该柱在雷达系的轨迹应是完美圆
③ 实际轨迹是个偏心圆：拟合圆心 (a, b) → 雷达相对转轴的偏移量
④ a/b 填进外参（base←雷达 的平移），全部距离换算到中心系
```

**有模型时**：直接读 URDF/SDF 的 joint origin（本机就是这么干的）。

**验证**：doc 17 的回归测试——补偿前 CBF 拦截 413 次（几乎全误拦），
补偿后 16 次（全真拦）。

### 2.2 里程计标定（两项系统性误差，各配手算）

里程计有两个**可标定的系统误差**（随机打滑标不掉，但系统偏差必须标）：

**① 平移比例（轮径误差）**：

```text
实验：命令直线走 5.0m，卷尺量实际位移 4.85m
误差系数 = 4.85/5.0 = 0.97
修正：轮径 × 0.97（或里程计输出 × 1/0.97）
```

**② 转向比例（轮距误差）**：

```text
实验：命令原地转 360°，陀螺仪/量角器实测转了 367°
轮距误差 = 367/360 = 1.0194 → 实际轮距比标称宽 1.94%
修正：wheel_separation × 1.0194
```

> 为什么是轮距：差速转向角速度 ω = (v_r − v_l)/b，b（轮距）偏小
> 会放大 ω。轮径误差影响 v，轮距误差影响 ω——两个实验分别激励
> 两者，这就是标定的"可观测性设计"。

标定后的效果直接进入 SLAM：里程计是扫描匹配的**初值**（doc 21 §6.2
的先验项）——初值越准，搜索窗可以越小、匹配越快越稳。

### 2.3 时间同步与运动畸变（de-skew）

**问题**：一帧激光 360 个点不是同时打的——gz 里也是按角度顺序
扫描的。扫描期间机器人在动，帧内前后点的"测量时刻"差一个扫描周期。

**手算本机畸变量**：TB3 雷达 5Hz → 一帧扫 200ms；机器人 0.2 m/s：

```text
帧首到帧尾机器人移动 0.2 × 0.2 = 4cm
→ 激光帧"自我扭曲"最多 4cm（≈一个栅格！）
```

**后果**：快速运动时墙变成"斜的"、圆柱变成"拖尾的椭圆"。
**对策**：
1. `use_sim_time` + 桥接时间戳正确（仿真里天然对齐；真机要配 NTP/软时钟）
2. de-skew：用里程计把每个点反推回"统一时刻"的位置再建图
   （slam_toolbox 内部有此处理；真机高频里程计是前提）
3. 建图时减速（0.2 m/s 本来就是慢速建图姿势——4cm 畸变可接受）

---

## 3. 抗干扰手册：激光的六个物理弱点与对策

| 干扰 | 现象 | 根因 | 对策 |
|---|---|---|---|
| **玻璃/镜面** | 地图上出现"穿墙点"（玻璃后的物体被画进来）或凭空短束 | 激光穿透玻璃 / 镜面反射走另一条路 | ① 标记玻璃区不建图 ② 用强度通道（穿透回波强度异常低）③ 红外补盲 |
| **黑色吸光物** | 黑色海绵/绒布"隐身"——激光全被吸收，地图上没有它 | 漫反射率太低，回波弱于检测阈 | ① 与其它传感器融合（深度相机/超声波）② 已知静态障碍写进 static layer ③ 别用纯激光做唯一防撞 |
| **动态物体（人/宠物）** | 地图上残留"幽灵墙" | 被占据的格子需要被"自由观测"对冲（doc 21 §3.3），走过的人只留下一瞬占据 | ① `min_pass_through` 提高对冲门槛 ② 建图选无人时段 ③ raytrace clearing 保持开启（默认开）④ 动态物过滤层（scan 的聚类剔除） |
| **多路径反射** | 镜面/光滑地面造成"凭空的远距点" | 回波走了折射路径才回來 | 强度过滤 + 与历史地图比对剔除离群点 |
| **灰尘/污渍** | 雷达窗口脏了 → 全局距离偏差/丢点 | 发射窗透光率下降 | 定期擦窗口；在线健康检查（对已知墙的读数统计漂移即报警） |
| **斜面掠射** | 打到 45° 光滑面的束丢失或回鬼点 | 反射方向背离接收器 | 点级丢弃（返回无效）——slam_toolbox 的 `min/max_laser_range` 与无效值过滤已兜住 |

**架构级对策（本仓库已具备）**：
1. **证据对冲**（doc 21 §3.3）：log-odds 让一瞬干扰被后续观测洗掉——
   动态物体"幽灵墙"的数学解
2. **双层防线**：SLAM 地图（全局规划用）+ 实时激光防撞
   （collision_monitor/CBF，doc 17）——地图脏了不影响即时安全
3. **保守阈值**：inflation 0.70m 的膨胀场本身就是对"地图有 1-2 格误差"
   的裕度设计

---

## 4. 故障速查表（症状 → 根因 → 处理）

| 症状 | 最可能根因 | 处理 |
|---|---|---|
| **地图出现"双墙"**（一道墙两条平行线，间距 5-15cm） | 里程计比例误差 → 来回经过的同一面墙画在两处 | §2.2 里程计标定；或提高扫描匹配搜索窗（`correlation_search_space_dimension`） |
| **地图重影/错位重叠** | 时间戳不同步 → 扫描与 TF 对不齐 | §2.3 时间同步；检查 `use_sim_time`；真机查时钟 |
| **墙变圆弧/斜线** | 转向时运动畸变（帧内扭曲） | §2.3 de-skew；建图减速 |
| **绕回起点地图"合不上"（开口）** | 回环没触发或响应阈值过高 | 查 `loop_match_minimum_response_*` 与 `loop_match_minimum_chain_size`；确认绕行距离 > chain_size×关键帧距 |
| **地图"折叠"（两处相似的地方焊在一起）** | **误回环**——最恶性故障 | 调高 response 阈值；缩小 `loop_search_maximum_distance`；增加环境辨识度 |
| **TF"map"帧时有时无** | 两个 map→odom 发布者打架 | `ros2 topic info /map` 数发布者；清理孤儿进程（doc 20 第二课，清理清单含 `[s]lam_toolbox`） |
| **建图时 RViz 地图不更新** | `map_update_interval` 太长 / slam 未 active | 查 lifecycle 状态与 `map_update_interval: 5.0` |
| **换了个世界/环境地图全崩** | 过拟合旧环境 / 特征缺失（大空旷白墙房间，激光特征贫乏） | 重新建图；空旷环境加特征物；lifelong 模式 |
| **导航时"costmap 跟不上 SLAM 地图"** | SLAM 每次扩张地图，静态层重置 | 正常现象（doc 20 建图会话日志里的 Resizing 记录） |

---

## 5. 复现与练习

### 复现（与 doc 20 §5 相同的管线）

```bash
# 建图会话（默认参数即本篇讲解的那套）
ros2 launch nav2_controller_lab controller_lab_launch.py \
  slam:=True headless:=True use_rviz:=False
python3 docs/tools/auto_explorer.py --duration 150
python3 docs/tools/save_slam_map.py --out docs/tools/maps/lab_built

# 对照实验：改参数（symlink-install 下改完即生效，doc 18 技巧 6）
# 例如把 minimum_travel_distance 0.5 → 0.25，对比地图密度与建图时长
# 参数背后的数学在 doc 21：先跑 python3 docs/tools/slam_playground.py
# --demo 6 理解"σ=信任度"，再动 variance_penalty 类参数
```

### 练习 1（参数推理）

在 0.5m 关键帧间距下，`loop_match_minimum_chain_size: 10` 意味着
子图至少覆盖多少米才有资格参与回环？一个 5m 宽的小房间绕一圈，
这个条件满足吗？

<details><summary>答案</summary>

10 帧 × 0.5m = **至少 5m 轨迹**。5m 宽房间绕一圈 ≈ 15-20m 轨迹 →
满足。但若把关键帧间距调到 1.0m，10 帧 = 10m——小房间绕两圈才够。
**结论：回环相关参数是联动的，单独调一个会踩别的坑。**
</details>

### 练习 2（畸变估算）

建图速度提到 0.5 m/s，雷达 5Hz（帧周期 200ms），帧内最大畸变多少？
相对 0.05m 栅格是什么概念？

<details><summary>答案</summary>

0.5 × 0.2 = **0.10m = 10cm = 两个栅格**。地图上墙会被"抹斜"2 格。
对策：降速到 0.2（4cm）或启用 de-skew。**这就是"建图要慢"的
定量依据**——不是经验主义，是帧周期 × 速度的算术。
</details>

---

## 6. 本机案例索引（标定与抗干扰的实战场）

| 案例 | 教训 | 出处 |
|---|---|---|
| 雷达外参 −0.064m 未补偿 → CBF 假死 | 外参是安全层第一课 | doc 17 §6 |
| 双 slam_toolbox 孤儿 → TF 精神分裂 | 帧唯一发布者铁律 | doc 20 §4 |
| 探索器旋转死锁 | 状态机免疫设计 | doc 20 §3.2 |
| map_saver 订阅超时 | 读懂格式自己写 | doc 20 §3.3 |
| 里程计 11s 漂 3cm | 里程计是起点不是答案 | doc 17 探针 |
| costmap 20Hz 调优 | 与控制器同频（doc 18） | doc 18 §1 |

---

## 7. 参考文献

- [slam_toolbox 官方参数文档（Humble）](https://docs.ros.org/en/humble/p/slam_toolbox)
- [slam_toolbox 默认配置（GitHub）](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)
- [fixtherobot.dev: SLAM Toolbox and Mapping Workflows](https://fixtherobot.dev)（关键帧/搜索空间的工程解读）
- [回环调参实战（robotics.stackexchange #111343）](https://robotics.stackexchange.com/questions/111343/which-parameters-should-i-modify-in-order-to-get-a-loopclouse-using-slam-toolbox)
- [Occupancy Grid Mapping 算法讲解（Medium）](https://medium.com/@SuriNaren/occupancy-grid-mapping-algorithm-e451701da0e8)
- [ECE 276A: Particle Filter SLAM 讲义](https://natanaso.github.io/ece276a2020/ref/ECE276A_9_ParticleFilterSLAM.pdf)
- Thrun, Burgard, Fox,《Probabilistic Robotics》（传感器模型/标定的标准口径）
