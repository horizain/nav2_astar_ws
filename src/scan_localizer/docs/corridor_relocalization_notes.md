# 长走廊退化与对称重定位改进说明（通用泛化 vs 工程优化评估）

本文记录 2026-11-26 基于实车（RK3399，`10.6.128.107`，49m × 28m 办公地图、约 36m 直线走廊）实测暴露的问题、针对性修改以及对改动性质（算法通用泛化提升 vs 场景工程优化）的评估，便于后续跨场景演进。

---

## 一、实测暴露的典型问题

1. **仅靠端点似然场导致 180° 走廊翻转得分虚高（`score ≈ 0.89`）**  
   位于走廊一端反向面对另一端时，左右侧墙依然能给出高端点似然分；少量射向走廊开口外侧（穿透端墙进入 `-1` 未建图灰区）的光束在纯端点似然场下不扣分。
2. **走廊内散落静态残影/桌椅触发物理一致性误杀**  
   初版射线可见性检查把擦过廊内 ~0.25–0.5m 障碍块后继续命中远端地图墙的光束按整段剩余距离计惩罚，导致正常沿走廊行驶（`FieldGnMatcher` 原始 `score = 0.81..0.92`、`inlier = 0.75..0.92`）时终分跌至 `0.10..0.44` 并频繁失锁重搜。
3. **长对称走廊横向/纵向退化耦合与次优局部极小**  
   - 纵向缺乏约束时 `FieldGnMatcher` Hessian 条件数失衡；转向叠加传感器/里程计时延极易误触 `MAHAL_GATE`。  
   - 对称候选与刚被连续否定的失效锚点共存时，自动恢复容易快速抢跑或被旧 `hasAnchor` 拉回错位点；落入 70~80 分次优局部解后缺乏后台逃逸机制；`global_correlative` 默认仅取 `top-12` 易被走廊某 6m 区段相邻重复峰占满。

---

## 二、改动性质评估：通用泛化提升 vs 场景工程优化

总体来看，约 **70% 属于定位算法通用泛化能力的补齐**（修正原有似然场和状态机的理论/逻辑漏洞），约 **30% 属于针对当前长走廊结构、动态杂物特征及嵌入式算力预算的工程调参**。

### 1. 属于“算法通用泛化能力提升”的改动

| 模块 | 改动与符号 | 通用意义 |
|---|---|---|
| 退化感知协方差 | `src/matchers.cpp` 中 `cov_from_hessian` 针对 `Hxy` 高条件数特征分解抬升弱轴方差下界 | 解决所有一维约束退化场景（走廊、单边墙、隧道）过自信与 `MAHAL_GATE` 误拒 |
| 射线可见性一致性 | `include/scan_localizer/matchers.hpp` / `src/matchers.cpp` 新增 `beam_visibility_consistency`；`global_correlative` 在 `level == 0` 跳过 `origin_cell < 0` | 将“端点贴墙”升级为“端点似然 + 射线路经自由/已知空间物理一致性”，任意建图边界与局部地图都能抑制反朝向/区外假峰 |
| 手动/自动恢复隔离与多峰迟滞 | `include/scan_localizer/localizer_core.hpp` / `src/localizer_core.cpp` 中 `request_global(bool manual)`、`hyp_ambiguity_margin`（`0.08`）、`hyp_ambiguous_confirm_frames`（`6`）、连续锚点校验、`last_pose_` 外推 | 区分显式人工重定位与自动异常恢复，避免静止或对称区“2 帧草率跨区跳变” |
| 窗口外健康匹配与失效锚点清理 | `scan_localizer_sdp_bridge/src/bridge.cpp` 中 `WINDOW_OUT` 软计数递增滑窗、`superiorJumpStreak`、`probeSuperiorMode` 与 `rejected >= hardLimit` 清空 `hasAnchor` | 通用修复两类状态机死锁：短时滑移误记失锁，以及跌入次优局部解后无法逃逸或被失效锚点反拉 |

### 2. 属于“针对特殊场景 / 硬件平台的工程优化”的部分

| 位置 | 当前实现 / 参数 | 场景假设与后续注意点 |
|---|---|---|
| `src/matchers.cpp` (`beam_visibility_consistency`) | `eff_unk_depth`（`unk_len >= 2.5m && end_unknown` 或 `>= 4.0m`，`min_unk_beams = max(3, used/50)`）与 `wall_pierce_depth`（`occ_len >= 0.55m`，`min_occ_beams = max(12, used/12)`，`occ_ratio >= 0.12`）分离 | 经验阈值围绕“走廊宽约 2m、偶发 0.5m 桌椅/行人遮挡、反朝向长射线深穿未知区 >10m”权衡；若用于极窄隔间或大面积货架搬空的动态仓库，需下调或外置为参数 |
| `scan_localizer_sdp_bridge/src/bridge.cpp` (`searchGlobal`) | `global_correlative` 粗峰配额 `12 -> 24`，精修去重保留最多 `6` 个空间分离假设（`0.55m / 0.30rad`） | 针对 30m+ 长走廊对称峰密集问题并兼顾 RK3399 `~240–450ms` 耗时上限；面向百米级超大厂房时，固定 `top-24` 仍可能不足，后续应改为按区域分散覆盖的 NMS |
| `scan_localizer_sdp_bridge/src/bridge.cpp` (`AdaptiveMatcher`, `coreConfig`) | `odom_trans_var_per_rad = 1.5e-2`、`odom_rot_var_per_rad = 1.5e-2`、GN 协方差最小下限 `(6.25e-4, 6.25e-4, 4.0e-4)`、`winLin >= 0.25m, winAng >= 0.20rad` | 针对当前底盘 ~10Hz 激光/里程计时序滞后与转向侧偏的标定 |

---

## 三、验证记录

在最终代码修改后完成以下验证：

1. **x86_64 单元测试**：
   - `scan_localizer` 非 ROS 单测（`ctest -R "^(test_se2|test_map_io|test_map_field|test_matchers|test_particle_filter|test_localizer_core|test_sim_world|test_benchmark_regression)$"`）：**8/8 通过**（含 `BeamVisibilityPenalizesOutOfMapAndWallPiercingCorridorFlip`、`AutoGlobalContinuityAndSymmetryAmbiguityGuard`、`CorridorLongitudinalLagRelaxesMahalGate`；`localizer_node_component` 因当前构建容器为 ROS Humble 未编入此轮检查）。
   - `scan_localizer_sdp_bridge`：`bridge` 与 `bridge_reference` **2/2 通过**。
2. **RK3399 目标板（`10.6.128.107`）验证与实车联调**：
   - 板端运行交叉编译产物 `test_bridge`：**25/25** 项匹配与生命周期检查全部通过（`auto recovery ms: cold=82.6286, lost=523.511, spam_req=508.962, occluded=591.754`，`fast rotation: 20/20 fresh matches`）。
   - 更新 `/usr/lib/aarch64-linux-gnu/libscan_localizer_sdp_bridge.so.1` 后实车运行验证：`15:40:03` 触发全局搜索后 `1063ms` 内完成多假设晋升（`GLOBAL->TRACKING score=0.96 inlier=0.95`），随后连续穿越此前易误拒走廊段（`-23.1m -> -13.3m`），连续窗口汇总 `acc=33/33`、`39/40`、`36/36`，均值匹配分 `0.92..0.93`，内点均值 `0.90..0.93`，实时 `/api/core/slam/v1/localization/quality` 为 `93`。

---

## 四、后续可演进方向

- 将 `beam_visibility_consistency` 的未知外溢与障碍穿行比例阈值抽离为可配置参数，方便在极小房间与高动态仓储场景间切换。
- 为 `global_correlative` 增加基于网格分桶的强制空间多样性保留，从算法层解决超大规模对称地图 `top-K` 被单一局域占满的问题。
